#![no_std]

//! Bit-banged async I2C implementation
use core::fmt::Debug;
use core::time::Duration;
use embedded_hal::digital;
use embedded_hal::i2c::{NoAcknowledgeSource, SevenBitAddress};
use embedded_hal_async::{delay::DelayNs, i2c};
use futures_lite::FutureExt;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<P> {
    Address,
    Bus,
    ArbitrationLoss,
    StretchTimeout,
    InvalidParameter,
    Nack(NoAcknowledgeSource),
    Overrun,
    Gpio(P),
    Other,
}

impl<P: Debug> embedded_hal_async::i2c::Error for Error<P> {
    fn kind(&self) -> i2c::ErrorKind {
        match self {
            Error::Bus => i2c::ErrorKind::Bus,
            Error::ArbitrationLoss => i2c::ErrorKind::ArbitrationLoss,
            Error::Nack(source) => i2c::ErrorKind::NoAcknowledge(*source),
            Error::Overrun => i2c::ErrorKind::Overrun,
            Error::Gpio(_) => i2c::ErrorKind::Bus,
            _ => i2c::ErrorKind::Other,
        }
    }
}

impl<P: Debug> core::fmt::Display for Error<P> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{self:?}")
    }
}

impl<P: Debug> core::error::Error for Error<P> {}

impl<P> From<P> for Error<P> {
    fn from(err: P) -> Self {
        Error::Gpio(err)
    }
}
pub trait Pin: digital::InputPin + digital::OutputPin + embedded_hal_async::digital::Wait {}

impl<T: digital::InputPin + digital::OutputPin + embedded_hal_async::digital::Wait> Pin for T {}

/// Transaction operation with an explicit address for each operation.
pub enum Operation<'a> {
    Read(SevenBitAddress, &'a mut [u8]),
    Write(SevenBitAddress, &'a [u8]),
}

struct Waiter {
    period_ns: u32,
    stretch_timeout_ms: u32,
}

impl Waiter {
    async fn set_clock_high<C: Pin<Error = P>, P: Into<Error<P>> + digital::Error, T: DelayNs>(
        &self,
        scl: &mut C,
        timer: &mut T,
    ) -> Result<(), Error<P>> {
        scl.set_high().map_err(Error::Gpio)?;
        timer.delay_ns(self.period_ns).await;

        async { scl.wait_for_high().await.map_err(Error::Gpio) }
            .or(async {
                timer.delay_ms(self.stretch_timeout_ms).await;
                Err(Error::StretchTimeout)
            })
            .await?;

        Ok(())
    }

    async fn wait(&self, timer: &mut impl DelayNs) {
        timer.delay_ns(self.period_ns).await;
    }
}

pub struct Initiator<T, D, C> {
    timer: T,
    sda: D,
    scl: C,
    waiter: Waiter,
}

impl<P: Into<Error<P>> + digital::Error, T: DelayNs, D: Pin<Error = P>, C: Pin<Error = P>>
    Initiator<T, D, C>
{
    pub fn new(
        timer: T,
        sda: D,
        scl: C,
        period: Duration,
        stretch_timeout: Duration,
    ) -> Result<Self, Error<P>> {
        Ok(Self {
            timer,
            sda,
            scl,
            waiter: Waiter {
                period_ns: period
                    .as_nanos()
                    .try_into()
                    .map_err(|_| Error::InvalidParameter)?,
                stretch_timeout_ms: stretch_timeout
                    .as_millis()
                    .try_into()
                    .map_err(|_| Error::InvalidParameter)?,
            },
        })
    }

    pub async fn init(&mut self) -> Result<(), Error<P>> {
        self.sda.set_high()?;
        self.waiter
            .set_clock_high(&mut self.scl, &mut self.timer)
            .await?;
        Ok(())
    }

    async fn start(&mut self) -> Result<(), Error<P>> {
        if self.sda.is_low()? {
            return Err(Error::ArbitrationLoss);
        }

        self.sda.set_low()?;
        self.wait().await;
        self.scl.set_low()?;
        Ok(())
    }

    async fn repeat_start(&mut self) -> Result<(), Error<P>> {
        self.sda.set_high()?;
        self.wait().await;
        self.waiter
            .set_clock_high(&mut self.scl, &mut self.timer)
            .await?;

        self.start().await
    }

    async fn stop(&mut self) -> Result<(), Error<P>> {
        self.sda.set_low()?;
        self.wait().await;

        self.waiter
            .set_clock_high(&mut self.scl, &mut self.timer)
            .await?;

        self.sda.set_high()?;
        self.wait().await;

        if self.sda.is_low()? {
            return Err(Error::ArbitrationLoss);
        }
        Ok(())
    }

    async fn read_bit(&mut self) -> Result<bool, Error<P>> {
        self.sda.set_high()?;
        self.wait().await;
        self.waiter
            .set_clock_high(&mut self.scl, &mut self.timer)
            .await?;

        let bit = self.sda.is_high()?;
        self.scl.set_low()?;
        Ok(bit)
    }

    async fn watch_arbitration<R, F: Future<Output = Result<R, Error<P>>>>(
        bit: bool,
        sda: &mut D,
        f: impl FnOnce() -> F,
    ) -> Result<R, Error<P>> {
        if bit {
            sda.set_high()?;
        } else {
            sda.set_low()?;
        }

        let fut = f();
        let loss = async {
            if bit {
                sda.wait_for_low().await?;
                Err(Error::ArbitrationLoss)
            } else {
                core::future::pending::<()>().await;
                unreachable!()
            }
        };
        Ok(loss.or(fut).await?)
    }

    async fn write_bit(&mut self, bit: bool) -> Result<(), Error<P>> {
        Self::watch_arbitration(bit, &mut self.sda, async || {
            self.waiter.wait(&mut self.timer).await;
            self.waiter
                .set_clock_high(&mut self.scl, &mut self.timer)
                .await?;
            self.scl.set_low()?;

            Ok(())
        })
        .await
    }

    async fn wait(&mut self) {
        self.waiter.wait(&mut self.timer).await;
    }

    async fn read_byte(&mut self, nack: bool) -> Result<u8, Error<P>> {
        let mut byte = 0;
        for i in 0..8 {
            let bit = self.read_bit().await?;
            if bit {
                byte |= 0x80 >> i;
            }
        }

        self.write_bit(nack).await?;
        Ok(byte)
    }

    /// Returns a boolean with the NACK status of the byte written.
    async fn write_byte(&mut self, byte: u8) -> Result<bool, Error<P>> {
        for i in 0..8 {
            let bit = (0x80 >> i) & byte != 0;
            self.write_bit(bit).await?;
        }
        self.read_bit().await
    }

    async fn read_transaction(
        &mut self,
        address: SevenBitAddress,
        buffer: &mut [u8],
    ) -> Result<(), Error<P>> {
        if address > 127 {
            return Err(Error::Address);
        }
        let nack = self.write_byte(address << 1 | 1).await?;
        if nack {
            return Err(Error::Nack(NoAcknowledgeSource::Address));
        }
        let buffer_len = buffer.len();
        for (last, byte) in buffer
            .iter_mut()
            .enumerate()
            .map(|(i, byte)| (i == buffer_len - 1, byte))
        {
            *byte = self.read_byte(last).await?;
        }
        Ok(())
    }

    async fn write_transaction(
        &mut self,
        address: SevenBitAddress,
        buffer: &[u8],
    ) -> Result<(), Error<P>> {
        if address > 127 {
            return Err(Error::Address);
        }
        let nack = self.write_byte(address << 1).await?;
        if nack {
            return Err(Error::Nack(NoAcknowledgeSource::Address));
        }
        let buffer_len = buffer.len();
        for (last, byte) in buffer
            .iter()
            .enumerate()
            .map(|(i, byte)| (i == buffer_len - 1, byte))
        {
            let nack = self.write_byte(*byte).await?;
            if nack && !last {
                return Err(Error::Nack(NoAcknowledgeSource::Data));
            }
        }
        Ok(())
    }

    async fn transaction_inner(&mut self, operation: Operation<'_>) -> Result<(), Error<P>> {
        match operation {
            Operation::Write(address, buffer) => self.write_transaction(address, buffer).await,
            Operation::Read(address, buffer) => self.read_transaction(address, buffer).await,
        }
    }

    pub async fn transaction<'a>(
        &mut self,
        operations: impl Iterator<Item = Operation<'a>>,
    ) -> Result<(), Error<P>> {
        for (i, operation) in operations.enumerate() {
            if i == 0 {
                self.start().await?;
            } else {
                self.repeat_start().await?;
            }
            if let Err(e) = self.transaction_inner(operation).await {
                self.stop().await?;
                return Err(e);
            }
        }
        self.stop().await?;
        Ok(())
    }
}

impl<P: Into<Error<P>> + digital::Error, T, D: Pin<Error = P>, C: Pin<Error = P>> i2c::ErrorType
    for Initiator<T, D, C>
{
    type Error = Error<P>;
}

impl<P: Into<Error<P>> + digital::Error, T: DelayNs, D: Pin<Error = P>, C: Pin<Error = P>> i2c::I2c
    for Initiator<T, D, C>
{
    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.transaction(operations.iter_mut().map(|op| match op {
            i2c::Operation::Read(buffer) => Operation::Read(address, buffer),
            i2c::Operation::Write(buffer) => Operation::Write(address, buffer),
        }))
        .await
    }
}
