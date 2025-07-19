//! Bit-banged async I2C implementation
use core::time::Duration;
use embedded_hal::i2c::{NoAcknowledgeSource, SevenBitAddress};
use embedded_hal_async::{delay::DelayNs, i2c};
use futures_lite::FutureExt;
use std::fmt::Debug;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<P> {
    Bus,
    ArbitrationLoss,
    StretchTimeout,
    Nack(NoAcknowledgeSource),
    Overrun,
    PinError(P),
    Other,
}

impl<P: Debug> embedded_hal_async::i2c::Error for Error<P> {
    fn kind(&self) -> i2c::ErrorKind {
        match self {
            Error::Bus => i2c::ErrorKind::Bus,
            Error::ArbitrationLoss => i2c::ErrorKind::ArbitrationLoss,
            Error::StretchTimeout => i2c::ErrorKind::Other,
            Error::Nack(source) => i2c::ErrorKind::NoAcknowledge(*source),
            Error::Overrun => i2c::ErrorKind::Overrun,
            Error::PinError(_) => i2c::ErrorKind::Bus,
            Error::Other => i2c::ErrorKind::Other,
        }
    }
}

impl<P: embedded_hal::digital::Error> From<P> for Error<P> {
    fn from(err: P) -> Self {
        Error::PinError(err)
    }
}
pub struct Operation<'a> {
    address: SevenBitAddress,
    buffer: OperationType<'a>,
}

pub trait I2cPin: embedded_hal::digital::InputPin + embedded_hal_async::digital::Wait {
    fn set_low(&mut self) -> Result<(), Self::Error>;
    fn set_high(&mut self) -> Result<(), Self::Error>;
}

pub enum OperationType<'a> {
    Read(&'a mut [u8]),
    Write(&'a [u8]),
}

pub struct Initiator<T: DelayNs, D: I2cPin, C: I2cPin> {
    timer: T,
    sda: D,
    scl: C,
    period: Duration,
    stretch_timeout: Duration,
}

impl<
    P: Into<Error<P>> + embedded_hal::digital::Error,
    T: DelayNs,
    D: I2cPin<Error = P>,
    C: I2cPin<Error = P>,
> Initiator<T, D, C>
{
    pub fn new(timer: T, sda: D, scl: C, period: Duration, stretch_timeout: Duration) -> Self {
        Self {
            timer,
            sda,
            scl,
            period,
            stretch_timeout,
        }
    }

    pub async fn init(&mut self) -> Result<(), Error<P>> {
        self.sda.set_high()?;
        self.scl.set_high()?;
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
        self.set_clock_high().await?;

        self.start().await
    }

    async fn stop(&mut self) -> Result<(), Error<P>> {
        self.sda.set_low()?;
        self.wait().await;

        self.set_clock_high().await?;

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
        self.set_clock_high().await?;

        let bit = self.sda.is_high()?;
        self.scl.set_low()?;
        Ok(bit)
    }

    async fn write_bit(&mut self, bit: bool) -> Result<(), Error<P>> {
        if bit {
            self.sda.set_high()?;
        } else {
            self.sda.set_low()?;
        }
        self.wait().await;
        self.set_clock_high().await?;

        if bit && self.sda.is_low()? {
            return Err(Error::ArbitrationLoss);
        }

        self.scl.set_low()?;
        Ok(())
    }

    async fn stretch_timeout(timer: &mut T, stretch_timeout: Duration) -> Result<(), Error<P>> {
        timer
            .delay_ms(stretch_timeout.as_millis().try_into().unwrap())
            .await;
        Err(Error::StretchTimeout)
    }

    async fn set_clock_high(&mut self) -> Result<(), Error<P>> {
        self.scl.set_high()?;
        self.wait().await;

        async { self.scl.wait_for_high().await.map_err(Error::PinError) }
            .or(Self::stretch_timeout(&mut self.timer, self.stretch_timeout))
            .await?;

        Ok(())
    }

    async fn wait(&mut self) {
        self.timer
            .delay_ns(self.period.as_nanos().try_into().unwrap())
            .await;
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

    async fn write_byte(&mut self, byte: u8) -> Result<bool, Error<P>> {
        //! Returns a boolean with the NACK status of the byte written.
        for i in 0..8 {
            let bit = (0x80 >> i) & byte != 0;
            self.write_bit(bit).await?;
        }
        Ok(self.read_bit().await?)
    }

    async fn read_transaction(
        &mut self,
        address: SevenBitAddress,
        buffer: &mut [u8],
    ) -> Result<(), Error<P>> {
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
        let nack = self.write_byte(address << 1).await?;
        if nack {
            return Err(Error::Nack(NoAcknowledgeSource::Address));
        }
        let buffer_len = buffer.len();
        for (last, byte) in buffer
            .iter()
            .enumerate()
            .map(|(i, byte)| (i + 1 == buffer_len, byte))
        {
            let nack = self.write_byte(*byte).await?;
            if nack && !last {
                return Err(Error::Nack(NoAcknowledgeSource::Data));
            }
        }
        Ok(())
    }

    async fn transaction_inner(&mut self, operation: &mut Operation<'_>) -> Result<(), Error<P>> {
        match operation.buffer {
            OperationType::Write(buffer) => self.write_transaction(operation.address, buffer).await,
            OperationType::Read(ref mut buffer) => {
                self.read_transaction(operation.address, buffer).await
            }
        }
    }

    pub async fn transaction<'a>(
        &mut self,
        operations: impl Iterator<Item = &'a mut Operation<'a>>,
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
