//! [I²C](https://en.wikipedia.org/wiki/I%C2%B2C) abstractions.

use core::{future, mem};
#[cfg(feature = "tracing")]
use tracing::{instrument, trace};

use crate::Register;

/// A simple async I²C trait.
pub trait Device {
    /// I/O error for the I²C implementation.
    type Error;

    /// [`Future`](future::Future) returned by [`Self::read`].
    type Read: future::Future<Output = Result<(), Self::Error>>;

    /// [`Future`](future::Future) returned by [`Self::write`].
    type Write: future::Future<Output = Result<(), Self::Error>>;

    /// Read some bytes into a buffer.
    fn read(&mut self, dest: &mut [u8]) -> Self::Read;

    /// Write bytes.
    fn write(&mut self, data: &[u8]) -> Self::Write;
}

#[inline]
pub(crate) async fn read_bytes<D: Device>(
    i2c: &mut D,
    reg: Register,
    dest: &mut [u8],
) -> Result<(), D::Error> {
    #[cfg(feature = "tracing")]
    trace!("write {:x?}", reg.as_bytes());
    i2c.write(&reg.as_bytes()).await?;

    #[cfg(feature = "tracing")]
    trace!("read {}", dest.len());
    i2c.read(dest).await?;

    Ok(())
}

macro_rules! read_impl {
    ($name:ident, $out:ty) => {
        #[inline]
        /// Read a
        #[doc = concat!("[`", stringify!($out), "`]")]
        /// from some [`Register`].
        pub async fn $name<D: Device>(i2c: &mut D, reg: Register) -> Result<$out, D::Error> {
            let mut buf = [0; mem::size_of::<$out>()];
            read_bytes(i2c, reg, &mut buf).await?;
            Ok(<$out>::from_be_bytes(buf))
        }
    };
}

read_impl!(read_byte, u8);
read_impl!(read_word, u16);
read_impl!(read_dword, u32);

macro_rules! write_impl {
    ($name:ident, $in:ty) => {
        #[cfg_attr(feature = "tracing", instrument(level = "trace", skip(i2c)))]
        #[inline]
        /// Write a
        #[doc = concat!("[`", stringify!($in), "`]")]
        /// into some [`Register`].
        pub async fn $name<D: Device>(
            i2c: &mut D,
            reg: Register,
            data: $in,
        ) -> Result<(), D::Error> {
            let mut msg = [0; 2 + mem::size_of::<$in>()]; // 2 bytes for register selection, rest for data
            msg[..2].copy_from_slice(&reg.as_bytes());
            msg[2..].copy_from_slice(&data.to_be_bytes());
            #[cfg(feature = "tracing")]
            trace!("write {:x?}", msg);
            i2c.write(&msg).await?;
            Ok(())
        }
    };
}

write_impl!(write_byte, u8);
write_impl!(write_word, u16);
write_impl!(write_dword, u32);

#[cfg(feature = "i2cdev")]
impl<D> Device for D
where
    D: i2cdev::core::I2CDevice,
{
    type Error = D::Error;

    type Read = future::Ready<Result<(), Self::Error>>;

    type Write = future::Ready<Result<(), Self::Error>>;

    fn read(&mut self, dest: &mut [u8]) -> Self::Read {
        future::ready(self.read(dest))
    }

    fn write(&mut self, data: &[u8]) -> Self::Write {
        future::ready(self.write(data))
    }
}

/// A mocked VL53L4CD I²C bus.
#[derive(Default)]
pub struct Mock;

impl Device for Mock {
    type Error = ();

    type Read = future::Ready<Result<(), Self::Error>>;

    type Write = future::Ready<Result<(), Self::Error>>;

    fn read(&mut self, _dest: &mut [u8]) -> Self::Read {
        todo!()
    }

    fn write(&mut self, _data: &[u8]) -> Self::Write {
        todo!()
    }
}
