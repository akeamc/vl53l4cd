use core::future::{self, Future};
#[cfg(feature = "tracing")]
use tracing::{instrument, trace};

use crate::Register;

pub trait I2c {
    type Error;

    type Read: Future<Output = Result<(), Self::Error>>;

    type Write: Future<Output = Result<(), Self::Error>>;

    fn read(&mut self, dest: &mut [u8]) -> Self::Read;

    fn write(&mut self, data: &[u8]) -> Self::Write;
}

#[inline]
pub(crate) async fn read_bytes<D: I2c>(
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
    ($name:ident, $out:ty, $bytes:literal) => {
        #[inline]
        pub async fn $name<D: I2c>(i2c: &mut D, reg: Register) -> Result<$out, D::Error> {
            let mut buf = [0; $bytes];
            read_bytes(i2c, reg, &mut buf).await?;
            Ok(<$out>::from_be_bytes(buf))
        }
    };
}

read_impl!(read_byte, u8, 1);
read_impl!(read_word, u16, 2);
read_impl!(read_dword, u32, 4);

macro_rules! write_impl {
    ($name:ident, $in:ty, $bytes:literal) => {
        #[cfg_attr(feature = "tracing", instrument(level = "trace", skip(i2c)))]
        #[inline]
        pub async fn $name<D: I2c>(i2c: &mut D, reg: Register, data: $in) -> Result<(), D::Error> {
            let mut msg = [0; 2 + $bytes]; // 2 bytes for register selection, rest for data
            msg[..2].copy_from_slice(&reg.as_bytes());
            msg[2..].copy_from_slice(&data.to_be_bytes());
            #[cfg(feature = "tracing")]
            trace!("write {:x?}", msg);
            i2c.write(&msg).await?;
            Ok(())
        }
    };
}

write_impl!(write_byte, u8, 1);
write_impl!(write_word, u16, 2);
write_impl!(write_dword, u32, 4);

#[cfg(feature = "i2cdev")]
impl<D> I2c for D
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
