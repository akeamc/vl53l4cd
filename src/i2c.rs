//! [I²C](https://en.wikipedia.org/wiki/I%C2%B2C) abstractions.

use embedded_hal_async::i2c::I2c;

use crate::Register;

pub struct Device<M: I2c> {
    pub addr: u8,
    pub i2c: M,
}

impl<M: I2c> Device<M> {
    pub async fn read_bytes(&mut self, reg: Register, dest: &mut [u8]) -> Result<(), M::Error> {
        #[cfg(feature = "defmt")]
        defmt::trace!("write {:x}", reg.as_bytes());
        self.i2c.write(self.addr, &reg.as_bytes()).await?;

        #[cfg(feature = "defmt")]
        defmt::trace!("read {}", dest.len());
        self.i2c.read(self.addr, dest).await?;

        Ok(())
    }

    pub async fn write(&mut self, data: &[u8]) -> Result<(), M::Error> {
        self.i2c.write(self.addr, data).await
    }
}

macro_rules! read_impl {
    ($name:ident, $out:ty) => {
        impl<M: I2c> Device<M> {
            /// Read a
            #[doc = concat!("[`", stringify!($out), "`]")]
            /// from some [`Register`].
            pub async fn $name(&mut self, reg: Register) -> Result<$out, M::Error> {
                let mut buf = [0; core::mem::size_of::<$out>()];
                self.read_bytes(reg, &mut buf).await?;
                Ok(<$out>::from_be_bytes(buf))
            }
        }
    };
}

read_impl!(read_byte, u8);
read_impl!(read_word, u16);
// read_impl!(read_dword, u32);

macro_rules! write_impl {
    ($name:ident, $in:ty) => {
        impl<M: I2c> Device<M> {
            /// Write a
            #[doc = concat!("[`", stringify!($in), "`]")]
            /// into some [`Register`].
            pub async fn $name(&mut self, reg: Register, data: $in) -> Result<(), M::Error> {
                let mut msg = [0; 2 + core::mem::size_of::<$in>()]; // 2 bytes for register selection, rest for data
                msg[..2].copy_from_slice(&reg.as_bytes());
                msg[2..].copy_from_slice(&data.to_be_bytes());
                #[cfg(feature = "defmt")]
                defmt::trace!("write {:x}", msg);
                self.write(&msg).await?;
                Ok(())
            }
        }
    };
}

write_impl!(write_byte, u8);
write_impl!(write_word, u16);
write_impl!(write_dword, u32);
