//! [I²C](https://en.wikipedia.org/wiki/I%C2%B2C) abstractions.

use embedded_hal_async::i2c::{I2c, SevenBitAddress};

use crate::Register;

pub struct Device<I2C> {
    pub addr: SevenBitAddress,
    pub bus: I2C,
}

impl<I2C: I2c> Device<I2C> {
    pub async fn read_bytes(&mut self, reg: Register, dest: &mut [u8]) -> Result<(), I2C::Error> {
        #[cfg(feature = "defmt-03")]
        defmt::trace!("write {:x}", reg.as_bytes());
        self.bus.write(self.addr, &reg.as_bytes()).await?;

        #[cfg(feature = "defmt-03")]
        defmt::trace!("read {}", dest.len());
        self.bus.read(self.addr, dest).await?;

        Ok(())
    }

    pub async fn write(&mut self, data: &[u8]) -> Result<(), I2C::Error> {
        self.bus.write(self.addr, data).await
    }
}

macro_rules! read_impl {
    ($name:ident, $out:ty) => {
        impl<I2C: I2c> Device<I2C> {
            /// Read from some [`Register`].
            pub async fn $name(&mut self, reg: Register) -> Result<$out, I2C::Error> {
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
        impl<I2C: I2c> Device<I2C> {
            /// Write to some [`Register`].
            pub async fn $name(&mut self, reg: Register, data: $in) -> Result<(), I2C::Error> {
                let mut msg = [0; 2 + core::mem::size_of::<$in>()]; // 2 bytes for register selection, rest for data
                msg[..2].copy_from_slice(&reg.as_bytes());
                msg[2..].copy_from_slice(&data.to_be_bytes());
                #[cfg(feature = "defmt-03")]
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
