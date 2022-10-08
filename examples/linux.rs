//! This example has been tested on the Raspberry Pi Zero W.

use vl53l4cd::i2cdev::linux::LinuxI2CDevice;
use vl53l4cd::Vl53l4cd;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let dev = LinuxI2CDevice::new("/dev/i2c-1", vl53l4cd::PERIPHERAL_ADDR)?;
    let mut vl53 = Vl53l4cd::new(dev);

    vl53.init().await?;
    vl53.set_range_timing(200, 0).await?;
    vl53.start_ranging().await?;

    loop {
        let measurement = vl53.measure().await?;
        if measurement.is_valid() {
            println!("{} mm", measurement.distance);
        } else {
            println!("{:?}", measurement.status);
        }
    }
}
