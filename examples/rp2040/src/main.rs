#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Pull};
use embassy_rp::i2c::{self, Config, InterruptHandler};
use embassy_rp::peripherals::I2C1;
use vl53l4cd::Vl53l4cd;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    I2C1_IRQ => InterruptHandler<I2C1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let sda = p.PIN_2;
    let scl = p.PIN_3;

    let i2c = i2c::I2c::new_async(p.I2C1, scl, sda, Irqs, Config::default());
    let mut vl53 = Vl53l4cd::new(
        i2c,
        embassy_time::Delay,
        vl53l4cd::wait::Interrupt(Input::new(p.PIN_5, Pull::Up)), // or vl53l4cd::wait::Poll
    );

    vl53.init().await.unwrap();
    vl53.start_ranging().await.unwrap();

    loop {
        let measurement = vl53.measure().await.unwrap();
        info!("measurement: {:?}", measurement);
    }
}
