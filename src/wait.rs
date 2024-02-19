//! The VL53L4CD sensor both pulls the `GPIO1` pin low and flips some bits when
//! a measurement is ready. This module provides a trait for waiting for a
//! measurement, regardless of how you connect the sensor to the
//! microcontroller.

use core::pin::pin;

use embedded_hal_async::{delay::DelayNs, digital::Wait, i2c::I2c};

use crate::{i2c::Device, Register};

pub(crate) async fn has_measurement<I2C>(i2c: &mut Device<I2C>) -> Result<bool, I2C::Error>
where
    I2C: I2c,
{
    let ctrl = i2c.read_byte(Register::GPIO_HV_MUX_CTRL).await?;
    let status = i2c.read_byte(Register::GPIO_TIO_HV_STATUS).await?;
    Ok(status & 1 != ctrl >> 4 & 1)
}

/// Trait for waiting for a measurement to be ready.
pub trait WaitForMeasurement<I2C, DELAY>
where
    I2C: I2c,
    DELAY: DelayNs,
{
    /// Wait for a measurement to be ready.
    ///
    /// # Errors
    ///
    /// - [`crate::Error::Timeout`] if the sensor does not respond within around 1 second.
    /// - [`crate::Error::Gpio`] if a GPIO error occurs.
    #[allow(async_fn_in_trait)]
    async fn wait_for_measurement(
        &mut self,
        i2c: &mut Device<I2C>,
        delay: &mut DELAY,
    ) -> Result<(), crate::Error<I2C::Error>>;
}

/// Wait for new measurements by polling the sensor continuously.
pub struct Poll;

impl<I2C, DELAY> WaitForMeasurement<I2C, DELAY> for Poll
where
    I2C: I2c,
    DELAY: DelayNs,
{
    #[inline]
    async fn wait_for_measurement(
        &mut self,
        i2c: &mut Device<I2C>,
        delay: &mut DELAY,
    ) -> Result<(), crate::Error<I2C::Error>> {
        for _ in 0u16..1000 {
            if has_measurement(i2c).await? {
                return Ok(());
            }
            delay.delay_ms(1).await;
        }

        Err(crate::Error::Timeout)
    }
}

/// Wait for `GPIO1` (the sensor's interrupt output) to go low.
pub struct Interrupt<PIN>(pub PIN);

impl<PIN, I2C, DELAY> WaitForMeasurement<I2C, DELAY> for Interrupt<PIN>
where
    I2C: I2c,
    DELAY: DelayNs,
    PIN: Wait,
{
    #[inline]
    async fn wait_for_measurement(
        &mut self,
        _i2c: &mut Device<I2C>,
        delay: &mut DELAY,
    ) -> Result<(), crate::Error<I2C::Error>> {
        use futures_util::future::{select, Either};

        match select(pin!(delay.delay_ms(1000)), pin!(self.0.wait_for_low())).await {
            Either::Left(_) => Err(crate::Error::Timeout),
            Either::Right((Ok(()), _)) => Ok(()),
            Either::Right((Err(_gpio_err), _)) => Err(crate::Error::Gpio),
        }
    }
}
