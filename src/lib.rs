//! Async driver for the [VL53L4CD ToF distance sensor](https://www.st.com/en/imaging-and-photonics-solutions/vl53l4cd.html).
//!
//! ```no_run
//! # tokio_test::block_on(async {
//! use vl53l4cd::Vl53l4cd;
//! use vl53l4cd::i2cdev::linux::LinuxI2CDevice;
//!
//! let mut dev = LinuxI2CDevice::new("/dev/i2c-1", vl53l4cd::PERIPHERAL_ADDR)?;
//! let mut vl53 = Vl53l4cd::new(dev);
//!
//! vl53.init().await?;
//! vl53.set_range_timing(200, 0)?;
//! vl53.start_ranging().await?;
//!
//! loop {
//!     let measurement = vl53.measure().await?;
//!     if measurement.is_valid() {
//!         println!("{} mm", measurement.distance);
//!     }
//! }
//! # Ok::<(), i2cdev::linux::LinuxI2CError>(())
//! # });
//! ```

#![warn(missing_docs)]

pub use i2cdev;
use tokio::time::sleep;

use std::time::Duration;

use i2cdev::{
    core::I2CDevice,
    linux::{LinuxI2CDevice, LinuxI2CError},
};

#[cfg(feature = "tracing")]
use tracing::{debug, instrument, trace};

const DEFAULT_CONFIG: &[u8] = &[
    // value    addr : description
    0x12, // 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else don't touch
    0x00, // 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD)
    0x00, // 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD)
    0x11, // 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low (bits 3:0 must be 0x1)
    0x02, // 0x31 : bit 1 = interrupt depending on the polarity
    0x00, // 0x32 : not user-modifiable
    0x02, // 0x33 : not user-modifiable
    0x08, // 0x34 : not user-modifiable
    0x00, // 0x35 : not user-modifiable
    0x08, // 0x36 : not user-modifiable
    0x10, // 0x37 : not user-modifiable
    0x01, // 0x38 : not user-modifiable
    0x01, // 0x39 : not user-modifiable
    0x00, // 0x3a : not user-modifiable
    0x00, // 0x3b : not user-modifiable
    0x00, // 0x3c : not user-modifiable
    0x00, // 0x3d : not user-modifiable
    0xFF, // 0x3e : not user-modifiable
    0x00, // 0x3f : not user-modifiable
    0x0F, // 0x40 : not user-modifiable
    0x00, // 0x41 : not user-modifiable
    0x00, // 0x42 : not user-modifiable
    0x00, // 0x43 : not user-modifiable
    0x00, // 0x44 : not user-modifiable
    0x00, // 0x45 : not user-modifiable
    0x20, // 0x46 : interrupt configuration 0->level low detection, 1-> level high, 2-> Out of window, 3->In window, 0x20-> New sample ready , TBC
    0x0B, // 0x47 : not user-modifiable
    0x00, // 0x48 : not user-modifiable
    0x00, // 0x49 : not user-modifiable
    0x02, // 0x4a : not user-modifiable
    0x14, // 0x4b : not user-modifiable
    0x21, // 0x4c : not user-modifiable
    0x00, // 0x4d : not user-modifiable
    0x00, // 0x4e : not user-modifiable
    0x05, // 0x4f : not user-modifiable
    0x00, // 0x50 : not user-modifiable
    0x00, // 0x51 : not user-modifiable
    0x00, // 0x52 : not user-modifiable
    0x00, // 0x53 : not user-modifiable
    0xC8, // 0x54 : not user-modifiable
    0x00, // 0x55 : not user-modifiable
    0x00, // 0x56 : not user-modifiable
    0x38, // 0x57 : not user-modifiable
    0xFF, // 0x58 : not user-modifiable
    0x01, // 0x59 : not user-modifiable
    0x00, // 0x5a : not user-modifiable
    0x08, // 0x5b : not user-modifiable
    0x00, // 0x5c : not user-modifiable
    0x00, // 0x5d : not user-modifiable
    0x01, // 0x5e : not user-modifiable
    0xCC, // 0x5f : not user-modifiable
    0x07, // 0x60 : not user-modifiable
    0x01, // 0x61 : not user-modifiable
    0xF1, // 0x62 : not user-modifiable
    0x05, // 0x63 : not user-modifiable
    0x00, // 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB), default value 90 mm
    0xA0, // 0x65 : Sigma threshold LSB
    0x00, // 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB)
    0x80, // 0x67 : Min count Rate LSB
    0x08, // 0x68 : not user-modifiable
    0x38, // 0x69 : not user-modifiable
    0x00, // 0x6a : not user-modifiable
    0x00, // 0x6b : not user-modifiable
    0x00, // 0x6c : Intermeasurement period MSB, 32 bits register
    0x00, // 0x6d : Intermeasurement period
    0x0F, // 0x6e : Intermeasurement period
    0x89, // 0x6f : Intermeasurement period LSB
    0x00, // 0x70 : not user-modifiable
    0x00, // 0x71 : not user-modifiable
    0x00, // 0x72 : distance threshold high MSB (in mm, MSB+LSB)
    0x00, // 0x73 : distance threshold high LSB
    0x00, // 0x74 : distance threshold low MSB ( in mm, MSB+LSB)
    0x00, // 0x75 : distance threshold low LSB
    0x00, // 0x76 : not user-modifiable
    0x01, // 0x77 : not user-modifiable
    0x07, // 0x78 : not user-modifiable
    0x05, // 0x79 : not user-modifiable
    0x06, // 0x7a : not user-modifiable
    0x06, // 0x7b : not user-modifiable
    0x00, // 0x7c : not user-modifiable
    0x00, // 0x7d : not user-modifiable
    0x02, // 0x7e : not user-modifiable
    0xC7, // 0x7f : not user-modifiable
    0xFF, // 0x80 : not user-modifiable
    0x9B, // 0x81 : not user-modifiable
    0x00, // 0x82 : not user-modifiable
    0x00, // 0x83 : not user-modifiable
    0x00, // 0x84 : not user-modifiable
    0x01, // 0x85 : not user-modifiable
    0x00, // 0x86 : clear interrupt, 0x01=clear
    0x00, // 0x87 : ranging, 0x00=stop, 0x40=start
];

const VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND: u16 = 0x0008;
const SYSTEM_START: u16 = 0x0087;
const GPIO_HV_MUX_CTRL: u16 = 0x0030;
const GPIO_TIO_HV_STATUS: u16 = 0x0031;
const RANGE_CONFIG_A: u16 = 0x005e;
const RANGE_CONFIG_B: u16 = 0x0061;
const INTERMEASUREMENT_MS: u16 = 0x006c;
const SYSTEM_INTERRUPT_CLEAR: u16 = 0x0086;
const RESULT_RANGE_STATUS: u16 = 0x0089;
const RESULT_NUM_SPADS: u16 = 0x008c;
const RESULT_SIGNAL_RATE: u16 = 0x008e;
const RESULT_AMBIENT_RATE: u16 = 0x0090;
const RESULT_SIGMA: u16 = 0x0092;
const RESULT_DISTANCE: u16 = 0x0096;
// const RESULT_OSC_CALIBRATE_VAL: u16 = 0x00de;
const IDENTIFICATION_MODEL_ID: u16 = 0x010f;

/// Default I<sup>2</sup>C address of the VL53L4CD.
pub const PERIPHERAL_ADDR: u16 = 0x29;

const DATA_POLL_INTERVAL: Duration = Duration::from_millis(1);
const BOOT_POLL_INTERVAL: Duration = Duration::from_millis(1);

/// A measurement status.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[repr(u8)]
pub enum Status {
    /// Returned distance is valid.
    Valid = 0,
    /// Sigma is above the defined threshol.
    SigmaAboveThreshold,
    /// Signal is below the defined threshold.
    SigmaBelowThreshold,
    /// Measured distance is below detection threshold.
    DistanceBelowDetectionThreshold,
    /// Phase out of valid limit.
    InvalidPhase,
    /// Hardware failure.
    HardwareFail,
    /// Phase valid but no wrap around check performed.
    NoWrapAroundCheck,
    /// Wrapped target, phase does not match.
    WrappedTargetPhaseMismatch,
    /// Processing fail.
    ProcessingFail,
    /// Crosstalk signal fail.
    XTalkFail,
    /// Interrupt error.
    InterruptError,
    /// Merged target.
    MergedTarget,
    /// Too low signal.
    SignalTooWeak,
    /// Other error (e.g. boot error).
    Other = 255,
}

/// Severity of a measurement status.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Severity {
    /// The measurement is completely valid.
    None,
    /// The computed measurement might be somewhat correct.
    Warning,
    /// Something went very wrong.
    Error,
}

impl Status {
    fn from_rtn(rtn: u8) -> Self {
        assert!(rtn < 24); // if rtn >= 24, return rtn

        match rtn {
            3 => Self::HardwareFail,
            4 | 5 => Self::SigmaBelowThreshold,
            6 => Self::SigmaAboveThreshold,
            7 => Self::WrappedTargetPhaseMismatch,
            8 => Self::DistanceBelowDetectionThreshold,
            9 => Self::Valid,
            12 => Self::XTalkFail,
            13 => Self::InterruptError,
            18 => Self::InterruptError,
            19 => Self::NoWrapAroundCheck,
            22 => Self::MergedTarget,
            23 => Self::SignalTooWeak,
            _ => Self::Other,
        }
    }

    /// Severity of this status as per the user manual.
    pub fn severity(&self) -> Severity {
        match self {
            Status::Valid => Severity::None,
            Status::SigmaAboveThreshold => Severity::Warning,
            Status::SigmaBelowThreshold => Severity::Warning,
            Status::DistanceBelowDetectionThreshold => Severity::Error,
            Status::InvalidPhase => Severity::Error,
            Status::HardwareFail => Severity::Error,
            Status::NoWrapAroundCheck => Severity::Warning,
            Status::WrappedTargetPhaseMismatch => Severity::Error,
            Status::ProcessingFail => Severity::Error,
            Status::XTalkFail => Severity::Error,
            Status::InterruptError => Severity::Error,
            Status::MergedTarget => Severity::Error,
            Status::SignalTooWeak => Severity::Error,
            Status::Other => Severity::Error,
        }
    }
}

/// A VL53L4CD measurement.
#[derive(Debug, Clone, Copy)]
pub struct Measurement {
    /// Validity of the measurement.
    pub status: Status,
    /// Measured distance to the target (millimeters).
    pub distance: u16,
    /// Ambient rate measurement performed on the
    /// return array, with no active photon emission, to
    /// measure the ambient signal rate due to noise.
    ///
    /// The returned value is measured in thousand counts
    /// per second (kcps) (10<sup>3</sup> * s<sup>-1</sup>).
    pub ambient_rate: u16,
    /// Number of detected photos during the VCSEL pulse.
    ///
    /// The returned value is measured in thousand counts
    /// per second (kcps) (10<sup>3</sup> * s<sup>-1</sup>).
    pub signal_rate: u16,
    /// Number of SPADs enabled for this measurement. Targets that
    /// are far away or have low reflectance will activate
    /// more SPADs.
    pub spads_enabled: u16,
    /// Sigma estimator for the noise in the reported
    /// target distance (millimeters).
    pub sigma: u16,
}

impl Measurement {
    /// Whether this measurement is valid or not, given its status.
    #[inline]
    pub fn is_valid(&self) -> bool {
        self.status == Status::Valid
    }
}

/// A VL53L4CD ToF range sensor.
pub struct Vl53l4cd {
    i2c: LinuxI2CDevice,
}

impl Vl53l4cd {
    /// Construct a new sensor, without sending
    /// any commands. To begin measuring, you
    /// need to call [`Self::init`] as well as
    /// [`Self::start_ranging`]:
    ///
    /// ```no_run
    ///
    /// ```
    pub fn new(i2c: LinuxI2CDevice) -> Self {
        Self { i2c }
    }

    /// Initialize the sensor.
    #[cfg_attr(feature = "tracing", instrument(err, skip(self)))]
    pub async fn init(&mut self) -> Result<(), LinuxI2CError> {
        self.i2c.write(&[])?;

        let id = self.device_id()?;

        assert_eq!(id, 0xebaa, "strange id ({:x})", id);

        #[cfg(feature = "tracing")]
        debug!("waiting for boot");

        while self.read_byte(0xE5)? != 0x3 {
            sleep(BOOT_POLL_INTERVAL).await; // wait for boot
        }

        #[cfg(feature = "tracing")]
        debug!("booted");

        self.write_bytes(0x2d, DEFAULT_CONFIG)?;

        // start VHV
        self.start_ranging().await?;
        self.stop_ranging()?;
        self.write_byte(VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, 0x09)?;
        self.write_byte(0x0b, 0)?;
        self.write_word(0x0024, 0x500)?;

        self.set_range_timing(50, 0)?;

        Ok(())
    }

    /// Set the range timing for this sensor. The timing budget *must*
    /// be greater than or equal to 10 ms and less than or equal to 200 ms.
    ///
    /// # Panics
    ///
    /// Panics if the timing budget is less than 10 ms or more than 200 ms,
    /// or if the timing budget is less than the inter-measurement time
    /// (except for then the inter-measurement time is zero).
    ///
    /// If the oscillation frequency reported by the sensor (2 bytes starting
    /// at register `0x0006`) is zero, this function panics.
    pub fn set_range_timing(
        &mut self,
        timing_budget_ms: u32,
        inter_measurement_ms: u32,
    ) -> Result<(), LinuxI2CError> {
        assert!(
            (10..=200).contains(&timing_budget_ms),
            "timing budget must be in range [10, 200]"
        );

        let osc_freq = u32::from(self.read_word(0x0006)?);

        assert_ne!(osc_freq, 0, "oscillation frequency is zero");

        let mut timing_budget_us = timing_budget_ms * 1000;
        let macro_period_us = (2304 * (0x40000000 / osc_freq)) >> 6;

        if inter_measurement_ms == 0 {
            // continuous mode
            self.write_dword(INTERMEASUREMENT_MS, 0)?;
            timing_budget_us -= 2500;
        } else if inter_measurement_ms > timing_budget_ms {
            // autonomous low power mode
            timing_budget_us -= 4300;
            timing_budget_us /= 2;
        } else {
            panic!("timing budget must not be less than inter-measurement");
        }

        // reg a
        let mut ms_byte = 0;
        timing_budget_us <<= 12;
        let mut tmp = macro_period_us * 16;
        let mut ls_byte = ((timing_budget_us + ((tmp >> 6) >> 1)) / (tmp >> 6)) - 1;

        while (ls_byte & 0xFFFFFF00) > 0 {
            ls_byte >>= 1;
            ms_byte += 1;
        }
        ms_byte <<= 8 + (ls_byte * 0xff) as u16;
        self.write_word(RANGE_CONFIG_A, ms_byte)?;

        // reg b
        ms_byte = 0;
        tmp = macro_period_us * 12;
        let mut ls_byte = ((timing_budget_us + ((tmp >> 6) >> 1)) / (tmp >> 6)) - 1;

        while (ls_byte & 0xFFFFFF00) > 0 {
            ls_byte >>= 1;
            ms_byte += 1;
        }
        ms_byte = (ms_byte << 8) + (ls_byte & 0xFF) as u16;
        self.write_word(RANGE_CONFIG_B, ms_byte)?;

        Ok(())
    }

    /// Wait for a measurement to be available on the sensor and then read
    /// the measurement. This function polls the sensor for a measurement
    /// until one is available, reads the measurement and finally clears
    /// the interrupt in order to request another measurement.
    pub async fn measure(&mut self) -> Result<Measurement, LinuxI2CError> {
        while !self.has_measurement()? {
            sleep(DATA_POLL_INTERVAL).await;
        }

        let measurement = self.read_measurement()?;
        self.clear_interrupt()?;

        Ok(measurement)
    }

    #[inline]
    fn has_measurement(&mut self) -> Result<bool, LinuxI2CError> {
        let ctrl = self.read_byte(GPIO_HV_MUX_CTRL)?;
        let status = self.read_byte(GPIO_TIO_HV_STATUS)?;
        Ok(status & 1 != ctrl >> 4 & 1)
    }

    #[inline]
    fn read_measurement(&mut self) -> Result<Measurement, LinuxI2CError> {
        let status = self.read_byte(RESULT_RANGE_STATUS)? & 0x1f;

        Ok(Measurement {
            status: Status::from_rtn(status),
            distance: self.read_word(RESULT_DISTANCE)?,
            spads_enabled: self.read_word(RESULT_NUM_SPADS)? / 256,
            ambient_rate: self.read_word(RESULT_AMBIENT_RATE)? * 8,
            signal_rate: self.read_word(RESULT_SIGNAL_RATE)? * 8,
            sigma: self.read_word(RESULT_SIGMA)? / 4,
        })
    }

    #[inline]
    fn clear_interrupt(&mut self) -> Result<(), LinuxI2CError> {
        self.write_byte(SYSTEM_INTERRUPT_CLEAR, 0x01)
    }

    /// Begin ranging.
    #[inline]
    pub async fn start_ranging(&mut self) -> Result<(), LinuxI2CError> {
        if self.read_dword(INTERMEASUREMENT_MS)? == 0 {
            // autonomous mode
            self.write_byte(SYSTEM_START, 0x21)
        } else {
            // continuous mode
            self.write_byte(SYSTEM_START, 0x40)
        }
    }

    /// Stop ranging.
    #[inline]
    pub fn stop_ranging(&mut self) -> Result<(), LinuxI2CError> {
        self.write_byte(SYSTEM_START, 0x00)
    }

    fn device_id(&mut self) -> Result<u16, LinuxI2CError> {
        self.read_word(IDENTIFICATION_MODEL_ID)
    }

    #[cfg_attr(feature = "tracing", instrument(level = "trace", skip(self, buf), fields(len = %buf.len())))]
    fn read_bytes(&mut self, reg: u16, buf: &mut [u8]) -> Result<(), LinuxI2CError> {
        #[cfg(feature = "tracing")]
        trace!("write {:x?}", reg.to_be_bytes());
        self.i2c.write(&reg.to_be_bytes())?;
        #[cfg(feature = "tracing")]
        trace!("read {}", buf.len());
        self.i2c.read(buf)
    }

    fn read_byte(&mut self, reg: u16) -> Result<u8, LinuxI2CError> {
        let mut buf = [0];
        self.read_bytes(reg, &mut buf)?;
        Ok(u8::from_be_bytes(buf))
    }

    fn read_word(&mut self, reg: u16) -> Result<u16, LinuxI2CError> {
        let mut buf = [0; 2];
        self.read_bytes(reg, &mut buf)?;
        Ok(u16::from_be_bytes(buf))
    }

    fn read_dword(&mut self, reg: u16) -> Result<u32, LinuxI2CError> {
        let mut buf = [0; 4];
        self.read_bytes(reg, &mut buf)?;
        Ok(u32::from_be_bytes(buf))
    }

    #[cfg_attr(feature = "tracing", instrument(level = "trace", skip(self, data)))]
    fn write_bytes(&mut self, reg: u16, data: &[u8]) -> Result<(), LinuxI2CError> {
        let data = [&reg.to_be_bytes(), data].concat();
        #[cfg(feature = "tracing")]
        trace!("write {:x?}", data);
        self.i2c.write(&data)
    }

    fn write_byte(&mut self, reg: u16, data: u8) -> Result<(), LinuxI2CError> {
        self.write_bytes(reg, &[data])
    }

    fn write_word(&mut self, reg: u16, data: u16) -> Result<(), LinuxI2CError> {
        self.write_bytes(reg, &data.to_be_bytes())
    }

    fn write_dword(&mut self, reg: u16, data: u32) -> Result<(), LinuxI2CError> {
        self.write_bytes(reg, &data.to_be_bytes())
    }
}
