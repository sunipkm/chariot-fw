#![no_std]
#![deny(missing_docs)]
//! Embassy-compatible driver for the BMI323 IMU
use bitfield_struct::bitfield;

pub use crate::{
    config::{Bmi323Config, IrqPinConfig},
    interface::{I2cInterface, SpiInterface},
    registers::{
        Bandwidth, AveragingSamples, OutputDataRate, AccelMode, AccelRange, GyroMode, GyroRange,
        IrqMap,
    },
};

#[cfg(feature = "async")]
mod r#async;
mod config;
mod interface;
mod registers;
#[cfg(feature = "sync")]
mod sync;

/// Default I2C address for the BMI323 sensor
pub const DEFAULT_I2C_ADDRESS: u8 = 0x68;

#[cfg(feature = "async")]
pub use crate::r#async::AsyncFunctions;
#[cfg(feature = "sync")]
pub use crate::sync::SyncFunctions;

/// BMI323 6-axis IMU device
pub struct Bmi323<IFACE, D> {
    iface: IFACE,
    delay: D,
    /// Configuration for the sensor
    pub config: Bmi323Config,
    running: bool,
}

impl<I2C, D> Bmi323<I2cInterface<I2C>, D> {
    /// Create a new instance of the [`Mmc5983`] device.
    ///
    /// # Arguments
    /// * `i2c` - The I2C peripheral to use.
    /// * `address` - The I2C address of the device. Use `DEFAULT_I2C_ADDRESS`.
    /// * `config` - The configuration to use.
    /// * `delay` - A delay provider to use for initialization.
    ///
    /// # Errors
    /// * Returns `Mmc5983Error::I2C` if there is an error communicating with the device.
    /// * Returns `Mmc5983Error::InvalidDevice` if the device ID is incorrect.
    pub fn new_with_i2c(i2c: I2C, address: u8, config: Bmi323Config, delay: D) -> Self {
        Self {
            iface: I2cInterface { i2c, address },
            delay,
            config,
            running: false,
        }
    }
}

impl<SPI, D> Bmi323<SpiInterface<SPI>, D> {
    /// Create a new instance of the [`Mmc5983`] device.
    ///
    /// # Arguments
    /// * `spi` - The SPI peripheral to use.
    /// * `config` - The configuration to use.
    /// * `delay` - A delay provider to use for initialization.
    ///
    /// # Errors
    /// * Returns `Mmc5983Error::I2C` if there is an error communicating with the device.
    /// * Returns `Mmc5983Error::InvalidDevice` if the device ID is incorrect.
    pub async fn new_with_spi(spi: SPI, config: Bmi323Config, delay: D) -> Self {
        Self {
            iface: SpiInterface { spi },
            delay,
            config,
            running: false,
        }
    }
}

/// Errors that can occur when interacting with the MMC5983MA sensor.
#[derive(Debug, Clone)]
pub enum Mmc5983Error<CommError> {
    /// I2C communication error
    Comm(CommError),
    /// Fatal error reported by the sensor
    FatalError,
    /// Invalid accelerometer configuration
    InvalidAccelConfig,
    /// Invalid gyroscope configuration
    InvalidGyroConfig,
    /// Invalid device (wrong device ID)
    InvalidDevice,
    /// Driver not ready (e.g., measurement not started)
    NotReady,
    /// No new data available to read
    NoDataAvailable,
    /// Driver is currently busy (e.g., during calibration)
    Busy,
    /// Operation requires a restart of the sensor
    RestartRequired,
    /// Self-calibration process timed out
    SelfCalTimedOut,
    /// No sensors (accelerometer or gyroscope) are enabled
    NoSensorsEnabled,
}

impl<CommError> From<CommError> for Mmc5983Error<CommError> {
    fn from(err: CommError) -> Self {
        Mmc5983Error::Comm(err)
    }
}

/// Raw measurement data from the sensor
#[bitfield(u64)]
pub struct MeasurementRaw3D {
    #[bits(16)]
    pub x: i16,
    #[bits(16)]
    pub y: i16,
    #[bits(16)]
    pub z: i16,
    #[bits(8)]
    pub kind: Measurement3DKind,
    #[bits(8)]
    _reserved: u8,
}

impl MeasurementRaw3D {
    /// Convert the raw measurement data to floating point values in physical units
    pub fn float(&self) -> (f32, f32, f32) {
        let sensitivity = match self.kind() {
            Measurement3DKind::Accel(range) => {
                range.sensitivity()
            }
            Measurement3DKind::Gyro(range) => {
                range.sensitivity()
            }
        };
        (
            self.x() as f32 / sensitivity,
            self.y() as f32 / sensitivity,
            self.z() as f32 / sensitivity,
        )
    }
}

/// Raw temperature measurement data from the sensor
#[derive(Debug, Clone, Copy)]
pub struct TemperatureMeasurement(pub i16);

impl TemperatureMeasurement {
    /// Convert the raw temperature measurement to degrees Celsius
    pub fn celcius(&self) -> f32 {
        self.0 as f32 / 512.0 + 23.0
    }
}

#[derive(Debug, Clone, Copy)]
/// Raw timestamp data from the sensor
pub struct TimestampMeasurement(pub u32);

/// Sensor readout
#[derive(Debug, Clone, Copy)]
pub struct Measurement {
    /// Accelerometer data, if enabled
    pub accel: Option<MeasurementRaw3D>,
    /// Gyroscope data, if enabled
    pub gyro: Option<MeasurementRaw3D>,
    /// Temperature data, if enabled
    pub temp: Option<TemperatureMeasurement>,
    /// Timestamp data, if enabled
    pub timestamp: TimestampMeasurement,
}


#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// 3D Measurement type
pub enum Measurement3DKind {
    /// Accelerometer measurement
    Accel(AccelRange),
    /// Gyroscope measurement
    Gyro(GyroRange),
}

impl Measurement3DKind {
    pub(crate) const fn into_bits(self) -> u8 {
        match self {
            Measurement3DKind::Accel(range) => range as u8,
            Measurement3DKind::Gyro(range) => 0x80 | (range as u8),
        }
    }

    pub(crate) const fn from_bits(bits: u8) -> Self {
        match bits & 0x80 {
            0x00 => Measurement3DKind::Accel(AccelRange::from_u8(bits & 0x7F)),
            0x80 => Measurement3DKind::Gyro(GyroRange::from_u8(bits & 0x7F)),
            _ => unreachable!(),
        }
    }
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Type of gyroscope self-calibration
pub enum SelfCalibrateType {
    /// Calibrate only sensitivity
    Sensitivity = 1,
    /// Calibrate only offset
    Offset = 2,
    /// Calibrate both sensitivity and offset
    Both = 3,
}

impl SelfCalibrateType {
    pub(crate) const fn sensitivity(&self) -> bool {
        matches!(self, SelfCalibrateType::Sensitivity | SelfCalibrateType::Both)
    }
    pub(crate) const fn offset(&self) -> bool {
        matches!(self, SelfCalibrateType::Offset | SelfCalibrateType::Both)
    }
}

const MAX_LOOPS: usize = 100;
