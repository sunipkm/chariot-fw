#![no_std]
#![deny(missing_docs)]
//! Embassy-compatible driver for the MMC5983MA 3-axis magnetometer
use bitfield_struct::bitfield;

use crate::config::Mmc5983Config;

#[cfg(feature = "async")]
mod r#async;
mod config;
mod registers;
#[cfg(feature = "sync")]
mod sync;

/// Default I2C address for the MMC5983MA sensor
pub const DEFAULT_I2C_ADDRESS: u8 = 0x30;
const SCALE_FACTOR: f32 = 0.0625; // mG/LSB

pub use crate::config::{Mmc5983ConfigBuilder, ContinuousMeasurementFreq, DecimationBw, AxisInhibit, PeriodicSetInterval};
#[cfg(feature = "async")]
pub use crate::r#async::AsyncFunctions;
#[cfg(feature = "sync")]
pub use crate::sync::SyncFunctions;

/// MMC5983MA 3-axis magnetometer device
pub struct Mmc5983<I2C, D> {
    i2c: I2C,
    delay: D,
    address: u8,
    config: Mmc5983Config,
    ofst: Option<MagMeasurementRaw>,
    running: bool,
}

/// Errors that can occur when interacting with the MMC5983MA sensor.
#[derive(Debug, Clone)]
pub enum Mmc5983Error<I2CError> {
    /// I2C communication error
    I2C(I2CError),
    /// Invalid device ID
    InvalidDevice,
    /// Measurement not ready
    NotReady,
    /// Invalid configuration
    InvalidConfig,
}

impl<I2CError> From<I2CError> for Mmc5983Error<I2CError> {
    fn from(err: I2CError) -> Self {
        Mmc5983Error::I2C(err)
    }
}


/// Raw measurement data from the sensor
#[bitfield(u64)]
pub struct MagMeasurementRaw {
    #[bits(18)]
    pub x: i32,
    #[bits(18)]
    pub y: i32,
    #[bits(18)]
    pub z: i32,
    #[bits(10)]
    _reserved: u32,
}

/// Converted measurement data from the sensor
#[cfg(feature = "float")]
pub struct MagMeasurement {
    /// X axis measurement in milliGauss
    pub x: f32,
    /// Y axis measurement in milliGauss
    pub y: f32,
    /// Z axis measurement in milliGauss
    pub z: f32,
}

/// Raw temperature measurement from the sensor
pub struct TempMeasurementRaw(u8);

#[cfg(feature = "float")]
impl TempMeasurementRaw {
    /// Convert the raw temperature measurement to a float in degrees Celsius
    pub fn celcius(&self) -> f32 {
        self.0 as f32 * 0.8 -75.0
    }
}

#[cfg(feature = "float")]
impl MagMeasurementRaw {
    /// Convert the raw magnetometer measurement to floats in milliGauss
    pub fn milligauss(&self) -> MagMeasurement {
        MagMeasurement {
            x: (self.x()) as f32 * SCALE_FACTOR,
            y: (self.y()) as f32 * SCALE_FACTOR,
            z: (self.z()) as f32 * SCALE_FACTOR,
        }
    }
}

const MAX_LOOPS: usize = 100;
