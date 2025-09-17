#![no_std]
#![deny(missing_docs)]
//! Embassy-compatible driver for the MMC5983MA 3-axis magnetometer
use bitfield_struct::bitfield;

use crate::{
    config::Mmc5983Config,
    interface::{I2cInterface, SpiInterface},
};

#[cfg(feature = "async")]
mod r#async;
mod config;
mod interface;
mod registers;
#[cfg(feature = "sync")]
mod sync;

/// Default I2C address for the MMC5983MA sensor
pub const DEFAULT_I2C_ADDRESS: u8 = 0x30;
const SCALE_FACTOR: f32 = 0.0625; // mG/LSB

pub use crate::config::{
    AxisInhibit, ContinuousMeasurementFreq, DecimationBw, Mmc5983ConfigBuilder, PeriodicSetInterval,
};
#[cfg(feature = "async")]
pub use crate::r#async::AsyncFunctions;
#[cfg(feature = "sync")]
pub use crate::sync::SyncFunctions;

/// MMC5983MA 3-axis magnetometer device
pub struct Mmc5983<IFACE, D> {
    iface: IFACE,
    delay: D,
    config: Mmc5983Config,
    ofst: Option<MagMeasurementRaw>,
    running: bool,
}

impl<I2C, D> Mmc5983<I2cInterface<I2C>, D> {
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
    pub fn new_with_i2c(i2c: I2C, address: u8, config: Mmc5983Config, delay: D) -> Self {
        Self {
            iface: I2cInterface { i2c, address },
            delay,
            config,
            ofst: None,
            running: false,
        }
    }
}

impl<SPI, D> Mmc5983<SpiInterface<SPI>, D> {
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
    pub async fn new_with_spi(spi: SPI, config: Mmc5983Config, delay: D) -> Self {
        Self {
            iface: SpiInterface { spi },
            delay,
            config,
            ofst: None,
            running: false,
        }
    }
}

/// Errors that can occur when interacting with the MMC5983MA sensor.
#[derive(Debug, Clone)]
pub enum Mmc5983Error<CommError> {
    /// I2C communication error
    Comm(CommError),
    /// Invalid device ID
    InvalidDevice,
    /// Measurement not ready
    NotReady,
    /// Invalid configuration
    InvalidConfig,
}

impl<CommError> From<CommError> for Mmc5983Error<CommError> {
    fn from(err: CommError) -> Self {
        Mmc5983Error::Comm(err)
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
        self.0 as f32 * 0.8 - 75.0
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
