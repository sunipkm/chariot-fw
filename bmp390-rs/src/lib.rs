#![no_std]
#![deny(missing_docs)]
//! Embassy-compatible driver for the BMP390 barometric pressure and temperature sensor
pub use crate::{
    config::{Bmp390Config, IrqPinConfig},
    interface::{I2cInterface, SpiInterface},
    registers::{IIRFilterConfig, OutputDataRate, Oversampling, SensorMode},
};

#[cfg(feature = "async")]
mod r#async;
mod config;
mod interface;
mod registers;
mod conversion;
#[cfg(feature = "sync")]
mod sync;

pub use uom::si::f32::{Length, Pressure, ThermodynamicTemperature};
pub use uom::si::length::{foot, meter};
pub use uom::si::pressure::{hectopascal, pascal, millibar};
pub use uom::si::thermodynamic_temperature::degree_celsius;

/// Default I2C address for the BMP390 sensor
pub const DEFAULT_I2C_ADDRESS: u8 = 0x77;

#[cfg(feature = "async")]
pub use crate::r#async::AsyncFunctions;
#[cfg(feature = "sync")]
pub use crate::sync::SyncFunctions;

/// BMI323 6-axis IMU device
pub struct Bmp390<IFACE, D> {
    iface: IFACE,
    delay: D,
    /// Configuration for the sensor
    pub config: Bmp390Config,
    running: bool,
}

impl<I2C, D> Bmp390<I2cInterface<I2C>, D> {
    /// Create a new instance of the [`Bmp390`] device.
    ///
    /// # Arguments
    /// * `i2c` - The I2C peripheral to use.
    /// * `address` - The I2C address of the device. Use `DEFAULT_I2C_ADDRESS`.
    /// * `config` - The configuration to use.
    /// * `delay` - A delay provider to use for initialization.
    ///
    /// # Errors
    /// * Returns [`Bmp390Error::Comm`] if there is an error communicating with the device.
    /// * Returns [`Bmp390Error::InvalidDevice`] if the device ID is incorrect.
    pub fn new_with_i2c(i2c: I2C, address: u8, config: Bmp390Config, delay: D) -> Self {
        Self {
            iface: I2cInterface { i2c, address },
            delay,
            config,
            running: false,
        }
    }
}

impl<SPI, D> Bmp390<SpiInterface<SPI>, D> {
    /// Create a new instance of the [`Mmc5983`] device.
    ///
    /// # Arguments
    /// * `spi` - The SPI peripheral to use.
    /// * `config` - The configuration to use.
    /// * `delay` - A delay provider to use for initialization.
    ///
    /// # Errors
    /// * Returns [`Bmp390Error::Comm`] if there is an error communicating with the device.
    /// * Returns [`Bmp390Error::InvalidDevice`] if the device ID is incorrect.
    pub async fn new_with_spi(spi: SPI, config: Bmp390Config, delay: D) -> Self {
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
pub enum Bmp390Error<CommError> {
    /// I2C communication error
    Comm(CommError),
    /// Fatal error reported by the sensor
    FatalError,
    /// Invalid sensor configuration
    InvalidConfiguration,
    /// Invalid device (wrong device ID)
    InvalidDevice,
    /// Driver not ready (e.g., measurement not started)
    NotReady,
    /// No new data available to read
    NoDataAvailable,
    /// Invalid command sent to the sensor
    InvalidCommand,
}

impl<CommError> From<CommError> for Bmp390Error<CommError> {
    fn from(err: CommError) -> Self {
        Bmp390Error::Comm(err)
    }
}

/// Sensor readout
#[derive(Debug, Clone, Copy)]
pub struct Measurement {
    /// Raw pressure data
    pub pressure: Option<u32>,
    /// Raw temperature data
    pub temperature: Option<u32>,
}

const MAX_LOOPS: usize = 100;
