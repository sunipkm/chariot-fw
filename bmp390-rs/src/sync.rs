#[cfg(feature = "defmt")]
use defmt::{debug, error, info, trace, warn};
use embedded_hal::{delay::DelayNs, i2c, spi};

use crate::{
    conversion::CalibrationCoefficients,
    interface::Interface,
    registers::{
        Command, DeviceId, ErrorReg, EventReg, IrqControl, IrqStatus, OversamplingReg, PowerCtrl,
        Register, StatusReg, NVM_PAR_T1_0, PRESSURE_DATA_ADDR,
    },
    Bmp390, Bmp390Error, I2cInterface, IIRFilterConfig, Measurement, OutputDataRate, SensorMode,
    SpiInterface, MAX_LOOPS,
};

pub(crate) trait SyncInterface: Interface {
    type Error;
    fn write(&mut self, data: &[u8]) -> Result<(), Self::Error>;
    fn write_read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error>;
}

impl<I2C, E> SyncInterface for I2cInterface<I2C>
where
    I2C: i2c::I2c<Error = E>,
{
    type Error = E;

    fn write(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        self.i2c.write(self.address, data)
    }

    fn write_read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.transaction(
            self.address,
            &mut [
                i2c::Operation::Write(&[address]),
                i2c::Operation::Read(buffer),
            ],
        )?;
        #[cfg(feature = "defmt")]
        {
            trace!("I2C Read from {:#x}: {=[u8]:#x}", address, buffer);
        }
        Ok(())
    }
}

impl<SPI, E> SyncInterface for SpiInterface<SPI>
where
    SPI: spi::SpiDevice<Error = E>,
{
    type Error = E;

    fn write(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        #[cfg(feature = "defmt")]
        {
            trace!("SPI Write: {=[u8]:#x}", data);
        }
        self.spi.write(data)
    }

    fn write_read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        let mut dummy = [0u8; 1];
        self.spi.transaction(&mut [
            spi::Operation::Write(&[address]),
            spi::Operation::Read(&mut dummy),
            spi::Operation::Read(buffer),
        ])?;
        #[cfg(feature = "defmt")]
        {
            trace!("SPI Read from {:#x}: {=[u8]:#x}", address, buffer);
        }
        Ok(())
    }
}

pub(crate) trait SyncRegister<IFACE>
where
    IFACE: SyncInterface,
    Self: Register + Sized,
{
    fn read_register(iface: &mut IFACE) -> Result<Self, IFACE::Error> {
        let mut data = [0u8; 1];
        iface.write_read(Self::ADDRESS, &mut data)?;
        Ok(Self::from_u8(data[0]))
    }

    fn write_register(&self, iface: &mut IFACE) -> Result<(), IFACE::Error> {
        let mut buf = [0; 2];
        buf[0] = Self::ADDRESS;
        buf[1] = self.to_u8();
        iface.write(&buf)?;
        Ok(())
    }
}

macro_rules! impl_async_register {
    ($($reg:ty),+) => {
        $(
            impl<IFACE> SyncRegister<IFACE> for $reg
            where
                IFACE: SyncInterface,
            {}
        )+
    };
}

impl_async_register! {DeviceId, ErrorReg, StatusReg, EventReg, IrqStatus, IrqControl, PowerCtrl, OversamplingReg, OutputDataRate, IIRFilterConfig, Command}

/// Synchronous functions for the MMC5983MA sensor.
#[allow(async_fn_in_trait)]
pub trait SyncFunctions<I, E, D> {
    /// Initialize the device.
    fn init(&mut self) -> Result<CalibrationCoefficients, Bmp390Error<E>>;

    /// Resets the device.
    fn reset(&mut self) -> Result<(), Bmp390Error<E>>;

    /// Starts continuous measurement mode.
    fn start(&mut self) -> Result<(), Bmp390Error<E>>;

    /// Stops continuous measurement mode.
    fn stop(&mut self) -> Result<(), Bmp390Error<E>>;

    /// Reads a measurement from the device.
    fn measure(&mut self) -> Result<Measurement, Bmp390Error<E>>;
}

impl<IFACE, E, D> SyncFunctions<IFACE, E, D> for Bmp390<IFACE, D>
where
    IFACE: SyncInterface<Error = E>,
    D: DelayNs,
{
    fn init(&mut self) -> Result<CalibrationCoefficients, Bmp390Error<E>> {
        self.delay.delay_ms(2);
        // Check device ID
        let device_id = DeviceId::read_register(&mut self.iface)?;
        if !device_id.validate() {
            return Err(Bmp390Error::InvalidDevice);
        }
        // Read calibration coefficients
        let mut coeffs_buf = [0u8; 21];
        self.iface.write_read(NVM_PAR_T1_0, &mut coeffs_buf)?;
        self.reset()?;
        Ok(CalibrationCoefficients::from_registers(&coeffs_buf))
    }

    fn reset(&mut self) -> Result<(), Bmp390Error<E>> {
        // Reset device
        Command::SoftReset.write_register(&mut self.iface)?;
        self.delay.delay_ms(20);
        #[cfg(feature = "defmt")]
        {
            debug!("BMP390 reset complete");
            debug!(
                "IRQ Control [{=u8:#x}]: {=u8:#b}",
                IrqControl::ADDRESS,
                self.config.irq_control.to_u8()
            );
            debug!(
                "Filter Config [{=u8:#x}]: {=u8:#b}",
                IIRFilterConfig::ADDRESS,
                self.config.filtercfg.to_u8()
            );
            debug!(
                "ODR [{=u8:#x}]: {=u8:#b}",
                OutputDataRate::ADDRESS,
                self.config.odr.to_u8()
            );
            debug!(
                "Oversampling [{=u8:#x}]: {=u8:#b}",
                OversamplingReg::ADDRESS,
                self.config.oversamp_config().to_u8()
            );
        }
        PowerCtrl::new()
            .with_mode(self.config.sensor_mode)
            .with_press_en(true)
            .with_temp_en(true)
            .write_register(&mut self.iface)?;
        let err = ErrorReg::read_register(&mut self.iface)?;
        if err.fatal() {
            #[cfg(feature = "defmt")]
            {
                error!("Fatal error during BMP390 reset: Setting normal mode");
            }
            return Err(Bmp390Error::FatalError);
        } else if err.configuration() {
            #[cfg(feature = "defmt")]
            {
                error!("Configuration error during BMP390 reset: Setting normal mode");
            }
            return Err(Bmp390Error::InvalidConfiguration);
        } else if err.command() {
            #[cfg(feature = "defmt")]
            {
                error!("Command error during BMP390 reset: Setting normal mode");
            }
            return Err(Bmp390Error::InvalidCommand);
        }
        self.config.irq_control.write_register(&mut self.iface)?;
        let err = ErrorReg::read_register(&mut self.iface)?;
        if err.fatal() {
            #[cfg(feature = "defmt")]
            {
                error!("Fatal error during BMP390 reset: Writing IRQ control config");
            }
            return Err(Bmp390Error::FatalError);
        } else if err.configuration() {
            #[cfg(feature = "defmt")]
            {
                error!("Configuration error during BMP390 reset: Writing IRQ control config");
            }
            return Err(Bmp390Error::InvalidConfiguration);
        } else if err.command() {
            #[cfg(feature = "defmt")]
            {
                error!("Command error during BMP390 reset: Writing IRQ control config");
            }
            return Err(Bmp390Error::InvalidCommand);
        }
        self.config.filtercfg.write_register(&mut self.iface)?;
        let err = ErrorReg::read_register(&mut self.iface)?;
        if err.fatal() {
            #[cfg(feature = "defmt")]
            {
                error!("Fatal error during BMP390 reset: Writing filter config");
            }
            return Err(Bmp390Error::FatalError);
        } else if err.configuration() {
            #[cfg(feature = "defmt")]
            {
                error!("Configuration error during BMP390 reset: Writing filter config");
            }
            return Err(Bmp390Error::InvalidConfiguration);
        } else if err.command() {
            #[cfg(feature = "defmt")]
            {
                error!("Command error during BMP390 reset: Writing filter config");
            }
            return Err(Bmp390Error::InvalidCommand);
        }
        self.config.odr.write_register(&mut self.iface)?;
        let err = ErrorReg::read_register(&mut self.iface)?;
        if err.fatal() {
            #[cfg(feature = "defmt")]
            {
                error!("Fatal error during BMP390 reset: Writing ODR config");
            }
            return Err(Bmp390Error::FatalError);
        } else if err.configuration() {
            #[cfg(feature = "defmt")]
            {
                error!("Configuration error during BMP390 reset: Writing ODR config");
            }
            return Err(Bmp390Error::InvalidConfiguration);
        } else if err.command() {
            #[cfg(feature = "defmt")]
            {
                error!("Command error during BMP390 reset: Writing ODR config");
            }
            return Err(Bmp390Error::InvalidCommand);
        }
        self.config
            .oversamp_config()
            .write_register(&mut self.iface)?;
        let error = ErrorReg::read_register(&mut self.iface)?;
        if error.fatal() {
            #[cfg(feature = "defmt")]
            {
                error!("Fatal error during BMP390 reset: Writing oversampling config");
            }
            Err(Bmp390Error::FatalError)
        } else if error.configuration() {
            #[cfg(feature = "defmt")]
            {
                error!("Configuration error during BMP390 reset: Writing oversampling config");
            }
            Err(Bmp390Error::InvalidConfiguration)
        } else if error.command() {
            #[cfg(feature = "defmt")]
            {
                error!("Command error during BMP390 reset: Writing oversampling config");
            }
            Err(Bmp390Error::InvalidCommand)
        } else {
            Ok(())
        }
    }

    fn start(&mut self) -> Result<(), Bmp390Error<E>> {
        if self.running {
            return Ok(());
        }
        self.config.power_ctrl().write_register(&mut self.iface)?;
        #[cfg(feature = "defmt")]
        {
            info!(
                "Power Control [{=u8:#x}]: {=u8:#b}",
                PowerCtrl::ADDRESS,
                self.config.power_ctrl().to_u8()
            );
        }
        let error = ErrorReg::read_register(&mut self.iface)?;
        #[cfg(feature = "defmt")]
        {
            if error.to_u8() != 0 {
                warn!(
                    "Error Register [{=u8:#x}]: {=u8:#b}",
                    ErrorReg::ADDRESS,
                    error.to_u8()
                );
            } else {
                info!(
                    "Error Register [{=u8:#x}]: {=u8:#b}",
                    ErrorReg::ADDRESS,
                    error.to_u8()
                );
            }
        }
        if error.fatal() {
            return Err(Bmp390Error::FatalError);
        } else if error.configuration() {
            return Err(Bmp390Error::InvalidConfiguration);
        } else if error.command() {
            return Err(Bmp390Error::InvalidCommand);
        }
        self.running = true;
        Ok(())
    }

    fn stop(&mut self) -> Result<(), Bmp390Error<E>> {
        if !self.running {
            return Ok(());
        }
        // Disable sensors
        PowerCtrl::new()
            .with_mode(SensorMode::Sleep)
            .with_press_en(false)
            .with_temp_en(false)
            .write_register(&mut self.iface)?;
        let error = ErrorReg::read_register(&mut self.iface)?;
        if error.fatal() {
            return Err(Bmp390Error::FatalError);
        } else if error.configuration() {
            return Err(Bmp390Error::InvalidConfiguration);
        } else if error.command() {
            return Err(Bmp390Error::InvalidCommand);
        }
        self.running = false;
        Ok(())
    }

    fn measure(&mut self) -> Result<Measurement, Bmp390Error<E>> {
        if !self.running {
            return Err(Bmp390Error::NotReady);
        }
        let status = if !self.config.irq_control.drdy() {
            let mut ctr = MAX_LOOPS;
            loop {
                let status = StatusReg::read_register(&mut self.iface)?;
                if status.press_drdy() || status.temp_drdy() {
                    break status;
                }
                self.delay.delay_us(self.config.min_delay_us);
                ctr -= 1;
                if ctr == 0 {
                    return Err(Bmp390Error::NoDataAvailable);
                }
            }
        } else {
            StatusReg::read_register(&mut self.iface)?
        };
        #[cfg(feature = "defmt")]
        {
            trace!("Status: {=u8:#x}", status.to_u8());
        }
        let read = status.temp_drdy() || status.press_drdy();
        if !read {
            return Err(Bmp390Error::NoDataAvailable);
        }
        let mut buf = [0u8; 6];
        self.iface.write_read(PRESSURE_DATA_ADDR, &mut buf)?;
        let pressure_data = if status.press_drdy() {
            let meas = u32::from_le_bytes([buf[0], buf[1], buf[2], 0]);
            Some(meas)
        } else {
            None
        };
        let temp_data = if status.temp_drdy() {
            let meas = u32::from_le_bytes([buf[3], buf[4], buf[5], 0]);
            Some(meas)
        } else {
            None
        };
        Ok(Measurement {
            pressure: pressure_data,
            temperature: temp_data,
        })
    }
}
