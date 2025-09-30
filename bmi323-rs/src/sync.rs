#[cfg(feature = "defmt")]
use defmt::{debug, info, trace, warn};

use crate::{
    interface::{I2cInterface, Interface, SpiInterface},
    registers::{
        AccelConfig, Command, DeviceId, ErrorReg, FeatEngAddr, FeatEngConfig, FeatEngIo0,
        FeatEngIoStat, FeatureDataStatus, FeatureDataTx, FeatureEngineControl, FeatureEngineStatus,
        FeatureInterruptMap, FeatureIo1Error, FifoConfig, FifoCtrl, FifoFillLevel, FifoWatermark,
        GyroConfig, GyroSelfCalibSelect, I2cWatchdogConfig, IbiStatus, Int1Status, Int2Status,
        IntLatchConfig, IntPinConfig, IoPadStrength, Register, SaturationReg, SensorInterruptMap,
        StatusReg, ACCEL_DATA_ADDR,
    },
    AccelMode, Bmi323, Bmi323Error, GyroMode, Measurement, MeasurementRaw3D, SelfCalibrateType,
    MAX_LOOPS,
};
use embedded_hal::{delay::DelayNs, i2c, spi};

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
        let mut dummy = [0u8; 2];
        self.i2c.transaction(
            self.address,
            &mut [
                i2c::Operation::Write(&[address]),
                i2c::Operation::Read(&mut dummy),
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
        let mut data = [0u8; 2];
        iface.write_read(Self::ADDRESS, &mut data)?;
        Ok(Self::from_u16(u16::from_le_bytes(data)))
    }

    fn write_register(&self, iface: &mut IFACE) -> Result<(), IFACE::Error> {
        let mut buf = [0; 3];
        buf[0] = Self::ADDRESS;
        buf[1..].copy_from_slice(&self.to_u16().to_le_bytes());
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

impl_async_register! {ErrorReg, StatusReg, SaturationReg, Int1Status, Int2Status, IbiStatus, FeatEngConfig, FeatEngIo0, FeatEngIoStat, FifoFillLevel, AccelConfig, GyroConfig, FifoWatermark, FifoConfig, FifoCtrl, IntPinConfig, IntLatchConfig, FeatureInterruptMap, SensorInterruptMap, FeatureEngineControl, FeatEngAddr, FeatureDataTx, FeatureDataStatus, FeatureEngineStatus, IoPadStrength, I2cWatchdogConfig, Command, DeviceId, GyroSelfCalibSelect}

/// Synchronous functions for the BMI323 sensor.
#[allow(async_fn_in_trait)]
pub trait SyncFunctions<I, E, D> {
    /// Initialize the device.
    fn init(&mut self) -> Result<(), Bmi323Error<E>>;

    /// Resets the device.
    fn reset(&mut self) -> Result<(), Bmi323Error<E>>;

    /// Starts continuous measurement mode.
    fn start(&mut self) -> Result<(), Bmi323Error<E>>;

    /// Stops continuous measurement mode.
    fn stop(&mut self) -> Result<(), Bmi323Error<E>>;

    /// Reads a measurement from the device.
    fn measure(&mut self) -> Result<Measurement, Bmi323Error<E>>;

    /// Calibrate the gyroscope.
    fn calibrate(&mut self, what: SelfCalibrateType) -> Result<(), Bmi323Error<E>>;

    /// Check if calibration is applied.
    fn is_calibration_applied(&mut self) -> Result<bool, Bmi323Error<E>>;
}

impl<IFACE, E, D> SyncFunctions<IFACE, E, D> for Bmi323<IFACE, D>
where
    IFACE: SyncInterface<Error = E>,
    D: DelayNs,
{
    fn init(&mut self) -> Result<(), Bmi323Error<E>> {
        // Check device ID
        let device_id = DeviceId::read_register(&mut self.iface)?;
        if !device_id.validate() {
            return Err(Bmi323Error::InvalidDevice);
        }
        self.reset()
    }

    fn reset(&mut self) -> Result<(), Bmi323Error<E>> {
        // Reset device
        Command::SoftReset.write_register(&mut self.iface)?;
        self.delay.delay_ms(20);

        // Write configuration
        let (
            feature_engine_control,
            fifo_config,
            sensor_interrupt_map,
            i2c_watchdog_config,
            int_pin_config,
            accel_config,
            gyro_config,
        ) = self.config.get_registers();

        feature_engine_control.write_register(&mut self.iface)?;
        fifo_config.write_register(&mut self.iface)?;
        sensor_interrupt_map.write_register(&mut self.iface)?;
        i2c_watchdog_config.write_register(&mut self.iface)?;
        int_pin_config.write_register(&mut self.iface)?;
        accel_config.write_register(&mut self.iface)?;
        gyro_config.write_register(&mut self.iface)?;

        let error = ErrorReg::read_register(&mut self.iface)?;
        if error.fatal() {
            Err(Bmi323Error::FatalError)
        } else if error.accel_conf() {
            Err(Bmi323Error::InvalidAccelConfig)
        } else if error.gyro_conf() {
            Err(Bmi323Error::InvalidGyroConfig)
        } else {
            Ok(())
        }
    }

    fn start(&mut self) -> Result<(), Bmi323Error<E>> {
        if self.running {
            return Ok(());
        }
        // Enable sensors
        let mut accel_config = AccelConfig::read_register(&mut self.iface)?;
        let mut gyro_config = GyroConfig::read_register(&mut self.iface)?;
        if self.config.sensors_enabled.is_accel_enabled() {
            accel_config.set_mode(self.config.accel_mode());
        }
        if self.config.sensors_enabled.is_gyro_enabled() {
            gyro_config.set_mode(self.config.gyro_mode());
        }
        accel_config.write_register(&mut self.iface)?;
        gyro_config.write_register(&mut self.iface)?;
        #[cfg(feature = "defmt")]
        {
            let accel_config = AccelConfig::read_register(&mut self.iface)?;
            let gyro_config = GyroConfig::read_register(&mut self.iface)?;
            debug!("Accel Config: {:#b}", accel_config.to_u16());
            debug!("Gyro Config: {:#b}", gyro_config.to_u16());
        }
        self.running = true;
        Ok(())
    }

    fn stop(&mut self) -> Result<(), Bmi323Error<E>> {
        if !self.running {
            return Ok(());
        }
        // Disable sensors
        let mut accel_config = AccelConfig::read_register(&mut self.iface)?;
        let mut gyro_config = GyroConfig::read_register(&mut self.iface)?;
        accel_config.set_mode(AccelMode::Off);
        gyro_config.set_mode(GyroMode::Off);
        accel_config.write_register(&mut self.iface)?;
        gyro_config.write_register(&mut self.iface)?;
        self.running = false;
        Ok(())
    }

    fn measure(&mut self) -> Result<Measurement, Bmi323Error<E>> {
        if !self.running {
            return Err(Bmi323Error::NotReady);
        }
        let status = if !self.config.irq_enabled {
            let mut ctr = MAX_LOOPS;
            loop {
                let status = StatusReg::read_register(&mut self.iface)?;
                if status.drdy_acc() || status.drdy_gyr() || status.drdy_temp() {
                    break status;
                }
                self.delay.delay_us(self.config.min_delay_us);
                ctr -= 1;
                if ctr == 0 {
                    return Err(Bmi323Error::NoDataAvailable);
                }
            }
        } else {
            StatusReg::read_register(&mut self.iface)?
        };
        #[cfg(feature = "defmt")]
        {
            trace!("Status: {=u16:#x}", status.to_u16());
        }
        let read = status.drdy_acc() || status.drdy_gyr() || status.drdy_temp();
        if !read {
            return Err(Bmi323Error::NoDataAvailable);
        }
        let mut buf = [0u8; 18];
        self.iface.write_read(ACCEL_DATA_ADDR, &mut buf)?;
        let accel_data = if status.drdy_acc() {
            let meas = MeasurementRaw3D::new()
                .with_x(i16::from_le_bytes([buf[0], buf[1]]))
                .with_y(i16::from_le_bytes([buf[2], buf[3]]))
                .with_z(i16::from_le_bytes([buf[4], buf[5]]))
                .with_kind(crate::Measurement3DKind::Accel(self.config.accel_range()));
            Some(meas)
        } else {
            None
        };
        let gyro_data = if status.drdy_gyr() {
            let meas = MeasurementRaw3D::new()
                .with_x(i16::from_le_bytes([buf[6], buf[7]]))
                .with_y(i16::from_le_bytes([buf[8], buf[9]]))
                .with_z(i16::from_le_bytes([buf[10], buf[11]]))
                .with_kind(crate::Measurement3DKind::Gyro(self.config.gyro_range()));
            Some(meas)
        } else {
            None
        };
        let temp_data = if status.drdy_temp() {
            Some(crate::TemperatureMeasurement(i16::from_le_bytes([
                buf[12], buf[13],
            ])))
        } else {
            None
        };
        let timestamp =
            crate::TimestampMeasurement(u32::from_le_bytes([buf[14], buf[15], buf[16], buf[17]]));
        Ok(Measurement {
            accel: accel_data,
            gyro: gyro_data,
            temp: temp_data,
            timestamp,
        })
    }

    fn calibrate(&mut self, what: SelfCalibrateType) -> Result<(), Bmi323Error<E>> {
        if self.running {
            return Err(Bmi323Error::Busy);
        }
        let feature_eng_stat = FeatEngIo0::read_register(&mut self.iface)?;
        if !(feature_eng_stat.errors() == FeatureIo1Error::Active
            || feature_eng_stat.errors() == FeatureIo1Error::Activated
            || feature_eng_stat.errors() == FeatureIo1Error::NoError)
        {
            return Err(Bmi323Error::RestartRequired);
        }
        let irq_config = IntPinConfig::new()
            .with_int1_output_en(false)
            .with_int2_output_en(false);
        let accel_cfg = AccelConfig::new()
            .with_odr(crate::OutputDataRate::Hz100)
            .with_range(crate::AccelRange::G8)
            .with_mode(AccelMode::HighPerformance);
        let gyro_cfg = GyroConfig::new()
            .with_odr(crate::OutputDataRate::Hz100)
            .with_range(crate::GyroRange::Dps2000)
            .with_mode(GyroMode::HighPerformance);
        irq_config.write_register(&mut self.iface)?;
        accel_cfg.write_register(&mut self.iface)?;
        gyro_cfg.write_register(&mut self.iface)?;
        GyroSelfCalibSelect::new()
            .with_sensitivity(what.sensitivity())
            .with_offset(what.offset())
            .with_apply(true)
            .write_register(&mut self.iface)?;
        self.delay.delay_ms(50);
        let errors = ErrorReg::read_register(&mut self.iface)?;
        if errors.fatal() || errors.accel_conf() || errors.gyro_conf() {
            return Err(Bmi323Error::FatalError);
        }
        #[cfg(feature = "defmt")]
        {
            warn!("Starting gyro calibration, keep the device still...");
        }
        Command::GyroSelfCalib.write_register(&mut self.iface)?;
        self.delay.delay_ms(350);
        #[cfg(feature = "defmt")]
        {
            info!("Gyro calibration in progress...");
            let feat_eng_io0 = FeatEngIo0::read_register(&mut self.iface)?;
            debug!("Feature Engine IO0: {=u16:#b}", feat_eng_io0.to_u16());
        }
        self.delay.delay_ms(80);
        let mut ctr = MAX_LOOPS;
        loop {
            let feat_eng_io0 = FeatEngIo0::read_register(&mut self.iface)?;
            #[cfg(feature = "defmt")]
            {
                debug!("Feature Engine IO0: {=u16:#b}", feat_eng_io0.to_u16());
            }
            if feat_eng_io0.self_proc_done() {
                break;
            }
            self.delay.delay_ms(5);
            ctr -= 1;
            if ctr == 0 {
                return Err(Bmi323Error::SelfCalTimedOut);
            }
        }
        #[cfg(feature = "defmt")]
        {
            info!("Gyro calibration complete.");
        }
        let (_, _, _, _, irq_config, gyro_cfg, accel_cfg) = self.config.get_registers();
        irq_config.write_register(&mut self.iface)?;
        gyro_cfg.write_register(&mut self.iface)?;
        accel_cfg.write_register(&mut self.iface)?;
        let error = ErrorReg::read_register(&mut self.iface)?;
        if error.sensor_fatal() {
            Err(Bmi323Error::FatalError)
        } else {
            Ok(())
        }
    }

    fn is_calibration_applied(&mut self) -> Result<bool, Bmi323Error<E>> {
        let gyro_self_calib = GyroSelfCalibSelect::read_register(&mut self.iface)?;
        Ok(gyro_self_calib.apply())
    }
}
