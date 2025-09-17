#[cfg(feature = "defmt")]
use defmt::info;
use embedded_hal::{delay::DelayNs, i2c::I2c};

use crate::{
    config::Mmc5983Config,
    registers::{
        AnalogControl, DigitalControl, MeasurementTriggerControl, ProductId, Register,
        StatusRegister, XYZOut2, MMC5983_DEVICE_ID,
    },
    MagMeasurementRaw, Mmc5983, Mmc5983Error, TempMeasurementRaw, MAX_LOOPS,
};

pub(crate) trait SyncRegister<I2C>
where
    I2C: I2c,
    Self: Register + Sized,
{
    fn read_register(address: u8, i2c: &mut I2C) -> Result<Self, I2C::Error> {
        let mut data = [0u8];
        i2c.write_read(address, &[Self::ADDRESS], &mut data)?;
        Ok(Self::from_u8(data[0]))
    }

    fn write_register(&self, address: u8, i2c: &mut I2C) -> Result<(), I2C::Error> {
        i2c.write(address, &[Self::ADDRESS, self.to_u8()])?;
        Ok(())
    }
}

impl<I2C> SyncRegister<I2C> for ProductId where I2C: I2c {}
impl<I2C> SyncRegister<I2C> for AnalogControl where I2C: I2c {}
impl<I2C> SyncRegister<I2C> for DigitalControl where I2C: I2c {}
impl<I2C> SyncRegister<I2C> for MeasurementTriggerControl where I2C: I2c {}
impl<I2C> SyncRegister<I2C> for StatusRegister where I2C: I2c {}
impl<I2C> SyncRegister<I2C> for XYZOut2 where I2C: I2c {}

/// Synchronous functions for the MMC5983MA sensor.
pub trait SyncFunctions<I2C, I2CError, D> {
    /// Resets the device using the provided delay provider.
    /// # Arguments
    /// * `delay` - A delay provider to use for the reset.
    fn reset(&mut self, delay: &mut D) -> Result<(), Mmc5983Error<I2CError>>;

    /// Calibrates the sensor using the set/reset method.
    fn calibrate(&mut self, delay: &mut D) -> Result<(), Mmc5983Error<I2CError>>;

    /// Gets a raw magnetometer measurement from the device.
    fn get_mag(&mut self) -> Result<MagMeasurementRaw, Mmc5983Error<I2CError>>;

    /// Gets a raw temperature measurement from the device.
    fn get_temp(&mut self, delay: &mut D) -> Result<TempMeasurementRaw, Mmc5983Error<I2CError>>;
}

impl<I2C, D> Mmc5983<I2C, D>
where
    I2C: I2c,
    D: DelayNs,
{
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
    pub fn new_blocking(
        i2c: I2C,
        address: u8,
        config: Mmc5983Config,
        delay: &mut D,
    ) -> Result<Self, Mmc5983Error<I2C::Error>> {
        let mut mmc = Mmc5983 {
            i2c,
            address,
            config,
            ofst: None,
            _marker: core::marker::PhantomData,
        };
        // Initialize the sensor
        mmc.init(delay)?;
        Ok(mmc)
    }

    fn init(&mut self, delay: &mut D) -> Result<(), Mmc5983Error<I2C::Error>> {
        let dev_id = ProductId::read_register(self.address, &mut self.i2c)?;
        if dev_id.to_u8() != MMC5983_DEVICE_ID {
            return Err(Mmc5983Error::InvalidDevice);
        }
        self.reset(delay)
    }
}

impl<I2C, I2CError, D> SyncFunctions<I2C, I2CError, D> for Mmc5983<I2C, D>
where
    I2C: I2c<Error = I2CError>,
    D: DelayNs,
{
    fn reset(&mut self, delay: &mut D) -> Result<(), Mmc5983Error<I2C::Error>> {
        let mut analogctrl = AnalogControl::read_register(self.address, &mut self.i2c)?;
        let mut digitalctrl = DigitalControl::read_register(self.address, &mut self.i2c)?;
        let mut measctrl = MeasurementTriggerControl::read_register(self.address, &mut self.i2c)?;
        self.config
            .to_registers(&mut analogctrl, &mut digitalctrl, &mut measctrl);
        analogctrl.set_sw_rst(true);
        analogctrl.write_register(self.address, &mut self.i2c)?;
        delay.delay_ms(10);
        analogctrl.set_sw_rst(false);
        analogctrl.write_register(self.address, &mut self.i2c)?;
        digitalctrl.write_register(self.address, &mut self.i2c)?;
        measctrl.write_register(self.address, &mut self.i2c)?;
        Ok(())
    }

    fn get_mag(&mut self) -> Result<MagMeasurementRaw, Mmc5983Error<I2CError>> {
        let mut status = StatusRegister::read_register(self.address, &mut self.i2c)?;
        if status.magmeas_done() {
            status.set_magmeas_done(true);
            let mut data = [0u8; 7];
            self.i2c.write_read(self.address, &[0x00], &mut data)?;
            status.write_register(self.address, &mut self.i2c)?;
            Ok(MagMeasurementRaw::from(data))
        } else {
            Err(Mmc5983Error::NotReady)
        }
    }

    fn get_temp(&mut self, delay: &mut D) -> Result<TempMeasurementRaw, Mmc5983Error<I2CError>> {
        let mut measctrl = MeasurementTriggerControl::read_register(self.address, &mut self.i2c)?;
        measctrl.set_tm_t(true);
        measctrl.write_register(self.address, &mut self.i2c)?;
        for _ in 0..MAX_LOOPS {
            let mut status = StatusRegister::read_register(self.address, &mut self.i2c)?;
            if status.tmeas_done() {
                status.set_tmeas_done(true);
                status.write_register(self.address, &mut self.i2c)?;
                let mut data = [0u8; 1];
                self.i2c.write_read(self.address, &[0x07], &mut data)?;
                return Ok(TempMeasurementRaw(data[0]));
            }
            delay.delay_ms(1);
        }
        Err(Mmc5983Error::NotReady)
    }

    fn calibrate(&mut self, delay: &mut D) -> Result<(), Mmc5983Error<I2CError>> {
let old_calib = self.ofst.take();
        let mut digitalctrl = DigitalControl::read_register(self.address, &mut self.i2c)?;
        let mut measctrl =
            MeasurementTriggerControl::read_register(self.address, &mut self.i2c)?;
        let measurement_en = digitalctrl.cmm_en();
        digitalctrl.set_cmm_en(false);
        digitalctrl
            .write_register(self.address, &mut self.i2c)
            ?;
        measctrl.set_m_set(true);
        measctrl.write_register(self.address, &mut self.i2c)?;
        delay.delay_ms(10);
        measctrl.set_tm_m(true);
        measctrl.write_register(self.address, &mut self.i2c)?;
        let mag_set = {
            let mut ctr = 0;
            loop {
                let mut status = StatusRegister::read_register(self.address, &mut self.i2c)?;
                if status.magmeas_done() {
                    let mag_set = self.get_mag()?;
                    status.set_magmeas_done(true);
                    status.write_register(self.address, &mut self.i2c)?;
                    break Some(mag_set);
                }
                delay.delay_ms(1);
                ctr += 1;
                if ctr >= MAX_LOOPS {
                    break None;
                }
            }
        };
        measctrl.set_m_set(false);
        measctrl.set_m_reset(true);
        measctrl.write_register(self.address, &mut self.i2c)?;
        delay.delay_ms(10);
        let mag_reset = {
            let mut ctr = 0;
            loop {
                let mut status = StatusRegister::read_register(self.address, &mut self.i2c)?;
                if status.magmeas_done() {
                    let mag_reset = self.get_mag()?;
                    status.set_magmeas_done(true);
                    status.write_register(self.address, &mut self.i2c)?;
                    break Some(mag_reset);
                }
                delay.delay_ms(1);
                ctr += 1;
                if ctr >= MAX_LOOPS {
                    break None;
                }
            }
        };
        measctrl.set_m_reset(false);
        measctrl.write_register(self.address, &mut self.i2c)?;
        delay.delay_ms(10);
        if let (Some(mag_set), Some(mag_reset)) = (mag_set, mag_reset) {
            #[cfg(feature = "defmt")]
            info!(
                "Set/Reset measurements: set=({},{},{}) reset=({},{},{})",
                mag_set.x(),
                mag_set.y(),
                mag_set.z(),
                mag_reset.x(),
                mag_reset.y(),
                mag_reset.z()
            );
            let fx = (mag_set.x() + mag_reset.x()) / 2;
            let fy = (mag_set.y() + mag_reset.y()) / 2;
            let fz = (mag_set.z() + mag_reset.z()) / 2;
            #[cfg(feature = "defmt")]
            info!("Offsets: x={} y={} z={}", fx, fy, fz);
            self.ofst = Some(MagMeasurementRaw(
                (fx as u64 & 0x3FFFF) | ((fy as u64 & 0x3FFFF) << 18) | ((fz as u64 & 0x3FFFF) << 36),
            ));
        } else {
            #[cfg(feature = "defmt")]
            info!("Set/Reset measurements: timed out");
            self.ofst = old_calib;
        }
        digitalctrl.set_cmm_en(measurement_en);
        Ok(())
    }
}
