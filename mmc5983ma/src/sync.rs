#[cfg(feature = "defmt")]
use defmt::{debug, info};
use embedded_hal::{delay::DelayNs, i2c::I2c};

use crate::{
    config::Mmc5983Config,
    registers::{
        AnalogControl, DigitalControl, MeasurementTriggerControl, ProductId, Register,
        StatusRegister, XYZOut2, MMC5983_DEVICE_ID,
    },
    ContinuousMeasurementFreq, MagMeasurementRaw, Mmc5983, Mmc5983Error, TempMeasurementRaw,
    MAX_LOOPS,
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
    /// Resets the device.
    fn reset(&mut self) -> Result<(), Mmc5983Error<I2CError>>;

    /// Starts continuous measurement mode.
    fn start(&mut self) -> Result<(), Mmc5983Error<I2CError>>;

    /// Stops continuous measurement mode.
    fn stop(&mut self) -> Result<(), Mmc5983Error<I2CError>>;

    /// Removes the soft iron offset using the SET/RESET method.
    fn remove_offset(&mut self) -> Result<(), Mmc5983Error<I2CError>>;

    /// Gets a raw magnetometer measurement from the device.
    fn get_mag(&mut self) -> Result<MagMeasurementRaw, Mmc5983Error<I2CError>>;

    /// Gets a raw temperature measurement from the device.
    fn get_temp(&mut self) -> Result<TempMeasurementRaw, Mmc5983Error<I2CError>>;
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
        delay: D,
    ) -> Result<Self, Mmc5983Error<I2C::Error>> {
        let mut mmc = Mmc5983 {
            i2c,
            delay,
            address,
            config,
            ofst: None,
            running: false,
        };
        // Initialize the sensor
        mmc.init()?;
        Ok(mmc)
    }

    fn init(&mut self) -> Result<(), Mmc5983Error<I2C::Error>> {
        let dev_id = ProductId::read_register(self.address, &mut self.i2c)?;
        if dev_id.to_u8() != MMC5983_DEVICE_ID {
            return Err(Mmc5983Error::InvalidDevice);
        }
        self.reset()
    }
}

impl<I2C, I2CError, D> SyncFunctions<I2C, I2CError, D> for Mmc5983<I2C, D>
where
    I2C: I2c<Error = I2CError>,
    D: DelayNs,
{
    fn reset(&mut self) -> Result<(), Mmc5983Error<I2C::Error>> {
        let (mut analogctrl, measctrl, mut digitalctrl) = self.config.to_registers();
        analogctrl.set_sw_rst(true);
        analogctrl.write_register(self.address, &mut self.i2c)?;
        self.delay.delay_ms(10);
        analogctrl.set_sw_rst(false);
        analogctrl.write_register(self.address, &mut self.i2c)?;
        self.remove_offset()?;
        digitalctrl.set_cm_freq(0);
        digitalctrl.set_cmm_en(false);
        digitalctrl.write_register(self.address, &mut self.i2c)?;
        measctrl.write_register(self.address, &mut self.i2c)?;
        Ok(())
    }

    fn get_mag(&mut self) -> Result<MagMeasurementRaw, Mmc5983Error<I2CError>> {
        if !self.running {
            let (_, mut measctrl, _) = self.config.to_registers();
            measctrl.set_tm_m(true);
            measctrl.write_register(self.address, &mut self.i2c)?;
            self.delay.delay_us(self.config.bandwidth.delay_us());
        }
        let mut status = StatusRegister::read_register(self.address, &mut self.i2c)?;
        if status.magmeas_done() {
            status.set_magmeas_done(true);
            let mut data = [0u8; 7];
            self.i2c.write_read(self.address, &[0x00], &mut data)?;
            status.write_register(self.address, &mut self.i2c)?;
            let mut mag = MagMeasurementRaw::from(data);
            if let Some(ofst) = self.ofst {
                mag.set_x(mag.x().wrapping_sub(ofst.x()));
                mag.set_y(mag.y().wrapping_sub(ofst.y()));
                mag.set_z(mag.z().wrapping_sub(ofst.z()));
            }
            Ok(mag)
        } else {
            Err(Mmc5983Error::NotReady)
        }
    }

    fn get_temp(&mut self) -> Result<TempMeasurementRaw, Mmc5983Error<I2CError>> {
        let running = self.running;
        if running {
            self.stop()?;
        }
        let (_, mut measctrl, _) = self.config.to_registers();
        measctrl.set_tm_t(true);
        measctrl.write_register(self.address, &mut self.i2c)?;
        let temp = {
            let mut ctr = 0;
            loop {
                let mut status = StatusRegister::read_register(self.address, &mut self.i2c)?;
                #[cfg(feature = "defmt")]
                debug!(
                    "{}> Status [0x{:02X}]: 0x{:08b}",
                    ctr,
                    StatusRegister::ADDRESS,
                    status.to_u8()
                );
                if status.tmeas_done() {
                    let mut data = [0u8; 1];
                    self.i2c.write_read(self.address, &[0x07], &mut data)?;
                    #[cfg(feature = "defmt")]
                    debug!("Temp raw data: {=[u8]:02x}", data);
                    status.set_tmeas_done(true);
                    status.write_register(self.address, &mut self.i2c)?;
                    break Some(TempMeasurementRaw(data[0]));
                }
                self.delay.delay_ms(1);
                ctr += 1;
                if ctr >= MAX_LOOPS {
                    #[cfg(feature = "defmt")]
                    info!("Temperature measurement: timed out");
                    break None;
                }
            }
        };
        if running {
            self.start()?;
        }
        temp.ok_or(Mmc5983Error::NotReady)
    }

    fn remove_offset(&mut self) -> Result<(), Mmc5983Error<I2CError>> {
        let running = self.running;
        if running {
            self.stop()?;
        }
        let old_calib = self.ofst.take();
        let (_, mut measctrl, _) = self.config.to_registers();
        measctrl.set_m_set(true);
        measctrl.write_register(self.address, &mut self.i2c)?;
        self.delay.delay_ms(10);
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
                self.delay.delay_ms(1);
                ctr += 1;
                if ctr >= MAX_LOOPS {
                    break None;
                }
            }
        };
        measctrl.set_m_set(false);
        measctrl.set_m_reset(true);
        measctrl.write_register(self.address, &mut self.i2c)?;
        self.delay.delay_ms(10);
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
                self.delay.delay_ms(1);
                ctr += 1;
                if ctr >= MAX_LOOPS {
                    break None;
                }
            }
        };
        measctrl.set_m_reset(false);
        measctrl.write_register(self.address, &mut self.i2c)?;
        self.delay.delay_ms(10);
        if let (Some(mag_set), Some(mag_reset)) = (mag_set, mag_reset) {
            self.update_offsets(mag_set, mag_reset);
        } else {
            #[cfg(feature = "defmt")]
            info!("Set/Reset measurements: timed out");
            self.ofst = old_calib;
        }
        if running {
            self.start()?;
        }
        Ok(())
    }

    fn start(&mut self) -> Result<(), Mmc5983Error<I2CError>> {
        if self.running {
            return Ok(());
        }
        if self.config.frequency != ContinuousMeasurementFreq::Off {
            let (_, _, mut digitalctrl) = self.config.to_registers();
            digitalctrl.set_cmm_en(true);
            digitalctrl.write_register(self.address, &mut self.i2c)?;
            self.running = true;
            Ok(())
        } else {
            Err(Mmc5983Error::InvalidConfig)
        }
    }

    fn stop(&mut self) -> Result<(), Mmc5983Error<I2CError>> {
        if !self.running {
            return Ok(());
        }
        if self.config.frequency != ContinuousMeasurementFreq::Off {
            let (_, _, mut digitalctrl) = self.config.to_registers();
            digitalctrl.set_cmm_en(true);
            digitalctrl.write_register(self.address, &mut self.i2c)?;
            digitalctrl.set_cmm_en(false);
            digitalctrl.set_cm_freq(0);
            digitalctrl.write_register(self.address, &mut self.i2c)?;
            self.running = false;
            Ok(())
        } else {
            Err(Mmc5983Error::InvalidConfig)
        }
    }
}
