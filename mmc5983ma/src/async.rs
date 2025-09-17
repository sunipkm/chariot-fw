#![allow(dead_code)]
#[cfg(feature = "defmt")]
use defmt::info;

use crate::{
    config::Mmc5983Config,
    registers::{
        AnalogControl, DigitalControl, MeasurementTriggerControl, ProductId, Register,
        StatusRegister, XYZOut2, MMC5983_DEVICE_ID,
    },
    MagMeasurementRaw, Mmc5983, Mmc5983Error, TempMeasurementRaw, MAX_LOOPS,
};
use embedded_hal_async::{delay::DelayNs, i2c::I2c};

pub(crate) trait AsyncRegister<I2C>
where
    I2C: I2c,
    Self: Register + Sized,
{
    async fn read_register(address: u8, i2c: &mut I2C) -> Result<Self, I2C::Error> {
        let mut data = [0u8];
        i2c.write_read(address, &[Self::ADDRESS], &mut data).await?;
        Ok(Self::from_u8(data[0]))
    }

    async fn write_register(&self, address: u8, i2c: &mut I2C) -> Result<(), I2C::Error> {
        i2c.write(address, &[Self::ADDRESS, self.to_u8()]).await?;
        Ok(())
    }
}

impl<I2C> AsyncRegister<I2C> for ProductId where I2C: I2c {}
impl<I2C> AsyncRegister<I2C> for AnalogControl where I2C: I2c {}
impl<I2C> AsyncRegister<I2C> for DigitalControl where I2C: I2c {}
impl<I2C> AsyncRegister<I2C> for MeasurementTriggerControl where I2C: I2c {}
impl<I2C> AsyncRegister<I2C> for StatusRegister where I2C: I2c {}
impl<I2C> AsyncRegister<I2C> for XYZOut2 where I2C: I2c {}

/// Synchronous functions for the MMC5983MA sensor.
#[allow(async_fn_in_trait)]
pub trait AsyncFunctions<I2C, I2CError, D> {
    /// Resets the device using the provided delay provider.
    /// # Arguments
    /// * `delay` - A delay provider to use for the reset.
    async fn reset(&mut self, delay: &mut D) -> Result<(), Mmc5983Error<I2CError>>;

    /// Calibrates the sensor using the set/reset method.
    async fn calibrate(&mut self, delay: &mut D) -> Result<(), Mmc5983Error<I2CError>>;

    /// Gets a raw magnetometer measurement from the device.
    async fn get_mag(&mut self) -> Result<MagMeasurementRaw, Mmc5983Error<I2CError>>;

    /// Gets a raw temperature measurement from the device.
    async fn get_temp(
        &mut self,
        delay: &mut D,
    ) -> Result<TempMeasurementRaw, Mmc5983Error<I2CError>>;
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
    pub async fn new_async(
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
        mmc.init_async(delay).await?;
        Ok(mmc)
    }

    async fn init_async(&mut self, delay: &mut D) -> Result<(), Mmc5983Error<I2C::Error>> {
        #[cfg(feature = "defmt")]
        info!("Reading device ID...");
        let dev_id = ProductId::read_register(self.address, &mut self.i2c).await?;
        #[cfg(feature = "defmt")]
        info!("Device ID: {:x}", dev_id.to_u8());
        if dev_id.to_u8() != MMC5983_DEVICE_ID {
            return Err(Mmc5983Error::InvalidDevice);
        }
        self.reset(delay).await
    }
}

impl<I2C, I2CError, D> AsyncFunctions<I2C, I2CError, D> for Mmc5983<I2C, D>
where
    I2C: I2c<Error = I2CError>,
    D: DelayNs,
{
    async fn reset(&mut self, delay: &mut D) -> Result<(), Mmc5983Error<I2C::Error>> {
        let mut analogctrl = AnalogControl::read_register(self.address, &mut self.i2c).await?;
        let mut digitalctrl = DigitalControl::read_register(self.address, &mut self.i2c).await?;
        let mut measctrl =
            MeasurementTriggerControl::read_register(self.address, &mut self.i2c).await?;
        self.config
            .to_registers(&mut analogctrl, &mut digitalctrl, &mut measctrl);
        analogctrl.set_sw_rst(true);
        analogctrl
            .write_register(self.address, &mut self.i2c)
            .await?;
        delay.delay_ms(10).await;
        analogctrl.set_sw_rst(false);
        analogctrl
            .write_register(self.address, &mut self.i2c)
            .await?;
        self.calibrate(delay).await?;
        digitalctrl
            .write_register(self.address, &mut self.i2c)
            .await?;
        measctrl.write_register(self.address, &mut self.i2c).await?;
        Ok(())
    }

    async fn get_mag(&mut self) -> Result<MagMeasurementRaw, Mmc5983Error<I2CError>> {
        let mut status = StatusRegister::read_register(self.address, &mut self.i2c).await?;
        if status.magmeas_done() {
            status.set_magmeas_done(true);
            let mut data = [0u8; 7];
            self.i2c
                .write_read(self.address, &[0x00], &mut data)
                .await?;
            status.write_register(self.address, &mut self.i2c).await?;
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

    async fn get_temp(
        &mut self,
        delay: &mut D,
    ) -> Result<TempMeasurementRaw, Mmc5983Error<I2CError>> {
        let mut measctrl =
            MeasurementTriggerControl::read_register(self.address, &mut self.i2c).await?;
        measctrl.set_tm_t(true);
        measctrl.write_register(self.address, &mut self.i2c).await?;
        for _ in 0..MAX_LOOPS {
            let mut status = StatusRegister::read_register(self.address, &mut self.i2c).await?;
            if status.tmeas_done() {
                status.set_tmeas_done(true);
                status.write_register(self.address, &mut self.i2c).await?;
                let mut data = [0u8; 1];
                self.i2c
                    .write_read(self.address, &[0x07], &mut data)
                    .await?;
                return Ok(TempMeasurementRaw(data[0]));
            }
            delay.delay_ms(1).await;
        }
        Err(Mmc5983Error::NotReady)
    }

    async fn calibrate(&mut self, delay: &mut D) -> Result<(), Mmc5983Error<I2CError>> {
        let old_calib = self.ofst.take();
        let mut digitalctrl = DigitalControl::read_register(self.address, &mut self.i2c).await?;
        let mut measctrl =
            MeasurementTriggerControl::read_register(self.address, &mut self.i2c).await?;
        let measurement_en = digitalctrl.cmm_en();
        digitalctrl.set_cmm_en(false);
        digitalctrl
            .write_register(self.address, &mut self.i2c)
            .await?;
        measctrl.set_m_set(true);
        measctrl.write_register(self.address, &mut self.i2c).await?;
        delay.delay_ms(10).await;
        measctrl.set_tm_m(true);
        measctrl.write_register(self.address, &mut self.i2c).await?;
        let mag_set = {
            let mut ctr = 0;
            loop {
                let mut status = StatusRegister::read_register(self.address, &mut self.i2c).await?;
                if status.magmeas_done() {
                    let mag_set = self.get_mag().await?;
                    status.set_magmeas_done(true);
                    status.write_register(self.address, &mut self.i2c).await?;
                    break Some(mag_set);
                }
                delay.delay_ms(1).await;
                ctr += 1;
                if ctr >= MAX_LOOPS {
                    break None;
                }
            }
        };
        measctrl.set_m_set(false);
        measctrl.set_m_reset(true);
        measctrl.write_register(self.address, &mut self.i2c).await?;
        delay.delay_ms(10).await;
        let mag_reset = {
            let mut ctr = 0;
            loop {
                let mut status = StatusRegister::read_register(self.address, &mut self.i2c).await?;
                if status.magmeas_done() {
                    let mag_reset = self.get_mag().await?;
                    status.set_magmeas_done(true);
                    status.write_register(self.address, &mut self.i2c).await?;
                    break Some(mag_reset);
                }
                delay.delay_ms(1).await;
                ctr += 1;
                if ctr >= MAX_LOOPS {
                    break None;
                }
            }
        };
        measctrl.set_m_reset(false);
        measctrl.write_register(self.address, &mut self.i2c).await?;
        delay.delay_ms(10).await;
        if let (Some(mag_set), Some(mag_reset)) = (mag_set, mag_reset) {
            self.update_offsets(mag_set, mag_reset);
        } else {
            #[cfg(feature = "defmt")]
            info!("Set/Reset measurements: timed out");
            self.ofst = old_calib;
        }
        digitalctrl.set_cmm_en(measurement_en);
        Ok(())
    }
}
