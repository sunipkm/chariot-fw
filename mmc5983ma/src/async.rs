#[cfg(feature = "defmt")]
use defmt::{debug, info};

use crate::{
    interface::{I2cInterface, Interface, SpiInterface},
    registers::{
        AnalogControl, DigitalControl, MeasurementTriggerControl, ProductId, Register,
        StatusRegister, XYZOut2, MMC5983_DEVICE_ID,
    },
    ContinuousMeasurementFreq, MagMeasurementRaw, Mmc5983, Mmc5983Error, TempMeasurementRaw,
    MAX_LOOPS,
};
use embedded_hal_async::{delay::DelayNs, i2c, spi};

pub(crate) trait AsyncInterface: Interface {
    type Error;
    async fn write(&mut self, data: &[u8]) -> Result<(), Self::Error>;
    async fn write_read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error>;
}

impl<I2C, E> AsyncInterface for I2cInterface<I2C>
where
    I2C: i2c::I2c<Error = E>,
{
    type Error = E;

    async fn write(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        self.i2c.write(self.address, data).await
    }

    async fn write_read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.write_read(self.address, &[address], buffer).await
    }
}

impl<SPI, E> AsyncInterface for SpiInterface<SPI>
where
    SPI: spi::SpiDevice<Error = E>,
{
    type Error = E;

    async fn write(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        self.spi.write(data).await
    }

    async fn write_read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.spi
            .transaction(&mut [
                spi::Operation::Write(&[address]),
                spi::Operation::Read(buffer),
            ])
            .await
    }
}

pub(crate) trait AsyncRegister<IFACE>
where
    IFACE: AsyncInterface,
    Self: Register + Sized,
{
    async fn read_register(iface: &mut IFACE) -> Result<Self, IFACE::Error> {
        let mut data = [0u8];
        iface.write_read(Self::ADDRESS, &mut data).await?;
        Ok(Self::from_u8(data[0]))
    }

    async fn write_register(&self, iface: &mut IFACE) -> Result<(), IFACE::Error> {
        iface.write(&[Self::ADDRESS, self.to_u8()]).await?;
        Ok(())
    }
}

impl<IFACE> AsyncRegister<IFACE> for ProductId where IFACE: AsyncInterface {}
impl<IFACE> AsyncRegister<IFACE> for AnalogControl where IFACE: AsyncInterface {}
impl<IFACE> AsyncRegister<IFACE> for DigitalControl where IFACE: AsyncInterface {}
impl<IFACE> AsyncRegister<IFACE> for MeasurementTriggerControl where IFACE: AsyncInterface {}
impl<IFACE> AsyncRegister<IFACE> for StatusRegister where IFACE: AsyncInterface {}
impl<IFACE> AsyncRegister<IFACE> for XYZOut2 where IFACE: AsyncInterface {}

/// Synchronous functions for the MMC5983MA sensor.
#[allow(async_fn_in_trait)]
pub trait AsyncFunctions<I, E, D> {
    /// Initialize the device.
    async fn init(&mut self) -> Result<(), Mmc5983Error<E>>;

    /// Resets the device.
    async fn reset(&mut self) -> Result<(), Mmc5983Error<E>>;

    /// Starts continuous measurement mode.
    async fn start(&mut self) -> Result<(), Mmc5983Error<E>>;

    /// Stops continuous measurement mode.
    async fn stop(&mut self) -> Result<(), Mmc5983Error<E>>;

    /// Removes the soft iron offset using the SET/RESET method.
    async fn remove_offset(&mut self) -> Result<(), Mmc5983Error<E>>;

    /// Gets a raw magnetometer measurement from the device.
    async fn get_mag(&mut self) -> Result<MagMeasurementRaw, Mmc5983Error<E>>;

    /// Gets a raw temperature measurement from the device.
    async fn get_temp(&mut self) -> Result<TempMeasurementRaw, Mmc5983Error<E>>;
}

impl<IFACE, E, D> AsyncFunctions<IFACE, E, D> for Mmc5983<IFACE, D>
where
    IFACE: AsyncInterface<Error = E>,
    D: DelayNs,
{
    async fn reset(&mut self) -> Result<(), Mmc5983Error<E>> {
        let (mut analogctrl, measctrl, mut digitalctrl) = self.config.to_registers();
        analogctrl.set_sw_rst(true);
        analogctrl.write_register(&mut self.iface).await?;
        self.delay.delay_ms(10).await;
        analogctrl.set_sw_rst(false);
        analogctrl.write_register(&mut self.iface).await?;
        self.remove_offset().await?;
        digitalctrl.set_cm_freq(0);
        digitalctrl.set_cmm_en(false);
        digitalctrl.write_register(&mut self.iface).await?;
        measctrl.write_register(&mut self.iface).await?;
        Ok(())
    }

    async fn get_mag(&mut self) -> Result<MagMeasurementRaw, Mmc5983Error<E>> {
        if !self.running {
            let (_, mut measctrl, _) = self.config.to_registers();
            measctrl.set_tm_m(true);
            measctrl.write_register(&mut self.iface).await?;
            self.delay.delay_us(self.config.bandwidth.delay_us()).await;
        }
        let mut status = StatusRegister::read_register(&mut self.iface).await?;
        if status.magmeas_done() {
            status.set_magmeas_done(true);
            let mut data = [0u8; 7];
            self.iface.write_read(0x0, &mut data).await?;
            status.write_register(&mut self.iface).await?;
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

    async fn get_temp(&mut self) -> Result<TempMeasurementRaw, Mmc5983Error<E>> {
        let running = self.running;
        if running {
            self.stop().await?;
        }
        let (_, mut measctrl, _) = self.config.to_registers();
        measctrl.set_tm_t(true);
        measctrl.write_register(&mut self.iface).await?;
        let temp = {
            let mut ctr = 0;
            loop {
                let mut status = StatusRegister::read_register(&mut self.iface).await?;
                #[cfg(feature = "defmt")]
                debug!(
                    "{}> Status [0x{:02X}]: 0x{:08b}",
                    ctr,
                    StatusRegister::ADDRESS,
                    status.to_u8()
                );
                if status.tmeas_done() {
                    let mut data = [0u8; 1];
                    self.iface.write_read(0x07, &mut data).await?;
                    #[cfg(feature = "defmt")]
                    debug!("Temp raw data: {=[u8]:02x}", data);
                    status.set_tmeas_done(true);
                    status.write_register(&mut self.iface).await?;
                    break Some(TempMeasurementRaw(data[0]));
                }
                self.delay.delay_ms(1).await;
                ctr += 1;
                if ctr >= MAX_LOOPS {
                    #[cfg(feature = "defmt")]
                    info!("Temperature measurement: timed out");
                    break None;
                }
            }
        };
        if running {
            self.start().await?;
        }
        temp.ok_or(Mmc5983Error::NotReady)
    }

    async fn remove_offset(&mut self) -> Result<(), Mmc5983Error<E>> {
        let running = self.running;
        if running {
            self.stop().await?;
        }
        let old_calib = self.ofst.take();
        let (_, mut measctrl, _) = self.config.to_registers();
        measctrl.set_m_set(true);
        measctrl.write_register(&mut self.iface).await?;
        self.delay.delay_ms(10).await;
        measctrl.set_tm_m(true);
        measctrl.write_register(&mut self.iface).await?;
        let mag_set = {
            let mut ctr = 0;
            loop {
                let mut status = StatusRegister::read_register(&mut self.iface).await?;
                if status.magmeas_done() {
                    let mag_set = self.get_mag().await?;
                    status.set_magmeas_done(true);
                    status.write_register(&mut self.iface).await?;
                    break Some(mag_set);
                }
                self.delay.delay_ms(1).await;
                ctr += 1;
                if ctr >= MAX_LOOPS {
                    break None;
                }
            }
        };
        measctrl.set_m_set(false);
        measctrl.set_m_reset(true);
        measctrl.write_register(&mut self.iface).await?;
        self.delay.delay_ms(10).await;
        let mag_reset = {
            let mut ctr = 0;
            loop {
                let mut status = StatusRegister::read_register(&mut self.iface).await?;
                if status.magmeas_done() {
                    let mag_reset = self.get_mag().await?;
                    status.set_magmeas_done(true);
                    status.write_register(&mut self.iface).await?;
                    break Some(mag_reset);
                }
                self.delay.delay_ms(1).await;
                ctr += 1;
                if ctr >= MAX_LOOPS {
                    break None;
                }
            }
        };
        measctrl.set_m_reset(false);
        measctrl.write_register(&mut self.iface).await?;
        self.delay.delay_ms(10).await;
        if let (Some(mag_set), Some(mag_reset)) = (mag_set, mag_reset) {
            self.update_offsets(mag_set, mag_reset);
        } else {
            #[cfg(feature = "defmt")]
            info!("Set/Reset measurements: timed out");
            self.ofst = old_calib;
        }
        if running {
            self.start().await?;
        }
        Ok(())
    }

    async fn start(&mut self) -> Result<(), Mmc5983Error<E>> {
        if self.running {
            return Ok(());
        }
        if self.config.frequency != ContinuousMeasurementFreq::Off {
            let (_, _, mut digitalctrl) = self.config.to_registers();
            digitalctrl.set_cmm_en(true);
            digitalctrl.write_register(&mut self.iface).await?;
            self.running = true;
            Ok(())
        } else {
            Err(Mmc5983Error::InvalidConfig)
        }
    }

    async fn stop(&mut self) -> Result<(), Mmc5983Error<E>> {
        if !self.running {
            return Ok(());
        }
        if self.config.frequency != ContinuousMeasurementFreq::Off {
            let (_, _, mut digitalctrl) = self.config.to_registers();
            digitalctrl.set_cmm_en(true);
            digitalctrl.write_register(&mut self.iface).await?;
            digitalctrl.set_cmm_en(false);
            digitalctrl.set_cm_freq(0);
            digitalctrl.write_register(&mut self.iface).await?;
            self.running = false;
            Ok(())
        } else {
            Err(Mmc5983Error::InvalidConfig)
        }
    }

    async fn init(&mut self) -> Result<(), Mmc5983Error<E>> {
        #[cfg(feature = "defmt")]
        info!("Reading device ID...");
        let dev_id = ProductId::read_register(&mut self.iface).await?;
        #[cfg(feature = "defmt")]
        info!("Device ID: {:x}", dev_id.to_u8());
        if dev_id.to_u8() != MMC5983_DEVICE_ID {
            return Err(Mmc5983Error::InvalidDevice);
        }
        self.reset().await
    }
}
