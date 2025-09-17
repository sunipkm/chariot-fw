//! This example test the flash connected to the RP2040 chip.

#![no_std]
#![no_main]

use assign_resources::assign_resources;
use defmt::*;
use embassy_embedded_hal::shared_bus::asynch::i2c::{self, I2cDevice};
use embassy_executor::Spawner;
use embassy_rp::{
    Peri, bind_interrupts,
    flash::{Async as FlashAsync, Flash},
    gpio::Input,
    i2c::{Async as I2cAsync, Config as I2cConfig, I2c, InterruptHandler as I2cInterruptHandler},
    peripherals::{self, I2C0},
};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Instant, Timer};
use mmc5983ma::{
    AsyncFunctions, ContinuousMeasurementFreq, DEFAULT_I2C_ADDRESS, Mmc5983, Mmc5983ConfigBuilder,
};
use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    I2C0_IRQ => I2cInterruptHandler<I2C0>;
});

assign_resources! {
    flashmem: FlashMem {
        flash: FLASH,
        dma: DMA_CH0,
    }
    i2cpins: I2CPins {
        scl: PIN_5,
        sda: PIN_4,
    }
    mmcirq: MmcIrq {
        irq: PIN_32,
    }
}

mod buffered_flash;
use crate::buffered_flash::BufferedFlash;

const FLASH_SIZE: usize = 2 * 1024 * 1024;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let resources = split_resources!(p);

    let i2c_config = I2cConfig::default();
    let i2c = Mutex::new(I2c::new_async(
        p.I2C0,
        resources.i2cpins.scl,
        resources.i2cpins.sda,
        Irqs,
        i2c_config,
    ));
    static I2C_BUS: StaticCell<Mutex<NoopRawMutex, I2c<I2C0, I2cAsync>>> = StaticCell::new();
    let i2c_bus = I2C_BUS.init(i2c);
    let mmc_i2c = I2cDevice::new(i2c_bus);
    info!("Initializing MMC5983MA...");
    let mut mmc = Mmc5983::new_with_i2c(
        mmc_i2c,
        DEFAULT_I2C_ADDRESS,
        Mmc5983ConfigBuilder::default()
            .frequency(ContinuousMeasurementFreq::Hz1)
            .set_interval(mmc5983ma::PeriodicSetInterval::Per100)
            .build(),
        embassy_time::Delay,
    );
    mmc.init().await.unwrap();
    info!("MMC5983MA initialized");
    info!("MMC5983MA started in continuous measurement mode");
    let temp = mmc.get_temp().await.unwrap();
    info!("Temperature: {}°C", temp.celcius());
    let mag = mmc.get_mag().await.unwrap().milligauss();
    info!("Magnetometer reading: x={} y={} z={}", mag.x, mag.y, mag.z);
    // Take a measurement to ensure the sensor is working
    let mut mmcirq = Input::new(resources.mmcirq.irq, embassy_rp::gpio::Pull::None);
    let mut ctr = 0;
    info!("Starting MMC5983MA interrupt-driven measurements...");
    mmc.start().await.unwrap();
    info!("Waiting for interrupts from MMC5983MA...");
    loop {
        mmcirq.wait_for_any_edge().await;
        let mag = mmc.get_mag().await.unwrap().milligauss();
        info!("Magnetometer reading: x={} y={} z={}", mag.x, mag.y, mag.z);
        let field_mag = libm::sqrtf(mag.x * mag.x + mag.y * mag.y + mag.z * mag.z);
        let theta = libm::atanf(mag.z / field_mag) * 180.0 / core::f32::consts::PI;
        let phi = libm::atan2f(mag.y, mag.x) * 180.0 / core::f32::consts::PI;
        info!(
            "Field magnitude: {}, Field angles: θ={}° φ={}°",
            field_mag, theta, phi
        );
        ctr += 1;
        if ctr >= 5 {
            let temp = mmc.get_temp().await.unwrap().celcius();
            info!("Temperature: {}°C", temp);
            mmc.stop().await.unwrap();
            info!("MMC5983MA stopped");
        }
    }

    // add some delay to give an attached debug probe time to parse the
    // defmt RTT header. Reading that header might touch flash memory, which
    // interferes with flash write operations.
    // https://github.com/knurling-rs/defmt/pull/683
    Timer::after_millis(10).await;

    spawner.spawn(core0_storage(resources.flashmem)).unwrap();
}

#[embassy_executor::task]
async fn core0_storage(flash: FlashMem) {
    let flash = Flash::<_, FlashAsync, FLASH_SIZE>::new(flash.flash, flash.dma);
    match BufferedFlash::new(flash) {
        Ok(mut state) => {
            info!(
                "Initialized BufferedFlash. Current offset: 0x{:x}",
                state.current_offset()
            );
            loop {
                match state.insert(b"   Hello world!\n") {
                    Ok(Some(ofst)) => {
                        info!("Wrote to flash at offset 0x{:x}", ofst);
                        break;
                    }
                    Ok(None) => {}
                    Err(e) => {
                        error!("Error inserting data into buffer: {}", e);
                        break;
                    }
                }
            }
        }
        Err(e) => {
            log::error!("Could not initialize BufferedFlash: {e:?}");
        }
    }
}
