//! This example test the flash connected to the RP2040 chip.

#![no_std]
#![no_main]

use assign_resources::assign_resources;
use bmi323_rs::{AsyncFunctions as ImuAsyncFunctions, Bmi323, Bmi323Config, GyroRange};
use defmt::*;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
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
    AsyncFunctions as MagAsyncFunctions, ContinuousMeasurementFreq, DEFAULT_I2C_ADDRESS, Mmc5983,
    Mmc5983ConfigBuilder,
};
use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    I2C0_IRQ => I2cInterruptHandler<I2C0>;
});

type SharedI2cBus = I2cDevice<'static, NoopRawMutex, I2c<'static, I2C0, I2cAsync>>;

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
    imuirq: ImuIrq {
        irq: PIN_31,
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
    let imu_i2c = I2cDevice::new(i2c_bus);

    spawner
        .spawn(magnetometer_task(mmc_i2c, resources.mmcirq))
        .unwrap();

    spawner.spawn(imu_task(imu_i2c, resources.imuirq)).unwrap();

    // add some delay to give an attached debug probe time to parse the
    // defmt RTT header. Reading that header might touch flash memory, which
    // interferes with flash write operations.
    // https://github.com/knurling-rs/defmt/pull/683
    // Timer::after_millis(10).await;

    // spawner.spawn(core0_storage(resources.flashmem)).unwrap();
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

#[embassy_executor::task]
async fn magnetometer_task(i2c: SharedI2cBus, irq: MmcIrq) {
    info!("Initializing MMC5983MA...");
    let mut mmc = Mmc5983::new_with_i2c(
        i2c,
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
    let mut mmcirq = Input::new(irq.irq, embassy_rp::gpio::Pull::None);
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
}

#[embassy_executor::task]
async fn imu_task(i2c: SharedI2cBus, irq: ImuIrq) {
    let mut imu = Bmi323::new_with_i2c(
        i2c,
        bmi323_rs::DEFAULT_I2C_ADDRESS,
        Bmi323Config::default()
            .with_accel_mode(bmi323_rs::AccelMode::HighPerformance)
            .with_gyro_mode(bmi323_rs::GyroMode::HighPerformance)
            .with_gyro_range(GyroRange::Dps250)
            .with_accel_odr(bmi323_rs::OutputDataRate::Hz25)
            .with_gyro_odr(bmi323_rs::OutputDataRate::Hz25)
            .with_acc_irq(bmi323_rs::IrqMap::Int2)
            .with_gyro_irq(bmi323_rs::IrqMap::Int2)
            .with_temp_irq(bmi323_rs::IrqMap::Int2),
        embassy_time::Delay,
    );
    imu.init().await.unwrap();
    info!("bmi323_rs initialized");
    imu.calibrate(bmi323_rs::SelfCalibrateType::Both)
        .await
        .unwrap();
    // info!("bmi323_rs calibrated");
    let mut imuirq = Input::new(irq.irq, embassy_rp::gpio::Pull::None);
    imu.start().await.unwrap();
    info!("bmi323_rs started in continuous measurement mode");
    let mut now = Instant::now();
    loop {
        imuirq.wait_for_falling_edge().await;
        let elapsed = now.elapsed();
        now = Instant::now();
        // Handle interrupt
        if let Ok(data) = imu.read_data().await {
            if let Some(accel) = data.accel {
                let (ax, ay, az) = accel.float();
                info!("Accel: x={}g y={}g z={}g", ax, ay, az);
            }
            if let Some(gyro) = data.gyro {
                let (gx, gy, gz) = gyro.float();
                info!("Gyro: x={}dps y={}dps z={}dps", gx, gy, gz);
            }
            if let Some(temp) = data.temp {
                info!("Temperature: {}°C", temp.celcius());
            }
            info!("Time since last sample: {}ms", elapsed.as_millis());
        }
    }
}
