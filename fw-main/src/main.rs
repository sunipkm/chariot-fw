//! This example test the flash connected to the RP2040 chip.

#![no_std]
#![no_main]

use assign_resources::assign_resources;
use defmt::*;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_rp::{
    Peri, bind_interrupts,
    flash::{Async as FlashAsync, Flash},
    i2c::{Async as I2cAsync, Config as I2cConfig, I2c, InterruptHandler as I2cInterruptHandler},
    peripherals::{self, I2C0},
};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_time::Duration;
#[allow(unused_imports)]
use embassy_time::{Instant, Timer};
use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

mod measurement;
mod sensor_tasks;
/// Ideally, we can write 
/// TODO: Check this value
pub const CADENCE: Duration = Duration::from_secs(2); // seconds
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
    baroirq: BaroIrq {
        irq: PIN_33,
    }
}

mod buffered_flash;
use crate::{
    buffered_flash::BufferedFlash,
    measurement::{MeasurementChannel, MeasurementReceiver, SINGLE_MEASUREMENT_SIZE},
    sensor_tasks::{baro_task, imu_task, magnetometer_task},
};

#[allow(unused)]
const FLASH_SIZE: usize = 2 * 1024 * 1024;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let resources = split_resources!(p);

    /// Set up a channel for sending measurements from sensor tasks to the storage task
    static DATA_CHANNEL: StaticCell<MeasurementChannel> = StaticCell::new();
    let channel = DATA_CHANNEL.init(Channel::new());

    info!("Chariot FW starting up...");
    // Set up flash memory
    // add some delay to give an attached debug probe time to parse the
    // defmt RTT header. Reading that header might touch flash memory, which
    // interferes with flash write operations.
    // https://github.com/knurling-rs/defmt/pull/683
    Timer::after_millis(10).await;

    spawner
        .spawn(core0_storage(resources.flashmem, channel.receiver()))
        .unwrap();

    let i2c_config = I2cConfig::default();
    let i2c = Mutex::new(I2c::new_async(
        p.I2C0,
        resources.i2cpins.scl,
        resources.i2cpins.sda,
        Irqs,
        i2c_config,
    ));

    /// Create I2C devices for each sensor
    static I2C_BUS: StaticCell<Mutex<NoopRawMutex, I2c<I2C0, I2cAsync>>> = StaticCell::new();
    let i2c_bus = I2C_BUS.init(i2c);
    let mmc_i2c = I2cDevice::new(i2c_bus);
    let imu_i2c = I2cDevice::new(i2c_bus);
    let baro_i2c = I2cDevice::new(i2c_bus);

    let reftime = Instant::now();

    // Spawn tasks to handle each sensor
    spawner
        .spawn(magnetometer_task(
            mmc_i2c,
            resources.mmcirq,
            channel.sender(),
            reftime,
        ))
        .unwrap();
    spawner
        .spawn(imu_task(
            imu_i2c,
            resources.imuirq,
            channel.sender(),
            reftime,
        ))
        .unwrap();
    spawner
        .spawn(baro_task(
            baro_i2c,
            resources.baroirq,
            channel.sender(),
            reftime,
        ))
        .unwrap();
}

#[embassy_executor::task]
async fn core0_storage(flash: FlashMem, receiver: MeasurementReceiver) {
    let flash = Flash::<_, FlashAsync, FLASH_SIZE>::new(flash.flash, flash.dma);
    match BufferedFlash::new(flash) {
        Ok(mut state) => {
            info!(
                "Initialized BufferedFlash. Current offset: 0x{:x}, capacity: {} bytes, messages: {}",
                state.current_offset(),
                state.remaining(),
                state.remaining() / SINGLE_MEASUREMENT_SIZE
            );
            let mut remaining_messages = state.remaining() / SINGLE_MEASUREMENT_SIZE;
            let mut remaining_bytes = state.remaining();
            let mut stored_messages = 0;
            const PAT: [u8; 8] = *b"CHARIOT\n";
            loop {
                match state.insert(PAT) {
                    Ok(Some(ofst)) => {
                        trace!("Wrote pattern to flash at offset 0x{:x}", ofst);
                        break;
                    }
                    Ok(None) => {}
                    Err(e) => {
                        error!("Error inserting data into buffer: {}", e);
                        break;
                    }
                }
            }
            loop {
                let measurement = receiver.receive().await;
                match state.insert(measurement.into_bytes()) {
                    Ok(maybeofst) => {
                        if let Some(ofst) = maybeofst {
                            trace!("Wrote measurement to flash at offset 0x{:x}", ofst);
                            remaining_bytes = state.remaining();
                        }
                        stored_messages += 1;
                        remaining_messages -= 1;
                        info!(
                            "Stored measurement {}, {} messages ({} bytes) remaining",
                            stored_messages, remaining_messages, remaining_bytes
                        );
                    }
                    Err(e) => {
                        error!("Error inserting measurement into buffer: {}", e);
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
