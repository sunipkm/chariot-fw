#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::flash::{Async, ERASE_SIZE};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

const FLASH_SIZE: usize = 2 * 1024 * 1024;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("Hello World!");

    // add some delay to give an attached debug probe time to parse the
    // defmt RTT header. Reading that header might touch flash memory, which
    // interferes with flash write operations.
    // https://github.com/knurling-rs/defmt/pull/683
    Timer::after_millis(10).await;

    let mut flash = embassy_rp::flash::Flash::<_, Async, FLASH_SIZE>::new(p.FLASH, p.DMA_CH0);

    let mut offset = FLASH_SIZE - ERASE_SIZE;
    while offset > 0 {
        flash
            .blocking_erase(offset as u32, (offset + ERASE_SIZE) as u32)
            .unwrap();
        info!("erased 0x{:X}..0x{:X}", offset, offset + ERASE_SIZE);
        offset -= ERASE_SIZE;
    }
}
