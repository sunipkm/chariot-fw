use defmt::{trace, warn};
use embassy_rp::{
    flash::{Async, Error, PAGE_SIZE},
    peripherals::FLASH,
};
use heapless::Vec;

/// Manages writing data to flash memory in PAGE_SIZE chunks.
pub struct BufferedFlash<const SIZE: usize> {
    offset: u32,
    buf: heapless::Vec<u8, PAGE_SIZE>,
    flash: embassy_rp::flash::Flash<'static, FLASH, Async, SIZE>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FlashError {
    ReadError(u32, Error),
    WriteError(u32, Error),
    BufferOverflow(usize),
    OutOfMemory,
}

pub type FlashResult<T> = Result<T, FlashError>;

impl<const SIZE: usize> BufferedFlash<SIZE> {
    /// Creates a new [`FlashState`] instance.
    ///
    /// This function scans the flash memory to find the first empty page (filled with 0xFF).
    ///
    /// # Errors
    /// - Returns `FlashError::ReadError` if there is an error reading from flash.
    pub fn new(
        mut flash: embassy_rp::flash::Flash<'static, FLASH, Async, SIZE>,
    ) -> FlashResult<Self> {
        let mut buf = [0u8; PAGE_SIZE];
        let mut offset = 0;
        loop {
            match flash.blocking_read(offset, &mut buf) {
                Ok(()) => {
                    if buf.iter().all(|x| *x == 0xFF) {
                        break;
                    } else {
                        offset += buf.len() as u32;
                    }
                }
                Err(e) => {
                    trace!("Error reading flash at offset {:x}: {:?}", offset, e);
                    return Err(FlashError::ReadError(offset, e));
                }
            }
        }
        let mut buf = Vec::from_array(buf);
        buf.clear();
        Ok(Self { offset, buf, flash })
    }

    /// Inserts data into the buffer and writes to flash if the buffer is full.
    ///
    /// # Returns
    /// - `Ok(true)` if the buffer was flushed to flash.
    /// - `Ok(false)` if the data was added to the buffer without flushing.
    ///
    /// # Errors
    /// - Returns `FlashError::BufferOverflow` if the data exceeds buffer capacity.
    /// - Returns `FlashError::WriteError` if there is an error writing to flash.
    /// - Returns `FlashError::ReadError` if there is an error reading from flash for verification.
    /// - Returns `FlashError::VerifyError` if the written data does not match the buffer.
    pub fn insert<T: AsRef<[u8]>>(&mut self, data: T) -> FlashResult<Option<u32>> {
        let mut flushed = None;
        let mut data = ReadonlyVec::new(data.as_ref());
        if self.buf.len() + data.len() + self.offset as usize > SIZE {
            if !self.buf.is_empty() {
                self.flush()?;
            }
            return Err(FlashError::OutOfMemory);
        }
        while data.len() > 0 {
            let to_copy = core::cmp::min(data.len(), self.buf.capacity() - self.buf.len());
            self.buf
                .extend_from_slice(data.drain(to_copy))
                .map_err(|_| FlashError::BufferOverflow(data.len()))?;
            if self.buf.len() == self.buf.capacity() {
                flushed = Some(self.flush()?);
            }
        }
        Ok(flushed)
    }

    fn flush(&mut self) -> FlashResult<u32> {
        let curr_ofst = self.offset;
        loop {
            self.flash
                .blocking_write(self.offset, &self.buf)
                .map_err(|e| FlashError::WriteError(self.offset, e))?;
            let read_buf = &mut [0u8; PAGE_SIZE][..self.buf.len()];
            self.flash
                .blocking_read(self.offset, read_buf)
                .map_err(|e| FlashError::ReadError(self.offset, e))?;
            if read_buf != self.buf.as_slice() {
                warn!("Flash verify error at offset 0x{:x}", self.offset);
                // info!("Expected: {=[u8]:#x}", self.buf.as_slice());
                // info!("Read:     {=[u8]:#x}", read_buf);
                self.offset += self.buf.len() as u32;
                continue;
            } else {
                self.offset += self.buf.len() as u32;
                break;
            }
        }
        self.buf.clear();
        Ok(curr_ofst)
    }

    /// Returns the current write offset in flash memory.
    pub fn current_offset(&self) -> u32 {
        self.offset
    }

    /// Returns the remaining space in the buffer.
    pub fn remaining(&self) -> usize {
        SIZE - self.offset as usize
    }
}

struct ReadonlyVec<'a, T> {
    data: &'a [T],
    pos: usize,
}

impl<'a, T> ReadonlyVec<'a, T> {
    pub fn new(data: &'a [T]) -> Self {
        Self { data, pos: 0 }
    }

    pub fn drain(&mut self, n: usize) -> &[T] {
        let end = core::cmp::min(self.pos + n, self.data.len());
        let slice = &self.data[self.pos..end];
        self.pos = end;
        slice
    }

    pub fn len(&self) -> usize {
        self.data.len() - self.pos
    }
}
