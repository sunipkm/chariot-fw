use embassy_sync::{
    blocking_mutex::raw::NoopRawMutex,
    channel::{Channel, Receiver, Sender},
};
use heapless::String;

pub type MeasurementChannel = Channel<NoopRawMutex, SingleMeasurement, 8>;
pub type MeasurementSender = Sender<'static, NoopRawMutex, SingleMeasurement, 8>;
pub type MeasurementReceiver = Receiver<'static, NoopRawMutex, SingleMeasurement, 8>;

type CrcState = crc16::State<crc16::XMODEM>;

const ACCEL_CODE: u16 = 0xACC1;
const GYRO_CODE: u16 = 0x6e50;
const MAG_CODE: u16 = 0x9A61;
const TEMP_CODE: u16 = 0x7E70;
const BARO_CODE: u16 = 0xB480;

/// Unified measurement type
#[derive(Debug, Clone, PartialEq)]
pub enum CommonMeasurement {
    /// Accelerometer measurement in g
    Accel(f32, f32, f32),
    /// Gyroscope measurement in degrees per second
    Gyro(f32, f32, f32),
    /// Magnetometer measurement in milligauss
    Mag(f32, f32, f32),
    /// Temperature measurement in degrees Celsius
    Temp(heapless::String<8>, f32),
    /// Temperature, pressure and altitude measurement
    Baro(f32, f32, f32),
}

const COMMON_MEASUREMENT_SIZE: usize = 1 + 12 + 1; // type (2 bytes) + 12 bytes (3 * f32, 8 bytes for Temp label + f32 value)

impl CommonMeasurement {
    pub fn into_bytes(self) -> [u8; COMMON_MEASUREMENT_SIZE] {
        let mut bytes = [0u8; COMMON_MEASUREMENT_SIZE];
        let code = match self {
            CommonMeasurement::Accel(_, _, _) => ACCEL_CODE,
            CommonMeasurement::Gyro(_, _, _) => GYRO_CODE,
            CommonMeasurement::Mag(_, _, _) => MAG_CODE,
            CommonMeasurement::Temp(_, _) => TEMP_CODE,
            CommonMeasurement::Baro(_, _, _) => BARO_CODE,
        };
        bytes[0..2].copy_from_slice(&code.to_le_bytes());
        match self {
            CommonMeasurement::Accel(x, y, z) => {
                bytes[2..6].copy_from_slice(&x.to_le_bytes());
                bytes[6..10].copy_from_slice(&y.to_le_bytes());
                bytes[10..COMMON_MEASUREMENT_SIZE].copy_from_slice(&z.to_le_bytes());
            }
            CommonMeasurement::Gyro(x, y, z) => {
                bytes[2..6].copy_from_slice(&x.to_le_bytes());
                bytes[6..10].copy_from_slice(&y.to_le_bytes());
                bytes[10..COMMON_MEASUREMENT_SIZE].copy_from_slice(&z.to_le_bytes());
            }
            CommonMeasurement::Mag(x, y, z) => {
                bytes[2..6].copy_from_slice(&x.to_le_bytes());
                bytes[6..10].copy_from_slice(&y.to_le_bytes());
                bytes[10..COMMON_MEASUREMENT_SIZE].copy_from_slice(&z.to_le_bytes());
            }
            CommonMeasurement::Temp(label, value) => {
                let label_bytes = label.as_bytes();
                let len = label_bytes.len().min(8);
                bytes[2..2 + len].copy_from_slice(&label_bytes[..len]);
                bytes[10..COMMON_MEASUREMENT_SIZE].copy_from_slice(&value.to_le_bytes());
            }
            CommonMeasurement::Baro(temp, pres, alt) => {
                bytes[2..6].copy_from_slice(&temp.to_le_bytes());
                bytes[6..10].copy_from_slice(&pres.to_le_bytes());
                bytes[10..COMMON_MEASUREMENT_SIZE].copy_from_slice(&alt.to_le_bytes());
            }
        }
        bytes
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CommonMeasurementError {
    Length,
    String,
    Type,
    CrcMismatch,
}

impl TryFrom<&[u8]> for CommonMeasurement {
    type Error = CommonMeasurementError;
    fn try_from(value: &[u8]) -> Result<Self, Self::Error> {
        if value.len() != COMMON_MEASUREMENT_SIZE {
            return Err(CommonMeasurementError::Length);
        }
        let measurement = match u16::from_le_bytes(value[0..2].try_into().unwrap()) {
            ACCEL_CODE => {
                let x = f32::from_le_bytes(value[2..6].try_into().unwrap());
                let y = f32::from_le_bytes(value[6..10].try_into().unwrap());
                let z = f32::from_le_bytes(value[10..COMMON_MEASUREMENT_SIZE].try_into().unwrap());
                CommonMeasurement::Accel(x, y, z)
            }
            GYRO_CODE => {
                let x = f32::from_le_bytes(value[2..6].try_into().unwrap());
                let y = f32::from_le_bytes(value[6..10].try_into().unwrap());
                let z = f32::from_le_bytes(value[10..COMMON_MEASUREMENT_SIZE].try_into().unwrap());
                CommonMeasurement::Gyro(x, y, z)
            }
            MAG_CODE => {
                let x = f32::from_le_bytes(value[2..6].try_into().unwrap());
                let y = f32::from_le_bytes(value[6..10].try_into().unwrap());
                let z = f32::from_le_bytes(value[10..COMMON_MEASUREMENT_SIZE].try_into().unwrap());
                CommonMeasurement::Mag(x, y, z)
            }
            TEMP_CODE => {
                let label_end = value[2..10].iter().position(|&b| b == 0).unwrap_or(8);
                let label: String<8> = String::try_from(
                    core::str::from_utf8(&value[2..2 + label_end]).unwrap_or("Unknown"),
                )
                .map_err(|_| CommonMeasurementError::String)?;
                let value =
                    f32::from_le_bytes(value[10..COMMON_MEASUREMENT_SIZE].try_into().unwrap());
                CommonMeasurement::Temp(label, value)
            }
            BARO_CODE => {
                let t = f32::from_le_bytes(value[2..6].try_into().unwrap());
                let p = f32::from_le_bytes(value[6..10].try_into().unwrap());
                let a = f32::from_le_bytes(value[10..COMMON_MEASUREMENT_SIZE].try_into().unwrap());
                CommonMeasurement::Baro(t, p, a)
            }
            _ => return Err(CommonMeasurementError::Type),
        };
        Ok(measurement)
    }
}

pub const SINGLE_MEASUREMENT_SIZE: usize = 20;

#[derive(Debug, Clone, PartialEq)]
pub struct SingleMeasurement {
    pub measurement: CommonMeasurement,
    pub timestamp: u32,
}

impl SingleMeasurement {
    /// Byte map:
    /// [2 byte TYPE] [12 bytes measurement data] [4 bytes timestamp] [2 bytes CRC]
    pub fn into_bytes(self) -> [u8; SINGLE_MEASUREMENT_SIZE] {
        let mut bytes = [0u8; SINGLE_MEASUREMENT_SIZE];
        bytes[0..COMMON_MEASUREMENT_SIZE].copy_from_slice(&self.measurement.into_bytes());
        bytes[COMMON_MEASUREMENT_SIZE..COMMON_MEASUREMENT_SIZE + 4]
            .copy_from_slice(&self.timestamp.to_le_bytes());
        let crc = CrcState::calculate(&bytes[0..SINGLE_MEASUREMENT_SIZE - 2]);
        bytes[SINGLE_MEASUREMENT_SIZE - 2..].copy_from_slice(&crc.to_le_bytes());
        bytes
    }
}

impl TryFrom<&[u8]> for SingleMeasurement {
    type Error = CommonMeasurementError;
    fn try_from(value: &[u8]) -> Result<Self, Self::Error> {
        if value.len() != SINGLE_MEASUREMENT_SIZE {
            return Err(CommonMeasurementError::Length);
        }
        let received_crc = u16::from_le_bytes(
            value[SINGLE_MEASUREMENT_SIZE - 2..SINGLE_MEASUREMENT_SIZE]
                .try_into()
                .unwrap(),
        );
        let computed_crc = CrcState::calculate(&value[0..SINGLE_MEASUREMENT_SIZE - 2]);
        if received_crc != computed_crc {
            return Err(CommonMeasurementError::CrcMismatch);
        }
        let measurement = CommonMeasurement::try_from(&value[0..COMMON_MEASUREMENT_SIZE])?;
        let timestamp = u32::from_le_bytes(
            value[COMMON_MEASUREMENT_SIZE..COMMON_MEASUREMENT_SIZE + 4]
                .try_into()
                .unwrap(),
        );
        Ok(SingleMeasurement {
            measurement,
            timestamp,
        })
    }
}
