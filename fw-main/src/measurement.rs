use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::{Channel, Receiver, Sender}};
use heapless::String;

pub type MeasurementChannel = Channel<NoopRawMutex, SingleMeasurement, 8>;
pub type MeasurementSender = Sender<'static, NoopRawMutex, SingleMeasurement, 8>;
pub type MeasurementReceiver = Receiver<'static, NoopRawMutex, SingleMeasurement, 8>;

/// Unified measurement type
#[repr(u8)]
#[derive(Debug, Clone)]
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

impl CommonMeasurement {
    pub fn into_bytes(self) -> [u8; 16] {
        let mut bytes = [0u8; 16];
        bytes[0] = match self {
            CommonMeasurement::Accel(_, _, _) => 0x01,
            CommonMeasurement::Gyro(_, _, _) => 0x02,
            CommonMeasurement::Mag(_, _, _) => 0x03,
            CommonMeasurement::Temp(_, _) => 0x04,
            CommonMeasurement::Baro(_, _, _) => 0x05,
        };
        match self {
            CommonMeasurement::Accel(x, y, z) => {
                bytes[1..5].copy_from_slice(&x.to_le_bytes());
                bytes[5..9].copy_from_slice(&y.to_le_bytes());
                bytes[9..13].copy_from_slice(&z.to_le_bytes());
            }
            CommonMeasurement::Gyro(x, y, z) => {
                bytes[1..5].copy_from_slice(&x.to_le_bytes());
                bytes[5..9].copy_from_slice(&y.to_le_bytes());
                bytes[9..13].copy_from_slice(&z.to_le_bytes());
            }
            CommonMeasurement::Mag(x, y, z) => {
                bytes[1..5].copy_from_slice(&x.to_le_bytes());
                bytes[5..9].copy_from_slice(&y.to_le_bytes());
                bytes[9..13].copy_from_slice(&z.to_le_bytes());
            }
            CommonMeasurement::Temp(label, value) => {
                let label_bytes = label.as_bytes();
                let len = label_bytes.len().min(7);
                bytes[1..1 + len].copy_from_slice(&label_bytes[..len]);
                bytes[8..12].copy_from_slice(&value.to_le_bytes());
            }
            CommonMeasurement::Baro(temp, pres, alt) => {
                bytes[1..5].copy_from_slice(&temp.to_le_bytes());
                bytes[5..9].copy_from_slice(&pres.to_le_bytes());
                bytes[9..13].copy_from_slice(&alt.to_le_bytes());
            }
        }
        bytes
    }
}

pub enum CommonMeasurementError {
    Length,
    String,
    Type,
}

impl TryFrom<&[u8]> for CommonMeasurement {
    type Error = CommonMeasurementError;
    fn try_from(value: &[u8]) -> Result<Self, Self::Error> {
        if value.len() != 16 {
            return Err(CommonMeasurementError::Length);
        }
        let measurement = match value[0] {
            0x01 => {
                let x = f32::from_le_bytes(value[1..5].try_into().unwrap());
                let y = f32::from_le_bytes(value[5..9].try_into().unwrap());
                let z = f32::from_le_bytes(value[9..13].try_into().unwrap());
                CommonMeasurement::Accel(x, y, z)
            }
            0x02 => {
                let x = f32::from_le_bytes(value[1..5].try_into().unwrap());
                let y = f32::from_le_bytes(value[5..9].try_into().unwrap());
                let z = f32::from_le_bytes(value[9..13].try_into().unwrap());
                CommonMeasurement::Gyro(x, y, z)
            }
            0x03 => {
                let x = f32::from_le_bytes(value[1..5].try_into().unwrap());
                let y = f32::from_le_bytes(value[5..9].try_into().unwrap());
                let z = f32::from_le_bytes(value[9..13].try_into().unwrap());
                CommonMeasurement::Mag(x, y, z)
            }
            0x04 => {
                let label_end = value[1..8].iter().position(|&b| b == 0).unwrap_or(7);
                let label: String<8> = String::try_from(
                    core::str::from_utf8(&value[1..1 + label_end]).unwrap_or("Unknown"),
                )
                .map_err(|_| CommonMeasurementError::String)?;
                let value = f32::from_le_bytes(value[8..12].try_into().unwrap());
                CommonMeasurement::Temp(label, value)
            }
            0x05 => {
                let t = f32::from_le_bytes(value[1..5].try_into().unwrap());
                let p = f32::from_le_bytes(value[5..9].try_into().unwrap());
                let a = f32::from_le_bytes(value[9..13].try_into().unwrap());
                CommonMeasurement::Baro(t, p, a)
            }
            _ => return Err(CommonMeasurementError::Type),
        };
        Ok(measurement)
    }
}

pub const SINGLE_MEASUREMENT_SIZE: usize = 20;

pub struct SingleMeasurement {
    pub measurement: CommonMeasurement,
    pub timestamp: u32,
}

impl SingleMeasurement {
    pub fn into_bytes(self) -> [u8; 20] {
        let mut bytes = [0u8; 20];
        bytes[0..16].copy_from_slice(&self.measurement.into_bytes());
        bytes[16..20].copy_from_slice(&self.timestamp.to_le_bytes());
        bytes
    }
}

impl TryFrom<&[u8]> for SingleMeasurement {
    type Error = CommonMeasurementError;
    fn try_from(value: &[u8]) -> Result<Self, Self::Error> {
        if value.len() != 20 {
            return Err(CommonMeasurementError::Length);
        }
        let measurement = CommonMeasurement::try_from(&value[0..16])?;
        let timestamp = u32::from_le_bytes(value[16..20].try_into().unwrap());
        Ok(SingleMeasurement {
            measurement,
            timestamp,
        })
    }
}
