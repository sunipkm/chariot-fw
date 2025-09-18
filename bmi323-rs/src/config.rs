use crate::{registers::{
    AccelConfig, Bandwidth, AveragingSamples, OutputDataRate, AccelMode, AccelRange,
    FeatureEngineControl, FifoConfig, GyroConfig, GyroMode, I2cWatchdogConfig, IrqMap,
    IntPinConfig, SensorInterruptMap,
}, GyroRange};

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(crate) enum AccGyroEnabled {
    #[default]
    None = 0,
    Accel = 1,
    Gyro = 2,
    Both = 3,
}

impl AccGyroEnabled {
    pub(crate) fn is_accel_enabled(&self) -> bool {
        matches!(self, AccGyroEnabled::Accel | AccGyroEnabled::Both)
    }

    pub(crate) fn is_gyro_enabled(&self) -> bool {
        matches!(self, AccGyroEnabled::Gyro | AccGyroEnabled::Both)
    }
}

impl From<u8> for AccGyroEnabled {
    fn from(value: u8) -> Self {
        match value {
            0 => AccGyroEnabled::None,
            1 => AccGyroEnabled::Accel,
            2 => AccGyroEnabled::Gyro,
            3 => AccGyroEnabled::Both,
            _ => AccGyroEnabled::None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
/// Configuration for an interrupt pin.
pub struct IrqPinConfig {
    /// [`true`] if the interrupt pin is active high, [`false`] if active low.
    pub active_high: bool,
    /// [`true`] if the interrupt pin is push-pull, [`false`] if open-drain.
    pub open_drain: bool,
}

#[derive(Debug, Clone, Copy, Default)]
/// Configuration for the BMI323 sensor.
pub struct Bmi323Config {
    accel_odr: OutputDataRate,
    accel_range: AccelRange,
    accel_bw: Bandwidth,
    accel_numavg: AveragingSamples,
    accel_mode: AccelMode,
    gyro_odr: OutputDataRate,
    gyro_range: GyroRange,
    gyro_bw: Bandwidth,
    gyro_numavg: AveragingSamples,
    gyro_mode: GyroMode,
    acc_irq: IrqMap,
    gyro_irq: IrqMap,
    temp_irq: IrqMap,
    int1_cfg: IrqPinConfig,
    int2_cfg: IrqPinConfig,
    pub(crate) sensors_enabled: AccGyroEnabled,
    pub(crate) irq_enabled: bool,
    pub(crate) min_delay_us: u32,
}

impl Bmi323Config {
    /// Set the accelerometer output data rate ([`AccelGyroOdr`]).
    pub fn with_accel_odr(mut self, odr: OutputDataRate) -> Self {
        self.accel_odr = odr;
        self.build();
        self
    }

    /// Set the accelerometer range ([`AccelRange`]).
    pub fn with_accel_range(mut self, range: AccelRange) -> Self {
        self.accel_range = range;
        self
    }

    /// Set the accelerometer bandwidth ([`AccelGyroBw`]).
    pub fn with_accel_bw(mut self, bw: Bandwidth) -> Self {
        self.accel_bw = bw;
        self
    }

    /// Set the accelerometer number of samples to average ([`AccelGyroNumAvg`]).
    pub fn with_accel_numavg(mut self, numavg: AveragingSamples) -> Self {
        self.accel_numavg = numavg;
        self.build();
        self
    }

    /// Enable the accelerometer.
    pub fn with_accel_mode(mut self, mode: AccelMode) -> Self {
        self.accel_mode = mode;
        self
    }

    /// Set the gyroscope output data rate ([`AccelGyroOdr`]).
    pub fn with_gyro_odr(mut self, odr: OutputDataRate) -> Self {
        self.gyro_odr = odr;
        self.build();
        self
    }

    /// Set the gyroscope range ([`GyroRange`]).
    pub fn with_gyro_range(mut self, range: GyroRange) -> Self {
        self.gyro_range = range;
        self
    }

    /// Set the gyroscope bandwidth ([`AccelGyroBw`]).
    pub fn with_gyro_bw(mut self, bw: Bandwidth) -> Self {
        self.gyro_bw = bw;
        self
    }

    /// Set the gyroscope number of samples to average ([`AccelGyroNumAvg`]).
    pub fn with_gyro_numavg(mut self, numavg: AveragingSamples) -> Self {
        self.gyro_numavg = numavg;
        self.build();
        self
    }

    /// Enable the gyroscope.
    pub fn with_gyro_mode(mut self, mode: GyroMode) -> Self {
        self.gyro_mode = mode;
        self
    }

    /// Set the interrupt mapping for the accelerometer.
    pub fn with_acc_irq(mut self, map: IrqMap) -> Self {
        self.acc_irq = map;
        self
    }

    /// Set the interrupt mapping for the gyroscope.
    pub fn with_gyro_irq(mut self, map: IrqMap) -> Self {
        self.gyro_irq = map;
        self
    }

    /// Set the interrupt mapping for the temperature sensor.
    pub fn with_temp_irq(mut self, map: IrqMap) -> Self {
        self.temp_irq = map;
        self
    }

    /// Set the configuration for the INT1 pin.
    pub fn with_int1_cfg(mut self, cfg: IrqPinConfig) -> Self {
        self.int1_cfg = cfg;
        self
    }

    /// Set the configuration for the INT2 pin.
    pub fn with_int2_cfg(mut self, cfg: IrqPinConfig) -> Self {
        self.int2_cfg = cfg;
        self
    }

    /// Get the accelerometer output data rate.
    pub fn accel_odr(&self) -> OutputDataRate {
        self.accel_odr
    }

    /// Get the accelerometer range.
    pub fn accel_range(&self) -> AccelRange {
        self.accel_range
    }

    /// Get the accelerometer bandwidth.
    pub fn accel_bw(&self) -> Bandwidth {
        self.accel_bw
    }

    /// Get the accelerometer number of samples to average.
    pub fn accel_numavg(&self) -> AveragingSamples {
        self.accel_numavg
    }

    /// Get whether the accelerometer is enabled.
    pub fn accel_mode(&self) -> AccelMode {
        self.accel_mode
    }

    /// Get the gyroscope output data rate.
    pub fn gyro_odr(&self) -> OutputDataRate {
        self.gyro_odr
    }

    /// Get the gyroscope range.
    pub fn gyro_range(&self) -> GyroRange {
        self.gyro_range
    }

    /// Get the gyroscope bandwidth.
    pub fn gyro_bw(&self) -> Bandwidth {
        self.gyro_bw
    }

    /// Get the gyroscope number of samples to average.
    pub fn gyro_numavg(&self) -> AveragingSamples {
        self.gyro_numavg
    }

    /// Get whether the gyroscope is enabled.
    pub fn gyro_mode(&self) -> GyroMode {
        self.gyro_mode
    }

    /// Get the interrupt mapping for the accelerometer.
    pub fn acc_irq(&self) -> IrqMap {
        self.acc_irq
    }

    /// Get the interrupt mapping for the gyroscope.
    pub fn gyro_irq(&self) -> IrqMap {
        self.gyro_irq
    }

    /// Get the interrupt mapping for the temperature sensor.
    pub fn temp_irq(&self) -> IrqMap {
        self.temp_irq
    }

    /// Get the configuration for the INT1 pin.
    pub fn int1_cfg(&self) -> IrqPinConfig {
        self.int1_cfg
    }

    /// Get the configuration for the INT2 pin.
    pub fn int2_cfg(&self) -> IrqPinConfig {
        self.int2_cfg
    }

    /// Validate and finalize the configuration.
    fn build(&mut self) {
        self.accel_numavg = limit_bw_from_odr(self.accel_odr, self.accel_numavg);
        self.gyro_numavg = limit_bw_from_odr(self.gyro_odr, self.gyro_numavg);
        self.min_delay_us = self.accel_odr.delay().min(self.gyro_odr.delay());
    }
}

fn limit_bw_from_odr(odr: OutputDataRate, num_avg: AveragingSamples) -> AveragingSamples {
    if odr > OutputDataRate::Hz400 {
        AveragingSamples::Samples1
    } else if odr > OutputDataRate::Hz200 {
        num_avg.min(AveragingSamples::Samples8)
    } else if odr > OutputDataRate::Hz100 {
        num_avg.min(AveragingSamples::Samples16)
    } else if odr > OutputDataRate::Hz50 {
        num_avg.min(AveragingSamples::Samples32)
    } else {
        num_avg
    }
}

impl Bmi323Config {
    /// Convert the configuration to register values.
    #[must_use]
    pub(crate) fn get_registers(
        &mut self,
    ) -> (
        FeatureEngineControl,
        FifoConfig,
        SensorInterruptMap,
        I2cWatchdogConfig,
        IntPinConfig,
        AccelConfig,
        GyroConfig,
    ) {
        // Self setup
        self.build(); // Ensure config is built
        self.sensors_enabled = AccGyroEnabled::from(
            ((self.accel_mode != AccelMode::Off) as u8)
                | ((self.gyro_mode != GyroMode::Off) as u8) << 1,
        );
        let irq1_en = self.acc_irq == IrqMap::Int1
            || self.gyro_irq == IrqMap::Int1
            || self.temp_irq == IrqMap::Int1;
        let irq2_en = self.acc_irq == IrqMap::Int2
            || self.gyro_irq == IrqMap::Int2
            || self.temp_irq == IrqMap::Int2;
        self.irq_enabled = irq1_en || irq2_en;
        // Registers
        let feature_engine_ctrl = FeatureEngineControl::new().with_enable(true);
        let fifo_cfg = FifoConfig::new(); // Default FIFO config, disabled
        let sensor_irq_map = SensorInterruptMap::new()
            .with_acc_drdy(self.acc_irq)
            .with_gyr_drdy(self.gyro_irq)
            .with_temp_drdy(self.temp_irq);
        let i2c_wdt_config = I2cWatchdogConfig::new().with_enable(false); // Disable I2C watchdog
        let int_pin_config: IntPinConfig = (self.int1_cfg, irq1_en, self.int2_cfg, irq2_en).into();
        let accel_config = AccelConfig::new()
            .with_odr(self.accel_odr)
            .with_range(self.accel_range)
            .with_bandwidth(self.accel_bw)
            .with_avg_samples(self.accel_numavg)
            .with_mode(AccelMode::Off); // Start with accel off
        let gyro_config = GyroConfig::new()
            .with_odr(self.gyro_odr)
            .with_range(self.gyro_range)
            .with_bandwidth(self.gyro_bw)
            .with_avg_samples(self.gyro_numavg)
            .with_mode(GyroMode::Off); // Start with gyro off
        (
            feature_engine_ctrl,
            fifo_cfg,
            sensor_irq_map,
            i2c_wdt_config,
            int_pin_config,
            accel_config,
            gyro_config,
        )
    }
}

impl From<(IrqPinConfig, bool, IrqPinConfig, bool)> for IntPinConfig {
    fn from(
        (int1_cfg, irq1_en, int2_cfg, irq2_en): (IrqPinConfig, bool, IrqPinConfig, bool),
    ) -> Self {
        IntPinConfig::new()
            .with_int1_active_high(int1_cfg.active_high)
            .with_int1_open_drain(int1_cfg.open_drain)
            .with_int1_output_en(irq1_en)
            .with_int2_active_high(int2_cfg.active_high)
            .with_int2_open_drain(int2_cfg.open_drain)
            .with_int2_output_en(irq2_en)
    }
}
