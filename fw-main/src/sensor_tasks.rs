use crate::{
    BaroIrq, CADENCE, ImuIrq, MmcIrq, SharedI2cBus,
    measurement::{CommonMeasurement, MeasurementSender, SingleMeasurement},
};
use defmt::*;
use embassy_rp::gpio::Input;
use embassy_time::Instant;

#[embassy_executor::task]
pub async fn magnetometer_task(
    i2c: SharedI2cBus,
    irq: MmcIrq,
    sender: MeasurementSender,
    tref: Instant,
) {
    use mmc5983ma::{
        AsyncFunctions as _, ContinuousMeasurementFreq, DEFAULT_I2C_ADDRESS, Mmc5983,
        Mmc5983ConfigBuilder,
    };
    trace!("Initializing MMC5983MA...");
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
    trace!("MMC5983MA initialized");
    trace!("MMC5983MA started in continuous measurement mode");
    let temp = mmc.get_temp().await.unwrap();
    trace!("Temperature: {}°C", temp.celcius());
    let mag = mmc.get_mag().await.unwrap().milligauss();
    trace!("Magnetometer reading: x={} y={} z={}", mag.x, mag.y, mag.z);
    // Take a measurement to ensure the sensor is working
    let mut mmcirq = Input::new(irq.irq, embassy_rp::gpio::Pull::None);
    trace!("Starting MMC5983MA interrupt-driven measurements...");
    mmc.start().await.unwrap();
    trace!("Waiting for interrupts from MMC5983MA...");
    let mut last = Instant::now();
    loop {
        mmcirq.wait_for_any_edge().await;
        let now = Instant::now();
        let mag = mmc.get_mag().await.unwrap().milligauss();
        let field_mag = libm::sqrtf(mag.x * mag.x + mag.y * mag.y + mag.z * mag.z);
        let theta = libm::acosf(mag.z / field_mag) * 180.0 / core::f32::consts::PI;
        let phi = libm::atan2f(mag.y, mag.x) * 180.0 / core::f32::consts::PI;
        trace!(
            "Magnetometer reading: x={} y={} z={}, mag={}, θ={}° φ={}°",
            mag.x, mag.y, mag.z, field_mag, theta, phi
        );
        if now.duration_since(last) >= CADENCE {
            last = now;
            if !sender.is_full() {
                let measurement = CommonMeasurement::Mag(mag.x, mag.y, mag.z);
                sender
                    .send(SingleMeasurement {
                        measurement,
                        timestamp: last.duration_since(tref).as_millis() as u32,
                    })
                    .await;
            }
        }
    }
}

#[embassy_executor::task]
pub async fn imu_task(i2c: SharedI2cBus, irq: ImuIrq, sender: MeasurementSender, tref: Instant) {
    use bmi323_rs::{AsyncFunctions as _, Bmi323, Bmi323Config, GyroRange};
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
    trace!("bmi323_rs initialized");
    imu.calibrate(bmi323_rs::SelfCalibrateType::Both)
        .await
        .unwrap();
    // trace!("bmi323_rs calibrated");
    let mut imuirq = Input::new(irq.irq, embassy_rp::gpio::Pull::None);
    imu.start().await.unwrap();
    trace!("bmi323_rs started in continuous measurement mode");
    let mut last = Instant::now();
    loop {
        imuirq.wait_for_falling_edge().await;
        let now = Instant::now();
        // Handle interrupt
        if let Ok(data) = imu.measure().await {
            let accel = if let Some(accel) = data.accel {
                let (ax, ay, az) = accel.float();
                let mag_a = libm::sqrtf(ax * ax + ay * ay + az * az);
                let theta_a = libm::acosf(az / mag_a) * 180.0 / core::f32::consts::PI;
                let phi_a = libm::atan2f(ay, ax) * 180.0 / core::f32::consts::PI;
                trace!(
                    "Accel: x={}g y={}g z={}g, Field magnitude: {}, Field angles: θ={}° φ={}°",
                    ax, ay, az, mag_a, theta_a, phi_a
                );
                Some((ax, ay, az))
            } else {
                None
            };
            let gyr = if let Some(gyro) = data.gyro {
                let (gx, gy, gz) = gyro.float();
                trace!("Gyro: x={}dps y={}dps z={}dps", gx, gy, gz);
                Some((gx, gy, gz))
            } else {
                None
            };
            if let Some(temp) = data.temp {
                trace!("Temperature: {}°C", temp.celcius());
            }
            if now.duration_since(last) >= CADENCE {
                last = now;
                if !sender.is_full() {
                    if let Some((ax, ay, az)) = accel {
                        let measurement = CommonMeasurement::Accel(ax, ay, az);
                        sender
                            .send(SingleMeasurement {
                                measurement,
                                timestamp: last.duration_since(tref).as_millis() as u32,
                            })
                            .await;
                    }
                    if let Some((gx, gy, gz)) = gyr {
                        let measurement = CommonMeasurement::Gyro(gx, gy, gz);
                        sender
                            .send(SingleMeasurement {
                                measurement,
                                timestamp: last.duration_since(tref).as_millis() as u32,
                            })
                            .await;
                    }
                }
            }
        }
    }
}

#[embassy_executor::task]
pub async fn baro_task(i2c: SharedI2cBus, irq: BaroIrq, sender: MeasurementSender, tref: Instant) {
    use bmp390_rs::{
        AsyncFunctions as _, Bmp390, Bmp390Config, Length, degree_celsius, foot, hectopascal, meter,
    };
    let mut baro = Bmp390::new_with_i2c(
        i2c,
        bmp390_rs::DEFAULT_I2C_ADDRESS,
        Bmp390Config::default()
            .enable_irq(true)
            .with_output_data_rate(bmp390_rs::OutputDataRate::Hz25),
        embassy_time::Delay,
    );
    let cal = baro.init().await.unwrap();
    trace!("bmp390_rs initialized");
    let mut baroirq = Input::new(irq.irq, embassy_rp::gpio::Pull::None);
    baro.start().await.unwrap();
    trace!("bmp390_rs started in continuous measurement mode");
    let mut last = Instant::now();
    loop {
        baroirq.wait_for_falling_edge().await;
        let now = Instant::now();
        // Handle interrupt
        if let Ok(data) = baro.measure().await {
            if let Some((temp, pres, alt)) = cal.convert(data, Length::new::<meter>(0.0)) {
                trace!(
                    "Temperature: {}°C, Pressure: {}hPa, Altitude: {}ft",
                    temp.get::<degree_celsius>(),
                    pres.get::<hectopascal>(),
                    alt.get::<foot>()
                );
                if now.duration_since(last) >= CADENCE {
                    last = now;
                    if !sender.is_full() {
                        let measurement = CommonMeasurement::Baro(
                            temp.get::<degree_celsius>(),
                            pres.get::<hectopascal>(),
                            alt.get::<foot>(),
                        );
                        sender
                            .send(SingleMeasurement {
                                measurement,
                                timestamp: last.duration_since(tref).as_millis() as u32,
                            })
                            .await;
                    }
                }
            }
        }
    }
}
