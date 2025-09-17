# MMC5983MA: 3-Axis Magnetic Sensor

A library to use the MEMSIC MMC5893MA magnetometer in both [`sync`](https://docs.rs/embedded-hal/latest/embedded_hal/) and [`async`](https://docs.rs/embedded-hal-async/latest/embedded_hal_async/) modes.

Implemented:
 - [x] I2C communication
 - [x] Read magnetic data
 - [x] Read temperature data
 - [x] Reset
 - [ ] Configuration
   - [x] Decimation Bandwidth
   - [x] Continuous Acquisition Modes
   - [x] Measurement Done Interrupts
   - [x] Periodic SET
 - [ ] OTP
 - [x] Bridge-offset SET/RST
 - [ ] Calibration
 - [x] Documentation
 - [ ] Examples
