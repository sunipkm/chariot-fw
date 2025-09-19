# CHARIOT Flight Firmware

## Setup

1. Install the [Rust toolchain](https://rustup.rs/).
2. Add the required platform target:
  ```sh
  rustup target add thumbv8m.main-none-eabihf
  ```
3. Install [`probe-rs`](https://probe.rs) for flashing the firmware.
4. For Linux:
   -  Install the `udev` rules:
    ```sh
    sudo install rules/*.rules /etc/udev/rules.d
    ```
   - Reload the rules:
    ```sh
    sudo udevadm control --reload-rules && sudo udevadm trigger
    ```

## Pre-flight Procedure
1. Connect the debug probe to your computer and the target board.
2. Power on the target board.
3. Erase the flash memory, wait for it to finish:
   ```sh
   cargo run -p erase-flash --release
   ```
  The flash eraser will crash at the end, this is expected.
4. Flash the firmware:
   ```sh
   cargo run -p chariot-fw --release | tee preflight_`date +%Y%m%d-%H%M%S`.log
   ```
5. Monitor the output log for any errors or important information. Specifically, look for the line indicating the start offset of the flash memory where measurements are being stored. Press <kbd>Ctrl</kbd> + <kbd>C</kbd> to stop logging once you have noted the offset.

## Post-flight Procedure
1. Power on the target board.
2. Connect the debug probe to your computer and the target board.
3. Attach to the target using `probe-rs`, and make note of the system state:
    ```sh
    probe-rs attach target/thumbv8m.main-none-eabihf/release/chariot-fw | tee postflight_`date +%Y%m%d-%H%M%S`.log
    ```
    Hit <kbd>Ctrl</kbd> + <kbd>C</kbd> to detach.
4. Flash the `do-nothing` binary to prevent further writes to flash:
    ```sh
    cargo run -p do-nothing --release
    ```
5. Detach from the target: <kbd>Ctrl</kbd> + <kbd>C</kbd>.
6. Use `dump_flash` to dump the flash memory to a binary file:
    ```sh
    ./dump_flash <Offset>
    ```