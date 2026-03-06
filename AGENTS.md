# Main goal

* Write an embedded driver for the MAX-M10-S driver for embedded-hal for Rust.
* The driver should needs to work on the Artemis Ambiq Apollo3 module. The
    library is available in https://github.com/gauteh/ambiq-rs
* Use the example in https://github.com/gauteh/ambiq-rs/blob/main/boards/redboard-nano/examples/uart_relay.rs as an example.

Use the Arduino library in https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library as a starting point to port the driver to rust.

## Constraints

* Only support the MAX-M10-S at first.
* Only support booting or setting up the device, configuring output rate,
    configuring PPS rate, and changing to UART output, putting the device to
    sleep and resuming.
