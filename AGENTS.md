# Main goal

* Write an embedded driver for the MAX-M10-S driver for embedded-hal for Rust.
* The driver should needs to work on the Artemis Ambiq Apollo3 module. The
    library is available in https://github.com/gauteh/ambiq-rs
* Use the example in https://github.com/gauteh/ambiq-rs/blob/main/boards/redboard-nano/examples/uart_relay.rs as an example.

Use the Arduino library in https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library as a starting point to port the driver to rust.

Use the repository:
 "https://github.com/gauteh/ism330dhcx" branch "gyro-accel-parse-range" as an
 example for how to write the driver. but this is not a strict requirement.

 Prefer the embedded-hal version supported by ambiq-rs currently.

## Constraints

* Only support the MAX-M10-S at first.
* Use I2C
* Only support booting or setting up the device, configuring output rate, configuring PPS rate, putting the device to
    sleep and resuming.

## Wiring

The example device (example-sfy project) uses the artemis ble module (not the
redboard-nano). The GPS is connected to the artemis ble module as follows:

use 100kHz baud rate.

* pad d25 connected to gps sda
* pad d27 connected to gps scl
* pad ad11 connected to gps ts
* pad d38 connected to gps V_IO and VCC, to power the device on. pin LOW means
    power on.

# Tasks

## As of 2026-03-07:

* In src/lib.rs drain: return an enum on Ok, with variants for nothing ready and
    not yet initialized. make sure `read_pvt` returns None correctly when
    there's a ready PVT package.

* Check if parse_nav_pvt and similar may be written in more idiomatic rust. Add
    asserts where array/slice length is not checked, if still there after
    idiomatic rust.

* Create a new seperate example (based on example-sfy) for using an GPIO interrupt for reading the
    ready PVT package, and time. See https://github.com/gauteh/sfy/blob/bf8750cc9d330700ee00b56536d825fb62a7b483/sfy-buoy/sfy-ext-gps/src/main.rs#L498 for an example of a GPIO interrupt.
