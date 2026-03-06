# max-m10s

`no_std` embedded-hal driver for the [u-blox MAX-M10S](https://www.u-blox.com/en/product/max-m10-series) GNSS module.

## Features

- Device initialisation / boot
- Configure navigation output rate (Hz)
- Configure PPS (timepulse) interval and pulse length
- Disable NMEA / switch to UBX-only output on UART
- Put device to sleep (backup power-save mode)
- Wake device from sleep

## Usage

```rust,ignore
use max_m10s::MaxM10S;

let mut gnss = MaxM10S::new(uart);
gnss.init().unwrap();

// 1 Hz navigation output
gnss.set_output_rate(1).unwrap();

// 1 second PPS with 100 ms pulse
gnss.set_pps_rate(1_000_000, 100_000).unwrap();

// Sleep and wake
gnss.sleep().unwrap();
// ... later ...
gnss.wake().unwrap();
```

## Compatibility

Targets the Ambiq Apollo3 platform via [`ambiq-rs`](https://github.com/gauteh/ambiq-rs),
but works with any `embedded-io` `Read + Write` UART implementation.

## References

- [MAX-M10S product page](https://www.u-blox.com/en/product/max-m10-series)
- [u-blox M10 receiver interface description (UBX-21035062)](https://www.u-blox.com/sites/default/files/documents/u-blox-M10_ReceiverDescription_UBX-21035062.pdf)
- [SparkFun Arduino library (reference)](https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library)

## License

MIT
