//! GPS position reader using the MAX-M10S over I2C (Artemis BLE module).
//!
//! Wiring (Apollo3 pad numbers, using sparkfun-redboard-nano BSP pin names):
//!   MAX-M10S SDA → d17 (Apollo3 pad 25, IOM2 SDA)
//!   MAX-M10S SCL → d18 (Apollo3 pad 27, IOM2 SCL)
//!   MAX-M10S VCC → d8  (Apollo3 pad 38, GPIO power enable, LOW = on)
//!   MAX-M10S GND → GND
//!
//! Flash and view RTT output with probe-rs:
//!   cargo build --release
//!   probe-rs run --chip AMA3B1KK-KBR \
//!       target/thumbv7em-none-eabihf/release/example-sfy

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use cortex_m_rt::entry;
use ambiq_hal as hal;
use hal::prelude::*;
use hal::iom::i2c::{Iom2, Freq};

use max_m10s::MaxM10S;

#[entry]
fn main() -> ! {
    let mut dp = hal::pac::Peripherals::take().unwrap();
    let core = hal::pac::CorePeripherals::take().unwrap();

    let mut delay = hal::delay::Delay::new(core.SYST, &mut dp.CLKGEN);
    let pins = hal::gpio::Pins::new(dp.GPIO);

    let mut led = pins.d19.into_push_pull_output();

    // Power on the GPS module via d8 (Apollo3 pad 38).
    defmt::info!("enabling gps_pwr");
    let mut gps_pwr = pins.d8.into_push_pull_output();
    gps_pwr.set_low().unwrap();
    delay.delay_ms(2000u32);

    defmt::info!("setting up i2c");
    // I2C on IOM2: SDA = d17 (pad 25), SCL = d18 (pad 27).
    let mut i2c = Iom2::new(dp.IOM2, pins.d17, pins.d18, Freq::F100kHz);

    defmt::info!("Initialising MAX-M10S…");
    let mut gnss = loop {
        match MaxM10S::new(&mut i2c) {
            Ok(dev) => break dev,
            Err(_) => {
                defmt::warn!("device not found — retrying");
                delay.delay_ms(2000_u32);
            }
        }
    };

    loop {
        match gnss.init(&mut i2c) {
            Ok(()) => break,
            Err(e) => {
                defmt::warn!("init failed: {:?} — retrying", defmt::Debug2Format(&e));
                delay.delay_ms(1000u32);
            }
        }
    }

    gnss.set_output_rate(&mut i2c, 1).unwrap();
    gnss.enable_pvt(&mut i2c).unwrap();

    defmt::info!("Entering GPS read loop");
    loop {
        led.toggle().unwrap();

        match gnss.read_pvt(&mut i2c) {
            Ok(Some(pvt)) => {
                let lat_deg = pvt.lat / 10_000_000;
                let lat_frac = (pvt.lat.abs() % 10_000_000) / 10;
                let lon_deg = pvt.lon / 10_000_000;
                let lon_frac = (pvt.lon.abs() % 10_000_000) / 10;

                defmt::info!(
                    "fix={} sats={} lat={}.{:06} lon={}.{:06} hMSL={}mm hAcc={}mm",
                    pvt.fix_type,
                    pvt.num_sv,
                    lat_deg,
                    lat_frac,
                    lon_deg,
                    lon_frac,
                    pvt.height_msl_mm,
                    pvt.h_acc_mm,
                );
            }
            Ok(None) => {}
            Err(e) => {
                defmt::warn!("read_pvt error: {:?}", defmt::Debug2Format(&e));
            }
        }

        delay.delay_ms(200u32);
    }
}
