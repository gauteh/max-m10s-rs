//! GPS timepulse interrupt example (Artemis BLE module).
//!
//! The GPS timepulse output (TS pin) fires once per second when the device
//! has a valid GNSS fix. This example configures a GPIO interrupt on the TS
//! pin so the CPU wakes from `wfi` only when new GPS time/position data is
//! ready rather than polling in a busy loop.
//!
//! Wiring (Apollo3 pad numbers / redboard-nano BSP pin names):
//!   MAX-M10S SDA  → d17  (pad 25, IOM2 SDA)
//!   MAX-M10S SCL  → d18  (pad 27, IOM2 SCL)
//!   MAX-M10S VCC  → d8   (pad 38, GPIO power enable, LOW = on)
//!   MAX-M10S TS   → a2   (pad 11, GPIO interrupt input, rising edge = PVT ready)
//!   MAX-M10S GND  → GND
//!
//! Flash and view RTT output with probe-rs:
//!   cargo build --release --bin interrupt
//!   probe-rs run --chip AMA3B1KK-KBR \
//!       target/thumbv7em-none-eabihf/release/interrupt

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};

use cortex_m::interrupt::{free, Mutex};
use cortex_m::asm;
use cortex_m_rt::entry;

use ambiq_hal as hal;
use hal::gpio::{InterruptOpt, Mode};
use hal::iom::i2c::{Freq, Iom2};
use hal::pac::interrupt;
use hal::prelude::*;

use max_m10s::MaxM10S;

// ---------------------------------------------------------------------------
// Shared state between main and the GPIO interrupt handler
// ---------------------------------------------------------------------------

/// Set to `true` by the GPIO ISR each time the timepulse fires.
static PVT_READY: AtomicBool = AtomicBool::new(false);

/// The TS interrupt pin, held in a critical-section-protected cell so that
/// the interrupt handler can clear and re-arm it.
static TS_PIN: Mutex<RefCell<Option<hal::gpio::pin::Pin<11, { Mode::Input }>>>> =
    Mutex::new(RefCell::new(None));

// ---------------------------------------------------------------------------
// GPIO interrupt handler
// ---------------------------------------------------------------------------

#[allow(non_snake_case)]
#[interrupt]
fn GPIO() {
    free(|cs| {
        if let Some(pin) = TS_PIN.borrow(cs).borrow_mut().as_mut() {
            pin.clear_interrupt();
            PVT_READY.store(true, Ordering::Release);
            pin.enable_interrupt();
        }
    });
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

#[entry]
fn main() -> ! {
    let mut dp = hal::pac::Peripherals::take().unwrap();
    let core = hal::pac::CorePeripherals::take().unwrap();

    let mut delay = hal::delay::Delay::new(core.SYST, &mut dp.CLKGEN);
    let pins = hal::gpio::Pins::new(dp.GPIO);

    let mut led = pins.d19.into_push_pull_output();

    // Power on the GPS module via d8 (pad 38). LOW = power on.
    let mut gps_pwr = pins.d8.into_push_pull_output();
    gps_pwr.set_low().unwrap();
    delay.delay_ms(100u32);

    // I2C on IOM2: SDA = d17 (pad 25), SCL = d18 (pad 27). 100 kHz per hardware spec.
    let mut i2c = Iom2::new(dp.IOM2, pins.d17, pins.d18, Freq::F100kHz);

    // Set up the TS interrupt pin: a2 = pad 11, rising edge (timepulse fires high).
    let mut ts = pins.a2.into_input();
    ts.configure_interrupt(InterruptOpt::LowToHigh);
    ts.clear_interrupt();
    // Don't enable yet — arm after GPS is configured.
    free(|cs| {
        TS_PIN.borrow(cs).replace(Some(ts));
    });

    defmt::info!("Initialising MAX-M10S…");
    let mut gnss = loop {
        match MaxM10S::new(&mut i2c) {
            Ok(dev) => break dev,
            Err(_) => {
                defmt::warn!("device not found — retrying");
                delay.delay_ms(500u32);
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
    // Configure PPS: 1 Hz period, 100 ms pulse, rising edge on TS pin.
    // Uses CFG-VALSET split across three ≤32-byte I2C frames.
    match gnss.set_pps_rate(&mut i2c, 1_000_000, 100_000) {
        Ok(()) => defmt::info!("PPS configured: 1 Hz, 100 ms pulse"),
        Err(e) => defmt::warn!("set_pps_rate failed: {:?}", defmt::Debug2Format(&e)),
    }
    gnss.enable_pvt(&mut i2c).unwrap();

    // Arm the TS interrupt and enable GPIO interrupts globally.
    free(|cs| {
        if let Some(pin) = TS_PIN.borrow(cs).borrow_mut().as_mut() {
            pin.enable_interrupt();
        }
    });
    free(|_cs| unsafe {
        hal::gpio::enable_gpio_interrupts();
        cortex_m::interrupt::enable();
    });

    defmt::info!("Waiting for GPS timepulse interrupts…");

    loop {
        // Sleep until the next interrupt.
        asm::wfi();

        if PVT_READY.swap(false, Ordering::Acquire) {
            led.toggle().unwrap();

            match gnss.read_pvt(&mut i2c) {
                Ok(Some(pvt)) => {
                    let time_valid = (pvt.valid & 0x03) == 0x03; // validDate + validTime

                    if time_valid {
                        defmt::info!(
                            "PPS @ {}-{:02}-{:02} {:02}:{:02}:{:02} UTC | fix={} sats={} hAcc={}mm",
                            pvt.year, pvt.month, pvt.day,
                            pvt.hour, pvt.min, pvt.sec,
                            pvt.fix_type, pvt.num_sv,
                            pvt.h_acc_mm,
                        );
                    } else {
                        defmt::info!(
                            "PPS (no valid time yet) | fix={} sats={}",
                            pvt.fix_type, pvt.num_sv,
                        );
                    }
                }
                Ok(None) => {
                    defmt::debug!("timepulse fired but no NAV-PVT in FIFO yet");
                }
                Err(e) => {
                    defmt::warn!("read_pvt error: {:?}", defmt::Debug2Format(&e));
                }
            }
        }
    }
}
