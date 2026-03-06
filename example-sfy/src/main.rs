//! GPS position reader for the SparkFun RedBoard Nano (Ambiq Apollo3).
//!
//! Wiring (RedBoard Nano):
//!   MAX-M10S TX → A0  (Apollo3 GPIO 13, UART1 RX)
//!   MAX-M10S RX → A16 (Apollo3 GPIO 12, UART1 TX)
//!   MAX-M10S VCC → 3.3 V
//!   MAX-M10S GND → GND
//!
//! Flash and view RTT output with probe-rs:
//!   cargo build --release
//!   probe-rs run --chip AMA3B1KK-KBR target/thumbv7em-none-eabihf/release/example-sfy

#![no_std]
#![no_main]

use defmt_rtt as _;   // RTT transport for defmt
use panic_probe as _; // defmt-aware panic handler

use cortex_m_rt::entry;
use ambiq_hal as hal;
use hal::prelude::*;

use embedded_io::{ErrorKind, ErrorType, Read as EioRead, Write as EioWrite};
use max_m10s::MaxM10S;

// -------------------------------------------------------------------------
// BlockingUart — adapts ambiq-hal's nb-based UART to embedded-io Read+Write
// -------------------------------------------------------------------------

/// Error type bridging the ambiq-hal UART to `embedded-io`.
#[derive(Debug, defmt::Format)]
enum UartError {
    /// A UART receive error occurred (framing, parity, overflow).
    Read,
}

impl embedded_io::Error for UartError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

/// Wraps any `embedded-hal 0.2` serial Read+Write implementation and exposes
/// blocking `embedded-io` Read+Write traits expected by `max-m10s`.
struct BlockingUart<S>(S);

impl<S> ErrorType for BlockingUart<S> {
    type Error = UartError;
}

impl<S> EioRead for BlockingUart<S>
where
    S: embedded_hal::serial::Read<u8>,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, UartError> {
        if buf.is_empty() {
            return Ok(0);
        }
        // Block until one byte is available.
        let b = nb::block!(self.0.read()).map_err(|_| UartError::Read)?;
        buf[0] = b;
        Ok(1)
    }
}

impl<S> EioWrite for BlockingUart<S>
where
    S: embedded_hal::serial::Write<u8>,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, UartError> {
        for &b in buf {
            // Infallible write (ambiq Write error = `!`), so map_err is unreachable.
            nb::block!(self.0.write(b)).unwrap_or(());
        }
        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), UartError> {
        nb::block!(self.0.flush()).unwrap_or(());
        Ok(())
    }
}

// -------------------------------------------------------------------------
// Entry point
// -------------------------------------------------------------------------

#[entry]
fn main() -> ! {
    let mut dp = hal::pac::Peripherals::take().unwrap();
    let core = hal::pac::CorePeripherals::take().unwrap();

    let mut delay = hal::delay::Delay::new(core.SYST, &mut dp.CLKGEN);
    let pins = hal::gpio::Pins::new(dp.GPIO);

    let mut led = pins.d19.into_push_pull_output();

    // GPS UART: UART1 on Apollo3 GPIO 12 (TX, board pin A16) and 13 (RX, board pin A0).
    // The MAX-M10S default baud rate is 9600.
    let gps_uart = hal::uart::new_12_13(dp.UART1, pins.a16, pins.a0, 9600);

    let mut gnss = MaxM10S::new(BlockingUart(gps_uart));

    defmt::info!("Initialising MAX-M10S…");
    loop {
        match gnss.init() {
            Ok(()) => break,
            Err(e) => {
                defmt::warn!("init failed: {:?} — retrying", defmt::Debug2Format(&e));
                delay.delay_ms(1000u32);
            }
        }
    }

    defmt::info!("Setting output rate to 1 Hz");
    gnss.set_output_rate(1).unwrap();

    defmt::info!("Enabling NAV-PVT output");
    gnss.enable_pvt_output().unwrap();

    defmt::info!("Entering GPS read loop");
    loop {
        led.toggle().unwrap();

        match gnss.read_pvt() {
            Ok(pvt) => {
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
            Err(e) => {
                defmt::warn!("read_pvt error: {:?}", defmt::Debug2Format(&e));
            }
        }
    }
}
