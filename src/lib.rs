#![no_std]
#![deny(missing_docs)]
//! Embedded driver for the u-blox MAX-M10S GNSS module.
//!
//! Uses the `embedded-hal` 0.2 blocking I2C interface. The struct does not own
//! the I2C bus — each method accepts a `&mut I2C` reference so the bus can be
//! shared with other devices.
//!
//! # Example
//!
//! ```no_run
//! use max_m10s::MaxM10S;
//! // i2c implements embedded_hal 0.2 blocking I2C traits
//! # fn run<I2C, E: core::fmt::Debug>(mut i2c: I2C)
//! # where
//! #   I2C: embedded_hal::blocking::i2c::Write<Error=E>
//! #      + embedded_hal::blocking::i2c::Read<Error=E>
//! #      + embedded_hal::blocking::i2c::WriteRead<Error=E>
//! # {
//! let mut gnss = MaxM10S::new(&mut i2c).unwrap();
//! gnss.set_output_rate(&mut i2c, 1).unwrap();
//! gnss.enable_pvt(&mut i2c).unwrap();
//! # }
//! ```

pub mod ubx;

use ubx::{
    encode_ubx, parse_nav_pvt, parse_ubx_response, CfgMsg, CfgRate, CfgTp5, RxmPmReq,
    NavPvt, ParseError, CLASS_CFG, CLASS_MON, CLASS_NAV,
    ID_CFG_RATE, ID_CFG_TP5, ID_CFG_MSG, ID_MON_VER, ID_NAV_PVT,
};

use embedded_hal::blocking::i2c::{Read, Write, WriteRead};

/// Default I2C address for the u-blox MAX-M10S.
pub const ADDR: u8 = 0x42;

/// I2C register for the bytes-available counter (MSB; LSB follows at 0xFE).
/// After reading both registers the internal pointer advances to 0xFF (data stream).
const REG_BYTES_AVAIL: u8 = 0xFD;

/// Maximum bytes per I2C read transaction (matches u-blox recommended chunk size).
const CHUNK: usize = 32;

/// Internal receive buffer — large enough to hold several UBX packets.
const RX_BUF: usize = 256;

/// Driver error type.
#[derive(Debug)]
pub enum Error<E> {
    /// Underlying I2C bus error.
    I2c(E),
    /// No device found at the expected I2C address.
    NoDevice,
    /// The device replied with ACK-NAK (command rejected).
    Nak,
    /// Timed out waiting for an ACK or response.
    Timeout,
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::I2c(e)
    }
}

/// Driver for the u-blox MAX-M10S GNSS module over I2C (DDC interface).
pub struct MaxM10S {
    address: u8,
}

impl MaxM10S {
    /// Probe the I2C bus for the device and return a driver instance.
    ///
    /// Returns `Err(Error::NoDevice)` if the device does not ACK its address.
    pub fn new<I2C, E>(i2c: &mut I2C) -> Result<Self, Error<E>>
    where
        I2C: Write<Error = E>,
    {
        i2c.write(ADDR, &[]).map_err(|_| Error::NoDevice)?;
        Ok(Self { address: ADDR })
    }

    /// Verify communication by polling MON-VER and waiting for a response.
    pub fn init<I2C, E>(&mut self, i2c: &mut I2C) -> Result<(), Error<E>>
    where
        I2C: Write<Error = E> + Read<Error = E> + WriteRead<Error = E>,
    {
        let mut buf = [0u8; 8];
        let n = encode_ubx(CLASS_MON, ID_MON_VER, &[], &mut buf);
        i2c.write(self.address, &buf[..n])?;
        self.wait_ack(i2c, CLASS_MON, ID_MON_VER)
    }

    /// Set the navigation measurement rate.
    ///
    /// `rate_hz` is clamped to 1–10 Hz.
    pub fn set_output_rate<I2C, E>(&mut self, i2c: &mut I2C, rate_hz: u8) -> Result<(), Error<E>>
    where
        I2C: Write<Error = E> + Read<Error = E> + WriteRead<Error = E>,
    {
        let meas_rate_ms = 1000u16 / (rate_hz.max(1).min(10) as u16);
        let rate = CfgRate { meas_rate_ms, nav_rate: 1, time_ref: 0 };
        let mut buf = [0u8; 16];
        let n = rate.encode(&mut buf);
        i2c.write(self.address, &buf[..n])?;
        self.wait_ack(i2c, CLASS_CFG, ID_CFG_RATE)
    }

    /// Configure the timepulse (PPS) output.
    ///
    /// `interval_us`: period in microseconds (e.g. `1_000_000` for 1 Hz).
    /// `pulse_len_us`: pulse width in microseconds.
    pub fn set_pps_rate<I2C, E>(
        &mut self,
        i2c: &mut I2C,
        interval_us: u32,
        pulse_len_us: u32,
    ) -> Result<(), Error<E>>
    where
        I2C: Write<Error = E> + Read<Error = E> + WriteRead<Error = E>,
    {
        let tp5 = CfgTp5 { tp_idx: 0, interval_us, pulse_len_us, active: true };
        let mut buf = [0u8; 48];
        let n = tp5.encode(&mut buf);
        i2c.write(self.address, &buf[..n])?;
        self.wait_ack(i2c, CLASS_CFG, ID_CFG_TP5)
    }

    /// Enable `UBX-NAV-PVT` output on the I2C port at every navigation fix.
    pub fn enable_pvt<I2C, E>(&mut self, i2c: &mut I2C) -> Result<(), Error<E>>
    where
        I2C: Write<Error = E> + Read<Error = E> + WriteRead<Error = E>,
    {
        let msg = CfgMsg { msg_class: CLASS_NAV, msg_id: ID_NAV_PVT, i2c_rate: 1 };
        let mut buf = [0u8; 16];
        let n = msg.encode(&mut buf);
        i2c.write(self.address, &buf[..n])?;
        self.wait_ack(i2c, CLASS_CFG, ID_CFG_MSG)
    }

    /// Send the device into indefinite backup sleep (`UBX-RXM-PMREQ`).
    ///
    /// No ACK is returned — the device enters sleep immediately. Wake it by
    /// toggling the power control GPIO and calling [`resume`](Self::resume).
    pub fn sleep<I2C, E>(&mut self, i2c: &mut I2C) -> Result<(), Error<E>>
    where
        I2C: Write<Error = E>,
    {
        let req = RxmPmReq::backup();
        let mut buf = [0u8; 24];
        let n = req.encode(&mut buf);
        i2c.write(self.address, &buf[..n])?;
        Ok(())
    }

    /// Poll until the device ACKs an I2C write after waking from sleep.
    ///
    /// The caller must have already toggled the power control GPIO. Returns
    /// `Err(Error::Timeout)` if the device does not respond within ~30 tries.
    pub fn resume<I2C, E>(&mut self, i2c: &mut I2C) -> Result<(), Error<E>>
    where
        I2C: Write<Error = E>,
    {
        for _ in 0..30 {
            if i2c.write(self.address, &[]).is_ok() {
                return Ok(());
            }
        }
        Err(Error::Timeout)
    }

    /// Read and parse one `UBX-NAV-PVT` message from the device output stream.
    ///
    /// Returns `Ok(None)` when no complete PVT message is currently available.
    pub fn read_pvt<I2C, E>(&mut self, i2c: &mut I2C) -> Result<Option<NavPvt>, Error<E>>
    where
        I2C: Read<Error = E> + WriteRead<Error = E>,
    {
        let mut rx = [0u8; RX_BUF];
        let n = self.drain(i2c, &mut rx)?;
        defmt::debug!("rx buf: {:x} [{}]", rx, n);
        Ok(parse_nav_pvt(&rx[..n]))
    }

    // -----------------------------------------------------------------
    // Private helpers
    // -----------------------------------------------------------------

    /// Read all bytes currently available from the device into `buf`.
    ///
    /// Checks registers 0xFD/0xFE for the byte count, then reads in 32-byte
    /// chunks from the data stream at 0xFF. Returns the number of bytes stored.
    fn drain<I2C, E>(&mut self, i2c: &mut I2C, buf: &mut [u8]) -> Result<usize, Error<E>>
    where
        I2C: Read<Error = E> + WriteRead<Error = E>,
    {
        let mut avail_bytes = [0u8; 2];
        i2c.write_read(self.address, &[REG_BYTES_AVAIL], &mut avail_bytes)?;
        let avail = u16::from_be_bytes(avail_bytes) as usize;
        // 0x0000 = nothing ready; 0xFFFF = not yet initialised
        if avail == 0 || avail == 0xFFFF {
            return Ok(0);
        }
        let to_read = avail.min(buf.len());
        let mut pos = 0;
        while pos < to_read {
            let chunk = (to_read - pos).min(CHUNK);
            i2c.read(self.address, &mut buf[pos..pos + chunk])?;
            pos += chunk;
        }
        Ok(pos)
    }

    /// Drain the I2C stream repeatedly, scanning for ACK/NAK for `(cls, id)`.
    ///
    /// Retries up to 50 times before returning `Err(Error::Timeout)`.
    fn wait_ack<I2C, E>(&mut self, i2c: &mut I2C, cls: u8, id: u8) -> Result<(), Error<E>>
    where
        I2C: Read<Error = E> + WriteRead<Error = E>,
    {
        let mut rx = [0u8; RX_BUF];
        let mut pos = 0;
        for _ in 0..50 {
            if rx.len() - pos < 16 {
                pos = 0;
            }
            let n = self.drain(i2c, &mut rx[pos..])?;
            pos += n;
            match parse_ubx_response(&rx[..pos], cls, id) {
                Ok(true) => return Ok(()),
                Err(ParseError::Nak) => return Err(Error::Nak),
                _ => {}
            }
        }
        Err(Error::Timeout)
    }
}
