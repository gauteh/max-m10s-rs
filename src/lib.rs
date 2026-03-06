//! Embedded driver for the u-blox MAX-M10S GNSS module.
//!
//! # Overview
//!
//! This crate provides a `no_std` driver for the u-blox MAX-M10S GNSS module,
//! targeting embedded platforms using [`embedded-hal`] v1 and [`embedded-io`].
//!
//! Communication is performed over UART using the UBX binary protocol.
//!
//! # Supported features
//!
//! - Device initialisation / boot
//! - Configure NMEA/UBX output message rate
//! - Configure PPS (timepulse) rate
//! - Switch to UART-only output
//! - Put device to sleep (power save mode)
//! - Resume device from sleep
//!
//! # Quick start
//!
//! ```rust,ignore
//! use max_m10s::MaxM10S;
//!
//! let mut gnss = MaxM10S::new(uart);
//! gnss.init().unwrap();
//! gnss.set_output_rate(1).unwrap(); // 1 Hz
//! ```
//!
//! # References
//!
//! - [MAX-M10S product page](https://www.u-blox.com/en/product/max-m10-series)
//! - [u-blox M10 receiver interface description](https://www.u-blox.com/sites/default/files/documents/u-blox-M10_ReceiverDescription_UBX-21035062.pdf)

#![cfg_attr(not(test), no_std)]
#![deny(missing_docs)]

pub mod ubx;

use embedded_io::{Read, Write};
use ubx::{CfgMsg, CfgPm2, CfgRate, CfgTp5, NavPvt, encode_ubx, parse_ubx_response};

/// Errors returned by this driver.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<E> {
    /// Underlying I/O error.
    Io(E),
    /// A UBX acknowledgement was not received.
    NoAck,
    /// Response checksum mismatch.
    ChecksumError,
    /// Received an unexpected or malformed packet.
    InvalidResponse,
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::Io(e)
    }
}

/// Driver for the u-blox MAX-M10S GNSS module.
pub struct MaxM10S<UART> {
    uart: UART,
}

impl<UART, E> MaxM10S<UART>
where
    UART: Read<Error = E> + Write<Error = E>,
{
    /// Create a new driver instance, taking ownership of the UART peripheral.
    pub fn new(uart: UART) -> Self {
        Self { uart }
    }

    /// Consume the driver and return the inner UART.
    pub fn release(self) -> UART {
        self.uart
    }

    /// Initialise the device: disable NMEA on UART, enable UBX protocol, and
    /// verify communication by polling the receiver version.
    pub fn init(&mut self) -> Result<(), Error<E>> {
        // Disable all default NMEA messages on UART1 to reduce noise.
        self.disable_nmea()?;
        // Confirm UBX comms are working.
        self.poll_version()?;
        Ok(())
    }

    /// Set the navigation/output message rate in Hz (1–10).
    ///
    /// This sends a `CFG-RATE` message with `measRate = 1000 / rate_hz` ms.
    pub fn set_output_rate(&mut self, rate_hz: u16) -> Result<(), Error<E>> {
        let meas_rate_ms = 1000u16 / rate_hz.max(1);
        let payload = CfgRate {
            meas_rate_ms,
            nav_rate: 1,
            time_ref: 0, // UTC
        };
        self.send_cfg(&payload.encode())?;
        self.wait_ack(ubx::CLASS_CFG, ubx::ID_CFG_RATE)
    }

    /// Set the PPS (timepulse) interval in microseconds.
    ///
    /// Sends a `CFG-TP5` message configuring timepulse 0.
    pub fn set_pps_rate(&mut self, interval_us: u32, pulse_len_us: u32) -> Result<(), Error<E>> {
        let payload = CfgTp5 {
            tp_idx: 0,
            interval_us,
            pulse_len_us,
            active: true,
        };
        self.send_cfg(&payload.encode())?;
        self.wait_ack(ubx::CLASS_CFG, ubx::ID_CFG_TP5)
    }

    /// Put the receiver into power-save mode (backup sleep).
    pub fn sleep(&mut self) -> Result<(), Error<E>> {
        let payload = CfgPm2::backup();
        self.send_cfg(&payload.encode())?;
        self.wait_ack(ubx::CLASS_CFG, ubx::ID_CFG_PM2)
    }

    /// Wake the receiver by sending a dummy byte and waiting for it to be ready.
    pub fn wake(&mut self) -> Result<(), Error<E>> {
        // Toggle UART line: send a dummy 0xFF byte to wake from backup mode.
        self.uart.write_all(&[0xFF])?;
        // Poll version to confirm the device is awake.
        self.poll_version()
    }

    /// Enable `UBX-NAV-PVT` output on UART at the configured message rate.
    ///
    /// Call this after [`init`](Self::init) to start receiving position fixes.
    /// Each navigation cycle will produce one NAV-PVT packet on the UART.
    pub fn enable_pvt_output(&mut self) -> Result<(), Error<E>> {
        let msg = CfgMsg {
            msg_class: ubx::CLASS_NAV,
            msg_id: ubx::ID_NAV_PVT,
            rate: 1,
        };
        let pkt = msg.encode();
        self.uart.write_all(&pkt)?;
        self.uart.flush()?;
        self.wait_ack(ubx::CLASS_CFG, ubx::ID_CFG_MSG)
    }

    /// Block until a valid `UBX-NAV-PVT` packet is received and return the
    /// parsed position/velocity/time solution.
    ///
    /// Call [`enable_pvt_output`](Self::enable_pvt_output) once before
    /// entering the read loop.
    pub fn read_pvt(&mut self) -> Result<NavPvt, Error<E>> {
        let mut buf = [0u8; 512];
        let mut pos = 0usize;

        for _ in 0..65536usize {
            let mut byte = [0u8; 1];
            if self.uart.read(&mut byte).map_err(Error::Io)? == 0 {
                continue;
            }
            buf[pos] = byte[0];
            pos += 1;

            if let Some(pvt) = ubx::parse_nav_pvt(&buf[..pos]) {
                return Ok(pvt);
            }

            if pos == buf.len() {
                // Keep the second half of the buffer to avoid discarding a
                // partially-received packet that straddles the boundary.
                buf.copy_within(256.., 0);
                pos = 256;
            }
        }
        Err(Error::InvalidResponse)
    }

    // -------------------------------------------------------------------------
    // Internal helpers
    // -------------------------------------------------------------------------

    /// Send a UBX CFG packet (class 0x06) and flush.
    fn send_cfg(&mut self, payload: &[u8]) -> Result<(), E> {
        self.uart.write_all(payload)?;
        self.uart.flush()
    }

    /// Poll `UBX-MON-VER` to verify the device responds.
    fn poll_version(&mut self) -> Result<(), Error<E>> {
        let pkt: heapless::Vec<u8, 8> = encode_ubx(ubx::CLASS_MON, ubx::ID_MON_VER, &[]);
        self.uart.write_all(&pkt)?;
        self.uart.flush()?;
        self.wait_ack(ubx::CLASS_MON, ubx::ID_MON_VER)
    }

    /// Disable all default NMEA output messages on UART1.
    fn disable_nmea(&mut self) -> Result<(), Error<E>> {
        const NMEA_IDS: &[u8] = &[0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09];
        for &id in NMEA_IDS {
            let msg = CfgMsg {
                msg_class: ubx::CLASS_NMEA,
                msg_id: id,
                rate: 0,
            };
            let pkt = msg.encode();
            self.uart.write_all(&pkt)?;
            self.uart.flush()?;
            self.wait_ack(ubx::CLASS_CFG, ubx::ID_CFG_MSG)?;
        }
        Ok(())
    }

    /// Read bytes from UART until a UBX ACK-ACK or ACK-NAK is received for the
    /// given class/id, or until a matching response packet is found.
    fn wait_ack(&mut self, cls: u8, id: u8) -> Result<(), Error<E>> {
        let mut buf = [0u8; 256];
        let mut pos = 0usize;

        // Simple finite-loop read to avoid blocking forever on no_std.
        for _ in 0..4096 {
            let mut byte = [0u8; 1];
            if self.uart.read(&mut byte).map_err(Error::Io)? == 0 {
                continue;
            }
            if pos < buf.len() {
                buf[pos] = byte[0];
                pos += 1;
            }

            // Try to parse from the beginning of the buffer.
            match parse_ubx_response(&buf[..pos], cls, id) {
                Ok(true) => return Ok(()),
                Ok(false) => {}
                Err(ubx::ParseError::Nak) => return Err(Error::NoAck),
                Err(ubx::ParseError::Checksum) => return Err(Error::ChecksumError),
                Err(_) => {}
            }
        }
        Err(Error::NoAck)
    }
}
