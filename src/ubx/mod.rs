//! UBX binary protocol encoding and decoding for the MAX-M10S.
//!
//! Only the subset of messages needed for device management is implemented.

// UBX message class bytes
/// CFG (Configuration) message class.
pub const CLASS_CFG: u8 = 0x06;
/// MON (Monitoring) message class.
pub const CLASS_MON: u8 = 0x0A;
/// ACK (Acknowledgement) message class.
pub const CLASS_ACK: u8 = 0x05;
/// NAV (Navigation) message class.
pub const CLASS_NAV: u8 = 0x01;
/// RXM (Receiver Manager) message class.
pub const CLASS_RXM: u8 = 0x02;

// UBX message IDs
/// CFG-RATE message ID.
pub const ID_CFG_RATE: u8 = 0x08;
/// CFG-TP5 (timepulse) message ID.
pub const ID_CFG_TP5: u8 = 0x31;
/// CFG-MSG message ID.
pub const ID_CFG_MSG: u8 = 0x01;
/// MON-VER (receiver version) message ID.
pub const ID_MON_VER: u8 = 0x04;
/// ACK-ACK message ID.
pub const ID_ACK_ACK: u8 = 0x01;
/// ACK-NAK message ID.
pub const ID_ACK_NAK: u8 = 0x00;
/// NAV-PVT message ID.
pub const ID_NAV_PVT: u8 = 0x07;
/// RXM-PMREQ (power management request / sleep) message ID.
pub const ID_RXM_PMREQ: u8 = 0x41;

/// UBX sync characters.
const SYNC1: u8 = 0xB5;
const SYNC2: u8 = 0x62;

// -------------------------------------------------------------------------
// Message structs
// -------------------------------------------------------------------------

/// `UBX-CFG-RATE` — navigation measurement rate configuration.
pub struct CfgRate {
    /// Measurement rate in milliseconds.
    pub meas_rate_ms: u16,
    /// Navigation rate (number of measurements per navigation solution, usually 1).
    pub nav_rate: u16,
    /// Time system reference: 0 = UTC, 1 = GPS.
    pub time_ref: u16,
}

impl CfgRate {
    /// Encode into `out`. Returns the number of bytes written.
    pub fn encode(&self, out: &mut [u8]) -> usize {
        let payload = [
            (self.meas_rate_ms & 0xFF) as u8,
            (self.meas_rate_ms >> 8) as u8,
            (self.nav_rate & 0xFF) as u8,
            (self.nav_rate >> 8) as u8,
            (self.time_ref & 0xFF) as u8,
            (self.time_ref >> 8) as u8,
        ];
        encode_ubx(CLASS_CFG, ID_CFG_RATE, &payload, out)
    }
}

/// `UBX-CFG-TP5` — timepulse (PPS) configuration for one timepulse channel.
pub struct CfgTp5 {
    /// Timepulse index (0 = TIMEPULSE, 1 = TIMEPULSE2).
    pub tp_idx: u8,
    /// Interval between pulses in microseconds.
    pub interval_us: u32,
    /// Pulse length in microseconds.
    pub pulse_len_us: u32,
    /// Whether to activate the timepulse output.
    pub active: bool,
}

impl CfgTp5 {
    /// Encode into `out`. Returns the number of bytes written.
    pub fn encode(&self, out: &mut [u8]) -> usize {
        let flags: u32 = if self.active { 0x77 } else { 0x00 };
        let mut payload = [0u8; 32];
        payload[0] = self.tp_idx;
        // bytes 1..7: reserved / cable delay (leave 0)
        payload[8..12].copy_from_slice(&self.interval_us.to_le_bytes()); // freqPeriod
        payload[12..16].copy_from_slice(&self.interval_us.to_le_bytes()); // freqPeriodLock
        payload[16..20].copy_from_slice(&self.pulse_len_us.to_le_bytes()); // pulseLenRatio
        payload[20..24].copy_from_slice(&self.pulse_len_us.to_le_bytes()); // pulseLenRatioLock
        // userConfigDelay (i32) at 24 — leave 0
        payload[28..32].copy_from_slice(&flags.to_le_bytes());
        encode_ubx(CLASS_CFG, ID_CFG_TP5, &payload, out)
    }
}

/// `UBX-CFG-MSG` — enable or disable a message on a specific port.
pub struct CfgMsg {
    /// Message class to configure.
    pub msg_class: u8,
    /// Message ID to configure.
    pub msg_id: u8,
    /// Output rate on the I2C (DDC) port (0 = off, 1 = every fix).
    pub i2c_rate: u8,
}

impl CfgMsg {
    /// Encode into `out`. Returns the number of bytes written.
    pub fn encode(&self, out: &mut [u8]) -> usize {
        let payload = [
            self.msg_class,
            self.msg_id,
            self.i2c_rate, // I2C / DDC
            0x00,          // UART1
            0x00,          // UART2
            0x00,          // USB
            0x00,          // SPI
            0x00,          // reserved
        ];
        encode_ubx(CLASS_CFG, ID_CFG_MSG, &payload, out)
    }
}

/// `UBX-RXM-PMREQ` — put the receiver into backup (sleep) mode.
///
/// Duration of 0 means sleep indefinitely until an external wake event.
pub struct RxmPmReq {
    /// Sleep duration in milliseconds (0 = indefinite).
    pub duration_ms: u32,
}

impl RxmPmReq {
    /// Create a request for indefinite backup sleep.
    pub fn backup() -> Self {
        Self { duration_ms: 0 }
    }

    /// Encode into `out`. Returns the number of bytes written.
    pub fn encode(&self, out: &mut [u8]) -> usize {
        let mut payload = [0u8; 8];
        payload[0..4].copy_from_slice(&self.duration_ms.to_le_bytes());
        // flags: bit 1 = backup
        payload[4..8].copy_from_slice(&2u32.to_le_bytes());
        encode_ubx(CLASS_RXM, ID_RXM_PMREQ, &payload, out)
    }
}

// -------------------------------------------------------------------------
// NAV-PVT message
// -------------------------------------------------------------------------

/// Parsed `UBX-NAV-PVT` position/velocity/time solution.
///
/// All fields match the u-blox M10 interface description (UBX-21035062).
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct NavPvt {
    /// UTC year (e.g. 2024). Valid when `valid` bit 0 is set.
    pub year: u16,
    /// UTC month (1–12). Valid when `valid` bit 0 is set.
    pub month: u8,
    /// UTC day of month (1–31). Valid when `valid` bit 0 is set.
    pub day: u8,
    /// UTC hour (0–23). Valid when `valid` bit 1 is set.
    pub hour: u8,
    /// UTC minute (0–59). Valid when `valid` bit 1 is set.
    pub min: u8,
    /// UTC second (0–60). Valid when `valid` bit 1 is set.
    pub sec: u8,
    /// Validity flags: bit 0=validDate, bit 1=validTime, bit 2=fullyResolved.
    pub valid: u8,
    /// Longitude in units of 1e-7 degrees (divide by 1e7 for degrees).
    pub lon: i32,
    /// Latitude in units of 1e-7 degrees (divide by 1e7 for degrees).
    pub lat: i32,
    /// Height above mean sea level in millimetres.
    pub height_msl_mm: i32,
    /// GNSS fix type: 0=no fix, 1=dead-reckoning, 2=2D, 3=3D, 4=GNSS+DR, 5=time-only.
    pub fix_type: u8,
    /// Number of satellites used in the navigation solution.
    pub num_sv: u8,
    /// Horizontal accuracy estimate in millimetres.
    pub h_acc_mm: u32,
}

/// Scan `buf` for the first valid `UBX-NAV-PVT` packet and return the parsed
/// position/time. Returns `None` if no complete, checksum-valid NAV-PVT is found.
pub fn parse_nav_pvt(buf: &[u8]) -> Option<NavPvt> {
    let mut i = 0;
    while i + 6 <= buf.len() {
        if buf[i] != SYNC1 || buf[i + 1] != SYNC2 {
            i += 1;
            continue;
        }
        let pkt_cls = *buf.get(i + 2)?;
        let pkt_id = *buf.get(i + 3)?;
        let pkt_len =
            u16::from_le_bytes([*buf.get(i + 4)?, *buf.get(i + 5)?]) as usize;
        let total = 6 + pkt_len + 2;

        if pkt_cls != CLASS_NAV || pkt_id != ID_NAV_PVT {
            i += if i + total <= buf.len() { total } else { 1 };
            continue;
        }
        if i + total > buf.len() || pkt_len < 92 {
            return None;
        }

        let payload = buf.get(i + 6..i + 6 + pkt_len)?;
        let (ck_a, ck_b) = checksum(pkt_cls, pkt_id, payload);
        if ck_a != *buf.get(i + 6 + pkt_len)? || ck_b != *buf.get(i + 6 + pkt_len + 1)? {
            i += 1;
            continue;
        }

        return Some(NavPvt {
            year:          u16::from_le_bytes(payload.get(4..6)?.try_into().ok()?),
            month:         *payload.get(6)?,
            day:           *payload.get(7)?,
            hour:          *payload.get(8)?,
            min:           *payload.get(9)?,
            sec:           *payload.get(10)?,
            valid:         *payload.get(11)?,
            fix_type:      *payload.get(20)?,
            num_sv:        *payload.get(23)?,
            lon:           i32::from_le_bytes(payload.get(24..28)?.try_into().ok()?),
            lat:           i32::from_le_bytes(payload.get(28..32)?.try_into().ok()?),
            height_msl_mm: i32::from_le_bytes(payload.get(36..40)?.try_into().ok()?),
            h_acc_mm:      u32::from_le_bytes(payload.get(40..44)?.try_into().ok()?),
        });
    }
    None
}

// -------------------------------------------------------------------------
// Encoding helpers
// -------------------------------------------------------------------------

/// Encode a UBX packet into `out` and return the total byte count.
///
/// `out` must be at least `8 + payload.len()` bytes.
pub fn encode_ubx(cls: u8, id: u8, payload: &[u8], out: &mut [u8]) -> usize {
    let n = payload.len();
    out[0] = SYNC1;
    out[1] = SYNC2;
    out[2] = cls;
    out[3] = id;
    out[4] = (n & 0xFF) as u8;
    out[5] = (n >> 8) as u8;
    out[6..6 + n].copy_from_slice(payload);
    let (ck_a, ck_b) = checksum(cls, id, payload);
    out[6 + n] = ck_a;
    out[7 + n] = ck_b;
    8 + n
}

/// Calculate UBX 8-bit Fletcher checksum over class, id, length, and payload.
pub fn checksum(cls: u8, id: u8, payload: &[u8]) -> (u8, u8) {
    let len = payload.len() as u16;
    let mut ck_a: u8 = 0;
    let mut ck_b: u8 = 0;
    for &b in &[cls, id, (len & 0xFF) as u8, (len >> 8) as u8] {
        ck_a = ck_a.wrapping_add(b);
        ck_b = ck_b.wrapping_add(ck_a);
    }
    for &b in payload {
        ck_a = ck_a.wrapping_add(b);
        ck_b = ck_b.wrapping_add(ck_a);
    }
    (ck_a, ck_b)
}

// -------------------------------------------------------------------------
// Parsing helpers
// -------------------------------------------------------------------------

/// Errors that can occur while parsing a UBX response.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum ParseError {
    /// Not enough data yet.
    Incomplete,
    /// Received ACK-NAK.
    Nak,
    /// Checksum mismatch.
    Checksum,
    /// Unrecognised packet structure.
    Invalid,
}

/// Scan `buf` for a UBX ACK-ACK (or direct response) for the given `(cls, id)`.
///
/// Returns `Ok(true)` when found, `Ok(false)` if not yet present,
/// `Err(ParseError::Nak)` on ACK-NAK, `Err(ParseError::Checksum)` on bad checksum.
pub fn parse_ubx_response(buf: &[u8], cls: u8, id: u8) -> Result<bool, ParseError> {
    let mut i = 0;
    while i + 6 <= buf.len() {
        if buf[i] != SYNC1 || buf[i + 1] != SYNC2 {
            i += 1;
            continue;
        }
        let pkt_cls = buf[i + 2];
        let pkt_id = buf[i + 3];
        let pkt_len = u16::from_le_bytes([buf[i + 4], buf[i + 5]]) as usize;
        let total = 6 + pkt_len + 2;

        if i + total > buf.len() {
            return Ok(false);
        }

        let payload = &buf[i + 6..i + 6 + pkt_len];
        let (ck_a, ck_b) = checksum(pkt_cls, pkt_id, payload);
        if ck_a != buf[i + 6 + pkt_len] || ck_b != buf[i + 6 + pkt_len + 1] {
            return Err(ParseError::Checksum);
        }

        if pkt_cls == CLASS_ACK && pkt_id == ID_ACK_ACK && pkt_len == 2 {
            if payload[0] == cls && payload[1] == id {
                return Ok(true);
            }
        }
        if pkt_cls == CLASS_ACK && pkt_id == ID_ACK_NAK && pkt_len == 2 {
            if payload[0] == cls && payload[1] == id {
                return Err(ParseError::Nak);
            }
        }
        if pkt_cls == cls && pkt_id == id {
            return Ok(true);
        }

        i += total;
    }
    Ok(false)
}

// -------------------------------------------------------------------------
// Tests
// -------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_checksum_cfg_rate() {
        // Known-good CFG-RATE for 1 Hz: meas=1000ms, nav=1, ref=UTC
        let payload = [0xE8u8, 0x03, 0x01, 0x00, 0x00, 0x00];
        let (ck_a, ck_b) = checksum(CLASS_CFG, ID_CFG_RATE, &payload);
        assert_eq!(ck_a, 0x00);
        assert_eq!(ck_b, 0x37);
    }

    #[test]
    fn test_encode_decode_ack() {
        let ack_payload = [CLASS_CFG, ID_CFG_RATE];
        let mut buf = [0u8; 16];
        let n = encode_ubx(CLASS_ACK, ID_ACK_ACK, &ack_payload, &mut buf);
        let result = parse_ubx_response(&buf[..n], CLASS_CFG, ID_CFG_RATE);
        assert_eq!(result, Ok(true));
    }

    #[test]
    fn test_encode_decode_nak() {
        let nak_payload = [CLASS_CFG, ID_CFG_RATE];
        let mut buf = [0u8; 16];
        let n = encode_ubx(CLASS_ACK, ID_ACK_NAK, &nak_payload, &mut buf);
        let result = parse_ubx_response(&buf[..n], CLASS_CFG, ID_CFG_RATE);
        assert_eq!(result, Err(ParseError::Nak));
    }
}
