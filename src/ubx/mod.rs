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
/// NMEA standard message class (0xF0).
pub const CLASS_NMEA: u8 = 0xF0;
/// NAV (Navigation) message class.
pub const CLASS_NAV: u8 = 0x01;

// UBX message IDs
/// CFG-RATE message ID.
pub const ID_CFG_RATE: u8 = 0x08;
/// CFG-TP5 (timepulse) message ID.
pub const ID_CFG_TP5: u8 = 0x31;
/// CFG-PM2 (power management) message ID.
pub const ID_CFG_PM2: u8 = 0x3B;
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
    /// Encode to a UBX packet.
    pub fn encode(&self) -> heapless::Vec<u8, 16> {
        let payload = [
            (self.meas_rate_ms & 0xFF) as u8,
            (self.meas_rate_ms >> 8) as u8,
            (self.nav_rate & 0xFF) as u8,
            (self.nav_rate >> 8) as u8,
            (self.time_ref & 0xFF) as u8,
            (self.time_ref >> 8) as u8,
        ];
        encode_ubx(CLASS_CFG, ID_CFG_RATE, &payload)
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
    /// Encode to a UBX packet.
    pub fn encode(&self) -> heapless::Vec<u8, 48> {
        let flags: u32 = if self.active { 0x77 } else { 0x00 };
        let mut payload = [0u8; 32];
        payload[0] = self.tp_idx;
        // bytes 1..3: reserved
        // antCableDelay (i16) at offset 4 — leave 0
        // rfGroupDelay  (i16) at offset 6 — leave 0
        payload[8..12].copy_from_slice(&self.interval_us.to_le_bytes());  // freqPeriod (locked)
        payload[12..16].copy_from_slice(&self.interval_us.to_le_bytes()); // freqPeriodLock
        payload[16..20].copy_from_slice(&self.pulse_len_us.to_le_bytes()); // pulseLenRatio
        payload[20..24].copy_from_slice(&self.pulse_len_us.to_le_bytes()); // pulseLenRatioLock
        // userConfigDelay (i32) at 24 — leave 0
        payload[28..32].copy_from_slice(&flags.to_le_bytes());
        encode_ubx(CLASS_CFG, ID_CFG_TP5, &payload)
    }
}

/// `UBX-CFG-PM2` — power management configuration.
pub struct CfgPm2 {
    /// Internal flags field.
    pub flags: u32,
    /// Update period in milliseconds (for cyclic tracking mode).
    pub update_period_ms: u32,
    /// Search period in milliseconds.
    pub search_period_ms: u32,
}

impl CfgPm2 {
    /// Create a backup/sleep configuration.
    pub fn backup() -> Self {
        Self {
            // bit 1 = extintWake disable, bits 4..5 = doNotEnterOff
            flags: 0x00000002,
            update_period_ms: 1000,
            search_period_ms: 10000,
        }
    }

    /// Encode to a UBX packet.
    pub fn encode(&self) -> heapless::Vec<u8, 64> {
        let mut payload = [0u8; 44];
        payload[0] = 0x01; // version
        // bytes 1..3: reserved
        payload[4..8].copy_from_slice(&self.flags.to_le_bytes());
        payload[8..12].copy_from_slice(&self.update_period_ms.to_le_bytes());
        payload[12..16].copy_from_slice(&self.search_period_ms.to_le_bytes());
        encode_ubx(CLASS_CFG, ID_CFG_PM2, &payload)
    }
}

/// `UBX-CFG-MSG` — enable or disable an output message on a port.
pub struct CfgMsg {
    /// Message class to configure.
    pub msg_class: u8,
    /// Message ID to configure.
    pub msg_id: u8,
    /// Output rate (0 = off).
    pub rate: u8,
}

impl CfgMsg {
    /// Encode to a UBX packet (sets rate on all 6 ports simultaneously).
    pub fn encode(&self) -> heapless::Vec<u8, 16> {
        let payload = [
            self.msg_class,
            self.msg_id,
            self.rate, // I2C
            self.rate, // UART1
            self.rate, // UART2
            self.rate, // USB
            self.rate, // SPI
            0x00,      // reserved
        ];
        encode_ubx(CLASS_CFG, ID_CFG_MSG, &payload)
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
    /// Validity flags (bit 0=validDate, bit 1=validTime, bit 2=fullyResolved).
    pub valid: u8,
    /// Horizontal accuracy estimate in millimetres.
    pub h_acc_mm: u32,
}

/// Scan `buf` for the first valid `UBX-NAV-PVT` packet and return the parsed
/// position. Returns `None` if no complete, checksum-valid NAV-PVT is found.
pub fn parse_nav_pvt(buf: &[u8]) -> Option<NavPvt> {
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

        if pkt_cls != CLASS_NAV || pkt_id != ID_NAV_PVT {
            i += total;
            continue;
        }
        if i + total > buf.len() || pkt_len < 92 {
            return None;
        }

        let payload = &buf[i + 6..i + 6 + pkt_len];
        let (ck_a, ck_b) = checksum(pkt_cls, pkt_id, payload);
        if ck_a != buf[i + 6 + pkt_len] || ck_b != buf[i + 6 + pkt_len + 1] {
            i += 1;
            continue;
        }

        return Some(NavPvt {
            lon: i32::from_le_bytes(payload[24..28].try_into().unwrap()),
            lat: i32::from_le_bytes(payload[28..32].try_into().unwrap()),
            height_msl_mm: i32::from_le_bytes(payload[36..40].try_into().unwrap()),
            fix_type: payload[20],
            num_sv: payload[23],
            valid: payload[11],
            h_acc_mm: u32::from_le_bytes(payload[40..44].try_into().unwrap()),
        });
    }
    None
}

// -------------------------------------------------------------------------
// Encoding helpers
// -------------------------------------------------------------------------

/// Encode a UBX packet with the given class, id, and payload.
///
/// Returns a `heapless::Vec` containing: sync1, sync2, class, id, len_lo, len_hi,
/// payload bytes, ck_a, ck_b.
pub fn encode_ubx<const N: usize>(cls: u8, id: u8, payload: &[u8]) -> heapless::Vec<u8, N> {
    let mut pkt: heapless::Vec<u8, N> = heapless::Vec::new();
    let len = payload.len() as u16;

    let _ = pkt.push(SYNC1);
    let _ = pkt.push(SYNC2);
    let _ = pkt.push(cls);
    let _ = pkt.push(id);
    let _ = pkt.push((len & 0xFF) as u8);
    let _ = pkt.push((len >> 8) as u8);
    for &b in payload {
        let _ = pkt.push(b);
    }

    let (ck_a, ck_b) = checksum(cls, id, payload);
    let _ = pkt.push(ck_a);
    let _ = pkt.push(ck_b);

    pkt
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

/// Scan `buf` for a UBX packet matching either:
/// - ACK-ACK for `(cls, id)` → `Ok(true)`
/// - A response packet with class `cls` and id `id` → `Ok(true)`
///
/// Returns `Ok(false)` if no relevant packet found yet,
/// `Err(ParseError::Nak)` for ACK-NAK, `Err(ParseError::Checksum)` on bad checksum.
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
            return Ok(false); // incomplete
        }

        let payload = &buf[i + 6..i + 6 + pkt_len];
        let (ck_a, ck_b) = checksum(pkt_cls, pkt_id, payload);
        if ck_a != buf[i + 6 + pkt_len] || ck_b != buf[i + 6 + pkt_len + 1] {
            return Err(ParseError::Checksum);
        }

        // ACK-ACK: class=0x05 id=0x01, payload=[cls, id]
        if pkt_cls == CLASS_ACK && pkt_id == ID_ACK_ACK && pkt_len == 2 {
            if payload[0] == cls && payload[1] == id {
                return Ok(true);
            }
        }

        // ACK-NAK: class=0x05 id=0x00
        if pkt_cls == CLASS_ACK && pkt_id == ID_ACK_NAK && pkt_len == 2 {
            if payload[0] == cls && payload[1] == id {
                return Err(ParseError::Nak);
            }
        }

        // Direct response match (e.g. MON-VER)
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
        // Build a synthetic ACK-ACK for CFG-RATE.
        let ack_payload = [CLASS_CFG, ID_CFG_RATE];
        let pkt = encode_ubx::<16>(CLASS_ACK, ID_ACK_ACK, &ack_payload);
        let result = parse_ubx_response(&pkt, CLASS_CFG, ID_CFG_RATE);
        assert_eq!(result, Ok(true));
    }

    #[test]
    fn test_encode_decode_nak() {
        let nak_payload = [CLASS_CFG, ID_CFG_RATE];
        let pkt = encode_ubx::<16>(CLASS_ACK, ID_ACK_NAK, &nak_payload);
        let result = parse_ubx_response(&pkt, CLASS_CFG, ID_CFG_RATE);
        assert_eq!(result, Err(ParseError::Nak));
    }
}
