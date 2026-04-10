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
/// CFG-VALSET (key-value configuration set) message ID.
pub const ID_CFG_VALSET: u8 = 0x8A;
/// CFG-MSG message ID.
pub const ID_CFG_MSG: u8 = 0x01;

// CFG-VALSET key IDs for the timepulse (CFG-TP group = 0x0005, TP1).
// Units for period and pulse length are **microseconds**.
/// TP1 period in µs when NOT locked to GNSS (U4).
pub const KEY_TP_PERIOD_TP1: u32      = 0x40050002;
/// TP1 period in µs when locked to GNSS (U4).
pub const KEY_TP_PERIOD_LOCK_TP1: u32 = 0x40050003;
/// TP1 pulse length in µs when NOT locked to GNSS (U4).
pub const KEY_TP_LEN_TP1: u32         = 0x40050004;
/// TP1 pulse length in µs when locked to GNSS (U4).
pub const KEY_TP_LEN_LOCK_TP1: u32    = 0x40050005;
/// Enable / disable TP1 output (U1/bool: 0=off, 1=on).
pub const KEY_TP_TP1_ENA: u32         = 0x10050007;
/// TP1 output polarity (U1/bool: 0=falling, 1=rising).
pub const KEY_TP_POL_TP1: u32         = 0x1005000b;
/// Pulse-length interpretation: 0 = duty-cycle ratio, 1 = absolute length in µs.
pub const KEY_TP_PULSE_LENGTH_DEF: u32 = 0x20050030;
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

// CFG-VALSET key IDs for the GNSS signal configuration (CFG-SIGNAL group = 0x0031).
// All are type L1 (boolean, 1 byte).  Reference: M10 Interface Description UBX-21035062.
/// Enable GPS constellation (L1).
pub const KEY_SIGNAL_GPS_ENA: u32  = 0x1031001F;
/// Enable SBAS (L1).
pub const KEY_SIGNAL_SBAS_ENA: u32 = 0x10310020;
/// Enable Galileo constellation (L1).
pub const KEY_SIGNAL_GAL_ENA: u32  = 0x10310021;
/// Enable BeiDou constellation (L1).
pub const KEY_SIGNAL_BDS_ENA: u32  = 0x10310022;
/// Enable QZSS constellation (L1).
pub const KEY_SIGNAL_QZSS_ENA: u32 = 0x10310024;
/// Enable GLONASS constellation (L1).
pub const KEY_SIGNAL_GLO_ENA: u32  = 0x10310025;

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

/// `UBX-CFG-VALSET` — set TP1 period (unlocked + locked) in µs.
///
/// 28-byte frame, safe for Apollo3 IOM I2C FIFO.
pub fn encode_tp_period(interval_us: u32, out: &mut [u8]) -> usize {
    let mut payload = [0u8; 20];
    payload[0] = 0x00; // version: set in RAM
    payload[1] = 0x01; // layers: RAM
    payload[4..8].copy_from_slice(&KEY_TP_PERIOD_TP1.to_le_bytes());
    payload[8..12].copy_from_slice(&interval_us.to_le_bytes());
    payload[12..16].copy_from_slice(&KEY_TP_PERIOD_LOCK_TP1.to_le_bytes());
    payload[16..20].copy_from_slice(&interval_us.to_le_bytes());
    encode_ubx(CLASS_CFG, ID_CFG_VALSET, &payload, out)
}

/// `UBX-CFG-VALSET` — set TP1 pulse length (unlocked + locked) in µs.
///
/// 28-byte frame, safe for Apollo3 IOM I2C FIFO.
pub fn encode_tp_len(pulse_len_us: u32, out: &mut [u8]) -> usize {
    let mut payload = [0u8; 20];
    payload[0] = 0x00;
    payload[1] = 0x01;
    payload[4..8].copy_from_slice(&KEY_TP_LEN_TP1.to_le_bytes());
    payload[8..12].copy_from_slice(&pulse_len_us.to_le_bytes());
    payload[12..16].copy_from_slice(&KEY_TP_LEN_LOCK_TP1.to_le_bytes());
    payload[16..20].copy_from_slice(&pulse_len_us.to_le_bytes());
    encode_ubx(CLASS_CFG, ID_CFG_VALSET, &payload, out)
}

/// `UBX-CFG-VALSET` — enable TP1, set rising-edge polarity, and configure
/// absolute pulse length mode (not duty-cycle ratio).
///
/// 27-byte frame, safe for Apollo3 IOM I2C FIFO.
pub fn encode_tp_enable(out: &mut [u8]) -> usize {
    let mut payload = [0u8; 19];
    payload[0] = 0x00;
    payload[1] = 0x01;
    // TP1_ENA = 1
    payload[4..8].copy_from_slice(&KEY_TP_TP1_ENA.to_le_bytes());
    payload[8] = 1;
    // POL_TP1 = 1 (rising edge)
    payload[9..13].copy_from_slice(&KEY_TP_POL_TP1.to_le_bytes());
    payload[13] = 1;
    // PULSE_LENGTH_DEF = 1 (interpret LEN as µs, not ratio)
    payload[14..18].copy_from_slice(&KEY_TP_PULSE_LENGTH_DEF.to_le_bytes());
    payload[18] = 1;
    encode_ubx(CLASS_CFG, ID_CFG_VALSET, &payload, out)
}

/// `UBX-CFG-VALSET` — disable SBAS, Galileo, BeiDou (first frame).
///
/// 27-byte frame, safe for Apollo3 IOM I2C FIFO (≤ 32 bytes).
/// Together with `encode_signal_gps_only_2` this restricts the receiver to
/// GPS-only mode, which is required for reliable 25 Hz navigation output.
pub fn encode_signal_gps_only_1(out: &mut [u8]) -> usize {
    let mut payload = [0u8; 19];
    payload[0] = 0x00; // version
    payload[1] = 0x01; // layers: RAM
    // GPS_ENA = 1
    payload[4..8].copy_from_slice(&KEY_SIGNAL_GPS_ENA.to_le_bytes());
    payload[8] = 1;
    // SBAS_ENA = 0
    payload[9..13].copy_from_slice(&KEY_SIGNAL_SBAS_ENA.to_le_bytes());
    payload[13] = 0;
    // GAL_ENA = 0
    payload[14..18].copy_from_slice(&KEY_SIGNAL_GAL_ENA.to_le_bytes());
    payload[18] = 0;
    encode_ubx(CLASS_CFG, ID_CFG_VALSET, &payload, out)
}

/// `UBX-CFG-VALSET` — disable BeiDou, QZSS, GLONASS (second frame).
///
/// 27-byte frame, safe for Apollo3 IOM I2C FIFO (≤ 32 bytes).
pub fn encode_signal_gps_only_2(out: &mut [u8]) -> usize {
    let mut payload = [0u8; 19];
    payload[0] = 0x00;
    payload[1] = 0x01;
    // BDS_ENA = 0
    payload[4..8].copy_from_slice(&KEY_SIGNAL_BDS_ENA.to_le_bytes());
    payload[8] = 0;
    // QZSS_ENA = 0
    payload[9..13].copy_from_slice(&KEY_SIGNAL_QZSS_ENA.to_le_bytes());
    payload[13] = 0;
    // GLO_ENA = 0
    payload[14..18].copy_from_slice(&KEY_SIGNAL_GLO_ENA.to_le_bytes());
    payload[18] = 0;
    encode_ubx(CLASS_CFG, ID_CFG_VALSET, &payload, out)
}


/// Configuration for a UBX message output rate on DDC (I2C).
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
    /// Time accuracy estimate in nanoseconds.
    pub t_acc_ns: u32,
    /// Fraction of second in nanoseconds (-1e9..+1e9). Valid when `valid` bit 2 (fullyResolved) is set.
    pub nano: i32,
    /// GNSS fix type: 0=no fix, 1=dead-reckoning, 2=2D, 3=3D, 4=GNSS+DR, 5=time-only.
    pub fix_type: u8,
    /// Fix status flags. Bits 5–6: carrSoln (0=no carrier, 1=float, 2=fixed).
    pub flags: u8,
    /// Number of satellites used in the navigation solution.
    pub num_sv: u8,
    /// Longitude in units of 1e-7 degrees (divide by 1e7 for degrees).
    pub lon: i32,
    /// Latitude in units of 1e-7 degrees (divide by 1e7 for degrees).
    pub lat: i32,
    /// Height above mean sea level in millimetres.
    pub height_msl_mm: i32,
    /// Horizontal accuracy estimate in millimetres.
    pub h_acc_mm: u32,
    /// Vertical accuracy estimate in millimetres.
    pub v_acc_mm: u32,
    /// NED north velocity in mm/s.
    pub vel_n_mm_s: i32,
    /// NED east velocity in mm/s.
    pub vel_e_mm_s: i32,
    /// NED down velocity in mm/s.
    pub vel_d_mm_s: i32,
    /// Speed (3-D) accuracy estimate in mm/s.
    pub s_acc_mm_s: u32,
}

/// Scan `buf` for the first valid `UBX-NAV-PVT` packet and return the parsed
/// position/time. Returns `None` if no complete, checksum-valid NAV-PVT is found.
pub fn parse_nav_pvt(buf: &[u8]) -> Option<NavPvt> {
    let mut out = None;
    iter_nav_pvts(buf, &mut |pvt| { out = Some(pvt); });
    out
}

/// Scan `buf` and call `f` once for every valid `UBX-NAV-PVT` packet found.
/// Non-NAV-PVT UBX messages and non-UBX bytes are skipped.
pub fn iter_nav_pvts<F: FnMut(NavPvt)>(buf: &[u8], f: &mut F) {
    let mut i = 0;
    while i + 6 <= buf.len() {
        if buf[i] != SYNC1 || buf[i + 1] != SYNC2 {
            i += 1;
            continue;
        }
        let Some(pkt_cls) = buf.get(i + 2).copied() else { break };
        let Some(pkt_id)  = buf.get(i + 3).copied() else { break };
        let (Some(&lo), Some(&hi)) = (buf.get(i + 4), buf.get(i + 5)) else { break };
        let pkt_len = u16::from_le_bytes([lo, hi]) as usize;
        let total = 6 + pkt_len + 2;

        if pkt_cls != CLASS_NAV || pkt_id != ID_NAV_PVT {
            i += if i + total <= buf.len() { total } else { 1 };
            continue;
        }
        if i + total > buf.len() || pkt_len < 92 {
            // Incomplete packet at end of buffer — stop scanning.
            break;
        }

        let payload = &buf[i + 6..i + 6 + pkt_len];
        let (ck_a, ck_b) = checksum(pkt_cls, pkt_id, payload);
        if ck_a != buf[i + 6 + pkt_len] || ck_b != buf[i + 6 + pkt_len + 1] {
            i += 1;
            continue;
        }

        let pvt = NavPvt {
            year:          u16::from_le_bytes(payload[4..6].try_into().unwrap()),
            month:         payload[6],
            day:           payload[7],
            hour:          payload[8],
            min:           payload[9],
            sec:           payload[10],
            valid:         payload[11],
            t_acc_ns:      u32::from_le_bytes(payload[12..16].try_into().unwrap()),
            nano:          i32::from_le_bytes(payload[16..20].try_into().unwrap()),
            fix_type:      payload[20],
            flags:         payload[21],
            num_sv:        payload[23],
            lon:           i32::from_le_bytes(payload[24..28].try_into().unwrap()),
            lat:           i32::from_le_bytes(payload[28..32].try_into().unwrap()),
            height_msl_mm: i32::from_le_bytes(payload[36..40].try_into().unwrap()),
            h_acc_mm:      u32::from_le_bytes(payload[40..44].try_into().unwrap()),
            v_acc_mm:      u32::from_le_bytes(payload[44..48].try_into().unwrap()),
            vel_n_mm_s:    i32::from_le_bytes(payload[48..52].try_into().unwrap()),
            vel_e_mm_s:    i32::from_le_bytes(payload[52..56].try_into().unwrap()),
            vel_d_mm_s:    i32::from_le_bytes(payload[56..60].try_into().unwrap()),
            s_acc_mm_s:    u32::from_le_bytes(payload[68..72].try_into().unwrap()),
        };
        f(pvt);
        i += total;
    }
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

    #[test]
    fn test_parse_nav_pvt_new_fields() {
        // Build a synthetic 92-byte NAV-PVT payload with known values at each offset.
        let mut payload = [0u8; 92];

        // Time fields (offsets 4–11)
        payload[4..6].copy_from_slice(&2024u16.to_le_bytes()); // year
        payload[6] = 7;   // month
        payload[7] = 15;  // day
        payload[8] = 12;  // hour
        payload[9] = 30;  // min
        payload[10] = 45; // sec
        payload[11] = 0x03; // valid: validDate + validTime

        // t_acc_ns = 500 (offset 12)
        payload[12..16].copy_from_slice(&500u32.to_le_bytes());
        // nano = -123456 (offset 16)
        payload[16..20].copy_from_slice(&(-123456i32).to_le_bytes());

        // fix_type = 3 (3D, offset 20), flags = 0x40 (carrSoln=2=fixed, offset 21)
        payload[20] = 3;
        payload[21] = 0x40;
        // numSV = 12 (offset 23)
        payload[23] = 12;

        // lon = 59_123_456 (offset 24), lat = 10_987_654 (offset 28)
        payload[24..28].copy_from_slice(&59_123_456i32.to_le_bytes());
        payload[28..32].copy_from_slice(&10_987_654i32.to_le_bytes());
        // hMSL = 45_000 mm (offset 36), hAcc = 2500 mm (offset 40)
        payload[36..40].copy_from_slice(&45_000i32.to_le_bytes());
        payload[40..44].copy_from_slice(&2_500u32.to_le_bytes());
        // vAcc = 3000 mm (offset 44)
        payload[44..48].copy_from_slice(&3_000u32.to_le_bytes());
        // velN = 100, velE = -200, velD = 50 mm/s (offsets 48, 52, 56)
        payload[48..52].copy_from_slice(&100i32.to_le_bytes());
        payload[52..56].copy_from_slice(&(-200i32).to_le_bytes());
        payload[56..60].copy_from_slice(&50i32.to_le_bytes());
        // sAcc = 75 mm/s (offset 68)
        payload[68..72].copy_from_slice(&75u32.to_le_bytes());

        // Wrap in a full UBX frame
        let mut frame = [0u8; 6 + 92 + 2];
        let n = encode_ubx(CLASS_NAV, ID_NAV_PVT, &payload, &mut frame);
        assert_eq!(n, 6 + 92 + 2);

        let pvt = parse_nav_pvt(&frame[..n]).expect("should parse");

        assert_eq!(pvt.year, 2024);
        assert_eq!(pvt.month, 7);
        assert_eq!(pvt.day, 15);
        assert_eq!(pvt.hour, 12);
        assert_eq!(pvt.min, 30);
        assert_eq!(pvt.sec, 45);
        assert_eq!(pvt.valid, 0x03);
        assert_eq!(pvt.t_acc_ns, 500);
        assert_eq!(pvt.nano, -123456);
        assert_eq!(pvt.fix_type, 3);
        assert_eq!(pvt.flags, 0x40);
        assert_eq!(pvt.num_sv, 12);
        assert_eq!(pvt.lon, 59_123_456);
        assert_eq!(pvt.lat, 10_987_654);
        assert_eq!(pvt.height_msl_mm, 45_000);
        assert_eq!(pvt.h_acc_mm, 2_500);
        assert_eq!(pvt.v_acc_mm, 3_000);
        assert_eq!(pvt.vel_n_mm_s, 100);
        assert_eq!(pvt.vel_e_mm_s, -200);
        assert_eq!(pvt.vel_d_mm_s, 50);
        assert_eq!(pvt.s_acc_mm_s, 75);
    }
}
