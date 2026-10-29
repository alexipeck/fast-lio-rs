//! Hesai UDP protocol v1.4 point packets, as the JT128 sends them. Layout in
//! `docs/hesai_jt128_decoder.md` §3.

use std::fmt;

#[derive(Debug, Clone, PartialEq)]
pub enum PacketError {
    NotPointCloud,
    NotJt128 { status_info_version: u8 },
    Truncated { expected: usize, actual: usize },
}

impl fmt::Display for PacketError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotPointCloud => f.write_str("not a v1.4 point-cloud packet"),
            Self::NotJt128 {
                status_info_version,
            } => {
                write!(
                    f,
                    "v1.4 packet from a different model (status-info version {status_info_version})"
                )
            }
            Self::Truncated { expected, actual } => {
                write!(f, "packet is {actual} bytes, header implies {expected}")
            }
        }
    }
}

impl std::error::Error for PacketError {}

const PRE_HEADER: usize = 6;
const HEADER: usize = 6;
const BODY_START: usize = PRE_HEADER + HEADER;
const TAIL: usize = 26;
const FUNC_SAFETY: usize = 17;
const IMU: usize = 22;

const FLAG_SEQ: u8 = 0x01;
const FLAG_IMU: u8 = 0x02;
const FLAG_FUNC_SAFETY: u8 = 0x04;
const FLAG_SIGNATURE: u8 = 0x08;

/// Raw IMU fields from the packet trailer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RawImu {
    /// Hundredths of a degree Celsius.
    pub temperature: i16,
    /// µg per LSB.
    pub accel_scale: u16,
    /// 10 µ°/s per LSB (that is, `raw * scale / 1e5` gives °/s).
    pub gyro_scale: u16,
    pub counter: u32,
    pub accel: [i16; 3],
    pub gyro: [i16; 3],
}

impl RawImu {
    pub fn accel_g(&self) -> [f64; 3] {
        self.accel
            .map(|v| f64::from(v) * f64::from(self.accel_scale) / 1e6)
    }

    pub fn gyro_deg_s(&self) -> [f64; 3] {
        self.gyro
            .map(|v| f64::from(v) * f64::from(self.gyro_scale) / 1e5)
    }
}

/// A validated packet; accessors read straight from the datagram.
#[derive(Debug, Clone, Copy)]
pub struct Packet<'a> {
    data: &'a [u8],
    pub laser_num: usize,
    pub block_num: usize,
    /// Metres per distance count.
    pub dist_unit: f32,
    pub flags: u8,
    unit_size: usize,
    tail: usize,
}

fn le16(d: &[u8], o: usize) -> u16 {
    u16::from_le_bytes([d[o], d[o + 1]])
}

fn le32(d: &[u8], o: usize) -> u32 {
    u32::from_le_bytes([d[o], d[o + 1], d[o + 2], d[o + 3]])
}

impl<'a> Packet<'a> {
    pub fn parse(data: &'a [u8]) -> Result<Self, PacketError> {
        if data.len() < BODY_START
            || data[0] != 0xEE
            || data[1] != 0xFF
            || data[2] != 1
            || data[3] != 4
        {
            return Err(PacketError::NotPointCloud);
        }
        if data[4] < 137 {
            return Err(PacketError::NotJt128 {
                status_info_version: data[4],
            });
        }
        let (laser_num, block_num, flags) = (usize::from(data[6]), usize::from(data[7]), data[11]);
        let unit_size = 3 + (flags & 0x70).count_ones() as usize;
        let tail = BODY_START
            + block_num * (2 + laser_num * unit_size)
            + 4
            + if flags & FLAG_FUNC_SAFETY != 0 {
                FUNC_SAFETY
            } else {
                0
            };
        let expected = tail
            + TAIL
            + if flags & FLAG_SEQ != 0 { 4 } else { 0 }
            + if flags & FLAG_IMU != 0 { IMU } else { 0 }
            + 4
            + if flags & FLAG_SIGNATURE != 0 { 32 } else { 0 };
        if data.len() != expected {
            return Err(PacketError::Truncated {
                expected,
                actual: data.len(),
            });
        }
        Ok(Self {
            data,
            laser_num,
            block_num,
            dist_unit: f32::from(data[9]) / 1000.0,
            flags,
            unit_size,
            tail,
        })
    }

    fn block_start(&self, block: usize) -> usize {
        BODY_START + block * (2 + self.laser_num * self.unit_size)
    }

    /// Block azimuth in hundredths of a degree.
    pub fn azimuth(&self, block: usize) -> u16 {
        le16(self.data, self.block_start(block))
    }

    /// Raw distance count and reflectivity of one channel.
    pub fn channel(&self, block: usize, channel: usize) -> (u16, u8) {
        let o = self.block_start(block) + 2 + channel * self.unit_size;
        (le16(self.data, o), self.data[o + 2])
    }

    pub fn operational_mode(&self) -> u8 {
        self.data[self.tail + 11]
    }

    pub fn return_mode(&self) -> u8 {
        self.data[self.tail + 12]
    }

    /// Motor speed field; 0.1 RPM per unit on the JT128.
    pub fn motor_speed_raw(&self) -> u16 {
        le16(self.data, self.tail + 13)
    }

    /// Packet time on the sensor clock, in microseconds since the Unix epoch.
    pub fn time_us(&self) -> i64 {
        let utc = &self.data[self.tail + 15..self.tail + 21];
        let us = i64::from(le32(self.data, self.tail + 21));
        let secs = if utc[0] != 0 {
            if utc[0] < 70 {
                return 0;
            }
            civil_to_unix(
                1900 + i64::from(utc[0]),
                utc[1],
                utc[2],
                utc[3],
                utc[4],
                utc[5],
            )
        } else {
            i64::from(u32::from_be_bytes([utc[2], utc[3], utc[4], utc[5]]))
        };
        secs * 1_000_000 + us
    }

    pub fn sequence(&self) -> Option<u32> {
        (self.flags & FLAG_SEQ != 0).then(|| le32(self.data, self.tail + TAIL))
    }

    pub fn imu(&self) -> Option<RawImu> {
        if self.flags & FLAG_IMU == 0 {
            return None;
        }
        let o = self.tail + TAIL + if self.flags & FLAG_SEQ != 0 { 4 } else { 0 };
        let d = self.data;
        let i16_at = |k: usize| le16(d, o + k) as i16;
        Some(RawImu {
            temperature: i16_at(0),
            accel_scale: le16(d, o + 2),
            gyro_scale: le16(d, o + 4),
            counter: le32(d, o + 6),
            accel: [i16_at(10), i16_at(12), i16_at(14)],
            gyro: [i16_at(16), i16_at(18), i16_at(20)],
        })
    }
}

/// Seconds since the Unix epoch for a UTC calendar time (proleptic Gregorian).
pub fn civil_to_unix(year: i64, month: u8, day: u8, hour: u8, min: u8, sec: u8) -> i64 {
    // Howard Hinnant's days_from_civil.
    let m = i64::from(month);
    let y = if m <= 2 { year - 1 } else { year };
    let era = y.div_euclid(400);
    let yoe = y - era * 400;
    let doy = (153 * (if m > 2 { m - 3 } else { m + 9 }) + 2) / 5 + i64::from(day) - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    let days = era * 146_097 + doe - 719_468;
    days * 86_400 + i64::from(hour) * 3600 + i64::from(min) * 60 + i64::from(sec)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn civil_dates() {
        assert_eq!(civil_to_unix(1970, 1, 1, 0, 0, 0), 0);
        assert_eq!(civil_to_unix(2000, 3, 1, 0, 0, 0), 951_868_800);
        assert_eq!(civil_to_unix(2026, 10, 28, 23, 36, 34), 1_793_230_594);
    }
}
