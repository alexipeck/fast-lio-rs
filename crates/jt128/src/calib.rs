//! The two JT128 calibration files, parsed as Hesai's SDK 2.0 parses them
//! (`udp1_4_parser.h` `LoadCorrectionString` and `LoadFiretimesFile`).

use std::fmt;

pub const CHANNELS: usize = 128;

#[derive(Debug, Clone, PartialEq)]
pub struct CalibError(String);

impl fmt::Display for CalibError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for CalibError {}

/// A line's comma-separated fields, as the SDK's `split_string` gives them.
fn fields(line: &[u8]) -> Vec<&[u8]> {
    let line = line.strip_suffix(b"\r").unwrap_or(line);
    let mut f: Vec<&[u8]> = line.split(|&b| b == b',').collect();
    // std::getline yields no empty field after a trailing comma.
    if f.len() > 1 && f.last().is_some_and(|l| l.is_empty()) {
        f.pop();
    }
    f
}

/// Per-channel angle offsets, in degrees, from the unit's angle-correction file.
#[derive(Debug, Clone, PartialEq)]
pub struct AngleCorrection {
    pub elevation: Vec<f32>,
    pub azimuth: Vec<f32>,
    /// The SHA-256 line at the end of the file, if present.
    pub hash: Option<String>,
    /// Whether that hash matches the lines above it. The SDK only warns on a mismatch.
    pub hash_matches: Option<bool>,
}

impl AngleCorrection {
    /// `Channel,Elevation,Azimuth` CSV: a header line (or `EEFF` plus one more line),
    /// then `id,elevation,azimuth` rows with ids counting up from 1, then an
    /// optional 64-hex-character SHA-256 of the lines above it. Takes bytes,
    /// because the file as the sensor sends it (PTC `0x05`) is a 4 KiB flash
    /// page: the CSV, then `0xFF` padding and a binary trailer, which are skipped.
    pub fn parse(bytes: &[u8]) -> Result<Self, CalibError> {
        let mut lines = bytes.split(|&b| b == b'\n');
        let first = lines
            .next()
            .filter(|l| !l.is_empty())
            .ok_or_else(|| CalibError("empty angle-correction file".into()))?;
        let first = first.strip_prefix("\u{feff}".as_bytes()).unwrap_or(first);
        let header_lines = if fields(first)[0].eq_ignore_ascii_case(b"eeff") {
            lines.next();
            2
        } else {
            1
        };
        let (mut elevation, mut azimuth) = (Vec::new(), Vec::new());
        let (mut hash, mut hash_matches) = (None, None);
        for line in lines {
            let f = fields(line);
            if f[0].len() == 64 {
                let text = String::from_utf8_lossy(f[0]).into_owned();
                // The SDK hashes the file up to the end of the last row read.
                let end = bytes
                    .iter()
                    .enumerate()
                    .filter(|&(_, &b)| b == b'\n')
                    .nth(elevation.len() + header_lines - 1)
                    .map_or(bytes.len(), |(i, _)| i + 1);
                hash_matches = Some(sha256_hex(&bytes[..end]) == text);
                hash = Some(text);
                continue;
            }
            if f.len() != 3 {
                continue;
            }
            let text = |s: &[u8]| String::from_utf8_lossy(s).trim().to_string();
            let id: usize = text(f[0])
                .parse()
                .map_err(|_| CalibError(format!("bad channel id in {:?}", text(line))))?;
            if id != elevation.len() + 1 {
                // The SDK warns and skips rows whose id doesn't continue the count.
                continue;
            }
            let num = |s: &[u8]| {
                text(s)
                    .parse::<f32>()
                    .map_err(|_| CalibError(format!("bad number in {:?}", text(line))))
            };
            elevation.push(num(f[1])?);
            azimuth.push(num(f[2])?);
        }
        if elevation.len() != CHANNELS {
            return Err(CalibError(format!(
                "expected {CHANNELS} channels, found {}",
                elevation.len()
            )));
        }
        Ok(Self {
            elevation,
            azimuth,
            hash,
            hash_matches,
        })
    }
}

fn sha256_hex(bytes: &[u8]) -> String {
    use sha2::{Digest, Sha256};
    Sha256::digest(bytes)
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect()
}

/// Per-channel firing delay in nanoseconds (`channel,firetime/us` CSV).
#[derive(Debug, Clone, PartialEq)]
pub struct Firetimes {
    pub ns: Vec<i32>,
}

impl Firetimes {
    pub fn parse(bytes: &[u8]) -> Result<Self, CalibError> {
        let mut ns = vec![0i32; CHANNELS];
        let mut seen = 0;
        for line in bytes.split(|&b| b == b'\n').skip(1) {
            let f = fields(line);
            if f.len() < 2 {
                continue;
            }
            let text = |s: &[u8]| String::from_utf8_lossy(s).trim().to_string();
            let line = text(line);
            let id: usize = text(f[0])
                .parse()
                .map_err(|_| CalibError(format!("bad channel id in {line:?}")))?;
            let us: f32 = text(f[1])
                .parse()
                .map_err(|_| CalibError(format!("bad firetime in {line:?}")))?;
            if id == 0 || id > CHANNELS {
                return Err(CalibError(format!("channel id {id} out of range")));
            }
            // The SDK computes int(stof(x) * 1000) in float, truncating toward zero.
            ns[id - 1] = (us * 1000.0f32) as i32;
            seen += 1;
        }
        if seen == 0 {
            return Err(CalibError("no firetimes found".into()));
        }
        Ok(Self { ns })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn angle_file(eol: &str) -> String {
        let mut text = format!("\u{feff}Channel,Elevation,Azimuth{eol}");
        for i in 1..=128 {
            text.push_str(&format!("{i},{i}.5,{}.25{eol}", i - 100));
        }
        text
    }

    #[test]
    fn angle_file_with_hash_and_bom() {
        let mut text = angle_file("\r\n");
        text.push_str(&format!("{},,\r\n", "a".repeat(64)));
        let c = AngleCorrection::parse(text.as_bytes()).unwrap();
        assert_eq!(c.elevation[0], 1.5);
        assert_eq!(c.azimuth[127], 28.25);
        assert_eq!(c.azimuth[0], -99.25);
        assert_eq!(c.hash.as_deref(), Some("a".repeat(64).as_str()));
        assert_eq!(c.hash_matches, Some(false));
    }

    #[test]
    fn angle_file_as_the_sensor_sends_it() {
        // A 4 KiB flash page: the CSV, its SHA-256, 0xFF padding and a binary trailer.
        let csv = angle_file("\n");
        let mut page = csv.clone().into_bytes();
        page.extend(format!("{}\n", sha256_hex(csv.as_bytes())).as_bytes());
        page.resize(4092, 0xFF);
        page.extend([0xFB, 0x1D, 0xBB, 0x42]);
        let c = AngleCorrection::parse(&page).unwrap();
        assert_eq!(c.hash_matches, Some(true));
        assert_eq!(c.elevation[127], 128.5);

        let mut changed = page.clone();
        let at = csv.find("7.25").unwrap();
        changed[at] = b'8';
        assert_eq!(
            AngleCorrection::parse(&changed).unwrap().hash_matches,
            Some(false)
        );

        let without = AngleCorrection::parse(csv.as_bytes()).unwrap();
        assert_eq!((without.hash, without.hash_matches), (None, None));
    }

    #[test]
    fn firetimes_truncate_like_the_sdk() {
        let text = "\u{feff}channel,firetime/us\n1,95.18\n2,23.24\n";
        let f = Firetimes::parse(text.as_bytes()).unwrap();
        assert_eq!(f.ns[0], (95.18f32 * 1000.0f32) as i32);
        assert_eq!(f.ns[1], (23.24f32 * 1000.0f32) as i32);
        assert_eq!(f.ns[2], 0);
    }
}
