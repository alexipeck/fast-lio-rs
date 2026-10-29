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

fn fields(line: &str) -> Vec<&str> {
    line.trim_end_matches(['\r', '\n']).split(',').collect()
}

/// Per-channel angle offsets, in degrees, from the unit's angle-correction file.
#[derive(Debug, Clone, PartialEq)]
pub struct AngleCorrection {
    pub elevation: Vec<f32>,
    pub azimuth: Vec<f32>,
    /// The SHA-256 line at the end of the file, if present. The SDK only warns on a mismatch.
    pub hash: Option<String>,
}

impl AngleCorrection {
    /// `Channel,Elevation,Azimuth` CSV: a header line (or `EEFF` plus one more line),
    /// then `id,elevation,azimuth` rows with ids counting up from 1, then an
    /// optional 64-hex-character hash line.
    pub fn parse(text: &str) -> Result<Self, CalibError> {
        let mut lines = text.lines();
        let first = lines
            .next()
            .ok_or_else(|| CalibError("empty angle-correction file".into()))?;
        let head = fields(first.trim_start_matches('\u{feff}'));
        if head[0].eq_ignore_ascii_case("eeff") {
            lines.next();
        }
        let (mut elevation, mut azimuth, mut hash) = (Vec::new(), Vec::new(), None);
        for line in lines {
            let f = fields(line);
            if f[0].len() == 64 {
                hash = Some(f[0].to_string());
                continue;
            }
            if f.len() != 3 {
                continue;
            }
            let id: usize = f[0]
                .trim()
                .parse()
                .map_err(|_| CalibError(format!("bad channel id in {line:?}")))?;
            if id != elevation.len() + 1 {
                // The SDK warns and skips rows whose id doesn't continue the count.
                continue;
            }
            let num = |s: &str| {
                s.trim()
                    .parse::<f32>()
                    .map_err(|_| CalibError(format!("bad number in {line:?}")))
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
        })
    }
}

/// Per-channel firing delay in nanoseconds (`channel,firetime/us` CSV).
#[derive(Debug, Clone, PartialEq)]
pub struct Firetimes {
    pub ns: Vec<i32>,
}

impl Firetimes {
    pub fn parse(text: &str) -> Result<Self, CalibError> {
        let mut ns = vec![0i32; CHANNELS];
        let mut seen = 0;
        for line in text.lines().skip(1) {
            let f = fields(line);
            if f.len() < 2 {
                continue;
            }
            let id: usize = f[0]
                .trim()
                .parse()
                .map_err(|_| CalibError(format!("bad channel id in {line:?}")))?;
            let us: f32 = f[1]
                .trim()
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

    #[test]
    fn angle_file_with_hash_and_bom() {
        let mut text = String::from("\u{feff}Channel,Elevation,Azimuth\r\n");
        for i in 1..=128 {
            text.push_str(&format!("{i},{}.5,{}.25\r\n", i, i + 100));
        }
        text.push_str(&format!("{},,\r\n", "a".repeat(64)));
        let c = AngleCorrection::parse(&text).unwrap();
        assert_eq!(c.elevation[0], 1.5);
        assert_eq!(c.azimuth[127], 228.25);
        assert_eq!(c.hash.as_deref(), Some("a".repeat(64).as_str()));
    }

    #[test]
    fn firetimes_truncate_like_the_sdk() {
        let text = "\u{feff}channel,firetime/us\n1,95.18\n2,23.24\n";
        let f = Firetimes::parse(text).unwrap();
        assert_eq!(f.ns[0], (95.18f32 * 1000.0f32) as i32);
        assert_eq!(f.ns[1], (23.24f32 * 1000.0f32) as i32);
        assert_eq!(f.ns[2], 0);
    }
}
