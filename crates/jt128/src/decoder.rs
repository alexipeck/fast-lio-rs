//! Packets in, LiDAR frames and IMU samples out.
//!
//! Point geometry and timing follow Hesai SDK 2.0's `Udp1_4Parser::ComputeXYZI`
//! for the JT128, including its integer angle steps of 1/25600° and its
//! rounding. Frames are cut where the first-block azimuth wraps past the
//! start angle (`GeneralParser::IsNeedFrameSplit`). See
//! `docs/hesai_jt128_decoder.md` for the departures: spin-up packets and the
//! points of the next half second are dropped, zero-range points are dropped,
//! IMU samples are split back out of merged runs, and the SDK's optical-centre
//! correction is on by default.

use crate::calib::{AngleCorrection, CHANNELS, Firetimes};
use crate::packet::{Packet, PacketError};
use crate::{Frame, ImuSample, Point};

/// Angle units of the SDK: 1/25600 of a degree.
const FINE: i32 = 25_600;
const CIRCLE: i32 = 36_000 * 256;
const RETURN_MODE_MULTI: u8 = 0x39;
const BLOCK_NS_OFFSET1: i64 = -1_888_000;
const BLOCK_NS_OFFSET2: i64 = -111_000;
/// The optical centre with the head at azimuth 0 (m): 13.67 mm towards the
/// connector, 7.6 mm aside and 12.74 mm up from the axis origin (JT128 manual,
/// Figure 5; the SDK's `Udp1_4Parser` constructor).
const OPTICAL_CENTRE: [f32; 3] = [-0.0076, 0.01367, 0.01274];

/// The SDK's `doubleToInt`: adds 1/16, then truncates toward zero.
fn double_to_int(v: f64) -> i32 {
    (v + 0.0625) as i32
}

fn circle_revise(a: i32) -> i32 {
    a.rem_euclid(CIRCLE)
}

/// sin and cos of an SDK angle, as its `float` lookup tables hold them.
fn sin_cos(a: i32) -> (f32, f32) {
    let x = f64::from(a * 2) * std::f64::consts::PI / f64::from(CIRCLE);
    (x.sin() as f32, x.cos() as f32)
}

#[derive(Clone, Debug)]
pub struct DecoderConfig {
    /// Azimuth (degrees) at which a frame starts. The SDK default is 0.
    pub frame_start_azimuth: f32,
    /// Nominal IMU sample period, used to split runs of identical samples.
    pub imu_period: f64,
    /// Apply the firing-delay azimuth correction (on in the SDK's driver).
    pub firetime_azimuth_correction: bool,
    /// Seconds of packet time after a spin-up whose points are dropped. For
    /// about 0.23 s after its motor spins up from standby the JT128 reports
    /// wrong ranges and reflectivity, unmarked in the packets. 0 keeps them.
    pub wake_hold: f64,
    /// Measure returns beyond 0.09 m from the axis origin instead of the
    /// optical centre, which turns with the head 15.6 mm off the axis: the
    /// SDK's `distance_correction_flag`, off by default there. Moves each such
    /// point by 2.0 cm.
    pub optical_centre: bool,
}

impl Default for DecoderConfig {
    fn default() -> Self {
        Self {
            frame_start_azimuth: 0.0,
            imu_period: 0.005,
            firetime_azimuth_correction: true,
            wake_hold: 0.5,
            optical_centre: true,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum Event {
    Frame(Frame),
    Imu(ImuSample),
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct DecoderStats {
    pub packets: u64,
    pub spinup_packets: u64,
    /// Packets within `wake_hold` after a spin-up: their points are dropped,
    /// their IMU samples kept.
    pub wake_packets: u64,
    pub sequence_gaps: u64,
    pub frames: u64,
    pub imu_samples: u64,
    /// IMU samples recovered from runs of identical values longer than one period.
    pub imu_samples_split: u64,
}

struct ChannelCalib {
    azimuth: i32,
    /// sin and cos of the elevation, which is fixed per channel.
    sin_el: f32,
    cos_el: f32,
    /// sin and cos of the azimuth offset alone, for the optical centre.
    sin_az: f32,
    cos_az: f32,
    firetime_ns: i32,
}

impl ChannelCalib {
    /// `GeneralParser::GetDistanceCorrection` for the JT128 (`OpticalCenter`):
    /// a return `d` along the beam from the optical centre, with the head at
    /// azimuth 0, as an azimuth offset, the sin and cos of an elevation, and a
    /// distance from the axis origin. The SDK truncates the two angles.
    fn to_axis_origin(&self, d: f32) -> (i32, (f32, f32), f32) {
        let [cx, cy, cz] = OPTICAL_CENTRE;
        let x = d * (self.cos_el * self.sin_az) + cx;
        let y = d * (self.cos_el * self.cos_az) + cy;
        let z = d * self.sin_el + cz;
        let r = (x * x + y * y + z * z).sqrt();
        // The SDK calls atan2f and asinf, whose last bit is the C library's:
        // glibc 2.35 and macOS put about 1% of these angles a step apart.
        // Taken in double and rounded, they are correctly rounded everywhere
        // and match the SDK on glibc 2.43 bit for bit.
        let atan2 = f64::from(x).atan2(f64::from(y)) as f32;
        let asin = f64::from(z / r).asin() as f32;
        // Radians to SDK units: two float products, then a division by the double π.
        let fine = |a: f32| (f64::from(a * 18_000.0 * 256.0) / std::f64::consts::PI) as i32;
        let elevation = circle_revise(fine(asin));
        (circle_revise(fine(atan2)), sin_cos(elevation), r)
    }
}

pub struct Decoder {
    cfg: DecoderConfig,
    channels: Vec<ChannelCalib>,
    have_firetimes: bool,
    wake_hold_us: i64,
    /// Packet time (µs) of the last spin-up packet, while its hold lasts.
    wake_us: Option<i64>,
    start_azimuth: i32,
    last_azimuth: Option<i32>,
    last_last_azimuth: Option<i32>,
    rotation_flag: i32,
    frame: Vec<(i64, Point)>,
    last_sequence: Option<u32>,
    imu_last: Option<(f64, [i16; 6], ImuSample)>,
    pub stats: DecoderStats,
}

impl Decoder {
    pub fn new(
        angles: &AngleCorrection,
        firetimes: Option<&Firetimes>,
        cfg: DecoderConfig,
    ) -> Self {
        let channels = (0..CHANNELS)
            .map(|ch| {
                let elevation = double_to_int(f64::from(angles.elevation[ch] * FINE as f32));
                let (sin_el, cos_el) = sin_cos(circle_revise(elevation));
                let azimuth = double_to_int(f64::from(angles.azimuth[ch] * FINE as f32));
                let (sin_az, cos_az) = sin_cos(circle_revise(azimuth));
                ChannelCalib {
                    azimuth,
                    sin_el,
                    cos_el,
                    sin_az,
                    cos_az,
                    firetime_ns: firetimes.map_or(0, |f| f.ns[ch]),
                }
            })
            .collect();
        let start = cfg.frame_start_azimuth;
        let start_azimuth = if (0.0..360.0).contains(&start) {
            (start * 100.0) as i32
        } else {
            0
        };
        Self {
            wake_hold_us: (cfg.wake_hold.max(0.0) * 1e6).round() as i64,
            cfg,
            channels,
            have_firetimes: firetimes.is_some(),
            wake_us: None,
            start_azimuth,
            last_azimuth: None,
            last_last_azimuth: None,
            rotation_flag: 0,
            frame: Vec::new(),
            last_sequence: None,
            imu_last: None,
            stats: DecoderStats::default(),
        }
    }

    /// Decodes one UDP payload. Completed frames and IMU samples are appended to `out`.
    pub fn push(&mut self, payload: &[u8], out: &mut Vec<Event>) -> Result<(), PacketError> {
        let p = Packet::parse(payload)?;
        self.stats.packets += 1;
        if let Some(seq) = p.sequence() {
            if self.last_sequence.is_some_and(|s| seq != s.wrapping_add(1)) {
                self.stats.sequence_gaps += 1;
            }
            self.last_sequence = Some(seq);
        }
        let t_pkt_us = p.time_us();
        if let Some(raw) = p.imu() {
            self.push_imu(t_pkt_us as f64 * 1e-6, raw, out);
        }

        // While the motor spins up from standby, the blocks don't advance (both
        // read 0 on the captured unit, with one transitional packet at each
        // end). Those points have no valid geometry. A real block step is
        // 0.4° at 10 Hz, so anything beyond 5° either way is rejected.
        let step = (i32::from(p.azimuth(1.min(p.block_num - 1))) - i32::from(p.azimuth(0)))
            .rem_euclid(36_000);
        if p.block_num >= 2 && !(1..=500).contains(&step) && !(35_500..36_000).contains(&step) {
            self.stats.spinup_packets += 1;
            // A wake. The frame in progress, if any, ended at the standby, and
            // frame splitting starts over as at the start of a stream.
            self.finish_frame(out);
            self.last_azimuth = None;
            self.last_last_azimuth = None;
            self.wake_us = Some(t_pkt_us);
            return Ok(());
        }
        // For 0.227 s after the last spin-up packet (the same on three
        // captures), the up-looking channels read 3.7 m to a ceiling 1.7 m
        // away and reflectivity is about 4 times too high. Only the point
        // bytes differ. A clock that steps back ends the hold.
        if let Some(w) = self.wake_us {
            if (0..self.wake_hold_us).contains(&(t_pkt_us - w)) {
                self.stats.wake_packets += 1;
                return Ok(());
            }
            self.wake_us = None;
        }

        let az = i32::from(p.azimuth(0));
        if self.need_split(az) {
            self.finish_frame(out);
        }
        if Some(az) != self.last_azimuth {
            self.last_last_azimuth = self.last_azimuth;
            self.last_azimuth = Some(az);
        }
        self.add_points(&p, t_pkt_us);
        Ok(())
    }

    /// Emits the frame in progress, if any, e.g. at the end of a capture.
    pub fn flush(&mut self, out: &mut Vec<Event>) {
        self.finish_frame(out);
    }

    fn add_points(&mut self, p: &Packet<'_>, t_pkt_us: i64) {
        let k = if p.return_mode() < RETURN_MODE_MULTI {
            1
        } else {
            2
        };
        let sign = if self.rotation_flag > 0 { 1.0 } else { -1.0 };
        let speed = f64::from(p.motor_speed_raw()) * 0.1;
        for b in 0..p.block_num {
            let block_offset =
                BLOCK_NS_OFFSET1 + BLOCK_NS_OFFSET2 * ((p.block_num - b - 1) / k) as i64;
            let raw_az = i32::from(p.azimuth(b)) * 256;
            for ch in 0..p.laser_num.min(CHANNELS) {
                let (dist, refl) = p.channel(b, ch);
                if dist == 0 {
                    continue;
                }
                let c = &self.channels[ch];
                let distance = f32::from(dist) * p.dist_unit;
                let mut azimuth = raw_az;
                if self.have_firetimes && self.cfg.firetime_azimuth_correction {
                    let corr_deg = f64::from(c.firetime_ns) * speed * 6e-9;
                    azimuth += (sign * f64::from(double_to_int(corr_deg * f64::from(FINE)))) as i32;
                }
                // The SDK applies the optical centre to the channel's own angles,
                // so it turns with the head.
                let (offset, (sin_el, cos_el), distance) =
                    if self.cfg.optical_centre && f64::from(distance) > 0.09 {
                        c.to_axis_origin(distance)
                    } else {
                        (c.azimuth, (c.sin_el, c.cos_el), distance)
                    };
                azimuth = circle_revise(azimuth + offset);
                let (sin_az, cos_az) = sin_cos(azimuth);
                let xy = distance * cos_el;
                let t_ns = t_pkt_us * 1000
                    + if self.have_firetimes {
                        block_offset + i64::from(c.firetime_ns)
                    } else {
                        0
                    };
                let point = Point {
                    x: xy * sin_az,
                    y: xy * cos_az,
                    z: distance * sin_el,
                    intensity: f32::from(refl),
                    dt_ms: 0.0,
                    range: distance,
                };
                self.frame.push((t_ns, point));
            }
        }
    }

    fn finish_frame(&mut self, out: &mut Vec<Event>) {
        if self.frame.is_empty() {
            return;
        }
        let t0 = self.frame.iter().map(|(t, _)| *t).min().unwrap();
        let points = self
            .frame
            .drain(..)
            .map(|(t, mut p)| {
                p.dt_ms = ((t - t0) as f64 * 1e-6) as f32;
                p
            })
            .collect();
        self.stats.frames += 1;
        out.push(Event::Frame(Frame {
            t0: t0 as f64 * 1e-9,
            points,
        }));
    }

    /// `GeneralParser::IsNeedFrameSplit`, with azimuths in hundredths of a degree.
    fn need_split(&mut self, az: i32) -> bool {
        let (Some(last), Some(last_last)) = (self.last_azimuth, self.last_last_azimuth) else {
            return false;
        };
        let division = (last - last_last).abs().min((last - az).abs());
        if division == 0 {
            return false;
        }
        let rotation = if last_last - last == division || last - az == division {
            -1
        } else {
            1
        };
        if self.rotation_flag.abs() != 10_240 {
            if self.rotation_flag.abs() > 128 * 8 {
                self.rotation_flag /= 128;
            }
            self.rotation_flag += rotation;
        }
        let start = self.start_azimuth;
        if self.rotation_flag > 0 {
            if last - az > division {
                return start > last || start <= az;
            }
            last < az && last < start && az >= start
        } else {
            if az - last > division {
                return start < last || start >= az;
            }
            last > az && last > start && az <= start
        }
    }

    /// De-duplicates IMU samples: each one rides in ~22 consecutive packets and
    /// the counter field is always 0, so a sample is a change in the six raw
    /// values, stamped with the packet that first carries it. A value that
    /// persists for about k periods was k identical samples; the extras are
    /// re-emitted one period apart.
    fn push_imu(&mut self, t: f64, raw: crate::packet::RawImu, out: &mut Vec<Event>) {
        let values = [
            raw.accel[0],
            raw.accel[1],
            raw.accel[2],
            raw.gyro[0],
            raw.gyro[1],
            raw.gyro[2],
        ];
        if let Some((t_prev, prev_values, prev_sample)) = &self.imu_last {
            if *prev_values == values {
                return;
            }
            let periods = ((t - t_prev) / self.cfg.imu_period).round() as i64;
            for j in 1..periods {
                let mut s = *prev_sample;
                s.t = t_prev + j as f64 * self.cfg.imu_period;
                out.push(Event::Imu(s));
                self.stats.imu_samples += 1;
                self.stats.imu_samples_split += 1;
            }
        }
        let sample = ImuSample {
            t,
            acc: raw.accel_g(),
            gyro: raw.gyro_deg_s().map(f64::to_radians),
        };
        out.push(Event::Imu(sample));
        self.stats.imu_samples += 1;
        self.imu_last = Some((t, values, sample));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A packet of two blocks 0.4° apart with no optional sections and the motor
    /// stopped: one return of `distance` counts (4 mm) on `ch` in the first.
    fn packet(azimuth: u16, ch: usize, distance: u16) -> Vec<u8> {
        let block = 2 + CHANNELS * 3;
        let tail = 12 + 2 * block + 4;
        let mut b = vec![0; tail + 26 + 4];
        b[..12].copy_from_slice(&[0xEE, 0xFF, 1, 4, 137, 0, CHANNELS as u8, 2, 0, 4, 1, 0]);
        b[12..14].copy_from_slice(&azimuth.to_le_bytes());
        b[14 + ch * 3..16 + ch * 3].copy_from_slice(&distance.to_le_bytes());
        b[12 + block..14 + block].copy_from_slice(&(azimuth + 40).to_le_bytes());
        b[tail + 12] = 0x37;
        b
    }

    /// The points of one packet per head azimuth (0.01°), as one frame.
    fn decode(
        angles: &AngleCorrection,
        optical_centre: bool,
        ch: usize,
        distance: u16,
        heads: &[u16],
    ) -> Vec<Point> {
        let cfg = DecoderConfig {
            optical_centre,
            ..DecoderConfig::default()
        };
        let mut dec = Decoder::new(angles, None, cfg);
        let mut out = Vec::new();
        for &h in heads {
            dec.push(&packet(h, ch, distance), &mut out).unwrap();
        }
        dec.flush(&mut out);
        let [Event::Frame(f)] = &out[..] else {
            panic!("{out:?}");
        };
        f.points.clone()
    }

    fn straight() -> AngleCorrection {
        AngleCorrection {
            elevation: vec![0.0; CHANNELS],
            azimuth: vec![0.0; CHANNELS],
            hash: None,
            hash_matches: None,
        }
    }

    fn assert_at(p: &Point, want: [f32; 3]) {
        let xyz = [p.x, p.y, p.z];
        assert!(
            (0..3).all(|k| (xyz[k] - want[k]).abs() < 1e-5),
            "{xyz:?} {want:?}"
        );
        assert!((p.range - (p.x * p.x + p.y * p.y + p.z * p.z).sqrt()).abs() < 1e-6);
    }

    #[test]
    fn optical_centre_turns_with_the_head() {
        let [o0, o1, o2] = OPTICAL_CENTRE;
        let on = decode(&straight(), true, 0, 500, &[0, 9000]);
        // Head at 0°: 2 m along +y from the optical centre.
        assert_at(&on[0], [o0, 2.0 + o1, o2]);
        // Head at 90°: the offset turned a quarter clockwise, (x, y) -> (y, -x).
        assert_at(&on[1], [2.0 + o1, -o0, o2]);
        // Off, and on for a return within 0.09 m: from the optical centre.
        let off = decode(&straight(), false, 0, 500, &[0, 9000]);
        assert_at(&off[0], [0.0, 2.0, 0.0]);
        assert_at(&off[1], [2.0, 0.0, 0.0]);
        let near = decode(&straight(), true, 0, 22, &[0]);
        assert_at(&near[0], [0.0, 0.088, 0.0]);
    }

    #[test]
    fn optical_centre_is_added_along_the_beam() {
        // A channel aimed 30° anticlockwise of the head and 10° up, 3 m, head at 120°:
        // the centre is added with the head at 0°, then the sum turned with the head.
        let mut angles = straight();
        angles.elevation[5] = 10.0;
        angles.azimuth[5] = -30.0;
        let p = decode(&angles, true, 5, 750, &[12_000])[0];
        let (se, ce) = 10f32.to_radians().sin_cos();
        let (sa, ca) = (-30f32).to_radians().sin_cos();
        let v = [0, 1, 2].map(|k| [3.0 * ce * sa, 3.0 * ce * ca, 3.0 * se][k] + OPTICAL_CENTRE[k]);
        let (sh, ch) = 120f32.to_radians().sin_cos();
        assert_at(&p, [v[0] * ch + v[1] * sh, v[1] * ch - v[0] * sh, v[2]]);
    }
}
