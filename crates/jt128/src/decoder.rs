//! Packets in, LiDAR frames and IMU samples out.
//!
//! Point geometry and timing follow Hesai SDK 2.0's `Udp1_4Parser::ComputeXYZI`
//! for the JT128, including its integer angle steps of 1/25600° and its
//! rounding. Frames are cut where the first-block azimuth wraps past the
//! start angle (`GeneralParser::IsNeedFrameSplit`). See
//! `docs/hesai_jt128_decoder.md` for the departures: spin-up packets are
//! dropped, zero-range points are dropped, and IMU samples are split back out
//! of merged runs.

use crate::calib::{AngleCorrection, CHANNELS, Firetimes};
use crate::packet::{Packet, PacketError};
use crate::{Frame, ImuSample, Point};

/// Angle units of the SDK: 1/25600 of a degree.
const FINE: i32 = 25_600;
const CIRCLE: i32 = 36_000 * 256;
const RETURN_MODE_MULTI: u8 = 0x39;
const BLOCK_NS_OFFSET1: i64 = -1_888_000;
const BLOCK_NS_OFFSET2: i64 = -111_000;

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
}

impl Default for DecoderConfig {
    fn default() -> Self {
        Self {
            frame_start_azimuth: 0.0,
            imu_period: 0.005,
            firetime_azimuth_correction: true,
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
    pub sequence_gaps: u64,
    pub frames: u64,
    pub imu_samples: u64,
    /// IMU samples recovered from runs of identical values longer than one period.
    pub imu_samples_split: u64,
}

struct ChannelCalib {
    azimuth: i32,
    elevation: i32,
    firetime_ns: i32,
}

pub struct Decoder {
    cfg: DecoderConfig,
    channels: Vec<ChannelCalib>,
    have_firetimes: bool,
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
            .map(|ch| ChannelCalib {
                azimuth: double_to_int(f64::from(angles.azimuth[ch] * FINE as f32)),
                elevation: double_to_int(f64::from(angles.elevation[ch] * FINE as f32)),
                firetime_ns: firetimes.map_or(0, |f| f.ns[ch]),
            })
            .collect();
        let start = cfg.frame_start_azimuth;
        let start_azimuth = if (0.0..360.0).contains(&start) {
            (start * 100.0) as i32
        } else {
            0
        };
        Self {
            cfg,
            channels,
            have_firetimes: firetimes.is_some(),
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
            return Ok(());
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
                azimuth = circle_revise(azimuth + c.azimuth);
                let elevation = circle_revise(c.elevation);
                let (sin_el, cos_el) = sin_cos(elevation);
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
