//! The feeder stage: JT128 UDP payloads in, per-point rays and the corrected
//! path out.
//!
//! [`Pipeline`] chains the `jt128` decoder, FAST-LIO (`lio-core`) and the ray
//! computation. Embed it in a processing pipeline by pushing each UDP payload
//! as it arrives; every completed frame comes back with one ray per raw point.
//!
//! The stages can also run on separate threads: a [`Decoder`] turns payloads
//! into [`Event`]s, an [`Estimator`] runs FAST-LIO on those events in order,
//! and [`Processed::rays`] computes each finished frame's rays. The results
//! are the same either way.

use std::collections::VecDeque;
use std::io::{self, Write};
use std::time::{Duration, Instant};

use jt128::{AngleCorrection, Decoder, DecoderConfig, Event, Firetimes};
use lio_core::imu_init::ImuMeasurement;
use lio_core::lio::{FrameKind, FrameOutput, Lio, LioConfig};
use lio_core::mat::Vec3;
use lio_core::point::Point;
use lio_core::rays::{RawPoint, Ray, frame_rays};

/// FAST-LIO's Mid-360 settings with the JT128's values where they differ,
/// measured on the first capture (docs/plan.md, 2026-10-29): `blind` 1.0 m and
/// 0.25 m voxels. The identity extrinsic stays: with the unit's own angle
/// file, the IMU's axes are the LiDAR's. These are the `lio` binary's defaults.
pub fn jt128_config() -> LioConfig {
    LioConfig {
        blind: 1.0,
        filter_size_surf: 0.25,
        filter_size_map: 0.25,
        ..LioConfig::default()
    }
}

/// One processed frame.
#[derive(Clone, Debug)]
pub struct FrameResult {
    pub output: FrameOutput,
    /// One ray per raw point of the frame, in the decoder's point order.
    pub rays: Vec<Ray>,
}

pub struct Pipeline {
    decoder: Decoder,
    estimator: Estimator,
    events: Vec<Event>,
}

impl Pipeline {
    pub fn new(
        angles: &AngleCorrection,
        firetimes: Option<&Firetimes>,
        decoder: DecoderConfig,
        lio: LioConfig,
    ) -> Self {
        Self {
            decoder: Decoder::new(angles, firetimes, decoder),
            estimator: Estimator::new(lio),
            events: Vec::new(),
        }
    }

    /// See [`Estimator::dump_input_to`].
    pub fn dump_input_to(&mut self, w: Box<dyn Write + Send>) {
        self.estimator.dump_input_to(w);
    }

    pub fn decoder_stats(&self) -> &jt128::DecoderStats {
        &self.decoder.stats
    }

    pub fn lio(&self) -> &Lio {
        self.estimator.lio()
    }

    pub fn lio_mut(&mut self) -> &mut Lio {
        self.estimator.lio_mut()
    }

    /// Feeds one UDP payload. Frames completed by it are appended to `out`.
    pub fn push_packet(
        &mut self,
        payload: &[u8],
        out: &mut Vec<FrameResult>,
    ) -> Result<(), jt128::packet::PacketError> {
        self.decoder.push(payload, &mut self.events)?;
        self.drain(out);
        Ok(())
    }

    /// Ends the stream: the frame in progress is decoded and processed if the
    /// IMU already covers it.
    pub fn finish(&mut self, out: &mut Vec<FrameResult>) {
        self.decoder.flush(&mut self.events);
        self.drain(out);
    }

    fn drain(&mut self, out: &mut Vec<FrameResult>) {
        for event in self.events.drain(..) {
            self.estimator.push_event(event, out);
        }
    }
}

/// A frame FAST-LIO has finished, before its rays are computed.
pub struct Processed {
    pub output: FrameOutput,
    frame: jt128::Frame,
}

impl Processed {
    /// Computes one ray per raw point of the frame.
    pub fn rays(self) -> FrameResult {
        let Self { output, frame } = self;
        debug_assert_eq!(frame.t0, output.lidar_beg_time);
        let raw: Vec<RawPoint> = frame
            .points
            .iter()
            .map(|p| RawPoint {
                x: p.x,
                y: p.y,
                z: p.z,
                dt: f64::from(p.dt_ms) / 1000.0,
                range: p.range,
                intensity: p.intensity,
            })
            .collect();
        let mut rays = Vec::with_capacity(raw.len());
        frame_rays(
            &raw,
            frame.t0,
            output.timeline.as_ref(),
            &output.state,
            &mut rays,
        );
        FrameResult { output, rays }
    }
}

/// FAST-LIO on decoder events: events in, finished frames out.
pub struct Estimator {
    lio: Lio,
    pending: VecDeque<jt128::Frame>,
    input_dump: Option<Box<dyn Write + Send>>,
    busy: Duration,
    processed: Vec<Processed>,
}

impl Estimator {
    pub fn new(lio: LioConfig) -> Self {
        Self {
            lio: Lio::new(lio),
            pending: VecDeque::new(),
            input_dump: None,
            busy: Duration::ZERO,
            processed: Vec::new(),
        }
    }

    /// Time spent so far in FAST-LIO, rays excluded.
    pub fn busy(&self) -> Duration {
        self.busy
    }

    /// Records exactly what FAST-LIO is fed, for replaying through the C++
    /// reference (`fixtures/src/run_fastlio.cpp`). Little-endian events: tag 1
    /// is an IMU sample (`f64` t, acc×3, gyro×3); tag 2 is a preprocessed frame
    /// (`f64` begin time, `u32` count, then `f32` x, y, z, intensity and
    /// curvature per point).
    pub fn dump_input_to(&mut self, w: Box<dyn Write + Send>) {
        self.input_dump = Some(w);
    }

    pub fn lio(&self) -> &Lio {
        &self.lio
    }

    pub fn lio_mut(&mut self) -> &mut Lio {
        &mut self.lio
    }

    /// Feeds one decoder event. Frames it completes are appended to `out`,
    /// with their rays.
    pub fn push_event(&mut self, event: Event, out: &mut Vec<FrameResult>) {
        let mut done = std::mem::take(&mut self.processed);
        self.process(event, &mut done);
        out.extend(done.drain(..).map(Processed::rays));
        self.processed = done;
    }

    /// Feeds one decoder event. Frames it completes are appended to `out`;
    /// their rays are left to [`Processed::rays`].
    pub fn process(&mut self, event: Event, out: &mut Vec<Processed>) {
        let t = Instant::now();
        match event {
            Event::Imu(s) => {
                if let Some(w) = self.input_dump.as_mut() {
                    let mut rec = vec![1u8];
                    for v in std::iter::once(s.t).chain(s.acc).chain(s.gyro) {
                        rec.extend_from_slice(&v.to_le_bytes());
                    }
                    w.write_all(&rec).expect("input dump write failed");
                }
                self.lio.push_imu(ImuMeasurement {
                    t: s.t,
                    acc: Vec3::from_array(s.acc),
                    gyro: Vec3::from_array(s.gyro),
                });
            }
            Event::Frame(f) => {
                let points = f.points.iter().map(|p| Point {
                    x: p.x,
                    y: p.y,
                    z: p.z,
                    intensity: p.intensity,
                    curvature: p.dt_ms,
                });
                let kept = self.lio.preprocess(points);
                if let Some(w) = self.input_dump.as_mut() {
                    let mut rec = vec![2u8];
                    rec.extend_from_slice(&f.t0.to_le_bytes());
                    rec.extend_from_slice(&(kept.len() as u32).to_le_bytes());
                    for p in &kept {
                        for v in [p.x, p.y, p.z, p.intensity, p.curvature] {
                            rec.extend_from_slice(&v.to_le_bytes());
                        }
                    }
                    w.write_all(&rec).expect("input dump write failed");
                }
                self.lio.push_preprocessed_frame(f.t0, kept);
                self.pending.push_back(f);
            }
        }
        for output in self.lio.step() {
            let frame = self
                .pending
                .pop_front()
                .expect("every processed frame was pushed");
            out.push(Processed { output, frame });
        }
        self.busy += t.elapsed();
    }
}

/// The LiDAR pose (position, quaternion w,x,y,z) at a frame's end.
pub fn lidar_pose(o: &FrameOutput) -> ([f64; 3], [f64; 4]) {
    let s = &o.state;
    let p = s.pos + s.rot.rotate(&s.offset_t_l_i);
    let q = s.rot * s.offset_r_l_i;
    ([p[0], p[1], p[2]], [q.w, q.x, q.y, q.z])
}

/// Binary ray stream, little-endian: the 8-byte magic `LIORAYS1`, then per
/// frame a `u32` ray count, `f64` frame begin and end times and a `u8` frame
/// kind, then that many 40-byte records of `f64 t`, `f32×3 origin`,
/// `f32×3 direction`, `f32 range`, `f32 intensity`.
pub struct RayWriter<W: Write> {
    w: W,
    stride: usize,
    /// One frame's records, written in a single call.
    buf: Vec<u8>,
}

pub const RAY_MAGIC: &[u8; 8] = b"LIORAYS1";

impl<W: Write> RayWriter<W> {
    /// Keeps every `stride`-th ray (1 keeps all).
    pub fn new(mut w: W, stride: usize) -> io::Result<Self> {
        w.write_all(RAY_MAGIC)?;
        Ok(Self {
            w,
            stride: stride.max(1),
            buf: Vec::new(),
        })
    }

    pub fn write_frame(&mut self, r: &FrameResult) -> io::Result<()> {
        let kept = r.rays.len().div_ceil(self.stride);
        let b = &mut self.buf;
        b.clear();
        b.reserve(21 + 40 * kept);
        b.extend_from_slice(&(kept as u32).to_le_bytes());
        b.extend_from_slice(&r.output.lidar_beg_time.to_le_bytes());
        b.extend_from_slice(&r.output.lidar_end_time.to_le_bytes());
        b.push(kind_code(r.output.kind));
        for ray in r.rays.iter().step_by(self.stride) {
            b.extend_from_slice(&ray.t.to_le_bytes());
            for v in ray.origin.iter().chain(&ray.direction) {
                b.extend_from_slice(&v.to_le_bytes());
            }
            b.extend_from_slice(&ray.range.to_le_bytes());
            b.extend_from_slice(&ray.intensity.to_le_bytes());
        }
        // Flushed per frame, so a reader on a pipe gets each frame as it is done.
        self.w.write_all(b)?;
        self.w.flush()
    }

    pub fn into_inner(self) -> W {
        self.w
    }
}

pub fn kind_code(k: FrameKind) -> u8 {
    match k {
        FrameKind::FirstScan => 0,
        FrameKind::ImuInit => 1,
        FrameKind::MapInit => 2,
        FrameKind::Updated => 3,
        FrameKind::Skipped => 4,
    }
}

/// Writes points as an ASCII PCD file (`x y z intensity`).
pub fn write_pcd(w: &mut impl Write, points: &[Point]) -> io::Result<()> {
    writeln!(w, "# .PCD v0.7 - Point Cloud Data file format")?;
    writeln!(
        w,
        "VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1"
    )?;
    writeln!(
        w,
        "WIDTH {}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {}\nDATA ascii",
        points.len(),
        points.len()
    )?;
    for p in points {
        writeln!(w, "{} {} {} {}", p.x, p.y, p.z, p.intensity)?;
    }
    Ok(())
}
