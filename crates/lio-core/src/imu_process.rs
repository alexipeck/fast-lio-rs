//! FAST-LIO's IMU front end, `ImuProcess` in `src/IMU_Processing.hpp`:
//! initialisation over the first frames, then per frame the forward
//! propagation at every IMU sample (the frame's motion timeline) and the
//! backward motion compensation of each point into the end-of-frame body.

use rayon::prelude::*;

use crate::esekf::Esekf;
use crate::imu_init::{G_M_S2, ImuInit, ImuMeasurement};
use crate::mat::{Mat3, Vec3};
use crate::point::Point;
use crate::process::{Input, ProcessNoise, process_noise_cov};
use crate::state::State;

/// One LiDAR frame and the IMU samples that cover it (`MeasureGroup`).
#[derive(Clone, Debug, Default)]
pub struct MeasureGroup {
    pub lidar_beg_time: f64,
    pub lidar_end_time: f64,
    /// Points with `curvature` = time since `lidar_beg_time`, in ms.
    pub lidar: Vec<Point>,
    pub imu: Vec<ImuMeasurement>,
}

/// The state at one IMU sample of a frame (`Pose6D`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TimelinePose {
    /// Seconds since `lidar_beg_time`.
    pub offset_time: f64,
    /// Acceleration in the world frame, gravity included.
    pub acc: Vec3,
    /// Bias-corrected angular rate in the body frame.
    pub gyr: Vec3,
    pub vel: Vec3,
    pub pos: Vec3,
    pub rot: Mat3,
}

/// A frame's motion timeline: the IMU-rate poses and the propagated state at
/// the frame's end, before the LiDAR correction.
#[derive(Clone, Debug)]
pub struct Timeline {
    pub lidar_beg_time: f64,
    pub poses: Vec<TimelinePose>,
    pub end_state: State,
}

/// `Exp(ang_vel, dt)` from `include/so3_math.h`: Rodrigues' formula, identity
/// below 1e-7 rad/s.
pub fn exp_rate(ang_vel: &Vec3, dt: f64) -> Mat3 {
    let norm = ang_vel.norm();
    if norm > 0.0000001 {
        let k = (*ang_vel / norm).hat();
        let r_ang = norm * dt;
        // One sincos call, as GCC emits for FAST-LIO's adjacent sin and cos.
        let (sin, cos) = r_ang.sin_cos();
        Mat3::identity() + k * sin + k * (1.0 - cos) * k
    } else {
        Mat3::identity()
    }
}

#[derive(Clone, Debug)]
pub struct ImuProcess {
    q: ProcessNoise,
    cov_acc: Vec3,
    cov_gyr: Vec3,
    cov_acc_scale: Vec3,
    cov_gyr_scale: Vec3,
    cov_bias_gyr: Vec3,
    cov_bias_acc: Vec3,
    lidar_t_wrt_imu: Vec3,
    lidar_r_wrt_imu: Mat3,
    init: ImuInit,
    imu_need_init: bool,
    last_imu: ImuMeasurement,
    angvel_last: Vec3,
    acc_s_last: Vec3,
    last_lidar_end_time: f64,
}

impl ImuProcess {
    pub fn new(
        lidar_t_wrt_imu: Vec3,
        lidar_r_wrt_imu: Mat3,
        gyr_cov: f64,
        acc_cov: f64,
        b_gyr_cov: f64,
        b_acc_cov: f64,
    ) -> Self {
        let c = |v: f64| Vec3::new(v, v, v);
        Self {
            q: process_noise_cov(),
            cov_acc: c(0.1),
            cov_gyr: c(0.1),
            cov_acc_scale: c(acc_cov),
            cov_gyr_scale: c(gyr_cov),
            cov_bias_gyr: c(b_gyr_cov),
            cov_bias_acc: c(b_acc_cov),
            lidar_t_wrt_imu,
            lidar_r_wrt_imu,
            init: ImuInit::default(),
            imu_need_init: true,
            last_imu: ImuMeasurement {
                t: 0.0,
                acc: Vec3::zeros(),
                gyro: Vec3::zeros(),
            },
            angvel_last: Vec3::zeros(),
            acc_s_last: Vec3::zeros(),
            last_lidar_end_time: 0.0,
        }
    }

    pub fn initialised(&self) -> bool {
        !self.imu_need_init
    }

    /// `Process`. While initialising, returns `None`. Afterwards it propagates
    /// `kf` to the frame's end and returns the frame's points compensated into
    /// the end-of-frame LiDAR frame (sorted by time), with the timeline used.
    pub fn process(
        &mut self,
        meas: &MeasureGroup,
        kf: &mut Esekf,
    ) -> Option<(Vec<Point>, Timeline)> {
        let last = *meas.imu.last()?;
        if self.imu_need_init {
            let done = self.init.feed(
                &meas.imu,
                meas.lidar_beg_time,
                kf,
                &self.lidar_t_wrt_imu,
                &self.lidar_r_wrt_imu,
            );
            self.last_imu = last;
            if done {
                self.imu_need_init = false;
                self.cov_acc = self.cov_acc_scale;
                self.cov_gyr = self.cov_gyr_scale;
            }
            return None;
        }
        Some(self.undistort(meas, kf))
    }

    /// `UndistortPcl`.
    fn undistort(&mut self, meas: &MeasureGroup, kf: &mut Esekf) -> (Vec<Point>, Timeline) {
        let mut v_imu = Vec::with_capacity(meas.imu.len() + 1);
        v_imu.push(self.last_imu);
        v_imu.extend_from_slice(&meas.imu);
        let imu_end_time = v_imu.last().unwrap().t;
        let pcl_beg_time = meas.lidar_beg_time;
        let pcl_end_time = meas.lidar_end_time;

        let mut pcl_out = meas.lidar.clone();
        pcl_out.sort_by(|a, b| a.curvature.total_cmp(&b.curvature));

        let mut imu_state = kf.x;
        let mut poses = vec![TimelinePose {
            offset_time: 0.0,
            acc: self.acc_s_last,
            gyr: self.angvel_last,
            vel: imu_state.vel,
            pos: imu_state.pos,
            rot: imu_state.rot.to_matrix(),
        }];

        let mut input = Input::default();
        let mean_acc_norm = self.init.mean_acc().norm();
        for w in v_imu.windows(2) {
            let (head, tail) = (&w[0], &w[1]);
            if tail.t < self.last_lidar_end_time {
                continue;
            }
            let angvel_avr = (head.gyro + tail.gyro) * 0.5;
            let acc_avr = (head.acc + tail.acc) * 0.5 * G_M_S2 / mean_acc_norm;
            let dt = if head.t < self.last_lidar_end_time {
                tail.t - self.last_lidar_end_time
            } else {
                tail.t - head.t
            };
            input.acc = acc_avr;
            input.gyro = angvel_avr;
            for i in 0..3 {
                self.q[(i, i)] = self.cov_gyr[i];
                self.q[(3 + i, 3 + i)] = self.cov_acc[i];
                self.q[(6 + i, 6 + i)] = self.cov_bias_gyr[i];
                self.q[(9 + i, 9 + i)] = self.cov_bias_acc[i];
            }
            kf.predict(dt, &self.q, &input);

            imu_state = kf.x;
            self.angvel_last = angvel_avr - imu_state.bg;
            self.acc_s_last = imu_state.rot.rotate(&(acc_avr - imu_state.ba)) + imu_state.grav.vec;
            poses.push(TimelinePose {
                offset_time: tail.t - pcl_beg_time,
                acc: self.acc_s_last,
                gyr: self.angvel_last,
                vel: imu_state.vel,
                pos: imu_state.pos,
                rot: imu_state.rot.to_matrix(),
            });
        }

        // Predict to the frame's end. FAST-LIO takes |pcl_end - imu_end|, so it
        // always steps forward.
        let note = if pcl_end_time > imu_end_time {
            1.0
        } else {
            -1.0
        };
        let dt = note * (pcl_end_time - imu_end_time);
        kf.predict(dt, &self.q, &input);
        imu_state = kf.x;
        self.last_imu = *meas.imu.last().unwrap();
        self.last_lidar_end_time = pcl_end_time;

        compensate(&mut pcl_out, &poses, &imu_state);
        (
            pcl_out,
            Timeline {
                lidar_beg_time: pcl_beg_time,
                poses,
                end_state: imu_state,
            },
        )
    }
}

/// The backward compensation loop of `UndistortPcl`. The C++ walks the points
/// back from the newest: each segment's points use its head pose, and points
/// not later than the first pose stay as they are. Once the walk reaches the
/// first point it stays there, so that point is compensated again for every
/// earlier segment whose head it is later than.
///
/// The walk is replayed here on times alone, to find the segment, or for the
/// first point the segments, each point gets. Every point is then compensated
/// on its own, in parallel, with the same arithmetic in the same order.
fn compensate(pcl: &mut [Point], poses: &[TimelinePose], end: &State) {
    if pcl.is_empty() {
        return;
    }
    // Tail pose of each point's segment; 0 leaves the point as it is.
    let mut segment = vec![0usize; pcl.len()];
    let mut first_again = Vec::new();
    let mut i = pcl.len() - 1;
    for k in (1..poses.len()).rev() {
        let head = &poses[k - 1];
        loop {
            if f64::from(pcl[i].curvature) / 1000.0 <= head.offset_time {
                break;
            }
            if i == 0 {
                if segment[0] == 0 {
                    segment[0] = k;
                } else {
                    first_again.push(k);
                }
                break;
            }
            segment[i] = k;
            i -= 1;
        }
    }
    pcl.par_iter_mut()
        .zip(segment.par_iter())
        .filter(|(_, k)| **k > 0)
        .for_each(|(p, &k)| compensate_point(p, &poses[k - 1], &poses[k], end));
    for k in first_again {
        compensate_point(&mut pcl[0], &poses[k - 1], &poses[k], end);
    }
}

/// Moves one point, measured during the segment from `head` to `tail`, into
/// the end-of-frame LiDAR frame.
fn compensate_point(p: &mut Point, head: &TimelinePose, tail: &TimelinePose, end: &State) {
    let dt = f64::from(p.curvature) / 1000.0 - head.offset_time;
    let r_i = head.rot * exp_rate(&tail.gyr, dt);
    let p_i = Vec3::new(f64::from(p.x), f64::from(p.y), f64::from(p.z));
    let t_ei = head.pos + head.vel * dt + tail.acc * 0.5 * dt * dt - end.pos;
    let in_imu = end.offset_r_l_i.rotate(&p_i) + end.offset_t_l_i;
    let world_rel = r_i * in_imu + t_ei;
    let p_c = end
        .offset_r_l_i
        .conjugate()
        .rotate(&(end.rot.conjugate().rotate(&world_rel) - end.offset_t_l_i));
    p.x = p_c[0] as f32;
    p.y = p_c[1] as f32;
    p.z = p_c[2] as f32;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::so3;

    /// The C++ loop as written, one point after another.
    fn compensate_serial(pcl: &mut [Point], poses: &[TimelinePose], end: &State) {
        if pcl.is_empty() {
            return;
        }
        let mut i = pcl.len() - 1;
        for k in (1..poses.len()).rev() {
            loop {
                if f64::from(pcl[i].curvature) / 1000.0 <= poses[k - 1].offset_time {
                    break;
                }
                compensate_point(&mut pcl[i], &poses[k - 1], &poses[k], end);
                if i == 0 {
                    break;
                }
                i -= 1;
            }
        }
    }

    /// xorshift64, uniform in [-1, 1).
    struct Rng(u64);

    impl Rng {
        fn next(&mut self) -> f64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            (self.0 >> 11) as f64 / (1u64 << 52) as f64 - 1.0
        }

        fn vec(&mut self, scale: f64) -> Vec3 {
            Vec3::new(self.next(), self.next(), self.next()) * scale
        }
    }

    #[test]
    fn parallel_compensation_matches_the_serial_walk() {
        let mut rng = Rng(0x9e37_79b9_7f4a_7c15);
        let end = State {
            pos: rng.vec(2.0),
            rot: so3::exp(&rng.vec(1.0), 1.0),
            offset_r_l_i: so3::exp(&rng.vec(1.0), 1.0),
            offset_t_l_i: rng.vec(0.1),
            ..State::default()
        };
        // Pose offsets in ms, and the first point's time in ms.
        let cases: [(&[f64], f64); 5] = [
            // A usual frame: the first point at the start.
            (&[0.0, 5.0, 10.0, 15.0], 0.0),
            // The first point is later than three poses, so the C++ loop
            // compensates it three times.
            (&[0.0, 0.002, 0.004, 5.0], 1.0),
            // An IMU sample just before the frame start.
            (&[0.0, -1.0, 4.0, 9.0], 0.5),
            // A single pose: nothing moves.
            (&[0.0], 0.0),
            // Points before the first pose stay as they are.
            (&[0.0, 5.0], -1.0),
        ];
        for (offsets_ms, first_ms) in cases {
            let poses: Vec<TimelinePose> = offsets_ms
                .iter()
                .map(|ms| TimelinePose {
                    offset_time: ms / 1000.0,
                    acc: rng.vec(3.0),
                    gyr: rng.vec(2.0),
                    vel: rng.vec(1.0),
                    pos: rng.vec(1.0),
                    rot: so3::exp(&rng.vec(1.0), 1.0).to_matrix(),
                })
                .collect();
            let mut points: Vec<Point> = (0..2000)
                .map(|j| {
                    let v = rng.vec(20.0);
                    Point {
                        curvature: (first_ms + f64::from(j) * 0.01) as f32,
                        ..Point::new(v[0] as f32, v[1] as f32, v[2] as f32)
                    }
                })
                .collect();
            let mut serial = points.clone();
            compensate_serial(&mut serial, &poses, &end);
            compensate(&mut points, &poses, &end);
            for (a, b) in points.iter().zip(&serial) {
                for (u, v) in [(a.x, b.x), (a.y, b.y), (a.z, b.z)] {
                    assert_eq!(u.to_bits(), v.to_bits(), "{offsets_ms:?}");
                }
            }
        }
    }
}
