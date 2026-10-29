//! FAST-LIO's IMU front end, `ImuProcess` in `src/IMU_Processing.hpp`:
//! initialisation over the first frames, then per frame the forward
//! propagation at every IMU sample (the frame's motion timeline) and the
//! backward motion compensation of each point into the end-of-frame body.

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

/// The backward compensation loop of `UndistortPcl`, kept to its exact shape:
/// points walk back from the newest, each segment's points use its head pose,
/// and points not later than the first pose stay as they are.
fn compensate(pcl: &mut [Point], poses: &[TimelinePose], end: &State) {
    if pcl.is_empty() {
        return;
    }
    let mut i = pcl.len() - 1;
    for k in (1..poses.len()).rev() {
        let head = &poses[k - 1];
        let tail = &poses[k];
        loop {
            let t = f64::from(pcl[i].curvature) / 1000.0;
            if t <= head.offset_time {
                break;
            }
            let dt = t - head.offset_time;
            let r_i = head.rot * exp_rate(&tail.gyr, dt);
            let p_i = Vec3::new(
                f64::from(pcl[i].x),
                f64::from(pcl[i].y),
                f64::from(pcl[i].z),
            );
            let t_ei = head.pos + head.vel * dt + tail.acc * 0.5 * dt * dt - end.pos;
            let in_imu = end.offset_r_l_i.rotate(&p_i) + end.offset_t_l_i;
            let world_rel = r_i * in_imu + t_ei;
            let p_c = end
                .offset_r_l_i
                .conjugate()
                .rotate(&(end.rot.conjugate().rotate(&world_rel) - end.offset_t_l_i));
            pcl[i].x = p_c[0] as f32;
            pcl[i].y = p_c[1] as f32;
            pcl[i].z = p_c[2] as f32;
            if i == 0 {
                break;
            }
            i -= 1;
        }
    }
}
