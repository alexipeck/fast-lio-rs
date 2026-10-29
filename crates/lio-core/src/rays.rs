//! The per-point output: for every raw point, the ray it was measured along,
//! in the world frame of the corrected path.
//!
//! Each point's LiDAR pose comes from its frame's IMU timeline, interpolated
//! at the point's own time the way `UndistortPcl` does it:
//! `R = R_head·Exp(ω·dt)` and `p = p_head + v·dt + ½·a·dt²`. That pose is then
//! anchored to the frame's LiDAR-corrected end state, exactly as FAST-LIO
//! places points in its map. This is an output FAST-LIO itself doesn't have;
//! it doesn't feed back into the estimator.

use rayon::prelude::*;

use crate::imu_process::{Timeline, exp_rate};
use crate::mat::{Mat3, Vec3};
use crate::state::State;

/// One raw point as the decoder gives it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RawPoint {
    /// Position in the LiDAR frame, metres.
    pub x: f32,
    pub y: f32,
    pub z: f32,
    /// Seconds since the frame's `lidar_beg_time`.
    pub dt: f64,
    pub range: f32,
    pub intensity: f32,
}

/// The measurement ray of one point.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Ray {
    /// Point time on the sensor clock, seconds.
    pub t: f64,
    /// LiDAR position in the world frame at `t`, metres.
    pub origin: [f32; 3],
    /// Unit beam direction in the world frame.
    pub direction: [f32; 3],
    /// Measured range, metres; the point is `origin + range·direction`.
    pub range: f32,
    pub intensity: f32,
}

/// A rigid transform `x ↦ r·x + t`.
#[derive(Clone, Copy, Debug)]
struct Rigid {
    r: Mat3,
    t: Vec3,
}

impl Rigid {
    fn of_state_imu(s: &State) -> Self {
        Self {
            r: s.rot.to_matrix(),
            t: s.pos,
        }
    }

    fn of_extrinsic(s: &State) -> Self {
        Self {
            r: s.offset_r_l_i.to_matrix(),
            t: s.offset_t_l_i,
        }
    }

    fn inverse(&self) -> Self {
        let rt = self.r.transpose();
        Self {
            r: rt,
            t: -(rt * self.t),
        }
    }

    fn then(&self, inner: &Rigid) -> Self {
        Self {
            r: self.r * inner.r,
            t: self.r * inner.t + self.t,
        }
    }
}

fn to_f32(v: &Vec3) -> [f32; 3] {
    [v[0] as f32, v[1] as f32, v[2] as f32]
}

/// Computes one ray per point. Without a timeline (frames before the IMU is
/// initialised, when the sensor must be still), every point uses `state`.
pub fn frame_rays(
    points: &[RawPoint],
    lidar_beg_time: f64,
    timeline: Option<&Timeline>,
    state: &State,
    out: &mut Vec<Ray>,
) {
    let ext_c = Rigid::of_extrinsic(state);
    let start = out.len();
    out.resize(start + points.len(), Ray::default());
    let dst = &mut out[start..];

    let Some(tl) = timeline.filter(|tl| !tl.poses.is_empty()) else {
        let pose = Rigid::of_state_imu(state).then(&ext_c);
        dst.par_iter_mut()
            .zip(points.par_iter())
            .for_each(|(ray, p)| {
                *ray = make_ray(&pose, p, lidar_beg_time);
            });
        return;
    };

    // World(corrected) ← LiDAR(end, corrected) ← LiDAR(end, predicted) ← world(predicted).
    let ext_p = Rigid::of_extrinsic(&tl.end_state);
    let anchor = Rigid::of_state_imu(state)
        .then(&ext_c)
        .then(&ext_p.inverse())
        .then(&Rigid::of_state_imu(&tl.end_state).inverse());
    let poses = &tl.poses;
    dst.par_iter_mut()
        .zip(points.par_iter())
        .for_each(|(ray, p)| {
            // The latest pose strictly before the point, as UndistortPcl picks it;
            // points not after the first pose extrapolate from it.
            let k = poses.partition_point(|q| q.offset_time < p.dt);
            let head = k.saturating_sub(1).min(poses.len().saturating_sub(2));
            let tail = (head + 1).min(poses.len() - 1);
            let (h, tl_) = (&poses[head], &poses[tail]);
            let dt = p.dt - h.offset_time;
            let imu = Rigid {
                r: h.rot * exp_rate(&tl_.gyr, dt),
                t: h.pos + h.vel * dt + tl_.acc * 0.5 * dt * dt,
            };
            let lidar = anchor.then(&imu).then(&ext_p);
            *ray = make_ray(&lidar, p, lidar_beg_time);
        });
}

fn make_ray(lidar: &Rigid, p: &RawPoint, lidar_beg_time: f64) -> Ray {
    let v = Vec3::new(f64::from(p.x), f64::from(p.y), f64::from(p.z));
    let n = v.norm();
    let dir = if n > 0.0 {
        lidar.r * (v / n)
    } else {
        Vec3::zeros()
    };
    Ray {
        t: lidar_beg_time + p.dt,
        origin: to_f32(&lidar.t),
        direction: to_f32(&dir),
        range: p.range,
        intensity: p.intensity,
    }
}
