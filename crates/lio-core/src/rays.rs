//! The per-point output: for every raw point, the ray it was measured along,
//! in the world frame of the corrected path.
//!
//! Each point's LiDAR pose comes from its frame's IMU timeline, interpolated
//! at the point's own time the way `UndistortPcl` does it:
//! `R = R_head·Exp(ω·dt)` and `p = p_head + v·dt + ½·a·dt²`. That pose is then
//! anchored to the frame's LiDAR-corrected end state, exactly as FAST-LIO
//! places points in its map. This is an output FAST-LIO itself doesn't have;
//! it doesn't feed back into the estimator.
//!
//! Everything but `Exp(ω·dt)` is fixed within an IMU segment, so it is
//! composed once per segment; per point, `Exp(ω·dt)` is applied to vectors
//! with Rodrigues' formula.

use rayon::prelude::*;

use crate::imu_process::{Timeline, TimelinePose};
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
    let segments: Vec<Segment> = (0..poses.len().saturating_sub(1).max(1))
        .map(|head| {
            let tail = (head + 1).min(poses.len() - 1);
            Segment::new(&anchor, &poses[head], &poses[tail])
        })
        .collect();
    dst.par_iter_mut()
        .zip(points.par_iter())
        .for_each(|(ray, p)| {
            // The latest pose strictly before the point, as UndistortPcl picks it;
            // points not after the first pose extrapolate from it.
            let k = poses.partition_point(|q| q.offset_time < p.dt);
            let s = &segments[k.saturating_sub(1).min(segments.len() - 1)];
            let dt = p.dt - s.offset_time;
            let spin = s.spin(dt);
            let v = Vec3::new(f64::from(p.x), f64::from(p.y), f64::from(p.z));
            let n = v.norm();
            let dir = if n > 0.0 {
                s.r * spin.apply(&(ext_p.r * (v / n)))
            } else {
                Vec3::zeros()
            };
            let origin = s.p0 + s.p1 * dt + s.p2 * (dt * dt) + s.r * spin.apply(&ext_p.t);
            *ray = Ray {
                t: lidar_beg_time + p.dt,
                origin: to_f32(&origin),
                direction: to_f32(&dir),
                range: p.range,
                intensity: p.intensity,
            };
        });
}

/// The parts of `anchor · (R_head·Exp(ω·dt), p_head + v·dt + ½·a·dt²)` that
/// are fixed within one IMU segment.
struct Segment {
    offset_time: f64,
    /// `anchor.r · R_head`.
    r: Mat3,
    /// The position is `p0 + p1·dt + p2·dt²` plus the rotated lever arm.
    p0: Vec3,
    p1: Vec3,
    p2: Vec3,
    /// Unit axis and rate of `Exp(ω·dt)`; `None` at rates FAST-LIO's `Exp`
    /// treats as zero.
    axis: Option<(Vec3, f64)>,
}

impl Segment {
    fn new(anchor: &Rigid, head: &TimelinePose, tail: &TimelinePose) -> Self {
        let rate = tail.gyr.norm();
        Self {
            offset_time: head.offset_time,
            r: anchor.r * head.rot,
            p0: anchor.r * head.pos + anchor.t,
            p1: anchor.r * head.vel,
            p2: anchor.r * (tail.acc * 0.5),
            axis: (rate > 0.0000001).then(|| (tail.gyr / rate, rate)),
        }
    }

    fn spin(&self, dt: f64) -> Spin {
        match self.axis {
            Some((k, rate)) => {
                let (sin, cos) = (rate * dt).sin_cos();
                Spin { k, sin, cos }
            }
            None => Spin {
                k: Vec3::zeros(),
                sin: 0.0,
                cos: 1.0,
            },
        }
    }
}

/// A rotation by an angle about a unit axis, for Rodrigues' formula.
struct Spin {
    k: Vec3,
    sin: f64,
    cos: f64,
}

impl Spin {
    fn apply(&self, w: &Vec3) -> Vec3 {
        let kw = self.k.cross(w);
        *w + kw * self.sin + self.k.cross(&kw) * (1.0 - self.cos)
    }
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::imu_process::exp_rate;
    use crate::so3;

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

        fn state(&mut self) -> State {
            State {
                pos: self.vec(5.0),
                rot: so3::exp(&self.vec(1.0), 1.0),
                offset_r_l_i: so3::exp(&self.vec(1.0), 1.0),
                offset_t_l_i: self.vec(0.1),
                ..State::default()
            }
        }
    }

    /// The per-segment composition gives the rays of composing every point's
    /// pose in full, to float precision.
    #[test]
    fn segment_rays_match_the_full_composition() {
        let mut rng = Rng(0x2545_f491_4f6c_dd1d);
        let state = rng.state();
        let end_state = rng.state();
        let poses: Vec<TimelinePose> = (0..21)
            .map(|k| TimelinePose {
                offset_time: f64::from(k) * 0.005,
                acc: rng.vec(3.0),
                // One segment below the rate FAST-LIO's Exp treats as zero.
                gyr: if k == 7 { Vec3::zeros() } else { rng.vec(2.0) },
                vel: rng.vec(1.0),
                pos: rng.vec(1.0),
                rot: so3::exp(&rng.vec(1.0), 1.0).to_matrix(),
            })
            .collect();
        let timeline = Timeline {
            lidar_beg_time: 100.0,
            poses,
            end_state,
        };
        let points: Vec<RawPoint> = (0..5000)
            .map(|j| {
                let v = rng.vec(30.0);
                RawPoint {
                    x: v[0] as f32,
                    y: v[1] as f32,
                    z: v[2] as f32,
                    // From before the first pose to past the last.
                    dt: -0.002 + f64::from(j) * 0.000_022,
                    range: v.norm() as f32,
                    intensity: 1.0,
                }
            })
            .collect();
        let mut rays = Vec::new();
        frame_rays(&points, 100.0, Some(&timeline), &state, &mut rays);

        let ext_p = Rigid::of_extrinsic(&end_state);
        let anchor = Rigid::of_state_imu(&state)
            .then(&Rigid::of_extrinsic(&state))
            .then(&ext_p.inverse())
            .then(&Rigid::of_state_imu(&end_state).inverse());
        let poses = &timeline.poses;
        for (p, ray) in points.iter().zip(&rays) {
            let k = poses.partition_point(|q| q.offset_time < p.dt);
            let head = k.saturating_sub(1).min(poses.len() - 2);
            let (h, t) = (&poses[head], &poses[head + 1]);
            let dt = p.dt - h.offset_time;
            let imu = Rigid {
                r: h.rot * exp_rate(&t.gyr, dt),
                t: h.pos + h.vel * dt + t.acc * 0.5 * dt * dt,
            };
            let want = make_ray(&anchor.then(&imu).then(&ext_p), p, 100.0);
            assert_eq!(ray.t, want.t);
            for i in 0..3 {
                assert!(
                    (ray.origin[i] - want.origin[i]).abs() <= 2e-6,
                    "{ray:?} {want:?}"
                );
                assert!(
                    (ray.direction[i] - want.direction[i]).abs() <= 2e-7,
                    "{ray:?} {want:?}"
                );
            }
        }
    }
}
