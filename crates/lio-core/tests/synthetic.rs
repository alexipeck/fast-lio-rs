//! Synthetic end-to-end test with ground truth.
//!
//! A JT128-like LiDAR and a 200 Hz IMU move through a box-shaped room with box
//! obstacles:
//! - **LiDAR:** 128 beams from −5° to +89°, 1° azimuth steps, ±2 cm range
//!   noise. It is mounted like the JT128's, turned 180° about z, and offset
//!   from the IMU.
//! - **Motion:** the sensor is still for 2 s, then moves smoothly. It turns up
//!   to 1 rad and travels up to 1.6 m.
//!
//! Two things are checked:
//! - **End-of-frame poses** against the truth. A box room's corners bias
//!   FAST-LIO's point-to-plane matching by a centimetre or two. That bias is
//!   FAST-LIO's, not this port's.
//! - **Every point's ray**, also after taking out its frame's end-pose error.
//!   That measures what the per-point interpolation along the frame's IMU
//!   timeline adds.

use std::collections::VecDeque;
use std::f64::consts::TAU;

use lio_core::imu_init::ImuMeasurement;
use lio_core::lio::{FrameKind, FrameOutput, Lio, LioConfig};
use lio_core::mat::{Mat3, Vec3};
use lio_core::point::Point;
use lio_core::rays::{RawPoint, Ray, frame_rays};
use lio_core::so3::{self, Quat};

const G: f64 = 9.81;
const IMU_RATE: f64 = 200.0;
const FRAME: f64 = 0.1;
const STILL: f64 = 2.0;
const FRAMES: usize = 100;
const CHANNELS: usize = 128;
const COLUMNS: usize = 360;
/// Full width of the uniform range noise, m.
const NOISE: f64 = 0.04;
/// LiDAR mounting in the IMU frame.
const EXT_R: [f64; 9] = [-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0];
const EXT_T: [f64; 3] = [0.03, -0.02, 0.06];

struct Aabb {
    min: [f64; 3],
    max: [f64; 3],
}

const ROOM: Aabb = Aabb {
    min: [-5.0, -4.0, -1.5],
    max: [5.0, 4.0, 2.5],
};
const OBSTACLES: [Aabb; 4] = [
    Aabb {
        min: [2.5, 1.5, -1.5],
        max: [3.5, 2.5, 0.5],
    },
    Aabb {
        min: [-3.0, -2.5, -1.5],
        max: [-2.0, -1.0, 1.0],
    },
    Aabb {
        min: [-1.5, 2.5, -1.5],
        max: [0.0, 3.2, -0.3],
    },
    Aabb {
        min: [3.0, -2.0, -1.5],
        max: [3.4, -1.6, 2.5],
    },
];

/// Distance along the unit ray `o + s·d` to the first surface, from inside the room.
fn cast(o: &Vec3, d: &Vec3) -> f64 {
    let mut best = f64::INFINITY;
    for a in 0..3 {
        if d[a] > 0.0 {
            best = best.min((ROOM.max[a] - o[a]) / d[a]);
        } else if d[a] < 0.0 {
            best = best.min((ROOM.min[a] - o[a]) / d[a]);
        }
    }
    for b in &OBSTACLES {
        let (mut near, mut far) = (f64::NEG_INFINITY, f64::INFINITY);
        for a in 0..3 {
            if d[a] == 0.0 {
                if o[a] < b.min[a] || o[a] > b.max[a] {
                    near = f64::INFINITY;
                }
                continue;
            }
            let (t0, t1) = ((b.min[a] - o[a]) / d[a], (b.max[a] - o[a]) / d[a]);
            near = near.max(t0.min(t1));
            far = far.min(t0.max(t1));
        }
        if near <= far && near > 0.0 {
            best = best.min(near);
        }
    }
    best
}

fn axis(x: f64, y: f64, z: f64) -> Quat {
    so3::exp(&Vec3::new(x, y, z), 1.0)
}

/// The IMU's pose in the world (z up) at `t`: still until [`STILL`], then a
/// smooth motion that starts from rest.
fn imu_pose(t: f64) -> (Vec3, Quat) {
    let s = (t - STILL).max(0.0);
    let c = |amp: f64, w: f64| amp * (1.0 - (w * s).cos());
    let pos = Vec3::new(c(0.8, 0.6), c(0.6, 0.45), c(0.15, 0.9));
    let q =
        axis(0.0, 0.0, c(0.5, 1.0)) * axis(0.0, c(-0.08, 1.1), 0.0) * axis(c(0.1, 0.7), 0.0, 0.0);
    (pos, q)
}

fn lidar_pose(t: f64) -> (Vec3, Quat) {
    let (p, q) = imu_pose(t);
    let r_il = Quat::from_matrix(&Mat3::from_row_slice(&EXT_R));
    (p + q.rotate(&Vec3::from_array(EXT_T)), q * r_il)
}

/// Noise-free IMU sample: body angular rate, and specific force in g.
fn imu_sample(t: f64) -> ImuMeasurement {
    let h = 1e-4;
    let ((p0, q0), (p1, q1), (p2, q2)) = (imu_pose(t - h), imu_pose(t), imu_pose(t + h));
    let acc_world = (p2 - p1 * 2.0 + p0) / (h * h);
    let gyro = so3::log(&(q0.conjugate() * q2)) / (2.0 * h);
    let force = q1.conjugate().rotate(&(acc_world + Vec3::new(0.0, 0.0, G)));
    ImuMeasurement {
        t,
        acc: force / G,
        gyro,
    }
}

/// A point's ground truth: the LiDAR's position and beam direction.
struct Truth {
    origin: Vec3,
    direction: Vec3,
}

/// One revolution starting at `t0`.
fn scan(t0: f64, rng: &mut u64) -> (Vec<RawPoint>, Vec<Truth>) {
    let (mut raw, mut truth) = (Vec::new(), Vec::new());
    for col in 0..COLUMNS {
        let dt = col as f64 / COLUMNS as f64 * FRAME;
        let (p, q) = lidar_pose(t0 + dt);
        let az = col as f64 / COLUMNS as f64 * TAU;
        for ch in 0..CHANNELS {
            let el = (-5.0 + 94.0 * ch as f64 / (CHANNELS - 1) as f64).to_radians();
            let d = Vec3::new(el.cos() * az.sin(), el.cos() * az.cos(), el.sin());
            let dw = q.rotate(&d);
            // xorshift64
            *rng ^= *rng << 13;
            *rng ^= *rng >> 7;
            *rng ^= *rng << 17;
            let noise = ((*rng >> 11) as f64 / (1u64 << 53) as f64 - 0.5) * NOISE;
            let r = cast(&p, &dw) + noise;
            let pl = d * r;
            raw.push(RawPoint {
                x: pl[0] as f32,
                y: pl[1] as f32,
                z: pl[2] as f32,
                dt,
                range: r as f32,
                intensity: 100.0,
            });
            truth.push(Truth {
                origin: p,
                direction: dw,
            });
        }
    }
    (raw, truth)
}

fn angle(a: &Quat, b: &Quat) -> f64 {
    so3::log(&(a.conjugate() * *b)).norm()
}

fn f64s(v: [f32; 3]) -> Vec3 {
    Vec3::new(f64::from(v[0]), f64::from(v[1]), f64::from(v[2]))
}

#[derive(Default)]
struct Errors {
    /// End-of-frame LiDAR pose against the truth.
    pose_pos: f64,
    pose_rot: f64,
    /// Ray end points against the true ones at the measured range.
    endpoint: Vec<f64>,
    /// Ray origin and direction after taking out the frame's end-pose error.
    rel_origin: f64,
    rel_direction: f64,
}

impl Errors {
    fn add(&mut self, out: &FrameOutput, rays: &[Ray], truth: &[Truth]) {
        let s = &out.state;
        let (est_p, est_q) = (
            s.pos + s.rot.rotate(&s.offset_t_l_i),
            s.rot * s.offset_r_l_i,
        );
        let (true_p, true_q) = lidar_pose(out.lidar_end_time);
        self.pose_pos = self.pose_pos.max((est_p - true_p).norm());
        self.pose_rot = self.pose_rot.max(angle(&est_q, &true_q));
        // The end-pose error as a world transform: x ↦ e·(x − true_p) + est_p.
        let e = est_q * true_q.conjugate();
        for (ray, t) in rays.iter().zip(truth) {
            let (o, d) = (f64s(ray.origin), f64s(ray.direction));
            let r = f64::from(ray.range);
            self.endpoint
                .push((o + d * r - (t.origin + t.direction * r)).norm());
            let o_rel = e.rotate(&(t.origin - true_p)) + est_p;
            let d_rel = e.rotate(&t.direction);
            self.rel_origin = self.rel_origin.max((o - o_rel).norm());
            self.rel_direction = self
                .rel_direction
                .max(d.dot(&d_rel).clamp(-1.0, 1.0).acos());
        }
    }

    /// Median and maximum ray end-point error.
    fn report(&mut self, what: &str) -> (f64, f64) {
        self.endpoint.sort_by(f64::total_cmp);
        let q = |f: f64| self.endpoint[((self.endpoint.len() - 1) as f64 * f) as usize];
        eprintln!(
            "{what}: {} rays. End pose within {:.2} mm and {:.3}°. Ray end points: median {:.2} mm, \
             99% {:.2} mm, max {:.2} mm. Within the frame: origin {:.3} mm, direction {:.4}°",
            self.endpoint.len(),
            self.pose_pos * 1e3,
            self.pose_rot.to_degrees(),
            q(0.5) * 1e3,
            q(0.99) * 1e3,
            q(1.0) * 1e3,
            self.rel_origin * 1e3,
            self.rel_direction.to_degrees(),
        );
        (q(0.5), q(1.0))
    }
}

#[test]
fn synthetic_imu_integrates_back_to_the_trajectory() {
    let dt = 1.0 / IMU_RATE;
    let (mut p, mut q) = imu_pose(0.0);
    let mut v = Vec3::new(0.0, 0.0, 0.0);
    let mut prev = imu_sample(0.0);
    for i in 1..=(FRAMES as f64 * FRAME * IMU_RATE) as usize {
        let t = i as f64 * dt;
        let cur = imu_sample(t);
        let q1 = q * so3::exp(&((prev.gyro + cur.gyro) * 0.5), dt);
        let up = Vec3::new(0.0, 0.0, G);
        let a = (q.rotate(&(prev.acc * G)) - up + q1.rotate(&(cur.acc * G)) - up) * 0.5;
        p += v * dt + a * (0.5 * dt * dt);
        v += a * dt;
        q = q1;
        prev = cur;
    }
    let (tp, tq) = imu_pose(FRAMES as f64 * FRAME);
    assert!(
        (p - tp).norm() < 1e-3,
        "dead reckoning ends {:.2} mm off",
        (p - tp).norm() * 1e3
    );
    assert!(
        angle(&q, &tq) < 1e-5,
        "dead reckoning ends {:e} rad off",
        angle(&q, &tq)
    );
}

#[test]
fn poses_and_rays_match_the_ground_truth() {
    let cfg = LioConfig {
        extrinsic_r: EXT_R,
        extrinsic_t: EXT_T,
        filter_size_surf: 0.25,
        filter_size_map: 0.25,
        ..LioConfig::default()
    };
    let mut lio = Lio::new(cfg);
    for i in 0..=(FRAMES as f64 * FRAME * IMU_RATE) as usize + 2 {
        lio.push_imu(imu_sample(i as f64 / IMU_RATE));
    }

    let mut rng = 0x9e37_79b9_7f4a_7c15u64;
    let mut pending = VecDeque::new();
    let (mut still, mut moving) = (Errors::default(), Errors::default());
    let mut kinds = Vec::new();
    for k in 0..FRAMES {
        let t0 = k as f64 * FRAME;
        let (raw, truth) = scan(t0, &mut rng);
        let points = raw.iter().map(|p| Point {
            x: p.x,
            y: p.y,
            z: p.z,
            intensity: p.intensity,
            curvature: (p.dt * 1000.0) as f32,
        });
        lio.push_frame(t0, points);
        pending.push_back((raw, truth));
        for out in lio.step() {
            let (raw, truth) = pending.pop_front().unwrap();
            let mut rays = Vec::new();
            frame_rays(
                &raw,
                out.lidar_beg_time,
                out.timeline.as_ref(),
                &out.state,
                &mut rays,
            );
            let errors = if out.lidar_end_time < STILL {
                &mut still
            } else {
                &mut moving
            };
            errors.add(&out, &rays, &truth);
            kinds.push(out.kind);
        }
    }

    assert_eq!(kinds.len(), FRAMES, "every frame is processed");
    assert_eq!(
        &kinds[..3],
        [FrameKind::FirstScan, FrameKind::ImuInit, FrameKind::MapInit]
    );
    assert!(
        kinds[3..].iter().all(|k| *k == FrameKind::Updated),
        "{kinds:?}"
    );
    still.report("still");
    let (median, max) = moving.report("moving");

    // Rays use the configured mounting from the first frame, before the IMU
    // initialises; with the state's identity extrinsic they would be 180° and
    // 70 mm off. The run gives 9 mm and 0.11°.
    assert!(
        still.pose_rot.to_degrees() < 0.25,
        "still: end pose {:.3}° off",
        still.pose_rot.to_degrees()
    );
    assert!(
        still.pose_pos < 0.02,
        "still: end pose {:.1} mm off",
        still.pose_pos * 1e3
    );
    // Within a frame, rays follow FAST-LIO's IMU timeline. What that misses
    // of the true motion (the velocity and bias estimates) stays at about
    // 2 mm and 0.02°.
    for e in [&still, &moving] {
        assert!(
            e.rel_origin < 5e-3,
            "ray origin {:.3} mm off within its frame",
            e.rel_origin * 1e3
        );
        assert!(
            e.rel_direction.to_degrees() < 0.05,
            "ray direction {:.4}° off",
            e.rel_direction.to_degrees()
        );
    }
    // Absolute accuracy, mostly the room's corner bias: the run gives 13 mm,
    // 0.12°, and end points with a 10 mm median and 24 mm at worst.
    assert!(
        moving.pose_pos < 0.025,
        "end pose {:.1} mm off",
        moving.pose_pos * 1e3
    );
    assert!(
        moving.pose_rot.to_degrees() < 0.25,
        "end pose {:.3}° off",
        moving.pose_rot.to_degrees()
    );
    assert!(
        median < 0.02,
        "median ray end point {:.1} mm off",
        median * 1e3
    );
    assert!(max < 0.05, "worst ray end point {:.1} mm off", max * 1e3);
}
