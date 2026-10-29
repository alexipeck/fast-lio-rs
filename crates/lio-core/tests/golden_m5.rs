//! Milestone 5 golden tests: motion compensation through `ImuProcess` and the
//! iterated update, against FAST-LIO's C++ (`fixtures/golden/m5.json`, from
//! `fixtures/src/gen_m5.cpp`).

use lio_core::esekf::{Cov, DynShare, Esekf};
use lio_core::imu_init::ImuMeasurement;
use lio_core::imu_process::{ImuProcess, MeasureGroup};
use lio_core::mat::{Mat, Mat3, Vec3};
use lio_core::point::Point;
use lio_core::s2::S2;
use lio_core::so3::Quat;
use lio_core::state::State;
use serde_json::Value;

fn golden() -> Value {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/golden/m5.json");
    let text =
        std::fs::read_to_string(path).expect("golden file missing; run fixtures/generate.sh");
    serde_json::from_str(&text).expect("golden file is not valid JSON")
}

fn nums(v: &Value) -> Vec<f64> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|x| x.as_f64().expect("number"))
        .collect()
}

fn mat<const R: usize, const C: usize>(v: &Value) -> Mat<R, C> {
    Mat::from_row_slice(&nums(v))
}

fn quat(v: &Value) -> Quat {
    let n = nums(v);
    Quat::new(n[0], n[1], n[2], n[3])
}

fn state(v: &Value) -> State {
    State {
        pos: mat(&v["pos"]),
        rot: quat(&v["rot"]),
        offset_r_l_i: quat(&v["off_r"]),
        offset_t_l_i: mat(&v["off_t"]),
        vel: mat(&v["vel"]),
        bg: mat(&v["bg"]),
        ba: mat(&v["ba"]),
        grav: S2 {
            vec: mat(&v["grav"]),
        },
    }
}

fn state_vec(s: &State) -> Vec<f64> {
    let mut out = s.pos.to_row_vec();
    for q in [s.rot, s.offset_r_l_i] {
        out.extend([q.w, q.x, q.y, q.z]);
    }
    for v in [s.offset_t_l_i, s.vel, s.bg, s.ba, s.grav.vec] {
        out.extend(v.to_row_vec());
    }
    out
}

fn cloud(v: &Value) -> Vec<Point> {
    v.as_array()
        .unwrap()
        .iter()
        .map(|p| {
            let n = nums(p);
            Point {
                x: n[0] as f32,
                y: n[1] as f32,
                z: n[2] as f32,
                intensity: n[3] as f32,
                curvature: n[4] as f32,
            }
        })
        .collect()
}

fn check(what: &str, got: &[f64], want: &[f64], tol: f64) -> f64 {
    assert_eq!(got.len(), want.len(), "{what}: length");
    let mut worst = 0.0f64;
    for (i, (g, w)) in got.iter().zip(want).enumerate() {
        let err = (g - w).abs() / w.abs().max(1.0);
        assert!(
            err <= tol,
            "{what}[{i}]: got {g:e}, want {w:e} (rel err {err:e} > {tol:e})"
        );
        worst = worst.max(err);
    }
    worst
}

#[test]
fn undistortion_through_imu_process() {
    let g = &golden()["undistort"];
    let ext_r: Mat3 = mat(&g["ext_r"]);
    let ext_t: Vec3 = mat(&g["ext_t"]);
    let mut imu = ImuProcess::new(ext_t, ext_r, 0.1, 0.1, 1e-4, 1e-4);
    let mut kf = Esekf::default();
    let (mut worst_pts, mut worst_state) = (0.0f64, 0.0f64);
    for (i, f) in g["frames"].as_array().unwrap().iter().enumerate() {
        let meas = MeasureGroup {
            lidar_beg_time: f["lidar_beg_time"].as_f64().unwrap(),
            lidar_end_time: f["lidar_end_time"].as_f64().unwrap(),
            lidar: cloud(&f["cloud"]),
            imu: f["imu"]
                .as_array()
                .unwrap()
                .iter()
                .map(|m| ImuMeasurement {
                    t: m["t"].as_f64().unwrap(),
                    acc: mat(&m["acc"]),
                    gyro: mat(&m["gyro"]),
                })
                .collect(),
        };
        let want = cloud(&f["undistorted"]);
        match imu.process(&meas, &mut kf) {
            None => assert!(
                want.is_empty(),
                "frame {i}: C++ undistorted while Rust was initialising"
            ),
            Some((pts, _)) => {
                assert_eq!(pts.len(), want.len(), "frame {i}: point count");
                let flat = |c: &[Point]| {
                    c.iter()
                        .flat_map(|p| [p.x, p.y, p.z, p.curvature].map(f64::from))
                        .collect::<Vec<_>>()
                };
                worst_pts = worst_pts.max(check(
                    &format!("frame {i} points"),
                    &flat(&pts),
                    &flat(&want),
                    1e-6,
                ));
            }
        }
        worst_state = worst_state.max(check(
            &format!("frame {i} x"),
            &state_vec(&kf.x),
            &state_vec(&state(&f["x"])),
            1e-11,
        ));
        let p: Cov = mat(&f["p"]);
        worst_state = worst_state.max(check(
            &format!("frame {i} P"),
            &kf.p.to_row_vec(),
            &p.to_row_vec(),
            1e-11,
        ));
    }
    eprintln!("undistortion: worst point error {worst_pts:e}, worst state/P error {worst_state:e}");
}

fn update_case(name: &str) {
    let g = &golden()[name];
    let body: Vec<Vec3> = g["body"].as_array().unwrap().iter().map(mat).collect();
    let normal: Vec<Vec3> = g["normal"].as_array().unwrap().iter().map(mat).collect();
    let d = nums(&g["d"]);
    let mut kf = Esekf {
        x: state(&g["x0"]),
        p: mat(&g["p0"]),
    };
    kf.update_iterated_dyn_share_modified(
        0.001,
        3,
        &[0.001; 23],
        |s: &State, out: &mut DynShare| {
            out.h_x.clear();
            out.h.clear();
            for ((be, n), di) in body.iter().zip(&normal).zip(&d) {
                let point_this = s.offset_r_l_i.rotate(be) + s.offset_t_l_i;
                let world = s.rot.rotate(&point_this) + s.pos;
                let c = s.rot.conjugate().rotate(n);
                let a = point_this.hat() * c;
                let b = be.hat() * s.offset_r_l_i.conjugate().to_matrix() * c;
                out.h_x.push([
                    n[0], n[1], n[2], a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2],
                ]);
                out.h.push(-(n.dot(&world) + di));
            }
        },
    );
    let wx = check(
        &format!("{name} x"),
        &state_vec(&kf.x),
        &state_vec(&state(&g["x"])),
        1e-9,
    );
    let want_p: Cov = mat(&g["p"]);
    let wp = check(
        &format!("{name} P"),
        &kf.p.to_row_vec(),
        &want_p.to_row_vec(),
        1e-9,
    );
    eprintln!("{name}: worst state error {wx:e}, worst P error {wp:e}");
}

#[test]
fn iterated_update_many_measurements() {
    update_case("update_many");
}

#[test]
fn iterated_update_fewer_measurements_than_states() {
    update_case("update_few");
}
