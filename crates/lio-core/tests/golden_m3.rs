//! Milestone 3 golden tests: lio-core against FAST-LIO's own C++ on the same
//! inputs (`fixtures/golden/m3.json`, produced by `fixtures/src/gen_m3.cpp`).
//!
//! Tolerances are relative to `max(1, |expected|)`. They allow for libm and
//! summation-order differences between glibc/Eigen and Rust, nothing more.

use lio_core::esekf::{Cov, Esekf};
use lio_core::imu_init::{ImuInit, ImuMeasurement};
use lio_core::mat::{Mat, Mat3, Vec2, Vec3};
use lio_core::process::{self, Input, ProcessNoise};
use lio_core::s2::S2;
use lio_core::so3::{self, Quat};
use lio_core::state::{ErrorState, FlatState, State};
use serde_json::Value;

fn golden() -> Value {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/golden/m3.json");
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

fn quat_vec(q: &Quat) -> Vec<f64> {
    vec![q.w, q.x, q.y, q.z]
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
    out.extend(quat_vec(&s.rot));
    out.extend(quat_vec(&s.offset_r_l_i));
    for v in [s.offset_t_l_i, s.vel, s.bg, s.ba, s.grav.vec] {
        out.extend(v.to_row_vec());
    }
    out
}

/// Panics if any element differs by more than `tol·max(1, |want|)`.
fn check(what: &str, got: &[f64], want: &[f64], tol: f64) {
    assert_eq!(got.len(), want.len(), "{what}: length");
    for (i, (g, w)) in got.iter().zip(want).enumerate() {
        let err = (g - w).abs() / w.abs().max(1.0);
        assert!(
            err <= tol,
            "{what}[{i}]: got {g:e}, want {w:e} (rel err {err:e} > {tol:e})"
        );
    }
}

#[test]
fn cos_sinc_sqrt() {
    for c in golden()["cos_sinc"].as_array().unwrap() {
        let (cos, sinc) = so3::cos_sinc_sqrt(c["x2"].as_f64().unwrap());
        check(
            "cos_sinc",
            &[cos, sinc],
            &[c["cos"].as_f64().unwrap(), c["sinc"].as_f64().unwrap()],
            1e-15,
        );
    }
}

#[test]
fn so3_exp_log_and_a_matrix() {
    let g = golden();
    for c in g["so3_exp"].as_array().unwrap() {
        let q = so3::exp(&mat(&c["v"]), c["scale"].as_f64().unwrap());
        check("so3_exp", &quat_vec(&q), &nums(&c["q"]), 1e-15);
    }
    for c in g["so3_log"].as_array().unwrap() {
        let v = so3::log(&quat(&c["q"]));
        check("so3_log", &v.to_row_vec(), &nums(&c["v"]), 1e-14);
    }
    for c in g["a_matrix"].as_array().unwrap() {
        let m = so3::a_matrix(&mat(&c["v"]));
        check("a_matrix", &m.to_row_vec(), &nums(&c["m"]), 1e-12);
    }
}

#[test]
fn s2_operations() {
    for c in golden()["s2"].as_array().unwrap() {
        let g = S2 {
            vec: mat(&c["vec"]),
        };
        let delta: Vec2 = mat(&c["delta"]);
        check("s2.bx", &g.bx().to_row_vec(), &nums(&c["bx"]), 1e-15);
        check("s2.mx", &g.mx(&delta).to_row_vec(), &nums(&c["mx"]), 1e-14);
        check(
            "s2.mx_zero",
            &g.mx(&Vec2::zeros()).to_row_vec(),
            &nums(&c["mx_zero"]),
            1e-15,
        );
        check(
            "s2.nx_yy",
            &g.nx_yy().to_row_vec(),
            &nums(&c["nx_yy"]),
            1e-15,
        );
        check("s2.hat", &g.hat().to_row_vec(), &nums(&c["hat"]), 0.0);
        let mut plus = g;
        plus.boxplus(&delta, 1.0);
        check(
            "s2.boxplus",
            &plus.vec.to_row_vec(),
            &nums(&c["boxplus"]),
            1e-15,
        );
        for b in c["boxminus"].as_array().unwrap() {
            let other = S2 {
                vec: mat(&b["other"]),
            };
            check(
                "s2.boxminus",
                &g.boxminus(&other).to_row_vec(),
                &nums(&b["res"]),
                1e-14,
            );
        }
    }
}

#[test]
fn state_boxplus_boxminus_oplus() {
    for c in golden()["state_ops"].as_array().unwrap() {
        let x = state(&c["x"]);
        let dx: ErrorState = mat(&c["dx"]);
        let mut plus = x;
        plus.boxplus(&dx);
        check(
            "state.boxplus",
            &state_vec(&plus),
            &state_vec(&state(&c["boxplus"])),
            1e-14,
        );

        let minus = x.boxminus(&state(&c["y"]));
        check(
            "state.boxminus",
            &minus.to_row_vec(),
            &nums(&c["boxminus"]),
            1e-13,
        );

        let f: FlatState = mat(&c["f"]);
        let mut op = x;
        op.oplus(&f, c["dt"].as_f64().unwrap());
        check(
            "state.oplus",
            &state_vec(&op),
            &state_vec(&state(&c["oplus"])),
            1e-14,
        );
    }
}

#[test]
fn process_model() {
    for c in golden()["process"].as_array().unwrap() {
        let x = state(&c["x"]);
        let input = Input {
            acc: mat(&c["acc"]),
            gyro: mat(&c["gyro"]),
        };
        check(
            "get_f",
            &process::get_f(&x, &input).to_row_vec(),
            &nums(&c["f"]),
            1e-14,
        );
        check(
            "df_dx",
            &process::df_dx(&x, &input).to_row_vec(),
            &nums(&c["df_dx"]),
            1e-14,
        );
        check(
            "df_dw",
            &process::df_dw(&x, &input).to_row_vec(),
            &nums(&c["df_dw"]),
            1e-15,
        );
    }
}

#[test]
fn predict_sequence() {
    let g = &golden()["predict"];
    let mut kf = Esekf {
        x: state(&g["x0"]),
        p: mat(&g["p0"]),
    };
    let q: ProcessNoise = mat(&g["q"]);
    for (i, step) in g["steps"].as_array().unwrap().iter().enumerate() {
        let input = Input {
            acc: mat(&step["acc"]),
            gyro: mat(&step["gyro"]),
        };
        kf.predict(step["dt"].as_f64().unwrap(), &q, &input);
        check(
            &format!("predict[{i}].x"),
            &state_vec(&kf.x),
            &state_vec(&state(&step["x"])),
            1e-12,
        );
        check(
            &format!("predict[{i}].p"),
            &kf.p.to_row_vec(),
            &nums(&step["p"]),
            1e-12,
        );
    }
}

#[test]
fn imu_init_through_process() {
    let g = &golden()["imu_init"];
    let ext_r: Mat3 = mat(&g["ext_r"]);
    let ext_t: Vec3 = mat(&g["ext_t"]);
    let mut kf = Esekf::default();
    let mut init = ImuInit::default();
    for (i, frame) in g["frames"].as_array().unwrap().iter().enumerate() {
        let imu: Vec<ImuMeasurement> = frame["imu"]
            .as_array()
            .unwrap()
            .iter()
            .map(|m| ImuMeasurement {
                t: m["t"].as_f64().unwrap(),
                acc: mat(&m["acc"]),
                gyro: mat(&m["gyro"]),
            })
            .collect();
        let done = init.feed(
            &imu,
            frame["lidar_beg_time"].as_f64().unwrap(),
            &mut kf,
            &ext_t,
            &ext_r,
        );
        assert_eq!(done, frame["done"].as_bool().unwrap(), "imu_init[{i}].done");
        check(
            &format!("imu_init[{i}].x"),
            &state_vec(&kf.x),
            &state_vec(&state(&frame["x"])),
            1e-14,
        );
        let want: Cov = mat(&frame["p"]);
        check(
            &format!("imu_init[{i}].p"),
            &kf.p.to_row_vec(),
            &want.to_row_vec(),
            0.0,
        );
    }
}
