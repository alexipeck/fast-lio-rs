//! Milestone 4 golden tests: plane fit, voxel downsample and ikd-Tree against
//! the C++ FAST-LIO runs (`fixtures/golden/m4.json`, from `fixtures/src/gen_m4.cpp`).
//!
//! These algorithms run in `f32`, so tolerances are float-sized. Each test
//! prints its worst error, so a tolerance never hides how close the match is.

use lio_core::plane::esti_plane;
use lio_core::point::Point;
use lio_core::voxel;
use serde_json::Value;

fn golden() -> Value {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/golden/m4.json");
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

fn points(v: &Value) -> Vec<Point> {
    v.as_array()
        .expect("array")
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

fn point_vec(p: &Point) -> [f64; 5] {
    [p.x, p.y, p.z, p.intensity, p.curvature].map(f64::from)
}

/// Worst error relative to `max(1, |want|)`.
fn worst_err(got: &[f64], want: &[f64]) -> f64 {
    assert_eq!(got.len(), want.len());
    got.iter()
        .zip(want)
        .map(|(g, w)| (g - w).abs() / w.abs().max(1.0))
        .fold(0.0, f64::max)
}

/// FAST-LIO solves `A·n = −1` in `f32`. For a small patch far from the origin
/// on a plane that passes near it, the system is ill-conditioned. There the C++
/// float result itself is off by up to ~4e-4 from the double-precision plane;
/// Rust and C++ land at different points in that band (case 19: true
/// d = 0.872159, C++ 0.871897, Rust 0.871777). So each case gets a
/// conditioning-sized tolerance, and the median must sit at float rounding,
/// which catches any systematic error.
#[test]
fn plane_fit() {
    let mut errs = Vec::new();
    for (i, c) in golden()["plane"].as_array().unwrap().iter().enumerate() {
        let pts = points(&c["points"]);
        let got = esti_plane(&pts, 0.1);
        let ok = c["ok"].as_bool().unwrap();
        assert_eq!(got.is_some(), ok, "plane[{i}]: accept decision");
        if let Some(abcd) = got {
            let err = worst_err(&abcd.map(f64::from), &nums(&c["abcd"]));
            assert!(
                err < 3e-4,
                "plane[{i}]: {abcd:?} vs {:?} (err {err:e})",
                nums(&c["abcd"])
            );
            errs.push(err);
        }
    }
    errs.sort_by(f64::total_cmp);
    let median = errs[errs.len() / 2];
    eprintln!(
        "plane_fit: {} accepted, median rel err {median:e}, worst {:e}",
        errs.len(),
        errs[errs.len() - 1]
    );
    assert!(median < 5e-6, "plane_fit median relative error {median:e}");
}

#[test]
fn voxel_downsample() {
    for (i, c) in golden()["voxel"].as_array().unwrap().iter().enumerate() {
        let input = points(&c["input"]);
        let want = points(&c["output"]);
        let got = voxel::downsample(&input, c["leaf"].as_f64().unwrap() as f32);
        assert_eq!(got.len(), want.len(), "voxel[{i}]: point count");
        let worst = got
            .iter()
            .zip(&want)
            .map(|(g, w)| worst_err(&point_vec(g), &point_vec(w)))
            .fold(0.0, f64::max);
        assert!(worst < 1e-5, "voxel[{i}]: worst relative error {worst:e}");
        eprintln!(
            "voxel[{i}] ({} -> {} points) worst relative error: {worst:e}",
            input.len(),
            got.len()
        );
    }
}
