//! Milestone 4 golden tests: plane fit, voxel downsample and ikd-Tree against
//! the C++ FAST-LIO runs (`fixtures/golden/m4.json`, from `fixtures/src/gen_m4.cpp`).
//!
//! These algorithms run in `f32`, so tolerances are float-sized. Each test
//! prints its worst error, so a tolerance never hides how close the match is.

use lio_core::ikd_tree::{BoxPoint, IkdTree};
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
    let mut accepted = 0;
    for (i, c) in golden()["plane"].as_array().unwrap().iter().enumerate() {
        let pts = points(&c["points"]);
        let got = esti_plane(&pts, 0.1);
        let ok = c["ok"].as_bool().unwrap();
        assert_eq!(got.is_some(), ok, "plane[{i}]: accept decision");
        if let Some(abcd) = got {
            // The sums follow Eigen's SIMD order, so the fit is bit-identical.
            assert_eq!(
                abcd.map(f64::from).to_vec(),
                nums(&c["abcd"]),
                "plane[{i}]: plane coefficients"
            );
            accepted += 1;
        }
    }
    eprintln!("plane_fit: {accepted} accepted, all bit-identical");
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

fn xyz_points(v: &Value) -> Vec<Point> {
    v.as_array()
        .expect("array")
        .iter()
        .map(|p| {
            let n = nums(p);
            Point::new(n[0] as f32, n[1] as f32, n[2] as f32)
        })
        .collect()
}

fn xyz(p: &Point) -> [f32; 3] {
    [p.x, p.y, p.z]
}

fn count(v: &Value) -> usize {
    v.as_f64().expect("number") as usize
}

/// Replays FAST-LIO's map operations and compares everything the algorithm can
/// observe: downsampled-insert counts, delete counts, live-point counts, every
/// 5-NN result (points and squared distances, exactly), and the final map.
/// Tree shape is free to differ.
#[test]
fn ikd_tree_operations() {
    let g = golden();
    let d = &g["ikd"];
    let mut tree = IkdTree::default();
    tree.set_downsample_param(d["downsample_size"].as_f64().unwrap() as f32);
    tree.build(xyz_points(&d["build"]));
    let (mut near, mut dist) = (Vec::new(), Vec::new());
    let mut queries = 0;
    for (r, round) in d["rounds"].as_array().unwrap().iter().enumerate() {
        let added = tree.add_points(&xyz_points(&round["add_ds"]), true);
        assert_eq!(
            added,
            count(&round["add_ds_count"]),
            "round {r}: downsampled inserts"
        );
        tree.add_points(&xyz_points(&round["add_plain"]), false);
        let boxes: Vec<BoxPoint> = round["delete_boxes"]
            .as_array()
            .unwrap()
            .iter()
            .map(|b| {
                let (lo, hi) = (nums(&b["min"]), nums(&b["max"]));
                BoxPoint {
                    min: [lo[0] as f32, lo[1] as f32, lo[2] as f32],
                    max: [hi[0] as f32, hi[1] as f32, hi[2] as f32],
                }
            })
            .collect();
        assert_eq!(
            tree.delete_point_boxes(&boxes),
            count(&round["delete_count"]),
            "round {r}: deleted"
        );
        assert_eq!(
            tree.valid_num(),
            count(&round["valid_num"]),
            "round {r}: live points"
        );

        for (i, q) in round["queries"].as_array().unwrap().iter().enumerate() {
            let p = nums(&q["p"]);
            let p = Point::new(p[0] as f32, p[1] as f32, p[2] as f32);
            let max_dist = q["max_dist"].as_f64().unwrap();
            let max_dist = if max_dist < 0.0 {
                f32::INFINITY
            } else {
                max_dist as f32
            };
            tree.nearest_search(&p, 5, max_dist, &mut near, &mut dist);
            let want_pts: Vec<[f32; 3]> = xyz_points(&q["near"]).iter().map(xyz).collect();
            let want_dist: Vec<f32> = nums(&q["dist"]).iter().map(|&x| x as f32).collect();
            assert_eq!(dist, want_dist, "round {r} query {i}: distances");
            assert_eq!(
                near.iter().map(xyz).collect::<Vec<_>>(),
                want_pts,
                "round {r} query {i}: points"
            );
            queries += 1;
        }
    }
    let mut got: Vec<[f32; 3]> = tree.flatten().iter().map(xyz).collect();
    got.sort_by(|a, b| {
        a.iter()
            .zip(b)
            .map(|(x, y)| x.total_cmp(y))
            .find(|o| o.is_ne())
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    let want: Vec<[f32; 3]> = xyz_points(&d["final_valid"]).iter().map(xyz).collect();
    assert_eq!(got, want, "final map contents");
    eprintln!(
        "ikd_tree: {queries} queries and the final map ({} points) match exactly",
        want.len()
    );
}
