//! Runs `lio-core` on an input recorded by `lio replay --dump-input`, the
//! Rust twin of `fixtures/src/run_fastlio.cpp`: same arguments, same per-frame
//! CSV. It exists to compare the two on identical input, results and speed.
//!
//! `proc_ms` is the wall time from the event that completes a frame to its
//! row: the push, `sync_packages` and the frame's processing. Reading the
//! input is not counted, as in the C++.
//!
//! ```text
//! cargo run --release -p lio --example run_input -- <input.bin> <frames.csv> [key=value ...]
//! ```
//!
//! Keys: max_iteration filter_size_surf filter_size_map cube_side_length
//! det_range gyr_cov acc_cov b_gyr_cov b_acc_cov extrinsic_est_en
//! extrinsic_R (9 comma-separated) extrinsic_T (3) time_offset. Defaults are
//! FAST-LIO's Mid-360 settings, as in the C++ harness.

use std::fs::File;
use std::io::{BufReader, BufWriter, Read, Write};
use std::process::ExitCode;
use std::time::Instant;

use lio::lidar_pose;
use lio_core::imu_init::ImuMeasurement;
use lio_core::lio::{Lio, LioConfig};
use lio_core::mat::Vec3;
use lio_core::point::Point;

fn config(args: &[String]) -> Result<LioConfig, String> {
    let mut c = LioConfig::default();
    for a in args {
        let (k, v) = a
            .split_once('=')
            .ok_or_else(|| format!("{a}: expected key=value"))?;
        let nums = || -> Result<Vec<f64>, String> {
            v.split(',')
                .map(|x| x.parse::<f64>().map_err(|e| format!("{k}: {e}")))
                .collect()
        };
        let num = || -> Result<f64, String> { Ok(nums()?[0]) };
        match k {
            "max_iteration" => c.max_iteration = num()? as usize,
            "filter_size_surf" => c.filter_size_surf = num()?,
            "filter_size_map" => c.filter_size_map = num()?,
            "cube_side_length" => c.cube_side_length = num()?,
            "det_range" => c.det_range = num()? as f32,
            "gyr_cov" => c.gyr_cov = num()?,
            "acc_cov" => c.acc_cov = num()?,
            "b_gyr_cov" => c.b_gyr_cov = num()?,
            "b_acc_cov" => c.b_acc_cov = num()?,
            "extrinsic_est_en" => c.extrinsic_est_en = num()? != 0.0,
            "time_offset" => c.time_offset_lidar_to_imu = num()?,
            "extrinsic_T" => {
                c.extrinsic_t = nums()?
                    .try_into()
                    .map_err(|_| "extrinsic_T needs 3 values")?;
            }
            "extrinsic_R" => {
                c.extrinsic_r = nums()?
                    .try_into()
                    .map_err(|_| "extrinsic_R needs 9 values")?;
            }
            _ => return Err(format!("unknown key {k}")),
        }
    }
    Ok(c)
}

fn f64s<const N: usize>(r: &mut impl Read) -> std::io::Result<[f64; N]> {
    let mut b = [0u8; 8];
    let mut v = [0.0; N];
    for x in &mut v {
        r.read_exact(&mut b)?;
        *x = f64::from_le_bytes(b);
    }
    Ok(v)
}

fn run(args: &[String]) -> Result<(), String> {
    let [input, csv, rest @ ..] = args else {
        return Err("usage: run_input <input.bin> <frames.csv> [key=value ...]".into());
    };
    let mut lio = Lio::new(config(rest)?);
    let mut r = BufReader::with_capacity(
        1 << 20,
        File::open(input).map_err(|e| format!("{input}: {e}"))?,
    );
    let mut out = BufWriter::new(File::create(csv).map_err(|e| format!("{csv}: {e}"))?);
    let io = |e: std::io::Error| e.to_string();
    writeln!(
        out,
        "frame,kind,beg,end,downsampled,effective,map_points,x,y,z,qw,qx,qy,qz,vx,vy,vz,bgx,bgy,bgz,bax,bay,baz,proc_ms"
    )
    .map_err(io)?;

    let (mut index, mut total_ms) = (0usize, 0.0);
    let mut tag = [0u8; 1];
    while r.read(&mut tag).map_err(io)? == 1 {
        let start = match tag[0] {
            1 => {
                let v = f64s::<7>(&mut r).map_err(io)?;
                let start = Instant::now();
                lio.push_imu(ImuMeasurement {
                    t: v[0],
                    acc: Vec3::from_array([v[1], v[2], v[3]]),
                    gyro: Vec3::from_array([v[4], v[5], v[6]]),
                });
                start
            }
            2 => {
                let [beg] = f64s::<1>(&mut r).map_err(io)?;
                let mut b4 = [0u8; 4];
                r.read_exact(&mut b4).map_err(io)?;
                let n = u32::from_le_bytes(b4) as usize;
                let mut raw = vec![0u8; n * 20];
                r.read_exact(&mut raw).map_err(io)?;
                let f = |c: &[u8]| f32::from_le_bytes(c.try_into().unwrap());
                let points = raw
                    .chunks_exact(20)
                    .map(|c| Point {
                        x: f(&c[0..4]),
                        y: f(&c[4..8]),
                        z: f(&c[8..12]),
                        intensity: f(&c[12..16]),
                        curvature: f(&c[16..20]),
                    })
                    .collect();
                let start = Instant::now();
                lio.push_preprocessed_frame(beg, points);
                start
            }
            t => return Err(format!("bad tag {t}")),
        };
        let frames = lio.step();
        let ms = start.elapsed().as_secs_f64() * 1e3 / frames.len().max(1) as f64;
        for o in &frames {
            total_ms += ms;
            let (p, q) = lidar_pose(o);
            let s = &o.state;
            writeln!(
                out,
                "{index},{:?},{:.6},{:.6},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{ms:.3}",
                o.kind,
                o.lidar_beg_time,
                o.lidar_end_time,
                o.downsampled_points,
                o.effective_features,
                o.map_points,
                p[0],
                p[1],
                p[2],
                q[0],
                q[1],
                q[2],
                q[3],
                s.vel[0],
                s.vel[1],
                s.vel[2],
                s.bg[0],
                s.bg[1],
                s.bg[2],
                s.ba[0],
                s.ba[1],
                s.ba[2],
            )
            .map_err(io)?;
            index += 1;
        }
    }
    out.flush().map_err(io)?;
    eprintln!("frames: {index}, processing {total_ms:.1} ms");
    Ok(())
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match run(&args) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("run_input: {e}");
            ExitCode::FAILURE
        }
    }
}
