//! `lio`: runs FAST-LIO on a JT128 recording or live stream and writes
//! per-point rays and the corrected path.
//!
//! It only ever listens. It never sends anything to the sensor, and live
//! mode refuses to start without `--max-seconds`.

use std::fs::File;
use std::io::{BufReader, BufWriter, Write};
use std::net::UdpSocket;
use std::process::ExitCode;
use std::time::{Duration, Instant};

use jt128::pcap::PcapReader;
use jt128::{AngleCorrection, DecoderConfig, Firetimes};
use lio::{FrameResult, Pipeline, RayWriter, lidar_pose, write_pcd};
use lio_core::lio::{FrameKind, LioConfig};

const USAGE: &str = "\
usage:
  lio replay <capture.pcap> --angles <csv> [--firetimes <csv>] [options]
  lio live --angles <csv> --max-seconds <s> [--bind <addr:port>] [--firetimes <csv>] [options]

outputs:
  --rays <file>          binary ray stream (see lio::RayWriter)
  --ray-stride <n>       keep every n-th ray in --rays (default 1: all)
  --path <file>          corrected LiDAR path at each frame end, TUM format
  --frames-csv <file>    per-frame diagnostics
  --map <file.pcd>       final map, ASCII PCD
  --dump-input <file>    record FAST-LIO's exact input for the C++ reference run

FAST-LIO settings (defaults: FAST-LIO's Mid-360 configuration):
  --extrinsic-r <9 comma-separated>  LiDAR-to-IMU rotation, row-major (default identity)
  --extrinsic-t <3 comma-separated>  LiDAR position in the IMU frame, m (default 0)
  --extrinsic-est                    estimate the extrinsic online
  --time-offset <s>                  time_offset_lidar_to_imu (default 0)
  --max-iteration <n> --filter-size-surf <m> --filter-size-map <m>
  --point-filter-num <n> --blind <m> --det-range <m> --cube-side-length <m>
  --gyr-cov <v> --acc-cov <v> --b-gyr-cov <v> --b-acc-cov <v>
  --max-frames <n>                   stop after n frames
";

struct Args {
    mode: String,
    capture: Option<String>,
    angles: String,
    firetimes: Option<String>,
    max_seconds: Option<f64>,
    bind: String,
    rays: Option<String>,
    ray_stride: usize,
    path: Option<String>,
    frames_csv: Option<String>,
    map: Option<String>,
    dump_input: Option<String>,
    max_frames: Option<usize>,
    lio: LioConfig,
}

fn floats<const N: usize>(s: &str) -> Result<[f64; N], String> {
    let v: Vec<f64> = s
        .split(',')
        .map(|x| x.trim().parse::<f64>())
        .collect::<Result<_, _>>()
        .map_err(|e| e.to_string())?;
    v.try_into()
        .map_err(|_| format!("expected {N} comma-separated numbers, got {s:?}"))
}

fn parse_args() -> Result<Args, String> {
    let mut it = std::env::args().skip(1);
    let mode = it.next().ok_or("missing mode")?;
    let mut a = Args {
        mode: mode.clone(),
        capture: None,
        angles: String::new(),
        firetimes: None,
        max_seconds: None,
        bind: "0.0.0.0:2368".into(),
        rays: None,
        ray_stride: 1,
        path: None,
        frames_csv: None,
        map: None,
        dump_input: None,
        max_frames: None,
        lio: LioConfig::default(),
    };
    if mode == "replay" {
        a.capture = Some(it.next().ok_or("replay needs a capture file")?);
    } else if mode != "live" {
        return Err(format!("unknown mode {mode:?}"));
    }
    while let Some(flag) = it.next() {
        if flag == "--extrinsic-est" {
            a.lio.extrinsic_est_en = true;
            continue;
        }
        let v = it.next().ok_or_else(|| format!("{flag} needs a value"))?;
        let num = || v.parse::<f64>().map_err(|e| format!("{flag}: {e}"));
        match flag.as_str() {
            "--angles" => a.angles = v,
            "--firetimes" => a.firetimes = Some(v),
            "--max-seconds" => a.max_seconds = Some(num()?),
            "--bind" => a.bind = v,
            "--rays" => a.rays = Some(v),
            "--ray-stride" => a.ray_stride = num()? as usize,
            "--path" => a.path = Some(v),
            "--frames-csv" => a.frames_csv = Some(v),
            "--map" => a.map = Some(v),
            "--dump-input" => a.dump_input = Some(v),
            "--max-frames" => a.max_frames = Some(num()? as usize),
            "--extrinsic-r" => a.lio.extrinsic_r = floats::<9>(&v)?,
            "--extrinsic-t" => a.lio.extrinsic_t = floats::<3>(&v)?,
            "--time-offset" => a.lio.time_offset_lidar_to_imu = num()?,
            "--max-iteration" => a.lio.max_iteration = num()? as usize,
            "--filter-size-surf" => a.lio.filter_size_surf = num()?,
            "--filter-size-map" => a.lio.filter_size_map = num()?,
            "--point-filter-num" => a.lio.point_filter_num = num()? as usize,
            "--blind" => a.lio.blind = num()?,
            "--det-range" => a.lio.det_range = num()? as f32,
            "--cube-side-length" => a.lio.cube_side_length = num()?,
            "--gyr-cov" => a.lio.gyr_cov = num()?,
            "--acc-cov" => a.lio.acc_cov = num()?,
            "--b-gyr-cov" => a.lio.b_gyr_cov = num()?,
            "--b-acc-cov" => a.lio.b_acc_cov = num()?,
            _ => return Err(format!("unknown option {flag}")),
        }
    }
    if a.angles.is_empty() {
        return Err("--angles is required (the unit's angle-correction CSV)".into());
    }
    if a.mode == "live" && a.max_seconds.is_none() {
        return Err("live mode needs --max-seconds: the sensor must not be left running".into());
    }
    Ok(a)
}

struct Outputs {
    rays: Option<RayWriter<BufWriter<File>>>,
    path: Option<BufWriter<File>>,
    frames: Option<BufWriter<File>>,
    counts: [usize; 5],
    rays_total: usize,
    first_pos: Option<[f64; 3]>,
    last_pos: [f64; 3],
    max_excursion: f64,
    path_length: f64,
}

impl Outputs {
    fn open(a: &Args) -> std::io::Result<Self> {
        let create = |p: &Option<String>| {
            p.as_ref()
                .map(|p| File::create(p).map(BufWriter::new))
                .transpose()
        };
        let rays = match &a.rays {
            Some(p) => Some(RayWriter::new(
                BufWriter::new(File::create(p)?),
                a.ray_stride,
            )?),
            None => None,
        };
        let mut frames = create(&a.frames_csv)?;
        if let Some(f) = frames.as_mut() {
            writeln!(
                f,
                "frame,kind,beg,end,downsampled,effective,map_points,x,y,z,qw,qx,qy,qz,vx,vy,vz,bgx,bgy,bgz,bax,bay,baz,proc_ms"
            )?;
        }
        Ok(Self {
            rays,
            path: create(&a.path)?,
            frames,
            counts: [0; 5],
            rays_total: 0,
            first_pos: None,
            last_pos: [0.0; 3],
            max_excursion: 0.0,
            path_length: 0.0,
        })
    }

    fn record(&mut self, index: usize, r: &FrameResult, proc_ms: f64) -> std::io::Result<()> {
        let o = &r.output;
        self.counts[lio::kind_code(o.kind) as usize] += 1;
        self.rays_total += r.rays.len();
        if let Some(w) = self.rays.as_mut() {
            w.write_frame(r)?;
        }
        let (p, q) = lidar_pose(o);
        if matches!(o.kind, FrameKind::Updated | FrameKind::MapInit) {
            if let Some(w) = self.path.as_mut() {
                writeln!(
                    w,
                    "{:.6} {} {} {} {} {} {} {}",
                    o.lidar_end_time, p[0], p[1], p[2], q[1], q[2], q[3], q[0]
                )?;
            }
            let first = *self.first_pos.get_or_insert(p);
            let d = |a: [f64; 3], b: [f64; 3]| {
                ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
            };
            self.max_excursion = self.max_excursion.max(d(p, first));
            if o.kind == FrameKind::Updated {
                self.path_length += d(p, self.last_pos);
            }
            self.last_pos = p;
        }
        if let Some(w) = self.frames.as_mut() {
            let s = &o.state;
            writeln!(
                w,
                "{index},{:?},{:.6},{:.6},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{:.3}",
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
                proc_ms
            )?;
        }
        Ok(())
    }

    fn finish(mut self) -> std::io::Result<()> {
        if let Some(w) = self.rays.take() {
            w.into_inner().flush()?;
        }
        for w in [self.path.as_mut(), self.frames.as_mut()]
            .into_iter()
            .flatten()
        {
            w.flush()?;
        }
        Ok(())
    }
}

fn run(a: Args) -> Result<(), String> {
    let angles = AngleCorrection::parse(
        &std::fs::read_to_string(&a.angles).map_err(|e| format!("{}: {e}", a.angles))?,
    )
    .map_err(|e| e.to_string())?;
    let firetimes = match &a.firetimes {
        Some(p) => Some(
            Firetimes::parse(&std::fs::read_to_string(p).map_err(|e| format!("{p}: {e}"))?)
                .map_err(|e| e.to_string())?,
        ),
        None => None,
    };
    let mut pipe = Pipeline::new(
        &angles,
        firetimes.as_ref(),
        DecoderConfig::default(),
        a.lio.clone(),
    );
    if let Some(p) = &a.dump_input {
        pipe.dump_input_to(Box::new(BufWriter::new(
            File::create(p).map_err(|e| format!("{p}: {e}"))?,
        )));
    }
    let mut outputs = Outputs::open(&a).map_err(|e| e.to_string())?;
    let mut results = Vec::new();
    let (mut frame_index, mut busy) = (0usize, Duration::ZERO);
    let wall = Instant::now();
    let mut sensor_span: Option<(f64, f64)> = None;

    let mut handle = |pipe: &mut Pipeline, payload: Option<&[u8]>| -> Result<bool, String> {
        let t = Instant::now();
        match payload {
            Some(p) => {
                if let Err(e) = pipe.push_packet(p, &mut results) {
                    // Non-point datagrams (fault messages) share the port; skip them.
                    if !matches!(e, jt128::packet::PacketError::NotPointCloud) {
                        return Err(e.to_string());
                    }
                }
            }
            None => pipe.finish(&mut results),
        }
        let dt = t.elapsed();
        busy += dt;
        let per_frame = if results.is_empty() {
            0.0
        } else {
            dt.as_secs_f64() * 1e3 / results.len() as f64
        };
        for r in results.drain(..) {
            let span =
                sensor_span.get_or_insert((r.output.lidar_beg_time, r.output.lidar_end_time));
            span.1 = r.output.lidar_end_time;
            outputs
                .record(frame_index, &r, per_frame)
                .map_err(|e| e.to_string())?;
            frame_index += 1;
        }
        Ok(a.max_frames.is_some_and(|m| frame_index >= m))
    };

    match a.mode.as_str() {
        "replay" => {
            let path = a.capture.as_ref().unwrap();
            let file = File::open(path).map_err(|e| format!("{path}: {e}"))?;
            let mut reader = PcapReader::new(BufReader::with_capacity(1 << 20, file))
                .map_err(|e| e.to_string())?;
            while let Some(pkt) = reader.next_udp().map_err(|e| e.to_string())? {
                if handle(&mut pipe, Some(pkt.payload))? {
                    break;
                }
            }
        }
        _ => {
            let limit = Duration::from_secs_f64(a.max_seconds.unwrap());
            let socket = UdpSocket::bind(&a.bind).map_err(|e| format!("bind {}: {e}", a.bind))?;
            socket
                .set_read_timeout(Some(Duration::from_millis(200)))
                .map_err(|e| e.to_string())?;
            let mut buf = vec![0u8; 65_536];
            eprintln!(
                "listening on {} for at most {:.1} s (receive only)",
                a.bind,
                limit.as_secs_f64()
            );
            while wall.elapsed() < limit {
                match socket.recv_from(&mut buf) {
                    Ok((n, _)) => {
                        if handle(&mut pipe, Some(&buf[..n]))? {
                            break;
                        }
                    }
                    Err(e)
                        if matches!(
                            e.kind(),
                            std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                        ) => {}
                    Err(e) => return Err(e.to_string()),
                }
            }
        }
    }
    handle(&mut pipe, None)?;

    if let Some(p) = &a.map {
        let points = pipe.lio_mut().tree.flatten();
        let mut w = BufWriter::new(File::create(p).map_err(|e| format!("{p}: {e}"))?);
        write_pcd(&mut w, &points).map_err(|e| e.to_string())?;
    }
    let s = pipe.decoder_stats().clone();
    let c = outputs.counts;
    let span = sensor_span.map_or(0.0, |(b, e)| e - b);
    eprintln!(
        "decoded {} packets ({} spin-up, {} sequence gaps), {} IMU samples ({} recovered)",
        s.packets, s.spinup_packets, s.sequence_gaps, s.imu_samples, s.imu_samples_split
    );
    eprintln!(
        "frames: {frame_index} (first {}, IMU init {}, map init {}, updated {}, skipped {}); rays: {}",
        c[0], c[1], c[2], c[3], c[4], outputs.rays_total
    );
    eprintln!(
        "processing: {:.2} s for {:.2} s of data ({:.1}x real time); map holds {} points",
        busy.as_secs_f64(),
        span,
        span / busy.as_secs_f64().max(1e-9),
        pipe.lio().tree.valid_num()
    );
    eprintln!(
        "path: {:.2} m travelled, furthest {:.2} m from start, ends at [{:.3}, {:.3}, {:.3}]",
        outputs.path_length,
        outputs.max_excursion,
        outputs.last_pos[0],
        outputs.last_pos[1],
        outputs.last_pos[2]
    );
    outputs.finish().map_err(|e| e.to_string())
}

fn main() -> ExitCode {
    match parse_args() {
        Ok(a) => match run(a) {
            Ok(()) => ExitCode::SUCCESS,
            Err(e) => {
                eprintln!("lio: {e}");
                ExitCode::FAILURE
            }
        },
        Err(e) => {
            eprintln!("lio: {e}\n\n{USAGE}");
            ExitCode::from(2)
        }
    }
}
