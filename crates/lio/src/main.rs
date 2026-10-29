//! `lio`: runs FAST-LIO on a JT128 recording or live stream and writes
//! per-point rays and the corrected path.
//!
//! It only ever listens. It never sends anything to the sensor, and live
//! mode refuses to start without `--max-seconds`.

use std::fs::File;
use std::io::{BufReader, BufWriter, Write};
use std::net::UdpSocket;
use std::process::ExitCode;
use std::sync::mpsc::{Receiver, SyncSender, sync_channel};
use std::time::{Duration, Instant};

use jt128::packet::PacketError;
use jt128::pcap::PcapReader;
use jt128::{AngleCorrection, Decoder, DecoderConfig, DecoderStats, Event, Firetimes};
use lio::{Estimator, FrameResult, Processed, RayWriter, lidar_pose, write_pcd};
use lio_core::lio::{FrameKind, LioConfig};

mod cpus;

/// Options built in at compile time from `LIO_DEFAULT_ARGS` (whitespace-
/// separated). They are read before the command line, which overrides them.
const BUILT_IN_ARGS: &str = match option_env!("LIO_DEFAULT_ARGS") {
    Some(s) => s,
    None => "",
};

/// Packets' worth of decoder events the decoding thread may run ahead of
/// FAST-LIO. A frame is about 450 packets, and only packets that finish a
/// frame or carry a new IMU sample produce events.
const EVENT_QUEUE: usize = 256;
/// Finished frames FAST-LIO may run ahead of the ray and output stage.
const FRAME_QUEUE: usize = 4;

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

FAST-LIO settings (defaults: FAST-LIO's Mid-360 configuration, except the
four JT128 values given here):
  --extrinsic-r <9 comma-separated>  LiDAR-to-IMU rotation, row-major
                                     (default -1,0,0,0,-1,0,0,0,1: the JT128's
                                     IMU x and y point opposite the LiDAR's)
  --blind <m>                        drop returns closer than this (default 1.0,
                                     which keeps a handheld operator out)
  --filter-size-surf <m> --filter-size-map <m>   voxel sizes (default 0.25)
  --extrinsic-t <3 comma-separated>  LiDAR position in the IMU frame, m (default 0)
  --extrinsic-est                    estimate the extrinsic online
  --time-offset <s>                  time_offset_lidar_to_imu (default 0)
  --max-iteration <n> --point-filter-num <n> --det-range <m> --cube-side-length <m>
  --gyr-cov <v> --acc-cov <v> --b-gyr-cov <v> --b-acc-cov <v>
  --max-frames <n>                   stop after n frames

threads and CPUs:
  lio runs three stages on their own threads, plus workers for the parallel
  parts. The stages:
    decode   receives and decodes the packets
    lio      FAST-LIO's serial steps
    output   collects the rays and writes the outputs
    workers  FAST-LIO's matching and motion compensation, and the rays
  --threads <n>          number of workers. Default: one per CPU given to
                         --pin workers, else the fastest cores where Linux
                         reports core capacities (ARM big.LITTLE boards),
                         else one per CPU
  --cpus <list>          run only on these CPUs, such as 4-7 (like taskset)
  --pin <stage>=<list>   run one stage only on these CPUs, such as lio=4 or
                         workers=5-7. Workers get one CPU each when there are
                         as many as CPUs listed. Repeat for each stage; an
                         unpinned stage runs on any CPU --cpus allows. A list
                         of `any` removes a restriction built in by default.
                         --cpus and --pin need Linux and are ignored elsewhere.

Any option can be given a default when lio is built, in LIO_DEFAULT_ARGS:
  LIO_DEFAULT_ARGS='--cpus 2-5 --pin workers=3-5' cargo build --release
The command line overrides these defaults.
";

/// FAST-LIO's Mid-360 settings with the JT128's values where they differ,
/// measured on the first capture (docs/plan.md, 2026-10-29).
fn jt128_defaults() -> LioConfig {
    LioConfig {
        extrinsic_r: [-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0],
        blind: 1.0,
        filter_size_surf: 0.25,
        filter_size_map: 0.25,
        ..LioConfig::default()
    }
}

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
    threads: Option<usize>,
    placement: Placement,
    lio: LioConfig,
}

/// A CPU list, or `any` for no restriction (to undo a built-in default).
fn cpu_option(list: &str) -> Result<Option<Vec<usize>>, String> {
    if list == "any" {
        return Ok(None);
    }
    cpus::parse_list(list).map(Some)
}

/// Where `lio`'s threads may run (`--cpus`, `--pin`).
#[derive(Clone, Default)]
struct Placement {
    process: Option<Vec<usize>>,
    decode: Option<Vec<usize>>,
    lio: Option<Vec<usize>>,
    output: Option<Vec<usize>>,
    workers: Option<Vec<usize>>,
}

impl Placement {
    fn stages(&self) -> [(&'static str, &Option<Vec<usize>>); 4] {
        [
            ("decoding", &self.decode),
            ("FAST-LIO", &self.lio),
            ("output", &self.output),
            ("workers", &self.workers),
        ]
    }

    fn is_set(&self) -> bool {
        self.process.is_some() || self.stages().iter().any(|(_, c)| c.is_some())
    }

    /// `--pin <stage>=<list>`.
    fn pin(&mut self, spec: &str) -> Result<(), String> {
        let (stage, list) = spec
            .split_once('=')
            .ok_or_else(|| format!("--pin {spec:?}: expected <stage>=<CPUs>"))?;
        let cpus = cpu_option(list)?;
        match stage {
            "decode" => self.decode = cpus,
            "lio" => self.lio = cpus,
            "output" => self.output = cpus,
            "workers" => self.workers = cpus,
            _ => {
                return Err(format!(
                    "--pin: unknown stage {stage:?} (decode, lio, output or workers)"
                ));
            }
        }
        Ok(())
    }
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
        threads: None,
        placement: Placement::default(),
        lio: jt128_defaults(),
    };
    if mode == "replay" {
        a.capture = Some(it.next().ok_or("replay needs a capture file")?);
    } else if mode != "live" {
        return Err(format!("unknown mode {mode:?}"));
    }
    let mut flags = BUILT_IN_ARGS.split_whitespace().map(String::from).chain(it);
    while let Some(flag) = flags.next() {
        if flag == "--extrinsic-est" {
            a.lio.extrinsic_est_en = true;
            continue;
        }
        let v = flags
            .next()
            .ok_or_else(|| format!("{flag} needs a value"))?;
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
            "--threads" => a.threads = Some((num()? as usize).max(1)),
            "--cpus" => a.placement.process = cpu_option(&v)?,
            "--pin" => a.placement.pin(&v)?,
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
    frame_count: usize,
    /// Sensor time from the first frame's start to the last frame's end.
    span: Option<(f64, f64)>,
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
            frame_count: 0,
            span: None,
            rays_total: 0,
            first_pos: None,
            last_pos: [0.0; 3],
            max_excursion: 0.0,
            path_length: 0.0,
        })
    }

    fn record(&mut self, r: &FrameResult, proc_ms: f64) -> std::io::Result<()> {
        let o = &r.output;
        let index = self.frame_count;
        self.frame_count += 1;
        self.span
            .get_or_insert((o.lidar_beg_time, o.lidar_end_time))
            .1 = o.lidar_end_time;
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

/// The decoding thread's end of the pipeline: payloads in, events sent on.
struct Source<'a> {
    decoder: &'a mut Decoder,
    events: Vec<Event>,
    tx: SyncSender<Vec<Event>>,
    busy: Duration,
}

impl Source<'_> {
    /// Decodes one payload, or flushes the frame in progress at the end, and
    /// sends the events on. Returns false once the estimator has stopped.
    fn push(&mut self, payload: Option<&[u8]>) -> Result<bool, String> {
        let t = Instant::now();
        match payload {
            Some(p) => {
                if let Err(e) = self.decoder.push(p, &mut self.events) {
                    // Non-point datagrams (fault messages) share the port; skip them.
                    if !matches!(e, PacketError::NotPointCloud) {
                        return Err(e.to_string());
                    }
                }
            }
            None => self.decoder.flush(&mut self.events),
        }
        self.busy += t.elapsed();
        Ok(self.events.is_empty() || self.tx.send(std::mem::take(&mut self.events)).is_ok())
    }
}

/// Reads the capture or the socket and decodes every packet. Runs on its own
/// thread; returns the time spent decoding.
fn decode(a: &Args, decoder: &mut Decoder, tx: SyncSender<Vec<Event>>) -> Result<Duration, String> {
    let mut src = Source {
        decoder,
        events: Vec::new(),
        tx,
        busy: Duration::ZERO,
    };
    match a.mode.as_str() {
        "replay" => {
            let path = a.capture.as_ref().unwrap();
            let file = File::open(path).map_err(|e| format!("{path}: {e}"))?;
            let mut reader = PcapReader::new(BufReader::with_capacity(1 << 20, file))
                .map_err(|e| e.to_string())?;
            while let Some(pkt) = reader.next_udp().map_err(|e| e.to_string())? {
                if !src.push(Some(pkt.payload))? {
                    return Ok(src.busy);
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
            let start = Instant::now();
            while start.elapsed() < limit {
                match socket.recv_from(&mut buf) {
                    Ok((n, _)) => {
                        if !src.push(Some(&buf[..n]))? {
                            return Ok(src.busy);
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
    src.push(None)?;
    Ok(src.busy)
}

/// FAST-LIO on the decoder's events, in order, on its own thread. Each
/// finished frame goes on with the milliseconds FAST-LIO spent on it.
fn estimate(
    estimator: &mut Estimator,
    events: Receiver<Vec<Event>>,
    frames: SyncSender<(Processed, f64)>,
) {
    let mut done = Vec::new();
    for event in events.into_iter().flatten() {
        let t = Instant::now();
        estimator.process(event, &mut done);
        let ms = t.elapsed().as_secs_f64() * 1e3 / done.len().max(1) as f64;
        for p in done.drain(..) {
            if frames.send((p, ms)).is_err() {
                return; // the output stage has stopped
            }
        }
    }
}

/// The rays and every output, on the calling thread, until FAST-LIO is done
/// or `--max-frames` is reached. Returns the time spent on rays and on
/// writing.
fn write_outputs(
    a: &Args,
    outputs: &mut Outputs,
    frames: Receiver<(Processed, f64)>,
) -> Result<(Duration, Duration), String> {
    let (mut rays, mut writing) = (Duration::ZERO, Duration::ZERO);
    for (p, lio_ms) in frames {
        let t = Instant::now();
        let r = p.rays();
        let dt = t.elapsed();
        rays += dt;
        let t = Instant::now();
        outputs
            .record(&r, lio_ms + dt.as_secs_f64() * 1e3)
            .map_err(|e| e.to_string())?;
        writing += t.elapsed();
        if a.max_frames.is_some_and(|m| outputs.frame_count >= m) {
            break;
        }
    }
    Ok((rays, writing))
}

/// Applies `--cpus`, checks the `--pin` lists and starts the workers.
/// Returns the placement left for the stage threads to apply.
fn place_threads(a: &Args) -> Result<Placement, String> {
    let mut placement = a.placement.clone();
    if placement.is_set() && !cpus::CAN_PIN {
        eprintln!("lio: pinning threads needs Linux; --cpus and --pin are ignored");
        placement = Placement::default();
    }
    // Every thread started from here on inherits this.
    if let Some(c) = &placement.process {
        cpus::pin(c)?;
    }
    let allowed = cpus::allowed();
    for (stage, list) in placement.stages() {
        if let Some(c) = list.iter().flatten().find(|c| !allowed.contains(c)) {
            return Err(format!(
                "--pin {stage}: CPU {c} is not available to lio (it may use {})",
                cpus::format_list(&allowed)
            ));
        }
    }

    // Slow cores of a big.LITTLE chip hold up FAST-LIO's short parallel loops
    // while idle workers spin, so by default only the fast cores get workers.
    // Zero leaves rayon's own default: RAYON_NUM_THREADS, else one per CPU.
    let workers = a
        .threads
        .or(placement.workers.as_ref().map(Vec::len))
        .or(cpus::fastest(&allowed).map(|f| f.len()))
        .unwrap_or(0);
    let mut pool = rayon::ThreadPoolBuilder::new()
        .num_threads(workers)
        .thread_name(|i| format!("lio-worker-{i}"));
    if let Some(list) = placement.workers.clone() {
        let one_each = list.len() == workers;
        pool = pool.start_handler(move |i| {
            let cpus = if one_each { &list[i..=i] } else { &list[..] };
            if let Err(e) = cpus::pin(cpus) {
                eprintln!("lio: worker {i}: {e}");
            }
        });
    }
    pool.build_global().map_err(|e| e.to_string())?;

    if placement.is_set() {
        let mut parts: Vec<String> = placement
            .stages()
            .iter()
            .filter_map(|(stage, c)| {
                c.as_ref()
                    .map(|c| format!("{stage} on CPUs {}", cpus::format_list(c)))
            })
            .collect();
        if let Some(c) = &placement.process {
            parts.push(format!("the rest on CPUs {}", cpus::format_list(c)));
        }
        eprintln!(
            "placement: {}; {} workers",
            parts.join(", "),
            rayon::current_num_threads()
        );
    }
    Ok(placement)
}

fn run(a: Args) -> Result<(), String> {
    if !BUILT_IN_ARGS.is_empty() {
        eprintln!("built-in options: {BUILT_IN_ARGS}");
    }
    let placement = place_threads(&a)?;
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
    let mut decoder = Decoder::new(&angles, firetimes.as_ref(), DecoderConfig::default());
    let mut estimator = Estimator::new(a.lio.clone());
    if let Some(p) = &a.dump_input {
        estimator.dump_input_to(Box::new(BufWriter::new(
            File::create(p).map_err(|e| format!("{p}: {e}"))?,
        )));
    }
    let mut outputs = Outputs::open(&a).map_err(|e| e.to_string())?;

    // Three stages, each on its own thread, joined by bounded queues: decoding,
    // FAST-LIO, then rays and outputs. Each stage sees the same items in the
    // same order as with `Pipeline`, so the results are the same.
    let wall = Instant::now();
    let (event_tx, event_rx) = sync_channel(EVENT_QUEUE);
    let (frame_tx, frame_rx) = sync_channel(FRAME_QUEUE);
    let pin = |c: &Option<Vec<usize>>| c.as_ref().map_or(Ok(()), |c| cpus::pin(c));
    let (decoding, estimated, written) = std::thread::scope(|scope| {
        // Named so they can be told apart in top -H, ps -L or htop.
        let thread = |name: &str| std::thread::Builder::new().name(name.into());
        let decoding = thread("lio-decode")
            .spawn_scoped(scope, || {
                pin(&placement.decode)?;
                decode(&a, &mut decoder, event_tx)
            })
            .expect("starting the decoder thread");
        let estimating = thread("lio-fastlio")
            .spawn_scoped(scope, || {
                pin(&placement.lio)?;
                estimate(&mut estimator, event_rx, frame_tx);
                Ok::<(), String>(())
            })
            .expect("starting the FAST-LIO thread");
        // This thread takes the output stage. Returning drops `frame_rx`; the
        // upstream stages then stop early if they are still running.
        let written = (|| {
            pin(&placement.output)?;
            write_outputs(&a, &mut outputs, frame_rx)
        })();
        (
            decoding.join().expect("decoder thread panicked"),
            estimating.join().expect("FAST-LIO thread panicked"),
            written,
        )
    });
    estimated?;
    let (decoding, (rays_busy, writing)) = (decoding?, written?);
    let wall = wall.elapsed();

    if let Some(p) = &a.map {
        let points = estimator.lio_mut().tree.flatten();
        let mut w = BufWriter::new(File::create(p).map_err(|e| format!("{p}: {e}"))?);
        write_pcd(&mut w, &points).map_err(|e| e.to_string())?;
    }
    let s: &DecoderStats = &decoder.stats;
    let c = outputs.counts;
    eprintln!(
        "decoded {} packets ({} spin-up, {} sequence gaps), {} IMU samples ({} recovered)",
        s.packets, s.spinup_packets, s.sequence_gaps, s.imu_samples, s.imu_samples_split
    );
    eprintln!(
        "frames: {} (first {}, IMU init {}, map init {}, updated {}, skipped {}); rays: {}",
        outputs.frame_count, c[0], c[1], c[2], c[3], c[4], outputs.rays_total
    );
    let span = outputs.span.map_or(0.0, |(b, e)| e - b);
    let lio_busy = estimator.busy();
    let secs = |d: Duration| d.as_secs_f64();
    if a.mode == "replay" {
        eprintln!(
            "processing: {:.2} s for {:.2} s of data ({:.1}x real time); map holds {} points",
            secs(wall),
            span,
            span / secs(wall).max(1e-9),
            estimator.lio().tree.valid_num()
        );
    } else {
        // Live, the wall time is the listening time; what matters is how much
        // of it the busiest stage needed.
        let busiest = decoding.max(lio_busy).max(rays_busy + writing);
        eprintln!(
            "processing: {:.2} s of data in {:.2} s of listening; the busiest stage kept up with {:.1}x headroom; map holds {} points",
            span,
            secs(wall),
            span / secs(busiest).max(1e-9),
            estimator.lio().tree.valid_num()
        );
    }
    eprintln!(
        "  busy: decoding {:.2} s, FAST-LIO {:.2} s, rays {:.2} s and writing {:.2} s, on three threads with {} workers",
        secs(decoding),
        secs(lio_busy),
        secs(rays_busy),
        secs(writing),
        rayon::current_num_threads()
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
            if !BUILT_IN_ARGS.is_empty() {
                eprintln!("This build's defaults: LIO_DEFAULT_ARGS='{BUILT_IN_ARGS}'");
            }
            ExitCode::from(2)
        }
    }
}
