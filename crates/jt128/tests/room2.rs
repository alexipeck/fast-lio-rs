//! Decodes the first real capture end to end. The capture is the user's own
//! recording and is not in the repository (`data/` is git-ignored), so this
//! test is ignored by default:
//!
//! ```text
//! cargo test --release -p jt128 --test room2 -- --ignored --nocapture
//! ```

use std::fs::File;
use std::io::BufReader;

use jt128::pcap::PcapReader;
use jt128::{AngleCorrection, Decoder, DecoderConfig, DecoderStats, Event, Firetimes};

const ROOT: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../..");

#[test]
#[ignore = "needs data/room-2.pcap and the calibration files in data/"]
fn room2_decodes_like_the_inspector_says() {
    let angles = AngleCorrection::parse(
        &std::fs::read(format!("{ROOT}/data/angle_correction.csv")).unwrap(),
    )
    .unwrap();
    let fire =
        Firetimes::parse(&std::fs::read(format!("{ROOT}/data/firetimes.csv")).unwrap()).unwrap();
    let mut dec = Decoder::new(&angles, Some(&fire), DecoderConfig::default());
    let mut pcap = PcapReader::new(BufReader::new(
        File::open(format!("{ROOT}/data/room-2.pcap")).unwrap(),
    ))
    .unwrap();
    let mut events = Vec::new();
    let (mut frames, mut imu, mut points) = (Vec::new(), Vec::new(), 0usize);
    while let Some(pkt) = pcap.next_udp().unwrap() {
        dec.push(pkt.payload, &mut events).unwrap();
        for e in events.drain(..) {
            match e {
                Event::Frame(f) => {
                    points += f.points.len();
                    frames.push((
                        f.t0,
                        f.points.len(),
                        f.points.iter().map(|p| p.dt_ms).fold(0.0f32, f32::max),
                    ));
                }
                Event::Imu(s) => imu.push(s),
            }
        }
    }
    dec.flush(&mut events);
    let s = &dec.stats;
    eprintln!("{s:?}; {points} points in {} frames", frames.len());
    assert_eq!(s.packets, 466_361);
    assert_eq!(s.sequence_gaps, 0);
    assert_eq!(s.spinup_packets, 66);
    // 0.5 s of packets after the spin-up, at about 4,480 a second.
    assert_eq!(s.wake_packets, 2242);
    assert!(
        (1030..=1034).contains(&frames.len()),
        "frames: {}",
        frames.len()
    );
    assert_eq!(s.imu_samples_split, 11);
    assert_eq!(imu.len() as u64, s.imu_samples);
    assert_eq!(s.imu_samples, 20_784 + 11);
    // Full frames (not the first or last) span one rotation.
    let full: Vec<_> = frames[1..frames.len() - 1].to_vec();
    let periods: Vec<f64> = full.windows(2).map(|w| w[1].0 - w[0].0).collect();
    let mean_period = periods.iter().sum::<f64>() / periods.len() as f64;
    eprintln!(
        "mean frame period {:.3} ms; mean frame span {:.3} ms",
        mean_period * 1e3,
        full.iter().map(|f| f64::from(f.2)).sum::<f64>() / full.len() as f64
    );
    assert!((mean_period - 0.10032).abs() < 0.0005);
    // IMU: times increase, the stationary start reads about 1 g straight up.
    assert!(imu.windows(2).all(|w| w[1].t > w[0].t));
    let still: Vec<_> = imu.iter().take(1000).collect();
    let mean = |i: usize| still.iter().map(|s| s.acc[i]).sum::<f64>() / still.len() as f64;
    let (ax, ay, az) = (mean(0), mean(1), mean(2));
    eprintln!("stationary mean acc [{ax:.4}, {ay:.4}, {az:.4}] g");
    assert!((az - 0.992).abs() < 0.01 && ax.abs() < 0.05 && ay.abs() < 0.05);
    let gmax = still
        .iter()
        .flat_map(|s| s.gyro)
        .fold(0.0f64, |m, v| m.max(v.abs()));
    assert!(
        gmax < 0.05,
        "stationary gyro should be well under 0.05 rad/s, got {gmax}"
    );
}

/// The capture's first `n` payloads.
fn first_payloads(n: usize) -> Vec<Vec<u8>> {
    let mut pcap = PcapReader::new(BufReader::new(
        File::open(format!("{ROOT}/data/room-2.pcap")).unwrap(),
    ))
    .unwrap();
    let mut out = Vec::with_capacity(n);
    while out.len() < n {
        out.push(pcap.next_udp().unwrap().unwrap().payload.to_vec());
    }
    out
}

/// Decodes `payloads` and returns the frames: their longest point time span
/// (ms) and how many points lie more than 2.3 m above the sensor, where the
/// ceiling is 1.7 m.
fn decode_frames(payloads: &[Vec<u8>], cfg: DecoderConfig) -> (DecoderStats, Vec<(f32, usize)>) {
    let angles = AngleCorrection::parse(
        &std::fs::read(format!("{ROOT}/data/angle_correction.csv")).unwrap(),
    )
    .unwrap();
    let fire =
        Firetimes::parse(&std::fs::read(format!("{ROOT}/data/firetimes.csv")).unwrap()).unwrap();
    let mut dec = Decoder::new(&angles, Some(&fire), cfg);
    let mut events = Vec::new();
    for p in payloads {
        dec.push(p, &mut events).unwrap();
    }
    dec.flush(&mut events);
    let frames = events
        .into_iter()
        .filter_map(|e| match e {
            Event::Frame(f) => Some((
                f.points.iter().map(|p| p.dt_ms).fold(0.0f32, f32::max),
                f.points.iter().filter(|p| p.z > 2.3).count(),
            )),
            Event::Imu(_) => None,
        })
        .collect();
    (dec.stats.clone(), frames)
}

#[test]
#[ignore = "needs data/room-2.pcap and the calibration files in data/"]
fn room2_wake_transient_is_held() {
    // About 5.8 s, all of it with the sensor still on the desk.
    let payloads = first_payloads(26_000);
    let above = |f: &[(f32, usize)]| f.iter().map(|f| f.1).sum::<usize>();

    let kept = DecoderConfig {
        wake_hold: 0.0,
        ..DecoderConfig::default()
    };
    let (s, frames) = decode_frames(&payloads, kept);
    eprintln!("no hold: {s:?}; {} points above 2.3 m", above(&frames));
    assert_eq!(s.wake_packets, 0);
    assert!(above(&frames[..3]) > 10_000);

    let (s, frames) = decode_frames(&payloads, DecoderConfig::default());
    eprintln!("held: {s:?}; {} points above 2.3 m", above(&frames));
    assert_eq!((s.spinup_packets, s.wake_packets), (66, 2242));
    // 2 stray points over 5 s, against 108,901 without the hold.
    assert!(above(&frames) < 10);

    // A wake in the middle of a stream: the same start fed again (its packet
    // times go back, as a stream's would not). The frame in progress ends at
    // the second spin-up and the second hold drops the transient again.
    let twice: Vec<_> = payloads[..13_000]
        .iter()
        .chain(&payloads[..13_000])
        .cloned()
        .collect();
    let (s, frames) = decode_frames(&twice, DecoderConfig::default());
    let longest = frames.iter().map(|f| f.0).fold(0.0f32, f32::max);
    eprintln!(
        "woken twice: {s:?}; {} points above 2.3 m; longest frame {longest:.1} ms",
        above(&frames)
    );
    assert_eq!((s.spinup_packets, s.wake_packets), (132, 2 * 2242));
    assert!(above(&frames) < 10);
    assert!(longest < 101.0, "a frame spans {longest} ms");
}
