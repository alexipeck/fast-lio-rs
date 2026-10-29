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
use jt128::{AngleCorrection, Decoder, DecoderConfig, Event, Firetimes};

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
    assert!(
        (1036..=1040).contains(&frames.len()),
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
