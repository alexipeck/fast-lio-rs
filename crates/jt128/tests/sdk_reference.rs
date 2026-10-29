//! Compares the decoder with Hesai SDK 2.0 on the first real capture. The
//! reference CSVs come from `fixtures/hesai` (see `fixtures/README.md`); like the
//! capture they stay local, so this test is ignored by default:
//!
//! ```text
//! cargo test --release -p jt128 --test sdk_reference -- --ignored --nocapture
//! ```

use std::collections::BTreeMap;
use std::fs::File;
use std::io::BufReader;

use jt128::pcap::PcapReader;
use jt128::{AngleCorrection, Decoder, DecoderConfig, Event, Firetimes, Frame, ImuSample};

const ROOT: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../..");

fn decode() -> (Vec<Frame>, Vec<ImuSample>) {
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
    let (mut events, mut frames, mut imu) = (Vec::new(), Vec::new(), Vec::new());
    while let Some(p) = pcap.next_udp().unwrap() {
        dec.push(p.payload, &mut events).unwrap();
    }
    dec.flush(&mut events);
    for e in events {
        match e {
            Event::Frame(f) => frames.push(f),
            Event::Imu(s) => imu.push(s),
        }
    }
    (frames, imu)
}

#[test]
#[ignore = "needs data/room-2.pcap and the SDK reference CSVs in data/"]
fn points_and_imu_match_the_sdk() {
    let (frames, imu) = decode();

    // SDK points of each dumped frame, zero-range placeholders removed:
    // (x, y, z, time in ns).
    let mut sdk: BTreeMap<u32, Vec<(f32, f32, f32, i64)>> = BTreeMap::new();
    let text = std::fs::read_to_string(format!("{ROOT}/data/sdk_points.csv")).unwrap();
    for line in text.lines().skip(1) {
        let f: Vec<&str> = line.split(',').collect();
        let (x, y, z): (f32, f32, f32) = (
            f[2].parse().unwrap(),
            f[3].parse().unwrap(),
            f[4].parse().unwrap(),
        );
        if x == 0.0 && y == 0.0 && z == 0.0 {
            continue;
        }
        let t = f[7].parse::<i64>().unwrap() * 1_000_000_000 + f[8].parse::<i64>().unwrap();
        sdk.entry(f[0].parse().unwrap())
            .or_default()
            .push((x, y, z, t));
    }

    for (frame_no, want) in &sdk {
        let t_first = want.iter().map(|p| p.3).min().unwrap();
        let ours = frames
            .iter()
            .find(|f| (f.t0 * 1e9 - t_first as f64).abs() < 1e3)
            .unwrap_or_else(|| {
                panic!("no decoded frame starts at the SDK frame {frame_no}'s first point")
            });
        assert_eq!(
            ours.points.len(),
            want.len(),
            "SDK frame {frame_no}: point count"
        );
        // t0 is f64 seconds since the epoch, good to ~240 ns at this magnitude,
        // so times are compared relative to the frame start.
        let (mut max_xyz, mut max_t) = (0.0f32, 0i64);
        for (p, w) in ours.points.iter().zip(want) {
            max_xyz = max_xyz
                .max((p.x - w.0).abs())
                .max((p.y - w.1).abs())
                .max((p.z - w.2).abs());
            let dt = (f64::from(p.dt_ms) * 1e6).round() as i64;
            max_t = max_t.max((dt - (w.3 - t_first)).abs());
        }
        eprintln!(
            "SDK frame {frame_no}: {} points; max coordinate difference {max_xyz:e} m, max time difference {max_t} ns",
            want.len()
        );
        assert_eq!(max_xyz, 0.0, "coordinates should match the SDK exactly");
        // dt_ms is f32 milliseconds within a 100 ms frame: 8 ns resolution at worst.
        assert!(max_t <= 8, "point times differ by up to {max_t} ns");
    }

    // IMU: the SDK's samples are ours minus the ones split back out of merged
    // runs, matched by value in order. Two kinds of SDK sample carry the next
    // packet's time instead of the first one carrying the value: the capture's
    // first (the SDK spends that packet detecting the lidar type), and those
    // whose value arrives in the packet that starts a new frame. Frame t0 is
    // that packet's time plus its earliest block offset, about -1.998 ms.
    let text = std::fs::read_to_string(format!("{ROOT}/data/sdk_imu.csv")).unwrap();
    let sdk_imu: Vec<Vec<f64>> = text
        .lines()
        .skip(1)
        .map(|l| l.split(',').map(|v| v.parse().unwrap()).collect())
        .collect();
    let starts: Vec<f64> = frames.iter().map(|f| f.t0).collect();
    let (mut i, mut matched, mut late) = (0, 0, 0);
    for (n, s) in sdk_imu.iter().enumerate() {
        let same = |o: &ImuSample| {
            (0..3).all(|k| {
                (o.acc[k] - s[1 + k]).abs() < 1e-8
                    && (o.gyro[k].to_degrees() - s[4 + k]).abs() < 1e-6
            })
        };
        while i < imu.len() && !(same(&imu[i]) && imu[i].t < s[0] + 1e-6) {
            i += 1;
        }
        assert!(i < imu.len(), "SDK sample {n} at {} has no match", s[0]);
        let o = &imu[i];
        let dt = s[0] - o.t;
        if dt > 1e-6 {
            assert!(
                dt < 2.3e-4,
                "SDK sample {n} is {dt} s after ours, more than one packet"
            );
            let starts_frame = starts.iter().any(|t| (o.t - t - 1.998e-3).abs() < 5e-6);
            assert!(
                n == 0 || starts_frame,
                "SDK sample {n} is late but not on a frame's first packet"
            );
            late += 1;
        }
        matched += 1;
        i += 1;
    }
    eprintln!(
        "IMU: {late} SDK samples one packet later than ours, all on the first packet or a frame's first packet"
    );
    eprintln!(
        "IMU: all {matched} SDK samples match ours ({} ours in total)",
        imu.len()
    );
    assert_eq!(matched, sdk_imu.len());
}
