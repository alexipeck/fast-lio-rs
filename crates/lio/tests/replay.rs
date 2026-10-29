//! Replay is deterministic: the same capture gives bit-identical rays and
//! states, run after run and whatever the thread count. Needs the first real
//! capture, which stays local, so it is ignored by default:
//!
//! ```text
//! cargo test --release -p lio --test replay -- --ignored --nocapture
//! ```

use std::fs::File;
use std::hash::{DefaultHasher, Hasher};
use std::io::BufReader;

use jt128::pcap::PcapReader;
use jt128::{AngleCorrection, DecoderConfig, Firetimes};
use lio::{FrameResult, Pipeline};
use lio_core::lio::LioConfig;

const ROOT: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../..");
const FRAMES: usize = 150;

/// Hash of every ray and every frame's state over the first [`FRAMES`] frames.
fn replay_hash() -> (u64, usize) {
    let read = |name: &str| std::fs::read_to_string(format!("{ROOT}/data/{name}")).unwrap();
    let angles = AngleCorrection::parse(&read("angle_correction.csv")).unwrap();
    let fire = Firetimes::parse(&read("firetimes.csv")).unwrap();
    let cfg = LioConfig {
        extrinsic_r: [-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0],
        blind: 1.0,
        filter_size_surf: 0.25,
        filter_size_map: 0.25,
        ..LioConfig::default()
    };
    let mut pipeline = Pipeline::new(&angles, Some(&fire), DecoderConfig::default(), cfg);
    let file = File::open(format!("{ROOT}/data/room-2.pcap")).unwrap();
    let mut pcap = PcapReader::new(BufReader::new(file)).unwrap();
    let (mut out, mut frames, mut rays) = (Vec::<FrameResult>::new(), 0, 0);
    let mut h = DefaultHasher::new();
    while frames < FRAMES {
        let Some(p) = pcap.next_udp().unwrap() else {
            break;
        };
        pipeline.push_packet(p.payload, &mut out).unwrap();
        for f in out.drain(..) {
            let s = &f.output.state;
            for v in s
                .pos
                .to_row_vec()
                .into_iter()
                .chain([s.rot.w, s.rot.x, s.rot.y, s.rot.z])
            {
                h.write_u64(v.to_bits());
            }
            h.write_usize(f.output.effective_features);
            for r in &f.rays {
                h.write_u64(r.t.to_bits());
                for v in r
                    .origin
                    .iter()
                    .chain(&r.direction)
                    .chain([&r.range, &r.intensity])
                {
                    h.write_u32(v.to_bits());
                }
            }
            rays += f.rays.len();
            frames += 1;
        }
    }
    assert_eq!(frames, FRAMES);
    (h.finish(), rays)
}

#[test]
#[ignore = "needs data/room-2.pcap and its calibration files"]
fn replay_is_bit_identical_across_runs_and_thread_counts() {
    let (first, rays) = replay_hash();
    let (again, _) = replay_hash();
    let one_thread = rayon::ThreadPoolBuilder::new()
        .num_threads(1)
        .build()
        .unwrap();
    let (single, _) = one_thread.install(replay_hash);
    eprintln!(
        "{FRAMES} frames, {rays} rays: hashes {first:016x}, {again:016x}, one thread {single:016x}"
    );
    assert_eq!(first, again, "two runs differ");
    assert_eq!(first, single, "one thread differs from many");
}
