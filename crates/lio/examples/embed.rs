//! `lio` inside another program: UDP payloads in, one ray per returned point
//! out. Reads a recording here; a live program passes each datagram from its
//! socket to `push_packet` the same way. See docs/integration.md.
//!
//! ```text
//! cargo run --release -p lio --example embed -- data/room-2.pcap data/angle_correction.csv data/firetimes.csv
//! ```

use std::fs::File;
use std::io::BufReader;

use jt128::packet::PacketError;
use jt128::pcap::PcapReader;
use jt128::{AngleCorrection, DecoderConfig, Firetimes};
use lio::{FrameResult, Pipeline, lidar_pose};
use lio_core::lio::FrameKind;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let [capture, angles, firetimes] = args.as_slice() else {
        return Err("usage: embed <capture.pcap> <angles.csv> <firetimes.csv>".into());
    };
    let angles = AngleCorrection::parse(&std::fs::read(angles)?)?;
    if angles.hash_matches == Some(false) {
        eprintln!("warning: the angle file's SHA-256 does not match");
    }
    let firetimes = Firetimes::parse(&std::fs::read(firetimes)?)?;
    let mut pipeline = Pipeline::new(
        &angles,
        Some(&firetimes),
        DecoderConfig::default(),
        lio::jt128_config(),
    );

    let mut pcap = PcapReader::new(BufReader::new(File::open(capture)?))?;
    let mut done = Vec::new();
    while let Some(packet) = pcap.next_udp()? {
        match pipeline.push_packet(packet.payload, &mut done) {
            // Fault messages share the port; they carry no points.
            Ok(()) | Err(PacketError::NotPointCloud) => {}
            Err(e) => return Err(e.into()),
        }
        for frame in done.drain(..) {
            consume(&frame);
        }
    }
    pipeline.finish(&mut done);
    done.iter().for_each(consume);
    Ok(())
}

/// Stands in for the downstream stage.
fn consume(frame: &FrameResult) {
    let o = &frame.output;
    if o.kind != FrameKind::Updated {
        println!(
            "{:.3} {:?}: {} rays",
            o.lidar_end_time,
            o.kind,
            frame.rays.len()
        );
        return;
    }
    let (position, _) = lidar_pose(o);
    let nearest = frame
        .rays
        .iter()
        .map(|r| r.range)
        .fold(f32::INFINITY, f32::min);
    let r = &frame.rays[0];
    let point: Vec<f32> = (0..3)
        .map(|i| r.origin[i] + r.range * r.direction[i])
        .collect();
    println!(
        "{:.3} {} rays, {} matched; LiDAR at {:.3?}; nearest return {nearest:.2} m; first point {point:.3?}",
        o.lidar_end_time,
        frame.rays.len(),
        o.effective_features,
        position
    );
}
