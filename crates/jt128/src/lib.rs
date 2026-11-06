//! Decoder for the Hesai JT128: UDP protocol v1.4 packets in, LiDAR frames
//! and IMU samples out.
//!
//! The packet layout, time and geometry rules are specified in
//! `docs/hesai_jt128_decoder.md`. Every type here uses the sensor's own clock.

pub mod calib;
pub mod decoder;
pub mod packet;
pub mod pcap;

pub use calib::{AngleCorrection, Firetimes};
pub use decoder::{Decoder, DecoderConfig, DecoderStats, Event};

/// One IMU reading, with gyro already converted to rad/s.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuSample {
    /// Sensor time, seconds.
    pub t: f64,
    /// Specific force in g. FAST-LIO rescales by the startup mean, so g is accepted as is.
    pub acc: [f64; 3],
    /// Angular rate, rad/s.
    pub gyro: [f64; 3],
}

/// One LiDAR return in the sensor frame (Hesai axes: +Y at azimuth 0, +X at 90°, +Z up).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Point {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: f32,
    /// Time since [`Frame::t0`], milliseconds. FAST-LIO keeps this in the PCL `curvature` field.
    pub dt_ms: f32,
    /// Measured range, metres.
    pub range: f32,
    /// The JT128's confidence byte (manual J01 §3.1; the SDK's weight factor): bits 7–6 the
    /// cover lens's contamination level, bits 5–0 the discrete-noise level. 0 when the packets
    /// carry none (status flags bit 5 clear).
    pub confidence: u8,
}

impl Point {
    /// Bits 5–0 of [`Point::confidence`], 0–63: the higher, the more likely the return is a
    /// discrete noise point (rain, dust, a weak or split return).
    pub fn noise_level(&self) -> u8 {
        self.confidence & 0x3F
    }

    /// Bits 7–6 of [`Point::confidence`], 0–3: how dirty the cover lens is.
    pub fn dirt_level(&self) -> u8 {
        self.confidence >> 6
    }
}

/// One sweep of points, cut at the frame-start azimuth.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Frame {
    /// Sensor time of the earliest point, seconds.
    pub t0: f64,
    pub points: Vec<Point>,
}
