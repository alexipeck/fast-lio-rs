//! FAST-LIO's point type, `pcl::PointXYZINormal`, minus the normal fields. Those
//! are always zero in FAST-LIO's pipeline and never read.

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Point {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: f32,
    /// FAST-LIO stores each point's time offset from the frame start here, in ms.
    pub curvature: f32,
}

impl Point {
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self {
            x,
            y,
            z,
            intensity: 0.0,
            curvature: 0.0,
        }
    }

    /// Squared distance (`calc_dist` in FAST-LIO and ikd-Tree).
    pub fn sq_dist(&self, o: &Point) -> f32 {
        (self.x - o.x) * (self.x - o.x)
            + (self.y - o.y) * (self.y - o.y)
            + (self.z - o.z) * (self.z - o.z)
    }
}
