//! Voxel-grid downsampling, as `pcl::VoxelGrid<PointXYZINormal>` (PCL 1.12)
//! does it with FAST-LIO's settings: every voxel kept, and every field
//! averaged in `f32`.
//!
//! PCL sorts points by voxel with an unstable sort, so the order in which a
//! voxel's points are summed, and so the last bit of each centroid, is
//! unspecified even in C++. This sort is stable.

use crate::point::Point;

/// One output point per occupied voxel of side `leaf`, ordered by voxel index.
pub fn downsample(input: &[Point], leaf: f32) -> Vec<Point> {
    if input.is_empty() {
        return Vec::new();
    }
    let inv = 1.0f32 / leaf;
    let (mut min, mut max) = ([f32::MAX; 3], [f32::MIN; 3]);
    for p in input {
        for (a, v) in [p.x, p.y, p.z].into_iter().enumerate() {
            min[a] = min[a].min(v);
            max[a] = max[a].max(v);
        }
    }

    let extent = |a: usize| ((max[a] - min[a]) * inv) as i64 + 1;
    if extent(0) * extent(1) * extent(2) > i64::from(i32::MAX) {
        // PCL warns that integer indices would overflow and returns the input.
        return input.to_vec();
    }

    let min_b = min.map(|v| (v * inv).floor() as i32);
    let max_b = max.map(|v| (v * inv).floor() as i32);
    let div0 = max_b[0] - min_b[0] + 1;
    let div1 = max_b[1] - min_b[1] + 1;
    let mul = [1, div0, div0 * div1];

    let mut index: Vec<(u32, usize)> = input
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let ijk = [p.x, p.y, p.z]
                .into_iter()
                .zip(min_b)
                .map(|(v, b)| ((v * inv).floor() - b as f32) as i32);
            let idx: i32 = ijk.zip(mul).map(|(c, m)| c * m).sum();
            (idx as u32, i)
        })
        .collect();
    index.sort_by_key(|&(idx, _)| idx);

    index
        .chunk_by(|a, b| a.0 == b.0)
        .map(|voxel| {
            let mut sum = Point::default();
            for &(_, i) in voxel {
                let p = &input[i];
                sum.x += p.x;
                sum.y += p.y;
                sum.z += p.z;
                sum.intensity += p.intensity;
                sum.curvature += p.curvature;
            }
            let n = voxel.len() as f32;
            Point {
                x: sum.x / n,
                y: sum.y / n,
                z: sum.z / n,
                intensity: sum.intensity / n,
                curvature: sum.curvature / n,
            }
        })
        .collect()
}
