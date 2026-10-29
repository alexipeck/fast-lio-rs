//! FAST-LIO's mapping loop (`src/laserMapping.cpp`) without ROS: buffering
//! and frame/IMU synchronisation (`sync_packages`), the correspondence model
//! (`h_share_model`), the local map window (`lasermap_fov_segment`) and map
//! growth (`map_incremental`).

use std::collections::VecDeque;

use rayon::prelude::*;

use crate::esekf::{DynShare, Esekf};
use crate::ikd_tree::{BoxPoint, IkdTree};
use crate::imu_init::ImuMeasurement;
use crate::imu_process::{ImuProcess, MeasureGroup, Timeline};
use crate::mat::{Mat3, Vec3};
use crate::plane::{NUM_MATCH_POINTS, esti_plane};
use crate::point::Point;
use crate::so3::Quat;
use crate::state::{DOF, State};
use crate::voxel;

const INIT_TIME: f64 = 0.1;
const LASER_POINT_COV: f64 = 0.001;
const MOV_THRESHOLD: f32 = 1.5;

/// FAST-LIO's parameters, named as in its YAML and launch files.
#[derive(Clone, Debug)]
pub struct LioConfig {
    pub max_iteration: usize,
    pub filter_size_surf: f64,
    pub filter_size_map: f64,
    pub cube_side_length: f64,
    pub det_range: f32,
    pub gyr_cov: f64,
    pub acc_cov: f64,
    pub b_gyr_cov: f64,
    pub b_acc_cov: f64,
    pub blind: f64,
    pub point_filter_num: usize,
    pub extrinsic_est_en: bool,
    /// LiDAR position in the IMU frame.
    pub extrinsic_t: [f64; 3],
    /// LiDAR-to-IMU rotation, row-major.
    pub extrinsic_r: [f64; 9],
    /// Subtracted from every IMU timestamp (`time_offset_lidar_to_imu`).
    pub time_offset_lidar_to_imu: f64,
}

impl Default for LioConfig {
    /// FAST-LIO's Mid-360 settings (`config/mid360.yaml`, `launch/mapping_mid360.launch`),
    /// the closest shipped configuration to the JT128.
    fn default() -> Self {
        Self {
            max_iteration: 3,
            filter_size_surf: 0.5,
            filter_size_map: 0.5,
            cube_side_length: 1000.0,
            det_range: 100.0,
            gyr_cov: 0.1,
            acc_cov: 0.1,
            b_gyr_cov: 0.0001,
            b_acc_cov: 0.0001,
            blind: 0.5,
            point_filter_num: 3,
            extrinsic_est_en: false,
            extrinsic_t: [0.0; 3],
            extrinsic_r: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            time_offset_lidar_to_imu: 0.0,
        }
    }
}

/// What happened to one synchronised frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameKind {
    /// The very first frame only sets the time origin.
    FirstScan,
    /// Frames spent averaging the IMU while the sensor is still.
    ImuInit,
    /// The first usable frame becomes the initial map; no update.
    MapInit,
    /// Matched against the map and corrected.
    Updated,
    /// Too few points survived to do anything.
    Skipped,
}

#[derive(Clone, Debug)]
pub struct FrameOutput {
    pub kind: FrameKind,
    pub lidar_beg_time: f64,
    pub lidar_end_time: f64,
    /// The state at the frame's end: corrected if `kind` is `Updated`. Before
    /// the IMU is initialised it holds the configured extrinsic, which
    /// FAST-LIO's own state only receives at initialisation.
    pub state: State,
    /// The frame's IMU timeline, once the IMU is initialised.
    pub timeline: Option<Timeline>,
    pub downsampled_points: usize,
    pub effective_features: usize,
    pub map_points: usize,
}

pub struct Lio {
    cfg: LioConfig,
    pub kf: Esekf,
    imu: ImuProcess,
    pub tree: IkdTree,
    lidar_buffer: VecDeque<(f64, Vec<Point>)>,
    imu_buffer: VecDeque<ImuMeasurement>,
    last_timestamp_lidar: f64,
    last_timestamp_imu: f64,
    lidar_pushed: bool,
    meas: MeasureGroup,
    lidar_mean_scantime: f64,
    scan_num: u64,
    flg_first_scan: bool,
    first_lidar_time: f64,
    local_map: Option<BoxPoint>,
    point_selected_surf: Vec<bool>,
    nearest_points: Vec<Vec<Point>>,
    limit: [f64; DOF],
}

impl Lio {
    pub fn new(cfg: LioConfig) -> Self {
        let t = Vec3::from_array(cfg.extrinsic_t);
        let r = Mat3::from_row_slice(&cfg.extrinsic_r);
        let imu = ImuProcess::new(t, r, cfg.gyr_cov, cfg.acc_cov, cfg.b_gyr_cov, cfg.b_acc_cov);
        Self {
            cfg,
            kf: Esekf::default(),
            imu,
            tree: IkdTree::default(),
            lidar_buffer: VecDeque::new(),
            imu_buffer: VecDeque::new(),
            last_timestamp_lidar: 0.0,
            last_timestamp_imu: -1.0,
            lidar_pushed: false,
            meas: MeasureGroup::default(),
            lidar_mean_scantime: 0.0,
            scan_num: 0,
            flg_first_scan: true,
            first_lidar_time: 0.0,
            local_map: None,
            point_selected_surf: Vec::new(),
            nearest_points: Vec::new(),
            limit: [0.001; DOF],
        }
    }

    pub fn config(&self) -> &LioConfig {
        &self.cfg
    }

    /// `imu_cbk`.
    pub fn push_imu(&mut self, mut m: ImuMeasurement) {
        m.t -= self.cfg.time_offset_lidar_to_imu;
        if m.t < self.last_timestamp_imu {
            self.imu_buffer.clear();
        }
        self.last_timestamp_imu = m.t;
        self.imu_buffer.push_back(m);
    }

    /// The preprocessing of FAST-LIO's spinning-LiDAR handlers: keep every
    /// `point_filter_num`-th point and drop those within `blind`. Points are in
    /// arrival order, with `curvature` = ms since the frame's start.
    pub fn preprocess(&self, points: impl IntoIterator<Item = Point>) -> Vec<Point> {
        let blind_sq = self.cfg.blind * self.cfg.blind;
        let n = self.cfg.point_filter_num.max(1);
        points
            .into_iter()
            .enumerate()
            .filter(|(i, p)| {
                let d = f64::from(p.x * p.x + p.y * p.y + p.z * p.z);
                i % n == 0 && d > blind_sq
            })
            .map(|(_, p)| p)
            .collect()
    }

    /// `standard_pcl_cbk` with [`Lio::preprocess`] applied.
    pub fn push_frame(&mut self, beg_time: f64, points: impl IntoIterator<Item = Point>) {
        let kept = self.preprocess(points);
        self.push_preprocessed_frame(beg_time, kept);
    }

    /// `standard_pcl_cbk` for an already preprocessed frame.
    pub fn push_preprocessed_frame(&mut self, beg_time: f64, points: Vec<Point>) {
        if beg_time < self.last_timestamp_lidar {
            self.lidar_buffer.clear();
        }
        self.lidar_buffer.push_back((beg_time, points));
        self.last_timestamp_lidar = beg_time;
    }

    /// Processes every frame whose IMU coverage is complete.
    pub fn step(&mut self) -> Vec<FrameOutput> {
        let mut out = Vec::new();
        while self.sync_packages() {
            out.push(self.process_frame());
        }
        out
    }

    /// `sync_packages`.
    fn sync_packages(&mut self) -> bool {
        if self.lidar_buffer.is_empty() || self.imu_buffer.is_empty() {
            return false;
        }
        if !self.lidar_pushed {
            // The cloud moves rather than copies; the emptied entry is popped
            // below once the IMU covers the frame, as FAST-LIO pops its pointer.
            let (beg, cloud) = self.lidar_buffer.front_mut().unwrap();
            self.meas.lidar = std::mem::take(cloud);
            self.meas.lidar_beg_time = *beg;
            let last_ms = self
                .meas
                .lidar
                .last()
                .map_or(0.0, |p| f64::from(p.curvature));
            let end = if self.meas.lidar.len() <= 1
                || last_ms / 1000.0 < 0.5 * self.lidar_mean_scantime
            {
                self.meas.lidar_beg_time + self.lidar_mean_scantime
            } else {
                self.scan_num += 1;
                self.lidar_mean_scantime +=
                    (last_ms / 1000.0 - self.lidar_mean_scantime) / self.scan_num as f64;
                self.meas.lidar_beg_time + last_ms / 1000.0
            };
            self.meas.lidar_end_time = end;
            self.lidar_pushed = true;
        }
        if self.last_timestamp_imu < self.meas.lidar_end_time {
            return false;
        }
        self.meas.imu.clear();
        let mut imu_time = self.imu_buffer.front().unwrap().t;
        while let Some(front) = self.imu_buffer.front() {
            if imu_time >= self.meas.lidar_end_time {
                break;
            }
            imu_time = front.t;
            if imu_time > self.meas.lidar_end_time {
                break;
            }
            self.meas.imu.push(*front);
            self.imu_buffer.pop_front();
        }
        self.lidar_buffer.pop_front();
        self.lidar_pushed = false;
        true
    }

    fn output(
        &self,
        kind: FrameKind,
        timeline: Option<Timeline>,
        down: usize,
        eff: usize,
    ) -> FrameOutput {
        let mut state = self.kf.x;
        if matches!(kind, FrameKind::FirstScan | FrameKind::ImuInit) {
            // FAST-LIO puts the configured extrinsic into the state only when
            // the IMU initialisation completes (`IMU_init`); until then the
            // state holds the identity. Report the configured one, as the
            // initialisation will set it, so these frames' rays use the real
            // mounting. The pose itself is the identity either way.
            state.offset_t_l_i = Vec3::from_array(self.cfg.extrinsic_t);
            state.offset_r_l_i = Quat::from_matrix(&Mat3::from_row_slice(&self.cfg.extrinsic_r));
        }
        FrameOutput {
            kind,
            lidar_beg_time: self.meas.lidar_beg_time,
            lidar_end_time: self.meas.lidar_end_time,
            state,
            timeline,
            downsampled_points: down,
            effective_features: eff,
            map_points: self.tree.valid_num(),
        }
    }

    /// One pass of `main`'s loop body after a successful `sync_packages`.
    fn process_frame(&mut self) -> FrameOutput {
        if self.flg_first_scan {
            self.first_lidar_time = self.meas.lidar_beg_time;
            self.flg_first_scan = false;
            return self.output(FrameKind::FirstScan, None, 0, 0);
        }
        let meas = std::mem::take(&mut self.meas);
        let processed = self.imu.process(&meas, &mut self.kf);
        self.meas = meas;
        let Some((undistorted, timeline)) = processed else {
            return self.output(FrameKind::ImuInit, None, 0, 0);
        };
        let state = self.kf.x;
        if undistorted.is_empty() {
            return self.output(FrameKind::Skipped, Some(timeline), 0, 0);
        }
        let flg_ekf_inited = self.meas.lidar_beg_time - self.first_lidar_time >= INIT_TIME;

        let pos_lid = state.pos + state.rot.rotate(&state.offset_t_l_i);
        let boxes = self.lasermap_fov_segment(&pos_lid);
        if !boxes.is_empty() {
            self.tree.delete_point_boxes(&boxes);
        }

        let down_body = voxel::downsample(&undistorted, self.cfg.filter_size_surf as f32);
        if self.tree.is_empty() {
            if down_body.len() > 5 {
                self.tree
                    .set_downsample_param(self.cfg.filter_size_map as f32);
                let world: Vec<Point> =
                    down_body.iter().map(|p| body_to_world(p, &state)).collect();
                self.tree.build(world);
            }
            return self.output(FrameKind::MapInit, Some(timeline), down_body.len(), 0);
        }
        if down_body.len() < 5 {
            return self.output(FrameKind::Skipped, Some(timeline), down_body.len(), 0);
        }

        let n = down_body.len();
        if self.point_selected_surf.len() < n {
            self.point_selected_surf.resize(n, true);
        }
        self.nearest_points.resize(n, Vec::new());
        let mut effective = 0;
        {
            let tree = &self.tree;
            let selected = &mut self.point_selected_surf[..n];
            let nearest = &mut self.nearest_points[..];
            let extrinsic_est = self.cfg.extrinsic_est_en;
            let body = &down_body;
            self.kf.update_iterated_dyn_share_modified(
                LASER_POINT_COV,
                self.cfg.max_iteration,
                &self.limit,
                |s, d| {
                    effective = h_share_model(s, body, tree, nearest, selected, extrinsic_est, d);
                },
            );
        }
        let corrected = self.kf.x;
        self.map_incremental(&down_body, &corrected, flg_ekf_inited);
        self.output(FrameKind::Updated, Some(timeline), n, effective)
    }

    /// `lasermap_fov_segment`: slides the local map cube with the sensor and
    /// returns the regions that fall out of it.
    fn lasermap_fov_segment(&mut self, pos: &Vec3) -> Vec<BoxPoint> {
        let half = self.cfg.cube_side_length / 2.0;
        let Some(local) = self.local_map else {
            self.local_map = Some(BoxPoint {
                min: [0, 1, 2].map(|i| (pos[i] - half) as f32),
                max: [0, 1, 2].map(|i| (pos[i] + half) as f32),
            });
            return Vec::new();
        };
        let det = self.cfg.det_range;
        let mut dist = [[0.0f32; 2]; 3];
        let mut need_move = false;
        for i in 0..3 {
            dist[i][0] = (pos[i] - f64::from(local.min[i])).abs() as f32;
            dist[i][1] = (pos[i] - f64::from(local.max[i])).abs() as f32;
            if dist[i][0] <= MOV_THRESHOLD * det || dist[i][1] <= MOV_THRESHOLD * det {
                need_move = true;
            }
        }
        if !need_move {
            return Vec::new();
        }
        let mut new_local = local;
        let mov_dist = ((self.cfg.cube_side_length
            - 2.0 * f64::from(MOV_THRESHOLD) * f64::from(det))
            * 0.5
            * 0.9)
            .max(f64::from(det * (MOV_THRESHOLD - 1.0))) as f32;
        let mut boxes = Vec::new();
        for (i, d) in dist.iter().enumerate() {
            let mut tmp = local;
            if d[0] <= MOV_THRESHOLD * det {
                new_local.max[i] -= mov_dist;
                new_local.min[i] -= mov_dist;
                tmp.min[i] = local.max[i] - mov_dist;
                boxes.push(tmp);
            } else if d[1] <= MOV_THRESHOLD * det {
                new_local.max[i] += mov_dist;
                new_local.min[i] += mov_dist;
                tmp.max[i] = local.min[i] + mov_dist;
                boxes.push(tmp);
            }
        }
        self.local_map = Some(new_local);
        boxes
    }

    /// `map_incremental`.
    fn map_incremental(&mut self, down_body: &[Point], state: &State, flg_ekf_inited: bool) {
        let f = self.cfg.filter_size_map;
        let mut to_add = Vec::with_capacity(down_body.len());
        let mut no_need_downsample = Vec::with_capacity(down_body.len());
        for (i, p) in down_body.iter().enumerate() {
            let world = body_to_world(p, state);
            let near = &self.nearest_points[i];
            if near.is_empty() || !flg_ekf_inited {
                to_add.push(world);
                continue;
            }
            let mid = |v: f32| ((f64::from(v) / f).floor() * f + 0.5 * f) as f32;
            let mid_point = Point::new(mid(world.x), mid(world.y), mid(world.z));
            let dist = world.sq_dist(&mid_point);
            let half = 0.5 * f;
            if f64::from((near[0].x - mid_point.x).abs()) > half
                && f64::from((near[0].y - mid_point.y).abs()) > half
                && f64::from((near[0].z - mid_point.z).abs()) > half
            {
                no_need_downsample.push(world);
                continue;
            }
            let mut need_add = true;
            for q in near.iter().take(NUM_MATCH_POINTS) {
                if near.len() < NUM_MATCH_POINTS {
                    break;
                }
                if q.sq_dist(&mid_point) < dist {
                    need_add = false;
                    break;
                }
            }
            if need_add {
                to_add.push(world);
            }
        }
        self.tree.add_points(&to_add, true);
        self.tree.add_points(&no_need_downsample, false);
    }
}

/// `pointBodyToWorld`: LiDAR-frame point to world, through the IMU.
pub fn body_to_world(p: &Point, s: &State) -> Point {
    let b = Vec3::new(f64::from(p.x), f64::from(p.y), f64::from(p.z));
    let w = s.rot.rotate(&(s.offset_r_l_i.rotate(&b) + s.offset_t_l_i)) + s.pos;
    Point {
        x: w[0] as f32,
        y: w[1] as f32,
        z: w[2] as f32,
        intensity: p.intensity,
        curvature: p.curvature,
    }
}

/// Per-point result of the correspondence search: plane normal and the
/// signed point-to-plane distance.
type Match = Option<([f32; 3], f32)>;

/// `h_share_model`: matches each downsampled point to a plane in the map and
/// fills the measurement Jacobian and residuals. Returns the number of
/// effective features.
fn h_share_model(
    s: &State,
    body: &[Point],
    tree: &IkdTree,
    nearest: &mut [Vec<Point>],
    selected: &mut [bool],
    extrinsic_est: bool,
    d: &mut DynShare,
) -> usize {
    let search = d.converge;
    let matches: Vec<Match> = body
        .par_iter()
        .zip(nearest.par_iter_mut())
        .zip(selected.par_iter_mut())
        .map_init(Vec::new, |dists, ((pb, near), sel)| {
            let p_body = Vec3::new(f64::from(pb.x), f64::from(pb.y), f64::from(pb.z));
            let g = s
                .rot
                .rotate(&(s.offset_r_l_i.rotate(&p_body) + s.offset_t_l_i))
                + s.pos;
            let world = Point::new(g[0] as f32, g[1] as f32, g[2] as f32);
            if search {
                tree.nearest_search(&world, NUM_MATCH_POINTS, f32::INFINITY, near, dists);
                *sel = near.len() >= NUM_MATCH_POINTS && dists[NUM_MATCH_POINTS - 1] <= 5.0;
            }
            if !*sel {
                return None;
            }
            *sel = false;
            let abcd = esti_plane(near, 0.1)?;
            let pd2 = abcd[0] * world.x + abcd[1] * world.y + abcd[2] * world.z + abcd[3];
            // `float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm())`: computed in
            // double, stored as float, compared in double.
            let score = (1.0 - 0.9 * f64::from(pd2.abs()) / p_body.norm().sqrt()) as f32;
            if f64::from(score) > 0.9 {
                *sel = true;
                Some(([abcd[0], abcd[1], abcd[2]], pd2))
            } else {
                None
            }
        })
        .collect();

    d.h_x.clear();
    d.h.clear();
    for (i, m) in matches.iter().enumerate() {
        let Some((normal, pd2)) = m else { continue };
        if !selected[i] {
            continue;
        }
        let pb = &body[i];
        let be = Vec3::new(f64::from(pb.x), f64::from(pb.y), f64::from(pb.z));
        let point_this = s.offset_r_l_i.rotate(&be) + s.offset_t_l_i;
        let norm_vec = Vec3::new(
            f64::from(normal[0]),
            f64::from(normal[1]),
            f64::from(normal[2]),
        );
        let c = s.rot.conjugate().rotate(&norm_vec);
        let a = point_this.hat() * c;
        let mut row = [0.0; 12];
        row[0..3].copy_from_slice(&[norm_vec[0], norm_vec[1], norm_vec[2]]);
        row[3..6].copy_from_slice(&[a[0], a[1], a[2]]);
        if extrinsic_est {
            let b = be.hat() * s.offset_r_l_i.conjugate().to_matrix() * c;
            row[6..9].copy_from_slice(&[b[0], b[1], b[2]]);
            row[9..12].copy_from_slice(&[c[0], c[1], c[2]]);
        }
        d.h_x.push(row);
        d.h.push(-f64::from(*pd2));
    }
    if d.h_x.is_empty() {
        d.valid = false;
    }
    d.h_x.len()
}
