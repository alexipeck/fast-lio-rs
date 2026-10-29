//! FAST-LIO's algorithm, implemented exactly as in hku-mars/FAST_LIO at commit
//! `7cc4175`.
//!
//! What "exactly" covers is listed in the algorithm contract in `docs/plan.md`.
//! Departures from the C++ are listed in `DEVIATIONS.md`. This crate has no
//! sensor or I/O code.

pub mod esekf;
pub mod ikd_tree;
pub mod imu_init;
pub mod imu_process;
pub mod lio;
pub mod mat;
pub mod plane;
pub mod point;
pub mod process;
pub mod rays;
pub mod s2;
pub mod so3;
pub mod state;
pub mod voxel;
