//! FAST-LIO's filter state (`state_ikfom` in `include/use-ikfom.hpp`) and its
//! manifold operations.
//!
//! The error state has 23 degrees of freedom; the flattened state that the
//! process model differentiates has 24. They share offsets because the only
//! component of different size, gravity, comes last.

use crate::mat::{Mat, Vec2, Vec3};
use crate::s2::S2;
use crate::so3::{self, Quat};

/// Degrees of freedom of the error state.
pub const DOF: usize = 23;
/// Size of the flattened state (`flatted_state` in IKFoM).
pub const DIM: usize = 24;

/// Offsets of each component in the error state and the flattened state.
pub mod idx {
    pub const POS: usize = 0;
    pub const ROT: usize = 3;
    pub const OFFSET_R: usize = 6;
    pub const OFFSET_T: usize = 9;
    pub const VEL: usize = 12;
    pub const BG: usize = 15;
    pub const BA: usize = 18;
    pub const GRAV: usize = 21;
}

pub type ErrorState = Mat<DOF, 1>;
pub type FlatState = Mat<DIM, 1>;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct State {
    /// IMU position in the world frame.
    pub pos: Vec3,
    /// IMU attitude, body to world.
    pub rot: Quat,
    /// LiDAR-to-IMU rotation.
    pub offset_r_l_i: Quat,
    /// LiDAR-to-IMU translation.
    pub offset_t_l_i: Vec3,
    pub vel: Vec3,
    /// Gyroscope bias.
    pub bg: Vec3,
    /// Accelerometer bias.
    pub ba: Vec3,
    pub grav: S2,
}

fn seg3<const N: usize>(v: &Mat<N, 1>, at: usize) -> Vec3 {
    Vec3::new(v[at], v[at + 1], v[at + 2])
}

fn put3<const N: usize>(v: &mut Mat<N, 1>, at: usize, s: &Vec3) {
    for i in 0..3 {
        v[at + i] = s[i];
    }
}

impl State {
    /// `boxplus`: applies an error-state increment.
    pub fn boxplus(&mut self, dx: &ErrorState) {
        use idx::*;
        self.pos += seg3(dx, POS);
        self.rot = self.rot * so3::exp(&seg3(dx, ROT), 1.0);
        self.offset_r_l_i = self.offset_r_l_i * so3::exp(&seg3(dx, OFFSET_R), 1.0);
        self.offset_t_l_i += seg3(dx, OFFSET_T);
        self.vel += seg3(dx, VEL);
        self.bg += seg3(dx, BG);
        self.ba += seg3(dx, BA);
        self.grav
            .boxplus(&Vec2::from_array([dx[GRAV], dx[GRAV + 1]]), 1.0);
    }

    /// `boxminus`: the error-state difference `self ⊟ other`.
    pub fn boxminus(&self, other: &State) -> ErrorState {
        use idx::*;
        let mut d = ErrorState::zeros();
        put3(&mut d, POS, &(self.pos - other.pos));
        put3(&mut d, ROT, &so3::log(&(other.rot.conjugate() * self.rot)));
        put3(
            &mut d,
            OFFSET_R,
            &so3::log(&(other.offset_r_l_i.conjugate() * self.offset_r_l_i)),
        );
        put3(&mut d, OFFSET_T, &(self.offset_t_l_i - other.offset_t_l_i));
        put3(&mut d, VEL, &(self.vel - other.vel));
        put3(&mut d, BG, &(self.bg - other.bg));
        put3(&mut d, BA, &(self.ba - other.ba));
        let g = self.grav.boxminus(&other.grav);
        d[GRAV] = g[0];
        d[GRAV + 1] = g[1];
        d
    }

    /// `oplus`: integrates the flattened-state derivative `f` over `dt`.
    pub fn oplus(&mut self, f: &FlatState, dt: f64) {
        use idx::*;
        self.pos += seg3(f, POS) * dt;
        self.rot = self.rot * so3::exp(&seg3(f, ROT), dt);
        self.offset_r_l_i = self.offset_r_l_i * so3::exp(&seg3(f, OFFSET_R), dt);
        self.offset_t_l_i += seg3(f, OFFSET_T) * dt;
        self.vel += seg3(f, VEL) * dt;
        self.bg += seg3(f, BG) * dt;
        self.ba += seg3(f, BA) * dt;
        self.grav.oplus(&seg3(f, GRAV), dt);
    }
}
