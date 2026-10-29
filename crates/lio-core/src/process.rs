//! The IMU process model: `get_f`, `df_dx`, `df_dw` and `process_noise_cov`
//! from `include/use-ikfom.hpp`.

use crate::mat::{Mat, Mat3, Vec2, Vec3};
use crate::state::{DIM, DOF, FlatState, State, idx};

/// Process-noise dimension: gyro, accel, gyro-bias and accel-bias white noise.
pub const NOISE_DOF: usize = 12;

pub type ProcessNoise = Mat<NOISE_DOF, NOISE_DOF>;

/// One IMU reading as the filter consumes it (`input_ikfom`).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Input {
    pub acc: Vec3,
    pub gyro: Vec3,
}

/// State derivative, in flattened (24-dim) coordinates.
pub fn get_f(s: &State, input: &Input) -> FlatState {
    let omega = input.gyro - s.bg;
    let a_inertial = s.rot.rotate(&(input.acc - s.ba));
    let mut res = FlatState::zeros();
    for i in 0..3 {
        res[idx::POS + i] = s.vel[i];
        res[idx::ROT + i] = omega[i];
        res[idx::VEL + i] = a_inertial[i] + s.grav.vec[i];
    }
    res
}

/// Jacobian of [`get_f`] with respect to the error state.
pub fn df_dx(s: &State, input: &Input) -> Mat<DIM, DOF> {
    let mut cov = Mat::<DIM, DOF>::zeros();
    cov.set_block(idx::POS, idx::VEL, &Mat3::identity());
    let acc = input.acc - s.ba;
    let r = s.rot.to_matrix();
    cov.set_block(idx::VEL, idx::ROT, &(-r * acc.hat()));
    cov.set_block(idx::VEL, idx::BA, &(-r));
    cov.set_block(idx::VEL, idx::GRAV, &s.grav.mx(&Vec2::zeros()));
    cov.set_block(idx::ROT, idx::BG, &(-Mat3::identity()));
    cov
}

/// Jacobian of [`get_f`] with respect to the process noise.
pub fn df_dw(s: &State, _input: &Input) -> Mat<DIM, NOISE_DOF> {
    let mut cov = Mat::<DIM, NOISE_DOF>::zeros();
    cov.set_block(idx::VEL, 3, &(-s.rot.to_matrix()));
    cov.set_block(idx::ROT, 0, &(-Mat3::identity()));
    cov.set_block(idx::BG, 6, &Mat3::identity());
    cov.set_block(idx::BA, 9, &Mat3::identity());
    cov
}

/// FAST-LIO's default process noise. The IMU front end overwrites the diagonal
/// from the configured covariances before each prediction.
pub fn process_noise_cov() -> ProcessNoise {
    let mut q = ProcessNoise::zeros();
    for i in 0..3 {
        q[(i, i)] = 0.0001;
        q[(3 + i, 3 + i)] = 0.0001;
        q[(6 + i, 6 + i)] = 0.00001;
        q[(9 + i, 9 + i)] = 0.00001;
    }
    q
}
