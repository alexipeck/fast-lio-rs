//! The error-state Kalman filter on manifolds (`esekfom::esekf` in
//! `include/IKFoM_toolkit/esekfom/esekfom.hpp`), with the dense code path
//! FAST-LIO compiles (`USE_sparse` undefined).

use crate::mat::{Mat, Mat3, Vec2, Vec3};
use crate::process::{self, Input, NOISE_DOF, ProcessNoise};
use crate::so3;
use crate::state::{DOF, State, idx};

pub type Cov = Mat<DOF, DOF>;

#[derive(Clone, Debug, PartialEq)]
pub struct Esekf {
    pub x: State,
    pub p: Cov,
}

impl Default for Esekf {
    fn default() -> Self {
        Self {
            x: State::default(),
            p: Cov::identity(),
        }
    }
}

const VECT_STATES: [usize; 5] = [idx::POS, idx::OFFSET_T, idx::VEL, idx::BG, idx::BA];
const SO3_STATES: [usize; 2] = [idx::ROT, idx::OFFSET_R];

fn rows3<const C: usize>(m: &Mat<24, C>, at: usize) -> Mat<3, C> {
    m.block(at, 0)
}

impl Esekf {
    /// Propagates the state and covariance over `dt` with one IMU input.
    pub fn predict(&mut self, dt: f64, q: &ProcessNoise, input: &Input) {
        let f = process::get_f(&self.x, input);
        let f_x = process::df_dx(&self.x, input);
        let f_w = process::df_dw(&self.x, input);
        let x_before = self.x;
        self.x.oplus(&f, dt);

        let mut f_x1 = Cov::identity();
        let mut f_x_final = Cov::zeros();
        let mut f_w_final = Mat::<DOF, NOISE_DOF>::zeros();

        for &at in &VECT_STATES {
            f_x_final.set_block(at, 0, &rows3(&f_x, at));
            f_w_final.set_block(at, 0, &rows3(&f_w, at));
        }

        // For the rotation states, IKFoM sets the diagonal block of F_x1 to
        // exp(-f·dt, scalar(1/2)). Integer division makes that scale 0, so the
        // block stays exactly the identity; only the A-matrix term remains.
        for &at in &SO3_STATES {
            let seg = Vec3::new(-f[at] * dt, -f[at + 1] * dt, -f[at + 2] * dt);
            let a = so3::a_matrix(&seg);
            f_x_final.set_block(at, 0, &(a * rows3(&f_x, at)));
            f_w_final.set_block(at, 0, &(a * rows3(&f_w, at)));
        }

        // Gravity (S2). The same scalar(1/2) makes its rotation the identity too.
        let at = idx::GRAV;
        let seg = Vec3::new(f[at] * dt, f[at + 1] * dt, f[at + 2] * dt);
        let nx = self.x.grav.nx_yy();
        let mx = x_before.grav.mx(&Vec2::zeros());
        f_x1.set_block(at, at, &(nx * Mat3::identity() * mx));
        let res_temp_s2 =
            -nx * Mat3::identity() * x_before.grav.hat() * so3::a_matrix(&seg).transpose();
        f_x_final.set_block(at, 0, &(res_temp_s2 * rows3(&f_x, at)));
        f_w_final.set_block(at, 0, &(res_temp_s2 * rows3(&f_w, at)));

        f_x1 += f_x_final * dt;
        let g = f_w_final * dt;
        self.p = f_x1 * self.p * f_x1.transpose() + g * *q * g.transpose();
    }
}
