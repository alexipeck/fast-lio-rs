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

/// What the measurement model returns each iteration (`dyn_share_datastruct`).
#[derive(Clone, Debug, Default)]
pub struct DynShare {
    /// Set to false by the model when it found nothing to match.
    pub valid: bool,
    /// True on the first iteration and after an iteration that converged.
    /// FAST-LIO searches for new correspondences only then.
    pub converge: bool,
    /// One row per measurement: the derivative with respect to position,
    /// rotation, LiDAR-to-IMU rotation and LiDAR-to-IMU translation.
    pub h_x: Vec<[f64; 12]>,
    pub h: Vec<f64>,
}

fn get3(v: &Mat<DOF, 1>, at: usize) -> Vec3 {
    Vec3::new(v[at], v[at + 1], v[at + 2])
}

fn set3(v: &mut Mat<DOF, 1>, at: usize, s: &Vec3) {
    for i in 0..3 {
        v[at + i] = s[i];
    }
}

/// Row-major dynamic matrix product helper for the rarely used small-measurement branch.
fn dyn_mul(a: &[f64], b: &[f64], n: usize, k: usize, m: usize) -> Vec<f64> {
    let mut out = vec![0.0; n * m];
    for r in 0..n {
        for j in 0..k {
            let x = a[r * k + j];
            for c in 0..m {
                out[r * m + c] += x * b[j * m + c];
            }
        }
    }
    out
}

impl Esekf {
    /// `update_iterated_dyn_share_modified`: the iterated error-state update
    /// with scalar measurement noise `r`, at most `maximum_iter + 1` model
    /// evaluations, and per-dimension convergence thresholds `limit`.
    pub fn update_iterated_dyn_share_modified(
        &mut self,
        r: f64,
        maximum_iter: usize,
        limit: &[f64; DOF],
        mut h_model: impl FnMut(&State, &mut DynShare),
    ) {
        let mut dyn_share = DynShare {
            valid: true,
            converge: true,
            ..Default::default()
        };
        let mut t = 0;
        let x_propagated = self.x;
        let p_propagated = self.p;
        let mut k_h: Mat<DOF, 1>;
        let mut k_x: Cov;
        let max = maximum_iter as i64;

        for i in -1..max {
            dyn_share.valid = true;
            h_model(&self.x, &mut dyn_share);
            if !dyn_share.valid {
                continue;
            }
            let m = dyn_share.h_x.len();
            let dx = self.x.boxminus(&x_propagated);
            let mut dx_new = dx;
            self.p = p_propagated;

            for &at in &SO3_STATES {
                let res = so3::a_matrix(&get3(&dx, at)).transpose();
                let rotated = res * get3(&dx_new, at);
                set3(&mut dx_new, at, &rotated);
                let rows: Mat<3, DOF> = self.p.block(at, 0);
                self.p.set_block(at, 0, &(res * rows));
                let cols: Mat<DOF, 3> = self.p.block(0, at);
                self.p.set_block(0, at, &(cols * res.transpose()));
            }
            {
                let at = idx::GRAV;
                let seg = Vec2::from_array([dx[at], dx[at + 1]]);
                let res = self.x.grav.nx_yy() * x_propagated.grav.mx(&seg);
                let d = res * Vec2::from_array([dx_new[at], dx_new[at + 1]]);
                dx_new[at] = d[0];
                dx_new[at + 1] = d[1];
                let rows: Mat<2, DOF> = self.p.block(at, 0);
                self.p.set_block(at, 0, &(res * rows));
                let cols: Mat<DOF, 2> = self.p.block(0, at);
                self.p.set_block(0, at, &(cols * res.transpose()));
            }

            if DOF > m {
                // Fewer measurements than states: K = P·Hᵀ·(H·P·Hᵀ/R + I)⁻¹/R.
                let h_cur: Vec<f64> = dyn_share
                    .h_x
                    .iter()
                    .flat_map(|row| (0..DOF).map(move |c| if c < 12 { row[c] } else { 0.0 }))
                    .collect();
                let h_t: Vec<f64> = (0..DOF)
                    .flat_map(|c| (0..m).map(move |k| (c, k)))
                    .map(|(c, k)| h_cur[k * DOF + c])
                    .collect();
                let p = self.p.to_row_vec();
                let pht = dyn_mul(&p, &h_t, DOF, DOF, m);
                let hp = dyn_mul(&h_cur, &p, m, DOF, DOF);
                let mut s = dyn_mul(&hp, &h_t, m, DOF, m);
                for a in 0..m {
                    for b in 0..m {
                        s[a * m + b] = s[a * m + b] / r + if a == b { 1.0 } else { 0.0 };
                    }
                }
                crate::mat::lu_inverse_in_place(&mut s, m);
                let k: Vec<f64> = dyn_mul(&pht, &s, DOF, m, m)
                    .into_iter()
                    .map(|v| v / r)
                    .collect();
                let kh = dyn_mul(&k, &dyn_share.h, DOF, m, 1);
                let kx = dyn_mul(&k, &h_cur, DOF, m, DOF);
                k_h = Mat::from_row_slice(&kh);
                k_x = Mat::from_row_slice(&kx);
            } else {
                let mut p_temp = (self.p / r).inverse();
                let mut hth = Mat::<12, 12>::zeros();
                for row in &dyn_share.h_x {
                    for a in 0..12 {
                        for b in 0..12 {
                            hth[(a, b)] += row[a] * row[b];
                        }
                    }
                }
                for a in 0..12 {
                    for b in 0..12 {
                        p_temp[(a, b)] += hth[(a, b)];
                    }
                }
                let p_inv = p_temp.inverse();
                let p_inv12: Mat<DOF, 12> = p_inv.block(0, 0);
                // (P_inv[:, :12] · Hᵀ) · h, in Eigen's evaluation order.
                k_h = Mat::<DOF, 1>::zeros();
                for (row, &hk) in dyn_share.h_x.iter().zip(&dyn_share.h) {
                    for a in 0..DOF {
                        let pa: f64 = (0..12).map(|c| p_inv12[(a, c)] * row[c]).sum();
                        k_h[a] += pa * hk;
                    }
                }
                k_x = Cov::zeros();
                k_x.set_block(0, 0, &(p_inv12 * hth));
            }

            let dx_ = k_h + (k_x - Cov::identity()) * dx_new;
            self.x.boxplus(&dx_);
            dyn_share.converge = !(0..DOF).any(|j| dx_[j].abs() > limit[j]);
            if dyn_share.converge {
                t += 1;
            }
            if t == 0 && i == max - 2 {
                dyn_share.converge = true;
            }

            if t > 1 || i == max - 1 {
                let mut l = self.p;
                for &at in &SO3_STATES {
                    let res = so3::a_matrix(&get3(&dx_, at)).transpose();
                    let rows: Mat<3, DOF> = self.p.block(at, 0);
                    l.set_block(at, 0, &(res * rows));
                    let kx_rows: Mat<3, 12> = k_x.block(at, 0);
                    k_x.set_block(at, 0, &(res * kx_rows));
                    let l_cols: Mat<DOF, 3> = l.block(0, at);
                    l.set_block(0, at, &(l_cols * res.transpose()));
                    let p_cols: Mat<DOF, 3> = self.p.block(0, at);
                    self.p.set_block(0, at, &(p_cols * res.transpose()));
                }
                {
                    let at = idx::GRAV;
                    let seg = Vec2::from_array([dx_[at], dx_[at + 1]]);
                    let res = self.x.grav.nx_yy() * x_propagated.grav.mx(&seg);
                    let rows: Mat<2, DOF> = self.p.block(at, 0);
                    l.set_block(at, 0, &(res * rows));
                    let kx_rows: Mat<2, 12> = k_x.block(at, 0);
                    k_x.set_block(at, 0, &(res * kx_rows));
                    let l_cols: Mat<DOF, 2> = l.block(0, at);
                    l.set_block(0, at, &(l_cols * res.transpose()));
                    let p_cols: Mat<DOF, 2> = self.p.block(0, at);
                    self.p.set_block(0, at, &(p_cols * res.transpose()));
                }
                let kx12: Mat<DOF, 12> = k_x.block(0, 0);
                let p12: Mat<12, DOF> = self.p.block(0, 0);
                self.p = l - kx12 * p12;
                return;
            }
        }
    }
}
