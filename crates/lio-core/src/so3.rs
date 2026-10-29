//! Rotations as unit quaternions, with MTK's exponential and logarithm.
//!
//! The formulas follow Eigen and MTK (`IKFoM_toolkit/mtk/src/mtkmath.hpp`,
//! `mtk/types/SOn.hpp`) term for term. That includes MTK's 3-term Taylor
//! expansion near zero and its 1e-11 tolerance, so results match FAST-LIO to
//! rounding.

use std::ops::Mul;

use crate::mat::{Mat3, Vec3};

/// MTK's `tolerance<double>()`.
pub const TOLERANCE: f64 = 1e-11;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quat {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Default for Quat {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Quat {
    pub const IDENTITY: Quat = Quat {
        w: 1.0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    pub const fn new(w: f64, x: f64, y: f64, z: f64) -> Self {
        Self { w, x, y, z }
    }

    pub fn vec(&self) -> Vec3 {
        Vec3::new(self.x, self.y, self.z)
    }

    pub fn conjugate(&self) -> Self {
        Self::new(self.w, -self.x, -self.y, -self.z)
    }

    /// Rotates `v`, using Eigen's `_transformVector` formula.
    pub fn rotate(&self, v: &Vec3) -> Vec3 {
        let q = self.vec();
        let mut uv = q.cross(v);
        uv += uv;
        *v + uv * self.w + q.cross(&uv)
    }

    /// Eigen's `toRotationMatrix()`.
    pub fn to_matrix(&self) -> Mat3 {
        let tx = 2.0 * self.x;
        let ty = 2.0 * self.y;
        let tz = 2.0 * self.z;
        let twx = tx * self.w;
        let twy = ty * self.w;
        let twz = tz * self.w;
        let txx = tx * self.x;
        let txy = ty * self.x;
        let txz = tz * self.x;
        let tyy = ty * self.y;
        let tyz = tz * self.y;
        let tzz = tz * self.z;
        Mat3::from_row_slice(&[
            1.0 - (tyy + tzz),
            txy - twz,
            txz + twy,
            txy + twz,
            1.0 - (txx + tzz),
            tyz - twx,
            txz - twy,
            tyz + twx,
            1.0 - (txx + tyy),
        ])
    }

    /// Eigen's quaternion-from-rotation-matrix conversion.
    pub fn from_matrix(m: &Mat3) -> Self {
        let mut t = m.trace();
        if t > 0.0 {
            t = (t + 1.0).sqrt();
            let w = 0.5 * t;
            t = 0.5 / t;
            Self::new(
                w,
                (m[(2, 1)] - m[(1, 2)]) * t,
                (m[(0, 2)] - m[(2, 0)]) * t,
                (m[(1, 0)] - m[(0, 1)]) * t,
            )
        } else {
            let mut i = 0;
            if m[(1, 1)] > m[(0, 0)] {
                i = 1;
            }
            if m[(2, 2)] > m[(i, i)] {
                i = 2;
            }
            let j = (i + 1) % 3;
            let k = (j + 1) % 3;
            t = (m[(i, i)] - m[(j, j)] - m[(k, k)] + 1.0).sqrt();
            let mut v = [0.0; 3];
            v[i] = 0.5 * t;
            t = 0.5 / t;
            let w = (m[(k, j)] - m[(j, k)]) * t;
            v[j] = (m[(j, i)] + m[(i, j)]) * t;
            v[k] = (m[(k, i)] + m[(i, k)]) * t;
            Self::new(w, v[0], v[1], v[2])
        }
    }
}

/// Hamilton product, in Eigen's term order.
impl Mul for Quat {
    type Output = Quat;
    fn mul(self, b: Quat) -> Quat {
        let a = self;
        Quat::new(
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y + a.y * b.w + a.z * b.x - a.x * b.z,
            a.w * b.z + a.z * b.w + a.x * b.y - a.y * b.x,
        )
    }
}

/// `(cos(sqrt(x2)), sinc(sqrt(x2)))`, MTK's `cos_sinc_sqrt`. Below 2^-13 it uses a
/// 3-term Taylor series rather than calling `cos`/`sin`.
pub fn cos_sinc_sqrt(x2: f64) -> (f64, f64) {
    const TAYLOR_N_BOUND: f64 = 1.220_703_125e-4; // sqrt(sqrt(f64::EPSILON))
    const INV: [f64; 7] = [
        1. / 3.,
        1. / 4.,
        1. / 5.,
        1. / 6.,
        1. / 7.,
        1. / 8.,
        1. / 9.,
    ];
    if x2 >= TAYLOR_N_BOUND {
        let x = x2.sqrt();
        return (x.cos(), x.sin() / x);
    }
    let (mut cosi, mut sinc) = (1.0, 1.0);
    let mut term = -0.5 * x2;
    for i in 0..3 {
        cosi += term;
        term *= INV[2 * i];
        sinc += term;
        term *= -INV[2 * i + 1] * x2;
    }
    (cosi, sinc)
}

/// MTK's `exp(result, vec, scale)`: the unit quaternion
/// `(cos(scale·|v|), sinc(scale·|v|)·scale·v)`.
pub fn mtk_exp(v: &Vec3, scale: f64) -> Quat {
    let (c, s) = cos_sinc_sqrt(scale * scale * v.squared_norm());
    let mult = s * scale;
    Quat::new(c, mult * v[0], mult * v[1], mult * v[2])
}

/// `SO3::exp(v, scale)`: the rotation by `scale·v` (axis-angle).
pub fn exp(v: &Vec3, scale: f64) -> Quat {
    mtk_exp(v, scale / 2.0)
}

/// `SO3::log`: the rotation vector of `q`. `q` and `-q` give the same result.
pub fn log(q: &Quat) -> Vec3 {
    let v = q.vec();
    let nv = v.norm().max(TOLERANCE);
    let s = 2.0 / nv * (nv / q.w).atan();
    v * s
}

/// MTK's `A_matrix`: the right Jacobian of SO(3), transposed.
pub fn a_matrix(v: &Vec3) -> Mat3 {
    let squared_norm = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    let norm = squared_norm.sqrt();
    if norm < TOLERANCE {
        return Mat3::identity();
    }
    let h = v.hat();
    Mat3::identity()
        + h * ((1.0 - norm.cos()) / squared_norm)
        + h * h * ((1.0 - norm.sin() / norm) / squared_norm)
}
