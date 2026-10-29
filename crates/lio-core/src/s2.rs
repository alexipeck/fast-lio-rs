//! The 2-sphere manifold FAST-LIO uses for gravity: a 3-vector of fixed length
//! with 2 degrees of freedom.
//!
//! This follows MTK's `S2<double, 98090, 10000, 1>` (`mtk/types/S2.hpp`). The
//! length is 9.809, and the tangent basis is built around the +x axis
//! (`S2_typ == 1`).

use crate::mat::{Mat, Mat3, Vec2, Vec3};
use crate::so3::{self, TOLERANCE};

/// 98090 / 10000. FAST-LIO uses 9.81 elsewhere; the mismatch is part of the algorithm.
pub const LENGTH: f64 = 98090.0 / 10000.0;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct S2 {
    pub vec: Vec3,
}

impl Default for S2 {
    fn default() -> Self {
        Self {
            vec: Vec3::new(1.0, 0.0, 0.0) * LENGTH,
        }
    }
}

impl S2 {
    /// Rescales `v` to length 9.809.
    pub fn new(v: &Vec3) -> Self {
        Self {
            vec: v.normalized() * LENGTH,
        }
    }

    pub fn hat(&self) -> Mat3 {
        self.vec.hat()
    }

    /// `S2_Bx`: 3×2 basis of the tangent plane at `vec`.
    pub fn bx(&self) -> Mat<3, 2> {
        let v = &self.vec;
        let l = LENGTH;
        if v[0] + l > TOLERANCE {
            let d = l + v[0];
            Mat::from_row_slice(&[
                -v[1],
                -v[2],
                l - v[1] * v[1] / d,
                -v[2] * v[1] / d,
                -v[2] * v[1] / d,
                l - v[2] * v[2] / d,
            ]) / l
        } else {
            let mut res = Mat::zeros();
            res[(1, 1)] = -1.0;
            res[(2, 0)] = 1.0;
            res
        }
    }

    pub fn boxplus(&mut self, delta: &Vec2, scale: f64) {
        let bu = self.bx() * *delta;
        let r = so3::mtk_exp(&bu, scale / 2.0);
        self.vec = r.to_matrix() * self.vec;
    }

    pub fn boxminus(&self, other: &S2) -> Vec2 {
        let v_sin = (self.vec.hat() * other.vec).norm();
        let v_cos = self.vec.dot(&other.vec);
        let theta = v_sin.atan2(v_cos);
        if v_sin < TOLERANCE {
            // MTK's truncated constant, kept so antipodal vectors give the same result.
            #[allow(clippy::approx_constant)]
            const MTK_PI: f64 = 3.1415926;
            if theta.abs() > TOLERANCE {
                Vec2::from_array([MTK_PI, 0.0])
            } else {
                Vec2::zeros()
            }
        } else {
            other.bx().transpose() * (theta / v_sin) * other.vec.hat() * self.vec
        }
    }

    /// Rotates by `exp(scale·delta)`, where `delta` is a 3-vector. Used by the predict step.
    pub fn oplus(&mut self, delta: &Vec3, scale: f64) {
        let r = so3::mtk_exp(delta, scale / 2.0);
        self.vec = r.to_matrix() * self.vec;
    }

    /// `S2_Nx_yy`.
    pub fn nx_yy(&self) -> Mat<2, 3> {
        self.bx().transpose() * (1.0 / LENGTH / LENGTH) * self.vec.hat()
    }

    /// `S2_Mx`.
    pub fn mx(&self, delta: &Vec2) -> Mat<3, 2> {
        let bx = self.bx();
        if delta.norm() < TOLERANCE {
            -self.vec.hat() * bx
        } else {
            // MTK writes exp(Bu, scalar(1/2)) here. Integer division makes the
            // scale 0, so that rotation is exactly the identity and is left out.
            let bu = bx * *delta;
            -self.vec.hat() * so3::a_matrix(&bu).transpose() * bx
        }
    }
}
