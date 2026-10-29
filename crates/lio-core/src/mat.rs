//! Small fixed-size dense matrices of `f64`, stored row-major.
//!
//! Every matrix in the filter has a size known at compile time (23×23
//! covariance, 24×23 Jacobian, 3×3 rotation), so const generics cover them
//! all without a linear-algebra dependency.

use std::ops::{Add, AddAssign, Div, Index, IndexMut, Mul, Neg, Sub, SubAssign};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mat<const R: usize, const C: usize>(pub [[f64; C]; R]);

pub type Vec2 = Mat<2, 1>;
pub type Vec3 = Mat<3, 1>;
pub type Mat3 = Mat<3, 3>;

impl<const R: usize, const C: usize> Mat<R, C> {
    pub const fn zeros() -> Self {
        Self([[0.0; C]; R])
    }

    /// Ones on the main diagonal, also for non-square shapes.
    pub fn identity() -> Self {
        let mut m = Self::zeros();
        for i in 0..R.min(C) {
            m.0[i][i] = 1.0;
        }
        m
    }

    pub fn from_fn(f: impl Fn(usize, usize) -> f64) -> Self {
        let mut m = Self::zeros();
        for r in 0..R {
            for c in 0..C {
                m.0[r][c] = f(r, c);
            }
        }
        m
    }

    /// Row-major elements, as the golden data stores them.
    pub fn from_row_slice(v: &[f64]) -> Self {
        assert_eq!(v.len(), R * C, "expected {R}x{C} values");
        Self::from_fn(|r, c| v[r * C + c])
    }

    pub fn to_row_vec(&self) -> Vec<f64> {
        self.0.iter().flatten().copied().collect()
    }

    pub fn transpose(&self) -> Mat<C, R> {
        Mat::from_fn(|r, c| self.0[c][r])
    }

    pub fn block<const BR: usize, const BC: usize>(&self, r: usize, c: usize) -> Mat<BR, BC> {
        Mat::from_fn(|i, j| self.0[r + i][c + j])
    }

    pub fn set_block<const BR: usize, const BC: usize>(
        &mut self,
        r: usize,
        c: usize,
        b: &Mat<BR, BC>,
    ) {
        for i in 0..BR {
            self.0[r + i][c..c + BC].copy_from_slice(&b.0[i]);
        }
    }
}

impl<const R: usize, const C: usize> Default for Mat<R, C> {
    fn default() -> Self {
        Self::zeros()
    }
}

impl<const N: usize> Mat<N, N> {
    pub fn trace(&self) -> f64 {
        (0..N).map(|i| self.0[i][i]).sum()
    }

    /// Inverse by LU decomposition with partial pivoting, the method Eigen's
    /// `inverse()` uses above 4×4. A singular matrix gives non-finite entries,
    /// as in Eigen.
    pub fn inverse(&self) -> Self {
        let mut a: Vec<f64> = self.to_row_vec();
        lu_inverse_in_place(&mut a, N);
        Self::from_row_slice(&a)
    }
}

/// Inverts the row-major `n`×`n` matrix in `a` in place (LU, partial pivoting).
pub fn lu_inverse_in_place(a: &mut [f64], n: usize) {
    let mut lu = a.to_vec();
    let mut perm: Vec<usize> = (0..n).collect();
    for k in 0..n {
        let p = (k..n).fold(k, |best, i| {
            if lu[i * n + k].abs() > lu[best * n + k].abs() {
                i
            } else {
                best
            }
        });
        if p != k {
            for j in 0..n {
                lu.swap(k * n + j, p * n + j);
            }
            perm.swap(k, p);
        }
        let pivot = lu[k * n + k];
        for i in k + 1..n {
            let f = lu[i * n + k] / pivot;
            lu[i * n + k] = f;
            for j in k + 1..n {
                lu[i * n + j] -= f * lu[k * n + j];
            }
        }
    }
    for col in 0..n {
        // Solve L·U·x = P·e_col.
        let mut x: Vec<f64> = (0..n)
            .map(|i| if perm[i] == col { 1.0 } else { 0.0 })
            .collect();
        for i in 0..n {
            let s: f64 = (0..i).map(|j| lu[i * n + j] * x[j]).sum();
            x[i] -= s;
        }
        for i in (0..n).rev() {
            let s: f64 = (i + 1..n).map(|j| lu[i * n + j] * x[j]).sum();
            x[i] = (x[i] - s) / lu[i * n + i];
        }
        for i in 0..n {
            a[i * n + col] = x[i];
        }
    }
}

impl<const R: usize> Mat<R, 1> {
    pub fn from_array(v: [f64; R]) -> Self {
        Self(v.map(|x| [x]))
    }

    pub fn dot(&self, o: &Self) -> f64 {
        (0..R).map(|i| self.0[i][0] * o.0[i][0]).sum()
    }

    pub fn squared_norm(&self) -> f64 {
        self.dot(self)
    }

    pub fn norm(&self) -> f64 {
        self.squared_norm().sqrt()
    }

    /// Eigen's `normalize()`: divide by the norm unless it is zero.
    pub fn normalized(&self) -> Self {
        let z = self.squared_norm();
        if z > 0.0 { *self / z.sqrt() } else { *self }
    }
}

impl Vec3 {
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self([[x], [y], [z]])
    }

    pub fn cross(&self, o: &Self) -> Self {
        let (a, b) = (self, o);
        Vec3::new(
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        )
    }

    /// Skew-symmetric matrix with `v.hat() * u == v.cross(u)` (MTK's `hat`).
    pub fn hat(&self) -> Mat3 {
        let v = self;
        Mat([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
    }
}

impl<const R: usize, const C: usize> Index<(usize, usize)> for Mat<R, C> {
    type Output = f64;
    fn index(&self, (r, c): (usize, usize)) -> &f64 {
        &self.0[r][c]
    }
}

impl<const R: usize, const C: usize> IndexMut<(usize, usize)> for Mat<R, C> {
    fn index_mut(&mut self, (r, c): (usize, usize)) -> &mut f64 {
        &mut self.0[r][c]
    }
}

impl<const R: usize> Index<usize> for Mat<R, 1> {
    type Output = f64;
    fn index(&self, i: usize) -> &f64 {
        &self.0[i][0]
    }
}

impl<const R: usize> IndexMut<usize> for Mat<R, 1> {
    fn index_mut(&mut self, i: usize) -> &mut f64 {
        &mut self.0[i][0]
    }
}

impl<const R: usize, const C: usize> Add for Mat<R, C> {
    type Output = Self;
    fn add(mut self, o: Self) -> Self {
        self += o;
        self
    }
}

impl<const R: usize, const C: usize> AddAssign for Mat<R, C> {
    fn add_assign(&mut self, o: Self) {
        for (ra, rb) in self.0.iter_mut().zip(&o.0) {
            for c in 0..C {
                ra[c] += rb[c];
            }
        }
    }
}

impl<const R: usize, const C: usize> Sub for Mat<R, C> {
    type Output = Self;
    fn sub(mut self, o: Self) -> Self {
        self -= o;
        self
    }
}

impl<const R: usize, const C: usize> SubAssign for Mat<R, C> {
    fn sub_assign(&mut self, o: Self) {
        for (ra, rb) in self.0.iter_mut().zip(&o.0) {
            for c in 0..C {
                ra[c] -= rb[c];
            }
        }
    }
}

impl<const R: usize, const C: usize> Neg for Mat<R, C> {
    type Output = Self;
    fn neg(mut self) -> Self {
        for row in &mut self.0 {
            for a in row {
                *a = -*a;
            }
        }
        self
    }
}

impl<const R: usize, const C: usize> Mul<f64> for Mat<R, C> {
    type Output = Self;
    fn mul(mut self, s: f64) -> Self {
        for row in &mut self.0 {
            for a in row {
                *a *= s;
            }
        }
        self
    }
}

impl<const R: usize, const C: usize> Mul<Mat<R, C>> for f64 {
    type Output = Mat<R, C>;
    fn mul(self, m: Mat<R, C>) -> Mat<R, C> {
        m * self
    }
}

impl<const R: usize, const C: usize> Div<f64> for Mat<R, C> {
    type Output = Self;
    fn div(mut self, s: f64) -> Self {
        for row in &mut self.0 {
            for a in row {
                *a /= s;
            }
        }
        self
    }
}

impl<const R: usize, const C: usize, const K: usize> Mul<Mat<C, K>> for Mat<R, C> {
    type Output = Mat<R, K>;
    fn mul(self, o: Mat<C, K>) -> Mat<R, K> {
        let mut out = Mat::<R, K>::zeros();
        for r in 0..R {
            for k in 0..C {
                let a = self.0[r][k];
                for c in 0..K {
                    out.0[r][c] += a * o.0[k][c];
                }
            }
        }
        out
    }
}
