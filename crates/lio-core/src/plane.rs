//! Plane fitting to a point's nearest neighbours (`esti_plane` in
//! `include/common_lib.h`).
//!
//! FAST-LIO solves `A·n = −1` for the plane `n·p + 1 = 0` in `f32`, using
//! Eigen's column-pivoting Householder QR. That solver is reproduced here step
//! for step, including pivot choice, the column-norm downdate and the rank
//! cut-off, so rank-deficient neighbourhoods behave as they do in FAST-LIO.

use crate::point::Point;

/// Neighbours per plane (`NUM_MATCH_POINTS`).
pub const NUM_MATCH_POINTS: usize = 5;

const ROWS: usize = NUM_MATCH_POINTS;
const COLS: usize = 3;

/// Least-squares solution of `a·x = b` by Eigen's `ColPivHouseholderQR`, in `f32`.
fn colpiv_qr_solve(mut qr: [[f32; COLS]; ROWS], b: [f32; ROWS]) -> [f32; COLS] {
    let eps = f32::EPSILON;
    let col_norm = |qr: &[[f32; COLS]; ROWS], c: usize, from: usize| {
        (from..ROWS)
            .map(|r| qr[r][c] * qr[r][c])
            .sum::<f32>()
            .sqrt()
    };

    let mut norms_direct = [0.0f32; COLS];
    for (c, n) in norms_direct.iter_mut().enumerate() {
        *n = col_norm(&qr, c, 0);
    }
    let mut norms_updated = norms_direct;
    let max_norm = norms_updated.iter().copied().fold(f32::MIN, f32::max);
    let threshold_helper = (max_norm * eps) * (max_norm * eps) / ROWS as f32;
    let norm_downdate_threshold = eps.sqrt();

    let mut h_coeffs = [0.0f32; COLS];
    let mut transpositions = [0usize; COLS];
    let mut nonzero_pivots = COLS;

    for k in 0..COLS {
        // First index of the largest remaining column norm (Eigen's maxCoeff).
        let mut biggest = k;
        for j in k + 1..COLS {
            if norms_updated[j] > norms_updated[biggest] {
                biggest = j;
            }
        }
        let biggest_sq = norms_updated[biggest] * norms_updated[biggest];
        if nonzero_pivots == COLS && biggest_sq < threshold_helper * (ROWS - k) as f32 {
            nonzero_pivots = k;
        }
        transpositions[k] = biggest;
        if k != biggest {
            for row in qr.iter_mut() {
                row.swap(k, biggest);
            }
            norms_updated.swap(k, biggest);
            norms_direct.swap(k, biggest);
        }

        // makeHouseholderInPlace on column k, rows k..
        let c0 = qr[k][k];
        let tail_sq: f32 = (k + 1..ROWS).map(|r| qr[r][k] * qr[r][k]).sum();
        let (tau, beta) = if tail_sq <= f32::MIN_POSITIVE {
            for row in qr.iter_mut().skip(k + 1) {
                row[k] = 0.0;
            }
            (0.0, c0)
        } else {
            let mut beta = (c0 * c0 + tail_sq).sqrt();
            if c0 >= 0.0 {
                beta = -beta;
            }
            for row in qr.iter_mut().skip(k + 1) {
                row[k] /= c0 - beta;
            }
            ((beta - c0) / beta, beta)
        };
        h_coeffs[k] = tau;
        qr[k][k] = beta;

        // applyHouseholderOnTheLeft to the columns right of k.
        for j in k + 1..COLS {
            apply_householder_col(&mut qr, k, j, tau);
        }

        for j in k + 1..COLS {
            if norms_updated[j] != 0.0 {
                let mut temp = qr[k][j].abs() / norms_updated[j];
                temp = (1.0 + temp) * (1.0 - temp);
                temp = temp.max(0.0);
                let ratio = norms_updated[j] / norms_direct[j];
                if temp * (ratio * ratio) <= norm_downdate_threshold {
                    norms_direct[j] = col_norm(&qr, j, k + 1);
                    norms_updated[j] = norms_direct[j];
                } else {
                    norms_updated[j] *= temp.sqrt();
                }
            }
        }
    }

    let mut perm = [0usize, 1, 2];
    for (k, &t) in transpositions.iter().enumerate() {
        perm.swap(k, t);
    }

    let mut x = [0.0f32; COLS];
    if nonzero_pivots == 0 {
        return x;
    }
    // c = Qᵀ·b, applying H_0 first.
    let mut c = b;
    for k in 0..nonzero_pivots {
        let essential: Vec<f32> = (k + 1..ROWS).map(|r| qr[r][k]).collect();
        apply_householder_vec(&mut c, k, &essential, h_coeffs[k]);
    }
    // Upper-triangular back substitution, column-oriented as Eigen does it.
    for i in (0..nonzero_pivots).rev() {
        if c[i] != 0.0 {
            c[i] /= qr[i][i];
            for s in 0..i {
                c[s] -= c[i] * qr[s][i];
            }
        }
    }
    for i in 0..nonzero_pivots {
        x[perm[i]] = c[i];
    }
    x
}

/// Applies the reflector stored below the diagonal of column `k` to rows `k..`
/// of column `j`.
fn apply_householder_col(qr: &mut [[f32; COLS]; ROWS], k: usize, j: usize, tau: f32) {
    if ROWS - k == 1 {
        qr[k][j] *= 1.0 - tau;
    } else if tau != 0.0 {
        let mut tmp: f32 = (k + 1..ROWS).map(|r| qr[r][k] * qr[r][j]).sum();
        tmp += qr[k][j];
        qr[k][j] -= tau * tmp;
        for row in qr.iter_mut().skip(k + 1) {
            row[j] -= tau * row[k] * tmp;
        }
    }
}

fn apply_householder_vec(c: &mut [f32; ROWS], k: usize, essential: &[f32], tau: f32) {
    if ROWS - k == 1 {
        c[k] *= 1.0 - tau;
    } else if tau != 0.0 {
        let mut tmp: f32 = essential.iter().zip(&c[k + 1..]).map(|(e, v)| e * v).sum();
        tmp += c[k];
        c[k] -= tau * tmp;
        for (v, e) in c[k + 1..].iter_mut().zip(essential) {
            *v -= tau * e * tmp;
        }
    }
}

/// Fits a plane `a·x + b·y + c·z + d = 0`, with unit normal `(a, b, c)`, to the
/// first [`NUM_MATCH_POINTS`] points. Returns `None` if any of them lies further
/// than `threshold` from it.
pub fn esti_plane(points: &[Point], threshold: f32) -> Option<[f32; 4]> {
    let mut a = [[0.0f32; COLS]; ROWS];
    for (row, p) in a.iter_mut().zip(points) {
        *row = [p.x, p.y, p.z];
    }
    let normvec = colpiv_qr_solve(a, [-1.0; ROWS]);
    let n = (normvec[0] * normvec[0] + normvec[1] * normvec[1] + normvec[2] * normvec[2]).sqrt();
    let pca = [normvec[0] / n, normvec[1] / n, normvec[2] / n, 1.0 / n];
    // Written as "no point exceeds" rather than "all within" so a NaN plane is
    // accepted, as in the C++.
    let rejected = points[..NUM_MATCH_POINTS]
        .iter()
        .any(|p| (pca[0] * p.x + pca[1] * p.y + pca[2] * p.z + pca[3]).abs() > threshold);
    (!rejected).then_some(pca)
}
