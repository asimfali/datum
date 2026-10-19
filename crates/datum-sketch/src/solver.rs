//! Damped minimum-norm Gauss–Newton and rank by QR (D54).
//!
//! After `cadcraft-constraints` (`lm.rs`; MIT OR Apache-2.0, © ArtCraft Team and the CADCraft
//! contributors), rewritten for exact sparse gradients. Each step solves
//! `min ‖Δx‖` subject to `J Δx = −r`, damped: `Δx = −Jᵀ (J Jᵀ + μ I)⁻¹ r`. An under-constrained
//! system therefore moves as little as possible from where it starts. Everything runs in a fixed
//! order: the same input gives the same result bit for bit.

/// Residual function: fills the residuals and, when asked, their gradients as sparse rows
/// `(coordinate, derivative)`; a coordinate may repeat in a row, its entries add.
pub(crate) type Residuals<'a> = dyn Fn(&[f64], Option<&mut Vec<Vec<(usize, f64)>>>, &mut Vec<f64>) + 'a;

pub(crate) enum Fit {
    Converged(Vec<f64>),
    Failed,
}

/// Iterations before giving up.
const MAX_ITER: usize = 200;
/// Steps after reaching the tolerance.
const POLISH: usize = 3;

fn max_abs(v: &[f64]) -> f64 {
    v.iter().fold(0.0f64, |m, a| if a.is_finite() { m.max(a.abs()) } else { f64::INFINITY })
}

fn norm2(v: &[f64]) -> f64 {
    v.iter().map(|a| a * a).sum()
}

/// Dense `rows × free` Jacobian (row-major) from sparse rows; `column[c]` is the position of
/// coordinate `c` among the free ones.
fn dense(rows: &[Vec<(usize, f64)>], column: &[Option<usize>], nf: usize) -> Vec<f64> {
    let mut j = vec![0.0; rows.len() * nf];
    for (i, row) in rows.iter().enumerate() {
        for &(c, d) in row {
            if let Some(Some(k)) = column.get(c) {
                j[i * nf + k] += d;
            }
        }
    }
    j
}

fn columns(free: &[usize], n: usize) -> Vec<Option<usize>> {
    let mut column = vec![None; n];
    for (k, &c) in free.iter().enumerate() {
        column[c] = Some(k);
    }
    column
}

/// Moves the `free` coordinates of `x0` until every residual is at most `tol`.
pub(crate) fn solve(f: &Residuals, x0: &[f64], free: &[usize], tol: f64) -> Fit {
    let mut x = x0.to_vec();
    let mut r = Vec::new();
    f(&x, None, &mut r);
    let rows = r.len();
    if max_abs(&r) <= tol {
        return Fit::Converged(x);
    }
    if free.is_empty() {
        return Fit::Failed;
    }
    let (nf, column) = (free.len(), columns(free, x.len()));
    let mut mu = 1e-10;
    let mut cost = norm2(&r);
    let (mut grads, mut rn) = (Vec::new(), Vec::with_capacity(rows));
    // Once within tolerance, a few more steps polish while they still help: convergence is
    // quadratic, so the result ends near rounding, not near the tolerance.
    let mut polish = POLISH;
    for _ in 0..MAX_ITER {
        f(&x, Some(&mut grads), &mut r);
        let jac = dense(&grads, &column, nf);
        // A = J Jᵀ
        let mut a = vec![0.0; rows * rows];
        for i in 0..rows {
            for k in i..rows {
                let s: f64 = (0..nf).map(|c| jac[i * nf + c] * jac[k * nf + c]).sum();
                a[i * rows + k] = s;
                a[k * rows + i] = s;
            }
        }
        let diag_max = (0..rows).map(|i| a[i * rows + i]).fold(0.0f64, f64::max).max(1e-300);
        let mut improved = false;
        for _ in 0..12 {
            let mut ad = a.clone();
            for i in 0..rows {
                ad[i * rows + i] += mu * diag_max;
            }
            let Some(y) = solve_dense(ad, r.clone(), rows) else {
                mu *= 10.0;
                continue;
            };
            let mut xn = x.clone();
            for (c, &j) in free.iter().enumerate() {
                let s: f64 = y.iter().enumerate().map(|(i, yi)| jac[i * nf + c] * yi).sum();
                xn[j] -= s;
            }
            f(&xn, None, &mut rn);
            let cn = norm2(&rn);
            if cn.is_finite() && cn < cost {
                x = xn;
                std::mem::swap(&mut r, &mut rn);
                cost = cn;
                mu = (mu * 0.1).max(1e-15);
                improved = true;
                break;
            }
            mu *= 10.0;
        }
        if max_abs(&r) <= tol {
            if !improved || polish == 0 {
                return Fit::Converged(x);
            }
            polish -= 1;
        } else if !improved {
            break;
        }
    }
    if max_abs(&r) <= tol { Fit::Converged(x) } else { Fit::Failed }
}

/// Solves `a y = b` (`n × n`, row-major) by Gaussian elimination with partial pivoting; `None`
/// when singular.
fn solve_dense(mut a: Vec<f64>, mut b: Vec<f64>, n: usize) -> Option<Vec<f64>> {
    let scale = a.iter().fold(0.0f64, |m, v| m.max(v.abs())).max(1e-300);
    for col in 0..n {
        let piv = (col..n).max_by(|&p, &q| a[p * n + col].abs().total_cmp(&a[q * n + col].abs()).then(q.cmp(&p)))?;
        if a[piv * n + col].abs() <= scale * 1e-15 {
            return None;
        }
        if piv != col {
            for c in 0..n {
                a.swap(piv * n + c, col * n + c);
            }
            b.swap(piv, col);
        }
        let p = a[col * n + col];
        for r in col + 1..n {
            let f = a[r * n + col] / p;
            if f == 0.0 {
                continue;
            }
            for c in col..n {
                a[r * n + c] -= f * a[col * n + c];
            }
            b[r] -= f * b[col];
        }
    }
    let mut y = vec![0.0; n];
    for r in (0..n).rev() {
        let s: f64 = b[r] - (r + 1..n).map(|c| a[r * n + c] * y[c]).sum::<f64>();
        y[r] = s / a[r * n + r];
    }
    y.iter().all(|v| v.is_finite()).then_some(y)
}

/// Rank of the Jacobian over the `free` coordinates, and an orthonormal basis of its null space
/// (vectors over `free`, in its order): the motions the constraints still allow. Rows are scaled
/// to unit length first; a pivot below `rel` times the first one counts as zero.
pub(crate) fn null_space(rows: &[Vec<(usize, f64)>], free: &[usize], rel: f64) -> (usize, Vec<Vec<f64>>) {
    let nf = free.len();
    let n = rows.iter().flatten().map(|e| e.0 + 1).chain(free.iter().map(|c| c + 1)).max().unwrap_or(0);
    let jac = dense(rows, &columns(free, n), nf);
    // QR of Jᵀ: its columns are the Jacobian's rows.
    let mut cols: Vec<Vec<f64>> = (0..rows.len())
        .map(|i| {
            let row = jac[i * nf..(i + 1) * nf].to_vec();
            let len = norm2(&row).sqrt();
            if len > 0.0 { row.iter().map(|v| v / len).collect() } else { row }
        })
        .collect();
    let mut reflectors: Vec<Vec<f64>> = Vec::new();
    let mut first = None;
    for k in 0..nf.min(cols.len()) {
        // Pivot: the column with the largest remaining norm, the lowest index on ties.
        let Some((best, norm)) = (k..cols.len()).map(|j| (j, norm2(&cols[j][k..]).sqrt())).max_by(|a, b| a.1.total_cmp(&b.1).then(b.0.cmp(&a.0)))
        else {
            break;
        };
        if norm <= first.unwrap_or(0.0) * rel || norm == 0.0 {
            break;
        }
        first.get_or_insert(norm);
        cols.swap(k, best);
        let mut v = vec![0.0; nf];
        v[k..].copy_from_slice(&cols[k][k..]);
        v[k] -= if v[k] >= 0.0 { -norm } else { norm };
        let vv = norm2(&v);
        if vv == 0.0 {
            break;
        }
        for c in cols.iter_mut().skip(k) {
            reflect(&v, vv, c);
        }
        reflectors.push(v);
    }
    let rank = reflectors.len();
    // The last `nf − rank` columns of Q = H₀ H₁ … span the null space.
    let basis = (rank..nf)
        .map(|j| {
            let mut e = vec![0.0; nf];
            e[j] = 1.0;
            for v in reflectors.iter().rev() {
                reflect(v, norm2(v), &mut e);
            }
            e
        })
        .collect();
    (rank, basis)
}

fn reflect(v: &[f64], vv: f64, c: &mut [f64]) {
    let f = 2.0 * v.iter().zip(c.iter()).map(|(a, b)| a * b).sum::<f64>() / vv;
    if f != 0.0 {
        for (ci, vi) in c.iter_mut().zip(v) {
            *ci -= f * vi;
        }
    }
}

#[cfg(test)]
#[allow(clippy::panic)]
mod tests {
    use super::*;

    #[test]
    fn minimum_norm_step() {
        // x + y = 2 from (0, 0): the least motion is to (1, 1).
        let f = |x: &[f64], j: Option<&mut Vec<Vec<(usize, f64)>>>, r: &mut Vec<f64>| {
            r.clear();
            r.push(x[0] + x[1] - 2.0);
            if let Some(j) = j {
                j.clear();
                j.push(vec![(0, 1.0), (1, 1.0)]);
            }
        };
        let Fit::Converged(x) = solve(&f, &[0.0, 0.0], &[0, 1], 1e-12) else { panic!("did not converge") };
        assert!((x[0] - 1.0).abs() < 1e-12 && (x[1] - 1.0).abs() < 1e-12);
        let Fit::Converged(x) = solve(&f, &[0.0, 0.0], &[1], 1e-12) else { panic!("did not converge") };
        assert_eq!(x[0], 0.0);
    }

    #[test]
    fn rank_and_null_space() {
        // x0 + x1 fixed, x2 free.
        let (rank, null) = null_space(&[vec![(0, 1.0), (1, 1.0)]], &[0, 1, 2], 1e-10);
        assert_eq!((rank, null.len()), (1, 2));
        assert!(null.iter().all(|v| (v[0] + v[1]).abs() < 1e-12));
        let (rank, _) = null_space(&[vec![(0, 1.0)], vec![(0, 2.0)]], &[0], 1e-10);
        assert_eq!(rank, 1);
    }
}
