//! Exact signs of polynomial expressions in `f64` inputs (D39).
//!
//! An expression is written once, generically over [`Ring`], and evaluated twice at most: first
//! as [`Approx`], `f64` with a running error bound, which settles almost every case; then, only
//! when the bound does not exclude zero, exactly on integers.

use std::ops::{Add, Mul, Sub};
use std::sync::atomic::{AtomicU64, Ordering};

use num_bigint::BigInt;

use crate::Sign;
use crate::implicit::Lpi;

/// The arithmetic an expression is evaluated in.
pub(crate) trait Ring: Clone + Add<Output = Self> + Sub<Output = Self> + Mul<Output = Self> {
    fn one() -> Self;

    /// Homogeneous coordinates of an implicit point. The filter reads the copy computed once when
    /// the point was made; the exact path computes them.
    fn lpi(l: &Lpi, lift: &impl Fn(f64) -> Self) -> [Self; 4] {
        l.homogeneous(lift)
    }
}

/// A polynomial in `f64` inputs whose sign a predicate needs.
pub(crate) trait Expr {
    /// Every input value: the exact path scales them all by one common power of two.
    fn inputs(&self) -> impl Iterator<Item = f64>;
    /// The expression, with each input mapped through `lift`.
    fn eval<T: Ring>(&self, lift: &impl Fn(f64) -> T) -> T;
}

/// Exact sign of `e`.
pub(crate) fn sign(e: &impl Expr) -> Sign {
    filtered(e).unwrap_or_else(|| {
        EXACT_FALLBACKS.fetch_add(1, Ordering::Relaxed);
        exact(e)
    })
}

/// The sign from `f64` evaluation, when the error bound guarantees it.
pub(crate) fn filtered(e: &impl Expr) -> Option<Sign> {
    e.eval(&Approx::input).sign()
}

/// The sign from integer evaluation: always right, and slow.
pub(crate) fn exact(e: &impl Expr) -> Sign {
    let scale = Scale::of(e.inputs());
    match e.eval(&|x| scale.lift(x)).sign() {
        num_bigint::Sign::Minus => Sign::Negative,
        num_bigint::Sign::NoSign => Sign::Zero,
        num_bigint::Sign::Plus => Sign::Positive,
    }
}

static EXACT_FALLBACKS: AtomicU64 = AtomicU64::new(0);

/// How many predicate evaluations so far, process-wide, the `f64` filter could not settle.
/// Benchmarks use it to tell whether the exact path is hot.
pub fn exact_fallbacks() -> u64 {
    EXACT_FALLBACKS.load(Ordering::Relaxed)
}

/// Unit roundoff: one rounding changes a value by at most `U` times its magnitude.
const U: f64 = f64::EPSILON / 2.0;
/// Absolute error of a product that underflows: half the smallest subnormal, rounded up.
const TINY: f64 = f64::from_bits(1);
/// The error bounds are themselves rounded: under 100 operations deep they are off by less than
/// a factor of `1 + 100·U ≈ 1 + 2^-46`, far inside this margin.
const SLACK: f64 = 1.0 + 1.0 / (1u64 << 40) as f64;

/// An `f64` value with a bound on its distance from the exact value of the same expression.
/// Overflow makes the bound infinite or the value NaN, and the sign is then left to the exact path.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Approx {
    v: f64,
    err: f64,
}

impl Approx {
    pub(crate) fn input(v: f64) -> Self {
        Self { v, err: 0.0 }
    }

    pub(crate) fn sign(self) -> Option<Sign> {
        (self.v.abs() > self.err * SLACK).then(|| Sign::of(self.v))
    }
}

impl Add for Approx {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        let v = self.v + o.v;
        Self { v, err: self.err + o.err + U * v.abs() }
    }
}

impl Sub for Approx {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        let v = self.v - o.v;
        Self { v, err: self.err + o.err + U * v.abs() }
    }
}

impl Mul for Approx {
    type Output = Self;
    fn mul(self, o: Self) -> Self {
        let v = self.v * o.v;
        Self { v, err: self.v.abs() * o.err + o.v.abs() * self.err + self.err * o.err + U * v.abs() + TINY }
    }
}

impl Ring for Approx {
    fn one() -> Self {
        Self::input(1.0)
    }

    fn lpi(l: &Lpi, _: &impl Fn(f64) -> Self) -> [Self; 4] {
        l.filter
    }
}

impl Ring for BigInt {
    fn one() -> Self {
        BigInt::from(1)
    }
}

/// Maps finite `f64` values to integers, all multiplied by the same power of two `2^-shift`.
/// Each `f64` is `±m·2^e` with integer `m`, so the map is exact; and the common positive factor
/// scales every point uniformly, which leaves the sign of every predicate unchanged.
struct Scale {
    shift: i32,
}

impl Scale {
    fn of(values: impl Iterator<Item = f64>) -> Self {
        Self { shift: values.filter(|v| *v != 0.0).map(|v| decompose(v).2).min().unwrap_or(0) }
    }

    fn lift(&self, v: f64) -> BigInt {
        let (negative, m, e) = decompose(v);
        // `e >= shift` for every input, and the difference is under 2100 bits.
        let n = BigInt::from(m) << (e - self.shift) as usize;
        if negative { -n } else { n }
    }
}

/// `v = ±m·2^e`, with `m` odd (or zero).
fn decompose(v: f64) -> (bool, u64, i32) {
    debug_assert!(v.is_finite(), "predicate input {v} is not finite");
    let bits = v.to_bits();
    let biased = ((bits >> 52) & 0x7ff) as i32;
    let fraction = bits & ((1 << 52) - 1);
    let (m, e) = if biased == 0 { (fraction, -1074) } else { (fraction | 1 << 52, biased - 1075) };
    if m == 0 {
        return (false, 0, 0);
    }
    let tz = m.trailing_zeros();
    (bits >> 63 == 1, m >> tz, e + tz as i32)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decompose_is_exact() {
        for v in [1.0, -0.75, 3.0e300, 5e-324, -2.2250738585072014e-308, 123456.789, 0.1] {
            let (neg, m, e) = decompose(v);
            let back = m as f64 * 2f64.powi(e.max(-1022)) * 2f64.powi((e + 1022).min(0));
            assert_eq!(if neg { -back } else { back }, v, "{v}");
        }
        assert_eq!(decompose(0.0), (false, 0, 0));
    }

    #[test]
    fn filter_defers_cancellation_to_exact() {
        // (1 + 2^-60) - 1 rounds to zero in f64: the filter must not claim a sign.
        let a = Approx::input(1.0) + Approx::input(2f64.powi(-60));
        assert_eq!((a - Approx::input(1.0)).sign(), None);
        assert_eq!((Approx::input(3.0) * Approx::input(2.0) - Approx::input(5.0)).sign(), Some(Sign::Positive));
    }
}
