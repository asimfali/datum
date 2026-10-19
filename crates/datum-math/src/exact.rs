//! Exact signs of polynomial expressions in `f64` inputs (D39), with symbolic perturbation (Q7).
//!
//! An expression is written once, generically over [`Ring`], and evaluated twice at most: first
//! as [`Approx`], `f64` with a running error bound, which settles almost every case; then, only
//! when the bound does not exclude zero, exactly on integers.
//!
//! When the exact sign is zero (a contact: a vertex on a face, an edge through an edge) and some
//! inputs belong to a moving shell, the expression is evaluated again for the configuration moved
//! by an infinitesimal `ε`: every moving point `p` of a shell with centre `c` goes to
//! `p + ε·(p − c) + ε²·SHIFT`. The shell grows a little and shifts in a fixed generic direction.
//! The value becomes a polynomial in `ε`, and its first non-zero coefficient gives the sign: one
//! consistent answer for every contact, without tolerances.

use std::ops::{Add, Mul, Sub};
use std::sync::atomic::{AtomicU64, Ordering};

use num_bigint::BigInt;

use crate::implicit::Lpi;
use crate::{Sign, Vec3};

/// The second-order shift of moving shells: a fixed direction with no simple relation to the
/// axes or to round numbers, so that it breaks what growth alone leaves tied.
const SHIFT: [f64; 3] = [0.754_877_666_246_692_7, 0.569_840_290_998_053_2, 0.330_235_127_221_684_3];
#[cfg(test)]
pub(crate) const SHIFT_FOR_TESTS: [f64; 3] = SHIFT;

/// The arithmetic an expression is evaluated in.
pub(crate) trait Ring: Clone + Add<Output = Self> + Sub<Output = Self> + Mul<Output = Self> {
    fn zero() -> Self;
    fn one() -> Self;

    /// Homogeneous coordinates of an implicit point. The filter reads the copy computed once when
    /// the point was made; the other arithmetics compute them.
    fn lpi(l: &Lpi, lift: &impl Lift<Self>) -> [Self; 4] {
        l.homogeneous(lift)
    }
}

/// Maps an input point, with the centre of its shell if it moves, into an arithmetic.
pub(crate) trait Lift<T> {
    fn point(&self, p: Vec3, centre: Option<Vec3>) -> [T; 3];
}

/// Points where they are: the unperturbed configuration.
pub(crate) struct Still<F>(pub(crate) F);

impl<T, F: Fn(f64) -> T> Lift<T> for Still<F> {
    fn point(&self, p: Vec3, _: Option<Vec3>) -> [T; 3] {
        p.to_array().map(&self.0)
    }
}

/// Points moved by `ε·(p − c) + ε²·SHIFT`, as polynomials in `ε` truncated after `ε^(N−1)`.
struct Moved<F, const N: usize>(F);

impl<T: Ring, F: Fn(f64) -> T, const N: usize> Lift<Poly<T, N>> for Moved<F, N> {
    fn point(&self, p: Vec3, centre: Option<Vec3>) -> [Poly<T, N>; 3] {
        let (p, c) = (p.to_array(), centre.map(Vec3::to_array));
        [0, 1, 2].map(|i| {
            let mut k: [T; N] = std::array::from_fn(|_| T::zero());
            k[0] = (self.0)(p[i]);
            if let Some(c) = c {
                if N > 1 {
                    k[1] = (self.0)(p[i]) - (self.0)(c[i]);
                }
                if N > 2 {
                    k[2] = (self.0)(SHIFT[i]);
                }
            }
            Poly(k)
        })
    }
}

/// A polynomial in `ε` with coefficients `0 .. N`; products drop higher powers, so the
/// coefficients kept are exact.
#[derive(Clone)]
pub(crate) struct Poly<T, const N: usize>([T; N]);

impl<T: Ring, const N: usize> Add for Poly<T, N> {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        let mut a = self.0;
        for (x, y) in a.iter_mut().zip(o.0) {
            *x = x.clone() + y;
        }
        Self(a)
    }
}

impl<T: Ring, const N: usize> Sub for Poly<T, N> {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        let mut a = self.0;
        for (x, y) in a.iter_mut().zip(o.0) {
            *x = x.clone() - y;
        }
        Self(a)
    }
}

impl<T: Ring, const N: usize> Mul for Poly<T, N> {
    type Output = Self;
    fn mul(self, o: Self) -> Self {
        let mut r: [T; N] = std::array::from_fn(|_| T::zero());
        for (i, x) in self.0.iter().enumerate() {
            for (j, y) in o.0.iter().take(N - i).enumerate() {
                r[i + j] = r[i + j].clone() + x.clone() * y.clone();
            }
        }
        Self(r)
    }
}

impl<T: Ring, const N: usize> Ring for Poly<T, N> {
    fn zero() -> Self {
        Self(std::array::from_fn(|_| T::zero()))
    }

    fn one() -> Self {
        let mut k: [T; N] = std::array::from_fn(|_| T::zero());
        k[0] = T::one();
        Self(k)
    }
}

/// A polynomial in `f64` inputs whose sign a predicate needs.
pub(crate) trait Expr {
    /// Every input value, centres of moving shells included: the exact path scales them all by
    /// one common power of two.
    fn inputs(&self) -> impl Iterator<Item = f64>;
    /// Whether some input moves under the perturbation.
    fn moves(&self) -> bool;
    /// The expression, with each input point mapped through `lift`.
    fn eval<T: Ring>(&self, lift: &impl Lift<T>) -> T;
}

/// Exact sign of `e`; a zero with moving inputs is settled by the perturbation.
pub(crate) fn sign(e: &impl Expr) -> Sign {
    let s = filtered(e).unwrap_or_else(|| {
        EXACT_FALLBACKS.fetch_add(1, Ordering::Relaxed);
        exact(e)
    });
    if s == Sign::Zero && e.moves() { perturbed(e) } else { s }
}

/// The sign from `f64` evaluation, when the error bound guarantees it.
pub(crate) fn filtered(e: &impl Expr) -> Option<Sign> {
    e.eval(&Still(Approx::input)).sign()
}

/// The sign from integer evaluation: always right, and slow.
pub(crate) fn exact(e: &impl Expr) -> Sign {
    let scale = Scale::of(e.inputs());
    big_sign(&e.eval(&Still(|x| scale.lift(x))))
}

fn big_sign(v: &BigInt) -> Sign {
    match v.sign() {
        num_bigint::Sign::Minus => Sign::Negative,
        num_bigint::Sign::NoSign => Sign::Zero,
        num_bigint::Sign::Plus => Sign::Positive,
    }
}

/// The sign under the perturbation, for an expression whose value is exactly zero. The `ε`
/// coefficient is tried by the filter (the constant one is known to vanish); otherwise exact
/// coefficients are computed to growing powers. Coordinates are quadratic in `ε` and predicates
/// at most of degree 12 in coordinates, so 25 coefficients hold the whole polynomial: zero there
/// is zero for every motion.
fn perturbed(e: &impl Expr) -> Sign {
    PERTURBED.fetch_add(1, Ordering::Relaxed);
    TIES.with(|t| t.set(t.get() + 1));
    if let Some(s) = e.eval(&Moved::<_, 2>(Approx::input)).0[1].sign() {
        return s;
    }
    // The shift is an input of the moved configuration too.
    let scale = Scale::of(e.inputs().chain(SHIFT));
    let lift = |x| scale.lift(x);
    first_nonzero(&e.eval(&Moved::<_, 2>(lift)).0)
        .or_else(|| first_nonzero(&e.eval(&Moved::<_, 3>(lift)).0))
        .or_else(|| first_nonzero(&e.eval(&Moved::<_, 7>(lift)).0))
        .or_else(|| first_nonzero(&e.eval(&Moved::<_, 25>(lift)).0))
        .unwrap_or(Sign::Zero)
}

fn first_nonzero(k: &[BigInt]) -> Option<Sign> {
    k.iter().map(big_sign).find(|s| *s != Sign::Zero)
}

static EXACT_FALLBACKS: AtomicU64 = AtomicU64::new(0);

thread_local! {
    /// Perturbed evaluations on this thread, read by [`ties`].
    static TIES: std::cell::Cell<u64> = const { std::cell::Cell::new(0) };
}

/// Runs `f` and tells whether some predicate in it tied in the actual geometry, its sign coming
/// from the perturbation. Ties mark where the result may have coinciding points or a point on
/// an edge it is not a vertex of. Pure: the answer depends only on the inputs `f` reads.
pub fn ties<R>(f: impl FnOnce() -> R) -> (R, bool) {
    let before = TIES.with(std::cell::Cell::get);
    let r = f();
    (r, TIES.with(std::cell::Cell::get) != before)
}
static PERTURBED: AtomicU64 = AtomicU64::new(0);

/// How many predicate evaluations so far, process-wide, the `f64` filter could not settle.
/// Benchmarks use it to tell whether the exact path is hot.
pub fn exact_fallbacks() -> u64 {
    EXACT_FALLBACKS.load(Ordering::Relaxed)
}

/// How many contacts so far, process-wide, the perturbation settled.
pub fn perturbed_signs() -> u64 {
    PERTURBED.load(Ordering::Relaxed)
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
    fn zero() -> Self {
        Self::input(0.0)
    }

    fn one() -> Self {
        Self::input(1.0)
    }

    /// The filter is only ever applied to the unperturbed configuration, whose homogeneous
    /// coordinates every implicit point keeps.
    fn lpi(l: &Lpi, _: &impl Lift<Self>) -> [Self; 4] {
        l.filter
    }
}

impl Ring for BigInt {
    fn zero() -> Self {
        BigInt::from(0)
    }

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
