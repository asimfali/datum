//! Points of a Boolean operation and exact predicates on them (D38).
//!
//! An intersection point is kept implicit, as the inputs that define it, and never rounded while
//! decisions are made: predicates read only the original `f64` coordinates. Every point is taken
//! in homogeneous coordinates `(x, y, z, w)` with `w > 0`, so one formula serves explicit and
//! implicit points alike.

use std::cmp::Ordering;

use crate::exact::{Expr, Ring, sign};
use crate::{Sign, Vec3};

/// A point: explicit coordinates or an implicit intersection.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Point {
    Explicit(Vec3),
    Lpi(Lpi),
}

/// The intersection of the line through `p, q` with the plane through the triangle `r, s, t`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Lpi {
    /// `p, q, r, s, t`, with `p, q` ordered so that the homogeneous `w` is positive.
    pts: [Vec3; 5],
}

impl Lpi {
    /// `None` when the line is parallel to the plane or the triangle is degenerate.
    pub fn new(p: Vec3, q: Vec3, r: Vec3, s: Vec3, t: Vec3) -> Option<Self> {
        let lpi = Self { pts: [p, q, r, s, t] };
        match sign(&Denominator(&lpi)) {
            Sign::Positive => Some(lpi),
            Sign::Negative => Some(Self { pts: [q, p, r, s, t] }),
            Sign::Zero => None,
        }
    }

    /// Rounded coordinates, for output only: decisions never read them.
    pub fn approx(&self) -> Vec3 {
        let [p, q, r, s, t] = self.pts;
        let n = (s - r).cross(t - r);
        p + (q - p) * (n.dot(r - p) / n.dot(q - p))
    }

    /// Normal of the plane, direction of the line, and a point on each: `p + dir·λ` meets the
    /// plane where `normal·(r - p) = λ·normal·dir`.
    fn parts<T: Ring>(&self, lift: &impl Fn(f64) -> T) -> [[T; 3]; 4] {
        let [p, q, r, s, t] = self.pts.map(|v| v.to_array().map(lift));
        let normal = cross(sub(&s, &r), sub(&t, &r));
        [sub(&q, &p), normal, p, r]
    }

    fn homogeneous<T: Ring>(&self, lift: &impl Fn(f64) -> T) -> [T; 4] {
        let [dir, normal, p, r] = self.parts(lift);
        let w = dot(&normal, &dir);
        let lambda = dot(&normal, &sub(&r, &p));
        let [x, y, z] = [0, 1, 2].map(|i| p[i].clone() * w.clone() + dir[i].clone() * lambda.clone());
        [x, y, z, w]
    }
}

impl Point {
    /// Rounded coordinates, for output only.
    pub fn approx(&self) -> Vec3 {
        match self {
            Self::Explicit(v) => *v,
            Self::Lpi(l) => l.approx(),
        }
    }

    fn inputs(&self) -> &[Vec3] {
        match self {
            Self::Explicit(v) => std::slice::from_ref(v),
            Self::Lpi(l) => &l.pts,
        }
    }

    fn homogeneous<T: Ring>(&self, lift: &impl Fn(f64) -> T) -> [T; 4] {
        match self {
            Self::Explicit(v) => {
                let [x, y, z] = v.to_array().map(lift);
                [x, y, z, T::one()]
            }
            Self::Lpi(l) => l.homogeneous(lift),
        }
    }
}

/// Exact orientation of `d` relative to the plane through `a, b, c`, with the convention of
/// [`robust::orient3d`]: `Positive` when `d` lies below the plane, `a, b, c` counterclockwise seen
/// from above; `Zero` when coplanar.
pub fn orient3d(a: &Point, b: &Point, c: &Point, d: &Point) -> Sign {
    if let [Point::Explicit(a), Point::Explicit(b), Point::Explicit(c), Point::Explicit(d)] = [a, b, c, d] {
        let p = |v: &Vec3| robust::Coord3D { x: v.x, y: v.y, z: v.z };
        return Sign::of(robust::orient3d(p(a), p(b), p(c), p(d)));
    }
    sign(&Orient3d([a, b, c, d]))
}

/// Exact orientation of `a, b, c` projected along `axis` (0 = x, 1 = y, 2 = z): `Positive` when
/// they turn counterclockwise seen from the positive end of the axis, `Zero` when collinear in
/// projection.
pub fn orient2d_along(a: &Point, b: &Point, c: &Point, axis: usize) -> Sign {
    if let [Point::Explicit(a), Point::Explicit(b), Point::Explicit(c)] = [a, b, c] {
        let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
        let p = |p: &Vec3| crate::Vec2::new(p.to_array()[u], p.to_array()[v]);
        return crate::orient2d(p(a), p(b), p(c));
    }
    sign(&Orient2d { pts: [a, b, c], axis })
}

/// Exact comparison of one coordinate (0 = x, 1 = y, 2 = z) of two points: orders points on a
/// line by its dominant axis.
pub fn cmp_axis(a: &Point, b: &Point, axis: usize) -> Ordering {
    if let (Point::Explicit(a), Point::Explicit(b)) = (a, b) {
        return a.to_array()[axis].partial_cmp(&b.to_array()[axis]).unwrap_or(Ordering::Equal);
    }
    match sign(&CmpAxis { pts: [a, b], axis }) {
        Sign::Negative => Ordering::Less,
        Sign::Zero => Ordering::Equal,
        Sign::Positive => Ordering::Greater,
    }
}

fn inputs<'a>(pts: &'a [&'a Point]) -> impl Iterator<Item = f64> + 'a {
    pts.iter().flat_map(|p| p.inputs()).flat_map(|v| v.to_array())
}

struct Denominator<'a>(&'a Lpi);

impl Expr for Denominator<'_> {
    fn inputs(&self) -> impl Iterator<Item = f64> {
        self.0.pts.iter().flat_map(|v| v.to_array())
    }
    fn eval<T: Ring>(&self, lift: &impl Fn(f64) -> T) -> T {
        let [dir, normal, ..] = self.0.parts(lift);
        dot(&normal, &dir)
    }
}

struct Orient3d<'a>([&'a Point; 4]);

impl Expr for Orient3d<'_> {
    fn inputs(&self) -> impl Iterator<Item = f64> {
        inputs(&self.0)
    }
    /// `det [a b c d]` with points as rows `(x, y, z, w)`, by Laplace expansion on the first two
    /// rows. For `w = 1` it equals `det [a-d; b-d; c-d]`, as in `robust`.
    fn eval<T: Ring>(&self, lift: &impl Fn(f64) -> T) -> T {
        let [a, b, c, d] = self.0.map(|p| p.homogeneous(lift));
        let top = |i: usize, j: usize| a[i].clone() * b[j].clone() - a[j].clone() * b[i].clone();
        let bottom = |i: usize, j: usize| c[i].clone() * d[j].clone() - c[j].clone() * d[i].clone();
        top(0, 1) * bottom(2, 3) - top(0, 2) * bottom(1, 3) + top(0, 3) * bottom(1, 2) + top(1, 2) * bottom(0, 3) - top(1, 3) * bottom(0, 2)
            + top(2, 3) * bottom(0, 1)
    }
}

struct Orient2d<'a> {
    pts: [&'a Point; 3],
    axis: usize,
}

impl Expr for Orient2d<'_> {
    fn inputs(&self) -> impl Iterator<Item = f64> {
        inputs(&self.pts)
    }
    /// `det [a b c]` with points as rows `(u, v, w)`; `u, v` follow `axis` cyclically, so the
    /// projection keeps the handedness seen from the axis.
    fn eval<T: Ring>(&self, lift: &impl Fn(f64) -> T) -> T {
        let (u, v) = ((self.axis + 1) % 3, (self.axis + 2) % 3);
        let [a, b, c] = self.pts.map(|p| p.homogeneous(lift));
        let minor = |i: usize, j: usize| b[i].clone() * c[j].clone() - b[j].clone() * c[i].clone();
        a[u].clone() * minor(v, 3) - a[v].clone() * minor(u, 3) + a[3].clone() * minor(u, v)
    }
}

struct CmpAxis<'a> {
    pts: [&'a Point; 2],
    axis: usize,
}

impl Expr for CmpAxis<'_> {
    fn inputs(&self) -> impl Iterator<Item = f64> {
        inputs(&self.pts)
    }
    /// `a/wa - b/wb` scaled by `wa·wb > 0`.
    fn eval<T: Ring>(&self, lift: &impl Fn(f64) -> T) -> T {
        let [a, b] = self.pts.map(|p| p.homogeneous(lift));
        a[self.axis].clone() * b[3].clone() - b[self.axis].clone() * a[3].clone()
    }
}

fn sub<T: Ring>(a: &[T; 3], b: &[T; 3]) -> [T; 3] {
    [0, 1, 2].map(|i| a[i].clone() - b[i].clone())
}

fn cross<T: Ring>(a: [T; 3], b: [T; 3]) -> [T; 3] {
    [0, 1, 2].map(|i| {
        let (j, k) = ((i + 1) % 3, (i + 2) % 3);
        a[j].clone() * b[k].clone() - a[k].clone() * b[j].clone()
    })
}

fn dot<T: Ring>(a: &[T; 3], b: &[T; 3]) -> T {
    a[0].clone() * b[0].clone() + a[1].clone() * b[1].clone() + a[2].clone() * b[2].clone()
}

#[cfg(test)]
mod tests {
    use proptest::prelude::*;

    use super::*;
    use crate::exact::{exact, filtered};

    fn x(v: [f64; 3]) -> Point {
        Point::Explicit(v.into())
    }

    /// Points in general position, far from the origin too (principle 8).
    fn pts<const N: usize>() -> impl Strategy<Value = [Vec3; N]> {
        let offset = prop::sample::select(vec![0.0, 1e3, 1e6, 1e7]);
        (offset, prop::array::uniform::<_, N>(prop::array::uniform3(-100.0..100.0f64)))
            .prop_map(|(o, a)| a.map(|[x, y, z]| Vec3::new(x + o, y - o, z + 0.5 * o)))
    }

    fn lpi([p, q, r, s, t]: [Vec3; 5]) -> Option<Point> {
        Lpi::new(p, q, r, s, t).map(Point::Lpi)
    }

    proptest! {
        #[test]
        fn lpi_lies_exactly_on_its_plane_and_line(v in pts::<5>()) {
            let Some(l) = lpi(v) else { return Ok(()) };
            let [p, q, r, s, t] = v.map(Point::Explicit);
            prop_assert_eq!(orient3d(&r, &s, &t, &l), Sign::Zero);
            prop_assert_eq!(orient3d(&l, &r, &s, &t), Sign::Zero);
            for axis in 0..3 {
                prop_assert_eq!(orient2d_along(&p, &q, &l, axis), Sign::Zero);
            }
        }

        #[test]
        fn lpi_does_not_depend_on_segment_direction(v in pts::<5>()) {
            let [p, q, r, s, t] = v;
            let (Some(a), Some(b)) = (lpi(v), lpi([q, p, r, s, t])) else { return Ok(()) };
            for axis in 0..3 {
                prop_assert_eq!(cmp_axis(&a, &b, axis), Ordering::Equal);
            }
        }

        #[test]
        fn filter_agrees_with_exact(v in pts::<10>(), w in pts::<2>()) {
            let (Some(a), Some(b)) = (lpi([v[0], v[1], v[2], v[3], v[4]]), lpi([v[5], v[6], v[7], v[8], v[9]])) else { return Ok(()) };
            let [c, d] = w.map(Point::Explicit);
            let e = Orient3d([&a, &c, &b, &d]);
            if let Some(s) = filtered(&e) {
                prop_assert_eq!(s, exact(&e));
            }
            let e = Orient2d { pts: [&a, &b, &c], axis: 1 };
            if let Some(s) = filtered(&e) {
                prop_assert_eq!(s, exact(&e));
            }
        }

        #[test]
        fn orient3d_is_antisymmetric(v in pts::<10>(), w in pts::<2>()) {
            let (Some(a), Some(b)) = (lpi([v[0], v[1], v[2], v[3], v[4]]), lpi([v[5], v[6], v[7], v[8], v[9]])) else { return Ok(()) };
            let [c, d] = w.map(Point::Explicit);
            prop_assert_eq!(orient3d(&a, &b, &c, &d), -orient3d(&b, &a, &c, &d));
            prop_assert_eq!(orient3d(&a, &b, &c, &d), -orient3d(&a, &b, &d, &c));
        }

        #[test]
        fn generic_formulas_match_robust(v in pts::<4>()) {
            let [a, b, c, d] = v.map(Point::Explicit);
            prop_assert_eq!(exact(&Orient3d([&a, &b, &c, &d])), orient3d(&a, &b, &c, &d));
            let flat = v.map(|p| crate::Vec2::new(p.x, p.y));
            prop_assert_eq!(orient2d_along(&a, &b, &c, 2), crate::orient2d(flat[0], flat[1], flat[2]));
        }
    }

    #[test]
    fn coplanar_explicit_points_are_zero_on_both_paths() {
        // On the plane z = x + y, exactly representable.
        let [a, b, c, d] = [[0.0, 0.0, 0.0], [3.0, 1.0, 4.0], [-2.0, 5.0, 3.0], [7.0, -3.0, 4.0]].map(x);
        assert_eq!(orient3d(&a, &b, &c, &d), Sign::Zero);
        assert_eq!(exact(&Orient3d([&a, &b, &c, &d])), Sign::Zero);
    }

    #[test]
    fn lpi_through_a_point_of_the_plane_is_that_point() {
        let (p, q) = (Vec3::new(0.1, 0.2, 0.0), Vec3::new(0.7, -0.3, 5.0));
        let l = Point::Lpi(Lpi::new(p, q, Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0)).unwrap());
        for axis in 0..3 {
            assert_eq!(cmp_axis(&l, &Point::Explicit(p), axis), Ordering::Equal);
        }
        assert_eq!(l.approx(), p);
    }

    #[test]
    fn parallel_line_or_degenerate_triangle_has_no_lpi() {
        let o = Vec3::ZERO;
        let (ex, ey) = (Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0));
        assert_eq!(Lpi::new(Vec3::new(0.0, 0.0, 1.0), Vec3::new(1.0, 1.0, 1.0), o, ex, ey), None);
        assert_eq!(Lpi::new(Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.0, 0.0, -1.0), o, ex, ex * 2.0), None);
    }

    #[test]
    fn orders_points_on_a_segment() {
        // The segment crosses z = 1 before z = 2; both planes are given by rotated-looking triangles.
        let (p, q) = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.3, 0.1, 3.0));
        let at = |z: f64| Point::Lpi(Lpi::new(q, p, Vec3::new(5.0, 0.0, z), Vec3::new(0.0, 5.0, z), Vec3::new(-3.0, -4.0, z)).unwrap());
        assert_eq!(cmp_axis(&at(1.0), &at(2.0), 2), Ordering::Less);
        assert_eq!(cmp_axis(&at(2.0), &at(1.0), 0), Ordering::Greater);
    }
}
