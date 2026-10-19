//! Points of a Boolean operation and exact predicates on them (D38).
//!
//! An intersection point is kept implicit, as the inputs that define it, and never rounded while
//! decisions are made: predicates read only the original `f64` coordinates. Every point is taken
//! in homogeneous coordinates `(x, y, z, w)` with `w > 0`, so one formula serves explicit and
//! implicit points alike.

use std::cmp::Ordering;

use num_bigint::BigInt;

use crate::exact::{Approx, Expr, Lift, Ring, Scale, Still, filtered, perturbed, sign};
use crate::{Sign, Vec3};

/// A point: explicit coordinates or an implicit intersection.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Point {
    Explicit(Vec3),
    /// An input vertex of a shell that moves under the symbolic perturbation (Q7): it grows away
    /// from the shell's centre, the second field, and shifts in a fixed direction, both by an
    /// infinitesimal amount. Decisions that tie on the coordinates are settled by that motion.
    Moved(Vec3, Vec3),
    Lpi(Lpi),
}

/// The intersection of the line through `p, q` with the plane through the triangle `r, s, t`.
#[derive(Clone, Copy, Debug)]
pub struct Lpi {
    /// `p, q, r, s, t`, with `p, q` ordered so that the homogeneous `w` is positive.
    pts: [Vec3; 5],
    /// Centre of the moving shell, and which of the line and the plane belong to it. Both may
    /// move only with one centre: points of two different moving shells are never combined.
    centre: Vec3,
    moving: [bool; 2],
    /// Homogeneous coordinates in filter arithmetic, computed once: every predicate on the point
    /// starts from them instead of rebuilding the plane and the line (Attene et al.).
    pub(crate) filter: [Approx; 4],
}

/// Two points are the same definition: the cached filter values follow from the inputs.
impl PartialEq for Lpi {
    fn eq(&self, o: &Self) -> bool {
        self.pts == o.pts && self.moving == o.moving && (self.moving == [false; 2] || self.centre == o.centre)
    }
}

impl Lpi {
    /// The line through `p, q` and the plane through `r, s, t`, none of which move. `None` when
    /// the line is parallel to the plane or the triangle is degenerate.
    pub fn new(p: Vec3, q: Vec3, r: Vec3, s: Vec3, t: Vec3) -> Option<Self> {
        Self::between([&Point::Explicit(p), &Point::Explicit(q)], [&Point::Explicit(r), &Point::Explicit(s), &Point::Explicit(t)])
    }

    /// The line through two input points and the plane through three, each explicit or moving
    /// with one common centre. `None` for implicit inputs, moving inputs with different centres, a
    /// line parallel to the plane or a degenerate triangle.
    pub fn between(line: [&Point; 2], plane: [&Point; 3]) -> Option<Self> {
        let site = |p: &Point| match *p {
            Point::Explicit(v) => Some((v, None)),
            Point::Moved(v, c) => Some((v, Some(c))),
            Point::Lpi(_) => None,
        };
        let ([p, q], [r, s, t]) = (line.map(site), plane.map(site));
        let (p, q, r, s, t) = (p?, q?, r?, s?, t?);
        let centre = match (p.1, r.1) {
            (Some(a), Some(b)) if a != b => return None,
            (a, b) => a.or(b),
        };
        let mut lpi = Self {
            pts: [p.0, q.0, r.0, s.0, t.0],
            centre: centre.unwrap_or(Vec3::ZERO),
            moving: [p.1.is_some(), r.1.is_some()],
            filter: [Approx::input(0.0); 4],
        };
        lpi.filter = lpi.homogeneous(&Still(Approx::input));
        let w = lpi.filter[3].sign().unwrap_or_else(|| sign(&Denominator(&lpi)));
        match w {
            Sign::Positive => Some(lpi),
            // Swapping `p, q` negates the direction, `w` and `λ`; the point stays.
            Sign::Negative => {
                lpi.pts.swap(0, 1);
                lpi.filter = lpi.homogeneous(&Still(Approx::input));
                Some(lpi)
            }
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
    #[inline]
    fn parts<T: Ring>(&self, lift: &impl Lift<T>) -> [[T; 3]; 4] {
        let (line, plane) = (self.moving[0].then_some(self.centre), self.moving[1].then_some(self.centre));
        let (p, q) = (lift.point(self.pts[0], line), lift.point(self.pts[1], line));
        let (r, s, t) = (lift.point(self.pts[2], plane), lift.point(self.pts[3], plane), lift.point(self.pts[4], plane));
        let normal = cross(sub(&s, &r), sub(&t, &r));
        [sub(&q, &p), normal, p, r]
    }

    pub(crate) fn homogeneous<T: Ring>(&self, lift: &impl Lift<T>) -> [T; 4] {
        let [dir, normal, p, r] = self.parts(lift);
        let w = dot(&normal, &dir);
        let lambda = dot(&normal, &sub(&r, &p));
        let [x, y, z] = [0, 1, 2].map(|i| p[i].clone() * w.clone() + dir[i].clone() * lambda.clone());
        [x, y, z, w]
    }

    fn inputs(&self) -> impl Iterator<Item = f64> + '_ {
        let centre = self.moving.contains(&true).then_some(&self.centre);
        self.pts.iter().chain(centre).flat_map(|v| v.to_array())
    }
}

impl Point {
    /// Rounded coordinates, for output only.
    pub fn approx(&self) -> Vec3 {
        match self {
            Self::Explicit(v) | Self::Moved(v, _) => *v,
            Self::Lpi(l) => l.approx(),
        }
    }

    /// The same point without the perturbation: for questions about the actual geometry, such as
    /// whether two points of a result coincide.
    pub fn still(&self) -> Self {
        match *self {
            Self::Moved(v, _) => Self::Explicit(v),
            Self::Lpi(l) => Self::Lpi(Lpi { moving: [false; 2], ..l }),
            p => p,
        }
    }

    fn moves(&self) -> bool {
        match self {
            Self::Explicit(_) => false,
            Self::Moved(..) => true,
            Self::Lpi(l) => l.moving.contains(&true),
        }
    }

    fn inputs(&self) -> impl Iterator<Item = f64> + '_ {
        let (own, lpi) = match self {
            Self::Explicit(v) => ([Some(*v), None], None),
            Self::Moved(v, c) => ([Some(*v), Some(*c)], None),
            Self::Lpi(l) => ([None, None], Some(l)),
        };
        own.into_iter().flatten().flat_map(|v| v.to_array()).chain(lpi.into_iter().flat_map(Lpi::inputs))
    }

    fn homogeneous<T: Ring>(&self, lift: &impl Lift<T>) -> [T; 4] {
        match self {
            Self::Explicit(v) => {
                let [x, y, z] = lift.point(*v, None);
                [x, y, z, T::one()]
            }
            Self::Moved(v, c) => {
                let [x, y, z] = lift.point(*v, Some(*c));
                [x, y, z, T::one()]
            }
            Self::Lpi(l) => T::lpi(l, lift),
        }
    }

    /// Coordinates when the point is an input vertex: the fast paths of the predicates.
    fn site(&self) -> Option<Vec3> {
        match self {
            Self::Explicit(v) | Self::Moved(v, _) => Some(*v),
            Self::Lpi(_) => None,
        }
    }
}

/// Exact orientation of `d` relative to the plane through `a, b, c`, with the convention of
/// [`robust::orient3d`]: `Positive` when `d` lies below the plane, `a, b, c` counterclockwise seen
/// from above; `Zero` when coplanar and nothing moving breaks the tie.
pub fn orient3d(a: &Point, b: &Point, c: &Point, d: &Point) -> Sign {
    let pts = [a, b, c, d];
    if let [Some(a), Some(b), Some(c), Some(d)] = pts.map(Point::site) {
        // An exact zero on the coordinates goes straight to the perturbation.
        let s = crate::orient3d_coords(a, b, c, d);
        return if s == Sign::Zero && pts.iter().any(|p| p.moves()) { perturbed(&Orient3d(pts)) } else { s };
    }
    sign(&Orient3d(pts))
}

/// Exact orientation of `a, b, c` projected along `axis` (0 = x, 1 = y, 2 = z): `Positive` when
/// they turn counterclockwise seen from the positive end of the axis, `Zero` when collinear in
/// projection and nothing moving breaks the tie.
pub fn orient2d_along(a: &Point, b: &Point, c: &Point, axis: usize) -> Sign {
    let pts = [a, b, c];
    if let [Some(a), Some(b), Some(c)] = pts.map(Point::site) {
        let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
        let p = |p: Vec3| crate::Vec2::new(p.to_array()[u], p.to_array()[v]);
        let s = crate::orient2d(p(a), p(b), p(c));
        return if s == Sign::Zero && pts.iter().any(|p| p.moves()) { perturbed(&Orient2d { pts, axis }) } else { s };
    }
    sign(&Orient2d { pts, axis })
}

/// Exact comparison of one coordinate (0 = x, 1 = y, 2 = z) of two points: orders points on a
/// line by its dominant axis.
pub fn cmp_axis(a: &Point, b: &Point, axis: usize) -> Ordering {
    if let (Some(x), Some(y)) = (a.site(), b.site()) {
        let o = x.to_array()[axis].partial_cmp(&y.to_array()[axis]).unwrap_or(Ordering::Equal);
        if o != Ordering::Equal || !(a.moves() || b.moves()) {
            return o;
        }
        return ordering(perturbed(&CmpAxis { pts: [a, b], axis }));
    }
    ordering(sign(&CmpAxis { pts: [a, b], axis }))
}

fn ordering(s: Sign) -> Ordering {
    match s {
        Sign::Negative => Ordering::Less,
        Sign::Zero => Ordering::Equal,
        Sign::Positive => Ordering::Greater,
    }
}

/// [`cmp_axis`] for points that do not move, when the `f64` filter settles it; `None` when the
/// coordinates may be equal and only [`Exact`] can tell.
pub fn cmp_axis_filtered(a: &Point, b: &Point, axis: usize) -> Option<Ordering> {
    debug_assert!(!a.moves() && !b.moves(), "still points only");
    if let (Some(x), Some(y)) = (a.site(), b.site()) {
        return x.to_array()[axis].partial_cmp(&y.to_array()[axis]);
    }
    filtered(&CmpAxis { pts: [a, b], axis }).map(ordering)
}

/// One integer scale for a whole set of points (see [`Exact`]).
pub struct ExactScale(Scale);

impl ExactScale {
    pub fn of<'a>(points: impl Iterator<Item = &'a Point>) -> Self {
        Self(Scale::of(points.flat_map(Point::inputs)))
    }

    /// Exact homogeneous coordinates of `p`, which must not move.
    pub fn exact(&self, p: &Point) -> Exact {
        debug_assert!(!p.moves(), "still points only");
        Exact(p.homogeneous(&Still(|x| self.0.lift(x))))
    }
}

/// Exact homogeneous coordinates of a still point, lifted with the scale of its set: computed
/// once, then compared with the others of the set by two products. Sorting many points with equal
/// coordinates (a perforation's columns) would otherwise recompute both points at every tie.
pub struct Exact([BigInt; 4]);

impl Exact {
    /// [`cmp_axis`] of the two points (`w > 0` for both).
    pub fn cmp_axis(&self, other: &Exact, axis: usize) -> Ordering {
        (&self.0[axis] * &other.0[3]).cmp(&(&other.0[axis] * &self.0[3]))
    }
}

fn inputs<'a>(pts: &'a [&'a Point]) -> impl Iterator<Item = f64> + 'a {
    pts.iter().flat_map(|p| p.inputs())
}

struct Denominator<'a>(&'a Lpi);

impl Expr for Denominator<'_> {
    fn inputs(&self) -> impl Iterator<Item = f64> {
        self.0.inputs()
    }
    fn moves(&self) -> bool {
        self.0.moving.contains(&true)
    }
    fn eval<T: Ring>(&self, lift: &impl Lift<T>) -> T {
        let [dir, normal, ..] = self.0.parts(lift);
        dot(&normal, &dir)
    }
}

struct Orient3d<'a>([&'a Point; 4]);

impl Expr for Orient3d<'_> {
    fn inputs(&self) -> impl Iterator<Item = f64> {
        inputs(&self.0)
    }
    fn moves(&self) -> bool {
        self.0.iter().any(|p| p.moves())
    }
    /// `det [a b c d]` with points as rows `(x, y, z, w)`, by Laplace expansion on the first two
    /// rows. For `w = 1` it equals `det [a-d; b-d; c-d]`, as in `robust`.
    fn eval<T: Ring>(&self, lift: &impl Lift<T>) -> T {
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
    fn moves(&self) -> bool {
        self.pts.iter().any(|p| p.moves())
    }
    /// `det [a b c]` with points as rows `(u, v, w)`; `u, v` follow `axis` cyclically, so the
    /// projection keeps the handedness seen from the axis.
    fn eval<T: Ring>(&self, lift: &impl Lift<T>) -> T {
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
    fn moves(&self) -> bool {
        self.pts.iter().any(|p| p.moves())
    }
    /// `a/wa - b/wb` scaled by `wa·wb > 0`.
    fn eval<T: Ring>(&self, lift: &impl Lift<T>) -> T {
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

    /// The perturbed sign is the sign for a small real motion of the same kind.
    #[test]
    fn contacts_resolve_as_a_small_growth() {
        let [a, b, c] = [[0.0, 0.0, 1.0], [4.0, 0.0, 1.0], [0.0, 4.0, 1.0]].map(x);
        let on_plane = Vec3::new(1.0, 1.0, 1.0);
        for centre in [Vec3::new(0.5, 0.5, -2.0), Vec3::new(0.5, 0.5, 3.0)] {
            let grown = on_plane + (on_plane - centre) * 1e-3;
            assert_eq!(orient3d(&a, &b, &c, &Point::Moved(on_plane, centre)), orient3d(&a, &b, &c, &Point::Explicit(grown)));
            assert_ne!(orient3d(&a, &b, &c, &Point::Moved(on_plane, centre)), Sign::Zero);
        }
        // Growth along the plane leaves the tie; the second-order shift breaks it.
        let centre = Vec3::new(-3.0, 2.0, 1.0);
        let shifted = on_plane + Vec3::from(crate::exact::SHIFT_FOR_TESTS) * 1e-3;
        assert_eq!(orient3d(&a, &b, &c, &Point::Moved(on_plane, centre)), orient3d(&a, &b, &c, &Point::Explicit(shifted)));
    }

    #[test]
    fn perturbed_signs_stay_antisymmetric() {
        let [a, b, c] = [[0.0, 0.0, 1.0], [4.0, 0.0, 1.0], [0.0, 4.0, 1.0]].map(x);
        let d = Point::Moved(Vec3::new(1.0, 1.0, 1.0), Vec3::new(2.0, 2.0, 2.0));
        assert_eq!(orient3d(&a, &b, &c, &d), -orient3d(&b, &a, &c, &d));
        assert_eq!(orient2d_along(&a, &b, &d, 2), -orient2d_along(&b, &a, &d, 2));
    }

    /// Points of one moving shell move together: a face of the shell stays planar.
    #[test]
    fn a_moving_face_stays_planar() {
        let centre = Vec3::new(0.3, 0.2, 0.0);
        let [a, b, c, d] = [[0.0, 0.0, 1.0], [4.0, 0.0, 1.0], [0.0, 4.0, 1.0], [1.0, 1.0, 1.0]].map(|v| Point::Moved(v.into(), centre));
        assert_eq!(orient3d(&a, &b, &c, &d), Sign::Zero);
    }

    /// A moving edge exactly through a still vertex: the crossing leaves the vertex, but the
    /// unperturbed geometry still knows they coincide.
    #[test]
    fn crossing_through_a_vertex_separates() {
        let centre = Vec3::new(0.0, 0.0, 5.0);
        let (p, q) = (Point::Moved(Vec3::new(1.0, 1.0, -1.0), centre), Point::Moved(Vec3::new(1.0, 1.0, 3.0), centre));
        let tri = [[1.0, 1.0, 1.0], [5.0, 1.0, 1.0], [1.0, 5.0, 1.0]].map(x);
        let l = Point::Lpi(Lpi::between([&p, &q], [&tri[0], &tri[1], &tri[2]]).unwrap());
        assert!((0..3).any(|axis| cmp_axis(&l, &tri[0], axis) != Ordering::Equal));
        assert!((0..3).all(|axis| cmp_axis(&l.still(), &tri[0], axis) == Ordering::Equal));
        assert_eq!(l.approx(), Vec3::new(1.0, 1.0, 1.0));
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
