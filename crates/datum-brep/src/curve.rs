//! Curved geometry of a body (M5, D63): cylindrical faces, arc edges, the development of a
//! rolled sheet with the edges and walls of cuts given in it, and the integrals over them that
//! volume, area and centre of mass need.
//!
//! Integrals over a cylinder are taken in its coordinates `(θ, z)`. By Green's theorem the
//! integral over a face is one along its boundary, and on a cylinder only the curves contribute
//! (`θ` is constant along a straight edge); along a curve the integrand is smooth, close to a
//! trigonometric polynomial of low degree, which 8-point Gauss–Legendre on pieces of at most 45°
//! integrates to rounding. The `z` direction is linear at most and integrated by Simpson's rule,
//! exactly. A wall is integrated over its own coordinates, along the curve and the radius.

use std::f64::consts::{FRAC_PI_4, TAU};

use datum_math::{Rigid, Vec2, Vec3};

use crate::{Plane, ProfileArc};

/// A circular cylinder: its axis through `origin` along the unit `axis`, and `radius`. When
/// `convex`, the body lies inside (a boss, normals away from the axis); otherwise outside (a hole).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Cylinder {
    pub origin: Vec3,
    pub axis: Vec3,
    pub radius: f64,
    pub convex: bool,
}

/// The surface a face lies on.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Surface {
    Plane(Plane),
    Cylinder(Cylinder),
    /// The wall of a cut through a rolled sheet (M5c): the piece swept along the radius, out of
    /// the body to the left of the piece's direction in the development.
    Wall(Rolled),
}

impl Surface {
    pub fn plane(&self) -> Option<&Plane> {
        match self {
            Self::Plane(p) => Some(p),
            Self::Cylinder(_) | Self::Wall(_) => None,
        }
    }

    /// The same surface moved by `m`.
    pub fn moved(&self, m: &Rigid) -> Self {
        match *self {
            Self::Plane(p) => Self::Plane(Plane { origin: m.apply(p.origin), normal: m.apply_vector(p.normal) }),
            Self::Cylinder(c) => Self::Cylinder(Cylinder { origin: m.apply(c.origin), axis: m.apply_vector(c.axis), ..c }),
            Self::Wall(w) => Self::Wall(Rolled { dev: w.dev.moved(m), ..w }),
        }
    }

    /// The unit normal out of the body at `p`, a point of the surface.
    pub fn normal_at(&self, p: Vec3) -> Vec3 {
        match self {
            Self::Plane(pl) => unit(pl.normal),
            Self::Cylinder(c) => {
                let r = radial(c, p);
                if c.convex { r } else { -r }
            }
            Self::Wall(w) => {
                let (q, r) = w.dev.unroll(p, w.from.x);
                w.normal(q, r)
            }
        }
    }
}

/// The development of a cylinder (D63): coordinates `(u, v)` in which a rolled sheet lies flat,
/// as its blank did before rolling. The point `(u, v)` at distance `r` from the axis is
/// `origin + axis·v + r·(start·cos φ + (axis × start)·sin φ)`, `φ = u / neutral`: `u` is length
/// along the neutral layer, `v` along the unit `axis`; `start`, a unit vector across the axis,
/// is where `u` is zero.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Development {
    pub origin: Vec3,
    pub axis: Vec3,
    pub start: Vec3,
    pub neutral: f64,
}

impl Development {
    /// The unit vector away from the axis at `u`.
    pub fn radial(&self, u: f64) -> Vec3 {
        let (s, c) = libm::sincos(u / self.neutral);
        self.start * c + self.axis.cross(self.start) * s
    }

    pub fn point(&self, p: Vec2, r: f64) -> Vec3 {
        self.origin + self.axis * p.y + self.radial(p.x) * r
    }

    /// How [`Development::point`] moves at `p` along `d`, at `r` from the axis.
    pub fn along(&self, p: Vec2, d: Vec2, r: f64) -> Vec3 {
        self.axis.cross(self.radial(p.x)) * (d.x * r / self.neutral) + self.axis * d.y
    }

    /// The coordinates of `q` and its distance from the axis, `u` within half a turn of `near`.
    pub fn unroll(&self, q: Vec3, near: f64) -> (Vec2, f64) {
        let w = q - self.origin;
        let v = w.dot(self.axis);
        let across = w - self.axis * v;
        let u = self.neutral * libm::atan2(across.dot(self.axis.cross(self.start)), across.dot(self.start));
        let turn = self.neutral * TAU;
        (Vec2::new(u + turn * libm::round((near - u) / turn), v), across.dot(across).sqrt())
    }

    pub fn moved(&self, m: &Rigid) -> Self {
        Self { origin: m.apply(self.origin), axis: m.apply_vector(self.axis), start: m.apply_vector(self.start), neutral: self.neutral }
    }

    pub(crate) fn floats(&self) -> [f64; 10] {
        let [o, a, s] = [self.origin, self.axis, self.start].map(Vec3::to_array);
        [o[0], o[1], o[2], a[0], a[1], a[2], s[0], s[1], s[2], self.neutral]
    }
}

/// A segment or an arc of a development, from `from` to `to`, the arc as in a profile. At a fixed
/// distance from the axis it is an edge on a cylinder (see [`Curve`]); swept along the radius,
/// the wall of a cut (see [`Surface::Wall`]).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rolled {
    pub dev: Development,
    pub from: Vec2,
    pub to: Vec2,
    pub arc: Option<ProfileArc>,
}

impl Rolled {
    pub fn reversed(&self) -> Self {
        Self { from: self.to, to: self.from, arc: self.arc.map(|a| ProfileArc { ccw: !a.ccw, ..a }), ..*self }
    }

    /// The point at `t` from 0 at `from` to 1 at `to`, and its derivative in `t`.
    pub fn at(&self, t: f64) -> (Vec2, Vec2) {
        let (f, g) = (self.from, self.to);
        match self.arc {
            None => {
                let d = Vec2::new(g.x - f.x, g.y - f.y);
                (Vec2::new(f.x + d.x * t, f.y + d.y * t), d)
            }
            Some(a) => {
                let (dx, dy, phi) = (f.x - a.center.x, f.y - a.center.y, a.turn(f, g));
                let (s, c) = libm::sincos(phi * t);
                let (x, y) = (dx * c - dy * s, dx * s + dy * c);
                (Vec2::new(a.center.x + x, a.center.y + y), Vec2::new(-phi * y, phi * x))
            }
        }
    }

    /// The unit normal at `p` of the wall over the piece, `r` from the axis: left of the
    /// direction of the piece.
    pub fn normal(&self, p: Vec2, r: f64) -> Vec3 {
        let d = match self.arc {
            None => Vec2::new(self.to.x - self.from.x, self.to.y - self.from.y),
            Some(a) if a.ccw => Vec2::new(a.center.y - p.y, p.x - a.center.x),
            Some(a) => Vec2::new(p.y - a.center.y, a.center.x - p.x),
        };
        unit(self.dev.radial(p.x).cross(self.dev.along(p, d, r)))
    }

    /// The radius of the arc (0 for a segment), the angle it turns (0 for a segment) and its length.
    pub(crate) fn measure(&self) -> (f64, f64, f64) {
        let (f, g) = (self.from, self.to);
        match self.arc {
            None => (0.0, 0.0, libm::hypot(g.x - f.x, g.y - f.y)),
            Some(a) => {
                let rho = libm::hypot(f.x - a.center.x, f.y - a.center.y);
                let turn = a.turn(f, g).abs();
                (rho, turn, rho * turn)
            }
        }
    }

    /// Pieces of `t` to integrate over: the piece turns at most 45° in each, in the development
    /// and about the axis.
    fn quadrature_pieces(&self) -> usize {
        let (_, turn, length) = self.measure();
        (turn.max(length / self.dev.neutral) / FRAC_PI_4).ceil().max(1.0) as usize
    }

    /// Chords for drawing the piece at `r` from the axis, each at most `chord` off: as many as
    /// the arc in the development needs, or the turn about the axis, whichever is more.
    pub fn chords(&self, r: f64, chord: f64) -> usize {
        let (rho, turn, length) = self.measure();
        let own = turn / TAU * per_turn(rho, chord);
        let bend = length / self.dev.neutral / TAU * per_turn(r, chord);
        own.max(bend).ceil().max(1.0) as usize
    }

    /// Widens the box `lo, hi` to hold the piece at `r` from the axis: the piece lies in its box
    /// in the development (a full circle's for an arc), and that box rolled is held by the arcs
    /// about the axis at its two ends along it.
    fn extend_bounds(&self, r: f64, lo: &mut Vec3, hi: &mut Vec3) {
        let (f, g) = (self.from, self.to);
        let rho = self.arc.map_or(0.0, |a| libm::hypot(f.x - a.center.x, f.y - a.center.y));
        let (mut u, mut v) = ([f.x.min(g.x), f.x.max(g.x)], [f.y.min(g.y), f.y.max(g.y)]);
        if let Some(a) = self.arc {
            u = [u[0].min(a.center.x - rho), u[1].max(a.center.x + rho)];
            v = [v[0].min(a.center.y - rho), v[1].max(a.center.y + rho)];
        }
        for along in v {
            let arc = Arc { center: self.dev.origin + self.dev.axis * along, axis: self.dev.axis, radius: r };
            let [a, b] = u.map(|u| self.dev.point(Vec2::new(u, along), r));
            arc.extend_bounds(a, b, lo, hi);
        }
    }
}

/// The curve of an edge: an arc, or a piece of a development at `radius` from its axis.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Curve {
    Arc(Arc),
    Rolled { piece: Rolled, radius: f64 },
}

impl Curve {
    /// The same curve run the other way.
    pub fn reversed(&self) -> Self {
        match self {
            Self::Arc(a) => Self::Arc(a.reversed()),
            Self::Rolled { piece, radius } => Self::Rolled { piece: piece.reversed(), radius: *radius },
        }
    }

    /// The same curve moved by `m`.
    pub fn moved(&self, m: &Rigid) -> Self {
        match *self {
            Self::Arc(a) => Self::Arc(Arc { center: m.apply(a.center), axis: m.apply_vector(a.axis), ..a }),
            Self::Rolled { piece, radius } => Self::Rolled { piece: Rolled { dev: piece.dev.moved(m), ..piece }, radius },
        }
    }

    pub fn arc(&self) -> Option<&Arc> {
        match self {
            Self::Arc(a) => Some(a),
            Self::Rolled { .. } => None,
        }
    }

    /// Points of the curve from `from` to `to` for drawing, both ends excluded, at most `chord`
    /// off the curve.
    pub fn inner_points(&self, from: Vec3, to: Vec3, chord: f64) -> Vec<Vec3> {
        match self {
            Self::Arc(a) => a.inner_points(from, to, chord).collect(),
            Self::Rolled { piece, radius } => {
                let n = piece.chords(*radius, chord);
                (1..n).map(|k| piece.dev.point(piece.at(k as f64 / n as f64).0, *radius)).collect()
            }
        }
    }

    /// Widens the box `lo, hi` to hold the curve from `from` to `to`.
    pub fn extend_bounds(&self, from: Vec3, to: Vec3, lo: &mut Vec3, hi: &mut Vec3) {
        match self {
            Self::Arc(a) => a.extend_bounds(from, to, lo, hi),
            Self::Rolled { piece, radius } => piece.extend_bounds(*radius, lo, hi),
        }
    }
}

/// Chords a full turn of a circle of `radius` needs to stay within `chord` of it, within
/// [`SEGMENTS`].
pub(crate) fn per_turn(radius: f64, chord: f64) -> f64 {
    // A chord over angle `a` is `r·(1 − cos(a/2))` off the arc.
    let n = if chord < radius { std::f64::consts::PI / libm::acos(1.0 - chord / radius) } else { 0.0 };
    n.ceil().clamp(SEGMENTS[0], SEGMENTS[1])
}

/// The unit vector from the axis of `c` towards `p`.
fn radial(c: &Cylinder, p: Vec3) -> Vec3 {
    let d = p - c.origin;
    unit(d - c.axis * d.dot(c.axis))
}

pub(crate) fn unit(v: Vec3) -> Vec3 {
    v * (1.0 / v.dot(v).sqrt())
}

/// The arc of an edge: counterclockwise about the unit `axis`, around `center`, from the edge's
/// lower-ID vertex to the other one. The vertices lie on the circle up to rounding; the arc
/// starts at the first vertex and turns until the direction of the second.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Arc {
    pub center: Vec3,
    pub axis: Vec3,
    pub radius: f64,
}

/// Chords per full turn of an arc drawn or exported as a polyline: at least enough to look round
/// (12, three arcs of a circle in four each), at most 64 (0.12 % of the radius off the circle).
/// Between them, as many as keep the chords within the body's own tolerance (see
/// [`crate::Body::chord`]): a hole of 5 mm in a sheet of 2 m needs no 64. For the screen and STL
/// only; the model keeps the arc.
const SEGMENTS: [f64; 2] = [12.0, 64.0];

impl Arc {
    /// The same arc run the other way: from the second vertex to the first.
    pub fn reversed(&self) -> Self {
        Self { axis: -self.axis, ..*self }
    }

    /// The angle turned counterclockwise about the axis from `from` to `to`, in `(0, 2π]`: the
    /// same point twice is a full circle.
    pub fn sweep(&self, from: Vec3, to: Vec3) -> f64 {
        let (u, v) = (from - self.center, to - self.center);
        let a = libm::atan2(self.axis.dot(u.cross(v)), u.dot(v));
        if a <= 0.0 { a + TAU } else { a }
    }

    /// The point at angle `t` from `from`, counterclockwise about the axis.
    pub fn at(&self, from: Vec3, t: f64) -> Vec3 {
        let d = from - self.center;
        let d = d - self.axis * d.dot(self.axis);
        let d = d * (self.radius / d.dot(d).sqrt());
        let (s, c) = libm::sincos(t);
        self.center + d * c + self.axis.cross(d) * s
    }

    /// Points of the arc from `from` to `to` for drawing, both ends excluded: chords off the arc
    /// by at most `chord` (within [`SEGMENTS`] a turn).
    pub fn inner_points(&self, from: Vec3, to: Vec3, chord: f64) -> impl Iterator<Item = Vec3> + '_ {
        let sweep = self.sweep(from, to);
        let n = (sweep / TAU * per_turn(self.radius, chord)).ceil().max(1.0) as usize;
        (1..n).map(move |k| self.at(from, sweep * k as f64 / n as f64))
    }

    /// Widens the box `lo, hi` to hold the arc from `from` to `to`: its ends and its points
    /// extreme along an axis that lie on it.
    pub fn extend_bounds(&self, from: Vec3, to: Vec3, lo: &mut Vec3, hi: &mut Vec3) {
        let sweep = self.sweep(from, to);
        let axes = [Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0)];
        let extremes = axes.into_iter().flat_map(|e| {
            // Twice: for an axis along `e` up to rounding, once leaves noise off the circle's plane.
            let w = e - self.axis * e.dot(self.axis);
            let w = w - self.axis * w.dot(self.axis);
            let len = w.dot(w).sqrt();
            [1.0, -1.0].map(|s| (len > 0.0).then(|| self.center + w * (s * self.radius / len)))
        });
        for p in [from, to].into_iter().chain(extremes.flatten().filter(|&p| self.sweep(from, p) < sweep)) {
            *lo = Vec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
            *hi = Vec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
        }
    }
}

/// Integrals over a face, relative to a point `o`, summed in a fixed order: six times the volume
/// of the cone from `o` over the face (`∫ 2·(x − o)·n dA`), 24 times its first moment about `o`
/// (`∫ 6·(x − o)·((x − o)·n) dA`), the area and the area vector `∫ n dA`. Every face is its cone:
/// the cones of a closed surface add up to the solid, whatever each face's shape.
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct Integrals {
    pub six_v: f64,
    pub moment: Vec3,
    pub area: f64,
    pub area_vector: Vec3,
}

/// The correction of a plane face's polygon for an arc from `u` to `v` turning about `k`: the
/// circular segment between the chord and the arc, added where the arc bulges out of the face and
/// taken away where it bulges in. `n` is the face's unit normal.
pub(crate) fn segment(acc: &mut Integrals, arc: &Arc, k: Vec3, u: Vec3, v: Vec3, n: Vec3, o: Vec3) {
    let turned = Arc { axis: k, ..*arc };
    let phi = turned.sweep(u, v);
    let r = arc.radius;
    let area = 0.5 * r * r * (phi - libm::sin(phi)) * if k.dot(n) > 0.0 { 1.0 } else { -1.0 };
    if area == 0.0 {
        return;
    }
    let mid = turned.at(u, 0.5 * phi) - arc.center;
    let half = libm::sin(0.5 * phi);
    let centroid = arc.center + mid * (4.0 * half * half * half / (3.0 * (phi - libm::sin(phi))));
    let d = (u - o).dot(n);
    let six_v = 2.0 * d * area;
    acc.six_v += six_v;
    acc.moment = acc.moment + (centroid - o) * (3.0 * six_v);
    acc.area_vector = acc.area_vector + n * area;
}

/// Nodes and weights of 8-point Gauss–Legendre on `[-1, 1]`, positive half.
const GAUSS: [(f64, f64); 4] = [
    (0.183_434_642_495_649_8, 0.362_683_783_378_362),
    (0.525_532_409_916_329, 0.313_706_645_877_887_3),
    (0.796_666_477_413_626_7, 0.222_381_034_453_374_5),
    (0.960_289_856_497_536_2, 0.101_228_536_290_376_3),
];

/// Coordinates across a cylinder's axis: `r(θ) = e1·cos θ + e2·sin θ`, `e2 = axis × e1`.
pub(crate) struct Frame {
    pub axis: Vec3,
    origin: Vec3,
    e1: Vec3,
    e2: Vec3,
}

pub(crate) fn frame(c: &Cylinder) -> Frame {
    let a = c.axis;
    let least = [Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0)]
        .into_iter()
        .min_by(|x, y| x.dot(a).abs().total_cmp(&y.dot(a).abs()))
        .unwrap_or(Vec3::new(1.0, 0.0, 0.0));
    let e1 = unit(least - a * least.dot(a));
    Frame { axis: a, origin: c.origin, e1, e2: a.cross(e1) }
}

impl Frame {
    /// The angle `θ` of point `p`, in `(−π, π]`.
    pub fn angle(&self, p: Vec3) -> f64 {
        let w = p - self.origin;
        libm::atan2(w.dot(self.e2), w.dot(self.e1))
    }

    /// The point at angle `theta`, `z` along the axis and `r` from it.
    pub fn point(&self, theta: f64, z: f64, r: f64) -> Vec3 {
        let (s, c) = libm::sincos(theta);
        self.origin + self.axis * z + (self.e1 * c + self.e2 * s) * r
    }
}

/// A curve of a cylinder face's boundary as its loop runs it: an arc about the axis from `u` to
/// `v`, turning about its own axis, or a piece of a development.
pub(crate) enum Path {
    Arc(Arc, Vec3, Vec3),
    Rolled(Rolled),
}

/// Integrals of a cylindrical face, from the curves of its loops.
pub(crate) fn cylinder(acc: &mut Integrals, c: &Cylinder, paths: impl Iterator<Item = Path>, o: Vec3) {
    let fr = frame(c);
    let (a, e1, e2) = (fr.axis, fr.e1, fr.e2);
    let (p0, r) = (c.origin - o, c.radius);
    // The integrands at (θ, s) without the sign of the normal, which the orientation of the loops
    // in (θ, z) cancels (see `Integrals`): area, six times volume, moment, area vector.
    let f = |th: f64, s: f64| -> [f64; 8] {
        let (sn, cs) = libm::sincos(th);
        let rv = e1 * cs + e2 * sn;
        let x = p0 + rv * r + a * s;
        // (x − o)·n without its sign: the cone from `o` over the patch, as on plane faces.
        let height = p0.dot(rv) + r;
        let m = x * (6.0 * r * height);
        [r, 2.0 * r * height, m.x, m.y, m.z, r * rv.x, r * rv.y, r * rv.z]
    };
    let mut sum = [0.0; 8];
    // −∫ P(θ, z) dθ along a path `t ↦ (θ, z, dθ/dt)` over `pieces` pieces of [0, 1], where
    // P(θ, z) = ∫₀ᶻ f(θ, s) ds, by Simpson: f is quadratic in s.
    let mut along = |pieces: usize, path: &dyn Fn(f64) -> (f64, f64, f64)| {
        let h = 1.0 / pieces as f64;
        for i in 0..pieces {
            let mid = h * (i as f64 + 0.5);
            for (x, wgt) in GAUSS {
                for t in [mid - 0.5 * h * x, mid + 0.5 * h * x] {
                    let (th, z, dth) = path(t);
                    let (f0, f1, f2) = (f(th, 0.0), f(th, 0.5 * z), f(th, z));
                    for q in 0..8 {
                        sum[q] -= 0.5 * h * wgt * dth * z / 6.0 * (f0[q] + 4.0 * f1[q] + f2[q]);
                    }
                }
            }
        }
    };
    for path in paths {
        match path {
            Path::Arc(arc, u, v) => {
                let dtheta = arc.sweep(u, v) * if arc.axis.dot(a) > 0.0 { 1.0 } else { -1.0 };
                let (th0, z) = (fr.angle(u), (arc.center - c.origin).dot(a));
                along((dtheta.abs() / FRAC_PI_4).ceil().max(1.0) as usize, &|t| (th0 + dtheta * t, z, dtheta));
            }
            Path::Rolled(w) => {
                // θ turns with u, the other way if the development's axis is against the cylinder's.
                let s = if w.dev.axis.dot(a) > 0.0 { 1.0 } else { -1.0 };
                let (start, z0) = (libm::atan2(w.dev.start.dot(e2), w.dev.start.dot(e1)), (w.dev.origin - c.origin).dot(a));
                along(w.quadrature_pieces(), &|t| {
                    let (p, d) = w.at(t);
                    (start + s * p.x / w.dev.neutral, z0 + s * p.y, s * d.x / w.dev.neutral)
                });
            }
        }
    }
    let sigma = if c.convex { 1.0 } else { -1.0 };
    acc.area += sigma * sum[0];
    acc.six_v += sum[1];
    acc.moment = acc.moment + Vec3::new(sum[2], sum[3], sum[4]);
    acc.area_vector = acc.area_vector + Vec3::new(sum[5], sum[6], sum[7]);
}

/// Integrals of the wall over `w` from `r0` to `r1 > r0` from the axis, over its own coordinates
/// `(t, r)` with `n dA = (∂r × ∂t) dt dr`: Gauss–Legendre both ways (along the radius the
/// integrands are polynomials of degree 3 at most, the area element a smooth root).
pub(crate) fn wall(acc: &mut Integrals, w: &Rolled, r0: f64, r1: f64, o: Vec3) {
    let pieces = w.quadrature_pieces();
    let (h, hr, mr) = (1.0 / pieces as f64, r1 - r0, 0.5 * (r0 + r1));
    let base = w.dev.origin - o;
    for i in 0..pieces {
        let mid = h * (i as f64 + 0.5);
        for (x, wt) in GAUSS {
            for t in [mid - 0.5 * h * x, mid + 0.5 * h * x] {
                let (p, d) = w.at(t);
                let radial = w.dev.radial(p.x);
                for (y, wr) in GAUSS {
                    for r in [mr - 0.5 * hr * y, mr + 0.5 * hr * y] {
                        let x = base + w.dev.axis * p.y + radial * r;
                        let n = radial.cross(w.dev.along(p, d, r)) * (0.25 * h * wt * hr * wr);
                        let height = x.dot(n);
                        acc.six_v += 2.0 * height;
                        acc.moment = acc.moment + x * (6.0 * height);
                        acc.area += n.dot(n).sqrt();
                        acc.area_vector = acc.area_vector + n;
                    }
                }
            }
        }
    }
}
