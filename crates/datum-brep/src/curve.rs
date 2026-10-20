//! Curved geometry of a body (M5, D63): cylindrical faces and arc edges, and the integrals over
//! them that volume, area and centre of mass need.
//!
//! Integrals over a cylinder are taken in its coordinates `(θ, z)`. By Green's theorem the
//! integral over a face is one along its boundary, and on a cylinder only the arcs contribute
//! (`θ` is constant along a straight edge); along an arc the integrand is a trigonometric
//! polynomial of low degree, which 8-point Gauss–Legendre on pieces of at most 45° integrates to
//! rounding. The `z` direction is linear at most and integrated by Simpson's rule, exactly.

use std::f64::consts::{FRAC_PI_4, TAU};

use datum_math::Vec3;

use crate::Plane;

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
}

impl Surface {
    pub fn plane(&self) -> Option<&Plane> {
        match self {
            Self::Plane(p) => Some(p),
            Self::Cylinder(_) => None,
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
        }
    }
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

/// Segments per full turn of an arc drawn or exported as a polyline: a chord deviates from the
/// circle by 0.12 % of the radius. For the screen and STL only; the model keeps the arc.
const SEGMENTS: f64 = 64.0;

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

    /// Points of the arc from `from` to `to` for drawing, both ends excluded: chords of at most
    /// `1/SEGMENTS` of a turn.
    pub fn inner_points(&self, from: Vec3, to: Vec3) -> impl Iterator<Item = Vec3> + '_ {
        let sweep = self.sweep(from, to);
        let n = (sweep / TAU * SEGMENTS).ceil().max(1.0) as usize;
        (1..n).map(move |k| self.at(from, sweep * k as f64 / n as f64))
    }

    /// Widens the box `lo, hi` to hold the arc from `from` to `to`: its ends and its points
    /// extreme along an axis that lie on it.
    pub fn extend_bounds(&self, from: Vec3, to: Vec3, lo: &mut Vec3, hi: &mut Vec3) {
        let sweep = self.sweep(from, to);
        let axes = [Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0)];
        let extremes = axes.into_iter().flat_map(|e| {
            let w = e - self.axis * e.dot(self.axis);
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
}

/// Integrals of a cylindrical face, from the arcs of its loops: each arc `(k, u, v)` turns about
/// `k` from `u` to `v` in the direction its loop runs.
pub(crate) fn cylinder(acc: &mut Integrals, c: &Cylinder, arcs: impl Iterator<Item = (Arc, Vec3, Vec3)>, o: Vec3) {
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
    for (arc, u, v) in arcs {
        let dtheta = arc.sweep(u, v) * if arc.axis.dot(a) > 0.0 { 1.0 } else { -1.0 };
        let th0 = fr.angle(u);
        let z = (arc.center - c.origin).dot(a);
        // P(θ) = −∫₀ᶻ f(θ, s) ds, by Simpson: f is quadratic in s.
        let pieces = (dtheta.abs() / FRAC_PI_4).ceil().max(1.0) as usize;
        let h = dtheta / pieces as f64;
        for i in 0..pieces {
            let mid = th0 + h * (i as f64 + 0.5);
            for (x, wgt) in GAUSS {
                for th in [mid - 0.5 * h * x, mid + 0.5 * h * x] {
                    let (f0, f1, f2) = (f(th, 0.0), f(th, 0.5 * z), f(th, z));
                    for q in 0..8 {
                        sum[q] -= 0.5 * h * wgt * z / 6.0 * (f0[q] + 4.0 * f1[q] + f2[q]);
                    }
                }
            }
        }
    }
    let sigma = if c.convex { 1.0 } else { -1.0 };
    acc.area += sigma * sum[0];
    acc.six_v += sum[1];
    acc.moment = acc.moment + Vec3::new(sum[2], sum[3], sum[4]);
    acc.area_vector = acc.area_vector + Vec3::new(sum[5], sum[6], sum[7]);
}
