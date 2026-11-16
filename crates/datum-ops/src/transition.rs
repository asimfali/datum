//! A faceted transition between two closed profiles on parallel planes (D264): flat triangles
//! over the straight pieces of the first profile, faceted cones (fans) over its vertices, and a
//! real bend of the rule's radius on every edge between two facets. The blank edge follows the
//! facets. The relief at every vertex of the first profile (D264): a sector from the vertex, its
//! sides along the extreme bends' axes, its far boundary the bends' end sections — cut at the
//! bottom and at the top rim (the bands stop short of the rim). The разметка below is what the
//! body is assembled from; `transition` refuses until the whole body validates.

use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::PI;

#[cfg(test)]
use datum_brep::{Body, FaceSpec, Plane, Surface, Vertex, build_with_curves};
use datum_brep::{BrepError, ProfileArc, ProfileLoop, TopoId};
use datum_math::{Vec2, Vec3};

use crate::sheet::Side;
use crate::sketch::{Placed, chosen, nearly_parallel, profile_regions, within_rounding};

/// A transition's numbers: the part's rule's thickness, bend radius and K-factor, where the sheet
/// lies from the profiles, the seam's gap and the tolerance of the fan's chords.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TransitionDims {
    pub thickness: f64,
    pub radius: f64,
    pub k_factor: f64,
    pub side: Side,
    pub gap: f64,
    pub tolerance: f64,
}

/// A straight or circular piece of a profile in the first sketch's coordinates.
#[derive(Clone, Copy, Debug)]
struct Piece {
    from: Vec2,
    to: Vec2,
    arc: Option<ProfileArc>,
}

/// A smooth run of pieces in the first sketch's coordinates: a line, or an arc of one circle.
#[derive(Clone, Copy, Debug)]
struct Smooth {
    from: Vec2,
    to: Vec2,
    kind: Kind,
    /// The length of a line, the swept angle of an arc (positive: `ccw` says the way round).
    span: f64,
}

#[derive(Clone, Copy, Debug)]
enum Kind {
    Line,
    Arc { center: Vec2, radius: f64, ccw: bool },
}

/// A flat facet of the разметка: its loop of vertices, in the order that faces out of the body,
/// its plane, and its top rim vertices (one apex at a flat triangle, two chord ends at a fan).
#[derive(Clone, Debug)]
pub(crate) struct Facet {
    pub(crate) loop_: Vec<TopoId>,
    pub(crate) origin: Vec3,
    pub(crate) normal: Vec3,
    /// The top rim vertices: the apex at a flat triangle, the two chord ends at a fan facet.
    pub(crate) top: Vec<TopoId>,
}

/// A real bend between two facets: a cylinder of the rule's radius tangent to both, its axis along
/// their shared edge.
#[derive(Clone, Debug)]
pub(crate) struct Bend {
    /// The two facets it joins, as indices of [`Layout::facets`].
    pub(crate) facets: [usize; 2],
    /// The shared edge's two vertices.
    pub(crate) edge: [TopoId; 2],
    /// A point of its axis.
    pub(crate) at: Vec3,
    /// The unit direction along its axis.
    pub(crate) axis: Vec3,
    pub(crate) radius_in: f64,
    pub(crate) radius_out: f64,
    /// The tangent points on the two facets' own planes: the axis's feet on them.
    pub(crate) tangent: [Vec3; 2],
    /// The angle the outer surface turns through: between the two facets' outward normals.
    pub(crate) sweep: f64,
    /// The point of the band's top end plane, square to the axis (D264, option b): the band stops
    /// short of the top rim, whose meeting with the band would be an ellipse. At an extreme bend,
    /// the meeting of its triangle's two tangent lines; at an inner bend, the axis point whose end
    /// face's highest point touches the top profile's plane (D265) — nothing of the blank's edge
    /// rises off the profile.
    pub(crate) top: Vec3,
}

/// The seam: a cut through the middle of the flat triangle over the first piece of `a`.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Seam {
    pub(crate) facet: usize,
    pub(crate) from: Vec3,
    pub(crate) to: Vec3,
    pub(crate) gap: f64,
}

/// The relief at a sharp vertex of `a` (D264): a sector from the vertex, its sides along the
/// extreme bends, its far boundary the end sections of every bend (each square to the bend's axis,
/// through the point where its tangent line meets its neighbour's). The bands cut by it do not
/// cross.
#[derive(Clone, Debug)]
pub(crate) struct Relief {
    /// The vertex of `a`.
    pub(crate) at: Vec3,
    /// The point on each fan facet where the two bends' tangent lines meet, one per facet.
    pub(crate) q: Vec<Vec3>,
    /// The bend at each rib, `m + 1` of them in the fan's order (D264).
    pub(crate) ribs: Vec<usize>,
    /// The end plane of each bend, `(axis, a point of the plane)`: the bend's way from the vertex
    /// and the point `q` (the last bend, having no next, takes the last `q`).
    pub(crate) ends: Vec<(Vec3, Vec3)>,
    /// The two side planes, `(point, unit normal)`: each holds the vertex and one extreme bend's
    /// axis, square to the shell.
    pub(crate) sides: [(Vec3, Vec3); 2],
}

/// The разметка of a transition (D264): where every facet is, every bend, and the rims.
#[derive(Clone, Debug)]
pub(crate) struct Layout {
    pub(crate) vertices: BTreeMap<TopoId, Vec3>,
    /// The first profile's vertices, in order round it.
    pub(crate) a: Vec<Vec3>,
    /// The apex point of each flat triangle, over the middle of each piece of `a`.
    pub(crate) b: Vec<Vec3>,
    /// The number of chords of the fan at each vertex of `a`.
    pub(crate) fans: Vec<usize>,
    pub(crate) facets: Vec<Facet>,
    pub(crate) bends: Vec<Bend>,
    /// The lengths of the chords along the second profile (the top rim).
    pub(crate) top_rim: Vec<f64>,
    /// The lengths of the pieces of the first profile (the bottom rim).
    pub(crate) bottom_rim: Vec<f64>,
    pub(crate) seam: Seam,
    /// The profiles' common normal, from the first plane toward the second.
    pub(crate) az: Vec3,
    /// The relief at each vertex of `a`, in its order.
    pub(crate) reliefs: Vec<Relief>,
}

/// Stable topological IDs shared by layout and body assembly.
#[derive(Clone, Copy)]
struct TransitionIds {
    feature: TopoId,
}

impl TransitionIds {
    fn a(self, i: usize) -> TopoId {
        TopoId::derive("transition.a").id(self.feature).index(i as u64).finish()
    }

    fn b(self, i: usize) -> TopoId {
        TopoId::derive("transition.b").id(self.feature).index(i as u64).finish()
    }

    fn fan(self, j: usize, k: usize) -> TopoId {
        TopoId::derive("transition.fan").id(self.feature).index(j as u64).index(k as u64).finish()
    }

    fn fan_vertex(self, j: usize, k: usize, m: usize, n: usize) -> TopoId {
        if k == 0 {
            self.b((j + n - 1) % n)
        } else if k == m {
            self.b(j)
        } else {
            self.fan(j, k)
        }
    }

    #[cfg(test)]
    fn inner(self, v: TopoId) -> TopoId {
        TopoId::derive("transition.inner").id(self.feature).id(v).finish()
    }
}

// ---------------------------------------------------------------------------------------------
// Small vector arithmetic in the plane and in space.

fn sub(a: Vec2, b: Vec2) -> Vec2 {
    Vec2::new(a.x - b.x, a.y - b.y)
}

fn add(a: Vec2, b: Vec2) -> Vec2 {
    Vec2::new(a.x + b.x, a.y + b.y)
}

fn mul(a: Vec2, s: f64) -> Vec2 {
    Vec2::new(a.x * s, a.y * s)
}

fn dot2(a: Vec2, b: Vec2) -> f64 {
    a.x * b.x + a.y * b.y
}

fn cross2(a: Vec2, b: Vec2) -> f64 {
    a.x * b.y - a.y * b.x
}

fn len2(a: Vec2) -> f64 {
    libm::hypot(a.x, a.y)
}

fn unit2(a: Vec2) -> Vec2 {
    let l = len2(a);
    Vec2::new(a.x / l, a.y / l)
}

fn mid(a: Vec2, b: Vec2) -> Vec2 {
    Vec2::new(0.5 * (a.x + b.x), 0.5 * (a.y + b.y))
}

fn unit3(a: Vec3) -> Vec3 {
    a * (1.0 / a.dot(a).sqrt())
}

fn average(points: &[Vec3]) -> Vec3 {
    let mut s = Vec3::ZERO;
    for &p in points {
        s = s + p;
    }
    s * (1.0 / points.len() as f64)
}

/// A point of the first sketch in the plane of the second.
fn to3(p: Vec2, frame: [Vec3; 4]) -> Vec3 {
    frame[0] + frame[1] * p.x + frame[2] * p.y
}

/// A point in the first sketch's coordinates.
fn to2(p: Vec3, frame: [Vec3; 4]) -> Vec2 {
    let w = p - frame[0];
    Vec2::new(w.dot(frame[1]), w.dot(frame[2]))
}

fn direction2(d: Vec3, frame: [Vec3; 4]) -> Vec2 {
    Vec2::new(d.dot(frame[1]), d.dot(frame[2]))
}

// ---------------------------------------------------------------------------------------------
// Profiles: the pieces of a closed profile, its orientation, and its smooth runs.

/// The closed profile `name` of sketch `s`, as its pieces in the sketch's own coordinates.
fn profile(feature: TopoId, s: &Placed, name: &str, lines_only: bool) -> Result<Vec<Piece>, BrepError> {
    if s.developed.is_some() {
        return Err(BrepError::Unsupported("a transition's profiles lie on planes, not on a rolled face".into()));
    }
    let chosen = chosen(feature, s, &[name.to_string()])?;
    let mut regions = profile_regions(feature, &chosen)?;
    if regions.len() != 1 {
        return Err(BrepError::Unsupported(format!(
            "a transition's profile is one closed contour, `{name}` names {}: choose a curve of one",
            regions.len()
        )));
    }
    let region = regions.remove(0);
    if !region.holes.is_empty() {
        return Err(BrepError::Unsupported("a transition's profile has no holes yet".into()));
    }
    if lines_only && region.outer.arcs.iter().any(Option::is_some) {
        return Err(BrepError::Unsupported(format!(
            "a transition's first profile is straight pieces: an arc among them (`{name}`'s contour) comes later"
        )));
    }
    let ProfileLoop { points, arcs, .. } = region.outer;
    let n = points.len();
    Ok((0..n).map(|k| Piece { from: points[k], to: points[(k + 1) % n], arc: arcs[k] }).collect())
}

/// The same pieces the other way round: every arc's way round flips.
fn reversed(pieces: &[Piece]) -> Vec<Piece> {
    pieces.iter().rev().map(|p| Piece { from: p.to, to: p.from, arc: p.arc.map(|a| ProfileArc { center: a.center, ccw: !a.ccw }) }).collect()
}

/// The pieces of a profile of the second sketch in the first sketch's coordinates. `flip` says the
/// map from the second sketch's plane to the first's turns an arc's way round.
fn mapped(pieces: &[Piece], from: [Vec3; 4], to: [Vec3; 4], flip: bool) -> Vec<Piece> {
    pieces
        .iter()
        .map(|p| Piece {
            from: to2(to3(p.from, from), to),
            to: to2(to3(p.to, from), to),
            arc: p.arc.map(|a| ProfileArc { center: to2(to3(a.center, from), to), ccw: a.ccw != flip }),
        })
        .collect()
}

/// Twice the signed area of the polygon through the pieces' starts.
fn shoelace(points: &[Vec2]) -> f64 {
    let n = points.len();
    (0..n).fold(0.0, |s, k| {
        let (a, b) = (points[k], points[(k + 1) % n]);
        s + (a.x * b.y - a.y * b.x)
    })
}

/// The angle swept from `from` to `to` about `center` the way `ccw` says, in `[0, 2π)`.
fn swept(from: Vec2, to: Vec2, center: Vec2, ccw: bool) -> f64 {
    let a0 = libm::atan2(from.y - center.y, from.x - center.x);
    let a1 = libm::atan2(to.y - center.y, to.x - center.x);
    let d = if ccw { a1 - a0 } else { a0 - a1 };
    d.rem_euclid(2.0 * PI)
}

/// The pieces of a profile as smooth runs: neighbouring arcs of one circle, or collinear lines,
/// merge. A full circle is one run from its first point back to itself, `span` 2π.
fn smooth(pieces: &[Piece], size: f64) -> Vec<Smooth> {
    let mut out: Vec<Smooth> = Vec::new();
    for p in pieces {
        let kind = match p.arc {
            None => Kind::Line,
            Some(a) => Kind::Arc { center: a.center, radius: len2(sub(p.from, a.center)), ccw: a.ccw },
        };
        let span = match kind {
            Kind::Line => len2(sub(p.to, p.from)),
            Kind::Arc { .. } if p.from == p.to => 2.0 * PI,
            Kind::Arc { center, ccw, .. } => swept(p.from, p.to, center, ccw),
        };
        if let Some(last) = out.last_mut() {
            let joins = within_rounding(len2(sub(last.to, p.from)), size);
            let same = match (last.kind, kind) {
                (Kind::Line, Kind::Line) => {
                    let a = sub(last.to, last.from);
                    let b = sub(p.to, p.from);
                    within_rounding(cross2(a, b), size * size) && dot2(a, b) > 0.0
                }
                (Kind::Arc { center: c0, radius: r0, ccw: k0 }, Kind::Arc { center: c1, radius: r1, ccw: k1 }) => {
                    within_rounding(len2(sub(c0, c1)), size) && within_rounding(r0 - r1, size) && k0 == k1
                }
                _ => false,
            };
            if joins && same {
                last.to = p.to;
                last.span += span;
                continue;
            }
        }
        out.push(Smooth { from: p.from, to: p.to, kind, span });
    }
    out
}

impl Smooth {
    /// The point at `t` along it, `t` in `[0, 1]`.
    fn point(&self, t: f64) -> Vec2 {
        match self.kind {
            Kind::Line => add(self.from, mul(sub(self.to, self.from), t)),
            Kind::Arc { center, radius, ccw } => {
                let a0 = libm::atan2(self.from.y - center.y, self.from.x - center.x);
                let a = a0 + (if ccw { 1.0 } else { -1.0 }) * t * self.span;
                let (s, c) = libm::sincos(a);
                Vec2::new(center.x + radius * c, center.y + radius * s)
            }
        }
    }

    /// Where `q`, a point of it, lies along it.
    fn at(&self, q: Vec2, size: f64) -> Option<f64> {
        match self.kind {
            Kind::Line => {
                let d = sub(self.to, self.from);
                let l2 = dot2(d, d);
                if l2 == 0.0 {
                    return None;
                }
                let t = dot2(sub(q, self.from), d) / l2;
                let off = cross2(d, sub(q, self.from));
                ((0.0..=1.0).contains(&t) && within_rounding(off, size * size)).then_some(t)
            }
            Kind::Arc { center, radius, ccw } => {
                if within_rounding(len2(sub(q, center)) - radius, size) {
                    let a = swept(self.from, q, center, ccw);
                    if a <= self.span + 64.0 * f64::EPSILON || within_rounding(self.span - 2.0 * PI, 1.0) {
                        return Some(if self.span > 0.0 { a / self.span } else { 0.0 });
                    }
                }
                None
            }
        }
    }

    /// The point of it nearest `m`, and where it lies along it. None if every point is as near
    /// (`m` is the arc's centre).
    fn closest(&self, m: Vec2) -> Option<(Vec2, f64)> {
        match self.kind {
            Kind::Line => {
                let d = sub(self.to, self.from);
                let l2 = dot2(d, d);
                if l2 == 0.0 {
                    return Some((self.from, 0.0));
                }
                let t = (dot2(sub(m, self.from), d) / l2).clamp(0.0, 1.0);
                Some((add(self.from, mul(d, t)), t))
            }
            Kind::Arc { center, radius, .. } => {
                let w = sub(m, center);
                if w.x == 0.0 && w.y == 0.0 {
                    return None;
                }
                let q = add(center, mul(unit2(w), radius));
                match self.at(q, radius.max(1.0)) {
                    Some(t) => Some((q, t)),
                    None => {
                        let (a, b) = (len2(sub(m, self.from)), len2(sub(m, self.to)));
                        Some(if a <= b { (self.from, 0.0) } else { (self.to, 1.0) })
                    }
                }
            }
        }
    }
}

/// The point of the boundary nearest `m`, and where it is: its run and the place along it. Refused
/// when two points of the boundary are as near (the apex over a piece's middle is not unique).
fn nearest(boundary: &[Smooth], m: Vec2, size: f64) -> Result<(Vec2, usize, f64), BrepError> {
    let mut found: Vec<(f64, Vec2, usize, f64)> = Vec::new();
    for (k, s) in boundary.iter().enumerate() {
        let Some((q, t)) = s.closest(m) else {
            return Err(BrepError::Degenerate("a transition's apex over a piece's middle is not unique: the middle is an arc's centre".into()));
        };
        let d = sub(q, m);
        found.push((dot2(d, d), q, k, t));
    }
    let least = found.iter().map(|f| f.0).fold(f64::INFINITY, f64::min);
    let tol = 64.0 * f64::EPSILON * size * size;
    let near: Vec<&(f64, Vec2, usize, f64)> = found.iter().filter(|f| f.0 <= least + tol).collect();
    let mut points: Vec<Vec2> = Vec::new();
    for f in &near {
        if !points.iter().any(|p| within_rounding(len2(sub(*p, f.1)), size)) {
            points.push(f.1);
        }
    }
    if points.len() > 1 {
        return Err(BrepError::Degenerate(format!(
            "a transition's apex over a piece's middle is not unique: {} points of the second profile are as near ({:.3} mm)",
            points.len(),
            least.sqrt()
        )));
    }
    let Some(pick) = near.iter().filter(|f| within_rounding(len2(sub(f.1, points[0])), size)).min_by_key(|f| f.2).copied() else {
        return Err(BrepError::Degenerate("a transition's apex over a piece's middle is not unique".into()));
    };
    Ok((pick.1, pick.2, pick.3))
}

/// How many chords an arc of `radius` and angle `delta` takes so that each strays no more than
/// `tolerance` from it (D229: a fixed, deterministic count).
fn chords(delta: f64, radius: f64, tolerance: f64) -> Result<usize, BrepError> {
    if delta <= 0.0 {
        return Ok(1);
    }
    let mut m = 1.0f64;
    if tolerance < radius {
        let half = libm::acos(1.0 - tolerance / radius);
        if half > 0.0 {
            m = (delta / (2.0 * half)).ceil().max(1.0);
        }
    }
    // A chord never spans a half turn or more: its sagitta formula holds under one.
    m = m.max((delta / PI).ceil().max(1.0));
    if !(m.is_finite() && m <= 4096.0) {
        return Err(BrepError::Degenerate(format!(
            "a transition's tolerance of {tolerance} mm is too fine for an arc of {radius} mm: more than 4096 chords"
        )));
    }
    Ok(m as usize)
}

/// The chords of one run from `t0` to `t1`, the first point left out, the last in.
fn run_chords(s: &Smooth, t0: f64, t1: f64, tolerance: f64) -> Result<Vec<Vec2>, BrepError> {
    match s.kind {
        Kind::Line => {
            if t1 - t0 >= 1.0 {
                return Err(BrepError::Degenerate("a transition's fan runs round the whole second profile".into()));
            }
            Ok(vec![s.point(t1)])
        }
        Kind::Arc { radius, .. } => {
            let delta = (t1 - t0) * s.span;
            let m = chords(delta, radius, tolerance)?;
            Ok((1..=m).map(|j| s.point(t0 + (t1 - t0) * j as f64 / m as f64)).collect())
        }
    }
}

/// The chords from `start` to `end` along the boundary, the way it runs: the first point left
/// out (the caller has it), the last in. The ends are `(run, place along it)`.
fn split(boundary: &[Smooth], start: (usize, f64), end: (usize, f64), tolerance: f64) -> Result<Vec<Vec2>, BrepError> {
    if start.0 == end.0 {
        if end.1 > start.1 {
            return run_chords(&boundary[start.0], start.1, end.1, tolerance);
        }
        // The same run again after going round: only a full circle is one run.
        let s = &boundary[start.0];
        if !within_rounding(s.span - 2.0 * PI, 1.0) {
            return Err(BrepError::Degenerate("a transition's fan wraps round a run that is no circle".into()));
        }
        return run_chords(s, start.1, end.1 + 1.0, tolerance);
    }
    let n = boundary.len();
    let mut out: Vec<Vec2> = Vec::new();
    let mut k = start.0;
    loop {
        let s = &boundary[k];
        let t0 = if k == start.0 { start.1 } else { 0.0 };
        let t1 = if k == end.0 { end.1 } else { 1.0 };
        if t1 > t0 {
            out.extend(run_chords(s, t0, t1, tolerance)?);
        }
        if k == end.0 {
            break;
        }
        k = (k + 1) % n;
        if k == start.0 {
            return Err(BrepError::Degenerate("a transition's fan runs round the whole second profile".into()));
        }
    }
    Ok(out)
}

/// The minimum-norm point `x` with `n1·x = b1` and `n2·x = b2`.
fn solve(n1: Vec3, b1: f64, n2: Vec3, b2: f64) -> Result<Vec3, BrepError> {
    let c = n1.dot(n2);
    let det = 1.0 - c * c;
    if det <= 64.0 * f64::EPSILON {
        return Err(BrepError::Degenerate("a transition's two facets lie on one plane: no bend".into()));
    }
    let alpha = (b1 - b2 * c) / det;
    let beta = (b2 - b1 * c) / det;
    Ok(n1 * alpha + n2 * beta)
}

/// The outer surface's offset from the profiles, along their outward normals, and the inner's.
fn shifts(side: Side, t: f64) -> (f64, f64) {
    match side {
        Side::Left => (0.0, -t),
        Side::Right => (t, 0.0),
        Side::Middle => (0.5 * t, -0.5 * t),
    }
}

/// Clips `poly` to `(p - at)·n ≥ 0`.
fn clip(poly: &[Vec3], at: Vec3, n: Vec3) -> Vec<Vec3> {
    let mut out: Vec<Vec3> = Vec::with_capacity(poly.len() + 1);
    for k in 0..poly.len() {
        let (a, b) = (poly[k], poly[(k + 1) % poly.len()]);
        let (da, db) = ((a - at).dot(n), (b - at).dot(n));
        if da >= 0.0 {
            out.push(a);
        }
        if (da > 0.0 && db < 0.0) || (da < 0.0 && db > 0.0) {
            out.push(a + (b - a) * (da / (da - db)));
        }
    }
    out
}

/// Where two lines meet: the point of `p0 + d0·s` = `p1 + d1·u` (both in one plane, not parallel).
fn meet(p0: Vec3, d0: Vec3, p1: Vec3, d1: Vec3) -> Result<Vec3, BrepError> {
    let n = d0.cross(d1);
    let denom = n.dot(n);
    if denom == 0.0 || !denom.is_finite() {
        return Err(BrepError::Degenerate("transition: two tangent lines are parallel: their bends' trims do not meet".into()));
    }
    let s = (p1 - p0).cross(d1).dot(n) / denom;
    if !s.is_finite() {
        return Err(BrepError::Degenerate("transition: two tangent lines meet nowhere".into()));
    }
    Ok(p0 + d0 * s)
}

/// How far a bend's end face reaches over the top profile's plane, as `(m, rho)`: the greatest
/// `dir·az` over the band's span and the radius it is taken at (the outer arc when the span faces
/// up, the inner one when the whole span faces down). `m` is 0 for an axis square to the profiles.
fn span_top(b: &Bend, radius_in: f64, radius_out: f64, az: Vec3) -> (f64, f64) {
    let u_of = |t: Vec3| -> Vec3 {
        let w = t - b.at;
        unit3(w - b.axis * w.dot(b.axis))
    };
    let (u0, u1) = (u_of(b.tangent[0]), u_of(b.tangent[1]));
    let n01 = u0.cross(u1);
    let m = if nearly_parallel(u0, u1) {
        u0.dot(az)
    } else {
        let w = az - b.axis * b.axis.dot(az);
        let inside = w.dot(w) > 0.0 && {
            let phi = libm::atan2(u0.cross(unit3(w)).dot(unit3(n01)), u0.dot(unit3(w)));
            0.0 <= phi && phi <= b.sweep
        };
        if inside { unit3(w).dot(az) } else { u0.dot(az).max(u1.dot(az)) }
    };
    (m, if m >= 0.0 { radius_out } else { radius_in })
}

/// A bend's lower end section: one plane perpendicular to its axis, with the two
/// concentric arcs where it meets the outer and inner cylindrical surfaces.
#[derive(Clone, Copy, Debug)]
struct BendSection {
    /// Signed distance along the unit bend axis from Bend::at.
    station: f64,
    /// The section circle's centre on the bend axis.
    center: Vec3,
    /// Endpoints on the outer cylinder, one on each adjacent facet.
    outer: [Vec3; 2],
    /// Endpoints on the inner cylinder, one on each adjacent facet.
    inner: [Vec3; 2],
}

#[derive(Clone, Copy, Debug)]
struct FacetSection {
    outer: Vec3,
    inner: Vec3,
}

/// The two tangent boundaries of one trimmed planar facet, including its lower and upper
/// cylinder intersections. q is the tangent-line intersection; x lies on the opposite rim edge.
#[derive(Clone, Copy, Debug)]
struct FacetTrim {
    facet: usize,
    bends: [usize; 2],
    q: Vec3,
    x: [Vec3; 2],
    lower: [FacetSection; 2],
    upper: [FacetSection; 2],
}

#[derive(Clone, Copy, Debug)]
struct PatchVertex {
    outer_id: TopoId,
    inner_id: TopoId,
    outer: Vec3,
    inner: Vec3,
}

/// One trimmed planar facet. Its six vertices run: lower/upper on the outgoing bend,
/// outgoing rim split, incoming rim split, upper/lower on the incoming bend.
#[derive(Clone, Debug)]
struct FacetPatch {
    facet: usize,
    bends: [usize; 2],
    boundary: Vec<PatchVertex>,
}

fn distance3(a: Vec3, b: Vec3) -> f64 {
    let d = a - b;
    d.dot(d).sqrt()
}

/// Places a section through point. point belongs to the tangent generator on seed_side;
/// the other generator is intersected with the same perpendicular plane, not tested against
/// point itself. The thickness offsets carry the two radial endpoints from each facet plane
/// onto the two cylindrical surfaces.
fn bend_section(bend: &Bend, point: Vec3, seed_side: usize, outer_offset: f64, inner_offset: f64, size: f64) -> Result<BendSection, BrepError> {
    if seed_side >= 2 {
        return Err(BrepError::Invalid("transition: a lower bend section names no tangent side".into()));
    }
    if !within_rounding(bend.axis.dot(bend.axis) - 1.0, 1.0) {
        return Err(BrepError::Invalid("transition: a lower bend section has a non-unit axis".into()));
    }
    let station = (point - bend.at).dot(bend.axis);
    let center = bend.at + bend.axis * station;
    let mut outer = [Vec3::ZERO; 2];
    let mut inner = [Vec3::ZERO; 2];
    for side in 0..2 {
        let radial = bend.tangent[side] - bend.at;
        if !within_rounding(radial.dot(bend.axis), size) {
            return Err(BrepError::Invalid("transition: a tangent generator is not perpendicular to its bend axis".into()));
        }
        let radius = radial.dot(radial).sqrt();
        if !(radius > 0.0 && radius.is_finite()) {
            return Err(BrepError::Degenerate("transition: a bend has a zero tangent radius".into()));
        }
        let direction = radial * (1.0 / radius);
        if !within_rounding(radius + outer_offset - bend.radius_out, size) || !within_rounding(radius + inner_offset - bend.radius_in, size) {
            return Err(BrepError::Invalid("transition: a bend section's thickness offsets miss its cylinder radii".into()));
        }

        // Each tangent generator is parallel to the bend axis. The section plane fixes the same
        // axial parameter on both generators; only the seed side contains point by definition.
        let generator = bend.tangent[side] + bend.axis * station;
        if side == seed_side && !within_rounding(distance3(generator, point), size) {
            return Err(BrepError::Invalid("transition: the relief point is off its selected tangent generator".into()));
        }
        outer[side] = center + direction * bend.radius_out;
        inner[side] = center + direction * bend.radius_in;
        if !within_rounding(distance3(outer[side], generator + direction * outer_offset), size)
            || !within_rounding(distance3(inner[side], generator + direction * inner_offset), size)
        {
            return Err(BrepError::Invalid("transition: a bend section does not meet its offset tangent generators".into()));
        }
        if !within_rounding((outer[side] - center).dot(bend.axis), size) || !within_rounding((inner[side] - center).dot(bend.axis), size) {
            return Err(BrepError::Invalid("transition: a bend section endpoint is off its perpendicular plane".into()));
        }
    }
    Ok(BendSection { station, center, outer, inner })
}

fn bend_section_at_station(bend: &Bend, station: f64, outer_offset: f64, inner_offset: f64, size: f64) -> Result<BendSection, BrepError> {
    if !station.is_finite() || !within_rounding(bend.axis.dot(bend.axis) - 1.0, 1.0) {
        return Err(BrepError::Invalid("transition: a bend section has an invalid station or axis".into()));
    }
    let center = bend.at + bend.axis * station;
    let mut outer = [Vec3::ZERO; 2];
    let mut inner = [Vec3::ZERO; 2];
    for side in 0..2 {
        let radial = bend.tangent[side] - bend.at;
        if !within_rounding(radial.dot(bend.axis), size) {
            return Err(BrepError::Invalid("transition: a tangent generator is not perpendicular to its bend axis".into()));
        }
        let radius = radial.dot(radial).sqrt();
        if !(radius > 0.0 && radius.is_finite()) {
            return Err(BrepError::Degenerate("transition: a bend has a zero tangent radius".into()));
        }
        let direction = radial * (1.0 / radius);
        if !within_rounding(radius + outer_offset - bend.radius_out, size) || !within_rounding(radius + inner_offset - bend.radius_in, size) {
            return Err(BrepError::Invalid("transition: a bend section's thickness offsets miss its cylinder radii".into()));
        }
        outer[side] = center + direction * bend.radius_out;
        inner[side] = center + direction * bend.radius_in;
        let generator = bend.tangent[side] + bend.axis * station;
        if !within_rounding(distance3(outer[side], generator + direction * outer_offset), size)
            || !within_rounding(distance3(inner[side], generator + direction * inner_offset), size)
        {
            return Err(BrepError::Invalid("transition: a bend section does not meet its offset tangent generators".into()));
        }
        if !within_rounding((outer[side] - center).dot(bend.axis), size) || !within_rounding((inner[side] - center).dot(bend.axis), size) {
            return Err(BrepError::Invalid("transition: a bend section endpoint is off its perpendicular plane".into()));
        }
    }
    Ok(BendSection { station, center, outer, inner })
}

fn station_between(value: f64, a: f64, b: f64, size: f64) -> bool {
    let lo = a.min(b);
    let hi = a.max(b);
    let above_lower = value >= lo || within_rounding(value - lo, size);
    let below_upper = value <= hi || within_rounding(value - hi, size);
    above_lower && below_upper
}

/// Resolves the lower end plane of every bend from its relief q point. A fan facet selects the
/// tangent generator that contains q; the opposite generator is intersected at the same axial
/// station. The resulting sections are indexed by bend, independently of relief order.
fn lower_sections(layout: &Layout, dims: &TransitionDims) -> Result<Vec<BendSection>, BrepError> {
    let n = layout.a.len();
    if layout.reliefs.len() != n || layout.fans.len() != n {
        return Err(BrepError::Invalid("transition: lower bend sections have incomplete relief data".into()));
    }
    let (outer_offset, inner_offset) = shifts(dims.side, dims.thickness);
    let size = layout
        .vertices
        .values()
        .flat_map(|p| [p.x, p.y, p.z])
        .chain(layout.a.iter().chain(&layout.b).flat_map(|p| [p.x, p.y, p.z]))
        .fold(1.0_f64, |m, x| m.max(x.abs()));
    let mut sections: Vec<Option<BendSection>> = vec![None; layout.bends.len()];
    let mut fan_base = n;
    for (j, relief) in layout.reliefs.iter().enumerate() {
        let m = layout.fans[j];
        if m == 0 || relief.ribs.len() != m + 1 || relief.ends.len() != m + 1 {
            return Err(BrepError::Invalid(format!("transition: relief {j} cannot define its lower bend sections")));
        }
        for k in 0..=m {
            let bend_index = relief.ribs[k];
            let bend =
                layout.bends.get(bend_index).ok_or_else(|| BrepError::Invalid(format!("transition: relief {j} names missing bend {bend_index}")))?;
            let fan_facet = fan_base + k.min(m - 1);
            let seed_side = bend
                .facets
                .iter()
                .position(|&facet| facet == fan_facet)
                .ok_or_else(|| BrepError::Invalid(format!("transition: bend {bend_index} does not border its relief's fan facet")))?;
            let (relief_axis, point) = relief.ends[k];
            if !nearly_parallel(relief_axis, bend.axis) {
                return Err(BrepError::Invalid(format!("transition: relief {j}'s lower section is not square to bend {bend_index}")));
            }
            let section = bend_section(bend, point, seed_side, outer_offset, inner_offset, size)?;
            if !section.station.is_finite() || !within_rounding(distance3(section.center, bend.at + bend.axis * section.station), size) {
                return Err(BrepError::Invalid(format!("transition: bend {bend_index}'s lower section is not on its axis")));
            }
            for side in 0..2 {
                if !within_rounding(distance3(section.outer[side], section.center) - bend.radius_out, size)
                    || !within_rounding(distance3(section.inner[side], section.center) - bend.radius_in, size)
                    || !within_rounding(distance3(section.outer[side], section.inner[side]) - dims.thickness, size)
                    || !within_rounding((section.outer[side] - section.center).dot(bend.axis), size)
                    || !within_rounding((section.inner[side] - section.center).dot(bend.axis), size)
                {
                    return Err(BrepError::Invalid(format!(
                        "transition: bend {bend_index}'s lower section misses a cylinder or its perpendicular plane"
                    )));
                }
            }

            // The lower section must lie on the finite rib, before its D264/D265 upper section.
            let p0 =
                *layout.vertices.get(&bend.edge[0]).ok_or_else(|| BrepError::Invalid("transition: lower section's bend vertex is missing".into()))?;
            let p1 =
                *layout.vertices.get(&bend.edge[1]).ok_or_else(|| BrepError::Invalid("transition: lower section's bend vertex is missing".into()))?;
            let far = if distance3(p0, relief.at) <= distance3(p1, relief.at) { p1 } else { p0 };
            let length = (far - relief.at).dot(relief_axis);
            let lower = (section.center - relief.at).dot(relief_axis);
            let upper = (bend.top - relief.at).dot(relief_axis);
            if !(length > 0.0 && lower > 0.0 && lower < upper && upper < length) {
                return Err(BrepError::Invalid(format!("transition: bend {bend_index}'s lower section is not between its corner and upper section")));
            }
            if sections[bend_index].replace(section).is_some() {
                return Err(BrepError::Invalid(format!("transition: bend {bend_index} has more than one lower section")));
            }
        }
        fan_base += m;
    }
    if fan_base != layout.facets.len() {
        return Err(BrepError::Invalid("transition: reliefs do not cover all fan facets".into()));
    }
    sections
        .into_iter()
        .enumerate()
        .map(|(i, section)| section.ok_or_else(|| BrepError::Invalid(format!("transition: bend {i} has no lower section"))))
        .collect()
}

/// The разметка of the transition between profile `a_name` of sketch `a` and `b_name` of `b`.
pub(crate) fn layout(feature: TopoId, a: &Placed, a_name: &str, b: &Placed, b_name: &str, dims: &TransitionDims) -> Result<Layout, BrepError> {
    let [ao, _ax, _ay, az] = a.frame;
    let bf = b.frame;
    if !nearly_parallel(az, bf[3]) {
        return Err(BrepError::Unsupported("a transition's two profiles lie on parallel planes: these planes meet".into()));
    }
    let h = (bf[0] - ao).dot(az);
    if h == 0.0 {
        return Err(BrepError::Degenerate("a transition's two profiles lie on one plane".into()));
    }
    // The profiles in the first sketch's coordinates, both running counterclockwise there.
    let mut pa = profile(feature, a, a_name, true)?;
    let pb = profile(feature, b, b_name, false)?;
    let det = cross2(direction2(bf[1], a.frame), direction2(bf[2], a.frame));
    let mut pb = mapped(&pb, bf, a.frame, det < 0.0);
    let a_pts: Vec<Vec2> = pa.iter().map(|p| p.from).collect();
    if shoelace(&a_pts) < 0.0 {
        pa = reversed(&pa);
    }
    let b_pts: Vec<Vec2> = pb.iter().map(|p| p.from).collect();
    if shoelace(&b_pts) < 0.0 {
        pb = reversed(&pb);
    }
    let n = pa.len();
    if n < 3 {
        return Err(BrepError::Degenerate(format!("a transition's first profile has {n} pieces: a contour needs at least 3")));
    }
    let size = a_pts.iter().chain(&b_pts).fold(1.0_f64, |m, p| m.max(p.x.abs()).max(p.y.abs()));
    let boundary = smooth(&pb, size);
    if boundary.is_empty() {
        return Err(BrepError::Sketch("a transition's second profile has no pieces".into()));
    }
    // The apex over each piece's middle.
    let a2: Vec<Vec2> = pa.iter().map(|p| p.from).collect();
    let mut b2: Vec<Vec2> = Vec::with_capacity(n);
    let mut at: Vec<(usize, f64)> = Vec::with_capacity(n);
    for i in 0..n {
        let m = mid(a2[i], a2[(i + 1) % n]);
        let (q, k, t) = nearest(&boundary, m, size)?;
        b2.push(q);
        at.push((k, t));
    }
    // The apex points follow the second profile's pieces as the first profile's do: a layout that
    // crosses itself cannot be developed.
    let glob: Vec<f64> = at.iter().map(|&(k, t)| k as f64 + t).collect();
    let mut order: Vec<usize> = (0..n).collect();
    order.sort_by(|&i, &j| glob[i].total_cmp(&glob[j]));
    if (0..n).any(|k| order[(k + 1) % n] != (order[k] + 1) % n) {
        return Err(BrepError::Degenerate(
            "a transition's разметка crosses: the apex points do not follow the second profile's pieces in order".into(),
        ));
    }
    // The fans: the chords of the second profile's arc over each vertex of the first.
    let mut fans: Vec<usize> = Vec::with_capacity(n);
    let mut fan_pts: Vec<Vec<Vec2>> = Vec::with_capacity(n);
    for j in 0..n {
        let (start, end) = (at[(j + n - 1) % n], at[j]);
        if start.0 == end.0 && within_rounding(start.1 - end.1, 1.0) {
            return Err(BrepError::Degenerate(
                "a transition's two apex points over neighbouring pieces are one point: the fan there has no arc".into(),
            ));
        }
        let mut pts = vec![b2[(j + n - 1) % n]];
        pts.extend(split(&boundary, start, end, dims.tolerance)?);
        fans.push(pts.len() - 1);
        fan_pts.push(pts);
    }
    // The points in space.
    let to_b = |p: Vec2| to3(p, a.frame) + az * h;
    let a3: Vec<Vec3> = a2.iter().map(|&p| to3(p, a.frame)).collect();
    let b3: Vec<Vec3> = b2.iter().map(|&p| to_b(p)).collect();
    let fan3: Vec<Vec<Vec3>> = fan_pts.iter().map(|pts| pts.iter().map(|&p| to_b(p)).collect()).collect();
    // The vertices and the facets, each facing out of the body.
    let ids = TransitionIds { feature };
    let mut vertices: BTreeMap<TopoId, Vec3> = BTreeMap::new();
    for i in 0..n {
        vertices.insert(ids.a(i), a3[i]);
        vertices.insert(ids.b(i), b3[i]);
    }
    for (j, pts) in fan3.iter().enumerate() {
        for (k, &point) in pts.iter().enumerate().skip(1).take(pts.len().saturating_sub(2)) {
            vertices.insert(ids.fan(j, k), point);
        }
    }
    let fan_vid = |j: usize, k: usize, m: usize| ids.fan_vertex(j, k, m, n);
    let axis_center = average(&a3);
    let mut facets: Vec<Facet> = Vec::with_capacity(n + fans.iter().sum::<usize>());
    let orient = |ids: Vec<TopoId>, pts: &[Vec3], top: Vec<TopoId>| -> Result<Facet, BrepError> {
        let nrm = (pts[1] - pts[0]).cross(pts[2] - pts[0]);
        if nrm.dot(nrm) == 0.0 {
            return Err(BrepError::Degenerate("a transition's facet has no area".into()));
        }
        let c = average(pts);
        let radial = c - axis_center - az * (c - axis_center).dot(az);
        let mut ids = ids;
        let mut nrm = nrm;
        if nrm.dot(radial) < 0.0 {
            ids.reverse();
            nrm = -nrm;
        }
        Ok(Facet { loop_: ids, origin: pts[0], normal: unit3(nrm), top })
    };
    // The flat triangle over each piece of `a`.
    for i in 0..n {
        let pts = [a3[i], a3[(i + 1) % n], b3[i]];
        facets.push(orient(vec![ids.a(i), ids.a((i + 1) % n), ids.b(i)], &pts, vec![ids.b(i)])?);
    }
    // The fan over each vertex of `a`.
    for j in 0..n {
        let m = fans[j];
        for k in 0..m {
            let pts = [a3[j], fan3[j][k + 1], fan3[j][k]];
            facets.push(orient(vec![ids.a(j), fan_vid(j, k + 1, m), fan_vid(j, k, m)], &pts, vec![fan_vid(j, k, m), fan_vid(j, k + 1, m)])?);
        }
    }
    // Every edge of the facets: two of them is a bend, one a rim.
    let mut edges: BTreeMap<(TopoId, TopoId), Vec<usize>> = BTreeMap::new();
    for (fi, f) in facets.iter().enumerate() {
        for k in 0..f.loop_.len() {
            let (u, v) = (f.loop_[k], f.loop_[(k + 1) % f.loop_.len()]);
            edges.entry((u.min(v), u.max(v))).or_default().push(fi);
        }
    }
    let (outer, _) = shifts(dims.side, dims.thickness);
    let radius_out = dims.radius + dims.thickness;
    let radius_in = dims.radius;
    let mut bends: Vec<Bend> = Vec::new();
    // The tangents each facet is trimmed at, by facet: `(point, direction away from the edge)`.
    let mut trims: Vec<Vec<(Vec3, Vec3)>> = vec![Vec::new(); facets.len()];
    for ((u, v), faces) in &edges {
        match faces.as_slice() {
            [fi, gi] => {
                let p1 = *vertices.get(u).ok_or_else(|| BrepError::Invalid("transition: a facet names no vertex".into()))?;
                let p2 = *vertices.get(v).ok_or_else(|| BrepError::Invalid("transition: a facet names no vertex".into()))?;
                let d = unit3(p2 - p1);
                let (f, g) = (&facets[*fi], &facets[*gi]);
                let bf = f.origin.dot(f.normal) + outer - radius_out;
                let bg = g.origin.dot(g.normal) + outer - radius_out;
                let x0 = solve(f.normal, bf, g.normal, bg)?;
                let m = (p1 + p2) * 0.5;
                let at = x0 + d * (m - x0).dot(d);
                let tangent = [at + f.normal * (radius_out - outer), at + g.normal * (radius_out - outer)];
                let sweep = libm::acos(f.normal.dot(g.normal).clamp(-1.0, 1.0));
                for (facet, _) in [(*fi, tangent[0]), (*gi, tangent[1])] {
                    let nrm = facets[facet].normal;
                    let pts: Vec<Vec3> = facets[facet].loop_.iter().map(|x| vertices[x]).collect();
                    let c = average(&pts);
                    let foot = p1 + d * (c - p1).dot(d);
                    let away = c - foot;
                    if away.dot(away) > 0.0 {
                        trims[facet].push((at + nrm * (radius_out - outer), unit3(away)));
                    }
                }
                bends.push(Bend {
                    facets: [*fi, *gi],
                    edge: [*u, *v],
                    at,
                    axis: d,
                    radius_in,
                    radius_out,
                    tangent,
                    sweep,
                    top: Vec3::new(0.0, 0.0, 0.0),
                });
            }
            [.., _, _] => {
                return Err(BrepError::NonManifold(format!("a transition's edge {u:?} → {v:?} is used by {} facets", faces.len())));
            }
            _ => {}
        }
    }
    // The top end of every band (D264, option b): a band's meeting with the top profile's plane is
    // an ellipse, and the kernel keeps no such curves on a cylinder — the band stops at a plane
    // square to its axis, short of the rim. At the two extreme bends of a flat triangle, both stop
    // where the triangle's two tangent lines meet; at an inner bend, the plane goes through the
    // middle of the two points where its tangent lines run into the top chords.
    let mut apex_meet: Vec<Vec3> = Vec::with_capacity(n);
    for i in 0..n {
        let mut lines: Vec<(Vec3, Vec3)> = Vec::new();
        for b in &bends {
            if let Some(side) = b.facets.iter().position(|&f| f == i) {
                lines.push((b.tangent[side], b.axis));
            }
        }
        if lines.len() != 2 {
            return Err(BrepError::Invalid("transition: a flat triangle has not two bends".into()));
        }
        apex_meet.push(meet(lines[0].0, lines[0].1, lines[1].0, lines[1].1)?);
    }
    let mut tops: Vec<Vec3> = Vec::with_capacity(bends.len());
    for b in &bends {
        let [fi, gi] = b.facets;
        let top = if fi < n {
            apex_meet[fi]
        } else if gi < n {
            apex_meet[gi]
        } else {
            // An inner bend: the band stops where its end face's highest point touches the top
            // profile's plane — a higher cut would lift the blank's edge off the profile (D265).
            let (m, rho) = span_top(b, radius_in, radius_out, az);
            let axis_az = b.axis.dot(az);
            if within_rounding(axis_az, 1.0) {
                return Err(BrepError::Degenerate("a transition's rib runs parallel to the top profile's plane".into()));
            }
            let s = (-(b.at - b3[0]).dot(az) - rho * m) / axis_az;
            b.at + b.axis * s
        };
        tops.push(top);
    }
    for (b, top) in bends.iter_mut().zip(tops) {
        b.top = top;
    }
    // The relief at each vertex of `a`: the sector from it, its far boundary the bends' end
    // sections (D264). On each fan facet, the point where its two bends' tangent lines meet: a
    // bend's tangent line runs along its rib, through the axis's foot on the facet's own plane,
    // that far off the rib as (radius_out - outer)·tan(sweep/2).
    let by_edge: BTreeMap<(TopoId, TopoId), usize> =
        bends.iter().enumerate().map(|(i, b)| ((b.edge[0].min(b.edge[1]), b.edge[0].max(b.edge[1])), i)).collect();
    let trim = radius_out - outer;
    let mut reliefs: Vec<Relief> = Vec::with_capacity(n);
    let mut fan_base = n;
    for j in 0..n {
        let m = fans[j];
        // The bend at the rib out to `fan3[j][k]`: its index, axis point and the rib's direction.
        let bend_at_rib = |k: usize| -> Result<usize, BrepError> {
            let edge = (ids.a(j).min(fan_vid(j, k, m)), ids.a(j).max(fan_vid(j, k, m)));
            by_edge.get(&edge).copied().ok_or_else(|| BrepError::Invalid("transition: a fan edge has no bend".into()))
        };
        let ribs: Vec<usize> = (0..=m).map(bend_at_rib).collect::<Result<_, _>>()?;
        let rib = |k: usize| -> Result<(Vec3, Vec3), BrepError> {
            let b = &bends[ribs[k]];
            Ok((b.at, unit3(fan3[j][k] - a3[j])))
        };
        let mut q: Vec<Vec3> = Vec::with_capacity(m);
        for k in 0..m {
            let n = facets[fan_base + k].normal;
            let ((at0, d0), (at1, d1)) = (rib(k)?, rib(k + 1)?);
            if d0.dot(d1) <= 0.0 {
                return Err(BrepError::Degenerate("a transition's fan folds back on itself at a vertex".into()));
            }
            let (t0, t1) = (at0 + n * trim, at1 + n * trim);
            q.push(meet(t0, d0, t1, d1)?);
        }
        let mut ends: Vec<(Vec3, Vec3)> = Vec::with_capacity(m + 1);
        for k in 0..=m {
            let axis = unit3(fan3[j][k] - a3[j]);
            ends.push((axis, if k < m { q[k] } else { q[m - 1] }));
        }
        // The sides: the planes through the vertex and the extreme bends' axes, their normals
        // into the sector, toward the fan's middle.
        let mid = average(&fan3[j]) - a3[j];
        let side = |k: usize| -> Result<(Vec3, Vec3), BrepError> {
            let (at, axis) = rib(k)?;
            let normal = axis.cross(at - a3[j]);
            if normal.dot(normal) == 0.0 {
                return Err(BrepError::Degenerate("a transition's extreme bend runs through its vertex".into()));
            }
            let mut normal = unit3(normal);
            if normal.dot(mid) < 0.0 {
                normal = -normal;
            }
            Ok((a3[j], normal))
        };
        let sides = [side(0)?, side(m)?];
        reliefs.push(Relief { at: a3[j], q, ribs, ends, sides });
        fan_base += m;
    }
    // The rims, and the seam through the middle of the flat triangle over the first piece.
    let bottom_rim: Vec<f64> = (0..n).map(|i| (a3[(i + 1) % n] - a3[i]).dot(a3[(i + 1) % n] - a3[i]).sqrt()).collect();
    let top_rim: Vec<f64> = fan3.iter().flat_map(|pts| pts.windows(2)).map(|w| (w[1] - w[0]).dot(w[1] - w[0]).sqrt()).collect();
    let seam = Seam { facet: 0, from: (a3[0] + a3[1]) * 0.5, to: b3[0], gap: dims.gap };
    // A facet the trims leave nothing of: the bend radius is too large for it.
    for (fi, cut) in trims.iter().enumerate() {
        let pts: Vec<Vec3> = facets[fi].loop_.iter().map(|x| vertices[x]).collect();
        let mut kept = pts.clone();
        for &(at, away) in cut {
            kept = clip(&kept, at, away);
        }
        if kept.len() < 3 {
            return Err(BrepError::Degenerate(format!(
                "a transition's bend radius of {} mm eats facet {} of {}: make the tolerance finer or the radius smaller",
                dims.radius,
                fi + 1,
                facets.len()
            )));
        }
    }
    Ok(Layout { vertices, a: a3, b: b3, fans, facets, bends, top_rim, bottom_rim, seam, az, reliefs })
}

/// A planar face through `points`, its loop `ids` turned to face `reference`.
#[cfg(test)]
fn planar(id: TopoId, mut ids: Vec<TopoId>, points: &[Vec3], reference: Vec3) -> Result<FaceSpec, BrepError> {
    let n = (points[1] - points[0]).cross(points[2] - points[0]);
    if n.dot(n) == 0.0 {
        return Err(BrepError::Degenerate(format!("transition: face {id:?} has no area")));
    }
    let n = if n.dot(reference) < 0.0 {
        ids.reverse();
        -n
    } else {
        n
    };
    Ok(FaceSpec { id, surface: Surface::Plane(Plane { origin: points[0], normal: unit3(n) }), loops: vec![ids] })
}


#[cfg(test)]
fn planar_polygon(
    id: TopoId,
    mut ids: Vec<TopoId>,
    points: &[Vec3],
    reference: Vec3,
) -> Result<FaceSpec, BrepError> {
    if ids.len() != points.len() || ids.len() < 3 {
        return Err(BrepError::Degenerate(format!("transition: face {id:?} has an invalid polygon loop")));
    }
    let normal = (0..points.len()).fold(Vec3::ZERO, |sum, i| sum + points[i].cross(points[(i + 1) % points.len()]));
    if !(normal.dot(normal) > 0.0 && normal.dot(normal).is_finite()) {
        return Err(BrepError::Degenerate(format!("transition: face {id:?} has no area")));
    }
    let normal = if normal.dot(reference) < 0.0 {
        ids.reverse();
        normal * -1.0
    } else {
        normal
    };
    Ok(FaceSpec { id, surface: Surface::Plane(Plane { origin: points[0], normal: unit3(normal) }), loops: vec![ids] })
}

#[cfg(test)]
fn facet_patch_prism(feature: TopoId, patch: &FacetPatch, points: &BTreeMap<TopoId, Vec3>, normal: Vec3) -> Result<Body, BrepError> {
    let outer_ids: Vec<TopoId> = patch.boundary.iter().map(|v| v.outer_id).collect();
    let outer_points: Vec<Vec3> = patch.boundary.iter().map(|v| v.outer).collect();
    let inner_ids: Vec<TopoId> = patch.boundary.iter().rev().map(|v| v.inner_id).collect();
    let inner_points: Vec<Vec3> = patch.boundary.iter().rev().map(|v| v.inner).collect();
    let mut vertices: Vec<Vertex> = points.iter().map(|(&id, &point)| Vertex { id, point }).collect();
    let mut faces = vec![
        planar_polygon(TopoId::derive("transition.patch.test.outer").id(feature).index(patch.facet as u64).finish(), outer_ids, &outer_points, normal)?,
        planar_polygon(TopoId::derive("transition.patch.test.inner").id(feature).index(patch.facet as u64).finish(), inner_ids, &inner_points, normal * -1.0)?,
    ];
    let center = (outer_points.iter().chain(&inner_points).fold(Vec3::ZERO, |sum, p| sum + *p))
        * (1.0 / (outer_points.len() + inner_points.len()) as f64);
    for i in 0..patch.boundary.len() {
        let j = (i + 1) % patch.boundary.len();
        let a = patch.boundary[i];
        let b = patch.boundary[j];
        let ids = vec![a.outer_id, b.outer_id, b.inner_id, a.inner_id];
        let wall_points = vec![a.outer, b.outer, b.inner, a.inner];
        let wall_center = wall_points.iter().fold(Vec3::ZERO, |sum, p| sum + *p) * 0.25;
        faces.push(planar(
            TopoId::derive("transition.patch.test.wall").id(feature).index(patch.facet as u64).index(i as u64).finish(),
            ids,
            &wall_points,
            wall_center - center,
        )?);
    }
    vertices.shrink_to_fit();
    build_with_curves(vertices, faces, Vec::new())
}

/// The sharp faceted shell: the outer facets, the inner one a thickness off, and the rims (the
/// first stage of the body, D264; the bends and the corner reliefs go on top of it).
#[cfg(test)]
fn shell(feature: TopoId, layout: &Layout, dims: &TransitionDims) -> Result<Body, BrepError> {
    let n = layout.a.len();
    let ids = TransitionIds { feature };
    let inner_id = |v: TopoId| ids.inner(v);
    let fan_vid = |j: usize, k: usize, m: usize| ids.fan_vertex(j, k, m, n);
    // The inner vertex is the outer one a thickness along the facets' normals there.
    let mut incident: BTreeMap<TopoId, Vec<usize>> = BTreeMap::new();
    for (fi, f) in layout.facets.iter().enumerate() {
        for &v in &f.loop_ {
            incident.entry(v).or_default().push(fi);
        }
    }
    let mut vertices: Vec<Vertex> = layout.vertices.iter().map(|(&id, &point)| Vertex { id, point }).collect();
    let mut point_of: BTreeMap<TopoId, Vec3> = layout.vertices.clone();
    for (&v, &p) in &layout.vertices {
        let mut s = Vec3::ZERO;
        for &fi in incident.get(&v).map(Vec::as_slice).unwrap_or(&[]) {
            s = s + layout.facets[fi].normal;
        }
        let q = if s.dot(s) > 0.0 { p - unit3(s) * dims.thickness } else { p };
        point_of.insert(inner_id(v), q);
        vertices.push(Vertex { id: inner_id(v), point: q });
    }
    let get = |v: TopoId| -> Result<Vec3, BrepError> {
        point_of.get(&v).copied().ok_or_else(|| BrepError::Invalid(format!("transition: no point for vertex {v:?}")))
    };
    let centroid = |ids: &[TopoId]| -> Result<Vec3, BrepError> {
        let mut s = Vec3::ZERO;
        for &v in ids {
            s = s + get(v)?;
        }
        Ok(s * (1.0 / ids.len() as f64))
    };
    let mut faces: Vec<FaceSpec> = Vec::new();
    for (fi, f) in layout.facets.iter().enumerate() {
        faces.push(FaceSpec {
            id: TopoId::derive("transition.outer").id(feature).index(fi as u64).finish(),
            surface: Surface::Plane(Plane { origin: f.origin, normal: f.normal }),
            loops: vec![f.loop_.clone()],
        });
        let loop_: Vec<TopoId> = f.loop_.iter().rev().map(|&v| inner_id(v)).collect();
        let origin = get(inner_id(f.loop_[0]))?;
        faces.push(FaceSpec {
            id: TopoId::derive("transition.inside").id(feature).index(fi as u64).finish(),
            surface: Surface::Plane(Plane { origin, normal: -f.normal }),
            loops: vec![loop_],
        });
    }
    // The bottom rims, one per piece of `a`; the top rims, one per chord.
    for (i, f) in layout.facets.iter().enumerate().take(n) {
        let (u, w) = (ids.a(i), ids.a((i + 1) % n));
        let ids = vec![u, w, inner_id(w), inner_id(u)];
        let pts = ids.iter().map(|&v| get(v)).collect::<Result<Vec<Vec3>, _>>()?;
        let reference = average(&pts) - centroid(&f.loop_)?;
        faces.push(planar(TopoId::derive("transition.rim").id(feature).index(i as u64).finish(), ids, &pts, reference)?);
    }
    let mut fan_at = n;
    for j in 0..n {
        let m = layout.fans[j];
        for k in 0..m {
            let (u, w) = (fan_vid(j, k, m), fan_vid(j, k + 1, m));
            let ids = vec![u, w, inner_id(w), inner_id(u)];
            let pts = ids.iter().map(|&v| get(v)).collect::<Result<Vec<Vec3>, _>>()?;
            let reference = average(&pts) - centroid(&layout.facets[fan_at + k].loop_)?;
            faces.push(planar(TopoId::derive("transition.top").id(feature).index((fan_at + k) as u64).finish(), ids, &pts, reference)?);
        }
        fan_at += m;
    }
    build_with_curves(vertices, faces, Vec::new())
}

/// The transition between the profiles: refused at the fan's corners until their relief is settled.
pub(crate) fn transition(
    feature: TopoId,
    a: &Placed,
    a_name: &str,
    b: &Placed,
    b_name: &str,
    dims: TransitionDims,
) -> Result<datum_brep::Body, BrepError> {
    if !(dims.thickness > 0.0 && dims.thickness.is_finite()) {
        return Err(BrepError::Degenerate(format!("a transition needs a positive thickness, got {}", dims.thickness)));
    }
    if !(dims.radius > 0.0 && dims.radius.is_finite()) {
        return Err(BrepError::Degenerate(format!("a transition needs a positive bend radius, got {}", dims.radius)));
    }
    if !(0.0..=1.0).contains(&dims.k_factor) {
        return Err(BrepError::Degenerate(format!("a K-factor lies between 0 and 1, got {}", dims.k_factor)));
    }
    if !(dims.tolerance > 0.0 && dims.tolerance.is_finite()) {
        return Err(BrepError::Degenerate(format!("a transition needs a positive tolerance, got {}", dims.tolerance)));
    }
    if !(dims.gap > 0.0 && dims.gap.is_finite()) {
        return Err(BrepError::Degenerate(format!("a transition needs a gap at its seam, or it closes and cannot be unrolled: got {}", dims.gap)));
    }
    let layout = layout(feature, a, a_name, b, b_name, &dims)?;
    verify(&layout, &dims)?;
    // The faceted shell (facets, the inner offset, the rims) validates; the bends and the corner
    // relief that make it the D264 body are being built (the layout's bends and reliefs are
    // verified above). Until then a body is refused rather than a sharp one blessed.
    let rims: f64 = layout.top_rim.iter().chain(&layout.bottom_rim).sum();
    Err(BrepError::Unsupported(format!(
        "transition: the rounded body (the bends and the corner relief) is not built yet: {} facets, {} bends ({:.3} rad), {} relief sectors, rims {:.3} mm, the seam {:.3} mm",
        layout.facets.len(),
        layout.bends.len(),
        layout.bends.iter().map(|b| b.sweep).sum::<f64>(),
        layout.reliefs.len(),
        rims,
        (layout.seam.to - layout.seam.from).dot(layout.seam.to - layout.seam.from).sqrt()
    )))
}

fn facet_trims(layout: &Layout, lower: &[BendSection], dims: &TransitionDims) -> Result<Vec<FacetTrim>, BrepError> {
    if lower.len() != layout.bends.len() {
        return Err(BrepError::Invalid("transition: facet trims have incomplete lower bend sections".into()));
    }
    let (outer_offset, inner_offset) = shifts(dims.side, dims.thickness);
    let size = layout
        .vertices
        .values()
        .flat_map(|p| [p.x, p.y, p.z])
        .chain(layout.a.iter().chain(&layout.b).flat_map(|p| [p.x, p.y, p.z]))
        .fold(1.0_f64, |m, x| m.max(x.abs()));
    let envelope = 64.0 * f64::EPSILON * size;
    let mut trims = Vec::with_capacity(layout.facets.len());

    for (fi, facet) in layout.facets.iter().enumerate() {
        if facet.loop_.len() != 3 {
            return Err(BrepError::Invalid(format!("transition: facet {fi} is not triangular")));
        }
        let shared = layout.bends.iter().find_map(|bend| {
            layout.bends.iter().find(|other| other.edge != bend.edge).and_then(|other| {
                bend.edge.iter().find(|v| other.edge.contains(v)).copied().filter(|v| facet.loop_.contains(v))
            })
        }).ok_or_else(|| BrepError::Invalid(format!("transition: facet {fi} has no shared bend vertex")))?;
        let shared_at = facet.loop_.iter().position(|&v| v == shared).unwrap();
        let next = facet.loop_[(shared_at + 1) % 3];
        let prev = facet.loop_[(shared_at + 2) % 3];
        let bend_for_edge = |a: TopoId, b: TopoId| {
            layout.bends.iter().enumerate().find_map(|(bi, bend)| {
                (bend.edge.contains(&a) && bend.edge.contains(&b))
                    .then(|| (bi, bend.facets.iter().position(|&f| f == fi)))
                    .and_then(|(bi, side)| side.map(|side| (bi, side)))
            })
        };
        let (b0i, b0s) = bend_for_edge(shared, next)
            .ok_or_else(|| BrepError::Invalid(format!("transition: facet {fi}'s outgoing edge has no bend")))?;
        let (b1i, b1s) = bend_for_edge(prev, shared)
            .ok_or_else(|| BrepError::Invalid(format!("transition: facet {fi}'s incoming edge has no bend")))?;
        let (b0, b1) = (&layout.bends[b0i], &layout.bends[b1i]);
        let opposite = vec![next, prev];

        let edge_a = *layout.vertices.get(&opposite[0]).ok_or_else(|| BrepError::Invalid("transition: facet rim vertex is missing".into()))?;
        let edge_b = *layout.vertices.get(&opposite[1]).ok_or_else(|| BrepError::Invalid("transition: facet rim vertex is missing".into()))?;
        let edge = edge_b - edge_a;
        let q = meet(b0.tangent[b0s], b0.axis, b1.tangent[b1s], b1.axis)?;
        if !within_rounding((q - facet.origin).dot(facet.normal), size) {
            return Err(BrepError::Invalid(format!("transition: facet {fi}'s tangent lines do not meet on its plane")));
        }

        let mut x = [Vec3::ZERO; 2];
        let mut lower_side = [FacetSection { outer: Vec3::ZERO, inner: Vec3::ZERO }; 2];
        let mut upper_side = [FacetSection { outer: Vec3::ZERO, inner: Vec3::ZERO }; 2];
        for (slot, (bend_index, bend_side)) in [(b0i, b0s), (b1i, b1s)].into_iter().enumerate() {
            let bend = &layout.bends[bend_index];
            let hit = meet(bend.tangent[bend_side], bend.axis, edge_a, edge)?;
            let u = (hit - edge_a).dot(edge) / edge.dot(edge);
            if u < -envelope || u > 1.0 + envelope {
                return Err(BrepError::Invalid(format!("transition: bend {bend_index}'s tangent misses facet {fi}'s finite rim edge")));
            }
            let projected = edge_a + edge * u;
            if !within_rounding(distance3(hit, projected), size) || !within_rounding((hit - facet.origin).dot(facet.normal), size) {
                return Err(BrepError::Invalid(format!("transition: bend {bend_index}'s rim split is off facet {fi}")));
            }
            x[slot] = hit;

            let lower_section = lower[bend_index];
            let top_station = (bend.top - bend.at).dot(bend.axis);
            let top_section = bend_section_at_station(bend, top_station, outer_offset, inner_offset, size)?;
            let q_station = (q - bend.at).dot(bend.axis);
            let x_station = (hit - bend.at).dot(bend.axis);
            // D264 relief end planes are assigned per rib from the q point on the following
            // fan facet. On the preceding facet, the same lower section can therefore extend
            // slightly past this facet's q; lower_sections already checks it against the finite
            // rib. Only the D265 upper section must remain between this facet's q and rim split.
            if !station_between(top_station, q_station, x_station, size) {
                return Err(BrepError::Invalid(format!(
                    "transition: bend {bend_index}, facet {fi}, tangent side {bend_side}: upper station {top_station:.9} falls outside q={q_station:.9} .. x={x_station:.9}"
                )));
            }
            if (top_station - lower_section.station).abs() <= envelope {
                return Err(BrepError::Degenerate(format!("transition: bend {bend_index} has no band length on facet {fi}")));
            }
            lower_side[slot] = FacetSection { outer: lower_section.outer[bend_side], inner: lower_section.inner[bend_side] };
            upper_side[slot] = FacetSection { outer: top_section.outer[bend_side], inner: top_section.inner[bend_side] };
        }

        // Fan facets must trim against exactly their two stored top chord endpoints.
        if facet.top.len() == 2 {
            let mut rim_ids = opposite.clone();
            let mut top_ids = facet.top.clone();
            rim_ids.sort_unstable();
            top_ids.sort_unstable();
            if rim_ids != top_ids {
                return Err(BrepError::Invalid(format!("transition: facet {fi}'s opposite edge is not its stored top chord")));
            }
        }
        trims.push(FacetTrim { facet: fi, bends: [b0i, b1i], q, x, lower: lower_side, upper: upper_side });
    }
    Ok(trims)
}


fn patch_band_id(feature: TopoId, bend: usize, side: usize, upper: bool, inner: bool) -> TopoId {
    TopoId::derive("transition.patch.band")
        .id(feature)
        .index(bend as u64)
        .index(side as u64)
        .index(u64::from(upper))
        .index(u64::from(inner))
        .finish()
}

fn patch_rim_id(feature: TopoId, facet: usize, slot: usize, inner: bool) -> TopoId {
    TopoId::derive("transition.patch.rim")
        .id(feature)
        .index(facet as u64)
        .index(slot as u64)
        .index(u64::from(inner))
        .finish()
}

fn register_patch_point(
    registry: &mut BTreeMap<TopoId, Vec3>,
    id: TopoId,
    point: Vec3,
    size: f64,
) -> Result<(), BrepError> {
    if let Some(old) = registry.get(&id) {
        if !within_rounding(distance3(*old, point), size) {
            return Err(BrepError::Invalid(format!("transition: patch vertex {id:?} has inconsistent positions")));
        }
    } else {
        registry.insert(id, point);
    }
    Ok(())
}

/// Gives each trimmed patch a shared topological identity for its two bend sides and its two
/// rim splits. Adjacent facets therefore name the same bend-side vertices; no coordinate welding
/// is left to the B-rep builder.
fn facet_patches(
    feature: TopoId,
    layout: &Layout,
    trims: &[FacetTrim],
    dims: &TransitionDims,
) -> Result<(Vec<FacetPatch>, BTreeMap<TopoId, Vec3>), BrepError> {
    if trims.len() != layout.facets.len() {
        return Err(BrepError::Invalid("transition: facet patches have incomplete trim records".into()));
    }
    let (outer_offset, inner_offset) = shifts(dims.side, dims.thickness);
    let size = layout
        .vertices
        .values()
        .flat_map(|p| [p.x, p.y, p.z])
        .chain(layout.a.iter().chain(&layout.b).flat_map(|p| [p.x, p.y, p.z]))
        .fold(1.0_f64, |m, x| m.max(x.abs()));
    let mut registry = BTreeMap::new();
    let mut patches = Vec::with_capacity(trims.len());

    for (fi, trim) in trims.iter().enumerate() {
        if trim.facet != fi {
            return Err(BrepError::Invalid(format!("transition: patch {fi} has trim data for facet {}", trim.facet)));
        }
        let facet = &layout.facets[fi];
        let mut boundary = Vec::with_capacity(6);
        for slot in 0..2 {
            let bend_index = trim.bends[slot];
            let bend = layout.bends.get(bend_index).ok_or_else(|| BrepError::Invalid(format!("transition: patch {fi} names missing bend {bend_index}")))?;
            let side = bend.facets.iter().position(|&f| f == fi).ok_or_else(|| {
                BrepError::Invalid(format!("transition: bend {bend_index} is not incident to patch {fi}"))
            })?;
            for (upper, section) in [(false, trim.lower[slot]), (true, trim.upper[slot])] {
                let vertex = PatchVertex {
                    outer_id: patch_band_id(feature, bend_index, side, upper, false),
                    inner_id: patch_band_id(feature, bend_index, side, upper, true),
                    outer: section.outer,
                    inner: section.inner,
                };
                register_patch_point(&mut registry, vertex.outer_id, vertex.outer, size)?;
                register_patch_point(&mut registry, vertex.inner_id, vertex.inner, size)?;
                boundary.push(vertex);
            }
            let x = trim.x[slot];
            let vertex = PatchVertex {
                outer_id: patch_rim_id(feature, fi, slot, false),
                inner_id: patch_rim_id(feature, fi, slot, true),
                outer: x + facet.normal * outer_offset,
                inner: x + facet.normal * inner_offset,
            };
            register_patch_point(&mut registry, vertex.outer_id, vertex.outer, size)?;
            register_patch_point(&mut registry, vertex.inner_id, vertex.inner, size)?;
            boundary.push(vertex);
        }
        // The second side was inserted as lower/upper/x; traverse the rim toward it, then back
        // along its bend edge. The first three entries are outgoing; rotate into boundary order.
        let ordered = vec![boundary[0], boundary[1], boundary[2], boundary[5], boundary[4], boundary[3]];
        for (i, vertex) in ordered.iter().enumerate() {
            let outer_plane = (vertex.outer - facet.origin).dot(facet.normal);
            let inner_plane = (vertex.inner - facet.origin).dot(facet.normal);
            if !within_rounding(outer_plane - outer_offset, size) || !within_rounding(inner_plane - inner_offset, size) {
                return Err(BrepError::Invalid(format!("transition: patch {fi}'s vertex {i} misses its offset plane")));
            }
        }
        let normal = (0..ordered.len()).fold(Vec3::ZERO, |sum, i| {
            sum + ordered[i].outer.cross(ordered[(i + 1) % ordered.len()].outer)
        });
        if !(normal.dot(facet.normal) > 0.0 && normal.dot(normal).is_finite()) {
            return Err(BrepError::Degenerate(format!("transition: trimmed facet {fi} has no outward planar area")));
        }
        // Distinct boundary positions must remain distinct after trimming.
        for i in 0..ordered.len() {
            let next = (i + 1) % ordered.len();
            if within_rounding(distance3(ordered[i].outer, ordered[next].outer), size)
                || within_rounding(distance3(ordered[i].inner, ordered[next].inner), size)
            {
                return Err(BrepError::Degenerate(format!("transition: trimmed facet {fi} has a collapsed boundary edge {i}")));
            }
        }
        patches.push(FacetPatch { facet: fi, bends: trim.bends, boundary: ordered });
    }
    Ok((patches, registry))
}

fn verify_facet_trim_records(layout: &Layout, trims: &[FacetTrim], dims: &TransitionDims) -> Result<(), BrepError> {
    if trims.len() != layout.facets.len() {
        return Err(BrepError::Invalid("transition: not every facet has one trimmed patch".into()));
    }
    let (outer_offset, inner_offset) = shifts(dims.side, dims.thickness);
    let size = layout
        .vertices
        .values()
        .flat_map(|p| [p.x, p.y, p.z])
        .chain(layout.a.iter().chain(&layout.b).flat_map(|p| [p.x, p.y, p.z]))
        .fold(1.0_f64, |m, x| m.max(x.abs()));

    for (fi, trim) in trims.iter().enumerate() {
        if trim.facet != fi {
            return Err(BrepError::Invalid(format!("transition: trimmed patch {fi} names facet {}", trim.facet)));
        }
        let facet = &layout.facets[fi];
        if !within_rounding((trim.q - facet.origin).dot(facet.normal), size) {
            return Err(BrepError::Invalid(format!("transition: trimmed facet {fi}'s q is off its neutral plane")));
        }
        for slot in 0..2 {
            let bend_index = trim.bends[slot];
            let bend = layout
                .bends
                .get(bend_index)
                .ok_or_else(|| BrepError::Invalid(format!("transition: trimmed facet {fi} names missing bend {bend_index}")))?;
            let side = bend
                .facets
                .iter()
                .position(|&f| f == fi)
                .ok_or_else(|| BrepError::Invalid(format!("transition: bend {bend_index} is not incident to trimmed facet {fi}")))?;
            for point in [trim.q, trim.x[slot]] {
                let along = bend.axis * (point - bend.tangent[side]).dot(bend.axis);
                let miss = point - bend.tangent[side] - along;
                if !within_rounding(distance3(miss, Vec3::ZERO), size) {
                    return Err(BrepError::Invalid(format!("transition: trimmed facet {fi}'s tangent boundary is off bend {bend_index}")));
                }
            }
            if !within_rounding((trim.x[slot] - facet.origin).dot(facet.normal), size) {
                return Err(BrepError::Invalid(format!("transition: trimmed facet {fi}'s rim split is off its neutral plane")));
            }
            for section in [trim.lower[slot], trim.upper[slot]] {
                if !within_rounding((section.outer - facet.origin).dot(facet.normal) - outer_offset, size)
                    || !within_rounding((section.inner - facet.origin).dot(facet.normal) - inner_offset, size)
                {
                    return Err(BrepError::Invalid(format!("transition: trimmed facet {fi}'s band endpoints miss its offset planes")));
                }
                if !within_rounding(distance3(section.outer, section.inner) - dims.thickness, size) {
                    return Err(BrepError::Invalid(format!("transition: trimmed facet {fi}'s band endpoints miss the sheet thickness")));
                }
                let station = (section.outer - bend.at).dot(bend.axis);
                let center = bend.at + bend.axis * station;
                let radial = unit3(bend.tangent[side] - bend.at);
                let want_outer = center + radial * bend.radius_out;
                let want_inner = center + radial * bend.radius_in;
                if !within_rounding(distance3(section.outer, want_outer), size) || !within_rounding(distance3(section.inner, want_inner), size) {
                    return Err(BrepError::Invalid(format!("transition: trimmed facet {fi}'s band endpoints miss bend {bend_index}'s cylinders")));
                }
            }
        }
    }
    Ok(())
}

/// Checks the разметка against itself (D264: a wrong layout is turned away here, before it is
/// developed): every facet's vertices lie on its plane, every bend's axis along its edge and at the
/// outer radius from both facets, its tangents the axis's feet on the facets' own planes, each
/// tangent line at the tangent offset `(radius_out - outer)·tan(sweep/2)` from its edge, its sweep
/// the angle between the facets' normals, and each relief's point on its facet's two tangent lines.
fn verify(layout: &Layout, dims: &TransitionDims) -> Result<(), BrepError> {
    let size = layout
        .vertices
        .values()
        .flat_map(|p| [p.x, p.y, p.z])
        .chain(layout.a.iter().chain(&layout.b).flat_map(|p| [p.x, p.y, p.z]))
        .fold(1.0_f64, |m, x| m.max(x.abs()));
    for f in &layout.facets {
        for &v in &f.loop_ {
            let p = *layout.vertices.get(&v).ok_or_else(|| BrepError::Invalid(format!("transition: facet vertex {v:?} is missing")))?;
            if !within_rounding((p - f.origin).dot(f.normal), size) {
                return Err(BrepError::Invalid(format!("transition: a facet's vertex {v:?} stands off its plane")));
            }
        }
    }
    let (outer, _) = shifts(dims.side, dims.thickness);
    let radius_out = dims.radius + dims.thickness;
    for b in &layout.bends {
        let (u, v) = (b.edge[0], b.edge[1]);
        let p1 = *layout.vertices.get(&u).ok_or_else(|| BrepError::Invalid(format!("transition: bend vertex {u:?} is missing")))?;
        let p2 = *layout.vertices.get(&v).ok_or_else(|| BrepError::Invalid(format!("transition: bend vertex {v:?} is missing")))?;
        let e = unit3(p2 - p1);
        if !nearly_parallel(e, b.axis) {
            return Err(BrepError::Invalid("transition: a bend's axis is not along its edge".into()));
        }
        for (fi, t) in b.facets.iter().zip(b.tangent) {
            let f = &layout.facets[*fi];
            let dist = (b.at - f.origin).dot(f.normal) - outer;
            if !within_rounding(dist + b.radius_out, size) {
                return Err(BrepError::Invalid("transition: a bend's axis is not at the outer radius from a facet".into()));
            }
            // The tangent point is the axis's foot on the facet's own plane, that far off the
            // shared edge as (radius_out - outer)·tan(sweep/2).
            let trim = b.radius_out - outer;
            let w = t - b.at;
            if !within_rounding(w.dot(w).sqrt() - trim, size) || !within_rounding(w.dot(f.normal) - trim, size) {
                return Err(BrepError::Invalid("transition: a bend's tangent point is off its facet".into()));
            }
            let u = t - p1;
            let off = (u - e * u.dot(e)).dot(u - e * u.dot(e)).sqrt();
            if !within_rounding(off - trim * libm::tan(b.sweep / 2.0), size) {
                return Err(BrepError::Invalid("transition: a bend's tangent line is not at the tangent offset from its edge".into()));
            }
        }
        if !within_rounding(b.radius_in + dims.thickness - b.radius_out, size) {
            return Err(BrepError::Invalid("transition: a bend's radii differ by the thickness".into()));
        }
        let [fi, gi] = b.facets;
        let angle = libm::acos(layout.facets[fi].normal.dot(layout.facets[gi].normal).clamp(-1.0, 1.0));
        if !within_rounding(b.sweep - angle, 1.0) {
            return Err(BrepError::Invalid("transition: a bend's sweep is not the angle between its facets".into()));
        }
    }
    // Every band stops short of the top rim (D264, option b): its end plane square to the axis,
    // through the meeting of its triangle's two tangent lines at an extreme bend, at the axis
    // point whose end face's top touches the top profile's plane at an inner one (D265) — and the
    // plane stands before the rib's top end.
    let n = layout.a.len();
    let o_t = layout.b[0];
    let az = layout.az;
    for b in &layout.bends {
        let [fi, gi] = b.facets;
        let want = if fi < n || gi < n {
            let ti = fi.min(gi);
            let mut lines: Vec<(Vec3, Vec3)> = Vec::new();
            for b2 in &layout.bends {
                if let Some(side) = b2.facets.iter().position(|&f| f == ti) {
                    lines.push((b2.tangent[side], b2.axis));
                }
            }
            if lines.len() != 2 {
                return Err(BrepError::Invalid("transition: a flat triangle has not two bends".into()));
            }
            meet(lines[0].0, lines[0].1, lines[1].0, lines[1].1)?
        } else {
            let (m, rho) = span_top(b, b.radius_in, b.radius_out, az);
            let axis_az = b.axis.dot(az);
            if within_rounding(axis_az, 1.0) {
                return Err(BrepError::Degenerate("a transition's rib runs parallel to the top profile's plane".into()));
            }
            let s = (-(b.at - o_t).dot(az) - rho * m) / axis_az;
            b.at + b.axis * s
        };
        if !within_rounding((b.top - want).dot(b.top - want).sqrt(), size) {
            return Err(BrepError::Invalid("transition: a bend's top end is off its point".into()));
        }
        // Nothing of the blank's edge rises off the top profile's plane (D265).
        let (m, rho) = span_top(b, b.radius_in, b.radius_out, az);
        let over = (b.top - o_t).dot(az) + rho * m;
        if over > 0.0 && !within_rounding(over, size) {
            return Err(BrepError::Invalid("transition: a bend's end face rises over the top profile's plane".into()));
        }
        let top_ids: BTreeSet<TopoId> = layout.facets[fi].top.iter().chain(&layout.facets[gi].top).copied().collect();
        let far = [b.edge[0], b.edge[1]]
            .iter()
            .find(|v| top_ids.contains(v))
            .copied()
            .ok_or_else(|| BrepError::Invalid("transition: a bend's edge has no top end".into()))?;
        let far = *layout.vertices.get(&far).ok_or_else(|| BrepError::Invalid("transition: rim vertex is missing".into()))?;
        let s_top = (b.top - b.at).dot(b.axis);
        let s_rim = (far - b.at).dot(b.axis);
        if !(s_top * s_rim > 0.0 && s_top.abs() < s_rim.abs()) {
            return Err(BrepError::Invalid("transition: a bend's band does not stop short of the top rim".into()));
        }
    }
    let seam = &layout.seam;
    if seam.facet >= layout.facets.len() || !(seam.gap > 0.0 && seam.gap.is_finite()) {
        return Err(BrepError::Invalid("transition: the seam names no facet, or has no gap".into()));
    }
    if layout.reliefs.len() != layout.a.len() {
        return Err(BrepError::Invalid("transition: not one relief at every vertex of the first profile".into()));
    }
    let mut fan_base = layout.a.len();
    for (j, r) in layout.reliefs.iter().enumerate() {
        let m = layout.fans[j];
        if r.q.len() != m || r.ribs.len() != m + 1 || r.ends.len() != m + 1 {
            return Err(BrepError::Invalid(format!(
                "transition: the relief at vertex {j} has {} points, {} ribs for {} fan facets",
                r.q.len(),
                r.ribs.len(),
                m
            )));
        }
        // The bend of each rib lies on the rib: its axis along it, from the vertex out to the fan.
        let rib = |k: usize| -> Result<(Vec3, Vec3), BrepError> {
            let b = &layout.bends[r.ribs[k]];
            let p1 = *layout.vertices.get(&b.edge[0]).ok_or_else(|| BrepError::Invalid("transition: bend vertex is missing".into()))?;
            let p2 = *layout.vertices.get(&b.edge[1]).ok_or_else(|| BrepError::Invalid("transition: bend vertex is missing".into()))?;
            let far = if (p1 - r.at).dot(p1 - r.at) <= (p2 - r.at).dot(p2 - r.at) { p2 } else { p1 };
            Ok((b.at, unit3(far - r.at)))
        };
        // Each point lies on both tangent lines of its fan facet — the bend at each of its ribs,
        // through the axis's foot on the facet's own plane, along the rib — and stays inside the
        // facet, between its three edges.
        let trim = radius_out - outer;
        for k in 0..m {
            let f = &layout.facets[fan_base + k];
            let line = |rib_i: usize| -> Result<(Vec3, Vec3), BrepError> {
                let (at, dir) = rib(rib_i)?;
                Ok((at + f.normal * trim, dir))
            };
            let (t0, d0) = line(k)?;
            let (t1, d1) = line(k + 1)?;
            for (t, d) in [(t0, d0), (t1, d1)] {
                let w = r.q[k] - t;
                let miss = (w - d * w.dot(d)).dot(w - d * w.dot(d)).sqrt();
                if !within_rounding(miss, size) {
                    return Err(BrepError::Invalid(format!("transition: the relief's point {k} at vertex {j} is off a tangent line")));
                }
            }
            let tri: Vec<Vec3> = f
                .loop_
                .iter()
                .map(|v| layout.vertices.get(v).copied().ok_or_else(|| BrepError::Invalid("transition: facet vertex is missing".into())))
                .collect::<Result<_, _>>()?;
            for i in 0..3 {
                let (a, b, c) = (tri[i], tri[(i + 1) % 3], tri[(i + 2) % 3]);
                let e = unit3(b - a);
                let side_p = e.cross(r.q[k] - a).dot(f.normal);
                let side_c = e.cross(c - a).dot(f.normal);
                if side_p * side_c < 0.0 && !within_rounding(side_p, size) && !within_rounding(side_c, size) {
                    return Err(BrepError::Invalid(format!("transition: the relief's point {k} at vertex {j} leaves its fan facet")));
                }
            }
        }
        fan_base += m;
        // Each end plane stands square to its bend: its axis along the rib, through the point the
        // bend's tangent line meets its neighbour's.
        for (k, (axis, point)) in r.ends.iter().enumerate() {
            let (_, rib_dir) = rib(k)?;
            if !nearly_parallel(*axis, rib_dir) {
                return Err(BrepError::Invalid(format!("transition: the relief's end plane {k} at vertex {j} is off its bend")));
            }
            let want = if k < m { r.q[k] } else { r.q[m - 1] };
            if !within_rounding(axis.dot(*axis) - 1.0, 1.0) || !within_rounding((*point - want).dot(*point - want).sqrt(), size) {
                return Err(BrepError::Invalid(format!("transition: the relief's end plane {k} at vertex {j} is off")));
            }
        }
        // The two side planes hold the vertex and their extreme bends' axes; the two extreme ones
        // are square to the shell.
        for (side, k) in [(r.sides[0], 0), (r.sides[1], m)] {
            let (point, normal) = side;
            if !within_rounding(normal.dot(normal) - 1.0, 1.0) || !within_rounding(normal.dot(r.ends[k].0), 1.0) {
                return Err(BrepError::Invalid(format!("transition: the relief's side plane at vertex {j} is off")));
            }
            if !within_rounding((point - r.at).dot(normal), size) {
                return Err(BrepError::Invalid(format!("transition: the relief's side plane misses its vertex {j}")));
            }
            let (at, _) = rib(k)?;
            if !within_rounding((at - r.at).dot(normal), size) {
                return Err(BrepError::Invalid(format!("transition: the relief's side plane misses its bend at vertex {j}")));
            }
        }
    }
    // Resolve one perpendicular lower station for every bend. The relief q point seeds one
    // tangent side only; the opposite side is intersected with the same plane.
    let sections = lower_sections(layout, dims)?;
    if sections.len() != layout.bends.len() {
        return Err(BrepError::Invalid("transition: not every bend has one lower section".into()));
    }
    let trims = facet_trims(layout, &sections, dims)?;
    verify_facet_trim_records(layout, &trims, dims)?;
    let (patches, registry) = facet_patches(TopoId::derive("transition.verify").finish(), layout, &trims, dims)?;
    if patches.len() != layout.facets.len() || registry.is_empty() {
        return Err(BrepError::Invalid("transition: trimmed facet patches are incomplete".into()));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Op, Side, SketchPlane, Value};
    use serde_json::json;

    fn id(tag: &str) -> TopoId {
        TopoId::derive(tag).finish()
    }

    fn placed(sketch: serde_json::Value, plane: Option<SketchPlane>) -> Placed {
        let op: Op = serde_json::from_value(json!({"op": "sketch", "sketch": sketch, "plane": plane})).unwrap();
        match op.eval(id("profile"), &[]).unwrap() {
            Value::Sketch(s) => *s,
            _ => panic!("a sketch"),
        }
    }

    /// The эталон's first profile: an 80 × 80 square about the origin on XY.
    fn square() -> Placed {
        placed(
            json!({
                "points": {"a": [-40, -40], "b": [40, -40], "c": [40, 40], "d": [-40, 40]},
                "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"},
                          "cd": {"from": "c", "to": "d"}, "da": {"from": "d", "to": "a"}}
            }),
            None,
        )
    }

    /// The эталон's second profile: a circle of radius 90 about the origin, 120 above.
    fn circle() -> Placed {
        placed(
            json!({"points": {"o": [0, 0]}, "circles": {"c": {"center": "o", "diameter": 180}}}),
            Some(SketchPlane { origin: Vec3::new(0.0, 0.0, 120.0), x_axis: Vec3::new(1.0, 0.0, 0.0), normal: Vec3::new(0.0, 0.0, 1.0) }),
        )
    }

    fn dims() -> TransitionDims {
        TransitionDims { thickness: 2.0, radius: 8.0, k_factor: 0.44, side: Side::Left, gap: 2.0, tolerance: 1.0 }
    }

    #[test]
    fn the_layout_of_a_square_to_a_circle() {
        let layout = layout(id("t"), &square(), "ab", &circle(), "c", &dims()).unwrap();
        verify(&layout, &dims()).unwrap();
        // 4 flat triangles and 6 facets per fan: 28 facets, 2 boundary bends per triangle and 5
        // inside each fan: 28 bends (D264).
        assert_eq!(layout.facets.len(), 28);
        assert_eq!(layout.bends.len(), 28);
        assert_eq!(layout.fans, vec![6, 6, 6, 6]);
        // The apex points over the middles of the sides: the circle's four quarter points.
        let mut apex: Vec<[f64; 3]> = layout.b.iter().map(|p| [p.x, p.y, p.z]).collect();
        apex.sort_by(|a, b| a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));
        assert_eq!(apex, vec![[-90.0, 0.0, 120.0], [0.0, -90.0, 120.0], [0.0, 90.0, 120.0], [90.0, 0.0, 120.0]]);
        // Every straight rim is the side; the round rim is 24 chords of the circle.
        assert_eq!(layout.bottom_rim.len(), 4);
        assert!(layout.bottom_rim.iter().all(|l| (l - 80.0).abs() < 1e-9));
        assert_eq!(layout.top_rim.len(), 24);
        let round: f64 = layout.top_rim.iter().sum();
        let exact = 24.0 * 2.0 * 90.0 * libm::sin(PI / 24.0);
        assert!((round - exact).abs() < 1e-9, "round rim {round} against {exact}");
        assert!(round < 2.0 * PI * 90.0);
        // Every bend is the rule's radius inside and the thickness more outside, with a real sweep.
        assert!(layout.bends.iter().all(|b| b.radius_in == 8.0 && b.radius_out == 10.0 && b.sweep > 0.0 && b.sweep < PI));
        // The seam runs from the middle of the first side to its apex.
        assert!((layout.seam.from - layout.seam.to).dot(layout.seam.from - layout.seam.to).sqrt() > 0.0);
        // A relief sector at each vertex: one end plane per bend (7 at a fan of 6) and two sides.
        assert_eq!(layout.reliefs.len(), 4);
        assert!(layout.reliefs.iter().all(|r| r.q.len() == 6 && r.ribs.len() == 7 && r.ends.len() == 7));
        assert!(layout.reliefs.iter().all(|r| r.sides.iter().all(|(_, n)| (n.dot(*n) - 1.0).abs() < 1e-9)));
        // Each tangent-line meeting sits by its vertex, a dozen millimetres up the fan: the
        // tangent offset is (radius_out - outer)·tan(sweep/2), ~1.3 mm at the fan's ~14.5° bends.
        for r in &layout.reliefs {
            let mut z: Vec<f64> = r.q.iter().map(|q| q.z).collect();
            z.sort_by(|a, b| a.total_cmp(b));
            for (got, want) in z.iter().zip([10.1138, 10.1138, 13.0519, 13.0519, 13.2608, 13.2608]) {
                assert!((got - want).abs() < 1e-3, "relief point heights {z:?}");
            }
        }
        // The bands stop short of the top rim: at the extreme bends, where their triangles' two
        // tangent lines meet (two millimetres below the rim); at the inner ones, as high as the
        // end face allows — nothing of it rises off the top profile's plane (D265).
        for b in &layout.bends {
            let extreme = b.facets.iter().any(|&f| f < 4);
            if extreme {
                assert!((b.top.z - 117.981).abs() < 1e-2, "bend top z {} against 117.981", b.top.z);
            } else {
                let (m, rho) = span_top(b, b.radius_in, b.radius_out, layout.az);
                let over = (b.top - layout.b[0]).dot(layout.az) + rho * m;
                assert!(over.abs() < 1e-9, "the end face's top off the profile plane: {over}");
            }
        }
    }

    #[test]
    fn every_bend_has_one_valid_lower_section_on_both_cylinders() {
        let feature = id("lower-sections");
        for side in [Side::Left, Side::Right, Side::Middle] {
            let mut d = dims();
            d.side = side;
            let layout = layout(feature, &square(), "ab", &circle(), "c", &d).unwrap();
            verify(&layout, &d).unwrap();
            let sections = lower_sections(&layout, &d).unwrap();
            assert_eq!(sections.len(), 28, "side {side:?}");

            for (i, (bend, section)) in layout.bends.iter().zip(&sections).enumerate() {
                assert!(section.station.is_finite(), "bend {i}, side {side:?}");
                assert!(distance3(section.center, bend.at + bend.axis * section.station) < 1e-10);
                for k in 0..2 {
                    let outer_radius = distance3(section.outer[k], section.center);
                    let inner_radius = distance3(section.inner[k], section.center);
                    assert!((outer_radius - bend.radius_out).abs() < 1e-9, "bend {i}, outer radius {outer_radius}");
                    assert!((inner_radius - bend.radius_in).abs() < 1e-9, "bend {i}, inner radius {inner_radius}");
                    assert!((distance3(section.outer[k], section.inner[k]) - d.thickness).abs() < 1e-9);
                    assert!((section.outer[k] - section.center).dot(bend.axis).abs() < 1e-9);
                    assert!((section.inner[k] - section.center).dot(bend.axis).abs() < 1e-9);
                }
            }
        }
    }

    #[test]
    fn every_facet_has_two_tangent_splits_and_bounded_sections() {
        let feature = id("facet-trims");
        for side in [Side::Left, Side::Right, Side::Middle] {
            let mut d = dims();
            d.side = side;
            let layout = layout(feature, &square(), "ab", &circle(), "c", &d).unwrap();
            let lower = lower_sections(&layout, &d).unwrap();
            let trims = facet_trims(&layout, &lower, &d).unwrap();
            assert_eq!(trims.len(), 28, "side {side:?}");
            // At least one rib's relief plane is defined by the q on its following fan facet,
            // so on this facet its lower section legitimately extends past the local q.
            let lower_relief_extensions = trims
                .iter()
                .flat_map(|trim| {
                    (0..2).filter_map(|slot| {
                        let bend = &layout.bends[trim.bends[slot]];
                        let lower_station = lower[trim.bends[slot]].station;
                        let q_station = (trim.q - bend.at).dot(bend.axis);
                        let x_station = (trim.x[slot] - bend.at).dot(bend.axis);
                        (!station_between(lower_station, q_station, x_station, 1.0)).then_some(())
                    })
                })
                .count();
            assert!(lower_relief_extensions > 0, "side {side:?}");
            assert_eq!(trims.iter().filter(|trim| trim.q.z < 30.0).count(), 24);
            assert_eq!(trims.iter().filter(|trim| trim.q.z > 90.0).count(), 4);
            for trim in trims {
                assert!(trim.facet < layout.facets.len());
                assert_eq!(trim.bends.len(), 2);
                for k in 0..2 {
                    assert!(distance3(trim.q, trim.x[k]) > 0.0);
                    assert!(distance3(trim.lower[k].outer, trim.lower[k].inner) > 0.0);
                    assert!(distance3(trim.upper[k].outer, trim.upper[k].inner) > 0.0);
                }
            }
        }
    }


    #[test]
    fn each_trimmed_facet_closes_as_a_thin_planar_patch() {
        let feature = id("facet-patch-prisms");
        for side in [Side::Left, Side::Right, Side::Middle] {
            let mut d = dims();
            d.side = side;
            let layout = layout(feature, &square(), "ab", &circle(), "c", &d).unwrap();
            let sections = lower_sections(&layout, &d).unwrap();
            let trims = facet_trims(&layout, &sections, &d).unwrap();
            let (patches, registry) = facet_patches(feature, &layout, &trims, &d).unwrap();
            assert_eq!(patches.len(), 28, "side {side:?}");
            assert!(registry.len() >= patches.len() * 6, "side {side:?}");
            for patch in &patches {
                let body = facet_patch_prism(feature, patch, &registry, layout.facets[patch.facet].normal).unwrap();
                body.validate().unwrap();
                assert!(body.volume() > 0.0, "facet {}, side {side:?}", patch.facet);
            }
        }
    }

    #[test]
    fn a_finer_tolerance_makes_more_facets() {
        let mut fine = dims();
        fine.tolerance = 0.1;
        let layout = layout(id("t"), &square(), "ab", &circle(), "c", &fine).unwrap();
        assert!(layout.fans.iter().all(|&m| m > 6), "fans {:?}", layout.fans);
        assert_eq!(layout.facets.len(), 4 + layout.fans.iter().sum::<usize>());
        assert_eq!(layout.bends.len(), 8 + layout.fans.iter().map(|m| m - 1).sum::<usize>());
    }

    #[test]
    fn an_arc_in_the_first_profile_is_refused() {
        let a = placed(
            json!({"points": {"a": [0, 0], "b": [80, 0], "c": [80, 80], "o": [0, 80]},
                   "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}},
                   "arcs": {"ca": {"center": "o", "from": "c", "to": "a"}}}),
            None,
        );
        let e = layout(id("t"), &a, "ab", &circle(), "c", &dims()).unwrap_err();
        assert!(e.to_string().contains("first profile is straight pieces"), "{e}");
    }

    #[test]
    fn planes_that_meet_are_refused() {
        let b = placed(
            json!({"points": {"o": [0, 0]}, "circles": {"c": {"center": "o", "diameter": 180}}}),
            Some(SketchPlane { origin: Vec3::new(0.0, 0.0, 120.0), x_axis: Vec3::new(1.0, 0.0, 0.0), normal: Vec3::new(0.0, 1.0, 0.0) }),
        );
        let e = layout(id("t"), &square(), "ab", &b, "c", &dims()).unwrap_err();
        assert!(e.to_string().contains("parallel planes"), "{e}");
    }

    #[test]
    fn a_bend_that_eats_a_facet_is_refused() {
        let mut big = dims();
        big.radius = 200.0;
        let e = layout(id("t"), &square(), "ab", &circle(), "c", &big).unwrap_err();
        assert!(e.to_string().contains("eats facet"), "{e}");
    }

    #[test]
    fn the_shell_of_the_etalon_validates() {
        let d = dims();
        let layout = layout(id("t"), &square(), "ab", &circle(), "c", &d).unwrap();
        let body = shell(id("t"), &layout, &d).unwrap();
        body.validate().unwrap();
        let mesh = datum_brep::tessellate(&body).unwrap();
        assert!(!mesh.triangles.is_empty());
        assert!(body.volume() > 0.0);
    }
}
