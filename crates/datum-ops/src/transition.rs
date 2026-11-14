//! A faceted transition between two closed profiles on parallel planes (D264): flat triangles
//! over the straight pieces of the first profile, faceted cones (fans) over its vertices, and a
//! real bend of the rule's radius on every edge between two facets. The blank edge follows the
//! facets. The corner where a fan's bends meet needs a relief the press can make; its shape waits
//! for the author (D264), so a body is refused there until it is settled — the разметка, the
//! bends and the checks below are what that relief is built on.

use std::collections::BTreeMap;
use std::f64::consts::PI;

use datum_brep::{BrepError, ProfileArc, ProfileLoop, TopoId};
use datum_math::{Vec2, Vec3};

use crate::sheet::Side;
use crate::sketch::{Placed, chosen, nearly_parallel, profile_regions, within_rounding};

/// Said when a fan's bends meet at a vertex of the first profile: the relief there is not designed
/// yet (D264, the author's answer is pending).
const CORNER: &str = "transition: the corner relief (a sector from the vertex, D264) is not built yet";

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
/// and its plane.
#[derive(Clone, Debug)]
pub(crate) struct Facet {
    pub(crate) loop_: Vec<TopoId>,
    pub(crate) origin: Vec3,
    pub(crate) normal: Vec3,
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
    /// The tangent points on the two facets' planes.
    pub(crate) tangent: [Vec3; 2],
    /// The angle the outer surface turns through: between the two facets' outward normals.
    pub(crate) sweep: f64,
}

/// The seam: a cut through the middle of the flat triangle over the first piece of `a`.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Seam {
    pub(crate) facet: usize,
    pub(crate) from: Vec3,
    pub(crate) to: Vec3,
    pub(crate) gap: f64,
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
        if !points.iter().any(|p| p.x == f.1.x && p.y == f.1.y) {
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
    let Some(pick) = near.iter().filter(|f| f.1 == points[0]).min_by_key(|f| f.2).copied() else {
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
    let a_id = |i: usize| TopoId::derive("transition.a").id(feature).index(i as u64).finish();
    let b_id = |i: usize| TopoId::derive("transition.b").id(feature).index(i as u64).finish();
    let f_id = |j: usize, k: usize| TopoId::derive("transition.fan").id(feature).index(j as u64).index(k as u64).finish();
    let mut vertices: BTreeMap<TopoId, Vec3> = BTreeMap::new();
    for i in 0..n {
        vertices.insert(a_id(i), a3[i]);
        vertices.insert(b_id(i), b3[i]);
    }
    for (j, pts) in fan3.iter().enumerate() {
        for (k, &point) in pts.iter().enumerate().skip(1).take(pts.len().saturating_sub(2)) {
            vertices.insert(f_id(j, k), point);
        }
    }
    let fan_vid = |j: usize, k: usize, m: usize| -> TopoId {
        if k == 0 {
            b_id((j + n - 1) % n)
        } else if k == m {
            b_id(j)
        } else {
            f_id(j, k)
        }
    };
    let axis_center = average(&a3);
    let mut facets: Vec<Facet> = Vec::with_capacity(n + fans.iter().sum::<usize>());
    let orient = |ids: Vec<TopoId>, pts: &[Vec3]| -> Result<Facet, BrepError> {
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
        Ok(Facet { loop_: ids, origin: pts[0], normal: unit3(nrm) })
    };
    // The flat triangle over each piece of `a`.
    for i in 0..n {
        let pts = [a3[i], a3[(i + 1) % n], b3[i]];
        facets.push(orient(vec![a_id(i), a_id((i + 1) % n), b_id(i)], &pts)?);
    }
    // The fan over each vertex of `a`.
    for j in 0..n {
        let m = fans[j];
        for k in 0..m {
            let pts = [a3[j], fan3[j][k + 1], fan3[j][k]];
            facets.push(orient(vec![a_id(j), fan_vid(j, k + 1, m), fan_vid(j, k, m)], &pts)?);
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
                let tangent = [at + f.normal * radius_out, at + g.normal * radius_out];
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
                bends.push(Bend { facets: [*fi, *gi], edge: [*u, *v], at, axis: d, radius_in, radius_out, tangent, sweep });
            }
            [.., _, _] => {
                return Err(BrepError::NonManifold(format!("a transition's edge {u:?} → {v:?} is used by {} facets", faces.len())));
            }
            _ => {}
        }
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
    Ok(Layout { vertices, a: a3, b: b3, fans, facets, bends, top_rim, bottom_rim, seam })
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
    // Every closed profile has a vertex where its fan's bends meet; the press cannot form them
    // together, so a relief is cut there. Its shape waits for the author (D264).
    let rims: f64 = layout.top_rim.iter().chain(&layout.bottom_rim).sum();
    let seam = (layout.seam.to - layout.seam.from).dot(layout.seam.to - layout.seam.from).sqrt();
    Err(BrepError::Unsupported(format!(
        "{CORNER}: {} facets, {} bends ({:.3} rad), {} fan corners, rims {:.3} mm, the seam {:.3} mm open by {} mm",
        layout.facets.len(),
        layout.bends.len(),
        layout.bends.iter().map(|b| b.sweep).sum::<f64>(),
        layout.fans.len(),
        rims,
        seam,
        layout.seam.gap
    )))
}

/// Checks the разметка against itself (D264: a wrong layout is turned away here, before it is
/// developed): every facet's vertices lie on its plane, every bend's axis along its edge and at the
/// outer radius from both facets, its tangents at that radius from the axis, its sweep the angle
/// between the facets' normals.
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
    for b in &layout.bends {
        let (u, v) = (b.edge[0], b.edge[1]);
        let p1 = *layout.vertices.get(&u).ok_or_else(|| BrepError::Invalid(format!("transition: bend vertex {u:?} is missing")))?;
        let p2 = *layout.vertices.get(&v).ok_or_else(|| BrepError::Invalid(format!("transition: bend vertex {v:?} is missing")))?;
        if !nearly_parallel(unit3(p2 - p1), b.axis) {
            return Err(BrepError::Invalid("transition: a bend's axis is not along its edge".into()));
        }
        for (fi, t) in b.facets.iter().zip(b.tangent) {
            let f = &layout.facets[*fi];
            let dist = (b.at - f.origin).dot(f.normal) - outer;
            if !within_rounding(dist + b.radius_out, size) {
                return Err(BrepError::Invalid("transition: a bend's axis is not at the outer radius from a facet".into()));
            }
            let w = t - b.at;
            if !within_rounding(w.dot(w).sqrt() - b.radius_out, size) || !within_rounding(w.dot(f.normal) - b.radius_out, size) {
                return Err(BrepError::Invalid("transition: a bend's tangent point is off its facet".into()));
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
    let seam = &layout.seam;
    if seam.facet >= layout.facets.len() || !(seam.gap > 0.0 && seam.gap.is_finite()) {
        return Err(BrepError::Invalid("transition: the seam names no facet, or has no gap".into()));
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
    fn the_body_waits_for_the_corner_relief() {
        let e = transition(id("t"), &square(), "ab", &circle(), "c", dims()).unwrap_err();
        assert!(e.to_string().contains("the corner relief (a sector from the vertex, D264) is not built yet"), "{e}");
    }
}
