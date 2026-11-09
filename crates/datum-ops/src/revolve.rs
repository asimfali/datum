//! Revolutions about a line of a sketch (D206, D207): a closed profile into a body, a line into
//! rolled sheet metal (a shell or a cone).

use std::f64::consts::TAU;

use datum_brep::{Axis, Body, BrepError, ProfileArc, ProfileLoop, Region, TopoId, revolve};
use datum_math::{Sign, Vec2, Vec3, orient2d, parallel2d};

use crate::sheet::Side;
use crate::sketch::{Placed, chosen, profile_regions};

/// A line of a sketch to turn about, and how points of the sketch lie to it.
struct Turn {
    o: Vec2,
    x: Vec2,
    at: Axis,
}

impl Turn {
    /// Line `axis` of `s`; profiles lie on its left (`start` points there) unless `right`.
    fn of(s: &Placed, axis: &str) -> Result<(Self, [Vec3; 2]), BrepError> {
        if s.developed.is_some() {
            return Err(BrepError::Unsupported("a revolution of a sketch on a cylinder face".into()));
        }
        let line = s.lines.get(axis).ok_or_else(|| BrepError::LostReference(format!("no line `{axis}` in the sketch to turn about")))?;
        let point = |name: &str| s.solved.points.get(name).copied().ok_or_else(|| BrepError::Sketch(format!("no point `{name}`")));
        let (o, x) = (point(&line.from)?, point(&line.to)?);
        if o == x {
            return Err(BrepError::Degenerate(format!("the axis `{axis}` has no length")));
        }
        let [origin, ex, ey, ez] = s.frame;
        let len = libm::hypot(x.x - o.x, x.y - o.y);
        let along = ex * ((x.x - o.x) / len) + ey * ((x.y - o.y) / len);
        let left = ez.cross(along);
        Ok((Self { o, x, at: Axis { origin: origin + ex * o.x + ey * o.y, axis: along, start: left } }, [left, -left]))
    }

    /// `points` (a closed loop if `closed`, else a chain) in the axis's coordinates: along it from
    /// its start, and the distance from it; and whether they lie on its right. What their edges
    /// turn into is decided in the sketch's coordinates: an edge parallel to the axis keeps its
    /// distance, one square to it its place along it, a point on the axis lies on it. Parallel or
    /// square exactly, or as far as the sketch's solver can tell (`tolerance`, D228): its ends
    /// off by no more than that across or along the axis. The shared coordinates are then made
    /// equal, so rounding cannot turn a cylinder into a cone of no slope.
    fn flat(&self, points: &[Vec2], closed: bool, tolerance: f64) -> Result<(Vec<Vec2>, bool), BrepError> {
        let (o, x) = (self.o, self.x);
        let sides: Vec<Sign> = points.iter().map(|&p| orient2d(o, x, p)).collect();
        let right = sides.contains(&Sign::Negative);
        if right && sides.contains(&Sign::Positive) {
            return Err(BrepError::Degenerate("the profile crosses the axis".into()));
        }
        let d = Vec2::new(x.x - o.x, x.y - o.y);
        let len = libm::hypot(d.x, d.y);
        let mut flat: Vec<Vec2> = points
            .iter()
            .zip(&sides)
            .map(|(&p, &side)| {
                let w = Vec2::new(p.x - o.x, p.y - o.y);
                let y = (d.x * w.y - d.y * w.x).abs() / len;
                // On the axis exactly, or as far as the solver can tell (D228).
                let y = if side == Sign::Zero || y <= tolerance { 0.0 } else { y };
                Vec2::new((d.x * w.x + d.y * w.y) / len, y)
            })
            .collect();
        // Square to the axis: parallel to it turned by a right angle, which is exact.
        let normal = (Vec2::new(0.0, 0.0), Vec2::new(-d.y, d.x));
        let n = points.len();
        for k in 0..if closed { n } else { n - 1 } {
            let (a, b) = (points[k], points[(k + 1) % n]);
            let (p, q) = (flat[k], flat[(k + 1) % n]);
            if parallel2d(a, b, o, x) || (p.y - q.y).abs() <= tolerance {
                flat[(k + 1) % n].y = flat[k].y;
            } else if parallel2d(a, b, normal.0, normal.1) || (p.x - q.x).abs() <= tolerance {
                flat[(k + 1) % n].x = flat[k].x;
            }
        }
        Ok((flat, right))
    }

    /// Arc `arc` in the axis's coordinates (see [`Turn::flat`]): seen from the right of the axis
    /// the distance from it runs the other way, and so does the arc. Its centre may lie across
    /// the axis, negative; on the axis as far as the solver can tell, it is on it.
    fn flat_arc(&self, arc: ProfileArc, right: bool, tolerance: f64) -> ProfileArc {
        let (o, d) = (self.o, Vec2::new(self.x.x - self.o.x, self.x.y - self.o.y));
        let len = libm::hypot(d.x, d.y);
        let w = Vec2::new(arc.center.x - o.x, arc.center.y - o.y);
        let y = (d.x * w.y - d.y * w.x) / len * if right { -1.0 } else { 1.0 };
        ProfileArc { center: Vec2::new((d.x * w.x + d.y * w.y) / len, if y.abs() <= tolerance { 0.0 } else { y }), ccw: arc.ccw != right }
    }
}

/// The closed profile of `s` (curves `profiles` choose it, D191) turned about line `axis` of the
/// sketch by `angle` degrees, counterclockwise looking against the line's direction (from its
/// `to` back to its `from`); 360 is a full turn. The profile lies on one side of the axis.
pub(crate) fn revolve_sketch(feature: TopoId, s: &Placed, axis: &str, angle: f64, profiles: &[String]) -> Result<Body, BrepError> {
    if !(angle > 0.0 && angle <= 360.0) {
        return Err(BrepError::Degenerate(format!("a revolution turns by more than 0° and at most 360°, got {angle}°")));
    }
    let (turn, [left, right]) = Turn::of(s, axis)?;
    let chosen = chosen(feature, s, profiles)?;
    let mut regions = profile_regions(feature, &chosen)?;
    if regions.len() != 1 {
        return Err(BrepError::Unsupported(format!("a revolution turns one closed profile, the sketch has {}: choose it", regions.len())));
    }
    let Region { outer: mut ring, holes, .. } = regions.remove(0);
    let (flat, on_right) = turn.flat(&ring.points, true, s.solved.tolerance)?;
    ring.points = flat;
    for arc in ring.arcs.iter_mut().flatten() {
        *arc = turn.flat_arc(*arc, on_right, s.solved.tolerance);
    }
    let at = Axis { start: if on_right { right } else { left }, ..turn.at };
    let caps = ["revolve.cap_start", "revolve.cap_end"].map(|t| TopoId::derive(t).id(feature).finish());
    let angle = if angle == 360.0 { TAU } else { angle.to_radians() };
    revolve(&Region { outer: ring, holes, caps }, &at, angle)
}

/// Rolled sheet metal (D207): its thickness and K-factor (the part's rule's), the side of the
/// line it lies on, and how far round it goes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RollDims {
    pub thickness: f64,
    pub k_factor: f64,
    pub side: Side,
    pub span: Span,
}

/// How far round rolled sheet goes (D213): a whole turn but a gap at its seam (millimetres), or
/// a piece of `degrees` from the sketch's plane.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Span {
    Full { gap: f64 },
    Part { degrees: f64 },
}

/// Line `path` of `s` (none: its only line other than the axis and construction lines) turned
/// about line `axis` into rolled sheet metal (D207): a line parallel to the axis rolls into a
/// shell, an oblique one into a cone. The sheet lies on `side` of the line looking along it, its
/// ends square to it (the blank is cut square). A whole turn leaves `gap` millimetres open at
/// the seam, measured along the neutral layer at the larger end, half each side of the sketch's
/// plane; a piece starts at the sketch's plane, the line on its edge, and turns as a revolution
/// does (D213).
pub(crate) fn rolled(feature: TopoId, s: &Placed, axis: &str, path: Option<&str>, dims: RollDims) -> Result<Body, BrepError> {
    let RollDims { thickness: t, k_factor: k, side, span } = dims;
    if !(t > 0.0 && t.is_finite()) {
        return Err(BrepError::Degenerate(format!("rolled sheet needs a positive thickness, got {t}")));
    }
    if !(0.0..=1.0).contains(&k) {
        return Err(BrepError::Degenerate(format!("a K-factor lies between 0 and 1, got {k}")));
    }
    match span {
        Span::Full { gap } if !(gap > 0.0 && gap.is_finite()) => {
            return Err(BrepError::Degenerate(format!("rolled sheet needs a gap at its seam, or it closes and cannot be unrolled: got {gap}")));
        }
        Span::Part { degrees } if !(degrees > 0.0 && degrees < 360.0) => {
            return Err(BrepError::Degenerate(format!("a piece of rolled sheet turns by more than 0° and less than 360°, got {degrees}°")));
        }
        _ => {}
    }
    if !s.arcs.is_empty() || !s.circles.is_empty() {
        return Err(BrepError::Unsupported("rolled sheet from arcs: its flat pattern is not made yet".into()));
    }
    let name = match path {
        Some(p) => p.to_string(),
        None => {
            let lines: Vec<&String> = s.lines.iter().filter(|(n, l)| *n != axis && !l.construction).map(|(n, _)| n).collect();
            match lines[..] {
                [one] => one.clone(),
                [] => return Err(BrepError::Sketch("rolled sheet: the sketch has no line to roll besides the axis".into())),
                _ => return Err(BrepError::Unsupported("rolled sheet rolls one line: choose it (`path`)".into())),
            }
        }
    };
    if name == axis {
        return Err(BrepError::Sketch("rolled sheet: the line to roll is the axis".into()));
    }
    let line = s.lines.get(&name).ok_or_else(|| BrepError::LostReference(format!("no line `{name}` in the sketch to roll")))?;
    let point = |n: &str| s.solved.points.get(n).copied().ok_or_else(|| BrepError::Sketch(format!("no point `{n}`")));
    let (turn, [left, right]) = Turn::of(s, axis)?;
    let (ends, on_right) = turn.flat(&[point(&line.from)?, point(&line.to)?], false, s.solved.tolerance)?;
    let (a, b) = (ends[0], ends[1]);
    if a.x == b.x {
        return Err(BrepError::Unsupported(format!("the line `{name}` is square to the axis: it turns into a flat ring, a face of sheet metal")));
    }
    // Left of the line in the sketch is left in the axis's coordinates unless they are mirrored.
    let len = libm::hypot(b.x - a.x, b.y - a.y);
    if len == 0.0 {
        return Err(BrepError::Degenerate(format!("the line `{name}` has no length")));
    }
    let n = Vec2::new(-(b.y - a.y) / len, (b.x - a.x) / len);
    let n = if on_right { Vec2::new(-n.x, -n.y) } else { n };
    let (near, far) = match side {
        Side::Left => (0.0, t),
        Side::Right => (-t, 0.0),
        Side::Middle => (-0.5 * t, 0.5 * t),
    };
    // Offset by `d` along `n`: a line along the axis or square to it moves one coordinate only.
    let off = |p: Vec2, d: f64| Vec2::new(if n.x == 0.0 { p.x } else { p.x + n.x * d }, if n.y == 0.0 { p.y } else { p.y + n.y * d });
    let points = vec![off(a, near), off(b, near), off(b, far), off(a, far)];
    // The neutral layer: `k` of the thickness from the face nearer the axis.
    let inner = if n.y > 0.0 { near } else { far };
    let neutral = inner + (far - near) * k * if n.y > 0.0 { 1.0 } else { -1.0 };
    let larger = off(a, neutral).y.max(off(b, neutral).y);
    let id = |tag: &str, k: u64| TopoId::derive(tag).id(feature).tag(&name).index(k).finish();
    let ring = ProfileLoop {
        points,
        vertices: (0..4).map(|k| [id("rolled.v", k); 2]).collect(),
        sides: (0..4).map(|k| id("rolled.side", k)).collect(),
        arcs: Vec::new(),
    };
    let caps = ["rolled.seam_start", "rolled.seam_end"].map(|t| TopoId::derive(t).id(feature).finish());
    let base = Axis { start: if on_right { right } else { left }, ..turn.at };
    let region = Region { outer: ring, holes: Vec::new(), caps };
    match span {
        // The seam half the gap past the sketch's plane each way.
        Span::Full { gap } => {
            let open = gap / larger;
            if open >= TAU {
                return Err(BrepError::Degenerate(format!("a gap of {gap} leaves nothing of a sheet {} round", TAU * larger)));
            }
            revolve(&region, &Axis { start: base.radial(0.5 * open), ..base }, TAU - open)
        }
        Span::Part { degrees } => revolve(&region, &base, degrees.to_radians()),
    }
}
