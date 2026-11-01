//! Revolution of a sketch's closed profile about a line of the sketch (D206).

use datum_brep::{Axis, Body, BrepError, Region, TopoId, revolve};
use datum_math::{Sign, Vec2, orient2d, parallel2d};

use crate::sketch::{Placed, chosen, profile_regions};

/// The closed profile of `s` (curves `profiles` choose it, D191) turned about line `axis` of the
/// sketch by `angle` degrees, counterclockwise looking against the line's direction (from its
/// `to` back to its `from`); 360 is a full turn. The profile lies on one side of the axis.
///
/// What the profile's edges turn into is decided exactly in the sketch's coordinates: an edge
/// parallel to the axis is a cylinder, one square to it a plane, a point on the axis stays on it.
/// The distances such points share are then made equal, so the solver's rounding cannot turn a
/// cylinder into a cone of no slope.
pub(crate) fn revolve_sketch(feature: TopoId, s: &Placed, axis: &str, angle: f64, profiles: &[String]) -> Result<Body, BrepError> {
    if s.developed.is_some() {
        return Err(BrepError::Unsupported("a revolution of a sketch on a cylinder face".into()));
    }
    if !(angle > 0.0 && angle <= 360.0) {
        return Err(BrepError::Degenerate(format!("a revolution turns by more than 0° and at most 360°, got {angle}°")));
    }
    let line = s.lines.get(axis).ok_or_else(|| BrepError::LostReference(format!("no line `{axis}` in the sketch to turn about")))?;
    let point = |name: &str| s.solved.points.get(name).copied().ok_or_else(|| BrepError::Sketch(format!("no point `{name}`")));
    let (o, x) = (point(&line.from)?, point(&line.to)?);
    if o == x {
        return Err(BrepError::Degenerate(format!("the axis `{axis}` has no length")));
    }
    let chosen = chosen(feature, s, profiles)?;
    let mut regions = profile_regions(feature, &chosen)?;
    if regions.len() != 1 {
        return Err(BrepError::Unsupported(format!("a revolution turns one closed profile, the sketch has {}: choose it", regions.len())));
    }
    let Region { outer: mut ring, holes, .. } = regions.remove(0);
    let n = ring.points.len();
    let sides: Vec<Sign> = ring.points.iter().map(|&p| orient2d(o, x, p)).collect();
    let left = sides.contains(&Sign::Positive);
    if left && sides.contains(&Sign::Negative) {
        return Err(BrepError::Degenerate("the profile crosses the axis".into()));
    }
    // In the axis's coordinates: along it from `o`, and the distance from it.
    let d = Vec2::new(x.x - o.x, x.y - o.y);
    let len = libm::hypot(d.x, d.y);
    let mut flat: Vec<Vec2> = ring
        .points
        .iter()
        .zip(&sides)
        .map(|(&p, &side)| {
            let w = Vec2::new(p.x - o.x, p.y - o.y);
            let y = if side == Sign::Zero { 0.0 } else { (d.x * w.y - d.y * w.x).abs() / len };
            Vec2::new((d.x * w.x + d.y * w.y) / len, y)
        })
        .collect();
    // Square to the axis: parallel to it turned by a right angle, which is exact.
    let normal = (Vec2::new(0.0, 0.0), Vec2::new(-d.y, d.x));
    for k in 0..n {
        let (a, b) = (ring.points[k], ring.points[(k + 1) % n]);
        if parallel2d(a, b, o, x) {
            flat[(k + 1) % n].y = flat[k].y;
        } else if parallel2d(a, b, normal.0, normal.1) {
            flat[(k + 1) % n].x = flat[k].x;
        }
    }
    ring.points = flat;
    let [origin, ex, ey, ez] = s.frame;
    let along = ex * (d.x / len) + ey * (d.y / len);
    let out = ez.cross(along);
    let at = Axis { origin: origin + ex * o.x + ey * o.y, axis: along, start: if left { out } else { -out } };
    let caps = ["revolve.cap_start", "revolve.cap_end"].map(|t| TopoId::derive(t).id(feature).finish());
    let turn = if angle == 360.0 { std::f64::consts::TAU } else { angle.to_radians() };
    revolve(&Region { outer: ring, holes, caps }, &at, turn)
}
