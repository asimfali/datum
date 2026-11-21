//! Sheet lofts between line profiles, coaxial circular arcs, or a circle and a rectangle.

mod rect;

use std::collections::BTreeMap;

use datum_brep::{Arc, Body, BrepError, Curve, CurveSpec, Cylinder, FaceSpec, Plane, ProfileArc, Surface, TopoId, Vertex, build_with_curves};
use datum_math::{Sign, Vec2, Vec3, orient3d_coords};

use crate::sheet::{Sheet, loft_chords};
use crate::sketch::within_rounding;
use crate::{Placed, Side, parallel};

/// Seam gap in millimetres for a new closed loft.
pub const DEFAULT_LOFT_GAP: f64 = 1.0;

/// Sheet and approximation parameters for the D264 loft cases.
#[derive(Clone, Copy)]
pub struct LoftDims {
    pub thickness: f64,
    pub radius: Option<f64>,
    pub k_factor: Option<f64>,
    pub side: Side,
    pub tolerance: f64,
    pub gap: Option<f64>,
}

/// Builds a sheet loft from a line or circular arc.
///
/// In the planar case the line profiles must be parallel and coplanar in world space. For
/// the bent cases the first line must lie in a plane parallel to the second profile.
pub fn loft(feature: TopoId, first: &Placed, second: &Placed, dims: LoftDims) -> Result<Body, BrepError> {
    let LoftDims { thickness, radius, k_factor, side, tolerance, gap } = dims;
    if !(thickness > 0.0 && thickness.is_finite()) {
        return Err(BrepError::Degenerate(format!("loft: thickness must be positive and finite, got {thickness}")));
    }
    if !(tolerance > 0.0 && tolerance.is_finite()) {
        return Err(BrepError::Degenerate(format!("loft: tolerance must be positive and finite, got {tolerance}")));
    }

    if first.circles.values().any(|circle| !circle.construction) || second.circles.values().any(|circle| !circle.construction) {
        let (radius, k) = loft_bend_rule(radius, k_factor)?;
        let (circle, rectangle) = if first.circles.values().any(|c| !c.construction) { (first, second) } else { (second, first) };
        let body = rect::circle_rectangle(feature, circle, rectangle, Sheet { thickness, radius, side }, tolerance, gap.unwrap_or(DEFAULT_LOFT_GAP))?;
        check_blank(&body, k, "the circle and rectangle")?;
        return Ok(body);
    }
    if first.arcs.values().any(|arc| !arc.construction) {
        let (radius, k) = loft_bend_rule(radius, k_factor)?;
        let body = arc_loft(feature, first, second, Sheet { thickness, radius, side }, tolerance)?;
        check_blank(&body, k, "the arc profiles")?;
        return Ok(body);
    }
    let (a0, a1) = profile_line(first, "first")?;
    let second_lines = profile_lines(second, "second")?;
    match second_lines.as_slice() {
        [(_, b0, b1)] => planar_loft(feature, a0, a1, *b0, *b1, thickness, side),
        [_, _] | [_, _, _] => {
            let (radius, k) = loft_bend_rule(radius, k_factor)?;
            let profile = bent_profile([a0, a1], second, &second_lines)?;
            let rule = BendRule { thickness, radius, side };
            let body = two_bend_body(feature, [a0, a1], profile, rule)?;
            let names = second_lines.iter().map(|line| line.0.as_str()).collect::<Vec<_>>().join(", ");
            check_blank(&body, k, &format!("second profile lines {names}"))?;
            Ok(body)
        }
        [] => Err(BrepError::Sketch("loft: the second sketch has no profile line".into())),
        _ => Err(BrepError::Unsupported("loft: the second profile must be one line or a chain of two or three lines".into())),
    }
}

fn loft_bend_rule(radius: Option<f64>, k_factor: Option<f64>) -> Result<(f64, f64), BrepError> {
    let (Some(radius), Some(k)) = (radius, k_factor) else {
        return Err(BrepError::Invalid("loft with bends takes thickness, bend radius and K-factor from the part's sheet rule".into()));
    };
    if !(radius > 0.0 && radius.is_finite()) {
        return Err(BrepError::Degenerate(format!("loft: bend radius must be positive and finite, got {radius}")));
    }
    if !(0.0..=1.0).contains(&k) {
        return Err(BrepError::Degenerate(format!("loft: K-factor must lie between 0 and 1, got {k}")));
    }
    Ok((radius, k))
}

fn check_blank(body: &Body, k: f64, profile: &str) -> Result<(), BrepError> {
    if !crate::unfold_side(body, false, k)?.overlaps.is_empty() {
        return Err(BrepError::Degenerate(format!("loft: the blank overlaps for {profile}")));
    }
    Ok(())
}

fn single_arc<'a>(sketch: &'a Placed, which: &str) -> Result<(&'a str, &'a datum_sketch::Arc), BrepError> {
    let unsupported = || BrepError::Unsupported(format!("loft: the {which} profile must be one open circular arc"));
    let mut arcs = sketch.arcs.iter().filter(|(_, arc)| !arc.construction);
    let (name, arc) = arcs.next().ok_or_else(unsupported)?;
    if arcs.next().is_some()
        || sketch.developed.is_some()
        || sketch.lines.values().any(|line| !line.construction)
        || sketch.circles.values().any(|circle| !circle.construction)
        || !sketch.ellipses.is_empty()
    {
        return Err(unsupported());
    }
    Ok((name, arc))
}

fn arc_loft(feature: TopoId, first: &Placed, second: &Placed, sheet: Sheet, tolerance: f64) -> Result<Body, BrepError> {
    let (name, a) = single_arc(first, "first")?;
    let (_, b) = single_arc(second, "second")?;
    let normal = first.frame[3];
    let other_normal = second.frame[3];
    let incompatible = || BrepError::Unsupported("loft: arc profiles must be coaxial on parallel planes and have matching angular ends".into());
    if !within_rounding(length(normal.cross(other_normal)), 1.0) {
        return Err(incompatible());
    }
    let distance = (second.frame[0] - first.frame[0]).dot(normal);
    if distance == 0.0 || !distance.is_finite() {
        return Err(BrepError::Degenerate("loft: arc profiles have no separation".into()));
    }
    let ac = profile_point(first, "first", &a.center)?;
    let bc = profile_point(second, "second", &b.center)?;
    let offset = normal * distance;
    let matches = |pa: Vec3, pb: Vec3| {
        let scale = length(pa).max(length(pb)).max(distance.abs());
        within_rounding(length(pb - pa - offset), scale)
    };
    if !matches(ac, bc) {
        return Err(incompatible());
    }
    let (b_from, b_to) = if normal.dot(other_normal) > 0.0 { (&b.from, &b.to) } else { (&b.to, &b.from) };
    let ap = profile_point(first, "first", &a.from)?;
    let aq = profile_point(first, "first", &a.to)?;
    let bp = profile_point(second, "second", b_from)?;
    let bq = profile_point(second, "second", b_to)?;
    let b_radius = length(bp - bc);
    for (u, v) in [(ap - ac, bp - bc), (aq - ac, bq - bc)] {
        let (Some(u), Some(v)) = (unit(u), unit(v)) else {
            return Err(BrepError::Degenerate("loft: an arc has no radius".into()));
        };
        if !within_rounding(length(u - v), 1.0) {
            return Err(incompatible());
        }
    }
    let translated = matches(ap, bp) && matches(aq, bq);

    let point = |name: &str| {
        first.solved.points.get(name).copied().ok_or_else(|| BrepError::LostReference(format!("loft: the first profile lost point {name}")))
    };
    let (center, start, end) = (point(&a.center)?, point(&a.from)?, point(&a.to)?);
    let radial = Vec2::new(start.x - center.x, start.y - center.y);
    let radius = libm::hypot(radial.x, radial.y);
    if !(radius > 0.0 && radius.is_finite()) || start == end {
        return Err(BrepError::Degenerate("loft: the first arc must have a radius and two distinct ends".into()));
    }
    let sweep = ProfileArc { center, ccw: true }.turn(start, end);
    let count = chord_count(if translated { radius } else { radius.max(b_radius) }, sweep, tolerance)?;
    let mut points = Vec::with_capacity(count + 1);
    points.push(start);
    for k in 1..count {
        let (sn, cs) = libm::sincos(sweep * k as f64 / count as f64);
        points.push(Vec2::new(center.x + radial.x * cs - radial.y * sn, center.y + radial.x * sn + radial.y * cs));
    }
    points.push(end);
    if translated {
        return loft_chords(feature, first.frame, name, &points, sheet, distance);
    }
    let [origin, x, y, _] = first.frame;
    let at = |p: Vec2| origin + x * p.x + y * p.y;
    let a: Vec<_> = points.into_iter().map(at).collect();
    let b: Vec<_> = a.iter().map(|&p| ac + (p - ac) * (b_radius / radius) + offset).collect();
    tapered_arc_body(feature, name, &a, &b, sheet, distance.is_sign_negative())
}

fn chord_count(radius: f64, sweep: f64, tolerance: f64) -> Result<usize, BrepError> {
    // Invert the chord's sagitta without subtracting two almost equal numbers.
    let max_angle = 4.0 * libm::asin(libm::sqrt((0.5 * (tolerance / radius)).min(1.0)));
    let n = (sweep / max_angle).ceil();
    const MAX_CHORDS: usize = 4096;
    let too_many = || BrepError::Unsupported(format!("loft: the tolerance requires more than {MAX_CHORDS} chords"));
    if !n.is_finite() || n > (MAX_CHORDS + 1) as f64 {
        return Err(too_many());
    }
    let sagitta = |count: usize| {
        let sine = libm::sin(0.25 * sweep / count as f64);
        2.0 * (radius * (sine * sine))
    };
    // Check the neighbouring counts at a floating-point boundary of the inverse.
    let mut count = (n as usize).max(1);
    if count > 1 && sagitta(count - 1) <= tolerance {
        count -= 1;
    } else if sagitta(count) > tolerance {
        count += 1;
    }
    if count > MAX_CHORDS {
        return Err(too_many());
    }
    Ok(count)
}

/// The one-face trapezoid: the first complete loft case.
fn planar_loft(feature: TopoId, a0: Vec3, a1: Vec3, mut b0: Vec3, mut b1: Vec3, thickness: f64, side: Side) -> Result<Body, BrepError> {
    let direction = a1 - a0;
    let other_direction = b1 - b0;
    if direction.dot(direction) == 0.0 || other_direction.dot(other_direction) == 0.0 {
        return Err(BrepError::Degenerate("loft: a line profile has zero length".into()));
    }
    if !parallel(direction, other_direction) {
        return Err(BrepError::Degenerate("loft: the two line profiles are not parallel".into()));
    }
    if direction.dot(other_direction) < 0.0 {
        std::mem::swap(&mut b0, &mut b1);
    }
    if orient3d_coords(a0, a1, b0, b1) != Sign::Zero {
        return Err(BrepError::Degenerate("loft: the four solved profile endpoints are not coplanar".into()));
    }

    let normal = unit(direction.cross(b0 - a0))
        .ok_or_else(|| BrepError::Degenerate("loft: the two profile lines coincide; the trapezoid has no area".into()))?;
    let (low, high) = thickness_sides(side, thickness);

    let base = [a0, a1, b1, b0];
    let outer: [Vec3; 4] = base.map(|p| p + normal * high);
    let inner: [Vec3; 4] = base.map(|p| p + normal * low);
    let outer_ids: [TopoId; 4] = std::array::from_fn(|i| vertex_id(feature, i as u64));
    let inner_ids: [TopoId; 4] = std::array::from_fn(|i| vertex_id(feature, (i + 4) as u64));

    let mut vertices = Vec::with_capacity(8);
    for i in 0..4 {
        vertices.push(Vertex { id: outer_ids[i], point: outer[i] });
        vertices.push(Vertex { id: inner_ids[i], point: inner[i] });
    }

    // One planar face on each side of the sheet. The second loop is reversed so every
    // perimeter edge has the opposite use in its thickness face.
    let mut faces = Vec::with_capacity(6);
    faces.push(planar_face(face_id(feature, 0), outer_ids.to_vec(), outer.to_vec())?);
    faces.push(planar_face(face_id(feature, 1), (0..4).rev().map(|i| inner_ids[i]).collect(), (0..4).rev().map(|i| inner[i]).collect())?);

    // The four thickness faces close the sheet around the trapezoid's real boundary.
    for i in 0..4 {
        let j = (i + 1) % 4;
        faces.push(planar_face(
            face_id(feature, (i + 2) as u64),
            vec![outer_ids[i], inner_ids[i], inner_ids[j], outer_ids[j]],
            vec![outer[i], inner[i], inner[j], outer[j]],
        )?);
    }

    build_with_curves(vertices, faces, Vec::new())
}

#[derive(Clone, Copy)]
struct Bend {
    axis: Vec3,
    center: Vec3,
    middle_tangent: Vec3,
    wing_tangent: Vec3,
    middle_in: Vec3,
    wing_in: Vec3,
    radial_sign: f64,
    length: f64,
}

#[derive(Clone, Copy)]
struct BendRule {
    thickness: f64,
    radius: f64,
    side: Side,
}

enum BendProfile {
    Shared { corner: Vec3, ends: [Vec3; 2] },
    Separate { corners: [Vec3; 2], ends: [Vec3; 2] },
}

fn bent_profile(profile: [Vec3; 2], second: &Placed, lines: &[(String, Vec3, Vec3)]) -> Result<BendProfile, BrepError> {
    let [a0, a1] = profile;
    let displacement = a1 - a0;
    let direction = unit(displacement).ok_or_else(|| BrepError::Degenerate("loft: the first profile line has zero length".into()))?;
    if !within_rounding(direction.dot(second.frame[3]), 1.0) {
        return Err(BrepError::Unsupported("loft: the first profile line must be parallel to the plane of the second profile".into()));
    }
    if lines.len() == 2 {
        let (left, corner, right) = second_profile_corner(second, lines, a0 + displacement * 0.5, direction)?;
        return Ok(BendProfile::Shared { corner, ends: [left, right] });
    }

    let chain = second.contours(&|_, construction| !construction);
    let [path] = chain.paths.as_slice() else {
        return Err(BrepError::Sketch("loft: the three lines must form one open chain".into()));
    };
    let [first, middle, last] = path.as_slice() else {
        return Err(BrepError::Sketch("loft: the three lines must form one open chain".into()));
    };
    let point = |name| profile_point(second, "second", name);
    let [left, mut b0, mut b1, right] = [point(first.from)?, point(middle.from)?, point(middle.to)?, point(last.to)?];
    let mut ends = [left, right];
    let center = b1 - b0;
    if center.dot(center) == 0.0 || !parallel(displacement, center) || orient3d_coords(a0, a1, b0, b1) != Sign::Zero {
        return Err(BrepError::Degenerate("loft: A and the middle line of B must be nonzero, parallel and coplanar".into()));
    }
    if center.dot(direction) < 0.0 {
        std::mem::swap(&mut b0, &mut b1);
        ends.reverse();
    }
    Ok(BendProfile::Separate { corners: [b0, b1], ends })
}

fn bend(start: Vec3, apex: Vec3, opposite: [Vec3; 2], radial_sign: f64, rule: BendRule) -> Result<Bend, BrepError> {
    let axis_vector = apex - start;
    let axis = unit(axis_vector).ok_or_else(|| BrepError::Degenerate("loft: a bend has zero length".into()))?;
    let across = opposite.map(|p| {
        let offset = p - start;
        offset - axis * offset.dot(axis)
    });
    let [middle_in, wing_in] = across.map(unit);
    let (Some(middle_in), Some(wing_in)) = (middle_in, wing_in) else {
        return Err(BrepError::Degenerate("loft: a facet has no width across its bend".into()));
    };
    let bisector = unit(middle_in + wing_in).ok_or_else(|| BrepError::Degenerate("loft: a bend has no unique bisector".into()))?;
    let (low, high) = thickness_sides(rule.side, rule.thickness);
    let reference_radius = rule.radius - (radial_sign * low).min(radial_sign * high);
    let angle = libm::atan2(length(middle_in.cross(wing_in)), -middle_in.dot(wing_in));
    let half_cos = libm::cos(0.5 * angle);
    let tangent_offset = reference_radius * libm::tan(0.5 * angle);
    if !(tangent_offset > 0.0 && across.into_iter().all(|v| tangent_offset < length(v))) {
        return Err(BrepError::Degenerate(format!("loft: bend radius {} mm leaves no room for its tangent lines", rule.radius)));
    }
    Ok(Bend {
        axis,
        center: start + bisector * (reference_radius / half_cos),
        middle_tangent: start + middle_in * tangent_offset,
        wing_tangent: start + wing_in * tangent_offset,
        middle_in,
        wing_in,
        radial_sign,
        length: length(axis_vector),
    })
}

/// Where two lines in a facet meet, with their parameters along d and e.
fn line_meeting(p: Vec3, d: Vec3, q: Vec3, e: Vec3) -> Result<(Vec3, f64, f64), BrepError> {
    line_meeting_at_scale(p, d, q, e, 0.0)
}

fn line_meeting_at_scale(p: Vec3, d: Vec3, q: Vec3, e: Vec3, scale: f64) -> Result<(Vec3, f64, f64), BrepError> {
    let n = d.cross(e);
    let n2 = n.dot(n);
    if !(n2 > 0.0 && n2.is_finite()) {
        return Err(BrepError::Degenerate("loft: the lines do not define a unique trim point".into()));
    }
    let delta = q - p;
    let s = delta.cross(e).dot(n) / n2;
    let t = delta.cross(d).dot(n) / n2;
    let first = d * s;
    let second = delta + e * t;
    let size = length(first).max(length(delta)).max(length(e * t)).max(scale);
    if !within_rounding(length(first - second), size) {
        return Err(BrepError::Degenerate("loft: lines used to trim a bend are skew".into()));
    }
    Ok((p + first, s, t))
}

fn second_profile_corner(sketch: &Placed, lines: &[(String, Vec3, Vec3)], midpoint: Vec3, direction: Vec3) -> Result<(Vec3, Vec3, Vec3), BrepError> {
    if lines.len() != 2 {
        return Err(BrepError::Unsupported("loft: the second profile must contain exactly two straight lines for this step".into()));
    }
    let first = sketch.lines.get(&lines[0].0).ok_or_else(|| BrepError::LostReference(format!("loft: line {} disappeared", lines[0].0)))?;
    let second = sketch.lines.get(&lines[1].0).ok_or_else(|| BrepError::LostReference(format!("loft: line {} disappeared", lines[1].0)))?;
    let ends_a = [&first.from, &first.to].map(|n| sketch.canonical_point(n));
    let ends_b = [&second.from, &second.to].map(|n| sketch.canonical_point(n));
    let shared: Vec<&str> = ends_a.iter().copied().filter(|p| ends_b.contains(p)).collect();
    if shared.len() != 1 {
        return Err(BrepError::Sketch("loft: the two lines of the second profile must share exactly one endpoint".into()));
    }
    let vertex_name = shared[0];
    let outer_a =
        *ends_a.iter().find(|&&p| p != vertex_name).ok_or_else(|| BrepError::Sketch("loft: the first line has no outer endpoint".into()))?;
    let outer_b =
        *ends_b.iter().find(|&&p| p != vertex_name).ok_or_else(|| BrepError::Sketch("loft: the second line has no outer endpoint".into()))?;
    let point = |name| profile_point(sketch, "second", name);
    let (mut a, mut b, vertex) = (point(outer_a)?, point(outer_b)?, point(vertex_name)?);
    let midpoint_offset = (vertex - midpoint).dot(direction);
    let size = [midpoint, vertex, a, b].iter().flat_map(|p| p.to_array()).fold(1.0_f64, |m, x| m.max(x.abs()));
    if !within_rounding(midpoint_offset, size) {
        return Err(BrepError::Degenerate("loft: the second profile's shared vertex must lie above the midpoint of the first line".into()));
    }
    let along_a = (a - midpoint).dot(direction);
    let along_b = (b - midpoint).dot(direction);
    if along_a == 0.0 || along_b == 0.0 || along_a.signum() == along_b.signum() {
        return Err(BrepError::Degenerate(
            "loft: the two outer endpoints of the second profile must lie on opposite sides of the first line's midpoint".into(),
        ));
    }
    if along_a > along_b {
        std::mem::swap(&mut a, &mut b);
    }
    Ok((a, vertex, b))
}

fn profile_line(sketch: &Placed, which: &str) -> Result<(Vec3, Vec3), BrepError> {
    let lines = profile_lines(sketch, which)?;
    match lines.as_slice() {
        [(_, a, b)] => Ok((*a, *b)),
        [] => Err(BrepError::Sketch(format!("loft: the {which} sketch has no profile line"))),
        _ => Err(BrepError::Unsupported(format!("loft: the {which} profile must be one straight line"))),
    }
}

fn profile_lines(sketch: &Placed, which: &str) -> Result<Vec<(String, Vec3, Vec3)>, BrepError> {
    if sketch.developed.is_some() {
        return Err(BrepError::Unsupported(format!("loft: the {which} profile must lie on a plane")));
    }
    if sketch.arcs.values().any(|arc| !arc.construction) || sketch.circles.values().any(|circle| !circle.construction) || !sketch.ellipses.is_empty()
    {
        return Err(BrepError::Unsupported(format!("loft: the {which} profile must contain only straight lines")));
    }
    let point = |name: &str| profile_point(sketch, which, name);
    sketch.lines.iter().filter(|(_, line)| !line.construction).map(|(name, line)| Ok((name.clone(), point(&line.from)?, point(&line.to)?))).collect()
}

fn profile_point(sketch: &Placed, which: &str, name: &str) -> Result<Vec3, BrepError> {
    let p =
        sketch.solved.points.get(name).copied().ok_or_else(|| BrepError::LostReference(format!("loft: the {which} profile lost point {name}")))?;
    Ok(sketch.frame[0] + sketch.frame[1] * p.x + sketch.frame[2] * p.y)
}

fn thickness_sides(side: Side, thickness: f64) -> (f64, f64) {
    match side {
        Side::Left => (0.0, thickness),
        Side::Right => (-thickness, 0.0),
        Side::Middle => (-0.5 * thickness, 0.5 * thickness),
    }
}

fn vertex_id(feature: TopoId, index: u64) -> TopoId {
    TopoId::derive("loft.vertex").id(feature).index(index).finish()
}

fn face_id(feature: TopoId, index: u64) -> TopoId {
    TopoId::derive("loft.face").id(feature).index(index).finish()
}

fn planar_face(id: TopoId, loop_ids: Vec<TopoId>, points: Vec<Vec3>) -> Result<FaceSpec, BrepError> {
    if loop_ids.len() != points.len() || points.len() < 3 {
        return Err(BrepError::Degenerate(format!("loft: face {id:?} has an invalid boundary")));
    }
    let normal = (0..points.len()).fold(Vec3::ZERO, |sum, i| sum + points[i].cross(points[(i + 1) % points.len()]));
    let length_squared = normal.dot(normal);
    if !(length_squared > 0.0 && length_squared.is_finite()) {
        return Err(BrepError::Degenerate(format!("loft: face {id:?} has no planar area")));
    }
    let normal = normal * (1.0 / length_squared.sqrt());
    Ok(FaceSpec { id, surface: Surface::Plane(Plane { origin: points[0], normal }), loops: vec![loop_ids] })
}

fn unit(vector: Vec3) -> Option<Vec3> {
    let squared = vector.dot(vector);
    (squared > 0.0 && squared.is_finite()).then(|| vector * (1.0 / squared.sqrt()))
}

fn length(vector: Vec3) -> f64 {
    vector.dot(vector).sqrt()
}

/// One bend's boundary on the profile surface. Offset skins reuse these stations and planar
/// displacements: every outline edge comes from one cut through the sheet's thickness.
struct BendOutline {
    start: f64,
    end: f64,
    middle_a: Option<Vec3>,
    wing_a: Option<Vec3>,
    middle_b: Option<Vec3>,
    wing_b: Option<Vec3>,
}

fn shared_bend_outline(bend: Bend, end: f64, a: Vec3, a_other: Vec3, b: Vec3, apex: Vec3) -> Result<BendOutline, BrepError> {
    let [middle_edge, wing_edge] = [a_other - a, b - a];
    let (_, middle_station) = tangent_crossing(bend.middle_tangent, bend.axis, a, middle_edge)?;
    let (_, wing_station) = tangent_crossing(bend.wing_tangent, bend.axis, a, wing_edge)?;
    let start = middle_station.max(wing_station);
    let bad_trim = || BrepError::Degenerate("loft: a bend does not fit between the profile edges".into());
    if !(start >= 0.0 && start < end && end < bend.length) {
        return Err(bad_trim());
    }
    let cut = |station: f64, from: Vec3, to: Vec3| {
        let delta = to - from;
        let t = (station - (from - a).dot(bend.axis)) / delta.dot(bend.axis);
        if t > 0.0 && t < 1.0 { Ok(from + delta * t) } else { Err(bad_trim()) }
    };
    let middle_a = (start != middle_station).then(|| cut(start, a, a_other)).transpose()?;
    let wing_a = (start != wing_station).then(|| cut(start, a, b)).transpose()?;
    let wing_b = cut(end, b, apex)?;
    if middle_a.is_some_and(|p| (p - bend.middle_tangent).dot(bend.middle_in) <= 0.0)
        || wing_a.is_some_and(|p| (p - bend.wing_tangent).dot(bend.wing_in) <= 0.0)
        || (wing_b - bend.wing_tangent).dot(bend.wing_in) <= 0.0
    {
        return Err(bad_trim());
    }
    Ok(BendOutline { start, end, middle_a, wing_a, middle_b: None, wing_b: Some(wing_b) })
}

fn tangent_crossing(tangent: Vec3, axis: Vec3, start: Vec3, edge: Vec3) -> Result<(Vec3, f64), BrepError> {
    tangent_crossing_at_scale(tangent, axis, start, edge, 0.0)
}

fn tangent_crossing_at_scale(tangent: Vec3, axis: Vec3, start: Vec3, edge: Vec3, scale: f64) -> Result<(Vec3, f64), BrepError> {
    let (point, station, fraction) = line_meeting_at_scale(tangent, axis, start, edge, scale)?;
    if !(0.0 < fraction && fraction < 1.0) {
        return Err(BrepError::Degenerate("loft: a tangent does not cross its profile edge".into()));
    }
    Ok((point, station))
}

// Cut the cylinder square at the nearer of the two boundary crossings at each end. If the
// other facet runs farther along the axis, its remaining tangent segment is on the outline.
fn separate_bend_outline(bend: Bend, a: [Vec3; 2], b: [Vec3; 2], outer: Vec3) -> Result<BendOutline, BrepError> {
    let crossing = |tangent, from, to| tangent_crossing(tangent, bend.axis, from, to - from);
    let [a0, a1] = a;
    let [b0, b1] = b;
    let (middle_a, ma) = crossing(bend.middle_tangent, a0, a1)?;
    let (wing_a, wa) = crossing(bend.wing_tangent, a0, outer)?;
    let (middle_b, mb) = crossing(bend.middle_tangent, b0, b1)?;
    let (wing_b, wb) = crossing(bend.wing_tangent, b0, outer)?;
    let (start, end) = (ma.max(wa), mb.min(wb));
    if start >= end {
        return Err(BrepError::Degenerate("loft: a bend does not fit between the profile edges".into()));
    }
    Ok(BendOutline {
        start,
        end,
        middle_a: (start != ma).then_some(middle_a),
        wing_a: (start != wa).then_some(wing_a),
        middle_b: (end != mb).then_some(middle_b),
        wing_b: (end != wb).then_some(wing_b),
    })
}

#[derive(Clone, Copy)]
struct WingVertices {
    middle_a: usize,
    middle_tangent: usize,
    wing_tangent_a: usize,
    wing_a: usize,
    outer: usize,
    middle_b: usize,
    middle_tangent_b: usize,
    wing_b: usize,
    wing_tangent_b: usize,
}

struct LoftShell {
    feature: TopoId,
    vertices: Vec<Vertex>,
    faces: Vec<FaceSpec>,
    curves: Vec<CurveSpec>,
}

impl LoftShell {
    fn vertex(&mut self, skin: &str, facet: &str, role: &str, point: Vec3) -> usize {
        let id = TopoId::derive("loft.bent.vertex").id(self.feature).tag(skin).tag(facet).tag(role).finish();
        let index = self.vertices.len();
        self.vertices.push(Vertex { id, point });
        index
    }

    fn point(&self, id: usize) -> Vec3 {
        self.vertices[id].point
    }

    fn face(&mut self, side: &str, role: &str, surface: Surface, mut ids: Vec<usize>) {
        ids.dedup();
        if ids.first() == ids.last() {
            ids.pop();
        }
        let origin = self.point(ids[0]);
        let normal = ids[1..].windows(2).fold(Vec3::ZERO, |area, pair| area + (self.point(pair[0]) - origin).cross(self.point(pair[1]) - origin));
        if normal.dot(surface.normal_at(origin)) < 0.0 {
            ids.reverse();
        }
        let id = TopoId::derive("loft.bent.face").id(self.feature).tag(side).tag(role).finish();
        self.faces.push(FaceSpec { id, surface, loops: vec![ids.into_iter().map(|v| self.vertices[v].id).collect()] });
    }

    fn plane(&mut self, side: &str, role: &str, normal: Vec3, ids: Vec<usize>) {
        let origin = self.point(ids[0]);
        self.face(side, role, Surface::Plane(Plane { origin, normal }), ids);
    }

    fn wall(&mut self, side: &str, role: &str, ids: [usize; 4], inside: usize) -> Result<(), BrepError> {
        let origin = self.point(ids[0]);
        let across = self.point(ids[3]) - origin;
        let along = self.point(ids[1]) - origin;
        let mut normal = unit(along.cross(across)).ok_or_else(|| BrepError::Degenerate("loft: an outline wall has no area".into()))?;
        if normal.dot(self.point(inside) - origin) > 0.0 {
            normal = -normal;
        }
        self.plane(side, role, normal, ids.to_vec());
        Ok(())
    }

    fn arc(&mut self, from: usize, to: usize, center: Vec3, axis: Vec3, radius: f64) {
        let axis = if (self.point(from) - center).cross(self.point(to) - center).dot(axis) < 0.0 { -axis } else { axis };
        self.curves.push(CurveSpec { from: self.vertices[from].id, to: self.vertices[to].id, curve: Curve::Arc(Arc { center, axis, radius }) });
    }

    fn build(self) -> Result<Body, BrepError> {
        let mut uses = BTreeMap::<_, (usize, i32)>::new();
        for face in &self.faces {
            let ids = &face.loops[0];
            for (&a, &b) in ids.iter().zip(ids.iter().cycle().skip(1)) {
                let (count, direction) = uses.entry((a.min(b), a.max(b))).or_default();
                *count += 1;
                *direction += if a < b { 1 } else { -1 };
            }
        }
        if let Some((edge, (count, _))) = uses.iter().find(|(_, (count, _))| *count != 2) {
            return Err(BrepError::NonManifold(format!("loft: edge {edge:?} has {count} incident faces")));
        }
        if let Some((edge, _)) = uses.iter().find(|(_, (_, direction))| *direction != 0) {
            return Err(BrepError::NonManifold(format!("loft: faces traverse edge {edge:?} in the same direction")));
        }
        build_with_curves(self.vertices, self.faces, self.curves)
    }
}

fn two_bend_body(feature: TopoId, a: [Vec3; 2], profile: BendProfile, rule: BendRule) -> Result<Body, BrepError> {
    let [a0, a1] = a;
    let (b, ends, shared) = match profile {
        BendProfile::Shared { corner, ends } => ([corner; 2], ends, true),
        BendProfile::Separate { corners, ends } => (corners, ends, false),
    };
    let [b0, b1] = b;
    let [left_end, right_end] = ends;
    let normal = |p: Vec3, q: Vec3, r: Vec3| unit((q - p).cross(r - p)).ok_or_else(|| BrepError::Degenerate("loft: a facet has no area".into()));
    let middle_normal = normal(a0, a1, b0)?;
    let wing_normals = [normal(a0, b0, left_end)?, normal(a1, right_end, b1)?];
    // The exact side of the wing's third point also gives the side of the bend centre.
    let radial_sign = |end| match orient3d_coords(a0, a1, b0, end) {
        Sign::Positive => Ok(1.0),
        Sign::Negative => Ok(-1.0),
        Sign::Zero => Err(BrepError::Degenerate("loft: a bend has no angle".into())),
    };
    let bends = [bend(a0, b0, [a1, left_end], radial_sign(left_end)?, rule)?, bend(a1, b1, [a0, right_end], radial_sign(right_end)?, rule)?];
    let [left, right] = bends;
    let (corner, outlines) = if shared {
        let (corner, end0, end1) = line_meeting(left.middle_tangent, left.axis, right.middle_tangent, right.axis)?;
        (Some(corner), [shared_bend_outline(left, end0, a0, a1, left_end, b0)?, shared_bend_outline(right, end1, a1, a0, right_end, b1)?])
    } else {
        (None, [separate_bend_outline(left, a, b, left_end)?, separate_bend_outline(right, [a1, a0], [b1, b0], right_end)?])
    };
    let along_a = a1 - a0;
    let [a_left, a_right] = [0, 1].map(|i| outlines[i].middle_a.unwrap_or(bends[i].middle_tangent + bends[i].axis * outlines[i].start));
    if (a_right - a_left).dot(along_a) <= 0.0 {
        return Err(BrepError::Degenerate("loft: the two bends consume profile A".into()));
    }
    if !shared {
        let [b_left, b_right] = [0, 1].map(|i| outlines[i].middle_b.unwrap_or(bends[i].middle_tangent + bends[i].axis * outlines[i].end));
        if (b_right - b_left).dot(along_a) <= 0.0 {
            return Err(BrepError::Degenerate("loft: the two bends consume the middle of profile B".into()));
        }
    }

    let (low, high) = thickness_sides(rule.side, rule.thickness);
    let wing_roles = ["first", "second"];
    let mut shell = LoftShell { feature, vertices: Vec::with_capacity(36), faces: Vec::with_capacity(22), curves: Vec::with_capacity(8) };
    let [lower, upper] = [("lower", low, -1.0), ("upper", high, 1.0)].map(|(skin, offset, outward)| {
        let middle_shift = middle_normal * offset;
        let shared_b = corner.map(|p| shell.vertex(skin, "middle", "B", p + middle_shift));
        let wings = [0, 1].map(|i| {
            let role = wing_roles[i];
            let bend = bends[i];
            let outline = &outlines[i];
            let wing_shift = wing_normals[i] * offset;
            let mut vertex = |name, point| shell.vertex(skin, role, name, point);
            let middle_tangent = vertex("middle_tangent_A", bend.middle_tangent + bend.axis * outline.start + middle_shift);
            let wing_tangent_a = vertex("wing_tangent_A", bend.wing_tangent + bend.axis * outline.start + wing_shift);
            let wing_tangent_b = vertex("wing_tangent_B", bend.wing_tangent + bend.axis * outline.end + wing_shift);
            let middle_tangent_b =
                shared_b.unwrap_or_else(|| vertex("middle_tangent_B", bend.middle_tangent + bend.axis * outline.end + middle_shift));
            let middle_a = outline.middle_a.map_or(middle_tangent, |p| vertex("middle_outline_A", p + middle_shift));
            let wing_a = outline.wing_a.map_or(wing_tangent_a, |p| vertex("wing_outline_A", p + wing_shift));
            let outer = vertex("outer_B", ends[i] + wing_shift);
            let middle_b = outline.middle_b.map_or(middle_tangent_b, |p| vertex("middle_outline_B", p + middle_shift));
            let wing_b = outline.wing_b.map_or(wing_tangent_b, |p| vertex("wing_outline_B", p + wing_shift));

            shell.plane(skin, role, wing_normals[i] * outward, vec![wing_tangent_a, wing_a, outer, wing_b, wing_tangent_b]);
            let convex = outward == bend.radial_sign;
            let radius = rule.radius + if convex { rule.thickness } else { 0.0 };
            shell.face(
                skin,
                if i == 0 { "first_bend" } else { "second_bend" },
                Surface::Cylinder(Cylinder { origin: bend.center, axis: bend.axis, radius, convex }),
                vec![middle_tangent, middle_tangent_b, wing_tangent_b, wing_tangent_a],
            );
            shell.arc(middle_tangent, wing_tangent_a, bend.center + bend.axis * outline.start, bend.axis, radius);
            shell.arc(middle_tangent_b, wing_tangent_b, bend.center + bend.axis * outline.end, bend.axis, radius);
            WingVertices { middle_a, middle_tangent, wing_tangent_a, wing_a, outer, middle_b, middle_tangent_b, wing_b, wing_tangent_b }
        });
        let [l, r] = wings;
        shell.plane(
            skin,
            "middle",
            middle_normal * outward,
            vec![l.middle_a, r.middle_a, r.middle_tangent, r.middle_tangent_b, r.middle_b, l.middle_b, l.middle_tangent_b, l.middle_tangent],
        );
        wings
    });

    let [l, r] = lower;
    let [u, v] = upper;
    shell.wall("outline", "A", [l.middle_a, r.middle_a, v.middle_a, u.middle_a], l.middle_tangent_b)?;
    if !shared {
        shell.wall("outline", "B", [l.middle_b, r.middle_b, v.middle_b, u.middle_b], l.middle_tangent)?;
    }
    for i in 0..2 {
        let role = wing_roles[i];
        let (l, u) = (lower[i], upper[i]);
        let axis = bends[i].axis;
        shell.wall(role, "side", [l.wing_a, l.outer, u.outer, u.wing_a], l.wing_b)?;
        shell.wall(role, "B", [l.outer, l.wing_b, u.wing_b, u.outer], l.wing_a)?;
        if shared {
            shell.plane(
                role,
                "A_end",
                -axis,
                vec![l.middle_a, l.middle_tangent, l.wing_tangent_a, l.wing_a, u.wing_a, u.wing_tangent_a, u.middle_tangent, u.middle_a],
            );
            shell.plane(role, "B_end", axis, vec![l.middle_tangent_b, l.wing_tangent_b, l.wing_b, u.wing_b, u.wing_tangent_b, u.middle_tangent_b]);
        } else {
            shell.plane(role, "A_end", -axis, vec![l.middle_tangent, l.wing_tangent_a, u.wing_tangent_a, u.middle_tangent]);
            shell.plane(role, "B_end", axis, vec![l.middle_tangent_b, l.wing_tangent_b, u.wing_tangent_b, u.middle_tangent_b]);
            // The steps join the square caps to the straight profile edges along the tangents.
            for (name, edge, inside) in [
                ("middle_A_step", [l.middle_a, l.middle_tangent, u.middle_tangent, u.middle_a], lower[1 - i].middle_a),
                ("middle_B_step", [l.middle_tangent_b, l.middle_b, u.middle_b, u.middle_tangent_b], lower[1 - i].middle_b),
                ("wing_A_step", [l.wing_tangent_a, l.wing_a, u.wing_a, u.wing_tangent_a], l.outer),
                ("wing_B_step", [l.wing_b, l.wing_tangent_b, u.wing_tangent_b, u.wing_b], l.outer),
            ] {
                if edge[0] != edge[1] {
                    shell.wall(role, name, edge, inside)?;
                }
            }
        }
    }
    shell.build()
}

/// The square ends of a bend between two successive chord facets.
struct TaperBend {
    bend: Bend,
    ends: [f64; 2],
}

fn tapered_arc_body(feature: TopoId, source: &str, a: &[Vec3], b: &[Vec3], sheet: Sheet, reversed: bool) -> Result<Body, BrepError> {
    let panels = a.len() - 1;
    let factor = if reversed { -1.0 } else { 1.0 };
    let bad_facet = || BrepError::Degenerate("loft: a chord facet has no room between the bends".into());
    let normals: Vec<_> = (0..panels).map(|i| unit((b[i] - a[i]).cross(a[i + 1] - a[i]) * factor).ok_or_else(bad_facet)).collect::<Result<_, _>>()?;
    let rule = BendRule { thickness: sheet.thickness, radius: sheet.radius, side: sheet.side };
    let mut bends = Vec::with_capacity(panels - 1);
    for i in 1..panels {
        let sign = match orient3d_coords(a[i - 1], b[i - 1], a[i], a[i + 1]) {
            Sign::Positive => factor,
            Sign::Negative => -factor,
            Sign::Zero => return Err(bad_facet()),
        };
        let bend = bend(a[i], b[i], [a[i - 1], a[i + 1]], sign, rule)?;
        let cap = |p: &[Vec3]| -> Result<f64, BrepError> {
            let (_, left) = tangent_crossing(bend.middle_tangent, bend.axis, p[i], p[i - 1] - p[i])?;
            let (_, right) = tangent_crossing(bend.wing_tangent, bend.axis, p[i], p[i + 1] - p[i])?;
            // Matching angular increments make the two stations equal analytically. The two
            // constructions may differ by rounding in the sampled chord endpoints.
            let size = bend.length.max(length(p[i] - p[i - 1])).max(length(p[i + 1] - p[i]));
            if !within_rounding(left - right, size) {
                return Err(BrepError::Degenerate("loft: the two sides of an arc bend do not meet its profile at one station".into()));
            }
            Ok(left)
        };
        let (start, end) = (cap(a)?, cap(b)?);
        if start >= end {
            return Err(bad_facet());
        }
        bends.push(TaperBend { bend, ends: [start, end] });
    }

    let last = a.len() - 1;
    let (low, high) = thickness_sides(sheet.side, sheet.thickness);
    let mut shell = LoftShell {
        feature,
        vertices: Vec::with_capacity(8 * panels),
        faces: Vec::with_capacity(8 * panels),
        curves: Vec::with_capacity(4 * bends.len()),
    };
    let [lower, upper] = [("lower", low, -1.0), ("upper", high, 1.0)].map(|(skin, offset, outward)| {
        let names = |i| format!("{source}.chord.{i}/{panels}");
        let mut sides = Vec::with_capacity(panels);
        let mut left =
            [shell.vertex(skin, source, "A_start", a[0] + normals[0] * offset), shell.vertex(skin, source, "B_start", b[0] + normals[0] * offset)];
        for (i, TaperBend { bend, ends }) in bends.iter().enumerate() {
            let name = names(i);
            let pair = |shell: &mut LoftShell, origin: Vec3, normal: Vec3, roles: [&str; 2]| {
                [0, 1].map(|j| shell.vertex(skin, &name, roles[j], origin + bend.axis * ends[j] + normal * offset))
            };
            let entry = pair(&mut shell, bend.middle_tangent, normals[i], ["A_entry", "B_entry"]);
            let exit = pair(&mut shell, bend.wing_tangent, normals[i + 1], ["A_exit", "B_exit"]);
            let convex = outward == bend.radial_sign;
            let radius = sheet.radius + if convex { sheet.thickness } else { 0.0 };
            shell.face(
                skin,
                &format!("{name}.bend"),
                Surface::Cylinder(Cylinder { origin: bend.center, axis: bend.axis, radius, convex }),
                vec![entry[0], entry[1], exit[1], exit[0]],
            );
            for j in 0..2 {
                shell.arc(entry[j], exit[j], bend.center + bend.axis * ends[j], bend.axis, radius);
            }
            sides.push([left, entry]);
            left = exit;
        }
        sides.push([
            left,
            [
                shell.vertex(skin, source, "A_end", a[last] + normals[last - 1] * offset),
                shell.vertex(skin, source, "B_end", b[last] + normals[last - 1] * offset),
            ],
        ]);
        for (i, [left, right]) in sides.iter().enumerate() {
            shell.plane(skin, &names(i), normals[i] * outward, vec![left[0], right[0], right[1], left[1]]);
        }
        sides
    });

    for (i, (&[ll, lr], &[ul, ur])) in lower.iter().zip(&upper).enumerate() {
        let name = format!("{source}.chord.{i}/{panels}");
        for (j, section, chord) in [(0, "A", a), (1, "B", b)] {
            if (shell.point(lr[j]) - shell.point(ll[j])).dot(chord[i + 1] - chord[i]) <= 0.0 {
                return Err(bad_facet());
            }
            shell.wall(&name, section, [ll[j], lr[j], ur[j], ul[j]], ll[1 - j])?;
        }
        if let Some(cut) = bends.get(i) {
            let (entry, exit) = (lr, lower[i + 1][0]);
            let (entry_top, exit_top) = (ur, upper[i + 1][0]);
            for (j, (role, normal)) in [("A_end", -cut.bend.axis), ("B_end", cut.bend.axis)].into_iter().enumerate() {
                shell.plane(&name, role, normal, vec![entry[j], exit[j], exit_top[j], entry_top[j]]);
            }
        }
    }
    for (i, edge, toward, name) in [(0, 0, 1, "start"), (panels - 1, 1, 0, "end")] {
        let (lo, up) = (lower[i][edge], upper[i][edge]);
        shell.wall(source, name, [lo[0], lo[1], up[1], up[0]], lower[i][toward][0])?;
    }
    shell.build()
}
