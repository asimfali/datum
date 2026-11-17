//! A planar loft, then the first two-bend D264 case.

use datum_brep::{Body, BrepError, FaceSpec, Plane, Surface, TopoId, Vertex, build_with_curves};
use datum_math::{Sign, Vec3, orient3d_coords};

use crate::sketch::within_rounding;
use crate::{Placed, Side, parallel};

/// Builds the planar first D264 case, or validates the next three-facet/two-bend layout.
///
/// The sketch support planes need not be parallel: the solved world-space line profiles
/// themselves must be parallel and all four endpoints coplanar in the first case.
pub fn loft(
    feature: TopoId,
    first: &Placed,
    second: &Placed,
    thickness: f64,
    radius: Option<f64>,
    k_factor: Option<f64>,
    side: Side,
    tolerance: f64,
) -> Result<Body, BrepError> {
    if !(thickness > 0.0 && thickness.is_finite()) {
        return Err(BrepError::Degenerate(format!("loft: thickness must be positive and finite, got {thickness}")));
    }
    if !(tolerance > 0.0 && tolerance.is_finite()) {
        return Err(BrepError::Degenerate(format!("loft: tolerance must be positive and finite, got {tolerance}")));
    }

    let (a0, a1) = profile_line(first, "first")?;
    let second_lines = profile_lines(second, "second")?;
    match second_lines.as_slice() {
        [(_, b0, b1)] => planar_loft(feature, a0, a1, *b0, *b1, thickness, side),
        [_, _] => {
            let (Some(radius), Some(k)) = (radius, k_factor) else {
                return Err(BrepError::Invalid(
                    "loft with two lines in its second profile takes thickness, bend radius and K-factor from the part's sheet rule".into(),
                ));
            };
            let layout = two_bend_layout(a0, a1, second, &second_lines, thickness, radius, k, side)?;
            let [first_bend, second_bend] = layout.bends;
            Err(BrepError::Unsupported(format!(
                "loft: the three-facet / two-bend layout is valid (folds {:.3}° and {:.3}°, radii {:.3}/{:.3} mm, neutral radii {:.3}/{:.3} mm, common corner stations {:.3}/{:.3} mm); corner-trimmed cylindrical B-rep assembly and development are not built yet",
                first_bend.angle.to_degrees(),
                second_bend.angle.to_degrees(),
                first_bend.radius_in,
                first_bend.radius_out,
                first_bend.neutral_radius,
                second_bend.neutral_radius,
                layout.corner_station[0],
                layout.corner_station[1],
            )))
        }
        [] => Err(BrepError::Sketch("loft: the second sketch has no profile line".into())),
        _ => Err(BrepError::Unsupported("loft: the second profile must be one line or two connected lines".into())),
    }
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
struct Facet {
    points: [Vec3; 3],
    normal: Vec3,
}

#[derive(Clone, Copy)]
struct BendLayout {
    axis: Vec3,
    hinge_length: f64,
    center: Vec3,
    angle: f64,
    radius_in: f64,
    radius_out: f64,
    neutral_radius: f64,
    tangent_offset: f64,
    middle_tangent: Vec3,
}

struct TwoBendLayout {
    bends: [BendLayout; 2],
    corner_station: [f64; 2],
}

/// Geometric part of D264's one-line / two-line case. The corner is where the two tangent
/// lines in the middle facet meet; both bends must terminate there, rather than overlap caps.
fn two_bend_layout(
    a0: Vec3,
    a1: Vec3,
    second: &Placed,
    lines: &[(String, Vec3, Vec3)],
    thickness: f64,
    radius: f64,
    k_factor: f64,
    side: Side,
) -> Result<TwoBendLayout, BrepError> {
    if !(radius > 0.0 && radius.is_finite()) {
        return Err(BrepError::Degenerate(format!("loft: bend radius must be positive and finite, got {radius}")));
    }
    if !(0.0..=1.0).contains(&k_factor) {
        return Err(BrepError::Degenerate(format!("loft: K-factor must lie between 0 and 1, got {k_factor}")));
    }
    let direction = unit(a1 - a0).ok_or_else(|| BrepError::Degenerate("loft: the first profile line has zero length".into()))?;
    let midpoint = (a0 + a1) * 0.5;
    let (b0, vertex, b1) = second_profile_corner(second, lines, midpoint, direction)?;
    let base_normal = unit((a1 - a0).cross(vertex - midpoint))
        .ok_or_else(|| BrepError::Degenerate("loft: the second profile's vertex lies on the first profile line".into()))?;

    let mut facets = [
        Facet { points: [a0, b0, vertex], normal: Vec3::ZERO },
        Facet { points: [a0, a1, vertex], normal: Vec3::ZERO },
        Facet { points: [a1, vertex, b1], normal: Vec3::ZERO },
    ];
    for (i, facet) in facets.iter_mut().enumerate() {
        let cross = (facet.points[1] - facet.points[0]).cross(facet.points[2] - facet.points[0]);
        facet.normal = unit(cross).ok_or_else(|| BrepError::Degenerate(format!("loft: triangular facet {i} has no area")))?;
        if facet.normal.dot(base_normal) < 0.0 {
            facet.normal = -facet.normal;
        }
    }

    // The profile is the outer face for Left, inner face for Right, and the middle for Middle.
    let reference_radius = match side {
        Side::Left => radius + thickness,
        Side::Right => radius,
        Side::Middle => radius + 0.5 * thickness,
    };
    let bend_1 = bend_layout(
        [a0, vertex],
        facets[0],
        facets[1],
        radius,
        thickness,
        reference_radius,
        radius + k_factor * thickness,
    )?;
    let bend_2 = bend_layout(
        [a1, vertex],
        facets[2],
        facets[1],
        radius,
        thickness,
        reference_radius,
        radius + k_factor * thickness,
    )?;

    // The two tangent lines on the middle triangle must meet once, inside both hinge segments.
    let (corner, first_station, second_station) =
        line_meeting(bend_1.middle_tangent, bend_1.axis, bend_2.middle_tangent, bend_2.axis)?;
    if !(first_station > 0.0
        && first_station < bend_1.hinge_length
        && second_station > 0.0
        && second_station < bend_2.hinge_length)
    {
        return Err(BrepError::Degenerate(format!(
            "loft: the two bend tangent lines meet outside their shared middle facet (stations {first_station:.6}, {second_station:.6})"
        )));
    }
    let first_meeting = bend_1.middle_tangent + bend_1.axis * first_station;
    let second_meeting = bend_2.middle_tangent + bend_2.axis * second_station;
    let size = [a0, a1, b0, vertex, b1].iter().flat_map(|p| p.to_array()).fold(1.0_f64, |m, x| m.max(x.abs()));
    if !within_rounding(length(corner - first_meeting), size) || !within_rounding(length(corner - second_meeting), size) {
        return Err(BrepError::Invalid("loft: the two bend tangents do not share one corner".into()));
    }

    Ok(TwoBendLayout { bends: [bend_1, bend_2], corner_station: [first_station, second_station] })
}

/// The sharp shared edge is the bend's axis; first and second are the facets on either side.
fn bend_layout(
    hinge: [Vec3; 2],
    first: Facet,
    second: Facet,
    radius: f64,
    thickness: f64,
    reference_radius: f64,
    neutral_radius: f64,
) -> Result<BendLayout, BrepError> {
    let axis_vector = hinge[1] - hinge[0];
    let hinge_length = length(axis_vector);
    let axis = unit(axis_vector).ok_or_else(|| BrepError::Degenerate("loft: a bend has zero length".into()))?;
    let inward = |facet: Facet| -> Result<Vec3, BrepError> {
        let third = facet.points
            .iter()
            .copied()
            .find(|p| *p != hinge[0] && *p != hinge[1])
            .ok_or_else(|| BrepError::Invalid("loft: a bend's adjacent facet has no opposite vertex".into()))?;
        let offset = third - hinge[0];
        let across = offset - axis * offset.dot(axis);
        unit(across).ok_or_else(|| BrepError::Degenerate("loft: a facet has no width across its bend".into()))
    };
    let u0 = inward(first)?;
    let u1 = inward(second)?;
    let dot = u0.dot(u1).clamp(-1.0, 1.0);
    let interior = libm::acos(dot);
    let angle = std::f64::consts::PI - interior;
    if !(angle > 0.0 && angle < std::f64::consts::PI && angle.is_finite()) {
        return Err(BrepError::Degenerate(format!("loft: a bend angle is invalid, got {angle} rad")));
    }
    let bisector = unit(u0 + u1).ok_or_else(|| BrepError::Degenerate("loft: the two facets have no unique bend bisector".into()))?;
    let cos = libm::cos(0.5 * angle);
    if !(cos > 0.0) {
        return Err(BrepError::Degenerate("loft: a bend angle has no finite tangent offset".into()));
    }
    let tangent_offset = reference_radius * libm::tan(0.5 * angle);
    let center_offset = reference_radius / cos;
    if !(tangent_offset > 0.0 && tangent_offset.is_finite() && center_offset.is_finite()) {
        return Err(BrepError::Degenerate("loft: a bend's tangent offset is not finite".into()));
    }

    // The radius must fit on both facets before the tangent lines reach their opposite edges.
    for (index, facet) in [first, second].into_iter().enumerate() {
        let third = facet.points
            .iter()
            .copied()
            .find(|p| *p != hinge[0] && *p != hinge[1])
            .ok_or_else(|| BrepError::Invalid("loft: a bend's adjacent facet has no opposite vertex".into()))?;
        let offset = third - hinge[0];
        let height = (offset - axis * offset.dot(axis)).dot(if index == 0 { u0 } else { u1 });
        if !(height > tangent_offset) {
            return Err(BrepError::Degenerate(format!(
                "loft: bend radius {radius} mm consumes an adjacent facet (available width {height:.6}, tangent offset {tangent_offset:.6})"
            )));
        }
    }

    let center = hinge[0] + bisector * center_offset;
    let tangent_0 = hinge[0] + u0 * tangent_offset;
    let tangent_1 = hinge[0] + u1 * tangent_offset;
    let radial_0 = unit(tangent_0 - center).ok_or_else(|| BrepError::Degenerate("loft: a bend tangent has no radius".into()))?;
    let radial_1 = unit(tangent_1 - center).ok_or_else(|| BrepError::Degenerate("loft: a bend tangent has no radius".into()))?;
    let measured_0 = length(tangent_0 - center);
    let measured_1 = length(tangent_1 - center);
    if !within_rounding(measured_0 - reference_radius, reference_radius)
        || !within_rounding(measured_1 - reference_radius, reference_radius)
    {
        return Err(BrepError::Invalid("loft: tangent points do not lie on the reference bend cylinder".into()));
    }
    let sweep = libm::atan2(axis.dot(radial_0.cross(radial_1)), radial_0.dot(radial_1)).abs();
    if !within_rounding(sweep - angle, 1.0) {
        return Err(BrepError::Invalid(format!("loft: bend sweep {sweep} does not match its facet angle {angle}")));
    }
    Ok(BendLayout {
        axis,
        hinge_length,
        center,
        angle,
        radius_in: radius,
        radius_out: radius + thickness,
        neutral_radius,
        tangent_offset,
        middle_tangent: tangent_1,
    })
}

/// The two tangent lines, both contained by the middle facet, meet at p + d*s.
fn line_meeting(p: Vec3, d: Vec3, q: Vec3, e: Vec3) -> Result<(Vec3, f64, f64), BrepError> {
    let n = d.cross(e);
    let n2 = n.dot(n);
    if !(n2 > 0.0 && n2.is_finite()) {
        return Err(BrepError::Degenerate("loft: the two bend axes do not define a unique corner relief".into()));
    }
    let delta = q - p;
    let s = delta.cross(e).dot(n) / n2;
    let t = delta.cross(d).dot(n) / n2;
    let a = p + d * s;
    let b = q + e * t;
    let size = [p, q, a, b].iter().flat_map(|x| x.to_array()).fold(1.0_f64, |m, x| m.max(x.abs()));
    if !within_rounding(length(a - b), size) {
        return Err(BrepError::Degenerate("loft: the two bend tangent lines in the middle facet are skew".into()));
    }
    Ok((a, s, t))
}

fn second_profile_corner(
    sketch: &Placed,
    lines: &[(String, Vec3, Vec3)],
    midpoint: Vec3,
    direction: Vec3,
) -> Result<(Vec3, Vec3, Vec3), BrepError> {
    if lines.len() != 2 {
        return Err(BrepError::Unsupported("loft: the second profile must contain exactly two straight lines for this step".into()));
    }
    let first = sketch.lines.get(&lines[0].0).ok_or_else(|| BrepError::LostReference(format!("loft: line {} disappeared", lines[0].0)))?;
    let second = sketch.lines.get(&lines[1].0).ok_or_else(|| BrepError::LostReference(format!("loft: line {} disappeared", lines[1].0)))?;
    let ends_a = [first.from.as_str(), first.to.as_str()];
    let ends_b = [second.from.as_str(), second.to.as_str()];
    let shared: Vec<&str> = ends_a.iter().copied().filter(|p| ends_b.contains(p)).collect();
    if shared.len() != 1 {
        return Err(BrepError::Sketch("loft: the two lines of the second profile must share exactly one endpoint".into()));
    }
    let vertex_name = shared[0];
    let outer_a = *ends_a.iter().find(|&&p| p != vertex_name).ok_or_else(|| BrepError::Sketch("loft: the first line has no outer endpoint".into()))?;
    let outer_b = *ends_b.iter().find(|&&p| p != vertex_name).ok_or_else(|| BrepError::Sketch("loft: the second line has no outer endpoint".into()))?;
    let world = |name: &str| -> Result<Vec3, BrepError> {
        let p = sketch.solved.points.get(name).copied().ok_or_else(|| BrepError::LostReference(format!("loft: the second profile lost point {}", name)))?;
        Ok(sketch.frame[0] + sketch.frame[1] * p.x + sketch.frame[2] * p.y)
    };
    let (mut a, mut b, vertex) = (world(outer_a)?, world(outer_b)?, world(vertex_name)?);
    let midpoint_offset = (vertex - midpoint).dot(direction);
    let size = [midpoint, vertex, a, b].iter().flat_map(|p| p.to_array()).fold(1.0_f64, |m, x| m.max(x.abs()));
    if !within_rounding(midpoint_offset, size) {
        return Err(BrepError::Degenerate("loft: the second profile's shared vertex must lie above the midpoint of the first line".into()));
    }
    let along_a = (a - midpoint).dot(direction);
    let along_b = (b - midpoint).dot(direction);
    if along_a == 0.0 || along_b == 0.0 || along_a.signum() == along_b.signum() {
        return Err(BrepError::Degenerate("loft: the two outer endpoints of the second profile must lie on opposite sides of the first line's midpoint".into()));
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
    if sketch.arcs.values().any(|arc| !arc.construction)
        || sketch.circles.values().any(|circle| !circle.construction)
        || !sketch.ellipses.is_empty()
    {
        return Err(BrepError::Unsupported(format!("loft: the {which} profile must contain only straight lines")));
    }
    let world = |name: &str| -> Result<Vec3, BrepError> {
        let p = sketch.solved.points.get(name).copied().ok_or_else(|| BrepError::LostReference(format!("loft: the {which} profile lost point {}", name)))?;
        Ok(sketch.frame[0] + sketch.frame[1] * p.x + sketch.frame[2] * p.y)
    };
    sketch.lines.iter().filter(|(_, line)| !line.construction)
        .map(|(name, line)| Ok((name.clone(), world(&line.from)?, world(&line.to)?)))
        .collect()
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