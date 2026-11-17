//! A planar loft from two parallel line profiles (D264, first step).

use datum_brep::{build_with_curves, Body, BrepError, FaceSpec, Plane, Surface, TopoId, Vertex};
use datum_math::{orient3d_coords, Sign, Vec3};

use crate::{parallel, Placed, Side};

/// Builds the first D264 case: two single-line sketches make one planar trapezoid.
///
/// The sketch support planes need not be parallel: the solved world-space line profiles
/// themselves must be parallel and their four endpoints coplanar. This covers the author's
/// XY/XZ test case, whose sketch planes are perpendicular but whose lines lie in one plane.
pub fn loft(
    feature: TopoId,
    first: &Placed,
    second: &Placed,
    thickness: f64,
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
    let (mut b0, mut b1) = profile_line(second, "second")?;
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
    let (low, high) = match side {
        Side::Left => (0.0, thickness),
        Side::Right => (-thickness, 0.0),
        Side::Middle => (-0.5 * thickness, 0.5 * thickness),
    };

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
    faces.push(planar_face(
        face_id(feature, 0),
        outer_ids.to_vec(),
        outer.to_vec(),
    )?);
    faces.push(planar_face(
        face_id(feature, 1),
        (0..4).rev().map(|i| inner_ids[i]).collect(),
        (0..4).rev().map(|i| inner[i]).collect(),
    )?);

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

fn profile_line(sketch: &Placed, which: &str) -> Result<(Vec3, Vec3), BrepError> {
    if sketch.developed.is_some() {
        return Err(BrepError::Unsupported(format!("loft: the {which} profile must lie on a plane")));
    }
    if sketch.arcs.values().any(|arc| !arc.construction)
        || sketch.circles.values().any(|circle| !circle.construction)
        || !sketch.ellipses.is_empty()
    {
        return Err(BrepError::Unsupported(format!("loft: the {which} profile must be one straight line")));
    }
    let mut lines = sketch.lines.iter().filter(|(_, line)| !line.construction);
    let Some((_, line)) = lines.next() else {
        return Err(BrepError::Sketch(format!("loft: the {which} sketch has no profile line")));
    };
    if lines.next().is_some() {
        return Err(BrepError::Unsupported(format!("loft: the {which} sketch must contain exactly one profile line")));
    }
    let world = |name: &str| {
        let point = sketch
            .solved
            .points
            .get(name)
            .copied()
            .ok_or_else(|| BrepError::LostReference(format!("loft: the {which} line lost point {name}")))?;
        Ok(sketch.frame[0] + sketch.frame[1] * point.x + sketch.frame[2] * point.y)
    };
    Ok((world(&line.from)?, world(&line.to)?))
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
    Ok(FaceSpec {
        id,
        surface: Surface::Plane(Plane { origin: points[0], normal }),
        loops: vec![loop_ids],
    })
}

fn unit(vector: Vec3) -> Option<Vec3> {
    let squared = vector.dot(vector);
    (squared > 0.0 && squared.is_finite()).then(|| vector * (1.0 / squared.sqrt()))
}
