//! Planar and two-bend sheet lofts.

use std::collections::BTreeMap;

use datum_brep::{Arc, Body, BrepError, Curve, CurveSpec, Cylinder, FaceSpec, Plane, Surface, TopoId, Vertex, build_with_curves};
use datum_math::{Sign, Vec3, orient3d_coords};

use crate::sketch::within_rounding;
use crate::{Placed, Side, parallel};

/// Parameters shared by the planar and first bent D264 loft cases.
#[derive(Clone, Copy)]
pub struct LoftDims {
    pub thickness: f64,
    pub radius: Option<f64>,
    pub k_factor: Option<f64>,
    pub side: Side,
    pub tolerance: f64,
}

/// Builds the planar or three-facet sheet loft.
///
/// In the planar case the line profiles must be parallel and coplanar in world space. For
/// the bent case the first line must lie in a plane parallel to the V profile.
pub fn loft(feature: TopoId, first: &Placed, second: &Placed, dims: LoftDims) -> Result<Body, BrepError> {
    let LoftDims { thickness, radius, k_factor, side, tolerance } = dims;
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
            let [b0, apex, b1] = two_bend_profile([a0, a1], second, &second_lines)?;
            let rule = BendRule { thickness, radius, k_factor: k, side };
            let body = two_bend_body(feature, [a0, a1], [b0, apex, b1], rule)?;
            let flat = crate::unfold_side(&body, false, k)?;
            if !flat.overlaps.is_empty() {
                return Err(BrepError::Degenerate(format!(
                    "loft: the blank overlaps for second profile lines {} and {}",
                    second_lines[0].0, second_lines[1].0
                )));
            }
            Ok(body)
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
    k_factor: f64,
    side: Side,
}

fn two_bend_profile(profile: [Vec3; 2], second: &Placed, lines: &[(String, Vec3, Vec3)]) -> Result<[Vec3; 3], BrepError> {
    let [a0, a1] = profile;
    let displacement = a1 - a0;
    let direction = unit(displacement).ok_or_else(|| BrepError::Degenerate("loft: the first profile line has zero length".into()))?;
    if !within_rounding(direction.dot(second.frame[3]), 1.0) {
        return Err(BrepError::Unsupported("loft: the first profile line must be parallel to the plane of the second profile".into()));
    }
    let (b0, apex, b1) = second_profile_corner(second, lines, a0 + displacement * 0.5, direction)?;
    Ok([b0, apex, b1])
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

/// The two tangent lines, both contained by the middle facet, meet at p + d*s.
fn line_meeting(p: Vec3, d: Vec3, q: Vec3, e: Vec3) -> Result<(Vec3, f64, f64), BrepError> {
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
    let size = length(first).max(length(delta)).max(length(e * t));
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
    let world = |name: &str| -> Result<Vec3, BrepError> {
        let p = sketch
            .solved
            .points
            .get(name)
            .copied()
            .ok_or_else(|| BrepError::LostReference(format!("loft: the second profile lost point {}", name)))?;
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
    let world = |name: &str| -> Result<Vec3, BrepError> {
        let p = sketch
            .solved
            .points
            .get(name)
            .copied()
            .ok_or_else(|| BrepError::LostReference(format!("loft: the {which} profile lost point {}", name)))?;
        Ok(sketch.frame[0] + sketch.frame[1] * p.x + sketch.frame[2] * p.y)
    };
    sketch.lines.iter().filter(|(_, line)| !line.construction).map(|(name, line)| Ok((name.clone(), world(&line.from)?, world(&line.to)?))).collect()
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
    wing_b: Vec3,
}

fn bend_outline(bend: Bend, end: f64, a: Vec3, a_other: Vec3, b: Vec3, apex: Vec3) -> Result<BendOutline, BrepError> {
    let [middle_edge, wing_edge] = [a_other - a, b - a];
    let (_, middle_station, middle_fraction) = line_meeting(bend.middle_tangent, bend.axis, a, middle_edge)?;
    let (_, wing_station, wing_fraction) = line_meeting(bend.wing_tangent, bend.axis, a, wing_edge)?;
    let start = middle_station.max(wing_station);
    let bad_trim = || BrepError::Degenerate("loft: a bend does not fit between the profile edges".into());
    if !(start >= 0.0 && start < end && end < bend.length) {
        return Err(bad_trim());
    }
    if [middle_fraction, wing_fraction].into_iter().any(|t| !(t > 0.0 && t < 1.0)) {
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
    Ok(BendOutline { start, end, middle_a, wing_a, wing_b })
}

#[derive(Clone, Copy)]
struct WingVertices {
    middle_a: usize,
    middle_tangent: usize,
    wing_tangent_a: usize,
    wing_a: usize,
    outer: usize,
    wing_b: usize,
    wing_tangent_b: usize,
}

#[derive(Clone, Copy)]
struct SkinVertices {
    corner: usize,
    wings: [WingVertices; 2],
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

fn two_bend_body(feature: TopoId, a: [Vec3; 2], b: [Vec3; 3], rule: BendRule) -> Result<Body, BrepError> {
    if !(rule.radius > 0.0 && rule.radius.is_finite()) {
        return Err(BrepError::Degenerate(format!("loft: bend radius must be positive and finite, got {}", rule.radius)));
    }
    if !(0.0..=1.0).contains(&rule.k_factor) {
        return Err(BrepError::Degenerate(format!("loft: K-factor must lie between 0 and 1, got {}", rule.k_factor)));
    }
    let [a0, a1] = a;
    let [b0, apex, b1] = b;
    let ends = [b0, b1];
    let normal =
        |p: Vec3, q: Vec3, r: Vec3| unit((q - p).cross(r - p)).ok_or_else(|| BrepError::Degenerate("loft: a triangular facet has no area".into()));
    let middle_normal = normal(a0, a1, apex)?;
    let wing_normals = [normal(a0, apex, b0)?, normal(a1, b1, apex)?];
    // The exact side of the wing's third point also gives the side of the bend centre.
    let radial_sign = |end| match orient3d_coords(a0, a1, apex, end) {
        Sign::Positive => Ok(1.0),
        Sign::Negative => Ok(-1.0),
        Sign::Zero => Err(BrepError::Degenerate("loft: a bend has no angle".into())),
    };
    let bends = [bend(a0, apex, [a1, b0], radial_sign(b0)?, rule)?, bend(a1, apex, [a0, b1], radial_sign(b1)?, rule)?];
    let [left, right] = bends;
    let (corner, end0, end1) = line_meeting(left.middle_tangent, left.axis, right.middle_tangent, right.axis)?;
    let outlines = [bend_outline(left, end0, a0, a1, b0, apex)?, bend_outline(right, end1, a1, a0, b1, apex)?];
    let along_a = a1 - a0;
    if (outlines[1].middle_a.unwrap_or(right.middle_tangent + right.axis * outlines[1].start)
        - outlines[0].middle_a.unwrap_or(left.middle_tangent + left.axis * outlines[0].start))
    .dot(along_a)
        <= 0.0
    {
        return Err(BrepError::Degenerate("loft: the two bends consume profile A".into()));
    }

    let (low, high) = thickness_sides(rule.side, rule.thickness);
    let wing_roles = ["first", "second"];
    let mut shell = LoftShell { feature, vertices: Vec::with_capacity(30), faces: Vec::with_capacity(17), curves: Vec::with_capacity(8) };
    let [lower, upper] = [("lower", low, -1.0), ("upper", high, 1.0)].map(|(skin, offset, outward)| {
        let middle_shift = middle_normal * offset;
        let corner = shell.vertex(skin, "middle", "B", corner + middle_shift);
        let wings = [0, 1].map(|i| {
            let role = wing_roles[i];
            let bend = bends[i];
            let outline = &outlines[i];
            let wing_shift = wing_normals[i] * offset;
            let mut vertex = |name, point| shell.vertex(skin, role, name, point);
            let middle_tangent = vertex("middle_tangent_A", bend.middle_tangent + bend.axis * outline.start + middle_shift);
            let wing_tangent_a = vertex("wing_tangent_A", bend.wing_tangent + bend.axis * outline.start + wing_shift);
            let wing_tangent_b = vertex("wing_tangent_B", bend.wing_tangent + bend.axis * outline.end + wing_shift);
            let middle_a = outline.middle_a.map_or(middle_tangent, |p| vertex("middle_outline_A", p + middle_shift));
            let wing_a = outline.wing_a.map_or(wing_tangent_a, |p| vertex("wing_outline_A", p + wing_shift));
            let outer = vertex("outer_B", ends[i] + wing_shift);
            let wing_b = vertex("wing_outline_B", outline.wing_b + wing_shift);

            shell.plane(skin, role, wing_normals[i] * outward, vec![wing_tangent_a, wing_a, outer, wing_b, wing_tangent_b]);
            let convex = outward == bend.radial_sign;
            let radius = rule.radius + if convex { rule.thickness } else { 0.0 };
            shell.face(
                skin,
                if i == 0 { "first_bend" } else { "second_bend" },
                Surface::Cylinder(Cylinder { origin: bend.center, axis: bend.axis, radius, convex }),
                vec![middle_tangent, corner, wing_tangent_b, wing_tangent_a],
            );
            shell.arc(middle_tangent, wing_tangent_a, bend.center + bend.axis * outline.start, bend.axis, radius);
            shell.arc(corner, wing_tangent_b, bend.center + bend.axis * outline.end, bend.axis, radius);
            WingVertices { middle_a, middle_tangent, wing_tangent_a, wing_a, outer, wing_b, wing_tangent_b }
        });
        let [l, r] = wings;
        shell.plane(skin, "middle", middle_normal * outward, vec![l.middle_a, r.middle_a, r.middle_tangent, corner, l.middle_tangent]);
        SkinVertices { corner, wings }
    });

    let [l, r] = lower.wings;
    let [u, v] = upper.wings;
    shell.wall("outline", "A", [l.middle_a, r.middle_a, v.middle_a, u.middle_a], lower.corner)?;
    for i in 0..2 {
        let role = wing_roles[i];
        let (l, u) = (lower.wings[i], upper.wings[i]);
        let axis = bends[i].axis;
        shell.wall(role, "side", [l.wing_a, l.outer, u.outer, u.wing_a], l.wing_b)?;
        shell.wall(role, "B", [l.outer, l.wing_b, u.wing_b, u.outer], l.wing_a)?;
        shell.plane(
            role,
            "A_end",
            -axis,
            vec![l.middle_a, l.middle_tangent, l.wing_tangent_a, l.wing_a, u.wing_a, u.wing_tangent_a, u.middle_tangent, u.middle_a],
        );
        shell.plane(role, "B_end", axis, vec![lower.corner, l.wing_tangent_b, l.wing_b, u.wing_b, u.wing_tangent_b, upper.corner]);
    }
    shell.build()
}
