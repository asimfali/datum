use std::collections::BTreeMap;
use std::f64::consts::TAU;

use datum_brep::{Axis, Body, BrepError, Ix, Region, TopoId, build_with_curves, revolve};
use datum_math::{Rigid, Vec2, Vec3};
use datum_sketch::Circle;

use super::{Forming, SectionSide};
use crate::sheet::{Sheet, SheetPath, Side, bend_profile};
use crate::{Num, Placed, Value, cut, num, rib::Formed, validated};

fn bad(message: &str) -> BrepError {
    BrepError::Unsupported(format!("dimple: {message}"))
}

#[allow(clippy::too_many_arguments)]
pub(super) fn form(
    feature: TopoId,
    wall: &Body,
    face: TopoId,
    onto_wall: Rigid,
    anchor: Vec3,
    section: &Placed,
    boundary: &Placed,
    name: &str,
    circle: &Circle,
    [t, bend_radius]: [f64; 2],
    metadata: &BTreeMap<String, serde_json::Value>,
    forming: &Forming,
) -> Result<Value, BrepError> {
    let contours = boundary.contours(&|_, construction| !construction);
    if !contours.paths.is_empty()
        || !contours.loops.is_empty()
        || !contours.branches.is_empty()
        || !boundary.patterns.is_empty()
        || boundary.ellipses.values().any(|e| !e.construction)
    {
        return Err(bad("the boundary needs one transition circle"));
    }
    let cut_on_blank = match metadata.get("stamp.cut") {
        None => true,
        Some(value) if value.as_str() == Some("flat") => true,
        Some(value) if value.as_str() == Some("tool") => false,
        _ => return Err(bad("the cut mode must be flat or tool")),
    };
    let inner = match metadata.get("stamp.section_side") {
        None => forming.section_side == SectionSide::Inner,
        Some(value) if value.as_str() == Some("outer") => false,
        Some(value) if value.as_str() == Some("inner") => true,
        _ => return Err(bad("the section side must be outer or inner")),
    };
    let [bo, bx, by, bz] = boundary.frame;
    let center = boundary.solved.points[&circle.center];
    let center = bo + bx * center.x + by * center.y;
    let [so, sx, sy, sz] = section.frame;
    let tol = section.solved.tolerance.max(boundary.solved.tolerance);
    if (center - so).dot(sz).abs() > tol || sx.dot(bz).abs() > tol || (sy.dot(bz) - 1.0).abs() > tol || (center - anchor).dot(bz).abs() > tol {
        return Err(bad("the section must stand through the transition circle axis"));
    }
    let r = num(&circle.diameter)? / 2.0;
    let shift = center - so;
    let local = |p: &str| {
        let p = section.solved.points[p];
        Vec2::new(shift.dot(sx) - p.x, p.y - shift.dot(sy))
    };
    let contours = section.contours(&|_, construction| !construction);
    let [path] = contours.paths.as_slice() else {
        return Err(bad("the section needs one open chain"));
    };
    if !contours.loops.is_empty()
        || !contours.branches.is_empty()
        || !section.patterns.is_empty()
        || section.circles.values().any(|c| !c.construction)
        || section.ellipses.values().any(|e| !e.construction)
    {
        return Err(bad("the section needs one open chain"));
    }
    let from_root = |p: Vec2| (p.x - r).abs() <= tol && p.y.abs() <= tol;
    let [first, ..] = path.as_slice() else {
        return Err(bad("empty section"));
    };
    let reversed;
    let path = if from_root(local(first.from)) {
        path.as_slice()
    } else if path.last().is_some_and(|last| from_root(local(last.to))) {
        reversed = datum_sketch::reversed(path);
        &reversed
    } else {
        return Err(bad("the section must start at the transition circle"));
    };

    let root = Vec2::new(r, 0.0);
    let (mut points, arcs, hole) = if let [step] = path
        && let Some(arc) = section.arcs.get(step.curve)
    {
        let c = local(&arc.center);
        if step.forward || c.x.abs() > tol || c.y > tol || !from_root(Vec2::new(-local(step.to).x, local(step.to).y)) {
            return Err(bad("a spherical section needs an axis-centred arc across the circle"));
        }
        let radius = libm::hypot(r, c.y);
        let c = Vec2::new(0.0, c.y);
        let height = c.y + radius;
        (vec![root, Vec2::new(0.0, height)], vec![None, Some((c, true))], 0.0)
    } else {
        let [slope, shelf, ..] = path else {
            return Err(bad("a conical section needs a slope and shelf"));
        };
        let shoulder = local(slope.to);
        let end = local(shelf.to);
        if !path.iter().all(|s| section.lines.contains_key(s.curve))
            || path.len() > 3
            || (path.len() == 3 && end.x >= -tol)
            || (end.y - shoulder.y).abs() > tol
            || shoulder.y <= 0.0
            || shoulder.x <= end.x.max(0.0)
            || shoulder.x >= r
        {
            return Err(bad("invalid conical slope or shelf"));
        }
        if let [_, _, other_slope] = path
            && ((end.x + shoulder.x).abs() > tol || !from_root(Vec2::new(-local(other_slope.to).x, local(other_slope.to).y)))
        {
            return Err(bad("a full conical section must be symmetric about the circle axis"));
        }
        let hole = if end.x > tol { end.x } else { 0.0 };
        (vec![root, shoulder, Vec2::new(hole, shoulder.y)], vec![None; 3], hole)
    };
    let origin = onto_wall.apply(center - anchor);
    let z = onto_wall.apply_vector(bz);
    let axis = Axis { origin, axis: z, start: onto_wall.apply_vector(bx) };
    let frame = Rigid::frame(origin, axis.start, z.cross(axis.start), z);
    let id = |tag, index| TopoId::derive(tag).id(feature).tag(name).index(index as u64).finish();
    // The flat lead lets the sheet bend builder find the tangency at the root on both skins.
    points.insert(0, Vec2::new(r + 2.0 * (bend_radius + t), 0.0));
    let names = &["lead", "root", "shoulder", "tip"][..points.len()];
    let lines = &["wall", "profile", "shelf"][..arcs.len()];
    let side = if inner { Side::Right } else { Side::Left };
    let mut outer = bend_profile(
        id("stamp.dimple.section", 0),
        SheetPath { p: &points, names, lines, arcs: &arcs },
        Sheet { thickness: t, radius: bend_radius, side },
        &|_, _| false,
    )?;
    // Drop the flat lead. Its two tangent rings become holes in the matching wall skins.
    outer.points.remove(0);
    outer.points.pop();
    outer.vertices.remove(0);
    outer.vertices.pop();
    outer.sides.remove(0);
    outer.sides.pop();
    outer.arcs.remove(0);
    outer.arcs.pop();
    let last = outer.points.len() - 1;
    let closing = id("stamp.dimple.closing", 0);
    outer.sides[last] = closing;
    let [outer_root, inner_root] = [outer.vertices[0][0], outer.vertices[last][0]];
    let extent = outer.points.iter().fold(Vec2::default(), |m, p| Vec2::new(m.x.max(p.x), m.y.max(p.y)));
    let shift = if inner { t } else { 0.0 };
    for point in &mut outer.points {
        *point = Vec2::new(point.y - shift, point.x);
    }
    for arc in outer.arcs.iter_mut().flatten() {
        arc.center = Vec2::new(arc.center.y - shift, arc.center.x);
        arc.ccw = !arc.ccw;
    }
    let patch = revolve(&Region { outer, holes: Vec::new(), caps: [closing; 2] }, &axis, TAU)?;
    let extent_r = extent.x;
    let (top, bottom) = super::stitch::wall_faces_in(
        wall,
        face,
        t,
        frame,
        [[-extent_r, extent_r]; 2],
        (Vec3::new(-extent_r, -extent_r, -t), Vec3::new(extent_r, extent_r, extent.y)),
        || bad("the dimple needs a clear opening in a flat wall"),
    )?;
    let mut faces = (0..wall.faces().len()).map(|i| wall.face_spec(i as Ix)).collect::<Vec<_>>();
    for (i, root, reverse) in [(top, outer_root, true), (bottom, inner_root, false)] {
        let ring = (0..3).map(|k| TopoId::derive("revolve.v").id(root).index(k).finish());
        faces[i].loops.push(if reverse { ring.rev().collect() } else { ring.collect() });
    }
    let caps = [closing, TopoId::derive("revolve.side").id(closing).index(1).finish(), TopoId::derive("revolve.side").id(closing).index(2).finish()];
    faces.extend((0..patch.faces().len()).filter(|&i| !caps.contains(&patch.faces()[i].id)).map(|i| patch.face_spec(i as Ix)));
    let mut vertices = wall.vertices().to_vec();
    vertices.extend_from_slice(patch.vertices());
    let mut curves = wall.curve_specs();
    curves.extend(patch.curve_specs());
    let body = build_with_curves(vertices, faces, curves)?;
    validated(&body)?;
    let blank = if hole > 0.0 && cut_on_blank {
        let mut cut_sketch = boundary.clone();
        cut_sketch.frame = [origin, axis.start, z.cross(axis.start), z];
        cut_sketch.face = Some(face);
        cut_sketch.solved.points.insert(circle.center.clone(), Vec2::default());
        cut_sketch.circles = [(name.to_owned(), Circle { diameter: Num::Value(2.0 * hole), ..circle.clone() })].into();
        cut::from_face(id("stamp.dimple.blank", 0), wall, &cut_sketch, cut::Reach::Through)?
    } else {
        wall.clone()
    };
    Ok(Value::Formed(Box::new(Formed { body, blank, imprints: Vec::new(), stamp_cuts: Vec::new() })))
}
