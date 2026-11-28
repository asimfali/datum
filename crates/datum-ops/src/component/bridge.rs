//! A straight formed slot with a transition into the wall at each end.

use datum_brep::{Body, BrepError, FaceSpec, Ix, Plane, ProfileLoop, Region, Surface, TopoId, Vertex, build_with_curves, extrude};
use datum_math::{Rigid, Vec2, Vec3};

use super::{
    Forming, SectionSide, section,
    stitch::{split_arc, trim_strip_face, wall_faces},
    tongue::to_part,
};
use crate::rib::Formed;
use crate::sketch::{Placed, within_rounding};
use crate::unfold::StampCut;
use crate::{Value, validated};

pub(super) struct Bridge<'a> {
    pub wall: &'a Body,
    pub blank: &'a Body,
    pub face: TopoId,
    pub thickness: f64,
    pub radius: Option<f64>,
    pub frame: Rigid,
    pub axes: [Vec3; 3],
    pub anchor: Vec3,
    pub corners: [Vec3; 4],
    pub sides: [&'a str; 4],
}

pub(super) fn form(feature: TopoId, forming: &Forming, sketch: &Placed, bridge: Bridge<'_>) -> Result<Value, BrepError> {
    let Bridge { wall, blank, face, thickness: t, radius, frame, axes: [x, y, z], anchor, corners: [a, b, c, d], sides } = bridge;
    let bad = || BrepError::Unsupported(format!("stamp: section `{}` needs two upright transitions and a flat bridge", forming.section));
    if forming.section_side != SectionSide::Outer {
        return Err(BrepError::Unsupported("stamp: the formed slot supports an outer section".into()));
    }
    let contour = sketch.contours(&|_, construction| !construction);
    let [path] = contour.paths.as_slice() else { return Err(bad()) };
    if !contour.loops.is_empty()
        || !contour.branches.is_empty()
        || !path.iter().all(|s| sketch.lines.contains_key(s.curve))
        || sketch.circles.values().any(|c| !c.construction)
        || sketch.ellipses.values().any(|e| !e.construction)
        || !sketch.patterns.is_empty()
    {
        return Err(bad());
    }
    let first = path.first().ok_or_else(bad)?;
    let mut names = Vec::with_capacity(path.len() + 1);
    names.push(first.from);
    names.extend(path.iter().map(|s| s.to));
    let point = |name: &str| to_part(sketch, sketch.solved.points[name]) - anchor;
    if point(names[0]).dot(x) > point(names.last().ok_or_else(bad)?).dot(x) {
        names.reverse();
    }
    let ([p, q, r, s], middle) = match *names.as_slice() {
        [p, q, r, s] => ([p, q, r, s], None),
        [p, q, m, r, s] => ([p, q, r, s], Some(m)),
        _ => return Err(bad()),
    };
    let [left, right] = [a.dot(x), d.dot(x)];
    let [lo, hi] = [a.dot(y).min(b.dot(y)), a.dot(y).max(b.dot(y))];
    let scale = [a, b, c, d, anchor, point(q), point(r)].iter().flat_map(|p| p.to_array()).fold(t, |scale, v| scale.max(v.abs()));
    let zero = |v| within_rounding(v, scale);
    let [p0, p1, p2, p3] = [p, q, r, s].map(point);
    let height = p1.dot(z);
    if !(left < right && lo < 0.0 && hi > 0.0 && height > t)
        || ![(b - a).dot(x), (c - d).dot(x), (d - a).dot(y), (c - b).dot(y)].into_iter().all(zero)
        || ![a, b, c, d].into_iter().all(|v| zero(v.dot(z)))
        || ![p0, p1, p2, p3].into_iter().all(|v| zero(v.dot(y)))
        || ![p0.dot(x) - left, p1.dot(x) - left, p2.dot(x) - right, p3.dot(x) - right, p0.dot(z), p3.dot(z), p2.dot(z) - height].into_iter().all(zero)
        || middle.is_some_and(|name| {
            let v = point(name);
            !(left < v.dot(x) && v.dot(x) < right && zero(v.dot(z) - height) && zero(v.dot(y)))
        })
    {
        return Err(bad());
    }
    let outer = [Vec2::new(left, 0.0), Vec2::new(left, height), Vec2::new(right, height), Vec2::new(right, 0.0)];
    let id = |kind: &str, side: &str, name: &str| TopoId::derive(kind).id(feature).tag(side).tag(name).finish();
    let joints = [id("stamp.joint", "", sides[0]), id("stamp.joint", "", sides[2])];
    let [near, far] = joints;
    let [low_cut, high_cut] = if a.dot(y) < b.dot(y) { [sides[3], sides[1]] } else { [sides[1], sides[3]] };
    let (profile, crosses, top_indices, bottom_indices) = if forming.round_section {
        let radius = radius.ok_or_else(|| BrepError::Invalid("stamp: a rounded slot needs a bend radius from the sheet rule".into()))?;
        let line = |name| path.iter().find(|s| s.from == name || s.to == name).map(|s| s.curve).ok_or_else(bad);
        let shelf = path.iter().find(|step| (step.from == q && step.to != p) || (step.to == q && step.from != p)).ok_or_else(bad)?;
        let (profile, crosses) = section::rounded_bridge(feature, outer, [p, q, r, s], [line(p)?, shelf.curve, line(s)?], joints, t, radius)?;
        let half = profile.points.len() / 2;
        (profile, Some(crosses), [2 * half - 1, half], [0, half - 1])
    } else {
        let mut inner = section::offset(outer, -t)?;
        inner[0].y = -t;
        inner[3].y = -t;
        let profile = ProfileLoop {
            points: outer.into_iter().chain(inner.into_iter().rev()).collect(),
            vertices: ["outer", "inner"]
                .into_iter()
                .zip([[p, q, r, s], [s, r, q, p]])
                .flat_map(|(skin, points)| points.map(|name| ["high", "low"].map(|side| id(skin, side, name))))
                .collect(),
            sides: [
                id("stamp.slot", "outer", p),
                id("stamp.slot", "outer", q),
                id("stamp.slot", "outer", r),
                far,
                id("stamp.slot", "inner", r),
                id("stamp.slot", "inner", q),
                id("stamp.slot", "inner", p),
                near,
            ]
            .into(),
            arcs: Vec::new(),
        };
        (profile, None, [0, 3], [7, 4])
    };
    let [start, end] = top_indices.map(|i| profile.points[i].x);
    let [[upper_near_high, upper_near_low], [upper_far_high, upper_far_low]] = top_indices.map(|i| profile.vertices[i]);
    let [[lower_near_high, lower_near_low], [lower_far_high, lower_far_low]] = bottom_indices.map(|i| profile.vertices[i]);
    let section_frame = Rigid::frame(Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.0, -1.0, 0.0));
    let strip =
        extrude(&[Region { outer: profile, holes: Vec::new(), caps: [id("stamp.strip", "", high_cut), id("stamp.strip", "", low_cut)] }], -hi, -lo)?
            .transformed(&section_frame.then(frame));
    let (top, bottom) = wall_faces(wall, &strip, face, t, frame, [[lo, hi], [start, end]], || {
        BrepError::Unsupported("stamp: the formed slot needs a clear opening in a flat wall".into())
    })?;
    let mut points = wall.vertices().to_vec();
    points.extend_from_slice(strip.vertices());
    let mut curves = strip.curve_specs();
    let mut splits = Vec::new();
    let roots =
        [(upper_near_low, lower_near_low), (upper_near_high, lower_near_high), (upper_far_low, lower_far_low), (upper_far_high, lower_far_high)];
    let sharp_upper = [upper_near_low, upper_near_high, upper_far_high, upper_far_low];
    let [near_low, near_high, far_low, far_high] = if let Some([x0, x1]) = crosses {
        let cross_ids = ["near.low", "near.high", "far.low", "far.high"].map(|name| id("stamp.slot.cross", "", name));
        for (((_, bottom), cross), (x, y)) in roots.into_iter().zip(cross_ids).zip([(x0, lo), (x0, hi), (x1, lo), (x1, hi)]) {
            points.push(Vertex { id: cross, point: frame.apply(Vec3::new(x, y, 0.0)) });
            splits.push(split_arc(&mut curves, bottom, cross).ok_or_else(bad)?);
        }
        cross_ids
    } else {
        [upper_near_low, upper_near_high, upper_far_low, upper_far_high]
    };
    let mut faces: Vec<_> = (0..wall.faces().len()).map(|i| wall.face_spec(i as Ix)).collect();
    let upper = if crosses.is_some() {
        vec![upper_near_low, upper_near_high, near_high, far_high, upper_far_high, upper_far_low, far_low, near_low]
    } else {
        sharp_upper.into()
    };
    faces[top].loops.push(upper);
    faces[bottom].loops.push(vec![lower_near_low, lower_far_low, lower_far_high, lower_near_high]);
    // Omit the two seam caps and stitch the roots into the matching wall skins.
    faces.extend((0..strip.faces().len()).filter(|&i| !joints.contains(&strip.faces()[i].id)).map(|i| {
        let mut face = strip.face_spec(i as Ix);
        if crosses.is_some() {
            trim_strip_face(&mut face, &splits, &roots);
        }
        face
    }));
    for (side, at, normal, ring) in [
        (low_cut, lo, 1.0, [near_low, far_low, lower_far_low, lower_near_low]),
        (high_cut, hi, -1.0, [far_high, near_high, lower_near_high, lower_far_high]),
    ] {
        faces.push(FaceSpec {
            id: id("stamp.cut.wall", "", side),
            surface: Surface::Plane(Plane { origin: frame.apply(Vec3::new(left, at, 0.0)), normal: frame.apply_vector(Vec3::new(0.0, normal, 0.0)) }),
            loops: vec![ring.into()],
        });
    }
    curves.extend(wall.curve_specs());
    let body = build_with_curves(points, faces, curves)?;
    validated(&body)?;
    // The slot is slit before forming. Its two cut lines are the sketch edges,
    // independent of the tangencies and the 3D openings between them.
    let stamp_cuts = [([b, c], sides[1]), ([d, a], sides[3])]
        .into_iter()
        .map(|(edge, name)| StampCut {
            faces: [face, wall.faces()[bottom].id],
            points: edge.map(|p| frame.apply(Vec3::new(p.dot(x), p.dot(y), 0.0))).into(),
            normal: frame.apply_vector(Vec3::new(0.0, 0.0, 1.0)),
            ids: vec![id("stamp.cut", "", name)],
            arcs: vec![None],
        })
        .collect();
    Ok(Value::Formed(Box::new(Formed { body, blank: blank.clone(), imprints: Vec::new(), stamp_cuts })))
}
