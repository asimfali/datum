use datum_brep::{
    Arc, Body, BrepError, Curve, CurveSpec, Cylinder, FaceSpec, Plane, ProfileArc, Surface, TopoId, Vertex, build_with_curves, segments_touch,
};
use datum_math::{Arc2, Sign, Vec2, Vec3, arcs_touch, orient2d, segment_touches_arc};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::sketch::{nearly_parallel, within_rounding};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum EdgeMode {
    #[default]
    Fillet,
    Chamfer,
}

/// Replace a corner by tangent points on its incoming and outgoing segments.
pub(crate) fn corner(vertex: Vec2, neighbors: [Vec2; 2], size: f64, mode: EdgeMode) -> Result<([Vec2; 2], Option<ProfileArc>), BrepError> {
    let bad = || BrepError::Degenerate("edge finish: invalid corner or size does not fit".into());
    if !size.is_finite() || size <= 0.0 || !vertex.x.is_finite() || !vertex.y.is_finite() {
        return Err(bad());
    }
    let mut units = [Vec2::default(); 2];
    let mut lengths = [0.0; 2];
    for i in 0..2 {
        let d = Vec2::new(neighbors[i].x - vertex.x, neighbors[i].y - vertex.y);
        lengths[i] = libm::hypot(d.x, d.y);
        if !lengths[i].is_finite() || lengths[i] == 0.0 {
            return Err(bad());
        }
        units[i] = Vec2::new(d.x / lengths[i], d.y / lengths[i]);
    }
    let turn = orient2d(neighbors[0], vertex, neighbors[1]);
    if turn == Sign::Zero {
        return Err(bad());
    }
    let dot = (units[0].x * units[1].x + units[0].y * units[1].y).clamp(-1.0, 1.0);
    let cross = (units[0].x * units[1].y - units[0].y * units[1].x).abs();
    let distance = match mode {
        EdgeMode::Chamfer => size,
        EdgeMode::Fillet => size * (1.0 + dot) / cross,
    };
    if !distance.is_finite() || distance <= 0.0 || lengths.iter().any(|&len| distance >= len) {
        return Err(bad());
    }
    let points = units.map(|u| Vec2::new(vertex.x + distance * u.x, vertex.y + distance * u.y));
    let arc = match mode {
        EdgeMode::Chamfer => None,
        EdgeMode::Fillet => {
            let n = if turn == Sign::Positive { Vec2::new(units[0].y, -units[0].x) } else { Vec2::new(-units[0].y, units[0].x) };
            Some(ProfileArc { center: Vec2::new(points[0].x + size * n.x, points[0].y + size * n.y), ccw: turn == Sign::Positive })
        }
    };
    Ok((points, arc))
}

/// Finish a straight edge where two planes meet between parallel end faces.
pub(crate) fn finish(feature: TopoId, body: &Body, edge: TopoId, size: f64, mode: EdgeMode) -> Result<Body, BrepError> {
    let bad = || BrepError::Unsupported("edge finish: expected a convex straight edge between parallel planar caps".into());
    let selected = body.edges().iter().find(|e| e.id == edge).ok_or_else(bad)?;
    if body.edge_curve(selected).is_some() {
        return Err(bad());
    }
    let half = body.half_edges()[selected.half as usize];
    let twin = body.half_edges()[half.twin as usize];
    let ends = [half.origin, twin.origin];
    let positions = ends.map(|v| body.point(v));
    let delta = positions[1] - positions[0];
    let height = libm::sqrt(delta.dot(delta));
    if !height.is_finite() || height <= 0.0 {
        return Err(bad());
    }
    let z = delta * (1.0 / height);
    let side_faces = [half, twin].map(|h| body.loops()[h.loop_ix as usize].face as usize);
    for &f in &side_faces {
        let plane = body.faces()[f].surface.plane().ok_or_else(bad)?;
        if !within_rounding(plane.normal.dot(z), libm::sqrt(plane.normal.dot(plane.normal))) {
            return Err(bad());
        }
    }
    // Each endpoint must have exactly one end cap and no other incident faces.
    let mut caps = [0usize; 2];
    let mut neighbors = [[0u32; 2]; 2];
    for k in 0..2 {
        let mut found = Vec::new();
        for (i, f) in body.faces().iter().enumerate() {
            if side_faces.contains(&i) {
                continue;
            }
            for l in f.first_loop..f.first_loop + f.loop_count {
                let hs: Vec<_> = body.loop_half_edges(l).collect();
                if let Some(at) = hs.iter().position(|&h| body.half_edges()[h as usize].origin == ends[k]) {
                    found.push((i, hs, at));
                }
            }
        }
        if found.len() != 1 {
            return Err(bad());
        }
        let (face, hs, at) = &found[0];
        let plane = body.faces()[*face].surface.plane().ok_or_else(bad)?;
        if !nearly_parallel(plane.normal, z) {
            return Err(bad());
        }
        caps[k] = *face;
        let prev = hs[(*at + hs.len() - 1) % hs.len()];
        let next = hs[*at];
        if body.edge_curve(&body.edges()[body.half_edges()[prev as usize].edge as usize]).is_some()
            || body.edge_curve(&body.edges()[body.half_edges()[next as usize].edge as usize]).is_some()
        {
            return Err(bad());
        }
        neighbors[k] = [body.half_edges()[prev as usize].origin, body.half_edges()[body.next(next) as usize].origin];
    }
    if caps[0] == caps[1] {
        return Err(bad());
    }
    let d = body.point(neighbors[0][1]) - positions[0];
    let len = libm::sqrt(d.dot(d));
    if len <= 0.0 || !within_rounding(d.dot(z), len) {
        return Err(bad());
    }
    let x = d * (1.0 / len);
    let y = z.cross(x);
    let project = |v: Vec3| Vec2::new(v.dot(x), v.dot(y));
    let n0 = neighbors[0].map(|v| project(body.point(v) - positions[0]));
    let (cuts, arc) = corner(Vec2::default(), n0, size, mode)?;
    let local = |p: Vec2| x * p.x + y * p.y;
    // Match both end-cap neighbours by their corresponding side face.
    let side_of = |endpoint: u32, neighbor: u32| -> Result<usize, BrepError> {
        let id = body.vertices()[neighbor as usize].id;
        let origin = body.vertices()[endpoint as usize].id;
        side_faces
            .iter()
            .copied()
            .find(|&f| {
                body.face_spec(f as u32)
                    .loops
                    .iter()
                    .any(|l| (0..l.len()).any(|i| (l[i] == id && l[(i + 1) % l.len()] == origin) || (l[i] == origin && l[(i + 1) % l.len()] == id)))
            })
            .ok_or_else(bad)
    };
    let side = [side_of(ends[0], neighbors[0][0])?, side_of(ends[0], neighbors[0][1])?];
    if side[0] == side[1] {
        return Err(bad());
    }
    let map = [side_of(ends[1], neighbors[1][0])?, side_of(ends[1], neighbors[1][1])?];
    if map != [side[1], side[0]] {
        return Err(bad());
    }
    for j in 0..2 {
        let a = body.point(neighbors[0][j]) - positions[0];
        let b = body.point(neighbors[1][1 - j]) - positions[1];
        let diff = a - b;
        if !within_rounding(libm::sqrt(diff.dot(diff)), len.max(height)) {
            return Err(bad());
        }
        let plane = body.faces()[side[1 - j]].surface.plane().ok_or_else(bad)?;
        if a.dot(plane.normal) >= 0.0 {
            return Err(bad());
        }
    }
    // The selected edge defines +Z. Cap 0 is viewed along that axis; Arc2 always
    // describes a counterclockwise arc, so reverse endpoints for clockwise arcs.
    let new_arc = arc.map(|a| {
        let [from, to] = if a.ccw { cuts } else { [cuts[1], cuts[0]] };
        Arc2 { center: a.center, from, to }
    });
    for k in 0..2 {
        for lp in body.face_loops(caps[k] as u32) {
            for h in body.loop_half_edges(lp) {
                let he = body.half_edges()[h as usize];
                let next = body.half_edges()[body.next(h) as usize];
                if he.origin == ends[k] || next.origin == ends[k] {
                    continue;
                }
                let a = project(body.point(he.origin) - positions[k]);
                let b = project(body.point(next.origin) - positions[k]);
                let edge = &body.edges()[he.edge as usize];
                let touches = if let Some(curve) = body.edge_curve(edge) {
                    let circle = curve.arc().ok_or_else(bad)?;
                    if !nearly_parallel(circle.axis, z) {
                        return Err(bad());
                    }
                    let forward = edge.half == h;
                    let ccw = (circle.axis.dot(z) > 0.0) == forward;
                    let [from, to] = if ccw { [a, b] } else { [b, a] };
                    let old_arc = Arc2 { center: project(circle.center - positions[k]), from, to };
                    new_arc.as_ref().map_or_else(|| segment_touches_arc(cuts[0], cuts[1], &old_arc), |new| arcs_touch(new, &old_arc))
                } else {
                    new_arc.as_ref().map_or_else(|| segments_touch(cuts[0], cuts[1], a, b), |new| segment_touches_arc(a, b, new))
                };
                if touches {
                    return Err(BrepError::Degenerate("edge finish: size intersects another boundary".into()));
                }
            }
        }
    }
    let ids = [
        [0u64, 1].map(|j| TopoId::derive("edge_finish.vertex").id(feature).index(0).index(j).finish()),
        [0u64, 1].map(|j| TopoId::derive("edge_finish.vertex").id(feature).index(1).index(j).finish()),
    ];
    let mut vertices: Vec<_> = body.vertices().iter().filter(|v| !ends.iter().any(|&i| body.vertices()[i as usize].id == v.id)).copied().collect();
    for k in 0..2 {
        for j in 0..2 {
            vertices.push(Vertex { id: ids[k][j], point: positions[k] + local(cuts[j]) });
        }
    }
    let old = ends.map(|i| body.vertices()[i as usize].id);
    let mut faces: Vec<_> = (0..body.faces().len()).map(|i| body.face_spec(i as u32)).collect();
    for (f, face) in faces.iter_mut().enumerate() {
        for boundary in &mut face.loops {
            for k in 0..2 {
                if let Some(at) = boundary.iter().position(|&v| v == old[k]) {
                    if f == caps[k] {
                        let replacement = if k == 0 { [ids[k][0], ids[k][1]] } else { [ids[k][1], ids[k][0]] };
                        boundary.splice(at..=at, replacement);
                    } else if let Some(j) = side.iter().position(|&s| s == f) {
                        boundary[at] = ids[k][j];
                    } else {
                        return Err(bad());
                    }
                }
            }
        }
    }
    let surface = if let Some(a) = arc {
        Surface::Cylinder(Cylinder { origin: positions[0] + local(a.center), axis: z, radius: size, convex: true })
    } else {
        let a = positions[0] + local(cuts[0]);
        let b = positions[0] + local(cuts[1]);
        Surface::Plane(Plane { origin: a, normal: (a - b).cross(z) })
    };
    faces.push(FaceSpec {
        id: TopoId::derive("edge_finish.face").id(feature).finish(),
        surface,
        loops: vec![vec![ids[0][1], ids[0][0], ids[1][0], ids[1][1]]],
    });
    let mut curves = body.curve_specs();
    if let Some(a) = arc {
        for k in 0..2 {
            curves.push(CurveSpec {
                from: ids[k][0],
                to: ids[k][1],
                curve: Curve::Arc(Arc { center: positions[k] + local(a.center), axis: if a.ccw { z } else { z * -1.0 }, radius: size }),
            });
        }
    }
    let result = build_with_curves(vertices, faces, curves)?;
    crate::validated(&result)?;
    Ok(result)
}
