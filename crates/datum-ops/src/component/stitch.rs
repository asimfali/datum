use datum_brep::{
    Arc, Body, BrepError, Curve, CurveSpec, Cylinder, FaceSpec, Ix, Plane, ProfileArc, ProfileLoop, Surface, TopoId, Vertex, build_with_curves,
};
use datum_math::{Rigid, Vec2, Vec3};

use crate::cut::Frame;
use crate::sketch::{nearly_parallel, within_rounding};

pub(super) struct FreeBoundary {
    pub points: Vec<Vec2>,
    pub arcs: Vec<Option<ProfileArc>>,
    pub ids: Vec<TopoId>,
}

pub(super) struct Window {
    pub face: TopoId,
    pub thickness: f64,
    pub root_cap: TopoId,
    /// The two outside root vertices, then the two inside ones, each in transverse order.
    pub root_ids: [TopoId; 4],
    /// Where the lower root bend crosses the outside plane, along the length of the window.
    pub root_cross: Option<f64>,
    pub to_world: Rigid,
    pub sides: [f64; 2],
    pub inner_root: f64,
    pub outer_root: f64,
    pub far: f64,
    pub outline: FreeBoundary,
}

/// Join a strip to a flat wall, leaving its developed window between the two skins.
pub(super) fn stitch(feature: TopoId, wall: &Body, strip: &Body, window: &Window) -> Result<(Body, TopoId), BrepError> {
    let Window { face, thickness: t, root_cap: root, root_ids, root_cross, to_world, sides, inner_root, outer_root, far, ref outline } = *window;
    let bad = || BrepError::Unsupported("stamp: the tongue needs a clear window in a flat wall".into());
    if !t.is_finite()
        || t <= 0.0
        || !sides.iter().all(|v| v.is_finite())
        || sides[0] >= sides[1]
        || !outer_root.is_finite()
        || !inner_root.is_finite()
        || !far.is_finite()
        || outer_root > inner_root
        || inner_root >= far
        || root_cross.is_some_and(|x| x >= outline.points[1].x || x >= outline.points[outline.points.len() - 2].x)
    {
        return Err(bad());
    }
    let (top, bottom) = wall_faces(wall, strip, face, t, to_world, [sides, [outer_root, far]], bad)?;
    let at = |a, b, c| to_world.apply(Vec3::new(a, b, c));
    let z = to_world.apply_vector(Vec3::new(0.0, 0.0, 1.0));
    let cap = strip.faces().iter().position(|f| f.id == root).ok_or_else(bad)?;
    let names = ["top.left.tip", "top.right.tip", "bottom.left.root", "bottom.left.tip", "bottom.right.tip", "bottom.right.root"];
    let id = |k: usize| TopoId::derive("stamp.window.vertex").id(feature).tag(names[k]).finish();
    let mut vertices = wall.vertices().to_vec();
    vertices.extend_from_slice(strip.vertices());
    if root_cross.is_none() {
        for (i, y) in [(2, sides[0]), (5, sides[1])] {
            vertices.push(Vertex { id: id(i), point: at(inner_root, y, -t) });
        }
    }
    let mut faces: Vec<FaceSpec> = (0..wall.faces().len()).map(|i| wall.face_spec(i as Ix)).collect();
    let [top_left, top_right, bottom_left, bottom_right] = root_ids;
    let mut strip_curves = strip.curve_specs();
    let mut split_edges = Vec::new();
    let [upper_left, upper_right] = if let Some(x) = root_cross {
        let crosses = ["left", "right"].map(|side| TopoId::derive("stamp.window.cross").id(feature).tag(side).finish());
        for ((bottom, cross), y) in [bottom_left, bottom_right].into_iter().zip(crosses).zip(sides) {
            vertices.push(Vertex { id: cross, point: at(x, y, 0.0) });
            split_edges.push(split_arc(&mut strip_curves, bottom, cross).ok_or_else(bad)?);
        }
        crosses
    } else {
        [bottom_left, bottom_right]
    };
    let [lower_left, lower_right] = if root_cross.is_some() { [bottom_left, bottom_right] } else { [id(2), id(5)] };
    let mut upper = vec![upper_left];
    let mut lower = vec![lower_left];
    let end = outline.points.len() - 1;
    for (i, p) in outline.points.iter().enumerate().take(end).skip(1) {
        let [a, b] = if end == 3 {
            [[id(0), id(3)], [id(1), id(4)]][i - 1]
        } else {
            ["top", "bottom"].map(|layer| TopoId::derive("stamp.window.vertex").id(outline.ids[i - 1]).tag(layer).finish())
        };
        vertices.push(Vertex { id: a, point: at(p.x, p.y, 0.0) });
        vertices.push(Vertex { id: b, point: at(p.x, p.y, -t) });
        upper.push(a);
        lower.push(b);
    }
    upper.push(upper_right);
    lower.push(lower_right);
    faces[top].loops.push([top_left, top_right].into_iter().chain(upper.iter().rev().copied()).collect());
    faces[bottom].loops.push(lower.clone());
    faces.extend((0..strip.faces().len()).filter(|&i| i != cap).map(|i| {
        let mut face = strip.face_spec(i as Ix);
        if root_cross.is_some() {
            trim_strip_face(&mut face, &split_edges, &[(top_left, bottom_left), (top_right, bottom_right)]);
        }
        face
    }));
    let mut curves = wall.curve_specs();
    curves.extend(strip_curves);
    for k in 0..=end {
        if k == end && root_cross.is_some() {
            continue;
        }
        let next = (k + 1) % outline.points.len();
        let (a, b) = (outline.points[k], outline.points[next]);
        let plane = || {
            let o = at(a.x, a.y, 0.0);
            Surface::Plane(Plane { origin: o, normal: z.cross(at(b.x, b.y, 0.0) - o) })
        };
        let surface = if let Some(arc) = outline.arcs.get(k).copied().flatten() {
            let radius = libm::hypot(a.x - arc.center.x, a.y - arc.center.y);
            for (height, ids) in [(0.0, &upper), (-t, &lower)] {
                curves.push(CurveSpec {
                    from: ids[k],
                    to: ids[next],
                    curve: Curve::Arc(Arc { center: at(arc.center.x, arc.center.y, height), axis: if arc.ccw { z } else { -z }, radius }),
                });
            }
            Surface::Cylinder(Cylinder { origin: at(arc.center.x, arc.center.y, 0.0), axis: z, radius, convex: false })
        } else {
            plane()
        };
        let id = if k == end {
            TopoId::derive("stamp.window.face").id(feature).tag("root").finish()
        } else if end == 3 {
            TopoId::derive("stamp.window.face").id(feature).tag(["left", "tip", "right"][k]).finish()
        } else {
            TopoId::derive("stamp.window.face").id(outline.ids[k]).finish()
        };
        faces.push(FaceSpec { id, surface, loops: vec![vec![upper[k], upper[next], lower[next], lower[k]]] });
    }
    Ok((build_with_curves(vertices, faces, curves)?, wall.faces()[bottom].id))
}

pub(super) fn split_arc(curves: &mut Vec<CurveSpec>, bottom: TopoId, cross: TopoId) -> Option<(TopoId, TopoId, TopoId)> {
    let arc = curves.iter_mut().find(|arc| arc.from == bottom || arc.to == bottom)?;
    let end = *arc;
    arc.to = cross;
    curves.push(CurveSpec { from: cross, ..end });
    Some((end.from, end.to, cross))
}

/// Split the cylindrical edge and trim the buried part from the adjacent strip side.
pub(super) fn trim_strip_face(face: &mut FaceSpec, splits: &[(TopoId, TopoId, TopoId)], roots: &[(TopoId, TopoId)]) {
    let boundary = &mut face.loops[0];
    for &(from, to, cross) in splits {
        if let Some(i) = (0..boundary.len()).find(|&i| {
            let edge = [boundary[i], boundary[(i + 1) % boundary.len()]];
            edge == [from, to] || edge == [to, from]
        }) {
            boundary.insert(i + 1, cross);
        }
    }
    for &(top, bottom) in roots {
        if boundary.contains(&top) {
            boundary.retain(|&v| v != bottom);
        }
    }
}

pub(super) fn wall_faces(
    wall: &Body,
    strip: &Body,
    face: TopoId,
    t: f64,
    to_world: Rigid,
    [sides, [start, end]]: [[f64; 2]; 2],
    bad: impl Fn() -> BrepError,
) -> Result<(usize, usize), BrepError> {
    let local = Rigid::into_frame(
        to_world.apply(Vec3::ZERO),
        to_world.apply_vector(Vec3::new(1.0, 0.0, 0.0)),
        to_world.apply_vector(Vec3::new(0.0, 1.0, 0.0)),
        to_world.apply_vector(Vec3::new(0.0, 0.0, 1.0)),
    );
    let (lo, hi) = (0..strip.faces().len() as Ix)
        .map(|i| strip.face_bounds(i, &local))
        .fold((Vec3::new(start, sides[0], -t), Vec3::new(end, sides[1], 0.0)), |(lo, hi), (a, b)| {
            (Vec3::new(lo.x.min(a.x), lo.y.min(a.y), lo.z.min(a.z)), Vec3::new(hi.x.max(b.x), hi.y.max(b.y), hi.z.max(b.z)))
        });
    wall_faces_in(wall, face, t, to_world, [[sides[0], sides[1]], [start, end]], (lo, hi), bad)
}

pub(super) fn wall_faces_in(
    wall: &Body,
    face: TopoId,
    t: f64,
    to_world: Rigid,
    [sides, [start, end]]: [[f64; 2]; 2],
    (lo, hi): (Vec3, Vec3),
    bad: impl Fn() -> BrepError,
) -> Result<(usize, usize), BrepError> {
    let top = wall.faces().iter().position(|f| f.id == face).ok_or_else(&bad)?;
    let o = to_world.apply(Vec3::ZERO);
    let x = to_world.apply_vector(Vec3::new(1.0, 0.0, 0.0));
    let y = to_world.apply_vector(Vec3::new(0.0, 1.0, 0.0));
    let z = to_world.apply_vector(Vec3::new(0.0, 0.0, 1.0));
    let frame = Frame::Plane { o, x, y, z };
    let at = |a: f64, b: f64, c: f64| to_world.apply(Vec3::new(a, b, c));
    let size = wall
        .vertices()
        .iter()
        .fold(t.max(o.x.abs()).max(o.y.abs()).max(o.z.abs()), |s, v| s.max(v.point.x.abs()).max(v.point.y.abs()).max(v.point.z.abs()));
    let top_plane = wall.faces()[top].surface.plane().ok_or_else(&bad)?;
    let top_normal = top_plane.normal * (1.0 / top_plane.normal.dot(top_plane.normal).sqrt());
    if top_normal.dot(z) <= 0.0 || !nearly_parallel(top_normal, z) || !within_rounding((o - top_plane.origin).dot(top_normal), size) {
        return Err(bad());
    }
    let ring = rectangle(Vec2::new(start, sides[0]), Vec2::new(end, sides[1]));
    let holds = |f| {
        let loops = crate::cut::outline(wall, f, &frame)?;
        Ok::<_, BrepError>(
            !loops.iter().any(|l| ring.touches(l) || ring.winding(l.points[0]) != 0)
                && loops.iter().map(|l| l.winding(ring.points[0])).sum::<i32>() != 0,
        )
    };
    if !holds(top as Ix)? {
        return Err(bad());
    }
    let inside = at(0.0, 0.0, -t);
    let mut bottom = None;
    for (i, f) in wall.faces().iter().enumerate() {
        let Some(p) = f.surface.plane().filter(|p| p.normal.dot(z) < 0.0 && nearly_parallel(p.normal, z)) else { continue };
        let normal = p.normal * (1.0 / p.normal.dot(p.normal).sqrt());
        if within_rounding((inside - p.origin).dot(normal), size) && holds(i as Ix)? && bottom.replace(i).is_some() {
            return Err(bad());
        }
    }
    let bottom = bottom.ok_or_else(&bad)?;
    // Check the volume swept by the window and raised strip against other faces. Where folding
    // turns a curved boundary away from the frame, its enclosing box must establish clearance.
    let local = Rigid::into_frame(o, x, y, z);
    let footprint = rectangle(Vec2::new(lo.x, lo.y), Vec2::new(hi.x, hi.y));
    for (i, f) in wall.faces().iter().enumerate() {
        if i == top || i == bottom || !crate::bounds::overlap((lo, hi), wall.face_bounds(i as Ix, &local)) {
            continue;
        }
        let exact = match &f.surface {
            Surface::Plane(p) => {
                nearly_parallel(p.normal, z)
                    || wall
                        .face_loops(i as Ix)
                        .flat_map(|l| wall.loop_half_edges(l))
                        .all(|h| wall.edge_curve(&wall.edges()[wall.half_edges()[h as usize].edge as usize]).is_none())
            }
            Surface::Cylinder(c) => nearly_parallel(c.axis, z),
            _ => false,
        };
        if !exact {
            return Err(bad());
        }
        let loops = crate::cut::outline(wall, i as Ix, &frame)?;
        if loops.iter().any(|l| footprint.touches(l) || footprint.winding(l.points[0]) != 0)
            || loops.iter().map(|l| l.winding(footprint.points[0])).sum::<i32>() != 0
        {
            return Err(bad());
        }
    }
    Ok((top, bottom))
}

fn rectangle(lo: Vec2, hi: Vec2) -> ProfileLoop {
    ProfileLoop { points: vec![lo, Vec2::new(hi.x, lo.y), hi, Vec2::new(lo.x, hi.y)], vertices: Vec::new(), sides: Vec::new(), arcs: vec![None; 4] }
}
