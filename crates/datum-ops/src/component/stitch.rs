use datum_brep::{Body, BrepError, FaceSpec, Ix, Plane, ProfileLoop, Surface, TopoId, Vertex, build_with_curves};
use datum_math::{Rigid, Sign, Vec2, Vec3, orient3d_coords};

use crate::cut::{Frame, outline};
use crate::sketch::{nearly_parallel, within_rounding};

#[derive(Clone, Copy)]
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
}

/// Join a raised strip to a simple flat plate, leaving its unfolded window in the plate.
pub(super) fn stitch(feature: TopoId, wall: &Body, strip: &Body, window: &Window) -> Result<Body, BrepError> {
    let Window { face, thickness: t, root_cap: root, root_ids, root_cross, to_world, sides, inner_root, outer_root, far } = *window;
    let bad = || BrepError::Unsupported("stamp: the tongue needs a clear window in a flat plate".into());
    if !t.is_finite()
        || t <= 0.0
        || !sides.iter().all(|v| v.is_finite())
        || sides[0] >= sides[1]
        || !outer_root.is_finite()
        || !inner_root.is_finite()
        || !far.is_finite()
        || outer_root > inner_root
        || inner_root >= far
    {
        return Err(bad());
    }
    // Only a single six-faced, straight-edged plate is supported. In particular there can be no
    // neighbouring face hidden inside the window or another wall at the same depth.
    if wall.faces().len() != 6
        || wall.faces().iter().any(|f| f.loop_count != 1 || f.surface.plane().is_none())
        || wall.edges().iter().any(|e| wall.edge_curve(e).is_some())
    {
        return Err(bad());
    }
    let top = wall.faces().iter().position(|f| f.id == face).ok_or_else(bad)?;
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
    let top_plane = wall.faces()[top].surface.plane().ok_or_else(bad)?;
    let top_normal = top_plane.normal * (1.0 / top_plane.normal.dot(top_plane.normal).sqrt());
    if top_normal.dot(z) <= 0.0 || !nearly_parallel(top_normal, z) || !within_rounding((o - top_plane.origin).dot(top_normal), size) {
        return Err(bad());
    }
    let bottom = wall
        .faces()
        .iter()
        .enumerate()
        .find(|(i, f)| *i != top && f.surface.plane().is_some_and(|p| p.normal.dot(z) < 0.0 && nearly_parallel(p.normal, z)))
        .map(|(i, _)| i)
        .ok_or_else(bad)?;
    let bottom_plane = wall.faces()[bottom].surface.plane().ok_or_else(bad)?;
    let bottom_normal = bottom_plane.normal * (1.0 / bottom_plane.normal.dot(bottom_plane.normal).sqrt());
    if !within_rounding((at(0.0, 0.0, -t) - bottom_plane.origin).dot(bottom_normal), size) {
        return Err(bad());
    }
    // The four other faces must bound a convex body and the whole window prism must lie
    // strictly inside them. Exact predicates decide clearance, including edge contact.
    for f in 0..wall.faces().len() {
        if f == top || f == bottom {
            continue;
        }
        let lp = wall.faces()[f].first_loop;
        let ids: Vec<_> = wall.loop_half_edges(lp).map(|h| wall.half_edges()[h as usize].origin).collect();
        let pts: Vec<_> = ids.iter().map(|&v| wall.point(v)).collect();
        let triangle = (1..pts.len())
            .find_map(|i| {
                (i + 1..pts.len()).find_map(|j| {
                    let n = (pts[i] - pts[0]).cross(pts[j] - pts[0]);
                    (n != Vec3::ZERO).then_some((pts[0], pts[i], pts[j]))
                })
            })
            .ok_or_else(bad)?;
        let side = |p| orient3d_coords(triangle.0, triangle.1, triangle.2, p);
        let signs: Vec<_> = wall.vertices().iter().enumerate().filter(|(i, _)| !ids.contains(&(*i as Ix))).map(|(_, v)| side(v.point)).collect();
        let inside = signs.iter().copied().find(|s| *s != Sign::Zero).ok_or_else(bad)?;
        if signs.iter().any(|s| *s != Sign::Zero && *s != inside) {
            return Err(bad());
        }
        for a in [outer_root, far] {
            for b in sides {
                for c in [0.0, -t] {
                    if side(at(a, b, c)) != inside {
                        return Err(bad());
                    }
                }
            }
        }
    }
    let ring = ProfileLoop {
        points: vec![Vec2::new(outer_root, sides[0]), Vec2::new(far, sides[0]), Vec2::new(far, sides[1]), Vec2::new(outer_root, sides[1])],
        vertices: Vec::new(),
        sides: Vec::new(),
        arcs: vec![None; 4],
    };
    for f in [top, bottom] {
        let outlines = outline(wall, f as Ix, &frame)?;
        if outlines.iter().map(|l| l.winding(ring.points[0])).sum::<i32>() == 0 || outlines.iter().any(|l| l.touches(&ring)) {
            return Err(bad());
        }
    }
    let cap = strip.faces().iter().position(|f| f.id == root).ok_or_else(bad)?;
    let names = ["top.left.tip", "top.right.tip", "bottom.left.root", "bottom.left.tip", "bottom.right.tip", "bottom.right.root"];
    let id = |k: usize| TopoId::derive("stamp.window.vertex").id(feature).tag(names[k]).finish();
    let mut vertices = wall.vertices().to_vec();
    vertices.extend_from_slice(strip.vertices());
    let positions = [
        (far, sides[0], 0.0),
        (far, sides[1], 0.0),
        (inner_root, sides[0], -t),
        (far, sides[0], -t),
        (far, sides[1], -t),
        (inner_root, sides[1], -t),
    ];
    for (k, &(a, b, c)) in positions.iter().enumerate() {
        if root_cross.is_some() && (k == 2 || k == 5) {
            continue;
        }
        vertices.push(Vertex { id: id(k), point: at(a, b, c) });
    }
    let mut faces: Vec<FaceSpec> = (0..wall.faces().len()).map(|i| wall.face_spec(i as Ix)).collect();
    let [top_left, top_right, bottom_left, bottom_right] = root_ids;
    let mut strip_curves = strip.curve_specs();
    let mut split_edges = Vec::new();
    let [upper_left, upper_right] = if let Some(x) = root_cross {
        let crosses = ["left", "right"].map(|side| TopoId::derive("stamp.window.cross").id(feature).tag(side).finish());
        for ((bottom, cross), y) in [bottom_left, bottom_right].into_iter().zip(crosses).zip(sides) {
            vertices.push(Vertex { id: cross, point: at(x, y, 0.0) });
            let arc = strip_curves.iter_mut().find(|arc| arc.from == bottom || arc.to == bottom).ok_or_else(bad)?;
            let end = *arc;
            split_edges.push((end.from, end.to, cross));
            arc.to = cross;
            strip_curves.push(datum_brep::CurveSpec { from: cross, ..end });
        }
        crosses
    } else {
        [bottom_left, bottom_right]
    };
    let [lower_left, lower_right] = if root_cross.is_some() { [bottom_left, bottom_right] } else { [id(2), id(5)] };
    faces[top].loops.push(vec![top_left, top_right, upper_right, id(1), id(0), upper_left]);
    faces[bottom].loops.push(vec![lower_left, id(3), id(4), lower_right]);
    faces.extend((0..strip.faces().len()).filter(|&i| i != cap).map(|i| {
        let mut face = strip.face_spec(i as Ix);
        if root_cross.is_some() {
            let boundary = &mut face.loops[0];
            for &(from, to, cross) in &split_edges {
                if let Some(i) = (0..boundary.len()).find(|&i| {
                    let edge = [boundary[i], boundary[(i + 1) % boundary.len()]];
                    edge == [from, to] || edge == [to, from]
                }) {
                    boundary.insert(i + 1, cross);
                }
            }
            // The part of either side cap below the plate's top is internal. The lower
            // part of the split arc belongs to the corresponding wall of the opening.
            for (top, bottom) in [(top_left, bottom_left), (top_right, bottom_right)] {
                if boundary.contains(&top) {
                    boundary.retain(|&v| v != bottom);
                }
            }
        }
        face
    }));
    let upper = [upper_left, id(0), id(1), upper_right];
    let lower = [lower_left, id(3), id(4), lower_right];
    let xy = [(inner_root, sides[0]), (far, sides[0]), (far, sides[1]), (inner_root, sides[1])];
    for k in 0..4 {
        if k == 3 && root_cross.is_some() {
            continue;
        }
        let next = (k + 1) % 4;
        let d = at(xy[next].0, xy[next].1, 0.0) - at(xy[k].0, xy[k].1, 0.0);
        faces.push(FaceSpec {
            id: TopoId::derive("stamp.window.face").id(feature).tag(["left", "tip", "right", "root"][k]).finish(),
            surface: Surface::Plane(Plane { origin: at(xy[k].0, xy[k].1, 0.0), normal: z.cross(d) }),
            loops: vec![vec![upper[k], upper[next], lower[next], lower[k]]],
        });
    }
    let mut curves = wall.curve_specs();
    curves.extend(strip_curves);
    build_with_curves(vertices, faces, curves)
}
