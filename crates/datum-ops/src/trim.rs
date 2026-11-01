//! Trimming rolled sheet metal by a plane (D211): the part on one side of a work plane goes, as
//! a laser cuts the blank before rolling. The cut follows where the plane meets the sheet's
//! neutral layer, a wave in the development; its wall stands square to the sheet, along the
//! radius. For now the sheet is a shell (one cylinder each side) and the plane crosses it from
//! seam to seam, cutting off one end whole.

use std::collections::BTreeMap;

use datum_brep::{Body, BrepError, Curve, CurveSpec, FaceSpec, Rolled, Surface, TopoId, Vertex, Wave, build_with_curves};
use datum_math::{Sign, Vec2, Vec3, orient3d_coords};

use crate::sketch::{SketchPlane, unrolled};

fn unsupported(m: &str) -> BrepError {
    BrepError::Unsupported(format!("trim: {m}"))
}

/// `body`, a rolled shell, trimmed by `plane`: what lies on the side its normal points to goes
/// (`flip`: the other side). `k`: the K-factor of the sheet's neutral layer.
pub(crate) fn trim(feature: TopoId, body: &Body, plane: &SketchPlane, flip: bool, k: f64) -> Result<Body, BrepError> {
    let faces = body.faces();
    if faces.iter().any(|f| matches!(f.surface, Surface::Cone(_))) {
        return Err(unsupported("a cone comes later"));
    }
    let cylinders: Vec<usize> = (0..faces.len()).filter(|&f| matches!(faces[f].surface, Surface::Cylinder(_))).collect();
    let [first, second] = cylinders[..] else { return Err(unsupported("the body is no rolled shell: one cylinder each side of the sheet")) };
    let (Surface::Cylinder(c), Surface::Cylinder(c2)) = (faces[first].surface, faces[second].surface) else {
        return Err(unsupported("no cylinder"));
    };
    let d = unrolled(body, first, &c, k)?;
    let dev = d.dev;
    let n_radius = dev.neutral;
    let [o, x, y, n] = plane.axes()?;
    let along = n.dot(dev.axis);
    if along == 0.0 {
        return Err(unsupported("a plane along the axis cuts the sheet lengthwise: comes later"));
    }
    // The plane on the neutral layer: v(u) where n·(origin + axis·v + N·radial(u)) = n·o.
    let wave =
        Wave { a: n.dot(o - dev.origin) / along, b: -n_radius * n.dot(dev.start) / along, c: -n_radius * n.dot(dev.axis.cross(dev.start)) / along };
    let near = 0.5 * d.span * n_radius;
    let uv = |p: Vec3| dev.unroll(p, near);
    let span = [0.0, d.span * n_radius];
    let ends = body.vertices().iter().map(|v| uv(v.point).0.y).fold([f64::INFINITY, f64::NEG_INFINITY], |[lo, hi], v| [lo.min(v), hi.max(v)]);
    let [low, high] = wave.range(span[0], span[1], n_radius);
    if low <= ends[0] || high >= ends[1] {
        return Err(unsupported(if high < ends[0] || low > ends[1] {
            "the plane misses the sheet"
        } else {
            "the plane leaves the sheet through its end: comes later"
        }));
    }
    // Each vertex's side, exactly; the side the normal points to goes, unless flipped.
    let side = |p: Vec3| orient3d_coords(o, o + x, o + y, p);
    let mut gone = vec![false; body.vertices().len()];
    for (i, v) in body.vertices().iter().enumerate() {
        gone[i] = match side(v.point) {
            Sign::Zero => return Err(BrepError::Degenerate("trim: a corner of the sheet lies on the plane".into())),
            s => (s == Sign::Negative) != flip,
        };
    }
    // The end that goes is wholly on the far side of the wave: `+v` or `-v` of it.
    let up = {
        let far = body.vertices().iter().enumerate().find(|(i, _)| gone[*i]).map(|(_, v)| uv(v.point).0.y);
        far.ok_or_else(|| unsupported("nothing of the sheet is on the side to cut off"))? > high
    };
    let index = |id: TopoId| body.vertices().binary_search_by_key(&id, |v| v.id).map_err(|_| BrepError::Invalid(format!("trim: vertex {id:?}")));
    // Holes and cuts stay clear of the wave, or the trim is refused.
    for (f, face) in faces.iter().enumerate() {
        let Surface::Wall(w) = face.surface else { continue };
        let ids: Vec<TopoId> = body.face_spec(f as u32).loops.concat();
        let all_gone = ids.iter().map(|&id| index(id).map(|i| gone[i])).collect::<Result<Vec<_>, _>>()?;
        if all_gone.iter().any(|&g| g) {
            return Err(unsupported("a cut in the part cut off, or across the plane: comes later"));
        }
        let rho = w.arc.map_or(0.0, |a| libm::hypot(w.from.x - a.center.x, w.from.y - a.center.y));
        let (cu, cv) = w.arc.map_or((w.from.x, w.from.y), |a| (a.center.x, a.center.y));
        let us = [w.from.x.min(w.to.x).min(cu - rho), w.from.x.max(w.to.x).max(cu + rho)];
        let vs = [w.from.y.min(w.to.y).min(cv - rho), w.from.y.max(w.to.y).max(cv + rho)];
        let [lo, hi] = wave.range(us[0], us[1], n_radius);
        if (up && vs[1] >= lo) || (!up && vs[0] <= hi) {
            return Err(unsupported("a cut reaches the plane's line: comes later"));
        }
    }
    let point = |u: f64, r: f64| dev.point(Vec2::new(u, wave.at(u, n_radius)[0]), r);
    // Where an edge from a kept vertex to a gone one (along the axis) meets the wave.
    let mut crossings: BTreeMap<(TopoId, TopoId), (TopoId, f64, f64)> = BTreeMap::new();
    let mut vertices: Vec<Vertex> = body.vertices().iter().enumerate().filter(|(i, _)| !gone[*i]).map(|(_, v)| *v).collect();
    let mut cross = |a: TopoId, b: TopoId| -> Result<TopoId, BrepError> {
        let key = (a.min(b), a.max(b));
        if let Some(&(id, ..)) = crossings.get(&key) {
            return Ok(id);
        }
        let kept = if gone[index(a)?] { b } else { a };
        // The edge runs along a seam, on the inside or the outside: the seam's `u` and that
        // face's radius, as the shell has them, not as unrolling the vertex rounds them.
        let (p, r) = uv(body.vertices()[index(kept)?].point);
        let u = if (p.x - span[0]).abs() <= (p.x - span[1]).abs() { span[0] } else { span[1] };
        let r = if (r - c.radius).abs() <= (r - c2.radius).abs() { c.radius } else { c2.radius };
        let p = Vec2::new(u, p.y);
        let id = TopoId::derive("trim.v").id(feature).id(key.0).id(key.1).finish();
        vertices.push(Vertex { id, point: point(p.x, r) });
        crossings.insert(key, (id, p.x, r));
        Ok(id)
    };
    let mut specs = Vec::with_capacity(faces.len() + 1);
    // The new edges each face got along the cut, as it runs them: its surface, from, to.
    let mut cut_edges: Vec<(Surface, TopoId, TopoId)> = Vec::new();
    for f in 0..faces.len() as u32 {
        let mut spec = body.face_spec(f);
        let mut loops = Vec::with_capacity(spec.loops.len());
        for (k, l) in spec.loops.iter().enumerate() {
            let out: Vec<bool> = l.iter().map(|&id| index(id).map(|i| gone[i])).collect::<Result<_, _>>()?;
            if out.iter().all(|&g| g) {
                if k == 0 {
                    break;
                }
                continue;
            }
            if !out.iter().any(|&g| g) {
                loops.push(l.clone());
                continue;
            }
            let m = l.len();
            let mut ring = Vec::with_capacity(m);
            // Into the run cut off, and out of a run begun before the loop's start.
            let (mut entry, mut first_exit) = (None, None);
            for i in 0..m {
                let (a, b) = (l[i], l[(i + 1) % m]);
                if !out[i] {
                    ring.push(a);
                }
                if out[i] != out[(i + 1) % m] {
                    let c = cross(a, b)?;
                    ring.push(c);
                    match (out[i], entry.take()) {
                        (false, _) => entry = Some(c),
                        (true, Some(e)) => cut_edges.push((spec.surface, e, c)),
                        (true, None) => first_exit = Some(c),
                    }
                }
            }
            if let (Some(e), Some(c)) = (entry, first_exit) {
                cut_edges.push((spec.surface, e, c));
            }
            loops.push(ring);
        }
        if loops.is_empty() {
            continue;
        }
        spec.loops = loops;
        specs.push(spec);
    }
    // The wall along the cut: each new edge back, chained.
    let mut next: BTreeMap<TopoId, TopoId> = cut_edges.iter().map(|&(_, a, b)| (b, a)).collect();
    let start = *next.keys().next().ok_or_else(|| BrepError::Invalid("trim: no cut".into()))?;
    let mut wall = vec![start];
    while let Some(v) = next.remove(wall.last().unwrap_or(&start)) {
        if v == start {
            break;
        }
        wall.push(v);
    }
    if !next.is_empty() || wall.len() != cut_edges.len() {
        return Err(BrepError::Invalid("trim: the cut does not close".into()));
    }
    // The wall's piece runs with the part cut off on its left (D68): `+u` if it is `+v`.
    let piece = |u0: f64, u1: f64| Rolled {
        dev,
        from: Vec2::new(u0, wave.at(u0, n_radius)[0]),
        to: Vec2::new(u1, wave.at(u1, n_radius)[0]),
        arc: None,
        wave: Some(wave),
    };
    let wall_piece = if up { piece(span[0], span[1]) } else { piece(span[1], span[0]) };
    specs.push(FaceSpec { id: TopoId::derive("trim.wall").id(feature).finish(), surface: Surface::Wall(wall_piece), loops: vec![wall] });
    // Curves: the body's, but those of edges cut off; the wave on each cylinder.
    let kept = |id: TopoId| index(id).is_ok_and(|i| !gone[i]);
    let mut curves: Vec<CurveSpec> = body.curve_specs().into_iter().filter(|s| kept(s.from) && kept(s.to)).collect();
    let at = |id: TopoId| crossings.values().find(|c| c.0 == id).map(|c| (c.1, c.2));
    for (surface, a, b) in &cut_edges {
        if let (Surface::Cylinder(_), Some((u0, r)), Some((u1, _))) = (surface, at(*a), at(*b)) {
            curves.push(CurveSpec { from: *a, to: *b, curve: Curve::Rolled { piece: piece(u0, u1), radius: r } });
        }
    }
    build_with_curves(vertices, specs, curves)
}
