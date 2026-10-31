//! Flanges of sheet metal (M8, D205): on an end face of a sheet (an end of a sheet along a path,
//! an edge of a face of sheet metal), a bend of the rule's inner radius and a wall beyond it,
//! made directly: the new faces are sewn to the end face's own vertices, which it gives up. A
//! flange's side lies in the plane of the face beside the end where that face is square to the
//! edge (a sheet's side, a plate's square corner): it joins that face, as a bend's side is part of
//! the sheet's side face.

use std::collections::BTreeSet;

use datum_brep::{Arc, Body, BrepError, Curve, CurveSpec, Cylinder, FaceSpec, Ix, Plane, Surface, TopoId, Vertex, build_with_curves};
use datum_math::Vec3;

use crate::sketch::{find_face, parallel};

/// What a flange is: its height from the outer surface of the sheet (for a right angle; else
/// along the flange from where its outer surface meets the sheet's), the angle it turns by in
/// degrees, its bend's inner radius, and the side it turns to (`flip`: the other).
#[derive(Clone, Copy, Debug)]
pub struct FlangeDims {
    pub height: f64,
    pub angle: f64,
    pub radius: f64,
    pub flip: bool,
}

/// Flanges on end faces `faces` of sheet metal `target`, one after another.
pub fn flanges(feature: TopoId, target: &Body, faces: &[String], dims: FlangeDims) -> Result<Body, BrepError> {
    if faces.is_empty() {
        return Err(BrepError::Sketch("a flange takes the end faces it stands on: `faces`".into()));
    }
    if !(dims.angle > 0.0 && dims.angle < 180.0) {
        return Err(BrepError::Degenerate(format!("a flange turns by more than 0° and less than 180°, got {}", dims.angle)));
    }
    if !(dims.radius > 0.0 && dims.height > 0.0 && dims.radius.is_finite() && dims.height.is_finite()) {
        return Err(BrepError::Degenerate("a flange needs a positive height and bend radius".into()));
    }
    // Ends meeting at a corner would need the corner relieved.
    let ids: Vec<TopoId> = faces.iter().map(|f| find_face(target, f).map(|(_, id)| id)).collect::<Result<_, _>>()?;
    let corners = |b: &Body, id: TopoId| -> BTreeSet<Ix> {
        let f = b.faces().binary_search_by_key(&id, |f| f.id).map_or(Ix::MAX, |f| f as Ix);
        b.face_loops(f).flat_map(|l| b.loop_half_edges(l)).map(|h| b.half_edges()[h as usize].origin).collect()
    };
    for (i, a) in ids.iter().enumerate() {
        for b in &ids[i + 1..] {
            if a == b || !corners(target, *a).is_disjoint(&corners(target, *b)) {
                return Err(BrepError::Unsupported("flanges on ends that meet at a corner (relieving the corner) come later".into()));
            }
        }
    }
    let mut body = target.clone();
    for id in ids {
        body = flange(feature, &body, id, dims)?;
    }
    Ok(body)
}

/// The faces of sheet metal `body` a flange may stand on: across the thickness (not on the
/// sheet's two sides), flat, four edges, the long two on parallel flat sides of the sheet. What
/// the window lights up for a flange; the flange itself checks the same.
pub fn flange_ends(body: &Body) -> Vec<TopoId> {
    let Ok(sides) = crate::unfold::sheet_sides(body) else { return Vec::new() };
    let on_sides: BTreeSet<Ix> = sides.iter().filter_map(|&s| crate::unfold::chain_through(body, s).ok()).flatten().flatten().collect();
    (0..body.faces().len() as Ix).filter(|x| !on_sides.contains(x) && end_of(body, *x).is_some()).map(|x| body.faces()[x as usize].id).collect()
}

/// The edges a flange is picked by, as in Inventor (D205): each end's two long edges, where it
/// meets the sheet's sides, with the end they belong to.
pub fn flange_edges(body: &Body) -> Vec<(TopoId, [Vec3; 2])> {
    let he = |h: Ix| body.half_edges()[h as usize];
    flange_ends(body)
        .into_iter()
        .filter_map(|id| {
            let f = body.faces().binary_search_by_key(&id, |f| f.id).ok()? as Ix;
            let (long, _) = end_of(body, f)?;
            Some(long.map(|h| (id, [body.point(he(h).origin), body.point(he(body.next(h)).origin)])))
        })
        .flatten()
        .collect()
}

/// Face `f` of `body` as an end a flange stands on: its two long half-edges and two short ones;
/// none if it is not one.
fn end_of(body: &Body, f: Ix) -> Option<([Ix; 2], [Ix; 2])> {
    let he = |h: Ix| body.half_edges()[h as usize];
    let face = &body.faces()[f as usize];
    if !matches!(face.surface, Surface::Plane(_)) || face.loop_count != 1 {
        return None;
    }
    let halves: Vec<Ix> = body.loop_half_edges(face.first_loop).collect();
    if halves.len() != 4 {
        return None;
    }
    let length = |h: Ix| {
        let d = body.point(he(body.next(h)).origin) - body.point(he(h).origin);
        d.dot(d)
    };
    let k = (0..4).max_by(|&a, &b| length(halves[a]).total_cmp(&length(halves[b])))?;
    let (long, short) = ([halves[k], halves[(k + 2) % 4]], [halves[(k + 1) % 4], halves[(k + 3) % 4]]);
    let plane = |h: Ix| match body.faces()[body.loops()[he(he(h).twin).loop_ix as usize].face as usize].surface {
        Surface::Plane(p) => Some(p.normal),
        _ => None,
    };
    parallel(plane(long[0])?, plane(long[1])?).then_some((long, short))
}

fn unit(v: Vec3) -> Vec3 {
    v * (1.0 / v.dot(v).sqrt())
}

/// `v` turned by `angle` (radians, as sine and cosine) about unit `axis`.
fn turned(v: Vec3, axis: Vec3, (sn, cs): (f64, f64)) -> Vec3 {
    v * cs + axis.cross(v) * sn + axis * (axis.dot(v) * (1.0 - cs))
}

/// One flange on end face `id` of `target`.
fn flange(feature: TopoId, target: &Body, id: TopoId, dims: FlangeDims) -> Result<Body, BrepError> {
    let unsupported = |m: &str| BrepError::Unsupported(format!("flange: {m}"));
    let f = target.faces().binary_search_by_key(&id, |f| f.id).map_err(|_| BrepError::LostReference(format!("face {id:?} is not in the body")))?;
    let f = f as Ix;
    let he = |h: Ix| target.half_edges()[h as usize];
    let face_of = |h: Ix| target.loops()[he(h).loop_ix as usize].face;
    let Surface::Plane(end) = target.faces()[f as usize].surface else { return Err(unsupported("it stands on a flat end face")) };
    // The two long edges lie on the sheet's two sides, parallel planes; the short ones across
    // the thickness, on the faces beside the end.
    let (long, short) = end_of(target, f).ok_or_else(|| {
        unsupported("it stands on an end of the sheet: flat, four edges, the long ones on its flat sides (a rolled end comes later)")
    })?;
    let beside = |h: Ix| face_of(he(h).twin);
    let plane = |x: Ix| match target.faces()[x as usize].surface {
        Surface::Plane(p) => Some(p),
        _ => None,
    };
    let n = unit(end.normal);
    // The side the flange turns to: toward the front side of the sheet, or the other.
    let front = crate::unfold::sheet_sides(target).ok().map(|s| crate::unfold::chain_through(target, s[0])).and_then(Result::ok);
    let on_front = |x: Ix| front.as_ref().is_some_and(|c| c.iter().flatten().any(|&y| y == x));
    let inner = usize::from(!on_front(beside(long[0])) == !dims.flip);
    let (hi, ho) = (long[inner], long[1 - inner]);
    let up = unit(match plane(beside(hi)) {
        Some(p) => p.normal,
        None => return Err(unsupported("the sheet's sides at the end are flat")),
    });
    // Each long edge's ends, at the start of the width and at its end: the inner one's way.
    let (i0, i1) = (he(hi).origin, he(target.next(hi)).origin);
    let w = unit(target.point(i1) - target.point(i0));
    // The outer edge runs the other way round the face.
    let (o1, o0) = (he(ho).origin, he(target.next(ho)).origin);
    let t = (target.point(i0) - target.point(o0)).dot(up);
    if t.is_nan() || t <= 0.0 {
        return Err(BrepError::Invalid("flange: the end's inner edge is not inside the sheet's thickness".into()));
    }
    let (r, a) = (dims.radius, dims.angle.to_radians());
    let turn = libm::sincos(a);
    // Turning `n` toward `up`: about `w` or `-w`.
    let axis = if w.cross(n).dot(up) > 0.0 { w } else { w * -1.0 };
    let along = turned(n, axis, turn);
    // At a right angle exactly `r + t`: tan 45° in floating point is not quite 1, and a flange as
    // high as its bend would come out a sliver instead of refused.
    let set_back = (r + t) * if dims.angle == 90.0 { 1.0 } else { libm::tan(0.5 * a) };
    let wall = dims.height - set_back;
    if wall.is_nan() || wall <= 0.0 {
        return Err(BrepError::Degenerate(format!("a flange {} high is no higher than its bend ({set_back:.3})", dims.height)));
    }
    let id = |tag: &str, k: u64| TopoId::derive(tag).id(feature).id(target.faces()[f as usize].id).index(k).finish();
    // From each corner of the end, about the bend's axis `up · radius` away: where the bend ends,
    // and the wall's end beyond it.
    let mut vertices: Vec<Vertex> = target.vertices().to_vec();
    let mut corner = |base: Ix, radius: f64, k: u64| -> [TopoId; 3] {
        let p = target.point(base);
        let c = p + up * radius;
        let q = c + turned(p - c, axis, turn);
        let (vq, ve) = (id("flange.v", k), id("flange.v", k + 1));
        vertices.push(Vertex { id: vq, point: q });
        vertices.push(Vertex { id: ve, point: q + along * wall });
        [target.vertices()[base as usize].id, vq, ve]
    };
    // Inner and outer, at the start and end of the width.
    let [ii0, ji0, ki0] = corner(i0, r, 0);
    let [ii1, ji1, ki1] = corner(i1, r, 2);
    let [oo0, po0, qo0] = corner(o0, r + t, 4);
    let [oo1, po1, qo1] = corner(o1, r + t, 6);
    let point = |v: TopoId| vertices.iter().find(|x| x.id == v).map_or(Vec3::ZERO, |x| x.point);
    // A loop of `ids` turning about `outward` by the right hand.
    let oriented = |mut ids: Vec<TopoId>, outward: Vec3| -> Vec<TopoId> {
        let p: Vec<Vec3> = ids.iter().map(|&v| point(v)).collect();
        let normal = (0..p.len()).fold(Vec3::ZERO, |s, k| s + p[k].cross(p[(k + 1) % p.len()]));
        if normal.dot(outward) < 0.0 {
            ids.reverse();
        }
        ids
    };
    let c_in = target.point(i0) + up * r;
    let c_out = target.point(o0) + up * (r + t);
    let in_mid = c_in + turned(target.point(i0) - c_in, axis, libm::sincos(0.5 * a)) - c_in;
    let inner_normal = turned(up, axis, turn);
    let mut faces: Vec<FaceSpec> = Vec::new();
    let face = |tag: &str, surface: Surface, loop_: Vec<TopoId>| FaceSpec { id: id(tag, 0), surface, loops: vec![loop_] };
    faces.push(face(
        "flange.bend_in",
        Surface::Cylinder(Cylinder { origin: c_in, axis: w, radius: r, convex: false }),
        oriented(vec![ii0, ii1, ji1, ji0], in_mid * -1.0),
    ));
    faces.push(face(
        "flange.bend_out",
        Surface::Cylinder(Cylinder { origin: c_out, axis: w, radius: r + t, convex: true }),
        oriented(vec![oo0, oo1, po1, po0], in_mid),
    ));
    faces.push(face(
        "flange.wall_in",
        Surface::Plane(Plane { origin: point(ji0), normal: inner_normal }),
        oriented(vec![ji0, ji1, ki1, ki0], inner_normal),
    ));
    faces.push(face(
        "flange.wall_out",
        Surface::Plane(Plane { origin: point(po0), normal: inner_normal * -1.0 }),
        oriented(vec![po0, po1, qo1, qo0], inner_normal * -1.0),
    ));
    faces.push(face("flange.end", Surface::Plane(Plane { origin: point(ki0), normal: along }), oriented(vec![ki0, ki1, qo1, qo0], along)));
    // The arcs of the bend at both sides, inner and outer, from the end face on.
    let arc = |c: Vec3, r: f64, from: TopoId, to: TopoId| {
        // Counterclockwise about `axis` from the end face.
        CurveSpec { from, to, curve: Curve::Arc(Arc { center: c, axis, radius: r }) }
    };
    let mut curves = target.curve_specs();
    let c_in1 = c_in + w * (target.point(i1) - target.point(i0)).dot(w);
    let c_out1 = c_out + w * (target.point(o1) - target.point(o0)).dot(w);
    curves.extend([arc(c_in, r, ii0, ji0), arc(c_in1, r, ii1, ji1), arc(c_out, r + t, oo0, po0), arc(c_out1, r + t, oo1, po1)]);
    // The sides: into the face beside the end where that face is square to the width, else faces
    // of their own.
    let mut gone: BTreeSet<Ix> = BTreeSet::from([f]);
    let starts = |h: Ix| he(h).origin == i0 || he(target.next(h)).origin == i0;
    let short = if starts(short[0]) { short } else { [short[1], short[0]] };
    for (s, (h, chain)) in short.iter().zip([[ii0, ji0, ki0, qo0, po0, oo0], [ii1, ji1, ki1, qo1, po1, oo1]]).enumerate() {
        let x = beside(*h);
        let outward = if s == 0 { w * -1.0 } else { w };
        let joins = plane(x).is_some_and(|p| parallel(p.normal, w));
        let chain = chain.to_vec();
        if !joins {
            faces.push(FaceSpec {
                id: id("flange.side", s as u64),
                surface: Surface::Plane(Plane { origin: point(chain[0]), normal: outward }),
                loops: vec![oriented(chain, outward)],
            });
            continue;
        }
        // The side face's edge across the end runs from one end of the short edge to the other:
        // the flange's outline goes in between, the same way.
        let mut spec = target.face_spec(x);
        let (a, b) = (target.vertices()[he(he(*h).twin).origin as usize].id, target.vertices()[he(target.next(he(*h).twin)).origin as usize].id);
        let run: Vec<TopoId> =
            if chain[0] == a { chain[1..chain.len() - 1].to_vec() } else { chain[1..chain.len() - 1].iter().rev().copied().collect() };
        for l in &mut spec.loops {
            if let Some(k) = (0..l.len()).find(|&k| l[k] == a && l[(k + 1) % l.len()] == b) {
                for (j, v) in run.iter().enumerate() {
                    l.insert(k + 1 + j, *v);
                }
            }
        }
        faces.push(spec);
        gone.insert(x);
    }
    faces.extend((0..target.faces().len() as Ix).filter(|x| !gone.contains(x)).map(|x| target.face_spec(x)));
    build_with_curves(vertices, faces, curves)
}
