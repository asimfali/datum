//! Flanges of sheet metal (M8, D205): on an end face of a sheet (an end of a sheet along a path,
//! an edge of a face of sheet metal), a bend of the rule's inner radius and a wall beyond it,
//! made directly: the new faces are sewn to the end face's own vertices, which it gives up. A
//! flange's side lies in the plane of the face beside the end where that face is square to the
//! edge (a sheet's side, a plate's square corner): it joins that face, as a bend's side is part of
//! the sheet's side face.

use std::collections::BTreeSet;

use datum_brep::{Arc, Body, BrepError, Curve, CurveSpec, Cylinder, FaceSpec, Ix, Plane, Surface, TopoId, Vertex, build_with_curves};
use datum_math::Vec3;

use crate::sketch::parallel;

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

/// Flanges on edges `edges` of sheet metal `target`, one after another (D205): a long edge of an
/// end of the sheet puts it on that end, an edge of a wall on the sheet's side face puts it along
/// the wall (outward, no relief: it ends at the wall's bend lines). Two edges of one end or of
/// one wall's side make one flange; flanges meeting at a corner are refused.
pub fn flanges(feature: TopoId, target: &Body, edges: &[String], dims: FlangeDims) -> Result<Body, BrepError> {
    if edges.is_empty() {
        return Err(BrepError::Sketch("a flange takes the edges of the sheet it stands on: `edges`".into()));
    }
    if !(dims.angle > 0.0 && dims.angle < 180.0) {
        return Err(BrepError::Degenerate(format!("a flange turns by more than 0° and less than 180°, got {}", dims.angle)));
    }
    if !(dims.radius > 0.0 && dims.height > 0.0 && dims.radius.is_finite() && dims.height.is_finite()) {
        return Err(BrepError::Degenerate("a flange needs a positive height and bend radius".into()));
    }
    let ids: Vec<TopoId> = edges
        .iter()
        .map(|e| u128::from_str_radix(e, 16).map(TopoId).map_err(|_| BrepError::Sketch(format!("`{e}` is not an edge ID"))))
        .collect::<Result<_, _>>()?;
    // Each flange once, by the corners of what it stands on; none sharing a corner.
    let mut sites: Vec<(TopoId, [TopoId; 4])> = Vec::new();
    for &e in &ids {
        let base = base(target, e, dims.flip)?;
        let corners = [base.inner[0], base.inner[1], base.outer[0], base.outer[1]].map(|v| target.vertices()[v as usize].id);
        let mut sorted = corners;
        sorted.sort();
        if sites.iter().any(|(_, c)| *c == sorted) {
            continue;
        }
        if sites.iter().any(|(_, c)| c.iter().any(|v| sorted.contains(v))) {
            return Err(BrepError::Unsupported("flanges that meet at a corner (relieving the corner) come later".into()));
        }
        sites.push((e, sorted));
    }
    let mut body = target.clone();
    for (e, _) in sites {
        let base = base(&body, e, dims.flip)?;
        body = grow(feature, &body, base, dims)?;
    }
    Ok(body)
}

/// What a flange stands on: the edge of the sheet's side it turns toward (`inner`, at the start
/// and end of its width) and the one across the thickness (`outer`), the way it leaves the sheet
/// (`out`), the side it turns to (`up`), the key its IDs come from, the faces it replaces with
/// what stays of them, and at each end of the width the face its side joins, if any.
struct Base {
    key: TopoId,
    inner: [Ix; 2],
    outer: [Ix; 2],
    out: Vec3,
    up: Vec3,
    gone: Vec<Ix>,
    kept: Vec<FaceSpec>,
    joins: [Option<Ix>; 2],
}

/// The flange edge `e` of `body` asks for (see [`flanges`]).
fn base(body: &Body, e: TopoId, flip: bool) -> Result<Base, BrepError> {
    let unsupported = |m: &str| BrepError::Unsupported(format!("flange: {m}"));
    let he = |h: Ix| body.half_edges()[h as usize];
    let face_of = |h: Ix| body.loops()[he(h).loop_ix as usize].face;
    let k = body.edges().binary_search_by_key(&e, |x| x.id).map_err(|_| BrepError::LostReference(format!("edge {e:?} is not in the body")))?;
    let h = body.edges()[k].half;
    let plane = |x: Ix| match body.faces()[x as usize].surface {
        Surface::Plane(p) => Some(p),
        _ => None,
    };
    let skip = crate::unfold::branches(body);
    let sides: BTreeSet<Ix> = crate::unfold::sheet_sides(body)
        .ok()
        .into_iter()
        .flatten()
        .filter_map(|s| crate::unfold::chain_through(body, s, &skip).ok())
        .flatten()
        .flatten()
        .collect();
    // An end's long edge: a flange on that end (a face across the thickness, not of the sheet's
    // sides, which a wall is).
    for g in [h, he(h).twin] {
        let f = face_of(g);
        if !sides.contains(&f)
            && let Some((long, short)) = end_of(body, f)
            && long.contains(&g)
        {
            return end_base(body, f, long, short, flip);
        }
    }
    // A wall's edge on the sheet's side face: the strip of that face over the wall.
    let (wall_h, cap_h) = match (sides.contains(&face_of(h)), sides.contains(&face_of(he(h).twin))) {
        (true, false) => (h, he(h).twin),
        (false, true) => (he(h).twin, h),
        _ => return Err(unsupported("it stands on an edge of the sheet: of an end, or of a wall on the sheet's side")),
    };
    let (wall, cap) = (face_of(wall_h), face_of(cap_h));
    let (Some(pw), Some(pc)) = (plane(wall), plane(cap)) else { return Err(unsupported("it stands along a flat wall, on a flat side")) };
    if !side_face(body, cap, &sides) {
        return Err(unsupported("it stands on a wall's edge on the sheet's side face (one along its bends too, no hole)"));
    }
    // The wall's other side: the face along the side face parallel to it, nearest.
    let across = body
        .loop_half_edges(body.faces()[cap as usize].first_loop)
        .filter(|&g| g != cap_h)
        .filter_map(|g| {
            let x = face_of(he(g).twin);
            let p = plane(x)?;
            (x != wall && parallel(p.normal, pw.normal)).then(|| (g, (p.origin - pw.origin).dot(unit(pw.normal)).abs()))
        })
        .filter(|(_, d)| *d > 0.0)
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(g, _)| g)
        .ok_or_else(|| unsupported("the wall has no other side along the sheet's side face"))?;
    let (a0, a1) = (he(cap_h).origin, he(body.next(cap_h)).origin);
    let (b0, b1) = (he(across).origin, he(body.next(across)).origin);
    // The other side's ends, matched to this one's by nearness along the edge.
    let d = body.point(a1) - body.point(a0);
    let (b0, b1) = if (body.point(b0) - body.point(a0)).dot(d) < (body.point(b1) - body.point(a0)).dot(d) { (b0, b1) } else { (b1, b0) };
    // Outward: toward the side of the wall its bends turn away from (the outer, convex one);
    // a wall with no bend, toward the sheet's front.
    let convex = |x: Ix| crate::unfold::chain_through(body, x, &skip).is_ok() && smooth_convex(body, x);
    let wall_out = match (convex(wall), convex(face_of(he(across).twin))) {
        (true, _) => true,
        (false, true) => false,
        _ => true,
    };
    let toward_wall = wall_out != flip;
    let (inner, outer) = if toward_wall { ([a1, a0], [b1, b0]) } else { ([b1, b0], [a1, a0]) };
    let up_face = if toward_wall { wall } else { face_of(he(across).twin) };
    let up = unit(plane(up_face).map_or(pw.normal, |p| p.normal));
    // The side face without the strip: its loop cut at the wall's two edges, each piece closed
    // across the thickness; a piece of two vertices is an end of the sheet already there.
    let spec = body.face_spec(cap);
    let l = &spec.loops[0];
    let id = |v: Ix| body.vertices()[v as usize].id;
    let at = |x: Ix, y: Ix| (0..l.len()).find(|&k| (l[k], l[(k + 1) % l.len()]) == (id(x), id(y)));
    let (Some(ka), Some(kb)) = (at(a0, a1), at(he(across).origin, he(body.next(across)).origin)) else {
        return Err(BrepError::Invalid("flange: the side face does not run along the wall".into()));
    };
    let run = |from: usize, to: usize| -> Vec<TopoId> {
        let mut out = vec![l[from % l.len()]];
        let mut k = from % l.len();
        while k != to % l.len() {
            k = (k + 1) % l.len();
            out.push(l[k]);
        }
        out
    };
    let mut kept = Vec::new();
    let mut joins = [None, None];
    let strip = [id(a0), id(a1), id(b0), id(b1)];
    let first = l.iter().find(|v| !strip.contains(v)).copied();
    for piece in [run(ka + 1, kb), run(kb + 1, ka)] {
        if piece.len() == 2 {
            // The sheet's end across the thickness: the flange's side meets it there.
            let (x, y) = (piece[0], piece[1]);
            let g = body.loop_half_edges(body.faces()[cap as usize].first_loop).find(|&g| id(he(g).origin) == x && id(he(body.next(g)).origin) == y);
            if let Some(g) = g {
                let end = face_of(he(g).twin);
                let s = usize::from(x == id(inner[1]) || y == id(inner[1]));
                joins[s] = plane(end).filter(|p| parallel(p.normal, d)).map(|_| end);
            }
            continue;
        }
        let face = if first.is_some_and(|v| piece.contains(&v)) {
            spec.id
        } else {
            TopoId::derive("sheet.divided").id(spec.id).id(piece.iter().min().copied().unwrap_or(spec.id)).finish()
        };
        kept.push(FaceSpec { id: face, surface: spec.surface, loops: vec![piece] });
    }
    let key = TopoId::derive("flange.side").id(id(a0).min(id(b0))).id(id(a1).max(id(b1))).finish();
    Ok(Base { key, inner, outer, out: unit(pc.normal), up, gone: vec![cap], kept, joins })
}

/// Whether face `y` is a side face of the sheet: across its thickness, along its bends too (a cut's
/// wall in one wall is not), one loop. `sides` are the faces of the sheet's two sides.
fn side_face(body: &Body, y: Ix, sides: &BTreeSet<Ix>) -> bool {
    let he = |h: Ix| body.half_edges()[h as usize];
    let face = &body.faces()[y as usize];
    face.loop_count == 1
        && body.loop_half_edges(face.first_loop).any(|h| {
            let x = body.loops()[he(he(h).twin).loop_ix as usize].face;
            sides.contains(&x) && matches!(body.faces()[x as usize].surface, Surface::Cylinder(_))
        })
}

/// Whether face `x` of `body` goes on smoothly into a convex bend: the outer side of the sheet.
fn smooth_convex(body: &Body, x: Ix) -> bool {
    let he = |h: Ix| body.half_edges()[h as usize];
    body.loop_half_edges(body.faces()[x as usize].first_loop).any(|h| {
        let y = body.loops()[he(he(h).twin).loop_ix as usize].face;
        matches!(body.faces()[y as usize].surface, Surface::Cylinder(c) if c.convex)
            && body.faces()[x as usize].surface.normal_at(body.point(he(h).origin)).dot(body.faces()[y as usize].surface.normal_at(body.point(he(h).origin))) > 0.5
    })
}

/// A flange on end face `f`, its long half-edges `long` and short ones `short`: toward the
/// sheet's front side, or the other.
fn end_base(body: &Body, f: Ix, long: [Ix; 2], short: [Ix; 2], flip: bool) -> Result<Base, BrepError> {
    let unsupported = |m: &str| BrepError::Unsupported(format!("flange: {m}"));
    let he = |h: Ix| body.half_edges()[h as usize];
    let beside = |h: Ix| body.loops()[he(he(h).twin).loop_ix as usize].face;
    let plane = |x: Ix| match body.faces()[x as usize].surface {
        Surface::Plane(p) => Some(p),
        _ => None,
    };
    let Surface::Plane(end) = body.faces()[f as usize].surface else { return Err(unsupported("it stands on a flat end face")) };
    let skip = crate::unfold::branches(body);
    let front = crate::unfold::sheet_sides(body).ok().map(|s| crate::unfold::chain_through(body, s[0], &skip)).and_then(Result::ok);
    let on_front = |x: Ix| front.as_ref().is_some_and(|c| c.iter().flatten().any(|&y| y == x));
    let k = usize::from(!on_front(beside(long[0])) == !flip);
    let (hi, ho) = (long[k], long[1 - k]);
    let up = unit(plane(beside(hi)).ok_or_else(|| unsupported("the sheet's sides at the end are flat"))?.normal);
    let (i0, i1) = (he(hi).origin, he(body.next(hi)).origin);
    // The outer edge runs the other way round the face.
    let (o1, o0) = (he(ho).origin, he(body.next(ho)).origin);
    let w = body.point(i1) - body.point(i0);
    let starts = |h: Ix| he(h).origin == i0 || he(body.next(h)).origin == i0;
    let short = if starts(short[0]) { short } else { [short[1], short[0]] };
    let joins = short.map(|h| Some(beside(h)).filter(|&x| plane(x).is_some_and(|p| parallel(p.normal, w))));
    Ok(Base { key: body.faces()[f as usize].id, inner: [i0, i1], outer: [o0, o1], out: unit(end.normal), up, gone: vec![f], kept: Vec::new(), joins })
}

/// The faces of sheet metal `body` a flange may stand on: across the thickness (not on the
/// sheet's two sides), flat, four edges, the long two on parallel flat sides of the sheet. What
/// the window lights up for a flange; the flange itself checks the same.
pub fn flange_ends(body: &Body) -> Vec<TopoId> {
    let Ok(sides) = crate::unfold::sheet_sides(body) else { return Vec::new() };
    let skip = crate::unfold::branches(body);
    let on_sides: BTreeSet<Ix> = sides.iter().filter_map(|&s| crate::unfold::chain_through(body, s, &skip).ok()).flatten().flatten().collect();
    (0..body.faces().len() as Ix).filter(|x| !on_sides.contains(x) && end_of(body, *x).is_some()).map(|x| body.faces()[x as usize].id).collect()
}

/// The edges a flange is picked by, as in Inventor (D205): each end's two long edges, and each
/// flat wall's edges on the sheet's flat side faces; each with its ID and its ends.
pub fn flange_edges(body: &Body) -> Vec<(TopoId, [Vec3; 2])> {
    let he = |h: Ix| body.half_edges()[h as usize];
    let skip = crate::unfold::branches(body);
    let Ok(sides) = crate::unfold::sheet_sides(body) else { return Vec::new() };
    let on_sides: BTreeSet<Ix> = sides.iter().filter_map(|&s| crate::unfold::chain_through(body, s, &skip).ok()).flatten().flatten().collect();
    let flat = |x: Ix| matches!(body.faces()[x as usize].surface, Surface::Plane(_));
    let mut out: Vec<(TopoId, [Vec3; 2])> = Vec::new();
    for &x in on_sides.iter().filter(|&&x| flat(x)) {
        for h in body.loop_half_edges(body.faces()[x as usize].first_loop) {
            let y = body.loops()[he(he(h).twin).loop_ix as usize].face;
            if on_sides.contains(&y) || skip.contains(&y) || !flat(y) {
                continue;
            }
            let e = &body.edges()[he(h).edge as usize];
            // A side face's edge along a wall, or an end's long edge.
            let end = end_of(body, y).is_some_and(|(long, _)| long.contains(&he(h).twin));
            if end || side_face(body, y, &on_sides) {
                out.push((e.id, [body.point(he(h).origin), body.point(he(body.next(h)).origin)]));
            }
        }
    }
    out.sort_by_key(|e| e.0);
    out.dedup_by_key(|e| e.0);
    out
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

/// The flange `base` asks for on `target`: a bend and a wall sewn to its edges.
fn grow(feature: TopoId, target: &Body, base: Base, dims: FlangeDims) -> Result<Body, BrepError> {
    let Base { key, inner: [i0, i1], outer: [o0, o1], out: n, up, gone, kept, joins } = base;
    let w = unit(target.point(i1) - target.point(i0));
    let t = (target.point(i0) - target.point(o0)).dot(up);
    if t.is_nan() || t <= 0.0 {
        return Err(BrepError::Invalid("flange: the inner edge is not inside the sheet's thickness".into()));
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
    let id = |tag: &str, k: u64| TopoId::derive(tag).id(feature).id(key).index(k).finish();
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
    // The sides: into the face at that end of the width where it is square to the width, else
    // faces of their own.
    let mut gone: BTreeSet<Ix> = gone.into_iter().collect();
    faces.extend(kept);
    for (s, (join, chain)) in joins.iter().zip([[ii0, ji0, ki0, qo0, po0, oo0], [ii1, ji1, ki1, qo1, po1, oo1]]).enumerate() {
        let outward = if s == 0 { w * -1.0 } else { w };
        let chain = chain.to_vec();
        let Some(x) = *join else {
            faces.push(FaceSpec {
                id: id("flange.side", s as u64),
                surface: Surface::Plane(Plane { origin: point(chain[0]), normal: outward }),
                loops: vec![oriented(chain, outward)],
            });
            continue;
        };
        // That face's edge across the thickness runs from one of the flange's corners to the
        // other: the flange's outline goes in between, the same way.
        let mut spec = target.face_spec(x);
        let (a, b) = (chain[0], chain[chain.len() - 1]);
        for l in &mut spec.loops {
            let n = l.len();
            if let Some(k) = (0..n).find(|&k| (l[k], l[(k + 1) % n]) == (a, b) || (l[k], l[(k + 1) % n]) == (b, a)) {
                let run: Vec<TopoId> =
                    if l[k] == a { chain[1..chain.len() - 1].to_vec() } else { chain[1..chain.len() - 1].iter().rev().copied().collect() };
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
