//! Flanges of sheet metal (M8, D205): on an end face of a sheet (an end of a sheet along a path,
//! an edge of a face of sheet metal), a bend of the rule's inner radius and a wall beyond it,
//! made directly: the new faces are sewn to the end face's own vertices, which it gives up. A
//! flange's side lies in the plane of the face beside the end where that face is square to the
//! edge (a sheet's side, a plate's square corner): it joins that face, as a bend's side is part of
//! the sheet's side face.

use std::collections::BTreeSet;

use datum_brep::{Arc, Body, BrepError, Curve, CurveSpec, Cylinder, FaceSpec, Ix, Plane, Surface, TopoId, Vertex, build_with_curves};
use datum_math::{Sign, Vec3, orient3d_coords};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::bent::Corner;
use crate::sketch::parallel;

/// What a flange is: its height from the outer surface of the sheet (for a right angle; else
/// along the flange from where its outer surface meets the sheet's), the angle it turns by in
/// degrees, its bend's inner radius, the side it turns to (`flip`: the other), and at corners
/// where flanges meet a round relief's diameter and the K-factor the blank is laid at (D229).
#[derive(Clone, Copy, Debug)]
pub struct FlangeDims {
    pub height: f64,
    pub angle: f64,
    pub radius: f64,
    pub flip: bool,
    pub round: Option<(f64, f64)>,
    /// How walls meeting at a corner close it, and the gap between them there.
    pub corner: Closing,
    pub gap: Option<f64>,
}

/// How the walls of flanges meeting at a corner close it (D229).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Closing {
    /// Each wall ends at its edge: the corner stays open.
    #[default]
    Open,
    /// Both walls go to the plane halfway between them, the gap apart.
    Miter,
    /// The walls along the first edge picked go to the other's outer face, the others to their
    /// inner face, the gap short of it.
    #[serde(rename = "overlap_1")]
    Overlap1,
    /// The other way round.
    #[serde(rename = "overlap_2")]
    Overlap2,
}

impl Closing {
    pub fn is_open(&self) -> bool {
        *self == Self::Open
    }
}

/// The relief at a corner where flanges meet (D229).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Relief {
    /// The square between the two bends' strips, which the open corner leaves.
    #[default]
    Square,
    /// That square and a circle about the corner where the bend lines meet.
    Round,
}

impl Relief {
    pub fn is_square(&self) -> bool {
        *self == Self::Square
    }
}

/// Flanges on edges `edges` of sheet metal `target`, one after another (D205): a long edge of an
/// end of the sheet puts it on that end, an edge of a wall on the sheet's side face puts it along
/// the wall (outward, no relief: it ends at the wall's bend lines). Two edges of one end or of
/// one wall's side make one flange; flanges on two ends meeting at a convex corner leave it open.
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
    // Each flange once, by the corners of what it stands on.
    let mut sites: Vec<(TopoId, [TopoId; 4], Base)> = Vec::new();
    for &e in &ids {
        let base = base(target, e, dims.flip, &BTreeSet::new())?;
        let corners = [base.inner[0], base.inner[1], base.outer[0], base.outer[1]].map(|v| target.vertices()[v as usize].id);
        let mut sorted = corners;
        sorted.sort();
        if sites.iter().all(|(_, c, _)| *c != sorted) {
            sites.push((e, sorted, base));
        }
    }
    // Flanges on two ends meeting at a corner leave it open (the corner's relief): neither's side
    // goes into the other's end. Each corner: its two vertices, the way away from the sheet there.
    let mut corners: Vec<([TopoId; 2], Vec3)> = Vec::new();
    for (k, (_, c, a)) in sites.iter().enumerate() {
        for (_, d, b) in &sites[k + 1..] {
            if c.iter().any(|v| d.contains(v)) {
                open_corner(target, a, b, dims.angle)?;
                let shared: Vec<TopoId> = c.iter().copied().filter(|v| d.contains(v)).collect();
                let [v, w] = shared[..] else { return Err(BrepError::Invalid("flange: a corner of more than one edge".into())) };
                corners.push(([v, w], a.out + b.out));
            }
        }
    }
    let bounds = closed(target, &sites, dims)?;
    let taken: BTreeSet<TopoId> = sites.iter().flat_map(|(.., b)| b.gone.iter().map(|&x| target.faces()[x as usize].id)).collect();
    let mut body = target.clone();
    for ((e, _, first), bounds) in sites.into_iter().zip(bounds) {
        // The side each turns to as the sheet picked had it: flanges made before it may tip
        // which side of the sheet is its front.
        let mut base = base(&body, e, dims.flip, &taken)?;
        if base.up.dot(first.up) < 0.0 {
            base = self::base(&body, e, !dims.flip, &taken)?;
        }
        body = grow(feature, &body, base, dims, bounds)?;
    }
    match dims.round {
        Some((d, k)) if !corners.is_empty() => relieve_corners(feature, &body, &corners, 0.5 * d, k),
        _ => Ok(body),
    }
}

/// Round reliefs at `corners` of `body` (D229): about the vertex of each on the sheet's front
/// side, cut through the blank from the face of the front side it is a corner of.
fn relieve_corners(feature: TopoId, body: &Body, corners: &[([TopoId; 2], Vec3)], radius: f64, k: f64) -> Result<Body, BrepError> {
    if !(radius > 0.0 && radius.is_finite()) {
        return Err(BrepError::Degenerate(format!("a relief's diameter is above zero, got {}", 2.0 * radius)));
    }
    let sides = crate::unfold::sheet_sides(body)?;
    let skip = crate::unfold::branches(body);
    let front: BTreeSet<Ix> = crate::unfold::chain_through(body, sides[0], &skip)?.into_iter().flatten().collect();
    let vertex = |id: TopoId| body.vertices().binary_search_by_key(&id, |v| v.id).map(|v| v as Ix).ok();
    // Each corner by the flat face of the front side it lies on (by its ID: faces are numbered
    // anew after a cut), the faces each in turn.
    let mut by_face: Vec<(TopoId, Vec<Corner>)> = Vec::new();
    for &(ids, away) in corners {
        let found = ids.iter().filter_map(|&id| vertex(id).map(|v| (id, v))).find_map(|(id, v)| {
            let x = front.iter().copied().find(|&x| {
                matches!(body.faces()[x as usize].surface, Surface::Plane(_))
                    && body.face_loops(x).flat_map(|l| body.loop_half_edges(l)).any(|h| body.half_edges()[h as usize].origin == v)
            })?;
            Some((body.faces()[x as usize].id, Corner { key: id, at: body.point(v), away }))
        });
        let (x, corner) =
            found.ok_or_else(|| BrepError::Unsupported("flange: a relief at a corner off the sheet's flat front comes later".into()))?;
        match by_face.iter_mut().find(|(y, _)| *y == x) {
            Some((_, cs)) => cs.push(corner),
            None => by_face.push((x, vec![corner])),
        }
    }
    let mut body = body.clone();
    for (x, cs) in by_face {
        let home = body.faces().binary_search_by_key(&x, |f| f.id).map_err(|_| BrepError::Invalid("relief: a face lost".into()))? as Ix;
        body = crate::bent::relieve(feature, &body, home, &cs, radius, k)?;
    }
    Ok(body)
}

/// Where the walls of flanges `sites` of `target` meeting at corners go, closing them as `dims`
/// says (D229): for each flange, at the start and end of its width, the plane its wall's side
/// goes to; none at an open corner.
fn closed(target: &Body, sites: &[(TopoId, [TopoId; 4], Base)], dims: FlangeDims) -> Result<Vec<[Option<Bound>; 2]>, BrepError> {
    let mut bounds = vec![[None, None]; sites.len()];
    if dims.corner.is_open() {
        return Ok(bounds);
    }
    let g = dims.gap.ok_or_else(|| BrepError::Invalid("a closed corner takes its gap from the part's sheet rule: it has none".into()))?;
    if !(g >= 0.0 && g.is_finite()) {
        return Err(BrepError::Degenerate(format!("a corner's gap is not below zero, got {g}")));
    }
    let shapes: Vec<Shape> = sites.iter().map(|(.., b)| Shape::of(target, b, dims)).collect::<Result<_, _>>()?;
    let first = shapes.first().map_or(Vec3::ZERO, |s| s.w);
    for a in 0..sites.len() {
        for b in a + 1..sites.len() {
            let (ca, cb) = (&sites[a].1, &sites[b].1);
            let Some(&shared) = ca.iter().find(|v| cb.contains(v)) else { continue };
            let corner = target.vertices().binary_search_by_key(&shared, |v| v.id).map(|v| target.point(v as Ix)).unwrap_or(Vec3::ZERO);
            // The end of each width at the corner.
            let end = |k: usize| {
                let base = &sites[k].2;
                usize::from(![base.inner[0], base.outer[0]].iter().any(|&v| target.vertices()[v as usize].id == shared))
            };
            let (ia, oa, ma) = shapes[a].faces(target, &sites[a].2);
            let (ib, ob, mb) = shapes[b].faces(target, &sites[b].2);
            let [ba, bb] = match dims.corner {
                Closing::Open => continue,
                Closing::Miter => {
                    // The plane through where the outer faces meet and where the inner faces do.
                    let l = unit(ma.cross(mb));
                    let meet = |p: Vec3, q: Vec3| solve([ma, mb, l], [p.dot(ma), q.dot(mb), corner.dot(l)]);
                    let (Some(out), Some(inn)) = (meet(oa, ob), meet(ia, ib)) else {
                        return Err(BrepError::Degenerate("flange: walls meeting at a corner are parallel".into()));
                    };
                    let mut mu = unit(l.cross(out - inn));
                    // `a`'s own edge lies on its side of the plane.
                    let middle = (target.point(sites[a].2.inner[0]) + target.point(sites[a].2.inner[1])) * 0.5;
                    if (middle - out).dot(mu) > 0.0 {
                        mu = mu * -1.0;
                    }
                    [Bound { at: out - mu * (0.5 * g), normal: mu }, Bound { at: out + mu * (0.5 * g), normal: mu * -1.0 }]
                }
                Closing::Overlap1 | Closing::Overlap2 => {
                    // The covering wall: the one along the first edge picked, more nearly.
                    let a_along = shapes[a].w.dot(first).abs() >= shapes[b].w.dot(first).abs();
                    let covers = a_along == (dims.corner == Closing::Overlap1);
                    let over = |o: Vec3, m: Vec3| Bound { at: o, normal: m };
                    let under = |i: Vec3, m: Vec3| Bound { at: i - m * g, normal: m };
                    if covers { [over(ob, mb), under(ia, ma)] } else { [under(ib, mb), over(oa, ma)] }
                }
            };
            bounds[a][end(a)] = Some(ba);
            bounds[b][end(b)] = Some(bb);
        }
    }
    // A wall of a flange made before beside an end's corner (D229): the new wall goes to its inner
    // face, the gap short of it; that wall stays as it is.
    for ((.., base), ends) in sites.iter().zip(&mut bounds) {
        for (s, bound) in ends.iter_mut().enumerate() {
            if bound.is_some() || base.joins[s].is_some() {
                continue;
            }
            if let Some(x) = base.beside[s]
                && let Some((at, n)) = wall_beside(target, x, base, s)
            {
                *bound = Some(Bound { at: at + n * g, normal: n * -1.0 });
            }
        }
    }
    Ok(bounds)
}

/// The inner face of the wall of a flange whose side is face `x`, beyond the end `s` of the
/// width of what `base` stands on: a point of it and its outward normal, back toward that end.
/// Of the planes beside `x`, a wall's two faces face away from each other; the nearest such inner
/// face; none when there is none.
fn wall_beside(target: &Body, x: Ix, base: &Base, s: usize) -> Option<(Vec3, Vec3)> {
    let he = |h: Ix| target.half_edges()[h as usize];
    let Surface::Plane(side) = target.faces()[x as usize].surface else { return None };
    let planes: Vec<Plane> = target
        .face_loops(x)
        .flat_map(|l| target.loop_half_edges(l))
        .filter_map(|h| match target.faces()[target.loops()[he(he(h).twin).loop_ix as usize].face as usize].surface {
            Surface::Plane(p) if !parallel(p.normal, side.normal) => Some(p),
            _ => None,
        })
        .collect();
    // From the corner on, past the end: the inner face faces back toward it.
    let corner = target.point(base.inner[s]);
    let w = unit(target.point(base.inner[1]) - target.point(base.inner[0]));
    let past = if s == 0 { w * -1.0 } else { w };
    planes
        .iter()
        .filter(|p| planes.iter().any(|q| parallel(q.normal, p.normal) && q.normal.dot(p.normal) < 0.0))
        .filter_map(|p| {
            let n = unit(p.normal);
            let gap = (corner - p.origin).dot(n);
            (n.dot(past) < 0.0 && gap > 0.0).then_some((gap, p.origin, n))
        })
        .min_by(|a, b| a.0.total_cmp(&b.0))
        .map(|(_, at, n)| (at, n))
}

/// The point on the planes `x · rows[k] = rhs[k]`; none when they share no single point.
fn solve(rows: [Vec3; 3], rhs: [f64; 3]) -> Option<Vec3> {
    let [a, b, c] = rows;
    let det = a.dot(b.cross(c));
    if det == 0.0 || !det.is_finite() {
        return None;
    }
    Some((b.cross(c) * rhs[0] + c.cross(a) * rhs[1] + a.cross(b) * rhs[2]) * (1.0 / det))
}

/// Whether flanges `a` and `b` of `target` may meet at a corner, left
/// open: both on ends, the corner convex, neither leaning in over the sheet past a right angle
/// (each then stays off the other's way, beyond its own edge).
fn open_corner(target: &Body, a: &Base, b: &Base, angle: f64) -> Result<(), BrepError> {
    let later = |m: &str| Err(BrepError::Unsupported(format!("flanges meeting at a corner: {m} come later")));
    if !(a.end && b.end) {
        return later("a flange along a wall and one on an end");
    }
    if angle > 90.0 {
        return later("flanges turning past a right angle");
    }
    // Convex: `b`'s far corner lies on the sheet's side of `a`'s end, the side away from where `a`
    // leaves the sheet.
    let p = |v: Ix| target.point(v);
    let plane = (p(a.inner[0]), p(a.inner[1]), p(a.outer[0]));
    let side = |q: Vec3| orient3d_coords(plane.0, plane.1, plane.2, q);
    let shared = |v: Ix| [a.inner, a.outer].iter().flatten().any(|&w| w == v);
    let far = [b.inner, b.outer].into_iter().flatten().find(|&v| !shared(v));
    let inside = far.map(|v| side(p(v)));
    if inside.is_none_or(|s| s == Sign::Zero || s == side(plane.0 + a.out)) {
        return later("flanges at an inner corner");
    }
    Ok(())
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
    /// On an end of the sheet (not along a wall).
    end: bool,
    /// The faces beside its width's start and end where a wall of a flange made before may stand:
    /// an end's across its short edges, a side face's strip's the side face itself.
    beside: [Option<Ix>; 2],
}

/// The flange edge `e` of `body` asks for (see [`flanges`]); its sides go into no face of `taken`
/// (other flanges' ends).
fn base(body: &Body, e: TopoId, flip: bool, taken: &BTreeSet<TopoId>) -> Result<Base, BrepError> {
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
            return end_base(body, f, long, short, flip, taken);
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
    // The wall's other side: of the sheet's sides along the side face, parallel to it, nearest
    // (a flange's own side, square to its wall, may be parallel too).
    let across = body
        .loop_half_edges(body.faces()[cap as usize].first_loop)
        .filter(|&g| g != cap_h)
        .filter_map(|g| {
            let x = face_of(he(g).twin);
            let p = plane(x)?;
            (x != wall && sides.contains(&x) && parallel(p.normal, pw.normal)).then(|| (g, (p.origin - pw.origin).dot(unit(pw.normal)).abs()))
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
    // The same whichever edge of the wall is picked: its ends in the order of their IDs.
    let id_of = |v: Ix| body.vertices()[v as usize].id;
    let (inner, outer) = if id_of(inner[0]) > id_of(inner[1]) { ([inner[1], inner[0]], [outer[1], outer[0]]) } else { (inner, outer) };
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
                joins[s] = plane(end).filter(|p| parallel(p.normal, d) && !taken.contains(&body.faces()[end as usize].id)).map(|_| end);
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
    let mut corners = strip;
    corners.sort();
    let key = TopoId::derive("flange.side").id(corners[0]).id(corners[3]).finish();
    Ok(Base { key, inner, outer, out: unit(pc.normal), up, gone: vec![cap], kept, joins, end: false, beside: [Some(cap); 2] })
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
            && body.faces()[x as usize]
                .surface
                .normal_at(body.point(he(h).origin))
                .dot(body.faces()[y as usize].surface.normal_at(body.point(he(h).origin)))
                > 0.5
    })
}

/// A flange on end face `f`, its long half-edges `long` and short ones `short`: toward the
/// sheet's front side, or the other.
fn end_base(body: &Body, f: Ix, long: [Ix; 2], short: [Ix; 2], flip: bool, taken: &BTreeSet<TopoId>) -> Result<Base, BrepError> {
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
    let joins =
        short.map(|h| Some(beside(h)).filter(|&x| plane(x).is_some_and(|p| parallel(p.normal, w)) && !taken.contains(&body.faces()[x as usize].id)));
    Ok(Base {
        key: body.faces()[f as usize].id,
        inner: [i0, i1],
        outer: [o0, o1],
        out: unit(end.normal),
        up,
        gone: vec![f],
        kept: Vec::new(),
        joins,
        end: true,
        beside: short.map(|h| Some(beside(h))),
    })
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

/// Where a flange's bend and wall go (see [`grow`]): the unit edge `w`, the axis it turns about,
/// the turn, the bend's axis point `c_in`, the wall's way up `along` and its height above the
/// bend, the sheet's thickness, and the inner face's outward normal.
struct Shape {
    w: Vec3,
    axis: Vec3,
    turn: (f64, f64),
    c_in: Vec3,
    along: Vec3,
    wall: f64,
    t: f64,
    inner_normal: Vec3,
}

impl Shape {
    fn of(target: &Body, base: &Base, dims: FlangeDims) -> Result<Self, BrepError> {
        let ([i0, i1], o0, n, up) = (base.inner, base.outer[0], base.out, base.up);
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
        // At a right angle exactly `r + t`: tan 45° in floating point is not quite 1, and a flange
        // as high as its bend would come out a sliver instead of refused.
        let set_back = (r + t) * if dims.angle == 90.0 { 1.0 } else { libm::tan(0.5 * a) };
        let wall = dims.height - set_back;
        if wall.is_nan() || wall <= 0.0 {
            return Err(BrepError::Degenerate(format!("a flange {} high is no higher than its bend ({set_back:.3})", dims.height)));
        }
        // One axis for the bend's both sides, through the inner edge's start `r` up: both sides
        // turn about it, so the two cylinders are coaxial exactly (made apart, they part in the
        // last bits, and the bend has no face across it).
        let c_in = target.point(i0) + up * r;
        Ok(Self { w, axis, turn, c_in, along, wall, t, inner_normal: turned(up, axis, turn) })
    }

    /// Where point `p` of the end goes as the bend turns it: where the bend ends, at the wall.
    fn turned(&self, p: Vec3) -> Vec3 {
        self.c_in + turned(p - self.c_in, self.axis, self.turn)
    }

    /// The wall's inner and outer faces: a point of each, and the outer face's outward normal.
    fn faces(&self, target: &Body, base: &Base) -> (Vec3, Vec3, Vec3) {
        (self.turned(target.point(base.inner[0])), self.turned(target.point(base.outer[0])), self.inner_normal * -1.0)
    }
}

/// A plane a side of a flange's wall goes to (D229): the wall keeps what lies where
/// `(x − at) · normal ≤ 0`.
#[derive(Clone, Copy, Debug)]
struct Bound {
    at: Vec3,
    normal: Vec3,
}

/// The flange `base` asks for on `target`: a bend and a wall sewn to its edges; the wall's side at
/// each end of the width square to it, or gone to `bounds` there (D229): the wall grows or is cut
/// along the edge to the plane, the bend not, a step across the thickness between them.
fn grow(feature: TopoId, target: &Body, base: Base, dims: FlangeDims, bounds: [Option<Bound>; 2]) -> Result<Body, BrepError> {
    let shape = Shape::of(target, &base, dims)?;
    let Shape { w, axis, c_in, along, wall, t, inner_normal, .. } = shape;
    let Base { key, inner: [i0, i1], outer: [o0, o1], gone, kept, joins, .. } = base;
    let (r, a) = (dims.radius, dims.angle.to_radians());
    let id = |tag: &str, k: u64| TopoId::derive(tag).id(feature).id(key).index(k).finish();
    let mut vertices: Vec<Vertex> = target.vertices().to_vec();
    // Along the edge from `p` to the plane of `b`.
    let reach = |b: &Bound, p: Vec3| -> Result<f64, BrepError> {
        let d = w.dot(b.normal);
        if d == 0.0 {
            return Err(BrepError::Unsupported("flange: a wall's side to a plane along its edge".into()));
        }
        Ok((b.at - p).dot(b.normal) / d)
    };
    // From each corner of the end, about the bend's axis: where the bend ends, and the wall's top;
    // with a bound at that end, also where the wall's foot meets it. The vertex at the bend's end,
    // at the wall's foot on the bound (itself when it is there), at the top.
    let mut corner = |end: usize, v: Ix, k: u64| -> Result<[TopoId; 4], BrepError> {
        let q = shape.turned(target.point(v));
        let (vq, ve, vs) = (id("flange.v", k), id("flange.v", k + 1), id("flange.v", 8 + k));
        vertices.push(Vertex { id: vq, point: q });
        let (foot, top) = match &bounds[end] {
            None => (vq, q + along * wall),
            Some(b) => {
                let out = if end == 0 { -1.0 } else { 1.0 };
                let (f, h) = (reach(b, q)?, reach(b, q + along * wall)?);
                // The bend stays as it is: a plane into it would cut it.
                if f * out < 0.0 {
                    return Err(BrepError::Unsupported("flange: the corner's plane cuts into the bend: comes later".into()));
                }
                if f != 0.0 {
                    vertices.push(Vertex { id: vs, point: q + w * f });
                }
                (if f == 0.0 { vq } else { vs }, q + along * wall + w * h)
            }
        };
        vertices.push(Vertex { id: ve, point: top });
        Ok([target.vertices()[v as usize].id, vq, foot, ve])
    };
    // Inner and outer, at the start and end of the width.
    let [ii0, ji0, fi0, ki0] = corner(0, i0, 0)?;
    let [ii1, ji1, fi1, ki1] = corner(1, i1, 2)?;
    let [oo0, po0, fo0, qo0] = corner(0, o0, 4)?;
    let [oo1, po1, fo1, qo1] = corner(1, o1, 6)?;
    let point = |v: TopoId| vertices.iter().find(|x| x.id == v).map_or(Vec3::ZERO, |x| x.point);
    // A loop of `ids` turning about `outward` by the right hand; a vertex twice in a row once.
    let oriented = |mut ids: Vec<TopoId>, outward: Vec3| -> Vec<TopoId> {
        ids.dedup();
        if ids.len() > 1 && ids.first() == ids.last() {
            ids.pop();
        }
        let p: Vec<Vec3> = ids.iter().map(|&v| point(v)).collect();
        let normal = (0..p.len()).fold(Vec3::ZERO, |s, k| s + p[k].cross(p[(k + 1) % p.len()]));
        if normal.dot(outward) < 0.0 {
            ids.reverse();
        }
        ids
    };
    let in_mid = c_in + turned(target.point(i0) - c_in, axis, libm::sincos(0.5 * a)) - c_in;
    let mut faces: Vec<FaceSpec> = Vec::new();
    let face = |tag: &str, k: u64, surface: Surface, loop_: Vec<TopoId>| FaceSpec { id: id(tag, k), surface, loops: vec![loop_] };
    faces.push(face(
        "flange.bend_in",
        0,
        Surface::Cylinder(Cylinder { origin: c_in, axis: w, radius: r, convex: false }),
        oriented(vec![ii0, ii1, ji1, ji0], in_mid * -1.0),
    ));
    faces.push(face(
        "flange.bend_out",
        0,
        Surface::Cylinder(Cylinder { origin: c_in, axis: w, radius: r + t, convex: true }),
        oriented(vec![oo0, oo1, po1, po0], in_mid),
    ));
    faces.push(face(
        "flange.wall_in",
        0,
        Surface::Plane(Plane { origin: point(ji0), normal: inner_normal }),
        oriented(vec![ji0, ji1, fi1, ki1, ki0, fi0], inner_normal),
    ));
    faces.push(face(
        "flange.wall_out",
        0,
        Surface::Plane(Plane { origin: point(po0), normal: inner_normal * -1.0 }),
        oriented(vec![po0, po1, fo1, qo1, qo0, fo0], inner_normal * -1.0),
    ));
    faces.push(face("flange.end", 0, Surface::Plane(Plane { origin: point(ki0), normal: along }), oriented(vec![ki0, ki1, qo1, qo0], along)));
    // The arcs of the bend at both sides, inner and outer, from the end face on.
    let arc = |c: Vec3, r: f64, from: TopoId, to: TopoId| {
        // Counterclockwise about `axis` from the end face.
        CurveSpec { from, to, curve: Curve::Arc(Arc { center: c, axis, radius: r }) }
    };
    let mut curves = target.curve_specs();
    // Each arc's centre on the axis, in its plane.
    let on_axis = |v: Ix| c_in + w * (target.point(v) - c_in).dot(w);
    curves.extend([
        arc(on_axis(i0), r, ii0, ji0),
        arc(on_axis(i1), r, ii1, ji1),
        arc(on_axis(o0), r + t, oo0, po0),
        arc(on_axis(o1), r + t, oo1, po1),
    ]);
    // The sides: into the face at that end of the width where it is square to the width, else
    // faces of their own; at a bound the bend's side, the step and the wall's side on the plane.
    let mut gone: BTreeSet<Ix> = gone.into_iter().collect();
    faces.extend(kept);
    let ends = [[ii0, ji0, fi0, ki0, qo0, fo0, po0, oo0], [ii1, ji1, fi1, ki1, qo1, fo1, po1, oo1]];
    for (s, (join, [ii, ji, fi, ki, qo, fo, po, oo])) in joins.iter().zip(ends).enumerate() {
        let outward = if s == 0 { w * -1.0 } else { w };
        let k = s as u64;
        if let Some(b) = &bounds[s] {
            faces.push(face("flange.side", k, Surface::Plane(Plane { origin: point(ii), normal: outward }), oriented(vec![ii, ji, po, oo], outward)));
            if (fi, fo) != (ji, po) {
                faces.push(face(
                    "flange.step",
                    k,
                    Surface::Plane(Plane { origin: point(ji), normal: along * -1.0 }),
                    oriented(vec![ji, fi, fo, po], along * -1.0),
                ));
            }
            faces.push(face(
                "flange.bound",
                k,
                Surface::Plane(Plane { origin: point(fi), normal: b.normal }),
                oriented(vec![fi, ki, qo, fo], b.normal),
            ));
            continue;
        }
        let chain = vec![ii, ji, ki, qo, po, oo];
        let Some(x) = *join else {
            faces.push(face("flange.side", k, Surface::Plane(Plane { origin: point(chain[0]), normal: outward }), oriented(chain, outward)));
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
