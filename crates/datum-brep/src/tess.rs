//! Triangulation of a body's faces for export, display and Boolean operations.

use datum_math::{Sign, Vec2, orient2d};

use crate::{Body, BrepError, Ix};

/// Triangle mesh sharing the body's vertices: `positions[i]` is vertex `i` of the body.
#[derive(Clone, Debug, PartialEq)]
pub struct Mesh {
    pub positions: Vec<datum_math::Vec3>,
    /// Counterclockwise seen from outside.
    pub triangles: Vec<[Ix; 3]>,
}

pub fn tessellate(body: &Body) -> Result<Mesh, BrepError> {
    let mut triangles = Vec::with_capacity(body.half_edges().len());
    for f in 0..body.faces().len() as Ix {
        triangles.extend(triangulate_face(body, f)?);
    }
    Ok(Mesh { positions: body.vertices().iter().map(|v| v.point).collect(), triangles })
}

/// Triangles of face `f` as vertex indices, counterclockwise seen from outside. No new vertices:
/// holes are joined to the outer loop by bridges, then the single ring is ear-clipped in the face's
/// dominant projection, all with exact predicates.
pub fn triangulate_face(body: &Body, f: Ix) -> Result<Vec<[Ix; 3]>, BrepError> {
    let face = &body.faces()[f as usize];
    let degenerate = || BrepError::Degenerate(format!("face {:?} cannot be triangulated", face.id));
    // Dropping the dominant axis keeps the loops' turning sense when the other two axes stay in
    // cyclic order; a negative dominant component flips it.
    let axis = face.plane.normal.dominant_axis();
    let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
    let convex = if face.plane.normal.to_array()[axis] < 0.0 { Sign::Negative } else { Sign::Positive };
    let pt = |i: Ix| {
        let p = body.point(i).to_array();
        Vec2::new(p[u], p[v])
    };

    let mut loops: Vec<Vec<Ix>> =
        body.face_loops(f).map(|l| body.loop_half_edges(l).map(|h| body.half_edges()[h as usize].origin).collect()).collect();
    let mut ring = loops.remove(0);
    // Holes in order of their rightmost vertex, as in Eberly's method; any order would do, since
    // every bridge is checked against all edges, but this one finds visible vertices sooner.
    let rightmost = |l: &[Ix]| (0..l.len()).max_by(|&a, &b| cmp_xy(pt(l[a]), pt(l[b]))).unwrap_or(0);
    loops.sort_by(|a, b| cmp_xy(pt(b[rightmost(b)]), pt(a[rightmost(a)])));
    for k in 0..loops.len() {
        let (hole, rest) = (&loops[k], &loops[k + 1..]);
        bridge(&mut ring, hole, rightmost(hole), rest, &pt, convex).map_err(|()| degenerate())?;
    }

    let mut triangles = Vec::with_capacity(ring.len() - 2);
    ear_clip(ring, &pt, convex, |t| triangles.push(t)).map_err(|()| degenerate())?;
    Ok(triangles)
}

fn cmp_xy(a: Vec2, b: Vec2) -> std::cmp::Ordering {
    a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y))
}

/// Splices `hole` into `ring` through a bridge from the hole's vertex `m` to a ring vertex `p`
/// that sees it: `… p, m, …hole…, m, p, …`. Candidates are tried nearest first; each must leave
/// both corners into the face and touch no edge of the ring or of the holes still to come.
fn bridge(ring: &mut Vec<Ix>, hole: &[Ix], m: usize, rest: &[Vec<Ix>], pt: &impl Fn(Ix) -> Vec2, convex: Sign) -> Result<(), ()> {
    let n = hole.len();
    let (mv, mp) = (hole[m], pt(hole[m]));
    let dist = |i: usize| {
        let d = pt(ring[i]);
        (d.x - mp.x) * (d.x - mp.x) + (d.y - mp.y) * (d.y - mp.y)
    };
    let mut order: Vec<usize> = (0..ring.len()).collect();
    order.sort_by(|&a, &b| dist(a).total_cmp(&dist(b)).then(a.cmp(&b)));
    let edges = |l: &[Ix]| (0..l.len()).map(move |i| (l[i], l[(i + 1) % l.len()])).collect::<Vec<_>>();
    let blockers: Vec<(Ix, Ix)> = edges(ring).into_iter().chain(edges(hole)).chain(rest.iter().flat_map(|l| edges(l))).collect();
    let r = ring.len();
    let found = order.into_iter().find(|&i| {
        let (pv, pp) = (ring[i], pt(ring[i]));
        enters(pt(ring[(i + r - 1) % r]), pp, pt(ring[(i + 1) % r]), mp, convex)
            && enters(pt(hole[(m + n - 1) % n]), mp, pt(hole[(m + 1) % n]), pp, convex)
            && blockers.iter().all(|&(a, b)| a == pv || a == mv || b == pv || b == mv || !segments_touch(pp, mp, pt(a), pt(b)))
    });
    let i = found.ok_or(())?;
    let spliced: Vec<Ix> = hole[m..].iter().chain(&hole[..=m]).copied().chain([ring[i]]).collect();
    ring.splice(i + 1..i + 1, spliced);
    Ok(())
}

/// The direction from corner `b` (between `a → b` and `b → c`, the face on the `convex` side)
/// towards `d` starts inside the face. Strict: along an edge is not inside.
fn enters(a: Vec2, b: Vec2, c: Vec2, d: Vec2, convex: Sign) -> bool {
    let (left_ab, left_bc) = (orient2d(a, b, d) == convex, orient2d(b, c, d) == convex);
    if orient2d(a, b, c) == convex { left_ab && left_bc } else { left_ab || left_bc }
}

/// Closed segments `ab` and `cd` share at least one point (exact).
pub(crate) fn segments_touch(a: Vec2, b: Vec2, c: Vec2, d: Vec2) -> bool {
    let (d1, d2) = (orient2d(a, b, c), orient2d(a, b, d));
    let (d3, d4) = (orient2d(c, d, a), orient2d(c, d, b));
    if [d1, d2, d3, d4].iter().all(|s| *s == Sign::Zero) {
        let overlap = |a0: f64, a1: f64, b0: f64, b1: f64| a0.min(a1) <= b0.max(b1) && b0.min(b1) <= a0.max(a1);
        return overlap(a.x, b.x, c.x, d.x) && overlap(a.y, b.y, c.y, d.y);
    }
    let straddles = |s: Sign, t: Sign| s == Sign::Zero || t == Sign::Zero || s != t;
    straddles(d1, d2) && straddles(d3, d4)
}

/// Ear clipping of a ring of vertices whose convex corners have sign `convex`. The ring may pass a
/// vertex twice (bridges), so vertices are compared by index. Emits `n − 2` triangles; `Err` when
/// no ear is found (a degenerate polygon).
fn ear_clip(mut ring: Vec<Ix>, pt: &impl Fn(Ix) -> Vec2, convex: Sign, mut emit: impl FnMut([Ix; 3])) -> Result<(), ()> {
    let mut i = 0;
    let mut misses = 0;
    while ring.len() > 3 {
        let n = ring.len();
        let (a, b, c) = (ring[(i + n - 1) % n], ring[i % n], ring[(i + 1) % n]);
        if is_ear(&ring, a, b, c, pt, convex) {
            emit([a, b, c]);
            ring.remove(i % n);
            misses = 0;
        } else {
            i += 1;
            misses += 1;
            if misses > n {
                return Err(());
            }
        }
    }
    emit([ring[0], ring[1], ring[2]]);
    Ok(())
}

fn is_ear(ring: &[Ix], a: Ix, b: Ix, c: Ix, pt: &impl Fn(Ix) -> Vec2, convex: Sign) -> bool {
    let (pa, pb, pc) = (pt(a), pt(b), pt(c));
    if orient2d(pa, pb, pc) != convex {
        return false;
    }
    // No other ring vertex inside or on the candidate triangle.
    ring.iter().filter(|&&k| k != a && k != b && k != c).all(|&k| {
        let p = pt(k);
        [orient2d(pa, pb, p), orient2d(pb, pc, p), orient2d(pc, pa, p)].iter().any(|s| *s != convex && *s != Sign::Zero)
    })
}
