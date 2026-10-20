//! Building a canonical body from vertices and faces given by topology IDs.

use rayon::prelude::*;

use crate::id::IdMap;
use crate::{Arc, Body, BrepError, Edge, Face, HalfEdge, Ix, Loop, Surface, TopoId, Vertex, in_order};

/// A face to build: `loops[0]` is the outer boundary, counterclockwise seen from outside the
/// body; further loops are holes, clockwise.
#[derive(Clone, Debug)]
pub struct FaceSpec {
    pub id: TopoId,
    pub surface: Surface,
    pub loops: Vec<Vec<TopoId>>,
}

/// An arc edge to build: the arc turns counterclockwise about its axis from vertex `from` to
/// vertex `to`. Edges without one are straight.
#[derive(Clone, Copy, Debug)]
pub struct ArcSpec {
    pub from: TopoId,
    pub to: TopoId,
    pub arc: Arc,
}

/// Builds a body in canonical order: vertices, faces and edges sorted by ID, each loop starting
/// at its lowest vertex ID, holes ordered by that vertex. Input order does not matter.
///
/// Structural errors found while linking are returned; the full [`Body::validate`] runs once per
/// operation in `datum-ops` (debug builds).
pub fn build(vertices: Vec<Vertex>, faces: Vec<FaceSpec>) -> Result<Body, BrepError> {
    build_with_arcs(vertices, faces, Vec::new())
}

/// [`build`] with arc edges: each of `arcs` names an edge of the faces by its two vertices.
pub fn build_with_arcs(mut vertices: Vec<Vertex>, mut faces: Vec<FaceSpec>, arcs: Vec<ArcSpec>) -> Result<Body, BrepError> {
    // IDs are unique (checked below), so the parallel unstable sorts are deterministic.
    vertices.par_sort_unstable_by_key(|v| v.id);
    if let Some(w) = vertices.windows(2).find(|w| w[0].id == w[1].id) {
        return Err(BrepError::Invalid(format!("duplicate vertex {:?}", w[0].id)));
    }
    faces.par_sort_unstable_by_key(|f| f.id);
    if let Some(w) = faces.windows(2).find(|w| w[0].id == w[1].id) {
        return Err(BrepError::Invalid(format!("duplicate face {:?}", w[0].id)));
    }
    // Lookups only, never iterated: the map's order cannot reach the body.
    let index: IdMap<Ix> = vertices.iter().enumerate().map(|(i, v)| (v.id, i as Ix)).collect();
    let vix = |id: TopoId| -> Result<Ix, BrepError> { index.get(&id).copied().ok_or_else(|| BrepError::Invalid(format!("unknown vertex {id:?}"))) };

    // Every face's loops as vertex indices in canonical order, in parallel; then laid out.
    let canonical = |spec: &FaceSpec| -> Result<Vec<Vec<Ix>>, BrepError> {
        let (outer, holes) = spec.loops.split_first().ok_or_else(|| BrepError::Degenerate(format!("face {:?} has no loops", spec.id)))?;
        let mut holes: Vec<Vec<TopoId>> = holes.iter().map(|l| rotate_to_min(l)).collect();
        holes.sort_unstable_by_key(|l| l.first().copied());
        std::iter::once(rotate_to_min(outer))
            .chain(holes)
            .map(|ids| {
                if ids.len() < 3 {
                    return Err(BrepError::Degenerate(format!("face {:?} has a loop of {} vertices", spec.id, ids.len())));
                }
                ids.into_iter().map(vix).collect()
            })
            .collect()
    };
    let face_loops = in_order(faces.par_iter().map(canonical))?;
    // Where every face's loops and half-edges start: prefix sums, then the layout in parallel.
    let (mut first_loop, mut first_he) = (Vec::with_capacity(faces.len()), Vec::with_capacity(faces.len()));
    let (mut nl, mut nh) = (0, 0);
    for lps in &face_loops {
        first_loop.push(nl);
        first_he.push(nh);
        nl += lps.len() as Ix;
        nh += lps.iter().map(|l| l.len() as Ix).sum::<Ix>();
    }
    let loops: Vec<Loop> = face_loops
        .par_iter()
        .enumerate()
        .flat_map_iter(|(f, lps)| {
            let starts = lps.iter().scan(first_he[f], |at, l| Some(std::mem::replace(at, *at + l.len() as Ix)));
            lps.iter().zip(starts).map(move |(l, first)| Loop { face: f as Ix, first, len: l.len() as Ix })
        })
        .collect();
    let half_edges: Vec<HalfEdge> = face_loops
        .par_iter()
        .enumerate()
        .flat_map_iter(|(f, lps)| {
            let base = first_loop[f];
            lps.iter()
                .enumerate()
                .flat_map(move |(k, l)| l.iter().map(move |&origin| HalfEdge { origin, twin: Ix::MAX, loop_ix: base + k as Ix, edge: Ix::MAX }))
        })
        .collect();
    let out_faces: Vec<Face> = faces
        .par_iter()
        .zip(&face_loops)
        .enumerate()
        .map(|(f, (spec, lps))| Face { id: spec.id, surface: spec.surface, first_loop: first_loop[f], loop_count: lps.len() as Ix, hash: [0; 32] })
        .collect();

    let mut body = Body { vertices, half_edges, edges: Vec::new(), loops, faces: out_faces, arcs: Vec::new(), hash: [0; 32] };
    link_edges(&mut body)?;
    attach_arcs(&mut body, arcs)?;
    body.rehash();
    Ok(body)
}

/// Pairs twins and creates edges. Directed `(origin, destination)` pairs are sorted once, then each
/// reverse pair is found by binary search.
fn link_edges(body: &mut Body) -> Result<(), BrepError> {
    let vid = |b: &Body, v: Ix| b.vertices[v as usize].id;
    // Uses of every edge, `(low, high, reversed, half-edge)`: sorted, the two uses of an edge are
    // neighbours, the one from the lower vertex first. No search over millions of half-edges.
    let mut uses: Vec<(Ix, Ix, bool, Ix)> = (0..body.half_edges.len() as Ix)
        .into_par_iter()
        .map(|h| {
            let (a, b) = (body.half_edges[h as usize].origin, body.half_edges[body.next(h) as usize].origin);
            (a.min(b), a.max(b), a > b, h)
        })
        .collect();
    uses.par_sort_unstable();
    if let Some(u) = uses.iter().find(|u| u.0 == u.1) {
        return Err(BrepError::Degenerate(format!("zero-length edge at {:?}", vid(body, u.0))));
    }
    let ends = |u: &(Ix, Ix, bool, Ix)| if u.2 { (u.1, u.0) } else { (u.0, u.1) };
    for (i, u) in uses.iter().enumerate().step_by(2) {
        let (a, b) = ends(u);
        let same = |k: usize| uses.get(k).is_some_and(|w| (w.0, w.1) == (u.0, u.1));
        if !same(i + 1) {
            return Err(BrepError::NotClosed(format!("edge {:?} → {:?} has no twin", vid(body, a), vid(body, b))));
        }
        if uses[i + 1].2 == u.2 || same(i + 2) {
            return Err(BrepError::NonManifold(format!("edge {:?} → {:?} used twice in one direction", vid(body, a), vid(body, b))));
        }
    }
    // Vertex indices follow vertex IDs, so the first use runs from the lower ID.
    let mut edges: Vec<Edge> = uses
        .par_chunks(2)
        .map(|c| Edge { id: TopoId::derive("edge").id(vid(body, c[0].0)).id(vid(body, c[0].1)).finish(), half: c[0].3, arc: None })
        .collect();
    for c in uses.chunks(2) {
        body.half_edges[c[0].3 as usize].twin = c[1].3;
        body.half_edges[c[1].3 as usize].twin = c[0].3;
    }
    edges.par_sort_unstable_by_key(|e| e.id);
    for (e, edge) in edges.iter().enumerate() {
        let t = body.half_edges[edge.half as usize].twin;
        body.half_edges[edge.half as usize].edge = e as Ix;
        body.half_edges[t as usize].edge = e as Ix;
    }
    body.edges = edges;
    Ok(())
}

/// Puts each arc on its edge, turned to run from the edge's lower-ID vertex.
fn attach_arcs(body: &mut Body, mut arcs: Vec<ArcSpec>) -> Result<(), BrepError> {
    if arcs.is_empty() {
        return Ok(());
    }
    let key = |a: &ArcSpec| (a.from.min(a.to), a.from.max(a.to));
    arcs.sort_unstable_by_key(key);
    if let Some(w) = arcs.windows(2).find(|w| key(&w[0]) == key(&w[1])) {
        return Err(BrepError::Invalid(format!("two arcs between {:?} and {:?}", w[0].from, w[0].to)));
    }
    let mut found = 0;
    for e in 0..body.edges.len() {
        let he = &body.half_edges[body.edges[e].half as usize];
        let ends = (body.vertices[he.origin as usize].id, body.vertices[body.half_edges[he.twin as usize].origin as usize].id);
        if let Ok(k) = arcs.binary_search_by_key(&ends, key) {
            let a = &arcs[k];
            body.edges[e].arc = Some(body.arcs.len() as Ix);
            body.arcs.push(if a.from == ends.0 { a.arc } else { a.arc.reversed() });
            found += 1;
        }
    }
    if found != arcs.len() {
        return Err(BrepError::Invalid("an arc names no edge of the faces".into()));
    }
    Ok(())
}

fn rotate_to_min(ids: &[TopoId]) -> Vec<TopoId> {
    let start = ids.iter().enumerate().min_by_key(|(_, id)| **id).map_or(0, |(i, _)| i);
    ids[start..].iter().chain(&ids[..start]).copied().collect()
}
