//! Building a canonical body from vertices and faces given by topology IDs.

use rayon::prelude::*;

use crate::{Body, BrepError, Edge, Face, HalfEdge, Ix, Loop, Plane, TopoId, Vertex, in_order};

/// A face to build: `loops[0]` is the outer boundary, counterclockwise seen from outside the
/// body; further loops are holes, clockwise.
#[derive(Clone, Debug)]
pub struct FaceSpec {
    pub id: TopoId,
    pub plane: Plane,
    pub loops: Vec<Vec<TopoId>>,
}

/// Builds a body in canonical order: vertices, faces and edges sorted by ID, each loop starting
/// at its lowest vertex ID, holes ordered by that vertex. Input order does not matter.
///
/// Structural errors found while linking are returned; the full [`Body::validate`] runs once per
/// operation in `datum-ops` (debug builds).
pub fn build(mut vertices: Vec<Vertex>, mut faces: Vec<FaceSpec>) -> Result<Body, BrepError> {
    // IDs are unique (checked below), so the parallel unstable sorts are deterministic.
    vertices.par_sort_unstable_by_key(|v| v.id);
    if let Some(w) = vertices.windows(2).find(|w| w[0].id == w[1].id) {
        return Err(BrepError::Invalid(format!("duplicate vertex {:?}", w[0].id)));
    }
    faces.par_sort_unstable_by_key(|f| f.id);
    if let Some(w) = faces.windows(2).find(|w| w[0].id == w[1].id) {
        return Err(BrepError::Invalid(format!("duplicate face {:?}", w[0].id)));
    }
    let vix = |id: TopoId| -> Result<Ix, BrepError> {
        vertices.binary_search_by_key(&id, |v| v.id).map(|i| i as Ix).map_err(|_| BrepError::Invalid(format!("unknown vertex {id:?}")))
    };

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
        .map(|(f, (spec, lps))| Face { id: spec.id, plane: spec.plane, first_loop: first_loop[f], loop_count: lps.len() as Ix, hash: [0; 32] })
        .collect();

    let mut body = Body { vertices, half_edges, edges: Vec::new(), loops, faces: out_faces, hash: [0; 32] };
    link_edges(&mut body)?;
    body.rehash();
    Ok(body)
}

/// Pairs twins and creates edges. Directed `(origin, destination)` pairs are sorted once, then each
/// reverse pair is found by binary search.
fn link_edges(body: &mut Body) -> Result<(), BrepError> {
    let vid = |b: &Body, v: Ix| b.vertices[v as usize].id;
    let mut directed: Vec<(Ix, Ix, Ix)> =
        (0..body.half_edges.len() as Ix).map(|h| (body.half_edges[h as usize].origin, body.half_edges[body.next(h) as usize].origin, h)).collect();
    directed.par_sort_unstable();
    if let Some(w) = directed.windows(2).find(|w| (w[0].0, w[0].1) == (w[1].0, w[1].1)) {
        return Err(BrepError::NonManifold(format!("edge {:?} → {:?} used twice in one direction", vid(body, w[0].0), vid(body, w[0].1))));
    }
    let twin = |&(a, b, _): &(Ix, Ix, Ix)| {
        if a == b {
            return Err(BrepError::Degenerate(format!("zero-length edge at {:?}", vid(body, a))));
        }
        directed
            .binary_search_by(|&(x, y, _)| (x, y).cmp(&(b, a)))
            .map(|t| directed[t].2)
            .map_err(|_| BrepError::NotClosed(format!("edge {:?} → {:?} has no twin", vid(body, a), vid(body, b))))
    };
    let twins = in_order(directed.par_iter().map(twin))?;
    // Vertex indices follow vertex IDs, so `a < b` picks the half-edge from the lower ID.
    let mut edges: Vec<Edge> = directed
        .par_iter()
        .filter(|d| d.0 < d.1)
        .map(|&(a, b, h)| Edge { id: TopoId::derive("edge").id(vid(body, a)).id(vid(body, b)).finish(), half: h })
        .collect();
    for (&(_, _, h), t) in directed.iter().zip(twins) {
        body.half_edges[h as usize].twin = t;
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

fn rotate_to_min(ids: &[TopoId]) -> Vec<TopoId> {
    let start = ids.iter().enumerate().min_by_key(|(_, id)| **id).map_or(0, |(i, _)| i);
    ids[start..].iter().chain(&ids[..start]).copied().collect()
}
