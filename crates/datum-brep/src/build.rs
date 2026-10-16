//! Building a canonical body from vertices and faces given by topology IDs.

use crate::{Body, BrepError, Edge, Face, HalfEdge, Ix, Loop, Plane, TopoId, Vertex};

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
pub fn build(mut vertices: Vec<Vertex>, mut faces: Vec<FaceSpec>) -> Result<Body, BrepError> {
    vertices.sort_unstable_by_key(|v| v.id);
    if let Some(w) = vertices.windows(2).find(|w| w[0].id == w[1].id) {
        return Err(BrepError::Invalid(format!("duplicate vertex {:?}", w[0].id)));
    }
    faces.sort_unstable_by_key(|f| f.id);
    if let Some(w) = faces.windows(2).find(|w| w[0].id == w[1].id) {
        return Err(BrepError::Invalid(format!("duplicate face {:?}", w[0].id)));
    }
    let vix = |id: TopoId| -> Result<Ix, BrepError> {
        vertices.binary_search_by_key(&id, |v| v.id).map(|i| i as Ix).map_err(|_| BrepError::Invalid(format!("unknown vertex {id:?}")))
    };

    let mut half_edges = Vec::new();
    let mut loops = Vec::new();
    let mut out_faces = Vec::with_capacity(faces.len());
    for (f, spec) in faces.iter().enumerate() {
        let (outer, holes) = spec.loops.split_first().ok_or_else(|| BrepError::Degenerate(format!("face {:?} has no loops", spec.id)))?;
        let mut holes: Vec<Vec<TopoId>> = holes.iter().map(|l| rotate_to_min(l)).collect();
        holes.sort_unstable_by_key(|l| l.first().copied());
        let first_loop = loops.len() as Ix;
        for ids in std::iter::once(rotate_to_min(outer)).chain(holes) {
            if ids.len() < 3 {
                return Err(BrepError::Degenerate(format!("face {:?} has a loop of {} vertices", spec.id, ids.len())));
            }
            let loop_ix = loops.len() as Ix;
            loops.push(Loop { face: f as Ix, first: half_edges.len() as Ix, len: ids.len() as Ix });
            for id in ids {
                half_edges.push(HalfEdge { origin: vix(id)?, twin: Ix::MAX, loop_ix, edge: Ix::MAX });
            }
        }
        out_faces.push(Face { id: spec.id, plane: spec.plane, first_loop, loop_count: loops.len() as Ix - first_loop, hash: [0; 32] });
    }

    let mut body = Body { vertices, half_edges, edges: Vec::new(), loops, faces: out_faces, hash: [0; 32] };
    link_edges(&mut body)?;
    body.rehash();
    if cfg!(debug_assertions) {
        body.validate()?;
    }
    Ok(body)
}

/// Pairs twins and creates edges. Directed `(origin, destination)` pairs are sorted once, then each
/// reverse pair is found by binary search.
fn link_edges(body: &mut Body) -> Result<(), BrepError> {
    let vid = |b: &Body, v: Ix| b.vertices[v as usize].id;
    let mut directed: Vec<(Ix, Ix, Ix)> =
        (0..body.half_edges.len() as Ix).map(|h| (body.half_edges[h as usize].origin, body.half_edges[body.next(h) as usize].origin, h)).collect();
    directed.sort_unstable();
    if let Some(w) = directed.windows(2).find(|w| (w[0].0, w[0].1) == (w[1].0, w[1].1)) {
        return Err(BrepError::NonManifold(format!("edge {:?} → {:?} used twice in one direction", vid(body, w[0].0), vid(body, w[0].1))));
    }
    let mut edges = Vec::with_capacity(directed.len() / 2);
    for &(a, b, h) in &directed {
        if a == b {
            return Err(BrepError::Degenerate(format!("zero-length edge at {:?}", vid(body, a))));
        }
        let t = directed
            .binary_search_by(|&(x, y, _)| (x, y).cmp(&(b, a)))
            .map_err(|_| BrepError::NotClosed(format!("edge {:?} → {:?} has no twin", vid(body, a), vid(body, b))))?;
        body.half_edges[h as usize].twin = directed[t].2;
        // Vertex indices follow vertex IDs, so `a < b` picks the half-edge from the lower ID.
        if a < b {
            edges.push(Edge { id: TopoId::derive("edge").id(vid(body, a)).id(vid(body, b)).finish(), half: h });
        }
    }
    edges.sort_unstable_by_key(|e| e.id);
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
