//! The immutable body and its queries.

use std::ops::Range;

use datum_math::{Rigid, Vec3};

use crate::TopoId;

/// Index into one of a body's element vectors.
pub type Ix = u32;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vertex {
    pub id: TopoId,
    pub point: Vec3,
}

/// A directed edge use. The half-edges of a loop are contiguous, so `next` is implied (see
/// [`Body::next`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HalfEdge {
    pub origin: Ix,
    pub twin: Ix,
    pub loop_ix: Ix,
    pub edge: Ix,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Edge {
    pub id: TopoId,
    /// The half-edge running from the edge's lower-ID vertex to its higher-ID one.
    pub half: Ix,
}

/// A boundary cycle of a face: half-edges `first .. first + len`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Loop {
    pub face: Ix,
    pub first: Ix,
    pub len: Ix,
}

/// A plane through `origin`; `normal` points out of the body and need not be unit length.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Plane {
    pub origin: Vec3,
    pub normal: Vec3,
}

/// A face: loops `first_loop .. first_loop + loop_count`, the outer one first, holes after it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Face {
    pub id: TopoId,
    pub plane: Plane,
    pub first_loop: Ix,
    pub loop_count: Ix,
    /// Merkle leaf: covers the face's ID, plane and loops (vertex IDs and points), not indices.
    pub hash: [u8; 32],
}

/// A closed, oriented, manifold solid. Immutable: operations make new bodies.
///
/// Element vectors are in canonical order (ARCHITECTURE §4.1): equal bodies have equal memory
/// layout and equal [`Body::hash`].
#[derive(Clone, Debug, PartialEq)]
pub struct Body {
    pub(crate) vertices: Vec<Vertex>,
    pub(crate) half_edges: Vec<HalfEdge>,
    pub(crate) edges: Vec<Edge>,
    pub(crate) loops: Vec<Loop>,
    pub(crate) faces: Vec<Face>,
    pub(crate) hash: [u8; 32],
}

impl Body {
    pub fn vertices(&self) -> &[Vertex] {
        &self.vertices
    }

    pub fn half_edges(&self) -> &[HalfEdge] {
        &self.half_edges
    }

    pub fn edges(&self) -> &[Edge] {
        &self.edges
    }

    pub fn loops(&self) -> &[Loop] {
        &self.loops
    }

    pub fn faces(&self) -> &[Face] {
        &self.faces
    }

    /// Canonical content hash: blake3 over the face hashes in face order.
    pub fn hash(&self) -> [u8; 32] {
        self.hash
    }

    pub fn face_loops(&self, f: Ix) -> Range<Ix> {
        let face = &self.faces[f as usize];
        face.first_loop..face.first_loop + face.loop_count
    }

    pub fn loop_half_edges(&self, l: Ix) -> Range<Ix> {
        let lp = &self.loops[l as usize];
        lp.first..lp.first + lp.len
    }

    pub fn next(&self, h: Ix) -> Ix {
        let lp = &self.loops[self.half_edges[h as usize].loop_ix as usize];
        if h + 1 == lp.first + lp.len { lp.first } else { h + 1 }
    }

    pub fn point(&self, v: Ix) -> Vec3 {
        self.vertices[v as usize].point
    }

    /// Points of a loop in order.
    pub fn loop_points(&self, l: Ix) -> impl Iterator<Item = Vec3> + '_ {
        self.loop_half_edges(l).map(|h| self.point(self.half_edges[h as usize].origin))
    }

    /// Enclosed volume by the divergence theorem, relative to the first vertex so that the result
    /// does not degrade with distance from the origin. Fixed summation order: deterministic.
    pub fn volume(&self) -> f64 {
        let Some(o) = self.vertices.first().map(|v| v.point) else { return 0.0 };
        let mut sum = 0.0;
        for l in 0..self.loops.len() as Ix {
            let mut pts = self.loop_points(l).map(|p| p - o);
            let Some(p0) = pts.next() else { continue };
            let mut prev = pts.next();
            for p in pts {
                if let Some(a) = prev {
                    sum += p0.dot(a.cross(p));
                }
                prev = Some(p);
            }
        }
        sum / 6.0
    }

    /// The same body moved by `m`. Topology IDs are unchanged; hashes follow the new coordinates.
    pub fn transformed(&self, m: &Rigid) -> Body {
        let mut b = self.clone();
        for v in &mut b.vertices {
            v.point = m.apply(v.point);
        }
        for f in &mut b.faces {
            f.plane = Plane { origin: m.apply(f.plane.origin), normal: m.apply_vector(f.plane.normal) };
        }
        b.rehash();
        b
    }

    pub(crate) fn rehash(&mut self) {
        for f in 0..self.faces.len() {
            self.faces[f].hash = self.face_hash(f as Ix);
        }
        let mut h = blake3::Hasher::new();
        for f in &self.faces {
            h.update(&f.hash);
        }
        self.hash = *h.finalize().as_bytes();
    }

    pub(crate) fn face_hash(&self, f: Ix) -> [u8; 32] {
        let face = &self.faces[f as usize];
        let mut h = blake3::Hasher::new();
        h.update(&face.id.0.to_le_bytes());
        for c in face.plane.origin.to_array().into_iter().chain(face.plane.normal.to_array()) {
            h.update(&c.to_bits().to_le_bytes());
        }
        for l in self.face_loops(f) {
            h.update(&self.loops[l as usize].len.to_le_bytes());
            for he in self.loop_half_edges(l) {
                let v = &self.vertices[self.half_edges[he as usize].origin as usize];
                h.update(&v.id.0.to_le_bytes());
                for c in v.point.to_array() {
                    h.update(&c.to_bits().to_le_bytes());
                }
            }
        }
        *h.finalize().as_bytes()
    }
}
