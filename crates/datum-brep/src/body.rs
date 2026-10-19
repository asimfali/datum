//! The immutable body and its queries.

use std::ops::Range;

use datum_math::{Rigid, Vec3};

use crate::{FaceSpec, TopoId};

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

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MassProperties {
    pub volume: f64,
    pub area: f64,
    pub centroid: Vec3,
    /// Bounding box corners.
    pub min: Vec3,
    pub max: Vec3,
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

    /// Calls `f(a, b, c)` for every fan triangle of the loops in `loops`, with points relative to
    /// the body's first vertex `o`. Fans of a closed body form a closed surface, so the divergence
    /// theorem holds even when a face's vertices are not exactly coplanar.
    fn fans(&self, loops: Range<Ix>, o: Vec3, mut f: impl FnMut(Vec3, Vec3, Vec3)) {
        for l in loops {
            let mut pts = self.loop_points(l).map(|p| p - o);
            let Some(p0) = pts.next() else { continue };
            let mut prev = pts.next();
            for p in pts {
                if let Some(a) = prev {
                    f(p0, a, p);
                }
                prev = Some(p);
            }
        }
    }

    fn reference(&self) -> Vec3 {
        self.vertices.first().map_or(Vec3::ZERO, |v| v.point)
    }

    /// Enclosed volume by the divergence theorem, relative to the first vertex so that the result
    /// does not degrade with distance from the origin. Fixed summation order: deterministic.
    pub fn volume(&self) -> f64 {
        let mut sum = 0.0;
        self.fans(0..self.loops.len() as Ix, self.reference(), |a, b, c| sum += a.dot(b.cross(c)));
        sum / 6.0
    }

    /// Area vector of a face: its normal scaled by its area (holes subtracted).
    pub fn face_area_vector(&self, f: Ix) -> Vec3 {
        let mut sum = Vec3::ZERO;
        self.fans(self.face_loops(f), self.reference(), |a, b, c| sum = sum + (b - a).cross(c - a));
        sum * 0.5
    }

    /// Volume, surface area, centre of mass (uniform density) and bounding box.
    pub fn mass_properties(&self) -> MassProperties {
        let o = self.reference();
        let (mut six_v, mut moment) = (0.0, Vec3::ZERO);
        self.fans(0..self.loops.len() as Ix, o, |a, b, c| {
            let v = a.dot(b.cross(c));
            six_v += v;
            moment = moment + (a + b + c) * v;
        });
        let area = (0..self.faces.len() as Ix).map(|f| self.face_area_vector(f)).map(|v| v.dot(v).sqrt()).sum();
        let centroid = if six_v != 0.0 { o + moment * (0.25 / six_v) } else { o };
        let mut min = o;
        let mut max = o;
        for v in &self.vertices {
            let p = v.point;
            min = Vec3::new(min.x.min(p.x), min.y.min(p.y), min.z.min(p.z));
            max = Vec3::new(max.x.max(p.x), max.y.max(p.y), max.z.max(p.z));
        }
        MassProperties { volume: six_v / 6.0, area, centroid, min, max }
    }

    /// The same body moved by `m`. Topology IDs are unchanged; hashes follow the new coordinates.
    pub fn transformed(&self, m: &Rigid) -> Body {
        self.mapped(|p| m.apply(p), |n| m.apply_vector(n))
    }

    /// The body under an affine map that keeps orientation: `point` maps points, `normal` maps
    /// normals (by the inverse transpose). Planes stay planes; topology IDs are unchanged.
    pub fn mapped(&self, point: impl Fn(Vec3) -> Vec3, normal: impl Fn(Vec3) -> Vec3) -> Body {
        let mut b = self.clone();
        for v in &mut b.vertices {
            v.point = point(v.point);
        }
        for f in &mut b.faces {
            f.plane = Plane { origin: point(f.plane.origin), normal: normal(f.plane.normal) };
        }
        b.rehash();
        b
    }

    /// Face `f` as builder input.
    pub fn face_spec(&self, f: Ix) -> FaceSpec {
        let face = &self.faces[f as usize];
        let loops = self
            .face_loops(f)
            .map(|l| self.loop_half_edges(l).map(|h| self.vertices[self.half_edges[h as usize].origin as usize].id).collect())
            .collect();
        FaceSpec { id: face.id, plane: face.plane, loops }
    }

    /// For every face, the lowest face index of its shell: faces joined by edges form a shell.
    pub(crate) fn face_shells(&self) -> Vec<Ix> {
        let mut parent: Vec<Ix> = (0..self.faces.len() as Ix).collect();
        fn find(p: &mut [Ix], mut x: Ix) -> Ix {
            while p[x as usize] != x {
                p[x as usize] = p[p[x as usize] as usize];
                x = p[x as usize];
            }
            x
        }
        let face = |h: Ix| self.loops[self.half_edges[h as usize].loop_ix as usize].face;
        for he in &self.half_edges {
            let (a, b) = (find(&mut parent, face(he.twin)), find(&mut parent, self.loops[he.loop_ix as usize].face));
            parent[a.max(b) as usize] = a.min(b);
        }
        (0..self.faces.len() as Ix).map(|f| find(&mut parent, f)).collect()
    }

    /// One vertex of every shell, shells in the order of their first faces.
    pub fn shell_vertices(&self) -> Vec<Ix> {
        let shells = self.face_shells();
        (0..self.faces.len())
            .filter(|&f| shells[f] == f as Ix)
            .map(|f| self.half_edges[self.loops[self.faces[f].first_loop as usize].first as usize].origin)
            .collect()
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
