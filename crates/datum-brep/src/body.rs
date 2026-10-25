//! The immutable body and its queries.

use std::ops::Range;

use datum_math::{Rigid, Vec3};
use rayon::prelude::*;
use smallvec::SmallVec;

use crate::curve::{self, Integrals, Path, unit};
use crate::{Arc, BrepError, Curve, CurveSpec, FaceSpec, Surface, TopoId};

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
    /// The edge's curve (see [`Body::edge_curve`]), an index into the body's curves: a plane
    /// body's edges stay small, and there are hundreds of thousands of them in a perforation.
    pub curve: Option<Ix>,
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
    pub surface: Surface,
    pub first_loop: Ix,
    pub loop_count: Ix,
    /// Merkle leaf: covers the face's ID, surface and loops (vertex IDs and points, arcs), not
    /// indices.
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
    /// Curves of edges, in edge order.
    pub(crate) curves: Vec<Curve>,
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

    /// The curve of an edge, from its lower-ID vertex; a straight edge has none.
    pub fn edge_curve(&self, e: &Edge) -> Option<Curve> {
        e.curve.map(|k| self.curves[k as usize])
    }

    /// The arc of an edge, if its curve is one.
    pub fn edge_arc(&self, e: &Edge) -> Option<Arc> {
        self.edge_curve(e).and_then(|c| c.arc().copied())
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

    pub(crate) fn reference(&self) -> Vec3 {
        self.vertices.first().map_or(Vec3::ZERO, |v| v.point)
    }

    /// The curves of face `f` as its loops run them: an arc turning about its axis in the
    /// direction of the loop, with its start and end, or a piece of a development.
    fn face_paths(&self, f: Ix) -> impl Iterator<Item = Path> + '_ {
        self.face_loops(f).flat_map(move |l| self.loop_half_edges(l)).filter_map(move |h| {
            let edge = &self.edges[self.half_edges[h as usize].edge as usize];
            let curve = self.edge_curve(edge)?;
            let curve = if edge.half == h { curve } else { curve.reversed() };
            Some(match curve {
                Curve::Arc(arc) => {
                    let (u, v) = (self.point(self.half_edges[h as usize].origin), self.point(self.half_edges[self.next(h) as usize].origin));
                    Path::Arc(arc, u, v)
                }
                Curve::Rolled { piece, .. } => Path::Rolled(piece),
            })
        })
    }

    /// The radii of the curves of face `f` that are pieces of a development.
    pub(crate) fn rolled_radii(&self, f: Ix) -> impl Iterator<Item = f64> + '_ {
        self.face_loops(f).flat_map(move |l| self.loop_half_edges(l)).filter_map(move |h| {
            match self.edge_curve(&self.edges[self.half_edges[h as usize].edge as usize])? {
                Curve::Rolled { radius, .. } => Some(radius),
                Curve::Arc(_) => None,
            }
        })
    }

    /// Adds face `f`'s volume and moment terms to `acc`, relative to `o`, and returns its area
    /// and area vector. A plane face is its fans, corrected by circular segments along its arcs;
    /// a cylinder face is integrated along its curves, a wall over itself between the radii of
    /// its edges.
    pub(crate) fn integrate_face(&self, f: Ix, o: Vec3, acc: &mut Integrals) -> (f64, Vec3) {
        match &self.faces[f as usize].surface {
            Surface::Plane(plane) => {
                let mut area_vector = Vec3::ZERO;
                self.fans(self.face_loops(f), o, |a, b, c| {
                    let v = a.dot(b.cross(c));
                    acc.six_v += v;
                    acc.moment = acc.moment + (a + b + c) * v;
                    area_vector = area_vector + (b - a).cross(c - a);
                });
                let mut seg = Integrals { area_vector: area_vector * 0.5, ..Integrals::default() };
                let n = unit(plane.normal);
                for path in self.face_paths(f) {
                    if let Path::Arc(arc, u, v) = path {
                        curve::segment(&mut seg, &arc, arc.axis, u, v, n, o);
                    }
                }
                acc.six_v += seg.six_v;
                acc.moment = acc.moment + seg.moment;
                (seg.area_vector.dot(seg.area_vector).sqrt(), seg.area_vector)
            }
            Surface::Cylinder(c) => {
                let mut cyl = Integrals::default();
                curve::cylinder(&mut cyl, c, self.face_paths(f), o);
                acc.six_v += cyl.six_v;
                acc.moment = acc.moment + cyl.moment;
                (cyl.area, cyl.area_vector)
            }
            Surface::Wall(w) => {
                let (r0, r1) = self.rolled_radii(f).fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), r| (lo.min(r), hi.max(r)));
                let mut wall = Integrals::default();
                curve::wall(&mut wall, w, r0, r1, o);
                acc.six_v += wall.six_v;
                acc.moment = acc.moment + wall.moment;
                (wall.area, wall.area_vector)
            }
        }
    }

    /// Enclosed volume by the divergence theorem, relative to the first vertex so that the result
    /// does not degrade with distance from the origin. Fixed summation order: deterministic.
    pub fn volume(&self) -> f64 {
        let (o, mut acc) = (self.reference(), Integrals::default());
        for f in 0..self.faces.len() as Ix {
            self.integrate_face(f, o, &mut acc);
        }
        acc.six_v / 6.0
    }

    /// Area vector of a face: its normal integrated over it (for a plane face, the normal scaled
    /// by the area, holes subtracted).
    pub fn face_area_vector(&self, f: Ix) -> Vec3 {
        self.integrate_face(f, self.reference(), &mut Integrals::default()).1
    }

    /// Area of a face.
    pub fn face_area(&self, f: Ix) -> f64 {
        self.integrate_face(f, self.reference(), &mut Integrals::default()).0
    }

    /// Volume, surface area, centre of mass (uniform density) and bounding box.
    pub fn mass_properties(&self) -> MassProperties {
        let (o, mut acc) = (self.reference(), Integrals::default());
        let area = (0..self.faces.len() as Ix).map(|f| self.integrate_face(f, o, &mut acc).0).sum();
        let centroid = if acc.six_v != 0.0 { o + acc.moment * (0.25 / acc.six_v) } else { o };
        let (min, max) = self.bounds();
        MassProperties { volume: acc.six_v / 6.0, area, centroid, min, max }
    }

    /// How far a chord may stray from an arc when the body is drawn or exported: a ten-thousandth
    /// of its size. A display tolerance; the model never sees it.
    pub fn chord(&self) -> f64 {
        let (lo, hi) = self.bounds();
        let d = hi - lo;
        1e-4 * d.dot(d).sqrt()
    }

    /// The bounding box: the vertices, and the points of curves beyond them.
    pub fn bounds(&self) -> (Vec3, Vec3) {
        let o = self.reference();
        let (mut min, mut max) = (o, o);
        for v in &self.vertices {
            let p = v.point;
            min = Vec3::new(min.x.min(p.x), min.y.min(p.y), min.z.min(p.z));
            max = Vec3::new(max.x.max(p.x), max.y.max(p.y), max.z.max(p.z));
        }
        for e in &self.edges {
            if let Some(curve) = &self.edge_curve(e) {
                let he = &self.half_edges[e.half as usize];
                curve.extend_bounds(self.point(he.origin), self.point(self.half_edges[he.twin as usize].origin), &mut min, &mut max);
            }
        }
        (min, max)
    }

    /// The box of face `f` moved by `m`: the box of its edges, which holds the face (a plane
    /// face, and the cylinders and walls between two curves the kernel builds).
    pub fn face_bounds(&self, f: Ix, m: &Rigid) -> (Vec3, Vec3) {
        let (mut lo, mut hi) =
            (Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY), Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY));
        for h in self.face_loops(f).flat_map(|l| self.loop_half_edges(l)) {
            let he = &self.half_edges[h as usize];
            let (u, v) = (m.apply(self.point(he.origin)), m.apply(self.point(self.half_edges[self.next(h) as usize].origin)));
            lo = Vec3::new(lo.x.min(u.x), lo.y.min(u.y), lo.z.min(u.z));
            hi = Vec3::new(hi.x.max(u.x), hi.y.max(u.y), hi.z.max(u.z));
            let edge = &self.edges[he.edge as usize];
            if let Some(c) = self.edge_curve(edge) {
                let c = if edge.half == h { c } else { c.reversed() };
                c.moved(m).extend_bounds(u, v, &mut lo, &mut hi);
            }
        }
        (lo, hi)
    }

    /// Whether some face is curved or some edge an arc: operations on plane geometry only refuse
    /// such bodies.
    pub fn has_curves(&self) -> bool {
        !self.curves.is_empty() || self.faces.iter().any(|f| f.surface.plane().is_none())
    }

    /// The same body moved by `m`. Topology IDs are unchanged; hashes follow the new coordinates.
    pub fn transformed(&self, m: &Rigid) -> Body {
        let mut b = self.clone();
        for v in &mut b.vertices {
            v.point = m.apply(v.point);
        }
        for f in &mut b.faces {
            f.surface = f.surface.moved(m);
        }
        for c in &mut b.curves {
            *c = c.moved(m);
        }
        b.rehash();
        b
    }

    /// The body under an affine map that keeps orientation: `point` maps points, `normal` maps
    /// normals (by the inverse transpose). Planes stay planes; topology IDs are unchanged. Arcs
    /// and cylinders would not stay circular: a body with them is refused.
    pub fn mapped(&self, point: impl Fn(Vec3) -> Vec3, normal: impl Fn(Vec3) -> Vec3) -> Result<Body, BrepError> {
        if self.has_curves() {
            return Err(BrepError::Unsupported("an affine map of a body with arcs or cylinders".into()));
        }
        let mut b = self.clone();
        for v in &mut b.vertices {
            v.point = point(v.point);
        }
        for f in &mut b.faces {
            if let Surface::Plane(p) = &mut f.surface {
                *p = Plane { origin: point(p.origin), normal: normal(p.normal) };
            }
        }
        b.rehash();
        Ok(b)
    }

    /// Face `f` as builder input.
    pub fn face_spec(&self, f: Ix) -> FaceSpec {
        let face = &self.faces[f as usize];
        let loops = self
            .face_loops(f)
            .map(|l| self.loop_half_edges(l).map(|h| self.vertices[self.half_edges[h as usize].origin as usize].id).collect())
            .collect();
        FaceSpec { id: face.id, surface: face.surface, loops }
    }

    /// The curves of the body as builder input, each from its edge's lower-ID vertex.
    pub fn curve_specs(&self) -> Vec<CurveSpec> {
        let id = |h: Ix| self.vertices[self.half_edges[h as usize].origin as usize].id;
        self.edges
            .iter()
            .filter_map(|e| Some(CurveSpec { from: id(e.half), to: id(self.half_edges[e.half as usize].twin), curve: self.edge_curve(e)? }))
            .collect()
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

    /// For every vertex, the number of its shell, shells numbered in the order of their first
    /// faces (as in [`Body::shell_vertices`]).
    pub fn vertex_shells(&self) -> Vec<u32> {
        let shells = self.face_shells();
        let mut number = vec![u32::MAX; self.faces.len()];
        let mut next = 0;
        for f in 0..self.faces.len() {
            if shells[f] == f as Ix {
                number[f] = next;
                next += 1;
            }
        }
        let mut of_vertex = vec![0; self.vertices.len()];
        for he in &self.half_edges {
            of_vertex[he.origin as usize] = number[shells[self.loops[he.loop_ix as usize].face as usize] as usize];
        }
        of_vertex
    }

    /// One vertex of every shell, shells in the order of their first faces.
    pub fn shell_vertices(&self) -> Vec<Ix> {
        let shells = self.face_shells();
        (0..self.faces.len())
            .filter(|&f| shells[f] == f as Ix)
            .map(|f| self.half_edges[self.loops[self.faces[f].first_loop as usize].first as usize].origin)
            .collect()
    }

    /// Face hashes in parallel, then the body hash over them in face order.
    pub(crate) fn rehash(&mut self) {
        let curves = !self.curves.is_empty();
        let hashes: Vec<[u8; 32]> = (0..self.faces.len() as Ix).into_par_iter().map(|f| self.face_hash(f, curves)).collect();
        for (face, hash) in self.faces.iter_mut().zip(hashes) {
            face.hash = hash;
        }
        let mut h = blake3::Hasher::new();
        for f in &self.faces {
            h.update(&f.hash);
        }
        self.hash = *h.finalize().as_bytes();
    }

    /// The Merkle leaf of face `f`; `curves` tells whether the body has curved edges at all, so
    /// that plane bodies (perforations by the ten thousand) skip looking them up.
    pub(crate) fn face_hash(&self, f: Ix, curves: bool) -> [u8; 32] {
        // The bytes are gathered first and hashed in one call: the same hash as streaming them,
        // without a streaming hasher per small face.
        let face = &self.faces[f as usize];
        let mut b: SmallVec<[u8; 512]> = SmallVec::new();
        b.extend_from_slice(&face.id.0.to_le_bytes());
        fn floats(b: &mut SmallVec<[u8; 512]>, v: &[f64]) {
            for c in v {
                b.extend_from_slice(&c.to_bits().to_le_bytes());
            }
        }
        fn rolled(b: &mut SmallVec<[u8; 512]>, w: &crate::Rolled) {
            floats(b, &w.dev.floats());
            floats(b, &[w.from.x, w.from.y, w.to.x, w.to.y]);
            if let Some(a) = w.arc {
                floats(b, &[a.center.x, a.center.y]);
                b.push(u8::from(a.ccw));
            }
        }
        match &face.surface {
            // A plane hashes as before cylinders existed: pinned hashes of plane bodies stay.
            Surface::Plane(p) => {
                floats(&mut b, &p.origin.to_array());
                floats(&mut b, &p.normal.to_array());
            }
            Surface::Cylinder(c) => {
                b.extend_from_slice(b"cylinder");
                floats(&mut b, &c.origin.to_array());
                floats(&mut b, &c.axis.to_array());
                floats(&mut b, &[c.radius]);
                b.push(u8::from(c.convex));
            }
            Surface::Wall(w) => {
                b.extend_from_slice(b"wall");
                rolled(&mut b, w);
            }
        }
        for l in self.face_loops(f) {
            b.extend_from_slice(&self.loops[l as usize].len.to_le_bytes());
            for he in self.loop_half_edges(l) {
                let v = &self.vertices[self.half_edges[he as usize].origin as usize];
                b.extend_from_slice(&v.id.0.to_le_bytes());
                floats(&mut b, &v.point.to_array());
                let curve = if curves { self.edges[self.half_edges[he as usize].edge as usize].curve } else { None };
                match curve.map(|k| &self.curves[k as usize]) {
                    Some(Curve::Arc(a)) => {
                        floats(&mut b, &a.center.to_array());
                        floats(&mut b, &a.axis.to_array());
                        floats(&mut b, &[a.radius]);
                    }
                    Some(Curve::Rolled { piece, radius }) => {
                        b.extend_from_slice(b"rolled");
                        rolled(&mut b, piece);
                        floats(&mut b, &[*radius]);
                    }
                    None => {}
                }
            }
        }
        *blake3::hash(&b).as_bytes()
    }
}
