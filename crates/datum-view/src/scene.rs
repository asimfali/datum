//! What the viewer draws: the bodies and sketches of a model as triangles and lines, in `f64`
//! world coordinates. GPU vertices are made relative to an anchor near the model, in `f64`,
//! before rounding to `f32` (D24): a site ten thousand kilometres from the origin draws as
//! steadily as a bolt.

use std::collections::HashMap;
use std::sync::Arc;

use datum_brep::{Body, Ix, triangulate_face};
use datum_graph::Value;
use datum_math::Vec3;
use rayon::prelude::*;

/// Cosine of the smallest angle between face normals at an edge drawn as a line (20°).
const SHARP: f64 = 0.939_692_620_785_908_4;

/// A body's triangles and edges. Cached by the body's content hash: an unchanged body is never
/// tessellated again.
pub struct BodyMesh {
    /// Corners of each triangle, counterclockwise seen from outside.
    pub triangles: Vec<[Vec3; 3]>,
    /// The face normal of each triangle, unit length.
    pub normals: Vec<[f32; 3]>,
    /// B-rep edges (not triangle edges): what an engineer reads a part by.
    pub edges: Vec<[Vec3; 2]>,
}

impl BodyMesh {
    pub fn new(body: &Body) -> Self {
        let faces: Vec<(Vec<[Vec3; 3]>, [f32; 3])> = (0..body.faces().len() as Ix)
            .into_par_iter()
            .map(|f| {
                let n = body.faces()[f as usize].plane.normal;
                let len = n.dot(n).sqrt();
                let normal = if len > 0.0 { [(n.x / len) as f32, (n.y / len) as f32, (n.z / len) as f32] } else { [0.0; 3] };
                // A face that cannot be triangulated is left out of the picture, not the program.
                let tris = triangulate_face(body, f).map(|t| t.iter().map(|t| t.map(|v| body.point(v))).collect()).unwrap_or_default();
                (tris, normal)
            })
            .collect();
        let mut mesh = Self { triangles: Vec::new(), normals: Vec::new(), edges: Vec::new() };
        for (tris, normal) in faces {
            mesh.normals.extend(std::iter::repeat_n(normal, tris.len()));
            mesh.triangles.extend(tris);
        }
        // Edges where the surface turns: between nearly coplanar faces (a terrain's cells, a
        // split face) an edge says nothing on screen. A display threshold; the model never sees it.
        let normal = |h: Ix| {
            let n = body.faces()[body.loops()[body.half_edges()[h as usize].loop_ix as usize].face as usize].plane.normal;
            n * (1.0 / n.dot(n).sqrt())
        };
        mesh.edges = body
            .edges()
            .iter()
            .filter(|e| normal(e.half).dot(normal(body.half_edges()[e.half as usize].twin)) < SHARP)
            .map(|e| {
                let he = &body.half_edges()[e.half as usize];
                [body.point(he.origin), body.point(body.half_edges()[he.twin as usize].origin)]
            })
            .collect();
        mesh
    }
}

/// Meshes of bodies by content hash (lookups only: the map's order never shows).
#[derive(Default)]
pub struct MeshCache(HashMap<[u8; 32], Arc<BodyMesh>>);

impl MeshCache {
    pub fn get(&mut self, body: &Body) -> Arc<BodyMesh> {
        self.0.entry(body.hash()).or_insert_with(|| Arc::new(BodyMesh::new(body))).clone()
    }

    /// Keeps only the meshes of `bodies`: memory follows the model, not its history.
    pub fn retain(&mut self, bodies: &[&Body]) {
        let keep: Vec<[u8; 32]> = bodies.iter().map(|b| b.hash()).collect();
        self.0.retain(|h, _| keep.contains(h));
    }
}

/// The drawable model: body meshes, sketch lines, and their bounds.
#[derive(Clone, Default)]
pub struct Scene {
    pub bodies: Vec<Arc<BodyMesh>>,
    pub sketch_lines: Vec<[Vec3; 2]>,
    pub bounds: Option<(Vec3, Vec3)>,
}

impl Scene {
    /// The scene of `values`: bodies as meshes, sketches as their lines in place.
    pub fn new(values: &[&Value], cache: &mut MeshCache) -> Self {
        let mut s = Self::default();
        for v in values {
            match v {
                Value::Body(b) => s.bodies.push(cache.get(b)),
                Value::Sketch(p) => {
                    let [o, x, y, _] = p.frame;
                    let at = |name: &str| p.solved.points.get(name).map(|q| o + x * q.x + y * q.y);
                    s.sketch_lines.extend(p.lines.values().filter_map(|l| Some([at(&l.from)?, at(&l.to)?])));
                }
            }
        }
        let points = s.bodies.iter().flat_map(|m| m.triangles.iter().flatten()).chain(s.sketch_lines.iter().flatten());
        s.bounds = points.fold(None, |b, &p| {
            Some(match b {
                None => (p, p),
                Some((lo, hi)) => (Vec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z)), Vec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z))),
            })
        });
        s
    }

    /// The point vertices are made relative to: the middle of the bounds.
    pub fn anchor(&self) -> Vec3 {
        self.bounds.map_or(Vec3::ZERO, |(lo, hi)| (lo + hi) * 0.5)
    }
}

/// GPU vertices, relative to an anchor.
pub struct Geometry {
    /// Per vertex: position (3 × f32), normal (3 × f32), colour (4 × u8).
    pub triangles: Vec<u8>,
    /// Per vertex: position (3 × f32), colour (4 × u8); pairs make segments.
    pub lines: Vec<u8>,
}

pub const TRIANGLE_VERTEX: u64 = 28;
pub const LINE_VERTEX: u64 = 16;

const BODY: [u8; 4] = [168, 182, 199, 255];
const EDGE: [u8; 4] = [32, 36, 44, 255];
const SKETCH: [u8; 4] = [214, 96, 32, 255];

impl Geometry {
    /// Vertices of `scene` relative to `anchor`: subtracted in `f64`, then rounded (D24).
    pub fn new(scene: &Scene, anchor: Vec3) -> Self {
        let rel = |p: Vec3| {
            let d = p - anchor;
            [d.x as f32, d.y as f32, d.z as f32]
        };
        let mut triangles = Vec::new();
        let mut lines = Vec::new();
        let push = |out: &mut Vec<u8>, floats: &[f32], colour: [u8; 4]| {
            for f in floats {
                out.extend_from_slice(&f.to_le_bytes());
            }
            out.extend_from_slice(&colour);
        };
        for m in &scene.bodies {
            for (t, n) in m.triangles.iter().zip(&m.normals) {
                for &p in t {
                    let p = rel(p);
                    push(&mut triangles, &[p[0], p[1], p[2], n[0], n[1], n[2]], BODY);
                }
            }
            for e in &m.edges {
                for &p in e {
                    push(&mut lines, &rel(p), EDGE);
                }
            }
        }
        for e in &scene.sketch_lines {
            for &p in e {
                push(&mut lines, &rel(p), SKETCH);
            }
        }
        Self { triangles, lines }
    }
}
