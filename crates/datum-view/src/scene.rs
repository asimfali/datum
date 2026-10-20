//! What the viewer draws: the bodies and sketches of a model as triangles and lines, in `f64`
//! world coordinates. GPU vertices are made relative to an anchor near the model, in `f64`,
//! before rounding to `f32` (D24): a site ten thousand kilometres from the origin draws as
//! steadily as a bolt.

use std::collections::HashMap;
use std::f64::consts::TAU;
use std::sync::Arc;

use datum_brep::{Body, Ix, edge_points, face_mesh};
use datum_graph::Value;
use datum_math::{Vec2, Vec3};
use rayon::prelude::*;

/// Cosine of the smallest angle between face normals at an edge drawn as a line (20°).
const SHARP: f64 = 0.939_692_620_785_908_4;

/// A body's triangles and edges. Cached by the body's content hash: an unchanged body is never
/// tessellated again.
pub struct BodyMesh {
    /// Corners of each triangle, counterclockwise seen from outside.
    pub triangles: Vec<[Vec3; 3]>,
    /// The surface normal at each corner, unit length: flat on planes, round on cylinders.
    pub normals: Vec<[[f32; 3]; 3]>,
    /// B-rep edges (not triangle edges), arcs as chords: what an engineer reads a part by.
    pub edges: Vec<[Vec3; 2]>,
}

impl BodyMesh {
    pub fn new(body: &Body) -> Self {
        let chord = if body.has_curves() { body.chord() } else { 0.0 };
        let f32s = |n: Vec3| [n.x as f32, n.y as f32, n.z as f32];
        let faces: Vec<Self> = (0..body.faces().len() as Ix)
            .into_par_iter()
            .map(|f| {
                // A face that cannot be triangulated is left out of the picture, not the program.
                let Ok(m) = face_mesh(body, f, true, chord) else { return Self { triangles: Vec::new(), normals: Vec::new(), edges: Vec::new() } };
                let corners = |t: &[u32; 3], v: &[Vec3]| t.map(|k| v[k as usize]);
                Self {
                    triangles: m.triangles.iter().map(|t| corners(t, &m.points)).collect(),
                    normals: m.triangles.iter().map(|t| corners(t, &m.normals).map(f32s)).collect(),
                    edges: Vec::new(),
                }
            })
            .collect();
        let mut mesh = Self { triangles: Vec::new(), normals: Vec::new(), edges: Vec::new() };
        for f in faces {
            mesh.triangles.extend(f.triangles);
            mesh.normals.extend(f.normals);
        }
        // Edges where the surface turns: between nearly coplanar faces (a terrain's cells, a
        // split face) or along the seams of a cylinder an edge says nothing on screen. A display
        // threshold; the model never sees it.
        let normal = |h: Ix, p: Vec3| body.faces()[body.loops()[body.half_edges()[h as usize].loop_ix as usize].face as usize].surface.normal_at(p);
        for (e, edge) in body.edges().iter().enumerate() {
            let pts = edge_points(body, e as Ix, chord);
            let p = pts[0];
            if normal(edge.half, p).dot(normal(body.half_edges()[edge.half as usize].twin, p)) < SHARP {
                mesh.edges.extend(pts.windows(2).map(|w| [w[0], w[1]]));
            }
        }
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
                    // Arcs and circles as chords, in the sketch's plane.
                    let place = |q: Vec2| o + x * q.x + y * q.y;
                    let pts = &p.solved.points;
                    let arcs = p.arcs.values().filter_map(|a| {
                        let (c, f, t) = (*pts.get(&a.center)?, *pts.get(&a.from)?, *pts.get(&a.to)?);
                        let start = libm::atan2(f.y - c.y, f.x - c.x);
                        let sweep = (libm::atan2(t.y - c.y, t.x - c.x) - start).rem_euclid(TAU);
                        Some((c, (f.x - c.x).hypot(f.y - c.y), start, if sweep == 0.0 { TAU } else { sweep }))
                    });
                    let circles = p.circles.values().filter_map(|k| Some((*pts.get(&k.center)?, 0.5 * k.diameter.value()?, 0.0, TAU)));
                    for (c, r, start, sweep) in arcs.chain(circles) {
                        let n = (sweep / TAU * 64.0).ceil().max(1.0) as usize;
                        let at = |k: usize| {
                            let (sn, cs) = libm::sincos(start + sweep * k as f64 / n as f64);
                            place(Vec2::new(c.x + r * cs, c.y + r * sn))
                        };
                        s.sketch_lines.extend((0..n).map(|k| [at(k), at(k + 1)]));
                    }
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

/// GPU vertices of one kind, each once, and the indices that make primitives of them. The
/// primitives run along a Morton curve through the scene and are cut into chunks with bounds:
/// close up only the chunks in view are drawn, and a vertex shared by neighbouring primitives is
/// processed once.
#[derive(Clone, Default)]
pub struct Part {
    pub vertices: Vec<u8>,
    pub indices: Vec<u32>,
    pub chunks: Vec<Chunk>,
}

/// Primitives close together, by their bounds (relative to the anchor) and where their indices
/// and their own vertices end; they start where the previous chunk's end.
#[derive(Clone, Copy, Debug)]
pub struct Chunk {
    pub lo: [f32; 3],
    pub hi: [f32; 3],
    pub end: u32,
    pub vertex_end: u32,
}

/// Primitives per chunk: small enough to leave out most of a large model close up, large enough
/// that testing chunks against the view costs nothing.
const CHUNK: usize = 4096;

impl Part {
    /// The part of primitives `(group, item)` keyed by `order` (a Morton code first), each with
    /// corners `corners(group, item)`: vertex bytes, a position (3 × f32) first. Chunks are made
    /// in parallel, each with its own vertices: one shared across a chunk's border is repeated.
    fn new<const N: usize, const K: usize>(mut order: Vec<(u64, u32, u32)>, corners: impl Fn(u32, u32) -> [[u8; N]; K] + Sync) -> Self {
        // `(group, item)` is unique: the order is total.
        order.par_sort_unstable();
        let chunks: Vec<(Vec<u8>, Vec<u32>, Chunk)> = order
            .par_chunks(CHUNK)
            .map(|run| {
                let mut c = Chunk { lo: [f32::INFINITY; 3], hi: [f32::NEG_INFINITY; 3], end: 0, vertex_end: 0 };
                let (mut vertices, mut indices) = (Vec::new(), Vec::with_capacity(run.len() * K));
                // Lookups only: the order of vertices is the order of corners.
                let mut seen: HashMap<[u8; N], u32> = HashMap::with_capacity(run.len() * K);
                for &(_, group, item) in run {
                    for v in corners(group, item) {
                        for k in 0..3 {
                            let x = f32::from_le_bytes([v[4 * k], v[4 * k + 1], v[4 * k + 2], v[4 * k + 3]]);
                            c.lo[k] = c.lo[k].min(x);
                            c.hi[k] = c.hi[k].max(x);
                        }
                        let next = (vertices.len() / N) as u32;
                        indices.push(*seen.entry(v).or_insert_with(|| {
                            vertices.extend_from_slice(&v);
                            next
                        }));
                    }
                }
                (vertices, indices, c)
            })
            .collect();
        let mut part = Self::default();
        for (vertices, indices, mut c) in chunks {
            let base = (part.vertices.len() / N) as u32;
            part.vertices.extend_from_slice(&vertices);
            part.indices.extend(indices.iter().map(|i| i + base));
            c.end = part.indices.len() as u32;
            c.vertex_end = (part.vertices.len() / N) as u32;
            part.chunks.push(c);
        }
        part
    }
}

/// The Morton code of `p` in the box from `lo` of size `size`: 21 bits per axis, interleaved.
fn morton(p: Vec3, lo: Vec3, size: Vec3) -> u64 {
    const MAX: f64 = ((1u64 << 21) - 1) as f64;
    let spread = |x: f64, lo: f64, size: f64| {
        let mut v = if size > 0.0 { ((x - lo) / size * MAX).clamp(0.0, MAX) as u64 } else { 0 };
        v = (v | v << 32) & 0x001f_0000_0000_ffff;
        v = (v | v << 16) & 0x001f_0000_ff00_00ff;
        v = (v | v << 8) & 0x100f_00f0_0f00_f00f;
        v = (v | v << 4) & 0x10c3_0c30_c30c_30c3;
        (v | v << 2) & 0x1249_2492_4924_9249
    };
    spread(p.x, lo.x, size.x) | spread(p.y, lo.y, size.y) << 1 | spread(p.z, lo.z, size.z) << 2
}

/// GPU vertices, relative to an anchor.
#[derive(Clone, Default)]
pub struct Geometry {
    /// Per vertex: position (3 × f32), normal (3 × f32), colour (4 × u8); triples of indices.
    pub triangles: Part,
    /// Per vertex: position (3 × f32), colour (4 × u8); pairs of indices.
    pub lines: Part,
}

pub const TRIANGLE_VERTEX: u64 = 28;
pub const LINE_VERTEX: u64 = 16;

const BODY: [u8; 4] = [168, 182, 199, 255];
const EDGE: [u8; 4] = [32, 36, 44, 255];
const SKETCH: [u8; 4] = [214, 96, 32, 255];

/// A vertex's bytes: `floats`, then `colour`.
fn vertex<const N: usize>(floats: &[f32], colour: [u8; 4]) -> [u8; N] {
    let mut v = [0u8; N];
    for (chunk, f) in v.as_chunks_mut::<4>().0.iter_mut().zip(floats.iter().map(|f| f.to_le_bytes()).chain([colour])) {
        *chunk = f;
    }
    v
}

impl Geometry {
    /// Vertices of `scene` relative to `anchor`: subtracted in `f64`, then rounded (D24).
    pub fn new(scene: &Scene, anchor: Vec3) -> Self {
        let rel = |p: Vec3| {
            let d = p - anchor;
            [d.x as f32, d.y as f32, d.z as f32]
        };
        let (lo, hi) = scene.bounds.unwrap_or((Vec3::ZERO, Vec3::ZERO));
        let key = |points: &[Vec3], group: usize, item: usize| {
            let c = points.iter().fold(Vec3::ZERO, |s, &p| s + p) * (1.0 / points.len() as f64);
            (morton(c, lo, hi - lo), group as u32, item as u32)
        };
        let bodies = &scene.bodies;
        let triangles = bodies.iter().enumerate().flat_map(|(b, m)| m.triangles.iter().enumerate().map(move |(i, t)| key(t, b, i))).collect();
        let triangles = Part::new(triangles, |b, i| {
            let (m, i) = (&bodies[b as usize], i as usize);
            let (t, n) = (m.triangles[i], m.normals[i]);
            [0, 1, 2].map(|k| {
                let (p, n) = (rel(t[k]), n[k]);
                vertex::<28>(&[p[0], p[1], p[2], n[0], n[1], n[2]], BODY)
            })
        });
        // Sketch lines are the group after the bodies.
        let edges = |g: usize| if g < bodies.len() { &bodies[g].edges } else { &scene.sketch_lines };
        let lines = (0..=bodies.len()).flat_map(|g| edges(g).iter().enumerate().map(move |(i, e)| key(e, g, i))).collect::<Vec<_>>();
        let lines = Part::new(lines, |g, i| {
            let colour = if (g as usize) < bodies.len() { EDGE } else { SKETCH };
            edges(g as usize)[i as usize].map(|p| vertex::<16>(&rel(p), colour))
        });
        Self { triangles, lines }
    }
}
