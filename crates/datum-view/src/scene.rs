//! What the viewer draws: the bodies and sketches of a model as triangles and lines, in `f64`
//! world coordinates. GPU vertices are made relative to an anchor near the model, in `f64`,
//! before rounding to `f32` (D24): a site ten thousand kilometres from the origin draws as
//! steadily as a bolt.

use std::collections::HashMap;
use std::f64::consts::TAU;
use std::sync::Arc;

use datum_brep::{Body, Ix, Surface, TopoId, edge_points, face_mesh};
use datum_graph::Value;
use datum_math::{Vec2, Vec3};
use rayon::prelude::*;

use crate::sketch::SketchEdit;

/// Cosine of the smallest angle between face normals at an edge drawn as a line (20°).
const SHARP: f64 = 0.939_692_620_785_908_4;

/// A body's triangles and edges. Cached by the body's content hash: an unchanged body is never
/// tessellated again.
#[derive(Default)]
pub struct BodyMesh {
    /// Corners of each triangle, counterclockwise seen from outside.
    pub triangles: Vec<[Vec3; 3]>,
    /// The surface normal at each corner, unit length: flat on planes, round on cylinders.
    pub normals: Vec<[[f32; 3]; 3]>,
    /// The face each triangle is of: what a click on it picks.
    pub faces: Vec<TopoId>,
    /// B-rep edges (not triangle edges), arcs as chords: what an engineer reads a part by.
    pub edges: Vec<[Vec3; 2]>,
    /// What can become the outline of a sphere or a torus on screen.
    pub contours: Vec<Contour>,
}

/// An edge of the triangles of spheres and tori: drawn only while one of its two triangles faces
/// the eye and the other does not, so that the lines drawn are the outline from where one looks.
#[derive(Clone, Copy)]
pub struct Contour {
    ends: [Vec3; 2],
    /// Which way the surface looks at each of the two triangles, of any length: only the side
    /// matters. The surface's normals at the corners: a thin triangle along a seam has a plane of
    /// its own, nothing like the surface.
    normals: [[f32; 3]; 2],
}

/// The contours of `triangles` among those numbered `round`: edges two of them share. Across a
/// seam of a sphere too: faces next to each other have the same points along their common edge.
fn contours(triangles: &[[Vec3; 3]], normals: &[[[f32; 3]; 3]], round: &[usize]) -> Vec<Contour> {
    // Adding zero makes -0 the same as 0.
    let bits = |p: Vec3| [p.x, p.y, p.z].map(|c| (c + 0.0).to_bits());
    let mut sides: Vec<_> = round
        .iter()
        .flat_map(|&t| {
            (0..3).map(move |k| {
                let (a, b) = (bits(triangles[t][k]), bits(triangles[t][(k + 1) % 3]));
                (if a < b { [a, b] } else { [b, a] }, t)
            })
        })
        .collect();
    sides.par_sort_unstable();
    let normal = |t: usize| [0, 1, 2].map(|k| normals[t].iter().map(|n| n[k]).sum::<f32>());
    let point = |p: [u64; 3]| Vec3::new(f64::from_bits(p[0]), f64::from_bits(p[1]), f64::from_bits(p[2]));
    sides.windows(2).filter(|w| w[0].0 == w[1].0).map(|w| Contour { ends: w[0].0.map(point), normals: [normal(w[0].1), normal(w[1].1)] }).collect()
}

impl BodyMesh {
    /// The mesh of `body`; `formed` is a body that is one tool's surface (a rib, D250), whose
    /// tangent seams are not edges a reader needs.
    pub fn new(body: &Body, formed: bool) -> Self {
        let chord = if body.has_curves() { body.chord() } else { 0.0 };
        let f32s = |n: Vec3| [n.x as f32, n.y as f32, n.z as f32];
        let faces: Vec<Self> = (0..body.faces().len() as Ix)
            .into_par_iter()
            .map(|f| {
                // A face that cannot be triangulated is left out of the picture, not the program.
                let Ok(m) = face_mesh(body, f, true, chord) else { return Self::default() };
                let corners = |t: &[u32; 3], v: &[Vec3]| t.map(|k| v[k as usize]);
                Self {
                    triangles: m.triangles.iter().map(|t| corners(t, &m.points)).collect(),
                    normals: m.triangles.iter().map(|t| corners(t, &m.normals).map(f32s)).collect(),
                    faces: vec![body.faces()[f as usize].id; m.triangles.len()],
                    ..Self::default()
                }
            })
            .collect();
        let mut mesh = Self::default();
        let mut round = Vec::new();
        for (f, face) in faces.into_iter().zip(body.faces()) {
            if matches!(face.surface, Surface::Torus(_)) {
                round.extend(mesh.triangles.len()..mesh.triangles.len() + f.triangles.len());
            }
            mesh.triangles.extend(f.triangles);
            mesh.normals.extend(f.normals);
            mesh.faces.extend(f.faces);
        }
        mesh.contours = contours(&mesh.triangles, &mesh.normals, &round);
        // Edges where the surface turns, where a flat face goes smoothly into a curved one, and
        // where a torus or a sphere ends: those lines are where a bend or a rib starts. A rib is
        // one tool's surface: its strips meet tangentially, and those seams say nothing on screen;
        // its outline is the foot, where a flat face meets a curved one. Between nearly coplanar
        // faces (a terrain's cells, a split face) or along the seams of a cylinder an edge says
        // nothing either. A display threshold; the model never sees it.
        let surface = |h: Ix| &body.faces()[body.loops()[body.half_edges()[h as usize].loop_ix as usize].face as usize].surface;
        let flat = |s: &Surface| matches!(s, Surface::Plane(_));
        let round = |s: &Surface| matches!(s, Surface::Torus(_));
        for (e, edge) in body.edges().iter().enumerate() {
            let pts = edge_points(body, e as Ix, chord);
            let p = pts[0];
            let (a, b) = (surface(edge.half), surface(body.half_edges()[edge.half as usize].twin));
            if flat(a) != flat(b) || (!formed && (round(a) || round(b)) && a != b) || a.normal_at(p).dot(b.normal_at(p)) < SHARP {
                mesh.edges.extend(pts.windows(2).map(|w| [w[0], w[1]]));
            }
        }
        mesh
    }
}

/// The chords of a curve of sketch `p` through `pts`, in place: in the sketch's plane, or on its
/// cylinder face, where each bends with the face: 64 chords a turn.
pub fn chords(p: &datum_ops::Placed, pts: Vec<[f64; 2]>, out: &mut Vec<[Vec3; 2]>) {
    let [o, x, y, _] = p.frame;
    let place = |q: Vec2| match &p.developed {
        None => o + x * q.x + y * q.y,
        Some(d) => d.point(q, d.radius),
    };
    for w in pts.windows(2) {
        let (a, b) = (w[0], w[1]);
        let n = p.developed.as_ref().map_or(1, |d| ((b[0] - a[0]).abs() / d.dev.neutral / TAU * 64.0).ceil().max(1.0) as usize);
        let at = |k: usize| {
            let t = k as f64 / n as f64;
            place(Vec2::new(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
        };
        out.extend((0..n).map(|k| [at(k), at(k + 1)]));
    }
}

/// Meshes of bodies by content hash and the way they are drawn (lookups only: the map's order
/// never shows).
#[derive(Default)]
pub struct MeshCache(HashMap<([u8; 32], bool), Arc<BodyMesh>>);

impl MeshCache {
    pub fn get(&mut self, body: &Body, formed: bool) -> Arc<BodyMesh> {
        self.0.entry((body.hash(), formed)).or_insert_with(|| Arc::new(BodyMesh::new(body, formed))).clone()
    }

    /// Keeps only the meshes of `bodies`: memory follows the model, not its history.
    pub fn retain(&mut self, bodies: &[&Body]) {
        let keep: Vec<[u8; 32]> = bodies.iter().map(|b| b.hash()).collect();
        self.0.retain(|(h, _), _| keep.contains(h));
    }
}

/// The drawable model: body meshes, sketch lines, and their bounds.
#[derive(Clone, Default)]
pub struct Scene {
    pub bodies: Vec<Arc<BodyMesh>>,
    pub sketch_lines: Vec<[Vec3; 2]>,
    pub bounds: Option<(Vec3, Vec3)>,
    /// The rest of the product around what is worked on (D160): drawn, not picked; `faded`, by
    /// its edges alone, faint, so as not to hide the part being worked on.
    pub context: Vec<Arc<BodyMesh>>,
    pub faded: bool,
    /// Which of `bodies` are turned copies of a part (D247): picked as any, drawn in their own
    /// colour.
    pub copies: Vec<usize>,
}

impl Scene {
    /// The scene of `values`: bodies as meshes, sketches as their lines in place.
    pub fn new(values: &[&Value], cache: &mut MeshCache) -> Self {
        let mut s = Self::default();
        for v in values {
            match v {
                Value::Body(b) => s.bodies.push(cache.get(b, false)),
                Value::Formed(f) => s.bodies.push(cache.get(&f.body, true)),
                // A flat blank is for export (DXF), not drawn in the 3D scene.
                // Work planes are drawn over the picture, from the model's inspection.
                Value::Flat(_) | Value::Plane(_) => {}
                Value::Sketch(p) => {
                    let curves = p.lines.keys().chain(p.arcs.keys()).chain(p.circles.keys()).chain(p.ellipses.keys());
                    for pts in curves.filter_map(|n| SketchEdit::curve_points(p, n)) {
                        chords(p, pts, &mut s.sketch_lines);
                    }
                    // The copies of patterns, as the profiles they make (the cell itself is drawn above).
                    for c in &p.patterns {
                        for (k, at) in &c.at {
                            if !(c.source && *k == [0, 0]) {
                                SketchEdit::copy_outline(p, c, at).into_iter().for_each(|pts| chords(p, pts, &mut s.sketch_lines));
                            }
                        }
                        c.chains.iter().for_each(|copy| chords(p, copy.outline(), &mut s.sketch_lines));
                    }
                }
            }
        }
        s.bound();
        s
    }

    /// The scene with `(body, formed)` around it, the rest of the product: drawn whole, or `faded`.
    pub fn with_context(mut self, bodies: &[(&Body, bool)], faded: bool, cache: &mut MeshCache) -> Self {
        self.context = bodies.iter().map(|&(b, formed)| cache.get(b, formed)).collect();
        self.faded = faded;
        self.bound();
        self
    }

    fn bound(&mut self) {
        let points = self.bodies.iter().chain(&self.context).flat_map(|m| m.triangles.iter().flatten()).chain(self.sketch_lines.iter().flatten());
        self.bounds = points.fold(None, |b, &p| {
            Some(match b {
                None => (p, p),
                Some((lo, hi)) => (Vec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z)), Vec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z))),
            })
        });
    }

    /// The body and face under a ray from `origin` along the unit `dir`: the nearest triangle it
    /// passes through, by Möller–Trumbore, how far along the ray, and the triangle's normal
    /// turned to the eye. A pick on the screen, not a geometric decision.
    pub fn pick(&self, origin: Vec3, dir: Vec3) -> Option<Hit> {
        let mut best: Option<Hit> = None;
        for (b, m) in self.bodies.iter().enumerate() {
            for (t, &face) in m.triangles.iter().zip(&m.faces) {
                let (e1, e2) = (t[1] - t[0], t[2] - t[0]);
                let p = dir.cross(e2);
                let det = e1.dot(p);
                if det == 0.0 {
                    continue;
                }
                let s = origin - t[0];
                let (u, q) = (s.dot(p) / det, s.cross(e1));
                let v = dir.dot(q) / det;
                let d = e2.dot(q) / det;
                if u >= 0.0 && v >= 0.0 && u + v <= 1.0 && d > 0.0 && best.is_none_or(|h| d < h.distance) {
                    let n = e1.cross(e2);
                    let n = n * (1.0 / n.dot(n).sqrt());
                    best = Some(Hit { distance: d, body: b, face, normal: if n.dot(dir) > 0.0 { -n } else { n } });
                }
            }
        }
        best
    }

    /// The point vertices are made relative to: the middle of the bounds.
    pub fn anchor(&self) -> Vec3 {
        self.bounds.map_or(Vec3::ZERO, |(lo, hi)| (lo + hi) * 0.5)
    }
}

/// A face under the cursor (see [`Scene::pick`]).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Hit {
    pub distance: f64,
    pub body: usize,
    pub face: TopoId,
    /// Unit, towards the eye.
    pub normal: Vec3,
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
    /// Per vertex: position, the edge's middle and its two triangles' normals (3 × f32 each),
    /// colour (4 × u8); pairs of indices.
    pub contours: Part,
}

pub const TRIANGLE_VERTEX: u64 = 28;
pub const LINE_VERTEX: u64 = 16;
pub const CONTOUR_VERTEX: u64 = 52;

const BODY: [u8; 4] = [168, 182, 199, 255];
const EDGE: [u8; 4] = [32, 36, 44, 255];
/// A turned copy of a part (D247): a warm grey beside the cold one of the parts.
const COPY: [u8; 4] = [206, 196, 178, 255];
const SKETCH: [u8; 4] = [214, 96, 32, 255];
/// The rest of the product, faded: its edges, faint.
const FADED: [u8; 4] = [150, 158, 170, 255];

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
        // Solid: the bodies, and the product around them unless it is faded.
        let solid: Vec<&BodyMesh> = scene.bodies.iter().chain(scene.context.iter().filter(|_| !scene.faded)).map(|m| &**m).collect();
        let bodies = &solid;
        let colours: Vec<[u8; 4]> = (0..solid.len()).map(|b| if scene.copies.contains(&b) { COPY } else { BODY }).collect();
        let triangles = bodies.iter().enumerate().flat_map(|(b, m)| m.triangles.iter().enumerate().map(move |(i, t)| key(t, b, i))).collect();
        let triangles = Part::new(triangles, |b, i| {
            let (m, i) = (bodies[b as usize], i as usize);
            let (t, n) = (m.triangles[i], m.normals[i]);
            [0, 1, 2].map(|k| {
                let (p, n) = (rel(t[k]), n[k]);
                vertex::<28>(&[p[0], p[1], p[2], n[0], n[1], n[2]], colours[b as usize])
            })
        });
        // Edges of the bodies, of the product around them, then the sketch lines: a group each.
        let around = if scene.faded { FADED } else { EDGE };
        let groups: Vec<(&[[Vec3; 2]], [u8; 4])> = scene
            .bodies
            .iter()
            .map(|m| (&m.edges[..], EDGE))
            .chain(scene.context.iter().map(|m| (&m.edges[..], around)))
            .chain([(&scene.sketch_lines[..], SKETCH)])
            .collect();
        let lines = groups.iter().enumerate().flat_map(|(g, (e, _))| e.iter().enumerate().map(move |(i, e)| key(e, g, i))).collect::<Vec<_>>();
        let lines = Part::new(lines, |g, i| {
            let (e, colour) = groups[g as usize];
            e[i as usize].map(|p| vertex::<16>(&rel(p), colour))
        });
        // The outlines of the bodies, then of the product around them.
        let meshes: Vec<(&BodyMesh, [u8; 4])> =
            scene.bodies.iter().map(|m| (&**m, EDGE)).chain(scene.context.iter().map(|m| (&**m, around))).collect();
        let order = meshes.iter().enumerate().flat_map(|(g, (m, _))| m.contours.iter().enumerate().map(move |(i, c)| key(&c.ends, g, i))).collect();
        let contours = Part::new(order, |g, i| {
            let (m, colour) = meshes[g as usize];
            let Contour { ends, normals: [a, b] } = m.contours[i as usize];
            let mid = rel((ends[0] + ends[1]) * 0.5);
            ends.map(|p| {
                let p = rel(p);
                vertex::<52>(&[p[0], p[1], p[2], mid[0], mid[1], mid[2], a[0], a[1], a[2], b[0], b[1], b[2]], colour)
            })
        });
        Self { triangles, lines, contours }
    }
}
