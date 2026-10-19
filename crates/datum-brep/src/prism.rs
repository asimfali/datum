//! Extrusions: plane regions, possibly with holes, swept along Z; the prism as the simplest one.

use datum_math::{Sign, Vec2, Vec3, orient2d};

use crate::tess::segments_touch;
use crate::{Body, BrepError, FaceSpec, Plane, TopoId, Vertex, build};

/// A closed loop of a region and the IDs its extrusion gives: for each point the vertices at the
/// start and at the end, for each edge from point `k` to `k + 1` the side face.
#[derive(Clone, Debug)]
pub struct ProfileLoop {
    pub points: Vec<Vec2>,
    pub vertices: Vec<[TopoId; 2]>,
    pub sides: Vec<TopoId>,
}

impl ProfileLoop {
    /// The same loop the other way round, each edge keeping its side ID.
    fn reversed(&self) -> Self {
        let n = self.points.len();
        let mut r = Self { points: self.points.clone(), vertices: self.vertices.clone(), sides: self.sides.clone() };
        r.points.reverse();
        r.vertices.reverse();
        // New edge k runs from old point n−1−k to n−2−k: old edge n−2−k.
        r.sides = (0..n).map(|k| self.sides[(2 * n - 2 - k) % n]).collect();
        r
    }

    fn counterclockwise(&self) -> bool {
        let p = &self.points;
        let n = p.len();
        let i = (0..n).min_by(|&a, &b| p[a].y.total_cmp(&p[b].y).then(p[a].x.total_cmp(&p[b].x))).unwrap_or(0);
        orient2d(p[(i + n - 1) % n], p[i], p[(i + 1) % n]) == Sign::Positive
    }
}

/// A region: an outer loop, the holes in it, and the IDs of its start and end caps.
#[derive(Clone, Debug)]
pub struct Region {
    pub outer: ProfileLoop,
    pub holes: Vec<ProfileLoop>,
    pub caps: [TopoId; 2],
}

/// The regions (in the XY plane, loops in either orientation) swept from `z0` to `z1`, with the
/// IDs they carry. Loops must be simple, of at least three points, and touch neither each other
/// nor themselves; holes must lie in their outer loop.
pub fn extrude(regions: &[Region], z0: f64, z1: f64) -> Result<Body, BrepError> {
    let degenerate = |m: String| Err(BrepError::Degenerate(m));
    if !z0.is_finite() || !z1.is_finite() || z0 >= z1 {
        return degenerate("non-finite coordinates or zero/negative height".into());
    }
    let loops: Vec<&ProfileLoop> = regions.iter().flat_map(|r| std::iter::once(&r.outer).chain(&r.holes)).collect();
    for l in &loops {
        let n = l.points.len();
        if n < 3 || l.vertices.len() != n || l.sides.len() != n {
            return degenerate(format!("a profile loop has {n} points"));
        }
        if l.points.iter().any(|p| !p.x.is_finite() || !p.y.is_finite()) {
            return degenerate("non-finite coordinates".into());
        }
    }
    let edges: Vec<(Vec2, Vec2, usize, usize)> = loops
        .iter()
        .enumerate()
        .flat_map(|(li, l)| (0..l.points.len()).map(move |k| (l.points[k], l.points[(k + 1) % l.points.len()], li, k)))
        .collect();
    // Edges touch only their neighbours in the same loop, at the shared point.
    for (i, a) in edges.iter().enumerate() {
        for b in &edges[i + 1..] {
            let n = loops[a.2].points.len();
            let adjacent = a.2 == b.2 && ((a.3 + 1) % n == b.3 || (b.3 + 1) % n == a.3);
            if !adjacent && segments_touch(a.0, a.1, b.0, b.1) {
                return degenerate("profile loops touch or cross".into());
            }
        }
    }

    let mut vertices = Vec::new();
    let mut faces = Vec::new();
    for r in regions {
        let outer = if r.outer.counterclockwise() { r.outer.clone() } else { r.outer.reversed() };
        let holes: Vec<ProfileLoop> = r.holes.iter().map(|h| if h.counterclockwise() { h.reversed() } else { h.clone() }).collect();
        let base = |p: Vec2, z: f64| Vec3::new(p.x, p.y, z);
        let start = outer.points[0];
        let layer = |l: &ProfileLoop, layer: usize, reverse: bool| -> Vec<TopoId> {
            let ids = l.vertices.iter().map(|v| v[layer]);
            if reverse { ids.rev().collect() } else { ids.collect() }
        };
        faces.push(FaceSpec {
            id: r.caps[0],
            plane: Plane { origin: base(start, z0), normal: Vec3::new(0.0, 0.0, -1.0) },
            loops: std::iter::once(&outer).chain(&holes).map(|l| layer(l, 0, true)).collect(),
        });
        faces.push(FaceSpec {
            id: r.caps[1],
            plane: Plane { origin: base(start, z1), normal: Vec3::new(0.0, 0.0, 1.0) },
            loops: std::iter::once(&outer).chain(&holes).map(|l| layer(l, 1, false)).collect(),
        });
        for l in std::iter::once(&outer).chain(&holes) {
            let n = l.points.len();
            for k in 0..n {
                vertices.push(Vertex { id: l.vertices[k][0], point: base(l.points[k], z0) });
                vertices.push(Vertex { id: l.vertices[k][1], point: base(l.points[k], z1) });
                let (a, b) = (l.points[k], l.points[(k + 1) % n]);
                let (v, w) = (l.vertices[k], l.vertices[(k + 1) % n]);
                faces.push(FaceSpec {
                    id: l.sides[k],
                    plane: Plane { origin: base(a, z0), normal: Vec3::new(b.y - a.y, a.x - b.x, 0.0) },
                    loops: vec![vec![v[0], w[0], w[1], v[1]]],
                });
            }
        }
    }
    build(vertices, faces)
}

/// Prism over `polygon` (either orientation) from `z0` to `z1`, with IDs derived from `feature`.
///
/// Degenerate input is refused: fewer than 3 vertices, non-finite coordinates, `z0 ≥ z1`,
/// collinear neighbours (including repeated points) or a self-touching polygon.
pub fn prism(feature: TopoId, polygon: &[Vec2], z0: f64, z1: f64) -> Result<Body, BrepError> {
    let n = polygon.len();
    if n < 3 {
        return Err(BrepError::Degenerate(format!("polygon has {n} vertices")));
    }
    if polygon.iter().any(|p| !p.x.is_finite() || !p.y.is_finite()) {
        return Err(BrepError::Degenerate("non-finite coordinates or zero/negative height".into()));
    }
    let at = |i: usize| polygon[i % n];
    if (0..n).any(|i| orient2d(at(i), at(i + 1), at(i + 2)) == Sign::Zero) {
        return Err(BrepError::Degenerate("collinear or repeated neighbouring vertices".into()));
    }
    // IDs follow the counterclockwise order, whichever way the polygon was given.
    let mut points = polygon.to_vec();
    let mut ring = ProfileLoop { points: points.clone(), vertices: Vec::new(), sides: Vec::new() };
    if !ring.counterclockwise() {
        points.reverse();
    }
    let vid = |layer: u64, k: usize| TopoId::derive("prism.v").id(feature).index(layer).index(k as u64).finish();
    ring = ProfileLoop {
        vertices: (0..n).map(|k| [vid(0, k), vid(1, k)]).collect(),
        sides: (0..n).map(|k| TopoId::derive("prism.side").id(feature).index(k as u64).finish()).collect(),
        points,
    };
    let caps = [TopoId::derive("prism.cap_start").id(feature).finish(), TopoId::derive("prism.cap_end").id(feature).finish()];
    extrude(&[Region { outer: ring, holes: Vec::new(), caps }], z0, z1).map_err(|e| match e {
        BrepError::Degenerate(m) if m.contains("touch") => BrepError::Degenerate("polygon touches or crosses itself".into()),
        e => e,
    })
}

/// Box from `min` to `max`: a prism over a rectangle.
pub fn cuboid(feature: TopoId, min: Vec3, max: Vec3) -> Result<Body, BrepError> {
    let rect = [Vec2::new(min.x, min.y), Vec2::new(max.x, min.y), Vec2::new(max.x, max.y), Vec2::new(min.x, max.y)];
    prism(feature, &rect, min.z, max.z)
}
