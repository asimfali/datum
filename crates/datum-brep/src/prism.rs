//! Prisms: a simple polygon in the XY plane extruded along Z.

use datum_math::{Sign, Vec2, Vec3, orient2d};

use crate::{Body, BrepError, FaceSpec, Plane, TopoId, Vertex, build};

/// Prism over `polygon` (either orientation) from `z0` to `z1`, with IDs derived from `feature`.
///
/// Degenerate input is refused: fewer than 3 vertices, non-finite coordinates, `z0 ≥ z1`,
/// collinear neighbours (including repeated points) or a self-touching polygon.
pub fn prism(feature: TopoId, polygon: &[Vec2], z0: f64, z1: f64) -> Result<Body, BrepError> {
    let n = polygon.len();
    if n < 3 {
        return Err(BrepError::Degenerate(format!("polygon has {n} vertices")));
    }
    if !z0.is_finite() || !z1.is_finite() || z0 >= z1 || polygon.iter().any(|p| !p.x.is_finite() || !p.y.is_finite()) {
        return Err(BrepError::Degenerate("non-finite coordinates or zero/negative height".into()));
    }
    let at = |i: usize| polygon[i % n];
    if (0..n).any(|i| orient2d(at(i), at(i + 1), at(i + 2)) == Sign::Zero) {
        return Err(BrepError::Degenerate("collinear or repeated neighbouring vertices".into()));
    }
    if !is_simple(polygon) {
        return Err(BrepError::Degenerate("polygon touches or crosses itself".into()));
    }
    let mut pts = polygon.to_vec();
    if !is_counterclockwise(&pts) {
        pts.reverse();
    }

    let vid = |layer: u64, k: usize| TopoId::derive("prism.v").id(feature).index(layer).index(k as u64).finish();
    let vertices = (0..2u64)
        .flat_map(|layer| pts.iter().enumerate().map(move |(k, p)| (layer, k, *p)))
        .map(|(layer, k, p)| Vertex { id: vid(layer, k), point: Vec3::new(p.x, p.y, if layer == 0 { z0 } else { z1 }) })
        .collect();

    let base = |p: Vec2, z: f64| Vec3::new(p.x, p.y, z);
    let mut faces = Vec::with_capacity(n + 2);
    faces.push(FaceSpec {
        id: TopoId::derive("prism.cap_start").id(feature).finish(),
        plane: Plane { origin: base(pts[0], z0), normal: Vec3::new(0.0, 0.0, -1.0) },
        loops: vec![(0..n).rev().map(|k| vid(0, k)).collect()],
    });
    faces.push(FaceSpec {
        id: TopoId::derive("prism.cap_end").id(feature).finish(),
        plane: Plane { origin: base(pts[0], z1), normal: Vec3::new(0.0, 0.0, 1.0) },
        loops: vec![(0..n).map(|k| vid(1, k)).collect()],
    });
    for k in 0..n {
        let (a, b) = (pts[k], pts[(k + 1) % n]);
        faces.push(FaceSpec {
            id: TopoId::derive("prism.side").id(feature).index(k as u64).finish(),
            plane: Plane { origin: base(a, z0), normal: Vec3::new(b.y - a.y, a.x - b.x, 0.0) },
            loops: vec![vec![vid(0, k), vid(0, (k + 1) % n), vid(1, (k + 1) % n), vid(1, k)]],
        });
    }
    build(vertices, faces)
}

/// Box from `min` to `max`: a prism over a rectangle.
pub fn cuboid(feature: TopoId, min: Vec3, max: Vec3) -> Result<Body, BrepError> {
    let rect = [Vec2::new(min.x, min.y), Vec2::new(max.x, min.y), Vec2::new(max.x, max.y), Vec2::new(min.x, max.y)];
    prism(feature, &rect, min.z, max.z)
}

/// Exact orientation: the turn at the lowest (then leftmost) vertex has the polygon's sign.
fn is_counterclockwise(p: &[Vec2]) -> bool {
    let n = p.len();
    let i = (0..n).min_by(|&a, &b| p[a].y.total_cmp(&p[b].y).then(p[a].x.total_cmp(&p[b].x))).unwrap_or(0);
    orient2d(p[(i + n - 1) % n], p[i], p[(i + 1) % n]) == Sign::Positive
}

/// No two non-adjacent edges touch. O(n²): enough for sketch-sized polygons.
fn is_simple(p: &[Vec2]) -> bool {
    let n = p.len();
    for i in 0..n {
        for j in i + 2..n {
            if i == 0 && j == n - 1 {
                continue;
            }
            if segments_touch(p[i], p[(i + 1) % n], p[j], p[(j + 1) % n]) {
                return false;
            }
        }
    }
    true
}

/// Closed segments `ab` and `cd` share at least one point (exact).
fn segments_touch(a: Vec2, b: Vec2, c: Vec2, d: Vec2) -> bool {
    let (d1, d2) = (orient2d(a, b, c), orient2d(a, b, d));
    let (d3, d4) = (orient2d(c, d, a), orient2d(c, d, b));
    if [d1, d2, d3, d4].iter().all(|s| *s == Sign::Zero) {
        let overlap = |a0: f64, a1: f64, b0: f64, b1: f64| a0.min(a1) <= b0.max(b1) && b0.min(b1) <= a0.max(a1);
        return overlap(a.x, b.x, c.x, d.x) && overlap(a.y, b.y, c.y, d.y);
    }
    let straddles = |s: Sign, t: Sign| s == Sign::Zero || t == Sign::Zero || s != t;
    straddles(d1, d2) && straddles(d3, d4)
}
