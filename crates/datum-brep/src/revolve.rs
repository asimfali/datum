//! Revolutions (D206): a closed profile of segments turned about an axis in its plane, fully or
//! by an angle. A segment along the axis turns into a cylinder, an oblique one into a cone, one
//! square to the axis into a plane (a disc or a ring).

use std::f64::consts::TAU;

use datum_math::{Sign, Vec2, Vec3, orient2d};

use crate::{Arc, Body, BrepError, Cone, Curve, CurveSpec, Cylinder, FaceSpec, Plane, Region, Surface, TopoId, Vertex, build_with_curves};

/// Where a profile is turned: its point `(x, y)` is `origin + axis·x + start·y` before turning,
/// `y` the distance from the axis. `axis` and `start` are unit and square to each other; the
/// turn is counterclockwise about `axis`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Axis {
    pub origin: Vec3,
    pub axis: Vec3,
    pub start: Vec3,
}

impl Axis {
    /// The unit vector away from the axis at angle `t`.
    pub fn radial(&self, t: f64) -> Vec3 {
        if t == 0.0 {
            return self.start;
        }
        let (s, c) = libm::sincos(t);
        self.start * c + self.axis.cross(self.start) * s
    }

    /// The profile's point `p` turned by angle `t`.
    pub fn point(&self, p: Vec2, t: f64) -> Vec3 {
        self.origin + self.axis * p.x + self.radial(t) * p.y
    }
}

/// The region's outer loop (in the axis's `(x, y)`, either orientation) turned about `at` by
/// `angle` in `(0, 2π]`: a full turn at `2π` and above, else a sector closed by two plane caps
/// with the region's cap IDs, at angle 0 and at `angle`.
///
/// The loop's point `k` names the vertices it turns into (`vertices[k][0]`), edge `k` the faces
/// (`sides[k]`); a full turn splits a circle into three arcs of 120° (D65), and the faces of a
/// curved edge with it: the first keeps the side's ID. Refused: holes and arcs in the region
/// (`Unsupported`: a torus or a sphere is not made yet), a point across the axis, collinear
/// neighbours, a loop touching itself (`Degenerate`), and an oblique edge reaching the axis
/// (`Unsupported`: a cone to a point).
pub fn revolve(region: &Region, at: &Axis, angle: f64) -> Result<Body, BrepError> {
    let degenerate = |m: &str| Err(BrepError::Degenerate(m.into()));
    let l = &region.outer;
    let n = l.points.len();
    if !region.holes.is_empty() {
        return Err(BrepError::Unsupported("a revolved profile with holes".into()));
    }
    if l.arcs.iter().any(Option::is_some) {
        return Err(BrepError::Unsupported("an arc in a revolved profile: a torus or a sphere is not made yet".into()));
    }
    if n < 3 || l.vertices.len() != n || l.sides.len() != n {
        return degenerate(&format!("a profile loop has {n} points"));
    }
    if !(angle.is_finite() && angle > 0.0) || !l.points.iter().all(|p| p.x.is_finite() && p.y.is_finite()) {
        return degenerate("non-finite coordinates or no angle");
    }
    if l.points.iter().any(|p| p.y < 0.0) {
        return degenerate("the profile crosses the axis");
    }
    if (0..n).any(|k| orient2d(l.points[k], l.points[(k + 1) % n], l.points[(k + 2) % n]) == Sign::Zero) {
        return degenerate("collinear or repeated neighbouring points in the profile");
    }
    if l.edges_touch(None) {
        return degenerate("the profile touches or crosses itself");
    }
    let l = if l.counterclockwise() { l.clone() } else { l.reversed() };
    let p = &l.points;
    if (0..n).any(|k| {
        let (a, b) = (p[k], p[(k + 1) % n]);
        (a.y == 0.0) != (b.y == 0.0) && a.x != b.x
    }) {
        return Err(BrepError::Unsupported("a cone to a point: an oblique edge reaches the axis".into()));
    }

    let full = angle >= TAU;
    let angles: &[f64] = if full { &[0.0, TAU / 3.0, 2.0 * TAU / 3.0] } else { &[0.0, angle] };
    let m = angles.len();
    // The vertex of point `k` at angle `i`: one on the axis, whatever the angle.
    let vid = |k: usize, i: usize| {
        let o = TopoId::derive("revolve.v").id(l.vertices[k][0]);
        if p[k].y == 0.0 { o.finish() } else { o.index(i as u64).finish() }
    };
    let on_axis = |k: usize| at.origin + at.axis * p[k].x;
    let mut vertices = Vec::new();
    let mut curves = Vec::new();
    for k in 0..n {
        if p[k].y == 0.0 {
            // A full turn leaves no vertex on the axis: its edges turn into nothing or a disc.
            if !full {
                vertices.push(Vertex { id: vid(k, 0), point: on_axis(k) });
            }
            continue;
        }
        for (i, &t) in angles.iter().enumerate() {
            vertices.push(Vertex { id: vid(k, i), point: at.point(p[k], t) });
        }
        let arc = Curve::Arc(Arc { center: on_axis(k), axis: at.axis, radius: p[k].y });
        for i in 0..if full { m } else { 1 } {
            curves.push(CurveSpec { from: vid(k, i), to: vid(k, (i + 1) % m), curve: arc });
        }
    }

    let mut faces = Vec::new();
    // The sector of edge `k` between angles `i` and `i + 1`: counterclockwise seen from outside
    // for a profile counterclockwise in `(x, y)`, a vertex on the axis once.
    let sector = |k: usize, i: usize| {
        let (j, w) = ((i + 1) % m, (k + 1) % n);
        let mut ids = vec![vid(k, i), vid(w, i), vid(w, j), vid(k, j)];
        ids.dedup();
        if ids.len() == 4 && ids[0] == ids[3] {
            ids.pop();
        }
        ids
    };
    for k in 0..n {
        let (a, b) = (p[k], p[(k + 1) % n]);
        if a.y == 0.0 && b.y == 0.0 {
            // Along the axis: an edge between the caps, or nothing in a full turn.
            continue;
        }
        let origin = on_axis(k);
        if a.x == b.x {
            // Square to the axis: facing along it where the profile's region is behind.
            let normal = if b.y > a.y { at.axis } else { -at.axis };
            let surface = Surface::Plane(Plane { origin, normal });
            let loops = if full {
                let circle = |k: usize| -> Vec<TopoId> {
                    let ids = (0..m).map(|i| vid(k, i));
                    if b.y > a.y { ids.collect() } else { ids.rev().collect() }
                };
                let (outer, inner) = if a.y > b.y { (k, (k + 1) % n) } else { ((k + 1) % n, k) };
                let mut loops = vec![circle(outer)];
                if p[inner].y > 0.0 {
                    loops.push(circle(inner).into_iter().rev().collect());
                }
                loops
            } else {
                vec![sector(k, 0)]
            };
            faces.push(FaceSpec { id: l.sides[k], surface, loops });
            continue;
        }
        // The region lies left of the edge: an edge running against the axis has it inside.
        let convex = b.x < a.x;
        let surface = if a.y == b.y {
            Surface::Cylinder(Cylinder { origin, axis: at.axis, radius: a.y, convex })
        } else {
            Surface::Cone(Cone { origin, axis: at.axis, radius: a.y, slope: (b.y - a.y) / (b.x - a.x), convex })
        };
        for i in 0..if full { m } else { 1 } {
            let id = if i == 0 { l.sides[k] } else { TopoId::derive("revolve.side").id(l.sides[k]).index(i as u64).finish() };
            faces.push(FaceSpec { id, surface, loops: vec![sector(k, i)] });
        }
    }
    if !full {
        let ring = |i: usize| (0..n).map(move |k| vid(k, i));
        let normal = |t: f64| at.axis.cross(at.radial(t));
        faces.push(FaceSpec {
            id: region.caps[0],
            surface: Surface::Plane(Plane { origin: at.origin, normal: -normal(0.0) }),
            loops: vec![ring(0).rev().collect()],
        });
        faces.push(FaceSpec {
            id: region.caps[1],
            surface: Surface::Plane(Plane { origin: at.origin, normal: normal(angle) }),
            loops: vec![ring(1).collect()],
        });
    }
    build_with_curves(vertices, faces, curves)
}
