//! Revolutions (D206): a closed profile of segments turned about an axis in its plane, fully or
//! by an angle. A segment along the axis turns into a cylinder, an oblique one into a cone, one
//! square to the axis into a plane (a disc or a ring), an arc into a torus, or a sphere when its
//! centre is on the axis.

use std::f64::consts::TAU;

use datum_math::{Arc2, Sign, Vec2, Vec3, orient2d, segment_touches_arc};

use crate::{
    Arc, Body, BrepError, Cone, Curve, CurveSpec, Cylinder, FaceSpec, Plane, ProfileLoop, Region, Surface, TopoId, Torus, Vertex, build_with_curves,
};

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
/// curved edge with it: the first keeps the side's ID. An arc turning more than 135° is cut into
/// equal pieces, each its own edge, so that no face runs from pole to pole (a circle's arcs of
/// 120° stay whole). Refused: holes in the
/// region, a point or arc across the axis, collinear neighbours, a loop touching itself
/// (`Degenerate`), and what would reach the axis and fold there (`Unsupported`): an oblique edge
/// (a cone to a point), an arc not centred on it or centred beyond it.
pub fn revolve(region: &Region, at: &Axis, angle: f64) -> Result<Body, BrepError> {
    let degenerate = |m: &str| Err(BrepError::Degenerate(m.into()));
    if !region.holes.is_empty() {
        return Err(BrepError::Unsupported("a revolved profile with holes".into()));
    }
    let l = &short_arcs(&region.outer);
    let n = l.points.len();
    if n < 3 || l.vertices.len() != n || l.sides.len() != n {
        return degenerate(&format!("a profile loop has {n} points"));
    }
    if !(angle.is_finite() && angle > 0.0) || !l.points.iter().all(|p| p.x.is_finite() && p.y.is_finite()) {
        return degenerate("non-finite coordinates or no angle");
    }
    if l.points.iter().any(|p| p.y < 0.0) {
        return degenerate("the profile crosses the axis");
    }
    let straight = |k: usize| l.arc(k % n).is_none();
    if (0..n).any(|k| straight(k) && straight(k + 1) && orient2d(l.points[k], l.points[(k + 1) % n], l.points[(k + 2) % n]) == Sign::Zero) {
        return degenerate("collinear or repeated neighbouring points in the profile");
    }
    if l.edges_touch(None) {
        return degenerate("the profile touches or crosses itself");
    }
    let l = if l.counterclockwise() { l.clone() } else { l.reversed() };
    let p = &l.points;
    if (0..n).any(|k| {
        let (a, b) = (p[k], p[(k + 1) % n]);
        l.arc(k).is_none() && (a.y == 0.0) != (b.y == 0.0) && a.x != b.x
    }) {
        return Err(BrepError::Unsupported("a cone to a point: an oblique edge reaches the axis".into()));
    }
    for (k, &a) in p.iter().enumerate() {
        if let Some(arc) = l.arc(k) {
            check_arc(a, p[(k + 1) % n], arc)?;
        }
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
    for (k, &q) in p.iter().enumerate() {
        if q.y == 0.0 {
            // A full turn leaves no vertex on the axis: its edges turn into nothing or a disc. A
            // pole of a sphere stays, where its meridians meet.
            if !full || l.arc(k).is_some() || l.arc((k + n - 1) % n).is_some() {
                vertices.push(Vertex { id: vid(k, 0), point: on_axis(k) });
            }
            continue;
        }
        for (i, &t) in angles.iter().enumerate() {
            vertices.push(Vertex { id: vid(k, i), point: at.point(q, t) });
        }
        let arc = Curve::Arc(Arc { center: on_axis(k), axis: at.axis, radius: q.y });
        for i in 0..if full { m } else { 1 } {
            curves.push(CurveSpec { from: vid(k, i), to: vid(k, (i + 1) % m), curve: arc });
        }
    }

    // Each arc of the profile at every angle it stands at: a meridian of its torus.
    for (k, &q) in p.iter().enumerate() {
        let Some(arc) = l.arc(k) else { continue };
        let radius = libm::hypot(q.x - arc.center.x, q.y - arc.center.y);
        for (i, &t) in angles.iter().enumerate() {
            let axis = at.axis.cross(at.radial(t)) * if arc.ccw { 1.0 } else { -1.0 };
            curves.push(CurveSpec {
                from: vid(k, i),
                to: vid((k + 1) % n, i),
                curve: Curve::Arc(Arc { center: at.point(arc.center, t), axis, radius }),
            });
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
        if let Some(arc) = l.arc(k) {
            // The region lies left of the edge: an arc counterclockwise has its centre inside.
            let (major, minor) = (arc.center.y, libm::hypot(a.x - arc.center.x, a.y - arc.center.y));
            let surface = Surface::Torus(Torus { center: at.origin + at.axis * arc.center.x, axis: at.axis, major, minor, convex: arc.ccw });
            for i in 0..if full { m } else { 1 } {
                let id = if i == 0 { l.sides[k] } else { TopoId::derive("revolve.side").id(l.sides[k]).index(i as u64).finish() };
                faces.push(FaceSpec { id, surface, loops: vec![sector(k, i)] });
            }
            continue;
        }
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
            // The axis's own point, the same for every cylinder: their origins differ by a
            // multiple of the axis only up to rounding when it is oblique, and the two sides of
            // a rolled sheet would not be known as coaxial (D227).
            Surface::Cylinder(Cylinder { origin: at.origin, axis: at.axis, radius: a.y, convex })
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

/// The loop with each arc turning more than 135° cut into equal pieces. The first piece keeps the
/// arc's side, the new points are named after it.
fn short_arcs(l: &ProfileLoop) -> ProfileLoop {
    let mut out = ProfileLoop { points: Vec::new(), vertices: Vec::new(), sides: Vec::new(), arcs: Vec::new() };
    let n = l.points.len();
    for k in 0..n {
        out.points.push(l.points[k]);
        out.vertices.push(l.vertices[k]);
        out.sides.push(l.sides[k]);
        out.arcs.push(l.arc(k));
        let Some(arc) = l.arc(k) else { continue };
        let (a, b) = (l.points[k], l.points[(k + 1) % n]);
        let turn = arc.turn(a, b);
        let pieces = libm::ceil(turn.abs() / (0.375 * TAU)) as u64;
        for j in 1..pieces {
            let (s, c) = libm::sincos(turn * j as f64 / pieces as f64);
            let (x, y) = (a.x - arc.center.x, a.y - arc.center.y);
            out.points.push(Vec2::new(arc.center.x + x * c - y * s, arc.center.y + x * s + y * c));
            let v = TopoId::derive("revolve.arc_point").id(l.sides[k]).index(j).finish();
            out.vertices.push([v, v]);
            out.sides.push(TopoId::derive("revolve.arc_side").id(l.sides[k]).index(j).finish());
            out.arcs.push(Some(arc));
        }
    }
    out
}

/// Whether arc piece `a → b` (of at most 135°) can be turned: on one side of the axis, and if an
/// end lies on the axis, centred on it, the end a sphere's pole.
fn check_arc(a: Vec2, b: Vec2, arc: crate::ProfileArc) -> Result<(), BrepError> {
    if arc.center.y < 0.0 {
        return Err(BrepError::Unsupported("an arc centred across the axis".into()));
    }
    let r = libm::hypot(a.x - arc.center.x, a.y - arc.center.y);
    if a.y == 0.0 || b.y == 0.0 {
        if arc.center.y != 0.0 {
            return Err(BrepError::Unsupported("an arc reaching the axis off its centre: a torus folded at the axis".into()));
        }
        let (s, c) = libm::sincos(0.5 * arc.turn(a, b));
        let (x, y) = (a.x - arc.center.x, a.y);
        return if x * s + y * c > 0.0 { Ok(()) } else { Err(BrepError::Degenerate("the profile crosses the axis".into())) };
    }
    let (from, to) = if arc.ccw { (a, b) } else { (b, a) };
    let axis = (Vec2::new(arc.center.x - 2.0 * r, 0.0), Vec2::new(arc.center.x + 2.0 * r, 0.0));
    if segment_touches_arc(axis.0, axis.1, &Arc2 { center: arc.center, from, to }) {
        return Err(BrepError::Degenerate("the profile crosses the axis".into()));
    }
    Ok(())
}
