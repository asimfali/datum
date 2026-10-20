//! Extrusions: plane regions, possibly with holes and arcs, swept along Z; the prism as the
//! simplest one.

use datum_math::{Arc2, Sign, Vec2, Vec3, arcs_touch, orient2d, segment_touches_arc};

use crate::tess::segments_touch;
use crate::{Arc, ArcSpec, Body, BrepError, Cylinder, FaceSpec, Plane, Surface, TopoId, Vertex, build_with_arcs};

/// A profile edge that is an arc around `center`: counterclockwise (seen from +Z) when `ccw`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ProfileArc {
    pub center: Vec2,
    pub ccw: bool,
}

/// A closed loop of a region and the IDs its extrusion gives: for each point the vertices at the
/// start and at the end, for each edge from point `k` to `k + 1` the side face, and the edge's
/// arc if it is one (`arcs` empty: all straight).
#[derive(Clone, Debug)]
pub struct ProfileLoop {
    pub points: Vec<Vec2>,
    pub vertices: Vec<[TopoId; 2]>,
    pub sides: Vec<TopoId>,
    pub arcs: Vec<Option<ProfileArc>>,
}

impl ProfileLoop {
    /// The same loop the other way round, each edge keeping its side ID and arc.
    fn reversed(&self) -> Self {
        let n = self.points.len();
        let mut r = self.clone();
        r.points.reverse();
        r.vertices.reverse();
        // New edge k runs from old point n−1−k to n−2−k: old edge n−2−k.
        let old = |k: usize| (2 * n - 2 - k) % n;
        r.sides = (0..n).map(|k| self.sides[old(k)]).collect();
        r.arcs = (0..self.arcs.len()).map(|k| self.arcs[old(k)].map(|a| ProfileArc { ccw: !a.ccw, ..a })).collect();
        r
    }

    fn arc(&self, k: usize) -> Option<ProfileArc> {
        self.arcs.get(k).copied().flatten()
    }

    /// Whether the loop runs counterclockwise. A polygon: the turn at its lowest point, exact. With
    /// arcs: the sign of the area, chords and circular segments, in `f64`. The loop was found
    /// simple by exact predicates before, and a simple loop's area is far from zero: rounding
    /// cannot flip its sign.
    fn counterclockwise(&self) -> bool {
        let p = &self.points;
        let n = p.len();
        if self.arcs.iter().all(Option::is_none) {
            let i = (0..n).min_by(|&a, &b| p[a].y.total_cmp(&p[b].y).then(p[a].x.total_cmp(&p[b].x))).unwrap_or(0);
            return orient2d(p[(i + n - 1) % n], p[i], p[(i + 1) % n]) == Sign::Positive;
        }
        let area: f64 = (0..n)
            .map(|k| {
                let (a, b) = (p[k], p[(k + 1) % n]);
                let chord = 0.5 * (a.x * b.y - a.y * b.x);
                chord
                    + self.arc(k).map_or(0.0, |arc| {
                        let (u, v) = (Vec2::new(a.x - arc.center.x, a.y - arc.center.y), Vec2::new(b.x - arc.center.x, b.y - arc.center.y));
                        let turn = libm::atan2(u.x * v.y - u.y * v.x, u.x * v.x + u.y * v.y);
                        let phi = if arc.ccw {
                            if turn <= 0.0 { turn + std::f64::consts::TAU } else { turn }
                        } else if turn >= 0.0 {
                            std::f64::consts::TAU - turn
                        } else {
                            -turn
                        };
                        let r2 = u.x * u.x + u.y * u.y;
                        0.5 * r2 * (phi - libm::sin(phi)) * if arc.ccw { 1.0 } else { -1.0 }
                    })
            })
            .sum();
        area > 0.0
    }
}

/// A region: an outer loop, the holes in it, and the IDs of its start and end caps.
#[derive(Clone, Debug)]
pub struct Region {
    pub outer: ProfileLoop,
    pub holes: Vec<ProfileLoop>,
    pub caps: [TopoId; 2],
}

/// An edge of a profile for the touch checks: a segment, or an arc counterclockwise.
enum Piece {
    Segment(Vec2, Vec2),
    Arc(Arc2),
}

fn touch(a: &Piece, b: &Piece) -> bool {
    match (a, b) {
        (Piece::Segment(p, q), Piece::Segment(r, s)) => segments_touch(*p, *q, *r, *s),
        (Piece::Segment(p, q), Piece::Arc(c)) | (Piece::Arc(c), Piece::Segment(p, q)) => segment_touches_arc(*p, *q, c),
        (Piece::Arc(c), Piece::Arc(d)) => arcs_touch(c, d),
    }
}

/// The regions (in the XY plane, loops in either orientation) swept from `z0` to `z1`, with the
/// IDs they carry. Loops must be simple, of at least three points, and touch neither each other
/// nor themselves; holes must lie in their outer loop. A straight edge sweeps to a plane face, an
/// arc to a cylindrical one.
pub fn extrude(regions: &[Region], z0: f64, z1: f64) -> Result<Body, BrepError> {
    let degenerate = |m: String| Err(BrepError::Degenerate(m));
    if !z0.is_finite() || !z1.is_finite() || z0 >= z1 {
        return degenerate("non-finite coordinates or zero/negative height".into());
    }
    let loops: Vec<&ProfileLoop> = regions.iter().flat_map(|r| std::iter::once(&r.outer).chain(&r.holes)).collect();
    for l in &loops {
        let n = l.points.len();
        if n < 3 || l.vertices.len() != n || l.sides.len() != n || !(l.arcs.is_empty() || l.arcs.len() == n) {
            return degenerate(format!("a profile loop has {n} points"));
        }
        let finite = |p: &Vec2| p.x.is_finite() && p.y.is_finite();
        if !l.points.iter().all(finite) || !l.arcs.iter().flatten().all(|a| finite(&a.center)) {
            return degenerate("non-finite coordinates".into());
        }
        if (0..n).any(|k| l.arc(k).is_some_and(|a| a.center == l.points[k])) {
            return degenerate("an arc of zero radius".into());
        }
    }
    let edges: Vec<(Piece, usize, usize)> = loops
        .iter()
        .enumerate()
        .flat_map(|(li, l)| {
            (0..l.points.len()).map(move |k| {
                let (a, b) = (l.points[k], l.points[(k + 1) % l.points.len()]);
                let piece = match l.arc(k) {
                    None => Piece::Segment(a, b),
                    Some(arc) if arc.ccw => Piece::Arc(Arc2 { center: arc.center, from: a, to: b }),
                    Some(arc) => Piece::Arc(Arc2 { center: arc.center, from: b, to: a }),
                };
                (piece, li, k)
            })
        })
        .collect();
    // Edges touch only their neighbours in the same loop, at the shared point.
    for (i, a) in edges.iter().enumerate() {
        for b in &edges[i + 1..] {
            let n = loops[a.1].points.len();
            let adjacent = a.1 == b.1 && ((a.2 + 1) % n == b.2 || (b.2 + 1) % n == a.2);
            if !adjacent && touch(&a.0, &b.0) {
                return degenerate("profile loops touch or cross".into());
            }
        }
    }

    let mut vertices = Vec::new();
    let mut faces = Vec::new();
    let mut arcs = Vec::new();
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
            surface: Surface::Plane(Plane { origin: base(start, z0), normal: Vec3::new(0.0, 0.0, -1.0) }),
            loops: std::iter::once(&outer).chain(&holes).map(|l| layer(l, 0, true)).collect(),
        });
        faces.push(FaceSpec {
            id: r.caps[1],
            surface: Surface::Plane(Plane { origin: base(start, z1), normal: Vec3::new(0.0, 0.0, 1.0) }),
            loops: std::iter::once(&outer).chain(&holes).map(|l| layer(l, 1, false)).collect(),
        });
        for l in std::iter::once(&outer).chain(&holes) {
            let n = l.points.len();
            for k in 0..n {
                vertices.push(Vertex { id: l.vertices[k][0], point: base(l.points[k], z0) });
                vertices.push(Vertex { id: l.vertices[k][1], point: base(l.points[k], z1) });
                let (a, b) = (l.points[k], l.points[(k + 1) % n]);
                let (v, w) = (l.vertices[k], l.vertices[(k + 1) % n]);
                let surface = match l.arc(k) {
                    None => Surface::Plane(Plane { origin: base(a, z0), normal: Vec3::new(b.y - a.y, a.x - b.x, 0.0) }),
                    Some(arc) => {
                        let radius = (a.x - arc.center.x).hypot(a.y - arc.center.y);
                        let axis = Vec3::new(0.0, 0.0, if arc.ccw { 1.0 } else { -1.0 });
                        for (layer, z) in [(0, z0), (1, z1)] {
                            arcs.push(ArcSpec { from: v[layer], to: w[layer], arc: Arc { center: base(arc.center, z), axis, radius } });
                        }
                        // The loop has the region on its left: an arc turning left bulges out.
                        Surface::Cylinder(Cylinder { origin: base(arc.center, z0), axis: Vec3::new(0.0, 0.0, 1.0), radius, convex: arc.ccw })
                    }
                };
                faces.push(FaceSpec { id: l.sides[k], surface, loops: vec![vec![v[0], w[0], w[1], v[1]]] });
            }
        }
    }
    build_with_arcs(vertices, faces, arcs)
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
    let mut ring = ProfileLoop { points: points.clone(), vertices: Vec::new(), sides: Vec::new(), arcs: Vec::new() };
    if !ring.counterclockwise() {
        points.reverse();
    }
    let vid = |layer: u64, k: usize| TopoId::derive("prism.v").id(feature).index(layer).index(k as u64).finish();
    ring = ProfileLoop {
        vertices: (0..n).map(|k| [vid(0, k), vid(1, k)]).collect(),
        sides: (0..n).map(|k| TopoId::derive("prism.side").id(feature).index(k as u64).finish()).collect(),
        arcs: Vec::new(),
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
