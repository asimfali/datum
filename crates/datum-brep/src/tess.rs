//! Triangulation of a body's faces for export, display and Boolean operations.

use datum_math::{Sign, Vec2, Vec3, incircle, orient2d};

use rayon::prelude::*;
use smallvec::{SmallVec, smallvec};

use crate::{Body, BrepError, Curve, Ix, Surface, in_order};

/// Triangle mesh: the body's vertices first (`positions[i]` is vertex `i`), then points along
/// its arcs.
#[derive(Clone, Debug, PartialEq)]
pub struct Mesh {
    pub positions: Vec<Vec3>,
    /// Counterclockwise seen from outside.
    pub triangles: Vec<[Ix; 3]>,
}

/// Triangles of one face; small faces need no heap allocation.
pub type Triangles = SmallVec<[[Ix; 3]; 2]>;

/// A face as triangles for the screen and for export: its own points (body vertices and points
/// along its arcs), the surface normal at each, and triangles over them, counterclockwise seen
/// from outside. Arcs become chords here only; the model keeps them.
#[derive(Clone, Debug, PartialEq)]
pub struct FaceMesh {
    pub points: Vec<Vec3>,
    pub normals: Vec<Vec3>,
    /// The body vertex each point is, or `Ix::MAX` for a point along an arc.
    pub vertices: Vec<Ix>,
    pub triangles: Vec<[u32; 3]>,
}

/// Faces are triangulated in parallel and concatenated in face order.
pub fn tessellate(body: &Body) -> Result<Mesh, BrepError> {
    let chord = if body.has_curves() { body.chord() } else { 0.0 };
    // A plane face without arcs is its vertices; only curved ones bring points of their own.
    let faces = in_order(
        (0..body.faces().len() as Ix)
            .into_par_iter()
            .map(|f| if curved(body, f) { face_mesh(body, f, false, chord).map(|m| Err(Box::new(m))) } else { triangulate_face(body, f).map(Ok) }),
    )?;
    let mut positions: Vec<Vec3> = body.vertices().iter().map(|v| v.point).collect();
    let mut triangles = Vec::new();
    for face in faces {
        match face {
            Ok(tris) => triangles.extend(tris),
            Err(m) => {
                let index: Vec<Ix> = m
                    .vertices
                    .iter()
                    .zip(&m.points)
                    .map(|(&v, &p)| {
                        if v != Ix::MAX {
                            return v;
                        }
                        positions.push(p);
                        positions.len() as Ix - 1
                    })
                    .collect();
                triangles.extend(m.triangles.iter().map(|t| t.map(|k| index[k as usize])));
            }
        }
    }
    Ok(Mesh { positions, triangles })
}

/// Whether face `f` is curved or bounded by a curve.
fn curved(body: &Body, f: Ix) -> bool {
    body.faces()[f as usize].surface.plane().is_none()
        || (!body.curves.is_empty()
            && body.face_loops(f).any(|l| body.loop_half_edges(l).any(|h| body.edges()[body.half_edges()[h as usize].edge as usize].curve.is_some())))
}

/// Triangles of plane face `f` as vertex indices, counterclockwise seen from outside. No new
/// vertices. In the face's dominant projection the loops are cut into monotone pieces by a
/// sweep, and each piece is triangulated from its sorted vertices (de Berg et al., *Computational
/// Geometry*, ch. 3). Every decision is an exact predicate; O(n log n) in the number of vertices,
/// so faces with thousands of holes stay cheap (PAINS P4). A face with arcs has no such
/// triangulation: see [`face_mesh`].
pub fn triangulate_face(body: &Body, f: Ix) -> Result<Triangles, BrepError> {
    let face = &body.faces()[f as usize];
    if curved(body, f) {
        return Err(BrepError::Unsupported(format!("face {:?} is curved or has arcs", face.id)));
    }
    // A triangle is its own triangulation (terrain: millions of them).
    if face.loop_count == 1 && body.loops()[face.first_loop as usize].len == 3 {
        let h = body.loop_half_edges(face.first_loop);
        return Ok(smallvec![[h.start, h.start + 1, h.start + 2].map(|h| body.half_edges()[h as usize].origin)]);
    }
    let (ids, tris) = plane_triangles(body, f, false)?;
    Ok(tris.into_iter().map(|t| t.map(|k| ids[k as usize])).collect())
}

/// The sweep (and flips) on a plane face without arcs: the vertices of its loops, and triangles
/// over them by position.
fn plane_triangles(body: &Body, f: Ix, delaunay: bool) -> Result<(Vec<Ix>, Vec<[u32; 3]>), BrepError> {
    let face = &body.faces()[f as usize];
    let project = Projection::of(&face.surface);
    let mut ids = Vec::new();
    let mut next = Vec::new();
    for l in body.face_loops(f) {
        let first = ids.len() as Ix;
        ids.extend(body.loop_half_edges(l).map(|h| body.half_edges()[h as usize].origin));
        let last = ids.len() as Ix - 1;
        next.extend((first..last).map(|k| k + 1).chain([first]));
    }
    let pts: Vec<Vec2> = ids.iter().map(|&i| project.at(body.point(i), 0.0)).collect();
    let tris = Polygon::new(&pts, next)
        .and_then(|poly| poly.triangulate(delaunay))
        .map_err(|()| BrepError::Degenerate(format!("face {:?} cannot be triangulated", face.id)))?;
    Ok((ids, tris))
}

/// Face `f` as triangles, flipped to constrained Delaunay if `delaunay`, arcs as chords at most
/// `chord` off (see [`Body::chord`]). The sweep leaves slivers
/// across the whole face, which a GPU fills slowly (a perforated panel close up: 290 ms a
/// frame); Boolean operations do not mind them and skip the flips, which cost more than the sweep.
pub fn face_mesh(body: &Body, f: Ix, delaunay: bool, chord: f64) -> Result<FaceMesh, BrepError> {
    let face = &body.faces()[f as usize];
    if let Surface::Plane(plane) = face.surface
        && !curved(body, f)
    {
        let n = crate::curve::unit(plane.normal);
        let (vertices, triangles) = if face.loop_count == 1 && body.loops()[face.first_loop as usize].len == 3 {
            (body.loop_half_edges(face.first_loop).map(|h| body.half_edges()[h as usize].origin).collect(), vec![[0, 1, 2]])
        } else {
            plane_triangles(body, f, delaunay)?
        };
        return Ok(FaceMesh { points: vertices.iter().map(|&v| body.point(v)).collect(), normals: vec![n; vertices.len()], vertices, triangles });
    }
    if let Surface::Wall(w) = face.surface {
        return wall_mesh(body, f, &w, chord);
    }
    let (mut points, mut vertices, mut next, mut angles) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    let frame = match face.surface {
        Surface::Cylinder(c) => Some(crate::curve::frame(&c)),
        _ => None,
    };
    for l in body.face_loops(f) {
        let first = points.len() as Vx;
        let start = body.point(body.half_edges()[body.loop_half_edges(l).start as usize].origin);
        let mut theta = frame.as_ref().map_or(0.0, |fr| fr.angle(start));
        for h in body.loop_half_edges(l) {
            let he = &body.half_edges()[h as usize];
            let (u, v) = (body.point(he.origin), body.point(body.half_edges()[body.next(h) as usize].origin));
            points.push(u);
            vertices.push(he.origin);
            angles.push(theta);
            let edge = &body.edges()[he.edge as usize];
            let Some(curve) = body.edge_curve(edge) else { continue };
            let curve = if edge.half == h { curve } else { curve.reversed() };
            let inner = curve.inner_points(u, v, chord);
            let n = inner.len() + 1;
            // On a cylinder a curve runs around the axis: θ turns with it, by `at(t)` at `t`.
            let at = |t: f64| match (&frame, &curve) {
                (None, _) => 0.0,
                (Some(fr), Curve::Arc(arc)) => arc.sweep(u, v) * fr.axis.dot(arc.axis).signum() * t,
                (Some(fr), Curve::Rolled { piece, .. }) => {
                    fr.axis.dot(piece.dev.axis).signum() / piece.dev.neutral * (piece.at(t).0.x - piece.from.x)
                }
            };
            for (k, p) in inner.into_iter().enumerate() {
                points.push(p);
                vertices.push(Ix::MAX);
                angles.push(theta + at((k + 1) as f64 / n as f64));
            }
            theta += at(1.0);
        }
        let last = points.len() as Vx - 1;
        next.extend((first..last).map(|k| k + 1).chain([first]));
    }
    let project = Projection::of(&face.surface);
    let mut flat: Vec<Vec2> = points.iter().zip(&angles).map(|(&p, &t)| project.at(p, t)).collect();
    // Unrolled, a sweep fans whole arcs from one corner: flat in the layout, those triangles cut
    // deep across the cylinder. Delaunay keeps them short along the arc.
    let delaunay = delaunay || frame.is_some();
    let mut triangles = Polygon::new(&flat, next)
        .and_then(|poly| poly.triangulate(delaunay))
        .map_err(|()| BrepError::Degenerate(format!("face {:?} cannot be triangulated", face.id)))?;
    // Between holes and far edges even Delaunay triangles span wide angles: no wider than a chord.
    if let (Surface::Cylinder(c), Some(fr)) = (face.surface, &frame) {
        let side = if c.convex { 1.0 } else { -1.0 };
        let known = flat.len();
        refine(&mut flat, &mut triangles, c.radius * std::f64::consts::TAU / crate::curve::per_turn(c.radius, chord));
        for q in &flat[known..] {
            points.push(fr.point(q.x * side / c.radius, q.y, c.radius));
            vertices.push(Ix::MAX);
        }
    }
    let normals = points.iter().map(|&p| face.surface.normal_at(p)).collect();
    Ok(FaceMesh { points, normals, vertices, triangles })
}

/// The wall `w` of face `f` in its own coordinates: distance from the axis, and length along
/// the piece (the normal `∂r × ∂t` points out of the body). Its points are those of its two
/// edges along the piece, each at its radius; a vertex's place on the piece is where those edges
/// start or end.
fn wall_mesh(body: &Body, f: Ix, w: &crate::Rolled, chord: f64) -> Result<FaceMesh, BrepError> {
    let bad = || BrepError::Degenerate(format!("wall {:?} is not bounded by its piece", body.faces()[f as usize].id));
    let length = w.measure().2;
    // Each half-edge along the piece, the piece's way or back, at its radius.
    let along = |h: Ix| {
        let edge = &body.edges()[body.half_edges()[h as usize].edge as usize];
        match body.edge_curve(edge)? {
            Curve::Rolled { piece, radius } => {
                let piece = if edge.half == h { piece } else { piece.reversed() };
                Some((piece.from == w.from && piece.to == w.to, radius, piece))
            }
            Curve::Arc(_) => None,
        }
    };
    // Each point: where it is, the body vertex it is (`Ix::MAX` for none), and `(t, r)`.
    let mut pts: Vec<(Vec3, Ix, f64, f64)> = Vec::new();
    let mut next: Vec<Vx> = Vec::new();
    for l in body.face_loops(f) {
        let first = pts.len() as Vx;
        for h in body.loop_half_edges(l) {
            let origin = body.half_edges()[h as usize].origin;
            let (t, r) = match along(h) {
                Some((forward, r, _)) => (if forward { 0.0 } else { 1.0 }, r),
                None => {
                    let prev = if h == body.loops()[l as usize].first { h + body.loops()[l as usize].len - 1 } else { h - 1 };
                    let (forward, r, _) = along(prev).ok_or_else(bad)?;
                    (if forward { 1.0 } else { 0.0 }, r)
                }
            };
            pts.push((body.point(origin), origin, t, r));
            if let Some((forward, r, piece)) = along(h) {
                let n = piece.chords(r, chord);
                for k in 1..n {
                    let t = k as f64 / n as f64;
                    pts.push((piece.dev.point(piece.at(t).0, r), Ix::MAX, if forward { t } else { 1.0 - t }, r));
                }
            }
        }
        let last = pts.len() as Vx - 1;
        next.extend((first..last).map(|k| k + 1).chain([first]));
    }
    let flat: Vec<Vec2> = pts.iter().map(|&(_, _, t, r)| Vec2::new(r + 0.0, t * length + 0.0)).collect();
    let normals = pts.iter().map(|&(_, _, t, r)| w.normal(w.at(t).0, r)).collect();
    let (points, vertices) = pts.iter().map(|&(p, v, _, _)| (p, v)).unzip();
    let triangles = Polygon::new(&flat, next).and_then(|poly| poly.triangulate(true)).map_err(|()| bad())?;
    Ok(FaceMesh { points, normals, vertices, triangles })
}

/// A face's own 2D coordinates, in which its loops have the interior on their left. A plane drops
/// its dominant axis: that keeps the loops' turning sense when the other two axes stay in cyclic
/// order; a negative dominant component flips it, and swapping the two flips it back. A cylinder
/// unrolls to `(R·θ, z)`, mirrored when the body is outside it.
enum Projection {
    Plane(usize, usize),
    Cylinder(crate::Cylinder),
}

impl Projection {
    fn of(s: &Surface) -> Self {
        match *s {
            Surface::Plane(plane) => {
                let axis = plane.normal.dominant_axis();
                let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
                if plane.normal.to_array()[axis] < 0.0 { Self::Plane(v, u) } else { Self::Plane(u, v) }
            }
            Surface::Cylinder(c) => Self::Cylinder(c),
            Surface::Wall(_) => unreachable!("walls are laid out by `wall_mesh`"),
        }
    }

    /// Point `p` at angle `theta` about a cylinder's axis (ignored on a plane). `+ 0.0` turns −0
    /// into +0, so the sweep order agrees with the predicates.
    fn at(&self, p: Vec3, theta: f64) -> Vec2 {
        match self {
            Self::Plane(u, v) => {
                let p = p.to_array();
                Vec2::new(p[*u] + 0.0, p[*v] + 0.0)
            }
            Self::Cylinder(c) => {
                let side = if c.convex { 1.0 } else { -1.0 };
                Vec2::new(side * c.radius * theta + 0.0, (p - c.origin).dot(c.axis) + 0.0)
            }
        }
    }
}

/// Points of edge `e` from its lower-ID vertex to the other: its ends, and points along it, at
/// most `chord` off, if it is curved.
pub fn edge_points(body: &Body, e: Ix, chord: f64) -> Vec<Vec3> {
    let edge = &body.edges()[e as usize];
    let he = &body.half_edges()[edge.half as usize];
    let (u, v) = (body.point(he.origin), body.point(body.half_edges()[he.twin as usize].origin));
    let mut pts = vec![u];
    if let Some(curve) = &body.edge_curve(edge) {
        pts.extend(curve.inner_points(u, v, chord));
    }
    pts.push(v);
    pts
}

/// Index of a polygon vertex.
type Vx = u32;

/// No neighbour: a loop edge.
const NONE: u32 = u32::MAX;

/// An edge `a → b` of triangle `t`: `(t, a, b)`.
type TriEdge = (u32, Vx, Vx);

/// The neighbour across edge `i → i + 1` of each triangle, and the interior edges `(triangle, a,
/// b)` in the order of their sorted ends.
fn neighbours(tris: &[[Vx; 3]]) -> (Vec<[u32; 3]>, Vec<TriEdge>) {
    let mut edges: Vec<(Vx, Vx, u32)> = Vec::with_capacity(tris.len() * 3);
    for (t, tri) in tris.iter().enumerate() {
        for i in 0..3 {
            let (a, b) = (tri[i], tri[(i + 1) % 3]);
            edges.push((a.min(b), a.max(b), (t * 3 + i) as u32));
        }
    }
    edges.sort_unstable();
    let mut nb = vec![[NONE; 3]; tris.len()];
    let mut inner = Vec::new();
    for w in edges.windows(2) {
        if (w[0].0, w[0].1) == (w[1].0, w[1].1) {
            let (s, t) = (w[0].2, w[1].2);
            nb[s as usize / 3][s as usize % 3] = t / 3;
            nb[t as usize / 3][t as usize % 3] = s / 3;
            inner.push((s / 3, tris[s as usize / 3][s as usize % 3], tris[s as usize / 3][(s as usize + 1) % 3]));
        }
    }
    (nb, inner)
}

/// Where edge `a → b` is in a triangle.
fn slot(tri: &[Vx; 3], a: Vx, b: Vx) -> Option<usize> {
    (0..3).find(|&i| tri[i] == a && tri[(i + 1) % 3] == b)
}

/// Points `from` in `nb[n]` at `to` instead.
fn repoint(nb: &mut [[u32; 3]], n: u32, from: u32, to: u32) {
    if n != NONE {
        for x in &mut nb[n as usize] {
            if *x == from {
                *x = to;
            }
        }
    }
}

/// Flips interior edges whose opposite vertex lies strictly inside the other triangle's
/// circumcircle (Lawson) until none is left. Loop edges border one triangle and stay.
fn flip(p: &[Vec2], tris: &mut [[Vx; 3]]) {
    let (mut nb, stack) = neighbours(tris);
    flip_from(p, tris, &mut nb, stack);
}

/// [`flip`] from the edges `stack`, with the neighbours `nb` kept up to date.
fn flip_from(p: &[Vec2], tris: &mut [[Vx; 3]], nb: &mut [[u32; 3]], mut stack: Vec<TriEdge>) {
    let pt = |k: Vx| p[k as usize];
    while let Some((t1, a, b)) = stack.pop() {
        // The edge may have been flipped away since it was pushed.
        let Some(i) = slot(&tris[t1 as usize], a, b) else { continue };
        let t2 = nb[t1 as usize][i];
        if t2 == NONE {
            continue;
        }
        let Some(j) = slot(&tris[t2 as usize], b, a) else { continue };
        let c = tris[t1 as usize][(i + 2) % 3];
        let d = tris[t2 as usize][(j + 2) % 3];
        if incircle(pt(a), pt(b), pt(c), pt(d)) != Sign::Positive
            || orient2d(pt(c), pt(a), pt(d)) != Sign::Positive
            || orient2d(pt(d), pt(b), pt(c)) != Sign::Positive
        {
            continue;
        }
        let (n_bc, n_ca) = (nb[t1 as usize][(i + 1) % 3], nb[t1 as usize][(i + 2) % 3]);
        let (n_ad, n_db) = (nb[t2 as usize][(j + 1) % 3], nb[t2 as usize][(j + 2) % 3]);
        tris[t1 as usize] = [c, a, d];
        nb[t1 as usize] = [n_ca, n_ad, t2];
        tris[t2 as usize] = [d, b, c];
        nb[t2 as usize] = [n_db, n_bc, t1];
        repoint(nb, n_ad, t2, t1);
        repoint(nb, n_bc, t1, t2);
        stack.extend([(t1, c, a), (t1, a, d), (t2, d, b), (t2, b, c)]);
    }
}

/// Inserts points into triangles wider along X than `step`, keeping Delaunay: on an unrolled
/// cylinder X runs around the axis, and a wide triangle cuts deep under the face (from a hole to
/// the far corners of a sheet). The points are those of a grid of half that spacing that fall
/// strictly inside such a triangle; each is found by walking from it and inserted, then the
/// triangulation flipped back to Delaunay around it. Loop edges are no wider already, being chords.
fn refine(p: &mut Vec<Vec2>, tris: &mut Vec<[Vx; 3]>, step: f64) {
    let span = |p: &[Vec2], t: &[Vx; 3]| {
        let x = t.map(|k| p[k as usize].x);
        x[0].max(x[1]).max(x[2]) - x[0].min(x[1]).min(x[2])
    };
    let h = 0.5 * step;
    // Until no wide triangle holds a point of the grid: each round inserts at least one, and a
    // point of the grid is inserted once at most.
    loop {
        // Grid points strictly inside a wide triangle, with that triangle.
        let mut grid: Vec<(Vec2, u32)> = Vec::new();
        for (t, tri) in tris.iter().enumerate().filter(|(_, tri)| span(p, tri) > step) {
            let q = tri.map(|k| p[k as usize]);
            let (lo, hi) = (
                Vec2::new(q[0].x.min(q[1].x).min(q[2].x), q[0].y.min(q[1].y).min(q[2].y)),
                Vec2::new(q[0].x.max(q[1].x).max(q[2].x), q[0].y.max(q[1].y).max(q[2].y)),
            );
            for i in (lo.x / h).ceil() as i64..=(hi.x / h).floor() as i64 {
                for j in (lo.y / h).ceil() as i64..=(hi.y / h).floor() as i64 {
                    let g = Vec2::new(i as f64 * h, j as f64 * h);
                    if (0..3).all(|k| orient2d(q[k], q[(k + 1) % 3], g) == Sign::Positive) {
                        grid.push((g, t as u32));
                    }
                }
            }
        }
        if grid.is_empty() || !insert(p, tris, &grid) {
            return;
        }
    }
}

/// Inserts each point `g` of `points` into the triangle it lies strictly in, found by walking
/// from triangle `start`, and flips back to Delaunay around it. A point the walk does not find
/// strictly inside the face is dropped (it only refines). Whether any was inserted.
fn insert(p: &mut Vec<Vec2>, tris: &mut Vec<[Vx; 3]>, points: &[(Vec2, u32)]) -> bool {
    let (mut nb, _) = neighbours(tris);
    let mut any = false;
    for &(g, start) in points {
        let (mut t, mut steps) = (start, 0);
        let found = loop {
            let tri = tris[t as usize];
            let out = (0..3).map(|k| (k, orient2d(p[tri[k] as usize], p[tri[(k + 1) % 3] as usize], g))).find(|&(_, o)| o != Sign::Positive);
            match out {
                None => break true,
                Some((_, Sign::Zero)) => break false,
                Some((k, _)) if nb[t as usize][k] == NONE || steps > tris.len() => break false,
                Some((k, _)) => t = nb[t as usize][k],
            }
            steps += 1;
        };
        if !found {
            continue;
        }
        any = true;
        let [a, b, c] = tris[t as usize];
        let [n_ab, n_bc, n_ca] = nb[t as usize];
        let m = p.len() as Vx;
        p.push(g);
        let (t2, t3) = (tris.len() as u32, tris.len() as u32 + 1);
        tris[t as usize] = [a, b, m];
        nb[t as usize] = [n_ab, t2, t3];
        tris.push([b, c, m]);
        nb.push([n_bc, t3, t]);
        tris.push([c, a, m]);
        nb.push([n_ca, t, t2]);
        repoint(&mut nb, n_bc, t, t2);
        repoint(&mut nb, n_ca, t, t3);
        flip_from(p, tris, &mut nb, vec![(t, a, b), (t2, b, c), (t3, c, a)]);
    }
    any
}

/// Vertex kinds of the monotone decomposition, by the neighbours' sweep positions and the turn.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Kind {
    Start,
    End,
    Split,
    Merge,
    /// On a left boundary: the interior lies east.
    Left,
    Right,
}

/// An edge `edge → next[edge]` crossing the sweep line, with interior to its east.
struct Active {
    edge: Vx,
    helper: Vx,
}

/// Loops with the interior on their left (outer counterclockwise, holes clockwise). The sweep
/// runs from top to bottom: by `y` descending, then `x` ascending.
struct Polygon<'a> {
    p: &'a [Vec2],
    next: Vec<Vx>,
    prev: Vec<Vx>,
    /// Sweep position of every vertex.
    rank: Vec<u32>,
    order: Vec<Vx>,
}

impl<'a> Polygon<'a> {
    fn new(p: &'a [Vec2], next: Vec<Vx>) -> Result<Self, ()> {
        let mut prev = vec![0; next.len()];
        for (k, &n) in next.iter().enumerate() {
            prev[n as usize] = k as Vx;
        }
        let sweep = |a: &Vx, b: &Vx| {
            let (a, b) = (p[*a as usize], p[*b as usize]);
            b.y.total_cmp(&a.y).then(a.x.total_cmp(&b.x))
        };
        let mut order: Vec<Vx> = (0..p.len() as Vx).collect();
        order.sort_unstable_by(sweep);
        if order.windows(2).any(|w| sweep(&w[0], &w[1]).is_eq()) {
            return Err(());
        }
        let mut rank = vec![0; p.len()];
        for (r, &k) in order.iter().enumerate() {
            rank[k as usize] = r as u32;
        }
        Ok(Self { p, next, prev, rank, order })
    }

    fn pt(&self, k: Vx) -> Vec2 {
        self.p[k as usize]
    }

    fn orient(&self, a: Vx, b: Vx, c: Vx) -> Sign {
        orient2d(self.pt(a), self.pt(b), self.pt(c))
    }

    fn above(&self, a: Vx, b: Vx) -> bool {
        self.rank[a as usize] < self.rank[b as usize]
    }

    fn kind(&self, v: Vx) -> Result<Kind, ()> {
        let (prev, next) = (self.prev[v as usize], self.next[v as usize]);
        let turn = self.orient(prev, v, next);
        Ok(match (self.above(prev, v), self.above(next, v), turn) {
            (_, _, Sign::Zero) if self.above(prev, v) == self.above(next, v) => return Err(()),
            (false, false, Sign::Positive) => Kind::Start,
            (false, false, _) => Kind::Split,
            (true, true, Sign::Positive) => Kind::End,
            (true, true, _) => Kind::Merge,
            (true, false, _) => Kind::Left,
            (false, true, _) => Kind::Right,
        })
    }

    fn triangulate(&self, delaunay: bool) -> Result<Vec<[Vx; 3]>, ()> {
        let diagonals = self.monotone_diagonals()?;
        let n = self.p.len();
        let loops = n - self.next.iter().enumerate().filter(|&(k, &x)| x as usize > k).count();
        // Euler: a face of `n` vertices and `h` holes has `n + 2h − 2` triangles.
        let expected = (n + 2 * loops).checked_sub(4).ok_or(())?;
        let mut tris = Vec::with_capacity(expected);
        for piece in self.pieces(&diagonals)? {
            self.triangulate_monotone(&piece, &mut tris)?;
        }
        if tris.len() != expected {
            return Err(());
        }
        if delaunay {
            flip(self.p, &mut tris);
        }
        Ok(tris)
    }

    /// Position in `status` of the first edge not strictly west of `v`.
    fn position(&self, status: &[Active], v: Vx) -> usize {
        status.partition_point(|a| self.orient(a.edge, self.next[a.edge as usize], v) == Sign::Positive)
    }

    /// Diagonals that cut the polygon into pieces monotone in the sweep direction.
    fn monotone_diagonals(&self) -> Result<Vec<(Vx, Vx)>, ()> {
        let kinds = (0..self.p.len() as Vx).map(|v| self.kind(v)).collect::<Result<Vec<_>, _>>()?;
        let merge = |v: Vx| kinds[v as usize] == Kind::Merge;
        let mut status: Vec<Active> = Vec::new();
        let mut diagonals = Vec::new();
        for &v in &self.order {
            let prev = self.prev[v as usize];
            // The edge ending at `v` leaves the sweep line.
            let finish = |status: &mut Vec<Active>, diagonals: &mut Vec<(Vx, Vx)>| {
                let i = self.position(status, v);
                if status.get(i).map(|a| a.edge) != Some(prev) {
                    return Err(());
                }
                let a = status.remove(i);
                if merge(a.helper) {
                    diagonals.push((v, a.helper));
                }
                Ok(())
            };
            // The edge starting at `v` joins it; `v` must not lie on another edge.
            let start = |status: &mut Vec<Active>| {
                let i = self.position(status, v);
                if status.get(i).is_some_and(|a| self.orient(a.edge, self.next[a.edge as usize], v) == Sign::Zero) {
                    return Err(());
                }
                status.insert(i, Active { edge: v, helper: v });
                Ok(())
            };
            // The edge directly west of `v`.
            let west = |status: &[Active]| match self.position(status, v) {
                0 => Err(()),
                i if status.get(i).is_some_and(|a| self.orient(a.edge, self.next[a.edge as usize], v) == Sign::Zero) => Err(()),
                i => Ok(i - 1),
            };
            match kinds[v as usize] {
                Kind::Start => start(&mut status)?,
                Kind::End => finish(&mut status, &mut diagonals)?,
                Kind::Split => {
                    let j = west(&status)?;
                    diagonals.push((v, status[j].helper));
                    status[j].helper = v;
                    start(&mut status)?;
                }
                Kind::Merge => {
                    finish(&mut status, &mut diagonals)?;
                    let j = west(&status)?;
                    if merge(status[j].helper) {
                        diagonals.push((v, status[j].helper));
                    }
                    status[j].helper = v;
                }
                Kind::Left => {
                    finish(&mut status, &mut diagonals)?;
                    start(&mut status)?;
                }
                Kind::Right => {
                    let j = west(&status)?;
                    if merge(status[j].helper) {
                        diagonals.push((v, status[j].helper));
                    }
                    status[j].helper = v;
                }
            }
        }
        if status.is_empty() { Ok(diagonals) } else { Err(()) }
    }

    /// The pieces bounded by the loops and the diagonals, each counterclockwise. Half-edges are
    /// numbered: `k` for the loop edge `k → next[k]`, then both directions of every diagonal.
    fn pieces(&self, diagonals: &[(Vx, Vx)]) -> Result<Vec<Vec<Vx>>, ()> {
        let n = self.p.len();
        // Diagonal half-edges `(from, to, number)`, sorted by `from`.
        let mut extra: Vec<(Vx, Vx, usize)> =
            diagonals.iter().enumerate().flat_map(|(d, &(a, b))| [(a, b, n + 2 * d), (b, a, n + 2 * d + 1)]).collect();
        extra.sort_unstable();
        let ends = |h: usize| match h.checked_sub(n) {
            None => (h as Vx, self.next[h]),
            Some(d) if d % 2 == 0 => diagonals[d / 2],
            Some(d) => (diagonals[d / 2].1, diagonals[d / 2].0),
        };
        // The half-edge after arriving at `w` from `u`: the first neighbour clockwise from `u`.
        let after = |u: Vx, w: Vx| -> Result<(Vx, usize), ()> {
            let from = extra.partition_point(|e| e.0 < w);
            let own = &extra[from..from + extra[from..].iter().take_while(|e| e.0 == w).count()];
            if own.is_empty() {
                return Ok((self.next[w as usize], w as usize));
            }
            let mut around: Vec<(Vx, Option<usize>)> = own.iter().map(|e| (e.1, Some(e.2))).collect();
            around.push((self.next[w as usize], Some(w as usize)));
            around.push((self.prev[w as usize], None));
            let c = self.pt(w);
            let upper = |a: Vx| {
                let p = self.pt(a);
                p.y > c.y || (p.y == c.y && p.x > c.x)
            };
            // Counterclockwise from east.
            around.sort_by(|a, b| {
                upper(b.0).cmp(&upper(a.0)).then_with(|| match self.orient(w, a.0, b.0) {
                    Sign::Positive => std::cmp::Ordering::Less,
                    Sign::Negative => std::cmp::Ordering::Greater,
                    Sign::Zero => std::cmp::Ordering::Equal,
                })
            });
            let i = around.iter().position(|a| a.0 == u).ok_or(())?;
            let (x, h) = around[(i + around.len() - 1) % around.len()];
            Ok((x, h.ok_or(())?))
        };
        let total = n + extra.len();
        let mut used = vec![false; total];
        let mut pieces = Vec::new();
        for first in 0..total {
            if used[first] {
                continue;
            }
            let (mut u, mut w) = ends(first);
            used[first] = true;
            let mut piece = vec![u];
            loop {
                let (x, h) = after(u, w)?;
                if h == first {
                    break;
                }
                if used[h] {
                    return Err(());
                }
                used[h] = true;
                piece.push(w);
                (u, w) = (w, x);
            }
            pieces.push(piece);
        }
        Ok(pieces)
    }

    /// Triangulates a counterclockwise piece monotone in the sweep direction by walking its
    /// vertices in sweep order with a stack of the not yet triangulated reflex chain.
    fn triangulate_monotone(&self, piece: &[Vx], tris: &mut Vec<[Vx; 3]>) -> Result<(), ()> {
        let m = piece.len();
        if m < 3 {
            return Err(());
        }
        let r = |i: usize| self.rank[piece[i % m] as usize];
        let top = (0..m).min_by_key(|&i| r(i)).unwrap_or(0);
        let bottom = (0..m).max_by_key(|&i| r(i)).unwrap_or(0);
        // Counterclockwise from the top runs down the left chain, then up the right one.
        let left: Vec<Vx> = (1..m).map(|k| (top + k) % m).take_while(|&i| i != bottom).map(|i| piece[i]).collect();
        let right: Vec<Vx> = (1..m).map(|k| (top + m - k) % m).take_while(|&i| i != bottom).map(|i| piece[i]).collect();
        let descending = |c: &[Vx]| c.windows(2).all(|w| self.above(w[0], w[1]));
        if !descending(&left) || !descending(&right) {
            return Err(());
        }
        // Vertices in sweep order with their chain (true: left).
        let mut sorted = Vec::with_capacity(m);
        sorted.push((piece[top], true));
        let (mut i, mut j) = (0, 0);
        while i < left.len() || j < right.len() {
            if j == right.len() || (i < left.len() && self.above(left[i], right[j])) {
                sorted.push((left[i], true));
                i += 1;
            } else {
                sorted.push((right[j], false));
                j += 1;
            }
        }
        let mut emit = |a: Vx, b: Vx, c: Vx| {
            tris.push(match self.orient(a, b, c) {
                Sign::Positive => [a, b, c],
                Sign::Negative => [a, c, b],
                Sign::Zero => return Err(()),
            });
            Ok(())
        };
        let mut stack = vec![sorted[0], sorted[1]];
        for k in 2..m - 1 {
            let (v, on_left) = sorted[k];
            let top_side = stack.last().map_or(on_left, |s| s.1);
            if on_left != top_side {
                for w in stack.windows(2) {
                    emit(v, w[0].0, w[1].0)?;
                }
                stack = vec![sorted[k - 1], sorted[k]];
            } else {
                let mut last = stack.pop().ok_or(())?;
                // The diagonal from `v` to the stack top stays inside while the popped vertex is convex.
                let convex = if on_left { Sign::Positive } else { Sign::Negative };
                while let Some(&t) = stack.last() {
                    if self.orient(t.0, last.0, v) != convex {
                        break;
                    }
                    emit(v, last.0, t.0)?;
                    last = stack.pop().ok_or(())?;
                }
                stack.push(last);
                stack.push(sorted[k]);
            }
        }
        let bottom = piece[bottom];
        for w in stack.windows(2) {
            emit(bottom, w[0].0, w[1].0)?;
        }
        Ok(())
    }
}

/// Closed segments `ab` and `cd` share at least one point (exact).
pub(crate) fn segments_touch(a: Vec2, b: Vec2, c: Vec2, d: Vec2) -> bool {
    let (d1, d2) = (orient2d(a, b, c), orient2d(a, b, d));
    let (d3, d4) = (orient2d(c, d, a), orient2d(c, d, b));
    if [d1, d2, d3, d4].iter().all(|s| *s == Sign::Zero) {
        let overlap = |a0: f64, a1: f64, b0: f64, b1: f64| a0.min(a1) <= b0.max(b1) && b0.min(b1) <= a0.max(a1);
        return overlap(a.x, b.x, c.x, d.x) && overlap(a.y, b.y, c.y, d.y);
    }
    let straddles = |s: Sign, t: Sign| s == Sign::Zero || t == Sign::Zero || s != t;
    straddles(d1, d2) && straddles(d3, d4)
}
