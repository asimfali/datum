//! N-ary Boolean operation (ARCHITECTURE §7, D38–D40): a target combined with any number of
//! pairwise disjoint tools in one pass, along the §7.1 pipeline. Stages run in parallel over
//! triangles, pairs, edges and faces; their results are collected by index and merged in a fixed
//! order, so the body is the same bit for bit on any number of threads (C4).
//!
//! Faces are triangulated internally. Intersection points stay implicit ([`Lpi`]) and every
//! decision is an exact predicate on the original coordinates. A point is named by the segment
//! that crosses a face, `(segment, face)` (§7.2), so the same point found from different triangle
//! pairs is recognised without tolerances. Output faces are traced per input face from the kept
//! parts of its boundary and the cut curves across it. Points where a cut crosses a triangulation
//! diagonal are dropped there, so the triangulation reaches neither the result nor its `TopoId`s.
//!
//! General position only: any contact the kernel does not resolve (coplanar faces, a vertex on a
//! face, two edges crossing) is refused as [`BrepError::Degenerate`].

use std::cmp::Ordering;
use std::collections::VecDeque;

use datum_brep::{Body, BrepError, FaceSpec, Ix, Plane, TopoId, Vertex, build, in_order, triangulate_face};
use datum_math::{Lpi, Point, Sign, Vec3, cmp_axis, orient2d_along, orient3d};

use rayon::iter::Either;
use rayon::prelude::*;
use smallvec::SmallVec;

use crate::bounds::{Aabb, bounds, overlap, overlapping_pairs};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum BooleanKind {
    Union,
    /// The target minus every tool.
    Difference,
    Intersection,
}

impl BooleanKind {
    pub fn name(self) -> &'static str {
        match self {
            Self::Union => "union",
            Self::Difference => "difference",
            Self::Intersection => "intersection",
        }
    }

    /// Whether part `x` (0 = target) keeps the part of its surface inside the other side: the
    /// tools for the target, the target for a tool.
    fn keeps_inside(self, x: Part) -> bool {
        match self {
            Self::Union => false,
            Self::Intersection => true,
            Self::Difference => x != 0,
        }
    }

    /// Tools of a difference bound the result from the other side: their faces turn inward.
    fn flips(self, x: Part) -> bool {
        self == Self::Difference && x != 0
    }
}

fn degenerate<T>(msg: impl Into<String>) -> Result<T, BrepError> {
    Err(BrepError::Degenerate(msg.into()))
}

/// Index of an input body: 0 is the target, then the tools.
type Part = usize;
/// A triangle: part and index in that part's triangles.
type TriRef = (Part, usize);

/// A side of an internal triangle: a B-rep edge, or a triangulation diagonal between two vertices.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Seg {
    Edge(Part, Ix),
    Diagonal(Part, Ix, Ix),
}

/// An intersection point: segment `seg` of one part crosses face `face` of part `part`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct Cross {
    seg: Seg,
    part: Part,
    face: Ix,
}

/// A point of an output loop: an input vertex or an intersection point.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Node {
    Vert(Part, Ix),
    Cross(Cross),
}

struct Tri {
    face: Ix,
    v: [Ix; 3],
    /// Side `k` runs from `v[k]` to `v[k + 1]`.
    segs: [Seg; 3],
    /// A projection axis that keeps the triangle non-degenerate, and its turning sense there.
    axis: usize,
    ccw: Sign,
    bounds: Aabb,
}

struct Input<'a> {
    body: &'a Body,
    tris: Vec<Tri>,
    bounds: Aabb,
}

impl<'a> Input<'a> {
    fn new(x: Part, body: &'a Body) -> Result<Self, BrepError> {
        // Directed half-edges `(origin, destination) → (face, edge)` label triangle sides.
        let mut directed: Vec<((Ix, Ix), (Ix, Ix))> = (0..body.half_edges().len() as Ix)
            .map(|h| {
                let he = &body.half_edges()[h as usize];
                let to = body.half_edges()[body.next(h) as usize].origin;
                ((he.origin, to), (body.loops()[he.loop_ix as usize].face, he.edge))
            })
            .collect();
        directed.par_sort_unstable();
        let face_tris = |f: Ix| -> Result<Vec<Tri>, BrepError> {
            let mut tris = Vec::new();
            for v in triangulate_face(body, f)? {
                let segs = [0, 1, 2].map(|k| {
                    let (a, b) = (v[k], v[(k + 1) % 3]);
                    match directed.binary_search_by_key(&(a, b), |d| d.0) {
                        Ok(i) if directed[i].1.0 == f => Seg::Edge(x, directed[i].1.1),
                        _ => Seg::Diagonal(x, a.min(b), a.max(b)),
                    }
                });
                let p = v.map(|i| body.point(i));
                let e = p.map(Point::Explicit);
                let n = (p[1] - p[0]).cross(p[2] - p[0]).dominant_axis();
                let Some((axis, ccw)) =
                    [n, (n + 1) % 3, (n + 2) % 3].into_iter().map(|a| (a, orient2d_along(&e[0], &e[1], &e[2], a))).find(|(_, s)| *s != Sign::Zero)
                else {
                    return degenerate(format!("face {:?} has a zero-area triangle", body.faces()[f as usize].id));
                };
                tris.push(Tri { face: f, v, segs, axis, ccw, bounds: bounds(p.iter().copied()) });
            }
            Ok(tris)
        };
        let tris: Vec<Tri> = in_order((0..body.faces().len() as Ix).into_par_iter().map(face_tris))?.into_iter().flatten().collect();
        Ok(Self { body, tris, bounds: bounds(body.vertices().iter().map(|v| v.point)) })
    }

    fn points(&self, t: usize) -> [Vec3; 3] {
        self.tris[t].v.map(|i| self.body.point(i))
    }
}

/// Up to two end points of a common segment, on the stack: one pair of triangles must not allocate.
type Ends = SmallVec<[(Cross, Hit); 4]>;

/// An intersection point and a triangle of the crossed face whose plane it lies on.
#[derive(Clone, Copy)]
struct Hit {
    point: Point,
    plane: TriRef,
}

/// Intersection points sorted by name. Names and points are kept apart: a lookup reads only the
/// compact names.
struct Hits {
    keys: Vec<Cross>,
    hits: Vec<Hit>,
}

impl Hits {
    /// Merges the points of all triangle pairs, given in pair order with the first error, if
    /// any. A name found by several pairs must name one point; the first pair's copy is kept.
    /// Only `(name, position)` is sorted, and the points are moved once.
    fn new(all: Vec<Result<(Cross, Hit), BrepError>>) -> Result<Self, BrepError> {
        if let Some(e) = all.iter().find_map(|r| r.as_ref().err()) {
            return Err(e.clone());
        }
        let all: Vec<&(Cross, Hit)> = all.iter().flatten().collect();
        let mut order: Vec<(Cross, u32)> = all.par_iter().enumerate().map(|(i, e)| (e.0, i as u32)).collect();
        order.par_sort_unstable();
        let differ = |p: &Point, q: &Point| p != q && (0..3).any(|a| cmp_axis(p, q, a) != Ordering::Equal);
        if order.par_windows(2).any(|w| w[0].0 == w[1].0 && differ(&all[w[0].1 as usize].1.point, &all[w[1].1 as usize].1.point)) {
            return degenerate("a segment crosses a face more than once");
        }
        order.dedup_by_key(|e| e.0);
        Ok(Self { keys: order.par_iter().map(|e| e.0).collect(), hits: order.par_iter().map(|e| all[e.1 as usize].1).collect() })
    }

    fn index(&self, c: &Cross) -> usize {
        self.keys.partition_point(|k| k < c)
    }

    fn get(&self, c: &Cross) -> &Hit {
        &self.hits[self.index(c)]
    }
}

/// What the triangle pairs found.
struct Found {
    hits: Hits,
    /// Per part and edge: the points on that edge, sorted from the origin of its `half`.
    on_edges: Vec<Vec<Vec<Cross>>>,
    /// Cut segments by part and face, oriented with the kept side on the left seen from outside
    /// the part.
    cuts: Vec<((Part, Ix), (Node, Node))>,
}

impl Found {
    fn face_cuts(&self, x: Part, f: Ix) -> impl Iterator<Item = (Node, Node)> + '_ {
        let from = self.cuts.partition_point(|c| c.0 < (x, f));
        self.cuts[from..].iter().take_while(move |c| c.0 == (x, f)).map(|c| c.1)
    }

    fn point(&self, inputs: &[Input], n: Node) -> Point {
        match n {
            Node::Vert(x, v) => Point::Explicit(inputs[x].body.point(v)),
            Node::Cross(c) => self.hits.get(&c).point,
        }
    }
}

/// The target (`bodies[0]`) combined with the tools (`bodies[1..]`, pairwise disjoint) for the
/// feature with ID `feature`. Target elements keep their IDs; tool elements get IDs derived from
/// the feature and the tool's position, so the same body may serve as several inputs. With
/// `through`, tools first reach past the target along that direction (see [`through_target`]).
pub fn boolean(feature: TopoId, kind: BooleanKind, through: Option<Vec3>, bodies: &[&Body]) -> Result<Body, BrepError> {
    let stretched = match (through, bodies.split_first()) {
        (Some(dir), Some((target, tools))) => tools.iter().map(|t| through_target(target, t, dir)).collect::<Result<Vec<_>, _>>()?,
        _ => Vec::new(),
    };
    let bodies: Vec<&Body> = if through.is_some() { bodies.iter().copied().take(1).chain(&stretched).collect() } else { bodies.to_vec() };
    let inputs = in_order(bodies.par_iter().enumerate().map(|(x, b)| Input::new(x, b)))?;
    // Pairs in parallel, merged in pair order.
    let pairs = candidate_pairs(&inputs);
    // One pass splits the items; errors travel with the points, so the first one in pair order is
    // reported.
    let (mut cuts, ends): (Vec<_>, Vec<_>) = pairs.par_iter().flat_map_iter(|&(a, b)| pair_items(&inputs, kind, a, b)).partition_map(|i| match i {
        Ok(Item::Cut(face, cut)) => Either::Left((face, cut)),
        Ok(Item::End(key, hit)) => Either::Right(Ok((key, hit))),
        Err(e) => Either::Right(Err(e)),
    });
    cuts.par_sort_by_key(|c| c.0);
    let hits = Hits::new(ends)?;
    let mut on_edges: Vec<Vec<Vec<Cross>>> = inputs.iter().map(|i| vec![Vec::new(); i.body.edges().len()]).collect();
    for key in &hits.keys {
        if let Seg::Edge(x, edge) = key.seg {
            on_edges[x][edge as usize].push(*key);
        }
    }
    let mut found = Found { hits, on_edges, cuts };
    check_nested_tools(&inputs)?;
    for (input, edges) in inputs.iter().zip(&mut found.on_edges) {
        in_order(edges.par_iter_mut().enumerate().map(|(e, on)| sort_along_edge(input.body, e as Ix, on, &found.hits)))?;
    }
    let status = in_order((0..inputs.len()).into_par_iter().map(|x| classify(&inputs, x, &found)))?;
    assemble(feature, kind, &inputs, &found, &status)
}

/// What a pair of triangles adds: a cut segment across a face of a part, or an intersection point.
enum Item {
    Cut((Part, Ix), (Node, Node)),
    End(Cross, Hit),
}

/// The items of one pair, or its error, without allocating.
fn pair_items(inputs: &[Input], kind: BooleanKind, a: TriRef, b: TriRef) -> SmallVec<[Result<Item, BrepError>; 4]> {
    let mut items = SmallVec::new();
    if let Err(e) = pair_contact(inputs, kind, a, b, &mut items) {
        items.clear();
        items.push(Err(e));
    }
    items
}

fn pair_contact(
    inputs: &[Input],
    kind: BooleanKind,
    a: TriRef,
    b: TriRef,
    items: &mut SmallVec<[Result<Item, BrepError>; 4]>,
) -> Result<(), BrepError> {
    if a.0 != 0 {
        // Tools may not even touch (D40).
        return match contact(inputs, a, b) {
            Ok(None) => Ok(()),
            _ => degenerate(format!("tools {} and {} intersect or touch", a.0, b.0)),
        };
    }
    let Some(c) = contact(inputs, a, b)? else { return Ok(()) };
    if let [p, q] = &c.ends[..] {
        for (t, signs) in [(a, c.sa), (b, c.sb)] {
            if let Some(cut) = orient_cut(inputs, t, signs, kind.keeps_inside(t.0), p, q)? {
                items.push(Ok(Item::Cut((t.0, inputs[t.0].tris[t.1].face), cut)));
            }
        }
    }
    items.extend(c.ends.into_iter().map(|(key, hit)| Ok(Item::End(key, hit))));
    Ok(())
}

/// `tool` stretched along `dir` so that it reaches past `target` on both sides by the target's own
/// extent along `dir` (D40). The map is affine: faces stay planar, IDs stay, and a prism along
/// `dir` keeps its section, so a hole stays through when the target grows thicker.
fn through_target(target: &Body, tool: &Body, dir: Vec3) -> Result<Body, BrepError> {
    let dd = dir.dot(dir);
    if !(dd > 0.0 && dd.is_finite()) {
        return degenerate("the `through` direction is zero");
    }
    let span =
        |b: &Body| b.vertices().iter().map(|v| v.point.dot(dir)).fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), s| (lo.min(s), hi.max(s)));
    let ((lo, hi), (from, to)) = (span(target), span(tool));
    // [from, to] → [lo − (hi − lo), hi + (hi − lo)].
    let k = 3.0 * (hi - lo) / (to - from);
    let start = 2.0 * lo - hi;
    let point = |p: Vec3| {
        let s = p.dot(dir);
        p + dir * ((start + (s - from) * k - s) / dd)
    };
    let normal = |n: Vec3| n + dir * ((1.0 / k - 1.0) * n.dot(dir) / dd);
    Ok(tool.mapped(point, normal))
}

/// Pairs of triangles of different parts with overlapping boxes. Target against tools, and tools
/// against each other to prove them disjoint (D40).
fn candidate_pairs(inputs: &[Input]) -> Vec<(TriRef, TriRef)> {
    let all: Vec<TriRef> = inputs.iter().enumerate().flat_map(|(x, i)| (0..i.tris.len()).map(move |t| (x, t))).collect();
    let boxes: Vec<Aabb> = all.iter().map(|r| inputs[r.0].tris[r.1].bounds).collect();
    overlapping_pairs(&boxes, |a| all[a].0).into_iter().map(|(a, b)| (all[a], all[b])).collect()
}

/// Whether two bodies are disjoint: their surfaces do not touch and neither lies inside the other.
pub(crate) fn disjoint(a: &Body, b: &Body) -> Result<bool, BrepError> {
    let inputs = [Input::new(0, a)?, Input::new(1, b)?];
    if candidate_pairs(&inputs).into_iter().any(|(s, t)| !matches!(contact(&inputs, s, t), Ok(None))) {
        return Ok(false);
    }
    Ok(!nested(&inputs, 0, 1)?)
}

enum Location {
    Outside,
    Interior,
    OnSide(usize),
    OnVertex,
}

/// Where `p`, a point in the plane of triangle `t`, lies relative to it.
fn locate(t: &Tri, pts: &[Vec3; 3], p: &Point) -> Location {
    let e = pts.map(Point::Explicit);
    let o = [0, 1, 2].map(|k| orient2d_along(&e[k], &e[(k + 1) % 3], p, t.axis));
    if o.contains(&-t.ccw) {
        return Location::Outside;
    }
    match o.iter().filter(|s| **s == Sign::Zero).count() {
        0 => Location::Interior,
        1 => Location::OnSide(o.iter().position(|s| *s == Sign::Zero).unwrap_or(0)),
        _ => Location::OnVertex,
    }
}

/// How two triangles of different parts meet: one or two end points of their common segment, and
/// the sides of each triangle's vertices relative to the other's plane (`Positive`: inside).
struct Contact {
    ends: Ends,
    sa: [Sign; 3],
    sb: [Sign; 3],
}

fn one_side(s: &[Sign; 3]) -> bool {
    s.iter().all(|x| *x == Sign::Positive) || s.iter().all(|x| *x == Sign::Negative)
}

fn contact(inputs: &[Input], a: TriRef, b: TriRef) -> Result<Option<Contact>, BrepError> {
    let (pa, pb) = (inputs[a.0].points(a.1), inputs[b.0].points(b.1));
    let (ea, eb) = (pa.map(Point::Explicit), pb.map(Point::Explicit));
    let sb = eb.each_ref().map(|p| orient3d(&ea[0], &ea[1], &ea[2], p));
    if one_side(&sb) {
        return Ok(None);
    }
    let sa = ea.each_ref().map(|p| orient3d(&eb[0], &eb[1], &eb[2], p));
    if one_side(&sa) {
        return Ok(None);
    }
    if sb.iter().all(|s| *s == Sign::Zero) {
        return if coplanar_overlap(&inputs[a.0].tris[a.1], &ea, &eb) {
            degenerate("faces of two bodies are coplanar and overlap")
        } else {
            Ok(None)
        };
    }
    let mut ends = Ends::new();
    boundary_in(inputs, b, &pb, &sb, a, &pa, &mut ends)?;
    boundary_in(inputs, a, &pa, &sa, b, &pb, &mut ends)?;
    ends.sort_by_key(|e| e.0);
    ends.dedup_by_key(|e| e.0);
    match ends.len() {
        0 => Ok(None),
        1 | 2 => Ok(Some(Contact { ends, sa, sb })),
        _ => degenerate("triangles of two bodies meet in more than a segment"),
    }
}

/// Where the boundary of triangle `a` crosses the plane of triangle `b` inside `b`. `sa` holds
/// the sides of `a`'s vertices relative to `b`'s plane.
fn boundary_in(inputs: &[Input], a: TriRef, pa: &[Vec3; 3], sa: &[Sign; 3], b: TriRef, pb: &[Vec3; 3], ends: &mut Ends) -> Result<(), BrepError> {
    let (ta, tb) = (&inputs[a.0].tris[a.1], &inputs[b.0].tris[b.1]);
    for k in 0..3 {
        if sa[k] == Sign::Zero && !matches!(locate(tb, pb, &Point::Explicit(pa[k])), Location::Outside) {
            let id = inputs[a.0].body.vertices()[ta.v[k] as usize].id;
            return degenerate(format!("vertex {id:?} of one body lies on a face of another"));
        }
        let k1 = (k + 1) % 3;
        if sa[k] == Sign::Zero || sa[k1] == Sign::Zero || sa[k] == sa[k1] {
            continue;
        }
        let Some(lpi) = Lpi::new(pa[k], pa[k1], pb[0], pb[1], pb[2]) else { return degenerate("a crossing edge is parallel to the face") };
        let point = Point::Lpi(lpi);
        let key = match locate(tb, pb, &point) {
            Location::Outside => continue,
            Location::Interior => Cross { seg: ta.segs[k], part: b.0, face: tb.face },
            Location::OnSide(j) => {
                canonical(Cross { seg: ta.segs[k], part: b.0, face: tb.face }, Cross { seg: tb.segs[j], part: a.0, face: ta.face })?
            }
            Location::OnVertex => return degenerate("an edge of one body passes through a vertex of another"),
        };
        ends.push((key, Hit { point, plane: if key.part == b.0 { b } else { a } }));
    }
    Ok(())
}

/// A point on a side of each triangle has two names. A B-rep edge names it (a diagonal is internal);
/// between two diagonals the smaller name wins.
fn canonical(c1: Cross, c2: Cross) -> Result<Cross, BrepError> {
    match (matches!(c1.seg, Seg::Edge(..)), matches!(c2.seg, Seg::Edge(..))) {
        (true, true) => degenerate("edges of two bodies cross"),
        (true, false) => Ok(c1),
        (false, true) => Ok(c2),
        (false, false) => Ok(c1.min(c2)),
    }
}

/// Coplanar triangles overlap unless a side of one has the other strictly outside it.
fn coplanar_overlap(ta: &Tri, ea: &[Point; 3], eb: &[Point; 3]) -> bool {
    let cb = orient2d_along(&eb[0], &eb[1], &eb[2], ta.axis);
    let separated =
        |s: &[Point; 3], c: Sign, o: &[Point; 3]| (0..3).any(|k| o.iter().all(|p| orient2d_along(&s[k], &s[(k + 1) % 3], p, ta.axis) == -c));
    !(separated(ea, ta.ccw, eb) || separated(eb, cb, ea))
}

/// Orients the segment `p, q` in triangle `t` so the side to keep is on its left seen from
/// outside: that side holds a vertex of `t` on the kept side of the other plane. `None` when no
/// vertex is there (the segment lies on a side of `t` and the kept side is the neighbour's).
fn orient_cut(
    inputs: &[Input],
    t: TriRef,
    signs: [Sign; 3],
    inside: bool,
    p: &(Cross, Hit),
    q: &(Cross, Hit),
) -> Result<Option<(Node, Node)>, BrepError> {
    let want = if inside { Sign::Positive } else { Sign::Negative };
    let Some(k) = signs.iter().position(|s| *s == want) else { return Ok(None) };
    let tri = &inputs[t.0].tris[t.1];
    let v = Point::Explicit(inputs[t.0].body.point(tri.v[k]));
    let (a, b) = (Node::Cross(p.0), Node::Cross(q.0));
    match orient2d_along(&p.1.point, &q.1.point, &v, tri.axis) {
        Sign::Zero => degenerate("a cut segment is degenerate"),
        s if s == tri.ccw => Ok(Some((a, b))),
        _ => Ok(Some((b, a))),
    }
}

/// Sorts the points on edge `e` from the origin of its `half`; two at one place are refused.
fn sort_along_edge(body: &Body, e: Ix, on: &mut [Cross], hits: &Hits) -> Result<(), BrepError> {
    if on.len() < 2 {
        return Ok(());
    }
    let h = body.edges()[e as usize].half;
    let from = body.half_edges()[h as usize].origin;
    let to = body.half_edges()[body.half_edges()[h as usize].twin as usize].origin;
    let d = body.point(to) - body.point(from);
    let axis = d.dominant_axis();
    let forward = d.to_array()[axis] > 0.0;
    let cmp = |a: &Cross, b: &Cross| {
        let o = cmp_axis(&hits.get(a).point, &hits.get(b).point, axis);
        if forward { o } else { o.reverse() }
    };
    on.sort_by(cmp);
    if on.windows(2).any(|w| cmp(&w[0], &w[1]) == Ordering::Equal) {
        return degenerate("an edge crosses two faces at one point");
    }
    Ok(())
}

/// Tools that do not touch must not nest either (D40).
fn check_nested_tools(inputs: &[Input]) -> Result<(), BrepError> {
    for j in 1..inputs.len() {
        for k in j + 1..inputs.len() {
            if nested(inputs, j, k)? {
                return degenerate(format!("tools {j} and {k} intersect"));
            }
        }
    }
    Ok(())
}

/// Whether a shell of part `j` lies inside part `k` or the other way round. The parts' surfaces
/// must not touch.
fn nested(inputs: &[Input], j: Part, k: Part) -> Result<bool, BrepError> {
    if !overlap(inputs[j].bounds, inputs[k].bounds) {
        return Ok(false);
    }
    for (a, b) in [(j, k), (k, j)] {
        for v in inputs[a].body.shell_vertices() {
            let p = inputs[a].body.point(v);
            if overlap((p, p), inputs[b].bounds) && ray_inside(inputs, b, p)? {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

/// For every vertex of part `x`: whether it lies inside the other side. Each crossing of an edge
/// flips the answer; the first crossing from an end tells that end's side, and a connected part
/// with no crossing at all is decided by a ray.
fn classify(inputs: &[Input], x: Part, found: &Found) -> Result<Vec<bool>, BrepError> {
    let body = inputs[x].body;
    let n = body.vertices().len();
    let mut status: Vec<Option<bool>> = vec![None; n];
    let mut adjacent: Vec<Vec<(Ix, bool)>> = vec![Vec::new(); n];
    let mut queue = VecDeque::new();
    let inconsistent = || degenerate("vertices cannot be classified consistently");
    for (e, edge) in body.edges().iter().enumerate() {
        let he = &body.half_edges()[edge.half as usize];
        let (u, v) = (he.origin, body.half_edges()[he.twin as usize].origin);
        let on = &found.on_edges[x][e];
        adjacent[u as usize].push((v, on.len() % 2 == 1));
        adjacent[v as usize].push((u, on.len() % 2 == 1));
        let Some(first) = on.first() else { continue };
        let plane = found.hits.get(first).plane;
        let p = inputs[plane.0].points(plane.1).map(Point::Explicit);
        let inside = match orient3d(&p[0], &p[1], &p[2], &Point::Explicit(body.point(u))) {
            Sign::Zero => return degenerate("a vertex lies on a face of another body"),
            s => s == Sign::Positive,
        };
        match status[u as usize] {
            None => {
                status[u as usize] = Some(inside);
                queue.push_back(u);
            }
            Some(s) if s != inside => return inconsistent(),
            Some(_) => {}
        }
    }
    loop {
        while let Some(u) = queue.pop_front() {
            let s = status[u as usize].unwrap_or_default();
            for &(v, flip) in &adjacent[u as usize] {
                match status[v as usize] {
                    None => {
                        status[v as usize] = Some(s ^ flip);
                        queue.push_back(v);
                    }
                    Some(t) if t != s ^ flip => return inconsistent(),
                    Some(_) => {}
                }
            }
        }
        let Some(v) = status.iter().position(Option::is_none) else { break };
        let p = body.point(v as Ix);
        let others: Vec<Part> = if x == 0 { (1..inputs.len()).collect() } else { vec![0] };
        let mut inside = false;
        for y in others {
            if overlap((p, p), inputs[y].bounds) && ray_inside(inputs, y, p)? {
                inside = true;
            }
        }
        status[v] = Some(inside);
        queue.push_back(v as Ix);
    }
    Ok(status.into_iter().map(Option::unwrap_or_default).collect())
}

/// Whether `p` (not on the surface) lies inside part `y`: the parity of crossings of a ray along
/// an axis. A ray through an edge or a vertex is retried along the next axis.
fn ray_inside(inputs: &[Input], y: Part, p: Vec3) -> Result<bool, BrepError> {
    let q = Point::Explicit(p);
    let c = p.to_array();
    'axes: for axis in 0..3 {
        let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
        let mut crossings = 0;
        for (t, tri) in inputs[y].tris.iter().enumerate() {
            let (lo, hi) = (tri.bounds.0.to_array(), tri.bounds.1.to_array());
            if hi[axis] < c[axis] || c[u] < lo[u] || c[u] > hi[u] || c[v] < lo[v] || c[v] > hi[v] {
                continue;
            }
            let e = inputs[y].points(t).map(Point::Explicit);
            let along = orient2d_along(&e[0], &e[1], &e[2], axis);
            let side = orient3d(&e[0], &e[1], &e[2], &q);
            if along == Sign::Zero {
                // The ray is parallel to the triangle: it may graze it only within its plane.
                if side == Sign::Zero {
                    continue 'axes;
                }
                continue;
            }
            let o = [0, 1, 2].map(|k| orient2d_along(&e[k], &e[(k + 1) % 3], &q, axis));
            if o.contains(&-along) {
                continue;
            }
            if o.contains(&Sign::Zero) {
                continue 'axes;
            }
            match side {
                Sign::Zero => return degenerate("a vertex lies on a face of another body"),
                s if s == along => crossings += 1,
                _ => {}
            }
        }
        return Ok(crossings % 2 == 1);
    }
    degenerate("a vertex cannot be classified: rays along every axis graze edges")
}

/// Output IDs: the target keeps its own, tools get theirs derived from the feature and position.
fn out_id(feature: TopoId, x: Part, id: TopoId) -> TopoId {
    if x == 0 { id } else { TopoId::derive("boolean.tool").id(feature).index(x as u64).id(id).finish() }
}

fn assemble(feature: TopoId, kind: BooleanKind, inputs: &[Input], found: &Found, status: &[Vec<bool>]) -> Result<Body, BrepError> {
    // Every intersection point is named once; `None` for points on triangulation diagonals,
    // which are dropped from the result.
    let names: Vec<Option<TopoId>> = found
        .hits
        .keys
        .par_iter()
        .map(|c| match c.seg {
            Seg::Edge(x, e) => {
                let edge = out_id(feature, x, inputs[x].body.edges()[e as usize].id);
                Some(TopoId::derive("boolean.x").id(edge).id(out_id(feature, c.part, inputs[c.part].body.faces()[c.face as usize].id)).finish())
            }
            Seg::Diagonal(..) => None,
        })
        .collect();
    let node_id = |n: Node| match n {
        Node::Vert(x, v) => Some(out_id(feature, x, inputs[x].body.vertices()[v as usize].id)),
        Node::Cross(c) => names[found.hits.index(&c)],
    };
    let all: Vec<(Part, Ix)> = inputs.iter().enumerate().flat_map(|(x, i)| (0..i.body.faces().len() as Ix).map(move |f| (x, f))).collect();
    let per_face = in_order(all.par_iter().map(|&(x, f)| assemble_face(feature, kind, inputs, found, &status[x], &node_id, (x, f))))?;
    let (mut vertices, mut faces) = (Vec::new(), Vec::new());
    for (v, f) in per_face {
        vertices.extend(v);
        faces.extend(f);
    }
    if faces.is_empty() {
        return degenerate("the result is empty");
    }
    // A vertex shared by faces comes from one node, so its copies are equal.
    vertices.par_sort_unstable_by_key(|v: &Vertex| v.id);
    vertices.dedup_by_key(|v| v.id);
    build(vertices, faces)
}

/// The output faces from face `f` of part `x`, and the vertices they use.
fn assemble_face(
    feature: TopoId,
    kind: BooleanKind,
    inputs: &[Input],
    found: &Found,
    status: &[bool],
    node_id: &(impl Fn(Node) -> Option<TopoId> + Sync),
    (x, f): (Part, Ix),
) -> Result<(Vec<Vertex>, Vec<FaceSpec>), BrepError> {
    let body = inputs[x].body;
    let mut pieces: Vec<(Node, Node)> = found.face_cuts(x, f).collect();
    for l in body.face_loops(f) {
        for h in body.loop_half_edges(l) {
            let he = &body.half_edges()[h as usize];
            let (u, v) = (he.origin, body.half_edges()[body.next(h) as usize].origin);
            let on = &found.on_edges[x][he.edge as usize];
            let forward = body.edges()[he.edge as usize].half == h;
            let mut nodes = vec![Node::Vert(x, u)];
            if forward {
                nodes.extend(on.iter().map(|c| Node::Cross(*c)));
            } else {
                nodes.extend(on.iter().rev().map(|c| Node::Cross(*c)));
            }
            nodes.push(Node::Vert(x, v));
            for (i, w) in nodes.windows(2).enumerate() {
                if status[u as usize] ^ (i % 2 == 1) == kind.keeps_inside(x) {
                    pieces.push((w[0], w[1]));
                }
            }
        }
    }
    let (mut vertices, mut faces) = (Vec::new(), Vec::new());
    if pieces.is_empty() {
        return Ok((vertices, faces));
    }
    let face = &body.faces()[f as usize];
    let groups = group_loops(trace(pieces)?, face.plane.normal, &|n| found.point(inputs, n))?;
    let several = groups.len() > 1;
    // A face may have thousands of loops (perforation): they are named in parallel.
    let name_loop = |lp: Vec<Node>| -> Result<(Vec<Vertex>, Vec<TopoId>), BrepError> {
        let mut verts = Vec::with_capacity(lp.len());
        for n in lp {
            let Some(id) = node_id(n) else { continue };
            verts.push(Vertex { id, point: found.point(inputs, n).approx() });
        }
        if verts.len() < 3 {
            return degenerate(format!("a piece of face {:?} collapses", face.id));
        }
        let mut ids: Vec<TopoId> = verts.iter().map(|v| v.id).collect();
        if kind.flips(x) {
            ids.reverse();
        }
        Ok((verts, ids))
    };
    for group in groups {
        let mut loops = Vec::with_capacity(group.len());
        for (verts, ids) in in_order(group.into_par_iter().map(name_loop))? {
            vertices.extend(verts);
            loops.push(ids);
        }
        let own = out_id(feature, x, face.id);
        let id = if several { TopoId::derive("boolean.split").id(own).id(loops[0].iter().copied().min().unwrap_or(own)).finish() } else { own };
        let normal = if kind.flips(x) { -face.plane.normal } else { face.plane.normal };
        faces.push(FaceSpec { id, plane: Plane { origin: face.plane.origin, normal }, loops });
    }
    Ok((vertices, faces))
}

/// Joins pieces `(from, to)` into closed loops: every point must start exactly one piece.
fn trace(mut pieces: Vec<(Node, Node)>) -> Result<Vec<Vec<Node>>, BrepError> {
    pieces.par_sort_unstable();
    if pieces.windows(2).any(|w| w[0].0 == w[1].0) {
        return degenerate("cut curves touch inside a face");
    }
    let mut used = vec![false; pieces.len()];
    let mut loops = Vec::new();
    for start in 0..pieces.len() {
        if used[start] {
            continue;
        }
        let mut lp = Vec::new();
        let mut i = start;
        loop {
            used[i] = true;
            lp.push(pieces[i].0);
            i = pieces.binary_search_by_key(&pieces[i].1, |p| p.0).or_else(|_| degenerate("a cut curve ends inside a face"))?;
            if i == start {
                break;
            }
            if used[i] {
                return Err(BrepError::Invalid("cut curves branch".into()));
            }
        }
        loops.push(lp);
    }
    Ok(loops)
}

/// Splits the loops of one face into faces: each outer loop (counterclockwise seen from outside)
/// with the holes it is the innermost outer loop around. Exact, in the face's dominant projection.
fn group_loops(loops: Vec<Vec<Node>>, normal: Vec3, point: &(impl Fn(Node) -> Point + Sync)) -> Result<Vec<Vec<Vec<Node>>>, BrepError> {
    // A face needs an outer loop: a single loop is one.
    if loops.len() == 1 {
        return Ok(vec![loops]);
    }
    let axis = normal.dominant_axis();
    let convex = if normal.to_array()[axis] < 0.0 { Sign::Negative } else { Sign::Positive };
    let signs = in_order(loops.par_iter().map(|lp| loop_sign(lp, axis, point)))?;
    let (mut outers, mut holes) = (Vec::new(), Vec::new());
    for (lp, sign) in loops.into_iter().zip(signs) {
        if sign == convex {
            outers.push(vec![lp]);
        } else {
            holes.push(lp);
        }
    }
    if outers.len() == 1 {
        outers[0].extend(holes);
        return Ok(outers);
    }
    for hole in holes {
        let p = point(hole[0]);
        let around: Vec<usize> = (0..outers.len()).filter(|&o| encloses(&outers[o][0], &p, axis, point)).collect();
        let inner = around.iter().copied().find(|&o| around.iter().all(|&q| q == o || encloses(&outers[q][0], &point(outers[o][0][0]), axis, point)));
        let Some(o) = inner else { return Err(BrepError::Invalid("a hole lies outside every outer loop".into())) };
        outers[o].push(hole);
    }
    Ok(outers)
}

/// Turning sense of a simple loop in projection along `axis`: the sign at its lowest vertex.
fn loop_sign(lp: &[Node], axis: usize, point: &(impl Fn(Node) -> Point + Sync)) -> Result<Sign, BrepError> {
    let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
    let n = lp.len();
    let pts: Vec<Point> = lp.iter().map(|&x| point(x)).collect();
    let lowest = (0..n).min_by(|&a, &b| cmp_axis(&pts[a], &pts[b], u).then_with(|| cmp_axis(&pts[a], &pts[b], v))).unwrap_or(0);
    match orient2d_along(&pts[(lowest + n - 1) % n], &pts[lowest], &pts[(lowest + 1) % n], axis) {
        Sign::Zero => degenerate("a face loop folds back on itself"),
        s => Ok(s),
    }
}

/// Whether `p` lies inside loop `lp` in projection along `axis`: winding number, exact.
fn encloses(lp: &[Node], p: &Point, axis: usize, point: &impl Fn(Node) -> Point) -> bool {
    let v = (axis + 2) % 3;
    let mut winding = 0i32;
    for i in 0..lp.len() {
        let (a, b) = (point(lp[i]), point(lp[(i + 1) % lp.len()]));
        let (a_above, b_above) = (cmp_axis(&a, p, v) == Ordering::Greater, cmp_axis(&b, p, v) == Ordering::Greater);
        if !a_above && b_above && orient2d_along(&a, &b, p, axis) == Sign::Positive {
            winding += 1;
        } else if a_above && !b_above && orient2d_along(&a, &b, p, axis) == Sign::Negative {
            winding -= 1;
        }
    }
    winding != 0
}
