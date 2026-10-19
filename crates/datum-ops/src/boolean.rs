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
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering as Atomic};
use std::sync::{Mutex, PoisonError};

use datum_brep::{Body, BrepError, FaceSpec, Ix, Plane, TopoId, Vertex, build, in_order, triangulate_face};
use datum_math::{Lpi, Point, Sign, Vec3, cmp_axis, orient2d_along, orient3d, orient3d_coords, ties};

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

/// A point for an error message: where the engineer should look.
fn at(p: Vec3) -> String {
    format!("({}, {}, {})", p.x, p.y, p.z)
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
    /// For a tool, the centre of each vertex's shell: tools grow from it under the symbolic
    /// perturbation (Q7), so a contact counts as an overlap. Empty for the target, which stays.
    centres: Vec<Vec3>,
}

impl<'a> Input<'a> {
    fn new(x: Part, body: &'a Body) -> Result<Self, BrepError> {
        let face_tris = |f: Ix| -> Result<SmallVec<[Tri; 2]>, BrepError> {
            // The face's own half-edges `(origin, destination, edge)` label triangle sides: a side
            // along one is a B-rep edge, any other is a diagonal.
            let mut own: SmallVec<[(Ix, Ix, Ix); 8]> = body
                .face_loops(f)
                .flat_map(|l| body.loop_half_edges(l))
                .map(|h| (body.half_edges()[h as usize].origin, body.half_edges()[body.next(h) as usize].origin, body.half_edges()[h as usize].edge))
                .collect();
            own.sort_unstable();
            let mut tris = SmallVec::new();
            for v in triangulate_face(body, f)? {
                let segs = [0, 1, 2].map(|k| {
                    let (a, b) = (v[k], v[(k + 1) % 3]);
                    match own.binary_search_by_key(&(a, b), |d| (d.0, d.1)) {
                        Ok(i) => Seg::Edge(x, own[i].2),
                        Err(_) => Seg::Diagonal(x, a.min(b), a.max(b)),
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
        // Streamed without a vector per face; the error of the lowest face wins, as in face order.
        let error: Mutex<Option<(Ix, BrepError)>> = Mutex::new(None);
        let tris: Vec<Tri> = (0..body.faces().len() as Ix)
            .into_par_iter()
            .flat_map_iter(|f| {
                face_tris(f).unwrap_or_else(|e| {
                    let mut first = error.lock().unwrap_or_else(PoisonError::into_inner);
                    if first.as_ref().is_none_or(|(g, _)| f < *g) {
                        *first = Some((f, e));
                    }
                    SmallVec::new()
                })
            })
            .collect();
        if let Some((_, e)) = error.into_inner().unwrap_or_else(PoisonError::into_inner) {
            return Err(e);
        }
        let centres = if x == 0 {
            Vec::new()
        } else {
            // The mean of a shell's vertices: strictly inside a convex shell, so every face of it
            // moves outward. (The middle of the bounding box can lie on a face.)
            let shells = body.vertex_shells();
            let mut sums: Vec<(Vec3, f64)> = vec![(Vec3::ZERO, 0.0); shells.iter().max().map_or(0, |&s| s as usize + 1)];
            for (v, &s) in body.vertices().iter().zip(&shells) {
                let e = &mut sums[s as usize];
                *e = (e.0 + v.point, e.1 + 1.0);
            }
            shells.iter().map(|&s| sums[s as usize].0 * (1.0 / sums[s as usize].1)).collect()
        };
        Ok(Self { body, tris, bounds: bounds(body.vertices().iter().map(|v| v.point)), centres })
    }

    /// Vertex `v` as a predicate input: moving if this part is a tool.
    fn point(&self, v: Ix) -> Point {
        match self.centres.get(v as usize) {
            Some(&c) => Point::Moved(self.body.point(v), c),
            None => Point::Explicit(self.body.point(v)),
        }
    }

    fn tri_points(&self, t: usize) -> [Point; 3] {
        self.tris[t].v.map(|i| self.point(i))
    }

    fn coords(&self, t: usize) -> [Vec3; 3] {
        self.tris[t].v.map(|i| self.body.point(i))
    }
}

/// The end points of a common segment, on the stack: one pair of triangles must not allocate.
/// After duplicates are dropped there are at most two; more are found only when a point lies on
/// sides of both triangles.
type Ends = SmallVec<[(Cross, Hit, Tie); 2]>;

/// An intersection point and a triangle of the crossed face whose plane it lies on.
#[derive(Clone, Copy)]
struct Hit {
    point: Point,
    plane: TriRef,
}

/// What the actual geometry says about an intersection point the perturbation placed (Q7).
#[derive(Clone, Copy, Default)]
struct Tie {
    /// It may coincide with other points.
    tied: bool,
    /// It lies on this edge of the other part, between its ends.
    on: Option<(u32, Ix)>,
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
    /// Points tied in the actual geometry come back apart, sorted by name, with what any pair
    /// learned about them: they are few, and the points themselves stay compact.
    fn new(all: Vec<Result<(Cross, Hit, Tie), BrepError>>) -> Result<(Self, Vec<(Cross, Tie)>), BrepError> {
        if let Some(e) = all.iter().find_map(|r| r.as_ref().err()) {
            return Err(e.clone());
        }
        let all: Vec<&(Cross, Hit, Tie)> = all.iter().flatten().collect();
        let mut marked: Vec<(Cross, Tie)> = all.par_iter().filter(|e| e.2.tied || e.2.on.is_some()).map(|e| (e.0, e.2)).collect();
        marked.par_sort_by_key(|e| e.0);
        marked.dedup_by(|later, first| {
            let same = later.0 == first.0;
            if same {
                first.1.tied |= later.1.tied;
                first.1.on = first.1.on.or(later.1.on);
            }
            same
        });
        let mut order: Vec<(Cross, u32)> = all.par_iter().enumerate().map(|(i, e)| (e.0, i as u32)).collect();
        order.par_sort_unstable();
        let differ = |p: &Point, q: &Point| p != q && (0..3).any(|a| cmp_axis(p, q, a) != Ordering::Equal);
        if order.par_windows(2).any(|w| w[0].0 == w[1].0 && differ(&all[w[0].1 as usize].1.point, &all[w[1].1 as usize].1.point)) {
            return degenerate("a segment crosses a face more than once");
        }
        order.dedup_by_key(|e| e.0);
        Ok((Self { keys: order.par_iter().map(|e| e.0).collect(), hits: order.par_iter().map(|e| all[e.1 as usize].1).collect() }, marked))
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
    /// Per part and edge: the points on that edge, sorted from the origin of its `half`, each
    /// with whether the other part crosses there (`false`: it only touches, Q7).
    on_edges: Vec<Vec<Vec<(Cross, bool)>>>,
    /// Points that coincide in the actual geometry, each with the one standing for its group
    /// (Q7), sorted.
    same: Vec<(Node, Node)>,
    /// Whether any decision tied in the actual geometry: only then can pieces of a face coincide.
    tied: bool,
    /// Cut segments by part and face, oriented with the kept side on the left seen from outside
    /// the part.
    cuts: Vec<((Part, Ix), (Node, Node))>,
}

impl Found {
    fn face_cuts(&self, x: Part, f: Ix) -> impl Iterator<Item = (Node, Node)> + '_ {
        let from = self.cuts.partition_point(|c| c.0 < (x, f));
        self.cuts[from..].iter().take_while(move |c| c.0 == (x, f)).map(|c| c.1)
    }

    /// The point standing for `n`: coinciding points are one vertex of the result.
    fn rep(&self, n: Node) -> Node {
        lead(&self.same, n)
    }

    fn point(&self, inputs: &[Input], n: Node) -> Point {
        match n {
            Node::Vert(x, v) => inputs[x].point(v),
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
    let (marks, ends): (Vec<_>, Vec<_>) = pairs.par_iter().flat_map_iter(|&(a, b)| pair_items(&inputs, kind, a, b)).partition_map(|i| match i {
        Ok(Item::Cut(face, cut)) => Either::Left(Ok((face, cut))),
        Ok(Item::Tied(x, v)) => Either::Left(Err(Node::Vert(x, v))),
        Ok(Item::End(key, hit, tie)) => Either::Right(Ok((key, hit, tie))),
        Err(e) => Either::Right(Err(e)),
    });
    let (hits, marked) = Hits::new(ends)?;
    let (mut cuts, mut tied): (Vec<_>, Vec<Node>) = marks.into_par_iter().partition_map(|m| match m {
        Ok(cut) => Either::Left(cut),
        Err(v) => Either::Right(v),
    });
    cuts.par_sort_by_key(|c| c.0);
    let mut on_edges: Vec<Vec<Vec<(Cross, bool)>>> = inputs.iter().map(|i| vec![Vec::new(); i.body.edges().len()]).collect();
    for key in &hits.keys {
        if let Seg::Edge(x, edge) = key.seg {
            on_edges[x][edge as usize].push((*key, true));
        }
    }
    for (key, tie) in &marked {
        if let Some((y, edge)) = tie.on {
            on_edges[y as usize][edge as usize].push((*key, false));
        }
        if tie.tied {
            tied.push(Node::Cross(*key));
        }
    }
    let any_tie = !tied.is_empty();
    let same = coinciding(tied, |n| match n {
        Node::Vert(x, v) => inputs[x].point(v).still(),
        Node::Cross(c) => hits.get(&c).point.still(),
    });
    let tied = any_tie;
    let mut found = Found { hits, on_edges, same, tied, cuts };
    check_nested_tools(&inputs)?;
    let (hits, same) = (&found.hits, &found.same);
    let rep = |n: Node| lead(same, n);
    for (input, edges) in inputs.iter().zip(&mut found.on_edges) {
        in_order(edges.par_iter_mut().enumerate().map(|(e, on)| sort_along_edge(input.body, e as Ix, on, hits, &rep)))?;
    }
    let status = in_order((0..inputs.len()).into_par_iter().map(|x| classify(&inputs, x, &found)))?;
    assemble(feature, kind, &inputs, &found, &status)
}

/// What a pair of triangles adds: a cut segment across a face of a part, an intersection point,
/// or an input vertex that lies on the other part's surface in the actual geometry (Q7). Items
/// stream into vectors by value: boxing the large one would allocate per point.
#[allow(clippy::large_enum_variant)]
enum Item {
    Cut((Part, Ix), (Node, Node)),
    End(Cross, Hit, Tie),
    Tied(Part, Ix),
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
    items.extend(c.ends.into_iter().map(|(key, hit, tie)| Ok(Item::End(key, hit, tie))));
    for (t, ties) in [(a, c.ta), (b, c.tb)] {
        let tri = &inputs[t.0].tris[t.1];
        items.extend((0..3).filter(|&k| ties[k]).map(|k| Ok(Item::Tied(t.0, tri.v[k]))));
    }
    Ok(())
}

/// The node leading the group of `n` in `same` (see [`coinciding`]), or `n` itself.
fn lead(same: &[(Node, Node)], n: Node) -> Node {
    same.binary_search_by_key(&n, |e| e.0).map_or(n, |i| same[i].1)
}

/// Groups points that coincide in the actual geometry (`still` gives that point), comparing them
/// exactly: sorted lexicographically, equal neighbours coincide. Each group is led by its least
/// node, an input vertex when it has one: its ID is stable and its coordinates exact.
fn coinciding(mut nodes: Vec<Node>, still: impl Fn(Node) -> Point + Sync) -> Vec<(Node, Node)> {
    nodes.par_sort_unstable();
    nodes.dedup();
    let cmp = |a: &Node, b: &Node| {
        let (p, q) = (still(*a), still(*b));
        (0..3).map(|axis| cmp_axis(&p, &q, axis)).find(|o| o.is_ne()).unwrap_or(Ordering::Equal)
    };
    nodes.par_sort_by(|a, b| cmp(a, b).then(a.cmp(b)));
    let mut same = Vec::new();
    let mut start = 0;
    for i in 1..=nodes.len() {
        if i == nodes.len() || cmp(&nodes[start], &nodes[i]).is_ne() {
            // Within equal points the group is sorted by node: its first node leads.
            if i - start > 1 {
                same.extend(nodes[start..i].iter().map(|&n| (n, nodes[start])));
            }
            start = i;
        }
    }
    same.par_sort_unstable();
    same
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
    /// On vertex `k` of the triangle.
    OnVertex(usize),
}

/// Where `p`, a point in the plane of triangle `t` with points `e`, lies relative to it, and on
/// which sides' lines it lies in the actual geometry (ties the perturbation settled, Q7).
fn locate(t: &Tri, e: &[Point; 3], p: &Point) -> (Location, [bool; 3]) {
    let found = [0, 1, 2].map(|k| ties(|| orient2d_along(&e[k], &e[(k + 1) % 3], p, t.axis)));
    let (o, tie) = (found.map(|f| f.0), found.map(|f| f.1));
    if o.contains(&-t.ccw) {
        return (Location::Outside, tie);
    }
    let at = match o.iter().filter(|s| **s == Sign::Zero).count() {
        0 => Location::Interior,
        1 => Location::OnSide(o.iter().position(|s| *s == Sign::Zero).unwrap_or(0)),
        // Vertex `k` closes side `k − 1` and opens side `k`.
        _ => Location::OnVertex((0..3).find(|&k| o[k] == Sign::Zero && o[(k + 2) % 3] == Sign::Zero).unwrap_or(0)),
    };
    (at, tie)
}

/// How two triangles of different parts meet: one or two end points of their common segment, and
/// the sides of each triangle's vertices relative to the other's plane (`Positive`: inside).
struct Contact {
    ends: Ends,
    sa: [Sign; 3],
    sb: [Sign; 3],
    /// Vertices of each triangle on the other's plane in the actual geometry (Q7).
    ta: [bool; 3],
    tb: [bool; 3],
}

fn one_side(s: &[Sign; 3]) -> bool {
    s.iter().all(|x| *x == Sign::Positive) || s.iter().all(|x| *x == Sign::Negative)
}

fn contact(inputs: &[Input], a: TriRef, b: TriRef) -> Result<Option<Contact>, BrepError> {
    // Signs on plain coordinates first: in general position they settle every pair, and only a
    // zero needs the moving points and the record of ties (Q7).
    let (pa, pb) = (inputs[a.0].coords(a.1), inputs[b.0].coords(b.1));
    let (ea, eb) = (inputs[a.0].tri_points(a.1), inputs[b.0].tri_points(b.1));
    let sides = |t: &[Vec3; 3], e: &[Point; 3], q: &[Vec3; 3], f: &[Point; 3]| {
        let mut s = q.map(|d| orient3d_coords(t[0], t[1], t[2], d));
        let mut tie = [false; 3];
        for k in 0..3 {
            if s[k] == Sign::Zero {
                (s[k], tie[k]) = ties(|| orient3d(&e[0], &e[1], &e[2], &f[k]));
            }
        }
        (s, tie)
    };
    let (sb, tb) = sides(&pa, &ea, &pb, &eb);
    if one_side(&sb) {
        return Ok(None);
    }
    let (sa, ta) = sides(&pb, &eb, &pa, &ea);
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
    boundary_in(inputs, b, &eb, (&sb, &tb), a, &ea, &mut ends)?;
    boundary_in(inputs, a, &ea, (&sa, &ta), b, &eb, &mut ends)?;
    ends.sort_by_key(|e| e.0);
    ends.dedup_by_key(|e| e.0);
    match ends.len() {
        0 => Ok(None),
        1 | 2 => Ok(Some(Contact { ends, sa, sb, ta, tb })),
        _ => degenerate("triangles of two bodies meet in more than a segment"),
    }
}

/// Where the boundary of triangle `a` crosses the plane of triangle `b` inside `b`. `sa` holds
/// the sides of `a`'s vertices relative to `b`'s plane.
fn boundary_in(
    inputs: &[Input],
    a: TriRef,
    pa: &[Point; 3],
    (sa, tied): (&[Sign; 3], &[bool; 3]),
    b: TriRef,
    pb: &[Point; 3],
    ends: &mut Ends,
) -> Result<(), BrepError> {
    let (ta, tb) = (&inputs[a.0].tris[a.1], &inputs[b.0].tris[b.1]);
    for k in 0..3 {
        if sa[k] == Sign::Zero && !matches!(locate(tb, pb, &pa[k]).0, Location::Outside) {
            return degenerate(format!("a vertex of one body at {} lies on a face of another", at(pa[k].approx())));
        }
        let k1 = (k + 1) % 3;
        if sa[k] == Sign::Zero || sa[k1] == Sign::Zero || sa[k] == sa[k1] {
            continue;
        }
        let Some(lpi) = Lpi::between([&pa[k], &pa[k1]], [&pb[0], &pb[1], &pb[2]]) else {
            return degenerate("a crossing edge is parallel to the face");
        };
        let point = Point::Lpi(lpi);
        let (place, on_line) = locate(tb, pb, &point);
        let key = match place {
            Location::Outside => continue,
            Location::Interior => Cross { seg: ta.segs[k], part: b.0, face: tb.face },
            Location::OnSide(j) => {
                canonical(Cross { seg: ta.segs[k], part: b.0, face: tb.face }, Cross { seg: tb.segs[j], part: a.0, face: ta.face })?
            }
            Location::OnVertex(j) => return degenerate(format!("an edge of one body passes through a vertex of another at {}", at(pb[j].approx()))),
        };
        // In the actual geometry the point is the edge's end on the plane, or it lies on the line
        // of a side of `tb`: on that side when it is an edge of the body, between its ends.
        let on = match (on_line.iter().filter(|t| **t).count(), on_line.iter().position(|t| *t)) {
            (1, Some(j)) => match tb.segs[j] {
                Seg::Edge(y, e) => Some((y as u32, e)),
                Seg::Diagonal(..) => None,
            },
            _ => None,
        };
        // A point named by a diagonal is internal and dropped: it touches no edge of the result.
        let on = on.filter(|_| matches!(key.seg, Seg::Edge(..)));
        let tied = tied[k] || tied[k1] || on_line.contains(&true);
        ends.push((key, Hit { point, plane: if key.part == b.0 { b } else { a } }, Tie { tied, on }));
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
    p: &(Cross, Hit, Tie),
    q: &(Cross, Hit, Tie),
) -> Result<Option<(Node, Node)>, BrepError> {
    let want = if inside { Sign::Positive } else { Sign::Negative };
    let Some(k) = signs.iter().position(|s| *s == want) else { return Ok(None) };
    let tri = &inputs[t.0].tris[t.1];
    let v = inputs[t.0].point(tri.v[k]);
    let (a, b) = (Node::Cross(p.0), Node::Cross(q.0));
    match orient2d_along(&p.1.point, &q.1.point, &v, tri.axis) {
        Sign::Zero => degenerate("a cut segment is degenerate"),
        s if s == tri.ccw => Ok(Some((a, b))),
        _ => Ok(Some((b, a))),
    }
}

/// Sorts the points on edge `e` from the origin of its `half`; two at one place are refused.
fn sort_along_edge(body: &Body, e: Ix, on: &mut [(Cross, bool)], hits: &Hits, rep: &impl Fn(Node) -> Node) -> Result<(), BrepError> {
    if on.len() < 2 {
        return Ok(());
    }
    let h = body.edges()[e as usize].half;
    let from = body.half_edges()[h as usize].origin;
    let to = body.half_edges()[body.half_edges()[h as usize].twin as usize].origin;
    let d = body.point(to) - body.point(from);
    let axis = d.dominant_axis();
    let forward = d.to_array()[axis] > 0.0;
    let cmp = |a: &(Cross, bool), b: &(Cross, bool)| {
        let o = cmp_axis(&hits.get(&a.0).point, &hits.get(&b.0).point, axis);
        if forward { o } else { o.reverse() }
    };
    // Points that coincide in the actual geometry are one vertex: their order does not matter,
    // only that it is fixed.
    on.sort_by(|a, b| cmp(a, b).then(a.0.cmp(&b.0)));
    if on.windows(2).any(|w| cmp(&w[0], &w[1]) == Ordering::Equal && rep(Node::Cross(w[0].0)) != rep(Node::Cross(w[1].0))) {
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
            if overlap((p, p), inputs[b].bounds) && ray_inside(inputs, b, &inputs[a].point(v))? {
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
    // Per vertex: unknown, outside or inside. Statuses follow from parity alone, so the
    // breadth-first walk runs level by level in parallel: a vertex is claimed by one atomic
    // compare-and-set, and a contradiction is seen from whichever end of an edge comes later.
    const UNKNOWN: u8 = 0;
    let code = |inside: bool| if inside { 2 } else { 1 };
    let body = inputs[x].body;
    let n = body.vertices().len();
    let status: Vec<AtomicU8> = (0..n).map(|_| AtomicU8::new(UNKNOWN)).collect();
    // One outgoing half-edge per vertex: its neighbours are walked around its fan, no lists.
    let mut out = vec![0; n];
    for (h, he) in body.half_edges().iter().enumerate() {
        out[he.origin as usize] = h as Ix;
    }
    let inconsistent = || degenerate("vertices cannot be classified consistently");
    let mut frontier = Vec::new();
    for (e, edge) in body.edges().iter().enumerate() {
        let he = &body.half_edges()[edge.half as usize];
        let u = he.origin;
        let Some((first, _)) = found.on_edges[x][e].iter().find(|c| c.1) else { continue };
        let plane = found.hits.get(first).plane;
        let p = inputs[plane.0].tri_points(plane.1);
        let inside = match orient3d(&p[0], &p[1], &p[2], &inputs[x].point(u)) {
            Sign::Zero => return degenerate(format!("a vertex of one body at {} lies on a face of another", at(body.point(u)))),
            s => s == Sign::Positive,
        };
        match status[u as usize].load(Atomic::Relaxed) {
            UNKNOWN => {
                status[u as usize].store(code(inside), Atomic::Relaxed);
                frontier.push(u);
            }
            c if c != code(inside) => return inconsistent(),
            _ => {}
        }
    }
    let conflict = AtomicBool::new(false);
    let mut unclassified = 0;
    loop {
        while !frontier.is_empty() {
            frontier = frontier
                .par_iter()
                .flat_map_iter(|&u| {
                    let s = status[u as usize].load(Atomic::Relaxed) == code(true);
                    let mut claimed: SmallVec<[Ix; 8]> = SmallVec::new();
                    let start = out[u as usize];
                    let mut h = start;
                    loop {
                        let he = &body.half_edges()[h as usize];
                        let v = body.half_edges()[body.next(h) as usize].origin;
                        let want = code(s ^ (found.on_edges[x][he.edge as usize].iter().filter(|c| c.1).count() % 2 == 1));
                        match status[v as usize].compare_exchange(UNKNOWN, want, Atomic::Relaxed, Atomic::Relaxed) {
                            Ok(_) => claimed.push(v),
                            Err(c) if c != want => conflict.store(true, Atomic::Relaxed),
                            Err(_) => {}
                        }
                        h = body.next(he.twin);
                        if h == start {
                            break;
                        }
                    }
                    claimed
                })
                .collect();
            if conflict.load(Atomic::Relaxed) {
                return inconsistent();
            }
        }
        // Classified vertices stay classified: the scan for the next part only moves forward.
        let Some(k) = status[unclassified..].iter().position(|s| s.load(Atomic::Relaxed) == UNKNOWN) else { break };
        unclassified += k;
        let p = body.point(unclassified as Ix);
        let others: Vec<Part> = if x == 0 { (1..inputs.len()).collect() } else { vec![0] };
        let mut inside = false;
        for y in others {
            if overlap((p, p), inputs[y].bounds) && ray_inside(inputs, y, &inputs[x].point(unclassified as Ix))? {
                inside = true;
            }
        }
        status[unclassified].store(code(inside), Atomic::Relaxed);
        frontier.push(unclassified as Ix);
    }
    Ok(status.into_iter().map(|s| s.into_inner() == code(true)).collect())
}

/// Whether `p` (not on the surface) lies inside part `y`: the parity of crossings of a ray along
/// an axis. A ray through an edge or a vertex is retried along the next axis.
fn ray_inside(inputs: &[Input], y: Part, q: &Point) -> Result<bool, BrepError> {
    let p = q.approx();
    let c = p.to_array();
    'axes: for axis in 0..3 {
        let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
        let mut crossings = 0;
        for (t, tri) in inputs[y].tris.iter().enumerate() {
            let (lo, hi) = (tri.bounds.0.to_array(), tri.bounds.1.to_array());
            if hi[axis] < c[axis] || c[u] < lo[u] || c[u] > hi[u] || c[v] < lo[v] || c[v] > hi[v] {
                continue;
            }
            let e = inputs[y].tri_points(t);
            let along = orient2d_along(&e[0], &e[1], &e[2], axis);
            let side = orient3d(&e[0], &e[1], &e[2], q);
            if along == Sign::Zero {
                // The ray is parallel to the triangle: it may graze it only within its plane.
                if side == Sign::Zero {
                    continue 'axes;
                }
                continue;
            }
            let o = [0, 1, 2].map(|k| orient2d_along(&e[k], &e[(k + 1) % 3], q, axis));
            if o.contains(&-along) {
                continue;
            }
            if o.contains(&Sign::Zero) {
                continue 'axes;
            }
            match side {
                Sign::Zero => return degenerate(format!("a vertex of one body at {} lies on a face of another", at(p))),
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
    let mut suspects = Vec::new();
    for (v, f) in per_face {
        vertices.extend(v);
        for (spec, suspect) in f {
            if suspect {
                suspects.push(faces.len());
            }
            faces.push(spec);
        }
    }
    drop_membranes(&mut faces, &suspects);
    if faces.is_empty() {
        return degenerate("the result is empty");
    }
    // A vertex shared by faces comes from one node, so its copies are equal.
    vertices.par_sort_unstable_by_key(|v: &Vertex| v.id);
    vertices.dedup_by_key(|v| v.id);
    build(vertices, faces)
}

/// Output faces, each with whether it may be one side of a membrane, and the vertices they use.
type FaceOut = (Vec<Vertex>, Vec<(FaceSpec, bool)>);

/// The output faces from face `f` of part `x`, and the vertices they use.
fn assemble_face(
    feature: TopoId,
    kind: BooleanKind,
    inputs: &[Input],
    found: &Found,
    status: &[bool],
    node_id: &(impl Fn(Node) -> Option<TopoId> + Sync),
    (x, f): (Part, Ix),
) -> Result<FaceOut, BrepError> {
    let body = inputs[x].body;
    let face = &body.faces()[f as usize];
    // A face that no cut reaches and no crossing splits is kept or dropped whole, as it is: most of
    // a large terrain or plate. Its vertices share one status.
    let half_edges = || body.face_loops(f).flat_map(|l| body.loop_half_edges(l)).map(|h| &body.half_edges()[h as usize]);
    let untouched = found.face_cuts(x, f).next().is_none()
        && half_edges().all(|he| found.on_edges[x][he.edge as usize].is_empty())
        && (found.same.is_empty() || half_edges().all(|he| found.same.binary_search_by_key(&Node::Vert(x, he.origin), |e| e.0).is_err()));
    if untouched {
        let first = body.half_edges()[body.loops()[face.first_loop as usize].first as usize].origin;
        if status[first as usize] != kind.keeps_inside(x) {
            return Ok((Vec::new(), Vec::new()));
        }
        let mut vertices = Vec::new();
        let loops = body
            .face_loops(f)
            .map(|l| {
                let mut ids: Vec<TopoId> = body
                    .loop_half_edges(l)
                    .filter_map(|h| {
                        let v = body.half_edges()[h as usize].origin;
                        let id = node_id(Node::Vert(x, v))?;
                        vertices.push(Vertex { id, point: body.point(v) });
                        Some(id)
                    })
                    .collect();
                if kind.flips(x) {
                    ids.reverse();
                }
                ids
            })
            .collect();
        let normal = if kind.flips(x) { -face.plane.normal } else { face.plane.normal };
        let id = out_id(feature, x, face.id);
        return Ok((vertices, vec![(FaceSpec { id, plane: Plane { origin: face.plane.origin, normal }, loops }, false)]));
    }
    let mut pieces: Vec<(Node, Node)> = found.face_cuts(x, f).collect();
    for l in body.face_loops(f) {
        for h in body.loop_half_edges(l) {
            let he = &body.half_edges()[h as usize];
            let (u, v) = (he.origin, body.half_edges()[body.next(h) as usize].origin);
            let on = &found.on_edges[x][he.edge as usize];
            let forward = body.edges()[he.edge as usize].half == h;
            // Points along the half-edge; the side flips where the other part crosses, not where
            // it only touches.
            let mut along: Vec<(Node, bool)> = vec![(Node::Vert(x, u), false)];
            if forward {
                along.extend(on.iter().map(|c| (Node::Cross(c.0), c.1)));
            } else {
                along.extend(on.iter().rev().map(|c| (Node::Cross(c.0), c.1)));
            }
            along.push((Node::Vert(x, v), false));
            let mut inside = status[u as usize];
            for w in along.windows(2) {
                if inside == kind.keeps_inside(x) {
                    pieces.push((w[0].0, w[1].0));
                }
                inside ^= w[1].1;
            }
        }
    }
    let pieces = if found.tied { cancel(pieces, |n| found.rep(n)) } else { pieces };
    let (mut vertices, mut faces) = (Vec::new(), Vec::new());
    if pieces.is_empty() {
        return Ok((vertices, faces));
    }
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
    let coincident = |n: &Node| found.same.binary_search_by_key(n, |e| e.0).is_ok();
    for group in groups {
        // Every point coincides with another: the face may be one side of a membrane.
        let suspect = group.iter().flatten().all(coincident);
        let mut loops = Vec::with_capacity(group.len());
        for (verts, ids) in in_order(group.into_par_iter().map(name_loop))? {
            vertices.extend(verts);
            loops.push(ids);
        }
        let own = out_id(feature, x, face.id);
        let id = if several { TopoId::derive("boolean.split").id(own).id(loops[0].iter().copied().min().unwrap_or(own)).finish() } else { own };
        let normal = if kind.flips(x) { -face.plane.normal } else { face.plane.normal };
        faces.push((FaceSpec { id, plane: Plane { origin: face.plane.origin, normal }, loops }, suspect));
    }
    Ok((vertices, faces))
}

/// Removes membranes (Q7): a tool that only touches the target with a whole face overlaps it,
/// under the perturbation, by an infinitesimal slab, which leaves two faces over the same loop
/// facing each other. They enclose nothing and go together. Only `suspects`, faces all of whose
/// points coincide with others, can be such.
fn drop_membranes(faces: &mut Vec<FaceSpec>, suspects: &[usize]) {
    let canonical = |l: &[TopoId]| {
        let start = l.iter().enumerate().min_by_key(|(_, id)| **id).map_or(0, |(i, _)| i);
        l[start..].iter().chain(&l[..start]).copied().collect::<Vec<_>>()
    };
    let mut keys: Vec<(Vec<TopoId>, usize)> =
        suspects.iter().filter(|&&f| faces[f].loops.len() == 1).map(|&f| (canonical(&faces[f].loops[0]), f)).collect();
    keys.sort();
    let mut gone = vec![false; faces.len()];
    for k in 0..keys.len() {
        let f = keys[k].1;
        if gone[f] {
            continue;
        }
        let back: Vec<TopoId> = faces[f].loops[0].iter().rev().copied().collect();
        let back = canonical(&back);
        let from = keys.partition_point(|e| e.0 < back);
        if let Some(&(_, g)) = keys[from..].iter().take_while(|e| e.0 == back).find(|e| !gone[e.1] && e.1 != f) {
            gone[f] = true;
            gone[g] = true;
        }
    }
    if gone.contains(&true) {
        let mut i = 0;
        faces.retain(|_| {
            i += 1;
            !gone[i - 1]
        });
    }
}

/// Pieces of a face with coinciding points made one (`rep`, Q7): a piece of zero length goes,
/// and pieces running both ways between two points cancel, since the region between them has no
/// area. What remains bounds the face.
fn cancel(pieces: Vec<(Node, Node)>, rep: impl Fn(Node) -> Node) -> Vec<(Node, Node)> {
    let mut pieces: Vec<(Node, Node)> = pieces.into_iter().map(|(a, b)| (rep(a), rep(b))).filter(|(a, b)| a != b).collect();
    if pieces.len() < 2 {
        return pieces;
    }
    // A point on a triangulation diagonal is internal: it only passes a cut curve on, one piece in
    // and one out. Spliced out, the curve meets any piece running the other way point for point.
    let internal = |n: &Node| matches!(n, Node::Cross(Cross { seg: Seg::Diagonal(..), .. }));
    if pieces.iter().any(|(a, _)| internal(a)) {
        let mut from: Vec<(Node, Node)> = pieces.iter().copied().filter(|(a, _)| internal(a)).collect();
        from.sort_unstable();
        let next = |n: Node| from.binary_search_by_key(&n, |p| p.0).ok().map(|i| from[i].1);
        pieces = pieces
            .iter()
            .filter(|(a, _)| !internal(a))
            .map(|&(a, mut b)| {
                // Bounded: a cut curve cannot pass more internal points than there are.
                for _ in 0..from.len() {
                    match next(b).filter(|_| internal(&b)) {
                        Some(c) => b = c,
                        None => break,
                    }
                }
                (a, b)
            })
            .filter(|(a, b)| a != b)
            .collect();
    }
    // Each undirected piece with a signed count: forward minus backward.
    let mut ends: Vec<((Node, Node), i32)> = pieces.iter().map(|&(a, b)| if a < b { ((a, b), 1) } else { ((b, a), -1) }).collect();
    ends.par_sort_unstable_by_key(|e| e.0);
    if ends.windows(2).all(|w| w[0].0 != w[1].0) {
        return pieces;
    }
    pieces.clear();
    let mut i = 0;
    while i < ends.len() {
        let key = ends[i].0;
        let mut n = 0;
        while i < ends.len() && ends[i].0 == key {
            n += ends[i].1;
            i += 1;
        }
        let piece = if n > 0 { key } else { (key.1, key.0) };
        pieces.extend(std::iter::repeat_n(piece, n.unsigned_abs() as usize));
    }
    pieces
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
