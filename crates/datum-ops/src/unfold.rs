//! The flat blank of sheet metal (M5d, M6; D69, D73): the outline and every cut of a sheet's
//! face laid flat, as the turret press cuts them before bending and rolling, and its DXF.
//!
//! Nothing is unrolled geometrically. The sheet is a tree of strips of plane and cylinder faces on
//! one side (D233), each going on smoothly from the one before across a bend line; each is laid by
//! a rigid map from where that line lies: a plane as it is, a cylinder by the length of its neutral
//! layer, at the K-factor of the thickness from the inside. Cuts keep their exact segments and arcs.
//! The blank is seen from the side of the face it is named by, Y along the bend lines of a chain or
//! up the face, X by the right hand, its corner at the origin. Parts of a blank may lie on one
//! another: it is still laid, and says where ([`Flat::overlaps`]).

use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::TAU;
use std::fmt::Write;
use std::ops::Range;

use datum_brep::{Body, BrepError, Curve, Cylinder, Ix, Surface, TopoId};
use datum_math::{Sign, Vec2, Vec3, orient2d};

use crate::sketch::{find_face, opposite_cylinder};

/// The radius of the neutral layer of a bend between faces of radii `a` and `b`: `k` of the
/// thickness from the inside.
pub(crate) fn neutral(a: f64, b: f64, k: f64) -> f64 {
    a.min(b) + k * (a - b).abs()
}

/// The wall of the piece after the one with wall `w` along a curve of a cut's profile: a curve
/// across bend lines is cut into pieces, a circle's arcs follow one another (D197). The blank
/// joins such pieces back into the curve.
pub(crate) fn next_piece(w: TopoId) -> TopoId {
    TopoId::derive("cut.piece").id(w).finish()
}
/// A piece of a flat blank's outline or of a cut: arcs run counterclockwise from `start` to
/// `end` (radians); a hole cut by a circle is one. A polyline stands for a curve with no exact
/// DXF entity (a plane's cut through a rolled sheet, D211), within [`POLYLINE_DEVIATION`]: the
/// DXF holds it only when the engineer agrees (D147).
#[derive(Clone, Debug, PartialEq)]
pub enum FlatPiece {
    Line(Vec2, Vec2),
    Arc { center: Vec2, radius: f64, start: f64, end: f64 },
    Circle { center: Vec2, radius: f64 },
    Polyline(Vec<Vec2>),
}

/// How far, in millimetres, a polyline of a blank strays from the curve it stands for: a
/// hundredth of what a laser cuts to.
pub const POLYLINE_DEVIATION: f64 = 1e-3;

impl FlatPiece {
    /// The same piece moved by `-lo`.
    fn shifted(&self, lo: Vec2) -> Self {
        let s = |p: Vec2| Vec2::new(p.x - lo.x, p.y - lo.y);
        match self {
            Self::Line(a, b) => Self::Line(s(*a), s(*b)),
            Self::Arc { center, radius, start, end } => Self::Arc { center: s(*center), radius: *radius, start: *start, end: *end },
            Self::Circle { center, radius } => Self::Circle { center: s(*center), radius: *radius },
            Self::Polyline(pts) => Self::Polyline(pts.iter().map(|&p| s(p)).collect()),
        }
    }
}

/// A flat blank: the value of an unfold feature.
#[derive(Clone, Debug, PartialEq)]
pub struct Flat {
    pub pieces: Vec<FlatPiece>,
    /// Width (along X) and height of the blank.
    pub size: Vec2,
    /// The name of each piece, from where it comes (D222): the face of the body beside it (the
    /// wall of a cut, an end of the sheet); a curve of a cut across bend lines by a first piece of
    /// it ([`run_name`]), the same on either side.
    pub ids: Vec<TopoId>,
    /// The bend lines (both tangent lines of each bend): guides on the blank, not in its DXF.
    pub bends: Vec<[Vec2; 2]>,
    /// The name of each bend line: from the two strips of the sheet it lies between, on the side
    /// the blank is seen from (the other side's are other faces).
    pub bend_ids: Vec<TopoId>,
    /// The pieces that are the blank's outer outline: one closed cycle, the rest of the outline
    /// (a cut across a bend line) lies inside it.
    pub outer: Range<usize>,
    /// The ends of each piece of the outer outline, the very vertices where it meets the next, so
    /// that pieces meeting there share a point: a line's as it runs, an arc's counterclockwise.
    pub ends: Vec<[Vec2; 2]>,
    /// The sketch frame (origin, X, Y, Z) on the plane of the face the blank is laid from, such
    /// that a point of the plane at sketch coordinates (X, Y) lies on the blank at (X, Y): a
    /// sketch with this frame lies on the blank (D222). Z is the face's outward normal, the side
    /// the blank is seen from. None for a rolled sheet or a cone.
    pub frame: Option<[Vec3; 4]>,
    /// The face the frame lies on, if there is one.
    pub home: Option<TopoId>,
    /// The K-factor it is laid at: what a sketch on it has to lay its bends at, or else its cuts
    /// across bends, made at this K, do not lie on the sketch's blank.
    pub k_factor: f64,
    /// Where the blank lies on itself, metal on metal, so it can't be cut in one piece until a corner
    /// is cut: a box about each such place. Not in the hash: it follows from the pieces.
    pub overlaps: Vec<[Vec2; 2]>,
    /// Of the pieces, what the DXF holds.
    pub hash: [u8; 32],
}

/// A strip of the sheet in the tree the blank is laid by (D233).
pub(crate) struct Node {
    pub(crate) faces: Vec<Ix>,
    /// The strip it is laid from, and the half-edge of that strip's on the bend line between them.
    pub(crate) parent: Option<(usize, Ix)>,
}

/// How a strip lies on the blank: a rigid map, whatever way it turned from the strip before it.
#[derive(Clone, Copy)]
pub(crate) enum Lay {
    /// A plane: `start` lies at `o`, and `ex` and `ey`, its unit directions, along the blank's X and Y.
    Plane { o: Vec2, start: Vec3, ex: Vec3, ey: Vec3 },
    /// A cylinder: `line`, a point of the bend line it starts from, lies at `o`; the length along
    /// its neutral layer goes along `u` on the blank, the length along the unit `d` along `v`.
    /// `origin` is a point of its axis, `axis` the unit axis it turns about along the sheet, `start`
    /// the unit direction across the axis where it starts, `span` its angle, `neutral` the radius
    /// of its neutral layer.
    Bend { o: Vec2, line: Vec3, d: Vec3, u: Vec2, v: Vec2, origin: Vec3, axis: Vec3, start: Vec3, span: f64, neutral: f64 },
}

impl Lay {
    /// Where `p`, a point of the strip (or of its neutral layer), lies on the blank.
    fn at(&self, p: Vec3) -> Vec2 {
        match *self {
            Self::Plane { o, start, ex, ey } => Vec2::new(o.x + (p - start).dot(ex), o.y + (p - start).dot(ey)),
            Self::Bend { o, line, d, u, v, origin, axis, start, span, neutral } => {
                let w = p - origin;
                let across = w - axis * w.dot(axis);
                let mut theta = libm::atan2(axis.dot(start.cross(across)), start.dot(across));
                // The turn about the axis is cut opposite the middle of the face.
                if theta < 0.5 * span - std::f64::consts::PI {
                    theta += TAU;
                }
                let (a, b) = (neutral * theta, (p - line).dot(d));
                Vec2::new(o.x + u.x * a + v.x * b, o.y + u.y * a + v.y * b)
            }
        }
    }
}

/// The faces that go on smoothly from `x` across an edge of its outline, each once with the first
/// half-edge it is across (a cut across the bend line breaks it into several): outward normals
/// agree along the edge (a bend line), where an end or a cap of the sheet turns by a right angle.
/// Two planes never meet at a bend line (a bend is a cylinder): walls of a mitred corner (D229)
/// meet at 45°, no less apart than a bend line's sides by the normals alone.
pub(crate) fn smooth(body: &Body, x: Ix) -> Vec<(Ix, Ix)> {
    let mut seen = BTreeSet::new();
    let he = |h: Ix| body.half_edges()[h as usize];
    let face_of = |h: Ix| body.loops()[he(h).loop_ix as usize].face;
    let normal = |x: Ix, p: Vec3| body.faces()[x as usize].surface.normal_at(p);
    body.face_loops(x)
        .flat_map(|l| body.loop_half_edges(l))
        .filter_map(|h| {
            let y = face_of(he(h).twin);
            // At a vertex, on both surfaces: the chord of a half circle (a round relief's edge)
            // has its middle on the cylinder's axis, where the normal is rounding alone.
            let m = body.point(he(h).origin);
            let planes = [x, y].iter().all(|&f| matches!(body.faces()[f as usize].surface, Surface::Plane(_)));
            (y != x && !planes && normal(x, m).dot(normal(y, m)) > 0.5).then_some((y, h))
        })
        .filter(|&(y, _)| seen.insert(y))
        .collect()
}

/// The strip of face `x`: the faces of its surface. One face, or the pieces a cut divided it
/// into (D201): they keep its surface bit for bit, so they are found by it, not by nearness.
/// In face order.
pub(crate) fn strip(body: &Body, x: Ix) -> Vec<Ix> {
    let surface = body.faces()[x as usize].surface;
    (0..body.faces().len() as Ix).filter(|&y| body.faces()[y as usize].surface == surface).collect()
}

/// The strips of the sheet that go on smoothly from strip `s`, each once, its own faces left out.
fn beside(body: &Body, s: &[Ix], skip: &BTreeSet<Ix>) -> Vec<Vec<Ix>> {
    let mut out: Vec<Vec<Ix>> = Vec::new();
    for &x in s {
        for (y, _) in smooth(body, x) {
            if !s.contains(&y) && !skip.contains(&y) && !out.iter().any(|t| t.contains(&y)) {
                out.push(strip(body, y));
            }
        }
    }
    out
}

/// The bends of `body` by the way their axes run, each class in face order, the classes in the
/// order they are first met.
fn bend_classes(body: &Body) -> Vec<Vec<Ix>> {
    let axis = |x: Ix| match body.faces()[x as usize].surface {
        Surface::Cylinder(c) => Some(c.axis),
        _ => None,
    };
    let mut classes: Vec<Vec<Ix>> = Vec::new();
    for x in 0..body.faces().len() as Ix {
        // A bend goes on into a wall and has a face across the sheet; a cut's round wall goes only
        // into its other half, a round relief's has nothing across it.
        let goes_on = smooth(body, x).iter().any(|(y, _)| matches!(body.faces()[*y as usize].surface, Surface::Plane(_)));
        let Some(a) = axis(x).filter(|_| goes_on && crate::sketch::opposite_cylinder(body, x as usize).is_some()) else {
            continue;
        };
        match classes.iter_mut().find(|c| axis(c[0]).is_some_and(|b| crate::sketch::nearly_parallel(b, a))) {
            Some(c) => c.push(x),
            None => classes.push(vec![x]),
        }
    }
    classes
}

/// The faces of flanges on the sheet's sides (D205): bends across the sheet's own bends, each with
/// the wall beyond it, which goes on to nothing. The sheet's own bends are those all others are
/// such flanges of; of two ways to see it, the one with more bends, then the one met first. None
/// for a sheet with bends one way only (or none).
pub(crate) fn branches(body: &Body) -> BTreeSet<Ix> {
    let none = BTreeSet::new();
    let classes = bend_classes(body);
    if classes.len() < 2 {
        return none;
    }
    // A flange: its bend's strip meets two strips, one of them a wall meeting nothing else.
    let leaf = |x: Ix| -> Option<Vec<Ix>> {
        let bend = strip(body, x);
        let ways = beside(body, &bend, &none);
        let [a, b] = &ways[..] else { return None };
        let end = |t: &Vec<Ix>| matches!(body.faces()[t[0] as usize].surface, Surface::Plane(_)) && beside(body, t, &none) == [bend.clone()];
        let wall = if end(a) {
            a
        } else if end(b) {
            b
        } else {
            return None;
        };
        Some(bend.iter().chain(wall).copied().collect())
    };
    let mut best: Option<(usize, BTreeSet<Ix>)> = None;
    for (k, main) in classes.iter().enumerate() {
        let others: Option<Vec<Vec<Ix>>> =
            classes.iter().enumerate().filter(|(j, _)| *j != k).flat_map(|(_, c)| c.iter().map(|&x| leaf(x))).collect();
        let Some(others) = others else { continue };
        if best.as_ref().is_none_or(|(n, _)| main.len() > *n) {
            best = Some((main.len(), others.into_iter().flatten().collect()));
        }
    }
    best.map_or(none, |(_, b)| b)
}

/// The chain of strips through face `f`: on from its strip each way while one strip goes on, in
/// order along the sheet (not yet turned the way it is laid); the flanges on its sides
/// (`skip`, see [`branches`]) left out.
pub(crate) fn chain_through(body: &Body, f: Ix, skip: &BTreeSet<Ix>) -> Result<Vec<Vec<Ix>>, BrepError> {
    let unsupported = |m: &str| BrepError::Unsupported(format!("unfold: {m}"));
    let home = strip(body, f);
    let ways = beside(body, &home, skip);
    if ways.len() > 2 {
        return Err(unsupported("the sheet branches further than flanges on its sides: not laid out yet"));
    }
    let mut halves = Vec::new();
    for first in ways {
        let (mut prev, mut cur, mut half) = (home.clone(), first.clone(), vec![first]);
        loop {
            let on: Vec<Vec<Ix>> = beside(body, &cur, skip).into_iter().filter(|t| *t != prev).collect();
            match &on[..] {
                [] => break,
                [t] if *t == home || half.contains(t) => return Err(unsupported("the sheet closes on itself: a closed path comes later")),
                [t] => {
                    half.push(t.clone());
                    (prev, cur) = (cur, t.clone());
                }
                _ => return Err(unsupported("the sheet branches further than flanges on its sides: not laid out yet")),
            }
        }
        halves.push(half);
    }
    let mut chain: Vec<Vec<Ix>> = halves.first().map(|h| h.iter().rev().cloned().collect()).unwrap_or_default();
    chain.push(home);
    chain.extend(halves.into_iter().nth(1).into_iter().flatten());
    Ok(chain)
}

/// A region of faces going on smoothly into each other, a strip's pieces together: a side of a sheet
/// or an edge of it.
struct Region {
    faces: Vec<Ix>,
    /// Its strips join in a ring (a closed path, the rounded rim of a plate): no blank can lie flat
    /// from one.
    ring: bool,
}

/// The regions of `body`'s faces on planes, cylinders and cones, in order of their least face.
fn regions(body: &Body) -> Vec<Region> {
    let faces = body.faces();
    let plain = |x: Ix| matches!(faces[x as usize].surface, Surface::Plane(_) | Surface::Cylinder(_) | Surface::Cone(_));
    // Each face's strip: its least face of the same surface. The pieces a cut leaves keep the
    // surface bit for bit (D201), so sorted bits find them without comparing all with all.
    let bits = |x: Ix| -> [u64; 10] {
        let v = |p: Vec3| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()];
        let (kind, [a, b, c], [d, e, f], g, h, i) = match faces[x as usize].surface {
            Surface::Plane(p) => (0, v(p.origin), v(p.normal), 0, 0, 0),
            Surface::Cylinder(c) => (1, v(c.origin), v(c.axis), c.radius.to_bits(), 0, u64::from(c.convex)),
            Surface::Cone(c) => (2, v(c.origin), v(c.axis), c.radius.to_bits(), c.slope.to_bits(), u64::from(c.convex)),
            Surface::Wall(_) => (3, [0; 3], [0; 3], 0, 0, 0),
        };
        [kind, a, b, c, d, e, f, g, h, i]
    };
    let mut sorted: Vec<([u64; 10], Ix)> = (0..faces.len() as Ix).filter(|&x| plain(x)).map(|x| (bits(x), x)).collect();
    sorted.sort_unstable();
    let mut strip_of: Vec<Ix> = (0..faces.len() as Ix).collect();
    for w in sorted.windows(2) {
        if w[0].0 == w[1].0 {
            strip_of[w[1].1 as usize] = strip_of[w[0].1 as usize];
        }
    }
    // The strips beside each other, each pair once, lower first.
    let mut pairs: BTreeSet<(Ix, Ix)> = BTreeSet::new();
    for x in (0..faces.len() as Ix).filter(|&x| plain(x)) {
        for (y, _) in smooth(body, x).into_iter().filter(|&(y, _)| plain(y)) {
            let (a, b) = (strip_of[x as usize], strip_of[y as usize]);
            if a != b {
                pairs.insert((a.min(b), a.max(b)));
            }
        }
    }
    // Joined by the pairs: a strip's root its region's least strip.
    let mut root: Vec<Ix> = (0..faces.len() as Ix).collect();
    fn find(root: &mut [Ix], mut x: Ix) -> Ix {
        while root[x as usize] != x {
            root[x as usize] = root[root[x as usize] as usize];
            x = root[x as usize];
        }
        x
    }
    // A pair joining what is already joined closes a ring: a tree has none.
    let mut rings: BTreeSet<Ix> = BTreeSet::new();
    for &(a, b) in &pairs {
        let (p, q) = (find(&mut root, a), find(&mut root, b));
        if p == q {
            rings.insert(p);
        } else {
            let (lo, hi) = (p.min(q), p.max(q));
            root[hi as usize] = lo;
            if rings.remove(&hi) {
                rings.insert(lo);
            }
        }
    }
    let mut by_root: BTreeMap<Ix, Vec<Ix>> = BTreeMap::new();
    for x in (0..faces.len() as Ix).filter(|&x| plain(x)) {
        let strip = strip_of[x as usize];
        by_root.entry(find(&mut root, strip)).or_default().push(x);
    }
    by_root.into_iter().map(|(r, faces)| Region { faces, ring: rings.contains(&r) }).collect()
}

/// The faces naming the two sides of the sheet `body` is, the front first: the two largest regions
/// of faces going on smoothly (a plate's two faces; a bent sheet's inner and outer side), however
/// they branch. The front is the side facing more toward +Z, then +Y, then +X: seen from there the
/// blank has the orientation of a sketch on XY. A solid with no two sides standing out is no sheet;
/// nor is one whose largest regions close in a ring: its caps must not pass for its sides.
pub fn sheet_sides(body: &Body) -> Result<[Ix; 2], BrepError> {
    let unsupported = |m: &str| BrepError::Unsupported(format!("unfold: {m}"));
    // Flanges on the sheet's sides are no part of the side's measure: what they add must not turn
    // which side is the front or which face it is named by.
    let skip = branches(body);
    let mut rings = 0.0_f64;
    // Each region: its area, its area vector, the face of it with the largest area.
    let mut sides: Vec<(f64, Vec3, Ix)> = Vec::new();
    for region in regions(body) {
        let faces: Vec<Ix> = region.faces.into_iter().filter(|x| !skip.contains(x)).collect();
        let area: f64 = faces.iter().map(|&x| body.face_area(x)).sum();
        if region.ring {
            rings = rings.max(area);
            continue;
        }
        let Some(&first) = faces.first() else { continue };
        let vector = faces.iter().fold(Vec3::ZERO, |s, &x| s + body.face_area_vector(x));
        let main = faces.iter().copied().max_by(|&a, &b| body.face_area(a).total_cmp(&body.face_area(b))).unwrap_or(first);
        sides.push((area, vector, main));
    }
    sides.sort_by(|a, b| b.0.total_cmp(&a.0));
    let [front, back, rest @ ..] = &sides[..] else { return Err(unsupported("fewer than two sides: not a sheet")) };
    if rest.first().is_some_and(|third| third.0 >= back.0) {
        return Err(unsupported("no two sides stand out: not a sheet"));
    }
    if rings > back.0 {
        return Err(unsupported("the sheet closes on itself: a closed path comes later"));
    }
    let toward = |v: Vec3| [v.z, v.y, v.x];
    let first = toward(front.1).iter().zip(toward(back.1)).find_map(|(a, b)| (*a != b).then_some(*a > b));
    Ok(if first == Some(false) { [back.2, front.2] } else { [front.2, back.2] })
}

/// The faces of both sides of sheet `body`: the strips of each side's tree, however it branches.
pub(crate) fn side_faces(body: &Body) -> Result<BTreeSet<Ix>, BrepError> {
    let mut out = BTreeSet::new();
    for s in sheet_sides(body)? {
        out.extend(side_of(body, s)?);
    }
    Ok(out)
}

/// The flat blank of the sheet one face of which is `face` of `body`, seen from that face's side,
/// the neutral layer of every bend at `k` of the thickness from its inside. Cuts on a bend must
/// have been sketched with the same K-factor: their pieces are then exact.
pub fn unfold(body: &Body, face: &str, k: f64) -> Result<Flat, BrepError> {
    let (f, _) = find_face(body, face)?;
    unfold_face(body, f as Ix, k)
}

/// The flat blank of sheet `body` seen from its front side, or from its back (D164).
pub fn unfold_side(body: &Body, back: bool, k: f64) -> Result<Flat, BrepError> {
    let sides = sheet_sides(body)?;
    unfold_face(body, sides[usize::from(back)], k)
}

/// The tree of strips of the side of the sheet that strip `root` is on: each strip off the one it
/// was reached from, across a bend line. A strip reached twice closes a ring: no blank lies flat.
pub(crate) fn tree(body: &Body, root: Vec<Ix>) -> Result<Vec<Node>, BrepError> {
    let mut owner: BTreeMap<Ix, usize> = root.iter().map(|&x| (x, 0)).collect();
    let mut nodes = vec![Node { faces: root, parent: None }];
    let mut i = 0;
    while i < nodes.len() {
        for x in nodes[i].faces.clone() {
            for (y, h) in smooth(body, x) {
                if matches!(body.faces()[y as usize].surface, Surface::Wall(_)) {
                    continue;
                }
                match owner.get(&y) {
                    Some(&j) if j == i || nodes[i].parent.is_some_and(|p| p.0 == j) || nodes[j].parent.is_some_and(|p| p.0 == i) => {}
                    Some(_) => return Err(BrepError::Unsupported("unfold: the sheet closes on itself: a closed path comes later".into())),
                    None => {
                        let faces: Vec<Ix> = strip(body, y).into_iter().filter(|z| !owner.contains_key(z)).collect();
                        owner.extend(faces.iter().map(|&z| (z, nodes.len())));
                        nodes.push(Node { faces, parent: Some((i, h)) });
                    }
                }
            }
        }
        i += 1;
    }
    Ok(nodes)
}

/// The angle of bend `faces` about `turn` and where it starts: by the arc of its outline (a cap's
/// edge) from bend line to bend line, on each of `beside`, the strips next to it. Laid from a bend
/// line `entry`, it starts there. A bend cut across at both its sides (D202), or relieved at both
/// its corners (D229), has no such edge left: its angle is then the turn between its bend lines.
fn bend_span(body: &Body, faces: &[Ix], c: Cylinder, turn: Vec3, entry: Option<Vec3>, beside: &[&[Ix]]) -> Result<(f64, Vec3), BrepError> {
    let he = |h: Ix| body.half_edges()[h as usize];
    let on_line = |v: Ix, t: &[Ix]| t.iter().any(|&y| body.face_loops(y).flat_map(|l| body.loop_half_edges(l)).any(|g| he(g).origin == v));
    let arc = faces.iter().flat_map(|&x| body.loop_half_edges(body.faces()[x as usize].first_loop)).find_map(|h| {
        let e = &body.edges()[he(h).edge as usize];
        let arc = body.edge_arc(e)?;
        let arc = if e.half == h { arc } else { arc.reversed() };
        let (v, w) = (he(h).origin, he(body.next(h)).origin);
        // Each bend line there is (a rolled sheet alone has none) holds an end of it.
        let whole = [v, w].iter().filter(|&&x| beside.iter().any(|t| on_line(x, t))).count() >= beside.len();
        whole.then(|| (arc, body.point(v), body.point(w)))
    });
    if let Some((arc, p, q)) = arc {
        return Ok((arc.sweep(p, q), entry.unwrap_or(if arc.axis.dot(turn) > 0.0 { p } else { q })));
    }
    let line = |t: &[Ix]| seam(body, faces, t).map(|(_, h)| body.point(he(h).origin));
    let exit = beside.iter().filter_map(|t| line(t)).find(|&q| Some(q) != entry);
    let (Some(from), Some(to)) = (entry, exit) else { return Err(BrepError::Unsupported("unfold: a bend without an edge across it".into())) };
    let radial = |q: Vec3| {
        let w = q - c.origin;
        unit(w - c.axis * w.dot(c.axis))
    };
    let (u, v, t) = (radial(from), radial(to), unit(turn));
    Ok((libm::atan2(t.dot(u.cross(v)), u.dot(v)).abs(), from))
}

/// Where strip `s` meets strip `t`: a face of `s` and its half-edge on the bend line.
fn seam(body: &Body, s: &[Ix], t: &[Ix]) -> Option<(Ix, Ix)> {
    s.iter().find_map(|&x| smooth(body, x).into_iter().find(|(y, _)| t.contains(y)).map(|(_, h)| (x, h)))
}

/// How node `i` of `nodes` lies, and the way it goes on from its bend line, from the way its parent
/// `from` lies: the bend line stays where it
/// is on the blank and the strip goes on across it. A bend line along an axis of the parent keeps
/// that axis bit for bit, so bend lines of a sheet with square turns stay exactly on the blank's
/// axes.
fn lay_from(body: &Body, nodes: &[Node], i: usize, from: Lay, h: Ix, k: f64) -> Result<(Lay, Vec2), BrepError> {
    let unsupported = |m: &str| BrepError::Unsupported(format!("unfold: {m}"));
    let he = |g: Ix| body.half_edges()[g as usize];
    let normal = |x: Ix, p: Vec3| unit(body.faces()[x as usize].surface.normal_at(p));
    let face_of = |g: Ix| body.loops()[he(g).loop_ix as usize].face;
    let node = &nodes[i];
    let sign = |x: f64| if x > 0.0 { 1.0 } else { -1.0 };
    let chord = body.point(he(body.next(h)).origin) - body.point(he(h).origin);
    // Along the axis of the bend on either side of the line: its ends may be where a cut crosses
    // it, off the axis's direction by rounding. Two planes never meet at a bend line.
    let axis = [face_of(h), face_of(he(h).twin)].iter().find_map(|&x| match body.faces()[x as usize].surface {
        Surface::Cylinder(c) => Some(unit(c.axis)),
        _ => None,
    });
    let along = axis.map(|a| a * sign(a.dot(chord))).ok_or_else(|| unsupported("a bend line off a cylinder"))?;
    // The line's direction `d`, `v` on the blank, and `dir` as the half-edge runs; `u` across it,
    // away from the parent: the parent lies to the left of the half-edge, seen from outside.
    let (d, v, u, dir) = match from {
        Lay::Plane { ex, ey, .. } => {
            let (d, v) = if crate::sketch::parallel(along, ey) {
                (ey, Vec2::new(0.0, 1.0))
            } else if crate::sketch::parallel(along, ex) {
                (ex, Vec2::new(1.0, 0.0))
            } else {
                let (x, y) = (along.dot(ex), along.dot(ey));
                let n = libm::sqrt(x * x + y * y);
                (along, Vec2::new(x / n, y / n))
            };
            let dir = d * sign(along.dot(d));
            let away = dir.cross(normal(face_of(h), body.point(he(h).origin)));
            let s = sign(away.dot(ex) * v.y - away.dot(ey) * v.x);
            (d, v, Vec2::new(s * v.y, -s * v.x), dir)
        }
        Lay::Bend { d, u, v, .. } => (d, v, u, d * sign(along.dot(d))),
    };
    let (x, g) = seam(body, &node.faces, &nodes[node.parent.map_or(0, |p| p.0)].faces)
        .ok_or_else(|| BrepError::Invalid("unfold: no bend line between strips of the sheet".into()))?;
    let line = body.point(he(g).origin);
    let o = from.at(line);
    let on = unit(dir.cross(normal(x, line)));
    match body.faces()[x as usize].surface {
        Surface::Plane(_) => {
            let (mx, my) = (on * u.x + d * v.x, on * u.y + d * v.y);
            Ok((Lay::Plane { o, start: line, ex: mx, ey: my }, u))
        }
        Surface::Cylinder(c) => {
            let w = line - c.origin;
            let start = unit(w - c.axis * w.dot(c.axis));
            let axis = d * sign(start.cross(on).dot(d));
            let beside: Vec<&[Ix]> = nodes
                .iter()
                .enumerate()
                .filter(|(j, n)| Some(*j) == node.parent.map(|p| p.0) || n.parent.is_some_and(|p| p.0 == i))
                .map(|(_, n)| &n.faces[..])
                .collect();
            let (span, _) = bend_span(body, &node.faces, c, axis, Some(line), &beside)?;
            let (_, other) = opposite_cylinder(body, x as usize).ok_or_else(|| unsupported("a bend without the face across the sheet"))?;
            Ok((Lay::Bend { o, line, d, u, v, origin: c.origin, axis, start, span, neutral: neutral(c.radius, other.radius, k) }, u))
        }
        Surface::Cone(_) => Err(unsupported("a cone joined to other faces of a sheet")),
        Surface::Wall(_) => Err(unsupported("the wall of a cut is not a face of the sheet")),
    }
}

/// How each of `nodes` lies, the first as `root`, each other from its parent; with the way each goes
/// on across its bend line on the blank.
pub(crate) fn lay_tree(body: &Body, nodes: &[Node], root: Lay, k: f64) -> Result<Vec<(Lay, Vec2)>, BrepError> {
    let mut lays: Vec<(Lay, Vec2)> = Vec::with_capacity(nodes.len());
    for (i, node) in nodes.iter().enumerate() {
        lays.push(match node.parent {
            Some((p, h)) => lay_from(body, nodes, i, lays[p].0, h, k)?,
            None => (root, Vec2::new(0.0, 0.0)),
        });
    }
    Ok(lays)
}

/// The faces of the side of sheet `body` that face `f` is on: the strips of its tree.
pub(crate) fn side_of(body: &Body, f: Ix) -> Result<BTreeSet<Ix>, BrepError> {
    Ok(tree(body, strip(body, f))?.into_iter().flat_map(|n| n.faces).collect())
}

/// The direction the blank's Y runs along and the strip it is laid from. A sheet that is a chain
/// (with flanges on its sides) keeps Y along its bends. One that branches further takes Y along the
/// class of the most bends, or up the strip as a sketch would see it if none lies along it.
pub(crate) fn axes(body: &Body, f: Ix, chain: Option<Vec<Vec<Ix>>>) -> Result<(Vec3, Vec<Ix>), BrepError> {
    let unsupported = |m: &str| BrepError::Unsupported(format!("unfold: {m}"));
    let surface = |x: Ix| body.faces()[x as usize].surface;
    let is_plane = |x: Ix| matches!(surface(x), Surface::Plane(_));
    // Y up the face, world +Z projected onto it; +Y on a face lying flat.
    let upright = |x: Ix| {
        let n = unit(
            surface(x).normal_at(body.point(body.half_edges()[body.loop_half_edges(body.faces()[x as usize].first_loop).start as usize].origin)),
        );
        let up = if n.cross(Vec3::new(0.0, 0.0, 1.0)) == Vec3::ZERO { Vec3::new(0.0, 1.0, 0.0) } else { Vec3::new(0.0, 0.0, 1.0) };
        unit(up - n * (up.dot(n) / n.dot(n)))
    };
    let cylinder = |x: Ix| match surface(x) {
        Surface::Cylinder(c) => Some(c.axis),
        _ => None,
    };
    // A chain keeps Y along its bends only if they all run one way: a tab's bend on a wall's end,
    // not along the wall's plane, is no axis of the blank (D233).
    let one_way = |c: &Vec<Vec<Ix>>| {
        let mut bends = c.iter().flatten().filter_map(|&x| cylinder(x));
        bends.next().is_none_or(|first| bends.all(|a| crate::sketch::nearly_parallel(first, a)))
    };
    if let Some(mut chain) = chain.filter(one_way) {
        let beside = |x: Ix| smooth(body, x).into_iter().find_map(|(y, _)| cylinder(y));
        let a = unit(chain.iter().flatten().find_map(|&x| cylinder(x)).or_else(|| beside(f)).unwrap_or_else(|| upright(chain[0][0])));
        if is_plane(f) {
            return Ok((a, strip(body, f)));
        }
        if let Some(wall) = chain.iter().find(|s| is_plane(s[0])) {
            return Ok((a, wall.clone()));
        }
        // All rolled: laid from the end `a × n` points away from.
        if chain.len() > 1 {
            let he = |h: Ix| body.half_edges()[h as usize];
            if let Some((x, h)) = seam(body, &chain[0], &chain[1]) {
                let m = (body.point(he(h).origin) + body.point(he(body.next(h)).origin)) * 0.5;
                if (m - centroid(body, x)).dot(a.cross(surface(x).normal_at(m))) < 0.0 {
                    chain.reverse();
                }
            }
        }
        return Ok((a, chain[0].clone()));
    }
    let nodes = tree(body, strip(body, f))?;
    let root = nodes.iter().find(|n| is_plane(n.faces[0])).ok_or_else(|| unsupported("a branching sheet with no flat wall"))?.faces.clone();
    // Y along the class of the most bends, the first of equals, as for a chain: a tray keeps its
    // blank's way as rims go on. A class square to the strip has no direction along it.
    let n = surface(root[0])
        .normal_at(body.point(body.half_edges()[body.loop_half_edges(body.faces()[root[0] as usize].first_loop).start as usize].origin));
    let mut classes = bend_classes(body);
    classes.sort_by_key(|c| std::cmp::Reverse(c.len()));
    let a = classes.iter().filter_map(|c| cylinder(c[0])).find(|&a| !crate::sketch::parallel(a, n)).map_or_else(
        || upright(root[0]),
        |a| {
            let n = unit(n);
            a - n * a.dot(n)
        },
    );
    Ok((unit(a), root))
}

fn unfold_face(body: &Body, f: Ix, k: f64) -> Result<Flat, BrepError> {
    let unsupported = |m: &str| BrepError::Unsupported(format!("unfold: {m}"));
    if !(0.0..=1.0).contains(&k) {
        return Err(BrepError::Degenerate(format!("a K-factor lies between 0 and 1, got {k}")));
    }
    let he = |h: Ix| body.half_edges()[h as usize];
    let face_of = |h: Ix| body.loops()[he(h).loop_ix as usize].face;
    let normal = |x: Ix, p: Vec3| body.faces()[x as usize].surface.normal_at(p);
    let id = |x: Ix| body.faces()[x as usize].id;
    let chain = chain_through(body, f, &branches(body)).ok();
    if let Some(chain) = chain.as_ref().filter(|c| c.iter().flatten().any(|&x| matches!(body.faces()[x as usize].surface, Surface::Cone(_)))) {
        return match &chain[..] {
            [one] if one.len() == 1 => cone_blank(body, one[0], k),
            _ => Err(unsupported("a cone joined to other faces of a sheet")),
        };
    }
    let (a, root) = axes(body, f, chain)?;
    let nodes = tree(body, root)?;
    let root_lay = {
        let node = &nodes[0];
        let x = node.faces[0];
        let at = body.point(he(body.loop_half_edges(body.faces()[x as usize].first_loop).start).origin);
        match body.faces()[x as usize].surface {
            Surface::Plane(p) => {
                let ex = unit(a.cross(unit(p.normal)));
                let points = node.faces.iter().flat_map(|&x| body.loop_points(body.faces()[x as usize].first_loop));
                let start = points.min_by(|p, q| p.dot(ex).total_cmp(&q.dot(ex))).unwrap_or(at);
                Lay::Plane { o: Vec2::new(0.0, 0.0), start, ex, ey: a }
            }
            Surface::Cylinder(c) => {
                let axis = unit(a * if c.convex { 1.0 } else { -1.0 });
                let beside: Vec<&[Ix]> = nodes.iter().filter(|n| n.parent.is_some_and(|p| p.0 == 0)).map(|n| &n.faces[..]).collect();
                let (span, from) = bend_span(body, &node.faces, c, axis, None, &beside)?;
                let w = from - c.origin;
                let start = unit(w - axis * w.dot(axis));
                let (_, other) = opposite_cylinder(body, x as usize).ok_or_else(|| unsupported("a bend without the face across the sheet"))?;
                Lay::Bend {
                    o: Vec2::new(0.0, 0.0),
                    line: from,
                    d: a,
                    u: Vec2::new(1.0, 0.0),
                    v: Vec2::new(0.0, 1.0),
                    origin: c.origin,
                    axis,
                    start,
                    span,
                    neutral: neutral(c.radius, other.radius, k),
                }
            }
            Surface::Cone(_) => return Err(unsupported("a cone in a chain of the sheet")),
            Surface::Wall(_) => return Err(unsupported("the wall of a cut is not a face of the sheet")),
        }
    };
    let lays: Vec<Lay> = lay_tree(body, &nodes, root_lay, k)?.into_iter().map(|l| l.0).collect();
    let laid: Vec<(usize, Ix, Lay)> = nodes.iter().zip(&lays).enumerate().flat_map(|(i, (n, &l))| n.faces.iter().map(move |&x| (i, x, l))).collect();
    // Every vertex where the first face of the tree that has it lays it.
    let mut at: BTreeMap<Ix, Vec2> = BTreeMap::new();
    for (_, x, l) in &laid {
        for lp in body.face_loops(*x) {
            for h in body.loop_half_edges(lp) {
                let v = he(h).origin;
                at.entry(v).or_insert_with(|| l.at(body.point(v)));
            }
        }
    }
    let in_tree: BTreeSet<Ix> = laid.iter().map(|l| l.1).collect();
    // The piece of edge `h` of face `x`, as its loop runs.
    let piece = |x: Ix, l: &Lay, h: Ix| -> Result<FlatPiece, BrepError> {
        let (from, to) = (at[&he(h).origin], at[&he(body.next(h)).origin]);
        let e = &body.edges()[he(h).edge as usize];
        let curve = body.edge_curve(e).map(|c| if e.half == h { c } else { c.reversed() });
        Ok(match (curve, l) {
            (None, _) | (Some(Curve::Arc(_)), Lay::Bend { .. }) => FlatPiece::Line(from, to),
            (Some(Curve::Arc(arc)), Lay::Plane { .. }) => {
                let n = normal(x, body.point(he(h).origin));
                arc_piece(l.at(arc.center), from, to, arc.axis.dot(n) > 0.0)
            }
            (Some(Curve::Rolled { piece, .. }), Lay::Bend { neutral, axis, d, u, v, .. }) => {
                if piece.dev.neutral != *neutral {
                    return Err(unsupported("a cut was made with another K-factor"));
                }
                match piece.arc {
                    None if piece.trace.is_some() => {
                        // Chords within the deviation.
                        let n = piece.polyline(POLYLINE_DEVIATION);
                        let inner = (1..n).map(|k| l.at(piece.dev.point(piece.at(k as f64 / n as f64).0, *neutral)));
                        FlatPiece::Polyline(std::iter::once(from).chain(inner).chain([to]).collect())
                    }
                    None => FlatPiece::Line(from, to),
                    Some(pa) => {
                        // Lengths of a development are the blank's, so the centre is found on it,
                        // not through space (one beyond the bend would wrap round its cylinder).
                        let o = l.at(piece.dev.point(Vec2::new(0.0, 0.0), *neutral));
                        let su = if piece.dev.axis.dot(*axis) > 0.0 { 1.0 } else { -1.0 };
                        let sv = if piece.dev.axis.dot(*d) > 0.0 { 1.0 } else { -1.0 };
                        let (a, b) = (su * pa.center.x, sv * pa.center.y);
                        let center = Vec2::new(o.x + u.x * a + v.x * b, o.y + u.y * a + v.y * b);
                        let keeps = su * sv * (u.x * v.y - u.y * v.x) > 0.0;
                        arc_piece(center, from, to, pa.ccw == keeps)
                    }
                }
            }
            (Some(Curve::Rolled { .. }), Lay::Plane { .. }) => return Err(unsupported("a rolled edge on a plane face")),
        })
    };
    // The outline: the faces' outer edges not shared along the chain, joined at their vertices; a
    // run of lines along one face of the body (a cap, an end) is one line.
    let mut edges: BTreeMap<Ix, (Ix, FlatPiece, Ix)> = BTreeMap::new();
    let mut holes: Vec<(FlatPiece, TopoId)> = Vec::new();
    // Every piece around the metal with the strip it is of and the ends its loop gives it.
    let mut rims: Vec<(usize, FlatPiece, [Vec2; 2])> = Vec::new();
    for &(node, x, ref l) in &laid {
        for lp in body.face_loops(x) {
            // A hole with a strip going on from it (a flange on its edge) is part of the outline.
            let outline = lp == body.faces()[x as usize].first_loop || body.loop_half_edges(lp).any(|h| in_tree.contains(&face_of(he(h).twin)));
            let mut pieces = Vec::new();
            for h in body.loop_half_edges(lp) {
                let beside = face_of(he(h).twin);
                if in_tree.contains(&beside) {
                    continue;
                }
                let p = piece(x, l, h)?;
                rims.push((node, p.clone(), [at[&he(h).origin], at[&he(body.next(h)).origin]]));
                if outline {
                    edges.insert(he(h).origin, (he(body.next(h)).origin, p, beside));
                } else {
                    pieces.push((p, id(beside)));
                }
            }
            if outline {
                continue;
            }
            let shapes: Vec<FlatPiece> = pieces.iter().map(|p| p.0.clone()).collect();
            match circle(&shapes) {
                Some(c) => holes.push((c, run_name(pieces.iter().map(|p| p.1)))),
                None => holes.extend(pieces),
            }
        }
    }
    let mut outline: Vec<(FlatPiece, TopoId, [Vec2; 2])> = Vec::new();
    // Where each closed cycle of the outline lies in it.
    let mut cycles: Vec<Range<usize>> = Vec::new();
    while let Some((&v0, _)) = edges.iter().next() {
        let mut v = v0;
        let mut cycle: Vec<(FlatPiece, Vec2, Ix)> = Vec::new();
        while let Some((to, p, beside)) = edges.remove(&v) {
            cycle.push((p, at[&v], beside));
            v = to;
        }
        if v != v0 {
            return Err(BrepError::Invalid("unfold: the outline does not close".into()));
        }
        let begin = outline.len();
        joined(body, cycle, &mut outline);
        cycles.push(begin..outline.len());
    }
    // The corner of the outline at the origin.
    let (lo, hi) = bounds(outline.iter().flat_map(|(p, ..)| key_points(p)));
    // The outer cycle holds the others: the one with the largest box.
    let area = |r: &Range<usize>| {
        let (lo, hi) = bounds(outline[r.clone()].iter().flat_map(|(p, ..)| key_points(p)));
        (hi.x - lo.x) * (hi.y - lo.y)
    };
    let outer = cycles.iter().max_by(|p, q| area(p).total_cmp(&area(q))).cloned().unwrap_or(0..0);
    let shift = |p: Vec2| Vec2::new(p.x - lo.x, p.y - lo.y);
    // Each seam between two strips, from its lowest end to its highest (along X when it runs more
    // that way): a bend line. Named by the strips in the order X grows across it, or else from
    // the strip it is laid off, so a name stays when the tree is rooted elsewhere. Those across X
    // first, by X, then the others by Y.
    let first_id = |st: &[Ix]| st.iter().map(|&x| id(x)).min();
    let bend_id = |a: TopoId, b: TopoId| TopoId::derive("flat.bend").id(a).id(b).finish();
    let mut bend_lines: Vec<(bool, f64, [Vec2; 2], TopoId)> = Vec::new();
    for node in &nodes {
        let Some((p, _)) = node.parent else { continue };
        let parent = &nodes[p].faces;
        let seam: Vec<Vec2> = node
            .faces
            .iter()
            .flat_map(|&x| body.face_loops(x))
            .flat_map(|l| body.loop_half_edges(l))
            .filter(|&h| parent.contains(&face_of(he(h).twin)))
            .flat_map(|h| [at[&he(h).origin], at[&he(body.next(h)).origin]])
            .collect();
        let box_ = bounds(seam.iter().copied());
        let across_x = box_.1.y - box_.0.y >= box_.1.x - box_.0.x;
        let key = |q: &&Vec2| if across_x { q.y } else { q.x };
        let (Some(&low), Some(&high)) = (seam.iter().min_by(|a, b| key(a).total_cmp(&key(b))), seam.iter().max_by(|a, b| key(a).total_cmp(&key(b))))
        else {
            continue;
        };
        // Which way the strip goes on from the line, on the blank.
        let on = centroid_laid(&node.faces, &at, body);
        let swap = across_x && on.x < low.x;
        let (Some(first), Some(second)) = (first_id(parent), first_id(&node.faces)) else { continue };
        let name = if swap { bend_id(second, first) } else { bend_id(first, second) };
        bend_lines.push((!across_x, if across_x { low.x } else { low.y }, [shift(low), shift(high)], name));
    }
    bend_lines.sort_by(|a, b| a.0.cmp(&b.0).then(a.1.total_cmp(&b.1)));
    let (bends, bend_ids): (Vec<[Vec2; 2]>, Vec<TopoId>) = bend_lines.into_iter().map(|(_, _, line, name)| (line, name)).unzip();
    let parents: Vec<Option<usize>> = nodes.iter().map(|n| n.parent.map(|p| p.0)).collect();
    let overlaps = overlaps(&parents, &rims).into_iter().map(|b| b.map(shift)).collect();
    let (pieces, ids) = outline.iter().map(|(p, i, _)| (p, i)).chain(holes.iter().map(|(p, i)| (p, i))).map(|(p, i)| (p.shifted(lo), *i)).unzip();
    let ends = outline[outer.clone()].iter().map(|(.., e)| e.map(shift)).collect();
    // The face the blank is laid from, if a plane: a point of it at (X, Y) of the blank is `start`
    // and the offsets from where it lies, so a sketch frame that gives (X, Y) the same is this.
    let home = (nodes[0].faces.contains(&f)).then_some(lays[0]).and_then(|l| match l {
        Lay::Plane { o, start, ex, ey } => {
            let origin = start + ex * (lo.x - o.x) + ey * (lo.y - o.y);
            Some((id(f), [origin, ex, ey, ex.cross(ey)]))
        }
        Lay::Bend { .. } => None,
    });
    Ok(Flat { k_factor: k, overlaps, ..Flat::new(pieces, ids, Vec2::new(hi.x - lo.x, hi.y - lo.y), bends, bend_ids, (outer, ends), home) })
}

/// The middle of the laid vertices of the outlines of `faces`.
fn centroid_laid(faces: &[Ix], at: &BTreeMap<Ix, Vec2>, body: &Body) -> Vec2 {
    let (mut sum, mut n) = (Vec2::new(0.0, 0.0), 0.0);
    for h in faces.iter().flat_map(|&x| body.loop_half_edges(body.faces()[x as usize].first_loop)) {
        let q = at[&body.half_edges()[h as usize].origin];
        sum = Vec2::new(sum.x + q.x, sum.y + q.y);
        n += 1.0;
    }
    Vec2::new(sum.x / n, sum.y / n)
}

/// Where strips lie on one another: a box about each place, in no order but a fixed one. Strips whose
/// boxes are apart are not looked into, so a sheet with thousands of holes pays nothing for them
/// unless the strip holding them meets another. Whether two edges cross is an exact predicate, on
/// chords of arcs within [`POLYLINE_DEVIATION`]; edges that only touch or run along each other leave
/// no metal on metal.
fn overlaps(parents: &[Option<usize>], rims: &[(usize, FlatPiece, [Vec2; 2])]) -> Vec<[Vec2; 2]> {
    let strips = parents.len();
    if strips < 2 {
        return Vec::new();
    }
    // Each strip's box, with what an arc bulges beyond its ends.
    let mut boxes = vec![(Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY)); strips];
    for (i, p, ends) in rims {
        let reach = match *p {
            FlatPiece::Arc { center, radius, .. } | FlatPiece::Circle { center, radius } => {
                vec![Vec2::new(center.x - radius, center.y - radius), Vec2::new(center.x + radius, center.y + radius)]
            }
            FlatPiece::Polyline(ref q) => q.clone(),
            FlatPiece::Line(..) => Vec::new(),
        };
        for q in ends.iter().copied().chain(reach) {
            let b = &mut boxes[*i];
            *b = (Vec2::new(b.0.x.min(q.x), b.0.y.min(q.y)), Vec2::new(b.1.x.max(q.x), b.1.y.max(q.y)));
        }
    }
    let apart = |a: (Vec2, Vec2), b: (Vec2, Vec2)| a.1.x <= b.0.x || b.1.x <= a.0.x || a.1.y <= b.0.y || b.1.y <= a.0.y;
    let mut pairs: Vec<(usize, usize)> = Vec::new();
    for j in 0..strips {
        for i in 0..j {
            // A strip and the next lie on the two sides of their bend line.
            if !apart(boxes[i], boxes[j]) && parents[j] != Some(i) && parents[i] != Some(j) {
                pairs.push((i, j));
            }
        }
    }
    // Each strip's chords once, only if it is in a pair.
    let mut chords: BTreeMap<usize, Vec<[Vec2; 2]>> = pairs.iter().flat_map(|&(i, j)| [(i, Vec::new()), (j, Vec::new())]).collect();
    for (i, p, ends) in rims {
        if let Some(c) = chords.get_mut(i) {
            c.extend(chords_of(p, *ends));
        }
    }
    let mut found: Vec<(Vec2, Vec2)> = Vec::new();
    for (i, j) in pairs {
        // Only what reaches where the two boxes meet can cross.
        let common = (
            Vec2::new(boxes[i].0.x.max(boxes[j].0.x), boxes[i].0.y.max(boxes[j].0.y)),
            Vec2::new(boxes[i].1.x.min(boxes[j].1.x), boxes[i].1.y.min(boxes[j].1.y)),
        );
        let within = |c: &&[Vec2; 2]| {
            let [p, q] = **c;
            p.x.max(q.x) >= common.0.x && p.x.min(q.x) <= common.1.x && p.y.max(q.y) >= common.0.y && p.y.min(q.y) <= common.1.y
        };
        let a: Vec<&[Vec2; 2]> = chords[&i].iter().filter(within).collect();
        let b: Vec<&[Vec2; 2]> = chords[&j].iter().filter(within).collect();
        let mut place: Option<(Vec2, Vec2)> = None;
        for &&[p, q] in &a {
            for &&[r, s] in &b {
                if p.x.max(q.x) < r.x.min(s.x) || r.x.max(s.x) < p.x.min(q.x) || p.y.max(q.y) < r.y.min(s.y) || r.y.max(s.y) < p.y.min(q.y) {
                    continue;
                }
                let opposite = |u: Sign, v: Sign| matches!((u, v), (Sign::Positive, Sign::Negative) | (Sign::Negative, Sign::Positive));
                if opposite(orient2d(p, q, r), orient2d(p, q, s)) && opposite(orient2d(r, s, p), orient2d(r, s, q)) {
                    let cross = |u: Vec2, v: Vec2, w: Vec2| (v.x - u.x) * (w.y - u.y) - (v.y - u.y) * (w.x - u.x);
                    let (d0, d1) = (cross(r, s, p), cross(r, s, q));
                    let t = d0 / (d0 - d1);
                    let c = Vec2::new(p.x + (q.x - p.x) * t, p.y + (q.y - p.y) * t);
                    place = Some(place.map_or((c, c), |(l, h)| (Vec2::new(l.x.min(c.x), l.y.min(c.y)), Vec2::new(h.x.max(c.x), h.y.max(c.y)))));
                }
            }
        }
        found.extend(place);
    }
    // Boxes sharing metal are one place: a wall and its bend lying on the same neighbour.
    let mut merged: Vec<(Vec2, Vec2)> = Vec::new();
    for mut b in found {
        while let Some(k) = merged.iter().position(|m| m.0.x <= b.1.x && b.0.x <= m.1.x && m.0.y <= b.1.y && b.0.y <= m.1.y) {
            let m = merged.swap_remove(k);
            b = (Vec2::new(m.0.x.min(b.0.x), m.0.y.min(b.0.y)), Vec2::new(m.1.x.max(b.1.x), m.1.y.max(b.1.y)));
        }
        merged.push(b);
    }
    merged.sort_by(|a, b| a.0.x.total_cmp(&b.0.x).then(a.0.y.total_cmp(&b.0.y)));
    merged.into_iter().map(|b| [b.0, b.1]).collect()
}

/// A piece as straight segments, its ends the very vertices of its loop, so a neighbour's segment
/// shares them bit for bit and does not cross it there.
fn chords_of(p: &FlatPiece, [from, to]: [Vec2; 2]) -> Vec<[Vec2; 2]> {
    let inner: Vec<Vec2> = match *p {
        FlatPiece::Line(..) => Vec::new(),
        FlatPiece::Polyline(ref q) => q[1..q.len() - 1].to_vec(),
        FlatPiece::Circle { center, radius } => {
            return chords_of(&FlatPiece::Arc { center, radius, start: 0.0, end: TAU }, [Vec2::new(center.x + radius, center.y); 2]);
        }
        FlatPiece::Arc { center, radius, start, end } => {
            let sweep = (end - start).rem_euclid(TAU);
            let sweep = if sweep == 0.0 { TAU } else { sweep };
            // A chord over an angle `a` strays `r(1 − cos(a/2))`, about `r a²/8`.
            let n = (sweep / libm::sqrt(8.0 * POLYLINE_DEVIATION / radius)).ceil().max(2.0) as usize;
            let on = |k: usize| {
                let (sn, cs) = libm::sincos(start + sweep * k as f64 / n as f64);
                Vec2::new(center.x + radius * cs, center.y + radius * sn)
            };
            let mut q: Vec<Vec2> = (1..n).map(on).collect();
            // Counterclockwise from `start`: reversed if the loop runs the other way.
            let far = |a: Vec2, b: Vec2| (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
            if far(on(0), from) > far(on(0), to) {
                q.reverse();
            }
            q
        }
    };
    let points: Vec<Vec2> = std::iter::once(from).chain(inner).chain([to]).collect();
    points.windows(2).map(|w| [w[0], w[1]]).collect()
}

/// The points that fix a piece's extent in its box: its ends, or the centre of an arc or circle.
fn key_points(p: &FlatPiece) -> Vec<Vec2> {
    match p {
        FlatPiece::Line(a, b) => vec![*a, *b],
        FlatPiece::Arc { center, .. } | FlatPiece::Circle { center, .. } => vec![*center],
        FlatPiece::Polyline(pts) => pts.clone(),
    }
}

/// The lowest and highest corners of the box around `points`.
fn bounds(points: impl Iterator<Item = Vec2>) -> (Vec2, Vec2) {
    points.fold((Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY)), |(lo, hi), p| {
        (Vec2::new(lo.x.min(p.x), lo.y.min(p.y)), Vec2::new(hi.x.max(p.x), hi.y.max(p.y)))
    })
}

/// The flat blank of a rolled cone (D207), face `f` its inner or outer face: an annular sector,
/// arcs about where the apex of its neutral layer lies, at `k` of the thickness from the inside.
/// A point of the cone lies on the blank by its distance from that apex along the cone (its ends
/// are square to the cone, so a whole end lies at one distance) and by its angle about the axis,
/// shrunk by the cone's sine. The sector hangs below its apex, seen from the face's side.
fn cone_blank(body: &Body, f: Ix, k: f64) -> Result<Flat, BrepError> {
    let unsupported = |m: &str| BrepError::Unsupported(format!("unfold: {m}"));
    let he = |h: Ix| body.half_edges()[h as usize];
    let face_of = |h: Ix| body.loops()[he(h).loop_ix as usize].face;
    let face = &body.faces()[f as usize];
    if face.loop_count > 1 {
        return Err(unsupported("a cut on a cone comes later"));
    }
    let Surface::Cone(c) = face.surface else { return Err(BrepError::Invalid("unfold: not a cone".into())) };
    // Across the sheet: the other cone face sharing no edge with this one (the ends share arcs).
    let near: BTreeSet<Ix> = body.loop_half_edges(face.first_loop).map(|h| face_of(he(h).twin)).collect();
    let other = (0..body.faces().len() as Ix).find_map(|y| match body.faces()[y as usize].surface {
        Surface::Cone(o) if y != f && !near.contains(&y) && o.convex != c.convex => Some(o),
        _ => None,
    });
    let other = other.ok_or_else(|| unsupported("a cone without the face across the sheet"))?;
    let (inner, outer) = if c.convex { (other, c) } else { (c, other) };
    let axis = unit(inner.axis);
    let slope = inner.slope;
    let stretch = libm::sqrt(1.0 + slope * slope);
    // In a half plane through the axis, `z` along it from the inner cone's origin and `r` out.
    let meridian = |p: Vec3| {
        let w = p - inner.origin;
        let z = w.dot(axis);
        let across = w - axis * z;
        (z, across.dot(across).sqrt())
    };
    let t = (outer.radius + outer.slope * (inner.origin - outer.origin).dot(axis) - inner.radius).abs() / stretch;
    // The neutral layer's radius at `z = 0`, and where it reaches the axis.
    let r0 = inner.radius + k * t * stretch;
    let apex = -r0 / slope;
    let rho = |p: Vec3| {
        let (z, r) = meridian(p);
        ((z - apex) + slope * r).abs() / stretch
    };
    let sine = slope.abs() / stretch;
    // Angles about the axis along the outline: arcs turn, lines (along the cone, across its ends'
    // thickness at a seam) do not.
    let loop_edges: Vec<Ix> = body.loop_half_edges(face.first_loop).collect();
    let mut theta = Vec::with_capacity(loop_edges.len() + 1);
    let mut at = 0.0;
    for &h in &loop_edges {
        theta.push(at);
        let e = &body.edges()[he(h).edge as usize];
        if let Some(Curve::Arc(arc)) = body.edge_curve(e).map(|cv| if e.half == h { cv } else { cv.reversed() }) {
            let (u, v) = (body.point(he(h).origin), body.point(he(body.next(h)).origin));
            at += arc.sweep(u, v) * if arc.axis.dot(axis) > 0.0 { 1.0 } else { -1.0 };
        }
    }
    let (lo, hi) = theta.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), &a| (lo.min(a), hi.max(a)));
    // Seen from the face's side the blank is not mirrored: X and Y turn as the face's angle and
    // its distance from the apex do, seen from where its normal points.
    let p0 = body.point(he(loop_edges[0]).origin);
    let radial = {
        let w = p0 - inner.origin;
        unit(w - axis * w.dot(axis))
    };
    let (z0, r0p) = meridian(p0);
    let out = (axis + radial * slope) * if (z0 - apex) + slope * r0p > 0.0 { 1.0 } else { -1.0 };
    let sigma = if axis.cross(radial).cross(out).dot(face.surface.normal_at(p0)) > 0.0 { -1.0 } else { 1.0 };
    let psi0 = -std::f64::consts::FRAC_PI_2 - sigma * 0.5 * (lo + hi) * sine;
    let place = |p: Vec3, th: f64| {
        let (s, cs) = libm::sincos(psi0 + sigma * th * sine);
        let d = rho(p);
        Vec2::new(d * cs, d * s)
    };
    let o = Vec2::new(0.0, 0.0);
    let n = loop_edges.len();
    let mut pieces = Vec::with_capacity(n);
    let ids = loop_edges.iter().map(|&h| body.faces()[face_of(he(h).twin) as usize].id).collect();
    for (i, &h) in loop_edges.iter().enumerate() {
        let (a, b) =
            (place(body.point(he(h).origin), theta[i]), place(body.point(he(body.next(h)).origin), theta.get(i + 1).copied().unwrap_or(0.0)));
        let turn = theta.get(i + 1).copied().unwrap_or(0.0) - theta[i];
        pieces.push(if turn == 0.0 { FlatPiece::Line(a, b) } else { arc_piece(o, a, b, sigma * turn > 0.0) });
    }
    // The box: the corners, and the arcs' points farthest along X and Y.
    let (mut lo, mut hi) = (Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY));
    let mut grow = |p: Vec2| {
        (lo, hi) = (Vec2::new(lo.x.min(p.x), lo.y.min(p.y)), Vec2::new(hi.x.max(p.x), hi.y.max(p.y)));
    };
    for p in &pieces {
        match *p {
            FlatPiece::Line(a, b) => [a, b].into_iter().for_each(&mut grow),
            FlatPiece::Polyline(ref pts) => pts.iter().copied().for_each(&mut grow),
            FlatPiece::Arc { center, radius, start, end } => {
                let span = (end - start).rem_euclid(TAU);
                let extremes = (0..4).map(|q| f64::from(q) * std::f64::consts::FRAC_PI_2).filter(|&a| (a - start).rem_euclid(TAU) <= span);
                for a in [start, end].into_iter().chain(extremes) {
                    let (sn, cs) = libm::sincos(a);
                    grow(Vec2::new(center.x + radius * cs, center.y + radius * sn));
                }
            }
            FlatPiece::Circle { .. } => {}
        }
    }
    let pieces: Vec<FlatPiece> = pieces.iter().map(|p| p.shifted(lo)).collect();
    let outer = (0..pieces.len(), Vec::new());
    Ok(Flat { k_factor: k, ..Flat::new(pieces, ids, Vec2::new(hi.x - lo.x, hi.y - lo.y), Vec::new(), Vec::new(), outer, None) })
}

impl Flat {
    /// A blank of `pieces` named `ids`, its `size` and bend lines, the `outer` pieces its outline
    /// with their ends, and `home`, the face it is laid from with the sketch frame on it; hashed by what the DXF
    /// holds.
    fn new(
        pieces: Vec<FlatPiece>,
        ids: Vec<TopoId>,
        size: Vec2,
        bends: Vec<[Vec2; 2]>,
        bend_ids: Vec<TopoId>,
        (outer, ends): (Range<usize>, Vec<[Vec2; 2]>),
        home: Option<(TopoId, [Vec3; 4])>,
    ) -> Self {
        let mut bytes = Vec::with_capacity(pieces.len() * 41);
        for p in &pieces {
            let (tag, v): (u8, [f64; 5]) = match *p {
                FlatPiece::Line(a, b) => (0, [a.x, a.y, b.x, b.y, 0.0]),
                FlatPiece::Arc { center, radius, start, end } => (1, [center.x, center.y, radius, start, end]),
                FlatPiece::Circle { center, radius } => (2, [center.x, center.y, radius, 0.0, 0.0]),
                FlatPiece::Polyline(ref pts) => {
                    bytes.push(3);
                    bytes.extend((pts.len() as u64).to_le_bytes());
                    bytes.extend(pts.iter().flat_map(|q| [q.x, q.y]).flat_map(|x| x.to_bits().to_le_bytes()));
                    continue;
                }
            };
            bytes.push(tag);
            bytes.extend(v.iter().flat_map(|x| x.to_bits().to_le_bytes()));
        }
        debug_assert!(unique(&ids) && unique(&bend_ids), "two lines of a blank with one name");
        Self {
            pieces,
            ids,
            size,
            bends,
            bend_ids,
            outer,
            ends,
            frame: home.map(|h| h.1),
            home: home.map(|h| h.0),
            k_factor: 0.5,
            overlaps: Vec::new(),
            hash: *blake3::hash(&bytes).as_bytes(),
        }
    }
}

/// The pieces of a closed run of the outline, each with where it starts and the face beside it,
/// as the blank has them: a run of lines along one face of the body (a cap, an end) is one line,
/// and so are the pieces of one curve of a cut across bend lines (see [`next_piece`]); such a
/// curve's arcs all round are one circle. Each is named by [`run_name`] of the faces beside it.
fn joined(body: &Body, mut cycle: Vec<(FlatPiece, Vec2, Ix)>, out: &mut Vec<(FlatPiece, TopoId, [Vec2; 2])>) {
    let id = |x: Ix| body.faces()[x as usize].id;
    let n = cycle.len();
    // Either way round: the sketch's side runs its outline against the profile.
    let linked = |a: Ix, b: Ix| next_piece(id(a)) == id(b) || next_piece(id(b)) == id(a);
    // A curve across bend lines starts the run, not a piece of it.
    if let Some(first) = (0..n).find(|&i| linked(cycle[(i + n - 1) % n].2, cycle[i].2) && !linked(cycle[(i + n - 2) % n].2, cycle[(i + n - 1) % n].2))
    {
        cycle.rotate_left((first + n - 1) % n);
    }
    // Whether piece `b` goes on the run piece `a` is in, the run begun beside face `run`.
    let goes_on = |run: Ix, a: Ix, (p, q): (&FlatPiece, &FlatPiece), b: Ix| {
        let line = matches!((p, q), (FlatPiece::Line(..), FlatPiece::Line(..)));
        let arc = matches!((p, q), (FlatPiece::Arc { .. }, FlatPiece::Arc { .. }));
        (line && b == run) || ((line || arc) && linked(a, b))
    };
    // Nor does the run the cycle happens to start in the middle of.
    if goes_on(cycle[n - 1].2, cycle[n - 1].2, (&cycle[n - 1].0, &cycle[0].0), cycle[0].2)
        && let Some(first) = (1..n).find(|&i| !goes_on(cycle[i - 1].2, cycle[i - 1].2, (&cycle[i - 1].0, &cycle[i].0), cycle[i].2))
    {
        cycle.rotate_left(first);
    }
    let mut k = 0;
    while k < n {
        let (p, from, beside) = cycle[k].clone();
        let mut last = k;
        while last + 1 < n && goes_on(beside, cycle[last].2, (&p, &cycle[last + 1].0), cycle[last + 1].2) {
            last += 1;
        }
        let to = cycle.get(last + 1).map_or(cycle[0].1, |e| e.1);
        let name = run_name(cycle[k..=last].iter().map(|e| id(e.2)));
        // An arc starts at `from` if it turns counterclockwise from there (as `arc_piece` laid it).
        let ccw = |center: Vec2, start: f64| libm::atan2(from.y - center.y, from.x - center.x) == start;
        let ends = match p {
            FlatPiece::Arc { center, start, .. } if !ccw(center, start) => [to, from],
            _ => [from, to],
        };
        let piece = match p {
            _ if last == k => p,
            FlatPiece::Line(..) => FlatPiece::Line(from, to),
            FlatPiece::Arc { center, radius, .. } if k == 0 && last == n - 1 => FlatPiece::Circle { center, radius },
            FlatPiece::Arc { center, start, .. } => arc_piece(center, from, to, ccw(center, start)),
            FlatPiece::Circle { .. } | FlatPiece::Polyline(_) => p,
        };
        out.push((piece, name, ends));
        k = last + 1;
    }
}

/// The name of a line of a blank made of pieces beside faces `walls`: a first piece of a curve
/// (one no other piece goes on from, [`next_piece`]), the least such (both halves of a punched
/// circle are first), so that either side, walked either way, names it alike.
fn run_name(walls: impl Iterator<Item = TopoId> + Clone) -> TopoId {
    let first = walls.clone().filter(|w| !walls.clone().any(|v| next_piece(v) == *w)).min();
    first.or_else(|| walls.min()).unwrap_or_else(|| TopoId::derive("flat.empty").finish())
}

/// Whether no name is given twice.
fn unique(ids: &[TopoId]) -> bool {
    let mut v = ids.to_vec();
    v.sort_unstable();
    v.windows(2).all(|w| w[0] != w[1])
}

/// An arc around `center` from `from` to `to`, turning counterclockwise if `ccw`.
fn arc_piece(center: Vec2, from: Vec2, to: Vec2, ccw: bool) -> FlatPiece {
    let (a, b) = if ccw { (from, to) } else { (to, from) };
    let angle = |p: Vec2| libm::atan2(p.y - center.y, p.x - center.x);
    FlatPiece::Arc { center, radius: libm::hypot(a.x - center.x, a.y - center.y), start: angle(a), end: angle(b) }
}

fn unit(v: Vec3) -> Vec3 {
    v * (1.0 / v.dot(v).sqrt())
}

/// The mean of the vertices of face `x`'s outline.
fn centroid(body: &Body, x: Ix) -> Vec3 {
    let l = body.faces()[x as usize].first_loop;
    let n = body.loops()[l as usize].len as f64;
    body.loop_points(l).fold(Vec3::ZERO, |s, p| s + p) * (1.0 / n)
}

/// The circle a loop of arcs about one centre is: a hole cut by a circle, one punch.
fn circle(pieces: &[FlatPiece]) -> Option<FlatPiece> {
    let FlatPiece::Arc { center, radius, .. } = *pieces.first()? else { return None };
    pieces.iter().all(|p| matches!(*p, FlatPiece::Arc { center: c, .. } if c == center)).then_some(FlatPiece::Circle { center, radius })
}

impl Flat {
    /// How many lines, arcs and circles the blank has.
    pub fn counts(&self) -> [usize; 3] {
        let mut n = [0; 3];
        for p in &self.pieces {
            match p {
                FlatPiece::Line(..) => n[0] += 1,
                FlatPiece::Arc { .. } => n[1] += 1,
                FlatPiece::Circle { .. } => n[2] += 1,
                FlatPiece::Polyline(_) => {}
            }
        }
        n
    }

    /// How many of its pieces are polylines: curves with no exact DXF entity, written only when
    /// the engineer agrees (D147, D211).
    pub fn polylines(&self) -> usize {
        self.pieces.iter().filter(|p| matches!(p, FlatPiece::Polyline(_))).count()
    }

    /// The blank as DXF R2000 (AC1015, D211): lines, arcs, circles and lightweight polylines on
    /// layer 0, millimetres. The sections a DXF R2000 needs besides its entities (tables, blocks,
    /// objects) are a fixed file made by ezdxf (`dxf2000.dxf`); the entities go into its model
    /// space with handles after the file's own, and its handle seed follows them.
    pub fn dxf(&self) -> String {
        const TEMPLATE: &str = include_str!("dxf2000.dxf");
        const SEED: &str = "$HANDSEED\n  5\n32\n";
        const ENTITIES: &str = "ENTITIES\n  0\nENDSEC\n";
        // The template's handle seed, and its model space's block record.
        const FIRST: u32 = 0x32;
        const MODEL_SPACE: &str = "17";
        // Writing to a `String` cannot fail.
        fn put(s: &mut String, code: u32, value: impl std::fmt::Display) {
            let _ = writeln!(s, "{code:>3}\n{value}");
        }
        // Coordinates as shortest round-trip decimals, always with a point.
        fn num(s: &mut String, code: u32, x: f64) {
            let _ = writeln!(s, "{code:>3}\n{x:?}");
        }
        let mut s = String::with_capacity(self.pieces.len() * 160 + 200);
        let mut handle = FIRST;
        for p in &self.pieces {
            let (kind, class) = match p {
                FlatPiece::Line(..) => ("LINE", "AcDbLine"),
                FlatPiece::Arc { .. } | FlatPiece::Circle { .. } => (if matches!(p, FlatPiece::Arc { .. }) { "ARC" } else { "CIRCLE" }, "AcDbCircle"),
                FlatPiece::Polyline(_) => ("LWPOLYLINE", "AcDbPolyline"),
            };
            put(&mut s, 0, kind);
            put(&mut s, 5, format_args!("{handle:X}"));
            put(&mut s, 330, MODEL_SPACE);
            put(&mut s, 100, "AcDbEntity");
            put(&mut s, 8, "0");
            put(&mut s, 100, class);
            handle += 1;
            match p {
                FlatPiece::Line(a, b) => {
                    for (c, v) in [(10, a.x), (20, a.y), (30, 0.0), (11, b.x), (21, b.y), (31, 0.0)] {
                        num(&mut s, c, v);
                    }
                }
                FlatPiece::Circle { center, radius } | FlatPiece::Arc { center, radius, .. } => {
                    for (c, v) in [(10, center.x), (20, center.y), (30, 0.0), (40, *radius)] {
                        num(&mut s, c, v);
                    }
                    if let FlatPiece::Arc { start, end, .. } = p {
                        put(&mut s, 100, "AcDbArc");
                        num(&mut s, 50, start.to_degrees());
                        num(&mut s, 51, end.to_degrees());
                    }
                }
                FlatPiece::Polyline(pts) => {
                    put(&mut s, 90, pts.len());
                    put(&mut s, 70, 0);
                    for q in pts {
                        num(&mut s, 10, q.x);
                        num(&mut s, 20, q.y);
                    }
                }
            }
        }
        TEMPLATE.replacen(SEED, &format!("$HANDSEED\n  5\n{handle:X}\n"), 1).replacen(ENTITIES, &format!("ENTITIES\n{s}  0\nENDSEC\n"), 1)
    }
}
