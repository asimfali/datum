//! The flat blank of sheet metal (M5d, M6; D69, D73): the outline and every cut of a sheet's
//! face laid flat, as the turret press cuts them before bending and rolling, and its DXF.
//!
//! Nothing is unrolled geometrically. The sheet is a chain of plane and cylinder faces on one
//! side, each joined to the next smoothly (a bend line); each is laid flat by itself: a plane
//! as it is, a cylinder by the length of its neutral layer, at the K-factor of the thickness from
//! the inside. Cuts keep their exact segments and arcs. The blank is seen from the side of the
//! face it is named by: X along the sheet (`axis × normal`), Y along the bend lines (the axis),
//! its corner at the origin.

use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::TAU;
use std::fmt::Write;
use std::ops::Range;

use datum_brep::{Body, BrepError, Curve, Ix, ProfileArc, Rolled, Surface, TopoId};
use datum_math::{Vec2, Vec3};

use crate::sketch::{find_face, opposite_cylinder};

/// The radius of the neutral layer of a bend between faces of radii `a` and `b`: `k` of the
/// thickness from the inside.
pub(crate) fn neutral(a: f64, b: f64, k: f64) -> f64 {
    a.min(b) + k * (a - b).abs()
}

/// The centre on a blank of the arc `pa` of rolled piece `piece`, whose development's origin lies
/// at `origin` on the blank, and whether it turns counterclockwise there: the blank's X grows as
/// the development's `u` turns about `along`, its Y along `up`. Lengths of a development are the
/// blank's, so the centre is found on it, not through space (one beyond the bend would wrap round
/// its cylinder); every arc of one development about one centre gets the same.
pub(crate) fn rolled_center(piece: &Rolled, pa: ProfileArc, origin: Vec2, along: Vec3, up: Vec3) -> (Vec2, bool) {
    let su = if piece.dev.axis.dot(along) > 0.0 { 1.0 } else { -1.0 };
    let sv = if piece.dev.axis.dot(up) > 0.0 { 1.0 } else { -1.0 };
    (Vec2::new(origin.x + su * pa.center.x, origin.y + sv * pa.center.y), pa.ccw == (su * sv > 0.0))
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
    /// Of the pieces, what the DXF holds.
    pub hash: [u8; 32],
}

/// A face of the chain as it lies in the blank, from `u0` along X.
#[derive(Clone, Copy)]
struct Laid {
    face: Ix,
    u0: f64,
    lay: Lay,
    /// A flange on the sheet's side (D205): laid across X from its wall, whose `lay` it has.
    branch: Option<Branch>,
}

/// A face of a flange on the sheet's side: its Y from the line it stands on (`y0`), away from the
/// sheet (`sign`), along its bend's neutral layer and then its wall.
#[derive(Clone, Copy)]
struct Branch {
    y0: f64,
    sign: f64,
    part: Part,
}

#[derive(Clone, Copy)]
enum Part {
    /// The bend: a point of its axis, the unit axis it turns about away from the wall, the unit
    /// direction across the axis where it starts, the radius of its neutral layer.
    Bend { origin: Vec3, axis: Vec3, start: Vec3, neutral: f64 },
    /// The wall: a point of its line on the bend, its unit direction away from it, and the bend's
    /// length before it.
    Wall { start: Vec3, dir: Vec3, offset: f64 },
}

#[derive(Clone, Copy)]
enum Lay {
    /// A plane: a point where it starts, and its unit direction along the sheet.
    Plane { start: Vec3, along: Vec3 },
    /// A cylinder: a point of its axis, the unit axis it turns about along the sheet, the unit
    /// direction across the axis where it starts, its angle and the radius of its neutral layer.
    Bend { origin: Vec3, axis: Vec3, start: Vec3, span: f64, neutral: f64 },
}

impl Laid {
    /// Where `p`, a point of the face, lies on the blank, Y of the sheet's own faces being `y`.
    fn at(&self, p: Vec3, y: impl Fn(Vec3) -> f64) -> Vec2 {
        let Some(b) = self.branch else { return Vec2::new(self.x(p), y(p)) };
        let away = match b.part {
            Part::Bend { origin, axis, start, neutral } => {
                let w = p - origin;
                let across = w - axis * w.dot(axis);
                neutral * libm::atan2(axis.dot(start.cross(across)), start.dot(across))
            }
            Part::Wall { start, dir, offset } => offset + (p - start).dot(dir),
        };
        Vec2::new(self.x(p), b.y0 + b.sign * away)
    }

    /// X of `p`, a point of the face (or of its neutral layer).
    fn x(&self, p: Vec3) -> f64 {
        match &self.lay {
            Lay::Plane { start, along } => self.u0 + (p - *start).dot(*along),
            Lay::Bend { origin, axis, start, span, neutral } => {
                let w = p - *origin;
                let across = w - *axis * w.dot(*axis);
                let mut theta = libm::atan2(axis.dot(start.cross(across)), start.dot(across));
                // The turn about the axis is cut opposite the middle of the face.
                if theta < 0.5 * span - std::f64::consts::PI {
                    theta += TAU;
                }
                self.u0 + neutral * theta
            }
        }
    }

    fn length(&self, body: &Body) -> f64 {
        match &self.lay {
            Lay::Plane { start, along } => {
                body.loop_points(body.faces()[self.face as usize].first_loop).map(|p| (p - *start).dot(*along)).fold(0.0, f64::max)
            }
            Lay::Bend { span, neutral, .. } => span * neutral,
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
            let m = (body.point(he(h).origin) + body.point(he(body.next(h)).origin)) * 0.5;
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

/// The faces of flanges on the sheet's sides (D205): bends across the sheet's own bends, each with
/// the wall beyond it, which goes on to nothing. The sheet's own bends are those all others are
/// such flanges of; of two ways to see it, the one with more bends, then the one met first. None
/// for a sheet with bends one way only (or none).
pub(crate) fn branches(body: &Body) -> BTreeSet<Ix> {
    let none = BTreeSet::new();
    let cylinders: Vec<(Ix, Vec3)> = (0..body.faces().len() as Ix)
        .filter_map(|x| match body.faces()[x as usize].surface {
            // A bend goes on into a wall; a cut's round wall only into its other half.
            Surface::Cylinder(c) if smooth(body, x).iter().any(|(y, _)| matches!(body.faces()[*y as usize].surface, Surface::Plane(_))) => {
                Some((x, c.axis))
            }
            _ => None,
        })
        .collect();
    let mut classes: Vec<Vec<Ix>> = Vec::new();
    for &(x, axis) in &cylinders {
        match classes.iter_mut().find(|c| crate::sketch::parallel(cylinders.iter().find(|y| y.0 == c[0]).map_or(axis, |y| y.1), axis)) {
            Some(c) => c.push(x),
            None => classes.push(vec![x]),
        }
    }
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

/// The faces of both sides of sheet `body`, the flanges on its sides left out.
pub(crate) fn side_faces(body: &Body) -> Result<BTreeSet<Ix>, BrepError> {
    let skip = branches(body);
    let mut out = BTreeSet::new();
    for s in sheet_sides(body)? {
        out.extend(chain_through(body, s, &skip)?.into_iter().flatten());
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

fn unfold_face(body: &Body, f: Ix, k: f64) -> Result<Flat, BrepError> {
    let unsupported = |m: &str| BrepError::Unsupported(format!("unfold: {m}"));
    if !(0.0..=1.0).contains(&k) {
        return Err(BrepError::Degenerate(format!("a K-factor lies between 0 and 1, got {k}")));
    }
    let he = |h: Ix| body.half_edges()[h as usize];
    let face_of = |h: Ix| body.loops()[he(h).loop_ix as usize].face;
    let normal = |x: Ix, p: Vec3| body.faces()[x as usize].surface.normal_at(p);
    let smooth = |x: Ix| smooth(body, x);
    let id = |x: Ix| body.faces()[x as usize].id;
    let skip = branches(body);
    let mut chain = chain_through(body, f, &skip)?;
    if chain.iter().flatten().any(|&x| matches!(body.faces()[x as usize].surface, Surface::Cone(_))) {
        return match &chain[..] {
            [one] if one.len() == 1 => cone_blank(body, one[0], k),
            _ => Err(unsupported("a cone joined to other faces of a sheet")),
        };
    }

    // Y runs along the bends' axis; a flat sheet takes the first edge of its face.
    let axis = chain.iter().flatten().find_map(|&x| match body.faces()[x as usize].surface {
        Surface::Cylinder(c) => Some(c.axis),
        _ => None,
    });
    let first = body.faces()[chain[0][0] as usize].first_loop;
    let corner = body.point(he(body.loop_half_edges(first).start).origin);
    // A plate has no axis of bends: Y runs up the face as the sketch drawn on it would see it
    // (world +Z projected onto the face; +Y on a face lying flat), so the blank looks as drawn.
    let upright = |n: Vec3| {
        let up = if n.cross(Vec3::new(0.0, 0.0, 1.0)) == Vec3::ZERO { Vec3::new(0.0, 1.0, 0.0) } else { Vec3::new(0.0, 0.0, 1.0) };
        unit(up - n * (up.dot(n) / n.dot(n)))
    };
    let a = unit(axis.unwrap_or_else(|| upright(normal(chain[0][0], corner))));
    // Where strip `s` meets strip `t`: a face of `s` and its half-edge on the bend line.
    let seam = |s: &[Ix], t: &[Ix]| s.iter().find_map(|&x| smooth(x).into_iter().find(|(y, _)| t.contains(y)).map(|(_, h)| (x, h)));
    // X along `a × n`: the chain runs from the end it points away from.
    if chain.len() > 1
        && let Some((x, h)) = seam(&chain[0], &chain[1])
    {
        let m = (body.point(he(h).origin) + body.point(he(body.next(h)).origin)) * 0.5;
        let centre = centroid(body, x);
        if (m - centre).dot(a.cross(normal(x, m))) < 0.0 {
            chain.reverse();
        }
    }

    // Each strip laid from where the previous one ends, all its faces alike.
    let mut laid: Vec<Laid> = Vec::with_capacity(chain.len());
    let mut u0 = 0.0;
    for (i, st) in chain.iter().enumerate() {
        let x = st[0];
        let outline = |x: Ix| body.faces()[x as usize].first_loop;
        // Where it starts: the bend line with the previous strip, or for the first strip its back end.
        let entry = (i > 0).then(|| seam(st, &chain[i - 1]).map(|(_, h)| body.point(he(h).origin))).flatten();
        let lay = match body.faces()[x as usize].surface {
            Surface::Plane(p) => {
                let along = unit(a.cross(unit(p.normal)));
                let start = entry.unwrap_or_else(|| {
                    st.iter().flat_map(|&x| body.loop_points(outline(x))).min_by(|p, q| p.dot(along).total_cmp(&q.dot(along))).unwrap_or(corner)
                });
                Lay::Plane { start, along }
            }
            Surface::Cylinder(c) => {
                let sigma = if c.convex { 1.0 } else { -1.0 };
                let turn = a * sigma;
                // The face's angle: the turn of an arc of its outline (a cap's edge) from bend
                // line to bend line, from the end where turning about `turn` starts.
                let on_line = |v: Ix, k: Option<usize>| {
                    k.and_then(|k| chain.get(k))
                        .is_some_and(|t| t.iter().any(|&y| body.face_loops(y).flat_map(|l| body.loop_half_edges(l)).any(|g| he(g).origin == v)))
                };
                let arc = st.iter().flat_map(|&x| body.loop_half_edges(outline(x))).find_map(|h| {
                    let e = &body.edges()[he(h).edge as usize];
                    let arc = body.edge_arc(e)?;
                    let arc = if e.half == h { arc } else { arc.reversed() };
                    let (v, w) = (he(h).origin, he(body.next(h)).origin);
                    // Each bend line there is (a rolled sheet alone has none) holds an end of it.
                    let ends = [i.checked_sub(1), Some(i + 1)];
                    let lines = ends.iter().filter(|k| k.and_then(|k| chain.get(k)).is_some()).count();
                    let whole = [v, w].iter().filter(|&&x| ends.iter().any(|&k| on_line(x, k))).count() >= lines;
                    whole.then(|| (arc, body.point(v), body.point(w)))
                });
                // A bend cut across at both its sides (D202), or relieved at both its corners
                // (D229), has no such edge left: its angle is then the turn between its two bend
                // lines.
                let (span, from) = match arc {
                    Some((arc, p, q)) => (arc.sweep(p, q), entry.unwrap_or(if arc.axis.dot(turn) > 0.0 { p } else { q })),
                    None => {
                        let exit = chain.get(i + 1).and_then(|next| seam(st, next)).map(|(_, h)| body.point(he(h).origin));
                        let (Some(from), Some(to)) = (entry, exit) else { return Err(unsupported("a bend without an edge across it")) };
                        let radial = |q: Vec3| {
                            let w = q - c.origin;
                            unit(w - c.axis * w.dot(c.axis))
                        };
                        let (u, v, t) = (radial(from), radial(to), unit(turn));
                        (libm::atan2(t.dot(u.cross(v)), u.dot(v)).abs(), from)
                    }
                };
                let w = from - c.origin;
                let start = unit(w - c.axis * w.dot(c.axis));
                let (_, other) = opposite_cylinder(body, x as usize).ok_or_else(|| unsupported("a bend without the face across the sheet"))?;
                let neutral = neutral(c.radius, other.radius, k);
                Lay::Bend { origin: c.origin, axis: unit(turn), start, span, neutral }
            }
            Surface::Cone(_) => return Err(unsupported("a cone in a chain of the sheet")),
            Surface::Wall(_) => return Err(unsupported("the wall of a cut is not a face of the sheet")),
        };
        let faces: Vec<Laid> = st.iter().map(|&x| Laid { face: x, u0, lay, branch: None }).collect();
        u0 += faces.iter().map(|l| l.length(body)).fold(f64::NEG_INFINITY, f64::max);
        laid.extend(faces);
    }
    let y = |p: Vec3| (p - corner).dot(a);
    // Flanges on the sheet's sides (D205): each from the wall it stands on, across X; the pairs of
    // faces their bend lines lie between.
    let mut seams: Vec<(Ix, Ix)> = Vec::new();
    let walls: Vec<Laid> = laid.iter().filter(|l| matches!(l.lay, Lay::Plane { .. })).copied().collect();
    for w in walls {
        for (b, h0) in smooth(w.face).into_iter().filter(|(b, _)| skip.contains(b)) {
            let Surface::Cylinder(c) = body.faces()[b as usize].surface else { continue };
            let Some((f, h1)) = smooth(b).into_iter().find(|(x, _)| *x != w.face && skip.contains(x)) else { continue };
            let (p0, q0) = (body.point(he(h0).origin), body.point(he(h1).origin));
            let c_axis = unit(c.axis);
            let radial = |q: Vec3| {
                let v = q - c.origin;
                unit(v - c_axis * v.dot(c_axis))
            };
            let (start, end) = (radial(p0), radial(q0));
            let turn = start.cross(end);
            if turn == Vec3::ZERO {
                return Err(unsupported("a flange bent back on itself"));
            }
            let axis = unit(turn);
            let span = libm::atan2(axis.dot(start.cross(end)), start.dot(end));
            let (_, other) = opposite_cylinder(body, b as usize).ok_or_else(|| unsupported("a flange's bend without the face across the sheet"))?;
            let neutral = neutral(c.radius, other.radius, k);
            let far = centroid(body, f) - q0;
            let along = match w.lay {
                Lay::Plane { along, .. } => along,
                Lay::Bend { .. } => continue,
            };
            let dir = unit(far - along * far.dot(along));
            let y0 = y(p0);
            let sign = if y(centroid(body, f)) > y0 { 1.0 } else { -1.0 };
            laid.push(Laid { face: b, branch: Some(Branch { y0, sign, part: Part::Bend { origin: c.origin, axis, start, neutral } }), ..w });
            laid.push(Laid { face: f, branch: Some(Branch { y0, sign, part: Part::Wall { start: q0, dir, offset: neutral * span } }), ..w });
            seams.extend([(w.face, b), (b, f)]);
        }
    }
    // Every vertex where the first face of the chain that has it lays it.
    let mut at: BTreeMap<Ix, Vec2> = BTreeMap::new();
    for l in &laid {
        for lp in body.face_loops(l.face) {
            for h in body.loop_half_edges(lp) {
                let v = he(h).origin;
                at.entry(v).or_insert_with(|| l.at(body.point(v), y));
            }
        }
    }
    let in_chain: BTreeSet<Ix> = laid.iter().map(|l| l.face).collect();
    // The piece of edge `h` of face `l`, as its loop runs.
    let piece = |l: &Laid, h: Ix| -> Result<FlatPiece, BrepError> {
        let (from, to) = (at[&he(h).origin], at[&he(body.next(h)).origin]);
        let e = &body.edges()[he(h).edge as usize];
        let curve = body.edge_curve(e).map(|c| if e.half == h { c } else { c.reversed() });
        let bend = matches!(l.branch, Some(Branch { part: Part::Bend { .. }, .. }));
        Ok(match (curve, &l.lay) {
            (None, _) | (Some(Curve::Arc(_)), Lay::Bend { .. }) => FlatPiece::Line(from, to),
            (Some(Curve::Arc(_)), _) if bend => FlatPiece::Line(from, to),
            (Some(Curve::Rolled { piece, .. }), Lay::Plane { along, .. }) if l.branch.is_some() => {
                let Some(Branch { sign, part: Part::Bend { axis, neutral, .. }, .. }) = l.branch else {
                    return Err(unsupported("a rolled edge on a flange's wall"));
                };
                if piece.dev.neutral != neutral {
                    return Err(unsupported("a cut was made with another K-factor"));
                }
                match (piece.arc, piece.trace) {
                    (_, Some(_)) => return Err(unsupported("a traced cut across a flange's bend on the side comes later")),
                    (None, None) => FlatPiece::Line(from, to),
                    (Some(pa), None) => {
                        // Across X: the development's `u` runs along Y away from the wall, its `v`
                        // along X.
                        let o = l.at(piece.dev.point(Vec2::new(0.0, 0.0), neutral), y);
                        let su = sign * if piece.dev.axis.dot(axis) > 0.0 { 1.0 } else { -1.0 };
                        let sv = if piece.dev.axis.dot(*along) > 0.0 { 1.0 } else { -1.0 };
                        arc_piece(Vec2::new(o.x + sv * pa.center.y, o.y + su * pa.center.x), from, to, pa.ccw == (su * sv < 0.0))
                    }
                }
            }
            (Some(Curve::Arc(arc)), Lay::Plane { .. }) => {
                let n = normal(l.face, body.point(he(h).origin));
                arc_piece(l.at(arc.center, y), from, to, arc.axis.dot(n) > 0.0)
            }
            (Some(Curve::Rolled { piece, .. }), Lay::Bend { neutral, .. }) => {
                if piece.dev.neutral != *neutral {
                    return Err(unsupported("a cut was made with another K-factor"));
                }
                match piece.arc {
                    None if piece.trace.is_some() => {
                        // Chords within the deviation.
                        let n = piece.polyline(POLYLINE_DEVIATION);
                        let inner = (1..n).map(|k| {
                            let q = piece.dev.point(piece.at(k as f64 / n as f64).0, *neutral);
                            Vec2::new(l.x(q), y(q))
                        });
                        FlatPiece::Polyline(std::iter::once(from).chain(inner).chain([to]).collect())
                    }
                    None => FlatPiece::Line(from, to),
                    Some(pa) => {
                        let Lay::Bend { axis, .. } = &l.lay else { unreachable!("matched a bend") };
                        let o = piece.dev.point(Vec2::new(0.0, 0.0), *neutral);
                        let (center, ccw) = rolled_center(&piece, pa, Vec2::new(l.x(o), y(o)), *axis, a);
                        arc_piece(center, from, to, ccw)
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
    for l in &laid {
        let mut loops = body.face_loops(l.face);
        if let Some(outer) = loops.next() {
            for h in body.loop_half_edges(outer) {
                let beside = face_of(he(h).twin);
                if !in_chain.contains(&beside) {
                    edges.insert(he(h).origin, (he(body.next(h)).origin, piece(l, h)?, beside));
                }
            }
        }
        for lp in loops {
            let pieces = body.loop_half_edges(lp).map(|h| Ok((piece(l, h)?, id(face_of(he(h).twin))))).collect::<Result<Vec<_>, BrepError>>()?;
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
    // Each seam between two faces of the chain, from its lowest end to its highest: a bend line.
    let by_y = |p: &&Vec2, q: &&Vec2| p.y.total_cmp(&q.y);
    let first_id = |st: &[Ix]| st.iter().map(|&x| id(x)).min();
    let bend_id = |a: TopoId, b: TopoId| TopoId::derive("flat.bend").id(a).id(b).finish();
    let (bends, bend_ids): (Vec<[Vec2; 2]>, Vec<TopoId>) = chain
        .windows(2)
        .filter_map(|w| {
            let seam: Vec<Vec2> = w[1]
                .iter()
                .flat_map(|&x| body.loop_half_edges(body.faces()[x as usize].first_loop))
                .filter(|&h| w[0].contains(&face_of(he(h).twin)))
                .flat_map(|h| [at[&he(h).origin], at[&he(body.next(h)).origin]])
                .collect();
            Some(([shift(*seam.iter().min_by(by_y)?), shift(*seam.iter().max_by(by_y)?)], bend_id(first_id(&w[0])?, first_id(&w[1])?)))
        })
        .chain(seams.iter().filter_map(|&(x, z)| {
            // A flange's bend line on the side runs along X.
            let by_x = |p: &&Vec2, q: &&Vec2| p.x.total_cmp(&q.x);
            let seam: Vec<Vec2> = body
                .loop_half_edges(body.faces()[z as usize].first_loop)
                .filter(|&h| face_of(he(h).twin) == x)
                .flat_map(|h| [at[&he(h).origin], at[&he(body.next(h)).origin]])
                .collect();
            Some(([shift(*seam.iter().min_by(by_x)?), shift(*seam.iter().max_by(by_x)?)], bend_id(id(x), id(z))))
        }))
        .unzip();
    let (pieces, ids) = outline.iter().map(|(p, i, _)| (p, i)).chain(holes.iter().map(|(p, i)| (p, i))).map(|(p, i)| (p.shifted(lo), *i)).unzip();
    let ends = outline[outer.clone()].iter().map(|(.., e)| e.map(shift)).collect();
    // The face the blank is laid from, if a plane: X of the blank is `u0 + (p - start)·along - lo.x`
    // and Y is `(p - corner)·a - lo.y`, so a sketch frame that gives (X, Y) the same is this.
    let home = laid.iter().find(|l| l.face == f && l.branch.is_none()).and_then(|l| match l.lay {
        Lay::Plane { start, along } => {
            let origin = start + along * (lo.x - l.u0) + a * (lo.y - (start - corner).dot(a));
            Some((id(f), [origin, along, a, along.cross(a)]))
        }
        Lay::Bend { .. } => None,
    });
    Ok(Flat::new(pieces, ids, Vec2::new(hi.x - lo.x, hi.y - lo.y), bends, bend_ids, (outer, ends), home))
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
    Ok(Flat::new(pieces, ids, Vec2::new(hi.x - lo.x, hi.y - lo.y), Vec::new(), Vec::new(), outer, None))
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
