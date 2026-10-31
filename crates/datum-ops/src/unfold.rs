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
/// `end` (radians); a hole cut by a circle is one.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FlatPiece {
    Line(Vec2, Vec2),
    Arc { center: Vec2, radius: f64, start: f64, end: f64 },
    Circle { center: Vec2, radius: f64 },
}

/// A flat blank: the value of an unfold feature.
#[derive(Clone, Debug, PartialEq)]
pub struct Flat {
    pub pieces: Vec<FlatPiece>,
    /// Width (along X) and height of the blank.
    pub size: Vec2,
    /// The bend lines (both tangent lines of each bend): guides on the blank, not in its DXF.
    pub bends: Vec<[Vec2; 2]>,
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
fn smooth(body: &Body, x: Ix) -> Vec<(Ix, Ix)> {
    let mut seen = BTreeSet::new();
    let he = |h: Ix| body.half_edges()[h as usize];
    let face_of = |h: Ix| body.loops()[he(h).loop_ix as usize].face;
    let normal = |x: Ix, p: Vec3| body.faces()[x as usize].surface.normal_at(p);
    body.loop_half_edges(body.faces()[x as usize].first_loop)
        .filter_map(|h| {
            let y = face_of(he(h).twin);
            let m = (body.point(he(h).origin) + body.point(he(body.next(h)).origin)) * 0.5;
            (y != x && normal(x, m).dot(normal(y, m)) > 0.5).then_some((y, h))
        })
        .filter(|&(y, _)| seen.insert(y))
        .collect()
}

/// The strip of face `x`: the faces of its surface. One face, or the pieces a cut divided it
/// into (D201): they keep its surface bit for bit, so they are found by it, not by nearness.
/// In face order.
fn strip(body: &Body, x: Ix) -> Vec<Ix> {
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
        let wall = if end(a) { a } else if end(b) { b } else { return None };
        Some(bend.iter().chain(wall).copied().collect())
    };
    let mut best: Option<(usize, BTreeSet<Ix>)> = None;
    for (k, main) in classes.iter().enumerate() {
        let others: Option<Vec<Vec<Ix>>> = classes.iter().enumerate().filter(|(j, _)| *j != k).flat_map(|(_, c)| c.iter().map(|&x| leaf(x))).collect();
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
        return Err(unsupported("the face meets more than two faces smoothly: not a sheet"));
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
                _ => return Err(unsupported("a face meets more than two faces smoothly: not a sheet")),
            }
        }
        halves.push(half);
    }
    let mut chain: Vec<Vec<Ix>> = halves.first().map(|h| h.iter().rev().cloned().collect()).unwrap_or_default();
    chain.push(home);
    chain.extend(halves.into_iter().nth(1).into_iter().flatten());
    Ok(chain)
}

/// The faces naming the two sides of the sheet `body` is, the front first: the two longest
/// chains of faces going on smoothly (a plate's two faces; a bent sheet's inner and outer side).
/// The front is the side facing more toward +Z, then +Y, then +X: seen from there the blank has
/// the orientation of a sketch on XY. A solid with no two sides standing out is no sheet.
pub fn sheet_sides(body: &Body) -> Result<[Ix; 2], BrepError> {
    let unsupported = |m: &str| BrepError::Unsupported(format!("unfold: {m}"));
    let plain = |x: Ix| matches!(body.faces()[x as usize].surface, Surface::Plane(_) | Surface::Cylinder(_));
    // Flanges on the sheet's sides are no side of it.
    let skip = branches(body);
    let mut seen: BTreeSet<Ix> = skip.clone();
    // Each chain: its area, its area vector, the face of it with the largest area.
    let mut chains: Vec<(f64, Vec3, Ix)> = Vec::new();
    for f in 0..body.faces().len() as Ix {
        if seen.contains(&f) || !plain(f) {
            continue;
        }
        let Ok(chain) = chain_through(body, f, &skip) else { continue };
        let chain: Vec<Ix> = chain.into_iter().flatten().collect();
        seen.extend(chain.iter().copied());
        if !chain.iter().all(|&x| plain(x)) {
            continue;
        }
        let area: f64 = chain.iter().map(|&x| body.face_area(x)).sum();
        let vector = chain.iter().fold(Vec3::ZERO, |s, &x| s + body.face_area_vector(x));
        let main = chain.iter().copied().max_by(|&a, &b| body.face_area(a).total_cmp(&body.face_area(b))).unwrap_or(f);
        chains.push((area, vector, main));
    }
    chains.sort_by(|a, b| b.0.total_cmp(&a.0));
    let [front, back, rest @ ..] = &chains[..] else { return Err(unsupported("fewer than two sides: not a sheet")) };
    if rest.first().is_some_and(|third| third.0 >= back.0) {
        return Err(unsupported("no two sides stand out: not a sheet"));
    }
    let toward = |v: Vec3| [v.z, v.y, v.x];
    let first = toward(front.1).iter().zip(toward(back.1)).find_map(|(a, b)| (*a != b).then_some(*a > b));
    Ok(if first == Some(false) { [back.2, front.2] } else { [front.2, back.2] })
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
    let skip = branches(body);
    let mut chain = chain_through(body, f, &skip)?;

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
                // The face's angle: the turn of an arc of its outline (a cap's edge), from the
                // end where turning about `turn` starts.
                let arc = st.iter().flat_map(|&x| body.loop_half_edges(outline(x))).find_map(|h| {
                    let e = &body.edges()[he(h).edge as usize];
                    let arc = body.edge_arc(e)?;
                    let arc = if e.half == h { arc } else { arc.reversed() };
                    Some((arc, body.point(he(h).origin), body.point(he(body.next(h)).origin)))
                });
                // A bend cut across at both its sides (D202) has no such edge left: its angle is
                // then the turn between its two bend lines.
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
            (Some(Curve::Rolled { .. }), _) if l.branch.is_some() => return Err(unsupported("a cut across a flange's bend on the side comes later")),
            (Some(Curve::Arc(arc)), Lay::Plane { .. }) => {
                let n = normal(l.face, body.point(he(h).origin));
                arc_piece(l.at(arc.center, y), from, to, arc.axis.dot(n) > 0.0)
            }
            (Some(Curve::Rolled { piece, .. }), Lay::Bend { neutral, .. }) => {
                if piece.dev.neutral != *neutral {
                    return Err(unsupported("a cut was made with another K-factor"));
                }
                match piece.arc {
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
    let mut holes: Vec<FlatPiece> = Vec::new();
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
            let pieces = body.loop_half_edges(lp).map(|h| piece(l, h)).collect::<Result<Vec<_>, _>>()?;
            holes.extend(circle(&pieces).map_or(pieces, |c| vec![c]));
        }
    }
    let mut outline: Vec<FlatPiece> = Vec::new();
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
        joined(body, cycle, &mut outline);
    }
    // The corner of the outline at the origin.
    let corners = outline.iter().flat_map(|p| match *p {
        FlatPiece::Line(a, b) => [a, b],
        FlatPiece::Arc { center, .. } | FlatPiece::Circle { center, .. } => [center, center],
    });
    let (lo, hi) = corners.fold((Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY)), |(lo, hi), p| {
        (Vec2::new(lo.x.min(p.x), lo.y.min(p.y)), Vec2::new(hi.x.max(p.x), hi.y.max(p.y)))
    });
    let shift = |p: Vec2| Vec2::new(p.x - lo.x, p.y - lo.y);
    // Each seam between two faces of the chain, from its lowest end to its highest: a bend line.
    let by_y = |p: &&Vec2, q: &&Vec2| p.y.total_cmp(&q.y);
    let bends: Vec<[Vec2; 2]> = chain
        .windows(2)
        .filter_map(|w| {
            let seam: Vec<Vec2> = w[1]
                .iter()
                .flat_map(|&x| body.loop_half_edges(body.faces()[x as usize].first_loop))
                .filter(|&h| w[0].contains(&face_of(he(h).twin)))
                .flat_map(|h| [at[&he(h).origin], at[&he(body.next(h)).origin]])
                .collect();
            Some([shift(*seam.iter().min_by(by_y)?), shift(*seam.iter().max_by(by_y)?)])
        })
        .chain(seams.iter().filter_map(|&(x, z)| {
            // A flange's bend line on the side runs along X.
            let by_x = |p: &&Vec2, q: &&Vec2| p.x.total_cmp(&q.x);
            let seam: Vec<Vec2> = body
                .loop_half_edges(body.faces()[z as usize].first_loop)
                .filter(|&h| face_of(he(h).twin) == x)
                .flat_map(|h| [at[&he(h).origin], at[&he(body.next(h)).origin]])
                .collect();
            Some([shift(*seam.iter().min_by(by_x)?), shift(*seam.iter().max_by(by_x)?)])
        }))
        .collect();
    let pieces: Vec<FlatPiece> = outline
        .into_iter()
        .chain(holes)
        .map(|p| match p {
            FlatPiece::Line(a, b) => FlatPiece::Line(shift(a), shift(b)),
            FlatPiece::Arc { center, radius, start, end } => FlatPiece::Arc { center: shift(center), radius, start, end },
            FlatPiece::Circle { center, radius } => FlatPiece::Circle { center: shift(center), radius },
        })
        .collect();
    let mut bytes = Vec::with_capacity(pieces.len() * 41);
    for p in &pieces {
        let (tag, v): (u8, [f64; 5]) = match *p {
            FlatPiece::Line(a, b) => (0, [a.x, a.y, b.x, b.y, 0.0]),
            FlatPiece::Arc { center, radius, start, end } => (1, [center.x, center.y, radius, start, end]),
            FlatPiece::Circle { center, radius } => (2, [center.x, center.y, radius, 0.0, 0.0]),
        };
        bytes.push(tag);
        bytes.extend(v.iter().flat_map(|x| x.to_bits().to_le_bytes()));
    }
    Ok(Flat { pieces, size: Vec2::new(hi.x - lo.x, hi.y - lo.y), bends, hash: *blake3::hash(&bytes).as_bytes() })
}

/// The pieces of a closed run of the outline, each with where it starts and the face beside it,
/// as the blank has them: a run of lines along one face of the body (a cap, an end) is one line,
/// and so are the pieces of one curve of a cut across bend lines (see [`next_piece`]); such a
/// curve's arcs all round are one circle.
fn joined(body: &Body, mut cycle: Vec<(FlatPiece, Vec2, Ix)>, out: &mut Vec<FlatPiece>) {
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
    let goes_on = |run: Ix, a: Ix, (p, q): (FlatPiece, FlatPiece), b: Ix| {
        let line = matches!((p, q), (FlatPiece::Line(..), FlatPiece::Line(..)));
        let arc = matches!((p, q), (FlatPiece::Arc { .. }, FlatPiece::Arc { .. }));
        (line && b == run) || ((line || arc) && linked(a, b))
    };
    // Nor does the run the cycle happens to start in the middle of.
    if goes_on(cycle[n - 1].2, cycle[n - 1].2, (cycle[n - 1].0, cycle[0].0), cycle[0].2)
        && let Some(first) = (1..n).find(|&i| !goes_on(cycle[i - 1].2, cycle[i - 1].2, (cycle[i - 1].0, cycle[i].0), cycle[i].2))
    {
        cycle.rotate_left(first);
    }
    let mut k = 0;
    while k < n {
        let (p, from, beside) = cycle[k];
        let mut last = k;
        while last + 1 < n && goes_on(beside, cycle[last].2, (p, cycle[last + 1].0), cycle[last + 1].2) {
            last += 1;
        }
        let to = cycle.get(last + 1).map_or(cycle[0].1, |e| e.1);
        out.push(match p {
            _ if last == k => p,
            FlatPiece::Line(..) => FlatPiece::Line(from, to),
            FlatPiece::Arc { center, radius, .. } if k == 0 && last == n - 1 => FlatPiece::Circle { center, radius },
            FlatPiece::Arc { center, start, .. } => arc_piece(center, from, to, libm::atan2(from.y - center.y, from.x - center.x) == start),
            FlatPiece::Circle { .. } => p,
        });
        k = last + 1;
    }
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
            n[match p {
                FlatPiece::Line(..) => 0,
                FlatPiece::Arc { .. } => 1,
                FlatPiece::Circle { .. } => 2,
            }] += 1;
        }
        n
    }

    /// The blank as DXF R12 (AC1009): lines, arcs and circles on layer 0, millimetres.
    pub fn dxf(&self) -> String {
        // Writing to a `String` cannot fail.
        fn put(s: &mut String, code: u32, value: impl std::fmt::Display) {
            let _ = writeln!(s, "{code}\n{value}");
        }
        // Coordinates as shortest round-trip decimals, always with a point.
        fn num(s: &mut String, code: u32, x: f64) {
            let _ = writeln!(s, "{code}\n{x:?}");
        }
        let mut s = String::with_capacity(self.pieces.len() * 120 + 200);
        for (c, v) in [(0, "SECTION"), (2, "HEADER"), (9, "$ACADVER"), (1, "AC1009"), (0, "ENDSEC"), (0, "SECTION"), (2, "ENTITIES")] {
            put(&mut s, c, v);
        }
        for p in &self.pieces {
            let (kind, values): (&str, &[(u32, f64)]) = match *p {
                FlatPiece::Line(a, b) => ("LINE", &[(10, a.x), (20, a.y), (30, 0.0), (11, b.x), (21, b.y), (31, 0.0)]),
                FlatPiece::Arc { center, radius, start, end } => {
                    ("ARC", &[(10, center.x), (20, center.y), (30, 0.0), (40, radius), (50, start.to_degrees()), (51, end.to_degrees())])
                }
                FlatPiece::Circle { center, radius } => ("CIRCLE", &[(10, center.x), (20, center.y), (30, 0.0), (40, radius)]),
            };
            put(&mut s, 0, kind);
            put(&mut s, 8, "0");
            for &(c, v) in values {
                num(&mut s, c, v);
            }
        }
        for (c, v) in [(0, "ENDSEC"), (0, "EOF")] {
            put(&mut s, c, v);
        }
        s
    }
}
