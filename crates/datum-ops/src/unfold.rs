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

use datum_brep::{Body, BrepError, Curve, Ix, Surface};
use datum_math::{Vec2, Vec3};

use crate::sketch::{find_face, opposite_cylinder};
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
struct Laid {
    face: Ix,
    u0: f64,
    lay: Lay,
}

enum Lay {
    /// A plane: a point where it starts, and its unit direction along the sheet.
    Plane { start: Vec3, along: Vec3 },
    /// A cylinder: a point of its axis, the unit axis it turns about along the sheet, the unit
    /// direction across the axis where it starts, its angle and the radius of its neutral layer.
    Bend { origin: Vec3, axis: Vec3, start: Vec3, span: f64, neutral: f64 },
}

impl Laid {
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

/// The faces that go on smoothly from `x` across an edge of its outline, each with the half-edge
/// it is across: outward normals agree along the edge (a bend line), where an end or a cap of the
/// sheet turns by a right angle.
fn smooth(body: &Body, x: Ix) -> Vec<(Ix, Ix)> {
    let he = |h: Ix| body.half_edges()[h as usize];
    let face_of = |h: Ix| body.loops()[he(h).loop_ix as usize].face;
    let normal = |x: Ix, p: Vec3| body.faces()[x as usize].surface.normal_at(p);
    body.loop_half_edges(body.faces()[x as usize].first_loop)
        .filter_map(|h| {
            let y = face_of(he(h).twin);
            let m = (body.point(he(h).origin) + body.point(he(body.next(h)).origin)) * 0.5;
            (y != x && normal(x, m).dot(normal(y, m)) > 0.5).then_some((y, h))
        })
        .collect()
}

/// The chain of faces through `f`: on from it each way while one face goes on, in order along the
/// sheet (not yet turned the way it is laid).
fn chain_through(body: &Body, f: Ix) -> Result<Vec<Ix>, BrepError> {
    let unsupported = |m: &str| BrepError::Unsupported(format!("unfold: {m}"));
    let ways = smooth(body, f);
    if ways.len() > 2 {
        return Err(unsupported("the face meets more than two faces smoothly: not a sheet"));
    }
    let mut halves = Vec::new();
    for &(first, _) in &ways {
        let (mut prev, mut cur, mut half) = (f, first, vec![first]);
        loop {
            let on: Vec<Ix> = smooth(body, cur).into_iter().map(|(y, _)| y).filter(|&y| y != prev).collect();
            match on[..] {
                [] => break,
                [y] if y == f || half.contains(&y) => return Err(unsupported("the sheet closes on itself: a closed path comes later")),
                [y] => {
                    half.push(y);
                    (prev, cur) = (cur, y);
                }
                _ => return Err(unsupported("a face meets more than two faces smoothly: not a sheet")),
            }
        }
        halves.push(half);
    }
    let mut chain: Vec<Ix> = halves.first().map(|h| h.iter().rev().copied().collect()).unwrap_or_default();
    chain.push(f);
    chain.extend(halves.get(1).into_iter().flatten());
    Ok(chain)
}

/// The faces naming the two sides of the sheet `body` is, the front first: the two longest
/// chains of faces going on smoothly (a plate's two faces; a bent sheet's inner and outer side).
/// The front is the side facing more toward +Z, then +Y, then +X: seen from there the blank has
/// the orientation of a sketch on XY. A solid with no two sides standing out is no sheet.
pub fn sheet_sides(body: &Body) -> Result<[Ix; 2], BrepError> {
    let unsupported = |m: &str| BrepError::Unsupported(format!("unfold: {m}"));
    let plain = |x: Ix| matches!(body.faces()[x as usize].surface, Surface::Plane(_) | Surface::Cylinder(_));
    let mut seen: BTreeSet<Ix> = BTreeSet::new();
    // Each chain: its area, its area vector, the face of it with the largest area.
    let mut chains: Vec<(f64, Vec3, Ix)> = Vec::new();
    for f in 0..body.faces().len() as Ix {
        if seen.contains(&f) || !plain(f) {
            continue;
        }
        let Ok(chain) = chain_through(body, f) else { continue };
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
    let mut chain = chain_through(body, f)?;

    // Y runs along the bends' axis; a flat sheet takes the first edge of its face.
    let axis = chain.iter().find_map(|&x| match body.faces()[x as usize].surface {
        Surface::Cylinder(c) => Some(c.axis),
        _ => None,
    });
    let first = body.faces()[chain[0] as usize].first_loop;
    let corner = body.point(he(body.loop_half_edges(first).start).origin);
    // A plate has no axis of bends: Y runs up the face as the sketch drawn on it would see it
    // (world +Z projected onto the face; +Y on a face lying flat), so the blank looks as drawn.
    let upright = |n: Vec3| {
        let up = if n.cross(Vec3::new(0.0, 0.0, 1.0)) == Vec3::ZERO { Vec3::new(0.0, 1.0, 0.0) } else { Vec3::new(0.0, 0.0, 1.0) };
        unit(up - n * (up.dot(n) / n.dot(n)))
    };
    let a = unit(axis.unwrap_or_else(|| upright(normal(chain[0], corner))));
    // X along `a × n`: the chain runs from the end it points away from.
    if chain.len() > 1 {
        let seam = smooth(chain[0]).into_iter().find(|&(y, _)| y == chain[1]).map(|(_, h)| h);
        if let Some(h) = seam {
            let m = (body.point(he(h).origin) + body.point(he(body.next(h)).origin)) * 0.5;
            let centre = centroid(body, chain[0]);
            if (m - centre).dot(a.cross(normal(chain[0], m))) < 0.0 {
                chain.reverse();
            }
        }
    }

    // Each face laid from where the previous one ends.
    let mut laid: Vec<Laid> = Vec::with_capacity(chain.len());
    let mut u0 = 0.0;
    for (i, &x) in chain.iter().enumerate() {
        let outline = body.faces()[x as usize].first_loop;
        // Where it starts: the bend line with the previous face, or for the first face its back end.
        let entry = (i > 0).then(|| smooth(x).into_iter().find(|&(y, _)| y == chain[i - 1]).map(|(_, h)| body.point(he(h).origin))).flatten();
        let lay = match body.faces()[x as usize].surface {
            Surface::Plane(p) => {
                let along = unit(a.cross(unit(p.normal)));
                let start = entry.unwrap_or_else(|| body.loop_points(outline).min_by(|p, q| p.dot(along).total_cmp(&q.dot(along))).unwrap_or(corner));
                Lay::Plane { start, along }
            }
            Surface::Cylinder(c) => {
                let sigma = if c.convex { 1.0 } else { -1.0 };
                let turn = a * sigma;
                // The face's angle: the turn of an arc of its outline (a cap's edge), from the
                // end where turning about `turn` starts.
                let arc = body.loop_half_edges(outline).find_map(|h| {
                    let e = &body.edges()[he(h).edge as usize];
                    let arc = body.edge_arc(e)?;
                    let arc = if e.half == h { arc } else { arc.reversed() };
                    Some((arc, body.point(he(h).origin), body.point(he(body.next(h)).origin)))
                });
                let Some((arc, p, q)) = arc else { return Err(unsupported("a bend without an edge across it")) };
                let span = arc.sweep(p, q);
                let from = entry.unwrap_or(if arc.axis.dot(turn) > 0.0 { p } else { q });
                let w = from - c.origin;
                let start = unit(w - c.axis * w.dot(c.axis));
                let (_, other) = opposite_cylinder(body, x as usize).ok_or_else(|| unsupported("a bend without the face across the sheet"))?;
                let neutral = c.radius.min(other.radius) + k * (c.radius - other.radius).abs();
                Lay::Bend { origin: c.origin, axis: unit(turn), start, span, neutral }
            }
            Surface::Wall(_) => return Err(unsupported("the wall of a cut is not a face of the sheet")),
        };
        let l = Laid { face: x, u0, lay };
        u0 += l.length(body);
        laid.push(l);
    }
    let y = |p: Vec3| (p - corner).dot(a);
    // Every vertex where the first face of the chain that has it lays it.
    let mut at: BTreeMap<Ix, Vec2> = BTreeMap::new();
    for l in &laid {
        for lp in body.face_loops(l.face) {
            for h in body.loop_half_edges(lp) {
                let v = he(h).origin;
                at.entry(v).or_insert_with(|| Vec2::new(l.x(body.point(v)), y(body.point(v))));
            }
        }
    }
    let in_chain: BTreeSet<Ix> = chain.iter().copied().collect();
    // The piece of edge `h` of face `l`, as its loop runs.
    let piece = |l: &Laid, h: Ix| -> Result<FlatPiece, BrepError> {
        let (from, to) = (at[&he(h).origin], at[&he(body.next(h)).origin]);
        let e = &body.edges()[he(h).edge as usize];
        let curve = body.edge_curve(e).map(|c| if e.half == h { c } else { c.reversed() });
        Ok(match (curve, &l.lay) {
            (None, _) | (Some(Curve::Arc(_)), Lay::Bend { .. }) => FlatPiece::Line(from, to),
            (Some(Curve::Arc(arc)), Lay::Plane { .. }) => {
                let n = normal(l.face, body.point(he(h).origin));
                arc_piece(Vec2::new(l.x(arc.center), y(arc.center)), from, to, arc.axis.dot(n) > 0.0)
            }
            (Some(Curve::Rolled { piece, .. }), Lay::Bend { neutral, .. }) => {
                if piece.dev.neutral != *neutral {
                    return Err(unsupported("a cut was made with another K-factor"));
                }
                match piece.arc {
                    None => FlatPiece::Line(from, to),
                    Some(pa) => {
                        // The blank's X turns with the development's `u` from the convex side, against it from the other.
                        let c3 = piece.dev.point(pa.center, *neutral);
                        let convex = matches!(body.faces()[l.face as usize].surface, Surface::Cylinder(c) if c.convex);
                        arc_piece(Vec2::new(l.x(c3), y(c3)), from, to, pa.ccw == convex)
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
        let mut run: Option<(Vec2, Vec2, Ix)> = None;
        while let Some((to, p, beside)) = edges.remove(&v) {
            match (p, &mut run) {
                (FlatPiece::Line(_, b), Some((_, end, side))) if *side == beside => *end = b,
                (p, _) => {
                    if let Some((a, b, _)) = run.take() {
                        outline.push(FlatPiece::Line(a, b));
                    }
                    match p {
                        FlatPiece::Line(a, b) => run = Some((a, b, beside)),
                        p => outline.push(p),
                    }
                }
            }
            v = to;
        }
        if let Some((a, b, _)) = run {
            outline.push(FlatPiece::Line(a, b));
        }
        if v != v0 {
            return Err(BrepError::Invalid("unfold: the outline does not close".into()));
        }
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
            let seam: Vec<Vec2> = body
                .loop_half_edges(body.faces()[w[1] as usize].first_loop)
                .filter(|&h| face_of(he(h).twin) == w[0])
                .flat_map(|h| [at[&he(h).origin], at[&he(body.next(h)).origin]])
                .collect();
            Some([shift(*seam.iter().min_by(by_y)?), shift(*seam.iter().max_by(by_y)?)])
        })
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
