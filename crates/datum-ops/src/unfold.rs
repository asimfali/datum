//! The flat blank of a rolled sheet (M5d, D69): its outline and every cut through it in the
//! sheet's development, as the turret press cuts them before rolling, and its DXF.
//!
//! Nothing is unrolled here: the outline is the cylinder face's, straight in the development,
//! and the cuts were made in the development and keep their exact segments and arcs. The blank
//! is seen from outside the sheet, X along the turn about the axis, Y along the axis, its corner
//! at the origin; lengths are those of the neutral layer.

use std::fmt::Write;

use datum_brep::{Body, BrepError, Curve, Ix, Surface};
use datum_math::Vec2;

use crate::sketch::{find_face, opposite_cylinder, unrolled};

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
    pub hash: [u8; 32],
}

/// The flat blank of the rolled sheet bounded by cylinder face `face` (either side) of `body`, its
/// neutral layer at `k` of the thickness from the inside. Cuts must have been made with the same
/// development (the same K-factor): their pieces are then exact.
pub fn unfold(body: &Body, face: &str, k: f64) -> Result<Flat, BrepError> {
    let (f, _) = find_face(body, face)?;
    let Surface::Cylinder(c) = body.faces()[f].surface else { return Err(BrepError::Unsupported("unfold: the face is not a cylinder".into())) };
    // Seen from outside: the development's own coordinates. The blank is the inner face's, as
    // the development is: the same whichever face names the sheet.
    let d = crate::Developed { mirrored: false, ..unrolled(body, f, &c, k)? };
    let f = if c.convex { opposite_cylinder(body, f).map_or(f, |(g, _)| g as usize) } else { f };
    let mut loops: Vec<Vec<FlatPiece>> = Vec::new();
    for l in body.face_loops(f as Ix) {
        let mut pieces = Vec::new();
        for h in body.loop_half_edges(l) {
            let he = &body.half_edges()[h as usize];
            let edge = &body.edges()[he.edge as usize];
            let ends = [he.origin, body.half_edges()[body.next(h) as usize].origin].map(|v| d.flat(body.point(v)).0);
            let curve = body.edge_curve(edge).map(|c| if edge.half == h { c } else { c.reversed() });
            pieces.push(match curve {
                // About the axis an arc is straight in the development.
                None | Some(Curve::Arc(_)) => FlatPiece::Line(ends[0], ends[1]),
                Some(Curve::Rolled { piece, .. }) if piece.dev == d.dev => match piece.arc {
                    None => FlatPiece::Line(piece.from, piece.to),
                    Some(a) => {
                        let (from, to) = if a.ccw { (piece.from, piece.to) } else { (piece.to, piece.from) };
                        let angle = |p: Vec2| libm::atan2(p.y - a.center.y, p.x - a.center.x);
                        FlatPiece::Arc {
                            center: a.center,
                            radius: libm::hypot(from.x - a.center.x, from.y - a.center.y),
                            start: angle(from),
                            end: angle(to),
                        }
                    }
                },
                Some(Curve::Rolled { .. }) => return Err(BrepError::Unsupported("unfold: a cut was made with another K-factor".into())),
            });
        }
        loops.push(circle(&pieces).map_or(pieces, |c| vec![c]));
    }
    // The corner of the outline (the first loop) at the origin.
    let corners = loops.first().into_iter().flatten().flat_map(|p| match *p {
        FlatPiece::Line(a, b) => [a, b],
        FlatPiece::Arc { center, .. } | FlatPiece::Circle { center, .. } => [center, center],
    });
    let (lo, hi) = corners.fold((Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY)), |(lo, hi), p| {
        (Vec2::new(lo.x.min(p.x), lo.y.min(p.y)), Vec2::new(hi.x.max(p.x), hi.y.max(p.y)))
    });
    let at = |p: Vec2| Vec2::new(p.x - lo.x, p.y - lo.y);
    let pieces: Vec<FlatPiece> = loops
        .into_iter()
        .flatten()
        .map(|p| match p {
            FlatPiece::Line(a, b) => FlatPiece::Line(at(a), at(b)),
            FlatPiece::Arc { center, radius, start, end } => FlatPiece::Arc { center: at(center), radius, start, end },
            FlatPiece::Circle { center, radius } => FlatPiece::Circle { center: at(center), radius },
        })
        .collect();
    let mut bytes = Vec::with_capacity(pieces.len() * 40);
    for p in &pieces {
        let (tag, v): (u8, [f64; 5]) = match *p {
            FlatPiece::Line(a, b) => (0, [a.x, a.y, b.x, b.y, 0.0]),
            FlatPiece::Arc { center, radius, start, end } => (1, [center.x, center.y, radius, start, end]),
            FlatPiece::Circle { center, radius } => (2, [center.x, center.y, radius, 0.0, 0.0]),
        };
        bytes.push(tag);
        bytes.extend(v.iter().flat_map(|x| x.to_bits().to_le_bytes()));
    }
    Ok(Flat { pieces, size: Vec2::new(hi.x - lo.x, hi.y - lo.y), hash: *blake3::hash(&bytes).as_bytes() })
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
