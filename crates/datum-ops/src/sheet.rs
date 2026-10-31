//! Sheet metal from a sketched path (M6, D73, D200): an open chain of sketch lines and arcs given a
//! thickness, bent at every corner of two lines with one inner radius, rolled along every arc,
//! and extruded. An arc goes on from its neighbours smoothly where the sketch makes them tangent.
//! The profile is the path offset to both faces of the sheet, bends and rolled sections concentric
//! arcs, so the body is an extrusion of plane and cylinder faces (M5a); its flat blank comes from
//! `unfold`.

use datum_brep::{BrepError, ProfileArc, ProfileLoop, Region, TopoId, extrude};
use datum_math::{Rigid, Sign, Vec2, orient2d};
use datum_sketch::Step;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::sketch::Placed;

/// Where the material lies from the path, looking along it from its start.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Side {
    Left,
    Right,
    /// Half the thickness each way.
    Middle,
}

impl Side {
    /// The next side for a "flip" button: left, right, middle, left…
    pub fn next(self) -> Self {
        match self {
            Self::Left => Self::Right,
            Self::Right => Self::Middle,
            Self::Middle => Self::Left,
        }
    }
}

/// The open chain of the profile curves of `s` that curve `path` lies on, or else its only one,
/// from its end named first.
fn chain<'s>(s: &'s Placed, path: Option<&str>) -> Result<Vec<Step<'s>>, BrepError> {
    let bad = |m: String| BrepError::Sketch(format!("sheet path: {m}"));
    let found = datum_sketch::contours(&s.lines, &s.arcs, &|_, c| !c);
    if let Some(p) = found.branches.first() {
        return Err(bad(format!("it branches at point `{p}`")));
    }
    let on = |c: &[Step], name: &str| c.iter().any(|st| st.curve == name);
    let chain = match path {
        Some(name) => found.paths.iter().find(|c| on(c, name)).ok_or_else(|| {
            if found.loops.iter().any(|c| on(c, name)) || s.circles.contains_key(name) {
                BrepError::Unsupported(format!("sheet path: `{name}` is on a closed contour; a closed path comes later"))
            } else {
                bad(format!("`{name}` is on no chain of the sketch"))
            }
        })?,
        None => match (&found.paths[..], &found.loops[..]) {
            ([], []) => return Err(bad("the sketch has no lines".into())),
            ([], _) => return Err(BrepError::Unsupported("sheet path: a closed path comes later".into())),
            ([one], _) => one,
            _ => return Err(bad("the sketch has several open chains: choose one by a curve of it (`path`)".into())),
        },
    };
    Ok(chain.clone())
}

/// A curve of each open chain of the profile curves of `s`, by its lowest-named end: the paths a
/// sheet may take.
pub fn paths(s: &Placed) -> Vec<String> {
    datum_sketch::contours(&s.lines, &s.arcs, &|_, c| !c).paths.iter().map(|c| c[0].curve.to_string()).collect()
}

/// The two end points of the open chain curve `curve` of `s` lies on (none: it is on none), the
/// one named first first: where a sheet's path may start.
pub fn path_ends(s: &Placed, curve: &str) -> Option<[String; 2]> {
    let c = chain(s, Some(curve)).ok()?;
    Some([c[0].from.to_string(), c[c.len() - 1].to.to_string()])
}

/// The sheet: the dimensions of a sheet-metal feature.
#[derive(Clone, Copy, Debug)]
pub struct Sheet {
    pub thickness: f64,
    /// Inner radius of every bend.
    pub radius: f64,
    pub side: Side,
}

/// The path of `s` (an open chain of its lines and arcs, not construction: the one curve `path` lies on, or
/// else the sketch's only one; closed contours and circles left out, D191) from `start` or else
/// from its end point named first, as sheet metal `sheet`, extruded from `z0` to `z1` along the
/// sketch's normal. Faces are named by the lines and by the corners' points: editing dimensions
/// keeps every ID.
pub fn sheet(
    feature: TopoId,
    s: &Placed,
    path: Option<&str>,
    start: Option<&str>,
    sheet: Sheet,
    z0: f64,
    z1: f64,
) -> Result<datum_brep::Body, BrepError> {
    let bad = |m: String| BrepError::Sketch(format!("sheet path: {m}"));
    if s.developed.is_some() {
        return Err(BrepError::Unsupported("sheet metal from a sketch on a cylinder face".into()));
    }
    let chain = chain(s, path)?;
    let Sheet { thickness: t, radius: r, side } = sheet;
    if !(t > 0.0 && r > 0.0 && t.is_finite() && r.is_finite()) {
        return Err(BrepError::Degenerate(format!("sheet metal needs a positive thickness and bend radius, got {t} and {r}")));
    }

    let chain = match start {
        None => chain,
        Some(p) if p == chain[0].from => chain,
        Some(p) if p == chain[chain.len() - 1].to => datum_sketch::reversed(&chain),
        Some(p) => return Err(bad(format!("`{p}` is not an end of the path"))),
    };
    let names: Vec<&str> = std::iter::once(chain[0].from).chain(chain.iter().map(|st| st.to)).collect();
    let lines: Vec<&str> = chain.iter().map(|st| st.curve).collect();
    let p: Vec<Vec2> = names.iter().map(|n| s.solved.points[*n]).collect();
    let n = p.len() - 1;
    // Each step's arc, if it is one: its centre, and whether the path turns counterclockwise
    // along it.
    let arcs: Vec<Option<(Vec2, bool)>> = chain.iter().map(|st| s.arcs.get(st.curve).map(|a| (s.solved.points[&a.center], st.forward))).collect();

    // Offsets of the two faces to the left of the path.
    let (d0, d1) = match side {
        Side::Left => (0.0, t),
        Side::Right => (-t, 0.0),
        Side::Middle => (-0.5 * t, 0.5 * t),
    };
    let dir = |k: usize| unit(sub(p[k + 1], p[k]));
    let left = |u: Vec2| Vec2::new(-u.y, u.x);
    // The left of the path at point `q` of step `k`: square to a line, along the radius of an arc.
    let normal = |k: usize, q: Vec2| match arcs[k] {
        None => left(dir(k)),
        Some((c, ccw)) => {
            let r = unit(sub(q, c));
            if ccw { Vec2::new(-r.x, -r.y) } else { r }
        }
    };
    // The direction of the path at point `q` of step `k`.
    let heading = |k: usize, q: Vec2| {
        let nl = normal(k, q);
        Vec2::new(nl.y, -nl.x)
    };
    // The point of step `k` nearest `q`: its foot on the line, or on the circle of the arc.
    let foot = |k: usize, q: Vec2| match arcs[k] {
        None => {
            let u = dir(k);
            let t = dot(sub(q, p[k]), u);
            Vec2::new(p[k].x + u.x * t, p[k].y + u.y * t)
        }
        Some((c, _)) => {
            let (w, r) = (unit(sub(q, c)), libm::hypot(p[k].x - c.x, p[k].y - c.y));
            Vec2::new(c.x + w.x * r, c.y + w.y * r)
        }
    };
    for (k, a) in arcs.iter().enumerate() {
        if let Some((c, ccw)) = *a {
            // Both faces of a rolled section stay outside its centre.
            let r = libm::hypot(p[k].x - c.x, p[k].y - c.y);
            if [d0, d1].iter().any(|&d| if ccw { r - d } else { r + d } <= 0.0) {
                return Err(BrepError::Degenerate(format!("sheet path: arc `{}` is too tight for the sheet's thickness", lines[k])));
            }
        }
    }
    // Each corner: smooth where the sketch makes its curves tangent, else a bend of two lines.
    let tangent = |k: usize| {
        let pair = if lines[k - 1] <= lines[k] { [lines[k - 1], lines[k]] } else { [lines[k], lines[k - 1]] };
        s.tangent.iter().any(|[a, b]| a == pair[0] && b == pair[1])
    };
    let smooth: Vec<bool> = (0..=n).map(|k| k > 0 && k < n && tangent(k)).collect();
    // Every bend: its centre, its turn (left or not), and where it leaves the lines before and
    // after.
    struct Bend {
        center: Vec2,
        ccw: bool,
        enter: Vec2,
        leave: Vec2,
    }
    let mut bends: Vec<Option<Bend>> = (0..=n).map(|_| None).collect();
    let mut cut = vec![[0.0f64; 2]; n];
    // Step `k` offset by `e` to the left of the path: a line, or the circle of an arc.
    let offset = |k: usize, e: f64| -> Result<Offset, BrepError> {
        Ok(match arcs[k] {
            None => {
                let nl = left(dir(k));
                Offset::Line(Vec2::new(p[k].x + nl.x * e, p[k].y + nl.y * e), dir(k))
            }
            Some((c, ccw)) => {
                let radius = libm::hypot(p[k].x - c.x, p[k].y - c.y) - if ccw { e } else { -e };
                if radius <= 0.0 {
                    return Err(BrepError::Degenerate(format!("sheet path: the bend does not fit inside arc `{}`", lines[k])));
                }
                Offset::Circle(c, radius)
            }
        })
    };
    for k in (1..n).filter(|&k| !smooth[k]) {
        let next_to_arc = arcs[k - 1].is_some() || arcs[k].is_some();
        // Which way the path turns: from the curves' directions at the corner next to an arc.
        let (turn, back) = if next_to_arc {
            let (a, b) = (heading(k - 1, p[k]), heading(k, p[k]));
            let turn = match a.x * b.y - a.y * b.x {
                c if c > 0.0 => Sign::Positive,
                c if c < 0.0 => Sign::Negative,
                _ => Sign::Zero,
            };
            (turn, dot(a, b) < 0.0)
        } else {
            (orient2d(p[k - 1], p[k], p[k + 1]), false)
        };
        if turn == Sign::Zero {
            let (a, b, at) = (lines[k - 1], lines[k], names[k]);
            return Err(BrepError::Degenerate(match () {
                _ if back => format!("sheet path: `{b}` turns back along `{a}` at `{at}`"),
                _ if next_to_arc => format!("sheet path: `{a}` and `{b}` meet with no corner at `{at}`: make them tangent in the sketch"),
                _ => format!("sheet path: lines `{a}` and `{b}` lie on one line"),
            }));
        }
        let ccw = turn == Sign::Positive;
        let s = if ccw { 1.0 } else { -1.0 };
        // The path's radius: the inner face of the bend is `r` from the centre.
        let rp = r + if ccw { d1 } else { -d0 };
        if next_to_arc {
            // The centre: on both curves offset by the path's radius toward the turn, the crossing
            // nearer the corner; the bend meets each curve square to it from there.
            let (a, b) = (offset(k - 1, s * rp)?, offset(k, s * rp)?);
            let center = a
                .crossings(&b)
                .into_iter()
                .min_by(|x, y| libm::hypot(x.x - p[k].x, x.y - p[k].y).total_cmp(&libm::hypot(y.x - p[k].x, y.y - p[k].y)))
                .ok_or_else(|| BrepError::Degenerate(format!("sheet path: the bend at `{}` does not fit", names[k])))?;
            let (enter, leave) = (foot(k - 1, center), foot(k, center));
            if arcs[k - 1].is_none() {
                cut[k - 1][1] = dot(sub(p[k], enter), dir(k - 1));
            }
            if arcs[k].is_none() {
                cut[k][0] = dot(sub(leave, p[k]), dir(k));
            }
            bends[k] = Some(Bend { center, ccw, enter, leave });
            continue;
        }
        let (u1, u2) = (dir(k - 1), dir(k));
        let phi = libm::atan2((u1.x * u2.y - u1.y * u2.x).abs(), u1.x * u2.x + u1.y * u2.y);
        let back = rp * libm::tan(0.5 * phi);
        let enter = Vec2::new(p[k].x - u1.x * back, p[k].y - u1.y * back);
        let leave = Vec2::new(p[k].x + u2.x * back, p[k].y + u2.y * back);
        let nl = left(u1);
        bends[k] = Some(Bend { center: Vec2::new(enter.x + nl.x * s * rp, enter.y + nl.y * s * rp), ccw, enter, leave });
        cut[k - 1][1] = back;
        cut[k][0] = back;
    }
    for k in 0..n {
        let fits = match arcs[k] {
            None => {
                let length = libm::hypot(p[k + 1].x - p[k].x, p[k + 1].y - p[k].y);
                cut[k][0] >= 0.0 && cut[k][1] >= 0.0 && cut[k][0] + cut[k][1] < length
            }
            // What is left of an arc between its bends: its start before its end, both on it.
            Some((c, ccw)) => {
                let arc = ProfileArc { center: c, ccw };
                let (from, to) = (bends[k].as_ref().map_or(p[k], |b| b.leave), bends[k + 1].as_ref().map_or(p[k + 1], |b| b.enter));
                // Turns are signed by the arc's way: their sizes along it.
                let whole = arc.turn(p[k], p[k + 1]).abs();
                let (a, b) =
                    (if from == p[k] { 0.0 } else { arc.turn(p[k], from).abs() }, if to == p[k + 1] { whole } else { arc.turn(p[k], to).abs() });
                a < b && b <= whole
            }
        };
        if !fits {
            return Err(BrepError::Degenerate(format!("sheet path: the bends do not fit on `{}`", lines[k])));
        }
    }
    // Where step `k` starts and ends on the path, and the left there: at a smooth corner the arc's
    // (the curves share the point; an arc's left is the one its faces are concentric about).
    let at_corner = |k: usize| if arcs[k].is_some() || arcs[k - 1].is_none() { normal(k, p[k]) } else { normal(k - 1, p[k]) };
    let start = |k: usize| match &bends[k] {
        Some(b) => (b.leave, normal(k, b.leave)),
        None if k > 0 => (p[k], at_corner(k)),
        None => (p[0], normal(0, p[0])),
    };
    let end = |k: usize| match &bends[k + 1] {
        Some(b) => (b.enter, normal(k, b.enter)),
        None if k + 1 < n => (p[k + 1], at_corner(k + 1)),
        None => (p[n], normal(k, p[n])),
    };

    // The profile: the face at `d0` along the path, the end, the face at `d1` back, the start.
    let id = |tag: &str, name: &str, side: u64| TopoId::derive(tag).id(feature).tag(name).index(side).finish();
    let vertex = |name: &str, side: u64, k: u64| {
        [0, 1].map(|layer| TopoId::derive("sheet.v").id(feature).index(layer).tag(name).index(side).index(k).finish())
    };
    let mut ring = ProfileLoop { points: Vec::new(), vertices: Vec::new(), sides: Vec::new(), arcs: Vec::new() };
    let mut push = |q: Vec2, v: [TopoId; 2], side: TopoId, arc: Option<ProfileArc>| {
        ring.points.push(q);
        ring.vertices.push(v);
        ring.sides.push(side);
        ring.arcs.push(arc);
    };
    let off = |(q, nl): (Vec2, Vec2), d: f64| Vec2::new(q.x + nl.x * d, q.y + nl.y * d);
    // A step's face: a wall along a line, a rolled section along an arc, turning as the path
    // does along face 0 and back along face 1.
    let face = |k: usize, j: u64| match arcs[k] {
        None => (id("sheet.face", lines[k], j), None),
        Some((center, ccw)) => (id("sheet.roll", lines[k], j), Some(ProfileArc { center, ccw: ccw == (j == 0) })),
    };
    // The vertex where a step's face begins: a corner of the profile at the path's end, the end
    // of a bend, or a smooth corner.
    let begins = |corner: bool, k: usize| {
        if corner {
            2
        } else if bends[k].is_some() {
            1
        } else {
            4
        }
    };
    for k in 0..n {
        let (side, arc) = face(k, 0);
        push(off(start(k), d0), vertex(names[k], 0, begins(k == 0, k)), side, arc);
        if let Some(bd) = &bends[k + 1] {
            let arc = ProfileArc { center: bd.center, ccw: bd.ccw };
            push(off(end(k), d0), vertex(names[k + 1], 0, 0), id("sheet.bend", names[k + 1], 0), Some(arc));
        } else if k == n - 1 {
            // The end of the sheet: across the thickness to the other face.
            push(off(end(k), d0), vertex(names[n], 0, 3), id("sheet.edge", "end", 0), None);
        }
    }
    for k in (0..n).rev() {
        let (side, arc) = face(k, 1);
        push(off(end(k), d1), vertex(names[k + 1], 1, begins(k == n - 1, k + 1)), side, arc);
        if let Some(bd) = &bends[k] {
            let arc = ProfileArc { center: bd.center, ccw: !bd.ccw };
            push(off(start(k), d1), vertex(names[k], 1, 0), id("sheet.bend", names[k], 1), Some(arc));
        } else if k == 0 {
            push(off(start(k), d1), vertex(names[0], 1, 3), id("sheet.edge", "start", 0), None);
        }
    }
    let caps = ["sheet.cap_start", "sheet.cap_end"].map(|tag| TopoId::derive(tag).id(feature).finish());
    let [o, x, y, z] = s.frame;
    Ok(extrude(&[Region { outer: ring, holes: Vec::new(), caps }], z0, z1)?.transformed(&Rigid::frame(o, x, y, z)))
}

/// A curve of a path offset to one side: a line through a point along a unit direction, or a
/// circle about its centre.
enum Offset {
    Line(Vec2, Vec2),
    Circle(Vec2, f64),
}

impl Offset {
    /// Where the two cross: none, one or two points.
    fn crossings(&self, other: &Offset) -> Vec<Vec2> {
        match (self, other) {
            (Self::Line(a, u), Self::Line(b, v)) => {
                let d = u.x * v.y - u.y * v.x;
                if d == 0.0 {
                    return Vec::new();
                }
                let t = (sub(*b, *a).x * v.y - sub(*b, *a).y * v.x) / d;
                vec![Vec2::new(a.x + u.x * t, a.y + u.y * t)]
            }
            (Self::Line(a, u), Self::Circle(c, r)) | (Self::Circle(c, r), Self::Line(a, u)) => {
                let t = dot(sub(*c, *a), *u);
                let f = Vec2::new(a.x + u.x * t, a.y + u.y * t);
                let h2 = r * r - dot(sub(*c, f), sub(*c, f));
                if h2 < 0.0 {
                    return Vec::new();
                }
                let h = h2.sqrt();
                vec![Vec2::new(f.x - u.x * h, f.y - u.y * h), Vec2::new(f.x + u.x * h, f.y + u.y * h)]
            }
            (Self::Circle(c1, r1), Self::Circle(c2, r2)) => {
                let d = libm::hypot(c2.x - c1.x, c2.y - c1.y);
                if d == 0.0 {
                    return Vec::new();
                }
                let a = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d);
                let h2 = r1 * r1 - a * a;
                if h2 < 0.0 {
                    return Vec::new();
                }
                let (u, h) = (unit(sub(*c2, *c1)), h2.sqrt());
                let f = Vec2::new(c1.x + u.x * a, c1.y + u.y * a);
                vec![Vec2::new(f.x - u.y * h, f.y + u.x * h), Vec2::new(f.x + u.y * h, f.y - u.x * h)]
            }
        }
    }
}

fn dot(a: Vec2, b: Vec2) -> f64 {
    a.x * b.x + a.y * b.y
}

fn sub(a: Vec2, b: Vec2) -> Vec2 {
    Vec2::new(a.x - b.x, a.y - b.y)
}

fn unit(v: Vec2) -> Vec2 {
    let l = libm::hypot(v.x, v.y);
    Vec2::new(v.x / l, v.y / l)
}
