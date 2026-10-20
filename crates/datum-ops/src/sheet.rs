//! Sheet metal from a sketched path (M6, D73): an open chain of sketch lines given a thickness,
//! bent at every corner with one inner radius, and extruded. The profile is the path offset to
//! both faces of the sheet, the bends concentric arcs, so the body is an extrusion of plane and
//! cylinder faces (M5a); its flat blank comes from `unfold`.

use std::collections::BTreeMap;

use datum_brep::{BrepError, ProfileArc, ProfileLoop, Region, TopoId, extrude};
use datum_math::{Rigid, Sign, Vec2, orient2d};
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

/// The sheet: the dimensions of a sheet-metal feature.
#[derive(Clone, Copy, Debug)]
pub struct Sheet {
    pub thickness: f64,
    /// Inner radius of every bend.
    pub radius: f64,
    pub side: Side,
}

/// The path of `s` (its lines, not construction, an open chain from `start` or else from its end
/// point named first) as sheet metal `sheet`, extruded from `z0` to `z1` along the sketch's
/// normal. Faces are named by the lines and by the corners' points: editing dimensions keeps
/// every ID.
pub fn sheet(feature: TopoId, s: &Placed, start: Option<&str>, sheet: Sheet, z0: f64, z1: f64) -> Result<datum_brep::Body, BrepError> {
    let bad = |m: String| BrepError::Sketch(format!("sheet path: {m}"));
    if s.developed.is_some() {
        return Err(BrepError::Unsupported("sheet metal from a sketch on a cylinder face".into()));
    }
    if s.arcs.values().any(|a| !a.construction) || s.circles.values().any(|c| !c.construction) {
        return Err(BrepError::Unsupported("sheet path: arcs in the path (rolled sections) come later".into()));
    }
    let Sheet { thickness: t, radius: r, side } = sheet;
    if !(t > 0.0 && r > 0.0 && t.is_finite() && r.is_finite()) {
        return Err(BrepError::Degenerate(format!("sheet metal needs a positive thickness and bend radius, got {t} and {r}")));
    }

    // The chain: points in order, and the line between each two.
    let mut at: BTreeMap<&str, Vec<(&str, &str)>> = BTreeMap::new();
    for (name, l) in s.lines.iter().filter(|(_, l)| !l.construction) {
        at.entry(&l.from).or_default().push((name, &l.to));
        at.entry(&l.to).or_default().push((name, &l.from));
    }
    if at.is_empty() {
        return Err(bad("the sketch has no lines".into()));
    }
    if let Some((p, _)) = at.iter().find(|(_, ls)| ls.len() > 2) {
        return Err(bad(format!("it branches at point `{p}`")));
    }
    let ends: Vec<&str> = at.iter().filter(|(_, ls)| ls.len() == 1).map(|(p, _)| *p).collect();
    if ends.is_empty() {
        return Err(BrepError::Unsupported("sheet path: a closed path comes later".into()));
    }
    let first = match start {
        Some(p) if ends.contains(&p) => p,
        Some(p) => return Err(bad(format!("`{p}` is not an end of the path"))),
        None => ends[0],
    };
    let (mut names, mut lines) = (vec![first], Vec::new());
    let mut prev = "";
    while let Some(&(line, next)) = at[names[names.len() - 1]].iter().find(|(l, _)| *l != prev) {
        lines.push(line);
        names.push(next);
        prev = line;
    }
    if lines.len() != at.values().map(Vec::len).sum::<usize>() / 2 {
        return Err(bad("its lines are not one chain".into()));
    }
    let p: Vec<Vec2> = names.iter().map(|n| s.solved.points[*n]).collect();
    let n = p.len() - 1;

    // Offsets of the two faces to the left of the path.
    let (d0, d1) = match side {
        Side::Left => (0.0, t),
        Side::Right => (-t, 0.0),
        Side::Middle => (-0.5 * t, 0.5 * t),
    };
    let dir = |k: usize| unit(sub(p[k + 1], p[k]));
    let left = |u: Vec2| Vec2::new(-u.y, u.x);
    // Every corner: the bend's centre, its turn (left or not), and where it leaves the lines
    // before and after.
    struct Bend {
        center: Vec2,
        ccw: bool,
        enter: Vec2,
        leave: Vec2,
    }
    let mut bends = Vec::with_capacity(n.saturating_sub(1));
    let mut cut = vec![[0.0f64; 2]; n];
    for k in 1..n {
        let turn = orient2d(p[k - 1], p[k], p[k + 1]);
        if turn == Sign::Zero {
            return Err(BrepError::Degenerate(format!("sheet path: lines `{}` and `{}` lie on one line", lines[k - 1], lines[k])));
        }
        let (u1, u2) = (dir(k - 1), dir(k));
        let ccw = turn == Sign::Positive;
        let s = if ccw { 1.0 } else { -1.0 };
        // The path's radius: the inner face of the bend is `r` from the centre.
        let rp = r + if ccw { d1 } else { -d0 };
        let phi = libm::atan2((u1.x * u2.y - u1.y * u2.x).abs(), u1.x * u2.x + u1.y * u2.y);
        let back = rp * libm::tan(0.5 * phi);
        let enter = Vec2::new(p[k].x - u1.x * back, p[k].y - u1.y * back);
        let leave = Vec2::new(p[k].x + u2.x * back, p[k].y + u2.y * back);
        let nl = left(u1);
        bends.push(Bend { center: Vec2::new(enter.x + nl.x * s * rp, enter.y + nl.y * s * rp), ccw, enter, leave });
        cut[k - 1][1] = back;
        cut[k][0] = back;
    }
    for k in 0..n {
        let length = libm::hypot(p[k + 1].x - p[k].x, p[k + 1].y - p[k].y);
        if cut[k][0] + cut[k][1] >= length {
            return Err(BrepError::Degenerate(format!("sheet path: the bends do not fit on line `{}`", lines[k])));
        }
    }

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
    let off = |q: Vec2, u: Vec2, d: f64| {
        let nl = left(u);
        Vec2::new(q.x + nl.x * d, q.y + nl.y * d)
    };
    for (j, d) in [(0, d0), (1, d1)] {
        // Along the path on face 0, back on face 1.
        let order: Vec<usize> = if j == 0 { (0..n).collect() } else { (0..n).rev().collect() };
        for k in order {
            let u = dir(k);
            let (from, to) = (if k == 0 { p[0] } else { bends[k - 1].leave }, if k == n - 1 { p[n] } else { bends[k].enter });
            let (a, b) = if j == 0 { (from, to) } else { (to, from) };
            // A line from `a`: from the path's end (a corner of the profile) or a bend.
            let (corner, name) = if j == 0 { (k == 0, names[k]) } else { (k == n - 1, names[k + 1]) };
            let start_v = if corner { vertex(name, j, 2) } else { vertex(name, j, 1) };
            push(off(a, u, d), start_v, id("sheet.face", lines[k], j), None);
            // Then the bend at its end, unless the line ends the path.
            let bend = if j == 0 { (k < n - 1).then_some(k) } else { (k > 0).then(|| k - 1) };
            if let Some(i) = bend {
                let bd = &bends[i];
                let corner = names[i + 1];
                let q = off(b, u, d);
                let arc = ProfileArc { center: bd.center, ccw: bd.ccw == (j == 0) };
                push(q, vertex(corner, j, 0), id("sheet.bend", corner, j), Some(arc));
            } else {
                // The end of the sheet: across the thickness to the other face.
                let end = if j == 0 { "end" } else { "start" };
                push(off(b, u, d), vertex(if j == 0 { names[n] } else { names[0] }, j, 3), id("sheet.edge", end, 0), None);
            }
        }
    }
    let caps = ["sheet.cap_start", "sheet.cap_end"].map(|tag| TopoId::derive(tag).id(feature).finish());
    let [o, x, y, z] = s.frame;
    Ok(extrude(&[Region { outer: ring, holes: Vec::new(), caps }], z0, z1)?.transformed(&Rigid::frame(o, x, y, z)))
}

fn sub(a: Vec2, b: Vec2) -> Vec2 {
    Vec2::new(a.x - b.x, a.y - b.y)
}

fn unit(v: Vec2) -> Vec2 {
    let l = libm::hypot(v.x, v.y);
    Vec2::new(v.x / l, v.y / l)
}
