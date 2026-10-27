//! Patterns of a sketch (D128, D153): copies of a group's closed profiles on a grid (so many or as
//! many as a region holds), along a path or around a point. The copies are a cell and where each
//! goes: the solver sees only the cell, and the profiles of the copies are made where the
//! sketch's profiles are (`profile_loops`), for a cut and an extrusion alike.

use std::collections::{BTreeMap, BTreeSet};

use datum_brep::{Box2, BrepError, ProfileArc, ProfileLoop, TopoId, overlap, overlapping};
use datum_math::Vec2;
use datum_sketch::{Along, Around, Pattern, Sketch};
use rayon::prelude::*;

use crate::sketch::{Found, Placed, closed_loops, loops};

/// A pattern of a sketch, placed: the curves it copies and each copy, by its place on the grid
/// (along a path or around a point, `(i, 0)`) and where it goes; what the sketch's status reports
/// of it.
#[derive(Clone, Debug, PartialEq)]
pub struct Copies {
    pub name: String,
    /// The profile curves copied: the group's lines, arcs and circles, not construction.
    pub cell: BTreeSet<String>,
    /// The profile curves of its path (not construction): a guide, no profile of the sketch.
    pub guide: BTreeSet<String>,
    pub at: At,
    /// Copy `(0, 0)` is the cell itself (`count`), keeping its IDs; filling, the cell is a sample.
    pub source: bool,
    /// The area one copy opens, and its profiles.
    pub area: f64,
    pub loops: usize,
    /// The area of the region filled.
    pub region: Option<f64>,
}

/// Each copy, by its place on the grid, and where it goes.
type At = Vec<([i64; 2], Place)>;

/// Where a copy goes: point `p` of the cell to `turn·p + by`, `turn` a rotation `(cos, sin)`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Place {
    pub turn: Vec2,
    pub by: Vec2,
}

impl Place {
    /// Moved by `by`, not turned.
    pub fn moved(by: Vec2) -> Self {
        Self { turn: Vec2::new(1.0, 0.0), by }
    }

    pub fn turned(&self) -> bool {
        self.turn != Vec2::new(1.0, 0.0)
    }

    pub fn apply(&self, p: Vec2) -> Vec2 {
        // Not turned, only added to: a grid's copies are where they were before turns (D153).
        if !self.turned() {
            return Vec2::new(p.x + self.by.x, p.y + self.by.y);
        }
        let Vec2 { x: c, y: s } = self.turn;
        Vec2::new(c * p.x - s * p.y + self.by.x, s * p.x + c * p.y + self.by.y)
    }

    /// Turned by `turn` about `about`, then moved to `to`.
    fn about(turn: Vec2, about: Vec2, to: Vec2) -> Self {
        let t = Self { turn, by: Vec2::new(0.0, 0.0) }.apply(about);
        Self { turn, by: Vec2::new(to.x - t.x, to.y - t.y) }
    }
}

/// The patterns of `sketch`, placed in `s` (solved, with no patterns yet): their copies, which
/// neither touch nor hold one another or another profile of the sketch.
pub(crate) fn place_patterns(sketch: &Sketch, s: &Placed) -> Result<Vec<Copies>, BrepError> {
    // A sketch without patterns may be a path, not profiles: nothing to check.
    if sketch.patterns.is_empty() {
        return Ok(Vec::new());
    }
    let curve = |n: &str| s.lines.contains_key(n) || s.arcs.contains_key(n) || s.circles.contains_key(n);
    let members = |n: &str| -> Vec<String> { sketch.groups.get(n).cloned().unwrap_or_else(|| vec![n.to_string()]) };
    let construction = |n: &str| {
        s.lines.get(n).map(|l| l.construction).or_else(|| s.arcs.get(n).map(|a| a.construction)).or_else(|| s.circles.get(n).map(|c| c.construction))
    };
    let profiles = |n: &str| members(n).into_iter().filter(|n| curve(n) && construction(n) == Some(false)).collect::<BTreeSet<String>>();
    let cells: Vec<BTreeSet<String>> = sketch.patterns.values().map(|p| profiles(&p.of)).collect();
    // A path is a guide, as construction geometry is: drawn as profile lines (as in Inventor), it
    // bounds nothing.
    let guides: Vec<BTreeSet<String>> = sketch.patterns.values().map(|p| p.along.as_ref().map(|a| profiles(&a.path)).unwrap_or_default()).collect();
    let copied: BTreeSet<&String> = cells.iter().chain(&guides).flatten().collect();
    let mut out = Vec::with_capacity(sketch.patterns.len());
    for (((name, p), cell), guide) in sketch.patterns.iter().zip(&cells).zip(&guides) {
        let bad = |m: String| BrepError::Sketch(format!("pattern `{name}`: {m}"));
        let base = loops(TopoId::derive("sketch.cell").finish(), s, &|n| cell.contains(n), false).map_err(|e| bad(format!("its cell: {e}")))?;
        if base.is_empty() {
            return Err(bad(format!("`{}` has no closed profile to copy", p.of)));
        }
        let (area, _) = nesting(&base);
        let mut shift = BTreeMap::new();
        for (key, by) in &p.shift {
            let k = parse_copy(key).ok_or_else(|| bad(format!("shift `{key}`: a copy is named `i,j`")))?;
            let [Some(dx), Some(dy)] = by.each_ref().map(|n| n.value().filter(|v| v.is_finite())) else {
                return Err(bad(format!("shift `{key}`: not a number")));
            };
            shift.insert(k, Vec2::new(dx, dy));
        }
        let centre = mid(union(base.iter().map(|l| l.ring.bounds())));
        let (at, source, region) = match (&p.along, &p.around) {
            (Some(a), _) => {
                let (at, source) = along(s, sketch, a, centre, &shift).map_err(bad)?;
                (at, source, None)
            }
            (None, Some(a)) => (around(s, a, centre, &shift).map_err(bad)?, true, None),
            (None, None) => {
                let step = steps(p).map_err(bad)?;
                let grow = match &p.grow {
                    Some(g) => Some(vectors(g, "grow").map_err(bad)?),
                    None => None,
                };
                let (at, source, region) = match (&p.count, &p.fill) {
                    (Some(count), _) => {
                        let n = count.each_ref().map(whole);
                        let [Some(nx), Some(ny)] = n else { return Err(bad("`count` must be whole numbers from 1".into())) };
                        let g = grow.unwrap_or([Vec2::new(0.0, 0.0); 2]);
                        let at = (0..ny)
                            .flat_map(|j| (0..nx).map(move |i| [i, j]))
                            .map(|k| {
                                let [a, b] = [k[0] as f64, k[1] as f64];
                                let mut o = offset(step, a, b);
                                if grow.is_some() {
                                    let d = offset(g, a * (a - 1.0) / 2.0, b * (b - 1.0) / 2.0);
                                    o = Vec2::new(o.x + d.x, o.y + d.y);
                                }
                                (k, Place::moved(o))
                            })
                            .collect();
                        (at, true, None)
                    }
                    (None, Some(fill)) => {
                        if grow.is_some() {
                            return Err(bad("filling, the steps cannot grow".into()));
                        }
                        let region: BTreeSet<String> = members(fill).into_iter().collect();
                        let ring = loops(TopoId::derive("sketch.region").finish(), s, &|n| region.contains(n), true)
                            .map_err(|e| bad(format!("its region: {e}")))?;
                        let [ring] = &ring[..] else { return Err(bad(format!("`{fill}` must bound one region"))) };
                        let others =
                            closed_loops(TopoId::derive("sketch.others").finish(), s, &|n| !copied.contains(&n.to_string()) && !region.contains(n));
                        let clearance =
                            p.clearance.value().filter(|c| c.is_finite() && *c >= 0.0).ok_or_else(|| bad("`clearance` must be 0 or more".into()))?;
                        let at = fill_region(&base, &ring.ring, &others, step, clearance).map_err(bad)?;
                        (at.into_iter().map(|(k, d)| (k, Place::moved(d))).collect(), false, Some(ring.ring.area().abs()))
                    }
                    (None, None) => return Err(bad("give `count` or `fill`".into())),
                };
                let mut at: At = at;
                for (k, d) in &shift {
                    if let Some((_, p)) = at.iter_mut().find(|(c, _)| c == k) {
                        p.by = Vec2::new(p.by.x + d.x, p.by.y + d.y);
                    }
                }
                (at, source, region)
            }
        };
        let at: At = at.into_iter().filter(|(k, _)| !p.skip.contains(k)).collect();
        out.push(Copies { name: name.clone(), cell: cell.clone(), guide: guide.clone(), at, source, area, loops: base.len(), region });
    }
    keep_apart(s, &out)?;
    Ok(out)
}

/// The two steps of a grid.
fn steps(p: &Pattern) -> Result<[Vec2; 2], String> {
    vectors(p.step.as_ref().ok_or("give `step`, `along` or `around`")?, "step")
}

fn vectors(s: &[[datum_math::expr::Num; 2]; 2], what: &str) -> Result<[Vec2; 2], String> {
    let v = |s: &[datum_math::expr::Num; 2]| match s.each_ref().map(|n| n.value().filter(|v| v.is_finite())) {
        [Some(x), Some(y)] => Ok(Vec2::new(x, y)),
        _ => Err(format!("`{what}` must be numbers")),
    };
    Ok([v(&s[0])?, v(&s[1])?])
}

/// A count: a whole number from 1.
fn whole(n: &datum_math::expr::Num) -> Option<i64> {
    n.value().filter(|v| v.fract() == 0.0 && *v >= 1.0 && *v <= 1e6).map(|v| v as i64)
}

fn number(n: &datum_math::expr::Num, what: &str) -> Result<f64, String> {
    n.value().filter(|v| v.is_finite()).ok_or_else(|| format!("`{what}` must be a number"))
}

/// How far copy `i` is from copy 0: `i` steps, each `grow` longer than the one before.
fn reach(step: f64, grow: f64, i: f64) -> f64 {
    step * i + grow * (i * (i - 1.0) / 2.0)
}

/// The step that takes copy `i` to `span`, given `grow`.
fn step_to(span: f64, grow: f64, i: f64) -> f64 {
    (span - grow * (i * (i - 1.0) / 2.0)) / i
}

/// A path's curves, joined end to end: from the first named, the way it runs, on along what
/// meets its end, back along what meets its start.
struct Path {
    pieces: Vec<Piece>,
    /// Where each piece starts along the path, and the path's length last.
    starts: Vec<f64>,
    closed: bool,
}

#[derive(Clone, Copy)]
enum Piece {
    Line {
        a: Vec2,
        b: Vec2,
    },
    /// Around `c`, `r` from it, from angle `from` by `sweep` (negative: clockwise).
    Arc {
        c: Vec2,
        r: f64,
        from: f64,
        sweep: f64,
    },
}

impl Piece {
    fn length(&self) -> f64 {
        match *self {
            Piece::Line { a, b } => libm::hypot(b.x - a.x, b.y - a.y),
            Piece::Arc { r, sweep, .. } => r * sweep.abs(),
        }
    }

    /// The point `t` of the way along (0 to 1), and the unit direction there.
    fn at(&self, t: f64) -> (Vec2, Vec2) {
        match *self {
            Piece::Line { a, b } => {
                let l = self.length();
                (Vec2::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t), Vec2::new((b.x - a.x) / l, (b.y - a.y) / l))
            }
            Piece::Arc { c, r, from, sweep } => {
                let (s, k) = (libm::sin(from + sweep * t), libm::cos(from + sweep * t));
                let w = sweep.signum();
                (Vec2::new(c.x + r * k, c.y + r * s), Vec2::new(-s * w, k * w))
            }
        }
    }

    /// How far along (0 to 1) the point of the piece nearest `p` is.
    fn nearest(&self, p: Vec2) -> f64 {
        match *self {
            Piece::Line { a, b } => {
                let (dx, dy) = (b.x - a.x, b.y - a.y);
                (((p.x - a.x) * dx + (p.y - a.y) * dy) / (dx * dx + dy * dy)).clamp(0.0, 1.0)
            }
            Piece::Arc { c, from, sweep, .. } => {
                // The angle of `p` past the start, the way the arc runs, in [0, 2π).
                let tau = std::f64::consts::TAU;
                let a = (libm::atan2(p.y - c.y, p.x - c.x) - from) * sweep.signum();
                let a = a.rem_euclid(tau);
                if a <= sweep.abs() {
                    a / sweep.abs()
                } else if a - sweep.abs() < tau - a {
                    1.0
                } else {
                    0.0
                }
            }
        }
    }
}

impl Path {
    fn new(s: &Placed, curves: &[String]) -> Result<Path, String> {
        let pt = |n: &str| s.solved.points.get(n).copied().ok_or_else(|| format!("no point `{n}`"));
        if let Some(n) = curves.iter().find(|n| s.circles.contains_key(*n)) {
            return Err(format!("the path takes lines and arcs; round a circle, copy `around` its centre: `{n}`"));
        }
        let found = datum_sketch::contours(&s.lines, &s.arcs, &|n, _| curves.iter().any(|c| c == n));
        if let Some(p) = found.branches.first() {
            return Err(format!("the path branches at `{p}`"));
        }
        let (chain, closed) = match (&found.paths[..], &found.loops[..]) {
            ([], []) => return Err("the path has no line or arc".into()),
            ([path], []) => (path, false),
            ([], [ring]) => (ring, true),
            _ => return Err("the path's lines and arcs are not one chain".into()),
        };
        // The way the first curve named runs, from it if the path closes.
        let first = curves.iter().find_map(|n| chain.iter().find(|st| st.curve == n)).unwrap_or(&chain[0]);
        let mut chain = if first.forward { chain.clone() } else { datum_sketch::reversed(chain) };
        if closed && let Some(k) = chain.iter().position(|st| st.curve == first.curve) {
            chain.rotate_left(k);
        }
        let mut pieces = Vec::with_capacity(chain.len());
        for st in &chain {
            let (a, b) = (pt(st.from)?, pt(st.to)?);
            pieces.push(match s.arcs.get(st.curve) {
                None => Piece::Line { a, b },
                Some(arc) => {
                    let c = pt(&arc.center)?;
                    let (p0, p1) = if st.forward { (a, b) } else { (b, a) };
                    let sweep = datum_brep::ProfileArc { center: c, ccw: true }.turn(p0, p1);
                    Piece::Arc {
                        c,
                        r: libm::hypot(p0.x - c.x, p0.y - c.y),
                        from: libm::atan2(a.y - c.y, a.x - c.x),
                        sweep: if st.forward { sweep } else { -sweep },
                    }
                }
            });
        }
        let mut starts = Vec::with_capacity(pieces.len() + 1);
        let mut l = 0.0;
        for p in &pieces {
            starts.push(l);
            l += p.length();
        }
        starts.push(l);
        Ok(Path { pieces, starts, closed })
    }

    fn length(&self) -> f64 {
        self.starts[self.pieces.len()]
    }

    /// The point `d` along the path and the unit direction there (round a closed path, `d` wraps).
    fn at(&self, d: f64) -> Result<(Vec2, Vec2), String> {
        let l = self.length();
        let d = if self.closed { d.rem_euclid(l) } else { d };
        if !(0.0..=l).contains(&d) {
            return Err(format!("a copy falls {} past the end of the path", if d < 0.0 { -d } else { d - l }));
        }
        let k = self.starts[1..].partition_point(|&s| s < d).min(self.pieces.len() - 1);
        let p = &self.pieces[k];
        let len = p.length();
        Ok(p.at(if len > 0.0 { (d - self.starts[k]) / len } else { 0.0 }))
    }

    /// How far along the path its point nearest `p` is (the first, of equally near).
    fn nearest(&self, p: Vec2) -> f64 {
        let mut best = (f64::INFINITY, 0.0);
        for (k, piece) in self.pieces.iter().enumerate() {
            let t = piece.nearest(p);
            let (q, _) = piece.at(t);
            let d = libm::hypot(q.x - p.x, q.y - p.y);
            if d < best.0 {
                best = (d, self.starts[k] + t * piece.length());
            }
        }
        best.1
    }
}

/// Copies along a path (D153), and whether the cell is copy 0.
fn along(s: &Placed, sketch: &Sketch, a: &Along, centre: Vec2, shift: &BTreeMap<[i64; 2], Vec2>) -> Result<(At, bool), String> {
    let curves = sketch.groups.get(&a.path).cloned().unwrap_or_else(|| vec![a.path.clone()]);
    let path = Path::new(s, &curves).map_err(|e| format!("its path: {e}"))?;
    let l = path.length();
    let grow = number(&a.grow, "grow")?;
    let count = a.count.as_ref().map(|c| whole(c).ok_or("`count` must be a whole number from 1")).transpose()?;
    let step = a.step.as_ref().map(|s| number(s, "step")).transpose()?;
    let s0 = path.nearest(centre);
    // Where copy 0 is along the path, the step, the copies; and whether the cell is copy 0.
    let (first, step, n, source) = match (step, count) {
        (Some(h), Some(n)) => (s0, h, n, true),
        (None, Some(n)) => {
            let (span, i) = if path.closed { (l, n as f64) } else { (l - s0, (n - 1) as f64) };
            (s0, if i > 0.0 { step_to(span, grow, i) } else { 0.0 }, n, true)
        }
        (Some(h), None) => {
            if h == 0.0 {
                return Err("`step` must not be 0".into());
            }
            let h = h.abs();
            let fits = |n: i64| if path.closed { reach(h, grow, n as f64) <= l } else { reach(h, grow, (n - 1) as f64) <= l };
            let mut n = 1;
            while n < 1_000_000 && fits(n + 1) && reach(h, grow, n as f64) > reach(h, grow, (n - 1) as f64) {
                n += 1;
            }
            if path.closed { (s0, h, n, true) } else { ((l - reach(h, grow, (n - 1) as f64)) / 2.0, h, n, false) }
        }
        (None, None) => return Err("along a path, give `step`, `count` or both".into()),
    };
    let (anchor, t0) = path.at(s0)?;
    let mut at = Vec::with_capacity(n as usize);
    for i in 0..n {
        let k = [i, 0];
        let d = shift.get(&k).copied().unwrap_or(Vec2::new(0.0, 0.0));
        let along = first + reach(step, grow, i as f64) + d.x;
        if source && along == s0 && d.y == 0.0 {
            at.push((k, Place::moved(Vec2::new(0.0, 0.0))));
            continue;
        }
        let (p, t) = path.at(along)?;
        let turn = if a.rotate {
            let (c, sn) = (t0.x * t.x + t0.y * t.y, t0.x * t.y - t0.y * t.x);
            let h = libm::hypot(c, sn);
            Vec2::new(c / h, sn / h)
        } else {
            Vec2::new(1.0, 0.0)
        };
        // Across: to the left of the way the path runs.
        let to = Vec2::new(p.x - t.y * d.y, p.y + t.x * d.y);
        at.push((k, Place::about(turn, anchor, to)));
    }
    Ok((at, source))
}

/// Copies around a point (D153).
fn around(s: &Placed, a: &Around, centre: Vec2, shift: &BTreeMap<[i64; 2], Vec2>) -> Result<At, String> {
    let c = s.solved.points.get(&a.center).copied().ok_or_else(|| format!("no point `{}`", a.center))?;
    let n = whole(&a.count).ok_or("`count` must be a whole number from 1")?;
    let grow = number(&a.grow, "grow")?;
    let step = match (&a.step, &a.angle) {
        (Some(h), _) => number(h, "step")?,
        (None, angle) => {
            let angle = angle.as_ref().map(|x| number(x, "angle")).transpose()?.unwrap_or(360.0);
            // Round the whole circle the last copy is a step short of the first; less, it ends the span.
            let i = if angle.abs() == 360.0 { n as f64 } else { (n - 1) as f64 };
            if i > 0.0 { step_to(angle, grow, i) } else { 0.0 }
        }
    };
    let out = Vec2::new(centre.x - c.x, centre.y - c.y);
    let r = libm::hypot(out.x, out.y);
    let mut at = Vec::with_capacity(n as usize);
    for i in 0..n {
        let k = [i, 0];
        let d = shift.get(&k).copied().unwrap_or(Vec2::new(0.0, 0.0));
        let deg = reach(step, grow, i as f64) + d.x;
        let rad = deg.to_radians();
        let turn = if deg == 0.0 { Vec2::new(1.0, 0.0) } else { Vec2::new(libm::cos(rad), libm::sin(rad)) };
        let mut place = Place::about(turn, c, c);
        if d.y != 0.0 {
            if r == 0.0 {
                return Err(format!("shift `{i},0`: the cell's centre is the centre: no way out"));
            }
            let o = Place { turn, by: Vec2::new(0.0, 0.0) }.apply(out);
            place.by = Vec2::new(place.by.x + o.x / r * d.y, place.by.y + o.y / r * d.y);
        }
        at.push((k, place));
    }
    Ok(at)
}

fn offset(step: [Vec2; 2], i: f64, j: f64) -> Vec2 {
    Vec2::new(step[0].x * i + step[1].x * j, step[0].y * i + step[1].y * j)
}

fn parse_copy(key: &str) -> Option<[i64; 2]> {
    let (i, j) = key.split_once(',')?;
    Some([i.trim().parse().ok()?, j.trim().parse().ok()?])
}

/// The area a set of loops opens (a loop inside another a hole, inside that an island), and the
/// depth of each in the others.
pub(crate) fn nesting(loops: &[Found]) -> (f64, Vec<usize>) {
    let n = loops.len();
    let depth: Vec<usize> = (0..n).map(|b| (0..n).filter(|&a| a != b && loops[a].ring.winding(loops[b].ring.points[0]) != 0).count()).collect();
    let area = (0..n).map(|k| loops[k].ring.area().abs() * if depth[k].is_multiple_of(2) { 1.0 } else { -1.0 }).sum();
    (area, depth)
}

/// Where a cell goes filling a region: on the grid of `step`, centred on the region's box, every
/// copy's box (the cell's, `clearance` larger) inside the region and clear of `others`. Of the
/// grid shifted by nothing or half a step along each, the one holding most.
fn fill_region(cell: &[Found], region: &ProfileLoop, others: &[Found], step: [Vec2; 2], clearance: f64) -> Result<Vec<([i64; 2], Vec2)>, String> {
    let cell_box = union(cell.iter().map(|l| l.ring.bounds()));
    let half = Vec2::new(0.5 * (cell_box.1.x - cell_box.0.x) + clearance, 0.5 * (cell_box.1.y - cell_box.0.y) + clearance);
    let cell_mid = mid(cell_box);
    let (lo, hi) = region.bounds();
    let centre = mid((lo, hi));
    // The grid's coordinates of the region's box corners: the copies to try.
    let [a, b] = step;
    let det = a.x * b.y - a.y * b.x;
    if det == 0.0 {
        return Err("filling, the two steps must not lie along one line".into());
    }
    let grid = |p: Vec2| {
        let (x, y) = (p.x - centre.x, p.y - centre.y);
        ((x * b.y - y * b.x) / det, (a.x * y - a.y * x) / det)
    };
    let corners = [lo, Vec2::new(hi.x, lo.y), hi, Vec2::new(lo.x, hi.y)].map(grid);
    let range = |f: fn(&(f64, f64)) -> f64| {
        let v = corners.iter().map(f);
        (v.clone().fold(f64::INFINITY, f64::min).floor() as i64 - 1, v.fold(f64::NEG_INFINITY, f64::max).ceil() as i64 + 1)
    };
    let ((i0, i1), (j0, j1)) = (range(|c| c.0), range(|c| c.1));
    if (i1 - i0 + 1).saturating_mul(j1 - j0 + 1) > 4_000_000 {
        return Err("the region holds more than four million steps: is the step right?".into());
    }
    let boxes: Vec<Box2> = others.iter().map(|o| o.ring.bounds()).collect();
    let fits = |p: Vec2| {
        let r = (Vec2::new(p.x - half.x, p.y - half.y), Vec2::new(p.x + half.x, p.y + half.y));
        let rect = rectangle(r);
        !rect.touches(region)
            && region.winding(r.0) != 0
            && others
                .iter()
                .zip(&boxes)
                .filter(|(_, bx)| overlap(**bx, r))
                .all(|(o, _)| !rect.touches(&o.ring) && o.ring.winding(r.0) == 0 && rect.winding(o.ring.points[0]) == 0)
    };
    let ks: Vec<[i64; 2]> = (j0..=j1).flat_map(|j| (i0..=i1).map(move |i| [i, j])).collect();
    let mut best: Vec<([i64; 2], Vec2)> = Vec::new();
    for (pa, pb) in [(0.0, 0.0), (0.5, 0.0), (0.0, 0.5), (0.5, 0.5)] {
        let found: Vec<([i64; 2], Vec2)> = ks
            .par_iter()
            .filter_map(|&k| {
                let o = offset(step, k[0] as f64 + pa, k[1] as f64 + pb);
                let p = Vec2::new(centre.x + o.x, centre.y + o.y);
                fits(p).then(|| (k, Vec2::new(p.x - cell_mid.x, p.y - cell_mid.y)))
            })
            .collect();
        if found.len() > best.len() {
            best = found;
        }
    }
    Ok(best)
}

/// A box as a loop, for the exact checks.
fn rectangle((lo, hi): Box2) -> ProfileLoop {
    ProfileLoop { points: vec![lo, Vec2::new(hi.x, lo.y), hi, Vec2::new(lo.x, hi.y)], vertices: Vec::new(), sides: Vec::new(), arcs: Vec::new() }
}

fn union(boxes: impl Iterator<Item = Box2>) -> Box2 {
    boxes.fold((Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY)), |(lo, hi), (a, b)| {
        (Vec2::new(lo.x.min(a.x), lo.y.min(a.y)), Vec2::new(hi.x.max(b.x), hi.y.max(b.y)))
    })
}

fn mid((lo, hi): Box2) -> Vec2 {
    Vec2::new(0.5 * (lo.x + hi.x), 0.5 * (lo.y + hi.y))
}

/// Loop `l` moved to `place`, its IDs those of copy `k` of pattern `name` (copy `(0, 0)` of a
/// cell copied from itself keeps its own).
pub(crate) fn moved(l: &ProfileLoop, place: Place, feature: TopoId, name: &str, k: [i64; 2], own: bool) -> ProfileLoop {
    let id =
        |t: TopoId| if own { t } else { TopoId::derive("sketch.copy").id(feature).tag(name).index(k[0] as u64).index(k[1] as u64).id(t).finish() };
    ProfileLoop {
        points: l.points.iter().map(|&p| place.apply(p)).collect(),
        vertices: l.vertices.iter().map(|v| v.map(id)).collect(),
        sides: l.sides.iter().map(|&s| id(s)).collect(),
        arcs: l.arcs.iter().map(|a| a.map(|a| ProfileArc { center: place.apply(a.center), ..a })).collect(),
    }
}

/// The curves that are no profile of the sketch's own: the cells patterns copy, and their paths.
pub(crate) fn aside(patterns: &[Copies]) -> BTreeSet<&String> {
    patterns.iter().flat_map(|p| p.cell.iter().chain(&p.guide)).collect()
}

/// Copies touch nothing and hold nothing: no other copy, no other profile of the sketch.
fn keep_apart(s: &Placed, patterns: &[Copies]) -> Result<(), BrepError> {
    let id = TopoId::derive("sketch.check").finish();
    let aside = aside(patterns);
    let own = closed_loops(id, s, &|n| !aside.contains(&n.to_string()));
    let mut all: Vec<(Option<usize>, ProfileLoop)> = own.into_iter().map(|l| (None, l.ring)).collect();
    for (k, p) in patterns.iter().enumerate() {
        let base = loops(id, s, &|n| p.cell.contains(n), false)?;
        all.par_extend(p.at.par_iter().flat_map_iter(|&(c, d)| base.iter().map(move |l| (Some(k), moved(&l.ring, d, id, &p.name, c, true)))));
    }
    let boxes: Vec<Box2> = all.iter().map(|(_, l)| l.bounds()).collect();
    let clash = overlapping(&boxes).into_par_iter().find_first(|&(a, b)| {
        // A copy may lie inside a profile (a hole in a plate), not the other way round.
        let ((pa, la), (pb, lb)) = (&all[a], &all[b]);
        let holds = |copy: &Option<usize>, l: &ProfileLoop, other: &ProfileLoop| copy.is_some() && l.winding(other.points[0]) != 0;
        (pa.is_some() || pb.is_some()) && (la.touches(lb) || holds(pa, la, lb) || holds(pb, lb, la))
    });
    match clash {
        Some((a, b)) => {
            let name = |k: usize| all[k].0.map_or("a profile".to_string(), |p| format!("a copy of pattern `{}`", patterns[p].name));
            Err(BrepError::Degenerate(format!("{} and {} touch or hold one another", name(a), name(b))))
        }
        None => Ok(()),
    }
}
