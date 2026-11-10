//! Bounded arrays of open chains (D250, D253): copies at a distance from a chain, the line going
//! parallel and the arc about the same centre, and each end of each copy stopped by a curve of
//! its bound: extended along its own line or circle, or cut short. What ribs are rolled along.

use std::collections::BTreeSet;
use std::f64::consts::TAU;

use datum_brep::{BrepError, ProfileArc};
use datum_math::{Carrier, Vec2, crossings};
use datum_sketch::{Offset, Pattern, Sketch, contours};

use crate::copies::{Copies, whole};
use crate::sketch::{ChainPiece, Placed, first_corner, named_pieces, within_rounding};

type Chain = Vec<(String, ChainPiece)>;

/// One chain of one copy. Copy 0 is the cell itself and keeps its curves' names; the curves of
/// the others are named `<pattern>[<k>].<curve>`.
#[derive(Clone, Debug, PartialEq)]
pub struct ChainCopy {
    pub k: i64,
    pub pieces: Chain,
}

impl ChainCopy {
    /// The copy as polyline points in the sketch's coordinates, arcs 64 chords a turn.
    pub fn outline(&self) -> Vec<[f64; 2]> {
        let mut pts = vec![[self.pieces[0].1.from.x, self.pieces[0].1.from.y]];
        for (_, c) in &self.pieces {
            if let Some(a) = c.arc {
                let turn = a.turn(c.from, c.to);
                let n = (turn.abs() / TAU * 64.0).ceil().max(1.0) as usize;
                let (x, y) = (c.from.x - a.center.x, c.from.y - a.center.y);
                pts.extend((1..n).map(|k| {
                    let t = turn * k as f64 / n as f64;
                    let (co, si) = (libm::cos(t), libm::sin(t));
                    [a.center.x + co * x - si * y, a.center.y + si * x + co * y]
                }));
            }
            pts.push([c.to.x, c.to.y]);
        }
        pts
    }
}

/// The copies of pattern `name`, which is `p` with `o`.
pub(crate) fn place(
    sketch: &Sketch,
    s: &Placed,
    name: &str,
    p: &Pattern,
    o: &Offset,
    cell: BTreeSet<String>,
    guide: BTreeSet<String>,
) -> Result<Copies, BrepError> {
    let bad = |m: String| BrepError::Sketch(format!("pattern `{name}`: {m}"));
    let members = group(sketch, &p.of);
    let chains = chains_of(s, &cell, &members).map_err(bad)?;
    let step = o.step.value().filter(|v| v.is_finite() && *v != 0.0).ok_or_else(|| bad("`step` must be a number other than zero".into()))?;
    let count = whole(&o.count).ok_or_else(|| bad("`count` must be a whole number from 1".into()))?;
    let clearance = p.clearance.value().filter(|v| v.is_finite() && *v >= 0.0).ok_or_else(|| bad("`clearance` must be a number from 0".into()))?;
    let bound = |g: &Option<String>| g.as_ref().map(|g| stops(sketch, s, g).map(|st| (g.clone(), st))).transpose().map_err(bad);
    let (start, end) = (bound(&o.start)?, bound(&o.end)?);
    let mut copies = Vec::with_capacity(count as usize * chains.len());
    for k in 0..count {
        for chain in &chains {
            let of = |m: String| bad(format!("copy {k}, chain of `{}`: {m}", chain[0].0));
            let mut pieces = shifted(chain, k as f64 * step).map_err(of)?;
            let (at, mid) = middle(&pieces);
            if let Some((g, st)) = &end {
                reach(&mut pieces, at, mid, st, clearance).map_err(|m| of(format!("its end {m} `{g}`")))?;
            }
            if let Some((g, st)) = &start {
                let mut back = turned(&pieces);
                let at = back.len() - 1 - at;
                reach(&mut back, at, mid, st, clearance).map_err(|m| of(format!("its start {m} `{g}`")))?;
                pieces = turned(&back);
            }
            let pieces = pieces.into_iter().map(|(n, c)| (if k == 0 { n } else { format!("{name}[{k}].{n}") }, c)).collect();
            copies.push(ChainCopy { k, pieces });
        }
    }
    Ok(Copies { name: name.to_string(), cell, guide, at: Vec::new(), chains: copies, source: true, area: 0.0, loops: 0, region: None })
}

fn group(sketch: &Sketch, n: &str) -> Vec<String> {
    sketch.groups.get(n).cloned().unwrap_or_else(|| vec![n.to_string()])
}

/// The open chains of `cell`, tangent at their joints, in the order of their first curves in
/// `members`. The first runs the way its first curve is drawn, the others along with it.
fn chains_of(s: &Placed, cell: &BTreeSet<String>, members: &[String]) -> Result<Vec<Chain>, String> {
    let keep = |n: &str, construction: bool| !construction && cell.contains(n);
    let found = contours(&s.lines, &s.arcs, &keep);
    if !found.branches.is_empty() {
        return Err(format!("its chains branch at point `{}`", found.branches[0]));
    }
    if !found.loops.is_empty() || cell.iter().any(|n| s.circles.contains_key(n)) {
        return Err("its curves must be open chains, not closed contours".into());
    }
    if found.paths.is_empty() {
        return Err("it has no line or arc to copy".into());
    }
    let rank = |n: &str| members.iter().position(|m| m == n).unwrap_or(usize::MAX);
    let mut ordered: Vec<(usize, Chain)> = Vec::with_capacity(found.paths.len());
    for path in &found.paths {
        let first = path.iter().min_by_key(|st| rank(st.curve)).unwrap_or(&path[0]);
        let run = if first.forward { path.clone() } else { datum_sketch::reversed(path) };
        ordered.push((rank(first.curve), named_pieces(s, &run).map_err(|e| e.to_string())?));
    }
    ordered.sort_by_key(|c| c.0);
    let mut chains: Vec<Chain> = ordered.into_iter().map(|c| c.1).collect();
    let toward = heading(&chains[0][0].1);
    for chain in &mut chains[1..] {
        let h = heading(&chain[0].1);
        if h.x * toward.x + h.y * toward.y < 0.0 {
            *chain = turned(chain);
        }
    }
    for chain in &chains {
        if let Some((a, b)) = first_corner(s, chain) {
            return Err(format!("copies cannot go round a corner: make `{a}` and `{b}` tangent"));
        }
    }
    Ok(chains)
}

/// Which way a piece sets off.
fn heading(c: &ChainPiece) -> Vec2 {
    match c.arc {
        None => Vec2::new(c.to.x - c.from.x, c.to.y - c.from.y),
        Some(a) => {
            let (x, y) = (c.from.x - a.center.x, c.from.y - a.center.y);
            if a.ccw { Vec2::new(-y, x) } else { Vec2::new(y, -x) }
        }
    }
}

/// The chain run the other way.
fn turned(chain: &[(String, ChainPiece)]) -> Chain {
    let back = |c: &ChainPiece| ChainPiece { from: c.to, to: c.from, arc: c.arc.map(|a| ProfileArc { center: a.center, ccw: !a.ccw }) };
    chain.iter().rev().map(|(n, c)| (n.clone(), back(c))).collect()
}

fn length(c: &ChainPiece) -> f64 {
    match c.arc {
        None => libm::hypot(c.to.x - c.from.x, c.to.y - c.from.y),
        Some(a) => libm::hypot(c.from.x - a.center.x, c.from.y - a.center.y) * a.turn(c.from, c.to).abs(),
    }
}

/// The unit normal to the left of a piece's run at its point `p`.
fn left(c: &ChainPiece, p: Vec2) -> Vec2 {
    match c.arc {
        None => {
            let (x, y) = (c.to.x - c.from.x, c.to.y - c.from.y);
            let l = libm::hypot(x, y);
            Vec2::new(-y / l, x / l)
        }
        Some(a) => {
            let (x, y) = (p.x - a.center.x, p.y - a.center.y);
            let l = if a.ccw { -libm::hypot(x, y) } else { libm::hypot(x, y) };
            Vec2::new(x / l, y / l)
        }
    }
}

/// `chain` at distance `d` to its left. Where pieces join they share the point moved, along an
/// arc's radius if one of them is an arc: exact there.
fn shifted(chain: &[(String, ChainPiece)], d: f64) -> Result<Chain, String> {
    let n = chain.len();
    let joints: Vec<Vec2> = (0..=n)
        .map(|i| {
            let before = i.checked_sub(1).map(|k| &chain[k].1);
            let after = chain.get(i).map(|(_, c)| c);
            let by = [before, after].into_iter().flatten().find(|c| c.arc.is_some()).or(before).or(after).unwrap_or(&chain[0].1);
            let p = before.map_or(chain[0].1.from, |c| c.to);
            let l = left(by, p);
            Vec2::new(p.x + l.x * d, p.y + l.y * d)
        })
        .collect();
    let mut out = Vec::with_capacity(n);
    for (i, (name, c)) in chain.iter().enumerate() {
        if let Some(a) = c.arc {
            let r = libm::hypot(c.from.x - a.center.x, c.from.y - a.center.y);
            if (if a.ccw { r - d } else { r + d }) <= 0.0 {
                return Err(format!("the arc `{name}` of radius {r:.3} has no copy so far to the side it curves to"));
            }
        }
        out.push((name.clone(), ChainPiece { from: joints[i], to: joints[i + 1], arc: c.arc }));
    }
    Ok(out)
}

/// Half way along a chain: the piece and the point.
fn middle(chain: &[(String, ChainPiece)]) -> (usize, Vec2) {
    let mut left_to_go = chain.iter().map(|(_, c)| length(c)).sum::<f64>() / 2.0;
    for (i, (_, c)) in chain.iter().enumerate() {
        let l = length(c);
        if left_to_go <= l || i == chain.len() - 1 {
            let t = left_to_go / l;
            return (
                i,
                match c.arc {
                    None => Vec2::new(c.from.x + (c.to.x - c.from.x) * t, c.from.y + (c.to.y - c.from.y) * t),
                    Some(a) => {
                        let angle = a.turn(c.from, c.to) * t;
                        let (x, y) = (c.from.x - a.center.x, c.from.y - a.center.y);
                        let (co, si) = (libm::cos(angle), libm::sin(angle));
                        Vec2::new(a.center.x + co * x - si * y, a.center.y + si * x + co * y)
                    }
                },
            );
        }
        left_to_go -= l;
    }
    (0, chain[0].1.from)
}

/// A curve that stops an end.
enum Stop {
    Line(Vec2, Vec2),
    /// Counterclockwise from the first point to the second.
    Arc {
        center: Vec2,
        radius: f64,
        from: Vec2,
        to: Vec2,
    },
    Circle(Vec2, f64),
}

/// The curves of group or curve `g`, construction ones too.
fn stops(sketch: &Sketch, s: &Placed, g: &str) -> Result<Vec<Stop>, String> {
    let at = |n: &str| s.solved.points.get(n).copied().ok_or_else(|| format!("no point `{n}`"));
    let mut out = Vec::new();
    for n in group(sketch, g) {
        if let Some(l) = s.lines.get(&n) {
            out.push(Stop::Line(at(&l.from)?, at(&l.to)?));
        } else if let Some(a) = s.arcs.get(&n) {
            let (center, from) = (at(&a.center)?, at(&a.from)?);
            out.push(Stop::Arc { center, radius: libm::hypot(from.x - center.x, from.y - center.y), from, to: at(&a.to)? });
        } else if let Some(c) = s.circles.get(&n) {
            let diameter = c.diameter.value().ok_or_else(|| format!("the circle `{n}` has no diameter yet"))?;
            out.push(Stop::Circle(at(&c.center)?, diameter / 2.0));
        }
    }
    if out.is_empty() {
        return Err(format!("`{g}` has no line, arc or circle to stop at"));
    }
    Ok(out)
}

fn near(p: Vec2, q: Vec2) -> bool {
    within_rounding(libm::hypot(p.x - q.x, p.y - q.y), size(p, q))
}

/// The coordinates' magnitude, what rounding is measured against.
fn size(p: Vec2, q: Vec2) -> f64 {
    p.x.abs().max(p.y.abs()).max(q.x.abs()).max(q.y.abs())
}

/// Counterclockwise angle about `c` from `a` to `b`, in `[0, 2π)`.
fn angle(c: Vec2, a: Vec2, b: Vec2) -> f64 {
    let (u, v) = (Vec2::new(a.x - c.x, a.y - c.y), Vec2::new(b.x - c.x, b.y - c.y));
    let t = libm::atan2(u.x * v.y - u.y * v.x, u.x * v.x + u.y * v.y);
    if t < 0.0 { t + TAU } else { t }
}

impl Stop {
    fn carrier(&self) -> Carrier {
        match *self {
            Self::Line(a, b) => Carrier::Line(a, b),
            Self::Arc { center, radius, .. } => Carrier::Circle(center, radius),
            Self::Circle(c, r) => Carrier::Circle(c, r),
        }
    }

    /// Whether `x`, on the curve's line or circle, is on the curve: an end by rounding too, so
    /// that an end shared by two curves of a group lets nothing through.
    fn holds(&self, x: Vec2) -> bool {
        match *self {
            Self::Line(a, b) => {
                let (u, v) = (Vec2::new(b.x - a.x, b.y - a.y), Vec2::new(x.x - a.x, x.y - a.y));
                let t = u.x * v.x + u.y * v.y;
                (t >= 0.0 && t <= u.x * u.x + u.y * u.y) || near(x, a) || near(x, b)
            }
            Self::Arc { center, from, to, .. } => angle(center, from, x) <= angle(center, from, to) || near(x, from) || near(x, to),
            Self::Circle(..) => true,
        }
    }

    /// The lines or circles at `d` to either side of its own.
    fn parallels(&self, d: f64) -> Vec<Carrier> {
        match self.carrier() {
            Carrier::Line(a, b) => {
                let (x, y) = (b.x - a.x, b.y - a.y);
                let l = libm::hypot(x, y);
                let n = Vec2::new(-y / l * d, x / l * d);
                [1.0, -1.0].map(|s| Carrier::Line(Vec2::new(a.x + n.x * s, a.y + n.y * s), Vec2::new(b.x + n.x * s, b.y + n.y * s))).to_vec()
            }
            Carrier::Circle(c, r) => [r - d, r + d].into_iter().filter(|r| *r > 0.0).map(|r| Carrier::Circle(c, r)).collect(),
        }
    }
}

/// A stretch of the walk along a chain: a piece from a point on it, on past its end if `open`.
struct Stretch<'a> {
    piece: &'a ChainPiece,
    from: Vec2,
    open: bool,
}

impl Stretch<'_> {
    fn carrier(&self) -> Carrier {
        let c = self.piece;
        match c.arc {
            None => Carrier::Line(c.from, c.to),
            Some(a) => Carrier::Circle(a.center, libm::hypot(c.from.x - a.center.x, c.from.y - a.center.y)),
        }
    }

    /// How far along the stretch point `x` of its line or circle lies, if it is on the stretch.
    /// Past the end of an arc, no further than a whole turn of the piece.
    fn distance(&self, x: Vec2) -> Option<f64> {
        let c = self.piece;
        if near(x, self.from) {
            return Some(0.0);
        }
        match c.arc {
            None => {
                let (ux, uy) = (c.to.x - c.from.x, c.to.y - c.from.y);
                let l = libm::hypot(ux, uy);
                let (ux, uy) = (ux / l, uy / l);
                let d = (x.x - self.from.x) * ux + (x.y - self.from.y) * uy;
                let room = (c.to.x - self.from.x) * ux + (c.to.y - self.from.y) * uy;
                (d > 0.0 && (self.open || d <= room || near(x, c.to))).then_some(d)
            }
            Some(a) => {
                let swept = |p: Vec2, q: Vec2| if a.ccw { angle(a.center, p, q) } else { angle(a.center, q, p) };
                let t = swept(self.from, x);
                let radius = libm::hypot(self.from.x - a.center.x, self.from.y - a.center.y);
                let inside = t <= swept(self.from, c.to) || near(x, c.to);
                (inside || (self.open && t < TAU - swept(c.from, self.from))).then_some(t * radius)
            }
        }
    }
}

/// Goes from `mid` on piece `at` to the end of `chain` and past it, and ends the chain at the
/// first curve of `stops`, `clearance` short of it.
fn reach(chain: &mut Chain, at: usize, mid: Vec2, stops: &[Stop], clearance: f64) -> Result<(), String> {
    let last = chain.len() - 1;
    let stretches: Vec<Stretch<'_>> =
        (at..=last).map(|j| Stretch { piece: &chain[j].1, from: if j == at { mid } else { chain[j].1.from }, open: j == last }).collect();
    let on = |stretch: &Stretch<'_>, carrier: Carrier| -> Vec<(f64, Vec2)> {
        crossings(stretch.carrier(), carrier).into_iter().filter_map(|x| stretch.distance(x).map(|d| (d, x))).collect()
    };
    let hit = stretches.iter().enumerate().find_map(|(j, stretch)| {
        let nearest = stops
            .iter()
            .flat_map(|stop| on(stretch, stop.carrier()).into_iter().filter(|(_, x)| stop.holds(*x)).map(move |(d, x)| (d, x, stop)))
            .min_by(|a, b| a.0.total_cmp(&b.0));
        nearest.map(|n| (j, n))
    });
    let Some((j, (d, x, stop))) = hit else { return Err("reaches nothing of".into()) };
    let (j, x) = if clearance == 0.0 {
        (j, x)
    } else {
        // Back from the stop to where the path is `clearance` away from what stopped it.
        let parallels = stop.parallels(clearance);
        let before = |k: usize, e: f64| k < j || e < d;
        (0..=j)
            .rev()
            .find_map(|k| {
                let found = parallels.iter().flat_map(|c| on(&stretches[k], *c)).filter(|(e, _)| before(k, *e)).max_by(|a, b| a.0.total_cmp(&b.0));
                found.map(|(_, y)| (k, y))
            })
            .ok_or("is closer than the clearance to")?
    };
    let piece = at + j;
    chain.truncate(piece + 1);
    chain[piece].1.to = x;
    Ok(())
}
