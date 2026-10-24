//! Patterns of a sketch (D128): copies of a group's closed profiles on a grid, so many or as
//! many as a region holds. The copies are a cell and the offsets it is moved by: the solver sees
//! only the cell, and the profiles of the copies are made where the sketch's profiles are
//! (`profile_loops`), for a cut and an extrusion alike.

use std::collections::BTreeSet;

use datum_brep::{Box2, BrepError, ProfileArc, ProfileLoop, TopoId, overlap, overlapping};
use datum_math::Vec2;
use datum_sketch::{Pattern, Sketch};
use rayon::prelude::*;

use crate::sketch::{Found, Placed, loops};

/// A pattern of a sketch, placed: the curves it copies and each copy, by its place on the grid
/// and the offset it is moved by; what the sketch's status reports of it.
#[derive(Clone, Debug, PartialEq)]
pub struct Copies {
    pub name: String,
    /// The profile curves copied: the group's lines, arcs and circles, not construction.
    pub cell: BTreeSet<String>,
    pub at: Vec<([i64; 2], Vec2)>,
    /// Copy `(0, 0)` is the cell itself (`count`), keeping its IDs; filling, the cell is a sample.
    pub source: bool,
    /// The area one copy opens, and its profiles.
    pub area: f64,
    pub loops: usize,
    /// The area of the region filled.
    pub region: Option<f64>,
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
    let cells: Vec<BTreeSet<String>> =
        sketch.patterns.values().map(|p| members(&p.of).into_iter().filter(|n| curve(n) && construction(n) == Some(false)).collect()).collect();
    let copied: BTreeSet<&String> = cells.iter().flatten().collect();
    let mut out = Vec::with_capacity(sketch.patterns.len());
    for ((name, p), cell) in sketch.patterns.iter().zip(&cells) {
        let bad = |m: String| BrepError::Sketch(format!("pattern `{name}`: {m}"));
        let base = loops(TopoId::derive("sketch.cell").finish(), s, &|n| cell.contains(n), false).map_err(|e| bad(format!("its cell: {e}")))?;
        if base.is_empty() {
            return Err(bad(format!("`{}` has no closed profile to copy", p.of)));
        }
        let step = steps(p).map_err(bad)?;
        let (area, _) = nesting(&base);
        let (at, source, region) = match (&p.count, &p.fill) {
            (Some(count), _) => {
                let n = count.each_ref().map(|c| c.value().filter(|v| v.fract() == 0.0 && *v >= 1.0 && *v <= 1e6));
                let [Some(nx), Some(ny)] = n else { return Err(bad("`count` must be whole numbers from 1".into())) };
                let (nx, ny) = (nx as i64, ny as i64);
                let at = (0..ny).flat_map(|j| (0..nx).map(move |i| [i, j])).map(|k| (k, offset(step, k[0] as f64, k[1] as f64))).collect();
                (at, true, None)
            }
            (None, Some(fill)) => {
                let region: BTreeSet<String> = members(fill).into_iter().collect();
                let ring =
                    loops(TopoId::derive("sketch.region").finish(), s, &|n| region.contains(n), true).map_err(|e| bad(format!("its region: {e}")))?;
                let [ring] = &ring[..] else { return Err(bad(format!("`{fill}` must bound one region"))) };
                let others = loops(TopoId::derive("sketch.others").finish(), s, &|n| !copied.contains(&n.to_string()) && !region.contains(n), false)?;
                let clearance =
                    p.clearance.value().filter(|c| c.is_finite() && *c >= 0.0).ok_or_else(|| bad("`clearance` must be 0 or more".into()))?;
                let at = fill_region(&base, &ring.ring, &others, step, clearance).map_err(bad)?;
                (at, false, Some(ring.ring.area().abs()))
            }
            (None, None) => return Err(bad("give `count` or `fill`".into())),
        };
        let mut at: Vec<([i64; 2], Vec2)> = at.into_iter().filter(|(k, _)| !p.skip.contains(k)).collect();
        for (key, by) in &p.shift {
            let k = parse_copy(key).ok_or_else(|| bad(format!("shift `{key}`: a copy is named `i,j`")))?;
            let [Some(dx), Some(dy)] = by.each_ref().map(|n| n.value().filter(|v| v.is_finite())) else {
                return Err(bad(format!("shift `{key}`: not a number")));
            };
            if let Some((_, d)) = at.iter_mut().find(|(c, _)| *c == k) {
                *d = Vec2::new(d.x + dx, d.y + dy);
            }
        }
        out.push(Copies { name: name.clone(), cell: cell.clone(), at, source, area, loops: base.len(), region });
    }
    keep_apart(s, &out)?;
    Ok(out)
}

/// The two steps of a pattern.
fn steps(p: &Pattern) -> Result<[Vec2; 2], String> {
    let v = |s: &[datum_math::expr::Num; 2]| match s.each_ref().map(|n| n.value().filter(|v| v.is_finite())) {
        [Some(x), Some(y)] => Ok(Vec2::new(x, y)),
        _ => Err("`step` must be numbers".to_string()),
    };
    Ok([v(&p.step[0])?, v(&p.step[1])?])
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

/// Loop `l` moved by `d`, its IDs those of copy `k` of pattern `name` (copy `(0, 0)` of a cell
/// copied from itself keeps its own).
pub(crate) fn moved(l: &ProfileLoop, d: Vec2, feature: TopoId, name: &str, k: [i64; 2], own: bool) -> ProfileLoop {
    let id =
        |t: TopoId| if own { t } else { TopoId::derive("sketch.copy").id(feature).tag(name).index(k[0] as u64).index(k[1] as u64).id(t).finish() };
    ProfileLoop {
        points: l.points.iter().map(|p| Vec2::new(p.x + d.x, p.y + d.y)).collect(),
        vertices: l.vertices.iter().map(|v| v.map(id)).collect(),
        sides: l.sides.iter().map(|&s| id(s)).collect(),
        arcs: l.arcs.iter().map(|a| a.map(|a| ProfileArc { center: Vec2::new(a.center.x + d.x, a.center.y + d.y), ..a })).collect(),
    }
}

/// Copies touch nothing and hold nothing: no other copy, no other profile of the sketch.
fn keep_apart(s: &Placed, patterns: &[Copies]) -> Result<(), BrepError> {
    let id = TopoId::derive("sketch.check").finish();
    let copied: BTreeSet<&String> = patterns.iter().flat_map(|p| &p.cell).collect();
    let own = loops(id, s, &|n| !copied.contains(&n.to_string()), false)?;
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
