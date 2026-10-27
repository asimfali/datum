//! Contours (D165): lines and arcs joined end to end at their named points into closed loops and
//! open paths, by names alone (where the curves run is for the caller). One walk for every user:
//! the profiles an extrusion sweeps, a sheet's path, a pattern's path and region, what a selection
//! in the window bounds. A circle is a loop by itself and is not walked.

use std::collections::{BTreeMap, BTreeSet};

use crate::{Arc, Line};

/// One curve of a chain, the way the chain runs: from point `from` to point `to`; `forward` if
/// the curve itself runs that way (an arc: counterclockwise).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Step<'a> {
    pub curve: &'a str,
    pub from: &'a str,
    pub to: &'a str,
    pub forward: bool,
}

/// `chain` run the other way.
pub fn reversed<'a>(chain: &[Step<'a>]) -> Vec<Step<'a>> {
    chain.iter().rev().map(|s| Step { curve: s.curve, from: s.to, to: s.from, forward: !s.forward }).collect()
}

/// The lines and arcs kept, as chains.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Contours<'a> {
    /// Closed chains, curves with a free end taken away first (a tail off a square leaves the
    /// square): by the lowest-named point not yet walked, starting there along its first curve
    /// (lines before arcs, each by name). A chain where three or more curves meet is none.
    pub loops: Vec<Vec<Step<'a>>>,
    /// Open chains, unbranched: from the lower-named end.
    pub paths: Vec<Vec<Step<'a>>>,
    /// Points where an end is free, by name.
    pub ends: Vec<&'a str>,
    /// Points where three or more curves meet, by name.
    pub branches: Vec<&'a str>,
}

impl<'a> Contours<'a> {
    /// The lowest-named point where an end is free or three or more meet: none, and every curve
    /// kept is on a loop.
    pub fn odd(&self) -> Option<&'a str> {
        self.ends.iter().chain(&self.branches).min().copied()
    }
}

/// The chains of the lines and arcs `keep` keeps (by name and whether construction).
pub fn contours<'a>(lines: &'a BTreeMap<String, Line>, arcs: &'a BTreeMap<String, Arc>, keep: &dyn Fn(&str, bool) -> bool) -> Contours<'a> {
    // Curves at each point: (curve, the other end, whether it starts here).
    let mut at: BTreeMap<&str, Vec<(&str, &str, bool)>> = BTreeMap::new();
    let ends = lines.iter().map(|(n, l)| (n, &l.from, &l.to, l.construction)).chain(arcs.iter().map(|(n, a)| (n, &a.from, &a.to, a.construction)));
    for (name, from, to, _) in ends.filter(|(n, _, _, c)| keep(n, *c)) {
        at.entry(from).or_default().push((name, to, true));
        at.entry(to).or_default().push((name, from, false));
    }
    let points = |k: &dyn Fn(usize) -> bool| -> Vec<&str> { at.iter().filter(|(_, cs)| k(cs.len())).map(|(p, _)| *p).collect() };
    let (ends, branches) = (points(&|k| k == 1), points(&|k| k > 2));

    // Paths: walked from a free end while each point on the way has two curves.
    let mut paths = Vec::new();
    let mut walked: BTreeSet<&str> = BTreeSet::new();
    for (&start, cs) in at.iter().filter(|(_, cs)| cs.len() == 1) {
        if walked.contains(cs[0].0) {
            continue;
        }
        let (mut p, mut via, mut path) = (start, cs[0], Vec::new());
        let open = loop {
            walked.insert(via.0);
            path.push(Step { curve: via.0, from: p, to: via.1, forward: via.2 });
            p = via.1;
            match at[p][..] {
                [_] => break true,
                [a, b] => via = if a.0 == via.0 { b } else { a },
                _ => break false,
            }
        };
        if open {
            paths.push(path);
        }
    }

    // Loops: what is left once curves with a free end are taken away, one after another.
    let mut left = at.clone();
    let mut free = ends.clone();
    while let Some(p) = free.pop() {
        let Some(&[(curve, other, _)]) = left.get(p).map(Vec::as_slice) else { continue };
        left.remove(p);
        if let Some(cs) = left.get_mut(other) {
            cs.retain(|c| c.0 != curve);
            match cs.len() {
                0 => {
                    left.remove(other);
                }
                1 => free.push(other),
                _ => {}
            }
        }
    }
    let mut loops = Vec::new();
    let mut used: BTreeSet<&str> = BTreeSet::new();
    for (&start, cs) in &left {
        if cs.iter().all(|c| used.contains(c.0)) {
            continue;
        }
        let (mut p, mut via, mut ring, mut simple) = (start, cs[0], Vec::new(), true);
        loop {
            used.insert(via.0);
            ring.push(Step { curve: via.0, from: p, to: via.1, forward: via.2 });
            simple &= left[p].len() == 2;
            p = via.1;
            if p == start {
                break;
            }
            match left[p].iter().find(|c| !used.contains(c.0)) {
                Some(&next) => via = next,
                None => break,
            }
        }
        if simple && p == start {
            loops.push(ring);
            continue;
        }
        // Three or more meet on the way: nothing joined to it is a loop.
        let (mut stack, mut seen) = (vec![start], BTreeSet::from([start]));
        while let Some(q) = stack.pop() {
            for c in &left[q] {
                used.insert(c.0);
                if seen.insert(c.1) {
                    stack.push(c.1);
                }
            }
        }
    }
    Contours { loops, paths, ends, branches }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lines(ls: &[(&str, &str, &str)]) -> BTreeMap<String, Line> {
        ls.iter().map(|(n, a, b)| (n.to_string(), Line { from: a.to_string(), to: b.to_string(), construction: false })).collect()
    }

    fn names<'a>(chains: &[Vec<Step<'a>>]) -> Vec<Vec<&'a str>> {
        chains.iter().map(|c| c.iter().map(|s| s.curve).collect()).collect()
    }

    /// A square with a tail, an open chain the other way round, an arc closing a loop with a line.
    #[test]
    fn loops_and_paths() {
        let ls = lines(&[
            ("s1", "a", "b"),
            ("s2", "b", "c"),
            ("s3", "c", "d"),
            ("s4", "d", "a"),
            ("tail", "b", "t"),
            ("p1", "y", "x"),
            ("p2", "y", "z"),
            ("chord", "m", "n"),
        ]);
        let arcs: BTreeMap<String, Arc> =
            [("cap".to_string(), Arc { center: "o".into(), from: "n".into(), to: "m".into(), construction: false })].into_iter().collect();
        let c = contours(&ls, &arcs, &|_, _| true);
        assert_eq!(names(&c.loops), [vec!["s1", "s2", "s3", "s4"], vec!["chord", "cap"]]);
        assert_eq!(c.loops[1][1], Step { curve: "cap", from: "n", to: "m", forward: true });
        assert_eq!(names(&c.paths), [vec!["p1", "p2"]]);
        assert_eq!((c.paths[0][0].from, c.paths[0][0].forward), ("x", false));
        assert_eq!((&c.ends[..], &c.branches[..], c.odd()), (&["t", "x", "z"][..], &["b"][..], Some("b")));
    }

    /// Two squares sharing a side: three curves meet at two points, no loop is clear; a construction
    /// line is left out when asked.
    #[test]
    fn branches_are_no_loop() {
        let ls = lines(&[("l1", "a", "b"), ("l2", "b", "c"), ("l3", "c", "d"), ("l4", "d", "a"), ("mid", "b", "d"), ("other", "e", "f")]);
        let none = BTreeMap::new();
        let c = contours(&ls, &none, &|n, _| n != "other");
        assert!(c.loops.is_empty() && c.paths.is_empty());
        assert_eq!((c.ends.len(), &c.branches[..]), (0, &["b", "d"][..]));
    }
}
