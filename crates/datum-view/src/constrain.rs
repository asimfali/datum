//! Geometric constraints added by hand (D118), as in Inventor's constraint panel: pick a kind,
//! then click what it ties; it is added once enough is picked. Constraints are otherwise only
//! inferred while drawing (D91), which misses what is drawn onto what is there already (an arc
//! ended on a line's end is not tangent to it).

use datum_ops::Placed;
use serde_json::{Value as Json, json};

use crate::dims::Pick2;
use crate::sketch::Glyph;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    Coincident,
    Horizontal,
    Vertical,
    Parallel,
    Perpendicular,
    Collinear,
    Equal,
    Tangent,
    Concentric,
    Midpoint,
    Symmetric,
    Fix,
}

/// Every kind with its glyph, name and what to click, in the panel's order.
pub const KINDS: [(Kind, Glyph, &str, &str); 12] = [
    (Kind::Coincident, Glyph::Coincident, "Coincident", "two points, or a point and a curve it lies on"),
    (Kind::Horizontal, Glyph::Horizontal, "Horizontal", "a line, or two points on one horizontal"),
    (Kind::Vertical, Glyph::Vertical, "Vertical", "a line, or two points on one vertical"),
    (Kind::Parallel, Glyph::Parallel, "Parallel", "two lines"),
    (Kind::Perpendicular, Glyph::Perpendicular, "Perpendicular", "two lines"),
    (Kind::Collinear, Glyph::Collinear, "Collinear", "two lines on one straight line"),
    (Kind::Equal, Glyph::Equal, "Equal", "two lines of one length, or two arcs or circles of one radius"),
    (Kind::Tangent, Glyph::Tangent, "Tangent", "two curves touching, one of them an arc or circle"),
    (Kind::Concentric, Glyph::Concentric, "Concentric", "two arcs or circles around one centre"),
    (Kind::Midpoint, Glyph::Midpoint, "Midpoint", "a point and the line it is halfway along"),
    (Kind::Symmetric, Glyph::Symmetric, "Symmetric", "two points, then the line they mirror in"),
    (Kind::Fix, Glyph::Fix, "Fix", "a point, held where it is"),
];

/// What the next click should pick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Want {
    Point,
    Curve,
    Either,
}

/// What a click picks next for a constraint of `kind`, `picks` picked so far: the point or the
/// curve under the cursor, as it needs (a tangent wants the line, not its end).
pub fn wants(kind: Kind, picks: &[Pick2]) -> Want {
    let first_point = matches!(picks.first(), Some(Pick2::Point(_)));
    match (kind, picks.len()) {
        (Kind::Fix, _) => Want::Point,
        (Kind::Horizontal | Kind::Vertical, 0) | (Kind::Midpoint, 0) => Want::Either,
        (Kind::Horizontal | Kind::Vertical, _) => Want::Point,
        (Kind::Coincident, 0) => Want::Point,
        (Kind::Coincident, _) => Want::Either,
        (Kind::Midpoint, _) if first_point => Want::Curve,
        (Kind::Midpoint, _) => Want::Point,
        (Kind::Symmetric, n) if n < 2 => Want::Point,
        _ => Want::Curve,
    }
}

/// What the picks come to.
#[derive(Clone, Debug, PartialEq)]
pub enum Step {
    /// Not enough picked yet.
    More,
    /// The constraint to add.
    Done(Json),
    /// The picks cannot make this kind: why.
    Wrong(String),
}

/// The constraint of `kind` tying `picks` in sketch `placed`, or what is still missing.
pub fn step(kind: Kind, picks: &[Pick2], placed: &Placed) -> Step {
    use Pick2::{Curve as C, Point as P};
    let line = |n: &str| placed.lines.contains_key(n);
    let round = |n: &str| placed.arcs.contains_key(n) || placed.circles.contains_key(n);
    let wrong = |what: &str| Step::Wrong(format!("{}: {what}", KINDS.iter().find(|k| k.0 == kind).map_or("", |k| k.2)));
    match (kind, picks) {
        (_, []) => Step::More,
        (Kind::Fix, [P(p)]) => Step::Done(json!({"type": "fix", "point": p})),
        (Kind::Horizontal | Kind::Vertical, [C(l)]) if line(l) => Step::Done(json!({"type": axis(kind), "line": l})),
        (Kind::Horizontal | Kind::Vertical, [P(_)]) => Step::More,
        (Kind::Horizontal | Kind::Vertical, [P(a), P(b)]) if a != b => Step::Done(json!({"type": axis(kind), "points": [a, b]})),
        (Kind::Coincident, [P(_)]) => Step::More,
        (Kind::Coincident, [P(a), P(b)]) if a != b => Step::Done(json!({"type": "coincident", "points": [a, b]})),
        (Kind::Coincident, [P(p), C(c)]) => Step::Done(json!({"type": "on", "point": p, "curve": c})),
        (Kind::Parallel | Kind::Perpendicular | Kind::Collinear, [C(a)]) if line(a) => Step::More,
        (Kind::Parallel | Kind::Perpendicular | Kind::Collinear, [C(a), C(b)]) if line(a) && line(b) && a != b => {
            let t = match kind {
                Kind::Parallel => "parallel",
                Kind::Perpendicular => "perpendicular",
                _ => "collinear",
            };
            Step::Done(json!({"type": t, "lines": [a, b]}))
        }
        (Kind::Equal | Kind::Tangent | Kind::Concentric, [C(_)]) => Step::More,
        (Kind::Equal, [C(a), C(b)]) if line(a) && line(b) && a != b => Step::Done(json!({"type": "equal", "lines": [a, b]})),
        (Kind::Equal, [C(a), C(b)]) if round(a) && round(b) && a != b => Step::Done(json!({"type": "equal_radius", "curves": [a, b]})),
        (Kind::Tangent, [C(a), C(b)]) if (round(a) || round(b)) && a != b => Step::Done(json!({"type": "tangent", "curves": [a, b]})),
        (Kind::Concentric, [C(a), C(b)]) if round(a) && round(b) && a != b => Step::Done(json!({"type": "concentric", "curves": [a, b]})),
        (Kind::Midpoint, [_]) => Step::More,
        (Kind::Midpoint, [P(p), C(l)] | [C(l), P(p)]) if line(l) => Step::Done(json!({"type": "midpoint", "point": p, "line": l})),
        (Kind::Symmetric, [P(_)] | [P(_), P(_)]) => Step::More,
        (Kind::Symmetric, [P(a), P(b), C(l)]) if line(l) && a != b => Step::Done(json!({"type": "symmetric", "points": [a, b], "line": l})),
        _ => wrong(KINDS.iter().find(|k| k.0 == kind).map_or("", |k| k.3)),
    }
}

fn axis(kind: Kind) -> &'static str {
    if kind == Kind::Horizontal { "horizontal" } else { "vertical" }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use std::collections::BTreeMap;

    use datum_brep::TopoId;

    use super::*;
    use crate::sketch::{Context, SketchEdit};
    use Pick2::{Curve as C, Point as P};

    /// The user's case: a line, then an arc ended on the line's end, drawn not tangent.
    fn editor() -> SketchEdit {
        let sketch = json!({"points": {"p1": [0, 0], "p2": [10, 0], "p3": [10, 5], "p4": [15, 5], "p5": [30, 0]},
            "lines": {"l1": {"from": "p1", "to": "p2"}, "l2": {"from": "p4", "to": "p5"}},
            "arcs": {"a1": {"center": "p3", "from": "p2", "to": "p4"}},
            "circles": {"c1": {"center": "p3", "diameter": 2}}});
        let json = json!({"name": "s", "op": "sketch", "inputs": [], "sketch": sketch});
        SketchEdit::new("s".into(), Context { id: TopoId::derive("feature").tag("s").finish(), input: None }, json, BTreeMap::new(), None)
    }

    fn c(n: &str) -> Pick2 {
        C(n.into())
    }

    fn p(n: &str) -> Pick2 {
        P(n.into())
    }

    #[test]
    fn what_each_kind_takes() {
        let e = editor();
        let placed = e.placed.as_ref().unwrap();
        let done = |kind, picks: &[Pick2]| match step(kind, picks, placed) {
            Step::Done(c) => c,
            other => panic!("{kind:?} {picks:?}: {other:?}"),
        };
        assert_eq!(done(Kind::Tangent, &[c("a1"), c("l2")]), json!({"type": "tangent", "curves": ["a1", "l2"]}));
        assert_eq!(done(Kind::Concentric, &[c("a1"), c("c1")]), json!({"type": "concentric", "curves": ["a1", "c1"]}));
        assert_eq!(done(Kind::Equal, &[c("a1"), c("c1")]), json!({"type": "equal_radius", "curves": ["a1", "c1"]}));
        assert_eq!(done(Kind::Equal, &[c("l1"), c("l2")]), json!({"type": "equal", "lines": ["l1", "l2"]}));
        assert_eq!(done(Kind::Horizontal, &[c("l1")]), json!({"type": "horizontal", "line": "l1"}));
        assert_eq!(done(Kind::Vertical, &[p("p1"), p("p3")]), json!({"type": "vertical", "points": ["p1", "p3"]}));
        assert_eq!(done(Kind::Coincident, &[p("p5"), c("a1")]), json!({"type": "on", "point": "p5", "curve": "a1"}));
        assert_eq!(done(Kind::Midpoint, &[c("l1"), p("p3")]), json!({"type": "midpoint", "point": "p3", "line": "l1"}));
        assert_eq!(done(Kind::Symmetric, &[p("p1"), p("p5"), c("l1")]), json!({"type": "symmetric", "points": ["p1", "p5"], "line": "l1"}));
        assert_eq!(step(Kind::Parallel, &[c("l1")], placed), Step::More);
        assert!(matches!(step(Kind::Tangent, &[c("l1"), c("l2")], placed), Step::Wrong(w) if w.contains("arc or circle")));
        assert!(matches!(step(Kind::Concentric, &[c("a1"), c("l1")], placed), Step::Wrong(_)));
        // What the next click picks: a tangent the curve, a midpoint's second what the first is not.
        assert_eq!(wants(Kind::Tangent, &[]), Want::Curve);
        assert_eq!(wants(Kind::Midpoint, &[c("l1")]), Want::Point);
        assert_eq!(wants(Kind::Symmetric, &[p("p1"), p("p5")]), Want::Curve);
    }

    /// Added through the window's check: tangent once, then refused as repeating it.
    #[test]
    fn a_constraint_that_repeats_is_refused() {
        let mut e = editor();
        let add = |e: &mut SketchEdit, c: Json| {
            let mut patch = serde_json::Map::new();
            patch.insert("add_constraints".into(), json!([c]));
            e.try_apply(patch, true)
        };
        assert!(add(&mut e, json!({"type": "tangent", "curves": ["a1", "l2"]})).is_ok());
        assert!(add(&mut e, json!({"type": "horizontal", "line": "l1"})).is_ok());
        let why = add(&mut e, json!({"type": "horizontal", "line": "l1"})).unwrap_err();
        assert!(why.contains("only repeat"), "{why}");
    }
}
