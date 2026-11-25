//! A regular polygon along a circle or arc: a circle or an arc selected, N, the number of sides
//! typed, Enter. A full circle gets `n` vertices on it, the sides between them closed, three sides
//! at least; an arc's length is cut into `n` equal pieces, `n + 1` vertices from its start to its
//! end, the sides an open chain, one side at least (the chord between the arc's ends). The circle
//! or arc turns construction, and its radius still holds the vertices, kept on it (`on`): changing
//! the radius moves the polygon with it. Neighbouring sides are equal, so the polygon stays regular
//! as it is dragged (a closed one turns, one free degree; an open chain is pinned to the arc's ends
//! and holds still). The vertices, the sides and the curve are one group of the sketch, named
//! `polygonN`: its badge gives the sides count to change.

use serde_json::{Map, Value as Json, json};

use crate::draw::Patch;
use crate::sketch::{Pick, SketchEdit};
use crate::trim::Shape;

/// A polygon as its group holds it: the curve it stands on and the vertices the sides join.
struct Held {
    curve: String,
    points: Vec<String>,
    sides: usize,
    closed: bool,
}

/// The circle or arc of the selection the polygon goes on: the first, in the order selected.
pub(crate) fn curve(edit: &SketchEdit) -> Option<String> {
    let sketch = &edit.json["sketch"];
    edit.selected.iter().find_map(|p| match p {
        Pick::Curve(n) if sketch["circles"].get(n).is_some() || sketch["arcs"].get(n).is_some() => Some(n.clone()),
        _ => None,
    })
}

/// The patch putting the polygon of `typed` sides (as typed and its value) on the selected curve;
/// none without a whole count of the least for that curve, or with no circle or arc selected, or
/// with one already holding a polygon.
pub(crate) fn plan(edit: &SketchEdit, typed: &(Json, f64)) -> Option<Map<String, Json>> {
    let name = curve(edit)?;
    let p = edit.placed.as_ref().ok()?;
    let s = Shape::of(p, &name)?;
    let closed = p.circles.contains_key(&name);
    let n = whole(typed.1, least(closed))?;
    // One polygon to a curve: it has it already.
    if group_on(&edit.json["sketch"], &name).is_some() {
        return None;
    }
    let mut patch = Patch::new(edit);
    let group = patch.fresh("polygon");
    let members = put(&mut patch, edit, &name, closed, s, n);
    patch.set("groups", &group, json!(members));
    patch.done()
}

/// The patch rebuilding the polygon group `group` with `n` sides, kept in the same group: its old
/// vertices go, and with them the sides and constraints on them; new ones are put as [`plan`]
/// puts them.
pub(crate) fn reedit(edit: &SketchEdit, group: &str, n: usize) -> Option<Map<String, Json>> {
    let held = held(&edit.json["sketch"], group)?;
    if n < least(held.closed) {
        return None;
    }
    let p = edit.placed.as_ref().ok()?;
    let s = Shape::of(p, &held.curve)?;
    let mut patch = Patch::new(edit);
    for q in &held.points {
        patch.set("points", q, Json::Null);
    }
    let members = put(&mut patch, edit, &held.curve, held.closed, s, n);
    patch.set("groups", group, json!(members));
    patch.done()
}

/// The sides the polygon group `group` holds, if it is one: its badge gives them to change.
pub(crate) fn sides_of(edit: &SketchEdit, group: &str) -> Option<usize> {
    held(&edit.json["sketch"], group).map(|h| h.sides)
}

/// The sides count as typed into the group's form: a whole number of `least` or more.
pub(crate) fn typed(text: &str, least: usize) -> Option<usize> {
    text.trim().parse::<f64>().ok().and_then(|v| whole(v, least))
}

/// The fewest sides a polygon has on its curve: three on a circle, whose chain is closed; one on
/// an arc, whose chain is open (the chord between its ends is a side already).
fn least(closed: bool) -> usize {
    if closed { 3 } else { 1 }
}

/// The fewest sides the polygon group `group` takes, if it is one: three on a circle, one on an
/// arc. Its form asks for no fewer.
pub(crate) fn least_of(edit: &SketchEdit, group: &str) -> Option<usize> {
    held(&edit.json["sketch"], group).map(|h| least(h.closed))
}

/// A count as it must be: a whole number of `least` or more.
fn whole(v: f64, least: usize) -> Option<usize> {
    (v.fract() == 0.0 && v >= least as f64).then_some(v as usize)
}

/// The polygon `sketch` puts in group `group`, if the group is one: one construction circle or
/// arc, the vertices held on it (`on`), and the sides between them.
fn held(sketch: &Json, group: &str) -> Option<Held> {
    let members: Vec<&str> = sketch["groups"][group].as_array()?.iter().filter_map(Json::as_str).collect();
    let curves: Vec<&str> = members.iter().copied().filter(|n| sketch["circles"].get(*n).is_some() || sketch["arcs"].get(*n).is_some()).collect();
    let [curve] = curves[..] else { return None };
    let closed = sketch["circles"].get(curve).is_some();
    if sketch[if closed { "circles" } else { "arcs" }][curve]["construction"].as_bool() != Some(true) {
        return None;
    }
    let points: Vec<String> = members.iter().copied().filter(|n| sketch["points"].get(*n).is_some()).map(String::from).collect();
    let lines: Vec<&str> = members.iter().copied().filter(|n| sketch["lines"].get(*n).is_some()).collect();
    let sides = lines.len();
    if sides < least(closed) || points.len() != if closed { sides } else { sides + 1 } || points.len() + sides + 1 != members.len() {
        return None;
    }
    let none = Vec::new();
    let constraints = sketch["constraints"].as_array().unwrap_or(&none);
    let on = |q: &str| constraints.iter().any(|c| c["type"] == "on" && c["point"].as_str() == Some(q) && c["curve"].as_str() == Some(curve));
    let vertex = |n: &Json| n.as_str().is_some_and(|n| points.iter().any(|q| q == n));
    let joined = lines.iter().all(|l| {
        let line = &sketch["lines"][*l];
        vertex(&line["from"]) && vertex(&line["to"])
    });
    (points.iter().all(|q| on(q)) && joined).then(|| Held { curve: curve.to_string(), points, sides, closed })
}

/// The group of the polygon standing on `curve`, if there is one.
fn group_on(sketch: &Json, curve: &str) -> Option<String> {
    sketch["groups"].as_object()?.iter().find(|(g, _)| held(sketch, g).is_some_and(|h| h.curve == curve)).map(|(g, _)| g.clone())
}

/// `patch` given the polygon of `n` sides on `curve` (a circle or an arc): its vertices, its
/// sides, the sides equal one to the next, the curve construction. Returns the group's members.
fn put(patch: &mut Patch, edit: &SketchEdit, curve: &str, closed: bool, s: Shape, n: usize) -> Vec<Json> {
    let sketch = &edit.json["sketch"];
    // A circle: `n` vertices round it; an arc: `n + 1`, its length cut into `n`.
    let vertices = if closed { n } else { n + 1 };
    let points: Vec<String> = (0..vertices)
        .map(|k| {
            let q = patch.fresh("p");
            patch.set("points", &q, json!(s.at(k as f64 / n as f64)));
            patch.constrain(json!({"type": "on", "point": q, "curve": curve}));
            q
        })
        .collect();
    // An open chain starts at the arc's `from` and ends at its `to`: it stays where the arc does.
    if !closed {
        let arc = &sketch["arcs"][curve];
        patch.constrain(json!({"type": "coincident", "points": [points[0], arc["from"]]}));
        patch.constrain(json!({"type": "coincident", "points": [points[vertices - 1], arc["to"]]}));
    }
    // A circle's last side closes to its first vertex; an arc's chain is open.
    let segments = if closed { vertices } else { vertices - 1 };
    let lines: Vec<String> = (0..segments)
        .map(|k| {
            let l = patch.fresh("l");
            patch.set("lines", &l, json!({"from": points[k], "to": points[(k + 1) % vertices]}));
            l
        })
        .collect();
    // Regular: each side as long as the next; a closed one keeps its turn free.
    for w in lines.windows(2) {
        patch.constrain(json!({"type": "equal", "lines": [w[0], w[1]]}));
    }
    // The circle or arc stays as a holder of the polygon: construction geometry.
    let mut c = sketch[if closed { "circles" } else { "arcs" }][curve].clone();
    c["construction"] = json!(true);
    patch.set(if closed { "circles" } else { "arcs" }, curve, c);
    points.into_iter().chain(lines).map(|m| json!(m)).chain([json!(curve)]).collect()
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use serde_json::{Value as Json, json};

    use super::*;
    use crate::draw::Tool;
    use crate::draw::tests::{click_at, editor};
    use crate::sketch::{Pick, SketchEdit};

    /// A sketch with circle `c` of diameter 20 about `[0, 0]`, selected.
    fn circle() -> SketchEdit {
        let mut e = editor(
            json!({"points": {"o": [0, 0]},
                   "circles": {"c": {"center": "o", "diameter": 20}},
                   "constraints": [{"type": "fix", "point": "o"}]}),
            false,
        );
        e.draft.tool = Tool::Polygon;
        e.selected = vec![Pick::Curve("c".into())];
        e
    }

    /// An arc about `[0, 0]` of radius 10 from angle 0 a quarter turn round, selected.
    fn arc() -> SketchEdit {
        let mut e = editor(
            json!({"points": {"o": [0, 0], "s": [10, 0], "t": [0, 10]},
                   "arcs": {"a": {"center": "o", "from": "s", "to": "t"}},
                   "constraints": [{"type": "fix", "point": "o"}, {"type": "radius", "arc": "a", "value": 10}]}),
            false,
        );
        e.draft.tool = Tool::Polygon;
        e.selected = vec![Pick::Curve("a".into())];
        e
    }

    /// Where point `n` of the solved sketch is, rounded to a millionth.
    fn at(e: &SketchEdit, n: &str) -> [f64; 2] {
        let v = e.placed.as_ref().unwrap().solved.points[n];
        [(v.x * 1e6).round() / 1e6, (v.y * 1e6).round() / 1e6]
    }

    /// The names of the curves in `key` that are not in `before`.
    fn new_ones(e: &SketchEdit, key: &str, before: &Json) -> Vec<String> {
        e.json["sketch"][key]
            .as_object()
            .into_iter()
            .flatten()
            .filter(|(n, _)| before[key].get(n.as_str()).is_none())
            .map(|(n, _)| n.clone())
            .collect()
    }

    /// The polygon's group in `e` (the only one): its name and its members.
    fn group(e: &SketchEdit) -> (String, Vec<String>) {
        let groups = e.json["sketch"]["groups"].as_object().unwrap();
        let (name, members) = groups.iter().next().expect("the polygon's group");
        (name.clone(), members.as_array().unwrap().iter().map(|m| m.as_str().unwrap().to_string()).collect())
    }

    /// Whether the polygon's sides are all as long as its first, to a millionth: regular.
    fn regular(e: &SketchEdit) -> bool {
        let (_, members) = group(e);
        let p = e.placed.as_ref().unwrap();
        let lengths: Vec<f64> = members
            .iter()
            .filter_map(|n| p.lines.get(n))
            .map(|l| {
                let (a, b) = (&p.solved.points[&l.from], &p.solved.points[&l.to]);
                libm::hypot(b.x - a.x, b.y - a.y)
            })
            .collect();
        !lengths.is_empty() && lengths.iter().all(|l| (l - lengths[0]).abs() < 1e-6)
    }

    /// How far apart points `a` and `b` are solved.
    fn apart(e: &SketchEdit, a: &str, b: &str) -> f64 {
        let p = e.placed.as_ref().unwrap();
        let (a, b) = (&p.solved.points[a], &p.solved.points[b]);
        libm::hypot(b.x - a.x, b.y - a.y)
    }

    /// Six sides on the circle: six vertices round it, six lines, the last closing to the first.
    #[test]
    fn a_circle_gets_a_closed_polygon() {
        let mut e = circle();
        let before = e.json["sketch"].clone();
        e.draft.typed = Some((json!(6), 6.0));
        let p = click_at(&mut e, [0.0, 0.0], false).unwrap();
        assert!(e.placed.is_ok(), "{:?}", e.placed.as_ref().err());
        let points = new_ones(&e, "points", &before);
        let lines = new_ones(&e, "lines", &before);
        assert_eq!(points.len(), 6, "{p}");
        assert_eq!(lines.len(), 6, "{p}");
        // Even steps round the circle from its start, the radius 10.
        for (k, q) in points.iter().enumerate() {
            let a = std::f64::consts::TAU * k as f64 / 6.0;
            let (sn, cs) = libm::sincos(a);
            let want = [(10.0 * cs * 1e6).round() / 1e6, (10.0 * sn * 1e6).round() / 1e6];
            assert_eq!(at(&e, q), want, "{q}");
        }
        // The last side closes to the first vertex: each vertex starts one side and ends one.
        let from: Vec<String> = lines.iter().map(|l| e.json["sketch"]["lines"][l]["from"].as_str().unwrap().to_string()).collect();
        let to: Vec<String> = lines.iter().map(|l| e.json["sketch"]["lines"][l]["to"].as_str().unwrap().to_string()).collect();
        let sorted = |mut v: Vec<String>| {
            v.sort();
            v
        };
        assert_eq!(sorted(from), sorted(points.clone()), "closed: each vertex starts a side");
        assert_eq!(sorted(to), sorted(points), "closed: each vertex ends a side");
        // The vertices, the sides and the circle are the polygon's one group.
        let (g, members) = group(&e);
        assert_eq!(g, "polygon1");
        let mut want = new_ones(&e, "points", &before);
        want.extend(new_ones(&e, "lines", &before));
        want.push("c".into());
        assert_eq!(members, want, "the group holds the vertices, the sides and the circle");
        // Regular by the sides alone: five equalities between neighbours, no angle.
        let kinds = |kind: &str| e.json["sketch"]["constraints"].as_array().unwrap().iter().filter(|c| c["type"] == kind).count();
        assert_eq!(kinds("equal"), 5);
        assert_eq!(kinds("angle"), 0, "no angular constraints: the solver would call them redundant");
    }

    /// Four sides on the arc: five vertices from its start to its end, a quarter turn cut into
    /// four, four lines and not a fifth closing it.
    #[test]
    fn an_arc_gets_an_open_chain() {
        let mut e = arc();
        let before = e.json["sketch"].clone();
        e.draft.typed = Some((json!(4), 4.0));
        let p = click_at(&mut e, [0.0, 0.0], false).unwrap();
        assert!(e.placed.is_ok(), "{:?}", e.placed.as_ref().err());
        let points = new_ones(&e, "points", &before);
        let lines = new_ones(&e, "lines", &before);
        assert_eq!(points.len(), 5, "{p}");
        assert_eq!(lines.len(), 4, "{p}");
        for (k, q) in points.iter().enumerate() {
            let a = std::f64::consts::FRAC_PI_2 * k as f64 / 4.0;
            let (sn, cs) = libm::sincos(a);
            let want = [(10.0 * cs * 1e6).round() / 1e6, (10.0 * sn * 1e6).round() / 1e6];
            assert_eq!(at(&e, q), want, "{q}");
        }
        // An open chain: from the first vertex to the last, each side from one vertex to the next.
        let from: Vec<String> = lines.iter().map(|l| e.json["sketch"]["lines"][l]["from"].as_str().unwrap().to_string()).collect();
        let to: Vec<String> = lines.iter().map(|l| e.json["sketch"]["lines"][l]["to"].as_str().unwrap().to_string()).collect();
        let sorted = |mut v: Vec<String>| {
            v.sort();
            v
        };
        assert_eq!(sorted(from), sorted(points[..4].to_vec()), "open: the sides start at the first four");
        assert_eq!(sorted(to), sorted(points[1..].to_vec()), "open: the sides end at the last four");
        // The chain's ends are the arc's: the first vertex on its `from`, the last on its `to`.
        assert!(e.json["sketch"]["constraints"].as_array().unwrap().contains(&json!({"type": "coincident", "points": [points[0], "s"]})));
        assert!(e.json["sketch"]["constraints"].as_array().unwrap().contains(&json!({"type": "coincident", "points": [points[4], "t"]})));
    }

    /// An arc takes one side: a single chord from its start to its end, both the arc's.
    #[test]
    fn an_arc_takes_one_side() {
        let mut e = arc();
        let before = e.json["sketch"].clone();
        e.draft.typed = Some((json!(1), 1.0));
        let p = click_at(&mut e, [0.0, 0.0], false).unwrap();
        assert!(e.placed.is_ok(), "{:?}", e.placed.as_ref().err());
        let points = new_ones(&e, "points", &before);
        let lines = new_ones(&e, "lines", &before);
        assert_eq!(points.len(), 2, "{p}");
        assert_eq!(lines.len(), 1, "{p}");
        // One chord from the arc's start to its end.
        assert_eq!(at(&e, &points[0]), [10.0, 0.0]);
        assert_eq!(at(&e, &points[1]), [0.0, 10.0]);
        assert_eq!(e.json["sketch"]["lines"][&lines[0]]["from"], json!(points[0]));
        assert_eq!(e.json["sketch"]["lines"][&lines[0]]["to"], json!(points[1]));
        // The chord's ends are the arc's; one side has none to be equal to.
        let constraints = e.json["sketch"]["constraints"].as_array().unwrap();
        assert!(constraints.contains(&json!({"type": "coincident", "points": [points[0], "s"]})));
        assert!(constraints.contains(&json!({"type": "coincident", "points": [points[1], "t"]})));
        assert_eq!(constraints.iter().filter(|c| c["type"] == "equal").count(), 0, "one side, nothing to compare");
        // The group is the polygon's, the arc construction.
        let (g, members) = group(&e);
        assert_eq!(g, "polygon1");
        assert_eq!(members.len(), 4, "two vertices, one side, the arc");
        assert_eq!(e.json["sketch"]["arcs"]["a"]["construction"], json!(true));
        assert_eq!(sides_of(&e, &g), Some(1));
        assert!(regular(&e));
    }

    /// An arc takes two sides: a middle vertex on it, the two chords from the arc's ends equal.
    #[test]
    fn an_arc_takes_two_sides() {
        let mut e = arc();
        let before = e.json["sketch"].clone();
        e.draft.typed = Some((json!(2), 2.0));
        let p = click_at(&mut e, [0.0, 0.0], false).unwrap();
        assert!(e.placed.is_ok(), "{:?}", e.placed.as_ref().err());
        let points = new_ones(&e, "points", &before);
        let lines = new_ones(&e, "lines", &before);
        assert_eq!(points.len(), 3, "{p}");
        assert_eq!(lines.len(), 2, "{p}");
        // The arc's quarter turn cut into two: 0, 45 and 90 degrees.
        for (k, q) in points.iter().enumerate() {
            let a = std::f64::consts::FRAC_PI_2 * k as f64 / 2.0;
            let (sn, cs) = libm::sincos(a);
            let want = [(10.0 * cs * 1e6).round() / 1e6, (10.0 * sn * 1e6).round() / 1e6];
            assert_eq!(at(&e, q), want, "{q}");
        }
        // The middle vertex is on the arc, the chain's ends the arc's own.
        let constraints = e.json["sketch"]["constraints"].as_array().unwrap();
        assert!(constraints.contains(&json!({"type": "coincident", "points": [points[0], "s"]})));
        assert!(constraints.contains(&json!({"type": "coincident", "points": [points[2], "t"]})));
        assert_eq!(constraints.iter().filter(|c| c["type"] == "equal").count(), 1, "the two sides equal");
        let (g, members) = group(&e);
        assert_eq!(g, "polygon1");
        assert_eq!(members.len(), 6, "three vertices, two sides, the arc");
        assert_eq!(sides_of(&e, &g), Some(2));
        assert!(regular(&e));
    }

    /// The fewest sides the form takes follows the curve: three on a circle, one on an arc.
    #[test]
    fn the_fewest_sides_follow_the_curve() {
        let mut e = circle();
        e.draft.typed = Some((json!(4), 4.0));
        click_at(&mut e, [0.0, 0.0], false).unwrap();
        let (g, _) = group(&e);
        assert_eq!(least_of(&e, &g), Some(3));
        assert_eq!(typed("3", 3), Some(3), "three on a circle");
        assert_eq!(typed("2", 3), None, "two on a circle is none");
        assert!(reedit(&e, &g, 2).is_none(), "no two-sided polygon on a circle");
        assert!(reedit(&e, &g, 1).is_none(), "no one-sided polygon on a circle");
        let mut e = arc();
        e.draft.typed = Some((json!(4), 4.0));
        click_at(&mut e, [0.0, 0.0], false).unwrap();
        let (g, _) = group(&e);
        assert_eq!(least_of(&e, &g), Some(1));
        assert_eq!(typed("1", 1), Some(1), "one on an arc");
        assert_eq!(typed("0", 1), None, "none on an arc");
    }

    /// The sides of an arc's polygon changed through its group: three, then two, then one, in the
    /// same group and under the same name, the arc still construction.
    #[test]
    fn an_arc_polygon_changes_its_sides_through_the_group() {
        let mut e = arc();
        e.draft.typed = Some((json!(3), 3.0));
        click_at(&mut e, [0.0, 0.0], false).unwrap();
        let (name, _) = group(&e);
        assert_eq!(name, "polygon1");
        for n in [2usize, 1] {
            let p = reedit(&e, &name, n).unwrap_or_else(|| panic!("{n} sides on the arc"));
            e.apply(p);
            assert!(e.placed.is_ok(), "{:?}", e.placed.as_ref().err());
            let (again, members) = group(&e);
            assert_eq!(again, name, "the same group");
            assert_eq!(e.json["sketch"]["groups"].as_object().unwrap().len(), 1);
            let count = |key: &str| members.iter().filter(|m| e.json["sketch"][key].get(*m).is_some()).count();
            assert_eq!((count("points"), count("lines"), count("arcs")), (n + 1, n, 1), "{members:?}");
            assert_eq!(e.json["sketch"]["points"].as_object().unwrap().len(), n + 4, "the arc's ends, the centre and the vertices");
            let equals = e.json["sketch"]["constraints"].as_array().unwrap().iter().filter(|c| c["type"] == "equal").count();
            assert_eq!(equals, n - 1, "the old sides' equalities are gone with them");
            assert_eq!(sides_of(&e, &name), Some(n));
            assert_eq!(e.json["sketch"]["arcs"]["a"]["construction"], json!(true));
            assert!(regular(&e));
        }
    }

    /// The circle turns construction; every vertex stands on it (`on`), so its radius holds the
    /// polygon.
    #[test]
    fn the_curve_becomes_construction_and_holds_the_vertices() {
        let mut e = circle();
        let before = e.json["sketch"].clone();
        e.draft.typed = Some((json!(5), 5.0));
        click_at(&mut e, [0.0, 0.0], false).unwrap();
        assert_eq!(e.json["sketch"]["circles"]["c"]["construction"], json!(true));
        let points = new_ones(&e, "points", &before);
        let on: Vec<Json> = e.json["sketch"]["constraints"].as_array().unwrap().iter().filter(|c| c["type"] == "on").cloned().collect();
        assert_eq!(on.len(), points.len(), "{on:?}");
        for q in &points {
            assert!(on.contains(&json!({"type": "on", "point": q, "curve": "c"})), "{q}");
        }
    }

    /// The circle's polygon is a whole number of three or more sides: fewer is none there. An
    /// arc takes fewer (see the arc's own tests).
    #[test]
    fn a_circle_needs_a_whole_count_of_three_or_more() {
        let mut e = circle();
        for count in [None, Some((json!(0), 0.0)), Some((json!(1), 1.0)), Some((json!(2), 2.0)), Some((json!(4.5), 4.5))] {
            let why = format!("{count:?}");
            e.draft.typed = count;
            assert!(click_at(&mut e, [0.0, 0.0], false).is_none(), "{why}");
        }
        // Nothing selected: nothing either.
        e.selected.clear();
        e.draft.typed = Some((json!(6), 6.0));
        assert!(click_at(&mut e, [0.0, 0.0], false).is_none());
    }

    /// Dragged by a vertex, a circle's polygon turns as one: every side stays as long as the
    /// first, and its one degree of freedom is the turn.
    #[test]
    fn a_dragged_vertex_keeps_a_circle_polygon_regular() {
        let mut e = circle();
        e.draft.typed = Some((json!(6), 6.0));
        click_at(&mut e, [0.0, 0.0], false).unwrap();
        assert_eq!(e.placed.as_ref().unwrap().solved.dof, 1, "the turn alone");
        let (_, members) = group(&e);
        let q = members[0].clone();
        let was = at(&e, &q);
        e.hover = Some(Pick::Point(q.clone()));
        assert!(e.start_drag(at(&e, &q)), "a vertex can be dragged");
        e.drag_to([7.0, 7.0], 0.0);
        assert!(e.placed.is_ok(), "{:?}", e.placed.as_ref().err());
        assert_ne!(at(&e, &q), was, "the vertex moved");
        assert!(regular(&e), "{:?}", e.placed.as_ref().unwrap().solved.points);
        assert_eq!(e.placed.as_ref().unwrap().solved.dof, 1);
    }

    /// The arc's chain is pinned to its ends and its steps: nothing of it moves, and the steps
    /// are equal.
    #[test]
    fn an_arc_chain_holds_still() {
        let mut e = arc();
        // The arc's ends held: the chain is what has to hold still.
        for p in ["s", "t"] {
            e.json["sketch"]["constraints"].as_array_mut().unwrap().push(json!({"type": "fix", "point": p}));
        }
        e.solve();
        e.draft.typed = Some((json!(4), 4.0));
        click_at(&mut e, [0.0, 0.0], false).unwrap();
        assert_eq!(e.placed.as_ref().unwrap().solved.dof, 0, "pinned to the arc's ends {:?}", e.placed.as_ref().unwrap().solved.free);
        assert!(regular(&e));
        let (_, members) = group(&e);
        let q = members[0].clone();
        e.hover = Some(Pick::Point(q.clone()));
        assert!(!e.start_drag(at(&e, &q)), "nothing of the chain moves");
    }

    /// The arc edited, the chain follows and its steps come out equal again: the constraints
    /// hold it regular.
    #[test]
    fn an_arc_edited_puts_the_steps_equal_again() {
        let mut e = arc();
        e.draft.typed = Some((json!(4), 4.0));
        click_at(&mut e, [0.0, 0.0], false).unwrap();
        assert!(regular(&e));
        // The arc's end turned back to 80 degrees: its length cut into four again.
        let (sn, cs) = libm::sincos(80.0_f64.to_radians());
        e.json["sketch"]["points"]["t"] = json!([10.0 * cs, 10.0 * sn]);
        e.solve();
        assert!(e.placed.is_ok(), "{:?}", e.placed.as_ref().err());
        assert!(regular(&e), "{:?}", e.placed.as_ref().unwrap().solved.points);
        // The chain still ends where the arc does.
        let (_, members) = group(&e);
        assert!(apart(&e, &members[4], "t") < 1e-6, "{members:?}");
    }

    /// The sides count changed through the polygon's group: its elements are put anew in the
    /// same group, and none of the old ones is left.
    #[test]
    fn the_sides_change_through_the_group() {
        let mut e = circle();
        e.draft.typed = Some((json!(6), 6.0));
        click_at(&mut e, [0.0, 0.0], false).unwrap();
        let (name, members) = group(&e);
        let old = members.clone();
        assert_eq!(sides_of(&e, &name), Some(6));
        for n in [4, 8] {
            let p = reedit(&e, &name, n).unwrap();
            e.apply(p);
            assert!(e.placed.is_ok(), "{:?}", e.placed.as_ref().err());
            let (again, members) = group(&e);
            assert_eq!(again, name, "the same group");
            assert_eq!(e.json["sketch"]["groups"].as_object().unwrap().len(), 1);
            let count = |key: &str| members.iter().filter(|m| e.json["sketch"][key].get(*m).is_some()).count();
            assert_eq!((count("points"), count("lines"), count("circles")), (n, n, 1), "{members:?}");
            assert_eq!(e.json["sketch"]["lines"].as_object().unwrap().len(), n, "no stale sides");
            assert_eq!(e.json["sketch"]["points"].as_object().unwrap().len(), n + 1, "the centre and the vertices");
            for gone in &old {
                assert!(e.json["sketch"]["points"].get(gone).is_none(), "{gone} is left behind");
            }
            let equals = e.json["sketch"]["constraints"].as_array().unwrap().iter().filter(|c| c["type"] == "equal").count();
            assert_eq!(equals, n - 1, "the old sides' equalities are gone with them");
            assert_eq!(sides_of(&e, &name), Some(n));
            assert!(regular(&e));
        }
    }

    /// A curve holds one polygon: a second on it is refused.
    #[test]
    fn a_curve_holds_one_polygon() {
        let mut e = circle();
        e.draft.typed = Some((json!(5), 5.0));
        click_at(&mut e, [0.0, 0.0], false).unwrap();
        assert!(plan(&e, &(json!(4), 4.0)).is_none(), "the circle has its polygon already");
        assert!(click_at(&mut e, [0.0, 0.0], false).is_none());
        assert_eq!(e.json["sketch"]["groups"].as_object().unwrap().len(), 1);
        assert_eq!(sides_of(&e, "polygon1"), Some(5));
    }

    /// A pattern's cell or a contour is no polygon: its group has no vertices held on a
    /// construction curve.
    #[test]
    fn other_groups_are_no_polygon() {
        let e = editor(
            json!({"points": {"o": [0, 0], "a": [10, 0], "b": [10, 10]},
                   "lines": {"l1": {"from": "o", "to": "a"}, "l2": {"from": "a", "to": "b"}},
                   "circles": {"c": {"center": "o", "diameter": 20, "construction": true}},
                   "groups": {"cell": ["c"], "contour": ["l1", "l2"]},
                   "constraints": [{"type": "fix", "point": "o"}]}),
            false,
        );
        assert_eq!(sides_of(&e, "cell"), None, "a cell of one curve");
        assert_eq!(sides_of(&e, "contour"), None, "a contour of lines");
        // A group of the curve and a point on it, without sides, is none either.
        let mut e = e;
        e.json["sketch"]["groups"]["held"] = json!(["c", "a"]);
        e.json["sketch"]["constraints"].as_array_mut().unwrap().push(json!({"type": "on", "point": "a", "curve": "c"}));
        e.solve();
        assert_eq!(sides_of(&e, "held"), None);
    }
}
