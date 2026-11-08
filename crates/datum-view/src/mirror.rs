//! Mirror in a sketch (D244): the selected curves and points copied across an axis, a line of
//! the sketch or a projection (M, then a click on the axis). The original holds the copy, as in
//! Inventor: each point pair `symmetric` about the axis, each line pair `equal`, each arc or
//! circle pair `equal_radius`; deleting them makes the copy its own, its shape still held by the
//! original's constraints among what is copied, mirrored: lengths, radii, distances, parallels, tangents,
//! an angle with its lines swapped (a mirror turns it the other way); horizontal and vertical
//! only across a horizontal or vertical axis. A constraint that ties the copied to anything
//! else is not copied, nor a fix. Points on the axis (its ends, or held on it) are the copy's as
//! well, so half a profile drawn from the axis closes. Tab: the original goes, only the copy
//! is left.

use std::collections::{BTreeMap, BTreeSet};

use datum_sketch::Constraint;
use serde_json::{Value as Json, json};

use crate::draw::Patch;
use crate::sketch::{Pick, SketchEdit};
use crate::trim::{Plan, Shape, Sketch, dot, sub};

/// What mirroring `picks` across line `axis` would do, the original kept if `keep`.
pub fn plan(edit: &SketchEdit, picks: &[Pick], axis: &str, keep: bool) -> Option<Plan> {
    let p = edit.placed.as_ref().ok()?;
    let sk = Sketch::new(edit, p)?;
    let json = sk.json();
    let line = p.lines.get(axis)?;
    let (a, b) = (sk.at(&line.from)?, sk.at(&line.to)?);
    let u = sub(b, a);
    let l2 = dot(u, u);
    if l2 == 0.0 {
        return None;
    }
    let mirrored = |x: [f64; 2]| {
        let t = dot(sub(x, a), u) / l2;
        let f = [a[0] + u[0] * t, a[1] + u[1] * t];
        [2.0 * f[0] - x[0], 2.0 * f[1] - x[1]]
    };
    // What is copied: the sketch's own curves picked, their points, and points picked.
    let own = |key: &str, n: &str| json[key].get(n).is_some();
    let curves: Vec<(&str, String)> = picks
        .iter()
        .filter_map(|k| if let Pick::Curve(n) = k { Some(n) } else { None })
        .filter(|n| n.as_str() != axis)
        .filter_map(|n| ["lines", "arcs", "circles"].into_iter().find(|k| own(k, n)).map(|k| (k, n.clone())))
        .collect();
    let mut points: BTreeSet<String> =
        picks.iter().filter_map(|k| if let Pick::Point(n) = k { Some(n.clone()) } else { None }).filter(|n| own("points", n)).collect();
    for (key, n) in &curves {
        for f in ["from", "to", "center"] {
            if let Some(e) = json[*key][n][f].as_str() {
                points.insert(e.to_string());
            }
        }
    }
    if curves.is_empty() && points.is_empty() {
        return None;
    }
    // A point on the axis is its own mirror.
    let ends = [line.from.as_str(), line.to.as_str()];
    let on_axis = |q: &str| ends.iter().any(|e| sk.same(e, q)) || sk.on(q, axis);
    let mut patch = Patch::new(edit);
    let mut map: BTreeMap<String, String> = BTreeMap::new();
    map.insert(axis.to_string(), axis.to_string());
    let mut at: BTreeMap<String, [f64; 2]> = BTreeMap::new();
    for q in &points {
        let x = sk.at(q)?;
        if on_axis(q) {
            map.insert(q.clone(), q.clone());
            at.insert(q.clone(), x);
        } else {
            let n = patch.fresh("p");
            let y = mirrored(x);
            patch.set("points", &n, json!(y));
            map.insert(q.clone(), n.clone());
            at.insert(n, y);
        }
    }
    let name = |n: &Json| n.as_str().and_then(|n| map.get(n)).cloned();
    let mut added = Vec::new();
    let mut gone = Vec::new();
    let mut copies = Vec::new();
    for (key, n) in &curves {
        let v = &json[*key][n];
        let mut copy = v.clone();
        let shape = match *key {
            "lines" => {
                let (f, t) = (name(&v["from"])?, name(&v["to"])?);
                // Along the axis, it is its own mirror.
                if f == v["from"] && t == v["to"] {
                    continue;
                }
                copy["from"] = json!(f);
                copy["to"] = json!(t);
                Shape::Line(at[&f], at[&t])
            }
            "arcs" => {
                // Mirrored, counterclockwise runs the other way: the ends swap.
                let (c, f, t) = (name(&v["center"])?, name(&v["to"])?, name(&v["from"])?);
                copy["center"] = json!(c);
                copy["from"] = json!(f);
                copy["to"] = json!(t);
                let (cc, fc, tc) = (at[&c], at[&f], at[&t]);
                let start = libm::atan2(fc[1] - cc[1], fc[0] - cc[0]);
                let sweep = (libm::atan2(tc[1] - cc[1], tc[0] - cc[0]) - start).rem_euclid(std::f64::consts::TAU);
                let r = libm::hypot(fc[0] - cc[0], fc[1] - cc[1]);
                Shape::Round { c: cc, r, start, sweep: if sweep == 0.0 { std::f64::consts::TAU } else { sweep }, closed: false }
            }
            _ => {
                let c = name(&v["center"])?;
                copy["center"] = json!(c);
                let r = 0.5 * p.circles.get(n.as_str())?.diameter.value()?;
                Shape::Round { c: at[&c], r, start: 0.0, sweep: std::f64::consts::TAU, closed: true }
            }
        };
        let copied = patch.fresh(match *key {
            "lines" => "l",
            "arcs" => "a",
            _ => "c",
        });
        patch.set(key, &copied, copy);
        copies.push((*key, n.clone(), copied));
        added.push(shape.polyline(0.0, 1.0));
        if !keep {
            gone.push(Shape::of(p, n)?.polyline(0.0, 1.0));
        }
    }
    // Kept, the original holds the copy: each point mirrored by `symmetric`, each line of equal
    // length, each arc or circle of equal radius. Deleting them makes the copy its own.
    if keep {
        for q in points.iter().filter(|q| !on_axis(q)) {
            patch.constrain(json!({"type": "symmetric", "points": [q, map[q.as_str()]], "line": axis}));
        }
        for (key, n, copied) in &copies {
            patch.constrain(match *key {
                "lines" => json!({"type": "equal", "lines": [n, copied]}),
                _ => json!({"type": "equal_radius", "curves": [n, copied]}),
            });
        }
    }
    map.extend(copies.into_iter().map(|(_, n, copied)| (n, copied)));
    // The constraints among what is copied, mirrored.
    let level = sk.typed.iter().any(|c| match c {
        Constraint::Horizontal { line: Some(l), .. } | Constraint::Vertical { line: Some(l), .. } => l == axis,
        _ => false,
    });
    for (c, raw) in sk.typed.iter().zip(&sk.raw) {
        let keeps = match c {
            Constraint::Fix { .. } => false,
            Constraint::Horizontal { .. }
            | Constraint::Vertical { .. }
            | Constraint::HorizontalDistance { .. }
            | Constraint::VerticalDistance { .. } => level,
            _ => true,
        };
        if !keeps {
            continue;
        }
        let mut copy = raw.clone();
        let mut all = true;
        datum_model::visit_names(&mut copy, &mut |n| match map.get(n.as_str()) {
            Some(m) => *n = m.clone(),
            None => all = false,
        });
        if !all || copy == *raw {
            continue;
        }
        if let Constraint::Angle { .. } = c
            && let Some(l) = copy["lines"].as_array_mut()
        {
            l.reverse();
        }
        patch.constrain(copy);
    }
    // Only the copy: the original's curves, and its points off the axis nothing else holds.
    if !keep {
        let picked: BTreeSet<&str> = curves.iter().map(|(_, n)| n.as_str()).collect();
        for (key, n) in &curves {
            patch.set(key, n, Json::Null);
        }
        for q in points.iter().filter(|q| !on_axis(q)) {
            let held = ["lines", "arcs", "circles", "patterns"].iter().filter_map(|k| json[*k].as_object()).any(|m| {
                m.iter().filter(|(n, _)| !picked.contains(n.as_str())).any(|(_, v)| {
                    let mut found = false;
                    datum_model::visit_names(&mut v.clone(), &mut |n| found |= n == q);
                    found
                })
            });
            if !held {
                patch.set("points", q, Json::Null);
            }
        }
    }
    Some(Plan { patch: patch.done()?, gone, added })
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use serde_json::{Value as Json, json};

    use crate::draw::Tool;
    use crate::draw::tests::{click_at, editor};
    use crate::sketch::{Pick, SketchEdit};

    fn round(v: f64) -> f64 {
        (v * 1e9).round() / 1e9
    }

    /// The sketch's point `n` as the sketch's JSON has it, rounded.
    fn at(e: &SketchEdit, n: &str) -> [f64; 2] {
        let p = &e.json["sketch"]["points"][n];
        [round(p[0].as_f64().unwrap()), round(p[1].as_f64().unwrap())]
    }

    /// The names of `key` (`lines`, `points`, ...) that are not among `before`, in order.
    fn fresh(e: &SketchEdit, key: &str, before: &[&str]) -> Vec<String> {
        e.json["sketch"][key].as_object().map(|m| m.keys().filter(|k| !before.contains(&k.as_str())).cloned().collect()).unwrap_or_default()
    }

    /// The sketch's constraints of one `kind`.
    fn cons(e: &SketchEdit, kind: &str) -> Vec<Json> {
        e.json["sketch"]["constraints"].as_array().unwrap().iter().filter(|c| c["type"] == kind).cloned().collect()
    }

    fn half_profile() -> Json {
        json!({"points": {"a": [0, 0], "b": [10, 0], "c": [10, 10], "d": [0, 15], "t": [0, 20]},
               "lines": {"axis": {"from": "a", "to": "t", "construction": true}, "ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}, "cd": {"from": "c", "to": "d"}},
               "constraints": [{"type": "vertical", "line": "axis"}, {"type": "horizontal", "line": "ab"}, {"type": "vertical", "line": "bc"},
                               {"type": "length", "line": "ab", "value": 10}, {"type": "on", "point": "d", "curve": "axis"}]})
    }

    fn pick(e: &mut SketchEdit, names: &[&str]) {
        e.selected = names.iter().map(|n| Pick::Curve((*n).into())).collect();
    }

    /// Half a profile drawn from a vertical axis, mirrored across it: the other half appears
    /// on the left, joined to the axis points `a` and `d`, with the same horizontal, vertical
    /// and length of 10; the profile closes, the original stays and holds the copy (symmetric,
    /// equal), until those go.
    #[test]
    fn half_a_profile_closes_across_the_axis() {
        let mut e = editor(half_profile(), false);
        assert!(e.fill.is_empty());
        pick(&mut e, &["ab", "bc", "cd"]);
        e.start_mirror();
        assert_eq!(e.draft.tool, Tool::Mirror);
        click_at(&mut e, [0.0, 7.0], false).unwrap();
        assert_eq!(e.draft.tool, Tool::Select);
        assert!(e.placed.is_ok(), "{:?}", e.placed.as_ref().err());
        assert!(!e.fill.is_empty(), "the profile is closed");
        let pts = fresh(&e, "points", &["a", "b", "c", "d", "t"]);
        let lines = fresh(&e, "lines", &["axis", "ab", "bc", "cd"]);
        assert_eq!((pts.len(), lines.len()), (2, 3), "{pts:?} {lines:?}");
        let mut got: Vec<[f64; 2]> = pts.iter().map(|n| at(&e, n)).collect();
        got.sort_by(|p, q| p[1].total_cmp(&q[1]));
        assert_eq!(got, [[-10.0, 0.0], [-10.0, 10.0]]);
        let sk = &e.json["sketch"];
        let ends: Vec<(String, String)> =
            lines.iter().map(|l| (sk["lines"][l]["from"].as_str().unwrap().into(), sk["lines"][l]["to"].as_str().unwrap().into())).collect();
        assert!(ends.iter().any(|(f, t)| f == "a" || t == "a"), "{ends:?}");
        assert!(ends.iter().any(|(f, t)| f == "d" || t == "d"), "{ends:?}");
        let own = |kind: &str, key: &str| cons(&e, kind).iter().filter(|c| lines.iter().any(|l| c[key] == json!(l))).count();
        assert_eq!(own("horizontal", "line"), 1);
        assert_eq!(own("vertical", "line"), 1);
        let lens: Vec<Json> = cons(&e, "length").into_iter().filter(|c| lines.iter().any(|l| c["line"] == json!(l))).collect();
        assert_eq!(lens.len(), 1);
        assert_eq!(lens[0]["value"], json!(10));
        // The original is untouched.
        for (n, p) in [("a", [0.0, 0.0]), ("b", [10.0, 0.0]), ("c", [10.0, 10.0]), ("d", [0.0, 15.0])] {
            assert_eq!(at(&e, n), p);
        }
        assert!(["ab", "bc", "cd"].iter().all(|l| sk["lines"].get(l).is_some()));
        // The original holds the copy: its points off the axis symmetric, its lines equal; the
        // shared `a` and `d` need none.
        let sym = cons(&e, "symmetric");
        assert_eq!(sym.len(), 2, "{sym:?}");
        assert!(sym.iter().all(|c| c["line"] == "axis" && (c["points"][0] == "b" || c["points"][0] == "c")));
        assert_eq!(cons(&e, "equal").len(), 3);
        // Deleted, the copy is its own and keeps its shape.
        let ties: Vec<Json> = cons(&e, "symmetric").into_iter().chain(cons(&e, "equal")).collect();
        e.apply(serde_json::from_value(json!({"remove_constraints": ties})).unwrap());
        assert!(e.placed.is_ok(), "{:?}", e.placed.as_ref().err());
    }

    /// With Tab pressed (the original goes), only the copy is left: the lines `ab`, `bc`, `cd`
    /// and the points `b`, `c` are gone, `a` and `d` on the axis stay.
    #[test]
    fn the_original_goes_when_the_mirror_moves() {
        let mut e = editor(half_profile(), false);
        pick(&mut e, &["ab", "bc", "cd"]);
        e.start_mirror();
        e.draft.mirror_moves = true;
        click_at(&mut e, [0.0, 7.0], false).unwrap();
        assert!(e.placed.is_ok(), "{:?}", e.placed.as_ref().err());
        let sk = &e.json["sketch"];
        for l in ["ab", "bc", "cd"] {
            assert!(sk["lines"].get(l).is_none(), "{l} stays");
        }
        for q in ["b", "c"] {
            assert!(sk["points"].get(q).is_none(), "{q} stays");
        }
        assert!(sk["points"].get("a").is_some() && sk["points"].get("d").is_some());
        assert_eq!(fresh(&e, "lines", &["axis"]).len(), 3);
        let mut got: Vec<[f64; 2]> = fresh(&e, "points", &["a", "d", "t"]).iter().map(|n| at(&e, n)).collect();
        got.sort_by(|p, q| p[1].total_cmp(&q[1]));
        assert_eq!(got, [[-10.0, 0.0], [-10.0, 10.0]]);
        // Nothing left to tie the copy to.
        assert!(cons(&e, "symmetric").is_empty() && cons(&e, "equal").is_empty());
    }

    /// Across an axis at 45 degrees a horizontal line of length 5 comes out as a vertical one
    /// at the exact reflections of its ends; its length is held, but not "horizontal" (the
    /// copy is not horizontal).
    #[test]
    fn a_slanted_axis_keeps_the_length_not_the_level() {
        let mut e = editor(
            json!({"points": {"s": [0, 0], "t": [10, 10], "p": [4, 0], "q": [9, 0]},
                   "lines": {"axis": {"from": "s", "to": "t", "construction": true}, "pq": {"from": "p", "to": "q"}},
                   "constraints": [{"type": "fix", "point": "s"}, {"type": "fix", "point": "t"},
                                   {"type": "horizontal", "line": "pq"}, {"type": "length", "line": "pq", "value": 5}]}),
            false,
        );
        pick(&mut e, &["pq"]);
        e.start_mirror();
        click_at(&mut e, [5.0, 5.0], false).unwrap();
        assert!(e.placed.is_ok(), "{:?}", e.placed.as_ref().err());
        let copy = fresh(&e, "lines", &["axis", "pq"]);
        let [l] = &copy[..] else { panic!("one copy: {copy:?}") };
        let sk = &e.json["sketch"];
        assert_eq!(at(&e, sk["lines"][l]["from"].as_str().unwrap()), [0.0, 4.0]);
        assert_eq!(at(&e, sk["lines"][l]["to"].as_str().unwrap()), [0.0, 9.0]);
        let on_copy = |kind: &str, key: &str| cons(&e, kind).iter().filter(|c| c[key] == json!(l)).count();
        assert_eq!(on_copy("length", "line"), 1);
        assert_eq!(on_copy("horizontal", "line"), 0);
        assert_eq!(cons(&e, "horizontal").len(), 1, "the original's only");
    }

    /// A quarter arc mirrored across a vertical axis: the same radius, the centre on the other
    /// side, its ends swapped so that it still turns counterclockwise through the same 90 degrees.
    #[test]
    fn an_arc_keeps_its_sweep() {
        let mut e = editor(
            json!({"points": {"s": [0, -5], "t": [0, 5], "c": [10, 0], "f": [15, 0], "g": [10, 5]},
                   "lines": {"axis": {"from": "s", "to": "t", "construction": true}},
                   "arcs": {"arc": {"center": "c", "from": "f", "to": "g"}},
                   "constraints": [{"type": "vertical", "line": "axis"}, {"type": "radius", "arc": "arc", "value": 5}]}),
            false,
        );
        pick(&mut e, &["arc"]);
        e.start_mirror();
        click_at(&mut e, [0.0, 0.0], false).unwrap();
        assert!(e.placed.is_ok(), "{:?}", e.placed.as_ref().err());
        let arcs = fresh(&e, "arcs", &["arc"]);
        let [a] = &arcs[..] else { panic!("one copy: {arcs:?}") };
        let sk = &e.json["sketch"];
        let (c, f, t) = ["center", "from", "to"].map(|k| sk["arcs"][a][k].as_str().unwrap().to_string()).into();
        assert_eq!(at(&e, &c), [-10.0, 0.0]);
        // Its start is the mirror of the original's end, and its end of the original's start.
        assert_eq!(at(&e, &f), [-10.0, 5.0]);
        assert_eq!(at(&e, &t), [-15.0, 0.0]);
        let p = &e.placed.as_ref().unwrap().solved.points;
        let (cc, fc, tc) = (p[&c], p[&f], p[&t]);
        assert_eq!(round(libm::hypot(fc.x - cc.x, fc.y - cc.y)), 5.0);
        let sweep = (libm::atan2(tc.y - cc.y, tc.x - cc.x) - libm::atan2(fc.y - cc.y, fc.x - cc.x)).rem_euclid(std::f64::consts::TAU);
        assert_eq!(round(sweep.to_degrees()), 90.0);
        let radii: Vec<Json> = cons(&e, "radius").into_iter().filter(|c| c["arc"] == json!(a)).collect();
        assert_eq!(radii.len(), 1);
        assert_eq!(radii[0]["value"], json!(5));
    }

    /// A circle mirrored keeps its diameter as it was written, a number or an expression, and
    /// stands on the reflected centre.
    #[test]
    fn a_circle_keeps_its_diameter() {
        for d in [json!(8), json!("2 * 4")] {
            let mut e = editor(
                json!({"points": {"s": [0, -5], "t": [0, 5], "c": [10, 2]},
                       "lines": {"axis": {"from": "s", "to": "t", "construction": true}},
                       "circles": {"ring": {"center": "c", "diameter": d}},
                       "constraints": [{"type": "vertical", "line": "axis"}]}),
                false,
            );
            pick(&mut e, &["ring"]);
            e.start_mirror();
            click_at(&mut e, [0.0, 0.0], false).unwrap();
            assert!(e.placed.is_ok(), "{:?}", e.placed.as_ref().err());
            let cs = fresh(&e, "circles", &["ring"]);
            let [c] = &cs[..] else { panic!("one copy: {cs:?}") };
            let sk = &e.json["sketch"];
            assert_eq!(sk["circles"][c]["diameter"], d);
            assert_eq!(at(&e, sk["circles"][c]["center"].as_str().unwrap()), [-10.0, 2.0]);
        }
    }

    /// An angle of 60 degrees between two lines comes to the copy with its lines in the other
    /// order, since a mirror turns the angle the other way.
    #[test]
    fn an_angle_swaps_its_lines() {
        let mut e = editor(
            json!({"points": {"s": [-3, -5], "t": [-3, 20], "o": [0, 0], "a": [10, 0], "b": [5, 8.660254037844386]},
                   "lines": {"axis": {"from": "s", "to": "t", "construction": true}, "l1": {"from": "o", "to": "a"}, "l2": {"from": "o", "to": "b"}},
                   "constraints": [{"type": "vertical", "line": "axis"}, {"type": "angle", "lines": ["l1", "l2"], "degrees": 60}]}),
            false,
        );
        pick(&mut e, &["l1", "l2"]);
        e.start_mirror();
        click_at(&mut e, [-3.0, 10.0], false).unwrap();
        assert!(e.placed.is_ok(), "{:?}", e.placed.as_ref().err());
        let copies = fresh(&e, "lines", &["axis", "l1", "l2"]);
        let [c1, c2] = &copies[..] else { panic!("two copies: {copies:?}") };
        // The copy of l1 ends at the mirror of `a`: x = -6 - 10.
        let sk = &e.json["sketch"];
        let (first, second) = if at(&e, sk["lines"][c1]["to"].as_str().unwrap())[1] == 0.0 { (c1, c2) } else { (c2, c1) };
        assert_eq!(at(&e, sk["lines"][first]["to"].as_str().unwrap()), [-16.0, 0.0]);
        let angles = cons(&e, "angle");
        assert_eq!(angles.len(), 2, "the original and its copy");
        assert_eq!(angles[0]["lines"], json!(["l1", "l2"]));
        assert_eq!(angles[1]["lines"], json!([second, first]));
        assert_eq!(angles[1]["degrees"], json!(60));
    }

    /// A dimension from a copied point to a point left out, and a fix on a copied point, stay
    /// with the original: the copy is held by its own length only.
    #[test]
    fn what_ties_to_outside_or_fixes_is_not_copied() {
        let mut e = editor(
            json!({"points": {"s": [0, -5], "t": [0, 20], "a": [10, 0], "b": [10, 10], "z": [30, 30]},
                   "lines": {"axis": {"from": "s", "to": "t", "construction": true}, "ab": {"from": "a", "to": "b"}},
                   "constraints": [{"type": "vertical", "line": "axis"}, {"type": "fix", "point": "a"}, {"type": "length", "line": "ab", "value": 10},
                                   {"type": "distance", "points": ["a", "z"], "value": 28.284271247461902}]}),
            false,
        );
        pick(&mut e, &["ab"]);
        e.start_mirror();
        click_at(&mut e, [0.0, 0.0], false).unwrap();
        assert!(e.placed.is_ok(), "{:?}", e.placed.as_ref().err());
        let copy = fresh(&e, "lines", &["axis", "ab"]);
        let [l] = &copy[..] else { panic!("one copy: {copy:?}") };
        let pts = fresh(&e, "points", &["s", "t", "a", "b", "z"]);
        assert_eq!(pts.len(), 2);
        assert_eq!(cons(&e, "fix").len(), 1, "the original's only");
        assert_eq!(cons(&e, "distance").len(), 1, "the original's only");
        assert!(cons(&e, "distance").iter().all(|c| c["points"] == json!(["a", "z"])));
        assert_eq!(cons(&e, "length").iter().filter(|c| c["line"] == json!(l)).count(), 1);
    }
}
