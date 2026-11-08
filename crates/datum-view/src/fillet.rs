//! Fillet and chamfer in a sketch (D240): one tool (F), Tab between the two, the size typed in
//! the field by the cursor and kept from one click to the next. A click on a corner, a point
//! where two of the sketch's own lines or arcs meet, rounds or bevels it; a click inside a closed
//! region does every corner of its outline, the fillets of one radius (`radius` on the first,
//! `equal_radius` to it on the rest), the chamfers of one size.
//!
//! The corner point stays, on both curves (`on`): a virtual sharp, so the dimensions and the fix
//! that hold the corner hold it still. A fillet is an arc tangent to both curves, its centre where
//! the curves moved in by the radius cross; a chamfer a line between the points the size away
//! from the corner along each curve (`distance` from the corner). Corners already smooth (the
//! curves tangent) or straight are left; a size the curves are too short for does nothing.

use std::collections::BTreeMap;

use serde_json::{Value as Json, json};

use crate::draw::Patch;
use crate::sketch::SketchEdit;
use crate::trim::{Plan, Shape, Sketch, cross, crossings, dist, dot, sub};

/// What a click at pixel offset `(dx, dy)` would do with size `size` (as typed and its value):
/// round (or, `chamfer`, bevel) the corner under the cursor, or every corner of the region it is
/// in. None over nothing it can change.
pub fn plan(edit: &SketchEdit, dx: f64, dy: f64, height: f64, size: &(Json, f64), chamfer: bool) -> Option<Plan> {
    let p = edit.placed.as_ref().ok()?;
    let sk = Sketch::new(edit, p)?;
    let corners = match edit.point_at(dx, dy, height) {
        Some(v) => vec![corner_at(&sk, &v)?],
        None => {
            let picks = edit.contour_picks(dx, dy, height);
            let curves: Vec<String> = picks.into_iter().filter_map(|k| if let crate::sketch::Pick::Curve(n) = k { Some(n) } else { None }).collect();
            corners_of(&sk, &curves)
        }
    };
    if corners.is_empty() {
        return None;
    }
    make(&sk, &corners, size, chamfer)
}

/// End `end` (`from` or `to`) of curve `curve`, in map `key`.
#[derive(Clone, Debug, PartialEq)]
struct End {
    curve: String,
    key: &'static str,
    end: &'static str,
}

/// Two curves meeting at point `at` (the first's end there).
#[derive(Clone, Debug, PartialEq)]
struct Corner {
    at: String,
    ends: [End; 2],
}

/// The own lines and arcs that end at point `v` (or at one coincident with it).
fn ends_at(sk: &Sketch, v: &str) -> Vec<End> {
    let json = sk.json();
    let mut out = Vec::new();
    for key in ["lines", "arcs"] {
        for (n, c) in json[key].as_object().into_iter().flatten() {
            for end in ["from", "to"] {
                if c[end].as_str().is_some_and(|e| sk.same(e, v)) {
                    out.push(End { curve: n.clone(), key, end });
                }
            }
        }
    }
    out
}

/// The corner at point `v`: exactly two curves end there, neither tangent to the other.
fn corner_at(sk: &Sketch, v: &str) -> Option<Corner> {
    let [a, b]: [End; 2] = ends_at(sk, v).try_into().ok()?;
    if a.curve == b.curve || sk.tangent(&a.curve, &b.curve) {
        return None;
    }
    Some(Corner { at: v.to_string(), ends: [a, b] })
}

/// The corners of the outline made of `curves`: where two of them meet, in the order of their
/// points' names.
fn corners_of(sk: &Sketch, curves: &[String]) -> Vec<Corner> {
    let json = sk.json();
    let mut points = std::collections::BTreeSet::new();
    for n in curves {
        for key in ["lines", "arcs"] {
            for end in ["from", "to"] {
                if let Some(e) = json[key][n][end].as_str() {
                    points.insert(e.to_string());
                }
            }
        }
    }
    points
        .iter()
        .filter_map(|v| corner_at(sk, v))
        .filter(|c| c.ends.iter().all(|e| curves.contains(&e.curve)))
        // Coincident points name one corner twice: the first name keeps it.
        .fold(Vec::new(), |mut out: Vec<Corner>, c| {
            if !out.iter().any(|o| o.ends == c.ends) {
                out.push(c);
            }
            out
        })
}

/// The direction curve `s` leaves end `end` in, into itself, as a unit vector.
fn leaving(s: &Shape, end: &str) -> [f64; 2] {
    let d = match *s {
        Shape::Line(a, b) if end == "from" => sub(b, a),
        Shape::Line(a, b) => sub(a, b),
        Shape::Round { start, sweep, .. } => {
            let t = if end == "from" { start } else { start + sweep };
            let (sn, cs) = libm::sincos(t);
            if end == "from" { [-sn, cs] } else { [sn, -cs] }
        }
    };
    let l = libm::hypot(d[0], d[1]);
    [d[0] / l, d[1] / l]
}

/// Where along curve `s` end `end` is: 0 or 1.
fn end_t(end: &str) -> f64 {
    if end == "from" { 0.0 } else { 1.0 }
}

/// Curve `s` moved by `r` to the side `toward` points to from its end at `v`: a line along
/// itself, an arc's circle in or out.
fn moved(s: &Shape, v: [f64; 2], u: [f64; 2], toward: [f64; 2], r: f64) -> Option<Shape> {
    match *s {
        Shape::Line(a, b) => {
            let mut n = [-u[1], u[0]];
            if dot(n, toward) < 0.0 {
                n = [-n[0], -n[1]];
            }
            Some(Shape::Line([a[0] + n[0] * r, a[1] + n[1] * r], [b[0] + n[0] * r, b[1] + n[1] * r]))
        }
        Shape::Round { c, r: big, .. } => {
            let inward = dot(sub(c, v), toward) > 0.0;
            let r2 = if inward { big - r } else { big + r };
            (r2 > 0.0).then_some(Shape::Round { c, r: r2, start: 0.0, sweep: std::f64::consts::TAU, closed: true })
        }
    }
}

/// The point of curve `s` nearest `f` on its line or circle (where an arc about `f` touches it).
fn foot(s: &Shape, f: [f64; 2]) -> [f64; 2] {
    match *s {
        Shape::Line(a, b) => {
            let u = sub(b, a);
            let t = dot(sub(f, a), u) / dot(u, u);
            [a[0] + u[0] * t, a[1] + u[1] * t]
        }
        Shape::Round { c, r, .. } => {
            let d = sub(f, c);
            let l = libm::hypot(d[0], d[1]);
            [c[0] + d[0] / l * r, c[1] + d[1] / l * r]
        }
    }
}

/// The point of curve `s` `d` away from its end `v`, along it the way `u` leaves.
fn along(s: &Shape, v: [f64; 2], u: [f64; 2], d: f64) -> Option<[f64; 2]> {
    match *s {
        Shape::Line(..) => Some([v[0] + u[0] * d, v[1] + u[1] * d]),
        Shape::Round { .. } => {
            let ring = Shape::Round { c: v, r: d, start: 0.0, sweep: std::f64::consts::TAU, closed: true };
            crossings(&ring, s).into_iter().find(|&x| dot(sub(x, v), u) > 0.0)
        }
    }
}

/// One corner done: where each curve ends now (along it, and where), and what goes in between:
/// a fillet's centre and the arc's ends in counterclockwise order, or none for a chamfer.
struct Done {
    cut: [(f64, [f64; 2]); 2],
    center: Option<[f64; 2]>,
}

fn corner(sk: &Sketch, c: &Corner, r: f64, chamfer: bool) -> Option<Done> {
    let shapes = [Shape::of(sk.p, &c.ends[0].curve)?, Shape::of(sk.p, &c.ends[1].curve)?];
    let v = sk.at(&c.at)?;
    let u = [leaving(&shapes[0], c.ends[0].end), leaving(&shapes[1], c.ends[1].end)];
    // Straight on, or back along itself: no corner to round.
    if cross(u[0], u[1]) == 0.0 {
        return None;
    }
    let (points, center) = if chamfer {
        ([along(&shapes[0], v, u[0], r)?, along(&shapes[1], v, u[1], r)?], None)
    } else {
        let m = [moved(&shapes[0], v, u[0], u[1], r)?, moved(&shapes[1], v, u[1], u[0], r)?];
        let f = crossings(&m[0], &m[1]).into_iter().min_by(|a, b| dist(*a, v).total_cmp(&dist(*b, v)))?;
        ([foot(&shapes[0], f), foot(&shapes[1], f)], Some(f))
    };
    let mut cut = [(0.0, [0.0; 2]); 2];
    for k in 0..2 {
        let (s, x) = (&shapes[k], points[k]);
        // On the curve, the corner's side of its other end.
        if !s.holds(x) || dot(sub(x, v), u[k]) <= 0.0 {
            return None;
        }
        cut[k] = (s.param(x), x);
    }
    Some(Done { cut, center })
}

/// The patch doing `corners` with size `size`, and what it takes away and adds, to show.
fn make(sk: &Sketch, corners: &[Corner], size: &(Json, f64), chamfer: bool) -> Option<Plan> {
    let done: Vec<Done> = corners.iter().map(|c| corner(sk, c, size.1, chamfer)).collect::<Option<_>>()?;
    let mut patch = Patch::new(sk.edit);
    // The curves as they end now: one may lose both its corners.
    let mut curves: BTreeMap<(&str, String), Json> = BTreeMap::new();
    let mut cuts: BTreeMap<String, Vec<(f64, &str)>> = BTreeMap::new();
    let (mut gone, mut added) = (Vec::new(), Vec::new());
    let mut first_arc: Option<String> = None;
    for (c, d) in corners.iter().zip(&done) {
        let mut ends = Vec::new();
        let mut corner_gone = Vec::new();
        for (k, e) in c.ends.iter().enumerate() {
            let q = patch.fresh("p");
            patch.set("points", &q, json!(d.cut[k].1));
            let json = curves.entry((e.key, e.curve.clone())).or_insert_with(|| sk.json()[e.key][&e.curve].clone());
            json[e.end] = json!(q);
            cuts.entry(e.curve.clone()).or_default().push((d.cut[k].0, e.end));
            // The corner stays where the curves would meet: a virtual sharp.
            patch.constrain(json!({"type": "on", "point": c.at, "curve": e.curve}));
            let s = Shape::of(sk.p, &e.curve)?;
            let mut piece = s.polyline(d.cut[k].0, end_t(e.end));
            if k == 1 {
                piece.reverse();
            }
            corner_gone.append(&mut piece);
            ends.push(q);
        }
        gone.push(corner_gone);
        match d.center {
            None => {
                let l = patch.fresh("l");
                patch.set("lines", &l, json!({"from": ends[0], "to": ends[1]}));
                for q in &ends {
                    patch.constrain(json!({"type": "distance", "points": [c.at, q], "value": size.0}));
                }
                added.push(vec![d.cut[0].1, d.cut[1].1]);
            }
            Some(f) => {
                let center = patch.fresh("p");
                patch.set("points", &center, json!(f));
                // Counterclockwise the short way round.
                let ccw = cross(sub(d.cut[0].1, f), sub(d.cut[1].1, f)) > 0.0;
                let (from, to) = if ccw { (&ends[0], &ends[1]) } else { (&ends[1], &ends[0]) };
                let a = patch.fresh("a");
                patch.set("arcs", &a, json!({"center": center, "from": from, "to": to}));
                for e in &c.ends {
                    patch.constrain(json!({"type": "tangent", "curves": [e.curve, a]}));
                }
                match &first_arc {
                    None => patch.constrain(json!({"type": "radius", "arc": a, "value": size.0})),
                    Some(f) => patch.constrain(json!({"type": "equal_radius", "curves": [f, a]})),
                }
                first_arc.get_or_insert(a.clone());
                let (fa, ta) = (if ccw { d.cut[0].1 } else { d.cut[1].1 }, if ccw { d.cut[1].1 } else { d.cut[0].1 });
                let start = libm::atan2(fa[1] - f[1], fa[0] - f[0]);
                let sweep = (libm::atan2(ta[1] - f[1], ta[0] - f[0]) - start).rem_euclid(std::f64::consts::TAU);
                added.push(Shape::Round { c: f, r: dist(fa, f), start, sweep, closed: false }.polyline(0.0, 1.0));
            }
        }
    }
    // A curve rounded at both ends keeps a piece between them.
    for v in cuts.values() {
        if let [(a, ea), (b, _)] = v[..] {
            let (start, end) = if ea == "from" { (a, b) } else { (b, a) };
            if start >= end {
                return None;
            }
        }
    }
    for ((key, n), v) in curves {
        patch.set(key, &n, v);
    }
    Some(Plan { patch: patch.done()?, gone, added })
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use serde_json::{Value as Json, json};

    use crate::draw::Tool;
    use crate::draw::tests::{click_at, editor};
    use crate::sketch::SketchEdit;

    /// A 20 x 10 rectangle held by its corner `a` and two dimensions.
    fn rectangle() -> SketchEdit {
        let mut e = editor(
            json!({"points": {"a": [0, 0], "b": [20, 0], "c": [20, 10], "d": [0, 10]},
                   "lines": {"bottom": {"from": "a", "to": "b"}, "right": {"from": "b", "to": "c"}, "top": {"from": "c", "to": "d"}, "left": {"from": "d", "to": "a"}},
                   "constraints": [{"type": "fix", "point": "a"},
                                   {"type": "horizontal_distance", "points": ["a", "b"], "value": 20},
                                   {"type": "vertical_distance", "points": ["a", "d"], "value": 10}]}),
            false,
        );
        e.draft.tool = Tool::Fillet;
        e.draft.fillet = Some((json!(3), 3.0));
        e
    }

    /// Where point `n` of the solved sketch is, rounded to a millionth.
    fn at(e: &SketchEdit, n: &str) -> [f64; 2] {
        let v = e.placed.as_ref().unwrap().solved.points[n];
        [(v.x * 1e6).round() / 1e6, (v.y * 1e6).round() / 1e6]
    }

    fn round6(v: f64) -> f64 {
        (v * 1e6).round() / 1e6
    }

    /// Where the ends of curve `n` in `key` are.
    fn ends(e: &SketchEdit, key: &str, n: &str) -> [[f64; 2]; 2] {
        let c = &e.json["sketch"][key][n];
        [at(e, c["from"].as_str().unwrap()), at(e, c["to"].as_str().unwrap())]
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

    fn constraints(e: &SketchEdit, kind: &str) -> Vec<Json> {
        e.json["sketch"]["constraints"].as_array().unwrap().iter().filter(|c| c["type"] == kind).cloned().collect()
    }

    fn has(e: &SketchEdit, c: Json) -> bool {
        e.json["sketch"]["constraints"].as_array().unwrap().contains(&c)
    }

    /// The radius of arc `n` in the solution.
    fn radius(e: &SketchEdit, n: &str) -> f64 {
        let a = &e.json["sketch"]["arcs"][n];
        let (c, f) = (at(e, a["center"].as_str().unwrap()), at(e, a["from"].as_str().unwrap()));
        round6(libm::hypot(f[0] - c[0], f[1] - c[1]))
    }

    /// A click on the right-hand bottom corner of the rectangle rounds it: an arc tangent to the
    /// bottom and the right side, the corner point still there, on both, the dimensions kept.
    #[test]
    fn one_corner_is_rounded() {
        let mut e = rectangle();
        let before = e.json["sketch"].clone();
        click_at(&mut e, [20.0, 0.0], false).unwrap();
        assert!(e.placed.is_ok(), "{:?}", e.placed.as_ref().err());
        let arcs = new_ones(&e, "arcs", &before);
        let [arc] = &arcs[..] else { panic!("one arc: {arcs:?}") };
        let a = &e.json["sketch"]["arcs"][arc];
        assert_eq!(at(&e, a["center"].as_str().unwrap()), [17.0, 3.0]);
        let mut tips = [at(&e, a["from"].as_str().unwrap()), at(&e, a["to"].as_str().unwrap())];
        tips.sort_by(|p, q| p.partial_cmp(q).unwrap());
        assert_eq!(tips, [[17.0, 0.0], [20.0, 3.0]]);
        assert_eq!(ends(&e, "lines", "bottom")[1], [17.0, 0.0]);
        assert_eq!(ends(&e, "lines", "right")[0], [20.0, 3.0]);
        assert_eq!(at(&e, "b"), [20.0, 0.0], "the corner stays");
        assert!(has(&e, json!({"type": "on", "point": "b", "curve": "bottom"})));
        assert!(has(&e, json!({"type": "on", "point": "b", "curve": "right"})));
        assert!(has(&e, json!({"type": "horizontal_distance", "points": ["a", "b"], "value": 20})));
        assert!(has(&e, json!({"type": "vertical_distance", "points": ["a", "d"], "value": 10})));
        assert!(has(&e, json!({"type": "tangent", "curves": ["bottom", arc]})));
        assert!(has(&e, json!({"type": "tangent", "curves": ["right", arc]})));
        assert!(has(&e, json!({"type": "radius", "arc": arc, "value": 3})));
    }

    /// A click inside the rectangle rounds all four corners: one radius to type, the rest equal
    /// to it; the region is still filled.
    #[test]
    fn a_click_inside_rounds_every_corner() {
        let mut e = rectangle();
        let before = e.json["sketch"].clone();
        click_at(&mut e, [10.0, 5.0], false).unwrap();
        assert!(e.placed.is_ok(), "{:?}", e.placed.as_ref().err());
        let arcs = new_ones(&e, "arcs", &before);
        assert_eq!(arcs.len(), 4, "{arcs:?}");
        let radii = constraints(&e, "radius");
        let [first] = &radii[..] else { panic!("one radius: {radii:?}") };
        assert_eq!(first["value"], json!(3));
        let first = first["arc"].as_str().unwrap();
        let equal = constraints(&e, "equal_radius");
        assert_eq!(equal.len(), 3, "{equal:?}");
        for c in &equal {
            assert_eq!(c["curves"][0], json!(first), "{c}");
        }
        assert!(!e.fill.is_empty(), "still a region");
        for a in &arcs {
            assert_eq!(radius(&e, a), 3.0, "{a}");
        }
    }

    /// The radius is a parameter, `r`: the constraint keeps its name, not the number.
    #[test]
    fn the_size_may_be_an_expression() {
        let mut e = rectangle();
        e.params.insert("r".into(), 3.0);
        e.solve();
        e.draft.fillet = Some((json!("r"), 3.0));
        click_at(&mut e, [20.0, 0.0], false).unwrap();
        assert!(e.placed.is_ok(), "{:?}", e.placed.as_ref().err());
        let radii = constraints(&e, "radius");
        let [r] = &radii[..] else { panic!("one radius: {radii:?}") };
        assert_eq!(r["value"], json!("r"));
    }

    /// Tab turned the tool to chamfers, size 2: a click on the corner cuts it with a straight line
    /// 2 along each side from the corner, no arc.
    #[test]
    fn one_corner_is_chamfered() {
        let mut e = rectangle();
        e.draft.chamfer = true;
        e.draft.fillet = Some((json!(2), 2.0));
        let before = e.json["sketch"].clone();
        click_at(&mut e, [20.0, 0.0], false).unwrap();
        assert!(e.placed.is_ok(), "{:?}", e.placed.as_ref().err());
        assert!(new_ones(&e, "arcs", &before).is_empty());
        let lines = new_ones(&e, "lines", &before);
        let [l] = &lines[..] else { panic!("one line: {lines:?}") };
        let mut tips = ends(&e, "lines", l);
        tips.sort_by(|p, q| p.partial_cmp(q).unwrap());
        assert_eq!(tips, [[18.0, 0.0], [20.0, 2.0]]);
        let distances: Vec<Json> = constraints(&e, "distance");
        assert_eq!(distances.len(), 2, "{distances:?}");
        for d in &distances {
            assert_eq!((d["points"][0].as_str(), &d["value"]), (Some("b"), &json!(2)), "{d}");
        }
    }

    /// A chamfer of size 2 by a click inside: four new lines, two distances each.
    #[test]
    fn every_corner_is_chamfered() {
        let mut e = rectangle();
        e.draft.chamfer = true;
        e.draft.fillet = Some((json!(2), 2.0));
        let before = e.json["sketch"].clone();
        click_at(&mut e, [10.0, 5.0], false).unwrap();
        assert!(e.placed.is_ok(), "{:?}", e.placed.as_ref().err());
        assert_eq!(new_ones(&e, "lines", &before).len(), 4);
        assert_eq!(constraints(&e, "distance").len(), 8);
        assert!(!e.fill.is_empty(), "still a region");
    }

    /// A radius of 11 on a corner whose side is 10 long does not fit: the click does nothing.
    #[test]
    fn a_radius_too_large_does_nothing() {
        let mut e = rectangle();
        e.draft.fillet = Some((json!(11), 11.0));
        let before = e.json.clone();
        assert!(click_at(&mut e, [20.0, 0.0], false).is_none());
        assert_eq!(e.json, before);
    }

    /// The corner of a line and an arc: a half circle on a diameter, its right end rounded with R2,
    /// the fillet tangent to the circle inside and to the line.
    #[test]
    fn a_line_and_an_arc_meet_in_a_fillet() {
        let mut e = editor(
            json!({"points": {"o": [0, 0], "l": [-10, 0], "r": [10, 0]},
                   "lines": {"bottom": {"from": "l", "to": "r"}},
                   "arcs": {"cap": {"center": "o", "from": "r", "to": "l"}}}),
            false,
        );
        e.draft.tool = Tool::Fillet;
        e.draft.fillet = Some((json!(2), 2.0));
        let before = e.json["sketch"].clone();
        click_at(&mut e, [10.0, 0.0], false).unwrap();
        assert!(e.placed.is_ok(), "{:?}", e.placed.as_ref().err());
        let arcs = new_ones(&e, "arcs", &before);
        let [arc] = &arcs[..] else { panic!("one arc: {arcs:?}") };
        let a = &e.json["sketch"]["arcs"][arc];
        let s = round6(60f64.sqrt());
        assert_eq!(at(&e, a["center"].as_str().unwrap()), [s, 2.0]);
        let mut tips = [at(&e, a["from"].as_str().unwrap()), at(&e, a["to"].as_str().unwrap())];
        tips.sort_by(|p, q| p.partial_cmp(q).unwrap());
        assert_eq!(tips, [[s, 0.0], [round6(9.682_458_365_5), 2.5]]);
        assert!(has(&e, json!({"type": "tangent", "curves": ["bottom", arc]})));
        assert!(has(&e, json!({"type": "tangent", "curves": ["cap", arc]})));
    }

    /// A slot already smooth at all four joints: a click inside has no corner to round.
    #[test]
    fn a_slot_has_no_corners() {
        let mut e = editor(
            json!({"points": {"c1": [0, 0], "c2": [20, 0], "a": [0, -5], "b": [20, -5], "c": [20, 5], "d": [0, 5]},
                   "lines": {"bottom": {"from": "a", "to": "b"}, "top": {"from": "c", "to": "d"}},
                   "arcs": {"right": {"center": "c2", "from": "b", "to": "c"}, "left": {"center": "c1", "from": "d", "to": "a"}},
                   "constraints": [{"type": "tangent", "curves": ["bottom", "right"]}, {"type": "tangent", "curves": ["right", "top"]},
                                   {"type": "tangent", "curves": ["top", "left"]}, {"type": "tangent", "curves": ["left", "bottom"]}]}),
            false,
        );
        e.draft.tool = Tool::Fillet;
        e.draft.fillet = Some((json!(1), 1.0));
        let before = e.json.clone();
        assert!(click_at(&mut e, [10.0, 0.0], false).is_none());
        assert_eq!(e.json, before);
    }

    /// The right side of a rectangle rounded at both its corners, dragged left frame by frame as
    /// a hand drags it: the fillets go with it, each still a quarter turn inside the corner, not
    /// turned inside out round the line's end (sketch_test2: the solver started each frame from
    /// where the fillet was drawn, nearer the inside-out placement the further it went).
    #[test]
    fn fillets_follow_a_dragged_side() {
        let mut e = rectangle();
        // Its width free, for the side to move.
        let width = json!({"type": "horizontal_distance", "points": ["a", "b"], "value": 20});
        e.apply(serde_json::from_value(json!({"remove_constraints": [width]})).unwrap());
        click_at(&mut e, [20.0, 0.0], false).unwrap();
        click_at(&mut e, [20.0, 10.0], false).unwrap();
        e.draft.tool = Tool::Select;
        e.hover = Some(crate::sketch::Pick::Curve("right".into()));
        assert!(e.start_drag([20.0, 5.0]));
        for k in 1..=40 {
            e.drag_to([20.0 - 0.25 * k as f64, 5.0], 0.0);
        }
        e.end_drag();
        assert!(e.placed.is_ok(), "{:?}", e.placed.as_ref().err());
        assert_eq!(ends(&e, "lines", "right")[0][0], 10.0, "the side went to the cursor");
        for a in ["a1", "a2"] {
            let arc = &e.json["sketch"]["arcs"][a];
            let (c, f, t) = (at(&e, arc["center"].as_str().unwrap()), at(&e, arc["from"].as_str().unwrap()), at(&e, arc["to"].as_str().unwrap()));
            let turn = (libm::atan2(t[1] - c[1], t[0] - c[0]) - libm::atan2(f[1] - c[1], f[0] - c[0])).rem_euclid(std::f64::consts::TAU);
            assert!((turn - std::f64::consts::FRAC_PI_2).abs() < 1e-6, "{a} turns {turn}");
            assert!(c[0] < 10.0, "{a}'s centre inside the corner: {c:?}");
        }
    }
}
