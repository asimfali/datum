//! Offset in a sketch (D245): a chain of the sketch's own lines and arcs, joined end to end, or
//! one circle, copied at a distance (O; Kompas calls it an equidistant). The size is typed in
//! the field by the cursor and kept from one click to the next; the cursor's side of the curve
//! under it is the side, Shift both. The copy follows the original: each line `parallel` to its
//! own at `point_line` the size, each arc about the same centre, a circle's diameter the
//! original's and twice the size; where the original turns smoothly (`tangent`) the copy does,
//! where it turns at a corner the copy's curves meet where they cross. A size the chain cannot
//! take (an arc turned inside out, curves that no longer cross) does nothing.

use std::collections::BTreeMap;

use serde_json::{Value as Json, json};

use crate::draw::Patch;
use crate::sketch::SketchEdit;
use crate::trim::{Plan, Shape, Sketch, cross, crossings, dist, sub};

/// What a click at pixel offset `(dx, dy)` would do with size `size` (as typed, and its value):
/// the chain under the cursor copied on its side, or on both sides if `both`.
pub fn plan(edit: &SketchEdit, dx: f64, dy: f64, height: f64, size: &(Json, f64), both: bool) -> Option<Plan> {
    if size.1 <= 0.0 {
        return None;
    }
    let p = edit.placed.as_ref().ok()?;
    let sk = Sketch::new(edit, p)?;
    let name = edit.curve_at(dx, dy, height)?;
    let at = edit.camera.at(dx, dy, height);
    let mut patch = Patch::new(edit);
    let mut added = Vec::new();
    if sk.json()["circles"].get(&name).is_some() {
        let s = Shape::of(p, &name)?;
        let Shape::Round { c, r, .. } = s else { return None };
        let outside = dist(at, c) > r;
        for sign in sides(outside, both) {
            added.push(circle(&sk, &mut patch, &name, size, sign)?);
        }
        return Some(Plan { patch: patch.done()?, gone: Vec::new(), added });
    }
    let chain = chain(&sk, &name)?;
    // The side: left of the way the chain runs, where the cursor is from the curve under it.
    let k = chain.iter().position(|s| s.curve == name)?;
    let s = &chain[k];
    let shape = Shape::of(p, &s.curve)?;
    let t = shape.param(at);
    let t = match shape {
        Shape::Line(..) => t.clamp(0.0, 1.0),
        _ => t.min(1.0),
    };
    let near = shape.at(t);
    let ahead = shape.at(if s.forward { (t + 1e-3).min(1.0) } else { (t - 1e-3).max(0.0) });
    let behind = shape.at(if s.forward { (t - 1e-3).max(0.0) } else { (t + 1e-3).min(1.0) });
    let left = cross(sub(ahead, behind), sub(at, near)) > 0.0;
    for sign in sides(left, both) {
        added.extend(copy_chain(&sk, &mut patch, &chain, size, sign)?);
    }
    Some(Plan { patch: patch.done()?, gone: Vec::new(), added })
}

/// The sides offset to: `first` (+1) or the other (−1), or both.
fn sides(first: bool, both: bool) -> Vec<f64> {
    let s = if first { 1.0 } else { -1.0 };
    if both { vec![s, -s] } else { vec![s] }
}

/// A curve of a chain, in the way the chain runs.
#[derive(Clone, Debug)]
struct Step {
    curve: String,
    key: &'static str,
    forward: bool,
    start: String,
    end: String,
}

/// The chain of the sketch's own lines and arcs through `name`: on from each end while exactly
/// one other curve ends there, back to `name` if it closes. Construction curves chain only with
/// their kind.
fn chain(sk: &Sketch, name: &str) -> Option<Vec<Step>> {
    let json = sk.json();
    let mut curves: BTreeMap<&str, (&'static str, String, String, bool)> = BTreeMap::new();
    for key in ["lines", "arcs"] {
        for (n, c) in json[key].as_object().into_iter().flatten() {
            curves.insert(n, (key, c["from"].as_str()?.to_string(), c["to"].as_str()?.to_string(), c["construction"] == json!(true)));
        }
    }
    let (key, from, to, construction) = curves.get(name)?.clone();
    let at_point = |q: &str, not: &str| -> Vec<&str> {
        curves.iter().filter(|(n, c)| **n != not && c.3 == construction && (sk.same(&c.1, q) || sk.same(&c.2, q))).map(|(n, _)| *n).collect()
    };
    let step = |n: &str, start: &str| -> Option<Step> {
        let (key, f, t, _) = curves.get(n)?.clone();
        let forward = sk.same(&f, start);
        Some(Step { curve: n.to_string(), key, forward, start: if forward { f.clone() } else { t.clone() }, end: if forward { t } else { f } })
    };
    let mut steps = vec![Step { curve: name.to_string(), key, forward: true, start: from, end: to }];
    // Ahead from the end.
    loop {
        let last = steps.last()?.clone();
        let next = at_point(&last.end, &last.curve);
        let [n] = next[..] else { break };
        if n == steps[0].curve {
            break;
        }
        if steps.iter().any(|s| s.curve == n) {
            break;
        }
        steps.push(step(n, &last.end)?);
    }
    let closed = sk.same(&steps.last()?.end, &steps[0].start) && steps.len() > 1;
    // Back from the start, unless it closed.
    if !closed {
        loop {
            let first = steps[0].clone();
            let next = at_point(&first.start, &first.curve);
            let [n] = next[..] else { break };
            if steps.iter().any(|s| s.curve == n) {
                break;
            }
            let s = step(n, &first.start)?;
            // Run it the chain's way: it ends where the chain starts.
            steps.insert(0, Step { forward: !s.forward, start: s.end, end: s.start, ..s });
        }
    }
    Some(steps)
}

/// Circle `name` copied `sign` (+1 outside) at `size`: about the same centre, its diameter the
/// original's and twice the size.
fn circle(sk: &Sketch, patch: &mut Patch, name: &str, size: &(Json, f64), sign: f64) -> Option<Vec<[f64; 2]>> {
    let c = &sk.json()["circles"][name];
    let Shape::Round { c: centre, r, .. } = Shape::of(sk.p, name)? else { return None };
    let r2 = r + sign * size.1;
    if r2 <= 0.0 {
        return None;
    }
    let diameter = shifted(&c["diameter"], &size.0, 2.0 * sign)?;
    let n = patch.fresh("c");
    let mut v = c.clone();
    v["diameter"] = diameter;
    patch.set("circles", &n, v);
    Some(Shape::Round { c: centre, r: r2, start: 0.0, sweep: std::f64::consts::TAU, closed: true }.polyline(0.0, 1.0))
}

/// `value` and `k` times `size`: numbers added, else an expression.
fn shifted(value: &Json, size: &Json, k: f64) -> Option<Json> {
    Some(match (value.as_f64(), size.as_f64()) {
        (Some(v), Some(s)) => json!(v + k * s),
        _ => {
            let show = |j: &Json| j.as_str().map(|e| format!("({e})")).or_else(|| j.as_f64().map(|x| x.to_string()));
            json!(format!("{} + {} * {}", show(value)?, k, show(size)?))
        }
    })
}

/// The chain copied `sign` (+1 left of its way) at `size`: the patch's curves and constraints,
/// and the copy as polylines.
fn copy_chain(sk: &Sketch, patch: &mut Patch, chain: &[Step], size: &(Json, f64), sign: f64) -> Option<Vec<Vec<[f64; 2]>>> {
    let d = size.1 * sign;
    // Each curve moved, as a shape in the chain's way, and where its ends go.
    let mut moved = Vec::new();
    for s in chain {
        let shape = Shape::of(sk.p, &s.curve)?;
        let m = match shape {
            Shape::Line(a, b) => {
                let (a, b) = if s.forward { (a, b) } else { (b, a) };
                let u = sub(b, a);
                let l = libm::hypot(u[0], u[1]);
                let n = [-u[1] / l * d, u[0] / l * d];
                Shape::Line([a[0] + n[0], a[1] + n[1]], [b[0] + n[0], b[1] + n[1]])
            }
            Shape::Round { c, r, start, sweep, closed } => {
                // Counterclockwise, its left is its centre.
                let r2 = if s.forward { r - d } else { r + d };
                if r2 <= 0.0 {
                    return None;
                }
                Shape::Round { c, r: r2, start, sweep, closed }
            }
        };
        moved.push(m);
    }
    let ends = |m: &Shape, s: &Step| -> ([f64; 2], [f64; 2]) {
        match *m {
            Shape::Line(a, b) => (a, b),
            Shape::Round { .. } if s.forward => (m.at(0.0), m.at(1.0)),
            Shape::Round { .. } => (m.at(1.0), m.at(0.0)),
        }
    };
    let n = chain.len();
    let closed = n > 1 && sk.same(&chain[n - 1].end, &chain[0].start);
    // The joints: where curve k ends and curve k + 1 starts. Smooth ones where the copies meet
    // already; at a corner, where they cross nearest the corner.
    let mut joints: Vec<[f64; 2]> = Vec::new();
    let pairs = if closed { n } else { n - 1 };
    for k in 0..pairs {
        let j = (k + 1) % n;
        let (a, b) = (ends(&moved[k], &chain[k]).1, ends(&moved[j], &chain[j]).0);
        let guess = [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5];
        if sk.tangent(&chain[k].curve, &chain[j].curve) {
            joints.push(guess);
        } else {
            let x = crossings(&moved[k], &moved[j]).into_iter().min_by(|p, q| dist(*p, guess).total_cmp(&dist(*q, guess)))?;
            joints.push(x);
        }
    }
    // The copy's points: a joint each, and the free ends of an open chain.
    let point = |patch: &mut Patch, x: [f64; 2]| {
        let q = patch.fresh("p");
        patch.set("points", &q, json!(x));
        q
    };
    let joint_names: Vec<String> = joints.iter().map(|&x| point(patch, x)).collect();
    let first = if closed { joint_names[n - 1].clone() } else { point(patch, ends(&moved[0], &chain[0]).0) };
    let last = if closed { joint_names[n - 1].clone() } else { point(patch, ends(&moved[n - 1], &chain[n - 1]).1) };
    let mut polylines = Vec::new();
    let mut copies = Vec::new();
    for (k, s) in chain.iter().enumerate() {
        let start = if k == 0 { first.clone() } else { joint_names[k - 1].clone() };
        let end = if k == n - 1 { last.clone() } else { joint_names[k].clone() };
        let orig = &sk.json()[s.key][&s.curve];
        let mut v = orig.clone();
        let (f, t) = if s.forward { (start.clone(), end.clone()) } else { (end.clone(), start.clone()) };
        v["from"] = json!(f);
        v["to"] = json!(t);
        let copy = patch.fresh(if s.key == "lines" { "l" } else { "a" });
        patch.set(s.key, &copy, v);
        match s.key {
            "lines" => {
                patch.constrain(json!({"type": "parallel", "lines": [s.curve, copy]}));
                patch.constrain(json!({"type": "point_line", "point": f, "line": s.curve, "value": size.0}));
            }
            // About the same centre, its radius set by its neighbours where it meets them
            // smoothly; alone, the original's radius and the size.
            _ => {
                let smooth = |o: Option<&Step>| o.is_some_and(|o| sk.tangent(&o.curve, &s.curve) && o.key == "lines");
                let prev = if k > 0 {
                    chain.get(k - 1)
                } else if closed {
                    chain.last()
                } else {
                    None
                };
                let next = if k + 1 < n {
                    chain.get(k + 1)
                } else if closed {
                    chain.first()
                } else {
                    None
                };
                if !(smooth(prev) || smooth(next)) {
                    let radius = sk.typed.iter().zip(&sk.raw).find_map(|(c, raw)| match c {
                        datum_sketch::Constraint::Radius { arc, .. } if *arc == s.curve => Some(raw["value"].clone()),
                        _ => None,
                    });
                    let Shape::Round { r, .. } = moved[k] else { return None };
                    let value = match radius {
                        Some(v) => shifted(&v, &size.0, if s.forward { -sign } else { sign })?,
                        None => json!(r),
                    };
                    patch.constrain(json!({"type": "radius", "arc": copy, "value": value}));
                }
            }
        }
        copies.push(copy);
        let a = match k {
            0 if closed => joints[n - 1],
            0 => ends(&moved[0], s).0,
            _ => joints[k - 1],
        };
        let b = if k == n - 1 && !closed { ends(&moved[k], s).1 } else { joints[k] };
        polylines.push(piece(&moved[k], s, a, b));
    }
    // Smooth where the original is.
    for k in 0..pairs {
        let j = (k + 1) % n;
        if sk.tangent(&chain[k].curve, &chain[j].curve) {
            patch.constrain(json!({"type": "tangent", "curves": [copies[k], copies[j]]}));
        }
    }
    Some(polylines)
}

/// Moved curve `m` of step `s` from `a` to `b` (in the chain's way), as a polyline.
fn piece(m: &Shape, s: &Step, a: [f64; 2], b: [f64; 2]) -> Vec<[f64; 2]> {
    match *m {
        Shape::Line(..) => vec![a, b],
        Shape::Round { c, r, .. } => {
            let (f, t) = if s.forward { (a, b) } else { (b, a) };
            let start = libm::atan2(f[1] - c[1], f[0] - c[0]);
            let sweep = (libm::atan2(t[1] - c[1], t[0] - c[0]) - start).rem_euclid(std::f64::consts::TAU);
            Shape::Round { c, r, start, sweep: if sweep == 0.0 { std::f64::consts::TAU } else { sweep }, closed: false }.polyline(0.0, 1.0)
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use serde_json::{Value as Json, json};

    use crate::draw::Tool;
    use crate::draw::tests::{click_at, editor};
    use crate::sketch::SketchEdit;

    fn rectangle() -> Json {
        json!({"points": {"a": [0, 0], "b": [20, 0], "c": [20, 10], "d": [0, 10]},
               "lines": {"bottom": {"from": "a", "to": "b"}, "right": {"from": "b", "to": "c"}, "top": {"from": "c", "to": "d"}, "left": {"from": "d", "to": "a"}},
               "constraints": [{"type": "fix", "point": "a"}, {"type": "horizontal", "line": "bottom"}, {"type": "vertical", "line": "right"},
                               {"type": "horizontal", "line": "top"}, {"type": "vertical", "line": "left"}]})
    }

    fn chain() -> Json {
        json!({"points": {"a": [0, 0], "b": [10, 0], "o": [10, 5], "c": [15, 5], "d": [15, 15]},
               "lines": {"l1": {"from": "a", "to": "b"}, "l2": {"from": "c", "to": "d"}},
               "arcs": {"r": {"center": "o", "from": "b", "to": "c"}},
               "constraints": [{"type": "tangent", "curves": ["l1", "r"]}, {"type": "tangent", "curves": ["r", "l2"]}]})
    }

    fn offset(sketch: Json, size: f64) -> SketchEdit {
        let mut e = editor(sketch, false);
        e.draft.tool = Tool::Offset;
        e.draft.offset = Some((json!(size), size));
        e
    }

    fn r6(x: f64) -> f64 {
        (x * 1e6).round() / 1e6
    }

    /// The solved copy's points (those the patch added), sorted by name, rounded.
    fn new_points(e: &SketchEdit, patch: &Json) -> Vec<(String, [f64; 2])> {
        let pl = e.placed.as_ref().unwrap();
        patch["points"].as_object().unwrap().keys().map(|n| (n.clone(), [r6(pl.solved.points[n].x), r6(pl.solved.points[n].y)])).collect()
    }

    fn corners(e: &SketchEdit, patch: &Json) -> Vec<[f64; 2]> {
        let mut c: Vec<[f64; 2]> = new_points(e, patch).into_iter().map(|(_, p)| p).collect();
        c.sort_by(|a, b| a.partial_cmp(b).unwrap());
        c
    }

    fn count(patch: &Json, ty: &str) -> usize {
        patch["add_constraints"].as_array().unwrap().iter().filter(|c| c["type"] == ty).count()
    }

    /// With the cursor a hair inside the bottom of a 20 x 10 rectangle and 2 typed, one click
    /// draws a second rectangle 2 inside the first: four lines, each parallel to its own and 2
    /// from it, corners at (2,2), (18,2), (18,8), (2,8); the sketch still solves.
    #[test]
    fn a_rectangle_is_offset_inside() {
        let mut e = offset(rectangle(), 2.0);
        let p = click_at(&mut e, [10.0, 0.3], false).unwrap();
        assert_eq!(p["lines"].as_object().unwrap().len(), 4, "{p}");
        assert_eq!(corners(&e, &p), [[2.0, 2.0], [2.0, 8.0], [18.0, 2.0], [18.0, 8.0]]);
        assert_eq!(count(&p, "parallel"), 4, "{p}");
        let dims: Vec<_> = p["add_constraints"].as_array().unwrap().iter().filter(|c| c["type"] == "point_line").collect();
        assert_eq!(dims.len(), 4, "{p}");
        assert!(dims.iter().all(|c| c["value"].as_f64() == Some(2.0)), "{p}");
        assert!(e.placed.is_ok(), "{:?}", e.placed.as_ref().err());
    }

    /// With the cursor a hair outside the bottom, the copy goes round the outside: corners at
    /// (-2,-2), (22,-2), (22,12), (-2,12).
    #[test]
    fn a_rectangle_is_offset_outside() {
        let mut e = offset(rectangle(), 2.0);
        let p = click_at(&mut e, [10.0, -0.3], false).unwrap();
        assert_eq!(p["lines"].as_object().unwrap().len(), 4, "{p}");
        assert_eq!(corners(&e, &p), [[-2.0, -2.0], [-2.0, 12.0], [22.0, -2.0], [22.0, 12.0]]);
        assert!(e.placed.is_ok(), "{:?}", e.placed.as_ref().err());
    }

    /// Shift on the line-arc-line chain copies it to both sides: six new curves, the two arcs
    /// about the original centre with radii 4 and 6, every smooth joint still tangent in both
    /// copies; the sketch solves.
    #[test]
    fn an_open_chain_is_offset_both_ways() {
        let mut e = offset(chain(), 1.0);
        let p = click_at(&mut e, [5.0, 0.5], true).unwrap();
        let (lines, arcs) = (p["lines"].as_object().unwrap(), p["arcs"].as_object().unwrap());
        assert_eq!((lines.len(), arcs.len()), (4, 2), "{p}");
        assert!(arcs.values().all(|a| a["center"] == json!("o")), "{p}");
        assert_eq!(count(&p, "tangent"), 4, "{p}");
        let pl = e.placed.as_ref().unwrap();
        let mut radii: Vec<f64> = arcs
            .keys()
            .map(|n| {
                let a = &pl.arcs[n];
                let (o, f) = (pl.solved.points["o"], pl.solved.points[&a.from]);
                r6(libm::hypot(f.x - o.x, f.y - o.y))
            })
            .collect();
        radii.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert_eq!(radii, [4.0, 6.0]);
    }

    /// A circle of diameter 10 offset 2 outside: a concentric circle of diameter 14 (a number);
    /// when the diameter is the parameter `d` it is an expression that gives 14 too.
    #[test]
    fn a_circle_is_offset_concentric() {
        let mut e = offset(json!({"points": {"h": [10, 10]}, "circles": {"c1": {"center": "h", "diameter": 10}}}), 2.0);
        let p = click_at(&mut e, [10.0, 15.3], false).unwrap();
        let [(_, copy)] = &p["circles"].as_object().unwrap().iter().collect::<Vec<_>>()[..] else { panic!("one circle: {p}") };
        assert_eq!(copy["center"], json!("h"), "{p}");
        assert_eq!(copy["diameter"], json!(14.0), "{p}");

        let mut e = offset(json!({"points": {"h": [10, 10]}, "circles": {"c1": {"center": "h", "diameter": "d"}}}), 2.0);
        e.refresh(e.context.clone(), e.json.clone(), [("d".to_string(), 10.0)].into());
        let p = click_at(&mut e, [10.0, 15.3], false).unwrap();
        let circles = p["circles"].as_object().unwrap();
        let [(name, copy)] = &circles.iter().collect::<Vec<_>>()[..] else { panic!("one circle: {p}") };
        assert!(copy["diameter"].is_string(), "{p}");
        assert_eq!(copy["center"], json!("h"), "{p}");
        let d = e.placed.as_ref().unwrap().circles[*name].diameter.value().unwrap();
        assert_eq!(r6(d), 14.0, "{p}");
    }

    /// An arc of radius 1 offset 2 inside would turn inside out: the click does nothing and the
    /// sketch stays as it was.
    #[test]
    fn an_arc_too_small_is_not_offset() {
        let sketch = json!({"points": {"o": [10, 10], "f": [11, 10], "t": [10, 11]}, "arcs": {"r": {"center": "o", "from": "f", "to": "t"}}});
        let mut e = offset(sketch, 2.0);
        let before = e.json.clone();
        // A hair inside the arc, at 45 degrees.
        let k = std::f64::consts::FRAC_1_SQRT_2;
        assert!(click_at(&mut e, [10.0 + 0.9 * k, 10.0 + 0.9 * k], false).is_none());
        assert_eq!(e.json, before);
    }

    /// The size stays on the cursor from click to click: two clicks on two different lines of
    /// the rectangle, one typed 2, copy at 2 both times.
    #[test]
    fn the_size_is_kept_from_click_to_click() {
        let mut e = offset(rectangle(), 2.0);
        for q in [[10.0, -0.3], [20.3, 5.0]] {
            let p = click_at(&mut e, q, false).unwrap();
            let dims: Vec<_> = p["add_constraints"].as_array().unwrap().iter().filter(|c| c["type"] == "point_line").collect();
            assert!(!dims.is_empty() && dims.iter().all(|c| c["value"].as_f64() == Some(2.0)), "{p}");
        }
    }
}
