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
use crate::trim::{Plan, Shape, Sketch, cross, crossings, dist, dot, sub};

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
        added.extend(copy_chain(&sk, &mut patch, &chain, size, sign, false)?.0);
    }
    Some(Plan { patch: patch.done()?, gone: Vec::new(), added })
}

/// The middle line (D245) between wall `wall`, picked first, and the chain under pixel offset
/// `(dx, dy)`, as construction geometry (it bounds no region). Two lines alone: the axis one
/// mirrors onto the other, parallel or closing in (`between_lines`). Chains: of one width.
/// Each piece of the first wall moved by the width must lie on the other's (parallel lines,
/// concentric arcs, as near as the sketch can tell, D228); two concentric circles make a circle.
/// It is the first wall's offset by half the width, each straight piece held as the axis of
/// symmetry of a point of one wall and the point across from it on the other: it stays in the
/// middle whatever sets the width, and when the width changes.
pub fn midline(edit: &SketchEdit, wall: &str, dx: f64, dy: f64, height: f64) -> Option<Plan> {
    let p = edit.placed.as_ref().ok()?;
    let sk = Sketch::new(edit, p)?;
    let other = edit.curve_at(dx, dy, height)?;
    if other == wall {
        return None;
    }
    let tol = p.solved.tolerance;
    let json = sk.json();
    let mut patch = Patch::new(edit);
    // Two circles about one centre: the circle between.
    if json["circles"].get(wall).is_some() || json["circles"].get(&other).is_some() {
        let (Some(Shape::Round { c, r, .. }), Some(Shape::Round { c: k, r: q, .. })) = (Shape::of(p, wall), Shape::of(p, &other)) else {
            return None;
        };
        if json["circles"].get(wall).is_none() || json["circles"].get(&other).is_none() || dist(c, k) > tol {
            return None;
        }
        let d = |n: &str| json["circles"][n]["diameter"].clone();
        let diameter = match (d(wall).as_f64(), d(&other).as_f64()) {
            (Some(a), Some(b)) => json!(0.5 * (a + b)),
            _ => json!(format!("({} + {}) / 2", show(&d(wall))?, show(&d(&other))?)),
        };
        let n = patch.fresh("c");
        patch.set("circles", &n, json!({"center": json["circles"][wall]["center"], "diameter": diameter, "construction": true}));
        let added = vec![Shape::Round { c, r: 0.5 * (r + q), start: 0.0, sweep: std::f64::consts::TAU, closed: true }.polyline(0.0, 1.0)];
        return Some(Plan { patch: patch.done()?, gone: Vec::new(), added });
    }
    let (a, b) = (chain(&sk, wall)?, chain(&sk, &other)?);
    if let ([x], [y]) = (&a[..], &b[..])
        && x.key == "lines"
        && y.key == "lines"
    {
        return between_lines(&sk, &mut patch, x, y).and_then(|added| Some(Plan { patch: patch.done()?, gone: Vec::new(), added }));
    }
    let shapes = |c: &[Step]| c.iter().map(|s| Shape::of(p, &s.curve)).collect::<Option<Vec<_>>>();
    let (sa, sb) = (shapes(&a)?, shapes(&b)?);
    // How far apart a piece of the first wall and one of the other are, if they run alike.
    let apart = |x: &Shape, y: &Shape| -> Option<f64> {
        match (*x, *y) {
            (Shape::Line(p0, p1), Shape::Line(q0, q1)) => {
                let u = sub(p1, p0);
                let l = libm::hypot(u[0], u[1]);
                let (d0, d1) = (cross(u, sub(q0, p0)) / l, cross(u, sub(q1, p0)) / l);
                ((d0 - d1).abs() <= tol).then_some(0.5 * (d0 + d1).abs())
            }
            (Shape::Round { c, r, .. }, Shape::Round { c: k, r: q, .. }) => (dist(c, k) <= tol).then_some((r - q).abs()),
            _ => None,
        }
    };
    let width = sb.iter().find_map(|y| apart(&sa[0], y))?;
    if width <= tol || !sa.iter().all(|x| sb.iter().any(|y| apart(x, y).is_some_and(|w| (w - width).abs() <= tol))) {
        return None;
    }
    // Toward the other wall: left of the first's way if a point of the other is.
    let s0 = &a[0];
    let (m, ahead) = (sa[0].at(0.5), sa[0].at(if s0.forward { 0.6 } else { 0.4 }));
    let q = sb.iter().flat_map(|y| y.polyline(0.0, 1.0)).min_by(|x, y| dist(*x, m).total_cmp(&dist(*y, m)))?;
    let sign = if cross(sub(ahead, m), sub(q, m)) > 0.0 { 1.0 } else { -1.0 };
    let (added, copies) = copy_chain(&sk, &mut patch, &a, &(json!(0.5 * width), 0.5 * width), sign, true)?;
    // Each straight piece in the middle whatever the width: the axis of symmetry of its wall's
    // start and the point square across from it on the other wall, held on that wall.
    for (k, x) in sa.iter().enumerate() {
        let Shape::Line(p0, p1) = *x else { continue };
        let Some((n, y)) = b.iter().zip(&sb).find(|(_, y)| apart(x, y).is_some()) else { continue };
        let Shape::Line(q0, q1) = *y else { continue };
        let start = if a[k].forward { p0 } else { p1 };
        let v = sub(q1, q0);
        let t = dot(sub(start, q0), v) / dot(v, v);
        let q = patch.fresh("p");
        patch.set("points", &q, json!([q0[0] + v[0] * t, q0[1] + v[1] * t]));
        patch.constrain(json!({"type": "on", "point": q, "curve": n.curve}));
        patch.constrain(json!({"type": "symmetric", "points": [a[k].start, q], "line": copies[k]}));
    }
    Some(Plan { patch: patch.done()?, gone: Vec::new(), added })
}

/// The middle line of two lines alone: the axis one mirrors onto the other, parallel or not (the
/// bisector between them where they meet), as construction, over the stretch both share along it
/// (or both, if they share none). Held as the axis of symmetry of each end of the first and its
/// mirror, held on the second: it stays the axis however either moves.
fn between_lines(sk: &Sketch, patch: &mut Patch, x: &Step, y: &Step) -> Option<Polylines> {
    let (Shape::Line(a0, a1), Shape::Line(b0, b1)) = (Shape::of(sk.p, &x.curve)?, Shape::of(sk.p, &y.curve)?) else { return None };
    let unit = |u: [f64; 2]| {
        let l = libm::hypot(u[0], u[1]);
        [u[0] / l, u[1] / l]
    };
    let (ua, mut ub) = (unit(sub(a1, a0)), unit(sub(b1, b0)));
    if dot(ua, ub) < 0.0 {
        ub = [-ub[0], -ub[1]];
    }
    let d = unit([ua[0] + ub[0], ua[1] + ub[1]]);
    // A point of the axis: where the lines meet, or, parallel, halfway across.
    let n = cross(ua, ub);
    let o = if n == 0.0 {
        let t = dot(sub(b0, a0), ua);
        let foot = [a0[0] + ua[0] * t, a0[1] + ua[1] * t];
        [(b0[0] + foot[0]) * 0.5, (b0[1] + foot[1]) * 0.5]
    } else {
        let t = cross(sub(b0, a0), ub) / n;
        [a0[0] + ua[0] * t, a0[1] + ua[1] * t]
    };
    // The axis that mirrors the first line onto the second, of the two through `o`.
    let mirror = |q: [f64; 2]| {
        let t = dot(sub(q, o), d);
        [2.0 * (o[0] + d[0] * t) - q[0], 2.0 * (o[1] + d[1] * t) - q[1]]
    };
    let along = |q: [f64; 2]| dot(sub(q, o), d);
    let (sa, sb) = ([along(a0), along(a1)], [along(b0), along(b1)]);
    let (lo, hi) = (sa[0].min(sa[1]).max(sb[0].min(sb[1])), sa[0].max(sa[1]).min(sb[0].max(sb[1])));
    let (lo, hi) = if lo < hi { (lo, hi) } else { (sa[0].min(sa[1]).min(sb[0].min(sb[1])), sa[0].max(sa[1]).max(sb[0].max(sb[1]))) };
    let ends = [[o[0] + d[0] * lo, o[1] + d[1] * lo], [o[0] + d[0] * hi, o[1] + d[1] * hi]];
    let names: Vec<String> = ends
        .iter()
        .map(|e| {
            let q = patch.fresh("p");
            patch.set("points", &q, json!(e));
            q
        })
        .collect();
    let m = patch.fresh("l");
    patch.set("lines", &m, json!({"from": names[0], "to": names[1], "construction": true}));
    let (fa, ta) = (sk.json()["lines"][&x.curve]["from"].as_str()?.to_string(), sk.json()["lines"][&x.curve]["to"].as_str()?.to_string());
    for (end, at) in [(fa, a0), (ta, a1)] {
        let q = patch.fresh("p");
        patch.set("points", &q, json!(mirror(at)));
        patch.constrain(json!({"type": "on", "point": q, "curve": y.curve}));
        patch.constrain(json!({"type": "symmetric", "points": [end, q], "line": m}));
    }
    Some(vec![ends.to_vec()])
}

/// A value as an expression's operand: a number, or an expression in brackets.
fn show(j: &Json) -> Option<String> {
    j.as_str().map(|e| format!("({e})")).or_else(|| j.as_f64().map(|x| x.to_string()))
}

/// Curves as polylines in sketch coordinates.
type Polylines = Vec<Vec<[f64; 2]>>;

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
        _ => json!(format!("{} + {} * {}", show(value)?, k, show(size)?)),
    })
}

/// The chain copied `sign` (+1 left of its way) at `size`: the patch's curves and constraints,
/// and the copy as polylines and its curves' names. A `middle` line: construction, not held at
/// the size.
fn copy_chain(sk: &Sketch, patch: &mut Patch, chain: &[Step], size: &(Json, f64), sign: f64, middle: bool) -> Option<(Polylines, Vec<String>)> {
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
        if middle {
            v["construction"] = json!(true);
        }
        let copy = patch.fresh(if s.key == "lines" { "l" } else { "a" });
        patch.set(s.key, &copy, v);
        match s.key {
            "lines" => {
                patch.constrain(json!({"type": "parallel", "lines": [s.curve, copy]}));
                // A middle line is held between its walls by its caller instead.
                if !middle {
                    patch.constrain(json!({"type": "point_line", "point": f, "line": s.curve, "value": size.0}));
                }
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
    Some((polylines, copies))
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

    /// A channel with an elbow, of one width all along (its walls parallel, its arcs about one
    /// centre), the width a dimension of 4: Tab, a click on one wall, a click on the other — the
    /// middle line, construction, 2 off each wall, its arc of radius 8. The width made 6, it
    /// stays in the middle: 3 off, radius 7.
    #[test]
    fn the_middle_line_stays_in_the_middle() {
        let mut e = editor(
            json!({"points": {"a": [0, 0], "b": [10, 0], "o": [10, 10], "c": [20, 10], "d": [20, 20],
                              "i": [0, 4], "j": [10, 4], "k": [16, 10], "m": [16, 20]},
                   "lines": {"out1": {"from": "a", "to": "b"}, "out2": {"from": "c", "to": "d"}, "in1": {"from": "i", "to": "j"}, "in2": {"from": "k", "to": "m"}},
                   "arcs": {"out": {"center": "o", "from": "b", "to": "c"}, "inr": {"center": "o", "from": "j", "to": "k"}},
                   "constraints": [{"type": "fix", "point": "o"}, {"type": "fix", "point": "a"}, {"type": "horizontal", "line": "out1"}, {"type": "vertical", "line": "out2"},
                                   {"type": "tangent", "curves": ["out1", "out"]}, {"type": "tangent", "curves": ["out", "out2"]},
                                   {"type": "tangent", "curves": ["in1", "inr"]}, {"type": "tangent", "curves": ["inr", "in2"]},
                                   {"type": "parallel", "lines": ["out1", "in1"]}, {"type": "parallel", "lines": ["out2", "in2"]},
                                   {"type": "point_line", "point": "i", "line": "out1", "value": 4}]}),
            false,
        );
        e.draft.tool = Tool::Offset;
        e.draft.midline = true;
        assert!(click_at(&mut e, [5.0, 0.0], false).is_none());
        assert_eq!(e.draft.wall.as_deref(), Some("out1"));
        let patch = click_at(&mut e, [5.0, 4.0], false).unwrap();
        assert!(e.placed.is_ok() && e.draft.wall.is_none());
        assert!(patch["lines"].as_object().unwrap().values().all(|l| l["construction"] == json!(true)));
        let at = |e: &crate::sketch::SketchEdit, n: &str| {
            let v = e.placed.as_ref().unwrap().solved.points[n];
            [(v.x * 1e6).round() / 1e6, (v.y * 1e6).round() / 1e6]
        };
        let line = |e: &crate::sketch::SketchEdit, n: &str| {
            [at(e, e.json["sketch"]["lines"][n]["from"].as_str().unwrap()), at(e, e.json["sketch"]["lines"][n]["to"].as_str().unwrap())]
        };
        assert_eq!(line(&e, "l1"), [[0.0, 2.0], [10.0, 2.0]]);
        assert_eq!(line(&e, "l2"), [[18.0, 10.0], [18.0, 20.0]]);
        let width = json!({"type": "point_line", "point": "i", "line": "out1", "value": 4});
        e.apply(
            serde_json::from_value(
                json!({"remove_constraints": [width], "add_constraints": [{"type": "point_line", "point": "i", "line": "out1", "value": 6}]}),
            )
            .unwrap(),
        );
        assert_eq!(line(&e, "l1"), [[0.0, 3.0], [10.0, 3.0]]);
        assert_eq!(line(&e, "l2"), [[17.0, 10.0], [17.0, 20.0]]);
    }

    /// A wall that turns a corner against a straight one: not of one width, no middle line of
    /// lines and arcs, nothing made.
    #[test]
    fn no_middle_line_where_the_width_changes() {
        let mut e = editor(
            json!({"points": {"a": [0, 0], "b": [10, 0], "f": [20, 5], "c": [0, 6], "d": [20, 6]},
                   "lines": {"low1": {"from": "a", "to": "b"}, "low2": {"from": "b", "to": "f"}, "high": {"from": "c", "to": "d"}}}),
            false,
        );
        e.draft.tool = Tool::Offset;
        e.draft.midline = true;
        click_at(&mut e, [5.0, 0.0], false);
        assert!(click_at(&mut e, [10.0, 6.0], false).is_none());
        assert!(e.json["sketch"]["lines"].get("l1").is_none());
    }

    /// Two lines closing in (sketch_test6): the middle line is the bisector between them,
    /// construction, each end of the first mirrored onto the second across it; moved, the second
    /// line keeps it the bisector.
    #[test]
    fn two_lines_closing_in_have_their_bisector() {
        let mut e = editor(
            json!({"points": {"a": [0, 0], "b": [0, 20], "c": [10, 0], "d": [6, 20]},
                   "lines": {"left": {"from": "a", "to": "b"}, "right": {"from": "c", "to": "d"}},
                   "constraints": [{"type": "fix", "point": "a"}, {"type": "fix", "point": "b"}, {"type": "fix", "point": "c"}]}),
            false,
        );
        e.draft.tool = Tool::Offset;
        e.draft.midline = true;
        click_at(&mut e, [0.0, 10.0], false);
        let patch = click_at(&mut e, [8.0, 10.0], false).unwrap();
        assert!(e.placed.is_ok(), "{:?}", e.placed.as_ref().err());
        let m = patch["lines"].as_object().unwrap().keys().next().unwrap().clone();
        assert_eq!(patch["lines"][&m]["construction"], json!(true));
        let bisects = |e: &crate::sketch::SketchEdit| {
            let p = e.placed.as_ref().unwrap();
            let q = |n: &str| p.solved.points[n];
            let l = &p.lines[&m];
            let (m0, m1) = (q(&l.from), q(&l.to));
            // Its middle as far from one wall as from the other.
            let off = |x: datum_math::Vec2, p0: datum_math::Vec2, p1: datum_math::Vec2| {
                ((p1.x - p0.x) * (x.y - p0.y) - (p1.y - p0.y) * (x.x - p0.x)).abs() / libm::hypot(p1.x - p0.x, p1.y - p0.y)
            };
            let mid = datum_math::Vec2::new(0.5 * (m0.x + m1.x), 0.5 * (m0.y + m1.y));
            let apart = (off(mid, q("a"), q("b")) - off(mid, q("c"), q("d"))).abs();
            // The angle it makes with each wall.
            let angle = |p0: datum_math::Vec2, p1: datum_math::Vec2| {
                let (u, v) = ([p1.x - p0.x, p1.y - p0.y], [m1.x - m0.x, m1.y - m0.y]);
                ((u[0] * v[0] + u[1] * v[1]) / (libm::hypot(u[0], u[1]) * libm::hypot(v[0], v[1]))).abs()
            };
            (apart, (angle(q("a"), q("b")) - angle(q("c"), q("d"))).abs())
        };
        let (d, t) = bisects(&e);
        assert!(d < 1e-9 && t < 1e-9, "{d} {t}");
        // The free end of the second wall moved: still the bisector.
        e.apply(serde_json::from_value(json!({"points": {"d": [9, 20]}})).unwrap());
        let (d, t) = bisects(&e);
        assert!(d < 1e-9 && t < 1e-9, "{d} {t}");
    }
}
