//! Trim and extend in a sketch (D239): one tool (X), what a click does decided by where it is.
//! Over a piece of a curve between crossings, the piece goes; over a curve that crosses nothing,
//! its nearer end goes on to the first curve ahead, or, with none ahead, the curve goes; Shift
//! held extends the nearer end. Any curve cuts, projections and construction lines too; only the
//! sketch's own are cut.
//!
//! Crossings are found on the solved sketch. Where the sketch itself says two curves meet (a
//! point shared or coincident, a point on a curve, a tangent), the meeting is taken from it, not
//! measured: an end that touches a line by its constraint is not a crossing a hair away. A new
//! end stands on what cut it (`on`); a point already there becomes the end. The length of what
//! is cut and the constraints of an end that goes go with it; the direction stays; a curve split
//! in two keeps its line (`collinear`) or its circle (`equal_radius`).

use std::f64::consts::TAU;

use datum_ops::Placed;
use datum_sketch::Constraint;
use serde_json::{Map, Value as Json, json};

use crate::draw::Patch;
use crate::sketch::SketchEdit;

/// What a click would do: the `edit_sketch` patch, the piece it takes away and the length it
/// adds, as polylines in sketch coordinates.
#[derive(Clone, Debug, PartialEq)]
pub struct Plan {
    pub patch: Map<String, Json>,
    pub gone: Vec<[f64; 2]>,
    pub added: Vec<[f64; 2]>,
}

/// What a click at pixel offset `(dx, dy)` would do, Shift held if `extend`; none over nothing
/// it can change.
pub fn plan(edit: &SketchEdit, dx: f64, dy: f64, height: f64, extend: bool) -> Option<Plan> {
    let name = edit.curve_at(dx, dy, height)?;
    let sketch = &edit.json["sketch"];
    let key = ["lines", "arcs", "circles"].into_iter().find(|k| sketch[k].get(&name).is_some())?;
    let p = edit.placed.as_ref().ok()?;
    let s = Shape::of(p, &name)?;
    let sk = Sketch::new(edit, p)?;
    let tc = s.nearest(edit.camera.at(dx, dy, height));
    let near = if tc < 0.5 { "from" } else { "to" };
    if extend {
        return sk.extend(&name, key, &s, near);
    }
    let cuts = sk.cuts(&name, &s);
    if cuts.is_empty() {
        let longer = if key == "circles" { None } else { sk.extend(&name, key, &s, near) };
        return longer.or_else(|| sk.delete(&name, key, &s));
    }
    sk.trim(&name, key, &s, &cuts, tc)
}

/// A line from `.0` to `.1`, or an arc of a circle about `c` from angle `start` counterclockwise
/// by `sweep` (a circle: a whole turn, `closed`). Along it, 0 is its start and 1 its end.
#[derive(Clone, Copy, Debug)]
enum Shape {
    Line([f64; 2], [f64; 2]),
    Round { c: [f64; 2], r: f64, start: f64, sweep: f64, closed: bool },
}

impl Shape {
    fn of(p: &Placed, name: &str) -> Option<Shape> {
        let at = |n: &str| p.solved.points.get(n).map(|v| [v.x, v.y]);
        if let Some(l) = p.lines.get(name) {
            return Some(Shape::Line(at(&l.from)?, at(&l.to)?));
        }
        if let Some(a) = p.arcs.get(name) {
            let (c, f, t) = (at(&a.center)?, at(&a.from)?, at(&a.to)?);
            let start = angle(c, f);
            let sweep = (angle(c, t) - start).rem_euclid(TAU);
            return Some(Shape::Round { c, r: dist(c, f), start, sweep: if sweep == 0.0 { TAU } else { sweep }, closed: false });
        }
        let k = p.circles.get(name)?;
        Some(Shape::Round { c: at(&k.center)?, r: 0.5 * k.diameter.value()?, start: 0.0, sweep: TAU, closed: true })
    }

    fn closed(&self) -> bool {
        matches!(self, Shape::Round { closed: true, .. })
    }

    /// Where point `x` of the curve's line or circle is along it; on an arc's circle past its
    /// end, over 1 up to a whole turn.
    fn param(&self, x: [f64; 2]) -> f64 {
        match *self {
            Shape::Line(a, b) => {
                let u = sub(b, a);
                dot(sub(x, a), u) / dot(u, u)
            }
            Shape::Round { c, start, sweep, .. } => (angle(c, x) - start).rem_euclid(TAU) / sweep,
        }
    }

    /// Where along the curve itself the point nearest `x` is.
    fn nearest(&self, x: [f64; 2]) -> f64 {
        let t = self.param(x);
        match *self {
            Shape::Line(..) => t.clamp(0.0, 1.0),
            Shape::Round { closed: true, .. } => t,
            // Off an arc, its nearer end.
            Shape::Round { sweep, .. } if t > 1.0 => {
                if t - 1.0 < TAU / sweep - t {
                    1.0
                } else {
                    0.0
                }
            }
            Shape::Round { .. } => t,
        }
    }

    fn at(&self, t: f64) -> [f64; 2] {
        match *self {
            Shape::Line(a, b) => [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t],
            Shape::Round { c, r, start, sweep, .. } => {
                let (s, co) = libm::sincos(start + sweep * t);
                [c[0] + r * co, c[1] + r * s]
            }
        }
    }

    /// Whether point `x` of the curve's line or circle is on the curve, its ends included.
    fn holds(&self, x: [f64; 2]) -> bool {
        let t = self.param(x);
        match self {
            Shape::Line(..) => (0.0..=1.0).contains(&t),
            Shape::Round { closed, .. } => *closed || t <= 1.0,
        }
    }

    /// Whether `t` lies between the curve's ends (a circle has none).
    fn inside(&self, t: f64) -> bool {
        self.closed() || (0.0 < t && t < 1.0)
    }

    /// How far past end `end` along the curve's line or circle point `x` is, by `t`: over 0 if
    /// it is past it, and where it is then (an arc's start backward, below 0).
    fn beyond(&self, x: [f64; 2], end: &str) -> Option<(f64, f64)> {
        let t = self.param(x);
        let (past, t) = match (*self, end) {
            (Shape::Line(..), "to") => (t - 1.0, t),
            (Shape::Line(..), _) => (-t, t),
            // On the arc itself, not past either end.
            (Shape::Round { .. }, _) if t <= 1.0 => return None,
            (Shape::Round { .. }, "to") => (t - 1.0, t),
            (Shape::Round { sweep, .. }, _) => (TAU / sweep - t, t - TAU / sweep),
        };
        (past > 0.0).then_some((past, t))
    }

    /// The curve from `t0` to `t1`, as polyline points.
    fn polyline(&self, t0: f64, t1: f64) -> Vec<[f64; 2]> {
        let n = match self {
            Shape::Line(..) => 1,
            Shape::Round { sweep, .. } => ((t1 - t0).abs() * sweep / TAU * 64.0).ceil().max(1.0) as usize,
        };
        (0..=n).map(|k| self.at(t0 + (t1 - t0) * k as f64 / n as f64)).collect()
    }
}

/// Where the lines and circles of two curves cross.
fn crossings(s: &Shape, o: &Shape) -> Vec<[f64; 2]> {
    match (*s, *o) {
        (Shape::Line(a, b), Shape::Line(p, q)) => {
            let (u, v) = (sub(b, a), sub(q, p));
            let d = cross(u, v);
            if d == 0.0 {
                return Vec::new();
            }
            let t = cross(sub(p, a), v) / d;
            vec![[a[0] + u[0] * t, a[1] + u[1] * t]]
        }
        (Shape::Line(a, b), Shape::Round { c, r, .. }) | (Shape::Round { c, r, .. }, Shape::Line(a, b)) => {
            let u = sub(b, a);
            let l2 = dot(u, u);
            let f = dot(sub(c, a), u) / l2;
            let foot = [a[0] + u[0] * f, a[1] + u[1] * f];
            let d2 = dot(sub(c, foot), sub(c, foot));
            if d2 > r * r {
                return Vec::new();
            }
            let h = ((r * r - d2) / l2).sqrt();
            let mut out = vec![[a[0] + u[0] * (f - h), a[1] + u[1] * (f - h)], [a[0] + u[0] * (f + h), a[1] + u[1] * (f + h)]];
            out.dedup();
            out
        }
        (Shape::Round { c, r, .. }, Shape::Round { c: k, r: q, .. }) => {
            let d = dist(c, k);
            if d == 0.0 || d > r + q || d < (r - q).abs() {
                return Vec::new();
            }
            let along = (r * r - q * q + d * d) / (2.0 * d);
            let h = (r * r - along * along).max(0.0).sqrt();
            let u = [(k[0] - c[0]) / d, (k[1] - c[1]) / d];
            let f = [c[0] + u[0] * along, c[1] + u[1] * along];
            let mut out = vec![[f[0] - u[1] * h, f[1] + u[0] * h], [f[0] + u[1] * h, f[1] - u[0] * h]];
            out.dedup();
            out
        }
    }
}

/// Where two curves the sketch makes tangent touch: the foot of the centre on the line, or on
/// the line of the centres at the radius, outside or inside.
fn touch(s: &Shape, o: &Shape) -> Option<[f64; 2]> {
    match (*s, *o) {
        (Shape::Line(a, b), Shape::Round { c, .. }) | (Shape::Round { c, .. }, Shape::Line(a, b)) => {
            let u = sub(b, a);
            let f = dot(sub(c, a), u) / dot(u, u);
            Some([a[0] + u[0] * f, a[1] + u[1] * f])
        }
        (Shape::Round { c, r, .. }, Shape::Round { c: k, r: q, .. }) => {
            let d = dist(c, k);
            if d == 0.0 {
                return None;
            }
            let outside = (d - (r + q)).abs() <= (d - (r - q).abs()).abs();
            let toward = if outside || r > q { 1.0 } else { -1.0 };
            Some([c[0] + (k[0] - c[0]) / d * r * toward, c[1] + (k[1] - c[1]) / d * r * toward])
        }
        (Shape::Line(..), Shape::Line(..)) => None,
    }
}

/// A point a curve is cut at: where along it, where, and what the new end stands on there.
#[derive(Clone, Debug)]
struct Cut {
    t: f64,
    at: [f64; 2],
    tie: Tie,
}

#[derive(Clone, Debug, PartialEq)]
enum Tie {
    /// A point of the sketch already there: it becomes the end.
    Point(String),
    /// A new point, on this curve.
    On(String),
}

/// The sketch as trimming reads it: solved, with its constraints, typed and as written.
struct Sketch<'a> {
    edit: &'a SketchEdit,
    p: &'a Placed,
    typed: Vec<Constraint>,
    raw: Vec<Json>,
}

impl<'a> Sketch<'a> {
    fn new(edit: &'a SketchEdit, p: &'a Placed) -> Option<Self> {
        let raw = edit.json["sketch"]["constraints"].as_array().cloned().unwrap_or_default();
        let typed = raw.iter().map(|c| serde_json::from_value(c.clone()).ok()).collect::<Option<Vec<Constraint>>>()?;
        Some(Self { edit, p, typed, raw })
    }

    fn json(&self) -> &Json {
        &self.edit.json["sketch"]
    }

    fn at(&self, n: &str) -> Option<[f64; 2]> {
        self.p.solved.points.get(n).map(|v| [v.x, v.y])
    }

    /// The ends of a line or arc, own or projected.
    fn ends(&self, n: &str) -> Vec<&'a str> {
        if let Some(l) = self.p.lines.get(n) {
            return vec![&l.from, &l.to];
        }
        self.p.arcs.get(n).map(|a| vec![a.from.as_str(), a.to.as_str()]).unwrap_or_default()
    }

    /// One point, or two made one by the sketch.
    fn same(&self, a: &str, b: &str) -> bool {
        a == b || self.typed.iter().any(|c| matches!(c, Constraint::Coincident { points: [x, y] } if (x == a && y == b) || (x == b && y == a)))
    }

    /// Whether the sketch puts point `q` on curve `c`.
    fn on(&self, q: &str, c: &str) -> bool {
        self.typed.iter().any(|k| match k {
            Constraint::On { point, curve } => point == q && curve == c,
            Constraint::Midpoint { point, line } => point == q && line == c,
            _ => false,
        })
    }

    fn tangent(&self, a: &str, b: &str) -> bool {
        self.typed.iter().any(|c| matches!(c, Constraint::Tangent { curves } if curves.iter().any(|n| n == a) && curves.iter().any(|n| n == b)))
    }

    /// Every curve that can cut, by name (ellipses do not yet).
    fn cutters(&self) -> impl Iterator<Item = (&'a String, Shape)> + '_ {
        self.p.lines.keys().chain(self.p.arcs.keys()).chain(self.p.circles.keys()).filter_map(|n| Some((n, Shape::of(self.p, n)?)))
    }

    /// Where curve `t` (shape `s`) meets curve `n` (shape `o`), off what the sketch already ties
    /// them by: each crossing on both, as measured. Where the sketch says they meet, the crossing
    /// measured there is the same meeting and is left out; of curves it makes tangent, only the
    /// point they touch at, unless they meet at a point it names.
    fn met(&self, t: &str, s: &Shape, n: &str, o: &Shape) -> Vec<[f64; 2]> {
        let (own, theirs) = (self.ends(t), self.ends(n));
        let mut known = Vec::new();
        for e in &theirs {
            if own.iter().any(|f| self.same(f, e)) || self.on(e, t) {
                known.extend(self.at(e));
            }
        }
        for f in &own {
            if self.on(f, n) {
                known.extend(self.at(f));
            }
        }
        let mut found =
            if self.tangent(t, n) { if known.is_empty() { touch(s, o).into_iter().collect() } else { Vec::new() } } else { crossings(s, o) };
        for k in known {
            if let Some(i) = (0..found.len()).min_by(|&i, &j| dist(found[i], k).total_cmp(&dist(found[j], k))) {
                found.remove(i);
            }
        }
        found.retain(|&x| o.holds(x));
        found
    }

    /// Where curve `t` (shape `s`) is cut, in order along it: points the sketch puts on it, and
    /// where other curves cross it between its ends.
    fn cuts(&self, t: &str, s: &Shape) -> Vec<Cut> {
        let own = self.ends(t);
        let mut out = Vec::new();
        for c in &self.typed {
            let q = match c {
                Constraint::On { point, curve } if curve == t => point,
                Constraint::Midpoint { point, line } if line == t => point,
                _ => continue,
            };
            if own.iter().any(|e| self.same(e, q)) {
                continue;
            }
            if let Some(x) = self.at(q) {
                let u = s.param(x);
                if s.inside(u) {
                    out.push(Cut { t: u, at: x, tie: Tie::Point(q.clone()) });
                }
            }
        }
        for (n, o) in self.cutters().filter(|(n, _)| n.as_str() != t) {
            for x in self.met(t, s, n, &o) {
                let u = s.param(x);
                if s.inside(u) {
                    out.push(Cut { t: u, at: x, tie: Tie::On(n.clone()) });
                }
            }
        }
        out.sort_by(|a, b| a.t.total_cmp(&b.t));
        out.dedup_by(|a, b| matches!((&a.tie, &b.tie), (Tie::Point(x), Tie::Point(y)) if x == y));
        out
    }

    /// The point a cut makes the new end: the one there, or a new one on what cut.
    fn end(&self, patch: &mut Patch, c: &Cut) -> String {
        match &c.tie {
            Tie::Point(q) => q.clone(),
            Tie::On(n) => {
                let q = patch.fresh("p");
                patch.set("points", &q, json!(c.at));
                patch.constrain(json!({"type": "on", "point": q, "curve": n}));
                q
            }
        }
    }

    /// Point `e`, left by curve `t`, taken away unless another curve or a pattern holds it.
    fn leave(&self, patch: &mut Patch, e: &str, t: &str) {
        let json = self.json();
        let held = ["lines", "arcs", "circles", "patterns"].iter().filter_map(|k| json[k].as_object()).any(|m| {
            m.iter().filter(|(n, _)| n.as_str() != t).any(|(_, v)| {
                let mut found = false;
                datum_model::visit_names(&mut v.clone(), &mut |n| found |= n == e);
                found
            })
        });
        if !held && json["points"].get(e).is_some() {
            patch.set("points", e, Json::Null);
        }
    }

    /// What of curve `t`'s constraints no longer holds once it is cut or extended to new ends
    /// `ends`: its length, a point on it that is now its end; a point at its middle stays on it.
    fn unconstrain(&self, patch: &mut Patch, t: &str, ends: &[&str]) {
        for (c, raw) in self.typed.iter().zip(&self.raw) {
            match c {
                Constraint::Length { line: n, .. } | Constraint::ArcLength { arc: n, .. } | Constraint::ArcAngle { arc: n, .. } if n == t => {}
                Constraint::Equal { lines } if lines.iter().any(|n| n == t) => {}
                Constraint::On { point, curve } if curve == t && ends.contains(&point.as_str()) => {}
                Constraint::Midpoint { point, line } if line == t => {
                    if !ends.contains(&point.as_str()) {
                        patch.constrain(json!({"type": "on", "point": point, "curve": t}));
                    }
                }
                _ => continue,
            }
            patch.unconstrain(raw.clone());
        }
    }

    /// Groups that hold curve `t`, each with `with` in it as well (`also`) or instead.
    fn regroup(&self, patch: &mut Patch, t: &str, with: &str, also: bool) {
        for (g, members) in self.json()["groups"].as_object().into_iter().flatten() {
            let Some(members) = members.as_array() else { continue };
            if !members.iter().any(|m| m == t) {
                continue;
            }
            let mut next: Vec<Json> = members.iter().filter(|m| also || *m != t).cloned().collect();
            next.push(json!(with));
            patch.set("groups", g, Json::Array(next));
        }
    }

    /// Curve `t` taken away whole, and its points nothing else holds.
    fn delete(&self, t: &str, key: &str, s: &Shape) -> Option<Plan> {
        let mut patch = Patch::new(self.edit);
        patch.set(key, t, Json::Null);
        let v = &self.json()[key][t];
        for f in ["from", "to", "center"] {
            if let Some(e) = v[f].as_str() {
                self.leave(&mut patch, e, t);
            }
        }
        Some(Plan { patch: patch.done()?, gone: s.polyline(0.0, 1.0), added: Vec::new() })
    }

    /// The piece of curve `t` (in map `key`, shape `s`) between the cuts `t_at` lies between
    /// taken away.
    fn trim(&self, t: &str, key: &str, s: &Shape, cuts: &[Cut], t_at: f64) -> Option<Plan> {
        let mut patch = Patch::new(self.edit);
        let v = &self.json()[key][t];
        if s.closed() {
            return self.open_circle(&mut patch, t, s, cuts, t_at).and_then(|gone| Some(Plan { patch: patch.done()?, gone, added: Vec::new() }));
        }
        let k = cuts.iter().filter(|c| c.t <= t_at).count();
        let mut kept = v.clone();
        let (gone, ends) = if k == 0 {
            let e = self.end(&mut patch, &cuts[0]);
            kept["from"] = json!(e);
            self.leave(&mut patch, v["from"].as_str()?, t);
            (s.polyline(0.0, cuts[0].t), vec![e])
        } else if k == cuts.len() {
            let e = self.end(&mut patch, &cuts[k - 1]);
            kept["to"] = json!(e);
            self.leave(&mut patch, v["to"].as_str()?, t);
            (s.polyline(cuts[k - 1].t, 1.0), vec![e])
        } else {
            // A piece in the middle: the curve in two, the second on the first's line or circle.
            let (a, b) = (self.end(&mut patch, &cuts[k - 1]), self.end(&mut patch, &cuts[k]));
            kept["to"] = json!(a);
            let second = patch.fresh(if key == "lines" { "l" } else { "a" });
            let mut other = v.clone();
            other["from"] = json!(b);
            patch.set(key, &second, other);
            let kind = if key == "lines" {
                json!({"type": "collinear", "lines": [t, second]})
            } else {
                json!({"type": "equal_radius", "curves": [t, second]})
            };
            patch.constrain(kind);
            self.regroup(&mut patch, t, &second, true);
            (s.polyline(cuts[k - 1].t, cuts[k].t), vec![a, b])
        };
        patch.set(key, t, kept);
        let ends: Vec<&str> = ends.iter().map(String::as_str).collect();
        self.unconstrain(&mut patch, t, &ends);
        Some(Plan { patch: patch.done()?, gone, added: Vec::new() })
    }

    /// Circle `t` opened at the piece between the cuts `t_at` lies between: an arc round the
    /// rest, of the circle's radius, under a new name with the circle's constraints. The piece
    /// taken away, as a polyline.
    fn open_circle(&self, patch: &mut Patch, t: &str, s: &Shape, cuts: &[Cut], t_at: f64) -> Option<Vec<[f64; 2]>> {
        if cuts.len() < 2 {
            return None;
        }
        let i = cuts.iter().rposition(|c| c.t <= t_at).unwrap_or(cuts.len() - 1);
        let (a, b) = (&cuts[i], &cuts[(i + 1) % cuts.len()]);
        let gone = s.polyline(a.t, if b.t > a.t { b.t } else { b.t + 1.0 });
        let (from, to) = (self.end(patch, b), self.end(patch, a));
        let circle = &self.json()["circles"][t];
        let arc = patch.fresh("a");
        let mut v = json!({"center": circle["center"], "from": from, "to": to});
        if circle["construction"] == json!(true) {
            v["construction"] = json!(true);
        }
        patch.set("arcs", &arc, v);
        patch.set("circles", t, Json::Null);
        // The circle's constraints go with it: those that hold of the arc come back, renamed.
        for (c, raw) in self.typed.iter().zip(&self.raw) {
            let mut named = false;
            datum_model::visit_names(&mut raw.clone(), &mut |n| named |= n == t);
            let ends_on = matches!(c, Constraint::On { point, .. } | Constraint::Midpoint { point, .. } if *point == from || *point == to);
            if !named || ends_on {
                continue;
            }
            let mut r = raw.clone();
            if let Constraint::Midpoint { point, .. } = c {
                r = json!({"type": "on", "point": point, "curve": t});
            }
            datum_model::visit_names(&mut r, &mut |n| {
                if n == t {
                    *n = arc.clone();
                }
            });
            patch.constrain(r);
        }
        // A circle another's diameter sizes stays that size by `equal_radius`.
        if !self.edit.followed_circles().contains_key(t) {
            let half = match &circle["diameter"] {
                Json::String(e) => json!(format!("({e}) / 2")),
                d => json!(d.as_f64()? / 2.0),
            };
            patch.constrain(json!({"type": "radius", "arc": arc, "value": half}));
        }
        self.regroup(patch, t, &arc, false);
        Some(gone)
    }

    /// End `end` (`from` or `to`) of curve `t` taken on along its line or circle to the first
    /// curve ahead. Only a free end: one no other curve shares.
    fn extend(&self, t: &str, key: &str, s: &Shape, end: &str) -> Option<Plan> {
        if s.closed() {
            return None;
        }
        let v = &self.json()[key][t];
        let e = v[end].as_str()?;
        let shared = self.p.lines.iter().map(|(n, l)| (n, [&l.from, &l.to])).chain(self.p.arcs.iter().map(|(n, a)| (n, [&a.from, &a.to])));
        if shared.filter(|(n, _)| n.as_str() != t).any(|(_, ends)| ends.iter().any(|q| self.same(q, e))) {
            return None;
        }
        let ahead = self
            .cutters()
            .filter(|(n, _)| n.as_str() != t)
            .flat_map(|(n, o)| self.met(t, s, n, &o).into_iter().filter_map(|x| Some((s.beyond(x, end)?, x, n.clone()))).collect::<Vec<_>>())
            .min_by(|a, b| a.0.0.total_cmp(&b.0.0));
        let ((_, to), x, n) = ahead?;
        let mut patch = Patch::new(self.edit);
        let q = self.end(&mut patch, &Cut { t: to, at: x, tie: Tie::On(n) });
        let mut longer = v.clone();
        longer[end] = json!(q);
        patch.set(key, t, longer);
        self.leave(&mut patch, e, t);
        self.unconstrain(&mut patch, t, &[&q]);
        let from = if end == "to" { 1.0 } else { 0.0 };
        Some(Plan { patch: patch.done()?, gone: Vec::new(), added: s.polyline(from, to) })
    }
}

fn sub(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [a[0] - b[0], a[1] - b[1]]
}

fn dot(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

fn cross(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

fn dist(a: [f64; 2], b: [f64; 2]) -> f64 {
    libm::hypot(a[0] - b[0], a[1] - b[1])
}

fn angle(c: [f64; 2], x: [f64; 2]) -> f64 {
    libm::atan2(x[1] - c[1], x[0] - c[0])
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use serde_json::{Value as Json, json};

    use crate::draw::Tool;
    use crate::draw::tests::{click_at, editor, hover_at};
    use crate::sketch::SketchEdit;

    /// A click of the trim tool at `q`, Shift held if `shift`: the sketch after it.
    fn trim_at(e: &mut SketchEdit, q: [f64; 2], shift: bool) -> Option<Json> {
        e.draft.tool = Tool::Trim;
        click_at(e, q, shift)?;
        assert!(e.placed.is_ok(), "{:?}", e.placed.as_ref().err());
        Some(e.json["sketch"].clone())
    }

    /// Where point `n` of the solved sketch is, rounded to a millionth.
    fn at(e: &SketchEdit, n: &str) -> [f64; 2] {
        let v = e.placed.as_ref().unwrap().solved.points[n];
        [(v.x * 1e6).round() / 1e6, (v.y * 1e6).round() / 1e6]
    }

    fn ends(e: &SketchEdit, key: &str, n: &str) -> [[f64; 2]; 2] {
        let c = &e.json["sketch"][key][n];
        [at(e, c["from"].as_str().unwrap()), at(e, c["to"].as_str().unwrap())]
    }

    fn has(s: &Json, c: Json) -> bool {
        s["constraints"].as_array().unwrap().contains(&c)
    }

    /// A horizontal line across a vertical one: the piece clicked past the crossing goes, its
    /// end gone with it, the new end on the vertical line.
    #[test]
    fn a_piece_past_a_crossing_goes() {
        let mut e = editor(
            json!({"points": {"a": [0, 0], "b": [20, 0], "c": [10, -5], "d": [10, 5]},
                   "lines": {"h": {"from": "a", "to": "b"}, "v": {"from": "c", "to": "d"}},
                   "constraints": [{"type": "length", "line": "h", "value": 20}, {"type": "horizontal", "line": "h"}]}),
            false,
        );
        let s = trim_at(&mut e, [16.0, 0.0], false).unwrap();
        assert_eq!(ends(&e, "lines", "h"), [[0.0, 0.0], [10.0, 0.0]]);
        assert!(s["points"].get("b").is_none(), "the end cut away goes");
        let end = s["lines"]["h"]["to"].as_str().unwrap();
        assert!(has(&s, json!({"type": "on", "point": end, "curve": "v"})));
        assert!(has(&s, json!({"type": "horizontal", "line": "h"})), "the direction stays");
        assert!(!has(&s, json!({"type": "length", "line": "h", "value": 20})), "the length goes");
        // The vertical line is cut by the horizontal one now: its lower piece.
        trim_at(&mut e, [10.0, -3.0], false).unwrap();
        assert_eq!(ends(&e, "lines", "v"), [[10.0, 0.0], [10.0, 5.0]]);
    }

    /// The piece between two crossings: the line in two, on one line.
    #[test]
    fn a_piece_between_crossings_splits_the_line() {
        let mut e = editor(
            json!({"points": {"a": [0, 0], "b": [20, 0], "c": [5, -5], "d": [5, 5], "f": [15, -5], "g": [15, 5]},
                   "lines": {"h": {"from": "a", "to": "b"}, "v": {"from": "c", "to": "d"}, "w": {"from": "f", "to": "g"}},
                   "groups": {"cell": ["h", "v"]}}),
            false,
        );
        let s = trim_at(&mut e, [10.0, 0.0], false).unwrap();
        assert_eq!(ends(&e, "lines", "h"), [[0.0, 0.0], [5.0, 0.0]]);
        assert_eq!(ends(&e, "lines", "l1"), [[15.0, 0.0], [20.0, 0.0]]);
        assert!(has(&s, json!({"type": "collinear", "lines": ["h", "l1"]})));
        assert_eq!(s["groups"]["cell"], json!(["h", "v", "l1"]), "the second piece joins the first's groups");
    }

    /// A line ending on another by its constraint (a T): the other is cut at that end exactly,
    /// the end itself its new end, the constraint no longer needed.
    #[test]
    fn a_tee_cuts_at_its_own_point() {
        let mut e = editor(
            json!({"points": {"a": [0, 0], "b": [20, 0], "c": [10, 0], "d": [10, 8]},
                   "lines": {"h": {"from": "a", "to": "b"}, "v": {"from": "c", "to": "d"}},
                   "constraints": [{"type": "on", "point": "c", "curve": "h"}]}),
            false,
        );
        let s = trim_at(&mut e, [4.0, 0.0], false).unwrap();
        assert_eq!(s["lines"]["h"], json!({"from": "c", "to": "b"}));
        assert!(s["points"].get("a").is_none());
        assert!(s["constraints"].as_array().unwrap().is_empty(), "{s}");
    }

    /// A corner of a rectangle stays where the side cut away from it met another.
    #[test]
    fn a_corner_stays_with_the_side_it_holds() {
        let mut e = editor(
            json!({"points": {"a": [0, 0], "b": [20, 0], "c": [20, 10], "d": [0, 10], "m": [8, -5], "n": [8, 5]},
                   "lines": {"bottom": {"from": "a", "to": "b"}, "right": {"from": "b", "to": "c"}, "top": {"from": "c", "to": "d"},
                             "left": {"from": "d", "to": "a"}, "cut": {"from": "m", "to": "n"}}}),
            false,
        );
        let s = trim_at(&mut e, [3.0, 0.0], false).unwrap();
        assert_eq!(ends(&e, "lines", "bottom"), [[8.0, 0.0], [20.0, 0.0]]);
        assert!(s["points"].get("a").is_some(), "the corner holds the left side");
        // The corner is no crossing: the cut line's piece below the bottom goes up to it.
        trim_at(&mut e, [8.0, -3.0], false).unwrap();
        assert_eq!(ends(&e, "lines", "cut"), [[8.0, 0.0], [8.0, 5.0]]);
    }

    /// A line that crosses nothing goes on to the next curve; Shift extends a line that does.
    #[test]
    fn a_line_crossing_nothing_extends() {
        let mut e = editor(
            json!({"points": {"a": [0, 0], "b": [5, 0], "c": [10, -5], "d": [10, 5], "f": [30, -5], "g": [30, 5]},
                   "lines": {"h": {"from": "a", "to": "b"}, "v": {"from": "c", "to": "d"}, "w": {"from": "f", "to": "g"}}}),
            false,
        );
        e.draft.tool = Tool::Trim;
        hover_at(&mut e, [4.0, 0.0], false);
        assert_eq!(e.draft.trim.as_ref().unwrap().added, vec![[5.0, 0.0], [10.0, 0.0]], "the length it adds is shown");
        let s = trim_at(&mut e, [4.0, 0.0], false).unwrap();
        assert_eq!(ends(&e, "lines", "h"), [[0.0, 0.0], [10.0, 0.0]]);
        assert!(s["points"].get("b").is_none());
        // It crosses `v` at its end now; held Shift, it goes on to `w`.
        trim_at(&mut e, [9.0, 0.0], true).unwrap();
        assert_eq!(ends(&e, "lines", "h"), [[0.0, 0.0], [30.0, 0.0]]);
        // Without Shift a piece of it past `v` goes.
        trim_at(&mut e, [20.0, 0.0], false).unwrap();
        assert_eq!(ends(&e, "lines", "h"), [[0.0, 0.0], [10.0, 0.0]]);
        assert!(e.json["sketch"]["lines"].get("l1").is_none(), "an end piece: no line in two");
    }

    /// A line alone crosses nothing and has nothing ahead: it goes, its points with it.
    #[test]
    fn a_line_alone_goes() {
        let mut e = editor(json!({"points": {"a": [0, 0], "b": [5, 0]}, "lines": {"h": {"from": "a", "to": "b"}}}), false);
        let s = trim_at(&mut e, [2.0, 0.0], false).unwrap();
        assert_eq!(s["lines"], json!({}));
        assert_eq!(s["points"], json!({}));
    }

    /// A circle cut by a line across it: the half clicked goes, the other half an arc of its
    /// radius.
    #[test]
    fn a_circle_cut_is_an_arc() {
        let mut e = editor(
            json!({"points": {"o": [10, 0], "a": [0, 0], "b": [20, 0]},
                   "lines": {"h": {"from": "a", "to": "b"}},
                   "circles": {"k": {"center": "o", "diameter": "d"}},
                   "constraints": [{"type": "on", "point": "o", "curve": "h"}]}),
            false,
        );
        e.params.insert("d".into(), 10.0);
        e.solve();
        let s = trim_at(&mut e, [10.0, 5.0], false).unwrap();
        assert!(s["circles"].get("k").is_none());
        assert_eq!(ends(&e, "arcs", "a1"), [[5.0, 0.0], [15.0, 0.0]], "{s}");
        assert!(has(&s, json!({"type": "radius", "arc": "a1", "value": "(d) / 2"})));
    }

    /// A slot drawn as two circles and two lines tangent to them, ending on them: the inner
    /// halves trimmed, the slot's outline is left, tangent where it was.
    #[test]
    fn a_slot_from_two_circles() {
        let mut e = editor(
            json!({"points": {"o": [0, 0], "q": [20, 0], "p1": [0, 5], "p2": [20, 5], "p3": [0, -5], "p4": [20, -5]},
                   "lines": {"top": {"from": "p1", "to": "p2"}, "bottom": {"from": "p3", "to": "p4"}},
                   "circles": {"k1": {"center": "o", "diameter": 10}, "k2": {"center": "q", "diameter": 10}},
                   "constraints": [{"type": "on", "point": "p1", "curve": "k1"}, {"type": "on", "point": "p3", "curve": "k1"},
                                   {"type": "on", "point": "p2", "curve": "k2"}, {"type": "on", "point": "p4", "curve": "k2"},
                                   {"type": "tangent", "curves": ["top", "k1"]}, {"type": "tangent", "curves": ["bottom", "k1"]},
                                   {"type": "tangent", "curves": ["top", "k2"]}, {"type": "tangent", "curves": ["bottom", "k2"]}]}),
            false,
        );
        let s = trim_at(&mut e, [5.0, 0.0], false).unwrap();
        assert_eq!(s["arcs"]["a1"], json!({"center": "o", "from": "p1", "to": "p3"}));
        trim_at(&mut e, [15.0, 0.0], false).unwrap();
        let s = &e.json["sketch"];
        assert_eq!(s["arcs"]["a2"], json!({"center": "q", "from": "p4", "to": "p2"}));
        for c in [
            json!({"type": "tangent", "curves": ["top", "a1"]}),
            json!({"type": "tangent", "curves": ["bottom", "a2"]}),
            json!({"type": "radius", "arc": "a1", "value": 5.0}),
        ] {
            assert!(has(s, c.clone()), "{c} in {s}");
        }
        assert!(!s["constraints"].as_array().unwrap().iter().any(|c| c["type"] == "on"), "the ends on the circles are the arcs' own: {s}");
        assert_eq!(at(&e, "p1"), [0.0, 5.0]);
        assert!(!e.fill.is_empty(), "a closed profile");
    }

    /// An arc cut by a line across it, then its end past the line extended back to it.
    #[test]
    fn an_arc_trims_and_extends() {
        let mut e = editor(
            json!({"points": {"o": [0, 0], "f": [10, 0], "t": [-10, 0], "a": [-20, 5], "b": [20, 5]},
                   "lines": {"h": {"from": "a", "to": "b"}},
                   "arcs": {"r": {"center": "o", "from": "f", "to": "t"}}}),
            false,
        );
        // The middle of the arc, over the line, goes: the arc in two on one circle.
        let s = trim_at(&mut e, [0.0, 10.0], false).unwrap();
        assert!(has(&s, json!({"type": "equal_radius", "curves": ["r", "a1"]})));
        let [f, to] = ends(&e, "arcs", "r");
        assert_eq!(f, [10.0, 0.0]);
        assert_eq!(to[1], 5.0);
        // Held Shift at its start, the first piece goes on clockwise round to the line: its
        // own circle's other piece is no crossing.
        let s = trim_at(&mut e, [10.0, 0.5], true).unwrap();
        let [f, _] = ends(&e, "arcs", "r");
        assert_eq!(f[1], 5.0, "{s}");
        assert!(f[0] < 0.0, "round the long way, clockwise from the start: {f:?}");
    }

    /// An edge of the plate projected cuts a line drawn over it, and the line's end past the
    /// plate goes, the new end held on the edge.
    #[test]
    fn a_projected_edge_cuts() {
        let mut e = editor(json!({"points": {"a": [20, 15], "b": [50, 15]}, "lines": {"h": {"from": "a", "to": "b"}}}), true);
        e.draft.tool = Tool::Project;
        let p = click_at(&mut e, [40.0, 7.0], false).unwrap();
        let edge = p["project"].as_object().unwrap().keys().next().unwrap().clone();
        let s = trim_at(&mut e, [45.0, 15.0], false).unwrap();
        assert_eq!(ends(&e, "lines", "h"), [[20.0, 15.0], [40.0, 15.0]]);
        let end = s["lines"]["h"]["to"].as_str().unwrap();
        assert!(has(&s, json!({"type": "on", "point": end, "curve": edge})), "{s}");
    }

    /// As an engineer draws: a rectangle by the line tool, clicked at its corners, a line across
    /// it from the bottom to the top (its ends snap onto them), then the bottom's right piece and
    /// the top's right piece trimmed, and the right side, crossing nothing now and with nothing
    /// ahead, goes: the left part is a closed outline.
    #[test]
    fn a_rectangle_drawn_then_cut_across() {
        let mut e = editor(json!({"points": {}}), false);
        e.draft.tool = Tool::Line;
        for q in [[0.0, 0.0], [20.0, 0.0], [20.0, 10.0], [0.0, 10.0], [0.0, 0.0]] {
            click_at(&mut e, q, false);
        }
        e.draft.escape();
        e.draft.tool = Tool::Line;
        click_at(&mut e, [12.0, 0.0], false);
        click_at(&mut e, [12.0, 10.0], false);
        e.draft.escape();
        let before = e.fill.len();
        trim_at(&mut e, [16.0, 0.0], false).unwrap();
        trim_at(&mut e, [16.0, 10.0], false).unwrap();
        let s = trim_at(&mut e, [20.0, 5.0], false).unwrap();
        assert_eq!(s["lines"].as_object().unwrap().len(), 4, "{s}");
        assert!(before == 0 && !e.fill.is_empty(), "lines meeting in tees bound no region; the outline left does: {s}");
        let xs: Vec<f64> = s["points"].as_object().unwrap().keys().map(|n| at(&e, n)[0]).collect();
        assert!(xs.iter().all(|&x| x == 0.0 || x == 12.0), "{xs:?}");
    }
}
