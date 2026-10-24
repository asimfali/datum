//! Dimensions in the window (M7d3, D92): the value typed while drawing, the dimension tool, a
//! double click on a dimension to change it. One field by the cursor serves all three; a value
//! is a number or an expression of the model's parameters. What a dimension measures now is
//! asked of the solver (a reference dimension for the moment), not worked out again here. A new
//! dimension that over-constrains the sketch is not sent: the window offers it as a reference
//! dimension instead, as Inventor offers a driven one.

use serde_json::{Map, Value as Json, json};

use crate::sketch::{Pick, SketchEdit};

/// What the field by the cursor is for.
#[derive(Clone, Debug, PartialEq)]
pub enum Entry {
    /// The length, diameter or radius of what is being drawn.
    Draw,
    /// A new dimension: its constraint, all but the value under `key` (`value` or `degrees`).
    New { constraint: Json, key: &'static str },
    /// Dimension `k` of the constraints, given a new value.
    Edit(usize),
    /// A circle's diameter.
    Diameter(String),
    /// Parameter `k` of the library element being placed (D98): Tab goes to the next.
    Parameter(usize),
}

impl Entry {
    /// Whether the field serves what is being drawn or placed, rather than a dimension.
    pub fn drawing(&self) -> bool {
        matches!(self, Self::Draw | Self::Parameter(_))
    }
}

/// The field: what it is for, its text, and where it stands (sketch coordinates).
#[derive(Clone, Debug)]
pub struct Field {
    pub entry: Entry,
    pub text: String,
    pub at: [f64; 2],
}

/// A dimension that would over-constrain the sketch: its constraint, and the conflict.
#[derive(Clone, Debug)]
pub struct Conflict {
    pub constraint: Json,
    pub why: String,
}

/// A value as typed: a number, or an expression of parameters.
pub fn value_of(text: &str) -> Json {
    let t = text.trim().replace(',', ".");
    t.parse::<f64>().map_or_else(|_| json!(text.trim()), |v| json!(v))
}

/// A value as a field shows it: a number in its shortest form, an expression as it is.
pub fn text_of(v: &Json) -> String {
    match v {
        Json::String(s) => s.clone(),
        v => v.as_f64().map_or_else(String::new, |x| {
            let s = format!("{x:.6}");
            s.trim_end_matches('0').trim_end_matches('.').to_string()
        }),
    }
}

impl SketchEdit {
    /// What reference dimension `m` measures on the sketch as it is.
    pub fn measure(&self, m: Json) -> Option<f64> {
        let mut probe = self.clone_for_probe();
        probe.json["sketch"]["measures"]["__probe"] = m;
        probe.solve();
        probe.placed.ok()?.solved.measures.get("__probe").copied()
    }

    /// A copy to solve variations on, without touching what the window shows.
    fn clone_for_probe(&self) -> SketchEdit {
        SketchEdit::new(self.feature.clone(), self.context.clone(), self.json.clone(), self.params.clone(), None)
    }

    /// The dimension the picked elements make, its value key, and what it measures now. One
    /// line: its length, or with `axis` (H, V) the horizontal or vertical distance of its ends.
    /// A point and a line: the distance between them. Two lines: the distance between them if
    /// they are parallel (within a line's lean), else the angle. The ends of one arc: the length
    /// along it. An arc picked twice: the angle it turns. Two points: with `axis` as it says, else as the cursor at `at` (sketch
    /// coordinates) where it goes says: horizontal when above or below them, vertical when to a
    /// side, along them when straight off their middle (within 15°). An arc: its radius.
    pub fn dimension(&self, picks: &[Pick2], at: [f64; 2], axis: Option<Axis>) -> Option<(Json, &'static str, f64)> {
        let p = self.placed.as_ref().ok()?;
        let pos = |n: &str| p.solved.points.get(n).map(|v| [v.x, v.y]);
        let ends = |l: &str| p.lines.get(l).map(|l| (l.from.clone(), l.to.clone()));
        let on_axis = |a: &str, b: &str| match axis {
            Some(Axis::Horizontal) => Some(json!({"type": "horizontal_distance", "points": [a, b]})),
            Some(Axis::Vertical) => Some(json!({"type": "vertical_distance", "points": [a, b]})),
            None => None,
        };
        let (c, key) = match picks {
            [Pick2::Curve(a)] if p.arcs.contains_key(a) => (json!({"type": "radius", "arc": a}), "value"),
            [Pick2::Curve(a), Pick2::Curve(b)] if a == b && p.arcs.contains_key(a) => (json!({"type": "arc_angle", "arc": a}), "degrees"),
            [Pick2::Curve(l)] => {
                let (a, b) = ends(l)?;
                (on_axis(&a, &b).unwrap_or_else(|| json!({"type": "length", "line": l})), "value")
            }
            [Pick2::Point(q), Pick2::Curve(l)] | [Pick2::Curve(l), Pick2::Point(q)] if p.lines.contains_key(l) => {
                (json!({"type": "point_line", "point": q, "line": l}), "value")
            }
            [Pick2::Curve(a), Pick2::Curve(b)] if p.lines.contains_key(a) && p.lines.contains_key(b) => {
                let dir = |l: &str| {
                    let (f, t) = ends(l)?;
                    let (f, t) = (pos(&f)?, pos(&t)?);
                    Some([t[0] - f[0], t[1] - f[1]])
                };
                if crate::draw::parallel(dir(a)?, dir(b)?) {
                    (json!({"type": "point_line", "point": ends(a)?.0, "line": b}), "value")
                } else {
                    (json!({"type": "angle", "lines": [a, b]}), "degrees")
                }
            }
            [Pick2::Point(a), Pick2::Point(b)] => {
                if let Some(c) = on_axis(a, b) {
                    (c, "value")
                } else if let Some((n, _)) = p.arcs.iter().find(|(_, c)| (&c.from, &c.to) == (a, b) || (&c.from, &c.to) == (b, a)) {
                    (json!({"type": "arc_length", "arc": n}), "value")
                } else {
                    let (pa, pb) = (pos(a)?, pos(b)?);
                    let d = [pb[0] - pa[0], pb[1] - pa[1]];
                    let w = [at[0] - (pa[0] + pb[0]) * 0.5, at[1] - (pa[1] + pb[1]) * 0.5];
                    let len = libm::hypot(d[0], d[1]);
                    let off = libm::hypot(w[0], w[1]);
                    // Straight off the middle, across the two points: along them.
                    let across = len > 0.0 && off > 0.0 && (d[0] * w[0] + d[1] * w[1]).abs() / (len * off) < libm::sin(15f64.to_radians());
                    let kind = if across && d[0] != 0.0 && d[1] != 0.0 {
                        "distance"
                    } else if (w[1].abs() >= w[0].abs() && d[0] != 0.0) || d[1] == 0.0 {
                        "horizontal_distance"
                    } else {
                        "vertical_distance"
                    };
                    (json!({"type": kind, "points": [a, b]}), "value")
                }
            }
            _ => return None,
        };
        let mut m = c.clone();
        if let Some(o) = m.as_object_mut() {
            o.remove("value");
            o.remove("degrees");
        }
        let now = self.measure(m)?;
        (now > 0.0).then_some((c, key, now))
    }

    /// Opens the field, at `at`, for the dimension `picks` make without a click where it goes:
    /// a line's length (horizontal or vertical with `axis`), an arc's radius, a circle's
    /// diameter, a point and a line's distance, two lines' distance or angle, an arc's length
    /// by its ends; two points with `axis`. False when they make none so: two other points wait
    /// for that click.
    pub fn dimension_now(&mut self, picks: &[Pick2], at: [f64; 2], axis: Option<Axis>) -> bool {
        if let [Pick2::Curve(c)] = picks
            && self.placed.as_ref().is_ok_and(|p| p.circles.contains_key(c))
        {
            self.edit_label(&Pick::Curve(c.clone()), at);
            return self.field.is_some();
        }
        if let [Pick2::Point(a), Pick2::Point(b)] = picks
            && axis.is_none()
            && !self.placed.as_ref().is_ok_and(|p| p.arcs.values().any(|c| (&c.from, &c.to) == (a, b) || (&c.from, &c.to) == (b, a)))
        {
            return false;
        }
        let Some((c, key, now)) = self.dimension(picks, at, axis) else { return false };
        self.ask(c, key, now, at);
        true
    }

    /// What a frame dragged in the dimension tool dimensions: its curves, or its points if it
    /// has no curve. One or two curves, or a point and a line, open the field at once (two
    /// lines: their distance or angle); two points are returned, to wait for the click where it
    /// goes.
    pub fn dimension_framed(&mut self, framed: &[Pick], axis: Option<Axis>) -> Vec<Pick2> {
        let curves: Vec<Pick2> = framed.iter().filter_map(|p| if let Pick::Curve(n) = p { Some(Pick2::Curve(n.clone())) } else { None }).collect();
        let picks = if curves.is_empty() {
            framed.iter().filter_map(|p| if let Pick::Point(n) = p { Some(Pick2::Point(n.clone())) } else { None }).collect()
        } else {
            curves
        };
        if !matches!(picks.len(), 1 | 2) {
            return Vec::new();
        }
        match self.spot(&picks) {
            Some(at) if !self.dimension_now(&picks, at, axis) && matches!(picks[..], [Pick2::Point(_), Pick2::Point(_)]) => picks,
            _ => Vec::new(),
        }
    }

    /// H or V in the dimension tool (its sub-mode): `axis` taken, or dropped if taken already.
    /// The dimension open in the field, its value not typed yet, is made again so.
    pub fn toggle_axis(&mut self, axis: Axis) {
        self.draft.axis = if self.draft.axis == Some(axis) { None } else { Some(axis) };
        if let (Some(f), Some(t)) = (&self.field, &self.draft.asked)
            && f.text == *t
        {
            let (picks, at, axis) = (self.draft.picks.clone(), f.at, self.draft.axis);
            if self.dimension_now(&picks, at, axis) {
                self.draft.asked = self.field.as_ref().map(|f| f.text.clone());
            }
        }
    }

    /// Where the field for `picks` stands: amid their middles.
    fn spot(&self, picks: &[Pick2]) -> Option<[f64; 2]> {
        let p = self.placed.as_ref().ok()?;
        let mut sum = [0.0, 0.0];
        for k in picks {
            let q = match k {
                Pick2::Point(n) => p.solved.points.get(n).map(|v| [v.x, v.y])?,
                Pick2::Curve(n) => SketchEdit::middle(p, n)?,
            };
            sum = [sum[0] + q[0], sum[1] + q[1]];
        }
        let n = picks.len() as f64;
        Some([sum[0] / n, sum[1] / n])
    }

    /// Opens the field for a new dimension `c` measuring `now`, by sketch point `at`.
    pub fn ask(&mut self, c: Json, key: &'static str, now: f64, at: [f64; 2]) {
        self.field = Some(Field { entry: Entry::New { constraint: c, key }, text: text_of(&json!(now)), at });
    }

    /// Opens the field to change the dimension under a double click: a constraint's value or a
    /// circle's diameter (of a circle equal to another, that one's).
    pub fn edit_label(&mut self, pick: &crate::sketch::Pick, at: [f64; 2]) {
        let followed = self.followed_circles();
        let sketch = &self.json["sketch"];
        let field = match pick {
            Pick::Constraint(k) => {
                let c = &sketch["constraints"][*k];
                c.get("value").or_else(|| c.get("degrees")).map(|v| Field { entry: Entry::Edit(*k), text: text_of(v), at })
            }
            Pick::Curve(n) if sketch["circles"].get(n).is_some() => {
                let n = followed.get(n).unwrap_or(n);
                Some(Field { entry: Entry::Diameter(n.clone()), text: text_of(&sketch["circles"][n]["diameter"]), at })
            }
            _ => None,
        };
        if field.is_some() {
            self.field = field;
        }
    }

    /// Applies the field (Enter): the `edit_sketch` parameters to send, applied here already;
    /// None when there is nothing to send (drawing, which the tool completes, or a conflict, now
    /// offered as a reference dimension).
    pub fn commit_field(&mut self) -> Option<Json> {
        let field = self.field.take()?;
        let value = value_of(&field.text);
        let sketch = &self.json["sketch"];
        let mut patch = Map::new();
        match &field.entry {
            Entry::Draw | Entry::Parameter(_) => {
                self.field = Some(field);
                return None;
            }
            Entry::New { constraint, key } => {
                let mut c = constraint.clone();
                c[*key] = value;
                patch.insert("add_constraints".into(), json!([c]));
            }
            Entry::Edit(k) => {
                let old = sketch["constraints"].get(*k)?.clone();
                let mut c = old.clone();
                let key = if c.get("degrees").is_some() { "degrees" } else { "value" };
                c[key] = value;
                patch.insert("remove_constraints".into(), json!([old]));
                patch.insert("add_constraints".into(), json!([c]));
            }
            Entry::Diameter(n) => {
                let mut circle = sketch["circles"].get(n)?.clone();
                circle["diameter"] = value;
                patch.insert("circles".into(), json!({ n.as_str(): circle }));
            }
        }
        let adds = matches!(field.entry, Entry::New { .. });
        match self.try_apply(patch, adds) {
            Ok(sent) => Some(sent),
            Err(why) => {
                let constraint = match &field.entry {
                    Entry::New { constraint, key } if !why.starts_with("invalid") => {
                        let mut c = constraint.clone();
                        c[*key] = value_of(&field.text);
                        c
                    }
                    _ => Json::Null,
                };
                self.conflict = Some(Conflict { constraint, why });
                None
            }
        }
    }

    /// Applies `patch` if the sketch still solves with it and, when it `adds` a constraint, is
    /// defined further by it: tried on a copy first, so one that cannot hold or only repeats
    /// what is defined already is not sent. The `edit_sketch` parameters to send, or why not.
    pub fn try_apply(&mut self, patch: Map<String, Json>, adds: bool) -> Result<Json, String> {
        let mut trial = self.clone_for_probe();
        trial.apply(patch.clone());
        let dof = |e: &SketchEdit| e.placed.as_ref().map(|p| p.solved.dof).ok();
        match &trial.placed {
            Err(why) => Err(why.clone()),
            Ok(_) if adds && dof(&trial) >= dof(self) => Err("the sketch is defined there already: this would only repeat it".into()),
            Ok(_) => Ok(self.apply(patch)),
        }
    }

    /// The conflicting dimension as a reference dimension instead: measured, holding nothing.
    pub fn conflict_as_reference(&mut self) -> Option<Json> {
        let c = self.conflict.take()?.constraint;
        let mut m = c.as_object()?.clone();
        m.remove("value");
        m.remove("degrees");
        let taken = self.json["sketch"]["measures"].as_object().cloned().unwrap_or_default();
        let name = (1..).map(|k| format!("d{k}")).find(|n| !taken.contains_key(n))?;
        let mut patch = Map::new();
        patch.insert("measures".into(), json!({ name: m }));
        Some(self.apply(patch))
    }
}

/// A dimension made horizontal or vertical wherever the cursor is (H, V while placing it).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Axis {
    Horizontal,
    Vertical,
}

/// An element the dimension tool picks: a point or a curve, by its own name.
#[derive(Clone, Debug, PartialEq)]
pub enum Pick2 {
    Point(String),
    Curve(String),
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use std::collections::BTreeMap;

    use datum_brep::TopoId;

    use super::*;
    use crate::sketch::Context;

    /// A rectangle 20 × 10, the lower left corner fixed, nothing else given.
    fn editor() -> SketchEdit {
        let json = json!({"name": "s", "op": "sketch", "inputs": [], "sketch": {
            "points": {"a": [0, 0], "b": [20, 0], "c": [20, 10], "d": [0, 10]},
            "lines": {"bottom": {"from": "a", "to": "b"}, "right": {"from": "b", "to": "c"}, "top": {"from": "c", "to": "d"}, "left": {"from": "d", "to": "a"}},
            "constraints": [{"type": "fix", "point": "a"}, {"type": "horizontal", "line": "bottom"}, {"type": "vertical", "line": "right"},
                            {"type": "horizontal", "line": "top"}, {"type": "vertical", "line": "left"}]}});
        SketchEdit::new(
            "s".into(),
            Context { id: TopoId::derive("feature").tag("s").finish(), input: None },
            json,
            BTreeMap::from([("w".into(), 25.0)]),
            None,
        )
    }

    #[test]
    fn values_are_numbers_or_expressions() {
        assert_eq!((value_of(" 12,5 "), value_of("w - 2")), (json!(12.5), json!("w - 2")));
    }

    /// Two points dimension as the cursor says: above them horizontal, beside vertical,
    /// straight off their middle along them.
    #[test]
    fn the_cursor_chooses_the_kind() {
        let e = editor();
        let pts = [Pick2::Point("a".into()), Pick2::Point("c".into())];
        let kind = |at: [f64; 2]| e.dimension(&pts, at, None).unwrap().0["type"].as_str().unwrap().to_string();
        assert_eq!(kind([10.0, 30.0]), "horizontal_distance");
        assert_eq!(kind([40.0, 5.0]), "vertical_distance");
        // On the perpendicular through their middle, (10, 5) + 10 · (−0.447, 0.894).
        assert_eq!(kind([5.53, 13.94]), "distance");
        let (c, key, now) = e.dimension(&[Pick2::Curve("bottom".into())], [10.0, -5.0], None).unwrap();
        assert_eq!((c, key), (json!({"type": "length", "line": "bottom"}), "value"));
        assert!((now - 20.0).abs() < 1e-9);
        let (_, key, now) = e.dimension(&[Pick2::Curve("bottom".into()), Pick2::Curve("left".into())], [1.0, 1.0], None).unwrap();
        assert_eq!(key, "degrees");
        assert!((now - 270.0).abs() < 1e-6, "{now}");
    }

    /// A new dimension goes in with the value typed; one that over-constrains is offered as a
    /// reference dimension instead.
    #[test]
    fn new_dimensions_and_conflicts() {
        let mut e = editor();
        let (c, key, now) = e.dimension(&[Pick2::Curve("bottom".into())], [10.0, -5.0], None).unwrap();
        e.ask(c, key, now, [10.0, -5.0]);
        assert_eq!(e.field.as_ref().unwrap().text, "20");
        e.field.as_mut().unwrap().text = "w".into();
        let sent = e.commit_field().unwrap();
        assert_eq!(sent["add_constraints"], json!([{"type": "length", "line": "bottom", "value": "w"}]));
        assert!((e.placed.as_ref().unwrap().solved.points["b"].x - 25.0).abs() < 1e-9);
        // The top is as long as the bottom already: its length too would only repeat it.
        let (c, key, now) = e.dimension(&[Pick2::Curve("top".into())], [10.0, 15.0], None).unwrap();
        e.ask(c, key, now, [10.0, 15.0]);
        assert!(e.commit_field().is_none());
        assert!(e.conflict.is_some());
        let sent = e.conflict_as_reference().unwrap();
        assert_eq!(sent["measures"], json!({"d1": {"type": "length", "line": "top"}}));
    }

    /// A click on a line dimensions it at once; two points wait for where the dimension goes.
    /// Two lines framed: their angle.
    #[test]
    fn curves_and_selections_dimension_at_once() {
        let mut e = editor();
        assert!(e.dimension_now(&[Pick2::Curve("bottom".into())], [10.0, 0.0], None));
        assert_eq!(e.field.as_ref().unwrap().entry, Entry::New { constraint: json!({"type": "length", "line": "bottom"}), key: "value" });
        e.field = None;
        assert!(!e.dimension_now(&[Pick2::Point("a".into()), Pick2::Point("c".into())], [10.0, 0.0], None));
        assert!(e.field.is_none());
        // A frame around two sides, their corners in it too: the angle of the sides.
        let framed = [Pick::Point("a".into()), Pick::Curve("bottom".into()), Pick::Curve("left".into()), Pick::Point("b".into())];
        assert!(e.dimension_framed(&framed, None).is_empty());
        let f = e.field.as_ref().unwrap();
        assert_eq!(f.entry, Entry::New { constraint: json!({"type": "angle", "lines": ["bottom", "left"]}), key: "degrees" });
        e.field = None;
        // Two points framed wait for where the dimension goes.
        assert_eq!(e.dimension_framed(&[Pick::Point("a".into()), Pick::Point("c".into())], None).len(), 2);
        assert!(e.field.is_none());
    }

    /// A double click on a dimension changes its value in place; on a circle, its diameter.
    #[test]
    fn values_change_in_place() {
        let mut e = editor();
        let (c, key, now) = e.dimension(&[Pick2::Curve("left".into())], [-5.0, 5.0], None).unwrap();
        e.ask(c, key, now, [-5.0, 5.0]);
        e.commit_field().unwrap();
        let k = e.json["sketch"]["constraints"].as_array().unwrap().len() - 1;
        e.edit_label(&Pick::Constraint(k), [0.0, 0.0]);
        assert_eq!(e.field.as_ref().unwrap().text, "10");
        e.field.as_mut().unwrap().text = "14".into();
        let sent = e.commit_field().unwrap();
        assert_eq!(sent["add_constraints"], json!([{"type": "length", "line": "left", "value": 14.0}]));
        assert!((e.placed.as_ref().unwrap().solved.points["d"].y - 14.0).abs() < 1e-9);
    }

    /// Two parallel lines: their distance. A slanted line: its length, with H or V the
    /// horizontal or vertical distance of its ends. An arc's two ends: the length along it.
    /// All at once, as a line is; two other points wait for where the dimension goes, unless
    /// H or V says it.
    #[test]
    fn distances_parallels_axes_and_arcs() {
        let mut e = editor();
        e.json["sketch"]["lines"]["diag"] = json!({"from": "a", "to": "c"});
        for (n, q) in [("o", [30, 0]), ("s", [35, 0]), ("t", [30, 5])] {
            e.json["sketch"]["points"][n] = json!(q);
        }
        e.json["sketch"]["arcs"] = json!({"bend": {"center": "o", "from": "s", "to": "t"}});
        e.solve();
        let curve = |n: &str| Pick2::Curve(n.into());
        let point = |n: &str| Pick2::Point(n.into());
        let made = |e: &SketchEdit, picks: &[Pick2], axis| e.dimension(picks, [0.0, 0.0], axis).map(|(c, _, now)| (c, now)).unwrap();
        let (c, now) = made(&e, &[curve("bottom"), curve("top")], None);
        assert_eq!(c, json!({"type": "point_line", "point": "a", "line": "top"}));
        assert!((now - 10.0).abs() < 1e-9, "{now}");
        assert_eq!(made(&e, &[curve("bottom"), curve("diag")], None).0["type"], "angle", "not parallel");
        assert_eq!(made(&e, &[curve("diag")], None).0, json!({"type": "length", "line": "diag"}));
        let (c, now) = made(&e, &[curve("diag")], Some(Axis::Horizontal));
        assert_eq!((c, now), (json!({"type": "horizontal_distance", "points": ["a", "c"]}), 20.0));
        let (c, now) = made(&e, &[curve("diag")], Some(Axis::Vertical));
        assert_eq!((c, now), (json!({"type": "vertical_distance", "points": ["a", "c"]}), 10.0));
        let (c, now) = made(&e, &[point("t"), point("s")], None);
        assert_eq!(c, json!({"type": "arc_length", "arc": "bend"}));
        assert!((now - 2.5 * std::f64::consts::PI).abs() < 1e-9, "{now}");
        assert!(e.dimension_now(&[point("s"), point("t")], [0.0, 0.0], None));
        let (c, now) = made(&e, &[curve("bend"), curve("bend")], None);
        assert_eq!(c, json!({"type": "arc_angle", "arc": "bend"}), "the arc twice: its angle");
        assert!((now - 90.0).abs() < 1e-9, "{now}");
        e.field = None;
        assert!(!e.dimension_now(&[point("a"), point("c")], [0.0, 0.0], None), "two points wait");
        assert!(e.dimension_now(&[point("a"), point("c")], [0.0, 0.0], Some(Axis::Vertical)), "unless V says");
    }

    /// H or V while the field shows a line's length, its value not typed yet: the dimension
    /// changes. Once a value is typed, the letter belongs to it.
    #[test]
    fn h_and_v_change_the_dimension_open() {
        let mut e = editor();
        e.json["sketch"]["lines"]["diag"] = json!({"from": "a", "to": "c"});
        e.solve();
        e.draft.picks = vec![Pick2::Curve("diag".into())];
        assert!(e.dimension_now(&e.draft.picks.clone(), [10.0, 5.0], None));
        e.draft.asked = e.field.as_ref().map(|f| f.text.clone());
        e.toggle_axis(Axis::Vertical);
        let entry = |e: &SketchEdit| match &e.field.as_ref().unwrap().entry {
            Entry::New { constraint, .. } => constraint["type"].clone(),
            _ => Json::Null,
        };
        assert_eq!((entry(&e), e.field.as_ref().unwrap().text.as_str()), (json!("vertical_distance"), "10"));
        e.draft.asked = e.field.as_ref().map(|f| f.text.clone());
        e.field.as_mut().unwrap().text = "w".into();
        e.toggle_axis(Axis::Horizontal);
        assert_eq!(entry(&e), json!("vertical_distance"), "typed: kept");
    }
}
