//! A sketch edited in the window (M7d1, D89): drawn over the model in its own orthographic view,
//! solved on the spot while a point is dragged (the same `Op::eval` the model uses), and changed
//! through the commands an agent uses (`edit_sketch`). What the window shows is what it writes:
//! when a drag ends, every point the solver moved is written where it stands.

use std::collections::BTreeMap;
use std::f64::consts::TAU;

use datum_brep::TopoId;
use datum_graph::{Op, Value, ValueRef};
use datum_ops::Placed;
use serde_json::{Value as Json, json};

use crate::camera::SketchCamera;

/// What the background thread knows of the sketch being edited: its feature's ID (for
/// `Op::eval`) and its input body (a face it lies on, edges it projects).
#[derive(Clone)]
pub struct Context {
    pub id: TopoId,
    pub input: Option<ValueRef>,
}

/// Something under the cursor or selected.
#[derive(Clone, Debug, PartialEq)]
pub enum Pick {
    Point(String),
    /// A line, arc or circle of the sketch.
    Curve(String),
    /// A dimension, by its place in the constraints.
    Constraint(usize),
    Measure(String),
    /// A projected edge or vertex, or one of its points.
    Projection(String),
}

pub struct SketchEdit {
    pub feature: String,
    pub context: Context,
    /// The feature as `inspect` lists it, with the current drag applied.
    pub json: Json,
    /// Parameter values, for the expressions of dimensions.
    pub params: BTreeMap<String, f64>,
    pub placed: Result<Placed, String>,
    pub camera: SketchCamera,
    pub hover: Option<Pick>,
    pub selected: Vec<Pick>,
    /// The point being dragged.
    pub drag: Option<String>,
    /// Show element names.
    pub names: bool,
}

/// Pixels within which the cursor is on a point, and on a curve or a label.
const NEAR_POINT: f64 = 8.0;
const NEAR_CURVE: f64 = 6.0;

const DEFINED: egui::Color32 = egui::Color32::from_rgb(24, 26, 30);
const FREE: egui::Color32 = egui::Color32::from_rgb(30, 90, 210);
const PROJECTED: egui::Color32 = egui::Color32::from_rgb(150, 110, 20);
const SELECTED: egui::Color32 = egui::Color32::from_rgb(235, 110, 20);
const DIMENSION: egui::Color32 = egui::Color32::from_rgb(20, 120, 60);
const NAME: egui::Color32 = egui::Color32::from_rgb(110, 110, 120);

/// A label as shown: its pixel offset from the middle of the picture, and its size.
pub type Laid = (Label, (f64, f64), (f64, f64));

/// A dimension's or reference dimension's label: what it picks, where, and its text.
pub struct Label {
    pub pick: Pick,
    pub at: [f64; 2],
    pub text: String,
}

impl SketchEdit {
    /// Starts editing feature `feature`, listed as `json`, viewed whole in a picture of width /
    /// height `aspect`; `scene` is the model's middle and size, for an empty sketch.
    pub fn new(feature: String, context: Context, json: Json, params: BTreeMap<String, f64>, scene: Option<(datum_math::Vec3, f64)>) -> Self {
        let camera = SketchCamera {
            frame: [
                datum_math::Vec3::ZERO,
                datum_math::Vec3::new(1.0, 0.0, 0.0),
                datum_math::Vec3::new(0.0, 1.0, 0.0),
                datum_math::Vec3::new(0.0, 0.0, 1.0),
            ],
            center: [0.0; 2],
            half_height: 50.0,
        };
        let mut s =
            Self { feature, context, json, params, placed: Err(String::new()), camera, hover: None, selected: Vec::new(), drag: None, names: true };
        s.solve();
        s.fit(scene);
        s
    }

    /// Looks at the whole sketch, or at the model's middle when the sketch is empty.
    pub fn fit(&mut self, scene: Option<(datum_math::Vec3, f64)>) {
        let Ok(p) = &self.placed else { return };
        self.camera.frame = p.frame;
        let pts: Vec<[f64; 2]> = p.solved.points.values().map(|v| [v.x, v.y]).collect();
        let [o, x, y, _] = p.frame;
        if pts.is_empty() {
            if let Some((middle, radius)) = scene {
                self.camera.center = [(middle - o).dot(x), (middle - o).dot(y)];
                self.camera.half_height = radius.max(f64::MIN_POSITIVE);
            }
            return;
        }
        let lo = pts.iter().fold([f64::INFINITY; 2], |m, p| [m[0].min(p[0]), m[1].min(p[1])]);
        let hi = pts.iter().fold([f64::NEG_INFINITY; 2], |m, p| [m[0].max(p[0]), m[1].max(p[1])]);
        self.camera.center = [(lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5];
        let half = 0.5 * (hi[0] - lo[0]).max(hi[1] - lo[1]);
        self.camera.half_height = if half > 0.0 { half * 1.4 } else { scene.map_or(10.0, |s| s.1 * 0.5) };
    }

    /// Takes the feature as the model now has it, unless a point is being dragged.
    pub fn refresh(&mut self, context: Context, json: Json, params: BTreeMap<String, f64>) {
        if self.drag.is_some() {
            return;
        }
        (self.context, self.json, self.params) = (context, json, params);
        self.solve();
    }

    /// The feature's operation, from its JSON.
    fn op(&self) -> Result<Op, String> {
        let mut fields = self.json.as_object().cloned().unwrap_or_default();
        fields.remove("name");
        fields.remove("inputs");
        fields.remove("status");
        fields.remove("error");
        serde_json::from_value(Json::Object(fields)).map_err(|e| e.to_string())
    }

    /// Solves and places the sketch as the model would.
    pub fn solve(&mut self) {
        self.placed = self.placed_with(None);
    }

    /// The sketch solved and placed, with point `pin.0` held at `pin.1` if given (a drag).
    fn placed_with(&self, pin: Option<(&str, [f64; 2])>) -> Result<Placed, String> {
        let lookup = |n: &str| self.params.get(n).copied().ok_or_else(|| datum_math::expr::ExprError::Unknown(n.to_string()));
        let mut op = self.op()?;
        if let (Some((n, at)), Op::Sketch { sketch, .. }) = (pin, &mut op) {
            sketch.constraints.push(datum_sketch::Constraint::Fix { point: n.into(), at: Some(datum_math::Vec2::new(at[0], at[1])) });
        }
        let op = op.resolve(&lookup)?;
        let inputs: Vec<&Value> = self.context.input.iter().map(|v| v.as_ref()).collect();
        match op.eval(self.context.id, &inputs).map_err(|e| e.to_string())? {
            Value::Sketch(p) => Ok(p),
            _ => Err("not a sketch".into()),
        }
    }

    /// The sketch's own points (not projections'), by name, as drawn.
    fn drawn(&self) -> BTreeMap<String, [f64; 2]> {
        self.json["sketch"]["points"]
            .as_object()
            .map(|m| m.iter().filter_map(|(n, v)| Some((n.clone(), [v[0].as_f64()?, v[1].as_f64()?]))).collect())
            .unwrap_or_default()
    }

    fn projections(&self) -> Vec<String> {
        self.json["project"].as_object().map(|m| m.keys().cloned().collect()).unwrap_or_default()
    }

    /// The projection a name is of: the projection itself or one of its points.
    fn projection_of(&self, name: &str) -> Option<String> {
        self.projections().into_iter().find(|p| name == p || name.strip_prefix(p.as_str()).is_some_and(|r| r.starts_with('.')))
    }

    /// Starts dragging the point under the cursor, if it is the sketch's own and can move.
    pub fn start_drag(&mut self) -> bool {
        let free = |n: &String| self.placed.as_ref().is_ok_and(|p| p.solved.free.contains_key(n));
        match &self.hover {
            Some(Pick::Point(n)) if self.drawn().contains_key(n) && free(n) => {
                self.drag = Some(n.clone());
                true
            }
            _ => false,
        }
    }

    /// Moves the dragged point toward sketch point `p`, as the cursor: the point stays under it
    /// if the constraints let it; if not, it goes as far along the direction it is free in
    /// (D89); failing that, the solver places it as drawn there. The others follow.
    pub fn drag_to(&mut self, p: [f64; 2]) {
        let Some(n) = self.drag.clone() else { return };
        self.json["sketch"]["points"][&n] = json!(p);
        // The direction it is free in, from the sketch solved without a pin: with one, the
        // point would have none, and frames would alternate between two placements.
        let plain = self.placed_with(None);
        let along = plain.as_ref().ok().and_then(|pl| Some((pl.solved.points.get(&n).copied()?, pl.solved.free.get(&n).copied()?)));
        let mut tries = vec![p];
        if let Some((at, Some(d))) = along {
            let t = (p[0] - at.x) * d.x + (p[1] - at.y) * d.y;
            tries.push([at.x + d.x * t, at.y + d.y * t]);
        }
        self.placed = tries.into_iter().map(|q| self.placed_with(Some((&n, q)))).find(Result::is_ok).unwrap_or(plain);
    }

    /// Ends the drag: `edit_sketch` parameters writing every own point where it now stands, or
    /// None if nothing moved. The window keeps showing the same until the model comes back.
    pub fn end_drag(&mut self) -> Option<Json> {
        self.drag.take()?;
        let Ok(placed) = &self.placed else { return None };
        let mut patch = serde_json::Map::new();
        for (n, d) in self.drawn() {
            if let Some(s) = placed.solved.points.get(&n)
                && [s.x, s.y] != d
            {
                patch.insert(n.clone(), json!([s.x, s.y]));
            }
        }
        if patch.is_empty() {
            return None;
        }
        for (n, v) in &patch {
            self.json["sketch"]["points"][n] = v.clone();
        }
        // Without the cursor's pin: what is still free shows blue again.
        self.solve();
        Some(json!({"name": self.feature, "points": patch}))
    }

    /// `edit_sketch` parameters removing what is selected, or None if nothing is.
    pub fn delete(&self) -> Option<Json> {
        let mut params = serde_json::Map::new();
        let mut remove = |key: &str, n: &str| {
            params.entry(key).or_insert_with(|| json!({}))[n] = Json::Null;
        };
        let mut constraints = Vec::new();
        let sketch = &self.json["sketch"];
        for p in &self.selected {
            match p {
                Pick::Point(n) => remove("points", n),
                Pick::Curve(n) => {
                    let key = ["lines", "arcs", "circles"].into_iter().find(|k| sketch[k].get(n).is_some()).unwrap_or("lines");
                    remove(key, n);
                }
                Pick::Measure(n) => remove("measures", n),
                Pick::Projection(n) => remove("project", n),
                Pick::Constraint(k) => constraints.extend(sketch["constraints"].get(*k).cloned()),
            }
        }
        if !constraints.is_empty() {
            params.insert("remove_constraints".into(), json!(constraints));
        }
        if params.is_empty() {
            return None;
        }
        params.insert("name".into(), json!(self.feature));
        Some(Json::Object(params))
    }

    /// The points of a curve of the placed sketch, in order, as polyline points: a line its two
    /// ends, an arc or circle chords (64 a turn).
    fn curve_points(p: &Placed, name: &str) -> Option<Vec<[f64; 2]>> {
        let at = |n: &str| p.solved.points.get(n).map(|v| [v.x, v.y]);
        if let Some(l) = p.lines.get(name) {
            return Some(vec![at(&l.from)?, at(&l.to)?]);
        }
        let (c, r, start, sweep) = if let Some(a) = p.arcs.get(name) {
            let (c, f, t) = (at(&a.center)?, at(&a.from)?, at(&a.to)?);
            let start = libm::atan2(f[1] - c[1], f[0] - c[0]);
            let sweep = (libm::atan2(t[1] - c[1], t[0] - c[0]) - start).rem_euclid(TAU);
            (c, libm::hypot(f[0] - c[0], f[1] - c[1]), start, if sweep == 0.0 { TAU } else { sweep })
        } else {
            let k = p.circles.get(name)?;
            (at(&k.center)?, 0.5 * k.diameter.value()?, 0.0, TAU)
        };
        let n = (sweep / TAU * 64.0).ceil().max(1.0) as usize;
        Some(
            (0..=n)
                .map(|k| {
                    let (sn, cs) = libm::sincos(start + sweep * k as f64 / n as f64);
                    [c[0] + r * cs, c[1] + r * sn]
                })
                .collect(),
        )
    }

    /// The middle of a curve: of a line between its ends, of an arc or circle on it.
    fn middle(p: &Placed, name: &str) -> Option<[f64; 2]> {
        let v = Self::curve_points(p, name)?;
        Some(match v[..] {
            [a, b] => [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5],
            _ => v[v.len() / 2],
        })
    }

    /// The points a curve is drawn through: free if any of them is.
    fn curve_ends(p: &Placed, name: &str) -> Vec<String> {
        if let Some(l) = p.lines.get(name) {
            return vec![l.from.clone(), l.to.clone()];
        }
        if let Some(a) = p.arcs.get(name) {
            return vec![a.center.clone(), a.from.clone(), a.to.clone()];
        }
        p.circles.get(name).map(|c| vec![c.center.clone()]).unwrap_or_default()
    }

    /// Labels of the dimensions and reference dimensions, placed by what they measure.
    pub fn labels(&self) -> Vec<Label> {
        let Ok(p) = &self.placed else { return Vec::new() };
        let at = |n: &Json| n.as_str().and_then(|n| p.solved.points.get(n)).map(|v| [v.x, v.y]);
        let mid = |a: [f64; 2], b: [f64; 2]| [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5];
        let line = |n: &Json| n.as_str().and_then(|n| Self::curve_points(p, n)).filter(|v| v.len() == 2).map(|v| (v[0], v[1]));
        let lookup = |n: &str| self.params.get(n).copied().ok_or_else(|| datum_math::expr::ExprError::Unknown(n.to_string()));
        let shown = |v: &Json| match v {
            Json::String(e) => {
                let value = serde_json::from_value::<datum_math::expr::Num>(v.clone()).ok().and_then(|n| n.resolve(&lookup).ok());
                value.map_or_else(|| e.clone(), |x| format!("{e} = {}", short(x)))
            }
            v => v.as_f64().map_or_else(String::new, short),
        };
        // Where a dimension's label stands: by its points, its line, or its arc.
        let place = |c: &Json| -> Option<[f64; 2]> {
            match c["type"].as_str()? {
                "distance" | "horizontal_distance" | "vertical_distance" => Some(mid(at(&c["points"][0])?, at(&c["points"][1])?)),
                "length" => c["line"].as_str().and_then(|n| Self::middle(p, n)),
                "point_line" => {
                    let (q, (a, b)) = (at(&c["point"])?, line(&c["line"])?);
                    let (d, w) = ([b[0] - a[0], b[1] - a[1]], [q[0] - a[0], q[1] - a[1]]);
                    let t = (d[0] * w[0] + d[1] * w[1]) / (d[0] * d[0] + d[1] * d[1]);
                    Some(mid(q, [a[0] + d[0] * t, a[1] + d[1] * t]))
                }
                "angle" => line(&c["lines"][0]).map(|(a, _)| a),
                "radius" => c["arc"].as_str().and_then(|n| Self::middle(p, n)),
                _ => None,
            }
        };
        let prefix = |c: &Json| match c["type"].as_str() {
            Some("radius") => "R",
            Some("angle") => "∠",
            _ => "",
        };
        let mut out = Vec::new();
        for (k, c) in self.json["sketch"]["constraints"].as_array().into_iter().flatten().enumerate() {
            let value = c.get("value").or_else(|| c.get("degrees"));
            if let (Some(v), Some(at)) = (value, place(c)) {
                out.push(Label {
                    pick: Pick::Constraint(k),
                    at,
                    text: format!("{}{}{}", prefix(c), shown(v), if c["type"] == "angle" { "°" } else { "" }),
                });
            }
        }
        for (n, c) in self.json["sketch"]["measures"].as_object().into_iter().flatten() {
            if let (Some(at), Some(v)) = (place(c), p.solved.measures.get(n)) {
                out.push(Label { pick: Pick::Measure(n.clone()), at, text: format!("({}{})", prefix(c), short(*v)) });
            }
        }
        for (n, c) in &p.circles {
            if let Some(v) = Self::curve_points(p, n) {
                let d = self.json["sketch"]["circles"][n]["diameter"].clone();
                let text = if d.is_null() { short(c.diameter.value().unwrap_or(0.0)) } else { shown(&d) };
                // Ø: the window's font has no ⌀.
                out.push(Label { pick: Pick::Curve(n.clone()), at: v[v.len() / 4], text: format!("Ø{text}") });
            }
        }
        out
    }

    /// The labels as shown in a picture `height` tall: each at its pixel offset from the middle,
    /// with its size (`size` of its text, padded). One that would cover an earlier one moves
    /// down until it does not: two dimensions between the same points both read.
    pub fn laid_out(&self, height: f64, size: impl Fn(&str) -> (f64, f64)) -> Vec<Laid> {
        let mut placed: Vec<Laid> = Vec::new();
        for l in self.labels() {
            let (w, h) = size(&l.text);
            let (x, mut y) = self.camera.offset(l.at, height);
            while placed.iter().any(|(_, (px, py), (pw, ph))| (x - px).abs() * 2.0 < w + pw && (y - py).abs() * 2.0 < h + ph) {
                y += h + 1.0;
            }
            placed.push((l, (x, y), (w, h)));
        }
        placed
    }

    /// What is under pixel offset `(dx, dy)` from the middle of a picture `height` tall: a point
    /// first, then a label, then a curve.
    pub fn hit(&self, dx: f64, dy: f64, height: f64, label_size: impl Fn(&str) -> (f64, f64)) -> Option<Pick> {
        let Ok(p) = &self.placed else { return None };
        let near = |q: [f64; 2]| {
            let (x, y) = self.camera.offset(q, height);
            libm::hypot(x - dx, y - dy)
        };
        let point = p
            .solved
            .points
            .iter()
            .map(|(n, v)| (near([v.x, v.y]), n))
            .filter(|(d, _)| *d <= NEAR_POINT)
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map(|(_, n)| n.clone());
        if let Some(n) = point {
            return Some(self.projection_of(&n).map_or(Pick::Point(n), Pick::Projection));
        }
        for (l, (x, y), (w, h)) in self.laid_out(height, label_size) {
            if (dx - x).abs() <= w * 0.5 && (dy - y).abs() <= h * 0.5 {
                return Some(l.pick);
            }
        }
        let curves = p.lines.keys().chain(p.arcs.keys()).chain(p.circles.keys());
        let distance = |n: &String| {
            let pts = Self::curve_points(p, n)?;
            let screen: Vec<(f64, f64)> = pts.iter().map(|q| self.camera.offset(*q, height)).collect();
            screen.windows(2).map(|w| segment_distance((dx, dy), w[0], w[1])).min_by(f64::total_cmp)
        };
        let curve = curves
            .filter_map(|n| Some((distance(n)?, n)))
            .filter(|(d, _)| *d <= NEAR_CURVE)
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map(|(_, n)| n.clone());
        curve.map(|n| self.projection_of(&n).map_or(Pick::Curve(n), Pick::Projection))
    }

    /// Draws the sketch over the view in `rect`.
    pub fn paint(&self, painter: &egui::Painter, rect: egui::Rect) {
        let Ok(p) = &self.placed else { return };
        let height = f64::from(rect.height());
        let screen = |q: [f64; 2]| {
            let (x, y) = self.camera.offset(q, height);
            rect.center() + egui::vec2(x as f32, y as f32)
        };
        let marked = |pick: &Pick| self.selected.contains(pick) || self.hover.as_ref() == Some(pick);
        let colour = |pick: &Pick, ends: &[String]| {
            if self.selected.contains(pick) {
                SELECTED
            } else if matches!(pick, Pick::Projection(_)) {
                PROJECTED
            } else if ends.iter().any(|e| p.solved.free.contains_key(e)) {
                FREE
            } else {
                DEFINED
            }
        };
        let laid = self.laid_out(height, |t| label_size(painter, t));
        let to_rect = |(x, y): (f64, f64), (w, h): (f64, f64)| {
            egui::Rect::from_center_size(rect.center() + egui::vec2(x as f32, y as f32), egui::vec2(w as f32, h as f32))
        };
        let taken: Vec<egui::Rect> = laid.iter().map(|(_, at, size)| to_rect(*at, *size)).collect();
        let construction = |n: &str| {
            p.lines
                .get(n)
                .map(|l| l.construction)
                .or_else(|| p.arcs.get(n).map(|a| a.construction))
                .or_else(|| p.circles.get(n).map(|c| c.construction))
                .unwrap_or(false)
        };
        for n in p.lines.keys().chain(p.arcs.keys()).chain(p.circles.keys()) {
            let Some(pts) = Self::curve_points(p, n) else { continue };
            let pick = self.projection_of(n).map_or(Pick::Curve(n.clone()), Pick::Projection);
            let width = if marked(&pick) { 3.0 } else { 1.6 };
            let stroke = egui::Stroke::new(width, colour(&pick, &Self::curve_ends(p, n)));
            let pts: Vec<egui::Pos2> = pts.into_iter().map(screen).collect();
            if construction(n) {
                painter.extend(egui::Shape::dashed_line(&pts, stroke, 6.0, 4.0));
            } else {
                painter.add(egui::Shape::line(pts, stroke));
            }
            // A name beside the middle of its curve, off to its left: the middle is a dimension's.
            if self.names
                && let (Some(m), Some(v)) = (Self::middle(p, n), Self::curve_points(p, n))
            {
                // Beside a line; outside an arc's middle, or a circle at the upper right.
                let centre = p.arcs.get(n).map(|a| &a.center).or_else(|| p.circles.get(n).map(|c| &c.center)).and_then(|c| p.solved.points.get(c));
                let (m, side) = match centre {
                    Some(c) => {
                        let m = if p.circles.contains_key(n) { v[v.len() / 8] } else { m };
                        (m, (screen(m) - screen([c.x, c.y])).normalized())
                    }
                    None => {
                        let d = screen(v[v.len() - 1]) - screen(v[0]);
                        (m, egui::vec2(d.y, -d.x).normalized())
                    }
                };
                let galley = painter.layout_no_wrap(n.clone(), egui::FontId::proportional(11.0), NAME);
                // Clear of the line by its half size that way, and more; on the other side if a
                // dimension's label is there, left out if both are taken: the dimension matters more.
                let reach = 10.0 + (side.x.abs() * galley.size().x + side.y.abs() * galley.size().y) * 0.5;
                let at = [side, -side]
                    .into_iter()
                    .map(|s| egui::Rect::from_center_size(screen(m) + s * reach, galley.size()))
                    .find(|r| !taken.iter().any(|t| t.intersects(*r)));
                if let Some(r) = at {
                    painter.galley(r.min, galley, NAME);
                }
            }
        }
        for (n, v) in &p.solved.points {
            let pick = self.projection_of(n).map_or(Pick::Point(n.clone()), Pick::Projection);
            let c = colour(&pick, std::slice::from_ref(n));
            let r = if marked(&pick) { 4.5 } else { 3.0 };
            let at = screen([v.x, v.y]);
            painter.rect_filled(egui::Rect::from_center_size(at, egui::vec2(2.0 * r, 2.0 * r)), 0.0, c);
            if self.names && self.drawn().contains_key(n) {
                painter.text(at + egui::vec2(5.0, 5.0), egui::Align2::LEFT_TOP, n, egui::FontId::proportional(11.0), NAME);
            }
        }
        for (l, at, size) in laid {
            let c = if self.selected.contains(&l.pick) { SELECTED } else { DIMENSION };
            let r = to_rect(at, size);
            painter.rect_filled(r, 2.0, egui::Color32::from_rgba_unmultiplied(255, 255, 255, 220));
            if marked(&l.pick) {
                painter.rect_stroke(r, 2.0, egui::Stroke::new(1.0, c), egui::StrokeKind::Inside);
            }
            painter.galley(r.min + egui::vec2(2.0, 1.0), painter.layout_no_wrap(l.text, LABEL, c), c);
        }
    }
}

/// The font of dimension labels.
const LABEL: egui::FontId = egui::FontId::proportional(13.0);

/// A label's size in pixels as drawn: its text, padded.
pub fn label_size(painter: &egui::Painter, text: &str) -> (f64, f64) {
    let g = painter.layout_no_wrap(text.to_string(), LABEL, egui::Color32::BLACK);
    (f64::from(g.size().x) + 4.0, f64::from(g.size().y) + 2.0)
}

/// A number as short as it reads: up to six decimals, no trailing zeros.
fn short(x: f64) -> String {
    let s = format!("{:.6}", x);
    let s = s.trim_end_matches('0').trim_end_matches('.');
    if s == "-0" { "0".into() } else { s.into() }
}

/// Distance from `p` to the segment `a`–`b`.
fn segment_distance(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let len2 = dx * dx + dy * dy;
    let t = if len2 > 0.0 { (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / len2).clamp(0.0, 1.0) } else { 0.0 };
    libm::hypot(p.0 - a.0 - t * dx, p.1 - a.1 - t * dy)
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    #[test]
    fn numbers_read_short() {
        assert_eq!([short(12.0), short(11.999_999_999_999_998), short(2.5), short(-0.000_000_1)], ["12", "12", "2.5", "0"]);
    }

    /// A rectangle with its width given and its height free, as `inspect` lists it.
    fn edit() -> SketchEdit {
        let json = json!({"name": "outline", "op": "sketch", "inputs": [], "sketch": {
            "points": {"a": [0, 0], "b": [12, 0], "c": [12, 4], "d": [0, 4]},
            "lines": {"bottom": {"from": "a", "to": "b"}, "right": {"from": "b", "to": "c"}, "top": {"from": "c", "to": "d"}, "left": {"from": "d", "to": "a"}},
            "constraints": [{"type": "fix", "point": "a"}, {"type": "horizontal", "line": "bottom"}, {"type": "vertical", "line": "right"},
                            {"type": "horizontal", "line": "top"}, {"type": "vertical", "line": "left"}, {"type": "length", "line": "bottom", "value": "w"}],
            "measures": {"height": {"type": "length", "line": "left"}}}});
        let id = TopoId::derive("feature").tag("outline").finish();
        SketchEdit::new("outline".into(), Context { id, input: None }, json, BTreeMap::from([("w".into(), 12.0)]), None)
    }

    #[test]
    fn solves_as_the_model_does() {
        let e = edit();
        let p = e.placed.as_ref().unwrap();
        assert!((p.solved.points["b"].x - 12.0).abs() < 1e-9);
        // Only the height is free: the top corners slide up and down.
        assert_eq!(p.solved.free.keys().collect::<Vec<_>>(), ["c", "d"]);
        let texts: Vec<String> = e.labels().into_iter().map(|l| l.text).collect();
        assert_eq!(texts, ["w = 12", "(4)"]);
    }

    /// Dragging the free top corner writes the corners where they stand, nothing else.
    #[test]
    fn a_drag_writes_what_it_shows() {
        let mut e = edit();
        let (dx, dy) = e.camera.offset([12.0, 4.0], 600.0);
        e.hover = e.hit(dx, dy, 600.0, |_| (0.0, 0.0));
        assert_eq!(e.hover, Some(Pick::Point("c".into())));
        assert!(e.start_drag());
        e.drag_to([13.0, 7.0]);
        let patch = e.end_drag().unwrap();
        // The corner keeps the width and the right angles: it rises to 7, its neighbour with it.
        let near = |v: &Json, want: f64| (v.as_f64().unwrap() - want).abs() < 1e-9;
        assert!(near(&patch["points"]["c"][0], 12.0) && near(&patch["points"]["c"][1], 7.0), "{patch}");
        assert!(near(&patch["points"]["d"][1], 7.0), "{patch}");
        assert!(patch["points"].get("a").is_none(), "{patch}");
        // Let go, the height is free again: the cursor's pin is gone.
        assert_eq!(e.placed.as_ref().map(|p| p.solved.dof), Ok(1));
        // A defined point does not drag.
        e.hover = Some(Pick::Point("b".into()));
        assert!(!e.start_drag());
    }

    /// Dragged where it cannot go, frame after frame, a point stays put: no flicker between the
    /// pinned and the projected placement.
    #[test]
    fn a_drag_is_steady() {
        let mut e = edit();
        e.hover = Some(Pick::Point("c".into()));
        assert!(e.start_drag());
        let at = |e: &SketchEdit| e.placed.as_ref().map(|p| p.solved.points["c"]).unwrap();
        e.drag_to([15.0, 7.0]);
        let first = at(&e);
        for _ in 0..4 {
            e.drag_to([15.0, 7.0]);
            assert_eq!(at(&e), first);
        }
        // The width is given: the corner slides up its side to the cursor's height.
        assert!((first.x - 12.0).abs() < 1e-9 && (first.y - 7.0).abs() < 1e-9, "{first:?}");
    }

    #[test]
    fn deleting_the_selection() {
        let mut e = edit();
        e.selected = vec![Pick::Curve("top".into()), Pick::Constraint(5), Pick::Measure("height".into())];
        let d = e.delete().unwrap();
        assert_eq!(d["lines"], json!({"top": null}));
        assert_eq!(d["measures"], json!({"height": null}));
        assert_eq!(d["remove_constraints"], json!([{"type": "length", "line": "bottom", "value": "w"}]));
        e.selected.clear();
        assert!(e.delete().is_none());
    }

    /// The sketch as it is drawn over the view; `DATUM_SKETCH_SHOT=file.png` saves a picture.
    #[test]
    fn draws_over_the_view() {
        let mut e = edit();
        e.selected = vec![Pick::Curve("top".into())];
        e.hover = Some(Pick::Constraint(5));
        let shot = std::env::var_os("DATUM_SKETCH_SHOT");
        let builder = egui_kittest::Harness::builder().with_size([600.0, 400.0]);
        let builder = if shot.is_some() { builder.wgpu() } else { builder };
        let mut h = builder.build_ui(|ui| {
            let (rect, _) = ui.allocate_exact_size(ui.available_size(), egui::Sense::hover());
            ui.painter().rect_filled(rect, 0.0, egui::Color32::from_rgb(235, 238, 242));
            e.paint(&ui.painter().with_clip_rect(rect), rect);
        });
        h.run();
        if let Some(path) = shot {
            h.render().unwrap().save(path).unwrap();
        }
    }

    /// The plate of `symmetric_plate.json`: no two labels cover each other, even the two between
    /// the same points. `DATUM_PLATE_SHOT=file.png` saves a picture.
    #[test]
    fn labels_do_not_cover_each_other() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/models/symmetric_plate.json");
        let model: Json = serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
        let params = model["parameters"].as_object().unwrap().iter().map(|(n, v)| (n.clone(), v.as_f64().unwrap())).collect();
        let id = TopoId::derive("feature").tag("outline").finish();
        let mut e = SketchEdit::new("outline".into(), Context { id, input: None }, model["features"][0].clone(), params, None);
        e.camera.half_height = 30.0;
        let shot = std::env::var_os("DATUM_PLATE_SHOT");
        let builder = egui_kittest::Harness::builder().with_size([1100.0, 630.0]);
        let builder = if shot.is_some() { builder.wgpu() } else { builder };
        let mut laid = Vec::new();
        let mut h = builder.build_ui(|ui| {
            let (rect, _) = ui.allocate_exact_size(ui.available_size(), egui::Sense::hover());
            ui.painter().rect_filled(rect, 0.0, egui::Color32::from_rgb(235, 238, 242));
            e.paint(&ui.painter().with_clip_rect(rect), rect);
            laid =
                e.laid_out(f64::from(rect.height()), |t| label_size(ui.painter(), t)).into_iter().map(|(l, at, size)| (l.text, at, size)).collect();
        });
        h.run();
        if let Some(path) = shot {
            h.render().unwrap().save(path).unwrap();
        }
        drop(h);
        assert!(laid.iter().any(|(t, ..)| t == "Øhole = 9"), "{:?}", laid.iter().map(|l| &l.0).collect::<Vec<_>>());
        for (i, (a, pa, sa)) in laid.iter().enumerate() {
            for (b, pb, sb) in &laid[i + 1..] {
                let apart = (pa.0 - pb.0).abs() * 2.0 >= sa.0 + sb.0 || (pa.1 - pb.1).abs() * 2.0 >= sa.1 + sb.1;
                assert!(apart, "`{a}` covers `{b}`");
            }
        }
    }

    #[test]
    fn labels_and_curves_are_hit() {
        let e = edit();
        let label = e.labels().into_iter().find(|l| l.pick == Pick::Constraint(5)).unwrap();
        let (x, y) = e.camera.offset(label.at, 600.0);
        assert_eq!(e.hit(x + 3.0, y, 600.0, |_| (40.0, 14.0)), Some(Pick::Constraint(5)));
        let (x, y) = e.camera.offset([12.0, 2.0], 600.0);
        assert_eq!(e.hit(x + 2.0, y, 600.0, |_| (0.0, 0.0)), Some(Pick::Curve("right".into())));
        assert_eq!(e.hit(x + 40.0, y, 600.0, |_| (0.0, 0.0)), None);
    }
}
