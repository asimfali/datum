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
use crate::draw::{Draft, face_edges};

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
    /// The regions the profiles bound, as triangles in sketch coordinates: filled, holes left
    /// open, so a hole pushed out of its outline shows as the disc it becomes.
    pub fill: Vec<[[f64; 2]; 3]>,
    pub camera: SketchCamera,
    pub hover: Option<Pick>,
    pub selected: Vec<Pick>,
    /// The point being dragged.
    pub drag: Option<String>,
    /// The selection frame being dragged.
    pub frame: Option<SelectFrame>,
    /// Show element names.
    pub names: bool,
    /// Show the badges of geometric constraints.
    pub constraints: bool,
    /// The drawing tool and what it has drawn so far.
    pub draft: Draft,
    /// The field by the cursor (a value typed while drawing, a dimension's value), and a new
    /// dimension that would over-constrain the sketch.
    pub field: Option<crate::dims::Field>,
    pub conflict: Option<crate::dims::Conflict>,
}

/// A selection frame: where it began and where the cursor is, pixels from the middle of the
/// view; `touching` when it takes what it touches whichever way it is dragged (the dimension
/// tool), not only to the left.
#[derive(Clone, Copy, Debug)]
pub struct SelectFrame {
    pub ends: [(f64, f64); 2],
    pub touching: bool,
}

impl SelectFrame {
    /// Whether it takes whatever it touches, rather than only what lies wholly inside.
    fn crossing(&self) -> bool {
        self.touching || self.ends[1].0 < self.ends[0].0
    }
}

/// What a geometric constraint's badge shows: drawn in lines, as the window's font has few of
/// the symbols (∥, ⊥).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Glyph {
    Horizontal,
    Vertical,
    Parallel,
    Perpendicular,
    Collinear,
    Equal,
    Fix,
    Coincident,
    Tangent,
    On,
    Midpoint,
    Symmetric,
    Concentric,
}

/// A geometric constraint's badge by one of the elements it ties: the constraint, its glyph,
/// and where it stands, pixels from the middle of the view.
#[derive(Clone, Debug, PartialEq)]
pub struct Badge {
    pub pick: Pick,
    pub glyph: Glyph,
    pub at: (f64, f64),
}

/// A badge's side, pixels.
const BADGE: f64 = 14.0;

/// The glyph of a geometric constraint and the elements its badges stand by; None for a
/// dimension, which has its label.
fn glyph_of(c: &Json) -> Option<(Glyph, Vec<String>)> {
    let names = |key: &str| -> Vec<String> {
        match &c[key] {
            Json::String(n) => vec![n.clone()],
            Json::Array(a) => a.iter().filter_map(|n| n.as_str().map(str::to_string)).collect(),
            _ => Vec::new(),
        }
    };
    let line_or_points = || if c["line"].is_string() { names("line") } else { names("points") };
    let (g, on) = match c["type"].as_str()? {
        "horizontal" => (Glyph::Horizontal, line_or_points()),
        "vertical" => (Glyph::Vertical, line_or_points()),
        "parallel" => (Glyph::Parallel, names("lines")),
        "perpendicular" => (Glyph::Perpendicular, names("lines")),
        "collinear" => (Glyph::Collinear, names("lines")),
        "equal" => (Glyph::Equal, names("lines")),
        "equal_radius" => (Glyph::Equal, names("curves")),
        "fix" => (Glyph::Fix, names("point")),
        "coincident" => (Glyph::Coincident, names("points").into_iter().take(1).collect()),
        "tangent" => (Glyph::Tangent, names("curves")),
        "on" => (Glyph::On, names("point")),
        "midpoint" => (Glyph::Midpoint, names("point")),
        "symmetric" => (Glyph::Symmetric, names("points")),
        "concentric" => (Glyph::Concentric, names("curves")),
        _ => return None,
    };
    Some((g, on))
}

/// Draws glyph `g` in a badge centred at `c`.
pub(crate) fn paint_glyph(painter: &egui::Painter, c: egui::Pos2, g: Glyph, colour: egui::Color32) {
    let s = 4.5;
    let p = |x: f32, y: f32| c + egui::vec2(x * s, y * s);
    let stroke = egui::Stroke::new(1.3, colour);
    let line = |a: (f32, f32), b: (f32, f32)| {
        painter.line_segment([p(a.0, a.1), p(b.0, b.1)], stroke);
    };
    match g {
        Glyph::Horizontal => line((-1.0, 0.0), (1.0, 0.0)),
        Glyph::Vertical => line((0.0, -1.0), (0.0, 1.0)),
        Glyph::Parallel => {
            line((-0.8, 1.0), (-0.1, -1.0));
            line((0.1, 1.0), (0.8, -1.0));
        }
        Glyph::Perpendicular => {
            line((-1.0, 1.0), (1.0, 1.0));
            line((0.0, 1.0), (0.0, -1.0));
        }
        Glyph::Collinear => {
            for x in [-1.0, -0.2, 0.6] {
                line((x, 0.0), (x + 0.4, 0.0));
            }
        }
        Glyph::Equal => {
            line((-1.0, -0.4), (1.0, -0.4));
            line((-1.0, 0.4), (1.0, 0.4));
        }
        Glyph::Fix => {
            painter.rect_stroke(egui::Rect::from_two_pos(p(-0.8, -0.1), p(0.8, 1.0)), 0.0, stroke, egui::StrokeKind::Middle);
            line((-0.5, -0.1), (-0.5, -0.8));
            line((-0.5, -0.8), (0.5, -0.8));
            line((0.5, -0.8), (0.5, -0.1));
        }
        Glyph::Coincident => {
            painter.rect_filled(egui::Rect::from_center_size(c, egui::vec2(s, s)), 0.0, colour);
        }
        Glyph::Tangent => {
            painter.circle_stroke(p(0.0, 0.35), 0.65 * s, stroke);
            line((-1.0, -0.3), (1.0, -0.3));
        }
        Glyph::On => {
            painter.circle_stroke(c, 0.8 * s, stroke);
            painter.circle_filled(p(0.0, -0.8), 0.35 * s, colour);
        }
        Glyph::Midpoint => {
            line((-0.9, 0.7), (0.9, 0.7));
            line((0.9, 0.7), (0.0, -0.8));
            line((0.0, -0.8), (-0.9, 0.7));
        }
        Glyph::Symmetric => {
            for x in [-1.0, 1.0] {
                line((x, -1.0), (x, 1.0));
                line((x, -1.0), (x * 0.5, -1.0));
                line((x, 1.0), (x * 0.5, 1.0));
            }
        }
        Glyph::Concentric => {
            painter.circle_stroke(c, 0.95 * s, stroke);
            painter.circle_stroke(c, 0.4 * s, stroke);
        }
    }
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
const FILL: egui::Color32 = egui::Color32::from_rgba_unmultiplied_const(60, 110, 200, 45);

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
        let mut s = Self {
            feature,
            context,
            json,
            params,
            placed: Err(String::new()),
            fill: Vec::new(),
            camera,
            hover: None,
            selected: Vec::new(),
            drag: None,
            frame: None,
            names: true,
            constraints: true,
            draft: Draft::default(),
            field: None,
            conflict: None,
        };
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
        self.set(self.placed_with(None));
    }

    /// Takes a solution, and the regions its profiles bound: the model's own (the regions an
    /// extrusion would sweep), triangulated as a face of a body one unit thick.
    fn set(&mut self, placed: Result<Placed, String>) {
        self.fill = placed.as_ref().ok().and_then(|p| fill(self.context.id, p)).unwrap_or_default();
        // The face the sketch lies on, to snap to and project from.
        self.draft.face = match (&self.context.input, &placed) {
            (Some(v), Ok(p)) => v.body().map(|b| face_edges(b, p)).unwrap_or_default(),
            _ => Vec::new(),
        };
        self.placed = placed;
    }

    /// Applies an `edit_sketch` patch drawn in the window at once, without waiting for the
    /// model, and returns it as the command's parameters.
    pub fn apply(&mut self, mut patch: serde_json::Map<String, Json>) -> Json {
        if let Some(fields) = self.json.as_object_mut() {
            let _ = datum_model::patch_sketch(fields, &patch);
        }
        self.solve();
        patch.insert("name".into(), json!(self.feature));
        Json::Object(patch)
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
        let placed = tries.into_iter().map(|q| self.placed_with(Some((&n, q)))).find(Result::is_ok).unwrap_or(plain);
        self.set(placed);
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
                // The model's origin is no point of the sketch's to remove.
                Pick::Point(n) if n == datum_ops::ORIGIN => {}
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
    pub(crate) fn curve_points(p: &Placed, name: &str) -> Option<Vec<[f64; 2]>> {
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
    pub(crate) fn middle(p: &Placed, name: &str) -> Option<[f64; 2]> {
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
                "angle" => Some(angle_spot(line(&c["lines"][0])?, line(&c["lines"][1])?)),
                "radius" => c["arc"].as_str().and_then(|n| Self::middle(p, n)),
                _ => None,
            }
        };
        // An angle reads by its °: the window's font has no ∠.
        let prefix = |c: &Json| if c["type"] == "radius" { "R" } else { "" };
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
    /// first, then a constraint's badge, then a label, then a curve.
    pub fn hit(&self, dx: f64, dy: f64, height: f64, label_size: impl Fn(&str) -> (f64, f64)) -> Option<Pick> {
        if let Some(n) = self.point_at(dx, dy, height) {
            return Some(self.projection_of(&n).map_or(Pick::Point(n), Pick::Projection));
        }
        if let Some(b) = self.badges(height).into_iter().find(|b| (dx - b.at.0).abs() <= BADGE * 0.5 && (dy - b.at.1).abs() <= BADGE * 0.5) {
            return Some(b.pick);
        }
        for (l, (x, y), (w, h)) in self.laid_out(height, label_size) {
            if (dx - x).abs() <= w * 0.5 && (dy - y).abs() <= h * 0.5 {
                return Some(l.pick);
            }
        }
        self.curve_at(dx, dy, height).map(|n| self.projection_of(&n).map_or(Pick::Curve(n), Pick::Projection))
    }

    /// The point under pixel offset `(dx, dy)`, by its own name (a projection's too).
    pub fn point_at(&self, dx: f64, dy: f64, height: f64) -> Option<String> {
        let p = self.placed.as_ref().ok()?;
        p.solved
            .points
            .iter()
            .map(|(n, v)| {
                let (x, y) = self.camera.offset([v.x, v.y], height);
                (libm::hypot(x - dx, y - dy), n)
            })
            .filter(|(d, _)| *d <= NEAR_POINT)
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map(|(_, n)| n.clone())
    }

    /// The line, arc or circle under pixel offset `(dx, dy)`, by its own name.
    pub fn curve_at(&self, dx: f64, dy: f64, height: f64) -> Option<String> {
        let p = self.placed.as_ref().ok()?;
        let distance = |n: &String| {
            let pts = Self::curve_points(p, n)?;
            let screen: Vec<(f64, f64)> = pts.iter().map(|q| self.camera.offset(*q, height)).collect();
            screen.windows(2).map(|w| segment_distance((dx, dy), w[0], w[1])).min_by(f64::total_cmp)
        };
        p.lines
            .keys()
            .chain(p.arcs.keys())
            .chain(p.circles.keys())
            .filter_map(|n| Some((distance(n)?, n)))
            .filter(|(d, _)| *d <= NEAR_CURVE)
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map(|(_, n)| n.clone())
    }

    /// What frame `fr` selects in a picture `height` tall, as in Inventor: dragged to the
    /// right, what lies wholly inside it; to the left, whatever it touches. Points, curves and
    /// dimensions of this sketch; projections stay out.
    pub fn framed(&self, fr: SelectFrame, height: f64, label_size: impl Fn(&str) -> (f64, f64)) -> Vec<Pick> {
        let Ok(p) = &self.placed else { return Vec::new() };
        let (f, crossing) = (fr.ends, fr.crossing());
        let (lo, hi) = ((f[0].0.min(f[1].0), f[0].1.min(f[1].1)), (f[0].0.max(f[1].0), f[0].1.max(f[1].1)));
        let inside = |q: (f64, f64)| (lo.0..=hi.0).contains(&q.0) && (lo.1..=hi.1).contains(&q.1);
        let mut out = Vec::new();
        let drawn = self.drawn();
        for (n, v) in &p.solved.points {
            if drawn.contains_key(n) && inside(self.camera.offset([v.x, v.y], height)) {
                out.push(Pick::Point(n.clone()));
            }
        }
        for n in p.lines.keys().chain(p.arcs.keys()).chain(p.circles.keys()) {
            if self.projection_of(n).is_some() {
                continue;
            }
            let Some(pts) = Self::curve_points(p, n) else { continue };
            let screen: Vec<(f64, f64)> = pts.iter().map(|q| self.camera.offset(*q, height)).collect();
            let taken = if crossing {
                screen.iter().any(|q| inside(*q)) || screen.windows(2).any(|w| segment_meets_box(w[0], w[1], lo, hi))
            } else {
                screen.iter().all(|q| inside(*q))
            };
            if taken {
                out.push(Pick::Curve(n.clone()));
            }
        }
        for (l, (x, y), (w, h)) in self.laid_out(height, label_size) {
            if !matches!(l.pick, Pick::Constraint(_) | Pick::Measure(_)) {
                continue;
            }
            let (a, b) = ((x - w * 0.5, y - h * 0.5), (x + w * 0.5, y + h * 0.5));
            let taken = if crossing { a.0 <= hi.0 && b.0 >= lo.0 && a.1 <= hi.1 && b.1 >= lo.1 } else { inside(a) && inside(b) };
            if taken {
                out.push(l.pick);
            }
        }
        out
    }

    /// The badges of the geometric constraints in a picture `height` tall: by a line, off it a
    /// third of the way along, on its left; by a point, above and to its left (its name is
    /// below and to the right); by an arc or circle, off its middle. More on one element line
    /// up beside the first.
    pub fn badges(&self, height: f64) -> Vec<Badge> {
        let Ok(p) = &self.placed else { return Vec::new() };
        if !self.constraints {
            return Vec::new();
        }
        let px = |n: &str| p.solved.points.get(n).map(|v| self.camera.offset([v.x, v.y], height));
        let step = BADGE + 2.0;
        let mut taken: BTreeMap<String, f64> = BTreeMap::new();
        let mut out = Vec::new();
        for (k, c) in self.json["sketch"]["constraints"].as_array().into_iter().flatten().enumerate() {
            let Some((glyph, on)) = glyph_of(c) else { continue };
            for n in on {
                let i = taken.entry(n.clone()).or_default();
                let shift = *i * step;
                *i += 1.0;
                let at = if let Some(l) = p.lines.get(&n) {
                    let (Some(a), Some(b)) = (px(&l.from), px(&l.to)) else { continue };
                    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
                    let len = libm::hypot(dx, dy);
                    let (ux, uy) = if len > 0.0 { (dx / len, dy / len) } else { (1.0, 0.0) };
                    (a.0 + dx / 3.0 + uy * 12.0 + ux * shift, a.1 + dy / 3.0 - ux * 12.0 + uy * shift)
                } else if let Some(q) = px(&n) {
                    (q.0 - 14.0 - shift, q.1 - 14.0)
                } else if let Some(m) = Self::middle(p, &n) {
                    let q = self.camera.offset(m, height);
                    (q.0 + 14.0 + shift, q.1 - 14.0)
                } else {
                    continue;
                };
                out.push(Badge { pick: Pick::Constraint(k), glyph, at });
            }
        }
        out
    }

    /// Follows the frame being dragged to `offset`; when the drag has `stopped`, ends it and
    /// gives what it selects.
    pub fn frame_to(&mut self, offset: Option<(f64, f64)>, stopped: bool, height: f64, label_size: impl Fn(&str) -> (f64, f64)) -> Option<Vec<Pick>> {
        let f = self.frame.as_mut()?;
        if let Some(o) = offset {
            f.ends[1] = o;
        }
        if !stopped {
            return None;
        }
        let f = self.frame.take()?;
        Some(self.framed(f, height, label_size))
    }

    /// Draws the selection frame being dragged: solid when it takes what is inside, dashed when
    /// what it touches.
    pub fn paint_frame(&self, painter: &egui::Painter, rect: egui::Rect) {
        let Some(f) = self.frame else { return };
        let at = |q: (f64, f64)| rect.center() + egui::vec2(q.0 as f32, q.1 as f32);
        let r = egui::Rect::from_two_pos(at(f.ends[0]), at(f.ends[1]));
        let (fill, stroke) = if f.crossing() {
            (egui::Color32::from_rgba_unmultiplied(40, 160, 60, 30), egui::Stroke::new(1.0, egui::Color32::from_rgb(40, 140, 60)))
        } else {
            (egui::Color32::from_rgba_unmultiplied(40, 100, 220, 30), egui::Stroke::new(1.0, egui::Color32::from_rgb(40, 100, 220)))
        };
        painter.rect_filled(r, 0.0, fill);
        if f.crossing() {
            let c = [r.left_top(), r.right_top(), r.right_bottom(), r.left_bottom(), r.left_top()];
            painter.extend(egui::Shape::dashed_line(&c, stroke, 6.0, 4.0));
        } else {
            painter.rect_stroke(r, 0.0, stroke, egui::StrokeKind::Inside);
        }
    }

    /// Draws the sketch over the view in `rect`.
    /// The insertion points of a selection being kept in the library, marked 1 and 2.
    pub fn paint_anchors(&self, painter: &egui::Painter, rect: egui::Rect, anchors: &[String]) {
        let Ok(p) = &self.placed else { return };
        let color = egui::Color32::from_rgb(230, 120, 20);
        for (k, n) in anchors.iter().enumerate() {
            let Some(q) = p.solved.points.get(n) else { continue };
            let (x, y) = self.camera.offset([q.x, q.y], f64::from(rect.height()));
            let at = rect.center() + egui::vec2(x as f32, y as f32);
            painter.circle_stroke(at, 9.0, egui::Stroke::new(2.0, color));
            painter.text(at + egui::vec2(10.0, -10.0), egui::Align2::LEFT_BOTTOM, format!("{}", k + 1), egui::FontId::proportional(14.0), color);
        }
    }

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
        let mut regions = egui::Mesh::default();
        for t in &self.fill {
            let base = regions.vertices.len() as u32;
            for &q in t {
                regions.colored_vertex(screen(q), FILL);
            }
            regions.add_triangle(base, base + 1, base + 2);
        }
        painter.add(egui::Shape::mesh(regions));
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
        // The model's origin: its axes, faint, to the edges of the view.
        if let Some(o) = p.solved.points.get(datum_ops::ORIGIN) {
            let at = screen([o.x, o.y]);
            let faint = egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(40, 40, 50, 70));
            painter.line_segment([egui::pos2(rect.left(), at.y), egui::pos2(rect.right(), at.y)], faint);
            painter.line_segment([egui::pos2(at.x, rect.top()), egui::pos2(at.x, rect.bottom())], faint);
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
        for b in self.badges(height) {
            let c = if marked(&b.pick) { SELECTED } else { NAME };
            let r = egui::Rect::from_center_size(rect.center() + egui::vec2(b.at.0 as f32, b.at.1 as f32), egui::vec2(BADGE as f32, BADGE as f32));
            painter.rect_filled(r, 2.0, egui::Color32::from_rgba_unmultiplied(255, 255, 255, 220));
            painter.rect_stroke(r, 2.0, egui::Stroke::new(1.0, c), egui::StrokeKind::Inside);
            paint_glyph(painter, r.center(), b.glyph, c);
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

/// Triangles of the regions `p`'s profiles bound, in its coordinates; None when the profiles
/// bound none (open, branching or touching loops).
fn fill(feature: TopoId, p: &Placed) -> Option<Vec<[[f64; 2]; 3]>> {
    let regions = datum_ops::profile_regions(feature, p).ok()?;
    let body = datum_brep::extrude(&regions, 0.0, 1.0).ok()?;
    let chord = if body.has_curves() { body.chord() } else { 0.0 };
    let mut out = Vec::new();
    for f in (0..body.faces().len()).filter(|&f| body.faces()[f].surface.plane().is_some_and(|q| q.normal.z > 0.0)) {
        let m = datum_brep::face_mesh(&body, f as datum_brep::Ix, false, chord).ok()?;
        out.extend(m.triangles.iter().map(|t| t.map(|k| [m.points[k as usize].x, m.points[k as usize].y])));
    }
    Some(out)
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

/// Where an angle's label stands: inside the angle, off the point where the two lines meet,
/// along the bisector toward their middles; between their middles if they are parallel.
fn angle_spot(l: ([f64; 2], [f64; 2]), m: ([f64; 2], [f64; 2])) -> [f64; 2] {
    let mid = |(a, b): ([f64; 2], [f64; 2])| [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5];
    let (ml, mm) = (mid(l), mid(m));
    let (d, e) = ([l.1[0] - l.0[0], l.1[1] - l.0[1]], [m.1[0] - m.0[0], m.1[1] - m.0[1]]);
    let det = d[0] * e[1] - d[1] * e[0];
    if det == 0.0 {
        return mid((ml, mm));
    }
    let t = ((m.0[0] - l.0[0]) * e[1] - (m.0[1] - l.0[1]) * e[0]) / det;
    let x = [l.0[0] + d[0] * t, l.0[1] + d[1] * t];
    // Unit rays from the vertex toward each line's middle; their sum is the bisector.
    let ray = |q: [f64; 2]| {
        let v = [q[0] - x[0], q[1] - x[1]];
        let n = libm::hypot(v[0], v[1]);
        if n > 0.0 { [v[0] / n, v[1] / n] } else { [0.0, 0.0] }
    };
    let (u, w) = (ray(ml), ray(mm));
    let b = [u[0] + w[0], u[1] + w[1]];
    let n = libm::hypot(b[0], b[1]);
    if n == 0.0 {
        return x;
    }
    // A quarter of the shorter line off the vertex.
    let r = 0.25 * libm::hypot(d[0], d[1]).min(libm::hypot(e[0], e[1]));
    [x[0] + b[0] / n * r, x[1] + b[1] / n * r]
}

/// Whether segment `a`–`b` meets the box `lo`–`hi` (Liang–Barsky clipping): what a frame
/// dragged to the left touches.
fn segment_meets_box(a: (f64, f64), b: (f64, f64), lo: (f64, f64), hi: (f64, f64)) -> bool {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let (mut t0, mut t1) = (0.0_f64, 1.0_f64);
    for (q, r) in [(-dx, a.0 - lo.0), (dx, hi.0 - a.0), (-dy, a.1 - lo.1), (dy, hi.1 - a.1)] {
        if q == 0.0 {
            if r < 0.0 {
                return false;
            }
        } else {
            let t = r / q;
            if q < 0.0 {
                t0 = t0.max(t);
            } else {
                t1 = t1.min(t);
            }
        }
    }
    t0 <= t1
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

    /// The plate's region is filled with its holes open; pushed out of the outline, a hole is
    /// a region of its own.
    #[test]
    fn regions_are_filled_holes_open() {
        let area = |e: &SketchEdit| {
            e.fill.iter().map(|t| 0.5 * ((t[1][0] - t[0][0]) * (t[2][1] - t[0][1]) - (t[2][0] - t[0][0]) * (t[1][1] - t[0][1])).abs()).sum::<f64>()
        };
        let mut e = edit();
        assert!((area(&e) - 48.0).abs() < 1e-9, "{}", area(&e));
        e.json["sketch"]["points"]["h"] = json!([6, 2]);
        e.json["sketch"]["circles"] = json!({"hole": {"center": "h", "diameter": 2}});
        e.solve();
        assert!(area(&e) < 48.0 - 3.0, "the hole is open: {}", area(&e));
        e.json["sketch"]["points"]["h"] = json!([30, 2]);
        e.solve();
        assert!(area(&e) > 48.0 + 3.0, "outside, a disc: {}", area(&e));
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

    /// A frame dragged to the right takes what is wholly inside; to the left, what it touches.
    #[test]
    fn frames_select_inside_or_touched() {
        let e = edit();
        let at = |q: [f64; 2]| e.camera.offset(q, 600.0);
        let size = |_: &str| (0.0, 0.0);
        // Around the bottom edge only, to the right: the bottom and its corners.
        let (a, b) = (at([-1.0, -1.0]), at([13.0, 1.0]));
        let frame = |a, b| SelectFrame { ends: [a, b], touching: false };
        let mut got = e.framed(frame(a, b), 600.0, size);
        got.retain(|p| !matches!(p, Pick::Constraint(_) | Pick::Measure(_)));
        assert_eq!(got, [Pick::Point("a".into()), Pick::Point("b".into()), Pick::Curve("bottom".into())]);
        // A thin frame across the middle, to the left: the two sides it cuts, no corner, and
        // the height's label on the left side.
        let (a, b) = (at([13.0, 1.5]), at([-1.0, 2.5]));
        let got = e.framed(frame(a, b), 600.0, size);
        assert_eq!(got, [Pick::Curve("left".into()), Pick::Curve("right".into()), Pick::Measure("height".into())]);
        // The same frame to the right: nothing lies wholly inside it.
        assert!(e.framed(frame(b, a), 600.0, size).iter().all(|p| matches!(p, Pick::Constraint(_) | Pick::Measure(_))));
        // The dimension tool's frame takes what it touches either way.
        assert_eq!(e.framed(SelectFrame { ends: [b, a], touching: true }, 600.0, size), e.framed(frame(a, b), 600.0, size));
    }

    /// An angle's label stands inside the angle, by the point where its lines meet.
    #[test]
    fn angle_labels_stand_by_the_vertex() {
        // Lines meeting at (10, 0): one toward (0, 0), one toward (10, 8).
        let at = angle_spot(([0.0, 0.0], [10.0, 0.0]), ([10.0, 0.0], [10.0, 8.0]));
        assert!((at[0] - (10.0 - 2.0 * libm::sqrt(0.5))).abs() < 1e-9 && (at[1] - 2.0 * libm::sqrt(0.5)).abs() < 1e-9, "{at:?}");
        assert_eq!(angle_spot(([0.0, 0.0], [4.0, 0.0]), ([0.0, 2.0], [4.0, 2.0])), [2.0, 1.0]);
    }

    /// Geometric constraints have badges by what they tie; a click on one picks the constraint.
    #[test]
    fn constraints_have_badges() {
        let e = edit();
        let glyphs: Vec<(Pick, Glyph)> = e.badges(600.0).into_iter().map(|b| (b.pick, b.glyph)).collect();
        assert_eq!(
            glyphs,
            [
                (Pick::Constraint(0), Glyph::Fix),
                (Pick::Constraint(1), Glyph::Horizontal),
                (Pick::Constraint(2), Glyph::Vertical),
                (Pick::Constraint(3), Glyph::Horizontal),
                (Pick::Constraint(4), Glyph::Vertical),
            ]
        );
        let b = &e.badges(600.0)[2];
        assert_eq!(e.hit(b.at.0 + 3.0, b.at.1, 600.0, |_| (0.0, 0.0)), Some(Pick::Constraint(2)));
    }
}
