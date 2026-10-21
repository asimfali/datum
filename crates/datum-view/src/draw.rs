//! Drawing in a sketch (M7d2, D91): lines (with tangent arcs), arcs, circles and points; snaps
//! to what is already there; constraints inferred while drawing, as in Inventor. Every click is
//! one `edit_sketch` patch, applied to the window's copy at once and sent to the model: one step
//! of undo each. Snap and inference tolerances are pixels and degrees on the screen: they only
//! choose which constraints to add, and the solver then holds those exactly.

use datum_brep::{Body, Curve, Ix};
use datum_ops::Placed;
use serde_json::{Map, Value as Json, json};

use crate::sketch::SketchEdit;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tool {
    Select,
    Line,
    Arc,
    Circle,
    Point,
}

/// What a click lands on.
#[derive(Clone, Debug, PartialEq)]
pub enum Snap {
    /// A point of the sketch, its own or a projection's.
    Point(String),
    /// The middle of a line.
    Middle(String),
    /// A curve of the sketch.
    On(String),
    /// A vertex of the face the sketch lies on, by ID: projected when used.
    Vertex(String),
    /// An edge of that face, by ID: projected when used.
    Edge(String),
    Free,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Spot {
    pub at: [f64; 2],
    pub snap: Snap,
}

/// A constraint inferred while drawing: a line horizontal, vertical, parallel or perpendicular
/// to another; an arc's end level with its centre (horizontal) or plumb above it (vertical).
#[derive(Clone, Debug, PartialEq)]
pub enum Lean {
    Horizontal,
    Vertical,
    Parallel(String),
    Perpendicular(String),
}

/// The end of the line chain being drawn: its point, and the curve that ends there with the
/// direction it was travelling (a tangent arc continues it).
#[derive(Clone, Debug, PartialEq)]
pub struct Chain {
    pub point: String,
    pub at: [f64; 2],
    pub last: Option<(String, [f64; 2])>,
}

/// An edge of the face the sketch lies on, in sketch coordinates: its ID, its ends (vertex ID
/// and point), and its circle if it is an arc.
#[derive(Clone, Debug)]
pub struct FaceEdge {
    pub id: String,
    pub ends: [(String, [f64; 2]); 2],
    pub circle: Option<([f64; 2], f64)>,
}

#[derive(Clone, Debug)]
pub struct Draft {
    pub tool: Tool,
    pub construction: bool,
    /// Line tool: the chain's end.
    pub chain: Option<Chain>,
    /// Arc tool: centre, then start; circle tool: centre.
    pub clicks: Vec<Spot>,
    /// Arc tool: the angle swept from the start, following the mouse (counterclockwise > 0).
    pub sweep: f64,
    /// Arc tool: the start's lean to the centre.
    pub start_lean: Option<Lean>,
    /// The next segment of the chain is an arc tangent to the last (T, or a drag from its end).
    pub tangent: bool,
    /// Where the cursor would click, and the inferred constraint of a line.
    pub cursor: Option<Spot>,
    pub lean: Option<Lean>,
    /// The face the sketch lies on, edge by edge.
    pub face: Vec<FaceEdge>,
}

impl Default for Draft {
    fn default() -> Self {
        Self {
            tool: Tool::Select,
            construction: false,
            chain: None,
            clicks: Vec::new(),
            sweep: 0.0,
            start_lean: None,
            tangent: false,
            cursor: None,
            lean: None,
            face: Vec::new(),
        }
    }
}

/// Pixels within which a click lands on a point (or a middle), and on a curve.
const SNAP_POINT: f64 = 10.0;
const SNAP_CURVE: f64 = 8.0;
/// Degrees within which a line is taken as horizontal, vertical, parallel or perpendicular.
const LEAN_DEG: f64 = 3.0;

const DRAFT: egui::Color32 = egui::Color32::from_rgb(235, 110, 20);

/// The edges of face `placed.face` of `body`, in the sketch's coordinates. Its arcs lie in its
/// plane, so they stay circles.
pub fn face_edges(body: &Body, placed: &Placed) -> Vec<FaceEdge> {
    let Some(id) = placed.face else { return Vec::new() };
    let Ok(f) = body.faces().binary_search_by_key(&id, |f| f.id) else { return Vec::new() };
    let [o, x, y, _] = placed.frame;
    let flat = |p: datum_math::Vec3| [(p - o).dot(x), (p - o).dot(y)];
    let mut out = Vec::new();
    for h in body.face_loops(f as Ix).flat_map(|l| body.loop_half_edges(l)) {
        let edge = &body.edges()[body.half_edges()[h as usize].edge as usize];
        let (a, b) = (body.half_edges()[edge.half as usize].origin, body.half_edges()[body.next(edge.half) as usize].origin);
        let end = |v: Ix| (format!("{:?}", body.vertices()[v as usize].id), flat(body.point(v)));
        let circle = match body.edge_curve(edge) {
            None => None,
            Some(Curve::Arc(arc)) => Some((flat(arc.center), arc.radius)),
            Some(Curve::Rolled { .. }) => continue,
        };
        out.push(FaceEdge { id: format!("{:?}", edge.id), ends: [end(a), end(b)], circle });
    }
    out
}

impl Draft {
    /// Cancels what is half drawn; true if there was something.
    pub fn cancel(&mut self) -> bool {
        let busy = self.chain.is_some() || !self.clicks.is_empty();
        self.chain = None;
        self.clicks.clear();
        self.tangent = false;
        busy
    }

    /// Esc: ends the command, as in Inventor: what is half drawn goes, the tool is Select.
    pub fn escape(&mut self) {
        self.cancel();
        self.tool = Tool::Select;
        self.cursor = None;
        self.lean = None;
    }

    /// Where a click at pixel offset `(dx, dy)` would land, `free` (Shift) without snaps.
    pub fn spot(&self, edit: &SketchEdit, dx: f64, dy: f64, height: f64, free: bool) -> Spot {
        let at = edit.camera.at(dx, dy, height);
        let free_spot = Spot { at, snap: Snap::Free };
        let Ok(p) = &edit.placed else { return free_spot };
        if free {
            return free_spot;
        }
        let px = |q: [f64; 2]| {
            let (x, y) = edit.camera.offset(q, height);
            libm::hypot(x - dx, y - dy)
        };
        // The chain's own end is no place to end the next segment.
        let skip = self.chain.as_ref().map(|c| c.point.as_str());
        let nearest =
            |cands: Vec<(f64, Spot)>, within: f64| cands.into_iter().filter(|(d, _)| *d <= within).min_by(|a, b| a.0.total_cmp(&b.0)).map(|(_, s)| s);
        let points = p
            .solved
            .points
            .iter()
            .filter(|(n, _)| Some(n.as_str()) != skip)
            .map(|(n, v)| (px([v.x, v.y]), Spot { at: [v.x, v.y], snap: Snap::Point(n.clone()) }));
        let middles = p.lines.keys().filter_map(|n| SketchEdit::middle(p, n).map(|m| (px(m), Spot { at: m, snap: Snap::Middle(n.clone()) })));
        let projected: Vec<&str> =
            edit.json["project"].as_object().map(|m| m.values().filter_map(|v| v["id"].as_str()).collect()).unwrap_or_default();
        let vertices = self
            .face
            .iter()
            .flat_map(|e| e.ends.iter())
            .filter(|(id, _)| !projected.contains(&id.as_str()))
            .map(|(id, q)| (px(*q), Spot { at: *q, snap: Snap::Vertex(id.clone()) }));
        if let Some(s) = nearest(points.chain(middles).chain(vertices).collect(), SNAP_POINT) {
            return s;
        }
        // On a curve of the sketch, or an edge of its face: the nearest point of it.
        let mut on = Vec::new();
        for n in p.lines.keys().chain(p.arcs.keys()).chain(p.circles.keys()) {
            if let Some(q) = SketchEdit::curve_points(p, n).and_then(|pts| foot(&pts, at)) {
                on.push((px(q), Spot { at: q, snap: Snap::On(n.clone()) }));
            }
        }
        for e in self.face.iter().filter(|e| !projected.contains(&e.id.as_str())) {
            let q = match e.circle {
                None => foot(&[e.ends[0].1, e.ends[1].1], at),
                Some((c, r)) => {
                    let d = libm::hypot(at[0] - c[0], at[1] - c[1]);
                    (d > 0.0).then(|| [c[0] + (at[0] - c[0]) * r / d, c[1] + (at[1] - c[1]) * r / d])
                }
            };
            if let Some(q) = q {
                on.push((px(q), Spot { at: q, snap: Snap::Edge(e.id.clone()) }));
            }
        }
        nearest(on, SNAP_CURVE).unwrap_or(free_spot)
    }

    /// The constraint a line from `from` to the free spot `to` leans to, and where that puts
    /// its end: horizontal or vertical first, then parallel or perpendicular to the nearest line
    /// by angle. None beyond a few degrees.
    pub fn lean(&self, edit: &SketchEdit, from: [f64; 2], to: [f64; 2]) -> Option<(Lean, [f64; 2])> {
        let d = [to[0] - from[0], to[1] - from[1]];
        let len = libm::hypot(d[0], d[1]);
        if len == 0.0 {
            return None;
        }
        let angle = libm::atan2(d[1], d[0]);
        // The angle between two directions, as lines (0..90°).
        let off = |a: f64, b: f64| {
            let x = (a - b).rem_euclid(std::f64::consts::PI);
            x.min(std::f64::consts::PI - x).to_degrees()
        };
        let along = |u: [f64; 2]| {
            let t = d[0] * u[0] + d[1] * u[1];
            [from[0] + u[0] * t, from[1] + u[1] * t]
        };
        if off(angle, 0.0) <= LEAN_DEG {
            return Some((Lean::Horizontal, [to[0], from[1]]));
        }
        if off(angle, std::f64::consts::FRAC_PI_2) <= LEAN_DEG {
            return Some((Lean::Vertical, [from[0], to[1]]));
        }
        let p = edit.placed.as_ref().ok()?;
        let mut best: Option<(f64, Lean, [f64; 2])> = None;
        for (n, pts) in p.lines.keys().filter_map(|n| Some((n, SketchEdit::curve_points(p, n)?))) {
            let (a, b) = (pts[0], pts[1]);
            let l = libm::hypot(b[0] - a[0], b[1] - a[1]);
            if l == 0.0 {
                continue;
            }
            let u = [(b[0] - a[0]) / l, (b[1] - a[1]) / l];
            let line = libm::atan2(u[1], u[0]);
            for (o, lean, dir) in [
                (off(angle, line), Lean::Parallel(n.clone()), u),
                (off(angle, line + std::f64::consts::FRAC_PI_2), Lean::Perpendicular(n.clone()), [-u[1], u[0]]),
            ] {
                if o <= LEAN_DEG && best.as_ref().is_none_or(|b| o < b.0) {
                    best = Some((o, lean, along(dir)));
                }
            }
        }
        best.map(|(_, lean, at)| (lean, at))
    }

    /// Follows the cursor: where it would click, what it leans to, and the arc's sweep.
    pub fn track(&mut self, edit: &SketchEdit, dx: f64, dy: f64, height: f64, free: bool) {
        let mut spot = self.spot(edit, dx, dy, height, free);
        self.lean = None;
        let leaning = !free && spot.snap == Snap::Free;
        let found = match (self.tool, &self.chain, &self.clicks[..]) {
            _ if !leaning => None,
            (Tool::Line, Some(c), _) if !self.tangent => self.lean(edit, c.at, spot.at),
            (Tool::Line, Some(Chain { at, last: Some((_, dir)), .. }), _) => tangent_lean(*at, *dir, spot.at),
            (Tool::Arc, _, [c] | [c, _]) => axis_lean(c.at, spot.at),
            _ => None,
        };
        if let Some((lean, at)) = found {
            spot.at = at;
            self.lean = Some(lean);
        }
        if let (Tool::Arc, [c, s]) = (self.tool, &self.clicks[..]) {
            let angle = |q: [f64; 2]| libm::atan2(q[1] - c.at[1], q[0] - c.at[0]);
            let before = self.cursor.as_ref().map_or(angle(s.at), |k| angle(k.at));
            let step = (angle(spot.at) - before + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI;
            self.sweep += step;
        }
        self.cursor = Some(spot);
    }

    /// A click where the cursor is: the `edit_sketch` patch it makes, if it completes something.
    pub fn click(&mut self, edit: &SketchEdit) -> Option<Map<String, Json>> {
        let spot = self.cursor.clone()?;
        let mut patch = Patch::new(edit);
        match self.tool {
            Tool::Select => return None,
            Tool::Point => {
                // A point already there needs no other; a vertex of the face is projected.
                if matches!(spot.snap, Snap::Point(_)) {
                    return None;
                }
                patch.point(&spot);
            }
            Tool::Circle => {
                let Some(centre) = self.clicks.first().cloned() else {
                    self.clicks.push(spot);
                    return None;
                };
                let r = libm::hypot(spot.at[0] - centre.at[0], spot.at[1] - centre.at[1]);
                if r == 0.0 {
                    return None;
                }
                let c = patch.point(&centre);
                let name = patch.fresh("c");
                patch.add("circles", &name, json!({"center": c, "diameter": short(2.0 * r)}), self.construction);
                self.clicks.clear();
            }
            Tool::Arc => {
                if self.clicks.len() < 2 {
                    if self.clicks.len() == 1 {
                        self.sweep = 0.0;
                        self.start_lean = self.lean.clone();
                    }
                    self.clicks.push(spot);
                    return None;
                }
                let (c, s) = (self.clicks[0].clone(), self.clicks[1].clone());
                let r = libm::hypot(s.at[0] - c.at[0], s.at[1] - c.at[1]);
                let d = libm::hypot(spot.at[0] - c.at[0], spot.at[1] - c.at[1]);
                if r == 0.0 || d == 0.0 || self.sweep == 0.0 {
                    return None;
                }
                // The end at the start's distance from the centre, toward the cursor.
                let end = match spot.snap {
                    Snap::Point(_) | Snap::Vertex(_) => spot,
                    _ => Spot { at: [c.at[0] + (spot.at[0] - c.at[0]) * r / d, c.at[1] + (spot.at[1] - c.at[1]) * r / d], snap: Snap::Free },
                };
                let (cn, sn, en) = (patch.point(&c), patch.point(&s), patch.point(&end));
                let name = patch.fresh("a");
                let (from, to) = if self.sweep > 0.0 { (sn.clone(), en.clone()) } else { (en.clone(), sn.clone()) };
                patch.add("arcs", &name, json!({"center": cn, "from": from, "to": to}), self.construction);
                for (lean, p) in [(self.start_lean.take(), sn), (self.lean.clone(), en)] {
                    patch.level(lean.as_ref(), &cn, &p);
                }
                self.clicks.clear();
            }
            Tool::Line => {
                let Some(chain) = self.chain.clone() else {
                    let point = patch.point(&spot);
                    self.chain = Some(Chain { point, at: spot.at, last: None });
                    return patch.done();
                };
                let end = patch.point(&spot);
                let from = chain.point.clone();
                let arc = match (&chain.last, self.tangent) {
                    (Some((curve, dir)), true) => tangent_arc(chain.at, *dir, spot.at).map(|(c, ccw)| (curve.clone(), c, ccw)),
                    _ => None,
                };
                let next_dir = if let Some((curve, c, ccw)) = arc {
                    let centre = patch.point(&Spot { at: c, snap: Snap::Free });
                    let name = patch.fresh("a");
                    let (f, t) = if ccw { (from, end.clone()) } else { (end.clone(), from) };
                    patch.add("arcs", &name, json!({"center": centre.clone(), "from": f, "to": t}), self.construction);
                    patch.constrain(json!({"type": "tangent", "curves": [curve, name.clone()]}));
                    patch.level(self.lean.as_ref(), &centre, &end);
                    // Travelling on: along the circle at the end, the way the arc turned.
                    let w = [spot.at[0] - c[0], spot.at[1] - c[1]];
                    let l = libm::hypot(w[0], w[1]);
                    let dir = if ccw { [-w[1] / l, w[0] / l] } else { [w[1] / l, -w[0] / l] };
                    (name, dir)
                } else {
                    let l = libm::hypot(spot.at[0] - chain.at[0], spot.at[1] - chain.at[1]);
                    if l == 0.0 {
                        return None;
                    }
                    let name = patch.fresh("l");
                    patch.add("lines", &name, json!({"from": from, "to": end}), self.construction);
                    match &self.lean {
                        Some(Lean::Horizontal) => patch.constrain(json!({"type": "horizontal", "line": name})),
                        Some(Lean::Vertical) => patch.constrain(json!({"type": "vertical", "line": name})),
                        Some(Lean::Parallel(o)) => patch.constrain(json!({"type": "parallel", "lines": [o, name]})),
                        Some(Lean::Perpendicular(o)) => patch.constrain(json!({"type": "perpendicular", "lines": [o, name]})),
                        None => {}
                    }
                    let dir = [(spot.at[0] - chain.at[0]) / l, (spot.at[1] - chain.at[1]) / l];
                    (name, dir)
                };
                self.tangent = false;
                // Closing on a point of the sketch ends the chain, as Inventor does.
                self.chain = match spot.snap {
                    Snap::Point(_) | Snap::Vertex(_) => None,
                    _ => Some(Chain { point: end, at: spot.at, last: Some(next_dir) }),
                };
            }
        }
        patch.done()
    }

    /// Draws what is half drawn and where the cursor would click.
    pub fn paint(&self, edit: &SketchEdit, painter: &egui::Painter, rect: egui::Rect) {
        let Some(spot) = self.cursor.as_ref().filter(|_| self.tool != Tool::Select) else { return };
        let height = f64::from(rect.height());
        let screen = |q: [f64; 2]| {
            let (x, y) = edit.camera.offset(q, height);
            rect.center() + egui::vec2(x as f32, y as f32)
        };
        let stroke = egui::Stroke::new(1.6, DRAFT);
        let polyline =
            |pts: Vec<[f64; 2]>| painter.extend(egui::Shape::dashed_line(&pts.into_iter().map(screen).collect::<Vec<_>>(), stroke, 6.0, 3.0));
        let arc = |c: [f64; 2], r: f64, start: f64, sweep: f64| {
            let n = ((sweep.abs() / std::f64::consts::TAU) * 64.0).ceil().max(2.0) as usize;
            (0..=n)
                .map(|k| {
                    let (s, co) = libm::sincos(start + sweep * k as f64 / n as f64);
                    [c[0] + r * co, c[1] + r * s]
                })
                .collect::<Vec<_>>()
        };
        match (self.tool, &self.chain, &self.clicks[..]) {
            (Tool::Line, Some(c), _) => match (&c.last, self.tangent) {
                (Some((_, dir)), true) => {
                    if let Some((centre, ccw)) = tangent_arc(c.at, *dir, spot.at) {
                        let r = libm::hypot(c.at[0] - centre[0], c.at[1] - centre[1]);
                        let a0 = libm::atan2(c.at[1] - centre[1], c.at[0] - centre[0]);
                        let a1 = libm::atan2(spot.at[1] - centre[1], spot.at[0] - centre[0]);
                        let sweep = if ccw { (a1 - a0).rem_euclid(std::f64::consts::TAU) } else { -(a0 - a1).rem_euclid(std::f64::consts::TAU) };
                        polyline(arc(centre, r, a0, sweep));
                    } else {
                        polyline(vec![c.at, spot.at]);
                    }
                }
                _ => polyline(vec![c.at, spot.at]),
            },
            (Tool::Circle, _, [c]) => {
                let r = libm::hypot(spot.at[0] - c.at[0], spot.at[1] - c.at[1]);
                polyline(arc(c.at, r, 0.0, std::f64::consts::TAU));
            }
            (Tool::Arc, _, [c]) => polyline(vec![c.at, spot.at]),
            (Tool::Arc, _, [c, s]) => {
                let r = libm::hypot(s.at[0] - c.at[0], s.at[1] - c.at[1]);
                polyline(arc(c.at, r, libm::atan2(s.at[1] - c.at[1], s.at[0] - c.at[0]), self.sweep));
            }
            _ => {}
        }
        // Where the click lands, and what it ties to.
        let at = screen(spot.at);
        painter.circle_stroke(at, 5.0, egui::Stroke::new(1.5, DRAFT));
        let mark = match (&spot.snap, &self.lean) {
            (Snap::Point(_), _) => "point",
            (Snap::Middle(_), _) => "middle",
            (Snap::On(_), _) => "on",
            (Snap::Vertex(_), _) => "vertex",
            (Snap::Edge(_), _) => "edge",
            (Snap::Free, Some(Lean::Horizontal)) => "H",
            (Snap::Free, Some(Lean::Vertical)) => "V",
            (Snap::Free, Some(Lean::Parallel(_))) => "parallel",
            (Snap::Free, Some(Lean::Perpendicular(_))) => "perpendicular",
            (Snap::Free, None) => "",
        };
        if !mark.is_empty() {
            painter.text(at + egui::vec2(9.0, -9.0), egui::Align2::LEFT_BOTTOM, mark, egui::FontId::proportional(12.0), DRAFT);
        }
    }
}

/// The centre of the arc leaving `from` along unit `dir` and passing through `to`, and whether it
/// turns counterclockwise; None if `to` lies straight ahead (or behind): no arc, a line.
pub fn tangent_arc(from: [f64; 2], dir: [f64; 2], to: [f64; 2]) -> Option<([f64; 2], bool)> {
    let v = [to[0] - from[0], to[1] - from[1]];
    let n = [-dir[1], dir[0]];
    let across = v[0] * n[0] + v[1] * n[1];
    if across == 0.0 {
        return None;
    }
    // The centre on the normal at `from`, as far from `to` as from `from`.
    let r = (v[0] * v[0] + v[1] * v[1]) / (2.0 * across);
    Some(([from[0] + n[0] * r, from[1] + n[1] * r], r > 0.0))
}

/// The four directions along the sketch's axes, each with its lean: from a centre, a point
/// along X is level with it (horizontal), along Y plumb above or below (vertical).
const AXES: [([f64; 2], Lean); 4] =
    [([1.0, 0.0], Lean::Horizontal), ([-1.0, 0.0], Lean::Horizontal), ([0.0, 1.0], Lean::Vertical), ([0.0, -1.0], Lean::Vertical)];

/// Degrees between two directions.
fn degrees(a: [f64; 2], b: [f64; 2]) -> f64 {
    libm::atan2(a[0] * b[1] - a[1] * b[0], a[0] * b[0] + a[1] * b[1]).abs().to_degrees()
}

/// A point `q` near the horizontal or vertical through centre `c`: projected onto it, as a
/// line's end is.
fn axis_lean(c: [f64; 2], q: [f64; 2]) -> Option<(Lean, [f64; 2])> {
    let w = [q[0] - c[0], q[1] - c[1]];
    let (u, lean) = AXES.into_iter().find(|(u, _)| w != [0.0, 0.0] && degrees(w, *u) <= LEAN_DEG)?;
    let t = w[0] * u[0] + w[1] * u[1];
    Some((lean, [c[0] + u[0] * t, c[1] + u[1] * t]))
}

/// The end of a tangent arc from `from` along `dir` near level with or plumb above its centre:
/// moved to where it is exactly, so the arc turns a quarter, a half or three quarters. The centre
/// is `from + n·r`; an end along axis `u` from it is `from + r·(n + u)` for `r > 0`, `from +
/// r·(n − u)` for `r < 0`: the cursor is projected onto that line.
fn tangent_lean(from: [f64; 2], dir: [f64; 2], to: [f64; 2]) -> Option<(Lean, [f64; 2])> {
    let (c, ccw) = tangent_arc(from, dir, to)?;
    let n = [-dir[1], dir[0]];
    let w = [to[0] - c[0], to[1] - c[1]];
    let (u, lean) = AXES.into_iter().find(|(u, _)| degrees(w, *u) <= LEAN_DEG)?;
    let s = if ccw { 1.0 } else { -1.0 };
    let line = [n[0] + s * u[0], n[1] + s * u[1]];
    let l2 = line[0] * line[0] + line[1] * line[1];
    if l2 == 0.0 {
        return None;
    }
    let r = ((to[0] - from[0]) * line[0] + (to[1] - from[1]) * line[1]) / l2;
    (r * s > 0.0).then(|| (lean, [from[0] + line[0] * r, from[1] + line[1] * r]))
}

/// The point of polyline `pts` nearest `q`.
fn foot(pts: &[[f64; 2]], q: [f64; 2]) -> Option<[f64; 2]> {
    pts.windows(2)
        .map(|w| {
            let (a, b) = (w[0], w[1]);
            let d = [b[0] - a[0], b[1] - a[1]];
            let l2 = d[0] * d[0] + d[1] * d[1];
            let t = if l2 > 0.0 { (((q[0] - a[0]) * d[0] + (q[1] - a[1]) * d[1]) / l2).clamp(0.0, 1.0) } else { 0.0 };
            [a[0] + d[0] * t, a[1] + d[1] * t]
        })
        .min_by(|a, b| libm::hypot(a[0] - q[0], a[1] - q[1]).total_cmp(&libm::hypot(b[0] - q[0], b[1] - q[1])))
}

/// A diameter as typed: six decimals at most.
fn short(x: f64) -> f64 {
    (x * 1e6).round() / 1e6
}

/// An `edit_sketch` patch being made: new names stay clear of the sketch's and of each other.
struct Patch {
    taken: Vec<String>,
    params: Map<String, Json>,
}

impl Patch {
    fn new(edit: &SketchEdit) -> Self {
        let sketch = &edit.json["sketch"];
        let mut taken: Vec<String> =
            ["points", "lines", "arcs", "circles", "measures"].iter().filter_map(|k| sketch[k].as_object()).flat_map(|m| m.keys().cloned()).collect();
        taken.extend(edit.json["project"].as_object().into_iter().flat_map(|m| m.keys().cloned()));
        Self { taken, params: Map::new() }
    }

    /// The first `prefix1`, `prefix2`… not taken.
    fn fresh(&mut self, prefix: &str) -> String {
        let n = (1..).map(|k| format!("{prefix}{k}")).find(|n| !self.taken.contains(n)).unwrap_or_default();
        self.taken.push(n.clone());
        n
    }

    fn add(&mut self, key: &str, name: &str, mut value: Json, construction: bool) {
        if construction {
            value["construction"] = json!(true);
        }
        self.params.entry(key).or_insert_with(|| json!({}))[name] = value;
    }

    /// An arc's point `p` level with (horizontal) or plumb above (vertical) its centre `c`.
    fn level(&mut self, lean: Option<&Lean>, c: &str, p: &str) {
        match lean {
            Some(Lean::Horizontal) => self.constrain(json!({"type": "horizontal", "points": [c, p]})),
            Some(Lean::Vertical) => self.constrain(json!({"type": "vertical", "points": [c, p]})),
            _ => {}
        }
    }

    fn constrain(&mut self, c: Json) {
        if let Json::Array(a) = self.params.entry("add_constraints").or_insert_with(|| json!([])) {
            a.push(c);
        }
    }

    /// The point a spot names, made as it needs: an existing point as it is; a vertex of the
    /// face projected; otherwise a new point there, on what it snapped to.
    fn point(&mut self, spot: &Spot) -> String {
        match &spot.snap {
            Snap::Point(n) => return n.clone(),
            Snap::Vertex(id) => {
                let v = self.fresh("v");
                self.params.entry("project").or_insert_with(|| json!({}))[&v] = json!({"id": id});
                return v;
            }
            _ => {}
        }
        let p = self.fresh("p");
        self.params.entry("points").or_insert_with(|| json!({}))[&p] = json!(spot.at);
        match &spot.snap {
            Snap::Middle(l) => self.constrain(json!({"type": "midpoint", "point": p, "line": l})),
            Snap::On(c) => self.constrain(json!({"type": "on", "point": p, "curve": c})),
            Snap::Edge(id) => {
                let e = self.fresh("e");
                self.params.entry("project").or_insert_with(|| json!({}))[&e] = json!({"id": id});
                self.constrain(json!({"type": "on", "point": p, "curve": e}));
            }
            _ => {}
        }
        p
    }

    fn done(self) -> Option<Map<String, Json>> {
        (!self.params.is_empty()).then_some(self.params)
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use std::collections::BTreeMap;

    use datum_brep::TopoId;

    use super::*;
    use crate::sketch::Context;

    const H: f64 = 600.0;

    /// A sketch editor on `sketch`, on the top face of a 40 × 30 × 8 plate if `on_plate`.
    fn editor(sketch: Json, on_plate: bool) -> SketchEdit {
        let id = TopoId::derive("feature").tag("s").finish();
        let (input, mut json) = (None, json!({"name": "s", "op": "sketch", "inputs": [], "sketch": sketch}));
        let input = if on_plate {
            let plate =
                datum_brep::cuboid(TopoId::derive("feature").tag("plate").finish(), datum_math::Vec3::ZERO, datum_math::Vec3::new(40.0, 30.0, 8.0))
                    .unwrap();
            let top = plate.faces().iter().find(|f| f.surface.plane().is_some_and(|q| q.normal.z > 0.0)).unwrap().id;
            json["face"] = json!(format!("{top:?}"));
            Some(std::sync::Arc::new(datum_graph::Value::Body(plate)))
        } else {
            input
        };
        let mut e = SketchEdit::new("s".into(), Context { id, input }, json, BTreeMap::new(), None);
        e.camera.center = [10.0, 10.0];
        e.camera.half_height = 30.0;
        e
    }

    /// Moves the cursor to sketch point `q` and clicks: the patch, applied.
    fn click_at(e: &mut SketchEdit, q: [f64; 2], free: bool) -> Option<Json> {
        let (x, y) = e.camera.offset(q, H);
        let mut d = std::mem::take(&mut e.draft);
        d.track(e, x, y, H, free);
        let patch = d.click(e);
        e.draft = d;
        patch.map(|p| e.apply(p))
    }

    fn rectangle() -> Json {
        json!({"points": {"a": [0, 0], "b": [20, 0], "c": [20, 10], "d": [0, 10]},
               "lines": {"bottom": {"from": "a", "to": "b"}, "right": {"from": "b", "to": "c"}, "top": {"from": "c", "to": "d"}, "left": {"from": "d", "to": "a"}}})
    }

    /// Lines leaning horizontal and vertical take those constraints; a tangent arc continues.
    #[test]
    fn a_chain_of_lines_and_a_tangent_arc() {
        let mut e = editor(json!({"points": {}}), false);
        e.draft.tool = Tool::Line;
        let p = click_at(&mut e, [0.0, 0.0], false).unwrap();
        assert_eq!(p["points"], json!({"p1": [0.0, 0.0]}));
        let p = click_at(&mut e, [10.0, 0.3], false).unwrap();
        assert_eq!(p["points"]["p2"], json!([10.0, 0.0]), "levelled onto the horizontal: {p}");
        assert_eq!(p["add_constraints"], json!([{"type": "horizontal", "line": "l1"}]));
        let p = click_at(&mut e, [10.2, 8.0], false).unwrap();
        assert_eq!(p["add_constraints"], json!([{"type": "vertical", "line": "l2"}]));
        e.draft.tangent = true;
        let p = click_at(&mut e, [5.0, 13.0], false).unwrap();
        assert_eq!(p["arcs"]["a1"], json!({"center": "p5", "from": "p3", "to": "p4"}), "{p}");
        assert_eq!(p["points"]["p5"], json!([5.0, 8.0]), "the centre across the line's end: {p}");
        // Its end straight above the centre: a quarter turn, held so.
        assert_eq!(p["add_constraints"], json!([{"type": "tangent", "curves": ["l2", "a1"]}, {"type": "vertical", "points": ["p5", "p4"]}]));
        assert!(!e.draft.tangent, "the next segment is a line again");
        assert!(e.placed.is_ok(), "{:?}", e.placed.as_ref().err());
        // Closing on the first point ends the chain.
        let p = click_at(&mut e, [0.1, 0.1], false).unwrap();
        assert_eq!(p["lines"]["l3"], json!({"from": "p4", "to": "p1"}), "{p}");
        assert!(e.draft.chain.is_none());
    }

    /// What is half drawn shows: `DATUM_DRAFT_SHOT=file.png` saves a picture.
    #[test]
    fn draws_the_draft() {
        let mut e = editor(rectangle(), false);
        e.draft.tool = Tool::Line;
        click_at(&mut e, [20.0, 10.0], false);
        click_at(&mut e, [30.0, 10.0], false);
        e.draft.tangent = true;
        let (x, y) = e.camera.offset([36.0, 17.0], H);
        let mut d = std::mem::take(&mut e.draft);
        d.track(&e, x, y, H, false);
        e.draft = d;
        let shot = std::env::var_os("DATUM_DRAFT_SHOT");
        let builder = egui_kittest::Harness::builder().with_size([900.0, 600.0]);
        let builder = if shot.is_some() { builder.wgpu() } else { builder };
        let mut h = builder.build_ui(|ui| {
            let (rect, _) = ui.allocate_exact_size(ui.available_size(), egui::Sense::hover());
            ui.painter().rect_filled(rect, 0.0, egui::Color32::from_rgb(235, 238, 242));
            let painter = ui.painter().with_clip_rect(rect);
            e.paint(&painter, rect);
            e.draft.paint(&e, &painter, rect);
        });
        h.run();
        if let Some(path) = shot {
            h.render().unwrap().save(path).unwrap();
        }
    }

    /// Esc ends the command at once: the chain half drawn goes, the tool, and its cursor.
    #[test]
    fn escape_ends_the_command() {
        let mut e = editor(json!({"points": {}}), false);
        e.draft.tool = Tool::Line;
        click_at(&mut e, [0.0, 0.0], false);
        assert!(e.draft.chain.is_some());
        e.draft.escape();
        assert!(e.draft.chain.is_none() && e.draft.tool == Tool::Select && e.draft.cursor.is_none());
    }

    /// A tangent arc ending level with its centre turns a half: the end is put there exactly and
    /// held there.
    #[test]
    fn a_tangent_arc_snaps_to_a_half_turn() {
        let mut e = editor(json!({"points": {}}), false);
        e.draft.tool = Tool::Line;
        click_at(&mut e, [0.0, 0.0], false);
        click_at(&mut e, [0.0, 10.0], false);
        e.draft.tangent = true;
        // Up the line, then a half turn right: the end near (8, 10.15), level with the centre (4, 10).
        let p = click_at(&mut e, [8.0, 10.15], false).unwrap();
        assert_eq!(p["points"]["p3"], json!([8.0, 10.0]), "{p}");
        assert_eq!(p["points"]["p4"], json!([4.0, 10.0]), "{p}");
        assert_eq!(p["add_constraints"][1], json!({"type": "horizontal", "points": ["p4", "p3"]}), "{p}");
        assert!(e.placed.is_ok(), "{:?}", e.placed.as_ref().err());
    }

    /// The arc tool's start and end level with or plumb above the centre take that constraint.
    #[test]
    fn arc_ends_snap_to_the_axes() {
        let mut e = editor(json!({"points": {}}), false);
        e.draft.tool = Tool::Arc;
        click_at(&mut e, [0.0, 0.0], false);
        click_at(&mut e, [5.0, 0.2], false);
        for q in [[4.0, 3.0], [0.2, 5.0]] {
            let (x, y) = e.camera.offset(q, H);
            let mut d = std::mem::take(&mut e.draft);
            d.track(&e, x, y, H, false);
            e.draft = d;
        }
        let p = click_at(&mut e, [-0.2, 5.0], false).unwrap();
        assert_eq!(p["points"]["p2"], json!([5.0, 0.0]), "{p}");
        assert_eq!(p["points"]["p3"], json!([0.0, 5.0]), "{p}");
        assert_eq!(
            p["add_constraints"],
            json!([{"type": "horizontal", "points": ["p1", "p2"]}, {"type": "vertical", "points": ["p1", "p3"]}]),
            "{p}"
        );
    }

    #[test]
    fn snaps_to_points_middles_and_curves() {
        let mut e = editor(rectangle(), false);
        e.draft.tool = Tool::Point;
        assert!(click_at(&mut e, [20.1, 0.1], false).is_none(), "a corner is a point already");
        let p = click_at(&mut e, [10.1, 0.2], false).unwrap();
        assert_eq!(p["add_constraints"], json!([{"type": "midpoint", "point": "p1", "line": "bottom"}]));
        let p = click_at(&mut e, [15.0, 10.2], false).unwrap();
        assert_eq!(p["add_constraints"], json!([{"type": "on", "point": "p2", "curve": "top"}]));
        assert_eq!(p["points"]["p2"], json!([15.0, 10.0]));
        let p = click_at(&mut e, [15.0, 10.2], true).unwrap();
        assert!(p.get("add_constraints").is_none(), "Shift: no snap: {p}");
    }

    /// The arc turns the way the mouse went, past half a turn too.
    #[test]
    fn arcs_follow_the_mouse() {
        for (path, from_start) in [(vec![[4.0, 3.0], [0.0, 5.0], [-5.0, 0.0], [0.0, -5.0]], true), (vec![[4.0, -3.0], [0.0, -5.0]], false)] {
            let mut e = editor(json!({"points": {}}), false);
            e.draft.tool = Tool::Arc;
            assert!(click_at(&mut e, [0.0, 0.0], true).is_none());
            assert!(click_at(&mut e, [5.0, 0.0], true).is_none());
            for q in &path[..path.len() - 1] {
                let (x, y) = e.camera.offset(*q, H);
                let mut d = std::mem::take(&mut e.draft);
                d.track(&e, x, y, H, true);
                e.draft = d;
            }
            let p = click_at(&mut e, *path.last().unwrap(), true).unwrap();
            let want = if from_start { json!({"center": "p1", "from": "p2", "to": "p3"}) } else { json!({"center": "p1", "from": "p3", "to": "p2"}) };
            assert_eq!(p["arcs"]["a1"], want, "{p}");
        }
    }

    #[test]
    fn circles_take_their_diameter() {
        let mut e = editor(json!({"points": {}}), false);
        e.draft.tool = Tool::Circle;
        e.draft.construction = true;
        assert!(click_at(&mut e, [2.0, 2.0], true).is_none());
        let p = click_at(&mut e, [5.0, 6.0], true).unwrap();
        assert_eq!(p["circles"]["c1"], json!({"center": "p1", "diameter": 10.0, "construction": true}));
    }

    /// The face's corner and edges are snapped to and projected.
    #[test]
    fn the_face_is_projected_when_used() {
        let mut e = editor(json!({"points": {}}), true);
        assert_eq!(e.draft.face.len(), 4);
        e.draft.tool = Tool::Point;
        let p = click_at(&mut e, [40.2, 0.1], false).unwrap();
        assert_eq!(p["project"].as_object().unwrap().keys().collect::<Vec<_>>(), ["v1"], "{p}");
        let p = click_at(&mut e, [20.0, 30.3], false).unwrap();
        assert_eq!(p["add_constraints"], json!([{"type": "on", "point": "p1", "curve": "e1"}]), "{p}");
        let placed = e.placed.as_ref().unwrap();
        assert!(placed.lines["e1"].construction && placed.solved.points.contains_key("v1"));
        // Projected now, the corner is a point like any: no second projection.
        assert!(click_at(&mut e, [40.2, 0.1], false).is_none());
    }

    #[test]
    fn a_tangent_arc_turns_the_way_the_cursor_is() {
        // Along +X from the origin, to (5, 5): a quarter turn left around (0, 5).
        let (c, ccw) = tangent_arc([0.0, 0.0], [1.0, 0.0], [5.0, 5.0]).unwrap();
        assert_eq!((c, ccw), ([0.0, 5.0], true));
        let (c, ccw) = tangent_arc([0.0, 0.0], [1.0, 0.0], [5.0, -5.0]).unwrap();
        assert_eq!((c, ccw), ([0.0, -5.0], false));
        assert!(tangent_arc([0.0, 0.0], [1.0, 0.0], [7.0, 0.0]).is_none());
    }
}
