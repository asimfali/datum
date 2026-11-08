//! Drawing in a sketch (M7d2, D91): lines (with tangent arcs), arcs, circles and points; snaps
//! to what is already there; constraints inferred while drawing, as in Inventor. Every click is
//! one `edit_sketch` patch, applied to the window's copy at once and sent to the model: one step
//! of undo each. Snap and inference tolerances are pixels and degrees on the screen: they only
//! choose which constraints to add, and the solver then holds those exactly.

use datum_brep::{Curve, Ix, Surface};
use datum_model::SketchPatch;
use datum_ops::Placed;
use serde_json::{Map, Value as Json, json};

use crate::i18n::I18n;
use crate::sketch::SketchEdit;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tool {
    Select,
    Line,
    Arc,
    Circle,
    Point,
    /// Pick a line, an arc, a circle, two points, a point and a line or two lines: dimensioned
    /// at once (two other points: then click where the dimension goes).
    Dimension,
    /// Measure what is clicked (D177): nothing is changed.
    Measure,
    /// Project what is clicked of the features before the sketch (D214): an edge or vertex of a
    /// body, a point, line, arc or circle of another sketch.
    Project,
    /// Place a template (`Draft::template`): a click per anchor.
    Template,
    /// Trim a piece of a curve away, or extend a curve to the next (D239): what the click does
    /// follows from where it is; Shift held extends.
    Trim,
    /// Add a geometric constraint of this kind: click what it ties (D118).
    Constrain(crate::constrain::Kind),
}

/// Where placing a library element is (D184).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stage {
    Anchors,
    Sockets,
    Sizes,
}

/// What a click lands on.
#[derive(Clone, Debug, PartialEq)]
pub enum Snap {
    /// A point of the sketch, its own or a projection's.
    Point(String),
    /// The middle of a line, or halfway round an arc (D210).
    Middle(String),
    /// A curve of the sketch.
    On(String),
    /// Geometry to project (D214): [`Draft::projectable`]'s, by index.
    Source(usize),
    /// Free, but in line with points of the sketch, as AutoCAD's tracking: plumb under point `x`
    /// (its X), level with point `y` (its Y), or both. The new point is tied to them so.
    Track {
        x: Option<String>,
        y: Option<String>,
    },
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
    /// A line leaving the arc that ends the chain along it.
    Tangent(String),
}

/// The end of the line chain being drawn: its point, and how the curve that ends there leaves
/// it (a tangent arc continues it).
#[derive(Clone, Debug, PartialEq)]
pub struct Chain {
    pub point: String,
    pub at: [f64; 2],
    pub last: Option<Heading>,
}

/// The curve that ends a chain and the direction it was travelling.
#[derive(Clone, Debug, PartialEq)]
pub struct Heading {
    pub curve: String,
    pub dir: [f64; 2],
}

/// Geometry the sketch can project (D214, D216), as it falls on its plane: of feature `from`,
/// the projection's `entry` (`id` of a body's edge or vertex, `face` of its face, `element` of a
/// sketch), its shape and how high above the plane it is (of things one over another, the
/// highest is offered).
#[derive(Clone, Debug, PartialEq)]
pub struct Projectable {
    pub from: String,
    pub entry: (&'static str, String),
    pub shape: Flat,
    pub height: f64,
}

/// A shape in sketch coordinates. An arc runs counterclockwise from angle `start` through
/// `sweep` (a full turn: a circle).
#[derive(Clone, Debug, PartialEq)]
pub enum Flat {
    Point([f64; 2]),
    Line([f64; 2], [f64; 2]),
    Arc {
        c: [f64; 2],
        r: f64,
        start: f64,
        sweep: f64,
    },
    /// A face: its loops as closed polylines.
    Region(Vec<Vec<[f64; 2]>>),
    /// An ellipse or an arc of it (D218), as points along it.
    Polyline(Vec<[f64; 2]>),
}

impl Flat {
    /// The point of it nearest `q`, if it is beside it (on a line between its ends, on an arc
    /// within its sweep).
    fn foot(&self, q: [f64; 2]) -> Option<[f64; 2]> {
        match *self {
            Flat::Region(_) => None,
            Flat::Polyline(ref pts) => foot(pts, q),
            Flat::Point(p) => Some(p),
            Flat::Line(a, b) => foot(&[a, b], q),
            Flat::Arc { c, r, start, sweep } => {
                let d = libm::hypot(q[0] - c[0], q[1] - c[1]);
                let turn = (libm::atan2(q[1] - c[1], q[0] - c[0]) - start).rem_euclid(std::f64::consts::TAU);
                (d > 0.0 && turn <= sweep).then(|| [c[0] + (q[0] - c[0]) * r / d, c[1] + (q[1] - c[1]) * r / d])
            }
        }
    }

    /// Whether a face holds `q`: inside an odd number of its loops.
    fn holds(&self, q: [f64; 2]) -> bool {
        let Flat::Region(loops) = self else { return false };
        let crossings = loops
            .iter()
            .flat_map(|l| l.iter().zip(l.iter().cycle().skip(1)))
            .filter(|(a, b)| (a[1] > q[1]) != (b[1] > q[1]) && q[0] < a[0] + (q[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1]));
        crossings.count() % 2 == 1
    }

    /// Points along it, to draw: a polyline per loop of a face, one for anything else.
    pub(crate) fn polylines(&self) -> Vec<Vec<[f64; 2]>> {
        match self {
            Flat::Region(loops) => loops.iter().map(|l| l.iter().chain(l.first()).copied().collect()).collect(),
            _ => vec![self.curve_points()],
        }
    }

    fn curve_points(&self) -> Vec<[f64; 2]> {
        match *self {
            Flat::Region(_) => Vec::new(),
            Flat::Polyline(ref pts) => pts.clone(),
            Flat::Point(p) => vec![p],
            Flat::Line(a, b) => vec![a, b],
            Flat::Arc { c, r, start, sweep } => {
                let n = ((sweep / std::f64::consts::TAU) * 64.0).ceil().max(2.0) as usize;
                (0..=n)
                    .map(|k| {
                        let (s, co) = libm::sincos(start + sweep * k as f64 / n as f64);
                        [c[0] + r * co, c[1] + r * s]
                    })
                    .collect()
            }
        }
    }
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
    /// Arc tool: the start's lean to the centre, and its radius as typed.
    pub start_lean: Option<Lean>,
    pub start_radius: Option<Json>,
    /// A value typed while drawing (the field by the cursor), and what it comes to: the length
    /// of the line, the circle's diameter, the arc's radius.
    pub typed: Option<(Json, f64)>,
    /// Dimension tool: what is picked so far; H or V (its sub-mode); and the value the field
    /// opened with while it shows the picks' dimension (a pick then changes it, H or V too
    /// until a value is typed).
    pub picks: Vec<crate::dims::Pick2>,
    pub axis: Option<crate::dims::Axis>,
    pub asked: Option<String>,
    /// Measure tool: what is measured, and where the last click was (its label goes there).
    pub measured: Vec<crate::measure::Taken>,
    pub measured_at: [f64; 2],
    /// Template tool: the library element placed, the preset chosen, the parameter values
    /// typed (by name), and the parameter the field by the cursor asks.
    pub template: Option<Json>,
    pub preset: Option<String>,
    pub answers: Map<String, Json>,
    pub asking: usize,
    /// The sockets of the element being placed completed so far, in order: what each is tied to,
    /// none for one skipped (D116). Asked after its anchors, before its sizes (D184).
    pub sockets: Vec<Option<String>>,
    /// Enter ended the anchors of the element being placed: the rest are not asked, its sizes
    /// give its size.
    pub anchored: bool,
    /// The next segment of the chain is an arc tangent to the last (T, or a drag from its end).
    pub tangent: bool,
    /// The command ended last: Enter takes it up again.
    pub last: Option<Tool>,
    /// Where the cursor would click, and the inferred constraint of a line.
    pub cursor: Option<Spot>,
    pub lean: Option<Lean>,
    /// What the sketch can project (D214), offered by the projection tool alone, and the
    /// sketch's frame it was found for: found again only when the plane moves.
    pub projectable: Vec<Projectable>,
    pub projectable_on: Option<[datum_math::Vec3; 4]>,
    /// Trim tool: what a click where the cursor is would do.
    pub trim: Option<crate::trim::Plan>,
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
            start_radius: None,
            typed: None,
            picks: Vec::new(),
            axis: None,
            asked: None,
            measured: Vec::new(),
            measured_at: [0.0; 2],
            template: None,
            preset: None,
            answers: Map::new(),
            asking: 0,
            sockets: Vec::new(),
            anchored: false,
            tangent: false,
            last: None,
            cursor: None,
            lean: None,
            projectable: Vec::new(),
            projectable_on: None,
            trim: None,
        }
    }
}

/// Pixels within which a click lands on a point (or a middle), and on a curve; a point dragged
/// onto another joins it.
pub(crate) const SNAP_POINT: f64 = 10.0;
/// Pixels within which the cursor is in line with a point of the sketch (tracking).
const TRACK: f64 = 6.0;
const SNAP_CURVE: f64 = 8.0;
/// Degrees within which a line is taken as horizontal, vertical, parallel or perpendicular.
const LEAN_DEG: f64 = 3.0;

const DRAFT: egui::Color32 = egui::Color32::from_rgb(235, 110, 20);
/// What the trim tool would take away.
const GONE: egui::Color32 = egui::Color32::from_rgb(220, 40, 40);

/// What of `sources` (features by name) sketch `placed` can project, as it falls on its plane:
/// a body's vertices and edges, another sketch's points and curves; an arc or circle only if
/// it lies parallel to the plane (else it would fall as an ellipse), a line seen end on as its
/// vertex. Of things that fall on one another (a box's top and bottom edges), the one nearest
/// the eye, above the plane.
pub fn projectable(sources: &[(String, datum_graph::ValueRef)], placed: &Placed) -> Vec<Projectable> {
    let [o, x, y, z] = placed.frame;
    let flat = |p: datum_math::Vec3| [(p - o).dot(x), (p - o).dot(y)];
    let height = |p: datum_math::Vec3| (p - o).dot(z);
    let angle = |c: [f64; 2], p: [f64; 2]| libm::atan2(p[1] - c[1], p[0] - c[0]);
    // An arc from `a` to `b` counterclockwise about `axis`, as it falls.
    // A circle's arc about `axis` from `a` to `b` (none: whole) as it falls: an arc, or at a
    // slant an ellipse's, as points along it (D218); none seen edge on.
    let arc = |c: datum_math::Vec3, r: f64, a: datum_math::Vec3, b: Option<datum_math::Vec3>, axis: datum_math::Vec3| {
        let tau = std::f64::consts::TAU;
        let n = axis * (1.0 / axis.dot(axis).sqrt());
        let (u, w) = ((a - c) * (1.0 / r), n.cross((a - c) * (1.0 / r)));
        let sweep = b.map_or(tau, |b| libm::atan2((b - c).dot(w), (b - c).dot(u)).rem_euclid(tau));
        let sweep = if sweep == 0.0 { tau } else { sweep };
        if datum_ops::parallel(axis, z) {
            let (fc, (fa, fb)) = (flat(c), if axis.dot(z) > 0.0 { (flat(a), b.map_or(flat(a), flat)) } else { (b.map_or(flat(a), flat), flat(a)) });
            let turn = (angle(fc, fb) - angle(fc, fa)).rem_euclid(tau);
            return Some(Flat::Arc { c: fc, r, start: angle(fc, fa), sweep: if turn == 0.0 { tau } else { turn } });
        }
        if n.dot(z) == 0.0 {
            return None;
        }
        let k = ((sweep / tau) * 64.0).ceil().max(2.0) as usize;
        Some(Flat::Polyline(
            (0..=k)
                .map(|i| {
                    let (sn, cs) = libm::sincos(sweep * i as f64 / k as f64);
                    flat(c + (u * cs + w * sn) * r)
                })
                .collect(),
        ))
    };
    let mut out: Vec<Projectable> = Vec::new();
    for (from, v) in sources {
        let mut put = |key: &'static str, name: String, shape: Flat, at: datum_math::Vec3| {
            out.push(Projectable { from: from.clone(), entry: (key, name), shape, height: height(at) });
        };
        if let Some(body) = v.body() {
            for vx in body.vertices() {
                put("id", format!("{:?}", vx.id), Flat::Point(flat(vx.point)), vx.point);
            }
            // Each edge as it falls; none if it cannot be projected.
            let edges: Vec<Option<Flat>> = body
                .edges()
                .iter()
                .map(|edge| {
                    let (a, b) = (body.half_edges()[edge.half as usize].origin, body.half_edges()[body.next(edge.half) as usize].origin);
                    let (pa, pb) = (body.point(a), body.point(b));
                    match body.edge_curve(edge) {
                        None => Some(Flat::Line(flat(pa), flat(pb))),
                        Some(Curve::Arc(c)) => arc(c.center, c.radius, pa, Some(pb), c.axis),
                        Some(Curve::Rolled { .. }) => None,
                    }
                })
                .collect();
            for (e, shape) in body.edges().iter().zip(&edges) {
                if let Some(shape) = shape.as_ref().filter(|s| !matches!(s, Flat::Line(a, b) if a == b)) {
                    put("id", format!("{:?}", e.id), shape.clone(), body.point(body.half_edges()[e.half as usize].origin));
                }
            }
            // A plane face not seen edge on, all its edges projectable: its loops, each edge's
            // points in the loop's order.
            for (f, face) in body.faces().iter().enumerate() {
                let Surface::Plane(plane) = face.surface else { continue };
                if plane.normal.dot(z) == 0.0 {
                    continue;
                }
                let loops: Option<Vec<Vec<[f64; 2]>>> = body
                    .face_loops(f as Ix)
                    .map(|l| {
                        let mut pts = Vec::new();
                        for h in body.loop_half_edges(l) {
                            let he = &body.half_edges()[h as usize];
                            let mut along = edges[he.edge as usize].as_ref()?.curve_points();
                            if body.edges()[he.edge as usize].half != h {
                                along.reverse();
                            }
                            along.pop();
                            pts.extend(along);
                        }
                        Some(pts)
                    })
                    .collect();
                if let Some(loops) = loops {
                    let at = body.point(body.half_edges()[body.loop_half_edges(body.face_loops(f as Ix).start).start as usize].origin);
                    put("face", format!("{:?}", face.id), Flat::Region(loops), at);
                }
            }
        } else if let Some(s) = v.sketch().filter(|s| s.developed.is_none()) {
            let [so, sx, sy, sz] = s.frame;
            let at = |n: &str| s.solved.points.get(n).map(|q| so + sx * q.x + sy * q.y);
            for n in s.solved.points.keys().filter(|n| n.as_str() != datum_ops::ORIGIN) {
                let p = at(n).unwrap_or(so);
                put("element", n.clone(), Flat::Point(flat(p)), p);
            }
            for (n, l) in &s.lines {
                if let (Some(a), Some(b)) = (at(&l.from), at(&l.to))
                    && flat(a) != flat(b)
                {
                    put("element", n.clone(), Flat::Line(flat(a), flat(b)), a);
                }
            }
            for (n, a) in &s.arcs {
                if let (Some(c), Some(p), Some(q)) = (at(&a.center), at(&a.from), at(&a.to)) {
                    let r = (p - c).dot(p - c).sqrt();
                    if let Some(shape) = arc(c, r, p, Some(q), sz) {
                        put("element", n.clone(), shape, c);
                    }
                }
            }
            for (n, c) in &s.circles {
                if let (Some(centre), Some(d)) = (at(&c.center), c.diameter.value())
                    && let Some(shape) = arc(centre, 0.5 * d, centre + sx * (0.5 * d), None, sz)
                {
                    put("element", n.clone(), shape, centre);
                }
            }
        }
    }
    // Of equal shapes the highest, then the first given: sorted by shape, height down, and kept
    // once. Bits compare exactly: what falls on one place falls there bit for bit or is another.
    let key = |f: &Flat| -> Vec<u64> {
        match f {
            Flat::Polyline(pts) => pts.iter().flat_map(|p| p.map(f64::to_bits)).collect(),
            // Its corners, in any order.
            Flat::Region(loops) => {
                let mut pts: Vec<[u64; 2]> = loops.iter().flatten().map(|p| p.map(f64::to_bits)).collect();
                pts.sort_unstable();
                pts.concat()
            }
            &Flat::Point(p) => p.map(f64::to_bits).to_vec(),
            // Either way along.
            &Flat::Line(a, b) => {
                let mut ends = [a.map(f64::to_bits), b.map(f64::to_bits)];
                ends.sort_unstable();
                ends.concat()
            }
            &Flat::Arc { c, r, start, sweep } => [c[0], c[1], r, start, sweep].map(f64::to_bits).to_vec(),
        }
    };
    // A face and an edge never share a key: a face's has an even number of corners' bits, and
    // is marked.
    let mut keyed: Vec<(&'static str, Vec<u64>, usize)> = out.iter().enumerate().map(|(i, p)| (p.entry.0, key(&p.shape), i)).collect();
    keyed.sort_by(|a, b| {
        (a.0 == "face").cmp(&(b.0 == "face")).then(a.1.cmp(&b.1)).then(out[b.2].height.total_cmp(&out[a.2].height)).then(a.2.cmp(&b.2))
    });
    keyed.dedup_by(|b, a| (a.0 == "face") == (b.0 == "face") && a.1 == b.1);
    let mut kept: Vec<usize> = keyed.into_iter().map(|(_, _, i)| i).collect();
    kept.sort_unstable();
    kept.into_iter().map(|i| out[i].clone()).collect()
}

impl Draft {
    /// Cancels what is half drawn; true if there was something.
    pub fn cancel(&mut self) -> bool {
        let busy = self.chain.is_some() || !self.clicks.is_empty() || !self.picks.is_empty() || !self.sockets.is_empty() || !self.measured.is_empty();
        self.sockets.clear();
        self.anchored = false;
        self.measured.clear();
        self.chain = None;
        self.clicks.clear();
        self.picks.clear();
        self.axis = None;
        self.asked = None;
        self.tangent = false;
        self.typed = None;
        self.trim = None;
        busy
    }

    /// Takes up `tool`, what is half drawn dropped.
    pub fn take_up(&mut self, tool: Tool) {
        self.cancel();
        if self.tool != Tool::Select {
            self.last = Some(self.tool);
        }
        self.tool = tool;
    }

    /// Enter or a right click with no command on: the last one taken up again, as in other CADs.
    /// A library element only while it is still chosen.
    pub fn again(&mut self) {
        if let Some(t) = self.last.filter(|t| *t != Tool::Template || self.template.is_some()) {
            self.take_up(t);
        }
    }

    /// A tool's button clicked: the tool taken up, or, clicked again, the command ended as Esc
    /// ends it: by the mouse alone, as in Inventor.
    pub fn press(&mut self, tool: Tool) {
        if self.tool == tool {
            self.escape();
        } else {
            self.take_up(tool);
        }
    }

    /// Starts placing library element `t`: no preset, nothing typed yet.
    pub fn choose(&mut self, t: Json) {
        self.cancel();
        self.tool = Tool::Template;
        self.template = Some(t);
        self.preset = None;
        self.answers.clear();
        self.asking = 0;
    }

    /// Whether the element placed goes by its presets (F4, D176): it has some, the mode is on.
    pub fn by_presets(&self, modes: &crate::sketch::Modes) -> bool {
        modes.presets && self.tool == Tool::Template && self.template.as_ref().and_then(|t| t["presets"].as_object()).is_some_and(|p| !p.is_empty())
    }

    /// Where placing a library element is (D184): its anchors are clicked first (by presets only
    /// the first: the preset gives the size), then its sockets, then its sizes are typed or its
    /// preset picked. None: no element is being placed.
    pub fn stage(&self, modes: &crate::sketch::Modes) -> Option<Stage> {
        let t = self.template.as_ref().filter(|_| self.tool == Tool::Template)?;
        let anchors = if self.by_presets(modes) { 1 } else { datum_model::library::anchors(t).len() };
        Some(if self.clicks.len() < anchors && !self.anchored {
            Stage::Anchors
        } else if self.sockets.len() < datum_model::library::sockets(t).len() {
            Stage::Sockets
        } else {
            Stage::Sizes
        })
    }

    /// Whether the sizes are asked once the anchors and sockets are in: by presets, or the element
    /// has parameters; otherwise it goes in at once.
    fn asks_sizes(&self, modes: &crate::sketch::Modes) -> bool {
        self.by_presets(modes) || self.template.as_ref().and_then(|t| datum_model::library::parameter_names(t).ok()).is_some_and(|n| !n.is_empty())
    }

    /// Enter while placing an element: the anchors end (with none clicked yet, the first goes
    /// where the cursor is), the socket asked is skipped, the sizes are taken.
    pub fn enter(&mut self, edit: &SketchEdit) -> Option<Map<String, Json>> {
        self.step(edit, None)
    }

    /// A click (`spot`) or Enter (none) while placing an element (D184): an anchor (Enter: where
    /// the cursor is), a socket completed by what it takes (Enter skips it), the sizes taken. The
    /// element and its sockets go in as one change once nothing is left to ask; the values typed
    /// stay for the next.
    fn step(&mut self, edit: &SketchEdit, spot: Option<Spot>) -> Option<Map<String, Json>> {
        match self.stage(&edit.modes)? {
            Stage::Anchors => match spot {
                Some(s) => self.clicks.push(s),
                None => {
                    if self.clicks.is_empty() {
                        self.clicks.push(self.cursor.clone()?);
                    }
                    self.anchored = true;
                }
            },
            Stage::Sockets => match spot.map(|s| s.snap) {
                None => self.sockets.push(None),
                Some(Snap::Point(n) | Snap::Middle(n) | Snap::On(n)) => {
                    let (_, kind) = datum_model::library::sockets(self.template.as_ref()?).into_iter().nth(self.sockets.len())?;
                    if !datum_model::library::fits(edit.placed.as_ref().ok()?, &kind, &n) {
                        return None;
                    }
                    self.sockets.push(Some(n));
                }
                Some(_) => return None,
            },
            Stage::Sizes => return self.put(edit),
        }
        (self.stage(&edit.modes) == Some(Stage::Sizes) && !self.asks_sizes(&edit.modes)).then(|| self.put(edit)).flatten()
    }

    /// The element at its anchors, with its sockets completed: one change.
    fn put(&mut self, edit: &SketchEdit) -> Option<Map<String, Json>> {
        let t = self.template.as_ref()?;
        let mut patch = Patch::new(edit);
        let links = crate::templates::place(&mut patch, t, self.preset.as_deref(), &self.answers, &self.clicks, self.construction, &edit.params)?;
        for (link, target) in links.iter().zip(&self.sockets) {
            if let Some(target) = target {
                patch.constrain(link.to(target));
            }
        }
        self.clicks.clear();
        self.sockets.clear();
        self.anchored = false;
        patch.done()
    }

    /// Whether a value typed now would size what is being drawn: a line from the chain's end,
    /// a circle from its centre, an arc's radius from its centre.
    pub fn takes_value(&self) -> bool {
        match self.tool {
            Tool::Line => self.chain.is_some() && !self.tangent,
            Tool::Circle => self.clicks.len() == 1,
            Tool::Arc => self.clicks.len() == 1,
            _ => false,
        }
    }

    /// Esc: ends the command, as in Inventor: what is half drawn goes, the tool is Select.
    pub fn escape(&mut self) {
        self.cancel();
        if self.tool != Tool::Select {
            self.last = Some(self.tool);
        }
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
        // Projecting: only what is not projected yet, a point before a curve.
        if self.tool == Tool::Project {
            let done = edit.projected();
            let left = || self.projectable.iter().enumerate().filter(|(_, c)| !done.contains(&(c.from.clone(), c.entry.1.clone())));
            let points = left().filter_map(|(i, c)| match c.shape {
                Flat::Point(q) => Some((px(q), Spot { at: q, snap: Snap::Source(i) })),
                _ => None,
            });
            let curves = left()
                .filter(|(_, c)| !matches!(c.shape, Flat::Point(_)))
                .filter_map(|(i, c)| c.shape.foot(at).map(|q| (px(q), Spot { at: q, snap: Snap::Source(i) })));
            // Away from edges, the highest face under the cursor.
            let face = || {
                left()
                    .filter(|(_, c)| c.shape.holds(at))
                    .max_by(|a, b| a.1.height.total_cmp(&b.1.height))
                    .map(|(i, _)| Spot { at, snap: Snap::Source(i) })
            };
            return nearest(points.collect(), SNAP_POINT).or_else(|| nearest(curves.collect(), SNAP_CURVE)).or_else(face).unwrap_or(free_spot);
        }
        let points = p
            .solved
            .points
            .iter()
            .filter(|(n, _)| Some(n.as_str()) != skip)
            .map(|(n, v)| (px([v.x, v.y]), Spot { at: [v.x, v.y], snap: Snap::Point(n.clone()) }));
        let middles = p
            .lines
            .keys()
            .chain(p.arcs.keys())
            .filter_map(|n| SketchEdit::middle(p, n).map(|m| (px(m), Spot { at: m, snap: Snap::Middle(n.clone()) })));
        if let Some(s) = nearest(points.chain(middles).collect(), SNAP_POINT) {
            return s;
        }
        // On a curve of the sketch: the nearest point of it.
        let mut on = Vec::new();
        for n in p.lines.keys().chain(p.arcs.keys()).chain(p.circles.keys()) {
            if let Some(q) = SketchEdit::curve_points(p, n).and_then(|pts| foot(&pts, at)) {
                on.push((px(q), Spot { at: q, snap: Snap::On(n.clone()) }));
            }
        }
        nearest(on, SNAP_CURVE).unwrap_or(free_spot)
    }

    /// The constraint a line from `from` to the free spot `to` leans to, and where that puts
    /// its end: on along the arc the chain ends in (`last`, with the direction it arrived in),
    /// then horizontal or vertical, then parallel or perpendicular to the nearest line by
    /// angle, the line the chain ends in first: of parallel lines, the one drawn from. None
    /// beyond a few degrees.
    pub fn lean(&self, edit: &SketchEdit, from: [f64; 2], to: [f64; 2], last: Option<&Heading>) -> Option<(Lean, [f64; 2])> {
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
        let arc_end = last.filter(|h| edit.placed.as_ref().is_ok_and(|p| p.arcs.contains_key(&h.curve)));
        if let Some(h) = arc_end
            && degrees(d, h.dir) <= LEAN_DEG
        {
            return Some((Lean::Tangent(h.curve.clone()), along(h.dir)));
        }
        if off(angle, 0.0) <= LEAN_DEG {
            return Some((Lean::Horizontal, [to[0], from[1]]));
        }
        if off(angle, std::f64::consts::FRAC_PI_2) <= LEAN_DEG {
            return Some((Lean::Vertical, [from[0], to[1]]));
        }
        let p = edit.placed.as_ref().ok()?;
        let mut best: Option<(f64, Lean, [f64; 2])> = None;
        // The chain's own line first: a tie stays with it.
        let own = last.map(|h| &h.curve).filter(|n| p.lines.contains_key(*n));
        let lines = own.into_iter().chain(p.lines.keys().filter(|n| Some(*n) != own));
        for (n, pts) in lines.filter_map(|n| Some((n, SketchEdit::curve_points(p, n)?))) {
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

    /// The free spot `at` moved in line with the nearest points of the sketch within [`TRACK`]
    /// pixels: onto one's X and another's Y; or, on a line leaning from `along` (its start and
    /// the spot on its lean), to where that line meets the nearest point's X or Y. The chain's own
    /// end is left to the lean. None if no point is that near in line.
    fn tracking(&self, edit: &SketchEdit, at: [f64; 2], height: f64, along: Option<[f64; 2]>) -> Option<Spot> {
        let p = edit.placed.as_ref().ok()?;
        let within = TRACK * edit.camera.per_pixel(height);
        let skip = self.chain.as_ref().map(|c| c.point.as_str());
        let points = || p.solved.points.iter().filter(move |(n, _)| Some(n.as_str()) != skip).map(|(n, v)| (n, [v.x, v.y]));
        if let Some(from) = along {
            // Where the line from `from` through `at` reaches coordinate `k` of `q`, if it does.
            let d = [at[0] - from[0], at[1] - from[1]];
            let meet = |k: usize, q: [f64; 2]| {
                (d[k] != 0.0).then(|| {
                    let t = (q[k] - from[k]) / d[k];
                    [from[0] + d[0] * t, from[1] + d[1] * t]
                })
            };
            let (n, k, q) = points()
                .flat_map(|(n, q)| [0, 1].into_iter().filter_map(move |k| meet(k, q).map(|m| (n, k, m))))
                .map(|(n, k, m)| (libm::hypot(m[0] - at[0], m[1] - at[1]), n, k, m))
                .filter(|(dist, ..)| *dist <= within)
                .min_by(|a, b| a.0.total_cmp(&b.0))
                .map(|(_, n, k, m)| (n.clone(), k, m))?;
            let (x, y) = if k == 0 { (Some(n), None) } else { (None, Some(n)) };
            return Some(Spot { at: q, snap: Snap::Track { x, y } });
        }
        let nearest = |k: usize| {
            points()
                .map(|(n, q)| ((q[k] - at[k]).abs(), n, q[k]))
                .filter(|(d, ..)| *d <= within)
                .min_by(|a, b| a.0.total_cmp(&b.0))
                .map(|(_, n, c)| (n.clone(), c))
        };
        let (x, y) = (nearest(0), nearest(1));
        if x.is_none() && y.is_none() {
            return None;
        }
        let spot_at = [x.as_ref().map_or(at[0], |t| t.1), y.as_ref().map_or(at[1], |t| t.1)];
        Some(Spot { at: spot_at, snap: Snap::Track { x: x.map(|t| t.0), y: y.map(|t| t.0) } })
    }

    /// Follows the cursor: where it would click, what it leans to, and the arc's sweep.
    pub fn track(&mut self, edit: &SketchEdit, dx: f64, dy: f64, height: f64, free: bool) {
        if self.tool == Tool::Trim {
            self.trim = crate::trim::plan(edit, dx, dy, height, free);
            self.cursor = Some(Spot { at: edit.camera.at(dx, dy, height), snap: Snap::Free });
            return;
        }
        let mut spot = self.spot(edit, dx, dy, height, free);
        self.lean = None;
        let leaning = !free && spot.snap == Snap::Free;
        let found = match (self.tool, &self.chain, &self.clicks[..]) {
            _ if !leaning => None,
            // Inferring off: no constraint but the tangent asked for.
            _ if !edit.modes.infer && !self.tangent => None,
            (Tool::Line, Some(c), _) if !self.tangent => self.lean(edit, c.at, spot.at, c.last.as_ref()),
            (Tool::Line, Some(c @ Chain { last: Some(h), .. }), _) => match tangent_arc(c.at, h, spot.at) {
                Some(_) => tangent_lean(c.at, h, spot.at),
                // Straight on: a line, leaning as one does.
                None => self.lean(edit, c.at, spot.at, c.last.as_ref()),
            },
            (Tool::Arc, _, [c] | [c, _]) => axis_lean(c.at, spot.at),
            _ => None,
        };
        if let Some((lean, at)) = found {
            spot.at = at;
            self.lean = Some(lean);
        }
        // In line with a point of the sketch, along what the lean leaves free.
        let starts = matches!((self.tool, &self.clicks[..]), (Tool::Line | Tool::Point, _) | (Tool::Circle | Tool::Arc, []));
        if leaning && starts && !self.tangent {
            // A lean fixes the line's direction: the spot slides along it (a horizontal line meets
            // only an X, a vertical one only a Y, a tangent or parallel one either).
            let along = self.lean.as_ref().and(self.chain.as_ref()).map(|c| c.at);
            if let Some(tracked) = self.tracking(edit, spot.at, height, along) {
                spot = tracked;
            }
        }
        // A typed size: as far from where the line, circle or arc starts, toward the cursor.
        let from = match (self.tool, &self.chain, &self.clicks[..]) {
            (Tool::Line, Some(c), _) if !self.tangent => Some((c.at, 1.0)),
            (Tool::Circle, _, [c]) => Some((c.at, 0.5)),
            (Tool::Arc, _, [c]) => Some((c.at, 1.0)),
            _ => None,
        };
        if let (Some((_, size)), Some((o, k))) = (&self.typed, from) {
            let d = [spot.at[0] - o[0], spot.at[1] - o[1]];
            let len = libm::hypot(d[0], d[1]);
            if len > 0.0 && *size > 0.0 {
                spot = Spot { at: [o[0] + d[0] / len * size * k, o[1] + d[1] / len * size * k], snap: Snap::Free };
            }
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
            Tool::Select | Tool::Dimension | Tool::Measure | Tool::Constrain(_) => return None,
            Tool::Trim => return self.trim.take().map(|t| t.patch),
            Tool::Project => {
                let Snap::Source(i) = spot.snap else { return None };
                let c = self.projectable.get(i)?;
                let name = patch.fresh(match c.shape {
                    Flat::Point(_) => "v",
                    Flat::Region(_) => "f",
                    _ => "e",
                });
                // A line of the blank is the sketch's own blank's: from no input.
                let mut entry = if c.entry.0 == "blank" { json!({}) } else { json!({"from": c.from}) };
                entry[c.entry.0] = json!(c.entry.1);
                patch.set("project", &name, entry);
            }
            Tool::Template => return self.step(edit, Some(spot)),
            Tool::Point => {
                // A point already there needs no other.
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
                let diameter = self.typed.take().map_or(json!(short(2.0 * r)), |t| t.0);
                patch.add("circles", &name, json!({"center": c, "diameter": diameter}), self.construction);
                self.clicks.clear();
            }
            Tool::Arc => {
                if self.clicks.len() < 2 {
                    if self.clicks.len() == 1 {
                        self.sweep = 0.0;
                        self.start_lean = self.lean.clone();
                        self.start_radius = self.typed.take().map(|t| t.0);
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
                    Snap::Point(_) => spot,
                    _ => Spot { at: [c.at[0] + (spot.at[0] - c.at[0]) * r / d, c.at[1] + (spot.at[1] - c.at[1]) * r / d], snap: Snap::Free },
                };
                let (cn, sn, en) = (patch.point(&c), patch.point(&s), patch.point(&end));
                let name = patch.fresh("a");
                let (from, to) = if self.sweep > 0.0 { (sn.clone(), en.clone()) } else { (en.clone(), sn.clone()) };
                patch.add("arcs", &name, json!({"center": cn, "from": from, "to": to}), self.construction);
                for (lean, p) in [(self.start_lean.take(), sn), (self.lean.clone(), en)] {
                    patch.level(lean.as_ref(), &cn, &p);
                }
                if let Some(r) = self.start_radius.take() {
                    patch.constrain(json!({"type": "radius", "arc": name, "value": r}));
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
                    (Some(h), true) => match tangent_arc(chain.at, h, spot.at) {
                        Some((c, ccw)) => Some((h.curve.clone(), c, ccw)),
                        None if behind(chain.at, h, spot.at) => return None,
                        None => None,
                    },
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
                    Heading { curve: name, dir }
                } else {
                    let l = libm::hypot(spot.at[0] - chain.at[0], spot.at[1] - chain.at[1]);
                    if l == 0.0 {
                        return None;
                    }
                    let name = patch.fresh("l");
                    patch.add("lines", &name, json!({"from": from, "to": end}), self.construction);
                    if let Some((v, _)) = self.typed.take() {
                        patch.constrain(json!({"type": "length", "line": name, "value": v}));
                    }
                    match &self.lean {
                        Some(Lean::Horizontal) => patch.constrain(json!({"type": "horizontal", "line": name})),
                        Some(Lean::Vertical) => patch.constrain(json!({"type": "vertical", "line": name})),
                        Some(Lean::Parallel(o)) => patch.constrain(json!({"type": "parallel", "lines": [o, name]})),
                        Some(Lean::Perpendicular(o)) => patch.constrain(json!({"type": "perpendicular", "lines": [o, name]})),
                        Some(Lean::Tangent(a)) => patch.constrain(json!({"type": "tangent", "curves": [a, name]})),
                        None => {}
                    }
                    let dir = [(spot.at[0] - chain.at[0]) / l, (spot.at[1] - chain.at[1]) / l];
                    Heading { curve: name, dir }
                };
                self.tangent = false;
                // Closing on a point of the sketch ends the chain, as Inventor does.
                self.chain = match spot.snap {
                    Snap::Point(_) => None,
                    _ => Some(Chain { point: end, at: spot.at, last: Some(next_dir) }),
                };
            }
        }
        patch.done()
    }

    /// Draws what is half drawn and where the cursor would click.
    pub fn paint(&self, edit: &SketchEdit, painter: &egui::Painter, rect: egui::Rect, i18n: &I18n) {
        let Some(spot) = self.cursor.as_ref().filter(|_| self.tool != Tool::Select) else { return };
        let height = f64::from(rect.height());
        let screen = |q: [f64; 2]| {
            let (x, y) = edit.camera.offset(q, height);
            rect.center() + egui::vec2(x as f32, y as f32)
        };
        let stroke = egui::Stroke::new(1.6, DRAFT);
        let polyline = |pts: Vec<[f64; 2]>| dashed(painter, &pts.into_iter().map(screen).collect::<Vec<_>>(), stroke, 6.0, 3.0);
        let arc = |c: [f64; 2], r: f64, start: f64, sweep: f64| {
            let n = ((sweep.abs() / std::f64::consts::TAU) * 64.0).ceil().max(2.0) as usize;
            (0..=n)
                .map(|k| {
                    let (s, co) = libm::sincos(start + sweep * k as f64 / n as f64);
                    [c[0] + r * co, c[1] + r * s]
                })
                .collect::<Vec<_>>()
        };
        // Projecting: what can be projected, faint; what the click would project, bold.
        if self.tool == Tool::Project {
            let faint = egui::Stroke::new(1.0, DRAFT.gamma_multiply(0.45));
            let bold = egui::Stroke::new(2.5, DRAFT);
            let done = edit.projected();
            for (i, c) in self.projectable.iter().enumerate().filter(|(_, c)| !done.contains(&(c.from.clone(), c.entry.1.clone()))) {
                let stroke = if spot.snap == Snap::Source(i) { bold } else { faint };
                match c.shape {
                    Flat::Point(q) => {
                        painter.circle_stroke(screen(q), 2.5, stroke);
                    }
                    // A face only when it would be projected: its edges are drawn anyway.
                    Flat::Region(_) if stroke == faint => {}
                    _ => {
                        for line in c.shape.polylines() {
                            painter.add(egui::Shape::line(line.into_iter().map(screen).collect(), stroke));
                        }
                    }
                }
            }
        }
        // Trimming: the piece a click takes away, red; the length it adds, dashed.
        if let Some(t) = self.trim.as_ref().filter(|_| self.tool == Tool::Trim) {
            let pts = |v: &[[f64; 2]]| v.iter().copied().map(screen).collect::<Vec<_>>();
            if t.gone.len() > 1 {
                dashed(painter, &pts(&t.gone), egui::Stroke::new(3.0, GONE), 6.0, 3.0);
            }
            if t.added.len() > 1 {
                dashed(painter, &pts(&t.added), egui::Stroke::new(2.5, DRAFT), 6.0, 3.0);
            }
        }
        // What the dimension or constraint tool has picked so far, marked.
        if let Ok(p) = &edit.placed {
            let mark = egui::Stroke::new(3.0, DRAFT);
            for pick in &self.picks {
                match pick {
                    crate::dims::Pick2::Point(n) => {
                        if let Some(q) = p.solved.points.get(n) {
                            painter.circle_stroke(screen([q.x, q.y]), 7.0, mark);
                        }
                    }
                    crate::dims::Pick2::Curve(n) => {
                        if let Some(pts) = SketchEdit::curve_points(p, n) {
                            painter.add(egui::Shape::line(pts.into_iter().map(screen).collect(), mark));
                        }
                    }
                }
            }
        }
        match (self.tool, &self.chain, &self.clicks[..]) {
            (Tool::Line, Some(c), _) => match (&c.last, self.tangent) {
                (Some(h), true) => {
                    if let Some((centre, ccw)) = tangent_arc(c.at, h, spot.at) {
                        let r = libm::hypot(c.at[0] - centre[0], c.at[1] - centre[1]);
                        let a0 = libm::atan2(c.at[1] - centre[1], c.at[0] - centre[0]);
                        let a1 = libm::atan2(spot.at[1] - centre[1], spot.at[0] - centre[0]);
                        let sweep = if ccw { (a1 - a0).rem_euclid(std::f64::consts::TAU) } else { -(a0 - a1).rem_euclid(std::f64::consts::TAU) };
                        polyline(arc(centre, r, a0, sweep));
                    } else if !behind(c.at, h, spot.at) {
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
            (Tool::Template, _, clicks) => {
                // The cursor is the anchor asked; once they are clicked, the element stays there.
                let asked = (self.stage(&edit.modes) == Some(Stage::Anchors)).then_some(spot.at);
                if let Some(t) = &self.template {
                    let at: Vec<[f64; 2]> = clicks.iter().map(|c| c.at).chain(asked).collect();
                    for line in crate::templates::outline(t, self.preset.as_deref(), &self.answers, &at, &edit.params) {
                        polyline(line);
                    }
                }
            }
            (Tool::Arc, _, [c, s]) => {
                let r = libm::hypot(s.at[0] - c.at[0], s.at[1] - c.at[1]);
                polyline(arc(c.at, r, libm::atan2(s.at[1] - c.at[1], s.at[0] - c.at[0]), self.sweep));
            }
            _ => {}
        }
        // The dimension the picks would make, by the cursor.
        if self.tool == Tool::Dimension && !self.picks.is_empty() && edit.field.is_none() {
            let text = match edit.dimension(&self.picks, spot.at, self.axis) {
                Some((c, _, now)) => {
                    let kind = match c["type"].as_str() {
                        Some("horizontal_distance") => "H ",
                        Some("vertical_distance") => "V ",
                        Some("radius") => "R",
                        Some("arc_length") => i18n.t("dim-arc", "arc "),
                        _ => "",
                    };
                    format!("{kind}{}{}", short(now), if c["type"] == "angle" { "°" } else { "" })
                }
                None => i18n.f("dim-picked", "{ $n } picked: a second, or click where the dimension goes", &[("n", &self.picks.len().to_string())]),
            };
            painter.text(screen(spot.at) + egui::vec2(12.0, 12.0), egui::Align2::LEFT_TOP, text, egui::FontId::proportional(13.0), DRAFT);
        }
        // Tracking: dashed from the points the spot is in line with.
        if let (Snap::Track { x, y }, Ok(p)) = (&spot.snap, &edit.placed) {
            let thin = egui::Stroke::new(1.0, DRAFT);
            for q in [x, y].into_iter().flatten().filter_map(|n| p.solved.points.get(n)) {
                dashed(painter, &[screen([q.x, q.y]), screen(spot.at)], thin, 4.0, 4.0);
            }
        }
        // Where the click lands, and what it ties to.
        let at = screen(spot.at);
        painter.circle_stroke(at, 5.0, egui::Stroke::new(1.5, DRAFT));
        let mark: std::borrow::Cow<str> = match (&spot.snap, &self.lean) {
            (Snap::Point(_), _) => i18n.t("snap-point", "point").into(),
            (Snap::Middle(_), _) => i18n.t("snap-middle", "middle").into(),
            (Snap::On(_), _) => i18n.t("snap-on", "on").into(),
            (Snap::Source(i), _) => match self.projectable.get(*i) {
                Some(c) if c.entry.0 == "element" => format!("{}: {}", c.from, c.entry.1).into(),
                Some(c) if matches!(c.shape, Flat::Point(_)) => format!("{}: {}", c.from, i18n.t("snap-vertex", "vertex")).into(),
                Some(c) if matches!(c.shape, Flat::Region(_)) => format!("{}: {}", c.from, i18n.t("snap-face", "face")).into(),
                Some(c) => format!("{}: {}", c.from, i18n.t("snap-edge", "edge")).into(),
                None => "".into(),
            },
            (Snap::Free | Snap::Track { .. }, Some(Lean::Horizontal)) => "H".into(),
            (Snap::Free | Snap::Track { .. }, Some(Lean::Vertical)) => "V".into(),
            (Snap::Free | Snap::Track { .. }, Some(Lean::Parallel(_))) => i18n.t("snap-parallel", "parallel").into(),
            (Snap::Free | Snap::Track { .. }, Some(Lean::Perpendicular(_))) => i18n.t("snap-perpendicular", "perpendicular").into(),
            (Snap::Free | Snap::Track { .. }, Some(Lean::Tangent(_))) => i18n.t("snap-tangent", "tangent").into(),
            (Snap::Free | Snap::Track { .. }, None) => "".into(),
        };
        if !mark.is_empty() {
            painter.text(at + egui::vec2(9.0, -9.0), egui::Align2::LEFT_BOTTOM, mark, egui::FontId::proportional(12.0), DRAFT);
        }
    }
}

/// The centre of the arc leaving `from` as `heading` does and passing through `to`, and whether
/// it turns counterclockwise; None if `to` lies straight ahead or behind, within a line's lean:
/// no arc. Nearer than that the arc only grows, to 10¹⁸ from rounding alone (the cursor, a
/// line's middle are on it only as near as rounding goes), too large to draw or to hold.
pub fn tangent_arc(from: [f64; 2], heading: &Heading, to: [f64; 2]) -> Option<([f64; 2], bool)> {
    let dir = heading.dir;
    let v = [to[0] - from[0], to[1] - from[1]];
    if v == [0.0, 0.0] || !(LEAN_DEG..=180.0 - LEAN_DEG).contains(&degrees(v, dir)) {
        return None;
    }
    let n = [-dir[1], dir[0]];
    let across = v[0] * n[0] + v[1] * n[1];
    // The centre on the normal at `from`, as far from `to` as from `from`.
    let r = (v[0] * v[0] + v[1] * v[1]) / (2.0 * across);
    Some(([from[0] + n[0] * r, from[1] + n[1] * r], r > 0.0))
}

/// Whether `to` lies behind `from` as `heading` goes: no tangent arc goes there, and a line
/// back over the last segment is no tangent continuation either, so it is not drawn.
fn behind(from: [f64; 2], heading: &Heading, to: [f64; 2]) -> bool {
    (to[0] - from[0]) * heading.dir[0] + (to[1] - from[1]) * heading.dir[1] < 0.0
}

/// `pts` (on screen) dashed, only where the painter shows them: a curve far larger than the view
/// (a tangent arc of a huge radius) would be cut into countless dashes and hang the window.
pub fn dashed(painter: &egui::Painter, pts: &[egui::Pos2], stroke: egui::Stroke, dash: f32, gap: f32) {
    let view = painter.clip_rect().expand(stroke.width + dash);
    let mut run: Vec<egui::Pos2> = Vec::new();
    let flush = |run: &mut Vec<egui::Pos2>| {
        if run.len() > 1 {
            painter.extend(egui::Shape::dashed_line(run, stroke, dash, gap));
        }
        run.clear();
    };
    for w in pts.windows(2) {
        match clip(w[0], w[1], view) {
            Some((a, b)) => {
                if run.last() != Some(&a) {
                    flush(&mut run);
                    run.push(a);
                }
                run.push(b);
            }
            None => flush(&mut run),
        }
    }
    flush(&mut run);
}

/// The part of segment `a`–`b` inside `r` (Liang–Barsky), its ends exactly `a` or `b` where they
/// are inside; none of one with an end beyond the screen's numbers (infinite, NaN).
fn clip(a: egui::Pos2, b: egui::Pos2, r: egui::Rect) -> Option<(egui::Pos2, egui::Pos2)> {
    if !(a.is_finite() && b.is_finite()) {
        return None;
    }
    let d = b - a;
    let (mut t0, mut t1) = (0.0f32, 1.0f32);
    for (p, q) in [(-d.x, a.x - r.min.x), (d.x, r.max.x - a.x), (-d.y, a.y - r.min.y), (d.y, r.max.y - a.y)] {
        if p == 0.0 {
            if q < 0.0 {
                return None;
            }
        } else if p < 0.0 {
            t0 = t0.max(q / p);
        } else {
            t1 = t1.min(q / p);
        }
    }
    let at = |t: f32| {
        if t == 0.0 {
            a
        } else if t == 1.0 {
            b
        } else {
            a + d * t
        }
    };
    (t0 <= t1).then(|| (at(t0), at(t1)))
}

/// The four directions along the sketch's axes, each with its lean: from a centre, a point
/// along X is level with it (horizontal), along Y plumb above or below (vertical).
const AXES: [([f64; 2], Lean); 4] =
    [([1.0, 0.0], Lean::Horizontal), ([-1.0, 0.0], Lean::Horizontal), ([0.0, 1.0], Lean::Vertical), ([0.0, -1.0], Lean::Vertical)];

/// Degrees between two directions.
/// Whether lines along `a` and `b` are parallel, within a line's lean: two lines that look so
/// are dimensioned by their distance, not by an angle of nearly nothing.
pub(crate) fn parallel(a: [f64; 2], b: [f64; 2]) -> bool {
    let d = degrees(a, b);
    d <= LEAN_DEG || d >= 180.0 - LEAN_DEG
}

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
fn tangent_lean(from: [f64; 2], heading: &Heading, to: [f64; 2]) -> Option<(Lean, [f64; 2])> {
    let (c, ccw) = tangent_arc(from, heading, to)?;
    let n = [-heading.dir[1], heading.dir[0]];
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

/// An `edit_sketch` patch being made ([`SketchPatch`]), with the window's snaps.
pub(crate) struct Patch(SketchPatch);

impl std::ops::Deref for Patch {
    type Target = SketchPatch;
    fn deref(&self) -> &SketchPatch {
        &self.0
    }
}

impl std::ops::DerefMut for Patch {
    fn deref_mut(&mut self) -> &mut SketchPatch {
        &mut self.0
    }
}

impl Patch {
    pub(crate) fn new(edit: &SketchEdit) -> Self {
        Self(SketchPatch::new(&edit.json))
    }

    /// An arc's point `p` level with (horizontal) or plumb above (vertical) its centre `c`.
    fn level(&mut self, lean: Option<&Lean>, c: &str, p: &str) {
        match lean {
            Some(Lean::Horizontal) => self.constrain(json!({"type": "horizontal", "points": [c, p]})),
            Some(Lean::Vertical) => self.constrain(json!({"type": "vertical", "points": [c, p]})),
            _ => {}
        }
    }

    /// The point a spot names, made as it needs: an existing point as it is; otherwise a new
    /// point there, on what it snapped to.
    pub(crate) fn point(&mut self, spot: &Spot) -> String {
        if let Snap::Point(n) = &spot.snap {
            return n.clone();
        }
        let p = self.fresh("p");
        self.set("points", &p, json!(spot.at));
        self.tie(&p, spot);
        p
    }

    /// New point `p` held on what `spot` snapped to: a middle, a curve, the points it is in line
    /// with.
    pub(crate) fn tie(&mut self, p: &str, spot: &Spot) {
        match &spot.snap {
            Snap::Track { x, y } => {
                if let Some(x) = x {
                    self.constrain(json!({"type": "vertical", "points": [x, p]}));
                }
                if let Some(y) = y {
                    self.constrain(json!({"type": "horizontal", "points": [y, p]}));
                }
            }
            Snap::Middle(l) => self.constrain(json!({"type": "midpoint", "point": p, "line": l})),
            Snap::On(c) => self.constrain(json!({"type": "on", "point": p, "curve": c})),
            _ => {}
        }
    }

    pub(crate) fn done(self) -> Option<Map<String, Json>> {
        self.0.done()
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
pub(crate) mod tests {
    use std::collections::BTreeMap;

    use datum_brep::TopoId;

    use super::*;
    use crate::sketch::Context;

    pub(crate) const H: f64 = 600.0;

    /// A sketch editor on `sketch`, on the top face of a 40 × 30 × 8 plate if `on_plate`.
    pub(crate) fn editor(sketch: Json, on_plate: bool) -> SketchEdit {
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
            // A plane whose projected origin lies far from what the tests draw, so it snaps to nothing.
            json["plane"] = json!({"origin": [-1000, -1000, 0], "x_axis": [1, 0, 0], "normal": [0, 0, 1]});
            input
        };
        let mut e = SketchEdit::new(
            "s".into(),
            Context {
                id,
                inputs: input.clone().map(|v| ("plate".to_string(), v)).into_iter().collect(),
                sources: input.map(|v| ("plate".to_string(), v)).into_iter().collect(),
            },
            json,
            BTreeMap::new(),
            None,
        );
        e.camera.center = [10.0, 10.0];
        e.camera.half_height = 30.0;
        e
    }

    /// Moves the cursor to sketch point `q` and clicks: the patch, applied.
    pub(crate) fn click_at(e: &mut SketchEdit, q: [f64; 2], free: bool) -> Option<Json> {
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

    /// Moves the cursor to sketch point `q` without clicking: where it would click.
    pub(crate) fn hover_at(e: &mut SketchEdit, q: [f64; 2], free: bool) -> Spot {
        let (x, y) = e.camera.offset(q, H);
        let mut d = std::mem::take(&mut e.draft);
        d.track(e, x, y, H, free);
        e.draft = d;
        e.draft.cursor.clone().unwrap()
    }

    /// A line drawn obliquely, its end a few pixels off the X of a remote point, goes onto that X
    /// and is tied plumb under the point.
    #[test]
    fn a_line_end_tracks_the_x_of_a_point() {
        let mut e = editor(json!({"points": {"r": [15, 25]}}), false);
        e.draft.tool = Tool::Line;
        click_at(&mut e, [0.0, 0.0], false);
        let spot = hover_at(&mut e, [15.4, 5.0], false);
        assert_eq!(spot.snap, Snap::Track { x: Some("r".into()), y: None });
        assert_eq!(spot.at[0], 15.0);
        // Farther than TRACK pixels (6 px = 0.6 units): free.
        assert_eq!(hover_at(&mut e, [14.0, 5.0], false).snap, Snap::Free);
        let p = click_at(&mut e, [15.4, 5.0], false).unwrap();
        assert_eq!(p["points"]["p2"][0].as_f64(), Some(15.0), "{p}");
        assert_eq!(p["add_constraints"], json!([{"type": "vertical", "points": ["r", "p2"]}]), "{p}");
    }

    /// A line leaning parallel to another slides along that direction to where it meets a
    /// point's Y: the lean stays, the tie is added (as a tangent line leaving an arc, D209).
    #[test]
    fn a_leaning_line_tracks_along_its_direction() {
        let mut e = editor(json!({"points": {"g0": [0, 0], "g1": [10, 5], "r": [0, 20]}, "lines": {"g": {"from": "g0", "to": "g1"}}}), false);
        e.draft.tool = Tool::Line;
        click_at(&mut e, [50.0, 0.0], false);
        let spot = hover_at(&mut e, [90.2, 20.15], false);
        assert_eq!(spot.snap, Snap::Track { x: None, y: Some("r".into()) });
        assert!((spot.at[0] - 90.0).abs() < 1e-9 && (spot.at[1] - 20.0).abs() < 1e-9, "{:?}", spot.at);
        assert_eq!(e.draft.lean, Some(Lean::Parallel("g".into())));
        let p = click_at(&mut e, [90.2, 20.15], false).unwrap();
        let ties = p["add_constraints"].as_array().unwrap();
        assert!(ties.contains(&json!({"type": "horizontal", "points": ["r", "p2"]})), "{p}");
        assert!(ties.iter().any(|c| c["type"] == "parallel"), "{p}");
    }

    /// In line with the X of one point and the Y of another: both ties.
    #[test]
    fn a_line_end_tracks_x_and_y_of_two_points() {
        let mut e = editor(json!({"points": {"r1": [15, 25], "r2": [30, 6]}}), false);
        e.draft.tool = Tool::Line;
        click_at(&mut e, [0.0, 0.0], false);
        let spot = hover_at(&mut e, [15.3, 6.2], false);
        assert_eq!(spot.snap, Snap::Track { x: Some("r1".into()), y: Some("r2".into()) });
        assert_eq!(spot.at, [15.0, 6.0]);
        let p = click_at(&mut e, [15.3, 6.2], false).unwrap();
        assert_eq!(
            p["add_constraints"],
            json!([{"type": "vertical", "points": ["r1", "p2"]}, {"type": "horizontal", "points": ["r2", "p2"]}]),
            "{p}"
        );
    }

    /// A line leaning horizontal tracks the X of a point only, and stays horizontal.
    #[test]
    fn a_horizontal_line_tracks_only_x() {
        let mut e = editor(json!({"points": {"r": [15, 3]}}), false);
        e.draft.tool = Tool::Line;
        click_at(&mut e, [0.0, 0.0], false);
        let spot = hover_at(&mut e, [15.3, 0.2], false);
        assert!(matches!(e.draft.lean, Some(Lean::Horizontal)));
        // Y stays with the lean.
        assert_eq!(spot.snap, Snap::Track { x: Some("r".into()), y: None });
        assert_eq!(spot.at, [15.0, 0.0]);
    }

    /// Shift: no tracking.
    #[test]
    fn shift_does_not_track() {
        let mut e = editor(json!({"points": {"r": [15, 25]}}), false);
        e.draft.tool = Tool::Line;
        click_at(&mut e, [0.0, 0.0], false);
        let spot = hover_at(&mut e, [15.4, 5.0], true);
        assert_eq!(spot.snap, Snap::Free);
        assert_eq!(spot.at, [15.4, 5.0]);
    }

    /// The point tool tracks the origin's X.
    #[test]
    fn a_point_tracks_the_origin() {
        let mut e = editor(json!({"points": {}}), false);
        e.json.as_object_mut().unwrap().remove("plane");
        e.solve();
        e.draft.tool = Tool::Point;
        let spot = hover_at(&mut e, [0.3, 5.0], false);
        assert_eq!(spot.snap, Snap::Track { x: Some("origin".into()), y: None });
        assert_eq!(spot.at[0], 0.0);
        let p = click_at(&mut e, [0.3, 5.0], false).unwrap();
        assert_eq!(p["add_constraints"], json!([{"type": "vertical", "points": ["origin", "p1"]}]), "{p}");
    }

    /// Inferring off (F7): a line goes where it is clicked and takes no constraint.
    #[test]
    fn inferring_off_constrains_nothing() {
        let mut e = editor(json!({"points": {}}), false);
        e.modes.infer = false;
        e.draft.tool = Tool::Line;
        click_at(&mut e, [0.0, 0.0], false).unwrap();
        let p = click_at(&mut e, [10.0, 0.3], false).unwrap();
        let y = p["points"]["p2"][1].as_f64().unwrap();
        assert!((y - 0.3).abs() < 1e-9, "not levelled: {p}");
        assert!(p.get("add_constraints").is_none(), "{p}");
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

    /// New names go past the highest taken, never into a gap a removal left: what referred to
    /// the removed line `l2` downstream must not latch onto a new one.
    #[test]
    fn freed_names_are_not_given_again() {
        let e = editor(
            json!({"points": {"p1": [0, 0], "p3": [5, 5], "q7": [1, 1]}, "lines": {"l1": {"from": "p1", "to": "p3"}, "l3": {"from": "p3", "to": "p1"}}}),
            false,
        );
        let mut patch = Patch::new(&e);
        assert_eq!([patch.fresh("p"), patch.fresh("p"), patch.fresh("l"), patch.fresh("a")], ["p4", "p5", "l4", "a1"]);
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
            e.draft.paint(&e, &painter, rect, &I18n::default());
        });
        h.run();
        if let Some(path) = shot {
            h.render().unwrap().save(path).unwrap();
        }
    }

    /// A line started by the model's origin starts at it: the sketch is tied to the origin.
    #[test]
    fn lines_start_at_the_origin() {
        let mut e = editor(json!({"points": {}}), false);
        e.json.as_object_mut().unwrap().remove("plane");
        e.solve();
        e.draft.tool = Tool::Line;
        assert!(click_at(&mut e, [0.2, -0.1], false).is_none(), "the origin is a point already");
        assert_eq!(e.draft.chain.as_ref().unwrap().point, "origin");
        let p = click_at(&mut e, [10.0, 0.2], false).unwrap();
        assert_eq!(p["lines"]["l1"], json!({"from": "origin", "to": "p1"}), "{p}");
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
        e.draft.again();
        assert_eq!(e.draft.tool, Tool::Line, "Enter takes it up again");
        e.draft.press(Tool::Arc);
        e.draft.escape();
        e.draft.again();
        assert_eq!(e.draft.tool, Tool::Arc, "the last one");
    }

    /// A tool's button again ends the command, as Esc does; another button changes the tool.
    #[test]
    fn a_button_again_ends_the_command() {
        let mut e = editor(json!({"points": {}}), false);
        e.draft.press(Tool::Line);
        click_at(&mut e, [0.0, 0.0], false);
        e.draft.press(Tool::Arc);
        assert!(e.draft.tool == Tool::Arc && e.draft.chain.is_none(), "another tool, the chain dropped");
        e.draft.press(Tool::Arc);
        assert!(e.draft.tool == Tool::Select && e.draft.cursor.is_none(), "the same again: ended");
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

    /// A length typed while drawing: the line ends that far toward the cursor, with that
    /// dimension; a diameter typed goes into the circle as typed, an expression too.
    #[test]
    fn typed_sizes() {
        let mut e = editor(json!({"points": {}}), false);
        e.draft.tool = Tool::Line;
        click_at(&mut e, [0.0, 0.0], true);
        assert!(e.draft.takes_value());
        e.draft.typed = Some((json!(25.0), 25.0));
        let p = click_at(&mut e, [3.0, 4.0], true).unwrap();
        assert_eq!(p["points"]["p2"], json!([15.0, 20.0]), "{p}");
        assert_eq!(p["add_constraints"], json!([{"type": "length", "line": "l1", "value": 25.0}]));
        assert!(e.draft.typed.is_none());
        e.draft.escape();
        e.draft.tool = Tool::Circle;
        click_at(&mut e, [30.0, 0.0], true);
        e.draft.typed = Some((json!("w / 2"), 12.5));
        let p = click_at(&mut e, [31.0, 0.0], true).unwrap();
        assert_eq!(p["circles"]["c1"]["diameter"], json!("w / 2"), "{p}");
    }

    /// After an arc, a line leaving along it is tangent to it, before anything else.
    #[test]
    fn a_line_leaves_an_arc_along_it() {
        let mut e = editor(json!({"points": {}}), false);
        e.draft.tool = Tool::Line;
        click_at(&mut e, [0.0, 0.0], false);
        click_at(&mut e, [10.0, 0.0], false);
        e.draft.tangent = true;
        // A quarter turn would snap; an eighth goes free: the arc ends heading up and right.
        click_at(&mut e, [12.93, 0.0 + 7.07], true);
        let dir = e.draft.chain.as_ref().unwrap().last.as_ref().unwrap().dir;
        let p = click_at(&mut e, [12.93 + dir[0] * 10.0 + 0.2, 7.07 + dir[1] * 10.0], false).unwrap();
        assert_eq!(p["add_constraints"], json!([{"type": "tangent", "curves": ["a1", "l2"]}]), "{p}");
        assert!(e.placed.is_ok(), "{:?}", e.placed.as_ref().err());
    }

    /// Of parallel lines, a line leaning perpendicular takes the one the chain is drawn from.
    #[test]
    fn a_line_leans_to_the_line_it_is_drawn_from() {
        // `far` is parallel to the chain's line, and first by name.
        let mut e = editor(json!({"points": {"p": [-20, 0], "q": [-15, 10]}, "lines": {"far": {"from": "p", "to": "q"}}}), false);
        e.draft.tool = Tool::Line;
        click_at(&mut e, [0.0, 0.0], false);
        click_at(&mut e, [5.0, 10.0], true);
        // Perpendicular to both: (5, 10) + 10 · (0.894, −0.447).
        let p = click_at(&mut e, [13.94, 5.53], false).unwrap();
        assert_eq!(p["add_constraints"], json!([{"type": "perpendicular", "lines": ["l1", "l2"]}]), "{p}");
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

    /// The face's corner and edges are no snaps of a drawing tool (no projection behind the
    /// engineer's back); the projection tool projects them, named by the feature (D214).
    #[test]
    fn the_face_is_projected_by_the_projection_tool() {
        let mut e = editor(json!({"points": {}}), true);
        e.draft.tool = Tool::Point;
        let p = click_at(&mut e, [40.2, 0.1], false).unwrap();
        assert!(p.get("project").is_none() && p["points"].get("p1").is_some(), "{p}");
        e.draft.tool = Tool::Project;
        // The top's corners, edges and face: the bottom's fall on them, the upright edges are seen
        // end on, the sides edge on.
        assert_eq!(e.draft.projectable.len(), 4 + 4 + 1);
        let p = click_at(&mut e, [40.2, 0.1], false).unwrap();
        let v = &p["project"]["v1"];
        assert_eq!(v["from"], "plate", "{p}");
        let p = click_at(&mut e, [20.0, 30.3], false).unwrap();
        assert!(p["project"]["e1"]["id"].is_string(), "{p}");
        let placed = e.placed.as_ref().unwrap();
        assert!(placed.lines["e1"].construction && placed.solved.points.contains_key("v1"));
        // Projected now, the corner is offered no more (nor the bottom one under it): the click
        // lands on an edge beside it.
        let p = click_at(&mut e, [40.2, 0.1], false).unwrap();
        assert!(p["project"].get("v2").is_none() && p["project"]["e2"]["id"].is_string(), "{p} {v}");
        // Q puts a projected edge into the profile, and back.
        e.selected = vec![crate::sketch::Pick::Projection("e1".into())];
        let p = e.toggle_construction().unwrap();
        assert_eq!(p["project"]["e1"]["construction"], json!(false), "{p}");
        assert!(!e.placed.as_ref().unwrap().lines["e1"].construction);
        let p = e.toggle_construction().unwrap();
        assert!(p["project"]["e1"].get("construction").is_none(), "{p}");
    }

    /// Inside the face, away from its edges, the projection tool projects the face; Project cut
    /// edges takes the section of a body the plane crosses (D216).
    #[test]
    fn faces_and_sections_are_projected() {
        let mut e = editor(json!({"points": {}}), true);
        e.draft.tool = Tool::Project;
        let p = click_at(&mut e, [20.0, 15.0], false).unwrap();
        assert!(p["project"]["f1"]["face"].is_string(), "{p}");
        let placed = e.placed.as_ref().unwrap();
        assert_eq!(placed.lines.values().filter(|l| !l.construction).count(), 4);
        // The sketch's plane at z = 0 through a block from z = -4 to 4.
        let mut e = editor(json!({"points": {}}), false);
        let block = datum_brep::cuboid(
            TopoId::derive("feature").tag("block").finish(),
            datum_math::Vec3::new(0.0, 0.0, -4.0),
            datum_math::Vec3::new(40.0, 30.0, 4.0),
        )
        .unwrap();
        e.context.sources = vec![("block".into(), std::sync::Arc::new(datum_graph::Value::Body(block)))];
        let p = e.project_sections().unwrap().unwrap();
        assert_eq!(p["project"]["cut1"], json!({"from": "block", "section": true}), "{p}");
        assert_eq!(e.placed.as_ref().unwrap().lines.values().filter(|l| !l.construction).count(), 4);
        // Done once: nothing more to project.
        assert!(e.project_sections().unwrap().is_none());
    }

    /// On the blank (D222), P offers the blank's lines by name, the outline no more once it is
    /// projected whole, and projects one as `blank`, from no input.
    #[test]
    fn the_projection_tool_projects_lines_of_the_blank() {
        let op = |j: Json| serde_json::from_value::<datum_graph::Op>(j).unwrap();
        let feature = |n: &str| TopoId::derive("feature").tag(n).finish();
        let plate = datum_brep::cuboid(feature("plate"), datum_math::Vec3::ZERO, datum_math::Vec3::new(40.0, 30.0, 2.0)).unwrap();
        let top = plate.faces().iter().find(|f| f.surface.plane().is_some_and(|q| q.normal.z > 0.0)).unwrap().id;
        let plate = datum_graph::Value::Body(plate);
        let hole = op(json!({"op": "sketch", "face": format!("{top:?}"), "sketch": {"points": {"c": [20, 15]}, "circles": {"h": {"center": "c", "diameter": 6}}}}))
            .eval(feature("hole"), &[&plate])
            .unwrap();
        let cut = op(json!({"op": "extrude", "mode": "cut", "through": true, "wall": true})).eval(feature("cut"), &[&plate, &hole]).unwrap();
        let blank = datum_ops::unfold_side(cut.body().unwrap(), false, 0.5).unwrap();
        let json = json!({"name": "s", "op": "sketch", "inputs": ["cut"], "flat": "front", "k_factor": 0.5,
            "project": {"blank": {"blank": "outline"}}, "sketch": {"points": {}}});
        let cut = std::sync::Arc::new(cut);
        let context = Context { id: feature("s"), inputs: vec![("cut".into(), cut.clone())], sources: vec![("cut".into(), cut)] };
        let mut e = SketchEdit::new("s".into(), context, json, BTreeMap::new(), None);
        e.draft.tool = Tool::Project;
        // The outline is in already: the hole alone is offered.
        assert_eq!(e.draft.projectable.len(), 1, "{:?}", e.draft.projectable);
        let circle = blank.pieces.iter().position(|p| matches!(p, datum_ops::FlatPiece::Circle { .. })).unwrap();
        assert_eq!(e.draft.projectable[0].entry, ("blank", format!("{:?}", blank.ids[circle])));
        // On the hole's edge (its centre is where the blank has it), a click projects it.
        let datum_ops::FlatPiece::Circle { center, radius } = blank.pieces[circle] else { unreachable!() };
        let p = click_at(&mut e, [center.x + radius, center.y], false).unwrap();
        assert_eq!(p["project"]["e1"], json!({"blank": format!("{:?}", blank.ids[circle])}), "{p}");
        assert!(e.placed.as_ref().unwrap().circles["e1"].construction);
    }

    /// A circle of a sketch on a tilted plane is offered and projected as an ellipse (D218).
    #[test]
    fn a_slanted_circle_is_projected_as_an_ellipse() {
        let mut e = editor(json!({"points": {}}), false);
        let tilted: datum_graph::Op = serde_json::from_value(json!({"op": "sketch",
            "plane": {"origin": [-1000, -1000, 0], "x_axis": [1, 0, 0], "normal": [0, -1, 1]},
            "sketch": {"points": {"o": [1010, 1010]}, "circles": {"ring": {"center": "o", "diameter": 8}},
                       "constraints": [{"type": "fix", "point": "o"}]}}))
        .unwrap();
        let ring = tilted.eval(TopoId::derive("feature").tag("tilted").finish(), &[]).unwrap();
        e.context.sources = vec![("tilted".into(), std::sync::Arc::new(ring))];
        e.draft.projectable_on = None;
        let mut patch = Patch::new(&e);
        patch.set("project", "e1", json!({"from": "tilted", "element": "ring"}));
        e.apply(patch.done().unwrap());
        let c = e.draft.projectable.iter().find(|c| c.entry.1 == "ring").expect("offered");
        assert!(matches!(c.shape, Flat::Polyline(_)), "{c:?}");
        let p = e.placed.as_ref().unwrap();
        assert!(p.ellipses["e1"].construction, "{:?}", p.ellipses);
        assert!(SketchEdit::curve_points(p, "e1").is_some_and(|pts| pts.len() > 2));
    }

    #[test]
    fn a_tangent_arc_turns_the_way_the_cursor_is() {
        // Along +X from the origin, to (5, 5): a quarter turn left around (0, 5).
        let east = Heading { curve: "l".into(), dir: [1.0, 0.0] };
        let (c, ccw) = tangent_arc([0.0, 0.0], &east, [5.0, 5.0]).unwrap();
        assert_eq!((c, ccw), ([0.0, 5.0], true));
        let (c, ccw) = tangent_arc([0.0, 0.0], &east, [5.0, -5.0]).unwrap();
        assert_eq!((c, ccw), ([0.0, -5.0], false));
        assert!(tangent_arc([0.0, 0.0], &east, [7.0, 0.0]).is_none());
    }

    /// Back to the start of the line it continues: no arc, whatever the line's direction.
    /// Rounding alone put the centre up to 3.6·10¹⁸ away, and the window hung drawing it.
    #[test]
    fn no_tangent_arc_straight_along_a_line() {
        for k in 1..400 {
            let a = [0.37 * f64::from(k), -1.3 + 0.11 * f64::from(k)];
            let b = [a[0] + 10.0 * libm::cos(f64::from(k)), a[1] + 7.0 * libm::sin(f64::from(k) * 1.7)];
            let l = libm::hypot(b[0] - a[0], b[1] - a[1]);
            let h = Heading { curve: "l".into(), dir: [(b[0] - a[0]) / l, (b[1] - a[1]) / l] };
            assert!(tangent_arc(b, &h, a).is_none(), "back to the start of line {k}");
        }
    }

    /// A tangent continuation straight back (to the line's start, its middle) is refused, and
    /// straight on is a line, in any direction: the cursor and a middle lie on the line only as
    /// near as rounding goes, and an arc through them was some 10¹⁶ large, the sketch unsolvable.
    #[test]
    fn no_tangent_arc_straight_back_or_on() {
        for k in 1..60 {
            let f = f64::from(k);
            let a = [0.37 * f - 20.0, 0.11 * f - 13.3];
            let b = [a[0] + 10.0 * libm::cos(f), a[1] + 7.0 * libm::sin(1.7 * f)];
            for to in 0..3 {
                let mut e = editor(json!({"points": {}}), false);
                e.draft.tool = Tool::Line;
                click_at(&mut e, a, false);
                click_at(&mut e, b, false);
                // Where the line ended up, leaning or not.
                let Some(Chain { at: b, last: Some(Heading { dir, .. }), .. }) = e.draft.chain.clone() else { panic!("{k}: a chain") };
                let (to, back) = match to {
                    0 => (a, true),
                    1 => ([(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0], true),
                    _ => ([b[0] + 10.0 * dir[0], b[1] + 10.0 * dir[1]], false),
                };
                e.draft.tangent = true;
                let p = click_at(&mut e, to, false);
                assert!(e.placed.is_ok(), "{k} {to:?}: {:?}", e.placed.as_ref().err());
                if back {
                    assert!(p.is_none() && e.draft.tangent && e.draft.chain.is_some(), "{k} back to {to:?}: refused, still tangent");
                } else {
                    let p = p.unwrap();
                    assert!(p["lines"]["l2"].is_object() && p.get("arcs").is_none(), "{k} straight on: a line, {p}");
                }
            }
        }
    }

    /// Dashes only where the view shows them: a curve 10¹⁸ long is drawn at once.
    #[test]
    fn dashes_only_in_view() {
        let r = egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(100.0, 100.0));
        assert_eq!(clip(egui::pos2(10.0, 10.0), egui::pos2(20.0, 30.0), r), Some((egui::pos2(10.0, 10.0), egui::pos2(20.0, 30.0))));
        assert_eq!(clip(egui::pos2(-50.0, 50.0), egui::pos2(150.0, 50.0), r), Some((egui::pos2(0.0, 50.0), egui::pos2(100.0, 50.0))));
        assert_eq!(clip(egui::pos2(-50.0, -1.0), egui::pos2(150.0, -1.0), r), None);
        assert_eq!(clip(egui::pos2(f32::NAN, 5.0), egui::pos2(50.0, 50.0), r), None, "NaN: not drawn, not a panic");
        assert_eq!(clip(egui::pos2(f32::INFINITY, 5.0), egui::pos2(50.0, 50.0), r), None);
        let ctx = egui::Context::default();
        let start = std::time::Instant::now();
        let mut out = ctx.run_ui(egui::RawInput::default(), |ui| {
            let huge = [egui::pos2(-1e18, 50.0), egui::pos2(1e18, 60.0), egui::pos2(1e18, -1e18)];
            dashed(ui.painter(), &huge, egui::Stroke::new(1.6, egui::Color32::WHITE), 6.0, 3.0);
        });
        out.textures_delta.clear();
        assert!(start.elapsed().as_secs() < 5, "{:?}", start.elapsed());
    }
}
