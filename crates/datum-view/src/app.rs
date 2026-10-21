//! The viewer window: the model in 3D, its features and their state, and its parameters to
//! edit. Edits go through the same commands as the CLI and MCP (§6.1) and rebuild in a background
//! thread; the view keeps the last good model meanwhile. The 3D view is the renderer's own
//! picture, drawn again only when the camera, the model or the view's size changes.

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::mpsc::{Receiver, Sender, channel};

use datum_model::Session;
use egui_wgpu::wgpu;
use serde_json::{Value as Json, json};

use crate::camera::Camera;
use crate::draw::Tool;
use crate::render::{BACKGROUND, Frame, Renderer};
use crate::scene::{Geometry, MeshCache, Scene};
use crate::sketch::{Context, SketchEdit};

/// Opens the viewer window, on the model at `path` if given; models open from the window too.
pub fn run(path: Option<PathBuf>) -> Result<(), String> {
    let options = eframe::NativeOptions {
        renderer: eframe::Renderer::Wgpu,
        viewport: egui::ViewportBuilder::default().with_title(title(path.as_deref())).with_inner_size([1280.0, 800.0]).with_drag_and_drop(true),
        ..Default::default()
    };
    eframe::run_native(
        "Datum",
        options,
        Box::new(move |cc| {
            let rs = cc.wgpu_render_state.as_ref().ok_or("the window has no wgpu renderer")?;
            let info = rs.adapter.get_info();
            let adapter = format!("{} ({:?})", info.name, info.backend);
            Ok(Box::new(Viewer::new(path, cc.egui_ctx.clone(), adapter)))
        }),
    )
    .map_err(|e| e.to_string())
}

fn title(path: Option<&std::path::Path>) -> String {
    path.map_or_else(|| "Datum".into(), |p| format!("Datum — {}", p.display()))
}

enum Request {
    Open(PathBuf),
    SetParameter(String, Json),
    /// Any other command of the session, with its parameters.
    Command(&'static str, Json),
    /// The sketch feature edited in the window, if any: its context comes with every model.
    Editing(Option<String>),
}

/// What the background thread hands the window after each rebuild.
struct Model {
    /// `inspect` of the session: features with status, parameters with values.
    inspect: Json,
    scene: Scene,
    /// GPU vertices of the scene, made on the background thread too.
    geometry: Arc<Geometry>,
    /// The last command's error, if it was refused.
    error: Option<String>,
    /// What the commands of this rebuild changed beyond what they were asked: a body's shape.
    notes: Vec<String>,
    /// A model was opened in this rebuild: the view fits it.
    opened: bool,
    /// Milliseconds the rebuild and the scene (meshes, vertices) took.
    rebuild_ms: f64,
    scene_ms: f64,
    /// The feature each body of the scene is of.
    bodies: Vec<String>,
    /// The sketch being edited (left out of the scene: the window draws it) and its context.
    editing: Option<(String, Context)>,
}

/// The session lives on its own thread; requests come in, models go out. Requests that pile up
/// while a rebuild runs are folded: only the latest value of each parameter is applied.
fn worker(requests: Receiver<Request>, models: Sender<Model>, ctx: egui::Context) {
    let mut session = Session::default();
    let mut meshes = MeshCache::default();
    let mut editing: Option<String> = None;
    while let Ok(first) = requests.recv() {
        let mut pending = vec![first];
        pending.extend(requests.try_iter());
        let start = std::time::Instant::now();
        let mut latest: BTreeMap<String, Json> = BTreeMap::new();
        let (mut error, mut opened) = (None, false);
        let mut notes = Vec::new();
        // A body whose shape an edit changed (pieces, through holes): worth a word.
        let mut note = |reply: &Json| {
            for c in reply["shape_changed"].as_array().into_iter().flatten() {
                let says = |s: &Json| format!("{} piece(s), {} through hole(s)", s["pieces"], s["through_holes"]);
                notes.push(format!("{} changed shape: {} → {}", c["feature"].as_str().unwrap_or_default(), says(&c["before"]), says(&c["after"])));
            }
        };
        for r in pending {
            match r {
                Request::Open(p) => {
                    latest.clear();
                    opened = true;
                    if let Err(e) = session.execute("open", &json!({"path": p.display().to_string()})) {
                        error = Some(e);
                    }
                }
                Request::SetParameter(name, value) => {
                    latest.insert(name, value);
                }
                Request::Command(command, params) => match session.execute(command, &params) {
                    Ok(reply) => note(&reply),
                    Err(e) => error = Some(e),
                },
                Request::Editing(name) => editing = name,
            }
        }
        for (name, value) in latest {
            match session.execute("set_parameter", &json!({"name": name, "value": value})) {
                Ok(reply) => note(&reply),
                Err(e) => error = Some(e),
            }
        }
        let rebuild_ms = start.elapsed().as_secs_f64() * 1e3;
        let start = std::time::Instant::now();
        let (graph, evaluation) = (session.graph(), session.evaluation());
        let shown: Vec<(&str, &datum_graph::Value)> = shown(graph, evaluation, editing.as_deref())
            .into_iter()
            .filter_map(|i| Some((graph.nodes()[i as usize].name.as_str(), evaluation.results[i as usize].as_deref().ok()?)))
            .collect();
        let finals: Vec<&datum_graph::Value> = shown.iter().map(|(_, v)| *v).collect();
        let bodies = shown.iter().filter(|(_, v)| v.body().is_some()).map(|(n, _)| n.to_string()).collect();
        let scene = Scene::new(&finals, &mut meshes);
        meshes.retain(&finals.iter().filter_map(|v| v.body()).collect::<Vec<_>>());
        let context = editing.as_ref().and_then(|name| {
            let node = graph.nodes().iter().position(|n| &n.name == name)?;
            let input = graph.nodes()[node].inputs.first().and_then(|&i| evaluation.results[i as usize].as_ref().ok().cloned());
            Some((name.clone(), Context { id: graph.nodes()[node].id, input }))
        });
        let geometry = Arc::new(Geometry::new(&scene, scene.anchor()));
        let scene_ms = start.elapsed().as_secs_f64() * 1e3;
        let inspect = session.execute("inspect", &json!({})).unwrap_or_default();
        if models.send(Model { inspect, scene, geometry, error, notes, opened, rebuild_ms, scene_ms, bodies, editing: context }).is_err() {
            return;
        }
        ctx.request_repaint();
    }
}

/// The features the view shows: what the model ends in, but the sketch being edited (the window
/// draws it) in place of itself shows its input, the body it lies on.
fn shown(graph: &datum_graph::Graph, evaluation: &datum_graph::Evaluation, editing: Option<&str>) -> Vec<datum_graph::NodeIx> {
    let edited = editing.and_then(|n| graph.find(n));
    let mut v: Vec<datum_graph::NodeIx> = evaluation.finals(graph).into_iter().filter(|i| Some(*i) != edited).collect();
    if let Some(e) = edited {
        v.extend(&graph.nodes()[e as usize].inputs);
        v.sort_unstable();
        v.dedup();
    }
    v
}

/// Geometry version, view-projection matrix, eye, and picture width, height and whether smooth.
type Drawn = (u64, [f32; 16], [f32; 3], [u32; 3]);

struct Viewer {
    requests: Sender<Request>,
    models: Receiver<Model>,
    model: Option<Model>,
    busy: bool,
    camera: Camera,
    fitted: bool,
    /// GPU vertices of the current scene, their version, and the anchor they are relative to.
    geometry: Arc<Geometry>,
    version: u64,
    /// Multisampling of the view when the camera is still: smooth edges, at about twice the GPU
    /// time on a weak card.
    smooth: bool,
    /// Made on the first frame.
    renderer: Option<Renderer>,
    /// The picture as an egui texture, and the size it was registered with.
    texture: Option<(egui::TextureId, [u32; 2])>,
    /// What the picture shows: geometry version, camera matrices, size and samples.
    drawn: Option<Drawn>,
    /// Smoothed time between frames, and of the window's own work in a frame, milliseconds.
    frame_ms: f64,
    ui_ms: f64,
    /// The GPU drawing the view and its API: a software one explains a slow frame.
    adapter: String,
    /// Parameter definitions as typed, by name.
    edits: BTreeMap<String, String>,
    /// The sketch being edited, and one asked for that the model has not brought yet.
    sketch: Option<SketchEdit>,
    entering: Option<String>,
    /// Picking a face to sketch on, and the body and face under the cursor.
    picking: bool,
    face: Option<(usize, datum_brep::TopoId)>,
}

impl Viewer {
    fn new(path: Option<PathBuf>, ctx: egui::Context, adapter: String) -> Self {
        let (requests, inbox) = channel();
        let (outbox, models) = channel();
        std::thread::spawn(move || worker(inbox, outbox, ctx));
        let geometry = Arc::new(Geometry::new(&Scene::default(), datum_math::Vec3::ZERO));
        let mut v = Self {
            requests,
            models,
            model: None,
            busy: false,
            camera: Camera::default(),
            fitted: false,
            geometry,
            version: 0,
            smooth: true,
            renderer: None,
            texture: None,
            drawn: None,
            frame_ms: 0.0,
            ui_ms: 0.0,
            adapter,
            edits: BTreeMap::new(),
            sketch: None,
            entering: None,
            picking: false,
            face: None,
        };
        if let Some(p) = path {
            v.open(p);
        }
        v
    }

    fn open(&mut self, path: PathBuf) {
        self.leave();
        self.send(Request::Open(path));
    }

    fn send(&mut self, r: Request) {
        let _ = self.requests.send(r);
        self.busy = true;
    }

    /// Starts editing sketch feature `name`, once the model brings its context.
    fn enter(&mut self, name: String) {
        self.picking = false;
        self.sketch = None;
        self.send(Request::Editing(Some(name.clone())));
        self.entering = Some(name);
    }

    fn leave(&mut self) {
        if self.sketch.is_some() || self.entering.is_some() {
            self.sketch = None;
            self.entering = None;
            let _ = self.requests.send(Request::Editing(None));
        }
    }

    /// A new sketch feature from `fields` (its plane or face), edited at once.
    fn new_sketch(&mut self, mut fields: Json) {
        let taken: Vec<&str> = self
            .model
            .as_ref()
            .map(|m| m.inspect["features"].as_array().into_iter().flatten().filter_map(|f| f["name"].as_str()).collect())
            .unwrap_or_default();
        let name = (1..).map(|k| format!("sketch{k}")).find(|n| !taken.contains(&n.as_str())).unwrap_or_default();
        fields["name"] = json!(name);
        fields["op"] = json!("sketch");
        fields["sketch"] = json!({"points": {}});
        self.send(Request::Command("add", fields));
        self.enter(name);
    }

    /// Brings the sketch being edited up to the model just received, or starts one asked for.
    fn sync_sketch(&mut self) {
        let Some(m) = &self.model else { return };
        let params: BTreeMap<String, f64> = m.inspect["parameters"]
            .as_object()
            .map(|p| p.iter().filter_map(|(n, v)| Some((n.clone(), v["value"].as_f64()?))).collect())
            .unwrap_or_default();
        let feature = |name: &str| m.inspect["features"].as_array().and_then(|f| f.iter().find(|f| f["name"] == name)).cloned();
        let wanted = self.sketch.as_ref().map(|s| s.feature.clone()).or_else(|| self.entering.clone());
        let Some(name) = wanted else { return };
        match (&m.editing, feature(&name)) {
            (Some((n, context)), Some(json)) if *n == name => match &mut self.sketch {
                Some(s) => s.refresh(context.clone(), json, params),
                None => {
                    let scene = m.scene.bounds.map(|(lo, hi)| (m.scene.anchor(), (hi - lo).dot(hi - lo).sqrt() * 0.5));
                    self.sketch = Some(SketchEdit::new(name, context.clone(), json, params, scene));
                    self.entering = None;
                }
            },
            // Still on its way, or gone (undone, or the model replaced).
            (_, Some(_)) if self.sketch.is_none() => {}
            _ => self.leave(),
        }
    }

    /// The toolbar: open a model by dialog; a `.json` dropped on the window opens too.
    fn toolbar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            if ui.button("Open…").clicked()
                && let Some(p) = rfd::FileDialog::new().add_filter("Datum model", &["json"]).pick_file()
            {
                ui.send_viewport_cmd(egui::ViewportCommand::Title(title(Some(&p))));
                self.open(p);
            }
            ui.weak("or drop a model (.json) on the window");
            ui.separator();
            ui.checkbox(&mut self.smooth, "Smooth edges")
                .on_hover_text("Multisampling (MSAA ×4) when the view is still; while it turns, frames are drawn without it.");
            ui.separator();
            if ui.button("↶").on_hover_text("Undo (Ctrl+Z)").clicked() {
                self.send(Request::Command("undo", json!({})));
            }
            if ui.button("↷").on_hover_text("Redo (Ctrl+Y)").clicked() {
                self.send(Request::Command("redo", json!({})));
            }
            ui.separator();
            if ui.button("New sketch").on_hover_text("A sketch on the XY plane").clicked() {
                self.new_sketch(json!({}));
            }
            if ui.selectable_label(self.picking, "Sketch on face").on_hover_text("Then click a plane face of the model").clicked() {
                self.leave();
                self.picking = !self.picking;
            }
        });
        let dropped = ui.input(|i| i.raw.dropped_files.iter().map(|f| f.path().to_path_buf()).find(|p| p.extension().is_some_and(|e| e == "json")));
        if let Some(p) = dropped {
            ui.send_viewport_cmd(egui::ViewportCommand::Title(title(Some(&p))));
            self.open(p);
        }
    }

    fn receive(&mut self) {
        while let Ok(m) = self.models.try_recv() {
            self.busy = false;
            // Fitted when it arrives, not when asked for: until then the view holds the last model.
            self.fitted &= !m.opened;
            self.geometry = m.geometry.clone();
            self.version += 1;
            if let Some(p) = m.inspect["parameters"].as_object() {
                self.edits = p.iter().map(|(n, v)| (n.clone(), definition(&v["definition"]))).collect();
            }
            self.model = Some(m);
            self.sync_sketch();
        }
    }

    fn side(&mut self, ui: &mut egui::Ui) {
        let Some(m) = &self.model else {
            ui.label(if self.busy { "Opening…" } else { "No model open." });
            return;
        };
        if let Some(e) = &m.error {
            ui.colored_label(egui::Color32::from_rgb(190, 40, 40), e);
        }
        for n in &m.notes {
            ui.colored_label(egui::Color32::from_rgb(190, 120, 0), format!("⚠ {n}"));
        }
        let mut done = false;
        if let Some(sk) = &mut self.sketch {
            ui.heading(format!("Sketch {}", sk.feature));
            match &sk.placed {
                Ok(p) if p.solved.dof == 0 => ui.label("Fully defined"),
                Ok(p) => {
                    ui.colored_label(egui::Color32::from_rgb(30, 90, 210), format!("{} degrees of freedom: blue points can still move", p.solved.dof))
                }
                Err(e) => ui.colored_label(egui::Color32::from_rgb(190, 40, 40), e),
            };
            // What the sketch breaks downstream: why the model behind did not change.
            for f in m.inspect["features"].as_array().into_iter().flatten().filter(|f| f["name"] != sk.feature.as_str()) {
                if let (Some(name), Some(e)) = (f["name"].as_str(), f["error"].as_str()) {
                    ui.colored_label(egui::Color32::from_rgb(190, 40, 40), format!("✖ {name}: {e}"));
                }
            }
            ui.horizontal_wrapped(|ui| {
                for (tool, label, hint) in [
                    (Tool::Select, "Select", "Select and drag (Esc)"),
                    (Tool::Line, "Line", "Lines in a chain; T or a drag from its end: a tangent arc; Esc ends (L)"),
                    (Tool::Arc, "Arc", "Centre, start, end (A)"),
                    (Tool::Circle, "Circle", "Centre, then a point on it (C)"),
                    (Tool::Point, "Point", "A point (P)"),
                ] {
                    if ui.selectable_label(sk.draft.tool == tool, label).on_hover_text(hint).clicked() {
                        sk.draft.cancel();
                        sk.draft.tool = tool;
                    }
                }
            });
            ui.horizontal(|ui| {
                ui.checkbox(&mut sk.draft.construction, "Construction");
                ui.checkbox(&mut sk.names, "Names");
                if sk.draft.tool == Tool::Line && sk.draft.chain.as_ref().is_some_and(|c| c.last.is_some()) {
                    ui.checkbox(&mut sk.draft.tangent, "Tangent arc (T)");
                }
            });
            ui.weak(match sk.draft.tool {
                Tool::Select => "Drag a blue point · click to select, Shift to add · Del removes · Esc clears, then leaves",
                _ => "Click to place · snaps to points, middles, curves and the face's edges · Shift: no snaps · Esc ends the command",
            });
            done = ui.button("Done").clicked();
            ui.separator();
        }
        ui.heading("Parameters");
        let params = m.inspect["parameters"].as_object().cloned().unwrap_or_default();
        if params.is_empty() {
            ui.weak("none");
        }
        let mut send = Vec::new();
        egui::Grid::new("parameters").num_columns(3).striped(true).show(ui, |ui| {
            for (name, p) in &params {
                ui.label(name);
                let text = self.edits.entry(name.clone()).or_default();
                // A grid cell is as wide as its column was: sized explicitly, the field widens it.
                let r = ui.add_sized([170.0, ui.spacing().interact_size.y], egui::TextEdit::singleline(text));
                // Applied when the field is left (Enter, Tab or a click elsewhere) changed. The
                // field itself takes the Enter key, so asking for it here would never fire.
                if r.lost_focus() && *text != definition(&p["definition"]) {
                    send.push((name.clone(), text.clone()));
                }
                match (p["value"].as_f64(), p["error"].as_str()) {
                    (Some(v), _) => ui.weak(format!("= {v}")),
                    (_, Some(e)) => ui.colored_label(egui::Color32::from_rgb(190, 40, 40), e),
                    _ => ui.label(""),
                };
                ui.end_row();
            }
        });
        for (name, text) in send {
            // A plain number stays a number; anything else is an expression.
            let value = text.trim().parse::<f64>().map_or_else(|_| json!(text.trim()), |v| json!(v));
            let _ = self.requests.send(Request::SetParameter(name, value));
            self.busy = true;
        }
        ui.separator();
        ui.heading("Features");
        let (mut export, mut edit, mut enter) = (None, None, None);
        egui::ScrollArea::vertical().show(ui, |ui| {
            for f in m.inspect["features"].as_array().into_iter().flatten() {
                let name = f["name"].as_str().unwrap_or_default();
                let op = f["op"].as_str().unwrap_or_default();
                match f["error"].as_str() {
                    // A flat blank: its size, and its DXF for the turret press.
                    None if f["status"]["flat"].is_object() => {
                        let flat = &f["status"]["flat"];
                        ui.horizontal(|ui| {
                            let (w, h) = (flat["width"].as_f64().unwrap_or(0.0), flat["height"].as_f64().unwrap_or(0.0));
                            ui.label(format!("✔ {name}  ·  {op}  ·  {w:.1} × {h:.1}"));
                            if ui.button("DXF…").clicked()
                                && let Some(p) = rfd::FileDialog::new().add_filter("DXF", &["dxf"]).set_file_name(format!("{name}.dxf")).save_file()
                            {
                                export = Some((name.to_string(), p));
                            }
                        });
                    }
                    // Sheet metal: the side of the path its thickness lies on, flipped by a click.
                    None if op == "sheet" => {
                        ui.horizontal(|ui| {
                            ui.label(format!("✔ {name}  ·  {op}"));
                            let side = serde_json::from_value::<datum_ops::Side>(f["side"].clone()).unwrap_or(datum_ops::Side::Left);
                            let label = serde_json::to_value(side).ok().and_then(|v| v.as_str().map(str::to_string)).unwrap_or_default();
                            if ui.button(format!("⇄ {label}")).on_hover_text("Thickness side of the path: left, right, middle").clicked() {
                                edit = Some((name.to_string(), json!({"side": side.next()})));
                            }
                        });
                    }
                    None if op == "sketch" => {
                        ui.horizontal(|ui| {
                            ui.label(format!("✔ {name}  ·  {op}"));
                            if ui.button("✎").on_hover_text("Edit the sketch").clicked() {
                                enter = Some(name.to_string());
                            }
                        });
                    }
                    None => {
                        ui.label(format!("✔ {name}  ·  {op}"));
                    }
                    Some(e) => {
                        ui.colored_label(egui::Color32::from_rgb(190, 40, 40), format!("✖ {name}  ·  {op}"));
                        ui.add(egui::Label::new(egui::RichText::new(e).small()).wrap());
                    }
                }
            }
        });
        if let Some((name, mut fields)) = edit {
            fields["name"] = json!(name);
            self.send(Request::Command("edit", fields));
        }
        if let Some((name, path)) = export {
            self.send(Request::Command("export_dxf", json!({"name": name, "path": path.display().to_string()})));
        }
        if let Some(name) = enter {
            self.enter(name);
        }
        if done {
            self.leave();
        }
    }

    fn viewport(&mut self, ui: &mut egui::Ui, rs: &egui_wgpu::RenderState) {
        let (rect, r) = ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());
        let aspect = f64::from(rect.width() / rect.height().max(1.0));
        let offset = r.hover_pos().or(r.interact_pointer_pos()).map(|p| (f64::from(p.x - rect.center().x), f64::from(p.y - rect.center().y)));
        if self.sketch.is_some() {
            self.sketch_input(ui, rect, &r, offset);
        } else {
            let d = r.drag_delta();
            if r.dragged_by(egui::PointerButton::Primary) {
                self.camera.orbit(f64::from(d.x), f64::from(d.y));
            } else if r.dragged_by(egui::PointerButton::Secondary) || r.dragged_by(egui::PointerButton::Middle) {
                self.camera.pan(f64::from(d.x), f64::from(d.y), f64::from(rect.height()));
            }
            if r.hovered() {
                let scroll = ui.input(|i| i.smooth_scroll_delta.y);
                if scroll != 0.0 {
                    self.camera.zoom(libm::exp(-f64::from(scroll) * 0.002));
                }
            }
        }
        // Picking a face to sketch on: the one under the cursor, and a click starts the sketch.
        let mut on_face = None;
        if self.picking
            && let Some(m) = &self.model
        {
            let half = (f64::from(rect.width()) * 0.5, f64::from(rect.height()) * 0.5);
            self.face = offset.and_then(|(x, y)| {
                let (origin, dir) = self.camera.ray(x / half.0, -y / half.1, aspect);
                m.scene.pick(origin, dir)
            });
            if r.clicked()
                && let Some((b, id)) = self.face
            {
                on_face = Some(json!({"inputs": [m.bodies[b]], "face": format!("{id:?}")}));
            }
        }
        if let Some(fields) = on_face {
            self.new_sketch(fields);
        }
        let Some(m) = &self.model else { return };
        let scene = m.scene.bounds.map(|(lo, hi)| (m.scene.anchor(), (hi - lo).dot(hi - lo).sqrt() * 0.5));
        // A model just opened is fitted once, where the view's shape is known.
        if r.double_clicked() || (!self.fitted && m.scene.bounds.is_some()) {
            match &mut self.sketch {
                Some(sk) if r.double_clicked() => sk.fit(scene),
                _ => self.camera.fit(m.scene.bounds, aspect),
            }
            self.fitted = true;
        }
        let anchor = m.scene.anchor();
        let radius = scene.map_or(1.0, |s| s.1);
        let (view_projection, eye) = match &self.sketch {
            Some(sk) => sk.camera.matrices(anchor, aspect, radius),
            None => self.camera.matrices(anchor, aspect, radius),
        };
        let ppp = ui.ctx().pixels_per_point();
        let [w, h] = [rect.width(), rect.height()].map(|x| (x * ppp).round() as u32);
        if w == 0 || h == 0 {
            return;
        }
        // While the camera moves, frames are drawn without multisampling, twice as fast on a weak
        // GPU; once it stops, the view is drawn smooth again.
        let moving = self.drawn.is_some_and(|d| (d.1, d.2) != (view_projection, eye));
        let smooth_now = self.smooth && !moving;
        let drawn = (self.version, view_projection, eye, [w, h, u32::from(smooth_now)]);
        if self.drawn != Some(drawn) {
            let renderer = self.renderer.get_or_insert_with(|| Renderer::new(&rs.device));
            let frame = Frame { version: self.version, geometry: self.geometry.clone(), view_projection, eye };
            let encoder = renderer.draw(&rs.device, &rs.queue, &frame, w, h, smooth_now);
            // Submitted before egui's own commands of this frame, which show the picture.
            rs.queue.submit([encoder.finish()]);
            if self.smooth && !smooth_now {
                ui.ctx().request_repaint();
            }
            if let Some(picture) = renderer.picture()
                && self.texture.is_none_or(|t| t.1 != [w, h])
            {
                let view = picture.create_view(&Default::default());
                let mut egui_renderer = rs.renderer.write();
                let id = match self.texture {
                    Some((id, _)) => {
                        egui_renderer.update_egui_texture_from_wgpu_texture(&rs.device, &view, wgpu::FilterMode::Nearest, id);
                        id
                    }
                    None => egui_renderer.register_native_texture(&rs.device, &view, wgpu::FilterMode::Nearest),
                };
                self.texture = Some((id, [w, h]));
            }
            self.drawn = Some(drawn);
        }
        if let Some((id, _)) = self.texture {
            ui.painter().image(id, rect, egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0)), egui::Color32::WHITE);
        }
        let painter = ui.painter().with_clip_rect(rect);
        if let Some(sk) = &self.sketch {
            sk.paint(&painter, rect);
            sk.draft.paint(sk, &painter, rect);
        }
        // The face a click would sketch on, tinted.
        if let (true, Some((b, id))) = (self.picking, self.face) {
            let mesh = &m.scene.bodies[b];
            let mut tint = egui::Mesh::default();
            for t in mesh.triangles.iter().zip(&mesh.faces).filter(|(_, f)| **f == id).map(|(t, _)| t) {
                let corners: Option<Vec<egui::Pos2>> = t.iter().map(|&q| on_screen(&view_projection, q - anchor, rect)).collect();
                if let Some(c) = corners {
                    let base = tint.vertices.len() as u32;
                    for p in c {
                        tint.colored_vertex(p, egui::Color32::from_rgba_unmultiplied(235, 110, 20, 90));
                    }
                    tint.add_triangle(base, base + 1, base + 2);
                }
            }
            painter.add(egui::Shape::mesh(tint));
        }
        // What the view costs, to tell a slow frame from a slow rebuild.
        let dt = f64::from(ui.input(|i| i.unstable_dt)) * 1e3;
        self.frame_ms = smooth(self.frame_ms, dt);
        let (tris, lines) = (m.geometry.triangles.indices.len() / 3, m.geometry.lines.indices.len() / 2);
        let status = format!(
            "{}frame {:.1} ms (ui {:.1}) · rebuild {:.0} ms · scene {:.0} ms · {tris} triangles · {lines} edges\n{}",
            if self.busy { "rebuilding…  " } else { "" },
            self.frame_ms,
            self.ui_ms,
            m.rebuild_ms,
            m.scene_ms,
            self.adapter
        );
        ui.painter().text(
            rect.left_top() + egui::vec2(10.0, 10.0),
            egui::Align2::LEFT_TOP,
            status,
            egui::FontId::monospace(12.0),
            egui::Color32::DARK_GRAY,
        );
    }
}

/// Where point `q` (relative to the anchor) shows in `rect` through view-projection `m`, if in
/// front of the eye.
fn on_screen(m: &[f32; 16], q: datum_math::Vec3, rect: egui::Rect) -> Option<egui::Pos2> {
    let v = [q.x as f32, q.y as f32, q.z as f32, 1.0];
    let row = |r: usize| (0..4).map(|c| m[c * 4 + r] * v[c]).sum::<f32>();
    let w = row(3);
    (w > 0.0).then(|| rect.center() + egui::vec2(row(0) / w * rect.width() * 0.5, -row(1) / w * rect.height() * 0.5))
}

impl Viewer {
    /// The mouse and keys while a sketch is edited: hover, select, drag points, pan, zoom,
    /// delete, Esc.
    fn sketch_input(&mut self, ui: &egui::Ui, rect: egui::Rect, r: &egui::Response, offset: Option<(f64, f64)>) {
        let Some(sk) = &mut self.sketch else { return };
        let h = f64::from(rect.height());
        let mut sent = Vec::new();
        let mut leave = false;
        let typing = ui.ctx().egui_wants_keyboard_input();
        if !typing {
            let tool = ui.input(|i| {
                [(egui::Key::L, Tool::Line), (egui::Key::A, Tool::Arc), (egui::Key::C, Tool::Circle), (egui::Key::P, Tool::Point)]
                    .into_iter()
                    .find(|(k, _)| i.key_pressed(*k) && i.modifiers.is_none())
                    .map(|(_, t)| t)
            });
            if let Some(t) = tool {
                sk.draft.cancel();
                sk.draft.tool = t;
            }
            if ui.input(|i| i.key_pressed(egui::Key::T)) && sk.draft.chain.as_ref().is_some_and(|c| c.last.is_some()) {
                sk.draft.tangent = !sk.draft.tangent;
            }
        }
        if sk.draft.tool != Tool::Select {
            let free = ui.input(|i| i.modifiers.shift);
            let mut draft = std::mem::take(&mut sk.draft);
            // As in Inventor: a drag from the chain's end draws a tangent arc.
            if r.drag_started_by(egui::PointerButton::Primary)
                && let (Some(c), Some(o)) = (&draft.chain, ui.input(|i| i.pointer.press_origin()))
                && c.last.is_some()
            {
                let (x, y) = sk.camera.offset(c.at, h);
                let at = rect.center() + egui::vec2(x as f32, y as f32);
                draft.tangent |= o.distance(at) <= 10.0;
            }
            if let Some((x, y)) = offset {
                draft.track(sk, x, y, h, free);
            }
            if (r.clicked() || (draft.tangent && r.drag_stopped_by(egui::PointerButton::Primary)))
                && let Some(patch) = draft.click(sk)
            {
                sent.push(Request::Command("edit_sketch", sk.apply(patch)));
            }
            if r.dragged_by(egui::PointerButton::Secondary) || r.dragged_by(egui::PointerButton::Middle) {
                let d = r.drag_delta();
                sk.camera.pan(f64::from(d.x), f64::from(d.y), h);
            }
            if !typing && ui.input(|i| i.key_pressed(egui::Key::Escape)) {
                draft.escape();
            }
            sk.draft = draft;
            let scroll = if r.hovered() { ui.input(|i| i.smooth_scroll_delta.y) } else { 0.0 };
            if scroll != 0.0 {
                let (x, y) = offset.unwrap_or((0.0, 0.0));
                sk.camera.zoom_at(libm::exp(-f64::from(scroll) * 0.002), x, y, h);
            }
            for r in sent {
                self.send(r);
            }
            return;
        }
        if sk.drag.is_none() {
            let painter = ui.painter();
            sk.hover = offset.and_then(|(x, y)| sk.hit(x, y, h, |t| crate::sketch::label_size(painter, t)));
        }
        // A drag starts a few pixels from where the button went down: what is there is dragged.
        if r.drag_started_by(egui::PointerButton::Primary) {
            if let Some(o) = ui.input(|i| i.pointer.press_origin()) {
                let (x, y) = (f64::from(o.x - rect.center().x), f64::from(o.y - rect.center().y));
                sk.hover = sk.hit(x, y, h, |t| crate::sketch::label_size(ui.painter(), t));
            }
            sk.start_drag();
        }
        if sk.drag.is_some() {
            if let Some((x, y)) = offset {
                sk.drag_to(sk.camera.at(x, y, h));
            }
            if r.drag_stopped()
                && let Some(p) = sk.end_drag()
            {
                sent.push(Request::Command("edit_sketch", p));
            }
        } else if r.dragged_by(egui::PointerButton::Secondary) || r.dragged_by(egui::PointerButton::Middle) {
            let d = r.drag_delta();
            sk.camera.pan(f64::from(d.x), f64::from(d.y), h);
        }
        if r.clicked() {
            let shift = ui.input(|i| i.modifiers.shift);
            match sk.hover.clone() {
                Some(p) if shift => {
                    if let Some(k) = sk.selected.iter().position(|q| *q == p) {
                        sk.selected.remove(k);
                    } else {
                        sk.selected.push(p);
                    }
                }
                Some(p) => sk.selected = vec![p],
                None if !shift => sk.selected.clear(),
                None => {}
            }
        }
        let scroll = if r.hovered() { ui.input(|i| i.smooth_scroll_delta.y) } else { 0.0 };
        if scroll != 0.0 {
            let (x, y) = offset.unwrap_or((0.0, 0.0));
            sk.camera.zoom_at(libm::exp(-f64::from(scroll) * 0.002), x, y, h);
        }
        if !ui.ctx().egui_wants_keyboard_input() {
            if ui.input(|i| i.key_pressed(egui::Key::Delete) || i.key_pressed(egui::Key::Backspace))
                && let Some(p) = sk.delete()
            {
                sent.push(Request::Command("edit_sketch", p));
                sk.selected.clear();
            }
            if ui.input(|i| i.key_pressed(egui::Key::Escape)) {
                if sk.selected.is_empty() {
                    leave = true;
                } else {
                    sk.selected.clear();
                }
            }
        }
        for r in sent {
            self.send(r);
        }
        if leave {
            self.leave();
        }
    }
}

/// A running average of frame times, steady enough to read.
fn smooth(average: f64, sample: f64) -> f64 {
    if average == 0.0 { sample } else { average * 0.9 + sample * 0.1 }
}

/// How a parameter's definition shows in its field: a number (whole ones without `.0`), an
/// expression as is.
fn definition(v: &Json) -> String {
    match (v.as_str(), v.as_f64()) {
        (Some(e), _) => e.to_string(),
        (None, Some(x)) if x.fract() == 0.0 && x.abs() < 1e15 => format!("{}", x as i64),
        _ => v.to_string(),
    }
}

impl eframe::App for Viewer {
    fn ui(&mut self, ui: &mut egui::Ui, frame: &mut eframe::Frame) {
        let start = std::time::Instant::now();
        self.receive();
        // Undo and redo, unless a text field takes the keys.
        if !ui.ctx().egui_wants_keyboard_input() {
            let shortcut = |k| egui::KeyboardShortcut::new(egui::Modifiers::COMMAND, k);
            let (undo, redo) = ui.input_mut(|i| (i.consume_shortcut(&shortcut(egui::Key::Z)), i.consume_shortcut(&shortcut(egui::Key::Y))));
            if undo {
                self.send(Request::Command("undo", json!({})));
            }
            if redo {
                self.send(Request::Command("redo", json!({})));
            }
        }
        egui::Panel::top("datum_toolbar").show(ui, |ui| self.toolbar(ui));
        egui::Panel::left("datum_side").min_size(340.0).show(ui, |ui| self.side(ui));
        let [r, g, b] = BACKGROUND.map(|c| (c * 255.0).round() as u8);
        egui::CentralPanel::default().frame(egui::Frame::NONE.fill(egui::Color32::from_rgb(r, g, b))).show(ui, |ui| {
            if let Some(rs) = frame.wgpu_render_state() {
                self.viewport(ui, rs);
            }
        });
        self.ui_ms = smooth(self.ui_ms, start.elapsed().as_secs_f64() * 1e3);
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use egui_kittest::kittest::Queryable;

    use super::*;

    /// A viewer showing reference model `model`, and the requests it sends.
    fn viewer(model: &str) -> (Viewer, std::sync::mpsc::Receiver<Request>) {
        let mut session = Session::default();
        let model = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/models").join(model);
        session.execute("open", &json!({"path": model.display().to_string()})).unwrap();
        let inspect = session.execute("inspect", &json!({})).unwrap();
        let (requests, inbox) = channel();
        let (_outbox, models) = channel();
        let edits = inspect["parameters"].as_object().unwrap().iter().map(|(n, v)| (n.clone(), definition(&v["definition"]))).collect();
        let v = Viewer {
            requests,
            models,
            model: Some(Model {
                inspect,
                scene: Scene::default(),
                geometry: Arc::new(Geometry::new(&Scene::default(), datum_math::Vec3::ZERO)),
                error: None,
                notes: Vec::new(),
                opened: false,
                rebuild_ms: 0.0,
                scene_ms: 0.0,
                bodies: Vec::new(),
                editing: None,
            }),
            busy: false,
            camera: Camera::default(),
            fitted: true,
            geometry: Arc::new(Geometry::new(&Scene::default(), datum_math::Vec3::ZERO)),
            version: 0,
            smooth: true,
            renderer: None,
            texture: None,
            drawn: None,
            frame_ms: 0.0,
            ui_ms: 0.0,
            adapter: String::new(),
            edits,
            sketch: None,
            entering: None,
            picking: false,
            face: None,
        };
        (v, inbox)
    }

    /// The side panel of a real model: every parameter's definition shows whole in its field.
    #[test]
    fn expressions_show_whole() {
        let (mut v, _inbox) = viewer("parametric.json");
        // Layout needs no GPU; only a picture of the panel (`DATUM_VIEW_SHOT=file.png`) does.
        let shot = std::env::var_os("DATUM_VIEW_SHOT");
        let builder = egui_kittest::Harness::builder().with_size([360.0, 420.0]);
        let builder = if shot.is_some() { builder.wgpu() } else { builder };
        let mut h = builder.build_ui(|ui| {
            egui::Panel::left("datum_side").min_size(340.0).show(ui, |ui| v.side(ui));
        });
        h.run();
        for text in ["floor_count", "hole * 2.5", "w / 2"] {
            // `floor_count` is also a parameter's name: the widest node is the field.
            let width = h.get_all_by_value(text).map(|n| n.rect().width()).fold(0.0f32, f32::max);
            let needed = h.ctx.fonts_mut(|f| f.layout_no_wrap(text.into(), egui::FontId::proportional(14.0), egui::Color32::WHITE).size().x);
            assert!(width > needed + 8.0, "`{text}` needs {needed}, the field is {width}");
        }
        if let Some(path) = shot {
            h.render().unwrap().save(path).unwrap();
        }
    }

    /// ✎ on a sketch asks to edit it; the model that comes back opens it in the view.
    #[test]
    fn a_sketch_opens_for_editing() {
        let (mut v, inbox) = viewer("symmetric_plate.json");
        {
            let mut h = egui_kittest::Harness::builder().with_size([360.0, 600.0]).build_ui(|ui| {
                egui::Panel::left("datum_side").min_size(340.0).show(ui, |ui| v.side(ui));
            });
            h.run();
            h.get_by_label("✎").click();
            h.run();
        }
        assert!(matches!(inbox.try_recv(), Ok(Request::Editing(Some(n))) if n == "outline"));
        // The worker's answer: the sketch's context with the model.
        let mut session = Session::default();
        let model = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/models/symmetric_plate.json");
        session.execute("open", &json!({"path": model.display().to_string()})).unwrap();
        let id = session.graph().nodes()[0].id;
        if let Some(m) = &mut v.model {
            m.editing = Some(("outline".into(), Context { id, input: None }));
        }
        v.sync_sketch();
        let sk = v.sketch.as_ref().expect("editing");
        assert_eq!(sk.placed.as_ref().map(|p| p.solved.dof), Ok(0));
        // The view looks at the whole sketch, 80 × 40.
        assert!(sk.camera.half_height > 20.0 && (sk.camera.center[0] - 40.0).abs() < 1e-9, "{:?}", sk.camera);
        // When the model no longer has it (undone), the window stops editing it.
        if let Some(m) = &mut v.model {
            m.inspect["features"] = json!([]);
        }
        v.sync_sketch();
        assert!(v.sketch.is_none());
        assert!(matches!(inbox.try_recv(), Ok(Request::Editing(None))));
    }

    /// A sketch on a face uses the plate up: while it is edited, the plate is still shown.
    #[test]
    fn the_face_a_sketch_is_on_stays_in_view() {
        let mut session = Session::default();
        let model = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/models/symmetric_plate.json");
        session.execute("open", &json!({"path": model.display().to_string()})).unwrap();
        let faces = session.execute("query_faces", &json!({"name": "plate", "normal": [0, 0, 1]})).unwrap();
        let top = faces["faces"][0]["id"].clone();
        session.execute("add", &json!({"name": "sketch1", "op": "sketch", "inputs": ["plate"], "face": top, "sketch": {"points": {}}})).unwrap();
        let names = |editing: Option<&str>| {
            shown(session.graph(), session.evaluation(), editing)
                .into_iter()
                .map(|i| session.graph().nodes()[i as usize].name.clone())
                .collect::<Vec<_>>()
        };
        assert_eq!(names(None), ["sketch1"]);
        assert_eq!(names(Some("sketch1")), ["plate"]);
    }

    /// A sheet's thickness side flips with one click: left, right, middle.
    #[test]
    fn the_side_of_a_sheet_flips() {
        let (mut v, inbox) = viewer("sheet_panel.json");
        let mut h = egui_kittest::Harness::builder().with_size([360.0, 600.0]).build_ui(|ui| {
            egui::Panel::left("datum_side").min_size(340.0).show(ui, |ui| v.side(ui));
        });
        h.run();
        h.get_by_label("⇄ left").click();
        h.run();
        let sent = inbox.try_recv();
        assert!(matches!(&sent, Ok(Request::Command("edit", fields)) if *fields == json!({"name": "panel", "side": "right"})), "an edit of the side");
    }
}
