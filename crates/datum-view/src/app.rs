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
use crate::dims::{Entry, Field, Pick2, value_of};
use crate::draw::Tool;
use crate::render::{BACKGROUND, Frame, Renderer};
use crate::scene::{Geometry, MeshCache, Scene};
use crate::sketch::{Context, SketchEdit};
use crate::sketch::{Pick, SelectFrame};

/// Opens the viewer window, on the model at `path` if given; models open from the window too.
pub fn run(path: Option<PathBuf>) -> Result<(), String> {
    crate::log::install();
    crate::log::write("opening the window");
    let mut options = eframe::NativeOptions {
        renderer: eframe::Renderer::Wgpu,
        viewport: egui::ViewportBuilder::default().with_title(title(path.as_deref())).with_inner_size([1280.0, 800.0]).with_drag_and_drop(true),
        ..Default::default()
    };
    // On Windows, DirectX 12 with OpenGL for GPUs without it, not Vulkan: a laptop's Vulkan
    // driver killed the process with an access violation that no log can catch (D103).
    // `WGPU_BACKEND` still chooses.
    if cfg!(windows)
        && wgpu::Backends::from_env().is_none()
        && let egui_wgpu::WgpuSetup::CreateNew(setup) = &mut options.wgpu_options.wgpu_setup
    {
        setup.instance_descriptor.backends = wgpu::Backends::DX12 | wgpu::Backends::GL;
    }
    eframe::run_native(
        "Datum",
        options,
        Box::new(move |cc| {
            let rs = cc.wgpu_render_state.as_ref().ok_or("the window has no wgpu renderer")?;
            let info = rs.adapter.get_info();
            let adapter = format!("{} ({:?})", info.name, info.backend);
            crate::log::write(&format!("window on {adapter}"));
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

/// A selection being kept in the library (D98): the element it makes, why the last change or
/// save failed, and which of its insertion points (0, 1) a click in the sketch picks now.
struct Keep {
    element: Json,
    error: Option<String>,
    picking: Option<usize>,
    /// The parameter names as typed, by dimension: one not taken yet is kept here meanwhile.
    naming: BTreeMap<String, String>,
    /// The sockets (by place in `links`) left out: not asked when placing.
    unlinked: std::collections::BTreeSet<usize>,
}

impl Keep {
    fn new(element: Json) -> Self {
        Self { element, error: None, picking: None, naming: BTreeMap::new(), unlinked: std::collections::BTreeSet::new() }
    }

    /// Insertion point `k` (0: the first, 1: the second, if any) is point `p` of the piece, or
    /// (`None`, the second) there is no second.
    fn anchor(&mut self, k: usize, p: Option<String>) {
        let mut anchors = datum_model::library::anchors(&self.element);
        match p {
            Some(p) if k < anchors.len() => anchors[k] = p,
            Some(p) => anchors.push(p),
            None => anchors.truncate(1),
        }
        anchors.dedup();
        if anchors.len() < 2 {
            if let Some(o) = self.element.as_object_mut() {
                o.remove("fit");
            }
        } else if self.element.get("fit").is_none() {
            self.element["fit"] = json!("stretch");
        }
        self.element["anchors"] = json!(anchors);
    }
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
/// Runs a command of the engineer's, journaled for the agent to read (D110).
fn journaled(session: &mut Session, command: &str, params: &Json) -> Result<Json, String> {
    let r = session.execute(command, params);
    crate::log::action(command, params, r.as_ref().err().map(String::as_str));
    r
}

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
                    if let Err(e) = journaled(&mut session, "open", &json!({"path": p.display().to_string()})) {
                        error = Some(e);
                    }
                }
                Request::SetParameter(name, value) => {
                    latest.insert(name, value);
                }
                Request::Command(command, params) => match journaled(&mut session, command, &params) {
                    Ok(reply) => note(&reply),
                    Err(e) => {
                        crate::log::write(&format!("{command} refused: {e}"));
                        error = Some(e);
                    }
                },
                Request::Editing(name) => editing = name,
            }
        }
        for (name, value) in latest {
            match journaled(&mut session, "set_parameter", &json!({"name": name, "value": value})) {
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
    /// The file the model was opened from or saved to.
    path: Option<PathBuf>,
    /// A newer release, once the background check finds one: its version and page.
    update: Receiver<(String, String)>,
    newer: Option<(String, String)>,
    /// The sketch being edited, and one asked for that the model has not brought yet.
    sketch: Option<SketchEdit>,
    entering: Option<String>,
    /// The sketch elements of the library (built in, beside the program, the project's).
    library: Vec<crate::templates::Item>,
    /// What the library palette is filtered by.
    library_filter: String,
    /// The selection being kept in the library.
    keeping: Option<Keep>,
    /// Picking a face to sketch on, and the body and face under the cursor.
    picking: bool,
    face: Option<(usize, datum_brep::TopoId)>,
}

impl Viewer {
    fn new(path: Option<PathBuf>, ctx: egui::Context, adapter: String) -> Self {
        let (requests, inbox) = channel();
        let (outbox, models) = channel();
        let repaint = ctx.clone();
        std::thread::spawn(move || worker(inbox, outbox, ctx));
        let geometry = Arc::new(Geometry::new(&Scene::default(), datum_math::Vec3::ZERO));
        let (found, update) = channel();
        std::thread::spawn(move || {
            if let Some(n) = crate::update::newer() {
                let _ = found.send(n);
                repaint.request_repaint();
            }
        });
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
            path: None,
            update,
            newer: None,
            sketch: None,
            entering: None,
            library: crate::templates::library(None),
            library_filter: String::new(),
            keeping: None,
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
        self.set_path(Some(path.clone()));
        self.send(Request::Open(path));
    }

    /// The model's file, and with it the project whose library elements are listed.
    fn set_path(&mut self, path: Option<PathBuf>) {
        self.library = crate::templates::library(path.as_deref());
        self.path = path;
    }

    /// Where models are opened and saved by dialog: `part` of the workspace (the project, or
    /// the folder beside the program), made if it is not there yet (D115).
    fn parts(&self) -> rfd::FileDialog {
        let dialog = rfd::FileDialog::new().add_filter("Datum model", &["json"]);
        match datum_model::library::workspace(self.path.as_deref()).map(|w| w.join("part")).filter(|d| std::fs::create_dir_all(d).is_ok()) {
            Some(d) => dialog.set_directory(d),
            None => dialog,
        }
    }

    /// Opens a model chosen in the file dialog (Open…, Ctrl+O).
    fn open_by_dialog(&mut self, ui: &egui::Ui) {
        if let Some(p) = self.parts().pick_file() {
            ui.send_viewport_cmd(egui::ViewportCommand::Title(title(Some(&p))));
            self.open(p);
        }
    }

    /// Saves the model: to its file, or (`ask`, or none yet) to one chosen.
    fn save(&mut self, ui: &egui::Ui, ask: bool) {
        let path = match (&self.path, ask) {
            (Some(p), false) => Some(p.clone()),
            _ => self.parts().set_file_name("model.json").save_file(),
        };
        if let Some(p) = path {
            ui.send_viewport_cmd(egui::ViewportCommand::Title(title(Some(&p))));
            self.send(Request::Command("save", json!({"path": p.display().to_string()})));
            self.set_path(Some(p));
        }
    }

    /// The library palette (D117): every sketch element with its outline, name and where it comes
    /// from; a double click places one (asking its parameters, then its sockets), a drag onto
    /// the sketch puts its first anchor where it is let go.
    fn palette(&mut self, ui: &mut egui::Ui) {
        ui.heading("Library");
        ui.add(egui::TextEdit::singleline(&mut self.library_filter).hint_text("Search"));
        ui.weak("Double-click to place · drag onto the sketch");
        let current =
            self.sketch.as_ref().filter(|s| s.draft.tool == Tool::Template).and_then(|s| s.draft.template.as_ref()).map(|t| t["name"].clone());
        let filter = self.library_filter.trim().to_lowercase();
        let mut chosen = None;
        egui::ScrollArea::vertical().show(ui, |ui| {
            for item in &self.library {
                let e = &item.entry.element;
                let (name, description) = (item.entry.name(), e["description"].as_str().unwrap_or_default());
                if !filter.is_empty() && !name.to_lowercase().contains(&filter) && !description.to_lowercase().contains(&filter) {
                    continue;
                }
                let (rect, r) = ui.allocate_exact_size(egui::vec2(ui.available_width(), 52.0), egui::Sense::click_and_drag());
                r.widget_info(|| egui::WidgetInfo::labeled(egui::WidgetType::Button, true, name));
                let visuals = ui.visuals();
                if current.as_ref() == Some(&e["name"]) {
                    ui.painter().rect_filled(rect, 4.0, visuals.selection.bg_fill);
                } else if r.hovered() {
                    ui.painter().rect_filled(rect, 4.0, visuals.widgets.hovered.weak_bg_fill);
                }
                let thumb = egui::Rect::from_min_size(rect.min + egui::vec2(4.0, 4.0), egui::vec2(44.0, 44.0));
                paint_outline(ui.painter(), thumb, &item.outline, visuals.text_color());
                let text = rect.min + egui::vec2(56.0, 8.0);
                ui.painter().text(text, egui::Align2::LEFT_TOP, name, egui::FontId::proportional(14.0), visuals.strong_text_color());
                let from = crate::templates::source(&item.entry);
                ui.painter().text(
                    text + egui::vec2(0.0, 20.0),
                    egui::Align2::LEFT_TOP,
                    from,
                    egui::FontId::proportional(11.0),
                    visuals.weak_text_color(),
                );
                let r = if description.is_empty() { r } else { r.on_hover_text(description) };
                if r.double_clicked() {
                    chosen = Some(e.clone());
                }
                if r.drag_started() {
                    egui::DragAndDrop::set_payload(ui.ctx(), e.clone());
                }
            }
        });
        // While dragged, the element's name follows the pointer.
        if let (Some(t), Some(pos)) = (egui::DragAndDrop::payload::<Json>(ui.ctx()), ui.input(|i| i.pointer.latest_pos())) {
            egui::Area::new(egui::Id::new("datum_library_drag")).fixed_pos(pos + egui::vec2(12.0, 12.0)).order(egui::Order::Tooltip).show(
                ui.ctx(),
                |ui| {
                    egui::Frame::popup(ui.style()).show(ui, |ui| ui.label(t["name"].as_str().unwrap_or_default()));
                },
            );
        }
        if let (Some(t), Some(sk)) = (chosen, &mut self.sketch) {
            sk.draft.choose(t);
            sk.field = None;
        }
    }

    /// The dialog keeping a selection in the library: its name, description and the order its
    /// parameters are asked in; saved where the engineer chooses, the library's folder first.
    fn keep(&mut self, ctx: &egui::Context) {
        let Some(k) = &mut self.keeping else { return };
        let (mut save, mut close) = (false, false);
        egui::Window::new("To library").collapsible(false).resizable(false).show(ctx, |ui| {
            egui::Grid::new("datum_keep").num_columns(2).show(ui, |ui| {
                for (key, label) in [("name", "Name"), ("description", "Description")] {
                    ui.label(label);
                    let mut text = k.element[key].as_str().unwrap_or_default().to_string();
                    if ui.text_edit_singleline(&mut text).changed() {
                        k.element[key] = json!(text);
                    }
                    ui.end_row();
                }
            });
            // Where the element is put: the engineer's choice of its points (D98).
            let anchors = datum_model::library::anchors(&k.element);
            egui::Grid::new("datum_keep_anchors").num_columns(3).show(ui, |ui| {
                for (i, label, hint) in [
                    (0, "Insertion point", "The point of the element that goes where the first click is"),
                    (
                        1,
                        "Second point",
                        "Only for an element put by two clicks: a rectangle by its corners, or a section between two points of a chain",
                    ),
                ] {
                    ui.label(label).on_hover_text(hint);
                    ui.strong(anchors.get(i).map_or("—", String::as_str));
                    ui.horizontal(|ui| {
                        let hint = "Click a point of the selection in the sketch";
                        if ui.add_enabled(i <= anchors.len(), egui::Button::new("Pick").selected(k.picking == Some(i))).on_hover_text(hint).clicked()
                        {
                            k.picking = if k.picking == Some(i) { None } else { Some(i) };
                        }
                        if i == 1 && anchors.len() > 1 && ui.button("✖").on_hover_text("No second point: the element is moved as it is").clicked() {
                            k.anchor(1, None);
                        }
                    });
                    ui.end_row();
                }
            });
            if k.picking.is_some() {
                ui.colored_label(egui::Color32::from_rgb(230, 120, 20), "Click a point of the selection in the sketch · Esc cancels");
            }
            if anchors.len() > 1 {
                let mut fit = k.element["fit"].as_str().unwrap_or("stretch").to_string();
                ui.radio_value(&mut fit, "stretch".into(), "Stretch along the axes to the second point (a rectangle by its corners)");
                ui.radio_value(&mut fit, "align".into(), "Turn toward the second point (a section between two points of a chain)");
                k.element["fit"] = json!(fit);
            }
            // The dimensions: a tick makes one a question of the element, under the name beside it.
            use datum_model::library::{dimensions, parameter_names, parametrize, rename_parameter, unparametrize};
            let dims = dimensions(&k.element);
            if dims.is_empty() {
                ui.weak("The selection has no dimensions: the element goes in as it is.");
            } else {
                ui.label("Dimensions — ticked ones are asked when the element is placed:");
            }
            let names = parameter_names(&k.element).unwrap_or_default();
            let mut change: Option<Result<(), String>> = None;
            egui::Grid::new("datum_keep_dims").num_columns(3).show(ui, |ui| {
                for d in &dims {
                    let asked = d.value.as_str().map(str::trim).filter(|t| names.iter().any(|n| n == t)).map(str::to_string);
                    let mut on = asked.is_some();
                    let given = d.value.is_number() || asked.is_some();
                    if ui.add_enabled(given, egui::Checkbox::new(&mut on, &d.label)).changed() {
                        change = Some(match &asked {
                            Some(n) => unparametrize(&mut k.element, n),
                            None => {
                                let free = (1..).map(|i| format!("d{i}")).find(|n| !names.contains(n)).unwrap_or_default();
                                parametrize(&mut k.element, &d.label, &free)
                            }
                        });
                    }
                    match &asked {
                        Some(n) => {
                            let text = k.naming.entry(d.label.clone()).or_insert_with(|| n.clone());
                            if ui.add(egui::TextEdit::singleline(text).desired_width(90.0)).changed() {
                                match rename_parameter(&mut k.element, n, text.trim()) {
                                    Ok(()) => change = Some(Ok(())),
                                    Err(e) => k.error = Some(e),
                                }
                            }
                        }
                        None => {
                            k.naming.remove(&d.label);
                            ui.label("");
                        }
                    }
                    // The value: a parameter's own, a number or an expression as given.
                    let value = asked
                        .as_ref()
                        .and_then(|n| k.element["parameters"].as_array()?.iter().find(|p| p["name"] == n.as_str()).map(|p| p["value"].clone()));
                    ui.weak(crate::dims::text_of(value.as_ref().unwrap_or(&d.value)));
                    ui.end_row();
                }
            });
            match change {
                Some(Ok(())) => k.error = None,
                Some(Err(e)) => k.error = Some(e),
                None => {}
            }
            // Constraints reaching out of the selection: asked to be completed when placing (D116).
            let links = k.element["links"].as_array().cloned().unwrap_or_default();
            if !links.is_empty() {
                ui.label("Constraints to what is outside — ticked ones are completed by a click when placing:");
                for (i, l) in links.iter().enumerate() {
                    let mut names = Vec::new();
                    datum_model::visit_names(&mut l["constraint"].clone(), &mut |n| names.push(n.clone()));
                    let label = format!(
                        "{} {} (? a {})",
                        l["constraint"]["type"].as_str().unwrap_or_default(),
                        names.join(" "),
                        l["kind"].as_str().unwrap_or_default()
                    );
                    let mut on = !k.unlinked.contains(&i);
                    if ui.checkbox(&mut on, label).changed() {
                        if on {
                            k.unlinked.remove(&i);
                        } else {
                            k.unlinked.insert(i);
                        }
                    }
                }
            }
            if let Some(list) = k.element.get_mut("parameters").and_then(Json::as_array_mut) {
                ui.label("Asked in this order:");
                let mut up = None;
                for (i, p) in list.iter().enumerate() {
                    ui.horizontal(|ui| {
                        if ui.add_enabled(i > 0, egui::Button::new("⏶")).clicked() {
                            up = Some(i);
                        }
                        ui.label(format!("{} = {}", p["name"].as_str().unwrap_or_default(), crate::dims::text_of(&p["value"])));
                    });
                }
                if let Some(i) = up {
                    list.swap(i - 1, i);
                }
            }
            if let Some(e) = &k.error {
                ui.colored_label(egui::Color32::from_rgb(190, 40, 40), e);
            }
            ui.horizontal(|ui| {
                save = ui.button("Save…").clicked();
                close = ui.button("Cancel").clicked();
            });
        });
        if save {
            let mut element = k.element.clone();
            if let Some(links) = element.get_mut("links").and_then(Json::as_array_mut) {
                let mut i = 0;
                links.retain(|_| {
                    i += 1;
                    !k.unlinked.contains(&(i - 1))
                });
                if links.is_empty()
                    && let Some(o) = element.as_object_mut()
                {
                    o.remove("links");
                }
            }
            let name = element["name"].as_str().unwrap_or_default().to_string();
            let folder = datum_model::library::folder("sketch", self.path.as_deref());
            let mut dialog = rfd::FileDialog::new().add_filter("Datum element", &["json"]).set_file_name(datum_model::library::file_name(&name));
            if let Some(f) = folder.filter(|f| std::fs::create_dir_all(f).is_ok()) {
                dialog = dialog.set_directory(f);
            }
            let written = datum_model::library::check("sketch", &element).and_then(|()| match dialog.save_file() {
                Some(path) => serde_json::to_string_pretty(&element)
                    .map_err(|e| e.to_string())
                    .and_then(|t| std::fs::write(&path, t).map_err(|e| format!("{}: {e}", path.display())))
                    .map(|()| true),
                None => Ok(false),
            });
            match written {
                Ok(true) => {
                    close = true;
                    self.library = crate::templates::library(self.path.as_deref());
                }
                Ok(false) => {}
                Err(e) => k.error = Some(e),
            }
        }
        if close {
            self.keeping = None;
        }
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
    /// S, as in Inventor: a sketch on a face to be clicked; with no model to click, on XY.
    fn start_sketch(&mut self) {
        if self.model.as_ref().is_some_and(|m| !m.bodies.is_empty()) {
            self.picking = !self.picking;
        } else {
            self.new_sketch(json!({}));
        }
    }

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
            if ui.button("Open…").on_hover_text("Ctrl+O").clicked() {
                self.open_by_dialog(ui);
            }
            if ui.button("New").on_hover_text("An empty model").clicked() {
                self.leave();
                self.set_path(None);
                ui.send_viewport_cmd(egui::ViewportCommand::Title(title(None)));
                self.send(Request::Command("new", json!({})));
            }
            if ui.button("Save").on_hover_text("Ctrl+S").clicked() {
                self.save(ui, false);
            }
            if ui.button("Save as…").clicked() {
                self.save(ui, true);
            }
            ui.separator();
            if let Ok(n) = self.update.try_recv() {
                self.newer = Some(n);
            }
            if let Some((version, url)) = &self.newer {
                ui.hyperlink_to(format!("⬆ Datum {version} is out"), url).on_hover_text("Its page: what is new, and the download");
                ui.separator();
            }
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
            if ui.selectable_label(self.picking, "Sketch on face").on_hover_text("Then click a plane face of the model (S)").clicked() {
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
        ui.weak(format!("Datum {} · problems are logged to {}", env!("CARGO_PKG_VERSION"), crate::log::path().display()));
        let mut on = crate::log::recording();
        let hint = "Every command you run, as a line an agent can read and repeat; begun anew each time. Off by default: models may be confidential";
        if ui.checkbox(&mut on, "Record actions for an agent").on_hover_text(hint).changed() {
            crate::log::record(on);
        }
        if on {
            ui.weak(format!("Recording to {}", crate::log::actions_path().display()));
        }
        if let Some(e) = &m.error {
            ui.colored_label(egui::Color32::from_rgb(190, 40, 40), e);
        }
        for n in &m.notes {
            ui.colored_label(egui::Color32::from_rgb(190, 120, 0), format!("⚠ {n}"));
        }
        let mut done = false;
        let mut sent = Vec::new();
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
                    (
                        Tool::Dimension,
                        "Dimension",
                        "Click a line, an arc or a circle; a point, then a line; two points, then where it goes. Frame two lines: their angle (D)",
                    ),
                ] {
                    if ui.selectable_label(sk.draft.tool == tool, label).on_hover_text(hint).clicked() {
                        sk.draft.cancel();
                        sk.draft.tool = tool;
                    }
                }
            });
            // The library is the palette on the right; a selection goes into it from here.
            ui.horizontal_wrapped(|ui| {
                let hint = "The selection as a library element: you choose where it is put, which dimensions it asks and what it ties to outside";
                if ui.add_enabled(!sk.selected.is_empty(), egui::Button::new("To library…")).on_hover_text(hint).clicked() {
                    self.keeping = sk.extract("New element", Some(&sk.params)).map(Keep::new);
                }
            });
            ui.horizontal(|ui| {
                ui.checkbox(&mut sk.draft.construction, "Construction");
                ui.checkbox(&mut sk.names, "Names");
                ui.checkbox(&mut sk.constraints, "Constraints")
                    .on_hover_text("Badges of the geometric constraints: click one to select it, Del removes it");
                if sk.draft.tool == Tool::Line && sk.draft.chain.as_ref().is_some_and(|c| c.last.is_some()) {
                    ui.checkbox(&mut sk.draft.tangent, "Tangent arc (T)");
                }
            });
            // A dimension that would over-constrain the sketch: as a reference one, or not at all.
            let (mut reference, mut dismiss) = (false, false);
            if let Some(c) = &sk.conflict {
                ui.colored_label(egui::Color32::from_rgb(190, 40, 40), &c.why);
                ui.horizontal(|ui| {
                    reference = !c.constraint.is_null() && ui.button("Add as reference").on_hover_text("Measured, holding nothing").clicked();
                    dismiss = ui.button("Dismiss").clicked();
                });
            }
            if dismiss {
                sk.conflict = None;
            }
            if reference && let Some(p) = sk.conflict_as_reference() {
                sent.push(Request::Command("edit_sketch", p));
            }
            ui.weak(match sk.draft.tool {
                Tool::Select => "Drag a blue point · double-click a dimension to change it · click to select, Shift to add · frame: to the right what is inside, to the left what it touches · Ctrl+C, Ctrl+V · Del removes · Esc clears, then leaves",
                Tool::Dimension => "Click a line, arc or circle and type the value, Enter · two points: then click where it goes · angle: frame two lines",
                Tool::Template => {
                    "Click where each anchor goes (a rectangle: a corner, then the other; from its centre: the centre, then a corner) · \
                     parameters in the field: Tab to the next, a preset beside it, clicks size what is not typed · Enter puts it at the cursor · Esc ends"
                }
                _ => "Click to place · type a number for the length (diameter, radius), Enter · snaps to points, middles, curves and the face's edges · Shift: no snaps · Esc ends the command",
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
                                && let Some(p) = beside(
                                    self.path.as_deref(),
                                    rfd::FileDialog::new().add_filter("DXF", &["dxf"]).set_file_name(format!("{name}.dxf")),
                                )
                                .save_file()
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
        for r in sent {
            self.send(r);
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
                // On a dimension, a double click changes it; elsewhere it fits the view.
                Some(sk) if r.double_clicked() => {
                    if sk.hover.is_none() {
                        sk.fit(scene);
                    }
                }
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
            sk.paint_frame(&painter, rect);
            if let Some(k) = &self.keeping {
                sk.paint_anchors(&painter, rect, &datum_model::library::anchors(&k.element));
            }
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
        // An element dragged from the palette and let go here: its first anchor goes there, as a
        // click would put it (the pointer belongs to the palette while it is dragged).
        if let (Some(t), Some(pos)) = (r.dnd_release_payload::<Json>(), ui.input(|i| i.pointer.latest_pos())) {
            sk.draft.choose((*t).clone());
            sk.field = None;
            let mut draft = std::mem::take(&mut sk.draft);
            draft.track(sk, f64::from(pos.x - rect.center().x), f64::from(pos.y - rect.center().y), h, false);
            if let Some(patch) = draft.click(sk) {
                sent.push(Request::Command("edit_sketch", sk.apply(patch)));
            }
            sk.draft = draft;
        }
        let typing = ui.ctx().egui_wants_keyboard_input();
        if !typing {
            let tool = ui.input(|i| {
                [
                    (egui::Key::L, Tool::Line),
                    (egui::Key::A, Tool::Arc),
                    (egui::Key::C, Tool::Circle),
                    (egui::Key::P, Tool::Point),
                    (egui::Key::D, Tool::Dimension),
                ]
                .into_iter()
                .find(|(k, _)| i.key_pressed(*k) && i.modifiers.is_none())
                .map(|(_, t)| t)
            });
            if let Some(t) = tool {
                sk.draft.cancel();
                sk.draft.tool = t;
            }
            if ui.input(|i| i.key_pressed(egui::Key::R) && i.modifiers.is_none())
                && let Some(t) = self.library.first()
            {
                sk.draft.choose(t.entry.element.clone());
                sk.field = None;
            }
            // Copy: the selection as an element, on the clipboard as JSON. Paste: an element
            // from the clipboard (a copy, or one written by hand or by an agent), placed by clicks.
            let (copy, paste) = ui.input(|i| {
                let copy = i.events.iter().any(|e| matches!(e, egui::Event::Copy));
                let paste = i.events.iter().find_map(|e| if let egui::Event::Paste(t) = e { Some(t.clone()) } else { None });
                (copy, paste)
            });
            if copy && let Some(t) = sk.copy() {
                ui.ctx().copy_text(serde_json::to_string_pretty(&t).unwrap_or_default());
            }
            if let Some(t) = paste.and_then(|p| serde_json::from_str::<Json>(&p).ok()).filter(|t| datum_model::library::check("sketch", t).is_ok()) {
                sk.draft.choose(t);
                sk.field = None;
            }
            if ui.input(|i| i.key_pressed(egui::Key::T)) && sk.draft.chain.as_ref().is_some_and(|c| c.last.is_some()) {
                sk.draft.tangent = !sk.draft.tangent;
            }
        }
        // Placing a library element with parameters: the field by the cursor asks them, one by
        // one, Tab to the next; a preset fills them all (D98).
        let names = sk
            .draft
            .template
            .as_ref()
            .filter(|_| sk.draft.tool == Tool::Template && sk.draft.linking.is_empty())
            .and_then(|t| datum_model::library::parameter_names(t).ok())
            .unwrap_or_default();
        // Its sockets next, one by one: what the one asked takes, by the cursor (D116).
        if let (Some(link), Some(spot)) = (sk.draft.linking.first(), &sk.draft.cursor) {
            let (x, y) = sk.camera.offset(spot.at, h);
            let pos = rect.center() + egui::vec2(x as f32 + 14.0, y as f32 + 14.0);
            let text = format!("{} — click a {} · Enter skips ({} left)", link.shown, link.kind, sk.draft.linking.len());
            egui::Area::new(egui::Id::new("datum_sketch_link")).fixed_pos(pos).order(egui::Order::Foreground).show(ui.ctx(), |ui| {
                egui::Frame::popup(ui.style()).show(ui, |ui| ui.label(text));
            });
        }
        if names.is_empty() {
            if sk.field.as_ref().is_some_and(|f| matches!(f.entry, Entry::Parameter(_))) {
                sk.field = None;
            }
        } else if let Some(spot) = &sk.draft.cursor {
            let k = sk.draft.asking % names.len();
            match &mut sk.field {
                Some(f) if f.entry == Entry::Parameter(k) => f.at = spot.at,
                _ => {
                    sk.field =
                        Some(Field { entry: Entry::Parameter(k), text: crate::templates::asked(&sk.draft, &names[k], &sk.params), at: spot.at })
                }
            }
        }
        // The field by the cursor: a value typed while drawing, a parameter of an element placed,
        // or a dimension's. Enter applies it, Esc drops it.
        let mut field_done = (false, false);
        let mut parameter_done = None;
        if let Some(f) = &mut sk.field {
            let (x, y) = sk.camera.offset(f.at, h);
            let pos = rect.center() + egui::vec2(x as f32 + 14.0, y as f32 + 14.0);
            let draft = &mut sk.draft;
            egui::Area::new(egui::Id::new("datum_sketch_field")).fixed_pos(pos).order(egui::Order::Foreground).show(ui.ctx(), |ui| {
                let asking = if let Entry::Parameter(k) = f.entry { Some(k) } else { None };
                let id = egui::Id::new("datum_sketch_field_text").with(asking);
                let fresh = !ui.memory(|m| m.has_focus(id));
                // Tab and Shift+Tab go through the parameters, not to another widget.
                let tab = asking.map(|_| {
                    ui.input_mut(|i| (i.consume_key(egui::Modifiers::NONE, egui::Key::Tab), i.consume_key(egui::Modifiers::SHIFT, egui::Key::Tab)))
                });
                let mut out = None;
                // Over the sketch, a parameter's name needs a ground of its own to be read.
                let frame = if asking.is_some() { egui::Frame::popup(ui.style()) } else { egui::Frame::NONE };
                frame.show(ui, |ui| {
                    ui.horizontal(|ui| {
                        if let Some(k) = asking {
                            ui.label(format!("{} ({}/{})", names[k], k + 1, names.len()));
                        }
                        let o = egui::TextEdit::singleline(&mut f.text).id(id).desired_width(110.0).show(ui);
                        if let Some(k) = asking
                            && o.response.changed()
                        {
                            match f.text.trim() {
                                "" => draft.answers.remove(&names[k]),
                                t => draft.answers.insert(names[k].clone(), value_of(t)),
                            };
                        }
                        if let Some(presets) = asking.and(draft.template.as_ref()).and_then(|t| t["presets"].as_object()).filter(|p| !p.is_empty()) {
                            let shown = draft.preset.clone().unwrap_or_else(|| "Element's values".into());
                            egui::ComboBox::from_id_salt("datum_sketch_preset").selected_text(shown).show_ui(ui, |ui| {
                                for name in std::iter::once(None).chain(presets.keys().map(Some)) {
                                    let label = name.map_or("Element's values", String::as_str);
                                    if ui.selectable_label(draft.preset.as_ref() == name, label).clicked() {
                                        draft.preset = name.cloned();
                                        draft.answers.clear();
                                        parameter_done = Some(0);
                                    }
                                }
                            });
                        }
                        out = Some(o);
                    })
                });
                let Some(mut out) = out else { return };
                // A dimension's or a parameter's value comes selected, to be typed over; a value
                // typed while drawing goes on from its first digit.
                if fresh && f.entry != Entry::Draw {
                    out.state.cursor.set_char_range(Some(egui::text::CCursorRange::two(
                        egui::text::CCursor::new(0),
                        egui::text::CCursor::new(f.text.chars().count()),
                    )));
                    out.state.store(ui.ctx(), id);
                }
                if parameter_done.is_none() {
                    out.response.request_focus();
                }
                field_done = ui.input(|i| (i.key_pressed(egui::Key::Enter), i.key_pressed(egui::Key::Escape)));
                if let (Some(k), Some((next, back))) = (asking, tab)
                    && (next || back)
                {
                    parameter_done = Some(if next { k + 1 } else { k + names.len() - 1 } % names.len());
                }
            });
            // While drawing, the value sizes what is drawn as it is typed.
            if f.entry == Entry::Draw {
                let v = value_of(&f.text);
                let lookup = |n: &str| sk.params.get(n).copied().ok_or_else(|| datum_math::expr::ExprError::Unknown(n.to_string()));
                let size = serde_json::from_value::<datum_math::expr::Num>(v.clone()).ok().and_then(|n| n.resolve(&lookup).ok());
                sk.draft.typed = size.filter(|s| *s > 0.0).map(|s| (v, s));
            }
        }
        // Another parameter asked (Tab, or a preset chosen): the field opens again on it.
        if let Some(k) = parameter_done {
            sk.draft.asking = k;
            sk.field = None;
        }
        // Esc on a parameter ends placing the element, as Esc ends any command.
        if field_done.1 && sk.field.as_ref().is_some_and(|f| matches!(f.entry, Entry::Parameter(_))) {
            sk.draft.escape();
        }
        let typing_field = sk.field.is_some();
        if field_done.1 {
            sk.field = None;
            sk.draft.typed = None;
        }
        // A digit typed while drawing opens the field with it.
        if !typing_field && !typing && sk.draft.takes_value() {
            let first = ui.input(|i| i.events.iter().find_map(|e| if let egui::Event::Text(t) = e { Some(t.clone()) } else { None }));
            if let (Some(t), Some(spot)) = (first.filter(|t| t.starts_with(|c: char| c.is_ascii_digit() || c == '.' || c == '-')), &sk.draft.cursor) {
                sk.field = Some(Field { entry: Entry::Draw, text: t, at: spot.at });
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
            let placing =
                r.clicked() || (draft.tangent && r.drag_stopped_by(egui::PointerButton::Primary)) || (field_done.0 && draft.typed.is_some());
            if draft.tool == Tool::Dimension {
                // A frame from an empty place, either way, picks what it touches to dimension: two
                // lines, their angle.
                if r.drag_started_by(egui::PointerButton::Primary)
                    && let Some(o) = ui.input(|i| i.pointer.press_origin())
                {
                    let (x, y) = (f64::from(o.x - rect.center().x), f64::from(o.y - rect.center().y));
                    if sk.point_at(x, y, h).is_none() && sk.curve_at(x, y, h).is_none() {
                        sk.frame = Some(SelectFrame { ends: [(x, y); 2], touching: true });
                    }
                }
                if let Some(framed) = sk.frame_to(offset, r.drag_stopped(), h, |t| crate::sketch::label_size(ui.painter(), t)) {
                    draft.picks = sk.dimension_framed(&framed);
                }
                if r.clicked()
                    && let Some((x, y)) = offset
                {
                    // An element picked, or the place where the dimension goes.
                    let pick = sk.point_at(x, y, h).map(Pick2::Point).or_else(|| sk.curve_at(x, y, h).map(Pick2::Curve));
                    let at = sk.camera.at(x, y, h);
                    match pick {
                        Some(p) if draft.picks.len() < 2 && !draft.picks.contains(&p) => {
                            draft.picks.push(p);
                            // A curve, or a point and a line: dimensioned at once.
                            if sk.dimension_now(&draft.picks, at) {
                                draft.picks.clear();
                            }
                        }
                        Some(_) => {}
                        // Where the dimension between two points goes.
                        None => {
                            if let Some((c, key, now)) = sk.dimension(&draft.picks, at) {
                                sk.ask(c, key, now, at);
                            }
                            draft.picks.clear();
                        }
                    }
                }
            } else if placing && let Some(patch) = draft.click(sk) {
                sent.push(Request::Command("edit_sketch", sk.apply(patch)));
                if field_done.0 || typing_field {
                    sk.field = None;
                }
            }
            // Enter while placing an element: it goes where the cursor is; on a socket, skips it.
            let skip = !draft.linking.is_empty() && !typing && ui.input(|i| i.key_pressed(egui::Key::Enter));
            if (field_done.0 || skip)
                && draft.tool == Tool::Template
                && let Some(patch) = draft.enter(sk)
            {
                sent.push(Request::Command("edit_sketch", sk.apply(patch)));
            }
            // Enter on a dimension's value (the dimension tool opened it).
            if field_done.0
                && sk.field.as_ref().is_some_and(|f| !f.entry.drawing())
                && let Some(p) = sk.commit_field()
            {
                sent.push(Request::Command("edit_sketch", p));
            }
            if r.dragged_by(egui::PointerButton::Secondary) || r.dragged_by(egui::PointerButton::Middle) {
                let d = r.drag_delta();
                sk.camera.pan(f64::from(d.x), f64::from(d.y), h);
            }
            if !typing && !typing_field && ui.input(|i| i.key_pressed(egui::Key::Escape)) {
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
        // A drag starts a few pixels from where the button went down: what is there is dragged;
        // from an empty place, a selection frame.
        if r.drag_started_by(egui::PointerButton::Primary)
            && let Some(o) = ui.input(|i| i.pointer.press_origin())
        {
            let (x, y) = (f64::from(o.x - rect.center().x), f64::from(o.y - rect.center().y));
            sk.hover = sk.hit(x, y, h, |t| crate::sketch::label_size(ui.painter(), t));
            if !sk.start_drag() && sk.hover.is_none() {
                sk.frame = Some(SelectFrame { ends: [(x, y); 2], touching: false });
            }
        }
        if let Some(framed) = sk.frame_to(offset, r.drag_stopped(), h, |t| crate::sketch::label_size(ui.painter(), t)) {
            if !ui.input(|i| i.modifiers.shift) {
                sk.selected.clear();
            }
            for p in framed {
                if !sk.selected.contains(&p) {
                    sk.selected.push(p);
                }
            }
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
        // A dimension changes on a double click.
        if r.double_clicked()
            && let (Some(p @ (Pick::Constraint(_) | Pick::Curve(_))), Some((x, y))) = (sk.hover.clone(), offset)
        {
            sk.edit_label(&p, sk.camera.at(x, y, h));
        }
        if field_done.0
            && let Some(p) = sk.commit_field()
        {
            sent.push(Request::Command("edit_sketch", p));
        }
        // Keeping a selection in the library: a click picks its insertion point instead.
        if let Some(k) = self.keeping.as_mut().filter(|k| k.picking.is_some()) {
            if ui.input(|i| i.key_pressed(egui::Key::Escape)) {
                k.picking = None;
            } else if r.clicked()
                && let (Some(i), Some((x, y))) = (k.picking, offset)
            {
                match sk.point_at(x, y, h).filter(|p| k.element["sketch"]["points"].get(p).is_some()) {
                    Some(p) => {
                        k.anchor(i, Some(p));
                        k.picking = None;
                        k.error = None;
                    }
                    None => k.error = Some("Not a point of the selection: pick one of its own".into()),
                }
            }
        } else if r.clicked() {
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
        if !ui.ctx().egui_wants_keyboard_input() && !typing_field && !field_done.1 {
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

/// An element's outline fitted into `rect`, the sketch's Y up.
fn paint_outline(painter: &egui::Painter, rect: egui::Rect, outline: &[Vec<[f64; 2]>], color: egui::Color32) {
    let points = || outline.iter().flatten();
    let (Some(x0), Some(x1)) = (points().map(|q| q[0]).min_by(f64::total_cmp), points().map(|q| q[0]).max_by(f64::total_cmp)) else { return };
    let (Some(y0), Some(y1)) = (points().map(|q| q[1]).min_by(f64::total_cmp), points().map(|q| q[1]).max_by(f64::total_cmp)) else { return };
    let span = (x1 - x0).max(y1 - y0);
    if span <= 0.0 {
        return;
    }
    let k = f64::from(rect.width().min(rect.height())) / span;
    let c = rect.center();
    let at = |q: &[f64; 2]| egui::pos2(c.x + ((q[0] - 0.5 * (x0 + x1)) * k) as f32, c.y - ((q[1] - 0.5 * (y0 + y1)) * k) as f32);
    for line in outline {
        painter.add(egui::Shape::line(line.iter().map(at).collect(), egui::Stroke::new(1.2, color)));
    }
}

/// `dialog` opened in the folder of the model's file, if it has one: what is made from a part
/// goes beside it.
fn beside(model: Option<&std::path::Path>, dialog: rfd::FileDialog) -> rfd::FileDialog {
    match model.and_then(std::path::Path::parent) {
        Some(d) => dialog.set_directory(d),
        None => dialog,
    }
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
            if ui.input_mut(|i| i.consume_shortcut(&shortcut(egui::Key::O))) {
                self.open_by_dialog(ui);
            }
            if ui.input_mut(|i| i.consume_shortcut(&shortcut(egui::Key::S))) {
                self.save(ui, false);
            } else if self.sketch.is_none() && ui.input(|i| i.key_pressed(egui::Key::S) && i.modifiers.is_none()) {
                self.start_sketch();
            }
            if redo {
                self.send(Request::Command("redo", json!({})));
            }
        }
        egui::Panel::top("datum_toolbar").show(ui, |ui| self.toolbar(ui));
        egui::Panel::left("datum_side").min_size(340.0).show(ui, |ui| self.side(ui));
        if self.sketch.is_some() {
            egui::Panel::right("datum_library").min_size(200.0).show(ui, |ui| self.palette(ui));
        }
        self.keep(ui.ctx());
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
            path: None,
            update: channel().1,
            newer: None,
            sketch: None,
            entering: None,
            library: crate::templates::library(None),
            library_filter: String::new(),
            keeping: None,
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

    /// Placing a library element with parameters in one stream of input: the field by the
    /// cursor asks the first, Tab the next, what is typed goes into its dimension, Enter puts the
    /// element at the cursor, Esc ends the command.
    #[test]
    fn an_element_asked_in_one_stream() {
        let (mut v, inbox) = viewer("symmetric_plate.json");
        let mut session = Session::default();
        let model = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/models/symmetric_plate.json");
        session.execute("open", &json!({"path": model.display().to_string()})).unwrap();
        if let Some(m) = &mut v.model {
            m.editing = Some(("outline".into(), Context { id: session.graph().nodes()[0].id, input: None }));
        }
        v.entering = Some("outline".into());
        v.sync_sketch();
        let element = json!({"name": "Plate", "anchors": ["a", "c"],
            "parameters": [{"name": "width", "value": 100}, {"name": "height", "value": 50}],
            "presets": {"Small": {"width": 20, "height": 10}},
            "sketch": {"points": {"a": [0, 0], "b": [1, 0], "c": [1, 1], "d": [0, 1]},
                       "lines": {"bottom": {"from": "a", "to": "b"}, "right": {"from": "b", "to": "c"}, "top": {"from": "c", "to": "d"}, "left": {"from": "d", "to": "a"}},
                       "constraints": [{"type": "length", "line": "bottom", "value": "width"}, {"type": "length", "line": "left", "value": "height"}]}});
        v.sketch.as_mut().unwrap().draft.choose(element);
        // `DATUM_ELEMENT_SHOT=file.png` saves a picture of the field and the preview.
        let shot = std::env::var_os("DATUM_ELEMENT_SHOT");
        let builder = egui_kittest::Harness::builder().with_size([600.0, 400.0]);
        let builder = if shot.is_some() { builder.wgpu() } else { builder };
        let mut h = builder.build_ui(|ui| {
            let (rect, r) = ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());
            ui.painter().rect_filled(rect, 0.0, egui::Color32::from_rgb(235, 238, 242));
            let offset = r.hover_pos().map(|p| (f64::from(p.x - rect.center().x), f64::from(p.y - rect.center().y)));
            v.sketch_input(ui, rect, &r, offset);
            if let Some(sk) = &v.sketch {
                let painter = ui.painter().with_clip_rect(rect);
                sk.paint(&painter, rect);
                sk.draft.paint(sk, &painter, rect);
            }
        });
        h.hover_at(egui::pos2(300.0, 200.0));
        h.run();
        h.get_by_label("width (1/2)");
        h.get_by_value("Element's values");
        if let Some(path) = shot {
            h.render().unwrap().save(path).unwrap();
        }
        h.key_press(egui::Key::Tab);
        h.run();
        h.get_by_label("height (2/2)");
        h.get_by_role(egui::accesskit::Role::TextInput).type_text("7");
        h.run();
        h.key_press(egui::Key::Enter);
        h.run();
        h.key_press(egui::Key::Escape);
        h.run();
        drop(h);
        let sent: Vec<Request> = inbox.try_iter().collect();
        let edit = sent.iter().find_map(|r| if let Request::Command("edit_sketch", p) = r { Some(p.clone()) } else { None }).expect("placed");
        let lengths: Vec<&Json> = edit["add_constraints"].as_array().unwrap().iter().filter(|c| c["type"] == "length").map(|c| &c["value"]).collect();
        assert_eq!(lengths, [&json!(100.0), &json!(7.0)], "{edit}");
        let sk = v.sketch.as_ref().unwrap();
        assert!(sk.draft.tool == Tool::Select && sk.field.is_none(), "Esc ends the command");
    }

    /// The whole plate kept in the library: its dimensions name w, h, pitch and hole, which the
    /// dialog lists to be put in order; ⏶ moves one up. `DATUM_KEEP_SHOT=file.png` saves a picture.
    #[test]
    fn a_selection_kept_in_the_library() {
        let (mut v, _inbox) = viewer("symmetric_plate.json");
        let mut session = Session::default();
        let model = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/models/symmetric_plate.json");
        session.execute("open", &json!({"path": model.display().to_string()})).unwrap();
        if let Some(m) = &mut v.model {
            m.editing = Some(("outline".into(), Context { id: session.graph().nodes()[0].id, input: None }));
        }
        v.entering = Some("outline".into());
        v.sync_sketch();
        let sk = v.sketch.as_mut().unwrap();
        sk.selected = ["bottom", "right", "top", "left", "axis", "left_hole", "right_hole"].iter().map(|n| Pick::Curve(n.to_string())).collect();
        let element = sk.extract("Plate", Some(&sk.params)).unwrap();
        let names = |e: &Json| e["parameters"].as_array().unwrap().iter().map(|p| p["name"].as_str().unwrap().to_string()).collect::<Vec<_>>();
        let first = names(&element);
        assert_eq!(first.len(), 4, "w, h, pitch, hole: {element}");
        v.keeping = Some(Keep::new(element));
        let shot = std::env::var_os("DATUM_KEEP_SHOT");
        let builder = egui_kittest::Harness::builder().with_size([560.0, 560.0]);
        let builder = if shot.is_some() { builder.wgpu() } else { builder };
        let mut h = builder.build_ui(|ui| v.keep(ui.ctx()));
        h.run();
        h.get_by_label("Asked in this order:");
        if let Some(path) = shot {
            h.render().unwrap().save(path).unwrap();
        }
        h.get_all_by_label("⏶").nth(1).unwrap().click();
        h.run();
        // The second insertion point: Pick, then a click in the sketch.
        h.get_all_by_label("Pick").nth(1).unwrap().click();
        h.run();
        h.get_by_label("Click a point of the selection in the sketch · Esc cancels");
        drop(h);
        let k = v.keeping.as_mut().unwrap();
        let now = names(&k.element);
        assert_eq!((&now[0], &now[1]), (&first[1], &first[0]), "the second moved up");
        assert_eq!(k.picking, Some(1));
        let proposed = datum_model::library::anchors(&k.element)[0].clone();
        k.anchor(1, Some("b".into()));
        assert_eq!((&k.element["anchors"], &k.element["fit"]), (&json!([proposed, "b"]), &json!("stretch")), "{}", k.element);
        k.anchor(0, Some("b".into()));
        assert_eq!((&k.element["anchors"], k.element.get("fit")), (&json!(["b"]), None), "one point twice is one point");
    }

    /// Dimensions given as numbers (no model parameters at all): a tick asks one when placing,
    /// under the name typed beside it. `DATUM_KEEP_SHOT=file.png` saves a picture.
    #[test]
    fn a_dimension_ticked_is_asked() {
        let (mut v, _inbox) = viewer("symmetric_plate.json");
        v.keeping = Some(Keep::new(json!({"name": "test", "anchors": ["p5"],
            "links": [{"constraint": {"type": "perpendicular", "lines": ["?", "l5"]}, "kind": "line"}],
            "sketch": {"points": {"p5": [0, 0], "p6": [-4, -8], "p7": [5, -11]},
                       "lines": {"l5": {"from": "p5", "to": "p6"}, "l6": {"from": "p6", "to": "p7"}},
                       "constraints": [{"type": "perpendicular", "lines": ["l5", "l6"]},
                                       {"type": "length", "line": "l5", "value": 8.900914}, {"type": "length", "line": "l6", "value": 6.62265}]}})));
        let shot = std::env::var_os("DATUM_KEEP_SHOT");
        let builder = egui_kittest::Harness::builder().with_size([560.0, 360.0]);
        let builder = if shot.is_some() { builder.wgpu() } else { builder };
        let mut h = builder.build_ui(|ui| v.keep(ui.ctx()));
        h.run();
        h.get_by_label("length l5").click();
        h.run();
        h.get_by_label("perpendicular ? l5 (? a line)");
        h.get_all_by_value("d1").next().unwrap().click();
        h.run();
        h.get_all_by_value("d1").next().unwrap().type_text("x");
        h.run();
        if let Some(path) = shot {
            h.render().unwrap().save(path).unwrap();
        }
        drop(h);
        let e = &v.keeping.as_ref().unwrap().element;
        assert_eq!(e["parameters"], json!([{"name": "d1x", "value": 8.900914}]), "{e}");
        assert_eq!(e["sketch"]["constraints"][1]["value"], "d1x");
        assert!(datum_model::library::check("sketch", e).is_ok());
    }

    /// The library palette: a double click on an element starts placing it, a drag onto the
    /// sketch puts it where it is let go. `DATUM_PALETTE_SHOT=file.png` saves a picture.
    #[test]
    fn the_palette_places_by_double_click_and_drag() {
        let (mut v, inbox) = viewer("symmetric_plate.json");
        let mut session = Session::default();
        let model = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/models/symmetric_plate.json");
        session.execute("open", &json!({"path": model.display().to_string()})).unwrap();
        if let Some(m) = &mut v.model {
            m.editing = Some(("outline".into(), Context { id: session.graph().nodes()[0].id, input: None }));
        }
        v.entering = Some("outline".into());
        v.sync_sketch();
        let shot = std::env::var_os("DATUM_PALETTE_SHOT");
        let ui = |v: &mut Viewer, ui: &mut egui::Ui| {
            egui::Panel::right("datum_library").min_size(200.0).show(ui, |ui| v.palette(ui));
            egui::CentralPanel::default().show(ui, |ui| {
                let (rect, r) = ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());
                let offset = r.hover_pos().map(|p| (f64::from(p.x - rect.center().x), f64::from(p.y - rect.center().y)));
                v.sketch_input(ui, rect, &r, offset);
            });
        };
        {
            // Frames of 1/60 s, as on a screen: kittest's own quarter second is longer than a
            // double click.
            let builder = egui_kittest::Harness::builder().with_size([800.0, 400.0]).with_step_dt(1.0 / 60.0);
            let builder = if shot.is_some() { builder.wgpu() } else { builder };
            let mut h = builder.build_ui(|u| ui(&mut v, u));
            h.run();
            if let Some(path) = &shot {
                h.render().unwrap().save(path).unwrap();
            }
            // A double click: two clicks within egui's double-click time.
            let item = h.get_by_label("Center rectangle");
            item.click();
            item.click();
            h.run();
        }
        assert_eq!(v.sketch.as_ref().unwrap().draft.template.as_ref().map(|t| t["name"].clone()), Some(json!("Center rectangle")));
        {
            let mut h = egui_kittest::Harness::builder().with_size([800.0, 400.0]).build_ui(|u| ui(&mut v, u));
            h.run();
            let from = h.get_by_label("Rectangle").rect().center();
            h.drag_at(from);
            h.run();
            h.hover_at(egui::pos2(300.0, 200.0));
            h.run();
            h.drop_at(egui::pos2(300.0, 200.0));
            h.run();
        }
        let sk = v.sketch.as_ref().unwrap();
        assert_eq!(sk.draft.template.as_ref().map(|t| t["name"].clone()), Some(json!("Rectangle")), "the dragged one is being placed");
        assert_eq!(sk.draft.clicks.len(), 1, "its first corner where it was let go");
        assert!(inbox.try_iter().all(|r| !matches!(r, Request::Command("edit_sketch", _))), "two corners: nothing placed yet");
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
