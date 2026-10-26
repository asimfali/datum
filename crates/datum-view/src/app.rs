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

use crate::actions::{ACTIONS, Act, Action, Keys};
use crate::camera::Camera;
use crate::dims::{Axis, Entry, Field, Pick2, value_of};
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
        // The window as it was left, size and place on the monitor (eframe keeps them); the first
        // time 1280 × 800 restored, opening maximized (see `Viewer::shown`).
        viewport: egui::ViewportBuilder::default().with_title(title(None, path.as_deref())).with_inner_size([1280.0, 800.0]).with_drag_and_drop(true),
        ..Default::default()
    };
    if let egui_wgpu::WgpuSetup::CreateNew(setup) = &mut options.wgpu_options.wgpu_setup {
        setup.instance_descriptor.backends = crate::render::backends();
    }
    eframe::run_native(
        "Datum",
        options,
        Box::new(move |cc| {
            let rs = cc.wgpu_render_state.as_ref().ok_or("the window has no wgpu renderer")?;
            let info = rs.adapter.get_info();
            let adapter = format!("{} ({:?})", info.name, info.backend);
            crate::log::write(&format!("window on {adapter}"));
            let maximized = cc.storage.and_then(|s| s.get_string(MAXIMIZED)).is_none_or(|m| m == "true");
            let kept = |key| cc.storage.and_then(|s| s.get_string(key)).unwrap_or_default();
            let mut viewer = Viewer::new(path, cc.egui_ctx.clone(), adapter);
            viewer.maximized = maximized;
            viewer.recent = serde_json::from_str(&kept(RECENT)).unwrap_or_default();
            viewer.recent_projects = serde_json::from_str(&kept(RECENT_PROJECTS)).unwrap_or_default();
            if let Some(layout) = cc.storage.and_then(|s| s.get_string(LAYOUT)) {
                viewer.dock = crate::dock::from_text(&layout);
            }
            viewer.side_bar = crate::sidebar::SideBar::from_text(&kept(SIDE_BAR));
            // The project left open opens again, if it is still one (D142).
            if let Some(p) = serde_json::from_str::<Option<PathBuf>>(&kept(PROJECT)).ok().flatten().filter(|p| is_project(p)) {
                viewer.set_project(Some(p));
            }
            Ok(Box::new(viewer))
        }),
    )
    .map_err(|e| e.to_string())
}

/// Whether the window was left maximized: kept apart from the window's size and place, which
/// Windows does not restore maximized.
const MAXIMIZED: &str = "datum_maximized";
/// Where the panes are (D161).
const LAYOUT: &str = "datum_layout";
/// The side bar's side, set and sections (D162).
const SIDE_BAR: &str = "datum_side_bar";

/// The models opened or saved last, the latest first: state beside the window's, not a setting.
const RECENT: &str = "datum_recent";
const RECENT_KEPT: usize = 10;

/// The project open, and the projects opened last, the latest first.
const PROJECT: &str = "datum_project";
const RECENT_PROJECTS: &str = "datum_recent_projects";

/// Whether `folder` is a Datum project.
fn is_project(folder: &std::path::Path) -> bool {
    folder.join(datum_model::workspace::PROJECT_FILE).is_file()
}

/// `path` as the latest of `recent`, once, the oldest dropped past [`RECENT_KEPT`].
fn remember(recent: &mut Vec<PathBuf>, path: PathBuf) {
    recent.retain(|p| *p != path);
    recent.insert(0, path);
    recent.truncate(RECENT_KEPT);
}

/// The window's title: the project open and the model's file.
fn title(project: Option<&std::path::Path>, path: Option<&std::path::Path>) -> String {
    let mut t = "Datum".to_string();
    if let Some(name) = project.and_then(std::path::Path::file_name) {
        t += &format!(" — [{}]", name.to_string_lossy());
    }
    if let Some(p) = path {
        t += &format!(" — {}", p.display());
    }
    t
}

enum Request {
    Open(PathBuf),
    /// No model any more: the session starts empty, the window shows none.
    Close,
    SetParameter(String, Json),
    /// Any other command of the session, with its parameters.
    Command(&'static str, Json),
    /// The sketch feature edited in the window, if any: its context comes with every model.
    Editing(Option<String>),
}

/// What a plane or a face is picked for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Purpose {
    Sketch,
    Look,
    /// A work plane offset from it.
    Plane,
}

/// What is under the cursor while picking.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Under {
    /// Origin plane `k` of [`crate::origin::origin_squares`].
    Origin(usize),
    /// Work plane `k` of [`Viewer::work_planes`].
    Work(usize),
    Face(crate::scene::Hit),
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
    /// Closed: the window shows no model.
    closed: bool,
    /// Milliseconds the rebuild and the scene (meshes, vertices) took.
    rebuild_ms: f64,
    scene_ms: f64,
    /// The feature each body of the scene is of.
    bodies: Vec<String>,
    /// The flat blanks of what the level worked in makes, by feature, or why there is none (D164).
    flats: Vec<(String, Result<Arc<datum_ops::Flat>, String>)>,
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
        let (mut error, mut opened, mut closed) = (None, false, false);
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
                    (opened, closed) = (true, false);
                    if let Err(e) = journaled(&mut session, "open", &json!({"path": p.display().to_string()})) {
                        error = Some(e);
                    }
                }
                Request::SetParameter(name, value) => {
                    latest.insert(name, value);
                }
                Request::Close => {
                    latest.clear();
                    session = Session::default();
                    (opened, closed) = (false, true);
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
        let flats = session.flats();
        let (graph, evaluation) = (session.graph(), session.evaluation());
        let shown: Vec<(&str, &datum_graph::Value)> = shown(graph, evaluation, editing.as_deref())
            .into_iter()
            .filter_map(|i| Some((graph.nodes()[i as usize].name.as_str(), evaluation.results[i as usize].as_deref().ok()?)))
            .collect();
        let finals: Vec<&datum_graph::Value> = shown.iter().map(|(_, v)| *v).collect();
        let bodies = shown.iter().filter(|(_, v)| v.body().is_some()).map(|(n, _)| n.to_string()).collect();
        // The rest of the product (D160): in the frame, every part in its chosen variant, whole; in a
        // part, the other parts, faded.
        let mine = session.active().and_then(|a| session.part_of(a));
        let around: Vec<datum_graph::ValueRef> = session
            .product()
            .into_iter()
            .filter(|(part, _, _)| Some(part) != mine.as_ref() && session.visible(part))
            .filter_map(|(_, _, bodies)| bodies.ok())
            .flatten()
            .map(|(_, v)| v)
            .collect();
        let context: Vec<&datum_brep::Body> = around.iter().filter_map(|v| v.body()).collect();
        let scene = Scene::new(&finals, &mut meshes).with_context(&context, session.active().is_some(), &mut meshes);
        meshes.retain(&finals.iter().filter_map(|v| v.body()).chain(context.iter().copied()).collect::<Vec<_>>());
        let context = editing.as_ref().and_then(|name| {
            let node = graph.nodes().iter().position(|n| &n.name == name)?;
            let input = graph.nodes()[node].inputs.first().and_then(|&i| evaluation.results[i as usize].as_ref().ok().cloned());
            Some((name.clone(), Context { id: graph.nodes()[node].id, input }))
        });
        let geometry = Arc::new(Geometry::new(&scene, scene.anchor()));
        let scene_ms = start.elapsed().as_secs_f64() * 1e3;
        let inspect = session.execute("inspect", &json!({})).unwrap_or_default();
        if models
            .send(Model { inspect, scene, geometry, error, notes, opened, closed, rebuild_ms, scene_ms, bodies, flats, editing: context })
            .is_err()
        {
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
    /// The view turning to where the navigation cube or the home view asked.
    turn: Option<crate::cube::Turn>,
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
    /// Parameter definitions as typed, by name; a new one's name and definition.
    edits: BTreeMap<String, String>,
    new_parameter: (String, String),
    /// Work plane offsets as typed in their rows, by feature.
    offsets: BTreeMap<String, String>,
    /// Why the window refused what was asked, before the model was asked.
    local_error: Option<String>,
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
    /// The menu a held right click opened on the sketch (D155), and the hold: since when and
    /// where; whether it opened the menu (then its release repeats nothing).
    menu: Option<Menu>,
    /// The activity bar and side bar: which side, what it shows, folded sections (D162).
    side_bar: crate::sidebar::SideBar,
    /// The panes and where they are (D161); what the flat pattern pane looks at.
    dock: egui_dock::DockState<crate::dock::Pane>,
    flat: crate::flat::FlatView,
    /// A part or variant being named: below which level (none: a part, below the frame), and the
    /// name typed.
    new_level: Option<(Option<String>, String)>,
    hold: Option<(f64, egui::Pos2, bool)>,
    /// Closing the window: once the model is saved (`true`), or at once, its changes dropped.
    quitting: Option<bool>,
    /// The folder of the model last opened or saved: where the next one is opened from.
    worked_in: Option<PathBuf>,
    recent: Vec<PathBuf>,
    /// The project open (its folder): where a new model is saved and the dialogs look while no
    /// model file says otherwise; it stays when a model of another project is opened (D142).
    project: Option<PathBuf>,
    recent_projects: Vec<PathBuf>,
    /// The title the window was last given.
    titled: String,
    /// The keys in force, the settings file they come from, and what in it could not be
    /// applied, shown until dismissed.
    keys: Keys,
    settings: Option<PathBuf>,
    settings_problems: Vec<String>,
    /// Frames drawn so far, up to two: the window is maximized once it is shown (after the
    /// first), if it was left so. Asked of the window as it is made, Windows drops it when eframe
    /// shows the window.
    shown: u8,
    maximized: bool,
    /// Picking a face to sketch on, and the body and face under the cursor.
    /// A plane or a face to be clicked, and what for.
    picking: Option<Purpose>,
    /// What a click would take while picking: an origin plane, a work plane or a face.
    under: Option<Under>,
    /// A work plane being made: its base (the feature's fields) and the offset typed.
    plane_draft: Option<(Json, String)>,
}

impl Viewer {
    fn new(path: Option<PathBuf>, ctx: egui::Context, adapter: String) -> Self {
        let (requests, inbox) = channel();
        let (outbox, models) = channel();
        let repaint = ctx.clone();
        std::thread::spawn(move || worker(inbox, outbox, ctx));
        let geometry = Arc::new(Geometry::new(&Scene::default(), datum_math::Vec3::ZERO));
        let settings = crate::settings::path();
        let (keys, settings_problems) = crate::settings::load(settings.as_deref());
        for p in &settings_problems {
            crate::log::write(&format!("settings: {p}"));
        }
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
            turn: None,
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
            offsets: BTreeMap::new(),
            new_parameter: Default::default(),
            local_error: None,
            path: None,
            update,
            newer: None,
            sketch: None,
            entering: None,
            library: crate::templates::library(None),
            library_filter: String::new(),
            keeping: None,
            menu: None,
            side_bar: crate::sidebar::SideBar::default(),
            dock: crate::dock::default_layout(),
            flat: crate::flat::FlatView::default(),
            new_level: None,
            hold: None,
            quitting: None,
            worked_in: None,
            recent: Vec::new(),
            project: None,
            recent_projects: Vec::new(),
            titled: String::new(),
            keys,
            settings,
            settings_problems,
            shown: 0,
            maximized: false,
            picking: None,
            under: None,
            plane_draft: None,
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
        if let Some(p) = &path {
            remember(&mut self.recent, std::path::absolute(p).unwrap_or_else(|_| p.clone()));
        }
        if let Some(d) = path.as_deref().and_then(std::path::Path::parent) {
            self.worked_in = Some(d.to_path_buf());
        }
        self.path = path;
        self.reload_library();
    }

    /// Opens project `folder`, or none: new models and the dialogs go there.
    fn set_project(&mut self, folder: Option<PathBuf>) {
        if let Some(f) = &folder {
            remember(&mut self.recent_projects, std::path::absolute(f).unwrap_or_else(|_| f.clone()));
        }
        // Open… looks in the project now, not where the last one was worked in.
        self.worked_in = None;
        self.project = folder;
        self.reload_library();
    }

    /// The project the window works in: the model file's own (a model keeps its project's
    /// rules), else, for a model without a file, the project open.
    fn current_project(&self) -> Option<PathBuf> {
        match &self.path {
            Some(p) => datum_model::workspace::project_root(p),
            None => self.project.clone(),
        }
    }

    fn reload_library(&mut self) {
        self.library = crate::templates::library(self.current_project().as_deref());
    }

    /// A dialog for models opened in `folder`.
    fn models_in(folder: Option<PathBuf>) -> rfd::FileDialog {
        let dialog = rfd::FileDialog::new().add_filter("Datum model", &["json"]);
        match folder {
            Some(d) => dialog.set_directory(d),
            None => dialog,
        }
    }

    /// Where a new model is saved by dialog: `part` of the workspace (the project, or the folder
    /// beside the program), made if it is not there yet (D115).
    fn parts(&self) -> rfd::FileDialog {
        Self::models_in(kept_in(self.current_project().as_deref(), "part"))
    }

    /// The dialog to export as `kind` (`dxf`, …): in that folder of the workspace, named after the
    /// model, or after `feature` while the model has no file (D133).
    fn exported(&self, kind: &str, feature: &str) -> rfd::FileDialog {
        let stem = self.path.as_deref().and_then(std::path::Path::file_stem).map_or_else(|| feature.to_string(), |s| s.to_string_lossy().to_string());
        let dialog = rfd::FileDialog::new().add_filter(kind.to_uppercase(), &[kind]).set_file_name(format!("{stem}.{kind}"));
        match kept_in(self.current_project().as_deref(), kind) {
            Some(d) => dialog.set_directory(d),
            None => dialog,
        }
    }

    /// Exports the model as STEP by dialog. Geometry STEP has no exact type for is written as
    /// B-splines only if the engineer says yes (D147); no, and nothing is written.
    fn export_step(&mut self) {
        let inexact: Vec<String> = self.model.as_ref().map_or_else(Vec::new, |m| {
            m.inspect["step_inexact"].as_array().into_iter().flatten().filter_map(|f| f.as_str().map(|f| format!("`{f}`"))).collect()
        });
        let approximate = !inexact.is_empty();
        if approximate {
            let question = format!(
                "{} cut(s) through a rolled sheet along circles or oblique lines of its development: STEP has no exact type for them.\n\n\
                 Write them as B-splines within {:e} mm? The receiving CAD cannot tell them from exact, and a note beside the file \
                 (.step.txt) says what was approximated.\n\nNo: nothing is written.",
                inexact.join(", "),
                datum_brep::DEVIATION
            );
            let answer = rfd::MessageDialog::new().set_title("Datum").set_description(question).set_buttons(rfd::MessageButtons::YesNo).show();
            if answer != rfd::MessageDialogResult::Yes {
                return;
            }
        }
        if let Some(p) = self.exported("step", "part").save_file() {
            self.send(Request::Command("export_step", json!({"path": p.display().to_string(), "approximate": approximate})));
        }
    }

    /// Where a model is opened, or saved again elsewhere, by dialog: where the engineer works —
    /// the folder of the model open, or of the last one opened or saved; else `part` (D121).
    fn where_working(&self) -> rfd::FileDialog {
        match self.working_folder() {
            Some(d) => Self::models_in(Some(d)),
            None => self.parts(),
        }
    }

    /// The folder of the model open, or of the last one opened or saved.
    fn working_folder(&self) -> Option<PathBuf> {
        self.path.as_deref().and_then(std::path::Path::parent).map(std::path::Path::to_path_buf).or_else(|| self.worked_in.clone())
    }

    /// Opens a model chosen in the file dialog (Open…, Ctrl+O).
    fn open_by_dialog(&mut self) {
        if !self.may_leave() {
            return;
        }
        if let Some(p) = self.where_working().pick_file() {
            self.open(p);
        }
    }

    /// A recent model chosen: opened once what is not saved is asked about.
    fn open_recent(&mut self, path: PathBuf) {
        if still_there(&mut self.recent, &path, path.exists(), "recent files") && self.may_leave() {
            self.open(path);
        }
    }

    /// A project chosen in the dialog: a new one in any folder (an empty one, made there), or
    /// one to open; a folder that is not a project yet is asked about.
    fn project_by_dialog(&mut self, new: bool) {
        let what = if new { "New project: a folder for it" } else { "Open project" };
        let start = self.project.as_deref().and_then(std::path::Path::parent).map(std::path::Path::to_path_buf);
        let dialog = rfd::FileDialog::new().set_title(what);
        let dialog = match start {
            Some(d) => dialog.set_directory(d),
            None => dialog,
        };
        if let Some(folder) = dialog.pick_folder() {
            self.open_project(folder, !new);
        }
    }

    /// Opens project `folder`; one without a project file becomes a project, asked first if `ask`.
    fn open_project(&mut self, folder: PathBuf, ask: bool) {
        if !is_project(&folder) {
            let question = format!("{} is not a Datum project yet. Make it one?", folder.display());
            if ask
                && rfd::MessageDialog::new().set_title("Datum").set_description(question).set_buttons(rfd::MessageButtons::YesNo).show()
                    != rfd::MessageDialogResult::Yes
            {
                return;
            }
            if let Err(e) = std::fs::write(folder.join(datum_model::workspace::PROJECT_FILE), "{}\n") {
                rfd::MessageDialog::new().set_title("Datum").set_description(format!("{}: {e}", folder.display())).show();
                return;
            }
        }
        self.set_project(Some(folder));
    }

    /// A recent project chosen: one that is no longer a project leaves the list, saying so.
    fn open_recent_project(&mut self, folder: PathBuf) {
        if still_there(&mut self.recent_projects, &folder, is_project(&folder), "recent projects") {
            self.set_project(Some(folder));
        }
    }

    /// Saves the model: to its file, or (`ask`, or none yet) to one chosen.
    fn save(&mut self, ask: bool) -> bool {
        let path = match (&self.path, ask) {
            (Some(p), false) => Some(p.clone()),
            (Some(p), true) => {
                let name = p.file_name().map_or_else(|| "model.json".to_string(), |n| n.to_string_lossy().to_string());
                self.where_working().set_file_name(name).save_file()
            }
            (None, _) => self.parts().set_file_name("model.json").save_file(),
        };
        let Some(p) = path else { return false };
        self.send(Request::Command("save", json!({"path": p.display().to_string()})));
        self.set_path(Some(p));
        true
    }

    /// Whether the model is changed since it was opened or saved.
    fn modified(&self) -> bool {
        self.model.as_ref().is_some_and(|m| m.inspect["modified"] == true)
    }

    /// Whether the model may be left (closed, replaced): asked first if it has changes not saved
    /// — saved, dropped, or kept open.
    fn may_leave(&mut self) -> bool {
        !self.modified() || self.ask_to_save().is_some()
    }

    /// Asks whether to save the changes: `Some(true)` saved (sent to be written), `Some(false)`
    /// dropped, `None` kept open.
    fn ask_to_save(&mut self) -> Option<bool> {
        let file = self.path.as_ref().and_then(|p| p.file_name()).map_or_else(|| "the new model".to_string(), |f| f.to_string_lossy().to_string());
        let answer = rfd::MessageDialog::new()
            .set_title("Datum")
            .set_description(format!("Save the changes to {file}?"))
            .set_buttons(rfd::MessageButtons::YesNoCancel)
            .show();
        match answer {
            rfd::MessageDialogResult::Yes => self.save(false).then_some(true),
            rfd::MessageDialogResult::No => Some(false),
            _ => None,
        }
    }

    /// An empty model (New, Ctrl+N), once what is not saved is asked about.
    fn new_model(&mut self) {
        if !self.may_leave() {
            return;
        }
        self.leave();
        self.set_path(None);
        self.send(Request::Command("new", json!({})));
    }

    /// Closes the model (Close, Ctrl+W): the window shows none.
    fn close(&mut self) {
        if self.model.is_none() || !self.may_leave() {
            return;
        }
        self.leave();
        self.set_path(None);
        self.send(Request::Close);
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
                // Two of a name: which file each is.
                let twin = self.library.iter().filter(|o| o.entry.name() == name).count() > 1;
                let file = item.entry.file.as_ref().and_then(|f| f.file_name()).map(|f| f.to_string_lossy().to_string()).filter(|_| twin);
                let from = match file {
                    Some(f) => format!("{} · {f}", crate::templates::source(&item.entry)),
                    None => crate::templates::source(&item.entry).to_string(),
                };
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
        if self.keeping.is_none() {
            return;
        }
        let project = self.current_project();
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
                        if i == 1
                            && anchors.len() > 1
                            && crate::icons::button(
                                ui,
                                crate::icons::Icon::Remove,
                                "No second point",
                                "No second point: the element is moved as it is",
                                true,
                            )
                            .clicked()
                        {
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
                        if crate::icons::button(ui, crate::icons::Icon::Up, "Up", "Ask it earlier", i > 0).clicked() {
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
            let folder = kept_in(project.as_deref(), "library").map(|l| l.join("sketch"));
            let mut dialog = rfd::FileDialog::new().add_filter("Datum element", &["json"]).set_file_name(datum_model::library::file_name(&name));
            if let Some(f) = folder.filter(|f| std::fs::create_dir_all(f).is_ok()) {
                dialog = dialog.set_directory(f);
            }
            let written = datum_model::library::check("sketch", &element).and_then(|()| match dialog.save_file() {
                Some(path) => {
                    // A file named otherwise in the dialog names the element: the palette shows
                    // what the folder does.
                    let stem = path.file_stem().map(|s| s.to_string_lossy().to_string()).unwrap_or_default();
                    if !stem.is_empty() && datum_model::library::file_name(&name) != datum_model::library::file_name(&stem) {
                        element["name"] = json!(stem);
                    }
                    serde_json::to_string_pretty(&element)
                        .map_err(|e| e.to_string())
                        .and_then(|t| std::fs::write(&path, t).map_err(|e| format!("{}: {e}", path.display())))
                        .map(|()| true)
                }
                None => Ok(false),
            });
            match written {
                Ok(true) => {
                    close = true;
                    self.reload_library();
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
        self.picking = None;
        self.sketch = None;
        self.send(Request::Editing(Some(name.clone())));
        self.entering = Some(name);
    }

    fn leave(&mut self) {
        self.menu = None;
        self.side_bar.container = crate::sidebar::Container::Model;
        if self.sketch.is_some() || self.entering.is_some() {
            self.sketch = None;
            self.entering = None;
            let _ = self.requests.send(Request::Editing(None));
        }
    }

    /// A new sketch feature from `fields` (its plane or face), edited at once.
    /// S, as in Inventor: a sketch on an origin plane or a plane face, to be clicked; XY is
    /// picked from the start, Enter or a right click takes it.
    fn start_sketch(&mut self) {
        if self.model.is_none() {
            // No view to pick in yet: XY at once.
            self.new_sketch(json!({}));
        } else {
            self.pick(Purpose::Sketch);
        }
    }

    /// Starts picking a plane or a face for `purpose`, or stops if already picking for it.
    fn pick(&mut self, purpose: Purpose) {
        self.picking = if self.picking == Some(purpose) { None } else { Some(purpose) };
        self.under = None;
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
                    // Its sections in the side bar while it is edited (D162).
                    self.side_bar.container = crate::sidebar::Container::Sketch;
                    self.side_bar.visible = true;
                    self.entering = None;
                }
            },
            // Still on its way, or gone (undone, or the model replaced).
            (_, Some(_)) if self.sketch.is_none() => {}
            _ => self.leave(),
        }
    }

    /// The menu and the toolbar in one row (the bar takes the whole width): the menu is narrow,
    /// the model gets the height.
    fn top_row(&mut self, ui: &mut egui::Ui) {
        egui::MenuBar::new().ui(ui, |ui| {
            self.menu(ui);
            ui.separator();
            self.toolbar(ui);
        });
    }

    /// The toolbar beside the menu; a `.json` dropped on the window opens.
    fn toolbar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            if let Ok(n) = self.update.try_recv() {
                self.newer = Some(n);
            }
            if let Some((version, url)) = &self.newer {
                ui.hyperlink_to(format!("⬆ Datum {version} is out"), url).on_hover_text("Its page: what is new, and the download");
                ui.separator();
            }
            use crate::icons::{Icon, button};
            for (act, icon) in [(Act::Undo, Icon::Undo), (Act::Redo, Icon::Redo)] {
                let a = act.action();
                if button(ui, icon, a.label, &self.keys.hint(ui.ctx(), act), self.available(a)).clicked() {
                    self.run(ui, act);
                }
            }
            ui.separator();
            for act in [Act::NewSketch, Act::WorkPlane, Act::LookAt] {
                self.item(ui, act, false);
            }
        });
        let dropped = ui.input(|i| i.raw.dropped_files.iter().map(|f| f.path().to_path_buf()).find(|p| p.extension().is_some_and(|e| e == "json")));
        if let Some(p) = dropped {
            self.open(p);
        }
    }

    fn receive(&mut self) {
        while let Ok(m) = self.models.try_recv() {
            self.busy = false;
            // Fitted when it arrives, not when asked for: until then the view holds the last model.
            // Again when it becomes empty or stops being so: a new model, its first sketch.
            let empty = |m: &Model| m.scene.bounds.is_none();
            self.fitted &= !m.opened && self.model.as_ref().map(empty) == Some(empty(&m));
            self.geometry = m.geometry.clone();
            self.version += 1;
            if let Some(p) = m.inspect["parameters"].as_object() {
                self.edits = p.iter().map(|(n, v)| (n.clone(), definition(&v["definition"]))).collect();
            }
            self.offsets = offsets(&m.inspect);
            // A save that failed while closing the window: it stays open, the error shown.
            if self.quitting == Some(true) && m.error.is_some() {
                self.quitting = None;
            }
            self.model = if m.closed { None } else { Some(m) };
            self.sync_sketch();
        }
    }

    /// The frame and its parts (D152), each part a list of itself and its variants, one chosen:
    /// a click works in it; a new part or variant is named here.
    fn levels_panel(&mut self, ui: &mut egui::Ui) {
        let Some(m) = &self.model else { return };
        let active = m.inspect["level"].as_str().map(String::from);
        // (name, parent, depth), depth first: a part, then its variants.
        let levels: Vec<(String, Option<String>, usize, Option<String>)> = m.inspect["levels"]
            .as_array()
            .into_iter()
            .flatten()
            .map(|l| {
                let s = |k: &str| l[k].as_str().map(String::from);
                (s("name").unwrap_or_default(), s("parent"), l["depth"].as_u64().unwrap_or(0) as usize, s("error"))
            })
            .collect();
        // The variant of each part in the product: the model's (D160).
        let chosen_of = |part: &str| m.inspect["chosen"][part].as_str().unwrap_or(part).to_string();
        let mut go: Option<Option<String>> = None;
        let mut choose: Option<String> = None;
        let mut show: Option<(String, bool)> = None;
        let mut remove: Option<String> = None;
        let frame = "The frame: the shared sketches, planes and parameters every part and variant takes";
        if ui.selectable_label(active.is_none(), "Frame").on_hover_text(frame).clicked() {
            go = Some(None);
        }
        for (k, (part, _, depth, _)) in levels.iter().enumerate() {
            if *depth > 0 {
                continue;
            }
            // The part and what is below it, up to the next part.
            let family: Vec<&(String, Option<String>, usize, Option<String>)> =
                std::iter::once(&levels[k]).chain(levels[k + 1..].iter().take_while(|l| l.2 > 0)).collect();
            let chosen = chosen_of(part);
            let here = family.iter().any(|l| Some(&l.0) == active.as_ref());
            ui.horizontal(|ui| {
                let mut visible = !m.inspect["hidden"].as_array().is_some_and(|h| h.iter().any(|p| p == part.as_str()));
                if ui.checkbox(&mut visible, "").on_hover_text("Shown in the product: in the frame, and faint around another part").changed() {
                    show = Some((part.clone(), visible));
                }
                if ui.selectable_label(here, part.as_str()).on_hover_text("Work in this part, in the variant chosen beside it").clicked() {
                    go = Some(Some(chosen.clone()));
                }
                let shown = if &chosen == part { "shared".to_string() } else { chosen.clone() };
                egui::ComboBox::from_id_salt(("datum_part", part)).selected_text(shown).show_ui(ui, |ui| {
                    for (n, _, d, err) in &family {
                        let label = if n == part { format!("{part} (shared)") } else { format!("{}{n}", "   ".repeat(*d)) };
                        let r = ui.selectable_label(&chosen == n, label);
                        let r = match err {
                            Some(e) => r.on_hover_text(format!("✖ {e}")),
                            None => r,
                        };
                        if r.clicked() {
                            choose = Some(n.clone());
                            go = Some(Some(n.clone()));
                        }
                    }
                    ui.separator();
                    if ui.button("+ New variant").on_hover_text(format!("Below `{chosen}`: it differs only where you change it")).clicked() {
                        self.new_level = Some((Some(chosen.clone()), fresh(&levels, "Variant")));
                    }
                });
                if here
                    && let Some(a) = &active
                    && crate::icons::button(ui, crate::icons::Icon::Remove, "Remove", &format!("Remove `{a}` (if nothing is below it)"), true)
                        .clicked()
                {
                    remove = Some(a.clone());
                }
            });
            for (n, _, _, err) in &family {
                if let Some(e) = err {
                    ui.colored_label(egui::Color32::from_rgb(190, 40, 40), format!("✖ {n}: {e}"));
                }
            }
        }
        let mut add: Option<(Option<String>, String)> = None;
        match &mut self.new_level {
            Some((parent, name)) => {
                ui.horizontal(|ui| {
                    ui.label(match parent {
                        Some(p) => format!("New variant of {p}:"),
                        None => "New part:".to_string(),
                    });
                    let r = ui.add(egui::TextEdit::singleline(name).desired_width(120.0));
                    r.request_focus();
                    let enter = r.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
                    if (ui.button("Create").clicked() || enter) && !name.trim().is_empty() {
                        add = Some((parent.clone(), name.trim().to_string()));
                    }
                    if ui.button("Cancel").clicked() || ui.input(|i| i.key_pressed(egui::Key::Escape)) {
                        add = Some((None, String::new()));
                    }
                });
            }
            None => {
                if ui.button("+ Part").on_hover_text("A part below the frame: its own features on the frame's sketches").clicked() {
                    self.new_level = Some((None, fresh(&levels, "Part")));
                }
            }
        }
        if let Some((parent, name)) = add {
            self.new_level = None;
            if !name.is_empty() {
                // A new variant is the one its part shows in the product.
                let variant = parent.is_some();
                self.send(Request::Command("add_level", json!({"name": name, "parent": parent})));
                if variant {
                    self.send(Request::Command("choose", json!({ "level": name })));
                }
            }
        }
        if let Some(level) = choose {
            self.send(Request::Command("choose", json!({ "level": level })));
        }
        if let Some((part, visible)) = show {
            self.send(Request::Command("show", json!({ "part": part, "visible": visible })));
        }
        if let Some(level) = go
            && level != active
        {
            self.send(Request::Command("activate", json!({ "level": level })));
        }
        if let Some(level) = remove {
            self.send(Request::Command("remove_level", json!({ "name": level })));
        }
    }

    /// The side bar (D162): the model's state, then the sections of the set its icon chose, one
    /// under another.
    fn side(&mut self, ui: &mut egui::Ui) {
        use crate::sidebar::{BORDER, HEADER, Section, header};
        self.status(ui);
        ui.separator();
        let list = self.side_bar.container.sections();
        let open: Vec<Section> = list.iter().copied().filter(|s| self.side_bar.is_open(*s)).collect();
        let fixed = HEADER * list.len() as f32 + BORDER * open.len().saturating_sub(1) as f32;
        let body = (ui.available_height() - fixed - 4.0).max(0.0);
        let total: f32 = open.iter().map(|s| self.side_bar.weight(*s)).sum::<f32>().max(1e-6);
        for s in list {
            let hint = match s {
                Section::Parts => LEVELS,
                Section::Parameters => FORMULAS,
                _ => "",
            };
            if header(ui, *s, self.side_bar.is_open(*s), hint) {
                self.side_bar.toggle(*s);
            }
            if !self.side_bar.is_open(*s) {
                continue;
            }
            let h = body * self.side_bar.weight(*s) / total;
            let w = ui.available_width();
            ui.allocate_ui_with_layout(egui::vec2(w, h), egui::Layout::top_down(egui::Align::Min), |ui| {
                ui.set_min_size(egui::vec2(w, h));
                egui::ScrollArea::vertical().id_salt(("datum_section", *s)).auto_shrink([false, false]).max_height(h).show(ui, |ui| match s {
                    Section::Parts => {
                        let editing = self.sketch.is_some();
                        ui.add_enabled_ui(!editing, |ui| self.levels_panel(ui)).response.on_disabled_hover_text("Done with the sketch first");
                    }
                    Section::Parameters => self.parameters_section(ui),
                    Section::Features => self.features_section(ui),
                    Section::Sketch if self.sketch.is_some() => self.sketch_section(ui),
                    Section::Sketch => {
                        ui.weak("No sketch open: S starts one, the pencil of a sketch feature edits it.");
                    }
                    Section::Library => self.palette(ui),
                });
            });
            // The border to the next open section, dragged.
            if let Some(next) = open.iter().skip_while(|o| *o != s).nth(1) {
                let (rect, r) = ui.allocate_exact_size(egui::vec2(w, BORDER), egui::Sense::drag());
                let r = r.on_hover_cursor(egui::CursorIcon::ResizeVertical);
                if r.hovered() || r.dragged() {
                    ui.painter().hline(rect.x_range(), rect.center().y, ui.visuals().widgets.hovered.fg_stroke);
                }
                if r.dragged() {
                    self.side_bar.drag(*s, *next, r.drag_delta().y, body);
                }
            }
        }
    }

    /// The activity bar (D162): an icon for each set of sections; clicked, the side bar shows it,
    /// or hides if it shows it already.
    fn activity(&mut self, ui: &mut egui::Ui) {
        use crate::sidebar::Container;
        ui.add_space(6.0);
        for c in Container::ALL {
            let (rect, r) = ui.allocate_exact_size(egui::vec2(ui.available_width(), 40.0), egui::Sense::click());
            let on = self.side_bar.visible && self.side_bar.container == c;
            let visuals = ui.visuals();
            let colour = if on || r.hovered() { visuals.strong_text_color() } else { visuals.weak_text_color() };
            crate::sidebar::icon(ui.painter(), rect.center(), c, colour);
            if on {
                // The mark of the set shown, at the edge toward the side bar.
                let x = if self.side_bar.right { rect.left() + 1.0 } else { rect.right() - 1.0 };
                ui.painter().vline(x, rect.y_range(), egui::Stroke::new(2.0, visuals.strong_text_color()));
            }
            if r.on_hover_text(c.title()).clicked() {
                self.side_bar.click(c);
            }
        }
    }

    /// The model's state over the side bar: the version, the journal of actions, the last error
    /// and what the last commands changed beyond what they asked.
    fn status(&mut self, ui: &mut egui::Ui) {
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
    }

    /// The sketch being edited: its state, tools, constraints, groups, Done.
    fn sketch_section(&mut self, ui: &mut egui::Ui) {
        let Some(m) = &self.model else { return };
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
                for act in TOOLS {
                    let (a, tool) = (act.action(), tool(act));
                    if ui.selectable_label(sk.draft.tool == tool, a.label).on_hover_text(self.keys.hint(ui.ctx(), act)).clicked() {
                        sk.draft.press(tool);
                    }
                }
            });
            // Geometric constraints by hand: a kind, then a click on each thing it ties (D118).
            ui.horizontal_wrapped(|ui| {
                ui.label("Constrain");
                for (kind, glyph, name, what) in crate::constrain::KINDS {
                    let on = sk.draft.tool == Tool::Constrain(kind);
                    let r = crate::icons::tile(ui, name, &format!("{name}: {what}"), on, true, |p, c, colour| {
                        crate::sketch::paint_glyph(p, c, glyph, colour)
                    });
                    if r.clicked() {
                        sk.draft.press(Tool::Constrain(kind));
                    }
                }
            });
            // The library is the palette on the right; a selection goes into it from here.
            ui.horizontal_wrapped(|ui| {
                let hint = "The selection as a library element: you choose where it is put, which dimensions it asks and what it ties to outside";
                if ui.add_enabled(!sk.selected.is_empty(), egui::Button::new("To library…")).on_hover_text(hint).clicked() {
                    self.keeping = sk.extract("New element", Some(&sk.params)).map(Keep::new);
                }
                // Groups: a name for elements together (a pattern's cell, a path); its badge
                // marks them, a click on it selects them.
                match sk.selected_group() {
                    Some(g) => {
                        if ui.button("Ungroup").on_hover_text(format!("Take group `{g}` apart: its elements stay")).clicked() {
                            sent.push(Request::Command("edit_sketch", sk.ungroup(&g)));
                        }
                    }
                    None => {
                        let hint = "The selected points and curves as a group: its badge marks them, a click on it selects them";
                        if ui.add_enabled(!sk.selected.is_empty(), egui::Button::new("Group")).on_hover_text(hint).clicked() {
                            sent.extend(sk.group_selection().map(|p| Request::Command("edit_sketch", p)));
                        }
                    }
                }
            });
            ui.horizontal(|ui| {
                ui.checkbox(&mut sk.draft.construction, "Construction");
                ui.checkbox(&mut sk.names, "Names");
                ui.checkbox(&mut sk.constraints, "Constraints")
                    .on_hover_text("Badges of the geometric constraints: click one to select it, Del removes it");
                if sk.draft.tool == Tool::Line && sk.draft.chain.as_ref().is_some_and(|c| c.last.is_some()) {
                    ui.checkbox(&mut sk.draft.tangent, self.keys.labelled(ui.ctx(), Act::Tangent));
                }
                // The dimension tool's sub-mode, by the mouse too.
                if sk.draft.tool == Tool::Dimension {
                    for (axis, label, hint) in [
                        (Axis::Horizontal, "Horizontal (H)", "A line's ends or two points: their horizontal distance"),
                        (Axis::Vertical, "Vertical (V)", "A line's ends or two points: their vertical distance"),
                    ] {
                        if ui.selectable_label(sk.draft.axis == Some(axis), label).on_hover_text(hint).clicked() {
                            sk.toggle_axis(axis);
                        }
                    }
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
                Tool::Select => "Drag a blue point, a curve or the selection · double-click a dimension to change it · click to select, Shift or Ctrl to add or take away · frame: to the right what is inside, to the left what it touches (Shift or Ctrl: added) · Ctrl+C, Ctrl+V · Del removes · Esc clears, then leaves · Enter or a right click: the last command again · the right button held half a second: the menu — a pattern by what is selected (a point or circle: around it, one or two lines: along them, a group: along or filling it, nothing: the axes), or the copies selected shifted, left out, their pattern edited · a pattern's badge (squares): a click selects its copies, the right button held on it edits the pattern",
                Tool::Dimension => "Click a line, arc or circle and type the value, Enter · then a point or a parallel line: the distance to it, another line: the angle · an arc's two ends: its length, the arc again: its angle · H, V: horizontal, vertical · two points: then click where it goes",
                Tool::Constrain(_) => "Click what the constraint ties, one by one · it is added once enough is picked · Esc or a right click ends",
                Tool::Template => {
                    "Click where each anchor goes (a rectangle: a corner, then the other; from its centre: the centre, then a corner) · \
                     parameters in the field: Tab to the next, a preset beside it, clicks size what is not typed · Enter puts it at the cursor · Esc or a right click ends"
                }
                _ => "Click to place · type a number for the length (diameter, radius), Enter · snaps to points, middles, curves and the face's edges · Shift: no snaps · Esc, a right click or the tool's button again ends the command",
            });
            done = ui.button("Done").clicked();
            ui.separator();
        }
        if done {
            self.leave();
        }
        for r in sent {
            self.send(r);
        }
    }

    /// The parameters of the level worked in: changed in place, added, removed or back as the
    /// parent has them.
    fn parameters_section(&mut self, ui: &mut egui::Ui) {
        let Some(m) = &self.model else { return };
        let level = m.inspect["level"].as_str();
        let (changed, own, set, own_parameters) = marks(&m.inspect);
        let mut revert: Option<Json> = None;
        let _ = (&changed, &own);
        let params = m.inspect["parameters"].as_object().cloned().unwrap_or_default();
        let (mut send, mut removed, mut added) = (Vec::new(), None, false);
        let field = |ui: &mut egui::Ui, text: &mut String, hint: &str| {
            ui.add_sized([170.0, ui.spacing().interact_size.y], egui::TextEdit::singleline(text).hint_text(hint))
        };
        egui::Grid::new("parameters").num_columns(4).striped(true).show(ui, |ui| {
            for (name, p) in &params {
                ui.label(name);
                let text = self.edits.entry(name.clone()).or_default();
                // A grid cell is as wide as its column was: sized explicitly, the field widens it.
                let r = field(ui, text, "");
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
                // In a level: one it sets otherwise goes back to the parent's; one it inherits stays.
                if set.contains(name) {
                    if ui.button("↺").on_hover_text("Set in this variant: back to the parent's value").clicked() {
                        revert = Some(json!({"parameter": name}));
                    }
                } else if (level.is_none() || own_parameters.contains(name))
                    && crate::icons::button(
                        ui,
                        crate::icons::Icon::Remove,
                        "Remove the parameter",
                        "Remove the parameter: what uses it then fails and says so",
                        true,
                    )
                    .clicked()
                {
                    removed = Some(name.clone());
                } else if level.is_some() && !own_parameters.contains(name) {
                    ui.weak("·").on_hover_text("From the parent: type another value to set it in this variant");
                }
                ui.end_row();
            }
            // A new parameter: its name, then its value; Enter in the value or Add adds it.
            let (name, value) = &mut self.new_parameter;
            ui.add_sized([60.0, ui.spacing().interact_size.y], egui::TextEdit::singleline(name).hint_text("name"));
            let r = field(ui, value, "value or formula");
            let enter = r.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
            added = (ui.button("Add").on_hover_text(FORMULAS).clicked() || enter) && !name.trim().is_empty();
            ui.end_row();
        });
        ui.weak("A value may be a formula: w / 2 - gap. Sketch dimensions and depths take parameters too").on_hover_text(FORMULAS);
        if added {
            let (name, value) = std::mem::take(&mut self.new_parameter);
            let name = name.trim().to_string();
            if params.contains_key(&name) {
                self.local_error = Some(format!("`{name}` is a parameter already: change its value in its row"));
                self.new_parameter = (name, value);
            } else {
                self.local_error = None;
                send.push((name, if value.trim().is_empty() { "0".into() } else { value }));
            }
        }
        if let Some(e) = &self.local_error {
            ui.colored_label(egui::Color32::from_rgb(190, 40, 40), e);
        }
        if let Some(name) = removed {
            let _ = self.requests.send(Request::Command("remove_parameter", json!({"name": name})));
            self.busy = true;
        }
        for (name, text) in send {
            let _ = self.requests.send(Request::SetParameter(name, typed(&text)));
            self.busy = true;
        }
        if let Some(r) = revert {
            self.send(Request::Command("revert", r));
        }
    }

    /// The features of the level worked in: their state, on or off, what to do with each.
    fn features_section(&mut self, ui: &mut egui::Ui) {
        let Some(m) = &self.model else { return };
        let (changed, own, set, own_parameters) = marks(&m.inspect);
        let mut revert: Option<Json> = None;
        let _ = (&set, &own_parameters);
        let (mut export, mut edit, mut enter) = (None, None, None);
        let mut toggle: Option<(String, Json)> = None;
        let mut show_flat: Option<String> = None;
        // Out of `self` while the rows borrow it.
        let mut offsets = std::mem::take(&mut self.offsets);
        egui::ScrollArea::vertical().show(ui, |ui| {
            for f in m.inspect["features"].as_array().into_iter().flatten() {
                let bare = f["name"].as_str().unwrap_or_default();
                let op = f["op"].as_str().unwrap_or_default();
                // In a level: what it changes of its parent's (a dot, ↺ back), what is its own (+).
                let (is_changed, is_own) = (changed.iter().any(|c| c == bare), own.iter().any(|c| c == bare));
                let name = bare;
                let mut row = |ui: &mut egui::Ui| match f["error"].as_str() {
                    // A flat blank: its size, and its DXF for the turret press.
                    None if f["status"]["flat"].is_object() => {
                        let flat = &f["status"]["flat"];
                        ui.horizontal(|ui| {
                            let (w, h) = (flat["width"].as_f64().unwrap_or(0.0), flat["height"].as_f64().unwrap_or(0.0));
                            ui.label(format!("✔ {name}  ·  {op}  ·  {w:.1} × {h:.1}"));
                            if ui.button("Show").on_hover_text("In the flat pattern pane").clicked() {
                                show_flat = Some(bare.to_string());
                            }
                            if ui.button("DXF…").clicked()
                                && let Some(p) = self.exported("dxf", name).save_file()
                            {
                                export = Some((bare.to_string(), p));
                            }
                        });
                    }
                    // Sheet metal: the side of the path its thickness lies on, flipped by a click.
                    None if op == "sheet" => {
                        ui.horizontal(|ui| {
                            ui.label(format!("✔ {name}  ·  {op}"));
                            let side = serde_json::from_value::<datum_ops::Side>(f["side"].clone()).unwrap_or(datum_ops::Side::Left);
                            let label = serde_json::to_value(side).ok().and_then(|v| v.as_str().map(str::to_string)).unwrap_or_default();
                            if ui.button(format!("Side: {label}")).on_hover_text("Thickness side of the path: left, right, middle").clicked() {
                                edit = Some((bare.to_string(), json!({"side": side.next()})));
                            }
                        });
                    }
                    // A work plane: what it is offset from, and the offset, changed in place.
                    None if op == "plane" => {
                        ui.horizontal(|ui| {
                            let from = match (f["inputs"][0].as_str(), f["face"].is_string()) {
                                (Some(body), true) => format!("a face of {body}"),
                                (Some(plane), false) => plane.to_string(),
                                (None, _) => f["base"].as_str().unwrap_or("xy").to_uppercase(),
                            };
                            ui.label(format!("✔ {name}  ·  {from} +"));
                            let text = offsets.entry(bare.to_string()).or_default();
                            let r = ui.add(egui::TextEdit::singleline(text).desired_width(90.0).hint_text("offset")).on_hover_text(FORMULAS);
                            // Applied when the field is left changed, as a parameter's value is.
                            if r.lost_focus() && *text != definition(&f["offset"]) {
                                edit = Some((bare.to_string(), json!({"offset": typed(text)})));
                            }
                        });
                    }
                    None if op == "sketch" => {
                        ui.horizontal(|ui| {
                            ui.label(format!("✔ {name}  ·  {op}"));
                            if crate::icons::button(ui, crate::icons::Icon::Edit, "Edit the sketch", "Edit the sketch", true).clicked() {
                                enter = Some(bare.to_string());
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
                };
                ui.horizontal(|ui| {
                    if is_changed && ui.button("↺").on_hover_text("Changed in this variant: back as the parent has it").clicked() {
                        revert = Some(json!({"feature": bare}));
                    }
                    // On or off (D159); by a formula, it says which and is changed in the JSON.
                    match &f["enabled"] {
                        Json::String(formula) => {
                            ui.weak("ƒ").on_hover_text(format!("On while {formula} is not 0"));
                        }
                        e => {
                            let mut on = e.as_f64().is_none_or(|v| v != 0.0);
                            let hint = "Off: the feature passes its input on unchanged — a variant does without a parent's feature so";
                            if ui.checkbox(&mut on, "").on_hover_text(hint).changed() {
                                toggle = Some((bare.to_string(), json!({ "enabled": on })));
                            }
                        }
                    }
                    // Drawn, not a glyph: the window's font may lack one.
                    let (r, painter) = ui.allocate_painter(egui::vec2(10.0, 10.0), egui::Sense::hover());
                    let colour = ui.visuals().widgets.active.fg_stroke.color;
                    if is_changed {
                        painter.circle_filled(r.rect.center(), 3.5, colour);
                        r.on_hover_text("Changed in this variant");
                    } else if is_own {
                        let c = r.rect.center();
                        let stroke = egui::Stroke::new(1.5, colour);
                        painter.line_segment([c - egui::vec2(4.0, 0.0), c + egui::vec2(4.0, 0.0)], stroke);
                        painter.line_segment([c - egui::vec2(0.0, 4.0), c + egui::vec2(0.0, 4.0)], stroke);
                        r.on_hover_text("This variant's own");
                    }
                    ui.vertical(|ui| row(ui));
                });
            }
        });
        self.offsets = offsets;
        let edit = edit.or(toggle);
        if let Some(f) = show_flat {
            self.flat.chosen = Some(f);
            crate::dock::open(&mut self.dock, crate::dock::Pane::Flat);
        }
        if let Some(r) = revert {
            self.send(Request::Command("revert", r));
        }
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
    }

    fn viewport(&mut self, ui: &mut egui::Ui, rs: &egui_wgpu::RenderState) {
        let (rect, r) = ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());
        let aspect = f64::from(rect.width() / rect.height().max(1.0));
        let offset = r.hover_pos().or(r.interact_pointer_pos()).map(|p| (f64::from(p.x - rect.center().x), f64::from(p.y - rect.center().y)));
        if self.sketch.is_some() {
            self.sketch_input(ui, rect, &r, offset);
            self.sketch_menu(ui.ctx());
        } else {
            self.view_input(ui, rect, &r);
        }
        self.pick_input(ui, rect, &r, offset);
        let Some(m) = &self.model else { return };
        let (lo, hi) = crate::origin::held(m.scene.bounds);
        let radius = (hi - lo).dot(hi - lo).sqrt() * 0.5;
        let scene = m.scene.bounds.map(|_| (m.scene.anchor(), radius));
        // A model just opened is fitted once, where the view's shape is known; an empty one to
        // the origin planes, until something is in it.
        if r.double_clicked() || !self.fitted {
            match &mut self.sketch {
                // On a dimension, a double click changes it; elsewhere it fits the view.
                Some(sk) if r.double_clicked() => {
                    if sk.hover.is_none() {
                        sk.fit(scene);
                    }
                }
                _ => self.camera.fit(Some((lo, hi)), aspect),
            }
            self.fitted = true;
        }
        let anchor = m.scene.anchor();
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
        // Work planes always; what a click would take lit while picking (a face tinted).
        let size = crate::origin::half(m.scene.bounds);
        let (clip, screen) = (|q| crate::camera::clip(&view_projection, q - anchor), |c| in_rect(c, rect));
        let work: Vec<crate::origin::Square> = self.work_planes().into_iter().map(|(_, s)| s).collect();
        if self.sketch.is_none() {
            let lit = match (self.picking, self.under) {
                (Some(_), Some(Under::Work(k))) => Some(k),
                _ => None,
            };
            crate::origin::paint(&painter, clip, screen, &work, size, lit, true);
        }
        // The origin planes while picking, and with no body yet: what work planes and sketches
        // are placed by.
        if self.picking.is_some() || (self.sketch.is_none() && m.scene.bodies.is_empty()) {
            let lit = self.picking.and_then(|purpose| match self.under {
                Some(Under::Origin(k)) => Some(k),
                Some(_) => None,
                None => (purpose == Purpose::Sketch).then_some(crate::origin::FIRST),
            });
            crate::origin::paint(&painter, clip, screen, &crate::origin::origin_squares(), size, lit, false);
        }
        if self.sketch.is_none() {
            crate::origin::paint_origin(&painter, clip, screen, size);
        }
        if let Some(purpose) = self.picking {
            if let Some(Under::Face(f)) = self.under {
                let mesh = &m.scene.bodies[f.body];
                let mut tint = egui::Mesh::default();
                for t in mesh.triangles.iter().zip(&mesh.faces).filter(|(_, id)| **id == f.face).map(|(t, _)| t) {
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
            let hint = match purpose {
                Purpose::Sketch => "Click a plane or a plane face to sketch on · Enter or a right click: XY · Esc: cancel",
                Purpose::Look => "Click a face or a plane to look straight at · Esc: cancel",
                Purpose::Plane => "Click an origin plane, a work plane or a plane face to offset the new plane from · Esc: cancel",
            };
            painter.text(
                rect.center_bottom() - egui::vec2(0.0, 12.0),
                egui::Align2::CENTER_BOTTOM,
                hint,
                egui::FontId::proportional(13.0),
                egui::Color32::from_rgb(60, 65, 75),
            );
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
        self.view_cube(ui, rect);
    }

    /// Picking an origin plane or a face, for a sketch or to look at: the nearest under the
    /// cursor; a click takes it. For a sketch XY is picked from the start, and Enter or a right
    /// click takes it; Esc gives up.
    fn pick_input(&mut self, ui: &egui::Ui, rect: egui::Rect, r: &egui::Response, offset: Option<(f64, f64)>) {
        let aspect = f64::from(rect.width() / rect.height().max(1.0));
        let mut chosen = None;
        let work = self.work_planes();
        if let (Some(purpose), Some(m)) = (self.picking, &self.model) {
            let half = (f64::from(rect.width()) * 0.5, f64::from(rect.height()) * 0.5);
            let size = crate::origin::half(m.scene.bounds);
            let squares: Vec<crate::origin::Square> = work.iter().map(|(_, s)| s.clone()).collect();
            self.under = offset.and_then(|(x, y)| {
                let (origin, dir) = self.camera.ray(x / half.0, -y / half.1, aspect);
                // The nearest along the ray: a face, an origin plane or a work plane.
                let face = m.scene.pick(origin, dir).map(|f| (f.distance, Under::Face(f)));
                let first = crate::origin::hit(&crate::origin::origin_squares(), origin, dir, size).map(|(t, k)| (t, Under::Origin(k)));
                let planes = crate::origin::hit(&squares, origin, dir, size).map(|(t, k)| (t, Under::Work(k)));
                [face, first, planes].into_iter().flatten().min_by(|a, b| a.0.total_cmp(&b.0)).map(|(_, u)| u)
            });
            if r.clicked() {
                chosen = self.under.map(|u| (purpose, u));
            } else if purpose == Purpose::Sketch && (r.secondary_clicked() || ui.input(|i| i.key_pressed(egui::Key::Enter))) {
                chosen = Some((purpose, Under::Origin(crate::origin::FIRST)));
            }
            if ui.input(|i| i.key_pressed(egui::Key::Escape)) {
                self.picking = None;
            }
        }
        // A click on a body in 3D (no pick asked for) brings its flat pattern forward (D164).
        if self.picking.is_none()
            && self.sketch.is_none()
            && r.clicked()
            && crate::dock::is_open(&self.dock, crate::dock::Pane::Flat)
            && let (Some(m), Some((x, y))) = (&self.model, offset)
        {
            let (origin, dir) = self.camera.ray(x / (f64::from(rect.width()) * 0.5), -y / (f64::from(rect.height()) * 0.5), aspect);
            let named = m.scene.pick(origin, dir).and_then(|f| m.bodies.get(f.body)).filter(|n| m.flats.iter().any(|(f, _)| f == *n)).cloned();
            if named.is_some() {
                self.flat.chosen = named;
            }
        }
        // What a pick refers to in a feature: an origin plane, a work plane, a body's face.
        let body = |v: &Self, f: crate::scene::Hit| v.model.as_ref().map(|m| m.bodies[f.body].clone());
        match chosen {
            Some((Purpose::Sketch, Under::Origin(k))) => self.new_sketch(crate::origin::sketch_fields(k)),
            Some((Purpose::Sketch, Under::Work(k))) => self.new_sketch(json!({"inputs": [work[k].0]})),
            Some((Purpose::Sketch, Under::Face(f))) => {
                let fields = json!({"inputs": [body(self, f)], "face": format!("{:?}", f.face)});
                self.new_sketch(fields);
            }
            Some((Purpose::Plane, under)) => {
                let from = match under {
                    Under::Origin(k) => json!({"base": datum_ops::OriginPlane::ALL[k]}),
                    Under::Work(k) => json!({"inputs": [work[k].0]}),
                    Under::Face(f) => json!({"inputs": [body(self, f)], "face": format!("{:?}", f.face)}),
                };
                self.plane_draft = Some((from, "10".into()));
                self.picking = None;
            }
            Some((Purpose::Look, under)) => {
                let forward = self.camera.axes().2;
                let normal = match under {
                    Under::Face(f) => f.normal,
                    Under::Origin(k) => crate::origin::origin_squares()[k].normal_towards(forward),
                    Under::Work(k) => work[k].1.normal_towards(forward),
                };
                self.turn = Some(crate::cube::Turn::toward(&self.camera, normal, ui.input(|i| i.time), false));
                self.picking = None;
                ui.ctx().request_repaint();
            }
            None => {}
        }
    }

    /// The model's work planes (D150) that are made, by feature, as squares about the model.
    fn work_planes(&self) -> Vec<(String, crate::origin::Square)> {
        let Some(m) = &self.model else { return Vec::new() };
        let near = m.scene.anchor();
        m.inspect["features"]
            .as_array()
            .into_iter()
            .flatten()
            .filter_map(|f| {
                let name = f["name"].as_str()?;
                let plane: datum_ops::SketchPlane = serde_json::from_value(f["status"]["plane"].clone()).ok()?;
                Some((name.to_string(), crate::origin::square(name, &plane, near)))
            })
            .collect()
    }

    /// The offset of a new work plane, asked once its base is picked: a number or a formula;
    /// Enter or OK adds the plane, Esc or Cancel drops it.
    fn plane_window(&mut self, ctx: &egui::Context) {
        let Some((from, offset)) = &mut self.plane_draft else { return };
        let (mut add, mut cancel) = (false, false);
        egui::Window::new("Plane").collapsible(false).resizable(false).anchor(egui::Align2::CENTER_TOP, egui::vec2(0.0, 60.0)).show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.label("Offset");
                let r = ui.add(egui::TextEdit::singleline(offset).desired_width(120.0).hint_text("number or formula"));
                add = r.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
                // Typed at once, as a sketch's dimension is.
                if !add && !r.has_focus() {
                    r.request_focus();
                }
            });
            ui.horizontal(|ui| {
                add |= ui.button("OK").clicked();
                cancel = ui.button("Cancel").clicked() || ui.input(|i| i.key_pressed(egui::Key::Escape));
            });
        });
        if cancel {
            self.plane_draft = None;
        } else if add {
            let mut fields = from.clone();
            fields["offset"] = typed(offset);
            fields["op"] = json!("plane");
            let taken: Vec<String> = self.work_planes().into_iter().map(|(n, _)| n).collect();
            let features: Vec<&str> = self
                .model
                .as_ref()
                .map(|m| m.inspect["features"].as_array().into_iter().flatten().filter_map(|f| f["name"].as_str()).collect())
                .unwrap_or_default();
            fields["name"] =
                json!((1..).map(|k| format!("plane{k}")).find(|n| !features.contains(&n.as_str()) && !taken.contains(n)).unwrap_or_default());
            self.plane_draft = None;
            self.send(Request::Command("add", fields));
        }
    }

    /// The mouse in the model's view: a turn to a view under way, orbit, pan, zoom.
    fn view_input(&mut self, ui: &egui::Ui, rect: egui::Rect, r: &egui::Response) {
        if let Some(turn) = self.turn {
            let ([right, up], done) = turn.at(ui.input(|i| i.time));
            self.camera.set(right, up);
            if done {
                self.turn = None;
                self.fitted &= !turn.fit;
            } else {
                ui.ctx().request_repaint();
            }
        }
        let d = r.drag_delta();
        if r.dragged_by(egui::PointerButton::Primary) {
            self.turn = None;
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

    /// The navigation cube over the model's view; not while a sketch is edited, whose view
    /// looks straight at it.
    fn view_cube(&mut self, ui: &mut egui::Ui, rect: egui::Rect) {
        if self.sketch.is_some() {
            return;
        }
        let now = ui.input(|i| i.time);
        match crate::cube::show(ui, rect, &self.camera) {
            Some(crate::cube::Ask::Look(d)) => {
                let d = datum_math::Vec3::new(d[0].into(), d[1].into(), d[2].into());
                self.turn = Some(crate::cube::Turn::toward(&self.camera, d, now, false));
                // The turn goes on in the frames to come: a click asks for none by itself.
                ui.ctx().request_repaint();
            }
            Some(crate::cube::Ask::Roll(quarter)) => {
                self.turn = Some(crate::cube::Turn::to(&self.camera, self.camera.rolled(quarter), now, false));
                ui.ctx().request_repaint();
            }
            Some(crate::cube::Ask::Home) => self.home(ui),
            Some(crate::cube::Ask::Orbit(dx, dy)) => {
                self.turn = None;
                self.camera.orbit(dx, dy);
            }
            None => {}
        }
    }

    /// Turns to the home view, the default direction, and fits the model there.
    fn home(&mut self, ui: &egui::Ui) {
        let c = Camera::default();
        self.turn = Some(crate::cube::Turn::to(&self.camera, [c.right, c.up], ui.input(|i| i.time), true));
        ui.ctx().request_repaint();
    }
}

/// Where clip coordinates `c` of a point in front of the eye show in `rect`.
fn in_rect(c: [f32; 4], rect: egui::Rect) -> egui::Pos2 {
    rect.center() + egui::vec2(c[0] / c[3] * rect.width() * 0.5, -c[1] / c[3] * rect.height() * 0.5)
}

/// Where point `q` (relative to the anchor) shows in `rect` through view-projection `m`, if in
/// front of the eye.
fn on_screen(m: &[f32; 16], q: datum_math::Vec3, rect: egui::Rect) -> Option<egui::Pos2> {
    let c = crate::camera::clip(m, q);
    (c[3] > 0.0).then(|| in_rect(c, rect))
}

impl Viewer {
    /// The mouse and keys while a sketch is edited: hover, select, drag points, pan, zoom,
    /// delete, Esc.
    /// The menu a held right click opened (D155): a pattern's numbers and the elements it
    /// copies, or what to do with the copies selected; previewed as it changes, sent on Done.
    fn sketch_menu(&mut self, ctx: &egui::Context) {
        let (Some(sk), Some(menu)) = (&mut self.sketch, &mut self.menu) else { return };
        let at = match menu {
            Menu::Pattern { at, .. } | Menu::Copies { at, .. } | Menu::Note { at, .. } => *at,
        };
        // What the menu ends in: sent, and what follows it.
        enum End {
            Stay,
            Cancel,
            Send(serde_json::Map<String, Json>),
            Edit(String),
        }
        let mut end = End::Stay;
        let (esc, enter) = ctx.input(|i| (i.key_pressed(egui::Key::Escape), i.key_pressed(egui::Key::Enter)));
        egui::Area::new(egui::Id::new("datum_sketch_menu")).fixed_pos(at).order(egui::Order::Foreground).show(ctx, |ui| {
            egui::Frame::popup(ui.style()).show(ui, |ui| match menu {
                Menu::Note { why, .. } => {
                    ui.label(why.as_str());
                    if ui.button("OK").clicked() || esc || enter {
                        end = End::Cancel;
                    }
                }
                Menu::Pattern { form, error, .. } => {
                    ui.strong(form.title());
                    if matches!(form.base, crate::pattern::Base::Along { closed: true, .. }) {
                        ui.horizontal(|ui| {
                            ui.radio_value(&mut form.fill, true, "Fill it");
                            ui.radio_value(&mut form.fill, false, "Along its outline");
                        });
                    }
                    for (row, keys, flip) in form.rows() {
                        ui.horizontal(|ui| {
                            ui.label(row);
                            if let Some(i) = flip
                                && ui.selectable_label(form.flip[i], "⇄").on_hover_text("The other way").clicked()
                            {
                                form.flip[i] = !form.flip[i];
                            }
                        });
                        egui::Grid::new(("datum_pattern_row", row)).num_columns(2).show(ui, |ui| {
                            for k in keys {
                                ui.label(crate::pattern::PatternForm::label(k));
                                if let Some(t) = form.fields.get_mut(k) {
                                    ui.add_sized([90.0, ui.spacing().interact_size.y], egui::TextEdit::singleline(t).hint_text("—"));
                                }
                                ui.end_row();
                            }
                        });
                    }
                    if matches!(form.base, crate::pattern::Base::Along { .. }) && !form.fill {
                        ui.checkbox(&mut form.rotate, "Turn with the path");
                    }
                    ui.horizontal(|ui| {
                        let hint = "Click or frame what to copy (holes and the like); Shift or Ctrl adds or takes away";
                        if ui.selectable_label(form.picking, "Select elements").on_hover_text(hint).clicked() && !form.picking {
                            form.picking = true;
                            sk.selected.clear();
                        }
                        match (form.cell.len(), &form.of) {
                            (0, Some(of)) => ui.weak(format!("copies `{of}`")),
                            (n, _) => ui.weak(format!("{n} picked")),
                        };
                    });
                    if !form.picking || !form.cell.is_empty() || form.of.is_some() {
                        *error = form.preview(sk).err().or_else(|| sk.placed.as_ref().err().cloned());
                    }
                    if let Some(e) = error {
                        ui.colored_label(egui::Color32::from_rgb(190, 40, 40), e.as_str());
                    }
                    let ready = error.is_none() && (!form.cell.is_empty() || form.of.is_some());
                    ui.horizontal(|ui| {
                        if (ui.add_enabled(ready, egui::Button::new("Done")).clicked() || (enter && ready))
                            && let Ok(Some(p)) = form.patch()
                        {
                            end = End::Send(p);
                        }
                        if ui.button("Cancel").clicked() || esc {
                            end = End::Cancel;
                        }
                    });
                }
                Menu::Copies { form, .. } => {
                    let n = form.copies.len();
                    match form.pattern() {
                        Some(p) => ui.strong(format!("{n} {} of `{p}`", if n == 1 { "copy" } else { "copies" })),
                        None => ui.strong(format!("{n} copies")),
                    };
                    let axes = form.axes(sk);
                    egui::Grid::new("datum_copies_shift").num_columns(2).show(ui, |ui| {
                        for (axis, shift) in axes.iter().zip(&mut form.shift) {
                            ui.label(*axis);
                            ui.add_sized([90.0, ui.spacing().interact_size.y], egui::TextEdit::singleline(shift).hint_text("0"));
                            ui.end_row();
                        }
                    });
                    let shifted = form.shifted(sk);
                    form.preview(sk, &shifted);
                    ui.horizontal(|ui| {
                        if ui.button("Shift").on_hover_text("Move them by these numbers; both empty: back where the pattern puts them").clicked()
                            || enter
                        {
                            end = End::Send(shifted);
                        }
                        if ui.button("Leave out").on_hover_text("No copy there (Del does it too)").clicked() {
                            end = End::Send(form.removed(sk));
                        }
                    });
                    ui.horizontal(|ui| {
                        if let Some(p) = form.pattern().map(String::from) {
                            if ui.button("Edit pattern…").clicked() {
                                end = End::Edit(p.clone());
                            }
                            if ui.button("Delete pattern").on_hover_text("The copies go; the elements copied stay").clicked() {
                                let mut m = serde_json::Map::new();
                                m.insert("patterns".into(), json!({ p: null }));
                                end = End::Send(m);
                            }
                        }
                        if ui.button("Cancel").clicked() || esc {
                            end = End::Cancel;
                        }
                    });
                }
            });
        });
        match end {
            End::Stay => {}
            End::Cancel => {
                menu.cancel(sk);
                self.menu = None;
            }
            End::Send(p) => {
                menu.cancel(sk);
                sk.selected.clear();
                let params = sk.apply(p);
                self.menu = None;
                self.send(Request::Command("edit_sketch", params));
            }
            End::Edit(name) => {
                menu.cancel(sk);
                sk.selected.clear();
                self.menu = Some(match crate::pattern::PatternForm::edit(sk, &name) {
                    Ok(form) => Menu::Pattern { at, form, error: None },
                    Err(why) => Menu::Note { at, why },
                });
            }
        }
    }

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
        let typing = ui.ctx().text_edit_focused();
        // H, V in the dimension tool, its sub-mode: horizontal or vertical, for what is picked
        // next, or for the dimension open while its value is not typed yet (then letters go to
        // the field: a formula).
        if sk.draft.tool == Tool::Dimension {
            let untouched = match (&sk.field, &sk.draft.asked) {
                (None, _) => !typing,
                (Some(f), Some(t)) => f.text == *t,
                (Some(_), None) => false,
            };
            for (key, axis) in [(egui::Key::H, Axis::Horizontal), (egui::Key::V, Axis::Vertical)] {
                if untouched && ui.input_mut(|i| crate::actions::pressed(i, &egui::KeyboardShortcut::new(egui::Modifiers::NONE, key))) {
                    let letter = format!("{key:?}");
                    ui.input_mut(|i| i.events.retain(|e| !matches!(e, egui::Event::Text(t) if t.eq_ignore_ascii_case(&letter))));
                    sk.toggle_axis(axis);
                }
            }
        }
        if !typing {
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
                    draft.picks = sk.dimension_framed(&framed, draft.axis);
                }
                if r.clicked()
                    && let Some((x, y)) = offset
                {
                    // An element picked, or the place where the dimension goes.
                    let pick = sk.point_at(x, y, h).map(Pick2::Point).or_else(|| sk.curve_at(x, y, h).map(Pick2::Curve));
                    let at = sk.camera.at(x, y, h);
                    // An arc again: its angle, after its radius.
                    let again = |p: &Pick2| matches!((p, &draft.picks[..]), (Pick2::Curve(n), [q]) if q == p && sk.placed.as_ref().is_ok_and(|s| s.arcs.contains_key(n)));
                    match pick {
                        Some(p) if !draft.picks.contains(&p) || again(&p) => {
                            // A third pick starts a dimension anew; a second changes the one
                            // open (a line's length into its distance to a point or a parallel).
                            if draft.picks.len() == 2 {
                                draft.picks.clear();
                            }
                            draft.picks.push(p);
                            // A curve, a point and a line, an arc's ends: dimensioned at once.
                            draft.asked =
                                if sk.dimension_now(&draft.picks, at, draft.axis) { sk.field.as_ref().map(|f| f.text.clone()) } else { None };
                        }
                        Some(_) => {}
                        // Where the dimension between two points goes.
                        None => {
                            if draft.asked.is_none()
                                && let Some((c, key, now)) = sk.dimension(&draft.picks, at, draft.axis)
                            {
                                sk.ask(c, key, now, at);
                            }
                            draft.picks.clear();
                            draft.axis = None;
                            draft.asked = None;
                        }
                    }
                }
            } else if let Tool::Constrain(kind) = draft.tool {
                if r.clicked()
                    && let (Some((x, y)), Ok(placed)) = (offset, &sk.placed)
                {
                    use crate::constrain::{Step, Want, step, wants};
                    let point = || sk.point_at(x, y, h).map(Pick2::Point);
                    let curve = || sk.curve_at(x, y, h).map(Pick2::Curve);
                    let pick = match wants(kind, &draft.picks) {
                        Want::Point => point(),
                        Want::Curve => curve(),
                        Want::Either => point().or_else(curve),
                    };
                    if let Some(p) = pick {
                        draft.picks.push(p);
                        let outcome = step(kind, &draft.picks, placed);
                        if outcome != Step::More {
                            draft.picks.clear();
                        }
                        let tried = match outcome {
                            Step::More => Ok(None),
                            Step::Done(c) => {
                                // Two circles made equal fix a diameter, not a point: the kernel
                                // refuses a repeat itself.
                                let circles = &sk.json["sketch"]["circles"];
                                let points = !(c["type"] == "equal_radius"
                                    && c["curves"].as_array().into_iter().flatten().all(|n| n.as_str().is_some_and(|n| circles.get(n).is_some())));
                                let mut patch = serde_json::Map::new();
                                patch.insert("add_constraints".into(), json!([c]));
                                sk.try_apply(patch, points).map(Some)
                            }
                            Step::Wrong(why) => Err(why),
                        };
                        match tried {
                            Ok(Some(p)) => {
                                sk.conflict = None;
                                sent.push(Request::Command("edit_sketch", p));
                            }
                            Ok(None) => {}
                            Err(why) => sk.conflict = Some(crate::dims::Conflict { constraint: Json::Null, why }),
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
            // The dimension the picks made is in (or its field gone): the next pick starts anew.
            if draft.asked.is_some() && sk.field.is_none() {
                draft.picks.clear();
                draft.axis = None;
                draft.asked = None;
            }
            if r.dragged_by(egui::PointerButton::Secondary) || r.dragged_by(egui::PointerButton::Middle) {
                let d = r.drag_delta();
                sk.camera.pan(f64::from(d.x), f64::from(d.y), h);
            }
            // Esc, or a right click (a right drag pans): the mouse alone ends the command too.
            if (!typing && !typing_field && ui.input(|i| i.key_pressed(egui::Key::Escape))) || r.secondary_clicked() {
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
            if !sk.start_drag(sk.camera.at(x, y, h)) && sk.hover.is_none() {
                sk.frame = Some(SelectFrame { ends: [(x, y); 2], touching: false });
            }
        }
        if let Some(framed) = sk.frame_to(offset, r.drag_stopped(), h, |t| crate::sketch::label_size(ui.painter(), t)) {
            crate::select::frame(&mut sk.selected, framed, ui.input(|i| crate::select::adding(&i.modifiers)));
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
            // A group's badge stands for what the group holds.
            let picks = match sk.hover.clone() {
                Some(Pick::Group(g)) => sk.group_picks(&g),
                Some(Pick::Pattern(n)) => sk.pattern_copies(&n),
                p => p.into_iter().collect(),
            };
            crate::select::click(&mut sk.selected, picks, ui.input(|i| crate::select::adding(&i.modifiers)));
        }
        let scroll = if r.hovered() { ui.input(|i| i.smooth_scroll_delta.y) } else { 0.0 };
        if scroll != 0.0 {
            let (x, y) = offset.unwrap_or((0.0, 0.0));
            sk.camera.zoom_at(libm::exp(-f64::from(scroll) * 0.002), x, y, h);
        }
        // A right click held still for half a second: the menu of what is selected (D155); a
        // short one (a right drag pans): the last command again, as Enter, by the mouse alone.
        let (now, pos, down, pressed) = ui.input(|i| (i.time, i.pointer.latest_pos(), i.pointer.secondary_down(), i.pointer.secondary_pressed()));
        if pressed && r.hovered() {
            self.hold = pos.map(|p| (now, p, false));
        }
        let mut held = false;
        if let (Some((since, at, opened)), Some(p)) = (&mut self.hold, pos) {
            if !down || p.distance(*at) > 6.0 {
                // Let go, or moved: a drag pans. A hold that opened the menu repeats nothing.
                held = *opened;
                self.hold = None;
            } else if !*opened && now - *since >= HOLD {
                *opened = true;
                if let Some(m) = self.menu.take() {
                    m.cancel(sk);
                }
                self.menu = Some(Menu::open(sk, *at));
            } else if !*opened {
                ui.ctx().request_repaint_after(std::time::Duration::from_millis(50));
            }
        }
        if r.secondary_clicked() && !held && self.menu.is_none() {
            sk.draft.again();
        }
        // The menu's pattern takes the selection as the elements to copy, previewed at once.
        if let Some(Menu::Pattern { form, error, .. }) = &mut self.menu
            && form.picking
        {
            let before = form.cell.len();
            form.take_cell(sk);
            if before == 0 && !form.cell.is_empty() {
                form.suggest_steps(sk);
            }
            *error = form.preview(sk).err();
        }
        if self.menu.is_none() && !ui.ctx().text_edit_focused() && !typing_field && !field_done.1 {
            // Enter: the last command again (an Enter that put a value in is not one).
            if !field_done.0 && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                sk.draft.again();
            }
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

/// What the level worked in does to its parent, by `inspect`: the parent's features it changes,
/// its own features, the parent's parameters it sets otherwise, its own parameters.
fn marks(inspect: &Json) -> (Vec<String>, Vec<String>, Vec<String>, Vec<String>) {
    let level = inspect["level"].as_str();
    let this = inspect["levels"].as_array().into_iter().flatten().find(|l| Some(l["name"].as_str().unwrap_or_default()) == level);
    let names = |key: &str| -> Vec<String> {
        this.and_then(|l| l[key].as_array()).into_iter().flatten().filter_map(|v| v.as_str().map(String::from)).collect()
    };
    (names("changed"), names("own"), names("parameters_set"), names("own_parameters"))
}

/// What parts and variants are, for the hint over them (D152).
const LEVELS: &str = "A part takes the frame's sketches and adds its own features; a variant takes its part and changes only what \
                      differs (a length, a few holes): a change to the frame or the part reaches every variant. A dot: changed here, \
                      ↺ back as the parent has it, + its own";

/// `prefix k`, the first not a level's name yet.
fn fresh(levels: &[(String, Option<String>, usize, Option<String>)], prefix: &str) -> String {
    (1..).map(|k| format!("{prefix} {k}")).find(|n| !levels.iter().any(|l| &l.0 == n)).unwrap_or_default()
}

/// How long a right click is held still to open the menu, in seconds.
const HOLD: f64 = 0.5;

/// The menu a held right click opens on the sketch: by what is selected, a pattern (its kind
/// by the selection), or what to do with copies selected (D155).
enum Menu {
    Pattern {
        at: egui::Pos2,
        form: crate::pattern::PatternForm,
        error: Option<String>,
    },
    Copies {
        at: egui::Pos2,
        form: crate::pattern::CopiesForm,
    },
    /// Nothing to do with this selection, and why.
    Note {
        at: egui::Pos2,
        why: String,
    },
}

impl Menu {
    fn open(sk: &SketchEdit, at: egui::Pos2) -> Self {
        // On a pattern's badge: that pattern, as it was given.
        if let Some(Pick::Pattern(n)) = &sk.hover {
            return match crate::pattern::PatternForm::edit(sk, n) {
                Ok(form) => Menu::Pattern { at, form, error: None },
                Err(why) => Menu::Note { at, why },
            };
        }
        if let Some(form) = crate::pattern::CopiesForm::new(sk) {
            return Menu::Copies { at, form };
        }
        match crate::pattern::base_of(sk) {
            Ok(base) => Menu::Pattern { at, form: crate::pattern::PatternForm::new(sk, base), error: None },
            Err(why) => Menu::Note { at, why },
        }
    }

    /// Back to the sketch as it was before the menu's preview.
    fn cancel(&self, sk: &mut SketchEdit) {
        match self {
            Menu::Pattern { form, .. } => form.cancel(sk),
            Menu::Copies { form, .. } => form.preview(sk, &serde_json::Map::new()),
            Menu::Note { .. } => {}
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
/// Whether `path`, chosen from `list` (named `what` in the message), is still `there`: one that
/// is not leaves the list, saying so.
fn still_there(list: &mut Vec<PathBuf>, path: &std::path::Path, there: bool, what: &str) -> bool {
    if !there {
        list.retain(|p| p != path);
        rfd::MessageDialog::new().set_title("Datum").set_description(format!("{} is no longer there: it leaves the {what}.", path.display())).show();
    }
    there
}

/// The folder for files of `kind` in `project`, or without one beside the program, made if it
/// is not there yet (D141).
fn kept_in(project: Option<&std::path::Path>, kind: &str) -> Option<PathBuf> {
    let root = project.map(std::path::Path::to_path_buf).or_else(|| datum_model::workspace::root(None));
    made(root.ok_or_else(|| "no folder to work in".to_string()).and_then(|r| datum_model::workspace::folder_in(&r, kind)))
}

/// `folder`, made if it is not there yet; why not, to the log.
fn made(folder: Result<PathBuf, String>) -> Option<PathBuf> {
    match folder.and_then(|f| std::fs::create_dir_all(&f).map(|()| f).map_err(|e| e.to_string())) {
        Ok(f) => Some(f),
        Err(why) => {
            crate::log::write(&why);
            None
        }
    }
}

/// How parameters are used, on hover over their heading and Add.
const FORMULAS: &str = "A parameter is a number or a formula of other parameters: w / 2 - gap, sqrt(a*a + b*b), max(t, 2). \
                        Operators + - * / and parentheses, pi, sqrt abs min max sin cos tan (degrees). A sketch dimension \
                        or a feature's number (a depth, an angle) takes a parameter's name or a formula too: type it in the field";

/// How a parameter's definition shows in its field: a number (whole ones without `.0`), an
/// expression as is.
/// A value as typed: a plain number stays a number, anything else is an expression.
fn typed(text: &str) -> Json {
    let text = text.trim();
    text.parse::<f64>().map_or_else(|_| json!(text), |v| json!(v))
}

/// The offsets of `inspect`'s work planes as their rows show them, by feature.
fn offsets(inspect: &Json) -> BTreeMap<String, String> {
    inspect["features"]
        .as_array()
        .into_iter()
        .flatten()
        .filter(|f| f["op"] == "plane")
        .filter_map(|f| Some((f["name"].as_str()?.to_string(), definition(&f["offset"]))))
        .collect()
}

fn definition(v: &Json) -> String {
    match (v.as_str(), v.as_f64()) {
        (Some(e), _) => e.to_string(),
        (None, Some(x)) if x.fract() == 0.0 && x.abs() < 1e15 => format!("{}", x as i64),
        _ => v.to_string(),
    }
}

/// The sketch tools, as the side panel lists them.
const TOOLS: [Act; 6] = [Act::Select, Act::Line, Act::Arc, Act::Circle, Act::Point, Act::Dimension];

/// The sketch tool action `act` takes up.
fn tool(act: Act) -> Tool {
    match act {
        Act::Line => Tool::Line,
        Act::Arc => Tool::Arc,
        Act::Circle => Tool::Circle,
        Act::Point => Tool::Point,
        Act::Dimension => Tool::Dimension,
        _ => Tool::Select,
    }
}

impl Viewer {
    /// The actions' keys, unless a text field takes them. A button focused (by Tab) does not:
    /// the keys would stop working until a click elsewhere.
    fn shortcuts(&mut self, ui: &egui::Ui) {
        if ui.ctx().text_edit_focused() {
            return;
        }
        for a in &ACTIONS {
            if let Some(key) = self.keys.get(a.act).copied()
                && self.available(a)
                && ui.input_mut(|i| crate::actions::pressed(i, &key))
            {
                self.run(ui, a.act);
            }
        }
    }

    /// Whether `a` can run now: where the window is, and what it needs.
    fn available(&self, a: &Action) -> bool {
        use crate::actions::Context;
        let here = match a.context {
            Context::Anywhere => true,
            Context::Model => self.sketch.is_none(),
            Context::Sketch => self.sketch.is_some(),
        };
        here && match a.act {
            Act::Save | Act::SaveAs | Act::Close | Act::ExportStep | Act::Home | Act::LookAt | Act::WorkPlane => self.model.is_some(),
            Act::CloseProject => self.project.is_some(),
            Act::Rectangle => !self.library.is_empty(),
            Act::Tangent => self.sketch.as_ref().is_some_and(|sk| sk.draft.chain.as_ref().is_some_and(|c| c.last.is_some())),
            _ => true,
        }
    }

    /// Whether `act` is on, for those that are turned on and off or chosen.
    fn checked(&self, act: Act) -> bool {
        let draft = self.sketch.as_ref().map(|sk| &sk.draft);
        match act {
            Act::SmoothEdges => self.smooth,
            Act::NewSketch => self.picking == Some(Purpose::Sketch),
            Act::WorkPlane => self.picking == Some(Purpose::Plane),
            Act::LookAt => self.picking == Some(Purpose::Look),
            Act::Tangent => draft.is_some_and(|d| d.tangent),
            Act::SideBar => self.side_bar.visible,
            Act::SideBarRight => self.side_bar.right,
            Act::FlatPane => crate::dock::is_open(&self.dock, crate::dock::Pane::Flat),
            _ if TOOLS.contains(&act) => draft.is_some_and(|d| d.tool == tool(act)),
            _ => false,
        }
    }

    fn run(&mut self, ui: &egui::Ui, act: Act) {
        match act {
            Act::New => self.new_model(),
            Act::Open => self.open_by_dialog(),
            Act::Save => {
                self.save(false);
            }
            Act::SaveAs => {
                self.save(true);
            }
            Act::Close => self.close(),
            Act::ExportStep => self.export_step(),
            Act::Exit => ui.send_viewport_cmd(egui::ViewportCommand::Close),
            Act::NewProject => self.project_by_dialog(true),
            Act::OpenProject => self.project_by_dialog(false),
            Act::CloseProject => self.set_project(None),
            Act::Undo => self.send(Request::Command("undo", json!({}))),
            Act::Redo => self.send(Request::Command("redo", json!({}))),
            Act::SmoothEdges => self.smooth = !self.smooth,
            Act::Home => self.home(ui),
            Act::NewSketch => {
                self.leave();
                self.start_sketch();
            }
            Act::LookAt => self.pick(Purpose::Look),
            Act::WorkPlane => {
                self.leave();
                self.pick(Purpose::Plane);
            }
            Act::Rectangle => {
                if let (Some(sk), Some(t)) = (&mut self.sketch, self.library.first()) {
                    sk.draft.choose(t.entry.element.clone());
                    sk.field = None;
                }
            }
            Act::Tangent => {
                if let Some(sk) = &mut self.sketch {
                    sk.draft.tangent = !sk.draft.tangent;
                }
            }
            Act::Select | Act::Line | Act::Arc | Act::Circle | Act::Point | Act::Dimension => {
                if let Some(sk) = &mut self.sketch {
                    sk.draft.take_up(tool(act));
                }
            }
            Act::SideBar => self.side_bar.visible = !self.side_bar.visible,
            Act::SideBarRight => self.side_bar.right = !self.side_bar.right,
            Act::FlatPane => {
                // On, closed; off, it opens where it belongs.
                if crate::dock::is_open(&self.dock, crate::dock::Pane::Flat) {
                    crate::dock::close(&mut self.dock, crate::dock::Pane::Flat);
                } else {
                    crate::dock::open(&mut self.dock, crate::dock::Pane::Flat);
                }
            }
            Act::ResetLayout => {
                self.dock = crate::dock::default_layout();
                self.side_bar = crate::sidebar::SideBar::default();
            }
        }
    }

    /// `act` as a button: in the menu with its key beside it, on the toolbar with it in the hint.
    fn item(&mut self, ui: &mut egui::Ui, act: Act, menu: bool) {
        let a = act.action();
        let mut button = egui::Button::new(a.label).selected(self.checked(act));
        if menu && let Some(k) = self.keys.get(act) {
            button = button.shortcut_text(ui.ctx().format_shortcut(k));
        }
        if ui.add_enabled(self.available(a), button).on_hover_text(self.keys.hint(ui.ctx(), act)).clicked() {
            self.run(ui, act);
        }
    }

    /// The menu (D131): every action, grouped as in other programs.
    fn menu(&mut self, ui: &mut egui::Ui) {
        ui.menu_button("File", |ui| {
            self.item(ui, Act::New, true);
            self.item(ui, Act::Open, true);
            ui.add_enabled_ui(!self.recent.is_empty(), |ui| {
                ui.menu_button("Open recent", |ui| {
                    if let Some(p) = recent_menu(ui, &self.recent, std::path::Path::exists) {
                        self.open_recent(p);
                    }
                });
            });
            ui.separator();
            self.item(ui, Act::NewProject, true);
            self.item(ui, Act::OpenProject, true);
            ui.add_enabled_ui(!self.recent_projects.is_empty(), |ui| {
                ui.menu_button("Recent projects", |ui| {
                    if let Some(p) = recent_menu(ui, &self.recent_projects, is_project) {
                        self.open_recent_project(p);
                    }
                });
            });
            self.item(ui, Act::CloseProject, true);
            ui.separator();
            for act in [Act::Save, Act::SaveAs, Act::Close] {
                self.item(ui, act, true);
            }
            ui.separator();
            self.item(ui, Act::ExportStep, true);
            ui.separator();
            self.item(ui, Act::Exit, true);
        });
        ui.menu_button("Edit", |ui| {
            self.item(ui, Act::Undo, true);
            self.item(ui, Act::Redo, true);
        });
        ui.menu_button("View", |ui| {
            self.item(ui, Act::Home, true);
            self.item(ui, Act::LookAt, true);
            self.item(ui, Act::SmoothEdges, true);
            ui.separator();
            for act in [Act::SideBar, Act::SideBarRight, Act::FlatPane] {
                self.item(ui, act, true);
            }
            self.item(ui, Act::ResetLayout, true);
        });
        ui.menu_button("Sketch", |ui| {
            self.item(ui, Act::NewSketch, true);
            self.item(ui, Act::WorkPlane, true);
            ui.separator();
            for act in TOOLS.into_iter().chain([Act::Rectangle, Act::Tangent]) {
                self.item(ui, act, true);
            }
        });
        ui.menu_button("Help", |ui| {
            ui.hyperlink_to(format!("Datum {}", env!("CARGO_PKG_VERSION")), env!("CARGO_PKG_REPOSITORY"));
            ui.weak(format!("Problems are logged to {}", crate::log::path().display()));
            if let Some(p) = &self.settings {
                ui.weak(format!("Keys and other settings: {}", p.display()));
            }
        });
    }

    /// What in the settings file could not be applied, until dismissed.
    fn settings_said(&mut self, ctx: &egui::Context) {
        if self.settings_problems.is_empty() {
            return;
        }
        let mut open = true;
        // Under the menu, in the middle: the menu stays in reach, the lines do not wrap narrow.
        let window = egui::Window::new("settings.json").open(&mut open).collapsible(false).resizable(false);
        window.anchor(egui::Align2::CENTER_TOP, [0.0, 36.0]).min_width(460.0).show(ctx, |ui| {
            if let Some(p) = &self.settings {
                ui.weak(p.display().to_string());
            }
            for p in &self.settings_problems {
                ui.label(p);
            }
        });
        if !open {
            self.settings_problems.clear();
        }
    }
}

/// A list of recent files or folders by name, the whole path on hover; one no longer `there`
/// shows grey. Returns the one clicked.
fn recent_menu(ui: &mut egui::Ui, list: &[PathBuf], there: impl Fn(&std::path::Path) -> bool) -> Option<PathBuf> {
    let mut chosen = None;
    for p in list {
        let name = p.file_name().map_or_else(|| p.display().to_string(), |n| n.to_string_lossy().to_string());
        let text = if there(p) { egui::RichText::new(name) } else { egui::RichText::new(name).weak() };
        if ui.button(text).on_hover_text(p.display().to_string()).clicked() {
            chosen = Some(p.clone());
        }
    }
    chosen
}

/// The panes' contents (D161).
struct Tabs<'a> {
    v: &'a mut Viewer,
    rs: Option<&'a egui_wgpu::RenderState>,
}

impl egui_dock::TabViewer for Tabs<'_> {
    type Tab = crate::dock::Pane;

    fn id(&mut self, tab: &mut Self::Tab) -> egui::Id {
        egui::Id::new(("datum_pane", *tab))
    }

    fn title(&mut self, tab: &mut Self::Tab) -> egui::WidgetText {
        tab.title().into()
    }

    fn ui(&mut self, ui: &mut egui::Ui, tab: &mut Self::Tab) {
        use crate::dock::Pane;
        match tab {
            Pane::Model => {
                let [r, g, b] = BACKGROUND.map(|c| (c * 255.0).round() as u8);
                ui.painter().rect_filled(ui.max_rect(), 0.0, egui::Color32::from_rgb(r, g, b));
                if let Some(rs) = self.rs {
                    self.v.viewport(ui, rs);
                }
            }
            Pane::Flat => self.v.flat_pane(ui),
        }
    }

    fn is_closeable(&self, tab: &Self::Tab) -> bool {
        *tab != crate::dock::Pane::Model
    }

    fn scroll_bars(&self, tab: &Self::Tab) -> [bool; 2] {
        use crate::dock::Pane;
        match tab {
            Pane::Model | Pane::Flat => [false, false],
        }
    }
}

impl Viewer {
    /// The flat pattern pane (D161, D164): the blank of a body of what the level worked in makes
    /// (of the several, the one chosen here or by a click on the body in 3D), its side, its bend
    /// lines as guides.
    fn flat_pane(&mut self, ui: &mut egui::Ui) {
        let Some(m) = &self.model else {
            ui.weak("No model open.");
            return;
        };
        if m.flats.is_empty() {
            ui.weak("No flat pattern here: work in a part that makes a sheet or a plate (a body's blank comes by itself).");
            return;
        }
        let chosen = self.flat.chosen.clone().filter(|c| m.flats.iter().any(|(n, _)| n == c)).unwrap_or_else(|| m.flats[0].0.clone());
        let by_hand = m.inspect["features"].as_array().into_iter().flatten().any(|f| f["name"] == chosen.as_str() && f["op"] == "unfold");
        let back = m.inspect["flats"][&chosen]["back"].as_bool().unwrap_or(false);
        let (mut set_back, mut export) = (None, false);
        ui.horizontal(|ui| {
            if m.flats.len() > 1 {
                egui::ComboBox::from_id_salt("datum_flat_chosen").selected_text(chosen.as_str()).show_ui(ui, |ui| {
                    for (n, _) in &m.flats {
                        if ui.selectable_label(*n == chosen, n.as_str()).clicked() {
                            self.flat.chosen = Some(n.clone());
                        }
                    }
                });
            } else {
                ui.label(chosen.as_str());
            }
            // A body's two sides; an unfold feature names its face itself.
            if !by_hand {
                let mut on = back;
                if ui.checkbox(&mut on, "Back side").on_hover_text("Seen from the other side of the sheet: the mirror image").changed() {
                    set_back = Some(on);
                }
            }
            if matches!(m.flats.iter().find(|(n, _)| *n == chosen), Some((_, Ok(_)))) {
                export = ui.button("DXF…").on_hover_text("The blank for the turret press: R12, lines, arcs and circles, no bend lines").clicked();
            }
        });
        if export && let Some(p) = self.exported("dxf", &chosen).save_file() {
            self.send(Request::Command("export_dxf", json!({"name": chosen, "path": p.display().to_string()})));
        }
        let Some(m) = &self.model else { return };
        match m.flats.iter().find(|(n, _)| *n == chosen).map(|(_, v)| v) {
            Some(Ok(f)) => self.flat.show(ui, f),
            Some(Err(e)) => {
                ui.colored_label(egui::Color32::from_rgb(190, 40, 40), format!("No blank for {chosen}: {e}"));
            }
            None => {}
        }
        if let Some(on) = set_back {
            self.send(Request::Command("set_flat", json!({"feature": chosen, "back": on})));
        }
    }
}

impl eframe::App for Viewer {
    fn ui(&mut self, ui: &mut egui::Ui, frame: &mut eframe::Frame) {
        let start = std::time::Instant::now();
        if self.shown < 2 {
            self.shown += 1;
            if self.shown == 2 && self.maximized {
                ui.send_viewport_cmd(egui::ViewportCommand::Maximized(true));
            }
            ui.ctx().request_repaint();
        }
        if self.shown == 2 {
            self.maximized = ui.input(|i| i.viewport().maximized.unwrap_or(self.maximized));
        }
        self.receive();
        let t = title(self.project.as_deref(), self.path.as_deref());
        if t != self.titled {
            ui.send_viewport_cmd(egui::ViewportCommand::Title(t.clone()));
            self.titled = t;
        }
        // The window closed with changes not saved: asked first; saved, it closes once the
        // background thread has written the file.
        if ui.input(|i| i.viewport().close_requested()) && self.quitting.is_none() && self.modified() {
            ui.send_viewport_cmd(egui::ViewportCommand::CancelClose);
            self.quitting = self.ask_to_save();
        }
        if let Some(saving) = self.quitting
            && (!saving || (!self.busy && !self.modified()))
        {
            ui.send_viewport_cmd(egui::ViewportCommand::Close);
        }
        self.shortcuts(ui);
        egui::Panel::top("datum_toolbar").show(ui, |ui| self.top_row(ui));
        // The activity bar at the edge, the side bar beside it (D162).
        let (activity, side) = if self.side_bar.right {
            (egui::Panel::right("datum_activity"), egui::Panel::right("datum_side"))
        } else {
            (egui::Panel::left("datum_activity"), egui::Panel::left("datum_side"))
        };
        activity.exact_size(44.0).resizable(false).show(ui, |ui| self.activity(ui));
        if self.side_bar.visible {
            side.default_size(360.0).min_size(240.0).show(ui, |ui| self.side(ui));
        }
        self.keep(ui.ctx());
        self.plane_window(ui.ctx());
        self.settings_said(ui.ctx());
        // The panes, docked where the engineer put them (D161).
        egui::CentralPanel::default().frame(egui::Frame::NONE).show(ui, |ui| {
            let mut dock = std::mem::replace(&mut self.dock, egui_dock::DockState::new(Vec::new()));
            let style = egui_dock::Style::from_egui(ui.style());
            egui_dock::DockArea::new(&mut dock)
                .style(style)
                .show_add_buttons(false)
                .show_inside(ui, &mut Tabs { v: self, rs: frame.wgpu_render_state() });
            self.dock = dock;
        });
        self.ui_ms = smooth(self.ui_ms, start.elapsed().as_secs_f64() * 1e3);
    }

    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        storage.set_string(MAXIMIZED, self.maximized.to_string());
        storage.set_string(LAYOUT, crate::dock::to_text(&self.dock));
        storage.set_string(SIDE_BAR, self.side_bar.to_text());
        storage.set_string(RECENT, json!(self.recent).to_string());
        storage.set_string(RECENT_PROJECTS, json!(self.recent_projects).to_string());
        storage.set_string(PROJECT, json!(self.project).to_string());
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
        let planes = offsets(&inspect);
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
                closed: false,
                rebuild_ms: 0.0,
                scene_ms: 0.0,
                bodies: Vec::new(),
                flats: Vec::new(),
                editing: None,
            }),
            busy: false,
            camera: Camera::default(),
            fitted: true,
            turn: None,
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
            new_parameter: Default::default(),
            offsets: planes,
            local_error: None,
            path: None,
            update: channel().1,
            newer: None,
            sketch: None,
            entering: None,
            library: crate::templates::library(None),
            library_filter: String::new(),
            keeping: None,
            menu: None,
            side_bar: crate::sidebar::SideBar::default(),
            dock: crate::dock::default_layout(),
            flat: crate::flat::FlatView::default(),
            new_level: None,
            hold: None,
            quitting: None,
            worked_in: None,
            recent: Vec::new(),
            project: None,
            recent_projects: Vec::new(),
            titled: String::new(),
            keys: Keys::default(),
            settings: None,
            settings_problems: Vec::new(),
            shown: 0,
            maximized: false,
            picking: None,
            under: None,
            plane_draft: None,
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

    /// The pencil on a sketch asks to edit it; the model that comes back opens it in the view.
    #[test]
    fn a_sketch_opens_for_editing() {
        let (mut v, inbox) = viewer("symmetric_plate.json");
        {
            let mut h = egui_kittest::Harness::builder().with_size([360.0, 600.0]).build_ui(|ui| {
                egui::Panel::left("datum_side").min_size(340.0).show(ui, |ui| v.side(ui));
            });
            h.run();
            h.get_by_label("Edit the sketch").click();
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

    /// The navigation cube in the model's view, as the window has it: a click on the top's
    /// middle turns the camera to look from above, frames coming only as long as the window
    /// asks for them (a turn that asked for none waited for the mouse to move).
    #[test]
    fn a_click_on_the_cube_turns_the_view() {
        let (v, _inbox) = viewer("symmetric_plate.json");
        let mut h = egui_kittest::Harness::builder().with_size([600.0, 400.0]).with_step_dt(1.0 / 60.0).with_max_steps(100).build_ui_state(
            |ui, v: &mut Viewer| {
                let (rect, r) = ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());
                v.view_input(ui, rect, &r);
                v.view_cube(ui, rect);
            },
            v,
        );
        h.run();
        let rect = egui::Rect::from_min_size(egui::pos2(0.0, 0.0), egui::vec2(600.0, 400.0));
        let top = crate::cube::face_middle(&h.state().camera, rect, datum_math::Vec3::new(0.0, 0.0, 1.0));
        h.hover_at(top);
        h.run();
        // As a hand clicks: the button held for a tenth of a second, the mouse a pixel off.
        h.event(egui::Event::PointerButton { pos: top, button: egui::PointerButton::Primary, pressed: true, modifiers: egui::Modifiers::NONE });
        for k in 0..6 {
            if k == 3 {
                h.hover_at(top + egui::vec2(1.0, 1.0));
            }
            h.step();
        }
        let at = top + egui::vec2(1.0, 1.0);
        h.event(egui::Event::PointerButton { pos: at, button: egui::PointerButton::Primary, pressed: false, modifiers: egui::Modifiers::NONE });
        h.step();
        h.run();
        let c = h.state().camera;
        assert_eq!((c.right, c.up), (datum_math::Vec3::new(1.0, 0.0, 0.0), datum_math::Vec3::new(0.0, 1.0, 0.0)));
    }

    /// S picks where to sketch with XY picked already: Enter starts the sketch on it. Look at,
    /// then a click on XY, turns the view to look down at it.
    #[test]
    fn a_plane_is_picked_for_a_sketch_or_to_look_at() {
        let (mut v, inbox) = viewer("symmetric_plate.json");
        let frame = |v: &mut Viewer, events: Vec<egui::Event>| {
            let mut h = egui_kittest::Harness::builder().with_size([600.0, 400.0]).with_step_dt(1.0 / 60.0).with_max_steps(100).build_ui_state(
                |ui, v: &mut Viewer| {
                    let (rect, r) = ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());
                    let offset = r.hover_pos().map(|p| (f64::from(p.x - rect.center().x), f64::from(p.y - rect.center().y)));
                    v.view_input(ui, rect, &r);
                    v.pick_input(ui, rect, &r, offset);
                },
                std::mem::replace(v, viewer("symmetric_plate.json").0),
            );
            h.run();
            for e in events {
                if let egui::Event::PointerButton { pos, .. } = e {
                    h.hover_at(pos);
                    h.step();
                }
                h.event(e);
                h.step();
            }
            h.run();
            *v = std::mem::replace(h.state_mut(), viewer("symmetric_plate.json").0);
        };
        v.start_sketch();
        assert_eq!(v.picking, Some(Purpose::Sketch));
        let enter = egui::Event::Key { key: egui::Key::Enter, physical_key: None, pressed: true, repeat: false, modifiers: egui::Modifiers::NONE };
        frame(&mut v, vec![enter]);
        let sent: Vec<Request> = inbox.try_iter().collect();
        assert!(
            sent.iter()
                .any(|r| matches!(r, Request::Command("add", f) if f["op"] == "sketch" && f.get("plane").is_none() && f.get("face").is_none())),
            "a sketch on XY"
        );
        // A point of XY in front of the eye, where nothing else is: (2, −2, 0).
        v.pick(Purpose::Look);
        let (m, _) = v.camera.matrices(datum_math::Vec3::ZERO, 1.5, 10.0);
        let at =
            on_screen(&m, datum_math::Vec3::new(2.0, -2.0, 0.0), egui::Rect::from_min_size(egui::pos2(0.0, 0.0), egui::vec2(600.0, 400.0))).unwrap();
        let click = |pressed| egui::Event::PointerButton { pos: at, button: egui::PointerButton::Primary, pressed, modifiers: egui::Modifiers::NONE };
        frame(&mut v, vec![click(true), click(false)]);
        assert_eq!(v.picking, None);
        assert_eq!((v.camera.right, v.camera.up), (datum_math::Vec3::new(1.0, 0.0, 0.0), datum_math::Vec3::new(0.0, 1.0, 0.0)), "from above");
        // A work plane from XY: the plane picked, then its offset typed and Enter.
        v.camera = Camera::default();
        v.pick(Purpose::Plane);
        frame(&mut v, vec![click(true), click(false)]);
        assert_eq!(v.plane_draft, Some((json!({"base": "xy"}), "10".to_string())));
        let mut h = egui_kittest::Harness::builder().with_size([600.0, 400.0]).build_ui_state(|ui, v: &mut Viewer| v.plane_window(ui.ctx()), v);
        h.run();
        h.key_press(egui::Key::Enter);
        h.run();
        let v = h.into_state();
        assert!(v.plane_draft.is_none());
        let sent: Vec<Request> = inbox.try_iter().collect();
        assert!(sent.iter().any(|r| matches!(r, Request::Command("add", f) if f["op"] == "plane" && f["base"] == "xy" && f["offset"] == 10.0)));
    }

    /// A right click ends the command, as Esc does, for those who draw with the mouse alone, and
    /// with no command on takes the last one up again, as Enter does; a right drag still pans.
    #[test]
    fn a_right_click_ends_the_command_and_repeats_it() {
        let (mut v, _inbox) = viewer("symmetric_plate.json");
        let mut session = Session::default();
        let model = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/models/symmetric_plate.json");
        session.execute("open", &json!({"path": model.display().to_string()})).unwrap();
        if let Some(m) = &mut v.model {
            m.editing = Some(("outline".into(), Context { id: session.graph().nodes()[0].id, input: None }));
        }
        v.entering = Some("outline".into());
        v.sync_sketch();
        v.sketch.as_mut().unwrap().draft.take_up(Tool::Line);
        let mut h = egui_kittest::Harness::builder().with_size([600.0, 400.0]).build_ui_state(
            |ui, v: &mut Viewer| {
                let (rect, r) = ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());
                let offset = r.hover_pos().map(|p| (f64::from(p.x - rect.center().x), f64::from(p.y - rect.center().y)));
                v.sketch_input(ui, rect, &r, offset);
            },
            v,
        );
        let tool = |h: &egui_kittest::Harness<Viewer>| h.state().sketch.as_ref().unwrap().draft.tool;
        let button = |h: &mut egui_kittest::Harness<Viewer>, pressed, at: egui::Pos2| {
            h.hover_at(at);
            h.event(egui::Event::PointerButton { pos: at, button: egui::PointerButton::Secondary, pressed, modifiers: egui::Modifiers::NONE });
            h.step();
        };
        h.run();
        button(&mut h, true, egui::pos2(300.0, 200.0));
        button(&mut h, false, egui::pos2(400.0, 250.0));
        assert_eq!(tool(&h), Tool::Line, "a right drag pans");
        button(&mut h, true, egui::pos2(300.0, 200.0));
        button(&mut h, false, egui::pos2(300.0, 200.0));
        assert_eq!(tool(&h), Tool::Select, "a right click ends");
        button(&mut h, true, egui::pos2(300.0, 200.0));
        button(&mut h, false, egui::pos2(400.0, 250.0));
        assert_eq!(tool(&h), Tool::Select, "a right drag pans, with no command on too");
        button(&mut h, true, egui::pos2(300.0, 200.0));
        button(&mut h, false, egui::pos2(300.0, 200.0));
        assert_eq!(tool(&h), Tool::Line, "a right click again: the last command");
        button(&mut h, true, egui::pos2(300.0, 200.0));
        button(&mut h, false, egui::pos2(300.0, 200.0));
        assert_eq!(tool(&h), Tool::Select);
        h.key_press(egui::Key::Enter);
        h.run();
        assert_eq!(tool(&h), Tool::Line, "Enter: the last command again");
    }

    /// A right click held still opens the menu (D155) and repeats nothing: the point selected
    /// first makes it a pattern around it; the hole picked next is copied, sent on Enter.
    #[test]
    fn a_held_right_click_opens_the_pattern_menu() {
        let (mut v, inbox) = viewer("symmetric_plate.json");
        let mut session = Session::default();
        let model = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/models/symmetric_plate.json");
        session.execute("open", &json!({"path": model.display().to_string()})).unwrap();
        if let Some(m) = &mut v.model {
            m.editing = Some(("outline".into(), Context { id: session.graph().nodes()[0].id, input: None }));
        }
        v.entering = Some("outline".into());
        v.sync_sketch();
        let sk = v.sketch.as_mut().unwrap();
        sk.draft.take_up(Tool::Line);
        sk.draft.escape();
        sk.selected = vec![Pick::Point("o".into())];
        let mut h = egui_kittest::Harness::builder().with_size([600.0, 400.0]).with_step_dt(1.0 / 60.0).build_ui_state(
            |ui, v: &mut Viewer| {
                let (rect, r) = ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());
                let offset = r.hover_pos().map(|p| (f64::from(p.x - rect.center().x), f64::from(p.y - rect.center().y)));
                v.sketch_input(ui, rect, &r, offset);
                v.sketch_menu(ui.ctx());
            },
            v,
        );
        let at = egui::pos2(40.0, 40.0);
        h.run();
        h.hover_at(at);
        h.step();
        h.event(egui::Event::PointerButton { pos: at, button: egui::PointerButton::Secondary, pressed: true, modifiers: egui::Modifiers::NONE });
        for _ in 0..40 {
            h.step();
        }
        h.event(egui::Event::PointerButton { pos: at, button: egui::PointerButton::Secondary, pressed: false, modifiers: egui::Modifiers::NONE });
        h.step();
        assert_eq!(h.state().sketch.as_ref().unwrap().draft.tool, Tool::Select, "held: no command again");
        let Some(Menu::Pattern { form, .. }) = &mut h.state_mut().menu else { panic!("the pattern menu") };
        assert_eq!(form.base, crate::pattern::Base::Around("o".into()));
        form.fields.insert("n", "2".into());
        form.fields.insert("angle", "90".into());
        form.picking = true;
        h.state_mut().sketch.as_mut().unwrap().selected = vec![Pick::Curve("right_hole".into())];
        h.step();
        assert_eq!(h.state().sketch.as_ref().unwrap().placed.as_ref().map(|p| p.patterns.len()), Ok(1), "previewed");
        h.key_press(egui::Key::Enter);
        h.run();
        assert!(h.state().menu.is_none());
        let sent: Vec<Request> = inbox.try_iter().collect();
        assert!(
            sent.iter().any(|r| matches!(r, Request::Command("edit_sketch", p)
                if p["patterns"]["pattern1"] == json!({"of": "right_hole", "around": {"center": "o", "count": 2.0, "angle": 90.0}}))),
            "the pattern sent"
        );
        // The pattern's badge: a click selects its copy; held, the menu as it was given.
        let sk = h.state().sketch.as_ref().unwrap();
        let badge = sk.badges(400.0).into_iter().find(|b| b.pick == Pick::Pattern("pattern1".into())).expect("a badge");
        let at = egui::pos2(300.0 + badge.at.0 as f32, 200.0 + badge.at.1 as f32);
        h.hover_at(at);
        h.step();
        for pressed in [true, false] {
            h.event(egui::Event::PointerButton { pos: at, button: egui::PointerButton::Primary, pressed, modifiers: egui::Modifiers::NONE });
            h.step();
        }
        assert_eq!(h.state().sketch.as_ref().unwrap().selected, [Pick::Copy("pattern1".into(), [1, 0])]);
        h.event(egui::Event::PointerButton { pos: at, button: egui::PointerButton::Secondary, pressed: true, modifiers: egui::Modifiers::NONE });
        for _ in 0..40 {
            h.step();
        }
        h.event(egui::Event::PointerButton { pos: at, button: egui::PointerButton::Secondary, pressed: false, modifiers: egui::Modifiers::NONE });
        h.step();
        let Some(Menu::Pattern { form, .. }) = &h.state().menu else { panic!("the pattern's menu") };
        assert_eq!((form.name.as_str(), form.fields["n"].as_str(), form.fields["angle"].as_str()), ("pattern1", "2", "90"));
        assert_eq!(form.of.as_deref(), Some("right_hole"));
    }

    /// In the dimension tool a click on a line opens its length at once; a click on a line
    /// parallel to it then makes it their distance, and Enter puts that in (here, the width
    /// given already: offered as a reference dimension).
    #[test]
    fn a_line_then_a_parallel_one_dimensions_their_distance() {
        let (mut v, inbox) = viewer("symmetric_plate.json");
        let mut session = Session::default();
        let model = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/models/symmetric_plate.json");
        session.execute("open", &json!({"path": model.display().to_string()})).unwrap();
        if let Some(m) = &mut v.model {
            m.editing = Some(("outline".into(), Context { id: session.graph().nodes()[0].id, input: None }));
        }
        v.entering = Some("outline".into());
        v.sync_sketch();
        v.sketch.as_mut().unwrap().draft.take_up(Tool::Dimension);
        let screen = |v: &Viewer, q: [f64; 2]| {
            let (x, y) = v.sketch.as_ref().unwrap().camera.offset(q, 400.0);
            egui::pos2(300.0 + x as f32, 200.0 + y as f32)
        };
        let (right, left) = (screen(&v, [80.0, 12.0]), screen(&v, [0.0, 12.0]));
        let mut h = egui_kittest::Harness::builder().with_size([600.0, 400.0]).build_ui_state(
            |ui, v: &mut Viewer| {
                let (rect, r) = ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());
                let offset = r.hover_pos().map(|p| (f64::from(p.x - rect.center().x), f64::from(p.y - rect.center().y)));
                v.sketch_input(ui, rect, &r, offset);
            },
            v,
        );
        let click = |h: &mut egui_kittest::Harness<Viewer>, at: egui::Pos2| {
            h.hover_at(at);
            h.step();
            for pressed in [true, false] {
                h.event(egui::Event::PointerButton { pos: at, button: egui::PointerButton::Primary, pressed, modifiers: egui::Modifiers::NONE });
                h.step();
            }
        };
        let field = |h: &egui_kittest::Harness<Viewer>| match h.state().sketch.as_ref().unwrap().field.as_ref().map(|f| &f.entry) {
            Some(Entry::New { constraint, .. }) => constraint.clone(),
            _ => Json::Null,
        };
        h.run();
        click(&mut h, right);
        assert_eq!(field(&h), json!({"type": "length", "line": "right"}), "a line: its length at once");
        click(&mut h, left);
        assert_eq!(field(&h)["type"], "point_line", "then a parallel line: their distance");
        h.key_press(egui::Key::Enter);
        h.run();
        // The plate's width is `w` already: the distance would repeat it, so it is offered as a
        // reference dimension.
        let sk = h.state().sketch.as_ref().unwrap();
        assert_eq!(sk.conflict.as_ref().map(|c| c.constraint["type"].clone()), Some(json!("point_line")));
        assert!(sk.draft.picks.is_empty() && sk.draft.asked.is_none(), "the next pick starts anew");
        drop(h);
        assert!(!inbox.try_iter().any(|r| matches!(r, Request::Command("edit_sketch", _))));
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
    /// dialog lists to be put in order; Up moves one up. `DATUM_KEEP_SHOT=file.png` saves a picture.
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
        h.get_all_by_label("Up").nth(1).unwrap().click();
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

    /// The plate's sketch in the window: the `button` of a constraint, then a click on each
    /// hole. `DATUM_CONSTRAIN_SHOT=file.png` saves a picture of the panel.
    /// A viewer editing the sketch of the symmetric plate.
    fn sketching() -> (Viewer, std::sync::mpsc::Receiver<Request>) {
        let (mut v, inbox) = viewer("symmetric_plate.json");
        let mut session = Session::default();
        let model = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/models/symmetric_plate.json");
        session.execute("open", &json!({"path": model.display().to_string()})).unwrap();
        if let Some(m) = &mut v.model {
            m.editing = Some(("outline".into(), Context { id: session.graph().nodes()[0].id, input: None }));
        }
        v.entering = Some("outline".into());
        v.sync_sketch();
        (v, inbox)
    }

    /// The actions' keys in a sketch: L takes up the line, Shift+L does not; S, the model's key
    /// for a sketch, does nothing there.
    #[test]
    fn keys_in_a_sketch() {
        let (mut v, inbox) = sketching();
        let _ = inbox.try_iter().count();
        let mut press = |key, modifiers| {
            let mut h = egui_kittest::Harness::builder().with_size([200.0, 80.0]).build_ui(|ui| v.shortcuts(ui));
            h.event(egui::Event::Key { key, physical_key: None, pressed: true, repeat: false, modifiers });
            h.run();
        };
        press(egui::Key::L, egui::Modifiers::SHIFT);
        press(egui::Key::S, egui::Modifiers::NONE);
        assert_eq!(v.sketch.as_ref().map(|sk| sk.draft.tool), Some(Tool::Select));
        assert!(v.picking.is_none() && inbox.try_iter().next().is_none(), "no other sketch");
        let mut h = egui_kittest::Harness::builder().with_size([200.0, 80.0]).build_ui(|ui| v.shortcuts(ui));
        h.event(egui::Event::Key { key: egui::Key::L, physical_key: None, pressed: true, repeat: false, modifiers: egui::Modifiers::NONE });
        h.run();
        drop(h);
        assert_eq!(v.sketch.as_ref().map(|sk| sk.draft.tool), Some(Tool::Line));
    }

    /// A key moved in the settings: the action runs by the new one, no longer by its default.
    #[test]
    fn a_key_moved() {
        let (mut v, inbox) = viewer("symmetric_plate.json");
        v.keys.set(Act::Undo, Some(egui::KeyboardShortcut::new(egui::Modifiers::NONE, egui::Key::F2)));
        let mut h = egui_kittest::Harness::builder().with_size([200.0, 80.0]).build_ui(|ui| v.shortcuts(ui));
        let press = |key, modifiers| egui::Event::Key { key, physical_key: None, pressed: true, repeat: false, modifiers };
        h.event(press(egui::Key::Z, egui::Modifiers::COMMAND));
        h.run();
        drop(h);
        assert!(inbox.try_iter().next().is_none(), "Ctrl+Z no more");
        let mut h = egui_kittest::Harness::builder().with_size([200.0, 80.0]).build_ui(|ui| v.shortcuts(ui));
        h.event(press(egui::Key::F2, egui::Modifiers::NONE));
        h.run();
        drop(h);
        assert!(inbox.try_iter().any(|r| matches!(r, Request::Command("undo", _))), "F2 undoes");
    }

    /// A folder `name` in the temporary folder, emptied; a project if `project`.
    fn folder(name: &str, project: bool) -> PathBuf {
        let d = std::env::temp_dir().join(format!("datum-view-{name}"));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(&d).unwrap();
        if project {
            std::fs::write(d.join(datum_model::workspace::PROJECT_FILE), "{}").unwrap();
        }
        d
    }

    /// A project opened (D142): a folder becomes one; a new model works in it, its library in
    /// the palette; a model of another project keeps its own, and the project open stays.
    #[test]
    fn a_project_open() {
        let (mut v, _inbox) = viewer("symmetric_plate.json");
        let a = folder("project-a", false);
        std::fs::create_dir_all(a.join("library/sketch")).unwrap();
        let own = json!({"name": "A's slot", "anchors": ["a"], "sketch": {"points": {"a": [0, 0]}}});
        std::fs::write(a.join("library/sketch/slot.json"), own.to_string()).unwrap();
        v.open_project(a.clone(), false);
        assert!(is_project(&a), "made a project");
        assert_eq!((v.current_project(), v.recent_projects.first()), (Some(a.clone()), Some(&std::path::absolute(&a).unwrap())));
        assert!(v.library.iter().any(|t| t.entry.name() == "A's slot"), "A's library in the palette");
        assert_eq!(kept_in(v.current_project().as_deref(), "part"), Some(a.join("part")), "a new model goes to A's part");
        let b = folder("project-b", true);
        v.set_path(Some(b.join("part/plate.json")));
        assert_eq!((v.current_project(), v.project.clone()), (Some(b.clone()), Some(a.clone())), "B's model, A open");
        assert!(!v.library.iter().any(|t| t.entry.name() == "A's slot"), "B's model, B's library");
        v.set_path(None);
        v.set_project(None);
        assert_eq!(v.current_project(), None);
        let name = a.file_name().unwrap().to_string_lossy().to_string();
        assert_eq!(title(Some(&a), Some(std::path::Path::new("m.json"))), format!("Datum — [{name}] — m.json"));
    }

    /// The recent models: the latest first, each once, ten kept.
    #[test]
    fn recent_latest_first_once() {
        let mut recent = Vec::new();
        for k in 0..12 {
            remember(&mut recent, format!("/m/{k}.json").into());
        }
        remember(&mut recent, "/m/5.json".into());
        let names: Vec<String> = recent.iter().map(|p| p.display().to_string()).collect();
        assert_eq!(
            names,
            ["/m/5.json", "/m/11.json", "/m/10.json", "/m/9.json", "/m/8.json", "/m/7.json", "/m/6.json", "/m/4.json", "/m/3.json", "/m/2.json"]
        );
    }

    fn both_holes_by_hand(button: &str, kind: crate::constrain::Kind) -> (Viewer, std::sync::mpsc::Receiver<Request>) {
        let (mut v, inbox) = sketching();
        {
            let shot = std::env::var_os("DATUM_CONSTRAIN_SHOT");
            let builder = egui_kittest::Harness::builder().with_size([360.0, 700.0]);
            let builder = if shot.is_some() { builder.wgpu() } else { builder };
            let mut h = builder.build_ui(|ui| {
                egui::Panel::left("datum_side").min_size(340.0).show(ui, |ui| v.side(ui));
            });
            h.run();
            if let Some(path) = shot {
                h.render().unwrap().save(path).unwrap();
            }
            h.get_by_label(button).click();
            h.run();
        }
        assert_eq!(v.sketch.as_ref().unwrap().draft.tool, Tool::Constrain(kind));
        let area = std::cell::Cell::new(egui::Rect::NOTHING);
        let mut h = egui_kittest::Harness::builder().with_size([600.0, 400.0]).build_ui(|ui| {
            let (rect, r) = ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());
            area.set(rect);
            let offset = r.hover_pos().map(|p| (f64::from(p.x - rect.center().x), f64::from(p.y - rect.center().y)));
            v.sketch_input(ui, rect, &r, offset);
        });
        h.run();
        drop(h);
        // Where each hole's rightmost point is on the screen.
        let sk = v.sketch.as_ref().unwrap();
        let rect = area.get();
        let at = |c: &str| {
            let q = sk.placed.as_ref().unwrap().solved.points[c];
            let (x, y) = sk.camera.offset([q.x + 4.5, q.y], f64::from(rect.height()));
            rect.center() + egui::vec2(x as f32, y as f32)
        };
        let (first, second) = (at("h1"), at("h2"));
        let mut h = egui_kittest::Harness::builder().with_size([600.0, 400.0]).build_ui(|ui| {
            let (rect, r) = ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());
            let offset = r.hover_pos().map(|p| (f64::from(p.x - rect.center().x), f64::from(p.y - rect.center().y)));
            v.sketch_input(ui, rect, &r, offset);
        });
        for pos in [first, second] {
            h.hover_at(pos);
            h.run();
            h.event(egui::Event::PointerButton { pos, button: egui::PointerButton::Primary, pressed: true, modifiers: egui::Modifiers::NONE });
            h.event(egui::Event::PointerButton { pos, button: egui::PointerButton::Primary, pressed: false, modifiers: egui::Modifiers::NONE });
            h.run();
        }
        drop(h);
        (v, inbox)
    }

    /// Constraints by hand: the plate is fully defined and its holes 50 apart, so concentric is
    /// refused with why, nothing sent.
    #[test]
    fn a_constraint_by_hand() {
        let (v, inbox) = both_holes_by_hand("Concentric", crate::constrain::Kind::Concentric);
        let sk = v.sketch.as_ref().unwrap();
        assert!(sk.draft.picks.is_empty(), "two holes picked, then done: {:?}", sk.draft.picks);
        let why = &sk.conflict.as_ref().expect("refused").why;
        assert!(!why.is_empty(), "with why");
        assert!(inbox.try_iter().all(|r| !matches!(r, Request::Command("edit_sketch", _))), "nothing sent");
    }

    /// Two circles made equal (issue #57): nothing moves, yet the second's diameter follows the
    /// first's from now on, so it is sent; the second shows no diameter of its own.
    #[test]
    fn equal_circles_by_hand() {
        let (v, inbox) = both_holes_by_hand("Equal", crate::constrain::Kind::Equal);
        let sk = v.sketch.as_ref().unwrap();
        assert!(sk.conflict.is_none(), "{:?}", sk.conflict.as_ref().map(|c| &c.why));
        let sent: Vec<Json> = inbox.try_iter().filter_map(|r| if let Request::Command("edit_sketch", p) = r { Some(p) } else { None }).collect();
        assert_eq!(sent.len(), 1);
        assert_eq!(sent[0]["add_constraints"][0]["type"], "equal_radius", "{}", sent[0]);
        let diameters: Vec<String> = sk.labels().into_iter().filter(|l| l.text.starts_with('Ø')).map(|l| l.text).collect();
        assert_eq!(diameters, ["Øhole = 9"], "one diameter shown");
    }

    /// Close (in the menu): a model with nothing unsaved closes without asking; the background thread is told,
    /// the file and title go.
    #[test]
    fn close_a_saved_model() {
        let (mut v, inbox) = viewer("symmetric_plate.json");
        v.path = Some("plate.json".into());
        assert!(!v.modified());
        {
            // `DATUM_MENU_SHOT=file.png` saves a picture of the open File menu.
            let shot = std::env::var_os("DATUM_MENU_SHOT");
            let builder = egui_kittest::Harness::builder().with_size([800.0, 300.0]);
            let builder = if shot.is_some() { builder.wgpu() } else { builder };
            let mut h = builder.build_ui(|ui| v.top_row(ui));
            h.run();
            h.get_by_label("File").click();
            h.run();
            if let Some(path) = shot {
                h.render().unwrap().save(path).unwrap();
            }
            h.get_by_label("Close Ctrl+W").click();
            h.run();
        }
        assert!(inbox.try_iter().any(|r| matches!(r, Request::Close)));
        assert!(v.path.is_none() && v.sketch.is_none());
    }

    /// A new parameter from the panel (issue #57): Add sends it, a formula as typed; a name
    /// taken already is refused in the window; ✖ removes one.
    #[test]
    fn parameters_are_added_and_removed() {
        let (mut v, inbox) = viewer("symmetric_plate.json");
        let panel = |v: &mut Viewer, click: &str| {
            let mut h = egui_kittest::Harness::builder().with_size([360.0, 700.0]).build_ui(|ui| v.side(ui));
            h.run();
            h.get_all_by_label(click).next().unwrap().click();
            h.run();
        };
        v.new_parameter = ("gap".into(), "w / 10".into());
        panel(&mut v, "Add");
        assert!(v.new_parameter.0.is_empty() && v.local_error.is_none());
        v.new_parameter = ("hole".into(), "5".into());
        panel(&mut v, "Add");
        assert!(v.local_error.as_ref().is_some_and(|e| e.contains("already")), "{:?}", v.local_error);
        assert_eq!(v.new_parameter.0, "hole", "kept to be renamed");
        panel(&mut v, "Remove the parameter");
        let sent: Vec<Request> = inbox.try_iter().collect();
        assert!(sent.iter().any(|r| matches!(r, Request::SetParameter(n, val) if n == "gap" && val == "w / 10")));
        assert!(!sent.iter().any(|r| matches!(r, Request::SetParameter(n, _) if n == "hole")));
        assert!(sent.iter().any(|r| matches!(r, Request::Command("remove_parameter", p) if p["name"] == "h")), "the first row's");
    }

    /// The frame and its part in the panel (D152): a click on the part works in it, in the variant
    /// the product has of it (D160); + Part names a new one; in a variant, the sketch it changes has ↺, which takes it back.
    #[test]
    fn parts_and_variants_in_the_panel() {
        let (mut v, inbox) = viewer("wall_variants.json");
        let click = |v: &mut Viewer, label: &str| {
            let mut h = egui_kittest::Harness::builder().with_size([360.0, 900.0]).build_ui(|ui| v.side(ui));
            h.run();
            h.get_by_label(label).click();
            h.run();
        };
        click(&mut v, "Стенка верхняя");
        click(&mut v, "+ Part");
        assert_eq!(v.new_level, Some((None, "Part 1".to_string())));
        click(&mut v, "Create");
        let sent: Vec<Request> = inbox.try_iter().collect();
        assert!(sent.iter().any(|r| matches!(r, Request::Command("activate", p) if p["level"] == "2 м")));
        assert!(sent.iter().any(|r| matches!(r, Request::Command("add_level", p) if p["name"] == "Part 1" && p["parent"].is_null())));
        // In the variant of 2.5 m: its sketch changed (a dot), its length set (↺ by the parameter too).
        let mut session = Session::default();
        let model = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/models/wall_variants.json");
        session.execute("open", &json!({"path": model.display().to_string()})).unwrap();
        session.execute("activate", &json!({"level": "2,5 м"})).unwrap();
        v.model.as_mut().unwrap().inspect = session.execute("inspect", &json!({})).unwrap();
        let mut h = egui_kittest::Harness::builder().with_size([360.0, 900.0]).build_ui(|ui| v.side(ui));
        h.run();
        assert!(h.query_by_label("✔ outline  ·  sketch").is_some());
        let reverts: Vec<_> = h.get_all_by_label("↺").collect();
        assert_eq!(reverts.len(), 3, "the length, the sketch and the notch turned off");
        reverts[1].click();
        h.run();
        drop(h);
        let sent: Vec<Request> = inbox.try_iter().collect();
        assert!(sent.iter().any(|r| matches!(r, Request::Command("revert", p) if p["feature"] == "outline")), "the sketch back");
    }

    /// A feature's box turns it off (D159): `edit` with `enabled` false.
    #[test]
    fn a_feature_is_turned_off_by_its_box() {
        let (mut v, inbox) = viewer("symmetric_plate.json");
        let last = v.model.as_ref().unwrap().inspect["features"].as_array().unwrap().last().unwrap()["name"].as_str().unwrap().to_string();
        let mut h = egui_kittest::Harness::builder().with_size([360.0, 900.0]).build_ui(|ui| v.side(ui));
        h.run();
        h.get_all_by_role(egui::accesskit::Role::CheckBox).last().unwrap().click();
        h.run();
        drop(h);
        let sent: Vec<Request> = inbox.try_iter().collect();
        assert!(sent.iter().any(|r| matches!(r, Request::Command("edit", p) if p["name"] == last.as_str() && p["enabled"] == false)));
    }

    /// The activity bar shows the side bar's sets (D162): the model's sections under one another;
    /// its icon again hides the side bar.
    #[test]
    fn the_activity_bar_shows_and_hides_the_side_bar() {
        let (mut v, _inbox) = viewer("wall_variants.json");
        let mut h = egui_kittest::Harness::builder().with_size([420.0, 700.0]).with_pixels_per_point(1.0).build_ui(|ui| {
            egui::Panel::left("datum_activity").exact_size(44.0).resizable(false).show(ui, |ui| v.activity(ui));
            if v.side_bar.visible {
                egui::Panel::left("datum_side").exact_size(360.0).show(ui, |ui| v.side(ui));
            }
        });
        h.run();
        for title in ["PARTS", "PARAMETERS", "FEATURES"] {
            assert!(h.query_by_label(title).is_some(), "{title}");
        }
        if std::env::var_os("DATUM_PICTURE").is_some() {
            let image = h.render().unwrap();
            image.save(std::env::var("DATUM_PICTURE").unwrap()).unwrap();
        }
        drop(h);
        v.side_bar.click(crate::sidebar::Container::Model);
        assert!(!v.side_bar.visible);
    }

    /// The flat pattern pane (D164): a body's blank with its side box and DXF button; the box asks
    /// for the other side by `set_flat`; a click in 3D on a body brings its blank forward.
    #[test]
    fn the_flat_pane_shows_a_bodys_blank() {
        let (mut v, inbox) = viewer("wall_variants.json");
        let mut session = Session::default();
        let model = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/models/wall_variants.json");
        session.execute("open", &json!({"path": model.display().to_string()})).unwrap();
        session.execute("activate", &json!({"level": "2,5 м"})).unwrap();
        let m = v.model.as_mut().unwrap();
        m.flats = session.flats();
        m.inspect = session.execute("inspect", &json!({})).unwrap();
        assert!(matches!(&m.flats[..], [(n, Ok(_))] if n == "notched"), "the part's body, laid flat");
        let mut h = egui_kittest::Harness::builder().with_size([700.0, 300.0]).with_pixels_per_point(1.0).build_ui(|ui| v.flat_pane(ui));
        h.run();
        assert!(h.query_by_label("DXF…").is_some());
        h.get_by_label("Back side").click();
        h.run();
        drop(h);
        let sent: Vec<Request> = inbox.try_iter().collect();
        assert!(
            sent.iter().any(|r| matches!(r, Request::Command("set_flat", p) if p["feature"] == "notched" && p["back"] == true)),
            "the other side"
        );
        // A flat of a model with no levels: a bent sheet's blank, its bend lines drawn dashed.
        if let Ok(dir) = std::env::var("DATUM_PICTURE") {
            let (mut v, _inbox) = viewer("sheet_panel.json");
            let mut session = Session::default();
            let model = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/models/sheet_panel.json");
            session.execute("open", &json!({"path": model.display().to_string()})).unwrap();
            v.model.as_mut().unwrap().flats = session.flats();
            let mut h = egui_kittest::Harness::builder().with_size([800.0, 360.0]).with_pixels_per_point(1.0).build_ui(|ui| v.flat_pane(ui));
            h.run();
            h.render().unwrap().save(dir).unwrap();
        }
    }

    /// Ctrl+W closes and Ctrl+N gives an empty model (issue #57), with a button focused by Tab
    /// too; while a text field is typed in, they are its own.
    #[test]
    fn keys_close_and_new() {
        let (mut v, inbox) = viewer("symmetric_plate.json");
        v.path = Some("plate.json".into());
        let ctrl = |key| egui::Event::Key { key, physical_key: None, pressed: true, repeat: false, modifiers: egui::Modifiers::COMMAND };
        let mut text = String::new();
        let mut h = egui_kittest::Harness::builder().with_size([400.0, 80.0]).build_ui(|ui| {
            ui.text_edit_singleline(&mut text);
            let _ = ui.button("Focused");
            v.shortcuts(ui);
        });
        h.run();
        h.get_by_label("Focused").focus();
        h.run();
        h.event(ctrl(egui::Key::W));
        h.run();
        h.get_by_role(egui::accesskit::Role::TextInput).focus();
        h.run();
        h.event(ctrl(egui::Key::N));
        h.run();
        drop(h);
        let sent: Vec<Request> = inbox.try_iter().collect();
        assert!(sent.iter().any(|r| matches!(r, Request::Close)), "closed");
        assert!(!sent.iter().any(|r| matches!(r, Request::Command("new", _))), "typing: not a new model");
        let mut h = egui_kittest::Harness::builder().with_size([400.0, 80.0]).build_ui(|ui| v.shortcuts(ui));
        h.event(ctrl(egui::Key::N));
        h.run();
        drop(h);
        assert!(inbox.try_iter().any(|r| matches!(r, Request::Command("new", _))), "a new model");
    }

    /// Open looks where the engineer works: the open model's folder, after Close the last one's;
    /// only with neither does it fall back to `part` (D121).
    #[test]
    fn open_looks_where_working() {
        let (mut v, _inbox) = viewer("symmetric_plate.json");
        assert_eq!(v.working_folder(), None, "nothing opened yet: the part folder");
        v.set_path(Some("/work/examples/bracket.json".into()));
        assert_eq!(v.working_folder(), Some("/work/examples".into()));
        v.set_path(None);
        assert_eq!(v.working_folder(), Some("/work/examples".into()), "closed: where it was");
    }

    /// A sketch on a face only refers to the plate (D150): the plate stays in view beside it,
    /// and alone, once, while the sketch is edited.
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
        assert_eq!(names(None), ["plate", "sketch1"]);
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
        h.get_by_label("Side: left").click();
        h.run();
        let sent = inbox.try_recv();
        assert!(matches!(&sent, Ok(Request::Command("edit", fields)) if *fields == json!({"name": "panel", "side": "right"})), "an edit of the side");
    }

    /// A work plane's row says what it is offset from; its offset changes in place, Enter.
    #[test]
    fn a_work_plane_offset_changes_in_its_row() {
        let (mut v, inbox) = viewer("work_planes.json");
        let mut h = egui_kittest::Harness::builder().with_size([360.0, 600.0]).build_ui(|ui| {
            egui::Panel::left("datum_side").min_size(340.0).show(ui, |ui| v.side(ui));
        });
        h.run();
        for row in ["✔ front  ·  XZ +", "✔ back  ·  front +", "✔ on_top  ·  a face of block +"] {
            h.get_by_label(row);
        }
        h.get_all_by_value("-gap").next().unwrap().click();
        h.run();
        h.key_press_modifiers(egui::Modifiers::COMMAND, egui::Key::A);
        h.get_all_by_value("-gap").next().unwrap().type_text("-gap - 5");
        h.run();
        h.key_press(egui::Key::Enter);
        h.run();
        let sent = inbox.try_recv();
        assert!(
            matches!(&sent, Ok(Request::Command("edit", fields)) if *fields == json!({"name": "back", "offset": "-gap - 5"})),
            "an edit of the offset"
        );
    }
}
