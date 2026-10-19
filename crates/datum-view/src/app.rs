//! The viewer window: the model in 3D, its features and their state, and its parameters to
//! edit. Edits go through the same commands as the CLI and MCP (§6.1) and rebuild in a background
//! thread; the view keeps the last good model meanwhile.

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::mpsc::{Receiver, Sender, channel};

use datum_model::Session;
use egui_wgpu::wgpu;
use serde_json::{Value as Json, json};

use crate::camera::Camera;
use crate::capture::BACKGROUND;
use crate::render::{Frame, Renderer};
use crate::scene::{Geometry, MeshCache, Scene};

/// Opens a window on the model at `path`.
pub fn run(path: PathBuf) -> Result<(), String> {
    let options = eframe::NativeOptions {
        renderer: eframe::Renderer::Wgpu,
        depth_buffer: 24,
        multisampling: SAMPLES as u16,
        viewport: egui::ViewportBuilder::default().with_title(format!("Datum — {}", path.display())).with_inner_size([1280.0, 800.0]),
        ..Default::default()
    };
    eframe::run_native(
        "Datum",
        options,
        Box::new(move |cc| {
            let rs = cc.wgpu_render_state.as_ref().ok_or("the window has no wgpu renderer")?;
            let depth = egui_wgpu::depth_format_from_bits(24, 0).unwrap_or(wgpu::TextureFormat::Depth24Plus);
            let renderer = Renderer::new(&rs.device, rs.target_format, depth, SAMPLES);
            rs.renderer.write().callback_resources.insert(renderer);
            Ok(Box::new(Viewer::new(path, cc.egui_ctx.clone())))
        }),
    )
    .map_err(|e| e.to_string())
}

const SAMPLES: u32 = 4;

enum Request {
    Open(PathBuf),
    SetParameter(String, Json),
}

/// What the background thread hands the window after each rebuild.
struct Model {
    /// `inspect` of the session: features with status, parameters with values.
    inspect: Json,
    scene: Scene,
    /// The last command's error, if it was refused.
    error: Option<String>,
}

/// The session lives on its own thread; requests come in, models go out. Requests that pile up
/// while a rebuild runs are folded: only the latest value of each parameter is applied.
fn worker(requests: Receiver<Request>, models: Sender<Model>, ctx: egui::Context) {
    let mut session = Session::default();
    let mut meshes = MeshCache::default();
    while let Ok(first) = requests.recv() {
        let mut pending = vec![first];
        pending.extend(requests.try_iter());
        let mut latest: BTreeMap<String, Json> = BTreeMap::new();
        let mut error = None;
        for r in pending {
            match r {
                Request::Open(p) => {
                    latest.clear();
                    if let Err(e) = session.execute("open", &json!({"path": p.display().to_string()})) {
                        error = Some(e);
                    }
                }
                Request::SetParameter(name, value) => {
                    latest.insert(name, value);
                }
            }
        }
        for (name, value) in latest {
            if let Err(e) = session.execute("set_parameter", &json!({"name": name, "value": value})) {
                error = Some(e);
            }
        }
        let finals = session.evaluation().final_values(session.graph());
        let scene = Scene::new(&finals, &mut meshes);
        meshes.retain(&finals.iter().filter_map(|v| v.body()).collect::<Vec<_>>());
        let inspect = session.execute("inspect", &json!({})).unwrap_or_default();
        if models.send(Model { inspect, scene, error }).is_err() {
            return;
        }
        ctx.request_repaint();
    }
}

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
    /// Parameter definitions as typed, by name.
    edits: BTreeMap<String, String>,
}

impl Viewer {
    fn new(path: PathBuf, ctx: egui::Context) -> Self {
        let (requests, inbox) = channel();
        let (outbox, models) = channel();
        std::thread::spawn(move || worker(inbox, outbox, ctx));
        let _ = requests.send(Request::Open(path));
        let geometry = Arc::new(Geometry::new(&Scene::default(), datum_math::Vec3::ZERO));
        Self { requests, models, model: None, busy: true, camera: Camera::default(), fitted: false, geometry, version: 0, edits: BTreeMap::new() }
    }

    fn receive(&mut self) {
        while let Ok(m) = self.models.try_recv() {
            self.busy = false;
            self.geometry = Arc::new(Geometry::new(&m.scene, m.scene.anchor()));
            self.version += 1;
            if !self.fitted && m.scene.bounds.is_some() {
                self.camera.fit(m.scene.bounds);
                self.fitted = true;
            }
            if let Some(p) = m.inspect["parameters"].as_object() {
                self.edits = p.iter().map(|(n, v)| (n.clone(), definition(&v["definition"]))).collect();
            }
            self.model = Some(m);
        }
    }

    fn side(&mut self, ui: &mut egui::Ui) {
        let Some(m) = &self.model else {
            ui.label("Opening…");
            return;
        };
        if let Some(e) = &m.error {
            ui.colored_label(egui::Color32::from_rgb(190, 40, 40), e);
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
                let r = ui.add(egui::TextEdit::singleline(text).desired_width(110.0));
                if r.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
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
        egui::ScrollArea::vertical().show(ui, |ui| {
            for f in m.inspect["features"].as_array().into_iter().flatten() {
                let name = f["name"].as_str().unwrap_or_default();
                let op = f["op"].as_str().unwrap_or_default();
                match f["error"].as_str() {
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
    }

    fn viewport(&mut self, ui: &mut egui::Ui) {
        let (rect, r) = ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());
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
        if r.double_clicked() {
            self.camera.fit(self.model.as_ref().and_then(|m| m.scene.bounds));
        }
        let Some(m) = &self.model else { return };
        let anchor = m.scene.anchor();
        let radius = m.scene.bounds.map_or(1.0, |(lo, hi)| (hi - lo).dot(hi - lo).sqrt() * 0.5);
        let (view_projection, eye) = self.camera.matrices(anchor, f64::from(rect.width() / rect.height().max(1.0)), radius);
        let frame = Frame { version: self.version, geometry: self.geometry.clone(), view_projection, eye };
        ui.painter().add(egui_wgpu::Callback::new_paint_callback(rect, Paint(frame)));
        if self.busy {
            ui.painter().text(
                rect.left_top() + egui::vec2(10.0, 10.0),
                egui::Align2::LEFT_TOP,
                "rebuilding…",
                egui::FontId::proportional(14.0),
                egui::Color32::GRAY,
            );
        }
    }
}

/// How a parameter's definition shows in its field: a number as written, an expression as is.
fn definition(v: &Json) -> String {
    v.as_str().map_or_else(|| v.to_string(), str::to_string)
}

impl eframe::App for Viewer {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.receive();
        egui::Panel::left("datum_side").default_size(300.0).show(ui, |ui| self.side(ui));
        let [r, g, b] = BACKGROUND.map(|c| (c * 255.0).round() as u8);
        egui::CentralPanel::default().frame(egui::Frame::NONE.fill(egui::Color32::from_rgb(r, g, b))).show(ui, |ui| self.viewport(ui));
    }
}

/// A frame of the view as an egui paint callback.
struct Paint(Frame);

impl egui_wgpu::CallbackTrait for Paint {
    fn prepare(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        _screen: &egui_wgpu::ScreenDescriptor,
        _encoder: &mut wgpu::CommandEncoder,
        resources: &mut egui_wgpu::CallbackResources,
    ) -> Vec<wgpu::CommandBuffer> {
        if let Some(r) = resources.get_mut::<Renderer>() {
            r.prepare(device, queue, &self.0);
        }
        Vec::new()
    }

    fn paint(&self, _info: egui::PaintCallbackInfo, pass: &mut wgpu::RenderPass<'static>, resources: &egui_wgpu::CallbackResources) {
        if let Some(r) = resources.get::<Renderer>() {
            r.paint(pass);
        }
    }
}
