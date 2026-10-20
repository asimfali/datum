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
use crate::render::{BACKGROUND, Frame, Renderer};
use crate::scene::{Geometry, MeshCache, Scene};

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
    /// Write the flat blank of an unfold feature as DXF.
    ExportDxf(String, PathBuf),
    /// Change fields of a feature (`edit`).
    Edit(String, Json),
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
    /// A model was opened in this rebuild: the view fits it.
    opened: bool,
    /// Milliseconds the rebuild and the scene (meshes, vertices) took.
    rebuild_ms: f64,
    scene_ms: f64,
}

/// The session lives on its own thread; requests come in, models go out. Requests that pile up
/// while a rebuild runs are folded: only the latest value of each parameter is applied.
fn worker(requests: Receiver<Request>, models: Sender<Model>, ctx: egui::Context) {
    let mut session = Session::default();
    let mut meshes = MeshCache::default();
    while let Ok(first) = requests.recv() {
        let mut pending = vec![first];
        pending.extend(requests.try_iter());
        let start = std::time::Instant::now();
        let mut latest: BTreeMap<String, Json> = BTreeMap::new();
        let (mut error, mut opened) = (None, false);
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
                Request::Edit(name, mut fields) => {
                    fields["name"] = json!(name);
                    if let Err(e) = session.execute("edit", &fields) {
                        error = Some(e);
                    }
                }
                Request::ExportDxf(name, path) => {
                    if let Err(e) = session.execute("export_dxf", &json!({"name": name, "path": path.display().to_string()})) {
                        error = Some(e);
                    }
                }
            }
        }
        for (name, value) in latest {
            if let Err(e) = session.execute("set_parameter", &json!({"name": name, "value": value})) {
                error = Some(e);
            }
        }
        let rebuild_ms = start.elapsed().as_secs_f64() * 1e3;
        let start = std::time::Instant::now();
        let finals = session.evaluation().final_values(session.graph());
        let scene = Scene::new(&finals, &mut meshes);
        meshes.retain(&finals.iter().filter_map(|v| v.body()).collect::<Vec<_>>());
        let geometry = Arc::new(Geometry::new(&scene, scene.anchor()));
        let scene_ms = start.elapsed().as_secs_f64() * 1e3;
        let inspect = session.execute("inspect", &json!({})).unwrap_or_default();
        if models.send(Model { inspect, scene, geometry, error, opened, rebuild_ms, scene_ms }).is_err() {
            return;
        }
        ctx.request_repaint();
    }
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
        };
        if let Some(p) = path {
            v.open(p);
        }
        v
    }

    fn open(&mut self, path: PathBuf) {
        let _ = self.requests.send(Request::Open(path));
        self.busy = true;
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
        let (mut export, mut edit) = (None, None);
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
        if let Some((name, fields)) = edit {
            let _ = self.requests.send(Request::Edit(name, fields));
            self.busy = true;
        }
        if let Some((name, path)) = export {
            let _ = self.requests.send(Request::ExportDxf(name, path));
            self.busy = true;
        }
    }

    fn viewport(&mut self, ui: &mut egui::Ui, rs: &egui_wgpu::RenderState) {
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
        let Some(m) = &self.model else { return };
        let aspect = f64::from(rect.width() / rect.height().max(1.0));
        // A model just opened is fitted once, where the view's shape is known.
        if r.double_clicked() || (!self.fitted && m.scene.bounds.is_some()) {
            self.camera.fit(m.scene.bounds, aspect);
            self.fitted = true;
        }
        let anchor = m.scene.anchor();
        let radius = m.scene.bounds.map_or(1.0, |(lo, hi)| (hi - lo).dot(hi - lo).sqrt() * 0.5);
        let (view_projection, eye) = self.camera.matrices(anchor, aspect, radius);
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
                opened: false,
                rebuild_ms: 0.0,
                scene_ms: 0.0,
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
        assert!(matches!(&sent, Ok(Request::Edit(name, fields)) if name == "panel" && *fields == json!({"side": "right"})), "an edit of the side");
    }
}
