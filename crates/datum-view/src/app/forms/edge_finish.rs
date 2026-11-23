//! The Edge tool of the model view (D295, D296): the sketch fillet's keys (D240, D283)
//! over the part's straight corners and flat caps. A trial follows the selection until Enter.

use std::collections::BTreeSet;
use std::sync::Arc;

use datum_brep::TopoId;
use datum_math::Vec3;
use datum_math::expr::{ExprError, Num};
use datum_ops::EdgeMode;
use serde_json::{Value as Json, json};

use crate::actions::Act;
use crate::app::{Request, Viewer, definition, in_rect, sphere};
use crate::dims::value_of;
use crate::scene::{BodyMesh, Scene};

/// What the cursor is over: a corner edge, or a flat cap with its corner edges.
pub(in crate::app) enum EdgeHover {
    Edge { body: usize, id: TopoId, ends: [Vec3; 2] },
    Face { body: usize, id: TopoId, edges: Vec<[Vec3; 2]> },
}

impl EdgeHover {
    fn body(&self) -> usize {
        match self {
            Self::Edge { body, .. } | Self::Face { body, .. } => *body,
        }
    }

    fn lines(&self) -> Vec<[Vec3; 2]> {
        match self {
            Self::Edge { ends, .. } => vec![*ends],
            Self::Face { edges, .. } => edges.clone(),
        }
    }
}

#[derive(Default)]
struct Targets {
    edges: BTreeSet<TopoId>,
    faces: BTreeSet<TopoId>,
    excluded: BTreeSet<TopoId>,
}

impl Targets {
    fn from_feature(feature: &Json) -> Self {
        let read = |list: &str, single: &str| {
            feature[list].as_array().into_iter().flatten().chain(feature.get(single)).filter_map(|v| TopoId::parse(v.as_str()?)).collect()
        };
        Self { edges: read("edges", "edge"), faces: read("faces", "face"), excluded: read("exclude_edges", "") }
    }

    fn face_edges<'a>(faces: &'a BTreeSet<TopoId>, mesh: &'a BodyMesh) -> impl Iterator<Item = TopoId> + 'a {
        faces.iter().filter_map(|face| mesh.finish_edges_of.get(face)).flatten().map(|&ix| mesh.finish_edges[ix].id)
    }

    fn effective(&self, mesh: &BodyMesh) -> BTreeSet<TopoId> {
        self.edges.iter().copied().chain(Self::face_edges(&self.faces, mesh)).filter(|id| !self.excluded.contains(id)).collect()
    }

    fn click(&mut self, hover: &EdgeHover, mesh: &BodyMesh, adding: bool) {
        if !adding {
            *self = Self::default();
        }
        match *hover {
            EdgeHover::Face { id, .. } => {
                if !self.faces.remove(&id) {
                    self.faces.insert(id);
                }
                self.excluded.retain(|id| self.edges.contains(id) || Self::face_edges(&self.faces, mesh).any(|edge| edge == *id));
            }
            EdgeHover::Edge { id, .. } => {
                let via_face = Self::face_edges(&self.faces, mesh).any(|edge| edge == id);
                if self.excluded.remove(&id) {
                    if !via_face {
                        self.edges.insert(id);
                    }
                } else if self.edges.remove(&id) || via_face {
                    if via_face {
                        self.excluded.insert(id);
                    }
                } else {
                    self.edges.insert(id);
                }
            }
        }
    }

    fn write(&self, mut params: Json) -> Json {
        let ids = |set: &BTreeSet<TopoId>| set.iter().map(|id| format!("{id:?}")).collect::<Vec<_>>();
        params["edges"] = json!(ids(&self.edges));
        params["faces"] = json!(ids(&self.faces));
        params["exclude_edges"] = json!(ids(&self.excluded));
        params
    }
}

struct Selection {
    body: String,
    targets: Targets,
}

/// A finished set edited from the tree, with the size as typed.
struct Edit {
    feature: String,
    selection: Selection,
    mode: EdgeMode,
    text: String,
    fresh: bool,
}

impl Edit {
    fn params(&self) -> Json {
        self.selection.targets.write(json!({
            "name": self.feature, "edge": "", "face": "", "size": value_of(&self.text), "mode": self.mode
        }))
    }
}

pub(in crate::app) struct EdgeFinishForm {
    selection: Option<Selection>,
    size: Option<(Json, f64)>,
    chamfer: bool,
    field: Option<String>,
    edit: Option<Edit>,
    sent: Option<(&'static str, Json)>,
    applying: bool,
}

impl EdgeFinishForm {
    pub(in crate::app) fn editing(name: String, feature: &Json) -> Self {
        let mode = serde_json::from_value(feature["mode"].clone()).unwrap_or_default();
        Self {
            selection: None,
            size: None,
            chamfer: mode == EdgeMode::Chamfer,
            field: None,
            edit: Some(Edit {
                feature: name,
                selection: Selection { body: feature["inputs"][0].as_str().unwrap_or_default().into(), targets: Targets::from_feature(feature) },
                mode,
                text: definition(&feature["size"]),
                fresh: true,
            }),
            sent: None,
            applying: false,
        }
    }

    pub(in crate::app) fn edited(&self) -> Option<&String> {
        self.edit.as_ref().map(|e| &e.feature)
    }
}

impl Viewer {
    pub(in crate::app) fn open_edge_finish(&mut self) {
        if self.docs[self.active].edge_finish.is_some() {
            self.close_edge_finish();
            return;
        }
        self.leave();
        self.docs[self.active].flange = None;
        self.docs[self.active].flange_edges.clear();
        self.docs[self.active].picking = None;
        self.docs[self.active].under = None;
        self.docs[self.active].plane_draft = None;
        self.docs[self.active].edge_hover = None;
        self.docs[self.active].edge_finish =
            Some(EdgeFinishForm { selection: None, size: None, chamfer: false, field: None, edit: None, sent: None, applying: false });
    }

    pub(in crate::app) fn close_edge_finish(&mut self) {
        if self.docs[self.active].edge_finish.take().is_some() {
            self.docs[self.active].edge_hover = None;
            self.send(Request::Preview(None));
        }
    }

    pub(in crate::app) fn edge_finish_applied(&mut self, applied: bool) {
        let Some(f) = self.docs[self.active].edge_finish.as_mut().filter(|f| f.applying) else { return };
        if applied && f.edit.is_some() {
            self.docs[self.active].edge_finish = None;
            self.docs[self.active].edge_hover = None;
        } else {
            f.applying = false;
            f.sent = None;
            if applied {
                f.selection = None;
            }
        }
    }

    pub(in crate::app) fn edge_finish_is_picking(&self) -> bool {
        self.docs[self.active].edge_finish.is_some()
    }

    /// The edit picks its own input, even if later operations removed some of its edges.
    fn edge_finish_meshes(&self) -> Option<&[Arc<BodyMesh>]> {
        let base = &self.docs[self.active].model.as_ref()?.edge_base;
        if let Some(edit) = self.docs[self.active].edge_finish.as_ref()?.edit.as_ref() {
            let (name, mesh) = base.edit.as_ref()?;
            (name == &edit.feature).then(|| std::slice::from_ref(mesh))
        } else {
            Some(&base.meshes)
        }
    }

    /// The hover and, on a click, the change to the draft selection.
    pub(in crate::app) fn edge_finish_picker(&mut self, rect: egui::Rect, response: &egui::Response) -> Option<Vec<[Vec3; 2]>> {
        self.docs[self.active].edge_hover =
            response.hover_pos().or(response.interact_pointer_pos()).and_then(|cursor| self.edge_finish_at(rect, cursor));
        if response.clicked() {
            let adding = response.ctx.input(|i| crate::select::adding(&i.modifiers));
            self.edge_finish_click(adding);
        }
        self.docs[self.active].edge_hover.as_ref().map(EdgeHover::lines)
    }

    fn edge_finish_click(&mut self, adding: bool) {
        let Some(body) = self.docs[self.active].edge_hover.as_ref().map(EdgeHover::body) else { return };
        let Some(mesh) = self.edge_finish_meshes().and_then(|meshes| meshes.get(body)).cloned() else { return };
        let d = &mut self.docs[self.active];
        let Some(hover) = &d.edge_hover else { return };
        let Some(f) = d.edge_finish.as_mut().filter(|f| !f.applying) else { return };
        if let Some(edit) = &mut f.edit {
            if adding {
                edit.selection.targets.click(hover, &mesh, true);
            }
            return;
        }
        let Some(body) = d.model.as_ref().and_then(|m| m.edge_base.bodies.get(hover.body())) else { return };
        if let Some(selection) = &mut f.selection {
            if &selection.body == body {
                selection.targets.click(hover, &mesh, adding);
                return;
            }
            if adding {
                return;
            }
        }
        let mut targets = Targets::default();
        targets.click(hover, &mesh, false);
        f.selection = Some(Selection { body: body.clone(), targets });
    }

    pub(in crate::app) fn edge_finish_selection(&self) -> Option<Vec<[Vec3; 2]>> {
        let f = self.docs[self.active].edge_finish.as_ref()?;
        let (mesh, targets) = if let Some(edit) = &f.edit {
            (self.edge_finish_meshes()?.first()?, &edit.selection.targets)
        } else {
            let selection = f.selection.as_ref()?;
            let base = &self.docs[self.active].model.as_ref()?.edge_base;
            let ix = base.bodies.iter().position(|body| body == &selection.body)?;
            (base.meshes.get(ix)?, &selection.targets)
        };
        let selected = targets.effective(mesh);
        Some(mesh.finish_edges.iter().filter(|edge| selected.contains(&edge.id)).map(|edge| edge.ends).collect())
    }

    fn preview_params(&self) -> Option<(&'static str, Json)> {
        let f = self.docs[self.active].edge_finish.as_ref()?;
        if let Some(edit) = &f.edit {
            return Some(("edit", edit.params()));
        }
        let (size, _) = f.size.as_ref()?;
        let base = &self.docs[self.active].model.as_ref()?.edge_base;
        let hovered;
        let selection = if let Some(selection) = &f.selection {
            let ix = base.bodies.iter().position(|name| name == &selection.body)?;
            if selection.targets.effective(base.meshes.get(ix)?).is_empty() {
                return None;
            }
            selection
        } else {
            let hover = self.docs[self.active].edge_hover.as_ref()?;
            let mesh = base.meshes.get(hover.body())?;
            let mut targets = Targets::default();
            targets.click(hover, mesh, false);
            hovered = Selection { body: base.bodies.get(hover.body())?.clone(), targets };
            &hovered
        };
        let mode = if f.chamfer { EdgeMode::Chamfer } else { EdgeMode::Fillet };
        Some((
            "add",
            selection.targets.write(json!({
                "name": base.next_name, "op": "edge_finish", "inputs": [selection.body], "size": size, "mode": mode
            })),
        ))
    }

    fn edge_finish_apply(&mut self, command: &'static str, params: Json) {
        if let Some(f) = self.docs[self.active].edge_finish.as_mut().filter(|f| !f.applying)
            && f.sent.as_ref().is_some_and(|(c, p)| *c == command && *p == params)
        {
            f.applying = true;
            self.send(Request::ApplyEdgeFinish(command, params));
        }
    }

    fn edge_finish_preview(&mut self) {
        if self.docs[self.active].edge_finish.as_ref().is_none_or(|f| f.applying) {
            return;
        }
        let params = self.preview_params();
        if let Some(f) = &mut self.docs[self.active].edge_finish
            && f.sent != params
        {
            f.sent.clone_from(&params);
            self.send(Request::Preview(params));
        }
    }

    /// Nearest visible candidate within mouse reach, in screen points. Check depth at the
    /// nearest point on the edge so a back edge crossing a visible face cannot take the click.
    /// Else the flat cap under the cursor, by its corners.
    fn edge_finish_at(&self, rect: egui::Rect, cursor: egui::Pos2) -> Option<EdgeHover> {
        let m = self.docs[self.active].model.as_ref()?;
        let half = rect.size() * 0.5;
        if half.x <= 0.0 || half.y <= 0.0 {
            return None;
        }
        let (anchor, radius) = sphere(&m.scene);
        let aspect = f64::from(half.x / half.y);
        let (vp, _) = self.docs[self.active].camera.matrices(anchor, aspect, radius);
        // The bodies without a standing preview: the previewed one has the hovered corner
        // rounded, and picking it would drop the hover and flicker.
        let base = self.edge_finish_meshes()?;
        let near = |edge: &crate::scene::FinishEdge| {
            let clips = edge.ends.map(|p| crate::camera::clip(&vp, p - anchor));
            if clips.iter().any(|p| p[3] <= 0.0) {
                return None;
            }
            let [a, b] = clips.map(|p| in_rect(p, rect));
            let line = b - a;
            let length = line.length_sq();
            if length == 0.0 {
                return None;
            }
            let t = ((cursor - a).dot(line) / length).clamp(0.0, 1.0);
            let point = a + line * t;
            let distance = point.distance(cursor);
            if distance >= 6.0 {
                return None;
            }
            let at = point - rect.center();
            let (origin, direction) = self.docs[self.active].camera.ray(f64::from(at.x / half.x), f64::from(-at.y / half.y), aspect, anchor, radius);
            if edge.normals.iter().all(|n| n.dot(direction) >= 0.0) {
                return None;
            }
            // Recover the point on the world-space edge from its screen interpolation.
            let start = f64::from((1.0 - t) / clips[0][3]);
            let end = f64::from(t / clips[1][3]);
            let p = edge.ends[0] + (edge.ends[1] - edge.ends[0]) * (end / (start + end));
            let depth = (p - origin).dot(direction);
            // The screen projection rounded its matrix to f32.
            let rounding = 32.0 * f64::from(f32::EPSILON) * radius.max(depth);
            if Scene::pick_in(base, origin, direction).is_some_and(|hit| hit.distance + rounding < depth) {
                return None;
            }
            Some((distance, depth))
        };
        let edge = base
            .iter()
            .enumerate()
            .flat_map(|(body, mesh)| mesh.finish_edges.iter().filter_map(move |&edge| Some((body, edge, near(&edge)?))))
            .min_by(|a, b| a.2.0.total_cmp(&b.2.0).then_with(|| a.2.1.total_cmp(&b.2.1)));
        if let Some((body, edge, _)) = edge {
            return Some(EdgeHover::Edge { body, id: edge.id, ends: edge.ends });
        }
        let at = cursor - rect.center();
        let (origin, direction) = self.docs[self.active].camera.ray(f64::from(at.x / half.x), f64::from(-at.y / half.y), aspect, anchor, radius);
        let hit = Scene::pick_in(base, origin, direction)?;
        let mesh = base.get(hit.body)?;
        let corners = mesh.finish_edges_of.get(&hit.face).filter(|corners| !corners.is_empty())?;
        Some(EdgeHover::Face { body: hit.body, id: hit.face, edges: corners.iter().map(|&i| mesh.finish_edges[i].ends).collect() })
    }

    /// The tool's frame: the edited set's window, else the field by the cursor and the keys.
    pub(in crate::app) fn edge_finish_ui(&mut self, ctx: &egui::Context) {
        if self.docs[self.active].edge_finish.as_ref().is_none_or(|f| f.applying) {
            return;
        }
        if self.docs[self.active].edge_finish.as_ref().is_some_and(|f| f.edit.is_some()) {
            self.edge_finish_edit_window(ctx);
            self.edge_finish_preview();
            return;
        }
        let typing = ctx.text_edit_focused();
        let had_field = self.docs[self.active].edge_finish.as_ref().is_some_and(|f| f.field.is_some());
        let mut close = false;
        // Enter keeps the typed size, Escape closes its field.
        if let Some(text) = self.docs[self.active].edge_finish.as_mut().and_then(|f| f.field.as_mut()) {
            let mut done = (false, false);
            egui::Area::new(egui::Id::new("datum_edge_finish_field"))
                .fixed_pos(ctx.input(|i| i.pointer.latest_pos().unwrap_or_default()) + egui::vec2(14.0, 14.0))
                .order(egui::Order::Foreground)
                .show(ctx, |ui| {
                    let field = egui::TextEdit::singleline(text).id(egui::Id::new("datum_edge_finish_text")).desired_width(110.0).show(ui);
                    field.response.request_focus();
                    done = ui.input(|i| (i.key_pressed(egui::Key::Enter), i.key_pressed(egui::Key::Escape)));
                });
            let (enter, escape) = done;
            if enter || escape {
                let text = self.docs[self.active].edge_finish.as_mut().and_then(|f| f.field.take()).unwrap_or_default();
                if enter && !text.trim().is_empty() {
                    let value = value_of(&text);
                    let size = self.resolved(&value);
                    if let Some(form) = self.docs[self.active].edge_finish.as_mut() {
                        form.size = Some((value, size));
                    }
                }
            }
        }
        // The keys, only with no field open and no text edit of another window focused.
        if !had_field && !typing {
            let (escape, enter, tab, up, down) = ctx.input_mut(|i| {
                (
                    i.key_pressed(egui::Key::Escape),
                    i.key_pressed(egui::Key::Enter),
                    i.consume_key(egui::Modifiers::NONE, egui::Key::Tab),
                    i.consume_key(egui::Modifiers::NONE, egui::Key::ArrowUp),
                    i.consume_key(egui::Modifiers::NONE, egui::Key::ArrowDown),
                )
            });
            if escape {
                close = true;
            }
            if tab && let Some(form) = self.docs[self.active].edge_finish.as_mut() {
                form.chamfer = !form.chamfer;
            }
            if up || down {
                let current = self.docs[self.active].edge_finish.as_ref().and_then(|f| f.size.as_ref()).map_or(f64::NAN, |(_, v)| *v);
                let chamfer = self.docs[self.active].edge_finish.as_ref().is_some_and(|f| f.chamfer);
                let chamfers: Vec<f64> = self.chamfer_presets.iter().map(|&(a, _)| a).collect();
                let presets = if chamfer { &chamfers } else { &self.fillet_presets };
                if let Some(size) = crate::draw::preset_step(presets, current, up)
                    && let Some(form) = self.docs[self.active].edge_finish.as_mut()
                {
                    form.size = Some((json!(size), size));
                }
            }
            let first = ctx.input(|i| i.events.iter().find_map(|e| if let egui::Event::Text(t) = e { Some(t.clone()) } else { None }));
            if let Some(t) = first.filter(|t| t.starts_with(|c: char| c.is_ascii_digit() || c == '.' || c == '-'))
                && let Some(form) = self.docs[self.active].edge_finish.as_mut()
            {
                form.field = Some(t);
            } else if enter
                && !escape
                && let Some((command, params)) = self.preview_params()
            {
                self.edge_finish_preview();
                self.edge_finish_apply(command, params);
            }
        }
        if close {
            self.close_edge_finish();
            return;
        }
        self.edge_finish_preview();
    }

    /// A number as typed, as it stands with the model's parameters.
    fn resolved(&self, value: &Json) -> f64 {
        serde_json::from_value::<Num>(value.clone())
            .ok()
            .and_then(|n| {
                n.resolve(&|name: &str| {
                    self.docs[self.active]
                        .model
                        .as_ref()
                        .and_then(|m| m.inspect["parameters"][name]["value"].as_f64())
                        .ok_or_else(|| ExprError::Unknown(name.into()))
                })
                .ok()
            })
            .unwrap_or(f64::NAN)
    }

    /// The window that edits a finished set from the tree: its corners or caps, the size and
    /// the mode, and clicks on the part to take targets in or out.
    fn edge_finish_edit_window(&mut self, ctx: &egui::Context) {
        let selected = self.edge_finish_selection().map_or(0, |lines| lines.len());
        let before = self.preview_params().map(|(_, params)| params);
        let d = &mut self.docs[self.active];
        let preview = before.as_ref().and_then(|p| d.model.as_ref()?.preview.as_ref().filter(|(q, _)| q == p).map(|(_, r)| r));
        let ready = preview.is_some_and(Result::is_ok);
        let enter = ctx.input_mut(|i| i.consume_key(egui::Modifiers::NONE, egui::Key::Enter));
        let mut accept = false;
        let mut cancel = false;
        if let Some(f) = &mut d.edge_finish {
            let Some(edit) = &mut f.edit else { return };
            let tab = ctx.input_mut(|i| i.consume_key(egui::Modifiers::NONE, egui::Key::Tab));
            if tab {
                edit.mode = match edit.mode {
                    EdgeMode::Fillet => EdgeMode::Chamfer,
                    EdgeMode::Chamfer => EdgeMode::Fillet,
                };
            }
            let (up, down, escape) = ctx.input_mut(|i| {
                (
                    i.consume_key(egui::Modifiers::NONE, egui::Key::ArrowUp),
                    i.consume_key(egui::Modifiers::NONE, egui::Key::ArrowDown),
                    i.key_pressed(egui::Key::Escape),
                )
            });
            if up || down {
                let lookup = |name: &str| {
                    d.model.as_ref().and_then(|m| m.inspect["parameters"][name]["value"].as_f64()).ok_or_else(|| ExprError::Unknown(name.into()))
                };
                let current = serde_json::from_value::<Num>(value_of(&edit.text)).ok().and_then(|n| n.resolve(&lookup).ok()).unwrap_or(f64::NAN);
                let chamfers: Vec<f64> = self.chamfer_presets.iter().map(|&(a, _)| a).collect();
                let presets = if edit.mode == EdgeMode::Chamfer { &chamfers } else { &self.fillet_presets };
                if let Some(size) = crate::draw::preset_step(presets, current, up) {
                    edit.text = definition(&json!(size));
                }
            }
            if escape {
                cancel = true;
            }
            egui::Window::new(self.i18n.label(Act::EdgeFinish))
                .collapsible(false)
                .resizable(false)
                .anchor(egui::Align2::CENTER_TOP, [0.0, 60.0])
                .show(ctx, |ui| {
                    ui.label(&edit.selection.body);
                    ui.label(self.i18n.f("edge-finish-corners", "Corners: { $n }", &[("n", &selected.to_string())]));
                    ui.label(self.i18n.t("edge-finish-toggle", "Shift/Ctrl + click to add or remove a corner or cap"));
                    ui.horizontal(|ui| {
                        ui.selectable_value(&mut edit.mode, EdgeMode::Fillet, self.i18n.t("edge-finish-fillet", "Fillet"));
                        ui.selectable_value(&mut edit.mode, EdgeMode::Chamfer, self.i18n.t("edge-finish-chamfer", "Chamfer"));
                    });
                    ui.horizontal(|ui| {
                        let label = match edit.mode {
                            EdgeMode::Fillet => self.i18n.t("edge-finish-radius", "Radius, mm"),
                            EdgeMode::Chamfer => self.i18n.t("edge-finish-distance", "Distance, mm"),
                        };
                        ui.label(label);
                        let mut field = egui::TextEdit::singleline(&mut edit.text).desired_width(90.0).show(ui);
                        if std::mem::take(&mut edit.fresh) {
                            field.response.request_focus();
                            let all = egui::text::CCursorRange::two(egui::text::CCursor::new(0), egui::text::CCursor::new(edit.text.chars().count()));
                            field.state.cursor.set_char_range(Some(all));
                            field.state.store(ctx, field.response.id);
                        }
                    });
                    if let Some(Err(why)) = preview {
                        ui.colored_label(ui.visuals().error_fg_color, why);
                    }
                    ui.horizontal(|ui| {
                        accept = ui.add_enabled(ready, egui::Button::new(self.i18n.t("ok", "OK"))).clicked();
                        cancel |= ui.button(self.i18n.t("cancel", "Cancel")).clicked();
                    });
                });
        }
        let after = self.preview_params().map(|(_, params)| params);
        if cancel {
            self.close_edge_finish();
        } else if (accept || enter)
            && after == before
            && let Some(after) = after
        {
            self.edge_finish_preview();
            self.edge_finish_apply("edit", after);
        }
    }

    /// The hint by the cursor, the presets beside it, and a preview that refused, under them.
    pub(in crate::app) fn edge_finish_hints(&self, ui: &egui::Ui, painter: &egui::Painter, cursor: egui::Pos2) {
        let Some(f) = self.docs[self.active].edge_finish.as_ref() else { return };
        if f.edit.is_some() || f.field.is_some() {
            return;
        }
        crate::draw::size_hint(&self.i18n, painter, cursor, f.size.as_ref().map(|(_, v)| *v), f.chamfer, &self.chamfer_presets, &self.fillet_presets);
        if let Some(why) = self.docs[self.active].model.as_ref().and_then(|m| m.preview.as_ref()).and_then(|(_, r)| r.as_ref().err()) {
            painter.text(cursor + egui::vec2(12.0, 48.0), egui::Align2::LEFT_TOP, why, egui::FontId::proportional(13.0), ui.visuals().error_fg_color);
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use std::sync::mpsc::channel;
    use std::time::{Duration, Instant};

    use super::*;
    use crate::app::tests::viewer;

    fn worker_viewer() -> Viewer {
        let (mut v, _) = viewer("parametric.json");
        let (requests, inbox) = channel();
        let (outbox, models) = channel();
        v.docs[v.active].requests = requests;
        v.docs[v.active].models = models;
        v.docs[v.active].model = None;
        std::thread::spawn(move || crate::app::worker(inbox, outbox, egui::Context::default()));
        let model: Json = serde_json::from_str(include_str!("../../../../../tests/models/edge-finish-set.json")).unwrap();
        for feature in &model["features"].as_array().unwrap()[..2] {
            v.send(Request::Command("add", feature.clone()));
        }
        wait_for(&mut v, |v| v.docs[v.active].model.as_ref().is_some_and(|m| m.bodies == ["box"]));
        v
    }

    fn wait_for(v: &mut Viewer, ready: impl Fn(&Viewer) -> bool) {
        let until = Instant::now() + Duration::from_secs(10);
        loop {
            v.receive();
            if ready(v) {
                return;
            }
            assert!(Instant::now() < until, "worker did not reach the expected state");
            std::thread::sleep(Duration::from_millis(2));
        }
    }

    fn hover_corner(v: &mut Viewer, x: f64, y: f64) -> TopoId {
        let mesh = &v.edge_finish_meshes().unwrap()[0];
        let edge = *mesh.finish_edges.iter().find(|e| e.ends.iter().all(|p| p.x == x && p.y == y)).unwrap();
        v.docs[v.active].edge_hover = Some(EdgeHover::Edge { body: 0, id: edge.id, ends: edge.ends });
        edge.id
    }

    fn preview(v: &mut Viewer) -> Json {
        let (_, params) = v.preview_params().unwrap();
        v.edge_finish_preview();
        wait_for(v, |v| v.docs[v.active].model.as_ref().and_then(|m| m.preview.as_ref()).is_some_and(|(p, result)| p == &params && result.is_ok()));
        params
    }

    fn press_enter(v: Viewer) -> Viewer {
        let mut h = egui_kittest::Harness::builder().with_size([600.0, 400.0]).build_ui_state(|ui, v: &mut Viewer| v.edge_finish_ui(ui.ctx()), v);
        h.key_press(egui::Key::Enter);
        h.run();
        h.into_state()
    }

    fn type_size(v: Viewer, text: &str) -> Viewer {
        let mut h = egui_kittest::Harness::builder().with_size([600.0, 400.0]).build_ui_state(|ui, v: &mut Viewer| v.edge_finish_ui(ui.ctx()), v);
        h.event(egui::Event::Text(text.into()));
        h.run();
        h.key_press(egui::Key::Enter);
        h.run();
        h.into_state()
    }

    #[test]
    fn the_worker_keeps_the_pick_stable_and_chains_two_radii() {
        let mut v = worker_viewer();
        v.open_edge_finish();
        v = type_size(v, "1.2");
        let first = hover_corner(&mut v, 20.0, 0.0);
        let rect = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(600.0, 400.0));
        {
            let d = &mut v.docs[v.active];
            d.camera.fit(d.model.as_ref().unwrap().scene.bounds, 1.5);
        }
        let cursor = {
            let (anchor, radius) = sphere(&v.docs[v.active].model.as_ref().unwrap().scene);
            let (vp, _) = v.docs[v.active].camera.matrices(anchor, 1.5, radius);
            in_rect(crate::camera::clip(&vp, Vec3::new(20.0, 0.0, 5.0) - anchor), rect)
        };
        v.docs[v.active].edge_hover = v.edge_finish_at(rect, cursor);
        assert!(matches!(v.docs[v.active].edge_hover, Some(EdgeHover::Edge { id, .. }) if id == first));
        let add = preview(&mut v);
        let model = v.docs[v.active].model.as_ref().unwrap();
        assert_eq!(model.bodies, ["edge_finish1"]);
        assert_eq!(model.edge_base.bodies, ["box"]);
        assert!(!model.inspect["features"].as_array().unwrap().iter().any(|f| f["name"] == "edge_finish1"));
        assert!(!model.scene.bodies[0].finish_edges.iter().any(|e| e.id == first));
        v.docs[v.active].edge_hover = v.edge_finish_at(rect, cursor);
        assert!(matches!(v.docs[v.active].edge_hover, Some(EdgeHover::Edge { id, .. }) if id == first));
        for _ in 0..3 {
            assert_eq!(v.preview_params(), Some(("add", add.clone())));
            v.edge_finish_preview();
            assert!(!v.docs[v.active].busy);
        }
        v = press_enter(v);
        wait_for(&mut v, |v| v.docs[v.active].model.as_ref().is_some_and(|m| m.edge_base.bodies == ["edge_finish1"]));
        assert!(v.docs[v.active].model.as_ref().unwrap().preview.is_none());

        v = type_size(v, "2");
        let other = hover_corner(&mut v, 0.0, 0.0);
        assert_ne!(other, first);
        let add = preview(&mut v);
        assert_eq!(add["inputs"], json!(["edge_finish1"]));
        assert_eq!(add["name"], "edge_finish2");
        v = press_enter(v);
        wait_for(&mut v, |v| v.docs[v.active].model.as_ref().is_some_and(|m| m.edge_base.bodies == ["edge_finish2"]));
        let model = v.docs[v.active].model.as_ref().unwrap();
        assert_eq!(model.scene.bodies.len(), 1);
        assert!(model.preview.is_none());
        let features = model.inspect["features"].as_array().unwrap();
        assert_eq!(features.iter().filter(|f| f["op"] == "edge_finish").count(), 2);
        assert_eq!(features.last().unwrap()["inputs"], json!(["edge_finish1"]));
        assert!(!model.scene.bodies[0].finish_edges.iter().any(|e| e.id == other || e.id == first));

        let name = "edge_finish2".to_string();
        v.docs[v.active].edge_finish = Some(EdgeFinishForm::editing(name.clone(), features.last().unwrap()));
        v.send(Request::Using(Some(name)));
        wait_for(&mut v, |v| v.edge_finish_meshes().is_some());
        assert_eq!(hover_corner(&mut v, 0.0, 0.0), other, "editing keeps the second feature's actual input");
    }

    #[test]
    fn a_face_selection_can_drop_one_corner_and_restore_it_in_edit() {
        let mut v = worker_viewer();
        v.open_edge_finish();
        v.docs[v.active].edge_finish.as_mut().unwrap().size = Some((json!(1.2), 1.2));
        let mesh = &v.edge_finish_meshes().unwrap()[0];
        let (face, corners) = mesh
            .finish_edges_of
            .iter()
            .find(|(_, corners)| {
                corners.len() == 4
                    && corners.iter().all(|&ix| {
                        let [a, b] = mesh.finish_edges[ix].ends;
                        a.x == b.x && a.y == b.y
                    })
            })
            .unwrap();
        v.docs[v.active].edge_hover =
            Some(EdgeHover::Face { body: 0, id: *face, edges: corners.iter().map(|&ix| mesh.finish_edges[ix].ends).collect() });
        v.edge_finish_click(false);
        assert_eq!(v.edge_finish_selection().unwrap().len(), 4);
        let excluded = hover_corner(&mut v, 0.0, 0.0);
        v.edge_finish_click(true);
        assert_eq!(v.edge_finish_selection().unwrap().len(), 3);
        v.docs[v.active].edge_hover = None;
        let params = preview(&mut v);
        assert_eq!(params["exclude_edges"], json!([format!("{excluded:?}")]));
        assert_eq!(params["faces"].as_array().unwrap().len(), 1);
        assert!(v.docs[v.active].model.as_ref().unwrap().inspect["features"].as_array().unwrap().iter().all(|f| f["op"] != "edge_finish"));
        v = press_enter(v);
        wait_for(&mut v, |v| v.docs[v.active].model.as_ref().is_some_and(|m| m.edge_base.bodies == ["edge_finish1"]));

        let name = "edge_finish1".to_string();
        let feature = v.docs[v.active].model.as_ref().unwrap().inspect["features"].as_array().unwrap().last().unwrap();
        v.docs[v.active].edge_finish = Some(EdgeFinishForm::editing(name.clone(), feature));
        v.send(Request::Using(Some(name.clone())));
        wait_for(&mut v, |v| v.edge_finish_meshes().is_some());
        assert_eq!(v.edge_finish_selection().unwrap().len(), 3);
        assert_eq!(hover_corner(&mut v, 0.0, 0.0), excluded);
        v.edge_finish_click(false);
        assert_eq!(v.edge_finish_selection().unwrap().len(), 3);
        v.edge_finish_click(true);
        assert_eq!(v.edge_finish_selection().unwrap().len(), 4);
        let edit = preview(&mut v);
        assert_eq!(edit["exclude_edges"], json!([]));
        v = press_enter(v);
        wait_for(&mut v, |v| v.docs[v.active].edge_finish.is_none());
        assert!(v.docs[v.active].model.as_ref().unwrap().preview.is_none());
        assert_eq!(v.docs[v.active].model.as_ref().unwrap().bodies, [name]);
    }
}
