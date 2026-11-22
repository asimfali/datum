//! The Edge tool of the model view (D295, D296): the sketch fillet's keys (D240, D283) over the
//! part's straight corners and flat caps. Clicks of one size and mode build a set; each set is
//! one `edge_finish` feature, made as soon as its first click lands, extended by the next clicks
//! of the set and shortened by a click on a member. The window only edits a finished set from
//! the tree.

use datum_brep::TopoId;
use datum_math::Vec3;
use datum_math::expr::{ExprError, Num};
use datum_ops::EdgeMode;
use serde_json::{Value as Json, json};

use crate::actions::Act;
use crate::app::{Request, Viewer, definition, in_rect, sphere};
use crate::dims::value_of;
use crate::scene::Scene;

/// What one set holds: single corners, or whole flat caps by themselves.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(in crate::app) enum Kind {
    Edges,
    Faces,
}

/// What the cursor is over: a corner edge, or a flat cap with its corner edges.
#[derive(Clone, Debug)]
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

    fn id(&self) -> TopoId {
        match self {
            Self::Edge { id, .. } | Self::Face { id, .. } => *id,
        }
    }

    fn kind(&self) -> Kind {
        match self {
            Self::Edge { .. } => Kind::Edges,
            Self::Face { .. } => Kind::Faces,
        }
    }

    /// The world-space lines to light while the cursor is here.
    fn lines(&self) -> Vec<[Vec3; 2]> {
        match self {
            Self::Edge { ends, .. } => vec![*ends],
            Self::Face { edges, .. } => edges.clone(),
        }
    }
}

/// A set of one size and mode: the feature made for it, its body, and what it holds.
pub(in crate::app) struct Set {
    feature: String,
    body: String,
    kind: Kind,
    targets: Vec<TopoId>,
    size: Json,
    mode: EdgeMode,
}

impl Set {
    fn ids(&self, targets: &[TopoId]) -> Vec<Json> {
        targets.iter().map(|t| json!(format!("{t:?}"))).collect()
    }

    /// The `add` that makes the set.
    fn add_params(&self) -> Json {
        let ids = self.ids(&self.targets);
        match self.kind {
            Kind::Edges => {
                json!({"name": self.feature, "op": "edge_finish", "inputs": [self.body], "edges": ids, "size": self.size, "mode": self.mode})
            }
            Kind::Faces => {
                json!({"name": self.feature, "op": "edge_finish", "inputs": [self.body], "faces": ids, "size": self.size, "mode": self.mode})
            }
        }
    }

    /// The `edit` that rebuilds the set with `targets`: the single `edge` cleared, one of the
    /// lists filled.
    fn edit_params(&self, targets: &[TopoId]) -> Json {
        let ids = self.ids(targets);
        let mut params = json!({"name": self.feature, "edge": "", "edges": [], "faces": []});
        params[match self.kind {
            Kind::Edges => "edges",
            Kind::Faces => "faces",
        }] = json!(ids);
        params
    }

    /// The `edit` that also retypes the size and the mode (the edit window's OK).
    fn edit_all_params(&self) -> Json {
        let mut params = self.edit_params(&self.targets);
        params["size"] = self.size.clone();
        params["mode"] = json!(self.mode);
        params
    }

    /// The set continues while the click keeps the body, the kind, the size and the mode.
    fn continues(&self, body: &str, kind: Kind, size: &Json, mode: EdgeMode) -> bool {
        self.body == body && self.kind == kind && self.size == *size && self.mode == mode
    }
}

/// A finished set edited from the tree: the set as it stands, the size as typed, and whether its
/// field is fresh.
pub(in crate::app) struct Edit {
    set: Set,
    text: String,
    fresh: bool,
}

/// The tool: the size and mode kept between sets (D240), the set the clicks build, the size
/// field open by the cursor, and the edit of a finished set.
pub(in crate::app) struct EdgeFinishForm {
    open: Option<Set>,
    size: Option<(Json, f64)>,
    chamfer: bool,
    field: Option<String>,
    edit: Option<Edit>,
    sent: Option<(&'static str, Json)>,
}

fn parse_id(value: &str) -> Option<TopoId> {
    TopoId::parse(value)
}

impl EdgeFinishForm {
    pub(in crate::app) fn editing(name: String, feature: &Json) -> Self {
        let (kind, targets) = if let Some(edges) = feature["edges"].as_array().filter(|e| !e.is_empty()) {
            (Kind::Edges, edges.iter().filter_map(|v| v.as_str().and_then(parse_id)).collect())
        } else if let Some(faces) = feature["faces"].as_array().filter(|e| !e.is_empty()) {
            (Kind::Faces, faces.iter().filter_map(|v| v.as_str().and_then(parse_id)).collect())
        } else {
            (Kind::Edges, feature["edge"].as_str().and_then(parse_id).into_iter().collect())
        };
        let set = Set {
            feature: name,
            body: feature["inputs"][0].as_str().unwrap_or_default().into(),
            kind,
            targets,
            size: feature["size"].clone(),
            mode: serde_json::from_value(feature["mode"].clone()).unwrap_or_default(),
        };
        Self {
            open: None,
            size: None,
            chamfer: set.mode == EdgeMode::Chamfer,
            field: None,
            edit: Some(Edit { text: definition(&feature["size"]), fresh: true, set }),
            sent: None,
        }
    }

    pub(in crate::app) fn edited(&self) -> Option<&String> {
        self.edit.as_ref().map(|e| &e.set.feature)
    }
}

impl Viewer {
    pub(in crate::app) fn open_edge_finish(&mut self) {
        if self.edge_finish.is_some() {
            self.close_edge_finish();
            return;
        }
        self.leave();
        self.flange = None;
        self.flange_edges.clear();
        self.picking = None;
        self.under = None;
        self.plane_draft = None;
        self.edge_hover = None;
        self.edge_finish = Some(EdgeFinishForm { open: None, size: None, chamfer: false, field: None, edit: None, sent: None });
    }

    pub(in crate::app) fn close_edge_finish(&mut self) {
        if self.edge_finish.take().is_some() {
            self.edge_hover = None;
            self.send(Request::Preview(None));
        }
    }

    pub(in crate::app) fn edge_finish_is_picking(&self) -> bool {
        self.edge_finish.is_some()
    }

    /// The hover and, on a click, the click itself: the lines to light, or none.
    pub(in crate::app) fn edge_finish_picker(&mut self, rect: egui::Rect, response: &egui::Response) -> Option<Vec<[Vec3; 2]>> {
        let cursor = response.hover_pos().or(response.interact_pointer_pos());
        let hover = cursor.and_then(|c| self.edge_finish_at(rect, c));
        let lines = hover.as_ref().map(EdgeHover::lines);
        // The cursor left the view: the stale hover would keep the preview of a target that is
        // no longer under the pointer (a size typed over a side panel would still show it).
        if hover.is_none() {
            self.edge_hover = None;
            return lines;
        }
        self.edge_hover = hover;
        if response.clicked() {
            self.edge_finish_click();
        }
        lines
    }

    /// What the click on the hovered edge or cap does: extend the open set, remove a member,
    /// take the last member's feature away, or make a new set. Nothing without a size typed.
    fn edge_finish_click(&mut self) {
        let Some(hover) = self.edge_hover.clone() else { return };
        // The edited set: a click toggles the target in it, the preview follows. A target of
        // another body or kind does not belong to the set: the window edits one set only.
        if let Some(edit) = self.edge_finish.as_mut().and_then(|f| f.edit.as_mut()) {
            // The body the clicks work in: the feature that began it, not the last feature on
            // it — after one set is applied the body belongs to that set's feature, and an
            // input named so would not resolve (D296).
            let body = self.model.as_ref().and_then(|m| m.begun.get(hover.body())).cloned();
            let same = body.as_deref() == Some(edit.set.body.as_str()) && edit.set.kind == hover.kind();
            if !same {
                return;
            }
            if let Some(k) = edit.set.targets.iter().position(|&t| t == hover.id()) {
                edit.set.targets.remove(k);
            } else {
                edit.set.targets.push(hover.id());
            }
            return;
        }
        let Some((size, _)) = self.edge_finish.as_ref().and_then(|f| f.size.clone()) else { return };
        let Some(f) = self.edge_finish.as_ref() else { return };
        let mode = if f.chamfer { EdgeMode::Chamfer } else { EdgeMode::Fillet };
        let Some(m) = self.model.as_ref() else { return };
        let Some(body) = m.begun.get(hover.body()).cloned() else { return };
        let open = f.open.as_ref().filter(|o| o.continues(&body, hover.kind(), &size, mode));
        if let Some(open) = open {
            let mut targets = open.targets.clone();
            if let Some(k) = targets.iter().position(|&t| t == hover.id()) {
                targets.remove(k);
            } else {
                targets.push(hover.id());
            }
            if targets.is_empty() {
                // The set empties: the feature made for it goes away with it.
                let feature = open.feature.clone();
                if let Some(form) = self.edge_finish.as_mut() {
                    form.open = None;
                    form.sent = None;
                }
                self.send(Request::Command("remove", json!({"name": feature})));
                return;
            }
            let params = open.edit_params(&targets);
            if self.ready(&params) {
                if let Some(form) = self.edge_finish.as_mut() {
                    form.sent = None;
                    if let Some(open) = &mut form.open {
                        open.targets = targets;
                    }
                }
                self.send(Request::Command("edit", params));
            }
        } else {
            let set = Set { feature: self.feature_name("edge_finish"), body, kind: hover.kind(), targets: vec![hover.id()], size, mode };
            let params = set.add_params();
            if self.ready(&params) {
                if let Some(form) = self.edge_finish.as_mut() {
                    form.open = Some(set);
                    form.sent = None;
                }
                self.send(Request::Command("add", params));
            }
        }
    }

    /// The preview of the hovered click matches the would-be command and stands.
    fn ready(&self, params: &Json) -> bool {
        self.model.as_ref().and_then(|m| m.preview.as_ref()).is_some_and(|(p, r)| p == params && r.is_ok())
    }

    /// What a click would send now: the edit of the set being edited, or the extension,
    /// removal or making of the open one. None over nothing to change.
    fn hover_params(&self) -> Option<(&'static str, Json)> {
        let f = self.edge_finish.as_ref()?;
        if let Some(edit) = &f.edit {
            return Some(("edit", edit.set.edit_all_params()));
        }
        let hover = self.edge_hover.as_ref()?;
        let (size, _) = f.size.as_ref()?;
        let mode = if f.chamfer { EdgeMode::Chamfer } else { EdgeMode::Fillet };
        let m = self.model.as_ref()?;
        // The body the set works in: the feature that began it, not the last feature on it
        // (an input named as the last feature would not resolve) — see `edge_finish_click`.
        let body = m.begun.get(hover.body())?.clone();
        let open = f.open.as_ref().filter(|o| o.continues(&body, hover.kind(), size, mode));
        if let Some(open) = open {
            let mut targets = open.targets.clone();
            if let Some(k) = targets.iter().position(|&t| t == hover.id()) {
                targets.remove(k);
                if targets.is_empty() {
                    // A removal that empties the set is sent as a removal, not previewed.
                    return None;
                }
            } else {
                targets.push(hover.id());
            }
            return Some(("edit", open.edit_params(&targets)));
        }
        let set = Set { feature: self.feature_name("edge_finish"), body, kind: hover.kind(), targets: vec![hover.id()], size: size.clone(), mode };
        Some(("add", set.add_params()))
    }

    /// The preview of what a click would do, sent when it changes; the edited set's size
    /// follows its text.
    pub(in crate::app) fn edge_finish_preview(&mut self) {
        if self.edge_finish.is_none() {
            return;
        }
        let params = self.hover_params();
        let sent = self.edge_finish.as_ref().and_then(|f| f.sent.clone());
        if params != sent {
            if let Some(form) = self.edge_finish.as_mut() {
                form.sent = params.clone();
            }
            self.send(Request::Preview(params));
        }
    }

    /// Nearest visible candidate within mouse reach, in screen points. Check depth at the
    /// nearest point on the edge so a back edge crossing a visible face cannot take the click.
    /// Else the flat cap under the cursor, by its corners.
    fn edge_finish_at(&self, rect: egui::Rect, cursor: egui::Pos2) -> Option<EdgeHover> {
        let m = self.model.as_ref()?;
        let half = rect.size() * 0.5;
        if half.x <= 0.0 || half.y <= 0.0 {
            return None;
        }
        let (anchor, radius) = sphere(&m.scene);
        let aspect = f64::from(half.x / half.y);
        let (vp, _) = self.camera.matrices(anchor, aspect, radius);
        // The bodies without a standing preview: the previewed one has the hovered corner
        // rounded, and picking it would drop the hover and flicker.
        let base = self.edge_base.as_ref().unwrap_or(&m.scene.bodies);
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
            let (origin, direction) = self.camera.ray(f64::from(at.x / half.x), f64::from(-at.y / half.y), aspect, anchor, radius);
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
        let (origin, direction) = self.camera.ray(f64::from(at.x / half.x), f64::from(-at.y / half.y), aspect, anchor, radius);
        let hit = Scene::pick_in(base, origin, direction)?;
        let mesh = base.get(hit.body)?;
        let corners = mesh.finish_edges_of.get(&hit.face)?;
        Some(EdgeHover::Face { body: hit.body, id: hit.face, edges: corners.iter().map(|&i| mesh.finish_edges[i].ends).collect() })
    }

    /// The tool's frame: the edited set's window, else the field by the cursor and the keys.
    pub(in crate::app) fn edge_finish_ui(&mut self, ctx: &egui::Context) {
        if self.edge_finish.is_none() {
            return;
        }
        if self.edge_finish.as_ref().is_some_and(|f| f.edit.is_some()) {
            self.edge_finish_edit_window(ctx);
            self.edge_finish_preview();
            return;
        }
        let typing = ctx.text_edit_focused();
        let mut close = false;
        // The size field open by the cursor: Enter stands the size, Esc drops it. A click
        // elsewhere leaves it standing (D286).
        if let Some(text) = self.edge_finish.as_mut().and_then(|f| f.field.as_mut()) {
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
                let text = self.edge_finish.as_mut().and_then(|f| f.field.take()).unwrap_or_default();
                if enter && !text.trim().is_empty() {
                    let value = value_of(&text);
                    let size = self.resolved(&value);
                    if let Some(form) = self.edge_finish.as_mut() {
                        form.size = Some((value, size));
                    }
                }
            }
        }
        // The keys, only with no field open and no text edit of another window focused.
        if self.edge_finish.as_ref().is_some_and(|f| f.field.is_none()) && !typing {
            let (escape, tab, up, down) = ctx.input_mut(|i| {
                (
                    i.key_pressed(egui::Key::Escape),
                    i.consume_key(egui::Modifiers::NONE, egui::Key::Tab),
                    i.consume_key(egui::Modifiers::NONE, egui::Key::ArrowUp),
                    i.consume_key(egui::Modifiers::NONE, egui::Key::ArrowDown),
                )
            });
            if escape {
                close = true;
            }
            if tab && let Some(form) = self.edge_finish.as_mut() {
                form.chamfer = !form.chamfer;
            }
            if up || down {
                let current = self.edge_finish.as_ref().and_then(|f| f.size.as_ref()).map_or(f64::NAN, |(_, v)| *v);
                let chamfer = self.edge_finish.as_ref().is_some_and(|f| f.chamfer);
                let chamfers: Vec<f64> = self.chamfer_presets.iter().map(|&(a, _)| a).collect();
                let presets = if chamfer { &chamfers } else { &self.fillet_presets };
                if let Some(size) = crate::draw::preset_step(presets, current, up)
                    && let Some(form) = self.edge_finish.as_mut()
                {
                    form.size = Some((json!(size), size));
                }
            }
            let first = ctx.input(|i| i.events.iter().find_map(|e| if let egui::Event::Text(t) = e { Some(t.clone()) } else { None }));
            if let Some(t) = first.filter(|t| t.starts_with(|c: char| c.is_ascii_digit() || c == '.' || c == '-'))
                && let Some(form) = self.edge_finish.as_mut()
            {
                form.field = Some(t);
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
                    self.model.as_ref().and_then(|m| m.inspect["parameters"][name]["value"].as_f64()).ok_or_else(|| ExprError::Unknown(name.into()))
                })
                .ok()
            })
            .unwrap_or(f64::NAN)
    }

    /// The window that edits a finished set from the tree: its corners or caps, the size and
    /// the mode, and clicks on the part to take targets in or out.
    fn edge_finish_edit_window(&mut self, ctx: &egui::Context) {
        // The size follows its text at once, before `before` is taken: a Enter of the same frame
        // must not stand a size the field has already replaced (D296).
        if let Some(edit) = self.edge_finish.as_mut().and_then(|f| f.edit.as_mut()) {
            edit.set.size = value_of(&edit.text);
        }
        let before = self.edge_finish.as_ref().and_then(|f| f.edit.as_ref()).map(|e| e.set.edit_all_params());
        let preview = before.as_ref().and_then(|p| self.model.as_ref()?.preview.as_ref().filter(|(q, _)| q == p).map(|(_, r)| r));
        let ready = preview.is_some_and(Result::is_ok);
        let enter = ctx.input_mut(|i| i.consume_key(egui::Modifiers::NONE, egui::Key::Enter));
        let mut accept = false;
        let mut cancel = false;
        if let Some(f) = &mut self.edge_finish {
            let Some(edit) = &mut f.edit else { return };
            let tab = ctx.input_mut(|i| i.consume_key(egui::Modifiers::NONE, egui::Key::Tab));
            if tab {
                edit.set.mode = match edit.set.mode {
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
                    self.model.as_ref().and_then(|m| m.inspect["parameters"][name]["value"].as_f64()).ok_or_else(|| ExprError::Unknown(name.into()))
                };
                let current = serde_json::from_value::<Num>(value_of(&edit.text)).ok().and_then(|n| n.resolve(&lookup).ok()).unwrap_or(f64::NAN);
                let chamfers: Vec<f64> = self.chamfer_presets.iter().map(|&(a, _)| a).collect();
                let presets = if edit.set.mode == EdgeMode::Chamfer { &chamfers } else { &self.fillet_presets };
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
                    ui.label(&edit.set.body);
                    match edit.set.kind {
                        Kind::Edges => {
                            ui.label(self.i18n.f("edge-finish-corners", "Corners: { $n }", &[("n", &edit.set.targets.len().to_string())]));
                        }
                        Kind::Faces => {
                            ui.label(self.i18n.f("edge-finish-caps", "Caps: { $n }", &[("n", &edit.set.targets.len().to_string())]));
                        }
                    }
                    ui.label(self.i18n.t("edge-finish-toggle", "Click the part to take a target in or out"));
                    ui.horizontal(|ui| {
                        ui.selectable_value(&mut edit.set.mode, EdgeMode::Fillet, self.i18n.t("edge-finish-fillet", "Fillet"));
                        ui.selectable_value(&mut edit.set.mode, EdgeMode::Chamfer, self.i18n.t("edge-finish-chamfer", "Chamfer"));
                    });
                    ui.horizontal(|ui| {
                        let label = match edit.set.mode {
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
        let after = self.edge_finish.as_ref().and_then(|f| f.edit.as_ref()).map(|e| e.set.edit_all_params());
        if cancel {
            self.close_edge_finish();
        } else if (accept || enter)
            && ready
            && after == before
            && let Some(after) = after
        {
            self.close_edge_finish();
            self.send(Request::Command("edit", after));
        }
    }

    /// The hint by the cursor, the presets beside it, and a preview that refused, under them.
    pub(in crate::app) fn edge_finish_hints(&self, ui: &egui::Ui, painter: &egui::Painter, cursor: egui::Pos2) {
        let Some(f) = self.edge_finish.as_ref() else { return };
        if f.edit.is_some() || f.field.is_some() {
            return;
        }
        crate::draw::size_hint(&self.i18n, painter, cursor, f.size.as_ref().map(|(_, v)| *v), f.chamfer, &self.chamfer_presets, &self.fillet_presets);
        if let Some(why) = self.model.as_ref().and_then(|m| m.preview.as_ref()).and_then(|(_, r)| r.as_ref().err()) {
            painter.text(cursor + egui::vec2(12.0, 48.0), egui::Align2::LEFT_TOP, why, egui::FontId::proportional(13.0), ui.visuals().error_fg_color);
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use std::sync::Arc;

    use datum_math::Vec2;

    use super::*;
    use crate::app::tests::viewer;
    use crate::scene::BodyMesh;

    fn hovering(v: &mut Viewer, id: TopoId, at: [f64; 3]) {
        v.edge_hover = Some(EdgeHover::Edge { body: 0, id, ends: [Vec3::new(at[0], at[1], at[2]), Vec3::new(at[0], at[1], at[2] + 10.0)] });
    }

    #[test]
    fn the_edge_tool_builds_sets_by_clicks() {
        let (mut v, inbox) = viewer("parametric.json");
        let corners = [Vec2::new(0.0, 0.0), Vec2::new(10.0, 0.0), Vec2::new(10.0, 10.0), Vec2::new(0.0, 10.0)];
        let body = datum_brep::prism(TopoId::derive("test-box").finish(), &corners, 0.0, 10.0).unwrap();
        let mesh = BodyMesh::new(&body, false);
        let edge = mesh.finish_edges.iter().find(|e| e.ends.iter().all(|p| p.x == 10.0 && p.y == 0.0)).unwrap().id;
        let other = mesh.finish_edges.iter().find(|e| e.ends.iter().all(|p| p.x == 10.0 && p.y == 10.0)).unwrap().id;
        let m = v.model.as_mut().unwrap();
        m.scene.bodies = vec![Arc::new(mesh)];
        m.scene.bounds = Some((Vec3::ZERO, Vec3::new(10.0, 10.0, 10.0)));
        m.bodies = vec!["box".into()];
        m.begun = vec!["box".into()];
        v.open_edge_finish();
        assert!(inbox.try_recv().is_err(), "nothing is sent while the tool opens");
        v.edge_finish.as_mut().unwrap().size = Some((json!(2.0), 2.0));

        // The hovered corner previews the `add` of a one-edge set; the click makes it.
        hovering(&mut v, edge, [10.0, 0.0, 0.0]);
        v.edge_finish_preview();
        let add =
            json!({"name": "edge_finish1", "op": "edge_finish", "inputs": ["box"], "edges": [format!("{edge:?}")], "size": 2.0, "mode": "fillet"});
        assert!(matches!(inbox.try_recv().unwrap(), Request::Preview(Some(("add", p))) if p == add));
        v.model.as_mut().unwrap().preview = Some((add.clone(), Ok(json!({}))));
        v.edge_finish_click();
        assert!(matches!(inbox.try_recv().unwrap(), Request::Command("add", p) if p == add));

        // The next click of the same size and mode extends the set.
        hovering(&mut v, other, [10.0, 10.0, 0.0]);
        v.edge_finish_preview();
        let extend = json!({"name": "edge_finish1", "edge": "", "edges": [format!("{edge:?}"), format!("{other:?}")], "faces": []});
        assert!(matches!(inbox.try_recv().unwrap(), Request::Preview(Some(("edit", p))) if p == extend));
        v.model.as_mut().unwrap().preview = Some((extend.clone(), Ok(json!({}))));
        v.edge_finish_click();
        assert!(matches!(inbox.try_recv().unwrap(), Request::Command("edit", p) if p == extend));

        // A click on a member removes it; the removal that empties the set is not previewed,
        // and takes the feature away.
        hovering(&mut v, edge, [10.0, 0.0, 0.0]);
        v.edge_finish_preview();
        let shorten = json!({"name": "edge_finish1", "edge": "", "edges": [format!("{other:?}")], "faces": []});
        assert!(matches!(inbox.try_recv().unwrap(), Request::Preview(Some(("edit", p))) if p == shorten));
        v.model.as_mut().unwrap().preview = Some((shorten.clone(), Ok(json!({}))));
        v.edge_finish_click();
        assert!(matches!(inbox.try_recv().unwrap(), Request::Command("edit", p) if p == shorten));
        hovering(&mut v, other, [10.0, 10.0, 0.0]);
        v.edge_finish_preview();
        assert!(inbox.try_recv().is_err(), "an emptying removal is not previewed");
        v.edge_finish_click();
        assert!(matches!(inbox.try_recv().unwrap(), Request::Command("remove", p) if p == json!({"name": "edge_finish1"})));

        // A new size starts a new set; the finished ones stand.
        v.model.as_mut().unwrap().inspect["features"].as_array_mut().unwrap().push(json!({"name": "edge_finish1", "op": "edge_finish"}));
        v.edge_finish.as_mut().unwrap().size = Some((json!(4.0), 4.0));
        v.edge_finish_preview();
        let next =
            json!({"name": "edge_finish2", "op": "edge_finish", "inputs": ["box"], "edges": [format!("{other:?}")], "size": 4.0, "mode": "fillet"});
        assert!(matches!(inbox.try_recv().unwrap(), Request::Preview(Some(("add", p))) if p == next));
        v.model.as_mut().unwrap().preview = Some((next.clone(), Ok(json!({}))));
        v.edge_finish_click();
        assert!(matches!(inbox.try_recv().unwrap(), Request::Command("add", p) if p == next));

        // A click whose preview does not stand does nothing; an emptying removal is its own
        // command, not previewed.
        v.model.as_mut().unwrap().preview = None;
        v.edge_finish.as_mut().unwrap().size = Some((json!(6.0), 6.0));
        hovering(&mut v, other, [10.0, 10.0, 0.0]);
        v.edge_finish_click();
        assert!(inbox.try_recv().is_err(), "no command without a standing preview");
    }

    /// The body the sets work in is the feature the part began with: after a set is applied,
    /// the scene body belongs to that set's feature, and an input named so would not resolve —
    /// the clicks would refuse with `unknown input` at every hover.
    #[test]
    fn the_sets_work_in_the_body_the_part_began_with() {
        let (mut v, inbox) = viewer("parametric.json");
        let corners = [Vec2::new(0.0, 0.0), Vec2::new(10.0, 0.0), Vec2::new(10.0, 10.0), Vec2::new(0.0, 10.0)];
        let body = datum_brep::prism(TopoId::derive("test-box").finish(), &corners, 0.0, 10.0).unwrap();
        let mesh = BodyMesh::new(&body, false);
        let edge = mesh.finish_edges.iter().find(|e| e.ends.iter().all(|p| p.x == 10.0 && p.y == 0.0)).unwrap().id;
        let other = mesh.finish_edges.iter().find(|e| e.ends.iter().all(|p| p.x == 10.0 && p.y == 10.0)).unwrap().id;
        let m = v.model.as_mut().unwrap();
        m.scene.bodies = vec![Arc::new(mesh)];
        m.scene.bounds = Some((Vec3::ZERO, Vec3::new(10.0, 10.0, 10.0)));
        // One set applied: the body now belongs to its feature.
        m.bodies = vec!["edge_finish1".into()];
        m.begun = vec!["box".into()];
        m.inspect["features"] = json!([{"name": "edge_finish1", "op": "edge_finish"}]);
        v.open_edge_finish();
        v.edge_finish.as_mut().unwrap().size = Some((json!(2.0), 2.0));

        hovering(&mut v, edge, [10.0, 0.0, 0.0]);
        v.edge_finish_preview();
        let add =
            json!({"name": "edge_finish2", "op": "edge_finish", "inputs": ["box"], "edges": [format!("{edge:?}")], "size": 2.0, "mode": "fillet"});
        assert!(
            matches!(inbox.try_recv().unwrap(), Request::Preview(Some(("add", p))) if p == add),
            "the input is the part's first feature, not the last one's"
        );
        v.model.as_mut().unwrap().preview = Some((add.clone(), Ok(json!({}))));
        v.edge_finish_click();
        assert!(matches!(inbox.try_recv().unwrap(), Request::Command("add", p) if p == add));

        // The second click of the same size and mode extends the set.
        hovering(&mut v, other, [10.0, 10.0, 0.0]);
        v.edge_finish_preview();
        let extend = json!({"name": "edge_finish2", "edge": "", "edges": [format!("{edge:?}"), format!("{other:?}")], "faces": []});
        assert!(matches!(inbox.try_recv().unwrap(), Request::Preview(Some(("edit", p))) if p == extend), "the open set is extended, not begun again");
    }

    /// The Edge tool picks the model without a standing preview: the previewed body has the
    /// hovered corner rounded already, so picking it would drop the hover and flicker.
    #[test]
    fn the_edge_tool_picks_the_model_without_a_preview() {
        let (mut v, _inbox) = viewer("parametric.json");
        let corners = [Vec2::new(0.0, 0.0), Vec2::new(10.0, 0.0), Vec2::new(10.0, 10.0), Vec2::new(0.0, 10.0)];
        let body = datum_brep::prism(TopoId::derive("test-box").finish(), &corners, 0.0, 10.0).unwrap();
        let base = BodyMesh::new(&body, false);
        let edge = base.finish_edges.iter().find(|e| e.ends.iter().all(|p| p.x == 10.0 && p.y == 0.0)).unwrap().id;
        // The displayed scene is the previewed one: the corner is rounded, its edge is gone.
        let mut shown = BodyMesh::new(&body, false);
        shown.finish_edges.clear();
        shown.finish_edges_of.clear();
        let m = v.model.as_mut().unwrap();
        m.scene.bodies = vec![Arc::new(shown)];
        m.scene.bounds = Some((Vec3::ZERO, Vec3::new(10.0, 10.0, 10.0)));
        m.bodies = vec!["box".into()];
        m.begun = vec!["box".into()];
        v.viewport = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(600.0, 400.0));
        v.camera.fit(m.scene.bounds, 1.5);
        let cursor = {
            let (anchor, radius) = sphere(&m.scene);
            let aspect = 300.0 / 200.0;
            let (vp, _) = v.camera.matrices(anchor, aspect, radius);
            in_rect(crate::camera::clip(&vp, Vec3::new(10.0, 0.0, 5.0) - anchor), v.viewport)
        };
        assert!(v.edge_finish_at(v.viewport, cursor).is_none(), "the previewed body has no corner edge to pick");
        v.edge_base = Some(vec![Arc::new(base)]);
        assert_eq!(v.edge_finish_at(v.viewport, cursor).unwrap().id(), edge, "the base body still has it");
    }

    #[test]
    fn the_edge_window_edits_a_set_from_the_tree() {
        let (mut v, inbox) = viewer("parametric.json");
        let corners = [Vec2::new(0.0, 0.0), Vec2::new(10.0, 0.0), Vec2::new(10.0, 10.0), Vec2::new(0.0, 10.0)];
        let body = datum_brep::prism(TopoId::derive("test-box").finish(), &corners, 0.0, 10.0).unwrap();
        let mesh = BodyMesh::new(&body, false);
        let edge = mesh.finish_edges.iter().find(|e| e.ends.iter().all(|p| p.x == 10.0 && p.y == 0.0)).unwrap().id;
        let other = mesh.finish_edges.iter().find(|e| e.ends.iter().all(|p| p.x == 10.0 && p.y == 10.0)).unwrap().id;
        let m = v.model.as_mut().unwrap();
        m.scene.bodies = vec![Arc::new(mesh)];
        m.scene.bounds = Some((Vec3::ZERO, Vec3::new(10.0, 10.0, 10.0)));
        m.bodies = vec!["box".into()];
        m.begun = vec!["box".into()];
        let feature =
            json!({"name": "edge_finish1", "op": "edge_finish", "inputs": ["box"], "edges": [format!("{edge:?}")], "size": 4.0, "mode": "chamfer"});
        v.edge_finish = Some(EdgeFinishForm::editing("edge_finish1".into(), &feature));
        assert_eq!(v.edited_feature().as_deref(), Some("edge_finish1"));

        let mut h = egui_kittest::Harness::builder().with_size([600.0, 400.0]).build_ui_state(|ui, v: &mut Viewer| v.edge_finish_ui(ui.ctx()), v);
        h.run();
        let edit = json!({"name": "edge_finish1", "edge": "", "edges": [format!("{edge:?}")], "faces": [], "size": 4.0, "mode": "chamfer"});
        assert!(matches!(inbox.try_recv().unwrap(), Request::Preview(Some(("edit", p))) if p == edit));

        // Tab turns the set to a fillet of the same size.
        h.state_mut().model.as_mut().unwrap().preview = Some((edit.clone(), Ok(json!({}))));
        h.key_press(egui::Key::Tab);
        h.run();
        let fillet = json!({"name": "edge_finish1", "edge": "", "edges": [format!("{edge:?}")], "faces": [], "size": 4.0, "mode": "fillet"});
        assert!(matches!(inbox.try_recv().unwrap(), Request::Preview(Some(("edit", p))) if p == fillet));

        // A click on the part takes a corner in.
        h.state_mut().model.as_mut().unwrap().preview = Some((fillet.clone(), Ok(json!({}))));
        h.state_mut().edge_hover = Some(EdgeHover::Edge { body: 0, id: other, ends: [Vec3::new(10.0, 10.0, 0.0), Vec3::new(10.0, 10.0, 10.0)] });
        h.state_mut().edge_finish_click();
        h.run();
        let grown = json!({"name": "edge_finish1", "edge": "", "edges": [format!("{edge:?}"), format!("{other:?}")], "faces": [], "size": 4.0, "mode": "fillet"});
        assert!(matches!(inbox.try_recv().unwrap(), Request::Preview(Some(("edit", p))) if p == grown));

        // Enter applies what the preview stands for, and the window closes.
        h.state_mut().model.as_mut().unwrap().preview = Some((grown.clone(), Ok(json!({}))));
        h.key_press(egui::Key::Enter);
        h.run();
        assert!(h.state().edge_finish.is_none());
        let sent: Vec<_> = inbox.try_iter().collect();
        assert!(sent.iter().any(|r| matches!(r, Request::Command("edit", p) if *p == grown)), "{sent:?}");
    }
}
