//! Selection, preview and editing for the Edge tool (D295).

use datum_brep::TopoId;
use datum_math::Vec3;
use datum_math::expr::{ExprError, Num};
use datum_ops::EdgeMode;
use serde_json::{Value as Json, json};

use crate::actions::Act;
use crate::app::{Request, Viewer, definition, in_rect, sphere};
use crate::dims::value_of;
use crate::scene::FinishEdge;

pub(in crate::app) struct EdgeFinishForm {
    name: String,
    target: Option<(String, String)>,
    mode: EdgeMode,
    size: String,
    editing: bool,
    fresh: bool,
    sent: Option<Json>,
}

impl EdgeFinishForm {
    pub(in crate::app) fn editing(name: String, feature: &Json) -> Self {
        let target = feature["inputs"][0].as_str().zip(feature["edge"].as_str()).map(|(body, edge)| (body.to_string(), edge.to_string()));
        Self {
            name,
            target,
            mode: serde_json::from_value(feature["mode"].clone()).unwrap_or_default(),
            size: definition(&feature["size"]),
            editing: true,
            fresh: true,
            sent: None,
        }
    }

    pub(in crate::app) fn edited(&self) -> Option<&String> {
        self.editing.then_some(&self.name)
    }

    fn params(&self) -> Option<(&'static str, Json)> {
        let (body, edge) = self.target.as_ref()?;
        let size = value_of(&self.size);
        if self.size.trim().is_empty() || size.is_null() {
            return None;
        }
        if self.editing {
            Some(("edit", json!({"name": self.name, "size": size, "mode": self.mode})))
        } else {
            Some(("add", json!({"name": self.name, "op": "edge_finish", "inputs": [body], "edge": edge, "size": size, "mode": self.mode})))
        }
    }
}

impl Viewer {
    pub(in crate::app) fn open_edge_finish(&mut self, cursor: Option<egui::Pos2>) {
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
        self.edge_finish = Some(EdgeFinishForm {
            name: self.feature_name("edge_finish"),
            target: None,
            mode: EdgeMode::Fillet,
            size: "2".into(),
            editing: false,
            fresh: false,
            sent: None,
        });
        if let Some(cursor) = cursor.filter(|p| self.viewport.contains(*p))
            && let Some((body, edge)) = self.edge_finish_at(self.viewport, cursor)
        {
            self.select_finish_edge(body, edge.id);
        }
    }

    pub(in crate::app) fn close_edge_finish(&mut self) {
        if self.edge_finish.take().is_some_and(|f| f.sent.is_some()) {
            self.send(Request::Preview(None));
        }
    }

    pub(in crate::app) fn edge_finish_is_picking(&self) -> bool {
        self.edge_finish.as_ref().is_some_and(|f| !f.editing && f.target.is_none())
    }

    pub(in crate::app) fn edge_finish_picker(&mut self, rect: egui::Rect, response: &egui::Response) -> Option<[Vec3; 2]> {
        let cursor = response.hover_pos().or(response.interact_pointer_pos())?;
        let (body, edge) = self.edge_finish_at(rect, cursor)?;
        if response.clicked() {
            self.select_finish_edge(body, edge.id);
        }
        Some(edge.ends)
    }

    fn select_finish_edge(&mut self, body: usize, edge: TopoId) {
        if let Some(f) = &mut self.edge_finish
            && let Some(body) = self.model.as_ref().and_then(|m| m.bodies.get(body))
        {
            f.target = Some((body.clone(), format!("{edge:?}")));
            f.fresh = true;
        }
    }

    /// Nearest visible candidate within mouse reach, in screen points. Check depth at the
    /// nearest point on the edge so a back edge crossing a visible face cannot take the click.
    fn edge_finish_at(&self, rect: egui::Rect, cursor: egui::Pos2) -> Option<(usize, FinishEdge)> {
        let m = self.model.as_ref()?;
        let half = rect.size() * 0.5;
        if half.x <= 0.0 || half.y <= 0.0 {
            return None;
        }
        let (anchor, radius) = sphere(&m.scene);
        let aspect = f64::from(half.x / half.y);
        let (vp, _) = self.camera.matrices(anchor, aspect, radius);
        let near = |edge: &FinishEdge| {
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
            if m.scene.pick(origin, direction).is_some_and(|hit| hit.distance + rounding < depth) {
                return None;
            }
            Some((distance, depth))
        };
        m.scene
            .bodies
            .iter()
            .enumerate()
            .flat_map(|(body, mesh)| mesh.finish_edges.iter().filter_map(move |&edge| Some((body, edge, near(&edge)?))))
            .min_by(|a, b| a.2.0.total_cmp(&b.2.0).then_with(|| a.2.1.total_cmp(&b.2.1)))
            .map(|(body, edge, _)| (body, edge))
    }

    pub(in crate::app) fn edge_finish_window(&mut self, ctx: &egui::Context) {
        let Some(f) = &mut self.edge_finish else { return };
        let (enter, escape, tab, up, down) = ctx.input_mut(|i| {
            (
                i.consume_key(egui::Modifiers::NONE, egui::Key::Enter),
                i.key_pressed(egui::Key::Escape),
                i.consume_key(egui::Modifiers::NONE, egui::Key::Tab),
                i.consume_key(egui::Modifiers::NONE, egui::Key::ArrowUp),
                i.consume_key(egui::Modifiers::NONE, egui::Key::ArrowDown),
            )
        });
        if escape {
            self.close_edge_finish();
            return;
        }
        if tab {
            f.mode = match f.mode {
                EdgeMode::Fillet => EdgeMode::Chamfer,
                EdgeMode::Chamfer => EdgeMode::Fillet,
            };
            f.fresh = f.target.is_some();
        }
        if up || down {
            let lookup = |name: &str| {
                self.model.as_ref().and_then(|m| m.inspect["parameters"][name]["value"].as_f64()).ok_or_else(|| ExprError::Unknown(name.into()))
            };
            let current = serde_json::from_value::<Num>(value_of(&f.size)).ok().and_then(|n| n.resolve(&lookup).ok()).unwrap_or(f64::NAN);
            let chamfers: Vec<f64> = self.chamfer_presets.iter().map(|&(a, _)| a).collect();
            let presets = if f.mode == EdgeMode::Chamfer { &chamfers } else { &self.fillet_presets };
            if let Some(size) = crate::draw::preset_step(presets, current, up) {
                f.size = definition(&json!(size));
            }
        }
        let before = f.params();
        let preview = before.as_ref().and_then(|(_, params)| self.model.as_ref()?.preview.as_ref().filter(|(p, _)| p == params).map(|(_, r)| r));
        let ready = preview.is_some_and(Result::is_ok);
        let (mut accept, mut cancel) = (false, false);
        egui::Window::new(self.i18n.label(Act::EdgeFinish)).collapsible(false).resizable(false).anchor(egui::Align2::CENTER_TOP, [0.0, 60.0]).show(
            ctx,
            |ui| {
                match &f.target {
                    Some((body, _)) => {
                        ui.label(body);
                    }
                    None => {
                        ui.label(self.i18n.t("edge-finish-pick", "Click a straight edge on the part"));
                    }
                }
                ui.horizontal(|ui| {
                    ui.selectable_value(&mut f.mode, EdgeMode::Fillet, self.i18n.t("edge-finish-fillet", "Fillet"));
                    ui.selectable_value(&mut f.mode, EdgeMode::Chamfer, self.i18n.t("edge-finish-chamfer", "Chamfer"));
                });
                ui.horizontal(|ui| {
                    let label = match f.mode {
                        EdgeMode::Fillet => self.i18n.t("edge-finish-radius", "Radius, mm"),
                        EdgeMode::Chamfer => self.i18n.t("edge-finish-distance", "Distance, mm"),
                    };
                    ui.label(label);
                    let mut edit = egui::TextEdit::singleline(&mut f.size).desired_width(90.0).show(ui);
                    if std::mem::take(&mut f.fresh) {
                        edit.response.request_focus();
                        let all = egui::text::CCursorRange::two(egui::text::CCursor::new(0), egui::text::CCursor::new(f.size.chars().count()));
                        edit.state.cursor.set_char_range(Some(all));
                        edit.state.store(ctx, edit.response.id);
                    }
                });
                if let Some(Err(why)) = preview {
                    ui.colored_label(ui.visuals().error_fg_color, why);
                }
                ui.horizontal(|ui| {
                    accept = ui.add_enabled(ready, egui::Button::new(self.i18n.t("ok", "OK"))).clicked();
                    cancel = ui.button(self.i18n.t("cancel", "Cancel")).clicked();
                });
            },
        );
        let after = f.params();
        if cancel {
            self.close_edge_finish();
        } else if let Some((command, params)) = after {
            if (accept || enter) && ready && before.as_ref().is_some_and(|(_, p)| *p == params) {
                self.close_edge_finish();
                self.send(Request::Command(command, params));
            } else if f.sent.as_ref() != Some(&params) {
                f.sent = Some(params.clone());
                self.send(Request::Preview(Some((command, params))));
            }
        } else if f.sent.take().is_some() {
            self.send(Request::Preview(None));
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

    #[test]
    fn the_edge_window_selects_previews_makes_and_edits() {
        let (mut v, inbox) = viewer("parametric.json");
        let corners = [Vec2::new(0.0, 0.0), Vec2::new(10.0, 0.0), Vec2::new(10.0, 10.0), Vec2::new(0.0, 10.0)];
        let body = datum_brep::prism(TopoId::derive("test-box").finish(), &corners, 0.0, 10.0).unwrap();
        let m = v.model.as_mut().unwrap();
        let mesh = Arc::new(BodyMesh::new(&body, false));
        let edge = mesh.finish_edges.iter().find(|e| e.ends.iter().all(|p| p.x == 10.0 && p.y == 0.0)).unwrap().id;
        m.scene.bodies = vec![mesh];
        m.scene.bounds = Some((Vec3::ZERO, Vec3::new(10.0, 10.0, 10.0)));
        m.bodies = vec!["box".into()];
        v.viewport = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(600.0, 400.0));
        v.camera.fit(m.scene.bounds, 1.5);
        let (anchor, radius) = sphere(&m.scene);
        let (vp, _) = v.camera.matrices(anchor, 1.5, radius);
        let near = |p| in_rect(crate::camera::clip(&vp, p - anchor), v.viewport);
        let cursor = near(Vec3::new(10.0, 0.0, 5.0));
        assert_eq!(v.edge_finish_at(v.viewport, cursor).unwrap().1.id, edge);
        assert!(v.edge_finish_at(v.viewport, near(Vec3::new(0.0, 10.0, 3.0))).is_none(), "occluded back edge cannot be picked");
        v.open_edge_finish(Some(cursor));
        let mut h = egui_kittest::Harness::builder().with_size([600.0, 400.0]).build_ui_state(|ui, v: &mut Viewer| v.edge_finish_window(ui.ctx()), v);
        h.run();
        let made = |mode, size| json!({"name": "edge_finish1", "op": "edge_finish", "inputs": ["box"], "edge": format!("{edge:?}"), "size": size, "mode": mode});
        let default = made("fillet", 2.0);
        assert!(matches!(inbox.try_recv().unwrap(), Request::Preview(Some(("add", p))) if p == default));

        h.state_mut().model.as_mut().unwrap().preview = Some((default, Ok(json!({}))));
        h.key_press(egui::Key::Tab);
        h.run();
        assert!(matches!(inbox.try_recv().unwrap(), Request::Preview(Some(("add", p))) if p == made("chamfer", 2.0)));
        h.key_press(egui::Key::ArrowUp);
        h.run();
        let chamfer = made("chamfer", 4.0);
        assert!(matches!(inbox.try_recv().unwrap(), Request::Preview(Some(("add", p))) if p == chamfer));
        h.key_press(egui::Key::Enter);
        h.run();
        assert!(inbox.try_recv().is_err(), "Enter waits for the current preview");
        h.state_mut().model.as_mut().unwrap().preview = Some((chamfer.clone(), Ok(json!({}))));
        h.key_press(egui::Key::Enter);
        h.run();
        assert!(inbox.try_iter().any(|r| matches!(r, Request::Command("add", p) if p == chamfer)));
        assert!(h.state().edge_finish.is_none());

        h.state_mut().model.as_mut().unwrap().inspect["features"].as_array_mut().unwrap().push(chamfer);
        h.state_mut().reedit("edge_finish1");
        h.run();
        assert_eq!(h.state().edited_feature().as_deref(), Some("edge_finish1"));
        assert!(
            matches!(inbox.try_recv().unwrap(), Request::Preview(Some(("edit", p))) if p == json!({"name": "edge_finish1", "size": 4.0, "mode": "chamfer"}))
        );
        h.key_press(egui::Key::Tab);
        h.run();
        let edit = json!({"name": "edge_finish1", "size": 4.0, "mode": "fillet"});
        assert!(matches!(inbox.try_recv().unwrap(), Request::Preview(Some(("edit", p))) if p == edit));
        h.state_mut().model.as_mut().unwrap().preview = Some((edit.clone(), Ok(json!({}))));
        h.key_press(egui::Key::Enter);
        h.run();
        let sent: Vec<_> = inbox.try_iter().collect();
        assert!(sent.iter().any(|r| matches!(r, Request::Command("edit", p) if *p == edit)), "{sent:?}");
    }
}
