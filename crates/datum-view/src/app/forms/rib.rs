//! The form of a rib (D250): the sheet, the sketch of its path, the tool (never chosen for the
//! engineer) and the side it rises on, the preview sent last.

use serde_json::{Value as Json, json};

use crate::actions::Act;
use crate::app::{Request, Viewer, flip_tile, on_sheet};

/// The form of a rib (D250): the sheet, the sketch of its path, the tool (never chosen for the
/// engineer) and the side it rises on, the preview sent last.
pub(in crate::app) struct RibForm {
    name: String,
    body: String,
    sketch: String,
    tool: Option<String>,
    flip: bool,
    /// Changing feature `name`, opened from the tree, not adding it.
    editing: bool,
    sent: Option<Json>,
}

impl RibForm {
    /// The form of `name`, a feature the tree opens for changing: it edits the tool and the side.
    pub(in crate::app) fn editing(name: String, body: String, sketch: String, tool: Option<String>, flip: bool) -> Self {
        Self { name, body, sketch, tool, flip, editing: true, sent: None }
    }

    /// The feature this form changes, if the tree opened it.
    pub(in crate::app) fn edited(&self) -> Option<&String> {
        self.editing.then_some(&self.name)
    }

    /// The command and its fields as the model takes them, none until a tool is chosen.
    fn params(&self) -> Option<(&'static str, Json)> {
        let tool = self.tool.as_ref()?;
        if self.editing {
            return Some(("edit", json!({"name": self.name, "tool": tool, "flip": self.flip})));
        }
        let mut p = json!({"name": self.name, "op": "rib", "inputs": [self.body, self.sketch], "tool": tool});
        if self.flip {
            p["flip"] = json!(true);
        }
        Some(("add", p))
    }
}

impl Viewer {
    /// The sketches a rib can run along: on a face of sheet metal, with an open chain. Each with
    /// the body at the end of the sheet it lies on, the newest sketch first.
    fn rib_sketches(&self) -> Vec<(String, String)> {
        let Some(m) = &self.model else { return Vec::new() };
        let features = m.inspect["features"].as_array().map_or(&[][..], Vec::as_slice);
        let by_name = |n: &str| features.iter().find(|f| f["name"].as_str() == Some(n));
        // Whether body `end`'s chain of first inputs comes to `start`.
        let reaches = |end: &str, start: &str| {
            let mut at = Some(end.to_string());
            for _ in features {
                let Some(name) = at else { return false };
                if name == start {
                    return true;
                }
                at = by_name(&name).and_then(|f| f["inputs"][0].as_str().map(String::from));
            }
            false
        };
        let open = |f: &Json| f["sketch"]["lines"].as_object().is_some_and(|l| l.values().any(|c| c["construction"] != true));
        let arcs = |f: &Json| f["sketch"]["arcs"].as_object().is_some_and(|l| l.values().any(|c| c["construction"] != true));
        features
            .iter()
            .rev()
            .filter(|f| f["op"] == "sketch" && f["face"].is_string() && (open(f) || arcs(f)))
            .filter_map(|f| {
                let name = f["name"].as_str()?;
                let start = f["inputs"][0].as_str()?;
                on_sheet(features, name).then_some(())?;
                let body = m.bodies.iter().find(|b| reaches(b, start))?;
                Some((name.to_string(), body.clone()))
            })
            .collect()
    }

    /// Opens the form of a rib on the newest sketch that can carry one, or says what is missing.
    pub(in crate::app) fn open_rib(&mut self, at: egui::Pos2) {
        let Some((sketch, body)) = self.rib_sketches().into_iter().next() else {
            self.contour_hint = Some((at, std::time::Instant::now(), Act::Rib));
            return;
        };
        self.close_revolve();
        self.flange = None;
        self.flange_edges.clear();
        self.picking = None;
        self.under = None;
        self.rib = Some(RibForm { name: self.feature_name("rib"), body, sketch, tool: None, flip: false, editing: false, sent: None });
    }

    /// Drops the rib form, and the preview it made.
    pub(in crate::app) fn close_rib(&mut self) {
        if let Some(r) = self.rib.take()
            && r.sent.is_some()
        {
            self.send(Request::Preview(None));
        }
    }

    /// The rib form: the sketch, the tool and the side; previewed at every change once a tool is
    /// chosen, Enter or OK makes it once the preview is not refused, Esc or Cancel drops it.
    pub(in crate::app) fn rib_window(&mut self, ctx: &egui::Context) {
        let sketches = self.rib_sketches();
        let tools: Vec<String> = self.tools.iter().filter_map(|t| t["name"].as_str().map(String::from)).collect();
        let Some(f) = &mut self.rib else { return };
        let params = f.params();
        let preview =
            self.model.as_ref().and_then(|m| m.preview.as_ref()).filter(|(p, _)| Some(p) == params.as_ref().map(|(_, j)| j)).map(|(_, r)| r);
        let ok = preview.is_some_and(Result::is_ok);
        let (mut add, mut cancel) = (false, false);
        egui::Window::new(self.i18n.label(Act::Rib))
            .collapsible(false)
            .resizable(false)
            .anchor(egui::Align2::CENTER_TOP, egui::vec2(0.0, 60.0))
            .show(ctx, |ui| {
                ui.label(format!("{}: {}", self.i18n.t("trim-sheet", "Sheet"), f.body));
                if f.editing {
                    ui.label(format!("{}: {}", self.i18n.t("rib-sketch", "Sketch"), f.sketch));
                } else {
                    egui::ComboBox::from_label(self.i18n.t("rib-sketch", "Sketch")).selected_text(f.sketch.clone()).show_ui(ui, |ui| {
                        for (sketch, body) in &sketches {
                            if ui.selectable_label(*sketch == f.sketch, sketch).clicked() {
                                (f.sketch, f.body) = (sketch.clone(), body.clone());
                            }
                        }
                    });
                }
                let shown = f.tool.clone().unwrap_or_else(|| self.i18n.t("rib-choose-tool", "Choose a tool").to_string());
                egui::ComboBox::from_label(self.i18n.t("rib-tool", "Tool")).selected_text(shown).show_ui(ui, |ui| {
                    for tool in &tools {
                        if ui.selectable_label(f.tool.as_ref() == Some(tool), tool).clicked() {
                            f.tool = Some(tool.clone());
                        }
                    }
                });
                if tools.is_empty() {
                    ui.label(self.i18n.t("rib-no-tools", "No tools yet: add them in the Tools table (View menu)"));
                }
                if flip_tile(ui, self.i18n.t("trim-flip", "Other side"), f.flip).clicked() {
                    f.flip = !f.flip;
                }
                if let Some(Err(why)) = preview {
                    ui.colored_label(ui.visuals().error_fg_color, why);
                }
                ui.horizontal(|ui| {
                    add = ui.add_enabled(ok, egui::Button::new(self.i18n.t("ok", "OK"))).clicked();
                    cancel = ui.button(self.i18n.t("cancel", "Cancel")).clicked();
                });
            });
        let (enter, esc) = ctx.input(|i| (i.key_pressed(egui::Key::Enter), i.key_pressed(egui::Key::Escape)));
        if cancel || esc {
            self.close_rib();
        } else if (add || enter) && ok {
            self.close_rib();
            if let Some((command, params)) = params {
                self.send(Request::Command(command, params));
            }
        } else if let Some((command, params)) = f.params()
            && f.sent.as_ref() != Some(&params)
        {
            f.sent = Some(params.clone());
            self.send(Request::Preview(Some((command, params))));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::RibForm;
    use crate::actions::Act;
    use crate::app::tests::viewer;
    use serde_json::json;

    /// B takes the newest sketch on a wall of sheet metal, and the body at the end of that sheet.
    #[test]
    fn b_offers_the_newest_sketch_on_the_sheet() {
        let (mut v, _inbox) = viewer("rib.json");
        v.model.as_mut().unwrap().bodies = vec!["rib2".into()];
        assert_eq!(v.rib_sketches(), [("path2".to_string(), "rib2".to_string()), ("path".to_string(), "rib2".to_string())]);
        v.open_rib(egui::pos2(0.0, 0.0));
        let f = v.rib.as_ref().unwrap();
        assert_eq!((f.sketch.as_str(), f.body.as_str(), f.tool.as_ref(), f.name.as_str()), ("path2", "rib2", None, "rib1"));
        // No sketch to run along: the form stays shut and says why.
        let (mut v, _inbox) = viewer("parametric.json");
        v.open_rib(egui::pos2(0.0, 0.0));
        assert!(v.rib.is_none() && v.contour_hint.is_some_and(|(_, _, a)| a == Act::Rib));
    }

    /// The rib's form asks for a tool and never chooses one; flipped it says so, an edit changes
    /// the tool and the side alone.
    #[test]
    fn a_rib_form_waits_for_a_tool() {
        let mut f = RibForm { name: "rib1".into(), body: "sheet".into(), sketch: "path".into(), tool: None, flip: false, editing: false, sent: None };
        assert_eq!(f.params(), None);
        f.tool = Some("Ролик".into());
        let add = json!({"name": "rib1", "op": "rib", "inputs": ["sheet", "path"], "tool": "Ролик"});
        assert_eq!(f.params(), Some(("add", add)));
        f.flip = true;
        assert_eq!(f.params().unwrap().1["flip"], true);
        f.editing = true;
        assert_eq!(f.params(), Some(("edit", json!({"name": "rib1", "tool": "Ролик", "flip": true}))));
    }
}
