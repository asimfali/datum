//! The loft form and its live preview.

use serde_json::{Value as Json, json};

use crate::actions::Act;
use crate::app::{Request, Viewer, definition};
use crate::dims::value_of;

#[derive(Clone, Copy, PartialEq, Eq)]
enum ProfilePick {
    First,
    Second,
}

pub(in crate::app) struct LoftForm {
    name: String,
    first: String,
    second: String,
    tolerance: String,
    side: &'static str,
    rule: Option<String>,
    editing: bool,
    sent: Option<Json>,
    picking: Option<ProfilePick>,
}

impl LoftForm {
    pub(in crate::app) fn editing(name: String, feature: &Json) -> Self {
        let side = match feature["side"].as_str() {
            Some("middle") => "middle",
            Some("right") => "right",
            _ => "left",
        };
        Self {
            name,
            first: feature["inputs"][0].as_str().unwrap_or_default().into(),
            second: feature["inputs"][1].as_str().unwrap_or_default().into(),
            tolerance: if feature["tolerance"].is_null() { "1".into() } else { definition(&feature["tolerance"]) },
            side,
            rule: feature["rule"].as_str().map(String::from),
            editing: true,
            sent: None,
            picking: None,
        }
    }

    pub(in crate::app) fn edited(&self) -> Option<&String> {
        self.editing.then_some(&self.name)
    }

    fn params(&self) -> Option<(&'static str, Json)> {
        if self.first.is_empty() || self.second.is_empty() || self.first == self.second || self.tolerance.trim().is_empty() {
            return None;
        }
        let tolerance = value_of(&self.tolerance);
        if self.editing {
            Some(("edit", json!({"name": self.name, "inputs": [self.first, self.second], "tolerance": tolerance, "side": self.side})))
        } else {
            Some((
                "loft",
                json!({"name": self.name, "first": self.first, "second": self.second, "rule": self.rule.as_ref()?, "tolerance": tolerance, "side": self.side}),
            ))
        }
    }
}

impl Viewer {
    pub(in crate::app) fn open_loft(&mut self, at: egui::Pos2) {
        self.clear_loft_selection();
        self.contour_hint = None;
        let Some(rule) = self.loft_toolbar_rule() else {
            self.contour_hint = Some((at, std::time::Instant::now(), Act::Loft));
            return;
        };
        self.loft = Some(LoftForm {
            name: self.feature_name("loft"),
            first: String::new(),
            second: String::new(),
            tolerance: "1".into(),
            side: "left",
            rule: Some(rule),
            editing: false,
            sent: None,
            picking: Some(ProfilePick::First),
        });
    }

    pub(in crate::app) fn loft_is_picking(&self) -> bool {
        self.loft.as_ref().is_some_and(|f| f.picking.is_some())
    }

    pub(in crate::app) fn loft_can_pick(&self, name: &str) -> bool {
        let Some(f) = &self.loft else { return false };
        let Some(which) = f.picking else { return false };
        if name == if which == ProfilePick::First { &f.second } else { &f.first } {
            return false;
        }
        self.model.as_ref().and_then(|m| m.inspect["features"].as_array()).is_some_and(|features| {
            features.iter().take_while(|g| !f.editing || g["name"] != f.name.as_str()).any(|g| g["op"] == "sketch" && g["name"] == name)
        })
    }

    pub(in crate::app) fn pick_loft_profile(&mut self, name: &str) {
        if !self.loft_can_pick(name) {
            return;
        }
        let Some(f) = &mut self.loft else { return };
        match f.picking {
            Some(ProfilePick::First) => {
                f.first = name.into();
                f.picking = if f.second.is_empty() { Some(ProfilePick::Second) } else { None };
            }
            Some(ProfilePick::Second) => {
                f.second = name.into();
                f.picking = None;
            }
            _ => {}
        }
    }

    pub(in crate::app) fn loft_pick_hint(&mut self, ctx: &egui::Context) {
        if !self.loft_is_picking() {
            return;
        }
        if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            self.close_loft();
            return;
        }
        if self.loft.as_ref().is_some_and(|f| !f.editing) && self.loft_toolbar_rule().is_none() {
            let at = ctx.pointer_hover_pos().unwrap_or(egui::Pos2::ZERO);
            self.close_loft();
            self.contour_hint = Some((at, std::time::Instant::now(), Act::Loft));
            return;
        }
        let Some(at) = ctx.pointer_hover_pos() else { return };
        let first = self.loft.as_ref().is_some_and(|f| f.picking == Some(ProfilePick::First));
        let label = if first {
            self.i18n.t("loft-pick-first", "Select the first profile")
        } else {
            self.i18n.t("loft-pick-second", "Select the second profile")
        };
        egui::Area::new(egui::Id::new("loft-pick-hint")).fixed_pos(at + egui::vec2(16.0, 20.0)).order(egui::Order::Tooltip).interactable(false).show(
            ctx,
            |ui| {
                egui::Frame::popup(ui.style()).show(ui, |ui| {
                    ui.label(label);
                });
            },
        );
    }

    pub(in crate::app) fn clear_loft_selection(&mut self) {
        if self.flange.take().is_some() {
            self.send(Request::Preview(None));
        }
        self.flange_edges.clear();
        self.picking = None;
        self.under = None;
        if let Some(m) = &mut self.model {
            m.preview = None;
        }
    }

    pub(in crate::app) fn close_loft(&mut self) {
        if let Some(f) = self.loft.take()
            && f.sent.is_some()
        {
            self.send(Request::Preview(None));
        }
    }

    pub(in crate::app) fn loft_window(&mut self, ctx: &egui::Context) {
        if self.loft.is_none() {
            return;
        }
        let rule = self.loft_toolbar_rule();
        if self.loft.as_ref().is_some_and(|f| !f.editing) && rule.is_none() {
            self.close_loft();
            self.contour_hint = Some((ctx.pointer_hover_pos().unwrap_or(egui::Pos2::ZERO), std::time::Instant::now(), Act::Loft));
            return;
        }
        let Some(f) = &mut self.loft else { return };
        if !f.editing {
            f.rule = rule;
        }
        if f.picking.is_some() {
            return;
        }
        let before = f.params();
        let preview =
            self.model.as_ref().and_then(|m| m.preview.as_ref()).filter(|(p, _)| Some(p) == before.as_ref().map(|(_, p)| p)).map(|(_, r)| r);
        let (mut add, mut cancel) = (false, false);
        egui::Window::new(self.i18n.label(Act::Loft))
            .collapsible(false)
            .resizable(false)
            .anchor(egui::Align2::CENTER_TOP, egui::vec2(0.0, 60.0))
            .show(ctx, |ui| {
                for (index, label, selected) in [
                    (ProfilePick::First, self.i18n.t("loft-first-profile", "First profile"), &f.first),
                    (ProfilePick::Second, self.i18n.t("loft-second-profile", "Second profile"), &f.second),
                ] {
                    ui.horizontal(|ui| {
                        ui.label(format!("{label}: {selected}"));
                        if ui.button(self.i18n.t("loft-repick-profile", "Select again")).clicked() {
                            f.picking = Some(index);
                        }
                    });
                }
                ui.horizontal(|ui| {
                    ui.label(self.i18n.t("loft-chord-tolerance", "Chord tolerance (mm)"));
                    ui.add(
                        egui::TextEdit::singleline(&mut f.tolerance)
                            .desired_width(120.0)
                            .hint_text(self.i18n.t("plane-offset-hint", "number or formula")),
                    );
                });
                ui.horizontal(|ui| {
                    ui.label(self.i18n.t("loft-side", "Side"));
                    ui.radio_value(&mut f.side, "middle", self.i18n.t("loft-middle", "Middle"));
                    ui.radio_value(&mut f.side, "right", self.i18n.t("loft-outside", "Outside"));
                    ui.radio_value(&mut f.side, "left", self.i18n.t("loft-inside", "Inside"));
                })
                .response
                .on_hover_text(
                    self.i18n.t("loft-side-hint", "For arcs, inside is toward the centre. Use the preview to check the side of line profiles."),
                );
                if before == f.params()
                    && let Some(Err(why)) = preview
                {
                    ui.colored_label(ui.visuals().error_fg_color, why);
                }
                let ok = f.picking.is_none() && before == f.params() && preview.is_some_and(Result::is_ok);
                ui.horizontal(|ui| {
                    add = ui.add_enabled(ok, egui::Button::new(self.i18n.t("ok", "OK"))).clicked();
                    cancel = ui.button(self.i18n.t("cancel", "Cancel")).clicked();
                });
            });
        let params = f.params();
        let ok = f.picking.is_none() && before == params && preview.is_some_and(Result::is_ok);
        let (enter, esc) = ctx.input(|i| (i.key_pressed(egui::Key::Enter), i.key_pressed(egui::Key::Escape)));
        if cancel || esc {
            self.close_loft();
        } else if (add || enter) && ok {
            self.close_loft();
            if let Some((command, params)) = params {
                if command == "loft" {
                    self.part = params["name"].as_str().map(String::from);
                }
                self.send(Request::Command(command, params));
            }
        } else if f.picking.is_some() {
            if f.sent.take().is_some() {
                self.send(Request::Preview(None));
            }
        } else if let Some((command, params)) = params {
            if f.sent.as_ref() != Some(&params) {
                f.sent = Some(params.clone());
                if let Some(m) = &mut self.model {
                    m.preview = None;
                }
                self.send(Request::Preview(Some((command, params))));
            }
        } else if f.sent.take().is_some() {
            if let Some(m) = &mut self.model {
                m.preview = None;
            }
            self.send(Request::Preview(None));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::LoftForm;
    use crate::app::{Request, Viewer, tests::viewer};
    use serde_json::json;

    fn frame(v: &mut Viewer, key: Option<egui::Key>) {
        let mut h = egui_kittest::Harness::builder().with_size([720.0, 380.0]).build_ui(|ui| v.loft_window(ui.ctx()));
        if let Some(key) = key {
            h.event(egui::Event::Key { key, physical_key: None, pressed: true, repeat: false, modifiers: egui::Modifiers::NONE });
            h.run();
        }
    }

    #[test]
    fn loft_without_toolbar_rule_does_not_start_picking() {
        let (mut v, _) = viewer("loft-01-line.json");
        v.rules.clear();
        v.next_rule = None;
        v.open_loft(egui::pos2(10.0, 20.0));
        assert!(v.loft.is_none());
        assert!(!v.loft_is_picking());
        assert!(v.contour_hint.is_some_and(|(_, _, action)| action == crate::actions::Act::Loft));
    }

    #[test]
    fn only_the_current_successful_preview_can_make_a_loft() {
        let (mut v, inbox) = viewer("loft-01-line.json");
        v.rules = vec![json!({"name":"Sheet","kind":"sheet"})];
        v.next_rule = Some("Sheet".into());
        v.open_loft(egui::Pos2::ZERO);
        assert!(v.loft_is_picking());
        v.pick_loft_profile("profile_a");
        assert!(!v.loft_can_pick("profile_a"));
        v.pick_loft_profile("profile_b");
        assert!(!v.loft_is_picking());
        frame(&mut v, None);
        let sent: Vec<_> = inbox.try_iter().collect();
        let [Request::Preview(Some(("loft", initial)))] = sent.as_slice() else { panic!("preview: {sent:?}") };
        v.model.as_mut().unwrap().preview = Some((initial.clone(), Ok(json!({}))));
        v.loft.as_mut().unwrap().tolerance = "0.25".into();
        frame(&mut v, Some(egui::Key::Enter));
        let sent: Vec<_> = inbox.try_iter().collect();
        let [Request::Preview(Some(("loft", fresh)))] = sent.as_slice() else { panic!("new preview: {sent:?}") };
        assert_eq!(fresh["tolerance"], 0.25);
        v.model.as_mut().unwrap().preview = Some((fresh.clone(), Ok(json!({}))));
        frame(&mut v, Some(egui::Key::Enter));
        let sent: Vec<_> = inbox.try_iter().collect();
        assert!(matches!(sent.as_slice(), [Request::Preview(None), Request::Command("loft", params)] if params == fresh), "{sent:?}");
        assert!(v.loft.is_none());
    }

    #[test]
    fn loft_needs_two_distinct_profiles_and_a_rule() {
        let mut f = LoftForm {
            name: "loft1".into(),
            first: "a".into(),
            second: "a".into(),
            tolerance: "1".into(),
            side: "left",
            rule: Some("Sheet".into()),
            editing: false,
            sent: None,
            picking: None,
        };
        assert!(f.params().is_none());
        f.second = "b".into();
        assert_eq!(
            f.params(),
            Some(("loft", json!({"name": "loft1", "first": "a", "second": "b", "rule": "Sheet", "tolerance": 1.0, "side": "left"})))
        );
        f.tolerance.clear();
        assert!(f.params().is_none());
        f.tolerance = "pitch / 2".into();
        f.editing = true;
        assert_eq!(f.params(), Some(("edit", json!({"name": "loft1", "inputs": ["a", "b"], "tolerance": "pitch / 2", "side": "left"}))));
    }
}
