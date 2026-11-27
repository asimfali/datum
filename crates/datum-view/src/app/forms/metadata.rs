//! The metadata editor of a sketch (D276): properties of the selected primitives, or of the
//! curves the sketch already marks, chosen from a list; presets fill the properties in (D277).
//! One Apply sends one `edit_metadata`, and one Undo takes it back; the window itself writes no
//! JSON. It stays open while the sketch is used: a selection made there does not retarget an
//! edit already begun.

use std::collections::{BTreeMap, BTreeSet};

use serde_json::{Map, Value as Json, json};

use crate::actions::Act;
use crate::app::{Request, Viewer};

/// A preset the window offers, its label in the window's language: a string, or a dictionary of
/// languages (falling back to English, then the ID).
pub(in crate::app) struct Preset {
    pub(in crate::app) id: String,
    pub(in crate::app) label: String,
    pub(in crate::app) set: Map<String, Json>,
}

/// What F12 opened: the primitives being changed, the properties as typed, and the presets on offer.
pub(in crate::app) struct MetadataForm {
    pub(in crate::app) feature: String,
    /// The primitives the edit targets: those selected when F12 was pressed, or one picked from the
    /// marked list. Fixed while the form is open: a selection made in the sketch is not taken.
    pub(in crate::app) targets: Vec<String>,
    /// The primitives with properties, to choose one when F12
    /// was pressed with nothing selected.
    pub(in crate::app) marked: Vec<String>,
    /// Each property as JSON text, by name; the names of all targets together.
    pub(in crate::app) values: BTreeMap<String, String>,
    /// The JSON text as it was: a property typed over is set, one untouched is left as it is.
    pub(in crate::app) initial: BTreeMap<String, String>,
    /// Properties whose targets disagree: no common value is shown, an empty field until typed.
    pub(in crate::app) mixed: BTreeSet<String>,
    /// Properties to take away from every target.
    pub(in crate::app) remove: BTreeSet<String>,
    /// A property to add: its name and its JSON text.
    pub(in crate::app) new_key: String,
    pub(in crate::app) new_value: String,
    pub(in crate::app) presets: Vec<Preset>,
    /// Why the presets could not all be read, shown as a technical line.
    pub(in crate::app) preset_problems: Vec<String>,
    /// What stops this Apply (a value that is no JSON, nothing to change), until the next.
    pub(in crate::app) notice: Option<String>,
}

impl MetadataForm {
    /// A preset fills the draft's properties: its values are what the model keeps on Apply, not
    /// a reference to the preset (D277).
    pub(in crate::app) fn use_preset(&mut self, id: &str) -> bool {
        let Some(p) = self.presets.iter().find(|p| p.id == id) else { return false };
        for (k, v) in &p.set {
            self.values.insert(k.clone(), serde_json::to_string(v).unwrap_or_default());
            self.initial.remove(k);
            self.mixed.remove(k);
            self.remove.remove(k);
        }
        true
    }

    /// Takes `targets` and their properties as the draft: what F12 opens on, and what a form
    /// without targets takes when a selection is made in the sketch (D276).
    pub(in crate::app) fn set_targets(&mut self, feature: &Json, targets: Vec<String>) {
        self.targets = targets;
        (self.values, self.initial, self.mixed) = draft(feature, &self.targets);
        self.remove.clear();
        self.notice = None;
    }
}

/// The sketch's own primitives in the selection.
pub(in crate::app) fn selection(sk: &crate::sketch::SketchEdit) -> Vec<String> {
    let curves: Vec<String> = sk
        .selected
        .iter()
        .filter_map(|p| match p {
            crate::sketch::Pick::Curve(n) | crate::sketch::Pick::Point(n) => Some(n.clone()),
            _ => None,
        })
        .filter(|n| primitive(&sk.json, n))
        .collect();
    unique(&curves)
}

impl Viewer {
    /// Opens the metadata editor on the open sketch: its selected primitives, or the list of
    /// the primitives it marks (D276). A second F12 takes the selection a form without targets was
    /// waiting for, and leaves a draft begun as it is.
    pub(in crate::app) fn open_metadata(&mut self) {
        {
            let d = &mut self.docs[self.active];
            if let Some(form) = &mut d.metadata {
                if form.targets.is_empty()
                    && let Some(sk) = &d.sketch
                    && form.feature == sk.feature
                {
                    let targets = selection(sk);
                    if !targets.is_empty() {
                        form.set_targets(&sk.json, targets);
                    }
                }
                return;
            }
        }
        let (feature, targets, marked, presets, values, initial, mixed) = {
            let Some(sk) = &self.docs[self.active].sketch else { return };
            let targets = selection(sk);
            let marked: Vec<String> =
                sk.json["sketch"]["metadata"].as_object().into_iter().flat_map(|m| m.keys()).filter(|n| primitive(&sk.json, n)).cloned().collect();
            let presets = self.metadata_presets();
            let (values, initial, mixed) = draft(&sk.json, &targets);
            (sk.feature.clone(), targets, marked, presets, values, initial, mixed)
        };
        self.docs[self.active].metadata = Some(MetadataForm {
            feature,
            targets,
            marked,
            values,
            initial,
            mixed,
            remove: BTreeSet::new(),
            new_key: String::new(),
            new_value: String::new(),
            presets: presets.0,
            preset_problems: presets.1,
            notice: None,
        });
    }

    /// The presets of the settings and of the project the open document works in (D277): the
    /// shipped ones, the engineer's and the project's, each in the language of the window.
    fn metadata_presets(&self) -> (Vec<Preset>, Vec<String>) {
        let listed = datum_model::presets::metadata_presets(self.settings.as_deref(), self.current_project().as_deref());
        let problems = listed["diagnostics"].as_array().into_iter().flatten().filter_map(Json::as_str).map(String::from).collect();
        let code = self.i18n.code().to_string();
        let presets = listed["presets"]
            .as_array()
            .into_iter()
            .flatten()
            .filter_map(|p| {
                Some(Preset {
                    id: p["id"].as_str()?.to_string(),
                    label: label(&p["label"], &code, p["id"].as_str().unwrap_or_default()),
                    set: p["set"].as_object()?.clone(),
                })
            })
            .collect();
        (presets, problems)
    }

    /// The editor window, while a sketch is open: not in the way of the sketch, which can still be
    /// selected in; Escape closes it without sending the draft.
    pub(in crate::app) fn metadata_window(&mut self, ctx: &egui::Context) {
        {
            let d = &mut self.docs[self.active];
            let Some(form) = &d.metadata else { return };
            if d.sketch.as_ref().is_none_or(|sk| sk.feature != form.feature) {
                d.metadata = None;
                return;
            }
            // Opened with nothing selected: a selection made in the sketch is what it waits for.
            if d.metadata.as_ref().is_some_and(|f| f.targets.is_empty())
                && let Some(sk) = &d.sketch
            {
                let targets = selection(sk);
                if !targets.is_empty()
                    && let Some(form) = &mut d.metadata
                {
                    form.set_targets(&sk.json, targets);
                }
            }
        }
        let mut open = self.docs[self.active].metadata.is_some();
        // A combo box's list takes Escape first; otherwise the editor closes without sending.
        let popup = ctx.any_popup_open();
        let mut close = !popup && ctx.input_mut(|i| i.consume_key(egui::Modifiers::NONE, egui::Key::Escape));
        let mut apply = false;
        egui::Window::new(self.i18n.label(Act::Metadata))
            .id(egui::Id::new("datum_metadata"))
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .anchor(egui::Align2::RIGHT_TOP, [-16.0, 48.0])
            .show(ctx, |ui| {
                let d = &mut self.docs[self.active];
                let Some(form) = d.metadata.as_mut() else { return };
                if form.targets.is_empty() {
                    if form.marked.is_empty() {
                        ui.label(self.i18n.t("metadata-none-marked", "Select a point or curve in the sketch"));
                        if ui.button(self.i18n.t("cancel", "Cancel")).clicked() {
                            close = true;
                        }
                        return;
                    }
                    ui.label(self.i18n.t("metadata-choose", "Marked elements"));
                    let marked = form.marked.clone();
                    for n in &marked {
                        if ui.selectable_label(false, n).clicked()
                            && let Some(sk) = d.sketch.as_ref()
                        {
                            form.set_targets(&sk.json, vec![n.clone()]);
                        }
                    }
                    if ui.button(self.i18n.t("cancel", "Cancel")).clicked() {
                        close = true;
                    }
                    return;
                }
                ui.label(self.i18n.f("metadata-elements", "Elements: { $names }", &[("names", &form.targets.join(", "))]));
                let mut picked: Option<String> = None;
                if !form.presets.is_empty() {
                    egui::ComboBox::from_id_salt("datum_metadata_preset").selected_text(self.i18n.t("metadata-preset", "Preset…")).show_ui(
                        ui,
                        |ui| {
                            for p in &form.presets {
                                if ui.selectable_label(false, &p.label).clicked() {
                                    picked = Some(p.id.clone());
                                }
                            }
                        },
                    );
                }
                if let Some(id) = picked {
                    form.use_preset(&id);
                }
                egui::Grid::new("datum_metadata_props").num_columns(3).striped(true).show(ui, |ui| {
                    let keys: Vec<String> = form.values.keys().cloned().collect();
                    for k in keys {
                        let mut out = form.remove.contains(&k);
                        ui.checkbox(&mut out, "").on_hover_text(self.i18n.t("metadata-remove", "Take the property away"));
                        if out {
                            form.remove.insert(k.clone());
                        } else {
                            form.remove.remove(&k);
                        }
                        ui.label(&k);
                        let mixed = form.mixed.contains(&k);
                        let text = form.values.entry(k.clone()).or_default();
                        let hint = if mixed && text.is_empty() { self.i18n.t("metadata-mixed", "mixed") } else { "JSON" };
                        let r = ui.add(egui::TextEdit::singleline(text).desired_width(180.0).hint_text(hint));
                        r.on_hover_text(self.i18n.t("metadata-value-hint", "The property's JSON value, as it will be saved"));
                        ui.end_row();
                    }
                });
                ui.separator();
                ui.horizontal(|ui| {
                    ui.add(egui::TextEdit::singleline(&mut form.new_key).desired_width(100.0).hint_text(self.i18n.t("metadata-key", "New property")));
                    ui.add(egui::TextEdit::singleline(&mut form.new_value).desired_width(180.0).hint_text("JSON"));
                });
                if !form.preset_problems.is_empty() {
                    ui.weak(self.i18n.t("metadata-presets-problems", "Some presets could not be read"))
                        .on_hover_text(form.preset_problems.join("\n"));
                }
                if let Some(n) = &form.notice {
                    ui.colored_label(ui.visuals().warn_fg_color, n);
                }
                ui.horizontal(|ui| {
                    apply = ui.button(self.i18n.t("ok", "OK")).clicked();
                    close |= ui.button(self.i18n.t("cancel", "Cancel")).clicked();
                });
            });
        if apply
            && let Err(why) = self.apply_metadata()
            && let Some(form) = &mut self.docs[self.active].metadata
        {
            form.notice = Some(why);
        }
        if close || !open {
            self.docs[self.active].metadata = None;
        }
    }

    /// Sends the draft as one `edit_metadata`, unless nothing changed or a value is no JSON.
    pub(in crate::app) fn apply_metadata(&mut self) -> Result<(), String> {
        let Some(form) = self.docs[self.active].metadata.as_ref() else { return Ok(()) };
        let mut set = Map::new();
        for (k, text) in &form.values {
            if form.remove.contains(k) {
                continue;
            }
            if form.mixed.contains(k) && text.is_empty() {
                continue;
            }
            if !form.mixed.contains(k) && form.initial.get(k) == Some(text) {
                continue;
            }
            let value = serde_json::from_str::<Json>(text).map_err(|e| format!("`{k}`: {e}"))?;
            set.insert(k.clone(), value);
        }
        if !form.new_key.is_empty() {
            if form.values.contains_key(&form.new_key) {
                return Err(format!("`{}` is listed already: change its value here", form.new_key));
            }
            let value = serde_json::from_str::<Json>(&form.new_value).map_err(|e| format!("`{}`: {e}", form.new_key))?;
            set.insert(form.new_key.clone(), value);
        }
        let remove: Vec<Json> = form.remove.iter().map(|k| json!(k)).collect();
        if set.is_empty() && remove.is_empty() {
            return Err(self.i18n.t("metadata-nothing", "Nothing to change").to_string());
        }
        let params = json!({"name": form.feature, "elements": form.targets, "set": set, "remove": remove});
        self.send(Request::Command("edit_metadata", params));
        self.docs[self.active].metadata = None;
        Ok(())
    }
}

/// Whether the sketch feature defines the named primitive.
fn primitive(feature: &Json, n: &str) -> bool {
    datum_sketch::Sketch::PRIMITIVE_FIELDS.into_iter().any(|k| feature["sketch"][k].get(n).is_some())
}

/// `names` in their order, each once.
fn unique(names: &[String]) -> Vec<String> {
    let mut out: Vec<String> = Vec::with_capacity(names.len());
    for n in names {
        if !out.contains(n) {
            out.push(n.clone());
        }
    }
    out
}

/// The properties of `targets`: each name once, its JSON text when they all say the same, and
/// the names they disagree on (a missing property disagrees with a value).
#[allow(clippy::type_complexity)]
fn draft(feature: &Json, targets: &[String]) -> (BTreeMap<String, String>, BTreeMap<String, String>, BTreeSet<String>) {
    let mut names: BTreeSet<String> = BTreeSet::new();
    for n in targets {
        if let Some(m) = feature["sketch"]["metadata"][n].as_object() {
            names.extend(m.keys().cloned());
        }
    }
    let mut values = BTreeMap::new();
    let mut initial = BTreeMap::new();
    let mut mixed = BTreeSet::new();
    for name in names {
        let all: Vec<Option<&Json>> = targets.iter().map(|n| feature["sketch"]["metadata"][n].get(&name)).collect();
        match all.first().copied().flatten() {
            Some(first) if all.iter().all(|v| *v == Some(first)) => {
                let text = serde_json::to_string(first).unwrap_or_default();
                values.insert(name.clone(), text.clone());
                initial.insert(name, text);
            }
            _ => {
                mixed.insert(name.clone());
                values.insert(name, String::new());
            }
        }
    }
    (values, initial, mixed)
}

/// A preset's label: text as written, or a dictionary of languages taken in `code`, then English,
/// then the ID.
fn label(label: &Json, code: &str, id: &str) -> String {
    match label {
        Json::String(s) => s.clone(),
        Json::Object(m) => m.get(code).or_else(|| m.get("en")).and_then(Json::as_str).map_or_else(|| id.to_string(), String::from),
        _ => id.to_string(),
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    /// A preset's label is text as written, or the window's language, then English, then the ID.
    #[test]
    fn a_label_is_text_or_a_dictionary() {
        assert_eq!(label(&json!("Cut"), "ru", "cut"), "Cut");
        assert_eq!(label(&json!({"ru": "Рез", "en": "Cut"}), "ru", "cut"), "Рез");
        assert_eq!(label(&json!({"ru": "Рез", "en": "Cut"}), "de", "cut"), "Cut");
        assert_eq!(label(&json!({"ru": "Рез"}), "de", "cut"), "cut");
        assert_eq!(label(&Json::Null, "ru", "cut"), "cut");
    }
}
