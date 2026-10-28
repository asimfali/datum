//! The window's main panes (D161): the 3D view, the flat pattern, the sketch library (later
//! drawings, documents),
//! each a tab of a dock (`egui_dock`): put left or right, below, together as tabs, closed and
//! opened again from the View menu. The layout is kept between runs. The model's sections live in
//! the side bar (D162).

use egui_dock::DockState;
use serde::{Deserialize, Serialize};

use crate::i18n::I18n;

/// A pane of the window.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Pane {
    /// The model in 3D.
    Model,
    /// The flat pattern of a body.
    Flat,
    /// Sketch elements to place: open while a sketch is edited (D163).
    Library,
}

impl Pane {
    pub fn title(self, i18n: &I18n) -> &str {
        match self {
            Pane::Model => "3D",
            Pane::Flat => i18n.t("pane-flat", "Flat pattern"),
            Pane::Library => i18n.t("pane-library", "Library"),
        }
    }
}

/// The layout a first run starts with: the 3D view; the flat pattern opens beside it when asked
/// for.
pub fn default_layout() -> DockState<Pane> {
    DockState::new(vec![Pane::Model])
}

/// Whether `pane` is open.
pub fn is_open(dock: &DockState<Pane>, pane: Pane) -> bool {
    dock.find_tab(&pane).is_some()
}

/// Opens `pane` where it belongs (the flat pattern and the library to the right of the 3D view),
/// or brings it to the front if open.
pub fn open(dock: &mut DockState<Pane>, pane: Pane) {
    if let Some(path) = dock.find_tab(&pane) {
        let _ = dock.set_active_tab(path);
        return;
    }
    match dock.find_main_surface_tab(&Pane::Model) {
        Some((model, _)) if pane == Pane::Flat => {
            dock.main_surface_mut().split_right(model, 0.5, vec![pane]);
        }
        Some((model, _)) if pane == Pane::Library => {
            dock.main_surface_mut().split_right(model, 0.8, vec![pane]);
        }
        _ => dock.push_to_first_leaf(pane),
    }
}

/// Closes `pane` if it is open.
pub fn close(dock: &mut DockState<Pane>, pane: Pane) {
    if let Some(path) = dock.find_tab(&pane) {
        dock.remove_tab(path);
    }
}

/// The layout as kept between runs, and back; a layout that does not read (or lacks the 3D
/// view) gives the default.
pub fn to_text(dock: &DockState<Pane>) -> String {
    serde_json::to_string(dock).unwrap_or_default()
}

pub fn from_text(text: &str) -> DockState<Pane> {
    let Ok(mut v) = serde_json::from_str::<serde_json::Value>(text) else { return default_layout() };
    unbounded(&mut v);
    serde_json::from_value::<DockState<Pane>>(v).ok().filter(|d| is_open(d, Pane::Model)).unwrap_or_else(default_layout)
}

/// A pane's rectangle not laid out yet is infinite, which JSON writes as null: read as 0, it is
/// laid out again on the next frame anyway.
fn unbounded(v: &mut serde_json::Value) {
    match v {
        serde_json::Value::Object(m) => {
            if m.len() == 2 && m.contains_key("x") && m.contains_key("y") {
                for c in m.values_mut().filter(|c| c.is_null()) {
                    *c = serde_json::json!(0.0);
                }
            }
            m.values_mut().for_each(unbounded);
        }
        serde_json::Value::Array(a) => a.iter_mut().for_each(unbounded),
        _ => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn panes_open_close_and_are_kept() {
        let mut dock = default_layout();
        assert!(is_open(&dock, Pane::Model) && !is_open(&dock, Pane::Flat));
        open(&mut dock, Pane::Flat);
        assert!(is_open(&dock, Pane::Flat));
        let back = from_text(&to_text(&dock));
        assert!(is_open(&back, Pane::Flat) && is_open(&back, Pane::Model));
        close(&mut dock, Pane::Flat);
        assert!(!is_open(&dock, Pane::Flat));
        assert!(!is_open(&from_text("not a layout"), Pane::Flat), "the default");
    }
}
