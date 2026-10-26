//! The side bar (D162, D163), as in VS Code: an activity bar of icons at the window's edge, each a
//! set of sections shown in the side bar one under another, folded or open, each as tall as what
//! it holds (pressed to the one above; a long one scrolls). The part's set holds its parts,
//! parameters and features, and while a sketch is edited the sketch under them. The side bar goes
//! left or right, hides, and is kept between runs.

use serde::{Deserialize, Serialize};

/// A set of sections, by its icon: the environments, as in Inventor (a part; drawings and
/// assemblies come with their content).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Container {
    Part,
}

impl Container {
    pub const ALL: [Container; 1] = [Container::Part];

    pub fn title(self) -> &'static str {
        match self {
            Container::Part => "Part: its parts, parameters, features and the sketch being edited",
        }
    }

    pub fn sections(self) -> &'static [Section] {
        match self {
            Container::Part => &[Section::Parts, Section::Parameters, Section::Features, Section::Sketch],
        }
    }
}

/// A section of the side bar.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Section {
    Parts,
    Parameters,
    Features,
    Sketch,
}

impl Section {
    pub fn title(self) -> &'static str {
        match self {
            Section::Parts => "PARTS",
            Section::Parameters => "PARAMETERS",
            Section::Features => "FEATURES",
            Section::Sketch => "SKETCH",
        }
    }
}

/// Where the side bar is and what it shows; what is folded.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct SideBar {
    pub right: bool,
    pub visible: bool,
    pub container: Container,
    pub folded: Vec<Section>,
}

impl Default for SideBar {
    fn default() -> Self {
        Self { right: false, visible: true, container: Container::Part, folded: Vec::new() }
    }
}

/// Pixels of a section's title row.
pub const HEADER: f32 = 22.0;

impl SideBar {
    pub fn is_open(&self, s: Section) -> bool {
        !self.folded.contains(&s)
    }

    pub fn toggle(&mut self, s: Section) {
        if self.is_open(s) {
            self.folded.push(s);
        } else {
            self.folded.retain(|f| *f != s);
        }
    }

    /// An icon clicked: its set shown; its set again, the side bar hidden (or shown back).
    pub fn click(&mut self, c: Container) {
        if self.container == c {
            self.visible = !self.visible;
        } else {
            self.container = c;
            self.visible = true;
        }
    }

    pub fn to_text(&self) -> String {
        serde_json::to_string(self).unwrap_or_default()
    }

    pub fn from_text(text: &str) -> Self {
        serde_json::from_str(text).unwrap_or_default()
    }
}

/// A section's title row: a fold arrow and the title, the hint on hover; clicked, true.
pub fn header(ui: &mut egui::Ui, s: Section, open: bool, hint: &str) -> bool {
    let (rect, r) = ui.allocate_exact_size(egui::vec2(ui.available_width(), HEADER), egui::Sense::click());
    r.widget_info(|| egui::WidgetInfo::labeled(egui::WidgetType::CollapsingHeader, true, s.title()));
    let visuals = ui.visuals();
    let fill = if r.hovered() { visuals.widgets.hovered.weak_bg_fill } else { visuals.faint_bg_color };
    let colour = visuals.strong_text_color();
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 0.0, fill);
    // The arrow, drawn: down when open, right when folded.
    let c = egui::pos2(rect.left() + 12.0, rect.center().y);
    let arrow = if open {
        vec![c + egui::vec2(-4.0, -2.0), c + egui::vec2(4.0, -2.0), c + egui::vec2(0.0, 3.0)]
    } else {
        vec![c + egui::vec2(-2.0, -4.0), c + egui::vec2(3.0, 0.0), c + egui::vec2(-2.0, 4.0)]
    };
    painter.add(egui::Shape::convex_polygon(arrow, colour, egui::Stroke::NONE));
    painter.text(egui::pos2(rect.left() + 22.0, rect.center().y), egui::Align2::LEFT_CENTER, s.title(), egui::FontId::proportional(11.5), colour);
    let r = if hint.is_empty() { r } else { r.on_hover_text(hint) };
    r.clicked()
}

/// An icon of the activity bar, drawn (the window's font may lack a glyph), about 20 pixels.
pub fn icon(painter: &egui::Painter, c: egui::Pos2, container: Container, colour: egui::Color32) {
    let stroke = egui::Stroke::new(1.8, colour);
    let at = |x: f32, y: f32| c + egui::vec2(x, y);
    match container {
        // A part: a block seen from a corner.
        Container::Part => {
            let front = [at(-7.0, -2.0), at(2.0, -2.0), at(2.0, 8.0), at(-7.0, 8.0)];
            painter.add(egui::Shape::closed_line(front.to_vec(), stroke));
            painter.add(egui::Shape::line(vec![at(-7.0, -2.0), at(-2.0, -7.0), at(7.0, -7.0), at(2.0, -2.0)], stroke));
            painter.add(egui::Shape::line(vec![at(7.0, -7.0), at(7.0, 3.0), at(2.0, 8.0)], stroke));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sections_fold_and_are_kept() {
        let mut b = SideBar::default();
        assert!(b.is_open(Section::Parts));
        b.toggle(Section::Parts);
        assert!(!b.is_open(Section::Parts));
        b.click(Container::Part);
        assert!(!b.visible, "its icon again: hidden");
        b.click(Container::Part);
        assert!(b.visible);
        b.right = true;
        assert_eq!(SideBar::from_text(&b.to_text()), b);
        assert_eq!(SideBar::from_text("nonsense"), SideBar::default());
        assert!(SideBar::from_text(r#"{"right": true, "weight": {"Parts": 2.0}}"#).right, "an older one read");
    }
}
