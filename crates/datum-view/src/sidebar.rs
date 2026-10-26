//! The side bar (D162), as in VS Code: an activity bar of icons at the window's edge, each a set of
//! sections shown in the side bar one under another, folded or open, each scrolled by itself, the
//! border between two open ones dragged. The model's set holds its parts, parameters and features;
//! the sketch's, the sketch being edited and the library. The side bar goes left or right, hides,
//! and is kept between runs.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

/// A set of sections, by its icon.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Container {
    Model,
    Sketch,
}

impl Container {
    pub const ALL: [Container; 2] = [Container::Model, Container::Sketch];

    pub fn title(self) -> &'static str {
        match self {
            Container::Model => "Model: parts, parameters, features",
            Container::Sketch => "Sketch: the one being edited, and the library",
        }
    }

    pub fn sections(self) -> &'static [Section] {
        match self {
            Container::Model => &[Section::Parts, Section::Parameters, Section::Features],
            Container::Sketch => &[Section::Sketch, Section::Library],
        }
    }
}

/// A section of the side bar.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum Section {
    Parts,
    Parameters,
    Features,
    Sketch,
    Library,
}

impl Section {
    pub fn title(self) -> &'static str {
        match self {
            Section::Parts => "PARTS",
            Section::Parameters => "PARAMETERS",
            Section::Features => "FEATURES",
            Section::Sketch => "SKETCH",
            Section::Library => "LIBRARY",
        }
    }
}

/// Where the side bar is and what it shows; what is folded, and the share of the height each open
/// section takes.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct SideBar {
    pub right: bool,
    pub visible: bool,
    pub container: Container,
    pub folded: Vec<Section>,
    pub weight: BTreeMap<Section, f32>,
}

impl Default for SideBar {
    fn default() -> Self {
        Self { right: false, visible: true, container: Container::Model, folded: Vec::new(), weight: BTreeMap::new() }
    }
}

/// Pixels of a section's title row, and of the border between two open sections.
pub const HEADER: f32 = 22.0;
pub const BORDER: f32 = 5.0;

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

    /// Its share of the height: as dragged, or at first the parts small and the features large.
    pub fn weight(&self, s: Section) -> f32 {
        self.weight.get(&s).copied().unwrap_or(match s {
            Section::Parts => 0.6,
            Section::Features => 1.4,
            _ => 1.0,
        })
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

    /// The border below `a` dragged by `dy` pixels of `body` pixels shared by the open sections:
    /// `a` grows, `b` (the next open one) shrinks, neither below a tenth of a share.
    pub fn drag(&mut self, a: Section, b: Section, dy: f32, body: f32) {
        let open: f32 = self.container.sections().iter().filter(|s| self.is_open(**s)).map(|s| self.weight(*s)).sum();
        if body <= 0.0 {
            return;
        }
        let d = dy / body * open;
        let (wa, wb) = (self.weight(a), self.weight(b));
        let d = d.clamp(0.1 - wa, wb - 0.1);
        self.weight.insert(a, wa + d);
        self.weight.insert(b, wb - d);
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
        // A tree: a trunk and three branches ending in boxes.
        Container::Model => {
            painter.line_segment([at(-7.0, -8.0), at(-7.0, 6.0)], stroke);
            for y in [-6.0, 0.0, 6.0] {
                painter.line_segment([at(-7.0, y), at(-2.0, y)], stroke);
                painter.rect_stroke(egui::Rect::from_center_size(at(3.0, y), egui::vec2(8.0, 4.0)), 0.0, stroke, egui::StrokeKind::Middle);
            }
        }
        // A pencil over a line.
        Container::Sketch => {
            let body = [at(-6.0, 4.0), at(-8.0, 8.0), at(-4.0, 6.0), at(6.0, -4.0), at(4.0, -6.0), at(-6.0, 4.0)];
            painter.add(egui::Shape::line(body.to_vec(), stroke));
            painter.line_segment([at(2.0, -4.0), at(4.0, -2.0)], stroke);
            painter.line_segment([at(-2.0, 9.0), at(9.0, 9.0)], stroke);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sections_fold_borders_move_and_are_kept() {
        let mut b = SideBar::default();
        assert!(b.is_open(Section::Parts));
        b.toggle(Section::Parts);
        assert!(!b.is_open(Section::Parts));
        b.drag(Section::Parameters, Section::Features, 50.0, 200.0);
        assert!(b.weight(Section::Parameters) > 1.0 && b.weight(Section::Features) < 1.4);
        b.drag(Section::Parameters, Section::Features, 10_000.0, 200.0);
        assert!((b.weight(Section::Features) - 0.1).abs() < 1e-6, "not below a tenth");
        b.click(Container::Model);
        assert!(!b.visible, "its icon again: hidden");
        b.click(Container::Sketch);
        assert!(b.visible && b.container == Container::Sketch);
        b.right = true;
        assert_eq!(SideBar::from_text(&b.to_text()), b);
        assert_eq!(SideBar::from_text("nonsense"), SideBar::default());
    }
}
