//! Icons drawn in lines (D119): the window's font has few symbols, and what it lacks shows as a
//! box on Windows (↶ ↷ ✎ ⇄). One tile for every icon button and the constraint glyphs.

use egui::{Color32, Painter, Pos2, Response, Sense, Stroke, Ui, Vec2, vec2};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Icon {
    Undo,
    Redo,
    /// A pencil: edit.
    Edit,
    /// Move up in a list.
    Up,
    /// Take away.
    Remove,
    /// A house: the home view.
    Home,
    /// Two arrows opposite: the other way.
    Flip,
    /// Sketch tools: the cursor, a line, an arc, a circle, a point, a dimension.
    Select,
    Line,
    Arc,
    Circle,
    Point,
    Dimension,
    /// The top bar's commands: a plane with a pencil, a plane offset (an arrow off it), an eye,
    /// a ruler.
    NewSketch,
    WorkPlane,
    LookAt,
    Measure,
}

/// A square button with `paint` drawing its face in the given colour: `name` for screen
/// readers and tests, `hint` on hover, highlighted when `on`.
pub fn tile(ui: &mut Ui, name: &str, hint: &str, on: bool, enabled: bool, paint: impl FnOnce(&Painter, Pos2, Color32)) -> Response {
    let sense = if enabled { Sense::click() } else { Sense::hover() };
    // As high as a button with text, so a row of both lines up.
    let side = ui.spacing().interact_size.y.max(20.0);
    let (rect, r) = ui.allocate_exact_size(vec2(side, side), sense);
    r.widget_info(|| egui::WidgetInfo::selected(egui::WidgetType::Button, enabled, on, name));
    let visuals = ui.visuals();
    let fill = if on {
        visuals.selection.bg_fill
    } else if enabled && r.hovered() {
        visuals.widgets.hovered.weak_bg_fill
    } else {
        visuals.widgets.inactive.weak_bg_fill
    };
    ui.painter().rect_filled(rect, 3.0, fill);
    let colour = if enabled { visuals.strong_text_color() } else { visuals.weak_text_color() };
    paint(ui.painter(), rect.center(), colour);
    r.on_hover_text(hint)
}

/// An icon button.
pub fn button(ui: &mut Ui, icon: Icon, name: &str, hint: &str, enabled: bool) -> Response {
    tile(ui, name, hint, false, enabled, |p, c, colour| paint(p, c, icon, colour))
}

/// Draws `icon` around `c`, about 12 pixels across.
pub fn paint(painter: &Painter, c: Pos2, icon: Icon, colour: Color32) {
    let stroke = Stroke::new(1.6, colour);
    let at = |x: f32, y: f32| c + vec2(x, y);
    // A hook: an arc over the top from the right to the left, its point down at the left end;
    // mirrored for redo.
    let turn = |mirror: f32| {
        let r = 5.0;
        let pts: Vec<Pos2> = (0..=20)
            .map(|k| {
                let a = (-35.0 + 215.0 * k as f32 / 20.0).to_radians();
                at(mirror * r * a.cos(), 1.5 - r * a.sin())
            })
            .collect();
        let end = pts[pts.len() - 1];
        painter.add(egui::Shape::line(pts, stroke));
        painter.add(egui::Shape::line(vec![end + Vec2::new(-3.0, -3.0), end, end + Vec2::new(3.0, -3.0)], stroke));
    };
    // A plane seen aslant, low in the icon.
    let plane = || vec![at(-7.0, 6.5), at(3.0, 6.5), at(7.0, 1.5), at(-3.0, 1.5)];
    match icon {
        Icon::Undo => turn(1.0),
        Icon::Redo => turn(-1.0),
        Icon::Edit => {
            // The pencil's body along the diagonal, its point at the lower left.
            let body = [at(-4.0, 6.0), at(-5.5, 6.5), at(-5.0, 5.0), at(4.0, -4.0), at(6.0, -2.0), at(-4.0, 6.0)];
            painter.add(egui::Shape::line(body.to_vec(), stroke));
            painter.line_segment([at(2.5, -5.5), at(5.5, -2.5)], stroke);
            painter.line_segment([at(-5.0, 5.0), at(-4.0, 6.0)], stroke);
        }
        Icon::Up => {
            painter.add(egui::Shape::convex_polygon(vec![at(0.0, -5.0), at(5.5, 3.5), at(-5.5, 3.5)], colour, Stroke::NONE));
        }
        Icon::Remove => {
            painter.line_segment([at(-5.0, -5.0), at(5.0, 5.0)], stroke);
            painter.line_segment([at(-5.0, 5.0), at(5.0, -5.0)], stroke);
        }
        Icon::Home => {
            let house = [at(-6.0, 0.0), at(0.0, -6.0), at(6.0, 0.0), at(4.0, 0.0), at(4.0, 6.0), at(-4.0, 6.0), at(-4.0, 0.0), at(-6.0, 0.0)];
            painter.add(egui::Shape::line(house.to_vec(), stroke));
        }
        Icon::Flip => {
            painter.line_segment([at(-6.0, -2.5), at(6.0, -2.5)], stroke);
            painter.line_segment([at(-6.0, 2.5), at(6.0, 2.5)], stroke);
            painter.add(egui::Shape::line(vec![at(3.0, -5.5), at(6.0, -2.5), at(3.0, 0.5)], stroke));
            painter.add(egui::Shape::line(vec![at(-3.0, -0.5), at(-6.0, 2.5), at(-3.0, 5.5)], stroke));
        }
        Icon::Select => {
            let arrow = vec![at(-4.0, -7.0), at(-4.0, 5.0), at(-1.0, 2.0), at(2.0, 7.0), at(4.0, 6.0), at(1.0, 1.0), at(5.0, 1.0)];
            painter.add(egui::Shape::closed_line(arrow, stroke));
        }
        Icon::Line => {
            painter.line_segment([at(-6.0, 5.0), at(6.0, -5.0)], stroke);
            for p in [at(-6.0, 5.0), at(6.0, -5.0)] {
                painter.rect_filled(egui::Rect::from_center_size(p, vec2(3.0, 3.0)), 0.0, colour);
            }
        }
        Icon::Arc => {
            let pts: Vec<Pos2> = (0..=16)
                .map(|k| {
                    let a = (180.0 - 180.0 * k as f32 / 16.0).to_radians();
                    at(6.0 * a.cos(), 3.0 - 6.0 * a.sin())
                })
                .collect();
            painter.add(egui::Shape::line(pts, stroke));
            painter.circle_filled(at(0.0, 3.0), 1.3, colour);
        }
        Icon::Circle => {
            painter.circle_stroke(c, 6.0, stroke);
            painter.circle_filled(c, 1.3, colour);
        }
        Icon::Point => {
            painter.rect_filled(egui::Rect::from_center_size(c, vec2(4.0, 4.0)), 0.0, colour);
            painter.line_segment([at(-6.0, 0.0), at(-3.5, 0.0)], stroke);
            painter.line_segment([at(3.5, 0.0), at(6.0, 0.0)], stroke);
        }
        Icon::Dimension => {
            painter.line_segment([at(-6.0, -6.0), at(-6.0, 6.0)], stroke);
            painter.line_segment([at(6.0, -6.0), at(6.0, 6.0)], stroke);
            painter.line_segment([at(-6.0, 0.0), at(6.0, 0.0)], stroke);
            painter.add(egui::Shape::convex_polygon(vec![at(-6.0, 0.0), at(-3.0, -2.0), at(-3.0, 2.0)], colour, Stroke::NONE));
            painter.add(egui::Shape::convex_polygon(vec![at(6.0, 0.0), at(3.0, 2.0), at(3.0, -2.0)], colour, Stroke::NONE));
        }
        Icon::NewSketch => {
            painter.add(egui::Shape::closed_line(plane(), stroke));
            // A pencil down to the plane, its point at the lower left.
            let body = [at(-1.0, 3.0), at(-1.5, 1.0), at(5.0, -5.5), at(6.5, -4.0), at(0.0, 2.5), at(-1.0, 3.0)];
            painter.add(egui::Shape::line(body.to_vec(), stroke));
        }
        Icon::WorkPlane => {
            painter.add(egui::Shape::closed_line(plane(), stroke));
            painter.line_segment([at(0.0, 3.5), at(0.0, -6.0)], stroke);
            painter.add(egui::Shape::line(vec![at(-2.5, -3.5), at(0.0, -6.0), at(2.5, -3.5)], stroke));
        }
        Icon::LookAt => {
            let lid = |sign: f32| -> Vec<Pos2> {
                (0..=16)
                    .map(|k| {
                        let a = (180.0 * k as f32 / 16.0).to_radians();
                        at(-7.0 * a.cos(), sign * 4.5 * a.sin())
                    })
                    .collect()
            };
            painter.add(egui::Shape::line(lid(1.0), stroke));
            painter.add(egui::Shape::line(lid(-1.0), stroke));
            painter.circle_filled(c, 2.3, colour);
        }
        Icon::Measure => {
            // A ruler laid aslant, its ticks.
            painter.add(egui::Shape::closed_line(vec![at(-7.0, 3.0), at(3.0, -7.0), at(7.0, -3.0), at(-3.0, 7.0)], stroke));
            for t in [2.5, 5.0, 7.5] {
                painter.line_segment([at(-7.0 + t, 3.0 - t), at(-5.0 + t, 5.0 - t)], stroke);
            }
        }
    }
}
