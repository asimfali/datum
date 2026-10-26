//! The flat pattern pane (D161): a blank drawn flat, as the turret press cuts it, moved and zoomed
//! by the mouse as a sketch is.

use datum_ops::{Flat, FlatPiece};

/// What the pane looks at: the blank's point in the middle, pixels per millimetre, and the blank
/// it was fitted to (fitted again when another is shown).
#[derive(Default)]
pub struct FlatView {
    pub center: [f64; 2],
    pub scale: f64,
    pub fitted: Option<[u8; 32]>,
    /// The flat shown, by its feature, when there are several.
    pub chosen: Option<String>,
}

const LINE: egui::Color32 = egui::Color32::from_rgb(24, 26, 30);
const PAPER: egui::Color32 = egui::Color32::from_rgb(250, 250, 248);

impl FlatView {
    /// The whole blank in `rect`, with a margin.
    fn fit(&mut self, flat: &Flat, rect: egui::Rect) {
        self.center = [flat.size.x * 0.5, flat.size.y * 0.5];
        let (w, h) = (f64::from(rect.width()), f64::from(rect.height()));
        let k = (w / flat.size.x.max(1e-9)).min(h / flat.size.y.max(1e-9));
        self.scale = if k.is_finite() && k > 0.0 { k * 0.9 } else { 1.0 };
        self.fitted = Some(flat.hash);
    }

    /// Draws `flat` filling `ui`, and takes the mouse: a right or middle drag moves it, the wheel
    /// zooms about the cursor, a double click fits it.
    pub fn show(&mut self, ui: &mut egui::Ui, flat: &Flat) {
        let (rect, r) = ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());
        if self.fitted != Some(flat.hash) || r.double_clicked() {
            self.fit(flat, rect);
        }
        if r.dragged_by(egui::PointerButton::Secondary) || r.dragged_by(egui::PointerButton::Middle) {
            let d = r.drag_delta();
            self.center[0] -= f64::from(d.x) / self.scale;
            self.center[1] += f64::from(d.y) / self.scale;
        }
        let scroll = if r.hovered() { ui.input(|i| i.smooth_scroll_delta.y) } else { 0.0 };
        if scroll != 0.0
            && let Some(p) = r.hover_pos()
        {
            let k = libm::exp(f64::from(scroll) * 0.002);
            let (dx, dy) = (f64::from(p.x - rect.center().x), f64::from(p.y - rect.center().y));
            // The point under the cursor stays there.
            let at = [self.center[0] + dx / self.scale, self.center[1] - dy / self.scale];
            self.scale *= k;
            self.center = [at[0] - dx / self.scale, at[1] + dy / self.scale];
        }
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 0.0, PAPER);
        let screen = |q: [f64; 2]| {
            egui::pos2(
                rect.center().x + ((q[0] - self.center[0]) * self.scale) as f32,
                rect.center().y - ((q[1] - self.center[1]) * self.scale) as f32,
            )
        };
        let stroke = egui::Stroke::new(1.2, LINE);
        for piece in &flat.pieces {
            match *piece {
                FlatPiece::Line(a, b) => {
                    painter.line_segment([screen([a.x, a.y]), screen([b.x, b.y])], stroke);
                }
                FlatPiece::Arc { center, radius, start, end } => {
                    let sweep = (end - start).rem_euclid(std::f64::consts::TAU);
                    let sweep = if sweep == 0.0 { std::f64::consts::TAU } else { sweep };
                    let n = (sweep / std::f64::consts::TAU * 64.0).ceil().max(2.0) as usize;
                    let pts = (0..=n)
                        .map(|k| {
                            let (s, c) = libm::sincos(start + sweep * k as f64 / n as f64);
                            screen([center.x + radius * c, center.y + radius * s])
                        })
                        .collect();
                    painter.add(egui::Shape::line(pts, stroke));
                }
                FlatPiece::Circle { center, radius } => {
                    painter.circle_stroke(screen([center.x, center.y]), (radius * self.scale) as f32, stroke);
                }
            }
        }
        painter.text(
            rect.left_bottom() + egui::vec2(8.0, -8.0),
            egui::Align2::LEFT_BOTTOM,
            format!("{:.1} × {:.1}", flat.size.x, flat.size.y),
            egui::FontId::proportional(13.0),
            LINE,
        );
    }
}
