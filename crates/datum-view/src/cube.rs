//! The navigation cube in the corner of the 3D view, as in Inventor: the model's directions as a
//! cube turned with the camera. A click on a face, an edge or a corner looks from there (26
//! views), a drag on it turns the view, the house above it goes back to the home view. The view
//! turns there smoothly rather than jumping, so the engineer keeps their bearings.

use datum_math::Vec3;
use egui::{Align2, Color32, FontId, Pos2, Rect, Sense, Stroke, Ui, pos2, vec2};

use crate::camera::Camera;

/// Side of the square the cube is drawn in, and its margin from the corner of the view.
const SIZE: f32 = 96.0;
const MARGIN: f32 = 10.0;
/// Where a face's middle ends and its edge band begins, from its centre (half side 1): the
/// corners and edges are narrow, the face itself the largest target.
const BAND: f64 = 0.6;
/// How long a turn to a view takes, in seconds.
const TURN: f64 = 0.35;

/// The faces: outward normal, the direction their label reads up and its text.
const FACES: [([f64; 3], [f64; 3], &str); 6] = [
    ([0.0, 0.0, 1.0], [0.0, 1.0, 0.0], "TOP"),
    ([0.0, 0.0, -1.0], [0.0, -1.0, 0.0], "BOTTOM"),
    ([0.0, -1.0, 0.0], [0.0, 0.0, 1.0], "FRONT"),
    ([0.0, 1.0, 0.0], [0.0, 0.0, 1.0], "BACK"),
    ([-1.0, 0.0, 0.0], [0.0, 0.0, 1.0], "LEFT"),
    ([1.0, 0.0, 0.0], [0.0, 0.0, 1.0], "RIGHT"),
];

/// What the cube asks of the view.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Ask {
    /// Look from this direction (components −1, 0 or 1).
    Look([i8; 3]),
    /// The home view: the default direction, the model fitted.
    Home,
    /// Turn by a drag of so many pixels.
    Orbit(f64, f64),
}

/// A turn of the camera under way: from and to yaw and pitch, and when it began.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Turn {
    from: [f64; 2],
    to: [f64; 2],
    start: f64,
    /// Fit the model once there (the home view).
    pub fit: bool,
}

impl Turn {
    /// A turn of `camera` to look from `d`, beginning at `now` (seconds).
    pub fn toward(camera: &Camera, d: Vec3, now: f64, fit: bool) -> Self {
        let mut to = camera.toward(d);
        // The shorter way round.
        let tau = std::f64::consts::TAU;
        to[0] = camera.yaw + (to[0] - camera.yaw) - tau * libm::round((to[0] - camera.yaw) / tau);
        Self { from: [camera.yaw, camera.pitch], to, start: now, fit }
    }

    /// Yaw and pitch at `now`, and whether the turn is over: eased in and out.
    pub fn at(&self, now: f64) -> ([f64; 2], bool) {
        let t = ((now - self.start) / TURN).clamp(0.0, 1.0);
        if t >= 1.0 {
            return (self.to, true);
        }
        let e = t * t * (3.0 - 2.0 * t);
        ([0, 1].map(|k| self.from[k] + (self.to[k] - self.from[k]) * e), false)
    }
}

/// A cell of a face: the direction it looks from and its corners on the cube.
struct Cell {
    look: [i8; 3],
    corners: [Vec3; 4],
}

/// The cells of the faces turned to the eye, each face cut 3 × 3: middle, edges, corners.
fn cells(towards_eye: Vec3) -> Vec<Cell> {
    let ranges = [(-1.0, -BAND), (-BAND, BAND), (BAND, 1.0)];
    let mut out = Vec::with_capacity(27);
    for (n, up, _) in FACES {
        let (n, b) = (Vec3::from(n), Vec3::from(up));
        if n.dot(towards_eye) <= 0.0 {
            continue;
        }
        let a = b.cross(n);
        for (i, (a0, a1)) in ranges.iter().enumerate() {
            for (j, (b0, b1)) in ranges.iter().enumerate() {
                let at = |x: f64, y: f64| n + a * x + b * y;
                let look = n + a * (i as f64 - 1.0) + b * (j as f64 - 1.0);
                out.push(Cell { look: [look.x, look.y, look.z].map(|v| v as i8), corners: [at(*a0, *b0), at(*a1, *b0), at(*a1, *b1), at(*a0, *b1)] });
            }
        }
    }
    out
}

/// Whether `p` is inside the convex polygon `q`, either way round.
fn inside(p: Pos2, q: &[Pos2]) -> bool {
    let side = |k: usize| {
        let (a, b) = (q[k], q[(k + 1) % q.len()]);
        (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x)
    };
    let s: Vec<f32> = (0..q.len()).map(side).collect();
    s.iter().all(|&v| v >= 0.0) || s.iter().all(|&v| v <= 0.0)
}

/// Where in `view` the cube is drawn, and the house above it: the top right corner.
fn place(view: Rect) -> (Rect, Rect) {
    let area = Rect::from_min_size(pos2(view.right() - SIZE - MARGIN, view.top() + MARGIN + 22.0), vec2(SIZE, SIZE));
    (area, Rect::from_center_size(pos2(area.left() + 8.0, view.top() + MARGIN + 8.0), vec2(18.0, 18.0)))
}

/// Where a point of the cube (half side 1) shows in `area`, as `camera` turns it: seen straight,
/// without perspective, so that its faces read as in a drawing.
fn projector(camera: &Camera, area: Rect) -> impl Fn(Vec3) -> Pos2 {
    let (right, up, _) = camera.axes();
    let (c, scale) = (area.center(), f64::from(SIZE) * 0.5 / 3f64.sqrt() * 0.95);
    move |p: Vec3| c + vec2((p.dot(right) * scale) as f32, (-p.dot(up) * scale) as f32)
}

/// Draws the cube in the top right corner of `view` as `camera` sees the model, and says what a
/// click or a drag on it asks.
pub fn show(ui: &mut Ui, view: Rect, camera: &Camera) -> Option<Ask> {
    let (area, home) = place(view);
    let r = ui.interact(area, ui.id().with("view cube"), Sense::click_and_drag());
    let h = ui.interact(home, ui.id().with("view cube home"), Sense::click()).on_hover_text("Home view");
    let forward = camera.axes().2;
    let screen = projector(camera, area);
    let cells = cells(-forward);
    let hover = r.hover_pos().and_then(|p| cells.iter().find(|cell| inside(p, &cell.corners.map(&screen))));

    let painter = ui.painter().with_clip_rect(view);
    let visuals = ui.visuals();
    // The view's background is light whatever the window's theme (`render::BACKGROUND`): the
    // cube is too, as in Inventor; the cell under the cursor takes the selection's colour.
    let (fill, line, text) = (Color32::from_rgb(214, 219, 227), Color32::from_rgb(130, 136, 148), Color32::from_rgb(70, 75, 86));
    for cell in &cells {
        let lit = hover.is_some_and(|h| h.look == cell.look);
        let colour = if lit { visuals.selection.bg_fill } else { fill };
        painter.add(egui::Shape::convex_polygon(cell.corners.map(&screen).to_vec(), colour, Stroke::NONE));
    }
    // The outline of each face, then its label, turned with it; hidden when seen edge-on.
    for (n, b, label) in FACES {
        let (n, b) = (Vec3::from(n), Vec3::from(b));
        let facing = n.dot(-forward);
        if facing <= 0.0 {
            continue;
        }
        let a = b.cross(n);
        let corners = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)].map(|(x, y)| screen(n + a * x + b * y));
        painter.add(egui::Shape::closed_line(corners.to_vec(), Stroke::new(1.0, line)));
        if facing > 0.35 {
            let along = screen(n + a) - screen(n);
            let galley = painter.layout_no_wrap(label.into(), FontId::proportional(10.0), text);
            // The text's top left corner, then turned about its middle.
            let corner = screen(n) - galley.size() * 0.5;
            let shape = egui::epaint::TextShape::new(corner, galley, text).with_angle_and_anchor(along.y.atan2(along.x), Align2::CENTER_CENTER);
            painter.add(shape);
        }
    }
    let home_colour = if h.hovered() { visuals.selection.bg_fill } else { text };
    crate::icons::paint(&painter, home.center(), crate::icons::Icon::Home, home_colour);

    if h.clicked() {
        return Some(Ask::Home);
    }
    if r.dragged() {
        let d = r.drag_delta();
        return Some(Ask::Orbit(f64::from(d.x), f64::from(d.y)));
    }
    if r.clicked() {
        return hover.map(|cell| Ask::Look(cell.look));
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 26 directions: 6 faces, 12 edges, 8 corners, each a cell of every face it touches.
    #[test]
    fn every_direction_has_its_cells() {
        let mut all = std::collections::BTreeMap::<[i8; 3], usize>::new();
        for (n, _, _) in FACES {
            for cell in cells(Vec3::from(n)) {
                *all.entry(cell.look).or_default() += 1;
            }
        }
        assert_eq!(all.len(), 26);
        for (look, n) in all {
            let touches = look.iter().filter(|v| **v != 0).count();
            assert_eq!(n, touches, "{look:?}");
        }
    }

    /// Facing the front, the cube shows the front alone; its middle looks from the front.
    #[test]
    fn the_front_seen_from_the_front() {
        let c = cells(Vec3::new(0.0, -1.0, 0.0));
        assert_eq!(c.len(), 9);
        assert!(c.iter().any(|cell| cell.look == [0, -1, 0]));
    }

    /// Clicks in a window: the middle of the top looks from above, an edge's band from both its
    /// faces, the house goes home.
    #[test]
    fn clicks_ask_for_views() {
        let camera = Camera::default();
        let view = Rect::from_min_size(pos2(0.0, 0.0), vec2(400.0, 300.0));
        let mut h = egui_kittest::Harness::builder().with_size([400.0, 300.0]).build_ui_state(
            move |ui, asked: &mut Vec<Ask>| {
                if let Some(a) = show(ui, view, &camera) {
                    asked.push(a);
                }
            },
            Vec::new(),
        );
        let (area, home) = place(view);
        let screen = projector(&camera, area);
        let click = |h: &mut egui_kittest::Harness<Vec<Ask>>, at: Pos2| {
            h.hover_at(at);
            for pressed in [true, false] {
                h.event(egui::Event::PointerButton { pos: at, button: egui::PointerButton::Primary, pressed, modifiers: egui::Modifiers::NONE });
                h.step();
            }
            h.step();
        };
        h.run();
        click(&mut h, screen(Vec3::new(0.0, 0.0, 1.0)));
        // On the front's band along the top: from the front and above.
        click(&mut h, screen(Vec3::new(0.0, -1.0, 0.8)));
        click(&mut h, home.center());
        assert_eq!(h.state(), &vec![Ask::Look([0, 0, 1]), Ask::Look([0, -1, 1]), Ask::Home]);
    }

    /// A turn from the default view to the top goes the short way and ends there exactly.
    #[test]
    fn a_turn_ends_where_it_was_asked() {
        let camera = Camera::default();
        let turn = Turn::toward(&camera, Vec3::new(0.0, 0.0, 1.0), 10.0, false);
        let (start, _) = turn.at(10.0);
        assert_eq!(start, [camera.yaw, camera.pitch]);
        let (end, done) = turn.at(10.0 + 2.0 * TURN);
        assert!(done);
        assert_eq!(end, [-std::f64::consts::FRAC_PI_2, std::f64::consts::FRAC_PI_2]);
        // From yaw 3 to the direction at yaw −3: across ±π, a little over 0.28 rad, not 6 back.
        let behind = Camera { yaw: 3.0, ..camera };
        let (s, c) = libm::sincos(-3.0);
        let turn = Turn::toward(&behind, Vec3::new(c, s, 0.0), 0.0, false);
        let turned = turn.at(TURN).0[0] - 3.0;
        assert!((turned - (std::f64::consts::TAU - 6.0)).abs() < 1e-12, "{turned}");
    }
}
