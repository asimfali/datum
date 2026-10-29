//! The navigation cube in the corner of the 3D view, as in Inventor: the model's directions as a
//! cube turned with the camera. A click on a face, an edge or a corner looks from there (26
//! views), a drag on it turns the view, the house above it goes back to the home view. The view
//! turns there smoothly rather than jumping, so the engineer keeps their bearings.

use datum_math::Vec3;
use egui::{Color32, FontId, Pos2, Rect, Sense, Stroke, Ui, pos2, vec2};

use crate::actions::Act;
use crate::camera::Camera;
use crate::i18n::I18n;
use crate::icons::Icon;

/// Side of the square the cube is drawn in, and its margin from the corner of the view.
const SIZE: f32 = 120.0;
const MARGIN: f32 = 10.0;
/// Where a face's middle ends and its edge band begins, from its centre (half side 1): the
/// corners and edges are narrow, the face itself the largest target.
const BAND: f64 = 0.6;
/// A label's height and its widest on a face, in the face's units (half side 1).
const LABEL_HEIGHT: f32 = 0.4;
const LABEL_WIDTH: f32 = 1.5;
/// A face turned from the eye further than this (the cosine) is too slanted to read: no label.
const EDGE_ON: f64 = 0.2;
/// How long a turn to a view takes, in seconds.
const TURN: f64 = 0.35;

/// The directions along the axes.
const AXES: [[f64; 3]; 6] = [[1.0, 0.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, -1.0]];

/// The faces: outward normal and the direction their label reads up.
const FACES: [([f64; 3], [f64; 3]); 6] = [
    ([0.0, 0.0, 1.0], [0.0, 1.0, 0.0]),
    ([0.0, 0.0, -1.0], [0.0, -1.0, 0.0]),
    ([0.0, -1.0, 0.0], [0.0, 0.0, 1.0]),
    ([0.0, 1.0, 0.0], [0.0, 0.0, 1.0]),
    ([-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
    ([1.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
];

/// The label of face `i` of [`FACES`].
fn face_label(i18n: &I18n, i: usize) -> &str {
    match i {
        0 => i18n.t("cube-top", "TOP"),
        1 => i18n.t("cube-bottom", "BOTTOM"),
        2 => i18n.t("cube-front", "FRONT"),
        3 => i18n.t("cube-back", "BACK"),
        4 => i18n.t("cube-left", "LEFT"),
        _ => i18n.t("cube-right", "RIGHT"),
    }
}

/// What the cube asks of the view.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Ask {
    /// Look from this direction (components −1, 0 or 1).
    Look([i8; 3]),
    /// The home view: the default direction, the model fitted.
    Home,
    /// Turn by a drag of so many pixels.
    Orbit(f64, f64),
    /// Turn the picture a quarter about the line of sight: 1 counterclockwise, −1 clockwise.
    Roll(i8),
}

/// A turn of the camera under way: from and to its right and up, and when it began.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Turn {
    from: [Vec3; 2],
    to: [Vec3; 2],
    start: f64,
    /// Fit the model once there (the home view).
    pub fit: bool,
}

impl Turn {
    /// A turn of `camera` to look from `d`, beginning at `now` (seconds).
    pub fn toward(camera: &Camera, d: Vec3, now: f64, fit: bool) -> Self {
        Self::to(camera, camera.toward(d), now, fit)
    }

    /// A turn of `camera` to right and up `to`.
    pub fn to(camera: &Camera, to: [Vec3; 2], now: f64, fit: bool) -> Self {
        Self { from: [camera.right, camera.up], to, start: now, fit }
    }

    /// Right and up at `now`, and whether the turn is over: the shortest way, eased in and out.
    pub fn at(&self, now: f64) -> ([Vec3; 2], bool) {
        let t = ((now - self.start) / TURN).clamp(0.0, 1.0);
        if t >= 1.0 {
            return (self.to, true);
        }
        if t <= 0.0 {
            return (self.from, false);
        }
        (crate::camera::between(self.from, self.to, t * t * (3.0 - 2.0 * t)), false)
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
    for (n, up) in FACES {
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
    let (c, scale) = (area.center(), f64::from(unit()));
    move |p: Vec3| c + vec2((p.dot(right) * scale) as f32, (-p.dot(up) * scale) as f32)
}

/// Points per unit of the cube (half side 1): it fits its square however it turns.
fn unit() -> f32 {
    SIZE * 0.5 / 3f32.sqrt() * 0.95
}

/// Where the middle of the face of normal `n` shows in `view` as `camera` turns the cube.
#[cfg(test)]
pub fn face_middle(camera: &Camera, view: Rect, n: Vec3) -> Pos2 {
    projector(camera, place(view).0)(n)
}

/// Paints `label` on the face of normal `n` as if printed on it: its glyphs laid on the face, so
/// that they slant and shorten with it. Of the four ways it can lie there (along either of the
/// face's `axes` or against it), the one that reads most nearly left to right and upright: never
/// upside down.
fn label_face(painter: &egui::Painter, screen: &impl Fn(Vec3) -> Pos2, n: Vec3, axes: [Vec3; 2], label: &str, colour: Color32) {
    let centre = screen(n);
    let on_screen = |d: Vec3| screen(n + d) - centre;
    // How well text with this right direction reads: rightwards, and its up upwards.
    let reads = |right: Vec3| {
        let (r, up) = (on_screen(right), on_screen(n.cross(right)));
        r.x / r.length().max(1e-6) - up.y / up.length().max(1e-6)
    };
    let [a, b] = axes;
    let right = [a, b, -a, -b].into_iter().max_by(|p, q| reads(*p).total_cmp(&reads(*q))).unwrap_or(a);
    // Glyphs made at the size the label has on a face seen straight, so that they are laid on
    // it one to one and stay sharp: as high as asked, unless that is too wide.
    let probe = painter.layout_no_wrap(label.into(), FontId::proportional(10.0), colour).size();
    let points = 10.0 * (LABEL_HEIGHT * unit() / probe.y).min(LABEL_WIDTH * unit() / probe.x);
    let galley = painter.layout_no_wrap(label.into(), FontId::proportional(points), colour);
    let (r, down) = (on_screen(right) / unit(), on_screen(n.cross(right)) / -unit());
    let middle = galley.rect.center();
    // A row's texture coordinates are in pixels of the font atlas, as the tessellator takes them.
    let atlas = painter.ctx().fonts(|f| f.font_image_size()).map(|n| 1.0 / n as f32);
    let mut mesh = egui::Mesh::default();
    for row in &galley.rows {
        let base = mesh.vertices.len() as u32;
        for v in &row.visuals.mesh.vertices {
            let p = v.pos + row.pos.to_vec2() - middle;
            let color = if v.color == Color32::PLACEHOLDER { colour } else { v.color };
            mesh.vertices.push(egui::epaint::Vertex { pos: centre + r * p.x + down * p.y, uv: pos2(v.uv.x * atlas[0], v.uv.y * atlas[1]), color });
        }
        mesh.indices.extend(row.visuals.mesh.indices.iter().map(|i| i + base));
    }
    painter.add(egui::Shape::mesh(mesh));
}

/// Draws the cube in the top right corner of `view` as `camera` sees the model, and says what a
/// click or a drag on it asks.
pub fn show(ui: &mut Ui, view: Rect, camera: &Camera, i18n: &I18n) -> Option<Ask> {
    let (area, home) = place(view);
    let r = ui.interact(area, ui.id().with("view cube"), Sense::click_and_drag());
    let h = ui.interact(home, ui.id().with("view cube home"), Sense::click()).on_hover_text(i18n.label(Act::Home));
    let forward = camera.axes().2;
    let screen = projector(camera, area);
    let cells = cells(-forward);
    // Looking straight at a face, as in Inventor: a triangle beyond each of its edges looks from
    // the face across it, and two arrows above turn the picture a quarter.
    let face_on = FACES.iter().map(|f| Vec3::from(f.0)).find(|n| n.dot(-forward) > 1.0 - 1e-9);
    let arrows: Vec<([i8; 3], [Pos2; 3])> = face_on.map_or_else(Vec::new, |n| {
        let c = screen(Vec3::ZERO);
        AXES.iter()
            .map(|&d| Vec3::from(d))
            .filter(|d| d.dot(n) == 0.0)
            .map(|d| {
                let along = (screen(d) - c).normalized();
                let across = egui::vec2(-along.y, along.x) * 7.0;
                let base = c + along * (unit() + 6.0);
                ([d.x, d.y, d.z].map(|v| v as i8), [c + along * (unit() + 17.0), base + across, base - across])
            })
            .collect()
    });
    let rolls = face_on.map(|_| {
        [(1, Icon::Undo, i18n.t("cube-roll-left", "Turn a quarter left")), (-1, Icon::Redo, i18n.t("cube-roll-right", "Turn a quarter right"))].map(
            |(dir, icon, hint)| {
                let at = Rect::from_center_size(pos2(area.right() - 30.0 + 18.0 * f32::from(u8::from(dir < 0)), area.top() + 8.0), vec2(16.0, 16.0));
                (dir, icon, ui.interact(at, ui.id().with(("view cube roll", dir)), Sense::click()).on_hover_text(hint))
            },
        )
    });
    let pointer = r.hover_pos();
    let arrow = pointer.and_then(|p| arrows.iter().find(|(_, t)| inside(p, t)));
    let hover = pointer.filter(|_| arrow.is_none()).and_then(|p| cells.iter().find(|cell| inside(p, &cell.corners.map(&screen))));

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
    for (i, (n, b)) in FACES.into_iter().enumerate() {
        let (n, b) = (Vec3::from(n), Vec3::from(b));
        let facing = n.dot(-forward);
        if facing <= 0.0 {
            continue;
        }
        let a = b.cross(n);
        let corners = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)].map(|(x, y)| screen(n + a * x + b * y));
        painter.add(egui::Shape::closed_line(corners.to_vec(), Stroke::new(1.0, line)));
        if facing > EDGE_ON {
            label_face(&painter, &screen, n, [a, b], face_label(i18n, i), text);
        }
    }
    for (look, t) in &arrows {
        let lit = arrow.is_some_and(|a| a.0 == *look);
        painter.add(egui::Shape::convex_polygon(t.to_vec(), if lit { visuals.selection.bg_fill } else { fill }, Stroke::new(1.0, line)));
    }
    let lit = |r: &egui::Response| if r.hovered() { visuals.selection.bg_fill } else { text };
    for (_, icon, roll) in rolls.iter().flatten() {
        crate::icons::paint(&painter, roll.rect.center(), *icon, lit(roll));
    }
    crate::icons::paint(&painter, home.center(), Icon::Home, lit(&h));

    if h.clicked() {
        return Some(Ask::Home);
    }
    if let Some((dir, ..)) = rolls.iter().flatten().find(|(_, _, roll)| roll.clicked()) {
        return Some(Ask::Roll(*dir));
    }
    // A click first: while the button is down egui calls the cube dragged already, and a drag
    // that has not moved is no turn of the view.
    if r.clicked() {
        return arrow.map(|a| Ask::Look(a.0)).or(hover.map(|cell| Ask::Look(cell.look)));
    }
    let d = r.drag_delta();
    (r.dragged() && d != egui::Vec2::ZERO).then(|| Ask::Orbit(f64::from(d.x), f64::from(d.y)))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 26 directions: 6 faces, 12 edges, 8 corners, each a cell of every face it touches.
    #[test]
    fn every_direction_has_its_cells() {
        let mut all = std::collections::BTreeMap::<[i8; 3], usize>::new();
        for (n, _) in FACES {
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
                if let Some(a) = show(ui, view, &camera, &I18n::default()) {
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

    /// Facing the front, as in Inventor: the triangle above the cube looks from the top, the
    /// left arrow turns the picture a quarter counterclockwise.
    #[test]
    fn facing_a_face_its_neighbours_are_a_click_away() {
        let camera = Camera::view("front").unwrap();
        let view = Rect::from_min_size(pos2(0.0, 0.0), vec2(400.0, 300.0));
        let mut h = egui_kittest::Harness::builder().with_size([400.0, 300.0]).build_ui_state(
            move |ui, asked: &mut Vec<Ask>| {
                if let Some(a) = show(ui, view, &camera, &I18n::default()) {
                    asked.push(a);
                }
            },
            Vec::new(),
        );
        let area = place(view).0;
        let click = |h: &mut egui_kittest::Harness<Vec<Ask>>, at: Pos2| {
            h.hover_at(at);
            h.step();
            for pressed in [true, false] {
                h.event(egui::Event::PointerButton { pos: at, button: egui::PointerButton::Primary, pressed, modifiers: egui::Modifiers::NONE });
                h.step();
            }
        };
        h.run();
        click(&mut h, area.center() - vec2(0.0, unit() + 11.0));
        click(&mut h, pos2(area.right() - 30.0, area.top() + 8.0));
        assert_eq!(h.state(), &vec![Ask::Look([0, 0, 1]), Ask::Roll(1)]);
    }

    /// A turn from the default view to the top ends there exactly, the front at the bottom; one
    /// from the front's top edge to the back's goes the short way, about the vertical.
    #[test]
    fn a_turn_ends_where_it_was_asked() {
        let camera = Camera::default();
        let turn = Turn::toward(&camera, Vec3::new(0.0, 0.0, 1.0), 10.0, false);
        assert_eq!(turn.at(10.0).0, [camera.right, camera.up]);
        let (end, done) = turn.at(10.0 + 2.0 * TURN);
        assert!(done);
        assert_eq!(end, [Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0)]);
        // Halfway, the eye is between: above the iso direction's way to the top.
        let ([right, up], _) = turn.at(10.0 + TURN * 0.5);
        let back = right.cross(up);
        assert!(back.z > camera.back().z && back.z < 1.0, "{back:?}");
    }
}
