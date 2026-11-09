//! The view turned while projecting into a sketch (D248): parts lying over one another hide the
//! edges under them, so the projection tool can look at the model in 3D. A left drag turns it, a
//! click takes what is under the cursor there; when the tool ends, the view is the sketch's again.

use datum_brep::TopoId;
use datum_math::Vec3;

use crate::camera::{Camera, SketchCamera, clip};
use crate::draw::{Projectable, SNAP_CURVE, SNAP_POINT};
use crate::scene::Scene;
use crate::sketch::SketchEdit;

const SKETCH: egui::Color32 = egui::Color32::from_rgb(214, 96, 32);
const CHOSEN: egui::Color32 = egui::Color32::from_rgb(235, 110, 20);

/// The face under the cursor: of which feature's body, and how far along the ray.
#[derive(Clone, Debug, PartialEq)]
pub struct Under {
    pub body: String,
    pub face: TopoId,
    pub distance: f64,
}

/// The view of a sketch turned away from its plane.
#[derive(Clone, Debug)]
pub struct Turned {
    pub camera: Camera,
    pub anchor: Vec3,
    pub radius: f64,
    /// The view's width and height, pixels.
    pub size: [f64; 2],
    pub under: Option<Under>,
}

impl Turned {
    /// The 3D view showing just what the sketch's view does: looking straight at its plane, as
    /// tall, about the same middle; `model`'s lens.
    pub fn new(sketch: &SketchCamera, model: &Camera, anchor: Vec3, radius: f64) -> Self {
        let [o, x, y, _] = sketch.frame;
        let target = o + x * sketch.center[0] + y * sketch.center[1];
        let distance = sketch.half_height / libm::tan(model.fov * 0.5);
        let camera = Camera { target, distance, right: x, up: y, ..*model };
        Self { camera, anchor, radius, size: [1.0, 1.0], under: None }
    }

    fn aspect(&self) -> f64 {
        self.size[0] / self.size[1]
    }

    pub(crate) fn view_projection(&self) -> [f32; 16] {
        self.camera.matrices(self.anchor, self.aspect(), self.radius).0
    }

    /// Point `p` as a pixel offset from the middle of the view, if in front of the eye.
    pub(crate) fn offset(&self, m: &[f32; 16], p: Vec3) -> Option<(f64, f64)> {
        let c = clip(m, p - self.anchor).map(f64::from);
        (c[3] > 0.0).then(|| (c[0] / c[3] * 0.5 * self.size[0], -c[1] / c[3] * 0.5 * self.size[1]))
    }

    /// The ray through pixel offset `(dx, dy)`.
    fn ray(&self, dx: f64, dy: f64) -> (Vec3, Vec3) {
        self.camera.ray(dx / (0.5 * self.size[0]), -dy / (0.5 * self.size[1]), self.aspect(), self.anchor, self.radius)
    }

    /// Takes the view's size and what lies under the cursor at `(dx, dy)`, of `bodies`, the names
    /// of `scene`'s bodies.
    pub fn look_at(&mut self, scene: &Scene, bodies: &[String], size: [f64; 2], dx: f64, dy: f64) {
        self.size = size;
        let (origin, dir) = self.ray(dx, dy);
        self.under = scene.pick(origin, dir).and_then(|h| Some(Under { body: bodies.get(h.body)?.clone(), face: h.face, distance: h.distance }));
    }

    /// Which of `left` (candidates by index) the cursor at `(dx, dy)` is on: a point before a curve,
    /// behind nothing; away from them, the face it is over. A pick on the screen, not a geometric
    /// decision: "behind" is within what a few pixels are worth.
    pub fn spot<'a>(&self, left: impl Iterator<Item = (usize, &'a Projectable)>, dx: f64, dy: f64) -> Option<usize> {
        let m = self.view_projection();
        let (origin, dir) = self.ray(dx, dy);
        let spare = SNAP_CURVE * 2.0 * self.camera.half_height() / self.size[1];
        let seen = |q: Vec3| self.under.as_ref().is_none_or(|u| (q - origin).dot(dir) <= u.distance + spare);
        let (mut point, mut curve, mut face) = (None::<(f64, usize)>, None::<(f64, usize)>, None);
        let closer = |best: &mut Option<(f64, usize)>, d: f64, i: usize| {
            if best.is_none_or(|b| d < b.0) {
                *best = Some((d, i));
            }
        };
        for (i, c) in left {
            if c.entry.0 == "face" {
                if self.under.as_ref().is_some_and(|u| u.body == c.from && format!("{:?}", u.face) == c.entry.1) {
                    face = Some(i);
                }
                continue;
            }
            for line in &c.world {
                if let [p] = line[..] {
                    if let Some((x, y)) = self.offset(&m, p)
                        && libm::hypot(x - dx, y - dy) <= SNAP_POINT
                        && seen(p)
                    {
                        closer(&mut point, libm::hypot(x - dx, y - dy), i);
                    }
                    continue;
                }
                for w in line.windows(2) {
                    let (Some(a), Some(b)) = (self.offset(&m, w[0]), self.offset(&m, w[1])) else { continue };
                    let (ex, ey) = (b.0 - a.0, b.1 - a.1);
                    let t = (((dx - a.0) * ex + (dy - a.1) * ey) / (ex * ex + ey * ey).max(f64::MIN_POSITIVE)).clamp(0.0, 1.0);
                    let d = libm::hypot(a.0 + ex * t - dx, a.1 + ey * t - dy);
                    if d <= SNAP_CURVE && seen(w[0] + (w[1] - w[0]) * t) {
                        closer(&mut curve, d, i);
                    }
                }
            }
        }
        point.or(curve).map(|b| b.1).or(face)
    }

    /// The sketch's curves in place, what the cursor is on, and how to go back.
    pub fn paint(&self, sk: &SketchEdit, painter: &egui::Painter, rect: egui::Rect, hint: &str) {
        let m = self.view_projection();
        let screen = |p: Vec3| self.offset(&m, p).map(|(x, y)| rect.center() + egui::vec2(x as f32, y as f32));
        if let Ok(p) = &sk.placed {
            let [o, x, y, _] = p.frame;
            for n in p.lines.keys().chain(p.arcs.keys()).chain(p.circles.keys()).chain(p.ellipses.keys()) {
                let pts: Vec<egui::Pos2> =
                    SketchEdit::curve_points(p, n).into_iter().flatten().filter_map(|q| screen(o + x * q[0] + y * q[1])).collect();
                painter.add(egui::Shape::line(pts, egui::Stroke::new(1.5, SKETCH)));
            }
        }
        if let Some(c) = sk.draft.chosen() {
            for line in &c.world {
                if let [p] = line[..] {
                    if let Some(at) = screen(p) {
                        painter.circle_filled(at, 4.0, CHOSEN);
                    }
                } else {
                    painter.add(egui::Shape::line(line.iter().filter_map(|&p| screen(p)).collect(), egui::Stroke::new(3.0, CHOSEN)));
                }
            }
        }
        painter.text(
            rect.center_bottom() - egui::vec2(0.0, 12.0),
            egui::Align2::CENTER_BOTTOM,
            hint,
            egui::FontId::proportional(13.0),
            egui::Color32::from_rgb(60, 65, 75),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::draw::Flat;

    fn edge(from: &str, a: Vec3, b: Vec3) -> Projectable {
        Projectable { from: from.into(), entry: ("id", from.into()), shape: Flat::Line([0.0, 0.0], [1.0, 0.0]), height: 0.0, world: vec![vec![a, b]] }
    }

    /// Looking from above at a sketch on XY, then turned a little.
    fn turned() -> Turned {
        let z = Vec3::new(0.0, 0.0, 1.0);
        let (x, y) = (Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0));
        let sketch = SketchCamera { frame: [Vec3::ZERO, x, y, z], center: [0.0, 0.0], half_height: 100.0 };
        let mut t = Turned::new(&sketch, &Camera::default(), Vec3::ZERO, 200.0);
        t.size = [800.0, 600.0];
        t.camera.orbit(0.0, -100.0);
        t
    }

    #[test]
    fn it_starts_where_the_sketch_looks() {
        let x = Vec3::new(1.0, 0.0, 0.0);
        let y = Vec3::new(0.0, 1.0, 0.0);
        let sketch = SketchCamera { frame: [Vec3::ZERO, x, y, Vec3::new(0.0, 0.0, 1.0)], center: [10.0, 20.0], half_height: 50.0 };
        let mut t = Turned::new(&sketch, &Camera::default(), Vec3::ZERO, 200.0);
        t.size = [800.0, 600.0];
        let m = t.view_projection();
        let (dx, dy) = t.offset(&m, Vec3::new(10.0, 20.0, 0.0)).unwrap();
        assert!(dx.abs() < 1e-3 && dy.abs() < 1e-3);
        let (_, up) = t.offset(&m, Vec3::new(10.0, 70.0, 0.0)).unwrap();
        assert!((up + 300.0).abs() < 1e-3, "half the height of the view: {up}");
    }

    /// Of two edges one above the other, each is taken where it shows on the screen.
    #[test]
    fn an_edge_under_another_is_taken_where_it_shows() {
        let t = turned();
        let cands = [
            edge("top", Vec3::new(-50.0, 0.0, 20.0), Vec3::new(50.0, 0.0, 20.0)),
            edge("under", Vec3::new(-50.0, 0.0, -20.0), Vec3::new(50.0, 0.0, -20.0)),
        ];
        let m = t.view_projection();
        for (k, c) in cands.iter().enumerate() {
            let (dx, dy) = t.offset(&m, c.world[0][0] + (c.world[0][1] - c.world[0][0]) * 0.5).unwrap();
            assert_eq!(t.spot(cands.iter().enumerate(), dx, dy), Some(k));
        }
    }

    /// An edge behind a face under the cursor is not taken; in front of it, it is.
    #[test]
    fn an_edge_behind_a_face_is_not_taken() {
        let mut t = turned();
        let c = edge("e", Vec3::new(-50.0, 0.0, 0.0), Vec3::new(50.0, 0.0, 0.0));
        let m = t.view_projection();
        let (dx, dy) = t.offset(&m, Vec3::ZERO).unwrap();
        let (origin, dir) = t.ray(dx, dy);
        let depth = (Vec3::ZERO - origin).dot(dir);
        let under = |distance| Some(Under { body: "b".into(), face: TopoId(1), distance });
        t.under = under(depth - 50.0);
        assert_eq!(t.spot([(0, &c)].into_iter(), dx, dy), None);
        t.under = under(depth + 50.0);
        assert_eq!(t.spot([(0, &c)].into_iter(), dx, dy), Some(0));
        t.under = under(depth);
        assert_eq!(t.spot([(0, &c)].into_iter(), dx, dy), Some(0), "an edge of the very face");
    }

    /// Off the edges, the face under the cursor.
    #[test]
    fn away_from_the_edges_the_face_under_the_cursor() {
        let mut t = turned();
        t.under = Some(Under { body: "b".into(), face: TopoId(7), distance: 1.0 });
        let face =
            Projectable { from: "b".into(), entry: ("face", format!("{:?}", TopoId(7))), shape: Flat::Point([0.0, 0.0]), height: 0.0, world: vec![] };
        let other = Projectable { from: "c".into(), ..face.clone() };
        assert_eq!(t.spot([(0, &other), (1, &face)].into_iter(), 300.0, 200.0), Some(1));
    }
}
