//! Planes in the view: the model's origin planes, XY, XZ and YZ, as Inventor shows them when a
//! plane is to be picked, and its work planes (D150), always. Each is a square as large as the
//! model; a sketch starts on one, the view looks straight at one, a work plane is offset from one.

use datum_math::Vec3;
use datum_ops::{OriginPlane, SketchPlane};
use egui::{Color32, Painter, Pos2, Stroke};

/// The plane a sketch starts on when no other is picked: XY, as in Inventor.
pub const FIRST: usize = 0;

/// A plane drawn as a square: its name, middle and the unit directions of its sides.
#[derive(Clone, Debug, PartialEq)]
pub struct Square {
    pub name: String,
    pub middle: Vec3,
    pub x: Vec3,
    pub y: Vec3,
}

impl Square {
    /// The unit normal, the way the plane faces.
    pub fn normal(&self) -> Vec3 {
        self.x.cross(self.y)
    }

    /// The unit normal turned to a viewer looking along `dir`.
    pub fn normal_towards(&self, dir: Vec3) -> Vec3 {
        let n = self.normal();
        if n.dot(dir) > 0.0 { -n } else { n }
    }
}

fn unit(v: Vec3) -> Vec3 {
    v * (1.0 / v.dot(v).sqrt())
}

/// The square of `plane` named `name`, its middle where `near` projects onto it.
pub fn square(name: &str, plane: &SketchPlane, near: Vec3) -> Square {
    let n = unit(plane.normal);
    let x = unit(plane.x_axis - n * plane.x_axis.dot(n));
    Square { name: name.into(), middle: near - n * (near - plane.origin).dot(n), x, y: n.cross(x) }
}

/// The origin planes, through the origin.
pub fn origin_squares() -> Vec<Square> {
    OriginPlane::ALL.iter().map(|o| square(o.name(), &o.plane(), Vec3::ZERO)).collect()
}

/// Half the side of the squares: past the model on every side, or 50 (mm) with no model.
pub fn half(bounds: Option<(Vec3, Vec3)>) -> f64 {
    let reach = bounds.map_or(0.0, |(lo, hi)| [lo.x, lo.y, lo.z, hi.x, hi.y, hi.z].into_iter().fold(0.0, |m: f64, v| m.max(v.abs())));
    if reach > 0.0 { reach * 1.15 } else { 50.0 }
}

/// The fields of a sketch feature on origin plane `k`: none for XY, where a sketch lies
/// without a plane, else the plane.
pub fn sketch_fields(k: usize) -> serde_json::Value {
    match OriginPlane::ALL[k] {
        OriginPlane::Xy => serde_json::json!({}),
        o => serde_json::json!({"plane": o.plane()}),
    }
}

/// The nearest of `squares` a ray from `origin` along `dir` passes through within `half` of its
/// middle, and how far along the ray. A pick on the screen.
pub fn hit(squares: &[Square], origin: Vec3, dir: Vec3, half: f64) -> Option<(f64, usize)> {
    squares
        .iter()
        .enumerate()
        .filter_map(|(k, s)| {
            let n = s.normal();
            let across = dir.dot(n);
            if across == 0.0 {
                return None;
            }
            let t = (s.middle - origin).dot(n) / across;
            let p = origin + dir * t - s.middle;
            (t > 0.0 && p.dot(s.x).abs() <= half && p.dot(s.y).abs() <= half).then_some((t, k))
        })
        .min_by(|a, b| a.0.total_cmp(&b.0))
}

/// Draws `squares`, see-through, `lit` the one a click or Enter would take; `screen` places a
/// point of the model on the screen (none behind the eye). Work planes are bluish, origin
/// planes yellowish, as in Inventor.
pub fn paint(painter: &Painter, screen: impl Fn(Vec3) -> Option<Pos2>, squares: &[Square], half: f64, lit: Option<usize>, work: bool) {
    for (k, s) in squares.iter().enumerate() {
        let corners: Option<Vec<Pos2>> =
            [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)].iter().map(|&(a, b)| screen(s.middle + (s.x * a + s.y * b) * half)).collect();
        let Some(corners) = corners else { continue };
        let (fill, line) = match (lit == Some(k), work) {
            (true, _) => (Color32::from_rgba_unmultiplied(235, 110, 20, 70), Color32::from_rgb(220, 100, 20)),
            (false, true) => (Color32::from_rgba_unmultiplied(90, 140, 220, 30), Color32::from_rgb(80, 120, 200)),
            (false, false) => (Color32::from_rgba_unmultiplied(240, 190, 60, 30), Color32::from_rgb(200, 160, 60)),
        };
        let label = corners[3];
        painter.add(egui::Shape::convex_polygon(corners, fill, Stroke::new(if lit == Some(k) { 2.0 } else { 1.0 }, line)));
        painter.text(label + egui::vec2(4.0, 4.0), egui::Align2::LEFT_TOP, &s.name, egui::FontId::proportional(12.0), line);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Looking down at the origin from above, the ray meets XY; through the front, XZ.
    #[test]
    fn rays_meet_the_planes() {
        let squares = origin_squares();
        let down = hit(&squares, Vec3::new(1.0, 2.0, 100.0), Vec3::new(0.0, 0.0, -1.0), 50.0);
        assert_eq!(down, Some((100.0, 0)));
        let along = hit(&squares, Vec3::new(3.0, -100.0, 4.0), Vec3::new(0.0, 1.0, 0.0), 50.0);
        assert_eq!(along, Some((100.0, 1)));
        assert_eq!(hit(&squares, Vec3::new(80.0, 0.0, 100.0), Vec3::new(0.0, 0.0, -1.0), 50.0), None, "beside the square");
    }

    /// The sketch's Y on each plane: XY's is Y, XZ's and YZ's is Z, so they read upright; a
    /// plane's square is centred where a point near it projects.
    #[test]
    fn squares_read_upright() {
        let s = origin_squares();
        assert_eq!([s[0].y, s[1].y, s[2].y], [Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.0, 0.0, 1.0)]);
        let raised = SketchPlane { origin: Vec3::new(0.0, 0.0, 5.0), x_axis: Vec3::new(1.0, 0.0, 0.0), normal: Vec3::new(0.0, 0.0, 2.0) };
        assert_eq!(square("p", &raised, Vec3::new(3.0, 4.0, 1.0)).middle, Vec3::new(3.0, 4.0, 5.0));
    }
}
