//! The model's origin planes, XY, XZ and YZ, as Inventor shows them when a plane is to be picked:
//! square, through the origin, as large as the model; a sketch starts on one, the view looks
//! straight at one.

use datum_math::Vec3;
use egui::{Color32, Painter, Pos2, Stroke};

/// Each plane: its name, the sketch's X on it and its normal (the sketch's Y is their cross
/// product, so XZ is seen from the front and YZ from the right).
pub const PLANES: [(&str, [f64; 3], [f64; 3]); 3] =
    [("XY", [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]), ("XZ", [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]), ("YZ", [0.0, 1.0, 0.0], [1.0, 0.0, 0.0])];

/// The plane a sketch starts on when no other is picked: XY, as in Inventor.
pub const FIRST: usize = 0;

/// Half the side of the planes: past the model on every side, or 50 (mm) with no model.
pub fn half(bounds: Option<(Vec3, Vec3)>) -> f64 {
    let reach = bounds.map_or(0.0, |(lo, hi)| [lo.x, lo.y, lo.z, hi.x, hi.y, hi.z].into_iter().fold(0.0, |m: f64, v| m.max(v.abs())));
    if reach > 0.0 { reach * 1.15 } else { 50.0 }
}

/// The plane's X, Y and normal.
fn axes(k: usize) -> [Vec3; 3] {
    let (_, x, n) = PLANES[k];
    let (x, n) = (Vec3::from(x), Vec3::from(n));
    [x, n.cross(x), n]
}

/// The fields of a sketch feature on origin plane `k`: none for XY, where a sketch lies
/// without a plane, else the plane.
pub fn sketch_fields(k: usize) -> serde_json::Value {
    let (_, x, n) = PLANES[k];
    if k == FIRST { serde_json::json!({}) } else { serde_json::json!({"plane": {"origin": [0.0, 0.0, 0.0], "x_axis": x, "normal": n}}) }
}

/// The nearest origin plane a ray from `origin` along `dir` passes through within `half` of the
/// origin, and how far along the ray. A pick on the screen.
pub fn hit(origin: Vec3, dir: Vec3, half: f64) -> Option<(f64, usize)> {
    (0..PLANES.len())
        .filter_map(|k| {
            let [x, y, n] = axes(k);
            let across = dir.dot(n);
            if across == 0.0 {
                return None;
            }
            let t = -origin.dot(n) / across;
            let p = origin + dir * t;
            (t > 0.0 && p.dot(x).abs() <= half && p.dot(y).abs() <= half).then_some((t, k))
        })
        .min_by(|a, b| a.0.total_cmp(&b.0))
}

/// The unit normal of origin plane `k` turned to a viewer looking along `dir`.
pub fn normal_towards(k: usize, dir: Vec3) -> Vec3 {
    let n = axes(k)[2];
    if n.dot(dir) > 0.0 { -n } else { n }
}

/// Draws the planes, see-through, `lit` the one a click or Enter would take; `screen` places a
/// point of the model on the screen (none behind the eye).
pub fn paint(painter: &Painter, screen: impl Fn(Vec3) -> Option<Pos2>, half: f64, lit: Option<usize>) {
    for (k, (name, ..)) in PLANES.iter().enumerate() {
        let [x, y, _] = axes(k);
        let corners: Option<Vec<Pos2>> =
            [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)].iter().map(|&(a, b)| screen((x * a + y * b) * half)).collect();
        let Some(corners) = corners else { continue };
        let on = lit == Some(k);
        let (fill, line) = if on {
            (Color32::from_rgba_unmultiplied(235, 110, 20, 70), Color32::from_rgb(220, 100, 20))
        } else {
            (Color32::from_rgba_unmultiplied(240, 190, 60, 30), Color32::from_rgb(200, 160, 60))
        };
        let label = corners[3];
        painter.add(egui::Shape::convex_polygon(corners, fill, Stroke::new(if on { 2.0 } else { 1.0 }, line)));
        painter.text(label + egui::vec2(4.0, 4.0), egui::Align2::LEFT_TOP, *name, egui::FontId::proportional(12.0), line);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Looking down at the origin from above, the ray meets XY; through the front, XZ.
    #[test]
    fn rays_meet_the_planes() {
        let down = hit(Vec3::new(1.0, 2.0, 100.0), Vec3::new(0.0, 0.0, -1.0), 50.0);
        assert_eq!(down, Some((100.0, 0)));
        let along = hit(Vec3::new(3.0, -100.0, 4.0), Vec3::new(0.0, 1.0, 0.0), 50.0);
        assert_eq!(along, Some((100.0, 1)));
        assert_eq!(hit(Vec3::new(80.0, 0.0, 100.0), Vec3::new(0.0, 0.0, -1.0), 50.0), None, "beside the square");
    }

    /// The sketch's Y on each plane: XY's is Y, XZ's and YZ's is Z, so they read upright.
    #[test]
    fn sketches_read_upright() {
        assert_eq!(axes(0)[1], Vec3::new(0.0, 1.0, 0.0));
        assert_eq!(axes(1)[1], Vec3::new(0.0, 0.0, 1.0));
        assert_eq!(axes(2)[1], Vec3::new(0.0, 0.0, 1.0));
    }
}
