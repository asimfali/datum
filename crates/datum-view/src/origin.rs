//! Planes in the view: the model's origin planes, XY, XZ and YZ, as Inventor shows them when a
//! plane is to be picked, and its work planes (D150), always. Each is a square as large as the
//! model; a sketch starts on one, the view looks straight at one, a work plane is offset from one.
//! The origin's point and axes show where the planes meet.

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

/// What the view holds: the model's `bounds`, or with nothing in it the origin planes.
pub fn held(bounds: Option<(Vec3, Vec3)>) -> (Vec3, Vec3) {
    let h = half(None);
    bounds.unwrap_or((Vec3::new(-h, -h, -h), Vec3::new(h, h, h)))
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

/// How far past the picture's edge what is drawn is kept, as a share of its half: the cuts there
/// stay out of sight.
const MARGIN: f32 = 1.25;

/// The signed distances of clip point `c` from the sides of what is drawn: the near plane, then
/// left, right, bottom and top with [`MARGIN`]. Not negative inside.
fn sides(c: [f32; 4]) -> [f32; 5] {
    let reach = MARGIN * c[3];
    [c[2], reach + c[0], reach - c[0], reach + c[1], reach - c[1]]
}

/// Where segment `a`–`b`, one end on each side of side `k`, crosses it.
fn cut(a: [f32; 4], b: [f32; 4], k: usize) -> [f32; 4] {
    let (da, db) = (sides(a)[k], sides(b)[k]);
    let t = da / (da - db);
    [0, 1, 2, 3].map(|i| a[i] + (b[i] - a[i]) * t)
}

/// The part of convex polygon `corners` (clip coordinates: x, y, depth, w) in the near plane's
/// front and about the picture: a square the eye stands within is cut, not dropped, and never
/// reaches the screen coordinates far beyond the picture that the eye's nearness gives, past a
/// pixel of the painter's single precision. Lines are straight in clip space, so cutting there is
/// exact.
fn in_front(corners: &[[f32; 4]]) -> Vec<[f32; 4]> {
    let mut kept = corners.to_vec();
    for k in 0..5 {
        let from = std::mem::take(&mut kept);
        for (j, &a) in from.iter().enumerate() {
            let b = from[(j + 1) % from.len()];
            let (ina, inb) = (sides(a)[k] >= 0.0, sides(b)[k] >= 0.0);
            if ina {
                kept.push(a);
            }
            if ina != inb {
                kept.push(cut(a, b, k));
            }
        }
    }
    kept
}

/// The part of segment `a`–`b` (clip coordinates) kept as [`in_front`] keeps a polygon's, if any.
fn segment_in_front(a: [f32; 4], b: [f32; 4]) -> Option<([f32; 4], [f32; 4])> {
    let (mut a, mut b) = (a, b);
    for k in 0..5 {
        match (sides(a)[k] >= 0.0, sides(b)[k] >= 0.0) {
            (true, true) => {}
            (true, false) => b = cut(a, b, k),
            (false, true) => a = cut(a, b, k),
            (false, false) => return None,
        }
    }
    Some((a, b))
}

/// Convex polygon `points` filled and outlined, as shapes with no corners of their own: seen nearly
/// edge on, or cut at the near plane, a square has corners nearly folded back, and a polygon shape's
/// outline and soft edge there reach out in proportion to how sharp they are, as wide bands that
/// jump as the view moves.
fn outlined(points: &[Pos2], fill: Color32, stroke: Stroke) -> Vec<egui::Shape> {
    let mut mesh = egui::Mesh::default();
    for &p in points {
        mesh.colored_vertex(p, fill);
    }
    for k in 2..points.len() as u32 {
        mesh.add_triangle(0, k - 1, k);
    }
    let edges = (0..points.len()).map(|k| egui::Shape::line_segment([points[k], points[(k + 1) % points.len()]], stroke));
    std::iter::once(egui::Shape::mesh(mesh)).chain(edges).collect()
}

/// Draws `squares`, see-through, `lit` the one a click or Enter would take; `clip` takes a
/// point of the model to clip coordinates, `screen` those in front of the eye to the screen.
/// Work planes are bluish, origin planes yellowish, as in Inventor.
pub fn paint(
    painter: &Painter,
    clip: impl Fn(Vec3) -> [f32; 4],
    screen: impl Fn([f32; 4]) -> Pos2,
    squares: &[Square],
    half: f64,
    lit: Option<usize>,
    work: bool,
) {
    for (k, s) in squares.iter().enumerate() {
        // From the corner the name is written at.
        let corners = [(-1.0, 1.0), (-1.0, -1.0), (1.0, -1.0), (1.0, 1.0)].map(|(a, b)| clip(s.middle + (s.x * a + s.y * b) * half));
        let shown: Vec<Pos2> = in_front(&corners).into_iter().map(&screen).collect();
        let Some(&label) = shown.first() else { continue };
        let (fill, line) = match (lit == Some(k), work) {
            (true, _) => (Color32::from_rgba_unmultiplied(235, 110, 20, 70), Color32::from_rgb(220, 100, 20)),
            (false, true) => (Color32::from_rgba_unmultiplied(90, 140, 220, 30), Color32::from_rgb(80, 120, 200)),
            (false, false) => (Color32::from_rgba_unmultiplied(240, 190, 60, 30), Color32::from_rgb(200, 160, 60)),
        };
        painter.extend(outlined(&shown, fill, Stroke::new(if lit == Some(k) { 2.0 } else { 1.0 }, line)));
        painter.text(label + egui::vec2(4.0, 4.0), egui::Align2::LEFT_TOP, &s.name, egui::FontId::proportional(12.0), line);
    }
}

/// Draws the origin: its axes X, Y and Z (red, green, blue) out to `half`, named at their ends,
/// and its point. `clip` and `screen` as for [`paint`].
pub fn paint_origin(painter: &Painter, clip: impl Fn(Vec3) -> [f32; 4], screen: impl Fn([f32; 4]) -> Pos2, half: f64) {
    let o = clip(Vec3::ZERO);
    let axes = [
        ("X", Vec3::new(1.0, 0.0, 0.0), Color32::from_rgb(210, 50, 50)),
        ("Y", Vec3::new(0.0, 1.0, 0.0), Color32::from_rgb(40, 150, 60)),
        ("Z", Vec3::new(0.0, 0.0, 1.0), Color32::from_rgb(40, 90, 220)),
    ];
    for (name, axis, colour) in axes {
        let end = clip(axis * half);
        let Some((a, b)) = segment_in_front(o, end) else { continue };
        painter.line_segment([screen(a), screen(b)], Stroke::new(1.5, colour));
        if b == end {
            painter.text(screen(b) + egui::vec2(4.0, -4.0), egui::Align2::LEFT_BOTTOM, name, egui::FontId::proportional(12.0), colour);
        }
    }
    if o[2] >= 0.0 {
        painter.circle_filled(screen(o), 3.0, Color32::from_rgb(200, 160, 60));
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

    /// An empty model's view, fitted to what it holds, shows the origin planes whole.
    #[test]
    fn an_empty_view_holds_the_origin_planes() {
        let mut camera = crate::camera::Camera::default();
        camera.fit(Some(held(None)), 1.5);
        let (lo, hi) = held(None);
        let (m, _) = camera.matrices(Vec3::ZERO, 1.5, (hi - lo).dot(hi - lo).sqrt() * 0.5);
        for s in origin_squares() {
            for (a, b) in [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] {
                let [x, y, z, w] = crate::camera::clip(&m, s.middle + (s.x * a + s.y * b) * half(None));
                assert!(z >= 0.0 && x.abs() <= w && y.abs() <= w, "{} at ({a}, {b})", s.name);
            }
        }
    }

    /// However near the eye comes, what is drawn of a square stays about the picture.
    #[test]
    fn squares_stay_about_the_picture_however_near_the_eye() {
        let plane = SketchPlane { origin: Vec3::new(0.0, 0.0, 12.0), x_axis: Vec3::new(1.0, 0.0, 0.0), normal: Vec3::new(0.0, 0.0, 1.0) };
        let s = square("p", &plane, Vec3::ZERO);
        let (lo, hi) = (Vec3::new(-760.0, -760.0, -760.0), Vec3::new(760.0, 760.0, 760.0));
        let radius = (hi - lo).dot(hi - lo).sqrt() * 0.5;
        let mut camera = crate::camera::Camera::default();
        camera.fit(Some((lo, hi)), 1.5);
        let mut seen = 0;
        for _ in 0..60 {
            camera.zoom(0.8);
            let (m, _) = camera.matrices(Vec3::ZERO, 1.5, radius);
            let corners =
                [(-1.0, 1.0), (-1.0, -1.0), (1.0, -1.0), (1.0, 1.0)].map(|(a, b)| crate::camera::clip(&m, s.middle + (s.x * a + s.y * b) * 760.0));
            for c in in_front(&corners) {
                seen += 1;
                assert!(c[3] > 0.0 && c[0].abs() <= 1.3 * c[3] && c[1].abs() <= 1.3 * c[3], "{c:?}");
            }
        }
        assert!(seen > 0);
    }

    /// How far from `points` the painter's triangles of `shapes` reach, in pixels.
    fn reach(shapes: Vec<egui::Shape>, points: &[Pos2]) -> f32 {
        let mut t = egui::epaint::Tessellator::new(1.0, egui::epaint::TessellationOptions::default(), [1, 1], vec![]);
        let mut mesh = egui::Mesh::default();
        for s in shapes {
            t.tessellate_shape(s, &mut mesh);
        }
        let (lo, hi) = points.iter().fold((Pos2::new(f32::MAX, f32::MAX), Pos2::new(f32::MIN, f32::MIN)), |(l, h), p| (l.min(*p), h.max(*p)));
        mesh.vertices.iter().map(|v| (v.pos.x - hi.x).max(lo.x - v.pos.x).max(v.pos.y - hi.y).max(lo.y - v.pos.y)).fold(0.0, f32::max)
    }

    /// A square seen nearly edge on or cut at the near plane has a corner nearly folded back: a
    /// polygon's outline reaches thousands of pixels out from it; what is drawn here stays on it.
    #[test]
    fn a_sharp_corner_makes_no_band() {
        let tip = [Pos2::new(100.0, 300.0), Pos2::new(1500.0, 300.0), Pos2::new(1500.0, 300.25)];
        let (fill, stroke) = (Color32::from_rgba_unmultiplied(90, 140, 220, 30), Stroke::new(1.0, Color32::BLUE));
        assert!(reach(vec![egui::Shape::convex_polygon(tip.to_vec(), fill, stroke)], &tip) > 1000.0, "the band this avoids");
        let ours = reach(outlined(&tip, fill, stroke), &tip);
        assert!(ours < 3.0, "{ours}");
    }

    /// A square half behind the near plane keeps the half in front, cut where it crosses; one
    /// wholly behind keeps nothing.
    #[test]
    fn squares_are_cut_at_the_near_plane() {
        let square = [[0.0, 0.0, 1.0, 1.0], [1.0, 0.0, 1.0, 1.0], [1.0, 0.0, -1.0, 1.0], [0.0, 0.0, -1.0, 1.0]];
        assert_eq!(in_front(&square), vec![[0.0, 0.0, 1.0, 1.0], [1.0, 0.0, 1.0, 1.0], [1.0, 0.0, 0.0, 1.0], [0.0, 0.0, 0.0, 1.0]]);
        assert_eq!(in_front(&square.map(|[x, y, _, w]| [x, y, -1.0, w])), Vec::<[f32; 4]>::new());
        let (a, b) = (square[1], square[2]);
        assert_eq!(
            [segment_in_front(a, b), segment_in_front(b, a)],
            [Some((a, [1.0, 0.0, 0.0, 1.0])), Some(([1.0, 0.0, 0.0, 1.0], a))],
            "an axis too"
        );
        assert_eq!(segment_in_front(b, b), None);
    }
}
