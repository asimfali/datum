//! An orbit camera around a target, Z up as in CAD at rest but free to turn over the top: its
//! orientation is its own right and up directions, not angles that stop at the poles.
//! Everything is `f64` until the matrices are made relative to the scene's anchor and rounded to
//! `f32` (D24).

use datum_math::Vec3;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Camera {
    pub target: Vec3,
    pub distance: f64,
    /// The view's right and up directions: unit and square to each other. It looks along
    /// `up × right`, the eye `distance` behind the target.
    pub right: Vec3,
    pub up: Vec3,
    /// Vertical field of view, radians.
    pub fov: f64,
}

impl Default for Camera {
    /// Isometric, as the home view of other CADs: from the front, the right and above, at equal
    /// angles to the three axes, Z up.
    fn default() -> Self {
        let [right, up] = frame(Vec3::new(1.0, -1.0, 1.0), Z);
        Self { target: Vec3::ZERO, distance: 10.0, right, up, fov: 35f64.to_radians() }
    }
}

const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

/// Standard views: the default isometric one and the six along the axes (front looks along +Y).
pub const VIEWS: [&str; 7] = ["iso", "top", "bottom", "front", "back", "left", "right"];

/// The right and up of a view from direction `from` (towards the eye) with `up` as near up as
/// it can be.
pub fn frame(from: Vec3, up: Vec3) -> [Vec3; 2] {
    let back = unit(from);
    let right = unit(up.cross(back));
    [right, back.cross(right)]
}

fn unit(v: Vec3) -> Vec3 {
    v * (1.0 / v.dot(v).sqrt())
}

/// `v` turned by `angle` about the unit `axis` (Rodrigues).
fn rotate(v: Vec3, axis: Vec3, angle: f64) -> Vec3 {
    let (s, c) = libm::sincos(angle);
    v * c + axis.cross(v) * s + axis * (axis.dot(v) * (1.0 - c))
}

/// The orientation `t` of the way from `a` to `b` (each a right and an up): the shortest turn
/// between them, at an even pace (quaternion slerp).
pub fn between(a: [Vec3; 2], b: [Vec3; 2], t: f64) -> [Vec3; 2] {
    let (qa, mut qb) = (quaternion(a), quaternion(b));
    let mut dot: f64 = (0..4).map(|k| qa[k] * qb[k]).sum();
    if dot < 0.0 {
        qb = qb.map(|v| -v);
        dot = -dot;
    }
    let q: [f64; 4] = if dot > 0.9995 {
        std::array::from_fn(|k| qa[k] + (qb[k] - qa[k]) * t)
    } else {
        let theta = libm::acos(dot);
        let (wa, wb) = (libm::sin((1.0 - t) * theta) / libm::sin(theta), libm::sin(t * theta) / libm::sin(theta));
        std::array::from_fn(|k| qa[k] * wa + qb[k] * wb)
    };
    let n = q.iter().map(|v| v * v).sum::<f64>().sqrt();
    let q = q.map(|v| v / n);
    [Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0)].map(|v| apply(q, v))
}

/// The rotation taking X, Y, Z to `right`, `up` and their cross product, as a unit quaternion
/// `[w, x, y, z]`.
fn quaternion([right, up]: [Vec3; 2]) -> [f64; 4] {
    let back = right.cross(up);
    // Rows of the matrix whose columns are right, up, back.
    let m = [[right.x, up.x, back.x], [right.y, up.y, back.y], [right.z, up.z, back.z]];
    let trace = m[0][0] + m[1][1] + m[2][2];
    if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        [s / 4.0, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s]
    } else if m[0][0] > m[1][1] && m[0][0] > m[2][2] {
        let s = (1.0 + m[0][0] - m[1][1] - m[2][2]).sqrt() * 2.0;
        [(m[2][1] - m[1][2]) / s, s / 4.0, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s]
    } else if m[1][1] > m[2][2] {
        let s = (1.0 + m[1][1] - m[0][0] - m[2][2]).sqrt() * 2.0;
        [(m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, s / 4.0, (m[1][2] + m[2][1]) / s]
    } else {
        let s = (1.0 + m[2][2] - m[0][0] - m[1][1]).sqrt() * 2.0;
        [(m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, s / 4.0]
    }
}

/// `v` rotated by unit quaternion `q`.
fn apply(q: [f64; 4], v: Vec3) -> Vec3 {
    let u = Vec3::new(q[1], q[2], q[3]);
    let t = u.cross(v) * 2.0;
    v + t * q[0] + u.cross(t)
}

impl Camera {
    /// The camera of standard view `name` (one of [`VIEWS`]), to be fitted to a scene: from the
    /// top and the bottom with the front down, as in other CADs.
    pub fn view(name: &str) -> Result<Self, String> {
        let (from, up) = match name {
            "iso" => return Ok(Self::default()),
            "top" => (Z, Vec3::new(0.0, 1.0, 0.0)),
            "bottom" => (-Z, Vec3::new(0.0, -1.0, 0.0)),
            "front" => (Vec3::new(0.0, -1.0, 0.0), Z),
            "back" => (Vec3::new(0.0, 1.0, 0.0), Z),
            "left" => (Vec3::new(-1.0, 0.0, 0.0), Z),
            "right" => (Vec3::new(1.0, 0.0, 0.0), Z),
            _ => return Err(format!("unknown view `{name}`; one of {}", VIEWS.join(", "))),
        };
        let [right, up] = frame(from, up);
        Ok(Self { right, up, ..Self::default() })
    }

    /// Towards the eye from the target.
    pub fn back(&self) -> Vec3 {
        self.right.cross(self.up)
    }

    /// The right and up of a view from direction `d` (towards the eye): the shortest turn from
    /// this view, its up then lined up with the nearest of the axes' directions across the line
    /// of sight. The turn goes on the way the view faces, over the top from the front's top edge
    /// to the back's rather than spinning about the vertical, and the view it ends in is square:
    /// a face's along its edges, an edge's or a corner's along the cube.
    pub fn toward(&self, d: Vec3) -> [Vec3; 2] {
        let (from, to) = (self.back(), unit(d));
        let axis = from.cross(to);
        let sin = axis.dot(axis).sqrt();
        // Exactly opposite, any turn is as short: about the view's up, which it leaves as it is.
        let up = if sin > 0.0 { rotate(self.up, axis * (1.0 / sin), libm::atan2(sin, from.dot(to))) } else { self.up };
        // The axes' directions across the line of sight; a tie (from a diagonal) goes to the
        // first: from the default view to the top, the front at the bottom.
        let across: Vec<Vec3> = [[0.0, 1.0, 0.0], [0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, -1.0]]
            .into_iter()
            .map(|a| {
                let a = Vec3::from(a);
                a - to * a.dot(to)
            })
            .filter(|a| a.dot(*a) > 1e-12)
            .map(unit)
            .collect();
        let best = across.iter().map(|a| a.dot(up)).fold(f64::NEG_INFINITY, f64::max);
        let near = across.iter().find(|a| a.dot(up) >= best - 1e-9).copied().unwrap_or(Z);
        frame(d, near)
    }

    pub fn eye(&self) -> Vec3 {
        self.target + self.back() * self.distance
    }

    /// Looks at the whole of `bounds` from the current direction, as close as a view of width
    /// / height `aspect` allows: every corner of the box just inside the picture, not its
    /// bounding sphere (a thin plate seen edge-on would be a line across a blank picture).
    pub fn fit(&mut self, bounds: Option<(Vec3, Vec3)>, aspect: f64) {
        let Some((lo, hi)) = bounds else { return };
        self.target = (lo + hi) * 0.5;
        let (r, u, f) = self.axes();
        let t = libm::tan(self.fov * 0.5);
        let half = (hi - lo) * 0.5;
        // A corner `p` from the target is in view when its offsets across the view fit the
        // frustum at its depth `distance + p·f`.
        let distance = (0..8).fold(0.0f64, |d, k| {
            let p = Vec3::new(
                if k & 1 == 0 { -half.x } else { half.x },
                if k & 2 == 0 { -half.y } else { half.y },
                if k & 4 == 0 { -half.z } else { half.z },
            );
            d.max(-p.dot(f) + (p.dot(r).abs() / (t * aspect.max(1e-6))).max(p.dot(u).abs() / t))
        });
        self.distance = (distance * 1.05).max(f64::MIN_POSITIVE);
    }

    /// Turns around the target by a drag of `dx, dy` pixels: across, about the view's up; up
    /// and down, about its right, on over the top as far as the drag goes.
    pub fn orbit(&mut self, dx: f64, dy: f64) {
        let right = rotate(self.right, self.up, -dx * 0.008);
        let up = rotate(self.up, right, -dy * 0.008);
        self.set(right, up);
    }

    /// Right and up turned a quarter about the line of sight: the picture turns
    /// counterclockwise for `1`, clockwise for `−1`.
    pub fn rolled(&self, quarter: i8) -> [Vec3; 2] {
        let (back, angle) = (self.back(), -f64::from(quarter) * std::f64::consts::FRAC_PI_2);
        [rotate(self.right, back, angle), rotate(self.up, back, angle)]
    }

    /// Turns the view by `angle` about Z: a turntable.
    pub fn spin(&mut self, angle: f64) {
        self.set(rotate(self.right, Z, angle), rotate(self.up, Z, angle));
    }

    /// Takes `right` and `up`, made unit and square again: turns add rounding.
    pub fn set(&mut self, right: Vec3, up: Vec3) {
        let right = unit(right);
        self.right = right;
        self.up = unit(up - right * up.dot(right));
    }

    /// Moves the target with a drag of `dx, dy` pixels in a view `height` pixels tall.
    pub fn pan(&mut self, dx: f64, dy: f64, height: f64) {
        let (right, up, _) = self.axes();
        let per_pixel = 2.0 * self.distance * libm::tan(self.fov * 0.5) / height.max(1.0);
        self.target = self.target - right * (dx * per_pixel) + up * (dy * per_pixel);
    }

    /// Moves closer (`factor < 1`) or away.
    pub fn zoom(&mut self, factor: f64) {
        self.distance = (self.distance * factor).max(f64::MIN_POSITIVE);
    }

    /// Right, up and forward directions of the view.
    pub fn axes(&self) -> (Vec3, Vec3, Vec3) {
        (self.right, self.up, -self.back())
    }

    /// The ray through a point of the picture, `x` and `y` from −1 to 1 (right, up), of width /
    /// height `aspect`: from the eye, a unit direction.
    pub fn ray(&self, x: f64, y: f64, aspect: f64) -> (Vec3, Vec3) {
        let (r, u, f) = self.axes();
        let t = libm::tan(self.fov * 0.5);
        let d = f + r * (x * t * aspect) + u * (y * t);
        (self.eye(), d * (1.0 / d.dot(d).sqrt()))
    }

    /// View-projection matrix (column-major, wgpu clip space with depth 0..1) for points given
    /// relative to `anchor`, and the eye relative to it. The near and far planes enclose a
    /// sphere of `radius` around the target.
    pub fn matrices(&self, anchor: Vec3, aspect: f64, radius: f64) -> ([f32; 16], [f32; 3]) {
        let eye = self.eye() - anchor;
        let (r, u, f) = self.axes();
        let far = self.distance + radius * 1.5;
        let near = (self.distance - radius * 1.5).max(far * 1e-5);
        let t = 1.0 / libm::tan(self.fov * 0.5);
        let (sx, sy) = (t / aspect.max(1e-6), t);
        // Rows of the view matrix: right, up, back; then perspective with depth near → 0, far → 1.
        let view = [[r.x, r.y, r.z, -r.dot(eye)], [u.x, u.y, u.z, -u.dot(eye)], [-f.x, -f.y, -f.z, f.dot(eye)]];
        let (a, b) = (far / (near - far), near * far / (near - far));
        let rows = [
            view[0].map(|v| v * sx),
            view[1].map(|v| v * sy),
            [a * view[2][0], a * view[2][1], a * view[2][2], a * view[2][3] + b],
            view[2].map(|v| -v),
        ];
        let mut m = [0f32; 16];
        for (row, values) in rows.iter().enumerate() {
            for (col, v) in values.iter().enumerate() {
                m[col * 4 + row] = *v as f32;
            }
        }
        (m, [eye.x as f32, eye.y as f32, eye.z as f32])
    }
}

/// The view of a sketch being edited: straight at its plane, orthographic, so that lengths on
/// the screen are true. Everything in sketch coordinates; `frame` is the sketch's origin and X,
/// Y, Z axes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SketchCamera {
    pub frame: [Vec3; 4],
    /// The sketch point at the middle of the picture.
    pub center: [f64; 2],
    /// Sketch units from the middle of the picture to its top.
    pub half_height: f64,
}

impl SketchCamera {
    /// Sketch units per pixel in a picture `height` pixels tall.
    pub fn per_pixel(&self, height: f64) -> f64 {
        2.0 * self.half_height / height.max(1.0)
    }

    /// The sketch point at pixel offset `(dx, dy)` from the middle of a picture `height` tall
    /// (y down, as on the screen), and back.
    pub fn at(&self, dx: f64, dy: f64, height: f64) -> [f64; 2] {
        let k = self.per_pixel(height);
        [self.center[0] + dx * k, self.center[1] - dy * k]
    }

    pub fn offset(&self, p: [f64; 2], height: f64) -> (f64, f64) {
        let k = self.per_pixel(height);
        ((p[0] - self.center[0]) / k, (self.center[1] - p[1]) / k)
    }

    /// Moves the picture with a drag of `dx, dy` pixels.
    pub fn pan(&mut self, dx: f64, dy: f64, height: f64) {
        let k = self.per_pixel(height);
        self.center = [self.center[0] - dx * k, self.center[1] + dy * k];
    }

    /// Zooms by `factor` (< 1: closer), keeping the sketch point under pixel offset `(dx, dy)`
    /// where it is.
    pub fn zoom_at(&mut self, factor: f64, dx: f64, dy: f64, height: f64) {
        let before = self.at(dx, dy, height);
        self.half_height = (self.half_height * factor).max(f64::MIN_POSITIVE);
        let after = self.at(dx, dy, height);
        self.center = [self.center[0] + before[0] - after[0], self.center[1] + before[1] - after[1]];
    }

    /// Orthographic view-projection matrix (as [`Camera::matrices`]) for points relative to
    /// `anchor`, the scene within `radius` of it, and an eye far along the plane's normal.
    pub fn matrices(&self, anchor: Vec3, aspect: f64, radius: f64) -> ([f32; 16], [f32; 3]) {
        let [o, x, y, z] = self.frame;
        let c = o + x * self.center[0] + y * self.center[1] - anchor;
        // Depth: the camera looks down −Z; the whole scene lies within `reach` of the plane.
        let reach = (z.dot(c).abs() + radius) * 1.01 + f64::MIN_POSITIVE;
        let (sx, sy) = (1.0 / (self.half_height * aspect.max(1e-6)), 1.0 / self.half_height);
        let rows = [
            [x.x * sx, x.y * sx, x.z * sx, -x.dot(c) * sx],
            [y.x * sy, y.y * sy, y.z * sy, -y.dot(c) * sy],
            [-z.x * 0.5 / reach, -z.y * 0.5 / reach, -z.z * 0.5 / reach, z.dot(c) * 0.5 / reach + 0.5],
            [0.0, 0.0, 0.0, 1.0],
        ];
        let mut m = [0f32; 16];
        for (row, values) in rows.iter().enumerate() {
            for (col, v) in values.iter().enumerate() {
                m[col * 4 + row] = *v as f32;
            }
        }
        let eye = c + z * (reach * 100.0);
        (m, [eye.x as f32, eye.y as f32, eye.z as f32])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// From the front's top edge to the back's, the turn goes over the top, about X, and ends
    /// upside down as the way there leaves it, not spun about the vertical; then on to the top,
    /// the front at the bottom.
    #[test]
    fn edge_to_edge_goes_over_the_top() {
        let c = Camera::view("front").unwrap();
        let [right, up] = c.toward(Vec3::new(0.0, -1.0, 1.0));
        assert_eq!(right, Vec3::new(1.0, 0.0, 0.0));
        let c = Camera { right, up, ..c };
        let [right, up] = c.toward(Vec3::new(0.0, 1.0, 1.0));
        assert_eq!(right, Vec3::new(1.0, 0.0, 0.0), "the same right: a turn about X");
        assert!(up.z < 0.0 && up.y > 0.0, "{up:?}");
        let c = Camera { right, up, ..c };
        let [right, up] = c.toward(Z);
        assert_eq!((right, up), (Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0)));
    }

    /// Dragged up and on, the view goes over the top and down the back, no ceiling at the pole;
    /// right and up stay unit and square.
    #[test]
    fn the_view_turns_over_the_top() {
        let mut c = Camera::view("front").unwrap();
        // A quarter turn is about 196 pixels of drag: 270 pixels are some 124°, a step at a time.
        for _ in 0..30 {
            c.orbit(0.0, 9.0);
        }
        let back = c.back();
        assert!(back.y > 0.0 && back.z > 0.0, "past the top, behind: {back:?}");
        assert!(c.up.z < 0.0, "upside down, as a turn over the top leaves it: {:?}", c.up);
        assert!((c.right.dot(c.right) - 1.0).abs() < 1e-12 && c.right.dot(c.up).abs() < 1e-12);
    }

    /// The standard views look along the axes, Z up, the top with the front down.
    #[test]
    fn standard_views() {
        let top = Camera::view("top").unwrap();
        assert_eq!((top.back(), top.up), (Z, Vec3::new(0.0, 1.0, 0.0)));
        let front = Camera::view("front").unwrap();
        assert_eq!((front.back(), front.up), (Vec3::new(0.0, -1.0, 0.0), Z));
    }
}
