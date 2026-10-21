//! An orbit camera around a target, Z up as in CAD. Everything is `f64` until the matrices are
//! made relative to the scene's anchor and rounded to `f32` (D24).

use datum_math::Vec3;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Camera {
    pub target: Vec3,
    pub distance: f64,
    /// Around Z, radians.
    pub yaw: f64,
    /// Above the XY plane, radians.
    pub pitch: f64,
    /// Vertical field of view, radians.
    pub fov: f64,
}

impl Default for Camera {
    fn default() -> Self {
        Self { target: Vec3::ZERO, distance: 10.0, yaw: -60f64.to_radians(), pitch: 30f64.to_radians(), fov: 35f64.to_radians() }
    }
}

/// Standard views: the default isometric one and the six along the axes (front looks along +Y).
pub const VIEWS: [&str; 7] = ["iso", "top", "bottom", "front", "back", "left", "right"];

impl Camera {
    /// The camera of standard view `name` (one of [`VIEWS`]), to be fitted to a scene.
    pub fn view(name: &str) -> Result<Self, String> {
        let quarter = std::f64::consts::FRAC_PI_2;
        let (yaw, pitch) = match name {
            "iso" => return Ok(Self::default()),
            "top" => (-quarter, quarter),
            "bottom" => (-quarter, -quarter),
            "front" => (-quarter, 0.0),
            "back" => (quarter, 0.0),
            "left" => (2.0 * quarter, 0.0),
            "right" => (0.0, 0.0),
            _ => return Err(format!("unknown view `{name}`; one of {}", VIEWS.join(", "))),
        };
        Ok(Self { yaw, pitch, ..Self::default() })
    }

    pub fn eye(&self) -> Vec3 {
        let (sy, cy) = libm::sincos(self.yaw);
        let (sp, cp) = libm::sincos(self.pitch);
        self.target + Vec3::new(cp * cy, cp * sy, sp) * self.distance
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

    /// Turns around the target by a drag of `dx, dy` pixels.
    pub fn orbit(&mut self, dx: f64, dy: f64) {
        self.yaw -= dx * 0.008;
        self.pitch = (self.pitch + dy * 0.008).clamp(-1.55, 1.55);
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
    fn axes(&self) -> (Vec3, Vec3, Vec3) {
        let f = self.target - self.eye();
        let f = f * (1.0 / f.dot(f).sqrt());
        // Right is horizontal and follows the yaw alone, so straight down (top view) is no special case.
        let (sy, cy) = libm::sincos(self.yaw);
        let r = Vec3::new(-sy, cy, 0.0);
        (r, r.cross(f), f)
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
