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

impl Camera {
    pub fn eye(&self) -> Vec3 {
        let (sy, cy) = libm::sincos(self.yaw);
        let (sp, cp) = libm::sincos(self.pitch);
        self.target + Vec3::new(cp * cy, cp * sy, sp) * self.distance
    }

    /// Looks at the whole of `bounds` from the current direction.
    pub fn fit(&mut self, bounds: Option<(Vec3, Vec3)>) {
        let Some((lo, hi)) = bounds else { return };
        let d = hi - lo;
        let radius = (d.dot(d).sqrt() * 0.5).max(f64::MIN_POSITIVE);
        self.target = (lo + hi) * 0.5;
        self.distance = radius / libm::sin(self.fov * 0.5) * 1.05;
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
        let r = f.cross(Vec3::new(0.0, 0.0, 1.0));
        let r = r * (1.0 / r.dot(r).sqrt().max(f64::MIN_POSITIVE));
        (r, r.cross(f), f)
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
