//! Where the lines and circles curves lie on cross.

use crate::Vec2;

/// The whole line or circle a curve lies on.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Carrier {
    /// Through two different points.
    Line(Vec2, Vec2),
    Circle(Vec2, f64),
}

/// Where `a` and `b` cross: none, one or two points. Parallel lines and circles about one centre
/// do not cross, whether or not they coincide.
pub fn crossings(a: Carrier, b: Carrier) -> Vec<Vec2> {
    let sub = |p: Vec2, q: Vec2| Vec2::new(p.x - q.x, p.y - q.y);
    let cross = |u: Vec2, v: Vec2| u.x * v.y - u.y * v.x;
    let dot = |u: Vec2, v: Vec2| u.x * v.x + u.y * v.y;
    let two = |x: Vec2, y: Vec2| if x == y { vec![x] } else { vec![x, y] };
    match (a, b) {
        (Carrier::Line(a, b), Carrier::Line(p, q)) => {
            let (u, v) = (sub(b, a), sub(q, p));
            let d = cross(u, v);
            if d == 0.0 {
                return Vec::new();
            }
            let t = cross(sub(p, a), v) / d;
            vec![Vec2::new(a.x + u.x * t, a.y + u.y * t)]
        }
        (Carrier::Line(a, b), Carrier::Circle(c, r)) | (Carrier::Circle(c, r), Carrier::Line(a, b)) => {
            let u = sub(b, a);
            let l2 = dot(u, u);
            let f = dot(sub(c, a), u) / l2;
            let foot = Vec2::new(a.x + u.x * f, a.y + u.y * f);
            let d2 = dot(sub(c, foot), sub(c, foot));
            if d2 > r * r {
                return Vec::new();
            }
            let h = ((r * r - d2) / l2).sqrt();
            two(Vec2::new(a.x + u.x * (f - h), a.y + u.y * (f - h)), Vec2::new(a.x + u.x * (f + h), a.y + u.y * (f + h)))
        }
        (Carrier::Circle(c, r), Carrier::Circle(k, q)) => {
            let d = libm::hypot(c.x - k.x, c.y - k.y);
            if d == 0.0 || d > r + q || d < (r - q).abs() {
                return Vec::new();
            }
            let along = (r * r - q * q + d * d) / (2.0 * d);
            let h = (r * r - along * along).max(0.0).sqrt();
            let u = Vec2::new((k.x - c.x) / d, (k.y - c.y) / d);
            let f = Vec2::new(c.x + u.x * along, c.y + u.y * along);
            two(Vec2::new(f.x - u.y * h, f.y + u.x * h), Vec2::new(f.x + u.y * h, f.y - u.x * h))
        }
    }
}
