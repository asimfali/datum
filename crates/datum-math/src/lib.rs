//! Datum math: vectors, rigid transforms and exact geometric predicates.
//!
//! Decisions ("which side", "do they cross") go through exact predicates: [`orient2d`] on plain
//! coordinates, and [`orient3d`], [`orient2d_along`], [`cmp_axis`] on [`Point`]s, which may be
//! implicit intersections (D38). Constructions (new coordinates) are plain `f64`, rounded once.

mod exact;
pub mod expr;
mod implicit;

pub use exact::{exact_fallbacks, perturbed_signs, ties};
pub use implicit::{Lpi, Point, cmp_axis, orient2d_along, orient3d};

use std::ops::{Add, Mul, Neg, Sub};

use schemars::{JsonSchema, Schema, SchemaGenerator};
use serde::{Deserialize, Serialize};

/// A 2D point or vector. In JSON: `[x, y]`.
#[derive(Clone, Copy, Debug, PartialEq, Default, Serialize, Deserialize)]
#[serde(from = "[f64; 2]", into = "[f64; 2]")]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}

impl Vec2 {
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
}

/// A 3D point or vector. In JSON: `[x, y, z]`.
#[derive(Clone, Copy, Debug, PartialEq, Default, Serialize, Deserialize)]
#[serde(from = "[f64; 3]", into = "[f64; 3]")]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const ZERO: Self = Self::new(0.0, 0.0, 0.0);

    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn dot(self, o: Self) -> f64 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }

    pub fn cross(self, o: Self) -> Self {
        Self::new(self.y * o.z - self.z * o.y, self.z * o.x - self.x * o.z, self.x * o.y - self.y * o.x)
    }

    pub fn to_array(self) -> [f64; 3] {
        [self.x, self.y, self.z]
    }

    /// Index of the coordinate with the largest magnitude (0 = x, 1 = y, 2 = z).
    pub fn dominant_axis(self) -> usize {
        let a = [self.x.abs(), self.y.abs(), self.z.abs()];
        if a[0] >= a[1] && a[0] >= a[2] {
            0
        } else if a[1] >= a[2] {
            1
        } else {
            2
        }
    }
}

impl From<[f64; 2]> for Vec2 {
    fn from([x, y]: [f64; 2]) -> Self {
        Self::new(x, y)
    }
}

impl From<Vec2> for [f64; 2] {
    fn from(v: Vec2) -> Self {
        [v.x, v.y]
    }
}

impl From<[f64; 3]> for Vec3 {
    fn from([x, y, z]: [f64; 3]) -> Self {
        Self::new(x, y, z)
    }
}

impl From<Vec3> for [f64; 3] {
    fn from(v: Vec3) -> Self {
        v.to_array()
    }
}

/// Vectors are plain arrays in JSON, and so in their schemas.
macro_rules! array_schema {
    ($t:ty, $n:literal) => {
        impl JsonSchema for $t {
            fn schema_name() -> std::borrow::Cow<'static, str> {
                stringify!($t).into()
            }
            fn inline_schema() -> bool {
                true
            }
            fn json_schema(g: &mut SchemaGenerator) -> Schema {
                <[f64; $n]>::json_schema(g)
            }
        }
    };
}
array_schema!(Vec2, 2);
array_schema!(Vec3, 3);

impl Add for Vec3 {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        Self::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}

impl Sub for Vec3 {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        Self::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}

impl Neg for Vec3 {
    type Output = Self;
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y, -self.z)
    }
}

impl Mul<f64> for Vec3 {
    type Output = Self;
    fn mul(self, s: f64) -> Self {
        Self::new(self.x * s, self.y * s, self.z * s)
    }
}

/// A rigid motion: `p' = rot · p + shift`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rigid {
    rot: [Vec3; 3],
    shift: Vec3,
}

impl Rigid {
    pub const IDENTITY: Self = Self { rot: [Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0)], shift: Vec3::ZERO };

    pub fn translation(shift: Vec3) -> Self {
        Self { shift, ..Self::IDENTITY }
    }

    /// Rotation by `angle` radians about `axis` (any non-zero length) through the origin.
    pub fn rotation(axis: Vec3, angle: f64) -> Self {
        let k = axis * (1.0 / axis.dot(axis).sqrt());
        // libm, not the platform's: bit-identical on every OS (C4).
        let (s, c) = libm::sincos(angle);
        let t = 1.0 - c;
        let rot = [
            Vec3::new(t * k.x * k.x + c, t * k.x * k.y - s * k.z, t * k.x * k.z + s * k.y),
            Vec3::new(t * k.x * k.y + s * k.z, t * k.y * k.y + c, t * k.y * k.z - s * k.x),
            Vec3::new(t * k.x * k.z - s * k.y, t * k.y * k.z + s * k.x, t * k.z * k.z + c),
        ];
        Self { rot, shift: Vec3::ZERO }
    }

    /// The motion taking coordinates in the frame with axes `x, y, z` (orthonormal, right-handed)
    /// at `origin` to world coordinates.
    pub fn frame(origin: Vec3, x: Vec3, y: Vec3, z: Vec3) -> Self {
        let rot = [Vec3::new(x.x, y.x, z.x), Vec3::new(x.y, y.y, z.y), Vec3::new(x.z, y.z, z.z)];
        Self { rot, shift: origin }
    }

    /// The motion that applies `self` first, then `next`.
    pub fn then(self, next: Self) -> Self {
        let col = |i: usize| Vec3::new(self.rot[0].to_array()[i], self.rot[1].to_array()[i], self.rot[2].to_array()[i]);
        let cols = [col(0), col(1), col(2)];
        let rot = next.rot.map(|row| Vec3::new(row.dot(cols[0]), row.dot(cols[1]), row.dot(cols[2])));
        Self { rot, shift: next.apply(self.shift) }
    }

    pub fn apply(&self, p: Vec3) -> Vec3 {
        self.apply_vector(p) + self.shift
    }

    /// Rotation only: for directions and normals.
    pub fn apply_vector(&self, v: Vec3) -> Vec3 {
        Vec3::new(self.rot[0].dot(v), self.rot[1].dot(v), self.rot[2].dot(v))
    }
}

/// Sign of an exact predicate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Sign {
    Negative,
    Zero,
    Positive,
}

impl Neg for Sign {
    type Output = Self;
    fn neg(self) -> Self {
        match self {
            Self::Negative => Self::Positive,
            Self::Zero => Self::Zero,
            Self::Positive => Self::Negative,
        }
    }
}

impl Sign {
    pub(crate) fn of(v: f64) -> Self {
        if v > 0.0 {
            Self::Positive
        } else if v < 0.0 {
            Self::Negative
        } else {
            Self::Zero
        }
    }
}

/// Exact orientation of `d` relative to the plane through `a, b, c`, on plain coordinates, with the
/// convention of [`orient3d`]. Nothing moves: a contact is `Zero`.
pub fn orient3d_coords(a: Vec3, b: Vec3, c: Vec3, d: Vec3) -> Sign {
    let p = |v: Vec3| robust::Coord3D { x: v.x, y: v.y, z: v.z };
    Sign::of(robust::orient3d(p(a), p(b), p(c), p(d)))
}

/// Exact orientation of `c` relative to the directed line `a → b`: `Positive` when `a, b, c` turn
/// counterclockwise, `Zero` when collinear.
pub fn orient2d(a: Vec2, b: Vec2, c: Vec2) -> Sign {
    let p = |v: Vec2| robust::Coord { x: v.x, y: v.y };
    Sign::of(robust::orient2d(p(a), p(b), p(c)))
}

/// Exact position of `d` relative to the circle through `a, b, c` (counterclockwise):
/// `Positive` inside, `Zero` on it.
pub fn incircle(a: Vec2, b: Vec2, c: Vec2, d: Vec2) -> Sign {
    let p = |v: Vec2| robust::Coord { x: v.x, y: v.y };
    Sign::of(robust::incircle(p(a), p(b), p(c), p(d)))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn orient2d_is_exact_near_collinear() {
        let a = Vec2::new(0.5, 0.5);
        let b = Vec2::new(12.0, 12.0);
        // c is one ulp off the line y = x: naive arithmetic can return zero or the wrong sign here.
        let c = Vec2::new(24.0, f64::from_bits(24.0_f64.to_bits() + 1));
        assert_eq!(orient2d(a, b, c), Sign::Positive);
        assert_eq!(orient2d(a, b, Vec2::new(24.0, 24.0)), Sign::Zero);
        assert_eq!(orient2d(b, a, c), Sign::Negative);
    }

    #[test]
    fn rotation_then_inverse_is_identity() {
        let r = Rigid::rotation(Vec3::new(1.0, 2.0, 3.0), 0.7).then(Rigid::translation(Vec3::new(5.0, 0.0, -1.0)));
        let back = Rigid::translation(Vec3::new(-5.0, 0.0, 1.0)).then(Rigid::rotation(Vec3::new(1.0, 2.0, 3.0), -0.7));
        let p = Vec3::new(0.3, -2.0, 7.0);
        let q = back.apply(r.apply(p));
        assert!((q - p).dot(q - p) < 1e-24);
    }
}
