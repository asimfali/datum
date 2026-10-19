//! Datum operations: pure functions from parameters and input values (bodies, sketches) to a
//! new value.

mod boolean;
mod bounds;
mod pattern;
mod sketch;

use std::ops::RangeInclusive;

pub use boolean::{BooleanKind, boolean};
use datum_brep::{Body, BrepError, Grid, TopoId, prism, terrain};
use datum_math::{Rigid, Vec2, Vec3};
pub use pattern::pattern_grid;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
pub use sketch::{Placed, SketchPlane, extrude_sketch, place};

/// In JSON the kind is the `"op"` field: `{"op": "prism", "polygon": [[0, 0], …], "z0": 0, "z1": 3}`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(tag = "op", rename_all = "snake_case", deny_unknown_fields)]
pub enum Op {
    /// A polygon in the XY plane extruded from `z0` to `z1`. No inputs.
    Prism { polygon: Vec<Vec2>, z0: f64, z1: f64 },
    /// Rotation by `angle_deg` about `axis` through the origin, then translation. One input.
    /// Defaults: no translation, Z axis, no rotation.
    Transform {
        #[serde(default)]
        translate: Vec3,
        #[serde(default = "z_axis")]
        axis: Vec3,
        #[serde(default)]
        angle_deg: f64,
    },
    /// The first input (target) combined with every further input (tools) in one pass. Tools
    /// must not touch each other; faces of different bodies must not be coplanar where they meet.
    /// With `through`, every tool is first stretched along that direction past the target on both
    /// sides: holes stay through when the target changes.
    Boolean {
        kind: BooleanKind,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        through: Option<Vec3>,
    },
    /// `count[0] × count[1]` copies of the input, copy `(i, j)` moved by `i·step[0] + j·step[1]`,
    /// as one body. Copies must not touch. One input.
    PatternGrid { count: [u32; 2], step: [Vec3; 2] },
    /// Ground under a regular grid of heights down to the level `base`: node `(i, j)` stands at
    /// `origin + (i·step[0], j·step[1])` with height `heights[j·count[0] + i]`. No inputs.
    Terrain { origin: Vec2, step: [f64; 2], count: [u32; 2], heights: Vec<f64>, base: f64 },
    /// A sketch (points, lines, constraints), solved and placed: on the XY plane, on `plane`, or
    /// on the face with ID `face` of the input body. No input, or the body holding the face.
    Sketch {
        sketch: datum_sketch::Sketch,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        plane: Option<SketchPlane>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        face: Option<String>,
    },
    /// The closed profiles of the input sketch swept from `z0` to `z1` along its normal; a loop
    /// inside another is a hole. One input: a sketch.
    Extrude { z0: f64, z1: f64 },
}

/// What a feature produces.
#[derive(Clone, Debug, PartialEq)]
pub enum Value {
    Body(Body),
    Sketch(Placed),
}

impl Value {
    /// Content hash: equal values hash equal (a body's Merkle hash, a sketch's placed geometry).
    pub fn hash(&self) -> [u8; 32] {
        match self {
            Self::Body(b) => b.hash(),
            Self::Sketch(s) => s.hash,
        }
    }

    pub fn body(&self) -> Option<&Body> {
        match self {
            Self::Body(b) => Some(b),
            Self::Sketch(_) => None,
        }
    }

    pub fn sketch(&self) -> Option<&Placed> {
        match self {
            Self::Sketch(s) => Some(s),
            Self::Body(_) => None,
        }
    }
}

fn z_axis() -> Vec3 {
    Vec3::new(0.0, 0.0, 1.0)
}

impl Op {
    pub fn kind(&self) -> &'static str {
        match self {
            Self::Prism { .. } => "prism",
            Self::Transform { .. } => "transform",
            Self::Boolean { .. } => "boolean",
            Self::PatternGrid { .. } => "pattern_grid",
            Self::Terrain { .. } => "terrain",
            Self::Sketch { .. } => "sketch",
            Self::Extrude { .. } => "extrude",
        }
    }

    /// Allowed numbers of inputs.
    pub fn arity(&self) -> RangeInclusive<usize> {
        match self {
            Self::Prism { .. } | Self::Terrain { .. } => 0..=0,
            Self::Sketch { .. } => 0..=1,
            Self::Transform { .. } | Self::PatternGrid { .. } | Self::Extrude { .. } => 1..=1,
            Self::Boolean { .. } => 2..=usize::MAX,
        }
    }

    /// Feeds the kind and parameters into a cache key. `f64` values go in by their bits.
    pub fn hash_params(&self, h: &mut blake3::Hasher) {
        h.update(self.kind().as_bytes());
        let mut num = |x: f64| {
            h.update(&x.to_bits().to_le_bytes());
        };
        match self {
            Self::Prism { polygon, z0, z1 } => {
                [polygon.len() as f64, *z0, *z1].into_iter().for_each(&mut num);
                polygon.iter().flat_map(|p| [p.x, p.y]).for_each(num);
            }
            Self::Transform { translate, axis, angle_deg } => {
                translate.to_array().into_iter().chain(axis.to_array()).chain([*angle_deg]).for_each(num);
            }
            Self::Boolean { kind, through } => {
                through.iter().flat_map(|d| d.to_array()).for_each(num);
                h.update(kind.name().as_bytes());
            }
            Self::PatternGrid { count, step } => {
                count.iter().map(|&c| f64::from(c)).chain(step.iter().flat_map(|s| s.to_array())).for_each(num);
            }
            Self::Terrain { origin, step, count, heights, base } => {
                // The same bytes as feeding the numbers one by one, in one call: a million heights.
                let values = [origin.x, origin.y, step[0], step[1], f64::from(count[0]), f64::from(count[1]), *base, heights.len() as f64];
                let bytes: Vec<u8> = values.iter().chain(heights).flat_map(|x| x.to_bits().to_le_bytes()).collect();
                h.update(&bytes);
            }
            // `Debug` spells every field and every `f64` exactly (shortest round-trip form), the
            // same on every platform: a faithful key without a second description of the sketch.
            Self::Sketch { .. } => {
                h.update(format!("{self:?}").as_bytes());
            }
            Self::Extrude { z0, z1 } => [*z0, *z1].into_iter().for_each(num),
        }
    }

    /// Evaluates the operation for the feature with ID `feature`. In debug builds the result is
    /// validated here, once for every operation.
    pub fn eval(&self, feature: TopoId, values: &[&Value]) -> Result<Value, BrepError> {
        if !self.arity().contains(&values.len()) {
            return Err(BrepError::Invalid(format!("{} takes {} input(s), got {}", self.kind(), describe_arity(self.arity()), values.len())));
        }
        // Every operation but `extrude` takes bodies.
        let sketch =
            |k: usize| values[k].sketch().ok_or_else(|| BrepError::Sketch(format!("{} takes a sketch, input {} is a body", self.kind(), k + 1)));
        let inputs: Vec<&Body> = match self {
            Self::Extrude { .. } => Vec::new(),
            _ => values
                .iter()
                .enumerate()
                .map(|(k, v)| v.body().ok_or_else(|| BrepError::Sketch(format!("{} takes bodies, input {} is a sketch", self.kind(), k + 1))))
                .collect::<Result<_, _>>()?,
        };
        let body = match self {
            Self::Prism { polygon, z0, z1 } => prism(feature, polygon, *z0, *z1)?,
            Self::Transform { translate, axis, angle_deg } => {
                let rot = if *angle_deg == 0.0 {
                    Rigid::IDENTITY
                } else if axis.dot(*axis) > 0.0 {
                    Rigid::rotation(*axis, angle_deg.to_radians())
                } else {
                    return Err(BrepError::Degenerate("rotation about a zero axis".into()));
                };
                inputs[0].transformed(&rot.then(Rigid::translation(*translate)))
            }
            Self::Boolean { kind, through } => boolean(feature, *kind, *through, &inputs)?,
            Self::PatternGrid { count, step } => pattern_grid(feature, inputs[0], *count, *step)?,
            Self::Terrain { origin, step, count, heights, base } => {
                terrain(feature, &Grid { origin: *origin, step: *step, count: *count, heights }, *base)?
            }
            Self::Sketch { sketch, plane, face } => {
                let host = match (inputs.first(), face) {
                    (Some(b), Some(f)) => Some((*b, f.as_str())),
                    (None, None) => None,
                    _ => return Err(BrepError::Sketch("a sketch on a face takes the body as its input and the face ID as `face`".into())),
                };
                return Ok(Value::Sketch(place(sketch, plane.as_ref(), host)?));
            }
            Self::Extrude { z0, z1 } => extrude_sketch(feature, sketch(0)?, *z0, *z1)?,
        };
        if cfg!(debug_assertions) {
            body.validate()?;
        }
        Ok(Value::Body(body))
    }
}

/// `"1"`, `"2–4"` or `"at least 2"`.
pub fn describe_arity(r: RangeInclusive<usize>) -> String {
    match (*r.start(), *r.end()) {
        (a, b) if a == b => a.to_string(),
        (a, usize::MAX) => format!("at least {a}"),
        (a, b) => format!("{a}–{b}"),
    }
}
