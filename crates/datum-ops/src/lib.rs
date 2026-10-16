//! Datum operations: pure functions from parameters and input bodies to a new body.

use datum_brep::{Body, BrepError, TopoId, prism};
use datum_math::{Rigid, Vec2, Vec3};
use serde::{Deserialize, Serialize};

/// In JSON the kind is the `"op"` field: `{"op": "prism", "polygon": [[0, 0], …], "z0": 0, "z1": 3}`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
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
}

fn z_axis() -> Vec3 {
    Vec3::new(0.0, 0.0, 1.0)
}

impl Op {
    pub fn kind(&self) -> &'static str {
        match self {
            Self::Prism { .. } => "prism",
            Self::Transform { .. } => "transform",
        }
    }

    /// Number of input bodies.
    pub fn arity(&self) -> usize {
        match self {
            Self::Prism { .. } => 0,
            Self::Transform { .. } => 1,
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
        }
    }

    /// Evaluates the operation for the feature with ID `feature`. In debug builds the result is
    /// validated here, once for every operation.
    pub fn eval(&self, feature: TopoId, inputs: &[&Body]) -> Result<Body, BrepError> {
        if inputs.len() != self.arity() {
            return Err(BrepError::Invalid(format!("{} takes {} input(s), got {}", self.kind(), self.arity(), inputs.len())));
        }
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
        };
        if cfg!(debug_assertions) {
            body.validate()?;
        }
        Ok(body)
    }
}
