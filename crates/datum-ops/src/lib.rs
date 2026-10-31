//! Datum operations: pure functions from parameters and input values (bodies, sketches) to a
//! new value.

mod bent;
mod boolean;
mod bounds;
mod copies;
mod cut;
mod flange;
mod pattern;
mod sheet;
mod sketch;
mod unfold;

use std::borrow::Cow;
use std::collections::BTreeMap;
use std::ops::RangeInclusive;

pub use bent::NearBend;
pub use boolean::{BooleanKind, boolean};
pub use copies::{Copies, Place};
use cut::{Reach, from_face, through_walls};
use datum_brep::{Body, BrepError, Grid, TopoId, prism, terrain};
use datum_math::expr::{ExprError, Num};
use datum_math::{Rigid, Vec2, Vec3};
pub use flange::{FlangeDims, flange_edges, flange_ends, flanges};
pub use pattern::pattern_grid;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
pub use sheet::{Sheet, Side, path_ends, paths, sheet};
pub use sketch::{
    Developed, ORIGIN, OriginPlane, Placed, Projection, SketchPlane, chosen, contour_at, depth_to, extrude_sketch, face_plane, place, profile_regions,
};
pub use unfold::{Flat, FlatPiece, sheet_sides, unfold, unfold_side};

/// In JSON the kind is the `"op"` field: `{"op": "prism", "polygon": [[0, 0], …], "z0": 0, "z1": 3}`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(tag = "op", rename_all = "snake_case", deny_unknown_fields)]
pub enum Op {
    /// A polygon in the XY plane extruded from `z0` to `z1`. No inputs.
    Prism { polygon: Vec<Vec2>, z0: Num, z1: Num },
    /// Rotation by `angle_deg` about `axis` through the origin, then translation. One input.
    /// Defaults: no translation, Z axis, no rotation.
    Transform {
        #[serde(default = "zero3")]
        translate: [Num; 3],
        #[serde(default = "z_axis")]
        axis: Vec3,
        #[serde(default = "zero")]
        angle_deg: Num,
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
    PatternGrid { count: [Num; 2], step: [[Num; 3]; 2] },
    /// Ground under a regular grid of heights down to the level `base`: node `(i, j)` stands at
    /// `origin + (i·step[0], j·step[1])` with height `heights[j·count[0] + i]`. No inputs.
    Terrain { origin: Vec2, step: [f64; 2], count: [u32; 2], heights: Vec<f64>, base: f64 },
    /// A sketch (points, lines, constraints), solved and placed: on the XY plane, on `plane`, or
    /// on the face with ID `face` of the input body. On a cylinder face of a rolled sheet, the
    /// sketch lies in the sheet's development, lengths along its neutral layer at `k_factor` of
    /// the thickness from the inside (0.5: the middle): coordinates of the flat blank; on a plane
    /// face of sheet metal, the K-factor a cut across its bends goes on through the blank at (D197). `project`
    /// names edges and vertices of the input body (by ID) projected into the sketch as fixed
    /// construction geometry to dimension from. No input, or the body holding the face or the
    /// projected edges.
    Sketch {
        sketch: datum_sketch::Sketch,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        plane: Option<SketchPlane>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        face: Option<String>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        k_factor: Option<Num>,
        #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
        project: BTreeMap<String, Projection>,
    },
    /// The closed profiles of the input sketch swept from `z0` to `z1` along its normal (either
    /// may be the larger); a loop inside another is a hole. `symmetric`: `z1` deep, half on each
    /// side of the sketch. Up to a plane (D150): the work plane as the last input instead of
    /// `z1`, parallel to the sketch. `mode` (D185, D187): a new body (the default), or added to
    /// (`join`), cut from (`cut`) or intersected with (`intersect`) the target body, the first
    /// input. From a face of the target the depth's sign is geometry: out of the body positive,
    /// into it negative; a boss is a join out of the face, a pocket a cut into it. `through`: a
    /// cut from a face of the target through to the opposite face, without `z1`. `profiles`
    /// (D191): curves of the sketch, each choosing the closed loop it is on with its holes; none:
    /// every closed loop; open chains are left out. Inputs: the target unless new, the sketch,
    /// then the plane if any.
    Extrude {
        #[serde(default = "zero")]
        z0: Num,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        z1: Option<Num>,
        #[serde(default, skip_serializing_if = "std::ops::Not::not")]
        symmetric: bool,
        #[serde(default, skip_serializing_if = "Mode::is_new")]
        mode: Mode,
        #[serde(default, skip_serializing_if = "std::ops::Not::not")]
        through: bool,
        /// Through one wall only, the one the sketch lies on: sheet metal's cut, as the blank is
        /// punched (D196). With `through`.
        #[serde(default, skip_serializing_if = "std::ops::Not::not")]
        wall: bool,
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        profiles: Vec<String>,
    },
    /// A work plane (D150), `offset` along its normal from the origin plane `base` (XY when not
    /// given), from the input work plane, from plane face `face` of the input body, or through
    /// three `points` (D204): its origin the first, its X toward the second, its normal by the
    /// right hand from the second to the third. Sketches lie on it (as their input), extrusions go
    /// up to it. No input, the plane, the body, or what the points are of (up to three).
    Plane {
        #[serde(default, skip_serializing_if = "Option::is_none")]
        base: Option<OriginPlane>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        face: Option<String>,
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        points: Vec<Spot>,
        #[serde(default = "zero")]
        offset: Num,
    },
    /// Flanges (M8, D205) on end faces `faces` of the input sheet metal: a bend of inner `radius`
    /// (the part's rule's, put here when the model is built) turning by `angle` degrees toward
    /// the sheet's front side (`flip`: the other), and a wall beyond it, `height` from the outer
    /// surface of the sheet (for a right angle; else along the flange from where its outer
    /// surface meets the sheet's). One input: the sheet.
    Flange {
        faces: Vec<String>,
        height: Num,
        #[serde(default = "right_angle")]
        angle: Num,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        radius: Option<Num>,
        #[serde(default, skip_serializing_if = "std::ops::Not::not")]
        flip: bool,
    },
    /// Sheet metal from the input sketch's path: its lines and arcs (an open chain, from point
    /// `start` or else from its end named first) given `thickness` on `side` of the path (looking
    /// along it), bent at every corner of two lines with inner `radius`, rolled along every arc
    /// (tangent to its neighbours by the sketch's constraints, D200), and extruded from `z0` to `z1`
    /// along the sketch's normal. One input: a sketch. The thickness and the radius are the part's sheet rule's
    /// (D193), put here when the model is built; a file has its own only from before rules.
    Sheet {
        #[serde(default, skip_serializing_if = "Option::is_none")]
        thickness: Option<Num>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        radius: Option<Num>,
        side: Side,
        z0: Num,
        z1: Num,
        /// A curve of the open chain to take (a skeleton sketch holds many, D191); none: the
        /// sketch's only one.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        path: Option<String>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        start: Option<String>,
    },
    /// A face of sheet metal (D195): the closed profiles of the input sketch (curves `profiles`
    /// choose them, D191; none: every one) as a plate of `thickness`, on the sketch's front
    /// (`flip`: its back; `symmetric`: half each way). The thickness is the part's sheet rule's,
    /// put here when the model is built (D193). One input: a sketch.
    Plate {
        #[serde(default, skip_serializing_if = "Option::is_none")]
        thickness: Option<Num>,
        #[serde(default, skip_serializing_if = "std::ops::Not::not")]
        flip: bool,
        #[serde(default, skip_serializing_if = "std::ops::Not::not")]
        symmetric: bool,
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        profiles: Vec<String>,
    },
    /// The flat blank of sheet metal (a rolled sheet, a sheet from a path): the sheet's faces on
    /// the side of face `face`, plane and bent, laid flat with every cut; bends by their neutral
    /// layer at `k_factor` of the thickness (the K-factor cuts on bends were sketched with). Seen
    /// from that face's side, its corner at the origin. Exported as DXF. One input: the body.
    Unfold { face: String, k_factor: Num },
}

/// Said when sheet metal is built with no rule giving its thickness.
const NO_SHEET_RULE: &str = "sheet metal takes its thickness and bend radius from the part's sheet rule: give the part one";

/// What an extrusion does with its target body (D185).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Mode {
    #[default]
    New,
    Join,
    Cut,
    Intersect,
}

impl Mode {
    fn is_new(&self) -> bool {
        *self == Self::New
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::New => "new",
            Self::Join => "join",
            Self::Cut => "cut",
            Self::Intersect => "intersect",
        }
    }
}

fn zero() -> Num {
    Num::Value(0.0)
}

fn right_angle() -> Num {
    Num::Value(90.0)
}

fn zero3() -> [Num; 3] {
    [zero(), zero(), zero()]
}

/// A number of a resolved operation (see [`Op::resolve`]).
fn num(n: &Num) -> Result<f64, BrepError> {
    n.value().ok_or_else(|| BrepError::Invalid(format!("`{n}` is not evaluated: resolve the operation's parameters first")))
}

fn vec3(v: &[Num; 3]) -> Result<Vec3, BrepError> {
    Ok(Vec3::new(num(&v[0])?, num(&v[1])?, num(&v[2])?))
}

/// A count: a whole number, not negative.
fn count(n: &Num) -> Result<u32, BrepError> {
    let v = num(n)?;
    if v >= 0.0 && v <= f64::from(u32::MAX) && v.fract() == 0.0 {
        Ok(v as u32)
    } else {
        Err(BrepError::Degenerate(format!("a count must be a whole number, got {v}")))
    }
}

/// What a feature produces.
#[derive(Clone, Debug, PartialEq)]
pub enum Value {
    Body(Body),
    /// Boxed: a placed sketch is much larger than the rest.
    Sketch(Box<Placed>),
    Flat(Flat),
    /// A work plane: its origin, X axis and normal, as a sketch plane.
    Plane(SketchPlane),
}

impl Value {
    /// Content hash: equal values hash equal (a body's Merkle hash, a sketch's placed geometry).
    pub fn hash(&self) -> [u8; 32] {
        match self {
            Self::Body(b) => b.hash(),
            Self::Sketch(s) => s.hash,
            Self::Flat(f) => f.hash,
            Self::Plane(p) => {
                let mut h = blake3::Hasher::new();
                for x in [p.origin, p.x_axis, p.normal].iter().flat_map(|v| v.to_array()) {
                    h.update(&x.to_bits().to_le_bytes());
                }
                *h.finalize().as_bytes()
            }
        }
    }

    pub fn body(&self) -> Option<&Body> {
        match self {
            Self::Body(b) => Some(b),
            _ => None,
        }
    }

    pub fn sketch(&self) -> Option<&Placed> {
        match self {
            Self::Sketch(s) => Some(s),
            _ => None,
        }
    }

    pub fn flat(&self) -> Option<&Flat> {
        match self {
            Self::Flat(f) => Some(f),
            _ => None,
        }
    }

    pub fn plane(&self) -> Option<&SketchPlane> {
        match self {
            Self::Plane(p) => Some(p),
            _ => None,
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
            Self::Unfold { .. } => "unfold",
            Self::Sheet { .. } => "sheet",
            Self::Plate { .. } => "plate",
            Self::Flange { .. } => "flange",
            Self::Plane { .. } => "plane",
        }
    }

    /// Allowed numbers of inputs.
    pub fn arity(&self) -> RangeInclusive<usize> {
        match self {
            Self::Prism { .. } | Self::Terrain { .. } => 0..=0,
            Self::Sketch { .. } => 0..=1,
            Self::Plane { .. } => 0..=3,
            Self::Extrude { .. } => 1..=3,
            Self::Transform { .. }
            | Self::PatternGrid { .. }
            | Self::Unfold { .. }
            | Self::Sheet { .. }
            | Self::Plate { .. }
            | Self::Flange { .. } => 1..=1,
            Self::Boolean { .. } => 2..=usize::MAX,
        }
    }

    /// The operation with every number evaluated, parameter values coming from `lookup` (D57).
    /// An error names the field and says what is wrong with its expression.
    pub fn resolve(&self, lookup: &impl Fn(&str) -> Result<f64, ExprError>) -> Result<Cow<'_, Op>, String> {
        let r = |n: &Num, field: &str| n.resolve(lookup).map(Num::Value).map_err(|e| format!("`{field}`: {e}"));
        let r3 = |v: &[Num; 3], field: &str| -> Result<[Num; 3], String> { Ok([r(&v[0], field)?, r(&v[1], field)?, r(&v[2], field)?]) };
        Ok(Cow::Owned(match self {
            Self::Prism { polygon, z0, z1 } => Self::Prism { polygon: polygon.clone(), z0: r(z0, "z0")?, z1: r(z1, "z1")? },
            Self::Transform { translate, axis, angle_deg } => {
                Self::Transform { translate: r3(translate, "translate")?, axis: *axis, angle_deg: r(angle_deg, "angle_deg")? }
            }
            Self::PatternGrid { count, step } => {
                Self::PatternGrid { count: [r(&count[0], "count")?, r(&count[1], "count")?], step: [r3(&step[0], "step")?, r3(&step[1], "step")?] }
            }
            Self::Sketch { sketch, plane, face, k_factor, project } => Self::Sketch {
                sketch: sketch.resolve(lookup)?,
                plane: *plane,
                face: face.clone(),
                k_factor: k_factor.as_ref().map(|k| r(k, "k_factor")).transpose()?,
                project: project.clone(),
            },
            Self::Extrude { z0, z1, symmetric, mode, through, wall, profiles } => Self::Extrude {
                z0: r(z0, "z0")?,
                z1: z1.as_ref().map(|z| r(z, "z1")).transpose()?,
                symmetric: *symmetric,
                mode: *mode,
                through: *through,
                wall: *wall,
                profiles: profiles.clone(),
            },
            Self::Plane { base, face, points, offset } => {
                Self::Plane { base: *base, face: face.clone(), points: points.clone(), offset: r(offset, "offset")? }
            }
            Self::Unfold { face, k_factor } => Self::Unfold { face: face.clone(), k_factor: r(k_factor, "k_factor")? },
            Self::Plate { thickness, flip, symmetric, profiles } => Self::Plate {
                thickness: thickness.as_ref().map(|n| r(n, "thickness")).transpose()?,
                flip: *flip,
                symmetric: *symmetric,
                profiles: profiles.clone(),
            },
            Self::Sheet { thickness, radius, side, z0, z1, path, start } => Self::Sheet {
                thickness: thickness.as_ref().map(|n| r(n, "thickness")).transpose()?,
                radius: radius.as_ref().map(|n| r(n, "radius")).transpose()?,
                side: *side,
                z0: r(z0, "z0")?,
                z1: r(z1, "z1")?,
                path: path.clone(),
                start: start.clone(),
            },
            Self::Flange { faces, height, angle, radius, flip } => Self::Flange {
                faces: faces.clone(),
                height: r(height, "height")?,
                angle: r(angle, "angle")?,
                radius: radius.as_ref().map(|n| r(n, "radius")).transpose()?,
                flip: *flip,
            },
            Self::Boolean { .. } | Self::Terrain { .. } => return Ok(Cow::Borrowed(self)),
        }))
    }

    /// Feeds the kind and parameters of a resolved operation into a cache key. `f64` values go
    /// in by their bits.
    pub fn hash_params(&self, h: &mut blake3::Hasher) {
        h.update(self.kind().as_bytes());
        let mut num = |x: f64| {
            h.update(&x.to_bits().to_le_bytes());
        };
        let value = |n: &Num| n.value().unwrap_or(f64::NAN);
        match self {
            Self::Prism { polygon, z0, z1 } => {
                [polygon.len() as f64, value(z0), value(z1)].into_iter().for_each(&mut num);
                polygon.iter().flat_map(|p| [p.x, p.y]).for_each(num);
            }
            Self::Transform { translate, axis, angle_deg } => {
                translate.iter().map(value).chain(axis.to_array()).chain([value(angle_deg)]).for_each(num);
            }
            Self::Boolean { kind, through } => {
                through.iter().flat_map(|d| d.to_array()).for_each(num);
                h.update(kind.name().as_bytes());
            }
            Self::PatternGrid { count, step } => {
                count.iter().chain(step.iter().flatten()).map(value).for_each(num);
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
            // `Debug` spells every field exactly, as for a sketch.
            Self::Extrude { .. } | Self::Plane { .. } => {
                h.update(format!("{self:?}").as_bytes());
            }
            Self::Unfold { face, k_factor } => {
                num(value(k_factor));
                h.update(face.as_bytes());
            }
            // `Debug` spells every field exactly, as for a sketch.
            Self::Sheet { .. } | Self::Plate { .. } | Self::Flange { .. } => {
                h.update(format!("{self:?}").as_bytes());
            }
        }
    }

    /// Evaluates the operation for the feature with ID `feature`. In debug builds the result is
    /// validated here, once for every operation.
    pub fn eval(&self, feature: TopoId, values: &[&Value]) -> Result<Value, BrepError> {
        if !self.arity().contains(&values.len()) {
            return Err(BrepError::Invalid(format!("{} takes {} input(s), got {}", self.kind(), describe_arity(self.arity()), values.len())));
        }
        // A work plane, and a sketch on one, take no body.
        if let Self::Plane { base, face, points, offset } = self {
            let from = match (values.first(), face) {
                _ if !points.is_empty() => {
                    if base.is_some() || face.is_some() {
                        return Err(BrepError::Sketch("a plane through points takes no `base` or `face`".into()));
                    }
                    through(points, values)?
                }
                (None, None) => base.unwrap_or_default().plane(),
                (Some(v), None) if base.is_none() => {
                    *v.plane().ok_or_else(|| BrepError::Sketch("a plane takes a work plane as its input, or a body with `face`".into()))?
                }
                (Some(v), Some(f)) if base.is_none() => {
                    face_plane(v.body().ok_or_else(|| BrepError::Sketch("a plane on a face takes the body as its input".into()))?, f)?
                }
                _ if values.len() > 1 => return Err(BrepError::Sketch("a plane takes more than one input only through `points`".into())),
                _ => return Err(BrepError::Sketch("a plane is offset from one thing: an origin plane (`base`), a work plane, or a face".into())),
            };
            let n = from.normal * (1.0 / from.normal.dot(from.normal).sqrt());
            return Ok(Value::Plane(SketchPlane { origin: from.origin + n * num(offset)?, ..from }));
        }
        if let (Self::Sketch { sketch, plane, face, k_factor, project }, Some(Value::Plane(on))) = (self, values.first().copied()) {
            if plane.is_some() || face.is_some() || !project.is_empty() || k_factor.is_some() {
                return Err(BrepError::Sketch("a sketch on a work plane takes no `plane`, `face`, `project` or `k_factor`".into()));
            }
            return Ok(Value::Sketch(Box::new(place(sketch, Some(on), None, None, None, project)?)));
        }
        // Every operation but `extrude` and `sheet` takes bodies.
        let sketch =
            |k: usize| values[k].sketch().ok_or_else(|| BrepError::Sketch(format!("{} takes a sketch, input {} is a body", self.kind(), k + 1)));
        let inputs: Vec<&Body> = match self {
            Self::Extrude { .. } | Self::Sheet { .. } | Self::Plate { .. } | Self::Plane { .. } => Vec::new(),
            _ => values
                .iter()
                .enumerate()
                .map(|(k, v)| v.body().ok_or_else(|| BrepError::Sketch(format!("{} takes bodies, input {} is a sketch", self.kind(), k + 1))))
                .collect::<Result<_, _>>()?,
        };
        let body = match self {
            Self::Prism { polygon, z0, z1 } => prism(feature, polygon, num(z0)?, num(z1)?)?,
            Self::Transform { translate, axis, angle_deg } => {
                let angle = num(angle_deg)?;
                let rot = if angle == 0.0 {
                    Rigid::IDENTITY
                } else if axis.dot(*axis) > 0.0 {
                    Rigid::rotation(*axis, angle.to_radians())
                } else {
                    return Err(BrepError::Degenerate("rotation about a zero axis".into()));
                };
                inputs[0].transformed(&rot.then(Rigid::translation(vec3(translate)?)))
            }
            Self::Boolean { kind, through } => boolean(feature, *kind, *through, &inputs)?,
            Self::PatternGrid { count: n, step } => {
                pattern_grid(feature, inputs[0], [count(&n[0])?, count(&n[1])?], [vec3(&step[0])?, vec3(&step[1])?])?
            }
            Self::Terrain { origin, step, count, heights, base } => {
                terrain(feature, &Grid { origin: *origin, step: *step, count: *count, heights }, *base)?
            }
            Self::Sketch { sketch, plane, face, k_factor, project } => {
                if !inputs.is_empty() && face.is_none() && project.is_empty() {
                    return Err(BrepError::Sketch(
                        "a sketch takes a body to lie on one of its faces (`face`) or to project its edges (`project`)".into(),
                    ));
                }
                let k = k_factor.as_ref().map(num).transpose()?;
                return Ok(Value::Sketch(Box::new(place(sketch, plane.as_ref(), inputs.first().copied(), face.as_deref(), k, project)?)));
            }
            Self::Unfold { face, k_factor } => return Ok(Value::Flat(unfold(inputs[0], face, num(k_factor)?)?)),
            Self::Extrude { .. } => extrude(feature, values, self)?,
            Self::Plate { thickness, flip, symmetric, profiles } => {
                let Some(t) = thickness else { return Err(BrepError::Invalid(NO_SHEET_RULE.into())) };
                let t = num(t)?;
                if !(t > 0.0 && t.is_finite()) {
                    return Err(BrepError::Degenerate(format!("a face of sheet metal needs a positive thickness, got {t}")));
                }
                let (a, b) = match (symmetric, flip) {
                    (true, _) => (-0.5 * t, 0.5 * t),
                    (false, false) => (0.0, t),
                    (false, true) => (-t, 0.0),
                };
                extrude_sketch(feature, &*chosen(feature, sketch(0)?, profiles)?, a, b)?
            }
            Self::Sheet { thickness, radius, side, z0, z1, path, start } => {
                let (Some(thickness), Some(radius)) = (thickness, radius) else { return Err(BrepError::Invalid(NO_SHEET_RULE.into())) };
                let dims = Sheet { thickness: num(thickness)?, radius: num(radius)?, side: *side };
                sheet(feature, sketch(0)?, path.as_deref(), start.as_deref(), dims, num(z0)?, num(z1)?)?
            }
            Self::Flange { faces, height, angle, radius, flip } => {
                let Some(radius) = radius else { return Err(BrepError::Invalid(NO_SHEET_RULE.into())) };
                let dims = FlangeDims { height: num(height)?, angle: num(angle)?, radius: num(radius)?, flip: *flip };
                flanges(feature, inputs[0], faces, dims)?
            }
            // Made above, before inputs are taken as bodies.
            Self::Plane { .. } => return Err(BrepError::Invalid("a plane is no body".into())),
        };
        if cfg!(debug_assertions) {
            body.validate()?;
        }
        Ok(Value::Body(body))
    }
}

/// A point a work plane goes through (D204): a vertex of an input body by its ID, a point of an
/// input sketch by its name, or, with neither, the model's origin.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Spot {
    /// The input it is of, from 0.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub input: Option<usize>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub vertex: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub point: Option<String>,
}

impl Spot {
    /// Where it is, of `values`.
    fn at(&self, values: &[&Value]) -> Result<Vec3, BrepError> {
        let of = |k: usize| {
            values.get(k).copied().ok_or_else(|| BrepError::Sketch(format!("a plane's point is of input {k}, there are {}", values.len())))
        };
        match (self.input, &self.vertex, &self.point) {
            (None, None, None) => Ok(Vec3::ZERO),
            (Some(k), Some(id), None) => {
                let body = of(k)?.body().ok_or_else(|| BrepError::Sketch(format!("a vertex of input {k}, which is no body")))?;
                let id = u128::from_str_radix(id, 16).map(TopoId).map_err(|_| BrepError::Sketch(format!("`{id}` is not a vertex ID")))?;
                let v = body
                    .vertices()
                    .binary_search_by_key(&id, |v| v.id)
                    .map_err(|_| BrepError::LostReference(format!("vertex {id:?} is not in input {k}")))?;
                Ok(body.vertices()[v].point)
            }
            (Some(k), None, Some(name)) => {
                let s = of(k)?.sketch().ok_or_else(|| BrepError::Sketch(format!("a point of input {k}, which is no sketch")))?;
                if s.developed.is_some() {
                    return Err(BrepError::Unsupported("a plane through a point of a sketch on a cylinder face".into()));
                }
                let p = s.solved.points.get(name).ok_or_else(|| BrepError::LostReference(format!("no point `{name}` in the sketch of input {k}")))?;
                let [o, x, y, _] = s.frame;
                Ok(o + x * p.x + y * p.y)
            }
            _ => Err(BrepError::Sketch("a plane's point is a `vertex` or a `point` of an `input`, or neither of them: the origin".into())),
        }
    }
}

/// The plane through three points of `values` (D204): origin the first, X toward the second,
/// normal `(b − a) × (c − a)`. Points on one line, decided exactly, have no plane.
fn through(points: &[Spot], values: &[&Value]) -> Result<SketchPlane, BrepError> {
    let [a, b, c] = points else { return Err(BrepError::Sketch(format!("a plane goes through three points, got {}", points.len()))) };
    let (a, b, c) = (a.at(values)?, b.at(values)?, c.at(values)?);
    let (u, w) = (b - a, c - a);
    if sketch::parallel(u, w) {
        return Err(BrepError::Degenerate("a plane's three points lie on one line".into()));
    }
    let x = u * (1.0 / u.dot(u).sqrt());
    let n = u.cross(w);
    Ok(SketchPlane { origin: a, x_axis: x, normal: n * (1.0 / n.dot(n).sqrt()) })
}

/// The profiles of feature `op`, a cut of sheet metal (`wall`), that lie closer to a bend than
/// [`NearBend`] allows, its inputs `values`; none for any other feature.
pub fn near_bends(feature: TopoId, values: &[&Value], op: &Op) -> Result<Vec<NearBend>, BrepError> {
    let (Op::Extrude { wall: true, profiles, .. }, [target, sketch, ..]) = (op, values) else { return Ok(Vec::new()) };
    let (Some(t), Some(s)) = (target.body(), sketch.sketch()) else { return Ok(Vec::new()) };
    bent::near_bends(feature, t, &*chosen(feature, s, profiles)?)
}

/// An extrusion (see [`Op::Extrude`]), its numbers evaluated.
fn extrude(feature: TopoId, values: &[&Value], op: &Op) -> Result<Body, BrepError> {
    let Op::Extrude { z0, z1, symmetric, mode, through, wall, profiles } = op else { return Err(BrepError::Invalid("not an extrusion".into())) };
    let (z0, z1, symmetric, mode, through) = (num(z0)?, z1.as_ref().map(num).transpose()?, *symmetric, *mode, *through);
    if *wall && !through {
        return Err(BrepError::Sketch("`wall` is a cut through one wall: with `through`".into()));
    }
    // The target body first unless the extrusion makes a new one.
    let k = usize::from(mode != Mode::New);
    if values.len() < k + 1 || values.len() > k + 2 {
        let target = if k == 0 { "" } else { "the target body, " };
        return Err(BrepError::Invalid(format!(
            "extrude `{}` takes {target}the sketch and maybe a plane, got {} input(s)",
            mode.name(),
            values.len()
        )));
    }
    let target = match k {
        0 => None,
        _ => Some(values[0].body().ok_or_else(|| BrepError::Sketch(format!("extrude `{}` takes the target body first", mode.name())))?),
    };
    let s = values[k].sketch().ok_or_else(|| BrepError::Sketch(format!("extrude takes a sketch, input {} is a body", k + 1)))?;
    let s = &*chosen(feature, s, profiles)?;
    let plane = values.get(k + 1).map(|v| v.plane());
    if through {
        let Some(t) = target.filter(|_| mode == Mode::Cut && z0 == 0.0 && z1.is_none() && !symmetric && plane.is_none()) else {
            return Err(BrepError::Sketch("`through` is a cut of the target along the sketch's normal: no `z0`, `z1`, `symmetric` or plane".into()));
        };
        // Through the walls from a face, one plate after another, made directly; through anything
        // else (a wall the profiles cross, a sketch off the body) by the general Boolean, the tool
        // stretched past the body on both sides: so no face of it lies on the sketch's (D186).
        let local = match s.face {
            // Sheet metal: through its thickness alone, made directly or refused.
            Some(_) if *wall => return bent::punch(feature, t, s),
            Some(_) => through_walls(feature, t, s),
            None if *wall => return Err(BrepError::Sketch("a cut of sheet metal takes a sketch on a face of it".into())),
            None => Err(BrepError::Unsupported("cut: the sketch lies on no face of the body".into())),
        };
        return match local {
            Err(BrepError::Unsupported(why)) => {
                let tool = extrude_sketch(feature, s, 0.0, 1.0)?;
                boolean(feature, BooleanKind::Difference, Some(s.frame[3]), &[t, &tool]).map_err(|e| match e {
                    BrepError::Unsupported(all) => BrepError::Unsupported(format!("{why}; through all of the body: {all}")),
                    e => e,
                })
            }
            made => made,
        };
    }
    let (a, b) = match (plane, z1, symmetric) {
        (None, Some(z1), false) => (z0.min(z1), z0.max(z1)),
        (None, Some(depth), true) if z0 == 0.0 => (-depth * 0.5, depth * 0.5),
        (None, Some(_), true) => return Err(BrepError::Sketch("a symmetric extrusion takes its depth as `z1`, no `z0`".into())),
        // The plane says which side: behind the sketch, the extrusion goes back.
        (Some(Some(p)), None, false) => {
            let b = depth_to(s, p)?;
            (z0.min(b), z0.max(b))
        }
        (Some(None), ..) => return Err(BrepError::Sketch("an extrusion goes up to a work plane: the last input is not one".into())),
        (None, None, _) => return Err(BrepError::Sketch("an extrusion needs `z1` or a plane to go up to".into())),
        (Some(_), ..) => return Err(BrepError::Sketch("up to a plane: no `z1`, not symmetric".into())),
    };
    let Some(t) = target else { return extrude_sketch(feature, s, a, b) };
    // From a face of the target, out of it or into it: made directly (D185 item 7), with no
    // Boolean operation meeting the face's plane.
    let on_face = s.face.is_some_and(|f| t.faces().binary_search_by_key(&f, |g| g.id).is_ok());
    if on_face && (a == 0.0 || b == 0.0) && mode != Mode::Intersect {
        let depth = a + b;
        return match (mode, depth > 0.0) {
            (Mode::Join, false) => Err(BrepError::Sketch(
                "join: the extrusion goes into the body, nothing to add; a boss goes out of the face, its depth positive".into(),
            )),
            (Mode::Cut, true) => Err(BrepError::Sketch(
                "cut: the extrusion goes out of the body, nothing to cut; a pocket goes into it, its depth negative, or cut `through`".into(),
            )),
            (Mode::Cut, false) => match from_face(feature, t, s, Reach::Depth(depth)) {
                // Into more than solid metal (a hole, walls beyond): the general Boolean
                // operation, the tool from the floor out past the body, where there is nothing
                // to cut, so no face of it lies on the sketch's (D186, D190).
                Err(BrepError::Unsupported(why)) => {
                    let out = t.vertices().iter().map(|v| (v.point - s.frame[0]).dot(s.frame[3])).fold(0.0, f64::max) - depth;
                    let tool = extrude_sketch(feature, s, depth, out)?;
                    boolean(feature, BooleanKind::Difference, None, &[t, &tool]).map_err(|e| match e {
                        BrepError::Unsupported(all) => BrepError::Unsupported(format!("{why}; as a Boolean cut: {all}")),
                        e => e,
                    })
                }
                made => made,
            },
            _ => from_face(feature, t, s, Reach::Depth(depth)),
        };
    }
    let kind = match mode {
        Mode::Join => BooleanKind::Union,
        Mode::Cut => BooleanKind::Difference,
        _ => BooleanKind::Intersection,
    };
    boolean(feature, kind, None, &[t, &extrude_sketch(feature, s, a, b)?])
}

/// `"1"`, `"2–4"` or `"at least 2"`.
pub fn describe_arity(r: RangeInclusive<usize>) -> String {
    match (*r.start(), *r.end()) {
        (a, b) if a == b => a.to_string(),
        (a, usize::MAX) => format!("at least {a}"),
        (a, b) => format!("{a}–{b}"),
    }
}
