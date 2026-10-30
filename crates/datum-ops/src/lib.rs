//! Datum operations: pure functions from parameters and input values (bodies, sketches) to a
//! new value.

mod boolean;
mod bounds;
mod copies;
mod cut;
mod pattern;
mod sheet;
mod sketch;
mod unfold;

use std::borrow::Cow;
use std::collections::BTreeMap;
use std::ops::RangeInclusive;

pub use boolean::{BooleanKind, boolean};
pub use copies::{Copies, Place};
use cut::{Reach, from_face};
use datum_brep::{Body, BrepError, Grid, TopoId, prism, terrain};
use datum_math::expr::{ExprError, Num};
use datum_math::{Rigid, Vec2, Vec3};
pub use pattern::pattern_grid;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
pub use sheet::{Sheet, Side, sheet};
pub use sketch::{Developed, ORIGIN, OriginPlane, Placed, Projection, SketchPlane, depth_to, extrude_sketch, face_plane, place, profile_regions};
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
    /// the thickness from the inside (0.5: the middle): coordinates of the flat blank. `project`
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
    /// cut from a face of the target through to the opposite face, without `z1`. Inputs: the
    /// target unless new, the sketch, then the plane if any.
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
    },
    /// A work plane (D150), `offset` along its normal from the origin plane `base` (XY when not
    /// given), from the input work plane, or from plane face `face` of the input body. Sketches
    /// lie on it (as their input), extrusions go up to it. No input, the plane, or the body.
    Plane {
        #[serde(default, skip_serializing_if = "Option::is_none")]
        base: Option<OriginPlane>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        face: Option<String>,
        #[serde(default = "zero")]
        offset: Num,
    },
    /// Sheet metal from the input sketch's path: its lines (an open chain, from point `start` or
    /// else from its end named first) given `thickness` on `side` of the path (looking along it),
    /// bent at every corner with inner `radius`, and extruded from `z0` to `z1` along the sketch's
    /// normal. One input: a sketch.
    Sheet {
        thickness: Num,
        radius: Num,
        side: Side,
        z0: Num,
        z1: Num,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        start: Option<String>,
    },
    /// The flat blank of sheet metal (a rolled sheet, a sheet from a path): the sheet's faces on
    /// the side of face `face`, plane and bent, laid flat with every cut; bends by their neutral
    /// layer at `k_factor` of the thickness (the K-factor cuts on bends were sketched with). Seen
    /// from that face's side, its corner at the origin. Exported as DXF. One input: the body.
    Unfold { face: String, k_factor: Num },
}

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
    Sketch(Placed),
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
            Self::Plane { .. } => "plane",
        }
    }

    /// Allowed numbers of inputs.
    pub fn arity(&self) -> RangeInclusive<usize> {
        match self {
            Self::Prism { .. } | Self::Terrain { .. } => 0..=0,
            Self::Sketch { .. } | Self::Plane { .. } => 0..=1,
            Self::Extrude { .. } => 1..=3,
            Self::Transform { .. } | Self::PatternGrid { .. } | Self::Unfold { .. } | Self::Sheet { .. } => 1..=1,
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
            Self::Extrude { z0, z1, symmetric, mode, through } => Self::Extrude {
                z0: r(z0, "z0")?,
                z1: z1.as_ref().map(|z| r(z, "z1")).transpose()?,
                symmetric: *symmetric,
                mode: *mode,
                through: *through,
            },
            Self::Plane { base, face, offset } => Self::Plane { base: *base, face: face.clone(), offset: r(offset, "offset")? },
            Self::Unfold { face, k_factor } => Self::Unfold { face: face.clone(), k_factor: r(k_factor, "k_factor")? },
            Self::Sheet { thickness, radius, side, z0, z1, start } => Self::Sheet {
                thickness: r(thickness, "thickness")?,
                radius: r(radius, "radius")?,
                side: *side,
                z0: r(z0, "z0")?,
                z1: r(z1, "z1")?,
                start: start.clone(),
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
            Self::Sheet { .. } => {
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
        if let Self::Plane { base, face, offset } = self {
            let from = match (values.first(), face) {
                (None, None) => base.unwrap_or_default().plane(),
                (Some(v), None) if base.is_none() => {
                    *v.plane().ok_or_else(|| BrepError::Sketch("a plane takes a work plane as its input, or a body with `face`".into()))?
                }
                (Some(v), Some(f)) if base.is_none() => {
                    face_plane(v.body().ok_or_else(|| BrepError::Sketch("a plane on a face takes the body as its input".into()))?, f)?
                }
                _ => return Err(BrepError::Sketch("a plane is offset from one thing: an origin plane (`base`), a work plane, or a face".into())),
            };
            let n = from.normal * (1.0 / from.normal.dot(from.normal).sqrt());
            return Ok(Value::Plane(SketchPlane { origin: from.origin + n * num(offset)?, ..from }));
        }
        if let (Self::Sketch { sketch, plane, face, k_factor, project }, Some(Value::Plane(on))) = (self, values.first().copied()) {
            if plane.is_some() || face.is_some() || !project.is_empty() || k_factor.is_some() {
                return Err(BrepError::Sketch("a sketch on a work plane takes no `plane`, `face`, `project` or `k_factor`".into()));
            }
            return Ok(Value::Sketch(place(sketch, Some(on), None, None, None, project)?));
        }
        // Every operation but `extrude` and `sheet` takes bodies.
        let sketch =
            |k: usize| values[k].sketch().ok_or_else(|| BrepError::Sketch(format!("{} takes a sketch, input {} is a body", self.kind(), k + 1)));
        let inputs: Vec<&Body> = match self {
            Self::Extrude { .. } | Self::Sheet { .. } | Self::Plane { .. } => Vec::new(),
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
                return Ok(Value::Sketch(place(sketch, plane.as_ref(), inputs.first().copied(), face.as_deref(), k, project)?));
            }
            Self::Unfold { face, k_factor } => return Ok(Value::Flat(unfold(inputs[0], face, num(k_factor)?)?)),
            Self::Extrude { z0, z1, symmetric, mode, through } => {
                extrude(feature, values, num(z0)?, z1.as_ref().map(num).transpose()?, *symmetric, *mode, *through)?
            }
            Self::Sheet { thickness, radius, side, z0, z1, start } => {
                let dims = Sheet { thickness: num(thickness)?, radius: num(radius)?, side: *side };
                sheet(feature, sketch(0)?, start.as_deref(), dims, num(z0)?, num(z1)?)?
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

/// An extrusion (see [`Op::Extrude`]) of numbers evaluated.
fn extrude(feature: TopoId, values: &[&Value], z0: f64, z1: Option<f64>, symmetric: bool, mode: Mode, through: bool) -> Result<Body, BrepError> {
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
    let plane = values.get(k + 1).map(|v| v.plane());
    if through {
        return match target {
            Some(t) if mode == Mode::Cut && z0 == 0.0 && z1.is_none() && !symmetric && plane.is_none() => from_face(feature, t, s, Reach::Through),
            _ => Err(BrepError::Sketch("`through` is a cut from a face of the target: no `z0`, `z1`, `symmetric` or plane".into())),
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
