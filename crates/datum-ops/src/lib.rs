//! Datum operations: pure functions from parameters and input values (bodies, sketches) to a
//! new value.

mod bent;
mod boolean;
mod bounded;
mod bounds;
mod branch;
mod component;
mod copies;
mod cut;
mod flange;
mod mirror;
mod pattern;
mod revolve;
mod rib;
mod sheet;
mod sketch;
mod trim;
mod unfold;

use std::borrow::Cow;
use std::collections::BTreeMap;
use std::ops::RangeInclusive;

pub use bent::NearBend;
pub use boolean::{BooleanKind, boolean};
pub use bounded::ChainCopy;
pub use component::{PartDef, PartFeature, PartParameter, Roles, Tool, tool};
pub use copies::{Copies, Place};
use cut::{Reach, from_face, through_walls};
use datum_brep::{Body, BrepError, Grid, TopoId, prism, terrain};
use datum_math::expr::{ExprError, Num};
use datum_math::{Rigid, Vec2, Vec3};
pub use flange::{
    BendPosition, BendRelief, Closing, Extent, FlangeDims, FlangeLoop, Limit, Notch, Relief, WallTo, Width, flange_edges, flange_ends, flange_loops,
    flanges,
};
pub use mirror::TurnAxis;
pub use pattern::pattern_grid;
use revolve::{RollDims, Span};
pub use rib::{Formed, Imprint};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
pub use sheet::{Sheet, Side, path_ends, paths, sheet};
pub use sketch::ChainPiece;
pub use sketch::{
    Developed, FlatSide, ORIGIN, OriginPlane, Placed, Projection, SketchPlane, Source, chosen, contour_at, crosses, depth_to, extrude_sketch,
    face_plane, open_curves, parallel, place, profile_curves, profile_regions,
};
pub use unfold::{Flat, FlatPiece, LineCut, POLYLINE_DEVIATION, sheet_sides, unfold, unfold_side};

/// In JSON the kind is the `"op"` field: `{"op": "prism", "polygon": [[0, 0], …], "z0": 0, "z1": 3}`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(tag = "op", rename_all = "snake_case", deny_unknown_fields)]
pub enum Op {
    /// The first input, a body, across a plane (D247): the origin plane `base`, the work plane
    /// that is the second input, or plane face `face` of the second input (none: of the first).
    /// `turn`: not reflected but the same part half a turn about that axis of the plane, through
    /// its origin. Either way the body mirrored stays. Reflected, it is a new part; turned, a live
    /// copy to see and to take geometry from, which only sketches and work planes may take.
    Mirror {
        #[serde(default, skip_serializing_if = "Option::is_none")]
        base: Option<OriginPlane>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        face: Option<String>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        turn: Option<TurnAxis>,
    },
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
    /// names geometry projected into the sketch as fixed curves and points to dimension from (see
    /// [`Projection`]): edges and vertices of a body by `id`, elements of another sketch by
    /// `element`, from the input named `from` (none: the first). Inputs: a work plane or the body
    /// holding the face first, if it lies on one; then the bodies and sketches it projects from.
    /// `flat`: the sketch lies on the flat blank of its first input, a sheet body, seen from
    /// that side (D222): its coordinates are the blank's, `k_factor` (required) the K-factor the
    /// bends are laid flat at; not with `plane` or `face`. A cut from it is the sheet cut at that
    /// place on the blank. `project` also takes the blank's outline and bend lines (`blank`).
    Sketch {
        sketch: datum_sketch::Sketch,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        plane: Option<SketchPlane>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        face: Option<String>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        k_factor: Option<Num>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        flat: Option<FlatSide>,
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
        /// punched (D196); with `join`, metal added to an edge of the blank, the wall going on in
        /// its plane (D222). With `through`.
        #[serde(default, skip_serializing_if = "std::ops::Not::not")]
        wall: bool,
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        profiles: Vec<String>,
        /// A cut of sheet metal on its blank only (D226): the body stays as it is, its blank and
        /// DXF take the cut (see [`blank_cut`]). With `wall`, `mode: cut`.
        #[serde(default, skip_serializing_if = "std::ops::Not::not")]
        blank_only: bool,
        /// How wide the laser cuts, from the part's sheet rule: with `wall`, `mode: cut`, an open line
        /// of the sketch is a cut this wide in the part and one line in the DXF (D235).
        #[serde(default, skip_serializing_if = "Option::is_none")]
        kerf: Option<Num>,
    },
    /// A work plane (D150), `offset` along its normal from the origin plane `base` (XY when not
    /// given), from the input work plane, from plane face `face` of the input body, or through
    /// three `points` (D204): its origin the first, its X toward the second, its normal by the
    /// right hand from the second to the third. Sketches lie on it (as their input), extrusions go
    /// up to it. With `line`, a line of the sketch that is its one input, the plane goes through that
    /// line, square to the sketch; its origin the line's start, its X along the line. No input, the
    /// plane, the body, the sketch, or what the points are of (up to three).
    Plane {
        #[serde(default, skip_serializing_if = "Option::is_none")]
        base: Option<OriginPlane>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        face: Option<String>,
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        points: Vec<Spot>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        line: Option<LineOf>,
        #[serde(default = "zero")]
        offset: Num,
    },
    /// Flanges (M8, D205) on edges `edges` of the input sheet metal: an end's long edge, or a
    /// wall's edge on the sheet's side face (along the wall, outward). A bend of inner `radius`
    /// (the part's rule's, put here when the model is built) turning by `angle` degrees (an end:
    /// toward the sheet's front side; a wall: outward; `flip`: the other way), and a wall beyond
    /// it, `height` from the outer surface of the sheet square to it, up to the flange's highest
    /// point (D229; `height_along`: along the flange from where its outer surface meets the
    /// sheet's, D205). The input: the sheet, then the work planes named in `to`.
    Flange {
        edges: Vec<String>,
        height: Num,
        /// The height along the flange (D205), not square to the sheet (D229).
        #[serde(default, skip_serializing_if = "std::ops::Not::not")]
        height_along: bool,
        #[serde(default = "right_angle")]
        angle: Num,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        radius: Option<Num>,
        #[serde(default, skip_serializing_if = "std::ops::Not::not")]
        flip: bool,
        /// The relief where flanges meet at a corner (D229): square, or round, a circle of
        /// `relief_diameter` about where their bend lines meet, cut through the blank laid at
        /// `k_factor`. Both the part's sheet rule's, put here when the model is built.
        #[serde(default, skip_serializing_if = "Relief::is_square")]
        relief: Relief,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        relief_diameter: Option<Num>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        k_factor: Option<Num>,
        /// How walls of flanges meeting at a corner close it (D229), `corner_gap` apart (the
        /// part's sheet rule's).
        #[serde(default, skip_serializing_if = "Closing::is_open")]
        corner: Closing,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        corner_gap: Option<Num>,
        /// In a mitre with a neighbour of another angle (D238): the lower wall goes up to the top
        /// of the higher.
        #[serde(default, skip_serializing_if = "std::ops::Not::not")]
        level_walls: bool,
        /// How much of each edge a flange takes (D229): all of it by default.
        #[serde(default, skip_serializing_if = "Extent::is_edge")]
        extent: Extent<Num>,
        /// The relief cut into the sheet beside the bend of a flange narrower than its edge (D229),
        /// `bend_relief_width` along the edge and `bend_relief_depth` into the sheet: the rule's.
        #[serde(default, skip_serializing_if = "BendRelief::is_none")]
        bend_relief: BendRelief,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        bend_relief_width: Option<Num>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        bend_relief_depth: Option<Num>,
        /// Where the bend lies against the edge (D229): beyond it (default) or flush.
        #[serde(default, skip_serializing_if = "BendPosition::is_outside")]
        bend_position: BendPosition,
        /// The work planes the wall's sides and free edge go to (D229), a gap short of them: the
        /// wall grows or is cut square to itself, the bend stays. The free edge's plane takes the
        /// place of `height`. Each plane an input of the flange, so that two parts may meet at one
        /// joint plane, not knowing of each other.
        #[serde(default, skip_serializing_if = "WallTo::is_empty")]
        to: WallTo<PlaneTo>,
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
    /// The closed profile of the input sketch (curves `profiles` choose it, D191) turned about
    /// the sketch's line `axis` by `angle` degrees (360, the default: a full turn), a new body
    /// (D206). Seen from the axis line's end `to`, the turn is counterclockwise. An edge parallel
    /// to the axis turns into a cylinder, an oblique one into a cone, one square to it into a
    /// plane, an arc into a torus or, centred on the axis, a sphere; an oblique edge reaching the
    /// axis (a cone to a point) is refused. One input: a sketch.
    Revolve {
        axis: String,
        #[serde(default = "full_turn")]
        angle: Num,
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        profiles: Vec<String>,
    },
    /// Rolled sheet metal (D207): line `path` of the input sketch (none: its only line other than
    /// the axis and construction lines) turned about the sketch's line `axis`: parallel to it, a
    /// shell; oblique, a cone. The sheet lies on `side` of the line looking along it, its ends
    /// square to it. `angle` degrees (360, the default: a whole turn) it goes round: a whole
    /// turn leaves `gap` millimetres open at the seam, along the neutral layer at the larger end,
    /// half each side of the sketch's plane (none or 0 is refused: the sheet would close and not
    /// unroll); a piece starts at the sketch's plane and turns as `revolve` does, `gap` unused
    /// (D213). The thickness and K-factor are the part's sheet rule's, put here when the
    /// model is built. One input: a sketch.
    Rolled {
        axis: String,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        path: Option<String>,
        side: Side,
        #[serde(default = "full_turn")]
        angle: Num,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        gap: Option<Num>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        thickness: Option<Num>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        k_factor: Option<Num>,
    },
    /// Rolled sheet metal (a shell) trimmed by a work plane (D211): what lies on the side the
    /// plane's normal points to goes (`flip`: the other side). The cut follows where the plane
    /// meets the neutral layer at `k_factor` (the part's rule's, put here when the model is
    /// built), its wall square to the sheet. Inputs: the body, then the work plane.
    Trim {
        #[serde(default, skip_serializing_if = "std::ops::Not::not")]
        flip: bool,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        k_factor: Option<Num>,
    },
    /// The end of a branch pipe cut to sit on a main one (D227): where the branch's neutral layer
    /// at `k_factor` (its part's rule's, put here when the model is built) meets the main pipe's
    /// outer cylinder, the end beyond it gone, the wall square to the sheet. Both are rolled
    /// shells. Inputs: the branch, then the main pipe.
    Saddle {
        #[serde(default, skip_serializing_if = "Option::is_none")]
        k_factor: Option<Num>,
    },
    /// The hole in a main pipe for a branch set on it (D227): where the branch's bore meets the
    /// main pipe's outer cylinder, cut square to the sheet; laid in the blank at the main pipe's
    /// `k_factor`. Both are rolled shells. Inputs: the main pipe, then the branch.
    BranchHole {
        #[serde(default, skip_serializing_if = "Option::is_none")]
        k_factor: Option<Num>,
    },
    /// Ribs rolled along the open chains of a sketch on a flat wall of sheet metal (D250): by the
    /// tool `tool` of the library (its `diameter`, `depth`, `shoulder` and `min_length` put here
    /// when the model is built). The rib rises out of the sketch's face, or into it (`flip`). The
    /// sheet's blank does not change. Inputs: the sheet, then the sketch.
    Rib {
        tool: String,
        #[serde(default, skip_serializing_if = "std::ops::Not::not")]
        flip: bool,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        diameter: Option<Num>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        depth: Option<Num>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        shoulder: Option<Num>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        min_length: Option<Num>,
    },
    /// A library part placed in the part (D260, D261): `part` is the tool's definition, written
    /// into the model — its features build its body, the `cut` role's extrusion cuts the sheet
    /// with its sketch's profile. `params` give the definition's parameters other values;
    /// `library` and `group` say which library element it came from (to update it from the
    /// library later), the model is built without the library. A `line` placement turns the
    /// tool's `axis` along a line of the input sketch and feeds the line's length to the
    /// parameter `fit`; a `point` placement puts it at a point of the sketch, its axis unturned.
    /// The profile, laid on the wall the placement's sketch lies on, punches one wall through as
    /// a press punch does (D196); the sheet and its blank take the cut, nothing else does.
    /// Inputs: the sheet, then the sketch the placements name. Refused: a parameter the part has
    /// not, no placements, a line or a point the sketch has not, a line placement of a part
    /// without an `axis`, a `cut` role that is no extrusion of a sketch, a profile off the wall's
    /// plane; a part that forms or hinges the sheet comes next.
    Component {
        part: PartDef,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        library: Option<String>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        group: Option<String>,
        #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
        params: BTreeMap<String, Num>,
        placements: Vec<Placement>,
    },
    /// The flat blank of sheet metal (a rolled sheet, a sheet from a path): the sheet's faces on
    /// the side of face `face`, plane and bent, laid flat with every cut; bends by their neutral
    /// layer at `k_factor` of the thickness (the K-factor cuts on bends were sketched with). Seen
    /// from that face's side, its corner at the origin. Exported as DXF. One input: the body.
    Unfold { face: String, k_factor: Num },
}

/// Said when sheet metal is built with no rule giving its thickness.
const NO_SHEET_RULE: &str = "sheet metal takes its thickness and bend radius from the part's sheet rule: give the part one";

/// Said when rolled sheet metal is built with no rule giving its thickness and K-factor.
const NO_ROLL_RULE: &str =
    "rolled sheet metal takes its thickness and K-factor (for the gap at its seam) from the part's sheet rule: give the part one with both";

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

/// A work plane an edge of a flange's wall goes to (D229), `gap` short of it: its name among the
/// flange's inputs.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct PlaneTo {
    pub plane: String,
    #[serde(default = "zero")]
    pub gap: Num,
}

/// A number of a resolved operation (see [`Op::resolve`]).
/// A sketch feature's value (see [`Op::Sketch`]). Its first input is what it lies on, if it lies
/// on a work plane or on a face (`face`); every other input is one its projections take
/// geometry from, by name (`from`, D214); a projection without `from` takes the first.
#[allow(clippy::too_many_arguments)]
fn sketch_value(
    sketch: &datum_sketch::Sketch,
    plane: Option<&SketchPlane>,
    face: Option<&str>,
    k_factor: Option<&Num>,
    flat: Option<FlatSide>,
    project: &BTreeMap<String, Projection>,
    values: &[&Value],
    names: &[&str],
) -> Result<Placed, BrepError> {
    // What it lies on, and whether the first input is that.
    let (on, host, lies_on) = match values.first() {
        Some(Value::Plane(p)) => {
            if plane.is_some() || face.is_some() || k_factor.is_some() || flat.is_some() {
                return Err(BrepError::Sketch("a sketch on a work plane takes no `plane`, `face`, `k_factor` or `flat`".into()));
            }
            (Some(p), None, 1)
        }
        Some(v) if face.is_some() || flat.is_some() => {
            (plane, Some(v.body().ok_or_else(|| BrepError::Sketch("a sketch on a face or the blank takes the body as its first input".into()))?), 1)
        }
        _ => (plane, None, 0),
    };
    // The sheet's blank, if the sketch lies on it.
    let blank = match (flat, host) {
        (Some(side), Some(body)) => {
            if plane.is_some() || face.is_some() {
                return Err(BrepError::Sketch("a sketch on the blank takes no plane or face".into()));
            }
            let k = k_factor.ok_or_else(|| BrepError::Sketch("a sketch on the blank needs the sheet's k_factor".into()))?;
            Some(unfold_side(body, side == FlatSide::Back, num(k)?)?)
        }
        _ => None,
    };
    let input = |from: Option<&str>| match from {
        None => Ok(0),
        Some(n) => names.iter().position(|m| *m == n).ok_or_else(|| BrepError::Sketch(format!("a projection is `from` `{n}`, not an input"))),
    };
    for k in lies_on..values.len() {
        if !project.values().any(|p| input(p.from.as_deref()).is_ok_and(|i| i == k)) {
            return Err(BrepError::Sketch(format!(
                "input {} of a sketch is neither what it lies on (a work plane, a body with `face`) nor projected from (`project`)",
                k + 1
            )));
        }
    }
    let source = |from: Option<&str>| match values.get(input(from)?) {
        Some(Value::Body(b)) => Ok(Source::Body(b)),
        Some(Value::Formed(f)) => Ok(Source::Body(&f.body)),
        Some(Value::Sketch(s)) => Ok(Source::Sketch(s)),
        Some(Value::Plane(_)) => Err(BrepError::Sketch("a projection names the input it takes geometry from (`from`), not the work plane".into())),
        Some(_) => Err(BrepError::Sketch("a projection takes geometry from a body or a sketch".into())),
        None => Err(BrepError::Sketch("a projection takes geometry from an input of the sketch".into())),
    };
    place(sketch, on, host, face, k_factor.map(num).transpose()?, project, &source, blank.as_ref())
}

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
    /// Boxed, as a sketch: a blank is larger than a body.
    Flat(Box<Flat>),
    /// Sheet metal with ribs (D250). Boxed, as it holds two bodies.
    Formed(Box<rib::Formed>),
    /// A work plane: its origin, X axis and normal, as a sketch plane.
    Plane(SketchPlane),
}

impl Value {
    /// Content hash: equal values hash equal (a body's Merkle hash, a sketch's placed geometry).
    pub fn hash(&self) -> [u8; 32] {
        match self {
            Self::Body(b) => b.hash(),
            Self::Formed(f) => f.body.hash(),
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
            Self::Formed(f) => Some(&f.body),
            _ => None,
        }
    }

    /// The body sheet metal is laid flat from: without its ribs (D250).
    pub fn blank(&self) -> Option<&Body> {
        match self {
            Self::Formed(f) => Some(&f.blank),
            _ => self.body(),
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

fn full_turn() -> Num {
    Num::Value(360.0)
}

fn z_axis() -> Vec3 {
    Vec3::new(0.0, 0.0, 1.0)
}

impl Op {
    pub fn kind(&self) -> &'static str {
        match self {
            Self::Mirror { .. } => "mirror",
            Self::Prism { .. } => "prism",
            Self::Transform { .. } => "transform",
            Self::Boolean { .. } => "boolean",
            Self::PatternGrid { .. } => "pattern_grid",
            Self::Terrain { .. } => "terrain",
            Self::Sketch { .. } => "sketch",
            Self::Extrude { .. } => "extrude",
            Self::Revolve { .. } => "revolve",
            Self::Rolled { .. } => "rolled",
            Self::Trim { .. } => "trim",
            Self::Saddle { .. } => "saddle",
            Self::BranchHole { .. } => "branch_hole",
            Self::Unfold { .. } => "unfold",
            Self::Rib { .. } => "rib",
            Self::Component { .. } => "component",
            Self::Sheet { .. } => "sheet",
            Self::Plate { .. } => "plate",
            Self::Flange { .. } => "flange",
            Self::Plane { .. } => "plane",
        }
    }

    /// Whether it makes a turned copy of a part (D247), which only sketches and work planes take.
    pub fn is_turned_copy(&self) -> bool {
        matches!(self, Self::Mirror { turn: Some(_), .. })
    }

    /// Whether it may take a turned copy as an input.
    pub fn takes_turned_copy(&self) -> bool {
        matches!(self, Self::Sketch { .. } | Self::Plane { .. })
    }

    /// Allowed numbers of inputs.
    pub fn arity(&self) -> RangeInclusive<usize> {
        match self {
            Self::Prism { .. } | Self::Terrain { .. } => 0..=0,
            Self::Sketch { .. } => 0..=usize::MAX,
            Self::Plane { .. } => 0..=3,
            Self::Extrude { .. } => 1..=3,
            Self::Mirror { .. } => 1..=2,
            Self::Transform { .. }
            | Self::PatternGrid { .. }
            | Self::Unfold { .. }
            | Self::Sheet { .. }
            | Self::Plate { .. }
            | Self::Revolve { .. }
            | Self::Rolled { .. } => 1..=1,
            Self::Flange { .. } => 1..=4,
            Self::Boolean { .. } => 2..=usize::MAX,
            Self::Trim { .. } | Self::Saddle { .. } | Self::BranchHole { .. } | Self::Rib { .. } | Self::Component { .. } => 2..=2,
        }
    }

    /// Whether the operation works on sheet metal without its ribs (D250): the blank is laid
    /// from it, and a flange or a cut changes it, the ribs laid again after.
    fn reads_blank(&self) -> bool {
        matches!(
            self,
            Self::Flange { .. }
                | Self::Extrude { .. }
                | Self::Trim { .. }
                | Self::Saddle { .. }
                | Self::BranchHole { .. }
                | Self::Unfold { .. }
                | Self::Sketch { flat: Some(_), .. }
        )
    }

    /// The operation with every number evaluated, parameter values coming from `lookup` (D57).
    /// An error names the field and says what is wrong with its expression.
    pub fn resolve(&self, lookup: &impl Fn(&str) -> Result<f64, ExprError>) -> Result<Cow<'_, Op>, String> {
        let r = |n: &Num, field: &str| n.resolve(lookup).map(Num::Value).map_err(|e| format!("`{field}`: {e}"));
        let r3 = |v: &[Num; 3], field: &str| -> Result<[Num; 3], String> { Ok([r(&v[0], field)?, r(&v[1], field)?, r(&v[2], field)?]) };
        Ok(Cow::Owned(match self {
            Self::Mirror { .. } => self.clone(),
            Self::Prism { polygon, z0, z1 } => Self::Prism { polygon: polygon.clone(), z0: r(z0, "z0")?, z1: r(z1, "z1")? },
            Self::Transform { translate, axis, angle_deg } => {
                Self::Transform { translate: r3(translate, "translate")?, axis: *axis, angle_deg: r(angle_deg, "angle_deg")? }
            }
            Self::PatternGrid { count, step } => {
                Self::PatternGrid { count: [r(&count[0], "count")?, r(&count[1], "count")?], step: [r3(&step[0], "step")?, r3(&step[1], "step")?] }
            }
            Self::Sketch { sketch, plane, face, k_factor, flat, project } => Self::Sketch {
                sketch: sketch.resolve(lookup)?,
                plane: *plane,
                face: face.clone(),
                k_factor: k_factor.as_ref().map(|k| r(k, "k_factor")).transpose()?,
                flat: *flat,
                project: project.clone(),
            },
            Self::Extrude { z0, z1, symmetric, mode, through, wall, profiles, blank_only, kerf } => Self::Extrude {
                z0: r(z0, "z0")?,
                z1: z1.as_ref().map(|z| r(z, "z1")).transpose()?,
                symmetric: *symmetric,
                mode: *mode,
                through: *through,
                wall: *wall,
                profiles: profiles.clone(),
                blank_only: *blank_only,
                kerf: kerf.as_ref().map(|n| r(n, "kerf")).transpose()?,
            },
            Self::Plane { base, face, points, line, offset } => {
                Self::Plane { base: *base, face: face.clone(), points: points.clone(), line: line.clone(), offset: r(offset, "offset")? }
            }
            Self::Unfold { face, k_factor } => Self::Unfold { face: face.clone(), k_factor: r(k_factor, "k_factor")? },
            Self::Rib { tool, flip, diameter, depth, shoulder, min_length } => {
                let opt = |n: &Option<Num>, field: &str| n.as_ref().map(|n| r(n, field)).transpose();
                Self::Rib {
                    tool: tool.clone(),
                    flip: *flip,
                    diameter: opt(diameter, "diameter")?,
                    depth: opt(depth, "depth")?,
                    shoulder: opt(shoulder, "shoulder")?,
                    min_length: opt(min_length, "min_length")?,
                }
            }
            Self::Trim { flip, k_factor } => Self::Trim { flip: *flip, k_factor: k_factor.as_ref().map(|k| r(k, "k_factor")).transpose()? },
            Self::Saddle { k_factor } => Self::Saddle { k_factor: k_factor.as_ref().map(|k| r(k, "k_factor")).transpose()? },
            Self::BranchHole { k_factor } => Self::BranchHole { k_factor: k_factor.as_ref().map(|k| r(k, "k_factor")).transpose()? },
            Self::Revolve { axis, angle, profiles } => Self::Revolve { axis: axis.clone(), angle: r(angle, "angle")?, profiles: profiles.clone() },
            Self::Rolled { axis, path, side, angle, gap, thickness, k_factor } => Self::Rolled {
                axis: axis.clone(),
                path: path.clone(),
                side: *side,
                angle: r(angle, "angle")?,
                gap: gap.as_ref().map(|g| r(g, "gap")).transpose()?,
                thickness: thickness.as_ref().map(|n| r(n, "thickness")).transpose()?,
                k_factor: k_factor.as_ref().map(|n| r(n, "k_factor")).transpose()?,
            },
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
            Self::Flange {
                edges,
                height,
                height_along,
                angle,
                radius,
                flip,
                relief,
                relief_diameter,
                k_factor,
                corner,
                corner_gap,
                level_walls,
                extent,
                bend_relief,
                bend_relief_width,
                bend_relief_depth,
                bend_position,
                to,
            } => Self::Flange {
                edges: edges.clone(),
                height: r(height, "height")?,
                height_along: *height_along,
                angle: r(angle, "angle")?,
                radius: radius.as_ref().map(|n| r(n, "radius")).transpose()?,
                flip: *flip,
                relief: *relief,
                relief_diameter: relief_diameter.as_ref().map(|n| r(n, "relief_diameter")).transpose()?,
                k_factor: k_factor.as_ref().map(|n| r(n, "k_factor")).transpose()?,
                corner: *corner,
                corner_gap: corner_gap.as_ref().map(|n| r(n, "corner_gap")).transpose()?,
                level_walls: *level_walls,
                extent: extent.map(|n, field| r(n, field))?,
                bend_relief: *bend_relief,
                bend_relief_width: bend_relief_width.as_ref().map(|n| r(n, "bend_relief_width")).transpose()?,
                bend_relief_depth: bend_relief_depth.as_ref().map(|n| r(n, "bend_relief_depth")).transpose()?,
                bend_position: *bend_position,
                to: to.map(|p| Ok::<_, String>(PlaneTo { plane: p.plane.clone(), gap: r(&p.gap, "to: gap")? }))?,
            },
            // The definition keeps its own numbers: they resolve inside the tool, against its
            // parameters (D260). What the occurrence gives them, and where it is placed, are the
            // model's numbers, as everywhere else.
            Self::Component { part, library, group, params, placements } => Self::Component {
                part: part.clone(),
                library: library.clone(),
                group: group.clone(),
                params: params.iter().map(|(name, n)| Ok((name.clone(), r(n, name)?))).collect::<Result<_, String>>()?,
                placements: placements.iter().map(|p| p.resolved(lookup)).collect::<Result<_, String>>()?,
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
            Self::Mirror { .. }
            | Self::Sheet { .. }
            | Self::Plate { .. }
            | Self::Flange { .. }
            | Self::Revolve { .. }
            | Self::Rolled { .. }
            | Self::Trim { .. }
            | Self::Saddle { .. }
            | Self::BranchHole { .. }
            | Self::Rib { .. }
            | Self::Component { .. } => {
                h.update(format!("{self:?}").as_bytes());
            }
        }
    }

    /// Evaluates the operation for the feature with ID `feature`. In debug builds the result is
    /// validated here, once for every operation.
    pub fn eval(&self, feature: TopoId, values: &[&Value]) -> Result<Value, BrepError> {
        self.eval_named(feature, values, &[])
    }

    /// [`Op::eval`] with the inputs' names, which a sketch's projections take their geometry
    /// from by (`from`, D214).
    pub fn eval_named(&self, feature: TopoId, values: &[&Value], names: &[&str]) -> Result<Value, BrepError> {
        if !self.arity().contains(&values.len()) {
            return Err(BrepError::Invalid(format!("{} takes {} input(s), got {}", self.kind(), describe_arity(self.arity()), values.len())));
        }
        if let (Some(Value::Formed(f)), true) = (values.first(), self.reads_blank()) {
            let blank = Value::Body(f.blank.clone());
            let mut plain = values.to_vec();
            plain[0] = &blank;
            return match self.eval_named(feature, &plain, names)? {
                Value::Body(body) => {
                    let mut laid = body.clone();
                    for i in &f.imprints {
                        laid = rib::imprint(&laid, i)?;
                    }
                    validated(&laid)?;
                    Ok(Value::Formed(Box::new(rib::Formed { body: laid, blank: body, imprints: f.imprints.clone() })))
                }
                other => Ok(other),
            };
        }
        // A work plane, and a sketch on one, take no body.
        if let Self::Plane { base, face, points, line, offset } = self {
            let from = match (line, values.first(), face) {
                (Some(_), ..) if base.is_some() || face.is_some() || !points.is_empty() || values.len() != 1 => {
                    return Err(BrepError::Sketch(
                        "a plane through a line stands square to the line's sketch, its one input: it takes no `base`, `face` or `points`".into(),
                    ));
                }
                (Some(l), ..) => square_to_sketch(l, values)?,
                _ if !points.is_empty() => {
                    if base.is_some() || face.is_some() {
                        return Err(BrepError::Sketch("a plane through points takes no `base` or `face`".into()));
                    }
                    through(points, values)?
                }
                (_, None, None) => base.unwrap_or_default().plane(),
                (_, Some(v), None) if base.is_none() => {
                    *v.plane().ok_or_else(|| BrepError::Sketch("a plane takes a work plane as its input, or a body with `face`".into()))?
                }
                (_, Some(v), Some(f)) if base.is_none() => {
                    face_plane(v.body().ok_or_else(|| BrepError::Sketch("a plane on a face takes the body as its input".into()))?, f)?
                }
                _ if values.len() > 1 => return Err(BrepError::Sketch("a plane takes more than one input only through `points`".into())),
                _ => return Err(BrepError::Sketch("a plane is offset from one thing: an origin plane (`base`), a work plane, or a face".into())),
            };
            let n = from.normal * (1.0 / from.normal.dot(from.normal).sqrt());
            return Ok(Value::Plane(SketchPlane { origin: from.origin + n * num(offset)?, ..from }));
        }
        // Every operation but those of sketches takes bodies.
        let sketch =
            |k: usize| values[k].sketch().ok_or_else(|| BrepError::Sketch(format!("{} takes a sketch, input {} is a body", self.kind(), k + 1)));
        let inputs: Vec<&Body> = match self {
            Self::Extrude { .. }
            | Self::Sketch { .. }
            | Self::Sheet { .. }
            | Self::Plate { .. }
            | Self::Plane { .. }
            | Self::Revolve { .. }
            | Self::Rolled { .. }
            | Self::Mirror { .. }
            | Self::Rib { .. }
            | Self::Trim { .. }
            | Self::Component { .. } => Vec::new(),
            // The sheet; its work planes are read by name.
            Self::Flange { .. } => {
                vec![values[0].body().ok_or_else(|| BrepError::Sketch("flange takes bodies, input 1 is a sketch".into()))?]
            }
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
            Self::Mirror { base, face, turn } => {
                let body = values[0].body().ok_or_else(|| BrepError::Sketch("a mirror takes a body".into()))?;
                let plane = match (base, face, values.get(1)) {
                    (Some(b), None, None) => b.plane(),
                    (None, None, Some(v)) => *v
                        .plane()
                        .ok_or_else(|| BrepError::Sketch("a mirror takes a work plane as its second input, or a body with `face`".into()))?,
                    (None, Some(f), holder) => face_plane(
                        holder
                            .map_or(Some(body), |v| v.body())
                            .ok_or_else(|| BrepError::Sketch("a mirror across a face takes the body of the face".into()))?,
                        f,
                    )?,
                    _ => return Err(BrepError::Sketch("a mirror goes across one thing: an origin plane (`base`), a work plane, or a face".into())),
                };
                mirror::mirror(body, &plane, *turn)?
            }
            Self::Boolean { kind, through } => boolean(feature, *kind, *through, &inputs)?,
            Self::PatternGrid { count: n, step } => {
                pattern_grid(feature, inputs[0], [count(&n[0])?, count(&n[1])?], [vec3(&step[0])?, vec3(&step[1])?])?
            }
            Self::Terrain { origin, step, count, heights, base } => {
                terrain(feature, &Grid { origin: *origin, step: *step, count: *count, heights }, *base)?
            }
            Self::Sketch { sketch, plane, face, k_factor, flat, project } => {
                return Ok(Value::Sketch(Box::new(sketch_value(
                    sketch,
                    plane.as_ref(),
                    face.as_deref(),
                    k_factor.as_ref(),
                    *flat,
                    project,
                    values,
                    names,
                )?)));
            }
            Self::Unfold { face, k_factor } => return Ok(Value::Flat(Box::new(unfold(inputs[0], face, num(k_factor)?)?))),
            Self::Extrude { .. } => extrude(feature, values, self)?,
            Self::Rib { flip, diameter, depth, shoulder, min_length, tool } => {
                let (Some(d), Some(h), Some(s)) = (diameter, depth, shoulder) else {
                    return Err(BrepError::Sketch(format!("rib: no tool `{tool}` in the library: add it to the tools, or choose one that is there")));
                };
                let tool =
                    rib::RibTool { radius: 0.5 * num(d)?, depth: num(h)?, shoulder: num(s)?, min_length: min_length.as_ref().map_or(Ok(0.0), num)? };
                let (sheet, s) = (values[0], sketch(1)?);
                let body = sheet.body().ok_or_else(|| BrepError::Sketch("a rib takes the sheet, then a sketch".into()))?;
                let i = rib::imprint_of(feature, s, tool, *flip)?;
                let laid = rib::imprint(body, &i)?;
                validated(&laid)?;
                let (blank, mut imprints) = match sheet {
                    Value::Formed(f) => (f.blank.clone(), f.imprints.clone()),
                    _ => (body.clone(), Vec::new()),
                };
                imprints.push(i);
                return Ok(Value::Formed(Box::new(rib::Formed { body: laid, blank, imprints })));
            }
            Self::Revolve { axis, angle, profiles } => revolve::revolve_sketch(feature, sketch(0)?, axis, num(angle)?, profiles)?,
            Self::Component { part, params, placements, .. } => {
                return component::cut(feature, part, params, placements, values);
            }
            Self::Trim { flip, k_factor } => {
                let Some(k) = k_factor else { return Err(BrepError::Invalid(NO_ROLL_RULE.into())) };
                let body = values[0].body().ok_or_else(|| BrepError::Sketch("trim takes a body, then a work plane".into()))?;
                let plane = values[1].plane().ok_or_else(|| BrepError::Sketch("trim takes a body, then a work plane".into()))?;
                trim::trim(feature, body, plane, *flip, num(k)?)?
            }
            Self::Saddle { k_factor } | Self::BranchHole { k_factor } => {
                let Some(k) = k_factor else { return Err(BrepError::Invalid(NO_ROLL_RULE.into())) };
                let k = num(k)?;
                match self {
                    Self::Saddle { .. } => branch::saddle(feature, inputs[0], inputs[1], k)?,
                    _ => branch::hole(feature, inputs[0], inputs[1], k)?,
                }
            }
            Self::Rolled { axis, path, side, angle, gap, thickness, k_factor } => {
                let (Some(thickness), Some(k)) = (thickness, k_factor) else { return Err(BrepError::Invalid(NO_ROLL_RULE.into())) };
                let span = match (num(angle)?, gap) {
                    (a, _) if a > 360.0 => return Err(BrepError::Degenerate(format!("rolled sheet goes round at most 360°, got {a}°"))),
                    (a, _) if a < 360.0 => Span::Part { degrees: a },
                    (_, Some(g)) => Span::Full { gap: num(g)? },
                    (_, None) => {
                        return Err(BrepError::Degenerate(
                            "a whole turn of rolled sheet needs a `gap` at its seam, or it closes and cannot be unrolled".into(),
                        ));
                    }
                };
                let dims = RollDims { thickness: num(thickness)?, k_factor: num(k)?, side: *side, span };
                revolve::rolled(feature, sketch(0)?, axis, path.as_deref(), dims)?
            }
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
            Self::Flange {
                edges,
                height,
                height_along,
                angle,
                radius,
                flip,
                relief,
                relief_diameter,
                k_factor,
                corner,
                corner_gap,
                level_walls,
                extent,
                bend_relief,
                bend_relief_width,
                bend_relief_depth,
                bend_position,
                to,
            } => {
                let Some(radius) = radius else { return Err(BrepError::Invalid(NO_SHEET_RULE.into())) };
                let round = match relief {
                    Relief::Square => None,
                    Relief::Round => {
                        let (Some(d), Some(k)) = (relief_diameter, k_factor) else {
                            return Err(BrepError::Invalid(
                                "a round relief takes its diameter and the K-factor from the part's sheet rule: it has none".into(),
                            ));
                        };
                        Some((num(d)?, num(k)?))
                    }
                };
                let gap = corner_gap.as_ref().map(num).transpose()?;
                let to = to.map(|p| {
                    let plane = names.iter().position(|n| *n == p.plane).and_then(|k| values.get(k)?.plane());
                    let plane =
                        plane.ok_or_else(|| BrepError::Sketch(format!("a flange goes to `{}`, not a work plane among its inputs", p.plane)))?;
                    let gap = num(&p.gap)?;
                    if !(gap >= 0.0 && gap.is_finite()) {
                        return Err(BrepError::Degenerate(format!("a flange's gap to a plane is not below zero, got {gap}")));
                    }
                    Ok(Limit { at: plane.origin, normal: plane.normal, gap })
                })?;
                let dims = FlangeDims {
                    height: num(height)?,
                    along: *height_along,
                    angle: num(angle)?,
                    radius: num(radius)?,
                    flip: *flip,
                    round,
                    corner: *corner,
                    gap,
                    width: extent.map(|n, _| num(n))?,
                    flush: *bend_position == BendPosition::Flush,
                    to,
                    level: *level_walls,
                    notch: match (bend_relief, bend_relief_width, bend_relief_depth) {
                        (BendRelief::None, ..) => None,
                        (shape, Some(w), Some(d)) => Some(Notch { round: *shape == BendRelief::Round, width: num(w)?, depth: num(d)? }),
                        _ => {
                            return Err(BrepError::Invalid("a bend relief takes its width and depth from the part's sheet rule: it has none".into()));
                        }
                    },
                };
                flanges(feature, inputs[0], edges, dims)?
            }
            // Made above, before inputs are taken as bodies.
            Self::Plane { .. } => return Err(BrepError::Invalid("a plane is no body".into())),
        };
        validated(&body)?;
        Ok(Value::Body(body))
    }
}

/// In debug builds, every operation's result is validated.
fn validated(body: &Body) -> Result<(), BrepError> {
    if cfg!(debug_assertions) {
        body.validate()?;
    }
    Ok(())
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
                sketch_point(s, name, k)
            }
            _ => Err(BrepError::Sketch("a plane's point is a `vertex` or a `point` of an `input`, or neither of them: the origin".into())),
        }
    }
}

/// A line of the sketch that is input `input` of a work plane.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct LineOf {
    pub input: usize,
    pub line: String,
}

/// Where a component is placed (D260, D261): along a line of an input sketch, or at a point of
/// one. The line is the tool's axis and its length feeds `fit`; the point puts it where it is,
/// its axis unturned.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(tag = "placement", rename_all = "snake_case", deny_unknown_fields)]
pub enum Placement {
    Line(LineOf),
    Point(At),
}

impl Placement {
    /// The placement with its numbers evaluated, a point's `at` by `lookup`.
    fn resolved(&self, lookup: &impl Fn(&str) -> Result<f64, ExprError>) -> Result<Self, String> {
        let r = |n: &Num, field: &str| n.resolve(lookup).map(Num::Value).map_err(|e| format!("`{field}`: {e}"));
        Ok(match self {
            Self::Line(l) => Self::Line(l.clone()),
            Self::Point(a) => Self::Point(At {
                input: a.input,
                point: a.point.clone(),
                at: a.at.as_ref().map(|[x, y]| Ok::<[Num; 2], String>([r(x, "at")?, r(y, "at")?])).transpose()?,
            }),
        })
    }
}

/// A point of the sketch that is input `input`: by the name of one of its points, or by its
/// coordinates `at` `[x, y]`; exactly one of the two.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct At {
    pub input: usize,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub point: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub at: Option<[Num; 2]>,
}

/// The sketch that is input `k` of `what`, or why it is none.
fn input_sketch<'a>(values: &'a [&Value], k: usize, what: &str) -> Result<&'a sketch::Placed, BrepError> {
    values.get(k).and_then(|v| v.sketch()).ok_or_else(|| BrepError::Sketch(format!("{what} is of input {k}, which is no sketch")))
}

/// The ends of line `name` of sketch `s`, input `k`.
fn line_ends(s: &sketch::Placed, name: &str, k: usize) -> Result<(Vec3, Vec3), BrepError> {
    let line = s.lines.get(name).ok_or_else(|| BrepError::LostReference(format!("no line `{name}` in the sketch of input {k}")))?;
    Ok((sketch_point(s, &line.from, k)?, sketch_point(s, &line.to, k)?))
}

/// Where point `name` of sketch `s`, input `k` of a work plane, lies.
fn sketch_point(s: &Placed, name: &str, k: usize) -> Result<Vec3, BrepError> {
    if s.developed.is_some() {
        return Err(BrepError::Unsupported("a plane through a sketch on a cylinder face".into()));
    }
    let p = s.solved.points.get(name).ok_or_else(|| BrepError::LostReference(format!("no point `{name}` in the sketch of input {k}")))?;
    let [o, x, y, _] = s.frame;
    Ok(o + x * p.x + y * p.y)
}

/// The plane through line `l` of `values`, square to the line's sketch: origin the line's start, X
/// along it.
fn square_to_sketch(l: &LineOf, values: &[&Value]) -> Result<SketchPlane, BrepError> {
    let s = input_sketch(values, l.input, "a plane's line")?;
    let (a, b) = line_ends(s, &l.line, l.input)?;
    let d = b - a;
    let n = d.cross(s.frame[3]);
    Ok(SketchPlane { origin: a, x_axis: d * (1.0 / d.dot(d).sqrt()), normal: n * (1.0 / n.dot(n).sqrt()) })
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
    if profile_curves(feature, s)?.is_empty() {
        return Ok(Vec::new());
    }
    bent::near_bends(feature, t, &*chosen(feature, s, profiles)?)
}

/// Sheet `target` cut by what `s` draws, if it has open lines (D235): each a cut `kerf` wide, with
/// the closed profiles `profiles` choose. None with no open line: an ordinary cut.
fn line_cut(feature: TopoId, target: &Body, s: &Placed, profiles: &[String], kerf: Option<f64>) -> Result<Option<Body>, BrepError> {
    let chains = sketch::open_chains(s)?;
    if chains.is_empty() {
        return Ok(None);
    }
    let kerf = kerf.ok_or_else(|| BrepError::Sketch("a cut along a line takes its width from the part's sheet rule: it has no `kerf`".into()))?;
    if !(kerf > 0.0 && kerf.is_finite()) {
        return Err(BrepError::Degenerate(format!("a cut along a line is wider than nothing, got {kerf}")));
    }
    let closed = if profile_curves(feature, s)?.is_empty() { None } else { Some(chosen(feature, s, profiles)?) };
    bent::punch_lines(feature, target, s, closed.as_deref(), &chains, kerf).map(Some)
}

/// Where the cuts along lines of `op` lie, `sketch` its sketch's value (D235, D237): each curve's ends
/// in space and the width, for the blank to take them back to one line in its DXF.
pub fn line_cuts(op: &Op, sketch: &Value, kerf: f64) -> Vec<LineCut> {
    let (Op::Extrude { wall: true, .. }, Some(s)) = (op, sketch.sketch()) else { return Vec::new() };
    let [o, x, y, normal] = s.frame;
    let at = |p: Vec2| o + x * p.x + y * p.y;
    let chains = sketch::open_chains(s).unwrap_or_default();
    chains
        .iter()
        .flat_map(|(_, pieces)| pieces)
        .map(|c| LineCut { ends: [at(c.from), at(c.to)], normal, kerf, arc: c.arc.map(|a| (at(a.center), a.ccw)) })
        .collect()
}

/// Body `body` with the cut `op` makes on the blank only (an extrusion with `blank_only`, D226),
/// `sketch` its sketch's value: as the sheet cut would make it, for the blank to be laid flat.
pub fn blank_cut(feature: TopoId, body: &Body, sketch: &Value, op: &Op) -> Result<Body, BrepError> {
    let Op::Extrude { blank_only: true, profiles, kerf, .. } = op else { return Err(BrepError::Invalid("not a cut on the blank only".into())) };
    let s = sketch.sketch().ok_or_else(|| BrepError::Sketch("a cut on the blank takes a sketch".into()))?;
    let width = kerf.as_ref().map(num).transpose()?;
    if let Some(made) = line_cut(feature, body, s, profiles, width)? {
        return Ok(made);
    }
    bent::punch(feature, body, &*chosen(feature, s, profiles)?, false)
}

/// An extrusion (see [`Op::Extrude`]), its numbers evaluated.
fn extrude(feature: TopoId, values: &[&Value], op: &Op) -> Result<Body, BrepError> {
    let Op::Extrude { z0, z1, symmetric, mode, through, wall, profiles, blank_only, kerf } = op else {
        return Err(BrepError::Invalid("not an extrusion".into()));
    };
    let (z0, z1, symmetric, mode, through) = (num(z0)?, z1.as_ref().map(num).transpose()?, *symmetric, *mode, *through);
    if *wall && !through {
        return Err(BrepError::Sketch("`wall` is a cut through one wall: with `through`".into()));
    }
    // On the blank only: the body as it was; its blank takes the cut.
    if *blank_only {
        return match (values.first().and_then(|v| v.body()), *wall && mode == Mode::Cut) {
            (Some(t), true) => Ok(t.clone()),
            _ => Err(BrepError::Sketch("`blank_only` is a cut of sheet metal (`wall`, `mode: cut`) its blank alone takes".into())),
        };
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
    if let (Some(t), true) = (target, *wall && through && mode == Mode::Cut && s.face.is_some()) {
        let kerf = kerf.as_ref().map(num).transpose()?;
        if let Some(made) = line_cut(feature, t, s, profiles, kerf)? {
            return Ok(made);
        }
    }
    let s = &*chosen(feature, s, profiles)?;
    let plane = values.get(k + 1).map(|v| v.plane());
    if through {
        let fits = mode == Mode::Cut || (mode == Mode::Join && *wall);
        let Some(t) = target.filter(|_| fits && z0 == 0.0 && z1.is_none() && !symmetric && plane.is_none()) else {
            return Err(BrepError::Sketch(
                "`through` is a cut of the target along the sketch's normal (or with `wall`, metal added to a sheet): no `z0`, `z1`, `symmetric` or plane"
                    .into(),
            ));
        };
        // Through the walls from a face, one plate after another, made directly; through anything
        // else (a wall the profiles cross, a sketch off the body) by the general Boolean, the tool
        // stretched past the body on both sides: so no face of it lies on the sketch's (D186).
        let local = match s.face {
            // Sheet metal: through its thickness alone, made directly or refused.
            Some(_) if *wall => return bent::punch(feature, t, s, mode == Mode::Join),
            Some(_) => through_walls(feature, t, s),
            None if *wall => return Err(BrepError::Sketch("a cut of sheet metal takes a sketch on a face of it".into())),
            None => Err(BrepError::Unsupported("cut: the sketch lies on no face of the body".into())),
        };
        return match local {
            // A sketch in a development has no tool to stretch through all: the cut's own reason.
            Err(BrepError::Unsupported(why)) if s.developed.is_some() => Err(BrepError::Unsupported(why)),
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
