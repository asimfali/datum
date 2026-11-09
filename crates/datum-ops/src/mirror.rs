//! A body mirrored across a plane, or turned half a turn about an axis of it (D247).

use datum_brep::{Body, BrepError};
use datum_math::Rigid;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::SketchPlane;

/// The axis of a plane a mirror turns about instead of reflecting: X or Y in the plane (the part
/// goes across it, as a reflection does) or its normal (the part stays on its side).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum TurnAxis {
    X,
    Y,
    Normal,
}

/// `body` reflected in `plane`, or, with `turn`, the same body half a turn about that axis through
/// the plane's origin.
pub(crate) fn mirror(body: &Body, plane: &SketchPlane, turn: Option<TurnAxis>) -> Result<Body, BrepError> {
    let [origin, x, y, normal] = plane.axes()?;
    let axis = match turn {
        None => return body.reflected(origin, normal),
        Some(TurnAxis::X) => x,
        Some(TurnAxis::Y) => y,
        Some(TurnAxis::Normal) => normal,
    };
    Ok(body.transformed(&Rigid::translation(-origin).then(Rigid::half_turn(axis)).then(Rigid::translation(origin))))
}
