//! A cut across a bend line (D197): a hole punched through the blank, on walls along the normal,
//! on bends along the radius; the blank shows it as drawn, whichever side it is seen from.
#![allow(clippy::unwrap_used, clippy::panic)]

use datum_brep::{Body, BrepError, Surface, TopoId};
use datum_ops::{FlatPiece, NearBend, Op, Value, near_bends, unfold_side};
use proptest::prelude::*;

const K: f64 = 0.44;
/// Thickness, bend radius and width of the sheet; the first wall's flat part on its outer face.
const T: f64 = 2.0;
const WIDTH: f64 = 40.0;
const FLAT: f64 = 58.0;
/// The length of a bend's neutral layer.
const BEND: f64 = std::f64::consts::FRAC_PI_2 * (T + K * T);

fn id(tag: &str) -> TopoId {
    TopoId::derive(tag).finish()
}

fn op(json: serde_json::Value) -> Op {
    serde_json::from_value(json).unwrap()
}

/// An L of sheet metal: up 60 from the origin, then 80 along X, on the left of its path.
fn sheet() -> Body {
    let path = serde_json::json!({"points": {"a": [0, 0], "b": [0, 60], "c": [80, 60]},
        "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}}});
    let p = op(serde_json::json!({"op": "sketch", "sketch": path})).eval(id("path"), &[]).unwrap();
    let s =
        op(serde_json::json!({"op": "sheet", "thickness": T, "radius": T, "side": "left", "z0": 0, "z1": WIDTH})).eval(id("sheet"), &[&p]).unwrap();
    s.body().unwrap().clone()
}

/// The ID of the outer face of the first wall (looking along −X), whose sketch has X along the
/// wall and Y along −Z.
fn first_wall(b: &Body) -> String {
    let f = b.faces().iter().find(|f| matches!(f.surface, Surface::Plane(p) if p.normal.x < -0.5)).unwrap();
    format!("{:?}", f.id)
}

/// A circle of `diameter` at `along` the wall and height `z`, cut through the wall of `b`.
fn punched(b: &Body, along: f64, z: f64, diameter: f64) -> Result<Body, BrepError> {
    let target = Value::Body(b.clone());
    let sketch = serde_json::json!({"points": {"h": [along, -z]}, "circles": {"c": {"center": "h", "diameter": diameter}}});
    let placed = op(serde_json::json!({"op": "sketch", "face": first_wall(b), "k_factor": K, "sketch": sketch})).eval(id("hole"), &[&target])?;
    let cut = op(serde_json::json!({"op": "extrude", "mode": "cut", "through": true, "wall": true}));
    let r = cut.eval(id("cut"), &[&target, &placed])?.body().unwrap().clone();
    r.validate().unwrap();
    Ok(r)
}

/// How close a circle of `diameter` at `along` the first wall of `b`, height `z`, comes to a bend.
fn near(b: &Body, along: f64, z: f64, diameter: f64) -> Vec<NearBend> {
    let target = Value::Body(b.clone());
    let sketch = serde_json::json!({"points": {"h": [along, -z]}, "circles": {"c": {"center": "h", "diameter": diameter}}});
    let placed =
        op(serde_json::json!({"op": "sketch", "face": first_wall(b), "k_factor": K, "sketch": sketch})).eval(id("hole"), &[&target]).unwrap();
    let cut = op(serde_json::json!({"op": "extrude", "mode": "cut", "through": true, "wall": true}));
    near_bends(id("cut"), &[&target, &placed], &cut).unwrap()
}

/// 2t + R from the bend line a hole distorts on bending: closer, the check gives the distance;
/// into the bend, none; farther, nothing. A hole on the wall beyond the bend is measured too.
#[test]
fn a_hole_near_a_bend_is_told() {
    let b = sheet();
    let clearance = 2.0 * T + T;
    assert_eq!(near(&b, FLAT - 5.0 - clearance - 0.5, 20.0, 10.0), []);
    let [NearBend { distance: Some(d), clearance: c }] = near(&b, FLAT - 5.0 - 3.0, 20.0, 10.0)[..] else { panic!() };
    assert!((d - 3.0).abs() < 1e-9 && (c - clearance).abs() < 1e-12, "{d} {c}");
    assert_eq!(near(&b, FLAT - 2.0, 20.0, 10.0), [NearBend { distance: None, clearance }]);
    let [NearBend { distance: Some(d), .. }] = near(&b, FLAT + BEND + 5.0 + 2.0, 20.0, 10.0)[..] else { panic!() };
    assert!((d - 2.0).abs() < 1e-9, "{d}");
    // Not a cut of sheet metal (through all the body): nothing to tell.
    let target = Value::Body(b.clone());
    assert_eq!(near_bends(id("x"), &[&target], &op(serde_json::json!({"op": "extrude", "mode": "cut", "through": true}))).unwrap(), []);
}

/// The circles of the blank of `b` seen from the front or the back, as their radii.
fn circles(b: &Body, back: bool) -> Vec<f64> {
    let flat = unfold_side(b, back, K).unwrap();
    assert_eq!(flat.counts()[1], 0, "{:?}", flat.pieces);
    flat.pieces.iter().filter_map(|p| if let FlatPiece::Circle { radius, .. } = *p { Some(radius) } else { None }).collect()
}

/// A hole across the first bend, as an example of both sides.
#[test]
fn a_hole_across_a_bend_is_a_circle_on_either_side() {
    let b = sheet();
    let cut = punched(&b, FLAT - 2.0, 20.0, 10.0).unwrap();
    assert!(cut.volume() < b.volume());
    for back in [true, false] {
        let radii = circles(&cut, back);
        assert!(radii.len() == 1 && (radii[0] - 5.0).abs() < 1e-9, "back: {back}: {radii:?}");
    }
}

/// A circle of radius `r` about `along` the wall, `z` up, crossing the first bend line and not
/// reaching the end of the bend (that would divide the bend's face).
fn crossing() -> impl Strategy<Value = (f64, f64, f64)> {
    (1.0..8.0f64).prop_flat_map(|r| {
        let (lo, hi) = ((r + 0.5).max(FLAT - r + 0.3), FLAT + BEND - r - 0.3);
        (Just(r), lo..hi, (r + 1.0)..(WIDTH - r - 1.0))
    })
}

fn check(back: bool) -> impl Fn((f64, f64, f64)) -> Result<(), TestCaseError> {
    move |(r, along, z)| {
        let b = sheet();
        let cut = punched(&b, along, z, 2.0 * r).map_err(|e| TestCaseError::fail(format!("r {r} along {along} z {z}: {e}")))?;
        prop_assert!(cut.volume() < b.volume());
        let radii = circles(&cut, back);
        prop_assert_eq!(radii.len(), 1, "r {} along {} z {}", r, along, z);
        prop_assert!((radii[0] - r).abs() < 1e-9 * r, "{} against {}", radii[0], r);
        Ok(())
    }
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(32))]

    /// A circle across the bend line is cut, the body valid and lighter, and the blank seen from
    /// its back has it as one circle of the radius drawn.
    #[test]
    fn a_circle_across_a_bend_is_one_circle_on_the_back(c in crossing()) {
        check(true)(c)?;
    }

    /// The same, seen from the front.
    #[test]
    fn a_circle_across_a_bend_is_one_circle_on_the_front(c in crossing()) {
        check(false)(c)?;
    }
}
