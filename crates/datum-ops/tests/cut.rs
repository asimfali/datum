//! Cuts through plates by sketch profiles (M5b): round holes and slots on a grid, holes next to
//! holes, refusals, and principle 8.
#![allow(clippy::unwrap_used, clippy::panic)]

use std::f64::consts::PI;

use datum_brep::{Body, BrepError, TopoId, cuboid, tessellate};
use datum_math::{Rigid, Vec3};
use datum_ops::{Op, Value};

fn id(tag: &str) -> TopoId {
    TopoId::derive(tag).finish()
}

fn assert_close(a: f64, b: f64) {
    assert!((a - b).abs() <= 1e-9 * b.abs().max(1.0), "{a} vs {b}");
}

fn op(json: serde_json::Value) -> Op {
    serde_json::from_value(json).unwrap()
}

/// The ID of the face of `b` facing along `n`, as a sketch names it.
fn face(b: &Body, n: Vec3) -> String {
    let f = b.faces().iter().find(|f| f.surface.plane().is_some_and(|p| p.normal.cross(n) == Vec3::ZERO && p.normal.dot(n) > 0.0)).unwrap();
    format!("{:?}", f.id)
}

/// `sketch` on face `on` of `plate`, cut through it with `pattern` (JSON or null) by the feature
/// named `name`.
fn cut_by(name: &str, plate: &Body, on: &str, sketch: serde_json::Value, pattern: serde_json::Value) -> Result<Body, BrepError> {
    let target = Value::Body(plate.clone());
    let placed = op(serde_json::json!({"op": "sketch", "face": on, "sketch": sketch})).eval(id(&format!("{name}.sketch")), &[&target])?;
    let mut c = serde_json::json!({"op": "cut"});
    if !pattern.is_null() {
        c["pattern"] = pattern;
    }
    let v = op(c).eval(id(name), &[&target, &placed])?;
    let b = v.body().unwrap().clone();
    b.validate().unwrap();
    tessellate(&b).unwrap();
    Ok(b)
}

fn cut(plate: &Body, on: &str, sketch: serde_json::Value, pattern: serde_json::Value) -> Result<Body, BrepError> {
    cut_by("cut", plate, on, sketch, pattern)
}

/// A hole of Ø10 at (20, 20) and a slot 20 × 6 beside it.
fn hole_and_slot() -> serde_json::Value {
    serde_json::json!({
        "points": {"h": [20, 20], "a": [30, 17], "b": [50, 17], "c": [50, 23], "d": [30, 23], "l": [30, 20], "r": [50, 20]},
        "lines": {"low": {"from": "a", "to": "b"}, "high": {"from": "c", "to": "d"}},
        "arcs": {"right": {"center": "r", "from": "b", "to": "c"}, "left": {"center": "l", "from": "d", "to": "a"}},
        "circles": {"hole": {"center": "h", "diameter": 10}}
    })
}

/// One hole and one slot in each copy: area of the pair.
const PAIR: f64 = 25.0 * PI + 20.0 * 6.0 + 9.0 * PI;

#[test]
fn holes_and_slots_on_a_grid() {
    let plate = cuboid(id("plate"), Vec3::ZERO, Vec3::new(200.0, 120.0, 3.0)).unwrap();
    let top = face(&plate, Vec3::new(0.0, 0.0, 1.0));
    let r = cut(&plate, &top, hole_and_slot(), serde_json::json!({"count": [4, 3], "step": [[45, 0], [0, 35]]})).unwrap();
    assert_close(r.volume(), (200.0 * 120.0 - 12.0 * PAIR) * 3.0);
    // Each copy brings a circle (3 walls) and a slot (4 walls): 7 faces, the plate keeps its 6.
    assert_eq!(r.faces().len(), 6 + 12 * 7);
    // The holes keep the faces' IDs: the top is still the top.
    assert!(r.faces().iter().any(|f| format!("{:?}", f.id) == top));
}

#[test]
fn from_the_bottom_and_next_to_earlier_holes() {
    let plate = cuboid(id("plate"), Vec3::ZERO, Vec3::new(100.0, 60.0, 3.0)).unwrap();
    let first = cut(&plate, &face(&plate, Vec3::new(0.0, 0.0, 1.0)), hole_and_slot(), serde_json::Value::Null).unwrap();
    // From below the sketch's X runs along the world's, its Y against: (20, -40) is world (20, 40).
    let below = serde_json::json!({"points": {"h": [20, -40]}, "circles": {"hole": {"center": "h", "diameter": 8}}});
    let second = cut_by("second", &first, &face(&first, Vec3::new(0.0, 0.0, -1.0)), below, serde_json::Value::Null).unwrap();
    assert_close(second.volume(), (100.0 * 60.0 - PAIR - 16.0 * PI) * 3.0);
}

#[test]
fn what_a_cut_refuses() {
    let plate = cuboid(id("plate"), Vec3::ZERO, Vec3::new(100.0, 60.0, 3.0)).unwrap();
    let top = face(&plate, Vec3::new(0.0, 0.0, 1.0));
    let circle = |x: f64, y: f64, d: f64| serde_json::json!({"points": {"h": [x, y]}, "circles": {"hole": {"center": "h", "diameter": d}}});
    // Across the plate's edge.
    assert!(matches!(cut(&plate, &top, circle(3.0, 30.0, 10.0), serde_json::Value::Null), Err(BrepError::Unsupported(_))));
    // Copies that overlap.
    let tight = serde_json::json!({"count": [2, 1], "step": [[8, 0], [0, 10]]});
    assert!(matches!(cut(&plate, &top, circle(30.0, 30.0, 10.0), tight), Err(BrepError::Degenerate(_))));
    // Over a hole already there, and around one.
    let holed = cut(&plate, &top, circle(30.0, 30.0, 10.0), serde_json::Value::Null).unwrap();
    let top = face(&holed, Vec3::new(0.0, 0.0, 1.0));
    assert!(matches!(cut(&holed, &top, circle(35.0, 30.0, 10.0), serde_json::Value::Null), Err(BrepError::Unsupported(_))));
    assert!(matches!(cut(&holed, &top, circle(30.0, 30.0, 20.0), serde_json::Value::Null), Err(BrepError::Unsupported(_))));
    // A side face: the opposite one is there, but the profile leaves it.
    let side = face(&plate, Vec3::new(1.0, 0.0, 0.0));
    assert!(matches!(cut(&plate, &side, circle(30.0, 1.5, 10.0), serde_json::Value::Null), Err(BrepError::Unsupported(_))));
}

/// Principle 8: turned by an odd angle and 10⁷ m (in mm) from the origin, the same holes and volume. The
/// sketch's frame on the turned face is its own: the holes are placed in it.
#[test]
fn far_away_and_turned() {
    let m = Rigid::rotation(Vec3::new(1.0, 2.0, 3.0), 0.7).then(Rigid::translation(Vec3::new(1e10, -3e9, 2e9)));
    let plate = cuboid(id("plate"), Vec3::ZERO, Vec3::new(200.0, 120.0, 3.0)).unwrap().transformed(&m);
    let top = face(&plate, m.apply_vector(Vec3::new(0.0, 0.0, 1.0)));
    let target = Value::Body(plate.clone());
    let empty = op(serde_json::json!({"op": "sketch", "face": top, "sketch": {}})).eval(id("frame"), &[&target]).unwrap();
    let [o, x, y, _] = empty.sketch().unwrap().frame;
    let flat = |p: Vec3| [(p - o).dot(x), (p - o).dot(y)];
    let along = |v: Vec3| [v.dot(x), v.dot(y)];
    let centre = flat(m.apply(Vec3::new(20.0, 20.0, 3.0)));
    let holes = serde_json::json!({"points": {"h": centre}, "circles": {"hole": {"center": "h", "diameter": 10}}});
    let pattern = serde_json::json!({"count": [4, 3], "step": [along(m.apply_vector(Vec3::new(45.0, 0.0, 0.0))), along(m.apply_vector(Vec3::new(0.0, 35.0, 0.0)))]});
    let r = cut(&plate, &top, holes, pattern).unwrap();
    assert_eq!(r.faces().len(), 6 + 12 * 3);
    assert!((r.volume() - (200.0 * 120.0 - 12.0 * 25.0 * PI) * 3.0).abs() < 1e-6 * r.volume(), "{}", r.volume());
}
