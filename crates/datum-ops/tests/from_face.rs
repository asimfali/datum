//! Extrusions from a face of the target body (D187): pockets into it, bosses out of it.
#![allow(clippy::unwrap_used, clippy::panic)]

use std::f64::consts::PI;

use datum_brep::{Body, BrepError, TopoId, cuboid, tessellate};
use datum_math::Vec3;
use datum_ops::{Op, Value};

fn id(tag: &str) -> TopoId {
    TopoId::derive(tag).finish()
}

fn assert_close(a: f64, b: f64) {
    assert!((a - b).abs() <= 1e-9 * b.abs().max(1.0), "{a} vs {b}");
}

/// The ID of the face of `b` facing along `n`, as a sketch names it.
fn face(b: &Body, n: Vec3) -> String {
    let f = b.faces().iter().find(|f| f.surface.plane().is_some_and(|p| p.normal.cross(n) == Vec3::ZERO && p.normal.dot(n) > 0.0)).unwrap();
    format!("{:?}", f.id)
}

/// `sketch` on face `on` of `target` extruded with `params` (mode, depth) by the feature `name`.
fn extrude(name: &str, target: &Body, on: &str, sketch: serde_json::Value, mut params: serde_json::Value) -> Result<Body, BrepError> {
    let target = Value::Body(target.clone());
    let s: Op = serde_json::from_value(serde_json::json!({"op": "sketch", "face": on, "sketch": sketch})).unwrap();
    let placed = s.eval(id(&format!("{name}.sketch")), &[&target])?;
    params["op"] = "extrude".into();
    let op: Op = serde_json::from_value(params).unwrap();
    let b = op.eval(id(name), &[&target, &placed])?.body().unwrap().clone();
    b.validate().unwrap();
    tessellate(&b).unwrap();
    Ok(b)
}

fn circle(x: f64, y: f64, d: f64) -> serde_json::Value {
    serde_json::json!({"points": {"h": [x, y]}, "circles": {"c": {"center": "h", "diameter": d}}})
}

#[test]
fn a_pocket_and_a_boss() {
    let plate = cuboid(id("plate"), Vec3::ZERO, Vec3::new(100.0, 60.0, 10.0)).unwrap();
    let top = face(&plate, Vec3::new(0.0, 0.0, 1.0));
    let pocket = extrude("pocket", &plate, &top, circle(30.0, 30.0, 20.0), serde_json::json!({"mode": "cut", "z1": -4})).unwrap();
    assert_close(pocket.volume(), 60_000.0 - 100.0 * PI * 4.0);
    let boss =
        extrude("boss", &pocket, &face(&pocket, Vec3::new(0.0, 0.0, 1.0)), circle(70.0, 30.0, 20.0), serde_json::json!({"mode": "join", "z1": 6}))
            .unwrap();
    assert_close(boss.volume(), 60_000.0 - 100.0 * PI * 4.0 + 100.0 * PI * 6.0);
    // The depth's sign is geometry: a cut out of the face, a join into it, are refused.
    assert!(matches!(extrude("x", &plate, &top, circle(30.0, 30.0, 20.0), serde_json::json!({"mode": "cut", "z1": 4})), Err(BrepError::Sketch(_))));
    assert!(matches!(extrude("x", &plate, &top, circle(30.0, 30.0, 20.0), serde_json::json!({"mode": "join", "z1": -4})), Err(BrepError::Sketch(_))));
    // A floor on the bottom face: refused.
    assert!(extrude("x", &plate, &top, circle(30.0, 30.0, 20.0), serde_json::json!({"mode": "cut", "z1": -10})).is_err());
}
