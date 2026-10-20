//! Sketches in the feature model (M3b): placement, profiles, extrusion, stable IDs.
#![allow(clippy::unwrap_used, clippy::panic)]

use datum_brep::{Body, BrepError, TopoId, cuboid, tessellate};
use datum_math::Vec3;
use datum_ops::{BooleanKind, Op, Value};
use datum_sketch::Sketch;

fn id(tag: &str) -> TopoId {
    TopoId::derive(tag).finish()
}

fn assert_close(a: f64, b: f64) {
    assert!((a - b).abs() <= 1e-9 * b.abs().max(1.0), "{a} vs {b}");
}

fn op(json: serde_json::Value) -> Op {
    serde_json::from_value(json).unwrap()
}

fn sketch_op(sketch: serde_json::Value, extra: serde_json::Value) -> Op {
    let mut o = serde_json::json!({"op": "sketch", "sketch": sketch});
    if let (Some(o), Some(e)) = (o.as_object_mut(), extra.as_object()) {
        o.extend(e.clone());
    }
    op(o)
}

fn eval(o: &Op, feature: &str, inputs: &[&Value]) -> Result<Value, BrepError> {
    o.eval(id(feature), inputs)
}

fn body(v: &Value) -> &Body {
    let b = v.body().unwrap();
    b.validate().unwrap();
    tessellate(b).unwrap();
    b
}

/// A square of side `outer` around a square hole of side `inner`, sloppily drawn, with its
/// dimensions as constraints.
fn framed(outer: f64, inner: f64) -> serde_json::Value {
    serde_json::json!({
        "points": {"a": [0, 0], "b": [9, 0.3], "c": [9.2, 8.7], "d": [0.2, 9], "e": [3, 3], "f": [6, 3.2], "g": [6.1, 6], "h": [2.9, 6.1]},
        "lines": {
            "s1": {"from": "a", "to": "b"}, "s2": {"from": "b", "to": "c"}, "s3": {"from": "c", "to": "d"}, "s4": {"from": "d", "to": "a"},
            "h1": {"from": "e", "to": "f"}, "h2": {"from": "f", "to": "g"}, "h3": {"from": "g", "to": "h"}, "h4": {"from": "h", "to": "e"},
            "diag": {"from": "a", "to": "c", "construction": true}
        },
        "constraints": [
            {"type": "fix", "point": "a"},
            {"type": "horizontal", "line": "s1"}, {"type": "vertical", "line": "s2"}, {"type": "horizontal", "line": "s3"}, {"type": "vertical", "line": "s4"},
            {"type": "length", "line": "s1", "value": outer}, {"type": "length", "line": "s4", "value": outer},
            {"type": "horizontal", "line": "h1"}, {"type": "vertical", "line": "h2"}, {"type": "horizontal", "line": "h3"}, {"type": "vertical", "line": "h4"},
            {"type": "length", "line": "h1", "value": inner}, {"type": "length", "line": "h4", "value": inner}
        ]
    })
}

#[test]
fn profile_with_a_hole() {
    let s = eval(&sketch_op(framed(10.0, 3.0), serde_json::json!({})), "s", &[]).unwrap();
    let v = eval(&op(serde_json::json!({"op": "extrude", "z0": 0, "z1": 4})), "x", &[&s]).unwrap();
    let b = body(&v);
    assert_close(b.volume(), (100.0 - 9.0) * 4.0);
    assert_eq!(b.faces().len(), 2 + 8);
}

/// Side faces are named by their lines: changing dimensions keeps every ID (§4.2, P5).
#[test]
fn dimensions_keep_ids() {
    let ids = |outer: f64, inner: f64| {
        let s = eval(&sketch_op(framed(outer, inner), serde_json::json!({})), "s", &[]).unwrap();
        let v = eval(&op(serde_json::json!({"op": "extrude", "z0": 0, "z1": 4})), "x", &[&s]).unwrap();
        let b = body(&v);
        (b.faces().iter().map(|f| f.id).collect::<Vec<_>>(), b.hash())
    };
    let (a, ha) = ids(10.0, 3.0);
    let (b, hb) = ids(14.0, 5.0);
    assert_eq!(a, b);
    assert_ne!(ha, hb);
}

/// A pocket sketched on the plate's top face and cut flush with it (Q7: an open pocket).
#[test]
fn pocket_on_a_face() {
    let plate = Value::Body(cuboid(id("plate"), Vec3::ZERO, Vec3::new(40.0, 30.0, 8.0)).unwrap());
    let top = plate.body().unwrap().faces().iter().find(|f| f.surface.plane().is_some_and(|p| p.normal.z > 0.0)).unwrap().id;
    let square = serde_json::json!({
        "points": {"p": [5, 5], "q": [15, 5], "r": [15, 12], "s": [5, 12]},
        "lines": {"l1": {"from": "p", "to": "q"}, "l2": {"from": "q", "to": "r"}, "l3": {"from": "r", "to": "s"}, "l4": {"from": "s", "to": "p"}}
    });
    let s = eval(&sketch_op(square, serde_json::json!({"face": format!("{top:?}")})), "s", &[&plate]).unwrap();
    // The sketch sits on the face: its normal is the face's outward normal.
    assert_eq!(s.sketch().unwrap().frame[3], Vec3::new(0.0, 0.0, 1.0));
    let tool = eval(&op(serde_json::json!({"op": "extrude", "z0": -3, "z1": 0})), "pocket", &[&s]).unwrap();
    let cut = eval(&op(serde_json::json!({"op": "boolean", "kind": "difference"})), "cut", &[&plate, &tool]).unwrap();
    assert_close(body(&cut).volume(), 40.0 * 30.0 * 8.0 - 10.0 * 7.0 * 3.0);
    let _ = BooleanKind::Difference;
}

#[test]
fn lost_face_and_bad_profiles() {
    let plate = Value::Body(cuboid(id("plate"), Vec3::ZERO, Vec3::new(4.0, 3.0, 2.0)).unwrap());
    let empty = serde_json::json!({"points": {}});
    match eval(&sketch_op(empty.clone(), serde_json::json!({"face": "0123"})), "s", &[&plate]) {
        Err(BrepError::LostReference(m)) => assert!(m.contains("not in the input body"), "{m}"),
        r => panic!("expected a lost reference, got {r:?}"),
    }
    let open = serde_json::json!({
        "points": {"a": [0, 0], "b": [1, 0], "c": [1, 1]},
        "lines": {"l1": {"from": "a", "to": "b"}, "l2": {"from": "b", "to": "c"}}
    });
    let s = eval(&sketch_op(open, serde_json::json!({})), "s", &[]).unwrap();
    match eval(&op(serde_json::json!({"op": "extrude", "z0": 0, "z1": 1})), "x", &[&s]) {
        Err(BrepError::Sketch(m)) => assert!(m.contains("open or branches at point `a`"), "{m}"),
        r => panic!("expected an open profile, got {r:?}"),
    }
    // Inputs of the wrong kind are named.
    match eval(&op(serde_json::json!({"op": "extrude", "z0": 0, "z1": 1})), "x", &[&plate]) {
        Err(BrepError::Sketch(m)) => assert!(m.contains("takes a sketch"), "{m}"),
        r => panic!("expected a kind error, got {r:?}"),
    }
    let conflicting = serde_json::json!({
        "points": {"a": [0, 0], "b": [1, 0]},
        "lines": {"l": {"from": "a", "to": "b"}},
        "constraints": [{"type": "length", "line": "l", "value": 2}, {"type": "length", "line": "l", "value": 3}]
    });
    match eval(&sketch_op(conflicting, serde_json::json!({})), "s", &[]) {
        Err(BrepError::Sketch(m)) => assert!(m.contains("in conflict"), "{m}"),
        r => panic!("expected a conflict, got {r:?}"),
    }
    let _: Sketch = serde_json::from_value(empty).unwrap();
}

/// An island in a hole is a body of its own; an explicit plane places the extrusion.
#[test]
fn islands_and_planes() {
    let nested = serde_json::json!({
        "points": {
            "a": [0, 0], "b": [10, 0], "c": [10, 10], "d": [0, 10],
            "e": [2, 2], "f": [8, 2], "g": [8, 8], "h": [2, 8],
            "i": [4, 4], "j": [6, 4], "k": [6, 6], "l": [4, 6]
        },
        "lines": {
            "o1": {"from": "a", "to": "b"}, "o2": {"from": "b", "to": "c"}, "o3": {"from": "c", "to": "d"}, "o4": {"from": "d", "to": "a"},
            "h1": {"from": "e", "to": "f"}, "h2": {"from": "f", "to": "g"}, "h3": {"from": "g", "to": "h"}, "h4": {"from": "h", "to": "e"},
            "i1": {"from": "i", "to": "j"}, "i2": {"from": "j", "to": "k"}, "i3": {"from": "k", "to": "l"}, "i4": {"from": "l", "to": "i"}
        }
    });
    let plane = serde_json::json!({"plane": {"origin": [0, 0, 100], "x_axis": [0, 1, 0], "normal": [1, 0, 0]}});
    let s = eval(&sketch_op(nested, plane), "s", &[]).unwrap();
    let v = eval(&op(serde_json::json!({"op": "extrude", "z0": 0, "z1": 2})), "x", &[&s]).unwrap();
    let b = body(&v);
    assert_close(b.volume(), (100.0 - 36.0 + 4.0) * 2.0);
    assert_eq!(b.shell_vertices().len(), 2);
    // Along the normal X from x = 0 to 2; the sketch's X runs along world Y.
    let m = b.mass_properties();
    assert_close(m.min.x, 0.0);
    assert_close(m.max.x, 2.0);
    assert_close(m.max.y, 10.0);
    assert_close(m.max.z, 110.0);
}
