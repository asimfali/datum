//! Choosing profiles of a skeleton sketch (D191): a closed loop by a curve of it, with its holes;
//! open chains (sheet metal paths) left out.
#![allow(clippy::unwrap_used, clippy::panic)]

use datum_brep::{BrepError, TopoId};
use datum_ops::{Op, Value};

fn id(tag: &str) -> TopoId {
    TopoId::derive(tag).finish()
}

/// A rectangle from (x0, y0) to (x1, y1), its points and lines named with `k`.
fn rect(sketch: &mut serde_json::Value, k: &str, x0: f64, y0: f64, x1: f64, y1: f64) {
    let p = |i: usize| format!("{k}p{i}");
    let l = |i: usize| format!("{k}{i}");
    for (i, at) in [[x0, y0], [x1, y0], [x1, y1], [x0, y1]].iter().enumerate() {
        sketch["points"][p(i)] = serde_json::json!(at);
    }
    for i in 0..4 {
        sketch["lines"][l(i)] = serde_json::json!({"from": p(i), "to": p((i + 1) % 4)});
    }
}

/// Plate `a` 100 × 50 with hole `h` 20 × 10, block `b` 30 × 30 beside it, and an open chain `c`
/// (a sheet's path) apart from both.
fn skeleton() -> Value {
    let mut s = serde_json::json!({"points": {}, "lines": {}});
    rect(&mut s, "a", 0.0, 0.0, 100.0, 50.0);
    rect(&mut s, "h", 40.0, 20.0, 60.0, 30.0);
    rect(&mut s, "b", 120.0, 0.0, 150.0, 30.0);
    s["points"]["c0"] = serde_json::json!([0, 80]);
    s["points"]["c1"] = serde_json::json!([60, 80]);
    s["points"]["c2"] = serde_json::json!([60, 120]);
    s["lines"]["c0"] = serde_json::json!({"from": "c0", "to": "c1"});
    s["lines"]["c1"] = serde_json::json!({"from": "c1", "to": "c2"});
    let op: Op = serde_json::from_value(serde_json::json!({"op": "sketch", "sketch": s})).unwrap();
    op.eval(id("skeleton"), &[]).unwrap()
}

fn volume(sketch: &Value, profiles: serde_json::Value) -> Result<f64, BrepError> {
    let op: Op = serde_json::from_value(serde_json::json!({"op": "extrude", "z1": 2, "profiles": profiles})).unwrap();
    let b = op.eval(id("x"), &[sketch])?.body().unwrap().clone();
    b.validate().unwrap();
    Ok(b.volume())
}

#[test]
fn a_loop_by_a_curve_with_its_holes() {
    let s = skeleton();
    let (plate, block) = ((100.0 * 50.0 - 20.0 * 10.0) * 2.0, 30.0 * 30.0 * 2.0);
    // Naming none: every closed loop, the open chain left out.
    assert_eq!(volume(&s, serde_json::json!([])).unwrap(), plate + block);
    assert_eq!(volume(&s, serde_json::json!(["a2"])).unwrap(), plate);
    assert_eq!(volume(&s, serde_json::json!(["b0"])).unwrap(), block);
    assert_eq!(volume(&s, serde_json::json!(["a0", "b3"])).unwrap(), plate + block);
    // The hole named: its own region, solid.
    assert_eq!(volume(&s, serde_json::json!(["h1"])).unwrap(), 20.0 * 10.0 * 2.0);
}

#[test]
fn what_cannot_be_chosen() {
    let s = skeleton();
    match volume(&s, serde_json::json!(["c0"])) {
        Err(BrepError::Sketch(m)) => assert!(m.contains("`c0` is on an open chain"), "{m}"),
        r => panic!("{r:?}"),
    }
    match volume(&s, serde_json::json!(["nothing"])) {
        Err(BrepError::Sketch(m)) => assert!(m.contains("`nothing` is on no profile"), "{m}"),
        r => panic!("{r:?}"),
    }
    // A line from the plate's corner to the chain: a branch, a mistake in a skeleton.
    let mut j = serde_json::json!({"points": {}, "lines": {}});
    rect(&mut j, "a", 0.0, 0.0, 100.0, 50.0);
    j["points"]["t"] = serde_json::json!([0, 80]);
    j["lines"]["t"] = serde_json::json!({"from": "ap3", "to": "t"});
    let op: Op = serde_json::from_value(serde_json::json!({"op": "sketch", "sketch": j})).unwrap();
    let branched = op.eval(id("branched"), &[]).unwrap();
    match volume(&branched, serde_json::json!([])) {
        Err(BrepError::Sketch(m)) => assert!(m.contains("branches at point `ap3`"), "{m}"),
        r => panic!("{r:?}"),
    }
}

/// A click inside a contour chooses it: the innermost loop around the point.
#[test]
fn the_contour_around_a_point() {
    let Value::Sketch(s) = skeleton() else { panic!("a sketch") };
    let at = |x: f64, y: f64| datum_ops::contour_at(id("skeleton"), &s, datum_math::Vec2::new(x, y));
    let names = |k: &str| (0..4).map(|i| format!("{k}{i}")).collect::<Vec<_>>();
    let sorted = |mut v: Vec<String>| {
        v.sort();
        v
    };
    assert_eq!(at(10.0, 10.0).map(sorted), Some(names("a")));
    assert_eq!(at(50.0, 25.0).map(sorted), Some(names("h")));
    assert_eq!(at(130.0, 10.0).map(sorted), Some(names("b")));
    assert_eq!(at(30.0, 100.0), None, "beside the open chain, in no loop");
}
