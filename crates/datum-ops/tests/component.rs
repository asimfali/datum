//! A library part placed in a part (D260, D261): its definition's features build one body, copied
//! to every placement; a line's length feeds the parameter `fit`, a point puts the tool where it
//! is. The sheet is not cut or formed yet.
#![allow(clippy::unwrap_used, clippy::panic)]

use std::collections::{BTreeMap, BTreeSet};

use datum_brep::{Body, BrepError, TopoId, cuboid, tessellate};
use datum_math::Vec3;
use datum_math::expr::Num;
use datum_ops::{Op, PartDef, Value, tool};
use serde_json::{Map, Value as Json, json};

fn id(tag: &str) -> TopoId {
    TopoId::derive(tag).finish()
}

fn op_of(j: Json) -> Op {
    serde_json::from_value(j).unwrap()
}

/// A stamp: its body a unit column as tall as the parameter `w`, which a line's length feeds.
fn part() -> Json {
    json!({
        "name": "Язык 20",
        "kind": "stamp",
        "anchor": [0.0, 0.0, 0.0],
        "axis": [1.0, 0.0, 0.0],
        "parameters": [{"name": "w", "value": 10.0}],
        "fit": "w",
        "features": [
            {"name": "body", "op": {"op": "prism", "polygon": [[0, 0], [1, 0], [1, 1], [0, 1]], "z0": 0, "z1": "w"}, "inputs": []}
        ],
        "roles": {"cut": "body", "form": "body"}
    })
}

fn def_of(part: &Json) -> PartDef {
    serde_json::from_value(part.clone()).unwrap()
}

fn params(w: f64) -> BTreeMap<String, Num> {
    BTreeMap::from([("w".to_string(), Num::Value(w))])
}

/// A component of `part` at `placements`, its parameters as given.
fn component(part: Json, placements: Json, params: Json) -> Op {
    op_of(json!({"op": "component", "part": part, "params": params, "placements": placements}))
}

/// Placements along the sketch's lines `l0`, `l1`, ….
fn lines(n: usize) -> Json {
    Json::Array((0..n).map(|i| json!({"placement": "line", "input": 1, "line": format!("l{i}")})).collect())
}

/// A sketch with line `l{i}` along X, `lengths[i]` long, at `y = 10·i`, and its points `a{i}`,
/// `b{i}`.
fn sketch(lengths: &[f64]) -> Value {
    let (mut points, mut lines) = (Map::new(), Map::new());
    for (i, length) in lengths.iter().enumerate() {
        let y = 10.0 * i as f64;
        points.insert(format!("a{i}"), json!([0.0, y]));
        points.insert(format!("b{i}"), json!([length, y]));
        lines.insert(format!("l{i}"), json!({"from": format!("a{i}"), "to": format!("b{i}")}));
    }
    op_of(json!({"op": "sketch", "sketch": {"points": points, "lines": lines}})).eval(id("sketch"), &[]).unwrap()
}

/// The sheet a component is placed on (not read yet), and the sketch.
fn eval(op: &Op, sketch: &Value) -> Result<Body, BrepError> {
    eval_as(id("comp"), op, sketch)
}

/// [`eval`] for the feature `feature`: an occurrence names its elements by it.
fn eval_as(feature: TopoId, op: &Op, sketch: &Value) -> Result<Body, BrepError> {
    let sheet = Value::Body(cuboid(id("sheet"), Vec3::ZERO, Vec3::new(50.0, 50.0, 1.0)).unwrap());
    let body = op.eval(feature, &[&sheet, sketch])?.body().cloned().ok_or_else(|| BrepError::Invalid("not a body".into()))?;
    body.validate().unwrap();
    tessellate(&body).unwrap();
    Ok(body)
}

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-9 * b.abs().max(1.0)
}

/// One shell per placement, every element named by its placement, the same body on a second run.
#[test]
fn each_placement_is_a_copy_with_ids_of_its_own() {
    let s = sketch(&[20.0, 30.0, 40.0]);
    let op = component(part(), lines(3), json!({}));
    let body = eval(&op, &s).unwrap();
    assert_eq!(body.shell_vertices().len(), 3);
    let ids: BTreeSet<TopoId> = body.faces().iter().map(|f| f.id).collect();
    assert_eq!(ids.len(), body.faces().len());
    assert!(close(body.volume(), 90.0), "volume {}", body.volume());
    let again = eval(&op, &s).unwrap();
    assert_eq!(body.hash(), again.hash());
    // Another occurrence of the same definition names its elements otherwise.
    let other = op_of(json!({"op": "component", "part": part(), "placements": lines(3)}));
    assert_ne!(eval_as(id("other"), &other, &s).unwrap().hash(), body.hash());
}

/// A line's length builds the tool: two lines of one length share its body, another length is
/// another body, and the tool's own parameter is what a point placement keeps.
#[test]
fn the_length_a_line_feeds_builds_the_tool() {
    let def = def_of(&part());
    let made = |w: f64| tool(&def, id("comp"), &params(w)).unwrap();
    assert_eq!(made(30.0).body.hash(), made(30.0).body.hash());
    assert_ne!(made(30.0).body.hash(), made(40.0).body.hash());
    assert!(close(made(30.0).body.volume(), 30.0), "volume {}", made(30.0).body.volume());
    assert!(close(tool(&def, id("comp"), &BTreeMap::new()).unwrap().body.volume(), 10.0));
    // 30 + 30 + 40: the two lines of one length make the same body.
    let s = sketch(&[30.0, 30.0, 40.0]);
    let body = eval(&component(part(), lines(3), json!({})), &s).unwrap();
    assert!(close(body.volume(), 100.0), "volume {}", body.volume());
    assert_eq!(body.shell_vertices().len(), 3);
}

/// A point placement puts the tool's anchor at a point of the sketch: by its name, or by the
/// sketch's coordinates; the axis is unturned.
#[test]
fn a_point_placement_puts_the_tool_at_the_point() {
    let s = sketch(&[20.0]);
    let at = |placement: Json, params: Json| {
        let body = eval(&component(part(), json!([placement]), params), &s).unwrap();
        assert_eq!(body.shell_vertices().len(), 1);
        body.bounds()
    };
    let (lo, hi) = at(json!({"placement": "point", "input": 1, "point": "b0"}), json!({}));
    assert_eq!(lo, Vec3::new(20.0, 0.0, 0.0));
    assert_eq!(hi, Vec3::new(21.0, 1.0, 10.0));
    let (lo, hi) = at(json!({"placement": "point", "input": 1, "at": [5.0, 2.0]}), json!({"w": 4.0}));
    assert_eq!(lo, Vec3::new(5.0, 2.0, 0.0));
    assert_eq!(hi, Vec3::new(6.0, 3.0, 4.0));
}

/// What a component refuses: a parameter its part has not, a line or a point the sketch has not,
/// no placements, a line placement of a part without an `axis`.
#[test]
fn what_a_component_refuses() {
    let s = sketch(&[20.0]);
    let fail = |op: &Op| eval(op, &s).unwrap_err();
    let bad = |m: &str, e: &BrepError| assert!(e.to_string().contains(m), "{e}");
    let nope = fail(&component(part(), lines(1), json!({"nope": 3.0})));
    assert!(matches!(&nope, BrepError::Invalid(m) if m.contains("no parameter `nope` in part `Язык 20`")), "{nope}");
    let line = fail(&component(part(), json!([{"placement": "line", "input": 1, "line": "zzz"}]), json!({})));
    assert!(matches!(&line, BrepError::LostReference(m) if m.contains("no line `zzz`")), "{line}");
    let point = fail(&component(part(), json!([{"placement": "point", "input": 1, "point": "zzz"}]), json!({})));
    assert!(matches!(&point, BrepError::LostReference(m) if m.contains("no point `zzz`")), "{point}");
    let neither = fail(&component(part(), json!([{"placement": "point", "input": 1}]), json!({})));
    assert!(matches!(&neither, BrepError::Sketch(_)), "{neither}");
    bad("`placements` is empty", &fail(&component(part(), json!([]), json!({}))));
    // A line placement turns the tool along the line: a part without an `axis` cannot be placed
    // by one, and a `fit` without an axis is refused where the part is written.
    let mut no_axis = part();
    no_axis.as_object_mut().unwrap().remove("axis");
    bad("has no `axis`", &fail(&component(no_axis.clone(), lines(1), json!({}))));
    assert!(def_of(&no_axis).validate().unwrap_err().contains("needs an `axis`"));
}

proptest::proptest! {
    #![proptest_config(proptest::prelude::ProptestConfig::with_cases(16))]

    /// Every placement gets a copy of its own, named by its place; the same on a second run.
    #[test]
    fn every_placement_gets_a_copy_of_its_own(k in 1usize..=8) {
        let lengths: Vec<f64> = (0..k).map(|i| 20.0 + i as f64).collect();
        let s = sketch(&lengths);
        let op = component(part(), lines(k), json!({}));
        let body = eval(&op, &s).unwrap();
        proptest::prop_assert_eq!(body.shell_vertices().len(), k);
        let ids: BTreeSet<TopoId> = body.faces().iter().map(|f| f.id).collect();
        proptest::prop_assert_eq!(ids.len(), body.faces().len());
        proptest::prop_assert_eq!(body.hash(), eval(&op, &s).unwrap().hash());
    }
}
