//! A library part placed in a model (D260, D261): the `cut` role's profile, laid on the sheet's
//! wall by each placement, punches it through; renaming a line or a point of the placement sketch
//! carries the component's placements along.
#![allow(clippy::unwrap_used, clippy::panic)]

use datum_model::Session;
use serde_json::{Value, json};

fn run(s: &mut Session, cmd: &str, p: Value) -> Value {
    s.execute(cmd, &p).unwrap_or_else(|e| panic!("{cmd}: {e}"))
}

fn volume(s: &mut Session, name: &str) -> f64 {
    run(s, "mass_properties", json!({"name": name}))["volume"].as_f64().unwrap()
}

/// The feature `name` as `inspect` lists it.
fn feature(s: &mut Session, name: &str) -> Value {
    run(s, "inspect", json!({}))["features"].as_array().unwrap().iter().find(|f| f["name"] == name).unwrap_or_else(|| panic!("no {name}")).clone()
}

/// A stamp (D256): its `cut` role an extrusion of a circle of diameter 10 about its anchor.
fn part() -> Value {
    json!({"name": "Язык 20", "kind": "stamp", "anchor": [0.0, 0.0, 0.0], "axis": [1.0, 0.0, 0.0],
        "features": [
            {"name": "profile", "inputs": [],
             "op": {"op": "sketch", "sketch": {"points": {"h": [0, 0]}, "circles": {"c": {"center": "h", "diameter": 10}}}}},
            {"name": "body", "inputs": ["profile"], "op": {"op": "extrude", "z0": 0, "z1": 10}}
        ],
        "roles": {"cut": "body"}})
}

/// A plate 100 × 60 × 2 and a sketch on its top face: a point `p` at (50, 30) and a line `l` from
/// (10, 10) to (40, 10), both as the face's sketch coordinates.
fn model() -> Session {
    let mut s = Session::default();
    let outline = json!({"points": {"a": [0, 0], "b": [100, 0], "c": [100, 60], "d": [0, 60]},
        "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}, "cd": {"from": "c", "to": "d"}, "da": {"from": "d", "to": "a"}}});
    run(&mut s, "add", json!({"name": "outline", "op": "sketch", "sketch": outline}));
    run(&mut s, "add", json!({"name": "sheet", "op": "plate", "inputs": ["outline"], "thickness": 2}));
    let top = run(&mut s, "query_faces", json!({"name": "sheet", "normal": [0, 0, 1]}))["faces"][0]["id"].as_str().unwrap().to_string();
    let sketch = json!({"points": {"p": [50, 30], "a": [10, 10], "b": [40, 10]}, "lines": {"l": {"from": "a", "to": "b"}}});
    run(&mut s, "add", json!({"name": "placements", "op": "sketch", "inputs": ["sheet"], "face": top, "sketch": sketch}));
    s
}

/// A stamp placed on the plate's top face cuts it through: the plate less the profile's area
/// through its thickness.
#[test]
fn a_component_punches_the_sheet_of_a_model() {
    let mut s = model();
    run(
        &mut s,
        "add",
        json!({"name": "punch", "op": "component", "inputs": ["sheet", "placements"], "part": part(),
            "placements": [{"placement": "point", "input": 1, "point": "p"}]}),
    );
    let f = feature(&mut s, "punch");
    assert!(f.get("error").is_none(), "{f}");
    let (plate, cut) = (volume(&mut s, "sheet"), volume(&mut s, "punch"));
    let gone = plate - cut;
    assert!((gone - std::f64::consts::PI * 25.0 * 2.0).abs() <= 1e-9 * plate, "removed {gone} of {plate}");
}

/// Renaming a point or a line of the placement sketch: the component's placements follow, and it
/// keeps building.
#[test]
fn renames_carry_into_a_components_placements() {
    let mut s = model();
    let placements = json!([{"placement": "point", "input": 1, "point": "p"}, {"placement": "line", "input": 1, "line": "l"}]);
    run(&mut s, "add", json!({"name": "punch", "op": "component", "inputs": ["sheet", "placements"], "part": part(), "placements": placements}));
    run(&mut s, "rename_in_sketch", json!({"name": "placements", "from": "p", "to": "stamp"}));
    run(&mut s, "rename_in_sketch", json!({"name": "placements", "from": "l", "to": "edge"}));
    let f = feature(&mut s, "punch");
    assert_eq!((&f["placements"][0]["point"], &f["placements"][1]["line"]), (&json!("stamp"), &json!("edge")), "{f}");
    assert!(f.get("error").is_none(), "{f}");
}
