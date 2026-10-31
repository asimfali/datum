//! Sheet metal from one open chain of a skeleton sketch (D194): the chain chosen by a curve of it,
//! closed contours and the other chains left out; renaming a curve or a point of the sketch
//! carries the sheet's `path` and `start` (and an extrusion's `profiles`) along.
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

/// A session in a project with the sheet rule `Лист` (thickness 2, radius `t`), and a skeleton
/// sketch: a rectangle with a round hole, an L (`l1`, `l2`) and a U (`u1`..`u3`), apart.
fn skeleton(name: &str) -> Session {
    let root = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join(name);
    let _ = std::fs::remove_dir_all(&root);
    std::fs::create_dir_all(root.join("part")).unwrap();
    std::fs::write(root.join("datum-project.json"), "{}").unwrap();
    let mut s = Session::default();
    run(&mut s, "save", json!({"path": root.join("part/skeleton.json")}));
    let put = |s: &mut Session, level: &str, element: Value| run(s, "put_library", json!({"level": level, "element": element}));
    put(&mut s, "material", json!({"name": "Сталь", "density": 7850}));
    put(&mut s, "rule", json!({"name": "Лист", "kind": "sheet", "material": "Сталь", "thickness": 2, "radius": "t"}));
    run(
        &mut s,
        "add",
        json!({"name": "skel", "op": "sketch", "sketch": {
            "points": {"a": [0, 0], "b": [100, 0], "c": [100, 50], "d": [0, 50], "h": [50, 25],
                "p1": [200, 0], "p2": [200, 50], "p3": [250, 50],
                "q1": [300, 0], "q2": [300, 40], "q3": [330, 40], "q4": [330, 0]},
            "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}, "cd": {"from": "c", "to": "d"}, "da": {"from": "d", "to": "a"},
                "l1": {"from": "p1", "to": "p2"}, "l2": {"from": "p2", "to": "p3"},
                "u1": {"from": "q1", "to": "q2"}, "u2": {"from": "q2", "to": "q3"}, "u3": {"from": "q3", "to": "q4"}},
            "circles": {"hole": {"center": "h", "diameter": 10}}}}),
    );
    s
}

fn sheet(s: &mut Session, extra: Value) -> Result<Value, String> {
    let mut p = json!({"name": "bent", "sketch": "skel", "rule": "Лист", "z1": 10});
    p.as_object_mut().unwrap().extend(extra.as_object().unwrap().clone());
    s.execute("sheet", &p)
}

/// The L chosen by a curve of it builds as the same L alone in its own sketch does.
#[test]
fn a_chain_is_chosen_by_its_curve() {
    let mut s = skeleton("sheet_chain_pick");
    sheet(&mut s, json!({"path": "l2", "start": "p1"})).unwrap();
    let mut own = skeleton("sheet_chain_own");
    run(
        &mut own,
        "add",
        json!({"name": "lonely", "op": "sketch", "sketch": {
            "points": {"p1": [200, 0], "p2": [200, 50], "p3": [250, 50]}, "lines": {"l1": {"from": "p1", "to": "p2"}, "l2": {"from": "p2", "to": "p3"}}}}),
    );
    run(
        &mut own,
        "add",
        json!({"name": "ref", "op": "sheet", "inputs": ["lonely"], "side": "left", "z0": 0, "z1": 10, "start": "p1", "thickness": 2, "radius": 2}),
    );
    let (a, b) = (volume(&mut s, "bent"), volume(&mut own, "ref"));
    assert!((a - b).abs() <= 1e-9 * b, "{a} vs {b}");
}

/// Several open chains need `path`; a closed contour is not a path (the feature fails to build,
/// the reply says why); the rule is required.
#[test]
fn the_chain_and_the_rule_are_asked_for() {
    let mut s = skeleton("sheet_chain_errors");
    let why = |s: &mut Session, extra: Value| sheet(s, extra).unwrap()["failed"].to_string();
    assert!(why(&mut s, json!({"name": "a"})).contains("path"));
    assert!(why(&mut s, json!({"name": "b", "path": "ab"})).contains("closed"));
    let mut p = json!({"name": "c", "sketch": "skel", "path": "l1", "z1": 10});
    let no_rule = s.execute("sheet", &p).unwrap_err();
    assert!(no_rule.contains("rule"), "{no_rule}");
    p["rule"] = json!("Лист");
    assert!(!s.execute("sheet", &p).unwrap()["failed"].to_string().contains(r#""feature":"c""#));
}

/// Symmetric: half the width each way.
#[test]
fn symmetric_goes_half_each_way() {
    let mut s = skeleton("sheet_chain_symmetric");
    sheet(&mut s, json!({"path": "l1", "symmetric": true})).unwrap();
    let f = feature(&mut s, "bent");
    assert_eq!((f["z0"].as_f64(), f["z1"].as_f64()), (Some(-5.0), Some(5.0)), "{f}");
}

/// Renaming a curve or a point of the sketch: the sheet's `path` and `start` and an extrusion's
/// `profiles` follow, and they keep building.
#[test]
fn renames_carry_path_start_and_profiles() {
    let mut s = skeleton("sheet_chain_rename");
    sheet(&mut s, json!({"path": "l1", "start": "p1"})).unwrap();
    let before = volume(&mut s, "bent");
    run(&mut s, "rename_in_sketch", json!({"name": "skel", "from": "l1", "to": "leg"}));
    run(&mut s, "rename_in_sketch", json!({"name": "skel", "from": "p1", "to": "foot"}));
    let f = feature(&mut s, "bent");
    assert_eq!((&f["path"], &f["start"]), (&json!("leg"), &json!("foot")), "{f}");
    let after = volume(&mut s, "bent");
    assert!((after - before).abs() <= 1e-9 * before, "{after} vs {before}");
    run(&mut s, "extrude", json!({"name": "slab", "sketch": "skel", "mode": "new", "z1": 5, "profiles": ["ab"]}));
    run(&mut s, "rename_in_sketch", json!({"name": "skel", "from": "ab", "to": "bottom"}));
    assert_eq!(feature(&mut s, "slab")["profiles"], json!(["bottom"]));
    assert!(volume(&mut s, "slab") > 0.0);
}

/// Renaming a sketch point: a plane through three points of the sketch (D204) follows, and builds.
#[test]
fn a_rename_follows_into_a_plane_through_points() {
    let mut s = Session::default();
    run(&mut s, "add", json!({"name": "tri", "op": "sketch", "sketch": {"points": {"a": [0, 0], "b": [10, 0], "c": [0, 20]}}}));
    let points = json!([{"input": 0, "point": "a"}, {"input": 0, "point": "b"}, {"input": 0, "point": "c"}]);
    run(&mut s, "add", json!({"name": "slope", "op": "plane", "inputs": ["tri", "tri", "tri"], "points": points}));
    run(&mut s, "rename_in_sketch", json!({"name": "tri", "from": "a", "to": "corner"}));
    let f = feature(&mut s, "slope");
    assert_eq!(f["points"][0]["point"], "corner", "{f}");
    assert_eq!((f["points"][1]["point"].as_str(), f["points"][2]["point"].as_str()), (Some("b"), Some("c")), "{f}");
    assert!(f.get("error").is_none(), "{f}");
    let all = run(&mut s, "inspect", json!({}));
    assert!(all["features"].as_array().unwrap().iter().all(|f| f.get("error").is_none()), "{all}");
}
