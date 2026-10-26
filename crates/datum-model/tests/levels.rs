//! Levels of a model (D152): a frame, a part below it, variants below the part; each what differs
//! from its parent, the parent's changes reaching every level below.
#![allow(clippy::unwrap_used, clippy::panic)]

use datum_model::Session;
use serde_json::{Value, json};

fn run(s: &mut Session, cmd: &str, p: Value) -> Value {
    s.execute(cmd, &p).unwrap_or_else(|e| panic!("{cmd}: {e}"))
}

fn volume(s: &mut Session, name: &str) -> f64 {
    run(s, "mass_properties", json!({"name": name}))["volume"].as_f64().unwrap()
}

/// The frame: a plate `w` × 50 drawn in a sketch (its width a dimension), `t` thick.
fn frame(s: &mut Session) {
    run(s, "set_parameter", json!({"name": "w", "value": 100}));
    run(s, "set_parameter", json!({"name": "t", "value": 2}));
    run(
        s,
        "add",
        json!({"name": "outline", "op": "sketch", "sketch": {
            "points": {"a": [0, 0], "b": [100, 0], "c": [100, 50], "d": [0, 50]},
            "lines": {"bottom": {"from": "a", "to": "b"}, "right": {"from": "b", "to": "c"}, "top": {"from": "c", "to": "d"}, "left": {"from": "d", "to": "a"}},
            "constraints": [{"type": "fix", "point": "a"}, {"type": "horizontal", "line": "bottom"}, {"type": "vertical", "line": "right"},
                            {"type": "horizontal", "line": "top"}, {"type": "vertical", "line": "left"},
                            {"type": "length", "line": "bottom", "value": "w"}, {"type": "length", "line": "left", "value": 50}]}}),
    );
    run(s, "add", json!({"name": "plate", "op": "extrude", "inputs": ["outline"], "z0": 0, "z1": "t"}));
}

/// A hole Ø10 at (20, 25), held by dimensions from the plate's corner.
fn hole() -> Value {
    json!({"name": "outline", "points": {"h": [20, 25]}, "circles": {"hole": {"center": "h", "diameter": 10}},
           "add_constraints": [{"type": "horizontal_distance", "points": ["a", "h"], "value": 20}, {"type": "vertical_distance", "points": ["a", "h"], "value": 25}]})
}

/// A variant longer, another with a hole too; a change to the frame reaches both, each keeping
/// what it set; what is shared is computed once.
#[test]
fn variants_follow_their_parent() {
    let mut s = Session::default();
    frame(&mut s);
    assert_eq!(volume(&mut s, "plate"), 10000.0);
    run(&mut s, "add_level", json!({"name": "long"}));
    run(&mut s, "set_parameter", json!({"name": "w", "value": 200}));
    assert_eq!(volume(&mut s, "plate"), 20000.0);
    run(&mut s, "add_level", json!({"name": "long_holed", "parent": "long"}));
    run(&mut s, "edit_sketch", hole());
    let holed = 20000.0 - 2.0 * 25.0 * std::f64::consts::PI;
    assert!((volume(&mut s, "plate") - holed).abs() < 1e-9);
    // The frame thicker and its height changed: both variants follow, each keeping its own.
    run(&mut s, "activate", json!({"level": null}));
    run(&mut s, "set_parameter", json!({"name": "t", "value": 3}));
    run(
        &mut s,
        "edit_sketch",
        json!({"name": "outline", "remove_constraints": [{"type": "length", "line": "left", "value": 50}],
                                       "add_constraints": [{"type": "length", "line": "left", "value": 60}]}),
    );
    assert_eq!(volume(&mut s, "plate"), 18000.0);
    run(&mut s, "activate", json!({"level": "long"}));
    assert_eq!(volume(&mut s, "plate"), 36000.0);
    run(&mut s, "activate", json!({"level": "long_holed"}));
    let holed = 36000.0 - 3.0 * 25.0 * std::f64::consts::PI;
    assert!((volume(&mut s, "plate") - holed).abs() < 1e-9, "the hole kept, the frame's height and thickness taken");
    // The level holds only what differs: the hole and its dimensions, not the frame's sketch.
    let levels = run(&mut s, "levels", json!({}));
    assert_eq!(levels["active"], "long_holed");
    assert_eq!(
        levels["levels"],
        json!([
            {"name": "long", "parent": null, "depth": 0, "parameters": {"w": 200.0}, "features": [],
             "changed": [], "own": [], "parameters_set": ["w"], "own_parameters": []},
            {"name": "long_holed", "parent": "long", "depth": 1, "parameters": {}, "features": ["outline"],
             "changed": ["outline"], "own": [], "parameters_set": [], "own_parameters": []}
        ])
    );
    // Back as the parent has it: the hole goes, then the length.
    run(&mut s, "revert", json!({"feature": "outline"}));
    assert_eq!(volume(&mut s, "plate"), 36000.0);
    let e = s.execute("revert", &json!({"parameter": "w"})).unwrap_err();
    assert!(e.contains("does not give"), "`w` is set in `long`, not here: {e}");
    run(&mut s, "revert", json!({"parameter": "w", "level": "long"}));
    assert_eq!(volume(&mut s, "plate"), 18000.0);
}

/// A command given `level` runs there and comes back; a level cannot drop what it inherits;
/// undo goes back through levels made and changed.
#[test]
fn levels_are_worked_in_by_name_and_undone() {
    let mut s = Session::default();
    frame(&mut s);
    run(&mut s, "add_level", json!({"name": "long"}));
    run(&mut s, "activate", json!({"level": null}));
    run(&mut s, "set_parameter", json!({"name": "w", "value": 300, "level": "long"}));
    assert_eq!(run(&mut s, "inspect", json!({}))["level"], Value::Null, "back in the root");
    assert_eq!(volume(&mut s, "plate"), 10000.0);
    assert_eq!(s.execute("mass_properties", &json!({"name": "plate", "level": "long"})).unwrap()["volume"].as_f64(), Some(30000.0));
    let e = s.execute("remove", &json!({"name": "plate", "level": "long"})).unwrap_err();
    assert!(e.contains("comes from the parent"), "{e}");
    let e = s.execute("remove_parameter", &json!({"name": "t", "level": "long"})).unwrap_err();
    assert!(e.contains("comes from the parent"), "{e}");
    assert_eq!(s.execute("mass_properties", &json!({"name": "plate", "level": "long"})).unwrap()["volume"].as_f64(), Some(30000.0));
    run(&mut s, "undo", json!({}));
    assert_eq!(s.execute("mass_properties", &json!({"name": "plate", "level": "long"})).unwrap()["volume"].as_f64(), Some(10000.0));
    run(&mut s, "undo", json!({}));
    assert!(s.execute("activate", &json!({"level": "long"})).is_err(), "the level made is undone");
    run(&mut s, "redo", json!({}));
    run(&mut s, "activate", json!({"level": "long"}));
    let e = s.execute("remove_level", &json!({"name": "nothing"})).unwrap_err();
    assert!(e.contains("no level"), "{e}");
    run(&mut s, "add_level", json!({"name": "longer", "parent": "long"}));
    let e = s.execute("remove_level", &json!({"name": "long"})).unwrap_err();
    assert!(e.contains("below"), "{e}");
}

/// A point renamed in the frame's sketch is renamed in the variant that dimensions from it.
#[test]
fn a_rename_reaches_the_levels_below() {
    let mut s = Session::default();
    frame(&mut s);
    run(&mut s, "add_level", json!({"name": "holed"}));
    run(&mut s, "edit_sketch", hole());
    run(&mut s, "activate", json!({"level": null}));
    run(&mut s, "rename_in_sketch", json!({"name": "outline", "from": "a", "to": "corner"}));
    run(&mut s, "activate", json!({"level": "holed"}));
    let f = &run(&mut s, "inspect", json!({}))["features"][0];
    assert!(f.get("error").is_none(), "{f}");
    let c = f["sketch"]["constraints"].as_array().unwrap();
    assert!(c.iter().any(|c| c["type"] == "horizontal_distance" && c["points"] == json!(["corner", "h"])), "{c:?}");
}

/// Saved and opened again, the levels are as they were, the file holding only the differences.
#[test]
fn levels_are_saved_and_opened() {
    let mut s = Session::default();
    frame(&mut s);
    run(&mut s, "add_level", json!({"name": "holed"}));
    run(&mut s, "edit_sketch", hole());
    run(&mut s, "add", json!({"name": "moved", "op": "transform", "inputs": ["plate"], "translate": [0, 0, 10]}));
    let dir = std::env::temp_dir().join(format!("datum-levels-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("wall.json");
    run(&mut s, "save", json!({"path": path.display().to_string()}));
    let file: Value = serde_json::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
    let level = &file["levels"]["holed"];
    assert_eq!(level["features"][0]["name"], "outline");
    assert_eq!(level["features"][0]["sketch"]["constraints"]["add"].as_array().map(Vec::len), Some(2), "{level}");
    assert!(level["features"][0]["sketch"]["lines"].is_null(), "the frame's lines are not repeated: {level}");
    assert_eq!(level["features"][1]["op"], "transform");
    let mut t = Session::default();
    run(&mut t, "open", json!({"path": path.display().to_string()}));
    assert_eq!(run(&mut t, "inspect", json!({}))["modified"], false);
    run(&mut t, "activate", json!({"level": "holed"}));
    let v = volume(&mut t, "moved");
    assert!((v - (10000.0 - 2.0 * 25.0 * std::f64::consts::PI)).abs() < 1e-9);
    std::fs::remove_dir_all(&dir).unwrap();
}

/// A feature turned off passes its first input on (D159): a variant does without the parent's
/// hole, a formula turns it off by a parameter, ↺ brings it back; off with no input, it fails.
#[test]
fn a_variant_turns_a_parents_feature_off() {
    let mut s = Session::default();
    frame(&mut s);
    run(&mut s, "add", json!({"name": "pin", "op": "prism", "polygon": [[10, 10], [20, 10], [20, 20], [10, 20]], "z0": -1, "z1": 5}));
    run(&mut s, "add", json!({"name": "drilled", "op": "boolean", "kind": "difference", "inputs": ["plate", "pin"]}));
    run(&mut s, "add", json!({"name": "moved", "op": "transform", "inputs": ["drilled"], "translate": [0, 0, 10]}));
    assert_eq!(volume(&mut s, "moved"), 10000.0 - 200.0);
    run(&mut s, "add_level", json!({"name": "plain"}));
    run(&mut s, "edit", json!({"name": "drilled", "enabled": false}));
    assert_eq!(volume(&mut s, "moved"), 10000.0, "the plate passed on as it was");
    let levels = run(&mut s, "levels", json!({}));
    assert_eq!(levels["levels"][0]["changed"], json!(["drilled"]));
    run(&mut s, "revert", json!({"feature": "drilled"}));
    assert_eq!(volume(&mut s, "moved"), 9800.0);
    // By a parameter: off while `holes` is 0.
    run(&mut s, "activate", json!({"level": null}));
    run(&mut s, "set_parameter", json!({"name": "holes", "value": 1}));
    run(&mut s, "edit", json!({"name": "drilled", "enabled": "holes"}));
    run(&mut s, "set_parameter", json!({"name": "holes", "value": 0, "level": "plain"}));
    assert_eq!(volume(&mut s, "moved"), 9800.0);
    assert_eq!(s.execute("mass_properties", &json!({"name": "moved", "level": "plain"})).unwrap()["volume"].as_f64(), Some(10000.0));
    // Off with nothing to pass on.
    run(&mut s, "edit", json!({"name": "outline", "enabled": 0}));
    let f = &run(&mut s, "inspect", json!({}))["features"][0];
    assert!(f["error"].as_str().unwrap().contains("no input"), "{f}");
    assert_eq!(f["enabled"], json!(0.0));
}
