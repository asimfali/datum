#![allow(clippy::unwrap_used, clippy::panic)] // helpers outside #[test] functions

use datum_model::Session;
use serde_json::{Value, json};

fn run(s: &mut Session, cmd: &str, p: Value) -> Value {
    s.execute(cmd, &p).unwrap_or_else(|e| panic!("{cmd}: {e}"))
}

fn plate(s: &mut Session) {
    run(s, "add", json!({"name": "plate", "op": "prism", "polygon": [[0, 0], [100, 0], [100, 50], [0, 50]], "z0": 0, "z1": 3}));
    run(s, "add", json!({"name": "moved", "op": "transform", "inputs": ["plate"], "translate": [0, 0, 10]}));
}

#[test]
fn edit_is_partial_and_incremental() {
    let mut s = Session::default();
    plate(&mut s);
    let r = run(&mut s, "edit", json!({"name": "moved", "angle_deg": 90}));
    assert_eq!((r["computed"].as_u64(), r["reused"].as_u64()), (Some(1), Some(1)));
    let f = &run(&mut s, "inspect", json!({}))["features"][1];
    assert_eq!((f["angle_deg"].as_f64(), &f["translate"]), (Some(90.0), &json!([0.0, 0.0, 10.0])));
}

#[test]
fn queries() {
    let mut s = Session::default();
    plate(&mut s);
    let m = run(&mut s, "mass_properties", json!({"name": "moved"}));
    assert_eq!(m["volume"].as_f64(), Some(15000.0));
    assert_eq!(m["centroid"], json!([50.0, 25.0, 11.5]));
    let top = run(&mut s, "query_faces", json!({"name": "moved", "normal": [0, 0, 1]}));
    assert_eq!(top["count"].as_u64(), Some(1));
    assert_eq!(top["faces"][0]["offset"].as_f64(), Some(13.0));
    assert_eq!(top["faces"][0]["area"].as_f64(), Some(5000.0));
    assert_eq!(run(&mut s, "query_faces", json!({"name": "moved"}))["count"].as_u64(), Some(6));
}

#[test]
fn failures_are_reported_not_hidden() {
    let mut s = Session::default();
    plate(&mut s);
    let r = run(&mut s, "edit", json!({"name": "plate", "z1": 0}));
    assert_eq!(r["failed"].as_array().map(Vec::len), Some(2));
    assert!(r["failed"][1]["error"].as_str().unwrap().contains("input `plate` failed"));
    assert!(s.execute("mass_properties", &json!({"name": "moved"})).unwrap_err().contains("failed"));
    assert!(s.execute("remove", &json!({"name": "plate"})).unwrap_err().contains("`moved` uses it"));
    run(&mut s, "remove", json!({"name": "moved"}));
    assert_eq!(s.graph().nodes().len(), 1);
    assert!(s.execute("frobnicate", &json!({})).unwrap_err().contains("unknown command"));
}

#[test]
fn save_and_open_round_trip() {
    let mut s = Session::default();
    plate(&mut s);
    let path = std::env::temp_dir().join(format!("datum-session-{}.json", std::process::id()));
    run(&mut s, "save", json!({"path": path}));
    let mut t = Session::default();
    let r = run(&mut t, "open", json!({"path": path}));
    std::fs::remove_file(&path).ok();
    assert_eq!(r["failed"], json!([]));
    assert_eq!(t.graph(), s.graph());
}

#[test]
fn every_command_has_a_schema() {
    let list = Session::commands();
    let cmds = list["commands"].as_array().unwrap();
    assert!(cmds.iter().all(|c| c["params"]["type"] == "object"));
    let add = cmds.iter().find(|c| c["name"] == "add").unwrap();
    assert!(add["params"].to_string().contains("transform"));
}

fn volume(s: &mut Session, name: &str) -> f64 {
    run(s, "mass_properties", json!({"name": name}))["volume"].as_f64().unwrap()
}

/// D57: parameters drive sketch dimensions, extrusion heights and pattern counts; an edit
/// recomputes only the features whose numbers change.
#[test]
fn parameters_drive_the_model() {
    let mut s = Session::default();
    run(&mut s, "set_parameter", json!({"name": "w", "value": 40}));
    run(&mut s, "set_parameter", json!({"name": "t", "value": "w / 8"}));
    run(&mut s, "set_parameter", json!({"name": "n", "value": "2 + 1"}));
    let square = json!({
        "points": {"a": [0, 0], "b": [9, 0], "c": [9, 9], "d": [0, 9]},
        "lines": {"l1": {"from": "a", "to": "b"}, "l2": {"from": "b", "to": "c"}, "l3": {"from": "c", "to": "d"}, "l4": {"from": "d", "to": "a"}},
        "constraints": [
            {"type": "fix", "point": "a"}, {"type": "horizontal", "line": "l1"}, {"type": "vertical", "line": "l2"},
            {"type": "horizontal", "line": "l3"}, {"type": "vertical", "line": "l4"},
            {"type": "length", "line": "l1", "value": "w"}, {"type": "equal", "lines": ["l1", "l2"]}
        ]
    });
    run(&mut s, "add", json!({"name": "base", "op": "sketch", "sketch": square}));
    run(&mut s, "add", json!({"name": "slab", "op": "extrude", "inputs": ["base"], "z0": 0, "z1": "t"}));
    run(&mut s, "add", json!({"name": "row", "op": "pattern_grid", "inputs": ["slab"], "count": ["n", 1], "step": [["w + 10", 0, 0], [0, 0, 0]]}));
    assert_eq!(volume(&mut s, "row"), 3.0 * 40.0 * 40.0 * 5.0);
    // A thicker slab: the sketch keeps its cache entry, slab and row are recomputed.
    let r = run(&mut s, "set_parameter", json!({"name": "t", "value": 6}));
    assert_eq!((r["computed"].as_u64(), r["reused"].as_u64()), (Some(2), Some(1)));
    assert_eq!(volume(&mut s, "row"), 3.0 * 40.0 * 40.0 * 6.0);
    let p = &run(&mut s, "inspect", json!({}))["parameters"];
    assert_eq!((&p["w"]["value"], &p["t"]["definition"]), (&json!(40.0), &json!(6.0)));

    // Mistakes are named where they bite.
    let failed =
        |r: &Value| r["failed"].as_array().unwrap().iter().map(|f| format!("{}: {}", f["feature"], f["error"])).collect::<Vec<_>>().join("; ");
    let r = run(&mut s, "set_parameter", json!({"name": "n", "value": "2.5"}));
    assert!(failed(&r).contains("a count must be a whole number"), "{}", failed(&r));
    let r = run(&mut s, "set_parameter", json!({"name": "w", "value": "t * size"}));
    assert!(failed(&r).contains("unknown parameter `size`"), "{}", failed(&r));
    let r = run(&mut s, "set_parameter", json!({"name": "w", "value": "t + w"}));
    assert!(failed(&r).contains("depends on itself"), "{}", failed(&r));
    assert!(s.execute("set_parameter", &json!({"name": "2x", "value": 1})).unwrap_err().contains("not a valid parameter name"));

    // Saved and opened again, bit for bit.
    run(&mut s, "set_parameter", json!({"name": "w", "value": 40}));
    run(&mut s, "set_parameter", json!({"name": "n", "value": 3}));
    let path = std::env::temp_dir().join(format!("datum-params-{}.json", std::process::id()));
    run(&mut s, "save", json!({"path": path.display().to_string()}));
    let mut t = Session::default();
    run(&mut t, "open", json!({"path": path.display().to_string()}));
    let _ = std::fs::remove_file(&path);
    assert_eq!(volume(&mut t, "row"), 3.0 * 40.0 * 40.0 * 6.0);
    assert_eq!(datum_model::to_json(t.graph()), datum_model::to_json(s.graph()));
}
