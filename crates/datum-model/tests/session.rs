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
