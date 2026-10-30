//! The `extrude` command (D185, D188): the mode given, the target body found as the window finds
//! it, the reply naming both.
#![allow(clippy::unwrap_used, clippy::panic)] // helpers outside #[test] functions

use datum_model::Session;
use serde_json::{Value, json};

fn run(s: &mut Session, cmd: &str, p: Value) -> Value {
    s.execute(cmd, &p).unwrap_or_else(|e| panic!("{cmd}: {e}"))
}

fn volume(s: &mut Session, name: &str) -> f64 {
    run(s, "mass_properties", json!({"name": name}))["volume"].as_f64().unwrap()
}

/// A rectangle from (x0, y0) to (x1, y1).
fn rectangle(x0: f64, y0: f64, x1: f64, y1: f64) -> Value {
    json!({"points": {"a": [x0, y0], "b": [x1, y0], "c": [x1, y1], "d": [x0, y1]},
        "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}, "cd": {"from": "c", "to": "d"}, "da": {"from": "d", "to": "a"}}})
}

/// Sketch `name` of `sketch` on the XY plane.
fn on_xy(s: &mut Session, name: &str, sketch: Value) {
    run(s, "add", json!({"name": name, "op": "sketch", "sketch": sketch}));
}

/// A plate 100 × 50 × 10 and, apart from it, a block 50 × 50 × 10 at x 200…250.
fn two_bodies() -> Session {
    let mut s = Session::default();
    on_xy(&mut s, "outline", rectangle(0.0, 0.0, 100.0, 50.0));
    let r = run(&mut s, "extrude", json!({"name": "plate", "sketch": "outline", "mode": "new", "z1": 10}));
    assert_eq!((r["mode"].as_str(), r["body"].as_str()), (Some("new"), Some("plate")));
    on_xy(&mut s, "block_outline", rectangle(200.0, 0.0, 250.0, 50.0));
    run(&mut s, "extrude", json!({"name": "block", "sketch": "block_outline", "mode": "new", "z1": 10}));
    s
}

#[test]
fn on_a_face_the_body_of_the_face() {
    let mut s = two_bodies();
    let top = run(&mut s, "query_faces", json!({"name": "plate", "normal": [0, 0, 1]}))["faces"][0]["id"].as_str().unwrap().to_string();
    let hole = json!({"points": {"h": [30, 25]}, "circles": {"c": {"center": "h", "diameter": 10}}});
    run(&mut s, "add", json!({"name": "hole_sketch", "op": "sketch", "inputs": ["plate"], "face": top, "sketch": hole}));
    let r = run(&mut s, "extrude", json!({"name": "pocket", "sketch": "hole_sketch", "mode": "cut", "z1": -4}));
    assert_eq!((r["mode"].as_str(), r["body"].as_str()), (Some("cut"), Some("plate")));
    let pocket = volume(&mut s, "pocket");
    assert!((pocket - (50_000.0 - 25.0 * std::f64::consts::PI * 4.0)).abs() < 1e-6, "{pocket}");
    // The next one on the same face goes to the body's last feature, the pocket.
    let boss = json!({"points": {"h": [70, 25]}, "circles": {"c": {"center": "h", "diameter": 10}}});
    run(&mut s, "add", json!({"name": "boss_sketch", "op": "sketch", "inputs": ["plate"], "face": top, "sketch": boss}));
    run(&mut s, "extrude", json!({"name": "boss", "sketch": "boss_sketch", "mode": "join", "z1": 5}));
    let inputs = &run(&mut s, "inspect", json!({}))["features"];
    let boss = inputs.as_array().unwrap().iter().find(|f| f["name"] == "boss").unwrap();
    assert_eq!(boss["inputs"], json!(["pocket", "boss_sketch"]), "{boss}");
}

#[test]
fn off_the_faces_the_body_it_meets() {
    let mut s = two_bodies();
    on_xy(&mut s, "rib_outline", rectangle(10.0, 10.0, 20.0, 40.0));
    let r = run(&mut s, "extrude", json!({"name": "rib", "sketch": "rib_outline", "mode": "join", "z0": 5, "z1": 20}));
    assert_eq!(r["body"].as_str(), Some("plate"));
    assert!((volume(&mut s, "rib") - (50_000.0 + 10.0 * 30.0 * 10.0)).abs() < 1e-6);

    // Over both bodies: which one is asked for, then given.
    on_xy(&mut s, "bar_outline", rectangle(50.0, 10.0, 220.0, 20.0));
    let e = s.execute("extrude", &json!({"name": "bar", "sketch": "bar_outline", "mode": "cut", "z0": 5, "z1": 20})).unwrap_err();
    assert!(e.contains("`plate`, `block`: give `body`"), "{e}");
    let r = run(&mut s, "extrude", json!({"name": "bar", "sketch": "bar_outline", "mode": "cut", "z0": 5, "z1": 20, "body": "block"}));
    assert_eq!(r["body"].as_str(), Some("block"));

    // Touching a body is not meeting it; far away, nothing to cut.
    on_xy(&mut s, "beside", rectangle(100.0, 0.0, 120.0, 50.0));
    let e = s.execute("extrude", &json!({"name": "x", "sketch": "beside", "mode": "cut", "z1": 10})).unwrap_err();
    assert!(e.contains("meets no body: nothing to cut"), "{e}");
}

#[test]
fn the_mode_is_given() {
    let mut s = two_bodies();
    on_xy(&mut s, "more", rectangle(0.0, 0.0, 10.0, 10.0));
    let e = s.execute("extrude", &json!({"name": "x", "sketch": "more", "z1": 3})).unwrap_err();
    assert!(e.contains("needs `mode`"), "{e}");
    let e = s.execute("add", &json!({"name": "x", "op": "extrude", "inputs": ["more"], "z1": 3})).unwrap_err();
    assert!(e.contains("needs `mode`"), "{e}");
    let e = s.execute("extrude", &json!({"name": "x", "sketch": "more", "mode": "join", "z1": 3, "body": "nothing"})).unwrap_err();
    assert!(e.contains("no body `nothing`; the bodies are `plate`, `block`"), "{e}");
    let e = s.execute("extrude", &json!({"name": "x", "sketch": "plate", "mode": "new", "z1": 3})).unwrap_err();
    assert!(e.contains("not a sketch"), "{e}");
}

/// A preview (D185 item 4): the extrusion seen, then gone without a trace; undo still undoes the
/// last change made.
#[test]
fn a_trial_leaves_no_trace() {
    let mut s = two_bodies();
    on_xy(&mut s, "rib_outline", rectangle(10.0, 10.0, 20.0, 40.0));
    let params = json!({"name": "rib", "sketch": "rib_outline", "mode": "join", "z0": 5, "z1": 20});
    let seen = s.trial("extrude", &params, |s, reply| (s.graph().find("rib").is_some(), reply["body"].clone())).unwrap();
    assert_eq!(seen, (true, json!("plate")));
    assert!(s.graph().find("rib").is_none());
    assert!(s.trial("extrude", &json!({"name": "x", "sketch": "rib_outline", "z1": 3}), |_, _| ()).unwrap_err().contains("needs `mode`"));
    run(&mut s, "undo", json!({}));
    assert!(s.graph().find("rib_outline").is_none(), "the sketch added last is undone");
    run(&mut s, "redo", json!({}));
    run(&mut s, "extrude", params);
    assert!((volume(&mut s, "rib") - (50_000.0 + 3_000.0)).abs() < 1e-6);
}
