//! Projection into a sketch from another body and from another sketch (D214), through the
//! session: edges by `id`, elements by name, the names following a rename, a lost element.
#![allow(clippy::unwrap_used, clippy::panic)] // helpers outside #[test] functions

use datum_model::Session;
use serde_json::{Value, json};

fn run(s: &mut Session, cmd: &str, p: Value) -> Value {
    s.execute(cmd, &p).unwrap_or_else(|e| panic!("{cmd}: {e}"))
}

fn volume(s: &mut Session, name: &str) -> f64 {
    run(s, "mass_properties", json!({"name": name}))["volume"].as_f64().unwrap()
}

fn feature(s: &mut Session, name: &str) -> Value {
    run(s, "inspect", json!({}))["features"].as_array().unwrap().iter().find(|f| f["name"] == name).unwrap_or_else(|| panic!("no {name}")).clone()
}

fn corner_ids(s: &mut Session, body: &str, face_normal: [i32; 3]) -> Vec<(String, [f64; 3])> {
    let face = run(s, "query_faces", json!({"name": body, "normal": face_normal}))["faces"][0]["id"].as_str().unwrap().to_string();
    let edges = run(s, "query_edges", json!({"name": body, "face": face}));
    let mut ends = Vec::new();
    for e in edges["edges"].as_array().unwrap() {
        for end in ["from", "to"] {
            let v = &e[end];
            let id = v["id"].as_str().unwrap().to_string();
            if !ends.iter().any(|(i, _): &(String, [f64; 3])| *i == id) {
                ends.push((id, serde_json::from_value(v["point"].clone()).unwrap()));
            }
        }
    }
    ends
}

/// Two bodies in the root; a sketch on a work plane at the top of the second projects its
/// top corners and an edge, joins the corners, and the extrusion builds.
#[test]
fn a_sketch_projects_from_the_second_body_of_the_root() {
    let rect = |x0: f64, x1: f64| {
        json!({"points": {"a": [x0, 0], "b": [x1, 0], "c": [x1, 50], "d": [x0, 50]},
            "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}, "cd": {"from": "c", "to": "d"}, "da": {"from": "d", "to": "a"}}})
    };
    let mut s = Session::default();
    for (n, x0, x1) in [("one", 0.0, 100.0), ("two", 200.0, 250.0)] {
        run(&mut s, "add", json!({"name": format!("{n}_outline"), "op": "sketch", "sketch": rect(x0, x1)}));
        run(&mut s, "extrude", json!({"name": n, "sketch": format!("{n}_outline"), "mode": "new", "z1": 10}));
    }
    let corners = corner_ids(&mut s, "two", [0, 0, 1]);
    assert_eq!(corners.len(), 4);
    let at = |x: f64, y: f64| corners.iter().find(|(_, p)| p[0] == x && p[1] == y).unwrap().0.clone();
    run(&mut s, "add", json!({"name": "wp", "op": "plane", "base": "xy", "offset": 10}));
    let edges = run(&mut s, "query_edges", json!({"name": "two"}));
    let front = edges["edges"]
        .as_array()
        .unwrap()
        .iter()
        .find(|e| e["from"]["point"][1] == 0.0 && e["to"]["point"][1] == 0.0 && e["from"]["point"][2] == 10.0)
        .unwrap();
    let project: Value = json!({
        "a": {"from": "two", "id": at(200.0, 0.0)}, "b": {"from": "two", "id": at(250.0, 0.0)},
        "c": {"from": "two", "id": at(250.0, 50.0)}, "d": {"from": "two", "id": at(200.0, 50.0)},
        "front": {"from": "two", "id": front["id"], "construction": false}
    });
    let outline = json!({"points": {}, "lines": {
        "l1": {"from": "a", "to": "b"}, "l2": {"from": "b", "to": "c"}, "l3": {"from": "c", "to": "d"}, "l4": {"from": "d", "to": "a"}}});
    run(&mut s, "add", json!({"name": "copy", "op": "sketch", "inputs": ["wp", "two"], "project": project, "sketch": outline}));
    let f = feature(&mut s, "copy");
    assert_eq!(f["status"]["sketch"]["points"]["c"], json!([250.0, 50.0]), "{f}");
    // `at` was recorded for every projection.
    assert!(f["project"].as_object().unwrap().values().all(|p| p["at"].is_array()), "{f}");
    run(&mut s, "extrude", json!({"name": "lid", "sketch": "copy", "mode": "new", "z1": 4}));
    assert!((volume(&mut s, "lid") - 50.0 * 50.0 * 4.0).abs() < 1e-6);
    // The first body is untouched by it.
    assert!((volume(&mut s, "one") - 100.0 * 50.0 * 10.0).abs() < 1e-6);
}

fn source(s: &mut Session) {
    run(s, "set_parameter", json!({"name": "w", "value": 30}));
    run(
        s,
        "add",
        json!({"name": "skel", "op": "sketch", "sketch": {
            "points": {"a": [0, 0], "b": [30, 0]}, "lines": {"l": {"from": "a", "to": "b"}},
            "constraints": [{"type": "fix", "point": "a"}, {"type": "horizontal", "line": "l"}, {"type": "length", "line": "l", "value": "w"}]}}),
    );
    run(s, "add", json!({"name": "wp", "op": "plane", "base": "xy", "offset": 10}));
    run(
        s,
        "add",
        json!({"name": "t", "op": "sketch", "inputs": ["wp", "skel"], "project": {"ln": {"from": "skel", "element": "l"}},
            "sketch": {"points": {}, "measures": {"len": {"type": "horizontal_distance", "points": ["ln.start", "ln.end"]}}}}),
    );
}

fn end_of(s: &mut Session) -> Value {
    feature(s, "t")["status"]["sketch"]["points"]["ln.end"].clone()
}

/// Sketch to sketch is associative: a parameter, a rename and a removal in the source reach the
/// sketch that projects from it.
#[test]
fn a_sketch_follows_the_sketch_it_projects_from() {
    let mut s = Session::default();
    source(&mut s);
    assert_eq!(end_of(&mut s), json!([30.0, 0.0]));
    assert_eq!(feature(&mut s, "t")["project"]["ln"]["at"], json!([[0.0, 0.0], [30.0, 0.0]]));
    run(&mut s, "set_parameter", json!({"name": "w", "value": 55}));
    assert_eq!(end_of(&mut s), json!([55.0, 0.0]));
    assert_eq!(feature(&mut s, "t")["status"]["sketch"]["measures"]["len"], json!(55.0));

    run(&mut s, "rename_in_sketch", json!({"name": "skel", "from": "l", "to": "leg"}));
    let t = feature(&mut s, "t");
    assert_eq!(t["project"]["ln"]["element"], json!("leg"), "{t}");
    assert!(t.get("error").is_none() && end_of(&mut s) == json!([55.0, 0.0]), "{t}");

    // The element removed: the projection is a lost reference, named.
    let r = run(
        &mut s,
        "edit_sketch",
        json!({"name": "skel", "lines": {"leg": null}, "remove_constraints": [{"type": "length", "line": "leg", "value": "w"}]}),
    );
    let failed = r["failed"].as_array().unwrap();
    let text = failed.iter().map(|f| f.to_string()).collect::<Vec<_>>().join("; ");
    assert!(text.contains("\"t\"") && text.contains("leg"), "{r}");
}
