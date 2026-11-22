//! Rule-driven fillets and chamfers on a convex prism edge.
#![allow(clippy::unwrap_used, clippy::panic)]

use datum_model::Session;
use serde_json::{Value, json};

const MODEL: &str = include_str!("../../../tests/models/edge-finish.json");

fn run(s: &mut Session, command: &str, params: Value) -> Value {
    s.execute(command, &params).unwrap_or_else(|error| panic!("{command}: {error}"))
}

fn close(a: f64, b: f64) {
    assert!((a - b).abs() < 1e-9 * a.abs().max(b.abs()).max(1.0), "{a} vs {b}");
}

fn session() -> Session {
    let model: Value = serde_json::from_str(MODEL).unwrap();
    let mut s = Session::default();
    for feature in &model["features"].as_array().unwrap()[..2] {
        run(&mut s, "add", feature.clone());
    }
    s
}

/// Find a prism corner edge from its geometry, never from a topology-vector index.
fn corner_edge(s: &mut Session, body: &str) -> String {
    let edges = run(s, "query_edges", json!({"name": body}));
    edges["edges"]
        .as_array()
        .unwrap()
        .iter()
        .find_map(|edge| {
            let a = &edge["from"]["point"];
            let b = &edge["to"]["point"];
            (a[0] == 0.0 && b[0] == 0.0 && a[1] == 0.0 && b[1] == 0.0 && a[2] != b[2]).then(|| edge["id"].as_str().unwrap().to_owned())
        })
        .unwrap_or_else(|| panic!("no vertical edge at prism corner in {body}: {edges}"))
}

fn volume(s: &mut Session, name: &str) -> f64 {
    run(s, "mass_properties", json!({"name": name}))["volume"].as_f64().unwrap()
}

fn finish(s: &mut Session, mode: &str, size: f64) {
    let edge = corner_edge(s, "box");
    run(s, "add", json!({"name": "finish", "op": "edge_finish", "inputs": ["box"], "edge": edge, "size": size, "mode": mode}));
    let result = run(s, "inspect", json!({}));
    let feature = result["features"].as_array().unwrap().iter().find(|f| f["name"] == "finish").unwrap();
    assert!(feature.get("error").is_none(), "{feature}");
}

fn volume_and_surfaces(s: &mut Session, mode: &str, size: f64) {
    finish(s, mode, size);
    let height = 10.0;
    let removed = if mode == "fillet" { size * size * (1.0 - std::f64::consts::PI / 4.0) * height } else { size * size / 2.0 * height };
    close(volume(s, "finish"), 20.0 * 12.0 * height - removed);
    let faces = run(s, "query_faces", json!({"name": "finish"}));
    if mode == "fillet" {
        assert!(faces["faces"].as_array().unwrap().iter().any(|face| face.get("cylinder").is_some_and(|c| c["radius"] == size)), "{faces}");
    } else {
        assert!(faces["faces"].as_array().unwrap().iter().all(|face| face.get("cylinder").is_none()), "{faces}");
        assert_eq!(faces["faces"].as_array().unwrap().len(), 7, "one planar chamfer face is added: {faces}");
    }
}

#[test]
fn edge_finish_builds_circular_fillet_and_planar_chamfer() {
    volume_and_surfaces(&mut session(), "fillet", 2.0);
    volume_and_surfaces(&mut session(), "chamfer", 2.0);
}

#[test]
fn edge_finish_size_edit_preserves_untouched_topology_ids() {
    let mut s = session();
    let edge = corner_edge(&mut s, "box");
    run(&mut s, "add", json!({"name": "finish", "op": "edge_finish", "inputs": ["box"], "edge": edge, "size": 1.0, "mode": "fillet"}));
    let before = run(&mut s, "query_faces", json!({"name": "finish"}));
    let untouched: Vec<String> = before["faces"]
        .as_array()
        .unwrap()
        .iter()
        .filter(|face| face["normal"] == json!([0.0, 0.0, 1.0]))
        .map(|face| face["id"].as_str().unwrap().to_owned())
        .collect();
    assert_eq!(untouched.len(), 1, "top face remains unchanged: {before}");
    run(&mut s, "edit", json!({"name": "finish", "size": 2.0}));
    let after = run(&mut s, "query_faces", json!({"name": "finish"}));
    let ids: Vec<&str> = after["faces"].as_array().unwrap().iter().map(|face| face["id"].as_str().unwrap()).collect();
    assert!(ids.contains(&untouched[0].as_str()), "unchanged top face lost its TopoId: {after}");
    close(volume(&mut s, "finish"), 2400.0 - 4.0 * (1.0 - std::f64::consts::PI / 4.0) * 10.0);
}

#[test]
fn edge_finish_rejects_a_size_that_does_not_fit_the_prism() {
    let mut s = session();
    let edge = corner_edge(&mut s, "box");
    let error = s.execute("add", &json!({"name": "too_large", "op": "edge_finish", "inputs": ["box"], "edge": edge, "size": 13.0, "mode": "fillet"}));
    assert!(
        error.is_err()
            || run(&mut s, "inspect", json!({}))["features"].as_array().unwrap().iter().any(|f| f["name"] == "too_large" && f.get("error").is_some()),
        "oversized finish unexpectedly succeeded"
    );
}

#[test]
fn edge_finish_accepts_a_rotated_prism() {
    let mut s = session();
    run(&mut s, "add", json!({"name": "turned_box", "op": "transform", "inputs": ["box"], "axis": [0, 0, 1], "angle_deg": 23}));
    let edge = corner_edge(&mut s, "turned_box");
    run(&mut s, "add", json!({"name": "turned_finish", "op": "edge_finish", "inputs": ["turned_box"], "edge": edge, "size": 2.0, "mode": "fillet"}));
    let faces = run(&mut s, "query_faces", json!({"name": "turned_finish"}));
    assert!(faces["faces"].as_array().unwrap().iter().any(|face| face.get("cylinder").is_some_and(|c| c["radius"] == 2.0)), "{faces}");
    close(volume(&mut s, "turned_finish"), 2400.0 - 4.0 * (1.0 - std::f64::consts::PI / 4.0) * 10.0);
}
