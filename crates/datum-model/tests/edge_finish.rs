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

fn profile_edges(s: &mut Session, body: &str) -> Vec<String> {
    let edges = run(s, "query_edges", json!({"name": body}));
    let mut ids: Vec<String> = edges["edges"]
        .as_array()
        .unwrap()
        .iter()
        .filter_map(|edge| {
            let a = &edge["from"]["point"];
            let b = &edge["to"]["point"];
            (a[0] == 0.0 && b[0] == 0.0 && a[1] == b[1] && a[2] != b[2]).then(|| edge["id"].as_str().unwrap().to_owned())
        })
        .collect();
    ids.sort();
    ids
}

fn feature(s: &mut Session, name: &str) -> Value {
    run(s, "inspect", json!({}))["features"]
        .as_array()
        .unwrap()
        .iter()
        .find(|f| f["name"] == name)
        .unwrap_or_else(|| panic!("no feature {name}"))
        .clone()
}

fn volume(s: &mut Session, name: &str) -> f64 {
    run(s, "mass_properties", json!({"name": name}))["volume"].as_f64().unwrap()
}

/// Two sets on one body build one on the other: the second set's input is the first set's
/// feature, and both fillets stand in the final body (D296).
#[test]
fn edge_finish_chains_on_the_body_it_works_on() {
    let mut s = session();
    let first = corner_edge(&mut s, "box");
    run(&mut s, "add", json!({"name": "finish1", "op": "edge_finish", "inputs": ["box"], "edge": first, "size": 2.0, "mode": "fillet"}));
    close(volume(&mut s, "finish1"), 2400.0 - (1.0 - std::f64::consts::PI / 4.0) * 4.0 * 10.0);
    // A second corner of the body after the first fillet still stands straight.
    let edges = run(&mut s, "query_edges", json!({"name": "finish1"}));
    let other = edges["edges"]
        .as_array()
        .unwrap()
        .iter()
        .find_map(|edge| {
            let a = &edge["from"]["point"];
            let b = &edge["to"]["point"];
            (a[0] == 20.0 && b[0] == 20.0 && a[1] == 0.0 && b[1] == 0.0 && a[2] != b[2]).then(|| edge["id"].as_str().unwrap().to_owned())
        })
        .unwrap_or_else(|| panic!("no second corner in {edges}"));
    run(&mut s, "add", json!({"name": "finish2", "op": "edge_finish", "inputs": ["finish1"], "edge": other, "size": 1.2, "mode": "fillet"}));
    close(volume(&mut s, "finish2"), 2400.0 - (1.0 - std::f64::consts::PI / 4.0) * (4.0 + 1.44) * 10.0);
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

#[test]
fn edge_finish_by_cap_rounds_every_eligible_profile_corner() {
    let mut s = session();
    let faces = run(&mut s, "query_faces", json!({"name": "box", "normal": [0, 0, 1]}));
    let face = faces["faces"][0]["id"].as_str().unwrap();
    run(&mut s, "add", json!({"name": "all_corners", "op": "edge_finish", "inputs": ["box"], "face": face, "size": 2.0, "mode": "fillet"}));
    let made = feature(&mut s, "all_corners");
    assert!(made.get("error").is_none(), "{made}");
    close(volume(&mut s, "all_corners"), 2400.0 - 4.0 * (4.0 - std::f64::consts::PI) * 10.0);
    let faces = run(&mut s, "query_faces", json!({"name": "all_corners"}));
    let cylinders = faces["faces"].as_array().unwrap().iter().filter(|f| f.get("cylinder").is_some()).count();
    assert_eq!(cylinders, 4, "each vertical profile edge receives a fillet: {faces}");
}

#[test]
fn edge_finish_cap_set_collapses_to_the_shared_corners() {
    // The two caps of a prism share their corner edges (the profile corners): a set of both
    // caps finishes the same four corners as one cap alone, and the hashes agree.
    let mut s = session();
    let top = run(&mut s, "query_faces", json!({"name": "box", "normal": [0, 0, 1]}))["faces"][0]["id"].take();
    let bottom = run(&mut s, "query_faces", json!({"name": "box", "normal": [0, 0, -1]}))["faces"][0]["id"].take();
    run(&mut s, "add", json!({"name": "both_caps", "op": "edge_finish", "inputs": ["box"], "faces": [top, bottom], "size": 1.0, "mode": "fillet"}));
    run(&mut s, "add", json!({"name": "one_cap", "op": "edge_finish", "inputs": ["box"], "faces": [top], "size": 1.0, "mode": "fillet"}));
    let both = feature(&mut s, "both_caps");
    let one = feature(&mut s, "one_cap");
    assert!(both.get("error").is_none() && one.get("error").is_none(), "{both} {one}");
    close(volume(&mut s, "both_caps"), 2400.0 - 4.0 * (1.0 - std::f64::consts::PI / 4.0) * 10.0);
    let faces = run(&mut s, "query_faces", json!({"name": "both_caps"}));
    let cylinders = faces["faces"].as_array().unwrap().iter().filter(|f| f.get("cylinder").is_some()).count();
    assert_eq!(cylinders, 4, "both caps name the same four corners: {faces}");
}

fn multi_edge_hash(edges: &[String]) -> String {
    let mut s = session();
    run(&mut s, "add", json!({"name": "multi", "op": "edge_finish", "inputs": ["box"], "edges": edges, "size": 2.0, "mode": "fillet"}));
    let made = feature(&mut s, "multi");
    assert!(made.get("error").is_none(), "{made}");
    close(volume(&mut s, "multi"), 2400.0 - 2.0 * (4.0 - std::f64::consts::PI) * 10.0);
    made["status"]["hash"].as_str().unwrap().to_owned()
}

#[test]
fn edge_finish_multiple_corners_is_order_independent_and_deduplicated() {
    let mut s = session();
    let edges = profile_edges(&mut s, "box");
    assert_eq!(edges.len(), 2, "the two profile corners at x=0: {edges:?}");
    let mut reversed = edges.clone();
    reversed.reverse();
    let mut duplicated = edges.clone();
    duplicated.push(edges[0].clone());
    let expected = multi_edge_hash(&edges);
    assert_eq!(multi_edge_hash(&reversed), expected, "reversing selection order changed the result");
    assert_eq!(multi_edge_hash(&duplicated), expected, "a duplicate selection changed the result");
}

#[test]
fn edge_finish_multiple_corners_fails_atomically_when_one_does_not_fit() {
    let mut s = session();
    let edges = profile_edges(&mut s, "box");
    run(&mut s, "add", json!({"name": "too_large", "op": "edge_finish", "inputs": ["box"], "edges": edges, "size": 13.0, "mode": "fillet"}));
    let failed = feature(&mut s, "too_large");
    assert!(failed.get("error").is_some(), "oversized multi-corner finish unexpectedly succeeded: {failed}");
    close(volume(&mut s, "box"), 2400.0);
    let still_valid = run(&mut s, "query_faces", json!({"name": "box"}));
    assert_eq!(still_valid["faces"].as_array().unwrap().len(), 6, "failed operation changed its input body: {still_valid}");
}

#[test]
fn edge_finish_combines_faces_and_edges_then_excludes_corners() {
    let mut s = session();
    let face = run(&mut s, "query_faces", json!({"name": "box", "normal": [0, 0, 1]}))["faces"][0]["id"].take();
    let edges = profile_edges(&mut s, "box");
    run(
        &mut s,
        "add",
        json!({"name": "mixed", "op": "edge_finish", "inputs": ["box"], "faces": [face], "edges": [edges[0]], "exclude_edges": [edges[1]], "size": 1.0}),
    );
    let made = feature(&mut s, "mixed");
    assert!(made.get("error").is_none(), "{made}");
    close(volume(&mut s, "mixed"), 2400.0 - 3.0 * (1.0 - std::f64::consts::PI / 4.0) * 10.0);
    run(
        &mut s,
        "add",
        json!({"name": "empty", "op": "edge_finish", "inputs": ["box"], "edges": [edges[0]], "exclude_edges": [edges[0]], "size": 1.0}),
    );
    let empty = feature(&mut s, "empty");
    assert!(empty.get("error").is_some(), "{empty}");
}
