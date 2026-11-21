//! Loft-flange integration with sheet rules and the common flat-sketch path.
#![allow(clippy::unwrap_used, clippy::panic)]

use datum_model::Session;
use serde_json::{Value, json};

fn run(s: &mut Session, command: &str, params: Value) -> Value {
    s.execute(command, &params).unwrap_or_else(|e| panic!("{command}: {e}"))
}

#[test]
fn loft_uses_the_sheet_rule_and_flat_sketch() {
    let root = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join("loft_session_rule_flat");
    std::fs::create_dir_all(root.join("part")).unwrap();
    std::fs::write(root.join("datum-project.json"), "{}").unwrap();
    let mut s = Session::default();
    run(&mut s, "save", json!({"path": root.join("part/loft.json")}));
    run(&mut s, "put_library", json!({"level": "material", "element": {"name": "Steel", "density": 7850}}));
    let rule = |t, k| json!({"level": "rule", "element": {"name": "Sheet", "kind": "sheet", "material": "Steel", "thickness": t, "radius": "t", "k_factor": k}});
    run(&mut s, "put_library", rule(2, 0.44));
    let model: Value = serde_json::from_str(include_str!("../../../tests/models/loft-02-bend.json")).unwrap();
    for feature in model["features"].as_array().unwrap().iter().take(3) {
        run(&mut s, "add", feature.clone());
    }
    run(&mut s, "loft", json!({"name": "loft", "first": "profile_a", "second": "profile_b", "rule": "Sheet", "tolerance": 1}));
    let flat = s.flats().into_iter().find(|(name, _)| name == "loft").unwrap().1.unwrap();
    assert_eq!(flat.k_factor, 0.44);
    let (bend_index, [a, b]) = flat.bends.iter().copied().enumerate().find(|(_, [a, b])| (a.x - b.x).abs() > 1.0 && (a.y - b.y).abs() > 1.0).unwrap();
    let mounting_index = flat
        .outer
        .clone()
        .zip(&flat.ends)
        .find(|(_, [a, b])| {
            let length_squared = (a.x - b.x).powi(2) + (a.y - b.y).powi(2);
            (150.0_f64.powi(2)..200.0_f64.powi(2)).contains(&length_squared)
        })
        .unwrap()
        .0;
    let project = json!({
        "mount": {"blank": format!("{:?}", flat.ids[mounting_index])},
        "bend": {"blank": format!("{:?}", flat.bend_ids[bend_index])},
    });
    run(
        &mut s,
        "add",
        json!({"name": "hole", "op": "sketch", "inputs": ["loft"], "flat": "front", "project": project, "sketch": {
            "points": {"h": [(a.x + b.x) * 0.5, (a.y + b.y) * 0.5]},
            "circles": {"c": {"center": "h", "diameter": 10}},
            "constraints": [{"type": "midpoint", "point": "h", "line": "bend"}]
        }}),
    );
    run(&mut s, "extrude", json!({"name": "cut", "sketch": "hole", "mode": "cut"}));
    let before = run(&mut s, "mass_properties", json!({"name": "cut"}))["volume"].as_f64().unwrap();
    let uncut = run(&mut s, "mass_properties", json!({"name": "loft"}))["volume"].as_f64().unwrap();
    assert!(before < uncut);
    let changed = run(
        &mut s,
        "edit_sketch",
        json!({"name": "profile_a",
        "remove_constraints": [{"type": "length", "line": "profile", "value": 100}],
        "add_constraints": [{"type": "length", "line": "profile", "value": 110}]}),
    );
    assert!(changed["failed"].as_array().unwrap().is_empty(), "{changed}");
    run(&mut s, "put_library", rule(3, 0.4));
    let inspect = run(&mut s, "inspect", json!({}));
    for name in ["loft", "hole", "cut"] {
        let f = inspect["features"].as_array().unwrap().iter().find(|f| f["name"] == name).unwrap();
        assert!(f.get("error").is_none() && f.get("status").is_some(), "{f}");
    }
    let points = inspect["features"].as_array().unwrap().iter().find(|f| f["name"] == "hole").unwrap()["status"]["sketch"]["points"].clone();
    for name in ["mount.start", "mount.end", "bend.start", "bend.end"] {
        assert!(points.get(name).is_some(), "{points}");
    }
    let after = run(&mut s, "mass_properties", json!({"name": "cut"}))["volume"].as_f64().unwrap();
    assert!(after > before, "{before} -> {after}");
    let flat = s.flat_of("cut").unwrap();
    assert_eq!(flat.k_factor, 0.4);
    assert_eq!(flat.counts()[2], 1);
    let path = root.join("cut.dxf");
    let export = run(&mut s, "export_dxf", json!({"name": "cut", "path": path}));
    assert_eq!(export["polylines"], 0, "{export}");
    let dxf = std::fs::read_to_string(path).unwrap();
    assert_eq!(dxf.lines().filter(|line| line.trim() == "CIRCLE").count(), 1);
}
