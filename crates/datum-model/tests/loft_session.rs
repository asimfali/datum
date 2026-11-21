//! Loft-flange integration with sheet rules and the common flat-sketch path.
#![allow(clippy::unwrap_used, clippy::panic)]

use datum_model::Session;
use serde_json::{Value, json};

fn run(s: &mut Session, command: &str, params: Value) -> Value {
    s.execute(command, &params).unwrap_or_else(|e| panic!("{command}: {e}"))
}

#[test]
fn loft_uses_the_sheet_rule_and_flat_sketch() {
    check_loft("loft_session_rule_flat", include_str!("../../../tests/models/loft-02-bend.json"), "profile_a", "profile_b", "profile");
}

#[test]
fn trapezoid_wings_use_the_sheet_rule_and_flat_sketch() {
    check_loft("loft_session_trapezoid_wings", include_str!("../../../tests/models/loft-03-trapezoid-wings.json"), "sketch1", "sketch2", "l1");
}

#[test]
fn arc_bridge_follows_the_projected_profile() {
    check_arc_bridge("loft-04-arc-chords.json", false);
}

#[test]
fn concentric_arc_bridge_follows_the_projected_profile() {
    check_arc_bridge("loft-04-concentric-arcs.json", true);
}

#[test]
fn closed_loft_seam_gap_is_parametric_and_can_reset_to_its_default() {
    let root = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join("loft_session_seam_gap");
    std::fs::create_dir_all(root.join("part")).unwrap();
    std::fs::write(root.join("datum-project.json"), "{}").unwrap();
    let mut s = Session::default();
    run(&mut s, "save", json!({"path": root.join("part/loft.json")}));
    run(&mut s, "put_library", json!({"level": "material", "element": {"name": "Steel", "density": 7850}}));
    run(
        &mut s,
        "put_library",
        json!({"level": "rule", "element": {"name": "Sheet", "kind": "sheet", "material": "Steel", "thickness": 2, "radius": 2, "k_factor": 0.44}}),
    );
    let model: Value = serde_json::from_str(include_str!("../../../tests/models/loft-circle-rectangle.json")).unwrap();
    for feature in model["features"].as_array().unwrap().iter().take(3) {
        run(&mut s, "add", feature.clone());
    }
    let changed = run(&mut s, "set_parameter", json!({"name": "pitch", "value": 2.0}));
    assert!(changed["failed"].as_array().unwrap().is_empty(), "{changed}");
    let added =
        run(&mut s, "loft", json!({"name": "loft", "first": "sketch1", "second": "sketch2", "rule": "Sheet", "tolerance": 1, "gap": "pitch / 2"}));
    assert!(added["failed"].as_array().unwrap().is_empty(), "{added}");
    let volume = |s: &Session| s.value("loft").unwrap().body().unwrap().volume();
    let before = volume(&s);
    assert!(s.flat_of("loft").unwrap().overlaps.is_empty());

    let changed = run(&mut s, "set_parameter", json!({"name": "pitch", "value": 4.0}));
    assert!(changed["failed"].as_array().unwrap().is_empty(), "{changed}");
    assert!(volume(&s) < before);
    assert!(s.flat_of("loft").unwrap().overlaps.is_empty());

    let reset = run(&mut s, "edit", json!({"name": "loft", "gap": null}));
    assert!(reset["failed"].as_array().unwrap().is_empty(), "{reset}");
    assert_eq!(volume(&s), before);
}

fn check_arc_bridge(model: &str, fixed_second_radius: bool) {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/models").join(model);
    let mut s = Session::default();
    run(&mut s, "open", json!({"path": path}));
    let before = run(&mut s, "inspect", json!({}));
    let feature = |inspect: &Value, name: &str| inspect["features"].as_array().unwrap().iter().find(|f| f["name"] == name).unwrap().clone();
    let projected = feature(&before, "sketch2");
    let loft = feature(&before, "loft");
    let flat = s.flat_of("loft").unwrap();
    assert_eq!(flat.k_factor, 0.44);
    assert_eq!(flat.bend_ids.len(), 2);
    let ids = flat.bend_ids.clone();
    let changed = run(
        &mut s,
        "edit_sketch",
        json!({"name": "sketch1",
        "remove_constraints": [{"type": "radius", "arc": "a1", "value": 50}],
        "add_constraints": [{"type": "radius", "arc": "a1", "value": 51}]}),
    );
    assert!(changed["failed"].as_array().unwrap().is_empty(), "{changed}");
    let after = run(&mut s, "inspect", json!({}));
    let new_projected = feature(&after, "sketch2");
    let new_loft = feature(&after, "loft");
    assert_ne!(projected["status"]["hash"], new_projected["status"]["hash"]);
    assert_ne!(loft["status"]["hash"], new_loft["status"]["hash"]);
    assert_eq!(new_projected["project"]["e1"]["element"], "a1");
    if fixed_second_radius {
        for sketch in [&projected, &new_projected] {
            let points = &sketch["status"]["sketch"]["points"];
            let p = |name: &str, axis: usize| points[name][axis].as_f64().unwrap();
            let radius = libm::hypot(p("p1", 0) - p("p2", 0), p("p1", 1) - p("p2", 1));
            assert!((radius - 40.0).abs() < 1e-8, "{radius}");
        }
    }
    let flat = s.flat_of("loft").unwrap();
    assert_eq!(flat.k_factor, 0.44);
    assert_eq!(flat.bend_ids, ids);
}

#[test]
fn an_existing_loft_can_replace_its_profiles() {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/models/loft-01-line.json");
    let mut session = Session::default();
    run(&mut session, "open", json!({"path": path}));
    let edited = run(&mut session, "edit", json!({"name": "loft", "inputs": ["profile_b", "profile_a"], "tolerance": 0.5, "side": "middle"}));
    assert!(edited["failed"].as_array().unwrap().is_empty(), "{edited}");
    let inputs = |s: &Session| s.graph().nodes()[s.graph().find("loft").unwrap() as usize].inputs.clone();
    let expected = [session.graph().find("profile_b").unwrap(), session.graph().find("profile_a").unwrap()];
    assert_eq!(inputs(&session), expected);
    assert!(session.execute("edit", &json!({"name": "loft", "inputs": ["profile_a", "profile_a"]})).is_err());
    assert!(session.execute("edit", &json!({"name": "loft", "inputs": ["profile_a", "loft"]})).is_err());
    assert_eq!(inputs(&session), expected);
    assert!(session.value("loft").unwrap().body().is_some());
}

fn check_loft(test_name: &str, model: &str, first: &str, second: &str, line: &str) {
    let root = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join(test_name);
    std::fs::create_dir_all(root.join("part")).unwrap();
    std::fs::write(root.join("datum-project.json"), "{}").unwrap();
    let mut s = Session::default();
    run(&mut s, "save", json!({"path": root.join("part/loft.json")}));
    run(&mut s, "put_library", json!({"level": "material", "element": {"name": "Steel", "density": 7850}}));
    let rule = |t, k| json!({"level": "rule", "element": {"name": "Sheet", "kind": "sheet", "material": "Steel", "thickness": t, "radius": "t", "k_factor": k}});
    run(&mut s, "put_library", rule(2, 0.44));
    let model: Value = serde_json::from_str(model).unwrap();
    for feature in model["features"].as_array().unwrap().iter().take(3) {
        run(&mut s, "add", feature.clone());
    }
    run(&mut s, "loft", json!({"name": "loft", "first": first, "second": second, "rule": "Sheet", "tolerance": 1}));
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
        json!({"name": first,
        "remove_constraints": [{"type": "length", "line": line, "value": 100}],
        "add_constraints": [{"type": "length", "line": line, "value": 110}]}),
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
    let mut thicker = rule(4, 0.38);
    thicker["element"]["name"] = json!("Thicker");
    run(&mut s, "put_library", thicker);
    let changed = run(&mut s, "set_part", json!({"body": "loft", "rule": "Thicker"}));
    assert!(changed["parts"].as_array().unwrap().iter().any(|p| p["part"] == "loft" && p["rule"] == "Thicker"), "{changed}");
    let thicker = run(&mut s, "mass_properties", json!({"name": "cut"}))["volume"].as_f64().unwrap();
    assert!(thicker > after);
    assert_eq!(s.flat_of("cut").unwrap().k_factor, 0.38);
}
