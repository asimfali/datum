//! A mirror of a part (D247): reflected, a new part; turned, a live copy of the same part to see
//! and to take geometry from, which follows the end of the part and takes part in nothing.
#![allow(clippy::unwrap_used, clippy::panic)] // helpers outside #[test] functions

use datum_model::Session;
use serde_json::{Value, json};

fn run(s: &mut Session, cmd: &str, p: Value) -> Value {
    s.execute(cmd, &p).unwrap_or_else(|e| panic!("{cmd}: {e}"))
}

fn volume(s: &mut Session, name: &str) -> f64 {
    run(s, "mass_properties", json!({"name": name}))["volume"].as_f64().unwrap()
}

fn centre(s: &mut Session, name: &str) -> [f64; 3] {
    let c = &run(s, "mass_properties", json!({"name": name}))["centroid"];
    [c[0].as_f64().unwrap(), c[1].as_f64().unwrap(), c[2].as_f64().unwrap()]
}

fn rectangle(x0: f64, y0: f64, x1: f64, y1: f64) -> Value {
    json!({"points": {"a": [x0, y0], "b": [x1, y0], "c": [x1, y1], "d": [x0, y1]},
        "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}, "cd": {"from": "c", "to": "d"}, "da": {"from": "d", "to": "a"}}})
}

fn top_of(s: &mut Session, body: &str) -> String {
    run(s, "query_faces", json!({"name": body, "normal": [0, 0, 1]}))["faces"][0]["id"].as_str().unwrap().to_string()
}

/// A plate 100 × 40 × 10 at x 20…120, y 10…50: off every origin plane, so a mirror shows.
fn plate() -> Session {
    let mut s = Session::default();
    run(&mut s, "add", json!({"name": "outline", "op": "sketch", "sketch": rectangle(20.0, 10.0, 120.0, 50.0)}));
    run(&mut s, "extrude", json!({"name": "plate", "sketch": "outline", "mode": "new", "z1": 10}));
    s
}

/// A round hole through the part ending in `body`, feature `name`, at (x, y) on its top.
fn drill(s: &mut Session, name: &str, body: &str, x: f64, y: f64) {
    let top = top_of(s, body);
    let sketch = format!("{name}_sketch");
    let hole = json!({"points": {"h": [x, y]}, "circles": {"c": {"center": "h", "diameter": 10}}});
    run(s, "add", json!({"name": sketch, "op": "sketch", "inputs": [body], "face": top, "sketch": hole}));
    run(s, "extrude", json!({"name": name, "sketch": sketch, "mode": "cut", "through": true}));
}

fn feature(s: &mut Session, name: &str) -> Value {
    run(s, "inspect", json!({}))["features"].as_array().unwrap().iter().find(|f| f["name"] == name).unwrap().clone()
}

fn parts(s: &mut Session) -> Vec<String> {
    run(s, "levels", json!({}))["parts"].as_array().unwrap().iter().map(|p| p["part"].as_str().unwrap().to_string()).collect()
}

fn near(a: [f64; 3], b: [f64; 3]) {
    assert!(a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-9), "{a:?} is not {b:?}");
}

#[test]
fn a_turn_goes_about_the_chosen_axis_of_the_plane() {
    // XZ has its X along world X, its Y along world Z and its normal along -Y.
    for (axis, at) in [("x", [70.0, -30.0, -5.0]), ("y", [-70.0, -30.0, 5.0]), ("normal", [-70.0, 30.0, -5.0])] {
        let mut s = plate();
        run(&mut s, "add", json!({"name": "copy", "op": "mirror", "inputs": ["plate"], "base": "xz", "turn": axis}));
        near(centre(&mut s, "copy"), at);
    }
}

#[test]
fn a_reflection_is_a_new_part_on_the_other_side() {
    let mut s = plate();
    drill(&mut s, "hole", "plate", 40.0, 20.0);
    run(&mut s, "add", json!({"name": "other", "op": "mirror", "inputs": ["hole"], "base": "yz"}));
    assert_eq!(parts(&mut s), ["plate", "other"]);
    let (v, c) = (volume(&mut s, "hole"), centre(&mut s, "hole"));
    assert!((volume(&mut s, "other") - v).abs() < 1e-6 * v);
    near(centre(&mut s, "other"), [-c[0], c[1], c[2]]);
}

#[test]
fn across_a_work_plane_and_a_face() {
    let mut s = plate();
    run(&mut s, "add", json!({"name": "wall", "op": "plane", "base": "yz", "offset": 200}));
    run(&mut s, "add", json!({"name": "by_plane", "op": "mirror", "inputs": ["plate", "wall"]}));
    near(centre(&mut s, "by_plane"), [330.0, 30.0, 5.0]);
    let top = top_of(&mut s, "plate");
    run(&mut s, "add", json!({"name": "by_face", "op": "mirror", "inputs": ["plate"], "face": top}));
    near(centre(&mut s, "by_face"), [70.0, 30.0, 15.0]);
}

#[test]
fn a_mirror_goes_across_one_thing() {
    let mut s = plate();
    assert!(s.execute("add", &json!({"name": "m", "op": "mirror", "inputs": ["plate"]})).is_err_or_failed());
}

trait Refused {
    fn is_err_or_failed(&self) -> bool;
}

impl Refused for Result<Value, String> {
    fn is_err_or_failed(&self) -> bool {
        self.as_ref().map_or(true, |r| r["failed"].as_array().is_some_and(|f| !f.is_empty()))
    }
}

#[test]
fn the_part_mirrored_stays_and_the_copy_is_no_part() {
    let mut s = plate();
    run(&mut s, "add", json!({"name": "copy", "op": "mirror", "inputs": ["plate"], "base": "xz", "turn": "y"}));
    assert_eq!(parts(&mut s), ["plate"]);
    assert!(volume(&mut s, "plate") > 0.0);
}

#[test]
fn a_copy_follows_the_end_of_its_part() {
    let mut s = plate();
    run(&mut s, "add", json!({"name": "copy", "op": "mirror", "inputs": ["plate"], "base": "xz", "turn": "y"}));
    drill(&mut s, "hole", "plate", 40.0, 20.0);
    assert_eq!(feature(&mut s, "copy")["inputs"], json!(["hole"]));
    let (v, c) = (volume(&mut s, "hole"), centre(&mut s, "hole"));
    assert!((volume(&mut s, "copy") - v).abs() < 1e-6 * v, "the hole reaches the copy");
    near(centre(&mut s, "copy"), [-c[0], -c[1], c[2]]);
    // Added again: further on.
    drill(&mut s, "hole2", "hole", 100.0, 40.0);
    assert_eq!(feature(&mut s, "copy")["inputs"], json!(["hole2"]));
    let v = volume(&mut s, "hole2");
    assert!((volume(&mut s, "copy") - v).abs() < 1e-6 * v);
}

#[test]
fn a_copy_following_its_part_is_no_change_of_shape_to_warn_of() {
    let mut s = plate();
    run(&mut s, "add", json!({"name": "copy", "op": "mirror", "inputs": ["plate"], "base": "xz", "turn": "y"}));
    let top = top_of(&mut s, "plate");
    let hole = json!({"points": {"h": [40, 20]}, "circles": {"c": {"center": "h", "diameter": 10}}});
    run(&mut s, "add", json!({"name": "s", "op": "sketch", "inputs": ["plate"], "face": top, "sketch": hole}));
    let reply = run(&mut s, "extrude", json!({"name": "hole", "sketch": "s", "mode": "cut", "through": true}));
    assert!(reply.get("shape_changed").is_none(), "{reply}");
}

#[test]
fn a_copy_goes_back_with_the_end_of_its_part() {
    let mut s = plate();
    run(&mut s, "add", json!({"name": "copy", "op": "mirror", "inputs": ["plate"], "base": "xz", "turn": "y"}));
    drill(&mut s, "hole", "plate", 40.0, 20.0);
    run(&mut s, "remove", json!({"name": "hole"}));
    assert_eq!(feature(&mut s, "copy")["inputs"], json!(["plate"]));
    let v = volume(&mut s, "plate");
    assert!((volume(&mut s, "copy") - v).abs() < 1e-6 * v);
}

#[test]
fn undo_takes_a_copy_back_too() {
    let mut s = plate();
    run(&mut s, "add", json!({"name": "copy", "op": "mirror", "inputs": ["plate"], "base": "xz", "turn": "y"}));
    let before = volume(&mut s, "copy");
    drill(&mut s, "hole", "plate", 40.0, 20.0);
    run(&mut s, "undo", json!({}));
    run(&mut s, "undo", json!({}));
    assert_eq!(feature(&mut s, "copy")["inputs"], json!(["plate"]));
    assert!((volume(&mut s, "copy") - before).abs() < 1e-9);
}

#[test]
fn a_copy_takes_part_in_no_operation() {
    let mut s = plate();
    run(&mut s, "add", json!({"name": "copy", "op": "mirror", "inputs": ["plate"], "base": "xz", "turn": "y"}));
    run(&mut s, "add", json!({"name": "tool", "op": "sketch", "sketch": rectangle(-100.0, -40.0, -60.0, -20.0)}));
    let why = s.execute("add", &json!({"name": "stuck", "op": "extrude", "mode": "join", "inputs": ["copy", "tool"], "z1": 5})).unwrap_err();
    assert!(why.contains("turned copy"), "{why}");
    let why = s.execute("extrude", &json!({"name": "stuck", "sketch": "tool", "mode": "join", "body": "copy", "z1": 5})).unwrap_err();
    assert!(why.contains("no body"), "{why}");
    let why = s.execute("add", &json!({"name": "again", "op": "mirror", "inputs": ["copy"], "base": "yz"})).unwrap_err();
    assert!(why.contains("turned copy"), "{why}");
}

#[test]
fn an_extrusion_that_meets_only_a_copy_has_nothing_to_join() {
    let mut s = plate();
    run(&mut s, "add", json!({"name": "copy", "op": "mirror", "inputs": ["plate"], "base": "xz", "turn": "y"}));
    run(&mut s, "add", json!({"name": "tool", "op": "sketch", "sketch": rectangle(-100.0, -40.0, -60.0, -20.0)}));
    let why = s.execute("extrude", &json!({"name": "stuck", "sketch": "tool", "mode": "join", "z1": 5})).unwrap_err();
    assert!(why.contains("meets no body"), "{why}");
}

#[test]
fn a_neighbour_is_made_from_a_copy_and_follows_it() {
    let mut s = plate();
    run(&mut s, "add", json!({"name": "copy", "op": "mirror", "inputs": ["plate"], "base": "xz", "turn": "y"}));
    let top = top_of(&mut s, "copy");
    run(&mut s, "add", json!({"name": "on_copy", "op": "sketch", "inputs": ["copy"], "face": top, "sketch": rectangle(-100.0, -40.0, -60.0, -20.0)}));
    run(&mut s, "extrude", json!({"name": "neighbour", "sketch": "on_copy", "mode": "new", "z1": 3}));
    assert_eq!(parts(&mut s), ["plate", "neighbour"]);
    // The plate grows; its copy goes after the growth, the neighbour still stands on the copy.
    drill(&mut s, "hole", "plate", 40.0, 20.0);
    let names: Vec<String> =
        run(&mut s, "inspect", json!({}))["features"].as_array().unwrap().iter().map(|f| f["name"].as_str().unwrap().to_string()).collect();
    let at = |n: &str| names.iter().position(|x| x == n).unwrap();
    assert!(at("hole") < at("copy") && at("copy") < at("on_copy") && at("on_copy") < at("neighbour"), "{names:?}");
    assert!(run(&mut s, "inspect", json!({}))["failed"].as_array().is_none_or(Vec::is_empty));
    assert!(volume(&mut s, "neighbour") > 0.0);
}

#[test]
fn a_part_cannot_stand_on_its_own_copy() {
    let mut s = plate();
    run(&mut s, "add", json!({"name": "copy", "op": "mirror", "inputs": ["plate"], "base": "xz", "turn": "y"}));
    let top = top_of(&mut s, "plate");
    let hole = json!({"points": {"h": [40, 20]}, "circles": {"c": {"center": "h", "diameter": 10}}});
    run(&mut s, "add", json!({"name": "s", "op": "sketch", "inputs": ["plate", "copy"], "face": top, "sketch": hole}));
    let before = feature(&mut s, "copy");
    let why = s.execute("extrude", &json!({"name": "hole", "sketch": "s", "mode": "cut", "through": true})).unwrap_err();
    assert!(why.contains("own copy"), "{why}");
    assert_eq!(feature(&mut s, "copy"), before, "the model is as it was");
}

#[test]
fn a_reflected_part_follows_and_goes_on_by_itself() {
    let mut s = plate();
    run(&mut s, "add", json!({"name": "other", "op": "mirror", "inputs": ["plate"], "base": "yz"}));
    drill(&mut s, "only_other", "other", -40.0, 20.0);
    drill(&mut s, "hole", "plate", 100.0, 40.0);
    assert_eq!(feature(&mut s, "other")["inputs"], json!(["hole"]));
    let v = volume(&mut s, "hole");
    assert!(volume(&mut s, "only_other") < v, "its own hole and the plate's");
    assert_eq!(parts(&mut s), ["plate", "other"]);
}

#[test]
fn a_copy_is_turned_into_a_reflection_and_back() {
    let mut s = plate();
    run(&mut s, "add", json!({"name": "m", "op": "mirror", "inputs": ["plate"], "base": "xz", "turn": "y"}));
    run(&mut s, "edit", json!({"name": "m", "turn": null}));
    assert_eq!(parts(&mut s), ["plate", "m"]);
    near(centre(&mut s, "m"), [70.0, -30.0, 5.0]);
    run(&mut s, "edit", json!({"name": "m", "turn": "normal"}));
    assert_eq!(parts(&mut s), ["plate"]);
    // A reflection something is built on cannot become a copy.
    run(&mut s, "edit", json!({"name": "m", "turn": null}));
    drill(&mut s, "own", "m", 40.0, -30.0);
    let why = s.execute("edit", &json!({"name": "m", "turn": "y"})).unwrap_err();
    assert!(why.contains("turned copy"), "{why}");
}

#[test]
fn a_reflected_cylinder_keeps_its_volume() {
    let mut s = Session::default();
    let round = json!({"points": {"h": [30, 25]}, "circles": {"c": {"center": "h", "diameter": 20}}});
    run(&mut s, "add", json!({"name": "o", "op": "sketch", "sketch": round}));
    run(&mut s, "extrude", json!({"name": "pin", "sketch": "o", "mode": "new", "z1": 15}));
    run(&mut s, "add", json!({"name": "other", "op": "mirror", "inputs": ["pin"], "base": "xz"}));
    let v = volume(&mut s, "pin");
    assert!((volume(&mut s, "other") - v).abs() < 1e-6 * v);
    near(centre(&mut s, "other"), [30.0, -25.0, 7.5]);
}

#[test]
fn a_copy_is_not_exported() {
    let root = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join("mirror_step");
    std::fs::create_dir_all(&root).unwrap();
    let (mut s, mut alone) = (plate(), plate());
    run(&mut s, "add", json!({"name": "copy", "op": "mirror", "inputs": ["plate"], "base": "xz", "turn": "y"}));
    let (a, b) = (root.join("with.step"), root.join("alone.step"));
    run(&mut s, "export_step", json!({"path": a}));
    run(&mut alone, "export_step", json!({"path": b}));
    let solids = |p: &std::path::Path| std::fs::read_to_string(p).unwrap().matches("MANIFOLD_SOLID_BREP").count();
    assert_eq!((solids(&a), solids(&b)), (1, 1));
}

/// A session in a project with the sheet rule `Лист` (thickness 2, radius `t`), and a plate of it
/// 100 × 40 at x 20…120 with a flange on one long side.
fn sheet(name: &str) -> Session {
    let root = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join(name);
    let _ = std::fs::remove_dir_all(&root);
    std::fs::create_dir_all(root.join("part")).unwrap();
    std::fs::write(root.join("datum-project.json"), "{}").unwrap();
    let mut s = Session::default();
    run(&mut s, "save", json!({"path": root.join("part/sheet.json")}));
    run(&mut s, "put_library", json!({"level": "material", "element": {"name": "Сталь", "density": 7850}}));
    let rule = json!({"name": "Лист", "kind": "sheet", "material": "Сталь", "thickness": 2, "radius": "t", "k_factor": 0.4});
    run(&mut s, "put_library", json!({"level": "rule", "element": rule}));
    run(&mut s, "add", json!({"name": "outline", "op": "sketch", "sketch": rectangle(20.0, 10.0, 120.0, 50.0)}));
    run(&mut s, "plate", json!({"name": "plate", "sketch": "outline", "rule": "Лист"}));
    let edges = run(&mut s, "query_edges", json!({"name": "plate"}));
    let long = edges["edges"].as_array().unwrap().iter().find(|e| {
        let (a, b) = (&e["from"]["point"], &e["to"]["point"]);
        a[1] == 50.0 && b[1] == 50.0 && a[2] == b[2] && a[2] == 2.0
    });
    let edge = long.unwrap_or_else(|| panic!("no long edge in {edges}"))["id"].as_str().unwrap().to_string();
    run(&mut s, "add", json!({"name": "wall", "op": "flange", "inputs": ["plate"], "edges": [edge], "height": 20, "radius": 2}));
    s
}

fn flats(s: &mut Session) -> Vec<(String, f64, f64)> {
    let list = run(s, "flats", json!({}));
    list["flats"]
        .as_array()
        .unwrap_or_else(|| panic!("{list}"))
        .iter()
        .map(|f| (f["feature"].as_str().unwrap().to_string(), f["width"].as_f64().unwrap_or(0.0), f["height"].as_f64().unwrap_or(0.0)))
        .collect()
}

#[test]
fn a_reflected_sheet_has_a_blank_of_the_same_size_and_a_copy_has_none() {
    let mut s = sheet("mirror_sheet");
    let alone = flats(&mut s);
    assert_eq!(alone.len(), 1, "{alone:?}");
    run(&mut s, "add", json!({"name": "copy", "op": "mirror", "inputs": ["wall"], "base": "xz", "turn": "y"}));
    assert_eq!(flats(&mut s).len(), 1, "a copy is no part: no blank");
    run(&mut s, "add", json!({"name": "other", "op": "mirror", "inputs": ["wall"], "base": "yz"}));
    let both = flats(&mut s);
    assert_eq!(both.len(), 2, "{both:?}");
    assert!((both[1].1 - alone[0].1).abs() < 1e-6 && (both[1].2 - alone[0].2).abs() < 1e-6, "{both:?} {alone:?}");
    assert_eq!(run(&mut s, "product", json!({}))["parts"].as_array().unwrap().len(), 2);
}

#[test]
fn a_reflected_sheet_is_sheet_metal_by_the_rule_of_what_it_mirrors() {
    let mut s = sheet("mirror_sheet_rule");
    run(&mut s, "add", json!({"name": "other", "op": "mirror", "inputs": ["wall"], "base": "yz"}));
    let flat = run(&mut s, "flats", json!({}));
    let other = flat["flats"].as_array().unwrap().iter().find(|f| f["feature"] == "other").unwrap();
    assert_eq!(other["k_factor"], 0.4, "{other}");
}

#[test]
fn a_reflected_sheet_with_a_cut_on_its_blank_only_says_so() {
    let mut s = sheet("mirror_sheet_blank");
    let hole = json!({"points": {"h": [40, 20]}, "circles": {"c": {"center": "h", "diameter": 10}}});
    run(&mut s, "add", json!({"name": "on_blank", "op": "sketch", "inputs": ["wall"], "flat": "front", "k_factor": 0.4, "sketch": hole}));
    run(
        &mut s,
        "add",
        json!({"name": "tech", "op": "extrude", "inputs": ["wall", "on_blank"], "mode": "cut", "through": true, "wall": true, "blank_only": true}),
    );
    run(&mut s, "add", json!({"name": "other", "op": "mirror", "inputs": ["tech"], "base": "yz"}));
    let flat = run(&mut s, "flats", json!({}));
    let other = flat["flats"].as_array().unwrap().iter().find(|f| f["feature"] == "other").unwrap();
    assert!(other["error"].as_str().is_some_and(|e| e.contains("on its blank")), "{other}");
}
