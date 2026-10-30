//! Levels of a model (D152, D169): the root a multi-body part, its bodies the parts, variants of a
//! part below it; each what differs from its parent, the parent's changes reaching every level
//! below.
#![allow(clippy::unwrap_used, clippy::panic)]

use datum_model::Session;
use serde_json::{Value, json};

fn run(s: &mut Session, cmd: &str, p: Value) -> Value {
    s.execute(cmd, &p).unwrap_or_else(|e| panic!("{cmd}: {e}"))
}

fn volume(s: &mut Session, name: &str) -> f64 {
    run(s, "mass_properties", json!({"name": name}))["volume"].as_f64().unwrap()
}

/// The root: a plate `w` × 50 drawn in a sketch (its width a dimension), `t` thick; its one part.
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
    run(s, "add", json!({"name": "plate", "op": "extrude", "mode": "new", "inputs": ["outline"], "z0": 0, "z1": "t"}));
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
    run(&mut s, "add_level", json!({"name": "long", "part": "plate"}));
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
    let r = run(&mut s, "activate", json!({"level": "long_holed"}));
    assert!(r.get("shape_changed").is_none(), "another level worked in is no edit: {r}");
    let holed = 36000.0 - 3.0 * 25.0 * std::f64::consts::PI;
    assert!((volume(&mut s, "plate") - holed).abs() < 1e-9, "the hole kept, the frame's height and thickness taken");
    // The level holds only what differs: the hole and its dimensions, not the frame's sketch.
    let levels = run(&mut s, "levels", json!({}));
    assert_eq!(levels["active"], "long_holed");
    assert_eq!(
        levels["levels"],
        json!([
            {"name": "long", "parent": null, "depth": 0, "parameters": {"w": 200.0}, "features": [],
             "changed": [], "own": [], "parameters_set": ["w"], "own_parameters": [], "part": "plate", "output": ["plate"]},
            {"name": "long_holed", "parent": "long", "depth": 1, "parameters": {}, "features": ["outline"],
             "changed": ["outline"], "own": [], "parameters_set": [], "own_parameters": [], "part": "plate", "output": ["plate"]}
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
    run(&mut s, "add_level", json!({"name": "long", "part": "plate"}));
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
    let e = s.execute("remove_level", &json!({"name": "long", "part": "plate"})).unwrap_err();
    assert!(e.contains("below"), "{e}");
}

/// A point renamed in the frame's sketch is renamed in the variant that dimensions from it.
#[test]
fn a_rename_reaches_the_levels_below() {
    let mut s = Session::default();
    frame(&mut s);
    run(&mut s, "add_level", json!({"name": "holed", "part": "plate"}));
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
    run(&mut s, "add_level", json!({"name": "holed", "part": "plate"}));
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
    run(&mut s, "add_level", json!({"name": "plain", "part": "plate"}));
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

/// The product (D160, D169): each body of the root a part, in its chosen variant (none: the
/// root's own); a helper body is no part; the choice, names and visibility are saved.
#[test]
fn the_product_is_each_part_in_its_chosen_variant() {
    let mut s = Session::default();
    run(&mut s, "set_parameter", json!({"name": "w", "value": 100}));
    run(
        &mut s,
        "add",
        json!({"name": "outline", "op": "sketch", "sketch": {
        "points": {"a": [0, 0], "b": [100, 0], "c": [100, 50], "d": [0, 50]},
        "lines": {"bottom": {"from": "a", "to": "b"}, "right": {"from": "b", "to": "c"}, "top": {"from": "c", "to": "d"}, "left": {"from": "d", "to": "a"}},
        "constraints": [{"type": "fix", "point": "a"}, {"type": "horizontal", "line": "bottom"}, {"type": "vertical", "line": "right"},
                        {"type": "horizontal", "line": "top"}, {"type": "vertical", "line": "left"},
                        {"type": "length", "line": "bottom", "value": "w"}, {"type": "length", "line": "left", "value": 50}]}}),
    );
    // Two parts on one sketch, a plate and a lid 10 above it, and a helper body; the plate cut.
    run(&mut s, "add", json!({"name": "slab", "op": "extrude", "mode": "new", "inputs": ["outline"], "z0": 0, "z1": 2}));
    run(&mut s, "add", json!({"name": "cover", "op": "extrude", "mode": "new", "inputs": ["outline"], "z0": 10, "z1": 11}));
    run(&mut s, "add", json!({"name": "helper", "op": "prism", "polygon": [[0, 0], [1, 0], [1, 1]], "z0": 0, "z1": 1}));
    run(&mut s, "add", json!({"name": "pin", "op": "prism", "polygon": [[10, 10], [20, 10], [20, 20], [10, 20]], "z0": -1, "z1": 5}));
    run(&mut s, "add", json!({"name": "drilled", "op": "boolean", "kind": "difference", "inputs": ["slab", "pin"]}));
    let product = |s: &mut Session| -> Vec<(String, Option<String>, Vec<String>)> {
        run(s, "product", json!({}))["parts"]
            .as_array()
            .unwrap()
            .iter()
            .map(|p| {
                let names = p["bodies"].as_array().unwrap().iter().map(|b| b["name"].as_str().unwrap().to_string()).collect();
                (p["part"].as_str().unwrap().into(), p["level"].as_str().map(String::from), names)
            })
            .collect()
    };
    let owned = |v: &[(&str, Option<&str>, &str)]| -> Vec<(String, Option<String>, Vec<String>)> {
        v.iter().map(|(p, l, b)| (p.to_string(), l.map(String::from), vec![b.to_string()])).collect()
    };
    // A part is named by the feature that began its body, its body the last of it.
    assert_eq!(product(&mut s), owned(&[("slab", None, "drilled"), ("cover", None, "cover"), ("helper", None, "helper")]));
    run(&mut s, "set_part", json!({"body": "helper", "part": false}));
    run(&mut s, "set_part", json!({"body": "slab", "name": "Плита"}));
    assert!(s.execute("set_part", &json!({"body": "drilled"})).is_err(), "a part is named by where its body begins");
    // A longer plate: the plate's variant; `w` is shared, but the product takes only the plate from it.
    run(&mut s, "add_level", json!({"name": "long", "part": "slab"}));
    run(&mut s, "set_parameter", json!({"name": "w", "value": 200}));
    assert!(s.execute("add_level", &json!({"name": "loose"})).is_err(), "a variant is a part's");
    run(&mut s, "choose", json!({"level": "long"}));
    assert_eq!(product(&mut s), owned(&[("slab", Some("long"), "drilled"), ("cover", None, "cover")]));
    let r = run(&mut s, "product", json!({}));
    assert_eq!(
        r["parts"][0]["bodies"][0]["volume"].as_f64(),
        Some(200.0 * 50.0 * 2.0 - 100.0 * 6.0 / 3.0 * 3.0 * 0.0 - 100.0 * 2.0),
        "the long plate, drilled"
    );
    assert_eq!(r["parts"][0]["name"], "Плита");
    assert_eq!(r["parts"][1]["bodies"][0]["volume"].as_f64(), Some(100.0 * 50.0), "the lid as the root has it");
    // The lid hidden in the product: listed, but not shown; kept in the file.
    run(&mut s, "show", json!({"part": "cover", "visible": false}));
    assert_eq!(run(&mut s, "product", json!({}))["parts"][1]["visible"], false);
    assert!(s.execute("show", &json!({"part": "long", "visible": false})).is_err(), "a variant is no part");
    // Saved and opened: the choice, names and visibility kept.
    let dir = std::env::temp_dir().join(format!("datum-product-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("product.json");
    run(&mut s, "save", json!({"path": path.display().to_string()}));
    let file: Value = serde_json::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
    assert_eq!(file["parts"], json!({"helper": null, "slab": {"name": "Плита"}}));
    assert_eq!(file["levels"]["long"]["part"], "slab");
    let mut t = Session::default();
    run(&mut t, "open", json!({"path": path.display().to_string()}));
    assert_eq!(product(&mut t), owned(&[("slab", Some("long"), "drilled"), ("cover", None, "cover")]));
    assert_eq!(run(&mut t, "inspect", json!({}))["parts"][1]["visible"], false);
    std::fs::remove_dir_all(&dir).unwrap();
    // A variant removed: its part goes back to the root's own; back by `choose` too.
    run(&mut s, "remove_level", json!({"name": "long"}));
    assert_eq!(product(&mut s)[0].1, None);
}

/// A file from before D169 (parts as levels below a frame) opens brought over: each part level's
/// features in the root, its body a part named as the level; a part level that changed the frame
/// stays as the part's variant; the variants and the choice as they were.
#[test]
fn a_file_with_parts_as_levels_is_brought_over() {
    let mut s = Session::default();
    frame(&mut s);
    let dir = std::env::temp_dir().join(format!("datum-bring-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("old.json");
    run(&mut s, "save", json!({"path": path.display().to_string()}));
    let mut file: Value = serde_json::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
    // The frame: the sketch alone.
    file["features"].as_array_mut().unwrap().retain(|f| f["name"] != "plate");
    file["levels"] = json!({
        "Плита": {"features": [{"name": "plate", "op": "extrude", "inputs": ["outline"], "z0": 0, "z1": "t"}]},
        "Длинная": {"parent": "Плита", "parameters": {"w": 200}},
        "Крышка": {"parameters": {"t": 1}, "features": [{"name": "lid", "op": "extrude", "inputs": ["outline"], "z0": 10, "z1": "10 + t"}],
                   "output": ["lid"]}
    });
    file["chosen"] = json!({"Плита": "Длинная"});
    file["hidden"] = json!(["Крышка"]);
    std::fs::write(&path, file.to_string()).unwrap();
    run(&mut s, "open", json!({"path": path.display().to_string()}));
    let levels = run(&mut s, "levels", json!({}));
    assert_eq!(
        levels["parts"],
        json!([{"part": "lid", "name": "Крышка", "chosen": "Крышка", "visible": false, "rule": null},
               {"part": "plate", "name": "Плита", "chosen": "Длинная", "visible": true, "rule": null}]),
        "the part levels by name, each a body of the root"
    );
    let names: Vec<&str> = levels["levels"].as_array().unwrap().iter().map(|l| l["name"].as_str().unwrap()).collect();
    assert_eq!(names, ["Длинная", "Крышка"], "the plate's level changed nothing but its features: gone");
    assert_eq!(volume(&mut s, "plate"), 10000.0);
    assert_eq!(s.execute("mass_properties", &json!({"name": "plate", "level": "Длинная"})).unwrap()["volume"].as_f64(), Some(20000.0));
    assert_eq!(s.execute("mass_properties", &json!({"name": "lid", "level": "Крышка"})).unwrap()["volume"].as_f64(), Some(5000.0));
    // Two parts with a feature of one name cannot share the root: said, not renamed.
    file["levels"]["Крышка"]["features"][0]["name"] = json!("plate");
    file["levels"]["Крышка"]["output"] = json!(["plate"]);
    std::fs::write(&path, file.to_string()).unwrap();
    let e = s.execute("open", &json!({"path": path.display().to_string()})).unwrap_err();
    assert!(e.contains("named as another part's"), "{e}");
    std::fs::remove_dir_all(&dir).unwrap();
}

/// A body is laid flat as its settings say (D164): the front as drawn, the back mirrored, a
/// K-factor of a formula; a model with no levels is one part; the settings are saved; the DXF
/// of a body has its holes and no bend lines.
#[test]
fn bodies_are_laid_flat_by_their_settings() {
    let model = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/models");
    let mut s = Session::default();
    run(&mut s, "open", json!({"path": model.join("symmetric_plate.json").display().to_string()}));
    let r = run(&mut s, "flats", json!({}));
    let plate = &r["flats"][0];
    assert_eq!((plate["feature"].as_str(), plate["side"].as_str(), plate["bend_lines"].as_u64()), (Some("plate"), Some("front"), Some(0)), "{r}");
    assert_eq!((plate["width"].as_f64(), plate["height"].as_f64()), (Some(80.0), Some(40.0)));
    // A part's variant: the length and the row of holes of the 2.5 m wall.
    run(&mut s, "open", json!({"path": model.join("wall_variants.json").display().to_string()}));
    let r = run(&mut s, "flats", json!({}));
    let all: Vec<&str> = r["flats"].as_array().unwrap().iter().map(|f| f["feature"].as_str().unwrap()).collect();
    assert_eq!(all, ["side", "notched"], "the root: every part");
    run(&mut s, "activate", json!({"level": "2,5 м"}));
    let r = run(&mut s, "flats", json!({}));
    let wall = &r["flats"][0];
    assert_eq!((wall["feature"].as_str(), wall["width"].as_f64(), wall["height"].as_f64()), (Some("notched"), Some(2500.0), Some(300.0)), "{r}");
    assert_eq!(wall["pieces"], json!({"lines": 4, "arcs": 0, "circles": 5}));
    let dir = std::env::temp_dir().join(format!("datum-flats-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let dxf = |s: &mut Session, name: &str| {
        let path = dir.join(name);
        run(s, "export_dxf", json!({"name": "notched", "path": path.display().to_string()}));
        std::fs::read_to_string(path).unwrap()
    };
    let front = dxf(&mut s, "front.dxf");
    assert_eq!((front.matches("\nCIRCLE\n").count(), front.matches("\nLINE\n").count()), (5, 4), "holes and the outline, no bend lines");
    // The back side: as its mirror; settings saved with the model.
    run(&mut s, "set_flat", json!({"feature": "notched", "back": true}));
    let back = dxf(&mut s, "back.dxf");
    assert_ne!(front, back);
    assert_eq!(run(&mut s, "flats", json!({}))["flats"][0]["side"], "back");
    let saved = dir.join("wall.json");
    run(&mut s, "save", json!({"path": saved.display().to_string()}));
    let file: Value = serde_json::from_str(&std::fs::read_to_string(&saved).unwrap()).unwrap();
    assert_eq!(file["flats"], json!({"notched": {"back": true}}));
    let mut t = Session::default();
    run(&mut t, "open", json!({"path": saved.display().to_string()}));
    run(&mut t, "activate", json!({"level": "2,5 м"}));
    assert_eq!(run(&mut t, "flats", json!({}))["flats"][0]["side"], "back");
    // Back to the default: the entry goes. A K-factor of a formula that is no parameter is an error
    // told on the blank, not a failed command.
    run(&mut s, "set_flat", json!({"feature": "notched", "back": false}));
    run(&mut s, "save", json!({"path": saved.display().to_string()}));
    assert!(!std::fs::read_to_string(&saved).unwrap().contains("flats"));
    run(&mut s, "set_flat", json!({"feature": "notched", "k_factor": "kk"}));
    assert!(run(&mut s, "flats", json!({}))["flats"][0]["error"].as_str().unwrap().contains("kk"));
    assert!(s.execute("set_flat", &json!({"feature": "nothing", "back": true})).is_err());
    std::fs::remove_dir_all(&dir).unwrap();
}

/// Sets (D169): the product as it is kept under a name and brought back at once; the set the
/// product is as now is told; a variant removed leaves the sets; sets are saved with the model.
#[test]
fn sets_switch_the_product() {
    let model = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/models/wall_variants.json");
    let mut s = Session::default();
    run(&mut s, "open", json!({"path": model.display().to_string()}));
    let chosen =
        |s: &mut Session| -> Vec<Value> { run(s, "levels", json!({}))["parts"].as_array().unwrap().iter().map(|p| p["chosen"].clone()).collect() };
    run(&mut s, "save_set", json!({"name": "Шкаф 2 м"}));
    assert_eq!(run(&mut s, "levels", json!({}))["set"], "Шкаф 2 м");
    run(&mut s, "choose", json!({"level": "2,5 м"}));
    run(&mut s, "choose", json!({"part": "side"}));
    run(&mut s, "show", json!({"part": "side", "visible": false}));
    run(&mut s, "save_set", json!({"name": "Шкаф 2,5 м"}));
    assert_eq!(chosen(&mut s), [Value::Null, json!("2,5 м")]);
    let r = run(&mut s, "apply_set", json!({"name": "Шкаф 2 м"}));
    assert_eq!((r["set"].as_str(), r["sets"].as_array().map(Vec::len)), (Some("Шкаф 2 м"), Some(2)));
    assert_eq!(chosen(&mut s), [json!("Стенка боковая 2 м"), json!("2 м")]);
    assert_eq!(run(&mut s, "levels", json!({}))["parts"][0]["visible"], true);
    // Changed by hand: no set is as the product now.
    run(&mut s, "choose", json!({"level": "1 м"}));
    assert_eq!(run(&mut s, "levels", json!({}))["set"], Value::Null);
    // A variant removed: gone from the sets too.
    run(&mut s, "remove_level", json!({"name": "2,5 м"}));
    run(&mut s, "apply_set", json!({"name": "Шкаф 2,5 м"}));
    assert_eq!(chosen(&mut s), [Value::Null, Value::Null]);
    assert!(s.execute("apply_set", &json!({"name": "nothing"})).is_err());
    // Saved and opened.
    let dir = std::env::temp_dir().join(format!("datum-sets-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("sets.json");
    run(&mut s, "save", json!({"path": path.display().to_string()}));
    let mut t = Session::default();
    run(&mut t, "open", json!({"path": path.display().to_string()}));
    run(&mut t, "apply_set", json!({"name": "Шкаф 2 м"}));
    assert_eq!(chosen(&mut t), [json!("Стенка боковая 2 м"), json!("2 м")]);
    run(&mut t, "remove_set", json!({"name": "Шкаф 2 м"}));
    assert_eq!(run(&mut t, "levels", json!({}))["sets"], json!(["Шкаф 2,5 м"]));
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn a_level_changes_a_title() {
    let mut s = Session::default();
    frame(&mut s);
    run(&mut s, "set_title", json!({"name": "plate", "title": "Основание"}));
    run(&mut s, "add_level", json!({"name": "long", "part": "plate"}));
    let title = |s: &mut Session, level: Value| run(s, "inspect", json!({"level": level}))["features"][1]["title"].clone();
    assert_eq!(title(&mut s, json!("long")), json!("Основание"), "inherited");
    run(&mut s, "set_title", json!({"name": "plate", "title": "Длинное основание", "level": "long"}));
    assert_eq!(title(&mut s, json!("long")), json!("Длинное основание"));
    assert_eq!(title(&mut s, Value::Null), json!("Основание"));
    run(&mut s, "set_title", json!({"name": "plate", "title": null, "level": "long"}));
    assert_eq!(title(&mut s, json!("long")), Value::Null);
}
