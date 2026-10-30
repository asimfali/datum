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
    // A sketch on the top: (0, 0) over the model's origin, X and Y the model's.
    assert_eq!(top["faces"][0]["sketch_axes"], json!({"origin": [0.0, 0.0, 13.0], "x": [1.0, 0.0, 0.0], "y": [0.0, 1.0, 0.0]}));
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
    run(&mut s, "add", json!({"name": "slab", "op": "extrude", "mode": "new", "inputs": ["base"], "z0": 0, "z1": "t"}));
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
    // A feature removed: the parameters stay (they were lost with it).
    run(&mut s, "remove", json!({"name": "row"}));
    assert_eq!(volume(&mut s, "slab"), 40.0 * 40.0 * 6.0);
    assert_eq!(run(&mut s, "inspect", json!({}))["parameters"]["n"]["value"], json!(3.0));
}

/// The flat blank of the rolled reference sheet goes out as DXF; a sheet body too, a sketch not.
#[test]
fn export_dxf() {
    let mut s = Session::default();
    let model = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/models/rolled_perforation.json");
    run(&mut s, "open", json!({"path": model.display().to_string()}));
    let blank =
        &run(&mut s, "inspect", json!({}))["features"].as_array().unwrap().iter().find(|f| f["name"] == "blank").unwrap()["status"]["flat"].clone();
    assert_eq!((blank["lines"].as_u64(), blank["arcs"].as_u64(), blank["circles"].as_u64()), (Some(28), Some(24), Some(100)));
    let path = std::env::temp_dir().join(format!("datum-blank-{}.dxf", std::process::id()));
    let r = run(&mut s, "export_dxf", json!({"name": "blank", "path": path.display().to_string()}));
    assert_eq!(r["pieces"], 152);
    let dxf = std::fs::read_to_string(&path).unwrap();
    std::fs::remove_file(&path).unwrap();
    assert_eq!(dxf.matches("\nCIRCLE\n").count(), 100);
    // A sheet body goes out too, laid flat on its own (D164): the uncut sheet's four lines; a sketch is no body.
    let r = run(&mut s, "export_dxf", json!({"name": "sheet", "path": path.display().to_string()}));
    assert_eq!(r["pieces"], 4);
    std::fs::remove_file(&path).unwrap();
    assert!(s.execute("export_dxf", &json!({"name": "profile", "path": path.display().to_string()})).unwrap_err().contains("no body"));
}

/// The middle of an outline replaced by another section (a library element in place of a piece
/// of the chain): what stands on the faces of the sections kept, and on the caps, stays; what
/// stood on a removed section's face is reported lost, not moved onto a new one.
#[test]
fn a_replaced_section_loses_only_its_own_faces() {
    let mut s = Session::default();
    run(
        &mut s,
        "add",
        json!({"name": "profile", "op": "sketch", "sketch": {
            "points": {"p1": [0, 0], "p2": [20, 60], "p3": [60, 80], "p4": [120, 50], "p5": [130, 0]},
            "lines": {"l1": {"from": "p1", "to": "p2"}, "l2": {"from": "p2", "to": "p3"}, "l3": {"from": "p3", "to": "p4"},
                      "l4": {"from": "p4", "to": "p5"}, "base": {"from": "p5", "to": "p1"}},
            "constraints": [{"type": "fix", "point": "p1"}, {"type": "length", "line": "l1", "value": 60}]
        }}),
    );
    run(&mut s, "add", json!({"name": "body", "op": "extrude", "mode": "new", "inputs": ["profile"], "z0": 0, "z1": 10}));
    let face = |s: &mut Session, normal: [f64; 3]| {
        let r = run(s, "query_faces", json!({"name": "body", "normal": normal}));
        assert_eq!(r["count"].as_u64(), Some(1), "{r}");
        r["faces"][0]["id"].clone()
    };
    for (name, normal) in [("on_l1", [-3.0, 1.0, 0.0]), ("on_l3", [1.0, 2.0, 0.0]), ("on_top", [0.0, 0.0, 1.0])] {
        let id = face(&mut s, normal);
        run(&mut s, "add", json!({"name": name, "op": "sketch", "inputs": ["body"], "face": id, "sketch": {"points": {"q": [0, 0]}}}));
    }
    let r = run(
        &mut s,
        "edit_sketch",
        json!({"name": "profile",
        "points": {"p3": null, "p6": [50, 20], "p7": [80, 70]},
        "lines": {"l5": {"from": "p2", "to": "p6"}, "l6": {"from": "p6", "to": "p7"}, "l7": {"from": "p7", "to": "p4"}}}),
    );
    let failed = |r: &Value| -> Vec<String> {
        assert!(r["failed"].as_array().unwrap().iter().all(|f| f["error"].as_str().unwrap().contains("lost reference")), "{r}");
        r["failed"].as_array().unwrap().iter().map(|f| f["feature"].as_str().unwrap().to_string()).collect()
    };
    assert_eq!(failed(&r), ["on_l3"]);
    let kept = &run(&mut s, "inspect", json!({}))["features"][0]["sketch"];
    assert!(kept["constraints"].as_array().unwrap().contains(&json!({"type": "length", "line": "l1", "value": 60.0})), "{kept}");
    // The corner the outline starts at replaced too: the caps are the feature's, not that point's.
    let r = run(
        &mut s,
        "edit_sketch",
        json!({"name": "profile",
        "points": {"p1": null, "p8": [0, 10], "p9": [10, 0]},
        "lines": {"l8": {"from": "p9", "to": "p8"}, "l9": {"from": "p8", "to": "p2"}, "l10": {"from": "p5", "to": "p9"}},
        "add_constraints": [{"type": "fix", "point": "p9"}]}),
    );
    assert_eq!(failed(&r), ["on_l1", "on_l3"]);
}

/// The user's chain with its middle cut out: a section from the library put between the two
/// ends left, turned toward the second, coincident with both; one of its lines keeps the name
/// of the line it replaces.
#[test]
fn an_element_between_two_points_of_a_chain() {
    let mut s = Session::default();
    run(
        &mut s,
        "add",
        json!({"name": "path", "op": "sketch", "sketch": {
            "points": {"p1": [0, 0], "p2": [20, 60], "p4": [120, 50], "p5": [130, 0]},
            "lines": {"l1": {"from": "p1", "to": "p2"}, "l4": {"from": "p4", "to": "p5"}},
            "constraints": [{"type": "fix", "point": "p1"}, {"type": "fix", "point": "p2"}, {"type": "fix", "point": "p4"}, {"type": "fix", "point": "p5"}]
        }}),
    );
    // A notch: in, across, out; the width across a parameter.
    let notch = json!({"name": "Notch", "anchors": ["s", "e"], "fit": "align",
        "parameters": [{"name": "width", "value": 15}],
        "sketch": {"points": {"s": [0, 0], "a": [3, -1], "b": [7, -1], "e": [10, 0]},
                   "lines": {"in": {"from": "s", "to": "a"}, "bottom": {"from": "a", "to": "b"}, "out": {"from": "b", "to": "e"}},
                   "constraints": [{"type": "equal", "lines": ["in", "out"]}, {"type": "distance", "points": ["a", "b"], "value": "width"}]}});
    let r = run(&mut s, "insert_element", json!({"name": "path", "element": notch, "at": ["p2", "p4"], "names": {"bottom": "l3"}}));
    assert!(r["failed"].as_array().unwrap().is_empty(), "{r}");
    assert_eq!(r["names"]["bottom"], "l3", "{r}");
    let sketch = &run(&mut s, "inspect", json!({}))["features"][0]["sketch"];
    let c = sketch["constraints"].as_array().unwrap();
    let start = r["names"]["s"].as_str().unwrap();
    assert!(c.contains(&json!({"type": "coincident", "points": [start, "p2"]})), "{sketch}");
    assert!(c.contains(&json!({"type": "distance", "points": [r["names"]["a"], r["names"]["b"]], "value": 15.0})), "{sketch}");
    assert!(sketch["points"].get("s").is_none(), "new names, not the element's");
    assert!(sketch.get("groups").is_none_or(|g| g.as_object().unwrap().is_empty()), "no preset, no group: {sketch}");
    let notch = json!({"name": "Notch", "parameters": [{"name": "width", "value": 15}], "presets": {"Deep": {"width": 20}}, "anchors": ["s"], "sketch": {"points": {"s": [0, 0], "t": [1, 0]}, "lines": {"in": {"from": "s", "to": "t"}}}});
    let r = run(&mut s, "insert_element", json!({"name": "path", "element": notch, "preset": "Deep", "at": ["p2"]}));
    let sketch = &run(&mut s, "inspect", json!({}))["features"][0]["sketch"];
    assert_eq!(sketch["groups"]["Deep"], json!([r["names"]["s"], r["names"]["t"], r["names"]["in"]]), "{sketch}");
    // Unknown elements and spots are refused, the model left as it was.
    assert!(s.execute("insert_element", &json!({"name": "path", "element": "Nothing", "at": ["p2"]})).unwrap_err().contains("no element"));
    assert!(s.execute("insert_element", &json!({"name": "path", "element": "Rectangle", "at": ["q"]})).unwrap_err().contains("no point `q`"));
}

/// A project's library comes over the built-in one: an element of the same name replaces it.
#[test]
fn the_project_library_comes_first() {
    let root = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join("library_project");
    let _ = std::fs::remove_dir_all(&root);
    std::fs::create_dir_all(root.join("library/sketch")).unwrap();
    std::fs::create_dir_all(root.join("part")).unwrap();
    std::fs::write(root.join("datum-project.json"), "{}").unwrap();
    let own = json!({"name": "Rectangle", "description": "ours", "anchors": ["a"], "sketch": {"points": {"a": [0, 0]}}});
    std::fs::write(root.join("library/sketch/rect.json"), own.to_string()).unwrap();
    std::fs::write(root.join("library/sketch/broken.json"), "{\"name\": \"x\"}").unwrap();
    // Two of a name in one folder: both kept, not one over the other.
    let twin = json!({"name": "New element", "anchors": ["a"], "sketch": {"points": {"a": [0, 0]}}});
    std::fs::write(root.join("library/sketch/test.json"), twin.to_string()).unwrap();
    std::fs::write(root.join("library/sketch/test2.json"), twin.to_string()).unwrap();
    let mut s = Session::default();
    let lib = run(&mut s, "library", json!({}));
    assert_eq!(lib["elements"][0]["source"], "built_in", "no project without a model file: {lib}");
    run(&mut s, "save", json!({"path": root.join("part/plate.json")}));
    let lib = run(&mut s, "library", json!({}));
    assert_eq!((&lib["elements"][0]["source"], &lib["elements"][0]["description"]), (&json!("project"), &json!("ours")), "{lib}");
    assert!(lib["skipped"][0].as_str().unwrap().contains("broken.json"), "{lib}");
    assert_eq!(lib["elements"].as_array().unwrap().iter().filter(|e| e["name"] == "New element").count(), 2, "{lib}");
    let model = root.join("part/plate.json");
    assert_eq!(datum_model::workspace::root(Some(&model)), Some(root.clone()), "the project holds part, assembly, drawing, library");
    assert_eq!(datum_model::library::folder("sketch", Some(&model)), Ok(root.join("library/sketch")));
    // The project names its library otherwise (D141): read from there, the old folder said.
    std::fs::write(root.join("datum-project.json"), r#"{"folders": {"library": "Библиотека"}}"#).unwrap();
    std::fs::create_dir_all(root.join("Библиотека/sketch")).unwrap();
    let mut renamed = own.clone();
    renamed["description"] = json!("renamed");
    std::fs::write(root.join("Библиотека/sketch/rect.json"), renamed.to_string()).unwrap();
    let lib = run(&mut s, "library", json!({}));
    assert_eq!(lib["elements"][0]["description"], "renamed", "{lib}");
    assert!(lib["skipped"].as_array().unwrap().iter().any(|w| w.as_str().unwrap().contains("the project keeps its library in")), "{lib}");
    assert_eq!(datum_model::library::folder("sketch", Some(&model)), Ok(root.join("Библиотека/sketch")));
}

/// Materials are the enterprise's own (D178): none built in; written, renamed and removed as
/// files of the project's library, as the table editor and an agent do it.
#[test]
fn materials_are_written_renamed_and_removed() {
    let root = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join("library_material");
    let _ = std::fs::remove_dir_all(&root);
    std::fs::create_dir_all(root.join("part")).unwrap();
    std::fs::write(root.join("datum-project.json"), "{}").unwrap();
    let mut s = Session::default();
    run(&mut s, "save", json!({"path": root.join("part/plate.json")}));
    let names = |s: &mut Session| -> Vec<Value> {
        run(s, "library", json!({"level": "material"}))["elements"].as_array().unwrap().iter().map(|e| e["name"].clone()).collect()
    };
    assert!(names(&mut s).is_empty(), "none built in");
    let steel = json!({"name": "08пс", "designation": "Лист [Б-ПН-{t} ГОСТ 19903-2015 | 08пс ГОСТ 16523-97]", "density": 7850});
    let r = run(&mut s, "put_library", json!({"level": "material", "element": steel}));
    assert_eq!(std::path::PathBuf::from(r["saved"].as_str().unwrap()), root.join("library/material/08пс.json"));
    let lib = run(&mut s, "library", json!({"level": "material"}));
    assert_eq!((&lib["elements"][0]["source"], &lib["elements"][0]["density"]), (&json!("project"), &json!(7850)), "{lib}");
    let put = |s: &mut Session, element: Value, was: &str| s.execute("put_library", &json!({"level": "material", "element": element, "was": was}));
    assert!(put(&mut s, json!({"name": "x", "density": 0}), "x").unwrap_err().contains("density"));
    assert!(put(&mut s, json!({"name": "x", "grade": "08пс"}), "x").unwrap_err().contains("has no `grade`"));
    put(&mut s, json!({"name": "Ст3"}), "Ст3").unwrap();
    assert!(put(&mut s, json!({"name": "Ст3"}), "08пс").unwrap_err().contains("already"), "a rename onto another");
    let mut renamed = steel.clone();
    renamed["name"] = json!("08пс лист");
    put(&mut s, renamed, "08пс").unwrap();
    assert_eq!(names(&mut s), [json!("08пс лист"), json!("Ст3")], "the old file gone");
    assert!(!root.join("library/material/08пс.json").exists());
    run(&mut s, "remove_library", json!({"level": "material", "name": "Ст3"}));
    assert_eq!(names(&mut s), [json!("08пс лист")]);
    assert!(s.execute("remove_library", &json!({"level": "sketch", "name": "Rectangle"})).unwrap_err().contains("built in"));
}

/// A rule is of a material (D179): one of no material in the library is refused; a material
/// renamed takes its rules along, one with rules is not removed; a sheet rule's bend radius may
/// be a formula of the thickness, a part's rule has neither.
#[test]
fn rules_are_of_a_material() {
    let root = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join("library_rule");
    let _ = std::fs::remove_dir_all(&root);
    std::fs::create_dir_all(root.join("part")).unwrap();
    std::fs::write(root.join("datum-project.json"), "{}").unwrap();
    let mut s = Session::default();
    run(&mut s, "save", json!({"path": root.join("part/plate.json")}));
    let put = |s: &mut Session, level: &str, element: Value| s.execute("put_library", &json!({"level": level, "element": element}));
    let sheet = json!({"name": "08пс 2", "kind": "sheet", "material": "08пс", "thickness": 2, "radius": "1.5 * t"});
    assert!(put(&mut s, "rule", sheet.clone()).unwrap_err().contains("no material `08пс`"));
    put(&mut s, "material", json!({"name": "08пс"})).unwrap();
    put(&mut s, "rule", sheet).unwrap();
    put(&mut s, "rule", json!({"name": "08пс", "kind": "part", "material": "08пс"})).unwrap();
    for (rule, why) in [
        (json!({"name": "x", "kind": "sheet"}), "needs the `material`"),
        (json!({"name": "x", "kind": "plate", "material": "08пс"}), "`kind`"),
        (json!({"name": "x", "kind": "part", "material": "08пс", "thickness": 2}), "has no `thickness`"),
        (json!({"name": "x", "kind": "sheet", "material": "08пс", "radius": "2 * s"}), "`radius`"),
        (json!({"name": "x", "kind": "sheet", "material": "08пс", "thickness": 0}), "`thickness`"),
    ] {
        assert!(put(&mut s, "rule", rule.clone()).unwrap_err().contains(why), "{rule}");
    }
    let r = s.execute("remove_library", &json!({"level": "material", "name": "08пс"})).unwrap_err();
    assert!(r.contains("`08пс 2`") && r.contains("`08пс`"), "{r}");
    run(&mut s, "put_library", json!({"level": "material", "element": {"name": "08пс ГОСТ"}, "was": "08пс"}));
    let rules = run(&mut s, "library", json!({"level": "rule"}));
    let of: Vec<&Value> = rules["elements"].as_array().unwrap().iter().map(|r| &r["material"]).collect();
    assert_eq!(of, [&json!("08пс ГОСТ"), &json!("08пс ГОСТ")], "{rules}");
    assert_eq!(datum_model::library::bend_radius(&json!("1.5 * t"), 2.0), Ok(3.0));
}

/// A piece of sketch kept in the project's library and placed again: the model's parameters its
/// dimensions name become the element's, in the order asked; placed, it is sized by the answers.
#[test]
fn a_selection_kept_in_the_library_and_placed_again() {
    let root = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join("library_keep");
    let _ = std::fs::remove_dir_all(&root);
    std::fs::create_dir_all(root.join("part")).unwrap();
    std::fs::write(root.join("datum-project.json"), "{}").unwrap();
    let mut s = Session::default();
    run(&mut s, "set_parameter", json!({"name": "w", "value": 80}));
    run(&mut s, "set_parameter", json!({"name": "h", "value": 30}));
    run(
        &mut s,
        "add",
        json!({"name": "outline", "op": "sketch", "sketch": {
            "points": {"a": [0, 0], "b": [80, 0], "c": [80, 30], "d": [0, 30]},
            "lines": {"bottom": {"from": "a", "to": "b"}, "right": {"from": "b", "to": "c"}, "top": {"from": "c", "to": "d"}, "left": {"from": "d", "to": "a"}},
            "constraints": [{"type": "fix", "point": "origin"}, {"type": "coincident", "points": ["a", "origin"]},
                            {"type": "horizontal", "line": "bottom"}, {"type": "vertical", "line": "right"}, {"type": "horizontal", "line": "top"}, {"type": "vertical", "line": "left"},
                            {"type": "length", "line": "bottom", "value": "w"}, {"type": "length", "line": "left", "value": "h / 2 + 15"}]
        }}),
    );
    run(&mut s, "save", json!({"path": root.join("part/plate.json")}));
    let r = run(
        &mut s,
        "to_library",
        json!({"name": "outline", "select": ["bottom", "right", "top", "left"], "element": "Plate: w × h",
        "description": "a plate", "order": ["h"], "anchors": ["a", "c"]}),
    );
    assert_eq!((&r["element"]["anchors"], &r["element"]["fit"]), (&json!(["a", "c"]), &Value::Null), "the engineer's points: {r}");
    let saved = std::path::PathBuf::from(r["saved"].as_str().unwrap());
    assert_eq!(saved, root.join("library/sketch/Plate_ w × h.json"), "in the project's library, a name a file system takes");
    let e = &r["element"];
    assert_eq!(e["parameters"], json!([{"name": "h", "value": 30.0}, {"name": "w", "value": 80.0}]), "asked in the order given: {e}");
    let c = e["sketch"]["constraints"].as_array().unwrap();
    assert!(!c.iter().any(|c| c.to_string().contains("origin")), "what ties to the origin stays behind: {e}");
    assert!(s.execute("to_library", &json!({"name": "outline", "select": ["bottom"], "element": "Plate: w × h"})).unwrap_err().contains("exists"));
    let ask = |dim: &str| json!({"name": "outline", "select": ["bottom", "left"], "element": "Corner", "ask": {dim: "x"}});
    assert!(s.execute("to_library", &ask("length bottom")).unwrap_err().contains("is not a number"), "given by `w` already");
    assert!(s.execute("to_library", &ask("vertical right")).unwrap_err().contains("no dimension"), "a constraint without a value");
    let refused = s.execute("to_library", &json!({"name": "outline", "select": ["bottom"], "element": "x", "anchors": ["c"]})).unwrap_err();
    assert!(refused.contains("anchor `c` is not a point"), "an insertion point outside the piece: {refused}");
    let lib = run(&mut s, "library", json!({}));
    assert!(lib["elements"].as_array().unwrap().iter().any(|e| e["name"] == "Plate: w × h" && e["source"] == "project"), "{lib}");
    run(&mut s, "add", json!({"name": "other", "op": "sketch", "sketch": {"points": {}}}));
    let r = run(&mut s, "insert_element", json!({"name": "other", "element": "Plate: w × h", "at": [[100, 0]], "values": {"h": 10}}));
    assert!(r["failed"].as_array().unwrap().is_empty(), "{r}");
    let other = &run(&mut s, "inspect", json!({}))["features"][1]["sketch"];
    let lengths: Vec<&Value> = other["constraints"].as_array().unwrap().iter().filter(|c| c["type"] == "length").map(|c| &c["value"]).collect();
    assert_eq!(lengths, [&json!(80.0), &json!(20.0)], "w as kept, h answered 10: {other}");
}

/// A constraint reaching out of the piece kept (l5 ⊥ l4, l4 left behind) becomes a socket: put
/// again, it is completed by a line of the sketch, or left open and said so.
#[test]
fn a_constraint_reaching_out_is_a_socket() {
    let root = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join("library_links");
    let _ = std::fs::remove_dir_all(&root);
    let mut s = Session::default();
    run(
        &mut s,
        "add",
        json!({"name": "path", "op": "sketch", "sketch": {
            "points": {"p4": [0, 0], "p5": [10, 2], "p6": [11, -3], "p7": [16, -2]},
            "lines": {"l4": {"from": "p4", "to": "p5"}, "l5": {"from": "p5", "to": "p6"}, "l6": {"from": "p6", "to": "p7"}},
            "constraints": [{"type": "perpendicular", "lines": ["l4", "l5"]}, {"type": "perpendicular", "lines": ["l5", "l6"]},
                            {"type": "length", "line": "l5", "value": 5}]
        }}),
    );
    let r = run(
        &mut s,
        "to_library",
        json!({"name": "path", "select": ["l5", "l6"], "element": "Step", "anchors": ["p5"], "path": root.join("step.json")}),
    );
    assert_eq!(r["element"]["links"], json!([{"constraint": {"type": "perpendicular", "lines": ["?", "l5"]}, "kind": "line"}]), "{r}");
    let step = r["element"].clone();
    // Put at the end of l6, its socket given l6: the new l5 is perpendicular to it.
    let r = run(&mut s, "insert_element", json!({"name": "path", "element": step, "at": ["p7"], "links": {"perpendicular ? l5": "l6"}}));
    assert!(r["failed"].as_array().unwrap().is_empty() && r["open_links"] == json!([]), "{r}");
    let new_l5 = r["names"]["l5"].as_str().unwrap();
    let c = &run(&mut s, "inspect", json!({}))["features"][0]["sketch"]["constraints"];
    assert!(c.as_array().unwrap().contains(&json!({"type": "perpendicular", "lines": ["l6", new_l5]})), "{c}");
    // Not given: open, and said so; given a point: refused.
    let r = run(&mut s, "insert_element", json!({"name": "path", "element": step, "at": [[40, 0]]}));
    assert_eq!(r["open_links"].as_array().unwrap().len(), 1, "{r}");
    let refused =
        s.execute("insert_element", &json!({"name": "path", "element": step, "at": [[60, 0]], "links": {"perpendicular ? l5": "p4"}})).unwrap_err();
    assert!(refused.contains("takes a line"), "{refused}");
}

/// A line and its dimension picked together and deleted: the dimension is removed by name, and
/// the line's removal does not look for it again (a frame picks both).
#[test]
fn a_line_deleted_with_its_dimension() {
    let mut s = Session::default();
    run(
        &mut s,
        "add",
        json!({"name": "s", "op": "sketch", "sketch": {
            "points": {"p9": [0, 0], "p10": [5, 0]}, "lines": {"l8": {"from": "p9", "to": "p10"}},
            "constraints": [{"type": "horizontal", "line": "l8"}, {"type": "length", "line": "l8", "value": 5.0}]
        }}),
    );
    let r =
        run(&mut s, "edit_sketch", json!({"name": "s", "lines": {"l8": null}, "remove_constraints": [{"type": "length", "line": "l8", "value": 5}]}));
    assert_eq!(r["also_removed"], json!(["constraints: 1"]), "the horizontal went with the line: {r}");
    assert!(
        s.execute("edit_sketch", &json!({"name": "s", "remove_constraints": [{"type": "length", "line": "l8", "value": 5}]}))
            .unwrap_err()
            .contains("no constraint")
    );
}

/// `inspect` says whether the model differs from its file: after an edit it does, after a save
/// or an undo back to what was saved it does not.
#[test]
fn modified_since_saved() {
    let path = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join("modified.json");
    let mut s = Session::default();
    let modified = |s: &mut Session| run(s, "inspect", json!({}))["modified"].as_bool().unwrap();
    assert!(!modified(&mut s), "an empty new model");
    plate(&mut s);
    assert!(modified(&mut s));
    run(&mut s, "save", json!({"path": path}));
    assert!(!modified(&mut s));
    run(&mut s, "edit", json!({"name": "moved", "angle_deg": 30}));
    assert!(modified(&mut s));
    run(&mut s, "undo", json!({}));
    assert!(!modified(&mut s), "back to what was saved");
    run(&mut s, "open", json!({"path": path}));
    assert!(!modified(&mut s));
}
