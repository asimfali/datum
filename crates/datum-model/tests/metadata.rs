//! Properties of a sketch's own primitives (D275, D276): by element name, any JSON value,
//! read and written through the common commands; they follow a rename, go with a removed curve,
//! and change no geometry, hash or computation.
#![allow(clippy::unwrap_used, clippy::panic)]

use datum_model::Session;
use serde_json::{Value, json};

fn run(s: &mut Session, cmd: &str, p: Value) -> Value {
    s.execute(cmd, &p).unwrap_or_else(|e| panic!("{cmd}: {e}"))
}

/// The feature `name` as `inspect` lists it.
fn feature(s: &mut Session, name: &str) -> Value {
    run(s, "inspect", json!({}))["features"].as_array().unwrap().iter().find(|f| f["name"] == name).unwrap_or_else(|| panic!("no {name}")).clone()
}

/// A rectangle 10 × 10 with a construction arc, and a plate extruded from it: the sketch has
/// lines and an arc to mark, and a consumer whose computation is watched.
fn framed() -> Session {
    let mut s = Session::default();
    run(
        &mut s,
        "add",
        json!({"name": "outline", "op": "sketch", "sketch": {
            "points": {"a": [0, 0], "b": [10, 0], "c": [10, 10], "d": [0, 10], "o": [5, 0]},
            "lines": {"bottom": {"from": "a", "to": "b"}, "right": {"from": "b", "to": "c"},
                      "top": {"from": "c", "to": "d"}, "left": {"from": "d", "to": "a"}},
            "arcs": {"fillet": {"center": "o", "from": "b", "to": "a", "construction": true}},
            "constraints": [{"type": "fix", "point": "a"}, {"type": "horizontal", "line": "bottom"},
                            {"type": "vertical", "line": "right"}, {"type": "horizontal", "line": "top"},
                            {"type": "vertical", "line": "left"}]}}),
    );
    run(&mut s, "add", json!({"name": "plate", "op": "extrude", "mode": "new", "inputs": ["outline"], "z0": 0, "z1": 2}));
    s
}

#[test]
fn circle_transition_and_point_marks_survive_model_edits() {
    let mut s = framed();
    run(
        &mut s,
        "edit_sketch",
        json!({"name": "outline", "circles": {
        "ring": {"center": "o", "diameter": 2, "construction": true}
    }, "metadata": {"o": {"note": "center"}}}),
    );
    let result = run(
        &mut s,
        "edit_metadata",
        json!({
            "name": "outline", "elements": ["ring"], "set": {"stamp.role": "transition"}
        }),
    );
    assert_eq!(result["computed"], 0);
    run(&mut s, "rename_in_sketch", json!({"name": "outline", "from": "ring", "to": "transition"}));
    assert_eq!(properties(&mut s, "outline", &["transition", "o"]), json!([{"stamp.role": "transition"}, {"note": "center"}]));
    run(&mut s, "edit_sketch", json!({"name": "outline", "points": {"o": null}}));
    assert_eq!(run(&mut s, "query_metadata", json!({"name": "outline"}))["elements"], json!([]));
}

/// The properties `elements` of sketch `name` say now.
fn properties(s: &mut Session, name: &str, elements: &[&str]) -> Value {
    run(s, "query_metadata", json!({"name": name, "elements": elements}))["elements"]
        .as_array()
        .unwrap()
        .iter()
        .map(|e| e["properties"].clone())
        .collect()
}

/// Properties of lines and of an arc, any JSON value (a null and a `constraints` key too), kept
/// as written; no geometry is computed again and no hash changes.
#[test]
fn marks_are_stored_apart_from_geometry() {
    let mut s = framed();
    let (sketch_hash, plate_hash) = (feature(&mut s, "outline")["status"]["hash"].clone(), feature(&mut s, "plate")["status"]["hash"].clone());
    let r = run(
        &mut s,
        "edit_metadata",
        json!({"name": "outline", "elements": ["bottom", "fillet"],
               "set": {"stamp.role": "cut", "note": {"goes": null, "constraints": [1, 2]}, "who": "bottom"}}),
    );
    assert_eq!(r["computed"], json!(0), "nothing is computed again: {r}");
    let marked = json!({"stamp.role": "cut", "note": {"goes": null, "constraints": [1, 2]}, "who": "bottom"});
    assert_eq!(properties(&mut s, "outline", &["bottom", "fillet"]), json!([marked, marked]));
    assert_eq!(feature(&mut s, "outline")["status"]["hash"], sketch_hash, "the sketch geometry is the same");
    assert_eq!(feature(&mut s, "plate")["status"]["hash"], plate_hash, "the consumer is not computed again");
    // Without `elements`: the marked curves, by name; a curve with no properties is no error.
    let all = run(&mut s, "query_metadata", json!({"name": "outline"}));
    let names: Vec<&str> = all["elements"].as_array().unwrap().iter().map(|e| e["element"].as_str().unwrap()).collect();
    assert_eq!(names, ["bottom", "fillet"]);
    assert_eq!(properties(&mut s, "outline", &["top"]), json!([{}]));
    // A missing property is no error; an unknown target or an empty name writes nothing.
    let before = feature(&mut s, "outline");
    run(&mut s, "edit_metadata", json!({"name": "outline", "elements": ["top"], "remove": ["nothing"]}));
    for params in [
        json!({"name": "outline", "elements": ["bottom", "nothing"], "set": {"x": 1}}),
        json!({"name": "outline", "elements": ["bottom"], "set": {"": 1}}),
        json!({"name": "outline", "elements": [], "set": {"x": 1}}),
    ] {
        assert!(s.execute("edit_metadata", &params).is_err(), "{params}");
    }
    assert_eq!(feature(&mut s, "outline"), before, "nothing was written");
    // One change over two curves is one Undo.
    run(&mut s, "edit_metadata", json!({"name": "outline", "elements": ["bottom", "fillet"], "set": {"made": true}}));
    run(&mut s, "undo", json!({}));
    assert_eq!(properties(&mut s, "outline", &["bottom", "fillet"]), json!([marked, marked]), "the mark before the change");
    run(&mut s, "redo", json!({}));
    assert_eq!(properties(&mut s, "outline", &["bottom"])[0]["made"], true);
}

/// A rename takes the properties' keys along and leaves their values alone, even a value that
/// spells a curve's name; a removed curve takes them, directly or through its point, and its name
/// given again later is new and unmarked.
#[test]
fn marks_follow_a_rename_and_go_with_a_curve() {
    let mut s = framed();
    run(&mut s, "edit_metadata", json!({"name": "outline", "elements": ["bottom", "fillet"], "set": {"note": "bottom"}}));
    run(&mut s, "rename_in_sketch", json!({"name": "outline", "from": "bottom", "to": "base"}));
    assert_eq!(properties(&mut s, "outline", &["base"]), json!([{"note": "bottom"}]), "the value is not a name");
    assert!(s.execute("query_metadata", &json!({"name": "outline", "elements": ["bottom"]})).is_err(), "the old name is gone");
    // Removed directly: its entry goes, and the same name given again comes back unmarked.
    run(&mut s, "edit_sketch", json!({"name": "outline", "lines": {"base": null}}));
    run(&mut s, "edit_sketch", json!({"name": "outline", "lines": {"base": {"from": "a", "to": "b"}}}));
    assert_eq!(properties(&mut s, "outline", &["base"]), json!([{}]));
    // Removed through a point: every curve on it goes with its properties.
    run(&mut s, "edit_metadata", json!({"name": "outline", "elements": ["right", "fillet"], "set": {"x": 1}}));
    run(&mut s, "edit_sketch", json!({"name": "outline", "points": {"b": null}}));
    assert!(s.execute("query_metadata", &json!({"name": "outline", "elements": ["right"]})).is_err(), "the curve is gone");
    let all = run(&mut s, "query_metadata", json!({"name": "outline"}));
    let names: Vec<&str> = all["elements"].as_array().unwrap().iter().map(|e| e["element"].as_str().unwrap()).collect();
    assert!(names.is_empty(), "base, right and fillet went with `b`: {names:?}");
}

/// Saved with the model and read back; a file with no marks is written as before, without them.
#[test]
fn marks_are_saved_and_opened() {
    let dir = std::env::temp_dir().join(format!("datum-metadata-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("marks.json");
    let mut s = framed();
    run(&mut s, "save", json!({"path": path.display().to_string()}));
    let plain = std::fs::read_to_string(&path).unwrap();
    assert!(!plain.contains("\"metadata\""), "a sketch with no marks writes none:\n{plain}");
    run(
        &mut s,
        "edit_metadata",
        json!({"name": "outline", "elements": ["bottom"], "set": {"stamp.role": "cut", "goes": null, "nested": {"a": [1]}}}),
    );
    run(&mut s, "save", json!({"path": path.display().to_string()}));
    let text = std::fs::read_to_string(&path).unwrap();
    let file: Value = serde_json::from_str(&text).unwrap();
    assert_eq!(
        file["features"][0]["sketch"]["metadata"]["bottom"],
        json!({"stamp.role": "cut", "goes": null, "nested": {"a": [1]}}),
        "the values as written:\n{text}"
    );
    let mut t = Session::default();
    run(&mut t, "open", json!({"path": path.display().to_string()}));
    assert_eq!(properties(&mut t, "outline", &["bottom"]), json!([{"stamp.role": "cut", "goes": null, "nested": {"a": [1]}}]));
    // Taken away whole, the section is gone from the file.
    run(&mut s, "edit_metadata", json!({"name": "outline", "elements": ["bottom"], "remove": ["stamp.role", "goes", "nested"]}));
    run(&mut s, "save", json!({"path": path.display().to_string()}));
    assert!(!std::fs::read_to_string(&path).unwrap().contains("\"metadata\""), "empty maps are not written");
    std::fs::remove_dir_all(&dir).unwrap();
}

/// The presets the command lists follow the project of the document open (D277): the shipped two
/// stand, the project's same ID replaces one whole, and a preset is only a way to fill `set`.
#[test]
fn presets_follow_the_open_documents_project() {
    let dir = std::env::temp_dir().join(format!("datum-metadata-presets-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join("datum-project.json"), r#"{"metadata_presets": {"cut": {"label": "Резка", "set": {"stamp.role": "cut", "shop": 7}}}}"#)
        .unwrap();
    let path = dir.join("model.json");
    let mut s = framed();
    run(&mut s, "save", json!({"path": path.display().to_string()}));
    let listed = run(&mut s, "list_metadata_presets", json!({}));
    let presets = listed["presets"].as_array().unwrap();
    let cut = presets.iter().find(|p| p["id"] == "cut").unwrap();
    assert_eq!((&cut["set"], &cut["source"], &cut["label"]), (&json!({"stamp.role": "cut", "shop": 7}), &json!("project"), &json!("Резка")));
    assert!(listed["diagnostics"].is_array());
    // Choosing it is an ordinary `edit_metadata.set`: the model keeps the values, not the preset.
    run(&mut s, "edit_metadata", json!({"name": "outline", "elements": ["bottom"], "set": cut["set"]}));
    assert_eq!(properties(&mut s, "outline", &["bottom"]), json!([{"stamp.role": "cut", "shop": 7}]));
    std::fs::remove_dir_all(&dir).unwrap();
}

/// A sketch read from JSON is normalized (D276): properties of a curve it does not have, maps of
/// no properties and a name given again later are dropped; an empty property name is refused.
#[test]
fn direct_sketch_json_is_normalized() {
    let mut s = Session::default();
    let sketch = json!({"points": {"a": [0, 0], "b": [10, 0]},
        "lines": {"bottom": {"from": "a", "to": "b"}},
        "metadata": {"bottom": {"stamp.role": "cut"}, "empty": {}, "ghost": {"note": "old"}}});
    run(&mut s, "add", json!({"name": "outline", "op": "sketch", "sketch": sketch}));
    assert_eq!(feature(&mut s, "outline")["sketch"]["metadata"], json!({"bottom": {"stamp.role": "cut"}}));
    // A new line of a dropped name carries none of the old properties.
    run(&mut s, "edit_sketch", json!({"name": "outline", "lines": {"ghost": {"from": "a", "to": "b"}}}));
    assert_eq!(properties(&mut s, "outline", &["ghost"]), json!([{}]));
    // An empty property name is not read.
    let mut t = Session::default();
    let e = t
        .execute(
            "add",
            &json!({"name": "outline", "op": "sketch", "sketch": {"points": {"a": [0, 0], "b": [10, 0]},
            "lines": {"bottom": {"from": "a", "to": "b"}}, "metadata": {"bottom": {"": 1}}}}),
        )
        .unwrap_err();
    assert!(e.contains("empty property name"), "{e}");
    // A whole sketch given to `edit` is normalized too; an explicit stale target in
    // `edit_sketch` is still an error, naming the curve.
    run(
        &mut s,
        "edit",
        json!({"name": "outline", "sketch": {"points": {"a": [0, 0], "b": [10, 0]},
        "lines": {"bottom": {"from": "a", "to": "b"}}, "metadata": {"bottom": {"stamp.role": "cut"}, "ghost": {"old": 1}}}}),
    );
    assert_eq!(feature(&mut s, "outline")["sketch"]["metadata"], json!({"bottom": {"stamp.role": "cut"}}));
    let e = s.execute("edit_sketch", &json!({"name": "outline", "metadata": {"nothing": {"x": 1}}})).unwrap_err();
    assert!(e.contains("no sketch primitive"), "{e}");
}

/// A file with a stale property on a curve it does not have opens cleaned, so the name given
/// again later is new; one with an empty property name is refused, naming the feature.
#[test]
fn an_opened_file_with_stale_metadata_is_cleaned() {
    let dir = std::env::temp_dir().join(format!("datum-metadata-open-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("stale.json");
    let mut s = framed();
    run(&mut s, "edit_metadata", json!({"name": "outline", "elements": ["bottom"], "set": {"note": "old"}}));
    run(&mut s, "save", json!({"path": path.display().to_string()}));
    let mut file: Value = serde_json::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
    file["features"][0]["sketch"]["metadata"]["ghost"] = json!({"note": "old"});
    std::fs::write(&path, file.to_string()).unwrap();
    let mut t = Session::default();
    run(&mut t, "open", json!({"path": path.display().to_string()}));
    assert_eq!(feature(&mut t, "outline")["sketch"]["metadata"], json!({"bottom": {"note": "old"}}));
    run(&mut t, "edit_sketch", json!({"name": "outline", "lines": {"ghost": {"from": "a", "to": "b"}}}));
    assert_eq!(properties(&mut t, "outline", &["ghost"]), json!([{}]), "no ghost on the new line");
    // An empty property name in the file is an error of the command, naming the feature.
    let mut file: Value = serde_json::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
    file["features"][0]["sketch"]["metadata"]["bottom"] = json!({"": 1});
    std::fs::write(&path, file.to_string()).unwrap();
    let mut u = Session::default();
    let e = u.execute("open", &json!({"path": path.display().to_string()})).unwrap_err();
    assert!(e.contains("outline") && e.contains("empty property name"), "{e}");
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn feature_marks_round_trip_and_levels_override_individual_properties() {
    let mut s = framed();
    let edit = |s: &mut Session, set: Value, remove: Value| {
        run(s, "edit_metadata", json!({"name": "outline", "target": "feature", "set": set, "remove": remove}))
    };
    let query = |s: &mut Session| run(s, "query_metadata", json!({"name": "outline", "target": "feature"}))["properties"].clone();
    let stamp = json!({"stamp.section_side": "outer", "stamp.cut": "flat", "note": [1, null]});
    assert_eq!(edit(&mut s, stamp.clone(), json!([]))["computed"], 0);
    assert_eq!(feature(&mut s, "outline")["metadata"], stamp);
    assert_eq!(run(&mut s, "query_metadata", json!({"name": "outline"}))["elements"], json!([]));

    run(&mut s, "add_level", json!({"name": "variant", "part": "plate"}));
    edit(&mut s, json!({"stamp.cut": "tool", "memo": null}), json!(["note"]));
    assert_eq!(query(&mut s), json!({"stamp.section_side": "outer", "stamp.cut": "tool", "memo": null}));
    run(&mut s, "undo", json!({}));
    assert_eq!(query(&mut s), stamp);
    run(&mut s, "redo", json!({}));

    run(&mut s, "activate", json!({"level": null}));
    edit(&mut s, json!({"stamp.section_side": "inner", "note": "changed upstream"}), json!([]));
    let dir = std::env::temp_dir().join(format!("datum-feature-metadata-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("model.json");
    run(&mut s, "save", json!({"path": path}));
    let mut reopened = Session::default();
    run(&mut reopened, "open", json!({"path": path}));
    assert_eq!(query(&mut reopened), json!({"stamp.section_side": "inner", "stamp.cut": "flat", "note": "changed upstream"}));
    run(&mut reopened, "activate", json!({"level": "variant"}));
    assert_eq!(query(&mut reopened), json!({"stamp.section_side": "inner", "stamp.cut": "tool", "memo": null}));
    std::fs::remove_dir_all(&dir).unwrap();
}
