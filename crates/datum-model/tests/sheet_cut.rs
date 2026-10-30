//! Sheet metal's cut (D196): a sketch on a face of a sheet metal part cuts through its thickness
//! alone, as the blank is punched; nothing else is done to sheet metal by an extrusion.
#![allow(clippy::unwrap_used, clippy::panic)]

use datum_model::Session;
use serde_json::{Value, json};

fn run(s: &mut Session, cmd: &str, p: Value) -> Value {
    s.execute(cmd, &p).unwrap_or_else(|e| panic!("{cmd}: {e}"))
}

fn volume(s: &mut Session, name: &str) -> f64 {
    run(s, "mass_properties", json!({"name": name}))["volume"].as_f64().unwrap()
}

/// A session in a project with the sheet rule `Лист` (thickness 2, radius `t`) and a sketch
/// `path` of `sketch`.
fn project(name: &str, sketch: Value) -> Session {
    let root = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join(name);
    let _ = std::fs::remove_dir_all(&root);
    std::fs::create_dir_all(root.join("part")).unwrap();
    std::fs::write(root.join("datum-project.json"), "{}").unwrap();
    let mut s = Session::default();
    run(&mut s, "save", json!({"path": root.join("part/sheet.json")}));
    run(&mut s, "put_library", json!({"level": "material", "element": {"name": "Сталь", "density": 7850}}));
    run(
        &mut s,
        "put_library",
        json!({"level": "rule", "element": {"name": "Лист", "kind": "sheet", "material": "Сталь", "thickness": 2, "radius": "t"}}),
    );
    run(&mut s, "add", json!({"name": "path", "op": "sketch", "sketch": sketch}));
    s
}

/// A sketch of circle `c` of diameter 10 at (x, y) on face `face` of body `body`.
fn hole_on(s: &mut Session, name: &str, body: &str, face: &str, x: f64, y: f64) {
    let hole = json!({"points": {"h": [x, y]}, "circles": {"c": {"center": "h", "diameter": 10}}});
    run(s, "add", json!({"name": name, "op": "sketch", "inputs": [body], "face": face, "sketch": hole}));
}

const HOLE: f64 = 25.0 * std::f64::consts::PI;

/// A face of sheet metal: a hole through it; a boss or a depth refused.
#[test]
fn a_plate_is_cut_through_its_thickness() {
    let rectangle = json!({"points": {"a": [0, 0], "b": [100, 0], "c": [100, 50], "d": [0, 50]},
        "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}, "cd": {"from": "c", "to": "d"}, "da": {"from": "d", "to": "a"}}});
    let mut s = project("sheet_cut_plate", rectangle);
    run(&mut s, "plate", json!({"name": "face", "sketch": "path", "rule": "Лист"}));
    let top = run(&mut s, "query_faces", json!({"name": "face", "normal": [0, 0, 1]}))["faces"][0]["id"].as_str().unwrap().to_string();
    hole_on(&mut s, "hole", "face", &top, 30.0, 25.0);
    let e = s.execute("extrude", &json!({"name": "boss", "sketch": "hole", "mode": "join", "z1": 5})).unwrap_err();
    assert!(e.contains("only cut"), "{e}");
    let e = s.execute("extrude", &json!({"name": "pocket", "sketch": "hole", "mode": "cut", "z1": -1})).unwrap_err();
    assert!(e.contains("no `z1`"), "{e}");
    let r = run(&mut s, "extrude", json!({"name": "cut", "sketch": "hole", "mode": "cut"}));
    assert_eq!(r["body"], "face", "{r}");
    let v = volume(&mut s, "cut");
    assert!((v - (5000.0 - HOLE) * 2.0).abs() < 1e-9 * v, "{v}");
}

/// A U of sheet metal: a hole in one wall does not go on through the opposite one.
#[test]
fn a_u_is_cut_through_one_wall() {
    let u = json!({"points": {"a": [0, 60], "b": [0, 0], "c": [80, 0], "d": [80, 60]},
        "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}, "cd": {"from": "c", "to": "d"}}});
    let mut s = project("sheet_cut_u", u);
    run(&mut s, "sheet", json!({"name": "u", "sketch": "path", "rule": "Лист", "side": "right", "z1": 40}));
    let whole = volume(&mut s, "u");
    // The outer face of the wall at x = 0, facing −x.
    let wall = run(&mut s, "query_faces", json!({"name": "u", "normal": [-1, 0, 0]}))["faces"][0]["id"].as_str().unwrap().to_string();
    hole_on(&mut s, "hole", "u", &wall, 30.0, -20.0);
    run(&mut s, "extrude", json!({"name": "cut", "sketch": "hole", "mode": "cut"}));
    let v = volume(&mut s, "cut");
    assert!((whole - v - HOLE * 2.0).abs() < 1e-6, "one wall's hole: {whole} − {v}");
}
