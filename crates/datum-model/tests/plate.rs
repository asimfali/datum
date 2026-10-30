//! A face of sheet metal (a plate): the closed profiles of a sketch extruded by the thickness of
//! the part's sheet rule, on the sketch's front, the back or both ways; the open chain beside it
//! left out; the rule edited in the library changes the thickness.
#![allow(clippy::unwrap_used, clippy::panic)]

use datum_model::Session;
use serde_json::{Value, json};

fn run(s: &mut Session, cmd: &str, p: Value) -> Value {
    s.execute(cmd, &p).unwrap_or_else(|e| panic!("{cmd}: {e}"))
}

fn props(s: &mut Session, name: &str) -> Value {
    run(s, "mass_properties", json!({"name": name}))
}

fn volume(s: &mut Session, name: &str) -> f64 {
    props(s, name)["volume"].as_f64().unwrap()
}

fn put(s: &mut Session, level: &str, element: Value) {
    run(s, "put_library", json!({"level": level, "element": element}));
}

/// A session in a project with the sheet rule `Лист` (thickness 2, radius `t`) and a part rule
/// `Сталь`, and a sketch: a 100 x 50 rectangle with a round hole of diameter 10, and an open L.
fn project(name: &str) -> Session {
    let root = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join(name);
    let _ = std::fs::remove_dir_all(&root);
    std::fs::create_dir_all(root.join("part")).unwrap();
    std::fs::write(root.join("datum-project.json"), "{}").unwrap();
    let mut s = Session::default();
    run(&mut s, "save", json!({"path": root.join("part/plate.json")}));
    put(&mut s, "material", json!({"name": "Сталь", "density": 7850}));
    put(&mut s, "rule", json!({"name": "Сталь", "kind": "part", "material": "Сталь"}));
    put(&mut s, "rule", json!({"name": "Лист", "kind": "sheet", "material": "Сталь", "thickness": 2, "radius": "t"}));
    run(
        &mut s,
        "add",
        json!({"name": "skel", "op": "sketch", "sketch": {
            "points": {"a": [0, 0], "b": [100, 0], "c": [100, 50], "d": [0, 50], "h": [50, 25],
                "p1": [200, 0], "p2": [200, 50], "p3": [250, 50]},
            "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}, "cd": {"from": "c", "to": "d"}, "da": {"from": "d", "to": "a"},
                "l1": {"from": "p1", "to": "p2"}, "l2": {"from": "p2", "to": "p3"}},
            "circles": {"hole": {"center": "h", "diameter": 10}}}}),
    );
    s
}

fn plate(s: &mut Session, extra: Value) -> Result<Value, String> {
    let mut p = json!({"name": "face", "sketch": "skel", "rule": "Лист"});
    p.as_object_mut().unwrap().extend(extra.as_object().unwrap().clone());
    s.execute("plate", &p)
}

fn close(a: f64, b: f64) {
    assert!((a - b).abs() <= 1e-9 * b.abs(), "{a} vs {b}");
}

/// The volume is the profile's area (the hole taken out, the open chain left out) times the
/// rule's thickness, and follows the rule when it is edited; the part is a sheet metal one.
#[test]
fn a_plate_is_the_profile_by_the_rules_thickness() {
    let mut s = project("plate_volume");
    let r = plate(&mut s, json!({})).unwrap();
    assert_eq!(r["rule"], "Лист", "{r}");
    let body = r["body"].as_str().unwrap().to_string();
    let area = 5000.0 - 25.0 * std::f64::consts::PI;
    close(volume(&mut s, &body), area * 2.0);
    let e = s.execute("set_part", &json!({"body": body, "rule": "Сталь"})).unwrap_err();
    assert!(e.contains("sheet metal part"), "{e}");
    put(&mut s, "rule", json!({"name": "Лист", "kind": "sheet", "material": "Сталь", "thickness": 3, "radius": "t"}));
    close(volume(&mut s, &body), area * 3.0);
}

/// `flip` puts the thickness behind the sketch, `symmetric` half each way.
#[test]
fn a_plate_lies_on_either_side() {
    let mut s = project("plate_sides");
    let front = plate(&mut s, json!({"name": "f"})).unwrap()["body"].as_str().unwrap().to_string();
    let back = plate(&mut s, json!({"name": "b", "flip": true})).unwrap()["body"].as_str().unwrap().to_string();
    let both = plate(&mut s, json!({"name": "m", "symmetric": true})).unwrap()["body"].as_str().unwrap().to_string();
    let z = |s: &mut Session, b: &str| {
        let p = props(s, b);
        (p["min"][2].as_f64().unwrap(), p["max"][2].as_f64().unwrap())
    };
    assert_eq!(z(&mut s, &front), (0.0, 2.0));
    assert_eq!(z(&mut s, &back), (-2.0, 0.0));
    assert_eq!(z(&mut s, &both), (-1.0, 1.0));
    let v = volume(&mut s, &front);
    close(volume(&mut s, &back), v);
    close(volume(&mut s, &both), v);
}

/// `profiles` names a curve of the loop; the rule is required; the blank is 100 x 50.
#[test]
fn a_plate_takes_profiles_and_a_rule_and_unfolds() {
    let mut s = project("plate_profiles");
    plate(&mut s, json!({"profiles": ["ab"]})).unwrap();
    let e = s.execute("plate", &json!({"name": "norule", "sketch": "skel"})).unwrap_err();
    assert!(e.contains("rule"), "{e}");
    let flats = run(&mut s, "flats", json!({}));
    let flat = &flats["flats"][0];
    let (w, h) = (flat["width"].as_f64().unwrap(), flat["height"].as_f64().unwrap());
    let (w, h) = if w > h { (w, h) } else { (h, w) };
    close(w, 100.0);
    close(h, 50.0);
}
