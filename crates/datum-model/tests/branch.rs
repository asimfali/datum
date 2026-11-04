//! The `branch` command (D227): a branch pipe set on a main one adds `branch_hole` and `saddle`
//! together, or, when either is refused, nothing.
#![allow(clippy::unwrap_used, clippy::panic)]

use datum_model::Session;
use serde_json::{Value, json};

fn run(s: &mut Session, cmd: &str, p: Value) -> Value {
    s.execute(cmd, &p).unwrap_or_else(|e| panic!("{cmd}: {e}"))
}

/// A session in a project with the sheet rule `Лист` (thickness 2, K-factor 0.44).
fn project(name: &str) -> Session {
    let root = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join(name);
    let _ = std::fs::remove_dir_all(&root);
    std::fs::create_dir_all(root.join("part")).unwrap();
    std::fs::write(root.join("datum-project.json"), "{}").unwrap();
    let mut s = Session::default();
    run(&mut s, "save", json!({"path": root.join("part/sheet.json")}));
    run(&mut s, "put_library", json!({"level": "material", "element": {"name": "Сталь", "density": 7850}}));
    let rule = json!({"name": "Лист", "kind": "sheet", "material": "Сталь", "thickness": 2, "radius": 2, "k_factor": 0.44});
    run(&mut s, "put_library", json!({"level": "rule", "element": rule}));
    s
}

/// A pipe `name`: the line `a`-`b` of a sketch in `plane` rolled about its axis `o`-`x`.
fn pipe(s: &mut Session, name: &str, plane: Option<Value>, points: Value) {
    let sketch = json!({"points": points, "lines": {"axis": {"from": "o", "to": "x", "construction": true}, "wall": {"from": "a", "to": "b"}}});
    let mut profile = json!({"name": format!("{name}_profile"), "op": "sketch", "sketch": sketch});
    if let Some(plane) = plane {
        profile["plane"] = plane;
    }
    run(s, "add", profile);
    run(s, "rolled", json!({"name": name, "sketch": format!("{name}_profile"), "rule": "Лист", "axis": "axis", "gap": 2}));
}

/// The main pipe along X (inner radius 50), and a branch along Z standing 20 from its axis, or
/// (`parallel`) along X as well.
fn tee(name: &str, parallel: bool) -> Session {
    let mut s = project(name);
    pipe(&mut s, "main", None, json!({"o": [-1000, 0], "x": [1000, 0], "a": [-100, 50], "b": [100, 50]}));
    if parallel {
        pipe(&mut s, "branch", None, json!({"o": [-1000, 0], "x": [1000, 0], "a": [-100, 20], "b": [100, 20]}));
    } else {
        let plane = json!({"origin": [0, 0, 0], "x_axis": [0, 1, 0], "normal": [1, 0, 0]});
        pipe(&mut s, "branch", Some(plane), json!({"o": [0, 0], "x": [0, 10], "a": [20, 0], "b": [20, 120]}));
    }
    s
}

fn names(s: &mut Session) -> Vec<String> {
    let inspect = run(s, "inspect", json!({}));
    inspect["features"].as_array().unwrap().iter().map(|f| f["name"].as_str().unwrap().to_string()).collect()
}

fn params() -> Value {
    json!({"main": "main", "branch": "branch", "hole": "main_with_hole", "saddle": "branch_saddle"})
}

#[test]
fn a_branch_adds_the_hole_and_the_saddle() {
    let mut s = tee("branch_tee", false);
    let before = names(&mut s);
    run(&mut s, "branch", params());
    let after = names(&mut s);
    assert_eq!(after[..before.len()], before[..]);
    assert_eq!(after[before.len()..], ["main_with_hole", "branch_saddle"]);
    let inspect = run(&mut s, "inspect", json!({}));
    assert!(inspect["failures"].as_array().is_none_or(Vec::is_empty), "{inspect}");
}

#[test]
fn a_refused_branch_leaves_the_features_as_they_were() {
    let mut s = tee("branch_parallel", true);
    let before = names(&mut s);
    let err = s.execute("branch", &params()).unwrap_err();
    assert!(err.contains("parallel"), "{err}");
    assert_eq!(names(&mut s), before);
    assert!(!before.iter().any(|n| n == "main_with_hole" || n == "branch_saddle"));
}
