//! A sheet rule's K-factor (D197): the flat pattern and a cut across a bend take it from the rule.
#![allow(clippy::unwrap_used, clippy::panic)]

use datum_model::Session;
use serde_json::{Value, json};

fn run(s: &mut Session, cmd: &str, p: Value) -> Value {
    s.execute(cmd, &p).unwrap_or_else(|e| panic!("{cmd}: {e}"))
}

/// A session with a sheet along an L and the rule `Лист` (thickness 2, radius `t`) with `rule_k`.
fn project(name: &str, rule_k: Option<f64>) -> Session {
    let root = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join(name);
    let _ = std::fs::remove_dir_all(&root);
    std::fs::create_dir_all(root.join("part")).unwrap();
    std::fs::write(root.join("datum-project.json"), "{}").unwrap();
    let mut s = Session::default();
    run(&mut s, "save", json!({"path": root.join("part/sheet.json")}));
    run(&mut s, "put_library", json!({"level": "material", "element": {"name": "Сталь", "density": 7850}}));
    let mut rule = json!({"name": "Лист", "kind": "sheet", "material": "Сталь", "thickness": 2, "radius": "t"});
    if let Some(k) = rule_k {
        rule["k_factor"] = json!(k);
    }
    run(&mut s, "put_library", json!({"level": "rule", "element": rule}));
    let l = json!({"points": {"a": [0, 0], "b": [50, 0], "c": [50, 30]},
        "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}}});
    run(&mut s, "add", json!({"name": "path", "op": "sketch", "sketch": l}));
    run(&mut s, "sheet", json!({"name": "l", "sketch": "path", "rule": "Лист", "side": "left", "z1": 20}));
    s
}

/// The blank's width and the K-factor the report of `flats` gives for the sheet.
fn blank(s: &mut Session) -> (f64, Value) {
    let r = run(s, "flats", json!({}));
    let f = &r["flats"][0];
    (f["width"].as_f64().unwrap(), f["k_factor"].clone())
}

/// The rule's K is the blank's, whatever the flat settings say; the report shows it.
#[test]
fn a_rule_gives_the_k_factor() {
    let (mut with, mut set) = (project("rule_k_with", Some(0.44)), project("rule_k_set", None));
    run(&mut set, "set_flat", json!({"feature": "l", "k_factor": 0.44}));
    let (w, k) = blank(&mut with);
    assert_eq!(k, json!(0.44));
    assert_eq!(w, blank(&mut set).0);
    // Settings of another K change nothing: the rule is the only place.
    run(&mut with, "set_flat", json!({"feature": "l", "k_factor": 0.3}));
    assert_eq!(blank(&mut with), (w, json!(0.44)));
}

/// A rule without one: the flat settings' K, 0.5 by default.
#[test]
fn a_rule_without_k_leaves_the_settings() {
    let mut s = project("rule_k_none", None);
    let (w, k) = blank(&mut s);
    assert_eq!(k, json!(0.5));
    run(&mut s, "set_flat", json!({"feature": "l", "k_factor": 0.44}));
    let (w2, k2) = blank(&mut s);
    assert_eq!(k2, json!(0.44));
    assert!(w2 < w, "{w2} {w}");
}

/// A K outside 0 to 1 is no K.
#[test]
fn a_k_factor_out_of_range_is_refused() {
    let mut s = project("rule_k_bad", None);
    for k in [json!(1.5), json!(-0.1), json!("0.4")] {
        let rule = json!({"name": "Плохой", "kind": "sheet", "material": "Сталь", "thickness": 2, "radius": "t", "k_factor": k});
        let e = s.execute("put_library", &json!({"level": "rule", "element": rule})).unwrap_err();
        assert!(e.contains("k_factor"), "{e}");
    }
}
