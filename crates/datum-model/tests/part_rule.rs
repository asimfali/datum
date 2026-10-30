//! A part's rule (D189): set on the part and in its variants, of the part's kind, reported by
//! `product` with its material and mass; a rule the library lacks does not stop the model opening.
#![allow(clippy::unwrap_used, clippy::panic)]

use datum_model::Session;
use serde_json::{Value, json};

fn run(s: &mut Session, cmd: &str, p: Value) -> Value {
    s.execute(cmd, &p).unwrap_or_else(|e| panic!("{cmd}: {e}"))
}

/// A project in a temp folder with materials `Сталь` (7850 kg/m³) and `Пластик` (no density),
/// part rules `Сталь` and `Пластик`, a sheet rule `Лист`; a session on a plate 100 × 50 × 10
/// (50 000 mm³) saved in it. Returns the session and the model's file.
fn project(name: &str) -> (Session, std::path::PathBuf) {
    let root = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join(name);
    let _ = std::fs::remove_dir_all(&root);
    std::fs::create_dir_all(root.join("part")).unwrap();
    std::fs::write(root.join("datum-project.json"), "{}").unwrap();
    let mut s = Session::default();
    let file = root.join("part/plate.json");
    run(&mut s, "save", json!({"path": file}));
    let put = |s: &mut Session, level: &str, element: Value| run(s, "put_library", json!({"level": level, "element": element}));
    put(&mut s, "material", json!({"name": "Сталь", "density": 7850}));
    put(&mut s, "material", json!({"name": "Пластик"}));
    put(&mut s, "rule", json!({"name": "Сталь", "kind": "part", "material": "Сталь"}));
    put(&mut s, "rule", json!({"name": "Пластик", "kind": "part", "material": "Пластик"}));
    put(&mut s, "rule", json!({"name": "Лист", "kind": "sheet", "material": "Сталь", "thickness": 2, "radius": "t"}));
    run(
        &mut s,
        "add",
        json!({"name": "outline", "op": "sketch", "sketch": {
            "points": {"a": [0, 0], "b": [100, 0], "c": [100, 50], "d": [0, 50]},
            "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}, "cd": {"from": "c", "to": "d"}, "da": {"from": "d", "to": "a"}}}}),
    );
    run(&mut s, "extrude", json!({"name": "plate", "sketch": "outline", "mode": "new", "z1": 10}));
    (s, file)
}

/// The `product` entry of part `part`.
fn product(s: &mut Session, part: &str) -> Value {
    let r = run(s, "product", json!({}));
    r["parts"].as_array().unwrap().iter().find(|p| p["part"] == part).unwrap_or_else(|| panic!("no part {part}: {r}")).clone()
}

fn mass(p: &Value) -> f64 {
    p["mass"].as_f64().unwrap_or_else(|| panic!("no mass: {p}"))
}

/// The mass of 50 000 mm³ of a material of `density` kg/m³.
fn plate_mass(density: f64) -> f64 {
    50000.0 * density * 1e-9
}

/// A rule is saved with the part, `product` reports it with the material and the mass; `null`
/// takes it off and the part's record goes when it held nothing else.
#[test]
fn rule_gives_material_and_mass() {
    let (mut s, file) = project("part_rule_set");
    let none = product(&mut s, "plate");
    assert_eq!((&none["rule"], &none["material"], &none["mass"], none.get("rule_missing")), (&Value::Null, &Value::Null, &Value::Null, None));
    let r = run(&mut s, "set_part", json!({"body": "plate", "rule": "Сталь"}));
    assert_eq!(r["parts"][0]["rule"], "Сталь", "{r}");
    let p = product(&mut s, "plate");
    assert_eq!((&p["rule"], &p["material"], p.get("rule_missing")), (&json!("Сталь"), &json!("Сталь"), None), "{p}");
    assert_eq!(mass(&p), plate_mass(7850.0));
    // A material without a density: a material, no mass.
    run(&mut s, "set_part", json!({"body": "plate", "rule": "Пластик"}));
    let p = product(&mut s, "plate");
    assert_eq!((&p["material"], &p["mass"]), (&json!("Пластик"), &Value::Null), "{p}");
    run(&mut s, "set_part", json!({"body": "plate", "rule": "Сталь"}));
    run(&mut s, "save", json!({}));
    let saved: Value = serde_json::from_str(&std::fs::read_to_string(&file).unwrap()).unwrap();
    assert_eq!(saved["parts"]["plate"]["rule"], "Сталь", "{saved}");
    let mut t = Session::default();
    run(&mut t, "open", json!({"path": file}));
    assert_eq!(mass(&product(&mut t, "plate")), plate_mass(7850.0));
    run(&mut s, "set_part", json!({"body": "plate", "rule": null}));
    let p = product(&mut s, "plate");
    assert_eq!((&p["rule"], &p["mass"]), (&Value::Null, &Value::Null));
    run(&mut s, "save", json!({}));
    let saved: Value = serde_json::from_str(&std::fs::read_to_string(&file).unwrap()).unwrap();
    assert!(saved.get("parts").is_none_or(|p| p.get("plate").is_none()), "{saved}");
}

/// A rule the library lacks, or of another kind than the part, is refused.
#[test]
fn rule_of_the_parts_kind() {
    let (mut s, _) = project("part_rule_kind");
    let set = |s: &mut Session, body: &str, rule: &str| s.execute("set_part", &json!({"body": body, "rule": rule})).unwrap_err();
    assert!(set(&mut s, "plate", "Нет такого").contains("no rule `Нет такого`"));
    assert!(set(&mut s, "plate", "Лист").contains("is for a sheet metal part; `plate` is an ordinary part"));
    bent(&mut s, json!({"thickness": 1, "radius": 1}));
    assert!(set(&mut s, "bent", "Сталь").contains("is for an ordinary part; `bent` is a sheet metal part"));
    // Nothing of the refused ones stayed.
    assert_eq!(product(&mut s, "bent")["rule"], Value::Null);
    assert_eq!(product(&mut s, "plate")["rule"], Value::Null);
}

/// Sheet metal `bent`, an L 20 × 15 bent 10 wide, with `numbers` of its own (thickness, radius)
/// or none.
fn bent(s: &mut Session, numbers: Value) {
    run(
        s,
        "add",
        json!({"name": "path", "op": "sketch", "sketch": {
            "points": {"p": [0, 0], "q": [20, 0], "r": [20, 15]}, "lines": {"base": {"from": "p", "to": "q"}, "wall": {"from": "q", "to": "r"}}}}),
    );
    let mut sheet = json!({"name": "bent", "op": "sheet", "inputs": ["path"], "side": "left", "z0": 0, "z1": 10, "start": "p"});
    sheet.as_object_mut().unwrap().extend(numbers.as_object().unwrap().clone());
    run(s, "add", sheet);
}

fn volume(s: &mut Session, name: &str) -> f64 {
    run(s, "mass_properties", json!({"name": name}))["volume"].as_f64().unwrap()
}

/// A sheet's thickness and bend radius are its rule's (D193): set, edited in the library, gone;
/// a sheet of no numbers and no rule fails asking for one.
#[test]
fn a_sheet_takes_its_rules_thickness() {
    let (mut s, _) = project("part_rule_sheet");
    bent(&mut s, json!({}));
    let failed = run(&mut s, "inspect", json!({}));
    assert!(failed.to_string().contains("from the part's sheet rule"), "{failed}");
    run(&mut s, "set_part", json!({"body": "bent", "rule": "Лист"}));
    let by_rule = volume(&mut s, "bent");
    // The same as numbers of its own, 2 and `t` = 2.
    let (mut own, _) = project("part_rule_sheet_own");
    bent(&mut own, json!({"thickness": 2, "radius": 2}));
    assert_eq!(by_rule, volume(&mut own, "bent"));
    // The rule thinner: the sheet follows.
    run(
        &mut s,
        "put_library",
        json!({"level": "rule", "element": {"name": "Лист", "kind": "sheet", "material": "Сталь", "thickness": 1, "radius": "t"}}),
    );
    let (mut thin, _) = project("part_rule_sheet_thin");
    bent(&mut thin, json!({"thickness": 1, "radius": 1}));
    assert_eq!(volume(&mut s, "bent"), volume(&mut thin, "bent"));
    assert!(mass(&product(&mut s, "bent")) > 0.0);
    // No rule: no numbers again.
    run(&mut s, "set_part", json!({"body": "bent", "rule": null}));
    assert!(s.execute("mass_properties", &json!({"name": "bent"})).is_err());
}

/// A sheet rule needs its thickness and its bend radius.
#[test]
fn a_sheet_rule_has_both_numbers() {
    let (mut s, _) = project("part_rule_sheet_numbers");
    let put = |s: &mut Session, e: Value| s.execute("put_library", &json!({"level": "rule", "element": e})).unwrap_err();
    assert!(put(&mut s, json!({"name": "Тонкий", "kind": "sheet", "material": "Сталь", "radius": 1})).contains("`thickness`"));
    assert!(put(&mut s, json!({"name": "Тонкий", "kind": "sheet", "material": "Сталь", "thickness": 1})).contains("`radius`"));
}

/// A variant takes its own rule; a variant of it without one has its parent's, `null` returns
/// to the parent's; the root keeps its own.
#[test]
fn variants_take_the_rule() {
    let (mut s, _) = project("part_rule_variants");
    run(&mut s, "set_part", json!({"body": "plate", "rule": "Пластик"}));
    run(&mut s, "add_level", json!({"name": "steel", "part": "plate"}));
    run(&mut s, "set_part", json!({"body": "plate", "rule": "Сталь"}));
    run(&mut s, "add_level", json!({"name": "steel_long", "parent": "steel"}));
    run(&mut s, "choose", json!({"level": "steel"}));
    let p = product(&mut s, "plate");
    assert_eq!((&p["level"], &p["rule"]), (&json!("steel"), &json!("Сталь")), "{p}");
    assert_eq!(mass(&p), plate_mass(7850.0));
    run(&mut s, "choose", json!({"level": "steel_long"}));
    assert_eq!(product(&mut s, "plate")["rule"], "Сталь", "inherited from the parent variant");
    // `level` names the variant the command works on, the root staying active.
    run(&mut s, "activate", json!({"level": null}));
    run(&mut s, "set_part", json!({"body": "plate", "rule": "Пластик", "level": "steel_long"}));
    assert_eq!(product(&mut s, "plate")["rule"], "Пластик");
    run(&mut s, "set_part", json!({"body": "plate", "rule": null, "level": "steel_long"}));
    assert_eq!(product(&mut s, "plate")["rule"], "Сталь", "the parent's again");
    run(&mut s, "choose", json!({"part": "plate"}));
    let p = product(&mut s, "plate");
    assert_eq!((&p["level"], &p["rule"], &p["mass"]), (&Value::Null, &json!("Пластик"), &Value::Null), "the root's own: {p}");
    let levels = run(&mut s, "levels", json!({}));
    assert_eq!(levels["parts"][0]["rule"], "Пластик", "{levels}");
    let rule = |l: &str| levels["levels"].as_array().unwrap().iter().find(|v| v["name"] == l).unwrap()["rule"].clone();
    assert_eq!((rule("steel"), rule("steel_long")), (json!("Сталь"), json!("Сталь")), "each variant's rule in force: {levels}");
}

/// A variant of another part cannot be given a part's rule.
#[test]
fn rule_in_another_parts_variant_is_refused() {
    let (mut s, _) = project("part_rule_other");
    run(
        &mut s,
        "add",
        json!({"name": "outline2", "op": "sketch", "sketch": {
        "points": {"a": [200, 0], "b": [250, 0], "c": [250, 50], "d": [200, 50]},
        "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}, "cd": {"from": "c", "to": "d"}, "da": {"from": "d", "to": "a"}}}}),
    );
    run(&mut s, "extrude", json!({"name": "block", "sketch": "outline2", "mode": "new", "z1": 10}));
    run(&mut s, "add_level", json!({"name": "long", "part": "plate"}));
    let e = s.execute("set_part", &json!({"body": "block", "rule": "Сталь"})).unwrap_err();
    assert!(e.contains("variant of `plate`, not of `block`"), "{e}");
    assert_eq!(product(&mut s, "block")["rule"], Value::Null);
}

/// `extrude` with a rule gives the new body that rule; joined to an existing body it changes
/// that part's.
#[test]
fn extrude_gives_the_rule() {
    let (mut s, _) = project("part_rule_extrude");
    run(
        &mut s,
        "add",
        json!({"name": "outline2", "op": "sketch", "sketch": {
        "points": {"a": [200, 0], "b": [250, 0], "c": [250, 50], "d": [200, 50]},
        "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}, "cd": {"from": "c", "to": "d"}, "da": {"from": "d", "to": "a"}}}}),
    );
    let r = run(&mut s, "extrude", json!({"name": "block", "sketch": "outline2", "mode": "new", "z1": 10, "rule": "Сталь"}));
    assert_eq!(r["rule"], "Сталь", "{r}");
    assert_eq!(mass(&product(&mut s, "block")), plate_mass(7850.0) / 2.0);
    run(
        &mut s,
        "add",
        json!({"name": "outline3", "op": "sketch", "sketch": {
        "points": {"a": [0, 0], "b": [10, 0], "c": [10, 10], "d": [0, 10]},
        "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}, "cd": {"from": "c", "to": "d"}, "da": {"from": "d", "to": "a"}}}}),
    );
    let r =
        run(&mut s, "extrude", json!({"name": "boss", "sketch": "outline3", "mode": "join", "body": "plate", "z0": 10, "z1": 20, "rule": "Пластик"}));
    assert_eq!(r["rule"], "Пластик", "{r}");
    assert_eq!(product(&mut s, "plate")["rule"], "Пластик");
    assert_eq!(product(&mut s, "block")["rule"], "Сталь");
}

/// A file naming a rule the library lacks opens; `product` says it is missing, with no mass.
#[test]
fn missing_rule_does_not_stop_opening() {
    let (mut s, file) = project("part_rule_missing");
    run(&mut s, "set_part", json!({"body": "plate", "rule": "Сталь"}));
    run(&mut s, "save", json!({}));
    let mut saved: Value = serde_json::from_str(&std::fs::read_to_string(&file).unwrap()).unwrap();
    saved["parts"]["plate"]["rule"] = json!("Пропала");
    std::fs::write(&file, saved.to_string()).unwrap();
    let mut t = Session::default();
    run(&mut t, "open", json!({"path": file}));
    let p = product(&mut t, "plate");
    assert_eq!((&p["rule"], &p["rule_missing"], &p["material"], &p["mass"]), (&json!("Пропала"), &json!(true), &Value::Null, &Value::Null), "{p}");
}
