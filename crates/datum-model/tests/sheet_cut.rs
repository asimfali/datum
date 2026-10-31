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
        json!({"level": "rule", "element": {"name": "Лист", "kind": "sheet", "material": "Сталь", "thickness": 2, "radius": "t", "k_factor": 0.44}}),
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

/// A hole closer to a bend line than 2t + R (here 2·2 + 2) is cut, and told of (D198): in the
/// command's reply and in `inspect`; one in the middle of the wall is not. The wall runs along
/// the sketch's x from the bend line at 2.
#[test]
fn a_hole_near_a_bend_is_told() {
    let u = json!({"points": {"a": [0, 60], "b": [0, 0], "c": [80, 0], "d": [80, 60]},
        "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}, "cd": {"from": "c", "to": "d"}}});
    let mut s = project("sheet_cut_near_bend", u);
    run(&mut s, "sheet", json!({"name": "u", "sketch": "path", "rule": "Лист", "side": "right", "z1": 40}));
    let wall = run(&mut s, "query_faces", json!({"name": "u", "normal": [-1, 0, 0]}))["faces"][0]["id"].as_str().unwrap().to_string();
    hole_on(&mut s, "middle", "u", &wall, 30.0, -20.0);
    let r = run(&mut s, "extrude", json!({"name": "cut", "sketch": "middle", "mode": "cut"}));
    assert_eq!(r["warnings"], json!([]), "{r}");
    hole_on(&mut s, "near", "cut", &wall, 10.0, -20.0);
    let r = run(&mut s, "extrude", json!({"name": "cut2", "sketch": "near", "mode": "cut"}));
    let [w] = &r["warnings"].as_array().unwrap()[..] else { panic!("{r}") };
    assert_eq!(
        (&w["feature"], &w["code"], &w["profiles"], &w["clearance"]),
        (&json!("cut2"), &json!("sheet.cut.near_bend"), &json!(1), &json!(6.0)),
        "{w}"
    );
    let d = w["distance"].as_f64().unwrap();
    assert!((d - 3.0).abs() < 1e-9, "{d}");
    let inspect = run(&mut s, "inspect", json!({}));
    let of = |name: &str| inspect["features"].as_array().unwrap().iter().find(|f| f["name"] == name).unwrap()["warnings"].clone();
    assert_eq!((of("cut"), of("cut2")[0]["code"].clone()), (Value::Null, json!("sheet.cut.near_bend")));
}

/// A sheet along two lines, neither along the axes, and the wall of the first: the face of its left
/// side with its flat length, and a sketch of a hole there at `along` from the wall's start.
fn inclined(name: &str, along: impl Fn(f64) -> f64) -> (Session, f64) {
    let chain = json!({"points": {"a": [0, 0], "b": [20, 75], "c": [110, 120]},
        "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}}});
    let mut s = project(name, chain);
    run(&mut s, "sheet", json!({"name": "w", "sketch": "path", "rule": "Лист", "side": "left", "z1": 40}));
    let whole = volume(&mut s, "w");
    // The face looking along the left normal of the first line, the one with the largest dot.
    let len = libm::hypot(20.0, 75.0);
    let left = [-75.0 / len, 20.0 / len];
    let faces = run(&mut s, "query_faces", json!({"name": "w"}));
    // Bends have no normal: they are not planes.
    let dot = |f: &Value| f["normal"][0].as_f64().unwrap_or(-1.0) * left[0] + f["normal"][1].as_f64().unwrap_or(-1.0) * left[1];
    let face = faces["faces"].as_array().unwrap().iter().max_by(|a, b| dot(a).total_cmp(&dot(b))).unwrap();
    assert!(dot(face) > 1.0 - 1e-9, "{face}");
    // The sketch's x runs along z and its y along the wall; the flat part is the face's area over 40 long.
    let flat = face["area"].as_f64().unwrap() / 40.0;
    hole_on(&mut s, "hole", "w", face["id"].as_str().unwrap(), 20.0, along(flat));
    (s, whole)
}

/// A wall not along the axes: the opposite face is found all the same.
#[test]
fn an_inclined_wall_is_cut() {
    let (mut s, whole) = inclined("sheet_cut_inclined", |flat| flat / 2.0);
    let r = run(&mut s, "extrude", json!({"name": "cut", "sketch": "hole", "mode": "cut"}));
    assert!(r.get("failed").is_none_or(|f| f.as_array().is_some_and(Vec::is_empty)), "{r}");
    let v = volume(&mut s, "cut");
    assert!((whole - v - HOLE * 2.0).abs() < 1e-6, "one wall's hole: {whole} − {v}");
}

/// A circle over the whole of a short bend divides the bend's faces (D201): cut, and told of.
#[test]
fn a_hole_over_a_whole_bend_is_cut() {
    let (mut s, whole) = inclined("sheet_cut_bend", |flat| flat - 2.0);
    let r = run(&mut s, "extrude", json!({"name": "cut", "sketch": "hole", "mode": "cut"}));
    assert!(r["failed"].as_array().is_some_and(Vec::is_empty), "{r}");
    assert_eq!(r["warnings"][0]["code"], "sheet.cut.across_bend", "{r}");
    assert!(volume(&mut s, "cut") < whole);
}

// Cuts across bend lines (D197): the profile is punched through the blank, and the blank shows it
// as drawn.

/// Where a sketch of a cut across a bend lies: the sketch is on the outer face of the first wall
/// of a sheet 40 wide, whose flat part is `flat` long; the sketch maps (along the wall, z) to its
/// own (x, y).
struct Wall {
    id: String,
    flat: f64,
    origin: [f64; 3],
    x: [f64; 3],
    y: [f64; 3],
}

impl Wall {
    /// The sketch's coordinates of the point `along` the wall (world Y) at height `z`.
    fn at(&self, along: f64, z: f64) -> [f64; 2] {
        let (dy, dz) = (along - self.origin[1], z - self.origin[2]);
        let (a, b, c, d) = (self.x[1], self.y[1], self.x[2], self.y[2]);
        let det = a * d - b * c;
        [(dy * d - b * dz) / det, (a * dz - c * dy) / det]
    }
}

fn vec3(v: &Value) -> [f64; 3] {
    [v[0].as_f64().unwrap(), v[1].as_f64().unwrap(), v[2].as_f64().unwrap()]
}

/// The outer face (looking along −x) of the first wall of the sheet `name`, laid along world Y.
fn first_wall(s: &mut Session, name: &str) -> Wall {
    let faces = run(s, "query_faces", json!({"name": name, "normal": [-1, 0, 0]}));
    let f = &faces["faces"][0];
    let axes = &f["sketch_axes"];
    Wall {
        id: f["id"].as_str().unwrap().to_string(),
        flat: f["area"].as_f64().unwrap() / 40.0,
        origin: vec3(&axes["origin"]),
        x: vec3(&axes["x"]),
        y: vec3(&axes["y"]),
    }
}

/// A sheet `name` of 40 wide along `chain` (points `a`, `b`, `c`…), with the project `project`
/// would make; the sketch `path` is replaced.
fn sheet_along(project_name: &str, points: &[[f64; 2]]) -> Session {
    let names = ["a", "b", "c", "d", "e"];
    let pts: serde_json::Map<String, Value> = points.iter().enumerate().map(|(i, p)| (names[i].to_string(), json!(p))).collect();
    let lines: serde_json::Map<String, Value> =
        (1..points.len()).map(|i| (format!("{}{}", names[i - 1], names[i]), json!({"from": names[i - 1], "to": names[i]}))).collect();
    let mut s = project(project_name, json!({"points": pts, "lines": lines}));
    run(&mut s, "sheet", json!({"name": "l", "sketch": "path", "rule": "Лист", "side": "left", "z1": 40}));
    s
}

/// An L: up 60, then along +x 80.
fn l_sheet(name: &str) -> Session {
    sheet_along(name, &[[0.0, 0.0], [0.0, 60.0], [80.0, 60.0]])
}

/// A sketch `name` of one circle of `diameter` at (`along`, `z`) of the first wall of `base`, with
/// the sketch's K-factor from the rule; and the cut by `<name>_cut`. Returns the cut's result.
fn punch_circle(s: &mut Session, base: &str, name: &str, along: f64, z: f64, diameter: f64) -> Value {
    let wall = first_wall(s, base);
    let c = wall.at(along, z);
    let sketch = json!({"points": {"h": c}, "circles": {"c": {"center": "h", "diameter": diameter}}});
    run(s, "add", json!({"name": name, "op": "sketch", "inputs": [base], "face": wall.id, "sketch": sketch}));
    run(s, "extrude", json!({"name": format!("{name}_cut"), "sketch": name, "mode": "cut", "through": true, "wall": true}))
}

fn assert_cut(r: &Value) {
    assert!(r.get("failed").is_none_or(|f| f.as_array().is_some_and(Vec::is_empty)), "{r}");
}

fn failure(r: &Value) -> &str {
    r["failed"][0]["error"].as_str().unwrap_or_else(|| panic!("{r}"))
}

/// The count of lines, arcs and circles of the blank of `name`.
fn pieces(s: &mut Session, name: &str) -> [u64; 3] {
    let r = run(s, "flats", json!({}));
    let f = r["flats"].as_array().unwrap().iter().find(|f| f["feature"] == name).unwrap_or_else(|| panic!("{r}"));
    assert!(f.get("error").is_none(), "{f}");
    let p = &f["pieces"];
    [p["lines"].as_u64().unwrap(), p["arcs"].as_u64().unwrap(), p["circles"].as_u64().unwrap()]
}

const FULL_HOLE: f64 = HOLE * 2.0;

/// A hole across a bend: one circle in the blank, of the radius drawn, and the sheet lighter by
/// about the hole's volume.
#[test]
fn a_hole_across_a_bend_is_one_circle() {
    let mut s = l_sheet("bend_cut_circle");
    let whole = volume(&mut s, "l");
    let wall = first_wall(&mut s, "l");
    assert!((wall.flat - 58.0).abs() < 1e-9, "{}", wall.flat);
    let r = punch_circle(&mut s, "l", "hole", wall.flat - 2.0, 20.0, 10.0);
    assert_cut(&r);
    let v = volume(&mut s, "hole_cut");
    let lost = whole - v;
    assert!(lost > 0.8 * FULL_HOLE && lost < 1.2 * FULL_HOLE, "{lost} against {FULL_HOLE}");
    assert_eq!(pieces(&mut s, "hole_cut"), [4, 0, 1]);
    let dxf = std::env::temp_dir().join("datum_bend_cut_circle.dxf");
    run(&mut s, "export_dxf", json!({"name": "hole_cut", "path": dxf}));
    let text = std::fs::read_to_string(&dxf).unwrap();
    let _ = std::fs::remove_file(&dxf);
    assert_eq!(text.matches("CIRCLE").count(), 1, "{text}");
    // The radius as drawn, to its last digit (as a hole within a wall has it).
    let radius: f64 =
        text.split("CIRCLE").nth(1).and_then(|c| c.split("\n40\n").nth(1)).and_then(|r| r.lines().next()).unwrap().trim().parse().unwrap();
    assert!((radius - 5.0).abs() <= 4.0 * f64::EPSILON * 5.0, "{radius}");
}

/// A rectangle across a bend keeps its four sides: each one line in the blank.
#[test]
fn a_rectangle_across_a_bend_keeps_its_lines() {
    let mut s = l_sheet("bend_cut_rectangle");
    let outline = pieces(&mut s, "l");
    assert_eq!(outline[1] + outline[2], 0);
    let wall = first_wall(&mut s, "l");
    let (along, z) = (wall.flat, 20.0);
    let corners = [(-4.0, -3.0), (4.0, -3.0), (4.0, 3.0), (-4.0, 3.0)];
    let points: serde_json::Map<String, Value> =
        corners.iter().enumerate().map(|(i, (da, dz))| (format!("p{i}"), json!(wall.at(along + da, z + dz)))).collect();
    let lines: serde_json::Map<String, Value> =
        (0..4).map(|i| (format!("l{i}"), json!({"from": format!("p{i}"), "to": format!("p{}", (i + 1) % 4)}))).collect();
    run(&mut s, "add", json!({"name": "rect", "op": "sketch", "inputs": ["l"], "face": wall.id, "sketch": {"points": points, "lines": lines}}));
    let r = run(&mut s, "extrude", json!({"name": "cut", "sketch": "rect", "mode": "cut", "through": true, "wall": true}));
    assert_cut(&r);
    assert_eq!(pieces(&mut s, "cut"), [outline[0] + 4, 0, 0]);
}

/// A hole over a whole short middle wall and both its bends divides the wall's faces in two
/// (D201); the blank has it as one circle.
#[test]
fn a_hole_across_two_bends_is_cut() {
    let mut s = sheet_along("bend_cut_two", &[[0.0, 0.0], [0.0, 60.0], [12.0, 60.0], [12.0, 0.0]]);
    let wall = first_wall(&mut s, "l");
    // The bend's neutral layer: a quarter turn at radius r + k t.
    let bend = std::f64::consts::FRAC_PI_2 * (2.0 + 0.44 * 2.0);
    // Ø24 over the end of the wall, the bend, the middle wall's flat 8, the next bend, and on.
    let r = punch_circle(&mut s, "l", "hole", wall.flat + bend + 4.0, 20.0, 24.0);
    assert_cut(&r);
    assert_eq!(pieces(&mut s, "hole_cut"), [4, 0, 1]);
}

/// Two holes along the same bend line, one after the other.
#[test]
fn a_second_hole_along_the_same_bend() {
    let mut s = l_sheet("bend_cut_second");
    let whole = volume(&mut s, "l");
    let flat = first_wall(&mut s, "l").flat;
    assert_cut(&punch_circle(&mut s, "l", "first", flat - 2.0, 10.0, 6.0));
    let one = volume(&mut s, "first_cut");
    assert_cut(&punch_circle(&mut s, "first_cut", "second", flat - 1.0, 30.0, 6.0));
    let two = volume(&mut s, "second_cut");
    assert!(whole > one && one > two, "{whole} {one} {two}");
    assert!(((whole - one) - (one - two)).abs() < 0.1 * (whole - one), "{whole} {one} {two}");
    assert_eq!(pieces(&mut s, "second_cut")[2], 2);
}

/// A hole across a bend and the sheet's edge is a notch there (D202): cut, the blank's edge broken
/// by its arcs.
#[test]
fn a_hole_across_a_bend_and_an_edge_is_cut() {
    let mut s = l_sheet("bend_cut_edge");
    let flat = first_wall(&mut s, "l").flat;
    let r = punch_circle(&mut s, "l", "hole", flat - 2.0, 3.0, 10.0);
    assert_cut(&r);
    let [lines, arcs, circles] = pieces(&mut s, "hole_cut");
    assert!(lines == 5 && arcs >= 1 && circles == 0, "{lines} {arcs} {circles}");
}

/// A slot from one end of the bend to the other cuts the sheet in two: refused for now (D202).
#[test]
fn a_slot_along_the_whole_bend_is_refused() {
    let mut s = l_sheet("bend_cut_slot");
    let wall = first_wall(&mut s, "l");
    let (a, b) = (wall.at(wall.flat - 3.0, -1.0), wall.at(wall.flat + 3.0, 41.0));
    let c = [wall.at(wall.flat + 3.0, -1.0), wall.at(wall.flat - 3.0, 41.0)];
    let sketch = json!({"points": {"a": a, "b": c[0], "c": b, "d": c[1]},
        "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}, "cd": {"from": "c", "to": "d"}, "da": {"from": "d", "to": "a"}}});
    run(&mut s, "add", json!({"name": "slot", "op": "sketch", "inputs": ["l"], "face": wall.id, "sketch": sketch}));
    let r = run(&mut s, "extrude", json!({"name": "cut", "sketch": "slot", "mode": "cut", "through": true, "wall": true}));
    assert!(failure(&r).contains("two parts"), "{r}");
}

/// A rule without a K-factor: a cut across a bend asks for one.
#[test]
fn a_cut_across_a_bend_needs_a_k_factor() {
    let mut s = project(
        "bend_cut_no_k",
        json!({"points": {"a": [0, 0], "b": [0, 60], "c": [80, 60]}, "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}}}),
    );
    run(
        &mut s,
        "put_library",
        json!({"level": "rule", "element": {"name": "Без K", "kind": "sheet", "material": "Сталь", "thickness": 2, "radius": "t"}}),
    );
    run(&mut s, "sheet", json!({"name": "l", "sketch": "path", "rule": "Без K", "side": "left", "z1": 40}));
    let flat = first_wall(&mut s, "l").flat;
    let r = punch_circle(&mut s, "l", "hole", flat - 2.0, 20.0, 10.0);
    assert!(failure(&r).contains("k_factor"), "{r}");
}
