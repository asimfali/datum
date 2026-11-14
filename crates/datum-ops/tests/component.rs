//! A library part placed in a part (D260, D261): its `cut` role's profile, laid on a wall of the
//! sheet by each placement, punches that wall through as a press punch does (D196). The sheet and
//! its blank take the cut, nothing else does; forming comes next.
#![allow(clippy::unwrap_used, clippy::panic)]

use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::PI;

use datum_brep::{Body, BrepError, Surface, TopoId, cuboid, tessellate};
use datum_math::Vec3;
use datum_math::expr::Num;
use datum_ops::{Op, PartDef, Value, tool, unfold_side};
use serde_json::{Map, Value as Json, json};

/// The plate the stamp cuts: 100 × 60 × 2, its top at z = 2.
const T: f64 = 2.0;
/// The K-factor a blank is laid at: the plate has no bends, so it changes nothing.
const K: f64 = 0.44;

/// The area the stamp's profile takes from the plate: a circle of diameter 10.
fn area() -> f64 {
    PI * 25.0
}

fn id(tag: &str) -> TopoId {
    TopoId::derive(tag).finish()
}

fn op_of(j: Json) -> Op {
    serde_json::from_value(j).unwrap()
}

fn def_of(part: &Json) -> PartDef {
    serde_json::from_value(part.clone()).unwrap()
}

fn params(w: f64) -> BTreeMap<String, Num> {
    BTreeMap::from([("w".to_string(), Num::Value(w))])
}

/// A rectangle of `x` by `y` from the origin, as a sketch.
fn rectangle(x: f64, y: f64) -> Json {
    json!({"points": {"a": [0, 0], "b": [x, 0], "c": [x, y], "d": [0, y]},
        "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"},
                  "cd": {"from": "c", "to": "d"}, "da": {"from": "d", "to": "a"}}})
}

/// A plate 100 × 60 × 2 of sheet metal, its blank to lay flat: the sheet the stamp is placed on.
fn plate() -> Value {
    let s = op_of(json!({"op": "sketch", "sketch": rectangle(100.0, 60.0)})).eval(id("plate.sketch"), &[]).unwrap();
    op_of(json!({"op": "plate", "thickness": T})).eval(id("plate"), &[&s]).unwrap()
}

/// The ID of the plate's top face.
fn top(plate: &Value) -> String {
    let b = plate.body().unwrap();
    let f = b.faces().iter().find(|f| matches!(f.surface, Surface::Plane(p) if p.normal.z > 0.5)).unwrap();
    format!("{:?}", f.id)
}

/// A sketch on the plate's top face, where the placements are: its coordinates are the world's X
/// and Y, its origin under the world origin.
fn on_top(plate: &Value, sketch: Json) -> Value {
    op_of(json!({"op": "sketch", "face": top(plate), "sketch": sketch})).eval(id("placements"), &[plate]).unwrap()
}

/// A stamp (D256): its `cut` role an extrusion of a circle of diameter 10 about its anchor, as
/// tall as the parameter `w`, which a line's length feeds.
fn part() -> Json {
    json!({
        "name": "Язык 20",
        "kind": "stamp",
        "anchor": [0.0, 0.0, 0.0],
        "axis": [1.0, 0.0, 0.0],
        "parameters": [{"name": "w", "value": 10.0}],
        "fit": "w",
        "features": [
            {"name": "profile", "inputs": [],
             "op": {"op": "sketch", "sketch": {"points": {"h": [0, 0]}, "circles": {"c": {"center": "h", "diameter": 10}}}}},
            {"name": "body", "inputs": ["profile"], "op": {"op": "extrude", "z0": 0, "z1": "w"}}
        ],
        "roles": {"cut": "body"}
    })
}

/// [`part`] with the `cut` role's profile sketch and extrusion given, and its `roles`.
fn part_with(profile: Json, body: Json, roles: Json) -> Json {
    let mut p = part();
    let features = p["features"].as_array_mut().unwrap();
    features[0]["op"] = profile;
    features[1]["op"] = body;
    p["roles"] = roles;
    p
}

/// A component of `part` at `placements`, its parameters as given.
fn component(part: Json, placements: Json, params: Json) -> Op {
    op_of(json!({"op": "component", "part": part, "params": params, "placements": placements}))
}

/// Placements at the points `at` of the placement sketch, as its coordinates.
fn points(at: &[[f64; 2]]) -> Json {
    Json::Array(at.iter().map(|p| json!({"placement": "point", "input": 1, "at": p})).collect())
}

/// The component `op` evaluated on `sheet` and the placement sketch `s`.
fn eval(op: &Op, sheet: &Value, s: &Value) -> Result<Body, BrepError> {
    let body = op.eval(id("comp"), &[sheet, s])?.body().cloned().ok_or_else(|| BrepError::Invalid("not a body".into()))?;
    body.validate().unwrap();
    tessellate(&body).unwrap();
    Ok(body)
}

/// How many lines, arcs and circles the plate's blank has.
fn blank_counts(b: &Body) -> [usize; 3] {
    unfold_side(b, false, K).unwrap().counts()
}

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-9 * b.abs().max(1.0)
}

/// The stamp cuts the plate where it is placed: the profile's area through the thickness, the
/// plate's blank a circle more.
#[test]
fn a_stamp_punches_the_wall_where_it_is_placed() {
    let sheet = plate();
    let s = on_top(&sheet, json!({"points": {"p": [50, 30]}}));
    let op = component(part(), json!([{"placement": "point", "input": 1, "point": "p"}]), json!({}));
    let cut = eval(&op, &sheet, &s).unwrap();
    let gone = sheet.body().unwrap().volume() - cut.volume();
    assert!(close(gone, area() * T), "removed {gone}");
    assert_eq!(blank_counts(&cut), [4, 0, 1]);
    // The hole is where the point is: its wall a cylinder about it.
    let hole = cut.faces().iter().find(|f| matches!(f.surface, Surface::Cylinder(_))).unwrap();
    let Surface::Cylinder(c) = hole.surface else { panic!() };
    assert!(close(c.radius, 5.0), "radius {}", c.radius);
    let [x, y, z] = c.origin.to_array();
    assert!(close(x, 50.0) && close(y, 30.0) && close(z, 0.0), "at {:?}", c.origin);
}

/// A line placement turns the tool along the line and feeds its length to `fit`: the profile the
/// tool's parameter builds cuts the hole of that size, at the line's start.
#[test]
fn a_line_feeds_its_length_to_the_profile() {
    let sheet = plate();
    let mut p = part();
    p["features"][0]["op"] = json!({"op": "sketch", "sketch": {"points": {"h": [0, 0]}, "circles": {"c": {"center": "h", "diameter": "w"}}}});
    let s = on_top(&sheet, json!({"points": {"a": [30, 30], "b": [50, 30]}, "lines": {"l": {"from": "a", "to": "b"}}}));
    let op = component(p, json!([{"placement": "line", "input": 1, "line": "l"}]), json!({}));
    let cut = eval(&op, &sheet, &s).unwrap();
    let gone = sheet.body().unwrap().volume() - cut.volume();
    // A circle of diameter 20 through the thickness.
    assert!(close(gone, PI * 100.0 * T), "removed {gone}");
    assert_eq!(blank_counts(&cut), [4, 0, 1]);
}

/// The depth of the `cut` role's extrusion is nothing: the profile goes through the wall.
#[test]
fn the_depth_of_the_cut_role_does_not_matter() {
    let sheet = plate();
    let s = on_top(&sheet, json!({"points": {"p": [50, 30]}}));
    let placements = json!([{"placement": "point", "input": 1, "point": "p"}]);
    let shallow = component(
        part_with(part()["features"][0]["op"].clone(), json!({"op": "extrude", "z0": 0, "z1": 0.01}), json!({"cut": "body"})),
        placements.clone(),
        json!({}),
    );
    let deep = component(
        part_with(part()["features"][0]["op"].clone(), json!({"op": "extrude", "z0": -50, "z1": 50}), json!({"cut": "body"})),
        placements,
        json!({}),
    );
    assert_eq!(eval(&shallow, &sheet, &s).unwrap().hash(), eval(&deep, &sheet, &s).unwrap().hash());
}

/// A stamp stands on the face's outward side: under a plate its anchor plane faces the other way,
/// and nothing cuts. Nor does a wall the tool's anchor plane is not laid on: a bent wall is turned
/// out of it, and a placement that turns the tool to a wall comes with forming.
#[test]
fn a_stamp_under_a_plate_is_refused() {
    let sheet = plate();
    let under = sheet.body().unwrap().faces().iter().find(|f| matches!(f.surface, Surface::Plane(p) if p.normal.z < -0.5)).unwrap();
    let s = op_of(json!({"op": "sketch", "face": format!("{:?}", under.id), "sketch": {"points": {"p": [50, 30]}}}))
        .eval(id("placements"), &[&sheet])
        .unwrap();
    let op = component(part(), json!([{"placement": "point", "input": 1, "point": "p"}]), json!({}));
    let e = op.eval(id("comp"), &[&sheet, &s]).unwrap_err();
    assert!(matches!(&e, BrepError::Invalid(m) if m.contains("not in the plane of its anchor")), "{e}");
}

/// A sheet with ribs is refused: its blank cannot be laid from the ribbed body, so the cut and
/// the ribs laid again come next.
#[test]
fn a_sheet_with_ribs_is_refused() {
    let sheet = plate();
    let path = on_top(&sheet, json!({"points": {"a": [20, 50], "b": [80, 50]}, "lines": {"l": {"from": "a", "to": "b"}}}));
    let rib = op_of(json!({"op": "rib", "tool": "ball", "diameter": 12, "depth": 3, "shoulder": 2})).eval(id("rib"), &[&sheet, &path]).unwrap();
    let s = on_top(&sheet, json!({"points": {"p": [50, 20]}}));
    let op = component(part(), json!([{"placement": "point", "input": 1, "point": "p"}]), json!({}));
    let e = op.eval(id("comp"), &[&rib, &s]).unwrap_err();
    assert!(matches!(&e, BrepError::Unsupported(m) if m.contains("with ribs comes later")), "{e}");
}

/// Every placement cuts a hole of its own, named by its place; the same on a second run.
#[test]
fn each_placement_cuts_its_own_hole() {
    let sheet = plate();
    let s = on_top(&sheet, json!({"points": {"a": [20, 20], "b": [50, 30], "c": [80, 40]}}));
    let placements = Json::Array(["a", "b", "c"].iter().map(|p| json!({"placement": "point", "input": 1, "point": p})).collect());
    let op = component(part(), placements, json!({}));
    let cut = eval(&op, &sheet, &s).unwrap();
    let gone = sheet.body().unwrap().volume() - cut.volume();
    assert!(close(gone, 3.0 * area() * T), "removed {gone}");
    assert_eq!(blank_counts(&cut), [4, 0, 3]);
    assert_eq!(cut.hash(), eval(&op, &sheet, &s).unwrap().hash());
    // A placement's hole is an element of its own: no two faces share an ID.
    let ids: BTreeSet<TopoId> = cut.faces().iter().map(|f| f.id).collect();
    assert_eq!(ids.len(), cut.faces().len());
}

/// The tool keeps the sketch its `cut` role extrudes: the profile is the sketch's, whatever the
/// body's height, and the body is built as the parameters say.
#[test]
fn the_tool_keeps_the_sketch_its_cut_role_extrudes() {
    let def = def_of(&part());
    let made = |w: f64| tool(&def, id("comp"), &params(w)).unwrap();
    assert_eq!(made(10.0).cut_sketch.unwrap().hash, made(20.0).cut_sketch.unwrap().hash);
    assert!(close(made(10.0).body.volume(), area() * 10.0), "volume {}", made(10.0).body.volume());
    assert_ne!(made(10.0).body.hash(), made(20.0).body.hash());
    assert!(close(tool(&def, id("comp"), &BTreeMap::new()).unwrap().body.volume(), area() * 10.0));
}

/// What a component refuses: a `cut` role that is no extrusion of a sketch, a part that forms or
/// hinges the sheet, a profile off the wall's plane, a placement off a face of the sheet.
#[test]
fn what_a_component_refuses() {
    let sheet = plate();
    let s = on_top(&sheet, json!({"points": {"p": [50, 30]}}));
    let one = json!([{"placement": "point", "input": 1, "point": "p"}]);
    let fail = |op: &Op| eval(op, &sheet, &s).unwrap_err();
    let says = |m: &str, e: &BrepError| assert!(e.to_string().contains(m), "{e}");
    // The `cut` role is an extrusion of a sketch.
    let mut prism = part_with(
        part()["features"][0]["op"].clone(),
        json!({"op": "prism", "polygon": [[0, 0], [1, 0], [1, 1]], "z0": 0, "z1": 1}),
        json!({"cut": "body"}),
    );
    prism["features"][1]["inputs"] = json!([]);
    says("is not an extrusion of a sketch", &fail(&component(prism, one.clone(), json!({}))));
    // Nor is a `cut` role whose extrusion takes a body first: a tool with a hole in it.
    let mut body_first = part();
    body_first["features"] = json!([
        {"name": "profile", "inputs": [], "op": {"op": "sketch", "sketch": rectangle(10.0, 10.0)}},
        {"name": "small", "inputs": [], "op": {"op": "sketch", "sketch": {"points": {"a": [3, 3], "b": [7, 3], "c": [7, 7], "d": [3, 7]},
            "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}, "cd": {"from": "c", "to": "d"}, "da": {"from": "d", "to": "a"}}}}},
        {"name": "body", "inputs": ["profile"], "op": {"op": "extrude", "z0": 0, "z1": 10}},
        {"name": "hole", "inputs": ["body", "small"], "op": {"op": "extrude", "mode": "cut", "z0": -1, "z1": 11}}
    ]);
    body_first["roles"] = json!({"cut": "hole"});
    says("is not an extrusion of a sketch", &fail(&component(body_first, one.clone(), json!({}))));
    // A part that cuts and forms, or hinges, comes next.
    says(
        "a cut agreed with the die comes later",
        &fail(&component(
            part_with(part()["features"][0]["op"].clone(), json!({"op": "extrude", "z0": 0, "z1": 1}), json!({"cut": "body", "form": "body"})),
            one.clone(),
            json!({}),
        )),
    );
    says(
        "forming comes next",
        &fail(&component(
            part_with(part()["features"][0]["op"].clone(), json!({"op": "extrude", "z0": 0, "z1": 1}), json!({"cut": "body", "hinge": "body"})),
            one.clone(),
            json!({}),
        )),
    );
    // The profile lies in the anchor plane: off it, or turned out of it, nothing cuts.
    let off = json!({"op": "sketch", "plane": {"origin": [0, 0, 5], "x_axis": [1, 0, 0], "normal": [0, 0, 1]},
                     "sketch": {"points": {"h": [0, 0]}, "circles": {"c": {"center": "h", "diameter": 10}}}});
    says(
        "not in the plane of its anchor",
        &fail(&component(part_with(off, json!({"op": "extrude", "z0": 0, "z1": 1}), json!({"cut": "body"})), one.clone(), json!({}))),
    );
    let turned = json!({"op": "sketch", "plane": {"origin": [0, 0, 0], "x_axis": [0, 1, 0], "normal": [1, 0, 0]},
                        "sketch": {"points": {"h": [0, 0]}, "circles": {"c": {"center": "h", "diameter": 10}}}});
    says(
        "not in the plane of its anchor",
        &fail(&component(part_with(turned, json!({"op": "extrude", "z0": 0, "z1": 1}), json!({"cut": "body"})), one.clone(), json!({}))),
    );
    // A placement is a point or a line of a sketch on a face of the sheet.
    let flat = op_of(json!({"op": "sketch", "sketch": {"points": {"p": [50, 30]}}})).eval(id("flat"), &[]).unwrap();
    let op = component(part(), one.clone(), json!({}));
    says("takes a sketch placed on a face", &op.eval(id("comp"), &[&sheet, &flat]).unwrap_err());
    // A face the sheet has not: the placement sketch was built on another sheet.
    let other = Value::Body(cuboid(id("other"), Vec3::ZERO, Vec3::new(10.0, 10.0, 1.0)).unwrap());
    assert!(matches!(op.eval(id("comp"), &[&other, &s]), Err(BrepError::LostReference(_))));
    // No placements at all, and a line or a point the sketch has not (D260, task A).
    says("`placements` is empty", &fail(&component(part(), json!([]), json!({}))));
    let line = fail(&component(part(), json!([{"placement": "line", "input": 1, "line": "zzz"}]), json!({})));
    assert!(matches!(&line, BrepError::LostReference(m) if m.contains("no line `zzz`")), "{line}");
    let point = fail(&component(part(), json!([{"placement": "point", "input": 1, "point": "zzz"}]), json!({})));
    assert!(matches!(&point, BrepError::LostReference(m) if m.contains("no point `zzz`")), "{point}");
    // A parameter the part has not, and a line placement of a part without an `axis`.
    says("no parameter `nope`", &fail(&component(part(), one.clone(), json!({"nope": 3.0}))));
    let mut no_axis = part();
    no_axis.as_object_mut().unwrap().remove("axis");
    let along = on_top(&sheet, json!({"points": {"a": [10, 10], "b": [40, 10]}, "lines": {"ab": {"from": "a", "to": "b"}}}));
    let placed = component(no_axis.clone(), json!([{"placement": "line", "input": 1, "line": "ab"}]), json!({}));
    says("has no `axis`", &eval(&placed, &sheet, &along).unwrap_err());
    assert!(def_of(&no_axis).validate().unwrap_err().contains("needs an `axis`"));
}

proptest::proptest! {
    #![proptest_config(proptest::prelude::ProptestConfig::with_cases(16))]

    /// `k` placements cut `k` holes, each its own, and the same on a second run.
    #[test]
    fn every_placement_cuts_a_hole_of_its_own(k in 1usize..=6) {
        let sheet = plate();
        let at: Vec<[f64; 2]> = (0..k).map(|i| [15.0 + 10.0 * i as f64, 20.0 + 5.0 * i as f64]).collect();
        let mut named = Map::new();
        for (i, p) in at.iter().enumerate() {
            named.insert(format!("p{i}"), json!(p));
        }
        let s = on_top(&sheet, json!({"points": named}));
        let placements = points(&at);
        let op = component(part(), placements, json!({}));
        let cut = eval(&op, &sheet, &s).unwrap();
        let gone = sheet.body().unwrap().volume() - cut.volume();
        proptest::prop_assert!(close(gone, k as f64 * area() * T), "removed {gone}");
        proptest::prop_assert_eq!(blank_counts(&cut), [4, 0, k]);
        proptest::prop_assert_eq!(cut.hash(), eval(&op, &sheet, &s).unwrap().hash());
    }
}
