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

// ---------------------------------------------------------------------------------------------
// Forming: a louver along a line (D261).

/// The die of the louver (the probe's matrix): an arc dipped into the sheet, from (−4, 0) to
/// (4, 0) about `(0, CY)`, its body extruded; the bend radius `R` and the line `L` long.
const CY: f64 = 6.066_666_666_666_667;
const R: f64 = 3.0;
const L: f64 = 50.0;

/// The die's arc radius.
fn rho() -> f64 {
    libm::sqrt(16.0 + CY * CY)
}

/// Half the louver's width: a circle of `R` about `(w, −R)` touches the die's arc `R + rho` from
/// its centre, and the sheet's face from below.
fn w() -> f64 {
    let reach = R + rho();
    libm::sqrt(reach * reach - (CY + R) * (CY + R))
}

/// Where the transition touches the die's arc: `R` from its centre along the centres' line.
fn touch() -> [f64; 2] {
    let (w, reach) = (w(), R + rho());
    [w - R * w / reach, -R + R * (CY + R) / reach]
}

/// The length of the working curve between the transitions: two arcs of `R` and the die's arc's
/// rest.
fn curve_length() -> f64 {
    let t = touch();
    let alpha = libm::acos((t[1] + R) / R);
    let beta = (libm::atan2(t[1] - CY, t[0]) - libm::atan2(t[1] - CY, -t[0])).abs();
    2.0 * R * alpha + rho() * beta
}

/// The area of the forming zone on the sheet: the stadium `2w` wide and `L` long.
fn zone_area() -> f64 {
    let (w, l) = (w(), L);
    2.0 * w * (l - 2.0 * w) + std::f64::consts::PI * w * w
}

/// The tongue's dome at one end, `π∫_H x dA`: `H` the band's half between the working curve and
/// `y = 0`, from the louver's axis to the wall, its boundary integral `∮ x²/2 dy` sampled on the
/// curve built from the die's own numbers. The fillet's offset band below is laid along the
/// segments' normals, a chord's construction: a trapezoid off the exact circular offset by
/// O(Δθ²) ≈ 2·10⁻¹⁰ relative — the `close` (1e-9) holds with that much room, no tighter.
fn dome_volume(t: f64) -> f64 {
    let (r, w, n) = (rho(), w(), 20_000);
    let touch = touch();
    let mut curve: Vec<[f64; 2]> = Vec::with_capacity(2 * n + 1);
    let (a0, a1) = (-std::f64::consts::FRAC_PI_2, libm::atan2(touch[1] - CY, touch[0]));
    for k in 0..=n {
        let a = a0 + (a1 - a0) * k as f64 / n as f64;
        curve.push([r * libm::cos(a), CY + r * libm::sin(a)]);
    }
    let (b0, b1) = (libm::atan2(touch[1] + R, touch[0] - w), std::f64::consts::FRAC_PI_2);
    for k in 1..=n {
        let a = b0 + (b1 - b0) * k as f64 / n as f64;
        curve.push([w + R * libm::cos(a), -R + R * libm::sin(a)]);
    }
    let mut area = 0.0;
    let term = |area: &mut f64, a: [f64; 2], b: [f64; 2]| *area += (a[0] * a[0] + a[0] * b[0] + b[0] * b[0]) / 6.0 * (b[1] - a[1]);
    for s in curve.windows(2) {
        term(&mut area, s[0], s[1]);
    }
    // Down the wall's face, back along the band's offset, and up the axis (x = 0 adds nothing).
    let top = curve[curve.len() - 1];
    term(&mut area, top, [top[0], top[1] - t]);
    let last = curve.len() - 1;
    let mut back: Vec<[f64; 2]> = Vec::with_capacity(curve.len());
    for k in (0..=last).rev() {
        let p = curve[k];
        // The band's offset: to the right of the way the curve runs, outwards.
        let (from, to) = if k < last { (p, curve[k + 1]) } else { (curve[k - 1], p) };
        let (dx, dy) = (to[0] - from[0], to[1] - from[1]);
        let l = libm::hypot(dx, dy);
        back.push([p[0] + t * dy / l, p[1] - t * dx / l]);
    }
    for s in back.windows(2) {
        term(&mut area, s[0], s[1]);
    }
    let end = back[back.len() - 1];
    term(&mut area, end, [end[0], end[1] + t]);
    std::f64::consts::PI * area.abs()
}

/// The part of the louver with `sketch` as its die: form-only, its `thickness` range.
fn louver_part_with(sketch: Json) -> Json {
    json!({
        "name": "Жалюзи", "kind": "stamp",
        "anchor": [0.0, 0.0, 0.0], "axis": [1.0, 0.0, 0.0],
        "thickness": [0.5, 3.0],
        "parameters": [],
        "features": [
            {"name": "die", "inputs": [], "op": {"op": "sketch", "plane": {"origin": [0, 0, 0], "x_axis": [0, 1, 0], "normal": [1, 0, 0]},
                "sketch": sketch}},
            {"name": "body", "inputs": ["die"], "op": {"op": "extrude", "z0": 0.0, "z1": 6.0}}
        ],
        "roles": {"form": "body"}
    })
}

/// The part of the louver: form-only, its `thickness` range and its die.
fn louver_part() -> Json {
    louver_part_with(json!({
        "points": {"a": [-4.0, 0.0], "b": [0.0, -1.2], "c": [4.0, 0.0], "m": [0.0, CY], "e": [4.0, 0.5], "f": [-4.0, 0.5]},
        "arcs": {"dip": {"center": "m", "from": "a", "to": "c"}},
        "lines": {"right": {"from": "c", "to": "e"}, "back": {"from": "e", "to": "f"}, "left": {"from": "f", "to": "a"}},
        "constraints": [{"type": "on", "curve": "dip", "point": "b"}]}))
}

/// A louver of `part` at `placements`, the rule's thickness and radius put on the op.
fn louver_of(part: Json, placements: Json, thickness: f64, radius: f64) -> Op {
    op_of(json!({
        "op": "component", "part": part, "params": {}, "placements": placements,
        "thickness": thickness, "radius": radius
    }))
}

/// A louver of [`louver_part`] at `placements`.
fn louver(placements: Json, thickness: f64, radius: f64) -> Op {
    louver_of(louver_part(), placements, thickness, radius)
}

/// The line from (30, 30) to (30 + `l`, 30) on the plate's top, and the op formed along it.
fn along(l: f64) -> (Value, Value, Op) {
    let sheet = plate();
    let s = on_top(&sheet, json!({"points": {"a": [30, 30], "b": [30.0 + l, 30]}, "lines": {"l": {"from": "a", "to": "b"}}}));
    let op = louver(json!([{"placement": "line", "input": 1, "line": "l"}]), T, R);
    (sheet, s, op)
}

#[test]
fn a_louver_forms_the_sheet_along_the_line() {
    let (sheet, s, op) = along(L);
    let formed = match op.eval(id("comp"), &[&sheet, &s]).unwrap() {
        Value::Formed(f) => f,
        other => panic!("not formed: {other:?}"),
    };
    let body = &formed.body;
    body.validate().unwrap();
    // The sheet less the zone, and the tongue: the band swept between the domes, and the domes.
    let tongue = T * curve_length() * (L - 2.0 * w()) + 2.0 * dome_volume(T);
    let expect = sheet.body().unwrap().volume() - zone_area() * T + tongue;
    assert!(close(body.volume(), expect), "volume {} expected {expect}", body.volume());
    // The middle is the section swept: lines to planes, arcs to cylinders; the ends turn to tori,
    // a piece about the louver's axis to a sphere.
    let middle = body.faces().iter().filter(|f| matches!(f.surface, Surface::Cylinder(_))).count();
    // The middle sweeps both curves of the section (4 pieces each); the domes turn both halves
    // of both curves at both ends, two quarters a piece; the slot's ends round: the tongue's rim
    // a quarter at a time, the hole's wall a half at once: 14 cylinders, 16 tori and spheres.
    assert_eq!(middle, 14, "cylinders {middle}");
    let turned = body.faces().iter().filter(|f| matches!(f.surface, Surface::Torus(_))).count();
    assert_eq!(turned, 16, "tori {turned}");
    // The tongue keeps the sheet's thickness: the axis point of the working curve and its offset.
    let on_axis = |v: &datum_brep::Vertex| close(v.point.x, 30.0 + w()) && close(v.point.y, 30.0);
    let (upper, lower) = body.vertices().iter().filter(|v| on_axis(v)).fold((None, None), |(hi, lo): (Option<f64>, Option<f64>), v| {
        (Some(hi.map_or(v.point.z, |h| h.max(v.point.z))), Some(lo.map_or(v.point.z, |l| l.min(v.point.z))))
    });
    let (Some(upper), Some(lower)) = (upper, lower) else { panic!("no axis vertices") };
    assert!(close(upper - lower, T), "thickness {} ", upper - lower);
    // The blank is the sheet less the zone, and its DXF is the outline and the zone's contour.
    assert!(close(formed.blank.volume(), 12000.0 - zone_area() * T), "blank {}", formed.blank.volume());
    assert_eq!(blank_counts(&formed.blank), [6, 2, 0]);
    // The same on a second run.
    let again = match op.eval(id("comp"), &[&sheet, &s]).unwrap() {
        Value::Formed(f) => f,
        other => panic!("not formed: {other:?}"),
    };
    assert_eq!(body.hash(), again.body.hash());
}

#[test]
fn each_line_forms_a_louver_of_its_own() {
    let sheet = plate();
    let lines = json!({"points": {"a": [20, 20], "b": [70, 20], "c": [20, 40], "d": [70, 40]},
        "lines": {"one": {"from": "a", "to": "b"}, "two": {"from": "c", "to": "d"}}});
    let s = on_top(&sheet, lines);
    let placements = json!([{"placement": "line", "input": 1, "line": "one"}, {"placement": "line", "input": 1, "line": "two"}]);
    let formed = match louver(placements, T, R).eval(id("comp"), &[&sheet, &s]).unwrap() {
        Value::Formed(f) => f,
        other => panic!("not formed: {other:?}"),
    };
    let tongue = T * curve_length() * (L - 2.0 * w()) + 2.0 * dome_volume(T);
    let expect = 12000.0 - 2.0 * zone_area() * T + 2.0 * tongue;
    assert!(close(formed.body.volume(), expect), "volume {} expected {expect}", formed.body.volume());
    assert_eq!(blank_counts(&formed.blank), [4 + 2 * 2, 2 * 2, 0]);
    assert!(close(formed.blank.volume(), 12000.0 - 2.0 * zone_area() * T), "blank {}", formed.blank.volume());
}

#[test]
fn what_a_formed_component_refuses() {
    let (sheet, s, _) = along(L);
    let one = json!([{"placement": "line", "input": 1, "line": "l"}]);
    let fail = |op: &Op| op.eval(id("comp"), &[&sheet, &s]).unwrap_err();
    let says = |m: &str, e: &BrepError| assert!(e.to_string().contains(m), "{e}");
    let with = |part: Json| louver_of(part, one.clone(), T, R);
    // A die that does not press into the sheet, and one too shallow for the transition, are
    // refused by `die_of` (its working curve must stand below the chord between its widest
    // points); a flat outline is refused there first.
    let flat = louver_part_with(json!({"points": {"a": [-4, 0], "b": [4, 0], "e": [4, 0.5], "f": [-4, 0.5]},
        "lines": {"ab": {"from": "a", "to": "b"}, "right": {"from": "b", "to": "e"}, "back": {"from": "e", "to": "f"}, "left": {"from": "f", "to": "a"}}}));
    says("no side below the chord", &fail(&with(flat)));
    // A sheet thicker than the part takes, and a bend radius no wider than it.
    says("outside its `thickness` range", &fail(&louver(one.clone(), 5.0, R)));
    says("the tongue's inner transition has no radius", &fail(&louver(one.clone(), T, T)));
    // No sheet rule: the thickness and radius come from the part's.
    let no_rule = op_of(json!({"op": "component", "part": louver_part(), "params": {}, "placements": one}));
    says("from the part's sheet rule", &fail(&no_rule));
    // A part that cuts too, and forming at a point.
    let mut both = louver_part();
    both["roles"] = json!({"cut": "body", "form": "body"});
    says("a cut agreed with the die comes later", &fail(&with(both)));
    let at = on_top(&sheet, json!({"points": {"p": [50, 30]}}));
    says("forms at a point", &louver(json!([{"placement": "point", "input": 1, "point": "p"}]), T, R).eval(id("comp"), &[&sheet, &at]).unwrap_err());
    // A louver running to the sheet's edge: its zone would cross it.
    let edge = on_top(&sheet, json!({"points": {"a": [30, 58], "b": [70, 58]}, "lines": {"l": {"from": "a", "to": "b"}}}));
    says(
        "does not stay inside its wall",
        &louver(json!([{"placement": "line", "input": 1, "line": "l"}]), T, R).eval(id("comp"), &[&sheet, &edge]).unwrap_err(),
    );
}

proptest::proptest! {
    #![proptest_config(proptest::prelude::ProptestConfig::with_cases(6))]

    /// `k` lines form `k` louvers, each its own, and the same on a second run.
    #[test]
    fn every_line_forms_a_louver_of_its_own(k in 1usize..=3) {
        let sheet = plate();
        let mut named = Map::new();
        let mut lines = Map::new();
        for i in 0..k {
            let y = 15.0 + 15.0 * i as f64;
            named.insert(format!("a{i}"), json!([20, y]));
            named.insert(format!("b{i}"), json!([70, y]));
            lines.insert(format!("l{i}"), json!({"from": format!("a{i}"), "to": format!("b{i}")}));
        }
        let s = on_top(&sheet, json!({"points": named, "lines": lines}));
        let placements = Json::Array((0..k).map(|i| json!({"placement": "line", "input": 1, "line": format!("l{i}")})).collect());
        let op = louver(placements, T, R);
        let formed = match op.eval(id("comp"), &[&sheet, &s]).unwrap() {
            Value::Formed(f) => f,
            other => panic!("not formed: {other:?}"),
        };
        let tongue = T * curve_length() * (L - 2.0 * w()) + 2.0 * dome_volume(T);
        let expect = 12000.0 - k as f64 * zone_area() * T + k as f64 * tongue;
        proptest::prop_assert!(close(formed.body.volume(), expect), "volume {} expected {expect}", formed.body.volume());
        proptest::prop_assert_eq!(blank_counts(&formed.blank), [4 + k * 2, k * 2, 0]);
        let again = match op.eval(id("comp"), &[&sheet, &s]).unwrap() {
            Value::Formed(f) => f,
            _ => panic!("not formed"),
        };
        proptest::prop_assert_eq!(formed.body.hash(), again.body.hash());
    }
}
