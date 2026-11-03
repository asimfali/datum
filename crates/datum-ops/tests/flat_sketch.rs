//! A sketch on the flat blank of a sheet (D222): its coordinates are the blank's, so a cut from it
//! lands on the blank where it was drawn, across bend lines too; the blank's outline and bend
//! lines project into it by stable names.
#![allow(clippy::unwrap_used, clippy::panic)]

use std::collections::BTreeSet;

use datum_brep::{Body, BrepError, TopoId};
use datum_ops::{Flat, FlatPiece, Op, Placed, Value, unfold_side};

const K: f64 = 0.44;
/// Thickness, bend radius and width of the sheet.
const T: f64 = 2.0;
const WIDTH: f64 = 40.0;
/// A hole narrower than a bend, so that it can lie across one bend line.
const DIAMETER: f64 = 4.0;

fn id(tag: &str) -> TopoId {
    TopoId::derive(tag).finish()
}

fn op(json: serde_json::Value) -> Op {
    serde_json::from_value(json).unwrap()
}

/// An L of sheet metal: up 60 from the origin, then `length` along X, on the left of its path.
fn sheet_of(length: f64) -> Body {
    let path = serde_json::json!({"points": {"a": [0, 0], "b": [0, 60], "c": [length, 60]},
        "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}}});
    let p = op(serde_json::json!({"op": "sketch", "sketch": path})).eval(id("path"), &[]).unwrap();
    let s =
        op(serde_json::json!({"op": "sheet", "thickness": T, "radius": T, "side": "left", "z0": 0, "z1": WIDTH})).eval(id("sheet"), &[&p]).unwrap();
    s.body().unwrap().clone()
}

/// The sketch on the blank of `b` seen from `side`, from the JSON `fields` of the operation.
fn on_blank(b: &Body, side: &str, fields: serde_json::Value) -> Result<Placed, BrepError> {
    let mut json = serde_json::json!({"op": "sketch", "flat": side, "k_factor": K, "sketch": {}});
    json.as_object_mut().unwrap().extend(fields.as_object().unwrap().clone());
    let Value::Sketch(placed) = op(json).eval(id("flat"), &[&Value::Body(b.clone())])? else { panic!("a sketch") };
    Ok(*placed)
}

fn circle_at(x: f64, y: f64, diameter: f64) -> serde_json::Value {
    serde_json::json!({"sketch": {"points": {"h": [x, y]}, "circles": {"c": {"center": "h", "diameter": diameter}}}})
}

/// The circles of the blank of `b` seen from `side`: centre and radius.
fn circles(b: &Body, back: bool) -> Vec<(f64, f64, f64)> {
    let flat = unfold_side(b, back, K).unwrap();
    flat.pieces.iter().filter_map(|p| if let FlatPiece::Circle { center, radius } = *p { Some((center.x, center.y, radius)) } else { None }).collect()
}

/// The cut of `b` through the sheet by a circle drawn on its blank seen from `side`.
fn cut_at(b: &Body, side: &str, x: f64, y: f64) -> Result<Body, BrepError> {
    let target = Value::Body(b.clone());
    let placed = on_blank(b, side, circle_at(x, y, DIAMETER))?;
    let cut = op(serde_json::json!({"op": "extrude", "mode": "cut", "through": true, "wall": true}));
    let r = cut.eval(id("cut"), &[&target, &Value::Sketch(Box::new(placed))])?.body().unwrap().clone();
    r.validate().unwrap();
    Ok(r)
}

fn lands_where_drawn(side: &str) {
    // The blank is laid from the larger wall: the first of a short sheet, the second of a long one.
    for length in [40.0, 80.0] {
        let b = sheet_of(length);
        let back = side == "back";
        let flat = unfold_side(&b, back, K).unwrap();
        // On the first wall, and across the bend line.
        for (x, y) in [(20.0, 20.0), (flat.bends[0][0].x + 1.3, 25.0)] {
            let cut = cut_at(&b, side, x, y).unwrap_or_else(|e| panic!("{length} ({x}, {y}): {e}"));
            assert!(cut.volume() < b.volume(), "{length} ({x}, {y})");
            let [(cx, cy, r)] = circles(&cut, back)[..] else { panic!("one circle: {:?}", circles(&cut, back)) };
            assert!(
                (cx - x).abs() < 1e-9 && (cy - y).abs() < 1e-9 && (r - 0.5 * DIAMETER).abs() < 1e-9,
                "{length}: ({x}, {y}) -> ({cx}, {cy}), r = {r}"
            );
        }
    }
}

#[test]
fn a_cut_drawn_on_the_blank_lands_where_drawn() {
    lands_where_drawn("front");
}

#[test]
fn a_cut_drawn_on_the_back_blank_lands_where_drawn() {
    lands_where_drawn("back");
}

/// The 8-digit name of an ID in a projection, as `part_name` makes it.
fn short(id: TopoId) -> String {
    format!("{id:?}")[..8].to_string()
}

fn project_blank() -> serde_json::Value {
    serde_json::json!({"project": {"blank": {"blank": "outline"}, "bends": {"blank": "bends"}}})
}

fn ends(placed: &Placed, from: &str, to: &str) -> [(f64, f64); 2] {
    let p = |n: &str| (placed.solved.points[n].x, placed.solved.points[n].y);
    [p(from), p(to)]
}

fn near(a: (f64, f64), b: (f64, f64)) -> bool {
    (a.0 - b.0).abs() < 1e-9 && (a.1 - b.1).abs() < 1e-9
}

fn outline_names(flat: &Flat) -> BTreeSet<String> {
    flat.outer.clone().map(|i| format!("blank.{}", short(flat.ids[i]))).collect()
}

#[test]
fn the_outline_and_bend_lines_project_by_name() {
    let b = sheet_of(80.0);
    let flat = unfold_side(&b, false, K).unwrap();
    assert!(!flat.outer.is_empty() && !flat.bends.is_empty());
    let placed = on_blank(&b, "front", project_blank()).unwrap();
    for i in flat.outer.clone() {
        let name = format!("blank.{}", short(flat.ids[i]));
        match flat.pieces[i] {
            FlatPiece::Line(a, c) => {
                let l = &placed.lines[&name];
                assert!(l.construction);
                let got = ends(&placed, &l.from, &l.to);
                assert!((near(got[0], (a.x, a.y)) && near(got[1], (c.x, c.y))), "{name}: {got:?} {a:?} {c:?}");
            }
            FlatPiece::Arc { .. } => assert!(placed.arcs.contains_key(&name), "{name}"),
            FlatPiece::Circle { .. } => assert!(placed.circles.contains_key(&name), "{name}"),
            FlatPiece::Polyline(_) => assert!(placed.solved.points.contains_key(&format!("{name}.start")), "{name}"),
        }
    }
    for (bend, id) in flat.bends.iter().zip(&flat.bend_ids) {
        let l = &placed.lines[&format!("bends.{}", short(*id))];
        let got = ends(&placed, &l.from, &l.to);
        assert!(near(got[0], (bend[0].x, bend[0].y)) && near(got[1], (bend[1].x, bend[1].y)), "{got:?} {bend:?}");
    }
    // Consecutive pieces of the outline share their corner: the lines make a closed profile.
    let lines: Vec<_> = placed.lines.iter().filter(|(n, _)| n.starts_with("blank.")).collect();
    let corners: BTreeSet<&str> = lines.iter().flat_map(|(_, l)| [l.from.as_str(), l.to.as_str()]).collect();
    assert!(corners.len() <= lines.len(), "{} corners of {} lines", corners.len(), lines.len());

    // A dimension from a point to a bend line.
    let bend = format!("bends.{}.start", short(flat.bend_ids[0]));
    let at = flat.bends[0][0];
    let mut fields = project_blank();
    fields["sketch"] = serde_json::json!({
        "points": {"p": [at.x + 3.0, 10.0]},
        "constraints": [{"type": "horizontal_distance", "points": [bend, "p"], "value": 7}]});
    let placed = on_blank(&b, "front", fields).unwrap();
    assert!((placed.solved.points["p"].x - (at.x + 7.0)).abs() < 1e-9, "{:?}", placed.solved.points["p"]);

    // One piece by its ID.
    let one = format!("{:?}", flat.ids[flat.outer.start]);
    let placed = on_blank(&b, "front", serde_json::json!({"project": {"edge": {"blank": one}}})).unwrap();
    assert!(placed.lines.contains_key("edge") || placed.arcs.contains_key("edge") || placed.circles.contains_key("edge"));
}

#[test]
fn a_sketch_on_the_blank_refuses_what_does_not_fit() {
    let b = sheet_of(80.0);
    let wall = format!("{:?}", b.faces()[0].id);
    assert!(on_blank(&b, "front", serde_json::json!({"face": wall})).is_err());
    let target = Value::Body(b.clone());
    let no_k = op(serde_json::json!({"op": "sketch", "flat": "front", "sketch": {}}));
    assert!(no_k.eval(id("flat"), &[&target]).is_err());
    let plain = op(serde_json::json!({"op": "sketch", "project": {"x": {"blank": "outline"}}, "sketch": {}}));
    assert!(plain.eval(id("plain"), &[]).is_err());
}

#[test]
fn outline_names_keep_when_the_sheet_gets_longer() {
    let names = |length: f64| outline_names(&unfold_side(&sheet_of(length), false, K).unwrap());
    let (short, long) = (names(80.0), names(120.0));
    assert_eq!(short, long);
    let placed = on_blank(&sheet_of(120.0), "front", project_blank()).unwrap();
    for n in &short {
        assert!(placed.lines.contains_key(n) || placed.arcs.contains_key(n), "{n}");
    }
}

/// A plate 100 × 60 with corners rounded 10.
fn rounded_plate() -> Body {
    let outline = serde_json::json!({
        "points": {"a": [10, 0], "b": [90, 0], "c": [100, 10], "d": [100, 50], "e": [90, 60], "f": [10, 60], "g": [0, 50], "h": [0, 10],
                   "c1": [90, 10], "c2": [90, 50], "c3": [10, 50], "c4": [10, 10]},
        "lines": {"ab": {"from": "a", "to": "b"}, "cd": {"from": "c", "to": "d"}, "ef": {"from": "e", "to": "f"}, "gh": {"from": "g", "to": "h"}},
        "arcs": {"r1": {"center": "c1", "from": "b", "to": "c"}, "r2": {"center": "c2", "from": "d", "to": "e"},
                 "r3": {"center": "c3", "from": "f", "to": "g"}, "r4": {"center": "c4", "from": "h", "to": "a"}}});
    let s = op(serde_json::json!({"op": "sketch", "sketch": outline})).eval(id("outline"), &[]).unwrap();
    op(serde_json::json!({"op": "plate", "thickness": T})).eval(id("plate"), &[&s]).unwrap().body().unwrap().clone()
}

/// The outline's arcs meet its lines at the very points the lines end at: projected, the outline
/// is one closed loop of shared points, a profile to dimension from (D222).
#[test]
fn an_outline_with_rounded_corners_projects_closed() {
    let placed = on_blank(&rounded_plate(), "front", project_blank()).unwrap();
    let blank = |n: &&String| n.starts_with("blank.");
    let line_ends: BTreeSet<&String> = placed.lines.iter().filter(|(n, _)| blank(n)).flat_map(|(_, l)| [&l.from, &l.to]).collect();
    let arcs: Vec<_> = placed.arcs.iter().filter(|(n, _)| blank(n)).collect();
    assert_eq!((placed.lines.keys().filter(blank).count(), arcs.len()), (4, 4));
    for (n, a) in arcs {
        assert!(line_ends.contains(&a.from) && line_ends.contains(&a.to), "{n}: {} {} not among {line_ends:?}", a.from, a.to);
        assert!(placed.arcs[n].construction);
    }
}

/// A plate 100 × 60, its corner at the origin.
fn plate() -> Body {
    let outline = serde_json::json!({"points": {"a": [0, 0], "b": [100, 0], "c": [100, 60], "d": [0, 60]},
        "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}, "cd": {"from": "c", "to": "d"}, "da": {"from": "d", "to": "a"}}});
    let s = op(serde_json::json!({"op": "sketch", "sketch": outline})).eval(id("outline"), &[]).unwrap();
    op(serde_json::json!({"op": "plate", "thickness": T})).eval(id("plate"), &[&s]).unwrap().body().unwrap().clone()
}

/// A closed profile of points `pts` in order, lines between them, but for an arc about `arc` (centre)
/// from point `k` to the next where given.
fn profile(pts: &[[f64; 2]], arc: Option<(usize, [f64; 2])>) -> serde_json::Value {
    let mut points = serde_json::Map::new();
    let (mut lines, mut arcs) = (serde_json::Map::new(), serde_json::Map::new());
    for (k, p) in pts.iter().enumerate() {
        points.insert(format!("p{k}"), serde_json::json!(p));
        let next = format!("p{}", (k + 1) % pts.len());
        match arc {
            Some((j, c)) if j == k => {
                points.insert("c".into(), serde_json::json!(c));
                arcs.insert(format!("a{k}"), serde_json::json!({"center": "c", "from": format!("p{k}"), "to": next}));
            }
            _ => {
                lines.insert(format!("l{k}"), serde_json::json!({"from": format!("p{k}"), "to": next}));
            }
        }
    }
    serde_json::json!({"sketch": {"points": points, "lines": lines, "arcs": arcs}})
}

/// `b` with `sketch` (fields of a sketch on its front blank) added to it (`join`) or cut from it.
fn on_sheet(b: &Body, sketch: serde_json::Value, mode: &str) -> Result<Body, BrepError> {
    let placed = on_blank(b, "front", sketch)?;
    let op = op(serde_json::json!({"op": "extrude", "mode": mode, "through": true, "wall": true}));
    let r = op.eval(id("edit"), &[&Value::Body(b.clone()), &Value::Sketch(Box::new(placed))])?.body().unwrap().clone();
    r.validate().unwrap();
    Ok(r)
}

/// A tab drawn on the blank from its edge (D222): the wall goes on in its plane by just the tab,
/// whether its side lies on the edge or reaches into the metal; a rounded end too; the blank
/// grows by it.
#[test]
fn a_tab_drawn_on_the_blank_adds_metal() {
    let b = plate();
    let square = 20.0 * 20.0 * T;
    for base in [50.0, 60.0] {
        let tab = on_sheet(&b, profile(&[[30.0, base], [50.0, base], [50.0, 80.0], [30.0, 80.0]], None), "join").unwrap();
        assert!((tab.volume() - b.volume() - square).abs() < 1e-6, "base at {base}: {}", tab.volume() - b.volume());
        let flat = unfold_side(&tab, false, K).unwrap();
        assert!((flat.size.y - 80.0).abs() < 1e-9 && flat.counts() == [8, 0, 0], "{:?} {:?}", flat.size, flat.counts());
    }
    // 20 wide, its end a half circle from (30, 70) round to (50, 70).
    let round = on_sheet(&b, profile(&[[30.0, 60.0], [50.0, 60.0], [50.0, 70.0], [30.0, 70.0]], Some((2, [40.0, 70.0]))), "join").unwrap();
    let area = 20.0 * 10.0 + 0.5 * std::f64::consts::PI * 100.0;
    assert!((round.volume() - b.volume() - area * T).abs() < 1e-6, "{}", round.volume() - b.volume());
    assert_eq!(unfold_side(&round, false, K).unwrap().counts(), [7, 1, 0]);
    // On a bent sheet, from the side edge of a wall, either side of the blank.
    let l = sheet_of(80.0);
    for side in ["front", "back"] {
        let flat = unfold_side(&l, side == "back", K).unwrap();
        let (x, h) = (flat.bends[0][0].x, flat.size.y);
        let placed = on_blank(&l, side, profile(&[[x - 30.0, h], [x - 20.0, h], [x - 20.0, h + 10.0], [x - 30.0, h + 10.0]], None)).unwrap();
        let op = op(serde_json::json!({"op": "extrude", "mode": "join", "through": true, "wall": true}));
        let r = op.eval(id("tab"), &[&Value::Body(l.clone()), &Value::Sketch(Box::new(placed))]).unwrap().body().unwrap().clone();
        r.validate().unwrap();
        assert!((r.volume() - l.volume() - 100.0 * T).abs() < 1e-6, "{side}: {}", r.volume() - l.volume());
        let grown = unfold_side(&r, side == "back", K).unwrap();
        assert!((grown.size.y - h - 10.0).abs() < 1e-9 && grown.bends.len() == 2, "{side}: {:?}", grown.size);
    }
}

/// A notch drawn with a side on the edge of the blank cuts as one crossing it.
#[test]
fn a_cut_with_a_side_on_the_edge_is_a_notch() {
    let b = plate();
    let notch = on_sheet(&b, profile(&[[30.0, 50.0], [50.0, 50.0], [50.0, 60.0], [30.0, 60.0]], None), "cut").unwrap();
    assert!((b.volume() - notch.volume() - 200.0 * T).abs() < 1e-6, "{}", b.volume() - notch.volume());
    assert_eq!(unfold_side(&notch, false, K).unwrap().counts(), [8, 0, 0]);
}

/// What an addition does not do yet, or cannot: a tab across a bend line, a profile off the edge.
#[test]
fn an_addition_refuses_what_it_does_not_do() {
    let b = sheet_of(40.0);
    let flat = unfold_side(&b, false, K).unwrap();
    let x = flat.bends[0][0].x;
    let h = flat.size.y;
    let across = on_sheet(&b, profile(&[[x - 5.0, h], [x + 10.0, h], [x + 10.0, h + 10.0], [x - 5.0, h + 10.0]], None), "join").unwrap_err();
    assert!(across.to_string().contains("across a bend line"), "{across}");
    let off = on_sheet(&plate(), profile(&[[30.0, 70.0], [50.0, 70.0], [50.0, 80.0], [30.0, 80.0]], None), "join").unwrap_err();
    assert!(off.to_string().contains("meets none"), "{off}");
}
