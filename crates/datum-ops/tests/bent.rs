//! A cut across a bend line (D197): a hole punched through the blank, on walls along the normal,
//! on bends along the radius; the blank shows it as drawn, whichever side it is seen from.
#![allow(clippy::unwrap_used, clippy::panic)]

use datum_brep::{Body, BrepError, Surface, TopoId};
use datum_ops::{FlatPiece, NearBend, Op, Value, near_bends, unfold_side};
use proptest::prelude::*;

const K: f64 = 0.44;
/// Thickness, bend radius and width of the sheet; the first wall's flat part on its outer face.
const T: f64 = 2.0;
const WIDTH: f64 = 40.0;
const FLAT: f64 = 58.0;
/// The length of a bend's neutral layer.
const BEND: f64 = std::f64::consts::FRAC_PI_2 * (T + K * T);

fn id(tag: &str) -> TopoId {
    TopoId::derive(tag).finish()
}

fn op(json: serde_json::Value) -> Op {
    serde_json::from_value(json).unwrap()
}

/// An L of sheet metal: up 60 from the origin, then 80 along X, on the left of its path.
fn sheet() -> Body {
    let path = serde_json::json!({"points": {"a": [0, 0], "b": [0, 60], "c": [80, 60]},
        "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}}});
    let p = op(serde_json::json!({"op": "sketch", "sketch": path})).eval(id("path"), &[]).unwrap();
    let s =
        op(serde_json::json!({"op": "sheet", "thickness": T, "radius": T, "side": "left", "z0": 0, "z1": WIDTH})).eval(id("sheet"), &[&p]).unwrap();
    s.body().unwrap().clone()
}

/// The ID of the outer face of the first wall (looking along −X), whose sketch has X along the
/// wall and Y along −Z.
fn first_wall(b: &Body) -> String {
    let f = b.faces().iter().find(|f| matches!(f.surface, Surface::Plane(p) if p.normal.x < -0.5 && p.origin.x <= 0.0)).unwrap();
    format!("{:?}", f.id)
}

/// A circle of `diameter` at `along` the wall and height `z`, cut through the wall of `b`.
fn punched(b: &Body, along: f64, z: f64, diameter: f64) -> Result<Body, BrepError> {
    let target = Value::Body(b.clone());
    let sketch = serde_json::json!({"points": {"h": [along, -z]}, "circles": {"c": {"center": "h", "diameter": diameter}}});
    let placed = op(serde_json::json!({"op": "sketch", "face": first_wall(b), "k_factor": K, "sketch": sketch})).eval(id("hole"), &[&target])?;
    let cut = op(serde_json::json!({"op": "extrude", "mode": "cut", "through": true, "wall": true}));
    let r = cut.eval(id("cut"), &[&target, &placed])?.body().unwrap().clone();
    r.validate().unwrap();
    Ok(r)
}

/// How close a circle of `diameter` at `along` the first wall of `b`, height `z`, comes to a bend.
fn near(b: &Body, along: f64, z: f64, diameter: f64) -> Vec<NearBend> {
    let target = Value::Body(b.clone());
    let sketch = serde_json::json!({"points": {"h": [along, -z]}, "circles": {"c": {"center": "h", "diameter": diameter}}});
    let placed =
        op(serde_json::json!({"op": "sketch", "face": first_wall(b), "k_factor": K, "sketch": sketch})).eval(id("hole"), &[&target]).unwrap();
    let cut = op(serde_json::json!({"op": "extrude", "mode": "cut", "through": true, "wall": true}));
    near_bends(id("cut"), &[&target, &placed], &cut).unwrap()
}

/// 2t + R from the bend line a hole distorts on bending: closer, the check gives the distance;
/// into the bend, none; farther, nothing. A hole on the wall beyond the bend is measured too.
#[test]
fn a_hole_near_a_bend_is_told() {
    let b = sheet();
    let clearance = 2.0 * T + T;
    assert_eq!(near(&b, FLAT - 5.0 - clearance - 0.5, 20.0, 10.0), []);
    let [NearBend { distance: Some(d), clearance: c }] = near(&b, FLAT - 5.0 - 3.0, 20.0, 10.0)[..] else { panic!() };
    assert!((d - 3.0).abs() < 1e-9 && (c - clearance).abs() < 1e-12, "{d} {c}");
    assert_eq!(near(&b, FLAT - 2.0, 20.0, 10.0), [NearBend { distance: None, clearance }]);
    let [NearBend { distance: Some(d), .. }] = near(&b, FLAT + BEND + 5.0 + 2.0, 20.0, 10.0)[..] else { panic!() };
    assert!((d - 2.0).abs() < 1e-9, "{d}");
    // Not a cut of sheet metal (through all the body): nothing to tell.
    let target = Value::Body(b.clone());
    assert_eq!(near_bends(id("x"), &[&target], &op(serde_json::json!({"op": "extrude", "mode": "cut", "through": true}))).unwrap(), []);
}

/// The circles of the blank of `b` seen from the front or the back, as their radii.
fn circles(b: &Body, back: bool) -> Vec<f64> {
    let flat = unfold_side(b, back, K).unwrap();
    assert_eq!(flat.counts()[1], 0, "{:?}", flat.pieces);
    flat.pieces.iter().filter_map(|p| if let FlatPiece::Circle { radius, .. } = *p { Some(radius) } else { None }).collect()
}

/// A hole across the first bend, as an example of both sides.
#[test]
fn a_hole_across_a_bend_is_a_circle_on_either_side() {
    let b = sheet();
    let cut = punched(&b, FLAT - 2.0, 20.0, 10.0).unwrap();
    assert!(cut.volume() < b.volume());
    for back in [true, false] {
        let radii = circles(&cut, back);
        assert!(radii.len() == 1 && (radii[0] - 5.0).abs() < 1e-9, "back: {back}: {radii:?}");
    }
}

/// A circle of radius `r` about `along` the wall, `z` up, crossing the first bend line and not
/// reaching the end of the bend (that would divide the bend's face).
fn crossing() -> impl Strategy<Value = (f64, f64, f64)> {
    (1.0..8.0f64).prop_flat_map(|r| {
        let (lo, hi) = ((r + 0.5).max(FLAT - r + 0.3), FLAT + BEND - r - 0.3);
        (Just(r), lo..hi, (r + 1.0)..(WIDTH - r - 1.0))
    })
}

fn check(back: bool) -> impl Fn((f64, f64, f64)) -> Result<(), TestCaseError> {
    move |(r, along, z)| {
        let b = sheet();
        let cut = punched(&b, along, z, 2.0 * r).map_err(|e| TestCaseError::fail(format!("r {r} along {along} z {z}: {e}")))?;
        prop_assert!(cut.volume() < b.volume());
        let radii = circles(&cut, back);
        prop_assert_eq!(radii.len(), 1, "r {} along {} z {}", r, along, z);
        prop_assert!((radii[0] - r).abs() < 1e-9 * r, "{} against {}", radii[0], r);
        Ok(())
    }
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(32))]

    /// A circle across the bend line is cut, the body valid and lighter, and the blank seen from
    /// its back has it as one circle of the radius drawn.
    #[test]
    fn a_circle_across_a_bend_is_one_circle_on_the_back(c in crossing()) {
        check(true)(c)?;
    }

    /// The same, seen from the front.
    #[test]
    fn a_circle_across_a_bend_is_one_circle_on_the_front(c in crossing()) {
        check(false)(c)?;
    }
}

/// A U of sheet metal: up 60 from the origin, `mid` along X, down to Y = 0.
fn u_sheet(mid: f64) -> Body {
    let path = serde_json::json!({"points": {"a": [0, 0], "b": [0, 60], "c": [mid, 60], "d": [mid, 0]},
        "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}, "cd": {"from": "c", "to": "d"}}});
    let p = op(serde_json::json!({"op": "sketch", "sketch": path})).eval(id("path"), &[]).unwrap();
    let s =
        op(serde_json::json!({"op": "sheet", "thickness": T, "radius": T, "side": "left", "z0": 0, "z1": WIDTH})).eval(id("sheet"), &[&p]).unwrap();
    s.body().unwrap().clone()
}

/// Cuts `sketch` (on the first wall's face) through the wall of `b`.
/// The cut's ID comes from the body's face count: a second cut has IDs of its own.
fn cut_with(b: &Body, sketch: serde_json::Value) -> Result<Body, BrepError> {
    let tag = format!("cut{}", b.faces().len());
    let target = Value::Body(b.clone());
    let placed =
        op(serde_json::json!({"op": "sketch", "face": first_wall(b), "k_factor": K, "sketch": sketch})).eval(id(&format!("{tag}s")), &[&target])?;
    let cut = op(serde_json::json!({"op": "extrude", "mode": "cut", "through": true, "wall": true}));
    let r = cut.eval(id(&tag), &[&target, &placed])?.body().unwrap().clone();
    r.validate().unwrap();
    Ok(r)
}

/// A rectangle from `x0` to `x1` along the wall and from `y0` to `y1` on the sketch's Y.
fn slot_cut(b: &Body, x0: f64, x1: f64, y0: f64, y1: f64) -> Result<Body, BrepError> {
    let sketch = serde_json::json!({"points": {"p": [x0, y0], "q": [x1, y0], "r": [x1, y1], "s": [x0, y1]},
        "lines": {"pq": {"from": "p", "to": "q"}, "qr": {"from": "q", "to": "r"}, "rs": {"from": "r", "to": "s"}, "sp": {"from": "s", "to": "p"}}});
    cut_with(b, sketch)
}

fn circle_cut(b: &Body, x: f64, y: f64, diameter: f64) -> Result<Body, BrepError> {
    cut_with(b, serde_json::json!({"points": {"h": [x, y]}, "circles": {"c": {"center": "h", "diameter": diameter}}}))
}

/// The size of the blank seen from the front and the counts of its pieces.
fn blank(b: &Body) -> ((f64, f64), [usize; 3]) {
    let flat = unfold_side(b, false, K).unwrap();
    ((flat.size.x, flat.size.y), flat.counts())
}

fn assert_size(got: (f64, f64), want: (f64, f64)) {
    assert!((got.0 - want.0).abs() < 1e-9 * want.0 && (got.1 - want.1).abs() < 1e-9 * want.1, "{got:?} against {want:?}");
}

/// A slot over a bend, the whole top and the next bend divides all three faces; the blank is
/// the outline and the slot as four whole lines.
#[test]
fn a_slot_over_a_whole_wall_divides_it() {
    let b = u_sheet(20.0);
    let cut = slot_cut(&b, 50.0, 85.0, -15.0, -25.0).unwrap();
    assert!(cut.volume() < b.volume());
    let ids: Vec<_> = cut.faces().iter().map(|f| f.id).collect();
    for f in b.faces() {
        assert!(ids.contains(&f.id), "{:?} is gone", f.id);
    }
    let (size, counts) = blank(&cut);
    assert_size(size, (141.0477868423386, 40.0));
    assert_eq!(counts, [8, 0, 0]);
}

/// A hole over a narrow top and both its bends is one circle in the blank.
#[test]
fn a_hole_over_a_narrow_wall_is_one_circle() {
    let cut = circle_cut(&u_sheet(10.0), 62.0, -20.0, 14.0).unwrap();
    let (size, counts) = blank(&cut);
    assert_size(size, (131.0477868423386, 40.0));
    assert_eq!(counts, [4, 0, 1]);
    let radii = circles(&cut, false);
    assert!(radii.len() == 1 && (radii[0] - 7.0).abs() < 1e-9, "{radii:?}");
}

/// A hole wider than a bend, over it and the top, is one circle too.
#[test]
fn a_hole_wider_than_a_bend_is_one_circle() {
    let cut = circle_cut(&u_sheet(80.0), 60.3, -20.0, 8.0).unwrap();
    let (size, counts) = blank(&cut);
    assert_size(size, (201.0477868423386, 40.0));
    assert_eq!(counts, [4, 0, 1]);
    let radii = circles(&cut, false);
    assert!(radii.len() == 1 && (radii[0] - 4.0).abs() < 1e-9, "{radii:?}");
}

/// A sheet with divided faces takes more cuts.
#[test]
fn a_divided_sheet_is_cut_again() {
    let slotted = slot_cut(&u_sheet(20.0), 50.0, 85.0, -15.0, -25.0).unwrap();
    let holed = circle_cut(&slotted, 68.0, -5.0, 4.0).unwrap();
    assert!(holed.volume() < slotted.volume());
    assert_eq!(blank(&holed).1, [8, 0, 1]);
    let again = slot_cut(&slotted, 50.0, 85.0, -30.0, -35.0).unwrap();
    assert!(again.volume() < slotted.volume());
    assert_eq!(blank(&again).1, [12, 0, 0]);
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(32))]

    /// Any slot across the bends and the top, clear of the sheet's edges, keeps the body valid
    /// and shows in the blank as four whole lines.
    #[test]
    fn a_slot_across_a_wall_is_four_lines(
        (lo, hi, h) in (40.0..55.0f64, 82.0..95.0f64, 2.0..8.0f64),
        t in 0.0..1.0f64,
    ) {
        let top = -5.0 - t * (30.0 - h);
        let b = u_sheet(20.0);
        let cut = slot_cut(&b, lo, hi, top, top - h)
            .map_err(|e| TestCaseError::fail(format!("{lo} {hi} {top} {h}: {e}")))?;
        prop_assert!(cut.volume() < b.volume());
        let (size, counts) = blank(&cut);
        prop_assert_eq!(counts, [8, 0, 0], "{} {} {} {}", lo, hi, top, h);
        prop_assert!((size.0 - 141.0477868423386).abs() < 1e-9 * 141.0 && (size.1 - 40.0).abs() < 1e-9);
    }
}

/// The volume `cut` removed from `b`, against `want`.
fn assert_removed(b: &Body, cut: &Body, want: f64) {
    cut.validate().unwrap();
    let got = b.volume() - cut.volume();
    assert!((got - want).abs() < 1e-9 * want, "removed {got} against {want}");
}

/// A slot reaching over both bends from the sheet's edge: a notch in the side.
#[test]
fn a_side_slot_over_both_bends() {
    let b = u_sheet(20.0);
    let cut = slot_cut(&b, 50.0, 85.0, 5.0, -10.0).unwrap();
    assert_removed(&b, &cut, 707.5398223686134);
    let (size, counts) = blank(&cut);
    assert_size(size, (141.0477868423386, 40.0));
    assert_eq!(counts, [8, 0, 0]);
}

/// Half a circle centred on the edge, on the flat wall.
#[test]
fn a_half_circle_on_the_edge_of_a_wall() {
    let b = u_sheet(20.0);
    let cut = circle_cut(&b, 30.0, 0.0, 10.0).unwrap();
    assert_removed(&b, &cut, std::f64::consts::PI * 25.0);
    assert_eq!(blank(&cut).1, [5, 1, 0]);
}

/// Half a circle centred on the edge, over a bend.
#[test]
fn a_half_circle_on_the_edge_over_a_bend() {
    let b = u_sheet(20.0);
    let cut = circle_cut(&b, 60.0, 0.0, 10.0).unwrap();
    assert_removed(&b, &cut, 80.35543932165456);
    assert_eq!(blank(&cut).1, [5, 1, 0]);
}

/// A notch in the corner at the sheet's end.
#[test]
fn a_corner_notch_at_the_end() {
    let b = u_sheet(20.0);
    let cut = slot_cut(&b, -5.0, 10.0, 5.0, -10.0).unwrap();
    assert_removed(&b, &cut, 200.0);
    assert_eq!(blank(&cut).1, [6, 0, 0]);
}

/// A notch on the flat wall only.
#[test]
fn a_side_slot_on_the_wall_only() {
    let b = u_sheet(20.0);
    let cut = slot_cut(&b, 20.0, 30.0, 5.0, -10.0).unwrap();
    assert_removed(&b, &cut, 200.0);
    assert_eq!(blank(&cut).1, [8, 0, 0]);
}

/// Notches in both sides of the sheet.
#[test]
fn notches_in_both_sides() {
    let b = u_sheet(20.0);
    let one = slot_cut(&b, 50.0, 85.0, 5.0, -10.0).unwrap();
    let two = slot_cut(&one, 50.0, 85.0, -30.0, -45.0).unwrap();
    assert!(two.volume() < one.volume());
    assert_eq!(blank(&two).1, [12, 0, 0]);
}

/// A cut across the whole width divides the sheet in two.
#[test]
fn a_cut_across_the_width_is_refused() {
    let e = slot_cut(&u_sheet(20.0), 50.0, 85.0, 5.0, -45.0).unwrap_err();
    assert!(matches!(&e, BrepError::Unsupported(m) if m.contains("two parts")), "{e:?}");
}

/// A notch reaching into an earlier hole joins it (D203): what it takes is its own area less the
/// hole's part in it; a side running through the hole's corners is a contact not resolved.
#[test]
fn a_notch_into_a_hole_joins_it() {
    let holed = circle_cut(&u_sheet(20.0), 30.0, -8.0, 6.0).unwrap();
    let joined = slot_cut(&holed, 25.0, 40.0, 5.0, -10.0).unwrap();
    // The hole below the notch's floor 2 from its centre: a circular segment.
    let segment = 9.0 * libm::acos(2.0 / 3.0) - 2.0 * 5.0_f64.sqrt();
    let want = 2.0 * (15.0 * 10.0 - (std::f64::consts::PI * 9.0 - segment));
    let got = holed.volume() - joined.volume();
    assert!((got - want).abs() < 1e-9 * want, "{got} against {want}");
    let e = slot_cut(&holed, 25.0, 40.0, 5.0, -8.0).unwrap_err();
    assert!(matches!(&e, BrepError::Degenerate(_)), "{e:?}");
}

/// A plain slab 100 by 60 by 2, with a notch cut from its top face (sketch X, Y as the world's).
fn plate_cut(sketch: serde_json::Value) -> (Body, Body) {
    let rect = serde_json::json!({"points": {"a": [0, 0], "b": [100, 0], "c": [100, 60], "d": [0, 60]},
        "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}, "cd": {"from": "c", "to": "d"}, "da": {"from": "d", "to": "a"}}});
    let s = op(serde_json::json!({"op": "sketch", "sketch": rect})).eval(id("plate_sketch"), &[]).unwrap();
    let b = op(serde_json::json!({"op": "extrude", "z0": 0, "z1": T})).eval(id("plate"), &[&s]).unwrap().body().unwrap().clone();
    let top = b.faces().iter().find(|f| matches!(f.surface, Surface::Plane(p) if p.normal.z > 0.5)).unwrap();
    let target = Value::Body(b.clone());
    let placed =
        op(serde_json::json!({"op": "sketch", "face": format!("{:?}", top.id), "sketch": sketch})).eval(id("notch_sketch"), &[&target]).unwrap();
    let cut = op(serde_json::json!({"op": "extrude", "mode": "cut", "through": true, "wall": true}));
    let r = cut.eval(id("notch"), &[&target, &placed]).unwrap().body().unwrap().clone();
    (b, r)
}

fn plate_blank(b: &Body) -> [usize; 3] {
    unfold_side(b, false, K).unwrap().counts()
}

fn rect_sketch(x0: f64, x1: f64, y0: f64, y1: f64) -> serde_json::Value {
    serde_json::json!({"points": {"p": [x0, y0], "q": [x1, y0], "r": [x1, y1], "s": [x0, y1]},
        "lines": {"pq": {"from": "p", "to": "q"}, "qr": {"from": "q", "to": "r"}, "rs": {"from": "r", "to": "s"}, "sp": {"from": "s", "to": "p"}}})
}

#[test]
fn a_plate_has_a_corner_notch() {
    let (b, cut) = plate_cut(rect_sketch(-5.0, 10.0, -5.0, 10.0));
    assert_removed(&b, &cut, 200.0);
    assert_eq!(plate_blank(&cut), [6, 0, 0]);
}

#[test]
fn a_plate_has_a_side_slot() {
    let (b, cut) = plate_cut(rect_sketch(40.0, 50.0, -5.0, 20.0));
    assert_removed(&b, &cut, 400.0);
    assert_eq!(plate_blank(&cut), [8, 0, 0]);
}

#[test]
fn a_plate_has_a_half_circle() {
    let (b, cut) = plate_cut(serde_json::json!({"points": {"h": [50, 60]}, "circles": {"c": {"center": "h", "diameter": 10}}}));
    assert_removed(&b, &cut, std::f64::consts::PI * 25.0);
    assert_eq!(plate_blank(&cut), [5, 1, 0]);
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(32))]

    /// A notch from the edge, on the first wall, removes its own volume and adds two lines
    /// to the blank's outline and none else.
    #[test]
    fn a_notch_from_the_edge_removes_its_volume(
        (lo, w, d) in (10.0..45.0f64, 3.0..12.0f64, 3.0..15.0f64),
    ) {
        prop_assume!(lo + w <= 55.0);
        let b = u_sheet(20.0);
        let cut = slot_cut(&b, lo, lo + w, 5.0, -d)
            .map_err(|e| TestCaseError::fail(format!("{lo} {w} {d}: {e}")))?;
        prop_assert!(((b.volume() - cut.volume()) - w * d * T).abs() < 1e-9 * w * d * T, "{} {} {}", lo, w, d);
        prop_assert_eq!(blank(&cut).1, [8, 0, 0], "{} {} {}", lo, w, d);
    }
}

/// The body is valid and every face meshes (curved walls too).
fn assert_meshes(b: &Body) {
    b.validate().unwrap();
    datum_brep::tessellate(b).unwrap();
}

/// A cut over an earlier cut on the U sheet: the earlier body, the result, the blank.
fn over(first: serde_json::Value, second: serde_json::Value) -> (Body, Body) {
    let holed = cut_with(&u_sheet(20.0), first).unwrap();
    let r = cut_with(&holed, second).unwrap();
    assert_meshes(&r);
    (holed, r)
}

fn circle_sketch(x: f64, y: f64, diameter: f64) -> serde_json::Value {
    serde_json::json!({"points": {"h": [x, y]}, "circles": {"c": {"center": "h", "diameter": diameter}}})
}

const U_BLANK: (f64, f64) = (141.0477868423386, 40.0);

/// The area of a circle segment of radius `r` beyond a chord `a` from the centre.
fn segment(r: f64, a: f64) -> f64 {
    r * r * libm::acos(a / r) - a * (r * r - a * a).sqrt()
}

/// A rectangle over part of a hole takes its own area less the hole's part in it.
#[test]
fn a_rect_over_part_of_a_hole() {
    let (holed, cut) = over(circle_sketch(30.0, -20.0, 10.0), rect_sketch(32.0, 45.0, -15.0, -25.0));
    assert_removed(&holed, &cut, 2.0 * (130.0 - segment(5.0, 2.0)));
    let (size, counts) = blank(&cut);
    assert_size(size, U_BLANK);
    assert_eq!(counts, [9, 3, 0]);
}

/// A rectangle holding the hole whole: the hole and its walls go away.
#[test]
fn a_rect_holding_a_hole() {
    let (holed, cut) = over(circle_sketch(30.0, -20.0, 10.0), rect_sketch(20.0, 45.0, -10.0, -30.0));
    assert_removed(&holed, &cut, 2.0 * (500.0 - 25.0 * std::f64::consts::PI));
    let (size, counts) = blank(&cut);
    assert_size(size, U_BLANK);
    assert_eq!(counts, [8, 0, 0]);
}

/// A circle over part of a hole: the lens is taken once.
#[test]
fn a_circle_over_part_of_a_hole() {
    let (holed, cut) = over(circle_sketch(30.0, -20.0, 10.0), circle_sketch(36.0, -20.0, 8.0));
    // The lens of circles of radii 5 and 4, 6 apart.
    let lens = 25.0 * libm::acos(45.0 / 60.0) + 16.0 * libm::acos(27.0 / 48.0) - 0.5 * (3.0 * 7.0 * 5.0 * 15.0_f64).sqrt();
    assert_removed(&holed, &cut, 2.0 * (16.0 * std::f64::consts::PI - lens));
    let (size, counts) = blank(&cut);
    assert_size(size, U_BLANK);
    assert_eq!(counts, [4, 7, 0]);
}

/// A circle centred on a slot's floor, over the slot's end.
#[test]
fn a_circle_over_a_slots_end() {
    let (slot, cut) = over(rect_sketch(20.0, 30.0, 5.0, -10.0), circle_sketch(25.0, -10.0, 8.0));
    assert_removed(&slot, &cut, 2.0 * 8.0 * std::f64::consts::PI);
    let (size, counts) = blank(&cut);
    assert_size(size, U_BLANK);
    assert_eq!(counts, [9, 1, 0]);
}

/// A rectangle over the arc of a notch on a bend.
#[test]
fn a_rect_over_a_notch_on_a_bend() {
    let (notch, cut) = over(circle_sketch(60.0, 0.0, 30.0), rect_sketch(70.0, 90.0, 5.0, -25.0));
    assert_removed(&notch, &cut, 931.9877259579771);
    let (size, counts) = blank(&cut);
    assert_size(size, U_BLANK);
    assert_eq!(counts, [8, 1, 0]);
}

/// A small circle centred on a notch's arc, its piece of wall on the bend wholly inside it.
#[test]
fn a_circle_on_the_arc_of_a_notch_on_a_bend() {
    let (notch, cut) = over(circle_sketch(60.0, 0.0, 30.0), circle_sketch(60.0, -15.0, 6.0));
    assert_removed(&notch, &cut, 30.50332523954785);
    let (size, counts) = blank(&cut);
    assert_size(size, U_BLANK);
    assert_eq!(counts, [5, 4, 0]);
}

/// A circle over a hole across a bend.
#[test]
fn a_circle_over_a_hole_across_a_bend() {
    let (holed, cut) = over(circle_sketch(58.0, -20.0, 8.0), circle_sketch(63.0, -20.0, 8.0));
    assert_removed(&holed, &cut, 75.12375167939535);
    let (size, counts) = blank(&cut);
    assert_size(size, U_BLANK);
    assert_eq!(counts, [4, 3, 0]);
}

/// A side running through the points where the hole's circle is split is a contact not resolved.
#[test]
fn a_cut_through_the_corners_of_a_hole_is_degenerate() {
    let holed = circle_cut(&u_sheet(20.0), 30.0, -20.0, 10.0).unwrap();
    let e = slot_cut(&holed, 28.0, 40.0, -20.0, -30.0).unwrap_err();
    assert!(matches!(&e, BrepError::Degenerate(_)), "{e:?}");
}

/// A plain slab with a hole at (50, 30), then a second cut over it: the slab with the hole and
/// the slab with both.
fn plate_over(second: serde_json::Value) -> (Body, Body) {
    let (_, holed) = plate_cut(circle_sketch(50.0, 30.0, 10.0));
    let top = holed.faces().iter().find(|f| matches!(f.surface, Surface::Plane(p) if p.normal.z > 0.5)).unwrap();
    let target = Value::Body(holed.clone());
    let placed =
        op(serde_json::json!({"op": "sketch", "face": format!("{:?}", top.id), "sketch": second})).eval(id("second_sketch"), &[&target]).unwrap();
    let cut = op(serde_json::json!({"op": "extrude", "mode": "cut", "through": true, "wall": true}));
    let r = cut.eval(id("second"), &[&target, &placed]).unwrap().body().unwrap().clone();
    assert_meshes(&r);
    (holed, r)
}

#[test]
fn a_plate_rect_over_part_of_a_hole() {
    let (holed, cut) = plate_over(rect_sketch(52.0, 70.0, 25.0, 35.0));
    assert_removed(&holed, &cut, 2.0 * (180.0 - segment(5.0, 2.0)));
    assert_eq!(plate_blank(&cut), [9, 3, 0]);
}

#[test]
fn a_plate_rect_holding_a_hole() {
    let (holed, cut) = plate_over(rect_sketch(40.0, 60.0, 20.0, 40.0));
    assert_removed(&holed, &cut, 2.0 * (400.0 - 25.0 * std::f64::consts::PI));
    assert_eq!(plate_blank(&cut), [8, 0, 0]);
}

#[test]
fn a_plate_notch_into_a_hole() {
    let (holed, cut) = plate_over(rect_sketch(48.0, 52.0, -5.0, 30.0));
    assert_removed(&holed, &cut, 201.09385491680223);
    assert_eq!(plate_blank(&cut), [7, 4, 0]);
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(32))]

    /// A rectangle crossing a hole's left side, its sides clear of the circle's split points,
    /// takes its own area less the circle's part in it, and leaves no circle in the blank.
    #[test]
    fn a_rect_over_a_hole_takes_the_rest(x0 in 31.0..34.0f64, b in 0.3..2.0f64) {
        let holed = circle_cut(&u_sheet(20.0), 30.0, -20.0, 10.0).unwrap();
        let d = 5.0 - b;
        let cut = slot_cut(&holed, x0, 45.0, -20.0 + d, -20.0 - d)
            .map_err(|e| TestCaseError::fail(format!("{x0} {b}: {e}")))?;
        assert_meshes(&cut);
        // The circle's part right of x0, within the band: twice the area under the chord.
        let a = x0 - 30.0;
        let h = d.min((25.0 - a * a).sqrt());
        let under = 0.5 * (h * (25.0 - h * h).sqrt() + 25.0 * libm::asin(h / 5.0));
        let overlap = 2.0 * (under - a * h);
        let want = T * ((45.0 - x0) * 2.0 * d - overlap);
        let got = holed.volume() - cut.volume();
        prop_assert!((got - want).abs() < 1e-9 * want, "{} {}: {} against {}", x0, b, got, want);
        prop_assert_eq!(blank(&cut).1[2], 0, "{} {}", x0, b);
    }
}
