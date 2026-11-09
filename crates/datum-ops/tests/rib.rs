//! Ribs rolled along a path on a flat wall (D250): the sheet keeps its thickness, so the volume
//! is the plate's and what the section adds, along the path and in the two half turns at its ends.
#![allow(clippy::unwrap_used, clippy::panic)]

use std::f64::consts::PI;

use datum_brep::{BrepError, Surface, TopoId, cuboid, step, tessellate};
use datum_math::Vec3;
use datum_ops::{Op, Value};
use serde_json::{Value as Json, json};

fn id(tag: &str) -> TopoId {
    TopoId::derive(tag).finish()
}

fn op(json: Json) -> Op {
    serde_json::from_value(json).unwrap()
}

/// A plate 200 × 100 × `t`, its top at z = `t`.
fn plate(t: f64) -> Value {
    Value::Body(cuboid(id("plate"), Vec3::ZERO, Vec3::new(200.0, 100.0, t)).unwrap())
}

fn top(v: &Value, t: f64) -> String {
    let b = v.body().unwrap();
    let is = |f: &datum_brep::Face| matches!(f.surface, Surface::Plane(p) if p.normal.z > 0.0 && p.origin.z == t);
    format!("{:?}", b.faces().iter().find(|f| is(f)).unwrap().id)
}

fn sketch_on(v: &Value, t: f64, sketch: Json) -> Value {
    op(json!({"op": "sketch", "face": top(v, t), "sketch": sketch})).eval(id("path"), &[v]).unwrap()
}

/// A straight path along X at y = 50.
fn straight() -> Json {
    json!({"points": {"a": [50, 50], "b": [150, 50]}, "lines": {"l": {"from": "a", "to": "b"}}})
}

fn rib_op(extra: Json) -> Op {
    let mut j = json!({"op": "rib", "tool": "ball", "diameter": 12, "depth": 3, "shoulder": 2});
    j.as_object_mut().unwrap().extend(extra.as_object().unwrap().clone());
    op(j)
}

fn ribbed(sheet: &Value, path: &Value, extra: Json) -> Result<Value, BrepError> {
    let v = rib_op(extra).eval(id("rib"), &[sheet, path])?;
    let b = v.body().unwrap();
    b.validate().unwrap();
    tessellate(b).unwrap();
    Ok(v)
}

/// What a section of crown `r`, depth `h` and shoulder `s` on sheet `t` adds per unit length
/// (over both sides of the path), and what a full turn of half of it adds: numeric, by the height
/// of the sheet over its flat place.
fn extra(r: f64, h: f64, s: f64, t: f64) -> (f64, f64) {
    let reach = r + s + t;
    let w = (h * (2.0 * reach - h)).sqrt();
    let cz = h - r;
    let dz = t + s;
    let height = |x: f64, layer: f64| -> f64 {
        let (cr, sr) = if layer == 0.0 { (r, s + t) } else { (r + t, s) };
        // Crown up to the touch point, shoulder after it. The touch point is on the line CD.
        let touch = cr * w / reach;
        if x <= touch { cz + (cr * cr - x * x).sqrt() } else { dz - (sr * sr - (w - x) * (w - x)).sqrt() }
    };
    // In the sheet the two sides run apart from each other: integrate each side up to its own edge.
    let n = 400_000;
    let (mut line, mut turn) = (0.0, 0.0);
    for i in 0..n {
        let x = w * (i as f64 + 0.5) / n as f64;
        let d = height(x, 1.0) - height(x, 0.0) - t;
        line += d * w / n as f64;
        turn += d * x * w / n as f64;
    }
    (2.0 * line, 2.0 * PI * turn)
}

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol * b.abs()
}

#[test]
fn a_straight_rib_adds_what_its_section_does() {
    let t = 2.0;
    let sheet = plate(t);
    let path = sketch_on(&sheet, t, straight());
    let v = ribbed(&sheet, &path, json!({})).unwrap();
    let b = v.body().unwrap();
    let (line, turn) = extra(6.0, 3.0, 2.0, t);
    let expected = 200.0 * 100.0 * t + 100.0 * line + turn;
    assert!(close(b.volume(), expected, 1e-6), "{} vs {expected}", b.volume());
    // A rib up out of the top: its crown is above the sheet, depth over the top face.
    let (_, hi) = b.bounds();
    assert!((hi.z - (t + 3.0)).abs() < 1e-9, "{}", hi.z);
    assert!(b.faces().iter().any(|f| matches!(f.surface, Surface::Torus(t) if t.major == 0.0)));
    // The blank is the plate's: nothing else changes what it is made from.
    let Value::Formed(f) = &v else { panic!("no formed value") };
    assert_eq!(f.blank.volume(), 200.0 * 100.0 * t);
    assert_eq!(b.shape(), (1, 0));
}

#[test]
fn flipped_it_goes_into_the_sketchs_face_with_the_same_volume() {
    let t = 2.0;
    let sheet = plate(t);
    let path = sketch_on(&sheet, t, straight());
    let (up, down) = (ribbed(&sheet, &path, json!({})).unwrap(), ribbed(&sheet, &path, json!({"flip": true})).unwrap());
    let (up, down) = (up.body().unwrap().clone(), down.body().unwrap().clone());
    assert!(close(up.volume(), down.volume(), 1e-9));
    let (lo, _) = down.bounds();
    assert!((lo.z + 3.0).abs() < 1e-9, "{}", lo.z);
}

/// A segment, then an arc turning left, then a segment: the path's centre line counts, what the
/// inside of a turn loses its outside gains.
#[test]
fn a_rib_round_a_bend_of_its_path() {
    let t = 1.5;
    let sheet = plate(t);
    let r = 30.0;
    let path = sketch_on(
        &sheet,
        t,
        json!({
            "points": {"a": [30, 20], "b": [80, 20], "c": [80, 50], "d": [110, 50], "e": [110, 90]},
            "lines": {"l1": {"from": "a", "to": "b"}, "l2": {"from": "d", "to": "e"}},
            "arcs": {"turn": {"center": "c", "from": "b", "to": "d"}},
            "constraints": [{"type": "tangent", "curves": ["l1", "turn"]}, {"type": "tangent", "curves": ["turn", "l2"]}]
        }),
    );
    let v = ribbed(&sheet, &path, json!({})).unwrap();
    let b = v.body().unwrap();
    let (line, turn) = extra(6.0, 3.0, 2.0, t);
    let length = 50.0 + 0.5 * PI * r + 40.0;
    let expected = 200.0 * 100.0 * t + length * line + turn;
    assert!(close(b.volume(), expected, 1e-6), "{} vs {expected}", b.volume());
    assert!(b.faces().iter().any(|f| matches!(f.surface, Surface::Torus(t) if t.major > 25.0)));
    // STEP writes all of it exactly: nothing approximated.
    let s = step(&[b], "rib", false).unwrap();
    assert_eq!(s.approximated, 0);
    assert!(s.text.contains("TOROIDAL_SURFACE") && s.text.contains("SPHERICAL_SURFACE"));
}

#[test]
fn a_sharp_corner_is_refused_naming_the_curves() {
    let t = 2.0;
    let sheet = plate(t);
    let path = sketch_on(
        &sheet,
        t,
        json!({
            "points": {"a": [40, 30], "b": [100, 30], "c": [100, 80]},
            "lines": {"l1": {"from": "a", "to": "b"}, "l2": {"from": "b", "to": "c"}}
        }),
    );
    let e = ribbed(&sheet, &path, json!({})).unwrap_err();
    assert!(matches!(&e, BrepError::Sketch(m) if m.contains("tangent") && m.contains("l1")), "{e:?}");
}

#[test]
fn a_rib_reaching_the_edge_is_refused() {
    let t = 2.0;
    let sheet = plate(t);
    let near = json!({"points": {"a": [50, 95], "b": [150, 95]}, "lines": {"l": {"from": "a", "to": "b"}}});
    let path = sketch_on(&sheet, t, near);
    let e = ribbed(&sheet, &path, json!({})).unwrap_err();
    assert!(matches!(&e, BrepError::Sketch(m) if m.contains("inside its wall")), "{e:?}");
}

#[test]
fn a_path_shorter_than_the_tool_rolls_is_refused() {
    let t = 2.0;
    let sheet = plate(t);
    let path = sketch_on(&sheet, t, straight());
    let e = ribbed(&sheet, &path, json!({"min_length": 120})).unwrap_err();
    assert!(matches!(&e, BrepError::Sketch(m) if m.contains("shorter")), "{e:?}");
}

#[test]
fn a_depth_past_the_radii_is_refused() {
    let t = 2.0;
    let sheet = plate(t);
    let path = sketch_on(&sheet, t, straight());
    let e = ribbed(&sheet, &path, json!({"depth": 11})).unwrap_err();
    assert!(matches!(&e, BrepError::Degenerate(m) if m.contains("undercuts")), "{e:?}");
}

/// Two ribs side by side; the second lies on the wall the first has already changed.
#[test]
fn two_ribs_one_after_the_other() {
    let t = 2.0;
    let sheet = plate(t);
    let first = ribbed(&sheet, &sketch_on(&sheet, t, straight()), json!({})).unwrap();
    let second = json!({"points": {"a": [50, 20], "b": [150, 20]}, "lines": {"l": {"from": "a", "to": "b"}}});
    let path = sketch_on(&first, t, second);
    let v = rib_op(json!({})).eval(id("rib2"), &[&first, &path]).unwrap();
    let (line, turn) = extra(6.0, 3.0, 2.0, t);
    let expected = 200.0 * 100.0 * t + 2.0 * (100.0 * line + turn);
    assert!(close(v.body().unwrap().volume(), expected, 1e-6));
    assert_eq!(v.body().unwrap().shape(), (1, 0));
}

/// Two ribs at once from one sketch, by two open chains.
#[test]
fn every_open_chain_of_the_sketch_is_a_rib() {
    let t = 2.0;
    let sheet = plate(t);
    let path = sketch_on(
        &sheet,
        t,
        json!({
            "points": {"a": [50, 30], "b": [150, 30], "c": [50, 70], "d": [150, 70]},
            "lines": {"l": {"from": "a", "to": "b"}, "m": {"from": "c", "to": "d"}}
        }),
    );
    let v = ribbed(&sheet, &path, json!({})).unwrap();
    let (line, turn) = extra(6.0, 3.0, 2.0, t);
    let expected = 200.0 * 100.0 * t + 2.0 * (100.0 * line + turn);
    assert!(close(v.body().unwrap().volume(), expected, 1e-6));
}

fn circle(x: f64, y: f64, d: f64) -> Json {
    json!({"points": {"c": [x, y]}, "circles": {"o": {"center": "c", "diameter": d}}})
}

/// The blank is laid from the sheet without its ribs, a later cut goes through both, and a cut
/// under the rib is refused.
#[test]
fn the_blank_and_later_cuts_see_the_sheet_without_its_ribs() {
    let t = 2.0;
    let sheet = plate(t);
    let v = ribbed(&sheet, &sketch_on(&sheet, t, straight()), json!({})).unwrap();
    let unfold = op(json!({"op": "unfold", "face": top(&sheet, t), "k_factor": 0.5}));
    let (plain, formed) = (unfold.eval(id("u"), &[&sheet]).unwrap(), unfold.eval(id("u"), &[&v]).unwrap());
    assert_eq!(plain.hash(), formed.hash(), "the blank does not change");
    // A hole away from the rib: taken from the sheet, the rib laid again.
    let hole = sketch_on(&v, t, circle(30.0, 20.0, 16.0));
    let cut = op(json!({"op": "extrude", "mode": "cut", "through": true}));
    let after = cut.eval(id("hole"), &[&v, &hole]).unwrap();
    let Value::Formed(f) = &after else { panic!("the rib is gone") };
    let area = PI * 64.0 * t;
    assert!(close(f.body.volume(), v.body().unwrap().volume() - area, 1e-9), "{}", f.body.volume());
    assert!(close(f.blank.volume(), 200.0 * 100.0 * t - area, 1e-9));
    // Under the rib: the hole lies in the foot, and at its edge the foot is crossed.
    for (x, y) in [(100.0, 50.0), (100.0, 44.0)] {
        let hole = sketch_on(&v, t, circle(x, y, 6.0));
        let e = cut.eval(id("hole"), &[&v, &hole]).unwrap_err();
        assert!(matches!(&e, BrepError::Sketch(m) if m.contains("inside its wall")), "{e:?}");
    }
}

/// A sheet thick for its section: the shoulder's torus at the end reaches its own axis, which
/// STEP has no ring for; it goes as a circle turned about the axis, and stays exact.
#[test]
fn a_shallow_rib_on_a_thick_sheet_is_exact_in_step() {
    let t = 4.0;
    let sheet = plate(t);
    let v = ribbed(&sheet, &sketch_on(&sheet, t, straight()), json!({"diameter": 6, "depth": 1, "shoulder": 2})).unwrap();
    let s = step(&[v.body().unwrap()], "rib", false).unwrap();
    assert_eq!(s.approximated, 0);
    assert!(s.text.contains("SURFACE_OF_REVOLUTION"));
}
