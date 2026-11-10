//! Bounded arrays of open chains (D253): concentric copies, each end stopped by a curve of a
//! group, and ribs rolled along them.
#![allow(clippy::unwrap_used, clippy::panic)]

use datum_brep::{BrepError, Surface, TopoId, cuboid, tessellate};
use datum_math::{Vec2, Vec3};
use datum_ops::{ChainCopy, Op, Placed, Value};
use serde_json::{Value as Json, json};

fn id(tag: &str) -> TopoId {
    TopoId::derive(tag).finish()
}

fn op(json: Json) -> Op {
    serde_json::from_value(json).unwrap()
}

fn sketch(s: Json) -> Result<Value, BrepError> {
    op(json!({"op": "sketch", "sketch": s})).eval(id("path"), &[])
}

fn placed(v: &Value) -> &Placed {
    let Value::Sketch(p) = v else { panic!("not a sketch") };
    p
}

fn copies(s: Json) -> Vec<ChainCopy> {
    placed(&sketch(s).unwrap()).patterns[0].chains.clone()
}

fn error(s: Json) -> String {
    sketch(s).err().unwrap().to_string()
}

fn close(a: Vec2, x: f64, y: f64) {
    assert!((a.x - x).abs() < 1e-9 && (a.y - y).abs() < 1e-9, "{a:?} is not ({x}, {y})");
}

/// A line along X from x = 50 to 150 at y = 20, a bound at x = 30 and one at x = 170, both
/// construction lines across it.
fn straight(offset: Json) -> Json {
    json!({
        "points": {"a": [50, 20], "b": [150, 20], "s0": [30, 0], "s1": [30, 100], "e0": [170, 0], "e1": [170, 100]},
        "lines": {"l": {"from": "a", "to": "b"},
                  "start": {"from": "s0", "to": "s1", "construction": true},
                  "end": {"from": "e0", "to": "e1", "construction": true}},
        "patterns": {"ribs": {"of": "l", "offset": offset}}
    })
}

/// Copies go to the left of the way the curve is drawn, and both ends reach their bounds, the
/// cell's too. The cell keeps its curve's name.
#[test]
fn copies_are_extended_to_their_bounds() {
    let c = copies(straight(json!({"step": 20, "count": 3, "start": "start", "end": "end"})));
    assert_eq!(c.len(), 3);
    for (k, y) in [(0, 20.0), (1, 40.0), (2, 60.0)] {
        let p = &c[k].pieces;
        assert_eq!(p.len(), 1);
        close(p[0].1.from, 30.0, y);
        close(p[0].1.to, 170.0, y);
    }
    assert_eq!(c[0].pieces[0].0, "l");
    assert_eq!(c[2].pieces[0].0, "ribs[2].l");
}

/// A negative step goes to the right, and an end with no bound stays where it is.
#[test]
fn an_end_without_a_bound_stays() {
    let c = copies(straight(json!({"step": -5, "count": 2, "end": "end"})));
    close(c[1].pieces[0].1.from, 50.0, 15.0);
    close(c[1].pieces[0].1.to, 170.0, 15.0);
}

/// A bound across the chain cuts it short.
#[test]
fn a_bound_cuts_a_copy_short() {
    let mut s = straight(json!({"step": 20, "count": 2, "end": "end"}));
    s["points"]["e0"] = json!([120, 0]);
    s["points"]["e1"] = json!([120, 100]);
    let c = copies(s);
    close(c[0].pieces[0].1.to, 120.0, 20.0);
    close(c[1].pieces[0].1.to, 120.0, 40.0);
}

/// The author's shape: a line up into a tangent arc, copies concentric, stopped by an arc and a
/// tangent line that are one group; the copy passing through their joint finds it.
fn hooked(offset: Json) -> Json {
    json!({
        "points": {"a": [0, 0], "b": [0, 50], "c": [50, 100], "o": [50, 50],
                   "g0": [80, 130], "g1": [80, 90], "g2": [120, 50], "go": [120, 90]},
        "lines": {"l": {"from": "a", "to": "b"},
                  "gl": {"from": "g0", "to": "g1", "construction": true}},
        "arcs": {"a1": {"center": "o", "from": "c", "to": "b"},
                 "ga": {"center": "go", "from": "g1", "to": "g2", "construction": true}},
        "constraints": [{"type": "tangent", "curves": ["l", "a1"]}],
        "groups": {"cell": ["l", "a1"], "bound": ["gl", "ga"]},
        "patterns": {"ribs": {"of": "cell", "offset": offset}}
    })
}

#[test]
fn an_arc_is_copied_about_its_centre() {
    let c = copies(hooked(json!({"step": 10, "count": 3})));
    // Up and round to the right: the left is away from the centre, on the side of larger radii.
    let arc = &c[2].pieces[1].1;
    assert_eq!(arc.arc.unwrap().center, Vec2::new(50.0, 50.0));
    close(c[2].pieces[0].1.from, -20.0, 0.0);
    close(arc.from, -20.0, 50.0);
    close(arc.to, 50.0, 120.0);
}

#[test]
fn a_copy_stops_on_any_curve_of_its_bound_group() {
    let c = copies(hooked(json!({"step": 10, "count": 2, "end": "bound"})));
    assert_eq!(c.len(), 2);
    // The arc of radius 50 goes on past where it was drawn and through the joint of the bound.
    close(c[0].pieces[1].1.to, 80.0, 90.0);
    close(c[1].pieces[1].1.to, 80.0, 50.0 + 2700f64.sqrt());
}

/// One group ends both ends: from the middle outward each finds its own side.
#[test]
fn one_group_bounds_both_ends() {
    let s = json!({
        "points": {"a": [50, 20], "b": [150, 20], "c": [100, 20], "t": [20, 20]},
        "lines": {"l": {"from": "a", "to": "b"}},
        "circles": {"round": {"center": "c", "diameter": 200, "construction": true}},
        "patterns": {"ribs": {"of": "l", "offset": {"step": 30, "count": 1, "start": "round", "end": "round"}}}
    });
    let c = copies(s);
    close(c[0].pieces[0].1.from, 0.0, 20.0);
    close(c[0].pieces[0].1.to, 200.0, 20.0);
}

/// Several chains in the cell: each is copied on the same side and stopped by itself, however it
/// was drawn.
#[test]
fn several_chains_go_the_same_way() {
    let s = json!({
        "points": {"a": [50, 20], "b": [150, 20], "c": [140, 50], "d": [60, 50], "e0": [170, 0], "e1": [170, 100]},
        "lines": {"one": {"from": "a", "to": "b"}, "two": {"from": "c", "to": "d"},
                  "end": {"from": "e0", "to": "e1", "construction": true}},
        "groups": {"cell": ["one", "two"]},
        "patterns": {"ribs": {"of": "cell", "offset": {"step": 10, "count": 2, "end": "end"}}}
    });
    let c = copies(s);
    assert_eq!(c.len(), 4);
    close(c[3].pieces[0].1.from, 60.0, 60.0);
    close(c[3].pieces[0].1.to, 170.0, 60.0);
}

/// The clearance is the distance from the bound, also for a bound the chain meets aslant.
#[test]
fn the_clearance_is_a_distance_from_the_bound() {
    let mut s = straight(json!({"step": 20, "count": 1, "end": "end"}));
    s["points"]["e0"] = json!([140, -50]);
    s["points"]["e1"] = json!([240, 50]);
    s["patterns"]["ribs"]["clearance"] = json!(10);
    let to = copies(s)[0].pieces[0].1.to;
    // The bound is y = x − 190; its distance from a point is |x − y − 190| / √2.
    assert!(to.x < 210.0);
    assert!(((to.x - to.y - 190.0).abs() / 2f64.sqrt() - 10.0).abs() < 1e-9, "{to:?}");
}

#[test]
fn a_copy_missing_its_bound_is_named() {
    let mut s = straight(json!({"step": 20, "count": 3, "end": "end"}));
    s["points"]["e1"] = json!([170, 30]);
    let e = error(s);
    assert!(e.contains("copy 1") && e.contains("end"), "{e}");
}

#[test]
fn an_arc_with_no_room_is_named() {
    let e = error(hooked(json!({"step": -30, "count": 3})));
    assert!(e.contains("copy 2") && e.contains("a1"), "{e}");
}

#[test]
fn a_corner_or_a_closed_contour_is_refused() {
    let mut s = hooked(json!({"step": 10, "count": 2}));
    s.as_object_mut().unwrap().remove("constraints");
    assert!(error(s).contains("tangent"));
    let s = json!({
        "points": {"c": [0, 0]}, "circles": {"o": {"center": "c", "diameter": 10}},
        "patterns": {"p": {"of": "o", "offset": {"step": 20, "count": 2}}}
    });
    assert!(error(s).contains("open chains"));
}

/// Ribs are rolled along every copy, the cell's bounded one in place of what is drawn.
#[test]
fn a_rib_goes_along_every_copy() {
    let plate = Value::Body(cuboid(id("plate"), Vec3::ZERO, Vec3::new(200.0, 100.0, 2.0)).unwrap());
    let top = {
        let b = plate.body().unwrap();
        let is = |f: &datum_brep::Face| matches!(f.surface, Surface::Plane(p) if p.normal.z > 0.0 && p.origin.z == 2.0);
        format!("{:?}", b.faces().iter().find(|f| is(f)).unwrap().id)
    };
    let path = op(json!({"op": "sketch", "face": top, "sketch": straight(json!({"step": 20, "count": 3, "start": "start", "end": "end"}))}))
        .eval(id("path"), &[&plate])
        .unwrap();
    let rib = op(json!({"op": "rib", "tool": "ball", "diameter": 12, "depth": 3, "shoulder": 2}));
    let v = rib.eval(id("rib"), &[&plate, &path]).unwrap();
    let b = v.body().unwrap();
    b.validate().unwrap();
    tessellate(b).unwrap();
    let one = op(json!({"op": "sketch", "face": v.body().map(|_| top.clone()).unwrap(), "sketch": straight(json!({"step": 20, "count": 1, "start": "start", "end": "end"}))}))
        .eval(id("path"), &[&plate])
        .unwrap();
    let single = rib.eval(id("rib"), &[&plate, &one]).unwrap();
    assert!(b.faces().len() > 2 * single.body().unwrap().faces().len() - 6, "{} faces", b.faces().len());
}
