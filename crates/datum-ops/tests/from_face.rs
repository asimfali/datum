//! Extrusions from a face of the target body (D187): pockets into it, bosses out of it.
#![allow(clippy::unwrap_used, clippy::panic)]

use std::f64::consts::PI;

use datum_brep::{Body, BrepError, TopoId, cuboid, tessellate};
use datum_math::Vec3;
use datum_ops::{Op, Value};

fn id(tag: &str) -> TopoId {
    TopoId::derive(tag).finish()
}

fn assert_close(a: f64, b: f64) {
    assert!((a - b).abs() <= 1e-9 * b.abs().max(1.0), "{a} vs {b}");
}

/// The ID of the face of `b` facing along `n`, as a sketch names it.
fn face(b: &Body, n: Vec3) -> String {
    let f = b.faces().iter().find(|f| f.surface.plane().is_some_and(|p| p.normal.cross(n) == Vec3::ZERO && p.normal.dot(n) > 0.0)).unwrap();
    format!("{:?}", f.id)
}

/// `sketch` on face `on` of `target` extruded with `params` (mode, depth) by the feature `name`.
fn extrude(name: &str, target: &Body, on: &str, sketch: serde_json::Value, mut params: serde_json::Value) -> Result<Body, BrepError> {
    let target = Value::Body(target.clone());
    let s: Op = serde_json::from_value(serde_json::json!({"op": "sketch", "face": on, "sketch": sketch})).unwrap();
    let placed = s.eval(id(&format!("{name}.sketch")), &[&target])?;
    params["op"] = "extrude".into();
    let op: Op = serde_json::from_value(params).unwrap();
    let b = op.eval(id(name), &[&target, &placed])?.body().unwrap().clone();
    b.validate().unwrap();
    tessellate(&b).unwrap();
    Ok(b)
}

fn circle(x: f64, y: f64, d: f64) -> serde_json::Value {
    serde_json::json!({"points": {"h": [x, y]}, "circles": {"c": {"center": "h", "diameter": d}}})
}

#[test]
fn a_pocket_and_a_boss() {
    let plate = cuboid(id("plate"), Vec3::ZERO, Vec3::new(100.0, 60.0, 10.0)).unwrap();
    let top = face(&plate, Vec3::new(0.0, 0.0, 1.0));
    let pocket = extrude("pocket", &plate, &top, circle(30.0, 30.0, 20.0), serde_json::json!({"mode": "cut", "z1": -4})).unwrap();
    assert_close(pocket.volume(), 60_000.0 - 100.0 * PI * 4.0);
    let boss =
        extrude("boss", &pocket, &face(&pocket, Vec3::new(0.0, 0.0, 1.0)), circle(70.0, 30.0, 20.0), serde_json::json!({"mode": "join", "z1": 6}))
            .unwrap();
    assert_close(boss.volume(), 60_000.0 - 100.0 * PI * 4.0 + 100.0 * PI * 6.0);
    // The depth's sign is geometry: a cut out of the face, a join into it, are refused.
    assert!(matches!(extrude("x", &plate, &top, circle(30.0, 30.0, 20.0), serde_json::json!({"mode": "cut", "z1": 4})), Err(BrepError::Sketch(_))));
    assert!(matches!(extrude("x", &plate, &top, circle(30.0, 30.0, 20.0), serde_json::json!({"mode": "join", "z1": -4})), Err(BrepError::Sketch(_))));
    // A floor on the bottom face: refused.
    assert!(extrude("x", &plate, &top, circle(30.0, 30.0, 20.0), serde_json::json!({"mode": "cut", "z1": -10})).is_err());
}

fn plate() -> Body {
    cuboid(id("plate"), Vec3::ZERO, Vec3::new(100.0, 60.0, 10.0)).unwrap()
}

fn up() -> Vec3 {
    Vec3::new(0.0, 0.0, 1.0)
}

fn pocket_of(depth: f64) -> serde_json::Value {
    serde_json::json!({"mode": "cut", "z1": depth})
}

fn boss_of(height: f64) -> serde_json::Value {
    serde_json::json!({"mode": "join", "z1": height})
}

/// A hole of Ø10 at (20, 20) and a slot 20 × 6 beside it.
fn hole_and_slot() -> serde_json::Value {
    serde_json::json!({
        "points": {"h": [20, 20], "a": [30, 17], "b": [50, 17], "c": [50, 23], "d": [30, 23], "l": [30, 20], "r": [50, 20]},
        "lines": {"low": {"from": "a", "to": "b"}, "high": {"from": "c", "to": "d"}},
        "arcs": {"right": {"center": "r", "from": "b", "to": "c"}, "left": {"center": "l", "from": "d", "to": "a"}},
        "circles": {"hole": {"center": "h", "diameter": 10}}
    })
}

/// `sketch` with a pattern of `count` copies `step` apart of all its curves.
fn patterned(mut sketch: serde_json::Value, count: [u32; 2], step: [[f64; 2]; 2]) -> serde_json::Value {
    let curves: Vec<String> = ["lines", "arcs", "circles"].iter().filter_map(|k| sketch[*k].as_object()).flat_map(|m| m.keys().cloned()).collect();
    sketch["groups"] = serde_json::json!({"cell": curves});
    sketch["patterns"] = serde_json::json!({"copies": {"of": "cell", "count": count, "step": step}});
    sketch
}

/// A grid of 4 × 3 holes of Ø10.
fn grid() -> serde_json::Value {
    patterned(circle(15.0, 15.0, 10.0), [4, 3], [[25.0, 0.0], [0.0, 15.0]])
}

/// The area of the slot and the hole of `hole_and_slot`.
const PAIR: f64 = 25.0 * PI + 20.0 * 6.0 + 9.0 * PI;

#[test]
fn a_slot_of_lines_and_arcs() {
    let plate = plate();
    let top = face(&plate, up());
    let pocket = extrude("pocket", &plate, &top, hole_and_slot(), pocket_of(-4.0)).unwrap();
    assert_close(pocket.volume(), 60_000.0 - PAIR * 4.0);
    let boss = extrude("boss", &plate, &top, hole_and_slot(), boss_of(6.0)).unwrap();
    assert_close(boss.volume(), 60_000.0 + PAIR * 6.0);
    assert_eq!(pocket.faces().len(), boss.faces().len());
}

#[test]
fn patterns_of_pockets_and_bosses() {
    let plate = plate();
    let top = face(&plate, up());
    let pockets = extrude("pockets", &plate, &top, grid(), pocket_of(-3.0)).unwrap();
    assert_close(pockets.volume(), 60_000.0 - 12.0 * 25.0 * PI * 3.0);
    assert_eq!(pockets.faces().len(), 6 + 12 * 4);
    let bosses = extrude("bosses", &plate, &top, grid(), boss_of(6.0)).unwrap();
    assert_close(bosses.volume(), 60_000.0 + 12.0 * 25.0 * PI * 6.0);
    assert_eq!(bosses.faces().len(), 6 + 12 * 4);
}

#[test]
fn floors_tops_and_walls() {
    let plate = plate();
    let top = face(&plate, up());
    let walls = |b: &Body| {
        b.faces().iter().filter_map(|f| if let datum_brep::Surface::Cylinder(c) = f.surface { Some(c.convex) } else { None }).collect::<Vec<_>>()
    };
    let pocket = extrude("pocket", &plate, &top, grid(), pocket_of(-4.0)).unwrap();
    let floors: Vec<_> = pocket.faces().iter().filter_map(|f| f.surface.plane()).filter(|p| p.origin.z == 6.0).collect();
    assert_eq!(floors.len(), 12);
    assert!(floors.iter().all(|p| p.normal == up()));
    let w = walls(&pocket);
    assert_eq!(w.len(), 36);
    assert!(w.iter().all(|&c| !c));
    let boss = extrude("boss", &plate, &top, grid(), boss_of(6.0)).unwrap();
    let tops: Vec<_> = boss.faces().iter().filter_map(|f| f.surface.plane()).filter(|p| p.origin.z == 16.0).collect();
    assert_eq!(tops.len(), 12);
    assert!(tops.iter().all(|p| p.normal == up()));
    let w = walls(&boss);
    assert_eq!(w.len(), 36);
    assert!(w.iter().all(|&c| c));
}

#[test]
fn ids_are_stable() {
    let plate = plate();
    let top = face(&plate, up());
    let ids = |b: &Body| b.faces().iter().map(|f| f.id).collect::<std::collections::BTreeSet<_>>();
    let shallow = extrude("pocket", &plate, &top, grid(), pocket_of(-3.0)).unwrap();
    let deep = extrude("pocket", &plate, &top, grid(), pocket_of(-5.0)).unwrap();
    assert_eq!(ids(&shallow), ids(&deep));
    let boss = extrude("boss", &plate, &top, grid(), boss_of(6.0)).unwrap();
    for b in [&shallow, &boss] {
        assert!(b.faces().iter().any(|f| format!("{:?}", f.id) == top));
    }
}

#[test]
fn from_the_bottom_and_from_a_side() {
    let plate = plate();
    // The bottom's axes: X along the world's, Y against it.
    let below = extrude("below", &plate, &face(&plate, -up()), circle(30.0, -30.0, 10.0), pocket_of(-3.0)).unwrap();
    assert_close(below.volume(), 60_000.0 - 25.0 * PI * 3.0);
    let below = extrude("below", &plate, &face(&plate, -up()), circle(30.0, -30.0, 10.0), boss_of(3.0)).unwrap();
    assert_close(below.volume(), 60_000.0 + 25.0 * PI * 3.0);
    let side = face(&plate, Vec3::new(1.0, 0.0, 0.0));
    let sketch = |x: f64, y: f64| circle(x, y, 6.0);
    // The side's axes: X along the world's Y (0…60), Y along Z (0…10).
    let found = extrude("side", &plate, &side, sketch(30.0, 5.0), pocket_of(-3.0)).unwrap();
    assert_close(found.volume(), 60_000.0 - 9.0 * PI * 3.0);
}

#[test]
fn what_a_pocket_or_a_boss_refuses() {
    let plate = plate();
    let top = face(&plate, up());
    // Across the face's edge.
    assert!(matches!(extrude("x", &plate, &top, circle(3.0, 30.0, 10.0), pocket_of(-3.0)), Err(BrepError::Unsupported(_))));
    assert!(matches!(extrude("x", &plate, &top, circle(3.0, 30.0, 10.0), boss_of(3.0)), Err(BrepError::Unsupported(_))));
    // A ring.
    let ring = serde_json::json!({"points": {"h": [50, 30]}, "circles": {"outer": {"center": "h", "diameter": 20}, "inner": {"center": "h", "diameter": 10}}});
    match extrude("x", &plate, &top, ring, pocket_of(-3.0)) {
        Err(BrepError::Unsupported(m)) => assert!(m.contains("inside another"), "{m}"),
        r => panic!("{:?}", r.map(|b| b.volume())),
    }
    // The floor on the bottom face exactly: said so, with what to do.
    match extrude("x", &plate, &top, circle(30.0, 30.0, 10.0), pocket_of(-10.0)) {
        Err(BrepError::Degenerate(m)) => assert!(m.contains("floor lies on a face of the body") && m.contains("cut through"), "{m}"),
        r => panic!("{:?}", r.map(|b| b.volume())),
    }
    // Over a hole made earlier, inside the profile.
    let through = serde_json::json!({"mode": "cut", "through": true});
    let holed = extrude("hole", &plate, &top, circle(30.0, 30.0, 6.0), through).unwrap();
    let r = extrude("x", &holed, &face(&holed, up()), circle(30.0, 30.0, 20.0), pocket_of(-3.0));
    assert!(matches!(r, Err(BrepError::Unsupported(_))), "{:?}", r.map(|b| b.volume()));
    // A second boss meeting the wall of the first.
    let first = extrude("first", &plate, &top, circle(30.0, 30.0, 20.0), boss_of(6.0)).unwrap();
    let r = extrude("second", &first, &face(&first, up()), circle(45.0, 30.0, 20.0), boss_of(6.0));
    assert!(matches!(r, Err(BrepError::Unsupported(_))), "{:?}", r.map(|b| b.volume()));
}

#[test]
fn mode_and_sign_misuse() {
    let plate = plate();
    let top = face(&plate, up());
    let sketch = |v: serde_json::Value| extrude("x", &plate, &top, circle(30.0, 30.0, 10.0), v);
    assert!(matches!(sketch(serde_json::json!({"mode": "join", "through": true})), Err(BrepError::Sketch(_))));
    assert!(matches!(sketch(serde_json::json!({"mode": "cut", "through": true, "z1": -3})), Err(BrepError::Sketch(_))));
    assert!(matches!(sketch(serde_json::json!({"mode": "cut", "z0": -1, "through": true})), Err(BrepError::Sketch(_))));
    // A new body takes a sketch alone: the body is no sketch.
    let r = sketch(serde_json::json!({"mode": "new", "z1": 3}));
    assert!(r.is_err(), "{:?}", r.map(|b| b.volume()));
}

#[test]
fn a_sketch_off_the_faces_goes_through_the_boolean() {
    let target = Value::Body(plate());
    let square = serde_json::json!({"points": {"a": [80, 20], "b": [120, 20], "c": [120, 40], "d": [80, 40]},
        "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}, "cd": {"from": "c", "to": "d"}, "da": {"from": "d", "to": "a"}}});
    let placed = serde_json::from_value::<Op>(serde_json::json!({"op": "sketch", "sketch": square})).unwrap().eval(id("square"), &[]).unwrap();
    let run = |mode: &str| {
        let op: Op = serde_json::from_value(serde_json::json!({"op": "extrude", "mode": mode, "z0": 2, "z1": 15})).unwrap();
        let b = op.eval(id(mode), &[&target, &placed]).unwrap().body().unwrap().clone();
        b.validate().unwrap();
        tessellate(&b).unwrap();
        b.volume()
    };
    // The prism is 40 × 20 × 13; it shares 20 × 20 × 8 with the plate.
    assert_close(run("join"), 60_000.0 + 10_400.0 - 3_200.0);
    assert_close(run("cut"), 60_000.0 - 3_200.0);
    assert_close(run("intersect"), 3_200.0);
}

#[test]
fn the_thread_count_does_not_matter() {
    let plate = cuboid(id("plate"), Vec3::ZERO, Vec3::new(200.0, 200.0, 10.0)).unwrap();
    let top = face(&plate, up());
    let sketch = patterned(circle(15.0, 15.0, 8.0), [10, 10], [[18.0, 0.0], [0.0, 18.0]]);
    let run = |threads| {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap();
        pool.install(|| extrude("pockets", &plate, &top, sketch.clone(), pocket_of(-3.0)).unwrap().hash())
    };
    let one = run(1);
    assert_eq!(run(4), one);
    let global = extrude("pockets", &plate, &top, sketch.clone(), pocket_of(-3.0)).unwrap();
    assert_eq!(global.hash(), one);
    assert_eq!(global.faces().len(), 6 + 100 * 4);
}

mod random {
    use proptest::prelude::*;

    use super::*;

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(48))]

        /// A single circle strictly inside the top face, a pocket or a boss by the depth's sign.
        #[test]
        fn one_circle(
            d in 2.0..20.0f64, fx in 0.0..1.0f64, fy in 0.0..1.0f64,
            depth in prop_oneof![-9.5..-0.5f64, 0.5..20.0f64],
        ) {
            let r = d / 2.0;
            let (cx, cy) = (r + 2.0 + fx * (96.0 - d), r + 2.0 + fy * (56.0 - d));
            let plate = plate();
            let mode = if depth < 0.0 { "cut" } else { "join" };
            let b = extrude("one", &plate, &face(&plate, up()), circle(cx, cy, d), serde_json::json!({"mode": mode, "z1": depth})).unwrap();
            let expected = 60_000.0 + PI * r * r * depth;
            prop_assert!((b.volume() - expected).abs() <= 1e-9 * expected, "{} vs {expected}", b.volume());
            prop_assert_eq!(b.faces().len(), 6 + 4);
        }
    }
}

/// A tube 60 × 40 × 20 around a hole 20 × 20: a cut through it from its front, wider than the
/// hole, goes through both walls and the side walls between, and a pocket into it beyond its
/// wall (D186, D190: the general Boolean operation where one plate's cut does not do); and a cut
/// through from a sketch on a plane, off the body.
#[test]
fn through_all_of_a_tube() {
    let rect = |x0: f64, y0: f64, x1: f64, y1: f64, k: &str| {
        let p = |i: usize| format!("{k}{i}");
        serde_json::json!({
            "points": {p(0): [x0, y0], p(1): [x1, y0], p(2): [x1, y1], p(3): [x0, y1]},
            "lines": {p(4): {"from": p(0), "to": p(1)}, p(5): {"from": p(1), "to": p(2)}, p(6): {"from": p(2), "to": p(3)}, p(7): {"from": p(3), "to": p(0)}}
        })
    };
    let mut outline = rect(0.0, 0.0, 60.0, 40.0, "o");
    let hole = rect(20.0, 10.0, 40.0, 30.0, "h");
    for key in ["points", "lines"] {
        let more = hole[key].as_object().unwrap().clone();
        outline[key].as_object_mut().unwrap().extend(more);
    }
    let sketch: Op = serde_json::from_value(serde_json::json!({"op": "sketch", "sketch": outline})).unwrap();
    let placed = sketch.eval(id("outline"), &[]).unwrap();
    let op: Op = serde_json::from_value(serde_json::json!({"op": "extrude", "z1": 20})).unwrap();
    let tube = op.eval(id("tube"), &[&placed]).unwrap().body().unwrap().clone();
    assert_close(tube.volume(), 2000.0 * 20.0);
    // On the front (−Y) the sketch's X runs along X, its Y along Z.
    let front = face(&tube, Vec3::new(0.0, -1.0, 0.0));
    let through = serde_json::json!({"mode": "cut", "through": true});
    let cut = extrude("window", &tube, &front, rect(15.0, 5.0, 45.0, 15.0, "w"), through.clone()).unwrap();
    // 10 high: 20 wide through both walls (10 each), 5 + 5 wide through the 40 of the sides.
    assert_close(cut.volume(), 40_000.0 - 10.0 * (20.0 * 20.0 + 10.0 * 40.0));
    // A pocket 20 deep from the front: through its wall into the hole, and 20 into the sides.
    let pocket = extrude("pocket", &tube, &front, rect(15.0, 5.0, 45.0, 15.0, "w"), pocket_of(-20.0)).unwrap();
    assert_close(pocket.volume(), 40_000.0 - 10.0 * (20.0 * 10.0 + 10.0 * 20.0));
    // Deeper than the tube: through, as a cut deeper than a plate is.
    let deep = extrude("deep", &tube, &front, rect(15.0, 5.0, 45.0, 15.0, "w"), pocket_of(-50.0)).unwrap();
    assert_close(deep.volume(), cut.volume());
    // A sketch on the XY plane, off the body: through it along Z.
    let s: Op = serde_json::from_value(serde_json::json!({"op": "sketch", "sketch": rect(5.0, 5.0, 15.0, 35.0, "p")})).unwrap();
    let placed = s.eval(id("slot"), &[]).unwrap();
    let op: Op = serde_json::from_value(serde_json::json!({"op": "extrude", "mode": "cut", "through": true})).unwrap();
    let slot = op.eval(id("slot_cut"), &[&Value::Body(tube.clone()), &placed]).unwrap().body().unwrap().clone();
    slot.validate().unwrap();
    assert_close(slot.volume(), 40_000.0 - 10.0 * 30.0 * 20.0);
}
