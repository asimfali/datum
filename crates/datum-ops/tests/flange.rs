//! Flanges (M8, D205): a bend and a wall on an end of sheet metal: volume and blank against
//! formulas, the side, refusals, IDs, and a cut across the flange's bend.
#![allow(clippy::unwrap_used, clippy::panic)]

use std::collections::BTreeSet;

use datum_brep::{Body, BrepError, Surface, TopoId, tessellate};
use datum_math::Vec3;
use datum_ops::{Op, Value, flange_ends, unfold_side};

const K: f64 = 0.44;
const T: f64 = 2.0;
const R: f64 = 2.0;
const WIDTH: f64 = 40.0;

fn id(tag: &str) -> TopoId {
    TopoId::derive(tag).finish()
}

fn op(json: serde_json::Value) -> Op {
    serde_json::from_value(json).unwrap()
}

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-9 * b.abs()
}

/// A U of sheet metal: up 60 from the origin, along X by 20, down again; on the left of its path.
fn u_sheet() -> Body {
    let path = serde_json::json!({"points": {"a": [0, 0], "b": [0, 60], "c": [20, 60], "d": [20, 0]},
        "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}, "cd": {"from": "c", "to": "d"}}});
    let p = op(serde_json::json!({"op": "sketch", "sketch": path})).eval(id("path"), &[]).unwrap();
    let s = op(serde_json::json!({"op": "sheet", "thickness": T, "radius": R, "side": "left", "z0": 0, "z1": WIDTH}));
    s.eval(id("sheet"), &[&p]).unwrap().body().unwrap().clone()
}

/// A plate of 100 × 60 and 2 thick.
fn plate() -> Body {
    let rect = serde_json::json!({"points": {"a": [0, 0], "b": [100, 0], "c": [100, 60], "d": [0, 60]},
        "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}, "cd": {"from": "c", "to": "d"}, "da": {"from": "d", "to": "a"}}});
    let s = op(serde_json::json!({"op": "sketch", "sketch": rect})).eval(id("plate.sketch"), &[]).unwrap();
    op(serde_json::json!({"op": "extrude", "z0": 0, "z1": T})).eval(id("plate"), &[&s]).unwrap().body().unwrap().clone()
}

fn hex(b: &Body, x: usize) -> String {
    format!("{:?}", b.faces()[x].id)
}

/// The IDs of the faces of `b` across the thickness: planes of the area `area`.
fn faces_of_area(b: &Body, area: f64) -> Vec<String> {
    (0..b.faces().len())
        .filter(|&x| matches!(b.faces()[x].surface, Surface::Plane(p) if p.normal.z == 0.0) && close(b.face_area(x as u32), area))
        .map(|x| hex(b, x))
        .collect()
}

fn ends(b: &Body) -> Vec<String> {
    faces_of_area(b, WIDTH * T)
}

/// Flanges on `faces` of `b`, with `extra` fields of the operation; the result checked.
fn flange(b: &Body, faces: &[String], extra: serde_json::Value) -> Result<Body, BrepError> {
    let mut j = serde_json::json!({"op": "flange", "faces": faces, "height": 20, "radius": R});
    for (k, v) in extra.as_object().unwrap() {
        j[k] = v.clone();
    }
    let r = op(j).eval(id("flange"), &[&Value::Body(b.clone())])?.body().unwrap().clone();
    r.validate().unwrap();
    tessellate(&r).unwrap();
    Ok(r)
}

/// The volume of `n` flanges of `width` high `h` turning by `a` degrees.
fn added(n: f64, width: f64, h: f64, a: f64) -> f64 {
    let (a, big) = (a.to_radians(), R + T);
    n * width * (a / 2.0 * (big * big - R * R) + (h - big * libm::tan(a / 2.0)) * T)
}

/// What a flange of `h` turning by `a` degrees adds to the blank's length: its wall and its bend.
fn longer(h: f64, a: f64) -> f64 {
    let a = a.to_radians();
    h - (R + T) * libm::tan(a / 2.0) + a * (R + K * T)
}

fn blank(b: &Body) -> ((f64, f64), [usize; 3], usize) {
    let f = unfold_side(b, false, K).unwrap();
    ((f.size.x, f.size.y), f.counts(), f.bends.len())
}

fn centroid(b: &Body, x: usize) -> Vec3 {
    let p: Vec<Vec3> = b.loop_half_edges(b.faces()[x].first_loop).map(|h| b.point(b.half_edges()[h as usize].origin)).collect();
    p.iter().fold(Vec3::ZERO, |s, q| s + *q) * (1.0 / p.len() as f64)
}

/// The faces of `after` that `before` does not have.
fn new_faces(before: &Body, after: &Body) -> Vec<usize> {
    let old: BTreeSet<TopoId> = before.faces().iter().map(|f| f.id).collect();
    (0..after.faces().len()).filter(|&x| !old.contains(&after.faces()[x].id)).collect()
}

/// The new end face of a flange: the new plane of the area of the sheet's cross-section.
fn new_end(before: &Body, after: &Body) -> usize {
    let ends: Vec<usize> = new_faces(before, after)
        .into_iter()
        .filter(|&x| matches!(after.faces()[x].surface, Surface::Plane(_)) && close(after.face_area(x as u32), WIDTH * T))
        .collect();
    assert_eq!(ends.len(), 1);
    ends[0]
}

fn sorted_ids(b: &Body) -> Vec<TopoId> {
    b.faces().iter().map(|f| f.id).collect()
}

#[test]
fn ends_of_a_u_are_its_two_ends() {
    let u = u_sheet();
    let found: BTreeSet<String> = flange_ends(&u).iter().map(|i| format!("{i:?}")).collect();
    assert_eq!(found, ends(&u).into_iter().collect());
    assert_eq!(found.len(), 2);
}

#[test]
fn ends_of_a_plate_are_its_four_sides() {
    let p = plate();
    let found: BTreeSet<String> = flange_ends(&p).iter().map(|i| format!("{i:?}")).collect();
    let sides: BTreeSet<String> =
        (0..p.faces().len()).filter(|&x| matches!(p.faces()[x].surface, Surface::Plane(s) if s.normal.z == 0.0)).map(|x| hex(&p, x)).collect();
    assert_eq!(found, sides);
    assert_eq!(found.len(), 4);
}

#[test]
fn one_flange_on_a_u() {
    let u = u_sheet();
    let (base, ..) = blank(&u);
    let b = flange(&u, &ends(&u)[..1], serde_json::json!({})).unwrap();
    assert!(close(b.volume() - u.volume(), added(1.0, WIDTH, 20.0, 90.0)), "{}", b.volume() - u.volume());
    let (size, counts, bends) = blank(&b);
    assert!(close(size.0, base.0 + longer(20.0, 90.0)) && close(size.1, WIDTH), "{size:?}");
    assert_eq!((counts, bends), ([4, 0, 0], 6));
}

#[test]
fn a_flange_on_both_ends() {
    let u = u_sheet();
    let (base, ..) = blank(&u);
    let b = flange(&u, &ends(&u), serde_json::json!({})).unwrap();
    assert!(close(b.volume() - u.volume(), added(2.0, WIDTH, 20.0, 90.0)));
    let (size, counts, bends) = blank(&b);
    assert!(close(size.0, base.0 + 2.0 * longer(20.0, 90.0)) && close(size.1, WIDTH), "{size:?}");
    assert_eq!((counts, bends), ([4, 0, 0], 8));
}

#[test]
fn a_flange_turns_to_the_front_or_the_other_side() {
    let u = u_sheet();
    let e = &ends(&u)[..1];
    let (a, b) = (flange(&u, e, serde_json::json!({})).unwrap(), flange(&u, e, serde_json::json!({"flip": true})).unwrap());
    assert!(close(b.volume(), a.volume()));
    assert_eq!(blank(&a), blank(&b));
    // The leg at the end lies along Y at x from −2 to 0; the flange's end is across it, either way.
    let (xa, xb) = (centroid(&a, new_end(&u, &a)).x + 1.0, centroid(&b, new_end(&u, &b)).x + 1.0);
    assert!(xa * xb < 0.0, "{xa} {xb}");
    assert!(close(xa.abs(), xb.abs()), "{xa} {xb}");
}

#[test]
fn a_flange_of_45_degrees() {
    let u = u_sheet();
    let (base, ..) = blank(&u);
    let b = flange(&u, &ends(&u)[..1], serde_json::json!({"angle": 45})).unwrap();
    assert!(close(b.volume() - u.volume(), added(1.0, WIDTH, 20.0, 45.0)), "{}", b.volume() - u.volume());
    let (size, counts, bends) = blank(&b);
    assert!(close(size.0, base.0 + longer(20.0, 45.0)) && close(size.1, WIDTH), "{size:?}");
    assert_eq!((counts, bends), ([4, 0, 0], 6));
}

#[test]
fn a_flange_on_a_plate() {
    let p = plate();
    let long: Vec<String> = faces_of_area(&p, 100.0 * T);
    assert_eq!(long.len(), 2);
    let one = flange(&p, &long[..1], serde_json::json!({})).unwrap();
    assert!(close(one.volume() - p.volume(), added(1.0, 100.0, 20.0, 90.0)));
    let (size, counts, bends) = blank(&one);
    assert!(close(size.0, 60.0 + longer(20.0, 90.0)) && close(size.1, 100.0), "{size:?}");
    assert_eq!((counts, bends), ([4, 0, 0], 2));
    let two = flange(&p, &long, serde_json::json!({})).unwrap();
    assert!(close(two.volume() - p.volume(), added(2.0, 100.0, 20.0, 90.0)));
    let (size, _, bends) = blank(&two);
    assert!(close(size.0, 60.0 + 2.0 * longer(20.0, 90.0)) && close(size.1, 100.0), "{size:?}");
    assert_eq!(bends, 4);
}

#[test]
fn refusals() {
    let u = u_sheet();
    let e = &ends(&u)[..1];
    for angle in [0, 180] {
        assert!(flange(&u, e, serde_json::json!({"angle": angle})).is_err(), "angle {angle}");
    }
    // The bend sets the wall back by 4: 3 is lower.
    assert!(matches!(flange(&u, e, serde_json::json!({"height": 3})), Err(BrepError::Degenerate(_))));
    // As high as its bend at a right angle: no wall left, refused rather than a sliver.
    assert!(matches!(flange(&u, e, serde_json::json!({"height": 4})), Err(BrepError::Degenerate(_))));
    assert!(flange(&u, e, serde_json::json!({"height": 5})).is_ok());
    // A cap of the sheet is no end.
    let cap = (0..u.faces().len()).find(|&x| matches!(u.faces()[x].surface, Surface::Plane(p) if p.normal.z > 0.5)).unwrap();
    assert!(flange(&u, &[hex(&u, cap)], serde_json::json!({})).is_err());
    // Ends meeting at a corner, or one end twice.
    let p = plate();
    let (long, short) = (faces_of_area(&p, 100.0 * T), faces_of_area(&p, 60.0 * T));
    let err = flange(&p, &[long[0].clone(), short[0].clone()], serde_json::json!({})).unwrap_err();
    assert!(matches!(err, BrepError::Unsupported(ref m) if m.contains("corner")), "{err}");
    assert!(matches!(flange(&p, &[long[0].clone(), long[0].clone()], serde_json::json!({})), Err(BrepError::Unsupported(_))));
}

#[test]
fn ids_do_not_depend_on_the_height() {
    let u = u_sheet();
    let e = ends(&u);
    let ids = |h: u32| sorted_ids(&flange(&u, &e, serde_json::json!({"height": h})).unwrap());
    assert_eq!(ids(20), ids(30));
    assert!(ids(20).windows(2).all(|w| w[0] < w[1]));
}

/// A circle of Ø10 across the flange's bend, cut from the flange's wall (`inner` or outer face).
#[test]
fn a_hole_across_a_flanges_bend() {
    for inner in [false, true] {
        let u = u_sheet();
        let f = flange(&u, &ends(&u)[..1], serde_json::json!({})).unwrap();
        let end = new_end(&u, &f);
        // The wall's faces: the new planes of 16 × 40; the inner one is nearer the leg's inner side.
        let walls: Vec<usize> = new_faces(&u, &f)
            .into_iter()
            .filter(|&x| matches!(f.faces()[x].surface, Surface::Plane(_)) && close(f.face_area(x as u32), 16.0 * WIDTH))
            .collect();
        assert_eq!(walls.len(), 2);
        // The flange lies beyond the leg's end (y = 0): the inner wall is the one nearer the leg.
        let nearer = (centroid(&f, walls[0]).y > centroid(&f, walls[1]).y) == inner;
        let wall = if nearer { walls[0] } else { walls[1] };
        // Its sketch axes; the hole 2 from the edge of the bend, in the middle of the width.
        let plane = datum_ops::face_plane(&f, &hex(&f, wall)).unwrap();
        let [o, x, y, _] = plane.axes().unwrap();
        let uv = |p: Vec3| ((p - o).dot(x), (p - o).dot(y));
        let pts: Vec<(f64, f64)> = f.loop_half_edges(f.faces()[wall].first_loop).map(|h| uv(f.point(f.half_edges()[h as usize].origin))).collect();
        let (lo, hi) = pts
            .iter()
            .fold(((f64::MAX, f64::MAX), (f64::MIN, f64::MIN)), |s, p| ((s.0.0.min(p.0), s.0.1.min(p.1)), (s.1.0.max(p.0), s.1.1.max(p.1))));
        let e = uv(centroid(&f, end));
        let mid = (0.5 * (lo.0 + hi.0), 0.5 * (lo.1 + hi.1));
        // The wall is 16 long, along the axis of the smaller extent; the bend is at its end away from the flange's end.
        let c = if hi.0 - lo.0 < hi.1 - lo.1 {
            (if e.0 > mid.0 { lo.0 + 2.0 } else { hi.0 - 2.0 }, mid.1)
        } else {
            (mid.0, if e.1 > mid.1 { lo.1 + 2.0 } else { hi.1 - 2.0 })
        };
        let hole = serde_json::json!({"points": {"h": [c.0, c.1]}, "circles": {"c": {"center": "h", "diameter": 10}}});
        let target = Value::Body(f.clone());
        let placed =
            op(serde_json::json!({"op": "sketch", "face": hex(&f, wall), "k_factor": K, "sketch": hole})).eval(id("hole"), &[&target]).unwrap();
        let cut = op(serde_json::json!({"op": "extrude", "mode": "cut", "through": true, "wall": true}));
        let r = cut.eval(id("cut"), &[&target, &placed]).unwrap().body().unwrap().clone();
        r.validate().unwrap();
        tessellate(&r).unwrap();
        assert!(r.volume() < f.volume());
        let flat = unfold_side(&r, false, K).unwrap();
        assert_eq!(flat.counts()[2], 1, "inner {inner}: {:?}", flat.counts());
    }
}
