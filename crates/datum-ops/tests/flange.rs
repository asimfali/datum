//! Flanges (M8, D205): a bend and a wall on an end of sheet metal: volume and blank against
//! formulas, the side, refusals, IDs, and a cut across the flange's bend.
#![allow(clippy::unwrap_used, clippy::panic)]

use std::collections::BTreeSet;

use datum_brep::{Body, BrepError, Surface, TopoId, tessellate};
use datum_math::Vec3;
use datum_ops::{Op, Value, flange_edges, flange_ends, unfold_side};

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

/// A U of sheet metal: up 60 from the origin, along X by `mid`, down again; on the left of its path.
fn u_of(mid: f64) -> Body {
    let path = serde_json::json!({"points": {"a": [0, 0], "b": [0, 60], "c": [mid, 60], "d": [mid, 0]},
        "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}, "cd": {"from": "c", "to": "d"}}});
    let p = op(serde_json::json!({"op": "sketch", "sketch": path})).eval(id("path"), &[]).unwrap();
    let s = op(serde_json::json!({"op": "sheet", "thickness": T, "radius": R, "side": "left", "z0": 0, "z1": WIDTH}));
    s.eval(id("sheet"), &[&p]).unwrap().body().unwrap().clone()
}

fn u_sheet() -> Body {
    u_of(20.0)
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

/// The IDs of the long edges of the end face `face` of `b` (the first of `ends`, say).
fn long_edges(b: &Body, face: &str) -> Vec<String> {
    let f = (0..b.faces().len()).find(|&x| hex(b, x) == face).unwrap();
    let len = |h: u32| {
        let d = b.point(b.half_edges()[b.next(h) as usize].origin) - b.point(b.half_edges()[h as usize].origin);
        d.dot(d)
    };
    let halves: Vec<u32> = b.loop_half_edges(b.faces()[f].first_loop).collect();
    let longest = halves.iter().map(|&h| len(h)).fold(0.0, f64::max);
    let ids: Vec<String> =
        halves.iter().filter(|&&h| len(h) == longest).map(|&h| format!("{:?}", b.edges()[b.half_edges()[h as usize].edge as usize].id)).collect();
    assert_eq!(ids.len(), 2);
    ids
}

/// The long edges of all `faces` of `b`.
fn ends_edges(b: &Body, faces: &[String]) -> Vec<String> {
    faces.iter().flat_map(|f| long_edges(b, f)).collect()
}

/// The edge of the flat side face at `z` of `b` that lies on `axis` (0 for x, 1 for y) = `at`.
fn wall_edge(b: &Body, axis: usize, at: f64, z: f64) -> String {
    let c = |p: Vec3| [p.x, p.y, p.z];
    let found: Vec<String> = datum_ops::flange_edges(b)
        .into_iter()
        .filter(|(_, e)| e.iter().all(|&p| (c(p)[axis] - at).abs() < 1e-9 && (p.z - z).abs() < 1e-9))
        .map(|e| format!("{:?}", e.0))
        .collect();
    assert_eq!(found.len(), 1, "axis {axis} at {at}, z {z}");
    found[0].clone()
}

/// Flanges on `edges` of `b`, with `extra` fields of the operation; the result checked.
fn flange(b: &Body, edges: &[String], extra: serde_json::Value) -> Result<Body, BrepError> {
    let mut j = serde_json::json!({"op": "flange", "edges": edges, "height": 20, "radius": R});
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

/// A flange is picked by an edge: an end's two long edges (the width long, along Z), or a wall's
/// edge on a flat side face of the sheet. The U has three walls and two caps (z = 0 and z = 40),
/// and a wall has two edges (inner and outer) on each: 3 · 2 · 2 = 12; the two ends add 2 · 2 = 4.
#[test]
fn a_flange_is_picked_by_an_edge_of_an_end_or_of_a_wall() {
    let u = u_sheet();
    let edges = flange_edges(&u);
    assert_eq!(edges.len(), 16);
    let along_z = |e: &(TopoId, [Vec3; 2])| (e.1[0].x == e.1[1].x && e.1[0].y == e.1[1].y).then(|| (e.1[1].z - e.1[0].z).abs());
    let ids: BTreeSet<String> = edges.iter().map(|e| format!("{:?}", e.0)).collect();
    assert_eq!(ids.len(), 16);
    // The ends' long edges: both of each end face, and no others along Z.
    let ends = ends(&u);
    let long: BTreeSet<String> = ends_edges(&u, &ends).into_iter().collect();
    assert_eq!(long.len(), 4);
    for e in &edges {
        assert_eq!(long.contains(&format!("{:?}", e.0)), along_z(e).is_some(), "{e:?}");
        if let Some(l) = along_z(e) {
            assert!((l - WIDTH).abs() < 1e-9 * WIDTH, "{l}");
        }
    }
    // The walls' edges: 6 on each cap, none on a bend (they run in the walls' planes).
    for z in [0.0, WIDTH] {
        let n = edges.iter().filter(|e| e.1.iter().all(|p| p.z == z)).count();
        assert_eq!(n, 6, "z {z}");
    }
    // Each wall: its inner and outer edge on z = 0 are there (wall 1 at x = 0 and −2, the top at y = 60 and 62).
    for (axis, at) in [(0, 0.0), (0, -2.0), (1, 60.0), (1, 62.0), (0, 20.0), (0, 22.0)] {
        wall_edge(&u, axis, at, 0.0);
    }
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
    let b = flange(&u, &long_edges(&u, &ends(&u)[0]), serde_json::json!({})).unwrap();
    assert!(close(b.volume() - u.volume(), added(1.0, WIDTH, 20.0, 90.0)), "{}", b.volume() - u.volume());
    let (size, counts, bends) = blank(&b);
    assert!(close(size.0, base.0 + longer(20.0, 90.0)) && close(size.1, WIDTH), "{size:?}");
    assert_eq!((counts, bends), ([4, 0, 0], 6));
}

#[test]
fn a_flange_on_both_ends() {
    let u = u_sheet();
    let (base, ..) = blank(&u);
    let b = flange(&u, &ends_edges(&u, &ends(&u)), serde_json::json!({})).unwrap();
    assert!(close(b.volume() - u.volume(), added(2.0, WIDTH, 20.0, 90.0)));
    let (size, counts, bends) = blank(&b);
    assert!(close(size.0, base.0 + 2.0 * longer(20.0, 90.0)) && close(size.1, WIDTH), "{size:?}");
    assert_eq!((counts, bends), ([4, 0, 0], 8));
}

#[test]
fn a_flange_turns_to_the_front_or_the_other_side() {
    let u = u_sheet();
    let e = &long_edges(&u, &ends(&u)[0]);
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
    let b = flange(&u, &long_edges(&u, &ends(&u)[0]), serde_json::json!({"angle": 45})).unwrap();
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
    let one = flange(&p, &long_edges(&p, &long[0]), serde_json::json!({})).unwrap();
    assert!(close(one.volume() - p.volume(), added(1.0, 100.0, 20.0, 90.0)));
    let (size, counts, bends) = blank(&one);
    assert!(close(size.0, 60.0 + longer(20.0, 90.0)) && close(size.1, 100.0), "{size:?}");
    assert_eq!((counts, bends), ([4, 0, 0], 2));
    let two = flange(&p, &ends_edges(&p, &long), serde_json::json!({})).unwrap();
    assert!(close(two.volume() - p.volume(), added(2.0, 100.0, 20.0, 90.0)));
    let (size, _, bends) = blank(&two);
    assert!(close(size.0, 60.0 + 2.0 * longer(20.0, 90.0)) && close(size.1, 100.0), "{size:?}");
    assert_eq!(bends, 4);
}

#[test]
fn refusals() {
    let u = u_sheet();
    let e = &long_edges(&u, &ends(&u)[0]);
    for angle in [0, 180] {
        assert!(flange(&u, e, serde_json::json!({"angle": angle})).is_err(), "angle {angle}");
    }
    // The bend sets the wall back by 4: 3 is lower.
    assert!(matches!(flange(&u, e, serde_json::json!({"height": 3})), Err(BrepError::Degenerate(_))));
    // As high as its bend at a right angle: no wall left, refused rather than a sliver.
    assert!(matches!(flange(&u, e, serde_json::json!({"height": 4})), Err(BrepError::Degenerate(_))));
    assert!(flange(&u, e, serde_json::json!({"height": 5})).is_ok());
    // An edge of a face that is no wall (a short edge of an end) is no pick.
    let end = (0..u.faces().len()).find(|&x| hex(&u, x) == ends(&u)[0]).unwrap();
    let short = u
        .loop_half_edges(u.faces()[end].first_loop)
        .map(|h| format!("{:?}", u.edges()[u.half_edges()[h as usize].edge as usize].id))
        .find(|i| !e.contains(i))
        .unwrap();
    assert!(flange(&u, &[short], serde_json::json!({})).is_err());
    // Ends meeting at a corner, or one end twice.
    let p = plate();
    let (long, short) = (faces_of_area(&p, 100.0 * T), faces_of_area(&p, 60.0 * T));
    let corner = [long_edges(&p, &long[0]), long_edges(&p, &short[0])].concat();
    let err = flange(&p, &corner, serde_json::json!({})).unwrap_err();
    assert!(matches!(err, BrepError::Unsupported(ref m) if m.contains("corner")), "{err}");
    // One end by its two edges, or by one edge twice, is one flange.
    let one = flange(&p, &long_edges(&p, &long[0]), serde_json::json!({})).unwrap();
    let twice = flange(&p, &vec![long_edges(&p, &long[0])[0].clone(); 2], serde_json::json!({})).unwrap();
    assert!(close(twice.volume(), one.volume()));
}

#[test]
fn ids_do_not_depend_on_the_height() {
    let u = u_sheet();
    let e = ends_edges(&u, &ends(&u));
    let ids = |h: u32| sorted_ids(&flange(&u, &e, serde_json::json!({"height": h})).unwrap());
    assert_eq!(ids(20), ids(30));
    assert!(ids(20).windows(2).all(|w| w[0] < w[1]));
}

/// A circle of Ø10 across the flange's bend, cut from the flange's wall (`inner` or outer face).
#[test]
fn a_hole_across_a_flanges_bend() {
    for inner in [false, true] {
        let u = u_sheet();
        let f = flange(&u, &long_edges(&u, &ends(&u)[0]), serde_json::json!({})).unwrap();
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

/// Side flanges (D205): on a U with a top 40 long; edges on the cap z = 0 (and z = 40).
const MID: f64 = 40.0;
const SIDE: f64 = 36.0; // the top's straight part: 40 less the two bends' set-back (2 · 2)
const LEG: f64 = 58.0; // a leg's straight part: 60 less the bend's set-back (2), to the sheet's end

fn side_of(b: &Body, edges: &[String], extra: serde_json::Value) -> Body {
    flange(b, edges, extra).unwrap()
}

/// The side flange's blank: Y grows by the unrolled length of each; X stays.
fn base_blank(u: &Body) -> (f64, f64) {
    blank(u).0
}

#[test]
fn a_side_flange_on_the_top() {
    let u = u_of(MID);
    let b = side_of(&u, &[wall_edge(&u, 1, 62.0, 0.0)], serde_json::json!({}));
    assert!(close(b.volume() - u.volume(), added(1.0, SIDE, 20.0, 90.0)), "{}", b.volume() - u.volume());
    let (size, counts, bends) = blank(&b);
    let base = base_blank(&u);
    assert!(close(size.0, base.0) && close(size.1, WIDTH + longer(20.0, 90.0)), "{size:?}");
    assert_eq!((counts, bends), ([8, 0, 0], 6));
}

#[test]
fn a_side_flange_on_a_leg_goes_outward() {
    let u = u_of(MID);
    let e = wall_edge(&u, 0, -2.0, 0.0);
    let b = side_of(&u, std::slice::from_ref(&e), serde_json::json!({}));
    assert!(close(b.volume() - u.volume(), added(1.0, LEG, 20.0, 90.0)), "{}", b.volume() - u.volume());
    let (size, counts, bends) = blank(&b);
    assert!(close(size.0, base_blank(&u).0) && close(size.1, WIDTH + longer(20.0, 90.0)), "{size:?}");
    assert_eq!((counts, bends), ([6, 0, 0], 6));
    // Outward: the leg's outer face is at x = −2, the flange reaches x = −20 (2 + 16 + 2 out of the wall).
    let (lo, _) = b.bounds();
    assert!(close(lo.x.abs(), 20.0), "{lo:?}");
    // Flipped: inward, the sheet's bounds stay.
    let f = side_of(&u, &[e], serde_json::json!({"flip": true}));
    assert!(close(f.volume() - u.volume(), added(1.0, LEG, 20.0, 90.0)));
    let (lo, _) = f.bounds();
    assert!(close(lo.x.abs(), 2.0), "{lo:?}");
    assert_eq!(blank(&f), blank(&b));
}

/// The inner edge of a wall makes the same flange as its outer one.
#[test]
fn both_edges_of_a_walls_side_make_one_flange() {
    let u = u_of(MID);
    let one = side_of(&u, &[wall_edge(&u, 1, 62.0, 0.0)], serde_json::json!({}));
    let inner = side_of(&u, &[wall_edge(&u, 1, 60.0, 0.0)], serde_json::json!({}));
    let both = side_of(&u, &[wall_edge(&u, 1, 60.0, 0.0), wall_edge(&u, 1, 62.0, 0.0)], serde_json::json!({}));
    assert!(close(inner.volume(), one.volume()) && close(both.volume(), one.volume()));
    // Whichever edge is picked, the flange's faces are named alike.
    assert_eq!(sorted_ids(&both), sorted_ids(&inner));
    assert_eq!(sorted_ids(&one), sorted_ids(&inner));
}

#[test]
fn several_side_flanges_in_one_operation() {
    let u = u_of(MID);
    let edges = [wall_edge(&u, 0, -2.0, 0.0), wall_edge(&u, 1, 62.0, 0.0), wall_edge(&u, 0, 42.0, 0.0)];
    let b = side_of(&u, &edges, serde_json::json!({}));
    assert!(close(b.volume() - u.volume(), added(2.0, LEG, 20.0, 90.0) + added(1.0, SIDE, 20.0, 90.0)), "{}", b.volume() - u.volume());
    let (size, counts, bends) = blank(&b);
    assert!(close(size.0, base_blank(&u).0) && close(size.1, WIDTH + longer(20.0, 90.0)), "{size:?}");
    assert_eq!((counts, bends), ([12, 0, 0], 10));
    let (lo, hi) = b.bounds();
    assert!(close(lo.x.abs(), 20.0) && close(hi.x, MID + 20.0) && close(hi.y, 80.0), "{lo:?} {hi:?}");
}

/// A wall flanged on both sides of the sheet's width (z = 0 and z = 40): the blank grows twice.
#[test]
fn a_wall_with_a_side_flange_on_each_side() {
    let u = u_of(MID);
    let edges = [wall_edge(&u, 0, -2.0, 0.0), wall_edge(&u, 0, -2.0, WIDTH)];
    let b = side_of(&u, &edges, serde_json::json!({}));
    assert!(close(b.volume() - u.volume(), added(2.0, LEG, 20.0, 90.0)), "{}", b.volume() - u.volume());
    let (size, counts, bends) = blank(&b);
    assert!(close(size.0, base_blank(&u).0) && close(size.1, WIDTH + 2.0 * longer(20.0, 90.0)), "{size:?}");
    assert_eq!((counts, bends), ([8, 0, 0], 8));
}

/// A flange on an end of the U with mid 40 for comparison with the side flanges.
#[test]
fn an_end_flange_on_a_long_u() {
    let u = u_of(MID);
    let b = flange(&u, &long_edges(&u, &ends(&u)[0]), serde_json::json!({})).unwrap();
    assert!(close(b.volume() - u.volume(), added(1.0, WIDTH, 20.0, 90.0)));
    let (size, counts, bends) = blank(&b);
    assert!(close(size.0, base_blank(&u).0 + longer(20.0, 90.0)) && close(size.1, WIDTH), "{size:?}");
    assert_eq!((counts, bends), ([4, 0, 0], 6));
}

/// A side flange along a leg and an end flange at the leg's end meet at a corner.
#[test]
fn a_side_flange_and_an_end_flange_at_a_corner_are_refused() {
    let u = u_of(MID);
    // The end at the origin: the one whose face lies at y = 0 and x from −2 to 0.
    let end = ends(&u)
        .into_iter()
        .find(|f| {
            let x = (0..u.faces().len()).find(|&x| hex(&u, x) == *f).unwrap();
            centroid(&u, x).x < 1.0
        })
        .unwrap();
    let mut edges = long_edges(&u, &end);
    edges.push(wall_edge(&u, 0, -2.0, 0.0));
    let err = flange(&u, &edges, serde_json::json!({})).unwrap_err();
    assert!(matches!(err, BrepError::Unsupported(ref m) if m.contains("corner")), "{err}");
}

#[test]
fn side_flange_ids_do_not_depend_on_the_height() {
    let u = u_of(MID);
    let e = [wall_edge(&u, 1, 62.0, 0.0)];
    let ids = |h: u32| sorted_ids(&flange(&u, &e, serde_json::json!({"height": h})).unwrap());
    assert_eq!(ids(20), ids(30));
    assert!(ids(20).windows(2).all(|w| w[0] < w[1]));
}

/// A hole well inside the top wall, beside a side flange on it, cuts as without the flange: one
/// circle in the blank. (The hole is on the leg, which has no flange: the cut reaches none.)
#[test]
fn a_hole_beside_a_side_flange_still_cuts() {
    let u = u_of(MID);
    let b = side_of(&u, &[wall_edge(&u, 1, 62.0, 0.0)], serde_json::json!({}));
    let target = Value::Body(b.clone());
    let first = b.faces().iter().find(|f| matches!(f.surface, Surface::Plane(p) if p.normal.x < -0.5 && p.origin.x <= 0.0)).unwrap();
    let sketch = serde_json::json!({"points": {"h": [30, -20]}, "circles": {"c": {"center": "h", "diameter": 6}}});
    let placed = op(serde_json::json!({"op": "sketch", "face": format!("{:?}", first.id), "k_factor": K, "sketch": sketch}))
        .eval(id("hole"), &[&target])
        .unwrap();
    let cut = op(serde_json::json!({"op": "extrude", "mode": "cut", "through": true, "wall": true}));
    let r = cut.eval(id("cut"), &[&target, &placed]).unwrap().body().unwrap().clone();
    r.validate().unwrap();
    tessellate(&r).unwrap();
    assert!(r.volume() < b.volume());
    let (_, counts, bends) = blank(&r);
    assert_eq!((counts[2], bends), (1, 6));
}

/// A hole in a side flange's wall, on its face: cut through the flange's thickness, and the blank
/// has it, laid with the flange (D205).
#[test]
fn a_hole_in_a_side_flange() {
    let u = u_of(MID);
    let b = side_of(&u, &[wall_edge(&u, 1, 62.0, 0.0)], serde_json::json!({}));
    let target = Value::Body(b.clone());
    let old: Vec<_> = u.faces().iter().map(|f| f.id).collect();
    // The flange's wall: its largest new flat face.
    let (x, wall) = (0..b.faces().len())
        .filter(|&x| !old.contains(&b.faces()[x].id) && matches!(b.faces()[x].surface, Surface::Plane(_)))
        .map(|x| (x, b.faces()[x].id))
        .max_by(|p, q| b.face_area(p.0 as u32).total_cmp(&b.face_area(q.0 as u32)))
        .unwrap();
    let middle = b.loop_points(b.faces()[x].first_loop).fold(datum_math::Vec3::ZERO, |m, p| m + p) * 0.25;
    let on_face = |at: [f64; 2]| {
        let sketch = serde_json::json!({"points": {"h": at}, "circles": {"c": {"center": "h", "diameter": 6}}});
        op(serde_json::json!({"op": "sketch", "face": format!("{wall:?}"), "k_factor": K, "sketch": sketch})).eval(id("hole"), &[&target]).unwrap()
    };
    // The hole at the wall's middle, in the sketch's own axes.
    let [o, ax, ay, _] = on_face([0.0, 0.0]).sketch().unwrap().frame;
    let placed = on_face([(middle - o).dot(ax), (middle - o).dot(ay)]);
    let cut = op(serde_json::json!({"op": "extrude", "mode": "cut", "through": true, "wall": true}));
    let r = cut.eval(id("cut"), &[&target, &placed]).unwrap().body().unwrap().clone();
    r.validate().unwrap();
    tessellate(&r).unwrap();
    assert!(close(b.volume() - r.volume(), std::f64::consts::PI * 9.0 * T), "{}", b.volume() - r.volume());
    let (_, counts, _) = blank(&r);
    assert_eq!(counts[2], 1);
}

/// A hole on a leg across the edge where its side flange stands reaches the flange: refused.
#[test]
fn a_hole_reaching_a_side_flange_is_refused() {
    let u = u_of(MID);
    let b = side_of(&u, &[wall_edge(&u, 0, -2.0, 0.0)], serde_json::json!({}));
    let target = Value::Body(b.clone());
    let first = b.faces().iter().find(|f| matches!(f.surface, Surface::Plane(p) if p.normal.x < -0.5 && p.origin.x <= 0.0)).unwrap();
    // The sketch's Y is −z: the circle spans z from −1 to 5.
    let sketch = serde_json::json!({"points": {"h": [30, -2]}, "circles": {"c": {"center": "h", "diameter": 6}}});
    let placed = op(serde_json::json!({"op": "sketch", "face": format!("{:?}", first.id), "k_factor": K, "sketch": sketch}))
        .eval(id("hole"), &[&target])
        .unwrap();
    let cut = op(serde_json::json!({"op": "extrude", "mode": "cut", "through": true, "wall": true}));
    let r = cut.eval(id("cut"), &[&target, &placed]);
    assert!(matches!(r, Err(BrepError::Unsupported(ref m)) if m.contains("flange")), "{:?}", r.map(|_| ()));
}
