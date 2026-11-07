//! Flanges (M8, D205): a bend and a wall on an end of sheet metal: volume and blank against
//! formulas, the side, refusals, IDs, and a cut across the flange's bend.
#![allow(clippy::unwrap_used, clippy::panic)]

use std::collections::BTreeSet;

use datum_brep::{Body, BrepError, Surface, TopoId, tessellate};
use datum_math::Vec3;
use datum_ops::{Flat, FlatPiece, Op, Value, flange_edges, flange_ends, unfold_side};

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
    let b = flange(&u, &long_edges(&u, &ends(&u)[0]), serde_json::json!({"angle": 45, "height_along": true})).unwrap();
    assert!(close(b.volume() - u.volume(), added(1.0, WIDTH, 20.0, 45.0)), "{}", b.volume() - u.volume());
    let (size, counts, bends) = blank(&b);
    assert!(close(size.0, base.0 + longer(20.0, 45.0)) && close(size.1, WIDTH), "{size:?}");
    assert_eq!((counts, bends), ([4, 0, 0], 6));
}

/// The height square to the sheet (D229): a flange leaning out or in reaches as high as a square
/// one, its highest point `height` above the sheet's outer surface.
#[test]
fn a_flanges_height_is_square_to_the_sheet() {
    let p = plate();
    let long = faces_of_area(&p, 100.0 * T);
    for angle in [90.0, 45.0, 120.0] {
        let b = flange(&p, &long_edges(&p, &long[0]), serde_json::json!({"angle": angle})).unwrap();
        let top = b.vertices().iter().map(|v| v.point.z).fold(f64::NEG_INFINITY, f64::max);
        assert!((top - 20.0).abs() < 1e-12, "{angle}: {top}");
    }
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

/// Flanges on all four ends of a plate: a tray with its corners open (D229), the blank a cross.
#[test]
fn flanges_round_a_plate_leave_the_corners_open() {
    let p = plate();
    let (long, short) = (faces_of_area(&p, 100.0 * T), faces_of_area(&p, 60.0 * T));
    let corner = [long_edges(&p, &long[0]), long_edges(&p, &short[0])].concat();
    let two = flange(&p, &corner, serde_json::json!({})).unwrap();
    assert!(close(two.volume() - p.volume(), added(1.0, 100.0, 20.0, 90.0) + added(1.0, 60.0, 20.0, 90.0)));
    let all = flange(&p, &ends_edges(&p, &[long.clone(), short.clone()].concat()), serde_json::json!({})).unwrap();
    assert!(close(all.volume() - p.volume(), added(2.0, 100.0, 20.0, 90.0) + added(2.0, 60.0, 20.0, 90.0)));
    // A cross: the plate and a flange off each end, the corners notched to the plate's corner.
    let (size, counts, bends) = blank(&all);
    assert!(close(size.0, 60.0 + 2.0 * longer(20.0, 90.0)) && close(size.1, 100.0 + 2.0 * longer(20.0, 90.0)), "{size:?}");
    assert_eq!((counts, bends), ([12, 0, 0], 8));
    // Leaning in over the plate, the walls meet above its corner: built all the same (D233).
    assert!(flange(&p, &corner, serde_json::json!({"angle": 120})).is_ok());
    assert!(flange(&p, &corner, serde_json::json!({"angle": 60})).is_ok());
}

/// A round relief at each corner of a tray (D229): a circle about the plate's corner, where the
/// bend lines meet, cut through the plate and both bends; on the blank one circle each.
#[test]
fn a_round_relief_at_each_corner_of_a_tray() {
    let p = plate();
    let (long, short) = (faces_of_area(&p, 100.0 * T), faces_of_area(&p, 60.0 * T));
    let edges = ends_edges(&p, &[long, short].concat());
    let open = flange(&p, &edges, serde_json::json!({})).unwrap();
    for d in [4.0, 6.0] {
        let round = flange(&p, &edges, serde_json::json!({"relief": "round", "relief_diameter": d, "k_factor": K})).unwrap();
        assert!(round.volume() < open.volume(), "{d}");
        let f = unfold_side(&round, false, K).unwrap();
        // The cross's outline goes round each circle's three quarters on the metal.
        let arcs: Vec<_> =
            f.pieces.iter().filter(|p| matches!(p, datum_ops::FlatPiece::Arc { radius, .. } if (radius - 0.5 * d).abs() < 1e-9)).collect();
        assert!(!arcs.is_empty(), "{:?}", f.pieces);
        let (size, ..) = blank(&round);
        assert!(close(size.0, 60.0 + 2.0 * longer(20.0, 90.0)) && close(size.1, 100.0 + 2.0 * longer(20.0, 90.0)), "{size:?}");
    }
    // Without the rule's diameter and K: refused.
    assert!(flange(&p, &edges, serde_json::json!({"relief": "round"})).is_err());
}

/// The corners of a tray closed (D229): at a right angle overlapping, the covering wall grows to
/// the other's outer face (`r + t`), the other to its inner face less the gap (`r − g`); mitred,
/// both to the plane halfway, cut square to the wall where its inner face meets it: `r` less the
/// gap's share. The bends stay.
#[test]
fn a_trays_corners_closed() {
    let p = plate();
    let (long, short) = (faces_of_area(&p, 100.0 * T), faces_of_area(&p, 60.0 * T));
    let edges = ends_edges(&p, &[long, short].concat());
    let open = flange(&p, &edges, serde_json::json!({})).unwrap();
    let wall = 20.0 - (R + T);
    for g in [0.0, 1.0] {
        let per_corner = |grow: f64| 4.0 * T * wall * grow;
        for (corner, grow) in
            [("overlap_1", (R + T) + (R - g)), ("overlap_2", (R + T) + (R - g)), ("miter", 2.0 * (R - g / std::f64::consts::SQRT_2))]
        {
            let b = flange(&p, &edges, serde_json::json!({"corner": corner, "corner_gap": g})).unwrap();
            let added = b.volume() - open.volume();
            assert!((added - per_corner(grow)).abs() < 1e-9 * b.volume(), "{corner} {g}: {added} vs {}", per_corner(grow));
            let f = unfold_side(&b, false, K).unwrap();
            assert!(f.size.x > 0.0 && f.size.y > 0.0);
        }
    }
    // A closed corner takes the rule's gap.
    assert!(flange(&p, &edges, serde_json::json!({"corner": "miter"})).is_err());
}

/// The author's tray (D229): the long sides flanged first, square; then the ends at another angle,
/// their walls going to the long walls' inner faces, the gap short; the long walls stay.
#[test]
fn ends_flanged_after_the_sides_go_to_their_walls() {
    let p = plate();
    let long = faces_of_area(&p, 100.0 * T);
    let sides = flange(&p, &ends_edges(&p, &long), serde_json::json!({})).unwrap();
    // The ends' edges on the top: at x = 0 and x = 100.
    let edges = vec![wall_edge(&sides, 0, 0.0, T), wall_edge(&sides, 0, 100.0, T)];
    for angle in [90.0, 45.0] {
        let open = flange(&sides, &edges, serde_json::json!({"angle": angle})).unwrap();
        let closed = flange(&sides, &edges, serde_json::json!({"angle": angle, "corner": "overlap_1", "corner_gap": 1})).unwrap();
        assert!(closed.volume() > open.volume(), "{angle}");
        if angle == 90.0 {
            // Each end's wall grows by `r − g` at each corner: to the long walls' inner faces.
            let wall = 20.0 - (R + T);
            assert!(close(closed.volume() - open.volume(), 4.0 * T * wall * (R - 1.0)), "{}", closed.volume() - open.volume());
        }
        let f = unfold_side(&closed, false, K).unwrap();
        assert!(f.size.x > 0.0);
        // The long walls as they were: their faces' areas.
        let walls = |b: &Body| faces_of_area(b, 100.0 * wall_height()).len();
        assert_eq!(walls(&closed), walls(&sides));
    }
}

fn wall_height() -> f64 {
    20.0 - (R + T)
}

/// At an inner corner of an L-shaped plate the flanges would run into each other: refused.
#[test]
fn flanges_at_an_inner_corner_are_built_and_told() {
    let l = serde_json::json!({"points": {"a": [0, 0], "b": [100, 0], "c": [100, 40], "d": [40, 40], "e": [40, 100], "f": [0, 100]},
        "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}, "cd": {"from": "c", "to": "d"},
                  "de": {"from": "d", "to": "e"}, "ef": {"from": "e", "to": "f"}, "fa": {"from": "f", "to": "a"}}});
    let s = op(serde_json::json!({"op": "sketch", "sketch": l})).eval(id("l.sketch"), &[]).unwrap();
    let p = op(serde_json::json!({"op": "extrude", "z0": 0, "z1": T})).eval(id("l"), &[&s]).unwrap().body().unwrap().clone();
    // The ends along cd (y = 40, x 40..100) and de (x = 40, y 40..100): both 60 long.
    let ends: Vec<String> = faces_of_area(&p, 60.0 * T)
        .into_iter()
        .filter(|f| {
            let c = centroid(&p, (0..p.faces().len()).find(|&x| hex(&p, x) == *f).unwrap());
            c.x > 39.0 && c.y > 39.0
        })
        .collect();
    assert_eq!(ends.len(), 2);
    let b = flange(&p, &ends_edges(&p, &ends), serde_json::json!({})).unwrap();
    assert!(!unfold_side(&b, false, K).unwrap().overlaps.is_empty());
}

/// Width extents (D229, as Inventor's): the flange on part of its edge, the end's rest kept; the
/// volume that of a flange as wide, the blank as wide as before (the plate) and the flange's strip.
#[test]
fn a_flange_on_part_of_its_edge() {
    let p = plate();
    let long = faces_of_area(&p, 100.0 * T);
    let edge = long_edges(&p, &long[0]);
    for (extent, width) in [
        (serde_json::json!({"kind": "center", "width": 30}), 30.0),
        (serde_json::json!({"kind": "offset", "offset": 10, "width": 30}), 30.0),
        (serde_json::json!({"kind": "offset", "offset": 10, "width": 30, "from_end": true}), 30.0),
        (serde_json::json!({"kind": "between", "start": 10, "end": 25}), 65.0),
        (serde_json::json!({"kind": "between", "start": 0, "end": 25}), 75.0),
    ] {
        let b = flange(&p, &edge, serde_json::json!({"extent": extent})).unwrap();
        assert!(close(b.volume() - p.volume(), added(1.0, width, 20.0, 90.0)), "{extent}: {}", b.volume() - p.volume());
        let (size, _, bends) = blank(&b);
        assert!(close(size.0, 60.0 + longer(20.0, 90.0)) && close(size.1, 100.0), "{extent}: {size:?}");
        assert_eq!(bends, 2);
    }
    // From the edge's start and from its end: mirror images on the 100 long edge.
    let span = |from_end: bool| {
        let extent = serde_json::json!({"kind": "offset", "offset": 10, "width": 30, "from_end": from_end});
        let b = flange(&p, &edge, serde_json::json!({"extent": extent})).unwrap();
        let xs: Vec<f64> = b.vertices().iter().filter(|v| v.point.z == 20.0).map(|v| v.point.x).collect();
        (xs.iter().copied().fold(f64::INFINITY, f64::min), xs.iter().copied().fold(f64::NEG_INFINITY, f64::max))
    };
    let mut both = [span(false), span(true)];
    both.sort_by(|a, b| a.0.total_cmp(&b.0));
    assert!(close(both[0].0, 10.0) && close(both[0].1, 40.0) && close(both[1].0, 60.0) && close(both[1].1, 90.0), "{both:?}");
    // Either edge of either long end, alone: the end found across the thickness, never the
    // plate's own face (rectangular too).
    for face in &long {
        for e in long_edges(&p, face) {
            let b = flange(&p, &[e], serde_json::json!({"extent": {"kind": "center", "width": 30}})).unwrap();
            assert!(close(b.volume() - p.volume(), added(1.0, 30.0, 20.0, 90.0)));
        }
    }
    // Wider than the edge, or off it: refused.
    for extent in [serde_json::json!({"kind": "center", "width": 120}), serde_json::json!({"kind": "offset", "offset": 80, "width": 30})] {
        assert!(matches!(flange(&p, &edge, serde_json::json!({"extent": extent})), Err(BrepError::Degenerate(_))), "{extent}");
    }
}

/// A bend relief beside a short flange's bend (D229): a slot into the plate at each end of the
/// flange that is not the edge's end, `w` along the edge, `d` deep; its bottom square or round.
#[test]
fn a_bend_relief_beside_a_short_flange() {
    let p = plate();
    let long = faces_of_area(&p, 100.0 * T);
    let edge = long_edges(&p, &long[0]);
    let (w, d) = (2.0, 3.0);
    let half_disc = std::f64::consts::PI * 0.125 * w * w;
    for (extent, notches) in
        [(serde_json::json!({"kind": "center", "width": 30}), 2.0), (serde_json::json!({"kind": "between", "start": 0, "end": 25}), 1.0)]
    {
        let plain = flange(&p, &edge, serde_json::json!({"extent": extent})).unwrap();
        for (shape, slot) in [("square", w * d), ("round", w * (d - 0.5 * w) + half_disc)] {
            let b = flange(&p, &edge, serde_json::json!({"extent": extent, "bend_relief": shape, "bend_relief_width": w, "bend_relief_depth": d}))
                .unwrap();
            assert!(close(plain.volume() - b.volume(), notches * slot * T), "{extent} {shape}: {}", plain.volume() - b.volume());
            let f = unfold_side(&b, false, K).unwrap();
            assert!(f.size.x > 0.0);
        }
    }
    // Reaching the edge's end, or without the rule's sizes: refused.
    let near = serde_json::json!({"kind": "offset", "offset": 1, "width": 30});
    assert!(flange(&p, &edge, serde_json::json!({"extent": near, "bend_relief": "square", "bend_relief_width": w, "bend_relief_depth": d})).is_err());
    assert!(flange(&p, &edge, serde_json::json!({"extent": {"kind": "center", "width": 30}, "bend_relief": "square"})).is_err());
}

/// The bend flush (D229, Inventor's «inside base face extents»): the plate set back by `r + t`, the
/// flange's outer face where the edge was; the flange no wider than the plate's own extents.
#[test]
fn a_flush_flange() {
    let p = plate();
    let (long, short) = (faces_of_area(&p, 100.0 * T), faces_of_area(&p, 60.0 * T));
    let edge = long_edges(&p, &long[0]);
    let back = R + T;
    let flush = |extra: serde_json::Value| {
        let mut j = serde_json::json!({"bend_position": "flush"});
        for (k, v) in extra.as_object().unwrap() {
            j[k] = v.clone();
        }
        j
    };
    // The whole edge: the plate as wide as before, back by `r + t` along the edge.
    let b = flange(&p, &edge, flush(serde_json::json!({}))).unwrap();
    assert!(close(b.volume() - p.volume(), added(1.0, 100.0, 20.0, 90.0) - 100.0 * back * T), "{}", b.volume() - p.volume());
    let within = |b: &Body| {
        let (have, plate) = (b.bounds(), p.bounds());
        let near = |a: f64, b: f64| (a - b).abs() < 1e-9;
        near(have.0.x, plate.0.x) && near(have.1.x, plate.1.x) && near(have.0.y, plate.0.y) && near(have.1.y, plate.1.y)
    };
    assert!(within(&b), "{:?}", b.bounds());
    unfold_side(&b, false, K).unwrap();
    // Narrower: only with a bend relief, its slot from the bend line on: the middle and both slots
    // set back.
    let centre = serde_json::json!({"kind": "center", "width": 30});
    assert!(matches!(flange(&p, &edge, flush(serde_json::json!({"extent": centre}))), Err(BrepError::Unsupported(_))));
    let (w, d) = (2.0, 3.0);
    for (shape, slot) in [("square", w * (back + d)), ("round", w * (back + d - 0.5 * w) + std::f64::consts::PI * 0.125 * w * w)] {
        let extra = serde_json::json!({"extent": centre, "bend_relief": shape, "bend_relief_width": w, "bend_relief_depth": d});
        let r = flange(&p, &edge, flush(extra)).unwrap();
        let expected = added(1.0, 30.0, 20.0, 90.0) - 30.0 * back * T - 2.0 * slot * T;
        assert!(close(r.volume() - p.volume(), expected), "{shape}: {} vs {expected}", r.volume() - p.volume());
        unfold_side(&r, false, K).unwrap();
    }
    // All four edges: a tray within the plate's extents.
    let all = flange(&p, &ends_edges(&p, &[long, short].concat()), flush(serde_json::json!({}))).unwrap();
    assert!(within(&all), "{:?}", all.bounds());
    unfold_side(&all, false, K).unwrap();
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
    // One end by its two edges, or by one edge twice, is one flange.
    let p = plate();
    let long = faces_of_area(&p, 100.0 * T);
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
fn a_side_flange_on_a_leg_goes_inward() {
    let u = u_of(MID);
    let e = wall_edge(&u, 0, -2.0, 0.0);
    let b = side_of(&u, std::slice::from_ref(&e), serde_json::json!({}));
    assert!(close(b.volume() - u.volume(), added(1.0, LEG, 20.0, 90.0)), "{}", b.volume() - u.volume());
    let (size, counts, bends) = blank(&b);
    assert!(close(size.0, base_blank(&u).0) && close(size.1, WIDTH + longer(20.0, 90.0)), "{size:?}");
    assert_eq!((counts, bends), ([6, 0, 0], 6));
    // Inward, the way the top stands from the leg (D229): the sheet's bounds stay.
    let (lo, _) = b.bounds();
    assert!(close(lo.x.abs(), 2.0), "{lo:?}");
    // Flipped: outward, the leg's outer face is at x = −2, the flange reaches x = −20 (2 + 16 + 2
    // out of the wall).
    let f = side_of(&u, &[e], serde_json::json!({"flip": true}));
    assert!(close(f.volume() - u.volume(), added(1.0, LEG, 20.0, 90.0)));
    let (lo, _) = f.bounds();
    assert!(close(lo.x.abs(), 20.0), "{lo:?}");
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
    // Inward, all within the sheet's bounds.
    assert!(close(lo.x.abs(), 2.0) && close(hi.x, MID + 2.0) && close(hi.y, 62.0), "{lo:?} {hi:?}");
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
fn a_side_flange_and_an_end_flange_at_a_corner_are_built() {
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
    flange(&u, &edges, serde_json::json!({})).unwrap();
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

/// A cut over the free edge of a side flange: a notch in the flange (D205), its wall laid as a
/// plate's. A square centred on that edge, whichever way it lies, takes half of itself.
#[test]
fn a_notch_in_a_side_flange() {
    let u = u_of(MID);
    let b = side_of(&u, &[wall_edge(&u, 1, 62.0, 0.0)], serde_json::json!({}));
    let target = Value::Body(b.clone());
    let old: Vec<_> = u.faces().iter().map(|f| f.id).collect();
    let new = |x: usize| !old.contains(&b.faces()[x].id) && matches!(b.faces()[x].surface, Surface::Plane(_));
    let (x, wall) = (0..b.faces().len())
        .filter(|&x| new(x))
        .map(|x| (x, b.faces()[x].id))
        .max_by(|p, q| b.face_area(p.0 as u32).total_cmp(&b.face_area(q.0 as u32)))
        .unwrap();
    // The wall's free edge: on the flange's end, the new flat face `width × t`.
    let he = |h: u32| b.half_edges()[h as usize];
    let edge = b
        .loop_half_edges(b.faces()[x].first_loop)
        .find(|&h| {
            let y = b.loops()[he(he(h).twin).loop_ix as usize].face as usize;
            new(y) && close(b.face_area(y as u32), SIDE * T)
        })
        .unwrap();
    let m = (b.point(he(edge).origin) + b.point(he(b.next(edge)).origin)) * 0.5;
    let on_face = |at: [f64; 2]| {
        let (cx, cy) = (at[0], at[1]);
        let square = serde_json::json!({"points": {"a": [cx - 3.0, cy - 3.0], "b": [cx + 3.0, cy - 3.0], "c": [cx + 3.0, cy + 3.0], "d": [cx - 3.0, cy + 3.0]},
            "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}, "cd": {"from": "c", "to": "d"}, "da": {"from": "d", "to": "a"}}});
        op(serde_json::json!({"op": "sketch", "face": format!("{wall:?}"), "k_factor": K, "sketch": square})).eval(id("notch"), &[&target]).unwrap()
    };
    let [o, ax, ay, _] = on_face([0.0, 0.0]).sketch().unwrap().frame;
    let placed = on_face([(m - o).dot(ax), (m - o).dot(ay)]);
    let cut = op(serde_json::json!({"op": "extrude", "mode": "cut", "through": true, "wall": true}));
    let r = cut.eval(id("cut"), &[&target, &placed]).unwrap().body().unwrap().clone();
    r.validate().unwrap();
    tessellate(&r).unwrap();
    assert!(close(b.volume() - r.volume(), 18.0 * T), "{}", b.volume() - r.volume());
    // The blank's outline goes round the notch: three more lines and one split.
    let (_, counts, _) = blank(&r);
    assert_eq!(counts, [12, 0, 0]);
}

/// A hole on a leg across the edge where its side flange stands goes on through the flange's bend
/// (D229): the blank lays the flange across from the leg, the hole on it one circle.
#[test]
fn a_hole_reaching_a_side_flange_goes_through_its_bend() {
    let u = u_of(MID);
    let b = side_of(&u, &[wall_edge(&u, 0, -2.0, 0.0)], serde_json::json!({}));
    let target = Value::Body(b.clone());
    let first = b.faces().iter().find(|f| matches!(f.surface, Surface::Plane(p) if p.normal.x < -0.5 && p.origin.x <= 0.0)).unwrap();
    // The sketch's Y is −z: the circle of 6 spans z from −1 to 5, into the bend; of 14 from −5 to
    // 9, on into the flange's wall.
    for d in [6.0, 14.0] {
        let sketch = serde_json::json!({"points": {"h": [30, -2]}, "circles": {"c": {"center": "h", "diameter": d}}});
        let placed = op(serde_json::json!({"op": "sketch", "face": format!("{:?}", first.id), "k_factor": K, "sketch": sketch}))
            .eval(id("hole"), &[&target])
            .unwrap();
        let cut = op(serde_json::json!({"op": "extrude", "mode": "cut", "through": true, "wall": true}));
        let r = cut.eval(id("cut"), &[&target, &placed]).unwrap().body().unwrap().clone();
        r.validate().unwrap();
        tessellate(&r).unwrap();
        assert!(r.volume() < b.volume());
        let flat = unfold_side(&r, false, K).unwrap();
        hole_of(&flat, 0.5 * d);
    }
}

/// The hole of radius `r` in `flat`: one circle, or arcs of one circle round it.
fn hole_of(flat: &datum_ops::Flat, r: f64) -> datum_math::Vec2 {
    use datum_ops::FlatPiece;
    let mut centre = None;
    let mut turn = 0.0;
    for p in &flat.pieces {
        let (c, radius, sweep) = match *p {
            FlatPiece::Circle { center, radius } => (center, radius, std::f64::consts::TAU),
            FlatPiece::Arc { center, radius, start, end } => (center, radius, (end - start).rem_euclid(std::f64::consts::TAU)),
            _ => continue,
        };
        if (radius - r).abs() > 1e-9 {
            continue;
        }
        let at = *centre.get_or_insert(c);
        assert!((c.x - at.x).abs() < 1e-9 && (c.y - at.y).abs() < 1e-9, "{c:?} {at:?}");
        turn += sweep;
    }
    assert!((turn - std::f64::consts::TAU).abs() < 1e-9, "{turn} {:?}", flat.pieces);
    centre.unwrap()
}

/// A work plane through `origin`, its normal `normal`.
fn work_plane(origin: [f64; 3], normal: [f64; 3]) -> Value {
    let x_axis = if normal[2] == 0.0 { Vec3::new(0.0, 0.0, 1.0) } else { Vec3::new(1.0, 0.0, 0.0) };
    Value::Plane(datum_ops::SketchPlane {
        origin: Vec3::new(origin[0], origin[1], origin[2]),
        x_axis,
        normal: Vec3::new(normal[0], normal[1], normal[2]),
    })
}

/// A flange of height 20 whose wall's edges go to work planes `planes` by `to` (D229).
fn flange_to(b: &Body, edges: &[String], planes: &[(&str, Value)], to: serde_json::Value, extra: serde_json::Value) -> Result<Body, BrepError> {
    let mut j = serde_json::json!({"op": "flange", "edges": edges, "height": 20, "radius": R, "to": to});
    for (k, v) in extra.as_object().unwrap() {
        j[k] = v.clone();
    }
    let sheet = Value::Body(b.clone());
    let mut values = vec![&sheet];
    let mut names = vec!["sheet"];
    for (n, v) in planes {
        values.push(v);
        names.push(n);
    }
    let r = op(j).eval_named(id("flange"), &values, &names)?.body().unwrap().clone();
    r.validate().unwrap();
    tessellate(&r).unwrap();
    Ok(r)
}

/// The least and most X of the vertices of `b` at height `z`.
fn x_span(b: &Body, z: f64) -> (f64, f64) {
    let xs = || b.vertices().iter().filter(|v| close(v.point.z, z)).map(|v| v.point.x);
    (xs().fold(f64::INFINITY, f64::min), xs().fold(f64::NEG_INFINITY, f64::max))
}

const WALL: f64 = 20.0 - (R + T);

/// The sides of a wall go to planes (D229): beyond its edge it grows, the bend stays, a gap short;
/// inside it is cut, the bend bare beyond; the blank has a wall as wide, its bends as they were.
#[test]
fn a_walls_sides_go_to_planes() {
    let p = plate();
    let long = faces_of_area(&p, 100.0 * T);
    let edge = long_edges(&p, &long[0]);
    let plain = flange(&p, &edge, serde_json::json!({})).unwrap();
    let x = |at: f64| work_plane([at, 0.0, 0.0], [1.0, 0.0, 0.0]);
    let to = |plane: &str, gap: f64| serde_json::json!({"plane": plane, "gap": gap});
    // Which side is which: one way round the planes are beyond each other, and leave no width.
    for (start, end) in [(-5.0, 110.0), (20.0, 90.0), (-5.0, 90.0)] {
        let mut made = 0;
        for (first, second) in [("start", "end"), ("end", "start")] {
            let tos = serde_json::json!({first: to("a", 1.0), second: to("b", 1.0)});
            let planes = [("a", x(start)), ("b", x(end))];
            match flange_to(&p, &edge, &planes, tos, serde_json::json!({})) {
                Ok(b) => {
                    let (lo, hi) = x_span(&b, 20.0);
                    assert!(close(lo, start + 1.0) && close(hi, end - 1.0), "{start} {end}: {lo} {hi}");
                    let wide = hi - lo;
                    assert!((b.volume() - plain.volume() - T * WALL * (wide - 100.0)).abs() < 1e-9 * b.volume(), "{start} {end}: {}", b.volume());
                    assert!(close(x_span(&b, T).1 - x_span(&b, T).0, 100.0), "the bend stays");
                    let (_, _, bends) = blank(&b);
                    assert_eq!(bends, 2);
                    made += 1;
                }
                Err(BrepError::Degenerate(_)) => {}
                Err(e) => panic!("{start} {end}: {e}"),
            }
        }
        assert_eq!(made, 1, "{start} {end}");
    }
}

/// `start` is the end a width is measured from: the same end for an offset and for a plane.
#[test]
fn a_walls_start_is_the_start_of_its_width() {
    let p = plate();
    let long = faces_of_area(&p, 100.0 * T);
    for face in &long {
        for e in long_edges(&p, face) {
            let edge = [e];
            let offset = flange(&p, &edge, serde_json::json!({"extent": {"kind": "offset", "offset": 10, "width": 30}})).unwrap();
            let low_start = x_span(&offset, 20.0).0 < 50.0;
            let at = if low_start { -7.0 } else { 107.0 };
            let planes = [("a", work_plane([at, 0.0, 0.0], [1.0, 0.0, 0.0]))];
            let b = flange_to(&p, &edge, &planes, serde_json::json!({"start": {"plane": "a"}}), serde_json::json!({})).unwrap();
            let (lo, hi) = x_span(&b, 20.0);
            assert!(close(if low_start { lo } else { hi }, at), "{lo} {hi}");
            // A plane's normal may face either way.
            let planes = [("a", work_plane([at, 0.0, 0.0], [-1.0, 0.0, 0.0]))];
            let same = flange_to(&p, &edge, &planes, serde_json::json!({"start": {"plane": "a"}}), serde_json::json!({})).unwrap();
            assert!(close(same.volume(), b.volume()));
        }
    }
}

/// A slanted plane: the wall's side slants, square to the wall all the same, and its blank is a
/// trapezoid.
#[test]
fn a_walls_side_to_a_slanted_plane() {
    let p = plate();
    let edge = long_edges(&p, &faces_of_area(&p, 100.0 * T)[0]);
    let plain = flange(&p, &edge, serde_json::json!({})).unwrap();
    // Through (100, y, R + T), the wall's foot, going out 1 along X for each 1 up: at the top `WALL`
    // further out.
    let planes = [("a", work_plane([100.0, 0.0, R + T], [1.0, 0.0, -1.0]))];
    let up = flange_to(&p, &edge, &planes, serde_json::json!({"start": {"plane": "a"}}), serde_json::json!({}));
    let down = flange_to(&p, &edge, &planes, serde_json::json!({"end": {"plane": "a"}}), serde_json::json!({}));
    let b = [up, down].into_iter().flatten().max_by(|a, b| a.volume().total_cmp(&b.volume())).unwrap();
    assert!(close(b.volume() - plain.volume(), T * WALL * WALL * 0.5), "{}", b.volume() - plain.volume());
    assert!(close(x_span(&b, 20.0).1, 100.0 + WALL));
    let (_, _, bends) = blank(&b);
    assert_eq!(bends, 2);
}

/// The free edge goes to a plane: the height is that alone, level or slanted, with the sides too.
#[test]
fn a_walls_top_goes_to_a_plane() {
    let p = plate();
    let edge = long_edges(&p, &faces_of_area(&p, 100.0 * T)[0]);
    let top = |gap: f64| serde_json::json!({"top": {"plane": "a", "gap": gap}});
    // Level at 16, a gap of 1: as high as a flange of 15; the height given says nothing, even one
    // no higher than the bend.
    let level = [("a", work_plane([0.0, 0.0, 16.0], [0.0, 0.0, 1.0]))];
    let same = flange(&p, &edge, serde_json::json!({"height": 15})).unwrap();
    for height in [20.0, 1.0] {
        let b = flange_to(&p, &edge, &level, top(1.0), serde_json::json!({"height": height})).unwrap();
        assert!(close(b.volume(), same.volume()), "{}", b.volume());
        assert_eq!(blank(&b).0, blank(&same).0);
    }
    // Slanted: 16 at x = 0, 6 higher at x = 100.
    let slant = [("a", work_plane([0.0, 0.0, 16.0], [-6.0, 0.0, 100.0]))];
    let b = flange_to(&p, &edge, &slant, top(0.0), serde_json::json!({})).unwrap();
    let wall = |z: f64| z - (R + T);
    assert!(
        close(b.volume() - p.volume(), same.volume() - p.volume() - T * 100.0 * wall(15.0) + T * 100.0 * 0.5 * (wall(16.0) + wall(22.0))),
        "{}",
        b.volume()
    );
    let tops: Vec<f64> = b.vertices().iter().map(|v| v.point.z).collect();
    assert!(close(tops.iter().copied().fold(f64::NEG_INFINITY, f64::max), 22.0));
    assert_eq!(blank(&b).2, 2);
    // And sides to planes with it: the corner where both planes meet.
    let planes = [
        ("a", work_plane([0.0, 0.0, 16.0], [0.0, 0.0, 1.0])),
        ("s", work_plane([-9.0, 0.0, 0.0], [1.0, 0.0, 0.0])),
        ("e", work_plane([109.0, 0.0, 0.0], [1.0, 0.0, 0.0])),
    ];
    let (sa, sb) = (
        serde_json::json!({"start": {"plane": "s"}, "end": {"plane": "e"}, "top": {"plane": "a"}}),
        serde_json::json!({"start": {"plane": "e"}, "end": {"plane": "s"}, "top": {"plane": "a"}}),
    );
    let b = [flange_to(&p, &edge, &planes, sa, serde_json::json!({})), flange_to(&p, &edge, &planes, sb, serde_json::json!({}))]
        .into_iter()
        .flatten()
        .next()
        .unwrap();
    assert_eq!(x_span(&b, 16.0), (-9.0, 109.0));
}

/// Two parts meet at one plane (D229): each goes to it by its own gap, knowing nothing of the other.
#[test]
fn two_parts_meet_at_a_joint_plane() {
    let p = plate();
    let edge = long_edges(&p, &faces_of_area(&p, 100.0 * T)[0]);
    let joint = [("joint", work_plane([130.0, 0.0, 0.0], [1.0, 0.0, 0.0]))];
    // The side toward the plane: the other, it leaves no width.
    let go = |plate: &Body, edge: &[String]| {
        let to = |key: &str| serde_json::json!({key: {"plane": "joint", "gap": 0.5}});
        let (s, e) = (to("start"), to("end"));
        [s, e].into_iter().find_map(|to| flange_to(plate, edge, &joint, to, serde_json::json!({})).ok()).unwrap()
    };
    let near = go(&p, &edge);
    assert!(close(x_span(&near, 20.0).1, 129.5));
    // Any flange at all, the plane the only thing shared: a second plate past the joint.
    let moved = op(serde_json::json!({"op": "transform", "translate": [160, 0, 0]}))
        .eval(id("moved"), &[&Value::Body(p.clone())])
        .unwrap()
        .body()
        .unwrap()
        .clone();
    let edge = long_edges(&moved, &faces_of_area(&moved, 100.0 * T)[0]);
    let far = go(&moved, &edge);
    assert!(close(x_span(&far, 20.0).0, 130.5));
}

/// With the other choices of a flange: a width of part of the edge, a flush bend; along a wall.
#[test]
fn a_plane_with_a_width_and_along_a_wall() {
    let p = plate();
    let edge = long_edges(&p, &faces_of_area(&p, 100.0 * T)[0]);
    let extent = serde_json::json!({"extent": {"kind": "center", "width": 30}});
    let plain = flange(&p, &edge, extent.clone()).unwrap();
    let planes = [("a", work_plane([0.0, 0.0, 16.0], [0.0, 0.0, 1.0]))];
    let b = flange_to(&p, &edge, &planes, serde_json::json!({"top": {"plane": "a"}}), extent.clone()).unwrap();
    assert!(close(plain.volume() - b.volume(), T * 30.0 * 4.0), "{}", b.volume());
    // The narrow wall grown from its own side: to x = 10, it spans from 35 or 65 on to there.
    let planes = [("a", work_plane([10.0, 0.0, 0.0], [1.0, 0.0, 0.0]))];
    let grown = ["start", "end"].iter().filter_map(|k| flange_to(&p, &edge, &planes, serde_json::json!({*k: {"plane": "a"}}), extent.clone()).ok());
    assert!(grown.into_iter().any(|b| x_span(&b, 20.0).0 == 10.0));
    // A side flange on a U's top wall: it turns along Y, its height from the outer face at 62.
    let u = u_of(MID);
    let side = wall_edge(&u, 1, 62.0, 0.0);
    let planes = [("a", work_plane([0.0, 50.0, 0.0], [0.0, 1.0, 0.0]))];
    let same = side_of(&u, std::slice::from_ref(&side), serde_json::json!({"height": 12}));
    let b = flange_to(&u, &[side], &planes, serde_json::json!({"top": {"plane": "a"}}), serde_json::json!({})).unwrap();
    assert!(close(b.volume(), same.volume()), "{} {}", b.volume(), same.volume());
    assert_eq!(blank(&b).2, blank(&same).2);
}

#[test]
fn a_plane_a_wall_cannot_go_to_is_refused() {
    let p = plate();
    let edge = long_edges(&p, &faces_of_area(&p, 100.0 * T)[0]);
    let start = serde_json::json!({"start": {"plane": "a"}});
    let top = serde_json::json!({"top": {"plane": "a"}});
    let none = serde_json::json!({});
    // Along the edge, and along the wall.
    let along = [("a", work_plane([0.0, 70.0, 0.0], [0.0, 1.0, 0.0]))];
    assert!(matches!(flange_to(&p, &edge, &along, start.clone(), none.clone()), Err(BrepError::Unsupported(_))));
    assert!(matches!(flange_to(&p, &edge, &along, top.clone(), none.clone()), Err(BrepError::Unsupported(_))));
    // Below the bend's end: no height. Beyond the other side: no width.
    let low = [("a", work_plane([0.0, 0.0, 1.0], [0.0, 0.0, 1.0]))];
    assert!(matches!(flange_to(&p, &edge, &low, top, none.clone()), Err(BrepError::Degenerate(_))));
    let cuts: Vec<_> = [-20.0, 120.0]
        .iter()
        .map(|&x| flange_to(&p, &edge, &[("a", work_plane([x, 0.0, 0.0], [1.0, 0.0, 0.0]))], start.clone(), none.clone()))
        .collect();
    assert!(cuts.iter().any(|r| matches!(r, Err(BrepError::Degenerate(_)))));
    // Not among the inputs, no work plane, a gap below zero.
    assert!(matches!(flange_to(&p, &edge, &[], start.clone(), none.clone()), Err(BrepError::Sketch(_))));
    let body = [("a", Value::Body(p.clone()))];
    assert!(matches!(flange_to(&p, &edge, &body, start, none.clone()), Err(BrepError::Sketch(_))));
    let x = [("a", work_plane([120.0, 0.0, 0.0], [1.0, 0.0, 0.0]))];
    assert!(matches!(flange_to(&p, &edge, &x, serde_json::json!({"end": {"plane": "a", "gap": -1}}), none), Err(BrepError::Degenerate(_))));
}

/// A plane through the wall's own end, level or slanted, on any edge: no step of rounding's width.
#[test]
fn a_plane_through_a_walls_end_leaves_no_step() {
    let p = plate();
    for face in faces_of_area(&p, 100.0 * T) {
        let edge = long_edges(&p, &face);
        let plain = flange(&p, &edge, serde_json::json!({})).unwrap();
        for (normal, grown) in [([1.0, 0.0, 0.0], 0.0), ([1.0, 0.0, -1.0], T * WALL * WALL * 0.5), ([1.0, 0.0, -0.1], T * WALL * WALL * 0.05)] {
            let planes = [("a", work_plane([100.0, 0.0, R + T], normal))];
            let made: Vec<Body> = ["start", "end"]
                .iter()
                .filter_map(|k| flange_to(&p, &edge, &planes, serde_json::json!({*k: {"plane": "a"}}), serde_json::json!({})).ok())
                .collect();
            assert_eq!(made.len(), 1, "{normal:?}");
            assert!(
                close(made[0].volume() - plain.volume(), grown) || (grown == 0.0 && close(made[0].volume(), plain.volume())),
                "{normal:?}: {}",
                made[0].volume() - plain.volume()
            );
            let steps = made[0].faces().len() - plain.faces().len();
            assert!(steps <= 1, "{normal:?}: {steps} new faces");
        }
    }
}

/// A rim on the top of each of the four walls of a tray: the blank is a tree of strips, a cross
/// with a rim on each arm, however many rims there are.
#[test]
fn rims_on_all_four_walls_lie_out_on_the_tree() {
    let p = plate();
    let ends = [faces_of_area(&p, 100.0 * T), faces_of_area(&p, 60.0 * T)].concat();
    let tray = flange(&p, &ends_edges(&p, &ends), serde_json::json!({})).unwrap();
    // The tops, before any rim: a rimmed body has no ends left to ask for.
    let tops: Vec<String> = flange_ends(&tray).iter().map(|e| format!("{e:?}")).collect();
    assert_eq!(tops.len(), 4);
    let grow = 2.0 * (longer(20.0, 90.0) + longer(10.0, 90.0));
    let (long_side, short_side) = (100.0 + grow, 60.0 + grow);
    let rim = |b: &Body, top: &str| flange(b, &long_edges(b, top)[..1], serde_json::json!({"height": 10})).unwrap();
    let mut body = tray.clone();
    let (mut along_x, mut along_y) = (0.0, 0.0);
    for (n, top) in tops.iter().enumerate() {
        body = rim(&body, top);
        let size = unfold_side(&body, false, K).unwrap().size;
        // A wall as wide as the plate's long side stands on a long end and reaches along Y.
        let wide = (0..tray.faces().len()).find(|&x| hex(&tray, x) == *top).map(|x| tray.face_area(x as u32) / T).unwrap();
        if close(wide, 100.0) {
            along_y += 1.0
        } else {
            along_x += 1.0
        }
        let (x, y) =
            (100.0 + 2.0 * longer(20.0, 90.0) + along_x * longer(10.0, 90.0), 60.0 + 2.0 * longer(20.0, 90.0) + along_y * longer(10.0, 90.0));
        // A rim on the walls of one class alone makes that class the one Y runs along.
        assert!((close(size.x, x) && close(size.y, y)) || (close(size.x, y) && close(size.y, x)), "{} rims: {size:?}, {x} x {y}", n + 1);
    }
    let f = unfold_side(&body, false, K).unwrap();
    // Rims on all walls: the blank lies the way the tray's does.
    assert!(close(f.size.x, short_side) && close(f.size.y, long_side), "{:?}, {short_side} x {long_side}", f.size);
    let plain = unfold_side(&tray, false, K).unwrap().size;
    assert!(plain.x < plain.y, "{plain:?}");
    assert_eq!(f.bends.len(), 16);
    let ids: BTreeSet<TopoId> = f.bend_ids.iter().copied().collect();
    assert_eq!(ids.len(), f.bend_ids.len());
    assert!(f.overlaps.is_empty(), "{:?}", f.overlaps);
}

/// Two walls on neighbouring ends of a plate, each grown by `to` past the plate's corner, away from
/// the plate (D229): on the blank the strip of one wall lies over the strip of the other at the
/// corner. The plate's corner at the origin, one wall of 100 along X grown to x = -30, one of 60
/// along Y grown to y = -30. Without the growing, the blank has no overlap.
#[test]
fn a_blank_lying_on_itself_is_laid_and_says_where() {
    let p = plate();
    let (long, short) = (faces_of_area(&p, 100.0 * T), faces_of_area(&p, 60.0 * T));
    let at = |f: &String| centroid(&p, (0..p.faces().len()).find(|&x| hex(&p, x) == *f).unwrap());
    let along_x = long_edges(&p, long.iter().find(|f| at(f).y < 30.0).unwrap());
    let along_y = long_edges(&p, short.iter().find(|f| at(f).x < 50.0).unwrap());
    let plain = flange(&flange(&p, &along_x[..1], serde_json::json!({})).unwrap(), &along_y[..1], serde_json::json!({})).unwrap();
    let flat = unfold_side(&plain, false, K).unwrap();
    assert!(flat.overlaps.is_empty(), "{:?}", flat.overlaps);
    let past_x = [("x", work_plane([-30.0, 0.0, 0.0], [1.0, 0.0, 0.0]))];
    let past_y = [("y", work_plane([0.0, -30.0, 0.0], [0.0, 1.0, 0.0]))];
    let first = flange_to(&p, &along_x[..1], &past_x, serde_json::json!({"start": {"plane": "x"}}), serde_json::json!({})).unwrap();
    let both = flange_to(&first, &along_y[..1], &past_y, serde_json::json!({"end": {"plane": "y"}}), serde_json::json!({})).unwrap();
    let flat = unfold_side(&both, false, K).unwrap();
    // The plate lies 30 in from each side of the blank; each wall strip reaches `longer` out from the
    // plate, its straight part (the wall, 20 less the bend's set-back) the outer 16 of it.
    assert!(close(flat.size.x, 60.0 + 30.0) && close(flat.size.y, 100.0 + 30.0), "{:?}", flat.size);
    assert_eq!(flat.overlaps.len(), 1, "{:?}", flat.overlaps);
    let [lo, hi] = flat.overlaps[0];
    assert!(lo.x >= 0.0 && lo.y >= 0.0 && hi.x <= flat.size.x && hi.y <= flat.size.y, "{lo:?} {hi:?}");
    let wall = 20.0 - (R + T);
    assert!(close(hi.x - lo.x, wall) && close(hi.y - lo.y, wall), "{lo:?} {hi:?}");
    // Across the corner, the walls' ends reach the same way out from the plate.
    let out = 30.0 - longer(20.0, 90.0);
    assert!(close(lo.x, out), "{lo:?}");
    let middle = out + wall / 2.0;
    assert!(lo.x < middle && middle < hi.x && lo.y < flat.size.y - middle && flat.size.y - middle < hi.y, "{lo:?} {hi:?}");
}

/// A short flush flange with a round relief: the blank is as large seen from either side. (The back
/// side once came out 4 larger.)
#[test]
fn a_blank_from_either_side_is_as_large() {
    let p = plate();
    for face in faces_of_area(&p, 100.0 * T) {
        for edge in long_edges(&p, &face) {
            let extra = serde_json::json!({"bend_position": "flush", "extent": {"kind": "center", "width": 40},
                "bend_relief": "round", "bend_relief_width": 2, "bend_relief_depth": 2});
            let b = flange(&p, &[edge], extra).unwrap();
            let (front, back) = (unfold_side(&b, false, K).unwrap().size, unfold_side(&b, true, K).unwrap().size);
            assert!(close(front.x, back.x) && close(front.y, back.y), "{front:?} {back:?}");
        }
    }
}

/// The plate of 100 x 60 with a flange of 20 on each end and a rim of 10 on the first long edge of
/// each wall's top. Returns the tray without rims too.
fn rimmed() -> Body {
    rimmed_with(serde_json::json!({}))
}

/// [`rimmed`] with `extra` fields on the flanges round the plate.
fn rimmed_with(extra: serde_json::Value) -> Body {
    let p = plate();
    let ends = [faces_of_area(&p, 100.0 * T), faces_of_area(&p, 60.0 * T)].concat();
    let tray = flange(&p, &ends_edges(&p, &ends), extra).unwrap();
    let tops: Vec<String> = flange_ends(&tray).iter().map(|e| format!("{e:?}")).collect();
    assert_eq!(tops.len(), 4);
    let mut body = tray;
    for top in &tops {
        let edge = long_edges(&body, top)[..1].to_vec();
        body = flange(&body, &edge, serde_json::json!({"height": 10})).unwrap();
    }
    body
}

/// The planes of `b`'s faces as (index, normal z, area).
fn plane_faces(b: &Body) -> Vec<(usize, f64, f64)> {
    (0..b.faces().len())
        .filter_map(|x| match b.faces()[x].surface {
            Surface::Plane(p) => Some((x, p.normal.z, b.face_area(x as u32))),
            _ => None,
        })
        .collect()
}

/// The corners of face `x`.
fn corners(b: &Body, x: usize) -> Vec<Vec3> {
    b.loop_half_edges(b.faces()[x].first_loop).map(|h| b.point(b.half_edges()[h as usize].origin)).collect()
}

/// A wall of the rimmed tray `b` on the long side (y < 0) and its outer face.
fn long_wall_face(b: &Body) -> usize {
    let found: Vec<usize> =
        plane_faces(b).into_iter().filter(|&(x, nz, a)| nz.abs() < 1e-9 && close(a, 100.0 * 16.0) && centroid(b, x).y < 0.0).map(|f| f.0).collect();
    assert_eq!(found.len(), 2, "the two faces of the wall at y < 0");
    // The outer one is the further from the plate's middle.
    *found.iter().min_by(|&&p, &&q| centroid(b, p).y.total_cmp(&centroid(b, q).y)).unwrap()
}

/// The faces of the rim on that wall: horizontal, 6 wide, over the wall.
fn rim_faces(b: &Body) -> Vec<usize> {
    let found: Vec<usize> = plane_faces(b)
        .into_iter()
        .filter(|&(x, nz, a)| (nz.abs() - 1.0).abs() < 1e-9 && close(a, 100.0 * 6.0) && centroid(b, x).y < 10.0 && centroid(b, x).z > 10.0)
        .map(|f| f.0)
        .collect();
    assert_eq!(found.len(), 2, "the two faces of the rim at y < 0");
    found
}

/// A circle of diameter `d` cut through `b` from face `face`, centred at the world point `q` of its
/// plane; the body and where the circle lies on the blank, if it does.
fn cut_at(b: &Body, face: usize, q: Vec3, d: f64) -> Body {
    let target = Value::Body(b.clone());
    let at = |sk: serde_json::Value| {
        op(serde_json::json!({"op": "sketch", "face": hex(b, face), "k_factor": 0.5, "sketch": sk})).eval(id("s"), &[&target]).unwrap()
    };
    let frame = at(serde_json::json!({"points": {"h": [0, 0]}})).sketch().unwrap().frame;
    let [o, ex, ey, _] = frame;
    let sk = serde_json::json!({"points": {"h": [(q - o).dot(ex), (q - o).dot(ey)]}, "circles": {"c": {"center": "h", "diameter": d}}});
    let placed = at(sk);
    let r = op(serde_json::json!({"op": "extrude", "mode": "cut", "through": true, "wall": true})).eval(id("c"), &[&target, &placed]).unwrap();
    let r = r.body().unwrap().clone();
    r.validate().unwrap();
    tessellate(&r).unwrap();
    r
}

/// The centre of the only circle of the blank of `b` (K 0.5).
fn circle_centre(b: &Body) -> datum_math::Vec2 {
    let f = unfold_side(b, false, 0.5).unwrap();
    let c: Vec<datum_math::Vec2> = f
        .pieces
        .iter()
        .filter_map(|p| match p {
            datum_ops::FlatPiece::Circle { center, .. } => Some(*center),
            _ => None,
        })
        .collect();
    assert_eq!(c.len(), 1);
    c[0]
}

/// A hole of diameter 6 (r = 3) cut from `cut` and `before`: it removes pi r^2 t and adds one circle
/// to the blank, whose size and lines stay.
fn assert_whole_hole(before: &Body, cut: &Body) {
    let (a, b) = (unfold_side(before, false, 0.5).unwrap(), unfold_side(cut, false, 0.5).unwrap());
    let lost = before.volume() - cut.volume();
    assert!((lost - std::f64::consts::PI * 9.0 * T).abs() < 1e-3, "lost {lost}");
    assert!(close(a.size.x, b.size.x) && close(a.size.y, b.size.y), "{:?} {:?}", a.size, b.size);
    assert_eq!(b.counts()[2], a.counts()[2] + 1);
    assert_eq!(b.counts()[0], a.counts()[0]);
}

/// The point of the wall face `w` of the rimmed tray `b` 1 below the wall's top.
fn wall_spot(b: &Body, w: usize) -> Vec3 {
    let zmax = corners(b, w).iter().fold(f64::MIN, |m, p| m.max(p.z));
    let c = centroid(b, w);
    Vec3::new(c.x, c.y, zmax - 1.0)
}

/// A hole of diameter 6 sketched on a wall's outer face, reaching 2 across the bend to the rim.
#[test]
fn a_hole_across_a_rims_bend_from_the_wall() {
    let b = rimmed();
    let w = long_wall_face(&b);
    let cut = cut_at(&b, w, wall_spot(&b, w), 6.0);
    assert_whole_hole(&b, &cut);
}

/// The same bend from the rim's face, the circle at the same spot of the blank.
#[test]
fn a_hole_across_a_rims_bend_from_the_rim() {
    let b = rimmed();
    let w = long_wall_face(&b);
    let from_wall = circle_centre(&cut_at(&b, w, wall_spot(&b, w), 6.0));
    for r in rim_faces(&b) {
        // The rim's sketch reaches over its bend: about 3.7 back from the rim's start (the wall's
        // 1 short of its top). Along the rim the blank follows 1 for 1: two probes find the spot.
        let (c, z) = (centroid(&b, r), centroid(&b, r).z);
        let c = Vec3::new(c.x, c.y - 6.7, c.z);
        let probe = |dy: f64| circle_centre(&cut_at(&b, r, Vec3::new(c.x, c.y + dy, z), 6.0));
        let (p0, p1) = (probe(0.0), probe(0.5));
        let (step, to) = ((p1.x - p0.x, p1.y - p0.y), (from_wall.x - p0.x, from_wall.y - p0.y));
        let s = 0.5 * (to.0 * step.0 + to.1 * step.1) / (step.0 * step.0 + step.1 * step.1);
        let cut = cut_at(&b, r, Vec3::new(c.x, c.y + s, z), 6.0);
        assert_whole_hole(&b, &cut);
        let here = circle_centre(&cut);
        assert!((here.x - from_wall.x).abs() < 1e-9 && (here.y - from_wall.y).abs() < 1e-9, "{here:?} {from_wall:?} (s {s})");
    }
}

/// On the plate's side face, 1 short of a wall's bend.
#[test]
fn a_hole_on_the_plate_across_a_walls_bend_of_a_rimmed_tray() {
    let b = rimmed();
    let plate_face = plane_faces(&b).into_iter().filter(|f| (f.1.abs() - 1.0).abs() < 1e-9).max_by(|p, q| p.2.total_cmp(&q.2)).unwrap().0;
    let xmax = corners(&b, plate_face).iter().fold(f64::MIN, |m, p| m.max(p.x));
    let c = centroid(&b, plate_face);
    let cut = cut_at(&b, plate_face, Vec3::new(xmax - 1.0, c.y, c.z), 6.0);
    assert_whole_hole(&b, &cut);
}

/// A circle on a rim's face over its free edge: a notch.
#[test]
fn a_notch_at_a_rims_free_edge() {
    let b = rimmed();
    let r = rim_faces(&b)[0];
    let c = centroid(&b, r);
    let ymax = corners(&b, r).iter().fold(f64::MIN, |m, p| m.max(p.y));
    let cut = cut_at(&b, r, Vec3::new(c.x, ymax - 1.0, c.z), 6.0);
    let (a, n) = (unfold_side(&b, false, 0.5).unwrap(), unfold_side(&cut, false, 0.5).unwrap());
    let lost = b.volume() - cut.volume();
    assert!(lost > 0.0 && lost < std::f64::consts::PI * 9.0 * T, "lost {lost}");
    assert!(n.counts()[0] + n.counts()[1] > a.counts()[0] + a.counts()[1], "{:?} {:?}", a.counts(), n.counts());
    assert_eq!(n.counts()[2], a.counts()[2]);
    assert!(close(a.size.x, n.size.x) && close(a.size.y, n.size.y), "{:?} {:?}", a.size, n.size);
}

/// Rims on a tray with a round relief at each corner: a hole centred on a relief still cuts.
#[test]
fn rims_keep_their_relief() {
    let b = rimmed_with(serde_json::json!({"relief": "round", "relief_diameter": 4, "k_factor": 0.5}));
    let f = plane_faces(&b).into_iter().filter(|f| (f.1.abs() - 1.0).abs() < 1e-9).max_by(|p, q| p.2.total_cmp(&q.2)).unwrap().0;
    let z = centroid(&b, f).z;
    let before = unfold_side(&b, false, 0.5).unwrap();
    let cut = |x: f64, y: f64| {
        let target = Value::Body(b.clone());
        let at = |sk: serde_json::Value| {
            op(serde_json::json!({"op": "sketch", "face": hex(&b, f), "k_factor": 0.5, "sketch": sk})).eval(id("s"), &[&target]).unwrap()
        };
        let [o, ex, ey, _] = at(serde_json::json!({"points": {"h": [0, 0]}})).sketch().unwrap().frame;
        let q = Vec3::new(x, y, z);
        let sk = serde_json::json!({"points": {"h": [(q - o).dot(ex), (q - o).dot(ey)]}, "circles": {"c": {"center": "h", "diameter": 3}}});
        let placed = at(sk);
        op(serde_json::json!({"op": "extrude", "mode": "cut", "through": true, "wall": true})).eval(id("c"), &[&target, &placed])
    };
    // Beside the relief: a whole hole, the relief's arcs stay.
    for (x, y) in [(4.0, 4.0), (6.0, 1.0)] {
        let c = cut(x, y).unwrap().body().unwrap().clone();
        c.validate().unwrap();
        let after = unfold_side(&c, false, 0.5).unwrap();
        assert!((b.volume() - c.volume() - std::f64::consts::PI * 2.25 * T).abs() < 1e-3);
        assert_eq!(after.counts(), [before.counts()[0], before.counts()[1], 1]);
        assert!(close(after.size.x, before.size.x) && close(after.size.y, before.size.y));
    }
    // Reaching into the relief: the circle opens into it, the blank keeps its size and its arcs.
    let c = cut(2.0, 2.0).unwrap().body().unwrap().clone();
    c.validate().unwrap();
    let after = unfold_side(&c, false, 0.5).unwrap();
    assert!(b.volume() - c.volume() > 0.0 && b.volume() - c.volume() < std::f64::consts::PI * 2.25 * T);
    assert!(after.counts()[1] > before.counts()[1] && after.counts()[2] == 0, "{:?} {:?}", before.counts(), after.counts());
    assert!(close(after.size.x, before.size.x) && close(after.size.y, before.size.y));
    // Off the sheet or a hair from a face: refused, not guessed.
    assert!(cut(0.0, 0.0).is_err());
    assert!(cut(3.0, 3.0).is_err());
}

/// The tray of a plate with a flange on each end, and the ID of the top of each of its walls.
fn tray_and_caps() -> (Body, Vec<String>) {
    let p = plate();
    let ends = [faces_of_area(&p, 100.0 * T), faces_of_area(&p, 60.0 * T)].concat();
    let tray = flange(&p, &ends_edges(&p, &ends), serde_json::json!({})).unwrap();
    let caps = flange_ends(&tray).iter().map(|e| format!("{e:?}")).collect();
    (tray, caps)
}

/// The tray and the tray with four rims picked in one flange, `extra` its fields: with `flip` and
/// a mitre, what the flange of the BMT inlet is (D233).
fn tray_with_rims(extra: serde_json::Value) -> (Body, Body) {
    let (tray, tops) = tray_and_caps();
    assert_eq!(tops.len(), 4);
    let edges: Vec<String> = tops.iter().map(|t| long_edges(&tray, t)[0].clone()).collect();
    let mut j = serde_json::json!({"height": 10});
    for (k, v) in extra.as_object().unwrap() {
        j[k] = v.clone();
    }
    let body = flange(&tray, &edges, j).unwrap();
    (tray, body)
}

/// The vertices of `b` in the rims' layer (above the walls' bends).
fn rim_vertices(b: &Body, z_above: f64) -> Vec<Vec3> {
    (0..b.vertices().len() as u32).map(|v| b.point(v)).filter(|p| p.z > z_above).collect()
}

/// Whether the rim face with centroid `c` lies over the tray (inside the walls' outer faces, which
/// stand 4 out from the plate of 100 x 60), not outside them.
fn lies_inward(c: Vec3) -> bool {
    if ((c.x - 50.0) / 50.0).abs() > ((c.y - 30.0) / 30.0).abs() { (c.x - 50.0).abs() < 54.0 } else { (c.y - 30.0).abs() < 34.0 }
}

/// The horizontal faces of the rims of `b`: two for each rim, in the rims' layer (the walls' caps
/// are smaller).
fn rim_planes(b: &Body) -> Vec<usize> {
    plane_faces(b)
        .into_iter()
        .filter(|&(x, nz, _)| (nz.abs() - 1.0).abs() < 1e-9 && centroid(b, x).z > 17.0 && b.face_area(x as u32) > 300.0)
        .map(|f| f.0)
        .collect()
}

/// A tray offers the edges of all its walls, before rims and after: each wall's cap takes a flange
/// on either edge, whatever the other walls carry.
#[test]
fn every_wall_of_a_tray_offers_its_edges() {
    let (tray, caps) = tray_and_caps();
    assert_eq!(flange_edges(&tray).len(), 24);
    for cap in &caps {
        assert_eq!(long_edges(&tray, cap).len(), 2);
        for edge in long_edges(&tray, cap) {
            flange(&tray, &[edge], serde_json::json!({"height": 10})).unwrap();
        }
    }
    for (w, cap) in caps.iter().enumerate() {
        let mut body = tray.clone();
        for other in caps.iter().enumerate().filter(|&(n, _)| n != w).map(|(_, c)| c) {
            body = flange(&body, &long_edges(&body, other)[..1], serde_json::json!({"height": 10})).unwrap();
        }
        assert!(flange_edges(&body).len() > 24, "{} edges", flange_edges(&body).len());
        for edge in long_edges(&body, cap) {
            flange(&body, &[edge], serde_json::json!({"height": 10})).unwrap();
        }
    }
    let (_, all) = tray_with_rims(serde_json::json!({}));
    assert!(flange_edges(&all).len() > 24, "{} edges", flange_edges(&all).len());
}

/// Four rims picked in one flange are the four rims made one by one: volume, faces, box and blank.
#[test]
fn four_rims_at_once_are_four_rims_one_by_one() {
    let (tray, caps) = tray_and_caps();
    let (_, once) = tray_with_rims(serde_json::json!({}));
    let mut each = tray;
    for cap in &caps {
        each = flange(&each, &long_edges(&each, cap)[..1], serde_json::json!({"height": 10})).unwrap();
    }
    assert!(close(once.volume(), each.volume()), "{} {}", once.volume(), each.volume());
    assert_eq!(once.faces().len(), each.faces().len());
    let (a, b) = (once.bounds(), each.bounds());
    for (p, q) in [(a.0, b.0), (a.1, b.1)] {
        assert!((p - q).dot(p - q) < 1e-18, "{a:?} {b:?}");
    }
    let (sa, sb) = (unfold_side(&once, false, K).unwrap(), unfold_side(&each, false, K).unwrap());
    assert!(close(sa.size.x, sb.size.x) && close(sa.size.y, sb.size.y), "{:?} {:?}", sa.size, sb.size);
}

/// Rims turn toward the sheet's front side (inward on a tray) unless flipped, all of them alike:
/// picked at once or one on each wall in turn.
#[test]
fn rims_all_turn_the_same_way() {
    let (tray, caps) = tray_and_caps();
    for (flip, lo, hi) in [(false, -4.0, 104.0), (true, -12.0, 112.0)] {
        let extra = serde_json::json!({"flip": flip});
        let (_, all) = tray_with_rims(extra.clone());
        let (min, max) = all.bounds();
        let (ylo, yhi) = if flip { (-12.0, 72.0) } else { (-4.0, 64.0) };
        assert!((min.x - lo).abs() < 1e-9 && (max.x - hi).abs() < 1e-9, "{min:?} {max:?}");
        assert!((min.y - ylo).abs() < 1e-9 && (max.y - yhi).abs() < 1e-9, "{min:?} {max:?}");
        let faces = rim_planes(&all);
        assert_eq!(faces.len(), 8);
        for x in faces {
            assert_eq!(lies_inward(centroid(&all, x)), !flip, "{:?}", centroid(&all, x));
        }
        for cap in &caps {
            let one = flange(&tray, &long_edges(&tray, cap)[..1], serde_json::json!({"height": 10, "flip": flip})).unwrap();
            let faces = rim_planes(&one);
            assert_eq!(faces.len(), 2);
            for x in faces {
                assert_eq!(lies_inward(centroid(&one, x)), !flip, "{:?}", centroid(&one, x));
            }
        }
    }
}

/// The rims of a tray mitred: at each corner the two free edges end `g` apart square to a plane at
/// 45 degrees through the corner, the blank lies out whole. The volume differs from square rims by
/// what the mitres add at the eight ends: a flat part's profile of area A (6 x 2) at y (the wall's
/// outer plane at -4, its end at 0) has its end at -y - g/sqrt(2) beyond the wall's end, so each
/// end adds -(first moment of the profile in y) - A g/sqrt(2).
#[test]
fn mitred_rims_meet_at_45_degrees_a_gap_apart() {
    let g = 1.0;
    // The bends end square at the walls' ends: only the flat parts (12 square, from 0 to 6 in or
    // 6 to 12 out) are mitred.
    let flat = 12.0;
    let moment = |out: bool| if out { -flat * 9.0 } else { flat * 3.0 };
    for flip in [true, false] {
        let (_, square) = tray_with_rims(serde_json::json!({"flip": flip}));
        let (_, mitred) = tray_with_rims(serde_json::json!({"flip": flip, "corner": "miter", "corner_gap": g}));
        let layer = rim_vertices(&mitred, 17.0);
        let (lo, hi) = if flip { (-12.0, 112.0) } else { (6.0, 94.0) };
        let (ylo, yhi) = if flip { (-12.0, 72.0) } else { (6.0, 54.0) };
        for (cx, ex, sx) in [(-4.0, lo, true), (104.0, hi, false)] {
            for (cy, ey, sy) in [(-4.0, ylo, true), (64.0, yhi, false)] {
                let top = layer.iter().fold(f64::MIN, |m, p| m.max(p.z));
                let on_top = |p: &&Vec3| (p.z - top).abs() < 1e-9;
                // The free edge along X ends nearest the corner at `a`, the one along Y at `b`.
                let ends_a: Vec<Vec3> = layer.iter().filter(on_top).filter(|p| (p.y - ey).abs() < 1e-9).copied().collect();
                let ends_b: Vec<Vec3> = layer.iter().filter(on_top).filter(|p| (p.x - ex).abs() < 1e-9).copied().collect();
                let near = |v: &[Vec3], x: bool, s: bool| {
                    let key = |p: &Vec3| if x { p.x } else { p.y };
                    *v.iter().min_by(|p, q| if s { key(p).total_cmp(&key(q)) } else { key(q).total_cmp(&key(p)) }).unwrap()
                };
                let (pa, pb) = (
                    near(&ends_a.iter().copied().filter(|p| (p.y < 30.0) == sy).collect::<Vec<_>>(), true, sx),
                    near(&ends_b.iter().copied().filter(|p| (p.x < 50.0) == sx).collect::<Vec<_>>(), false, sy),
                );
                let d = pa - pb;
                assert!((d.dot(d).sqrt() - g).abs() < 1e-9, "flip {flip}: {pa:?} {pb:?}");
                assert!((d.x.abs() - d.y.abs()).abs() < 1e-9 && d.z.abs() < 1e-9, "45 degrees: {d:?}");
                // The middle between them lies on the diagonal through the walls' outer corner.
                let m = (pa + pb) * 0.5;
                assert!(((m.x - cx).abs() - (m.y - cy).abs()).abs() < 1e-9, "{m:?} at ({cx}, {cy})");
            }
        }
        let per_end = -moment(flip) - flat * g / std::f64::consts::SQRT_2;
        let (have, want) = (mitred.volume() - square.volume(), 8.0 * per_end);
        assert!((have - want).abs() < 1e-6 * want.abs(), "flip {flip}: {have} against {want}");
        let f = unfold_side(&mitred, false, K).unwrap();
        assert!(f.overlaps.is_empty(), "{:?}", f.overlaps);
        assert_eq!(f.bends.len(), 16);
        let back = unfold_side(&mitred, true, K).unwrap();
        assert!(close(f.size.x, back.size.x) && close(f.size.y, back.size.y), "{:?} {:?}", f.size, back.size);
    }
}

/// Whether all vertices of `b` above `z_above` within 1.5 of the plate's side `x` = `side` lie on it.
fn rim_ends_square(b: &Body, side: f64, z_above: f64) -> bool {
    rim_vertices(b, z_above).iter().filter(|p| (p.x - side).abs() < 1.5).all(|p| (p.x - side).abs() < 1e-9)
}

/// Rims on three walls only: the ends of the rims on the missing wall's side have no neighbour and
/// stay square, the other corners are mitred.
#[test]
fn a_rim_end_without_a_neighbour_stays_square() {
    let (tray, caps) = tray_and_caps();
    // The wall at x = 100: the one whose cap has its centre there.
    let at = |c: &String| centroid(&tray, (0..tray.faces().len()).find(|&x| hex(&tray, x) == *c).unwrap());
    let missing = caps.iter().position(|c| at(c).x > 90.0).unwrap();
    let edges: Vec<String> = caps.iter().enumerate().filter(|&(n, _)| n != missing).map(|(_, c)| long_edges(&tray, c)[0].clone()).collect();
    let b = flange(&tray, &edges, serde_json::json!({"height": 10, "corner": "miter", "corner_gap": 1})).unwrap();
    // Every vertex of the rims within 1.5 of x = 100 lies on it: square ends, no mitre there.
    let layer = rim_vertices(&b, 17.0);
    assert!(layer.iter().any(|p| (p.x - 100.0).abs() < 1e-9), "{layer:?}");
    assert!(rim_ends_square(&b, 100.0, 17.0));
    // The corners at x = 0 are mitred: some rim vertex stands off the wall's end at x = 0.
    assert!(layer.iter().any(|p| p.x > 0.5 && p.x < 8.0), "{layer:?}");
}

/// A tray whose wall at y < 0 is 30 high, the others 20: its rim is not in the plane of the others
/// and stays square at both ends; the three others in one plane are mitred where they meet.
#[test]
fn rims_in_other_planes_stay_square() {
    let p = plate();
    let long = faces_of_area(&p, 100.0 * T);
    let at = |f: &String| centroid(&p, (0..p.faces().len()).find(|&x| hex(&p, x) == *f).unwrap());
    let front = long.iter().find(|f| at(f).y < 30.0).unwrap();
    let ends = [long.clone(), faces_of_area(&p, 60.0 * T)].concat();
    let rest: Vec<String> = ends.iter().filter(|f| *f != front).cloned().collect();
    let low = flange(&p, &ends_edges(&p, &rest), serde_json::json!({})).unwrap();
    let tray = flange(&low, &long_edges(&p, front), serde_json::json!({"height": 30})).unwrap();
    let tops: Vec<String> = flange_ends(&tray).iter().map(|e| format!("{e:?}")).collect();
    assert_eq!(tops.len(), 4);
    let edges: Vec<String> = tops.iter().map(|t| long_edges(&tray, t)[0].clone()).collect();
    let b = flange(&tray, &edges, serde_json::json!({"height": 10, "corner": "miter", "corner_gap": 1})).unwrap();
    // The tall wall's rim, above z = 26: its ends at x = 0 and x = 100, as the square rim has.
    let tall_rim = rim_vertices(&b, 26.0);
    assert!(!tall_rim.is_empty());
    assert!(tall_rim.iter().all(|p| p.x > -1e-9 && p.x < 100.0 + 1e-9), "{tall_rim:?}");
    // The back rim, in the plane of the two side rims, ends short of the wall's end by a mitre.
    let back: Vec<Vec3> = rim_vertices(&b, 17.0).into_iter().filter(|p| p.z < 25.0 && p.y > 30.0).collect();
    assert!(back.iter().any(|p| p.x > 0.5 && p.x < 8.0), "{back:?}");
}

/// A mitre needs a gap, as a closed corner of walls does.
#[test]
fn a_mitre_of_rims_needs_a_gap() {
    let (tray, caps) = tray_and_caps();
    let edges: Vec<String> = caps.iter().map(|c| long_edges(&tray, c)[0].clone()).collect();
    assert!(flange(&tray, &edges, serde_json::json!({"height": 10, "corner": "miter"})).is_err());
    assert!(flange(&tray, &edges, serde_json::json!({"height": 10, "corner": "miter", "corner_gap": 1})).is_ok());
}

// ---- Tabs on a profile's end (D233) ----

/// Width of the profiles below, along Z.
const LONG: f64 = 300.0;

/// A sheet along the path through `pts`, on its left, `LONG` wide.
fn profile(pts: &[[f64; 2]]) -> Body {
    let names: Vec<String> = (0..pts.len()).map(|i| format!("p{i}")).collect();
    let points: serde_json::Map<String, serde_json::Value> = names.iter().zip(pts).map(|(n, p)| (n.clone(), serde_json::json!(p))).collect();
    let lines: serde_json::Map<String, serde_json::Value> =
        (1..pts.len()).map(|i| (format!("l{i}"), serde_json::json!({"from": names[i - 1], "to": names[i]}))).collect();
    let path = op(serde_json::json!({"op": "sketch", "sketch": {"points": points, "lines": lines}})).eval(id("path"), &[]).unwrap();
    let s = op(serde_json::json!({"op": "sheet", "thickness": T, "radius": R, "side": "left", "z0": 0, "z1": LONG}));
    s.eval(id("sheet"), &[&path]).unwrap().body().unwrap().clone()
}

/// The edges the sheet offers on wall `wall` of the path `pts` at its end `z`: the wall's outer and
/// inner edge, each with its ID and ends.
fn cap_edges(b: &Body, pts: &[[f64; 2]], wall: usize, z: f64) -> Vec<(String, [Vec3; 2])> {
    let (a, c) = (pts[wall], pts[wall + 1]);
    let len = ((c[0] - a[0]).powi(2) + (c[1] - a[1]).powi(2)).sqrt();
    let (dx, dy) = ((c[0] - a[0]) / len, (c[1] - a[1]) / len);
    let found: Vec<(String, [Vec3; 2])> = flange_edges(b)
        .into_iter()
        .filter(|(_, e)| e.iter().all(|p| (p.z - z).abs() < 1e-9))
        .filter(|(_, e)| {
            let m = (e[0] + e[1]) * 0.5;
            let (rx, ry) = (m.x - a[0], m.y - a[1]);
            let (t, n) = (rx * dx + ry * dy, dx * ry - dy * rx);
            t > 0.0 && t < len && n > -1e-6 && n < T + 1e-6
        })
        .map(|(e, p)| (format!("{e:?}"), p))
        .collect();
    assert_eq!(found.len(), 2, "wall {wall} at z {z}");
    found
}

/// The unit direction of wall `wall` of the path `pts`.
fn wall_dir(pts: &[[f64; 2]], wall: usize) -> Vec3 {
    let d = Vec3::new(pts[wall + 1][0] - pts[wall][0], pts[wall + 1][1] - pts[wall][1], 0.0);
    d * (1.0 / d.dot(d).sqrt())
}

/// The start of the edge `e` of `b` (its end with the lower vertex ID), the unit way to its other
/// end and its length.
fn along(b: &Body, e: [Vec3; 2]) -> (Vec3, Vec3, f64) {
    let vertex = |p: Vec3| b.vertices().iter().find(|v| (v.point - p).dot(v.point - p) < 1e-18).unwrap().id;
    let (from, to) = if vertex(e[0]) < vertex(e[1]) { (e[0], e[1]) } else { (e[1], e[0]) };
    let d = to - from;
    let len = d.dot(d).sqrt();
    (from, d * (1.0 / len), len)
}

fn edge_length(e: [Vec3; 2]) -> f64 {
    let d = e[1] - e[0];
    d.dot(d).sqrt()
}

/// The vertices of the tab on the end z = 0, beyond the sheet.
fn tab_vertices(b: &Body) -> Vec<Vec3> {
    (0..b.vertices().len() as u32).map(|v| b.point(v)).filter(|p| p.z < -1e-9).collect()
}

/// Where the tab of `b` on the edge `e` (of the sheet before it) lies along the edge, from its
/// start: the least and the greatest distance.
fn tab_span(before: &Body, b: &Body, e: [Vec3; 2]) -> (f64, f64) {
    let (from, way, _) = along(before, e);
    let t: Vec<f64> = tab_vertices(b).iter().map(|p| (*p - from).dot(way)).collect();
    (t.iter().copied().fold(f64::INFINITY, f64::min), t.iter().copied().fold(f64::NEG_INFINITY, f64::max))
}

/// Which side of the wall (of direction `d`, through `e`) the tab of `b` lies: positive on its left.
fn tab_side(b: &Body, e: [Vec3; 2], d: Vec3) -> f64 {
    let v = tab_vertices(b);
    let mean = v.iter().fold(Vec3::ZERO, |s, p| s + *p) * (1.0 / v.len() as f64);
    let m = (e[0] + e[1]) * 0.5;
    (mean.x - m.x) * -d.y + (mean.y - m.y) * d.x
}

fn tab(width: f64) -> serde_json::Value {
    serde_json::json!({"extent": {"kind": "center", "width": width}})
}

const PROFILES: [(&str, &[[f64; 2]]); 2] =
    [("V", &[[0.0, 60.0], [0.0, 0.0], [60.0, 0.0]]), ("C", &[[-40.0, 40.0], [0.0, 0.0], [60.0, 0.0], [100.0, 40.0]])];

/// A tab of 20 on a wall's edge at the sheet's end, on every wall of a V and of a C (45 degree
/// walls too): the volume that of a flange 20 wide (the full flange's by the edge's length), the
/// blank the full flange's in size and bends, laid whole from either side.
#[test]
fn a_tab_on_a_walls_cap_edge_is_a_narrow_flange() {
    for (name, pts) in PROFILES {
        let b = profile(pts);
        let base = unfold_side(&b, false, K).unwrap();
        for wall in 0..pts.len() - 1 {
            for (edge, e) in cap_edges(&b, pts, wall, 0.0) {
                let what = format!("{name} wall {wall}");
                let full = flange(&b, std::slice::from_ref(&edge), serde_json::json!({})).unwrap();
                let narrow = flange(&b, &[edge], tab(20.0)).unwrap();
                assert!(close(full.volume() - b.volume(), added(1.0, edge_length(e), 20.0, 90.0)), "{what}: {}", full.volume() - b.volume());
                assert!(close(narrow.volume() - b.volume(), added(1.0, 20.0, 20.0, 90.0)), "{what}: {}", narrow.volume() - b.volume());
                let (f, n) = (unfold_side(&full, false, K).unwrap(), unfold_side(&narrow, false, K).unwrap());
                assert!(close(n.size.x, f.size.x) && close(n.size.y, f.size.y), "{what}: {:?} {:?}", n.size, f.size);
                assert!(close(n.size.y, LONG + longer(20.0, 90.0)) && close(n.size.x, base.size.x), "{what}: {:?}", n.size);
                assert_eq!(f.bends.len(), base.bends.len() + 2, "{what}");
                assert_eq!(n.bends.len(), f.bends.len(), "{what}");
                assert!(n.overlaps.is_empty(), "{what}: {:?}", n.overlaps);
                let back = unfold_side(&narrow, true, K).unwrap();
                assert!(close(back.size.x, n.size.x) && close(back.size.y, n.size.y), "{what}: {:?} {:?}", back.size, n.size);
                assert_eq!(back.bends.len(), n.bends.len(), "{what}");
            }
        }
    }
}

/// The width's place along a wall's edge, from the edge's start: in the middle, `offset` off the
/// start or off the end, or all but `start` and `end`; a width that does not fit is refused.
#[test]
fn a_tabs_width_lies_where_the_extent_says() {
    for (name, pts) in PROFILES {
        let b = profile(pts);
        for wall in 0..pts.len() - 1 {
            let (edge, e) = cap_edges(&b, pts, wall, 0.0).remove(0);
            let len = along(&b, e).2;
            for (extent, want) in [
                (serde_json::json!({"kind": "center", "width": 20}), (len / 2.0 - 10.0, len / 2.0 + 10.0)),
                (serde_json::json!({"kind": "offset", "offset": 5, "width": 20}), (5.0, 25.0)),
                (serde_json::json!({"kind": "offset", "offset": 5, "width": 20, "from_end": true}), (len - 25.0, len - 5.0)),
                (serde_json::json!({"kind": "between", "start": 8, "end": 11}), (8.0, len - 11.0)),
            ] {
                let t = flange(&b, std::slice::from_ref(&edge), serde_json::json!({"extent": extent})).unwrap();
                let (lo, hi) = tab_span(&b, &t, e);
                assert!((lo - want.0).abs() < 1e-7 && (hi - want.1).abs() < 1e-7, "{name} wall {wall} {extent}: {lo} {hi}, wanted {want:?}");
            }
            for extent in [
                serde_json::json!({"kind": "center", "width": len + 4.0}),
                serde_json::json!({"kind": "offset", "offset": len - 10.0, "width": 20}),
                serde_json::json!({"kind": "between", "start": len / 2.0 + 1.0, "end": len / 2.0 + 1.0}),
            ] {
                let r = flange(&b, std::slice::from_ref(&edge), serde_json::json!({"extent": extent}));
                assert!(matches!(r, Err(BrepError::Degenerate(_))), "{name} wall {wall} {extent}: {r:?}");
            }
        }
    }
}

/// The inner and the outer edge of a wall make the same tab, and both together make one.
#[test]
fn both_edges_of_a_walls_cap_make_one_tab() {
    for (name, pts) in PROFILES {
        let b = profile(pts);
        for wall in 0..pts.len() - 1 {
            let edges = cap_edges(&b, pts, wall, 0.0);
            let extra = serde_json::json!({"extent": {"kind": "offset", "offset": 5, "width": 20}});
            let one = flange(&b, &[edges[0].0.clone()], extra.clone()).unwrap();
            let other = flange(&b, &[edges[1].0.clone()], extra.clone()).unwrap();
            let both = flange(&b, &[edges[0].0.clone(), edges[1].0.clone()], extra).unwrap();
            let what = format!("{name} wall {wall}");
            assert!(close(one.volume(), other.volume()) && close(one.volume(), both.volume()), "{what}");
            assert_eq!(sorted_ids(&one), sorted_ids(&other), "{what}");
            assert_eq!(sorted_ids(&one), sorted_ids(&both), "{what}");
            let spans: Vec<(f64, f64)> = [&one, &other, &both].iter().map(|t| tab_span(&b, t, edges[0].1)).collect();
            assert!(spans.iter().all(|s| (s.0 - spans[0].0).abs() < 1e-7 && (s.1 - spans[0].1).abs() < 1e-7), "{what}: {spans:?}");
        }
    }
}

/// A narrow tab turns the way a tab on the whole wall does, flipped or not, and `flip` turns it
/// the other way.
#[test]
fn a_narrow_tab_turns_as_a_full_one() {
    for (name, pts) in PROFILES {
        let b = profile(pts);
        for wall in 0..pts.len() - 1 {
            let d = wall_dir(pts, wall);
            for (edge, e) in cap_edges(&b, pts, wall, 0.0) {
                let mut sides = Vec::new();
                for flip in [false, true] {
                    let full = flange(&b, std::slice::from_ref(&edge), serde_json::json!({"flip": flip})).unwrap();
                    let mut j = tab(20.0);
                    j["flip"] = serde_json::json!(flip);
                    let narrow = flange(&b, std::slice::from_ref(&edge), j).unwrap();
                    let (f, n) = (tab_side(&full, e, d), tab_side(&narrow, e, d));
                    assert!(f * n > 4.0, "{name} wall {wall} flip {flip}: full {f}, narrow {n}");
                    sides.push(f);
                }
                assert!(sides[0] * sides[1] < 0.0, "{name} wall {wall}: {sides:?}");
            }
        }
    }
}

/// A bend relief beside a tab, square or round: the blank is as large from either side and not
/// turned against the one without relief, with as many bends; the slots take their material.
#[test]
fn a_bend_relief_beside_a_tab_keeps_the_blank() {
    let (w, d) = (2.0, 3.0);
    let half_disc = std::f64::consts::PI * 0.125 * w * w;
    let mut wrong = Vec::new();
    for (name, pts) in PROFILES {
        let b = profile(pts);
        for wall in 0..pts.len() - 1 {
            let (edge, _) = cap_edges(&b, pts, wall, 0.0).remove(0);
            let plain = flange(&b, std::slice::from_ref(&edge), tab(20.0)).unwrap();
            let (p, p_back) = (unfold_side(&plain, false, K).unwrap(), unfold_side(&plain, true, K).unwrap());
            for (shape, slot) in [("square", w * d), ("round", w * (d - 0.5 * w) + half_disc)] {
                let what = format!("{name} wall {wall} {shape}");
                let mut j = tab(20.0);
                j["bend_relief"] = serde_json::json!(shape);
                j["bend_relief_width"] = serde_json::json!(w);
                j["bend_relief_depth"] = serde_json::json!(d);
                let r = flange(&b, std::slice::from_ref(&edge), j).unwrap();
                assert!(close(plain.volume() - r.volume(), 2.0 * slot * T), "{what}: {}", plain.volume() - r.volume());
                let (front, back) = (unfold_side(&r, false, K).unwrap(), unfold_side(&r, true, K).unwrap());
                let same = |a: &datum_ops::Flat, b: &datum_ops::Flat| close(a.size.x, b.size.x) && close(a.size.y, b.size.y);
                if !same(&front, &back) {
                    wrong.push(format!("{what}: front {:?}, back {:?}", front.size, back.size));
                }
                if !same(&front, &p) {
                    wrong.push(format!("{what}: front {:?}, without relief {:?}", front.size, p.size));
                }
                if !same(&back, &p_back) {
                    wrong.push(format!("{what}: back {:?}, without relief {:?}", back.size, p_back.size));
                }
                if front.bends.len() != p.bends.len() || back.bends.len() != p_back.bends.len() {
                    wrong.push(format!(
                        "{what}: bends {} {}, without relief {} {}",
                        front.bends.len(),
                        back.bends.len(),
                        p.bends.len(),
                        p_back.bends.len()
                    ));
                }
                if !front.overlaps.is_empty() {
                    wrong.push(format!("{what}: overlaps {:?}", front.overlaps));
                }
            }
        }
    }
    assert!(wrong.is_empty(), "{}", wrong.join("\n"));
}

/// The corners of the tabs' outer faces (horizontal, at the lowest z of `b`), one polygon per tab.
fn tab_faces(b: &Body) -> Vec<Vec<Vec3>> {
    let low = b.bounds().0.z;
    plane_faces(b)
        .into_iter()
        .filter(|&(x, nz, _)| nz.abs() > 1.0 - 1e-9 && corners(b, x).iter().all(|p| (p.z - low).abs() < 1e-9))
        .map(|f| corners(b, f.0))
        .collect()
}

/// The wall of the path `pts` nearest to the point `p`.
fn nearest_wall(pts: &[[f64; 2]], p: Vec3) -> usize {
    let distance = |i: usize| {
        let (a, d) = (Vec3::new(pts[i][0], pts[i][1], 0.0), wall_dir(pts, i));
        let len = Vec3::new(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1], 0.0);
        let t = (p - a).dot(d).clamp(0.0, len.dot(d));
        let q = a + d * t - p;
        libm::hypot(q.x, q.y)
    };
    (0..pts.len() - 1).min_by(|&i, &j| distance(i).total_cmp(&distance(j))).unwrap()
}

/// The edges of the tab faces of `b` on each wall of `pts` that are neither along the wall nor
/// square to it: its mitres.
fn mitre_edges(b: &Body, pts: &[[f64; 2]]) -> Vec<Vec<(Vec3, Vec3)>> {
    let mut found = vec![Vec::new(); pts.len() - 1];
    for face in tab_faces(b) {
        let c = face.iter().fold(Vec3::ZERO, |s, p| s + *p) * (1.0 / face.len() as f64);
        let wall = nearest_wall(pts, c);
        let d = wall_dir(pts, wall);
        for (i, &p) in face.iter().enumerate() {
            let q = face[(i + 1) % face.len()];
            let u = q - p;
            let u = u * (1.0 / u.dot(u).sqrt());
            if u.dot(d).abs() > 1e-6 && u.dot(d).abs() < 1.0 - 1e-6 {
                found[wall].push((p, q));
            }
        }
    }
    found
}

/// The distance between the mitre edges `a` and `b` square to them, if they are parallel.
fn mitre_gap(a: (Vec3, Vec3), b: (Vec3, Vec3)) -> f64 {
    let u = a.1 - a.0;
    let u = u * (1.0 / u.dot(u).sqrt());
    let v = b.1 - b.0;
    let v = v * (1.0 / v.dot(v).sqrt());
    assert!(u.cross(v).dot(u.cross(v)) < 1e-18, "mitres not parallel: {u:?} {v:?}");
    let r = b.0 - a.0;
    r.cross(u).dot(r.cross(u)).sqrt()
}

/// The mitre edge of `edges` nearest to the corner of the path at `p`.
fn nearest_to(edges: &[(Vec3, Vec3)], p: [f64; 2]) -> (Vec3, Vec3) {
    let at = Vec3::new(p[0], p[1], 0.0);
    let d = |e: &(Vec3, Vec3)| {
        let m = (e.0 + e.1) * 0.5 - at;
        libm::hypot(m.x, m.y)
    };
    *edges.iter().min_by(|a, b| d(a).total_cmp(&d(b))).unwrap()
}

/// Tabs on all walls of a V and of a C picked together, turned inward, mitred: at each corner the
/// two mitres are parallel, `g` apart square to them, wherever the walls turn; the blank is laid
/// whole, the same from either side.
#[test]
fn mitred_tabs_on_a_profile_are_a_gap_apart() {
    let g = 1.0;
    for (name, pts) in PROFILES {
        let b = profile(pts);
        let edges: Vec<String> = (0..pts.len() - 1).map(|w| cap_edges(&b, pts, w, 0.0).remove(0).0).collect();
        let t = flange(&b, &edges, serde_json::json!({"corner": "miter", "corner_gap": g})).unwrap();
        let square = flange(&b, &edges, serde_json::json!({})).unwrap();
        assert_eq!(tab_faces(&t).len(), pts.len() - 1, "{name}");
        let mitres = mitre_edges(&t, pts);
        for corner in 1..pts.len() - 1 {
            let (a, c) = (nearest_to(&mitres[corner - 1], pts[corner]), nearest_to(&mitres[corner], pts[corner]));
            let gap = mitre_gap(a, c);
            assert!((gap - g).abs() < 1e-7, "{name} corner {corner}: gap {gap}");
        }
        // One mitre at each end of the tabs between corners, none at the path's ends.
        let counts: Vec<usize> = mitres.iter().map(Vec::len).collect();
        let want: Vec<usize> = (0..pts.len() - 1).map(|w| usize::from(w > 0) + usize::from(w < pts.len() - 2)).collect();
        assert_eq!(counts, want, "{name}");
        assert!(t.volume() != square.volume());
        let (f, back) = (unfold_side(&t, false, K).unwrap(), unfold_side(&t, true, K).unwrap());
        assert!(f.overlaps.is_empty(), "{name}: {:?}", f.overlaps);
        assert!(close(f.size.x, back.size.x) && close(f.size.y, back.size.y), "{name}: {:?} {:?}", f.size, back.size);
        assert_eq!(f.bends.len(), unfold_side(&b, false, K).unwrap().bends.len() + 2 * (pts.len() - 1), "{name}");
    }
}

/// Tabs on the first two walls of a C, each 20 long from their corner: the end at the corner is
/// mitred a gap from the other tab's, the end with the rest of its wall beside it stays square, at
/// the tab's width.
#[test]
fn a_tab_end_beside_its_wall_stays_square() {
    let g = 1.0;
    let (_, pts) = PROFILES[1];
    let b = profile(pts);
    let corner = Vec3::new(pts[1][0], pts[1][1], 0.0);
    let near = |p: Vec3| (p - corner).dot(p - corner) < 100.0;
    // A width is measured from the start of the wall's first edge: both walls start away from the
    // corner here, so `from_end` puts both tabs at it.
    let walls: Vec<(String, [Vec3; 2])> = (0..2).map(|wall| cap_edges(&b, pts, wall, 0.0).remove(0)).collect();
    assert!(walls.iter().all(|(_, e)| !near(along(&b, *e).0)));
    let edges: Vec<String> = walls.iter().map(|w| w.0.clone()).collect();
    let extent = serde_json::json!({"kind": "offset", "offset": 0, "width": 20, "from_end": true});
    let t = flange(&b, &edges, serde_json::json!({"extent": extent, "corner": "miter", "corner_gap": g})).unwrap();
    let faces = tab_faces(&t);
    assert_eq!(faces.len(), 2);
    for face in &faces {
        let c = face.iter().fold(Vec3::ZERO, |s, p| s + *p) * (1.0 / face.len() as f64);
        let wall = nearest_wall(pts, c);
        // The tab's length from the end of its edge at the corner, along the wall.
        let e = walls[wall].1;
        let (at, other) = if near(e[0]) { (e[0], e[1]) } else { (e[1], e[0]) };
        let away = (other - at) * (1.0 / edge_length(e));
        let s: Vec<f64> = face.iter().map(|p| (*p - at).dot(away)).collect();
        let far = s.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        assert!((far - 20.0).abs() < 1e-7, "wall {wall}: the square end at the width, not {far}");
        assert_eq!(s.iter().filter(|x| (**x - far).abs() < 1e-9).count(), 2, "wall {wall}: a square end in {face:?}");
    }
    let mitres = mitre_edges(&t, pts);
    assert_eq!(mitres.iter().map(Vec::len).collect::<Vec<_>>(), [1, 1, 0]);
    let gap = mitre_gap(mitres[0][0], mitres[1][0]);
    assert!((gap - g).abs() < 1e-7, "gap {gap}");
    let f = unfold_side(&t, false, K).unwrap();
    assert!(f.overlaps.is_empty(), "{:?}", f.overlaps);
    let back = unfold_side(&t, true, K).unwrap();
    assert!(close(f.size.x, back.size.x) && close(f.size.y, back.size.y), "{:?} {:?}", f.size, back.size);
}

/// Tabs in the middle of their walls touch no corner: a mitre asked for changes nothing.
#[test]
fn tabs_in_the_middle_have_no_mitre() {
    for (name, pts) in PROFILES {
        let b = profile(pts);
        let edges: Vec<String> = (0..pts.len() - 1).map(|w| cap_edges(&b, pts, w, 0.0).remove(0).0).collect();
        let plain = flange(&b, &edges, tab(20.0)).unwrap();
        let mut j = tab(20.0);
        j["corner"] = serde_json::json!("miter");
        j["corner_gap"] = serde_json::json!(1);
        let mitred = flange(&b, &edges, j).unwrap();
        assert!(close(plain.volume(), mitred.volume()), "{name}");
        assert_eq!(plain.faces().len(), mitred.faces().len(), "{name}");
        assert!(mitre_edges(&mitred, pts).iter().all(Vec::is_empty), "{name}");
    }
}

/// A flange on the free end of a narrow tab: the blank grows by its strip, whole.
#[test]
fn a_tab_on_a_tab() {
    for (name, pts) in PROFILES {
        let b = profile(pts);
        let (edge, _) = cap_edges(&b, pts, 0, 0.0).remove(0);
        let first = flange(&b, &[edge], tab(20.0)).unwrap();
        let before: BTreeSet<TopoId> = flange_ends(&b).into_iter().collect();
        // The tab's free end: 20 wide and as thick as the sheet; the other new ends are the cap's
        // pieces beside the tab (16 or 18 long).
        let area = |e: TopoId| first.face_area(first.faces().iter().position(|f| f.id == e).unwrap() as u32);
        let tips: Vec<String> =
            flange_ends(&first).into_iter().filter(|e| !before.contains(e) && close(area(*e), 20.0 * T)).map(|e| format!("{e:?}")).collect();
        assert_eq!(tips.len(), 1, "{name}");
        let second = flange(&first, &long_edges(&first, &tips[0])[..1], serde_json::json!({"height": 10})).unwrap();
        let (one, two) = (unfold_side(&first, false, K).unwrap(), unfold_side(&second, false, K).unwrap());
        assert_eq!(two.bends.len(), one.bends.len() + 2, "{name}");
        // Which way the blank lies is not asked: its long side grows by the second strip.
        let dims = |f: &datum_ops::Flat| if f.size.x > f.size.y { (f.size.x, f.size.y) } else { (f.size.y, f.size.x) };
        assert!(close(dims(&two).0, dims(&one).0 + longer(10.0, 90.0)) && close(dims(&two).1, dims(&one).1), "{name}: {:?} {:?}", two.size, one.size);
        assert!(two.overlaps.is_empty(), "{name}: {:?}", two.overlaps);
        let back = unfold_side(&second, true, K).unwrap();
        assert!(close(back.size.x, two.size.x) && close(back.size.y, two.size.y), "{name}: {:?} {:?}", back.size, two.size);
    }
}

/// A tab on each end of the profile (z = 0 and z = 300): both build, the blank grows both ways.
#[test]
fn tabs_on_both_ends_of_a_profile() {
    for (name, pts) in PROFILES {
        let b = profile(pts);
        let base = unfold_side(&b, false, K).unwrap();
        let (low, _) = cap_edges(&b, pts, 0, 0.0).remove(0);
        let (high, _) = cap_edges(&b, pts, 0, LONG).remove(0);
        let t = flange(&b, &[low, high], tab(20.0)).unwrap();
        assert!(close(t.volume() - b.volume(), added(2.0, 20.0, 20.0, 90.0)), "{name}: {}", t.volume() - b.volume());
        let f = unfold_side(&t, false, K).unwrap();
        // Both ways: the blank's long side is the sheet's width and the two tabs' strips (which
        // way it lies is not asked).
        let (long, short) = if f.size.x > f.size.y { (f.size.x, f.size.y) } else { (f.size.y, f.size.x) };
        assert!(close(long, LONG + 2.0 * longer(20.0, 90.0)) && close(short, base.size.x), "{name}: {:?}", f.size);
        assert_eq!(f.bends.len(), base.bends.len() + 4, "{name}");
        assert!(f.overlaps.is_empty(), "{name}: {:?}", f.overlaps);
    }
}

/// A flat face's contours: a plate's four ends at once, and a window's edges apart from them; every
/// edge of a contour is one `flange_edges` offers, and the plate's flanged from it builds.
#[test]
fn a_faces_contours_are_what_a_click_takes() {
    let p = plate();
    let loops = datum_ops::flange_loops(&p);
    // Each of the two big faces: four ends.
    assert_eq!(loops.iter().map(|l| l.edges.len()).collect::<Vec<_>>(), [4, 4]);
    let offered: BTreeSet<TopoId> = flange_edges(&p).into_iter().map(|e| e.0).collect();
    assert_eq!(offered.len(), 8);
    assert!(loops.iter().flat_map(|l| &l.edges).all(|e| offered.contains(&e.0)));
    let picked: Vec<String> = loops[0].edges.iter().map(|e| format!("{:?}", e.0)).collect();
    let tray = flange(&p, &picked, serde_json::json!({})).unwrap();
    let plain = flange(&p, &ends_edges(&p, &[faces_of_area(&p, 100.0 * T), faces_of_area(&p, 60.0 * T)].concat()), serde_json::json!({})).unwrap();
    assert!(close(tray.volume(), plain.volume()));
    // A window: a second contour of the face, of its own four edges.
    let w = windowed();
    let loops = datum_ops::flange_loops(&w);
    assert_eq!(loops.iter().map(|l| l.edges.len()).collect::<Vec<_>>(), [4, 4, 4, 4]);
    assert_eq!(flange_edges(&w).len(), 16);
    assert_eq!(loops[0].face, loops[1].face);
}

/// A plate of 300 x 200 with a window of 100 x 100.
fn windowed() -> Body {
    windowed_by(100.0, 100.0)
}

/// A plate of 300 x 200 with a window of `wx` x `wy` from (100, 50).
fn windowed_by(wx: f64, wy: f64) -> Body {
    let (x1, y1) = (100.0 + wx, 50.0 + wy);
    let rect = serde_json::json!({"points": {"a": [0, 0], "b": [300, 0], "c": [300, 200], "d": [0, 200], "e": [100, 50], "f": [x1, 50], "g": [x1, y1], "h": [100, y1]},
        "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}, "cd": {"from": "c", "to": "d"}, "da": {"from": "d", "to": "a"},
                  "ef": {"from": "e", "to": "f"}, "fg": {"from": "f", "to": "g"}, "gh": {"from": "g", "to": "h"}, "he": {"from": "h", "to": "e"}}});
    let s = op(serde_json::json!({"op": "sketch", "sketch": rect})).eval(id("w.sketch"), &[]).unwrap();
    op(serde_json::json!({"op": "extrude", "z0": 0, "z1": T})).eval(id("w"), &[&s]).unwrap().body().unwrap().clone()
}

/// The bottom edges of the window of [`windowed`], one for each wall.
fn window_bottom(w: &Body) -> Vec<String> {
    [window_edges(w, true), window_edges(w, false)].concat()
}

/// The bottom edges of the window of [`windowed_by`] (up to 140 x 100) that run along X, or along Y.
fn window_edges(w: &Body, along_x: bool) -> Vec<String> {
    let window = |x: f64, y: f64| x > 50.0 && x < 250.0 && y > 25.0 && y < 175.0;
    flange_edges(w)
        .into_iter()
        .filter(|(_, e)| e.iter().all(|p| window(p.x, p.y) && p.z == 0.0) && (e[0].y == e[1].y) == along_x)
        .map(|e| format!("{:?}", e.0))
        .collect()
}

/// One flange on a window's edge, its side not joined to the wall beside it, which looks back along
/// the edge: it builds, tessellates (inside `flange`) and holds its material.
#[test]
fn a_flange_on_a_windows_edge() {
    let w = windowed();
    let edges = window_bottom(&w);
    assert_eq!(edges.len(), 4);
    for e in &edges {
        let b = flange(&w, std::slice::from_ref(e), serde_json::json!({"height": 20})).unwrap();
        assert!(close(b.volume() - w.volume(), added(1.0, 100.0, 20.0, 90.0)), "{}", b.volume() - w.volume());
        let f = unfold_side(&b, false, K).unwrap();
        assert_eq!(f.bends.len(), 2);
        assert!(f.overlaps.is_empty(), "{:?}", f.overlaps);
    }
}

/// `b` cut through by `sketch` drawn on its blank laid at K-factor `k`, seen from the front.
fn cut_sketch(b: &Body, sketch: serde_json::Value, k: f64) -> Result<Body, BrepError> {
    let target = Value::Body(b.clone());
    let drawn = op(serde_json::json!({"op": "sketch", "flat": "front", "k_factor": k, "sketch": sketch})).eval(id("slits"), &[&target])?;
    let r = op(serde_json::json!({"op": "extrude", "mode": "cut", "through": true, "wall": true}))
        .eval(id("slit"), &[&target, &drawn])?
        .body()
        .unwrap()
        .clone();
    r.validate().unwrap();
    tessellate(&r).unwrap();
    Ok(r)
}

/// The sketch of closed polygons with the given corners.
fn polygons_sketch(polygons: &[Vec<(f64, f64)>]) -> serde_json::Value {
    let (mut points, mut lines) = (serde_json::Map::new(), serde_json::Map::new());
    for (k, corners) in polygons.iter().enumerate() {
        for (j, c) in corners.iter().enumerate() {
            points.insert(format!("p{k}_{j}"), serde_json::json!([c.0, c.1]));
            lines.insert(format!("l{k}_{j}"), serde_json::json!({"from": format!("p{k}_{j}"), "to": format!("p{k}_{}", (j + 1) % corners.len())}));
        }
    }
    serde_json::json!({"points": points, "lines": lines})
}

/// `b` cut through by closed polygons drawn on its blank at [`K`], seen from the front.
fn cut_on_blank(b: &Body, polygons: &[Vec<(f64, f64)>]) -> Result<Body, BrepError> {
    cut_sketch(b, polygons_sketch(polygons), K)
}

/// A circle of `d` across at `at` on the blank.
fn circle_on_blank(at: (f64, f64), d: f64) -> serde_json::Value {
    serde_json::json!({"points": {"c": [at.0, at.1]}, "circles": {"h": {"center": "c", "diameter": d}}})
}

/// A slit `g` wide on the blank from `from` to `to`, running on `over` beyond both.
fn slit(from: (f64, f64), to: (f64, f64), g: f64, over: f64) -> Vec<(f64, f64)> {
    let (dx, dy) = (to.0 - from.0, to.1 - from.1);
    let l = (dx * dx + dy * dy).sqrt();
    let (u, n) = ((dx / l, dy / l), (-dy / l * 0.5 * g, dx / l * 0.5 * g));
    let (a, b) = ((from.0 - u.0 * over, from.1 - u.1 * over), (to.0 + u.0 * over, to.1 + u.1 * over));
    vec![(a.0 - n.0, a.1 - n.1), (b.0 - n.0, b.1 - n.1), (b.0 + n.0, b.1 + n.1), (a.0 + n.0, a.1 + n.1)]
}

/// The window on a blank of four walls: left, right, bottom and top of the bend lines' box.
fn window_box(bends: &[[datum_math::Vec2; 2]]) -> [f64; 4] {
    let at = |f: fn(&datum_math::Vec2) -> f64, pick: fn(f64, f64) -> f64, from: f64| bends.iter().flatten().map(f).fold(from, pick);
    [
        at(|p| p.x, f64::min, f64::INFINITY),
        at(|p| p.x, f64::max, f64::NEG_INFINITY),
        at(|p| p.y, f64::min, f64::INFINITY),
        at(|p| p.y, f64::max, f64::NEG_INFINITY),
    ]
}

/// The window's four corners with the way into the window from each.
fn window_corners([lo, hi, bottom, top]: [f64; 4]) -> [((f64, f64), (f64, f64)); 4] {
    [((lo, bottom), (1.0, 1.0)), ((hi, bottom), (-1.0, 1.0)), ((lo, top), (1.0, -1.0)), ((hi, top), (-1.0, -1.0))]
}

/// At each corner a slit `g` wide from the bend lines' corner through the point `(px, py)` into the window.
fn corner_slits(window: [f64; 4], (px, py): (f64, f64), g: f64, over: f64) -> Vec<Vec<(f64, f64)>> {
    window_corners(window).iter().map(|&((x, y), (sx, sy))| slit((x, y), (x + sx * px, y + sy * py), g, over)).collect()
}

/// Where a slit `g` wide through `(px, py)` leaves the free edges (`rx` from the bend line along X, `ry`
/// along Y) of the walls beside the corner: the wall along X keeps the side of the slit it lies on.
fn kept_ends((rx, ry): (f64, f64), (px, py): (f64, f64), g: f64) -> [(f64, f64); 2] {
    let l = (px * px + py * py).sqrt();
    [(rx, rx * py / px + g * l / (2.0 * px)), (ry * px / py + g * l / (2.0 * py), ry)]
}

/// The point `at` of the blank `before` on the blank `after`, which a cut may lay otherwise: both lie in
/// the plane of the sheet's front, so their frames carry the point over.
fn carry(before: &Flat, after: &Flat, at: (f64, f64)) -> (f64, f64) {
    let (from, to) = (before.frame.unwrap(), after.frame.unwrap());
    let p = from[0] + from[1] * at.0 + from[2] * at.1 - to[0];
    (p.dot(to[1]), p.dot(to[2]))
}

/// Whether the pieces of `after` have a vertex at `at` on the blank `before`.
fn has_vertex(after: &Flat, before: &Flat, at: (f64, f64)) -> bool {
    let (x, y) = carry(before, after, at);
    let near = |p: &datum_math::Vec2| (p.x - x).abs() < 1e-6 && (p.y - y).abs() < 1e-6;
    after.pieces.iter().any(|p| match p {
        FlatPiece::Line(a, b) => near(a) || near(b),
        FlatPiece::Polyline(q) => q.iter().any(near),
        _ => false,
    })
}

/// `b` with slits `g` wide across the four corners of its window through the points a flange of `h`
/// reaches to along both walls, as the blank of `b` has them.
fn settled(b: &Body, h: f64) -> Body {
    let window = window_box(&unfold_side(b, false, K).unwrap().bends);
    cut_on_blank(b, &corner_slits(window, (longer(h, 90.0), longer(h, 90.0)), 1.0, 3.0)).unwrap()
}

/// How many round holes the blank has, and how many the body has of radius `r`: a hole's wall is three
/// faces of a cylinder.
fn holes(b: &Body, r: f64) -> (usize, usize) {
    let circles = unfold_side(b, false, K).unwrap().pieces.iter().filter(|p| matches!(p, FlatPiece::Circle { .. })).count();
    let faces = b.faces().iter().filter(|f| matches!(f.surface, Surface::Cylinder(c) if (c.radius - r).abs() < 1e-9)).count();
    assert_eq!(faces % 3, 0, "{faces}");
    (circles, faces / 3)
}

/// The blank of `b` laid at `k` from both sides: of one size, the bends all kept, nothing on itself.
fn lies_flat(b: &Body, k: f64, bends: usize) -> Flat {
    let flat = unfold_side(b, false, k).unwrap();
    assert!(flat.overlaps.is_empty(), "{:?}", flat.overlaps);
    assert_eq!(flat.bends.len(), bends);
    let back = unfold_side(b, true, k).unwrap();
    assert!(close(back.size.x, flat.size.x) && close(back.size.y, flat.size.y), "{:?} {:?}", back.size, flat.size);
    flat
}

/// Flanges on all a window's edges: built, the blank telling where it lies on itself; slits across
/// the corners drawn on the blank part the overlap off, each strip keeping its side, at whatever
/// angle the slit goes: then the blank lies flat and the metal gone is what was doubled.
#[test]
fn a_windows_corners_are_settled_on_the_blank() {
    let w = windowed();
    let b = flange(&w, &window_bottom(&w), serde_json::json!({"height": 20})).unwrap();
    let f = unfold_side(&b, false, K).unwrap();
    assert_eq!(f.overlaps.len(), 4, "{:?}", f.overlaps);
    assert_eq!(f.bends.len(), 8);
    // The bend lines' corners on the blank, and how far the strips reach into the window.
    let window = window_box(&f.bends);
    let reach = f.overlaps[0][1].x - window[0];
    assert!(reach > 15.0 && reach < 25.0, "{reach}");
    // Overlap frames sit in the corners.
    assert!((f.overlaps[0][0].x - window[0]).abs() < 6.0 && (f.overlaps[0][0].y - window[2]).abs() < 6.0, "{:?}", f.overlaps[0]);
    for (name, tilt) in [("45", 1.0_f64), ("steeper", 1.5)] {
        let cut = cut_on_blank(&b, &corner_slits(window, (reach, reach * tilt), 1.0, 3.0)).unwrap();
        lies_flat(&cut, K, 8);
        assert!(cut.volume() < b.volume(), "{name}");
    }
}

/// The walls along one pair of edges higher than the other pair: the free edges meet off the diagonal,
/// and a slit from the bend lines' corner through that point settles each corner.
#[test]
fn walls_of_two_heights_are_settled_by_a_slit_through_the_free_edges() {
    let w = windowed_by(140.0, 90.0);
    let first = flange(&w, &window_edges(&w, true), serde_json::json!({"height": 20})).unwrap();
    let along_y = window_edges(&first, false);
    assert_eq!(along_y.len(), 2);
    let b = flange(&first, &along_y, serde_json::json!({"height": 30})).unwrap();
    let f = unfold_side(&b, false, K).unwrap();
    assert_eq!((f.overlaps.len(), f.bends.len()), (4, 8), "{:?}", f.overlaps);
    // The edges along X lie along the blank's Y, so the walls along them reach `rx` from the left and right.
    let (rx, ry) = (longer(20.0, 90.0), longer(30.0, 90.0));
    let window = window_box(&f.bends);
    let (lo, bottom) = (window[0], window[2]);
    let first_corner = f.overlaps.iter().find(|o| o[0].x < lo + 10.0 && o[0].y < bottom + 10.0).unwrap();
    assert!(close(first_corner[1].x - lo, rx) && close(first_corner[1].y - bottom, ry), "{first_corner:?}");
    let cut = cut_on_blank(&b, &corner_slits(window, (rx, ry), 1.0, 3.0)).unwrap();
    let flat = lies_flat(&cut, K, 8);
    let [left, down] = kept_ends((rx, ry), (rx, ry), 1.0);
    for &((x, y), (sx, sy)) in &window_corners(window) {
        assert!(has_vertex(&flat, &f, (x + sx * left.0, y + sy * left.1)), "{x} {y} {left:?}");
        assert!(has_vertex(&flat, &f, (x + sx * down.0, y + sy * down.1)), "{x} {y} {down:?}");
    }
}

/// Walls along one pair of edges leaning to 60 degrees, along the other square: the blank is the same
/// kind of cross, and the slit settles it as it does at right angles.
#[test]
fn walls_at_two_angles_are_settled_by_a_slit() {
    let w = windowed();
    let first = flange(&w, &window_edges(&w, true), serde_json::json!({"height": 20, "height_along": true})).unwrap();
    let b = flange(&first, &window_edges(&first, false), serde_json::json!({"height": 20, "height_along": true, "angle": 60})).unwrap();
    let f = unfold_side(&b, false, K).unwrap();
    assert_eq!((f.overlaps.len(), f.bends.len()), (4, 8), "{:?}", f.overlaps);
    let (rx, ry) = (longer(20.0, 90.0), longer(20.0, 60.0));
    let window = window_box(&f.bends);
    let cut = cut_on_blank(&b, &corner_slits(window, (rx, ry), 1.0, 3.0)).unwrap();
    let flat = lies_flat(&cut, K, 8);
    let [left, down] = kept_ends((rx, ry), (rx, ry), 1.0);
    let (lo, bottom) = (window[0], window[2]);
    assert!(has_vertex(&flat, &f, (lo + left.0, bottom + left.1)) && has_vertex(&flat, &f, (lo + down.0, bottom + down.1)));
}

/// A slit that is not at 45 degrees: each strip keeps the side of it that it lies on, so the free edges
/// end where the line, a half gap off, crosses them.
#[test]
fn a_slit_off_the_diagonal_gives_each_strip_its_side() {
    let w = windowed();
    let b = flange(&w, &window_bottom(&w), serde_json::json!({"height": 20})).unwrap();
    let before = unfold_side(&b, false, K).unwrap();
    let window = window_box(&before.bends);
    let r = longer(20.0, 90.0);
    for tilt in [1.5, 0.8] {
        let through = (r, r * tilt);
        let cut = cut_on_blank(&b, &corner_slits(window, through, 1.0, 10.0)).unwrap();
        let flat = lies_flat(&cut, K, 8);
        let [left, down] = kept_ends((r, r), through, 1.0);
        for &((x, y), (sx, sy)) in &window_corners(window) {
            assert!(has_vertex(&flat, &before, (x + sx * left.0, y + sy * left.1)), "{tilt}: {x} {y} {left:?}");
            assert!(has_vertex(&flat, &before, (x + sx * down.0, y + sy * down.1)), "{tilt}: {x} {y} {down:?}");
        }
    }
}

/// A flange's reach on the blank at K = 0.5, where the neutral layer lies mid-thickness.
fn reach_mid(h: f64) -> f64 {
    h - (R + T) + std::f64::consts::FRAC_PI_2 * (R + 0.5 * T)
}

/// At K = 0.5 the metal gone is the area gone times the thickness: for a 45 degree slit `g` wide the
/// triangle of each strip below or above it, and the notch in the plate; a wide cut over the corner takes
/// the square of both strips and no part falls off.
#[test]
fn what_a_slit_and_a_wide_cut_take_off_is_known_at_a_half() {
    let w = windowed();
    let b = flange(&w, &window_bottom(&w), serde_json::json!({"height": 20})).unwrap();
    let window = window_box(&unfold_side(&b, false, 0.5).unwrap().bends);
    let (g, over) = (1.0_f64, 3.0);
    let r = reach_mid(20.0);
    let slits = cut_sketch(&b, polygons_sketch(&corner_slits(window, (r, r), g, over)), 0.5).unwrap();
    // Per corner: the part of each strip beside the slit, and the notch in the plate.
    let c = g / 2.0_f64.sqrt();
    let gone = 4.0 * T * (2.0 * (r * r / 2.0 + c * r) + over * g + g * g / 4.0);
    assert!(close(b.volume() - slits.volume(), gone), "{} {gone}", b.volume() - slits.volume());
    lies_flat(&slits, 0.5, 8);
    // A triangle with legs `l` from `m` outside the corner on both sides, its hypotenuse past the strips' square.
    let (m, margin) = (3.0, 4.0);
    let s = 2.0 * r + margin;
    let triangles: Vec<Vec<(f64, f64)>> = window_corners(window)
        .iter()
        .map(|&((x, y), (sx, sy))| {
            let l = s + 2.0 * m;
            vec![(x - sx * m, y - sy * m), (x + sx * (l - m), y - sy * m), (x - sx * m, y + sy * (l - m))]
        })
        .collect();
    let wide = cut_sketch(&b, polygons_sketch(&triangles), 0.5).unwrap();
    let strips = 2.0 * (s * r - r * r / 2.0);
    let plate = m * (2.0 * s + 2.0 * m);
    assert!(close(b.volume() - wide.volume(), 4.0 * T * (strips + plate)), "{} ", b.volume() - wide.volume());
    lies_flat(&wide, 0.5, 8);
    assert!(wide.volume() < slits.volume());
}

/// Holes on a blank lying on itself: one in the plate by a corner is one hole, before the slits and after;
/// one in the walls where they lie on one another goes through both, and when the slits part one of the
/// strips off, its hole goes with it.
#[test]
fn holes_by_a_windows_corners_are_made_once_and_through_all_strips() {
    let w = windowed();
    let b = flange(&w, &window_bottom(&w), serde_json::json!({"height": 20})).unwrap();
    // 1.5 across: the bends' inner radius is another.
    let (d, r) = (3.0, 1.5);
    let lower_left = |b: &Body| {
        let window = window_box(&unfold_side(b, false, K).unwrap().bends);
        (window[0], window[2])
    };
    let at = |b: &Body, (dx, dy): (f64, f64)| {
        let (x, y) = lower_left(b);
        circle_on_blank((x + dx, y + dy), d)
    };
    // In the plate, off the diagonal the slit runs along.
    let plate_hole = cut_sketch(&b, at(&b, (-6.0, -14.0)), K).unwrap();
    assert_eq!(holes(&plate_hole, r), (1, 1));
    assert_eq!(unfold_side(&plate_hole, false, K).unwrap().overlaps.len(), 4);
    assert_eq!(holes(&settled(&plate_hole, 20.0), r), (1, 1));
    let slits_first = settled(&b, 20.0);
    let hole_after = cut_sketch(&slits_first, at(&slits_first, (-6.0, -14.0)), K).unwrap();
    assert_eq!(holes(&hole_after, r), (1, 1));
    lies_flat(&hole_after, K, 8);
    // In the walls where they lie on one another, below the diagonal: through both strips.
    let both = cut_sketch(&b, at(&b, (16.0, 8.0)), K).unwrap();
    assert_eq!(holes(&both, r), (2, 2));
    let parted = settled(&both, 20.0);
    lies_flat(&parted, K, 8);
    assert_eq!(holes(&parted, r), (1, 1));
}

/// Which corners are told: a frame at each corner of an unsettled window, its far corner where the free
/// edges meet; slits across two corners leave the other two told.
#[test]
fn a_window_settled_at_two_corners_tells_the_other_two() {
    let w = windowed();
    let b = flange(&w, &window_bottom(&w), serde_json::json!({"height": 20})).unwrap();
    let before = unfold_side(&b, false, K).unwrap();
    let window = window_box(&before.bends);
    let r = longer(20.0, 90.0);
    let meets: Vec<(f64, f64)> = window_corners(window).iter().map(|&((x, y), (sx, sy))| (x + sx * r, y + sy * r)).collect();
    // A frame is a box with the meeting point at a corner and the rest between there and the window's corner.
    let told = |flat: &Flat, corner: (f64, f64), meet: (f64, f64)| {
        let (c, m) = (carry(&before, flat, corner), carry(&before, flat, meet));
        let eq = |a: f64, b: f64| (a - b).abs() < 1e-9;
        flat.overlaps.iter().any(|o| {
            let (lo, hi) = ((o[0].x.min(o[1].x), o[0].y.min(o[1].y)), (o[0].x.max(o[1].x), o[0].y.max(o[1].y)));
            let at_meet = (eq(m.0, lo.0) || eq(m.0, hi.0)) && (eq(m.1, lo.1) || eq(m.1, hi.1));
            let within = lo.0 >= c.0.min(m.0) - 1e-9 && hi.0 <= c.0.max(m.0) + 1e-9 && lo.1 >= c.1.min(m.1) - 1e-9 && hi.1 <= c.1.max(m.1) + 1e-9;
            at_meet && within && hi.0 > lo.0 && hi.1 > lo.1
        })
    };
    let corners: Vec<(f64, f64)> = window_corners(window).iter().map(|c| c.0).collect();
    for (&c, &m) in corners.iter().zip(&meets) {
        assert!(told(&before, c, m), "{c:?} {:?}", before.overlaps);
    }
    let two = cut_on_blank(&b, &corner_slits(window, (r, r), 1.0, 3.0)[..2]).unwrap();
    let half = unfold_side(&two, false, K).unwrap();
    assert_eq!((half.overlaps.len(), half.bends.len()), (2, 8), "{:?}", half.overlaps);
    for k in 0..4 {
        assert_eq!(told(&half, corners[k], meets[k]), k >= 2, "corner {k}: {:?}", half.overlaps);
    }
}

/// A flange on a wall's end and flanges on both its cap edges, in one operation (a corner of two strips
/// beside each other, not on one another): built, unfolded with every bend, and nothing told on itself,
/// whichever way and as far as they lean.
#[test]
fn flanges_on_a_walls_end_and_cap_edges_lie_flat() {
    let u = u_of(MID);
    let end = ends(&u)
        .into_iter()
        .find(|f| {
            let x = (0..u.faces().len()).find(|&x| hex(&u, x) == *f).unwrap();
            centroid(&u, x).x < 1.0
        })
        .unwrap();
    let mut edges = vec![long_edges(&u, &end)[0].clone()];
    edges.push(wall_edge(&u, 0, -2.0, 0.0));
    edges.push(wall_edge(&u, 0, -2.0, WIDTH));
    for extra in [serde_json::json!({}), serde_json::json!({"angle": 120}), serde_json::json!({"angle": 60}), serde_json::json!({"flip": true})] {
        let b = flange(&u, &edges, extra.clone()).unwrap();
        // The U's two bends and the three flanges'.
        lies_flat(&b, K, 10);
        assert!(b.volume() > u.volume(), "{extra}");
    }
}

/// A line on a blank: its two ends.
type Segment = ((f64, f64), (f64, f64));

/// `b` cut by open lines drawn on its blank, a `kerf` wide.
fn line_cut_on_blank(b: &Body, segments: &[Segment], kerf: f64) -> Result<Body, BrepError> {
    let (mut points, mut lines) = (serde_json::Map::new(), serde_json::Map::new());
    for (k, (p, q)) in segments.iter().enumerate() {
        points.insert(format!("a{k}"), serde_json::json!([p.0, p.1]));
        points.insert(format!("b{k}"), serde_json::json!([q.0, q.1]));
        lines.insert(format!("l{k}"), serde_json::json!({"from": format!("a{k}"), "to": format!("b{k}")}));
    }
    let target = Value::Body(b.clone());
    let sketch = op(serde_json::json!({"op": "sketch", "flat": "front", "k_factor": K, "sketch": {"points": points, "lines": lines}}))
        .eval(id("lines"), &[&target])?;
    let r = op(serde_json::json!({"op": "extrude", "mode": "cut", "through": true, "wall": true, "kerf": kerf}))
        .eval(id(&format!("line.cut.{}", b.faces().len())), &[&target, &sketch])?
        .body()
        .unwrap()
        .clone();
    r.validate().unwrap();
    tessellate(&r).unwrap();
    Ok(r)
}

/// Lines on a window's corners: in the part a cut the kerf wide, as a slit that wide takes; on the
/// blank as the DXF has it, the gap closed: one line in each corner, the free edges running to it.
#[test]
fn a_line_cut_has_a_gap_in_the_part_and_none_in_the_blank() {
    let w = windowed();
    let b = flange(&w, &window_bottom(&w), serde_json::json!({"height": 20})).unwrap();
    let f = unfold_side(&b, false, K).unwrap();
    let reach = f.overlaps[0][1].x - 50.0;
    let over = 3.0;
    let line = |x: f64, y: f64, sx: f64, sy: f64| ((x - sx * over, y - sy * over), (x + sx * (reach + over), y + sy * (reach + over)));
    let segments = [line(50.0, 100.0, 1.0, 1.0), line(150.0, 100.0, -1.0, 1.0), line(50.0, 200.0, 1.0, -1.0), line(150.0, 200.0, -1.0, -1.0)];
    let g = 1.0;
    let cut = line_cut_on_blank(&b, &segments, g).unwrap();
    assert!(cut.volume() < b.volume());
    let flat = unfold_side(&cut, false, K).unwrap();
    assert!(flat.overlaps.is_empty(), "{:?}", flat.overlaps);
    assert_eq!(flat.bends.len(), 8);
    // Each line in space, on the blank's frame: taken back to one line.
    let carried: Vec<datum_ops::LineCut> = {
        let [fo, fx, fy, fz] = f.frame.unwrap();
        let at = |p: (f64, f64)| fo + fx * p.0 + fy * p.1;
        segments.iter().map(|(p, q)| datum_ops::LineCut { ends: [at(*p), at(*q)], normal: fz, kerf: g, arc: None }).collect()
    };
    let closed = flat.with_lines_closed(&carried);
    assert!(closed.pieces.len() < flat.pieces.len(), "{} of {}", closed.pieces.len(), flat.pieces.len());
    assert_ne!(closed.hash, flat.hash);
    assert!(closed.dxf().contains("LINE"));
    // Every end of a piece meets another's but the lines' ends in the plate.
    let ends: Vec<datum_math::Vec2> =
        closed.pieces.iter().filter_map(|p| if let datum_ops::FlatPiece::Line(a, b) = p { Some([*a, *b]) } else { None }).flatten().collect();
    let lone = ends.iter().filter(|a| ends.iter().filter(|b| (a.x - b.x).abs() < 1e-6 && (a.y - b.y).abs() < 1e-6).count() < 2).count();
    // The plate's outline, the window's, and a line in each corner, its far end in the plate.
    assert_eq!(closed.pieces.len(), 12);
    assert_eq!(lone, 4);
    // No kerf: asked of the rule.
    let target = Value::Body(b.clone());
    let sketch = op(serde_json::json!({"op": "sketch", "flat": "front", "k_factor": K, "sketch": {"points": {"a": [55, 105], "b": [60, 110]}, "lines": {"l": {"from": "a", "to": "b"}}}}))
        .eval(id("lines"), &[&target])
        .unwrap();
    let err = op(serde_json::json!({"op": "extrude", "mode": "cut", "through": true, "wall": true})).eval(id("c"), &[&target, &sketch]).unwrap_err();
    assert!(err.to_string().contains("kerf"), "{err}");
}

/// `b` cut by the open lines of `sketch` drawn on its blank at [`K`], the cut's own fields `extra` (the
/// kerf among them).
fn line_cut_sketch(b: &Body, sketch: serde_json::Value, extra: serde_json::Value) -> Result<Body, BrepError> {
    let target = Value::Body(b.clone());
    let drawn = op(serde_json::json!({"op": "sketch", "flat": "front", "k_factor": K, "sketch": sketch})).eval(id("lines"), &[&target])?;
    let mut j = serde_json::json!({"op": "extrude", "mode": "cut", "through": true, "wall": true});
    for (k, v) in extra.as_object().unwrap() {
        j[k] = v.clone();
    }
    let r = op(j).eval(id("line.cut"), &[&target, &drawn])?.body().unwrap().clone();
    r.validate().unwrap();
    tessellate(&r).unwrap();
    Ok(r)
}

/// The lines of `segments` as a sketch of separate open lines.
fn lines_sketch(segments: &[Segment]) -> serde_json::Value {
    let (mut points, mut lines) = (serde_json::Map::new(), serde_json::Map::new());
    for (k, (p, q)) in segments.iter().enumerate() {
        points.insert(format!("a{k}"), serde_json::json!([p.0, p.1]));
        points.insert(format!("b{k}"), serde_json::json!([q.0, q.1]));
        lines.insert(format!("l{k}"), serde_json::json!({"from": format!("a{k}"), "to": format!("b{k}")}));
    }
    serde_json::json!({"points": points, "lines": lines})
}

/// The `segments` drawn on the blank of `before` (seen from the front at [`K`]) in space, each `kerf` wide.
fn in_space(before: &Body, segments: &[Segment], kerf: f64) -> Vec<datum_ops::LineCut> {
    let [fo, fx, fy, fz] = unfold_side(before, false, K).unwrap().frame.unwrap();
    let at = |q: (f64, f64)| fo + fx * q.0 + fy * q.1;
    segments.iter().map(|(p, q)| datum_ops::LineCut { ends: [at(*p), at(*q)], normal: fz, kerf, arc: None }).collect()
}

/// The ends of the lines of `f` that no other end meets.
fn lone_ends(f: &Flat) -> Vec<datum_math::Vec2> {
    let ends: Vec<datum_math::Vec2> = lines_of(f).iter().flat_map(|(a, b)| [*a, *b]).collect();
    let met = |a: &datum_math::Vec2| ends.iter().filter(|b| (a.x - b.x).abs() < 1e-6 && (a.y - b.y).abs() < 1e-6).count();
    ends.iter().filter(|a| met(a) < 2).copied().collect()
}

/// The lines of the blank `f`.
fn lines_of(f: &Flat) -> Vec<(datum_math::Vec2, datum_math::Vec2)> {
    f.pieces.iter().filter_map(|p| if let FlatPiece::Line(a, b) = p { Some((*a, *b)) } else { None }).collect()
}

/// A line inside a plate's metal is a slot of length x kerf x thickness; on the blank one line and no
/// rectangle. A line from the edge in is a notch, the outline closed but for the line.
#[test]
fn a_line_inside_the_metal_is_a_slot_and_one_line_on_the_blank() {
    let p = plate();
    let whole = unfold_side(&p, false, K).unwrap();
    assert_eq!(whole.counts(), [4, 0, 0]);
    // Inside, ends in the metal.
    let seg = ((30.0, 30.0), (70.0, 30.0));
    let slot = line_cut_sketch(&p, lines_sketch(&[seg]), serde_json::json!({"kerf": 1.5})).unwrap();
    let want = p.volume() - 40.0 * 1.5 * T;
    assert!(close(slot.volume(), want), "{} vs {want}", slot.volume());
    let raw = unfold_side(&slot, false, K).unwrap();
    assert_eq!(raw.counts()[0], 8, "the rectangle: {:?}", raw.pieces);
    let closed = raw.with_lines_closed(&in_space(&p, &[seg], 1.5));
    assert_eq!(closed.counts(), [5, 0, 0], "{:?}", closed.pieces);
    // From beyond the edge in: a notch.
    let seg = ((-5.0, 30.0), (20.0, 30.0));
    let notch = line_cut_sketch(&p, lines_sketch(&[seg]), serde_json::json!({"kerf": 1.5})).unwrap();
    let want = p.volume() - 20.0 * 1.5 * T;
    assert!(close(notch.volume(), want), "{} vs {want}", notch.volume());
    let raw = unfold_side(&notch, false, K).unwrap();
    let closed = raw.with_lines_closed(&in_space(&p, &[seg], 1.5));
    // Bottom, right, top, the left edge in two and the line.
    assert_eq!(closed.counts(), [6, 0, 0], "{:?}", closed.pieces);
    let lone = lone_ends(&closed);
    assert_eq!(lone.len(), 1, "the outline is closed but for the line's far end: {lone:?}");
}

/// A line across a U's bend, from one wall to the next, over either of its two bends: the cut goes over
/// the bend, the body is valid, and the blank has the line alone, its outline closed.
#[test]
fn a_line_across_a_bend_goes_over_it() {
    let u = u_of(MID);
    let f = unfold_side(&u, false, K).unwrap();
    assert_eq!(f.bends.len(), 4);
    for bend in [3, 0] {
        let x = f.bends[bend][0].x;
        let y = 0.5 * (f.bends[bend][0].y + f.bends[bend][1].y);
        let seg = ((x - 12.0, y), (x + 12.0, y));
        let cut = line_cut_sketch(&u, lines_sketch(&[seg]), serde_json::json!({"kerf": 1.0})).unwrap();
        let gone = u.volume() - cut.volume();
        // The bend's layers are longer than the blank's neutral one: more than the straight cut takes.
        assert!(gone > 24.0 * T * 0.9 && gone < 24.0 * T * 1.5, "bend {bend}: {gone}");
        let raw = unfold_side(&cut, false, K).unwrap();
        assert!(raw.overlaps.is_empty() && raw.bends.len() == 4);
        let closed = raw.with_lines_closed(&in_space(&u, &[seg], 1.0));
        assert_eq!(closed.counts()[0], f.counts()[0] + 1, "bend {bend}: frames {:?} then {:?}: {:?}", f.frame, raw.frame, closed.pieces);
        assert_eq!(lone_ends(&closed).len(), 2, "bend {bend}: {:?}", lone_ends(&closed));
    }
}

/// A chain over a bend, its arc on the bend line: the gap goes over the bend, on the blank one pass,
/// the arc whole though the cut's walls are in pieces on either side of the line.
#[test]
fn a_chain_goes_over_a_bend() {
    let u = u_of(MID);
    let f = unfold_side(&u, false, K).unwrap();
    let [o, x, y, z] = f.frame.unwrap();
    let (bx, my) = (f.bends[3][0].x, 0.5 * (f.bends[3][0].y + f.bends[3][1].y));
    let (a, b, c, d) = ((bx - 10.0, my - 6.0), (bx - 10.0, my), (bx - 10.0 + 20.0, my), (bx - 10.0 + 20.0, my - 6.0));
    let tab = serde_json::json!({"points": {"p1": [a.0, a.1], "p2": [b.0, b.1], "p3": [c.0, c.1], "p4": [bx, my], "p5": [d.0, d.1]},
        "lines": {"l1": {"from": "p1", "to": "p2"}, "l2": {"from": "p3", "to": "p5"}},
        "arcs": {"a1": {"center": "p4", "from": "p3", "to": "p2"}}});
    let cut = line_cut_sketch(&u, tab, serde_json::json!({"kerf": 0.5})).unwrap();
    assert!(cut.volume() < u.volume());
    let at = |q: (f64, f64)| o + x * q.0 + y * q.1;
    let piece = |p: (f64, f64), q: (f64, f64), arc| datum_ops::LineCut { ends: [at(p), at(q)], normal: z, kerf: 0.5, arc };
    let lines = [piece(a, b, None), piece(b, c, Some((at((bx, my)), false))), piece(c, d, None)];
    let closed = unfold_side(&cut, false, K).unwrap().with_lines_closed(&lines);
    let [plain, curved, _] = f.counts();
    assert_eq!(closed.counts()[0], plain + 2, "{:?}", closed.pieces);
    assert_eq!(closed.counts()[1], curved + 1, "{:?}", closed.pieces);
}

/// Closed contours and lines in one sketch: both are cut; on the blank the circle stays a circle and
/// the line a line.
#[test]
fn contours_and_lines_in_one_sketch_are_both_cut() {
    let p = plate();
    let mut sketch = lines_sketch(&[((50.0, 15.0), (80.0, 15.0))]);
    sketch["points"]["c"] = serde_json::json!([25, 30]);
    sketch["circles"] = serde_json::json!({"hole": {"center": "c", "diameter": 10}});
    let cut = line_cut_sketch(&p, sketch, serde_json::json!({"kerf": 1.0})).unwrap();
    let want = p.volume() - (25.0 * std::f64::consts::PI + 30.0 * 1.0) * T;
    assert!(close(cut.volume(), want), "{} vs {want}", cut.volume());
    let closed = unfold_side(&cut, false, K).unwrap().with_lines_closed(&in_space(&p, &[((50.0, 15.0), (80.0, 15.0))], 1.0));
    assert_eq!(closed.counts(), [5, 0, 1], "{:?}", closed.pieces);
}

/// A line cut on the blank only leaves the body as it was; `blank_cut` has the cut one.
#[test]
fn a_line_cut_on_the_blank_only_leaves_the_part() {
    let p = plate();
    let seg = ((30.0, 30.0), (70.0, 30.0));
    let only = serde_json::json!({"kerf": 1.0, "blank_only": true});
    let same = line_cut_sketch(&p, lines_sketch(&[seg]), only.clone()).unwrap();
    assert_eq!(same.volume(), p.volume());
    assert_eq!(same.faces().len(), p.faces().len());
    let target = Value::Body(p.clone());
    let drawn = op(serde_json::json!({"op": "sketch", "flat": "front", "k_factor": K, "sketch": lines_sketch(&[seg])}))
        .eval(id("lines"), &[&target])
        .unwrap();
    let cut_op = op(serde_json::json!({"op": "extrude", "mode": "cut", "through": true, "wall": true, "kerf": 1.0, "blank_only": true}));
    let cut = datum_ops::blank_cut(id("line.cut"), &p, &drawn, &cut_op).unwrap();
    assert!(close(cut.volume(), p.volume() - 40.0 * T), "{} vs {}", cut.volume(), p.volume() - 40.0 * T);
    // No kerf: asked of the rule here too.
    let no_kerf = op(serde_json::json!({"op": "extrude", "mode": "cut", "through": true, "wall": true, "blank_only": true}));
    let err = datum_ops::blank_cut(id("line.cut"), &p, &drawn, &no_kerf).unwrap_err();
    assert!(err.to_string().contains("kerf"), "{err}");
}

/// A kerf of nothing, one too wide for a curve of the chain, and an arc ending on an edge are refused,
/// not cut wrong.
#[test]
fn a_line_cut_that_cannot_be_made_is_refused() {
    let p = plate();
    let kerf = serde_json::json!({"kerf": 1.0});
    // An arc whose end lies on the plate's edge: its way past the edge is not made yet.
    let arc = serde_json::json!({"points": {"a": [30, 0], "b": [50, 0], "o": [40, 0]}, "arcs": {"r": {"center": "o", "from": "b", "to": "a"}}});
    let r = line_cut_sketch(&p, arc, kerf.clone());
    assert!(matches!(r, Err(BrepError::Unsupported(_))), "{r:?}");
    // Wider than an arc allows, or than a line is long.
    let r = line_cut_sketch(&p, u_tab((40.0, 20.0), 1.0, 8.0), serde_json::json!({"kerf": 3.0}));
    assert!(matches!(r, Err(BrepError::Degenerate(_))), "{r:?}");
    let short = serde_json::json!({"points": {"a": [20, 20], "b": [50, 20], "c": [50, 20.5], "d": [20, 20.5]},
        "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}, "cd": {"from": "c", "to": "d"}}});
    let r = line_cut_sketch(&p, short, serde_json::json!({"kerf": 2.0}));
    assert!(matches!(r, Err(BrepError::Degenerate(_))), "{r:?}");
    // A kerf of nothing, or less.
    for k in [0.0, -1.0] {
        let r = line_cut_sketch(&p, lines_sketch(&[((30.0, 30.0), (70.0, 30.0))]), serde_json::json!({"kerf": k}));
        assert!(matches!(r, Err(BrepError::Degenerate(_))), "{k}: {r:?}");
    }
}

/// A wire's tab (D237): up a line from `at`, round an arc of `radius`, down a line as long: a chain
/// tangent throughout, open at the bottom.
fn u_tab(at: (f64, f64), radius: f64, height: f64) -> serde_json::Value {
    let (x, y) = at;
    let (right, top) = (x + 2.0 * radius, y + height);
    serde_json::json!({"points": {"p1": [x, y], "p2": [x, top], "p3": [right, top], "p4": [x + radius, top], "p5": [right, y]},
        "lines": {"l1": {"from": "p1", "to": "p2"}, "l2": {"from": "p3", "to": "p5"}},
        "arcs": {"a1": {"center": "p4", "from": "p3", "to": "p2"}}})
}

/// The pieces the cut added: those of `cut`'s blank not in `whole`'s.
fn new_pieces<'a>(cut: &'a Flat, whole: &Flat) -> Vec<&'a FlatPiece> {
    cut.pieces.iter().filter(|p| !whole.pieces.contains(p)).collect()
}

/// One pass of the laser round a wire's tab: in the part a gap the kerf wide, on the blank one line,
/// one arc, one line, ending where the sketch's curves do, with no sides or ends of the gap.
#[test]
fn a_chain_with_an_arc_is_one_cut_and_one_line_in_the_blank() {
    let (p, kerf, radius, height) = (plate(), 0.5, 5.0, 12.0);
    let whole = unfold_side(&p, false, K).unwrap();
    let cut = line_cut_sketch(&p, u_tab((40.0, 15.0), radius, height), serde_json::json!({"kerf": kerf})).unwrap();
    let t = p.bounds().1.z - p.bounds().0.z;
    let length = 2.0 * height + std::f64::consts::PI * radius;
    let squares = kerf * length * t;
    assert!((p.volume() - cut.volume() - squares).abs() < 1e-9 * p.volume().max(1.0) + 1e-6, "{} vs {}", p.volume() - cut.volume(), squares);
    let a = |x: f64, y: f64| datum_math::Vec2::new(x, y);
    let lines = [
        datum_ops::LineCut { ends: [(40.0, 15.0), (40.0, 27.0)].map(|q| at_blank(&p, q)), normal: normal_of(&p), kerf, arc: None },
        datum_ops::LineCut {
            ends: [(40.0, 27.0), (50.0, 27.0)].map(|q| at_blank(&p, q)),
            normal: normal_of(&p),
            kerf,
            arc: Some((at_blank(&p, (45.0, 27.0)), false)),
        },
        datum_ops::LineCut { ends: [(50.0, 27.0), (50.0, 15.0)].map(|q| at_blank(&p, q)), normal: normal_of(&p), kerf, arc: None },
    ];
    let closed = unfold_side(&cut, false, K).unwrap().with_lines_closed(&lines);
    let more = new_pieces(&closed, &whole);
    assert_eq!(more.len(), 3, "{more:?}");
    let close = |p: datum_math::Vec2, q: datum_math::Vec2| (p.x - q.x).abs() < 1e-9 && (p.y - q.y).abs() < 1e-9;
    let straight: Vec<_> = more.iter().filter_map(|p| if let FlatPiece::Line(a, b) = p { Some((*a, *b)) } else { None }).collect();
    assert_eq!(straight.len(), 2);
    assert!(close(straight[0].0, a(40.0, 15.0)) && close(straight[0].1, a(40.0, 27.0)), "{straight:?}");
    assert!(close(straight[1].0, a(50.0, 27.0)) && close(straight[1].1, a(50.0, 15.0)), "{straight:?}");
    let arcs: Vec<_> = more
        .iter()
        .filter_map(|p| if let FlatPiece::Arc { center, radius, start, end } = p { Some((*center, *radius, *start, *end)) } else { None })
        .collect();
    assert_eq!(arcs.len(), 1, "{more:?}");
    assert!(close(arcs[0].0, a(45.0, 27.0)) && (arcs[0].1 - radius).abs() < 1e-9, "{arcs:?}");
    // Over the top: from the right round to the left.
    assert!(arcs[0].2.abs() < 1e-9 && (arcs[0].3 - std::f64::consts::PI).abs() < 1e-9, "{arcs:?}");
}

/// A point of plate `p`'s blank, seen from the front, in space.
fn at_blank(p: &Body, q: (f64, f64)) -> datum_math::Vec3 {
    let [o, x, y, _] = unfold_side(p, false, K).unwrap().frame.unwrap();
    o + x * q.0 + y * q.1
}

fn normal_of(p: &Body) -> datum_math::Vec3 {
    unfold_side(p, false, K).unwrap().frame.unwrap()[3]
}

/// Corners, not only tangents: two lines at a right angle, a line into an arc at an angle, the gap
/// the kerf wide all along, whatever way the chain is drawn.
#[test]
fn a_chain_turns_a_corner() {
    let (p, kerf) = (plate(), 1.0);
    let t = p.bounds().1.z - p.bounds().0.z;
    let elbow = |from: &str, to: &str| {
        serde_json::json!({"points": {"a": [20, 20], "b": [50, 20], "c": [50, 40]},
            "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": from, "to": to}}})
    };
    for sketch in [elbow("b", "c"), elbow("c", "b")] {
        let cut = line_cut_sketch(&p, sketch, serde_json::json!({"kerf": kerf})).unwrap();
        // Each line's strip, ending square at the corner's middle: together the area as if unfolded.
        let area = kerf * (30.0 + 20.0);
        assert!((p.volume() - cut.volume() - area * t).abs() < 1e-6, "{}", p.volume() - cut.volume());
    }
    // A line meeting an arc's end square to its tangent, on either side of it.
    let kinked = serde_json::json!({"points": {"a": [20, 30], "b": [40, 30], "c": [60, 30], "o": [50, 30]},
        "lines": {"ab": {"from": "a", "to": "b"}}, "arcs": {"r": {"center": "o", "from": "c", "to": "b"}}});
    let cut = line_cut_sketch(&p, kinked, serde_json::json!({"kerf": 0.5})).unwrap();
    assert!(cut.volume() < p.volume());
}

/// A chain drawn from a point snapped to the plate's edge: its line goes on past the edge, and the
/// other end, in the metal, leaves the tab joined to the plate.
#[test]
fn a_chain_snapped_to_the_edge_opens_it() {
    let p = plate();
    let one_end = serde_json::json!({"points": {"p1": [40, 0], "p2": [40, 12], "p3": [50, 12], "p4": [45, 12], "p5": [50, 5]},
        "lines": {"l1": {"from": "p1", "to": "p2"}, "l2": {"from": "p3", "to": "p5"}},
        "arcs": {"a1": {"center": "p4", "from": "p3", "to": "p2"}}});
    let cut = line_cut_sketch(&p, one_end, serde_json::json!({"kerf": 0.5})).unwrap();
    assert!(cut.volume() < p.volume());
}

/// A window of walls of two heights cut by lines off 45 degrees: the blank lies flat, and on the closed
/// blank the free edges end on each line.
#[test]
fn lines_off_45_degrees_settle_walls_of_two_heights() {
    let w = windowed_by(140.0, 90.0);
    let first = flange(&w, &window_edges(&w, true), serde_json::json!({"height": 20})).unwrap();
    let b = flange(&first, &window_edges(&first, false), serde_json::json!({"height": 30})).unwrap();
    let f = unfold_side(&b, false, K).unwrap();
    let (rx, ry) = (longer(20.0, 90.0), longer(30.0, 90.0));
    let window = window_box(&f.bends);
    let tilt = 1.25;
    let through = (rx, ry * tilt);
    let over = 3.0;
    let segments: Vec<_> = window_corners(window)
        .iter()
        .map(|&((x, y), (sx, sy))| {
            let l = (through.0 * through.0 + through.1 * through.1).sqrt();
            let (ux, uy) = (through.0 / l, through.1 / l);
            ((x - sx * ux * over, y - sy * uy * over), (x + sx * (through.0 + ux * over), y + sy * (through.1 + uy * over)))
        })
        .collect();
    let cut = line_cut_sketch(&b, lines_sketch(&segments), serde_json::json!({"kerf": 1.0})).unwrap();
    let flat = lies_flat(&cut, K, 8);
    let closed = flat.with_lines_closed(&in_space(&b, &segments, 1.0));
    // Where the line crosses the edge `rx` out along X, and the edge `ry` out along Y.
    for &((x, y), (sx, sy)) in &window_corners(window) {
        let on_x_edge = (x + sx * rx, y + sy * through.1);
        let on_y_edge = (x + sx * ry * through.0 / through.1, y + sy * ry);
        assert!(has_vertex(&closed, &f, on_x_edge), "{x} {y} {on_x_edge:?}");
        assert!(has_vertex(&closed, &f, on_y_edge), "{x} {y} {on_y_edge:?}");
    }
    assert!(closed.pieces.len() < flat.pieces.len());
}

/// The blank of the cut window lies another way round than the blank the lines were drawn on (same plane,
/// turned): the lines, given in space, are still found, and the blank closed has the same 12 pieces.
#[test]
fn lines_are_closed_on_a_blank_laid_another_way_round() {
    let w = windowed();
    let b = flange(&w, &window_bottom(&w), serde_json::json!({"height": 20})).unwrap();
    let f = unfold_side(&b, false, K).unwrap();
    let reach = f.overlaps[0][1].x - 50.0;
    let over = 3.0;
    let line = |x: f64, y: f64, sx: f64, sy: f64| ((x - sx * over, y - sy * over), (x + sx * (reach + over), y + sy * (reach + over)));
    let segments = [line(50.0, 100.0, 1.0, 1.0), line(150.0, 100.0, -1.0, 1.0), line(50.0, 200.0, 1.0, -1.0), line(150.0, 200.0, -1.0, -1.0)];
    let cut = line_cut_sketch(&b, lines_sketch(&segments), serde_json::json!({"kerf": 1.0})).unwrap();
    let flat = unfold_side(&cut, false, K).unwrap();
    let (before, after) = (f.frame.unwrap(), flat.frame.unwrap());
    assert!(before[1].dot(after[1]) < 0.5, "the same way round: {before:?} {after:?}");
    assert!(before[3].dot(after[3]) > 1.0 - 1e-9 && (before[0] - after[0]).dot(before[3]).abs() < 1e-9, "not one plane");
    let closed = flat.with_lines_closed(&in_space(&b, &segments, 1.0));
    assert_eq!(closed.pieces.len(), 12, "{} of {}", closed.pieces.len(), flat.pieces.len());
    assert_eq!(lone_ends(&closed).len(), 4);
}

/// Corners settled by cuts one after another, not in one: each cut goes on the blank as the one
/// before it left it, the last leaves it flat.
#[test]
fn a_windows_corners_are_settled_one_cut_after_another() {
    let w = windowed();
    let b = flange(&w, &window_bottom(&w), serde_json::json!({"height": 20})).unwrap();
    let mut body = b.clone();
    for (x, y, sx, sy) in [(50.0, 100.0, 1.0, 1.0), (150.0, 100.0, -1.0, 1.0), (50.0, 200.0, 1.0, -1.0), (150.0, 200.0, -1.0, -1.0)] {
        // In space, for the blank as it now lies.
        let [fo, fx, fy, _] = unfold_side(&b, false, K).unwrap().frame.unwrap();
        let reach = unfold_side(&b, false, K).unwrap().overlaps[0][1].x - 50.0;
        let [o, ex, ey, _] = unfold_side(&body, false, K).unwrap().frame.unwrap();
        let now = |p: (f64, f64)| {
            let q = fo + fx * p.0 + fy * p.1;
            ((q - o).dot(ex), (q - o).dot(ey))
        };
        let seg = (now((x - sx * 3.0, y - sy * 3.0)), now((x + sx * (reach + 3.0), y + sy * (reach + 3.0))));
        body = line_cut_on_blank(&body, &[seg], 1.0).unwrap();
    }
    let flat = unfold_side(&body, false, K).unwrap();
    assert!(flat.overlaps.is_empty(), "{:?}", flat.overlaps);
    assert_eq!(flat.bends.len(), 8);
}

/// A line drawn from the corner of the bend lines, snapped to it, to the very point the free edges
/// meet at (or past it): the cut goes through as if drawn past them; the blank has the line from
/// point to point, no further.
#[test]
fn a_line_from_a_corner_to_where_the_free_edges_meet() {
    let w = windowed();
    let b = flange(&w, &window_bottom(&w), serde_json::json!({"height": 20})).unwrap();
    let f = unfold_side(&b, false, K).unwrap();
    let reach = f.overlaps[0][1].x - 50.0;
    for far in [reach, reach + 5.0] {
        let line = |x: f64, y: f64, sx: f64, sy: f64| ((x, y), (x + sx * far, y + sy * far));
        let segments = [line(50.0, 100.0, 1.0, 1.0), line(150.0, 100.0, -1.0, 1.0), line(50.0, 200.0, 1.0, -1.0), line(150.0, 200.0, -1.0, -1.0)];
        let cut = line_cut_on_blank(&b, &segments, 0.2).unwrap();
        let flat = unfold_side(&cut, false, K).unwrap();
        assert!(flat.overlaps.is_empty(), "{far}: {:?}", flat.overlaps);
        let closed = flat.with_lines_closed(&in_space(&b, &segments, 0.2));
        // The plate's outline, the window's and the corners' lines, ending where they meet: no end alone.
        assert_eq!(closed.pieces.len(), 12, "{far}: {:?}", closed.pieces);
        assert_eq!(lone_ends(&closed).len(), 4, "{far}: {:?}", lone_ends(&closed));
        for e in lone_ends(&closed) {
            let on_corner =
                [(50.0, 100.0), (150.0, 100.0), (50.0, 200.0), (150.0, 200.0)].iter().any(|c| (e.x - c.0).abs() < 1e-6 && (e.y - c.1).abs() < 1e-6);
            assert!(on_corner, "{far}: {e:?}");
        }
    }
}
