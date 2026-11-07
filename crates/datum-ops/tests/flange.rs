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
    // Leaning in over the plate, the walls would meet above its corner.
    let err = flange(&p, &corner, serde_json::json!({"angle": 120})).unwrap_err();
    assert!(matches!(err, BrepError::Unsupported(ref m) if m.contains("corner")), "{err}");
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
fn flanges_at_an_inner_corner_are_refused() {
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
    let err = flange(&p, &ends_edges(&p, &ends), serde_json::json!({})).unwrap_err();
    assert!(matches!(err, BrepError::Unsupported(ref m) if m.contains("inner corner")), "{err}");
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
