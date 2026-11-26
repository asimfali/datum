//! A flange reaching a flat face of another body, `to_face` (D305): the wall's outer surface
//! parallel to the face and exactly `gap` from it (`0`: touching), the angle and side from the
//! face, and the wall's own `height`.
#![allow(clippy::unwrap_used, clippy::panic)]

use datum_brep::{Body, BrepError, Surface, TopoId};
use datum_math::Vec3;
use datum_ops::{Op, Value, flange_edges, flange_face_near};

const T: f64 = 2.0;
const R: f64 = 2.0;

fn id(tag: &str) -> TopoId {
    TopoId::derive(tag).finish()
}

fn op(json: serde_json::Value) -> Op {
    serde_json::from_value(json).unwrap()
}

/// A sheet metal ribbon from the line `a`–`b`, thickness [`T`] on `side`, width `z1`.
fn sheet(name: &str, a: [f64; 2], b: [f64; 2], side: &str, start: &str, z1: f64) -> Body {
    let sketch = serde_json::json!({"points": {"p1": a, "p2": b}, "lines": {"l1": {"from": "p1", "to": "p2"}}});
    let p = op(serde_json::json!({"op": "sketch", "sketch": sketch})).eval(id(&format!("{name}.sketch")), &[]).unwrap();
    op(serde_json::json!({"op": "sheet", "thickness": T, "radius": R, "side": side, "start": start, "path": "l1", "z0": 0.0, "z1": z1}))
        .eval(id(name), &[&p])
        .unwrap()
        .body()
        .unwrap()
        .clone()
}

/// The flange's sheet: horizontal, its end at `x = 0`, the flange on that end bends up (`+Y`).
fn flange_sheet() -> Body {
    sheet("a", [-98.45120368112408, 37.0], [0.0, 37.0], "right", "p2", 30.0)
}

/// Another sheet whose path plane goes through `q` at `degrees` to the horizontal, so its big
/// face there is the target.
fn target_sheet(q: [f64; 2], degrees: f64) -> Body {
    target_sheet_named("b", q, degrees)
}

/// [`target_sheet`] named `tag` for its IDs: two alike in place, apart in ID.
fn target_sheet_named(tag: &str, q: [f64; 2], degrees: f64) -> Body {
    let (s, c) = (libm::sin(degrees.to_radians()), libm::cos(degrees.to_radians()));
    let d = [c, s];
    sheet(tag, [q[0] - 30.0 * d[0], q[1] - 30.0 * d[1]], [q[0] + 30.0 * d[0], q[1] + 30.0 * d[1]], "left", "p2", 40.0)
}

/// A round post along `Z` at `center` (a cylinder's flat caps are along the flange's edge).
fn post(center: [f64; 2], diameter: f64, z1: f64) -> Body {
    let sketch = serde_json::json!({"points": {"h": center}, "circles": {"c": {"center": "h", "diameter": diameter}}});
    let p = op(serde_json::json!({"op": "sketch", "sketch": sketch})).eval(id("post.sketch"), &[]).unwrap();
    op(serde_json::json!({"op": "extrude", "z0": 0.0, "z1": z1})).eval(id("post"), &[&p]).unwrap().body().unwrap().clone()
}

/// The ID of the largest flat face of `b` whose plane contains `q`.
fn face_through(body: &Body, q: [f64; 2]) -> String {
    let p = Vec3::new(q[0], q[1], 0.0);
    let x = (0..body.faces().len())
        .filter(|&x| {
            body.faces()[x].surface.plane().is_some_and(|pl| {
                let n = pl.normal * (1.0 / pl.normal.dot(pl.normal).sqrt());
                close((p - pl.origin).dot(n), 0.0)
            })
        })
        .max_by(|&a, &b| body.face_area(a as u32).total_cmp(&body.face_area(b as u32)))
        .unwrap();
    format!("{:?}", body.faces()[x].id)
}

/// The long edge of the end of `b` at `x` (both endpoints there).
fn end_edge(b: &Body, x: f64) -> String {
    flange_edges(b).into_iter().find(|(_, e)| e.iter().all(|p| close(p.x, x))).map(|(e, _)| format!("{e:?}")).unwrap()
}

/// The flange on `a`'s end edge reaching `face` of `b`, with `extra` fields, and the target's plane.
fn reach_edge(a: &Body, b: &Body, edge: &str, face: &str, gap: f64, extra: serde_json::Value) -> Result<(Body, Vec3, Vec3), BrepError> {
    let mut j = serde_json::json!({"op": "flange", "edges": [edge], "height": 20, "radius": R,
        "to_face": {"face": face, "from": "b", "gap": gap}});
    for (k, v) in extra.as_object().unwrap() {
        j[k] = v.clone();
    }
    let out = op(j).eval_named(id("flange"), &[&Value::Body(a.clone()), &Value::Body(b.clone())], &["a", "b"])?.body().unwrap().clone();
    out.validate().unwrap();
    let target = b.faces().iter().find(|f| format!("{:?}", f.id) == face).unwrap().surface.plane().copied().unwrap();
    let n = target.normal * (1.0 / target.normal.dot(target.normal).sqrt());
    Ok((out, target.origin, n))
}

/// The flange on `a`'s end at `x = 0` reaching `face` of `b`.
fn reach(a: &Body, b: &Body, face: &str, gap: f64, extra: serde_json::Value) -> Result<(Body, Vec3, Vec3), BrepError> {
    reach_edge(a, b, &end_edge(a, 0.0), face, gap, extra)
}

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-9 * b.abs().max(1.0)
}

/// The new flat face of `after` (not in `before`) most nearly across `n`, the nearest to plane
/// point `at`.
fn new_face_across_index(before: &Body, after: &Body, at: Vec3, n: Vec3) -> usize {
    let old: std::collections::BTreeSet<TopoId> = before.faces().iter().map(|f| f.id).collect();
    (0..after.faces().len())
        .filter(|&x| !old.contains(&after.faces()[x].id))
        .filter(|&x| {
            after.faces()[x].surface.plane().is_some_and(|p| {
                let u = p.normal * (1.0 / p.normal.dot(p.normal).sqrt());
                close(u.cross(n).dot(u.cross(n)).sqrt(), 0.0)
            })
        })
        .min_by(|&x, &y| {
            let d = |x: usize| after.faces()[x].surface.plane().map_or(f64::INFINITY, |p| ((p.origin - at).dot(n)).abs());
            d(x).total_cmp(&d(y))
        })
        .unwrap()
}

/// The new flat face of `after` (not in `before`) most nearly across `n`, the nearest to plane
/// point `at`, and its normal.
fn new_face_across(before: &Body, after: &Body, at: Vec3, n: Vec3) -> (Vec3, Vec3) {
    let x = new_face_across_index(before, after, at, n);
    let p = after.faces()[x].surface.plane().unwrap();
    (p.origin, p.normal * (1.0 / p.normal.dot(p.normal).sqrt()))
}

fn distance(origin: Vec3, normal: Vec3, plane: Vec3) -> f64 {
    (origin - plane).dot(normal)
}

#[test]
fn a_wall_reaches_a_face_across_it() {
    let a = flange_sheet();
    let b = target_sheet([0.0, 37.0], 90.0);
    let face = face_through(&b, [0.0, 37.0]);
    let (out, at, n) = reach(&a, &b, &face, 0.0, serde_json::json!({"bend_position": "flush"})).unwrap();
    let (o, w) = new_face_across(&a, &out, at, n);
    // The wall's outer surface is parallel to the face (its normal across it).
    assert!(close(w.dot(n).abs(), 1.0), "wall normal {w:?} against {n:?}");
    // `flush`, at no gap: the wall's outer surface lies on the target's plane.
    assert!(close(distance(o, n, at), 0.0), "{}", distance(o, n, at));
    // The gap is absolute: 2 moves the outer surface exactly 2 from the face.
    let (out2, _, _) = reach(&a, &b, &face, 2.0, serde_json::json!({"bend_position": "flush"})).unwrap();
    let (o2, _) = new_face_across(&a, &out2, at, n);
    assert!(close(distance(o2, n, at), 2.0), "{}", distance(o2, n, at));
}

#[test]
fn a_wall_turns_to_sixty_degrees() {
    let a = flange_sheet();
    let b = target_sheet([0.0, 37.0], 60.0);
    let face = face_through(&b, [0.0, 37.0]);
    let (out, at, n) = reach(&a, &b, &face, 0.0, serde_json::json!({"bend_position": "flush"})).unwrap();
    let (o, w) = new_face_across(&a, &out, at, n);
    assert!(close(w.dot(n).abs(), 1.0), "the wall is parallel to the face");
    assert!(close(distance(o, n, at), 0.0));
    // The angle is the face's: the wall's outer surface turned 60° from the sheet's own end.
    let (s, c) = (libm::sin(60.0f64.to_radians()), libm::cos(60.0f64.to_radians()));
    assert!(close(w.x.abs(), s) && close(w.y.abs(), c), "wall normal {w:?} at 60°");
}

/// The pair of the author's `test10`: `s1` at 45°, `s2` horizontal with its end at the flange's
/// edge, and the target face `s1`'s big face through the origin.
fn author_pair() -> (Body, Body, String, String) {
    let s1 = sheet("s1", [0.0, 0.0], [70.71067811865476, 70.71067811865476], "left", "p2", 50.0);
    let s2 = sheet("s2", [-62.64616890505148, 36.99707274014928], [35.8050347760726, 36.99707274014928], "right", "p2", 30.0);
    let edge = end_edge(&s2, 35.8050347760726);
    let face = face_through(&s1, [0.0, 0.0]);
    (s1, s2, edge, face)
}

#[test]
fn the_authors_pair_is_reproduced_at_forty_five_degrees() {
    let (s1, s2, edge, face) = author_pair();
    let target = s1.faces().iter().find(|f| format!("{:?}", f.id) == face).unwrap().surface.plane().copied().unwrap();
    let n = target.normal * (1.0 / target.normal.dot(target.normal).sqrt());
    // The author's own flange: 45°, flush, 20 high.
    let manual = op(serde_json::json!({"op": "flange", "edges": [edge.clone()], "height": 20.0, "angle": 45.0, "radius": R,
        "bend_position": "flush"}))
    .eval(id("manual"), &[&Value::Body(s2.clone())])
    .unwrap()
    .body()
    .unwrap()
    .clone();
    let (mo, mw) = new_face_across(&s2, &manual, target.origin, n);
    // The wall's own clearance from the face, which the author's flange has.
    let d0 = distance(mo, n, target.origin);
    // The same wall reached at that gap: the same outer plane and the same angle.
    let (tf, at, nn) = reach_edge(&s2, &s1, &edge, &face, d0, serde_json::json!({"bend_position": "flush"})).unwrap();
    let (to, tw) = new_face_across(&s2, &tf, at, nn);
    assert!(close(tw.dot(nn).abs(), 1.0), "the wall is parallel to the face");
    // The angle is the face's 45°: the same the author's manual flange has.
    let (s, c) = (libm::sin(45.0f64.to_radians()), libm::cos(45.0f64.to_radians()));
    assert!(close(tw.x.abs(), s) && close(tw.y.abs(), c), "wall normal {tw:?} at 45°");
    assert!(close(distance(to, mw, mo), 0.0), "the same outer plane as the author's {}", distance(to, mw, mo));
}

#[test]
fn the_gap_is_absolute_with_a_natural_clearance() {
    let a = flange_sheet();
    let n = [-libm::sin(45.0f64.to_radians()), libm::cos(45.0f64.to_radians())];
    let q = [-0.5 * n[0], 37.0 - 0.5 * n[1]];
    let b = target_sheet(q, 45.0);
    let face = face_through(&b, q);
    let (out0, at0, nn0) = reach(&a, &b, &face, 0.0, serde_json::json!({"bend_position": "flush"})).unwrap();
    let (o0, _) = new_face_across(&a, &out0, at0, nn0);
    assert!(close(distance(o0, nn0, at0), 0.0), "no gap is touching: {}", distance(o0, nn0, at0));
    let (out1, at1, nn1) = reach(&a, &b, &face, 1.0, serde_json::json!({"bend_position": "flush"})).unwrap();
    let (o1, _) = new_face_across(&a, &out1, at1, nn1);
    assert!(close(distance(o1, nn1, at1), 1.0), "{}", distance(o1, nn1, at1));
}

/// The corner of the sheet's outer surface (its longest flat face across `m`) furthest along
/// `out`: the end a flange stands on.
fn outer_corner(a: &Body, m: Vec3, out: Vec3) -> Vec3 {
    let x = (0..a.faces().len())
        .filter(|&x| {
            a.faces()[x].surface.plane().is_some_and(|p| {
                let u = p.normal * (1.0 / p.normal.dot(p.normal).sqrt());
                close(u.dot(m), 1.0)
            })
        })
        .max_by(|&x, &y| a.face_area(x as u32).total_cmp(&a.face_area(y as u32)))
        .unwrap();
    a.face_loops(x as u32)
        .flat_map(|l| a.loop_half_edges(l))
        .map(|h| a.point(a.half_edges()[h as usize].origin))
        .max_by(|p, q| p.dot(out).total_cmp(&q.dot(out)))
        .unwrap()
}

/// How much the flat part of `a` must grow for a flange reaching the face `at`/`n` at `gap` (D305):
/// the distance along the end's normal `out` from the sheet's end to the line where the outer
/// surface across `m` meets the face's plane, `flush_recess` short of it.
fn extension_needed(a: &Body, m: Vec3, out: Vec3, at: Vec3, n: Vec3, gap: f64, degrees: f64) -> f64 {
    let end = outer_corner(a, m, out);
    (gap - (end - at).dot(n)) / out.dot(n) - (R + T) * libm::tan(0.5 * degrees.to_radians())
}

/// The sheet and the face `q` at 45° that it stops short of: the author's case of the acceptance,
/// the plane passing below the sheet's outer surface.
fn short_sheet() -> (Body, Body, String) {
    let a = flange_sheet();
    let n = [-libm::sin(45.0f64.to_radians()), libm::cos(45.0f64.to_radians())];
    let q = [-2.0 * n[0], 37.0 - 2.0 * n[1]];
    let b = target_sheet(q, 45.0);
    let face = face_through(&b, q);
    (a, b, face)
}

#[test]
fn a_sheet_short_of_the_face_is_lengthened_to_it() {
    let (a, b, face) = short_sheet();
    // `gap` 0 is touching, and the sheet is short of the face: the ordinary case now (D305).
    let (out, at, n) = reach(&a, &b, &face, 0.0, serde_json::json!({"bend_position": "flush"})).unwrap();
    let (o, w) = new_face_across(&a, &out, at, n);
    assert!(close(w.dot(n).abs(), 1.0), "the wall is parallel to the face");
    assert!(close(distance(o, n, at), 0.0), "the wall touches the face: {}", distance(o, n, at));
    // The flat part is lengthened: its outer surface (the sheet's lower face) reaches past the old
    // end to the line where it meets the face's plane, `flush_recess` short of it.
    let (outer, out_x) = (Vec3::new(0.0, -1.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
    let ext = extension_needed(&a, outer, out_x, at, n, 0.0, 45.0);
    assert!(ext > 0.0, "the sheet is short: {ext}");
    let reach_out = (outer_corner(&out, outer, out_x) - outer_corner(&a, outer, out_x)).dot(out_x);
    assert!(close(reach_out, ext), "the sheet reaches {reach_out} past its old end, not {ext}");
    // The strip is part of the sheet: against the same flange at the edge of the sheet as it was
    // (the bend outside it, nothing set back), the volume grows by `ext · width · thickness`.
    let manual = op(serde_json::json!({"op": "flange", "edges": [end_edge(&a, 0.0)], "height": 20.0, "angle": 45.0, "radius": R}))
        .eval(id("manual"), &[&Value::Body(a.clone())])
        .unwrap()
        .body()
        .unwrap()
        .clone();
    assert!(close(out.volume() - manual.volume(), ext * 30.0 * T), "{}", out.volume() - manual.volume());
}

/// The author's own shape: a sheet whose end is 10 short of a plate's face across it (`test9`).
#[test]
fn a_sheet_short_of_a_face_across_it_is_lengthened() {
    let a = flange_sheet();
    let b = target_sheet([10.0, 37.0], 90.0);
    let face = face_through(&b, [10.0, 37.0]);
    let (out, at, n) = reach(&a, &b, &face, 0.0, serde_json::json!({"bend_position": "flush"})).unwrap();
    let (o, w) = new_face_across(&a, &out, at, n);
    assert!(close(w.dot(n).abs(), 1.0), "the wall is parallel to the face");
    assert!(close(distance(o, n, at), 0.0), "the wall touches the face: {}", distance(o, n, at));
    // A right angle: the flat part grows by the 10 less the flush setback `r + t`.
    let outer = Vec3::new(0.0, -1.0, 0.0);
    let out_x = Vec3::new(1.0, 0.0, 0.0);
    let ext = extension_needed(&a, outer, out_x, at, n, 0.0, 90.0);
    assert!(close(ext, 10.0 - (R + T)), "{ext}");
    let reach_out = (outer_corner(&out, outer, out_x) - outer_corner(&a, outer, out_x)).dot(out_x);
    assert!(close(reach_out, ext), "the sheet reaches {reach_out} past its old end, not {ext}");
    let manual = op(serde_json::json!({"op": "flange", "edges": [end_edge(&a, 0.0)], "height": 20.0, "angle": 90.0, "radius": R}))
        .eval(id("manual"), &[&Value::Body(a.clone())])
        .unwrap()
        .body()
        .unwrap()
        .clone();
    assert!(close(out.volume() - manual.volume(), ext * 30.0 * T), "{}", out.volume() - manual.volume());
}

#[test]
fn a_lengthened_sheet_leaves_exactly_the_gap() {
    let (a, b, face) = short_sheet();
    for gap in [0.5, 2.0] {
        let (out, at, n) = reach(&a, &b, &face, gap, serde_json::json!({"bend_position": "flush"})).unwrap();
        let (o, _) = new_face_across(&a, &out, at, n);
        assert!(close(distance(o, n, at), gap), "the gap is exactly {gap}: {}", distance(o, n, at));
        let (outer, out_x) = (Vec3::new(0.0, -1.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
        let ext = extension_needed(&a, outer, out_x, at, n, gap, 45.0);
        let reach_out = (outer_corner(&out, outer, out_x) - outer_corner(&a, outer, out_x)).dot(out_x);
        assert!(close(reach_out, ext), "at {gap}: reaches {reach_out}, not {ext}");
    }
}

#[test]
fn a_lengthened_sheet_comes_out_longer_flat() {
    let (a, b, face) = short_sheet();
    let (out, at, n) = reach(&a, &b, &face, 0.0, serde_json::json!({"bend_position": "flush"})).unwrap();
    // The blank laid from the sheet's outer face, the one the wall's outer surface goes on from.
    let home = (0..a.faces().len())
        .filter(|&x| a.faces()[x].surface.plane().is_some_and(|p| p.normal.y < -0.99))
        .max_by(|&x, &y| a.face_area(x as u32).total_cmp(&a.face_area(y as u32)))
        .unwrap();
    let blank = |body: &Body| {
        let flat = op(serde_json::json!({"op": "unfold", "face": format!("{:?}", a.faces()[home].id), "k_factor": 0.44}))
            .eval(id("flat"), &[&Value::Body(body.clone())])
            .unwrap();
        flat.flat().unwrap().size
    };
    // The same flange at the edge of the sheet as it was (the bend outside it): the blank of the
    // lengthened sheet is longer by the strip sewn on, `ext`, and no other way.
    let manual = op(serde_json::json!({"op": "flange", "edges": [end_edge(&a, 0.0)], "height": 20.0, "angle": 45.0, "radius": R}))
        .eval(id("manual"), &[&Value::Body(a.clone())])
        .unwrap()
        .body()
        .unwrap()
        .clone();
    let ext = extension_needed(&a, Vec3::new(0.0, -1.0, 0.0), Vec3::new(1.0, 0.0, 0.0), at, n, 0.0, 45.0);
    let (grown, was) = (blank(&out), blank(&manual));
    assert!(close(grown.x - was.x, ext), "the blank grew {} not {ext}", grown.x - was.x);
    assert!(close(grown.y, was.y), "{grown:?} and {was:?}");
}

#[test]
fn an_unreachable_recess_is_refused() {
    let a = flange_sheet();
    let b = target_sheet([0.0, 37.0], 90.0);
    let face = face_through(&b, [0.0, 37.0]);
    let e = reach(&a, &b, &face, 1.0e6, serde_json::json!({"bend_position": "flush"})).unwrap_err();
    assert!(matches!(e, BrepError::Degenerate(_)), "{e:?}");
}

#[test]
fn angle_is_refused_with_a_face() {
    let a = flange_sheet();
    let b = target_sheet([0.0, 37.0], 90.0);
    let face = face_through(&b, [0.0, 37.0]);
    let e = reach(&a, &b, &face, 0.0, serde_json::json!({"angle": 60.0})).unwrap_err();
    assert!(matches!(e, BrepError::Sketch(_)), "{e:?}");
}

#[test]
fn height_along_is_refused() {
    let a = flange_sheet();
    let b = target_sheet([0.0, 37.0], 90.0);
    let face = face_through(&b, [0.0, 37.0]);
    let e = reach(&a, &b, &face, 0.0, serde_json::json!({"height_along": true})).unwrap_err();
    assert!(matches!(e, BrepError::Sketch(_)), "{e:?}");
}

#[test]
fn height_is_required_with_a_face() {
    let a = flange_sheet();
    let b = target_sheet([0.0, 37.0], 90.0);
    let face = face_through(&b, [0.0, 37.0]);
    let j = serde_json::json!({"op": "flange", "edges": [end_edge(&a, 0.0)], "radius": R,
        "bend_position": "flush", "to_face": {"face": face, "from": "b", "gap": 0.0}});
    let e = op(j).eval_named(id("flange"), &[&Value::Body(a.clone()), &Value::Body(b.clone())], &["a", "b"]).unwrap_err();
    assert!(matches!(e, BrepError::Sketch(_)), "{e:?}");
}

#[test]
fn neither_way_is_refused() {
    let a = flange_sheet();
    let j = serde_json::json!({"op": "flange", "edges": [end_edge(&a, 0.0)], "radius": R});
    let e = op(j).eval(id("flange"), &[&Value::Body(a.clone())]).unwrap_err();
    assert!(matches!(e, BrepError::Sketch(_)), "{e:?}");
}

#[test]
fn a_negative_gap_is_refused() {
    let a = flange_sheet();
    let b = target_sheet([0.0, 37.0], 90.0);
    let face = face_through(&b, [0.0, 37.0]);
    let e = reach(&a, &b, &face, -1.0, serde_json::json!({"bend_position": "flush"})).unwrap_err();
    assert!(matches!(e, BrepError::Degenerate(_)), "{e:?}");
}

#[test]
fn a_lost_face_is_refused() {
    let a = flange_sheet();
    let b = target_sheet([0.0, 37.0], 90.0);
    let e = reach(&a, &b, "0123456789abcdef0123456789abcdef", 0.0, serde_json::json!({"bend_position": "flush"})).unwrap_err();
    assert!(matches!(e, BrepError::LostReference(_)), "{e:?}");
}

#[test]
fn a_face_in_the_sheets_plane_is_refused() {
    let a = flange_sheet();
    // The sheet's own flat side, whose plane the wall cannot turn to.
    let own = (0..a.faces().len())
        .filter(|&x| a.faces()[x].surface.plane().is_some_and(|p| p.normal.y.abs() > 0.99))
        .max_by(|&x, &y| a.face_area(x as u32).total_cmp(&a.face_area(y as u32)))
        .unwrap();
    let j = serde_json::json!({"op": "flange", "edges": [end_edge(&a, 0.0)], "height": 20, "radius": R,
        "bend_position": "flush", "to_face": {"face": format!("{:?}", a.faces()[own].id), "from": "a", "gap": 0.0}});
    let e = op(j).eval_named(id("flange"), &[&Value::Body(a.clone()), &Value::Body(a.clone())], &["a", "a"]).unwrap_err();
    assert!(matches!(e, BrepError::Degenerate(_)), "{e:?}");
}

#[test]
fn a_curved_face_is_refused() {
    // A U of sheet metal has cylindrical bends.
    let path = serde_json::json!({"points": {"a": [0, 0], "b": [0, 60], "c": [20, 60], "d": [20, 0]},
        "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}, "cd": {"from": "c", "to": "d"}}});
    let p = op(serde_json::json!({"op": "sketch", "sketch": path})).eval(id("u.path"), &[]).unwrap();
    let u = op(serde_json::json!({"op": "sheet", "thickness": T, "radius": R, "side": "left", "z0": 0, "z1": 40}))
        .eval(id("u"), &[&p])
        .unwrap()
        .body()
        .unwrap()
        .clone();
    let cyl = (0..u.faces().len()).find(|&x| matches!(u.faces()[x].surface, Surface::Cylinder(_))).unwrap();
    let a = flange_sheet();
    let j = serde_json::json!({"op": "flange", "edges": [end_edge(&a, 0.0)], "height": 20, "radius": R,
        "to_face": {"face": format!("{:?}", u.faces()[cyl].id), "from": "b", "gap": 0.0}});
    let e = op(j).eval_named(id("flange"), &[&Value::Body(a.clone()), &Value::Body(u.clone())], &["a", "b"]).unwrap_err();
    assert!(matches!(e, BrepError::Unsupported(_)), "{e:?}");
}

#[test]
fn a_missing_body_is_refused() {
    let a = flange_sheet();
    let b = target_sheet([0.0, 37.0], 90.0);
    let face = face_through(&b, [0.0, 37.0]);
    let j = serde_json::json!({"op": "flange", "edges": [end_edge(&a, 0.0)], "height": 20, "radius": R,
        "bend_position": "flush", "to_face": {"face": face, "from": "gone", "gap": 0.0}});
    let vb = Value::Body(b.clone());
    let e = op(j).eval_named(id("flange"), &[&Value::Body(a.clone()), &vb], &["a", "b"]).unwrap_err();
    assert!(matches!(e, BrepError::Sketch(_)), "{e:?}");
}

#[test]
fn a_side_plane_bounds_the_wall_with_the_face() {
    let a = flange_sheet();
    let b = target_sheet([0.0, 37.0], 90.0);
    let face = face_through(&b, [0.0, 37.0]);
    // A work plane across the width (z = 10), the wall's start going to it.
    let cut = op(serde_json::json!({"op": "plane", "base": "xy", "offset": 10.0})).eval(id("cut"), &[]).unwrap();
    let j = serde_json::json!({"op": "flange", "edges": [end_edge(&a, 0.0)], "height": 20, "radius": R,
        "bend_position": "flush", "to_face": {"face": face, "from": "b", "gap": 0.0},
        "to": {"start": {"plane": "cut", "gap": 0.0}}});
    let out = op(j)
        .eval_named(id("flange"), &[&Value::Body(a.clone()), &Value::Body(b.clone()), &cut], &["a", "b", "cut"])
        .unwrap()
        .body()
        .unwrap()
        .clone();
    out.validate().unwrap();
    let target = b.faces().iter().find(|f| format!("{:?}", f.id) == face).unwrap().surface.plane().copied().unwrap();
    let n = target.normal * (1.0 / target.normal.dot(target.normal).sqrt());
    let (_, w) = new_face_across(&a, &out, target.origin, n);
    assert!(close(w.dot(n).abs(), 1.0), "the face still gives the angle");
    // The wall's side went to the plane `z = 10`: a new face lies there.
    let old: std::collections::BTreeSet<TopoId> = a.faces().iter().map(|f| f.id).collect();
    let start = (0..out.faces().len()).filter(|&x| !old.contains(&out.faces()[x].id)).find(|&x| {
        out.faces()[x].surface.plane().is_some_and(|p| {
            let u = p.normal * (1.0 / p.normal.dot(p.normal).sqrt());
            close(u.z.abs(), 1.0)
                && out
                    .face_loops(x as u32)
                    .flat_map(|l| out.loop_half_edges(l))
                    .all(|h| close(out.point(out.half_edges()[h as usize].origin).z, 10.0))
        })
    });
    let start = start.expect("the wall's side went to z = 10");
    assert!(out.face_area(start as u32) > 0.0, "the cut face at z = 10 has an area");
    assert!(out.faces().len() > a.faces().len());
}

#[test]
fn moving_the_target_rebuilds_the_wall() {
    let a = flange_sheet();
    let b1 = target_sheet([0.0, 37.0], 45.0);
    let face1 = face_through(&b1, [0.0, 37.0]);
    let (out1, _, n1) = reach(&a, &b1, &face1, 0.0, serde_json::json!({"bend_position": "flush"})).unwrap();
    let (_, w1) = new_face_across(&a, &out1, b1.faces().iter().find(|f| format!("{:?}", f.id) == face1).unwrap().surface.plane().unwrap().origin, n1);
    // The same target moved along its own normal: the face still sets the angle, and the wall
    // follows it to keep the gap.
    let d = [-libm::sin(45.0f64.to_radians()), libm::cos(45.0f64.to_radians())];
    let b2 = target_sheet([-d[0], 37.0 - d[1]], 45.0);
    let face2 = face_through(&b2, [-d[0], 37.0 - d[1]]);
    let (out2, _, n2) = reach(&a, &b2, &face2, 0.0, serde_json::json!({"bend_position": "flush"})).unwrap();
    let (_, w2) = new_face_across(&a, &out2, b2.faces().iter().find(|f| format!("{:?}", f.id) == face2).unwrap().surface.plane().unwrap().origin, n2);
    assert!(close(w1.dot(w2).abs(), 1.0), "the angle is the same, the wall follows the face");
    assert_ne!(out1.hash(), out2.hash());
}

#[test]
fn a_wall_reaches_a_face_without_flush() {
    let a = flange_sheet();
    let b = target_sheet([0.0, 37.0], 90.0);
    let face = face_through(&b, [0.0, 37.0]);
    // The bend stays outside the edge (`base_recess = 0`): the wall still sits exactly the gap
    // from the face.
    let (out, at, n) = reach(&a, &b, &face, 2.0, serde_json::json!({})).unwrap();
    let (o, w) = new_face_across(&a, &out, at, n);
    assert!(close(w.dot(n).abs(), 1.0), "the wall is parallel to the face");
    assert!(close(distance(o, n, at), 2.0), "the gap is exactly 2: {}", distance(o, n, at));
}

#[test]
fn a_top_plane_is_refused_with_a_face() {
    let a = flange_sheet();
    let b = target_sheet([0.0, 37.0], 90.0);
    let face = face_through(&b, [0.0, 37.0]);
    let cut = op(serde_json::json!({"op": "plane", "base": "xy", "offset": 10.0})).eval(id("cut"), &[]).unwrap();
    let j = serde_json::json!({"op": "flange", "edges": [end_edge(&a, 0.0)], "height": 20, "radius": R,
        "to_face": {"face": face, "from": "b", "gap": 0.0}, "to": {"top": {"plane": "cut", "gap": 0.0}}});
    let e = op(j).eval_named(id("flange"), &[&Value::Body(a.clone()), &Value::Body(b.clone()), &cut], &["a", "b", "cut"]).unwrap_err();
    assert!(matches!(e, BrepError::Sketch(_)), "{e:?}");
}

#[test]
fn a_closed_corner_is_refused_with_a_face() {
    let a = flange_sheet();
    let b = target_sheet([0.0, 37.0], 90.0);
    let face = face_through(&b, [0.0, 37.0]);
    let e = reach(&a, &b, &face, 0.0, serde_json::json!({"corner": "miter"})).unwrap_err();
    assert!(matches!(e, BrepError::Unsupported(_)), "{e:?}");
}

#[test]
fn a_width_is_refused_with_a_face() {
    let a = flange_sheet();
    let b = target_sheet([0.0, 37.0], 90.0);
    let face = face_through(&b, [0.0, 37.0]);
    let e = reach(&a, &b, &face, 0.0, serde_json::json!({"extent": {"kind": "center", "width": 5.0}})).unwrap_err();
    assert!(matches!(e, BrepError::Unsupported(_)), "{e:?}");
}

#[test]
fn a_walls_edge_is_refused_with_a_face() {
    let a = flange_sheet();
    // A wall on the sheet's end: its own edges on the sheet's side face, not an end's long edge.
    let bent = op(serde_json::json!({"op": "flange", "edges": [end_edge(&a, 0.0)], "height": 20, "radius": R}))
        .eval(id("bent"), &[&Value::Body(a.clone())])
        .unwrap()
        .body()
        .unwrap()
        .clone();
    let b = target_sheet([0.0, 37.0], 90.0);
    let face = face_through(&b, [0.0, 37.0]);
    // An end's long edges run along z; a wall's edge on a side face stays at one z.
    let wall = flange_edges(&bent).into_iter().find(|(_, e)| close(e[0].z, e[1].z)).map(|(e, _)| format!("{e:?}")).unwrap();
    let e = reach_edge(&bent, &b, &wall, &face, 0.0, serde_json::json!({})).unwrap_err();
    let BrepError::Unsupported(why) = e else { panic!("{e:?}") };
    assert!(why.contains("wall's edge"), "{why}");
}

#[test]
fn several_edges_are_refused_with_a_face() {
    let a = flange_sheet();
    let b = target_sheet([0.0, 37.0], 90.0);
    let face = face_through(&b, [0.0, 37.0]);
    let edges: Vec<String> = flange_edges(&a).into_iter().filter(|(_, e)| e.iter().all(|p| close(p.x, 0.0))).map(|(e, _)| format!("{e:?}")).collect();
    assert_eq!(edges.len(), 2, "an end's two long edges");
    let j = serde_json::json!({"op": "flange", "edges": edges, "height": 20, "radius": R,
        "to_face": {"face": face, "from": "b", "gap": 0.0}});
    let e = op(j).eval_named(id("flange"), &[&Value::Body(a.clone()), &Value::Body(b.clone())], &["a", "b"]).unwrap_err();
    let BrepError::Unsupported(why) = e else { panic!("{e:?}") };
    assert!(why.contains("one edge"), "{why}");
}

#[test]
fn an_unfold_has_the_derived_bend() {
    let a = flange_sheet();
    let b = target_sheet([0.0, 37.0], 60.0);
    let face = face_through(&b, [0.0, 37.0]);
    let (out, _, _) = reach(&a, &b, &face, 0.0, serde_json::json!({"bend_position": "flush"})).unwrap();
    let base = (0..out.faces().len())
        .filter(|&x| out.faces()[x].surface.plane().is_some_and(|p| p.normal.y.abs() > 0.99))
        .max_by(|&x, &y| out.face_area(x as u32).total_cmp(&out.face_area(y as u32)))
        .unwrap();
    let flat = op(serde_json::json!({"op": "unfold", "face": format!("{:?}", out.faces()[base].id), "k_factor": 0.44}))
        .eval(id("flat"), &[&Value::Body(out.clone())])
        .unwrap();
    assert!(flat.flat().is_some());
}

/// The end of `a` as the search takes it.
fn end_of(a: &Body) -> TopoId {
    TopoId::parse(&end_edge(a, 0.0)).unwrap()
}

#[test]
fn the_nearest_face_of_another_body_is_taken() {
    let a = flange_sheet();
    let near = target_sheet([10.0, 37.0], 90.0);
    let far = target_sheet_named("far", [40.0, 37.0], 90.0);
    // The bodies come in the view's order; the nearer face wins over it.
    let (name, face) = flange_face_near(&a, end_of(&a), &[("far", &far), ("near", &near)]).unwrap();
    assert_eq!(name, "near");
    // A flat face across the edge, not the target's end along it.
    let plane = near.faces().iter().find(|f| f.id == face).unwrap().surface.plane().copied().unwrap();
    assert!(plane.normal.x.abs() > 0.99, "{plane:?}");
    // The flange on that face is made: the nearest is a face the bend reaches.
    let (out, ..) = reach_edge(&a, &near, &end_edge(&a, 0.0), &format!("{face:?}"), 0.0, serde_json::json!({})).unwrap();
    out.validate().unwrap();
}

#[test]
fn a_face_the_bend_cannot_reach_is_skipped() {
    let a = flange_sheet();
    // Behind the sheet's far end: the bend would lie past it.
    let behind = target_sheet_named("behind", [-150.0, 37.0], 90.0);
    assert_eq!(flange_face_near(&a, end_of(&a), &[("behind", &behind)]), None);
    let ahead = target_sheet([10.0, 37.0], 90.0);
    assert_eq!(flange_face_near(&a, end_of(&a), &[("behind", &behind), ("ahead", &ahead)]).unwrap().0, "ahead");
}

#[test]
fn a_face_along_the_edge_is_no_face_to_reach() {
    let a = flange_sheet();
    // A round post's flat caps are along the edge, its side round: no face across the edge.
    let post = post([10.0, 37.0], 20.0, 40.0);
    assert_eq!(flange_face_near(&a, end_of(&a), &[("post", &post)]), None);
    let plate = target_sheet([10.0, 37.0], 90.0);
    assert_eq!(flange_face_near(&a, end_of(&a), &[("post", &post), ("plate", &plate)]).unwrap().0, "plate");
}

#[test]
fn an_equal_distance_keeps_the_lower_face_id() {
    let a = flange_sheet();
    let b1 = target_sheet_named("b1", [10.0, 37.0], 90.0);
    let b2 = target_sheet_named("b2", [10.0, 37.0], 90.0);
    let alone = |b: &Body, name: &str| flange_face_near(&a, end_of(&a), &[(name, b)]).unwrap().1;
    let (f1, f2) = (alone(&b1, "b1"), alone(&b2, "b2"));
    let first = flange_face_near(&a, end_of(&a), &[("b1", &b1), ("b2", &b2)]).unwrap();
    let second = flange_face_near(&a, end_of(&a), &[("b2", &b2), ("b1", &b1)]).unwrap();
    assert_eq!(first, second, "the order of the bodies does not decide");
    assert_eq!(first.1, f1.min(f2), "the same distance keeps the lower face ID");
}
