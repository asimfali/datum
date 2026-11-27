//! Bending along a line of a sketch on a flat face (D312), stage 1: the strips of the line over
//! the material of the face and the areas of the material on either side of it.
#![allow(clippy::unwrap_used, clippy::panic)]

use datum_brep::{Body, BrepError, TopoId};
use datum_math::{Vec2, Vec3};
use datum_ops::{Op, Placed, Value, bend_strips};

fn id(tag: &str) -> TopoId {
    TopoId::derive(tag).finish()
}

fn op(json: serde_json::Value) -> Op {
    serde_json::from_value(json).unwrap()
}

fn close(a: f64, b: f64) {
    assert!((a - b).abs() <= 1e-9 * b.abs().max(1.0), "{a} vs {b}");
}

fn at(p: Vec2, x: f64, y: f64) {
    close(p.x, x);
    close(p.y, y);
}

/// The top face of `b`, facing `+Z`, as a sketch names it.
fn top(b: &Body) -> String {
    let up = Vec3::new(0.0, 0.0, 1.0);
    let f = b.faces().iter().find(|f| f.surface.plane().is_some_and(|p| p.normal.cross(up) == Vec3::ZERO && p.normal.dot(up) > 0.0)).unwrap();
    format!("{:?}", f.id)
}

/// A plate `w` × `h`, 2 thick, its top face in the XY plane.
fn plate(w: f64, h: f64) -> Body {
    let polygon = serde_json::json!([[0.0, 0.0], [w, 0.0], [w, h], [0.0, h]]);
    op(serde_json::json!({"op": "prism", "polygon": polygon, "z0": 0.0, "z1": 2.0})).eval(id("plate"), &[]).unwrap().body().unwrap().clone()
}

/// `body` with the closed profile of `sketch` cut through its top face.
fn cut(body: &Body, sketch: serde_json::Value) -> Body {
    let body_value = Value::Body(body.clone());
    let s = op(serde_json::json!({"op": "sketch", "face": top(body), "sketch": sketch})).eval(id("cut.sketch"), &[&body_value]).unwrap();
    op(serde_json::json!({"op": "extrude", "mode": "cut", "through": true})).eval(id("cut"), &[&body_value, &s]).unwrap().body().unwrap().clone()
}

/// The rectangle `[x0, x1] × [y0, y1]` cut through `body`.
fn with_hole(body: &Body, [x0, y0, x1, y1]: [f64; 4]) -> Body {
    let sketch = serde_json::json!({
        "points": {"a": [x0, y0], "b": [x1, y0], "c": [x1, y1], "d": [x0, y1]},
        "lines": {"l1": {"from": "a", "to": "b"}, "l2": {"from": "b", "to": "c"}, "l3": {"from": "c", "to": "d"}, "l4": {"from": "d", "to": "a"}}
    });
    cut(body, sketch)
}

/// The circle of `diameter` at `center` cut through `body`.
fn with_round_hole(body: &Body, center: [f64; 2], diameter: f64) -> Body {
    let sketch = serde_json::json!({"points": {"h": center}, "circles": {"c": {"center": "h", "diameter": diameter}}});
    cut(body, sketch)
}

/// A sketch on the top face of `body` with the one line `l1` from `a` to `b`.
fn on_top(body: &Body, a: [f64; 2], b: [f64; 2]) -> Placed {
    let sketch = serde_json::json!({"points": {"p1": a, "p2": b}, "lines": {"l1": {"from": "p1", "to": "p2"}}});
    let s =
        op(serde_json::json!({"op": "sketch", "face": top(body), "sketch": sketch})).eval(id("bend.sketch"), &[&Value::Body(body.clone())]).unwrap();
    s.sketch().unwrap().clone()
}

/// The strips of the line from `a` to `b` on the top face of `body`.
fn strips(body: &Body, a: [f64; 2], b: [f64; 2]) -> datum_ops::BendStrips {
    bend_strips(body, &on_top(body, a, b), "l1").unwrap()
}

fn degenerate(body: &Body, a: [f64; 2], b: [f64; 2]) -> String {
    match bend_strips(body, &on_top(body, a, b), "l1") {
        Err(BrepError::Degenerate(m)) => m,
        other => panic!("{other:?}"),
    }
}

/// The plate 60 × 40 with the hole 10…40 × 10…30, as the author's case: a line across the cut
/// leaves two strips, one on each side of it.
fn holed() -> Body {
    with_hole(&plate(60.0, 40.0), [10.0, 10.0, 40.0, 30.0])
}

/// A skew line through the cut on and on: two strips, their ends on the edges of the face and of
/// the hole (the line 2, 36 → 50, 4).
#[test]
fn two_strips_through_the_cut() {
    let r = strips(&holed(), [2.0, 36.0], [50.0, 4.0]);
    assert_eq!(r.strips.len(), 2);
    at(r.strips[0].ends[0], 0.0, 112.0 / 3.0);
    at(r.strips[0].ends[1], 11.0, 30.0);
    at(r.strips[1].ends[0], 40.0, 32.0 / 3.0);
    at(r.strips[1].ends[1], 56.0, 0.0);
}

/// The same line cut short at the edge of the hole: it touches the strip beyond only at a point,
/// so one strip is bent — the author's "segment to the cut, one tab".
#[test]
fn a_segment_ending_on_the_cut_reaches_one_strip() {
    let r = strips(&holed(), [11.0, 30.0], [50.0, 4.0]);
    assert_eq!(r.strips.len(), 1);
    at(r.strips[0].ends[0], 40.0, 32.0 / 3.0);
    at(r.strips[0].ends[1], 56.0, 0.0);
}

/// A line wholly in the cut meets no material.
#[test]
fn a_line_in_the_cut_is_refused() {
    assert!(degenerate(&holed(), [20.0, 20.0], [30.0, 20.0]).starts_with("bend: "));
}

/// A line of the material short of both edges is a strip drawn out to them.
#[test]
fn a_short_line_is_drawn_out_to_the_edges() {
    let r = strips(&holed(), [5.0, 15.0], [5.0, 25.0]);
    assert_eq!(r.strips.len(), 1);
    at(r.strips[0].ends[0], 5.0, 0.0);
    at(r.strips[0].ends[1], 5.0, 40.0);
}

/// A line that reaches no material of the face, though its own line crosses it elsewhere.
#[test]
fn a_line_short_of_the_face_is_refused() {
    assert!(degenerate(&holed(), [62.0, 20.0], [65.0, 20.0]).starts_with("bend: "));
}

/// The areas on the sides of the line: the material left of it and right of it, the hole
/// subtracted from both.
#[test]
fn the_sides_are_the_material_on_each_side() {
    let r = strips(&holed(), [25.0, 5.0], [25.0, 35.0]);
    assert_eq!(r.strips.len(), 2);
    // Left of +Y is −X: 25 × 40 less the hole's 15 × 20; right is 35 × 40 less it.
    close(r.sides[0], 700.0);
    close(r.sides[1], 1100.0);
}

/// A line through the middle of the material: the sides are equal.
#[test]
fn a_middle_line_has_equal_sides() {
    let plate = with_hole(&plate(50.0, 40.0), [10.0, 10.0, 40.0, 30.0]);
    let r = strips(&plate, [25.0, 5.0], [25.0, 35.0]);
    assert_eq!(r.strips.len(), 2);
    close(r.sides[0], 700.0);
    close(r.sides[1], 700.0);
}

/// The strips come in the direction of the line, whichever way the line was sketched.
#[test]
fn the_strips_follow_the_line() {
    let body = holed();
    let a = [2.0, 36.0];
    let b = [50.0, 4.0];
    let forward = strips(&body, a, b);
    let backward = strips(&body, b, a);
    assert_eq!(forward.strips.len(), 2);
    assert_eq!(backward.strips.len(), 2);
    at(backward.strips[0].ends[0], forward.strips[1].ends[1].x, forward.strips[1].ends[1].y);
    at(backward.strips[1].ends[1], forward.strips[0].ends[0].x, forward.strips[0].ends[0].y);
    // Each strip runs on along the line, its first end before its second.
    let along = Vec2::new(b[0] - a[0], b[1] - a[1]);
    for s in &forward.strips {
        let d = Vec2::new(s.ends[1].x - s.ends[0].x, s.ends[1].y - s.ends[0].y);
        assert!(d.x * along.x + d.y * along.y > 0.0);
    }
    assert!(along.x > 0.0);
}

/// Through a round hole: the line meets its arcs, and the sides take the circular segments.
#[test]
fn a_line_through_a_round_hole() {
    let body = with_round_hole(&plate(60.0, 40.0), [30.0, 20.0], 20.0);
    let r = strips(&body, [30.0, 5.0], [30.0, 35.0]);
    assert_eq!(r.strips.len(), 2);
    at(r.strips[0].ends[0], 30.0, 0.0);
    at(r.strips[0].ends[1], 30.0, 10.0);
    at(r.strips[1].ends[0], 30.0, 30.0);
    at(r.strips[1].ends[1], 30.0, 40.0);
    // Half the plate less half the hole on each side.
    let half = 1200.0 - 50.0 * std::f64::consts::PI;
    close(r.sides[0], half);
    close(r.sides[1], half);
}

/// A line through a round hole across one of the arcs' own vertices: the boundary crosses the
/// line there, and the hole cuts the line in two as anywhere else.
#[test]
fn a_line_through_the_vertex_of_a_round_hole() {
    let body = with_round_hole(&plate(60.0, 40.0), [30.0, 20.0], 20.0);
    let r = strips(&body, [0.0, 20.0], [60.0, 20.0]);
    assert_eq!(r.strips.len(), 2);
    at(r.strips[0].ends[0], 0.0, 20.0);
    at(r.strips[0].ends[1], 20.0, 20.0);
    at(r.strips[1].ends[0], 40.0, 20.0);
    at(r.strips[1].ends[1], 60.0, 20.0);
    let half = 1200.0 - 50.0 * std::f64::consts::PI;
    close(r.sides[0], half);
    close(r.sides[1], half);
}

/// A plate with a half-disc hole: its arc is a circle of radius 10 about `(30, 20)` exactly, the
/// straight side the diameter at `y = 20`, in two pieces so the loop has three vertices.
fn half_disc_hole(body: &Body) -> Body {
    let sketch = serde_json::json!({
        "points": {"a": [20.0, 20.0], "m": [30.0, 20.0], "b": [40.0, 20.0], "c": [30.0, 20.0]},
        "lines": {"l1": {"from": "a", "to": "m"}, "l2": {"from": "m", "to": "b"}},
        "arcs": {"a1": {"center": "c", "from": "b", "to": "a"}}
    });
    cut(body, sketch)
}

/// A line touching a round hole without crossing it: the touch joins the strip, it does not
/// break it. The line `y = 30` touches the half disc's arc at `(30, 30)`.
#[test]
fn a_line_touching_a_round_hole_keeps_one_strip() {
    let body = half_disc_hole(&plate(60.0, 40.0));
    let r = strips(&body, [0.0, 30.0], [60.0, 30.0]);
    assert_eq!(r.strips.len(), 1);
    at(r.strips[0].ends[0], 0.0, 30.0);
    at(r.strips[0].ends[1], 60.0, 30.0);
    // The hole, a half disc of 50π, lies wholly below the line.
    close(r.sides[0], 600.0);
    close(r.sides[1], 1800.0 - 50.0 * std::f64::consts::PI);
}

/// A plate whose top face has an arc touching the line `y = 0` at the vertex `(10, 0)`, the edge
/// before it coming from below: the loop crosses the line there even though the arc only touches
/// it, so the strip starts at the vertex.
fn tangent_at_a_vertex() -> Body {
    let sketch = serde_json::json!({
        "points": {"p1": [0.0, -10.0], "p2": [30.0, -10.0], "p3": [30.0, 10.0], "p4": [20.0, 10.0], "p5": [10.0, 0.0], "c": [10.0, 10.0]},
        "lines": {
            "l1": {"from": "p1", "to": "p2"}, "l2": {"from": "p2", "to": "p3"},
            "l3": {"from": "p3", "to": "p4"}, "l4": {"from": "p5", "to": "p1"}
        },
        "arcs": {"a1": {"center": "c", "from": "p5", "to": "p4"}}
    });
    let s = op(serde_json::json!({"op": "sketch", "sketch": sketch})).eval(id("arc.sketch"), &[]).unwrap();
    op(serde_json::json!({"op": "extrude", "z0": 0.0, "z1": 2.0})).eval(id("arc"), &[&s]).unwrap().body().unwrap().clone()
}

/// An arc touching the line at a vertex with the edge before it on the other side still crosses
/// it: the strip ends at that vertex.
#[test]
fn an_arc_touching_the_line_at_a_vertex_crosses_it() {
    let r = strips(&tangent_at_a_vertex(), [5.0, 0.0], [25.0, 0.0]);
    assert_eq!(r.strips.len(), 1);
    at(r.strips[0].ends[0], 10.0, 0.0);
    at(r.strips[0].ends[1], 30.0, 0.0);
}

/// Three strips between two cuts: both holes cut through one sketch, as a part has them.
#[test]
fn three_strips_between_two_cuts() {
    let sketch = serde_json::json!({
        "points": {
            "a": [10.0, 10.0], "b": [30.0, 10.0], "c": [30.0, 30.0], "d": [10.0, 30.0],
            "e": [50.0, 10.0], "f": [70.0, 10.0], "g": [70.0, 30.0], "h": [50.0, 30.0]
        },
        "lines": {
            "l1": {"from": "a", "to": "b"}, "l2": {"from": "b", "to": "c"},
            "l3": {"from": "c", "to": "d"}, "l4": {"from": "d", "to": "a"},
            "m1": {"from": "e", "to": "f"}, "m2": {"from": "f", "to": "g"},
            "m3": {"from": "g", "to": "h"}, "m4": {"from": "h", "to": "e"}
        }
    });
    let body = cut(&plate(100.0, 40.0), sketch);
    let r = strips(&body, [0.0, 20.0], [100.0, 20.0]);
    assert_eq!(r.strips.len(), 3);
    at(r.strips[0].ends[0], 0.0, 20.0);
    at(r.strips[0].ends[1], 10.0, 20.0);
    at(r.strips[1].ends[0], 30.0, 20.0);
    at(r.strips[1].ends[1], 50.0, 20.0);
    at(r.strips[2].ends[0], 70.0, 20.0);
    at(r.strips[2].ends[1], 100.0, 20.0);
}

/// A hole wholly on one side of the line is taken from that side alone, and both sides together
/// are the whole material.
#[test]
fn a_hole_on_one_side_is_taken_from_it() {
    let r = strips(&holed(), [45.0, 5.0], [45.0, 35.0]);
    close(r.sides[0], 1200.0);
    close(r.sides[1], 600.0);
    let material =
        shoelace(&[[0.0, 0.0], [60.0, 0.0], [60.0, 40.0], [0.0, 40.0]]) - shoelace(&[[10.0, 10.0], [40.0, 10.0], [40.0, 30.0], [10.0, 30.0]]);
    close(material, 1800.0);
    close(r.sides[0] + r.sides[1], material);
}

/// The area of the closed polygon `p`, measured on its own.
fn shoelace(p: &[[f64; 2]]) -> f64 {
    let n = p.len();
    p.iter()
        .enumerate()
        .map(|(k, a)| {
            let b = p[(k + 1) % n];
            a[0] * b[1] - b[0] * a[1]
        })
        .sum::<f64>()
        .abs()
        / 2.0
}

/// A line of no length has no direction to bend along. The sketch refuses to solve one, so the
/// solved value is made here.
#[test]
fn a_line_of_no_length_is_refused() {
    let body = holed();
    let mut s = on_top(&body, [10.0, 20.0], [20.0, 20.0]);
    s.solved.points.insert("p2".into(), Vec2::new(10.0, 20.0));
    match bend_strips(&body, &s, "l1") {
        Err(BrepError::Degenerate(m)) => assert!(m.starts_with("bend: ")),
        other => panic!("{other:?}"),
    }
}

/// A sketch in the air, on no face of the body, is no bend.
#[test]
fn a_sketch_in_the_air_is_refused() {
    let body = holed();
    let sketch = serde_json::json!({"points": {"p1": [10.0, 20.0], "p2": [20.0, 20.0]}, "lines": {"l1": {"from": "p1", "to": "p2"}}});
    let s = op(serde_json::json!({"op": "sketch", "sketch": sketch})).eval(id("air"), &[]).unwrap();
    match bend_strips(&body, s.sketch().unwrap(), "l1") {
        Err(BrepError::Unsupported(m)) => assert!(m.starts_with("bend: ")),
        other => panic!("{other:?}"),
    }
}

/// A line the sketch has not got.
#[test]
fn an_unknown_line_is_refused() {
    let body = holed();
    assert!(matches!(bend_strips(&body, &on_top(&body, [2.0, 36.0], [50.0, 4.0]), "nope"), Err(BrepError::LostReference(_))));
}

/// A line along an edge of the face: no side of the material is chosen.
#[test]
fn a_line_along_an_edge_is_refused() {
    assert!(degenerate(&holed(), [12.0, 30.0], [38.0, 30.0]).starts_with("bend: "));
}
