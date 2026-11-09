//! Revolution of a sketch's profile about one of its lines (D206): cylinders, cones and planes
//! from the profile's edges, partial turns, an inclined axis, refusals.
#![allow(clippy::unwrap_used, clippy::panic)]

use std::f64::consts::{FRAC_PI_2, PI};

use datum_brep::{Body, BrepError, Surface, TopoId, tessellate};
use datum_ops::{Op, Value};
use serde_json::{Value as Json, json};

fn id(tag: &str) -> TopoId {
    TopoId::derive(tag).finish()
}

fn op(json: Json) -> Op {
    serde_json::from_value(json).unwrap()
}

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-9 * b.abs().max(1.0)
}

/// A closed polygon through `pts`: points `p0..`, lines `e0..` (`e<k>` from `p<k>` to the next).
fn polygon(pts: &[[f64; 2]]) -> Json {
    let n = pts.len();
    let points: serde_json::Map<String, Json> = pts.iter().enumerate().map(|(k, p)| (format!("p{k}"), json!(p))).collect();
    let lines: serde_json::Map<String, Json> =
        (0..n).map(|k| (format!("e{k}"), json!({"from": format!("p{k}"), "to": format!("p{}", (k + 1) % n)}))).collect();
    json!({"points": points, "lines": lines})
}

/// Adds a construction line `ax` from `a` to `b` to a sketch.
fn with_axis(mut sketch: Json, a: [f64; 2], b: [f64; 2]) -> Json {
    sketch["points"]["ax0"] = json!(a);
    sketch["points"]["ax1"] = json!(b);
    sketch["lines"]["ax"] = json!({"from": "ax0", "to": "ax1", "construction": true});
    sketch
}

fn sketch_value(sketch: Json) -> Result<Value, BrepError> {
    op(json!({"op": "sketch", "sketch": sketch})).eval(id("profile"), &[])
}

fn revolve_op(extra: Json) -> Op {
    let mut j = json!({"op": "revolve"});
    j.as_object_mut().unwrap().extend(extra.as_object().unwrap().clone());
    op(j)
}

fn revolved(s: &Value, extra: Json) -> Result<Body, BrepError> {
    let b = revolve_op(extra).eval(id("rev"), &[s])?.body().unwrap().clone();
    b.validate().unwrap();
    tessellate(&b).unwrap();
    Ok(b)
}

fn build(sketch: Json, extra: Json) -> Result<Body, BrepError> {
    revolved(&sketch_value(sketch)?, extra)
}

fn count(b: &Body, pick: fn(&Surface) -> bool) -> usize {
    b.faces().iter().filter(|f| pick(&f.surface)).count()
}

fn cylinders(b: &Body) -> usize {
    count(b, |s| matches!(s, Surface::Cylinder(_)))
}

fn cones(b: &Body) -> usize {
    count(b, |s| matches!(s, Surface::Cone(_)))
}

fn planes(b: &Body) -> usize {
    count(b, |s| matches!(s, Surface::Plane(_)))
}

fn frustum(h: f64, r1: f64, r2: f64) -> f64 {
    PI * h * (r1 * r1 + r1 * r2 + r2 * r2) / 3.0
}

/// A rectangle `x0..x1` by `r0..r1` above the construction axis along X.
fn tube_sketch(x0: f64, x1: f64, r0: f64, r1: f64) -> Json {
    with_axis(polygon(&[[x0, r0], [x1, r0], [x1, r1], [x0, r1]]), [0.0, 0.0], [100.0, 0.0])
}

#[test]
fn a_stepped_shaft_about_its_own_edge() {
    // Along the axis 0..50: Ø20 up to 30, Ø12 up to 48, a 45° chamfer down to Ø8 at the end.
    let pts = [[0.0, 0.0], [50.0, 0.0], [50.0, 4.0], [48.0, 6.0], [30.0, 6.0], [30.0, 10.0], [0.0, 10.0]];
    let b = build(polygon(&pts), json!({"axis": "e0"})).unwrap();
    let want = PI * 100.0 * 30.0 + PI * 36.0 * 18.0 + frustum(2.0, 4.0, 6.0);
    assert!(close(b.volume(), want), "{} vs {want}", b.volume());
    // Three sectors for each of the two cylinders and the chamfer.
    assert_eq!((cylinders(&b), cones(&b)), (6, 3));
    // The end disc, the step, the other end disc; the edge on the axis makes no face.
    assert_eq!(planes(&b), 3);
}

#[test]
fn a_tube_off_a_construction_axis() {
    let (r, big, h) = (5.0, 8.0, 30.0);
    let b = build(tube_sketch(10.0, 40.0, r, big), json!({"axis": "ax"})).unwrap();
    assert!(close(b.volume(), PI * (big * big - r * r) * h), "{}", b.volume());
    let m = b.mass_properties();
    let area = 2.0 * PI * (big + r) * h + 2.0 * PI * (big * big - r * r);
    assert!(close(m.area, area), "{} vs {area}", m.area);
    assert!(close(m.volume, b.volume()));
    assert_eq!((cylinders(&b), cones(&b), planes(&b)), (6, 0, 2));
}

#[test]
fn a_quarter_turn_of_a_trapezoid() {
    let pts = [[10.0, 5.0], [30.0, 5.0], [25.0, 12.0], [10.0, 12.0]];
    // Shoelace area and the centroid's distance from the axis (the X axis).
    let (mut a2, mut cy6) = (0.0_f64, 0.0_f64);
    for k in 0..pts.len() {
        let (p, q) = (pts[k], pts[(k + 1) % pts.len()]);
        let cross = p[0] * q[1] - q[0] * p[1];
        a2 += cross;
        cy6 += (p[1] + q[1]) * cross;
    }
    let (area, ybar) = ((0.5 * a2).abs(), (cy6 / (3.0 * a2)).abs());
    let b = build(with_axis(polygon(&pts), [0.0, 0.0], [100.0, 0.0]), json!({"axis": "ax", "angle": 90})).unwrap();
    let want = area * FRAC_PI_2 * ybar;
    assert!(close(b.volume(), want), "{} vs {want}", b.volume());
    // One face for each of the four edges and two caps.
    assert_eq!((cylinders(&b), cones(&b), planes(&b)), (2, 1, 3));
    assert_eq!(b.faces().len(), 6);
}

/// The tube about the line from (0, 0) to (3, 4): a step of `k` along it is 5 long, a step of
/// `m` square to it is 5 high, and every point is an integer.
fn inclined_tube() -> Json {
    let at = |k: f64, m: f64| [3.0 * k - 4.0 * m, 4.0 * k + 3.0 * m];
    with_axis(polygon(&[at(2.0, 1.0), at(8.0, 1.0), at(8.0, 2.0), at(2.0, 2.0)]), [0.0, 0.0], [3.0, 4.0])
}

#[test]
fn an_axis_not_along_a_sketch_axis() {
    let b = build(inclined_tube(), json!({"axis": "ax"})).unwrap();
    let want = PI * (100.0 - 25.0) * 30.0;
    assert!(close(b.volume(), want), "{} vs {want}", b.volume());
    assert_eq!((cylinders(&b), cones(&b)), (6, 0));
}

#[test]
fn a_sketch_on_another_plane() {
    let plane = json!({"op": "sketch", "sketch": inclined_tube(), "plane": {"origin": [0, 0, 100], "x_axis": [0, 1, 0], "normal": [1, 0, 0]}});
    let s = op(plane).eval(id("profile"), &[]).unwrap();
    let b = revolved(&s, json!({"axis": "ax"})).unwrap();
    let want = PI * (100.0 - 25.0) * 30.0;
    assert!(close(b.volume(), want), "{} vs {want}", b.volume());
    assert_eq!(cones(&b), 0);
}

#[test]
fn constraints_make_a_cylinder_not_a_flat_cone() {
    // Drawn a little off, then constrained to a rectangle 30 by 3 at 5 above the axis.
    let sk = json!({
        "points": {"a": [10, 5.01], "b": [40, 5], "c": [40.2, 8], "d": [10, 8.1], "o": [0, 0], "x": [100, 0]},
        "lines": {
            "ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"},
            "cd": {"from": "c", "to": "d"}, "da": {"from": "d", "to": "a"},
            "ax": {"from": "o", "to": "x", "construction": true}
        },
        "constraints": [
            {"type": "fix", "point": "o"}, {"type": "fix", "point": "x"},
            {"type": "fix", "point": "a", "at": [10, 5]},
            {"type": "horizontal", "line": "ab"}, {"type": "length", "line": "ab", "value": 30},
            {"type": "vertical", "line": "bc"}, {"type": "horizontal", "line": "cd"},
            {"type": "vertical", "line": "da"}, {"type": "point_line", "point": "d", "line": "ax", "value": 8}
        ]
    });
    let b = build(sk, json!({"axis": "ax"})).unwrap();
    assert_eq!((cylinders(&b), cones(&b)), (6, 0));
    let want = PI * (64.0 - 25.0) * 30.0;
    assert!(close(b.volume(), want), "{} vs {want}", b.volume());
}

fn refusal(sketch: Json, extra: Json) -> BrepError {
    build(sketch, extra).unwrap_err()
}

/// A profile with an arc, on either side of the axis, as a sketch draws it: a rounded corner.
#[test]
fn an_arc_in_the_profile_turns_into_a_torus() {
    for y in [1.0, -1.0] {
        let sk = with_axis(
            json!({
                "points": {"a": [10, 5.0 * y], "b": [30, 5.0 * y], "c": [32, 7.0 * y], "d": [10, 7.0 * y], "m": [30, 7.0 * y]},
                "lines": {"ab": {"from": "a", "to": "b"}, "cd": {"from": "c", "to": "d"}, "da": {"from": "d", "to": "a"}},
                "arcs": {"round": if y > 0.0 { json!({"center": "m", "from": "b", "to": "c"}) } else { json!({"center": "m", "from": "c", "to": "b"}) }}
            }),
            [0.0, 0.0],
            [100.0, 0.0],
        );
        let b = build(sk, json!({"axis": "ax"})).unwrap();
        // The tube of a ring of 5 to 7 with one end rounded inward (the centre at the outer radius).
        let pi = PI;
        let corner = (pi * 4.0 / 4.0, 7.0 - 4.0 * 2.0 / (3.0 * pi));
        let expected = pi * (49.0 - 25.0) * 20.0 + 2.0 * pi * corner.1 * corner.0;
        assert!(close(b.volume(), expected), "{} vs {expected}", b.volume());
        assert!(b.faces().iter().any(|f| matches!(f.surface, Surface::Torus(t) if t.convex && close(t.major, 7.0) && close(t.minor, 2.0))));
    }
}

/// A ball: an arc drawn from one end of the axis to the other, closed by the axis line itself.
#[test]
fn a_half_circle_on_the_axis_turns_into_a_ball() {
    let sk = json!({
        "points": {"a": [-4, 0], "b": [4, 0], "m": [0, 0]},
        "lines": {"ax": {"from": "a", "to": "b"}},
        "arcs": {"top": {"center": "m", "from": "b", "to": "a"}}
    });
    let b = build(sk, json!({"axis": "ax"})).unwrap();
    assert!(close(b.volume(), 4.0 / 3.0 * PI * 64.0), "{}", b.volume());
    assert_eq!(b.faces().len(), 6);
    assert!(b.faces().iter().all(|f| matches!(f.surface, Surface::Torus(t) if t.major == 0.0 && t.convex)));
}

#[test]
fn a_cone_to_a_point_is_refused() {
    let sk = with_axis(polygon(&[[10.0, 0.0], [30.0, 5.0], [10.0, 5.0]]), [0.0, 0.0], [100.0, 0.0]);
    assert!(matches!(refusal(sk, json!({"axis": "ax"})), BrepError::Unsupported(m) if m.contains("cone to a point")));
}

#[test]
fn a_profile_across_the_axis_is_refused() {
    assert!(matches!(refusal(tube_sketch(10.0, 40.0, -3.0, 4.0), json!({"axis": "ax"})), BrepError::Degenerate(m) if m.contains("crosses the axis")));
}

#[test]
fn angles_out_of_range_are_refused() {
    for angle in [0.0, -90.0, 400.0] {
        let err = refusal(tube_sketch(10.0, 40.0, 5.0, 8.0), json!({"axis": "ax", "angle": angle}));
        assert!(matches!(err, BrepError::Degenerate(_)), "{angle}: {err:?}");
    }
}

#[test]
fn several_profiles_must_be_chosen() {
    let mut sk = tube_sketch(10.0, 40.0, 5.0, 8.0);
    for (k, p) in [[50.0, 5.0], [60.0, 5.0], [60.0, 8.0], [50.0, 8.0]].iter().enumerate() {
        sk["points"][format!("q{k}")] = json!(p);
    }
    for k in 0..4 {
        sk["lines"][format!("f{k}")] = json!({"from": format!("q{k}"), "to": format!("q{}", (k + 1) % 4)});
    }
    assert!(matches!(refusal(sk, json!({"axis": "ax"})), BrepError::Unsupported(m) if m.contains("choose it")));
}

#[test]
fn an_unknown_axis_is_refused() {
    assert!(matches!(refusal(tube_sketch(10.0, 40.0, 5.0, 8.0), json!({"axis": "nope"})), BrepError::LostReference(_)));
}

#[test]
fn the_same_revolution_hashes_the_same() {
    let hashes: Vec<_> = (0..2).map(|_| build(inclined_tube(), json!({"axis": "ax", "angle": 120})).unwrap().hash()).collect();
    assert_eq!(hashes[0], hashes[1]);
    let full = build(inclined_tube(), json!({"axis": "ax"})).unwrap().hash();
    assert_ne!(full, hashes[0]);
}
