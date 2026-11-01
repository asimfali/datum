//! Rolled sheet metal (D207): a shell from a line parallel to the axis, a cone from an oblique one,
//! the gap at the seam along the neutral layer, the blank unfolded flat, refusals.
#![allow(clippy::unwrap_used, clippy::panic)]

use std::f64::consts::{PI, TAU};

use datum_brep::{Body, BrepError, Surface, TopoId, tessellate};
use datum_ops::{Flat, FlatPiece, Op, Value, unfold_side};
use serde_json::{Value as Json, json};

const T: f64 = 2.0;
const K: f64 = 0.44;
const GAP: f64 = 2.0;

fn id(tag: &str) -> TopoId {
    TopoId::derive(tag).finish()
}

fn op(json: Json) -> Op {
    serde_json::from_value(json).unwrap()
}

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-9 * b.abs().max(1.0)
}

/// A sketch with the construction axis `ax` from `a0` to `a1` and the line `path` from `p` to `q`.
fn sketch(a0: [f64; 2], a1: [f64; 2], p: [f64; 2], q: [f64; 2]) -> Json {
    json!({
        "points": {"a0": a0, "a1": a1, "p": p, "q": q},
        "lines": {"ax": {"from": "a0", "to": "a1", "construction": true}, "path": {"from": "p", "to": "q"}}
    })
}

/// The axis along X and the line from `p` to `q`.
fn along_x(p: [f64; 2], q: [f64; 2]) -> Json {
    sketch([0.0, 0.0], [100.0, 0.0], p, q)
}

fn sketch_value(sketch: Json) -> Value {
    op(json!({"op": "sketch", "sketch": sketch})).eval(id("profile"), &[]).unwrap()
}

fn rolled_op(side: &str, gap: f64) -> Json {
    json!({"op": "rolled", "axis": "ax", "side": side, "gap": gap, "thickness": T, "k_factor": K})
}

fn eval(sketch: Json, mut j: Json) -> Result<Body, BrepError> {
    if j.get("op").is_none() {
        j = rolled_op("left", GAP).as_object().unwrap().iter().chain(j.as_object().unwrap()).map(|(k, v)| (k.clone(), v.clone())).collect();
    }
    let b = op(j).eval(id("roll"), &[&sketch_value(sketch)])?.body().unwrap().clone();
    b.validate().unwrap();
    tessellate(&b).unwrap();
    Ok(b)
}

fn rolled(sketch: Json, side: &str, gap: f64) -> Body {
    eval(sketch, rolled_op(side, gap)).unwrap()
}

fn flat(b: &Body) -> Flat {
    unfold_side(b, false, K).unwrap()
}

fn count(b: &Body, pick: fn(&Surface) -> bool) -> usize {
    b.faces().iter().filter(|f| pick(&f.surface)).count()
}

fn cones(b: &Body) -> usize {
    count(b, |s| matches!(s, Surface::Cone(_)))
}

fn cylinders(b: &Body) -> usize {
    count(b, |s| matches!(s, Surface::Cylinder(_)))
}

fn planes(b: &Body) -> usize {
    count(b, |s| matches!(s, Surface::Plane(_)))
}

#[test]
fn a_shell_on_each_side_of_its_line() {
    let (r, l) = (50.0, 100.0);
    // Inner radius of the sheet on the side of the line; the neutral layer is K*T above it.
    for (side, r_in) in [("left", r), ("right", r - T), ("middle", r - 0.5 * T)] {
        let b = rolled(along_x([0.0, r], [l, r]), side, GAP);
        // The neutral radius Rn, the gap turns the sheet by TAU - GAP / Rn; the volume is
        // that fraction of a full turn of the ring: (TAU - GAP / Rn) / 2 * (Rout^2 - Rin^2) * L.
        let rn = r_in + K * T;
        let r_out = r_in + T;
        let want = (TAU - GAP / rn) / 2.0 * (r_out * r_out - r_in * r_in) * l;
        assert!(close(b.volume(), want), "{side}: {} vs {want}", b.volume());
        assert_eq!(cones(&b), 0, "{side}");
        let f = flat(&b);
        // The neutral layer unrolled: TAU * Rn - GAP long, L across.
        assert!(close(f.size.x, TAU * rn - GAP) && close(f.size.y, l), "{side}: {:?}", f.size);
        assert_eq!(f.counts(), [4, 0, 0], "{side}");
    }
}

#[test]
fn the_shell_of_the_example_is_a_known_rectangle() {
    let f = flat(&rolled(along_x([0.0, 50.0], [100.0, 50.0]), "left", GAP));
    assert!(close(f.size.x, 317.68846842929736) && close(f.size.y, 100.0), "{:?}", f.size);
}

/// The neutral layer of a cone: where the inner face is, and K*T further from the axis.
/// For a line `p` to `q` above the axis on `side`, the distances from the axis to the neutral
/// layer at both ends. Sheet offsets are along the line's left normal `n`: left 0..T, right -T..0;
/// the face nearer the axis is the inner one, which is the `n`-lowest offset when `n` points up.
fn neutral_radii(p: [f64; 2], q: [f64; 2], side: &str) -> [f64; 2] {
    let len = libm::hypot(q[0] - p[0], q[1] - p[1]);
    let n = [-(q[1] - p[1]) / len, (q[0] - p[0]) / len];
    let (near, far) = if side == "left" { (0.0, T) } else { (-T, 0.0) };
    let off = if n[1] > 0.0 { near + K * T } else { far - K * T };
    [p[1] + off * n[1], q[1] + off * n[1]]
}

/// Checks a cone of the line `p`-`q` (above the axis, sheet on `side` there): its faces, the
/// annular sector, and the gap on the larger arc.
fn check_cone(b: &Body, p: [f64; 2], q: [f64; 2], side: &str) {
    let (dy, dx) = (q[1] - p[1], q[0] - p[0]);
    let len = libm::hypot(dx, dy);
    // Half-angle of the cone: sin(alpha) = slope / sqrt(1 + slope^2).
    let sin_a = dy.abs() / len;
    assert_eq!((cones(b), planes(b)), (4, 2), "{side}");
    let f = flat(b);
    assert_eq!(f.counts(), [2, 2, 0], "{side}");
    // The arcs are centred at the apex, at the distance rho = r / sin(alpha) along the generatrix
    // from it to each neutral end, whose distance from the axis is r.
    let rs = neutral_radii(p, q, side);
    let mut want = [rs[0] / sin_a, rs[1] / sin_a];
    want.sort_by(f64::total_cmp);
    let mut arcs: Vec<(f64, f64)> = f
        .pieces
        .iter()
        .filter_map(|x| match x {
            FlatPiece::Arc { radius, start, end, .. } => Some((*radius, end - start)),
            _ => None,
        })
        .collect();
    arcs.sort_by(|a, b| a.0.total_cmp(&b.0));
    assert!(close(arcs[0].0, want[0]) && close(arcs[1].0, want[1]), "{side}: {arcs:?} vs {want:?}");
    // The ends square to the line: the difference of the radii is the line's length.
    assert!(close(arcs[1].0 - arcs[0].0, len), "{side}: {}", arcs[1].0 - arcs[0].0);
    // Both arcs span the same angle: (TAU - GAP / Rn_max) * sin(alpha).
    let rn_max = want[1] * sin_a;
    let angle = (TAU - GAP / rn_max) * sin_a;
    assert!(close(arcs[0].1, angle) && close(arcs[1].1, angle), "{side}: {arcs:?} vs {angle}");
    // The gap measured on the blank along the larger arc.
    assert!(close((TAU * sin_a - arcs[1].1) * arcs[1].0, GAP), "{side}");
}

#[test]
fn a_cone_on_each_side() {
    let (p, q) = ([0.0, 50.0], [100.0, 80.0]);
    for side in ["left", "right"] {
        check_cone(&rolled(along_x(p, q), side, GAP), p, q, side);
    }
}

#[test]
fn the_example_cone_is_a_known_sector() {
    let f = flat(&rolled(along_x([0.0, 50.0], [100.0, 80.0]), "left", GAP));
    let mut radii: Vec<f64> = f.pieces.iter().filter_map(|x| if let FlatPiece::Arc { radius, .. } = x { Some(*radius) } else { None }).collect();
    radii.sort_by(f64::total_cmp);
    assert!(close(radii[0], 176.93844181517585) && close(radii[1], 281.34150690428135), "{radii:?}");
    let angle = f.pieces.iter().find_map(|x| if let FlatPiece::Arc { start, end, .. } = x { Some(end - start) } else { None }).unwrap();
    assert!((angle - 1.79836).abs() < 1e-4, "{angle}");
}

#[test]
fn a_cone_narrowing_and_a_line_below_the_axis() {
    // Large radius to small.
    let (p, q) = ([0.0, 80.0], [100.0, 50.0]);
    for side in ["left", "right"] {
        check_cone(&rolled(along_x(p, q), side, GAP), p, q, side);
    }
    // Below the axis: the mirror image, in which left of the line is right of its mirror.
    let (p, q) = ([0.0, 50.0], [100.0, 80.0]);
    for (side, mirrored) in [("left", "right"), ("right", "left")] {
        let b = rolled(along_x([p[0], -p[1]], [q[0], -q[1]]), side, GAP);
        check_cone(&b, p, q, mirrored);
    }
}

#[test]
fn an_axis_not_along_a_sketch_axis() {
    // The tube about the line from (0, 0) to (3, 4): a step of k along it is 5 long, a step of m
    // square to it is 5 high, and every point is an integer.
    let at = |k: f64, m: f64| [3.0 * k - 4.0 * m, 4.0 * k + 3.0 * m];
    let (r, l) = (10.0, 30.0);
    let b = rolled(sketch([0.0, 0.0], [3.0, 4.0], at(2.0, r / 5.0), at(8.0, r / 5.0)), "left", GAP);
    let (r_in, rn) = (r, r + K * T);
    let want = (TAU - GAP / rn) / 2.0 * ((r_in + T).powi(2) - r_in * r_in) * l;
    assert!(close(b.volume(), want), "{} vs {want}", b.volume());
    assert_eq!(cones(&b), 0);
    assert!(cylinders(&b) > 0);
}

#[test]
fn the_seam_is_symmetric_about_the_sketch_plane() {
    let b = rolled(along_x([0.0, 50.0], [100.0, 50.0]), "left", GAP);
    // The planes with no X component in the normal are the seam's two faces; the ends are square
    // to the axis.
    let seams: Vec<_> = b.faces().iter().filter_map(|f| f.surface.plane()).filter(|p| p.normal.x.abs() < 1e-9).collect();
    assert_eq!(seams.len(), 2);
    let (a, c) = (seams[0].normal, seams[1].normal);
    assert!(close(a.y, c.y) && close(a.z, -c.z), "{a:?} {c:?}");
    assert!(a.z.abs() > 0.0);
}

fn refusal(sketch: Json, extra: Json) -> BrepError {
    eval(sketch, extra).unwrap_err()
}

#[test]
fn refusals() {
    let ok = || along_x([0.0, 50.0], [100.0, 50.0]);
    for gap in [0.0, -1.0] {
        assert!(matches!(refusal(ok(), json!({"gap": gap})), BrepError::Degenerate(m) if m.contains("gap")), "{gap}");
    }
    // A line square to the axis is a flat ring.
    let ring = along_x([20.0, 10.0], [20.0, 50.0]);
    assert!(matches!(refusal(ring, json!({})), BrepError::Unsupported(_)));
    // The sheet rule's numbers are missing.
    for missing in ["thickness", "k_factor"] {
        let mut j = rolled_op("left", GAP);
        j.as_object_mut().unwrap().remove(missing);
        assert!(matches!(refusal(ok(), j), BrepError::Invalid(_)), "{missing}");
    }
    // Two lines to roll.
    let mut two = ok();
    two["points"]["r"] = json!([0.0, 70.0]);
    two["points"]["s"] = json!([100.0, 70.0]);
    two["lines"]["other"] = json!({"from": "r", "to": "s"});
    assert!(matches!(refusal(two.clone(), json!({})), BrepError::Unsupported(m) if m.contains("choose")));
    let b = eval(two.clone(), json!({"path": "other"})).unwrap();
    let want = (TAU - GAP / (70.0 + K * T)) / 2.0 * (72.0_f64.powi(2) - 70.0_f64.powi(2)) * 100.0;
    assert!(close(b.volume(), want), "{} vs {want}", b.volume());
    let b = eval(two, json!({"path": "path"})).unwrap();
    let want = (TAU - GAP / (50.0 + K * T)) / 2.0 * (52.0_f64.powi(2) - 50.0_f64.powi(2)) * 100.0;
    assert!(close(b.volume(), want), "{} vs {want}", b.volume());
    // Arcs.
    let mut arcs = ok();
    arcs["points"]["m"] = json!([50.0, 60.0]);
    arcs["points"]["u"] = json!([40.0, 60.0]);
    arcs["points"]["v"] = json!([60.0, 60.0]);
    arcs["arcs"] = json!({"round": {"center": "m", "from": "u", "to": "v"}});
    assert!(matches!(refusal(arcs, json!({})), BrepError::Unsupported(_)));
    // A line across the axis.
    let across = along_x([0.0, -20.0], [100.0, 30.0]);
    assert!(matches!(refusal(across, json!({})), BrepError::Degenerate(_)));
}

#[test]
fn the_same_roll_hashes_the_same() {
    for (p, q) in [([0.0, 50.0], [100.0, 50.0]), ([0.0, 50.0], [100.0, 80.0])] {
        let a = rolled(along_x(p, q), "left", GAP);
        let b = rolled(along_x(p, q), "left", GAP);
        assert_eq!(a.hash(), b.hash());
        assert_eq!(flat(&a).hash, flat(&b).hash);
        assert_ne!(a.hash(), rolled(along_x(p, q), "right", GAP).hash());
    }
}

/// A piece of a shell (D213): half a turn from the sketch's plane, no gap; the blank half the
/// neutral layer's round. A whole turn with no gap, or more than 360°, is refused.
#[test]
fn half_a_shell() {
    let (r, l) = (50.0, 100.0);
    let b = eval(along_x([0.0, r], [l, r]), json!({"angle": 180, "gap": null})).unwrap();
    let want = 0.5 * PI * ((r + T) * (r + T) - r * r) * l;
    assert!(close(b.volume(), want), "{} vs {want}", b.volume());
    let f = flat(&b);
    assert!(close(f.size.x, PI * (r + K * T)) && close(f.size.y, l), "{:?}", f.size);
    // The line drawn lies on its edge: a vertex of the body is where the line starts.
    assert!(b.vertices().iter().any(|v| (v.point - datum_math::Vec3::new(0.0, r, 0.0)).dot(v.point - datum_math::Vec3::new(0.0, r, 0.0)) < 1e-20));
    assert!(matches!(eval(along_x([0.0, r], [l, r]), json!({"gap": null})), Err(BrepError::Degenerate(m)) if m.contains("gap")));
    assert!(matches!(eval(along_x([0.0, r], [l, r]), json!({"angle": 400})), Err(BrepError::Degenerate(_))));
}
