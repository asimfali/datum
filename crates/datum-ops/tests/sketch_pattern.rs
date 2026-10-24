//! Patterns of a sketch (D128): copies of a group on a grid, so many or filling a region,
//! centred, clear of other profiles; skipped and shifted copies; extruded and placed.
#![allow(clippy::unwrap_used, clippy::panic)]

use std::f64::consts::PI;

use datum_brep::{BrepError, TopoId, tessellate};
use datum_ops::{Op, Placed, Value};

fn id(tag: &str) -> TopoId {
    TopoId::derive(tag).finish()
}

fn assert_close(a: f64, b: f64) {
    assert!((a - b).abs() <= 1e-9 * b.abs().max(1.0), "{a} vs {b}");
}

fn placed(sketch: serde_json::Value) -> Result<Value, BrepError> {
    let op: Op = serde_json::from_value(serde_json::json!({"op": "sketch", "sketch": sketch})).unwrap();
    op.eval(id("s"), &[])
}

fn copies(v: &Value) -> &Placed {
    v.sketch().unwrap()
}

/// A plate 100 × 60 (lines in a group `panel`), a square hole 20 × 20 in its middle, and a hole
/// Ø5 off the plate: the cell, a sample.
fn plate(pattern: serde_json::Value) -> serde_json::Value {
    serde_json::json!({
        "points": {"a": [0, 0], "b": [100, 0], "c": [100, 60], "d": [0, 60], "e": [40, 20], "f": [60, 20], "g": [60, 40], "k": [40, 40], "h": [200, 0]},
        "lines": {"bottom": {"from": "a", "to": "b"}, "right": {"from": "b", "to": "c"}, "top": {"from": "c", "to": "d"}, "left": {"from": "d", "to": "a"},
                  "w1": {"from": "e", "to": "f"}, "w2": {"from": "f", "to": "g"}, "w3": {"from": "g", "to": "k"}, "w4": {"from": "k", "to": "e"}},
        "circles": {"hole": {"center": "h", "diameter": 5}},
        "groups": {"panel": ["bottom", "right", "top", "left"], "cell": ["hole"]},
        "patterns": {"perf": pattern}
    })
}

/// Filling: centred on the plate, every hole's box 3 clear of its edge and of the window. Across
/// 100, centres within 44.5 of the middle on a step of 10: 9, on the half step 8; across 60, 5
/// or 4. The window and 3 around it take 3 × 3 of the 9 × 5. The sample is not cut.
#[test]
fn a_region_is_filled_centred_and_clear() {
    let s = placed(plate(serde_json::json!({"of": "cell", "step": [[10, 0], [0, 10]], "fill": "panel", "clearance": 3}))).unwrap();
    let p = &copies(&s).patterns[0];
    assert_eq!(p.at.len(), 36);
    assert!(!p.source);
    let xs: std::collections::BTreeSet<i64> = p.at.iter().map(|(_, d)| (d.x + 200.0) as i64).collect();
    assert_eq!(xs.into_iter().collect::<Vec<_>>(), [10, 20, 30, 40, 50, 60, 70, 80, 90], "centred: 10 from either side");
    assert_close(p.area, 6.25 * PI);
    assert_close(p.region.unwrap(), 6000.0);
    let extrude: Op = serde_json::from_value(serde_json::json!({"op": "extrude", "z0": 0, "z1": 2})).unwrap();
    let b = extrude.eval(id("plate"), &[&s]).unwrap();
    let b = b.body().unwrap();
    b.validate().unwrap();
    tessellate(b).unwrap();
    assert_close(b.volume(), 2.0 * (6000.0 - 400.0 - 36.0 * 6.25 * PI));
}

/// A grid of 3 × 2 from the cell itself, copy (1, 0) skipped and (2, 1) moved up 1: five holes.
#[test]
fn a_grid_skips_and_shifts() {
    let s = placed(serde_json::json!({
        "points": {"h": [10, 10]}, "circles": {"hole": {"center": "h", "diameter": 4}},
        "patterns": {"row": {"of": "hole", "count": [3, 2], "step": [[10, 0], [0, 10]], "skip": [[1, 0]], "shift": {"2,1": [0, 1]}}}
    }))
    .unwrap();
    let p = &copies(&s).patterns[0];
    let at: Vec<([i64; 2], [f64; 2])> = p.at.iter().map(|(k, d)| (*k, [d.x, d.y])).collect();
    assert_eq!(at, [([0, 0], [0.0, 0.0]), ([2, 0], [20.0, 0.0]), ([0, 1], [0.0, 10.0]), ([1, 1], [10.0, 10.0]), ([2, 1], [20.0, 11.0])]);
    assert!(p.source);
}

/// Copies that would touch are refused, as are a step along one line when filling, a count
/// that is not whole, and a cell with no closed profile.
#[test]
fn bad_patterns_are_refused() {
    let hole = |pattern: serde_json::Value| {
        serde_json::json!({"points": {"h": [0, 0], "p": [5, 5], "q": [9, 5]}, "circles": {"hole": {"center": "h", "diameter": 4}},
            "lines": {"open": {"from": "p", "to": "q", "construction": true}}, "patterns": {"p1": pattern}})
    };
    let refused = |pattern: serde_json::Value, why: &str| match placed(hole(pattern)) {
        Err(e) => assert!(e.to_string().contains(why), "{e}"),
        Ok(_) => panic!("expected `{why}`"),
    };
    refused(serde_json::json!({"of": "hole", "count": [2, 1], "step": [[3, 0], [0, 1]]}), "touch or hold one another");
    refused(serde_json::json!({"of": "hole", "count": [2.5, 1], "step": [[5, 0], [0, 5]]}), "whole numbers");
    refused(serde_json::json!({"of": "open", "count": [2, 1], "step": [[5, 0], [0, 5]]}), "no closed profile");
    let s = plate(serde_json::json!({"of": "cell", "step": [[10, 0], [20, 0]], "fill": "panel"}));
    assert!(placed(s).unwrap_err().to_string().contains("along one line"));
}

/// The same pattern placed twice is the same, bit for bit (on any number of threads: the copies
/// are found in parallel and kept in grid order).
#[test]
fn filling_is_deterministic() {
    let s = plate(serde_json::json!({"of": "cell", "step": [[7.4, 0], [3.7, 6.4]], "fill": "panel", "clearance": 0.5}));
    let one = rayon::ThreadPoolBuilder::new().num_threads(1).build().unwrap().install(|| placed(s.clone()).unwrap());
    let all = placed(s).unwrap();
    assert_eq!(copies(&one).patterns, copies(&all).patterns);
    assert_eq!(copies(&one).hash, copies(&all).hash);
}
