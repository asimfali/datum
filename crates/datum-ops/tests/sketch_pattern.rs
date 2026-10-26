//! Patterns of a sketch (D128, D153): copies of a group on a grid, so many or filling a region,
//! centred, clear of other profiles; along a path and around a point; growing steps; skipped and
//! shifted copies; extruded and placed.
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
    let xs: std::collections::BTreeSet<i64> = p.at.iter().map(|(_, d)| (d.by.x + 200.0) as i64).collect();
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
    let at: Vec<([i64; 2], [f64; 2])> = p.at.iter().map(|(k, d)| (*k, [d.by.x, d.by.y])).collect();
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

/// Each copy's centre (its cell's centre at `c`, moved to its place), by copy.
fn centres(v: &Value, c: [f64; 2]) -> Vec<([i64; 2], [f64; 2])> {
    copies(v).patterns[0]
        .at
        .iter()
        .map(|(k, p)| {
            let q = p.apply(datum_math::Vec2::new(c[0], c[1]));
            (*k, [q.x, q.y])
        })
        .collect()
}

fn assert_centres(v: &Value, c: [f64; 2], want: &[[f64; 2]]) {
    let got = centres(v, c);
    assert_eq!(got.len(), want.len(), "{got:?}");
    for (i, ((k, g), w)) in got.iter().zip(want).enumerate() {
        assert_eq!(*k, [i as i64, 0]);
        assert_close(g[0], w[0]);
        assert_close(g[1], w[1]);
    }
}

/// A hole Ø4 at `h` and a construction path: a line from (0, 0) to (100, 0), then a quarter arc
/// around (100, 50) up to (150, 50).
fn path_sketch(h: [f64; 2], path: serde_json::Value, pattern: serde_json::Value) -> serde_json::Value {
    serde_json::json!({
        "points": {"h": h, "a": [0, 0], "b": [100, 0], "c": [100, 50], "d": [150, 50]},
        "circles": {"hole": {"center": "h", "diameter": 4}},
        "lines": {"l1": {"from": "a", "to": "b", "construction": true}},
        "arcs": {"r1": {"center": "c", "from": "b", "to": "d", "construction": true}},
        "groups": {"edge": path},
        "patterns": {"p": pattern}
    })
}

/// Along a line then round an arc, 10 to the left of the path: on the arc the copies keep their
/// distance from it, whichever curve the group names first.
#[test]
fn copies_follow_a_path_of_lines_and_arcs() {
    let pattern = serde_json::json!({"of": "hole", "along": {"path": "edge", "step": 50, "count": 4}});
    let phi = -std::f64::consts::FRAC_PI_2 + 1.0;
    let on_arc = [100.0 + 40.0 * phi.cos(), 50.0 + 40.0 * phi.sin()];
    let want = [[0.0, 10.0], [50.0, 10.0], [100.0, 10.0], on_arc];
    for group in [serde_json::json!(["l1", "r1"]), serde_json::json!(["r1", "l1"])] {
        let s = placed(path_sketch([0.0, 10.0], group, pattern.clone())).unwrap();
        assert_centres(&s, [0.0, 10.0], &want);
        assert!(copies(&s).patterns[0].source);
    }
    // Not turned, a copy only moves with the path: on the arc, off its distance.
    let pattern = serde_json::json!({"of": "hole", "along": {"path": "edge", "step": 50, "count": 4, "rotate": false}});
    let s = placed(path_sketch([0.0, 10.0], serde_json::json!(["l1", "r1"]), pattern)).unwrap();
    let last = centres(&s, [0.0, 10.0])[3].1;
    assert_close(last[0], 100.0 + 50.0 * phi.cos());
    assert_close(last[1], 60.0 + 50.0 * phi.sin());
}

/// `count` alone spreads the copies from the cell to the path's end; `step` alone fits as many as
/// the path is long, centred, the cell a sample; steps that grow.
#[test]
fn copies_spread_fit_and_grow_along_a_path() {
    let line = serde_json::json!(["l1"]);
    let s = placed(path_sketch([20.0, -5.0], line.clone(), serde_json::json!({"of": "hole", "along": {"path": "edge", "count": 5}}))).unwrap();
    assert_centres(&s, [20.0, -5.0], &[[20.0, -5.0], [40.0, -5.0], [60.0, -5.0], [80.0, -5.0], [100.0, -5.0]]);
    let s = placed(path_sketch([20.0, -5.0], line.clone(), serde_json::json!({"of": "hole", "along": {"path": "edge", "step": 30}}))).unwrap();
    assert_centres(&s, [20.0, -5.0], &[[5.0, -5.0], [35.0, -5.0], [65.0, -5.0], [95.0, -5.0]]);
    assert!(!copies(&s).patterns[0].source);
    let s = placed(path_sketch(
        [20.0, -5.0],
        line.clone(),
        serde_json::json!({"of": "hole", "along": {"path": "edge", "step": 10, "count": 4, "grow": 5}}),
    ))
    .unwrap();
    assert_centres(&s, [20.0, -5.0], &[[20.0, -5.0], [30.0, -5.0], [45.0, -5.0], [65.0, -5.0]]);
    // Spread with growing steps: 0, h, 2h + 5, 3h + 15 = 80 from the cell to the end.
    let s = placed(path_sketch([20.0, -5.0], line, serde_json::json!({"of": "hole", "along": {"path": "edge", "count": 4, "grow": 5}}))).unwrap();
    let h = (80.0 - 15.0) / 3.0;
    assert_centres(&s, [20.0, -5.0], &[[20.0, -5.0], [20.0 + h, -5.0], [25.0 + 2.0 * h, -5.0], [100.0, -5.0]]);
}

/// Round a closed path, `count` alone spreads the copies evenly; a shift moves one along the path
/// and across it; a skip leaves one out.
#[test]
fn copies_go_round_a_closed_path_shifted_and_skipped() {
    let s = placed(serde_json::json!({
        "points": {"h": [20, 5], "a": [0, 0], "b": [100, 0], "c": [100, 100], "d": [0, 100]},
        "circles": {"hole": {"center": "h", "diameter": 2}},
        "lines": {"s1": {"from": "a", "to": "b", "construction": true}, "s2": {"from": "b", "to": "c", "construction": true},
                  "s3": {"from": "c", "to": "d", "construction": true}, "s4": {"from": "d", "to": "a", "construction": true}},
        "groups": {"square": ["s1", "s2", "s3", "s4"]},
        "patterns": {"p": {"of": "hole", "along": {"path": "square", "count": 8}, "skip": [[2, 0]], "shift": {"1,0": [5, 1]}}}
    }))
    .unwrap();
    // Inside the square, 5 to the left of its sides: 20 along the first, then every 50.
    let got = centres(&s, [20.0, 5.0]);
    let want = [
        ([0, 0], [20.0, 5.0]),
        ([1, 0], [75.0, 6.0]),
        ([3, 0], [95.0, 70.0]),
        ([4, 0], [80.0, 95.0]),
        ([5, 0], [30.0, 95.0]),
        ([6, 0], [5.0, 80.0]),
        ([7, 0], [5.0, 30.0]),
    ];
    assert_eq!(got.len(), want.len(), "{got:?}");
    for ((k, g), (wk, w)) in got.iter().zip(want) {
        assert_eq!(*k, wk);
        assert_close(g[0], w[0]);
        assert_close(g[1], w[1]);
    }
}

/// Around a point: evenly round, spread over an angle, `step` apart and growing, shifted out.
#[test]
fn copies_go_around_a_point() {
    let around = |a: serde_json::Value| {
        placed(serde_json::json!({"points": {"h": [20, 0], "o": [0, 0]}, "circles": {"hole": {"center": "h", "diameter": 4}},
            "patterns": {"p": {"of": "hole", "around": a, "shift": {"9,0": [0, 0]}}}}))
        .unwrap()
    };
    let r = std::f64::consts::FRAC_1_SQRT_2 * 20.0;
    assert_centres(&around(serde_json::json!({"center": "o", "count": 4})), [20.0, 0.0], &[[20.0, 0.0], [0.0, 20.0], [-20.0, 0.0], [0.0, -20.0]]);
    assert_centres(&around(serde_json::json!({"center": "o", "count": 3, "angle": 90})), [20.0, 0.0], &[[20.0, 0.0], [r, r], [0.0, 20.0]]);
    assert_centres(&around(serde_json::json!({"center": "o", "count": 3, "step": -45, "grow": -45})), [20.0, 0.0], &[[20.0, 0.0], [r, -r], [-r, -r]]);
    let s = placed(serde_json::json!({"points": {"h": [20, 0], "o": [0, 0]}, "circles": {"hole": {"center": "h", "diameter": 4}},
        "patterns": {"p": {"of": "hole", "around": {"center": "o", "count": 2, "step": 90}, "shift": {"1,0": [0, 5]}}}}))
    .unwrap();
    assert_centres(&s, [20.0, 0.0], &[[20.0, 0.0], [0.0, 25.0]]);
}

/// On a grid, each step `grow` longer than the one before.
#[test]
fn grid_steps_grow() {
    let s = placed(serde_json::json!({
        "points": {"h": [0, 0]}, "circles": {"hole": {"center": "h", "diameter": 4}},
        "patterns": {"row": {"of": "hole", "count": [4, 1], "step": [[10, 0], [0, 10]], "grow": [[2, 0], [0, 0]]}}
    }))
    .unwrap();
    let xs: Vec<f64> = copies(&s).patterns[0].at.iter().map(|(_, p)| p.by.x).collect();
    assert_eq!(xs, [0.0, 10.0, 22.0, 36.0]);
}

/// Slots turned along an arc are cut as drawn: the plate is whole, each slot takes its area.
#[test]
fn slots_along_an_arc_are_extruded() {
    let s = placed(serde_json::json!({
        "points": {"a": [-100, -100], "b": [100, -100], "c": [100, 100], "d": [-100, 100], "o": [0, 0], "s": [60, 0], "e": [0, 60],
                   "p1": [56, -2], "p2": [64, -2], "p3": [64, 2], "p4": [56, 2]},
        "lines": {"bottom": {"from": "a", "to": "b"}, "right": {"from": "b", "to": "c"}, "top": {"from": "c", "to": "d"}, "left": {"from": "d", "to": "a"},
                  "q1": {"from": "p1", "to": "p2"}, "q2": {"from": "p2", "to": "p3"}, "q3": {"from": "p3", "to": "p4"}, "q4": {"from": "p4", "to": "p1"}},
        "arcs": {"guide": {"center": "o", "from": "s", "to": "e", "construction": true}},
        "groups": {"slot": ["q1", "q2", "q3", "q4"]},
        "patterns": {"p": {"of": "slot", "along": {"path": "guide", "count": 4}}}
    }))
    .unwrap();
    let extrude: Op = serde_json::from_value(serde_json::json!({"op": "extrude", "z0": 0, "z1": 2})).unwrap();
    let b = extrude.eval(id("plate"), &[&s]).unwrap();
    let b = b.body().unwrap();
    b.validate().unwrap();
    tessellate(b).unwrap();
    assert_close(b.volume(), 2.0 * (40000.0 - 4.0 * 32.0));
}

/// A path that branches, a copy past its end, a circle for a path, growing steps filling, and a
/// grid with a path are refused.
#[test]
fn bad_paths_are_refused() {
    let refused = |sketch: serde_json::Value, why: &str| match placed(sketch) {
        Err(e) => assert!(e.to_string().contains(why), "{e}"),
        Ok(_) => panic!("expected `{why}`"),
    };
    let along = |a: serde_json::Value| serde_json::json!({"of": "hole", "along": a});
    refused(path_sketch([0.0, 10.0], serde_json::json!(["l1"]), along(serde_json::json!({"path": "edge", "step": 50, "count": 4}))), "past the end");
    let mut branch = path_sketch([0.0, 10.0], serde_json::json!(["l1", "r1", "l2"]), along(serde_json::json!({"path": "edge", "count": 3})));
    branch["points"]["e"] = serde_json::json!([100, -50]);
    branch["lines"]["l2"] = serde_json::json!({"from": "b", "to": "e", "construction": true});
    refused(branch, "branches at `b`");
    let mut circle = path_sketch([0.0, 10.0], serde_json::json!(["l1", "ring"]), along(serde_json::json!({"path": "edge", "count": 3})));
    circle["circles"]["ring"] = serde_json::json!({"center": "c", "diameter": 10, "construction": true});
    refused(circle, "around");
    let mut grid = path_sketch([0.0, 10.0], serde_json::json!(["l1"]), along(serde_json::json!({"path": "edge", "count": 3})));
    grid["patterns"]["p"]["step"] = serde_json::json!([[1, 0], [0, 1]]);
    refused(grid, "one of them");
    refused(plate(serde_json::json!({"of": "cell", "step": [[10, 0], [0, 10]], "fill": "panel", "grow": [[1, 0], [0, 0]]})), "cannot grow");
}
