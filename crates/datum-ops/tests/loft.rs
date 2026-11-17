//! D264's first loft-flange case: two parallel, coplanar line profiles form a flat trapezoid.
#![allow(clippy::unwrap_used, clippy::panic)]

use datum_brep::{BrepError, TopoId};
use datum_math::expr::Num;
use datum_ops::{Op, Side, Value, unfold_side};

fn id(name: &str) -> TopoId {
    TopoId::derive(name).finish()
}

fn op(value: serde_json::Value) -> Op {
    serde_json::from_value(value).unwrap()
}

fn profile(half_length: f64) -> serde_json::Value {
    serde_json::json!({
        "points": {
            "origin": [0.0, 0.0],
            "a": [-half_length, 0.0],
            "b": [half_length, 0.0]
        },
        "lines": {
            "profile": {"from": "a", "to": "b"}
        },
        "constraints": [
            {"type": "fix", "point": "origin", "at": [0.0, 0.0]},
            {"type": "midpoint", "point": "origin", "line": "profile"},
            {"type": "horizontal", "line": "profile"},
            {"type": "length", "line": "profile", "value": 2.0 * half_length}
        ]
    })
}

fn sketches(second_plane_offset: f64) -> (Value, Value) {
    let first = op(serde_json::json!({"op": "sketch", "sketch": profile(10.0)})).eval(id("loft.first"), &[]).unwrap();

    let plane = op(serde_json::json!({"op": "plane", "base": "xz", "offset": second_plane_offset})).eval(id("loft.second-plane"), &[]).unwrap();
    let second = op(serde_json::json!({"op": "sketch", "sketch": profile(15.0)})).eval(id("loft.second"), &[&plane]).unwrap();

    (first, second)
}

fn body(side: Side, offset: f64) -> Result<datum_brep::Body, BrepError> {
    let (first, second) = sketches(offset);
    let loft = Op::Loft { thickness: Some(Num::Value(2.0)), side, tolerance: Num::Value(1.0) };
    let value = loft.eval(id("loft.body"), &[&first, &second])?;
    Ok(value.body().ok_or_else(|| BrepError::Invalid("loft returned no body".into()))?.clone())
}

/// XY and XZ work planes are perpendicular in this authored input. The solved profiles are
/// parallel in world space and all four ends lie in z=0, so the output is one trapezoid, not
/// two triangles or a pair of ruled faces.
#[test]
fn two_line_profiles_make_one_thick_trapezoid_and_unfold() {
    for side in [Side::Left, Side::Right, Side::Middle] {
        let body = body(side, 15.0).unwrap();
        body.validate().unwrap();
        assert_eq!(body.faces().len(), 6, "side {side:?}");
        assert!((body.volume() - 750.0).abs() < 1e-8, "side {side:?}: volume {}", body.volume());

        let flat = unfold_side(&body, false, 0.5).unwrap();
        assert!((flat.size.x - 30.0).abs() < 1e-8, "side {side:?}: blank width {}", flat.size.x);
        assert!((flat.size.y - 15.0).abs() < 1e-8, "side {side:?}: blank height {}", flat.size.y);
        assert_eq!(flat.outer.end - flat.outer.start, 4, "one four-edge trapezoid outline");
    }
}

/// Coincident profiles have no plane or area to thicken; this is a geometric degeneracy.
#[test]
fn coincident_line_profiles_are_refused_as_degenerate() {
    assert!(matches!(body(Side::Left, 0.0), Err(BrepError::Degenerate(_))));
}

/// A vertical second line cannot produce this first-step trapezoid; do not silently triangulate
/// or approximate the unsupported profile pair.
#[test]
fn non_parallel_line_profiles_are_refused_as_degenerate() {
    let first = op(serde_json::json!({"op": "sketch", "sketch": profile(10.0)})).eval(id("nonparallel.first"), &[]).unwrap();
    let plane = op(serde_json::json!({"op": "plane", "base": "xz", "offset": 15})).eval(id("nonparallel.plane"), &[]).unwrap();
    let second_sketch = serde_json::json!({
        "points": {"origin": [0, 0], "a": [0, -15], "b": [0, 15]},
        "lines": {"profile": {"from": "a", "to": "b"}},
        "constraints": [
            {"type": "fix", "point": "origin", "at": [0, 0]},
            {"type": "midpoint", "point": "origin", "line": "profile"},
            {"type": "vertical", "line": "profile"},
            {"type": "length", "line": "profile", "value": 30}
        ]
    });
    let second = op(serde_json::json!({"op": "sketch", "sketch": second_sketch})).eval(id("nonparallel.second"), &[&plane]).unwrap();
    let loft = Op::Loft { thickness: Some(Num::Value(2.0)), side: Side::Left, tolerance: Num::Value(1.0) };
    assert!(matches!(loft.eval(id("nonparallel.body"), &[&first, &second]), Err(BrepError::Degenerate(_))));
}
