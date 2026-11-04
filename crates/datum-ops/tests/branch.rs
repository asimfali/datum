//! A branch pipe set on a main one (D227): the hole in the main shell and the branch's end.
#![allow(clippy::unwrap_used, clippy::panic)]

use datum_brep::{Body, BrepError, TopoId, tessellate};
use datum_ops::{Op, Value};
use serde_json::{Value as Json, json};

const T: f64 = 2.0;
const K: f64 = 0.44;

fn id(tag: &str) -> TopoId {
    TopoId::derive(tag).finish()
}

fn op(json: Json) -> Op {
    serde_json::from_value(json).unwrap()
}

fn rolled(plane: &str, p: [f64; 2], q: [f64; 2], tag: &str) -> Body {
    let sketch = json!({
        "points": {"a0": [0, 0], "a1": [0, 10], "b0": [-1000, 0], "b1": [1000, 0], "p": p, "q": q},
        "lines": {"ax": {"from": if plane == "XY" { "b0" } else { "a0" }, "to": if plane == "XY" { "b1" } else { "a1" }, "construction": true}, "path": {"from": "p", "to": "q"}}
    });
    let frame = if plane == "XY" {
        json!({"origin": [0, 0, 0], "x_axis": [1, 0, 0], "normal": [0, 0, 1]})
    } else {
        json!({"origin": [0, 0, 0], "x_axis": [0, 1, 0], "normal": [1, 0, 0]})
    };
    let s = op(json!({"op": "sketch", "sketch": sketch, "plane": frame})).eval(id(&format!("{tag}.sketch")), &[]).unwrap();
    let roll = json!({"op": "rolled", "axis": "ax", "side": "left", "gap": 2.0, "thickness": T, "k_factor": K});
    op(roll).eval(id(tag), &[&s]).unwrap().body().unwrap().clone()
}

/// The main pipe along X, 50 from the axis, from -100 to 100; the branch along Z, 20 from it.
fn tee() -> (Body, Body) {
    (rolled("XY", [-100.0, 50.0], [100.0, 50.0], "main"), rolled("YZ", [20.0, 0.0], [20.0, 120.0], "branch"))
}

fn run(kind: &str, a: &Body, b: &Body) -> Result<Body, BrepError> {
    let r = op(json!({"op": kind, "k_factor": K})).eval(id(kind), &[&Value::Body(a.clone()), &Value::Body(b.clone())])?.body().unwrap().clone();
    r.validate().unwrap();
    tessellate(&r).unwrap();
    Ok(r)
}

#[test]
fn tee_builds() {
    let (main, branch) = tee();
    let holed = run("branch_hole", &main, &branch).unwrap();
    let cut = run("saddle", &branch, &main).unwrap();
    let (m0, m1) = (main.mass_properties().volume, holed.mass_properties().volume);
    let (b0, b1) = (branch.mass_properties().volume, cut.mass_properties().volume);
    eprintln!("main {m0} -> {m1}, branch {b0} -> {b1}");
    assert!(m1 < m0 && b1 < b0);
}

#[test]
fn tee_blanks() {
    let (main, branch) = tee();
    for (name, b) in [("hole", run("branch_hole", &main, &branch).unwrap()), ("saddle", run("saddle", &branch, &main).unwrap())] {
        let f = datum_ops::unfold_side(&b, false, K).unwrap();
        let poly: Vec<usize> = f.pieces.iter().filter_map(|p| if let datum_ops::FlatPiece::Polyline(v) = p { Some(v.len()) } else { None }).collect();
        eprintln!("{name}: {} pieces, size {:?}, polylines {poly:?}", f.pieces.len(), f.size);
        assert!(datum_brep::step(&[&b], "x", false).is_err());
        datum_brep::step(&[&b], "x", true).unwrap();
    }
}
