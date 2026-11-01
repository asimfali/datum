//! DXF of a trimmed rolled sheet (D211): the wave is a polyline, written only by consent and
//! told of in a note beside the file; a blank without one takes an old note away.
#![allow(clippy::unwrap_used)]

use datum_brep::TopoId;
use datum_math::Vec3;
use datum_ops::{Flat, Op, SketchPlane, Value, unfold_side};
use serde_json::json;

fn id(tag: &str) -> TopoId {
    TopoId::derive(tag).finish()
}

fn op(j: serde_json::Value) -> Op {
    serde_json::from_value(j).unwrap()
}

/// The blank of a shell, 100 long at radius 50, trimmed by a plane leaning 20 degrees (or not
/// trimmed: `None`).
fn blank(trim: Option<f64>) -> Flat {
    let sk = json!({
        "points": {"a0": [0, 0], "a1": [10, 0], "p": [0, 50], "q": [100, 50]},
        "lines": {"ax": {"from": "a0", "to": "a1", "construction": true}, "path": {"from": "p", "to": "q"}}
    });
    let s = op(json!({"op": "sketch", "sketch": sk})).eval(id("profile"), &[]).unwrap();
    let roll = json!({"op": "rolled", "axis": "ax", "side": "left", "gap": 2, "thickness": 2, "k_factor": 0.44});
    let mut body = op(roll).eval(id("roll"), &[&s]).unwrap();
    if let Some(deg) = trim {
        let (s, c) = (libm::sin(deg.to_radians()), libm::cos(deg.to_radians()));
        let plane = SketchPlane { origin: Vec3::new(70.0, 0.0, 0.0), x_axis: Vec3::new(-s, c, 0.0), normal: Vec3::new(c, s, 0.0) };
        body = op(json!({"op": "trim", "k_factor": 0.44})).eval(id("trim"), &[&body, &Value::Plane(plane)]).unwrap();
    }
    unfold_side(body.body().unwrap(), false, 0.44).unwrap()
}

#[test]
fn a_polyline_dxf_needs_consent_and_leaves_a_note() {
    let dir = std::env::temp_dir().join(format!("datum-trim-dxf-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("trimmed.dxf");
    let note = dir.join("trimmed.dxf.txt");
    let trimmed = blank(Some(20.0));
    assert_eq!(trimmed.polylines(), 1);
    let dxf = trimmed.dxf();
    assert_eq!(dxf.matches("\nLWPOLYLINE\n").count(), 1);
    assert!(dxf.contains("$ACADVER") && dxf.contains("AC1015"));
    // Refused without consent, and nothing is written.
    let err = datum_model::write_dxf(&trimmed, &path, false).unwrap_err();
    assert!(err.contains("polyline"), "{err}");
    assert!(!path.exists() && !note.exists());
    // Agreed to: the file, and the note.
    assert_eq!(datum_model::write_dxf(&trimmed, &path, true), Ok(1));
    assert_eq!(std::fs::read_to_string(&path).unwrap(), dxf);
    let told = std::fs::read_to_string(&note).unwrap();
    assert!(told.starts_with("Datum DXF note"), "{told}");
    // A blank with none over the same path takes the note away.
    let plain = blank(None);
    assert_eq!(plain.polylines(), 0);
    assert_eq!(datum_model::write_dxf(&plain, &path, false), Ok(0));
    assert!(path.exists() && !note.exists());
    assert!(!std::fs::read_to_string(&path).unwrap().contains("LWPOLYLINE"));
    std::fs::remove_dir_all(&dir).unwrap();
}
