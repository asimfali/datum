//! Axisymmetric two-sketch stamps and their developed blanks.
#![allow(clippy::unwrap_used, clippy::panic)]

use datum_brep::Surface;
use datum_model::Session;
use datum_ops::FlatPiece;
use serde_json::{Value, json};

const SPHERE: &str = include_str!("../../../tests/models/stamp-dimple-sphere.json");
const CONE: &str = include_str!("../../../tests/models/stamp-dimple-cone.json");
const PIERCED: &str = include_str!("../../../tests/models/stamp-dimple-pierced.json");

fn build(model: &Value) -> Session {
    let mut session = Session::default();
    for feature in model["features"].as_array().unwrap() {
        let reply = session.execute("add", feature).unwrap();
        assert_eq!(reply["failed"], json!([]), "{feature}: {reply}");
    }
    assert!(session.value("blank").is_ok());
    session
}

fn blank_holes(session: &mut Session) -> Vec<FlatPiece> {
    let flat = session.flat_of("stamp").unwrap();
    flat.pieces.iter().cloned().enumerate().filter_map(|(i, piece)| (!flat.outer.contains(&i)).then_some(piece)).collect()
}

#[test]
fn spherical_and_conical_dimples_preserve_a_solid_blank() {
    let models = [(SPHERE, true), (CONE, false)];
    for (source, spherical) in models {
        let mut session = build(&serde_json::from_str(source).unwrap());
        let value = session.value("stamp").unwrap();
        assert!(value.has_stamp());
        let body = value.body().unwrap();
        body.validate().unwrap();
        datum_brep::tessellate(body).unwrap();
        assert!(body.faces().iter().any(|face| match face.surface {
            Surface::Torus(torus) => spherical && torus.major == 0.0,
            Surface::Cone(_) => !spherical,
            _ => false,
        }));
        assert!(blank_holes(&mut session).is_empty());
    }
}

#[test]
fn pierced_dimple_cuts_the_blank_only_in_flat_mode() {
    let mut model: Value = serde_json::from_str(PIERCED).unwrap();
    let mut flat = build(&model);
    let value = flat.value("stamp").unwrap();
    let body = value.body().unwrap();
    body.validate().unwrap();
    datum_brep::tessellate(body).unwrap();
    assert!(
        body.faces()
            .iter()
            .any(|face| { matches!(face.surface, Surface::Cylinder(cylinder) if !cylinder.convex && (cylinder.radius - 4.0).abs() < 1e-8) })
    );
    let hash = body.hash();
    let holes = blank_holes(&mut flat);
    let [FlatPiece::Circle { radius, .. }] = holes.as_slice() else { panic!("expected one circular hole, got {holes:?}") };
    assert!((radius - 4.0).abs() < 1e-8);

    model["features"][3]["part"]["features"][0]["metadata"]["stamp.cut"] = json!("tool");
    let mut tool = build(&model);
    assert_eq!(tool.value("stamp").unwrap().body().unwrap().hash(), hash);
    assert!(blank_holes(&mut tool).is_empty());
}
