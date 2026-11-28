//! Axisymmetric two-sketch stamps and their developed blanks.
#![allow(clippy::unwrap_used, clippy::panic)]

use datum_brep::Surface;
use datum_model::Session;
use datum_ops::FlatPiece;
use serde_json::{Value, json};

const SPHERE: &str = include_str!("../../../tests/models/stamp-dimple-sphere.json");
const CONE: &str = include_str!("../../../tests/models/stamp-dimple-cone.json");
const PIERCED: &str = include_str!("../../../tests/models/stamp-dimple-pierced.json");
const TORUS: &str = include_str!("../../../tests/models/stamp-torus.json");

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
fn dimple_bends_use_the_sheet_rule_on_both_skins() {
    for (source, bends) in [(SPHERE, 1), (CONE, 2), (PIERCED, 2)] {
        let mut model: Value = serde_json::from_str(source).unwrap();
        for radius in [0.5, 0.75] {
            model["features"][3]["radius"] = json!(radius);
            let session = build(&model);
            let value = session.value("stamp").unwrap();
            let body = value.body().unwrap();
            for expected in [radius, radius + 0.5] {
                let sectors = body
                    .faces()
                    .iter()
                    .filter(|face| matches!(face.surface, Surface::Torus(t) if t.major > 0.0 && (t.minor - expected).abs() < 1e-8));
                assert_eq!(sectors.count(), 3 * bends);
            }
        }
    }
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

#[test]
fn toroidal_transition_cuts_the_blank_at_the_neutral_arc_length() {
    let mut model: Value = serde_json::from_str(TORUS).unwrap();
    let section = &model["features"][3]["part"]["features"][0];
    let mut sketch_feature = section["op"].clone();
    sketch_feature["name"] = section["name"].clone();
    let mut sketch_session = Session::default();
    let reply = sketch_session.execute("add", &sketch_feature).unwrap();
    assert_eq!(reply["failed"], json!([]));
    let sketch = sketch_session.value("sketch1").unwrap().sketch().unwrap();
    let arc = &sketch.arcs["a1"];
    let points = &sketch.solved.points;
    let center = points[&arc.center];
    let offset = |name: &str| (points[name].x - center.x, points[name].y - center.y);
    let ((sx, sy), (ex, ey)) = (offset(&arc.from), offset(&arc.to));
    let angle = libm::atan2(sx * ey - sy * ex, sx * ex + sy * ey);
    let neutral_radius = libm::hypot(sx, sy);
    let expected = 100.0 - neutral_radius * angle;

    model["features"][3]["k_factor"] = json!(0.3);
    model["features"][4]["k_factor"] = json!(0.3);
    let mut flat = build(&model);
    let body = flat.value("stamp").unwrap().body().unwrap();
    body.validate().unwrap();
    datum_brep::tessellate(body).unwrap();
    for minor in [neutral_radius - 0.15, neutral_radius + 0.35] {
        let sectors = body
            .faces()
            .iter()
            .filter(|face| matches!(face.surface, Surface::Torus(torus) if (torus.major - 100.0).abs() < 1e-8 && (torus.minor - minor).abs() < 1e-8));
        assert_eq!(sectors.count(), 3, "expected three sectors with minor radius {minor}");
    }
    let hash = body.hash();
    let holes = blank_holes(&mut flat);
    let [FlatPiece::Circle { radius, .. }] = holes.as_slice() else { panic!("expected one circular hole, got {holes:?}") };
    assert!((radius - expected).abs() < 1e-8, "expected radius {expected}, got {radius}");

    model["features"][3]["part"]["features"][0]["metadata"]["stamp.cut"] = json!("tool");
    let mut tool = build(&model);
    assert_eq!(tool.value("stamp").unwrap().body().unwrap().hash(), hash);
    assert!(blank_holes(&mut tool).is_empty());
}

#[test]
fn a_shared_sheet_retains_the_cuts_from_earlier_stamps() {
    let model: Value = serde_json::from_str(include_str!("../../../tests/models/stamps.json")).unwrap();
    let mut session = build(&model);
    let flat = session.flat_of("blank").unwrap();
    let cuts = flat.pieces.iter().enumerate().filter_map(|(i, piece)| (!flat.outer.contains(&i)).then_some(piece)).collect::<Vec<_>>();
    assert_eq!(cuts.iter().filter(|p| matches!(p, FlatPiece::Circle { .. })).count(), 2);
    for name in ["torus", "dimple_pierced", "emboss_window", "slot_rounded", "louver"] {
        let before = session.flat_of(name).unwrap();
        for (i, piece) in before.pieces.iter().enumerate() {
            if !before.outer.contains(&i) {
                assert!(cuts.contains(&piece), "{name}: {piece:?}");
            }
        }
    }
    let final_body = session.value("tongue_nose").unwrap().body().unwrap();
    final_body.validate().unwrap();
    datum_brep::tessellate(final_body).unwrap();
}
