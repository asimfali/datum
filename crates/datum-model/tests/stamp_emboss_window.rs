//! The closed window of an embossed shelf must carry through to the flat blank.
#![allow(clippy::unwrap_used, clippy::panic)]

use datum_brep::Surface;
use datum_math::Vec2;
use datum_model::Session;
use datum_ops::FlatPiece;
use serde_json::{Value, json};

const MODEL: &str = include_str!("../../../tests/models/stamp-emboss-window.json");

#[test]
fn emboss_window_is_open_in_both_skins_and_closed_on_the_blank() {
    let model: Value = serde_json::from_str(MODEL).unwrap();
    check_window(&model, 0);
}

#[test]
fn narrow_shelf_preserves_the_window_corner_arcs() {
    let mut model: Value = serde_json::from_str(MODEL).unwrap();
    let constraints = model["features"][3]["part"]["features"][0]["op"]["sketch"]["constraints"].as_array_mut().unwrap();
    let width = constraints.iter_mut().find(|c| c["type"] == "length" && c["line"] == "l3").unwrap();
    width["value"] = json!(0.6);
    check_window(&model, 5);
}

fn check_window(model: &Value, arcs: usize) {
    let mut session = Session::default();
    for feature in model["features"].as_array().unwrap() {
        let reply = session.execute("add", feature).unwrap();
        assert_eq!(reply["failed"], json!([]), "{feature}: {reply}");
    }

    let formed = session.value("stamp").unwrap();
    assert!(formed.has_stamp());
    let body = formed.body().unwrap();
    body.validate().unwrap();
    datum_brep::tessellate(body).unwrap();
    // The two original skins have holes where the walls of the emboss rise. The window adds a
    // hole to each raised shelf face as well.
    let perforated_skins: Vec<_> = body
        .faces()
        .iter()
        .filter_map(|face| match face.surface {
            Surface::Plane(plane) if face.loop_count == 2 => Some(plane.origin.z),
            _ => None,
        })
        .collect();
    assert_eq!(perforated_skins.len(), 4);
    assert_eq!(perforated_skins.iter().filter(|&&z| z > 1.0).count(), 2);

    let blank = session.flat_of("blank").unwrap();
    let opening: Vec<_> = blank.pieces.iter().enumerate().filter(|(i, _)| !blank.outer.contains(i)).map(|(_, piece)| piece).collect();
    let lines = opening.iter().filter(|piece| matches!(piece, FlatPiece::Line(..))).count();
    assert_eq!(lines, 5);
    let edges = opening
        .iter()
        .map(|piece| match piece {
            FlatPiece::Line(from, to) => [*from, *to],
            FlatPiece::Arc { center, radius, start, end } => [start, end].map(|angle| {
                let (s, c) = libm::sincos(*angle);
                Vec2::new(center.x + radius * c, center.y + radius * s)
            }),
            _ => panic!("unexpected piece in the window: {piece:?}"),
        })
        .collect::<Vec<_>>();
    // Every edge joins the next one, including after the last. Arc endpoints can have the
    // opposite traversal order because DXF uses counterclockwise arcs.
    let close = |a: Vec2, b: Vec2| libm::hypot(a.x - b.x, a.y - b.y) < 1e-8;
    for (i, edge) in edges.iter().enumerate() {
        let next = edges[(i + 1) % edges.len()];
        assert!(edge.iter().any(|&a| next.iter().any(|&b| close(a, b))), "disconnected edges: {edge:?}, {next:?}");
    }
    assert_eq!(blank.hash, session.flat_of("stamp").unwrap().hash);

    let dxf = blank.dxf();
    let entities: Vec<_> = dxf.split_once("ENTITIES\n").unwrap().1.split_once("\nENDSEC\n").unwrap().0.lines().collect();
    let count = |kind| entities.as_chunks::<2>().0.iter().filter(|pair| pair[0].trim() == "0" && pair[1] == kind).count();
    assert_eq!(count("LINE"), blank.outer.len() + lines);
    assert_eq!(edges.len(), lines + arcs);
    assert_eq!(count("ARC"), arcs);
}
