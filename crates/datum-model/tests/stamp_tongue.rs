//! The sharp and rounded two-sketch tongues placed on a flat plate.
#![allow(clippy::unwrap_used, clippy::panic)]

use datum_brep::{Curve, Surface};
use datum_graph::Evaluator;
use datum_math::{Vec2, Vec3};
use datum_model::Session;
use datum_ops::FlatPiece;
use serde_json::{Value, json};
use std::path::Path;

const MODEL: &str = include_str!("../../../tests/models/stamp-tongue-straight.json");
const ROUNDED: &str = include_str!("../../../tests/models/stamp-tongue-rounded.json");
const NOSE: &str = include_str!("../../../tests/models/stamp-tongue-nose.json");

fn run(session: &mut Session, command: &str, params: Value) -> Value {
    session.execute(command, &params).unwrap_or_else(|error| panic!("{command}: {error}"))
}

fn model() -> Value {
    serde_json::from_str(MODEL).unwrap()
}

fn add_base(session: &mut Session, model: &Value) -> f64 {
    let features = model["features"].as_array().unwrap();
    for feature in &features[..3] {
        run(session, "add", feature.clone());
    }
    run(session, "mass_properties", json!({"name": "plate"}))["volume"].as_f64().unwrap()
}

fn add_stamp(session: &mut Session, model: &Value) -> Value {
    let stamp = model["features"].as_array().unwrap().iter().find(|f| f["name"] == "stamp").unwrap();
    run(session, "add", stamp.clone())
}

fn feature(session: &mut Session, name: &str) -> Value {
    run(session, "inspect", json!({}))["features"]
        .as_array()
        .unwrap()
        .iter()
        .find(|item| item["name"] == name)
        .unwrap_or_else(|| panic!("no feature {name}"))
        .clone()
}

fn close(a: f64, b: f64) {
    assert!((a - b).abs() <= 1e-9 * a.abs().max(b.abs()).max(1.0), "{a} vs {b}");
}

fn flat_line_ends(session: &mut Session, name: &str) -> Vec<[Vec2; 2]> {
    let flat = session.flat_of(name).unwrap_or_else(|error| panic!("flat {name}: {error}"));
    flat.pieces
        .iter()
        .enumerate()
        .filter(|(i, _)| !flat.outer.contains(i))
        .filter_map(|(_, piece)| match piece {
            FlatPiece::Line(a, b) => Some([*a, *b]),
            _ => None,
        })
        .collect()
}

fn near(a: Vec2, b: Vec2) -> bool {
    (a.x - b.x).abs() <= 1e-9 && (a.y - b.y).abs() <= 1e-9
}

fn cut_x_bounds(session: &mut Session, body: &str) -> (f64, f64) {
    flat_line_ends(session, body)
        .iter()
        .flat_map(|line| line.iter().map(|p| p.x))
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), x| (lo.min(x), hi.max(x)))
}

fn top_height(session: &mut Session, body: &str) -> f64 {
    let faces = run(session, "query_faces", json!({"name": body, "normal": [0, 0, 1]}));
    faces["faces"].as_array().unwrap().iter().filter_map(|face| face["offset"].as_f64()).max_by(f64::total_cmp).unwrap()
}

fn put_sheet_rule(session: &mut Session, name: &str, thickness: f64) {
    run(
        session,
        "put_library",
        json!({"level": "rule", "element": {"name": name, "kind": "sheet", "material": "Steel", "thickness": thickness, "radius": "t"}}),
    );
}

/// The model entry forms one body without changing the nominal amount of sheet material; the
/// blank exposes the three open cut edges, ending at the free end of the window in the wall.
#[test]
fn straight_tongue_forms_one_volume_preserving_body_and_unfolds_its_cut() {
    let model = model();
    let mut session = Session::default();
    let original = add_base(&mut session, &model);
    add_stamp(&mut session, &model);
    let stamp = feature(&mut session, "stamp");
    assert!(stamp.get("error").is_none(), "{stamp}");

    let body = run(&mut session, "mass_properties", json!({"name": "stamp"}));
    assert!(body.get("error").is_none(), "{body}");
    close(original, body["volume"].as_f64().unwrap());

    // A regular unfold feature runs the same flat preview path used by manufacturing exports.
    let blank = model["features"].as_array().unwrap().iter().find(|f| f["name"] == "blank").unwrap();
    run(&mut session, "add", blank.clone());
    let lines = flat_line_ends(&mut session, "blank");
    assert_eq!(lines.len(), 3, "cut should contain the two sides and free end");
    let body_flat = session.flat_of("stamp").unwrap();
    assert_eq!(flat_line_ends(&mut session, "stamp").len(), 3);
    close(body_flat.size.x, session.flat_of("blank").unwrap().size.x);
    close(body_flat.size.y, session.flat_of("blank").unwrap().size.y);

    // Three connected cut segments have two free ends at the root; the cut stays open.
    let mut endpoints: Vec<Vec2> = lines.iter().flat_map(|line| line.iter().copied()).collect();
    endpoints.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
    let mut odd = 0;
    let mut i = 0;
    while i < endpoints.len() {
        let mut j = i + 1;
        while j < endpoints.len() && near(endpoints[j], endpoints[i]) {
            j += 1;
        }
        if (j - i) % 2 == 1 {
            odd += 1;
        }
        i = j;
    }
    assert_eq!(odd, 2, "cut must remain an open chain");

    let xs = lines.iter().flat_map(|line| line.iter().map(|p| p.x));
    let (xmin, xmax) = xs.fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), x| (lo.min(x), hi.max(x)));
    let root_ends = lines.iter().flat_map(|line| line.iter()).filter(|p| (p.x - 42.0).abs() <= 1e-9).count();
    assert_eq!(root_ends, 2, "the two cut sides start at the root station (x={xmin}..{xmax})");
    let free_walls = run(&mut session, "query_faces", json!({"name": "stamp", "normal": [-1, 0, 0]}));
    let far_wall = free_walls["faces"]
        .as_array()
        .unwrap()
        .iter()
        .filter_map(|face| face["offset"].as_f64().map(|x| -x))
        .filter(|x| (42.0..60.0).contains(x))
        .max_by(f64::total_cmp)
        .expect("formed free-end wall");
    close(xmax, far_wall);
}

/// The transition must land on a supported interior wall, not run off the plate edge.
#[test]
fn straight_tongue_rejects_a_transition_on_the_plate_edge() {
    let mut model = model();
    model["features"][2]["sketch"]["points"]["p1"] = json!([0, 30]);
    let mut session = Session::default();
    add_base(&mut session, &model);
    add_stamp(&mut session, &model);
    let stamp = feature(&mut session, "stamp");
    assert!(stamp.get("error").is_some(), "placement on the edge unexpectedly built: {stamp}");
}

/// A tongue that runs across an existing opening is refused instead of bridging unsupported
/// empty space in the sheet.
#[test]
fn straight_tongue_rejects_crossing_a_window() {
    let mut model = model();
    model["features"][0]["sketch"]["points"]["window_center"] = json!([45, 30]);
    model["features"][0]["sketch"]["circles"]["window"] = json!({"center": "window_center", "diameter": 4});
    let mut session = Session::default();
    add_base(&mut session, &model);
    add_stamp(&mut session, &model);
    let stamp = feature(&mut session, "stamp");
    assert!(stamp.get("error").is_some(), "tongue crossing the window unexpectedly built: {stamp}");
}

/// The section length parameter also drives the plan boundary, so changing it keeps both sketches
/// joined at the free end and preserves the amount of sheet material.
#[test]
fn straight_tongue_parameter_updates_both_profiles() {
    let mut model = model();
    model["features"][3]["params"] = json!({"nose_length": 5.25});
    let mut session = Session::default();
    let original = add_base(&mut session, &model);
    add_stamp(&mut session, &model);
    let stamp = feature(&mut session, "stamp");
    assert!(stamp.get("error").is_none(), "{stamp}");
    let formed = run(&mut session, "mass_properties", json!({"name": "stamp"}));
    close(original, formed["volume"].as_f64().unwrap());
}

/// An assigned sheet rule rebuilds both the plate and its tongue. The 1 mm rule reduces both
/// volumes, lengthens the developed window by D291, and leaves the outer section three mm above
/// the wall surface.
#[test]
fn straight_tongue_tracks_a_sheet_rule_thickness_change() {
    let root = Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("stamp_tongue_rule_{}", std::process::id()));
    std::fs::create_dir_all(root.join("part")).unwrap();
    std::fs::write(root.join("datum-project.json"), "{}").unwrap();

    let mut session = Session::default();
    run(&mut session, "save", json!({"path": root.join("part/model.json")}));
    run(&mut session, "put_library", json!({"level": "material", "element": {"name": "Steel", "density": 7850}}));
    put_sheet_rule(&mut session, "Sheet", 2.0);

    let model = model();
    for item in &model["features"].as_array().unwrap()[..3] {
        run(&mut session, "add", item.clone());
    }
    run(&mut session, "set_part", json!({"body": "plate", "rule": "Sheet"}));
    add_stamp(&mut session, &model);

    let plate_2 = run(&mut session, "mass_properties", json!({"name": "plate"}))["volume"].as_f64().unwrap();
    let stamp_2 = run(&mut session, "mass_properties", json!({"name": "stamp"}))["volume"].as_f64().unwrap();
    close(plate_2, stamp_2);
    close(plate_2, 40_000.0);
    let (root_2, far_2) = cut_x_bounds(&mut session, "stamp");
    let window_2 = far_2 - root_2;
    let outside_2 = top_height(&mut session, "stamp") - 2.0;

    put_sheet_rule(&mut session, "Sheet", 1.0);
    let plate_1 = run(&mut session, "mass_properties", json!({"name": "plate"}))["volume"].as_f64().unwrap();
    let stamp_1 = run(&mut session, "mass_properties", json!({"name": "stamp"}))["volume"].as_f64().unwrap();
    close(plate_1, stamp_1);
    close(plate_1, 20_000.0);
    assert!(plate_1 < plate_2, "plate volume did not follow rule: {plate_2} -> {plate_1}");
    assert!(stamp_1 < stamp_2, "formed body volume did not follow rule: {stamp_2} -> {stamp_1}");
    let (root_1, far_1) = cut_x_bounds(&mut session, "stamp");
    let window_1 = far_1 - root_1;
    assert!(window_1 > window_2, "1 mm sheet should have a longer developed window: {window_2} -> {window_1}");
    assert!((root_1 - root_2).abs() > 1e-9 || (far_1 - far_2).abs() > 1e-9, "the developed cut did not move");
    let outside_1 = top_height(&mut session, "stamp") - 1.0;
    close(outside_2, 3.0);
    close(outside_1, outside_2);

    put_sheet_rule(&mut session, "Sheet 0.5", 0.5);
    run(&mut session, "set_part", json!({"body": "plate", "rule": "Sheet 0.5"}));
    let plate_half = run(&mut session, "mass_properties", json!({"name": "plate"}))["volume"].as_f64().unwrap();
    let stamp_half = run(&mut session, "mass_properties", json!({"name": "stamp"}))["volume"].as_f64().unwrap();
    close(plate_half, 10_000.0);
    close(stamp_half, plate_half);
    let (root_half, far_half) = cut_x_bounds(&mut session, "stamp");
    assert!(far_half - root_half > window_1, "the 0.5 mm reference should further lengthen the blank: {window_1} -> {}", far_half - root_half);
    close(top_height(&mut session, "stamp") - 0.5, 3.0);
}

#[test]
fn rounded_tongue_develops_at_the_current_sheet_rule() {
    let root = Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("rounded_tongue_rule_{}", std::process::id()));
    std::fs::create_dir_all(root.join("part")).unwrap();
    std::fs::write(root.join("datum-project.json"), "{}").unwrap();
    let mut session = Session::default();
    run(&mut session, "save", json!({"path": root.join("part/model.json")}));
    run(&mut session, "put_library", json!({"level": "material", "element": {"name": "Steel", "density": 7850}}));

    let model: Value = serde_json::from_str(ROUNDED).unwrap();
    add_base(&mut session, &model);
    add_stamp(&mut session, &model);
    run(&mut session, "add", model["features"][4].clone());
    let baseline = cut_x_bounds(&mut session, "stamp");

    let put = |session: &mut Session, name, t, radius: Value, k| {
        run(
            session,
            "put_library",
            json!({"level": "rule", "element": {
                "name": name, "kind": "sheet", "material": "Steel", "thickness": t, "radius": radius, "k_factor": k
            }}),
        );
    };
    put(&mut session, "Sheet", 1.0, json!("0.4 * t"), 0.5);
    run(&mut session, "set_part", json!({"body": "plate", "rule": "Sheet"}));
    assert_eq!(cut_x_bounds(&mut session, "stamp"), baseline);

    let check_length = |session: &mut Session, t, r, k| {
        let cut = flat_line_ends(session, "stamp");
        assert_eq!(cut.len(), 3);
        assert_eq!(cut, flat_line_ends(session, "blank"));
        let (root, far) = cut_x_bounds(session, "stamp");
        let straight = 13.0 - r - 2.0 * (r + t) - 2.0 * r * libm::tan(7.5_f64.to_radians());
        let arcs = (r + k * t) * 195_f64.to_radians();
        close(far - root, straight + arcs);
        (root, far)
    };
    check_length(&mut session, 1.0, 0.4, 0.5);
    put(&mut session, "Sheet", 1.0, json!(0.6), 0.5);
    let (root_r, far_r) = check_length(&mut session, 1.0, 0.6, 0.5);
    close((baseline.0 - root_r).abs(), 0.2);

    put(&mut session, "Sheet", 1.0, json!(0.6), 0.25);
    let (root_k, far_k) = check_length(&mut session, 1.0, 0.6, 0.25);
    close(root_k, root_r);
    close(far_r - far_k, 0.25 * 195_f64.to_radians());

    put(&mut session, "Thin sheet", 0.5, json!("0.4 * t"), 0.35);
    run(&mut session, "set_part", json!({"body": "plate", "rule": "Thin sheet"}));
    check_length(&mut session, 0.5, 0.2, 0.35);
    close(run(&mut session, "mass_properties", json!({"name": "plate"}))["volume"].as_f64().unwrap(), 10_000.0);
}

#[test]
fn nose_fillet_keeps_its_radius_in_the_tilted_face_and_in_the_blank() {
    let doc = datum_model::parse_doc(NOSE).unwrap();
    let (_, graph) = doc.graphs().into_iter().next().unwrap();
    let graph = graph.unwrap();
    let evaluation = Evaluator::default().evaluate(&graph);
    let stamp = evaluation.results[graph.find("stamp").unwrap() as usize].as_ref().unwrap();
    let body = stamp.body().unwrap();
    let axis = Vec3::new(-libm::sin(15_f64.to_radians()), 0.0, libm::cos(15_f64.to_radians()));
    let tip_radius = |r: f64| (r - 2.0).abs() < 1e-12;
    let nose: Vec<_> = body
        .faces()
        .iter()
        .filter_map(|face| match face.surface {
            Surface::Cylinder(c) if c.convex && tip_radius(c.radius) => Some(c),
            _ => None,
        })
        .collect();
    assert_eq!(nose.len(), 2);
    for cylinder in nose {
        close(cylinder.axis.dot(axis).abs(), 1.0);
    }
    let tip_arcs: Vec<_> = body
        .edges()
        .iter()
        .filter_map(|edge| match body.edge_curve(edge) {
            Some(Curve::Arc(arc)) if tip_radius(arc.radius) && arc.axis.dot(axis).abs() > 0.99 => Some(arc),
            _ => None,
        })
        .collect();
    assert_eq!(tip_arcs.len(), 4);
    let mut tangencies = 0;
    for edge in body.edges().iter().filter(|edge| body.edge_curve(edge).is_none()) {
        let a = body.half_edges()[edge.half as usize];
        let b = body.half_edges()[a.twin as usize];
        let surface = |half: datum_brep::HalfEdge| body.faces()[body.loops()[half.loop_ix as usize].face as usize].surface;
        let (cylinder, plane) = match (surface(a), surface(b)) {
            (Surface::Cylinder(c), Surface::Plane(p)) | (Surface::Plane(p), Surface::Cylinder(c)) if c.convex && tip_radius(c.radius) => (c, p),
            _ => continue,
        };
        let v = body.point(a.origin) - cylinder.origin;
        let radial = v - cylinder.axis * v.dot(cylinder.axis);
        let cosine = radial.dot(plane.normal) / libm::sqrt(radial.dot(radial) * plane.normal.dot(plane.normal));
        close(cosine.abs(), 1.0);
        tangencies += 1;
    }
    assert_eq!(tangencies, 4);
    let blank = evaluation.results[graph.find("blank").unwrap() as usize].as_ref().unwrap();
    let datum_ops::Value::Flat(flat) = &**blank else { panic!("expected blank") };
    nose_cut_end(flat);
}

fn nose_cut_end(flat: &datum_ops::Flat) -> f64 {
    let cuts: Vec<_> = flat.pieces.iter().enumerate().filter(|(i, _)| !flat.outer.contains(i)).map(|(_, piece)| piece).collect();
    let [
        FlatPiece::Line(..),
        FlatPiece::Arc { center, radius: left, .. },
        FlatPiece::Line(..),
        FlatPiece::Arc { radius: right, .. },
        FlatPiece::Line(..),
    ] = cuts.as_slice()
    else {
        panic!("expected a five-piece open cut: {cuts:?}");
    };
    close(*left, 2.0);
    close(*right, 2.0);
    center.x + left
}

#[test]
fn nose_fillet_contour_moves_with_the_sheet_rule_and_keeps_its_radius() {
    let root = Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("nose_fillet_rule_{}", std::process::id()));
    std::fs::create_dir_all(root.join("part")).unwrap();
    std::fs::write(root.join("datum-project.json"), "{}").unwrap();
    let mut session = Session::default();
    run(&mut session, "save", json!({"path": root.join("part/model.json")}));
    run(&mut session, "put_library", json!({"level": "material", "element": {"name": "Steel", "density": 7850}}));
    let model = serde_json::from_str(NOSE).unwrap();
    add_base(&mut session, &model);
    add_stamp(&mut session, &model);
    let original = nose_cut_end(&session.flat_of("stamp").unwrap());
    put_sheet_rule(&mut session, "Thin sheet", 0.75);
    run(&mut session, "set_part", json!({"body": "plate", "rule": "Thin sheet"}));
    let updated = nose_cut_end(&session.flat_of("stamp").unwrap());
    assert!((updated - original).abs() > 1e-9);
}
