#![allow(clippy::unwrap_used)] // helpers outside #[test] functions

use datum_graph::{Graph, Op};
use datum_math::{Vec2, Vec3};
use datum_model::{ModelError, parse, to_json};

fn sample() -> Graph {
    let mut g = Graph::default();
    // Values chosen to break naive float printing: 0.1 + 0.2, a subnormal-adjacent tiny number.
    let poly = vec![Vec2::new(0.0, 0.0), Vec2::new(0.1 + 0.2, 0.0), Vec2::new(0.0, 1e-300 * 1e299)];
    let p = g.add("plate", Op::Prism { polygon: poly, z0: -1.5, z1: std::f64::consts::PI }, &[]).unwrap();
    g.add("moved", Op::Transform { translate: Vec3::new(1e7, -3.0, 0.0), axis: Vec3::new(0.3, -0.5, 0.8), angle_deg: 37.25 }, &[p]).unwrap();
    g
}

#[test]
fn round_trip_is_exact_and_canonical() {
    let g = sample();
    let text = to_json(&g);
    let back = parse(&text).unwrap();
    assert_eq!(back, g);
    assert_eq!(to_json(&back), text);
}

#[test]
fn defaults_and_input_order() {
    let g = parse(
        r#"{"format": "datum/1", "features": [
        {"name": "a", "op": "prism", "polygon": [[0, 0], [1, 0], [0, 1]], "z0": 0, "z1": 1},
        {"name": "b", "op": "transform", "inputs": ["a"]}
    ]}"#,
    )
    .unwrap();
    assert_eq!(g.nodes()[1].op, Op::Transform { translate: Vec3::ZERO, axis: Vec3::new(0.0, 0.0, 1.0), angle_deg: 0.0 });
}

#[test]
fn mistakes_are_reported_with_the_feature() {
    let case = |features: &str| parse(&format!(r#"{{"format": "datum/1", "features": [{features}]}}"#)).unwrap_err().to_string();
    assert!(case(r#"{"name": "b", "op": "transform", "inputs": ["a"]}"#).contains("unknown input `a`"));
    assert!(case(r#"{"name": "a", "op": "prism", "polygon": [], "z0": 0, "z1": 1, "z2": 5}"#).contains("z2"));
    assert!(case(r#"{"name": "a", "op": "cylinder"}"#).contains("cylinder"));
    assert!(matches!(parse(r#"{"format": "datum/2", "features": []}"#), Err(ModelError::Format(_))));
}
