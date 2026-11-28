//! A two-sketch louver, including its open slit and return to the wall.
#![allow(clippy::unwrap_used, clippy::panic)]

use datum_math::{Sign, Vec2, Vec3, orient2d};
use datum_model::Session;
use datum_ops::FlatPiece;
use serde_json::{Value, json};

#[test]
fn louver_opens_a_slit_and_returns_to_the_wall_at_both_ends() {
    let model: Value = serde_json::from_str(include_str!("../../../tests/models/stamp-louver.json")).unwrap();
    let mut session = Session::default();
    for feature in model["features"].as_array().unwrap() {
        let reply = session.execute("add", feature).unwrap();
        assert_eq!(reply["failed"], json!([]), "{feature}: {reply}");
    }

    let places = session.value("places").unwrap().sketch().unwrap();
    let [origin, x, y, z] = places.frame;
    let placement = places.solved.points["p1"];
    let origin = origin + x * placement.x + y * placement.y;
    let value = session.value("stamp").unwrap();
    assert!(value.has_stamp());
    let body = value.body().unwrap();
    body.validate().unwrap();
    let mesh = datum_brep::tessellate(body).unwrap();

    let local = |p: Vec3| {
        let q = p - origin;
        [q.dot(x), q.dot(y), q.dot(z)]
    };
    let close = |a: f64, b: f64| (a - b).abs() < 1e-7;
    let slit_x = 10.171761896163648;
    let at_midspan = body.vertices().iter().map(|v| local(v.point)).any(|[u, v, h]| close(u, slit_x) && close(v, 0.0) && close(h, -2.0));
    assert!(at_midspan, "the slit should keep the full section height in the middle");
    for end in [-25.0, 25.0] {
        let at_wall = body.vertices().iter().map(|v| local(v.point)).any(|[u, v, h]| close(u, slit_x) && close(v, end) && close(h, 0.0));
        assert!(at_wall, "the slit should return to the wall at station {end}");
    }
    let between_lips = Vec2::new(slit_x + 0.1, 0.0);
    assert!(
        !mesh.triangles.iter().any(|tri| {
            let [a, b, c] = tri.map(|i| {
                let [u, v, _] = local(mesh.positions[i as usize]);
                Vec2::new(u, v)
            });
            let winding = orient2d(a, b, c);
            winding != Sign::Zero
                && [(a, b), (b, c), (c, a)].into_iter().all(|(p, q)| {
                    let side = orient2d(p, q, between_lips);
                    side == Sign::Zero || side == winding
                })
        }),
        "the slit should be open through the sheet"
    );

    let flat = session.flat_of("stamp").unwrap();
    assert_eq!(flat.pieces, session.flat_of("blank").unwrap().pieces);
    let [flat_origin, fx, fy, _] = flat.frame.unwrap();
    let project = |p: Vec3| {
        let q = p - flat_origin;
        Vec2::new(q.dot(fx), q.dot(fy))
    };
    let expected = [-25.0, 25.0].map(|end| project(origin + x * slit_x + y * end));
    let inside = flat.pieces.iter().enumerate().filter_map(|(i, p)| (!flat.outer.contains(&i)).then_some(p)).collect::<Vec<_>>();
    let [FlatPiece::Line(a, b)] = inside.as_slice() else { panic!("expected one straight cut, got {inside:?}") };
    let same = |a: Vec2, b: Vec2| close(a.x, b.x) && close(a.y, b.y);
    assert!((same(*a, expected[0]) && same(*b, expected[1])) || (same(*b, expected[0]) && same(*a, expected[1])));
}
