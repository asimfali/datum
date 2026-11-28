//! A two-sketch louver, including its open slit and return to the wall.
#![allow(clippy::unwrap_used, clippy::panic)]

use datum_brep::{Curve, Surface};
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
    let walls: Vec<_> = body.faces().iter().filter_map(|f| if let Surface::CubicWall(w) = f.surface { Some(w) } else { None }).collect();
    assert!(walls.iter().any(|w| local(w.edges[0].points[0])[1] < 0.0));
    assert!(walls.iter().any(|w| local(w.edges[0].points[0])[1] > 0.0));
    for wall in &walls {
        let normal = |t| Surface::CubicWall(*wall).normal_at(wall.edges[0].at(t).0);
        let change = normal(0.0) - normal(1.0);
        assert!(change.dot(change) > 0.0);
    }
    let mut joins = 0;
    for a in &walls {
        for b in &walls {
            let join = a.edges[0].points[3];
            if join == b.edges[0].points[0] {
                let difference = Surface::CubicWall(*a).normal_at(join) - Surface::CubicWall(*b).normal_at(join);
                assert!(difference.dot(difference) < 1e-20);
                joins += 1;
            }
        }
    }
    assert!(joins > 0);
    let step = datum_brep::step(&[body], "louver", false).unwrap();
    assert_eq!(step.text.matches("B_SPLINE_SURFACE_WITH_KNOTS('',3,1").count(), walls.len());
    let mut patches = 0;
    for (i, f) in body.faces().iter().enumerate() {
        if matches!(f.surface, Surface::Bicubic(_)) {
            let mesh = datum_brep::face_mesh(body, i as datum_brep::Ix, false, 0.01).unwrap();
            assert!(mesh.normals.iter().all(|n| n.to_array().into_iter().all(f64::is_finite)));
            patches += 1;
        }
    }
    assert!(patches > 0);
    assert_eq!(step.text.matches("B_SPLINE_SURFACE_WITH_KNOTS('',3,3").count(), patches);
    let mut seams = 0;
    for edge in body.edges() {
        let h = &body.half_edges()[edge.half as usize];
        let twin = &body.half_edges()[h.twin as usize];
        let surfaces = [h, twin].map(|he| &body.faces()[body.loops()[he.loop_ix as usize].face as usize].surface);
        if let [Surface::Bicubic(a), Surface::Bicubic(b)] = surfaces {
            let Some(Curve::Cubic(c)) = body.edge_curve(edge) else { panic!("a patch seam must have a shared curve") };
            let mid = c.at(0.5).0;
            let normals = [a, b].map(|p| Surface::Bicubic(*p).normal_at(mid));
            assert!(normals[0].dot(normals[1]) > 0.9999, "patch seam at {mid:?}: {normals:?}");
            seams += 1;
        }
    }
    assert!(seams > 0);

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
