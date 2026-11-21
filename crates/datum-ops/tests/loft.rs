//! Planar and two-bend loft-flange cases (D264–D267).
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
            "mid": [0.0, 0.0],
            "a": [-half_length, 0.0],
            "b": [half_length, 0.0]
        },
        "lines": {
            "profile": {"from": "a", "to": "b"}
        },
        "constraints": [
            {"type": "fix", "point": "mid", "at": [0.0, 0.0]},
            {"type": "midpoint", "point": "mid", "line": "profile"},
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
    let loft = Op::Loft { thickness: Some(Num::Value(2.0)), radius: None, k_factor: None, side, tolerance: Num::Value(1.0) };
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
        "points": {"mid": [0, 0], "a": [0, -15], "b": [0, 15]},
        "lines": {"profile": {"from": "a", "to": "b"}},
        "constraints": [
            {"type": "fix", "point": "mid", "at": [0, 0]},
            {"type": "midpoint", "point": "mid", "line": "profile"},
            {"type": "vertical", "line": "profile"},
            {"type": "length", "line": "profile", "value": 30}
        ]
    });
    let second = op(serde_json::json!({"op": "sketch", "sketch": second_sketch})).eval(id("nonparallel.second"), &[&plane]).unwrap();
    let loft = Op::Loft { thickness: Some(Num::Value(2.0)), radius: None, k_factor: None, side: Side::Left, tolerance: Num::Value(1.0) };
    assert!(matches!(loft.eval(id("nonparallel.body"), &[&first, &second]), Err(BrepError::Degenerate(_))));
}

fn bent_sketches(apex_x: f64, apex_height: f64) -> (Value, Value) {
    let first = op(serde_json::json!({"op": "sketch", "sketch": profile(10.0)})).eval(id("bent-loft.first"), &[]).unwrap();
    let plane = op(serde_json::json!({"op": "plane", "base": "xz", "offset": 15})).eval(id("bent-loft.plane"), &[]).unwrap();
    let second_sketch = serde_json::json!({
        "points": {
            "left": [-15.0, 0.0],
            "peak": [apex_x, apex_height],
            "right": [15.0, 0.0]
        },
        "lines": {
            "left_segment": {"from": "left", "to": "peak"},
            "right_segment": {"from": "peak", "to": "right"}
        },
        "constraints": [
            {"type": "fix", "point": "left", "at": [-15.0, 0.0]},
            {"type": "fix", "point": "peak", "at": [apex_x, apex_height]},
            {"type": "fix", "point": "right", "at": [15.0, 0.0]}
        ]
    });
    let second = op(serde_json::json!({"op": "sketch", "sketch": second_sketch})).eval(id("bent-loft.second"), &[&plane]).unwrap();
    (first, second)
}

fn bent_op(side: Side, radius: f64) -> Op {
    Op::Loft {
        thickness: Some(Num::Value(2.0)),
        radius: Some(Num::Value(radius)),
        k_factor: Some(Num::Value(0.44)),
        side,
        tolerance: Num::Value(1.0),
    }
}

fn reference_sketches(half_a: f64, leg: f64) -> (Value, Value) {
    let half_b = leg * libm::cos(20.0 * std::f64::consts::PI / 180.0);
    let height = leg * libm::sin(20.0 * std::f64::consts::PI / 180.0);
    let first = op(serde_json::json!({"op": "sketch", "sketch": profile(half_a)})).eval(id("reference.first"), &[]).unwrap();
    let plane = op(serde_json::json!({"op": "plane", "base": "xz", "offset": 300.0})).eval(id("reference.plane"), &[]).unwrap();
    let second_sketch = serde_json::json!({
        "points": {"left": [-half_b, 0.0], "peak": [0.0, height], "right": [half_b, 0.0]},
        "lines": {
            "left_segment": {"from": "left", "to": "peak"},
            "right_segment": {"from": "peak", "to": "right"}
        },
        "constraints": [
            {"type": "fix", "point": "left", "at": [-half_b, 0.0]},
            {"type": "fix", "point": "peak", "at": [0.0, height]},
            {"type": "fix", "point": "right", "at": [half_b, 0.0]}
        ]
    });
    let second = op(serde_json::json!({"op": "sketch", "sketch": second_sketch})).eval(id("reference.second"), &[&plane]).unwrap();
    (first, second)
}

/// D266's reference profile has two 200 mm legs at 20 degrees, 300 mm from A.
#[test]
fn one_line_to_two_lines_has_three_facets_and_two_valid_bends() {
    let (first, second) = reference_sketches(50.0, 200.0);
    for side in [Side::Left, Side::Right, Side::Middle] {
        let value = bent_op(side, 2.0).eval(id("reference.body"), &[&first, &second]).unwrap();
        let body = value.body().unwrap();
        body.validate().unwrap();
        let cylinders: Vec<_> = body
            .faces()
            .iter()
            .filter_map(|f| match f.surface {
                datum_brep::Surface::Cylinder(c) => Some(c),
                _ => None,
            })
            .collect();
        assert_eq!(cylinders.len(), 4, "side {side:?}");
        for (f, face) in body.faces().iter().enumerate() {
            let datum_brep::Surface::Cylinder(cylinder) = face.surface else { continue };
            let mut tangents = 0;
            for h in body.face_loops(f as datum_brep::Ix).flat_map(|l| body.loop_half_edges(l)) {
                let edge = body.half_edges()[h as usize];
                let other_loop = body.half_edges()[edge.twin as usize].loop_ix;
                let other = body.faces()[body.loops()[other_loop as usize].face as usize].surface;
                if let datum_brep::Surface::Plane(plane) = other
                    && plane.normal.dot(cylinder.axis).abs() < 1e-10
                {
                    assert!(face.surface.normal_at(body.point(edge.origin)).dot(plane.normal) > 1.0 - 1e-10);
                    tangents += 1;
                }
            }
            assert_eq!(tangents, 2, "side {side:?}");
        }
        for inner in cylinders.iter().filter(|c| (c.radius - 2.0).abs() < 1e-9) {
            let matches: Vec<_> = cylinders
                .iter()
                .filter(|outer| {
                    (outer.radius - 4.0).abs() < 1e-9
                        && inner.axis.cross(outer.axis).dot(inner.axis.cross(outer.axis)) < 1e-18
                        && (outer.origin - inner.origin).cross(inner.axis).dot((outer.origin - inner.origin).cross(inner.axis)) < 1e-18
                })
                .collect();
            assert_eq!(matches.len(), 1, "side {side:?}");
            assert!(!inner.convex && matches[0].convex, "side {side:?}");
        }
        assert_eq!(cylinders.iter().filter(|c| (c.radius - 2.0).abs() < 1e-9).count(), 2);
        let flat = unfold_side(body, false, 0.44).unwrap();
        assert!(!flat.outer.is_empty(), "side {side:?}");
        assert!(flat.overlaps.is_empty(), "side {side:?}: {:?}", flat.overlaps);
        assert_eq!(flat.bends.len(), 4, "side {side:?}");
        assert!(flat.bends.iter().all(|line| line[0] != line[1]), "side {side:?}");
        let ids: std::collections::BTreeSet<_> = flat.bend_ids.iter().copied().collect();
        assert_eq!(ids.len(), 4, "side {side:?}");
        let half_b = 200.0 * libm::cos(20.0 * std::f64::consts::PI / 180.0);
        let height = 200.0 * libm::sin(20.0 * std::f64::consts::PI / 180.0);
        let axis = datum_math::Vec3::new(-50.0, 300.0, height);
        let leg = datum_math::Vec3::new(half_b, 0.0, -height);
        let middle_normal = axis.cross(datum_math::Vec3::new(100.0, 0.0, 0.0));
        let wing_normal = axis.cross(leg);
        let cosine = middle_normal.dot(wing_normal).abs() / (middle_normal.dot(middle_normal) * wing_normal.dot(wing_normal)).sqrt();
        let width = (2.0 + 0.44 * 2.0) * libm::acos(cosine);
        for line in &flat.bends {
            let d = datum_math::Vec2::new(line[1].x - line[0].x, line[1].y - line[0].y);
            let length = (d.x * d.x + d.y * d.y).sqrt();
            let partners = flat
                .bends
                .iter()
                .filter(|other| {
                    let e = datum_math::Vec2::new(other[1].x - other[0].x, other[1].y - other[0].y);
                    let offset = datum_math::Vec2::new(other[0].x - line[0].x, other[0].y - line[0].y);
                    (d.x * e.y - d.y * e.x).abs() < 1e-6 && ((d.x * offset.y - d.y * offset.x).abs() / length - width).abs() < 1e-7
                })
                .count();
            assert_eq!(partners, 1, "side {side:?}: width {width}, bends {:?}", flat.bends);
        }
    }
}

proptest::proptest! {
    #![proptest_config(proptest::prelude::ProptestConfig::with_cases(16))]

    #[test]
    fn bent_loft_keeps_a_single_sheet_as_dimensions_change(
        half_a in 40.0..60.0,
        leg in 180.0..220.0,
        thickness in 1.0..3.0,
        radius in 1.0..4.0,
        k in 0.3..0.6,
    ) {
        let (first, second) = reference_sketches(half_a, leg);
        let loft = Op::Loft {
            thickness: Some(Num::Value(thickness)),
            radius: Some(Num::Value(radius)),
            k_factor: Some(Num::Value(k)),
            side: Side::Middle,
            tolerance: Num::Value(1.0),
        };
        let value = loft.eval(id("reference.body"), &[&first, &second]).unwrap();
        let body = value.body().unwrap();
        body.validate().unwrap();
        for back in [false, true] {
            let flat = unfold_side(body, back, k).unwrap();
            proptest::prop_assert_eq!(flat.polylines(), 0);
            proptest::prop_assert_eq!(flat.outer.len(), flat.pieces.len());
            proptest::prop_assert_eq!(flat.bends.len(), 4);
            proptest::prop_assert!(flat.overlaps.is_empty());
        }
    }
}

#[test]
fn mounting_edge_and_blank_references_survive_profile_and_rule_edits() {
    let build = |half_a, leg, radius, k| {
        let (first, second) = reference_sketches(half_a, leg);
        let loft = Op::Loft {
            thickness: Some(Num::Value(2.0)),
            radius: Some(Num::Value(radius)),
            k_factor: Some(Num::Value(k)),
            side: Side::Left,
            tolerance: Num::Value(1.0),
        };
        loft.eval(id("reference.body"), &[&first, &second]).unwrap().body().unwrap().clone()
    };
    let before = build(50.0, 200.0, 2.0, 0.44);
    let after = build(52.0, 205.0, 2.5, 0.40);
    let flat_before = unfold_side(&before, false, 0.44).unwrap();
    let flat_after = unfold_side(&after, false, 0.40).unwrap();
    assert_eq!(flat_before.bend_ids.iter().copied().collect::<std::collections::BTreeSet<_>>(), flat_after.bend_ids.iter().copied().collect());
    assert_eq!(flat_before.ids.iter().copied().collect::<std::collections::BTreeSet<_>>(), flat_after.ids.iter().copied().collect());
    let edges = datum_ops::flange_edges(&before);
    let angle = 20.0 * std::f64::consts::PI / 180.0;
    let selected: Vec<_> = edges
        .iter()
        .filter(|(_, ends)| {
            let d = ends[1] - ends[0];
            let length = d.dot(d).sqrt();
            length > 10.0
                && d.y.abs() < 1e-7
                && (d.x.abs() / length - libm::cos(angle)).abs() < 1e-7
                && (ends[0].y.abs() - 300.0).abs() < 1e-7
                && (ends[1].y.abs() - 300.0).abs() < 1e-7
                && ends[0].x.min(ends[1].x) < 0.0
        })
        .map(|(edge, _)| *edge)
        .collect();
    assert_eq!(selected.len(), 1, "mounting edge: {selected:?}, candidates: {edges:?}");
    assert!(datum_ops::flange_edges(&after).iter().any(|(edge, _)| *edge == selected[0]));
    for body in [before, after] {
        let target = Value::Body(body);
        let flange = op(
            serde_json::json!({"op": "flange", "edges": [format!("{:?}", selected[0])], "height": 10.0, "height_along": true, "angle": 60.0, "radius": 2.0}),
        );
        flange.eval(id("reference.flange"), &[&target]).unwrap().body().unwrap().validate().unwrap();
    }
}

/// A vertex offset along the first line is not above its midpoint and must be refused.
#[test]
fn two_line_loft_requires_the_vertex_over_the_first_line_midpoint() {
    let (first, second) = bent_sketches(3.0, 10.0);
    let error = bent_op(Side::Left, 8.0).eval(id("bent-loft.off-centre"), &[&first, &second]).unwrap_err();
    assert!(matches!(error, BrepError::Degenerate(_)), "{error}");
}

/// A radius that consumes either triangular panel is a geometric degeneracy, not a reason
/// to relax the checks or approximate a sharp bend.
#[test]
fn two_line_loft_refuses_a_bend_that_consumes_a_facet() {
    let (first, second) = bent_sketches(0.0, 10.0);
    let error = bent_op(Side::Left, 100.0).eval(id("bent-loft.large-radius"), &[&first, &second]).unwrap_err();
    assert!(matches!(error, BrepError::Degenerate(_)), "{error}");
}

/// An implementation probe for D264 step 2: the middle triangle can take two neighbouring
/// flanges at once. This verifies the shared-corner path in the existing flange B-rep builder.
#[test]
fn a_triangular_plate_can_take_two_adjacent_cylindrical_flanges() {
    let triangle = op(serde_json::json!({
        "op": "sketch",
        "sketch": {
            "points": {"a": [-10.0, 0.0], "b": [10.0, 0.0], "v": [0.0, 15.0]},
            "lines": {
                "ab": {"from": "a", "to": "b"},
                "bv": {"from": "b", "to": "v"},
                "va": {"from": "v", "to": "a"}
            }
        }
    }))
    .eval(id("two-flange.profiles"), &[])
    .unwrap();
    let plate =
        op(serde_json::json!({"op": "extrude", "z0": -1.0, "z1": 1.0})).eval(id("two-flange.plate"), &[&triangle]).unwrap().body().unwrap().clone();
    plate.validate().unwrap();

    let close_xy = |p: datum_math::Vec3, q: [f64; 2]| (p.x - q[0]).abs() < 1e-9 && (p.y - q[1]).abs() < 1e-9;
    let edge_names = |a: [f64; 2], b: [f64; 2]| -> Vec<String> {
        datum_ops::flange_edges(&plate)
            .into_iter()
            .filter(|(_, ends)| (close_xy(ends[0], a) && close_xy(ends[1], b)) || (close_xy(ends[0], b) && close_xy(ends[1], a)))
            .map(|(edge, _)| format!("{edge:?}"))
            .collect()
    };
    let mut edges = edge_names([-10.0, 0.0], [0.0, 15.0]);
    edges.extend(edge_names([10.0, 0.0], [0.0, 15.0]));
    assert_eq!(edges.len(), 4, "two hinge edges, inner and outer: {edges:?}");

    let target = Value::Body(plate);
    let bent = op(serde_json::json!({
        "op": "flange",
        "edges": edges,
        "height": 10.0,
        "height_along": true,
        "angle": 60.0,
        "radius": 2.0
    }))
    .eval(id("two-flange.bent"), &[&target])
    .unwrap()
    .body()
    .unwrap()
    .clone();

    bent.validate().unwrap();
    assert_eq!(bent.faces().iter().filter(|face| matches!(face.surface, datum_brep::Surface::Cylinder(_))).count(), 4);
    let flat = unfold_side(&bent, false, 0.44).unwrap();
    // `Flat::bends` holds a line per seam of the unfold tree, both tangent lines of each bend
    // (as the flange tests count them: a new flange adds two). Two adjacent flanges: four.
    assert_eq!(flat.bends.len(), 4);
    let ids: std::collections::BTreeSet<TopoId> = flat.bend_ids.iter().copied().collect();
    assert_eq!(ids.len(), flat.bend_ids.len());
}
