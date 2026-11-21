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

fn arc_profiles(radius: f64, dx: f64) -> (Value, Value) {
    arc_profiles_at_angle(radius, dx, std::f64::consts::FRAC_PI_4)
}

fn arc_profiles_at_angle(radius: f64, dx: f64, angle: f64) -> (Value, Value) {
    let arc = |r: f64, x: f64, a: f64| {
        serde_json::json!({
            "points": {"center": [x, 0.0], "start": [x + r, 0.0],
                "end": [x + r * libm::cos(a), r * libm::sin(a)]},
            "arcs": {"arc": {"center": "center", "from": "start", "to": "end"}}
        })
    };
    let first = op(serde_json::json!({"op": "sketch", "sketch": arc(50.0, 0.0, std::f64::consts::FRAC_PI_4)})).eval(id("arc.first"), &[]).unwrap();
    let plane = op(serde_json::json!({"op": "plane", "base": "xy", "offset": 50.0})).eval(id("arc.plane"), &[]).unwrap();
    let second = op(serde_json::json!({"op": "sketch", "sketch": arc(radius, dx, angle)})).eval(id("arc.second"), &[&plane]).unwrap();
    (first, second)
}

fn arc_loft(tolerance: f64) -> Op {
    Op::Loft {
        thickness: Some(Num::Value(2.0)),
        radius: Some(Num::Value(2.0)),
        k_factor: Some(Num::Value(0.44)),
        side: Side::Left,
        tolerance: Num::Value(tolerance),
    }
}

/// Two congruent arcs define a faceted sheet; each internal chord boundary is a bend.
#[test]
fn equal_arc_profiles_make_chords_and_bends_at_the_requested_tolerance() {
    let (first, second) = arc_profiles(50.0, 0.0);
    for (tolerance, chords) in [(4.0, 1), (1.0, 2), (0.25, 4)] {
        let value = arc_loft(tolerance).eval(id("arc.body"), &[&first, &second]).unwrap();
        let body = value.body().unwrap();
        body.validate().unwrap();
        let cylinders = body.faces().iter().filter(|face| matches!(face.surface, datum_brep::Surface::Cylinder(_))).count();
        assert_eq!(cylinders, 2 * (chords - 1), "tolerance {tolerance}");
        let planes = body.faces().iter().filter(|face| matches!(face.surface, datum_brep::Surface::Plane(_))).count();
        assert_eq!(planes, 2 * chords + 4, "tolerance {tolerance}");
        for back in [false, true] {
            let flat = unfold_side(body, back, 0.44).unwrap();
            assert_eq!(flat.outer.len(), flat.pieces.len());
            assert!(!flat.outer.is_empty());
            assert!(flat.overlaps.is_empty(), "tolerance {tolerance}, back {back}: {:?}", flat.overlaps);
            assert_eq!(flat.bends.len(), 2 * (chords - 1));
            assert!(flat.bends.iter().all(|line| line[0] != line[1]));
            assert_eq!(flat.bend_ids.iter().copied().collect::<std::collections::BTreeSet<_>>().len(), flat.bends.len());
        }
        let sagitta = 50.0 * (1.0 - libm::cos(std::f64::consts::FRAC_PI_4 / (2.0 * chords as f64)));
        assert!(sagitta <= tolerance);
        if chords > 1 {
            let previous = 50.0 * (1.0 - libm::cos(std::f64::consts::FRAC_PI_4 / (2.0 * (chords - 1) as f64)));
            assert!(previous > tolerance);
        }
    }
}

#[test]
fn shifted_or_different_sweep_arc_profiles_are_refused() {
    for (dx, angle) in [(1.0, std::f64::consts::FRAC_PI_4), (0.0, std::f64::consts::FRAC_PI_4 * 0.8)] {
        let (first, second) = arc_profiles_at_angle(50.0, dx, angle);
        let error = arc_loft(1.0).eval(id("arc.invalid"), &[&first, &second]).unwrap_err();
        assert!(matches!(error, BrepError::Unsupported(_)), "offset {dx}, angle {angle}: {error}");
    }
}

#[test]
fn concentric_arc_profiles_with_different_radii_make_a_tapered_sheet() {
    for (radius_b, tolerance, chords) in [(40.0, 4.0, 1), (40.0, 1.0, 2), (40.0, 0.25, 4), (60.0, 1.0, 3)] {
        let (first, second) = arc_profiles(radius_b, 0.0);
        for side in [Side::Left, Side::Right, Side::Middle] {
            let mut loft = arc_loft(tolerance);
            if let Op::Loft { side: loft_side, .. } = &mut loft {
                *loft_side = side;
            }
            let value = loft.eval(id("arc.taper"), &[&first, &second]).unwrap();
            let body = value.body().unwrap();
            body.validate().unwrap();
            assert!(body.volume().is_finite() && body.volume() > 0.0);
            assert_eq!(body.faces().iter().filter(|f| matches!(f.surface, datum_brep::Surface::Cylinder(_))).count(), 2 * (chords - 1));
            assert_eq!(body.faces().iter().filter(|f| matches!(f.surface, datum_brep::Surface::Plane(_))).count(), 6 * chords);
            for back in [false, true] {
                let flat = unfold_side(body, back, 0.44).unwrap();
                assert!(!flat.outer.is_empty());
                assert_eq!(flat.outer.len(), flat.pieces.len());
                assert!(flat.overlaps.is_empty(), "side {side:?}, tolerance {tolerance}, back {back}: {:?}", flat.overlaps);
                assert_eq!(flat.bends.len(), 2 * (chords - 1));
                assert!(flat.bends.iter().all(|line| line[0] != line[1]));
            }
        }
    }
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

fn three_segment_sketches(half_a: f64, middle: f64, leg: f64, reverse: bool) -> (Value, Value) {
    let half_b = middle / 2.0;
    let wing = leg * libm::cos(std::f64::consts::FRAC_PI_4);
    let first = op(serde_json::json!({"op": "sketch", "sketch": profile(half_a)})).eval(id("three.first"), &[]).unwrap();
    let plane = op(serde_json::json!({"op": "plane", "base": "xz", "offset": 300.0})).eval(id("three.plane"), &[]).unwrap();
    let (l0, l1, m0, m1, r0, r1) = if reverse {
        ("joint_left", "left", "joint_right", "joint_left_copy", "right", "joint_right")
    } else {
        ("left", "joint_left", "joint_left_copy", "joint_right", "joint_right", "right")
    };
    let sketch = serde_json::json!({
        "points": {
            "left": [-half_b - wing, wing], "joint_left": [-half_b, 0.0],
            "joint_left_copy": [-half_b, 0.0], "joint_right": [half_b, 0.0], "right": [half_b + wing, wing]
        },
        "lines": {
            "wing_left": {"from": l0, "to": l1},
            "middle": {"from": m0, "to": m1},
            "wing_right": {"from": r0, "to": r1}
        },
        "constraints": [
            {"type": "fix", "point": "left", "at": [-half_b - wing, wing]},
            {"type": "fix", "point": "joint_left", "at": [-half_b, 0.0]},
            {"type": "coincident", "points": ["joint_left", "joint_left_copy"]},
            {"type": "fix", "point": "joint_right", "at": [half_b, 0.0]},
            {"type": "fix", "point": "right", "at": [half_b + wing, wing]}
        ]
    });
    let second = op(serde_json::json!({"op": "sketch", "sketch": sketch})).eval(id("three.second"), &[&plane]).unwrap();
    (first, second)
}

/// D269's three-segment B profile has an explicitly coincident endpoint at one joint.
#[test]
fn one_line_to_three_lines_unfolds_and_keeps_mounting_references() {
    let build = |half_a, middle, leg, radius, k, side, reverse| {
        let (first, second) = three_segment_sketches(half_a, middle, leg, reverse);
        let loft = Op::Loft {
            thickness: Some(Num::Value(2.0)),
            radius: Some(Num::Value(radius)),
            k_factor: Some(Num::Value(k)),
            side,
            tolerance: Num::Value(1.0),
        };
        loft.eval(id("three.body"), &[&first, &second]).unwrap().body().unwrap().clone()
    };
    for side in [Side::Left, Side::Right, Side::Middle] {
        let body = build(50.0, 200.0, 100.0, 2.0, 0.44, side, false);
        body.validate().unwrap();
        assert_eq!(body.faces().iter().filter(|face| matches!(face.surface, datum_brep::Surface::Cylinder(_))).count(), 4, "side {side:?}");
        for back in [false, true] {
            let flat = unfold_side(&body, back, 0.44).unwrap();
            assert_eq!(flat.outer.len(), flat.pieces.len(), "side {side:?}, back {back}");
            assert!(!flat.outer.is_empty());
            assert!(flat.overlaps.is_empty(), "side {side:?}, back {back}: {:?}", flat.overlaps);
            assert_eq!(flat.bends.len(), 4);
            assert!(flat.bends.iter().all(|line| line[0] != line[1]));
            assert_eq!(flat.bend_ids.iter().copied().collect::<std::collections::BTreeSet<_>>().len(), 4);
        }
    }
    let before = build(50.0, 200.0, 100.0, 2.0, 0.44, Side::Left, false);
    let after = build(52.0, 205.0, 105.0, 2.5, 0.40, Side::Left, false);
    for back in [false, true] {
        let flat_before = unfold_side(&before, back, 0.44).unwrap();
        let flat_after = unfold_side(&after, back, 0.40).unwrap();
        assert_eq!(flat_before.ids.iter().copied().collect::<std::collections::BTreeSet<_>>(), flat_after.ids.iter().copied().collect());
        assert_eq!(flat_before.bend_ids.iter().copied().collect::<std::collections::BTreeSet<_>>(), flat_after.bend_ids.iter().copied().collect());
    }
    let edges = datum_ops::flange_edges(&before);
    let selected: Vec<_> = edges
        .iter()
        .filter(|(_, ends)| {
            let d = ends[1] - ends[0];
            d.x.abs() > 100.0 && d.y.abs() < 1e-7 && d.z.abs() < 1e-7 && (ends[0].y.abs() - 300.0).abs() < 1e-7 && ends[0].z.abs() < 1e-7
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
        flange.eval(id("three.flange"), &[&target]).unwrap().body().unwrap().validate().unwrap();
    }
    let reversed = build(50.0, 200.0, 100.0, 2.0, 0.44, Side::Left, true);
    reversed.validate().unwrap();
    let flat = unfold_side(&reversed, false, 0.44).unwrap();
    assert_eq!(flat.bends.len(), 4);
    assert!(flat.overlaps.is_empty());
    let parallel_bends = build(50.0, 100.0, 100.0, 2.0, 0.44, Side::Left, false);
    parallel_bends.validate().unwrap();
    assert_eq!(unfold_side(&parallel_bends, false, 0.44).unwrap().bends.len(), 4);
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
        middle in 95.0..220.0,
        leg in 180.0..220.0,
        thickness in 1.0..3.0,
        radius in 1.0..4.0,
        k in 0.3..0.6,
    ) {
        let loft = Op::Loft {
            thickness: Some(Num::Value(thickness)),
            radius: Some(Num::Value(radius)),
            k_factor: Some(Num::Value(k)),
            side: Side::Middle,
            tolerance: Num::Value(1.0),
        };
        for (first, second) in [reference_sketches(half_a, leg), three_segment_sketches(half_a, middle, leg * 0.5, false)] {
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
