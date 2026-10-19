//! N-ary Boolean operation (M2b): volumes, identities, invariance, degenerate input.
#![allow(clippy::unwrap_used, clippy::panic)]

use datum_brep::{Body, BrepError, TopoId, cuboid, prism, tessellate};
use datum_math::{Rigid, Vec2, Vec3};
use datum_ops::{BooleanKind, boolean};
use proptest::prelude::*;

use BooleanKind::{Difference, Intersection, Union};

fn id(tag: &str) -> TopoId {
    TopoId::derive(tag).finish()
}

fn feature() -> TopoId {
    id("test.boolean")
}

fn run(kind: BooleanKind, bodies: &[&Body]) -> Result<Body, BrepError> {
    let b = boolean(feature(), kind, None, bodies)?;
    b.validate().unwrap();
    // Every face, holes included, triangulates, and the mesh encloses the same volume.
    let m = tessellate(&b).unwrap();
    let o = m.positions[0];
    let mesh_volume: f64 = m
        .triangles
        .iter()
        .map(|t| (m.positions[t[0] as usize] - o).dot((m.positions[t[1] as usize] - o).cross(m.positions[t[2] as usize] - o)))
        .sum::<f64>()
        / 6.0;
    assert_close(mesh_volume, b.volume(), 1e-9);
    Ok(b)
}

fn assert_close(a: f64, b: f64, rel: f64) {
    assert!((a - b).abs() <= rel * b.abs().max(1.0), "{a} vs {b}");
}

fn plate() -> Body {
    cuboid(id("plate"), Vec3::ZERO, Vec3::new(100.0, 50.0, 10.0)).unwrap()
}

/// A 10 × 10 square tool through the plate, turned about Z by `angle` around (`x`, `y`).
fn square_tool(tag: &str, x: f64, y: f64, angle: f64) -> Body {
    let b = cuboid(id(tag), Vec3::new(-5.0, -5.0, -5.0), Vec3::new(5.0, 5.0, 15.0)).unwrap();
    b.transformed(&Rigid::rotation(Vec3::new(0.0, 0.0, 1.0), angle).then(Rigid::translation(Vec3::new(x, y, 0.0))))
}

#[test]
fn hole_through_a_plate() {
    let p = plate();
    let top = p.faces().iter().find(|f| f.plane.normal.z > 0.0).unwrap().id;
    let r = run(Difference, &[&p, &square_tool("t", 30.0, 25.0, 0.37)]).unwrap();
    assert_close(r.volume(), 50_000.0 - 1_000.0, 1e-12);
    assert_eq!((r.faces().len(), r.loops().len()), (10, 12));
    // The drilled top face keeps its identity: references to it survive the hole (§4.2).
    let f = r.faces().iter().position(|f| f.id == top).unwrap();
    assert_eq!(r.face_loops(f as u32).len(), 2);
}

#[test]
fn many_tools_in_one_pass() {
    let p = plate();
    let tools: Vec<Body> = (0..4).map(|k| square_tool(&format!("t{k}"), 15.0 + 22.0 * k as f64, 20.0 + k as f64, 0.1 + 0.3 * k as f64)).collect();
    let mut bodies = vec![&p];
    bodies.extend(&tools);
    let r = run(Difference, &bodies).unwrap();
    assert_close(r.volume(), 50_000.0 - 4_000.0, 1e-12);
    assert_eq!(r.faces().len(), 6 + 4 * 4);
}

#[test]
fn union_and_intersection_of_overlapping_boxes() {
    let a = cuboid(id("a"), Vec3::ZERO, Vec3::new(2.0, 2.0, 2.0)).unwrap();
    let b =
        cuboid(id("b"), Vec3::new(1.0, 1.0, 1.0), Vec3::new(3.0, 3.0, 3.0)).unwrap().transformed(&Rigid::rotation(Vec3::new(1.0, 2.0, 3.0), 1e-3));
    let u = run(Union, &[&a, &b]).unwrap().volume();
    let i = run(Intersection, &[&a, &b]).unwrap().volume();
    assert_close(u + i, 16.0, 1e-12);
    assert_close(i, 1.0, 1e-2);
}

#[test]
fn tool_inside_leaves_a_cavity() {
    let p = plate();
    let inner = cuboid(id("inner"), Vec3::new(10.0, 10.0, 2.0), Vec3::new(20.0, 20.0, 8.0)).unwrap();
    let r = run(Difference, &[&p, &inner]).unwrap();
    assert_close(r.volume(), 50_000.0 - 600.0, 1e-12);
    assert_eq!(r.faces().len(), 12);
    let only = run(Intersection, &[&p, &inner]).unwrap();
    assert_close(only.volume(), 600.0, 1e-12);
}

#[test]
fn union_of_disjoint_bodies_has_two_shells() {
    let a = cuboid(id("a"), Vec3::ZERO, Vec3::new(1.0, 1.0, 1.0)).unwrap();
    let b = cuboid(id("b"), Vec3::new(5.0, 0.0, 0.0), Vec3::new(6.0, 2.0, 1.0)).unwrap();
    assert_close(run(Union, &[&a, &b]).unwrap().volume(), 3.0, 1e-12);
    assert!(matches!(boolean(feature(), Intersection, None, &[&a, &b]), Err(BrepError::Degenerate(_))));
}

/// Tool edges exactly on the diagonals of the internal triangulation are not a degeneracy of the
/// input: the triangulation is the kernel's own (D38).
#[test]
fn edges_on_triangulation_diagonals() {
    let p = cuboid(id("square"), Vec3::ZERO, Vec3::new(100.0, 100.0, 10.0)).unwrap();
    let on_diagonals = cuboid(id("t"), Vec3::new(40.0, 40.0, -5.0), Vec3::new(50.0, 50.0, 15.0)).unwrap();
    assert_close(run(Difference, &[&p, &on_diagonals]).unwrap().volume(), 100_000.0 - 1_000.0, 1e-12);
    let diamond = prism(id("d"), &[(35.0, 30.0), (30.0, 35.0), (25.0, 30.0), (30.0, 25.0)].map(|(x, y)| Vec2::new(x, y)), -5.0, 15.0).unwrap();
    assert_close(run(Difference, &[&p, &diamond, &on_diagonals]).unwrap().volume(), 100_000.0 - 1_000.0 - 500.0, 1e-12);
}

/// Tools must not touch each other (D40). Contacts with the target are resolved (Q7, see
/// `contacts.rs`); between tools the growth makes them overlap, which is refused too.
#[test]
fn touching_tools_are_refused() {
    let p = plate();
    let refused = |bodies: &[&Body], why: &str| match boolean(feature(), Difference, None, bodies) {
        Err(BrepError::Degenerate(m)) => assert!(m.contains(why), "{m}"),
        r => panic!("expected `{why}`, got {r:?}"),
    };
    refused(&[&p, &square_tool("a", 30.0, 25.0, 0.2), &square_tool("b", 36.0, 25.0, 0.5)], "tools 1 and 2 intersect");
    // A tool nested in another, touching neither its surface nor the plate's.
    let big = cuboid(id("big"), Vec3::new(10.0, 10.0, -5.0), Vec3::new(40.0, 40.0, 15.0)).unwrap();
    let small = cuboid(id("small"), Vec3::new(20.0, 20.0, 1.0), Vec3::new(25.0, 25.0, 5.0)).unwrap();
    refused(&[&p, &big, &small], "tools 1 and 2 intersect");
    // Two cut-outs sharing a wall.
    let left = cuboid(id("l"), Vec3::new(10.0, 10.0, -5.0), Vec3::new(20.0, 20.0, 15.0)).unwrap();
    let right = cuboid(id("r"), Vec3::new(20.0, 10.0, -5.0), Vec3::new(30.0, 20.0, 15.0)).unwrap();
    refused(&[&p, &left, &right], "tools 1 and 2 intersect");
}

/// Principle 8: the same model far from the origin and turned gives the same topology and volume.
#[test]
fn invariant_to_placement() {
    let (p, t, u) = (plate(), square_tool("t", 30.0, 25.0, 0.37), square_tool("u", 70.0, 30.0, 1.1));
    let topology = |b: &Body| {
        let mut f: Vec<_> = b.faces().iter().map(|f| f.id).collect();
        f.sort();
        let mut v: Vec<_> = b.vertices().iter().map(|v| v.id).collect();
        v.sort();
        (f, v, b.edges().len(), b.loops().len())
    };
    let reference = run(Difference, &[&p, &t, &u]).unwrap();
    for shift in [1e3, 1e6, 1e7] {
        let m = Rigid::rotation(Vec3::new(0.3, -0.5, 0.8), 0.731).then(Rigid::translation(Vec3::new(shift, -0.7 * shift, 0.3 * shift)));
        let r = run(Difference, &[&p.transformed(&m), &t.transformed(&m), &u.transformed(&m)]).unwrap();
        assert_eq!(topology(&r), topology(&reference), "shift {shift}");
        assert_close(r.volume(), reference.volume(), 1e-6);
    }
}

fn random_box(tag: &str, size: [f64; 3], axis: [f64; 3], angle: f64, at: [f64; 3]) -> Body {
    let half = Vec3::from(size) * 0.5;
    cuboid(id(tag), -half, half).unwrap().transformed(&Rigid::rotation(axis.into(), angle).then(Rigid::translation(at.into())))
}

/// Volume of a Boolean result; an empty one has none.
fn volume(kind: BooleanKind, bodies: &[&Body]) -> f64 {
    match run(kind, bodies) {
        Ok(r) => r.volume(),
        Err(BrepError::Degenerate(m)) if m == "the result is empty" => 0.0,
        Err(e) => panic!("{e}"),
    }
}

/// An L-shaped (non-convex) prism, turned and moved.
fn l_prism(axis: [f64; 3], angle: f64) -> Body {
    let l = [(-1.0, -1.0), (1.2, -1.0), (1.2, 0.1), (0.1, 0.1), (0.1, 1.3), (-1.0, 1.3)].map(|(x, y)| Vec2::new(x, y));
    prism(id("l"), &l, -0.9, 0.8).unwrap().transformed(&Rigid::rotation(axis.into(), angle))
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(48))]

    /// §9: V(A∪B) + V(A∩B) = V(A) + V(B) and V(A−B) = V(A) − V(A∩B) for boxes in general position.
    #[test]
    fn volume_identities(
        sa in prop::array::uniform3(1.0..3.0f64), sb in prop::array::uniform3(0.5..2.5f64),
        axa in prop::array::uniform3(0.1..1.0f64), axb in prop::array::uniform3(-1.0..1.0f64),
        aa in 0.1..3.0f64, ab in 0.1..3.0f64, at in prop::array::uniform3(-0.8..0.8f64),
    ) {
        prop_assume!(axb.iter().any(|c| c.abs() > 0.1));
        let a = random_box("a", sa, axa, aa, [0.0; 3]);
        let b = random_box("b", sb, axb, ab, at);
        let (va, vb) = (a.volume(), b.volume());
        // The boxes may miss each other, or B may swallow A.
        let [u, i, d] = [Union, Intersection, Difference].map(|k| volume(k, &[&a, &b]));
        let tol = 1e-9 * (va + vb);
        prop_assert!((u + i - va - vb).abs() <= tol, "union {u} + intersection {i} vs {va} + {vb}");
        prop_assert!((d - (va - i)).abs() <= tol, "difference {d} vs {va} - {i}");
    }

    /// The same identities for a non-convex target, and again on a Boolean result: its faces have
    /// holes, split edges and, after rounding, are not exactly planar.
    #[test]
    fn identities_on_boolean_results(
        axl in prop::array::uniform3(0.1..1.0f64), al in 0.1..3.0f64,
        sb in prop::array::uniform3(0.3..1.2f64), axb in prop::array::uniform3(0.1..1.0f64), ab in 0.1..3.0f64, at in prop::array::uniform3(-0.6..0.6f64),
        sc in prop::array::uniform3(0.3..1.2f64), axc in prop::array::uniform3(0.1..1.0f64), ac in 0.1..3.0f64, ct in prop::array::uniform3(-0.6..0.6f64),
    ) {
        let a = l_prism(axl, al);
        let b = random_box("b", sb, axb, ab, at);
        let c = random_box("c", sc, axc, ac, ct);
        let tol = 1e-9 * a.volume();
        for target in [a.clone(), run(Difference, &[&a, &b]).unwrap(), run(Union, &[&a, &b]).unwrap()] {
            let (vt, vc) = (target.volume(), c.volume());
            let [u, i, d] = [Union, Intersection, Difference].map(|k| volume(k, &[&target, &c]));
            prop_assert!((u + i - vt - vc).abs() <= tol, "union {u} + intersection {i} vs {vt} + {vc}");
            prop_assert!((d - (vt - i)).abs() <= tol, "difference {d} vs {vt} - {i}");
        }
    }
}
