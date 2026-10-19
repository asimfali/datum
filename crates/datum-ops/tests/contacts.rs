//! Contacts with the target (Q7): a tool counts as infinitesimally larger, so touching means
//! overlapping, and what has zero size in the result is removed exactly.
#![allow(clippy::unwrap_used, clippy::panic)]

use datum_brep::{Body, BrepError, TopoId, cuboid, prism, tessellate};
use datum_math::Vec2;
use datum_ops::{BooleanKind, boolean};

use BooleanKind::{Difference, Intersection, Union};

fn id(tag: &str) -> TopoId {
    TopoId::derive(tag).finish()
}

fn boxed(tag: &str, min: [f64; 3], max: [f64; 3]) -> Body {
    cuboid(id(tag), min.into(), max.into()).unwrap()
}

/// The result, checked: valid, one face per expected count, tessellated with its own volume.
fn run(kind: BooleanKind, bodies: &[&Body]) -> Body {
    let b = boolean(id("op"), kind, None, bodies).unwrap();
    b.validate().unwrap();
    let m = tessellate(&b).unwrap();
    let o = m.positions[0];
    let v: f64 = m
        .triangles
        .iter()
        .map(|t| (m.positions[t[0] as usize] - o).dot((m.positions[t[1] as usize] - o).cross(m.positions[t[2] as usize] - o)))
        .sum::<f64>()
        / 6.0;
    assert_close(v, b.volume());
    b
}

fn assert_close(a: f64, b: f64) {
    assert!((a - b).abs() <= 1e-12 * b.abs().max(1.0), "{a} vs {b}");
}

fn plate() -> Body {
    boxed("plate", [0.0, 0.0, 0.0], [100.0, 50.0, 10.0])
}

/// No vertex of the result may coincide with another: zero-size elements are gone.
fn no_coincident_vertices(b: &Body) {
    let mut pts: Vec<[u64; 3]> = b.vertices().iter().map(|v| v.point.to_array().map(f64::to_bits)).collect();
    pts.sort_unstable();
    assert!(pts.windows(2).all(|w| w[0] != w[1]), "coincident vertices");
}

/// A pin exactly as thick as the plate: the cut goes through, no `through` needed.
#[test]
fn flush_pin_cuts_through() {
    let r = run(Difference, &[&plate(), &boxed("pin", [10.0, 10.0, 0.0], [20.0, 20.0, 10.0])]);
    assert_close(r.volume(), 50_000.0 - 1_000.0);
    assert_eq!((r.faces().len(), r.loops().len(), r.vertices().len()), (10, 12, 16));
    no_coincident_vertices(&r);
}

/// A pocket flush with the top is open; its corners are the tool's own vertices.
#[test]
fn flush_pocket_is_open() {
    let r = run(Difference, &[&plate(), &boxed("pocket", [10.0, 10.0, 4.0], [20.0, 20.0, 10.0])]);
    assert_close(r.volume(), 50_000.0 - 600.0);
    assert_eq!(r.faces().len(), 6 + 5);
    no_coincident_vertices(&r);
}

/// A notch whose wall is flush with the plate's side.
#[test]
fn flush_notch() {
    let r = run(Difference, &[&plate(), &boxed("notch", [80.0, 20.0, -5.0], [100.0, 30.0, 15.0])]);
    assert_close(r.volume(), 50_000.0 - 2_000.0);
    no_coincident_vertices(&r);
}

/// Boxes stacked face to face unite into one body.
#[test]
fn stacked_boxes_unite() {
    let a = boxed("a", [0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
    let same = boxed("b", [0.0, 0.0, 2.0], [2.0, 2.0, 4.0]);
    let r = run(Union, &[&a, &same]);
    assert_close(r.volume(), 16.0);
    assert_eq!(r.shell_vertices().len(), 1);
    no_coincident_vertices(&r);
    let smaller = boxed("c", [0.5, 0.5, 2.0], [1.5, 1.5, 3.0]);
    let r = run(Union, &[&a, &smaller]);
    assert_close(r.volume(), 9.0);
    assert_eq!(r.shell_vertices().len(), 1);
    // Intersection of boxes sharing faces is the common box.
    let shifted = boxed("d", [1.0, 0.0, 0.0], [3.0, 2.0, 2.0]);
    assert_close(run(Intersection, &[&a, &shifted]).volume(), 4.0);
}

/// An edge of the tool through the plate's vertical edge (formerly "edges of two bodies cross").
#[test]
fn edge_through_an_edge() {
    let across = prism(id("across"), &[(-5.0, 5.0), (5.0, -5.0), (5.0, 5.0)].map(|(x, y)| Vec2::new(x, y)), 2.0, 8.0).unwrap();
    let r = run(Difference, &[&plate(), &across]);
    assert_close(r.volume(), 50_000.0 - 25.0 * 6.0);
    no_coincident_vertices(&r);
}

/// The answer does not depend on where the coincidence sits: the same cut moved off the grid
/// and back gives the same volume.
#[test]
fn contact_and_general_position_agree() {
    for (x, refused) in [(10.0, false), (10.25, false)] {
        let r = boolean(id("op"), Difference, None, &[&plate(), &boxed("pin", [x, 10.0, 0.0], [x + 10.0, 20.0, 10.0])]);
        assert_eq!(r.is_err(), refused);
        assert_close(r.unwrap().volume(), 49_000.0);
    }
    let _ = BrepError::Degenerate(String::new());
}

/// Volume of a result; empty when everything cancels (boxes that only touch).
fn volume(kind: BooleanKind, a: &Body, b: &Body) -> Result<f64, BrepError> {
    match boolean(id("op"), kind, None, &[a, b]) {
        Ok(r) => {
            r.validate()?;
            tessellate(&r)?;
            Ok(r.volume())
        }
        Err(BrepError::Degenerate(m)) if m == "the result is empty" => Ok(0.0),
        Err(e) => Err(e),
    }
}

proptest::proptest! {
    #![proptest_config(proptest::prelude::ProptestConfig::with_cases(400))]

    /// Boxes on an integer grid: shared faces, edges and corners in nearly every case. Every
    /// operation gives the exact volume.
    #[test]
    fn boxes_on_a_grid(
        alo in proptest::array::uniform3(0..4i32), asize in proptest::array::uniform3(1..4i32),
        blo in proptest::array::uniform3(0..4i32), bsize in proptest::array::uniform3(1..4i32),
    ) {
        let (ahi, bhi) = ([0, 1, 2].map(|i| alo[i] + asize[i]), [0, 1, 2].map(|i| blo[i] + bsize[i]));
        let f = |v: [i32; 3]| v.map(f64::from);
        let (ba, bb) = (boxed("a", f(alo), f(ahi)), boxed("b", f(blo), f(bhi)));
        let common: f64 = (0..3).map(|i| f64::from((ahi[i].min(bhi[i]) - alo[i].max(blo[i])).max(0))).product();
        let (va, vb) = (ba.volume(), bb.volume());
        proptest::prop_assert_eq!(volume(Intersection, &ba, &bb), Ok(common), "intersection {:?} {:?}", (alo, ahi), (blo, bhi));
        // Boxes meeting only along an edge or at a corner unite into a non-manifold solid: that
        // is refused, not approximated.
        let overlap = |i: usize| ahi[i].min(bhi[i]) - alo[i].max(blo[i]);
        let pinched = (0..3).all(|i| overlap(i) >= 0) && (0..3).filter(|&i| overlap(i) > 0).count() < 2;
        let union = volume(Union, &ba, &bb);
        if pinched {
            proptest::prop_assert!(matches!(union, Err(BrepError::NonManifold(_))), "union {:?} {:?}: {:?}", (alo, ahi), (blo, bhi), union);
        } else {
            proptest::prop_assert_eq!(union, Ok(va + vb - common), "union {:?} {:?}", (alo, ahi), (blo, bhi));
        }
        proptest::prop_assert_eq!(volume(Difference, &ba, &bb), Ok(va - common), "difference {:?} {:?}", (alo, ahi), (blo, bhi));
    }
}
