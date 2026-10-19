//! Terrain and excavation (M2e, PAINS P9): the grid solid, stable IDs, cut volumes.
#![allow(clippy::unwrap_used, clippy::panic)]

use datum_brep::{Body, BrepError, Grid, TopoId, prism, terrain, tessellate};
use datum_math::{Rigid, Vec2, Vec3};
use datum_ops::{BooleanKind, boolean};

fn id(tag: &str) -> TopoId {
    TopoId::derive(tag).finish()
}

fn assert_close(a: f64, b: f64, rel: f64) {
    assert!((a - b).abs() <= rel * b.abs().max(1.0), "{a} vs {b}");
}

fn checked(r: Result<Body, BrepError>) -> Body {
    let b = r.unwrap();
    b.validate().unwrap();
    tessellate(&b).unwrap();
    b
}

/// `n × n` nodes, 1 m apart from (0, 0), heights from `h`.
fn ground(n: u32, h: impl Fn(f64, f64) -> f64) -> Result<Body, BrepError> {
    let heights: Vec<f64> = (0..n).flat_map(|j| (0..n).map(move |i| (i, j))).map(|(i, j)| h(f64::from(i), f64::from(j))).collect();
    terrain(id("ground"), &Grid { origin: Vec2::new(0.0, 0.0), step: [1.0, 1.0], count: [n, n], heights: &heights }, -5.0)
}

/// Rolling ground: no two neighbouring cells coplanar.
fn hills(x: f64, y: f64) -> f64 {
    10.0 + 0.5 * wave(0.37 * x) + 0.3 * wave(0.29 * y + 0.11 * x)
}

/// A smooth periodic bump from polynomials, so the test needs no transcendental functions.
fn wave(t: f64) -> f64 {
    let u = t - 4.0 * (t / 4.0).floor() - 2.0;
    u * u * (2.0 - u.abs()) - 1.0
}

/// A square pit of side `side` with its bottom at `bottom`, turned by `angle` about its centre.
fn pit(centre: (f64, f64), side: f64, angle: f64, bottom: f64) -> Body {
    let h = side / 2.0;
    let square = [(-h, -h), (h, -h), (h, h), (-h, h)].map(|(x, y)| Vec2::new(x, y));
    prism(id("pit"), &square, bottom, 100.0)
        .unwrap()
        .transformed(&Rigid::rotation(Vec3::new(0.0, 0.0, 1.0), angle).then(Rigid::translation(Vec3::new(centre.0, centre.1, 0.0))))
}

#[test]
fn grid_solid_has_the_volume_under_its_heights() {
    // A tilted plane: the volume over the base is the area times the mean height above it.
    let g = checked(ground(11, |x, y| 3.0 + 0.25 * x + 0.5 * y));
    assert_close(g.volume(), 100.0 * (3.0 + 0.25 * 5.0 + 0.5 * 5.0 + 5.0), 1e-12);
    assert_eq!(g.faces().len(), 2 * 10 * 10 + 5);
    // Flat ground: collinear nodes along the sides still triangulate.
    assert_close(checked(ground(5, |_, _| 2.0)).volume(), 16.0 * 7.0, 1e-12);
}

/// Changing a height moves geometry, not identity: references to cells survive survey updates.
#[test]
fn changed_heights_keep_ids() {
    let ids = |b: &Body| b.faces().iter().map(|f| f.id).collect::<Vec<_>>();
    let a = ground(6, hills).unwrap();
    let b = ground(6, |x, y| hills(x, y) + if x == 2.0 && y == 3.0 { 0.7 } else { 0.0 }).unwrap();
    assert_eq!(ids(&a), ids(&b));
    assert_ne!(a.hash(), b.hash());
}

#[test]
fn bad_grids_are_refused() {
    let bad = |r: Result<Body, BrepError>, why: &str| match r {
        Err(BrepError::Degenerate(m)) => assert!(m.contains(why), "{m}"),
        r => panic!("expected `{why}`, got {r:?}"),
    };
    bad(ground(1, hills), "at least 2 × 2");
    bad(ground(4, |_, _| -6.0), "above the base");
    bad(terrain(id("t"), &Grid { origin: Vec2::new(0.0, 0.0), step: [1.0, 1.0], count: [3, 3], heights: &[1.0; 8] }, 0.0), "needs 9 heights");
    bad(terrain(id("t"), &Grid { origin: Vec2::new(0.0, 0.0), step: [1.0, -1.0], count: [2, 2], heights: &[1.0; 4] }, 0.0), "positive");
}

/// On planar ground the excavated volume is exact: pit area times the depth at its centre.
#[test]
fn excavation_of_planar_ground() {
    let g = ground(21, |x, y| 3.0 + 0.25 * x + 0.5 * y).unwrap();
    let (c, side, bottom) = ((9.3, 10.6), 6.0, 1.5);
    let cut = checked(boolean(id("dig"), BooleanKind::Intersection, None, &[&g, &pit(c, side, 0.4, bottom)]));
    assert_close(cut.volume(), side * side * (3.0 + 0.25 * c.0 + 0.5 * c.1 - bottom), 1e-12);
}

/// On rolling ground excavated and remaining soil add up to the ground.
#[test]
fn excavation_and_remainder_add_up() {
    let g = ground(30, hills).unwrap();
    let p = pit((14.2, 13.7), 9.0, 0.3, 8.5);
    let dug = checked(boolean(id("dig"), BooleanKind::Intersection, None, &[&g, &p])).volume();
    let left = checked(boolean(id("dig"), BooleanKind::Difference, None, &[&g, &p])).volume();
    assert_close(dug + left, g.volume(), 1e-12);
    assert!(dug > 9.0 * 9.0 * 0.5);
}

/// Q7: walls along grid lines and a bottom at a node's height are ordinary for an engineer.
/// The grid nodes on the walls stay vertices of the result, and the volume is exact.
#[test]
fn pit_on_the_grid() {
    let g = ground(21, |x, y| 3.0 + 0.25 * x + 0.5 * y).unwrap();
    // Walls on x = 6, 12 and y = 5, 13; the bottom is the height of node (6, 5).
    let pit = prism(id("pit"), &[(6.0, 5.0), (12.0, 5.0), (12.0, 13.0), (6.0, 13.0)].map(|(x, y)| Vec2::new(x, y)), 1.0, 100.0).unwrap();
    let cut = checked(boolean(id("dig"), BooleanKind::Intersection, None, &[&g, &pit]));
    assert_close(cut.volume(), 48.0 * (3.0 + 0.25 * 9.0 + 0.5 * 9.0 - 1.0), 1e-12);
    assert!(cut.vertices().iter().any(|v| v.point == Vec3::new(6.0, 5.0, 3.0 + 1.5 + 2.5)));
    let rolling = ground(16, hills).unwrap();
    let flat_bottom =
        prism(id("pit"), &[(3.0, 4.0), (9.0, 4.0), (9.0, 11.0), (3.0, 11.0)].map(|(x, y)| Vec2::new(x, y)), hills(5.0, 6.0), 100.0).unwrap();
    let dug = checked(boolean(id("dig"), BooleanKind::Intersection, None, &[&rolling, &flat_bottom])).volume();
    let left = checked(boolean(id("dig"), BooleanKind::Difference, None, &[&rolling, &flat_bottom])).volume();
    assert_close(dug + left, rolling.volume(), 1e-12);
}
