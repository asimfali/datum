use datum_math::{Rigid, Vec2, Vec3};
use proptest::prelude::*;

use crate::*;

fn id(tag: &str) -> TopoId {
    TopoId::derive(tag).finish()
}

fn unit_cube() -> Body {
    cuboid(id("test.cube"), Vec3::ZERO, Vec3::new(1.0, 1.0, 1.0)).unwrap()
}

/// L-shaped (non-convex) prism of area 3 and height 1.5.
fn l_prism() -> Body {
    let l = [(0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (1.0, 1.0), (1.0, 2.0), (0.0, 2.0)].map(|(x, y)| Vec2::new(x, y));
    prism(id("test.l"), &l, 0.0, 1.5).unwrap()
}

fn hex(h: [u8; 32]) -> String {
    h.iter().map(|b| format!("{b:02x}")).collect()
}

/// The faces of `b` as builder input.
fn specs(b: &Body) -> Vec<FaceSpec> {
    (0..b.faces().len() as Ix).map(|f| b.face_spec(f)).collect()
}

fn mesh_volume(m: &Mesh) -> f64 {
    let o = m.positions[0];
    m.triangles
        .iter()
        .map(|t| (m.positions[t[0] as usize] - o).dot((m.positions[t[1] as usize] - o).cross(m.positions[t[2] as usize] - o)))
        .sum::<f64>()
        / 6.0
}

#[test]
fn unit_cube_counts_volume_and_mesh() {
    let b = unit_cube();
    b.validate().unwrap();
    assert_eq!((b.vertices().len(), b.edges().len(), b.faces().len(), b.half_edges().len()), (8, 12, 6, 24));
    assert_eq!(b.volume(), 1.0);
    assert_eq!(tessellate(&b).unwrap().triangles.len(), 12);
}

/// Canonical hash pinned across runs, platforms and thread counts. Changing it is a format change.
#[test]
fn unit_cube_hash_is_pinned() {
    assert_eq!(hex(unit_cube().hash()), "576d3c1b863281770188fc9febadc109ccd38387dc5f1498c32cfe0afdc87530");
}

#[test]
fn mass_properties_of_a_box() {
    let b = cuboid(id("box"), Vec3::new(1.0, 2.0, 3.0), Vec3::new(3.0, 3.0, 7.0)).unwrap();
    let m = b.mass_properties();
    assert_eq!((m.volume, m.area), (8.0, 2.0 * (2.0 + 8.0 + 4.0)));
    assert_eq!(m.centroid, Vec3::new(2.0, 2.5, 5.0));
    assert_eq!((m.min, m.max), (Vec3::new(1.0, 2.0, 3.0), Vec3::new(3.0, 3.0, 7.0)));
}

#[test]
fn non_convex_prism() {
    let b = l_prism();
    assert_eq!(b.volume(), 4.5);
    let m = tessellate(&b).unwrap();
    assert_eq!(m.triangles.len(), 2 * 4 + 2 * 6);
    assert_eq!(mesh_volume(&m), 4.5);
}

#[test]
fn input_order_does_not_change_the_body() {
    let b = l_prism();
    let mut vertices = b.vertices().to_vec();
    vertices.reverse();
    let mut faces = specs(&b);
    faces.reverse();
    for f in &mut faces {
        f.loops[0].rotate_left(1);
    }
    let again = build(vertices, faces).unwrap();
    assert_eq!(again, b);
}

#[test]
fn polygon_orientation_is_normalised() {
    let mut l = [(0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (1.0, 1.0), (1.0, 2.0), (0.0, 2.0)].map(|(x, y)| Vec2::new(x, y));
    l.reverse();
    assert_eq!(prism(id("test.l"), &l, 0.0, 1.5).unwrap().volume(), 4.5);
}

#[test]
fn degenerate_input_is_refused() {
    let p = |v: &[(f64, f64)]| v.iter().map(|&(x, y)| Vec2::new(x, y)).collect::<Vec<_>>();
    let cases = [
        (p(&[(0.0, 0.0), (1.0, 0.0)]), 1.0),
        (p(&[(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (1.0, 1.0)]), 1.0), // collinear neighbours
        (p(&[(0.0, 0.0), (1.0, 0.0), (1.0, 0.0), (0.0, 1.0)]), 1.0), // repeated point
        (p(&[(0.0, 0.0), (1.0, 1.0), (1.0, 0.0), (0.0, 1.0)]), 1.0), // bow tie
        (p(&[(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (2.0, 0.0), (0.0, 4.0)]), 1.0), // touches itself
        (p(&[(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)]), 0.0),             // zero height
        (p(&[(0.0, 0.0), (f64::NAN, 0.0), (0.0, 1.0)]), 1.0),
    ];
    for (poly, h) in cases {
        assert!(matches!(prism(id("x"), &poly, 0.0, h), Err(BrepError::Degenerate(_))), "{poly:?} h={h}");
    }
}

#[test]
fn open_shell_is_refused() {
    let b = unit_cube();
    let mut faces = specs(&b);
    faces.pop();
    assert!(matches!(build(b.vertices().to_vec(), faces), Err(BrepError::NotClosed(_))));
}

/// Principle 8: no modelling box. Far from the origin and turned by an awkward angle, the body has
/// the same topology and, within the rounding of its coordinates, the same volume.
#[test]
fn moving_far_away_keeps_topology_and_volume() {
    let b = l_prism();
    let ids = |b: &Body| {
        (
            b.vertices().iter().map(|v| v.id).collect::<Vec<_>>(),
            b.edges().iter().map(|e| e.id).collect::<Vec<_>>(),
            b.faces().iter().map(|f| f.id).collect::<Vec<_>>(),
        )
    };
    for offset in [0.0, 1e3, 1e6, 1e7] {
        let m = Rigid::rotation(Vec3::new(0.3, -0.5, 0.8), 0.7366).then(Rigid::translation(Vec3::new(1.0, 0.6, -0.3) * offset));
        let moved = b.transformed(&m);
        moved.validate().unwrap();
        assert_eq!(ids(&moved), ids(&b));
        assert_eq!(moved.half_edges(), b.half_edges());
        // Coordinates round to about ε·|p|; the volume of a body of unit size follows.
        let bound = 64.0 * f64::EPSILON * (1.0 + 1.2 * offset);
        let rel = (moved.volume() - b.volume()).abs() / b.volume();
        assert!(rel <= bound, "offset {offset}: relative volume error {rel} > {bound}");
        // Triangles may differ: a vertex exactly on a candidate diagonal is collinear before the
        // move and not after rounding, and the exact predicate answers each case truthfully.
        let mesh = tessellate(&moved).unwrap();
        assert_eq!(mesh.triangles.len(), tessellate(&b).unwrap().triangles.len());
        assert!((mesh_volume(&mesh) - moved.volume()).abs() <= bound * b.volume());
    }
}

/// Star-shaped polygons with `n` vertices: angles spread with jitter, so every gap is below π
/// and the polygon is simple.
fn star_polygon() -> impl Strategy<Value = Vec<Vec2>> {
    (4usize..40).prop_flat_map(|n| proptest::collection::vec((0.0..0.9f64, 0.1..10.0f64), n)).prop_map(|v| {
        let n = v.len() as f64;
        v.iter()
            .enumerate()
            .map(|(k, &(jitter, r))| {
                let a = std::f64::consts::TAU * (k as f64 + jitter) / n;
                Vec2::new(r * a.cos(), r * a.sin())
            })
            .collect()
    })
}

proptest! {
    #[test]
    fn prism_invariants(poly in star_polygon(), h in 0.01..100.0f64) {
        let b = prism(id("prop"), &poly, -h / 2.0, h / 2.0).unwrap();
        b.validate().unwrap();
        let n = poly.len();
        prop_assert_eq!((b.vertices().len(), b.edges().len(), b.faces().len()), (2 * n, 3 * n, n + 2));
        let area = (0..n).map(|i| { let (a, c) = (poly[i], poly[(i + 1) % n]); a.x * c.y - c.x * a.y }).sum::<f64>() / 2.0;
        prop_assert!((b.volume() - area * h).abs() <= 1e-12 * area * h);
        let m = tessellate(&b).unwrap();
        prop_assert_eq!(m.triangles.len(), 2 * (n - 2) + 2 * n);
        prop_assert!((mesh_volume(&m) - b.volume()).abs() <= 1e-12 * b.volume());
        prop_assert_eq!(prism(id("prop"), &poly, -h / 2.0, h / 2.0).unwrap().hash(), b.hash());
    }
}
