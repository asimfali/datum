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

/// A loop of `n` points with IDs from `tag`; arcs as given.
fn ring(tag: &str, points: Vec<Vec2>, arcs: Vec<Option<ProfileArc>>) -> ProfileLoop {
    let n = points.len();
    let vid = |layer: u64, k: usize| TopoId::derive(tag).index(layer).index(k as u64).finish();
    ProfileLoop {
        points,
        vertices: (0..n).map(|k| [vid(0, k), vid(1, k)]).collect(),
        sides: (0..n).map(|k| TopoId::derive(tag).tag("side").index(k as u64).finish()).collect(),
        arcs,
    }
}

/// A circle as three arcs, counterclockwise.
fn circle(tag: &str, c: Vec2, r: f64) -> ProfileLoop {
    let at = |k: usize| {
        let (s, co) = libm::sincos(std::f64::consts::TAU * k as f64 / 3.0);
        Vec2::new(c.x + r * co, c.y + r * s)
    };
    ring(tag, (0..3).map(at).collect(), vec![Some(ProfileArc { center: c, ccw: true }); 3])
}

fn square(tag: &str, side: f64) -> ProfileLoop {
    let pts = [(0.0, 0.0), (side, 0.0), (side, side), (0.0, side)].map(|(x, y)| Vec2::new(x, y)).to_vec();
    ring(tag, pts, Vec::new())
}

fn region(outer: ProfileLoop, holes: Vec<ProfileLoop>) -> Region {
    Region { outer, holes, caps: [id("cap0"), id("cap1")] }
}

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-12 * b.abs().max(1.0)
}

#[test]
fn cylinder_volume_area_centroid_and_bounds() {
    let b = extrude(&[region(circle("c", Vec2::new(1.0, -2.0), 2.0), Vec::new())], 0.0, 3.0).unwrap();
    b.validate().unwrap();
    assert_eq!((b.vertices().len(), b.edges().len(), b.faces().len()), (6, 9, 5));
    let pi = std::f64::consts::PI;
    let m = b.mass_properties();
    assert!(close(m.volume, pi * 4.0 * 3.0), "{}", m.volume);
    assert!(close(m.area, 2.0 * pi * 4.0 + 2.0 * pi * 2.0 * 3.0), "{}", m.area);
    let c = m.centroid - Vec3::new(1.0, -2.0, 1.5);
    assert!(c.dot(c).sqrt() < 1e-12, "{:?}", m.centroid);
    let (lo, hi) = (m.min - Vec3::new(-1.0, -4.0, 0.0), m.max - Vec3::new(3.0, 0.0, 3.0));
    assert!(lo.dot(lo) < 1e-24 && hi.dot(hi) < 1e-24, "{:?} {:?}", m.min, m.max);
    // The chords of the mesh lie inside the circle: a little less volume, by 0.16 % at 64 a turn.
    let mesh = mesh_volume(&tessellate(&b).unwrap());
    assert!(mesh < m.volume && mesh > m.volume * 0.998, "{mesh}");
}

#[test]
fn plate_with_a_round_hole() {
    let b = extrude(&[region(square("sq", 10.0), vec![circle("h", Vec2::new(4.0, 5.0), 1.5)])], 0.0, 2.0).unwrap();
    b.validate().unwrap();
    let m = b.mass_properties();
    let pi = std::f64::consts::PI;
    let hole = pi * 1.5 * 1.5;
    assert!(close(m.volume, (100.0 - hole) * 2.0), "{}", m.volume);
    assert!(close(m.area, 2.0 * (100.0 - hole) + 4.0 * 10.0 * 2.0 + 2.0 * pi * 1.5 * 2.0), "{}", m.area);
    // The hole is left of centre: the centre of mass moves right.
    let x = (100.0 * 5.0 - hole * 4.0) / (100.0 - hole);
    assert!(close(m.centroid.x, x) && close(m.centroid.y, 5.0) && close(m.centroid.z, 1.0), "{:?}", m.centroid);
}

/// The profile of a rolled sheet: a ring sector between radii 100 and 103, over 90°.
#[test]
fn rolled_sheet_sector() {
    let (r, big) = (100.0, 103.0);
    let pts = vec![Vec2::new(r, 0.0), Vec2::new(big, 0.0), Vec2::new(0.0, big), Vec2::new(0.0, r)];
    let o = Vec2::new(0.0, 0.0);
    let arcs = vec![None, Some(ProfileArc { center: o, ccw: true }), None, Some(ProfileArc { center: o, ccw: false })];
    let b = extrude(&[region(ring("sheet", pts, arcs), Vec::new())], 0.0, 500.0).unwrap();
    b.validate().unwrap();
    let v = std::f64::consts::FRAC_PI_4 * (big * big - r * r) * 500.0;
    assert!(close(b.volume(), v), "{} vs {v}", b.volume());
    let convex = b.faces().iter().filter_map(|f| match f.surface {
        Surface::Cylinder(c) => Some(c.convex),
        Surface::Plane(_) => None,
    });
    let mut convex: Vec<bool> = convex.collect();
    convex.sort();
    assert_eq!(convex, [false, true], "the inner face sees the axis, the outer one turns away");
}

/// A slot: two lines and two half circles; the ends' circles do not touch the lines elsewhere.
#[test]
fn slot() {
    let (l, r) = (10.0, 2.5);
    let pts = vec![Vec2::new(0.0, -r), Vec2::new(l, -r), Vec2::new(l, r), Vec2::new(0.0, r)];
    let arcs =
        vec![None, Some(ProfileArc { center: Vec2::new(l, 0.0), ccw: true }), None, Some(ProfileArc { center: Vec2::new(0.0, 0.0), ccw: true })];
    let b = extrude(&[region(ring("slot", pts, arcs), Vec::new())], 0.0, 1.0).unwrap();
    b.validate().unwrap();
    assert!(close(b.volume(), 2.0 * r * l + std::f64::consts::PI * r * r), "{}", b.volume());
}

#[test]
fn arcs_that_touch_are_refused() {
    // A hole across the plate's edge, and a hole crossing another. (Exact tangency is not a case:
    // the vertices of a circle are rounded, and the arcs' radii with them.)
    let across = extrude(&[region(square("sq", 10.0), vec![circle("h", Vec2::new(1.4, 5.0), 1.5)])], 0.0, 1.0);
    assert!(matches!(across, Err(BrepError::Degenerate(_))), "{across:?}");
    let crossing =
        extrude(&[region(square("sq", 10.0), vec![circle("a", Vec2::new(4.0, 5.0), 1.5), circle("b", Vec2::new(6.0, 5.0), 1.5)])], 0.0, 1.0);
    assert!(matches!(crossing, Err(BrepError::Degenerate(_))), "{crossing:?}");
}

/// Principle 8 with arcs: far away and turned, the same topology, volume and hashes of the
/// same rebuild.
#[test]
fn arcs_far_away_and_turned() {
    let b = extrude(&[region(square("sq", 10.0), vec![circle("h", Vec2::new(4.0, 5.0), 1.5)])], 0.0, 2.0).unwrap();
    let m = Rigid::rotation(Vec3::new(1.0, 2.0, 3.0), 0.7).then(Rigid::translation(Vec3::new(1e7, -3e6, 2e6)));
    let far = b.transformed(&m);
    far.validate().unwrap();
    assert_eq!((far.vertices().len(), far.edges().len(), far.faces().len()), (b.vertices().len(), b.edges().len(), b.faces().len()));
    assert!((far.volume() - b.volume()).abs() < 1e-8 * b.volume(), "{} vs {}", far.volume(), b.volume());
    let rebuilt = build_with_arcs(far.vertices().to_vec(), specs(&far), far.arc_specs()).unwrap();
    assert_eq!(rebuilt.hash(), far.hash());
    assert!(far.mapped(|p| p, |n| n).is_err(), "an affine map of arcs is refused");
}
