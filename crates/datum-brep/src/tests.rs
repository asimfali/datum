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
    for offset in [0.0, 1e3, 1e6, 1e7, 1e10] {
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
                Vec2::new(r * libm::cos(a), r * libm::sin(a))
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
        _ => None,
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
    let rebuilt = build_with_curves(far.vertices().to_vec(), specs(&far), far.curve_specs()).unwrap();
    assert_eq!(rebuilt.hash(), far.hash());
    assert!(far.mapped(|p| p, |n| n).is_err(), "an affine map of arcs is refused");
}

#[test]
fn loop_perimeter_counts_chords_and_arcs() {
    assert_eq!(square("p", 10.0).perimeter(), 40.0);
    // A semicircle of radius 1 and its diameter.
    let pts = vec![Vec2::new(0.0, 0.0), Vec2::new(2.0, 0.0)];
    let arcs = vec![Some(ProfileArc { center: Vec2::new(1.0, 0.0), ccw: true }), None];
    assert!(close(ring("half", pts, arcs).perimeter(), 2.0 + std::f64::consts::PI));
    assert!(close(circle("c", Vec2::new(3.0, 1.0), 2.0).perimeter(), 2.0 * std::f64::consts::TAU));
}

/// A tilted axis away from the origin: revolutions must not lean on coordinate axes.
fn tilted_axis() -> Axis {
    let axis = Vec3::new(1.0, 2.0, 2.0) * (1.0 / 3.0);
    let start = Vec3::new(2.0, -2.0, 1.0) * (1.0 / 3.0);
    Axis { origin: Vec3::new(5.0, -7.0, 11.0), axis, start }
}

/// Area and centroid of a polygon by the shoelace formula.
fn polygon_area_centroid(p: &[Vec2]) -> (f64, Vec2) {
    let (mut a, mut cx, mut cy) = (0.0, 0.0, 0.0);
    for k in 0..p.len() {
        let (u, v) = (p[k], p[(k + 1) % p.len()]);
        let c = u.x * v.y - v.x * u.y;
        a += c;
        cx += (u.x + v.x) * c;
        cy += (u.y + v.y) * c;
    }
    (0.5 * a, Vec2::new(cx / (3.0 * a), cy / (3.0 * a)))
}

fn revolved(points: &[(f64, f64)], at: &Axis, angle: f64) -> Result<Body, BrepError> {
    let pts = points.iter().map(|&(x, y)| Vec2::new(x, y)).collect();
    revolve(&region(ring("rev", pts, Vec::new()), Vec::new()), at, angle)
}

/// Volume, area and centroid of revolutions against Pappus and the frustum's formulas, in full
/// turns and in sectors, with cylinders, cones and planes.
#[test]
fn revolution_mass_properties() {
    let pi = std::f64::consts::PI;
    let at = tilted_axis();
    // A tube: two cylinders and two rings.
    let (h, r1, r2) = (4.0, 2.0, 3.0);
    let tube = revolved(&[(0.0, r1), (h, r1), (h, r2), (0.0, r2)], &at, std::f64::consts::TAU).unwrap();
    tube.validate().unwrap();
    assert_eq!((tube.vertices().len(), tube.edges().len(), tube.faces().len()), (12, 18, 8));
    let m = tube.mass_properties();
    assert!(close(m.volume, pi * (r2 * r2 - r1 * r1) * h), "{}", m.volume);
    assert!(close(m.area, 2.0 * pi * (r1 + r2) * h + 2.0 * pi * (r2 * r2 - r1 * r1)), "{}", m.area);
    let centre = at.origin + at.axis * (0.5 * h);
    assert!((m.centroid - centre).dot(m.centroid - centre) < 1e-20, "{:?}", m.centroid);

    // A solid frustum: a cone and two discs, the profile clockwise.
    let frustum = revolved(&[(0.0, 0.0), (0.0, r1), (h, r2), (h, 0.0)], &at, std::f64::consts::TAU).unwrap();
    frustum.validate().unwrap();
    let cones = frustum.faces().iter().filter(|f| matches!(f.surface, Surface::Cone(c) if c.convex)).count();
    assert_eq!((cones, frustum.faces().len()), (3, 5));
    let m = frustum.mass_properties();
    assert!(close(m.volume, pi * h * (r1 * r1 + r1 * r2 + r2 * r2) / 3.0), "{}", m.volume);
    let slant = libm::hypot(h, r2 - r1);
    assert!(close(m.area, pi * (r1 + r2) * slant + pi * (r1 * r1 + r2 * r2)), "{}", m.area);

    // A sector of an oblique quadrilateral: Pappus, V = A·φ·ȳ, two cones inside and out.
    let quad = [(0.0, 1.0), (4.0, 1.5), (3.0, 3.0), (1.0, 2.0)];
    let phi = 2.0;
    let sector = revolved(&quad, &at, phi).unwrap();
    sector.validate().unwrap();
    let (area, c) = polygon_area_centroid(&quad.map(|(x, y)| Vec2::new(x, y)));
    let m = sector.mass_properties();
    assert!(close(m.volume, area * phi * c.y), "{} vs {}", m.volume, area * phi * c.y);
    // The centroid of a sector of revolution: at the mean angle, 2·sin(φ/2)/φ of the way out.
    // Along the axis ∫ x·y dA / ∫ y dA, out from it ∫ y² dA / ∫ y dA over the profile (by
    // triangles from the origin), the latter shrunk by 2·sin(φ/2)/φ.
    let (mut iy, mut ixy, mut iyy) = (0.0, 0.0, 0.0);
    for k in 0..quad.len() {
        let (u, v) = (quad[k], quad[(k + 1) % quad.len()]);
        let cr = u.0 * v.1 - v.0 * u.1;
        iy += cr * (u.1 + v.1) / 6.0;
        ixy += cr * (2.0 * u.0 * u.1 + u.0 * v.1 + v.0 * u.1 + 2.0 * v.0 * v.1) / 24.0;
        iyy += cr * (u.1 * u.1 + u.1 * v.1 + v.1 * v.1) / 12.0;
    }
    let expected = at.origin + at.axis * (ixy / iy) + at.radial(0.5 * phi) * (iyy / iy * 2.0 * libm::sin(0.5 * phi) / phi);
    assert!((m.centroid - expected).dot(m.centroid - expected) < 1e-20, "{:?} vs {expected:?}", m.centroid);

    // A half-turn sector of a profile along the axis: the axis edge joins the two caps.
    let half = revolved(&[(0.0, 0.0), (3.0, 0.0), (3.0, 1.0), (0.0, 2.0)], &at, pi).unwrap();
    half.validate().unwrap();
    let (area, c) = polygon_area_centroid(&[Vec2::new(0.0, 0.0), Vec2::new(3.0, 0.0), Vec2::new(3.0, 1.0), Vec2::new(0.0, 2.0)]);
    assert!(close(half.volume(), area * pi * c.y), "{}", half.volume());
}

/// What a revolution refuses: a cone to a point, arcs, holes, a profile across the axis.
#[test]
fn revolution_refusals() {
    let at = tilted_axis();
    let full = std::f64::consts::TAU;
    assert!(matches!(revolved(&[(0.0, 0.0), (2.0, 0.0), (0.0, 1.0)], &at, full), Err(BrepError::Unsupported(m)) if m.contains("cone to a point")));
    assert!(matches!(revolved(&[(0.0, -1.0), (2.0, 1.0), (0.0, 1.0)], &at, full), Err(BrepError::Degenerate(m)) if m.contains("axis")));
    assert!(matches!(revolved(&[(0.0, 1.0), (1.0, 1.0), (2.0, 1.0), (1.0, 2.0)], &at, full), Err(BrepError::Degenerate(_))));
    let arc = revolve(&region(circle("c", Vec2::new(0.0, 3.0), 1.0), Vec::new()), &at, full);
    assert!(matches!(arc, Err(BrepError::Unsupported(m)) if m.contains("torus")));
}

/// Cones on screen and in STEP: the mesh holds about the volume, STEP names the cone.
#[test]
fn cone_tessellation_and_step() {
    let at = tilted_axis();
    let b = revolved(&[(0.0, 1.0), (3.0, 2.0), (3.0, 4.0), (0.0, 3.0)], &at, std::f64::consts::TAU).unwrap();
    b.validate().unwrap();
    let mesh = tessellate(&b).unwrap();
    let (v, mv) = (b.volume(), mesh_volume(&mesh));
    // Chords lose up to (2π/64)²/6 ≈ 0.16 % of a round section.
    assert!((mv - v).abs() < 3e-3 * v, "{mv} vs {v}");
    let s = step(&[&b], "cone", false).unwrap();
    assert_eq!(s.text.matches("CONICAL_SURFACE").count(), 6);
    assert_eq!(s.approximated, 0);
}

/// `Wave::range` against dense sampling: ends, ranges over one turning point and several,
/// reversed ends, a pure cosine or sine, a constant.
#[test]
fn wave_range_matches_dense_sampling() {
    let cases = [
        (Wave { a: 70.0, b: -18.5, c: 0.3 }, 50.88, 0.0, 317.7),
        (Wave { a: 70.0, b: -18.5, c: 0.3 }, 50.88, 317.7, 0.0),
        (Wave { a: -3.0, b: 5.0, c: -7.0 }, 10.0, 1.0, 5.0),
        (Wave { a: -3.0, b: 5.0, c: -7.0 }, 10.0, 0.0, 400.0),
        (Wave { a: 1.0, b: 0.0, c: 2.0 }, 3.0, -20.0, 20.0),
        (Wave { a: 1.0, b: 2.0, c: 0.0 }, 3.0, 20.0, -20.0),
        (Wave { a: 0.0, b: 0.0, c: 0.0 }, 3.0, 0.0, 10.0),
        (Wave { a: 5.0, b: 1.0, c: 1.0 }, 7.0, 2.0, 2.0),
        // The range starts and ends exactly at turning points.
        (Wave { a: 0.0, b: 1.0, c: 0.0 }, 1.0, 0.0, 3.0 * std::f64::consts::PI),
    ];
    for (w, n, u0, u1) in cases {
        let [lo, hi] = w.range(u0, u1, n);
        let steps = 200_000;
        let (mut slo, mut shi) = (f64::INFINITY, f64::NEG_INFINITY);
        for i in 0..=steps {
            let v = w.at(u0 + (u1 - u0) * f64::from(i) / f64::from(steps), n)[0];
            slo = slo.min(v);
            shi = shi.max(v);
        }
        // Sampling never beats the exact range, and comes close to it.
        let slack = 1e-6 * (1.0 + (w.b * w.b + w.c * w.c).sqrt());
        assert!(lo <= slo + 1e-12 && hi >= shi - 1e-12, "{w:?} {u0}..{u1}: [{lo}, {hi}] inside [{slo}, {shi}]");
        assert!(lo >= slo - slack && hi <= shi + slack, "{w:?} {u0}..{u1}: [{lo}, {hi}] vs [{slo}, {shi}]");
    }
}

/// A piece following a wave: points, tangents and second derivatives against finite
/// differences, whichever way it runs.
#[test]
fn rolled_wave_derivatives_match_finite_differences() {
    let dev = Development { origin: Vec3::ZERO, axis: Vec3::new(1.0, 0.0, 0.0), start: Vec3::new(0.0, 1.0, 0.0), neutral: 50.88 };
    let wave = Wave { a: 70.0, b: -18.5, c: 0.7 };
    for (u0, u1) in [(0.0, 317.7), (317.7, 0.0), (40.0, 90.0)] {
        let r = Rolled {
            dev,
            from: Vec2::new(u0, wave.at(u0, dev.neutral)[0]),
            to: Vec2::new(u1, wave.at(u1, dev.neutral)[0]),
            arc: None,
            trace: Some(crate::Trace::Wave(wave)),
        };
        assert_eq!(r.at(0.0).0, r.from);
        assert!((r.at(1.0).0.x - u1).abs() < 1e-12 && (r.at(1.0).0.y - wave.at(u1, dev.neutral)[0]).abs() < 1e-9);
        let h = 1e-5;
        for t in [0.1, 0.35, 0.5, 0.8] {
            let (p, d) = r.at(t);
            let (a, b) = (r.at(t - h), r.at(t + h));
            let fd = Vec2::new((b.0.x - a.0.x) / (2.0 * h), (b.0.y - a.0.y) / (2.0 * h));
            let fd2 = Vec2::new((b.1.x - a.1.x) / (2.0 * h), (b.1.y - a.1.y) / (2.0 * h));
            let s = r.second(t);
            let scale = (u1 - u0).abs();
            assert!((fd.x - d.x).abs() <= 1e-6 * scale && (fd.y - d.y).abs() <= 1e-6 * scale, "{u0}..{u1} t {t}: {fd:?} vs {d:?}");
            assert!((fd2.x - s.x).abs() <= 1e-6 * scale && (fd2.y - s.y).abs() <= 1e-4 * scale, "{u0}..{u1} t {t}: {fd2:?} vs {s:?}");
            assert!(s.x == 0.0 && (p.y - wave.at(p.x, dev.neutral)[0]).abs() < 1e-9);
        }
        // Reversed: the same curve the other way, derivatives of opposite sign.
        let back = r.reversed();
        let ((p, d), (q, e)) = (r.at(0.3), back.at(0.7));
        assert!((p.x - q.x).abs() < 1e-9 && (p.y - q.y).abs() < 1e-9);
        assert!((d.x + e.x).abs() < 1e-9 && (d.y + e.y).abs() < 1e-9);
    }
}
