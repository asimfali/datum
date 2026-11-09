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
    // An arc crossing the axis, reaching it off centre, centred beyond it.
    let arcs = |pts: &[(f64, f64)], arc: ProfileArc, on: usize| {
        let mut arcs = vec![None; pts.len()];
        arcs[on] = Some(arc);
        revolve(&region(ring("rev", pts.iter().map(|&(x, y)| Vec2::new(x, y)).collect(), arcs), Vec::new()), &at, full)
    };
    let crossing = arcs(&[(-1.0, 0.5), (1.0, 0.5), (1.0, 3.0), (-1.0, 3.0)], ProfileArc { center: Vec2::new(0.0, 0.5), ccw: true }, 0);
    assert!(matches!(crossing, Err(BrepError::Degenerate(m)) if m.contains("axis")));
    let folded = arcs(&[(0.0, 0.0), (2.0, 2.0), (0.0, 2.0)], ProfileArc { center: Vec2::new(0.0, 2.0), ccw: false }, 0);
    assert!(matches!(folded, Err(BrepError::Unsupported(m)) if m.contains("off its centre")));
    let beyond = arcs(&[(-1.0, 1.0), (1.0, 1.0), (1.0, 3.0), (-1.0, 3.0)], ProfileArc { center: Vec2::new(0.0, -1.0), ccw: true }, 2);
    assert!(matches!(beyond, Err(BrepError::Unsupported(m)) if m.contains("across the axis")));
}

/// A closed body's mesh: every edge, by where its ends are (faces number the points along an
/// arc apart), used once each way; and about the body's volume.
fn check_mesh(b: &Body) {
    let mesh = tessellate(b).unwrap();
    let key = |i: u32| mesh.positions[i as usize].to_array().map(|c| (c * 1e6).round() as i64);
    let mut edges = std::collections::BTreeMap::new();
    for t in &mesh.triangles {
        for k in 0..3 {
            *edges.entry((key(t[k]), key(t[(k + 1) % 3]))).or_insert(0) += 1;
        }
    }
    assert!(edges.iter().all(|(&(a, b), &n)| n == 1 && edges.get(&(b, a)) == Some(&1)), "the mesh is not closed");
    let (v, mv) = (b.volume(), mesh_volume(&mesh));
    // Chords of at most 64 a turn both ways round: not the 0.16 % of a cylinder, but about 1 %.
    assert!((mv - v).abs() < 1.5e-2 * v, "mesh volume {mv} vs {v}");
}

/// Volume, area, centroid and box of tori and spheres against their formulas, on a tilted axis.
#[test]
fn torus_and_sphere_mass_properties() {
    use std::f64::consts::{PI, TAU};
    let at = tilted_axis();
    let on_axis = |x: f64, y: f64| at.origin + at.axis * x + at.start * y;
    // A ring: Pappus. The body's box reaches past every edge the kernel has.
    let (major, minor) = (5.0, 1.5);
    let b = revolve(&region(circle("c", Vec2::new(2.0, major), minor), Vec::new()), &at, TAU).unwrap();
    b.validate().unwrap();
    check_mesh(&b);
    assert_eq!(b.faces().len(), 9);
    let m = b.mass_properties();
    let closer = |a: f64, x: f64| (a - x).abs() <= 1e-10 * x.abs().max(1.0);
    assert!(closer(m.volume, 2.0 * PI * PI * major * minor * minor), "{}", m.volume);
    assert!(closer(m.area, 4.0 * PI * PI * major * minor), "{}", m.area);
    let c = on_axis(2.0, 0.0);
    assert!((m.centroid - c).dot(m.centroid - c) < 1e-18, "{:?}", m.centroid);
    // Along X the torus reaches `minor` past the circle of radius `major` tilted with the axis.
    let ax = at.axis.x;
    let reach = (major + minor) * (1.0 - ax * ax).sqrt();
    let extreme = major * (1.0 - ax * ax).sqrt() + minor;
    assert!(m.max.x - c.x >= reach * (1.0 - 1e-9) && (m.max.x - c.x - extreme).abs() < 1e-9, "{} vs {extreme}", m.max.x - c.x);
    // A ball: a half circle and its diameter on the axis, the arc cut in two for the poles.
    let r = 3.0;
    let half = vec![Some(ProfileArc { center: Vec2::new(1.0, 0.0), ccw: true }), None];
    let ball = revolve(&region(ring("s", vec![Vec2::new(1.0 + r, 0.0), Vec2::new(1.0 - r, 0.0)], half), Vec::new()), &at, TAU).unwrap();
    ball.validate().unwrap();
    check_mesh(&ball);
    let m = ball.mass_properties();
    assert!(closer(m.volume, 4.0 / 3.0 * PI * r * r * r), "{}", m.volume);
    assert!(closer(m.area, 4.0 * PI * r * r), "{}", m.area);
    let c = on_axis(1.0, 0.0);
    assert!((m.centroid - c).dot(m.centroid - c) < 1e-18, "{:?}", m.centroid);
    assert!((m.max.x - c.x - r).abs() < 1e-9 && (c.y - m.min.y - r).abs() < 1e-9, "{:?}", m.max);
    // A quarter of the ball by a sector: a pole on each of its faces, two plane caps with arcs.
    let half = vec![Some(ProfileArc { center: Vec2::new(1.0, 0.0), ccw: true }), None];
    let quarter = revolve(&region(ring("s", vec![Vec2::new(1.0 + r, 0.0), Vec2::new(1.0 - r, 0.0)], half), Vec::new()), &at, PI / 2.0).unwrap();
    quarter.validate().unwrap();
    check_mesh(&quarter);
    assert!(closer(quarter.volume(), PI * r * r * r / 3.0), "{}", quarter.volume());
    assert!(closer(quarter.mass_properties().area, PI * r * r + 2.0 * (0.5 * PI * r * r)), "{}", quarter.mass_properties().area);
}

/// A dome and a groove: a convex and a concave arc against the volumes of their shapes.
#[test]
fn dome_and_concave_groove() {
    use std::f64::consts::{PI, TAU};
    let at = tilted_axis();
    // Cylinder of radius 4 and height 2, topped by a half ball of radius 4: pole on the axis.
    let (r, h) = (4.0, 2.0);
    let top = |p: Vec<Vec2>, a: Vec<Option<ProfileArc>>| revolve(&region(ring("d", p, a), Vec::new()), &at, TAU);
    let dome = {
        let p = [(-r, 0.0), (h, 0.0), (h, r), (0.0, r)].map(|(x, y)| Vec2::new(x, y)).to_vec();
        let mut a = vec![None; 4];
        a[3] = Some(ProfileArc { center: Vec2::new(0.0, 0.0), ccw: true });
        top(p, a).unwrap()
    };
    dome.validate().unwrap();
    check_mesh(&dome);
    let expected = PI * r * r * h + 2.0 / 3.0 * PI * r * r * r;
    assert!((dome.volume() - expected).abs() < 1e-9 * expected, "{} vs {expected}", dome.volume());
    // A rod of radius 6 and length 4 with a groove: an arc of radius 2 into it, centred on its
    // surface. The groove takes away half a torus's section, less its bulge toward the axis.
    let (big, s) = (6.0, 2.0);
    let p = [(-2.0 * s, 0.0), (2.0 * s, 0.0), (2.0 * s, big), (s, big), (-s, big), (-2.0 * s, big)].map(|(x, y)| Vec2::new(x, y)).to_vec();
    let mut a = vec![None; 6];
    a[3] = Some(ProfileArc { center: Vec2::new(0.0, big), ccw: false });
    let rod = top(p, a).unwrap();
    rod.validate().unwrap();
    check_mesh(&rod);
    // Half a disc of radius 2: its area and centroid's distance from the axis, by Pappus.
    let (area, centroid) = (PI * s * s / 2.0, big - 4.0 * s / (3.0 * PI));
    let expected = PI * big * big * (4.0 * s) - TAU * centroid * area;
    assert!((rod.volume() - expected).abs() < 1e-9 * expected, "{} vs {expected}", rod.volume());
    assert!(rod.faces().iter().any(|f| matches!(f.surface, Surface::Torus(t) if !t.convex && t.major == big && t.minor == s)));
}

/// STEP names the surfaces; a torus reaching its own axis goes as a circle turned about it.
#[test]
fn torus_and_sphere_in_step() {
    use std::f64::consts::TAU;
    let at = tilted_axis();
    let b = revolve(&region(circle("c", Vec2::new(2.0, 5.0), 1.5), Vec::new()), &at, TAU).unwrap();
    let s = step(&[&b], "ring", false).unwrap();
    assert_eq!(s.text.matches("TOROIDAL_SURFACE").count(), 9);
    assert_eq!(s.approximated, 0);
    let half = vec![Some(ProfileArc { center: Vec2::new(0.0, 0.0), ccw: true }), None];
    let ball = revolve(&region(ring("s", vec![Vec2::new(3.0, 0.0), Vec2::new(-3.0, 0.0)], half), Vec::new()), &at, TAU).unwrap();
    assert_eq!(step(&[&ball], "ball", false).unwrap().text.matches("SPHERICAL_SURFACE").count(), 6);
    // Minor radius 2 about a circle of radius 1: the arc stays off the axis, its torus does not.
    let x = libm::sqrt(3.0);
    let mut arcs = vec![None; 4];
    arcs[0] = Some(ProfileArc { center: Vec2::new(0.0, 1.0), ccw: true });
    let pts = [(x, 2.0), (-x, 2.0), (-x, 1.0), (x, 1.0)].map(|(x, y)| Vec2::new(x, y)).to_vec();
    let fat = revolve(&region(ring("f", pts, arcs), Vec::new()), &at, TAU).unwrap();
    fat.validate().unwrap();
    check_mesh(&fat);
    let s = step(&[&fat], "fat", false).unwrap();
    assert_eq!(s.text.matches("SURFACE_OF_REVOLUTION").count(), 3);
    assert_eq!(s.approximated, 0);
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

// --- Meet (D227): where the generatrices of one pipe meet another cylinder ---

fn len(v: Vec3) -> f64 {
    v.dot(v).sqrt()
}

fn near(a: f64, b: f64, rel: f64) -> bool {
    (a - b).abs() <= rel * b.abs().max(1.0)
}

fn near3(a: Vec3, b: Vec3, rel: f64) -> bool {
    let scale = len(b).max(1.0);
    len(a - b) <= rel * scale
}

/// Distance of `p` from the line through `o` along the unit `a`.
fn from_line(p: Vec3, o: Vec3, a: Vec3) -> f64 {
    let w = p - o;
    len(w - a * w.dot(a))
}

const NEUTRAL: f64 = 10.0;

/// A branch of the development's own pipe (radius `NEUTRAL`, along z) meeting a wide cylinder
/// tilted against it; `own` ±1 chooses which way the generatrices run.
fn meet_own(own: f64) -> Meet {
    let tilt = Vec3::new(0.6, 0.0, 0.8);
    Meet { origin: Vec3::new(1.0, 3.0, 40.0 * own), axis: tilt, radius: 30.0, own, turn: None }
}

/// A bore of radius 8 tilted and offset, its generatrices towards the cylinder of radius 30
/// about z (the main pipe of the development).
fn meet_turn() -> Meet {
    let a = Vec3::new(-1.0, 0.1, 0.2);
    Meet { origin: Vec3::new(100.0, 5.0, 20.0), axis: a * (1.0 / len(a)), radius: 8.0, own: 30.0, turn: Some([0.0, TAU_TEST]) }
}

const TAU_TEST: f64 = std::f64::consts::TAU;

#[test]
fn meet_point_lies_on_both_cylinders() {
    for m in [meet_own(1.0), meet_own(-1.0)] {
        for k in 0..64 {
            let phi = TAU_TEST * f64::from(k) / 64.0 + 0.01;
            let p = m.point(phi, NEUTRAL)[0];
            assert!(near(from_line(p, Vec3::ZERO, Vec3::new(0.0, 0.0, 1.0)), NEUTRAL, 1e-9), "{phi}: {p:?}");
            assert!(near(from_line(p, m.origin, m.axis), m.radius, 1e-9), "{phi}: {p:?}");
        }
    }
    let m = meet_turn();
    for k in 0..64 {
        let phi = TAU_TEST * f64::from(k) / 64.0 + 0.01;
        let p = m.point(phi, NEUTRAL)[0];
        assert!(near(from_line(p, m.origin, m.axis), m.radius, 1e-9), "{phi}: {p:?}");
        assert!(near(from_line(p, Vec3::ZERO, Vec3::new(0.0, 0.0, 1.0)), m.own, 1e-9), "{phi}: {p:?}");
    }
}

/// The derivatives against central differences: the first of the point, the second of the first.
/// `h = 1e-5`: truncation `h²/6 · f‴ ≈ 1e-11`, rounding `1e-16 · |f| / h ≈ 1e-9` at worst.
#[test]
fn meet_point_derivatives_match_finite_differences() {
    let h = 1e-5;
    for m in [meet_own(1.0), meet_own(-1.0), meet_turn()] {
        for k in 0..32 {
            let phi = TAU_TEST * (f64::from(k) + 0.37) / 32.0;
            let [_, d1, d2] = m.point(phi, NEUTRAL);
            let (a, b) = (m.point(phi - h, NEUTRAL), m.point(phi + h, NEUTRAL));
            assert!(near3(d1, (b[0] - a[0]) * (0.5 / h), 1e-6), "d1 at {phi}: {d1:?}");
            assert!(near3(d2, (b[1] - a[1]) * (0.5 / h), 1e-6), "d2 at {phi}: {d2:?}");
        }
    }
}

fn dev() -> Development {
    Development { origin: Vec3::ZERO, axis: Vec3::new(0.0, 0.0, 1.0), start: Vec3::new(1.0, 0.0, 0.0), neutral: NEUTRAL }
}

/// The pieces of a Meet as the operations build them: one piece over `u` for a branch's end, four
/// quarter turns chained for a hole.
fn meet_pieces() -> Vec<Rolled> {
    let mut out = Vec::new();
    for m in [meet_own(1.0), meet_own(-1.0)] {
        let (u0, u1) = (0.3 * NEUTRAL, 5.5 * NEUTRAL);
        out.push(Rolled { dev: dev(), from: Vec2::new(u0, 0.0), to: Vec2::new(u1, 0.0), arc: None, trace: Some(Trace::Meet(m)) });
    }
    let quarter = std::f64::consts::FRAC_PI_2;
    let mut near_u = 0.0;
    for k in 0..4 {
        let trace = Trace::Meet(Meet { turn: Some([quarter * f64::from(k), quarter * f64::from(k + 1)]), ..meet_turn() });
        let mut p = Rolled { dev: dev(), from: Vec2::new(near_u, 0.0), to: Vec2::new(near_u, 0.0), arc: None, trace: Some(trace) };
        p.from = p.at(0.0).0;
        p.to = p.at(1.0).0;
        near_u = p.to.x;
        out.push(p);
    }
    out
}

#[test]
fn rolled_meet_at_and_second_match_finite_differences() {
    let h = 1e-5;
    for p in meet_pieces() {
        // The ends are at the stated `u`; `v` is the curve's own.
        let (s, e) = (p.at(0.0).0, p.at(1.0).0);
        assert!(near(s.x, p.from.x, 1e-12) && near(e.x, p.to.x, 1e-12));
        for k in 1..20 {
            let t = f64::from(k) / 20.0;
            let ((q, d), s2) = (p.at(t), p.second(t));
            let (a, b) = (p.at(t - h), p.at(t + h));
            let fd1 = Vec2::new((b.0.x - a.0.x) / (2.0 * h), (b.0.y - a.0.y) / (2.0 * h));
            let fd2 = Vec2::new((b.1.x - a.1.x) / (2.0 * h), (b.1.y - a.1.y) / (2.0 * h));
            let scale = d.x.abs().max(d.y.abs()).max(1.0);
            assert!((d.x - fd1.x).abs() <= 1e-6 * scale && (d.y - fd1.y).abs() <= 1e-6 * scale, "d at {t}: {d:?} vs {fd1:?} ({q:?})");
            let scale = s2.x.abs().max(s2.y.abs()).max(1.0);
            assert!((s2.x - fd2.x).abs() <= 1e-6 * scale && (s2.y - fd2.y).abs() <= 1e-6 * scale, "second at {t}: {s2:?} vs {fd2:?}");
        }
    }
}

/// The box holds 1000 samples, and each of its four bounds is reached: by the best sample
/// refined by a ternary search around it (samples a thousandth apart miss a turning point by
/// `h² · f″ ≈ 1e-6`; the search gets within rounding of it).
#[test]
fn rolled_meet_uv_box_holds_and_is_attained() {
    for p in meet_pieces() {
        let [bu, bv] = p.uv_box();
        let coord = |t: f64, axis: usize| {
            let q = p.at(t).0;
            if axis == 0 { q.x } else { q.y }
        };
        for axis in 0..2 {
            let b = if axis == 0 { bu } else { bv };
            let samples: Vec<f64> = (0..=1000).map(|k| coord(f64::from(k) / 1000.0, axis)).collect();
            for (k, &x) in samples.iter().enumerate() {
                assert!(x >= b[0] - 1e-9 && x <= b[1] + 1e-9, "axis {axis} sample {k}: {x} outside {b:?}");
            }
            for (bound, sign) in [(b[0], -1.0), (b[1], 1.0)] {
                let best = (0..=1000).max_by(|&i, &j| (sign * samples[i]).total_cmp(&(sign * samples[j]))).unwrap();
                let (mut lo, mut hi) = ((best.max(1) - 1) as f64 / 1000.0, ((best + 1).min(1000)) as f64 / 1000.0);
                if best == 0 {
                    lo = 0.0;
                }
                for _ in 0..200 {
                    let (m1, m2) = (lo + (hi - lo) / 3.0, hi - (hi - lo) / 3.0);
                    if sign * coord(m1, axis) < sign * coord(m2, axis) { lo = m1 } else { hi = m2 }
                }
                let reached = coord(0.5 * (lo + hi), axis);
                assert!((reached - bound).abs() <= 1e-9, "axis {axis}: bound {bound} reached only {reached}");
            }
        }
    }
}

#[test]
fn a_reflection_is_valid_and_twice_is_the_body_itself() {
    let b = l_prism();
    let r = b.reflected(Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0)).unwrap();
    r.validate().unwrap();
    assert_eq!(r.mass_properties().volume, b.mass_properties().volume);
    assert_eq!(r.bounds().0.x, -2.0);
    assert_ne!(r.hash(), b.hash());
    assert_eq!(r.reflected(Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0)).unwrap().hash(), b.hash());
}

proptest! {
    /// Reflected in any plane a body stays valid and keeps its volume; its IDs stay.
    #[test]
    fn a_reflection_keeps_the_body(
        (x, y, z) in (-1.0..1.0f64, -1.0..1.0f64, 0.1..1.0f64),
        origin in (-50.0..50.0f64, -50.0..50.0f64, -50.0..50.0f64),
    ) {
        let b = l_prism();
        let r = b.reflected(Vec3::new(origin.0, origin.1, origin.2), Vec3::new(x, y, z)).unwrap();
        prop_assert!(r.validate().is_ok());
        prop_assert!((r.mass_properties().volume - b.mass_properties().volume).abs() < 1e-9);
        prop_assert_eq!(r.faces().len(), b.faces().len());
        prop_assert!(r.faces().iter().zip(b.faces()).all(|(a, c)| a.id == c.id));
    }
}
