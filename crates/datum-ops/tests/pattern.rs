//! Grid pattern and through cuts (M2c): copies, stable IDs, contact checks, perforation.
#![allow(clippy::unwrap_used, clippy::panic)]

use datum_brep::{Body, BrepError, TopoId, cuboid, face_mesh, prism, tessellate, triangulate_face};
use datum_math::{Rigid, Sign, Vec2, Vec3, incircle};
use datum_ops::{BooleanKind, boolean, pattern_grid};

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

/// A 4 × 4 square pin from `z0` to `z1`, turned about Z so that no edge is axis-aligned.
fn pin(z0: f64, z1: f64) -> Body {
    cuboid(id("pin"), Vec3::new(-2.0, -2.0, z0), Vec3::new(2.0, 2.0, z1)).unwrap().transformed(&Rigid::rotation(Vec3::new(0.0, 0.0, 1.0), 0.37))
}

fn grid(source: &Body, count: [u32; 2], pitch: f64) -> Result<Body, BrepError> {
    pattern_grid(id("grid"), source, count, [Vec3::new(pitch, 0.0, 0.0), Vec3::new(0.0, pitch, 0.0)])
}

#[test]
fn copies_form_one_body_with_a_shell_each() {
    let p = pin(-1.0, 6.0);
    let g = checked(grid(&p, [4, 3], 10.0));
    assert_eq!(g.shell_vertices().len(), 12);
    assert_eq!(g.faces().len(), 12 * 6);
    assert_close(g.volume(), 12.0 * p.volume(), 1e-12);
}

/// Copy `(i, j)` is named by `(i, j)`, not by its position in the list: more columns keep the
/// existing copies' IDs (§4.2).
#[test]
fn more_columns_keep_existing_ids() {
    let p = pin(-1.0, 6.0);
    let ids = |b: &Body| b.faces().iter().map(|f| f.id).collect::<std::collections::BTreeSet<_>>();
    let (small, large) = (ids(&checked(grid(&p, [3, 2], 10.0))), ids(&checked(grid(&p, [5, 2], 10.0))));
    assert!(small.is_subset(&large));
    assert_eq!(large.len() - small.len(), 4 * 6);
}

/// Copies whose boxes overlap are checked exactly: diamonds on a diagonal grid miss each other,
/// on a square grid of pitch 2 they touch at their corners.
#[test]
fn touching_copies_are_refused_exactly() {
    let diamond = prism(id("d"), &[(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)].map(|(x, y)| Vec2::new(x, y)), 0.0, 1.0).unwrap();
    let diagonal = pattern_grid(id("g"), &diamond, [3, 3], [Vec3::new(1.5, 1.5, 0.0), Vec3::new(1.5, -1.5, 0.0)]);
    assert_close(checked(diagonal).volume(), 9.0 * 2.0, 1e-12);
    match grid(&diamond, [2, 1], 2.0) {
        Err(BrepError::Degenerate(m)) => assert!(m.contains("copies [0, 0] and [1, 0] touch"), "{m}"),
        r => panic!("expected touching copies, got {r:?}"),
    }
    assert!(matches!(grid(&diamond, [0, 3], 5.0), Err(BrepError::Degenerate(_))));
}

/// A tool nested in the second copy of a pattern is caught: every shell is tested, not only the
/// first one.
#[test]
fn a_tool_inside_any_copy_is_refused() {
    let plate = cuboid(id("plate"), Vec3::ZERO, Vec3::new(100.0, 100.0, 10.0)).unwrap();
    let cells = grid(&cuboid(id("cell"), Vec3::new(10.0, 10.0, 2.0), Vec3::new(20.0, 20.0, 8.0)).unwrap(), [2, 1], 30.0).unwrap();
    let inner = cuboid(id("inner"), Vec3::new(42.0, 12.0, 3.0), Vec3::new(48.0, 18.0, 7.0)).unwrap();
    match boolean(id("cut"), BooleanKind::Difference, None, &[&plate, &cells, &inner]) {
        Err(BrepError::Degenerate(m)) => assert!(m.contains("tools 1 and 2 intersect"), "{m}"),
        r => panic!("expected nested tools, got {r:?}"),
    }
}

/// Pins shorter than the plate cut through it with `through`, and still do when the plate grows
/// thicker: the hole keeps up with the target (D40).
#[test]
fn through_cuts_follow_the_target() {
    let pins = grid(&pin(1.0, 4.0), [5, 4], 10.0).unwrap();
    let section = pin(0.0, 1.0).volume();
    for thickness in [5.0, 12.0] {
        let plate = cuboid(id("plate"), Vec3::new(-5.0, -5.0, 0.0), Vec3::new(45.0, 35.0, thickness)).unwrap();
        let top = plate.faces().iter().find(|f| f.surface.plane().is_some_and(|p| p.normal.z > 0.0)).unwrap().id;
        let r = checked(boolean(id("cut"), BooleanKind::Difference, Some(Vec3::new(0.0, 0.0, 1.0)), &[&plate, &pins]));
        assert_close(r.volume(), plate.volume() - 20.0 * section * thickness, 1e-12);
        let f = r.faces().iter().position(|f| f.id == top).unwrap();
        assert_eq!(r.face_loops(f as u32).len(), 21);
    }
    // Without `through` the same pins leave closed cavities.
    let plate = cuboid(id("plate"), Vec3::new(-5.0, -5.0, 0.0), Vec3::new(45.0, 35.0, 5.0)).unwrap();
    let r = checked(boolean(id("cut"), BooleanKind::Difference, None, &[&plate, &pins]));
    assert_eq!(r.shell_vertices().len(), 21);
    assert!(matches!(boolean(id("cut"), BooleanKind::Difference, Some(Vec3::ZERO), &[&plate, &pins]), Err(BrepError::Degenerate(_))));
}

/// Principle 8: a through cut far from the origin and turned gives the same topology and volume.
#[test]
fn through_is_invariant_to_placement() {
    let pins = grid(&pin(0.0, 5.0), [3, 3], 10.0).unwrap();
    let plate = cuboid(id("plate"), Vec3::new(-5.0, -5.0, 0.0), Vec3::new(25.0, 25.0, 5.0)).unwrap();
    let cut = |m: &Rigid| {
        checked(boolean(
            id("cut"),
            BooleanKind::Difference,
            Some(m.apply_vector(Vec3::new(0.0, 0.0, 1.0))),
            &[&plate.transformed(m), &pins.transformed(m)],
        ))
    };
    let topology = |b: &Body| {
        let mut ids: Vec<_> = b.faces().iter().map(|f| f.id).chain(b.vertices().iter().map(|v| v.id)).collect();
        ids.sort();
        (ids, b.edges().len(), b.loops().len())
    };
    let reference = cut(&Rigid::IDENTITY);
    for shift in [1e3, 1e6, 1e7] {
        let m = Rigid::rotation(Vec3::new(0.3, -0.5, 0.8), 0.731).then(Rigid::translation(Vec3::new(shift, -0.7 * shift, 0.3 * shift)));
        let r = cut(&m);
        assert_eq!(topology(&r), topology(&reference), "shift {shift}");
        assert_close(r.volume(), reference.volume(), 1e-6);
    }
}

/// C4: the parallel stages give the same body, and the same error, on any number of threads.
#[test]
fn same_result_on_any_thread_count() {
    let pins = grid(&pin(0.0, 5.0), [12, 9], 10.0).unwrap();
    let plate = cuboid(id("plate"), Vec3::new(-5.0, -5.0, 0.0), Vec3::new(115.0, 85.0, 5.0)).unwrap();
    // A second tool that touches the pins: refused, with one message whatever the scheduling.
    let bar = cuboid(id("bar"), Vec3::new(-1.0, -1.0, -3.0), Vec3::new(90.0, 1.0, 8.0)).unwrap();
    let run = |threads: usize| {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap();
        pool.install(|| {
            let ok = boolean(id("cut"), BooleanKind::Difference, Some(Vec3::new(0.0, 0.0, 1.0)), &[&plate, &pins]).unwrap();
            let err = boolean(id("cut"), BooleanKind::Difference, None, &[&plate, &pins, &bar]).unwrap_err();
            (ok.hash(), err)
        })
    };
    let reference = run(1);
    for threads in [2, 3, 7, 16] {
        assert_eq!(run(threads), reference, "{threads} threads");
    }
}

/// The view's triangulation of a perforated face: the same area in as many triangles, no point
/// inside a neighbour's circumcircle, and none of the sweep's slivers across the plate.
#[test]
fn delaunay_removes_slivers() {
    let pins = grid(&pin(1.0, 4.0), [12, 10], 10.0).unwrap();
    let plate = cuboid(id("plate"), Vec3::new(-5.0, -5.0, 0.0), Vec3::new(115.0, 95.0, 5.0)).unwrap();
    let r = checked(boolean(id("cut"), BooleanKind::Difference, Some(Vec3::new(0.0, 0.0, 1.0)), &[&plate, &pins]));
    let f = r.faces().iter().position(|f| f.surface.plane().is_some_and(|p| p.normal.z > 0.0)).unwrap() as u32;
    let xy = |v: u32| {
        let p = r.point(v);
        Vec2::new(p.x, p.y)
    };
    let measure = |tris: &[[u32; 3]]| {
        tris.iter().fold((0.0, 0.0), |(area, perimeter), t| {
            let [a, b, c] = t.map(xy);
            let len = |p: Vec2, q: Vec2| ((q.x - p.x).powi(2) + (q.y - p.y).powi(2)).sqrt();
            (area + ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)) * 0.5, perimeter + len(a, b) + len(b, c) + len(c, a))
        })
    };
    let sweep = triangulate_face(&r, f).unwrap();
    let m = face_mesh(&r, f, true, 0.0).unwrap();
    let fine: Vec<[u32; 3]> = m.triangles.iter().map(|t| t.map(|k| m.vertices[k as usize])).collect();
    assert_eq!(fine.len(), sweep.len());
    let ((a0, p0), (a1, p1)) = (measure(&sweep), measure(&fine));
    assert_close(a1, a0, 1e-12);
    assert!(p1 * 2.0 < p0, "perimeter {p1} after flips, {p0} before");
    for (i, t) in fine.iter().enumerate() {
        for u in &fine[i + 1..] {
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                if let Some(j) = (0..3).find(|&j| u[j] == b && u[(j + 1) % 3] == a) {
                    let d = u[(j + 2) % 3];
                    assert_ne!(incircle(xy(a), xy(b), xy(t[(k + 2) % 3]), xy(d)), Sign::Positive);
                }
            }
        }
    }
}
