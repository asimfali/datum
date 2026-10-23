//! C3 (PAINS P4): a 100 × 100 grid of pins cut through a plate in one Boolean operation, and the
//! tessellation of the drilled plate. Then the way a sheet is usually punched: square punches on
//! the axes, a plain difference, the punch as thick as the sheet (every face meets a face of the
//! sheet) or longer; and round holes of the same pitch cut by a sketch profile (M5b), in a flat
//! sheet and in a rolled one, in its development (M5c); a plate 1500 × 1000 filled with a cluster
//! of four Ø7.5 holes (D128), placed and extruded. Each on one thread and on all threads.
#![allow(clippy::expect_used)]

use criterion::{Criterion, criterion_group, criterion_main};
use datum_brep::{Body, TopoId, cuboid, tessellate};
use datum_math::{Rigid, Vec3};
use datum_ops::{BooleanKind, Op, Value, boolean, pattern_grid};
use rayon::ThreadPool;

fn pools() -> Vec<(String, ThreadPool)> {
    let all = std::thread::available_parallelism().map_or(1, |n| n.get());
    [1, all].into_iter().map(|n| (format!("threads={n}"), rayon::ThreadPoolBuilder::new().num_threads(n).build().expect("thread pool"))).collect()
}

/// A plate and an `n × n` grid of turned square pins as one tool.
fn plate_and_pins(n: u32) -> (Body, Body) {
    let id = |s: &str| TopoId::derive(s).finish();
    let side = 10.0 * f64::from(n);
    let plate = cuboid(id("plate"), Vec3::ZERO, Vec3::new(side, side, 5.0)).expect("plate");
    let pin = cuboid(id("pin"), Vec3::new(-2.0, -2.0, 0.0), Vec3::new(2.0, 2.0, 5.0))
        .expect("pin")
        .transformed(&Rigid::rotation(Vec3::new(0.0, 0.0, 1.0), 0.37).then(Rigid::translation(Vec3::new(5.0, 5.0, 0.0))));
    let pins = pattern_grid(id("grid"), &pin, [n, n], [Vec3::new(10.0, 0.0, 0.0), Vec3::new(0.0, 10.0, 0.0)]).expect("pattern");
    (plate, pins)
}

fn drill(plate: &Body, pins: &Body) -> Body {
    boolean(TopoId::derive("drill").finish(), BooleanKind::Difference, Some(Vec3::new(0.0, 0.0, 1.0)), &[plate, pins]).expect("boolean")
}

fn perforation(c: &mut Criterion) {
    let (plate, pins) = plate_and_pins(100);
    let mut group = c.benchmark_group("c3_perforation_10000");
    group.sample_size(10);
    for (name, pool) in pools() {
        group.bench_function(&name, |b| b.iter(|| pool.install(|| drill(&plate, &pins))));
    }
    group.finish();
}

/// A 3 mm sheet and an `n × n` grid of 5 mm square punches from `z0` to `z1`, as in
/// `examples/perforated_panel.json`.
fn sheet_and_punches(n: u32, z0: f64, z1: f64) -> (Body, Body) {
    let id = |s: &str| TopoId::derive(s).finish();
    let side = 10.0 * f64::from(n) + 20.0;
    let sheet = cuboid(id("sheet"), Vec3::ZERO, Vec3::new(side, side, 3.0)).expect("sheet");
    let punch = cuboid(id("punch"), Vec3::new(12.5, 12.5, z0), Vec3::new(17.5, 17.5, z1)).expect("punch");
    let punches = pattern_grid(id("grid"), &punch, [n, n], [Vec3::new(10.0, 0.0, 0.0), Vec3::new(0.0, 10.0, 0.0)]).expect("pattern");
    (sheet, punches)
}

fn punched(c: &mut Criterion) {
    for (group, z0, z1) in [("punch_flush_10000", 0.0, 3.0), ("punch_long_10000", -1.0, 4.0)] {
        let (sheet, punches) = sheet_and_punches(100, z0, z1);
        let mut group = c.benchmark_group(group);
        group.sample_size(10);
        for (name, pool) in pools() {
            group.bench_function(&name, |b| {
                b.iter(|| {
                    pool.install(|| boolean(TopoId::derive("punch").finish(), BooleanKind::Difference, None, &[&sheet, &punches]).expect("boolean"))
                })
            });
        }
        group.finish();
    }
}

/// The sheet of `sheet_and_punches` with an `n × n` grid of Ø5 holes at pitch 10, cut by one
/// circle of a sketch on its top face.
fn round_holes(c: &mut Criterion) {
    let n = 100;
    let side = 10.0 * f64::from(n) + 20.0;
    let sheet = Value::Body(cuboid(TopoId::derive("sheet").finish(), Vec3::ZERO, Vec3::new(side, side, 3.0)).expect("sheet"));
    let top = sheet.body().and_then(|b| b.faces().iter().find(|f| f.surface.plane().is_some_and(|p| p.normal.z > 0.0))).expect("top").id;
    let json = |v: serde_json::Value| serde_json::from_value::<Op>(v).expect("op");
    let sketch = json(
        serde_json::json!({"op": "sketch", "face": format!("{top:?}"), "sketch": {"points": {"c": [15, 15]}, "circles": {"hole": {"center": "c", "diameter": 5}},
            "patterns": {"grid": {"of": "hole", "count": [n, n], "step": [[10, 0], [0, 10]]}}}}),
    );
    let sketch = sketch.eval(TopoId::derive("holes").finish(), &[&sheet]).expect("sketch");
    let cut = json(serde_json::json!({"op": "cut"}));
    let mut group = c.benchmark_group("cut_round_10000");
    group.sample_size(10);
    for (name, pool) in pools() {
        group.bench_function(&name, |b| b.iter(|| pool.install(|| cut.eval(TopoId::derive("cut").finish(), &[&sheet, &sketch]).expect("cut"))));
    }
    group.finish();
}

/// The same grid of holes in a quarter of a tube 3 thick, inner radius 700, 1100 long, cut in its
/// development from outside.
fn rolled_holes(c: &mut Criterion) {
    let (r, t) = (700.0, 3.0);
    let json = |v: serde_json::Value| serde_json::from_value::<Op>(v).expect("op");
    let profile = json(serde_json::json!({"op": "sketch", "sketch": {
        "points": {"o": [0, 0], "a": [r, 0], "b": [r + t, 0], "c": [0, r + t], "d": [0, r]},
        "lines": {"start": {"from": "a", "to": "b"}, "end": {"from": "c", "to": "d"}},
        "arcs": {"outer": {"center": "o", "from": "b", "to": "c"}, "inner": {"center": "o", "from": "a", "to": "d"}}
    }}))
    .eval(TopoId::derive("profile").finish(), &[])
    .expect("profile");
    let sheet = json(serde_json::json!({"op": "extrude", "z0": 0, "z1": 1100})).eval(TopoId::derive("sheet").finish(), &[&profile]).expect("sheet");
    let outer =
        sheet.body().and_then(|b| b.faces().iter().find(|f| matches!(f.surface, datum_brep::Surface::Cylinder(c) if c.convex))).expect("outer").id;
    let sketch = json(serde_json::json!({"op": "sketch", "face": format!("{outer:?}"), "k_factor": 0.44,
        "sketch": {"points": {"c": [15, 15]}, "circles": {"hole": {"center": "c", "diameter": 5}},
            "patterns": {"grid": {"of": "hole", "count": [100, 100], "step": [[10, 0], [0, 10]]}}}}))
    .eval(TopoId::derive("holes").finish(), &[&sheet])
    .expect("sketch");
    let cut = json(serde_json::json!({"op": "cut"}));
    let mut group = c.benchmark_group("cut_rolled_10000");
    group.sample_size(10);
    for (name, pool) in pools() {
        group.bench_function(&name, |b| b.iter(|| pool.install(|| cut.eval(TopoId::derive("cut").finish(), &[&sheet, &sketch]).expect("cut"))));
    }
    group.finish();
}

fn tessellate_drilled(c: &mut Criterion) {
    let (plate, pins) = plate_and_pins(100);
    let drilled = drill(&plate, &pins);
    let mut group = c.benchmark_group("tessellate_10000_holes");
    group.sample_size(10);
    for (name, pool) in pools() {
        group.bench_function(&name, |b| b.iter(|| pool.install(|| tessellate(&drilled).expect("tessellate"))));
    }
    group.finish();
}

criterion_group!(benches, perforation, tessellate_drilled, punched, round_holes, rolled_holes, fill_cluster);
criterion_main!(benches);

/// A plate 1500 × 1000 filled by a cluster of four Ø7.5 holes at 19 × 17, 10 clear of its edge
/// (about 4500 punches, 18 000 holes): the sketch placed, then extruded.
fn fill_cluster(c: &mut Criterion) {
    let json = |v: serde_json::Value| serde_json::from_value::<Op>(v).expect("op");
    let sketch = json(serde_json::json!({"op": "sketch", "sketch": {
        "points": {"a": [0, 0], "b": [1500, 0], "c": [1500, 1000], "d": [0, 1000], "c1": [1600, 0], "c2": [1609.5, 0], "c3": [1604.75, 8.5], "c4": [1614.25, 8.5]},
        "lines": {"bottom": {"from": "a", "to": "b"}, "right": {"from": "b", "to": "c"}, "top": {"from": "c", "to": "d"}, "left": {"from": "d", "to": "a"}},
        "circles": {"f1": {"center": "c1", "diameter": 7.5}, "f2": {"center": "c2", "diameter": 7.5}, "f3": {"center": "c3", "diameter": 7.5}, "f4": {"center": "c4", "diameter": 7.5}},
        "groups": {"outline": ["bottom", "right", "top", "left"], "cluster": ["f1", "f2", "f3", "f4"]},
        "patterns": {"perforation": {"of": "cluster", "step": [[19, 0], [0, 17]], "fill": "outline", "clearance": 10}}
    }}));
    let extrude = json(serde_json::json!({"op": "extrude", "z0": 0, "z1": 1}));
    let mut group = c.benchmark_group("fill_cluster_plate");
    group.sample_size(10);
    for (name, pool) in pools() {
        group.bench_function(&name, |b| {
            b.iter(|| {
                pool.install(|| {
                    let s = sketch.eval(TopoId::derive("panel").finish(), &[]).expect("sketch");
                    extrude.eval(TopoId::derive("plate").finish(), &[&s]).expect("plate")
                })
            })
        });
    }
    group.finish();
}
