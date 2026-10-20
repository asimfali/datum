//! C3 (PAINS P4): a 100 × 100 grid of pins cut through a plate in one Boolean operation, and the
//! tessellation of the drilled plate. Then the way a sheet is usually punched: square punches on
//! the axes, a plain difference, the punch as thick as the sheet (every face meets a face of the
//! sheet) or longer. Each on one thread and on all threads.
#![allow(clippy::expect_used)]

use criterion::{Criterion, criterion_group, criterion_main};
use datum_brep::{Body, TopoId, cuboid, tessellate};
use datum_math::{Rigid, Vec3};
use datum_ops::{BooleanKind, boolean, pattern_grid};
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

criterion_group!(benches, perforation, tessellate_drilled, punched);
criterion_main!(benches);
