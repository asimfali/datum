//! C3 (PAINS P4): a 100 × 100 grid of pins cut through a plate in one Boolean operation, and the
//! tessellation of the drilled plate. Each on one thread and on all threads.
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

criterion_group!(benches, perforation, tessellate_drilled);
criterion_main!(benches);
