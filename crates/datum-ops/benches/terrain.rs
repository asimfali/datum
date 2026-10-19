//! C3 (PAINS P9): ground of 1000 × 1000 nodes (2 million triangles) and a pit, the excavated
//! volume as one Boolean intersection. On one thread and on all threads.
#![allow(clippy::expect_used)]

use criterion::{Criterion, criterion_group, criterion_main};
use datum_brep::{Body, Grid, TopoId, prism, terrain};
use datum_math::{Rigid, Vec2, Vec3};
use datum_ops::{BooleanKind, boolean};
use rayon::ThreadPool;

fn pools() -> Vec<(String, ThreadPool)> {
    let all = std::thread::available_parallelism().map_or(1, |n| n.get());
    [1, all].into_iter().map(|n| (format!("threads={n}"), rayon::ThreadPoolBuilder::new().num_threads(n).build().expect("thread pool"))).collect()
}

/// A smooth periodic bump from polynomials.
fn wave(t: f64) -> f64 {
    let u = t - 4.0 * (t / 4.0).floor() - 2.0;
    u * u * (2.0 - u.abs()) - 1.0
}

fn ground(n: u32) -> Body {
    let heights: Vec<f64> = (0..n)
        .flat_map(|j| (0..n).map(move |i| (f64::from(i), f64::from(j))))
        .map(|(x, y)| 120.0 + 3.0 * wave(x / 37.0) + 2.0 * wave(y / 29.0 + x / 97.0))
        .collect();
    terrain(TopoId::derive("ground").finish(), &Grid { origin: Vec2::new(0.0, 0.0), step: [1.0, 1.0], count: [n, n], heights: &heights }, 100.0)
        .expect("terrain")
}

fn excavation(c: &mut Criterion) {
    let g = ground(1000);
    let rect = [(-100.0, -75.0), (100.0, -75.0), (100.0, 75.0), (-100.0, 75.0)].map(|(x, y)| Vec2::new(x, y));
    let pit = prism(TopoId::derive("pit").finish(), &rect, 117.5, 140.0)
        .expect("pit")
        .transformed(&Rigid::rotation(Vec3::new(0.0, 0.0, 1.0), 0.23).then(Rigid::translation(Vec3::new(497.3, 503.1, 0.0))));
    let mut group = c.benchmark_group("c3_terrain_1000x1000_pit");
    group.sample_size(10);
    for (name, pool) in pools() {
        group.bench_function(&name, |b| {
            b.iter(|| pool.install(|| boolean(TopoId::derive("dig").finish(), BooleanKind::Intersection, None, &[&g, &pit]).expect("boolean")))
        });
    }
    group.finish();
}

criterion_group!(benches, excavation);
criterion_main!(benches);
