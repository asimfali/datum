//! Cost of exact predicates on implicit points (D38, D39): the `f64` filter path (general
//! position) against the exact path (a point on its own plane), on one thread and on all threads.
#![allow(clippy::expect_used)]

use criterion::{Criterion, criterion_group, criterion_main};
use datum_math::{Lpi, Point, Sign, Vec3, orient3d};
use rayon::prelude::*;

/// Implicit points with three more explicit points each.
fn cases(n: usize) -> Vec<(Point, [Point; 3], [Point; 3])> {
    let mut state = 0x9e37_79b9_7f4a_7c15u64;
    let mut next = || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        (state >> 11) as f64 / (1u64 << 53) as f64 * 200.0 - 100.0
    };
    let mut v = || Vec3::new(next(), next(), next());
    let mut out = Vec::with_capacity(n);
    while out.len() < n {
        let (p, q, r, s, t) = (v(), v(), v(), v(), v());
        if let Some(l) = Lpi::new(p, q, r, s, t) {
            out.push((Point::Lpi(l), [v(), v(), v()].map(Point::Explicit), [r, s, t].map(Point::Explicit)));
        }
    }
    out
}

fn predicates(c: &mut Criterion) {
    let all = std::thread::available_parallelism().map_or(1, |n| n.get());
    let data = cases(10_000);
    let mut group = c.benchmark_group("lpi_orient3d_10k");
    for threads in [1, all] {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().expect("thread pool");
        group.bench_function(format!("filter/threads={threads}"), |b| {
            b.iter(|| pool.install(|| data.par_iter().map(|(l, [a, b, c], _)| orient3d(l, a, b, c)).collect::<Vec<Sign>>()))
        });
        group.bench_function(format!("exact/threads={threads}"), |b| {
            b.iter(|| pool.install(|| data.par_iter().map(|(l, _, [r, s, t])| orient3d(r, s, t, l)).collect::<Vec<Sign>>()))
        });
    }
    group.finish();
}

criterion_group!(benches, predicates);
criterion_main!(benches);
