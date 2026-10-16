//! C1 (wide graph scales with cores) and C2 (edit at the end of a long chain), each on one thread
//! and on all threads.
#![allow(clippy::expect_used)]

#[path = "../tests/common/mod.rs"]
mod common;

use common::{chain, turn, wide};
use criterion::{Criterion, criterion_group, criterion_main};
use datum_graph::Evaluator;
use datum_math::Vec3;
use rayon::ThreadPool;

fn pools() -> Vec<(String, ThreadPool)> {
    let all = std::thread::available_parallelism().map_or(1, |n| n.get());
    [1, all].into_iter().map(|n| (format!("threads={n}"), rayon::ThreadPoolBuilder::new().num_threads(n).build().expect("thread pool"))).collect()
}

fn wide_graph(c: &mut Criterion) {
    let g = wide(1000, 64);
    let mut group = c.benchmark_group("c1_wide_1000x64");
    for (name, pool) in pools() {
        group.bench_function(&name, |b| b.iter(|| pool.install(|| Evaluator::default().evaluate(&g))));
    }
    group.finish();
}

fn edit_last(c: &mut Criterion) {
    let mut group = c.benchmark_group("c2_chain_200_edit_last");
    for (name, pool) in pools() {
        let (mut g, last) = chain(200);
        let ev = Evaluator::default();
        pool.install(|| ev.evaluate(&g));
        let mut k = 0u64;
        group.bench_function(&name, |b| {
            b.iter(|| {
                k += 1;
                g.set_op(last, turn(k as f64 * 1e-3, Vec3::new(1.0, 0.0, 0.0))).expect("same arity");
                pool.install(|| ev.evaluate(&g))
            })
        });
    }
    group.finish();
}

criterion_group!(benches, wide_graph, edit_last);
criterion_main!(benches);
