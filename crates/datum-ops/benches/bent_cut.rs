//! A perforation across a bend (D197): a row of 50 holes of Ø5 along the bend line of an L of
//! sheet metal, cut through the blank in one operation, and the blank's development. Each on one
//! thread and on all threads.
#![allow(clippy::expect_used)]

use criterion::{Criterion, criterion_group, criterion_main};
use datum_brep::{Body, Surface, TopoId};
use datum_ops::{Op, Value, unfold_side};
use rayon::ThreadPool;

fn pools() -> Vec<(String, ThreadPool)> {
    let all = std::thread::available_parallelism().map_or(1, |n| n.get());
    [1, all].into_iter().map(|n| (format!("threads={n}"), rayon::ThreadPoolBuilder::new().num_threads(n).build().expect("thread pool"))).collect()
}

fn json(v: serde_json::Value) -> Op {
    serde_json::from_value(v).expect("op")
}

fn id(tag: &str) -> TopoId {
    TopoId::derive(tag).finish()
}

/// An L of sheet metal 2 thick, 520 wide: up 60, then 80 along X.
fn sheet() -> Body {
    let path = json(serde_json::json!({"op": "sketch", "sketch": {"points": {"a": [0, 0], "b": [0, 60], "c": [80, 60]},
        "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}}}}))
    .eval(id("path"), &[])
    .expect("path");
    let l = json(serde_json::json!({"op": "sheet", "thickness": 2, "radius": 2, "side": "left", "z0": 0, "z1": 520}));
    l.eval(id("sheet"), &[&path]).expect("sheet").body().expect("body").clone()
}

fn bent_perforation(c: &mut Criterion) {
    let body = sheet();
    // The outer face of the first wall: its sketch runs along the wall and down.
    let wall = body.faces().iter().find(|f| matches!(f.surface, Surface::Plane(p) if p.normal.x < -0.5)).expect("wall").id;
    let target = Value::Body(body);
    // The holes' centres 2 mm short of the bend line (58), at a pitch of 10.
    let sketch = json(serde_json::json!({"op": "sketch", "face": format!("{wall:?}"), "k_factor": 0.44,
        "sketch": {"points": {"c": [56, -5]}, "circles": {"hole": {"center": "c", "diameter": 5}},
            "patterns": {"row": {"of": "hole", "count": [1, 50], "step": [[10, 0], [0, -10]]}}}}))
    .eval(id("holes"), &[&target])
    .expect("sketch");
    let cut = json(serde_json::json!({"op": "extrude", "mode": "cut", "through": true, "wall": true}));
    let mut group = c.benchmark_group("cut_across_a_bend_50");
    group.sample_size(10);
    for (name, pool) in pools() {
        group.bench_function(&name, |b| b.iter(|| pool.install(|| cut.eval(id("cut"), &[&target, &sketch]).expect("cut"))));
    }
    group.finish();
    let cut = cut.eval(id("cut"), &[&target, &sketch]).expect("cut").body().expect("body").clone();
    let mut group = c.benchmark_group("unfold_across_a_bend_50");
    group.sample_size(10);
    for (name, pool) in pools() {
        group.bench_function(&name, |b| b.iter(|| pool.install(|| unfold_side(&cut, true, 0.44).expect("unfold"))));
    }
    group.finish();
}

criterion_group!(benches, bent_perforation);
criterion_main!(benches);
