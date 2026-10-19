// Helpers outside #[test] functions are not covered by clippy.toml's test allowances.
#![allow(clippy::unwrap_used)]

mod common;

use common::{chain, ngon, prism, turn, wide};
use datum_brep::BrepError;
use datum_graph::{EvalError, Evaluator, Graph, GraphError, Op};
use datum_math::{Vec2, Vec3};

fn hashes(g: &Graph, ev: &Evaluator) -> Vec<[u8; 32]> {
    ev.evaluate(g).results.into_iter().map(|r| r.unwrap().hash()).collect()
}

#[test]
fn chain_evaluates_and_keeps_volume() {
    let (g, last) = chain(10);
    let e = Evaluator::default().evaluate(&g);
    assert_eq!((e.computed, e.reused), (11, 0));
    let volume = |r: &Result<datum_graph::ValueRef, _>| r.as_ref().unwrap().body().unwrap().volume();
    let (base, end) = (volume(&e.results[0]), volume(&e.results[last as usize]));
    assert!((end - base).abs() <= 1e-12 * base);
}

/// C2: an edit at the end of a long history recomputes one node.
#[test]
fn editing_the_last_node_recomputes_only_it() {
    let (mut g, last) = chain(200);
    let ev = Evaluator::default();
    ev.evaluate(&g);
    g.set_op(last, turn(0.75, Vec3::new(1.0, 0.0, 0.0))).unwrap();
    let e = ev.evaluate(&g);
    assert_eq!((e.computed, e.reused), (1, 200));
    // Undo: the old parameters are still cached.
    g.set_op(last, turn(0.5, Vec3::new(1.0, 0.0, 0.0))).unwrap();
    assert_eq!(ev.evaluate(&g).computed, 0);
}

/// Different parameters, identical body: nodes below are not recomputed.
#[test]
fn early_cutoff_stops_at_an_unchanged_body() {
    let identity = |axis| Op::Transform { translate: Vec3::ZERO, axis, angle_deg: 0.0 };
    let mut g = Graph::default();
    let p = g.add("p", prism(ngon(6, 1.0)), &[]).unwrap();
    let a = g.add("a", identity(Vec3::new(0.0, 0.0, 1.0)), &[p]).unwrap();
    g.add("b", turn(30.0, Vec3::ZERO), &[a]).unwrap();
    let ev = Evaluator::default();
    ev.evaluate(&g);
    g.set_op(a, identity(Vec3::new(1.0, 0.0, 0.0))).unwrap();
    let e = ev.evaluate(&g);
    assert_eq!((e.computed, e.reused), (1, 2));
}

#[test]
fn failures_name_their_cause() {
    let mut g = Graph::default();
    let p = g.add("flat", Op::Prism { polygon: ngon(4, 1.0), z0: 1.0, z1: 1.0 }, &[]).unwrap();
    g.add("moved", turn(10.0, Vec3::ZERO), &[p]).unwrap();
    let e = Evaluator::default().evaluate(&g);
    assert!(matches!(e.results[0], Err(EvalError::Op(BrepError::Degenerate(_)))));
    assert_eq!(e.results[1], Err(EvalError::Input("flat".into())));
}

#[test]
fn bad_graphs_are_refused() {
    let mut g = Graph::default();
    let p = g.add("p", prism(vec![Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0), Vec2::new(0.0, 1.0)]), &[]).unwrap();
    assert_eq!(g.add("p", prism(ngon(3, 1.0)), &[]), Err(GraphError::DuplicateName("p".into())));
    assert!(matches!(g.add("t", turn(1.0, Vec3::ZERO), &[]), Err(GraphError::Arity { .. })));
    assert!(matches!(g.add("t", turn(1.0, Vec3::ZERO), &[p + 1]), Err(GraphError::UnknownInput { .. })));
}

/// C4: bit-identical results for any thread count.
#[test]
fn results_do_not_depend_on_thread_count() {
    let g = wide(64, 24);
    let run = |threads| {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap();
        pool.install(|| hashes(&g, &Evaluator::default()))
    };
    let one = run(1);
    assert_eq!(run(4), one);
    assert_eq!(run(16), one);
}
