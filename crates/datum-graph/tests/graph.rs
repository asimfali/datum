// Helpers outside #[test] functions are not covered by clippy.toml's test allowances.
#![allow(clippy::unwrap_used)]

mod common;

use common::{chain, ngon, prism, turn, wide};
use datum_brep::BrepError;
use datum_graph::{EvalError, Evaluator, Graph, GraphError, Op};
use datum_math::expr::Num;
use datum_math::{Vec2, Vec3};
use datum_ops::{BendPosition, BendRelief, Closing, Extent, FaceTo, Relief, WallTo};

fn hashes(g: &Graph, ev: &Evaluator) -> Vec<[u8; 32]> {
    ev.evaluate(g).results.into_iter().map(|r| r.unwrap().hash()).collect()
}

/// A flange on its sheet reaching a face of the body named `from` (D305).
fn flange_to_face(from: &str) -> Op {
    Op::Flange {
        edges: Vec::new(),
        height: Some(5.0.into()),
        height_along: false,
        angle: 90.0.into(),
        radius: None,
        flip: false,
        relief: Relief::Square,
        relief_diameter: None,
        k_factor: None,
        corner: Closing::Open,
        corner_gap: None,
        level_walls: false,
        extent: Extent::Edge,
        bend_relief: BendRelief::None,
        bend_relief_width: None,
        bend_relief_depth: None,
        bend_position: BendPosition::Outside,
        to: WallTo::default(),
        to_face: Some(FaceTo { face: "0123456789abcdef0123456789abcdef".into(), from: from.into(), gap: 0.0.into() }),
    }
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
    let identity = |axis| Op::Transform { translate: [0.0.into(), 0.0.into(), 0.0.into()], axis, angle_deg: 0.0.into() };
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
    let p = g.add("flat", Op::Prism { polygon: ngon(4, 1.0), z0: 1.0.into(), z1: 1.0.into() }, &[]).unwrap();
    g.add("moved", turn(10.0, Vec3::ZERO), &[p]).unwrap();
    let e = Evaluator::default().evaluate(&g);
    assert!(matches!(e.results[0], Err(EvalError::Op(BrepError::Degenerate(_)))));
    assert_eq!(e.results[1], Err(EvalError::Input("flat".into())));
}

/// A failed feature shows the last good state before it: the bodies it took, unless a feature
/// that did not fail uses them.
#[test]
fn a_failure_shows_the_last_good_state() {
    let bad = Op::Transform { translate: [0.0.into(), 0.0.into(), 0.0.into()], axis: Vec3::new(0.0, 0.0, 1.0), angle_deg: Num::Expr("nope".into()) };
    let mut g = Graph::default();
    let p = g.add("p", prism(ngon(4, 1.0)), &[]).unwrap();
    let a = g.add("a", bad.clone(), &[p]).unwrap();
    let b = g.add("b", turn(10.0, Vec3::ZERO), &[a]).unwrap();
    let q = g.add("q", prism(ngon(5, 1.0)), &[]).unwrap();
    let c = g.add("c", turn(10.0, Vec3::ZERO), &[q]).unwrap();
    let d = g.add("d", bad, &[q]).unwrap();
    let e = Evaluator::default().evaluate(&g);
    assert!(e.results[b as usize].is_err() && e.results[d as usize].is_err());
    // `b` failed through `a`: `p` stands for it; `q` goes on in `c`, so `d` shows nothing.
    assert_eq!(e.last_good(&g), [(p, b), (c, c)]);
}

/// A flange reads the face of another body, it does not take it apart (D305): the body stays what
/// the model ends in, as the body a work plane is on does.
#[test]
fn a_flange_to_a_face_leaves_the_body_it_reads_a_final() {
    let mut g = Graph::default();
    let sheet = g.add("sheet", prism(ngon(4, 10.0)), &[]).unwrap();
    let other = g.add("other", prism(ngon(4, 10.0)), &[]).unwrap();
    let f = g.add("flange", flange_to_face("other"), &[sheet, other]).unwrap();
    assert!(g.users()[other as usize].is_empty());
    assert_eq!(g.users()[sheet as usize], [f]);
    assert_eq!(g.finals(), [other, f]);
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
