//! Graphs shared by tests and benchmarks.
#![allow(dead_code)]

use datum_graph::{Graph, NodeIx, Op};
use datum_math::{Vec2, Vec3};

/// Regular polygon with `n` vertices.
pub fn ngon(n: usize, r: f64) -> Vec<Vec2> {
    (0..n)
        .map(|k| {
            let a = std::f64::consts::TAU * k as f64 / n as f64;
            Vec2::new(r * a.cos(), r * a.sin())
        })
        .collect()
}

pub fn prism(polygon: Vec<Vec2>) -> Op {
    Op::Prism { polygon, z0: 0.0, z1: 10.0 }
}

pub fn turn(angle_deg: f64, translate: Vec3) -> Op {
    Op::Transform { translate, axis: Vec3::new(0.3, -0.5, 0.8), angle_deg }
}

/// C1: `n` independent bodies, each a prism over a `sides`-gon, then moved.
pub fn wide(n: usize, sides: usize) -> Graph {
    let mut g = Graph::default();
    for k in 0..n {
        let p = g.add(&format!("p{k}"), prism(ngon(sides, 1.0 + k as f64 * 0.01)), &[]).unwrap_or(0);
        g.add(&format!("t{k}"), turn(k as f64, Vec3::new(k as f64 * 30.0, 0.0, 0.0)), &[p]).ok();
    }
    g
}

/// C2: one prism, then `len` transforms in a chain. Returns the graph and its last node.
pub fn chain(len: usize) -> (Graph, NodeIx) {
    let mut g = Graph::default();
    let mut last = g.add("base", prism(ngon(16, 5.0)), &[]).unwrap_or(0);
    for k in 0..len {
        last = g.add(&format!("t{k}"), turn(0.5, Vec3::new(1.0, 0.0, 0.0)), &[last]).unwrap_or(0);
    }
    (g, last)
}
