//! Probes the louver's dome faces: which face fails to tessellate, whether its loop's points lie
//! on the surface the face declares, and the body's volume against the analytic one.
#![allow(clippy::unwrap_used, clippy::panic)]

use datum_brep::{Ix, Surface, TopoId, face_mesh, tessellate};
use datum_math::Vec3;
use datum_ops::{Op, Value};
use serde_json::json;

fn id(tag: &str) -> TopoId {
    TopoId::derive(tag).finish()
}

/// A plate 100 × 60 × 2 and a sketch on its top face with the placement line.
fn setup() -> (Value, Value) {
    let outline = json!({"op": "sketch", "sketch": {
        "points": {"a": [0, 0], "b": [100, 0], "c": [100, 60], "d": [0, 60]},
        "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}, "cd": {"from": "c", "to": "d"}, "da": {"from": "d", "to": "a"}}}});
    let plate = json!({"op": "plate", "thickness": 2});
    let sketch_op: Op = serde_json::from_value(outline).unwrap();
    let outline = sketch_op.eval(id("outline"), &[]).unwrap();
    let plate_op: Op = serde_json::from_value(plate).unwrap();
    let plate = plate_op.eval(id("plate"), &[&outline]).unwrap();
    let body = plate.body().unwrap();
    let top = body.faces().iter().find(|f| matches!(f.surface, Surface::Plane(p) if p.normal.z > 0.0)).map(|f| format!("{:?}", f.id)).unwrap();
    let placements = json!({"op": "sketch", "face": top, "sketch": {
        "points": {"a": [30, 30], "b": [80, 30]}, "lines": {"l": {"from": "a", "to": "b"}}}});
    let placements_op: Op = serde_json::from_value(placements).unwrap();
    let placed = placements_op.eval(id("placements"), &[&plate]).unwrap();
    (plate, placed)
}

/// The form-only louver: its die a shallow arc dipped 1.2 into a wall of thickness 2, the
/// component's thickness and radius put here (the rule would put them).
fn component() -> Op {
    let part = json!({
        "name": "Жалюзи", "kind": "stamp",
        "anchor": [0.0, 0.0, 0.0],
        "axis": [1.0, 0.0, 0.0],
        "thickness": [0.5, 3.0],
        "parameters": [{"name": "w", "value": 6.0}],
        "features": [
            {"name": "die", "inputs": [], "op": {"op": "sketch", "plane": {"origin": [0, 0, 0], "x_axis": [0, 1, 0], "normal": [1, 0, 0]}, "sketch": {
                "points": {"a": [-4.0, 0.0], "b": [0.0, -1.2], "c": [4.0, 0.0], "m": [0.0, 6.066666666666667], "e": [4.0, 0.5], "f": [-4.0, 0.5]},
                "arcs": {"dip": {"center": "m", "from": "a", "to": "c"}},
                "lines": {"right": {"from": "c", "to": "e"}, "back": {"from": "e", "to": "f"}, "left": {"from": "f", "to": "a"}},
                "constraints": [{"type": "on", "curve": "dip", "point": "b"}]
            }}},
            {"name": "body", "inputs": ["die"], "op": {"op": "extrude", "z0": 0.0, "z1": 6.0}}
        ],
        "roles": {"form": "body"}
    });
    serde_json::from_value(json!({
        "op": "component", "part": part,
        "thickness": 2.0, "radius": 3.0,
        "placements": [{"placement": "line", "input": 1, "line": "l"}]
    }))
    .unwrap()
}

#[test]
fn sweep_sanity() {
    let a = datum_brep::Arc { center: Vec3::new(0.0, 0.0, 0.0), axis: Vec3::new(0.0, 0.0, -1.0), radius: 1.0 };
    let (u, v) = (Vec3::new(0.0, -1.0, 0.0), Vec3::new(-1.0, 0.0, 0.0));
    println!("fwd {}", a.sweep(u, v).to_degrees());
    println!("rev {}", a.reversed().sweep(v, u).to_degrees());
    println!("rev-axis fwd {}", a.reversed().sweep(u, v).to_degrees());
}

#[test]
fn probe_dome_faces() {
    let (plate, placed) = setup();
    let op = component();
    let made = op.eval(id("punch"), &[&plate, &placed]);
    let formed = match made {
        Ok(Value::Formed(f)) => f,
        Ok(other) => panic!("not formed: {other:?}"),
        Err(e) => panic!("eval failed: {e}"),
    };
    let body = &formed.body;
    println!("volume {:.4}, shells {:?}", body.volume(), body.shape());
    for (k, f) in body.faces().iter().enumerate() {
        match face_mesh(body, k as Ix, false, body.chord()) {
            Ok(_) => {}
            Err(e) => {
                println!("FAIL face {k} {:?}: {e}", f.id);
                println!("  surface: {:?}", f.surface);
                for l in body.face_loops(k as Ix) {
                    for h in body.loop_half_edges(l) {
                        let he = &body.half_edges()[h as usize];
                        let e = &body.edges()[he.edge as usize];
                        let (u, v) = (body.point(he.origin), body.point(body.half_edges()[body.next(h) as usize].origin));
                        if let Some(c) = e.curve.map(|c| body.curve_specs()[c as usize].curve) {
                            let eff = if e.half == h { c } else { c.reversed() };
                            if let datum_brep::Curve::Arc(a) = eff {
                                println!(
                                    "    he {h} u {:?} v {:?} sweep {:.1}° eff axis {:?}",
                                    u.to_array(),
                                    v.to_array(),
                                    a.sweep(u, v).to_degrees(),
                                    a.axis.to_array()
                                );
                            } else {
                                println!("    he {h} u {:?} v {:?} straight", u.to_array(), v.to_array());
                            }
                        } else {
                            println!("    he {h} u {:?} v {:?} no curve", u.to_array(), v.to_array());
                        }
                    }
                }
            }
        }
    }
    println!("--- stored arcs with a sweep >= 180 deg (canonical direction)");
    for e in body.edges() {
        if let Some(c) = e.curve {
            let spec = body.curve_specs()[c as usize];
            if let datum_brep::Curve::Arc(a) = spec.curve {
                let he = &body.half_edges()[e.half as usize];
                let (u, v) = (body.point(he.origin), body.point(body.half_edges()[body.next(e.half) as usize].origin));
                let sw = a.sweep(u, v).to_degrees();
                if sw >= 180.0 {
                    println!("  sweep {sw:.1}° r {:.3} u {:?} v {:?}", a.radius, u.to_array(), v.to_array());
                }
            }
        }
    }
    match tessellate(body) {
        Ok(_) => println!("tessellate: ok"),
        Err(e) => println!("tessellate: {e}"),
    }
}
