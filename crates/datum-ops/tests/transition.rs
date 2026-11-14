//! A faceted transition between two profiles on parallel planes (D264): the разметка (flat
//! triangles over the pieces, fans over the vertices), the bends of the rule's radius, and the
//! refusals. The body waits for the corner relief the author is settling, so the эталон's eval is
//! refused there; the layout's numbers are tested in the module.
#![allow(clippy::unwrap_used, clippy::panic)]

use datum_brep::TopoId;
use datum_math::Vec3;
use datum_ops::{Op, SketchPlane, Value};
use serde_json::{Value as Json, json};

fn id(tag: &str) -> TopoId {
    TopoId::derive(tag).finish()
}

fn op_of(j: Json) -> Op {
    serde_json::from_value(j).unwrap()
}

fn placed(sketch: Json, plane: Option<SketchPlane>) -> Value {
    op_of(json!({"op": "sketch", "sketch": sketch, "plane": plane})).eval(id("profile"), &[]).unwrap()
}

/// The эталон's first profile: an 80 × 80 square about the origin on XY.
fn square() -> Value {
    placed(
        json!({
            "points": {"a": [-40, -40], "b": [40, -40], "c": [40, 40], "d": [-40, 40]},
            "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"},
                      "cd": {"from": "c", "to": "d"}, "da": {"from": "d", "to": "a"}}
        }),
        None,
    )
}

/// The эталон's second profile: a circle of radius 90 about the origin, 120 above.
fn circle() -> Value {
    placed(
        json!({"points": {"o": [0, 0]}, "circles": {"c": {"center": "o", "diameter": 180}}}),
        Some(SketchPlane { origin: Vec3::new(0.0, 0.0, 120.0), x_axis: Vec3::new(1.0, 0.0, 0.0), normal: Vec3::new(0.0, 0.0, 1.0) }),
    )
}

/// The transition as the model writes it: the part's rule's numbers put in, as `built` does.
fn transition() -> Json {
    json!({"op": "transition", "a": "ab", "b": "c", "side": "left", "gap": 2.0, "tolerance": 1.0,
           "thickness": 2.0, "radius": 8.0, "k_factor": 0.44})
}

fn eval(op: Json, a: &Value, b: &Value) -> Result<Value, datum_brep::BrepError> {
    op_of(op).eval(id("transition"), &[a, b])
}

#[test]
fn the_op_is_a_transition_of_two_inputs() {
    let op = op_of(transition());
    assert_eq!(op.kind(), "transition");
    assert_eq!(op.arity(), 2..=2);
    // The tolerance defaults to half a millimetre; an expression resolves.
    let mut j = transition();
    j.as_object_mut().unwrap().remove("tolerance");
    let op = op_of(j);
    assert!(format!("{op:?}").contains("tolerance: Value(0.5)"));
    let mut j = transition();
    j["tolerance"] = json!("t / 2");
    let op = op_of(j);
    let r = op.resolve(&|n| if n == "t" { Ok(2.0) } else { Err(datum_math::expr::ExprError::Unknown(n.into())) }).unwrap();
    assert!(format!("{r:?}").contains("tolerance: Value(1.0)"));
}

#[test]
fn the_etalon_is_refused_at_the_corner() {
    let e = eval(transition(), &square(), &circle()).unwrap_err();
    let m = e.to_string();
    assert!(m.contains("the corner relief (a sector from the vertex, D264) is not built yet"), "{m}");
    // The layout is still made: 28 facets and 28 bends at tolerance 1 (D264).
    assert!(m.contains("28 facets"), "{m}");
    assert!(m.contains("28 bends"), "{m}");
}

#[test]
fn a_whole_turn_needs_a_gap() {
    let mut j = transition();
    j.as_object_mut().unwrap().remove("gap");
    let e = eval(j, &square(), &circle()).unwrap_err();
    assert!(e.to_string().contains("needs a `gap` at its seam"), "{e}");
    let mut j = transition();
    j["gap"] = json!(0.0);
    let e = eval(j, &square(), &circle()).unwrap_err();
    assert!(e.to_string().contains("needs a gap at its seam"), "{e}");
}

#[test]
fn the_rule_gives_thickness_radius_and_k() {
    for key in ["thickness", "radius", "k_factor"] {
        let mut j = transition();
        j.as_object_mut().unwrap().remove(key);
        let e = eval(j, &square(), &circle()).unwrap_err();
        assert!(e.to_string().contains("from the part's sheet rule"), "{key}: {e}");
    }
}

#[test]
fn planes_that_meet_are_refused() {
    let b = placed(
        json!({"points": {"o": [0, 0]}, "circles": {"c": {"center": "o", "diameter": 180}}}),
        Some(SketchPlane { origin: Vec3::new(0.0, 0.0, 120.0), x_axis: Vec3::new(1.0, 0.0, 0.0), normal: Vec3::new(0.0, 1.0, 0.0) }),
    );
    let e = eval(transition(), &square(), &b).unwrap_err();
    assert!(e.to_string().contains("parallel planes"), "{e}");
}

#[test]
fn an_arc_in_the_first_profile_is_refused() {
    let a = placed(
        json!({"points": {"a": [0, 0], "b": [80, 0], "c": [80, 80], "o": [0, 80]},
               "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}},
               "arcs": {"ca": {"center": "o", "from": "c", "to": "a"}}}),
        None,
    );
    let e = eval(transition(), &a, &circle()).unwrap_err();
    assert!(e.to_string().contains("first profile is straight pieces"), "{e}");
}

#[test]
fn a_bend_that_eats_a_facet_is_refused() {
    let mut j = transition();
    j["radius"] = json!(200.0);
    let e = eval(j, &square(), &circle()).unwrap_err();
    assert!(e.to_string().contains("eats facet"), "{e}");
}
