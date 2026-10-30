//! Cuts through a rolled sheet in its development (M5c): holes and slots punched in the blank
//! before rolling, from either side, slanted slots, refusals, and principle 8.
#![allow(clippy::unwrap_used, clippy::panic)]

use std::f64::consts::{FRAC_PI_2, PI};

use datum_brep::{Body, BrepError, Surface, TopoId, tessellate};
use datum_math::{Rigid, Vec3};
use datum_ops::{Op, Value};

/// Inner radius, thickness and length of the sheet: a quarter of a turn.
const R: f64 = 500.0;
const T: f64 = 3.0;
const L: f64 = 400.0;

fn id(tag: &str) -> TopoId {
    TopoId::derive(tag).finish()
}

fn op(json: serde_json::Value) -> Op {
    serde_json::from_value(json).unwrap()
}

fn close(a: f64, b: f64, rel: f64) -> bool {
    (a - b).abs() <= rel * b.abs()
}

/// A quarter of a tube from `R` to `R + T` around the Z axis, `L` long.
fn sheet() -> Body {
    let profile = serde_json::json!({
        "points": {"o": [0, 0], "a": [R, 0], "b": [R + T, 0], "c": [0, R + T], "d": [0, R]},
        "lines": {"start": {"from": "a", "to": "b"}, "end": {"from": "c", "to": "d"}},
        "arcs": {"outer": {"center": "o", "from": "b", "to": "c"}, "inner": {"center": "o", "from": "a", "to": "d"}}
    });
    let s = op(serde_json::json!({"op": "sketch", "sketch": profile})).eval(id("profile"), &[]).unwrap();
    let v = op(serde_json::json!({"op": "extrude", "z0": 0, "z1": L})).eval(id("sheet"), &[&s]).unwrap();
    v.body().unwrap().clone()
}

/// The ID of the cylinder face of `b` facing out (`convex`) or in.
fn side(b: &Body, convex: bool) -> String {
    let f = b.faces().iter().find(|f| matches!(f.surface, Surface::Cylinder(c) if c.convex == convex)).unwrap();
    format!("{:?}", f.id)
}

/// `sketch` on face `on` of `b` with K-factor `k`, cut through by the feature `name`.
fn cut_by(name: &str, b: &Body, on: &str, k: f64, sketch: serde_json::Value, pattern: serde_json::Value) -> Result<Body, BrepError> {
    let target = Value::Body(b.clone());
    let placed = op(serde_json::json!({"op": "sketch", "face": on, "k_factor": k, "sketch": patterned(sketch, pattern)}))
        .eval(id(&format!("{name}.sketch")), &[&target])?;
    let c = serde_json::json!({"op": "extrude", "mode": "cut", "through": true});
    let r = op(c).eval(id(name), &[&target, &placed])?.body().unwrap().clone();
    r.validate().unwrap();
    Ok(r)
}

fn circle(x: f64, y: f64, d: f64) -> serde_json::Value {
    serde_json::json!({"points": {"h": [x, y]}, "circles": {"hole": {"center": "h", "diameter": d}}})
}

/// A hole of Ø10 at (20, 20) and a slot 20 × 6 beside it.
fn hole_and_slot() -> serde_json::Value {
    serde_json::json!({
        "points": {"h": [20, 20], "a": [30, 17], "b": [50, 17], "c": [50, 23], "d": [30, 23], "l": [30, 20], "r": [50, 20]},
        "lines": {"low": {"from": "a", "to": "b"}, "high": {"from": "c", "to": "d"}},
        "arcs": {"right": {"center": "r", "from": "b", "to": "c"}, "left": {"center": "l", "from": "d", "to": "a"}},
        "circles": {"hole": {"center": "h", "diameter": 10}}
    })
}

/// One hole and one slot: area in the development.
const PAIR: f64 = 25.0 * PI + 20.0 * 6.0 + 9.0 * PI;

/// Volume of the sheet less holes of `area` in the development with the neutral layer at `k`:
/// a hole takes `area · (R₂² − R₁²) / (2·Rₙ)`, the thickness itself at `k` = 0.5.
fn holed(area: f64, k: f64) -> f64 {
    let (r2, neutral) = ((R + T) * (R + T) - R * R, R + k * T);
    FRAC_PI_2 / 2.0 * r2 * L - area * r2 / (2.0 * neutral)
}

/// The mesh of every curved face of `b` keeps close to it: no triangle of a cylinder face strays
/// from the cylinder further than about two chords of 1/64 of a turn (none from a hole to the far
/// corners of the sheet, nine times deeper under the face), and its volume is the body's up to
/// the chords.
fn mesh_agrees(b: &Body) {
    for f in 0..b.faces().len() as u32 {
        let Surface::Cylinder(c) = b.faces()[f as usize].surface else { continue };
        let sag = 2.5 * c.radius * (1.0 - libm::cos(PI / 64.0));
        let m = datum_brep::face_mesh(b, f, false, b.chord()).unwrap();
        for t in &m.triangles {
            let g = (m.points[t[0] as usize] + m.points[t[1] as usize] + m.points[t[2] as usize]) * (1.0 / 3.0) - c.origin;
            let across = g - c.axis * g.dot(c.axis);
            let off = c.radius - across.dot(across).sqrt();
            assert!(off >= -1e-9 && off <= sag, "a triangle {off} under a face of radius {}", c.radius);
        }
    }
    let m = tessellate(b).unwrap();
    let o = m.positions[0];
    let v: f64 = m
        .triangles
        .iter()
        .map(|t| (m.positions[t[0] as usize] - o).dot((m.positions[t[1] as usize] - o).cross(m.positions[t[2] as usize] - o)))
        .sum::<f64>()
        / 6.0;
    assert!(close(v, b.volume(), 0.05), "mesh {v} vs body {}", b.volume());
}

#[test]
fn holes_and_slots_on_a_grid() {
    let s = sheet();
    let r = cut_by("cut", &s, &side(&s, true), 0.5, hole_and_slot(), serde_json::json!({"count": [4, 3], "step": [[45, 0], [0, 35]]})).unwrap();
    assert!(close(r.volume(), holed(12.0 * PAIR, 0.5), 1e-12), "{} vs {}", r.volume(), holed(12.0 * PAIR, 0.5));
    // Each copy: a circle (3 walls) and a slot (4 walls); the sheet keeps its 6 faces and IDs.
    assert_eq!(r.faces().len(), 6 + 12 * 7);
    assert_eq!(side(&r, true), side(&s, true));
    mesh_agrees(&r);
    // The area of a wall: the length of its piece in 3D times the thickness, close to that in
    // the development (a hole Ø10 is short beside the radius).
    let walls: f64 = (0..r.faces().len() as u32).filter(|&f| matches!(r.faces()[f as usize].surface, Surface::Wall(_))).map(|f| r.face_area(f)).sum();
    let perimeter = 12.0 * (10.0 * PI + 40.0 + 6.0 * PI);
    assert!(close(walls, perimeter * T, 1e-3), "{walls} vs {}", perimeter * T);
}

/// From inside, the sketch's Y runs against the axis (seen from the inside as on a plane face):
/// the hole at (100, −100) inside is the hole at (100, 100) outside, whichever face it is cut
/// from, and the K-factor stretches it the same.
#[test]
fn from_either_side_and_another_k_factor() {
    let s = sheet();
    let outside = cut_by("cut", &s, &side(&s, true), 0.3, circle(100.0, 100.0, 12.0), serde_json::Value::Null).unwrap();
    let inside = cut_by("cut", &s, &side(&s, false), 0.3, circle(100.0, -100.0, 12.0), serde_json::Value::Null).unwrap();
    let (a, b) = (outside.mass_properties(), inside.mass_properties());
    assert!(close(a.volume, holed(36.0 * PI, 0.3), 1e-12), "{} vs {}", a.volume, holed(36.0 * PI, 0.3));
    assert!(close(b.volume, a.volume, 1e-12));
    assert!((a.centroid - b.centroid).dot(a.centroid - b.centroid).sqrt() < 1e-9, "{:?} vs {:?}", a.centroid, b.centroid);
    // Then holes beside it from the other side: one development, one sheet.
    let both = cut_by("second", &outside, &side(&outside, false), 0.3, circle(130.0, -100.0, 12.0), serde_json::Value::Null).unwrap();
    assert!(close(both.volume(), holed(72.0 * PI, 0.3), 1e-12));
    mesh_agrees(&both);
    // Over the first hole, from inside: refused.
    let over = cut_by("over", &outside, &side(&outside, false), 0.3, circle(105.0, -100.0, 12.0), serde_json::Value::Null);
    assert!(matches!(over, Err(BrepError::Unsupported(_))), "{over:?}");
}

/// Slots "herringbone": slanted in the development, their sides are helices on the sheet.
#[test]
fn slanted_slots() {
    let (w, len) = (6.0, 30.0);
    let (c, s) = (libm::cos(PI / 4.0), libm::sin(PI / 4.0));
    let p = |x: f64, y: f64| [60.0 + c * x - s * y, 60.0 + s * x + c * y];
    let slot = serde_json::json!({
        "points": {"a": p(0.0, -w / 2.0), "b": p(len, -w / 2.0), "c": p(len, w / 2.0), "d": p(0.0, w / 2.0), "l": p(0.0, 0.0), "r": p(len, 0.0)},
        "lines": {"low": {"from": "a", "to": "b"}, "high": {"from": "c", "to": "d"}},
        "arcs": {"right": {"center": "r", "from": "b", "to": "c"}, "left": {"center": "l", "from": "d", "to": "a"}}
    });
    let sh = sheet();
    let r = cut_by("slots", &sh, &side(&sh, true), 0.5, slot, serde_json::json!({"count": [10, 5], "step": [[40, 0], [0, 50]]})).unwrap();
    let area = 50.0 * (len * w + PI * w * w / 4.0);
    assert!(close(r.volume(), holed(area, 0.5), 1e-12), "{} vs {}", r.volume(), holed(area, 0.5));
    mesh_agrees(&r);
}

#[test]
fn what_a_rolled_cut_refuses() {
    let s = sheet();
    let out = side(&s, true);
    // Across the sheet's edge, at u = 0.
    assert!(matches!(cut_by("c", &s, &out, 0.5, circle(3.0, 100.0, 10.0), serde_json::Value::Null), Err(BrepError::Unsupported(_))));
    // Past the end of the sheet in u: a quarter turn of the neutral layer is 789 long.
    assert!(matches!(cut_by("c", &s, &out, 0.5, circle(790.0, 100.0, 10.0), serde_json::Value::Null), Err(BrepError::Unsupported(_))));
    // Copies that overlap.
    let tight = serde_json::json!({"count": [2, 1], "step": [[8, 0], [0, 10]]});
    assert!(matches!(cut_by("c", &s, &out, 0.5, circle(30.0, 30.0, 10.0), tight), Err(BrepError::Degenerate(_))));
    // Without a K-factor, or with one off any face (on a plane face it lays bends flat, D197).
    let target = Value::Body(s.clone());
    let no_k = op(serde_json::json!({"op": "sketch", "face": out, "sketch": circle(30.0, 30.0, 10.0)})).eval(id("s"), &[&target]);
    assert!(matches!(no_k, Err(BrepError::Sketch(ref m)) if m.contains("k_factor")), "{no_k:?}");
    let k_off_face = op(serde_json::json!({"op": "sketch", "k_factor": 0.5, "sketch": circle(30.0, 30.0, 10.0)})).eval(id("s"), &[]);
    assert!(matches!(k_off_face, Err(BrepError::Sketch(_))), "{k_off_face:?}");
    // A tube has no edge to unroll from.
    let rings =
        serde_json::json!({"points": {"o": [0, 0]}, "circles": {"out": {"center": "o", "diameter": 106}, "in": {"center": "o", "diameter": 100}}});
    let tube = op(serde_json::json!({"op": "sketch", "sketch": rings})).eval(id("rings"), &[]).unwrap();
    let tube = Value::Body(op(serde_json::json!({"op": "extrude", "z0": 0, "z1": 50})).eval(id("tube"), &[&tube]).unwrap().body().unwrap().clone());
    let on = side(tube.body().unwrap(), true);
    let closed = op(serde_json::json!({"op": "sketch", "face": on, "k_factor": 0.5, "sketch": circle(30.0, 30.0, 10.0)})).eval(id("s"), &[&tube]);
    assert!(matches!(closed, Err(BrepError::Unsupported(_))), "{closed:?}");
}

/// Principle 8: turned by an odd angle and 10⁷ m (in mm) from the origin, the same holes, and the
/// volume up to the rounding of coordinates that far out (about ε·|p| each). The development's
/// `v` starts across the axis from the world origin: here, far along the axis.
#[test]
fn far_away_and_turned() {
    let m = Rigid::rotation(Vec3::new(1.0, 2.0, 3.0), 0.7).then(Rigid::translation(Vec3::new(1e10, -3e9, 2e9)));
    let s = sheet().transformed(&m);
    let target = Value::Body(s.clone());
    let empty = op(serde_json::json!({"op": "sketch", "face": side(&s, true), "k_factor": 0.5, "sketch": {}})).eval(id("frame"), &[&target]).unwrap();
    let d = **empty.sketch().unwrap().developed.as_ref().unwrap();
    let v0 = d.flat(m.apply(Vec3::new(R, 0.0, 0.0))).0.y;
    let r = cut_by("cut", &s, &side(&s, true), 0.5, circle(20.0, v0 + 20.0, 10.0), serde_json::json!({"count": [4, 3], "step": [[45, 0], [0, 35]]}))
        .unwrap();
    assert_eq!(r.faces().len(), 6 + 12 * 3);
    let far = 64.0 * f64::EPSILON * 1e10 / L;
    assert!(close(r.volume(), holed(12.0 * 25.0 * PI, 0.5), 1e-9 + far), "{} vs {}", r.volume(), holed(12.0 * 25.0 * PI, 0.5));
    mesh_agrees(&r);
}

/// The blank of `b` unfolded at `k` about face `on`.
fn unfold(b: &Body, on: &str, k: f64) -> Result<datum_ops::Flat, BrepError> {
    let v = op(serde_json::json!({"op": "unfold", "face": on, "k_factor": k})).eval(id("unfold"), &[&Value::Body(b.clone())])?;
    Ok(v.flat().unwrap().clone())
}

/// The blank is the neutral layer's rectangle, the holes where they were sketched: from outside
/// as sketched, from inside with Y turned back. Slots keep their lines and arcs; circles are one
/// entity each, as a punch. Unfolded about the inner face, the blank is seen from inside.
#[test]
fn the_flat_blank() {
    use datum_ops::FlatPiece;
    let s = sheet();
    let plain = unfold(&s, &side(&s, false), 0.44).unwrap();
    let width = FRAC_PI_2 * (R + 0.44 * T);
    assert!(close(plain.size.x, width, 1e-12) && close(plain.size.y, L, 1e-12), "{:?}", plain.size);
    assert_eq!(plain.counts(), [4, 0, 0]);

    let holes =
        cut_by("holes", &s, &side(&s, true), 0.44, circle(20.0, 20.0, 10.0), serde_json::json!({"count": [4, 3], "step": [[45, 0], [0, 35]]}))
            .unwrap();
    let both = cut_by("slots", &holes, &side(&holes, false), 0.44, hole_and_slot_below(), serde_json::Value::Null).unwrap();
    let flat = unfold(&both, &side(&both, true), 0.44).unwrap();
    // 4 sides; 12 holes and one inside; the slot's 2 lines and 2 arcs.
    assert_eq!(flat.counts(), [4 + 2, 2, 13]);
    let centres: Vec<(f64, f64)> = flat
        .pieces
        .iter()
        .filter_map(|p| match *p {
            FlatPiece::Circle { center, radius } => Some((center.x, center.y, radius)),
            _ => None,
        })
        .map(|(x, y, r)| {
            assert!(close(r, 5.0, 1e-12));
            (x, y)
        })
        .collect();
    for (i, j) in (0..4).flat_map(|i| (0..3).map(move |j| (i, j))) {
        let want = (20.0 + 45.0 * f64::from(i), 20.0 + 35.0 * f64::from(j));
        assert!(centres.iter().any(|c| (c.0 - want.0).abs() < 1e-9 && (c.1 - want.1).abs() < 1e-9), "no hole at {want:?}");
    }
    // Sketched from inside at (300, −200): at (300, 200) on the blank.
    assert!(centres.iter().any(|c| (c.0 - 300.0).abs() < 1e-9 && (c.1 - 200.0).abs() < 1e-9), "{centres:?}");
    // Seen from inside, the same blank mirrored: X runs the other way.
    let inside = unfold(&both, &side(&both, false), 0.44).unwrap();
    assert!(close(inside.size.x, flat.size.x, 1e-12) && close(inside.size.y, flat.size.y, 1e-12));
    for p in &inside.pieces {
        if let FlatPiece::Circle { center, .. } = *p {
            let mirrored = (flat.size.x - center.x, center.y);
            assert!(centres.iter().any(|c| (c.0 - mirrored.0).abs() < 1e-9 && (c.1 - mirrored.1).abs() < 1e-9), "{center:?}");
        }
    }
    // Another K-factor than the cuts': refused, not stretched.
    assert!(matches!(unfold(&both, &side(&both, true), 0.5), Err(BrepError::Unsupported(_))));

    let dxf = flat.dxf();
    assert!(dxf.starts_with("0\nSECTION\n2\nHEADER\n9\n$ACADVER\n1\nAC1009\n") && dxf.ends_with("0\nENDSEC\n0\nEOF\n"));
    assert_eq!((dxf.matches("\nCIRCLE\n").count(), dxf.matches("\nARC\n").count(), dxf.matches("\nLINE\n").count()), (13, 2, 6));
}

/// A hole Ø10 at (300, −200) and a slot 20 × 6 at (340…360, −200), sketched from inside.
fn hole_and_slot_below() -> serde_json::Value {
    serde_json::json!({
        "points": {"h": [300, -200], "a": [340, -203], "b": [360, -203], "c": [360, -197], "d": [340, -197], "l": [340, -200], "r": [360, -200]},
        "lines": {"low": {"from": "a", "to": "b"}, "high": {"from": "c", "to": "d"}},
        "arcs": {"right": {"center": "r", "from": "b", "to": "c"}, "left": {"center": "l", "from": "d", "to": "a"}},
        "circles": {"hole": {"center": "h", "diameter": 10}}
    })
}

/// `sketch` with `pattern` (JSON or null): `count` and `step` of the copies of all its curves.
fn patterned(mut sketch: serde_json::Value, pattern: serde_json::Value) -> serde_json::Value {
    if let serde_json::Value::Object(mut p) = pattern {
        let curves: Vec<String> =
            ["lines", "arcs", "circles"].iter().filter_map(|k| sketch[*k].as_object()).flat_map(|m| m.keys().cloned()).collect();
        p.insert("of".into(), serde_json::json!("cell"));
        sketch["groups"] = serde_json::json!({"cell": curves});
        sketch["patterns"] = serde_json::json!({"copies": p});
    }
    sketch
}
