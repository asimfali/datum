//! Sheet metal from a sketched path (M6): bends at the corners, either side of the path, the flat
//! blank, refusals.
#![allow(clippy::unwrap_used, clippy::panic)]

use std::f64::consts::{FRAC_PI_2, FRAC_PI_4};

use datum_brep::{Body, BrepError, Surface, TopoId, tessellate};
use datum_ops::{Op, Value};

fn id(tag: &str) -> TopoId {
    TopoId::derive(tag).finish()
}

fn op(json: serde_json::Value) -> Op {
    serde_json::from_value(json).unwrap()
}

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-9 * b.abs().max(1.0)
}

/// The panel's profile: a lip of 20, a slope at 45° rising 30, a top of 500, a flange of 15 down.
fn panel_path() -> serde_json::Value {
    serde_json::json!({
        "points": {"a": [0, 0], "b": [20, 0], "c": [50, 30], "d": [550, 30], "e": [550, 15]},
        "lines": {"lip": {"from": "a", "to": "b"}, "slope": {"from": "b", "to": "c"}, "top": {"from": "c", "to": "d"}, "flange": {"from": "d", "to": "e"}}
    })
}

fn sheet_of(path: serde_json::Value, side: &str, t: f64, r: f64, width: f64) -> Result<Body, BrepError> {
    let s = op(serde_json::json!({"op": "sketch", "sketch": path})).eval(id("path"), &[])?;
    let v = op(serde_json::json!({"op": "sheet", "thickness": t, "radius": r, "side": side, "z0": 0, "z1": width})).eval(id("sheet"), &[&s])?;
    let b = v.body().unwrap().clone();
    b.validate().unwrap();
    tessellate(&b).unwrap();
    Ok(b)
}

/// The volume of the panel: the path's lines shortened at the bends, each bend a ring sector.
fn panel_volume(side: &str, t: f64, r: f64, width: f64) -> f64 {
    let (lines, turns) = ([20.0, 30.0 * std::f64::consts::SQRT_2, 500.0, 15.0], [(FRAC_PI_4, true), (FRAC_PI_4, false), (FRAC_PI_2, false)]);
    // The path's radius at a bend: the inner face is `r` from the centre.
    let (d0, d1) = match side {
        "left" => (0.0, t),
        "right" => (-t, 0.0),
        _ => (-0.5 * t, 0.5 * t),
    };
    let mut straight: f64 = lines.iter().sum();
    let mut bends = 0.0;
    for (phi, ccw) in turns {
        let rp = r + if ccw { d1 } else { -d0 };
        straight -= 2.0 * rp * libm::tan(0.5 * phi);
        bends += 0.5 * phi * ((r + t) * (r + t) - r * r);
    }
    (straight * t + bends) * width
}

#[test]
fn a_panel_on_either_side() {
    for side in ["left", "right", "middle"] {
        let b = sheet_of(panel_path(), side, 1.5, 2.0, 1000.0).unwrap();
        let want = panel_volume(side, 1.5, 2.0, 1000.0);
        assert!(close(b.volume(), want), "{side}: {} vs {want}", b.volume());
        // Four lines and three bends on each face, two ends, two caps.
        let cylinders = b.faces().iter().filter(|f| matches!(f.surface, Surface::Cylinder(_))).count();
        assert_eq!((b.faces().len(), cylinders), (2 * 4 + 2 * 3 + 2 + 2, 6), "{side}");
    }
}

/// The face of `b` facing along `n` furthest that way, as a sketch names it.
fn face_along(b: &Body, n: datum_math::Vec3) -> String {
    let f = b
        .faces()
        .iter()
        .filter(|f| f.surface.plane().is_some_and(|p| p.normal.cross(n) == datum_math::Vec3::ZERO && p.normal.dot(n) > 0.0))
        .max_by(|x, y| x.surface.plane().unwrap().origin.dot(n).total_cmp(&y.surface.plane().unwrap().origin.dot(n)))
        .unwrap();
    format!("{:?}", f.id)
}

/// `sketch` on face `on` of `b` cut through it.
fn cut(b: &Body, on: &str, sketch: serde_json::Value, extra: serde_json::Value) -> Body {
    let target = Value::Body(b.clone());
    let mut s = serde_json::json!({"op": "sketch", "face": on, "sketch": sketch});
    if let (Some(s), Some(e)) = (s.as_object_mut(), extra.as_object()) {
        s.extend(e.clone());
    }
    let placed = op(s).eval(id(&format!("{on}.sketch")), &[&target]).unwrap();
    let r = op(serde_json::json!({"op": "cut"})).eval(id(&format!("{on}.cut")), &[&target, &placed]).unwrap();
    r.body().unwrap().clone()
}

fn circle(x: f64, y: f64, d: f64) -> serde_json::Value {
    serde_json::json!({"points": {"h": [x, y]}, "circles": {"hole": {"center": "h", "diameter": d}}})
}

/// The panel laid flat, seen from its outside (the top): bends by their neutral length, holes
/// apart by their distance along the sheet, the outline four lines.
#[test]
fn the_panel_laid_flat() {
    use datum_ops::FlatPiece;
    let (t, r, k) = (1.5, 2.0, 0.44);
    let b = sheet_of(panel_path(), "left", t, r, 1000.0).unwrap();
    let up = datum_math::Vec3::new(0.0, 1.0, 0.0);
    // The top's outer face is the highest facing up; the lip's, below it. On a face facing up
    // the sketch's X is the world's, its Y against Z.
    let top = face_along(&b, up);
    let b = cut(&b, &top, circle(300.0, -500.0, 8.0), serde_json::json!({}));
    let lip = b
        .faces()
        .iter()
        .filter(|f| f.surface.plane().is_some_and(|p| p.normal.cross(up) == datum_math::Vec3::ZERO && p.normal.dot(up) > 0.0))
        .min_by(|x, y| x.surface.plane().unwrap().origin.y.total_cmp(&y.surface.plane().unwrap().origin.y))
        .unwrap();
    let lip = format!("{:?}", lip.id);
    let b = cut(&b, &lip, circle(8.0, -500.0, 6.0), serde_json::json!({}));
    b.validate().unwrap();

    let v = op(serde_json::json!({"op": "unfold", "face": top, "k_factor": k})).eval(id("blank"), &[&Value::Body(b.clone())]).unwrap();
    let flat = v.flat().unwrap();
    // Straight lengths less the bends' set-backs, and each bend by its neutral arc.
    let turns = [(std::f64::consts::FRAC_PI_4, true), (std::f64::consts::FRAC_PI_4, false), (std::f64::consts::FRAC_PI_2, false)];
    let back = |(phi, ccw): (f64, bool)| (r + if ccw { t } else { 0.0 }) * libm::tan(0.5 * phi);
    let straight = 20.0 + 30.0 * std::f64::consts::SQRT_2 + 500.0 + 15.0 - turns.iter().map(|&b| 2.0 * back(b)).sum::<f64>();
    let neutral: Vec<f64> = turns.iter().map(|&(phi, _)| phi * (r + k * t)).collect();
    let width = straight + neutral.iter().sum::<f64>();
    assert!(close(flat.size.x, width) && close(flat.size.y, 1000.0), "{:?} vs {width}", flat.size);
    assert_eq!(flat.counts(), [4, 0, 2]);
    let centres: Vec<(f64, f64)> = flat
        .pieces
        .iter()
        .filter_map(|p| match *p {
            FlatPiece::Circle { center, .. } => Some((center.x, center.y)),
            _ => None,
        })
        .collect();
    // From the lip's hole to the top's along the sheet: the rest of the lip, the first bend,
    // the slope, the second bend, the top up to x = 300.
    let along = (20.0 - back(turns[0]) - 8.0)
        + neutral[0]
        + (30.0 * std::f64::consts::SQRT_2 - back(turns[0]) - back(turns[1]))
        + neutral[1]
        + (300.0 - (50.0 + back(turns[1])));
    assert!((centres[0].0 - centres[1].0).abs() - along < 1e-9 && (centres[0].0 - centres[1].0).abs() - along > -1e-9, "{centres:?}, {along}");
    assert!(centres.iter().all(|c| (c.1 - 500.0).abs() < 1e-9), "{centres:?}");
}

/// Dimensions and the side change the body, not its faces' names; a path the sheet cannot follow
/// is refused with the reason.
#[test]
fn names_stay_and_bad_paths_are_refused() {
    let ids = |b: &Body| b.faces().iter().map(|f| f.id).collect::<Vec<_>>();
    let a = sheet_of(panel_path(), "left", 1.5, 2.0, 1000.0).unwrap();
    let b = sheet_of(panel_path(), "right", 3.0, 5.0, 800.0).unwrap();
    assert_eq!(ids(&a), ids(&b));
    assert_ne!(a.hash(), b.hash());

    let refused = |path: serde_json::Value, r: f64| sheet_of(path, "left", 1.5, r, 100.0).unwrap_err();
    let lines = |pts: serde_json::Value, ls: serde_json::Value| serde_json::json!({"points": pts, "lines": ls});
    // A branch, a closed path, a bend too big for its lines, two lines in one.
    let branch = lines(
        serde_json::json!({"a": [0, 0], "b": [10, 0], "c": [10, 10], "d": [20, 0]}),
        serde_json::json!({"l1": {"from": "a", "to": "b"}, "l2": {"from": "b", "to": "c"}, "l3": {"from": "b", "to": "d"}}),
    );
    assert!(matches!(refused(branch, 2.0), BrepError::Sketch(m) if m.contains("branches")));
    let closed = lines(
        serde_json::json!({"a": [0, 0], "b": [10, 0], "c": [10, 10]}),
        serde_json::json!({"l1": {"from": "a", "to": "b"}, "l2": {"from": "b", "to": "c"}, "l3": {"from": "c", "to": "a"}}),
    );
    assert!(matches!(refused(closed, 2.0), BrepError::Unsupported(_)));
    assert!(matches!(refused(panel_path(), 30.0), BrepError::Degenerate(m) if m.contains("do not fit")));
    let straight = lines(
        serde_json::json!({"a": [0, 0], "b": [10, 0], "c": [30, 0]}),
        serde_json::json!({"l1": {"from": "a", "to": "b"}, "l2": {"from": "b", "to": "c"}}),
    );
    assert!(matches!(refused(straight, 2.0), BrepError::Degenerate(m) if m.contains("one line")));
    let rolled = serde_json::json!({"points": {"o": [0, 0], "a": [10, 0], "b": [0, 10]}, "arcs": {"r": {"center": "o", "from": "a", "to": "b"}}});
    assert!(matches!(refused(rolled, 2.0), BrepError::Unsupported(_)));
}

fn plate_with_hole() -> Body {
    let s = op(serde_json::json!({"op": "sketch", "sketch": {
        "points": {"a": [0, 0], "b": [100, 0], "c": [100, 60], "d": [0, 60], "h": [20, 25]},
        "lines": {"bottom": {"from": "a", "to": "b"}, "right": {"from": "b", "to": "c"}, "top": {"from": "c", "to": "d"}, "left": {"from": "d", "to": "a"}},
        "circles": {"hole": {"center": "h", "diameter": 10}}}}))
    .eval(id("plate_sketch"), &[])
    .unwrap();
    let b = op(serde_json::json!({"op": "extrude", "z0": 0, "z1": 3})).eval(id("plate"), &[&s]).unwrap().body().unwrap().clone();
    b.validate().unwrap();
    b
}

/// A plate's two sides, and a bent sheet's: the front seen as drawn, the back as its mirror;
/// the bend lines are guides, one pair for each bend, across the whole width; a cube is no sheet.
#[test]
fn a_sheets_sides_and_bend_lines() {
    use datum_ops::{FlatPiece, sheet_sides, unfold_side};
    let hole = |f: &datum_ops::Flat| {
        f.pieces.iter().find_map(|p| if let FlatPiece::Circle { center, .. } = p { Some((center.x, center.y)) } else { None }).unwrap()
    };
    let b = plate_with_hole();
    let sides = sheet_sides(&b).unwrap();
    let normal = |s: u32| b.faces()[s as usize].surface.plane().unwrap().normal;
    assert_eq!((normal(sides[0]).z, normal(sides[1]).z), (1.0, -1.0), "the front faces up");
    let (front, back) = (unfold_side(&b, false, 0.5).unwrap(), unfold_side(&b, true, 0.5).unwrap());
    assert_eq!((hole(&front), hole(&back)), ((20.0, 25.0), (80.0, 25.0)), "as drawn, and its mirror");
    assert!(front.bends.is_empty() && close(front.size.x, 100.0) && close(front.size.y, 60.0));
    assert_ne!(front.hash, back.hash);
    // Three bends, a tangent line each side of each, as long as the sheet is wide.
    let p = sheet_of(panel_path(), "left", 1.5, 2.0, 1000.0).unwrap();
    for back in [false, true] {
        let flat = unfold_side(&p, back, 0.44).unwrap();
        assert_eq!(flat.bends.len(), 6, "back: {back}");
        assert!(flat.bends.iter().all(|l| l[0].x == l[1].x && l[0].y == 0.0 && close(l[1].y, 1000.0)), "{:?}", flat.bends);
        let xs: Vec<f64> = flat.bends.iter().map(|l| l[0].x).collect();
        assert!(xs.windows(2).all(|w| w[0] < w[1]) && xs[0] > 0.0 && xs[5] < flat.size.x, "{xs:?}");
        // Seen from either side the blank is as long; DXF has no bend lines.
        assert!(close(flat.size.x, 577.2266941436376), "{:?}", flat.size);
        assert_eq!(flat.dxf().matches("LINE").count(), 4);
    }
    // A cube has no two sides standing out.
    let cube =
        op(serde_json::json!({"op": "prism", "polygon": [[0, 0], [10, 0], [10, 10], [0, 10]], "z0": 0, "z1": 10})).eval(id("cube"), &[]).unwrap();
    assert!(matches!(sheet_sides(cube.body().unwrap()), Err(BrepError::Unsupported(_))));
}
