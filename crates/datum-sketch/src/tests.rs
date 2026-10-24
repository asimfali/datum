#![allow(clippy::unwrap_used, clippy::panic)]

use std::collections::BTreeMap;

use datum_math::Vec2;

use super::*;

fn sketch(json: &str) -> Sketch {
    serde_json::from_str(json).unwrap()
}

fn close(a: Vec2, b: [f64; 2], tol: f64) {
    assert!((a.x - b[0]).abs() <= tol && (a.y - b[1]).abs() <= tol, "{a:?} vs {b:?}");
}

/// A sloppily drawn rectangle with one corner fixed and two dimensions: fully defined.
fn rectangle(w: f64, h: f64, scale: f64) -> Sketch {
    let p = |x: f64, y: f64| Vec2::new(x * scale, y * scale);
    let mut s = Sketch {
        points: BTreeMap::from([("a".into(), p(0.0, 0.0)), ("b".into(), p(9.0, 0.7)), ("c".into(), p(10.3, 5.2)), ("d".into(), p(-0.4, 4.1))]),
        ..Sketch::default()
    };
    for (n, f, t) in [("bottom", "a", "b"), ("right", "b", "c"), ("top", "c", "d"), ("left", "d", "a")] {
        s.lines.insert(n.into(), Line { from: f.into(), to: t.into(), construction: false });
    }
    s.constraints = vec![
        Constraint::Fix { point: "a".into(), at: None },
        Constraint::Horizontal { line: Some("bottom".into()), points: None },
        Constraint::Horizontal { line: Some("top".into()), points: None },
        Constraint::Vertical { line: Some("left".into()), points: None },
        Constraint::Vertical { line: Some("right".into()), points: None },
        Constraint::Length { line: "bottom".into(), value: (w * scale).into() },
        Constraint::Length { line: "left".into(), value: (h * scale).into() },
    ];
    s
}

#[test]
fn rectangle_is_fully_defined() {
    let r = solve(&rectangle(12.0, 5.0, 1.0)).unwrap();
    assert_eq!((r.dof, r.free.len()), (0, 0));
    // The fixed corner keeps its drawn coordinates exactly.
    assert_eq!(r.points["a"], Vec2::new(0.0, 0.0));
    close(r.points["b"], [12.0, 0.0], 1e-12);
    close(r.points["c"], [12.0, 5.0], 1e-12);
    close(r.points["d"], [0.0, 5.0], 1e-12);
}

/// Principle 8: a bolt-sized and a site-sized sketch solve alike.
#[test]
fn scale_does_not_matter() {
    for scale in [1e-3, 1.0, 1e6] {
        let r = solve(&rectangle(12.0, 5.0, scale)).unwrap();
        assert_eq!(r.dof, 0, "scale {scale}");
        close(r.points["c"], [12.0 * scale, 5.0 * scale], 1e-11 * scale);
    }
}

#[test]
fn missing_constraints_leave_freedom() {
    let mut s = rectangle(12.0, 5.0, 1.0);
    s.constraints.retain(|c| !matches!(c, Constraint::Length { .. }));
    let r = solve(&s).unwrap();
    // Width and height are free; the fixed corner is not.
    assert_eq!(r.dof, 2);
    assert_eq!(r.free.keys().collect::<Vec<_>>(), ["b", "c", "d"]);
    // The bottom's far end slides along it; the top corners move both ways.
    assert_eq!((r.free["b"], r.free["c"]), (Some(Vec2::new(1.0, 0.0)), None));
    assert_eq!(r.free["d"], Some(Vec2::new(0.0, 1.0)));
    // Nothing moves more than the constraints need: the right side straightens halfway between
    // its drawn ends, the bottom levels out.
    close(r.points["b"], [9.65, 0.0], 1e-12);
    close(r.points["c"], [9.65, 4.65], 1e-12);
}

#[test]
fn conflicts_are_named() {
    let mut s = rectangle(12.0, 5.0, 1.0);
    s.constraints.push(Constraint::Length { line: "top".into(), value: 13.0.into() });
    let Err(SketchError::Conflict(names)) = solve(&s) else { panic!("expected a conflict") };
    // Either width, or the top's own length, or a constraint keeping them equal must go.
    assert!(names.iter().any(|n| n.contains("length (top = 13)")), "{names:?}");
    assert!(names.iter().any(|n| n.contains("length (bottom = 12)")), "{names:?}");
    assert!(!names.iter().any(|n| n.contains("fix")), "{names:?}");
}

#[test]
fn angle_and_distance_from_a_line() {
    let s = sketch(
        r#"{
        "points": {"o": [0, 0], "x": [10, 0.5], "p": [7, 6], "q": [3, 9]},
        "lines": {"base": {"from": "o", "to": "x"}, "arm": {"from": "o", "to": "p"}},
        "constraints": [
            {"type": "fix", "point": "o"},
            {"type": "horizontal", "line": "base"},
            {"type": "length", "line": "base", "value": 10},
            {"type": "angle", "lines": ["base", "arm"], "degrees": 30},
            {"type": "length", "line": "arm", "value": 8},
            {"type": "point_line", "point": "q", "line": "base", "value": 4},
            {"type": "distance", "points": ["q", "o"], "value": 5}
        ]
    }"#,
    );
    let r = solve(&s).unwrap();
    let (c, sn) = (30f64.to_radians().cos(), 30f64.to_radians().sin());
    close(r.points["p"], [8.0 * c, 8.0 * sn], 1e-12);
    // On the side it was drawn: above the base.
    close(r.points["q"], [3.0, 4.0], 1e-12);
    assert_eq!(r.dof, 0);
}

#[test]
fn bad_sketches_are_refused() {
    let invalid = |s: &str, why: &str| match solve(&sketch(s)) {
        Err(SketchError::Invalid(m)) => assert!(m.contains(why), "{m}"),
        r => panic!("expected `{why}`, got {r:?}"),
    };
    invalid(r#"{"points": {"a": [0, 0]}, "lines": {"l": {"from": "a", "to": "b"}}}"#, "unknown point `b`");
    invalid(r#"{"points": {"a": [0, 0], "b": [0, 0]}, "lines": {"l": {"from": "a", "to": "b"}}}"#, "zero length");
    invalid(r#"{"points": {"a": [0, 0]}, "constraints": [{"type": "horizontal", "line": "l"}]}"#, "unknown line `l`");
    invalid(
        r#"{"points": {"a": [0, 0], "b": [1, 0]}, "lines": {"l": {"from": "a", "to": "b"}}, "constraints": [{"type": "length", "line": "l", "value": -1}]}"#,
        "positive",
    );
    assert!(serde_json::from_str::<Sketch>(r#"{"points": {}, "shapes": []}"#).is_err());
}

/// C4: groups solve in parallel; the result is the same on any number of threads.
#[test]
fn same_result_on_any_thread_count() {
    let mut s = Sketch::default();
    for k in 0..12 {
        let mut r = rectangle(3.0 + f64::from(k), 2.0, 1.0);
        r.points = r.points.into_iter().map(|(n, p)| (format!("{n}{k}"), Vec2::new(p.x + 20.0 * f64::from(k), p.y))).collect();
        r.lines = r
            .lines
            .into_iter()
            .map(|(n, l)| (format!("{n}{k}"), Line { from: format!("{}{k}", l.from), to: format!("{}{k}", l.to), construction: false }))
            .collect();
        s.points.extend(r.points);
        s.lines.extend(r.lines);
        s.constraints.extend(r.constraints.into_iter().map(|c| match c {
            Constraint::Fix { point, at } => Constraint::Fix { point: format!("{point}{k}"), at },
            Constraint::Horizontal { line, points } => Constraint::Horizontal { line: line.map(|l| format!("{l}{k}")), points },
            Constraint::Vertical { line, points } => Constraint::Vertical { line: line.map(|l| format!("{l}{k}")), points },
            Constraint::Length { line, value } => Constraint::Length { line: format!("{line}{k}"), value },
            c => c,
        }));
    }
    let run = |t: usize| rayon::ThreadPoolBuilder::new().num_threads(t).build().unwrap().install(|| solve(&s).unwrap());
    let reference = run(1);
    for t in [2, 5, 16] {
        assert_eq!(run(t), reference);
    }
}

proptest::proptest! {
    #![proptest_config(proptest::prelude::ProptestConfig::with_cases(200))]

    /// Sloppy rectangles of any proportion solve exactly, wherever they were drawn.
    #[test]
    fn rectangles(w in 0.01..1000.0f64, h in 0.01..1000.0f64, jitter in proptest::array::uniform8(-0.2..0.2f64)) {
        let mut s = rectangle(1.0, 1.0, 1.0);
        let drawn = [(0.0, 0.0), (w, 0.0), (w, h), (0.0, h)];
        for (k, (n, p)) in s.points.iter_mut().enumerate() {
            let _ = n;
            *p = Vec2::new(drawn[k].0 + jitter[2 * k] * w, drawn[k].1 + jitter[2 * k + 1] * h);
        }
        s.points.insert("a".into(), Vec2::new(0.0, 0.0));
        for c in &mut s.constraints {
            match c {
                Constraint::Length { line, value } if line == "bottom" => *value = w.into(),
                Constraint::Length { value, .. } => *value = h.into(),
                _ => {}
            }
        }
        let r = solve(&s).unwrap();
        proptest::prop_assert_eq!(r.dof, 0);
        let tol = 1e-12 * w.max(h);
        close(r.points["c"], [w, h], tol);
    }

    /// A triangle from three sides: lengths hold, the base stays level, the apex keeps its side.
    #[test]
    fn triangles(a in 1.0..10.0f64, b in 1.0..10.0f64, t in 0.1..0.9f64) {
        // The third side, strictly between |a − b| and a + b.
        let c = (a - b).abs() + t * (a + b - (a - b).abs());
        let s = sketch(&format!(
            r#"{{"points": {{"o": [0, 0], "p": [{a}, 0.3], "q": [1, 2]}},
            "lines": {{"base": {{"from": "o", "to": "p"}}, "left": {{"from": "o", "to": "q"}}, "right": {{"from": "p", "to": "q"}}}},
            "constraints": [{{"type": "fix", "point": "o"}}, {{"type": "horizontal", "line": "base"}},
                {{"type": "length", "line": "base", "value": {a}}}, {{"type": "length", "line": "left", "value": {b}}},
                {{"type": "length", "line": "right", "value": {c}}}]}}"#
        ));
        let r = solve(&s).unwrap();
        let d = |m: &str, n: &str| ((r.points[m].x - r.points[n].x).powi(2) + (r.points[m].y - r.points[n].y).powi(2)).sqrt();
        for (len, want) in [(d("o", "p"), a), (d("o", "q"), b), (d("p", "q"), c)] {
            proptest::prop_assert!((len - want).abs() <= 1e-12 * a.max(b).max(c), "{} vs {}", len, want);
        }
        proptest::prop_assert!(r.points["q"].y > 0.0 && r.points["p"].y.abs() <= 1e-15);
    }
}

/// A slot drawn roughly: two lines tangent to two arcs, centres 10 apart, radius 2.5.
fn slot() -> Sketch {
    sketch(
        r#"{
        "points": {"l": [0, 0], "r": [9.7, 0.3], "a": [0.2, -2.3], "b": [9.6, -2.8], "c": [10.2, 2.1], "d": [-0.3, 2.6]},
        "lines": {"bottom": {"from": "a", "to": "b"}, "top": {"from": "c", "to": "d"}, "axis": {"from": "l", "to": "r", "construction": true}},
        "arcs": {"right": {"center": "r", "from": "b", "to": "c"}, "left": {"center": "l", "from": "d", "to": "a"}},
        "constraints": [
            {"type": "fix", "point": "l"},
            {"type": "horizontal", "line": "axis"},
            {"type": "length", "line": "axis", "value": 10},
            {"type": "radius", "arc": "right", "value": 2.5},
            {"type": "radius", "arc": "left", "value": 2.5},
            {"type": "tangent", "curves": ["bottom", "right"]},
            {"type": "tangent", "curves": ["top", "right"]},
            {"type": "tangent", "curves": ["top", "left"]},
            {"type": "tangent", "curves": ["left", "bottom"]}
        ]}"#,
    )
}

#[test]
fn slot_is_fully_defined() {
    let r = solve(&slot()).unwrap();
    assert_eq!(r.dof, 0, "{:?}", r.free);
    close(r.points["r"], [10.0, 0.0], 1e-12);
    close(r.points["a"], [0.0, -2.5], 1e-12);
    close(r.points["b"], [10.0, -2.5], 1e-12);
    close(r.points["c"], [10.0, 2.5], 1e-12);
    close(r.points["d"], [0.0, 2.5], 1e-12);
}

#[test]
fn circles_and_tangent_lines() {
    let s = sketch(
        r#"{
        "points": {"o": [0, 0], "a": [-8, -4.6], "b": [7, -5.3]},
        "lines": {"floor": {"from": "a", "to": "b"}},
        "circles": {"hole": {"center": "o", "diameter": "d"}},
        "constraints": [{"type": "fix", "point": "o"}, {"type": "horizontal", "line": "floor"}, {"type": "tangent", "curves": ["floor", "hole"]}]
        }"#,
    );
    let s = s.resolve(&|n| if n == "d" { Ok(10.0) } else { Err(datum_math::expr::ExprError::Unknown(n.into())) }).unwrap();
    let r = solve(&s).unwrap();
    // The line moves as little as it can: its ends straight down, give or take.
    assert!((r.points["a"].y + 5.0).abs() < 1e-12 && (r.points["b"].y + 5.0).abs() < 1e-12, "{:?}", r.points);
    // The line's ends slide along it: two degrees of freedom left.
    assert_eq!(r.dof, 2);
}

#[test]
fn arcs_and_circles_are_checked() {
    let bad = |json: &str| matches!(solve(&sketch(json)), Err(SketchError::Invalid(_)));
    assert!(bad(r#"{"points": {"c": [0, 0], "f": [1, 0]}, "arcs": {"x": {"center": "c", "from": "f", "to": "f"}}}"#));
    assert!(bad(r#"{"points": {"c": [0, 0]}, "circles": {"x": {"center": "c", "diameter": 0}}}"#));
    assert!(bad(r#"{"points": {"a": [0, 0], "b": [1, 0], "c": [0, 1]}, "lines": {"x": {"from": "a", "to": "b"}, "y": {"from": "a", "to": "c"}},
        "constraints": [{"type": "tangent", "curves": ["x", "y"]}]}"#));
}

/// A symmetric plate as drawings dimension it: horizontal and vertical distances from a base
/// corner, holes mirrored in the centre line, the centre line through the middle of the bottom.
#[test]
fn drawing_dimensions_and_symmetry() {
    let s = sketch(
        r#"{
        "points": {"o": [0, 0], "a": [41, 1], "b": [39, 22], "c": [-1, 19], "m": [21, 0.5], "t": [19, 25],
                   "h1": [9, 11], "h2": [29, 9]},
        "lines": {"bottom": {"from": "o", "to": "a"}, "right": {"from": "a", "to": "b"}, "top": {"from": "b", "to": "c"},
                  "left": {"from": "c", "to": "o"}, "axis": {"from": "m", "to": "t", "construction": true}},
        "circles": {"left_hole": {"center": "h1", "diameter": 6}, "right_hole": {"center": "h2", "diameter": 6}},
        "constraints": [
            {"type": "fix", "point": "o"},
            {"type": "horizontal", "line": "bottom"}, {"type": "horizontal", "line": "top"},
            {"type": "vertical", "line": "left"}, {"type": "vertical", "line": "right"},
            {"type": "horizontal_distance", "points": ["o", "a"], "value": 40},
            {"type": "vertical_distance", "points": ["o", "c"], "value": 20},
            {"type": "midpoint", "point": "m", "line": "bottom"},
            {"type": "on", "point": "t", "curve": "top"},
            {"type": "vertical", "line": "axis"},
            {"type": "symmetric", "points": ["h1", "h2"], "line": "axis"},
            {"type": "horizontal_distance", "points": ["h1", "h2"], "value": 24},
            {"type": "vertical_distance", "points": ["o", "h1"], "value": 10}
        ],
        "measures": {"to_edge": {"type": "horizontal_distance", "points": ["o", "h1"]}, "width": {"type": "length", "line": "top"}}
        }"#,
    );
    let r = solve(&s).unwrap();
    close(r.points["a"], [40.0, 0.0], 1e-12);
    close(r.points["b"], [40.0, 20.0], 1e-12);
    close(r.points["m"], [20.0, 0.0], 1e-12);
    close(r.points["h1"], [8.0, 10.0], 1e-12);
    close(r.points["h2"], [32.0, 10.0], 1e-12);
    // The vertical axis through the middle of the bottom meets the top edge at one point.
    close(r.points["t"], [20.0, 20.0], 1e-12);
    assert_eq!(r.dof, 0, "{:?}", r.free);
    assert!((r.measures["to_edge"] - 8.0).abs() < 1e-12 && (r.measures["width"] - 40.0).abs() < 1e-12, "{:?}", r.measures);
}

/// A point on an arc, on a circle and on the line through a segment beyond its ends.
#[test]
fn points_on_curves() {
    let s = sketch(
        r#"{
        "points": {"o": [0, 0], "f": [5, 0], "e": [0, 5], "p": [3.1, 4.4], "c": [20, 0], "q": [23.5, 0.4], "x": [10, 0.2], "y": [12, -0.1], "z": [15, 1]},
        "lines": {"l": {"from": "x", "to": "y"}},
        "arcs": {"quarter": {"center": "o", "from": "f", "to": "e"}},
        "circles": {"ring": {"center": "c", "diameter": 8}},
        "constraints": [
            {"type": "fix", "point": "o"}, {"type": "fix", "point": "f"}, {"type": "fix", "point": "c"},
            {"type": "fix", "point": "x"}, {"type": "fix", "point": "y"},
            {"type": "on", "point": "p", "curve": "quarter"},
            {"type": "on", "point": "q", "curve": "ring"},
            {"type": "on", "point": "z", "curve": "l"}
        ]}"#,
    );
    let r = solve(&s).unwrap();
    let d = |a: Vec2, b: Vec2| libm::hypot(a.x - b.x, a.y - b.y);
    assert!((d(r.points["p"], r.points["o"]) - 5.0).abs() < 1e-12);
    assert!((d(r.points["q"], r.points["c"]) - 4.0).abs() < 1e-12);
    let (x, y, z) = (r.points["x"], r.points["y"], r.points["z"]);
    assert!(((y.x - x.x) * (z.y - x.y) - (y.y - x.y) * (z.x - x.x)).abs() < 1e-12);
    // Each point slides along its curve; the arc's other end stays on the arc too.
    assert_eq!(r.dof, 4, "{:?}", r.free);
    let along = r.free["z"].unwrap();
    assert!((along.x * (y.y - x.y) - along.y * (y.x - x.x)).abs() < 1e-9, "{along:?}");
}

/// Arcs of one radius around one centre as a circle, and an arc as large as a circle.
#[test]
fn equal_radius_and_concentric() {
    let s = sketch(
        r#"{
        "points": {"o": [0, 0], "a": [4, 0.3], "b": [-0.2, 4.1], "c": [0.4, 0.3], "d": [-5.2, 0], "e": [0.1, -4.8], "k": [10, 0]},
        "arcs": {"first": {"center": "o", "from": "a", "to": "b"}, "second": {"center": "c", "from": "d", "to": "e"}},
        "circles": {"hole": {"center": "k", "diameter": 9}},
        "constraints": [
            {"type": "fix", "point": "o"}, {"type": "fix", "point": "k"},
            {"type": "concentric", "curves": ["first", "second"]},
            {"type": "equal_radius", "curves": ["second", "first"]},
            {"type": "equal_radius", "curves": ["hole", "first"]}
        ],
        "measures": {"r": {"type": "radius", "arc": "second"}}
        }"#,
    );
    let r = solve(&s).unwrap();
    close(r.points["c"], [0.0, 0.0], 1e-12);
    assert!((r.measures["r"] - 4.5).abs() < 1e-12, "{:?}", r.measures);
    let bad = |c: &str, why: &str| {
        let mut t = s.clone();
        t.circles.insert("other".into(), Circle { center: "o".into(), diameter: 3.0.into(), construction: false });
        t.constraints = serde_json::from_str(&format!("[{c}]")).unwrap();
        match solve(&t) {
            Err(SketchError::Invalid(m)) => assert!(m.contains(why), "{m}"),
            r => panic!("expected `{why}`, got {r:?}"),
        }
    };
    bad(r#"{"type": "equal_radius", "curves": ["hole", "other"]}, {"type": "equal_radius", "curves": ["other", "hole"]}"#, "of one diameter already");
    bad(r#"{"type": "concentric", "curves": ["hole", "nothing"]}"#, "unknown curve `nothing`");
}

/// Of two equal circles the second takes the first's diameter, through a chain too: in the
/// solver, and in the sketch resolved for the profile and the view.
#[test]
fn equal_circles_follow_the_first() {
    let s = sketch(
        r#"{
        "points": {"a": [0, 0], "b": [10, 0], "c": [20, 0]},
        "circles": {"one": {"center": "a", "diameter": "d"}, "two": {"center": "b", "diameter": 3}, "three": {"center": "c", "diameter": 7}},
        "constraints": [
            {"type": "fix", "point": "a"}, {"type": "fix", "point": "b"},
            {"type": "equal_radius", "curves": ["one", "two"]},
            {"type": "equal_radius", "curves": ["two", "three"]},
            {"type": "on", "point": "c", "curve": "two"}
        ]
        }"#,
    );
    let s = s.resolve(&|n| if n == "d" { Ok(4.0) } else { Err(datum_math::expr::ExprError::Unknown(n.into())) }).unwrap();
    assert!(s.circles.values().all(|c| c.diameter == Num::Value(4.0)), "{:?}", s.circles);
    let r = solve(&s).unwrap();
    close(r.points["c"], [12.0, 0.0], 1e-9);
}

/// A reference dimension measures and constrains nothing: the sketch keeps its freedom.
#[test]
fn measures_constrain_nothing() {
    let mut s = rectangle(12.0, 5.0, 1.0);
    s.constraints.retain(|c| !matches!(c, Constraint::Length { .. }));
    let free = solve(&s).unwrap();
    s.measures = BTreeMap::from([
        ("diagonal".into(), serde_json::from_str(r#"{"type": "distance", "points": ["a", "c"]}"#).unwrap()),
        ("corner".into(), serde_json::from_str(r#"{"type": "angle", "lines": ["bottom", "left"]}"#).unwrap()),
        ("height".into(), serde_json::from_str(r#"{"type": "point_line", "point": "d", "line": "bottom"}"#).unwrap()),
    ]);
    let r = solve(&s).unwrap();
    assert_eq!((r.dof, &r.points), (free.dof, &free.points));
    let (b, c) = (r.points["b"], r.points["c"]);
    assert!((r.measures["diagonal"] - libm::hypot(c.x, c.y)).abs() < 1e-12);
    assert!((r.measures["height"] - (c.y - b.y)).abs() < 1e-12);
    // From the bottom (→) to the left side (↓), counterclockwise: three quarters of a turn.
    assert!((r.measures["corner"] - 270.0).abs() < 1e-9, "{:?}", r.measures);
}

/// Horizontal and vertical take two points as well as a line: a centre level with a corner.
#[test]
fn points_level_and_plumb() {
    let s = sketch(
        r#"{"points": {"o": [0, 0], "c": [3.2, 4.1], "e": [7.9, 0.4]},
            "constraints": [{"type": "fix", "point": "o"}, {"type": "horizontal", "points": ["o", "e"]}, {"type": "vertical", "points": ["o", "c"]}]}"#,
    );
    let r = solve(&s).unwrap();
    assert!(r.points["e"].y.abs() < 1e-12 && r.points["c"].x.abs() < 1e-12, "{:?}", r.points);
    let bad = r#"{"points": {"o": [0, 0]}, "constraints": [{"type": "horizontal"}]}"#;
    assert!(matches!(solve(&sketch(bad)), Err(SketchError::Invalid(m)) if m.contains("a line or two points")));
}

/// An arc's length from its start to its end, counterclockwise: a quarter, three quarters of a
/// turn; measured too.
#[test]
fn arc_length() {
    let quarter = std::f64::consts::FRAC_PI_2 * 5.0;
    for (turns, drawn, end) in [(1.0, [0.6, 4.7], [0.0, 5.0]), (3.0, [-0.8, -4.6], [0.0, -5.0])] {
        let s = sketch(&format!(
            r#"{{
            "points": {{"o": [0, 0], "a": [5, 0], "b": [{}, {}]}},
            "arcs": {{"bend": {{"center": "o", "from": "a", "to": "b"}}}},
            "constraints": [{{"type": "fix", "point": "o"}}, {{"type": "fix", "point": "a"}}, {{"type": "arc_length", "arc": "bend", "value": {}}}],
            "measures": {{"along": {{"type": "arc_length", "arc": "bend"}}}}
            }}"#,
            drawn[0],
            drawn[1],
            turns * quarter
        ));
        let r = solve(&s).unwrap();
        assert_eq!(r.dof, 0, "{:?}", r.free);
        close(r.points["b"], end, 1e-12);
        assert!((r.measures["along"] - turns * quarter).abs() < 1e-12, "{:?}", r.measures);
    }
}

/// The angle an arc turns, from its start to its end counterclockwise: a quarter, three
/// quarters; measured in degrees too. Beyond (0°, 360°] it is refused.
#[test]
fn arc_angle() {
    for (degrees, drawn, end) in [(90.0, [0.6, 4.7], [0.0, 5.0]), (270.0, [-0.8, -4.6], [0.0, -5.0])] {
        let s = sketch(&format!(
            r#"{{
            "points": {{"o": [0, 0], "a": [5, 0], "b": [{}, {}]}},
            "arcs": {{"bend": {{"center": "o", "from": "a", "to": "b"}}}},
            "constraints": [{{"type": "fix", "point": "o"}}, {{"type": "fix", "point": "a"}}, {{"type": "radius", "arc": "bend", "value": 5}},
                            {{"type": "arc_angle", "arc": "bend", "degrees": {degrees}}}],
            "measures": {{"turn": {{"type": "arc_angle", "arc": "bend"}}}}
            }}"#,
            drawn[0], drawn[1]
        ));
        let r = solve(&s).unwrap();
        close(r.points["b"], end, 1e-12);
        assert!((r.measures["turn"] - degrees).abs() < 1e-9, "{:?}", r.measures);
    }
    let s = sketch(
        r#"{"points": {"o": [0, 0], "a": [5, 0], "b": [0, 5]}, "arcs": {"bend": {"center": "o", "from": "a", "to": "b"}},
            "constraints": [{"type": "arc_angle", "arc": "bend", "degrees": 400}]}"#,
    );
    assert!(matches!(solve(&s), Err(SketchError::Invalid(m)) if m.contains("up to 360")));
}
