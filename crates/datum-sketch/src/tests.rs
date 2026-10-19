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
        Constraint::Fix { point: "a".into() },
        Constraint::Horizontal { line: "bottom".into() },
        Constraint::Horizontal { line: "top".into() },
        Constraint::Vertical { line: "left".into() },
        Constraint::Vertical { line: "right".into() },
        Constraint::Length { line: "bottom".into(), value: w * scale },
        Constraint::Length { line: "left".into(), value: h * scale },
    ];
    s
}

#[test]
fn rectangle_is_fully_defined() {
    let r = solve(&rectangle(12.0, 5.0, 1.0)).unwrap();
    assert_eq!((r.dof, r.free_points.len()), (0, 0));
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
    assert_eq!(r.free_points, ["b", "c", "d"]);
    // Nothing moves more than the constraints need: the right side straightens halfway between
    // its drawn ends, the bottom levels out.
    close(r.points["b"], [9.65, 0.0], 1e-12);
    close(r.points["c"], [9.65, 4.65], 1e-12);
}

#[test]
fn conflicts_are_named() {
    let mut s = rectangle(12.0, 5.0, 1.0);
    s.constraints.push(Constraint::Length { line: "top".into(), value: 13.0 });
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
            Constraint::Fix { point } => Constraint::Fix { point: format!("{point}{k}") },
            Constraint::Horizontal { line } => Constraint::Horizontal { line: format!("{line}{k}") },
            Constraint::Vertical { line } => Constraint::Vertical { line: format!("{line}{k}") },
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
                Constraint::Length { line, value } if line == "bottom" => *value = w,
                Constraint::Length { value, .. } => *value = h,
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
