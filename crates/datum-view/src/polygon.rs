//! A regular polygon along a circle or arc: a circle or an arc selected, N, the number of sides
//! typed, Enter. A full circle gets `n` vertices on it, the sides between them closed; an arc's
//! length is cut into `n` equal pieces, `n + 1` vertices from its start to its end, the sides an
//! open chain. The circle or arc turns construction, and its radius still holds the vertices,
//! kept on it (`on`): changing the radius moves the polygon with it.

use serde_json::{Map, Value as Json, json};

use crate::draw::Patch;
use crate::sketch::{Pick, SketchEdit};
use crate::trim::Shape;

/// The circle or arc of the selection the polygon goes on: the first, in the order selected.
pub(crate) fn curve(edit: &SketchEdit) -> Option<String> {
    let sketch = &edit.json["sketch"];
    edit.selected.iter().find_map(|p| match p {
        Pick::Curve(n) if sketch["circles"].get(n).is_some() || sketch["arcs"].get(n).is_some() => Some(n.clone()),
        _ => None,
    })
}

/// The patch putting the polygon of `typed` sides (as typed and its value) on the selected curve;
/// none without a whole count of three or more, or with no circle or arc selected.
pub(crate) fn plan(edit: &SketchEdit, typed: &(Json, f64)) -> Option<Map<String, Json>> {
    let sides = typed.1;
    if sides.fract() != 0.0 || sides < 3.0 {
        return None;
    }
    let n = sides as usize;
    let name = curve(edit)?;
    let p = edit.placed.as_ref().ok()?;
    let s = Shape::of(p, &name)?;
    let (key, closed) = if p.circles.contains_key(&name) { ("circles", true) } else { ("arcs", false) };
    let mut patch = Patch::new(edit);
    // A circle: `n` vertices round it; an arc: `n + 1`, its length cut into `n`.
    let vertices = if closed { n } else { n + 1 };
    let points: Vec<String> = (0..vertices)
        .map(|k| {
            let q = patch.fresh("p");
            patch.set("points", &q, json!(s.at(k as f64 / n as f64)));
            patch.constrain(json!({"type": "on", "point": q, "curve": name}));
            q
        })
        .collect();
    // A circle's last side closes to its first vertex; an arc's chain is open.
    let segments = if closed { points.len() } else { points.len() - 1 };
    for k in 0..segments {
        let l = patch.fresh("l");
        patch.set("lines", &l, json!({"from": points[k], "to": points[(k + 1) % points.len()]}));
    }
    // The circle or arc stays as a holder of the polygon: construction geometry.
    let mut c = edit.json["sketch"][key][&name].clone();
    c["construction"] = json!(true);
    patch.set(key, &name, c);
    patch.done()
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use serde_json::{Value as Json, json};

    use crate::draw::Tool;
    use crate::draw::tests::{click_at, editor};
    use crate::sketch::{Pick, SketchEdit};

    /// A sketch with circle `c` of diameter 20 about `[0, 0]`, selected.
    fn circle() -> SketchEdit {
        let mut e = editor(
            json!({"points": {"o": [0, 0]},
                   "circles": {"c": {"center": "o", "diameter": 20}},
                   "constraints": [{"type": "fix", "point": "o"}]}),
            false,
        );
        e.draft.tool = Tool::Polygon;
        e.selected = vec![Pick::Curve("c".into())];
        e
    }

    /// An arc about `[0, 0]` of radius 10 from angle 0 a quarter turn round, selected.
    fn arc() -> SketchEdit {
        let mut e = editor(
            json!({"points": {"o": [0, 0], "s": [10, 0], "t": [0, 10]},
                   "arcs": {"a": {"center": "o", "from": "s", "to": "t"}},
                   "constraints": [{"type": "fix", "point": "o"}, {"type": "radius", "arc": "a", "value": 10}]}),
            false,
        );
        e.draft.tool = Tool::Polygon;
        e.selected = vec![Pick::Curve("a".into())];
        e
    }

    /// Where point `n` of the solved sketch is, rounded to a millionth.
    fn at(e: &SketchEdit, n: &str) -> [f64; 2] {
        let v = e.placed.as_ref().unwrap().solved.points[n];
        [(v.x * 1e6).round() / 1e6, (v.y * 1e6).round() / 1e6]
    }

    /// The names of the curves in `key` that are not in `before`.
    fn new_ones(e: &SketchEdit, key: &str, before: &Json) -> Vec<String> {
        e.json["sketch"][key]
            .as_object()
            .into_iter()
            .flatten()
            .filter(|(n, _)| before[key].get(n.as_str()).is_none())
            .map(|(n, _)| n.clone())
            .collect()
    }

    /// Six sides on the circle: six vertices round it, six lines, the last closing to the first.
    #[test]
    fn a_circle_gets_a_closed_polygon() {
        let mut e = circle();
        let before = e.json["sketch"].clone();
        e.draft.typed = Some((json!(6), 6.0));
        let p = click_at(&mut e, [0.0, 0.0], false).unwrap();
        assert!(e.placed.is_ok(), "{:?}", e.placed.as_ref().err());
        let points = new_ones(&e, "points", &before);
        let lines = new_ones(&e, "lines", &before);
        assert_eq!(points.len(), 6, "{p}");
        assert_eq!(lines.len(), 6, "{p}");
        // Even steps round the circle from its start, the radius 10.
        for (k, q) in points.iter().enumerate() {
            let a = std::f64::consts::TAU * k as f64 / 6.0;
            let (sn, cs) = libm::sincos(a);
            let want = [(10.0 * cs * 1e6).round() / 1e6, (10.0 * sn * 1e6).round() / 1e6];
            assert_eq!(at(&e, q), want, "{q}");
        }
        // The last side closes to the first vertex: each vertex starts one side and ends one.
        let from: Vec<String> = lines.iter().map(|l| e.json["sketch"]["lines"][l]["from"].as_str().unwrap().to_string()).collect();
        let to: Vec<String> = lines.iter().map(|l| e.json["sketch"]["lines"][l]["to"].as_str().unwrap().to_string()).collect();
        let sorted = |mut v: Vec<String>| {
            v.sort();
            v
        };
        assert_eq!(sorted(from), sorted(points.clone()), "closed: each vertex starts a side");
        assert_eq!(sorted(to), sorted(points), "closed: each vertex ends a side");
    }

    /// Four sides on the arc: five vertices from its start to its end, a quarter turn cut into
    /// four, four lines and not a fifth closing it.
    #[test]
    fn an_arc_gets_an_open_chain() {
        let mut e = arc();
        let before = e.json["sketch"].clone();
        e.draft.typed = Some((json!(4), 4.0));
        let p = click_at(&mut e, [0.0, 0.0], false).unwrap();
        assert!(e.placed.is_ok(), "{:?}", e.placed.as_ref().err());
        let points = new_ones(&e, "points", &before);
        let lines = new_ones(&e, "lines", &before);
        assert_eq!(points.len(), 5, "{p}");
        assert_eq!(lines.len(), 4, "{p}");
        for (k, q) in points.iter().enumerate() {
            let a = std::f64::consts::FRAC_PI_2 * k as f64 / 4.0;
            let (sn, cs) = libm::sincos(a);
            let want = [(10.0 * cs * 1e6).round() / 1e6, (10.0 * sn * 1e6).round() / 1e6];
            assert_eq!(at(&e, q), want, "{q}");
        }
        // An open chain: from the first vertex to the last, each side from one vertex to the next.
        let from: Vec<String> = lines.iter().map(|l| e.json["sketch"]["lines"][l]["from"].as_str().unwrap().to_string()).collect();
        let to: Vec<String> = lines.iter().map(|l| e.json["sketch"]["lines"][l]["to"].as_str().unwrap().to_string()).collect();
        let sorted = |mut v: Vec<String>| {
            v.sort();
            v
        };
        assert_eq!(sorted(from), sorted(points[..4].to_vec()), "open: the sides start at the first four");
        assert_eq!(sorted(to), sorted(points[1..].to_vec()), "open: the sides end at the last four");
    }

    /// The circle turns construction; every vertex stands on it (`on`), so its radius holds the
    /// polygon.
    #[test]
    fn the_curve_becomes_construction_and_holds_the_vertices() {
        let mut e = circle();
        let before = e.json["sketch"].clone();
        e.draft.typed = Some((json!(5), 5.0));
        click_at(&mut e, [0.0, 0.0], false).unwrap();
        assert_eq!(e.json["sketch"]["circles"]["c"]["construction"], json!(true));
        let points = new_ones(&e, "points", &before);
        let on: Vec<Json> = e.json["sketch"]["constraints"].as_array().unwrap().iter().filter(|c| c["type"] == "on").cloned().collect();
        assert_eq!(on.len(), points.len(), "{on:?}");
        for q in &points {
            assert!(on.contains(&json!({"type": "on", "point": q, "curve": "c"})), "{q}");
        }
    }

    /// No count typed, or one that is not a whole number of three or more: nothing is made, as
    /// the fillet makes nothing without its size.
    #[test]
    fn a_whole_count_of_three_or_more_is_needed() {
        let mut e = circle();
        for count in [None, Some((json!(0), 0.0)), Some((json!(2), 2.0)), Some((json!(4.5), 4.5))] {
            let why = format!("{count:?}");
            e.draft.typed = count;
            assert!(click_at(&mut e, [0.0, 0.0], false).is_none(), "{why}");
        }
        // Nothing selected: nothing either.
        e.selected.clear();
        e.draft.typed = Some((json!(6), 6.0));
        assert!(click_at(&mut e, [0.0, 0.0], false).is_none());
    }
}
