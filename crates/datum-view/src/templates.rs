//! Sketch templates (M7e, D94): pieces of sketch as data — points, lines, arcs, circles and the
//! constraints between them — placed by clicks. A template names one or two of its points
//! `anchors`: the clicks land there. Two anchors stretch it along the sketch's axes (a rectangle
//! by two corners, or by its centre and a corner); one moves it as it is (a copy pasted). Names
//! are made new on placing; an anchor clicked on a point of the sketch becomes that point. The
//! built-in templates are JSON files in `library/sketches`; a copy of a selection is a template
//! too, on the clipboard as JSON.

use std::collections::BTreeMap;

use serde_json::{Map, Value as Json, json};

use crate::draw::{Patch, Spot};
use crate::sketch::{Pick, SketchEdit};

/// The built-in templates.
pub fn library() -> Vec<Json> {
    [include_str!("../../../library/sketches/rectangle.json"), include_str!("../../../library/sketches/center_rectangle.json")]
        .iter()
        .filter_map(|t| serde_json::from_str(t).ok())
        .collect()
}

/// The anchors of a template: one or two of its points, in order.
pub fn anchors(t: &Json) -> Vec<String> {
    t["anchors"].as_array().into_iter().flatten().filter_map(|a| a.as_str().map(str::to_string)).collect()
}

fn point(t: &Json, n: &str) -> Option<[f64; 2]> {
    let p = &t["points"][n];
    Some([p[0].as_f64()?, p[1].as_f64()?])
}

/// How a template's coordinates go to the sketch's: its first anchor `from` onto the first click
/// `to`, stretched along each axis by `scale`.
pub struct Placement {
    from: [f64; 2],
    to: [f64; 2],
    pub scale: [f64; 2],
}

impl Placement {
    pub fn map(&self, p: [f64; 2]) -> [f64; 2] {
        [self.to[0] + (p[0] - self.from[0]) * self.scale[0], self.to[1] + (p[1] - self.from[1]) * self.scale[1]]
    }
}

/// The placement of template `t` by clicks `at` (one per anchor): moved, or stretched along
/// each axis so the anchors land on the clicks.
pub fn placement(t: &Json, at: &[[f64; 2]]) -> Option<Placement> {
    let names = anchors(t);
    let a = point(t, names.first()?)?;
    let c1 = *at.first()?;
    let scale = match (names.get(1), at.get(1)) {
        (Some(b), Some(c2)) => {
            let b = point(t, b)?;
            let s = |k: usize| if b[k] == a[k] { 1.0 } else { (c2[k] - c1[k]) / (b[k] - a[k]) };
            [s(0), s(1)]
        }
        _ => [1.0, 1.0],
    };
    Some(Placement { from: a, to: c1, scale })
}

/// The polylines of the template placed by clicks `at`: its lines, arcs and circles.
pub fn outline(t: &Json, at: &[[f64; 2]]) -> Vec<Vec<[f64; 2]>> {
    let Some(place) = placement(t, at) else { return Vec::new() };
    let scale = place.scale;
    let pt = |n: &Json| n.as_str().and_then(|n| point(t, n)).map(|p| place.map(p));
    let mut out = Vec::new();
    for l in t["lines"].as_object().into_iter().flat_map(|m| m.values()) {
        if let (Some(a), Some(b)) = (pt(&l["from"]), pt(&l["to"])) {
            out.push(vec![a, b]);
        }
    }
    let ring = |c: [f64; 2], r: f64, start: f64, sweep: f64| {
        (0..=48).map(move |k| {
            let (s, co) = libm::sincos(start + sweep * f64::from(k) / 48.0);
            [c[0] + r * co, c[1] + r * s]
        })
    };
    for a in t["arcs"].as_object().into_iter().flat_map(|m| m.values()) {
        if let (Some(c), Some(f), Some(e)) = (pt(&a["center"]), pt(&a["from"]), pt(&a["to"])) {
            let start = libm::atan2(f[1] - c[1], f[0] - c[0]);
            let sweep = (libm::atan2(e[1] - c[1], e[0] - c[0]) - start).rem_euclid(std::f64::consts::TAU);
            out.push(ring(c, libm::hypot(f[0] - c[0], f[1] - c[1]), start, sweep).collect());
        }
    }
    for k in t["circles"].as_object().into_iter().flat_map(|m| m.values()) {
        if let (Some(c), Some(d)) = (pt(&k["center"]), k["diameter"].as_f64()) {
            out.push(ring(c, 0.5 * d * libm::sqrt((scale[0] * scale[1]).abs()), 0.0, std::f64::consts::TAU).collect());
        }
    }
    out
}

/// Places template `t` by `spots` (one per anchor) into `patch`: new names for all, the
/// anchors made from the spots as any click is (on a point of the sketch: that point).
pub fn place(patch: &mut Patch, t: &Json, spots: &[Spot], construction: bool) -> Option<()> {
    let at: Vec<[f64; 2]> = spots.iter().map(|s| s.at).collect();
    let place = placement(t, &at)?;
    let scale = place.scale;
    let mut names: BTreeMap<String, String> = BTreeMap::new();
    for (a, s) in anchors(t).iter().zip(spots) {
        names.insert(a.clone(), patch.point(s));
    }
    let others: Vec<String> = t["points"].as_object()?.keys().filter(|n| !names.contains_key(*n)).cloned().collect();
    for n in others {
        let new = patch.fresh("p");
        names.insert(n, new);
    }
    for (key, prefix) in [("lines", "l"), ("arcs", "a"), ("circles", "c"), ("measures", "d")] {
        for n in t[key].as_object().into_iter().flat_map(|m| m.keys()) {
            let new = patch.fresh(prefix);
            names.insert(n.clone(), new);
        }
    }
    let rename = |v: &Json| {
        let mut v = v.clone();
        datum_model::visit_names(&mut v, &mut |s| {
            if let Some(n) = names.get(s.as_str()) {
                *s = n.clone();
            }
        });
        v
    };
    for (n, p) in t["points"].as_object()?.iter().filter(|(n, _)| !anchors(t).contains(n)) {
        let q = place.map([p[0].as_f64()?, p[1].as_f64()?]);
        patch.set("points", &names[n], json!(q));
    }
    for key in ["lines", "arcs", "circles"] {
        for (n, v) in t[key].as_object().into_iter().flatten() {
            let mut v = rename(v);
            // A circle keeps its shape: its diameter stretched by the mean of the two stretches.
            if let Some(d) = v["diameter"].as_f64() {
                v["diameter"] = json!(d * libm::sqrt((scale[0] * scale[1]).abs()));
            }
            patch.add(key, &names[n], v, construction);
        }
    }
    for (n, v) in t["measures"].as_object().into_iter().flatten() {
        patch.set("measures", &names[n], rename(v));
    }
    for c in t["constraints"].as_array().into_iter().flatten() {
        patch.constrain(rename(c));
    }
    Some(())
}

impl SketchEdit {
    /// The selection as a template to paste: the curves picked with their points, the points
    /// picked, and the constraints and reference dimensions among them; the point lowest and
    /// leftmost is the anchor. Projections and the origin stay behind, and so does what ties to
    /// them.
    pub fn copy(&self) -> Option<Json> {
        let p = self.placed.as_ref().ok()?;
        let sketch = &self.json["sketch"];
        let own = |n: &str| sketch["points"].get(n).is_some();
        let mut names: Vec<String> = Vec::new();
        let mut out = Map::new();
        for pick in &self.selected {
            match pick {
                Pick::Curve(n) => {
                    for key in ["lines", "arcs", "circles"] {
                        if let Some(v) = sketch[key].get(n) {
                            let mut v = v.clone();
                            let mut ends = Vec::new();
                            datum_model::visit_names(&mut v, &mut |s| ends.push(s.clone()));
                            if ends.iter().all(|e| own(e)) {
                                out.entry(key).or_insert_with(|| json!({}))[n] = v;
                                names.push(n.clone());
                                names.extend(ends);
                            }
                        }
                    }
                }
                Pick::Point(n) if own(n) => names.push(n.clone()),
                _ => {}
            }
        }
        names.sort();
        names.dedup();
        let points: Map<String, Json> =
            names.iter().filter_map(|n| p.solved.points.get(n).filter(|_| own(n)).map(|v| (n.clone(), json!([v.x, v.y])))).collect();
        if points.is_empty() {
            return None;
        }
        // Constraints and measures whose names are all copied come along.
        let within = |v: &Json| {
            let mut v = v.clone();
            let mut all = true;
            datum_model::visit_names(&mut v, &mut |s| all &= names.contains(s));
            all
        };
        let constraints: Vec<Json> = sketch["constraints"].as_array().into_iter().flatten().filter(|c| within(c)).cloned().collect();
        let measures: Map<String, Json> =
            sketch["measures"].as_object().into_iter().flatten().filter(|(_, v)| within(v)).map(|(n, v)| (n.clone(), v.clone())).collect();
        let anchor = points
            .iter()
            .min_by(|a, b| {
                let (pa, pb) = (point_of(a.1), point_of(b.1));
                (pa[1], pa[0]).partial_cmp(&(pb[1], pb[0])).unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|(n, _)| n.clone())?;
        out.insert("name".into(), json!("Copy"));
        out.insert("anchors".into(), json!([anchor]));
        out.insert("points".into(), Json::Object(points));
        out.insert("constraints".into(), json!(constraints));
        if !measures.is_empty() {
            out.insert("measures".into(), Json::Object(measures));
        }
        Some(Json::Object(out))
    }
}

fn point_of(v: &Json) -> [f64; 2] {
    [v[0].as_f64().unwrap_or(0.0), v[1].as_f64().unwrap_or(0.0)]
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use std::collections::BTreeMap;

    use datum_brep::TopoId;

    use super::*;
    use crate::draw::{Snap, Tool};
    use crate::sketch::Context;

    fn editor(sketch: Json) -> SketchEdit {
        let json = json!({"name": "s", "op": "sketch", "inputs": [], "sketch": sketch});
        SketchEdit::new("s".into(), Context { id: TopoId::derive("feature").tag("s").finish(), input: None }, json, BTreeMap::new(), None)
    }

    /// Places template `t` by `spots` through the tool, as clicks would.
    fn place_by(e: &mut SketchEdit, t: Json, spots: &[Spot]) -> Json {
        e.draft.tool = Tool::Template;
        e.draft.template = Some(t);
        let mut patch = None;
        for s in spots {
            e.draft.cursor = Some(s.clone());
            let mut d = std::mem::take(&mut e.draft);
            patch = d.click(e);
            e.draft = d;
        }
        e.apply(patch.unwrap())
    }

    fn free(at: [f64; 2]) -> Spot {
        Spot { at, snap: Snap::Free }
    }

    /// The template follows the cursor before the last click: `DATUM_TEMPLATE_SHOT=file.png`
    /// saves a picture.
    #[test]
    fn previews_before_placing() {
        let mut e = editor(json!({"points": {}}));
        e.camera.center = [0.0, 0.0];
        e.camera.half_height = 20.0;
        e.draft.tool = Tool::Template;
        e.draft.template = Some(library()[1].clone());
        e.draft.clicks = vec![Spot { at: [0.0, 0.0], snap: Snap::Point("origin".into()) }];
        e.draft.cursor = Some(free([12.0, 7.0]));
        let shot = std::env::var_os("DATUM_TEMPLATE_SHOT");
        let builder = egui_kittest::Harness::builder().with_size([800.0, 500.0]);
        let builder = if shot.is_some() { builder.wgpu() } else { builder };
        let mut h = builder.build_ui(|ui| {
            let (rect, _) = ui.allocate_exact_size(ui.available_size(), egui::Sense::hover());
            ui.painter().rect_filled(rect, 0.0, egui::Color32::from_rgb(235, 238, 242));
            let painter = ui.painter().with_clip_rect(rect);
            e.paint(&painter, rect);
            e.draft.paint(&e, &painter, rect);
        });
        h.run();
        if let Some(path) = shot {
            h.render().unwrap().save(path).unwrap();
        }
        assert_eq!(outline(&library()[1], &[[0.0, 0.0], [12.0, 7.0]]).len(), 6);
    }

    #[test]
    fn the_library_loads() {
        let names: Vec<String> = library().iter().map(|t| t["name"].as_str().unwrap().to_string()).collect();
        assert_eq!(names, ["Rectangle", "Center rectangle"]);
    }

    #[test]
    fn a_rectangle_by_two_corners() {
        let mut e = editor(json!({"points": {}}));
        let p = place_by(&mut e, library()[0].clone(), &[free([2.0, 3.0]), free([12.0, 8.0])]);
        assert_eq!(p["points"], json!({"p1": [2.0, 3.0], "p2": [12.0, 8.0], "p3": [12.0, 3.0], "p4": [2.0, 8.0]}), "{p}");
        assert_eq!(p["lines"].as_object().unwrap().len(), 4);
        assert_eq!(p["add_constraints"][0], json!({"type": "horizontal", "line": "l1"}));
        let placed = e.placed.as_ref().unwrap();
        assert_eq!(placed.solved.dof, 4, "a rectangle anywhere, of any size");
    }

    /// From its centre, put on the origin: the rectangle stays centred on it.
    #[test]
    fn a_rectangle_from_its_centre_on_the_origin() {
        let mut e = editor(json!({"points": {}}));
        let p = place_by(&mut e, library()[1].clone(), &[Spot { at: [0.0, 0.0], snap: Snap::Point("origin".into()) }, free([5.0, 3.0])]);
        assert!(p["points"].get("origin").is_none(), "the origin is not made again: {p}");
        assert!(p["add_constraints"].as_array().unwrap().iter().any(|c| c["point"] == "origin"), "{p}");
        let placed = e.placed.as_ref().unwrap();
        assert_eq!(placed.solved.dof, 2, "width and height: {:?}", placed.solved.free);
        let corner = placed.solved.points[p["lines"]["l2"]["to"].as_str().unwrap()];
        assert!((corner.x - 5.0).abs() < 1e-9 && (corner.y - 3.0).abs() < 1e-9, "{corner:?}");
    }

    /// A copy takes what is between the copied elements along, under new names, and pastes
    /// where clicked.
    #[test]
    fn copy_and_paste() {
        let mut e = editor(json!({"points": {}}));
        place_by(&mut e, library()[0].clone(), &[free([0.5, 0.5]), free([10.5, 4.5])]);
        e.apply(serde_json::from_value(json!({"add_constraints": [{"type": "length", "line": "l1", "value": 10}]})).unwrap());
        e.selected = ["l1", "l2", "l3", "l4"].iter().map(|n| Pick::Curve(n.to_string())).collect();
        let t = e.copy().unwrap();
        assert_eq!(anchors(&t), ["p1"], "the lowest, leftmost point: {t}");
        assert_eq!(t["constraints"].as_array().unwrap().len(), 5, "{t}");
        let p = place_by(&mut e, t, &[free([30.0, 0.0])]);
        let mut corners: Vec<String> = p["points"].as_object().unwrap().values().map(|v| v.to_string()).collect();
        corners.sort();
        assert_eq!(corners, ["[30.0,0.0]", "[30.0,4.0]", "[40.0,0.0]", "[40.0,4.0]"], "moved by the click: {p}");
        assert!(p["add_constraints"].as_array().unwrap().contains(&json!({"type": "length", "line": "l5", "value": 10})), "{p}");
        assert_eq!(e.placed.as_ref().unwrap().solved.dof, 6, "two rectangles, each 10 wide");
    }
}
