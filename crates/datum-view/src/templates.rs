//! Library elements in the sketch (M7e, D94; D98, D99): pieces of sketch as data, placed by
//! clicks, one per anchor; two anchors stretch the piece (a rectangle by two corners, or by its
//! centre and a corner). Loading and placing are the model's (`datum_model::library`), shared
//! with the agent; here: the preview, the snaps of the clicks, and the copy of a selection,
//! which is an element too, on the clipboard as JSON.

use std::collections::BTreeMap;

use datum_math::expr::ExprError;
use datum_model::library::{self, Anchor, Entry, Source};
use serde_json::{Map, Value as Json};

use crate::draw::{Draft, Patch, Snap, Spot};
use crate::sketch::{Pick, SketchEdit};

/// An element of the library as the palette shows it: with its outline, drawn once.
pub struct Item {
    pub entry: Entry,
    pub outline: Vec<Vec<[f64; 2]>>,
}

/// The sketch elements for the model at `model`: built in, beside the program, in its project.
/// Files that are not elements go to the log.
pub fn library(model: Option<&std::path::Path>) -> Vec<Item> {
    let lib = library::load("sketch", model);
    for why in &lib.skipped {
        crate::log::write(&format!("{why}: not a sketch element"));
    }
    let none = BTreeMap::new();
    lib.entries.into_iter().map(|entry| Item { outline: outline(&entry.element, None, &Map::new(), &[[0.0, 0.0]], &none), entry }).collect()
}

/// Where an element in the list comes from, as the list shows it.
pub fn source(e: &Entry) -> &'static str {
    match e.source {
        Source::BuiltIn => "built in",
        Source::Program => "beside the program",
        Source::Project => "the project's",
    }
}

/// The model's parameter values, for expressions in answers.
fn lookup(params: &BTreeMap<String, f64>) -> impl Fn(&str) -> Result<f64, ExprError> + '_ {
    |n: &str| params.get(n).copied().ok_or_else(|| ExprError::Unknown(n.to_string()))
}

/// The polylines of element `t` placed by clicks `at` with the parameter values of `preset` and
/// `given` (the clicks answering the rest, D98): its lines, arcs and circles as it will be, or
/// as drawn where it does not solve alone.
pub fn outline(t: &Json, preset: Option<&str>, given: &Map<String, Json>, at: &[[f64; 2]], params: &BTreeMap<String, f64>) -> Vec<Vec<[f64; 2]>> {
    let lookup = lookup(params);
    let Ok(values) = library::answers(t, preset, given, at, &lookup) else { return Vec::new() };
    let Ok(s) = library::shape(t, &values, at, &lookup).or_else(|_| library::placed(t, &values, at, &lookup)) else { return Vec::new() };
    let pt = |n: &str| s.points.get(n).map(|q| [q.x, q.y]);
    let mut out = Vec::new();
    for l in s.lines.values() {
        if let (Some(a), Some(b)) = (pt(&l.from), pt(&l.to)) {
            out.push(vec![a, b]);
        }
    }
    let ring = |c: [f64; 2], r: f64, start: f64, sweep: f64| {
        (0..=48).map(move |k| {
            let (s, co) = libm::sincos(start + sweep * f64::from(k) / 48.0);
            [c[0] + r * co, c[1] + r * s]
        })
    };
    for a in s.arcs.values() {
        if let (Some(c), Some(f), Some(e)) = (pt(&a.center), pt(&a.from), pt(&a.to)) {
            let start = libm::atan2(f[1] - c[1], f[0] - c[0]);
            let sweep = (libm::atan2(e[1] - c[1], e[0] - c[0]) - start).rem_euclid(std::f64::consts::TAU);
            out.push(ring(c, libm::hypot(f[0] - c[0], f[1] - c[1]), start, sweep).collect());
        }
    }
    for k in s.circles.values() {
        if let (Some(c), Some(d)) = (pt(&k.center), k.diameter.value()) {
            out.push(ring(c, 0.5 * d, 0.0, std::f64::consts::TAU).collect());
        }
    }
    out
}

/// The value parameter `name` of the element being placed has now, as the field shows it: typed,
/// else the preset's, else the element's own.
pub fn asked(draft: &Draft, name: &str, params: &BTreeMap<String, f64>) -> String {
    let Some(t) = &draft.template else { return String::new() };
    library::answers(t, draft.preset.as_deref(), &draft.answers, &[], &lookup(params))
        .ok()
        .and_then(|v| v.get(name).map(crate::dims::text_of))
        .unwrap_or_default()
}

/// Places element `t` by `spots` (a spot for its first anchor or for each) into `patch`, with
/// the parameter values of `preset` and `given`, the clicks answering the rest. An anchor
/// clicked on a point of the sketch is made coincident with it; on a vertex of the face, with
/// its projection; on a middle, a curve or an edge, held there as any click is. Returns its
/// sockets, to be completed next (D116).
pub fn place(
    patch: &mut Patch,
    t: &Json,
    preset: Option<&str>,
    given: &Map<String, Json>,
    spots: &[Spot],
    construction: bool,
    params: &BTreeMap<String, f64>,
) -> Option<Vec<library::Link>> {
    let lookup = lookup(params);
    let at: Vec<[f64; 2]> = spots.iter().map(|s| s.at).collect();
    let values = library::answers(t, preset, given, &at, &lookup).ok()?;
    let anchors: Vec<Anchor> = spots
        .iter()
        .map(|s| Anchor {
            at: s.at,
            point: match &s.snap {
                Snap::Point(_) | Snap::Vertex(_) => Some(patch.point(s)),
                _ => None,
            },
        })
        .collect();
    let renamed = library::place(patch, t, &values, &anchors, &BTreeMap::new(), construction, &lookup).ok()?;
    for (a, s) in library::anchors(t).iter().zip(spots) {
        patch.tie(&renamed[a], s);
    }
    library::links(t, &values, &renamed).ok()
}

impl SketchEdit {
    /// The selection as an element named `name`: the curves picked with their points, the
    /// points picked, and the constraints and reference dimensions among them (see
    /// [`library::extract`]). With `params`, the model's parameters its dimensions name become
    /// the element's (to keep in the library); without, a copy to paste.
    pub fn extract(&self, name: &str, params: Option<&BTreeMap<String, f64>>) -> Option<Json> {
        let p = self.placed.as_ref().ok()?;
        let (mut curves, mut points) = (Vec::new(), Vec::new());
        for pick in &self.selected {
            match pick {
                Pick::Curve(n) => curves.push(n.clone()),
                Pick::Point(n) => points.push(n.clone()),
                _ => {}
            }
        }
        library::extract(&self.json, &p.solved.points, &curves, &points, name, params)
    }

    /// The selection as an element to paste (Ctrl+C), its dimensions as they are.
    pub fn copy(&self) -> Option<Json> {
        self.extract("Copy", None)
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use std::collections::BTreeMap;

    use datum_brep::TopoId;

    use super::*;
    use crate::draw::{Snap, Tool};
    use crate::sketch::Context;
    use serde_json::json;

    fn editor(sketch: Json) -> SketchEdit {
        let json = json!({"name": "s", "op": "sketch", "inputs": [], "sketch": sketch});
        SketchEdit::new("s".into(), Context { id: TopoId::derive("feature").tag("s").finish(), input: None }, json, BTreeMap::new(), None)
    }

    /// Places template `t` by `spots` through the tool, as clicks would.
    fn place_by(e: &mut SketchEdit, t: Json, spots: &[Spot]) -> Json {
        e.draft.tool = Tool::Template;
        e.draft.template = Some(t);
        e.draft.clicks.clear();
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
        e.draft.template = Some(library(None)[1].entry.element.clone());
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
        assert_eq!(outline(&library(None)[1].entry.element, None, &Map::new(), &[[0.0, 0.0], [12.0, 7.0]], &BTreeMap::new()).len(), 6);
    }

    #[test]
    fn the_library_loads() {
        let names: Vec<String> = library(None).iter().map(|t| format!("{} ({})", t.entry.name(), source(&t.entry))).collect();
        assert!(library(None).iter().all(|t| t.outline.len() == 4 || t.outline.len() == 6), "outlines drawn for the palette");
        assert_eq!(names, ["Rectangle (built in)", "Center rectangle (built in)"]);
    }

    #[test]
    fn a_rectangle_by_two_corners() {
        let mut e = editor(json!({"points": {}}));
        let p = place_by(&mut e, library(None)[0].entry.element.clone(), &[free([2.0, 3.0]), free([12.0, 8.0])]);
        assert_eq!(p["points"], json!({"p1": [2.0, 3.0], "p2": [12.0, 3.0], "p3": [12.0, 8.0], "p4": [2.0, 8.0]}), "{p}");
        assert_eq!(p["lines"].as_object().unwrap().len(), 4);
        assert_eq!(p["add_constraints"][0], json!({"type": "horizontal", "line": "l1"}));
        let placed = e.placed.as_ref().unwrap();
        assert_eq!(placed.solved.dof, 4, "a rectangle anywhere, of any size");
    }

    /// From its centre, put on the origin: the rectangle stays centred on it.
    #[test]
    fn a_rectangle_from_its_centre_on_the_origin() {
        let mut e = editor(json!({"points": {}}));
        let p = place_by(
            &mut e,
            library(None)[1].entry.element.clone(),
            &[Spot { at: [0.0, 0.0], snap: Snap::Point("origin".into()) }, free([5.0, 3.0])],
        );
        assert!(p["points"].get("origin").is_none(), "the origin is not made again: {p}");
        assert!(
            p["add_constraints"].as_array().unwrap().iter().any(|c| c["type"] == "coincident" && c["points"][1] == "origin"),
            "coincident, not merged: {p}"
        );
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
        place_by(&mut e, library(None)[0].entry.element.clone(), &[free([0.5, 0.5]), free([10.5, 4.5])]);
        e.apply(serde_json::from_value(json!({"add_constraints": [{"type": "length", "line": "l1", "value": 10}]})).unwrap());
        e.selected = ["l1", "l2", "l3", "l4"].iter().map(|n| Pick::Curve(n.to_string())).collect();
        let t = e.copy().unwrap();
        assert_eq!(library::anchors(&t), ["p1"], "the lowest, leftmost point: {t}");
        assert!(library::check("sketch", &t).is_ok(), "a copy is an element: {t}");
        assert_eq!(t["sketch"]["constraints"].as_array().unwrap().len(), 5, "{t}");
        let p = place_by(&mut e, t, &[free([30.0, 0.0])]);
        let mut corners: Vec<String> = p["points"].as_object().unwrap().values().map(|v| v.to_string()).collect();
        corners.sort();
        assert_eq!(corners, ["[30.0,0.0]", "[30.0,4.0]", "[40.0,0.0]", "[40.0,4.0]"], "moved by the click: {p}");
        assert!(p["add_constraints"].as_array().unwrap().contains(&json!({"type": "length", "line": "l5", "value": 10})), "{p}");
        assert_eq!(e.placed.as_ref().unwrap().solved.dof, 6, "two rectangles, each 10 wide");
    }

    /// A plate with its width and height as parameters, a preset, two corners as anchors.
    fn plate() -> Json {
        json!({"name": "Plate", "anchors": ["a", "c"],
            "parameters": [{"name": "width", "value": 100}, {"name": "height", "value": 50}],
            "presets": {"Small": {"width": 20, "height": 10}},
            "sketch": {"points": {"a": [0, 0], "b": [1, 0], "c": [1, 1], "d": [0, 1]},
                       "lines": {"bottom": {"from": "a", "to": "b"}, "right": {"from": "b", "to": "c"}, "top": {"from": "c", "to": "d"}, "left": {"from": "d", "to": "a"}},
                       "constraints": [{"type": "horizontal", "line": "bottom"}, {"type": "vertical", "line": "right"},
                                       {"type": "horizontal", "line": "top"}, {"type": "vertical", "line": "left"},
                                       {"type": "length", "line": "bottom", "value": "width"}, {"type": "length", "line": "left", "value": "height"}]}})
    }

    fn lengths(p: &Json) -> Vec<Json> {
        p["add_constraints"].as_array().unwrap().iter().filter(|c| c["type"] == "length").map(|c| c["value"].clone()).collect()
    }

    /// One stream of input: the field asks the parameters (typed, else the preset's, else the
    /// element's), Enter puts the element where the cursor is.
    #[test]
    fn parameters_typed_then_enter() {
        let mut e = editor(json!({"points": {}}));
        e.draft.choose(plate());
        assert_eq!(asked(&e.draft, "width", &e.params), "100");
        e.draft.preset = Some("Small".into());
        e.draft.answers.insert("height".into(), json!("h + 1"));
        e.params.insert("h".into(), 4.0);
        assert_eq!((asked(&e.draft, "width", &e.params), asked(&e.draft, "height", &e.params)), ("20".into(), "h + 1".into()));
        e.draft.cursor = Some(free([3.0, 4.0]));
        let mut d = std::mem::take(&mut e.draft);
        let patch = d.enter(&e).unwrap();
        e.draft = d;
        let p = e.apply(patch);
        assert_eq!(lengths(&p), [json!(20.0), json!("(h + 1)")], "{p}");
        assert!(e.draft.answers.contains_key("height"), "the values stay for the next one");
        let placed = e.placed.as_ref().unwrap();
        assert_eq!(placed.solved.dof, 2, "sized, free to move: {:?}", placed.solved.free);
    }

    /// Two clicks answer what is not typed: the width between them, the height as typed.
    #[test]
    fn clicks_answer_the_rest() {
        let mut e = editor(json!({"points": {}}));
        e.draft.choose(plate());
        e.draft.answers.insert("height".into(), json!(5));
        let t = e.draft.template.clone().unwrap();
        let p = place_by(&mut e, t, &[free([0.0, 0.0]), free([30.0, 20.0])]);
        let l = lengths(&p);
        assert!((l[0].as_f64().unwrap() - 30.0).abs() < 1e-9 && l[1] == json!(5.0), "{p}");
    }

    /// A socket asked after the element is placed: a click on a line completes it, a click on a
    /// point does not (it takes a line), Enter skips it.
    #[test]
    fn sockets_after_placing() {
        let mut e = editor(json!({"points": {"p1": [0, 0], "p2": [20, 0]}, "lines": {"l1": {"from": "p1", "to": "p2"}}}));
        let step = json!({"name": "Step", "anchors": ["a"],
            "links": [{"constraint": {"type": "perpendicular", "lines": ["?", "up"]}, "kind": "line"}],
            "sketch": {"points": {"a": [0, 0], "b": [0, 5]}, "lines": {"up": {"from": "a", "to": "b"}}}});
        place_by(&mut e, step.clone(), &[free([5.0, 3.0])]);
        assert_eq!(e.draft.linking.len(), 1);
        assert_eq!(e.draft.linking[0].shown, "perpendicular ? l2");
        let click = |e: &mut SketchEdit, snap: Snap| {
            e.draft.cursor = Some(Spot { at: [10.0, 0.0], snap });
            let mut d = std::mem::take(&mut e.draft);
            let patch = d.click(e);
            e.draft = d;
            patch.map(|p| e.apply(p))
        };
        assert!(click(&mut e, Snap::Point("p1".into())).is_none(), "a point is not a line");
        let p = click(&mut e, Snap::On("l1".into())).unwrap();
        assert_eq!(p["add_constraints"], json!([{"type": "perpendicular", "lines": ["l1", "l2"]}]), "{p}");
        assert!(e.draft.linking.is_empty());
        // The next one: Enter skips its socket.
        place_by(&mut e, step, &[free([8.0, 3.0])]);
        let mut d = std::mem::take(&mut e.draft);
        assert!(d.enter(&e).is_none() && d.linking.is_empty());
    }
}
