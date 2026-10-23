//! The library of reusable work (VISION §10, D98, D99, D109): elements as data, kept in folders
//! with no registry. An element is a sketch piece now; parts, assemblies and drawings come as
//! their levels do, in the same envelope:
//!
//! ```json
//! {"name": "Slot", "description": "…",
//!  "parameters": [{"name": "length", "value": 40}, {"name": "width", "value": 10}],
//!  "presets": {"M8": {"length": 40, "width": 9}},
//!  "anchors": ["a"], "fit": "stretch",
//!  "sketch": {"points": …, "constraints": [{"type": "length", "line": "l1", "value": "length"}]}}
//! ```
//!
//! Parameters are asked in their order; dimensions name them in their values. Placing is a copy:
//! the answers go into the values (a number, or an expression of the model's parameters in
//! parentheses), new names are given, and nothing remembers the element.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use datum_math::Vec2;
use datum_math::expr::{ExprError, eval, substitute, valid_name};
use datum_sketch::{Constraint, Sketch};
use serde::Serialize;
use serde_json::{Map, Value, json};

use crate::patch::SketchPatch;

/// The file that makes a folder a project (VISION §6): its `library` holds the project's
/// elements, beside `part`, `assembly` and `drawing`.
pub const PROJECT_FILE: &str = "datum-project.json";

/// The levels with elements so far: a library folder's subfolders by the same names.
pub const LEVELS: [&str; 1] = ["sketch"];

const BUILT_IN_SKETCH: [&str; 2] =
    [include_str!("../../../library/sketch/rectangle.json"), include_str!("../../../library/sketch/center_rectangle.json")];

/// Where an element comes from: the most specific wins a name.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Source {
    BuiltIn,
    /// `library` beside the program: the engineer's or the company's own.
    Program,
    /// `library` of the model's project.
    Project,
}

#[derive(Clone, Debug)]
pub struct Entry {
    pub source: Source,
    pub element: Value,
}

impl Entry {
    pub fn name(&self) -> &str {
        self.element["name"].as_str().unwrap_or_default()
    }
}

/// The elements of a level, and the files that are not elements, each with why.
#[derive(Clone, Debug, Default)]
pub struct Library {
    pub entries: Vec<Entry>,
    pub skipped: Vec<String>,
}

impl Library {
    pub fn get(&self, name: &str) -> Option<&Entry> {
        self.entries.iter().find(|e| e.name() == name)
    }
}

/// The project the model file `model` belongs to: the nearest folder above it with
/// [`PROJECT_FILE`].
pub fn project_root(model: &Path) -> Option<PathBuf> {
    model.ancestors().skip(1).find(|d| d.join(PROJECT_FILE).is_file()).map(Path::to_path_buf)
}

/// The elements of `level`: built in, then beside the program, then in the project of the
/// model at `model`, each folder by file name. One of a name already listed takes its place.
pub fn load(level: &str, model: Option<&Path>) -> Library {
    let mut lib = Library::default();
    let put = |lib: &mut Library, source: Source, element: Value| match lib.entries.iter_mut().find(|e| e.element["name"] == element["name"]) {
        Some(e) => *e = Entry { source, element },
        None => lib.entries.push(Entry { source, element }),
    };
    if level == "sketch" {
        for t in BUILT_IN_SKETCH {
            if let Ok(e) = serde_json::from_str(t) {
                put(&mut lib, Source::BuiltIn, e);
            }
        }
    }
    let program = std::env::current_exe().ok().and_then(|e| e.parent().map(Path::to_path_buf));
    let project = model.and_then(project_root);
    for (source, dir) in [(Source::Program, program), (Source::Project, project)] {
        let Some(dir) = dir.map(|d| d.join("library").join(level)) else { continue };
        let mut files: Vec<PathBuf> = std::fs::read_dir(&dir)
            .into_iter()
            .flatten()
            .filter_map(|e| e.ok().map(|e| e.path()))
            .filter(|p| p.extension().is_some_and(|x| x == "json"))
            .collect();
        files.sort();
        for f in files {
            let element = std::fs::read_to_string(&f)
                .map_err(|e| e.to_string())
                .and_then(|t| serde_json::from_str::<Value>(&t).map_err(|e| e.to_string()))
                .and_then(|e| check(level, &e).map(|()| e));
            match element {
                Ok(e) => put(&mut lib, source, e),
                Err(why) => lib.skipped.push(format!("{}: {why}", f.display())),
            }
        }
    }
    lib
}

/// Whether `e` is an element of `level`: why not, if it is not.
pub fn check(level: &str, e: &Value) -> Result<(), String> {
    if level != "sketch" {
        return Err(format!("no elements of level `{level}` yet (levels: {LEVELS:?})"));
    }
    if !e["name"].as_str().is_some_and(|n| !n.is_empty()) {
        return Err("an element needs a `name`".into());
    }
    let sketch: Sketch = serde_json::from_value(e["sketch"].clone()).map_err(|err| format!("`sketch`: {err}"))?;
    let anchors = anchors(e);
    if anchors.is_empty() || anchors.len() > 2 {
        return Err("`anchors`: one or two of its points".into());
    }
    if let Some(a) = anchors.iter().find(|a| !sketch.points.contains_key(*a)) {
        return Err(format!("anchor `{a}` is not a point of the sketch"));
    }
    match e.get("fit").map(|f| f.as_str()) {
        None | Some(Some("stretch")) => {}
        Some(Some("align")) if anchors.len() == 2 => {}
        _ => return Err("`fit`: `stretch`, or `align` with two anchors".into()),
    }
    let names = parameter_names(e)?;
    for (preset, values) in e.get("presets").and_then(Value::as_object).into_iter().flatten() {
        let values = values.as_object().ok_or_else(|| format!("preset `{preset}`: parameter values by name"))?;
        if let Some(n) = values.keys().find(|n| !names.contains(n)) {
            return Err(format!("preset `{preset}`: no parameter `{n}`"));
        }
    }
    Ok(())
}

/// The anchors of an element: one or two of its points, where it is put.
pub fn anchors(e: &Value) -> Vec<String> {
    e["anchors"].as_array().into_iter().flatten().filter_map(|a| a.as_str().map(str::to_string)).collect()
}

/// The parameters' names, in the order they are asked.
pub fn parameter_names(e: &Value) -> Result<Vec<String>, String> {
    let mut names: Vec<String> = Vec::new();
    for p in e.get("parameters").and_then(Value::as_array).into_iter().flatten() {
        let n = p["name"].as_str().filter(|n| valid_name(n)).ok_or_else(|| format!("parameter {p}: `name` must be a name"))?;
        if !(p["value"].is_number() || p["value"].is_string()) {
            return Err(format!("parameter `{n}`: `value` is a number or an expression"));
        }
        if names.iter().any(|m| m == n) {
            return Err(format!("parameter `{n}` given twice"));
        }
        names.push(n.to_string());
    }
    Ok(names)
}

/// The value of every parameter for placing element `e` by `at` (a spot per anchor given): the
/// element's own, over them the preset's, over those `given`. With both anchors' spots of a
/// stretched element, a parameter neither given nor in the preset that a dimension holds alone
/// (`"value": "width"`) is what the spots make it, measured (D98: clicks answer the same
/// questions). Each is a number
/// or an expression; `lookup` gives the model's parameter values.
pub fn answers(
    e: &Value,
    preset: Option<&str>,
    given: &Map<String, Value>,
    at: &[[f64; 2]],
    lookup: &impl Fn(&str) -> Result<f64, ExprError>,
) -> Result<BTreeMap<String, Value>, String> {
    let mut out: BTreeMap<String, Value> = BTreeMap::new();
    for p in e.get("parameters").and_then(Value::as_array).into_iter().flatten() {
        if let Some(n) = p["name"].as_str() {
            out.insert(n.to_string(), p["value"].clone());
        }
    }
    let preset = match preset {
        Some(name) => Some(e["presets"].get(name).and_then(Value::as_object).ok_or_else(|| format!("no preset `{name}`"))?),
        None => None,
    };
    let mut open: Vec<String> = out.keys().cloned().collect();
    for (n, v) in preset.into_iter().flatten().chain(given) {
        let slot = out.get_mut(n).ok_or_else(|| format!("no parameter `{n}`"))?;
        if !(v.is_number() || v.is_string()) {
            return Err(format!("parameter `{n}`: a number or an expression"));
        }
        *slot = v.clone();
        open.retain(|o| o != n);
    }
    // Spots answer only by stretching: an aligned element's second spot is where it joins.
    if at.len() < 2 || anchors(e).len() < 2 || e["fit"] == "align" {
        return Ok(out);
    }
    let held: Vec<String> = open.into_iter().filter(|n| dimensions(e).any(|v| v.as_str().is_some_and(|t| t.trim() == n))).collect();
    if !held.is_empty() {
        let (_, solved) = solve_placed(e, &out, at, &held, lookup)?;
        for n in held {
            if let Some(v) = solved.measures.get(&n) {
                out.insert(n, json!(v));
            }
        }
    }
    Ok(out)
}

/// The values of the element's dimensions, as given in it.
fn dimensions(e: &Value) -> impl Iterator<Item = &Value> {
    e["sketch"]["constraints"].as_array().into_iter().flatten().filter_map(|c| c.get("value").or_else(|| c.get("degrees")))
}

/// `value` (a dimension's: a number, or an expression of the element's parameters) with the
/// parameters' values put in: a number once nothing is left to look up, else an expression of
/// the model's parameters.
fn resolve(value: &Value, values: &BTreeMap<String, Value>) -> Result<Value, String> {
    let Some(text) = value.as_str() else { return Ok(value.clone()) };
    let text_of = |v: &Value| v.as_str().map_or_else(|| v.to_string(), str::to_string);
    let mut text = text.to_string();
    // A parameter may be given by others: a round for each, and one more that meets none.
    for round in 0.. {
        let met = std::cell::Cell::new(false);
        text = substitute(&text, &|n: &str| {
            let v = values.get(n)?;
            met.set(true);
            Some(text_of(v))
        })
        .map_err(|err| format!("`{text}`: {err}"))?;
        if !met.get() {
            break;
        }
        if round == values.len() {
            return Err(format!("the parameters depend on themselves: `{}`", value.as_str().unwrap_or_default()));
        }
    }
    Ok(match eval(&text, &|n| Err(ExprError::Unknown(n.to_string()))) {
        Ok(v) => json!(v),
        Err(_) => json!(text),
    })
}

/// How an element's coordinates go to the sketch's: `p ↦ to + m·(p − from)`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Placement {
    from: [f64; 2],
    to: [f64; 2],
    m: [[f64; 2]; 2],
}

impl Placement {
    pub fn map(&self, p: [f64; 2]) -> [f64; 2] {
        let d = [p[0] - self.from[0], p[1] - self.from[1]];
        [self.to[0] + self.m[0][0] * d[0] + self.m[0][1] * d[1], self.to[1] + self.m[1][0] * d[0] + self.m[1][1] * d[1]]
    }

    /// How a circle's diameter scales: the mean of the two stretches.
    pub fn scale(&self) -> f64 {
        libm::sqrt((self.m[0][0] * self.m[1][1] - self.m[0][1] * self.m[1][0]).abs())
    }
}

/// The placement of element `e` by `at`, one spot per anchor given: the first anchor onto the
/// first spot; with a second, stretched along the sketch's axes so the second lands on it
/// (`fit: stretch`: a rectangle by two corners), or turned and scaled alike so it lies toward it
/// (`fit: align`: a section between two points of a chain).
pub fn placement(e: &Value, at: &[[f64; 2]]) -> Option<Placement> {
    let names = anchors(e);
    let point = |n: &str| {
        let p = &e["sketch"]["points"][n];
        Some([p[0].as_f64()?, p[1].as_f64()?])
    };
    let a = point(names.first()?)?;
    let c1 = *at.first()?;
    let mut m = [[1.0, 0.0], [0.0, 1.0]];
    if let (Some(b), Some(c2)) = (names.get(1), at.get(1)) {
        let b = point(b)?;
        let (v, w) = ([b[0] - a[0], b[1] - a[1]], [c2[0] - c1[0], c2[1] - c1[1]]);
        if e["fit"] == "align" {
            let vv = v[0] * v[0] + v[1] * v[1];
            if vv > 0.0 && (w[0] != 0.0 || w[1] != 0.0) {
                let (dot, cross) = ((v[0] * w[0] + v[1] * w[1]) / vv, (v[0] * w[1] - v[1] * w[0]) / vv);
                m = [[dot, -cross], [cross, dot]];
            }
        } else {
            let s = |k: usize| if v[k] == 0.0 { 1.0 } else { w[k] / v[k] };
            m = [[s(0), 0.0], [0.0, s(1)]];
        }
    }
    Some(Placement { from: a, to: c1, m })
}

/// Where an anchor goes: a spot, and the point of the sketch it lands on, if any; the anchor is
/// made coincident with that point, not merged into it (D98).
#[derive(Clone, Debug, PartialEq)]
pub struct Anchor {
    pub at: [f64; 2],
    pub point: Option<String>,
}

/// The element's sketch placed by `at` with parameter values `values` put in: its points moved,
/// diameters given as numbers scaled, dimensions' values resolved; the dimensions held by a
/// parameter in `measure` become reference dimensions named by it.
fn placed_sketch(e: &Value, values: &BTreeMap<String, Value>, at: &[[f64; 2]], measure: &[String]) -> Result<Value, String> {
    let place = placement(e, at).ok_or("the element has no anchor point")?;
    let mut sketch = e["sketch"].clone();
    let mapped = |q: &Value| -> Option<[f64; 2]> { Some(place.map([q.get(0)?.as_f64()?, q.get(1)?.as_f64()?])) };
    for q in sketch.get_mut("points").and_then(Value::as_object_mut).into_iter().flat_map(|m| m.values_mut()) {
        *q = json!(mapped(q).ok_or("a point is not [x, y]")?);
    }
    for c in sketch.get_mut("circles").and_then(Value::as_object_mut).into_iter().flat_map(|m| m.values_mut()) {
        // A circle keeps its shape: a diameter given as a number scales with the placement.
        c["diameter"] = match c["diameter"].as_f64() {
            Some(d) => json!(d * place.scale()),
            None => resolve(&c["diameter"], values)?,
        };
    }
    let mut constraints = Vec::new();
    for mut c in sketch.get("constraints").and_then(Value::as_array).cloned().unwrap_or_default() {
        let key = ["value", "degrees"].into_iter().find(|k| c.get(*k).is_some());
        if let Some(k) = key {
            if let Some(n) = c[k].as_str().map(str::trim).filter(|t| measure.iter().any(|m| m == t)) {
                let n = n.to_string();
                let mut m = c.clone();
                if let Some(o) = m.as_object_mut() {
                    o.remove(k);
                }
                sketch["measures"][n.as_str()] = m;
                continue;
            }
            c[k] = resolve(&c[k], values)?;
        }
        if let Some(q) = c.get("at").and_then(mapped) {
            c["at"] = json!(q);
        }
        constraints.push(c);
    }
    sketch["constraints"] = json!(constraints);
    Ok(sketch)
}

/// The element placed by `at` with parameter values `values` put in (see [`placed_sketch`]),
/// every value a number: the model's parameters from `lookup`. Not solved.
fn placed_with(
    e: &Value,
    values: &BTreeMap<String, Value>,
    at: &[[f64; 2]],
    measure: &[String],
    lookup: &impl Fn(&str) -> Result<f64, ExprError>,
) -> Result<Sketch, String> {
    let sketch: Sketch = serde_json::from_value(placed_sketch(e, values, at, measure)?).map_err(|e| e.to_string())?;
    sketch.resolve(lookup)
}

/// The placed element solved on its own, its first anchor held at the first spot.
fn solve_placed(
    e: &Value,
    values: &BTreeMap<String, Value>,
    at: &[[f64; 2]],
    measure: &[String],
    lookup: &impl Fn(&str) -> Result<f64, ExprError>,
) -> Result<(Sketch, datum_sketch::Solved), String> {
    let mut sketch = placed_with(e, values, at, measure, lookup)?;
    let first = anchors(e).into_iter().next().ok_or("the element has no anchor")?;
    let q = at.first().ok_or("no spot for the anchor")?;
    sketch.constraints.push(Constraint::Fix { point: first, at: Some(Vec2::new(q[0], q[1])) });
    let solved = datum_sketch::solve(&sketch).map_err(|e| e.to_string())?;
    Ok((sketch, solved))
}

/// Element `e` placed by `at` with parameter values `values` (see [`answers`]), every value a
/// number, as drawn: not solved. What a preview shows where [`shape`] fails.
pub fn placed(
    e: &Value,
    values: &BTreeMap<String, Value>,
    at: &[[f64; 2]],
    lookup: &impl Fn(&str) -> Result<f64, ExprError>,
) -> Result<Sketch, String> {
    placed_with(e, values, at, &[], lookup)
}

/// Element `e` placed as [`placed`], solved on its own with its first anchor held at the first
/// spot: what a preview shows, and where [`place`] puts its points.
pub fn shape(
    e: &Value,
    values: &BTreeMap<String, Value>,
    at: &[[f64; 2]],
    lookup: &impl Fn(&str) -> Result<f64, ExprError>,
) -> Result<Sketch, String> {
    let (mut sketch, solved) = solve_placed(e, values, at, &[], lookup)?;
    sketch.points = solved.points;
    Ok(sketch)
}

/// Places element `e` into `patch` with parameter values `values` (see [`answers`]) by `at`, a
/// spot for its first anchor or for each. Its points start where [`shape`] puts them (where
/// the element alone does not solve, where the spots put them). Its elements take the names
/// `names` gives them (element name → name in the sketch, free), the others new ones. Returns
/// every element name with the name it got.
pub fn place(
    patch: &mut SketchPatch,
    e: &Value,
    values: &BTreeMap<String, Value>,
    at: &[Anchor],
    names: &BTreeMap<String, String>,
    construction: bool,
    lookup: &impl Fn(&str) -> Result<f64, ExprError>,
) -> Result<BTreeMap<String, String>, String> {
    let anchor_names = anchors(e);
    if at.is_empty() || at.len() > anchor_names.len() {
        return Err(format!("`at`: a spot for the first anchor or for each of its {}", anchor_names.len()));
    }
    let spots: Vec<[f64; 2]> = at.iter().map(|a| a.at).collect();
    let sketch = placed_sketch(e, values, &spots, &[])?;
    let solved = shape(e, values, &spots, lookup).ok().map(|s| s.points);
    if let Some(n) = names.keys().find(|n| !["points", "lines", "arcs", "circles", "measures"].iter().any(|k| sketch[*k].get(n.as_str()).is_some())) {
        return Err(format!("`names`: the element has no `{n}`"));
    }
    let mut renamed: BTreeMap<String, String> = BTreeMap::new();
    for (key, prefix) in [("points", "p"), ("lines", "l"), ("arcs", "a"), ("circles", "c"), ("measures", "d")] {
        for n in sketch[key].as_object().into_iter().flat_map(|m| m.keys()) {
            let new = match names.get(n) {
                Some(given) => {
                    patch.claim(given)?;
                    given.clone()
                }
                None => patch.fresh(prefix),
            };
            renamed.insert(n.clone(), new);
        }
    }
    let rename = |v: &Value| {
        let mut v = v.clone();
        crate::visit_names(&mut v, &mut |s| {
            if let Some(n) = renamed.get(s.as_str()) {
                *s = n.clone();
            }
        });
        v
    };
    for (n, q) in sketch["points"].as_object().into_iter().flatten() {
        let q = solved.as_ref().and_then(|s| s.get(n)).map_or_else(|| q.clone(), |q| json!([q.x, q.y]));
        patch.set("points", &renamed[n], q);
    }
    for (a, spot) in anchor_names.iter().zip(at) {
        if let Some(p) = &spot.point {
            patch.constrain(json!({"type": "coincident", "points": [renamed[a], p]}));
        }
    }
    for key in ["lines", "arcs", "circles"] {
        for (n, v) in sketch[key].as_object().into_iter().flatten() {
            patch.add(key, &renamed[n], rename(v), construction);
        }
    }
    for (n, v) in sketch["measures"].as_object().into_iter().flatten() {
        patch.set("measures", &renamed[n], rename(v));
    }
    for c in sketch["constraints"].as_array().into_iter().flatten() {
        patch.constrain(rename(c));
    }
    Ok(renamed)
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    fn none(n: &str) -> Result<f64, ExprError> {
        Err(ExprError::Unknown(n.to_string()))
    }

    /// A slot-like piece: a line whose length is a parameter, the other derived from it.
    fn element() -> Value {
        json!({"name": "Bar", "anchors": ["a", "b"], "fit": "align",
               "parameters": [{"name": "len", "value": 40}, {"name": "half", "value": "len / 2"}],
               "presets": {"short": {"len": 10}},
               "sketch": {"points": {"a": [0, 0], "b": [1, 0], "m": [0.5, 0]},
                          "lines": {"bar": {"from": "a", "to": "b"}},
                          "constraints": [{"type": "length", "line": "bar", "value": "len"},
                                          {"type": "distance", "points": ["a", "m"], "value": "half"}]}})
    }

    #[test]
    fn the_built_ins_are_elements() {
        let lib = load("sketch", None);
        let names: Vec<&str> = lib.entries.iter().filter(|e| e.source == Source::BuiltIn).map(Entry::name).collect();
        assert_eq!(names, ["Rectangle", "Center rectangle"]);
        assert!(check("sketch", &element()).is_ok());
        assert!(check("part", &element()).unwrap_err().contains("no elements of level"));
        let mut bad = element();
        bad["presets"]["short"]["width"] = json!(3);
        assert!(check("sketch", &bad).unwrap_err().contains("no parameter `width`"));
    }

    #[test]
    fn answers_go_into_the_dimensions() {
        let e = element();
        let v = answers(&e, None, &Map::new(), &[], &none).unwrap();
        assert_eq!(resolve(&json!("len"), &v).unwrap(), json!(40.0));
        assert_eq!(resolve(&json!("half"), &v).unwrap(), json!(20.0), "a parameter of another");
        let v = answers(&e, Some("short"), &Map::new(), &[], &none).unwrap();
        assert_eq!(resolve(&json!("half"), &v).unwrap(), json!(5.0));
        // An answer of the model's parameters stays an expression of them.
        let given = json!({"len": "w - 2*gap"});
        let v = answers(&e, Some("short"), given.as_object().unwrap(), &[], &none).unwrap();
        assert_eq!(resolve(&json!("2 * len"), &v).unwrap(), json!("2 * (w - 2*gap)"));
        assert_eq!(resolve(&json!("half"), &v).unwrap(), json!("((w - 2*gap) / 2)"));
        assert!(answers(&e, Some("long"), &Map::new(), &[], &none).is_err());
        let looped = BTreeMap::from([("x".to_string(), json!("y + 1")), ("y".to_string(), json!("x"))]);
        assert!(resolve(&json!("x"), &looped).unwrap_err().contains("depend on themselves"));
    }

    /// Aligned between two points of a chain: turned toward the second, its anchors coincident
    /// with the points, not merged into them.
    #[test]
    fn placed_between_two_points() {
        let feature = json!({"sketch": {"points": {"p1": [0, 0], "p2": [0, 10], "p4": [6, 18]}, "lines": {"l1": {"from": "p1", "to": "p2"}}}});
        let mut patch = SketchPatch::new(&feature);
        let e = element();
        let v = answers(&e, None, &Map::new(), &[], &none).unwrap();
        let at = [Anchor { at: [0.0, 10.0], point: Some("p2".into()) }, Anchor { at: [6.0, 18.0], point: Some("p4".into()) }];
        let names = BTreeMap::from([("bar".to_string(), "l3".to_string())]);
        let renamed = place(&mut patch, &e, &v, &at, &names, false, &none).unwrap();
        assert_eq!(renamed["bar"], "l3", "named as asked");
        assert_eq!((renamed["a"].as_str(), renamed["b"].as_str(), renamed["m"].as_str()), ("p5", "p6", "p7"));
        let p = Value::Object(patch.done().unwrap());
        let near = |n: &str, q: [f64; 2]| (p["points"][n][0].as_f64().unwrap() - q[0]).hypot(p["points"][n][1].as_f64().unwrap() - q[1]) < 1e-9;
        assert!(near("p6", [24.0, 42.0]), "40 long, as its dimension says, turned along p2 → p4: {p}");
        assert!(near("p7", [12.0, 26.0]), "halfway: {p}");
        let c = p["add_constraints"].as_array().unwrap();
        assert!(c.contains(&json!({"type": "coincident", "points": ["p5", "p2"]})), "{p}");
        assert!(c.contains(&json!({"type": "length", "line": "l3", "value": 40.0})), "{p}");
        let mut again = SketchPatch::new(&json!({"sketch": {"points": {"p1": [0, 0]}, "lines": {"l3": {"from": "p1", "to": "p1"}}}}));
        assert!(place(&mut again, &e, &v, &at, &names, false, &none).unwrap_err().contains("`l3` is taken"));
    }

    /// A rectangle with its width and height as parameters, by two corners: what is typed holds,
    /// what is not comes from the clicks.
    #[test]
    fn clicks_answer_what_is_not_typed() {
        let e = json!({"name": "Plate", "anchors": ["a", "c"],
            "parameters": [{"name": "width", "value": 100}, {"name": "height", "value": 50}],
            "sketch": {"points": {"a": [0, 0], "b": [1, 0], "c": [1, 1], "d": [0, 1]},
                       "lines": {"bottom": {"from": "a", "to": "b"}, "right": {"from": "b", "to": "c"}, "top": {"from": "c", "to": "d"}, "left": {"from": "d", "to": "a"}},
                       "constraints": [{"type": "horizontal", "line": "bottom"}, {"type": "vertical", "line": "right"},
                                       {"type": "horizontal", "line": "top"}, {"type": "vertical", "line": "left"},
                                       {"type": "length", "line": "bottom", "value": "width"}, {"type": "length", "line": "left", "value": "height"}]}});
        let at = [[10.0, 10.0], [40.0, 30.0]];
        let given = json!({"height": 5});
        let v = answers(&e, None, given.as_object().unwrap(), &at, &none).unwrap();
        let width = v["width"].as_f64().unwrap();
        assert!((width - 30.0).abs() < 1e-9, "measured between the clicks: {v:?}");
        assert_eq!(v["height"], json!(5));
        // One click: the element's own size, put there.
        assert_eq!(answers(&e, None, &Map::new(), &at[..1], &none).unwrap()["width"], json!(100));
        let q = shape(&e, &v, &at, &none).unwrap().points;
        assert_eq!(q["a"], Vec2::new(10.0, 10.0), "the first anchor where clicked");
        assert!((q["c"].x - 40.0).abs() < 1e-9 && (q["c"].y - 15.0).abs() < 1e-9, "5 high as typed: {q:?}");
    }
}
