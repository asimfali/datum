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
use crate::workspace;

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
    /// The file it was read from; none for a built-in one.
    pub file: Option<PathBuf>,
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

/// The elements of `level`: built in, then beside the program, then in `project` (its root),
/// each folder by file name. One of a name already listed from a less specific place takes its
/// place; two of a name in one folder are both listed.
pub fn load(level: &str, project: Option<&Path>) -> Library {
    let mut lib = Library::default();
    // One of a name from a more specific place takes the place of the other; two of a name in
    // one folder are both kept.
    let put = |lib: &mut Library, entry: Entry| match lib
        .entries
        .iter_mut()
        .find(|e| e.element["name"] == entry.element["name"] && e.source != entry.source)
    {
        Some(e) => *e = entry,
        None => lib.entries.push(entry),
    };
    if level == "sketch" {
        for t in BUILT_IN_SKETCH {
            if let Ok(e) = serde_json::from_str(t) {
                put(&mut lib, Entry { source: Source::BuiltIn, element: e, file: None });
            }
        }
    }
    let program = std::env::current_exe().ok().and_then(|e| e.parent().map(Path::to_path_buf));
    for (source, root) in [(Source::Program, program), (Source::Project, project.map(Path::to_path_buf))] {
        let Some(root) = root else { continue };
        let dir = match workspace::folder_in(&root, "library") {
            Ok(d) => d,
            Err(why) => {
                lib.skipped.push(why);
                continue;
            }
        };
        // Renamed in the project with elements left in the old folder: they are not read, and
        // that is said.
        let old = root.join("library").join(level);
        if dir != root.join("library") && old.is_dir() {
            lib.skipped.push(format!("{}: not read, the project keeps its library in {}", old.display(), dir.display()));
        }
        let dir = dir.join(level);
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
                Ok(e) => put(&mut lib, Entry { source, element: e, file: Some(f) }),
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
    let own: Vec<&String> = sketch.points.keys().chain(sketch.lines.keys()).chain(sketch.arcs.keys()).chain(sketch.circles.keys()).collect();
    for l in e.get("links").and_then(Value::as_array).into_iter().flatten() {
        let c = &l["constraint"];
        let label = label_of(c);
        serde_json::from_value::<Constraint>(c.clone()).map_err(|err| format!("link `{label}`: {err}"))?;
        let mut names = Vec::new();
        crate::visit_names(&mut c.clone(), &mut |n| names.push(n.clone()));
        if names.iter().filter(|n| *n == SOCKET).count() != 1 || names.iter().any(|n| n != SOCKET && !own.contains(&n)) {
            return Err(format!("link `{label}`: one `{SOCKET}` for the side outside, the rest the element's own"));
        }
        if !l["kind"].as_str().is_some_and(|k| KINDS.contains(&k)) {
            return Err(format!("link `{label}`: `kind` is one of {KINDS:?}"));
        }
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
    let dims = dimensions(e);
    let held: Vec<String> = open.into_iter().filter(|n| dims.iter().any(|d| d.value.as_str().is_some_and(|t| t.trim() == n))).collect();
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

/// How [`place`] puts an element in.
#[derive(Clone, Copy, Default)]
pub struct Put<'a> {
    /// Its lines, arcs and circles are construction geometry.
    pub construction: bool,
    /// The name of the sketch group its points and curves make, a preset's (D176): the name
    /// itself, or the first free `name (2)`, `name (3)`, …
    pub group: Option<&'a str>,
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
    put: Put,
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
            patch.add(key, &renamed[n], rename(v), put.construction);
        }
    }
    for (n, v) in sketch["measures"].as_object().into_iter().flatten() {
        patch.set("measures", &renamed[n], rename(v));
    }
    for c in sketch["constraints"].as_array().into_iter().flatten() {
        patch.constrain(rename(c));
    }
    if let Some(group) = put.group {
        let members: Vec<&String> = ["points", "lines", "arcs", "circles"]
            .iter()
            .flat_map(|k| sketch[*k].as_object().into_iter().flat_map(|m| m.keys()))
            .map(|n| &renamed[n])
            .collect();
        let group = patch.claim_free(group);
        patch.set("groups", &group, json!(members));
    }
    Ok(renamed)
}

/// What stands for the outside side of a socket's constraint (see [`Link`]).
pub const SOCKET: &str = "?";

/// What a socket takes: a point, a line, an arc or circle, or any curve (a projected edge).
pub const KINDS: [&str; 4] = ["point", "line", "circle", "curve"];

/// What name `n` of sketch feature `feature` is, as a socket would take it.
fn kind_of(feature: &Value, n: &str) -> Option<&'static str> {
    let sketch = &feature["sketch"];
    let has = |key: &str| sketch[key].get(n).is_some();
    if has("points") || n == datum_ops::ORIGIN || n.contains('.') {
        Some("point")
    } else if has("lines") {
        Some("line")
    } else if has("arcs") || has("circles") {
        Some("circle")
    } else {
        feature["project"].get(n).map(|_| "curve")
    }
}

/// A socket of a placed element: a constraint between it and something outside it, completed
/// by naming that (D116). Kept when the piece was taken from a sketch with the constraint
/// reaching out of it: parallel to a line outside, concentric with a circle outside, a distance
/// to a point outside.
#[derive(Clone, Debug, PartialEq)]
pub struct Link {
    /// How it reads, by the element's names (`perpendicular l5 ?`): how the agent names it.
    pub label: String,
    /// How it reads once placed, by the sketch's names.
    pub shown: String,
    /// What it takes: one of [`KINDS`].
    pub kind: String,
    /// The constraint, by the sketch's names, [`SOCKET`] for the side outside.
    pub constraint: Value,
}

impl Link {
    /// The constraint with `target` (a name in the sketch) for the side outside.
    pub fn to(&self, target: &str) -> Value {
        let mut c = self.constraint.clone();
        crate::visit_names(&mut c, &mut |s| {
            if s == SOCKET {
                *s = target.to_string();
            }
        });
        c
    }
}

/// Whether `target` is something of sketch `placed` a socket of `kind` takes.
pub fn fits(placed: &datum_ops::Placed, kind: &str, target: &str) -> bool {
    let line = placed.lines.contains_key(target);
    let circle = placed.arcs.contains_key(target) || placed.circles.contains_key(target);
    match kind {
        "point" => placed.solved.points.contains_key(target),
        "line" => line,
        "circle" => circle,
        _ => line || circle,
    }
}

/// How a constraint reads: its type and the names it ties, `perpendicular l5 ?`.
fn label_of(c: &Value) -> String {
    let mut names = Vec::new();
    crate::visit_names(&mut c.clone(), &mut |n| names.push(n.clone()));
    format!("{} {}", c["type"].as_str().unwrap_or_default(), names.join(" "))
}

/// The sockets of element `e` placed with parameter values `values` and its names `renamed`
/// (as [`place`] gave them), in order.
pub fn links(e: &Value, values: &BTreeMap<String, Value>, renamed: &BTreeMap<String, String>) -> Result<Vec<Link>, String> {
    let mut out = Vec::new();
    for l in e["links"].as_array().into_iter().flatten() {
        let mut c = l["constraint"].clone();
        let label = label_of(&c);
        crate::visit_names(&mut c, &mut |s| {
            if let Some(n) = renamed.get(s.as_str()) {
                *s = n.clone();
            }
        });
        for k in ["value", "degrees"] {
            if c.get(k).is_some() {
                c[k] = resolve(&c[k], values)?;
            }
        }
        out.push(Link { label, shown: label_of(&c), kind: l["kind"].as_str().unwrap_or("curve").to_string(), constraint: c });
    }
    Ok(out)
}

/// The piece of sketch feature `feature` (its JSON) picked: curves `curves` with their points,
/// points `picked`, and the constraints and reference dimensions among them, as element `name`
/// with its point lowest and leftmost (by `points`, the solved ones) as anchor. Projections and
/// the origin stay behind, and so does what ties to them. With `params` (the model's parameter
/// values), the model's parameters its dimensions name become the element's (D98: named
/// dimensions become parameters), with their values now, asked in the order the dimensions name
/// them; without, the dimensions stay as they are (a copy to paste in the same model).
pub fn extract(
    feature: &Value,
    points: &BTreeMap<String, Vec2>,
    curves: &[String],
    picked: &[String],
    name: &str,
    params: Option<&BTreeMap<String, f64>>,
) -> Option<Value> {
    let sketch = &feature["sketch"];
    let own = |n: &str| sketch["points"].get(n).is_some();
    let mut names: Vec<String> = picked.iter().filter(|n| own(n)).cloned().collect();
    let mut out = Map::new();
    for n in curves {
        for key in ["lines", "arcs", "circles"] {
            if let Some(v) = sketch[key].get(n) {
                let mut v = v.clone();
                let mut ends = Vec::new();
                crate::visit_names(&mut v, &mut |s| ends.push(s.clone()));
                if ends.iter().all(|e| own(e)) {
                    out.entry(key).or_insert_with(|| json!({}))[n.as_str()] = v;
                    names.push(n.clone());
                    names.extend(ends);
                }
            }
        }
    }
    names.sort();
    names.dedup();
    let at: Map<String, Value> = names.iter().filter(|n| own(n)).filter_map(|n| points.get(n).map(|q| (n.clone(), json!([q.x, q.y])))).collect();
    let anchor = at
        .iter()
        .filter_map(|(n, q)| Some((n, q[0].as_f64()?, q[1].as_f64()?)))
        .min_by(|a, b| (a.2, a.1).partial_cmp(&(b.2, b.1)).unwrap_or(std::cmp::Ordering::Equal))?
        .0
        .clone();
    // Constraints and measures whose names are all taken come along.
    let within = |v: &Value| {
        let mut v = v.clone();
        let mut all = true;
        crate::visit_names(&mut v, &mut |s| all &= names.contains(s));
        all
    };
    let constraints: Vec<Value> = sketch["constraints"].as_array().into_iter().flatten().filter(|c| within(c)).cloned().collect();
    let measures: Map<String, Value> =
        sketch["measures"].as_object().into_iter().flatten().filter(|(_, v)| within(v)).map(|(n, v)| (n.clone(), v.clone())).collect();
    // Constraints with one side outside: kept as sockets, completed when the element is placed.
    let mut links = Vec::new();
    for c in sketch["constraints"].as_array().into_iter().flatten().filter(|c| c["type"] != "fix") {
        let mut v = c.clone();
        let mut outside: Vec<String> = Vec::new();
        let mut inside = false;
        crate::visit_names(&mut v, &mut |n| match names.contains(n) {
            true => inside = true,
            false if !outside.contains(n) => outside.push(n.clone()),
            false => {}
        });
        let ([n], true) = (&outside[..], inside) else { continue };
        let Some(kind) = kind_of(feature, n) else { continue };
        let n = n.clone();
        crate::visit_names(&mut v, &mut |s| {
            if *s == n {
                *s = SOCKET.to_string();
            }
        });
        links.push(json!({"constraint": v, "kind": kind}));
    }
    let mut element = json!({"name": name, "anchors": [anchor]});
    if !links.is_empty() {
        element["links"] = json!(links);
    }
    if let Some(params) = params {
        let named = std::cell::RefCell::new(Vec::<String>::new());
        let linked = links.iter().map(|l| &l["constraint"]);
        let values = constraints.iter().chain(linked).filter_map(|c| c.get("value").or_else(|| c.get("degrees")));
        let diameters = out.get("circles").and_then(Value::as_object).into_iter().flat_map(|m| m.values()).map(|c| &c["diameter"]);
        for text in values.chain(diameters).filter_map(Value::as_str) {
            let _ = substitute(text, &|n: &str| {
                if params.contains_key(n) && !named.borrow().iter().any(|m| m == n) {
                    named.borrow_mut().push(n.to_string());
                }
                None
            });
        }
        let parameters: Vec<Value> = named.into_inner().into_iter().map(|n| json!({"name": n, "value": params[&n]})).collect();
        if !parameters.is_empty() {
            element["parameters"] = json!(parameters);
        }
    }
    out.insert("points".into(), Value::Object(at));
    out.insert("constraints".into(), json!(constraints));
    if !measures.is_empty() {
        out.insert("measures".into(), Value::Object(measures));
    }
    element["sketch"] = Value::Object(out);
    Some(element)
}

/// A dimension of an element, as a list of them shows it: what it holds (`length l5`,
/// `diameter hole`) and its value as given (a number, a parameter, an expression).
#[derive(Clone, Debug, PartialEq)]
pub struct Dimension {
    pub label: String,
    pub value: Value,
}

/// Where a dimension's value is in the element's JSON: a constraint's `value` or `degrees`, or a
/// circle's `diameter`.
fn dimension_slots(e: &mut Value) -> Vec<(String, &mut Value)> {
    let mut out = Vec::new();
    let sketch = &mut e["sketch"];
    let Some(sketch) = sketch.as_object_mut() else { return out };
    let (mut constraints, mut circles) = (None, None);
    for (k, v) in sketch.iter_mut() {
        match k.as_str() {
            "constraints" => constraints = v.as_array_mut(),
            "circles" => circles = v.as_object_mut(),
            _ => {}
        }
    }
    for c in constraints.into_iter().flatten() {
        let kind = c["type"].as_str().unwrap_or_default().to_string();
        let mut names = Vec::new();
        crate::visit_names(&mut c.clone(), &mut |n| names.push(n.clone()));
        let Some(o) = c.as_object_mut() else { continue };
        if let Some((_, v)) = o.iter_mut().find(|(k, _)| *k == "value" || *k == "degrees") {
            out.push((format!("{kind} {}", names.join(" ")), v));
        }
    }
    for (n, c) in circles.into_iter().flatten() {
        if let Some(v) = c.get_mut("diameter") {
            out.push((format!("diameter {n}"), v));
        }
    }
    out
}

/// The dimensions of element `e`, in the order of its constraints, then its circles.
pub fn dimensions(e: &Value) -> Vec<Dimension> {
    dimension_slots(&mut e.clone()).into_iter().map(|(label, v)| Dimension { label, value: v.clone() }).collect()
}

/// Dimension `label` of element `e` (a number) asked as new parameter `name` when the element is
/// placed: its value now is the parameter's, the last asked.
pub fn parametrize(e: &mut Value, label: &str, name: &str) -> Result<(), String> {
    if !valid_name(name) {
        return Err(format!("`{name}` is not a name: a letter or _, then letters, digits or _"));
    }
    if parameter_names(e)?.iter().any(|n| n == name) {
        return Err(format!("the element has a parameter `{name}` already"));
    }
    let mut slots = dimension_slots(e);
    let (_, slot) = slots.iter_mut().find(|(l, _)| l == label).ok_or_else(|| format!("no dimension `{label}`"))?;
    let value = slot.as_f64().ok_or_else(|| format!("`{label}` is not a number"))?;
    **slot = json!(name);
    if !e["parameters"].is_array() {
        e["parameters"] = json!([]);
    }
    if let Some(list) = e["parameters"].as_array_mut() {
        list.push(json!({"name": name, "value": value}));
    }
    Ok(())
}

/// Parameter `from` of element `e` renamed `to`, in the dimensions that use it and in its place
/// among the questions.
pub fn rename_parameter(e: &mut Value, from: &str, to: &str) -> Result<(), String> {
    if !valid_name(to) {
        return Err(format!("`{to}` is not a name: a letter or _, then letters, digits or _"));
    }
    if from == to {
        return Ok(());
    }
    if parameter_names(e)?.iter().any(|n| n == to) {
        return Err(format!("the element has a parameter `{to}` already"));
    }
    let p = e["parameters"].as_array_mut().and_then(|l| l.iter_mut().find(|p| p["name"] == from)).ok_or_else(|| format!("no parameter `{from}`"))?;
    p["name"] = json!(to);
    for (_, v) in dimension_slots(e) {
        if let Some(t) = v.as_str() {
            *v = json!(substitute(t, &|n: &str| (n == from).then(|| to.to_string())).map_err(|err| err.to_string())?.replace(&format!("({to})"), to));
        }
    }
    Ok(())
}

/// Parameter `name` of element `e` taken back: the dimensions it alone gives get its value as a
/// number. An error if another dimension uses it within an expression.
pub fn unparametrize(e: &mut Value, name: &str) -> Result<(), String> {
    let list = e["parameters"].as_array().ok_or("the element has no parameters")?;
    let value = list.iter().find(|p| p["name"] == name).map(|p| p["value"].clone()).ok_or_else(|| format!("no parameter `{name}`"))?;
    let used =
        |v: &Value| v.as_str().is_some_and(|t| t.trim() != name && substitute(t, &|n: &str| (n == name).then(String::new)).is_ok_and(|s| s != t));
    if let Some((label, _)) = dimension_slots(e).into_iter().find(|(_, v)| used(v)) {
        return Err(format!("`{label}` uses `{name}` in an expression"));
    }
    for (_, v) in dimension_slots(e) {
        if v.as_str().is_some_and(|t| t.trim() == name) {
            *v = value.clone();
        }
    }
    let none_left = e["parameters"].as_array_mut().is_some_and(|list| {
        list.retain(|p| p["name"] != name);
        list.is_empty()
    });
    if none_left && let Some(o) = e.as_object_mut() {
        o.remove("parameters");
    }
    Ok(())
}

/// Where new elements of `level` are saved unless put elsewhere: `<level>` in the library
/// folder of the workspace.
pub fn folder(level: &str, model: Option<&Path>) -> Result<PathBuf, String> {
    Ok(workspace::folder("library", model)?.join(level))
}

/// A file name for element `name`: what a file system would refuse, replaced.
pub fn file_name(name: &str) -> String {
    let stem: String = name.chars().map(|c| if c.is_control() || r#"<>:"/\|?*"#.contains(c) { '_' } else { c }).collect();
    format!("{}.json", stem.trim())
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
        let renamed = place(&mut patch, &e, &v, &at, &names, Put::default(), &none).unwrap();
        assert_eq!(renamed["bar"], "l3", "named as asked");
        assert_eq!((renamed["a"].as_str(), renamed["b"].as_str(), renamed["m"].as_str()), ("p5", "p6", "p7"));
        let p = Value::Object(patch.done().unwrap());
        let near = |n: &str, q: [f64; 2]| libm::hypot(p["points"][n][0].as_f64().unwrap() - q[0], p["points"][n][1].as_f64().unwrap() - q[1]) < 1e-9;
        assert!(near("p6", [24.0, 42.0]), "40 long, as its dimension says, turned along p2 → p4: {p}");
        assert!(near("p7", [12.0, 26.0]), "halfway: {p}");
        let c = p["add_constraints"].as_array().unwrap();
        assert!(c.contains(&json!({"type": "coincident", "points": ["p5", "p2"]})), "{p}");
        assert!(c.contains(&json!({"type": "length", "line": "l3", "value": 40.0})), "{p}");
        let mut again = SketchPatch::new(&json!({"sketch": {"points": {"p1": [0, 0]}, "lines": {"l3": {"from": "p1", "to": "p1"}}}}));
        assert!(place(&mut again, &e, &v, &at, &names, Put::default(), &none).unwrap_err().contains("`l3` is taken"));
    }

    /// A preset placed makes a group of its name, of the points and curves placed; the name
    /// taken, the next free one; no preset, no group.
    #[test]
    fn a_preset_placed_is_a_group() {
        let e = element();
        let v = answers(&e, Some("short"), &Map::new(), &[], &none).unwrap();
        let at = [Anchor { at: [0.0, 0.0], point: None }];
        let mut patch = SketchPatch::new(&json!({"sketch": {"points": {"Плита": [0, 0]}}}));
        let renamed = place(&mut patch, &e, &v, &at, &BTreeMap::new(), Put { group: Some("Плита"), ..Put::default() }, &none).unwrap();
        place(&mut patch, &e, &v, &at, &BTreeMap::new(), Put { group: Some("Плита"), ..Put::default() }, &none).unwrap();
        let p = Value::Object(patch.done().unwrap());
        assert_eq!(p["groups"]["Плита (2)"].as_array().unwrap().len(), 4, "{p}");
        assert_eq!(p["groups"]["Плита (3)"].as_array().unwrap().len(), 4, "{p}");
        assert!(p["groups"].get("Плита").is_none(), "taken by a point: {p}");
        assert!(p["groups"]["Плита (2)"].as_array().unwrap().contains(&json!(renamed["bar"])), "{p}");
        let mut plain = SketchPatch::new(&json!({"sketch": {}}));
        place(&mut plain, &e, &v, &at, &BTreeMap::new(), Put::default(), &none).unwrap();
        assert!(plain.done().unwrap().get("groups").is_none());
    }

    /// A dimension given as a number made a parameter and taken back.
    #[test]
    fn a_number_made_a_parameter_and_back() {
        let mut e = json!({"name": "Bar", "anchors": ["a"],
            "sketch": {"points": {"a": [0, 0], "b": [8.9, 0], "c": [5, 5]}, "lines": {"l5": {"from": "a", "to": "b"}},
                       "circles": {"hole": {"center": "c", "diameter": 4}},
                       "constraints": [{"type": "length", "line": "l5", "value": 8.9}]}});
        let labels: Vec<String> = dimensions(&e).into_iter().map(|d| d.label).collect();
        assert_eq!(labels, ["length l5", "diameter hole"]);
        parametrize(&mut e, "length l5", "len").unwrap();
        parametrize(&mut e, "diameter hole", "d").unwrap();
        assert_eq!(e["parameters"], json!([{"name": "len", "value": 8.9}, {"name": "d", "value": 4.0}]));
        assert_eq!(e["sketch"]["constraints"][0]["value"], "len");
        assert!(check("sketch", &e).is_ok(), "{e}");
        assert!(parametrize(&mut e, "length l5", "x").unwrap_err().contains("not a number"));
        assert!(parametrize(&mut e, "diameter hole", "len").unwrap_err().contains("already"));
        rename_parameter(&mut e, "len", "length").unwrap();
        assert_eq!((&e["sketch"]["constraints"][0]["value"], &e["parameters"][0]["name"]), (&json!("length"), &json!("length")));
        unparametrize(&mut e, "length").unwrap();
        assert_eq!(e["sketch"]["constraints"][0]["value"], json!(8.9));
        unparametrize(&mut e, "d").unwrap();
        assert!(e.get("parameters").is_none(), "{e}");
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
