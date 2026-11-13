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

use datum_graph::Graph;
use datum_math::Vec2;
use datum_math::expr::{ExprError, Num, eval, substitute, valid_name};
use datum_ops::Op;
use datum_sketch::{Constraint, Sketch};
use serde::Serialize;
use serde_json::{Map, Value, json};

use crate::patch::SketchPatch;
use crate::workspace;

/// The levels with elements so far: a library folder's subfolders by the same names.
/// A material (D178) and a rule (D179) are the enterprise's own: none is built in. A part (D256)
/// is a body built by features, for the punch press's stamps so far.
pub const LEVELS: [&str; 5] = ["sketch", "material", "rule", "part", "tool"];

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
    /// The subfolder of `library/<level>` it is in, its names joined by `/` (D259); none at the
    /// level's top and for a built-in one.
    pub group: Option<String>,
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

    /// The element `name` of `group` (D259): `None` is the level's top, no built-in in a group.
    pub fn get_in(&self, group: Option<&str>, name: &str) -> Option<&Entry> {
        self.entries.iter().find(|e| e.name() == name && e.group.as_deref() == group)
    }

    /// The one element `name` (D259): none, or the only one; of a name in two groups the answer
    /// lists them, for the caller to name one.
    pub fn unique(&self, name: &str) -> Result<Option<&Entry>, String> {
        let hits: Vec<&Entry> = self.entries.iter().filter(|e| e.name() == name).collect();
        match hits.as_slice() {
            [] => Ok(None),
            [one] => Ok(Some(one)),
            many => {
                let mut groups: Vec<&str> = many.iter().map(|e| e.group.as_deref().unwrap_or("the top level")).collect();
                groups.sort_unstable();
                groups.dedup();
                Err(match groups.as_slice() {
                    [one] => format!("`{name}` is in two files of {one}: name the group or the file"),
                    _ => format!("`{name}` is in groups {}: name the group", groups.join(", ")),
                })
            }
        }
    }

    /// The one element `name`, which has to be there (D259).
    pub fn one(&self, name: &str) -> Result<&Entry, String> {
        self.unique(name)?.ok_or_else(|| format!("no element `{name}` in the library"))
    }
}

/// The elements of `level`: built in, then beside the program, then in `project` (its root),
/// each folder by file name, the folders below it as groups (D259). One of a name already listed
/// from a less specific place takes its place, its group does not matter (an engineer's
/// `Rectangle` in a group replaces the built-in one); two of a name in one source are both
/// listed, their groups apart.
pub fn load(level: &str, project: Option<&Path>) -> Library {
    let mut lib = Library::default();
    // One of a name from a more specific place takes the place of the other, whatever group each
    // is in; two of a name in one source are both kept.
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
                put(&mut lib, Entry { source: Source::BuiltIn, element: e, group: None, file: None });
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
        let mut files = Vec::new();
        files_in(&dir, None, &mut files);
        for (group, f) in files {
            let element = std::fs::read_to_string(&f)
                .map_err(|e| e.to_string())
                .and_then(|t| serde_json::from_str::<Value>(&t).map_err(|e| e.to_string()))
                .and_then(|e| check(level, &e).map(|()| e));
            match element {
                Ok(e) => put(&mut lib, Entry { source, element: e, group, file: Some(f) }),
                Err(why) => lib.skipped.push(format!("{}: {why}", f.display())),
            }
        }
    }
    lib
}

/// The element files of `dir` and of its folders, each with its group: the path below `dir` of
/// the folder it is in, `None` at the top. Sorted by path, so the file system's order never
/// shows; a folder whose name starts with `.` is not read, and a folder that is a symlink is
/// not entered: a link to an ancestor would loop (D259).
fn files_in(dir: &Path, group: Option<&str>, out: &mut Vec<(Option<String>, PathBuf)>) {
    let Ok(read) = std::fs::read_dir(dir) else { return };
    let mut paths: Vec<(PathBuf, bool)> = read.flatten().filter_map(|e| e.file_type().ok().map(|t| (e.path(), t.is_dir()))).collect();
    paths.sort();
    for (path, is_dir) in paths {
        let name = path.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
        if is_dir {
            if name.starts_with('.') {
                continue;
            }
            let group = match group {
                Some(g) => format!("{g}/{name}"),
                None => name,
            };
            files_in(&path, Some(&group), out);
        } else if path.extension().is_some_and(|x| x == "json") {
            out.push((group.map(str::to_string), path));
        }
    }
}

/// Whether `e` is an element of `level`: why not, if it is not.
pub fn check(level: &str, e: &Value) -> Result<(), String> {
    if !LEVELS.contains(&level) {
        return Err(format!("no elements of level `{level}` yet (levels: {LEVELS:?})"));
    }
    if !e["name"].as_str().is_some_and(|n| !n.trim().is_empty()) {
        return Err("an element needs a `name`".into());
    }
    match level {
        "material" => return check_material(e),
        "rule" => return check_rule(e),
        "part" => return check_part(e),
        "tool" => return check_tool(e),
        _ => {}
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
    check_presets(e, &names)?;
    Ok(())
}

/// The presets of an element name only its parameters.
fn check_presets(e: &Value, names: &[String]) -> Result<(), String> {
    for (preset, values) in e.get("presets").and_then(Value::as_object).into_iter().flatten() {
        let values = values.as_object().ok_or_else(|| format!("preset `{preset}`: parameter values by name"))?;
        if let Some(n) = values.keys().find(|n| !names.contains(n)) {
            return Err(format!("preset `{preset}`: no parameter `{n}`"));
        }
    }
    Ok(())
}

/// A material (D178): its `name`, a `designation` as the engineer writes it in a title block (a
/// cell's text form: `Лист [Б-ПН-{t} ГОСТ 19903-2015 | 08пс ГОСТ 16523-97]`, `{t}` where a
/// part's thickness goes), its `density` in kg/m³ and a `description`; nothing else.
fn check_material(e: &Value) -> Result<(), String> {
    let o = fields("material", e, &["name", "designation", "density", "description"], &["designation", "description"])?;
    if o.get("density").is_some_and(|d| !d.as_f64().is_some_and(|d| d > 0.0)) {
        return Err("`density`: kg/m³, above zero".into());
    }
    Ok(())
}

/// The fields only a sheet rule has: its `thickness` in mm (the part's: its only place, D193),
/// inner bend `radius`, the `k_factor` of its bends (0 to 1, D197), the `corner_gap` between
/// flanges meeting at a corner, the `relief_diameter` of a round corner relief, and the
/// `bend_relief_width` and `bend_relief_depth` of a short flange's bend relief (D229).
pub const SHEET_FIELDS: [&str; 8] =
    ["thickness", "radius", "k_factor", "corner_gap", "relief_diameter", "bend_relief_width", "bend_relief_depth", "kerf"];

/// A rule (D179, D180): its `name`, `kind` (`sheet` or `part`), the `material` it is of (by
/// name; a rule is never without one) and a `description`; a sheet rule's [`SHEET_FIELDS`]: the
/// thickness, the radius (mm, or a formula of the thickness `t`: `t`, `1.5 * t`) and optionally
/// the K-factor, the corner gap (mm not below zero, or of `t`) and the relief's Ø (mm above
/// zero, or of `t`).
fn check_rule(e: &Value) -> Result<(), String> {
    let keys: Vec<&str> = ["name", "kind", "material", "description"].into_iter().chain(SHEET_FIELDS).collect();
    let o = fields("rule", e, &keys, &["kind", "material", "description"])?;
    if !o.get("material").and_then(Value::as_str).is_some_and(|m| !m.trim().is_empty()) {
        return Err("a rule needs the `material` it is of".into());
    }
    match o.get("kind").and_then(Value::as_str) {
        Some("sheet") => {}
        Some("part") => {
            if let Some(k) = SHEET_FIELDS.into_iter().find(|k| o.contains_key(*k)) {
                return Err(format!("a part's rule has no `{k}`: that of a `sheet` rule"));
            }
            return Ok(());
        }
        _ => return Err("`kind`: `sheet` or `part`".into()),
    }
    let Some(t) = o.get("thickness").and_then(Value::as_f64).filter(|t| *t > 0.0) else {
        return Err("a sheet rule's `thickness`: mm, above zero".into());
    };
    if o.get("k_factor").is_some_and(|k| !k.as_f64().is_some_and(|k| (0.0..=1.0).contains(&k))) {
        return Err("a sheet rule's `k_factor`: a number from 0 to 1".into());
    }
    if o.get("corner_gap").is_some_and(|g| !of_thickness(g, t).is_ok_and(|g| g >= 0.0)) {
        return Err("a sheet rule's `corner_gap`: mm not below zero, or a formula of the thickness `t`".into());
    }
    for key in ["relief_diameter", "bend_relief_width", "bend_relief_depth", "kerf"] {
        if o.get(key).is_some_and(|d| !of_thickness(d, t).is_ok_and(|d| d > 0.0)) {
            return Err(format!("a sheet rule's `{key}`: mm above zero, or a formula of the thickness `t`"));
        }
    }
    match o.get("radius").map(|r| of_thickness(r, t)) {
        Some(Ok(r)) if r > 0.0 => Ok(()),
        _ => Err("a sheet rule's bend `radius`: mm above zero, or a formula of the thickness `t`".into()),
    }
}

/// A tool of the punch press (D250): its `name`, its `kind` (`roll`, a ball or a roller rolled
/// along a path), its `diameter`, the `depth` it presses and the die's `shoulder` radius, all in
/// mm above zero, the shortest path it rolls (`min_length`, optional) and a `description`.
fn check_tool(e: &Value) -> Result<(), String> {
    let o = fields("tool", e, &["name", "kind", "diameter", "depth", "shoulder", "min_length", "description"], &["kind", "description"])?;
    if o.get("kind").and_then(Value::as_str) != Some("roll") {
        return Err("a tool's `kind`: `roll`".into());
    }
    for key in ["diameter", "depth", "shoulder"] {
        if !o.get(key).and_then(Value::as_f64).is_some_and(|v| v > 0.0) {
            return Err(format!("a tool's `{key}`: mm, above zero"));
        }
    }
    if o.get("min_length").is_some_and(|v| !v.as_f64().is_some_and(|v| v >= 0.0)) {
        return Err("a tool's `min_length`: mm, not below zero".into());
    }
    Ok(())
}

/// A part of the library (D256): its `kind` (`stamp` so far), the `features` that build its body
/// (the model's operations, by name), the `roles` of the body (`cut`, `form`, optional `hinge`),
/// the `anchor` where it is put in its local frame, an optional `thickness` range and the
/// parameters of its size series. The body is built and checked when the part is used, not here:
/// this keeps the library readable and lets a part be made before its stamp operation exists.
fn check_part(e: &Value) -> Result<(), String> {
    let keys = ["name", "kind", "description", "parameters", "presets", "features", "roles", "anchor", "axis", "fit", "thickness"];
    let o = fields("part", e, &keys, &["kind", "description"])?;
    if o.get("kind").and_then(Value::as_str) != Some("stamp") {
        return Err("a part's `kind`: `stamp`".into());
    }
    let anchor = o.get("anchor").and_then(Value::as_array).ok_or("a part's `anchor`: a point [x, y, z]")?;
    if anchor.len() != 3 || anchor.iter().any(|v| v.as_f64().is_none()) {
        return Err("a part's `anchor`: a point [x, y, z]".into());
    }
    // Placed by a line (a louver along it), the tool's axis turns to the line and its length
    // feeds the parameter `fit` names (D260), as a sketch element stretches by two anchors.
    if let Some(a) = o.get("axis") {
        let a = a.as_array().filter(|a| a.len() == 3 && a.iter().all(Value::is_number)).ok_or("a part's `axis`: a direction [x, y, z]")?;
        if a.iter().all(|v| v.as_f64().is_some_and(|x| x == 0.0)) {
            return Err("a part's `axis`: not the zero vector".into());
        }
    }
    let names = parameter_names(e)?;
    if let Some(f) = o.get("fit") {
        let f = f.as_str().filter(|f| !f.trim().is_empty()).ok_or("a part's `fit`: a parameter's name")?;
        if !names.iter().any(|n| n == f) {
            return Err(format!("a part's `fit`: no parameter `{f}`"));
        }
        if o.get("axis").is_none() {
            return Err("a part's `fit` needs an `axis`: the line's length feeds it".into());
        }
    }
    if let Some(t) = o.get("thickness") {
        let t = t.as_array().filter(|t| t.len() == 2 && t.iter().all(Value::is_number)).ok_or("a part's `thickness`: [min, max]")?;
        let (min, max) = (t[0].as_f64().unwrap_or_default(), t[1].as_f64().unwrap_or_default());
        if !(min > 0.0 && max >= min) {
            return Err("a part's `thickness`: mm, 0 < min <= max".into());
        }
    }
    check_presets(e, &names)?;
    let mut graph = Graph::default();
    for p in e.get("parameters").and_then(Value::as_array).into_iter().flatten() {
        let name = p["name"].as_str().unwrap_or_default();
        let value: Num = serde_json::from_value(p["value"].clone()).map_err(|err| format!("parameter `{name}`: {err}"))?;
        graph.set_parameter(name, value).map_err(|err| err.to_string())?;
    }
    let features = o.get("features").and_then(Value::as_array).filter(|f| !f.is_empty()).ok_or("a part's `features`: a non-empty list")?;
    for f in features {
        let f = fields("feature", f, &["name", "op", "inputs"], &[])?;
        let name = f.get("name").and_then(Value::as_str).filter(|n| !n.trim().is_empty()).ok_or("a feature's `name`: text")?;
        let op: Op = serde_json::from_value(f["op"].clone()).map_err(|err| format!("feature `{name}`: {err}"))?;
        let inputs: Vec<&str> = match f.get("inputs") {
            None => Vec::new(),
            Some(v) => v
                .as_array()
                .and_then(|a| a.iter().map(Value::as_str).collect::<Option<Vec<_>>>())
                .ok_or_else(|| format!("feature `{name}`: `inputs` are feature names"))?,
        };
        let mut at = Vec::with_capacity(inputs.len());
        for input in inputs {
            let ix = graph.find(input).ok_or_else(|| format!("feature `{name}`: input `{input}` is not an earlier feature"))?;
            at.push(ix);
        }
        graph.add(name, op, &at).map_err(|err| err.to_string())?;
    }
    let roles = o.get("roles").and_then(Value::as_object).ok_or("a part's `roles`: `cut`, `form` or `hinge`")?;
    if let Some(k) = roles.keys().find(|k| !["cut", "form", "hinge"].contains(&k.as_str())) {
        return Err(format!("a part's role `{k}`: `cut`, `form` or `hinge`"));
    }
    for (role, name) in roles {
        let name = name.as_str().ok_or_else(|| format!("a part's role `{role}`: a feature name"))?;
        if graph.find(name).is_none() {
            return Err(format!("a part's role `{role}`: no feature `{name}`"));
        }
    }
    if !roles.contains_key("cut") && !roles.contains_key("form") {
        return Err("a part's `roles`: `cut` or `form`".into());
    }
    Ok(())
}

/// A length of a sheet rule (mm, or a formula of `t`: the bend radius, the corner gap, the
/// relief's Ø) for thickness `t`.
pub fn of_thickness(length: &Value, t: f64) -> Result<f64, String> {
    match length {
        Value::String(f) => eval(f, &|n| if n == "t" { Ok(t) } else { Err(ExprError::Unknown(n.into())) }).map_err(|e| e.to_string()),
        r => r.as_f64().ok_or_else(|| "a number or a formula".into()),
    }
}

/// `e` as an object of `what` with no keys but `keys`, those of `text` text.
fn fields<'a>(what: &str, e: &'a Value, keys: &[&str], text: &[&str]) -> Result<&'a Map<String, Value>, String> {
    let o = e.as_object().ok_or_else(|| format!("a {what} is an object"))?;
    if let Some(k) = o.keys().find(|k| !keys.contains(&k.as_str())) {
        let keys: Vec<String> = keys.iter().map(|k| format!("`{k}`")).collect();
        return Err(format!("a {what} has no `{k}`: {}", keys.join(", ")));
    }
    if let Some(k) = text.iter().find(|k| o.get(**k).is_some_and(|v| !v.is_string())) {
        return Err(format!("`{k}` is text"));
    }
    Ok(o)
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

/// Element `e` mirrored left-right about the vertical through its first anchor, in its local
/// coordinates: the element itself is reflected before placing, not a copy made after it (D258).
/// Its points' x are reflected, an arc's ends swap (it runs counterclockwise from `from` to
/// `to`), an angle's two lines swap, and a fix's `at` is reflected. Lengths, radii, distances,
/// tangencies and symmetry about a vertical stay as they are. `links` stay untouched: they name
/// the side outside the element, and their labels must read the same wherever it is put.
/// Patterns are not supported: they would need their own reflection.
pub fn mirrored(e: &Value) -> Result<Value, String> {
    let mut out = e.clone();
    let ax = {
        let first = anchors(e).into_iter().next().ok_or("the element has no anchor point")?;
        let p = &e["sketch"]["points"][first.as_str()];
        let p = p.as_array().filter(|p| p.len() == 2 && p.iter().all(Value::is_number)).ok_or("a point is not [x, y]")?;
        p[0].as_f64().ok_or("a point is not [x, y]")?
    };
    let sketch = out["sketch"].as_object_mut().ok_or("the element has no `sketch`")?;
    if sketch.get("patterns").and_then(Value::as_object).is_some_and(|p| !p.is_empty()) {
        return Err("mirroring an element with patterns is not supported".into());
    }
    if let Some(points) = sketch.get_mut("points").and_then(Value::as_object_mut) {
        for p in points.values_mut() {
            *p = mirror_point(p, ax)?;
        }
    }
    for a in sketch.get_mut("arcs").and_then(Value::as_object_mut).into_iter().flat_map(|m| m.values_mut()) {
        let Some(o) = a.as_object_mut() else { continue };
        let (from, to) = (o.get("from").cloned(), o.get("to").cloned());
        if let (Some(from), Some(to)) = (from, to) {
            o.insert("from".into(), to);
            o.insert("to".into(), from);
        }
    }
    let swap_angle = |c: &mut Value| {
        if c["type"] == "angle"
            && let Some(lines) = c.get_mut("lines").and_then(Value::as_array_mut)
            && lines.len() == 2
        {
            lines.swap(0, 1);
        }
    };
    for c in sketch.get_mut("constraints").and_then(Value::as_array_mut).into_iter().flatten() {
        if c["type"] == "fix" {
            if let Some(at) = c.get("at").filter(|v| !v.is_null()).cloned() {
                c["at"] = mirror_point(&at, ax)?;
            }
        } else {
            swap_angle(c);
        }
    }
    if let Some(measures) = sketch.get_mut("measures").and_then(Value::as_object_mut) {
        for m in measures.values_mut() {
            swap_angle(m);
        }
    }
    Ok(out)
}

/// Point `p` (`[x, y]`) reflected about the vertical line `x = ax`.
fn mirror_point(p: &Value, ax: f64) -> Result<Value, String> {
    let q = p.as_array().filter(|q| q.len() == 2 && q.iter().all(Value::is_number)).ok_or("a point is not [x, y]")?;
    let (x, y) = (q[0].as_f64().ok_or("a point is not [x, y]")?, q[1].as_f64().ok_or("a point is not [x, y]")?);
    Ok(json!([2.0 * ax - x, y]))
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

/// The sockets of element `e` before it is placed, in order: how each reads by the element's
/// names and what it takes (one of [`KINDS`]).
pub fn sockets(e: &Value) -> Vec<(String, String)> {
    e["links"].as_array().into_iter().flatten().map(|l| (label_of(&l["constraint"]), l["kind"].as_str().unwrap_or("curve").to_string())).collect()
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
/// among the questions and the presets.
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
    for values in e["presets"].as_object_mut().into_iter().flat_map(|m| m.values_mut()).filter_map(Value::as_object_mut) {
        if let Some(v) = values.remove(from) {
            values.insert(to.to_string(), v);
        }
    }
    for (_, v) in dimension_slots(e) {
        if let Some(t) = v.as_str() {
            *v = json!(substitute(t, &|n: &str| (n == from).then(|| to.to_string())).map_err(|err| err.to_string())?.replace(&format!("({to})"), to));
        }
    }
    Ok(())
}

/// Parameter `name` of element `e` taken back: the dimensions it alone gives get its value as a
/// number, and the presets drop it. An error if another dimension uses it within an expression.
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
    for values in e["presets"].as_object_mut().into_iter().flat_map(|m| m.values_mut()).filter_map(Value::as_object_mut) {
        values.remove(name);
    }
    let none_left = e["parameters"].as_array_mut().is_some_and(|list| {
        list.retain(|p| p["name"] != name);
        list.is_empty()
    });
    if none_left && let Some(o) = e.as_object_mut() {
        o.remove("parameters");
        o.remove("presets");
    }
    Ok(())
}

/// Where new elements of `level` are saved unless put elsewhere: `<level>` in the library
/// folder of the workspace.
pub fn folder(level: &str, model: Option<&Path>) -> Result<PathBuf, String> {
    Ok(workspace::folder("library", model)?.join(level))
}

/// Writes `element` (checked) of `level`, in place of the one named `was` (its own name if
/// none): into the file of the library's element of that name, renamed for a new name, or else
/// into `group` of the library of `project`, or beside the program without one (a built-in one
/// is so overridden). `group` is the subfolder of `library/<level>` it goes to, its names joined
/// by `/`; an element found by `was` in another folder is moved there. A name taken by another
/// element of the same group is refused. Where it went.
pub fn save(level: &str, element: &Value, was: Option<&str>, group: Option<&str>, project: Option<&Path>, lib: &Library) -> Result<PathBuf, String> {
    check(level, element)?;
    let name = element["name"].as_str().unwrap_or_default().trim();
    let given = was.is_some();
    let was = was.unwrap_or(name);
    // `was` in that group, else, when it is named, the one of its name — but only when the name
    // is not in two groups: then the group says which (D259). An element moved to another group
    // is found where it was, and its file goes once written. Without `was`, an element of the
    // name in another group is another one (D259): this is a new element, not a move.
    let old = match lib.get_in(group, was) {
        Some(e) => e.file.clone(),
        None if given => lib.unique(was)?.and_then(|e| e.file.clone()),
        None => None,
    };
    if name != was && lib.get_in(group, name).is_some() {
        return Err(format!("`{name}` is in this group already"));
    }
    let dir = match group {
        Some(g) => level_folder(level, project)?.join(group_path(g)?),
        None => match &old {
            Some(f) => f.parent().map(Path::to_path_buf).ok_or("no folder")?,
            None => level_folder(level, project)?,
        },
    };
    let path = dir.join(file_name(name));
    if old.as_ref() != Some(&path) && path.exists() {
        return Err(format!("{} exists and is not `{was}`", path.display()));
    }
    write(&path, element)?;
    drop_moved(old.as_deref(), &path)?;
    Ok(path)
}

/// The folder of `level` in the workspace of `project`, or beside the program without one.
fn level_folder(level: &str, project: Option<&Path>) -> Result<PathBuf, String> {
    match project {
        Some(root) => Ok(workspace::folder_in(root, "library")?.join(level)),
        None => folder(level, None),
    }
}

/// The folder of `group` under a level's folder (D259): its names, `/` between them. A name is
/// what a file system takes, as in [`file_name`].
pub fn group_path(group: &str) -> Result<PathBuf, String> {
    let mut path = PathBuf::new();
    for name in group.split('/') {
        if name.is_empty() || name == "." || name == ".." || name.chars().any(forbidden) {
            return Err(format!("`{group}` is not a group: its folders are names, `/` between them"));
        }
        path.push(name);
    }
    Ok(path)
}

/// [`save`] kept whole with the other levels (D179): a rule's material must be in the library
/// (`project`'s, as [`load`] reads it); a material renamed takes its rules along.
pub fn put(level: &str, element: &Value, was: Option<&str>, group: Option<&str>, project: Option<&Path>) -> Result<PathBuf, String> {
    if level == "rule" {
        let m = element["material"].as_str().unwrap_or_default();
        if !m.is_empty() && load("material", project).get(m).is_none() {
            return Err(format!("no material `{m}` in the library: a rule is of one"));
        }
    }
    let path = save(level, element, was, group, project, &load(level, project))?;
    let name = element["name"].as_str().unwrap_or_default().trim();
    if level == "material"
        && let Some(was) = was.filter(|w| *w != name)
    {
        for r in rules_of(was, project) {
            let mut e = r.element;
            e["material"] = json!(name);
            if let Some(f) = r.file {
                write(&f, &e)?;
            }
        }
    }
    Ok(path)
}

/// [`remove`] kept whole with the other levels: a material with rules stays.
pub fn delete(level: &str, name: &str, group: Option<&str>, project: Option<&Path>) -> Result<PathBuf, String> {
    if level == "material" {
        let rules: Vec<String> = rules_of(name, project).iter().map(|r| format!("`{}`", r.name())).collect();
        if !rules.is_empty() {
            return Err(format!("`{name}` has rules ({}): remove them or give them another material first", rules.join(", ")));
        }
    }
    remove(name, group, &load(level, project))
}

/// The rules of material `name`.
fn rules_of(name: &str, project: Option<&Path>) -> Vec<Entry> {
    load("rule", project).entries.into_iter().filter(|e| e.element["material"] == name).collect()
}

/// Removes element `name`'s file, the most specific one; a built-in element stays. Of a name in
/// two groups, `group` says which; without it the groups are listed. Two files of the name in
/// one folder: the first goes, as it did before the groups (D259).
pub fn remove(name: &str, group: Option<&str>, lib: &Library) -> Result<PathBuf, String> {
    let hits: Vec<&Entry> = lib.entries.iter().filter(|e| e.name() == name && group.is_none_or(|g| e.group.as_deref() == Some(g))).collect();
    if hits.len() > 1 && hits.iter().any(|e| e.group != hits[0].group) {
        let mut groups: Vec<&str> = hits.iter().map(|e| e.group.as_deref().unwrap_or("the top level")).collect();
        groups.sort_unstable();
        groups.dedup();
        return Err(format!("`{name}` is in groups {}: name the group", groups.join(", ")));
    }
    let e = hits.first().copied().ok_or_else(|| match group {
        Some(g) => format!("no element `{name}` in group `{g}` of the library"),
        None => format!("no element `{name}` in the library"),
    })?;
    let f = e.file.clone().ok_or_else(|| format!("`{name}` is built in"))?;
    std::fs::remove_file(&f).map_err(|e| format!("{}: {e}", f.display()))?;
    Ok(f)
}

/// `element` as a file at `path`, its folder made.
pub fn write(path: &Path, element: &Value) -> Result<(), String> {
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    }
    let text = serde_json::to_string_pretty(element).map_err(|e| e.to_string())?;
    std::fs::write(path, text).map_err(|e| format!("{}: {e}", path.display()))
}

/// What a file system refuses in a name (D259): `file_name` replaces it, `group_path` refuses it.
fn forbidden(c: char) -> bool {
    c.is_control() || r#"<>:"/\|?*"#.contains(c)
}

/// A file name for element `name`: what a file system would refuse, replaced.
pub fn file_name(name: &str) -> String {
    let stem: String = name.chars().map(|c| if forbidden(c) { '_' } else { c }).collect();
    format!("{}.json", stem.trim())
}

/// The file an element was moved from (D259): another file than the one just written, gone. The
/// same place reached the other way is the same file, not a move.
pub fn drop_moved(old: Option<&Path>, new: &Path) -> Result<(), String> {
    let Some(old) = old else { return Ok(()) };
    let same = match (std::fs::canonicalize(old), std::fs::canonicalize(new)) {
        (Ok(a), Ok(b)) => a == b,
        _ => old == new,
    };
    if same {
        return Ok(());
    }
    std::fs::remove_file(old).map_err(|e| format!("{}: {e}", old.display()))
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
        let mut bad = element();
        bad["presets"]["short"]["width"] = json!(3);
        assert!(check("sketch", &bad).unwrap_err().contains("no parameter `width`"));
    }

    #[test]
    fn a_tool_has_its_sizes() {
        let tool = |e: Value| check("tool", &e);
        assert!(tool(json!({"name": "R6", "kind": "roll", "diameter": 12, "depth": 3, "shoulder": 2})).is_ok());
        assert!(tool(json!({"name": "R6", "kind": "roll", "diameter": 12, "depth": 3, "shoulder": 2, "min_length": 20, "description": "x"})).is_ok());
        assert!(tool(json!({"name": "R6", "kind": "punch", "diameter": 12, "depth": 3, "shoulder": 2})).unwrap_err().contains("`roll`"));
        assert!(tool(json!({"name": "R6", "kind": "roll", "depth": 3, "shoulder": 2})).unwrap_err().contains("`diameter`"));
        assert!(tool(json!({"name": "R6", "kind": "roll", "diameter": 12, "depth": 0, "shoulder": 2})).unwrap_err().contains("`depth`"));
        assert!(
            tool(json!({"name": "R6", "kind": "roll", "diameter": 12, "depth": 3, "shoulder": 2, "min_length": -1}))
                .unwrap_err()
                .contains("`min_length`")
        );
        assert!(tool(json!({"name": "R6", "kind": "roll", "diameter": 12, "depth": 3, "shoulder": 2, "x": 1})).unwrap_err().contains("no `x`"));
    }

    /// A stamp part: its body is a feature graph, its roles name the features the stamp uses.
    #[test]
    fn a_stamp_has_its_body_and_roles() {
        let stamp = json!({
            "name": "Язык 20",
            "kind": "stamp",
            "description": "x",
            "anchor": [0.0, 0.0, 0.0],
            "thickness": [0.5, 3.0],
            "parameters": [{"name": "w", "value": 20.0}],
            "presets": {"20": {"w": 20.0}},
            "features": [
                {"name": "body", "op": {"op": "prism", "polygon": [[0, 0], [1, 0], [1, 1], [0, 1]], "z0": 0, "z1": 1}, "inputs": []}
            ],
            "roles": {"cut": "body", "form": "body"}
        });
        assert!(check("part", &stamp).is_ok());
        let mut bad = stamp.clone();
        bad["kind"] = json!("part");
        assert!(check("part", &bad).unwrap_err().contains("`stamp`"));
        let mut bad = stamp.clone();
        bad["roles"]["cut"] = json!("missing");
        assert!(check("part", &bad).unwrap_err().contains("no feature `missing`"));
        let mut bad = stamp.clone();
        bad["thickness"] = json!([3.0, 1.0]);
        assert!(check("part", &bad).unwrap_err().contains("0 < min <= max"));
        let mut bad = stamp.clone();
        bad["features"][0]["inputs"] = json!(["later"]);
        assert!(check("part", &bad).unwrap_err().contains("not an earlier feature"));
        let mut bad = stamp.clone();
        bad["features"][0]["inputs"] = json!([1]);
        assert!(check("part", &bad).unwrap_err().contains("`inputs` are feature names"));
        let mut bad = stamp.clone();
        bad["features"][0]["op"] = json!({"op": "prism", "polygon": [[0, 0], [1, 0], [1, 1]], "z0": 0});
        assert!(check("part", &bad).unwrap_err().contains("feature `body`"));
        // Placed by a line: the tool's axis and the parameter its length feeds (D260).
        let mut louver = stamp.clone();
        louver["name"] = json!("Жалюзи 30");
        louver["axis"] = json!([1.0, 0.0, 0.0]);
        louver["fit"] = json!("w");
        assert!(check("part", &louver).is_ok());
        let mut bad = louver.clone();
        bad["axis"] = json!([0, 0, 0]);
        assert!(check("part", &bad).unwrap_err().contains("zero vector"));
        let mut bad = louver.clone();
        bad["fit"] = json!("none");
        assert!(check("part", &bad).unwrap_err().contains("no parameter `none`"));
        let mut bad = louver.clone();
        bad.as_object_mut().unwrap().remove("axis");
        assert!(check("part", &bad).unwrap_err().contains("needs an `axis`"));
    }

    #[test]
    fn the_starter_library_is_valid() {
        let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../library/");
        let mut materials: Vec<String> = Vec::new();
        for level in ["material", "rule", "tool"] {
            let dir = format!("{root}{level}");
            let mut paths = Vec::new();
            files_in(Path::new(&dir), None, &mut paths);
            for (_, path) in paths {
                let file = path.display().to_string();
                let e: Value = serde_json::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
                let valid = check(level, &e);
                assert!(valid.is_ok(), "{file}: {valid:?}");
                if level == "material" {
                    materials.push(e["name"].as_str().unwrap().to_string());
                } else if level == "rule" {
                    let of = e["material"].as_str().unwrap_or_default();
                    assert!(materials.iter().any(|m| m == of), "{file}: no material `{of}` in the library");
                }
            }
        }
    }

    #[test]
    fn presets_follow_their_parameters() {
        let mut e = element();
        e["presets"]["short"]["half"] = json!(3);
        rename_parameter(&mut e, "len", "length").unwrap();
        assert_eq!(e["presets"]["short"], json!({"length": 10, "half": 3}));
        assert!(check("sketch", &e).is_ok());
        let mut e = json!({"name": "Bar", "anchors": ["a"], "parameters": [{"name": "d1", "value": 4}, {"name": "d2", "value": 5}],
                           "presets": {"A": {"d1": 1, "d2": 2}},
                           "sketch": {"points": {"a": [0, 0], "b": [1, 0]}, "lines": {"l": {"from": "a", "to": "b"}},
                                      "constraints": [{"type": "length", "line": "l", "value": "d1"}, {"type": "distance", "points": ["a", "b"], "value": "d2"}]}});
        unparametrize(&mut e, "d1").unwrap();
        assert_eq!(e["presets"]["A"], json!({"d2": 2}));
        assert!(check("sketch", &e).is_ok());
        unparametrize(&mut e, "d2").unwrap();
        assert!(e.get("presets").is_none() && e.get("parameters").is_none());
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

    /// A piece mirrored about the vertical through its first anchor: its points, an arc's ends,
    /// an angle's lines and a fix's spot; placed once, not twice (D258).
    #[test]
    fn a_mirrored_element_is_placed_once() {
        let e = json!({"name": "Hook", "anchors": ["a"],
            "sketch": {"points": {"a": [0, 0], "b": [4, 0], "c": [4, 2], "m": [2, 1]},
                       "lines": {"bottom": {"from": "a", "to": "b"}, "edge": {"from": "b", "to": "c"}},
                       "arcs": {"bow": {"center": "m", "from": "a", "to": "b"}},
                       "constraints": [{"type": "angle", "lines": ["bottom", "edge"], "degrees": 90},
                                       {"type": "fix", "point": "c", "at": [4, 2]}],
                       "measures": {"bend": {"type": "angle", "lines": ["bottom", "edge"]}}}});
        let m = mirrored(&e).unwrap();
        assert_eq!(m["sketch"]["points"]["b"], json!([-4.0, 0.0]), "about the first anchor at x = 0: {m}");
        assert_eq!(m["sketch"]["points"]["c"], json!([-4.0, 2.0]), "{m}");
        assert_eq!((&m["sketch"]["arcs"]["bow"]["from"], &m["sketch"]["arcs"]["bow"]["to"]), (&json!("b"), &json!("a")), "{m}");
        assert_eq!(m["sketch"]["constraints"][0]["lines"], json!(["edge", "bottom"]), "{m}");
        assert_eq!(m["sketch"]["constraints"][1]["at"], json!([-4.0, 2.0]), "{m}");
        assert_eq!(m["sketch"]["measures"]["bend"]["lines"], json!(["edge", "bottom"]), "{m}");
        // Placed once: the points, lines and arc of the mirror, not the element and its copy.
        let v = answers(&m, None, &Map::new(), &[], &none).unwrap();
        let at = [Anchor { at: [10.0, 10.0], point: None }];
        let mut patch = SketchPatch::new(&json!({"sketch": {}}));
        let renamed = place(&mut patch, &m, &v, &at, &BTreeMap::new(), Put::default(), &none).unwrap();
        let p = Value::Object(patch.done().unwrap());
        assert_eq!(p["points"].as_object().unwrap().len(), 4, "{p}");
        assert_eq!(p["lines"].as_object().unwrap().len(), 2, "{p}");
        assert_eq!(p["arcs"].as_object().unwrap().len(), 1, "{p}");
        let q = |n: &str| {
            let point = &p["points"][renamed[n].as_str()];
            [point[0].as_f64().unwrap(), point[1].as_f64().unwrap()]
        };
        assert_eq!(q("a"), [10.0, 10.0], "the anchor where clicked: {p}");
        assert_eq!(q("b"), [6.0, 10.0], "left of the anchor, not right: {p}");
        assert_eq!(q("c"), [6.0, 12.0], "{p}");
        // A point that is not a pair of numbers, and patterns, are refused.
        let mut bad = e.clone();
        bad["sketch"]["points"]["b"] = json!([4]);
        assert!(mirrored(&bad).unwrap_err().contains("not [x, y]"));
        let mut patterned = e.clone();
        patterned["sketch"]["patterns"] = json!({"grid": {"of": "bottom", "count": 2, "step": [5, 0]}});
        assert!(mirrored(&patterned).unwrap_err().contains("patterns"));
    }

    /// An empty project of the test: its own folder, so the files do not mix.
    fn temp(name: &str) -> PathBuf {
        let root = std::env::temp_dir().join(format!("datum-library-{name}"));
        let _ = std::fs::remove_dir_all(&root);
        std::fs::create_dir_all(&root).unwrap();
        root
    }

    /// A piece of sketch called `name`, as a file of a library.
    fn piece(name: &str) -> Value {
        json!({"name": name, "anchors": ["a"], "sketch": {"points": {"a": [0, 0]}}})
    }

    /// The folders of `library/<level>` are groups (D259): a file in one carries the path of the
    /// folder it is in, two levels deep too; one name in two groups is two elements; a hidden
    /// folder is not read.
    #[test]
    fn the_folders_of_a_level_are_groups() {
        let root = temp("groups");
        let dir = root.join("library/sketch");
        let write = |rel: &str, name: &str| {
            let path = dir.join(rel);
            std::fs::create_dir_all(path.parent().unwrap()).unwrap();
            std::fs::write(path, piece(name).to_string()).unwrap();
        };
        write("top.json", "Top");
        write("Плиты/plate.json", "Плита");
        write("Плиты/Перегородки/deep.json", "Deep");
        write("Косынки/plate.json", "Плита");
        write(".hidden/hidden.json", "Hidden");
        let lib = load("sketch", Some(&root));
        let groups: Vec<Option<&str>> = lib.entries.iter().filter(|e| e.source == Source::Project).map(|e| e.group.as_deref()).collect();
        assert!(groups.contains(&None), "{groups:?}");
        assert!(groups.contains(&Some("Плиты")) && groups.contains(&Some("Плиты/Перегородки")), "{groups:?}");
        assert_eq!(lib.get_in(None, "Top").and_then(|e| e.group.clone()), None);
        assert_eq!(lib.get_in(Some("Плиты/Перегородки"), "Deep").and_then(|e| e.file.clone()), Some(dir.join("Плиты/Перегородки/deep.json")));
        assert_eq!(lib.entries.iter().filter(|e| e.name() == "Плита").count(), 2, "one name in two groups: both");
        assert!(lib.get("Hidden").is_none(), "a hidden folder is not read");
    }

    /// An element of a group replaces the one of its name from a less specific place, whatever
    /// group each is in (D259): the engineer's `Rectangle` in a group replaces the built-in one,
    /// and the list shows it once.
    #[test]
    fn a_grouped_element_replaces_a_less_specific_one() {
        let root = temp("groups_override");
        let dir = root.join("library/sketch/Тест");
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("rectangle.json"), piece("Rectangle").to_string()).unwrap();
        let lib = load("sketch", Some(&root));
        assert_eq!(lib.entries.iter().filter(|e| e.name() == "Rectangle").count(), 1, "one of the name");
        let e = lib.get("Rectangle").unwrap();
        assert_eq!(e.source, Source::Project, "the engineer's one shows, not the built-in");
        assert_eq!(e.group.as_deref(), Some("Тест"));
    }

    /// Saved into a group, an element makes its folders; named `was`, it moves from the folder it
    /// was in, its old file gone; a group that is a path is refused (D259).
    #[test]
    fn an_element_saved_into_a_group() {
        let root = temp("save_group");
        let e = piece("Плита");
        let path = save("sketch", &e, None, Some("Плиты/Перегородки"), Some(&root), &load("sketch", Some(&root))).unwrap();
        assert_eq!(path, root.join("library/sketch/Плиты/Перегородки/Плита.json"));
        assert!(path.exists(), "its folders made");
        let lib = load("sketch", Some(&root));
        assert_eq!(lib.get_in(Some("Плиты/Перегородки"), "Плита").and_then(|e| e.group.clone()).as_deref(), Some("Плиты/Перегородки"));
        // Named, it moves to another group: the file it was in is gone.
        let moved = save("sketch", &e, Some("Плита"), Some("Косынки"), Some(&root), &lib).unwrap();
        assert_eq!(moved, root.join("library/sketch/Косынки/Плита.json"));
        assert!(!path.exists(), "the old file gone");
        // Another group is another element: a save of the name alone leaves the one it was.
        let lib = load("sketch", Some(&root));
        let other = save("sketch", &e, None, Some("Плиты"), Some(&root), &lib).unwrap();
        assert_eq!(other, root.join("library/sketch/Плиты/Плита.json"));
        assert!(moved.exists() && other.exists(), "both groups hold it");
        // A name in two groups: a named move says which, it does not take one silently (D259).
        let why = save("sketch", &e, Some("Плита"), Some("Новая"), Some(&root), &load("sketch", Some(&root))).unwrap_err();
        assert!(why.contains("Косынки") && why.contains("Плиты") && why.contains("name the group"), "{why}");
        for bad in ["../x", "a//b", "a/./b", "a*b", ""] {
            assert!(save("sketch", &e, None, Some(bad), Some(&root), &load("sketch", Some(&root))).is_err(), "`{bad}` is not a group");
        }
    }

    /// The file an element was moved from is gone, unless it is the same file reached the other
    /// way (D259).
    #[test]
    fn a_move_to_the_same_file_keeps_it() {
        let root = temp("drop_moved");
        let dir = root.join("a/b");
        std::fs::create_dir_all(&dir).unwrap();
        let file = dir.join("one.json");
        std::fs::write(&file, "{}").unwrap();
        drop_moved(Some(&dir.join("..").join("b").join("one.json")), &file).unwrap();
        assert!(file.exists(), "the same file through `..` is not a move");
        let other = root.join("two.json");
        std::fs::write(&other, "{}").unwrap();
        drop_moved(Some(&other), &file).unwrap();
        assert!(!other.exists(), "another file is a move: the old one gone");
        drop_moved(None, &file).unwrap();
        assert!(file.exists(), "no old file: nothing to drop");
    }

    /// A name in two groups is removed by its group; without one, the groups are listed (D259).
    #[test]
    fn a_name_in_two_groups_is_removed_by_its_group() {
        let root = temp("remove_group");
        let e = piece("Плита");
        for g in ["Плиты", "Косынки"] {
            save("sketch", &e, None, Some(g), Some(&root), &load("sketch", Some(&root))).unwrap();
        }
        let lib = load("sketch", Some(&root));
        let why = remove("Плита", None, &lib).unwrap_err();
        assert!(why.contains("Плиты") && why.contains("Косынки") && why.contains("name the group"), "{why}");
        let gone = remove("Плита", Some("Плиты"), &lib).unwrap();
        assert_eq!(gone, root.join("library/sketch/Плиты/Плита.json"));
        assert!(!gone.exists());
        assert_eq!(load("sketch", Some(&root)).entries.iter().filter(|e| e.name() == "Плита").count(), 1, "the other stays");
        assert!(remove("Плита", Some("Плиты"), &lib).is_err(), "nothing left there");
    }
}
