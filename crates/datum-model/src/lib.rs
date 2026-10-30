//! Datum model files (ARCHITECTURE §6.2): the feature graph as canonical JSON.
//!
//! ```json
//! {
//!   "format": "datum/1",
//!   "features": [
//!     { "name": "plate", "op": "prism", "polygon": [[0.0, 0.0], [100.0, 0.0], [100.0, 50.0]], "z0": 0.0, "z1": 3.0 },
//!     { "name": "moved", "op": "transform", "inputs": ["plate"], "angle_deg": 30.0 }
//!   ]
//! }
//! ```
//!
//! Bodies are not stored: evaluation rebuilds them bit for bit. Features come after their
//! inputs, in history order. Keys are written sorted, `f64` values round-trip exactly.

use std::collections::BTreeMap;
use std::fmt;

use datum_math::expr::Num;

pub mod levels;
pub mod library;
mod patch;
mod session;
pub mod workspace;

pub use patch::SketchPatch;
pub use session::{Session, patch_sketch, visit_names};

use datum_graph::{Graph, GraphError, NodeIx, Op};
use serde::{Deserialize, Serialize};
use serde_json::{Map, Value};

pub const FORMAT: &str = "datum/1";

#[derive(Debug)]
pub enum ModelError {
    Json(serde_json::Error),
    Format(String),
    Feature {
        index: usize,
        msg: String,
    },
    Graph(GraphError),
    /// The levels and parts, as a file before D169 had them, could not be brought over.
    Levels(String),
}

impl fmt::Display for ModelError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Json(e) => write!(f, "JSON: {e}"),
            Self::Format(v) => write!(f, "format `{v}` is not supported (expected `{FORMAT}`)"),
            Self::Feature { index, msg } => write!(f, "feature #{index}: {msg}"),
            Self::Graph(e) => e.fmt(f),
            Self::Levels(m) => write!(f, "levels: {m}"),
        }
    }
}

impl std::error::Error for ModelError {}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct File {
    format: String,
    /// Model parameters: numbers or expressions of each other (D57).
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    parameters: BTreeMap<String, Num>,
    features: Vec<Map<String, Value>>,
    /// The variants of parts (D152, D169): each what differs from its parent.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    levels: BTreeMap<String, levels::Level>,
    /// The root's bodies as parts (D169), by the feature that began each: what each is (its
    /// name), or `null` for a body that is no part.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    parts: BTreeMap<String, Option<levels::PartInfo>>,
    /// The variant of each part in the product, when not the root's own.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    chosen: BTreeMap<String, String>,
    /// Parts not shown in the product.
    #[serde(default, skip_serializing_if = "std::collections::BTreeSet::is_empty")]
    hidden: std::collections::BTreeSet<String>,
    /// How each body is laid flat, when not the default (D164).
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    flats: BTreeMap<String, levels::FlatSettings>,
    /// The product's variants by name (D169): each part's variant and the parts hidden.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    sets: BTreeMap<String, levels::Set>,
}

/// The root's graph of a model file (its levels left out).
pub fn parse(json: &str) -> Result<Graph, ModelError> {
    parse_doc(json).map(|d| d.root)
}

/// A model file whole: the root's graph and the levels.
pub fn parse_doc(json: &str) -> Result<levels::Doc, ModelError> {
    let file: File = serde_json::from_str(json).map_err(ModelError::Json)?;
    if file.format != FORMAT {
        return Err(ModelError::Format(file.format));
    }
    let mut g = Graph::default();
    for (name, value) in file.parameters {
        g.set_parameter(&name, value).map_err(ModelError::Graph)?;
    }
    let mut features = file.features;
    migrate(&mut features).map_err(|msg| ModelError::Feature { index: 0, msg })?;
    for (index, fields) in features.into_iter().enumerate() {
        add_feature(&mut g, fields).map_err(|msg| ModelError::Feature { index, msg })?;
    }
    let mut doc =
        levels::Doc { root: g, levels: file.levels, parts: file.parts, chosen: file.chosen, hidden: file.hidden, flats: file.flats, sets: file.sets };
    doc.bring_over().map_err(ModelError::Levels)?;
    Ok(doc)
}

/// Files from before extrusion modes (D187) have a cut through as its own operation `cut`: it
/// becomes an extrusion cutting `through`, its inputs the same. Files from before patterns in
/// sketches (D128) also gave a cut a `pattern`: it becomes a pattern of the cut's sketch named
/// after the cut, its cell a group `<cut>_cell` of every profile curve of the sketch, as the cut
/// copied them all.
fn migrate(features: &mut [Map<String, Value>]) -> Result<(), String> {
    for k in 0..features.len() {
        if features[k].get("op").and_then(Value::as_str) != Some("cut") {
            continue;
        }
        features[k].insert("op".into(), "extrude".into());
        features[k].insert("mode".into(), "cut".into());
        features[k].insert("through".into(), true.into());
        let Some(pattern) = features[k].remove("pattern") else { continue };
        let cut = features[k].get("name").and_then(Value::as_str).unwrap_or_default().to_string();
        let of = features[k].get("inputs").and_then(|i| i.get(1)).and_then(Value::as_str).unwrap_or_default().to_string();
        let sketch = features[..k]
            .iter_mut()
            .find(|f| f.get("name").and_then(Value::as_str) == Some(&of))
            .and_then(|f| f.get_mut("sketch"))
            .and_then(Value::as_object_mut)
            .ok_or_else(|| format!("cut `{cut}`: its pattern goes to its sketch, and `{of}` is none before it"))?;
        let curves: Vec<Value> = ["lines", "arcs", "circles"]
            .iter()
            .filter_map(|key| sketch.get(*key).and_then(Value::as_object))
            .flat_map(|m| m.iter().filter(|(_, c)| c.get("construction") != Some(&Value::Bool(true))).map(|(n, _)| Value::String(n.clone())))
            .collect();
        let group = format!("{cut}_cell");
        let entry = |sketch: &mut Map<String, Value>, key: &str| -> Result<(), String> {
            match sketch.entry(key).or_insert_with(|| Value::Object(Map::new())) {
                Value::Object(_) => Ok(()),
                _ => Err(format!("`{of}`: `{key}` is not an object")),
            }
        };
        entry(sketch, "groups")?;
        entry(sketch, "patterns")?;
        sketch["groups"][&group] = Value::Array(curves);
        sketch["patterns"][&cut] = serde_json::json!({"of": group, "count": pattern["count"], "step": pattern["step"]});
    }
    Ok(())
}

/// Adds a feature given as in the file: `name`, optional `inputs` (earlier names), `op` and its
/// parameters. The same object is the `add` command's parameters.
pub fn add_feature(g: &mut Graph, mut fields: Map<String, Value>) -> Result<NodeIx, String> {
    let name = match fields.remove("name") {
        Some(Value::String(s)) => s,
        _ => return Err("`name` must be a string".into()),
    };
    let inputs: Vec<String> = match fields.remove("inputs") {
        None => Vec::new(),
        Some(v) => serde_json::from_value(v).map_err(|e| format!("`{name}`: inputs: {e}"))?,
    };
    let inputs = inputs
        .iter()
        .map(|i| g.find(i).ok_or_else(|| format!("`{name}`: unknown input `{i}` (features come after their inputs)")))
        .collect::<Result<Vec<_>, _>>()?;
    let enabled = fields.remove("enabled").map(|v| enabled_of(&name, v)).transpose()?;
    let title = match fields.remove("title") {
        None | Some(Value::Null) => None,
        Some(Value::String(t)) => Some(t),
        Some(_) => return Err(format!("`{name}`: title must be a string")),
    };
    if let Some(other) = g.nodes().iter().find(|n| n.title.as_deref() == Some(&name)) {
        return Err(format!("`{name}`: the key is the title of feature `{}`", other.name));
    }
    let op: Op = serde_json::from_value(Value::Object(fields)).map_err(|e| format!("`{name}`: {e}"))?;
    let ix = g.add(&name, op, &inputs).map_err(|e| e.to_string())?;
    if let Some(e) = enabled {
        g.set_enabled(ix, e);
    }
    if title.is_some() {
        set_title(g, ix, title)?;
    }
    Ok(ix)
}

/// Gives feature `ix` a title (none: removes it). A title is not empty, differs from every other
/// feature's title and key (a command given it would be ambiguous), and no key of a later feature
/// may equal it either: [`add_feature`] checks that when the key comes.
pub fn set_title(g: &mut Graph, ix: NodeIx, title: Option<String>) -> Result<(), String> {
    let title = title.map(|t| t.trim().to_string());
    if let Some(t) = &title {
        let name = &g.nodes()[ix as usize].name;
        if t.is_empty() {
            return Err(format!("`{name}`: the title is empty"));
        }
        if let Some(other) = g.nodes().iter().enumerate().find(|(i, n)| *i != ix as usize && (n.name == *t || n.title.as_deref() == Some(t))) {
            let what = if other.1.name == *t { "the key" } else { "the title" };
            return Err(format!("`{name}`: the title `{t}` is {what} of feature `{}`", other.1.name));
        }
    }
    g.set_title(ix, title);
    Ok(())
}

/// A feature's `enabled` (D159): `true`, `false`, a number (0: off) or a formula.
pub fn enabled_of(name: &str, v: Value) -> Result<Num, String> {
    match v {
        Value::Bool(b) => Ok(Num::Value(f64::from(u8::from(b)))),
        v => serde_json::from_value(v).map_err(|e| format!("`{name}`: enabled: {e}")),
    }
}

/// A feature as its file object.
pub fn feature_json(g: &Graph, ix: NodeIx) -> Map<String, Value> {
    let node = &g.nodes()[ix as usize];
    let mut fields = match serde_json::to_value(&node.op) {
        Ok(Value::Object(m)) => m,
        _ => Map::new(),
    };
    fields.insert("name".into(), Value::String(node.name.clone()));
    if let Some(t) = &node.title {
        fields.insert("title".into(), Value::String(t.clone()));
    }
    if node.enabled != Num::Value(1.0) {
        fields.insert("enabled".into(), serde_json::to_value(&node.enabled).unwrap_or(Value::Null));
    }
    if !node.inputs.is_empty() {
        let names = node.inputs.iter().map(|&i| Value::String(g.nodes()[i as usize].name.clone())).collect();
        fields.insert("inputs".into(), Value::Array(names));
    }
    fields
}

/// The features that bring geometry STEP has no exact type for into the final bodies (D147):
/// those on the way to them whose body has more such faces than any of their inputs'.
pub fn inexact_features(g: &Graph, e: &datum_graph::Evaluation) -> Vec<String> {
    let inexact = |i: NodeIx| e.results[i as usize].as_ref().ok().and_then(|v| v.body()).map_or(0, datum_brep::inexact_faces);
    let mut used = vec![false; g.nodes().len()];
    let mut todo = e.finals(g);
    while let Some(i) = todo.pop() {
        if !std::mem::replace(&mut used[i as usize], true) {
            todo.extend(&g.nodes()[i as usize].inputs);
        }
    }
    (0..g.nodes().len() as NodeIx)
        .filter(|&i| used[i as usize] && inexact(i) > g.nodes()[i as usize].inputs.iter().map(|&k| inexact(k)).max().unwrap_or(0))
        .map(|i| g.nodes()[i as usize].name.clone())
        .collect()
}

/// The first line of the note beside a STEP file with approximated geometry.
const NOTE: &str = "Datum STEP note";

/// Writes the final bodies of an evaluated model to `path` as a STEP part named after the file
/// (D130). Geometry STEP has no exact type for is refused with the features that bring it in,
/// or, when `approximate`, written as B-splines and told of in `<path>.txt` (D147); a note left
/// by an earlier export goes. Returns the faces approximated.
pub fn write_step(g: &Graph, e: &datum_graph::Evaluation, path: &std::path::Path, approximate: bool) -> Result<usize, String> {
    let bodies: Vec<&datum_brep::Body> = e.final_values(g).into_iter().filter_map(datum_graph::Value::body).collect();
    if bodies.is_empty() {
        return Err("the model has no body to export".into());
    }
    let features = inexact_features(g, e);
    if !features.is_empty() && !approximate {
        return Err(format!(
            "{}: geometry STEP has no exact type for (cuts through a rolled sheet along circles or oblique lines of its development); \
             agreed to, it is written as B-splines within {:e} mm",
            features.iter().map(|f| format!("`{f}`")).collect::<Vec<_>>().join(", "),
            datum_brep::DEVIATION
        ));
    }
    let file = path.file_name().map_or_else(String::new, |n| n.to_string_lossy().into_owned());
    let name = path.file_stem().map_or_else(String::new, |n| n.to_string_lossy().into_owned());
    let step = datum_brep::step(&bodies, &name, approximate).map_err(|err| err.to_string())?;
    let mut note = path.as_os_str().to_owned();
    note.push(".txt");
    let note = std::path::PathBuf::from(note);
    let write = |p: &std::path::Path, text: &str| std::fs::write(p, text).map_err(|err| format!("{}: {err}", p.display()));
    write(path, &step.text)?;
    if step.approximated > 0 {
        let (list, n, d, bound) = (features.join(", "), step.approximated, step.deviation, datum_brep::DEVIATION);
        write(
            &note,
            &format!(
                "{NOTE}: {file}\n\
                 Geometry STEP has no exact type for is approximated by B-splines of degree 5.\n\
                 Features: {list}.\n\
                 Faces approximated: {n}; largest deviation from the model {d:.1e} mm (bound {bound:e} mm).\n\
                 Volume, surface area and centre of mass in the file are those of the exact model: the receiving CAD checks the splines against them.\n\
                 \n\
                 Заметка Datum к STEP: {file}\n\
                 Геометрия, у которой в STEP нет точного типа, записана B-сплайнами 5-й степени.\n\
                 Операции: {list}.\n\
                 Приближённых граней: {n}; наибольшее отклонение от модели {d:.1e} мм (не больше {bound:e} мм).\n\
                 Объём, площадь и центр масс в файле — точной модели: принимающая CAD сверяет с ними сплайны.\n"
            ),
        )?;
    } else if std::fs::read_to_string(&note).is_ok_and(|t| t.starts_with(NOTE)) {
        std::fs::remove_file(&note).map_err(|err| format!("{}: {err}", note.display()))?;
    }
    Ok(step.approximated)
}

/// Canonical JSON: sorted keys, two-space indent, trailing newline.
pub fn to_json(g: &Graph) -> String {
    to_json_doc(&levels::Doc { root: g.clone(), ..levels::Doc::default() })
}

/// [`to_json`] of a whole model, its levels too.
pub fn to_json_doc(d: &levels::Doc) -> String {
    let g = &d.root;
    let features = (0..g.nodes().len() as NodeIx).map(|i| feature_json(g, i)).collect();
    let file = File {
        format: FORMAT.into(),
        parameters: g.parameters().clone(),
        features,
        levels: d.levels.clone(),
        parts: d.parts.clone(),
        chosen: d.chosen.clone(),
        hidden: d.hidden.clone(),
        flats: d.flats.clone(),
        sets: d.sets.clone(),
    };
    let mut s = serde_json::to_string_pretty(&file).unwrap_or_default();
    s.push('\n');
    s
}
