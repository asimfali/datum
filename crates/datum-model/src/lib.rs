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
    Feature { index: usize, msg: String },
    Graph(GraphError),
}

impl fmt::Display for ModelError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Json(e) => write!(f, "JSON: {e}"),
            Self::Format(v) => write!(f, "format `{v}` is not supported (expected `{FORMAT}`)"),
            Self::Feature { index, msg } => write!(f, "feature #{index}: {msg}"),
            Self::Graph(e) => e.fmt(f),
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
}

pub fn parse(json: &str) -> Result<Graph, ModelError> {
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
    Ok(g)
}

/// Files from before patterns in sketches (D128) gave a cut a `pattern`: it becomes a pattern
/// of the cut's sketch named after the cut, its cell a group `<cut>_cell` of every profile curve
/// of the sketch, as the cut copied them all.
fn migrate(features: &mut [Map<String, Value>]) -> Result<(), String> {
    for k in 0..features.len() {
        if features[k].get("op").and_then(Value::as_str) != Some("cut") {
            continue;
        }
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
    let op: Op = serde_json::from_value(Value::Object(fields)).map_err(|e| format!("`{name}`: {e}"))?;
    g.add(&name, op, &inputs).map_err(|e| e.to_string())
}

/// A feature as its file object.
pub fn feature_json(g: &Graph, ix: NodeIx) -> Map<String, Value> {
    let node = &g.nodes()[ix as usize];
    let mut fields = match serde_json::to_value(&node.op) {
        Ok(Value::Object(m)) => m,
        _ => Map::new(),
    };
    fields.insert("name".into(), Value::String(node.name.clone()));
    if !node.inputs.is_empty() {
        let names = node.inputs.iter().map(|&i| Value::String(g.nodes()[i as usize].name.clone())).collect();
        fields.insert("inputs".into(), Value::Array(names));
    }
    fields
}

/// Canonical JSON: sorted keys, two-space indent, trailing newline.
pub fn to_json(g: &Graph) -> String {
    let features = (0..g.nodes().len() as NodeIx).map(|i| feature_json(g, i)).collect();
    let mut s = serde_json::to_string_pretty(&File { format: FORMAT.into(), parameters: g.parameters().clone(), features }).unwrap_or_default();
    s.push('\n');
    s
}
