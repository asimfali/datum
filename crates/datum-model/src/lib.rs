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

use std::fmt;

use datum_graph::{Graph, GraphError, Op};
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
    features: Vec<Map<String, Value>>,
}

pub fn parse(json: &str) -> Result<Graph, ModelError> {
    let file: File = serde_json::from_str(json).map_err(ModelError::Json)?;
    if file.format != FORMAT {
        return Err(ModelError::Format(file.format));
    }
    let mut g = Graph::default();
    for (index, mut fields) in file.features.into_iter().enumerate() {
        let err = |msg: String| ModelError::Feature { index, msg };
        let name = match fields.remove("name") {
            Some(Value::String(s)) => s,
            _ => return Err(err("`name` must be a string".into())),
        };
        let inputs: Vec<String> = match fields.remove("inputs") {
            None => Vec::new(),
            Some(v) => serde_json::from_value(v).map_err(|e| err(format!("`{name}`: inputs: {e}")))?,
        };
        let inputs = inputs
            .iter()
            .map(|i| g.find(i).ok_or_else(|| err(format!("`{name}`: unknown input `{i}` (features come after their inputs)"))))
            .collect::<Result<Vec<_>, _>>()?;
        let op: Op = serde_json::from_value(Value::Object(fields)).map_err(|e| err(format!("`{name}`: {e}")))?;
        g.add(&name, op, &inputs).map_err(ModelError::Graph)?;
    }
    Ok(g)
}

/// Canonical JSON: sorted keys, two-space indent, trailing newline.
pub fn to_json(g: &Graph) -> String {
    let features = g
        .nodes()
        .iter()
        .map(|node| {
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
        })
        .collect();
    let mut s = serde_json::to_string_pretty(&File { format: FORMAT.into(), features }).unwrap_or_default();
    s.push('\n');
    s
}
