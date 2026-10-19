//! An editing session and the command registry (ARCHITECTURE §6.1): one table of commands with
//! JSON parameters, used by MCP and any other front end.

use std::path::PathBuf;

use datum_brep::{Body, Ix};
use datum_graph::{Evaluation, Evaluator, Graph, NodeIx, Op, Value as Feature};
use datum_math::Vec3;
use serde_json::{Map, Value, json};

use crate::{add_feature, feature_json, parse, to_json};

/// The model being edited, its cache and its latest evaluation. Every edit re-evaluates; the
/// cache makes that cost proportional to what changed.
pub struct Session {
    graph: Graph,
    evaluator: Evaluator,
    last: Evaluation,
    path: Option<PathBuf>,
}

impl Default for Session {
    fn default() -> Self {
        let evaluator = Evaluator::default();
        let last = evaluator.evaluate(&Graph::default());
        Self { graph: Graph::default(), evaluator, last, path: None }
    }
}

struct Command {
    name: &'static str,
    description: &'static str,
    params: fn() -> Value,
    run: fn(&mut Session, &Map<String, Value>) -> Result<Value, String>,
}

const COMMANDS: &[Command] = &[
    Command { name: "new", description: "Start an empty model.", params: no_params, run: |s, _| Ok(s.replace(Graph::default(), None)) },
    Command {
        name: "open",
        description: "Open a model file (.json).",
        params: || json!({"type": "object", "properties": {"path": {"type": "string"}}, "required": ["path"]}),
        run: |s, p| {
            let path = PathBuf::from(text(p, "path")?);
            let src = std::fs::read_to_string(&path).map_err(|e| format!("{}: {e}", path.display()))?;
            let g = parse(&src).map_err(|e| format!("{}: {e}", path.display()))?;
            Ok(s.replace(g, Some(path)))
        },
    },
    Command {
        name: "save",
        description: "Save the model as canonical JSON. Without `path`: to the file it was opened from.",
        params: || json!({"type": "object", "properties": {"path": {"type": "string"}}}),
        run: |s, p| {
            let path = match p.get("path").and_then(Value::as_str) {
                Some(x) => PathBuf::from(x),
                None => s.path.clone().ok_or("the model has no file yet: give `path`")?,
            };
            std::fs::write(&path, to_json(&s.graph)).map_err(|e| format!("{}: {e}", path.display()))?;
            s.path = Some(path.clone());
            Ok(json!({"saved": path.display().to_string()}))
        },
    },
    Command {
        name: "add",
        description: "Add a feature at the end of the history: `name`, `inputs` (names of earlier features), `op` and its parameters, \
                      exactly as a feature in the model file.",
        params: feature_schema,
        run: |s, p| {
            add_feature(&mut s.graph, p.clone())?;
            Ok(s.rebuild())
        },
    },
    Command {
        name: "edit",
        description: "Change a feature's parameters: give `name` and only the fields to change (e.g. {name: \"moved\", angle_deg: 45}); \
                      `op` changes its kind. Inputs stay.",
        params: || json!({"type": "object", "properties": {"name": {"type": "string"}}, "required": ["name"], "additionalProperties": true}),
        run: |s, p| {
            let ix = s.find(text(p, "name")?)?;
            let mut fields = match serde_json::to_value(&s.graph.nodes()[ix as usize].op) {
                Ok(Value::Object(m)) => m,
                _ => Map::new(),
            };
            fields.extend(p.iter().filter(|(k, _)| *k != "name").map(|(k, v)| (k.clone(), v.clone())));
            let op: Op = serde_json::from_value(Value::Object(fields)).map_err(|e| e.to_string())?;
            s.graph.set_op(ix, op).map_err(|e| e.to_string())?;
            Ok(s.rebuild())
        },
    },
    Command {
        name: "remove",
        description: "Remove a feature that no other feature uses.",
        params: name_param,
        run: |s, p| {
            let ix = s.find(text(p, "name")?)?;
            if let Some(user) = s.graph.nodes().iter().find(|n| n.inputs.contains(&ix)) {
                return Err(format!("`{}` uses it: remove or change that first", user.name));
            }
            let mut g = Graph::default();
            for i in (0..s.graph.nodes().len() as NodeIx).filter(|&i| i != ix) {
                add_feature(&mut g, feature_json(&s.graph, i))?;
            }
            s.graph = g;
            Ok(s.rebuild())
        },
    },
    Command {
        name: "inspect",
        description: "The feature history with each feature's status: volume, element counts and hash, or why it failed.",
        params: no_params,
        run: |s, _| Ok(s.inspect()),
    },
    Command {
        name: "mass_properties",
        description: "Volume, surface area, centre of mass and bounding box of a feature's body.",
        params: name_param,
        run: |s, p| {
            let m = s.body(text(p, "name")?)?.mass_properties();
            Ok(json!({"volume": m.volume, "area": m.area, "centroid": m.centroid, "min": m.min, "max": m.max}))
        },
    },
    Command {
        name: "query_faces",
        description: "Faces of a feature's body with their stable IDs, unit normals, plane offsets and areas. With `normal`, only faces \
                      facing that direction within `tolerance_deg` (default 0.01°).",
        params: || {
            json!({"type": "object", "properties": {
                "name": {"type": "string"},
                "normal": {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3},
                "tolerance_deg": {"type": "number", "minimum": 0}
            }, "required": ["name"]})
        },
        run: |s, p| {
            let want = match p.get("normal") {
                Some(v) => Some(serde_json::from_value::<Vec3>(v.clone()).map_err(|e| format!("normal: {e}"))?),
                None => None,
            };
            let cos_tol = p.get("tolerance_deg").and_then(Value::as_f64).unwrap_or(0.01).to_radians().cos();
            let b = s.body(text(p, "name")?)?;
            let unit = |v: Vec3| v * (1.0 / v.dot(v).sqrt());
            let faces: Vec<Value> = b
                .faces()
                .iter()
                .enumerate()
                .filter(|(_, f)| want.is_none_or(|w| unit(f.plane.normal).dot(unit(w)) >= cos_tol))
                .map(|(i, f)| {
                    let n = unit(f.plane.normal);
                    let a = b.face_area_vector(i as Ix);
                    json!({"id": format!("{:?}", f.id), "normal": n, "offset": n.dot(f.plane.origin), "area": a.dot(a).sqrt()})
                })
                .collect();
            Ok(json!({"count": faces.len(), "faces": faces}))
        },
    },
];

fn no_params() -> Value {
    json!({"type": "object", "properties": {}})
}

fn name_param() -> Value {
    json!({"type": "object", "properties": {"name": {"type": "string"}}, "required": ["name"]})
}

/// A feature object: the operation's generated schema plus `name` and `inputs`.
fn feature_schema() -> Value {
    json!({
        "type": "object",
        "properties": {"name": {"type": "string"}, "inputs": {"type": "array", "items": {"type": "string"}}},
        "required": ["name", "op"],
        "allOf": [schemars::schema_for!(Op)]
    })
}

fn text<'p>(p: &'p Map<String, Value>, key: &str) -> Result<&'p str, String> {
    p.get(key).and_then(Value::as_str).ok_or_else(|| format!("`{key}` must be a string"))
}

impl Session {
    pub fn graph(&self) -> &Graph {
        &self.graph
    }

    /// Every command with its description and parameter schema.
    pub fn commands() -> Value {
        let list: Vec<Value> = COMMANDS.iter().map(|c| json!({"name": c.name, "description": c.description, "params": (c.params)()})).collect();
        json!({"commands": list})
    }

    /// Runs a command. Edits return the rebuild summary.
    pub fn execute(&mut self, command: &str, params: &Value) -> Result<Value, String> {
        let c = COMMANDS.iter().find(|c| c.name == command).ok_or_else(|| format!("unknown command `{command}` (see list_commands)"))?;
        let empty = Map::new();
        let p = match params {
            Value::Object(m) => m,
            Value::Null => &empty,
            _ => return Err("params must be an object".into()),
        };
        (c.run)(self, p)
    }

    fn replace(&mut self, g: Graph, path: Option<PathBuf>) -> Value {
        self.graph = g;
        self.path = path;
        self.rebuild()
    }

    fn rebuild(&mut self) -> Value {
        self.last = self.evaluator.evaluate(&self.graph);
        let failed: Vec<Value> = self.failures().collect();
        json!({"features": self.graph.nodes().len(), "computed": self.last.computed, "reused": self.last.reused, "failed": failed})
    }

    fn failures(&self) -> impl Iterator<Item = Value> + '_ {
        self.graph
            .nodes()
            .iter()
            .zip(&self.last.results)
            .filter_map(|(n, r)| r.as_ref().err().map(|e| json!({"feature": n.name, "error": e.to_string()})))
    }

    fn find(&self, name: &str) -> Result<NodeIx, String> {
        self.graph.find(name).ok_or_else(|| format!("no feature named `{name}`"))
    }

    fn body(&self, name: &str) -> Result<&Body, String> {
        let ix = self.find(name)?;
        let value = self.last.results[ix as usize].as_ref().map_err(|e| format!("`{name}` failed: {e}"))?;
        value.body().ok_or_else(|| format!("`{name}` is a sketch, not a body"))
    }

    fn inspect(&self) -> Value {
        let features: Vec<Value> = (0..self.graph.nodes().len() as NodeIx)
            .zip(&self.last.results)
            .map(|(i, r)| {
                let mut f = feature_json(&self.graph, i);
                match r {
                    Ok(v) => {
                        let hash: String = v.hash()[..8].iter().map(|x| format!("{x:02x}")).collect();
                        let status = match v.as_ref() {
                            Feature::Body(b) => {
                                json!({"volume": b.volume(), "faces": b.faces().len(), "edges": b.edges().len(), "vertices": b.vertices().len(), "hash": hash})
                            }
                            Feature::Sketch(s) => {
                                let points: serde_json::Map<String, Value> = s.solved.points.iter().map(|(n, p)| (n.clone(), json!([p.x, p.y]))).collect();
                                json!({"sketch": {"dof": s.solved.dof, "free_points": s.solved.free_points, "points": points}, "hash": hash})
                            }
                        };
                        f.insert("status".into(), status);
                    }
                    Err(e) => {
                        f.insert("error".into(), Value::String(e.to_string()));
                    }
                }
                Value::Object(f)
            })
            .collect();
        json!({"file": self.path.as_ref().map(|p| p.display().to_string()), "features": features})
    }
}
