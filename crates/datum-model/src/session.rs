//! An editing session and the command registry (ARCHITECTURE §6.1): one table of commands with
//! JSON parameters, used by MCP and any other front end.

use std::path::PathBuf;

use datum_brep::{Body, Curve, Ix, Surface};
use datum_graph::{Evaluation, Evaluator, Graph, NodeIx, Op, Value as Feature};
use datum_math::Vec3;
use datum_math::expr::{Num, parameter_values};
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
        name: "export_dxf",
        description: "Write the flat blank of an `unfold` feature as DXF (R12: lines, arcs, circles on layer 0, mm) to `path`.",
        params: || json!({"type": "object", "properties": {"name": {"type": "string"}, "path": {"type": "string"}}, "required": ["name", "path"]}),
        run: |s, p| {
            let name = text(p, "name")?;
            let flat = s.value(name)?.flat().ok_or_else(|| format!("`{name}` is not an unfold"))?;
            let path = PathBuf::from(text(p, "path")?);
            std::fs::write(&path, flat.dxf()).map_err(|e| format!("{}: {e}", path.display()))?;
            Ok(json!({"written": path.display().to_string(), "width": flat.size.x, "height": flat.size.y, "pieces": flat.pieces.len()}))
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
            let ix = add_feature(&mut s.graph, p.clone())?;
            Ok(s.rebuild_recording(ix))
        },
    },
    Command {
        name: "set_parameter",
        description: "Set a model parameter to a number or an expression of other parameters (e.g. {name: \"t\", value: \"w / 5\"}). \
                      Numbers of features and sketch dimensions may be expressions of parameters: `+ - * /`, parentheses, `pi`, \
                      sqrt, abs, min, max, sin, cos, tan (degrees).",
        params: || {
            json!({"type": "object", "properties": {
                "name": {"type": "string"},
                "value": {"type": ["number", "string"]}
            }, "required": ["name", "value"]})
        },
        run: |s, p| {
            let value: Num = serde_json::from_value(p.get("value").cloned().unwrap_or(Value::Null)).map_err(|e| format!("value: {e}"))?;
            s.graph.set_parameter(text(p, "name")?, value).map_err(|e| e.to_string())?;
            Ok(s.rebuild())
        },
    },
    Command {
        name: "remove_parameter",
        description: "Remove a model parameter; features still using it then fail and say so.",
        params: name_param,
        run: |s, p| {
            s.graph.remove_parameter(text(p, "name")?).map_err(|e| e.to_string())?;
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
            Ok(s.rebuild_recording(ix))
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
        description: "Faces of a feature's body with their stable IDs and areas: a plane face with its unit normal and offset, a \
                      cylindrical one with its axis, radius and whether it is convex (a boss) or not (a hole). With `normal`, only \
                      plane faces facing that direction within `tolerance_deg` (default 0.01°).",
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
                .filter(|(_, f)| want.is_none_or(|w| f.surface.plane().is_some_and(|p| unit(p.normal).dot(unit(w)) >= cos_tol)))
                .map(|(i, f)| {
                    let (id, area) = (format!("{:?}", f.id), b.face_area(i as Ix));
                    match f.surface {
                        Surface::Plane(p) => {
                            let n = unit(p.normal);
                            json!({"id": id, "normal": n, "offset": n.dot(p.origin), "area": area})
                        }
                        Surface::Cylinder(c) => {
                            json!({"id": id, "cylinder": {"origin": c.origin, "axis": c.axis, "radius": c.radius, "convex": c.convex}, "area": area})
                        }
                        // A wall of a cut through a rolled sheet: its piece in the sheet's development.
                        Surface::Wall(w) => {
                            let arc = w.arc.map(|a| json!({"center": a.center, "ccw": a.ccw}));
                            json!({"id": id, "wall": {"axis": w.dev.axis, "neutral_radius": w.dev.neutral, "from": w.from, "to": w.to, "arc": arc}, "area": area})
                        }
                    }
                })
                .collect();
            Ok(json!({"count": faces.len(), "faces": faces}))
        },
    },
    Command {
        name: "query_edges",
        description: "Edges of a feature's body with their stable IDs, to project into a sketch: each runs from its lower-ID vertex \
                      (`from`, a sketch projection's `<name>.start`) to the other (`to`), with the vertices' IDs and points; an arc \
                      with its centre, axis and radius. With `face` (an ID from query_faces), only that face's edges, loop by loop.",
        params: || json!({"type": "object", "properties": {"name": {"type": "string"}, "face": {"type": "string"}}, "required": ["name"]}),
        run: |s, p| {
            let b = s.body(text(p, "name")?)?;
            let edges: Vec<Ix> = match p.get("face").and_then(Value::as_str) {
                None => (0..b.edges().len() as Ix).collect(),
                Some(id) => {
                    let f = u128::from_str_radix(id, 16)
                        .ok()
                        .and_then(|id| b.faces().binary_search_by_key(&id, |f| f.id.0).ok())
                        .ok_or_else(|| format!("no face `{id}` in `{}`", text(p, "name").unwrap_or_default()))?;
                    b.face_loops(f as Ix).flat_map(|l| b.loop_half_edges(l)).map(|h| b.half_edges()[h as usize].edge).collect()
                }
            };
            let vertex = |v: Ix| json!({"id": format!("{:?}", b.vertices()[v as usize].id), "point": b.point(v)});
            let list: Vec<Value> = edges
                .iter()
                .map(|&e| {
                    let edge = &b.edges()[e as usize];
                    let (from, to) = (b.half_edges()[edge.half as usize].origin, b.half_edges()[b.next(edge.half) as usize].origin);
                    let mut j = json!({"id": format!("{:?}", edge.id), "from": vertex(from), "to": vertex(to)});
                    match b.edge_curve(edge) {
                        Some(Curve::Arc(a)) => j["arc"] = json!({"center": a.center, "axis": a.axis, "radius": a.radius}),
                        Some(Curve::Rolled { .. }) => j["rolled"] = json!(true),
                        None => {}
                    }
                    j
                })
                .collect();
            Ok(json!({"count": list.len(), "edges": list}))
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
    /// The last evaluation: a result per feature.
    pub fn evaluation(&self) -> &Evaluation {
        &self.last
    }

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

    /// Rebuilds after feature `ix` was added or edited. A sketch projection given without `at`
    /// gets where it is now, as drawn (D87): later edits upstream that move the projected edge
    /// do not flip the sketch.
    fn rebuild_recording(&mut self, ix: NodeIx) -> Value {
        let summary = self.rebuild();
        let node = &self.graph.nodes()[ix as usize];
        let (Op::Sketch { project, .. }, Some(Ok(Feature::Sketch(placed)))) = (&node.op, self.last.results.get(ix as usize).map(|r| r.as_deref()))
        else {
            return summary;
        };
        if project.values().all(|p| !p.at.is_empty()) {
            return summary;
        }
        let mut op = node.op.clone();
        if let Op::Sketch { project, .. } = &mut op {
            for (name, p) in project.iter_mut().filter(|(_, p)| p.at.is_empty()) {
                p.at = placed.projected.get(name).cloned().unwrap_or_default();
            }
        }
        // What the edit recomputed: the second rebuild only reads the projections back.
        if self.graph.set_op(ix, op).is_ok() {
            self.rebuild();
        }
        summary
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

    /// The value of feature `name` in the last evaluation, or why there is none.
    pub fn value(&self, name: &str) -> Result<&Feature, String> {
        let ix = self.find(name)?;
        self.last.results[ix as usize].as_deref().map_err(|e| format!("`{name}` failed: {e}"))
    }

    fn body(&self, name: &str) -> Result<&Body, String> {
        self.value(name)?.body().ok_or_else(|| format!("`{name}` is not a body"))
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
                                json!({"sketch": {"dof": s.solved.dof, "free_points": s.solved.free_points, "points": points, "measures": s.solved.measures}, "hash": hash})
                            }
                            Feature::Flat(f) => {
                                let [lines, arcs, circles] = f.counts();
                                json!({"flat": {"width": f.size.x, "height": f.size.y, "lines": lines, "arcs": arcs, "circles": circles}, "hash": hash})
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
        let values = parameter_values(self.graph.parameters());
        let parameters: serde_json::Map<String, Value> = self
            .graph
            .parameters()
            .iter()
            .map(|(n, def)| {
                let v = match &values[n] {
                    Ok(v) => json!({"definition": def, "value": v}),
                    Err(e) => json!({"definition": def, "error": e.to_string()}),
                };
                (n.clone(), v)
            })
            .collect();
        json!({"file": self.path.as_ref().map(|p| p.display().to_string()), "parameters": parameters, "features": features})
    }
}
