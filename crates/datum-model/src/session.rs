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
    /// Models before each change, the latest last; and those undone, for `redo`.
    undo: Vec<Graph>,
    redo: Vec<Graph>,
}

/// Changes `undo` goes back through.
const UNDO_DEPTH: usize = 100;

impl Default for Session {
    fn default() -> Self {
        let evaluator = Evaluator::default();
        let last = evaluator.evaluate(&Graph::default());
        Self { graph: Graph::default(), evaluator, last, path: None, undo: Vec::new(), redo: Vec::new() }
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
        name: "edit_sketch",
        description: "Change part of a sketch feature `name` without rewriting it. `points`, `lines`, `arcs`, `circles`, `measures` \
                      and `project` are merged by name: an entry replaces the one of that name, null removes it, and with it \
                      whatever refers to it (the lines through a removed point, the constraints and dimensions naming either: \
                      listed in `also_removed`). `add_constraints` appends constraints; `remove_constraints` removes those equal \
                      to the ones given (as `inspect` lists them).",
        params: || {
            let map = json!({"type": "object", "additionalProperties": true});
            let list = json!({"type": "array", "items": {"type": "object"}});
            json!({"type": "object", "properties": {
                "name": {"type": "string"},
                "points": map, "lines": map, "arcs": map, "circles": map, "measures": map, "project": map,
                "add_constraints": list, "remove_constraints": list
            }, "required": ["name"], "additionalProperties": false})
        },
        run: |s, p| {
            let ix = s.find(text(p, "name")?)?;
            let Ok(Value::Object(mut op)) = serde_json::to_value(&s.graph.nodes()[ix as usize].op) else { return Err("unreadable feature".into()) };
            if op.get("op") != Some(&json!("sketch")) {
                return Err(format!("`{}` is not a sketch", text(p, "name")?));
            }
            let also = patch_sketch(&mut op, p)?;
            let op: Op = serde_json::from_value(Value::Object(op)).map_err(|e| e.to_string())?;
            s.graph.set_op(ix, op).map_err(|e| e.to_string())?;
            let mut reply = s.rebuild_recording(ix);
            reply["also_removed"] = json!(also);
            Ok(reply)
        },
    },
    Command {
        name: "rename_in_sketch",
        description: "Rename a point, line, arc, circle, reference dimension or projection of sketch feature `name` from `from` to \
                      `to`, everywhere it is used: the sketch's constraints, a sheet's `start`. Names of faces made from the sketch \
                      change with it; faces, edges and vertices referred to downstream by ID are found again by their geometry, \
                      which a rename leaves as it was. The reply lists references it could not find again.",
        params: || {
            json!({"type": "object", "properties": {"name": {"type": "string"}, "from": {"type": "string"}, "to": {"type": "string"}},
                   "required": ["name", "from", "to"]})
        },
        run: |s, p| s.rename_in_sketch(text(p, "name")?, text(p, "from")?, text(p, "to")?),
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
        name: "undo",
        description: "Go back one change of the model: one command, whatever it changed.",
        params: no_params,
        run: |s, _| s.step(true),
    },
    Command { name: "redo", description: "Make a change undone again.", params: no_params, run: |s, _| s.step(false) },
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
        if matches!(c.name, "undo" | "redo") {
            return (c.run)(self, p);
        }
        // A command changes the model whole or not at all: one that fails halfway leaves it as
        // it was, and each change is one step of `undo`.
        let before = self.graph.clone();
        let results_before = self.last.results.clone();
        let mut result = (c.run)(self, p);
        if let Ok(reply) = &mut result {
            let changed = self.shape_changes(&before, &results_before);
            if !changed.is_empty() {
                reply["shape_changed"] = json!(changed);
            }
        }
        match (&result, self.graph == before) {
            (_, true) => {}
            (Err(_), false) => {
                self.graph = before;
                self.rebuild();
            }
            (Ok(_), false) if matches!(c.name, "new" | "open") => {
                self.undo.clear();
                self.redo.clear();
            }
            (Ok(_), false) => {
                self.undo.push(before);
                if self.undo.len() > UNDO_DEPTH {
                    self.undo.remove(0);
                }
                self.redo.clear();
            }
        }
        result
    }

    /// Bodies whose shape (pieces, through holes; [`Body::shape`]) an edit changed, by feature:
    /// a hole pushed out of its plate becomes a disc beside it, which no dimension shows.
    fn shape_changes(&self, graph_before: &Graph, before: &[Result<datum_graph::ValueRef, datum_graph::EvalError>]) -> Vec<Value> {
        let shape = |(shells, holes): (usize, usize)| json!({"pieces": shells, "through_holes": holes});
        self.graph
            .nodes()
            .iter()
            .zip(&self.last.results)
            .filter_map(|(n, now)| {
                let old = graph_before.find(&n.name).and_then(|i| before.get(i as usize))?.as_ref().ok()?;
                let now = now.as_ref().ok()?;
                if std::sync::Arc::ptr_eq(old, now) {
                    return None;
                }
                let (a, b) = (old.body()?.shape(), now.body()?.shape());
                (a != b).then(|| json!({"feature": n.name, "before": shape(a), "after": shape(b)}))
            })
            .collect()
    }

    /// One step back (`undo`) or forward.
    fn step(&mut self, back: bool) -> Result<Value, String> {
        let (from, to) = if back { (&mut self.undo, &mut self.redo) } else { (&mut self.redo, &mut self.undo) };
        let g = from.pop().ok_or(if back { "nothing to undo" } else { "nothing to redo" })?;
        to.push(std::mem::replace(&mut self.graph, g));
        Ok(self.rebuild())
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

    fn rename_in_sketch(&mut self, feature: &str, from: &str, to: &str) -> Result<Value, String> {
        let ix = self.find(feature)?;
        if to.is_empty() || to.contains('.') || from == to || to == datum_ops::ORIGIN {
            return Err(format!(
                "the new name must be a different name, without `.` (it would read as a projection's point), not `{}`",
                datum_ops::ORIGIN
            ));
        }
        let Ok(Value::Object(mut op)) = serde_json::to_value(&self.graph.nodes()[ix as usize].op) else { return Err("unreadable feature".into()) };
        let sketch = op.get_mut("sketch").and_then(Value::as_object_mut).ok_or_else(|| format!("`{feature}` is not a sketch"))?;
        // The names a sketch defines: its elements', its projections' and their points'.
        let mut defined: Vec<String> = ["points", "lines", "arcs", "circles", "measures"]
            .iter()
            .filter_map(|k| sketch.get(*k).and_then(Value::as_object))
            .flat_map(|m| m.keys().cloned())
            .collect();
        let projections: Vec<String> = op.get("project").and_then(Value::as_object).map(|m| m.keys().cloned().collect()).unwrap_or_default();
        defined.extend(projections.iter().flat_map(|n| [n.clone(), format!("{n}.start"), format!("{n}.end"), format!("{n}.center")]));
        if !defined.iter().any(|n| n == from) {
            return Err(format!("`{feature}` has no `{from}`"));
        }
        if defined.iter().any(|n| n == to) {
            return Err(format!("`{feature}` already has `{to}`"));
        }
        let projection = projections.iter().any(|n| n == from);
        let renamed = |n: &str| match n.strip_prefix(from) {
            Some("") => Some(to.to_string()),
            Some(rest) if projection && rest.starts_with('.') => Some(format!("{to}{rest}")),
            _ => None,
        };
        rename_names(&mut op, &renamed);
        let mut ops: Vec<(NodeIx, Op)> = vec![(ix, serde_json::from_value(Value::Object(op)).map_err(|e| e.to_string())?)];
        for (j, n) in self.graph.nodes().iter().enumerate().skip(ix as usize + 1) {
            if let (Op::Sheet { start: Some(st), .. }, true) = (&n.op, n.inputs.contains(&ix))
                && let Some(new) = renamed(st)
            {
                let mut op = n.op.clone();
                if let Op::Sheet { start, .. } = &mut op {
                    *start = Some(new);
                }
                ops.push((j as NodeIx, op));
            }
        }
        let before = self.last.results.clone();
        for (j, op) in ops {
            self.graph.set_op(j, op).map_err(|e| e.to_string())?;
        }
        let summary = self.rebuild();
        // References by ID downstream: the same element, found by its unchanged geometry.
        let mut lost = Vec::new();
        let mut remapped = Vec::new();
        for (j, n) in self.graph.nodes().iter().enumerate().skip(ix as usize + 1) {
            let Some(&input) = n.inputs.first() else { continue };
            let (Some(old), Some(new)) = (body_of(&before, input), body_of(&self.last.results, input)) else { continue };
            let mut op = n.op.clone();
            let mut changed = false;
            let mut again = |id: &mut String, kinds: &[Kind], what: String| match find_again(old, new, id, kinds) {
                Some(found) if found != *id => {
                    *id = found;
                    changed = true;
                }
                Some(_) => {}
                None => lost.push(what),
            };
            match &mut op {
                Op::Sketch { face, project, .. } => {
                    if let Some(f) = face {
                        again(f, &[Kind::Face], format!("{}: face", n.name));
                    }
                    for (pn, pr) in project.iter_mut() {
                        again(&mut pr.id, &[Kind::Vertex, Kind::Edge], format!("{}: projection `{pn}`", n.name));
                    }
                }
                Op::Unfold { face, .. } => again(face, &[Kind::Face], format!("{}: face", n.name)),
                _ => {}
            }
            if changed {
                remapped.push((j as NodeIx, op));
            }
        }
        let found_again = remapped.len();
        for (j, op) in remapped {
            self.graph.set_op(j, op).map_err(|e| e.to_string())?;
        }
        if found_again > 0 {
            self.rebuild();
        }
        let failed: Vec<Value> = self.failures().collect();
        Ok(json!({"renamed": {"from": from, "to": to}, "features_with_ids_found_again": found_again, "lost": lost, "failed": failed,
                  "computed": summary["computed"]}))
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
                                // A free point: "plane", or the direction it slides along.
                                let free: Map<String, Value> =
                                    s.solved.free.iter().map(|(n, d)| (n.clone(), d.map_or(json!("plane"), |d| json!([d.x, d.y])))).collect();
                                let [o, x, y, z] = s.frame;
                                let frame = json!({"origin": o, "x": x, "y": y, "normal": z});
                                json!({"sketch": {"dof": s.solved.dof, "free": free, "points": points, "measures": s.solved.measures, "frame": frame}, "hash": hash})
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

/// Renames, in a sketch feature's JSON, every name `renamed` maps: keys of the element maps and
/// of `project`, and every string naming an element. Expressions (`value`, `degrees`,
/// `diameter`), constraint kinds and IDs are left alone.
fn rename_names(op: &mut Map<String, Value>, renamed: &impl Fn(&str) -> Option<String>) {
    let rekey = |m: &mut Value| {
        if let Some(m) = m.as_object_mut() {
            *m = std::mem::take(m).into_iter().map(|(k, v)| (renamed(&k).unwrap_or(k), v)).collect();
        }
    };
    if let Some(p) = op.get_mut("project") {
        rekey(p);
    }
    if let Some(sketch) = op.get_mut("sketch") {
        if let Some(sk) = sketch.as_object_mut() {
            for key in ["points", "lines", "arcs", "circles", "measures"] {
                if let Some(m) = sk.get_mut(key) {
                    rekey(m);
                }
            }
        }
        visit_names(sketch, &mut |s| {
            if let Some(n) = renamed(s) {
                *s = n;
            }
        });
    }
}

/// Calls `f` on every string in a part of a sketch's JSON that names an element: all of them but
/// expressions (`value`, `degrees`, `diameter`), constraint kinds and IDs.
pub fn visit_names(v: &mut Value, f: &mut impl FnMut(&mut String)) {
    match v {
        Value::String(s) => f(s),
        Value::Array(a) => a.iter_mut().for_each(|v| visit_names(v, f)),
        Value::Object(m) => {
            for (k, v) in m.iter_mut() {
                if !["type", "value", "degrees", "diameter", "id"].contains(&k.as_str()) {
                    visit_names(v, f);
                }
            }
        }
        _ => {}
    }
}

/// The body feature `ix` evaluated to in `results`, if it did.
fn body_of(results: &[Result<datum_graph::ValueRef, datum_graph::EvalError>], ix: NodeIx) -> Option<&Body> {
    results.get(ix as usize)?.as_ref().ok()?.body()
}

#[derive(Clone, Copy, PartialEq)]
enum Kind {
    Vertex,
    Edge,
    Face,
}

/// The ID in `new` of the element with ID `id` (hex) in `old`, found by its geometry: a vertex by
/// its point, an edge by its ends and curve, a face by its surface and the points of its loops,
/// all compared exactly. None if `old` has no such element or `new` has not exactly one.
fn find_again(old: &Body, new: &Body, id: &str, kinds: &[Kind]) -> Option<String> {
    let id = u128::from_str_radix(id, 16).ok()?;
    let bits = |p: Vec3| p.to_array().map(f64::to_bits);
    let vertex = |b: &Body, v: usize| format!("{:?}", bits(b.vertices()[v].point));
    let edge = |b: &Body, e: usize| {
        let ed = &b.edges()[e];
        let mut ends = [b.half_edges()[ed.half as usize].origin, b.half_edges()[b.next(ed.half) as usize].origin].map(|v| bits(b.point(v)));
        ends.sort_unstable();
        format!("{ends:?} {:?}", b.edge_curve(ed))
    };
    let face = |b: &Body, f: usize| {
        let mut pts: Vec<[u64; 3]> =
            b.face_loops(f as Ix).flat_map(|l| b.loop_half_edges(l)).map(|h| bits(b.point(b.half_edges()[h as usize].origin))).collect();
        pts.sort_unstable();
        format!("{:?} {pts:?}", b.faces()[f].surface)
    };
    for &kind in kinds {
        let found: Vec<_> = match kind {
            Kind::Vertex => {
                let Ok(v) = old.vertices().binary_search_by_key(&id, |v| v.id.0) else { continue };
                let key = vertex(old, v);
                (0..new.vertices().len()).filter(|&w| vertex(new, w) == key).map(|w| new.vertices()[w].id).collect()
            }
            Kind::Edge => {
                let Ok(e) = old.edges().binary_search_by_key(&id, |e| e.id.0) else { continue };
                let key = edge(old, e);
                (0..new.edges().len()).filter(|&w| edge(new, w) == key).map(|w| new.edges()[w].id).collect()
            }
            Kind::Face => {
                let Ok(f) = old.faces().binary_search_by_key(&id, |f| f.id.0) else { continue };
                let key = face(old, f);
                (0..new.faces().len()).filter(|&w| face(new, w) == key).map(|w| new.faces()[w].id).collect()
            }
        };
        return match found[..] {
            [one] => Some(format!("{one:?}")),
            _ => None,
        };
    }
    None
}

/// Applies an `edit_sketch` patch to a sketch feature's JSON (`op` and its fields): element maps
/// and `project` merged by name (null removes, and with it whatever refers to it), constraints
/// added and removed. Returns what went with the removed. One place for the session and the
/// window, which applies its own edits at once rather than wait for the rebuild.
pub fn patch_sketch(op: &mut Map<String, Value>, p: &Map<String, Value>) -> Result<Vec<String>, String> {
    let merge = |target: &mut Map<String, Value>, key: &str, patch: Option<&Value>| -> Result<(), String> {
        let Some(patch) = patch else { return Ok(()) };
        let patch = patch.as_object().ok_or_else(|| format!("`{key}` must be an object"))?;
        let entries = target.entry(key).or_insert_with(|| json!({}));
        let entries = entries.as_object_mut().ok_or_else(|| format!("`{key}` is not an object"))?;
        for (n, v) in patch {
            match v {
                Value::Null => entries.remove(n).map(|_| ()).ok_or_else(|| format!("`{key}` has no `{n}`"))?,
                v => {
                    entries.insert(n.clone(), v.clone());
                }
            }
        }
        Ok(())
    };
    // Names removed, and the projections among them (their points go too).
    let removed_in = |key: &str| -> Vec<String> {
        p.get(key).and_then(Value::as_object).map(|m| m.iter().filter(|(_, v)| v.is_null()).map(|(n, _)| n.clone()).collect()).unwrap_or_default()
    };
    let mut gone: Vec<String> = ["points", "lines", "arcs", "circles", "measures"].iter().flat_map(|k| removed_in(k)).collect();
    let projections = removed_in("project");
    gone.extend(projections.iter().flat_map(|n| [n.clone(), format!("{n}.start"), format!("{n}.end"), format!("{n}.center")]));
    merge(op, "project", p.get("project"))?;
    let sketch = op.get_mut("sketch").and_then(Value::as_object_mut).ok_or("the sketch is missing")?;
    for key in ["points", "lines", "arcs", "circles", "measures"] {
        merge(sketch, key, p.get(key))?;
    }
    let mut also = Vec::new();
    let refers = |v: &Value, gone: &[String]| {
        let mut v = v.clone();
        let mut found = false;
        visit_names(&mut v, &mut |n| found |= gone.contains(n));
        found
    };
    // Curves on removed points, then dimensions on anything removed.
    for key in ["lines", "arcs", "circles", "measures"] {
        if let Some(m) = sketch.get_mut(key).and_then(Value::as_object_mut) {
            let doomed: Vec<String> = m.iter().filter(|(_, v)| refers(v, &gone)).map(|(n, _)| n.clone()).collect();
            for n in doomed {
                m.remove(&n);
                also.push(format!("{key}: {n}"));
                gone.push(n);
            }
        }
    }
    if let Some(c) = sketch.get_mut("constraints").and_then(Value::as_array_mut) {
        let before = c.len();
        c.retain(|c| !refers(c, &gone));
        if c.len() < before {
            also.push(format!("constraints: {}", before - c.len()));
        }
    }
    let constraints = sketch.entry("constraints").or_insert_with(|| json!([]));
    let constraints = constraints.as_array_mut().ok_or("`constraints` is not a list")?;
    // Constraints compare as parsed: field order and number spelling do not matter.
    let parse = |v: &Value| serde_json::from_value::<datum_sketch::Constraint>(v.clone()).map_err(|e| format!("constraint {v}: {e}"));
    for gone in p.get("remove_constraints").and_then(Value::as_array).into_iter().flatten() {
        let gone = parse(gone)?;
        let k = constraints.iter().position(|c| parse(c).is_ok_and(|c| c == gone)).ok_or_else(|| format!("no constraint {gone:?}"))?;
        constraints.remove(k);
    }
    for added in p.get("add_constraints").and_then(Value::as_array).into_iter().flatten() {
        parse(added)?;
        constraints.push(added.clone());
    }
    Ok(also)
}
