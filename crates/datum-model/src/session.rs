//! An editing session and the command registry (ARCHITECTURE §6.1): one table of commands with
//! JSON parameters, used by MCP and any other front end.

use std::collections::BTreeMap;
use std::path::PathBuf;

use datum_brep::{Body, Curve, Ix, Surface};
use datum_graph::{Evaluation, Evaluator, Graph, NodeIx, Op, Value as Feature};
use datum_math::Vec3;
use datum_math::expr::{ExprError, Num, parameter_values};
use serde_json::{Map, Value, json};

use crate::{SketchPatch, add_feature, feature_json, library, parse, to_json};

/// The model being edited, its cache and its latest evaluation. Every edit re-evaluates; the
/// cache makes that cost proportional to what changed.
pub struct Session {
    graph: Graph,
    evaluator: Evaluator,
    last: Evaluation,
    path: Option<PathBuf>,
    /// The model as last opened or saved: what differs from it is not saved yet.
    saved: Graph,
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
        Self { graph: Graph::default(), evaluator, last, path: None, saved: Graph::default(), undo: Vec::new(), redo: Vec::new() }
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
            s.saved = s.graph.clone();
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
        description: "Change part of a sketch feature `name` without rewriting it. `points`, `lines`, `arcs`, `circles`, `measures`, \
                      `groups` and `project` are merged by name: an entry replaces the one of that name, null removes it, and with it \
                      whatever refers to it (the lines through a removed point, the constraints and dimensions naming either: \
                      listed in `also_removed`; a removed element leaves its groups, and a group left empty goes). A group is a \
                      list of element names (`{\"cell\": [\"h1\", \"h2\"]}`): only a union, removing it removes no element. \
                      A pattern copies the closed profiles of a group (or a curve) `of` on the grid of `step`: `count` \
                      copies along each step, or as many as `fill` (a group bounding a region, or a circle) holds, centred, \
                      `clearance` from its edge and every other profile; `skip` [[i, j]] leaves copies out, `shift` \
                      {\"i,j\": [dx, dy]} moves one: `{\"perf\": {\"of\": \"cell\", \"step\": [[19, 0], [0, 17]], \"fill\": \"panel\", \
                      \"clearance\": 3}}`. A cut and an extrusion take the copies with the sketch's own profiles; `inspect` \
                      gives each pattern's copies, profiles and open area. `add_constraints` appends constraints; `remove_constraints` removes those equal \
                      to the ones given (as `inspect` lists them).",
        params: || {
            let map = json!({"type": "object", "additionalProperties": true});
            let list = json!({"type": "array", "items": {"type": "object"}});
            json!({"type": "object", "properties": {
                "name": {"type": "string"},
                "points": map, "lines": map, "arcs": map, "circles": map, "measures": map, "groups": map, "patterns": map, "project": map,
                "add_constraints": list, "remove_constraints": list
            }, "required": ["name"], "additionalProperties": false})
        },
        run: |s, p| s.edit_sketch(text(p, "name")?, p),
    },
    Command {
        name: "library",
        description: "The library's elements of a `level` (VISION §10; `sketch` so far): built in, then from `library/<level>` \
                      beside the program, then from `library/<level>` of the model's project (the nearest folder above the model \
                      file with `datum-project.json`); one of a name already listed takes its place. Each with its `source`, \
                      `parameters` in the order they are asked, `presets` (typical values by name) and `anchors`. `skipped`: \
                      files that are not elements, with why.",
        params: || json!({"type": "object", "properties": {"level": {"type": "string", "enum": library::LEVELS}}}),
        run: |s, p| {
            let level = p.get("level").and_then(Value::as_str).unwrap_or("sketch");
            let lib = library::load(level, s.path.as_deref());
            let list: Vec<Value> = lib
                .entries
                .iter()
                .map(|e| {
                    let mut v = json!({"source": e.source});
                    for k in ["name", "description", "parameters", "presets", "anchors", "fit", "links"] {
                        if let Some(x) = e.element.get(k) {
                            v[k] = x.clone();
                        }
                    }
                    v
                })
                .collect();
            Ok(json!({"elements": list, "skipped": lib.skipped}))
        },
    },
    Command {
        name: "to_library",
        description: "Keep a piece of sketch feature `name` as a library element (D98): `select` names its curves (taken with \
                      their points) and points; the constraints and reference dimensions among them come along. `anchors`: \
                      its insertion points, one or two of its points (else the lowest, leftmost); with two, `fit` is `stretch` \
                      (along the axes, the default) or `align` (turned toward the second: a section of a chain). The model's \
                      parameters its dimensions name become the element's, with their values now, and so do the dimensions \
                      `ask` names (as `dimensions` of the reply lists them: `length l5` → its parameter name); `order` gives the order they are asked in (the rest follow). Saved as `element` (its name) \
                      with `description` to `path`, or else to `library/sketch` of the model's project (beside the program \
                      without one); an existing file is kept unless `replace`. The reply has the element and where it went.",
        params: || {
            json!({"type": "object", "properties": {
                "name": {"type": "string"},
                "select": {"type": "array", "items": {"type": "string"}, "minItems": 1},
                "element": {"type": "string"},
                "description": {"type": "string"},
                "order": {"type": "array", "items": {"type": "string"}},
                "anchors": {"type": "array", "items": {"type": "string"}, "minItems": 1, "maxItems": 2},
                "ask": {"type": "object", "additionalProperties": {"type": "string"}},
                "fit": {"type": "string", "enum": ["stretch", "align"]},
                "path": {"type": "string"},
                "replace": {"type": "boolean"}
            }, "required": ["name", "select", "element"], "additionalProperties": false})
        },
        run: |s, p| {
            let name = text(p, "name")?;
            let element_name = text(p, "element")?;
            let Ok(Feature::Sketch(placed)) = s.value(name) else { return Err(format!("`{name}` is not a sketch that solves")) };
            let feature = serde_json::to_value(&s.graph.nodes()[s.find(name)? as usize].op).map_err(|e| e.to_string())?;
            let select: Vec<String> =
                p.get("select").and_then(Value::as_array).into_iter().flatten().filter_map(|v| v.as_str().map(str::to_string)).collect();
            let sketch = &feature["sketch"];
            let curve = |n: &String| ["lines", "arcs", "circles"].iter().any(|k| sketch[*k].get(n.as_str()).is_some());
            if let Some(n) = select.iter().find(|n| !curve(n) && sketch["points"].get(n.as_str()).is_none()) {
                return Err(format!("`select`: `{name}` has no point or curve `{n}` of its own"));
            }
            let (curves, points): (Vec<String>, Vec<String>) = select.into_iter().partition(curve);
            let params: BTreeMap<String, f64> =
                parameter_values(s.graph.parameters()).into_iter().filter_map(|(n, v)| v.ok().map(|v| (n, v))).collect();
            let mut element =
                library::extract(&feature, &placed.solved.points, &curves, &points, element_name, Some(&params)).ok_or("nothing to keep")?;
            for (label, n) in p.get("ask").and_then(Value::as_object).into_iter().flatten() {
                library::parametrize(&mut element, label, n.as_str().ok_or("`ask`: parameter names")?)?;
            }
            if let Some(d) = p.get("description").and_then(Value::as_str) {
                element["description"] = json!(d);
            }
            if let (Some(order), Some(list)) = (p.get("order").and_then(Value::as_array), element.get_mut("parameters").and_then(Value::as_array_mut))
            {
                let rank = |v: &Value| order.iter().position(|o| *o == v["name"]).unwrap_or(order.len());
                list.sort_by_key(rank);
            }
            if let Some(a) = p.get("anchors") {
                element["anchors"] = a.clone();
            }
            if let Some(f) = p.get("fit") {
                element["fit"] = f.clone();
            }
            library::check("sketch", &element)?;
            let path = match p.get("path").and_then(Value::as_str) {
                Some(x) => PathBuf::from(x),
                None => library::folder("sketch", s.path.as_deref())?.join(library::file_name(element_name)),
            };
            if path.exists() && !p.get("replace").and_then(Value::as_bool).unwrap_or(false) {
                return Err(format!("{} exists: give `replace` to write over it", path.display()));
            }
            if let Some(dir) = path.parent() {
                std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
            }
            let text = serde_json::to_string_pretty(&element).map_err(|e| e.to_string())?;
            std::fs::write(&path, text).map_err(|e| format!("{}: {e}", path.display()))?;
            let dimensions: Vec<Value> = library::dimensions(&element).into_iter().map(|d| json!({"dimension": d.label, "value": d.value})).collect();
            Ok(json!({"element": element, "saved": path.display().to_string(), "dimensions": dimensions}))
        },
    },
    Command {
        name: "insert_element",
        description: "Insert a library element into sketch feature `name` as a copy (D98): its geometry, constraints and \
                      dimensions under new names, nothing tied to the library. `element`: a name from `library`, or an element \
                      object itself. Parameter values: the element's, over them `preset`'s, over those `values` (a number, or an \
                      expression of the model's parameters); with a spot for each of two anchors of a stretched element, a parameter \
                      not given that a dimension holds alone is measured between the spots. `at`: a spot for the first anchor or for each, a point of the sketch \
                      by name (the anchor is made coincident with it) or `[x, y]`; two anchors stretch the element along the \
                      sketch's axes, or with `fit: align` turn it toward the second. `names`: element names that take the given \
                      free names (a section put in place of one removed keeps its name, and so what referred to it downstream). \
                      `links`: the element's sockets (constraints reaching out of it, `perpendicular l5 ?`, as `library` lists \
                      them) completed by a point, line or curve of the sketch. The reply has the edit's result, `names`: each \
                      element name with the name it got, and `open_links`: the sockets left open.",
        params: || {
            json!({"type": "object", "properties": {
                "name": {"type": "string"},
                "element": {"oneOf": [{"type": "string"}, {"type": "object"}]},
                "preset": {"type": "string"},
                "values": {"type": "object", "additionalProperties": {"oneOf": [{"type": "number"}, {"type": "string"}]}},
                "at": {"type": "array", "minItems": 1, "maxItems": 2,
                       "items": {"oneOf": [{"type": "string"}, {"type": "array", "items": {"type": "number"}, "minItems": 2, "maxItems": 2}]}},
                "names": {"type": "object", "additionalProperties": {"type": "string"}},
                "links": {"type": "object", "additionalProperties": {"type": "string"}},
                "construction": {"type": "boolean"}
            }, "required": ["name", "element", "at"], "additionalProperties": false})
        },
        run: |s, p| {
            let name = text(p, "name")?;
            let element = match &p["element"] {
                Value::String(n) => library::load("sketch", s.path.as_deref())
                    .get(n)
                    .map(|e| e.element.clone())
                    .ok_or_else(|| format!("no element `{n}` in the library"))?,
                e => {
                    library::check("sketch", e)?;
                    e.clone()
                }
            };
            let Ok(Feature::Sketch(placed)) = s.value(name) else { return Err(format!("`{name}` is not a sketch that solves")) };
            let mut at = Vec::new();
            for spot in p.get("at").and_then(Value::as_array).into_iter().flatten() {
                at.push(match spot {
                    Value::String(n) => {
                        let q = placed.solved.points.get(n).ok_or_else(|| format!("`at`: `{name}` has no point `{n}`"))?;
                        library::Anchor { at: [q.x, q.y], point: Some(n.clone()) }
                    }
                    q => library::Anchor { at: serde_json::from_value(q.clone()).map_err(|e| format!("`at`: {e}"))?, point: None },
                });
            }
            let names: BTreeMap<String, String> = match p.get("names") {
                Some(n) => serde_json::from_value(n.clone()).map_err(|e| format!("`names`: {e}"))?,
                None => BTreeMap::new(),
            };
            let parameters = parameter_values(s.graph.parameters());
            let lookup = |n: &str| parameters.get(n).and_then(|v| v.as_ref().ok()).copied().ok_or_else(|| ExprError::Unknown(n.to_string()));
            let empty = Map::new();
            let given = p.get("values").and_then(Value::as_object).unwrap_or(&empty);
            let spots: Vec<[f64; 2]> = at.iter().map(|a| a.at).collect();
            let values = library::answers(&element, p.get("preset").and_then(Value::as_str), given, &spots, &lookup)?;
            let feature = serde_json::to_value(&s.graph.nodes()[s.find(name)? as usize].op).map_err(|e| e.to_string())?;
            let mut patch = SketchPatch::new(&feature);
            let construction = p.get("construction").and_then(Value::as_bool).unwrap_or(false);
            let renamed = library::place(&mut patch, &element, &values, &at, &names, construction, &lookup)?;
            // Sockets completed now, by the names given; the rest stay open, listed in the reply.
            let mut open = Vec::new();
            let given = p.get("links").and_then(Value::as_object);
            let sockets = library::links(&element, &values, &renamed)?;
            if let Some(l) = given.into_iter().flat_map(|g| g.keys()).find(|l| !sockets.iter().any(|k| k.label == **l)) {
                return Err(format!("`links`: the element has no socket `{l}`"));
            }
            for link in sockets {
                match given.and_then(|g| g.get(&link.label)).and_then(Value::as_str) {
                    Some(target) if library::fits(placed, &link.kind, target) => patch.constrain(link.to(target)),
                    Some(target) => return Err(format!("`links`: `{}` takes a {}, `{target}` is not one in `{name}`", link.label, link.kind)),
                    None => open.push(link.shown),
                }
            }
            let mut params = patch.done().unwrap_or_default();
            params.insert("name".into(), json!(name));
            let mut reply = s.edit_sketch(name, &params)?;
            reply["names"] = json!(renamed);
            reply["open_links"] = json!(open);
            Ok(reply)
        },
    },
    Command {
        name: "rename_in_sketch",
        description: "Rename a point, line, arc, circle, reference dimension, group or projection of sketch feature `name` from `from` \
                      to `to`, everywhere it is used: the sketch's constraints and groups, a sheet's `start`. Names of faces made from the sketch \
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
        description: "The feature history with each feature's status: volume, element counts and hash, or why it failed; \
                      `modified`: whether the model differs from its file as last opened or saved.",
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
    /// Applies `edit_sketch` patch `p` to sketch feature `name`.
    fn edit_sketch(&mut self, name: &str, p: &Map<String, Value>) -> Result<Value, String> {
        let ix = self.find(name)?;
        let Ok(Value::Object(mut op)) = serde_json::to_value(&self.graph.nodes()[ix as usize].op) else { return Err("unreadable feature".into()) };
        if op.get("op") != Some(&json!("sketch")) {
            return Err(format!("`{name}` is not a sketch"));
        }
        let also = patch_sketch(&mut op, p)?;
        let op: Op = serde_json::from_value(Value::Object(op)).map_err(|e| e.to_string())?;
        self.graph.set_op(ix, op).map_err(|e| e.to_string())?;
        let mut reply = self.rebuild_recording(ix);
        reply["also_removed"] = json!(also);
        Ok(reply)
    }

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
        self.saved = g.clone();
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
        let mut defined: Vec<String> = ["points", "lines", "arcs", "circles", "measures", "groups", "patterns"]
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
                                // A pattern's totals: copies (punches), the profiles they make, the
                                // area they open, and of a region filled its share.
                                let patterns: Map<String, Value> = s
                                    .patterns
                                    .iter()
                                    .map(|p| {
                                        let n = p.at.len() as f64;
                                        let mut t = json!({"copies": p.at.len(), "profiles": p.at.len() * p.loops, "open_area": p.area * n});
                                        if let Some(r) = p.region.filter(|r| *r > 0.0) {
                                            t["region_area"] = json!(r);
                                            t["open_share"] = json!(p.area * n / r);
                                        }
                                        (p.name.clone(), t)
                                    })
                                    .collect();
                                let mut sketch = json!({"dof": s.solved.dof, "free": free, "points": points, "measures": s.solved.measures, "frame": frame});
                                if !patterns.is_empty() {
                                    sketch["patterns"] = Value::Object(patterns);
                                }
                                json!({"sketch": sketch, "hash": hash})
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
        json!({"file": self.path.as_ref().map(|p| p.display().to_string()), "modified": self.graph != self.saved, "parameters": parameters,
               "features": features})
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
            for key in ["points", "lines", "arcs", "circles", "measures", "groups", "patterns"] {
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
                // Numbers (or parameters' names), and a pattern's: not elements.
                if !["type", "value", "degrees", "diameter", "id", "step", "count", "clearance", "skip", "shift"].contains(&k.as_str()) {
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
    let mut gone: Vec<String> = ["points", "lines", "arcs", "circles", "measures", "groups", "patterns"].iter().flat_map(|k| removed_in(k)).collect();
    let projections = removed_in("project");
    gone.extend(projections.iter().flat_map(|n| [n.clone(), format!("{n}.start"), format!("{n}.end"), format!("{n}.center")]));
    merge(op, "project", p.get("project"))?;
    let sketch = op.get_mut("sketch").and_then(Value::as_object_mut).ok_or("the sketch is missing")?;
    // Constraints removed by name first: those of an element removed with them go with it below,
    // and are not looked for again.
    // Constraints compare as parsed: field order and number spelling do not matter.
    let parse = |v: &Value| serde_json::from_value::<datum_sketch::Constraint>(v.clone()).map_err(|e| format!("constraint {v}: {e}"));
    let constraints = sketch.entry("constraints").or_insert_with(|| json!([])).as_array_mut().ok_or("`constraints` is not a list")?;
    for gone in p.get("remove_constraints").and_then(Value::as_array).into_iter().flatten() {
        let gone = parse(gone)?;
        let k = constraints.iter().position(|c| parse(c).is_ok_and(|c| c == gone)).ok_or_else(|| format!("no constraint {gone:?}"))?;
        constraints.remove(k);
    }
    for key in ["points", "lines", "arcs", "circles", "measures", "groups", "patterns"] {
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
    // Groups lose what was removed; one left empty goes.
    if let Some(groups) = sketch.get_mut("groups").and_then(Value::as_object_mut) {
        for members in groups.values_mut().filter_map(Value::as_array_mut) {
            members.retain(|n| !n.as_str().is_some_and(|n| gone.iter().any(|g| g == n)));
        }
        let empty: Vec<String> = groups.iter().filter(|(_, m)| m.as_array().is_some_and(Vec::is_empty)).map(|(n, _)| n.clone()).collect();
        for n in empty {
            groups.remove(&n);
            also.push(format!("groups: {n}"));
            gone.push(n);
        }
        if groups.is_empty() {
            sketch.remove("groups");
        }
    }
    // Patterns of what is gone: their cell or their region.
    if let Some(m) = sketch.get_mut("patterns").and_then(Value::as_object_mut) {
        let doomed: Vec<String> = m.iter().filter(|(_, v)| refers(v, &gone)).map(|(n, _)| n.clone()).collect();
        for n in doomed {
            m.remove(&n);
            also.push(format!("patterns: {n}"));
        }
        if m.is_empty() {
            sketch.remove("patterns");
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
    for added in p.get("add_constraints").and_then(Value::as_array).into_iter().flatten() {
        parse(added)?;
        constraints.push(added.clone());
    }
    Ok(also)
}
