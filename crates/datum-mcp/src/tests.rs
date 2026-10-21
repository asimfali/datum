use serde_json::{Value, json};

use crate::{Server, base64};

fn call(s: &mut Server, id: u64, method: &str, params: Value) -> Value {
    let line = json!({"jsonrpc": "2.0", "id": id, "method": method, "params": params}).to_string();
    serde_json::from_str(&s.handle_line(&line).unwrap()).unwrap()
}

fn tool(s: &mut Server, name: &str, args: Value) -> (bool, Value) {
    let r = call(s, 9, "tools/call", json!({"name": name, "arguments": args}));
    let text = r["result"]["content"][0]["text"].as_str().unwrap().to_string();
    let is_error = r["result"]["isError"] == json!(true);
    (is_error, serde_json::from_str(&text).unwrap_or(Value::String(text)))
}

#[test]
fn lifecycle_and_tools() {
    let mut s = Server::default();
    let init = call(&mut s, 1, "initialize", json!({"protocolVersion": "2025-03-26"}));
    assert_eq!(init["result"]["protocolVersion"], "2025-03-26");
    assert_eq!(s.handle_line(r#"{"jsonrpc":"2.0","method":"notifications/initialized"}"#), None);
    let tools = call(&mut s, 2, "tools/list", json!({}));
    let names: Vec<&str> = tools["result"]["tools"].as_array().unwrap().iter().map(|t| t["name"].as_str().unwrap()).collect();
    assert_eq!(names, ["execute", "list_commands", "inspect_model", "mass_properties", "query_topology", "render"]);
    assert!(tools["result"]["tools"].as_array().unwrap().iter().all(|t| t["inputSchema"]["type"] == "object"));

    let add = json!({"name": "plate", "op": "prism", "polygon": [[0, 0], [10, 0], [10, 5], [0, 5]], "z0": 0, "z1": 2});
    let (err, r) = tool(&mut s, "execute", json!({"command": "add", "params": add}));
    assert!(!err);
    assert_eq!(r["computed"], 1);
    let (_, m) = tool(&mut s, "mass_properties", json!({"name": "plate"}));
    assert_eq!(m["volume"], 100.0);
    let (_, q) = tool(&mut s, "query_topology", json!({"name": "plate", "normal": [0, 0, -1]}));
    assert_eq!(q["count"], 1);
}

#[test]
fn errors_are_tool_results_or_protocol_errors() {
    let mut s = Server::default();
    let (err, text) = tool(&mut s, "execute", json!({"command": "add", "params": {"name": "x", "op": "prism", "polygon": [], "z0": 0}}));
    assert!(err, "{text}");
    assert_eq!(call(&mut s, 3, "nope", json!({}))["error"]["code"], -32601);
    assert_eq!(call(&mut s, 4, "tools/call", json!({"name": "nope"}))["error"]["code"], -32602);
    let garbage: Value = serde_json::from_str(&s.handle_line("{not json").unwrap()).unwrap();
    assert_eq!(garbage["error"]["code"], -32700);
}

#[test]
fn base64_matches_rfc_4648() {
    for (raw, encoded) in
        [("", ""), ("f", "Zg=="), ("fo", "Zm8="), ("foo", "Zm9v"), ("foob", "Zm9vYg=="), ("fooba", "Zm9vYmE="), ("foobar", "Zm9vYmFy")]
    {
        assert_eq!(base64(raw.as_bytes()), encoded);
    }
}

/// `render` answers with a PNG image, or with the reason in text. Skipped where wgpu finds no
/// adapter at all (not even a software one).
#[test]
fn render_returns_a_png() {
    let mut s = Server::default();
    let add = json!({"name": "plate", "op": "prism", "polygon": [[0, 0], [10, 0], [10, 5], [0, 5]], "z0": 0, "z1": 2});
    tool(&mut s, "execute", json!({"command": "add", "params": add}));
    let r = call(&mut s, 5, "tools/call", json!({"name": "render", "arguments": {"view": "top", "width": 64, "height": 48}}));
    let content = &r["result"]["content"][0];
    if r["result"]["isError"] == json!(true) {
        let text = content["text"].as_str().unwrap();
        assert!(text.contains("no GPU adapter"), "{text}");
        eprintln!("skipping: {text}");
        return;
    }
    assert_eq!(content["type"], "image");
    assert_eq!(content["mimeType"], "image/png");
    // The PNG signature, base64-encoded.
    assert!(content["data"].as_str().unwrap().starts_with("iVBORw0KGgo"));
    for bad in [json!({"view": "sideways"}), json!({"features": ["nope"]}), json!({"width": 0})] {
        let (err, text) = tool(&mut s, "render", bad);
        assert!(err, "{text}");
    }
}

/// What an agent does to dimension from a model edge: find the face, list its edges, project one.
#[test]
fn query_edges_and_project_one() {
    let mut s = Server::default();
    let add = json!({"name": "plate", "op": "prism", "polygon": [[0, 0], [10, 0], [10, 5], [0, 5]], "z0": 0, "z1": 2});
    tool(&mut s, "execute", json!({"command": "add", "params": add}));
    let (_, q) = tool(&mut s, "query_topology", json!({"name": "plate", "normal": [0, 0, 1]}));
    let top = q["faces"][0]["id"].as_str().unwrap().to_string();
    let (err, e) = tool(&mut s, "execute", json!({"command": "query_edges", "params": {"name": "plate", "face": top}}));
    assert!(!err && e["count"] == 4, "{e}");
    let bottom = e["edges"].as_array().unwrap().iter().find(|e| e["from"]["point"][1] == 0.0 && e["to"]["point"][1] == 0.0).unwrap();
    let sketch = json!({"name": "mark", "op": "sketch", "inputs": ["plate"], "face": top, "project": {"edge": {"id": bottom["id"]}},
        "sketch": {"points": {"m": [4, 1]}, "constraints": [{"type": "point_line", "point": "m", "line": "edge", "value": 2}],
                   "measures": {"along": {"type": "distance", "points": ["edge.start", "m"]}}}});
    let (err, r) = tool(&mut s, "execute", json!({"command": "add", "params": sketch}));
    assert!(!err && r["failed"] == json!([]), "{r}");
    let (_, m) = tool(&mut s, "inspect_model", json!({}));
    let mark = &m["features"][1]["status"]["sketch"];
    assert_eq!(mark["points"]["m"][1], 2.0, "{mark}");
    assert!(mark["measures"]["along"].as_f64().unwrap() > 0.0, "{mark}");
    // The session wrote down where the edge was when the sketch was added.
    let at = &m["features"][1]["project"]["edge"]["at"];
    assert_eq!(at.as_array().map(Vec::len), Some(2), "{}", m["features"][1]);
}

fn execute(s: &mut Server, command: &str, params: Value) -> Value {
    let (err, r) = tool(s, "execute", json!({"command": command, "params": params}));
    assert!(!err, "{command}: {r}");
    r
}

fn feature<'a>(m: &'a Value, name: &str) -> &'a Value {
    m["features"].as_array().unwrap().iter().find(|f| f["name"] == name).unwrap()
}

/// A rectangle, 10 × 4, the lower left corner fixed: the outline of a plate.
fn rectangle() -> Value {
    json!({"name": "outline", "op": "sketch", "sketch": {
        "points": {"a": [0, 0], "b": [10, 0], "c": [10, 4], "d": [0, 4]},
        "lines": {"bottom": {"from": "a", "to": "b"}, "right": {"from": "b", "to": "c"}, "top": {"from": "c", "to": "d"}, "left": {"from": "d", "to": "a"}},
        "constraints": [{"type": "fix", "point": "a"}, {"type": "horizontal", "line": "bottom"}, {"type": "vertical", "line": "right"},
                        {"type": "horizontal", "line": "top"}, {"type": "vertical", "line": "left"}]}})
}

/// An agent changes a sketch piece by piece, and sees what is still free.
#[test]
fn edit_a_sketch_piece_by_piece() {
    let mut s = Server::default();
    execute(&mut s, "add", rectangle());
    let sketch = |s: &mut Server| tool(s, "inspect_model", json!({})).1["features"][0].clone();
    // Width and height are free: the right side slides along X, the top along Y, the far corner both.
    let free = &sketch(&mut s)["status"]["sketch"]["free"];
    assert_eq!((free["b"].clone(), free["c"].clone(), free["d"].clone()), (json!([1.0, 0.0]), json!("plane"), json!([0.0, 1.0])), "{free}");
    execute(
        &mut s,
        "edit_sketch",
        json!({"name": "outline",
        "add_constraints": [{"type": "horizontal_distance", "points": ["a", "b"], "value": 12}, {"value": 5, "points": ["a", "d"], "type": "vertical_distance"}],
        "measures": {"diagonal": {"type": "distance", "points": ["a", "c"]}}}),
    );
    let f = sketch(&mut s);
    assert_eq!(f["status"]["sketch"]["dof"], 0, "{f}");
    assert_eq!(f["status"]["sketch"]["measures"]["diagonal"], 13.0);
    // Constraints are removed by what they say, whatever the order of their fields.
    execute(
        &mut s,
        "edit_sketch",
        json!({"name": "outline", "remove_constraints": [{"type": "vertical_distance", "points": ["a", "d"], "value": 5.0}],
        "measures": {"diagonal": null}}),
    );
    let f = sketch(&mut s);
    assert_eq!(f["status"]["sketch"]["dof"], 1, "{f}");
    assert!(f["sketch"].get("measures").is_none(), "{f}");
    let (err, r) = tool(&mut s, "execute", json!({"command": "edit_sketch", "params": {"name": "outline", "points": {"z": null}}}));
    assert!(err && r.as_str().unwrap().contains("no `z`"), "{r}");
}

/// Renaming the line a side face is named after: the sketch on that face and its projected edge
/// are found again by their geometry, and nothing downstream breaks.
#[test]
fn rename_keeps_downstream_references() {
    let mut s = Server::default();
    execute(&mut s, "add", rectangle());
    execute(&mut s, "add", json!({"name": "plate", "op": "extrude", "inputs": ["outline"], "z0": 0, "z1": 3}));
    let (_, q) = tool(&mut s, "query_topology", json!({"name": "plate", "normal": [-1, 0, 0]}));
    let side = q["faces"][0]["id"].as_str().unwrap().to_string();
    let edges = execute(&mut s, "query_edges", json!({"name": "plate", "face": side.clone()}));
    let top_edge =
        edges["edges"].as_array().unwrap().iter().find(|e| e["from"]["point"][2] == 3.0 && e["to"]["point"][2] == 3.0).unwrap()["id"].clone();
    execute(
        &mut s,
        "add",
        json!({"name": "slot", "op": "sketch", "inputs": ["plate"], "face": side, "project": {"rim": {"id": top_edge}},
        "sketch": {"points": {"p": [1, 1]}, "constraints": [{"type": "point_line", "point": "p", "line": "rim", "value": 1}]}}),
    );
    let r = execute(&mut s, "rename_in_sketch", json!({"name": "outline", "from": "left", "to": "edge_side"}));
    assert_eq!((r["lost"].clone(), r["failed"].clone(), r["features_with_ids_found_again"].clone()), (json!([]), json!([]), json!(1)), "{r}");
    let m = tool(&mut s, "inspect_model", json!({})).1;
    let slot = feature(&m, "slot");
    assert_ne!(slot["face"], json!(side), "the side face is named after the line: its ID changed");
    assert!(slot["status"]["sketch"]["points"]["p"].is_array(), "{slot}");
    assert!(feature(&m, "outline")["sketch"]["lines"].get("edge_side").is_some());
    // Points too, and a projection with its points.
    execute(&mut s, "rename_in_sketch", json!({"name": "slot", "from": "rim", "to": "brim"}));
    let m = tool(&mut s, "inspect_model", json!({})).1;
    assert_eq!(feature(&m, "slot")["sketch"]["constraints"][0]["line"], "brim");
    assert!(feature(&m, "slot")["status"]["sketch"]["points"].get("brim.start").is_some());
    for bad in [
        json!({"name": "outline", "from": "a", "to": "b"}),
        json!({"name": "outline", "from": "x", "to": "y"}),
        json!({"name": "outline", "from": "a", "to": "p.q"}),
    ] {
        assert!(tool(&mut s, "execute", json!({"command": "rename_in_sketch", "params": bad})).0);
    }
}

/// A sheet starts from a named point of its path: the rename follows it there.
#[test]
fn rename_follows_into_a_sheet() {
    let mut s = Server::default();
    execute(
        &mut s,
        "add",
        json!({"name": "path", "op": "sketch", "sketch": {
        "points": {"a": [0, 0], "b": [20, 0], "c": [20, 15]},
        "lines": {"base": {"from": "a", "to": "b"}, "wall": {"from": "b", "to": "c"}}}}),
    );
    execute(
        &mut s,
        "add",
        json!({"name": "bent", "op": "sheet", "inputs": ["path"], "thickness": 1, "radius": 1, "side": "left", "z0": 0, "z1": 10, "start": "a"}),
    );
    let r = execute(&mut s, "rename_in_sketch", json!({"name": "path", "from": "a", "to": "origin"}));
    assert_eq!(r["failed"], json!([]), "{r}");
    let m = tool(&mut s, "inspect_model", json!({})).1;
    assert_eq!(feature(&m, "bent")["start"], "origin");
}
