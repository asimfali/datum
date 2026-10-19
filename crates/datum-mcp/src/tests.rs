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
