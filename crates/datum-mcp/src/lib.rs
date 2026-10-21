//! Datum MCP server (ARCHITECTURE §6.3): JSON-RPC 2.0 over stdio, newline-delimited, on top of
//! the command registry of [`Session`]. Tools are thin: `execute` runs any command, and the
//! query tools are named shortcuts whose schemas come from the registry.

use std::io::{BufRead, Write};

use datum_model::Session;
use serde_json::{Value, json};

pub const PROTOCOL_VERSION: &str = "2025-06-18";
const SUPPORTED: &[&str] = &[PROTOCOL_VERSION, "2025-03-26", "2024-11-05"];

const PARSE_ERROR: i64 = -32700;
const INVALID_REQUEST: i64 = -32600;
const METHOD_NOT_FOUND: i64 = -32601;
const INVALID_PARAMS: i64 = -32602;

const INSTRUCTIONS: &str = "Datum is a parametric solid-modelling kernel. A model is a history of features (JSON objects with \
`name`, `inputs` and `op` plus parameters); bodies are rebuilt from it deterministically and incrementally. Start with \
list_commands, build with execute (add, edit, remove, open, save), check with inspect_model, mass_properties and \
query_topology; edges to project into a sketch come from execute `query_edges`. Look at the result with render. Every edit returns which features were recomputed and which \
failed, with the cause.";

/// Query tools: (tool name, command it runs, description).
const SHORTCUTS: &[(&str, &str, &str)] = &[
    ("inspect_model", "inspect", "The feature history with each feature's status: volume, element counts and hash, or why it failed."),
    ("mass_properties", "mass_properties", "Volume, surface area, centre of mass and bounding box of a feature's body."),
    (
        "query_topology",
        "query_faces",
        "Faces of a feature's body with stable IDs, unit normals, plane offsets and areas; filter by `normal` within `tolerance_deg`.",
    ),
];

#[derive(Default)]
pub struct Server {
    session: Session,
}

impl Server {
    /// Serves until `input` closes. Nothing but protocol messages is written to `output`.
    pub fn serve(&mut self, input: impl BufRead, mut output: impl Write) -> std::io::Result<()> {
        for line in input.lines() {
            if let Some(reply) = self.handle_line(&line?) {
                writeln!(output, "{reply}")?;
                output.flush()?;
            }
        }
        Ok(())
    }

    pub fn handle_line(&mut self, line: &str) -> Option<String> {
        if line.trim().is_empty() {
            return None;
        }
        let reply = match serde_json::from_str::<Value>(line) {
            Ok(msg) => self.handle(&msg),
            Err(e) => Some(error(Value::Null, PARSE_ERROR, format!("parse error: {e}"))),
        };
        reply.map(|r| r.to_string())
    }

    fn handle(&mut self, msg: &Value) -> Option<Value> {
        let Some(method) = msg.get("method").and_then(Value::as_str) else {
            // A response from the client, or garbage: answer garbage only.
            return (msg.get("result").is_none() && msg.get("error").is_none()).then(|| error(Value::Null, INVALID_REQUEST, "not a request"));
        };
        // Notifications (no id) get no reply.
        let id = msg.get("id")?.clone();
        let params = msg.get("params").cloned().unwrap_or(Value::Null);
        Some(match self.request(method, &params) {
            Ok(result) => json!({"jsonrpc": "2.0", "id": id, "result": result}),
            Err((code, m)) => error(id, code, m),
        })
    }

    fn request(&mut self, method: &str, params: &Value) -> Result<Value, (i64, String)> {
        match method {
            "initialize" => {
                let asked = params.get("protocolVersion").and_then(Value::as_str).unwrap_or(PROTOCOL_VERSION);
                let version = if SUPPORTED.contains(&asked) { asked } else { PROTOCOL_VERSION };
                Ok(json!({
                    "protocolVersion": version,
                    "capabilities": {"tools": {"listChanged": false}},
                    "serverInfo": {"name": "datum", "version": env!("CARGO_PKG_VERSION")},
                    "instructions": INSTRUCTIONS
                }))
            }
            "ping" => Ok(json!({})),
            "tools/list" => Ok(tool_definitions()),
            "tools/call" => {
                let name = params.get("name").and_then(Value::as_str).ok_or((INVALID_PARAMS, "missing tool `name`".to_string()))?;
                let args = params.get("arguments").cloned().unwrap_or(Value::Null);
                let result = match name {
                    "execute" => {
                        let cmd = args.get("command").and_then(Value::as_str).ok_or((INVALID_PARAMS, "missing `command`".to_string()))?;
                        self.session.execute(cmd, args.get("params").unwrap_or(&Value::Null))
                    }
                    "list_commands" => Ok(Session::commands()),
                    "render" => {
                        return Ok(match self.render(&args) {
                            Ok(png) => json!({"content": [{"type": "image", "data": base64(&png), "mimeType": "image/png"}]}),
                            Err(e) => json!({"content": [{"type": "text", "text": e}], "isError": true}),
                        });
                    }
                    _ => match SHORTCUTS.iter().find(|(tool, ..)| *tool == name) {
                        Some((_, cmd, _)) => self.session.execute(cmd, &args),
                        None => return Err((INVALID_PARAMS, format!("unknown tool `{name}`"))),
                    },
                };
                Ok(match result {
                    Ok(v) => json!({"content": [{"type": "text", "text": serde_json::to_string_pretty(&v).unwrap_or_default()}]}),
                    Err(e) => json!({"content": [{"type": "text", "text": e}], "isError": true}),
                })
            }
            _ => Err((METHOD_NOT_FOUND, format!("method `{method}` not found"))),
        }
    }
}

impl Server {
    /// A PNG of the named features, or of what the model ends in.
    fn render(&self, args: &Value) -> Result<Vec<u8>, String> {
        let s = &self.session;
        let values = match args.get("features").and_then(Value::as_array) {
            Some(names) => names.iter().map(|n| s.value(n.as_str().ok_or("`features` lists feature names")?)).collect::<Result<Vec<_>, _>>()?,
            None => s.evaluation().final_values(s.graph()),
        };
        let size = |key: &str, default: u64| match args.get(key) {
            None => Ok(default as u32),
            Some(v) => {
                v.as_u64().filter(|n| (16..=MAX_SIZE).contains(n)).map(|n| n as u32).ok_or(format!("`{key}` is a whole number from 16 to {MAX_SIZE}"))
            }
        };
        let view = args.get("view").map_or(Ok("iso"), |v| v.as_str().ok_or("`view` must be a string"))?;
        datum_view::capture::snapshot(&values, view, size("width", 800)?, size("height", 600)?)
    }
}

/// The largest side of a rendered picture, in pixels.
const MAX_SIZE: u64 = 2048;

/// Standard base64 with padding (RFC 4648 §4), for image content.
fn base64(bytes: &[u8]) -> String {
    const ALPHABET: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::with_capacity(bytes.len().div_ceil(3) * 4);
    for chunk in bytes.chunks(3) {
        let b = [chunk[0], chunk.get(1).copied().unwrap_or(0), chunk.get(2).copied().unwrap_or(0)];
        let n = u32::from(b[0]) << 16 | u32::from(b[1]) << 8 | u32::from(b[2]);
        for k in 0..4 {
            out.push(if k <= chunk.len() { char::from(ALPHABET[(n >> (18 - 6 * k)) as usize & 63]) } else { '=' });
        }
    }
    out
}

fn error(id: Value, code: i64, message: impl Into<String>) -> Value {
    json!({"jsonrpc": "2.0", "id": id, "error": {"code": code, "message": message.into()}})
}

fn tool_definitions() -> Value {
    let commands = Session::commands();
    let schema_of = |cmd: &str| commands["commands"].as_array().and_then(|l| l.iter().find(|c| c["name"] == cmd)).map(|c| c["params"].clone());
    let mut tools = vec![
        json!({"name": "execute", "description": "Run any Datum command with JSON parameters (see list_commands): add, edit, remove, new, open, save, and the queries.",
            "inputSchema": {"type": "object", "properties": {"command": {"type": "string"}, "params": {"type": "object"}}, "required": ["command"]}}),
        json!({"name": "list_commands", "description": "Every command with its description and parameter schema (the `add` schema lists all operations).",
            "inputSchema": {"type": "object", "properties": {}}}),
    ];
    tools.extend(SHORTCUTS.iter().map(|(tool, cmd, description)| json!({"name": tool, "description": description, "inputSchema": schema_of(cmd)})));
    tools.push(json!({"name": "render",
        "description": "A PNG picture of the model: shaded faces, sharp edges, sketches in orange. Shows what the model ends in, or only `features`.",
        "inputSchema": {"type": "object", "properties": {
            "features": {"type": "array", "items": {"type": "string"}, "description": "Feature names to show; default: features nothing else uses."},
            "view": {"type": "string", "enum": datum_view::camera::VIEWS, "description": "Direction of view, fitted to the shown features; front looks along +Y. Default: iso."},
            "width": {"type": "integer", "minimum": 16, "maximum": MAX_SIZE, "description": "Pixels, default 800."},
            "height": {"type": "integer", "minimum": 16, "maximum": MAX_SIZE, "description": "Pixels, default 600."}}}}));
    json!({"tools": tools})
}

#[cfg(test)]
mod tests;
