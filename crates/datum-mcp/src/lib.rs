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
query_topology. Every edit returns which features were recomputed and which failed, with the cause.";

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
    json!({"tools": tools})
}

#[cfg(test)]
mod tests;
