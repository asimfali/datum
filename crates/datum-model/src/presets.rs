//! Presets of sketch metadata (D277): named sets of properties the window offers to fill in,
//! from one JSON section in three places. The shipped file first, then the engineer's
//! `settings.json`, then the active project's `datum-project.json`; the same ID replaces the
//! whole entry, `null` hides it. A chosen preset writes its values into the model, never a
//! reference: changing the presets later changes no marked sketch.

use std::collections::BTreeMap;
use std::path::Path;

use serde_json::{Map, Value, json};

use crate::workspace::PROJECT_FILE;

/// The presets every model has, in the program: the two the stamping starts with.
const BUILTIN: &str = include_str!("../data/metadata_presets.json");

/// The presets of the engineer's settings file and of project `project` (its folder; none: no
/// project), merged over the shipped ones: `presets` by ID in order, each with its source
/// (`built-in`, `settings`, `project`), and what could not be read.
pub fn metadata_presets(settings: Option<&Path>, project: Option<&Path>) -> Value {
    let mut presets: BTreeMap<String, (Value, &'static str)> = BTreeMap::new();
    let mut diagnostics = Vec::new();
    match serde_json::from_str::<Value>(BUILTIN) {
        Ok(v) => layer(&mut presets, &mut diagnostics, &v, "built-in", None),
        Err(e) => diagnostics.push(format!("built-in presets: not read: {e}")),
    }
    if let Some(path) = settings {
        read(&mut presets, &mut diagnostics, path, "settings", Some("metadata_presets"));
    }
    if let Some(root) = project {
        read(&mut presets, &mut diagnostics, &root.join(PROJECT_FILE), "project", Some("metadata_presets"));
    }
    let list: Vec<Value> = presets
        .into_iter()
        .map(|(id, (entry, source))| {
            let mut v = entry;
            v["id"] = json!(id);
            v["source"] = json!(source);
            v
        })
        .collect();
    json!({"presets": list, "diagnostics": diagnostics})
}

/// Reads `path` (a file that may be absent) and merges its `metadata_presets` (the whole file's,
/// when `section` is none: the shipped presets file).
fn read(
    presets: &mut BTreeMap<String, (Value, &'static str)>,
    diagnostics: &mut Vec<String>,
    path: &Path,
    source: &'static str,
    section: Option<&str>,
) {
    let text = match std::fs::read_to_string(path) {
        Ok(t) => t,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return,
        Err(e) => {
            diagnostics.push(format!("{}: {e}", path.display()));
            return;
        }
    };
    let json: Value = match serde_json::from_str(&text) {
        Ok(v) => v,
        Err(e) => {
            diagnostics.push(format!("{}: not read: {e}", path.display()));
            return;
        }
    };
    let section = match section {
        None => json,
        Some(key) => {
            if !json.is_object() {
                diagnostics.push(format!("{}: not a JSON object", path.display()));
                return;
            }
            match json.get(key) {
                None => return,
                Some(v) => v.clone(),
            }
        }
    };
    layer(presets, diagnostics, &section, source, Some(path));
}

/// Merges one layer: `null` hides an ID, an object replaces the entry whole.
fn layer(
    presets: &mut BTreeMap<String, (Value, &'static str)>,
    diagnostics: &mut Vec<String>,
    value: &Value,
    source: &'static str,
    path: Option<&Path>,
) {
    let at = |what: &str| path.map_or_else(|| format!("{source}: {what}"), |p| format!("{}: {what}", p.display()));
    let Some(entries) = value.as_object() else {
        diagnostics.push(at("`metadata_presets` is an object of presets"));
        return;
    };
    for (id, entry) in entries {
        if id.is_empty() {
            diagnostics.push(at("a preset without an ID"));
            continue;
        }
        if entry.is_null() {
            presets.remove(id);
            continue;
        }
        let Some(entry) = entry.as_object() else {
            diagnostics.push(at(&format!("`{id}`: an object with `set`")));
            continue;
        };
        let set = match entry.get("set") {
            Some(Value::Object(m)) if !m.is_empty() && !m.keys().any(String::is_empty) => m.clone(),
            Some(Value::Object(m)) if m.is_empty() => {
                diagnostics.push(at(&format!("`{id}`: `set` is empty")));
                continue;
            }
            Some(Value::Object(_)) => {
                diagnostics.push(at(&format!("`{id}`: `set` has an empty property name")));
                continue;
            }
            _ => {
                diagnostics.push(at(&format!("`{id}`: `set` is an object of properties")));
                continue;
            }
        };
        for key in entry.keys().filter(|k| !["set", "label"].contains(&k.as_str())) {
            diagnostics.push(at(&format!("`{id}`: `{key}` is no preset field: `set` and `label`")));
        }
        let mut preset = Map::new();
        preset.insert("set".into(), Value::Object(set));
        match entry.get("label") {
            Some(label @ (Value::String(_) | Value::Object(_))) if label.as_object().is_none_or(|m| m.values().all(Value::is_string)) => {
                preset.insert("label".into(), label.clone());
            }
            Some(label) => diagnostics.push(at(&format!("`{id}`: `label` is text or a dictionary of languages, not {label}"))),
            None => {}
        }
        presets.insert(id.clone(), (Value::Object(preset), source));
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use std::path::PathBuf;

    use super::*;

    fn folder(tag: &str) -> PathBuf {
        let p = std::env::temp_dir().join(format!("datum-presets-{tag}"));
        let _ = std::fs::remove_dir_all(&p);
        std::fs::create_dir_all(&p).unwrap();
        p
    }

    fn list(v: &Value) -> Vec<(String, Value)> {
        v["presets"].as_array().unwrap().iter().map(|p| (p["id"].as_str().unwrap().to_string(), p["set"].clone())).collect()
    }

    /// The shipped settings and the project's replace entries whole; `null` hides one; an ID
    /// nobody names stays; the order is by ID.
    #[test]
    fn three_layers_by_id() {
        let dir = folder("layers");
        let project = dir.join("Проект");
        std::fs::create_dir_all(&project).unwrap();
        let settings = dir.join("settings.json");
        std::fs::write(
            &settings,
            r#"{"metadata_presets": {
            "cut": {"set": {"stamp.role": "cut", "company": 1}},
            "bend": {"label": {"ru": "Гиб"}, "set": {"stamp.role": "transition", "goes": null}}
        }}"#,
        )
        .unwrap();
        std::fs::write(
            project.join(PROJECT_FILE),
            r#"{"metadata_presets": {"transition": null, "cut": {"label": "Резка", "set": {"stamp.role": "cut"}}}}"#,
        )
        .unwrap();
        let v = metadata_presets(Some(&settings), Some(&project));
        assert_eq!(v["diagnostics"], json!([]), "{v}");
        assert_eq!(
            list(&v),
            [("bend".to_string(), json!({"stamp.role": "transition", "goes": null})), ("cut".to_string(), json!({"stamp.role": "cut"})),]
        );
        assert_eq!(v["presets"][1]["label"], "Резка");
        assert_eq!(v["presets"][1]["source"], "project");
        assert_eq!(v["presets"][0]["source"], "settings");
        // Without a project the shipped two stand, the settings' over one of them.
        let v = metadata_presets(Some(&settings), None);
        assert_eq!(
            list(&v),
            [
                ("bend".to_string(), json!({"stamp.role": "transition", "goes": null})),
                ("cut".to_string(), json!({"stamp.role": "cut", "company": 1})),
                ("transition".to_string(), json!({"stamp.role": "transition"})),
            ]
        );
        assert_eq!(v["presets"][1]["source"], "settings");
        std::fs::remove_dir_all(&dir).unwrap();
    }

    /// A broken section or entry is said and the rest stands; no file is no error.
    #[test]
    fn broken_parts_are_said() {
        let dir = folder("broken");
        let settings = dir.join("settings.json");
        std::fs::write(
            &settings,
            r#"{"metadata_presets": {
                "bad": {"set": {}},
                "empty": {"set": {"": 1}},
                "label": {"label": 3, "set": {"x": 1}},
                "keep": {"set": {"x": 1}},
                "typo": {"set": {"x": 2}, "lable": "X"}
            }}"#,
        )
        .unwrap();
        let v = metadata_presets(Some(&settings), Some(&dir.join("nowhere")));
        let ids: Vec<&str> = v["presets"].as_array().unwrap().iter().map(|p| p["id"].as_str().unwrap()).collect();
        assert_eq!(ids, ["cut", "keep", "label", "transition", "typo"]);
        let said = v["diagnostics"].as_array().unwrap().iter().filter_map(Value::as_str).collect::<Vec<_>>().join("\n");
        assert!(said.contains("`bad`: `set` is empty"), "{said}");
        assert!(said.contains("`empty`: `set` has an empty property name"), "{said}");
        assert!(said.contains("`label`: `label` is text or a dictionary"), "{said}");
        assert!(said.contains("`typo`: `lable` is no preset field"), "{said}");
        std::fs::write(&settings, "{not json").unwrap();
        let v = metadata_presets(Some(&settings), None);
        assert_eq!(list(&v).len(), 2, "the shipped presets stand");
        assert!(v["diagnostics"][0].as_str().unwrap().contains("not read"), "{v}");
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
