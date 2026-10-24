//! The engineer's settings (D132): one `settings.json`, as in VS Code; they never change a model.
//! Beside the program if it is there (a portable copy takes its settings along), else in the
//! user's folder: `%APPDATA%\Datum` on Windows, `$XDG_CONFIG_HOME/datum` or `~/.config/datum`
//! elsewhere. Section `keys` holds only what differs from the defaults, so a new version brings
//! its new keys:
//!
//! ```json
//! {"keys": [
//!     {"key": "Ctrl+Shift+Z", "action": "edit.redo"},
//!     {"key": "Ctrl+Y", "action": null}
//! ]}
//! ```
//!
//! An action gets the key given instead of its own, and the key leaves an action it would run
//! with too; `null` takes the key from whoever has it. What cannot be applied is said, never
//! dropped silently.

use std::path::PathBuf;

use egui::{Key, KeyboardShortcut, Modifiers};
use serde_json::Value as Json;

use crate::actions::{ACTIONS, Keys};

const FILE: &str = "settings.json";

/// The settings file: beside the program if there, else in the user's folder (there or not).
pub fn path() -> Option<PathBuf> {
    let beside = std::env::current_exe().ok().and_then(|e| e.parent().map(|d| d.join(FILE)));
    if let Some(p) = beside.filter(|p| p.exists()) {
        return Some(p);
    }
    let env = |name| std::env::var_os(name).filter(|v| !v.is_empty()).map(PathBuf::from);
    let folder = if cfg!(windows) {
        env("APPDATA").map(|d| d.join("Datum"))
    } else {
        env("XDG_CONFIG_HOME").or_else(|| env("HOME").map(|h| h.join(".config"))).map(|d| d.join("datum"))
    };
    folder.map(|d| d.join(FILE))
}

/// The keys in force and what could not be applied, read from `path` (none there: the defaults).
pub fn load(path: Option<&std::path::Path>) -> (Keys, Vec<String>) {
    let Some(text) = path.and_then(|p| std::fs::read_to_string(p).ok()) else { return (Keys::default(), Vec::new()) };
    parse(&text)
}

fn parse(text: &str) -> (Keys, Vec<String>) {
    let mut keys = Keys::default();
    let settings: Json = match serde_json::from_str(text) {
        Ok(s) => s,
        Err(e) => return (keys, vec![format!("not read, the defaults stand: {e}")]),
    };
    let Some(sections) = settings.as_object() else { return (keys, vec!["not read, the defaults stand: not a JSON object".into()]) };
    let mut problems: Vec<String> = sections.keys().filter(|k| *k != "keys").map(|k| format!("`{k}`: no such section")).collect();
    let entries = match sections.get("keys") {
        None => &Vec::new(),
        Some(Json::Array(e)) => e,
        Some(_) => return (keys, vec!["`keys`: not a list, the default keys stand".into()]),
    };
    for (k, entry) in entries.iter().enumerate() {
        if let Err(why) = apply(&mut keys, entry, &mut problems) {
            problems.push(format!("keys[{k}] {entry}: {why}"));
        }
    }
    (keys, problems)
}

/// Applies one entry of `keys`; an action losing its key to it is said in `problems`.
fn apply(keys: &mut Keys, entry: &Json, problems: &mut Vec<String>) -> Result<(), String> {
    let text = entry["key"].as_str().ok_or("no `key`")?;
    let key =
        shortcut(text).ok_or_else(|| format!("`{text}` is not a key; keys are written as the menu shows them, e.g. Ctrl+Shift+S, F2, Delete"))?;
    match &entry["action"] {
        Json::Null if entry.get("action").is_some() => {
            let had = keys.with(&key);
            if had.is_empty() {
                return Err("no action has this key".into());
            }
            for act in had {
                keys.set(act, None);
            }
        }
        Json::String(name) => {
            let a = ACTIONS.iter().find(|a| a.name == name).ok_or_else(|| format!("no action `{name}`; the menu shows each one's name"))?;
            for lost in keys.set(a.act, Some(key)) {
                problems.push(format!("{text}: `{}` has it no more, `{name}` has", lost.action().name));
            }
        }
        _ => return Err("`action` is an action's name, or null to take the key off".into()),
    }
    Ok(())
}

/// A key as written: modifiers and a key joined by `+`, in any case ("ctrl+shift+s", "F2").
fn shortcut(text: &str) -> Option<KeyboardShortcut> {
    let mut parts: Vec<&str> = text.split('+').map(str::trim).collect();
    let key = parts.pop()?;
    let mut modifiers = Modifiers::NONE;
    for m in parts {
        modifiers = modifiers.plus(match m.to_lowercase().as_str() {
            "ctrl" | "cmd" | "command" => Modifiers::COMMAND,
            "shift" => Modifiers::SHIFT,
            "alt" => Modifiers::ALT,
            _ => return None,
        });
    }
    let capitalised = |s: &str| s.chars().take(1).flat_map(char::to_uppercase).chain(s.chars().skip(1)).collect::<String>();
    let key = Key::from_name(key).or_else(|| Key::from_name(&capitalised(&key.to_lowercase()))).or_else(|| Key::from_name(&key.to_uppercase()))?;
    Some(KeyboardShortcut::new(modifiers, key))
}

#[cfg(test)]
mod tests {
    use serde_json::json;

    use super::*;
    use crate::actions::Act;

    fn ctrl(key: Key) -> KeyboardShortcut {
        KeyboardShortcut::new(Modifiers::COMMAND, key)
    }

    #[test]
    fn keys_as_written() {
        assert_eq!(shortcut("Ctrl+Shift+S"), Some(KeyboardShortcut::new(Modifiers::COMMAND | Modifiers::SHIFT, Key::S)));
        assert_eq!(shortcut("ctrl + y"), Some(ctrl(Key::Y)));
        assert_eq!(shortcut("delete"), Some(KeyboardShortcut::new(Modifiers::NONE, Key::Delete)));
        assert_eq!(shortcut("f2"), Some(KeyboardShortcut::new(Modifiers::NONE, Key::F2)));
        assert_eq!(shortcut("Hyper+S"), None);
        assert_eq!(shortcut("Ctrl+"), None);
    }

    /// Only differences: the rest keep their defaults; a key moved, one taken off.
    #[test]
    fn differences_from_the_defaults() {
        let text = json!({"keys": [{"key": "Ctrl+Shift+Z", "action": "edit.redo"}, {"key": "Ctrl+W", "action": null}]}).to_string();
        let (keys, problems) = parse(&text);
        assert!(problems.is_empty(), "{problems:?}");
        assert_eq!(keys.get(Act::Redo), Some(&KeyboardShortcut::new(Modifiers::COMMAND | Modifiers::SHIFT, Key::Z)));
        assert_eq!(keys.get(Act::Close), None);
        assert_eq!(keys.get(Act::Undo), Some(&ctrl(Key::Z)));
    }

    /// What cannot be applied is said, and the rest applies; a key taken from another action
    /// is said too.
    #[test]
    fn problems_said() {
        let text = json!({
            "keys": [
                {"key": "Ctrl+S", "action": "sketch.line"},
                {"key": "Ctrl+Q", "action": "file.quit"},
                {"key": "Hyper+Q", "action": "file.exit"},
                {"key": "F9", "action": null},
                {"action": "file.exit"},
                {"key": "Ctrl+Q", "action": "file.exit"}
            ],
            "layuot": {}
        })
        .to_string();
        let (keys, problems) = parse(&text);
        assert_eq!(keys.get(Act::Line), Some(&ctrl(Key::S)));
        assert_eq!(keys.get(Act::Save), None);
        assert_eq!(keys.get(Act::Exit), Some(&ctrl(Key::Q)));
        let said = problems.join("\n");
        for part in [
            "`layuot`: no such section",
            "`file.save` has it no more",
            "no action `file.quit`",
            "`Hyper+Q` is not a key",
            "no action has this key",
            "no `key`",
        ] {
            assert!(said.contains(part), "{part}:\n{said}");
        }
        assert_eq!(problems.len(), 6, "{said}");
    }

    #[test]
    fn a_broken_file_keeps_the_defaults() {
        let (keys, problems) = parse("{\"keys\": [");
        assert_eq!(keys.get(Act::Save), Some(&ctrl(Key::S)));
        assert!(problems[0].starts_with("not read"), "{problems:?}");
    }
}
