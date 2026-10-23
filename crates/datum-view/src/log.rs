//! The log of what went wrong, for testers to send: panics with their backtrace and refused
//! commands, appended to `datum.log` beside the program, or in the temporary folder where the
//! program's own is not writable. The window's start and its GPU are logged too: a log that
//! stops at "opening the window" means the process died in the GPU driver.
//!
//! Beside it, the journal of actions (D110): every command the window runs, a JSON line as the
//! agent's `execute` takes it (`{"command": …, "params": …}`, and `error` if refused), to the
//! console and to `datum-actions.jsonl`, begun anew at each start. The engineer shows a sequence
//! of operations; the agent reads it, talks it over, repeats it.

use std::io::Write;
use std::path::PathBuf;
use std::sync::OnceLock;

/// File `name` beside the program, or in the temporary folder if that is not writable.
fn beside(name: &str) -> PathBuf {
    let beside = std::env::current_exe().ok().and_then(|e| e.parent().map(|d| d.join(name)));
    let writable = |p: &PathBuf| std::fs::OpenOptions::new().create(true).append(true).open(p).is_ok();
    beside.filter(writable).unwrap_or_else(|| std::env::temp_dir().join(name))
}

/// The log file.
pub fn path() -> &'static PathBuf {
    static PATH: OnceLock<PathBuf> = OnceLock::new();
    PATH.get_or_init(|| beside("datum.log"))
}

/// The journal of actions, emptied the first time it is asked for.
pub fn actions_path() -> &'static PathBuf {
    static PATH: OnceLock<PathBuf> = OnceLock::new();
    PATH.get_or_init(|| {
        let p = beside("datum-actions.jsonl");
        let _ = std::fs::File::create(&p);
        p
    })
}

/// Journals command `command` run with `params`: refused, with `error`.
pub fn action(command: &str, params: &serde_json::Value, error: Option<&str>) {
    let mut line = serde_json::json!({"command": command, "params": params});
    if let Some(e) = error {
        line["error"] = e.into();
    }
    // A window without a console has no stdout: nothing to say there, and no harm.
    let _ = writeln!(std::io::stdout().lock(), "{line}");
    if let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open(actions_path()) {
        let _ = writeln!(f, "{line}");
    }
}

/// Appends a line, with the time (seconds since 1970) and the version.
pub fn write(message: &str) {
    let secs = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_secs());
    if let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open(path()) {
        let _ = writeln!(f, "[{secs}] datum {}: {message}", env!("CARGO_PKG_VERSION"));
    }
}

/// Logs a failure and says so on the screen: a program without a console has no other way
/// (D97).
pub fn fail(message: &str) {
    write(message);
    rfd::MessageDialog::new()
        .set_level(rfd::MessageLevel::Error)
        .set_title("Datum")
        .set_description(format!("{message}\n\nThis is written to {} too: please send that file.", path().display()))
        .show();
}

/// Logs every panic, with where it happened, before the usual report.
pub fn install() {
    let previous = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        write(&format!("panic: {info}\n{}", std::backtrace::Backtrace::force_capture()));
        // The main thread's panic ends the program: said on the screen too.
        if std::thread::current().name() == Some("main") {
            fail(&format!("Datum stopped: {info}"));
        }
        previous(info);
    }));
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use serde_json::{Value, json};

    /// A journaled action reads back as the agent's `execute` arguments.
    #[test]
    fn actions_are_execute_calls() {
        let params = json!({"name": "s", "lines": {"l1": {"from": "p1", "to": "p2"}}});
        super::action("edit_sketch", &params, None);
        super::action("remove", &json!({"name": "x"}), Some("no feature `x`"));
        let text = std::fs::read_to_string(super::actions_path()).unwrap();
        let lines: Vec<Value> = text.lines().map(|l| serde_json::from_str(l).unwrap()).collect();
        assert!(lines.contains(&json!({"command": "edit_sketch", "params": params})), "{text}");
        assert!(lines.contains(&json!({"command": "remove", "params": {"name": "x"}, "error": "no feature `x`"})), "{text}");
    }
}
