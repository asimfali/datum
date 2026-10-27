//! The log of what went wrong, for testers to send: panics with their backtrace and refused
//! commands, appended to `datum.log` beside the program, or in the temporary folder where the
//! program's own is not writable. The window's start and its GPU are logged too: a log that
//! stops at "opening the window" means the process died in the GPU driver.
//!
//! Beside it, the journal of actions (D110), while the engineer records: every command the
//! window runs, a JSON line as the agent's `execute` takes it (`{"command": …, "params": …}`,
//! and `error` if refused), to the console and to `datum-actions.jsonl`, begun anew each time
//! recording starts. Off by default: models may be confidential. The engineer shows a sequence
//! of operations; the agent reads it, talks it over, repeats it.

use std::io::Write;
use std::path::PathBuf;
use std::sync::OnceLock;
use std::sync::atomic::{AtomicBool, Ordering};

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

/// The journal of actions.
pub fn actions_path() -> &'static PathBuf {
    static PATH: OnceLock<PathBuf> = OnceLock::new();
    PATH.get_or_init(|| beside("datum-actions.jsonl"))
}

static RECORDING: AtomicBool = AtomicBool::new(false);

/// Whether actions are journaled.
pub fn recording() -> bool {
    RECORDING.load(Ordering::Relaxed)
}

/// Starts recording, with the journal emptied, or stops it.
pub fn record(on: bool) {
    if on {
        let _ = std::fs::File::create(actions_path());
    }
    RECORDING.store(on, Ordering::Relaxed);
}

/// Journals command `command` run with `params`, while recording: refused, with `error`.
pub fn action(command: &str, params: &serde_json::Value, error: Option<&str>) {
    journal(actions_path(), recording(), command, params, error);
}

/// [`action`] into the journal at `path`, if `on`. Apart from the switch, so a test of it leaves
/// the window's alone: on, every side bar shows a line more, and a click in a window test
/// running beside it would land a line off.
fn journal(path: &std::path::Path, on: bool, command: &str, params: &serde_json::Value, error: Option<&str>) {
    if !on {
        return;
    }
    let mut line = serde_json::json!({"command": command, "params": params});
    if let Some(e) = error {
        line["error"] = e.into();
    }
    // A window without a console has no stdout: nothing to say there, and no harm.
    let _ = writeln!(std::io::stdout().lock(), "{line}");
    if let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
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

    /// A journaled action reads back as the agent's `execute` arguments; nothing is journaled
    /// unless recording. In a file of its own, the window's switch left alone.
    #[test]
    fn actions_are_execute_calls() {
        let path = std::env::temp_dir().join(format!("datum-actions-test-{}.jsonl", std::process::id()));
        let _ = std::fs::remove_file(&path);
        let params = json!({"name": "s", "lines": {"l1": {"from": "p1", "to": "p2"}}});
        super::journal(&path, false, "add", &json!({"name": "secret"}), None);
        super::journal(&path, true, "edit_sketch", &params, None);
        super::journal(&path, true, "remove", &json!({"name": "x"}), Some("no feature `x`"));
        super::journal(&path, false, "add", &json!({"name": "secret"}), None);
        let text = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert!(!text.contains("secret"), "{text}");
        let lines: Vec<Value> = text.lines().map(|l| serde_json::from_str(l).unwrap()).collect();
        assert_eq!(
            lines,
            [json!({"command": "edit_sketch", "params": params}), json!({"command": "remove", "params": {"name": "x"}, "error": "no feature `x`"})]
        );
    }
}
