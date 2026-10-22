//! The log of what went wrong, for testers to send: panics with their backtrace and refused
//! commands, appended to `datum.log` beside the program, or in the temporary folder where the
//! program's own is not writable. The window's start and its GPU are logged too: a log that
//! stops at "opening the window" means the process died in the GPU driver.

use std::io::Write;
use std::path::PathBuf;
use std::sync::OnceLock;

/// The log file.
pub fn path() -> &'static PathBuf {
    static PATH: OnceLock<PathBuf> = OnceLock::new();
    PATH.get_or_init(|| {
        let beside = std::env::current_exe().ok().and_then(|e| e.parent().map(|d| d.join("datum.log")));
        let writable = |p: &PathBuf| std::fs::OpenOptions::new().create(true).append(true).open(p).is_ok();
        beside.filter(writable).unwrap_or_else(|| std::env::temp_dir().join("datum.log"))
    })
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
