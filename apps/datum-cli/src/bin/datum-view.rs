//! The Datum window without a console: what a double click starts on Windows. `datum` (the
//! command line, MCP) opens the same window with `datum view`.
#![windows_subsystem = "windows"]

use std::path::PathBuf;

fn main() {
    // A model given (dropped on the program, or opened with it) is shown.
    let model = std::env::args().nth(1).map(PathBuf::from);
    if let Err(e) = datum_view::run(model) {
        datum_view::log::fail(&format!("Datum could not open its window: {e}"));
    }
}
