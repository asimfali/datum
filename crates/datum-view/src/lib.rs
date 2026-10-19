//! Datum viewer (M4): a window to look at a model and edit its parameters live, and headless
//! pictures of it for the CLI, MCP and tests.

mod app;
pub mod camera;
pub mod capture;
pub mod render;
pub mod scene;

pub use app::run;
