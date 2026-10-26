//! Datum viewer (M4): a window to look at a model and edit its parameters live, and headless
//! pictures of it for the CLI, MCP and tests.

mod actions;
mod app;
pub mod camera;
pub mod capture;
mod constrain;
mod cube;
mod dims;
mod dock;
mod draw;
mod flat;
mod icons;
pub mod log;
mod origin;
mod pattern;
pub mod render;
pub mod scene;
mod select;
mod settings;
mod sidebar;
mod sketch;
mod templates;
mod update;

pub use app::run;
