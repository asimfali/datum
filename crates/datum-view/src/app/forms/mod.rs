//! The forms of the model's operations: each its own module, state, window and tests (D255).

mod loft;
mod metadata;
mod rib;

pub(in crate::app) use loft::LoftForm;
pub(in crate::app) use metadata::MetadataForm;
#[cfg(test)]
pub(in crate::app) use metadata::Preset;
pub(in crate::app) use rib::RibForm;
