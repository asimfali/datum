//! Datum B-rep: immutable half-edge bodies with origin-derived topology IDs.
//!
//! Every body is built by [`build`], which puts it in canonical order, so equal bodies have equal
//! layout and equal [`Body::hash`] regardless of input order or thread count.

use std::fmt;

mod body;
mod build;
mod id;
mod prism;
mod tess;
mod validate;

pub use body::{Body, Edge, Face, HalfEdge, Ix, Loop, MassProperties, Plane, Vertex};
pub use build::{FaceSpec, build};
pub use id::{Origin, TopoId};
pub use prism::{cuboid, prism};
pub use tess::{Mesh, tessellate, triangulate_face};

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BrepError {
    /// Input in a degenerate configuration the kernel refuses to approximate.
    Degenerate(String),
    /// An edge has no opposite half-edge.
    NotClosed(String),
    NonManifold(String),
    /// A broken structural invariant: a bug in the operation that built the body.
    Invalid(String),
    /// Valid input the kernel does not handle yet.
    Unsupported(String),
}

impl fmt::Display for BrepError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let (kind, msg) = match self {
            Self::Degenerate(m) => ("degenerate", m),
            Self::NotClosed(m) => ("not closed", m),
            Self::NonManifold(m) => ("non-manifold", m),
            Self::Invalid(m) => ("invalid body", m),
            Self::Unsupported(m) => ("unsupported", m),
        };
        write!(f, "{kind}: {msg}")
    }
}

impl std::error::Error for BrepError {}

#[cfg(test)]
mod tests;
