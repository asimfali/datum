//! Datum B-rep: immutable half-edge bodies with origin-derived topology IDs.
//!
//! Every body is built by [`build`], which puts it in canonical order, so equal bodies have equal
//! layout and equal [`Body::hash`] regardless of input order or thread count.

use std::fmt;

mod body;
mod build;
mod curve;
mod id;
mod prism;
mod profile;
mod terrain;
mod tess;
mod validate;

pub use body::{Body, Edge, Face, HalfEdge, Ix, Loop, MassProperties, Plane, Vertex};
pub use build::{CurveSpec, FaceSpec, build, build_with_curves};
pub use curve::{Arc, Curve, Cylinder, Development, Rolled, Surface};
pub use id::{Origin, TopoId};
pub use prism::{Region, cuboid, extrude, prism};
pub use profile::{ProfileArc, ProfileLoop};
pub use terrain::{Grid, terrain};
pub use tess::{FaceMesh, Mesh, Triangles, edge_points, face_mesh, tessellate, triangulate_face};

/// Results of a parallel stage in input order, or the first error in input order: the outcome,
/// error message included, does not depend on the thread count (C4).
pub fn in_order<T: Send, E: Send>(results: impl rayon::iter::IndexedParallelIterator<Item = Result<T, E>>) -> Result<Vec<T>, E> {
    results.collect::<Vec<_>>().into_iter().collect()
}

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
    /// A referenced element is gone after a rebuild (§4.2, P5): the reference names it.
    LostReference(String),
    /// A sketch is malformed or its constraints cannot all hold.
    Sketch(String),
}

impl fmt::Display for BrepError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let (kind, msg) = match self {
            Self::Degenerate(m) => ("degenerate", m),
            Self::NotClosed(m) => ("not closed", m),
            Self::NonManifold(m) => ("non-manifold", m),
            Self::Invalid(m) => ("invalid body", m),
            Self::Unsupported(m) => ("unsupported", m),
            Self::LostReference(m) => ("lost reference", m),
            Self::Sketch(m) => ("sketch", m),
        };
        write!(f, "{kind}: {msg}")
    }
}

impl std::error::Error for BrepError {}

#[cfg(test)]
mod tests;
