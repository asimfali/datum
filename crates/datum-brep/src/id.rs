//! Origin-derived topology IDs (ARCHITECTURE §4.2).

use std::fmt;

/// Stable identifier of a topological element, derived from its origin. Never a counter.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct TopoId(pub u128);

impl TopoId {
    /// Starts deriving an ID for an element of kind `tag` (e.g. `"prism.side"`).
    pub fn derive(tag: &str) -> Origin {
        Origin(blake3::Hasher::new()).tag(tag)
    }
}

impl fmt::Debug for TopoId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:032x}", self.0)
    }
}

/// Builder of a [`TopoId`]: every part is typed and length-prefixed, so different origins never
/// produce the same byte stream.
pub struct Origin(blake3::Hasher);

impl Origin {
    pub fn tag(mut self, s: &str) -> Self {
        self.0.update(b"s");
        self.0.update(&(s.len() as u64).to_le_bytes());
        self.0.update(s.as_bytes());
        self
    }

    pub fn id(mut self, id: TopoId) -> Self {
        self.0.update(b"i");
        self.0.update(&id.0.to_le_bytes());
        self
    }

    pub fn index(mut self, k: u64) -> Self {
        self.0.update(b"n");
        self.0.update(&k.to_le_bytes());
        self
    }

    pub fn finish(self) -> TopoId {
        let mut b = [0u8; 16];
        b.copy_from_slice(&self.0.finalize().as_bytes()[..16]);
        TopoId(u128::from_le_bytes(b))
    }
}
