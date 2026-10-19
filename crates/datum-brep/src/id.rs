//! Origin-derived topology IDs (ARCHITECTURE §4.2).

use std::collections::HashMap;
use std::fmt;
use std::hash::{BuildHasherDefault, Hasher};

use smallvec::SmallVec;

/// Stable identifier of a topological element, derived from its origin. Never a counter.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct TopoId(pub u128);

impl TopoId {
    /// Starts deriving an ID for an element of kind `tag` (e.g. `"prism.side"`).
    pub fn derive(tag: &str) -> Origin {
        Origin(SmallVec::new()).tag(tag)
    }
}

impl fmt::Debug for TopoId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:032x}", self.0)
    }
}

/// A map keyed by topology IDs. The IDs are already uniform hashes, so the hasher takes their low
/// 64 bits as they are: fast and the same on every run (no `DefaultHasher`). For lookups only.
pub(crate) type IdMap<V> = HashMap<TopoId, V, BuildHasherDefault<IdHasher>>;

#[derive(Default)]
pub(crate) struct IdHasher(u64);

impl Hasher for IdHasher {
    fn write(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.0 = self.0.rotate_left(8) ^ u64::from(b);
        }
    }

    fn write_u128(&mut self, v: u128) {
        self.0 = v as u64;
    }

    fn finish(&self) -> u64 {
        self.0
    }
}

/// Builder of a [`TopoId`]: every part is typed and length-prefixed, so different origins never
/// produce the same byte stream. The bytes are hashed in one call when finished, which gives the
/// same hash as streaming them and is several times cheaper for these short inputs.
pub struct Origin(SmallVec<[u8; 128]>);

impl Origin {
    pub fn tag(mut self, s: &str) -> Self {
        self.0.push(b's');
        self.0.extend_from_slice(&(s.len() as u64).to_le_bytes());
        self.0.extend_from_slice(s.as_bytes());
        self
    }

    pub fn id(mut self, id: TopoId) -> Self {
        self.0.push(b'i');
        self.0.extend_from_slice(&id.0.to_le_bytes());
        self
    }

    pub fn index(mut self, k: u64) -> Self {
        self.0.push(b'n');
        self.0.extend_from_slice(&k.to_le_bytes());
        self
    }

    pub fn finish(self) -> TopoId {
        let mut b = [0u8; 16];
        b.copy_from_slice(&blake3::hash(&self.0).as_bytes()[..16]);
        TopoId(u128::from_le_bytes(b))
    }
}
