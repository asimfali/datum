//! Structural invariants of a body (ARCHITECTURE §4.1). Run on every operation result in debug
//! builds.

use crate::{Body, BrepError, Ix};

fn invalid<T>(msg: String) -> Result<T, BrepError> {
    Err(BrepError::Invalid(msg))
}

impl Body {
    /// Checks canonical order, index consistency, closedness, manifold vertices, the
    /// Euler–Poincaré formula, outward orientation and hashes.
    pub fn validate(&self) -> Result<(), BrepError> {
        self.check_order()?;
        self.check_links()?;
        self.check_vertex_fans()?;
        self.check_euler()?;
        let volume = self.volume();
        if volume.is_nan() || volume <= 0.0 {
            return invalid(format!("volume {volume} is not positive: faces point inward or the body is empty"));
        }
        let mut b = self.clone();
        b.rehash();
        if b.hash != self.hash {
            return invalid("stored hashes do not match the content".into());
        }
        Ok(())
    }

    fn check_order(&self) -> Result<(), BrepError> {
        if !self.vertices.is_sorted_by(|a, b| a.id < b.id) {
            return invalid("vertices are not strictly sorted by ID".into());
        }
        if !self.edges.is_sorted_by(|a, b| a.id < b.id) {
            return invalid("edges are not strictly sorted by ID".into());
        }
        if !self.faces.is_sorted_by(|a, b| a.id < b.id) {
            return invalid("faces are not strictly sorted by ID".into());
        }
        Ok(())
    }

    fn check_links(&self) -> Result<(), BrepError> {
        let (nv, nh, ne) = (self.vertices.len() as Ix, self.half_edges.len() as Ix, self.edges.len() as Ix);
        let mut next_loop = 0;
        for (f, face) in self.faces.iter().enumerate() {
            if face.first_loop != next_loop || face.loop_count == 0 {
                return invalid(format!("face {:?}: loops are not contiguous", face.id));
            }
            if self.face_loops(f as Ix).any(|l| self.loops[l as usize].face != f as Ix) {
                return invalid(format!("face {:?}: a loop names another face", face.id));
            }
            next_loop += face.loop_count;
        }
        if next_loop as usize != self.loops.len() {
            return invalid("loops not owned by any face".into());
        }
        let mut next_he = 0;
        for (l, lp) in self.loops.iter().enumerate() {
            if lp.first != next_he || lp.len < 3 {
                return invalid(format!("loop {l}: half-edges are not contiguous or fewer than 3"));
            }
            if self.loop_half_edges(l as Ix).any(|h| self.half_edges[h as usize].loop_ix != l as Ix) {
                return invalid(format!("loop {l}: a half-edge names another loop"));
            }
            next_he += lp.len;
        }
        if next_he != nh {
            return invalid("half-edges not owned by any loop".into());
        }
        for (h, he) in self.half_edges.iter().enumerate() {
            let h = h as Ix;
            if he.origin >= nv || he.twin >= nh || he.edge >= ne || he.twin == h {
                return invalid(format!("half-edge {h}: index out of range or self-twin"));
            }
            let tw = &self.half_edges[he.twin as usize];
            if tw.twin != h || tw.edge != he.edge || tw.origin != self.half_edges[self.next(h) as usize].origin {
                return invalid(format!("half-edge {h}: twin does not run the other way along the same edge"));
            }
        }
        for (e, edge) in self.edges.iter().enumerate() {
            let Some(he) = self.half_edges.get(edge.half as usize) else { return invalid(format!("edge {:?}: half-edge out of range", edge.id)) };
            if he.edge != e as Ix || he.origin >= self.half_edges[he.twin as usize].origin {
                return invalid(format!("edge {:?}: wrong half-edge", edge.id));
            }
        }
        Ok(())
    }

    /// Every vertex is used, and its outgoing half-edges form one fan: the vertex is manifold.
    fn check_vertex_fans(&self) -> Result<(), BrepError> {
        let mut count = vec![0u32; self.vertices.len()];
        let mut first = vec![Ix::MAX; self.vertices.len()];
        for (h, he) in self.half_edges.iter().enumerate() {
            count[he.origin as usize] += 1;
            if first[he.origin as usize] == Ix::MAX {
                first[he.origin as usize] = h as Ix;
            }
        }
        for (v, &start) in first.iter().enumerate() {
            if start == Ix::MAX {
                return invalid(format!("vertex {:?} is not used", self.vertices[v].id));
            }
            let (mut h, mut steps) = (start, 0);
            loop {
                h = self.next(self.half_edges[h as usize].twin);
                steps += 1;
                if h == start || steps > count[v] {
                    break;
                }
            }
            if steps != count[v] {
                return Err(BrepError::NonManifold(format!("vertex {:?}: faces around it form more than one fan", self.vertices[v].id)));
            }
        }
        Ok(())
    }

    /// `V − E + 2F − L = 2(S − G)` with `S` shells and genus `G ≥ 0`.
    fn check_euler(&self) -> Result<(), BrepError> {
        let mut parent: Vec<Ix> = (0..self.faces.len() as Ix).collect();
        fn find(p: &mut [Ix], mut x: Ix) -> Ix {
            while p[x as usize] != x {
                p[x as usize] = p[p[x as usize] as usize];
                x = p[x as usize];
            }
            x
        }
        for he in &self.half_edges {
            let face = |h: Ix| self.loops[self.half_edges[h as usize].loop_ix as usize].face;
            let (a, b) = (find(&mut parent, face(he.twin)), find(&mut parent, self.loops[he.loop_ix as usize].face));
            parent[a.max(b) as usize] = a.min(b);
        }
        let shells = (0..self.faces.len() as Ix).filter(|&f| find(&mut parent, f) == f).count() as i64;
        let chi = self.vertices.len() as i64 - self.edges.len() as i64 + 2 * self.faces.len() as i64 - self.loops.len() as i64;
        if chi % 2 != 0 || chi > 2 * shells {
            return invalid(format!("Euler–Poincaré fails: V − E + 2F − L = {chi} with {shells} shell(s)"));
        }
        Ok(())
    }
}
