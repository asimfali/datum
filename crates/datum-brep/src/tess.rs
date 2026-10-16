//! Triangulation of a body's faces for export and display.

use datum_math::{Sign, Vec2, orient2d};

use crate::{Body, BrepError, Ix};

/// Triangle mesh sharing the body's vertices: `positions[i]` is vertex `i` of the body.
#[derive(Clone, Debug, PartialEq)]
pub struct Mesh {
    pub positions: Vec<datum_math::Vec3>,
    /// Counterclockwise seen from outside.
    pub triangles: Vec<[Ix; 3]>,
}

/// Triangulates every face by ear clipping in the face's dominant projection, with exact
/// predicates. Faces with holes are not supported yet (they arrive with Booleans, M2).
pub fn tessellate(body: &Body) -> Result<Mesh, BrepError> {
    let mut triangles = Vec::with_capacity(body.half_edges().len());
    for (f, face) in body.faces().iter().enumerate() {
        if face.loop_count != 1 {
            return Err(BrepError::Unsupported(format!("face {:?} has holes", face.id)));
        }
        let lp = body.face_loops(f as Ix).start;
        let verts: Vec<Ix> = body.loop_half_edges(lp).map(|h| body.half_edges()[h as usize].origin).collect();
        // Dropping the dominant axis keeps the loop's turning sense when the other two axes stay
        // in cyclic order; a negative dominant component flips it.
        let axis = face.plane.normal.dominant_axis();
        let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
        let flipped = face.plane.normal.to_array()[axis] < 0.0;
        let pts: Vec<Vec2> = verts
            .iter()
            .map(|&i| {
                let p = body.point(i).to_array();
                Vec2::new(p[u], p[v])
            })
            .collect();
        let convex = if flipped { Sign::Negative } else { Sign::Positive };
        ear_clip(&pts, convex, |a, b, c| triangles.push([verts[a], verts[b], verts[c]]))
            .map_err(|()| BrepError::Degenerate(format!("face {:?} cannot be triangulated", face.id)))?;
    }
    Ok(Mesh { positions: body.vertices().iter().map(|v| v.point).collect(), triangles })
}

/// Ear clipping of a simple polygon whose convex corners have sign `convex`. Emits `n − 2`
/// triangles as index triples into `pts`. `Err` when no ear is found (a degenerate polygon).
fn ear_clip(pts: &[Vec2], convex: Sign, mut emit: impl FnMut(usize, usize, usize)) -> Result<(), ()> {
    let mut ring: Vec<usize> = (0..pts.len()).collect();
    let mut i = 0;
    let mut misses = 0;
    while ring.len() > 3 {
        let n = ring.len();
        let (a, b, c) = (ring[(i + n - 1) % n], ring[i % n], ring[(i + 1) % n]);
        if is_ear(pts, &ring, a, b, c, convex) {
            emit(a, b, c);
            ring.remove(i % n);
            misses = 0;
        } else {
            i += 1;
            misses += 1;
            if misses > n {
                return Err(());
            }
        }
    }
    emit(ring[0], ring[1], ring[2]);
    Ok(())
}

fn is_ear(pts: &[Vec2], ring: &[usize], a: usize, b: usize, c: usize, convex: Sign) -> bool {
    if orient2d(pts[a], pts[b], pts[c]) != convex {
        return false;
    }
    // No other ring vertex inside or on the candidate triangle.
    ring.iter().filter(|&&k| k != a && k != b && k != c).all(|&k| {
        let p = pts[k];
        [orient2d(pts[a], pts[b], p), orient2d(pts[b], pts[c], p), orient2d(pts[c], pts[a], p)].iter().any(|s| *s != convex && *s != Sign::Zero)
    })
}
