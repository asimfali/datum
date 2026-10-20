//! Grid pattern: copies of a body in rows and columns, as one body (PAINS P4).

use datum_brep::{Body, BrepError, Curve, CurveSpec, FaceSpec, Surface, TopoId, Vertex, build_with_curves, in_order};
use datum_math::Vec3;
use rayon::prelude::*;

use crate::boolean::disjoint;
use crate::bounds::{Aabb, overlapping_pairs};

/// `count[0] × count[1]` copies of `source`, copy `(i, j)` moved by `i·step[0] + j·step[1]`, as one
/// body with a shell per copy. Copies must not touch: boxes first, then the exact test of the
/// Boolean operation for copies whose boxes meet. Element IDs derive from the feature, the copy's
/// `(i, j)` and the source ID (§4.2): adding rows or columns keeps the IDs of existing copies.
pub fn pattern_grid(feature: TopoId, source: &Body, count: [u32; 2], step: [Vec3; 2]) -> Result<Body, BrepError> {
    if count.contains(&0) {
        return Err(BrepError::Degenerate("a pattern needs at least one copy".into()));
    }
    if step.iter().flat_map(|s| s.to_array()).any(|c| !c.is_finite()) {
        return Err(BrepError::Degenerate("non-finite pattern step".into()));
    }
    let copies: Vec<[u32; 2]> = (0..count[1]).flat_map(|j| (0..count[0]).map(move |i| [i, j])).collect();
    let offset = |[i, j]: [u32; 2]| step[0] * f64::from(i) + step[1] * f64::from(j);
    // Rounding is monotone, so the moved box is exactly the box of the moved points.
    let (lo, hi) = source.bounds();
    let boxes: Vec<Aabb> = copies.iter().map(|&c| (lo + offset(c), hi + offset(c))).collect();
    let copy = |c: [u32; 2]| {
        let o = offset(c);
        let id = |id: TopoId| TopoId::derive("pattern.copy").id(feature).index(c[0].into()).index(c[1].into()).id(id).finish();
        let vertices: Vec<Vertex> = source.vertices().iter().map(|v| Vertex { id: id(v.id), point: v.point + o }).collect();
        let faces: Vec<FaceSpec> = (0..source.faces().len() as u32)
            .map(|f| {
                let mut s = source.face_spec(f);
                s.id = id(s.id);
                match &mut s.surface {
                    Surface::Plane(p) => p.origin = p.origin + o,
                    Surface::Cylinder(c) => c.origin = c.origin + o,
                    Surface::Wall(w) => w.dev.origin = w.dev.origin + o,
                }
                s.loops.iter_mut().flatten().for_each(|v| *v = id(*v));
                s
            })
            .collect();
        let arcs: Vec<CurveSpec> = source
            .curve_specs()
            .into_iter()
            .map(|a| {
                let curve = match a.curve {
                    Curve::Arc(arc) => Curve::Arc(datum_brep::Arc { center: arc.center + o, ..arc }),
                    Curve::Rolled { mut piece, radius } => {
                        piece.dev.origin = piece.dev.origin + o;
                        Curve::Rolled { piece, radius }
                    }
                };
                CurveSpec { from: id(a.from), to: id(a.to), curve }
            })
            .collect();
        (vertices, faces, arcs)
    };
    let touching = |(a, b): (usize, usize)| -> Result<(), BrepError> {
        let body = |c: [u32; 2]| {
            let (v, f, a) = copy(c);
            build_with_curves(v, f, a)
        };
        if disjoint(&body(copies[a])?, &body(copies[b])?)? {
            Ok(())
        } else {
            Err(BrepError::Degenerate(format!("pattern copies {:?} and {:?} touch", copies[a], copies[b])))
        }
    };
    in_order(overlapping_pairs(&boxes, |i| i).into_par_iter().map(touching))?;
    let parts = copies.par_iter().map(|&c| copy(c)).collect::<Vec<_>>();
    let mut vertices = Vec::with_capacity(copies.len() * source.vertices().len());
    let mut faces = Vec::with_capacity(copies.len() * source.faces().len());
    let mut arcs = Vec::new();
    for (v, f, a) in parts {
        vertices.extend(v);
        faces.extend(f);
        arcs.extend(a);
    }
    build_with_curves(vertices, faces, arcs)
}
