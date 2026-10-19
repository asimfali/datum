//! Sketches placed in space, and extrusions of their profiles (M3, D53).

use std::collections::BTreeMap;

use datum_brep::{Body, BrepError, ProfileLoop, Region, TopoId, extrude};
use datum_math::{Rigid, Sign, Vec2, Vec3, orient2d};
use datum_sketch::{Line, Sketch, Solved, solve};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

/// An explicit sketch plane: its origin, the direction of the sketch's X axis (projected into
/// the plane) and the normal, along which extrusions go.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SketchPlane {
    pub origin: Vec3,
    pub x_axis: Vec3,
    pub normal: Vec3,
}

/// A solved sketch in its place: the value of a sketch feature.
#[derive(Clone, Debug, PartialEq)]
pub struct Placed {
    /// Origin and the X, Y, Z axes of the sketch plane, Z its normal.
    pub frame: [Vec3; 4],
    pub solved: Solved,
    pub lines: BTreeMap<String, Line>,
    pub hash: [u8; 32],
}

fn unit(v: Vec3) -> Option<Vec3> {
    let len = v.dot(v).sqrt();
    (len > 0.0 && len.is_finite()).then(|| v * (1.0 / len))
}

/// The frame with normal `z` and X axis along `x` projected into the plane.
fn frame(origin: Vec3, x: Vec3, z: Vec3) -> Result<[Vec3; 4], BrepError> {
    let bad = || BrepError::Degenerate("the sketch plane needs a normal and an X axis not along it".into());
    let z = unit(z).ok_or_else(bad)?;
    let x = unit(x - z * x.dot(z)).ok_or_else(bad)?;
    Ok([origin, x, z.cross(x), z])
}

/// Solves `sketch` and places it: on the XY plane, on `plane`, or on the face with ID `face` of
/// `host`. On a face, the origin is the world origin projected onto the face's plane and X the
/// world axis least along the normal, projected: on a top face sketch coordinates are world X and
/// Y. The frame follows the face's plane only. A face gone after a rebuild is a lost reference.
pub fn place(sketch: &Sketch, plane: Option<&SketchPlane>, face: Option<(&Body, &str)>) -> Result<Placed, BrepError> {
    let frame = match (plane, face) {
        (Some(_), Some(_)) => return Err(BrepError::Sketch("give a plane or a face, not both".into())),
        (Some(p), None) => frame(p.origin, p.x_axis, p.normal)?,
        (None, Some((body, id))) => {
            let id = u128::from_str_radix(id, 16).map(TopoId).map_err(|_| BrepError::Sketch(format!("`{id}` is not a face ID")))?;
            let f = body
                .faces()
                .binary_search_by_key(&id, |f| f.id)
                .map_err(|_| BrepError::LostReference(format!("face {id:?} is not in the input body")))?;
            let plane = body.faces()[f].plane;
            let n = unit(plane.normal).ok_or_else(|| BrepError::Degenerate("the face has no normal".into()))?;
            let axes = [Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0)];
            let x = axes.into_iter().min_by(|a, b| a.dot(n).abs().total_cmp(&b.dot(n).abs())).unwrap_or(axes[0]);
            frame(n * n.dot(plane.origin), x, n)?
        }
        (None, None) => [Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0)],
    };
    let solved = solve(sketch).map_err(|e| BrepError::Sketch(e.to_string()))?;
    let mut h = blake3::Hasher::new();
    let mut bytes = |b: &[u8]| {
        h.update(&(b.len() as u64).to_le_bytes());
        h.update(b);
    };
    for c in frame.iter().flat_map(|v| v.to_array()) {
        bytes(&c.to_bits().to_le_bytes());
    }
    for (name, p) in &solved.points {
        bytes(name.as_bytes());
        bytes(&[p.x.to_bits().to_le_bytes(), p.y.to_bits().to_le_bytes()].concat());
    }
    for (name, l) in &sketch.lines {
        for s in [name, &l.from, &l.to] {
            bytes(s.as_bytes());
        }
        bytes(&[u8::from(l.construction)]);
    }
    Ok(Placed { frame, solved, lines: sketch.lines.clone(), hash: *h.finalize().as_bytes() })
}

/// The closed profiles of `s` swept from `z0` to `z1` along its normal. Profile lines (not
/// construction) must form closed loops that neither branch nor touch; a loop inside another is
/// a hole, one inside a hole an island. Side faces are named by their lines, vertices by their
/// points (§4.2): renaming nothing keeps every ID through edits of the sketch.
pub fn extrude_sketch(feature: TopoId, s: &Placed, z0: f64, z1: f64) -> Result<Body, BrepError> {
    let loops = profile_loops(s)?;
    let at = |p: &str| s.solved.points[p];
    let pts: Vec<Vec<Vec2>> = loops.iter().map(|l| l.iter().map(|(p, _)| at(p)).collect()).collect();
    let inside = |a: usize, b: usize| a != b && encloses(&pts[a], pts[b][0]);
    let depth: Vec<usize> = (0..loops.len()).map(|b| (0..loops.len()).filter(|&a| inside(a, b)).count()).collect();
    let parent = |b: usize| (0..loops.len()).find(|&a| inside(a, b) && depth[a] + 1 == depth[b]);
    let ring = |i: usize| ProfileLoop {
        points: pts[i].clone(),
        vertices: loops[i].iter().map(|(p, _)| [0, 1].map(|layer| TopoId::derive("extrude.v").id(feature).index(layer).tag(p).finish())).collect(),
        sides: loops[i].iter().map(|(_, l)| TopoId::derive("extrude.side").id(feature).tag(l).finish()).collect(),
    };
    let regions: Vec<Region> = (0..loops.len())
        .filter(|&i| depth[i].is_multiple_of(2))
        .map(|i| {
            let first = loops[i].iter().map(|(p, _)| *p).min().unwrap_or_default();
            Region {
                outer: ring(i),
                holes: (0..loops.len()).filter(|&j| !depth[j].is_multiple_of(2) && parent(j) == Some(i)).map(ring).collect(),
                caps: ["extrude.cap_start", "extrude.cap_end"].map(|t| TopoId::derive(t).id(feature).tag(first).finish()),
            }
        })
        .collect();
    let [o, x, y, z] = s.frame;
    Ok(extrude(&regions, z0, z1)?.transformed(&Rigid::frame(o, x, y, z)))
}

/// Profile loops as `(point, line from it to the next point)`, each starting at its lowest point.
fn profile_loops(s: &Placed) -> Result<Vec<Vec<(&str, &str)>>, BrepError> {
    let mut at: BTreeMap<&str, Vec<(&str, &str)>> = BTreeMap::new();
    for (name, l) in s.lines.iter().filter(|(_, l)| !l.construction) {
        at.entry(&l.from).or_default().push((name, &l.to));
        at.entry(&l.to).or_default().push((name, &l.from));
    }
    if at.is_empty() {
        return Err(BrepError::Sketch("the sketch has no profile lines".into()));
    }
    if let Some((p, _)) = at.iter().find(|(_, ls)| ls.len() != 2) {
        return Err(BrepError::Sketch(format!("the profile is open or branches at point `{p}`")));
    }
    let mut used: Vec<&str> = Vec::new();
    let mut loops = Vec::new();
    for (&start, lines) in &at {
        if lines.iter().all(|(l, _)| used.contains(l)) {
            continue;
        }
        let mut lp = Vec::new();
        let (mut p, mut via) = (start, lines[0]);
        loop {
            used.push(via.0);
            lp.push((p, via.0));
            p = via.1;
            if p == start {
                break;
            }
            match at[p].iter().find(|(l, _)| !used.contains(l)) {
                Some(&next) => via = next,
                None => return Err(BrepError::Sketch(format!("the profile does not close at point `{p}`"))),
            }
        }
        loops.push(lp);
    }
    Ok(loops)
}

/// Whether `p` lies inside the polygon `poly` (winding number, exact).
fn encloses(poly: &[Vec2], p: Vec2) -> bool {
    let mut winding = 0i32;
    for k in 0..poly.len() {
        let (a, b) = (poly[k], poly[(k + 1) % poly.len()]);
        if a.y <= p.y && b.y > p.y && orient2d(a, b, p) == Sign::Positive {
            winding += 1;
        } else if a.y > p.y && b.y <= p.y && orient2d(a, b, p) == Sign::Negative {
            winding -= 1;
        }
    }
    winding != 0
}
