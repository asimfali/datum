//! Cuts through a plate or a rolled sheet (M5b–c, D63): the closed profiles of a sketch on a
//! plane face cut perpendicular to the face through to the opposite one, or of a sketch in the
//! development of a cylinder face cut along the radius through the sheet (as a hole punched in
//! the blank before rolling); optionally repeated on a grid in the sketch's coordinates (punches
//! of a turret press, PAINS P4).
//!
//! Every decision is a 2D one in the sketch's coordinates, by exact predicates on segments and
//! arcs: a profile lies inside the entry face and inside the exit face, touches neither their
//! edges nor another profile, and no other face of the body crosses the cut. The result is then
//! built directly: the profiles become holes of both faces, their edges walls between them. The
//! coordinates of the body's vertices in the sketch's frame are rounded once; on a plane face
//! along the axes, the usual case, they are exact.
use datum_brep::{
    Arc, Body, BrepError, Curve, CurveSpec, Cylinder, FaceSpec, Ix, Plane, ProfileArc, ProfileLoop, Rolled, Surface, TopoId, Vertex,
    build_with_curves, in_order,
};
use datum_math::{Rigid, Vec2, Vec3};
use rayon::prelude::*;

use crate::sketch::{Developed, Placed, coaxial, opposite_cylinder, profile_loops};

/// Copies of a sketch's profiles: copy `(i, j)` moved by `i·step[0] + j·step[1]`.
#[derive(Clone, Copy, Debug)]
pub struct ProfileGrid {
    pub count: [u32; 2],
    pub step: [Vec2; 2],
}

impl ProfileGrid {
    pub const ONE: Self = Self { count: [1, 1], step: [Vec2::new(0.0, 0.0), Vec2::new(0.0, 0.0)] };
}

/// A 2D box `(min, max)`.
type Box2 = (Vec2, Vec2);

/// The profiles of `sketch`, which lies on a face of `target`, cut through `target`.
pub fn cut(feature: TopoId, target: &Body, sketch: &Placed, grid: ProfileGrid) -> Result<Body, BrepError> {
    let unsupported = |m: String| BrepError::Unsupported(format!("cut: {m}"));
    let id = sketch.face.ok_or_else(|| BrepError::Sketch("a cut takes a sketch placed on a face of its body".into()))?;
    let f = target.faces().binary_search_by_key(&id, |f| f.id).map_err(|_| BrepError::LostReference(format!("face {id:?} is not in the body")))?;
    let frame = match (target.faces()[f].surface, sketch.developed.as_deref().copied()) {
        (Surface::Plane(_), None) => {
            let [o, x, y, z] = sketch.frame;
            Frame::Plane { o, x, y, z }
        }
        (Surface::Cylinder(c), Some(d)) => Frame::Rolled { d, radius: c.radius, side: if c.convex { 1.0 } else { -1.0 } },
        _ => return Err(BrepError::Invalid("the sketch does not lie on its face".into())),
    };
    if grid.count.contains(&0) {
        return Err(BrepError::Degenerate("a cut pattern needs at least one copy".into()));
    }

    // The profiles, counterclockwise in the sketch, copied over the grid.
    let base: Vec<ProfileLoop> =
        profile_loops(feature, sketch)?.into_iter().map(|l| if l.ring.counterclockwise() { l.ring } else { l.ring.reversed() }).collect();
    let copies: Vec<[u32; 2]> = (0..grid.count[1]).flat_map(|j| (0..grid.count[0]).map(move |i| [i, j])).collect();
    let profiles: Vec<ProfileLoop> = copies
        .par_iter()
        .flat_map_iter(|&[i, j]| {
            let d = Vec2::new(
                grid.step[0].x * f64::from(i) + grid.step[1].x * f64::from(j),
                grid.step[0].y * f64::from(i) + grid.step[1].y * f64::from(j),
            );
            let id = move |t: TopoId| {
                if [i, j] == [0, 0] { t } else { TopoId::derive("cut.copy").id(feature).index(i.into()).index(j.into()).id(t).finish() }
            };
            base.iter().map(move |l| ProfileLoop {
                points: l.points.iter().map(|p| Vec2::new(p.x + d.x, p.y + d.y)).collect(),
                vertices: l.vertices.iter().map(|v| v.map(id)).collect(),
                sides: l.sides.iter().map(|&s| id(s)).collect(),
                arcs: l.arcs.iter().map(|a| a.map(|a| ProfileArc { center: Vec2::new(a.center.x + d.x, a.center.y + d.y), ..a })).collect(),
            })
        })
        .collect();
    let boxes: Vec<Box2> = profiles.iter().map(ProfileLoop::bounds).collect();

    // Profiles neither touch nor hold one another.
    in_order(within(&boxes).into_par_iter().map(|(a, b)| {
        if profiles[a].touches(&profiles[b]) {
            Err(BrepError::Degenerate("cut profiles touch".into()))
        } else if profiles[a].winding(profiles[b].points[0]) != 0 || profiles[b].winding(profiles[a].points[0]) != 0 {
            Err(unsupported("a profile inside another".into()))
        } else {
            Ok(())
        }
    }))?;

    // The exit: on a plane, the nearest face below facing exactly the other way that holds the
    // first profile; on a cylinder, the face across the sheet.
    let exit = match &frame {
        Frame::Plane { z, .. } => {
            let first = profiles[0].points[0];
            let faces = (0..target.faces().len() as Ix).filter_map(|g| {
                let Surface::Plane(p) = target.faces()[g as usize].surface else { return None };
                let opposite = g as usize != f && p.normal.cross(*z) == Vec3::ZERO && p.normal.dot(*z) < 0.0;
                let d = frame
                    .depth(target.point(target.half_edges()[target.loops()[target.faces()[g as usize].first_loop as usize].first as usize].origin));
                let holds = || outline(target, g, &frame).is_ok_and(|ls| ls.iter().map(|l| l.winding(first)).sum::<i32>() != 0);
                (opposite && d < 0.0 && holds()).then_some((g, d))
            });
            faces.max_by(|a, b| a.1.total_cmp(&b.1))
        }
        Frame::Rolled { radius, side, .. } => opposite_cylinder(target, f).map(|(g, c)| (g, (c.radius - radius) * side)),
    };
    let Some((g, bottom)) = exit else {
        return Err(unsupported("no face opposite the sketch's face holds the profiles: a cut goes through a plate".into()));
    };

    // Inside both faces, clear of their edges and of their holes.
    for (face, name) in [(f as Ix, "entry"), (g, "exit")] {
        let loops = outline(target, face, &frame)?;
        let loop_boxes: Vec<Box2> = loops.iter().map(ProfileLoop::bounds).collect();
        let pairs = between(&boxes, &loop_boxes);
        in_order(pairs.par_iter().map(|&(p, l)| {
            if profiles[p].touches(&loops[l]) {
                Err(unsupported(format!("a profile crosses an edge of the {name} face")))
            } else if profiles[p].winding(loops[l].points[0]) != 0 {
                Err(unsupported(format!("a hole of the {name} face lies inside a profile")))
            } else {
                Ok(())
            }
        }))?;
        let mut winding = vec![0i32; profiles.len()];
        for &(p, l) in &pairs {
            winding[p] += loops[l].winding(profiles[p].points[0]);
        }
        if winding.contains(&0) {
            return Err(unsupported(format!("a profile leaves the {name} face")));
        }
    }

    // No other face crosses the cut between the two.
    let other: Vec<Ix> = (0..target.faces().len() as Ix).filter(|&h| h as usize != f && h != g).collect();
    let crossing = match &frame {
        Frame::Plane { z, .. } => plane_neighbours(target, &other, &frame, *z, bottom)?,
        Frame::Rolled { d, radius, side } => {
            let radii = [*radius, radius + side * bottom];
            let Surface::Cylinder(entry) = &target.faces()[f].surface else {
                return Err(BrepError::Invalid("the sketch does not lie on its face".into()));
            };
            rolled_neighbours(target, &other, &frame, d, entry, [radii[0].min(radii[1]), radii[0].max(radii[1])], &boxes)?
        }
    };
    let crossing_boxes: Vec<Box2> = crossing.iter().map(|(l, _)| l.bounds()).collect();
    in_order(between(&boxes, &crossing_boxes).into_par_iter().map(|(p, l)| {
        let (lp, along) = (&crossing[l].0, crossing[l].1);
        let inside = profiles[p].winding(lp.points[0]) != 0 || (!along && lp.winding(profiles[p].points[0]) != 0);
        if inside || profiles[p].touches(lp) { Err(unsupported("another face of the body crosses the cut".into())) } else { Ok(()) }
    }))?;

    // The body: the profiles as holes of both faces, walls between them.
    let at = |p: Vec2, d: f64| frame.point(p, d);
    let mut vertices: Vec<Vertex> = target.vertices().to_vec();
    let mut curves = target.curve_specs();
    let mut faces: Vec<FaceSpec> = (0..target.faces().len() as Ix).filter(|&h| h as usize != f && h != g).map(|h| target.face_spec(h)).collect();
    let (mut top, mut low) = (target.face_spec(f as Ix), target.face_spec(g));
    for l in &profiles {
        let n = l.points.len();
        // Holes run clockwise seen from outside: against the profile on the entry face, along
        // it on the exit face, which faces the other way.
        top.loops.push(l.vertices.iter().rev().map(|v| v[0]).collect());
        low.loops.push(l.vertices.iter().map(|v| v[1]).collect());
        for k in 0..n {
            let (a, b) = (l.points[k], l.points[(k + 1) % n]);
            let (v, w) = (l.vertices[k], l.vertices[(k + 1) % n]);
            vertices.push(Vertex { id: v[0], point: at(a, 0.0) });
            vertices.push(Vertex { id: v[1], point: at(a, bottom) });
            let surface = match &frame {
                Frame::Plane { z, .. } => match l.arc(k) {
                    None => {
                        let d = at(b, 0.0) - at(a, 0.0);
                        Surface::Plane(Plane { origin: at(a, 0.0), normal: z.cross(d) })
                    }
                    Some(arc) => {
                        let radius = (a.x - arc.center.x).hypot(a.y - arc.center.y);
                        let axis = if arc.ccw { *z } else { -*z };
                        for (layer, depth) in [(0, 0.0), (1, bottom)] {
                            let arc = Arc { center: at(arc.center, depth), axis, radius };
                            curves.push(CurveSpec { from: v[layer], to: w[layer], curve: Curve::Arc(arc) });
                        }
                        // The body is outside the hole: an arc turning left (around the hole) is
                        // concave from the body.
                        Surface::Cylinder(Cylinder { origin: at(arc.center, bottom), axis: *z, radius, convex: !arc.ccw })
                    }
                },
                Frame::Rolled { d, radius, side } => {
                    let piece = Rolled { dev: d.dev, from: d.uv(a), to: d.uv(b), arc: l.arc(k).map(|arc| d.arc(arc)) };
                    for (layer, depth) in [(0, 0.0), (1, bottom)] {
                        curves.push(CurveSpec { from: v[layer], to: w[layer], curve: Curve::Rolled { piece, radius: radius + side * depth } });
                    }
                    // Into the hole is left of the profile in the sketch; mirrored, right in the
                    // development.
                    Surface::Wall(if d.mirrored { piece.reversed() } else { piece })
                }
            };
            faces.push(FaceSpec { id: l.sides[k], surface, loops: vec![vec![v[0], w[0], w[1], v[1]]] });
        }
    }
    faces.push(top);
    faces.push(low);
    build_with_curves(vertices, faces, curves)
}

/// The sketch's coordinates in space: `depth` runs out of the body from its face.
enum Frame {
    /// On a plane face: origin and axes, `z` its normal.
    Plane { o: Vec3, x: Vec3, y: Vec3, z: Vec3 },
    /// On a cylinder face of `radius`: `side` is 1 when the sheet lies inside it, −1 outside.
    Rolled { d: Developed, radius: f64, side: f64 },
}

impl Frame {
    fn flat(&self, p: Vec3) -> Vec2 {
        match self {
            Self::Plane { o, x, y, .. } => {
                let q = p - *o;
                Vec2::new(q.dot(*x), q.dot(*y))
            }
            Self::Rolled { d, .. } => d.flat(p).0,
        }
    }

    fn depth(&self, p: Vec3) -> f64 {
        match self {
            Self::Plane { o, z, .. } => (p - *o).dot(*z),
            Self::Rolled { d, radius, side } => (d.flat(p).1 - radius) * side,
        }
    }

    fn point(&self, p: Vec2, depth: f64) -> Vec3 {
        match self {
            Self::Plane { o, x, y, z } => *o + *x * p.x + *y * p.y + *z * depth,
            Self::Rolled { d, radius, side } => d.point(p, radius + side * depth),
        }
    }
}

/// Outlines of the faces among `other` that reach between the sketch's face and `bottom` (with
/// `true` if a face runs along the cut), on a plane: a face along the cut shows as its edges, one
/// across it covers an area.
fn plane_neighbours(target: &Body, other: &[Ix], frame: &Frame, z: Vec3, bottom: f64) -> Result<Vec<(ProfileLoop, bool)>, BrepError> {
    let others: Vec<(Ix, bool)> = in_order(other.par_iter().map(|&h| {
        let face = &target.faces()[h as usize];
        let depths = target.face_loops(h).flat_map(|l| target.loop_points(l)).map(|p| frame.depth(p));
        let (lo, hi) = depths.fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), d| (lo.min(d), hi.max(d)));
        let along = match &face.surface {
            Surface::Plane(p) => p.normal.dot(z) == 0.0,
            Surface::Cylinder(c) => c.axis.cross(z) == Vec3::ZERO,
            Surface::Wall(_) => false,
        };
        if hi <= bottom || lo >= 0.0 {
            return Ok(None);
        }
        if !along && face.surface.plane().is_none() {
            return Err(BrepError::Unsupported("cut: a curved face across the cut".into()));
        }
        Ok(Some((h, along)))
    }))?
    .into_iter()
    .flatten()
    .collect();
    let loops = in_order(others.into_par_iter().map(|(h, along)| Ok::<_, BrepError>((outline(target, h, frame)?, along))))?;
    Ok(loops.into_iter().flat_map(|(ls, along)| ls.into_iter().map(move |l| (l, along))).collect())
}

/// Outlines of the faces among `other` that reach between the radii `r` of a cut through a
/// rolled sheet, as [`plane_neighbours`]: walls of earlier cuts in the same development (along
/// the cut) and coaxial cylinders (across it) are exact in the development. Of any other face
/// only its box is known, in the development's own axes: it must keep clear of the box of every
/// profile's cut (`boxes`, the profiles' boxes in the sketch), or the cut is refused.
fn rolled_neighbours(
    target: &Body,
    other: &[Ix],
    frame: &Frame,
    d: &Developed,
    entry: &Cylinder,
    r: [f64; 2],
    boxes: &[Box2],
) -> Result<Vec<(ProfileLoop, bool)>, BrepError> {
    let unsupported = || BrepError::Unsupported("cut: a face of the body runs close to the cut".into());
    let (a, e1) = (d.dev.axis, d.dev.start);
    let local = Rigid::into_frame(d.dev.origin, e1, a.cross(e1), a);
    let radii = |h: Ix| {
        let rs = target.face_loops(h).flat_map(|l| target.loop_points(l)).map(|p| d.flat(p).1);
        rs.fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), x| (lo.min(x), hi.max(x)))
    };
    // Exact faces by outline (`Some(along)`), the others by their boxes (`None`).
    let kinds: Vec<Option<(Ix, Option<bool>)>> = in_order(other.par_iter().map(|&h| {
        Ok::<_, BrepError>(match target.faces()[h as usize].surface {
            Surface::Wall(w) if w.dev == d.dev => {
                let (lo, hi) = radii(h);
                (hi > r[0] && lo < r[1]).then_some((h, Some(true)))
            }
            Surface::Cylinder(c) if coaxial(&c, entry) => (c.radius > r[0] && c.radius < r[1]).then_some((h, Some(false))),
            _ => Some((h, None)),
        })
    }))?;
    // The cut of each profile in the development's axes: around the axis between its ends in
    // `u`, between the radii, along the axis between its ends in `v`.
    let cuts: Vec<(Vec3, Vec3)> = boxes
        .par_iter()
        .map(|&(lo, hi)| {
            let (v0, v1) = (d.uv(lo).y, d.uv(hi).y);
            let (mut a, mut b) = (Vec3::new(f64::INFINITY, f64::INFINITY, 0.0), Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, 0.0));
            for radius in r {
                let at = |u: f64| {
                    let (s, c) = libm::sincos(u / d.dev.neutral);
                    Vec3::new(radius * c, radius * s, 0.0)
                };
                let arc = Arc { center: Vec3::ZERO, axis: Vec3::new(0.0, 0.0, 1.0), radius };
                arc.extend_bounds(at(lo.x), at(hi.x), &mut a, &mut b);
            }
            (Vec3::new(a.x, a.y, v0.min(v1)), Vec3::new(b.x, b.y, v0.max(v1)))
        })
        .collect();
    let boxed: Vec<(Vec3, Vec3)> = kinds.iter().flatten().filter(|k| k.1.is_none()).map(|&(h, _)| target.face_bounds(h, &local)).collect();
    let flat = |b: &(Vec3, Vec3)| (Vec2::new(b.0.x, b.0.y), Vec2::new(b.1.x, b.1.y));
    let (cut_xy, boxed_xy): (Vec<Box2>, Vec<Box2>) = (cuts.iter().map(flat).collect(), boxed.iter().map(flat).collect());
    if between(&cut_xy, &boxed_xy).into_iter().any(|(c, f)| cuts[c].0.z <= boxed[f].1.z && boxed[f].0.z <= cuts[c].1.z) {
        return Err(unsupported());
    }
    let exact: Vec<(Ix, bool)> = kinds.into_iter().flatten().filter_map(|(h, along)| Some((h, along?))).collect();
    let loops = in_order(exact.into_par_iter().map(|(h, along)| Ok::<_, BrepError>((outline(target, h, frame)?, along))))?;
    Ok(loops.into_iter().flat_map(|(ls, along)| ls.into_iter().map(move |l| (l, along))).collect())
}

/// The loops of face `h` seen along the cut, in the sketch's coordinates. Points that fall
/// together (the two ends of an edge along the cut) are merged: a face along the cut shows as its
/// edges, there and back. In a development an arc about the axis is straight, and a piece of the
/// same development keeps its arc; a piece of another one has no outline here.
fn outline(body: &Body, h: Ix, frame: &Frame) -> Result<Vec<ProfileLoop>, BrepError> {
    body.face_loops(h)
        .map(|l| {
            let mut ring = ProfileLoop { points: Vec::new(), vertices: Vec::new(), sides: Vec::new(), arcs: Vec::new() };
            for he in body.loop_half_edges(l) {
                let edge = &body.edges()[body.half_edges()[he as usize].edge as usize];
                let p = frame.flat(body.point(body.half_edges()[he as usize].origin));
                let curve = body.edge_curve(edge).map(|c| if edge.half == he { c } else { c.reversed() });
                let arc = match (frame, curve) {
                    (_, None) | (Frame::Rolled { .. }, Some(Curve::Arc(_))) => None,
                    (Frame::Plane { z, .. }, Some(Curve::Arc(a))) => Some(ProfileArc { center: frame.flat(a.center), ccw: a.axis.dot(*z) > 0.0 }),
                    (Frame::Rolled { d, .. }, Some(Curve::Rolled { piece, .. })) if piece.dev == d.dev => piece.arc.map(|a| d.arc(a)),
                    _ => return Err(BrepError::Unsupported("cut: a face beside the cut has edges of a cut in another development".into())),
                };
                match ring.points.last() {
                    // An edge along the cut: its arc, if any, is the next one's.
                    Some(&q) if q == p => {
                        if let Some(last) = ring.arcs.last_mut() {
                            *last = last.or(arc);
                        }
                    }
                    _ => {
                        ring.points.push(p);
                        ring.arcs.push(arc);
                    }
                }
            }
            while ring.points.len() > 1 && ring.points.first() == ring.points.last() {
                ring.points.pop();
                ring.arcs.pop();
            }
            Ok(ring)
        })
        .collect()
}

/// Overlapping pairs `(a, b)`, `a < b`, of `boxes`: a sweep along X.
fn within(boxes: &[Box2]) -> Vec<(usize, usize)> {
    let mut order: Vec<usize> = (0..boxes.len()).collect();
    order.sort_unstable_by(|&a, &b| boxes[a].0.x.total_cmp(&boxes[b].0.x).then(a.cmp(&b)));
    let mut active: Vec<usize> = Vec::new();
    let mut pairs = Vec::new();
    for &i in &order {
        active.retain(|&a| boxes[a].1.x >= boxes[i].0.x);
        pairs.extend(active.iter().filter(|&&a| overlap(boxes[a], boxes[i])).map(|&a| (a.min(i), a.max(i))));
        active.push(i);
    }
    pairs.sort_unstable();
    pairs
}

/// Overlapping pairs `(a, b)` of a box of `left` and one of `right`, sorted.
fn between(left: &[Box2], right: &[Box2]) -> Vec<(usize, usize)> {
    let all: Vec<Box2> = left.iter().chain(right).copied().collect();
    within(&all).into_iter().filter(|&(a, b)| a < left.len() && b >= left.len()).map(|(a, b)| (a, b - left.len())).collect()
}

fn overlap(a: Box2, b: Box2) -> bool {
    a.0.x <= b.1.x && b.0.x <= a.1.x && a.0.y <= b.1.y && b.0.y <= a.1.y
}
