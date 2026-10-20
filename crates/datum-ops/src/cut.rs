//! Cuts through a plate (M5b, D63): the closed profiles of a sketch on a plane face, cut
//! perpendicular to the face through to the opposite one, optionally repeated on a grid in the
//! sketch's coordinates (punches of a turret press, PAINS P4).
//!
//! Every decision is a 2D one in the sketch's coordinates, by exact predicates on segments and
//! arcs: a profile lies inside the entry face and inside the exit face, touches neither their
//! edges nor another profile, and no other face of the body crosses the cut. The result is then
//! built directly: the profiles become holes of both faces, their edges walls between them. The
//! coordinates of the body's vertices in the sketch's frame are rounded once; on a face along the
//! axes, the usual case, they are exact.

use datum_brep::{
    Arc, ArcSpec, Body, BrepError, Cylinder, FaceSpec, Ix, Plane, ProfileArc, ProfileLoop, Surface, TopoId, Vertex, build_with_arcs, in_order,
};
use datum_math::{Vec2, Vec3};
use rayon::prelude::*;

use crate::sketch::{Placed, profile_loops};

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

/// The profiles of `sketch`, which lies on a plane face of `target`, cut through `target`.
pub fn cut(feature: TopoId, target: &Body, sketch: &Placed, grid: ProfileGrid) -> Result<Body, BrepError> {
    let unsupported = |m: String| BrepError::Unsupported(format!("cut: {m}"));
    let id = sketch.face.ok_or_else(|| BrepError::Sketch("a cut takes a sketch placed on a face of its body".into()))?;
    let f = target.faces().binary_search_by_key(&id, |f| f.id).map_err(|_| BrepError::LostReference(format!("face {id:?} is not in the body")))?;
    let Surface::Plane(entry) = target.faces()[f].surface else { return Err(unsupported("the sketch's face is curved".into())) };
    if grid.count.contains(&0) {
        return Err(BrepError::Degenerate("a cut pattern needs at least one copy".into()));
    }
    let [o, x, y, z] = sketch.frame;
    let frame = Frame { o, x, y, z };

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

    // The exit: the nearest face below facing exactly the other way that holds the first profile.
    let first = profiles[0].points[0];
    let exit = (0..target.faces().len() as Ix)
        .filter_map(|g| {
            let Surface::Plane(p) = target.faces()[g as usize].surface else { return None };
            let opposite = g as usize != f && p.normal.cross(entry.normal) == Vec3::ZERO && p.normal.dot(entry.normal) < 0.0;
            let d =
                frame.depth(target.point(target.half_edges()[target.loops()[target.faces()[g as usize].first_loop as usize].first as usize].origin));
            (opposite && d < 0.0 && outline(target, g, &frame).iter().map(|l| l.winding(first)).sum::<i32>() != 0).then_some((g, d))
        })
        .max_by(|a, b| a.1.total_cmp(&b.1));
    let Some((g, bottom)) = exit else {
        return Err(unsupported("no face opposite the sketch's face holds the profiles: a cut goes through a plate".into()));
    };

    // Inside both faces, clear of their edges and of their holes.
    for (face, name) in [(f as Ix, "entry"), (g, "exit")] {
        let loops = outline(target, face, &frame);
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

    // No other face crosses the cut between the two. A face along the cut shows as its edges;
    // one across it covers an area.
    let other: Vec<Ix> = (0..target.faces().len() as Ix).filter(|&h| h as usize != f && h != g).collect();
    let others: Vec<(Ix, bool)> = in_order(other.par_iter().map(|&h| {
        let face = &target.faces()[h as usize];
        let depths = target.face_loops(h).flat_map(|l| target.loop_points(l)).map(|p| frame.depth(p));
        let (lo, hi) = depths.fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), d| (lo.min(d), hi.max(d)));
        let along = match &face.surface {
            Surface::Plane(p) => p.normal.dot(z) == 0.0,
            Surface::Cylinder(c) => c.axis.cross(z) == Vec3::ZERO,
        };
        if hi <= bottom || lo >= 0.0 {
            return Ok(None);
        }
        if !along && face.surface.plane().is_none() {
            return Err(unsupported("a curved face across the cut".into()));
        }
        Ok(Some((h, along)))
    }))?
    .into_iter()
    .flatten()
    .collect();
    let crossing: Vec<(ProfileLoop, bool)> =
        others.par_iter().flat_map_iter(|&(h, along)| outline(target, h, &frame).into_iter().map(move |l| (l, along))).collect();
    let crossing_boxes: Vec<Box2> = crossing.iter().map(|(l, _)| l.bounds()).collect();
    in_order(between(&boxes, &crossing_boxes).into_par_iter().map(|(p, l)| {
        let (lp, along) = (&crossing[l].0, crossing[l].1);
        let inside = profiles[p].winding(lp.points[0]) != 0 || (!along && lp.winding(profiles[p].points[0]) != 0);
        if inside || profiles[p].touches(lp) { Err(unsupported("another face of the body crosses the cut".into())) } else { Ok(()) }
    }))?;

    // The body: the profiles as holes of both faces, walls between them.
    let at = |p: Vec2, d: f64| frame.point(p, d);
    let mut vertices: Vec<Vertex> = target.vertices().to_vec();
    let mut arcs = target.arc_specs();
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
            let surface = match l.arc(k) {
                None => {
                    let d = at(b, 0.0) - at(a, 0.0);
                    Surface::Plane(Plane { origin: at(a, 0.0), normal: z.cross(d) })
                }
                Some(arc) => {
                    let radius = (a.x - arc.center.x).hypot(a.y - arc.center.y);
                    let axis = if arc.ccw { z } else { -z };
                    for (layer, depth) in [(0, 0.0), (1, bottom)] {
                        arcs.push(ArcSpec { from: v[layer], to: w[layer], arc: Arc { center: at(arc.center, depth), axis, radius } });
                    }
                    // The body is outside the hole: an arc turning left (around the hole) is
                    // concave from the body.
                    Surface::Cylinder(Cylinder { origin: at(arc.center, bottom), axis: z, radius, convex: !arc.ccw })
                }
            };
            faces.push(FaceSpec { id: l.sides[k], surface, loops: vec![vec![v[0], w[0], w[1], v[1]]] });
        }
    }
    faces.push(top);
    faces.push(low);
    build_with_arcs(vertices, faces, arcs)
}

/// The sketch's frame: origin and axes, `z` its normal.
struct Frame {
    o: Vec3,
    x: Vec3,
    y: Vec3,
    z: Vec3,
}

impl Frame {
    fn flat(&self, p: Vec3) -> Vec2 {
        let q = p - self.o;
        Vec2::new(q.dot(self.x), q.dot(self.y))
    }

    fn depth(&self, p: Vec3) -> f64 {
        (p - self.o).dot(self.z)
    }

    fn point(&self, p: Vec2, depth: f64) -> Vec3 {
        self.o + self.x * p.x + self.y * p.y + self.z * depth
    }
}

/// The loops of face `h` seen along the sketch's normal, in its coordinates. Points that fall
/// together (the two ends of an edge along the normal) are merged: a face along the cut shows as
/// its edges, there and back.
fn outline(body: &Body, h: Ix, frame: &Frame) -> Vec<ProfileLoop> {
    body.face_loops(h)
        .map(|l| {
            let mut ring = ProfileLoop { points: Vec::new(), vertices: Vec::new(), sides: Vec::new(), arcs: Vec::new() };
            for he in body.loop_half_edges(l) {
                let edge = &body.edges()[body.half_edges()[he as usize].edge as usize];
                let p = frame.flat(body.point(body.half_edges()[he as usize].origin));
                let arc = body.edge_arc(edge).map(|a| {
                    let turn = if edge.half == he { a.axis } else { -a.axis };
                    ProfileArc { center: frame.flat(a.center), ccw: turn.dot(frame.z) > 0.0 }
                });
                match ring.points.last() {
                    // An edge along the normal: its arc, if any, is the next one's.
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
            ring
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
