//! Cuts through a plate or a rolled sheet (M5b–c, D63): the closed profiles of a sketch on a
//! plane face cut perpendicular to the face through to the opposite one, or of a sketch in the
//! development of a cylinder face cut along the radius through the sheet (as a hole punched in
//! the blank before rolling); a pattern of the sketch's (D128) gives many profiles at once
//! (punches of a turret press, PAINS P4). From a plane face the profiles also go a depth along
//! its normal (D187): into the body, a pocket with a new plane floor; out of it, a boss with a
//! new plane top. The face's own plane is known from where the sketch lies, so no Boolean
//! operation meets coplanar faces or arcs (D185 item 7).
//!
//! Every decision is a 2D one in the sketch's coordinates, by exact predicates on segments and
//! arcs: a profile lies inside the entry face and inside the exit face, touches neither their
//! edges nor another profile, and no other face of the body crosses the cut. The result is then
//! built directly: the profiles become holes of both faces, their edges walls between them. The
//! coordinates of the body's vertices in the sketch's frame are rounded once; on a plane face
//! along the axes, the usual case, they are exact.
use datum_brep::{
    Arc, Body, Box2, BrepError, Curve, CurveSpec, Cylinder, FaceSpec, Ix, Plane, ProfileArc, ProfileLoop, Rolled, Surface, TopoId, Vertex,
    build_with_curves, in_order, overlapping,
};
use datum_math::{Rigid, Vec2, Vec3};
use rayon::prelude::*;

use crate::sketch::{Developed, Found, Placed, coaxial, opposite_cylinder, profile_loops, profile_regions};

/// How far the profiles of a sketch on a face go.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Reach {
    /// Into the body through to the opposite face.
    Through,
    /// Along the face's normal, out of the body: a negative depth makes a pocket, a positive
    /// one a boss.
    Depth(f64),
}

/// The profiles of `sketch`, which lies on a face of `target`, cut into `target` or added to it
/// as far as `reach` says.
pub fn from_face(feature: TopoId, target: &Body, sketch: &Placed, reach: Reach) -> Result<Body, BrepError> {
    let what = match reach {
        Reach::Through => "cut",
        Reach::Depth(d) if d < 0.0 => "pocket",
        Reach::Depth(d) if d > 0.0 => "boss",
        Reach::Depth(_) => return Err(BrepError::Degenerate("an extrusion from a face of zero depth".into())),
    };
    let unsupported = |m: String| BrepError::Unsupported(format!("{what}: {m}"));
    let id = sketch.face.ok_or_else(|| BrepError::Sketch("a cut takes a sketch placed on a face of its body".into()))?;
    let f = target.faces().binary_search_by_key(&id, |f| f.id).map_err(|_| BrepError::LostReference(format!("face {id:?} is not in the body")))?;
    let frame = match (target.faces()[f].surface, sketch.developed.as_deref().copied()) {
        (Surface::Plane(_), None) => {
            let [o, x, y, z] = sketch.frame;
            Frame::Plane { o, x, y, z }
        }
        (Surface::Cylinder(c), Some(d)) if reach == Reach::Through => Frame::Rolled { d, radius: c.radius, side: if c.convex { 1.0 } else { -1.0 } },
        (Surface::Cylinder(_), Some(_)) => return Err(unsupported("a sketch on a rolled sheet cuts through it".into())),
        _ => return Err(BrepError::Invalid("the sketch does not lie on its face".into())),
    };
    // A pocket's floor and a boss's top take the far caps of an extrusion of the sketch: the
    // regions its profiles bound, none inside another.
    let (found, ends): (Vec<Found>, Vec<TopoId>) = match reach {
        Reach::Through => (profile_loops(feature, sketch)?, Vec::new()),
        Reach::Depth(_) => {
            let regions = profile_regions(feature, sketch)?;
            if regions.iter().any(|r| !r.holes.is_empty()) {
                return Err(unsupported("a profile inside another".into()));
            }
            regions.into_iter().map(|r| (Found { ring: r.outer, name: String::new(), copy: None }, r.caps[1])).unzip()
        }
    };
    // The profiles, counterclockwise in the sketch; a pattern's copies among them, each turning
    // as its loop of the cell does (found once).
    let mut turns: std::collections::BTreeMap<(usize, usize), bool> = std::collections::BTreeMap::new();
    let ccw: Vec<bool> = found
        .iter()
        .map(|l| match l.copy {
            None => l.ring.counterclockwise(),
            Some(c) => *turns.entry((c.pattern, c.base)).or_insert_with(|| l.ring.counterclockwise()),
        })
        .collect();
    let profiles: Vec<ProfileLoop> = found.into_par_iter().zip(ccw).map(|(l, ccw)| if ccw { l.ring } else { l.ring.reversed() }).collect();
    let boxes: Vec<Box2> = profiles.iter().map(ProfileLoop::bounds).collect();

    // Profiles neither touch nor hold one another.
    in_order(overlapping(&boxes).into_par_iter().map(|(a, b)| {
        if profiles[a].touches(&profiles[b]) {
            Err(BrepError::Degenerate("cut profiles touch".into()))
        } else if profiles[a].winding(profiles[b].points[0]) != 0 || profiles[b].winding(profiles[a].points[0]) != 0 {
            Err(unsupported("a profile inside another".into()))
        } else {
            Ok(())
        }
    }))?;

    // A profile off the sketch's face (a sketch on a face has its own axes: the model's origin
    // projected, a model axis): said so, with where the face is in those axes (issue #100).
    let entry = outline(target, f as Ix, &frame)?;
    if entry.iter().map(|l| l.winding(profiles[0].points[0])).sum::<i32>() == 0 {
        let (lo, hi) = entry[0].bounds();
        return Err(unsupported(format!(
            "a profile lies off the sketch's face: in the sketch's axes the face spans x {:.3}…{:.3}, y {:.3}…{:.3}",
            lo.x, hi.x, lo.y, hi.y
        )));
    }

    // The exit: on a plane, the nearest face below facing exactly the other way that holds the
    // first profile; on a cylinder, the face across the sheet. A pocket and a boss have none.
    let exit = match (reach, &frame) {
        (Reach::Depth(d), _) => Some((None, d)),
        (Reach::Through, Frame::Plane { z, .. }) => {
            let first = profiles[0].points[0];
            let faces = (0..target.faces().len() as Ix).filter_map(|g| {
                let Surface::Plane(p) = target.faces()[g as usize].surface else { return None };
                let opposite = g as usize != f && p.normal.cross(*z) == Vec3::ZERO && p.normal.dot(*z) < 0.0;
                let d = frame
                    .depth(target.point(target.half_edges()[target.loops()[target.faces()[g as usize].first_loop as usize].first as usize].origin));
                let holds = || outline(target, g, &frame).is_ok_and(|ls| ls.iter().map(|l| l.winding(first)).sum::<i32>() != 0);
                (opposite && d < 0.0 && holds()).then_some((g, d))
            });
            faces.max_by(|a, b| a.1.total_cmp(&b.1)).map(|(g, d)| (Some(g), d))
        }
        (Reach::Through, Frame::Rolled { radius, side, .. }) => opposite_cylinder(target, f).map(|(g, c)| (Some(g), (c.radius - radius) * side)),
    };
    let Some((g, end)) = exit else {
        return Err(unsupported("no face opposite the sketch's face holds the profiles: a cut goes through a plate".into()));
    };

    // Inside both faces, clear of their edges and of their holes.
    for (face, name) in std::iter::once((f as Ix, "entry")).chain(g.map(|g| (g, "exit"))) {
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

    // No other face crosses the cut between the two, or reaches a pocket's floor or a boss's top.
    let other: Vec<Ix> = (0..target.faces().len() as Ix).filter(|&h| h as usize != f && Some(h) != g).collect();
    let span = match reach {
        Reach::Through => Span { lo: end, hi: 0.0, closed: [false, false] },
        Reach::Depth(d) if d < 0.0 => Span { lo: d, hi: 0.0, closed: [true, false] },
        Reach::Depth(d) => Span { lo: 0.0, hi: d, closed: [false, true] },
    };
    let crossing = match &frame {
        Frame::Plane { .. } => plane_neighbours(target, &other, &frame, &boxes, span)?,
        Frame::Rolled { d, radius, side } => {
            let radii = [*radius, radius + side * end];
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
        if inside || profiles[p].touches(lp) { Err(unsupported(format!("another face of the body meets the {what}"))) } else { Ok(()) }
    }))?;

    // The body: the profiles as holes of the sketch's face, walls along them, and holes of the
    // exit face or new faces at the far end. A boss's walls face away from its profiles, the
    // walls of a hole into them.
    let outward = matches!(reach, Reach::Depth(d) if d > 0.0);
    let at = |p: Vec2, d: f64| frame.point(p, d);
    let mut vertices: Vec<Vertex> = target.vertices().to_vec();
    let mut curves = target.curve_specs();
    let mut faces: Vec<FaceSpec> = other.iter().map(|&h| target.face_spec(h)).collect();
    let mut top = target.face_spec(f as Ix);
    let mut low = g.map(|g| target.face_spec(g));
    for (i, l) in profiles.iter().enumerate() {
        let n = l.points.len();
        // Holes run clockwise seen from outside: against the profile on the entry face, along
        // it on the exit face, which faces the other way. A floor or a top faces out along the
        // normal: the profile runs counterclockwise on it.
        top.loops.push(l.vertices.iter().rev().map(|v| v[0]).collect());
        let far: Vec<TopoId> = l.vertices.iter().map(|v| v[1]).collect();
        match &mut low {
            Some(low) => low.loops.push(far),
            None => {
                let Frame::Plane { z, .. } = &frame else { return Err(BrepError::Invalid("a pocket in a development".into())) };
                faces.push(FaceSpec { id: ends[i], surface: Surface::Plane(Plane { origin: at(l.points[0], end), normal: *z }), loops: vec![far] });
            }
        }
        for k in 0..n {
            let (a, b) = (l.points[k], l.points[(k + 1) % n]);
            let (v, w) = (l.vertices[k], l.vertices[(k + 1) % n]);
            vertices.push(Vertex { id: v[0], point: at(a, 0.0) });
            vertices.push(Vertex { id: v[1], point: at(a, end) });
            let surface = match &frame {
                Frame::Plane { z, .. } => match l.arc(k) {
                    None => {
                        let d = at(b, 0.0) - at(a, 0.0);
                        let normal = z.cross(d);
                        Surface::Plane(Plane { origin: at(a, 0.0), normal: if outward { -normal } else { normal } })
                    }
                    Some(arc) => {
                        let radius = libm::hypot(a.x - arc.center.x, a.y - arc.center.y);
                        let axis = if arc.ccw { *z } else { -*z };
                        for (layer, depth) in [(0, 0.0), (1, end)] {
                            let arc = Arc { center: at(arc.center, depth), axis, radius };
                            curves.push(CurveSpec { from: v[layer], to: w[layer], curve: Curve::Arc(arc) });
                        }
                        // The body is outside the hole: an arc turning left (around the hole) is
                        // concave from the body, and convex around a boss.
                        Surface::Cylinder(Cylinder { origin: at(arc.center, end), axis: *z, radius, convex: arc.ccw == outward })
                    }
                },
                Frame::Rolled { d, radius, side } => {
                    let piece = Rolled { dev: d.dev, from: d.uv(a), to: d.uv(b), arc: l.arc(k).map(|arc| d.arc(arc)) };
                    for (layer, depth) in [(0, 0.0), (1, end)] {
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
    faces.extend(low);
    build_with_curves(vertices, faces, curves)
}

/// The depths a cut spans along the normal of its sketch's face, each end in it or not.
#[derive(Clone, Copy)]
struct Span {
    lo: f64,
    hi: f64,
    closed: [bool; 2],
}

impl Span {
    /// Whether the depths from `lo` to `hi` meet the span.
    fn meets(&self, lo: f64, hi: f64) -> bool {
        let above = if self.closed[0] { hi >= self.lo } else { hi > self.lo };
        let below = if self.closed[1] { lo <= self.hi } else { lo < self.hi };
        above && below
    }
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

/// Outlines of the faces among `other` that meet the cut's `span` of depths (with
/// `true` if a face runs along the cut), on a plane: a face along the cut shows as its edges, one
/// across it covers an area. A curved face across the cut (a bend of a sheet) is known by its box
/// in the sketch's axes: it must keep clear of every profile's (`boxes`).
fn plane_neighbours(target: &Body, other: &[Ix], frame: &Frame, boxes: &[Box2], span: Span) -> Result<Vec<(ProfileLoop, bool)>, BrepError> {
    let Frame::Plane { o, x, y, z } = *frame else { return Err(BrepError::Invalid("a plane cut in a development".into())) };
    // Faces by outline (`Some(along)`), curved faces across the cut by their boxes (`None`).
    let kinds: Vec<Option<(Ix, Option<bool>)>> = in_order(other.par_iter().map(|&h| {
        let face = &target.faces()[h as usize];
        let along = match &face.surface {
            Surface::Plane(p) => p.normal.dot(z) == 0.0,
            Surface::Cylinder(c) => c.axis.cross(z) == Vec3::ZERO,
            Surface::Wall(_) => false,
        };
        if !along && face.surface.plane().is_none() {
            return Ok::<_, BrepError>(Some((h, None)));
        }
        let depths = target.face_loops(h).flat_map(|l| target.loop_points(l)).map(|p| frame.depth(p));
        let (lo, hi) = depths.fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), d| (lo.min(d), hi.max(d)));
        Ok(span.meets(lo, hi).then_some((h, Some(along))))
    }))?;
    let cuts: Vec<(Vec3, Vec3)> = boxes.iter().map(|&(lo, hi)| (Vec3::new(lo.x, lo.y, span.lo), Vec3::new(hi.x, hi.y, span.hi))).collect();
    keep_clear(target, &kinds, &Rigid::into_frame(o, x, y, z), &cuts, span.closed)?;
    exact_outlines(target, kinds, frame)
}

/// Faces known only by their boxes (`None` in `kinds`) must keep clear of every profile's cut
/// (`cuts`, boxes in the axes of `local`, their ends along Z in them or not as `closed` says): a
/// box that reaches one refuses the cut.
fn keep_clear(target: &Body, kinds: &[Option<(Ix, Option<bool>)>], local: &Rigid, cuts: &[(Vec3, Vec3)], closed: [bool; 2]) -> Result<(), BrepError> {
    let boxed: Vec<(Vec3, Vec3)> = kinds.iter().flatten().filter(|k| k.1.is_none()).map(|&(h, _)| target.face_bounds(h, local)).collect();
    let flat = |b: &(Vec3, Vec3)| (Vec2::new(b.0.x, b.0.y), Vec2::new(b.1.x, b.1.y));
    let (cut_xy, boxed_xy): (Vec<Box2>, Vec<Box2>) = (cuts.iter().map(flat).collect(), boxed.iter().map(flat).collect());
    let meets = |c: usize, f: usize| Span { lo: cuts[c].0.z, hi: cuts[c].1.z, closed }.meets(boxed[f].0.z, boxed[f].1.z);
    if between(&cut_xy, &boxed_xy).into_iter().any(|(c, f)| meets(c, f)) {
        return Err(BrepError::Unsupported("cut: a face of the body runs close to the cut".into()));
    }
    Ok(())
}

/// The outlines of the faces known exactly (`Some(along)` in `kinds`), each loop with `along`.
fn exact_outlines(target: &Body, kinds: Vec<Option<(Ix, Option<bool>)>>, frame: &Frame) -> Result<Vec<(ProfileLoop, bool)>, BrepError> {
    let exact: Vec<(Ix, bool)> = kinds.into_iter().flatten().filter_map(|(h, along)| Some((h, along?))).collect();
    let loops = in_order(exact.into_par_iter().map(|(h, along)| Ok::<_, BrepError>((outline(target, h, frame)?, along))))?;
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
    keep_clear(target, &kinds, &local, &cuts, [false, false])?;
    exact_outlines(target, kinds, frame)
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

/// Overlapping pairs `(a, b)` of a box of `left` and one of `right`, sorted.
fn between(left: &[Box2], right: &[Box2]) -> Vec<(usize, usize)> {
    let all: Vec<Box2> = left.iter().chain(right).copied().collect();
    overlapping(&all).into_iter().filter(|&(a, b)| a < left.len() && b >= left.len()).map(|(a, b)| (a, b - left.len())).collect()
}
