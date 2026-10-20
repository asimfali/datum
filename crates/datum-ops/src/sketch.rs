//! Sketches placed in space, and extrusions of their profiles (M3, D53).

use std::collections::BTreeMap;

use datum_brep::{Body, BrepError, Curve, Cylinder, Development, Ix, ProfileArc, ProfileLoop, Region, Surface, TopoId, extrude};
use datum_math::{Rigid, Vec2, Vec3};
use datum_sketch::{Arc, Circle, Line, Sketch, Solved, solve};
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
    /// Origin and the X, Y, Z axes of the sketch plane, Z its normal. On a cylinder face, the
    /// plane touching the face at the sketch's origin; the sketch lies on the face (`developed`).
    pub frame: [Vec3; 4],
    /// On a cylinder face, where the sketch lies (boxed: most sketches are on planes).
    pub developed: Option<Box<Developed>>,
    pub solved: Solved,
    pub lines: BTreeMap<String, Line>,
    pub arcs: BTreeMap<String, Arc>,
    /// Circles, each diameter a number.
    pub circles: BTreeMap<String, Circle>,
    /// The face the sketch lies on, if it was placed on one.
    pub face: Option<TopoId>,
    pub hash: [u8; 32],
}

/// A sketch on a cylinder face of a rolled sheet (M5c, D68): its coordinates are the sheet's
/// development, lengths along the neutral layer as on the blank before rolling. X runs along `u`,
/// Y along `v`, or against it (`mirrored`) on the face that sees the axis, so that seen from
/// outside the face the sketch is drawn as on a plane face.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Developed {
    pub dev: Development,
    pub mirrored: bool,
    /// The radius of the sketch's face.
    pub radius: f64,
    /// The angle the sheet spans about the axis from `dev.start`.
    pub span: f64,
}

impl Developed {
    /// Sketch coordinates to the development's and back.
    pub fn uv(&self, p: Vec2) -> Vec2 {
        if self.mirrored { Vec2::new(p.x, -p.y) } else { p }
    }

    /// The point at `p` of the sketch, `r` from the axis.
    pub fn point(&self, p: Vec2, r: f64) -> Vec3 {
        self.dev.point(self.uv(p), r)
    }

    /// The sketch coordinates of `q`, and its distance from the axis. The turn about the axis
    /// is cut opposite the middle of the sheet.
    pub fn flat(&self, q: Vec3) -> (Vec2, f64) {
        let (p, r) = self.dev.unroll(q, 0.5 * self.span * self.dev.neutral);
        (self.uv(p), r)
    }

    /// An arc of the sketch in the development's coordinates, or the other way.
    pub fn arc(&self, a: ProfileArc) -> ProfileArc {
        ProfileArc { center: self.uv(a.center), ccw: a.ccw != self.mirrored }
    }
}

/// Whether two cylinders share their axis, exactly: as the faces of one extruded profile do.
pub(crate) fn coaxial(a: &Cylinder, b: &Cylinder) -> bool {
    a.axis.cross(b.axis) == Vec3::ZERO && (a.origin - b.origin).cross(b.axis) == Vec3::ZERO
}

/// The face across a rolled sheet from cylinder face `f`: the nearest coaxial cylinder face
/// facing the other way, on the side of the body.
/// None if there is none, or two at the same distance.
pub(crate) fn opposite_cylinder(body: &Body, f: usize) -> Option<(Ix, Cylinder)> {
    let Surface::Cylinder(c) = body.faces()[f].surface else { return None };
    let mut found: Vec<(Ix, Cylinder)> = (0..body.faces().len())
        .filter_map(|g| match body.faces()[g].surface {
            Surface::Cylinder(o) if g != f && coaxial(&o, &c) && o.convex != c.convex && (o.radius < c.radius) == c.convex => Some((g as Ix, o)),
            _ => None,
        })
        .collect();
    found.sort_by(|a, b| (a.1.radius - c.radius).abs().total_cmp(&(b.1.radius - c.radius).abs()));
    match found[..] {
        [one] => Some(one),
        [a, b, ..] if a.1.radius != b.1.radius => Some(a),
        _ => None,
    }
}

/// Where a sketch on cylinder face `f` lies, its neutral layer at `k` of the thickness from the
/// inside. The development is the sheet's, whichever side the sketch is on: its axis and origin
/// those of the inner face, `u` zero where that face starts turning about the axis, `v` zero
/// across the axis from the world origin (as on a plane face).
fn unrolled(body: &Body, f: usize, c: &Cylinder, k: f64) -> Result<Developed, BrepError> {
    let unsupported = |m: &str| BrepError::Unsupported(format!("a sketch on a cylinder face: {m}"));
    if !(0.0..=1.0).contains(&k) {
        return Err(BrepError::Degenerate(format!("a K-factor lies between 0 and 1, got {k}")));
    }
    let (g, other) = opposite_cylinder(body, f).ok_or_else(|| unsupported("no face across the sheet (a coaxial cylinder facing the other way)"))?;
    let (inside, inner, outer) = if c.convex { (g, other, *c) } else { (f as Ix, *c, other) };
    let axis = inner.axis * (1.0 / inner.axis.dot(inner.axis).sqrt());
    let across = |q: Vec3| {
        let w = q - inner.origin;
        w - axis * w.dot(axis)
    };
    // The inner face's outline turns about the axis and back: from its least turn, the sheet.
    let l = body.faces()[inside as usize].first_loop;
    let first = body.point(body.half_edges()[body.loop_half_edges(l).start as usize].origin);
    let (mut turn, mut least, mut most, mut start) = (0.0f64, 0.0f64, 0.0f64, across(first));
    for h in body.loop_half_edges(l) {
        let he = &body.half_edges()[h as usize];
        let edge = &body.edges()[he.edge as usize];
        let (u, v) = (body.point(he.origin), body.point(body.half_edges()[body.next(h) as usize].origin));
        match body.edge_curve(edge) {
            // Along the axis: the sheet's edge, unless the cylinder goes on past it (a tube cut
            // into faces).
            None => {
                let across = body.loops()[body.half_edges()[he.twin as usize].loop_ix as usize].face;
                if matches!(body.faces()[across as usize].surface, Surface::Cylinder(o) if coaxial(&o, &inner) && o.convex == inner.convex) {
                    return Err(unsupported("the face is part of a closed tube: a development needs the sheet's edge"));
                }
            }
            Some(Curve::Arc(arc)) => {
                let arc = if edge.half == h { arc } else { arc.reversed() };
                turn += arc.sweep(u, v) * arc.axis.dot(axis).signum();
            }
            Some(Curve::Rolled { .. }) => return Err(unsupported("the face's outline is a cut")),
        }
        if turn < least {
            (least, start) = (turn, across(v));
        }
        most = most.max(turn);
    }
    if turn.abs() > std::f64::consts::PI {
        return Err(unsupported("the face closes around the axis: a development needs the sheet's edge"));
    }
    let start = start * (1.0 / start.dot(start).sqrt());
    let dev =
        Development { origin: inner.origin - axis * inner.origin.dot(axis), axis, start, neutral: inner.radius + k * (outer.radius - inner.radius) };
    Ok(Developed { dev, mirrored: !c.convex, radius: c.radius, span: most - least })
}

/// The diameter of circle `c`, a number once the sketch is resolved.
fn diameter(c: &Circle) -> f64 {
    c.diameter.value().unwrap_or(0.0)
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
/// `host`. On a plane face, the origin is the world origin projected onto the face's plane and X
/// the world axis least along the normal, projected: on a top face sketch coordinates are world X
/// and Y. The frame follows the face's plane only. On a cylinder face of a rolled sheet the
/// sketch lies in the sheet's development (see [`Developed`]) with its neutral layer at
/// `k_factor` of the thickness. A face gone after a rebuild is a lost reference.
pub fn place(sketch: &Sketch, plane: Option<&SketchPlane>, face: Option<(&Body, &str)>, k_factor: Option<f64>) -> Result<Placed, BrepError> {
    let mut developed = None;
    let (frame, on) = match (plane, face) {
        (Some(_), Some(_)) => return Err(BrepError::Sketch("give a plane or a face, not both".into())),
        (Some(p), None) => (frame(p.origin, p.x_axis, p.normal)?, None),
        (None, Some((body, id))) => {
            let id = u128::from_str_radix(id, 16).map(TopoId).map_err(|_| BrepError::Sketch(format!("`{id}` is not a face ID")))?;
            let f = body
                .faces()
                .binary_search_by_key(&id, |f| f.id)
                .map_err(|_| BrepError::LostReference(format!("face {id:?} is not in the input body")))?;
            match body.faces()[f].surface {
                Surface::Plane(_) if k_factor.is_some() => return Err(BrepError::Sketch("`k_factor` is for a sketch on a cylinder face".into())),
                Surface::Plane(plane) => {
                    let n = unit(plane.normal).ok_or_else(|| BrepError::Degenerate("the face has no normal".into()))?;
                    let axes = [Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0)];
                    let x = axes.into_iter().min_by(|a, b| a.dot(n).abs().total_cmp(&b.dot(n).abs())).unwrap_or(axes[0]);
                    (frame(n * n.dot(plane.origin), x, n)?, Some(id))
                }
                Surface::Cylinder(c) => {
                    let k = k_factor.ok_or_else(|| BrepError::Sketch("a sketch on a cylinder face needs the sheet's `k_factor`".into()))?;
                    let d = unrolled(body, f, &c, k)?;
                    developed = Some(d);
                    let r = d.dev.radial(0.0);
                    (frame(d.point(Vec2::new(0.0, 0.0), c.radius), d.dev.axis.cross(r), if c.convex { r } else { -r })?, Some(id))
                }
                Surface::Wall(_) => return Err(BrepError::Unsupported("a sketch on the wall of a cut".into())),
            }
        }
        (None, None) => ([Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0)], None),
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
    for (name, a) in &sketch.arcs {
        for s in [name, &a.center, &a.from, &a.to] {
            bytes(s.as_bytes());
        }
        bytes(&[u8::from(a.construction)]);
    }
    for (name, c) in &sketch.circles {
        for s in [name, &c.center] {
            bytes(s.as_bytes());
        }
        bytes(&diameter(c).to_bits().to_le_bytes());
        bytes(&[u8::from(c.construction)]);
    }
    // The face matters to a cut: two coplanar faces give one frame.
    if let Some(id) = on {
        bytes(&id.0.to_le_bytes());
    }
    if let Some(d) = &developed {
        bytes(b"developed");
        let [o, a, s] = [d.dev.origin, d.dev.axis, d.dev.start].map(Vec3::to_array);
        for c in o.into_iter().chain(a).chain(s).chain([d.dev.neutral, d.radius, d.span]) {
            bytes(&c.to_bits().to_le_bytes());
        }
        bytes(&[u8::from(d.mirrored)]);
    }
    Ok(Placed {
        face: on,
        developed: developed.map(Box::new),
        frame,
        solved,
        lines: sketch.lines.clone(),
        arcs: sketch.arcs.clone(),
        circles: sketch.circles.clone(),
        hash: *h.finalize().as_bytes(),
    })
}

/// The closed profiles of `s` swept from `z0` to `z1` along its normal. Profile lines and arcs
/// (not construction) must form closed loops that neither branch nor touch, and a circle is a
/// loop of its own; a loop inside another is a hole, one inside a hole an island. Side faces are
/// named by their lines and arcs, vertices by their points (§4.2), a circle's by its name:
/// renaming nothing keeps every ID through edits of the sketch.
pub fn extrude_sketch(feature: TopoId, s: &Placed, z0: f64, z1: f64) -> Result<Body, BrepError> {
    if s.developed.is_some() {
        return Err(BrepError::Unsupported("an extrusion of a sketch on a cylinder face".into()));
    }
    let mut loops = profile_loops(feature, s)?;
    // Each loop with the name its caps would be named by: its lowest point.
    let inside = |a: &Found, b: &Found| a.ring.winding(b.ring.points[0]) != 0;
    let n = loops.len();
    let depth: Vec<usize> = (0..n).map(|b| (0..n).filter(|&a| a != b && inside(&loops[a], &loops[b])).count()).collect();
    let parent = |b: usize| (0..n).find(|&a| a != b && inside(&loops[a], &loops[b]) && depth[a] + 1 == depth[b]);
    let parents: Vec<Option<usize>> = (0..n).map(parent).collect();
    let mut regions = Vec::new();
    for i in (0..n).filter(|&i| depth[i].is_multiple_of(2)) {
        let holes = (0..n).filter(|&j| !depth[j].is_multiple_of(2) && parents[j] == Some(i)).map(|j| loops[j].ring.clone()).collect();
        let first = std::mem::take(&mut loops[i].name);
        regions.push(Region {
            outer: loops[i].ring.clone(),
            holes,
            caps: ["extrude.cap_start", "extrude.cap_end"].map(|t| TopoId::derive(t).id(feature).tag(&first).finish()),
        });
    }
    let [o, x, y, z] = s.frame;
    Ok(extrude(&regions, z0, z1)?.transformed(&Rigid::frame(o, x, y, z)))
}

/// A profile loop of a sketch and the name its region's caps take.
pub(crate) struct Found {
    pub ring: ProfileLoop,
    pub name: String,
}

/// Profile loops: lines and arcs joined at their points, each loop starting at its lowest point;
/// then circles, each a loop of three arcs from its own points at 0°, 120° and 240°.
pub(crate) fn profile_loops(feature: TopoId, s: &Placed) -> Result<Vec<Found>, BrepError> {
    // Curves at each point: (curve, the other end, the arc's centre if an arc and whether it runs
    // counterclockwise from this end).
    type Via<'a> = (&'a str, &'a str, Option<(&'a str, bool)>);
    let mut at: BTreeMap<&str, Vec<Via>> = BTreeMap::new();
    for (name, l) in s.lines.iter().filter(|(_, l)| !l.construction) {
        at.entry(&l.from).or_default().push((name, &l.to, None));
        at.entry(&l.to).or_default().push((name, &l.from, None));
    }
    for (name, a) in s.arcs.iter().filter(|(_, a)| !a.construction) {
        at.entry(&a.from).or_default().push((name, &a.to, Some((&a.center, true))));
        at.entry(&a.to).or_default().push((name, &a.from, Some((&a.center, false))));
    }
    let circles: Vec<(&String, &Circle)> = s.circles.iter().filter(|(_, c)| !c.construction).collect();
    if at.is_empty() && circles.is_empty() {
        return Err(BrepError::Sketch("the sketch has no profile lines, arcs or circles".into()));
    }
    if let Some((p, _)) = at.iter().find(|(_, ls)| ls.len() != 2) {
        return Err(BrepError::Sketch(format!("the profile is open or branches at point `{p}`")));
    }
    let point = |p: &str| s.solved.points[p];
    let vertex = |name: &str| [0, 1].map(|layer| TopoId::derive("extrude.v").id(feature).index(layer).tag(name).finish());
    let side = |name: &str| TopoId::derive("extrude.side").id(feature).tag(name).finish();
    let mut used: Vec<&str> = Vec::new();
    let mut loops = Vec::new();
    for (&start, curves) in &at {
        if curves.iter().all(|(c, _, _)| used.contains(c)) {
            continue;
        }
        let mut ring = ProfileLoop { points: Vec::new(), vertices: Vec::new(), sides: Vec::new(), arcs: Vec::new() };
        let (mut p, mut via) = (start, curves[0]);
        loop {
            used.push(via.0);
            ring.points.push(point(p));
            ring.vertices.push(vertex(p));
            ring.sides.push(side(via.0));
            ring.arcs.push(via.2.map(|(c, ccw)| ProfileArc { center: point(c), ccw }));
            p = via.1;
            if p == start {
                break;
            }
            match at[p].iter().find(|(c, _, _)| !used.contains(c)) {
                Some(&next) => via = next,
                None => return Err(BrepError::Sketch(format!("the profile does not close at point `{p}`"))),
            }
        }
        loops.push(Found { ring, name: start.to_string() });
    }
    for (name, c) in circles {
        let (centre, r) = (point(&c.center), 0.5 * diameter(c));
        let ids = |tag: &str, layer: Option<u64>, k: usize| {
            let h = TopoId::derive(tag).id(feature);
            let h = if let Some(l) = layer { h.index(l) } else { h };
            h.tag(name).index(k as u64).finish()
        };
        let at = |k: usize| {
            let (sn, cs) = libm::sincos(std::f64::consts::TAU * k as f64 / 3.0);
            Vec2::new(centre.x + r * cs, centre.y + r * sn)
        };
        let ring = ProfileLoop {
            points: (0..3).map(at).collect(),
            vertices: (0..3).map(|k| [0, 1].map(|layer| ids("extrude.circle.v", Some(layer), k))).collect(),
            sides: (0..3).map(|k| ids("extrude.circle.side", None, k)).collect(),
            arcs: vec![Some(ProfileArc { center: centre, ccw: true }); 3],
        };
        loops.push(Found { ring, name: format!("circle {name}") });
    }
    Ok(loops)
}
