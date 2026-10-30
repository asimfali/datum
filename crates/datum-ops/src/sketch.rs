//! Sketches placed in space, and extrusions of their profiles (M3, D53).

use std::borrow::Cow;
use std::collections::{BTreeMap, BTreeSet};

use datum_brep::{Body, BrepError, Curve, Cylinder, Development, Ix, ProfileArc, ProfileLoop, Region, Surface, TopoId, extrude};
use datum_math::{Rigid, Sign, Vec2, Vec3, orient2d};
use datum_sketch::{Arc, Circle, Constraint, Line, Sketch, Solved, Step, contours, solve};
use rayon::prelude::*;

use crate::copies::{Copies, moved, place_patterns};
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

impl SketchPlane {
    /// The axes a sketch on it has: origin, X, Y and the normal, X and Y unit and across it.
    pub fn axes(&self) -> Result<[Vec3; 4], BrepError> {
        frame(self.origin, self.x_axis, self.normal)
    }
}

/// The model's origin planes (D149, D150): XY, XZ seen from the front (its X along X, its Y
/// along Z) and YZ seen from the right.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum OriginPlane {
    #[default]
    Xy,
    Xz,
    Yz,
}

impl OriginPlane {
    pub const ALL: [Self; 3] = [Self::Xy, Self::Xz, Self::Yz];

    pub fn name(self) -> &'static str {
        match self {
            Self::Xy => "XY",
            Self::Xz => "XZ",
            Self::Yz => "YZ",
        }
    }

    pub fn plane(self) -> SketchPlane {
        let (x, normal) = match self {
            Self::Xy => (Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0)),
            Self::Xz => (Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, -1.0, 0.0)),
            Self::Yz => (Vec3::new(0.0, 1.0, 0.0), Vec3::new(1.0, 0.0, 0.0)),
        };
        SketchPlane { origin: Vec3::ZERO, x_axis: x, normal }
    }
}

/// The plane of plane face `id` of `body`, as a sketch on it lies: the world origin projected
/// onto it, X the world axis least along the normal, projected; on a top face sketch
/// coordinates are world X and Y. Not a plane face: `Unsupported`.
pub fn face_plane(body: &Body, id: &str) -> Result<SketchPlane, BrepError> {
    plane_of(body, find_face(body, id)?.0)
}

/// [`face_plane`] of face `f`, by index.
fn plane_of(body: &Body, f: usize) -> Result<SketchPlane, BrepError> {
    let Surface::Plane(plane) = body.faces()[f].surface else {
        return Err(BrepError::Unsupported(format!("face {:?} is not plane", body.faces()[f].id)));
    };
    let n = unit(plane.normal).ok_or_else(|| BrepError::Degenerate("the face has no normal".into()))?;
    let axes = [Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0)];
    let x = axes.into_iter().min_by(|a, b| a.dot(n).abs().total_cmp(&b.dot(n).abs())).unwrap_or(axes[0]);
    Ok(SketchPlane { origin: n * n.dot(plane.origin), x_axis: x, normal: n })
}

/// The name of the point every plane sketch has where the model's origin projects onto its
/// plane (D93): fixed, to dimension from, as Inventor projects the part's centre point. Not
/// part of the sketch's hash: the plane says where it is.
pub const ORIGIN: &str = "origin";

/// An edge or vertex of the input body projected into a sketch (D87), by ID. `at` keeps where
/// its points were when the sketch was drawn, in sketch coordinates: a vertex's point; a line's
/// start and end; an arc's centre, start and end. Sides (a point above a line, a circle inside or
/// outside another) are decided there, so a projection moved by an edit upstream does not flip the
/// sketch. Empty: drawn where the projection is now.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Projection {
    pub id: String,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub at: Vec<Vec2>,
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
    /// Where each projection's points stand now, in the order of [`Projection::at`].
    pub projected: BTreeMap<String, Vec<Vec2>>,
    /// The patterns, by name: their profiles are made with the sketch's own.
    pub patterns: Vec<Copies>,
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
pub(crate) fn unrolled(body: &Body, f: usize, c: &Cylinder, k: f64) -> Result<Developed, BrepError> {
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

/// The face of `body` with the ID written `id` (hex), and the ID. A face gone after a rebuild is
/// a lost reference.
pub(crate) fn find_face(body: &Body, id: &str) -> Result<(usize, TopoId), BrepError> {
    let id = u128::from_str_radix(id, 16).map(TopoId).map_err(|_| BrepError::Sketch(format!("`{id}` is not a face ID")))?;
    let f =
        body.faces().binary_search_by_key(&id, |f| f.id).map_err(|_| BrepError::LostReference(format!("face {id:?} is not in the input body")))?;
    Ok((f, id))
}

/// Whether `a` and `b` lie along one line, exactly: every 2 × 2 minor of the pair is zero.
fn parallel(a: Vec3, b: Vec3) -> bool {
    let minor = |i: usize, j: usize| {
        let (a, b) = (a.to_array(), b.to_array());
        orient2d(Vec2::new(0.0, 0.0), Vec2::new(a[i], a[j]), Vec2::new(b[i], b[j]))
    };
    [minor(0, 1), minor(1, 2), minor(0, 2)] == [Sign::Zero; 3]
}

/// `sketch` with edges and vertices of `body` projected onto its plane (D87): each entry of
/// `project` names an edge or vertex by ID. A vertex becomes a fixed point of that name. An edge
/// becomes a construction line or arc of that name between fixed points `<name>.start` (its
/// lower-ID vertex) and `<name>.end`, an arc around a fixed `<name>.center`. The sketch's own
/// constraints tie to them like to any point or line. `normal` is the sketch plane's normal as
/// given (not made unit), so an arc's axis is tested along it exactly: an arc seen at a slant
/// would be an ellipse, which a sketch does not have.
/// Also returns where each projection's points stand.
fn projected(
    sketch: &Sketch,
    body: &Body,
    frame: &[Vec3; 4],
    normal: Vec3,
    project: &BTreeMap<String, Projection>,
) -> Result<(Sketch, BTreeMap<String, Vec<Vec2>>), BrepError> {
    let mut s = sketch.clone();
    let mut now = BTreeMap::new();
    let [o, x, y, _] = *frame;
    let flat = |p: Vec3| Vec2::new((p - o).dot(x), (p - o).dot(y));
    for (name, proj) in project {
        let mut points: Vec<(String, Vec2)> = Vec::with_capacity(3);
        // The point drawn at its `at`, or where it is; fixed where it is.
        let mut add = |s: &mut Sketch, n: String, p: Vec3| -> Result<String, BrepError> {
            let p = flat(p);
            let drawn = match proj.at.len() {
                0 => p,
                _ => *proj.at.get(points.len()).ok_or_else(|| BrepError::Sketch(format!("projection `{name}`: `at` has too few points")))?,
            };
            if s.points.insert(n.clone(), drawn).is_some() {
                return Err(BrepError::Sketch(format!("the sketch already has a point `{n}`")));
            }
            s.constraints.push(Constraint::Fix { point: n.clone(), at: Some(p) });
            points.push((n.clone(), p));
            Ok(n)
        };
        let id = u128::from_str_radix(&proj.id, 16)
            .map(TopoId)
            .map_err(|_| BrepError::Sketch(format!("projection `{name}`: `{}` is not an ID", proj.id)))?;
        if let Ok(v) = body.vertices().binary_search_by_key(&id, |v| v.id) {
            add(&mut s, name.clone(), body.vertices()[v].point)?;
            check_at(name, proj, &points)?;
            now.insert(name.clone(), points.into_iter().map(|(_, p)| p).collect());
            continue;
        }
        let e = body
            .edges()
            .binary_search_by_key(&id, |e| e.id)
            .map_err(|_| BrepError::LostReference(format!("projection `{name}`: no edge or vertex {id:?} in the input body")))?;
        let edge = &body.edges()[e];
        let (a, b) = (body.half_edges()[edge.half as usize].origin, body.half_edges()[body.next(edge.half) as usize].origin);
        let (pa, pb) = (body.point(a), body.point(b));
        if s.lines.contains_key(name) || s.arcs.contains_key(name) || s.circles.contains_key(name) {
            return Err(BrepError::Sketch(format!("the sketch already has a curve `{name}`")));
        }
        match body.edge_curve(edge) {
            None => {
                if flat(pa) == flat(pb) {
                    return Err(BrepError::Degenerate(format!("projection `{name}`: the edge is across the sketch plane; project a vertex")));
                }
                let (from, to) = (add(&mut s, format!("{name}.start"), pa)?, add(&mut s, format!("{name}.end"), pb)?);
                s.lines.insert(name.clone(), Line { from, to, construction: true });
            }
            Some(Curve::Arc(arc)) if parallel(arc.axis, normal) => {
                let center = add(&mut s, format!("{name}.center"), arc.center)?;
                let (start, end) = (add(&mut s, format!("{name}.start"), pa)?, add(&mut s, format!("{name}.end"), pb)?);
                // The edge turns counterclockwise about its axis from its lower-ID vertex.
                let (from, to) = if arc.axis.dot(normal) > 0.0 { (start, end) } else { (end, start) };
                s.arcs.insert(name.clone(), Arc { center, from, to, construction: true });
            }
            Some(Curve::Arc(_)) => {
                return Err(BrepError::Unsupported(format!("projection `{name}`: the arc is not parallel to the sketch plane")));
            }
            Some(Curve::Rolled { .. }) => {
                return Err(BrepError::Unsupported(format!("projection `{name}`: an edge of a cut in a rolled sheet")));
            }
        }
        check_at(name, proj, &points)?;
        now.insert(name.clone(), points.into_iter().map(|(_, p)| p).collect());
    }
    Ok((s, now))
}

fn check_at(name: &str, proj: &Projection, points: &[(String, Vec2)]) -> Result<(), BrepError> {
    if proj.at.is_empty() || proj.at.len() == points.len() {
        Ok(())
    } else {
        Err(BrepError::Sketch(format!("projection `{name}`: `at` has {} points, the projection {}", proj.at.len(), points.len())))
    }
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
/// `k_factor` of the thickness. With `project`, edges and vertices of `body` are projected into
/// the sketch (see [`projected`]). A face, edge or vertex gone after a rebuild is a lost
/// reference.
pub fn place(
    sketch: &Sketch,
    plane: Option<&SketchPlane>,
    body: Option<&Body>,
    face: Option<&str>,
    k_factor: Option<f64>,
    project: &BTreeMap<String, Projection>,
) -> Result<Placed, BrepError> {
    let mut developed = None;
    // The plane's normal as given, for projections.
    let mut normal = Vec3::new(0.0, 0.0, 1.0);
    let face = match (body, face) {
        (Some(b), Some(f)) => Some((b, f)),
        (None, Some(_)) => return Err(BrepError::Sketch("a sketch on a face takes the body as its input and the face ID as `face`".into())),
        _ => None,
    };
    let (frame, on) = match (plane, face) {
        (Some(_), Some(_)) => return Err(BrepError::Sketch("give a plane or a face, not both".into())),
        (Some(p), None) => {
            normal = p.normal;
            (frame(p.origin, p.x_axis, p.normal)?, None)
        }
        (None, Some((body, id))) => {
            let (f, id) = find_face(body, id)?;
            match body.faces()[f].surface {
                Surface::Plane(_) if k_factor.is_some() => return Err(BrepError::Sketch("`k_factor` is for a sketch on a cylinder face".into())),
                Surface::Plane(raw) => {
                    let p = plane_of(body, f)?;
                    normal = raw.normal;
                    (frame(p.origin, p.x_axis, p.normal)?, Some(id))
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
    if sketch.points.contains_key(ORIGIN) {
        return Err(BrepError::Sketch(format!("`{ORIGIN}` is the model's origin projected into the sketch: name the point otherwise")));
    }
    // The model's origin on the plane; a sketch in a sheet's development has its own.
    let with_origin;
    let sketch = match &developed {
        Some(_) => sketch,
        None => {
            let [o, x, y, _] = frame;
            let mut s = sketch.clone();
            // `0 − …`: no negative zero for an origin on the plane's own.
            s.points.insert(ORIGIN.into(), Vec2::new(0.0 - o.dot(x), 0.0 - o.dot(y)));
            s.constraints.push(Constraint::Fix { point: ORIGIN.into(), at: None });
            with_origin = s;
            &with_origin
        }
    };
    let with_projections;
    let mut now = BTreeMap::new();
    let sketch = match body {
        _ if project.is_empty() => sketch,
        None => return Err(BrepError::Sketch("a projection takes the body as the sketch's input".into())),
        Some(_) if developed.is_some() => return Err(BrepError::Unsupported("projections into a sketch on a cylinder face".into())),
        Some(b) => {
            (with_projections, now) = projected(sketch, b, &frame, normal, project)?;
            &with_projections
        }
    };
    let solved = solve(sketch).map_err(|e| BrepError::Sketch(e.to_string()))?;
    let mut placed = Placed {
        face: on,
        projected: now,
        developed: developed.map(Box::new),
        frame,
        solved,
        lines: sketch.lines.clone(),
        arcs: sketch.arcs.clone(),
        circles: sketch.circles.clone(),
        patterns: Vec::new(),
        hash: [0; 32],
    };
    placed.patterns = place_patterns(sketch, &placed)?;
    let (solved, developed) = (&placed.solved, &placed.developed);
    let mut h = blake3::Hasher::new();
    let mut bytes = |b: &[u8]| {
        h.update(&(b.len() as u64).to_le_bytes());
        h.update(b);
    };
    for c in frame.iter().flat_map(|v| v.to_array()) {
        bytes(&c.to_bits().to_le_bytes());
    }
    for (name, p) in solved.points.iter().filter(|(n, _)| n.as_str() != ORIGIN) {
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
    for p in &placed.patterns {
        bytes(b"pattern");
        bytes(p.name.as_bytes());
        for c in &p.cell {
            bytes(c.as_bytes());
        }
        bytes(&[u8::from(p.source)]);
        // A path of profile lines bounds nothing (D157); one of construction lines changed
        // nothing, its sketch hashed as before.
        if !p.guide.is_empty() {
            bytes(b"guide");
            for c in &p.guide {
                bytes(c.as_bytes());
            }
        }
        for (k, d) in &p.at {
            bytes(&[k[0].to_le_bytes(), k[1].to_le_bytes(), d.by.x.to_bits().to_le_bytes(), d.by.y.to_bits().to_le_bytes()].concat());
            // A grid's copies are not turned: their bytes as before turns (D153).
            if d.turned() {
                bytes(&[d.turn.x.to_bits().to_le_bytes(), d.turn.y.to_bits().to_le_bytes()].concat());
            }
        }
    }
    if let Some(d) = developed {
        bytes(b"developed");
        let [o, a, s] = [d.dev.origin, d.dev.axis, d.dev.start].map(Vec3::to_array);
        for c in o.into_iter().chain(a).chain(s).chain([d.dev.neutral, d.radius, d.span]) {
            bytes(&c.to_bits().to_le_bytes());
        }
        bytes(&[u8::from(d.mirrored)]);
    }
    placed.hash = *h.finalize().as_bytes();
    Ok(placed)
}

/// The closed profiles of `s` swept from `z0` to `z1` along its normal. Profile lines and arcs
/// (not construction) must form closed loops that neither branch nor touch, and a circle is a
/// loop of its own; a loop inside another is a hole, one inside a hole an island. Side faces are
/// named by their lines and arcs, vertices by their points (§4.2), a circle's by its name:
/// renaming nothing keeps every ID through edits of the sketch.
/// How far along the normal of sketch `s` plane `p` lies: an extrusion's depth up to it (D150).
/// The plane must be parallel to the sketch, exactly, both normals made unit alike: planes offset
/// from one another and the face a sketch lies on are. A slanted end is not supported yet.
pub fn depth_to(s: &Placed, p: &SketchPlane) -> Result<f64, BrepError> {
    let n = unit(p.normal).ok_or_else(|| BrepError::Degenerate("the plane has no normal".into()))?;
    if !parallel(s.frame[3], n) {
        return Err(BrepError::Unsupported("an extrusion up to a plane not parallel to its sketch".into()));
    }
    Ok((p.origin - s.frame[0]).dot(s.frame[3]))
}

pub fn extrude_sketch(feature: TopoId, s: &Placed, z0: f64, z1: f64) -> Result<Body, BrepError> {
    if s.developed.is_some() {
        return Err(BrepError::Unsupported("an extrusion of a sketch on a cylinder face".into()));
    }
    let regions = profile_regions(feature, s)?;
    let [o, x, y, z] = s.frame;
    Ok(extrude(&regions, z0, z1)?.transformed(&Rigid::frame(o, x, y, z)))
}

/// The regions the closed profiles of `s` bound, in its plane: a loop inside another is a hole,
/// one inside a hole an island. Caps are named by the feature, and each of several regions' also
/// by the point its outer loop starts at.
pub fn profile_regions(feature: TopoId, s: &Placed) -> Result<Vec<Region>, BrepError> {
    let mut loops = profile_loops(feature, s)?;
    let n = loops.len();
    // Who holds whom: of loops whose boxes overlap, the sketch's own against each other and
    // against copies, and the loops of one copy among themselves (as its cell's lie in one
    // another); copies hold nothing else (`place_patterns`).
    let start = |k: usize| loops[k].copy.map(|c| k - c.base);
    let boxes: Vec<datum_brep::Box2> = loops.par_iter().map(|l| l.ring.bounds()).collect();
    let holds: Vec<(usize, usize)> = datum_brep::overlapping(&boxes)
        .into_par_iter()
        .filter(|&(a, b)| loops[a].copy.is_none() || loops[b].copy.is_none() || start(a) == start(b))
        .flat_map_iter(|(a, b)| {
            let inside = |a: usize, b: usize| loops[a].ring.winding(loops[b].ring.points[0]) != 0;
            [(a, b), (b, a)].into_iter().filter(move |&(h, l)| inside(h, l))
        })
        .collect();
    let mut holders: Vec<Vec<usize>> = vec![Vec::new(); n];
    for (h, l) in holds {
        holders[l].push(h);
    }
    let depth: Vec<usize> = holders.iter().map(Vec::len).collect();
    let parent: Vec<Option<usize>> = (0..n).map(|b| holders[b].iter().copied().find(|&a| depth[a] + 1 == depth[b])).collect();
    let mut holes: Vec<Vec<usize>> = vec![Vec::new(); n];
    for j in (0..n).filter(|&j| !depth[j].is_multiple_of(2)) {
        if let Some(i) = parent[j] {
            holes[i].push(j);
        }
    }
    let outer: Vec<usize> = (0..n).filter(|&i| depth[i].is_multiple_of(2)).collect();
    // The caps of a single region are the feature's own: a section of its outline replaced,
    // the first point among those removed, keeps them.
    let single = outer.len() == 1;
    let mut regions = Vec::with_capacity(outer.len());
    // The names of the loops of each pattern's cell, once a copy's caps need one.
    let mut cells: BTreeMap<usize, Vec<String>> = BTreeMap::new();
    for i in outer {
        let holes = holes[i].iter().map(|&j| loops[j].ring.clone()).collect();
        let first = match loops[i].copy {
            None => std::mem::take(&mut loops[i].name),
            Some(c) => {
                let names = match cells.entry(c.pattern) {
                    std::collections::btree_map::Entry::Occupied(e) => e.into_mut(),
                    std::collections::btree_map::Entry::Vacant(e) => {
                        e.insert(self::loops(feature, s, &|n| s.patterns[c.pattern].cell.contains(n), false)?.into_iter().map(|l| l.name).collect())
                    }
                };
                format!("{}[{},{}].{}", s.patterns[c.pattern].name, c.k[0], c.k[1], names[c.base])
            }
        };
        let cap = |t: &str| {
            let h = TopoId::derive(t).id(feature);
            if single { h.finish() } else { h.tag(&first).finish() }
        };
        regions.push(Region { outer: loops[i].ring.clone(), holes, caps: ["extrude.cap_start", "extrude.cap_end"].map(cap) });
    }
    Ok(regions)
}

/// A profile loop of a sketch and the name its region's caps take (a copy's is made only if its
/// caps need it: see [`Found::name`]); of a copy, its pattern, its place on the grid, and its place
/// among its cell's loops (they lie together among the loops, in the cell's order).
pub(crate) struct Found {
    pub ring: ProfileLoop,
    pub name: String,
    pub copy: Option<Copy>,
}

#[derive(Clone, Copy)]
pub(crate) struct Copy {
    pub pattern: usize,
    pub k: [i64; 2],
    pub base: usize,
}

/// Profile loops: the sketch's own (see [`loops`]), then each copy of each pattern, its cell's
/// loops moved.
pub(crate) fn profile_loops(feature: TopoId, s: &Placed) -> Result<Vec<Found>, BrepError> {
    let aside = crate::copies::aside(&s.patterns);
    let mut out = loops(feature, s, &|n| !aside.contains(&n.to_string()), false)?;
    for (pattern, p) in s.patterns.iter().enumerate() {
        let base = loops(feature, s, &|n| p.cell.contains(n), false)?;
        out.par_extend(p.at.par_iter().flat_map_iter(|&(k, d)| {
            let own = p.source && k == [0, 0];
            base.iter().enumerate().map(move |(b, l)| Found {
                ring: moved(&l.ring, d, feature, &p.name, k, own),
                name: String::new(),
                copy: Some(Copy { pattern, k, base: b }),
            })
        }));
    }
    if out.is_empty() {
        return Err(BrepError::Sketch("the sketch has no profile lines, arcs or circles".into()));
    }
    Ok(out)
}

/// The sketch with only the profiles an extrusion takes (D191): the closed loops a curve of which
/// `names` names, each with the loops right inside it (its holes); naming none, every closed loop.
/// Open chains are left out (in a skeleton sketch, the paths of sheet metal parts); a branch is
/// an error, as lines of a skeleton do not cross. A pattern stays when every curve of its cell
/// does.
pub fn chosen<'s>(feature: TopoId, s: &'s Placed, names: &[String]) -> Result<Cow<'s, Placed>, BrepError> {
    let keep = |_: &str, construction: bool| !construction;
    let found = contours(&s.lines, &s.arcs, &keep);
    if let Some(p) = found.branches.first() {
        return Err(BrepError::Sketch(format!("the profile branches at point `{p}`")));
    }
    let loops = rings(feature, s, &found.loops, &keep);
    // The curves of each loop, in the order of `loops`: the chains, then the circles.
    let circles = s.circles.iter().filter(|(_, c)| !c.construction).map(|(n, _)| vec![n.as_str()]);
    let curves: Vec<Vec<&str>> = found.loops.iter().map(|c| c.iter().map(|st| st.curve).collect()).chain(circles).collect();
    if loops.is_empty() {
        return Err(BrepError::Sketch(match found.ends.first() {
            Some(p) => format!("the sketch has no closed profile: it is open at point `{p}`"),
            None => "the sketch has no profile lines, arcs or circles".into(),
        }));
    }
    let picked: Vec<usize> = if names.is_empty() {
        (0..loops.len()).collect()
    } else {
        let on_path = |name: &str| found.paths.iter().flatten().any(|st| st.curve == name);
        names
            .iter()
            .map(|name| {
                curves.iter().position(|c| c.contains(&name.as_str())).ok_or_else(|| {
                    BrepError::Sketch(if on_path(name) {
                        format!("`{name}` is on an open chain: it bounds no profile")
                    } else {
                        format!("`{name}` is on no profile of the sketch")
                    })
                })
            })
            .collect::<Result<_, _>>()?
    };
    // Who holds whom; a loop's holes are held by it and by one loop fewer than it is.
    let n = loops.len();
    let holders: Vec<Vec<usize>> =
        (0..n).map(|b| (0..n).filter(|&a| a != b && loops[a].ring.winding(loops[b].ring.points[0]) != 0).collect()).collect();
    let mut taken = BTreeSet::new();
    for &a in &picked {
        taken.insert(a);
        taken.extend((0..n).filter(|&b| holders[b].contains(&a) && holders[b].len() == holders[a].len() + 1));
    }
    let kept: BTreeSet<&str> = taken.iter().flat_map(|&k| curves[k].iter().copied()).collect();
    let all = s.lines.iter().map(|(n, l)| (n, l.construction)).chain(s.arcs.iter().map(|(n, a)| (n, a.construction)));
    if all.chain(s.circles.iter().map(|(n, c)| (n, c.construction))).all(|(n, c)| c || kept.contains(n.as_str())) {
        return Ok(Cow::Borrowed(s));
    }
    let mut out = s.clone();
    out.lines.retain(|n, l| l.construction || kept.contains(n.as_str()));
    out.arcs.retain(|n, a| a.construction || kept.contains(n.as_str()));
    out.circles.retain(|n, c| c.construction || kept.contains(n.as_str()));
    out.patterns.retain(|p| p.cell.iter().all(|c| kept.contains(c.as_str())));
    Ok(Cow::Owned(out))
}

/// The closed loops of the curves `keep` keeps (construction ones too if `construction`), an
/// error if a curve is on none: lines and arcs joined at their points, each loop starting at its
/// lowest point (see [`contours`]); then circles, each a loop of three arcs from its own points at
/// 0°, 120° and 240°.
pub(crate) fn loops(feature: TopoId, s: &Placed, keep: &dyn Fn(&str) -> bool, construction: bool) -> Result<Vec<Found>, BrepError> {
    let keep = |name: &str, c: bool| keep(name) && (construction || !c);
    let found = contours(&s.lines, &s.arcs, &keep);
    if let Some(p) = found.odd() {
        return Err(BrepError::Sketch(format!("the profile is open or branches at point `{p}`")));
    }
    Ok(rings(feature, s, &found.loops, &keep))
}

/// The closed loops of the profile curves `keep` keeps, open chains left out: a line not joined
/// yet, a path, a sheet's profile bound nothing.
pub(crate) fn closed_loops(feature: TopoId, s: &Placed, keep: &dyn Fn(&str) -> bool) -> Vec<Found> {
    let keep = |name: &str, c: bool| keep(name) && !c;
    rings(feature, s, &contours(&s.lines, &s.arcs, &keep).loops, &keep)
}

/// Chains `chains` of `s` as profile loops, then the circles `keep` keeps.
fn rings(feature: TopoId, s: &Placed, chains: &[Vec<Step>], keep: &dyn Fn(&str, bool) -> bool) -> Vec<Found> {
    let point = |p: &str| s.solved.points[p];
    let vertex = |name: &str| [0, 1].map(|layer| TopoId::derive("extrude.v").id(feature).index(layer).tag(name).finish());
    let side = |name: &str| TopoId::derive("extrude.side").id(feature).tag(name).finish();
    let mut loops: Vec<Found> = chains
        .iter()
        .map(|chain| {
            let ring = ProfileLoop {
                points: chain.iter().map(|st| point(st.from)).collect(),
                vertices: chain.iter().map(|st| vertex(st.from)).collect(),
                sides: chain.iter().map(|st| side(st.curve)).collect(),
                arcs: chain.iter().map(|st| s.arcs.get(st.curve).map(|a| ProfileArc { center: point(&a.center), ccw: st.forward })).collect(),
            };
            Found { ring, name: chain[0].from.to_string(), copy: None }
        })
        .collect();
    let circles = s.circles.iter().filter(|(n, c)| keep(n, c.construction));
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
        loops.push(Found { ring, name: format!("circle {name}"), copy: None });
    }
    loops
}
