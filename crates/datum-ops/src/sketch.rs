//! Sketches placed in space, and extrusions of their profiles (M3, D53).

use std::borrow::Cow;
use std::collections::{BTreeMap, BTreeSet};

use datum_brep::{Body, BrepError, Curve, Cylinder, Development, Ix, ProfileArc, ProfileLoop, Region, Surface, TopoId, extrude};
use datum_math::{Rigid, Sign, Vec2, Vec3, orient2d, orient3d_coords};
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

/// Geometry of an input projected into a sketch (D87, D214, D216): an edge or vertex of a body by
/// `id`, the outline of its face by `face`, its section by the sketch plane (`section`), or a
/// point, line, arc or circle of a sketch by `element`, from the input named `from` (none: the
/// first). `at` keeps where its points were when the sketch was drawn, in sketch coordinates:
/// a point's; a line's start and end; an arc's centre, start and end; a circle's centre. Sides (a
/// point above a line, a circle inside or outside another) are decided there, so a projection
/// moved by an edit upstream does not flip the sketch. Empty: drawn where the projection is now;
/// a face or a section, of many curves, is always drawn so. `construction`: whether it bounds no
/// profile; none: an edge or an element is construction (projected to dimension and snap to), a
/// face or a section is not (projected to be a profile).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Projection {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub from: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub id: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub element: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub face: Option<String>,
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub section: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub construction: Option<bool>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub at: Vec<Vec2>,
}

impl Projection {
    /// Whether it is one edge, vertex or element, with `at`: not a face or a section.
    pub fn single(&self) -> bool {
        self.face.is_none() && !self.section
    }
}

/// What a projection takes its geometry from: an input of the sketch feature.
#[derive(Clone, Copy)]
pub enum Source<'a> {
    Body(&'a Body),
    Sketch(&'a Placed),
}

/// A point of projected geometry in the world, and what it is in its source (a body's vertex,
/// a sketch's point), if anything: curves ending at one such point share it in the sketch.
type End = (Vec3, Option<String>);

/// Projected geometry in the world, before it is laid on the sketch plane.
enum Shape {
    Point(End),
    /// Start and end.
    Line(End, End),
    /// Centre, start, end, and the axis it turns counterclockwise about from start to end.
    Arc(End, End, End, Vec3),
    /// Centre, axis, diameter.
    Circle(End, Vec3, f64),
    /// A cylinder about `axis` of radius `r` cut by the sketch plane (D219): around its point
    /// on the plane, from start to end counterclockwise about `axis`, or whole.
    Cut(End, Option<(End, End)>, Vec3, f64),
}

/// An ellipse a projection made (D218): a circle or an arc seen at a slant. Its points are
/// fixed: around point `center`, `major` its half major axis as a vector in sketch coordinates,
/// `minor` its half minor axis (along `major` turned a quarter counterclockwise); an arc of it
/// runs counterclockwise from point `ends[0]` to `ends[1]`, none: whole. Not a curve of the
/// solver: points are dimensioned, nothing is put on it; it bounds no profile yet.
#[derive(Clone, Debug, PartialEq)]
pub struct Ellipse {
    pub center: String,
    pub major: Vec2,
    pub minor: f64,
    pub ends: Option<[String; 2]>,
    pub construction: bool,
}

impl Ellipse {
    /// Its point at parameter `t` (radians from the major axis), given its centre.
    pub fn at(&self, center: Vec2, t: f64) -> Vec2 {
        let (s, c) = libm::sincos(t);
        let k = self.minor / (self.major.x * self.major.x + self.major.y * self.major.y).sqrt();
        Vec2::new(center.x + self.major.x * c - self.major.y * k * s, center.y + self.major.y * c + self.major.x * k * s)
    }

    /// The parameter of `p`, a point of it, given its centre.
    pub fn parameter(&self, center: Vec2, p: Vec2) -> f64 {
        let d = Vec2::new(p.x - center.x, p.y - center.y);
        let len2 = self.major.x * self.major.x + self.major.y * self.major.y;
        let along = (d.x * self.major.x + d.y * self.major.y) / len2;
        let across = (d.y * self.major.x - d.x * self.major.y) / (len2.sqrt() * self.minor);
        libm::atan2(across, along)
    }
}

/// A solved sketch in its place: the value of a sketch feature.
#[derive(Clone, Debug, PartialEq)]
pub struct Placed {
    /// Origin and the X, Y, Z axes of the sketch plane, Z its normal. On a cylinder face, the
    /// plane touching the face at the sketch's origin; the sketch lies on the face (`developed`).
    pub frame: [Vec3; 4],
    /// On a cylinder face, where the sketch lies (boxed: most sketches are on planes).
    pub developed: Option<Box<Developed>>,
    /// On a plane face of sheet metal, the K-factor its bends are laid flat at: a cut across a
    /// bend goes on through the blank (D197).
    pub k_factor: Option<f64>,
    pub solved: Solved,
    pub lines: BTreeMap<String, Line>,
    pub arcs: BTreeMap<String, Arc>,
    /// Circles, each diameter a number.
    pub circles: BTreeMap<String, Circle>,
    /// Ellipses projections made (D218).
    pub ellipses: BTreeMap<String, Ellipse>,
    /// The pairs of curves the sketch makes tangent, each in name order: a sheet's path goes
    /// smoothly from one to the other, a fact the sketch states rather than one measured.
    pub tangent: BTreeSet<[String; 2]>,
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
/// facing the other way, on the side of the body; of the pieces a cut divided it into (one
/// surface, bit for bit, D201), the first. None if there is none, or two cylinders at the same
/// distance.
pub(crate) fn opposite_cylinder(body: &Body, f: usize) -> Option<(Ix, Cylinder)> {
    let Surface::Cylinder(c) = body.faces()[f].surface else { return None };
    let mut found: Vec<(Ix, Cylinder)> = (0..body.faces().len())
        .filter_map(|g| match body.faces()[g].surface {
            Surface::Cylinder(o) if g != f && coaxial(&o, &c) && o.convex != c.convex && (o.radius < c.radius) == c.convex => Some((g as Ix, o)),
            _ => None,
        })
        .collect();
    found.sort_by(|a, b| (a.1.radius - c.radius).abs().total_cmp(&(b.1.radius - c.radius).abs()));
    let first = *found.first()?;
    found.iter().take_while(|o| o.1.radius == first.1.radius).all(|o| o.1 == first.1).then_some(first)
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
            // A trim's edge (D211) turns about the axis as the development's `u` runs.
            Some(Curve::Rolled { piece, .. }) if piece.wave.is_some() => {
                let piece = if edge.half == h { piece } else { piece.reversed() };
                turn += (piece.to.x - piece.from.x) / piece.dev.neutral * piece.dev.axis.dot(axis).signum();
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
pub fn parallel(a: Vec3, b: Vec3) -> bool {
    let minor = |i: usize, j: usize| {
        let (a, b) = (a.to_array(), b.to_array());
        orient2d(Vec2::new(0.0, 0.0), Vec2::new(a[i], a[j]), Vec2::new(b[i], b[j]))
    };
    [minor(0, 1), minor(1, 2), minor(0, 2)] == [Sign::Zero; 3]
}

/// `sketch` with the geometry `project` names projected onto its plane (D87, D214), each entry's
/// from the input `source` gives for its `from`. A vertex or point becomes a fixed point of the
/// entry's name; an edge or line, a line of that name between fixed points `<name>.start` (a
/// body edge's lower-ID vertex) and `<name>.end`, or a fixed point `<name>` if it runs across the
/// plane; an arc, an arc around a fixed `<name>.center`; a circle, a circle of fixed diameter
/// around it. The sketch's own constraints tie to them like to any point or line. Curves that
/// meet at one point of their source (a vertex, a sketch's point) meet in the sketch: they all
/// end at the point of the first projection, in name order, that has it, so projected edges of
/// a face bound a profile; each projection's own points stay, fixed there too. `normal` is the
/// sketch plane's normal as given (not made unit), so an arc's axis is tested along it exactly:
/// an arc seen at a slant would be an ellipse, which a sketch does not have.
/// Also returns where each projection's points stand.
fn projected<'a>(
    sketch: &Sketch,
    frame: &[Vec3; 4],
    normal: Vec3,
    project: &BTreeMap<String, Projection>,
    source: &dyn Fn(Option<&str>) -> Result<Source<'a>, BrepError>,
) -> Result<Projected, BrepError> {
    let mut s = sketch.clone();
    let mut now = BTreeMap::new();
    let mut ellipses = BTreeMap::new();
    let [o, x, y, z] = *frame;
    let flat = |p: Vec3| Vec2::new((p - o).dot(x), (p - o).dot(y));
    // The sketch's point for each point of a source, by input and what it is there.
    let mut shared: BTreeMap<(Option<&str>, String), String> = BTreeMap::new();
    for (name, proj) in project {
        let one = |shape: Shape| vec![(name.clone(), shape)];
        let shapes = match (source(proj.from.as_deref())?, &proj.id, &proj.element, &proj.face, proj.section) {
            (Source::Body(b), Some(id), None, None, false) => one(body_shape(name, b, id)?),
            (Source::Body(b), None, None, Some(f), false) => face_shapes(name, b, f)?,
            (Source::Body(b), None, None, None, true) => section_shapes(name, b, frame)?,
            (Source::Sketch(p), None, Some(e), None, false) => one(sketch_shape(name, p, e)?),
            (Source::Body(_), ..) => {
                return Err(BrepError::Sketch(format!(
                    "projection `{name}`: a body's edge or vertex is named by `id`, a face by `face`, or its section is `section`"
                )));
            }
            (Source::Sketch(_), ..) => return Err(BrepError::Sketch(format!("projection `{name}`: a sketch's element is named by `element`"))),
        };
        if !proj.single() && !proj.at.is_empty() {
            return Err(BrepError::Sketch(format!("projection `{name}`: a face or a section is drawn where it is, without `at`")));
        }
        let construction = proj.construction.unwrap_or(proj.single());
        let mut points: Vec<Vec2> = Vec::with_capacity(3);
        // The point drawn at its `at`, or where it is; fixed where it is.
        let mut add = |s: &mut Sketch, n: String, (p, of): End| -> Result<String, BrepError> {
            let p = flat(p);
            let drawn = match proj.at.len() {
                0 => p,
                _ => *proj.at.get(points.len()).ok_or_else(|| BrepError::Sketch(format!("projection `{name}`: `at` has too few points")))?,
            };
            if s.points.insert(n.clone(), drawn).is_some() {
                return Err(BrepError::Sketch(format!("the sketch already has a point `{n}`")));
            }
            s.constraints.push(Constraint::Fix { point: n.clone(), at: Some(p) });
            points.push(p);
            Ok(match of {
                Some(of) => shared.entry((proj.from.as_deref(), of)).or_insert(n).clone(),
                None => n,
            })
        };
        let edge_on = || BrepError::Unsupported(format!("projection `{name}`: an arc or circle seen edge on falls on a line (later)"));
        for (name, shape) in shapes {
            if s.lines.contains_key(&name) || s.arcs.contains_key(&name) || s.circles.contains_key(&name) {
                return Err(BrepError::Sketch(format!("the sketch already has a curve `{name}`")));
            }
            match shape {
                Shape::Point(p) => {
                    add(&mut s, name.clone(), p)?;
                }
                // Seen end on, a line is a point.
                Shape::Line(a, b) if flat(a.0) == flat(b.0) => {
                    add(&mut s, name.clone(), a)?;
                }
                Shape::Line(a, b) => {
                    let (from, to) = (add(&mut s, format!("{name}.start"), a)?, add(&mut s, format!("{name}.end"), b)?);
                    s.lines.insert(name.clone(), Line { from, to, construction });
                }
                // At a slant, an ellipse (D218).
                Shape::Arc(c, a, b, axis) if !parallel(axis, normal) => {
                    let r = (a.0 - c.0).dot(a.0 - c.0).sqrt();
                    let (major, minor) = slant(axis, z, x, y).ok_or_else(edge_on)?;
                    let center = add(&mut s, format!("{name}.center"), c)?;
                    let (start, end) = (add(&mut s, format!("{name}.start"), a)?, add(&mut s, format!("{name}.end"), b)?);
                    let ends = if axis.dot(z) > 0.0 { [start, end] } else { [end, start] };
                    ellipses.insert(
                        name.clone(),
                        Ellipse { center, major: Vec2::new(major.x * r, major.y * r), minor: minor * r, ends: Some(ends), construction },
                    );
                }
                Shape::Circle(c, axis, d) if !parallel(axis, normal) => {
                    let (major, minor) = slant(axis, z, x, y).ok_or_else(edge_on)?;
                    let center = add(&mut s, format!("{name}.center"), c)?;
                    let r = 0.5 * d;
                    ellipses.insert(
                        name.clone(),
                        Ellipse { center, major: Vec2::new(major.x * r, major.y * r), minor: minor * r, ends: None, construction },
                    );
                }
                // Cut square to its axis, a cylinder is a circle; at a slant, an ellipse whose
                // half axes are the radius and the radius over the slant's cosine (D219).
                Shape::Cut(c, ends, axis, r) if parallel(axis, normal) => {
                    let center = add(&mut s, format!("{name}.center"), c)?;
                    match ends {
                        Some((a, b)) => {
                            let (start, end) = (add(&mut s, format!("{name}.start"), a)?, add(&mut s, format!("{name}.end"), b)?);
                            let (from, to) = if axis.dot(normal) > 0.0 { (start, end) } else { (end, start) };
                            s.arcs.insert(name.clone(), Arc { center, from, to, construction });
                        }
                        None => {
                            s.circles.insert(name.clone(), Circle { center, diameter: (2.0 * r).into(), construction });
                        }
                    }
                }
                Shape::Cut(c, ends, axis, r) => {
                    let a = axis * (1.0 / axis.dot(axis).sqrt());
                    let across = a.cross(z);
                    let v = across * (1.0 / across.dot(across).sqrt());
                    let u = v.cross(a);
                    // The cylinder's point at turn φ about `a` from `u` lies on the plane at
                    // `c + r·cos φ·major + r·sin φ·v`.
                    let major = (u - a * (u.dot(z) / a.dot(z))) * r;
                    let center = add(&mut s, format!("{name}.center"), c)?;
                    let ends = match ends {
                        Some((p, q)) => {
                            let (start, end) = (add(&mut s, format!("{name}.start"), p)?, add(&mut s, format!("{name}.end"), q)?);
                            // Turning about `a` turns counterclockwise in the sketch, or the other way.
                            Some(if major.cross(v).dot(z) > 0.0 { [start, end] } else { [end, start] })
                        }
                        None => None,
                    };
                    ellipses.insert(name.clone(), Ellipse { center, major: Vec2::new(major.dot(x), major.dot(y)), minor: r, ends, construction });
                }
                Shape::Arc(c, a, b, axis) => {
                    let center = add(&mut s, format!("{name}.center"), c)?;
                    let (start, end) = (add(&mut s, format!("{name}.start"), a)?, add(&mut s, format!("{name}.end"), b)?);
                    let (from, to) = if axis.dot(normal) > 0.0 { (start, end) } else { (end, start) };
                    s.arcs.insert(name.clone(), Arc { center, from, to, construction });
                }
                Shape::Circle(c, _, d) => {
                    let center = add(&mut s, format!("{name}.center"), c)?;
                    s.circles.insert(name.clone(), Circle { center, diameter: d.into(), construction });
                }
            }
        }
        if !proj.at.is_empty() && proj.at.len() != points.len() {
            return Err(BrepError::Sketch(format!("projection `{name}`: `at` has {} points, the projection {}", proj.at.len(), points.len())));
        }
        if proj.single() {
            now.insert(name.clone(), points);
        }
    }
    Ok((s, now, ellipses))
}

/// A sketch with its projections: the sketch with their points and curves, where each single
/// projection's points stand, and the ellipses they made.
type Projected = (Sketch, BTreeMap<String, Vec<Vec2>>, BTreeMap<String, Ellipse>);

/// How a circle about `axis` falls on the plane of unit normal `z` and axes `x`, `y`: the unit
/// direction of its major axis in sketch coordinates (the diameter that stays whole, along the
/// line its plane meets the sketch's) and how much of its radius is left across it, `|axis·z|`
/// of a unit axis. None seen edge on: it falls on a line.
fn slant(axis: Vec3, z: Vec3, x: Vec3, y: Vec3) -> Option<(Vec2, f64)> {
    let axis = axis * (1.0 / axis.dot(axis).sqrt());
    let along = axis.cross(z);
    let len = along.dot(along).sqrt();
    let minor = axis.dot(z).abs();
    (minor > 0.0 && len > 0.0).then(|| (Vec2::new(along.dot(x) / len, along.dot(y) / len), minor))
}

/// The edge or vertex with ID `id` (hex) of `body`, for projection `name`.
fn body_shape(name: &str, body: &Body, id: &str) -> Result<Shape, BrepError> {
    let id = u128::from_str_radix(id, 16).map(TopoId).map_err(|_| BrepError::Sketch(format!("projection `{name}`: `{id}` is not an ID")))?;
    let vertex = |v: Ix| (body.point(v), Some(format!("{:?}", body.vertices()[v as usize].id)));
    if let Ok(v) = body.vertices().binary_search_by_key(&id, |v| v.id) {
        return Ok(Shape::Point(vertex(v as Ix)));
    }
    let e = body
        .edges()
        .binary_search_by_key(&id, |e| e.id)
        .map_err(|_| BrepError::LostReference(format!("projection `{name}`: no edge or vertex {id:?} in the input body")))?;
    edge_shape(name, body, e)
}

/// Edge `e` of `body`, for projection `name`.
fn edge_shape(name: &str, body: &Body, e: usize) -> Result<Shape, BrepError> {
    let vertex = |v: Ix| (body.point(v), Some(format!("{:?}", body.vertices()[v as usize].id)));
    let edge = &body.edges()[e];
    let (a, b) = (body.half_edges()[edge.half as usize].origin, body.half_edges()[body.next(edge.half) as usize].origin);
    let (pa, pb) = (vertex(a), vertex(b));
    match body.edge_curve(edge) {
        None => Ok(Shape::Line(pa, pb)),
        // The edge turns counterclockwise about its axis from its lower-ID vertex.
        Some(Curve::Arc(arc)) => Ok(Shape::Arc((arc.center, None), pa, pb, arc.axis)),
        Some(Curve::Rolled { .. }) => Err(BrepError::Unsupported(format!("projection `{name}`: an edge of a cut in a rolled sheet"))),
    }
}

/// The short name of the curve element `id` makes of projection `name`: `<name>.<the ID's first
/// 8 hex digits>`, as stable as the ID (D216). Numbers in order would shift when the face gains
/// a hole, and a dimension on one would move to another.
fn part_name(name: &str, id: TopoId) -> String {
    let hex = format!("{id:?}");
    format!("{name}.{}", &hex[..hex.len().min(8)])
}

/// Two curves of one projection named alike: refused rather than one taken for the other.
fn named_once(name: &str, parts: Vec<(String, Shape)>) -> Result<Vec<(String, Shape)>, BrepError> {
    let mut names: Vec<&str> = parts.iter().map(|(n, _)| n.as_str()).collect();
    names.sort_unstable();
    match names.windows(2).find(|w| w[0] == w[1]) {
        Some(w) => Err(BrepError::Degenerate(format!("projection `{name}`: two of its curves are named `{}`", w[0]))),
        None => Ok(parts),
    }
}

/// The edges of face `id` (hex) of `body`, each a curve `<name>.<its ID>` (see [`part_name`]).
fn face_shapes(name: &str, body: &Body, id: &str) -> Result<Vec<(String, Shape)>, BrepError> {
    let (f, _) = find_face(body, id)?;
    let mut edges: Vec<usize> =
        body.face_loops(f as Ix).flat_map(|l| body.loop_half_edges(l)).map(|h| body.half_edges()[h as usize].edge as usize).collect();
    edges.sort_unstable();
    edges.dedup();
    let parts = edges.into_iter().map(|e| Ok((part_name(name, body.edges()[e].id), edge_shape(name, body, e)?))).collect::<Result<_, BrepError>>()?;
    named_once(name, parts)
}

/// The section of `body` by the sketch plane (`frame`): where it crosses each plane face, a
/// line `<name>.<the face's ID>` (`.1`, `.2`… along it if it crosses the face more than once).
/// The sides of every vertex are decided exactly; a crossing point lies on its edge, one point
/// for both faces of it. The plane through a vertex, across an arc, or crossing a curved face
/// (a circle or an ellipse there) is refused for now.
fn section_shapes(name: &str, body: &Body, frame: &[Vec3; 4]) -> Result<Vec<(String, Shape)>, BrepError> {
    let parts = section_parts(name, body, frame)?;
    if parts.is_empty() {
        return Err(BrepError::Degenerate(format!("projection `{name}`: the sketch plane does not cross the body")));
    }
    named_once(name, parts)
}

/// Whether the plane of `frame` crosses `body` so that its section can be projected (see
/// [`section_shapes`]): false if it misses the body; why not, if it cannot be.
pub fn crosses(body: &Body, frame: &[Vec3; 4]) -> Result<bool, BrepError> {
    section_parts("section", body, frame).map(|p| !p.is_empty())
}

fn section_parts(name: &str, body: &Body, frame: &[Vec3; 4]) -> Result<Vec<(String, Shape)>, BrepError> {
    let [o, x, y, z] = *frame;
    let side = |q: Vec3| orient3d_coords(o, o + x, o + y, q);
    let signs: Vec<Sign> = body.vertices().iter().map(|v| side(v.point)).collect();
    if signs.contains(&Sign::Zero) {
        return Err(BrepError::Degenerate(format!(
            "projection `{name}`: the sketch plane passes through a vertex of the body: project the face, or move the plane"
        )));
    }
    let unsupported = |m: &str| BrepError::Unsupported(format!("projection `{name}`: {m}"));
    let height = |q: Vec3| (q - o).dot(z);
    // Where the edge crosses the plane, if it does: from its own ends, the same for both its faces.
    let crossing = |e: usize| -> Result<Option<End>, BrepError> {
        let edge = &body.edges()[e];
        let (a, b) = (body.half_edges()[edge.half as usize].origin, body.half_edges()[body.next(edge.half) as usize].origin);
        if signs[a as usize] == signs[b as usize] {
            return Ok(None);
        }
        if body.edge_curve(edge).is_some() {
            return Err(unsupported("the sketch plane crosses an arc of the body (a section through a curve: later)"));
        }
        let (pa, pb) = (body.point(a), body.point(b));
        let (ha, hb) = (height(pa), height(pb));
        Ok(Some((pa + (pb - pa) * (ha / (ha - hb)), Some(format!("cross {:?}", edge.id)))))
    };
    let mut parts = Vec::new();
    for (f, face) in body.faces().iter().enumerate() {
        let edges: Vec<usize> =
            body.face_loops(f as Ix).flat_map(|l| body.loop_half_edges(l)).map(|h| body.half_edges()[h as usize].edge as usize).collect();
        let crossed = edges.iter().any(|&e| {
            let edge = &body.edges()[e];
            signs[body.half_edges()[edge.half as usize].origin as usize] != signs[body.half_edges()[body.next(edge.half) as usize].origin as usize]
        });
        // An arc across the plane bulges between its ends: refused if its circle may reach the
        // plane, which it does within its radius times the sine of its slant. One parallel to
        // the plane stays at its centre's height, on the side its ends are.
        let reaches = edges.iter().any(|&e| match body.edge_curve(&body.edges()[e]) {
            Some(Curve::Arc(a)) => {
                let slant = (a.axis * (1.0 / a.axis.dot(a.axis).sqrt())).cross(z);
                height(a.center).abs() <= a.radius * slant.dot(slant).sqrt()
            }
            Some(Curve::Rolled { .. }) => true,
            None => false,
        });
        if reaches {
            return Err(unsupported("the sketch plane may cross an arc of the body: through the end of a cylinder (later)"));
        }
        let plane = match face.surface {
            Surface::Plane(plane) => plane,
            Surface::Cylinder(cyl) if crossed => {
                let mut ends: Vec<End> = Vec::new();
                for &e in &edges {
                    if let Some(c) = crossing(e)?.filter(|c| !ends.iter().any(|d| d.1 == c.1)) {
                        ends.push(c);
                    }
                }
                parts.push((part_name(name, face.id), cylinder_cut(name, body, &cyl, &edges, ends, frame)?));
                continue;
            }
            _ if crossed => return Err(unsupported("the sketch plane crosses a cone or a cut's wall (later)")),
            _ => continue,
        };
        if !crossed {
            continue;
        }
        let mut ends: Vec<End> = Vec::new();
        for &e in &edges {
            if let Some(c) = crossing(e)? {
                ends.push(c);
            }
        }
        // Along the line the face meets the plane in: in, out, in, out.
        let along = plane.normal.cross(z);
        ends.sort_by(|a, b| a.0.dot(along).total_cmp(&b.0.dot(along)));
        if !ends.len().is_multiple_of(2) || ends.windows(2).any(|w| w[0].0.dot(along) == w[1].0.dot(along)) {
            return Err(BrepError::Degenerate(format!("projection `{name}`: the sketch plane meets a face of the body along its edge")));
        }
        let base = part_name(name, face.id);
        let many = ends.len() > 2;
        for (k, pair) in ends.chunks(2).enumerate() {
            let n = if many { format!("{base}.{}", k + 1) } else { base.clone() };
            parts.push((n, Shape::Line(pair[0].clone(), pair[1].clone())));
        }
    }
    Ok(parts)
}

/// Where the sketch plane (`frame`) cuts cylinder face `cyl` of `body`, its edges `edges`, across
/// its straight edges at `ends` (D219): an arc between them, turning as an arc edge of the face
/// does (its span is the face's), or the whole turn if the face closes round. Refused along the
/// axis (lines there, later) and across the face more than once.
fn cylinder_cut(name: &str, body: &Body, cyl: &Cylinder, edges: &[usize], ends: Vec<End>, frame: &[Vec3; 4]) -> Result<Shape, BrepError> {
    let [o, _, _, z] = *frame;
    let unsupported = |m: &str| BrepError::Unsupported(format!("projection `{name}`: {m}"));
    let a = cyl.axis * (1.0 / cyl.axis.dot(cyl.axis).sqrt());
    if a.dot(z) == 0.0 {
        return Err(unsupported("the sketch plane runs along a cylinder's axis: its section is lines (later)"));
    }
    let c = (cyl.origin - a * ((cyl.origin - o).dot(z) / a.dot(z)), None);
    match ends.len() {
        0 | 1 => Ok(Shape::Cut(c, None, a, cyl.radius)),
        2 => {
            // An arc edge of the face: from its first vertex counterclockwise about its axis.
            let (e, arc) = edges
                .iter()
                .find_map(|&e| match body.edge_curve(&body.edges()[e]) {
                    Some(Curve::Arc(arc)) => Some((e, arc)),
                    _ => None,
                })
                .ok_or_else(|| unsupported("a cylinder face with no arc edge"))?;
            let n = arc.axis * (1.0 / arc.axis.dot(arc.axis).sqrt());
            let from = body.point(body.half_edges()[body.edges()[e].half as usize].origin) - arc.center;
            let u = from - n * from.dot(n);
            let (u, w) = (u * (1.0 / u.dot(u).sqrt()), n.cross(u * (1.0 / u.dot(u).sqrt())));
            let turn = |p: Vec3| libm::atan2((p - arc.center).dot(w), (p - arc.center).dot(u)).abs();
            let (start, end) =
                if turn(ends[0].0) <= turn(ends[1].0) { (ends[0].clone(), ends[1].clone()) } else { (ends[1].clone(), ends[0].clone()) };
            Ok(Shape::Cut(c, Some((start, end)), n, cyl.radius))
        }
        _ => Err(unsupported("the sketch plane crosses a cylinder face more than once")),
    }
}

/// The point, line, arc or circle named `element` of the placed sketch `p`, for projection `name`.
fn sketch_shape(name: &str, p: &Placed, element: &str) -> Result<Shape, BrepError> {
    if p.developed.is_some() {
        return Err(BrepError::Unsupported(format!("projection `{name}`: from a sketch on a cylinder face")));
    }
    let [o, x, y, z] = p.frame;
    let point = |n: &str| p.solved.points.get(n).map(|q| (o + x * q.x + y * q.y, Some(n.to_string())));
    let lost = || BrepError::LostReference(format!("projection `{name}`: the sketch has no point, line, arc or circle `{element}`"));
    if let Some(q) = point(element) {
        return Ok(Shape::Point(q));
    }
    let at = |n: &str| point(n).ok_or_else(lost);
    if let Some(l) = p.lines.get(element) {
        return Ok(Shape::Line(at(&l.from)?, at(&l.to)?));
    }
    if let Some(a) = p.arcs.get(element) {
        return Ok(Shape::Arc(at(&a.center)?, at(&a.from)?, at(&a.to)?, z));
    }
    match p.circles.get(element) {
        Some(c) => Ok(Shape::Circle(at(&c.center)?, z, diameter(c))),
        None => Err(lost()),
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
/// `k_factor` of the thickness; on a plane face of sheet metal, `k_factor` lays its bends flat
/// for a cut across them. With `project`, geometry of the inputs `source` gives by name is
/// projected into the sketch (see [`projected`]). A face, edge, vertex or element gone after a
/// rebuild is a lost reference.
pub fn place<'a>(
    sketch: &Sketch,
    plane: Option<&SketchPlane>,
    body: Option<&Body>,
    face: Option<&str>,
    k_factor: Option<f64>,
    project: &BTreeMap<String, Projection>,
    source: &dyn Fn(Option<&str>) -> Result<Source<'a>, BrepError>,
) -> Result<Placed, BrepError> {
    let mut developed = None;
    // The plane's normal as given, for projections.
    let mut normal = Vec3::new(0.0, 0.0, 1.0);
    let face = match (body, face) {
        (Some(b), Some(f)) => Some((b, f)),
        (None, Some(_)) => return Err(BrepError::Sketch("a sketch on a face takes the body as its input and the face ID as `face`".into())),
        _ => None,
    };
    if k_factor.is_some() && face.is_none() {
        return Err(BrepError::Sketch("`k_factor` is for a sketch on a face of sheet metal".into()));
    }
    let (frame, on) = match (plane, face) {
        (Some(_), Some(_)) => return Err(BrepError::Sketch("give a plane or a face, not both".into())),
        (Some(p), None) => {
            normal = p.normal;
            (frame(p.origin, p.x_axis, p.normal)?, None)
        }
        (None, Some((body, id))) => {
            let (f, id) = find_face(body, id)?;
            match body.faces()[f].surface {
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
                Surface::Cone(_) => return Err(BrepError::Unsupported("a sketch on a cone face".into())),
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
    let mut ellipses = BTreeMap::new();
    let sketch = match developed {
        _ if project.is_empty() => sketch,
        Some(_) => return Err(BrepError::Unsupported("projections into a sketch on a cylinder face".into())),
        None => {
            (with_projections, now, ellipses) = projected(sketch, &frame, normal, project, source)?;
            &with_projections
        }
    };
    let solved = solve(sketch).map_err(|e| BrepError::Sketch(e.to_string()))?;
    let mut placed = Placed {
        face: on,
        projected: now,
        k_factor: if developed.is_some() { None } else { k_factor },
        developed: developed.map(Box::new),
        frame,
        solved,
        lines: sketch.lines.clone(),
        arcs: sketch.arcs.clone(),
        circles: sketch.circles.clone(),
        ellipses,
        tangent: sketch
            .constraints
            .iter()
            .filter_map(|c| match c {
                Constraint::Tangent { curves: [a, b] } => Some(if a <= b { [a.clone(), b.clone()] } else { [b.clone(), a.clone()] }),
                _ => None,
            })
            .collect(),
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
    // Only a sketch with ellipses hashes them (D218): the rest keep their hashes.
    for (name, e) in &placed.ellipses {
        bytes(b"ellipse");
        bytes(name.as_bytes());
        bytes(e.center.as_bytes());
        for c in [e.major.x, e.major.y, e.minor] {
            bytes(&c.to_bits().to_le_bytes());
        }
        for end in e.ends.iter().flatten() {
            bytes(end.as_bytes());
        }
        bytes(&[u8::from(e.construction)]);
    }
    // Only a sketch with tangents hashes them: the rest keep their hashes.
    for [a, b] in &placed.tangent {
        bytes(b"tangent");
        bytes(a.as_bytes());
        bytes(b.as_bytes());
    }
    if let Some(k) = placed.k_factor {
        bytes(b"k_factor");
        bytes(&k.to_bits().to_le_bytes());
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
    let (found, loops, curves) = named_loops(feature, s)?;
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

/// The chains of the profile curves of `s`, its closed loops (the chains', then the circles') and
/// the curves of each; a branch is an error.
type Named<'s> = (datum_sketch::Contours<'s>, Vec<Found>, Vec<Vec<&'s str>>);

fn named_loops(feature: TopoId, s: &Placed) -> Result<Named<'_>, BrepError> {
    no_ellipse(s, &|_| true)?;
    let keep = |_: &str, construction: bool| !construction;
    let found = contours(&s.lines, &s.arcs, &keep);
    if let Some(p) = found.branches.first() {
        return Err(BrepError::Sketch(format!("the profile branches at point `{p}`")));
    }
    let loops = rings(feature, s, &found.loops, &keep);
    let circles = s.circles.iter().filter(|(_, c)| !c.construction).map(|(n, _)| vec![n.as_str()]);
    let curves = found.loops.iter().map(|c| c.iter().map(|st| st.curve).collect()).chain(circles).collect();
    Ok((found, loops, curves))
}

/// An ellipse among the profile curves `keep` keeps is refused (D218): it bounds no profile yet.
fn no_ellipse(s: &Placed, keep: &dyn Fn(&str) -> bool) -> Result<(), BrepError> {
    match s.ellipses.iter().find(|(n, e)| !e.construction && keep(n)) {
        Some((n, _)) => Err(BrepError::Unsupported(format!("an ellipse bounds no profile yet: make `{n}` construction (Q)"))),
        None => Ok(()),
    }
}

/// The curves of the innermost closed loop of `s` around point `p` (D191: a click inside a
/// contour chooses it), if any.
pub fn contour_at(feature: TopoId, s: &Placed, p: Vec2) -> Option<Vec<String>> {
    let (_, loops, curves) = named_loops(feature, s).ok()?;
    let around: Vec<usize> = (0..loops.len()).filter(|&k| loops[k].ring.winding(p) != 0).collect();
    // The innermost: inside every other one around the point.
    let inner = around.iter().copied().find(|&k| around.iter().all(|&a| a == k || loops[a].ring.winding(loops[k].ring.points[0]) != 0))?;
    Some(curves[inner].iter().map(|c| c.to_string()).collect())
}

/// The closed loops of the curves `keep` keeps (construction ones too if `construction`), an
/// error if a curve is on none: lines and arcs joined at their points, each loop starting at its
/// lowest point (see [`contours`]); then circles, each a loop of three arcs from its own points at
/// 0°, 120° and 240°.
pub(crate) fn loops(feature: TopoId, s: &Placed, keep: &dyn Fn(&str) -> bool, construction: bool) -> Result<Vec<Found>, BrepError> {
    if !construction {
        no_ellipse(s, keep)?;
    }
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
