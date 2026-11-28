//! Preview of a louver with a constant middle section and smooth end returns.

use std::collections::BTreeMap;

use datum_brep::{Body, BrepError, FaceSpec, Ix, Plane, ProfileArc, Surface, TopoId, Vertex, build_with_curves};
use datum_math::{Rigid, Sign, Vec2, Vec3, orient2d};
use serde_json::Value as Json;

use super::tongue::to_part;
use super::{STAMP_ROLE, SectionSide, unit};
use crate::rib::Formed;
use crate::sketch::Placed;
use crate::{Value, validated};

pub(super) struct Louver<'a> {
    pub wall: &'a Body,
    pub face: TopoId,
    pub onto_wall: Rigid,
    pub anchor: Vec3,
    pub section: &'a Placed,
    pub boundary: &'a Placed,
    pub thickness: f64,
    pub metadata: &'a BTreeMap<String, BTreeMap<String, Json>>,
    pub section_side: SectionSide,
}

fn bad(message: &str) -> BrepError {
    BrepError::Unsupported(format!("louver: {message}"))
}

/// Middle section on the upper branch of its circle.
struct Rise {
    center: Vec2,
    radius_squared: f64,
    sign: f64,
    start: Vec2,
    end: Vec2,
}

impl Rise {
    fn new(center: Vec2, start: Vec2, end: Vec2, tol: f64) -> Result<Self, BrepError> {
        let sign = if start.y + end.y >= 2.0 * center.y { 1.0 } else { -1.0 };
        if start.x >= end.x || sign * (start.y - center.y) < -tol || sign * (end.y - center.y) < -tol {
            return Err(bad("an arc needs to span the section on one side of its centre"));
        }
        let radius = libm::hypot(start.x - center.x, start.y - center.y);
        Ok(Self { center, radius_squared: radius * radius, sign, start, end })
    }

    fn at(&self, x: f64) -> f64 {
        if x <= self.start.x {
            self.start.y
        } else if x >= self.end.x {
            self.end.y
        } else {
            let dx = x - self.center.x;
            self.center.y + self.sign * libm::sqrt((self.radius_squared - dx * dx).max(0.0))
        }
    }
}

struct End {
    start: Vec2,
    end: Vec2,
    center: Vec2,
    turn: f64,
}

impl End {
    fn at(&self, fraction: f64) -> Vec2 {
        if fraction == 0.0 {
            return self.start;
        }
        if fraction == 1.0 {
            return self.end;
        }
        let angle = fraction * self.turn;
        let (sin, cos) = (libm::sin(angle), libm::cos(angle));
        let (x, y) = (self.start.x - self.center.x, self.start.y - self.center.y);
        Vec2::new(self.center.x + x * cos - y * sin, self.center.y + x * sin + y * cos)
    }
}

pub(super) fn form(feature: TopoId, louver: Louver<'_>) -> Result<Value, BrepError> {
    let Louver { wall, face, onto_wall, anchor, section, boundary, thickness, metadata, section_side } = louver;
    if section_side != SectionSide::Outer {
        return Err(bad("the preview takes the outer section"));
    }
    let role = |name: &str| metadata.get(name).and_then(|props| props.get(STAMP_ROLE)).and_then(Json::as_str);
    let active_lines = boundary.lines.iter().filter(|(_, line)| !line.construction).collect::<Vec<_>>();
    let active_arcs = boundary.arcs.iter().filter(|(_, arc)| !arc.construction).collect::<Vec<_>>();
    let profile_arcs = section.arcs.values().filter(|arc| !arc.construction).collect::<Vec<_>>();
    let [profile_arc] = profile_arcs.as_slice() else {
        return Err(bad("the middle section needs one arc"));
    };
    if section.lines.values().any(|line| !line.construction)
        || active_lines.len() != 4
        || active_arcs.len() != 2
        || [section, boundary]
            .iter()
            .any(|s| !s.patterns.is_empty() || s.circles.values().any(|c| !c.construction) || s.ellipses.values().any(|e| !e.construction))
    {
        return Err(bad("expected a middle arc and a boundary with two cuts and two end transitions"));
    }

    let [_, sx, sy, _] = section.frame;
    let z = boundary.frame[3];
    let tol = section.solved.tolerance.max(boundary.solved.tolerance);
    let height_sign = sy.dot(z).signum();
    if (sy.dot(z).abs() - 1.0).abs() > tol || (section.frame[0] - anchor).dot(z).abs() > tol {
        return Err(bad("the section must stand on the boundary plane"));
    }
    let y = unit(z.cross(sx));
    let x = y.cross(z);
    let frame = Rigid::frame(Vec3::ZERO, x, y, z).then(onto_wall);
    let local = |s: &Placed, name: &str| {
        let p = to_part(s, s.solved.points[name]) - anchor;
        Vec3::new(p.dot(x), p.dot(y), p.dot(z))
    };
    let profile_point = |name: &str| {
        let p = local(section, name);
        Vec2::new(p.x, p.z * height_sign)
    };
    let plan = |name: &str| {
        let p = local(boundary, name);
        Vec2::new(p.x, p.y)
    };
    let ordered = |mut a: Vec2, mut b: Vec2| {
        if a.x > b.x {
            std::mem::swap(&mut a, &mut b);
        }
        [a, b]
    };
    let same = |a: Vec2, b: Vec2| libm::hypot(a.x - b.x, a.y - b.y) <= tol;
    let [root, tip] = ordered(profile_point(&profile_arc.from), profile_point(&profile_arc.to));
    let profile_center = profile_point(&profile_arc.center);
    let profile = Rise::new(profile_center, root, tip, tol)?;
    if !same(root, Vec2::default()) || tip.y <= 0.0 || profile.sign < 0.0 || (tip.x - profile_center.x).abs() > tol {
        return Err(bad("the middle arc must rise from the anchor to a horizontal tangent"));
    }
    let mut ends = Vec::with_capacity(2);
    for (name, arc) in active_arcs {
        if role(name) != Some("transition") {
            return Err(bad("the end arcs must be marked as transitions"));
        }
        let from = plan(&arc.from);
        let [start, end] = ordered(from, plan(&arc.to));
        let sign = end.y.signum();
        let center = plan(&arc.center);
        let turn = ProfileArc { center, ccw: from == start }.turn(start, end);
        if start.x.abs() > tol || sign * start.y <= tol || end.y.abs() <= tol || turn.abs() >= std::f64::consts::PI {
            return Err(bad("the end arcs must lead from the root to opposite ends of the cut"));
        }
        ends.push((sign, End { start, end, center, turn }));
    }
    ends.sort_by(|a, b| a.0.total_cmp(&b.0));
    let [(-1.0, lower), (1.0, upper)] = ends.as_slice() else {
        return Err(bad("the end transitions must lie on opposite sides of the section"));
    };
    let cut = upper.end.x;
    if (lower.end.x - cut).abs() > tol || tip.x > cut + tol {
        return Err(bad("the cuts must meet on a line at or beyond the end of the middle arc"));
    }
    for (role_name, x, endpoint) in [("transition", 0.0, [lower.start.y, upper.start.y]), ("cut", cut, [lower.end.y, upper.end.y])] {
        for y in endpoint {
            let center = Vec2::new(x, 0.0);
            let end = Vec2::new(x, y);
            let count = active_lines
                .iter()
                .filter(|(name, line)| {
                    let [a, b] = [plan(&line.from), plan(&line.to)];
                    role(name) == Some(role_name) && ((same(a, center) && same(b, end)) || (same(b, center) && same(a, end)))
                })
                .count();
            if count != 1 {
                return Err(bad("the root and slit need two marked segments each"));
            }
        }
    }
    let sweep = ProfileArc { center: profile_center, ccw: false }.turn(root, tip).abs();
    let pullback = libm::sqrt(profile.radius_squared) * sweep - tip.x;
    if pullback <= 0.0 {
        return Err(bad("the raised section has no allowance for an open slit"));
    }

    assemble(feature, wall, face, frame, thickness, &profile, [lower, upper], cut, pullback, height_sign)
}

#[allow(clippy::too_many_arguments)]
fn assemble(
    feature: TopoId,
    wall: &Body,
    face: TopoId,
    frame: Rigid,
    t: f64,
    profile: &Rise,
    ends: [&End; 2],
    cut: f64,
    pullback: f64,
    height_sign: f64,
) -> Result<Value, BrepError> {
    // Include the horizontal land after the drawn arc and the ends of the middle plateau.
    const SECTION_STEPS: usize = 16;
    const HALF_WIDTH_STEPS: usize = 12;
    let mut stations = (0..=SECTION_STEPS).map(|i| profile.end.x * i as f64 / SECTION_STEPS as f64).collect::<Vec<_>>();
    if cut > profile.end.x {
        stations.push(cut);
    }
    let last = stations.len() - 1;
    let width = 2 * HALF_WIDTH_STEPS;
    let mut vertices = wall.vertices().to_vec();
    let id = |tag, i, j, k| TopoId::derive(tag).id(feature).index(i as u64).index(j as u64).index(k as u64).finish();
    let v = |i, j, k| id("stamp.louver.skin", i, j, k);
    let lip = |j, k| if j == 0 || j == width { v(last, j, k) } else { id("stamp.louver.lip", last, j, k) };
    let mut points = BTreeMap::new();
    let mut vertex = |id, point| {
        points.insert(id, point);
        vertices.push(Vertex { id, point: frame.apply(point) });
    };
    let mut y_bounds = [0.0_f64; 2];
    let mut x_min = 0.0_f64;
    for (i, &station) in stations.iter().enumerate() {
        let ends = ends.map(|end| end.at(station / cut));
        for j in 0..=width {
            let across = (j as f64 - HALF_WIDTH_STEPS as f64) / HALF_WIDTH_STEPS as f64;
            let end = if across < 0.0 { ends[0] } else { ends[1] };
            let x = if i == 0 || i == last { station } else { station + across.abs() * (end.x - station) };
            let y = across.abs() * end.y;
            x_min = x_min.min(x);
            y_bounds = [y_bounds[0].min(y), y_bounds[1].max(y)];
            let u = (2.0 * across.abs() - 1.0).max(0.0);
            let fade = (1.0 - u) * (1.0 - u) * (1.0 + 2.0 * u);
            let height = if i == 0 { 0.0 } else { height_sign * fade * profile.at(x) };
            for k in 0..2 {
                let z = -(k as f64) * t;
                vertex(v(i, j, k), Vec3::new(x, y, height + z));
                if i == last && j != 0 && j != width {
                    vertex(lip(j, k), Vec3::new(cut + pullback * fade * fade, y, z));
                }
            }
        }
    }

    let rise = height_sign * profile.end.y;
    let lo = Vec3::new(x_min, y_bounds[0], rise.min(0.0) - t);
    let hi = Vec3::new(cut + pullback, y_bounds[1], rise.max(0.0));
    let (top, bottom) = super::stitch::wall_faces_in(wall, face, t, frame, [y_bounds, [lo.x, hi.x]], (lo, hi), || {
        bad("the louver needs a clear opening in a flat wall")
    })?;
    let mut faces = (0..wall.faces().len()).map(|i| wall.face_spec(i as Ix)).collect::<Vec<_>>();
    let ring = |k| {
        (0..last)
            .map(|i| v(i, 0, k))
            .chain((0..=width).map(|j| lip(j, k)))
            .chain((0..last).rev().map(|i| v(i, width, k)))
            .chain((1..width).rev().map(|j| v(0, j, k)))
            .collect::<Vec<_>>()
    };
    faces[top].loops.push(ring(0).into_iter().rev().collect());
    faces[bottom].loops.push(ring(1));
    let mut add = |id, ring: Vec<TopoId>| {
        let [a, b, c] = [points[&ring[0]], points[&ring[1]], points[&ring[2]]];
        let normal = unit((b - a).cross(c - a));
        let surface = Surface::Plane(Plane { origin: a, normal }).moved(&frame);
        faces.push(FaceSpec { id, surface, loops: vec![ring] });
    };
    let plan = |i, j| {
        let p = points[&v(i, j, 0)];
        Vec2::new(p.x, p.y)
    };
    for i in 0..last {
        for j in 0..width {
            let [a, b, c, d] = [plan(i, j), plan(i + 1, j), plan(i + 1, j + 1), plan(i, j + 1)];
            if orient2d(a, b, c) != Sign::Positive || orient2d(a, c, d) != Sign::Positive {
                return Err(BrepError::Degenerate("louver: the end transition folds in plan".into()));
            }
            for k in 0..2 {
                let mut quad = [v(i, j, k), v(i + 1, j, k), v(i + 1, j + 1, k), v(i, j + 1, k)];
                if k == 1 {
                    quad.reverse();
                }
                add(id("stamp.louver.panel", i, j, 2 * k), vec![quad[0], quad[1], quad[2]]);
                add(id("stamp.louver.panel", i, j, 2 * k + 1), vec![quad[0], quad[2], quad[3]]);
            }
        }
    }
    for j in 0..width {
        add(id("stamp.louver.cut", last, j, 0), vec![v(last, j + 1, 0), v(last, j, 0), v(last, j, 1), v(last, j + 1, 1)]);
        add(id("stamp.louver.cut", last, j, 1), vec![lip(j, 0), lip(j + 1, 0), lip(j + 1, 1), lip(j, 1)]);
    }
    let body = build_with_curves(vertices, faces, wall.curve_specs())?;
    validated(&body)?;
    Ok(Value::Formed(Box::new(Formed {
        body,
        blank: wall.clone(),
        imprints: Vec::new(),
        blank_unsupported: Some("louver: the end returns need a manufacturing development rule"),
        stamp_cuts: Vec::new(),
    })))
}
