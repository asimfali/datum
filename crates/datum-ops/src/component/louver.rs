//! Preview of a louver with a constant middle section and smooth end returns.

use std::collections::BTreeMap;

use datum_brep::{
    Bicubic, Body, BrepError, Cubic, CubicWall, Curve, CurveSpec, FaceSpec, Ix, Plane, ProfileArc, Surface, TopoId, Vertex, build_with_curves,
};
use datum_math::{Rigid, Sign, Vec2, Vec3, orient2d};
use serde_json::Value as Json;

use super::tongue::to_part;
use super::{STAMP_ROLE, SectionSide, unit};
use crate::rib::Formed;
use crate::sketch::Placed;
use crate::unfold::StampCut;
use crate::{Value, validated};

pub(super) struct Louver<'a> {
    pub wall: &'a Body,
    pub blank: &'a Body,
    pub face: TopoId,
    pub onto_wall: Rigid,
    pub anchor: Vec3,
    pub section: &'a Placed,
    pub boundary: &'a Placed,
    pub thickness: f64,
    pub bend_radius: f64,
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
    end: Vec2,
}

impl Rise {
    fn new(center: Vec2, start: Vec2, end: Vec2, tol: f64) -> Result<Self, BrepError> {
        let sign = if start.y + end.y >= 2.0 * center.y { 1.0 } else { -1.0 };
        if start.x >= end.x || sign * (start.y - center.y) < -tol || sign * (end.y - center.y) < -tol {
            return Err(bad("an arc needs to span the section on one side of its centre"));
        }
        let radius = libm::hypot(start.x - center.x, start.y - center.y);
        Ok(Self { center, radius_squared: radius * radius, sign, end })
    }
}

/// Two concentric reverse bends tangent to the flat sheet and the middle section.
struct BackBend {
    center_x: f64,
    radius: [f64; 2],
    join_x: [f64; 2],
}

impl BackBend {
    fn new(profile: &Rise, inside: f64, thickness: f64, height_sign: f64) -> Result<Self, BrepError> {
        if !inside.is_finite() || inside <= 0.0 {
            return Err(bad("the sheet rule needs a positive bend radius"));
        }
        let radius = if height_sign < 0.0 { [inside + thickness, inside] } else { [inside, inside + thickness] };
        let distance = radius[0] + libm::sqrt(profile.radius_squared);
        let dy = radius[0] - profile.center.y;
        let dx = libm::sqrt((distance - dy) * (distance + dy));
        let center_x = profile.center.x - dx;
        let join_x = radius.map(|r| center_x + dx * r / distance);
        if !center_x.is_finite() || center_x >= 0.0 || join_x.iter().any(|&x| x >= profile.end.x) {
            return Err(bad("the bend does not fit between the flat wall and the middle arc"));
        }
        Ok(Self { center_x, radius, join_x })
    }

    fn section(&self, profile: &Rise, x: f64, skin: usize) -> (f64, f64, f64) {
        let radius = self.radius[skin];
        if x <= self.center_x {
            (0.0, 0.0, 0.0)
        } else if x <= self.join_x[skin] {
            let dx = x - self.center_x;
            let vertical = libm::sqrt((radius - dx) * (radius + dx));
            (dx * dx / (radius + vertical), dx / vertical, radius * radius / (vertical * vertical * vertical))
        } else {
            let offset = self.radius[0] - radius;
            let dx = x.min(profile.end.x) - profile.center.x;
            let radius = libm::sqrt(profile.radius_squared) + offset;
            let vertical = libm::sqrt((radius - dx) * (radius + dx));
            let (slope, curvature) =
                if x >= profile.end.x { (0.0, 0.0) } else { (-dx / vertical, -radius * radius / (vertical * vertical * vertical)) };
            (profile.center.y + vertical - offset, slope, curvature)
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

    fn station(&self, station: f64, rear: f64, cut: f64) -> (Vec2, Vec2) {
        let speed = self.turn / cut;
        let tangent = |point: Vec2| Vec2::new((self.center.y - point.y) * speed + self.start.x / cut, (point.x - self.center.x) * speed);
        if station < 0.0 {
            // Extend the end arc to the flat start of the reverse bend. This Hermite span
            // matches the longitudinal direction at the rear and the arc tangent at zero.
            let span = -rear;
            let u = (station - rear) / span;
            let correction = span * u * u * (u - 1.0);
            let slope = u * (3.0 * u - 2.0);
            let start_tangent = tangent(self.start);
            (
                Vec2::new(station + correction * (start_tangent.x - 1.0), self.start.y + correction * start_tangent.y),
                Vec2::new(1.0 + slope * (start_tangent.x - 1.0), slope * start_tangent.y),
            )
        } else {
            let fraction = (station / cut).min(1.0);
            let end_point = self.at(fraction);
            // Put the sketch's solved root on the common station at working precision.
            (Vec2::new(end_point.x - (1.0 - fraction) * self.start.x, end_point.y), tangent(end_point))
        }
    }
}

/// The transverse section of a skin and its two tangent vectors. The plan frame carries the
/// skew of the end arc independently of the fade from the middle section to the wall.
struct SkinSection {
    plan: Vec2,
    along: Vec2,
    across: Vec2,
    mixed: Vec2,
    fade: f64,
    fade_slope: f64,
}

impl SkinSection {
    fn at((end_point, end_tangent): (Vec2, Vec2), station: f64, across: f64, cut: f64) -> Self {
        let width = across.abs();
        let side = across.signum();
        let dx = if station >= cut { 0.0 } else { end_point.x - station };
        let x = station + width * dx;
        let u = (2.0 * width - 1.0).max(0.0);
        Self {
            plan: Vec2::new(x, width * end_point.y),
            along: Vec2::new(1.0 + width * (end_tangent.x - 1.0), width * end_tangent.y),
            across: Vec2::new(side * dx, side * end_point.y),
            mixed: Vec2::new(side * (end_tangent.x - 1.0), side * end_tangent.y),
            fade: (1.0 - u) * (1.0 - u) * (1.0 + 2.0 * u),
            fade_slope: -12.0 * side * u * (1.0 - u),
        }
    }

    fn point_and_tangents(&self, profile: (f64, f64, f64), base: f64, direction: f64) -> (Vec3, [Vec3; 3]) {
        let (rise, slope, curvature) = profile;
        let dz_dx = direction * self.fade * slope;
        (
            Vec3::new(self.plan.x, self.plan.y, direction * self.fade * rise + base),
            [
                Vec3::new(self.along.x, self.along.y, dz_dx * self.along.x),
                Vec3::new(self.across.x, self.across.y, dz_dx * self.across.x + direction * self.fade_slope * rise),
                Vec3::new(
                    self.mixed.x,
                    self.mixed.y,
                    dz_dx * self.mixed.x + direction * (self.fade_slope * slope + self.fade * curvature * self.across.x) * self.along.x,
                ),
            ],
        )
    }
}

pub(super) fn form(feature: TopoId, louver: Louver<'_>) -> Result<Value, BrepError> {
    let Louver { wall, blank, face, onto_wall, anchor, section, boundary, thickness, bend_radius, metadata, section_side } = louver;
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
    let mut cut_edge = Vec::with_capacity(2);
    for (role_name, x, endpoint) in [("transition", 0.0, [lower.start.y, upper.start.y]), ("cut", cut, [lower.end.y, upper.end.y])] {
        for y in endpoint {
            let center = Vec2::new(x, 0.0);
            let end = Vec2::new(x, y);
            let matches = active_lines
                .iter()
                .filter_map(|(name, line)| {
                    if role(name) != Some(role_name) {
                        return None;
                    }
                    let [a, b] = [plan(&line.from), plan(&line.to)];
                    [(a, b), (b, a)].into_iter().find_map(|(p, q)| (same(p, center) && same(q, end)).then_some((*name, q)))
                })
                .collect::<Vec<_>>();
            let [edge] = matches.as_slice() else {
                return Err(bad("the root and slit need two marked segments each"));
            };
            if role_name == "cut" {
                cut_edge.push(*edge);
            }
        }
    }
    let [(first, a), (last, b)] = cut_edge.as_slice() else {
        return Err(bad("the slit needs both marked segments"));
    };
    let cut_id = TopoId::derive("stamp.louver.slit").id(feature).tag(first).tag(last).finish();
    let slit = [*a, *b].map(|p| frame.apply(Vec3::new(p.x, p.y, 0.0)));
    let sweep = ProfileArc { center: profile_center, ccw: false }.turn(root, tip).abs();
    let pullback = libm::sqrt(profile.radius_squared) * sweep - tip.x;
    if pullback <= 0.0 {
        return Err(bad("the raised section has no allowance for an open slit"));
    }

    let bend = BackBend::new(&profile, bend_radius, thickness, height_sign)?;
    assemble(feature, [wall, blank], face, frame, thickness, &profile, &bend, [lower, upper], cut, pullback, height_sign, (slit, cut_id))
}

#[allow(clippy::too_many_arguments)]
fn assemble(
    feature: TopoId,
    [wall, blank]: [&Body; 2],
    face: TopoId,
    frame: Rigid,
    t: f64,
    profile: &Rise,
    bend: &BackBend,
    ends: [&End; 2],
    cut: f64,
    pullback: f64,
    height_sign: f64,
    (slit, cut_id): ([Vec3; 2], TopoId),
) -> Result<Value, BrepError> {
    // Include the horizontal land after the drawn arc and the ends of the middle plateau.
    const SECTION_STEPS: usize = 16;
    const HALF_WIDTH_STEPS: usize = 12;
    let mut stations = (0..=SECTION_STEPS).map(|i| profile.end.x * i as f64 / SECTION_STEPS as f64).collect::<Vec<_>>();
    stations.extend((0..HALF_WIDTH_STEPS).map(|i| bend.center_x * (1.0 - i as f64 / HALF_WIDTH_STEPS as f64)));
    stations.extend(bend.join_x);
    stations.sort_by(f64::total_cmp);
    stations.dedup_by(|a, b| a == b);
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
        let end_stations = ends.map(|end| end.station(station, bend.center_x, cut));
        for j in 0..=width {
            let across = (j as f64 - HALF_WIDTH_STEPS as f64) / HALF_WIDTH_STEPS as f64;
            let end = end_stations[usize::from(across >= 0.0)];
            let sample = SkinSection::at(end, station, across, cut);
            let Vec2 { x, y } = sample.plan;
            x_min = x_min.min(x);
            y_bounds = [y_bounds[0].min(y), y_bounds[1].max(y)];
            for k in 0..2 {
                let base = -(k as f64) * t;
                let (point, [along, across, _]) = sample.point_and_tangents(bend.section(profile, x, k), base, height_sign);
                if k == 0
                    && j > 0
                    && j < width
                    && orient2d(Vec2::default(), Vec2::new(along.x, along.y), Vec2::new(across.x, across.y)) != Sign::Positive
                {
                    return Err(BrepError::Degenerate("louver: the end transition folds in plan".into()));
                }
                vertex(v(i, j, k), point);
                if i == last && j != 0 && j != width {
                    vertex(lip(j, k), Vec3::new(cut + pullback * sample.fade * sample.fade, y, base));
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
    let plan = |i, j| {
        let p = points[&v(i, j, 0)];
        Vec2::new(p.x, p.y)
    };
    let mut skin_edges = BTreeMap::new();
    let across_step = 1.0 / HALF_WIDTH_STEPS as f64;
    for i in 0..last {
        let du = stations[i + 1] - stations[i];
        for j in 0..width {
            let [a, b, c, d] = [plan(i, j), plan(i + 1, j), plan(i + 1, j + 1), plan(i, j + 1)];
            if orient2d(a, b, c) != Sign::Positive || orient2d(a, c, d) != Sign::Positive {
                return Err(BrepError::Degenerate("louver: the end transition folds in plan".into()));
            }
            let side = usize::from(j >= HALF_WIDTH_STEPS);
            for k in 0..2 {
                let corners = [(i, j), (i, j + 1), (i + 1, j), (i + 1, j + 1)].map(|(i, j)| {
                    let across = ((j as f64 - HALF_WIDTH_STEPS as f64) * across_step).abs() * if side == 0 { -1.0 } else { 1.0 };
                    let station = stations[i];
                    let end = ends[side].station(station, bend.center_x, cut);
                    let sample = SkinSection::at(end, station, across, cut);
                    let point = points[&v(i, j, k)];
                    let (_, [u, v, uv]) = sample.point_and_tangents(bend.section(profile, point.x, k), -(k as f64) * t, height_sign);
                    [point, u * du, v * across_step, uv * (du * across_step)]
                });
                let mut patch = Bicubic::hermite(corners);
                let mut ring = [v(i, j, k), v(i + 1, j, k), v(i + 1, j + 1, k), v(i, j + 1, k)];
                if k == 1 {
                    patch.points = std::array::from_fn(|u| std::array::from_fn(|v| patch.points[v][u]));
                    ring.swap(1, 3);
                }
                patch.points = patch.points.map(|row| row.map(|p| frame.apply(p)));
                let mut bounds = patch.boundaries();
                bounds[2].points.reverse();
                bounds[3].points.reverse();
                for (index, mut curve) in bounds.into_iter().enumerate() {
                    let (a, b) = (ring[index], ring[(index + 1) % 4]);
                    let (from, to) = (a.min(b), a.max(b));
                    if from != a {
                        curve.points.reverse();
                    }
                    let existing = skin_edges.entry((from, to)).or_insert(curve);
                    if *existing != curve {
                        return Err(BrepError::Invalid("louver: neighboring patches disagree on their shared edge".into()));
                    }
                }
                faces.push(FaceSpec { id: id("stamp.louver.skin.patch", i, j, k), surface: Surface::Bicubic(patch), loops: vec![ring.to_vec()] });
            }
        }
    }
    let mut add = |id, ring: Vec<TopoId>| {
        let [a, b, c] = [points[&ring[0]], points[&ring[1]], points[&ring[2]]];
        let normal = unit((b - a).cross(c - a));
        let surface = Surface::Plane(Plane { origin: a, normal }).moved(&frame);
        faces.push(FaceSpec { id, surface, loops: vec![ring] });
    };
    for j in 0..width {
        add(id("stamp.louver.cut", last, j, 0), vec![v(last, j + 1, 0), v(last, j, 0), v(last, j, 1), v(last, j + 1, 1)]);
        if (width / 4..3 * width / 4).contains(&j) {
            add(id("stamp.louver.cut", last, j, 1), vec![lip(j, 0), lip(j + 1, 0), lip(j + 1, 1), lip(j, 1)]);
        }
    }

    // The stationary lip has a planar land in the middle and a cubic span along each
    // sampled interval of its returns. Both skins and the cut wall share the same curves.
    let lip_slope = |j: usize| {
        let across = (j as f64 - HALF_WIDTH_STEPS as f64) / HALF_WIDTH_STEPS as f64;
        let end = ends[usize::from(across >= 0.0)].station(cut, bend.center_x, cut);
        let sample = SkinSection::at(end, cut, across, cut);
        2.0 * pullback * sample.fade * sample.fade_slope / HALF_WIDTH_STEPS as f64
    };
    let mut curves = wall.curve_specs();
    curves.extend(skin_edges.into_iter().map(|((from, to), c)| CurveSpec { from, to, curve: Curve::Cubic(c) }));
    for j in (0..width / 4).chain(3 * width / 4..width) {
        let dy = points[&lip(j + 1, 0)].y - points[&lip(j, 0)].y;
        let tangents = [j, j + 1].map(|k| frame.apply_vector(Vec3::new(lip_slope(k), dy, 0.0)));
        let edges = [0, 1].map(|k| {
            let [from, to] = [lip(j, k), lip(j + 1, k)];
            let cubic = Cubic::hermite([frame.apply(points[&from]), frame.apply(points[&to])], tangents);
            curves.push(CurveSpec { from, to, curve: Curve::Cubic(cubic) });
            cubic
        });
        faces.push(FaceSpec {
            id: id("stamp.louver.cut", last, j, 1),
            surface: Surface::CubicWall(CubicWall { edges, along: frame.apply_vector(Vec3::new(0.0, 1.0, 0.0)) }),
            loops: vec![vec![lip(j, 0), lip(j + 1, 0), lip(j + 1, 1), lip(j, 1)]],
        });
    }
    let body = build_with_curves(vertices, faces, curves)?;
    validated(&body)?;
    let cut = StampCut {
        faces: [face, wall.faces()[bottom].id],
        points: slit.into(),
        normal: frame.apply_vector(Vec3::new(0.0, 0.0, 1.0)),
        ids: vec![cut_id],
        arcs: vec![None],
    };
    Ok(Value::Formed(Box::new(Formed { body, blank: blank.clone(), imprints: Vec::new(), stamp_cuts: vec![cut] })))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn end_return_is_tangent_at_the_flat_wall_and_the_middle_section() -> Result<(), BrepError> {
        let profile = Rise::new(Vec2::new(3.0, -4.0), Vec2::default(), Vec2::new(3.0, 1.0), 0.0)?;
        for side in [-1.0, 1.0] {
            let end = End {
                start: Vec2::new(0.0, side * 8.0),
                end: Vec2::new(4.0, side * 12.0),
                center: Vec2::new(4.0, side * 8.0),
                turn: -side * std::f64::consts::FRAC_PI_2,
            };
            let bend = BackBend::new(&profile, 0.75, 0.5, side)?;
            for skin in 0..2 {
                let at = |station, across| {
                    let sample = SkinSection::at(end.station(station, bend.center_x, 4.0), station, across, 4.0);
                    let section = bend.section(&profile, sample.plan.x, skin);
                    sample.point_and_tangents(section, -0.5 * skin as f64, side)
                };
                let (_, [along, across, _]) = at(2.0, side);
                assert_eq!(along.z, 0.0);
                assert_eq!(across.z, 0.0);
                let (middle, [along, across, _]) = at(2.0, side * 0.5);
                let normal = unit(along.cross(across));
                let h = 1e-7;
                let (outside, _) = at(2.0, side * (0.5 + h));
                assert!(((outside - middle) * (1.0 / h)).dot(normal).abs() < 1e-5);

                let (left, [left_tangent, ..]) = at(-h, side * 0.75);
                let (root, [right_tangent, ..]) = at(0.0, side * 0.75);
                let (right, _) = at(h, side * 0.75);
                for tangent in [left_tangent, (root - left) * (1.0 / h), (right - root) * (1.0 / h)] {
                    let error = tangent - right_tangent;
                    assert!(error.dot(error) < 1e-10);
                }
            }
        }
        Ok(())
    }

    #[test]
    fn back_bend_is_tangent_on_both_skins_for_either_forming_direction() -> Result<(), BrepError> {
        let profile = Rise::new(Vec2::new(3.0, -4.0), Vec2::default(), Vec2::new(3.0, 1.0), 0.0)?;
        let thickness = 0.5;
        let inside = 0.75;
        for sign in [-1.0, 1.0] {
            let bend = BackBend::new(&profile, inside, thickness, sign)?;
            let [a, b] = bend.radius;
            assert_eq!(a.min(b), inside);
            assert_eq!(a.max(b), inside + thickness);
            for (skin, radius) in bend.radius.into_iter().enumerate() {
                let join = bend.join_x[skin];
                let dx = join - bend.center_x;
                let rise = bend.section(&profile, join, skin).0;
                let offset = bend.radius[0] - radius;
                let radial = Vec2::new(dx, rise - radius);
                let profile_radial = Vec2::new(join - profile.center.x, rise + offset - profile.center.y);
                assert!((libm::hypot(radial.x, radial.y) - radius).abs() < 1e-12);
                assert!((libm::hypot(profile_radial.x, profile_radial.y) - (5.0 + offset)).abs() < 1e-12);
                assert!((radial.x * profile_radial.y - radial.y * profile_radial.x).abs() < 1e-12);
                assert_eq!(bend.section(&profile, bend.center_x, skin).0, 0.0);
            }
        }
        Ok(())
    }
}
