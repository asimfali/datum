//! Form a raised area along a convex chain of transitions, open at a slit or closed.

use datum_brep::{Body, BrepError, Curve, CurveSpec, FaceSpec, Ix, Plane, Surface, TopoId, Vertex, build_with_curves, segments_touch};
use datum_math::{Rigid, Sign, Vec2, Vec3, orient2d};
use datum_sketch::Step;

use super::{Piece, Stand, arc_between, swept, turned, unit};
use crate::edge_finish::{EdgeMode, corner};
use crate::rib::Formed;
use crate::sheet::{Sheet, SheetPath, Side, bend_profile};
use crate::sketch::{Placed, within_rounding};
use crate::unfold::StampCut;
use crate::{Value, validated};

fn bad(message: impl std::fmt::Display) -> BrepError {
    BrepError::Degenerate(format!("stamp transition: {message}"))
}

fn names<'a, 'b>(chain: &'b [Step<'a>]) -> impl Iterator<Item = &'a str> + 'b {
    chain.iter().map(|s| s.from).chain(chain.last().map(|s| s.to))
}

fn length(v: Vec3) -> f64 {
    libm::sqrt(v.dot(v))
}

fn xy(p: Vec2, z: f64) -> Vec3 {
    Vec3::new(p.x, p.y, z)
}

pub(super) struct Pocket<'a> {
    pub wall: &'a Body,
    pub face: TopoId,
    pub frame: Rigid,
    pub section: &'a Placed,
    pub boundary: &'a Placed,
    pub anchor: Vec3,
    pub cut: Option<&'a str>,
    pub transitions: Vec<Step<'a>>,
    pub thickness: f64,
    pub radius: f64,
}

const SECTION_STEPS: [(usize, usize); 6] = [(0, 1), (1, 2), (2, 3), (4, 7), (5, 8), (6, 9)];

struct Station {
    point: Vec3,
    inward: Vec3,
}

pub(super) fn form(feature: TopoId, pocket: Pocket<'_>) -> Result<Value, BrepError> {
    let Pocket { wall, face, frame, section, boundary, anchor, cut, mut transitions, thickness: t, radius: r } = pocket;
    let [_, x, y, z] = boundary.frame;
    let point = |name: &str| {
        let p = super::tongue::to_part(boundary, boundary.solved.points[name]) - anchor;
        Vec2::new(p.dot(x), p.dot(y))
    };
    let points = |chain: &[Step<'_>]| names(chain).map(point).collect::<Vec<_>>();
    let closed = cut.is_none();
    let mut polygon = points(&transitions);
    if closed {
        polygon.pop();
    }
    if orient2d(polygon[0], polygon[1], polygon[2]) == Sign::Negative {
        transitions.reverse();
        for step in &mut transitions {
            std::mem::swap(&mut step.from, &mut step.to);
        }
        polygon.reverse();
        if closed {
            polygon.rotate_right(1);
        }
    }
    let count = polygon.len();
    let turn = |i| orient2d(polygon[(i + count - 1) % count], polygon[i], polygon[(i + 1) % count]);
    if (0..count).any(|i| turn(i) != Sign::Positive)
        || (0..count).any(|i| {
            (i + 2..count).any(|j| (i != 0 || j + 1 != count) && segments_touch(polygon[i], polygon[i + 1], polygon[j], polygon[(j + 1) % count]))
        })
    {
        return Err(bad("the transition edges must bound a strictly convex contour"));
    }
    let first = xy(polygon[0], 0.0);
    let last = xy(polygon[count - 1], 0.0);
    let slit = unit(last - first);
    let direction = |i: usize| unit(xy(polygon[(i + 1) % count], 0.0) - xy(polygon[i], 0.0));
    let normal = Vec3::new(0.0, 0.0, 1.0);
    let inward = |i| normal.cross(direction(i));
    if !closed && ![0, count - 2].into_iter().all(|i| within_rounding(direction(i).dot(slit), 1.0)) {
        return Err(bad("both transitions at the slit must be perpendicular to it"));
    }

    // The transverse sketch defines the rise and draft. Solve it before
    // taking dimensions, since its saved points may differ from the constrained result.
    let contours = section.contours(&|_, construction| !construction);
    let [chain] = contours.paths.as_slice() else { return Err(bad("the section needs one open chain")) };
    if !contours.loops.is_empty()
        || !contours.branches.is_empty()
        || chain.len() < 3
        || !chain.iter().all(|s| section.lines.contains_key(s.curve))
        || section.circles.values().any(|c| !c.construction)
        || section.ellipses.values().any(|e| !e.construction)
        || !section.patterns.is_empty()
    {
        return Err(bad("the section needs two walls and a flat raised span"));
    }
    let coords = names(chain)
        .map(|n| {
            let p = super::tongue::to_part(section, section.solved.points[n]) - anchor;
            Vec3::new(p.dot(x), p.dot(y), p.dot(z))
        })
        .collect::<Vec<_>>();
    let height = coords[1].z;
    let scale = length(last - first).max(height.abs()).max(t).max(r);
    let offset = (boundary.frame[0] - anchor).dot(z);
    if !within_rounding(offset, scale) {
        return Err(bad("the boundary must lie on the sheet through the stamp anchor"));
    }
    let same = |a: Vec3, b: Vec3| within_rounding(length(a - b), scale);
    let a = coords[0];
    let b = coords[coords.len() - 1];
    let along = if closed { unit(b - a) } else { slit };
    let shoulder_a = coords[1];
    let shoulder_b = coords[coords.len() - 2];
    let draft = if closed { (shoulder_a - a).dot(along) } else { 0.0 };
    if closed {
        if !same(a, Vec3::new(a.x, a.y, 0.0)) || !same(b, Vec3::new(b.x, b.y, 0.0)) || draft < 0.0 || (shoulder_b - shoulder_a).dot(along) <= 0.0 {
            return Err(bad("the section needs two equal inward slopes from the sheet to the shelf"));
        }
    } else if !(((same(a, first) && same(b, last)) || (same(a, last) && same(b, first))) && height > r + r + t) {
        return Err(bad("the section must rise at both slit ends with room for both bends"));
    }
    if !(height > 0.0 && r > 0.0)
        || !same(shoulder_a, a + normal * height + along * draft)
        || !same(shoulder_b, b + normal * height - along * draft)
        || coords[1..coords.len() - 1].iter().any(|p| {
            let from = *p - a;
            !same(*p, a + along * from.dot(along) + normal * height)
        })
    {
        return Err(bad("the section needs two equal slopes and a flat raised span"));
    }
    let upright_run = r + r + t;
    let section_points = [Vec2::new(-r, 0.0), Vec2::new(r, 0.0), Vec2::new(r + draft, height), Vec2::new(upright_run + r + draft, height)];
    let profile = bend_profile(
        feature,
        SheetPath { p: &section_points, names: &["lead", "foot", "shoulder", "shelf"], lines: &["lead", "wall", "shelf"], arcs: &[None; 3] },
        Sheet { thickness: t, radius: r, side: Side::Right },
        &|_, _| false,
    )?;
    // Keep the six bend/wall pieces, plus the tangencies at both ends of each skin.
    let indices = [1, 2, 3, 4, 7, 8, 9, 10];
    let root = if closed { profile.points[1].x } else { 0.0 };
    let section = indices.map(|i| {
        let p = profile.points[i];
        let p = Vec2::new(p.x - root, p.y);
        Vec2::new(if within_rounding(p.x, scale) { 0.0 } else { p.x }, p.y)
    });
    // The shelf's tangency is inward by the full run of both bends. Leave the
    // rule radius there, so all meridians of the two bends stay off the turn axis.
    let round = if closed { section[3].x + r } else { upright_run + r };
    let mut profile = profile;
    if closed {
        for arc in profile.arcs.iter_mut().flatten() {
            arc.center.x -= root;
        }
    }
    let mut stations = Vec::with_capacity(2 * count);
    let mut axes = Vec::with_capacity(2 * count);
    if !closed {
        stations.push(Station { point: first, inward: inward(0) });
    }
    let corners = if closed { 0..count } else { 1..count - 1 };
    for i in corners {
        let previous = (i + count - 1) % count;
        let ([a, b], Some(arc)) = corner(polygon[i], [polygon[previous], polygon[(i + 1) % count]], round, EdgeMode::Fillet)
            .map_err(|_| bad(format!("a corner radius of {round} mm does not fit at `{}`", transitions[i].from)))?
        else {
            return Err(bad("expected a round corner"));
        };
        if let Some(last) = stations.last() {
            if (xy(a, 0.0) - last.point).dot(direction(previous)) <= 0.0 {
                return Err(bad(format!("corner radii of {round} mm overlap along `{}`", transitions[previous].curve)));
            }
            axes.push(None);
        }
        stations.push(Station { point: xy(a, 0.0), inward: inward(previous) });
        axes.push(Some(xy(arc.center, 0.0)));
        stations.push(Station { point: xy(b, 0.0), inward: inward(i) });
    }
    if closed {
        if (stations[0].point - stations[stations.len() - 1].point).dot(direction(count - 1)) <= 0.0 {
            return Err(bad(format!("corner radii of {round} mm overlap along `{}`", transitions[count - 1].curve)));
        }
    } else {
        stations.push(Station { point: last, inward: inward(count - 2) });
    }
    axes.push(None);
    // All sections move towards the convex interior and rise above the original wall.
    let bounds = polygon
        .into_iter()
        .fold((first, first), |(lo, hi), p| (Vec3::new(lo.x.min(p.x), lo.y.min(p.y), -t), Vec3::new(hi.x.max(p.x), hi.y.max(p.y), height)));
    assemble(feature, wall, face, frame, cut, &transitions, &stations, &axes, &profile, section, t, r, bounds)
}

#[allow(clippy::too_many_arguments)]
fn assemble(
    feature: TopoId,
    wall: &Body,
    face: TopoId,
    frame: Rigid,
    cut: Option<&str>,
    transitions: &[Step<'_>],
    stations: &[Station],
    axes: &[Option<Vec3>],
    profile: &datum_brep::ProfileLoop,
    section: [Vec2; 8],
    t: f64,
    r: f64,
    (lo, hi): (Vec3, Vec3),
) -> Result<Value, BrepError> {
    let z = Vec3::new(0.0, 0.0, 1.0);
    let id = |tag: &str, station: usize, row: usize| {
        let step = transitions[station / 2];
        TopoId::derive(tag).id(feature).tag(step.curve).index((station % 2) as u64).index(row as u64).finish()
    };
    let v = |s, k| id("stamp.pocket.vertex", s, k);
    let n = stations.len() - 1;
    let pt = |s: usize, q: Vec2| stations[s].point + stations[s].inward * q.x + z * q.y;
    let mut vertices = wall.vertices().to_vec();
    let mut curves = wall.curve_specs();
    let mut faces: Vec<_> = (0..wall.faces().len()).map(|i| wall.face_spec(i as Ix)).collect();
    let mut add_arc = |from, to, center, a, b, radius| {
        let arc = arc_between(frame.apply(center), frame.apply(a), frame.apply(b), radius).ok_or_else(|| bad("a swept arc collapsed"))?;
        curves.push(CurveSpec { from, to, curve: Curve::Arc(arc) });
        Ok::<_, BrepError>(())
    };
    let cross = Vec2::new(super::section::root_cross_offset(r, t), 0.0);
    for s in 0..=n {
        for (k, p) in section.into_iter().enumerate() {
            vertices.push(Vertex { id: v(s, k), point: frame.apply(pt(s, p)) });
        }
        let end = cut.is_some() && (s == 0 || s == n);
        if end {
            vertices.push(Vertex { id: v(s, 8), point: frame.apply(pt(s, cross)) });
        }
        for (k, index) in SECTION_STEPS {
            let Some(arc) = profile.arc(index) else { continue };
            let center = pt(s, arc.center);
            let a = pt(s, section[k]);
            let b = pt(s, section[k + 1]);
            let radius = length(a - center);
            if end && k == 0 {
                let mid = pt(s, cross);
                add_arc(v(s, 0), v(s, 8), center, a, mid, radius)?;
                add_arc(v(s, 8), v(s, 1), center, mid, b, radius)?;
            } else {
                add_arc(v(s, k), v(s, k + 1), center, a, b, radius)?;
            }
        }
    }
    for (s, &axis) in axes.iter().enumerate() {
        let next = (s + 1) % stations.len();
        let tangent = stations[s].inward.cross(z);
        for (k, index) in SECTION_STEPS {
            let piece = profile.arc(index).map_or(Piece::Line, |a| Piece::Arc { center: a.center, ccw: !a.ccw });
            let a = section[k + 1];
            let b = section[k];
            let mut stand = Stand { o: stations[s].point, x: stations[s].inward, y: z, z: tangent, at: 0.0 };
            let surface = if let Some(center) = axis {
                let radius = length(stand.o - center);
                stand.o = center;
                let shift = |p: Vec2| Vec2::new(p.x - radius, p.y);
                let piece = match piece {
                    Piece::Line => Piece::Line,
                    Piece::Arc { center, ccw } => Piece::Arc { center: shift(center), ccw },
                };
                turned(&piece, shift(a), shift(b), stand, radius)?
            } else {
                swept(&piece, a, b, stand)?
            };
            let mut ring = vec![v(s, k), v(s, k + 1), v(next, k + 1), v(next, k)];
            if cut.is_some() && k == 0 && s == 0 {
                ring.insert(1, v(0, 8));
            }
            if cut.is_some() && k == 0 && next == n {
                ring.insert(3, v(n, 8));
            }
            faces.push(FaceSpec { id: id("stamp.pocket.sweep", s, k), surface: surface.moved(&frame), loops: vec![ring] });
        }
        if let Some(center) = axis {
            for (k, p) in section.into_iter().enumerate() {
                let a = pt(s, p);
                let b = pt(next, p);
                let center = center + z * p.y;
                add_arc(v(s, k), v(next, k), center, a, b, length(a - center))?;
            }
        }
    }

    let plane = |p, normal| Surface::Plane(Plane { origin: frame.apply(p), normal: frame.apply_vector(normal) });
    let chain = |k| (0..=n).map(|s| v(s, k)).collect::<Vec<_>>();
    for (k, normal) in [(3, -z), (4, z)] {
        let mut ring = chain(k);
        if k == 3 {
            ring.reverse();
        }
        faces.push(FaceSpec { id: id("stamp.pocket.shelf", 0, k), surface: plane(pt(0, section[k]), normal), loops: vec![ring] });
    }
    if cut.is_some() {
        let normal = unit((stations[0].point - stations[n].point).cross(z));
        for s in [0, n] {
            let mut ring = vec![v(s, 8)];
            ring.extend((1..=6).map(|k| v(s, k)));
            ring.push(v(s, 7));
            if s == 0 {
                ring.reverse();
            }
            faces.push(FaceSpec { id: id("stamp.pocket.cap", s, 0), surface: plane(stations[s].point, normal), loops: vec![ring] });
        }
        for (k, ring) in [(0, vec![v(n, 3), v(0, 3), v(0, 4), v(n, 4)]), (1, vec![v(0, 0), v(n, 0), v(n, 8), v(0, 8)])] {
            faces.push(FaceSpec {
                id: id("stamp.pocket.slit", 0, k),
                surface: plane(stations[0].point, if k == 0 { normal } else { -normal }),
                loops: vec![ring],
            });
        }
    }
    let (top, bottom) = super::stitch::wall_faces_in(wall, face, t, frame, [[lo.y, hi.y], [lo.x, hi.x]], (lo, hi), || {
        bad("the pocket needs a clear opening in a flat wall")
    })?;
    let mut upper = chain(7);
    upper.reverse();
    if cut.is_some() {
        upper.extend([v(0, 8), v(n, 8)]);
    }
    faces[top].loops.push(upper);
    faces[bottom].loops.push(chain(0));
    let body = build_with_curves(vertices, faces, curves)?;
    validated(&body)?;
    let stamp_cuts = cut
        .map(|cut| StampCut {
            faces: [face, wall.faces()[bottom].id],
            points: [stations[0].point, stations[n].point].map(|p| frame.apply(p)).into(),
            normal: frame.apply_vector(z),
            ids: vec![TopoId::derive("stamp.cut").id(feature).tag(cut).finish()],
            arcs: vec![None],
        })
        .into_iter()
        .collect();
    Ok(Value::Formed(Box::new(Formed { body, blank: wall.clone(), imprints: Vec::new(), stamp_cuts })))
}
