//! A cut of sheet metal across its bend lines (D197): the profiles of a sketch on a wall laid on
//! the blank, the sheet before bending, and punched through it there: across a wall along its
//! normal, across a bend along the radius, on into the next wall. Bends lie flat at the sketch's
//! K-factor as [`crate::unfold`] lays them, so the blank shows the profile as it was drawn.
//!
//! The blank's coordinates `(s, y)`: `y` along the bend lines from the sketch's origin, `s` across
//! them, lengths along each wall and along each bend's neutral layer; on the sketch's wall they
//! are its own coordinates turned. Each profile is cut at the bend lines into pieces, one face
//! each; the faces of both sides it crosses are built anew, their bend lines broken where it
//! crosses, and every piece gets its wall. A piece of a curve after the first, and the next arc
//! of a circle, take the wall [`next_piece`] of the one before: the blank joins them back into the
//! curve drawn, a hole across a bend into one circle.

use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::{PI, TAU};

use datum_brep::{
    Arc, Body, BrepError, Curve, CurveSpec, Cylinder, Development, FaceSpec, Ix, Plane, ProfileArc, ProfileLoop, Rolled, Surface, TopoId, Vertex,
    build_with_curves,
};
use datum_math::{Sign, Vec2, Vec3, inside_circle, orient2d};

use crate::cut::{Reach, from_face};
use crate::sketch::within_rounding;
use crate::sketch::{Placed, opposite_cylinder, profile_loops};
use crate::unfold::{chain_through, next_piece};

/// The profiles of `sketch`, which lies on a face of sheet metal `target`, cut through the sheet:
/// within the face as [`from_face`] cuts, or on across bend lines through the blank. `grow`:
/// added to it instead (D222), the blank growing by the profiles off its metal: a wall goes on in
/// its plane, so a profile meets an edge of the blank and crosses no bend line.
pub(crate) fn punch(feature: TopoId, target: &Body, sketch: &Placed, grow: bool) -> Result<Body, BrepError> {
    let id = sketch.face.ok_or_else(|| BrepError::Sketch("a cut of sheet metal takes a sketch on a face of it".into()))?;
    let f = target.faces().binary_search_by_key(&id, |f| f.id).map_err(|_| BrepError::LostReference(format!("face {id:?} is not in the body")))?;
    let later = |m: &str| BrepError::Unsupported(format!("adding to sheet metal: {m} comes later"));
    let within = || if grow { Err(later("a sheet without its blank")) } else { from_face(feature, target, sketch, Reach::Sheet) };
    if sketch.developed.is_some() {
        return if grow { Err(later("a rolled sheet")) } else { within() };
    }
    let blank = Blank::new(target, f as Ix, sketch.frame, sketch.k_factor)?;
    let Some(blank) = blank else { return within() };
    let profiles = blank.laid(feature, sketch)?;
    cut(feature, target, &blank, profiles, grow, Some(&within))
}

/// The closed profiles `closed` of sketch `sketch` on a face of sheet metal `target`, and its open
/// lines `lines`, each a cut `kerf` wide along it, its ends square (D235).
pub(crate) fn punch_lines(
    feature: TopoId,
    target: &Body,
    sketch: &Placed,
    closed: Option<&Placed>,
    lines: &[String],
    kerf: f64,
) -> Result<Body, BrepError> {
    let unsupported = |m: &str| BrepError::Unsupported(format!("a cut along a line: {m}"));
    let id = sketch.face.ok_or_else(|| BrepError::Sketch("a cut of sheet metal takes a sketch on a face of it".into()))?;
    let f = target.faces().binary_search_by_key(&id, |f| f.id).map_err(|_| BrepError::LostReference(format!("face {id:?} is not in the body")))?;
    if sketch.developed.is_some() {
        return Err(unsupported("on a rolled sheet comes later"));
    }
    let Some(blank) = Blank::new(target, f as Ix, sketch.frame, sketch.k_factor)? else {
        return Err(unsupported("on a sheet without its blank comes later"));
    };
    let mut profiles = closed.map_or_else(|| Ok(Vec::new()), |c| blank.laid(feature, c))?;
    let [_, x, y, _] = sketch.frame;
    let turn = |p: Vec2| Vec2::new(p.x * x.dot(blank.e) + p.y * y.dot(blank.e), p.x * x.dot(blank.a) + p.y * y.dot(blank.a));
    // Everything a line's end may lie on: the bend lines and the blank's edges.
    let mut edges: Vec<[Vec2; 2]> = blank.seams.iter().map(|sm| [sm.a, sm.b]).collect();
    for o in blank.outline(target)? {
        let m = o.ring.points.len();
        edges.extend((0..m).filter(|&e| o.ring.arc(e).is_none()).map(|e| [o.ring.points[e], o.ring.points[(e + 1) % m]]));
    }
    for name in lines {
        let line = sketch.lines.get(name).ok_or_else(|| unsupported("along an arc comes later"))?;
        let at = |n: &String| sketch.solved.points.get(n).copied().ok_or_else(|| BrepError::Sketch(format!("no point `{n}`")));
        let (a, b) = (turn(at(&line.from)?), turn(at(&line.to)?));
        let (dx, dy) = (b.x - a.x, b.y - a.y);
        let l = libm::hypot(dx, dy);
        if l == 0.0 {
            return Err(BrepError::Degenerate(format!("line `{name}` has no length")));
        }
        let (ux, uy) = (dx / l, dy / l);
        let (nx, ny) = (-uy * 0.5 * kerf, ux * 0.5 * kerf);
        // An end on an edge of the blank or a bend line (a line drawn from a corner) goes on past it,
        // as the laser's beam does: a cut's end square on a corner would only touch it.
        let (past_a, past_b) = (past_edges(a, (ux, uy), &edges, kerf), past_edges(b, (ux, uy), &edges, kerf));
        let (a, b) = (Vec2::new(a.x - ux * past_a, a.y - uy * past_a), Vec2::new(b.x + ux * past_b, b.y + uy * past_b));
        let id = |tag: &str, k: u64, layer: u64| TopoId::derive(tag).id(feature).tag(name).index(k).index(layer).finish();
        profiles.push(ProfileLoop {
            // Counterclockwise: along the line on its right, back on its left.
            points: vec![Vec2::new(a.x - nx, a.y - ny), Vec2::new(b.x - nx, b.y - ny), Vec2::new(b.x + nx, b.y + ny), Vec2::new(a.x + nx, a.y + ny)],
            vertices: (0..4).map(|k| [id("cut.line.v", k, 0), id("cut.line.v", k, 1)]).collect(),
            sides: (0..4).map(|k| id("cut.line.side", k, 0)).collect(),
            arcs: vec![None; 4],
        });
    }
    cut(feature, target, &blank, profiles, false, None)
}

/// How far a cut along a line, `kerf` wide and going the unit way `u`, runs on past its end `p`: past
/// every one of `edges` `p` lies on, so that both its sides cross them. None off them: it ends there.
/// At most [`crate::unfold::KERF_PAST`] kerfs: along an edge no way past it is near.
fn past_edges(p: Vec2, u: (f64, f64), edges: &[[Vec2; 2]], kerf: f64) -> f64 {
    let mut past: f64 = 0.0;
    for [a, b] in edges {
        let (dx, dy) = (b.x - a.x, b.y - a.y);
        let l2 = dx * dx + dy * dy;
        let t = (((p.x - a.x) * dx + (p.y - a.y) * dy) / l2).clamp(0.0, 1.0);
        if dist(p, Vec2::new(a.x + dx * t, a.y + dy * t)) > crate::unfold::ON_KERF {
            continue;
        }
        let l = l2.sqrt();
        let (sin, cos) = (((u.0 * dy - u.1 * dx) / l).abs(), ((u.0 * dx + u.1 * dy) / l).abs());
        // A side crosses the edge this far along, a half kerf off the line; and a bit more.
        past = past.max(0.5 * kerf * (1.0 + cos / sin.max(f64::MIN_POSITIVE)));
    }
    past.min(crate::unfold::KERF_PAST * kerf)
}

/// A corner of sheet metal to relieve: the vertex's ID (the relief's IDs come from it), where it
/// is, and the way away from the sheet there.
pub(crate) struct Corner {
    pub key: TopoId,
    pub at: Vec3,
    pub away: Vec3,
}

/// Round reliefs at corners of sheet metal `target` where flanges meet (D229): a circle of
/// `radius` about each corner (a vertex of plane face `home` of the sheet's front side, with the
/// way `away` from the sheet there), cut through the blank laid at K-factor `k`: the sheet's
/// corner and both bends.
pub(crate) fn relieve(feature: TopoId, target: &Body, home: Ix, corners: &[Corner], radius: f64, k: f64) -> Result<Body, BrepError> {
    let Surface::Plane(p) = target.faces()[home as usize].surface else { return Err(BrepError::Invalid("relief: a corner off a flat face".into())) };
    let z = unit(p.normal);
    let x = unit(if z.x.abs() < 0.9 { Vec3::new(1.0, 0.0, 0.0) } else { Vec3::new(0.0, 1.0, 0.0) }.cross(z));
    let frame = [p.origin, x, z.cross(x), z];
    let blank = Blank::new(target, home, frame, Some(k))?.ok_or_else(|| BrepError::Invalid("relief: a corner of a sheet without bends".into()))?;
    let profiles = corners
        .iter()
        .map(|&Corner { key, at: corner, away }| {
            let c = blank.flat(blank.home, corner);
            let d = blank.flat(blank.home, corner + away);
            let a0 = libm::atan2(d.y - c.y, d.x - c.x);
            // Two arcs about the corner, from either side of the way away from the sheet: the short
            // one off the metal, the long one through it.
            let at = |a: f64| {
                let (sn, cs) = libm::sincos(a);
                Vec2::new(c.x + radius * cs, c.y + radius * sn)
            };
            let id = |tag: &str, k: u64, layer: u64| TopoId::derive(tag).id(feature).id(key).index(k).index(layer).finish();
            ProfileLoop {
                points: vec![at(a0 - 0.3), at(a0 + 0.3)],
                vertices: (0..2).map(|k| [id("relief.v", k, 0), id("relief.v", k, 1)]).collect(),
                sides: (0..2).map(|k| id("relief.side", k, 0)).collect(),
                arcs: vec![Some(ProfileArc { center: c, ccw: true }); 2],
            }
        })
        .collect();
    cut(feature, target, &blank, profiles, false, None)
}

/// `profiles`, on `blank` of `target`, cut through it (`grow`: added): see [`punch`]; a cut within
/// the blank's own face `within` makes.
fn cut(
    feature: TopoId,
    target: &Body,
    blank: &Blank,
    profiles: Vec<ProfileLoop>,
    grow: bool,
    within: Option<&dyn Fn() -> Result<Body, BrepError>>,
) -> Result<Body, BrepError> {
    let later = |m: &str| BrepError::Unsupported(format!("adding to sheet metal: {m} comes later"));
    let loops = blank.outline(target)?;
    // The blank's outline: the loop round the others (the cuts in it).
    let outer = (0..loops.len())
        .max_by(|&a, &b| loops[a].ring.area().abs().total_cmp(&loops[b].ring.area().abs()))
        .ok_or_else(|| BrepError::Invalid("cut across a bend: the blank has no outline".into()))?;
    // Without a K-factor the bends lie flat at no length: the outline beyond the sketch's face is
    // no blank, and a cut off it is refused below.
    let laid = blank.strips.iter().all(|s| !matches!(s.lay, Lay::Bend { neutral: None, .. }));
    // A profile meeting an edge of the blank, or holding an earlier cut, is cut through the
    // blank (D202, D203).
    let holds = |l: &ProfileLoop, o: usize| o != outer && l.winding(loops[o].ring.points[0]) != 0;
    let open = laid && profiles.iter().any(|l| (0..loops.len()).any(|o| l.touches(&loops[o].ring) || holds(l, o)));
    // Within the sketch's face, a cut as before.
    let home = &blank.strips[blank.home];
    let inside = |p: &Vec2| home.lo() < p.x && p.x < home.hi() && blank.strip(*p) == blank.home;
    if let Some(within) = within
        && !grow
        && !open
        && profiles.iter().all(|l| l.points.iter().all(inside) && (0..l.points.len()).all(|k| l.arc(k).is_none_or(|a| arc_within(l, k, a, home))))
    {
        return within();
    }
    let unsupported = |m: &str| BrepError::Unsupported(format!("cut across a bend: {m}"));
    if blank.strips.iter().any(|s| matches!(s.lay, Lay::Bend { neutral: None, .. })) {
        return Err(BrepError::Sketch(
            "a cut across a bend lays the sheet flat at its K-factor: the sketch needs `k_factor` (the part's rule gives it)".into(),
        ));
    }

    // Profiles neither touch nor hold one another, nor an edge of the blank or another cut in it.
    let boxes: Vec<_> = profiles.iter().map(ProfileLoop::bounds).collect();
    for (a, b) in datum_brep::overlapping(&boxes) {
        if profiles[a].touches(&profiles[b]) {
            return Err(BrepError::Degenerate("cut profiles touch".into()));
        }
        if profiles[a].winding(profiles[b].points[0]) != 0 || profiles[b].winding(profiles[a].points[0]) != 0 {
            return Err(unsupported("a profile inside another"));
        }
    }
    // The metal: inside the outline, outside every earlier cut.
    let metal = |p: Vec2| {
        if blank.layered { !blank.strips_at(p).is_empty() } else { (0..loops.len()).all(|o| (loops[o].ring.winding(p) != 0) == (o == outer)) }
    };
    if grow {
        if profiles.iter().any(|l| loops.iter().all(|o| !l.touches(&o.ring))) {
            return Err(BrepError::Sketch("adding to sheet metal: a profile goes on from an edge of the blank, and meets none".into()));
        }
    } else if profiles.iter().any(|l| loops.iter().all(|o| !l.touches(&o.ring)) && !metal(l.points[0]) && !(0..loops.len()).any(|o| holds(l, o))) {
        return Err(unsupported("a profile lies off the sheet"));
    }
    // What of a profile becomes an edge of the sheet: a cut's on the metal, an addition's off it.
    let keep = |p: Vec2| metal(p) != grow;

    // The pieces on the sheet, and where they cross each bend line and each edge of the sheet.
    let mut parts = Parts {
        pieces: Vec::new(),
        seams: vec![Vec::new(); blank.seams.len()],
        rims: loops.iter().map(|o| vec![Vec::new(); o.edges.len()]).collect(),
    };
    for l in &profiles {
        blank.split(feature, l, &loops, &keep, &mut parts)?;
    }
    let Parts { pieces, seams, rims: crossings } = parts;
    // An addition goes on one wall in its plane, the wall one face.
    if grow
        && let Some(p) = pieces.iter().find(|p| {
            let st = &blank.strips[p.strip];
            p.strip != pieces[0].strip || !matches!(st.lay, Lay::Plane { .. }) || st.faces.iter().any(|f| f.len() > 1)
        })
    {
        let st = &blank.strips[p.strip];
        return Err(later(if st.faces.iter().any(|f| f.len() > 1) { "a wall a cut divided" } else { "across a bend line" }));
    }
    let he = |h: Ix| target.half_edges()[h as usize];
    let rims: Vec<Rim> = loops
        .iter()
        .zip(crossings)
        .flat_map(|(o, c)| o.edges.iter().copied().zip(c))
        .filter(|(_, c)| !c.is_empty())
        .map(|((h, strip), crossings)| Rim { h, strip, beyond: target.loops()[he(he(h).twin).loop_ix as usize].face, crossings })
        .collect();
    // A face across the thickness no profile crosses but whose every edge along the sheet a
    // profile takes goes whole (D203): a wall of an earlier cut, an earlier cut held whole; and
    // with it the faces of the strips around.
    let taken = |p: Vec2| profiles.iter().any(|l| covers(l, p));
    let mut along: BTreeMap<Ix, (bool, Vec<usize>)> = BTreeMap::new();
    for o in &loops {
        let m = o.ring.points.len();
        for (e, &(h, i)) in o.edges.iter().enumerate() {
            let t = target.loops()[he(he(h).twin).loop_ix as usize].face;
            let entry = along.entry(t).or_insert((true, Vec::new()));
            entry.0 &= taken(halfway(o.ring.points[e], o.ring.points[(e + 1) % m], o.ring.arc(e)));
            entry.1.push(i);
        }
    }
    let crossed_faces: BTreeSet<Ix> = rims.iter().map(|r| r.beyond).collect();
    let gone: BTreeSet<Ix> = along.iter().filter(|(t, (all, _))| *all && !crossed_faces.contains(t)).map(|(&t, _)| t).collect();
    let around: BTreeSet<usize> = gone.iter().flat_map(|t| along[t].1.iter().copied()).collect();

    // The body: new vertices and curves, the walls, the faces crossed built anew.
    let mut vertices: Vec<Vertex> = target.vertices().to_vec();
    // Each new vertex once: a piece's start, or its end where the next piece is off the sheet.
    let mut new: BTreeMap<TopoId, Vec3> = BTreeMap::new();
    let mut curves = target.curve_specs();
    let mut faces: Vec<FaceSpec> = Vec::new();
    for p in &pieces {
        let st = &blank.strips[p.strip];
        for layer in 0..2 {
            new.insert(p.ends[0][layer], blank.point(p.strip, p.from, layer));
            new.insert(p.ends[1][layer], blank.point(p.strip, p.to, layer));
        }
        let (v, w) = (p.ends[0], p.ends[1]);
        let surface = match &st.lay {
            Lay::Plane { n, .. } => match p.arc {
                // Out of the body: into the hole a cut makes, out of what an addition adds.
                None => {
                    let a = blank.point(p.strip, p.from, 0);
                    let across = n.cross(blank.point(p.strip, p.to, 0) - a);
                    Surface::Plane(Plane { origin: a, normal: if grow { -across } else { across } })
                }
                Some(arc) => {
                    for layer in 0..2 {
                        let axis = if arc.ccw { *n } else { -*n };
                        let arc3 = Arc { center: blank.point(p.strip, arc.center, layer), axis, radius: p.radius };
                        curves.push(CurveSpec { from: v[layer], to: w[layer], curve: Curve::Arc(arc3) });
                    }
                    // A cut's body is outside the hole: concave around it; an addition's within.
                    Surface::Cylinder(Cylinder { origin: blank.point(p.strip, arc.center, 1), axis: *n, radius: p.radius, convex: arc.ccw == grow })
                }
            },
            Lay::Bend { dev, map, radii, .. } => {
                let uv = |q: Vec2| map.uv(q);
                let piece = Rolled {
                    dev: *dev,
                    from: uv(p.from),
                    to: uv(p.to),
                    arc: p.arc.map(|a| ProfileArc { center: uv(a.center), ccw: a.ccw != map.mirrored() }),
                    trace: None,
                };
                for layer in 0..2 {
                    curves.push(CurveSpec { from: v[layer], to: w[layer], curve: Curve::Rolled { piece, radius: radii[layer] } });
                }
                // Out of the body (into the hole) is left of the profile on the blank: of the
                // development too unless it is mirrored.
                Surface::Wall(if map.mirrored() { piece.reversed() } else { piece })
            }
        };
        let around = if grow { vec![v[0], v[1], w[1], w[0]] } else { vec![v[0], w[0], w[1], v[1]] };
        faces.push(FaceSpec { id: p.wall, surface, loops: vec![around] });
    }
    // Vertices of the body are kept as they are; a new one is a new point.
    vertices.extend(
        new.into_iter().filter(|(id, _)| target.vertices().binary_search_by_key(id, |v| v.id).is_err()).map(|(id, point)| Vertex { id, point }),
    );
    let cuts = Cuts { open: !rims.is_empty() || !gone.is_empty(), grow, pieces, seams, rims, profiles };
    // The curves of curved edges cut in pieces, a piece each.
    let mut split: Vec<CurveSpec> = Vec::new();
    let crossed: BTreeSet<usize> = cuts.pieces.iter().map(|p| p.strip).chain(cuts.rims.iter().map(|r| r.strip)).chain(around).collect();
    let mut rebuilt: BTreeSet<Ix> = BTreeSet::new();
    for &i in &crossed {
        for side in 0..2 {
            faces.extend(blank.rebuilt(target, i, side, &cuts, &mut split)?);
            rebuilt.extend(blank.strips[i].faces[side].iter().copied());
        }
    }
    // The faces across the sheet's thickness the profiles cross: its edges and ends.
    let beyond: BTreeSet<Ix> = cuts.rims.iter().map(|r| r.beyond).collect();
    for &t in &beyond {
        faces.extend(blank.rim_face(target, t, &cuts, &split)?);
        rebuilt.insert(t);
    }
    rebuilt.extend(gone);
    curves.extend(split);
    faces.extend((0..target.faces().len() as Ix).filter(|x| !rebuilt.contains(x)).map(|x| target.face_spec(x)));
    // On a blank lying on itself what a cut parts from the sheet falls off, as it does on the laser:
    // that is how the overlap is settled (D233), the part with the sketch's face stays. Elsewhere
    // a sheet in two parts is no one part.
    let mut parts = connected(&faces);
    if parts.len() > 1 {
        if cuts.grow || !blank.layered {
            return Err(unsupported("the cut divides the sheet into two parts: comes later"));
        }
        let home = target.faces()[blank.strips[blank.home].faces[0][0] as usize].id;
        let most = (0..parts.len()).max_by_key(|&k| parts[k].len()).unwrap_or(0);
        let stays = parts.iter().position(|part| part.iter().any(|&x| faces[x].id == home)).unwrap_or(most);
        let kept: BTreeSet<usize> = parts.swap_remove(stays).into_iter().collect();
        faces = faces.into_iter().enumerate().filter(|(x, _)| kept.contains(x)).map(|(_, f)| f).collect();
    }
    if cuts.open {
        // What a profile took whole: its vertices and curves go with it.
        let mut used: BTreeSet<TopoId> = BTreeSet::new();
        let mut edges: BTreeSet<(TopoId, TopoId)> = BTreeSet::new();
        for l in faces.iter().flat_map(|f| f.loops.iter()) {
            for (k, &v) in l.iter().enumerate() {
                let w = l[(k + 1) % l.len()];
                used.insert(v);
                edges.insert((v.min(w), v.max(w)));
            }
        }
        vertices.retain(|v| used.contains(&v.id));
        curves.retain(|c| edges.contains(&(c.from.min(c.to), c.from.max(c.to))));
    }
    build_with_curves(vertices, faces, curves)
}

/// How close a profile of a cut of sheet metal comes to a bend: `distance` on the blank from it
/// to the bend's nearer bend line, or none when it reaches into the bend; under `clearance`,
/// 2t + R (R the bend's inner radius). Closer than that a hole distorts on bending, as process
/// engineers check before a blank is cut. Only its edges that would: a line square to the bend
/// lines (a slot or a relief across the bend) keeps its shape, and so does an edge beside which
/// the profile cuts the bend away across its width: nothing there bends.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NearBend {
    pub distance: Option<f64>,
    pub clearance: f64,
}

/// The profiles of `sketch`, a cut of sheet metal `target`, closer to a bend than [`NearBend`]
/// allows; none on a sheet without bends or on a rolled one's development.
pub(crate) fn near_bends(feature: TopoId, target: &Body, sketch: &Placed) -> Result<Vec<NearBend>, BrepError> {
    let (Some(id), None) = (sketch.face, &sketch.developed) else { return Ok(Vec::new()) };
    let Ok(f) = target.faces().binary_search_by_key(&id, |f| f.id) else { return Ok(Vec::new()) };
    let Some(blank) = Blank::new(target, f as Ix, sketch.frame, sketch.k_factor)? else { return Ok(Vec::new()) };
    let mut near = Vec::new();
    for l in blank.laid(feature, sketch)? {
        // The edges that distort: arcs, and lines not square to the bend lines (along the
        // blank's Y), decided exactly: a slant by rounding is told of, never let pass.
        let boxes: Vec<(Vec2, Vec2)> = (0..l.points.len()).map(|k| reach(&l, k)).collect();
        let distorting: Vec<(Vec2, Vec2)> =
            (0..l.points.len()).filter(|&k| l.arc(k).is_some() || l.points[k].y != l.points[(k + 1) % l.points.len()].y).map(|k| boxes[k]).collect();
        // Whether the profile cuts the bend from `lo` to `hi` away across its width over `y0`…`y1`:
        // no edge of it inside that band (so its middle lies on none), and the middle within it.
        let cuts_away = |lo: f64, hi: f64, y0: f64, y1: f64| {
            y0 < y1
                && boxes.iter().all(|(a, b)| b.x <= lo || a.x >= hi || b.y <= y0 || a.y >= y1)
                && l.winding(Vec2::new(0.5 * (lo + hi), 0.5 * (y0 + y1))) != 0
        };
        let nearest = blank
            .strips
            .iter()
            .filter_map(|s| match s.lay {
                Lay::Bend { radii, .. } => {
                    let gap = distorting
                        .iter()
                        .filter(|(a, b)| !cuts_away(s.lo(), s.hi(), a.y, b.y))
                        .map(|(a, b)| (s.lo() - b.x).max(a.x - s.hi()))
                        .min_by(f64::total_cmp)?;
                    Some((gap, 2.0 * blank.thickness + radii[0].min(radii[1])))
                }
                Lay::Plane { .. } => None,
            })
            .min_by(|a, b| a.0.total_cmp(&b.0));
        match nearest {
            Some((gap, clearance)) if gap <= 0.0 => near.push(NearBend { distance: None, clearance }),
            Some((gap, clearance)) if gap < clearance => near.push(NearBend { distance: Some(gap), clearance }),
            _ => {}
        }
    }
    Ok(near)
}

/// The box edge `k` of loop `l` lies in: its ends and, an arc, its extremes.
fn reach(l: &ProfileLoop, k: usize) -> (Vec2, Vec2) {
    let n = l.points.len();
    let (a, b) = (l.points[k], l.points[(k + 1) % n]);
    let (mut lo, mut hi) = (Vec2::new(a.x.min(b.x), a.y.min(b.y)), Vec2::new(a.x.max(b.x), a.y.max(b.y)));
    if let Some(arc) = l.arc(k) {
        let c = arc.center;
        let angle = |p: Vec2| libm::atan2(p.y - c.y, p.x - c.x);
        // The turn from `a` toward `b` the arc's way, and whether it passes direction `t`.
        let turn = |from: f64, to: f64| {
            let d = if arc.ccw { to - from } else { from - to };
            if d <= 0.0 { d + TAU } else { d }
        };
        let (start, sweep) = (angle(a), turn(angle(a), angle(b)));
        let r = libm::hypot(a.x - c.x, a.y - c.y);
        if turn(start, 0.0) <= sweep {
            hi.x = hi.x.max(c.x + r);
        }
        if turn(start, 0.5 * PI) <= sweep {
            hi.y = hi.y.max(c.y + r);
        }
        if turn(start, PI) <= sweep {
            lo.x = lo.x.min(c.x - r);
        }
        if turn(start, -0.5 * PI) <= sweep {
            lo.y = lo.y.min(c.y - r);
        }
    }
    (lo, hi)
}

/// The crossings of profiles with one edge of the blank: each point with its vertices on both
/// sides of the sheet.
type Crossings = Vec<(Vec2, [TopoId; 2])>;

/// What splitting the profiles gives: their pieces on the metal, and their crossings with each
/// bend line and with each edge of each loop of the blank.
struct Parts {
    pieces: Vec<Piece>,
    seams: Vec<Vec<(Vec2, [TopoId; 2])>>,
    rims: Vec<Vec<Crossings>>,
}

/// A point a profile stops at on the blank: where it is, and the vertices of the metal there, each
/// with the strips they are of.
type StopOn = (Vec2, Vec<(Vec<usize>, [TopoId; 2])>);

/// A line a profile is cut at: a bend line, or an edge of the blank's outline.
#[derive(Clone, Copy)]
enum Line {
    Bend(usize),
    /// Edge `e` of loop `o` of the blank.
    Rim(usize, usize),
}

/// Where segment `a`–`b` crosses segment `p`–`q`: its share of the way from `a`, and the point.
/// Which side each end lies on is decided exactly; an end on the other segment, or the two along
/// one line, is a contact the cut does not resolve.
fn segment_crossing(a: Vec2, b: Vec2, p: Vec2, q: Vec2) -> Result<Option<(f64, Vec2)>, BrepError> {
    let (sa, sb, sp, sq) = (orient2d(p, q, a), orient2d(p, q, b), orient2d(a, b, p), orient2d(a, b, q));
    let apart = |x: Sign, y: Sign| x != Sign::Zero && y != Sign::Zero && x != y;
    if apart(sa, sb) && apart(sp, sq) {
        let (d, e) = (Vec2::new(b.x - a.x, b.y - a.y), Vec2::new(q.x - p.x, q.y - p.y));
        let t = ((p.x - a.x) * e.y - (p.y - a.y) * e.x) / (d.x * e.y - d.y * e.x);
        return Ok(Some((t, Vec2::new(a.x + d.x * t, a.y + d.y * t))));
    }
    // An end of the profile's segment on the edge is a corner of the profile there: see
    // [`on_edge`].
    let on = |s: Sign, x: Sign, y: Sign| s == Sign::Zero && x != y;
    if on(sp, sa, sb) || on(sq, sa, sb) || [sa, sb, sp, sq].iter().all(|&s| s == Sign::Zero) {
        let boxes = |u: Vec2, v: Vec2| (Vec2::new(u.x.min(v.x), u.y.min(v.y)), Vec2::new(u.x.max(v.x), u.y.max(v.y)));
        if datum_brep::overlap(boxes(a, b), boxes(p, q)) {
            return Err(BrepError::Degenerate("a profile touches an edge of the sheet at a corner, or runs along it".into()));
        }
    }
    Ok(None)
}

/// Whether segment `a`–`b` runs along segment `p`–`q`: on its line, the two overlapping.
fn runs_along(a: Vec2, b: Vec2, p: Vec2, q: Vec2) -> bool {
    let boxes = |u: Vec2, v: Vec2| (Vec2::new(u.x.min(v.x), u.y.min(v.y)), Vec2::new(u.x.max(v.x), u.y.max(v.y)));
    orient2d(p, q, a) == Sign::Zero && orient2d(p, q, b) == Sign::Zero && datum_brep::overlap(boxes(a, b), boxes(p, q))
}

/// Whether profile `l` takes point `p` of the blank: inside it, or on a line of it (an edge of
/// the sheet a profile runs along goes with it).
fn covers(l: &ProfileLoop, p: Vec2) -> bool {
    let n = l.points.len();
    let on_line = (0..n).any(|k| {
        let (a, b) = (l.points[k], l.points[(k + 1) % n]);
        l.arc(k).is_none()
            && orient2d(a, b, p) == Sign::Zero
            && a.x.min(b.x) <= p.x
            && p.x <= a.x.max(b.x)
            && a.y.min(b.y) <= p.y
            && p.y <= a.y.max(b.y)
    });
    on_line || l.winding(p) != 0
}

/// Where the arc from `a` to `b` (of radius `r`) crosses segment `p`–`q`: each point with its
/// share of the arc's turn from `a`. A tangent, or a crossing at an end of either, is a contact
/// the cut does not resolve.
fn arc_segment_crossings(a: Vec2, b: Vec2, arc: ProfileArc, r: f64, p: Vec2, q: Vec2) -> Result<Vec<(f64, Vec2)>, BrepError> {
    let touch = || BrepError::Degenerate("a profile touches an edge of the sheet".into());
    let (u, c) = (Vec2::new(q.x - p.x, q.y - p.y), arc.center);
    let l2 = u.x * u.x + u.y * u.y;
    let f = ((c.x - p.x) * u.x + (c.y - p.y) * u.y) / l2;
    let foot = Vec2::new(p.x + u.x * f, p.y + u.y * f);
    let d2 = (c.x - foot.x).powi(2) + (c.y - foot.y).powi(2);
    let total = arc.turn(a, b);
    if d2 > r * r {
        return Ok(Vec::new());
    }
    // A tangent of the line through the edge counts only on the edge and on the arc.
    if d2 == r * r {
        return if (0.0..=1.0).contains(&f) && arc.turn(a, foot) / total <= 1.0 { Err(touch()) } else { Ok(Vec::new()) };
    }
    let h = ((r * r - d2) / l2).sqrt();
    let mut found: Vec<(f64, Vec2)> = [f - h, f + h].into_iter().map(|s| (s, Vec2::new(p.x + u.x * s, p.y + u.y * s))).collect();
    // An end of the arc on the edge's line is one of the two: a corner of the profile there (see
    // [`on_edge`]), not a crossing.
    for end in [a, b] {
        if orient2d(p, q, end) == Sign::Zero {
            let near = (0..found.len()).min_by(|&i, &j| dist(found[i].1, end).total_cmp(&dist(found[j].1, end)));
            if let Some(i) = near {
                found.remove(i);
            }
        }
    }
    let mut out = Vec::new();
    for (s, x) in found {
        if !(0.0..=1.0).contains(&s) {
            continue;
        }
        let t = arc.turn(a, x) / total;
        if t < 1.0 {
            if s == 0.0 || s == 1.0 {
                return Err(touch());
            }
            out.push((t, x));
        }
    }
    Ok(out)
}

fn dist(a: Vec2, b: Vec2) -> f64 {
    libm::hypot(a.x - b.x, a.y - b.y)
}

/// The edge of the blank (loop, edge) corner `a` of a profile lies on, exactly: on a line by
/// `orient2d`, on an arc by `inside_circle`; a corner on a corner of the blank is a contact the
/// cut does not resolve.
fn on_edge(a: Vec2, loops: &[Outline]) -> Result<Option<(usize, usize)>, BrepError> {
    for (o, outline) in loops.iter().enumerate() {
        let ring = &outline.ring;
        let m = ring.points.len();
        for e in 0..m {
            let (p, q) = (ring.points[e], ring.points[(e + 1) % m]);
            let (on, within) = match ring.arc(e) {
                None => {
                    let along = (a.x - p.x) * (q.x - p.x) + (a.y - p.y) * (q.y - p.y);
                    (orient2d(p, q, a) == Sign::Zero, 0.0 < along && along < (q.x - p.x).powi(2) + (q.y - p.y).powi(2))
                }
                Some(d) => (inside_circle(d.center, p, a) == Sign::Zero, (d.turn(p, a) / d.turn(p, q)) < 1.0),
            };
            if !on {
                continue;
            }
            if a == p || a == q {
                return Err(BrepError::Degenerate("a corner of a profile lies on a corner of the sheet or of a cut".into()));
            }
            if within {
                return Ok(Some((o, e)));
            }
        }
    }
    Ok(None)
}

/// Halfway from `from` to `to`: along the line, or along the arc.
fn halfway(from: Vec2, to: Vec2, arc: Option<ProfileArc>) -> Vec2 {
    match arc {
        None => Vec2::new(0.5 * (from.x + to.x), 0.5 * (from.y + to.y)),
        Some(c) => {
            let (sn, cs) = libm::sincos(0.5 * c.turn(from, to));
            let (dx, dy) = (from.x - c.center.x, from.y - c.center.y);
            Vec2::new(c.center.x + dx * cs - dy * sn, c.center.y + dx * sn + dy * cs)
        }
    }
}

/// Points of `found` without the one nearest each of `ends` that lies on the circle about
/// `center` through `through` (exactly): that one is the end itself, a corner of the profile
/// there (see [`on_edge`]), not a crossing.
fn without_ends(mut found: Vec<Vec2>, ends: [Vec2; 2], center: Vec2, through: Vec2) -> Vec<Vec2> {
    for end in ends {
        if inside_circle(center, through, end) == Sign::Zero
            && let Some(i) = (0..found.len()).min_by(|&i, &j| dist(found[i], end).total_cmp(&dist(found[j], end)))
        {
            found.remove(i);
        }
    }
    found
}

/// Where segment `a`–`b` crosses the arc from `p` to `q` (`arc`), each point with its share of the
/// way from `a`. An end of the arc on the segment, or a tangent, is a contact the cut does not
/// resolve.
fn segment_arc_crossings(a: Vec2, b: Vec2, p: Vec2, q: Vec2, arc: ProfileArc) -> Result<Vec<(f64, Vec2)>, BrepError> {
    let touch = || BrepError::Degenerate("a profile touches the edge of an earlier cut".into());
    for end in [p, q] {
        if orient2d(a, b, end) == Sign::Zero && dist(a, end) + dist(end, b) <= dist(a, b) {
            return Err(touch());
        }
    }
    let (u, c, r) = (Vec2::new(b.x - a.x, b.y - a.y), arc.center, dist(p, arc.center));
    let l2 = u.x * u.x + u.y * u.y;
    let f = ((c.x - a.x) * u.x + (c.y - a.y) * u.y) / l2;
    let foot = Vec2::new(a.x + u.x * f, a.y + u.y * f);
    let d2 = (c.x - foot.x).powi(2) + (c.y - foot.y).powi(2);
    if d2 > r * r {
        return Ok(Vec::new());
    }
    // A tangent of the line through the segment counts only on the segment and on the arc.
    if d2 == r * r {
        return if (0.0..=1.0).contains(&f) && arc.turn(p, foot) / arc.turn(p, q) <= 1.0 { Err(touch()) } else { Ok(Vec::new()) };
    }
    let h = ((r * r - d2) / l2).sqrt();
    let found = without_ends([f - h, f + h].map(|t| Vec2::new(a.x + u.x * t, a.y + u.y * t)).to_vec(), [a, b], c, p);
    let span = arc.turn(p, q);
    let along = |x: Vec2| ((x.x - a.x) * u.x + (x.y - a.y) * u.y) / l2;
    Ok(found.into_iter().map(|x| (along(x), x)).filter(|&(t, x)| 0.0 < t && t < 1.0 && arc.turn(p, x) / span < 1.0).collect())
}

/// Where the arc from `a` to `b` (`arc`, radius `r`) crosses the arc from `p` to `q` (`other`),
/// each point with its share of the first arc's turn. An end of the other on the first, or a
/// tangent, is a contact the cut does not resolve.
#[allow(clippy::too_many_arguments)]
fn arc_arc_crossings(a: Vec2, b: Vec2, arc: ProfileArc, r: f64, p: Vec2, q: Vec2, other: ProfileArc) -> Result<Vec<(f64, Vec2)>, BrepError> {
    let touch = || BrepError::Degenerate("a profile touches the edge of an earlier cut".into());
    let (c1, c2, r2) = (arc.center, other.center, dist(p, other.center));
    let d = dist(c1, c2);
    if d == 0.0 || d > r + r2 || d < (r - r2).abs() {
        return Ok(Vec::new());
    }
    let total = arc.turn(a, b);
    // Circles touching count only where both arcs are.
    if d == r + r2 || d == (r - r2).abs() {
        let toward = if d == r + r2 || r > r2 { 1.0 } else { -1.0 };
        let x = Vec2::new(c1.x + (c2.x - c1.x) / d * r * toward, c1.y + (c2.y - c1.y) / d * r * toward);
        return if arc.turn(a, x) / total <= 1.0 && other.turn(p, x) / other.turn(p, q) <= 1.0 { Err(touch()) } else { Ok(Vec::new()) };
    }
    for end in [p, q] {
        if inside_circle(c1, a, end) == Sign::Zero && arc.turn(a, end) / total < 1.0 {
            return Err(touch());
        }
    }
    let k = (r * r - r2 * r2 + d * d) / (2.0 * d);
    let h = (r * r - k * k).max(0.0).sqrt();
    let u = Vec2::new((c2.x - c1.x) / d, (c2.y - c1.y) / d);
    let f = Vec2::new(c1.x + u.x * k, c1.y + u.y * k);
    let found = without_ends(vec![Vec2::new(f.x - u.y * h, f.y + u.x * h), Vec2::new(f.x + u.y * h, f.y - u.x * h)], [a, b], c2, p);
    let span = other.turn(p, q);
    Ok(found.into_iter().map(|x| (arc.turn(a, x) / total, x)).filter(|&(t, x)| t < 1.0 && other.turn(p, x) / span < 1.0).collect())
}

/// Where the segment or arc from `a` to `b` (radius `radius`) crosses bend line `p`–`q`, which runs
/// along an axis of the blank (the chain's across X, a flange's on the side along it): each point
/// with its share of the way, on the line exactly. A tangent is refused.
fn across_seam(a: Vec2, b: Vec2, arc: Option<ProfileArc>, radius: f64, p: Vec2, q: Vec2) -> Result<Vec<(f64, Vec2)>, BrepError> {
    // As if the line ran along X = s: a horizontal one with the axes swapped.
    let swap = p.x != q.x;
    let f = |v: Vec2| if swap { Vec2::new(v.y, v.x) } else { v };
    let (a, b, p, q) = (f(a), f(b), f(p), f(q));
    let s = p.x;
    let found: Vec<(f64, Vec2)> = match arc {
        None => {
            let (da, db) = (a.x - s, b.x - s);
            if (da < 0.0) == (db < 0.0) {
                Vec::new()
            } else {
                let t = da / (da - db);
                vec![(t, Vec2::new(s, a.y + t * (b.y - a.y)))]
            }
        }
        Some(c) => {
            let c = ProfileArc { center: f(c.center), ccw: c.ccw != swap };
            if (s - c.center.x).abs() == radius {
                return Err(BrepError::Degenerate("a profile touches a bend line".into()));
            }
            arc_crossings(a, b, c, radius, s)
        }
    };
    let (lo, hi) = (p.y.min(q.y), p.y.max(q.y));
    Ok(found.into_iter().filter(|(_, x)| lo <= x.y && x.y <= hi).map(|(t, x)| (t, f(x))).collect())
}

/// Whether arc `k` of `l` stays between the bend lines of `strip`: it crosses neither.
fn arc_within(l: &ProfileLoop, k: usize, arc: ProfileArc, strip: &Strip) -> bool {
    let (a, b) = (l.points[k], l.points[(k + 1) % l.points.len()]);
    let r = libm::hypot(a.x - arc.center.x, a.y - arc.center.y);
    [strip.lo(), strip.hi()].iter().all(|&s| arc_crossings(a, b, arc, r, s).is_empty())
}

/// Where the arc from `a` to `b` crosses the line `x = s`: its points there, each with its share
/// of the arc's turn from `a`. A tangent counts as a crossing, so it is refused.
fn arc_crossings(a: Vec2, b: Vec2, arc: ProfileArc, r: f64, s: f64) -> Vec<(f64, Vec2)> {
    let dx = s - arc.center.x;
    if dx.abs() > r {
        return Vec::new();
    }
    let dy = (r * r - dx * dx).sqrt();
    let total = arc.turn(a, b);
    let mut out: Vec<(f64, Vec2)> = [dy, -dy]
        .into_iter()
        .map(|d| Vec2::new(s, arc.center.y + d))
        .filter(|&p| p != a)
        .map(|p| (arc.turn(a, p) / total, p))
        .filter(|&(t, _)| t < 1.0)
        .collect();
    out.dedup_by(|x, y| x.1 == y.1);
    out
}

/// A piece of a profile within one face: from `from` to `to` on the blank, the arc as the
/// profile's (radius `radius`), its vertices at both ends on both sides of the sheet, its wall.
struct Piece {
    from: Vec2,
    to: Vec2,
    /// Halfway along it: which face of its strip it lies in.
    middle: Vec2,
    arc: Option<ProfileArc>,
    radius: f64,
    strip: usize,
    ends: [[TopoId; 2]; 2],
    wall: TopoId,
}

/// Faces of the sheet laid flat alike: the faces of the sketch's side and the faces across them,
/// one face each or the pieces a cut divided it into (D201). The chain's strips, between two bend
/// lines, lie from `lo` to `hi` in `s` (`span`); a flange on the sheet's side (D205) lies across
/// them from the wall it stands on, where its faces' outlines are (`region`).
struct Strip {
    faces: [Vec<Ix>; 2],
    span: Option<(f64, f64)>,
    region: Vec<ProfileLoop>,
    /// The holes in `region`, on a blank that lies on itself (see `Blank::layered`).
    holes: Vec<ProfileLoop>,
    lay: Lay,
}

impl Strip {
    /// Where a strip of the chain begins and ends in `s` (a flange across has neither).
    fn lo(&self) -> f64 {
        self.span.map_or(f64::NAN, |s| s.0)
    }

    fn hi(&self) -> f64 {
        self.span.map_or(f64::NAN, |s| s.1)
    }
}

enum Lay {
    /// A wall: its unit normal out of the sketch's side, the unit `ex` and `ey` along the blank's
    /// X and Y, and `base`, the point at `at` on the blank.
    Plane { n: Vec3, ex: Vec3, ey: Vec3, base: Vec3, at: Vec2 },
    /// A bend: its development (`map` from the blank to `(u, v)`, `u` from the bend line it is
    /// laid from), `near` halfway along it in `u`, and the radii of the sketch's side and of the
    /// other. No development without a K-factor.
    Bend { dev: Development, map: UvMap, near: f64, radii: [f64; 2], neutral: Option<f64> },
}

/// From the blank to a development and back: `(u, v)` is `s` times the offset from `o`, its X and
/// Y swapped for a bend across the chain (a flange on the sheet's side).
#[derive(Clone, Copy)]
struct UvMap {
    swap: bool,
    o: Vec2,
    s: [f64; 2],
}

impl UvMap {
    fn uv(&self, p: Vec2) -> Vec2 {
        let (a, b) = if self.swap { (p.y - self.o.y, p.x - self.o.x) } else { (p.x - self.o.x, p.y - self.o.y) };
        Vec2::new(self.s[0] * a, self.s[1] * b)
    }

    fn blank(&self, uv: Vec2) -> Vec2 {
        let (a, b) = (self.s[0] * uv.x, self.s[1] * uv.y);
        let (dx, dy) = if self.swap { (b, a) } else { (a, b) };
        Vec2::new(self.o.x + dx, self.o.y + dy)
    }

    /// Whether the development is the blank mirrored: turning senses swap.
    fn mirrored(&self) -> bool {
        (self.s[0] * self.s[1] < 0.0) != self.swap
    }
}

/// A bend line between strips `strips`, from `a` to `b` on the blank: along one of its axes.
struct Seam {
    a: Vec2,
    b: Vec2,
    strips: [usize; 2],
}

/// A closed loop of the blank's edges (its outline or a cut in it), and for each edge the
/// half-edge of a face of the sketch's side it is and that face's strip.
struct Outline {
    ring: ProfileLoop,
    edges: Vec<(Ix, usize)>,
}

/// Where profiles cross an edge of the blank's outline (D202): the half-edge of the sketch's
/// side, its strip, the face beyond it (an edge or end of the sheet, across its thickness), and
/// the crossings, each with its vertices on both sides of the sheet.
struct Rim {
    h: Ix,
    strip: usize,
    beyond: Ix,
    crossings: Vec<(Vec2, [TopoId; 2])>,
}

/// What the profiles do to the sheet: their pieces on it, where they cross each bend line and
/// each edge of the sheet, and the profiles themselves (on the blank).
struct Cuts {
    /// Profiles meet edges of the blank or hold earlier cuts: edges may go whole (D202, D203).
    open: bool,
    /// Added to the sheet, not cut from it (see [`punch`]).
    grow: bool,
    pieces: Vec<Piece>,
    seams: Vec<Vec<(Vec2, [TopoId; 2])>>,
    rims: Vec<Rim>,
    profiles: Vec<ProfileLoop>,
}

/// The sheet laid flat from the sketch's face: the chain's faces in order of `s`, then the flanges
/// on its sides; the bend lines between them.
struct Blank {
    a: Vec3,
    e: Vec3,
    thickness: f64,
    strips: Vec<Strip>,
    seams: Vec<Seam>,
    home: usize,
    /// Strips not joined may lie on one another (flanges into a window, or meeting at a corner):
    /// every strip then goes by its faces' outlines alone, none by a span, and one point of the
    /// blank may be on several. A cut goes through all of them, as a laser through what is stacked.
    layered: bool,
}

impl Blank {
    /// The blank of the sheet `target` whose face `f` the sketch lies on, by the same tree of strips
    /// the flat blank is laid by (D233), from that face in the sketch's frame; none for a sheet
    /// without bends. Bend lines must run along the blank's axes. Without a K-factor the strips off
    /// the sketch's face have no true place: a cut reaching them is refused.
    fn new(target: &Body, f: Ix, frame: [Vec3; 4], k: Option<f64>) -> Result<Option<Self>, BrepError> {
        let unsupported = |m: &str| BrepError::Unsupported(format!("cut across a bend: {m}"));
        let surface = |x: Ix| target.faces()[x as usize].surface;
        if !matches!(surface(f), Surface::Plane(_)) {
            return Ok(None);
        }
        let nodes = crate::unfold::tree(target, crate::unfold::strip(target, f))?;
        if nodes.len() == 1 {
            return Self::plate(target, &[crate::unfold::strip(target, f)], frame);
        }
        let chain = chain_through(target, f, &crate::unfold::branches(target)).ok();
        let (a, _) = crate::unfold::axes(target, f, chain)?;
        let [o, _, _, z] = frame;
        let e = unit(a.cross(z));
        let root = crate::unfold::Lay::Plane { o: Vec2::new(0.0, 0.0), start: o, ex: e, ey: a };
        let lays = crate::unfold::lay_tree(target, &nodes, root, k.unwrap_or(0.5))?;

        // The other side, strip by strip: a bend across each bend, and across each wall the wall that
        // goes on from the bend beside it there.
        let mut back: Vec<Vec<Ix>> = Vec::with_capacity(nodes.len());
        let across = |x: Ix| {
            opposite_cylinder(target, x as usize)
                .map(|(g, _)| crate::unfold::strip(target, g))
                .ok_or_else(|| unsupported("a bend without the face across the sheet"))
        };
        let off = |bend: &[Ix], left: &[Ix]| {
            bend.iter().flat_map(|&x| crate::unfold::smooth(target, x)).map(|(y, _)| y).find(|y| !bend.contains(y) && !left.contains(y))
        };
        for node in &nodes {
            let x = node.faces[0];
            let other = match (surface(x), node.parent) {
                (Surface::Cylinder(_), _) => across(x)?,
                (_, Some((p, _))) => {
                    let grand = nodes[p].parent.map_or(&[][..], |g| &back[g.0][..]);
                    let wall = off(&back[p], grand).ok_or_else(|| unsupported("a wall without its other side"))?;
                    crate::unfold::strip(target, wall)
                }
                (Surface::Plane(pl), None) => {
                    // The sketch's face: of what goes on from a bend across, the wall facing away from it.
                    let bend = nodes
                        .iter()
                        .find(|n| n.parent.is_some_and(|p| p.0 == 0))
                        .ok_or_else(|| BrepError::Invalid("cut across a bend: a tree of one".into()))?;
                    let bend = across(bend.faces[0])?;
                    let wall = bend.iter().flat_map(|&b| crate::unfold::smooth(target, b)).map(|(y, _)| y).find(|&y| match surface(y) {
                        Surface::Plane(q) => crate::sketch::parallel(q.normal, pl.normal) && q.normal.dot(pl.normal) < 0.0,
                        _ => false,
                    });
                    crate::unfold::strip(target, wall.ok_or_else(|| unsupported("the sheet's two sides do not pair face by face"))?)
                }
                _ => return Err(unsupported("a face of the sheet neither a wall nor a bend")),
            };
            back.push(other);
        }

        // The chain: on from the sketch's face along X, each way, as far as strips go. Each has its
        // span in X; every other strip lies where its faces are.
        let way = |i: usize| lays[i].1.x;
        let mut spans: Vec<Option<(f64, f64)>> = vec![None; nodes.len()];
        let at_x = |i: usize| match lays[i].0 {
            crate::unfold::Lay::Plane { o, .. } | crate::unfold::Lay::Bend { o, .. } => o.x,
        };
        let child = |p: usize, sign: f64| (0..nodes.len()).find(|&j| nodes[j].parent.is_some_and(|q| q.0 == p) && way(j) * sign > 0.0);
        let (right, left) = (child(0, 1.0), child(0, -1.0));
        spans[0] = Some((left.map_or(f64::NEG_INFINITY, at_x), right.map_or(f64::INFINITY, at_x)));
        for (first, sign) in [(right, 1.0), (left, -1.0)] {
            let mut cur = first;
            while let Some(i) = cur {
                let next = child(i, sign);
                let far = next.map_or(sign * f64::INFINITY, at_x);
                spans[i] = Some(if sign > 0.0 { (at_x(i), far) } else { (far, at_x(i)) });
                cur = next;
            }
        }
        // In order along the chain, the others after.
        let mut order: Vec<usize> = (0..nodes.len()).collect();
        order.sort_by(|&p, &q| match (spans[p], spans[q]) {
            (Some(a), Some(b)) => a.0.total_cmp(&b.0),
            (Some(_), None) => std::cmp::Ordering::Less,
            (None, Some(_)) => std::cmp::Ordering::Greater,
            (None, None) => p.cmp(&q),
        });
        let index = |i: usize| order.iter().position(|&j| j == i).unwrap_or(0);
        let mut strips = Vec::with_capacity(nodes.len());
        for &i in &order {
            let lay = match lays[i].0 {
                crate::unfold::Lay::Plane { o: at, start, ex, ey } => {
                    let n = if i == 0 {
                        z
                    } else {
                        match surface(nodes[i].faces[0]) {
                            Surface::Plane(p) => unit(p.normal),
                            _ => return Err(BrepError::Invalid("cut across a bend: a wall off a plane".into())),
                        }
                    };
                    Lay::Plane { n, ex, ey, base: start, at }
                }
                crate::unfold::Lay::Bend { o: at, line, d, u, v, origin, axis, start, span, neutral: n } => {
                    let Surface::Cylinder(c) = surface(nodes[i].faces[0]) else {
                        return Err(BrepError::Invalid("cut across a bend: a bend off a cylinder".into()));
                    };
                    let (_, other) = opposite_cylinder(target, nodes[i].faces[0] as usize)
                        .ok_or_else(|| unsupported("a bend without the face across the sheet"))?;
                    // Only bend lines along the blank's axes: `split` and `UvMap` go by them.
                    if (u.x != 0.0 && u.y != 0.0) || (v.x != 0.0 && v.y != 0.0) {
                        return Err(unsupported("a bend line oblique to the blank's axes comes later"));
                    }
                    let swap = u.x == 0.0;
                    let s_axis = if axis.dot(d) > 0.0 { 1.0 } else { -1.0 };
                    let at_v = at.x * v.x + at.y * v.y;
                    // `v` of the development is 0 where the blank's coordinate along the line is 0.
                    let foot = origin - d * ((origin - line).dot(d) + at_v);
                    let map = if swap {
                        UvMap { swap, o: Vec2::new(0.0, at.y), s: [u.y, s_axis * v.x] }
                    } else {
                        UvMap { swap, o: Vec2::new(at.x, 0.0), s: [u.x, s_axis * v.y] }
                    };
                    let dev = Development { origin: foot, axis, start, neutral: n };
                    Lay::Bend { dev, map, near: 0.5 * n * span, radii: [c.radius, other.radius], neutral: k.map(|_| n) }
                }
            };
            strips.push(Strip { faces: [nodes[i].faces.clone(), back[i].clone()], span: spans[i], region: Vec::new(), holes: Vec::new(), lay });
        }
        let he = |h: Ix| target.half_edges()[h as usize];
        let face_of = |h: Ix| target.loops()[he(h).loop_ix as usize].face;
        let thickness = strips
            .iter()
            .find_map(|s| match s.lay {
                Lay::Bend { radii, .. } => Some((radii[0] - radii[1]).abs()),
                Lay::Plane { .. } => None,
            })
            .unwrap_or(0.0);
        let mut blank = Self { a, e, thickness, strips, seams: Vec::new(), home: index(0), layered: false };
        // The bend lines: along an axis, as far along it as the faces meet.
        for (i, node) in nodes.iter().enumerate() {
            let Some((p, _)) = node.parent else { continue };
            let u = lays[i].1;
            let at = match lays[i].0 {
                crate::unfold::Lay::Plane { o, .. } | crate::unfold::Lay::Bend { o, .. } => o,
            };
            let points: Vec<Vec2> = node
                .faces
                .iter()
                .flat_map(|&x| target.face_loops(x))
                .flat_map(|l| target.loop_half_edges(l))
                .filter(|&h| nodes[p].faces.contains(&face_of(he(h).twin)))
                .flat_map(|h| [he(h).origin, he(target.next(h)).origin])
                .map(|v| blank.flat(index(i), target.point(v)))
                .collect();
            let across_x = u.x != 0.0;
            let along = |q: &Vec2| if across_x { q.y } else { q.x };
            let (lo, hi) = (points.iter().map(along).fold(f64::INFINITY, f64::min), points.iter().map(along).fold(f64::NEG_INFINITY, f64::max));
            let (a, b) = if across_x { (Vec2::new(at.x, lo), Vec2::new(at.x, hi)) } else { (Vec2::new(lo, at.y), Vec2::new(hi, at.y)) };
            blank.seams.push(Seam { a, b, strips: [index(p), index(i)] });
        }
        // Where each strip's faces lie on the blank. Those of the chain go by their span, unless
        // strips not joined may lie on one another: their boxes overlap.
        let mut outlines = Vec::with_capacity(blank.strips.len());
        for i in 0..blank.strips.len() {
            let faces = blank.strips[i].faces[0].clone();
            let region = faces.iter().map(|&x| blank.region(target, i, target.faces()[x as usize].first_loop)).collect::<Result<Vec<_>, _>>()?;
            outlines.push(region);
        }
        let boxes: Vec<(Vec2, Vec2)> = outlines
            .iter()
            .map(|r| {
                let points = || r.iter().flat_map(|l| l.points.iter());
                let lo = points().fold(Vec2::new(f64::INFINITY, f64::INFINITY), |m, p| Vec2::new(m.x.min(p.x), m.y.min(p.y)));
                let hi = points().fold(Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY), |m, p| Vec2::new(m.x.max(p.x), m.y.max(p.y)));
                (lo, hi)
            })
            .collect();
        let joined = |i: usize, j: usize| blank.seams.iter().any(|sm| sm.strips.contains(&i) && sm.strips.contains(&j));
        let size = boxes.iter().flat_map(|b| [b.0.x, b.0.y, b.1.x, b.1.y]).fold(1.0, |m: f64, x| m.max(x.abs()));
        let apart = |a: f64, b: f64| a - b <= 64.0 * f64::EPSILON * size;
        blank.layered = (0..boxes.len()).any(|i| {
            (i + 1..boxes.len()).any(|j| {
                let (p, q) = (boxes[i], boxes[j]);
                !joined(i, j) && !apart(p.1.x, q.0.x) && !apart(q.1.x, p.0.x) && !apart(p.1.y, q.0.y) && !apart(q.1.y, p.0.y)
            })
        });
        for (i, region) in outlines.into_iter().enumerate() {
            if blank.layered {
                let faces = blank.strips[i].faces[0].clone();
                let inner = faces.iter().flat_map(|&x| target.face_loops(x).skip(1)).map(|l| blank.region(target, i, l));
                let holes = inner.collect::<Result<Vec<_>, _>>()?;
                blank.strips[i].holes = holes;
                blank.strips[i].span = None;
            }
            if blank.strips[i].span.is_none() {
                blank.strips[i].region = region;
            }
        }
        Ok(Some(blank))
    }

    /// The blank of a sheet without bends (a face of sheet metal, D195; D202): one strip, laid in
    /// the sketch's own axes; the other side the sheet's other side. None for anything else.
    fn plate(target: &Body, front: &[Vec<Ix>], frame: [Vec3; 4]) -> Result<Option<Self>, BrepError> {
        let [home] = front else { return Ok(None) };
        let Surface::Plane(p) = target.faces()[home[0] as usize].surface else { return Ok(None) };
        let Ok(sides) = crate::unfold::sheet_sides(target) else { return Ok(None) };
        let Some(&other) = sides.iter().find(|x| !home.contains(x)) else { return Ok(None) };
        let Surface::Plane(q) = target.faces()[other as usize].surface else { return Ok(None) };
        let [o, x, y, z] = frame;
        let n = unit(p.normal);
        let back = chain_through(target, other, &crate::unfold::branches(target))?.concat();
        let strip = Strip {
            faces: [home.clone(), back],
            span: Some((f64::NEG_INFINITY, f64::INFINITY)),
            region: Vec::new(),
            holes: Vec::new(),
            lay: Lay::Plane { n: z, ex: x, ey: y, base: o, at: Vec2::new(0.0, 0.0) },
        };
        Ok(Some(Self { a: y, e: x, thickness: (p.origin - q.origin).dot(n), strips: vec![strip], seams: Vec::new(), home: 0, layered: false }))
    }

    /// The profiles of `sketch` on the blank, counterclockwise.
    fn laid(&self, feature: TopoId, sketch: &Placed) -> Result<Vec<ProfileLoop>, BrepError> {
        let [_, x, y, _] = sketch.frame;
        let turn = |p: Vec2| Vec2::new(p.x * x.dot(self.e) + p.y * y.dot(self.e), p.x * x.dot(self.a) + p.y * y.dot(self.a));
        Ok(profile_loops(feature, sketch)?
            .into_iter()
            .map(|l| {
                let mut ring = if l.ring.counterclockwise() { l.ring } else { l.ring.reversed() };
                ring.points.iter_mut().for_each(|p| *p = turn(*p));
                ring.arcs.iter_mut().flatten().for_each(|a| a.center = turn(a.center));
                ring
            })
            .collect())
    }

    /// The point at `p` of the blank on strip `i`, on the sketch's side (`layer` 0) or the other.
    fn point(&self, i: usize, p: Vec2, layer: usize) -> Vec3 {
        let st = &self.strips[i];
        match &st.lay {
            Lay::Plane { n, ex, ey, base, at } => {
                let q = *base + *ex * (p.x - at.x) + *ey * (p.y - at.y);
                if layer == 1 { q - *n * self.thickness } else { q }
            }
            Lay::Bend { dev, map, radii, .. } => dev.point(map.uv(p), radii[layer]),
        }
    }

    /// Where `q`, a point of strip `i`, lies on the blank.
    fn flat(&self, i: usize, q: Vec3) -> Vec2 {
        let st = &self.strips[i];
        match &st.lay {
            Lay::Plane { ex, ey, base, at, .. } => Vec2::new(at.x + (q - *base).dot(*ex), at.y + (q - *base).dot(*ey)),
            Lay::Bend { dev, map, near, .. } => map.blank(dev.unroll(q, *near).0),
        }
    }

    /// The arc half-edge `h` of a face of strip `i` runs on, as the blank has it; none for a line.
    fn arc_of(&self, target: &Body, i: usize, h: Ix) -> Result<Option<ProfileArc>, BrepError> {
        let flat = |p: Vec3| self.flat(i, p);
        let edge = &target.edges()[target.half_edges()[h as usize].edge as usize];
        let curve = target.edge_curve(edge).map(|c| if edge.half == h { c } else { c.reversed() });
        Ok(match (curve, &self.strips[i].lay) {
            (None, _) | (Some(Curve::Arc(_)), Lay::Bend { .. }) => None,
            (Some(Curve::Arc(arc)), Lay::Plane { n, .. }) => Some(ProfileArc { center: flat(arc.center), ccw: arc.axis.dot(*n) > 0.0 }),
            (Some(Curve::Rolled { piece, .. }), Lay::Bend { dev, map, .. }) => piece.arc.map(|pa| {
                // The piece's development and the strip's turn about one axis, maybe the other
                // way: both coordinates then run the other way.
                let s = if piece.dev.axis.dot(dev.axis) > 0.0 { 1.0 } else { -1.0 };
                let origin = map.uv(flat(piece.dev.point(Vec2::new(0.0, 0.0), piece.dev.neutral)));
                ProfileArc { center: map.blank(Vec2::new(origin.x + s * pa.center.x, origin.y + s * pa.center.y)), ccw: pa.ccw != map.mirrored() }
            }),
            (Some(Curve::Rolled { .. }), Lay::Plane { .. }) => return Err(BrepError::Invalid("a rolled edge on a wall".into())),
        })
    }

    /// The strip point `p` lies in, off every bend line: a flange's on the sheet's side where its
    /// faces are, else the chain's at its `s`.
    fn strip(&self, p: Vec2) -> usize {
        let across = self.strips.iter().position(|st| st.region.iter().any(|r| r.winding(p) != 0));
        across.unwrap_or_else(|| self.strips.iter().take_while(|st| st.span.is_some()).filter(|st| st.span.is_some_and(|s| s.1 < p.x)).count())
    }

    /// Every strip point `p` lies on, of a blank that lies on itself.
    fn strips_at(&self, p: Vec2) -> Vec<usize> {
        let on = |st: &Strip| st.region.iter().any(|r| r.winding(p) != 0) && st.holes.iter().all(|h| h.winding(p) == 0);
        (0..self.strips.len()).filter(|&i| on(&self.strips[i])).collect()
    }

    /// The edges of the blank on the sketch's side as closed loops: its outline and every cut in
    /// it, the bend lines left out.
    fn outline(&self, target: &Body) -> Result<Vec<Outline>, BrepError> {
        let he = |h: Ix| target.half_edges()[h as usize];
        let face_of = |h: Ix| target.loops()[he(h).loop_ix as usize].face;
        let strip_of: BTreeMap<Ix, usize> = self.strips.iter().enumerate().flat_map(|(i, s)| s.faces[0].iter().map(move |&x| (x, i))).collect();
        let mut next: BTreeMap<Ix, (Ix, Vec2, Option<ProfileArc>, Ix, usize)> = BTreeMap::new();
        for (&x, &i) in &strip_of {
            for l in target.face_loops(x) {
                for h in target.loop_half_edges(l) {
                    if strip_of.contains_key(&face_of(he(h).twin)) {
                        continue;
                    }
                    let (v, w) = (he(h).origin, he(target.next(h)).origin);
                    next.insert(v, (w, self.flat(i, target.point(v)), self.arc_of(target, i, h)?, h, i));
                }
            }
        }
        let mut loops = Vec::new();
        while let Some((&v0, _)) = next.iter().next() {
            let mut ring = ProfileLoop { points: Vec::new(), vertices: Vec::new(), sides: Vec::new(), arcs: Vec::new() };
            let mut edges = Vec::new();
            let mut v = v0;
            while let Some((w, p, arc, h, i)) = next.remove(&v) {
                ring.points.push(p);
                ring.arcs.push(arc);
                edges.push((h, i));
                v = w;
            }
            if v != v0 {
                return Err(BrepError::Invalid("cut across a bend: the blank's outline does not close".into()));
            }
            loops.push(Outline { ring, edges });
        }
        Ok(loops)
    }

    /// Profile `l` cut into pieces at the bend lines and at the edges of the blank's outline
    /// `outer` (D202), each crossing kept with its line or edge; only the pieces `keep` takes by
    /// their middle (a cut's on the metal, an addition's off it).
    fn split(&self, feature: TopoId, l: &ProfileLoop, loops: &[Outline], keep: &dyn Fn(Vec2) -> bool, out: &mut Parts) -> Result<(), BrepError> {
        let Parts { pieces, seams, rims } = out;
        let n = l.points.len();
        let mut last: Option<TopoId> = None;
        // Corners of the profile on an edge of the sheet, and whether each segment's first and
        // last pieces are on the sheet.
        let corners: Vec<Option<(usize, usize)>> = l.points.iter().map(|&a| on_edge(a, loops)).collect::<Result<_, _>>()?;
        // The strips a point's own IDs are for: on a blank lying on itself another strip there has
        // IDs of its own, from these.
        let owners: Vec<Vec<usize>> = (0..n)
            .map(|k| match corners[k] {
                Some((o, e)) => vec![loops[o].edges[e].1],
                None if self.layered => self.strips_at(l.points[k]).into_iter().take(1).collect(),
                None => Vec::new(),
            })
            .collect();
        let mut ends: Vec<[bool; 2]> = Vec::with_capacity(n);
        for k in 0..n {
            let mut on_sheet = [false; 2];
            let (a, b, arc) = (l.points[k], l.points[(k + 1) % n], l.arc(k));
            let radius = arc.map_or(0.0, |c| libm::hypot(a.x - c.center.x, a.y - c.center.y));
            let mut cuts: Vec<(f64, Vec2, Line)> = Vec::new();
            for (j, seam) in self.seams.iter().enumerate() {
                let (p, q) = (seam.a, seam.b);
                if orient2d(p, q, a) == Sign::Zero
                    && datum_brep::overlap((a, a), (Vec2::new(p.x.min(q.x), p.y.min(q.y)), Vec2::new(p.x.max(q.x), p.y.max(q.y))))
                {
                    return Err(BrepError::Degenerate("a corner of a profile lies on a bend line".into()));
                }
                cuts.extend(across_seam(a, b, arc, radius, p, q)?.into_iter().map(|(t, x)| (t, x, Line::Bend(j))));
            }
            // The edges of the blank it crosses: the sheet's sides and ends, earlier cuts. A line
            // along edges of the blank (a tab drawn on the sheet's edge) crosses none of them, and
            // is no edge of the sheet after: only the corners at its ends are.
            let mut along = false;
            for (o, outline) in loops.iter().enumerate() {
                let m = outline.ring.points.len();
                for e in 0..m {
                    let (p, q) = (outline.ring.points[e], outline.ring.points[(e + 1) % m]);
                    if arc.is_none() && outline.ring.arc(e).is_none() && runs_along(a, b, p, q) {
                        along = true;
                        continue;
                    }
                    let found = match (arc, outline.ring.arc(e)) {
                        (None, None) => segment_crossing(a, b, p, q)?.into_iter().collect(),
                        (Some(c), None) => arc_segment_crossings(a, b, c, radius, p, q)?,
                        (None, Some(d)) => segment_arc_crossings(a, b, p, q, d)?,
                        (Some(c), Some(d)) => arc_arc_crossings(a, b, c, radius, p, q, d)?,
                    };
                    cuts.extend(found.into_iter().map(|(t, x)| (t, x, Line::Rim(o, e))));
                }
            }
            cuts.sort_by(|x, y| x.0.total_cmp(&y.0));
            let cross =
                |m: usize| [0, 1].map(|layer| TopoId::derive("cut.cross").id(feature).id(l.sides[k]).index(m as u64).index(layer as u64).finish());
            // Each point: where it is, and whose vertices are which. Where two lines lie at one point
            // of the blank (a flange's end on the bend line of the one beside it) they are one
            // point there, but different metal: each keeps vertices of its own.
            let corner = |k: usize| (l.points[k], vec![(owners[k].clone(), l.vertices[k])]);
            let mut points = vec![corner(k)];
            for (m, &(_, p, line)) in cuts.iter().enumerate() {
                let on_line = match line {
                    Line::Bend(j) => {
                        seams[j].push((p, cross(m)));
                        self.seams[j].strips.to_vec()
                    }
                    Line::Rim(o, e) => {
                        rims[o][e].push((p, cross(m)));
                        vec![loops[o].edges[e].1]
                    }
                };
                let at = points[points.len() - 1].0;
                match points.last_mut() {
                    Some(last) if self.layered && within_rounding(dist(at, p), at.x.abs().max(at.y.abs()).max(1.0)) => {
                        last.1.push((on_line, cross(m)))
                    }
                    _ => points.push((p, vec![(on_line, cross(m))])),
                }
            }
            points.push(corner((k + 1) % n));
            // A circle's next arc goes on as its first did.
            let goes_on = k > 0 && matches!((l.arc(k - 1), arc), (Some(p), Some(q)) if p == q);
            let count = points.len() - 1;
            for (i, w) in points.windows(2).enumerate() {
                let (from, to) = (w[0].0, w[1].0);
                let middle = halfway(from, to, arc);
                let wall = match last {
                    Some(prev) if i > 0 || goes_on => next_piece(prev),
                    _ => l.sides[k],
                };
                last = Some(wall);
                // Not kept: no wall there.
                let on = !along && keep(middle);
                if i == 0 {
                    on_sheet[0] = on;
                }
                if i + 1 == count {
                    on_sheet[1] = on;
                }
                if !on {
                    continue;
                }
                let layers = if self.layered { self.strips_at(middle) } else { Vec::new() };
                if layers.is_empty() {
                    pieces.push(Piece { from, to, middle, arc, radius, strip: self.strip(middle), ends: [w[0].1[0].1, w[1].1[0].1], wall });
                    continue;
                }
                let layer_id = |id: TopoId, s: usize| TopoId::derive("cut.layer").id(id).index(s as u64).finish();
                for (j, &s) in layers.iter().enumerate() {
                    let mine = |p: &StopOn| p.1.iter().find(|g| g.0.contains(&s)).map_or_else(|| p.1[0].1.map(|id| layer_id(id, s)), |g| g.1);
                    let wall = if j == 0 { wall } else { layer_id(wall, s) };
                    pieces.push(Piece { from, to, middle, arc, radius, strip: s, ends: [mine(&w[0]), mine(&w[1])], wall });
                }
            }
            ends.push(on_sheet);
        }
        // A corner on an edge where the profile's kept pieces end crosses the edge there; one
        // with kept pieces on both sides only touches it.
        for (k, e) in corners.into_iter().enumerate() {
            let Some((o, e)) = e else { continue };
            match (ends[(k + n - 1) % n][1], ends[k][0]) {
                (true, true) => return Err(BrepError::Degenerate("a profile only touches an edge of the sheet at a corner".into())),
                (false, false) => {}
                _ => rims[o][e].push((l.points[k], l.vertices[k])),
            }
        }
        Ok(())
    }

    /// The faces of side `side` (0: the sketch's side) of strip `i` with the pieces in them: each
    /// face's outline broken where profiles cross its bend lines and run along them instead, a
    /// profile within it a hole. A face the profiles cross from one bend line to the other comes
    /// out in pieces (D201): the piece with the face's first vertex keeps its ID, the others take
    /// one from it and their lowest vertex; every hole goes to the piece around it.
    fn rebuilt(&self, target: &Body, i: usize, side: usize, cuts: &Cuts, curves: &mut Vec<CurveSpec>) -> Result<Vec<FaceSpec>, BrepError> {
        let Cuts { open, grow, pieces, seams, rims, profiles } = cuts;
        let members = &self.strips[i].faces[side];
        let he = |h: Ix| target.half_edges()[h as usize];
        let id = |v: Ix| target.vertices()[v as usize].id;
        let at = |v: Ix| self.flat(i, target.point(v));
        // The face of the strip each piece lies in: the one whose outline holds its middle.
        let regions: Vec<ProfileLoop> = if members.len() == 1 {
            Vec::new()
        } else {
            members.iter().map(|&x| self.region(target, i, target.faces()[x as usize].first_loop)).collect::<Result<_, _>>()?
        };
        let mut out = Vec::with_capacity(members.len());
        for (m, &x) in members.iter().enumerate() {
            let spec = target.face_spec(x);
            // From each vertex: the next, where it is on the blank, and the arc between.
            let mut next: BTreeMap<TopoId, (TopoId, Vec2, Option<ProfileArc>)> = BTreeMap::new();
            let mut link = |from: TopoId, to: TopoId, p: Vec2, arc: Option<ProfileArc>| match next.insert(from, (to, p, arc)) {
                Some(_) => Err(BrepError::Invalid("cut across a bend: two edges leave one vertex of a face".into())),
                None => Ok(()),
            };
            // The outline and every earlier cut a profile crosses go edge by edge; a cut a profile
            // holds goes whole; the others stay as they are (D203).
            let taken = |p: Vec2| profiles.iter().any(|l| covers(l, p));
            // The bend line half-edge `h` lies on, if any.
            let line_of = |h: Ix| {
                let beside = target.loops()[he(he(h).twin).loop_ix as usize].face;
                self.seams
                    .iter()
                    .position(|sm| sm.strips.contains(&i) && sm.strips.iter().any(|&k| k != i && self.strips[k].faces[side].contains(&beside)))
            };
            // Whether a profile crosses the bend line of `h` on it (a flange on a window's edge).
            let seam_crossed = |h: Ix| {
                line_of(h).is_some_and(|j| {
                    let (pu, pv) = (at(he(h).origin), at(he(target.next(h)).origin));
                    let d = Vec2::new(pv.x - pu.x, pv.y - pu.y);
                    seams[j].iter().any(|c| {
                        let t = ((c.0.x - pu.x) * d.x + (c.0.y - pu.y) * d.y) / (d.x * d.x + d.y * d.y);
                        0.0 < t && t < 1.0
                    })
                })
            };
            let mut edges: Vec<Ix> = Vec::new();
            let mut kept: Vec<Vec<TopoId>> = Vec::new();
            for (k, l) in target.face_loops(x).enumerate() {
                let crossed =
                    k > 0 && target.loop_half_edges(l).any(|h| seam_crossed(h) || (*open && !self.on_rim(target, rims, i, side, h).is_empty()));
                if k == 0 || crossed {
                    edges.extend(target.loop_half_edges(l));
                } else if !(*open && taken(at(he(target.loop_half_edges(l).start).origin))) {
                    kept.push(spec.loops[k].clone());
                }
            }
            for h in edges {
                let (u, v) = (he(h).origin, he(target.next(h)).origin);
                let line = line_of(h);
                let Some(j) = line else {
                    // An edge of the sheet, broken where profiles cross it (D202): the stretches
                    // outside them.
                    let arc = self.arc_of(target, i, h)?;
                    let cuts = if *open { self.on_rim(target, rims, i, side, h) } else { Vec::new() };
                    if cuts.is_empty() {
                        // Kept whole, unless a profile takes it whole.
                        if !(*open && taken(halfway(at(u), at(v), arc))) {
                            link(id(u), id(v), at(u), arc)?;
                        }
                        continue;
                    }
                    let stops: Vec<(Vec2, TopoId)> =
                        std::iter::once((at(u), id(u))).chain(cuts.iter().map(|c| (c.0, c.1[side]))).chain([(at(v), id(v))]).collect();
                    let edge = &target.edges()[he(h).edge as usize];
                    let curve = target.edge_curve(edge).map(|c| if edge.half == h { c } else { c.reversed() });
                    for w in stops.windows(2) {
                        if taken(halfway(w[0].0, w[1].0, arc)) {
                            continue;
                        }
                        link(w[0].1, w[1].1, w[0].0, arc)?;
                        // A piece of a curved edge: an arc keeps its circle; a curve on a bend
                        // its development, between its new ends there.
                        let piece = match curve {
                            None => None,
                            Some(Curve::Arc(a)) => Some(Curve::Arc(a)),
                            Some(Curve::Rolled { piece, radius }) => {
                                // The same on both sides of the sheet: the curve's own ends where
                                // they stay, a new end from where it is on the blank.
                                let uv = |k: usize| match w[k].1 {
                                    x if x == id(u) => piece.from,
                                    x if x == id(v) => piece.to,
                                    _ => piece.dev.unroll(self.point(i, w[k].0, 0), piece.from.x).0,
                                };
                                Some(Curve::Rolled { piece: Rolled { from: uv(0), to: uv(1), ..piece }, radius })
                            }
                        };
                        if let Some(curve) = piece {
                            curves.push(CurveSpec { from: w[0].1, to: w[1].1, curve });
                        }
                    }
                    continue;
                };
                // Along the bend line, from crossing to crossing: the stretches outside the profiles.
                let (pu, pv) = (at(u), at(v));
                let d = Vec2::new(pv.x - pu.x, pv.y - pu.y);
                let along = |p: Vec2| ((p.x - pu.x) * d.x + (p.y - pu.y) * d.y) / (d.x * d.x + d.y * d.y);
                if seams[j].iter().any(|c| c.0 == pu || c.0 == pv) {
                    return Err(BrepError::Degenerate("a profile crosses a bend line at a corner of the sheet".into()));
                }
                let mut on: Vec<(f64, &(Vec2, [TopoId; 2]))> = seams[j].iter().map(|c| (along(c.0), c)).filter(|c| 0.0 < c.0 && c.0 < 1.0).collect();
                on.sort_by(|p, q| p.0.total_cmp(&q.0));
                let stops: Vec<(Vec2, TopoId)> =
                    std::iter::once((pu, id(u))).chain(on.iter().map(|c| (c.1.0, c.1.1[side]))).chain([(pv, id(v))]).collect();
                for w in stops.windows(2) {
                    let middle = Vec2::new(0.5 * (w[0].0.x + w[1].0.x), 0.5 * (w[0].0.y + w[1].0.y));
                    if profiles.iter().all(|p| p.winding(middle) == 0) {
                        link(w[0].1, w[1].1, w[0].0, None)?;
                    }
                }
            }
            // A cut's profiles run against the outline on the sketch's side, along it on the
            // other; an addition's on as the outline does.
            for p in pieces.iter().filter(|p| p.strip == i && (regions.is_empty() || regions[m].winding(p.middle) != 0)) {
                if (side == 0) != *grow {
                    link(p.ends[1][side], p.ends[0][side], p.to, p.arc.map(|a| ProfileArc { ccw: !a.ccw, ..a }))?
                } else {
                    link(p.ends[0][side], p.ends[1][side], p.from, p.arc)?
                }
            }
            let first: Vec<TopoId> = spec.loops[0].clone();
            let round = self.region(target, i, target.faces()[x as usize].first_loop)?.area();
            let starts: Vec<TopoId> = first.iter().copied().chain(next.keys().copied()).collect();
            let (mut outers, mut holes) = (Vec::new(), Vec::new());
            for v0 in starts {
                if !next.contains_key(&v0) {
                    continue;
                }
                let (mut cycle, mut ring) =
                    (Vec::new(), ProfileLoop { points: Vec::new(), vertices: Vec::new(), sides: Vec::new(), arcs: Vec::new() });
                let mut v = v0;
                while let Some((w, p, arc)) = next.remove(&v) {
                    cycle.push(v);
                    ring.points.push(p);
                    ring.arcs.push(arc);
                    v = w;
                }
                if v != v0 {
                    return Err(BrepError::Invalid("cut across a bend: a face's outline does not close".into()));
                }
                // A piece runs round as the face's outline does, with or without a corner of it (a
                // wall whose corners a cut parted off); a hole the other way.
                let as_outline = ring.area() * round > 0.0;
                if cycle.iter().any(|v| first.contains(v)) || as_outline { outers.push((cycle, ring)) } else { holes.push((cycle, ring)) }
            }
            // Every corner of the outline cut away (reliefs at all its corners): the largest is it.
            if outers.is_empty()
                && let Some(k) = (0..holes.len()).max_by(|&a, &b| holes[a].1.area().abs().total_cmp(&holes[b].1.area().abs()))
            {
                outers.push(holes.remove(k));
            }
            if let [(outer, _)] = &outers[..] {
                let loops = std::iter::once(outer.clone()).chain(kept).chain(holes.into_iter().map(|h| h.0)).collect();
                out.push(FaceSpec { id: spec.id, surface: spec.surface, loops });
                continue;
            }
            // Divided: each hole, old or new, to the piece around a point of it.
            let point_of = |v: TopoId| -> Option<Vec2> {
                let ix = target.vertices().binary_search_by_key(&v, |w| w.id).ok()?;
                Some(at(ix as Ix))
            };
            let mut loops: Vec<Vec<Vec<TopoId>>> = outers.iter().map(|(c, _)| vec![c.clone()]).collect();
            let old = kept.into_iter().map(|l| {
                let p = l.first().and_then(|&v| point_of(v));
                (l, p)
            });
            let new = holes.into_iter().map(|(c, r)| (c, r.points.first().copied()));
            for (hole, p) in old.chain(new) {
                let k = p.and_then(|p| outers.iter().position(|(_, r)| r.winding(p) != 0));
                let k = k.ok_or_else(|| BrepError::Invalid("cut across a bend: a hole in no piece of a divided face".into()))?;
                loops[k].push(hole);
            }
            for ((cycle, _), loops) in outers.iter().zip(loops) {
                let face = if cycle.contains(&first[0]) {
                    spec.id
                } else {
                    let lowest = cycle.iter().min().copied().unwrap_or(spec.id);
                    TopoId::derive("sheet.divided").id(spec.id).id(lowest).finish()
                };
                out.push(FaceSpec { id: face, surface: spec.surface, loops });
            }
        }
        Ok(out)
    }

    /// Where profiles cross half-edge `h` of a face of side `side` of strip `i`, an edge of the
    /// sheet: in order from its start, each with its vertices on both sides. On the sketch's side
    /// the crossings found on `h` itself; across the sheet those of the same edge of the sheet
    /// (the face beyond, the strip) that lie on `h` as the blank has it.
    fn on_rim(&self, target: &Body, rims: &[Rim], i: usize, side: usize, h: Ix) -> Vec<(Vec2, [TopoId; 2])> {
        let he = |h: Ix| target.half_edges()[h as usize];
        let beyond = target.loops()[he(he(h).twin).loop_ix as usize].face;
        let (u, v) = (self.flat(i, target.point(he(h).origin)), self.flat(i, target.point(he(target.next(h)).origin)));
        let d = Vec2::new(v.x - u.x, v.y - u.y);
        let along = |p: Vec2| ((p.x - u.x) * d.x + (p.y - u.y) * d.y) / (d.x * d.x + d.y * d.y);
        let mut out: Vec<(f64, Vec2, [TopoId; 2])> = rims
            .iter()
            .filter(|r| if side == 0 { r.h == h } else { r.beyond == beyond && r.strip == i })
            .flat_map(|r| r.crossings.iter())
            .map(|&(p, ids)| (along(p), p, ids))
            .filter(|c| side == 0 || (0.0 < c.0 && c.0 < 1.0))
            .collect();
        out.sort_by(|a, b| a.0.total_cmp(&b.0));
        out.into_iter().map(|(_, p, ids)| (p, ids)).collect()
    }

    /// Face `t` across the sheet's thickness (an edge or an end of the sheet) with the profiles
    /// crossing it (D202): its edges along the sheet's sides broken as theirs are, and across the
    /// thickness at each crossing an edge the cut's wall shares. Cut in pieces, the one with its
    /// first vertex keeps its ID, the others take one from it and their lowest vertex.
    fn rim_face(&self, target: &Body, t: Ix, cuts: &Cuts, split: &[CurveSpec]) -> Result<Vec<FaceSpec>, BrepError> {
        let Cuts { rims, profiles, .. } = cuts;
        let spec = target.face_spec(t);
        let he = |h: Ix| target.half_edges()[h as usize];
        let id = |v: Ix| target.vertices()[v as usize].id;
        let mut next: BTreeMap<TopoId, TopoId> = BTreeMap::new();
        let link = |next: &mut BTreeMap<TopoId, TopoId>, from: TopoId, to: TopoId| match next.insert(from, to) {
            Some(_) => Err(BrepError::Invalid("cut to the edge: two edges leave one vertex of a face".into())),
            None => Ok(()),
        };
        let mut pairs: Vec<[TopoId; 2]> = Vec::new();
        // Where each vertex of the sheet's faces is on the blank.
        let mut on_blank: BTreeMap<Ix, Vec2> = BTreeMap::new();
        for (i, st) in self.strips.iter().enumerate() {
            for &x in st.faces.iter().flatten() {
                for v in target.face_loops(x).flat_map(|l| target.loop_half_edges(l)).map(|h| he(h).origin) {
                    on_blank.entry(v).or_insert_with(|| self.flat(i, target.point(v)));
                }
            }
        }
        let taken = |p: Vec2| profiles.iter().any(|l| covers(l, p));
        for l in target.face_loops(t) {
            for g in target.loop_half_edges(l) {
                let (a, b) = (he(g).origin, he(target.next(g)).origin);
                let twin = he(g).twin;
                let face = target.loops()[he(twin).loop_ix as usize].face;
                let found = self.strips.iter().enumerate().find_map(|(i, st)| (0..2).find(|&s| st.faces[s].contains(&face)).map(|s| (i, s)));
                let cuts = found.map(|(i, s)| (i, s, self.on_rim(target, rims, i, s, twin))).filter(|c| !c.2.is_empty());
                let Some((i, side, mut cuts)) = cuts else {
                    // Kept whole, unless a profile takes it: along the sheet its middle, across the
                    // thickness the point of the blank it stands at.
                    let gone = match (found, on_blank.get(&a), on_blank.get(&b)) {
                        (Some((i, _)), Some(&p), Some(&q)) => {
                            taken(halfway(p, q, self.arc_of(target, i, twin)?.map(|d| ProfileArc { ccw: !d.ccw, ..d })))
                        }
                        (_, Some(&p), _) | (_, _, Some(&p)) => taken(p),
                        _ => false,
                    };
                    if !gone {
                        link(&mut next, id(a), id(b))?;
                    }
                    continue;
                };
                // The sheet's face runs the edge the other way.
                cuts.reverse();
                pairs.extend(cuts.iter().map(|c| c.1));
                let at = |v: Ix| self.flat(i, target.point(v));
                // The edge as this face runs it: the sheet's face's arc the other way.
                let arc = self.arc_of(target, i, twin)?.map(|d| ProfileArc { ccw: !d.ccw, ..d });
                let stops: Vec<(Vec2, TopoId)> =
                    std::iter::once((at(a), id(a))).chain(cuts.iter().map(|c| (c.0, c.1[side]))).chain([(at(b), id(b))]).collect();
                for w in stops.windows(2) {
                    if !taken(halfway(w[0].0, w[1].0, arc)) {
                        link(&mut next, w[0].1, w[1].1)?;
                    }
                }
            }
        }
        // Across the thickness where a stretch ends at a crossing and the other side's begins;
        // each crossing is met on both sides.
        pairs.sort();
        pairs.dedup();
        let ends: Vec<TopoId> = pairs.iter().flatten().copied().filter(|c| !next.contains_key(c)).collect();
        for [c0, c1] in pairs {
            if ends.contains(&c0) {
                link(&mut next, c0, c1)?;
            } else if ends.contains(&c1) {
                link(&mut next, c1, c0)?;
            }
        }
        let first = spec.loops[0][0];
        let mut out = Vec::new();
        while let Some((&v0, _)) = next.iter().next() {
            let mut cycle = Vec::new();
            let mut v = v0;
            while let Some(w) = next.remove(&v) {
                cycle.push(v);
                v = w;
            }
            if v != v0 {
                return Err(BrepError::Invalid("cut to the edge: a face's outline does not close".into()));
            }
            out.push(cycle);
        }
        let several = out.len() > 1;
        Ok(out
            .into_iter()
            .map(|cycle| {
                let face = if !several || cycle.contains(&first) {
                    spec.id
                } else {
                    TopoId::derive("sheet.divided").id(spec.id).id(cycle.iter().min().copied().unwrap_or(spec.id)).finish()
                };
                // A wall of an earlier cut on a bend, shortened: its piece is its edges' new one.
                let surface = match spec.surface {
                    Surface::Wall(w) => (0..cycle.len())
                        .find_map(|k| {
                            let (a, b) = (cycle[k], cycle[(k + 1) % cycle.len()]);
                            split.iter().find(|c| (c.from, c.to) == (a, b) || (c.from, c.to) == (b, a))
                        })
                        .and_then(|c| match c.curve {
                            Curve::Rolled { piece, .. } => {
                                let same =
                                    (w.to.x - w.from.x) * (piece.to.x - piece.from.x) + (w.to.y - w.from.y) * (piece.to.y - piece.from.y) > 0.0;
                                Some(Surface::Wall(if same { piece } else { piece.reversed() }))
                            }
                            Curve::Arc(_) => None,
                        })
                        .unwrap_or(spec.surface),
                    other => other,
                };
                FaceSpec { id: face, surface, loops: vec![cycle] }
            })
            .collect())
    }

    /// Loop `l` of a face of strip `i` as the blank has it, its arcs too.
    fn region(&self, target: &Body, i: usize, l: Ix) -> Result<ProfileLoop, BrepError> {
        let mut ring = ProfileLoop { points: Vec::new(), vertices: Vec::new(), sides: Vec::new(), arcs: Vec::new() };
        for h in target.loop_half_edges(l) {
            ring.points.push(self.flat(i, target.point(target.half_edges()[h as usize].origin)));
            ring.arcs.push(self.arc_of(target, i, h)?);
        }
        Ok(ring)
    }
}

/// The faces in parts: each reached from the others across the edges they share. In order of
/// their least face.
fn connected(faces: &[FaceSpec]) -> Vec<Vec<usize>> {
    let mut by_edge: BTreeMap<(TopoId, TopoId), Vec<usize>> = BTreeMap::new();
    for (x, f) in faces.iter().enumerate() {
        for l in &f.loops {
            for (k, &v) in l.iter().enumerate() {
                let w = l[(k + 1) % l.len()];
                by_edge.entry((v.min(w), v.max(w))).or_default().push(x);
            }
        }
    }
    let mut seen = vec![false; faces.len()];
    let mut parts = Vec::new();
    for start in 0..faces.len() {
        if std::mem::replace(&mut seen[start], true) {
            continue;
        }
        let mut part = vec![start];
        let mut todo = vec![start];
        while let Some(x) = todo.pop() {
            for l in &faces[x].loops {
                for (k, &v) in l.iter().enumerate() {
                    let w = l[(k + 1) % l.len()];
                    for &y in &by_edge[&(v.min(w), v.max(w))] {
                        if !std::mem::replace(&mut seen[y], true) {
                            part.push(y);
                            todo.push(y);
                        }
                    }
                }
            }
        }
        parts.push(part);
    }
    parts
}

fn unit(v: Vec3) -> Vec3 {
    v * (1.0 / v.dot(v).sqrt())
}
