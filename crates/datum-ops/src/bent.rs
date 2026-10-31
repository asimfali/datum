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
use crate::sketch::{Placed, opposite_cylinder, profile_loops};
use crate::unfold::{chain_through, neutral, next_piece, rolled_center};

/// The profiles of `sketch`, which lies on a face of sheet metal `target`, cut through the sheet:
/// within the face as [`from_face`] cuts, or on across bend lines through the blank.
pub(crate) fn punch(feature: TopoId, target: &Body, sketch: &Placed) -> Result<Body, BrepError> {
    let id = sketch.face.ok_or_else(|| BrepError::Sketch("a cut of sheet metal takes a sketch on a face of it".into()))?;
    let f = target.faces().binary_search_by_key(&id, |f| f.id).map_err(|_| BrepError::LostReference(format!("face {id:?} is not in the body")))?;
    let within = || from_face(feature, target, sketch, Reach::Sheet);
    if sketch.developed.is_some() {
        return within();
    }
    let Some(blank) = Blank::new(target, f as Ix, sketch)? else { return within() };
    let profiles = blank.laid(feature, sketch)?;
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
    let inside = |p: &Vec2| home.lo < p.x && p.x < home.hi;
    if !open
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
    let metal = |p: Vec2| (0..loops.len()).all(|o| (loops[o].ring.winding(p) != 0) == (o == outer));
    if profiles.iter().any(|l| loops.iter().all(|o| !l.touches(&o.ring)) && !metal(l.points[0]) && !(0..loops.len()).any(|o| holds(l, o))) {
        return Err(unsupported("a profile lies off the sheet"));
    }

    // The pieces on the sheet, and where they cross each bend line and each edge of the sheet.
    let mut parts = Parts {
        pieces: Vec::new(),
        seams: vec![Vec::new(); blank.strips.len() - 1],
        rims: loops.iter().map(|o| vec![Vec::new(); o.edges.len()]).collect(),
    };
    for l in &profiles {
        blank.split(feature, l, &loops, &metal, &mut parts)?;
    }
    let Parts { pieces, seams, rims: crossings } = parts;
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
    let taken = |p: Vec2| profiles.iter().any(|l| l.winding(p) != 0);
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
                None => {
                    let a = blank.point(p.strip, p.from, 0);
                    Surface::Plane(Plane { origin: a, normal: n.cross(blank.point(p.strip, p.to, 0) - a) })
                }
                Some(arc) => {
                    for layer in 0..2 {
                        let axis = if arc.ccw { *n } else { -*n };
                        let arc3 = Arc { center: blank.point(p.strip, arc.center, layer), axis, radius: p.radius };
                        curves.push(CurveSpec { from: v[layer], to: w[layer], curve: Curve::Arc(arc3) });
                    }
                    // The body is outside the hole: concave around it.
                    Surface::Cylinder(Cylinder { origin: blank.point(p.strip, arc.center, 1), axis: *n, radius: p.radius, convex: !arc.ccw })
                }
            },
            Lay::Bend { dev, sign, radii, .. } => {
                let uv = |q: Vec2| Vec2::new(q.x - st.lo, sign * q.y);
                let piece = Rolled {
                    dev: *dev,
                    from: uv(p.from),
                    to: uv(p.to),
                    arc: p.arc.map(|a| ProfileArc { center: uv(a.center), ccw: a.ccw == (*sign > 0.0) }),
                };
                for layer in 0..2 {
                    curves.push(CurveSpec { from: v[layer], to: w[layer], curve: Curve::Rolled { piece, radius: radii[layer] } });
                }
                // Out of the body (into the hole) is left of the profile on the blank: of the
                // development too unless it is mirrored.
                Surface::Wall(if *sign < 0.0 { piece.reversed() } else { piece })
            }
        };
        faces.push(FaceSpec { id: p.wall, surface, loops: vec![vec![v[0], w[0], w[1], v[1]]] });
    }
    // Vertices of the body are kept as they are; a new one is a new point.
    vertices.extend(
        new.into_iter().filter(|(id, _)| target.vertices().binary_search_by_key(id, |v| v.id).is_err()).map(|(id, point)| Vertex { id, point }),
    );
    // A flange on the sheet's side (D205) is laid off the blank: a cut reaching it comes later.
    let flanges = crate::unfold::branches(target);
    if rims.iter().any(|r| flanges.contains(&r.beyond)) || gone.iter().any(|t| flanges.contains(t)) {
        return Err(unsupported("a cut reaching a flange on the sheet's side comes later"));
    }
    let cuts = Cuts { open: !rims.is_empty() || !gone.is_empty(), pieces, seams, rims, profiles };
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
    if !whole(&faces) {
        return Err(unsupported("the cut divides the sheet into two parts: comes later"));
    }
    build_with_curves(vertices, faces, curves)
}

/// How close a profile of a cut of sheet metal comes to a bend: `distance` on the blank from it
/// to the bend's nearer bend line, or none when it reaches into the bend; under `clearance`,
/// 2t + R (R the bend's inner radius). Closer than that a hole distorts on bending, as process
/// engineers check before a blank is cut.
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
    let Some(blank) = Blank::new(target, f as Ix, sketch)? else { return Ok(Vec::new()) };
    let mut near = Vec::new();
    for l in blank.laid(feature, sketch)? {
        let (lo, hi) = reach(&l);
        let nearest = blank
            .strips
            .iter()
            .filter_map(|s| match s.lay {
                Lay::Bend { radii, .. } => Some(((s.lo - hi).max(lo - s.hi), 2.0 * blank.thickness + radii[0].min(radii[1]))),
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

/// How far loop `l` reaches along X: its points and the extremes of its arcs.
fn reach(l: &ProfileLoop) -> (f64, f64) {
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    let n = l.points.len();
    for (k, &a) in l.points.iter().enumerate() {
        (lo, hi) = (lo.min(a.x), hi.max(a.x));
        let Some(arc) = l.arc(k) else { continue };
        let (c, b) = (arc.center, l.points[(k + 1) % n]);
        let angle = |p: Vec2| libm::atan2(p.y - c.y, p.x - c.x);
        // The turn from `a` toward `b` the arc's way, and whether it passes direction `t`.
        let turn = |from: f64, to: f64| {
            let d = if arc.ccw { to - from } else { from - to };
            if d <= 0.0 { d + TAU } else { d }
        };
        let (start, sweep) = (angle(a), turn(angle(a), angle(b)));
        let r = libm::hypot(a.x - c.x, a.y - c.y);
        if turn(start, 0.0) <= sweep {
            hi = hi.max(c.x + r);
        }
        if turn(start, PI) <= sweep {
            lo = lo.min(c.x - r);
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
    seams: Vec<Vec<(f64, [TopoId; 2])>>,
    rims: Vec<Vec<Crossings>>,
}

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

/// Whether arc `k` of `l` stays between the bend lines of `strip`: it crosses neither.
fn arc_within(l: &ProfileLoop, k: usize, arc: ProfileArc, strip: &Strip) -> bool {
    let (a, b) = (l.points[k], l.points[(k + 1) % l.points.len()]);
    let r = libm::hypot(a.x - arc.center.x, a.y - arc.center.y);
    [strip.lo, strip.hi].iter().all(|&s| arc_crossings(a, b, arc, r, s).is_empty())
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

/// The faces of the sketch's side of the sheet between two bend lines and the faces across them,
/// as they lie on the blank: from `lo` to `hi` in `s`. One face each, or the pieces a cut divided
/// it into (D201).
struct Strip {
    faces: [Vec<Ix>; 2],
    lo: f64,
    hi: f64,
    lay: Lay,
}

enum Lay {
    /// A wall: its unit normal out of the sketch's side, the unit `e` along `s`, and `base`, the
    /// point where `s` is `at` and `y` zero.
    Plane { n: Vec3, e: Vec3, base: Vec3, at: f64 },
    /// A bend: its development from the bend line at `lo` (`u = s − lo`, `v = sign·y`), and the
    /// radii of the sketch's side and of the other. No development without a K-factor.
    Bend { dev: Development, sign: f64, radii: [f64; 2], neutral: Option<f64> },
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
    pieces: Vec<Piece>,
    seams: Vec<Vec<(f64, [TopoId; 2])>>,
    rims: Vec<Rim>,
    profiles: Vec<ProfileLoop>,
}

/// The sheet laid flat from the sketch's face: its faces in order of `s`.
struct Blank {
    o: Vec3,
    a: Vec3,
    e: Vec3,
    thickness: f64,
    strips: Vec<Strip>,
    home: usize,
}

impl Blank {
    /// The blank of the sheet `target` whose face `f` the sketch lies on; none for a sheet
    /// without bends.
    fn new(target: &Body, f: Ix, sketch: &Placed) -> Result<Option<Self>, BrepError> {
        let unsupported = |m: &str| BrepError::Unsupported(format!("cut across a bend: {m}"));
        let surface = |x: Ix| target.faces()[x as usize].surface;
        let skip = crate::unfold::branches(target);
        let mut front = chain_through(target, f, &skip)?;
        let Some(bend) = front.iter().position(|s| matches!(surface(s[0]), Surface::Cylinder(_))) else { return Self::plate(target, &front, sketch) };
        let Surface::Cylinder(c0) = surface(front[bend][0]) else { return Ok(None) };
        let (across, _) =
            opposite_cylinder(target, front[bend][0] as usize).ok_or_else(|| unsupported("a bend without the face across the sheet"))?;
        let mut back = chain_through(target, across, &skip)?;
        // The other side strip by strip: a bend across each bend, a wall across each wall.
        let paired = |back: &[Vec<Ix>]| {
            back.len() == front.len()
                && front.iter().zip(back).all(|(xs, ys)| match surface(xs[0]) {
                    Surface::Cylinder(_) => opposite_cylinder(target, xs[0] as usize).is_some_and(|(g, _)| ys.contains(&g)),
                    _ => matches!(surface(ys[0]), Surface::Plane(_)),
                })
        };
        if !paired(&back) {
            back.reverse();
            if !paired(&back) {
                return Err(unsupported("the sheet's two sides do not pair face by face"));
            }
        }
        let [o, _, _, z] = sketch.frame;
        let a = unit(c0.axis);
        let e = unit(a.cross(z));
        let s_of = |p: Vec3| (p - o).dot(e);
        let y_of = |p: Vec3| (p - o).dot(a);
        // A point of the bend line between strips `xs` and `ys`.
        let seam = |xs: &[Ix], ys: &[Ix]| {
            let he = |h: Ix| target.half_edges()[h as usize];
            xs.iter()
                .find_map(|&x| {
                    target
                        .loop_half_edges(target.faces()[x as usize].first_loop)
                        .find(|&h| ys.contains(&target.loops()[he(he(h).twin).loop_ix as usize].face))
                })
                .map(|h| target.point(he(h).origin))
                .ok_or_else(|| BrepError::Invalid("cut across a bend: no bend line between faces of the sheet".into()))
        };
        // `s` grows along the chain.
        let mut home = front.iter().position(|s| s.contains(&f)).ok_or_else(|| BrepError::Invalid("the sketch's face is not in its sheet".into()))?;
        let middle = {
            let l = target.faces()[f as usize].first_loop;
            target.loop_points(l).fold(Vec3::ZERO, |m, p| m + p) * (1.0 / f64::from(target.loops()[l as usize].len))
        };
        let ahead = if home + 1 < front.len() {
            s_of(seam(&front[home], &front[home + 1])?) > s_of(middle)
        } else {
            s_of(seam(&front[home], &front[home - 1])?) < s_of(middle)
        };
        if !ahead {
            front.reverse();
            back.reverse();
            home = front.len() - 1 - home;
        }
        let k = sketch.k_factor;
        let n = front.len();
        let mut strips: Vec<Option<Strip>> = (0..n).map(|_| None).collect();
        let lo = if home > 0 { s_of(seam(&front[home], &front[home - 1])?) } else { f64::NEG_INFINITY };
        let hi = if home + 1 < n { s_of(seam(&front[home], &front[home + 1])?) } else { f64::INFINITY };
        strips[home] = Some(Strip { faces: [front[home].clone(), back[home].clone()], lo, hi, lay: Lay::Plane { n: z, e, base: o, at: 0.0 } });
        // Each face from its bend line nearer the sketch's face, out each way.
        let order = (home + 1..n).map(|i| (i, true)).chain((0..home).rev().map(|i| (i, false)));
        for (i, up) in order {
            let near = if up { strips[i - 1].as_ref().map(|s| s.hi) } else { strips[i + 1].as_ref().map(|s| s.lo) };
            let near = near.ok_or_else(|| BrepError::Invalid("cut across a bend: a face laid out of order".into()))?;
            let (low, high) = (
                if i > 0 { Some(seam(&front[i], &front[i - 1])?) } else { None },
                if i + 1 < n { Some(seam(&front[i], &front[i + 1])?) } else { None },
            );
            let (lay, length) = match (surface(front[i][0]), surface(back[i][0])) {
                (Surface::Cylinder(c), Surface::Cylinder(other)) => {
                    let (Some(low), Some(high)) = (low, high) else { return Err(unsupported("a bend ends the sheet")) };
                    let origin = c.origin - a * y_of(c.origin);
                    let radial = |q: Vec3| {
                        let w = q - origin;
                        unit(w - a * w.dot(a))
                    };
                    let sign = if c.convex { 1.0 } else { -1.0 };
                    let (axis, start) = (a * sign, radial(low));
                    let end = radial(high);
                    let angle = libm::atan2(axis.dot(start.cross(end)), start.dot(end));
                    if angle <= 0.0 {
                        return Err(BrepError::Invalid("cut across a bend: a bend turning back".into()));
                    }
                    let neutral = k.map(|k| neutral(c.radius, other.radius, k));
                    let dev = Development { origin, axis, start, neutral: neutral.unwrap_or(1.0) };
                    (Lay::Bend { dev, sign, radii: [c.radius, other.radius], neutral }, neutral.map_or(0.0, |r| r * angle))
                }
                (Surface::Plane(p), _) => {
                    let nn = unit(p.normal);
                    let ee = unit(a.cross(nn));
                    let q = if up { low } else { high }.ok_or_else(|| BrepError::Invalid("cut across a bend: a wall off the chain".into()))?;
                    let far = if up { high } else { low };
                    let length = far.map_or(f64::INFINITY, |r| (r - q).dot(ee).abs());
                    (Lay::Plane { n: nn, e: ee, base: q - a * y_of(q), at: near }, length)
                }
                _ => return Err(unsupported("a face of the sheet neither a wall nor a bend")),
            };
            let (lo, hi) = if up { (near, near + length) } else { (near - length, near) };
            let lay = match lay {
                // A wall laid down from its far line keeps `at` where `base` is: its near line.
                Lay::Plane { n, e, base, at } => Lay::Plane { n, e, base, at },
                bend => bend,
            };
            strips[i] = Some(Strip { faces: [front[i].clone(), back[i].clone()], lo, hi, lay });
        }
        let strips: Vec<Strip> = strips.into_iter().flatten().collect();
        let thickness = strips
            .iter()
            .find_map(|s| match s.lay {
                Lay::Bend { radii, .. } => Some((radii[0] - radii[1]).abs()),
                Lay::Plane { .. } => None,
            })
            .unwrap_or(0.0);
        Ok(Some(Self { o, a, e, thickness, strips, home }))
    }

    /// The blank of a sheet without bends (a face of sheet metal, D195; D202): one strip, laid in
    /// the sketch's own axes; the other side the sheet's other side. None for anything else.
    fn plate(target: &Body, front: &[Vec<Ix>], sketch: &Placed) -> Result<Option<Self>, BrepError> {
        let [home] = front else { return Ok(None) };
        let Surface::Plane(p) = target.faces()[home[0] as usize].surface else { return Ok(None) };
        let Ok(sides) = crate::unfold::sheet_sides(target) else { return Ok(None) };
        let Some(&other) = sides.iter().find(|x| !home.contains(x)) else { return Ok(None) };
        let Surface::Plane(q) = target.faces()[other as usize].surface else { return Ok(None) };
        let [o, x, y, z] = sketch.frame;
        let n = unit(p.normal);
        let back = chain_through(target, other, &crate::unfold::branches(target))?.concat();
        let strip = Strip { faces: [home.clone(), back], lo: f64::NEG_INFINITY, hi: f64::INFINITY, lay: Lay::Plane { n: z, e: x, base: o, at: 0.0 } };
        Ok(Some(Self { o, a: y, e: x, thickness: (p.origin - q.origin).dot(n), strips: vec![strip], home: 0 }))
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
            Lay::Plane { n, e, base, at } => {
                let q = *base + *e * (p.x - at) + self.a * p.y;
                if layer == 1 { q - *n * self.thickness } else { q }
            }
            Lay::Bend { dev, sign, radii, .. } => dev.point(Vec2::new(p.x - st.lo, sign * p.y), radii[layer]),
        }
    }

    /// Where `q`, a point of strip `i`, lies on the blank.
    fn flat(&self, i: usize, q: Vec3) -> Vec2 {
        let st = &self.strips[i];
        match &st.lay {
            Lay::Plane { e, base, at, .. } => Vec2::new(at + (q - *base).dot(*e), (q - self.o).dot(self.a)),
            Lay::Bend { dev, sign, .. } => {
                let (uv, _) = dev.unroll(q, 0.5 * (st.hi - st.lo));
                Vec2::new(st.lo + uv.x, sign * uv.y)
            }
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
            (Some(Curve::Rolled { piece, .. }), Lay::Bend { dev, .. }) => piece.arc.map(|pa| {
                let origin = flat(piece.dev.point(Vec2::new(0.0, 0.0), piece.dev.neutral));
                let (center, ccw) = rolled_center(&piece, pa, origin, dev.axis, self.a);
                ProfileArc { center, ccw }
            }),
            (Some(Curve::Rolled { .. }), Lay::Plane { .. }) => return Err(BrepError::Invalid("a rolled edge on a wall".into())),
        })
    }

    /// The strip `s` lies in, off every bend line.
    fn strip(&self, s: f64) -> usize {
        self.strips.partition_point(|st| st.hi < s)
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
    /// `outer` (D202), each crossing kept with its line or edge; the pieces off the sheet left
    /// out.
    fn split(&self, feature: TopoId, l: &ProfileLoop, loops: &[Outline], metal: &dyn Fn(Vec2) -> bool, out: &mut Parts) -> Result<(), BrepError> {
        let Parts { pieces, seams, rims } = out;
        let lines: Vec<f64> = self.strips[..self.strips.len() - 1].iter().map(|s| s.hi).collect();
        let n = l.points.len();
        let mut last: Option<TopoId> = None;
        // Corners of the profile on an edge of the sheet, and whether each segment's first and
        // last pieces are on the sheet.
        let corners: Vec<Option<(usize, usize)>> = l.points.iter().map(|&a| on_edge(a, loops)).collect::<Result<_, _>>()?;
        let mut ends: Vec<[bool; 2]> = Vec::with_capacity(n);
        for k in 0..n {
            let mut on_sheet = [false; 2];
            let (a, b, arc) = (l.points[k], l.points[(k + 1) % n], l.arc(k));
            if lines.contains(&a.x) {
                return Err(BrepError::Degenerate("a corner of a profile lies on a bend line".into()));
            }
            let radius = arc.map_or(0.0, |c| libm::hypot(a.x - c.center.x, a.y - c.center.y));
            let mut cuts: Vec<(f64, Vec2, Line)> = Vec::new();
            for (j, &s) in lines.iter().enumerate() {
                match arc {
                    None => {
                        let (da, db) = (a.x - s, b.x - s);
                        if (da < 0.0) != (db < 0.0) {
                            let t = da / (da - db);
                            cuts.push((t, Vec2::new(s, a.y + t * (b.y - a.y)), Line::Bend(j)));
                        }
                    }
                    Some(c) => {
                        if (s - c.center.x).abs() == radius {
                            return Err(BrepError::Degenerate("a profile touches a bend line".into()));
                        }
                        cuts.extend(arc_crossings(a, b, c, radius, s).into_iter().map(|(t, p)| (t, p, Line::Bend(j))));
                    }
                }
            }
            // The edges of the blank it crosses: the sheet's sides and ends, earlier cuts.
            for (o, outline) in loops.iter().enumerate() {
                let m = outline.ring.points.len();
                for e in 0..m {
                    let (p, q) = (outline.ring.points[e], outline.ring.points[(e + 1) % m]);
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
            let cross = |m: usize| [0, 1].map(|layer| TopoId::derive("cut.cross").id(feature).id(l.sides[k]).index(m as u64).index(layer).finish());
            let mut points = vec![(a, l.vertices[k])];
            for (m, &(_, p, line)) in cuts.iter().enumerate() {
                points.push((p, cross(m)));
                match line {
                    Line::Bend(j) => seams[j].push((p.y, cross(m))),
                    Line::Rim(o, e) => rims[o][e].push((p, cross(m))),
                }
            }
            points.push((b, l.vertices[(k + 1) % n]));
            // A circle's next arc goes on as its first did.
            let goes_on = k > 0 && matches!((l.arc(k - 1), arc), (Some(p), Some(q)) if p == q);
            let count = points.len() - 1;
            for (i, w) in points.windows(2).enumerate() {
                let ((from, v), (to, u)) = (w[0], w[1]);
                let middle = halfway(from, to, arc);
                let wall = match last {
                    Some(prev) if i > 0 || goes_on => next_piece(prev),
                    _ => l.sides[k],
                };
                last = Some(wall);
                // Off the metal: no wall there.
                let on = metal(middle);
                if i == 0 {
                    on_sheet[0] = on;
                }
                if i + 1 == count {
                    on_sheet[1] = on;
                }
                if !on {
                    continue;
                }
                pieces.push(Piece { from, to, middle, arc, radius, strip: self.strip(middle.x), ends: [v, u], wall });
            }
            ends.push(on_sheet);
        }
        // A corner on an edge where the profile goes off the sheet or onto it crosses the edge
        // there; one with the sheet on both sides only touches it.
        for (k, e) in corners.into_iter().enumerate() {
            let Some((o, e)) = e else { continue };
            match (ends[(k + n - 1) % n][1], ends[k][0]) {
                (true, true) => return Err(BrepError::Degenerate("a profile touches an edge of the sheet from inside".into())),
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
        let Cuts { open, pieces, seams, rims, profiles } = cuts;
        let members = &self.strips[i].faces[side];
        let he = |h: Ix| target.half_edges()[h as usize];
        let id = |v: Ix| target.vertices()[v as usize].id;
        let y_of = |v: Ix| (target.point(v) - self.o).dot(self.a);
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
            let taken = |p: Vec2| profiles.iter().any(|l| l.winding(p) != 0);
            let mut edges: Vec<Ix> = Vec::new();
            let mut kept: Vec<Vec<TopoId>> = Vec::new();
            for (k, l) in target.face_loops(x).enumerate() {
                let crossed = k > 0 && *open && target.loop_half_edges(l).any(|h| !self.on_rim(target, rims, i, side, h).is_empty());
                if k == 0 || crossed {
                    edges.extend(target.loop_half_edges(l));
                } else if !(*open && taken(at(he(target.loop_half_edges(l).start).origin))) {
                    kept.push(spec.loops[k].clone());
                }
            }
            for h in edges {
                let (u, v) = (he(h).origin, he(target.next(h)).origin);
                let beside = target.loops()[he(he(h).twin).loop_ix as usize].face;
                let line = if i > 0 && self.strips[i - 1].faces[side].contains(&beside) {
                    Some(i - 1)
                } else if i + 1 < self.strips.len() && self.strips[i + 1].faces[side].contains(&beside) {
                    Some(i)
                } else {
                    None
                };
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
                let (yu, yv) = (y_of(u), y_of(v));
                if seams[j].iter().any(|c| c.0 == yu || c.0 == yv) {
                    return Err(BrepError::Degenerate("a profile crosses a bend line at a corner of the sheet".into()));
                }
                let mut on: Vec<&(f64, [TopoId; 2])> = seams[j].iter().filter(|c| (c.0 - yu) * (c.0 - yv) < 0.0).collect();
                on.sort_by(|p, q| (p.0 - yu).abs().total_cmp(&(q.0 - yu).abs()));
                let stops: Vec<(f64, TopoId)> =
                    std::iter::once((yu, id(u))).chain(on.iter().map(|c| (c.0, c.1[side]))).chain([(yv, id(v))]).collect();
                let s = self.strips[j].hi;
                for w in stops.windows(2) {
                    let middle = Vec2::new(s, 0.5 * (w[0].0 + w[1].0));
                    if profiles.iter().all(|p| p.winding(middle) == 0) {
                        let from = if w[0].1 == id(u) { at(u) } else { Vec2::new(s, w[0].0) };
                        link(w[0].1, w[1].1, from, None)?;
                    }
                }
            }
            // Profiles run against the outline on the sketch's side, along it on the other.
            for p in pieces.iter().filter(|p| p.strip == i && (regions.is_empty() || regions[m].winding(p.middle) != 0)) {
                if side == 0 {
                    link(p.ends[1][0], p.ends[0][0], p.to, p.arc.map(|a| ProfileArc { ccw: !a.ccw, ..a }))?
                } else {
                    link(p.ends[0][1], p.ends[1][1], p.from, p.arc)?
                }
            }
            let first: Vec<TopoId> = spec.loops[0].clone();
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
                if cycle.iter().any(|v| first.contains(v)) { outers.push((cycle, ring)) } else { holes.push((cycle, ring)) }
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
        let taken = |p: Vec2| profiles.iter().any(|l| l.winding(p) != 0);
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

/// Whether every face is reached from the first across the edges they share: one piece.
fn whole(faces: &[FaceSpec]) -> bool {
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
    let mut todo = vec![0];
    seen[0] = !faces.is_empty();
    while let Some(x) = todo.pop() {
        for l in &faces[x].loops {
            for (k, &v) in l.iter().enumerate() {
                let w = l[(k + 1) % l.len()];
                for &y in &by_edge[&(v.min(w), v.max(w))] {
                    if !seen[y] {
                        seen[y] = true;
                        todo.push(y);
                    }
                }
            }
        }
    }
    seen.into_iter().all(|s| s)
}

fn unit(v: Vec3) -> Vec3 {
    v * (1.0 / v.dot(v).sqrt())
}
