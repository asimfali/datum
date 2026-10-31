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
use datum_math::{Vec2, Vec3};

use crate::cut::{Reach, between, from_face};
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
    // Within the sketch's face, a cut as before.
    let home = &blank.strips[blank.home];
    let inside = |p: &Vec2| home.lo < p.x && p.x < home.hi;
    if profiles.iter().all(|l| l.points.iter().all(inside) && (0..l.points.len()).all(|k| l.arc(k).is_none_or(|a| arc_within(l, k, a, home)))) {
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
    let edges = blank.outline(target)?;
    let edge_boxes: Vec<_> = edges.iter().map(ProfileLoop::bounds).collect();
    let mut winding = vec![0i32; profiles.len()];
    for (p, e) in between(&boxes, &edge_boxes) {
        if profiles[p].touches(&edges[e]) {
            return Err(unsupported("a profile meets an edge of the sheet or of another cut: a cut open to the edge comes later"));
        }
        if profiles[p].winding(edges[e].points[0]) != 0 {
            return Err(unsupported("a cut of the sheet lies inside a profile"));
        }
        winding[p] += edges[e].winding(profiles[p].points[0]);
    }
    if winding.contains(&0) {
        return Err(unsupported("a profile leaves the sheet"));
    }

    // The pieces, and where they cross each bend line.
    let mut pieces: Vec<Piece> = Vec::new();
    let mut seams: Vec<Vec<(f64, [TopoId; 2])>> = vec![Vec::new(); blank.strips.len() - 1];
    for l in &profiles {
        blank.split(feature, l, &mut pieces, &mut seams)?;
    }

    // The body: new vertices and curves, the walls, the faces crossed built anew.
    let mut vertices: Vec<Vertex> = target.vertices().to_vec();
    let mut curves = target.curve_specs();
    let mut faces: Vec<FaceSpec> = Vec::new();
    for p in &pieces {
        let st = &blank.strips[p.strip];
        for layer in 0..2 {
            vertices.push(Vertex { id: p.ends[0][layer], point: blank.point(p.strip, p.from, layer) });
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
    let crossed: BTreeSet<usize> = pieces.iter().map(|p| p.strip).collect();
    let mut rebuilt: BTreeSet<Ix> = BTreeSet::new();
    for &i in &crossed {
        for side in 0..2 {
            faces.extend(blank.rebuilt(target, i, side, &pieces, &seams, &profiles)?);
            rebuilt.extend(blank.strips[i].faces[side].iter().copied());
        }
    }
    faces.extend((0..target.faces().len() as Ix).filter(|x| !rebuilt.contains(x)).map(|x| target.face_spec(x)));
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
        let mut front = chain_through(target, f)?;
        let Some(bend) = front.iter().position(|s| matches!(surface(s[0]), Surface::Cylinder(_))) else { return Ok(None) };
        let Surface::Cylinder(c0) = surface(front[bend][0]) else { return Ok(None) };
        let (across, _) =
            opposite_cylinder(target, front[bend][0] as usize).ok_or_else(|| unsupported("a bend without the face across the sheet"))?;
        let mut back = chain_through(target, across)?;
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
    fn outline(&self, target: &Body) -> Result<Vec<ProfileLoop>, BrepError> {
        let he = |h: Ix| target.half_edges()[h as usize];
        let face_of = |h: Ix| target.loops()[he(h).loop_ix as usize].face;
        let strip_of: BTreeMap<Ix, usize> = self.strips.iter().enumerate().flat_map(|(i, s)| s.faces[0].iter().map(move |&x| (x, i))).collect();
        let mut next: BTreeMap<Ix, (Ix, Vec2, Option<ProfileArc>)> = BTreeMap::new();
        for (&x, &i) in &strip_of {
            for l in target.face_loops(x) {
                for h in target.loop_half_edges(l) {
                    if strip_of.contains_key(&face_of(he(h).twin)) {
                        continue;
                    }
                    let (v, w) = (he(h).origin, he(target.next(h)).origin);
                    next.insert(v, (w, self.flat(i, target.point(v)), self.arc_of(target, i, h)?));
                }
            }
        }
        let mut loops = Vec::new();
        while let Some((&v0, _)) = next.iter().next() {
            let mut ring = ProfileLoop { points: Vec::new(), vertices: Vec::new(), sides: Vec::new(), arcs: Vec::new() };
            let mut v = v0;
            while let Some((w, p, arc)) = next.remove(&v) {
                ring.points.push(p);
                ring.arcs.push(arc);
                v = w;
            }
            if v != v0 {
                return Err(BrepError::Invalid("cut across a bend: the blank's outline does not close".into()));
            }
            loops.push(ring);
        }
        Ok(loops)
    }

    /// Profile `l` cut into pieces at the bend lines, each crossing kept with its line.
    fn split(&self, feature: TopoId, l: &ProfileLoop, pieces: &mut Vec<Piece>, seams: &mut [Vec<(f64, [TopoId; 2])>]) -> Result<(), BrepError> {
        let lines: Vec<f64> = self.strips[..self.strips.len() - 1].iter().map(|s| s.hi).collect();
        let n = l.points.len();
        let mut last: Option<TopoId> = None;
        for k in 0..n {
            let (a, b, arc) = (l.points[k], l.points[(k + 1) % n], l.arc(k));
            if lines.contains(&a.x) {
                return Err(BrepError::Degenerate("a corner of a profile lies on a bend line".into()));
            }
            let radius = arc.map_or(0.0, |c| libm::hypot(a.x - c.center.x, a.y - c.center.y));
            let mut cuts: Vec<(f64, Vec2, usize)> = Vec::new();
            for (j, &s) in lines.iter().enumerate() {
                match arc {
                    None => {
                        let (da, db) = (a.x - s, b.x - s);
                        if (da < 0.0) != (db < 0.0) {
                            let t = da / (da - db);
                            cuts.push((t, Vec2::new(s, a.y + t * (b.y - a.y)), j));
                        }
                    }
                    Some(c) => {
                        if (s - c.center.x).abs() == radius {
                            return Err(BrepError::Degenerate("a profile touches a bend line".into()));
                        }
                        cuts.extend(arc_crossings(a, b, c, radius, s).into_iter().map(|(t, p)| (t, p, j)));
                    }
                }
            }
            cuts.sort_by(|x, y| x.0.total_cmp(&y.0));
            let cross = |m: usize| [0, 1].map(|layer| TopoId::derive("cut.cross").id(feature).id(l.sides[k]).index(m as u64).index(layer).finish());
            let mut points = vec![(a, l.vertices[k])];
            for (m, &(_, p, j)) in cuts.iter().enumerate() {
                points.push((p, cross(m)));
                seams[j].push((p.y, cross(m)));
            }
            points.push((b, l.vertices[(k + 1) % n]));
            // A circle's next arc goes on as its first did.
            let goes_on = k > 0 && matches!((l.arc(k - 1), arc), (Some(p), Some(q)) if p == q);
            for (i, w) in points.windows(2).enumerate() {
                let ((from, v), (to, u)) = (w[0], w[1]);
                let middle = match arc {
                    None => Vec2::new(0.5 * (from.x + to.x), 0.5 * (from.y + to.y)),
                    Some(c) => {
                        let (sn, cs) = libm::sincos(0.5 * c.turn(from, to));
                        let (dx, dy) = (from.x - c.center.x, from.y - c.center.y);
                        Vec2::new(c.center.x + dx * cs - dy * sn, c.center.y + dx * sn + dy * cs)
                    }
                };
                let wall = match last {
                    Some(prev) if i > 0 || goes_on => next_piece(prev),
                    _ => l.sides[k],
                };
                last = Some(wall);
                pieces.push(Piece { from, to, middle, arc, radius, strip: self.strip(middle.x), ends: [v, u], wall });
            }
        }
        Ok(())
    }

    /// The faces of side `side` (0: the sketch's side) of strip `i` with the pieces in them: each
    /// face's outline broken where profiles cross its bend lines and run along them instead, a
    /// profile within it a hole. A face the profiles cross from one bend line to the other comes
    /// out in pieces (D201): the piece with the face's first vertex keeps its ID, the others take
    /// one from it and their lowest vertex; every hole goes to the piece around it.
    fn rebuilt(
        &self,
        target: &Body,
        i: usize,
        side: usize,
        pieces: &[Piece],
        seams: &[Vec<(f64, [TopoId; 2])>],
        profiles: &[ProfileLoop],
    ) -> Result<Vec<FaceSpec>, BrepError> {
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
            let outer = target.faces()[x as usize].first_loop;
            for h in target.loop_half_edges(outer) {
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
                    link(id(u), id(v), at(u), self.arc_of(target, i, h)?)?;
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
                let loops = std::iter::once(outer.clone()).chain(spec.loops[1..].iter().cloned()).chain(holes.into_iter().map(|h| h.0)).collect();
                out.push(FaceSpec { id: spec.id, surface: spec.surface, loops });
                continue;
            }
            // Divided: each hole, old or new, to the piece around a point of it.
            let point_of = |v: TopoId| -> Option<Vec2> {
                let ix = target.vertices().binary_search_by_key(&v, |w| w.id).ok()?;
                Some(at(ix as Ix))
            };
            let mut loops: Vec<Vec<Vec<TopoId>>> = outers.iter().map(|(c, _)| vec![c.clone()]).collect();
            let old = spec.loops[1..].iter().map(|l| (l.clone(), l.first().and_then(|&v| point_of(v))));
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

fn unit(v: Vec3) -> Vec3 {
    v * (1.0 / v.dot(v).sqrt())
}
