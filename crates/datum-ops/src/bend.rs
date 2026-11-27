//! Bending sheet metal along a line of a sketch on a flat face (D312): the 2D half of the
//! operation, before any 3D geometry is made.
//!
//! The line of the sketch is an infinite axis. The material of the face, seen in the sketch's
//! coordinates, meets it in strips: each connected component of the axis over the material,
//! extended to the component's ends. A strip is bent whole, and only the strips that the sketch's
//! own segment reaches over a length of its own are bent: a segment ending on the edge of a cut
//! does not reach the strip beyond it. The area of the material on each side of the line tells
//! which side the caller bends by default.
//!
//! Which side of the line a point is on, and whether a point is material, are exact predicates;
//! where the axis meets an arc comes from the equation of the line and the circle. Areas are
//! `f64`, chords and circular segments summed in a fixed order, as [`ProfileLoop::area`] does.

use datum_brep::{Body, BrepError, Ix, ProfileArc, ProfileLoop, Surface};
use datum_math::{Carrier, Sign, Vec2, crossings, inside_circle, orient2d};

use crate::cut::{Frame, outline};
use crate::sketch::Placed;

/// One strip of the bend line: a connected component of the infinite line through the sketch's
/// segment over the face's material, extended to the component's ends (sketch coordinates,
/// ordered along the line's direction).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BendStrip {
    pub ends: [Vec2; 2],
}

/// The strips of a bend line and the areas of the material on either side of it.
#[derive(Clone, Debug, PartialEq)]
pub struct BendStrips {
    pub strips: Vec<BendStrip>,
    /// Area of the material on the left / right of the line's direction.
    pub sides: [f64; 2],
}

/// Stage 1 of the bend by a sketch line (D312): the 2D half, the strips the sketch's segment
/// reaches and the areas of the material on each side of the line.
pub fn bend_strips(body: &Body, sketch: &Placed, line: &str) -> Result<BendStrips, BrepError> {
    let unsupported = |m: &str| BrepError::Unsupported(format!("bend: {m}"));
    let id = sketch.face.ok_or_else(|| unsupported("a bend takes a sketch placed on a face of its body"))?;
    let f =
        body.faces().binary_search_by_key(&id, |f| f.id).map_err(|_| BrepError::LostReference(format!("bend: face {id:?} is not in the body")))?;
    // A sketch on a cylinder face carries its development, and its face is a cylinder: a sketch
    // with a plane frame lies on a plane face (`cut::from_face`), so the face decides.
    if !matches!(body.faces()[f].surface, Surface::Plane(_)) {
        return Err(unsupported("a bend takes a sketch on a plane face"));
    }
    let [o, x, y, z] = sketch.frame;
    // The face's material in the sketch's coordinates: its outline, the first loop outer and the
    // rest holes (D312).
    let loops = outline(body, f as Ix, &Frame::Plane { o, x, y, z })?;

    let l = sketch.lines.get(line).ok_or_else(|| BrepError::LostReference(format!("bend: no line `{line}` in the sketch")))?;
    let point = |n: &str| sketch.solved.points.get(n).copied().ok_or_else(|| BrepError::Sketch(format!("bend: no point `{n}` in the sketch")));
    let (a, b) = (point(&l.from)?, point(&l.to)?);
    let axis = Axis::new(a, b).ok_or_else(|| BrepError::Degenerate(format!("bend: the line `{line}` of the sketch has no length")))?;

    let parts = runs(&loops, &axis)?;
    if parts.is_empty() {
        return Err(BrepError::Degenerate("bend: the line meets no material of the face".into()));
    }
    // The strip the segment reaches over a length of its own: a segment ending on an edge of a
    // cut touches the strip beyond it at a point, and does not bend it.
    let reach = axis.at(b);
    let strips: Vec<BendStrip> = parts.iter().filter(|r| r.t[0].max(0.0) < r.t[1].min(reach)).map(|r| BendStrip { ends: r.ends }).collect();
    if strips.is_empty() {
        return Err(BrepError::Degenerate("bend: the line of the sketch reaches no material of the face".into()));
    }
    Ok(BendStrips { strips, sides: material_sides(&loops, &axis) })
}

/// The bend line in the sketch's coordinates: two points of the sketch's line, exactly, and the
/// unit direction from the first to the second.
struct Axis {
    a: Vec2,
    b: Vec2,
    along: Vec2,
}

impl Axis {
    fn new(a: Vec2, b: Vec2) -> Option<Self> {
        let d = Vec2::new(b.x - a.x, b.y - a.y);
        let len = libm::hypot(d.x, d.y);
        (len > 0.0).then(|| Self { a, b, along: Vec2::new(d.x / len, d.y / len) })
    }

    /// Where `p` lies along the line, from the segment's start.
    fn at(&self, p: Vec2) -> f64 {
        (p.x - self.a.x) * self.along.x + (p.y - self.a.y) * self.along.y
    }

    /// Which side of the line `p` is on, exactly: `Positive` left of the direction.
    fn side(&self, p: Vec2) -> Sign {
        orient2d(self.a, self.b, p)
    }
}

/// `p` in the line's coordinates: along the line from its start and to the left of it.
fn flat(axis: &Axis, p: Vec2) -> Vec2 {
    let d = Vec2::new(p.x - axis.a.x, p.y - axis.a.y);
    Vec2::new(d.x * axis.along.x + d.y * axis.along.y, -d.x * axis.along.y + d.y * axis.along.x)
}

/// The sketch's point at `p` of the line's coordinates.
fn unflat(axis: &Axis, p: Vec2) -> Vec2 {
    Vec2::new(axis.a.x + axis.along.x * p.x - axis.along.y * p.y, axis.a.y + axis.along.y * p.x + axis.along.x * p.y)
}

/// A strip of the line over the material, with its ends' places along the line.
struct Run {
    ends: [Vec2; 2],
    t: [f64; 2],
}

/// The components of the line over the material of the face, in the line's direction.
///
/// Stops are the same point exactly when they are the same point (`Vec2` equality): the places one
/// crossing is found at cannot differ but by rounding. A window whose middle lies on an edge has no
/// length of its own there, where `ProfileLoop::winding` is not defined: it takes the material of
/// both its neighbours when both are material — a touch of the boundary, as a line tangent to a
/// hole, joins the strip — and none otherwise, ending the strip at the nearer stop. Nothing else
/// decides between a touch and a cut, and a hairline dip is no cut.
fn runs(loops: &[ProfileLoop], axis: &Axis) -> Result<Vec<Run>, BrepError> {
    let mut stops: Vec<(f64, Vec2)> = Vec::new();
    for l in loops {
        stops.extend(loop_stops(l, axis)?);
    }
    stops.sort_by(|a, b| a.0.total_cmp(&b.0));
    stops.dedup_by(|a, b| a.1 == b.1);
    let mut states: Vec<State> = stops
        .windows(2)
        .map(|w| {
            let mid = Vec2::new(0.5 * (w[0].1.x + w[1].1.x), 0.5 * (w[0].1.y + w[1].1.y));
            if on_boundary(loops, mid) {
                State::OnEdge
            } else if material(loops, mid) {
                State::Inside
            } else {
                State::Outside
            }
        })
        .collect();
    // A window on the boundary is material only with material on both sides of it.
    for k in 0..states.len() {
        if states[k] != State::OnEdge {
            continue;
        }
        let before = states[..k].iter().rev().find(|s| **s != State::OnEdge);
        let after = states[k + 1..].iter().find(|s| **s != State::OnEdge);
        states[k] = if before == Some(&State::Inside) && after == Some(&State::Inside) { State::Inside } else { State::Outside };
    }
    let mut runs: Vec<Run> = Vec::new();
    let mut was_inside = false;
    for (k, w) in stops.windows(2).enumerate() {
        if states[k] == State::Outside {
            was_inside = false;
            continue;
        }
        match runs.last_mut() {
            Some(run) if was_inside => {
                run.t[1] = w[1].0;
                run.ends[1] = w[1].1;
            }
            _ => runs.push(Run { ends: [w[0].1, w[1].1], t: [w[0].0, w[1].0] }),
        }
        was_inside = true;
    }
    Ok(runs)
}

/// Which side of its ends a window of the line lies on: material, none, or the boundary itself.
#[derive(Clone, Copy, PartialEq)]
enum State {
    Inside,
    Outside,
    /// A window with no length of its own: the middle lies on an edge of the face.
    OnEdge,
}

/// Whether `p`, a point of the line, lies on an edge of the face's loops: a contact is `Zero`.
fn on_boundary(loops: &[ProfileLoop], p: Vec2) -> bool {
    loops.iter().any(|l| {
        let n = l.points.len();
        l.points.iter().enumerate().any(|(k, &a)| {
            let b = l.points[(k + 1) % n];
            match l.arc(k) {
                None => orient2d(a, b, p) == Sign::Zero && p.x >= a.x.min(b.x) && p.x <= a.x.max(b.x) && p.y >= a.y.min(b.y) && p.y <= a.y.max(b.y),
                Some(arc) => {
                    let profile = ProfileArc { center: arc.center, ccw: arc.ccw };
                    p == a || p == b || (inside_circle(arc.center, a, p) == Sign::Zero && profile.turn(a, p).abs() <= profile.turn(a, b).abs())
                }
            }
        })
    })
}

/// Whether `p` lies in the material of the face: inside its first loop and outside the others.
fn material(loops: &[ProfileLoop], p: Vec2) -> bool {
    let Some((outer, holes)) = loops.split_first() else { return false };
    outer.winding(p) != 0 && holes.iter().all(|h| h.winding(p) == 0)
}

/// The places the line crosses the loop's boundary: inside an edge, and at a vertex with the
/// loop on both sides of the line there.
fn loop_stops(l: &ProfileLoop, axis: &Axis) -> Result<Vec<(f64, Vec2)>, BrepError> {
    let n = l.points.len();
    let mut out = Vec::new();
    for (k, &p) in l.points.iter().enumerate() {
        let q = l.points[(k + 1) % n];
        match l.arc(k) {
            None => {
                let (sp, sq) = (axis.side(p), axis.side(q));
                if sp != Sign::Zero
                    && sq != Sign::Zero
                    && sp != sq
                    && let Some(&x) = crossings(Carrier::Line(axis.a, axis.b), Carrier::Line(p, q)).first()
                {
                    out.push((axis.at(x), x));
                }
            }
            Some(arc) => {
                let r = libm::hypot(p.x - arc.center.x, p.y - arc.center.y);
                let profile = ProfileArc { center: arc.center, ccw: arc.ccw };
                let total = profile.turn(p, q).abs();
                for x in crossings(Carrier::Line(axis.a, axis.b), Carrier::Circle(arc.center, r)) {
                    // An end of the arc is a vertex of the loop: the pass below counts it.
                    if x == p || x == q {
                        continue;
                    }
                    let at = profile.turn(p, x).abs();
                    if at > 0.0 && at < total {
                        out.push((axis.at(x), x));
                    }
                }
            }
        }
    }
    for (k, &v) in l.points.iter().enumerate() {
        if axis.side(v) != Sign::Zero {
            continue;
        }
        let before = local_side(l, (k + n - 1) % n, axis, true)?;
        let after = local_side(l, k, axis, false)?;
        if before != Sign::Zero && after != Sign::Zero && before != after {
            out.push((axis.at(v), v));
        }
    }
    Ok(out)
}

/// Which side of the line the loop's edge `k` lies on just next to its end (or its start): the
/// side of the edge's points nearest that vertex.
fn local_side(l: &ProfileLoop, k: usize, axis: &Axis, at_end: bool) -> Result<Sign, BrepError> {
    let n = l.points.len();
    let (p, q) = (l.points[k], l.points[(k + 1) % n]);
    match l.arc(k) {
        None => {
            let other = if at_end { p } else { q };
            match axis.side(other) {
                // Both ends on the line: the edge runs along it.
                Sign::Zero => Err(BrepError::Degenerate("bend: the line runs along an edge of the face".into())),
                s => Ok(s),
            }
        }
        Some(arc) => {
            let v = if at_end { q } else { p };
            let d = Vec2::new(v.x - arc.center.x, v.y - arc.center.y);
            let cos = d.x * axis.along.x + d.y * axis.along.y;
            // The edge leaves the line at the rate `ccw · cos φ`; just before its end it comes
            // from the other side. A line the circle only touches at the vertex leaves it at no
            // rate at all: the circle then lies wholly on its centre's side, at either end.
            match Sign::of(if arc.ccw { cos } else { -cos }) {
                Sign::Zero => match axis.side(arc.center) {
                    // A circle touching the line at a vertex whose centre is on the line too has
                    // no radius: a body the build refuses.
                    Sign::Zero => Err(BrepError::Degenerate("bend: an arc of the face has no side at a vertex on the line".into())),
                    s => Ok(s),
                },
                s if at_end => Ok(-s),
                s => Ok(s),
            }
        }
    }
}

/// The area of the material on the left and on the right of the line: the face's first loop
/// less its holes, each cut by the line into its two parts.
fn material_sides(loops: &[ProfileLoop], axis: &Axis) -> [f64; 2] {
    let mut sides = [0.0; 2];
    for (i, l) in loops.iter().enumerate() {
        let [left, right] = loop_sides(l, axis).map(f64::abs);
        let outer = if i == 0 { 1.0 } else { -1.0 };
        sides[0] += outer * left;
        sides[1] += outer * right;
    }
    sides
}

/// The signed areas of the parts of `l`'s region left and right of the line: chords and circular
/// segments, negative when the loop runs clockwise.
fn loop_sides(l: &ProfileLoop, axis: &Axis) -> [f64; 2] {
    let mut sides = [0.0; 2];
    let n = l.points.len();
    for (k, &p) in l.points.iter().enumerate() {
        let q = l.points[(k + 1) % n];
        match l.arc(k) {
            None => {
                let (sp, sq) = (axis.side(p), axis.side(q));
                if sp == Sign::Zero && sq == Sign::Zero {
                    continue;
                }
                if sp != Sign::Zero && sq != Sign::Zero && sp != sq {
                    if let Some(&x) = crossings(Carrier::Line(axis.a, axis.b), Carrier::Line(p, q)).first() {
                        add(&mut sides, straight(p, x, axis));
                        add(&mut sides, straight(x, q, axis));
                    }
                } else {
                    add(&mut sides, straight(p, q, axis));
                }
            }
            Some(arc) => {
                let profile = ProfileArc { center: arc.center, ccw: arc.ccw };
                let r = libm::hypot(p.x - arc.center.x, p.y - arc.center.y);
                let total = profile.turn(p, q).abs();
                let mut at = vec![0.0, total];
                for x in crossings(Carrier::Line(axis.a, axis.b), Carrier::Circle(arc.center, r)) {
                    if x == p || x == q {
                        continue;
                    }
                    let s = profile.turn(p, x).abs();
                    if s > 0.0 && s < total {
                        at.push(s);
                    }
                }
                at.sort_by(f64::total_cmp);
                at.dedup();
                let c = flat(axis, arc.center);
                let fp = flat(axis, p);
                let start = libm::atan2(fp.y - c.y, fp.x - c.x);
                let dir = if arc.ccw { 1.0 } else { -1.0 };
                let at_point = |s: f64| {
                    if s == 0.0 {
                        p
                    } else if s == total {
                        q
                    } else {
                        let (sn, cs) = libm::sincos(start + dir * s);
                        unflat(axis, Vec2::new(c.x + r * cs, c.y + r * sn))
                    }
                };
                for w in at.windows(2) {
                    if w[0] == w[1] {
                        continue;
                    }
                    add(&mut sides, curved(at_point(w[0]), at_point(w[1]), arc, axis));
                }
            }
        }
    }
    sides
}

/// The signed area under a straight piece and which side of the line it lies on.
fn straight(p: Vec2, q: Vec2, axis: &Axis) -> (Sign, f64) {
    let (fp, fq) = (flat(axis, p), flat(axis, q));
    let m = Vec2::new(0.5 * (p.x + q.x), 0.5 * (p.y + q.y));
    sided(axis.side(m), 0.5 * (fp.x * fq.y - fp.y * fq.x))
}

/// The signed area under a piece of the arc around `arc`'s centre from `p` to `q`, and which
/// side of the line it lies on.
fn curved(p: Vec2, q: Vec2, arc: ProfileArc, axis: &Axis) -> (Sign, f64) {
    let r = libm::hypot(p.x - arc.center.x, p.y - arc.center.y);
    let c = flat(axis, arc.center);
    let fp = flat(axis, p);
    let start = libm::atan2(fp.y - c.y, fp.x - c.x);
    let turn = ProfileArc { center: arc.center, ccw: arc.ccw }.turn(p, q);
    let (s1, c1) = libm::sincos(start);
    let (s2, c2) = libm::sincos(start + turn);
    let area = 0.5 * (r * c.x * (s2 - s1) - r * c.y * (c2 - c1) + r * r * turn);
    let (sm, cm) = libm::sincos(start + 0.5 * turn);
    let at = unflat(axis, Vec2::new(c.x + r * cm, c.y + r * sm));
    sided(axis.side(at), area)
}

/// A piece's area on the side it lies on. A piece on the line has no side and bounds no area:
/// only the hairline piece between two places of one crossing, rounded apart, is ever there.
fn sided(on: Sign, area: f64) -> (Sign, f64) {
    (on, if on == Sign::Zero { 0.0 } else { area })
}

/// Add a piece to the side it lies on.
fn add(sides: &mut [f64; 2], (on, area): (Sign, f64)) {
    match on {
        Sign::Positive => sides[0] += area,
        Sign::Negative => sides[1] += area,
        // On the line is no side, and such a piece bounds no area: only the hairline piece
        // between two places of one crossing, rounded apart, is ever there.
        Sign::Zero => debug_assert_eq!(area, 0.0, "a piece on the line has an area"),
    }
}
