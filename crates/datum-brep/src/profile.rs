//! Closed profiles in a plane: loops of segments and arcs with the IDs they give a body, and
//! exact 2D checks on them (do two loops touch, how often does a loop wind around a point).

use datum_math::{Arc2, Sign, Vec2, arcs_touch, inside_circle, orient2d, segment_touches_arc};

use crate::TopoId;
use crate::tess::segments_touch;

/// A profile edge that is an arc around `center`: counterclockwise (seen from +Z) when `ccw`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ProfileArc {
    pub center: Vec2,
    pub ccw: bool,
}

impl ProfileArc {
    /// The angle the arc turns around its centre from `a` to `b`: in `(0, 2π]` counterclockwise,
    /// in `[−2π, 0)` clockwise. The same point twice is a full circle.
    pub fn turn(&self, a: Vec2, b: Vec2) -> f64 {
        let (u, v) = (Vec2::new(a.x - self.center.x, a.y - self.center.y), Vec2::new(b.x - self.center.x, b.y - self.center.y));
        let t = libm::atan2(u.x * v.y - u.y * v.x, u.x * v.x + u.y * v.y);
        match (self.ccw, t) {
            (true, t) if t <= 0.0 => t + std::f64::consts::TAU,
            (false, t) if t >= 0.0 => t - std::f64::consts::TAU,
            (_, t) => t,
        }
    }
}

/// A closed loop of a region and the IDs its extrusion (or cut) gives: for each point the
/// vertices at the start and at the end, for each edge from point `k` to `k + 1` the side face,
/// and the edge's arc if it is one (`arcs` empty: all straight). A loop only looked at, such as
/// a face's outline, has no IDs.
#[derive(Clone, Debug)]
pub struct ProfileLoop {
    pub points: Vec<Vec2>,
    pub vertices: Vec<[TopoId; 2]>,
    pub sides: Vec<TopoId>,
    pub arcs: Vec<Option<ProfileArc>>,
}

impl ProfileLoop {
    /// The same loop the other way round, each edge keeping its side ID and arc.
    pub fn reversed(&self) -> Self {
        let n = self.points.len();
        let mut r = self.clone();
        r.points.reverse();
        r.vertices.reverse();
        // New edge k runs from old point n−1−k to n−2−k: old edge n−2−k.
        let old = |k: usize| (2 * n - 2 - k) % n;
        r.sides = (0..n).map(|k| self.sides[old(k)]).collect();
        r.arcs = (0..self.arcs.len()).map(|k| self.arcs[old(k)].map(|a| ProfileArc { ccw: !a.ccw, ..a })).collect();
        r
    }

    pub fn arc(&self, k: usize) -> Option<ProfileArc> {
        self.arcs.get(k).copied().flatten()
    }

    /// Whether the loop runs counterclockwise. A polygon: the turn at its lowest point, exact. With
    /// arcs: the sign of the area, chords and circular segments, in `f64`. The loop was found
    /// simple by exact predicates before, and a simple loop's area is far from zero: rounding
    /// cannot flip its sign.
    pub fn counterclockwise(&self) -> bool {
        let p = &self.points;
        let n = p.len();
        if self.arcs.iter().all(Option::is_none) {
            let i = (0..n).min_by(|&a, &b| p[a].y.total_cmp(&p[b].y).then(p[a].x.total_cmp(&p[b].x))).unwrap_or(0);
            return orient2d(p[(i + n - 1) % n], p[i], p[(i + 1) % n]) == Sign::Positive;
        }
        self.area() > 0.0
    }

    /// The area the loop bounds, positive counterclockwise: chords and circular segments, in
    /// `f64` and in a fixed order.
    pub fn area(&self) -> f64 {
        let p = &self.points;
        let n = p.len();
        (0..n)
            .map(|k| {
                let (a, b) = (p[k], p[(k + 1) % n]);
                let chord = 0.5 * (a.x * b.y - a.y * b.x);
                chord
                    + self.arc(k).map_or(0.0, |arc| {
                        let u = Vec2::new(a.x - arc.center.x, a.y - arc.center.y);
                        let phi = arc.turn(a, b).abs();
                        let r2 = u.x * u.x + u.y * u.y;
                        0.5 * r2 * (phi - libm::sin(phi)) * if arc.ccw { 1.0 } else { -1.0 }
                    })
            })
            .sum()
    }

    /// The length of the loop's boundary: chords and arcs, in `f64` and in a fixed order.
    pub fn perimeter(&self) -> f64 {
        let p = &self.points;
        let n = p.len();
        (0..n)
            .map(|k| {
                let (a, b) = (p[k], p[(k + 1) % n]);
                match self.arc(k) {
                    None => libm::hypot(b.x - a.x, b.y - a.y),
                    Some(arc) => libm::hypot(a.x - arc.center.x, a.y - arc.center.y) * arc.turn(a, b).abs(),
                }
            })
            .sum()
    }

    /// Edge `k` for the touch checks.
    pub(crate) fn piece(&self, k: usize) -> Piece {
        let n = self.points.len();
        let (a, b) = (self.points[k], self.points[(k + 1) % n]);
        match self.arc(k) {
            None => Piece::Segment(a, b),
            Some(arc) if arc.ccw => Piece::Arc(Arc2 { center: arc.center, from: a, to: b }),
            Some(arc) => Piece::Arc(Arc2 { center: arc.center, from: b, to: a }),
        }
    }

    /// Whether some edge of this loop and some edge of `other` share a point (exact).
    pub fn touches(&self, other: &ProfileLoop) -> bool {
        let (n, m) = (self.points.len(), other.points.len());
        (0..n).any(|i| {
            let a = self.piece(i);
            (0..m).any(|j| touch(&a, &other.piece(j)))
        })
    }

    /// The winding number of the loop around `p`, exact: its polygon of chords, and for every arc
    /// the circular segment between chord and arc, counted with the arc's turning sense. `p`
    /// must lie on no edge.
    pub fn winding(&self, p: Vec2) -> i32 {
        let poly = &self.points;
        let mut winding = 0i32;
        for k in 0..poly.len() {
            let (a, b) = (poly[k], poly[(k + 1) % poly.len()]);
            if a.y <= p.y && b.y > p.y && orient2d(a, b, p) == Sign::Positive {
                winding += 1;
            } else if a.y > p.y && b.y <= p.y && orient2d(a, b, p) == Sign::Negative {
                winding -= 1;
            }
            if let Some(arc) = self.arc(k) {
                // Counterclockwise from `a` to `b` the segment lies right of the chord, clockwise left.
                let beyond = if arc.ccw { Sign::Negative } else { Sign::Positive };
                if orient2d(a, b, p) == beyond && inside_circle(arc.center, a, p) == Sign::Positive {
                    winding += if arc.ccw { 1 } else { -1 };
                }
            }
        }
        winding
    }

    /// A box around the loop: its points, and the full circle of every arc, rounded outwards (a
    /// box is a filter before the exact checks: it may hold more, never less).
    pub fn bounds(&self) -> (Vec2, Vec2) {
        let mut lo = Vec2::new(f64::INFINITY, f64::INFINITY);
        let mut hi = Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY);
        let mut add = |a: Vec2, b: Vec2| {
            lo = Vec2::new(lo.x.min(a.x), lo.y.min(a.y));
            hi = Vec2::new(hi.x.max(b.x), hi.y.max(b.y));
        };
        for (k, &p) in self.points.iter().enumerate() {
            add(p, p);
            if let Some(arc) = self.arc(k) {
                let (c, r) = (arc.center, libm::hypot(p.x - arc.center.x, p.y - arc.center.y).next_up());
                add(Vec2::new((c.x - r).next_down(), (c.y - r).next_down()), Vec2::new((c.x + r).next_up(), (c.y + r).next_up()));
            }
        }
        (lo, hi)
    }
}

/// A 2D box `(min, max)`.
pub type Box2 = (Vec2, Vec2);

/// The pairs `(a, b)`, `a < b`, of `boxes` that overlap (closed: boxes that touch do), sorted: a
/// sweep along X. The filter before exact checks of what may touch.
pub fn overlapping(boxes: &[Box2]) -> Vec<(usize, usize)> {
    let mut order: Vec<usize> = (0..boxes.len()).collect();
    order.sort_unstable_by(|&a, &b| boxes[a].0.x.total_cmp(&boxes[b].0.x).then(a.cmp(&b)));
    let mut active: Vec<usize> = Vec::new();
    let mut pairs = Vec::new();
    for &i in &order {
        active.retain(|&a| boxes[a].1.x >= boxes[i].0.x);
        pairs.extend(active.iter().filter(|&&a| overlap(boxes[a], boxes[i])).map(|&a| (a.min(i), a.max(i))));
        active.push(i);
    }
    pairs.sort_unstable();
    pairs
}

/// Whether two closed boxes share a point.
pub fn overlap(a: Box2, b: Box2) -> bool {
    a.0.x <= b.1.x && b.0.x <= a.1.x && a.0.y <= b.1.y && b.0.y <= a.1.y
}

impl ProfileLoop {
    /// Edges `i` and `k` of this loop and of `other` share a point, past the point two neighbours
    /// of one loop share. Of the same loop when `other` is `None`.
    pub(crate) fn edges_touch(&self, other: Option<&ProfileLoop>) -> bool {
        let n = self.points.len();
        match other {
            None => (0..n).any(|i| (i + 1..n).any(|k| (i + 1) % n != k && (k + 1) % n != i && touch(&self.piece(i), &self.piece(k)))),
            Some(o) => self.touches(o),
        }
    }
}

/// An edge of a profile for the touch checks: a segment, or an arc counterclockwise.
pub(crate) enum Piece {
    Segment(Vec2, Vec2),
    Arc(Arc2),
}

pub(crate) fn touch(a: &Piece, b: &Piece) -> bool {
    match (a, b) {
        (Piece::Segment(p, q), Piece::Segment(r, s)) => segments_touch(*p, *q, *r, *s),
        (Piece::Segment(p, q), Piece::Arc(c)) | (Piece::Arc(c), Piece::Segment(p, q)) => segment_touches_arc(*p, *q, c),
        (Piece::Arc(c), Piece::Arc(d)) => arcs_touch(c, d),
    }
}
