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
        let area: f64 = (0..n)
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
            .sum();
        area > 0.0
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

    /// A box around the loop: its points, and the full circle of every arc.
    pub fn bounds(&self) -> (Vec2, Vec2) {
        let mut lo = Vec2::new(f64::INFINITY, f64::INFINITY);
        let mut hi = Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY);
        let mut add = |p: Vec2, r: f64| {
            lo = Vec2::new(lo.x.min(p.x - r), lo.y.min(p.y - r));
            hi = Vec2::new(hi.x.max(p.x + r), hi.y.max(p.y + r));
        };
        for (k, &p) in self.points.iter().enumerate() {
            add(p, 0.0);
            if let Some(arc) = self.arc(k) {
                add(arc.center, (p.x - arc.center.x).hypot(p.y - arc.center.y));
            }
        }
        (lo, hi)
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
