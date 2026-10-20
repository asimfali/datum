//! Exact predicates on circular arcs in the plane (M5, D63): whether a segment or another arc
//! touches an arc, and whether a point lies inside a circle.
//!
//! An arc is given by explicit points only: its centre, where it starts and the direction where
//! it ends. Its radius is the distance from the centre to the start, so `r²` is a polynomial in
//! the inputs and no square root is ever taken. Where a segment or a circle meets the circle,
//! the meeting points are `α + β·√Δ` with polynomial `α, β, Δ`; the sign of such a number follows
//! from the signs of `α`, `β` and `α² − β²·Δ`, each an exact predicate.

use crate::exact::{Expr, Lift, Ring, sign};
use crate::{Sign, Vec2, Vec3, orient2d};

/// A circular arc, counterclockwise around `center` from `from` to the direction of `to`. Its
/// radius is the distance from `center` to `from`; `to` need not lie on the circle exactly.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Arc2 {
    pub center: Vec2,
    pub from: Vec2,
    pub to: Vec2,
}

/// Exact position of `p` relative to the circle around `center` through `through`: `Positive`
/// inside, `Zero` on it.
pub fn inside_circle(center: Vec2, through: Vec2, p: Vec2) -> Sign {
    let mut e = Poly::default();
    let [c, t, p] = [center, through, p].map(|v| e.point(v));
    let (tc, pc) = (e.sub2(t, c), e.sub2(p, c));
    let (r2, d2) = (e.dot2(tc, tc), e.dot2(pc, pc));
    let v = e.sub(r2, d2);
    e.sign(v)
}

/// Whether the segment `p → q` and the arc have a point in common, ends and tangency included.
pub fn segment_touches_arc(p: Vec2, q: Vec2, arc: &Arc2) -> bool {
    let mut e = Poly::default();
    let [c, f, g, p, q] = [arc.center, arc.from, arc.to, p, q].map(|v| e.point(v));
    // Points of the segment are `p + t·d`; on the circle `A·t² + B·t + C = 0`.
    let (d, pc, fc) = (e.sub2(q, p), e.sub2(p, c), e.sub2(f, c));
    let a = e.dot2(d, d);
    let half_b = e.dot2(d, pc);
    let b = e.add(half_b, half_b);
    let (pc2, r2) = (e.dot2(pc, pc), e.dot2(fc, fc));
    let cc = e.sub(pc2, r2);
    let (bb, ac) = (e.mul(b, b), e.mul(a, cc));
    let ac2 = e.add(ac, ac);
    let ac4 = e.add(ac2, ac2);
    let disc = e.sub(bb, ac4);
    let tangent = match e.sign(disc) {
        Sign::Negative => return false,
        Sign::Zero => true,
        Sign::Positive => false,
    };
    let span = Span::of(&mut e, arc);
    let neg_b = e.neg(b);
    let two_a = e.add(a, a);
    let two_a_b = e.add(two_a, b);
    let ends = [fc, e.sub2(g, c)];
    for s in if tangent { &[1.0][..] } else { &[1.0, -1.0][..] } {
        // t = (−B + s·√Δ) / 2A, and 0 ≤ t ≤ 1.
        let (plus, minus) = (e.var(*s), e.var(-*s));
        if e.root_sign(neg_b, plus, disc) == Sign::Negative || e.root_sign(two_a_b, minus, disc) == Sign::Negative {
            continue;
        }
        // 2A·cross(u − c, x − c) = 2A·cross(u − c, p − c) − B·cross(u − c, d) + s·√Δ·cross(u − c, d).
        let side = ends.map(|u| {
            let (up, ud) = (e.cross2(u, pc), e.cross2(u, d));
            let (x, y) = (e.mul(two_a, up), e.mul(b, ud));
            let alpha = e.sub(x, y);
            let beta = e.mul(plus, ud);
            e.root_sign(alpha, beta, disc)
        });
        if span.contains(side) {
            return true;
        }
    }
    false
}

/// Whether two arcs have a point in common, ends and tangency included.
pub fn arcs_touch(a: &Arc2, b: &Arc2) -> bool {
    let mut e = Poly::default();
    let [c1, f1, g1, c2, f2, g2] = [a.center, a.from, a.to, b.center, b.from, b.to].map(|v| e.point(v));
    let (f1c, f2c) = (e.sub2(f1, c1), e.sub2(f2, c2));
    let (r1, r2) = (e.dot2(f1c, f1c), e.dot2(f2c, f2c));
    let ec = e.sub2(c2, c1);
    let l = e.dot2(ec, ec);
    let (span1, span2) = (Span::of(&mut e, a), Span::of(&mut e, b));
    if e.sign(l) == Sign::Zero {
        // One centre: one circle if the radii agree, and then the arcs touch where one holds an
        // end of the other.
        let dr = e.sub(r1, r2);
        if e.sign(dr) != Sign::Zero {
            return false;
        }
        let on = |arc: &Arc2, x: Vec2| [arc.from, arc.to].map(|u| orient2d(arc.center, u, x));
        return [b.from, b.to].into_iter().any(|x| span1.contains(on(a, x))) || [a.from, a.to].into_iter().any(|x| span2.contains(on(b, x)));
    }
    // With `L = |c2 − c1|²`, the circles meet at `2L·(x − c1) = K·e ± √H·e⊥`: `K = L + r1² − r2²`,
    // `H = 4L·r1² − K²`, `e = c2 − c1`, `e⊥` it turned a quarter.
    let lr = e.sub(l, r2);
    let k = e.add(lr, r1);
    let (lr1, kk) = (e.mul(l, r1), e.mul(k, k));
    let lr2 = e.add(lr1, lr1);
    let lr4 = e.add(lr2, lr2);
    let h = e.sub(lr4, kk);
    let tangent = match e.sign(h) {
        Sign::Negative => return false,
        Sign::Zero => true,
        Sign::Positive => false,
    };
    let perp = [e.neg(ec[1]), ec[0]];
    let two_l = e.add(l, l);
    // On circle 2: 2L·(x − c2) = (K − 2L)·e ± √H·e⊥.
    let k2 = e.sub(k, two_l);
    let ends1 = [f1c, e.sub2(g1, c1)];
    let ends2 = [f2c, e.sub2(g2, c2)];
    for s in if tangent { &[1.0][..] } else { &[1.0, -1.0][..] } {
        let plus = e.var(*s);
        let mut side = |kk: H, u: [H; 2]| {
            let (ue, up) = (e.cross2(u, ec), e.cross2(u, perp));
            let alpha = e.mul(kk, ue);
            let beta = e.mul(plus, up);
            e.root_sign(alpha, beta, h)
        };
        let on1 = ends1.map(|u| side(k, u));
        let on2 = ends2.map(|u| side(k2, u));
        if span1.contains(on1) && span2.contains(on2) {
            return true;
        }
    }
    false
}

/// Which points of the circle an arc covers, from the signs `orient2d(c, from, x)` and
/// `orient2d(c, to, x)` of a point `x` on the circle.
#[derive(Clone, Copy)]
struct Span {
    /// `orient2d(c, from, to)`: the arc turns less than half a circle when positive.
    turn: Sign,
    /// `to` points away from `from` (a half circle when `turn` is zero).
    opposite: bool,
}

impl Span {
    fn of(e: &mut Poly, arc: &Arc2) -> Self {
        let [c, f, g] = [arc.center, arc.from, arc.to].map(|v| e.point(v));
        let (fc, gc) = (e.sub2(f, c), e.sub2(g, c));
        let d = e.dot2(fc, gc);
        Self { turn: orient2d(arc.center, arc.from, arc.to), opposite: e.sign(d) == Sign::Negative }
    }

    /// Whether a point of the circle with orientations `[from, to]` lies on the closed arc.
    fn contains(self, [o1, o2]: [Sign; 2]) -> bool {
        match self.turn {
            Sign::Positive => o1 != Sign::Negative && o2 != Sign::Positive,
            Sign::Negative => o1 != Sign::Negative || o2 != Sign::Positive,
            // Half a circle: the side left of `from`; or the whole circle when `to` is `from`.
            Sign::Zero => !self.opposite || o1 != Sign::Negative,
        }
    }
}

/// A node of a polynomial: an input or an operation on earlier nodes.
#[derive(Clone, Copy)]
enum Node {
    Input(usize),
    Add(usize, usize),
    Sub(usize, usize),
    Mul(usize, usize),
}

/// A handle to a node of a [`Poly`].
#[derive(Clone, Copy)]
struct H(usize);

/// Polynomials in `f64` inputs, built node by node; the sign of any node is an exact predicate.
/// Arc predicates are cold (sketch and profile checks), so a small expression graph serves them
/// all instead of one hand-written expression per sign.
#[derive(Default)]
struct Poly {
    inputs: Vec<f64>,
    nodes: Vec<Node>,
}

impl Poly {
    fn push(&mut self, n: Node) -> H {
        self.nodes.push(n);
        H(self.nodes.len() - 1)
    }

    fn var(&mut self, v: f64) -> H {
        self.inputs.push(v);
        self.push(Node::Input(self.inputs.len() - 1))
    }

    fn point(&mut self, p: Vec2) -> [H; 2] {
        [self.var(p.x), self.var(p.y)]
    }

    fn add(&mut self, a: H, b: H) -> H {
        self.push(Node::Add(a.0, b.0))
    }

    fn sub(&mut self, a: H, b: H) -> H {
        self.push(Node::Sub(a.0, b.0))
    }

    fn mul(&mut self, a: H, b: H) -> H {
        self.push(Node::Mul(a.0, b.0))
    }

    fn neg(&mut self, a: H) -> H {
        let zero = self.var(0.0);
        self.sub(zero, a)
    }

    fn sub2(&mut self, a: [H; 2], b: [H; 2]) -> [H; 2] {
        [self.sub(a[0], b[0]), self.sub(a[1], b[1])]
    }

    fn dot2(&mut self, a: [H; 2], b: [H; 2]) -> H {
        let (x, y) = (self.mul(a[0], b[0]), self.mul(a[1], b[1]));
        self.add(x, y)
    }

    fn cross2(&mut self, a: [H; 2], b: [H; 2]) -> H {
        let (x, y) = (self.mul(a[0], b[1]), self.mul(a[1], b[0]));
        self.sub(x, y)
    }

    fn sign(&self, root: H) -> Sign {
        sign(&Root(self, root))
    }

    /// Sign of `α + β·√Δ`, `Δ ≥ 0`.
    fn root_sign(&mut self, alpha: H, beta: H, delta: H) -> Sign {
        let (sa, sb) = (self.sign(alpha), self.sign(beta));
        if sb == Sign::Zero || self.sign(delta) == Sign::Zero {
            return sa;
        }
        if sa == Sign::Zero || sa == sb {
            return sb;
        }
        // Opposite signs: the larger of |α| and |β|·√Δ wins, by α² − β²·Δ.
        let (aa, bb) = (self.mul(alpha, alpha), self.mul(beta, beta));
        let bbd = self.mul(bb, delta);
        let diff = self.sub(aa, bbd);
        match self.sign(diff) {
            Sign::Positive => sa,
            Sign::Negative => sb,
            Sign::Zero => Sign::Zero,
        }
    }
}

/// One node of a [`Poly`] as an expression.
struct Root<'a>(&'a Poly, H);

impl Expr for Root<'_> {
    fn inputs(&self) -> impl Iterator<Item = f64> {
        self.0.inputs.iter().copied()
    }

    fn moves(&self) -> bool {
        false
    }

    fn eval<T: Ring>(&self, lift: &impl Lift<T>) -> T {
        let mut v: Vec<T> = Vec::with_capacity(self.1.0 + 1);
        for n in &self.0.nodes[..=self.1.0] {
            let x = match *n {
                Node::Input(i) => {
                    let [x, _, _] = lift.point(Vec3::new(self.0.inputs[i], 0.0, 0.0), None);
                    x
                }
                Node::Add(a, b) => v[a].clone() + v[b].clone(),
                Node::Sub(a, b) => v[a].clone() - v[b].clone(),
                Node::Mul(a, b) => v[a].clone() * v[b].clone(),
            };
            v.push(x);
        }
        v.swap_remove(self.1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn v(x: f64, y: f64) -> Vec2 {
        Vec2::new(x, y)
    }

    /// The upper half of the unit circle, from (1, 0) to (−1, 0).
    fn upper() -> Arc2 {
        Arc2 { center: v(0.0, 0.0), from: v(1.0, 0.0), to: v(-1.0, 0.0) }
    }

    #[test]
    fn points_and_circles() {
        let (c, t) = (v(0.0, 0.0), v(3.0, 4.0));
        assert_eq!(inside_circle(c, t, v(4.9, 0.0)), Sign::Positive);
        assert_eq!(inside_circle(c, t, v(0.0, -5.0)), Sign::Zero);
        assert_eq!(inside_circle(c, t, v(5.0, 0.1)), Sign::Negative);
    }

    #[test]
    fn segments_and_arcs() {
        let a = upper();
        // Through the arc, through the missing lower half, tangent at the top, tangent below.
        assert!(segment_touches_arc(v(0.0, 0.0), v(0.0, 2.0), &a));
        assert!(!segment_touches_arc(v(0.0, 0.0), v(0.0, -2.0), &a));
        assert!(segment_touches_arc(v(-1.0, 1.0), v(1.0, 1.0), &a));
        assert!(!segment_touches_arc(v(-1.0, -1.0), v(1.0, -1.0), &a));
        // Short of the circle, and ending exactly on its end point.
        assert!(!segment_touches_arc(v(0.0, 0.0), v(0.0, 0.999_999_999), &a));
        assert!(segment_touches_arc(v(2.0, 0.0), v(1.0, 0.0), &a));
        // Inside the circle all along.
        assert!(!segment_touches_arc(v(-0.5, 0.1), v(0.5, 0.1), &a));
        // A chord that leaves both ends outside but crosses the lower half twice.
        assert!(!segment_touches_arc(v(-2.0, -0.5), v(2.0, -0.5), &a));
        assert!(segment_touches_arc(v(-2.0, 0.5), v(2.0, 0.5), &a));
    }

    #[test]
    fn arcs_and_arcs() {
        let a = upper();
        // A slot's two ends face away from each other: their circles touch, the arcs do not.
        let left = Arc2 { center: v(0.0, 0.0), from: v(0.0, 1.0), to: v(0.0, -1.0) };
        let right = Arc2 { center: v(2.0, 0.0), from: v(2.0, -1.0), to: v(2.0, 1.0) };
        assert!(!arcs_touch(&left, &right));
        // The same circles, the arcs turned towards each other: they meet at (1, 0).
        let left_in = Arc2 { center: v(0.0, 0.0), from: v(0.0, -1.0), to: v(0.0, 1.0) };
        let right_in = Arc2 { center: v(2.0, 0.0), from: v(2.0, 1.0), to: v(2.0, -1.0) };
        assert!(arcs_touch(&left_in, &right_in));
        // Concentric: never, whatever the spans.
        let inner = Arc2 { center: v(0.0, 0.0), from: v(0.5, 0.0), to: v(-0.5, 0.0) };
        assert!(!arcs_touch(&a, &inner));
        // One circle: the lower half touches the upper one at both ends.
        let lower = Arc2 { center: v(0.0, 0.0), from: v(-1.0, 0.0), to: v(1.0, 0.0) };
        assert!(arcs_touch(&a, &lower));
        let lower_part = Arc2 { center: v(0.0, 0.0), from: v(-0.6, -0.8), to: v(0.6, -0.8) };
        assert!(!arcs_touch(&a, &lower_part));
        // Crossing circles, crossing points on both arcs or not.
        let b = Arc2 { center: v(1.0, 0.5), from: v(1.0, -0.5), to: v(1.0, 1.5) };
        assert!(!arcs_touch(&a, &b), "b covers its right half only, away from the unit circle");
        let b_left = Arc2 { center: v(1.0, 0.5), from: v(1.0, 1.5), to: v(1.0, -0.5) };
        assert!(arcs_touch(&a, &b_left));
    }

    /// Far from the origin, the same configurations: principle 8.
    #[test]
    fn far_away() {
        let o = v(1e7, -3e6);
        let at = |p: Vec2| v(p.x + o.x, p.y + o.y);
        let a = Arc2 { center: at(v(0.0, 0.0)), from: at(v(1.0, 0.0)), to: at(v(-1.0, 0.0)) };
        assert!(segment_touches_arc(at(v(-1.0, 1.0)), at(v(1.0, 1.0)), &a));
        assert!(!segment_touches_arc(at(v(-1.0, -1.0)), at(v(1.0, -1.0)), &a));
    }
}
