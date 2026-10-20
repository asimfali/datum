//! Constraints as equations with exact derivatives, grouped into independent systems.
//!
//! The equations live in normalised coordinates `(p − centre) / L`, `L` the size of the sketch
//! as drawn (D55): one tolerance fits a bolt and a site plan alike. Results go back as offsets
//! from the drawn coordinates, so a point the solver does not move keeps its coordinates bit for
//! bit.

use datum_math::Vec2;
use rayon::prelude::*;

use crate::solver::{self, Fit};
use datum_math::expr::Num;

use crate::{Constraint, Sketch, SketchError};

/// A constraint with names resolved: point and line indices, values normalised.
#[derive(Clone, Copy)]
enum Eq {
    Coincident(usize, usize),
    Horizontal(usize),
    Vertical(usize),
    Parallel(usize, usize),
    Perpendicular(usize, usize),
    Collinear(usize, usize),
    Equal(usize, usize),
    /// No equation: the point is not a variable.
    Fix,
    Distance(usize, usize, f64),
    Length(usize, f64),
    /// Point, line, distance, and the side of the line the point was drawn on.
    PointLine(usize, usize, f64, f64),
    Angle(usize, usize, f64),
    /// An arc's centre and ends: the ends at one distance.
    ArcEnds(usize, usize, usize),
    /// Centre, a point on the circle, radius.
    Radius(usize, usize, f64),
    /// Line, centre, radius, and the side of the line the centre was drawn on.
    LineCircle(usize, usize, Rad, f64),
    /// Two circles by centre and radius; touching outside, or inside with the sign of `r1 − r2`
    /// as drawn.
    CircleCircle(usize, Rad, usize, Rad, Contact),
    /// A line and an arc meeting at a point: line, centre, the point. The radius there is
    /// across the line. (Distance from the centre would be flat to first order at the point.)
    LineArcAt(usize, usize, usize),
    /// Two arcs meeting at a point: centres and the point, on one straight line.
    ArcArcAt(usize, usize, usize),
}

/// The radius of an arc (the distance from its centre to its start) or of a circle (a value).
#[derive(Clone, Copy)]
enum Rad {
    Point(usize),
    Value(f64),
}

#[derive(Clone, Copy)]
enum Contact {
    Outside,
    Inside(f64),
}

/// A curve a tangency names: a line, or a circle by centre and radius, with the points that
/// end it.
enum Curve {
    Line(usize),
    Circle(usize, Rad, Vec<usize>),
}

/// An independent system: points tied by constraints, and those constraints.
pub(crate) struct Group {
    pub points: Vec<usize>,
    pub constraints: Vec<usize>,
}

pub(crate) struct Equations {
    /// Point names, in name order: point `p` is `names[p]`.
    pub names: Vec<String>,
    /// Coordinates as drawn.
    pub start: Vec<Vec2>,
    /// Line ends.
    lines: Vec<(usize, usize)>,
    eqs: Vec<Eq>,
    /// How messages name each equation: the constraint, or the arc whose ends it ties.
    described: Vec<String>,
    fixed: Vec<bool>,
    /// Points no constraint ties: they keep their coordinates and both degrees of freedom.
    pub unconstrained: Vec<usize>,
    centre: [f64; 2],
    size: f64,
}

/// Convergence and rank thresholds in normalised coordinates (D55).
const TOLERANCE: f64 = 1e-12;
const RANK: f64 = 1e-10;
/// A null-space component above this marks a point that can still move.
const MOVES: f64 = 1e-8;

impl Equations {
    pub fn new(sketch: &Sketch) -> Result<Self, SketchError> {
        let invalid = |m: String| Err(SketchError::Invalid(m));
        let names: Vec<String> = sketch.points.keys().cloned().collect();
        let start: Vec<Vec2> = sketch.points.values().copied().collect();
        if let Some((n, _)) = sketch.points.iter().find(|(_, p)| !(p.x.is_finite() && p.y.is_finite())) {
            return invalid(format!("point `{n}` has non-finite coordinates"));
        }
        let point = |n: &str| names.binary_search_by(|x| x.as_str().cmp(n)).map_err(|_| SketchError::Invalid(format!("unknown point `{n}`")));
        let line_names: Vec<&String> = sketch.lines.keys().collect();
        let mut lines = Vec::with_capacity(sketch.lines.len());
        for (n, l) in &sketch.lines {
            let (a, b) = (point(&l.from)?, point(&l.to)?);
            if a == b || (start[a].x == start[b].x && start[a].y == start[b].y) {
                return invalid(format!("line `{n}` has zero length"));
            }
            lines.push((a, b));
        }
        let line = |n: &str| line_names.binary_search_by(|x| x.as_str().cmp(n)).map_err(|_| SketchError::Invalid(format!("unknown line `{n}`")));
        let mut arcs = Vec::with_capacity(sketch.arcs.len());
        for (n, a) in &sketch.arcs {
            let (c, f, t) = (point(&a.center)?, point(&a.from)?, point(&a.to)?);
            if c == f || c == t || f == t || start[c] == start[f] || start[c] == start[t] {
                return invalid(format!("arc `{n}` needs a centre and two different ends away from it"));
            }
            arcs.push((c, f, t));
        }
        let mut circles = Vec::with_capacity(sketch.circles.len());
        for (n, c) in &sketch.circles {
            circles.push((
                point(&c.center)?,
                c.diameter
                    .value()
                    .filter(|d| d.is_finite() && *d > 0.0)
                    .ok_or_else(|| SketchError::Invalid(format!("circle `{n}` needs a positive diameter")))?,
            ));
        }
        let arc_names: Vec<&String> = sketch.arcs.keys().collect();
        let circle_names: Vec<&String> = sketch.circles.keys().collect();
        let find = |names: &[&String], n: &str| names.binary_search_by(|x| x.as_str().cmp(n)).ok();
        let (lo, hi) = start
            .iter()
            .fold(([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]), |(lo, hi), p| ([lo[0].min(p.x), lo[1].min(p.y)], [hi[0].max(p.x), hi[1].max(p.y)]));
        let centre = if start.is_empty() { [0.0; 2] } else { [(lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5] };
        let diagonal = if start.is_empty() { 0.0 } else { ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2)).sqrt() };
        // The size of the sketch: as drawn, or as dimensioned when everything was drawn at one spot.
        let largest = sketch.constraints.iter().fold(0.0f64, |m, c| match c {
            Constraint::Distance { value, .. }
            | Constraint::Length { value, .. }
            | Constraint::PointLine { value, .. }
            | Constraint::Radius { value, .. } => m.max(value.value().unwrap_or(0.0).abs()),
            _ => m,
        });
        let largest = circles.iter().fold(largest, |m, c| m.max(c.1));
        let size = if diagonal > 0.0 {
            diagonal
        } else if largest > 0.0 {
            largest
        } else {
            1.0
        };
        let mut fixed = vec![false; names.len()];
        let mut eqs = Vec::with_capacity(sketch.constraints.len());
        for (k, c) in sketch.constraints.iter().enumerate() {
            let number = |v: &Num| v.value().ok_or_else(|| SketchError::Invalid(format!("{}: the expression is not evaluated", c.describe(k))));
            let positive = |v: &Num| {
                let v = number(v)?;
                if v.is_finite() && v > 0.0 {
                    Ok(v / size)
                } else {
                    Err(SketchError::Invalid(format!("{}: the value must be positive", c.describe(k))))
                }
            };
            let two = |l: &[String; 2]| -> Result<(usize, usize), SketchError> { Ok((line(&l[0])?, line(&l[1])?)) };
            eqs.push(match c {
                Constraint::Coincident { points } => Eq::Coincident(point(&points[0])?, point(&points[1])?),
                Constraint::Horizontal { line: l } => Eq::Horizontal(line(l)?),
                Constraint::Vertical { line: l } => Eq::Vertical(line(l)?),
                Constraint::Parallel { lines } => {
                    let (a, b) = two(lines)?;
                    Eq::Parallel(a, b)
                }
                Constraint::Perpendicular { lines } => {
                    let (a, b) = two(lines)?;
                    Eq::Perpendicular(a, b)
                }
                Constraint::Collinear { lines } => {
                    let (a, b) = two(lines)?;
                    Eq::Collinear(a, b)
                }
                Constraint::Equal { lines } => {
                    let (a, b) = two(lines)?;
                    Eq::Equal(a, b)
                }
                Constraint::Fix { point: p } => {
                    fixed[point(p)?] = true;
                    Eq::Fix
                }
                Constraint::Distance { points, value } => Eq::Distance(point(&points[0])?, point(&points[1])?, positive(value)?),
                Constraint::Length { line: l, value } => Eq::Length(line(l)?, positive(value)?),
                Constraint::PointLine { point: p, line: l, value } => {
                    let (p, l) = (point(p)?, line(l)?);
                    let (a, b) = lines[l];
                    let (d, q) = ([start[b].x - start[a].x, start[b].y - start[a].y], [start[p].x - start[a].x, start[p].y - start[a].y]);
                    let side = if d[0] * q[1] - d[1] * q[0] < 0.0 { -1.0 } else { 1.0 };
                    Eq::PointLine(p, l, positive(value)?, side)
                }
                Constraint::Angle { lines, degrees } => {
                    let degrees = number(degrees)?;
                    if !degrees.is_finite() {
                        return invalid(format!("{}: the angle must be finite", c.describe(k)));
                    }
                    let (a, b) = two(lines)?;
                    Eq::Angle(a, b, degrees.to_radians())
                }
                Constraint::Radius { arc, value } => {
                    let k = find(&arc_names, arc).ok_or_else(|| SketchError::Invalid(format!("unknown arc `{arc}`")))?;
                    Eq::Radius(arcs[k].0, arcs[k].1, positive(value)?)
                }
                Constraint::Tangent { curves } => {
                    let curve = |n: &str| -> Result<Curve, SketchError> {
                        let found = [
                            line_names.binary_search_by(|x| x.as_str().cmp(n)).ok().map(Curve::Line),
                            find(&arc_names, n).map(|a| Curve::Circle(arcs[a].0, Rad::Point(arcs[a].1), vec![arcs[a].1, arcs[a].2])),
                            find(&circle_names, n).map(|c| Curve::Circle(circles[c].0, Rad::Value(0.5 * circles[c].1 / size), Vec::new())),
                        ];
                        let mut found = found.into_iter().flatten();
                        match (found.next(), found.next()) {
                            (Some(c), None) => Ok(c),
                            (None, _) => Err(SketchError::Invalid(format!("{}: unknown curve `{n}`", c.describe(k)))),
                            (Some(_), Some(_)) => Err(SketchError::Invalid(format!("{}: `{n}` names two curves", c.describe(k)))),
                        }
                    };
                    let drawn = |r: Rad, centre: usize| match r {
                        Rad::Point(p) => (libm::hypot(start[p].x - start[centre].x, start[p].y - start[centre].y)) / size,
                        Rad::Value(v) => v,
                    };
                    match (curve(&curves[0])?, curve(&curves[1])?) {
                        (Curve::Line(_), Curve::Line(_)) => return invalid(format!("{}: two lines cannot touch", c.describe(k))),
                        (Curve::Line(l), Curve::Circle(centre, _, ends)) | (Curve::Circle(centre, _, ends), Curve::Line(l))
                            if ends.iter().any(|e| [lines[l].0, lines[l].1].contains(e)) =>
                        {
                            let at = ends.iter().copied().find(|e| [lines[l].0, lines[l].1].contains(e)).unwrap_or(centre);
                            Eq::LineArcAt(l, centre, at)
                        }
                        (Curve::Circle(c1, _, e1), Curve::Circle(c2, _, e2)) if e1.iter().any(|e| e2.contains(e)) => {
                            let at = e1.iter().copied().find(|e| e2.contains(e)).unwrap_or(c1);
                            Eq::ArcArcAt(c1, c2, at)
                        }
                        (Curve::Line(l), Curve::Circle(centre, r, _)) | (Curve::Circle(centre, r, _), Curve::Line(l)) => {
                            let (a, b) = lines[l];
                            let (d, q) =
                                ([start[b].x - start[a].x, start[b].y - start[a].y], [start[centre].x - start[a].x, start[centre].y - start[a].y]);
                            let side = if d[0] * q[1] - d[1] * q[0] < 0.0 { -1.0 } else { 1.0 };
                            Eq::LineCircle(l, centre, r, side)
                        }
                        (Curve::Circle(c1, r1, _), Curve::Circle(c2, r2, _)) => {
                            // Outside or inside, whichever the drawing is closer to.
                            let dist = libm::hypot(start[c2].x - start[c1].x, start[c2].y - start[c1].y) / size;
                            let (a, b) = (drawn(r1, c1), drawn(r2, c2));
                            let contact = if (dist - (a + b)).abs() <= (dist - (a - b).abs()).abs() {
                                Contact::Outside
                            } else {
                                Contact::Inside((a - b).signum())
                            };
                            Eq::CircleCircle(c1, r1, c2, r2, contact)
                        }
                    }
                }
            });
        }
        let mut described: Vec<String> = sketch.constraints.iter().enumerate().map(|(k, c)| c.describe(k)).collect();
        for ((n, _), &(c, f, t)) in sketch.arcs.iter().zip(&arcs) {
            eqs.push(Eq::ArcEnds(c, f, t));
            described.push(format!("arc `{n}` (its ends at one distance from the centre)"));
        }
        let mut eq = Self { names, start, lines, eqs, described, fixed, unconstrained: Vec::new(), centre, size };
        let tied = eq.tied_points();
        eq.unconstrained = (0..eq.names.len()).filter(|&p| !tied[p] && !eq.fixed[p]).collect();
        Ok(eq)
    }

    /// The points each constraint involves.
    fn involved(&self, k: usize) -> Vec<usize> {
        let l = |i: usize| [self.lines[i].0, self.lines[i].1];
        let r = |c: usize, r: Rad| match r {
            Rad::Point(p) => vec![c, p],
            Rad::Value(_) => vec![c],
        };
        match self.eqs[k] {
            Eq::Coincident(a, b) | Eq::Distance(a, b, _) | Eq::Radius(a, b, _) => vec![a, b],
            Eq::Horizontal(i) | Eq::Vertical(i) | Eq::Length(i, _) => l(i).to_vec(),
            Eq::Parallel(i, j) | Eq::Perpendicular(i, j) | Eq::Collinear(i, j) | Eq::Equal(i, j) | Eq::Angle(i, j, _) => {
                l(i).into_iter().chain(l(j)).collect()
            }
            Eq::PointLine(p, i, _, _) => [p].into_iter().chain(l(i)).collect(),
            Eq::ArcEnds(c, f, t) => vec![c, f, t],
            Eq::LineCircle(i, c, rad, _) => l(i).into_iter().chain(r(c, rad)).collect(),
            Eq::CircleCircle(c1, r1, c2, r2, _) => r(c1, r1).into_iter().chain(r(c2, r2)).collect(),
            Eq::LineArcAt(i, c, p) => l(i).into_iter().chain([c, p]).collect(),
            Eq::ArcArcAt(c1, c2, p) => vec![c1, c2, p],
            Eq::Fix => Vec::new(),
        }
    }

    fn tied_points(&self) -> Vec<bool> {
        let mut tied = vec![false; self.names.len()];
        for k in 0..self.eqs.len() {
            for p in self.involved(k) {
                tied[p] = true;
            }
        }
        tied
    }

    /// Independent systems: points joined by constraints, in order of their lowest point.
    pub fn groups(&self) -> Vec<Group> {
        let n = self.names.len();
        let mut parent: Vec<usize> = (0..n).collect();
        fn root(p: &mut [usize], mut x: usize) -> usize {
            while p[x] != x {
                p[x] = p[p[x]];
                x = p[x];
            }
            x
        }
        // Each constraint belongs to the group of its first point.
        let owner: Vec<Option<usize>> = (0..self.eqs.len()).map(|k| self.involved(k).first().copied()).collect();
        for k in 0..self.eqs.len() {
            let pts = self.involved(k);
            for &q in pts.iter().skip(1) {
                let (a, b) = (root(&mut parent, pts[0]), root(&mut parent, q));
                parent[a.max(b)] = a.min(b);
            }
        }
        let tied = self.tied_points();
        let mut groups: Vec<Group> = Vec::new();
        let mut index = vec![usize::MAX; n];
        for p in (0..n).filter(|&p| tied[p]) {
            let r = root(&mut parent, p);
            if index[r] == usize::MAX {
                index[r] = groups.len();
                groups.push(Group { points: Vec::new(), constraints: Vec::new() });
            }
            groups[index[r]].points.push(p);
        }
        for (k, o) in owner.iter().enumerate() {
            if let Some(&o) = o.as_ref() {
                let r = root(&mut parent, o);
                groups[index[r]].constraints.push(k);
            }
        }
        groups
    }

    fn scaled(&self, p: usize) -> [f64; 2] {
        [(self.start[p].x - self.centre[0]) / self.size, (self.start[p].y - self.centre[1]) / self.size]
    }

    /// Solves group `g` with the constraints `use_`: normalised coordinates of its points, or
    /// `None` when they cannot all hold.
    pub fn fit(&self, g: &Group, use_: &[usize]) -> Option<Vec<f64>> {
        let x0: Vec<f64> = g.points.iter().flat_map(|&p| self.scaled(p)).collect();
        let free: Vec<usize> = g.points.iter().enumerate().filter(|(_, p)| !self.fixed[**p]).flat_map(|(k, _)| [2 * k, 2 * k + 1]).collect();
        let f = |x: &[f64], jac: Option<&mut Vec<Vec<(usize, f64)>>>, r: &mut Vec<f64>| self.residuals(g, use_, x, r, jac);
        match solver::solve(&f, &x0, &free, TOLERANCE) {
            Fit::Converged(x) => Some(x),
            Fit::Failed => None,
        }
    }

    /// Degrees of freedom left in group `g` at `x`, and the points that can still move.
    pub fn freedom(&self, g: &Group, x: &[f64]) -> (usize, Vec<usize>) {
        let free: Vec<usize> = g.points.iter().enumerate().filter(|(_, p)| !self.fixed[**p]).flat_map(|(k, _)| [2 * k, 2 * k + 1]).collect();
        let mut rows = Vec::new();
        let mut r = Vec::new();
        self.residuals(g, &g.constraints, x, &mut r, Some(&mut rows));
        let (rank, null) = solver::null_space(&rows, &free, RANK);
        let moving = |k: usize| null.iter().any(|v| free.iter().zip(v).any(|(&c, s)| c / 2 == k && s.abs() > MOVES));
        let points = g.points.iter().enumerate().filter(|(k, _)| moving(*k)).map(|(_, &p)| p).collect();
        (free.len() - rank, points)
    }

    /// Coordinates back from normalised `x`, as offsets from the drawn ones.
    pub fn unscale(&self, g: &Group, x: &[f64]) -> Vec<Vec2> {
        g.points
            .iter()
            .enumerate()
            .map(|(k, &p)| {
                let s = self.scaled(p);
                let move_by = |i: usize| if x[2 * k + i] == s[i] { 0.0 } else { (x[2 * k + i] - s[i]) * self.size };
                Vec2::new(self.start[p].x + move_by(0), self.start[p].y + move_by(1))
            })
            .collect()
    }

    /// The constraints of group `g` each of which, left out, lets the rest be solved.
    pub fn conflicts(&self, g: &Group) -> Vec<String> {
        g.constraints
            .par_iter()
            .filter(|&&k| !matches!(self.eqs[k], Eq::Fix))
            .filter(|&&k| {
                let rest: Vec<usize> = g.constraints.iter().copied().filter(|&c| c != k).collect();
                self.fit(g, &rest).is_some()
            })
            .map(|&k| self.described[k].clone())
            .collect()
    }

    /// Residuals of the constraints `use_` at `x`, and with `jac` their gradients as sparse rows
    /// over the group's coordinates.
    fn residuals(&self, g: &Group, use_: &[usize], x: &[f64], r: &mut Vec<f64>, mut jac: Option<&mut Vec<Vec<(usize, f64)>>>) {
        r.clear();
        if let Some(j) = jac.as_deref_mut() {
            j.clear();
        }
        let local = |p: usize| g.points.binary_search(&p).unwrap_or(0);
        let at = |p: usize| {
            let k = local(p);
            [x[2 * k], x[2 * k + 1]]
        };
        let mut emit = |value: f64, grads: &[(usize, [f64; 2])]| {
            r.push(value);
            if let Some(j) = jac.as_deref_mut() {
                j.push(grads.iter().flat_map(|&(p, d)| [(2 * local(p), d[0]), (2 * local(p) + 1, d[1])]).collect());
            }
        };
        for &k in use_ {
            match self.eqs[k] {
                Eq::Fix => {}
                Eq::Coincident(a, b) => {
                    let (pa, pb) = (at(a), at(b));
                    emit(pa[0] - pb[0], &[(a, [1.0, 0.0]), (b, [-1.0, 0.0])]);
                    emit(pa[1] - pb[1], &[(a, [0.0, 1.0]), (b, [0.0, -1.0])]);
                }
                Eq::Horizontal(l) => {
                    let (a, b) = self.lines[l];
                    emit(at(b)[1] - at(a)[1], &[(a, [0.0, -1.0]), (b, [0.0, 1.0])]);
                }
                Eq::Vertical(l) => {
                    let (a, b) = self.lines[l];
                    emit(at(b)[0] - at(a)[0], &[(a, [-1.0, 0.0]), (b, [1.0, 0.0])]);
                }
                Eq::Parallel(i, j) => {
                    let (di, dj) = (self.dir(i, &at), self.dir(j, &at));
                    let v = cross(di.u, dj.u);
                    emit(v, &self.through2((i, di, [dj.u[1], -dj.u[0]]), (j, dj, [-di.u[1], di.u[0]])));
                }
                Eq::Perpendicular(i, j) => {
                    let (di, dj) = (self.dir(i, &at), self.dir(j, &at));
                    emit(dot(di.u, dj.u), &self.through2((i, di, dj.u), (j, dj, di.u)));
                }
                Eq::Collinear(i, j) => {
                    let (di, dj) = (self.dir(i, &at), self.dir(j, &at));
                    emit(cross(di.u, dj.u), &self.through2((i, di, [dj.u[1], -dj.u[0]]), (j, dj, [-di.u[1], di.u[0]])));
                    // The start of the second line on the first.
                    let (a, _) = self.lines[i];
                    let (c, _) = self.lines[j];
                    let q = sub(at(c), at(a));
                    let mut g = self.through1(i, &di, [q[1], -q[0]]).to_vec();
                    let along = [-di.u[1], di.u[0]];
                    g.push((c, along));
                    g.push((a, [-along[0], -along[1]]));
                    emit(cross(di.u, q), &g);
                }
                Eq::Equal(i, j) => {
                    let (di, dj) = (self.dir(i, &at), self.dir(j, &at));
                    let ((a, b), (c, d)) = (self.lines[i], self.lines[j]);
                    emit(di.len - dj.len, &[(b, di.u), (a, neg(di.u)), (d, neg(dj.u)), (c, dj.u)]);
                }
                Eq::Distance(a, b, value) => {
                    let d = sub(at(b), at(a));
                    let len = norm(d);
                    let u = [d[0] / len, d[1] / len];
                    emit(len - value, &[(b, u), (a, neg(u))]);
                }
                Eq::Length(l, value) => {
                    let (a, b) = self.lines[l];
                    let d = self.dir(l, &at);
                    emit(d.len - value, &[(b, d.u), (a, neg(d.u))]);
                }
                Eq::PointLine(p, l, value, side) => {
                    let (a, _) = self.lines[l];
                    let d = self.dir(l, &at);
                    let q = sub(at(p), at(a));
                    let mut g = self.through1(l, &d, [side * q[1], -side * q[0]]).to_vec();
                    let toward = [-side * d.u[1], side * d.u[0]];
                    g.push((p, toward));
                    g.push((a, neg(toward)));
                    emit(side * cross(d.u, q) - value, &g);
                }
                Eq::ArcEnds(c, f, t) => {
                    let (uf, df) = unit_from(at(c), at(f));
                    let (ut, dt) = unit_from(at(c), at(t));
                    emit(df - dt, &[(f, uf), (c, sub(ut, uf)), (t, neg(ut))]);
                }
                Eq::Radius(c, f, value) => {
                    let (u, d) = unit_from(at(c), at(f));
                    emit(d - value, &[(f, u), (c, neg(u))]);
                }
                Eq::LineCircle(l, c, rad, side) => {
                    let (a, _) = self.lines[l];
                    let d = self.dir(l, &at);
                    let q = sub(at(c), at(a));
                    let mut g = self.through1(l, &d, [side * q[1], -side * q[0]]).to_vec();
                    let toward = [-side * d.u[1], side * d.u[0]];
                    g.push((c, toward));
                    g.push((a, neg(toward)));
                    let r = radius(rad, c, &at, -1.0, &mut g);
                    emit(side * cross(d.u, q) - r, &g);
                }
                Eq::CircleCircle(c1, r1, c2, r2, contact) => {
                    let (u, dist) = unit_from(at(c1), at(c2));
                    let mut g = vec![(c2, u), (c1, neg(u))];
                    // Outside: dist − (r1 + r2); inside: dist − s·(r1 − r2).
                    let (s1, s2) = match contact {
                        Contact::Outside => (-1.0, -1.0),
                        Contact::Inside(s) => (-s, s),
                    };
                    let a = radius(r1, c1, &at, s1, &mut g);
                    let b = radius(r2, c2, &at, s2, &mut g);
                    emit(dist + s1 * a + s2 * b, &g);
                }
                Eq::LineArcAt(l, c, p) => {
                    let d = self.dir(l, &at);
                    let (w, len) = unit_from(at(c), at(p));
                    let mut g = self.through1(l, &d, w).to_vec();
                    let gp = across(w, d.u, len);
                    g.push((p, gp));
                    g.push((c, neg(gp)));
                    emit(dot(d.u, w), &g);
                }
                Eq::ArcArcAt(c1, c2, p) => {
                    let ((w1, l1), (w2, l2)) = (unit_from(at(c1), at(p)), unit_from(at(c2), at(p)));
                    let (g1, g2) = (across(w1, [w2[1], -w2[0]], l1), across(w2, [-w1[1], w1[0]], l2));
                    emit(cross(w1, w2), &[(p, [g1[0] + g2[0], g1[1] + g2[1]]), (c1, neg(g1)), (c2, neg(g2))]);
                }
                Eq::Angle(i, j, target) => {
                    let (di, dj) = (self.dir(i, &at), self.dir(j, &at));
                    let (s, c) = (cross(di.u, dj.u), dot(di.u, dj.u));
                    let off = libm::atan2(s, c) - target;
                    let wrapped = off - std::f64::consts::TAU * libm::floor((off + std::f64::consts::PI) / std::f64::consts::TAU);
                    // dφ = c·d(cross) − s·d(dot) for unit directions.
                    let gi = [c * dj.u[1] - s * dj.u[0], -c * dj.u[0] - s * dj.u[1]];
                    let gj = [-c * di.u[1] - s * di.u[0], c * di.u[0] - s * di.u[1]];
                    emit(wrapped, &self.through2((i, di, gi), (j, dj, gj)));
                }
            }
        }
    }

    fn dir(&self, l: usize, at: &impl Fn(usize) -> [f64; 2]) -> Dir {
        let (a, b) = self.lines[l];
        let d = sub(at(b), at(a));
        let len = norm(d);
        Dir { u: [d[0] / len, d[1] / len], len }
    }

    /// Gradient with respect to the ends of line `l`, given the gradient `gu` with respect to its
    /// unit direction: `∂u/∂b = (I − u uᵀ) / len`, `∂u/∂a = −∂u/∂b`.
    fn through1(&self, l: usize, d: &Dir, gu: [f64; 2]) -> [(usize, [f64; 2]); 2] {
        let (a, b) = self.lines[l];
        let k = dot(gu, d.u);
        let gb = [(gu[0] - d.u[0] * k) / d.len, (gu[1] - d.u[1] * k) / d.len];
        [(b, gb), (a, neg(gb))]
    }

    fn through2(&self, (i, di, gi): (usize, Dir, [f64; 2]), (j, dj, gj): (usize, Dir, [f64; 2])) -> [(usize, [f64; 2]); 4] {
        let [x, y] = self.through1(i, &di, gi);
        let [z, w] = self.through1(j, &dj, gj);
        [x, y, z, w]
    }
}

/// The unit vector from `a` to `b` and the distance.
fn unit_from(a: [f64; 2], b: [f64; 2]) -> ([f64; 2], f64) {
    let d = sub(b, a);
    let len = norm(d);
    ([d[0] / len, d[1] / len], len)
}

/// Gradient with respect to the far end of a unit vector `w` of length-`len` difference, given
/// the gradient `gw` with respect to `w`: `(I − w wᵀ) gw / len`.
fn across(w: [f64; 2], gw: [f64; 2], len: f64) -> [f64; 2] {
    let k = dot(gw, w);
    [(gw[0] - w[0] * k) / len, (gw[1] - w[1] * k) / len]
}

/// The radius `rad` of the circle around `c`, its gradient times `scale` added to `g`.
fn radius(rad: Rad, c: usize, at: &impl Fn(usize) -> [f64; 2], scale: f64, g: &mut Vec<(usize, [f64; 2])>) -> f64 {
    match rad {
        Rad::Point(p) => {
            let (u, d) = unit_from(at(c), at(p));
            g.push((p, [scale * u[0], scale * u[1]]));
            g.push((c, [-scale * u[0], -scale * u[1]]));
            d
        }
        Rad::Value(v) => v,
    }
}

#[derive(Clone, Copy)]
struct Dir {
    u: [f64; 2],
    len: f64,
}

fn sub(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [a[0] - b[0], a[1] - b[1]]
}

fn neg(a: [f64; 2]) -> [f64; 2] {
    [-a[0], -a[1]]
}

fn dot(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

fn cross(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

fn norm(a: [f64; 2]) -> f64 {
    dot(a, a).sqrt()
}
