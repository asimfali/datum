//! Datum sketches (M3, D53–D55; arcs and circles M5, D63): points, line segments, arcs and
//! circles in a plane, constraints between them, and the solver that places them.
//!
//! A sketch is data: points with the coordinates they were drawn at, lines between named points,
//! constraints by name. [`solve`] moves the points as little as possible from those coordinates
//! until every constraint holds, so the result is a pure function of the sketch: the same file
//! gives the same geometry bit for bit, whatever was solved before. Each connected group of
//! constrained points is solved on its own, in parallel.
//!
//! Under-constrained sketches solve; [`Solved`] reports the degrees of freedom left and which
//! points can still move. A sketch that cannot be solved is an error naming the constraints that
//! conflict.

mod equations;
mod solver;

use std::collections::BTreeMap;
use std::fmt;

use datum_math::Vec2;
use datum_math::expr::{ExprError, Num};
use rayon::prelude::*;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use equations::{Equations, Group};

/// A sketch in its own plane. In JSON:
/// `{"points": {"a": [0, 0], …}, "lines": {"bottom": {"from": "a", "to": "b"}, …},
///   "arcs": {"end": {"center": "c", "from": "b", "to": "d"}}, "circles": {"hole": {"center": "h", "diameter": 5}},
///   "constraints": [{"type": "horizontal", "line": "bottom"}, …]}`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Sketch {
    /// Points by name, at the coordinates they were drawn at: the solver starts from here.
    #[serde(default)]
    pub points: BTreeMap<String, Vec2>,
    /// Segments between named points.
    #[serde(default)]
    pub lines: BTreeMap<String, Line>,
    /// Arcs between named points, counterclockwise around a named centre: dimensioned by radius.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub arcs: BTreeMap<String, Arc>,
    /// Circles around named points: dimensioned by diameter, as on a drawing. A circle equal to
    /// another (`equal_radius`) takes that one's diameter instead of its own.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub circles: BTreeMap<String, Circle>,
    #[serde(default)]
    pub constraints: Vec<Constraint>,
    /// Reference dimensions by name: measured on the solved sketch, they constrain nothing.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub measures: BTreeMap<String, Measure>,
    /// Groups by name: sets of the sketch's points, lines, arcs and circles, to name together
    /// (a pattern's cell, a path). Only a union: they hold nothing, may share elements, hold no
    /// groups. A group's name is no element's.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub groups: BTreeMap<String, Vec<String>>,
    /// Patterns by name: copies of a group's profiles (or of one curve's).
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub patterns: BTreeMap<String, Pattern>,
}

/// Copies of the closed profiles of `of` (a group or a curve), on the grid of `step`: copy
/// `(i, j)` moved by `i·step[0] + j·step[1]`. Either `count` copies along each step, the group
/// itself copy `(0, 0)`; or as many as `fill` (a group bounding a region, or a circle) holds,
/// centred in it, `clearance` from its edge and from every other profile of the sketch, the
/// group a sample only. `skip` leaves copies out, `shift` moves one copy by its own vector.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Pattern {
    pub of: String,
    pub step: [[Num; 2]; 2],
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub count: Option<[Num; 2]>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub fill: Option<String>,
    #[serde(default = "zero", skip_serializing_if = "is_zero")]
    pub clearance: Num,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub skip: Vec<[i64; 2]>,
    /// By copy, `"i,j"`.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub shift: BTreeMap<String, [Num; 2]>,
}

fn zero() -> Num {
    Num::Value(0.0)
}

fn is_zero(n: &Num) -> bool {
    *n == Num::Value(0.0)
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Line {
    pub from: String,
    pub to: String,
    /// A construction line guides other geometry but bounds no profile.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub construction: bool,
}

/// An arc counterclockwise around `center` from `from` to `to`. Its ends stay at one distance
/// from the centre.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Arc {
    pub center: String,
    pub from: String,
    pub to: String,
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub construction: bool,
}

/// A circle around the point `center`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Circle {
    pub center: String,
    pub diameter: Num,
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub construction: bool,
}

/// A constraint, by the names of the points and lines it ties. Lines are directed `from → to`;
/// distances and lengths are positive; angles are in degrees, counterclockwise from the first
/// line to the second.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum Constraint {
    Coincident {
        points: [String; 2],
    },
    /// A line along the sketch's X axis, or two points on one horizontal: give `line` or
    /// `points`.
    Horizontal {
        #[serde(default, skip_serializing_if = "Option::is_none")]
        line: Option<String>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        points: Option<[String; 2]>,
    },
    /// A line along the sketch's Y axis, or two points on one vertical.
    Vertical {
        #[serde(default, skip_serializing_if = "Option::is_none")]
        line: Option<String>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        points: Option<[String; 2]>,
    },
    Parallel {
        lines: [String; 2],
    },
    Perpendicular {
        lines: [String; 2],
    },
    /// On one straight line.
    Collinear {
        lines: [String; 2],
    },
    /// Of equal length.
    Equal {
        lines: [String; 2],
    },
    /// The point stays where it was drawn, or, with `at`, stands there. Sides are still decided
    /// where the point was drawn: a projection drawn before the model changed (D87).
    Fix {
        point: String,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        at: Option<Vec2>,
    },
    Distance {
        points: [String; 2],
        value: Num,
    },
    /// Distance along the sketch's X axis, on the side the second point was drawn.
    HorizontalDistance {
        points: [String; 2],
        value: Num,
    },
    /// Distance along the sketch's Y axis, on the side the second point was drawn.
    VerticalDistance {
        points: [String; 2],
        value: Num,
    },
    Length {
        line: String,
        value: Num,
    },
    /// Distance from a point to the line through a segment, on the side the point was drawn.
    PointLine {
        point: String,
        line: String,
        value: Num,
    },
    Angle {
        lines: [String; 2],
        degrees: Num,
    },
    /// Radius of an arc.
    Radius {
        arc: String,
        value: Num,
    },
    /// Length along an arc, from its start to its end.
    ArcLength {
        arc: String,
        value: Num,
    },
    /// A line, arc or circle touching an arc or circle, on the side it was drawn: a line's
    /// distance from the centre is the radius; two circles touch outside or inside each other.
    Tangent {
        curves: [String; 2],
    },
    /// The point on a curve: on the line through a segment, or on the circle of an arc or circle.
    On {
        point: String,
        curve: String,
    },
    /// The point halfway along a segment.
    Midpoint {
        point: String,
        line: String,
    },
    /// Two points mirrored in the line through a segment.
    Symmetric {
        points: [String; 2],
        line: String,
    },
    /// Arcs or circles of one radius; of two circles, the second takes the first's diameter.
    EqualRadius {
        curves: [String; 2],
    },
    /// Arcs or circles around one centre.
    Concentric {
        curves: [String; 2],
    },
}

/// A reference dimension: what it measures, as the dimension constraint of the same name would
/// fix it. Lengths in the sketch's units, angles in degrees counterclockwise from the first line
/// to the second, from 0 up to 360.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum Measure {
    Distance { points: [String; 2] },
    HorizontalDistance { points: [String; 2] },
    VerticalDistance { points: [String; 2] },
    Length { line: String },
    PointLine { point: String, line: String },
    Angle { lines: [String; 2] },
    Radius { arc: String },
    ArcLength { arc: String },
}

impl Sketch {
    /// The same sketch with every dimension a number: expressions evaluated with parameter
    /// values from `lookup`. An error names the constraint.
    pub fn resolve(&self, lookup: &impl Fn(&str) -> Result<f64, ExprError>) -> Result<Sketch, String> {
        let mut s = self.clone();
        for (name, c) in &mut s.circles {
            c.diameter = Num::Value(c.diameter.resolve(lookup).map_err(|e| format!("circle `{name}`: {e}"))?);
        }
        for (name, p) in &mut s.patterns {
            let num = |n: &mut Num, what: &str| -> Result<(), String> {
                *n = Num::Value(n.resolve(lookup).map_err(|e| format!("pattern `{name}`: {what}: {e}"))?);
                Ok(())
            };
            for n in p.step.iter_mut().flatten().chain(p.count.iter_mut().flatten()) {
                num(n, "step or count")?;
            }
            num(&mut p.clearance, "clearance")?;
            for (k, v) in &mut p.shift {
                for n in v {
                    num(n, &format!("shift {k}"))?;
                }
            }
        }
        for (circle, leader) in self.followed_circles()? {
            let d = s.circles[&leader].diameter.clone();
            if let Some(c) = s.circles.get_mut(&circle) {
                c.diameter = d;
            }
        }
        for (k, c) in s.constraints.iter_mut().enumerate() {
            let describe = c.describe(k);
            if let Constraint::Distance { value, .. }
            | Constraint::HorizontalDistance { value, .. }
            | Constraint::VerticalDistance { value, .. }
            | Constraint::Length { value, .. }
            | Constraint::PointLine { value, .. }
            | Constraint::Radius { value, .. }
            | Constraint::ArcLength { value, .. }
            | Constraint::Angle { degrees: value, .. } = c
            {
                *value = Num::Value(value.resolve(lookup).map_err(|e| format!("{describe}: {e}"))?);
            }
        }
        Ok(s)
    }
}

impl Sketch {
    /// The circles whose diameter follows another's through `equal_radius` (the second named
    /// follows the first), each by the circle the chain starts at. An error names a constraint
    /// that ties circles of one diameter already.
    pub fn followed_circles(&self) -> Result<BTreeMap<String, String>, String> {
        fn first<'a>(followed: &'a BTreeMap<String, String>, mut n: &'a str) -> String {
            while let Some(m) = followed.get(n) {
                n = m;
            }
            n.to_string()
        }
        let mut followed = BTreeMap::new();
        for (k, c) in self.constraints.iter().enumerate() {
            if let Constraint::EqualRadius { curves: [a, b] } = c
                && self.circles.contains_key(a)
                && self.circles.contains_key(b)
            {
                let (a, b) = (first(&followed, a), first(&followed, b));
                if a == b {
                    return Err(format!("{}: the circles are of one diameter already", c.describe(k)));
                }
                followed.insert(b, a);
            }
        }
        Ok(followed.keys().map(|n| (n.clone(), first(&followed, n))).collect())
    }
}

impl Constraint {
    /// How a message names the constraint: its position and what it ties.
    pub(crate) fn describe(&self, k: usize) -> String {
        let (kind, what) = match self {
            Self::Coincident { points } => ("coincident", points.join(", ")),
            Self::Horizontal { line, points } => ("horizontal", line.clone().or_else(|| points.as_ref().map(|p| p.join(", "))).unwrap_or_default()),
            Self::Vertical { line, points } => ("vertical", line.clone().or_else(|| points.as_ref().map(|p| p.join(", "))).unwrap_or_default()),
            Self::Parallel { lines } => ("parallel", lines.join(", ")),
            Self::Perpendicular { lines } => ("perpendicular", lines.join(", ")),
            Self::Collinear { lines } => ("collinear", lines.join(", ")),
            Self::Equal { lines } => ("equal", lines.join(", ")),
            Self::Fix { point, .. } => ("fix", point.clone()),
            Self::Distance { points, value } => ("distance", format!("{} = {value}", points.join(", "))),
            Self::HorizontalDistance { points, value } => ("horizontal_distance", format!("{} = {value}", points.join(", "))),
            Self::VerticalDistance { points, value } => ("vertical_distance", format!("{} = {value}", points.join(", "))),
            Self::Length { line, value } => ("length", format!("{line} = {value}")),
            Self::PointLine { point, line, value } => ("point_line", format!("{point}, {line} = {value}")),
            Self::Angle { lines, degrees } => ("angle", format!("{} = {degrees}°", lines.join(", "))),
            Self::Radius { arc, value } => ("radius", format!("{arc} = {value}")),
            Self::ArcLength { arc, value } => ("arc_length", format!("{arc} = {value}")),
            Self::Tangent { curves } => ("tangent", curves.join(", ")),
            Self::On { point, curve } => ("on", format!("{point}, {curve}")),
            Self::Midpoint { point, line } => ("midpoint", format!("{point}, {line}")),
            Self::Symmetric { points, line } => ("symmetric", format!("{} about {line}", points.join(", "))),
            Self::EqualRadius { curves } => ("equal_radius", curves.join(", ")),
            Self::Concentric { curves } => ("concentric", curves.join(", ")),
        };
        format!("#{k} {kind} ({what})")
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SketchError {
    /// The sketch refers to something it does not have, or a value makes no sense.
    Invalid(String),
    /// No placement satisfies the constraints; these conflict (each one, removed, would let the
    /// rest be solved).
    Conflict(Vec<String>),
}

impl fmt::Display for SketchError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Invalid(m) => write!(f, "invalid sketch: {m}"),
            Self::Conflict(c) if c.is_empty() => write!(f, "the constraints cannot all hold"),
            Self::Conflict(c) => write!(f, "the constraints cannot all hold; in conflict: {}", c.join("; ")),
        }
    }
}

impl std::error::Error for SketchError {}

/// A solved sketch.
#[derive(Clone, Debug, PartialEq)]
pub struct Solved {
    pub points: BTreeMap<String, Vec2>,
    /// Degrees of freedom left: independent ways the points can still move.
    pub dof: usize,
    /// Points that can still move, by name: free in the plane (`None`) or sliding along a unit
    /// direction (the first nonzero coordinate positive) — what a dimension would still fix.
    pub free: BTreeMap<String, Option<Vec2>>,
    /// The values of the reference dimensions, by name.
    pub measures: BTreeMap<String, f64>,
}

/// Places the points so that every constraint holds, moving them as little as possible from the
/// coordinates they were drawn at.
pub fn solve(sketch: &Sketch) -> Result<Solved, SketchError> {
    let eq = Equations::new(sketch)?;
    let groups = eq.groups();
    let solved = groups.par_iter().map(|g| solve_group(&eq, g)).collect::<Vec<_>>();
    let mut points = eq.start.clone();
    let mut dof = 2 * eq.unconstrained.len();
    let mut free: Vec<(usize, Option<Vec2>)> = eq.unconstrained.iter().map(|&p| (p, None)).collect();
    for (g, r) in groups.iter().zip(solved) {
        let r = r?;
        for (k, &p) in g.points.iter().enumerate() {
            points[p] = r.points[k];
        }
        dof += r.dof;
        free.extend(r.free);
    }
    for (p, at) in eq.fixed_at.iter().enumerate() {
        if let Some(at) = at {
            points[p] = *at;
        }
    }
    free.sort_unstable_by_key(|f| f.0);
    let measures = eq.measures(&points);
    Ok(Solved {
        points: eq.names.iter().cloned().zip(points).collect(),
        dof,
        free: free.into_iter().map(|(p, d)| (eq.names[p].clone(), d)).collect(),
        measures,
    })
}

struct GroupResult {
    points: Vec<Vec2>,
    dof: usize,
    free: Vec<(usize, Option<Vec2>)>,
}

fn solve_group(eq: &Equations, g: &Group) -> Result<GroupResult, SketchError> {
    let fitted = eq.fit(g, &g.constraints);
    match fitted {
        Some(x) => {
            let (dof, free) = eq.freedom(g, &x);
            Ok(GroupResult { points: eq.unscale(g, &x), dof, free })
        }
        None => Err(SketchError::Conflict(eq.conflicts(g))),
    }
}

#[cfg(test)]
mod tests;
