//! Datum sketches (M3, D53–D55): points and line segments in a plane, constraints between them,
//! and the solver that places them.
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
use rayon::prelude::*;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use equations::{Equations, Group};

/// A sketch in its own plane. In JSON:
/// `{"points": {"a": [0, 0], …}, "lines": {"bottom": {"from": "a", "to": "b"}, …},
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
    #[serde(default)]
    pub constraints: Vec<Constraint>,
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

/// A constraint, by the names of the points and lines it ties. Lines are directed `from → to`;
/// distances and lengths are positive; angles are in degrees, counterclockwise from the first
/// line to the second.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum Constraint {
    Coincident {
        points: [String; 2],
    },
    Horizontal {
        line: String,
    },
    Vertical {
        line: String,
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
    /// The point stays where it was drawn.
    Fix {
        point: String,
    },
    Distance {
        points: [String; 2],
        value: f64,
    },
    Length {
        line: String,
        value: f64,
    },
    /// Distance from a point to the line through a segment, on the side the point was drawn.
    PointLine {
        point: String,
        line: String,
        value: f64,
    },
    Angle {
        lines: [String; 2],
        degrees: f64,
    },
}

impl Constraint {
    /// How a message names the constraint: its position and what it ties.
    fn describe(&self, k: usize) -> String {
        let (kind, what) = match self {
            Self::Coincident { points } => ("coincident", points.join(", ")),
            Self::Horizontal { line } => ("horizontal", line.clone()),
            Self::Vertical { line } => ("vertical", line.clone()),
            Self::Parallel { lines } => ("parallel", lines.join(", ")),
            Self::Perpendicular { lines } => ("perpendicular", lines.join(", ")),
            Self::Collinear { lines } => ("collinear", lines.join(", ")),
            Self::Equal { lines } => ("equal", lines.join(", ")),
            Self::Fix { point } => ("fix", point.clone()),
            Self::Distance { points, value } => ("distance", format!("{} = {value}", points.join(", "))),
            Self::Length { line, value } => ("length", format!("{line} = {value}")),
            Self::PointLine { point, line, value } => ("point_line", format!("{point}, {line} = {value}")),
            Self::Angle { lines, degrees } => ("angle", format!("{} = {degrees}°", lines.join(", "))),
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
    /// Points that can still move, in name order.
    pub free_points: Vec<String>,
}

/// Places the points so that every constraint holds, moving them as little as possible from the
/// coordinates they were drawn at.
pub fn solve(sketch: &Sketch) -> Result<Solved, SketchError> {
    let eq = Equations::new(sketch)?;
    let groups = eq.groups();
    let solved = groups.par_iter().map(|g| solve_group(&eq, g)).collect::<Vec<_>>();
    let mut points = eq.start.clone();
    let mut dof = 2 * eq.unconstrained.len();
    let mut free: Vec<usize> = eq.unconstrained.clone();
    for (g, r) in groups.iter().zip(solved) {
        let r = r?;
        for (k, &p) in g.points.iter().enumerate() {
            points[p] = r.points[k];
        }
        dof += r.dof;
        free.extend(r.free);
    }
    free.sort_unstable();
    Ok(Solved { points: eq.names.iter().cloned().zip(points).collect(), dof, free_points: free.into_iter().map(|p| eq.names[p].clone()).collect() })
}

struct GroupResult {
    points: Vec<Vec2>,
    dof: usize,
    free: Vec<usize>,
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
