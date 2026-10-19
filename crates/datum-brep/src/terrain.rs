//! Terrain: a regular grid of heights closed into a solid (PAINS P9).

use datum_math::{Vec2, Vec3};
use rayon::prelude::*;

use crate::{Body, BrepError, FaceSpec, Plane, TopoId, Vertex, build};

/// A regular grid of heights.
#[derive(Clone, Copy, Debug)]
pub struct Grid<'a> {
    /// Plan position of node `(0, 0)`.
    pub origin: Vec2,
    /// Node spacing along X and Y, both positive.
    pub step: [f64; 2],
    /// Nodes along X and Y, at least 2 each.
    pub count: [u32; 2],
    /// Height of node `(i, j)` at `heights[j·count[0] + i]`.
    pub heights: &'a [f64],
}

/// The ground under `grid` down to the level `base`, below every height: each grid cell is two
/// triangles split by the diagonal from node `(i, j)` to `(i + 1, j + 1)`, the sides are vertical
/// and the bottom is flat. IDs derive from `feature` and grid positions, so a changed height keeps
/// every ID and a larger grid keeps the IDs of the nodes and cells it shares.
pub fn terrain(feature: TopoId, grid: &Grid, base: f64) -> Result<Body, BrepError> {
    let degenerate = |msg: &str| Err(BrepError::Degenerate(msg.into()));
    let [nx, ny] = grid.count.map(|c| c as usize);
    if nx < 2 || ny < 2 {
        return degenerate("a terrain grid needs at least 2 × 2 nodes");
    }
    if grid.heights.len() != nx * ny {
        return Err(BrepError::Degenerate(format!("a {nx} × {ny} grid needs {} heights, got {}", nx * ny, grid.heights.len())));
    }
    let [dx, dy] = grid.step;
    if !(dx > 0.0 && dy > 0.0 && dx.is_finite() && dy.is_finite() && grid.origin.x.is_finite() && grid.origin.y.is_finite()) {
        return degenerate("terrain steps must be positive and coordinates finite");
    }
    if !base.is_finite() || grid.heights.iter().any(|h| !h.is_finite() || *h <= base) {
        return degenerate("every height must be finite and above the base");
    }

    // Every node is named once and used by up to six triangles and a side.
    let names: Vec<TopoId> = (0..nx * ny)
        .into_par_iter()
        .map(|k| TopoId::derive("terrain.node").id(feature).index((k % nx) as u64).index((k / nx) as u64).finish())
        .collect();
    let node = |i: usize, j: usize| names[j * nx + i];
    let corner = |k: u64| TopoId::derive("terrain.corner").id(feature).index(k).finish();
    let (x, y) = (|i: usize| grid.origin.x + dx * i as f64, |j: usize| grid.origin.y + dy * j as f64);
    let at = |i: usize, j: usize| Vec3::new(x(i), y(j), grid.heights[j * nx + i]);

    let mut vertices: Vec<Vertex> =
        (0..ny).into_par_iter().flat_map_iter(|j| (0..nx).map(move |i| Vertex { id: node(i, j), point: at(i, j) })).collect();
    let corners = [(0, 0), (nx - 1, 0), (nx - 1, ny - 1), (0, ny - 1)].map(|(i, j)| Vec3::new(x(i), y(j), base));
    vertices.extend((0..4).map(|k| Vertex { id: corner(k as u64), point: corners[k] }));

    let triangle = |a: (usize, usize), b: (usize, usize), c: (usize, usize), k: u64| {
        let (pa, pb, pc) = (at(a.0, a.1), at(b.0, b.1), at(c.0, c.1));
        FaceSpec {
            id: TopoId::derive("terrain.top").id(feature).index(a.0 as u64).index(a.1 as u64).index(k).finish(),
            plane: Plane { origin: pa, normal: (pb - pa).cross(pc - pa) },
            loops: vec![vec![node(a.0, a.1), node(b.0, b.1), node(c.0, c.1)]],
        }
    };
    let mut faces: Vec<FaceSpec> = (0..ny - 1)
        .into_par_iter()
        .flat_map_iter(|j| {
            (0..nx - 1).flat_map(move |i| [triangle((i, j), (i + 1, j), (i + 1, j + 1), 0), triangle((i, j), (i + 1, j + 1), (i, j + 1), 1)])
        })
        .collect();

    // Sides counterclockwise seen from outside: two bottom corners, then the top edge back.
    let side = |k: u64, normal: Vec3, top: Vec<TopoId>| {
        let mut ids = vec![corner(k), corner((k + 1) % 4)];
        ids.extend(top);
        FaceSpec {
            id: TopoId::derive("terrain.side").id(feature).index(k).finish(),
            plane: Plane { origin: corners[k as usize], normal },
            loops: vec![ids],
        }
    };
    faces.push(side(0, Vec3::new(0.0, -1.0, 0.0), (0..nx).rev().map(|i| node(i, 0)).collect()));
    faces.push(side(1, Vec3::new(1.0, 0.0, 0.0), (0..ny).rev().map(|j| node(nx - 1, j)).collect()));
    faces.push(side(2, Vec3::new(0.0, 1.0, 0.0), (0..nx).map(|i| node(i, ny - 1)).collect()));
    faces.push(side(3, Vec3::new(-1.0, 0.0, 0.0), (0..ny).map(|j| node(0, j)).collect()));
    faces.push(FaceSpec {
        id: TopoId::derive("terrain.bottom").id(feature).finish(),
        plane: Plane { origin: corners[0], normal: Vec3::new(0.0, 0.0, -1.0) },
        loops: vec![[0, 3, 2, 1].map(corner).to_vec()],
    });
    build(vertices, faces)
}
