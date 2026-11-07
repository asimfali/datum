//! Flanges of sheet metal (M8, D205): on an end face of a sheet (an end of a sheet along a path,
//! an edge of a face of sheet metal), a bend of the rule's inner radius and a wall beyond it,
//! made directly: the new faces are sewn to the end face's own vertices, which it gives up. A
//! flange's side lies in the plane of the face beside the end where that face is square to the
//! edge (a sheet's side, a plate's square corner): it joins that face, as a bend's side is part of
//! the sheet's side face.

use std::collections::BTreeSet;

use datum_brep::{Arc, Body, BrepError, Curve, CurveSpec, Cylinder, FaceSpec, Ix, Plane, Surface, TopoId, Vertex, build_with_curves};
use datum_math::{Sign, Vec3, orient3d_coords};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

use crate::bent::Corner;
use crate::sketch::parallel;

/// What a flange is: its height from the outer surface of the sheet, square to it up to the
/// flange's highest point (`along`: for another angle than a right one, along the flange from where
/// its outer surface meets the sheet's, D205), the angle it turns by in degrees, its bend's inner radius, the side it turns to (`flip`: the other), and at corners
/// where flanges meet a round relief's diameter and the K-factor the blank is laid at (D229).
#[derive(Clone, Copy, Debug)]
pub struct FlangeDims {
    pub height: f64,
    pub along: bool,
    pub angle: f64,
    pub radius: f64,
    pub flip: bool,
    pub round: Option<(f64, f64)>,
    /// How walls meeting at a corner close it, and the gap between them there.
    pub corner: Closing,
    pub gap: Option<f64>,
    /// How much of its edge each flange takes, and the relief cut beside its bend where that
    /// leaves the edge's rest flat.
    pub width: Width,
    pub notch: Option<Notch>,
    /// The bend flush (D229): set back so the flange's outer faces meet the sheet's at the edge,
    /// not added beyond it.
    pub flush: bool,
    /// The planes the wall's edges go to, instead of their own place.
    pub to: WallTo<Limit>,
}

/// What each of a flange's wall's three edges goes to (D229), none where it stays as it is: the
/// sides at the start and the end of its width (as [`Extent`] has them) and its free edge, `top`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(bound(deserialize = "P: Deserialize<'de>"))]
pub struct WallTo<P> {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub start: Option<P>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub end: Option<P>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub top: Option<P>,
}

impl<P> Default for WallTo<P> {
    fn default() -> Self {
        Self { start: None, end: None, top: None }
    }
}

impl<P> WallTo<P> {
    pub fn is_empty(&self) -> bool {
        self.start.is_none() && self.end.is_none() && self.top.is_none()
    }

    /// What there is of it, in that order.
    pub fn iter(&self) -> impl Iterator<Item = &P> {
        [&self.start, &self.end, &self.top].into_iter().flatten()
    }

    /// The same with each `f` of it.
    pub fn map<Q, E>(&self, mut f: impl FnMut(&P) -> Result<Q, E>) -> Result<WallTo<Q>, E> {
        Ok(WallTo {
            start: self.start.as_ref().map(&mut f).transpose()?,
            end: self.end.as_ref().map(&mut f).transpose()?,
            top: self.top.as_ref().map(&mut f).transpose()?,
        })
    }
}

/// A plane an edge of a wall goes to, the wall stopping `gap` short of it on the side it comes from.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Limit {
    pub at: Vec3,
    pub normal: Vec3,
    pub gap: f64,
}

impl Limit {
    /// The bound it makes of a wall going the way `way`: none for a plane along that way.
    fn bound(&self, way: Vec3) -> Option<Bound> {
        let n = unit(self.normal);
        let d = n.dot(way);
        if d == 0.0 {
            return None;
        }
        let n = if d < 0.0 { n * -1.0 } else { n };
        Some(Bound { at: self.at - n * self.gap, normal: n })
    }
}

/// Where a flange's bend lies against its edge (D229, as Inventor's bend position): beyond it,
/// the flange added to the sheet; or flush, the sheet set back so the flange's outer faces meet
/// its own at the edge.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum BendPosition {
    #[default]
    Outside,
    Flush,
}

impl BendPosition {
    pub fn is_outside(&self) -> bool {
        *self == Self::Outside
    }
}

/// A bend relief at an end of a flange narrower than its edge (D229): a slot into the sheet beside
/// the bend, `width` along the edge, `depth` into the sheet, its bottom square or round.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Notch {
    pub round: bool,
    pub width: f64,
    pub depth: f64,
}

/// The bend relief a flange narrower than its edge cuts beside its bend (D229).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum BendRelief {
    #[default]
    None,
    Square,
    Round,
}

impl BendRelief {
    pub fn is_none(&self) -> bool {
        *self == Self::None
    }
}

/// How much of the edge a flange takes (D229, as Inventor's width extents): all of it; `width`
/// in the middle; `width` from `offset` off the edge's start (`from_end`: its end); or all but
/// `start` and `end` off its ends. The edge's start: its end with the lower vertex ID.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Extent<N> {
    #[default]
    Edge,
    Center {
        width: N,
    },
    Offset {
        offset: N,
        width: N,
        #[serde(default, skip_serializing_if = "std::ops::Not::not")]
        from_end: bool,
    },
    Between {
        start: N,
        end: N,
    },
}

/// The extent with its numbers.
pub type Width = Extent<f64>;

impl<N> Extent<N> {
    pub fn is_edge(&self) -> bool {
        matches!(self, Self::Edge)
    }

    /// The same with each number `f` of it.
    pub fn map<M, E>(&self, mut f: impl FnMut(&N, &str) -> Result<M, E>) -> Result<Extent<M>, E> {
        Ok(match self {
            Self::Edge => Extent::Edge,
            Self::Center { width } => Extent::Center { width: f(width, "width")? },
            Self::Offset { offset, width, from_end } => {
                Extent::Offset { offset: f(offset, "offset")?, width: f(width, "width")?, from_end: *from_end }
            }
            Self::Between { start, end } => Extent::Between { start: f(start, "start")?, end: f(end, "end")? },
        })
    }
}

impl Width {
    /// Where on an edge `length` long the flange begins and ends, from the edge's start.
    fn span(&self, length: f64) -> Result<(f64, f64), BrepError> {
        let (a, b) = match *self {
            Self::Edge => (0.0, length),
            Self::Center { width } => (0.5 * (length - width), 0.5 * (length + width)),
            Self::Offset { offset, width, from_end: false } => (offset, offset + width),
            Self::Offset { offset, width, from_end: true } => (length - offset - width, length - offset),
            Self::Between { start, end } => (start, length - end),
        };
        if !(0.0 <= a && a < b && b <= length) {
            return Err(BrepError::Degenerate(format!("a flange's width from {a:.3} to {b:.3} lies not on its edge {length:.3} long")));
        }
        Ok((a, b))
    }
}

/// How the walls of flanges meeting at a corner close it (D229).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Closing {
    /// Each wall ends at its edge: the corner stays open.
    #[default]
    Open,
    /// Both walls go to the plane halfway between them, the gap apart.
    Miter,
    /// The walls along the first edge picked go to the other's outer face, the others to their
    /// inner face, the gap short of it.
    #[serde(rename = "overlap_1")]
    Overlap1,
    /// The other way round.
    #[serde(rename = "overlap_2")]
    Overlap2,
}

impl Closing {
    pub fn is_open(&self) -> bool {
        *self == Self::Open
    }
}

/// The relief at a corner where flanges meet (D229).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Relief {
    /// The square between the two bends' strips, which the open corner leaves.
    #[default]
    Square,
    /// That square and a circle about the corner where the bend lines meet.
    Round,
}

impl Relief {
    pub fn is_square(&self) -> bool {
        *self == Self::Square
    }
}

/// Flanges on edges `edges` of sheet metal `target`, one after another (D205): a long edge of an
/// end of the sheet puts it on that end, an edge of a wall on the sheet's side face puts it along
/// the wall (outward, no relief: it ends at the wall's bend lines). Two edges of one end or of
/// one wall's side make one flange; flanges on two ends meeting at a convex corner leave it open.
pub fn flanges(feature: TopoId, target: &Body, edges: &[String], dims: FlangeDims) -> Result<Body, BrepError> {
    if edges.is_empty() {
        return Err(BrepError::Sketch("a flange takes the edges of the sheet it stands on: `edges`".into()));
    }
    if !(dims.angle > 0.0 && dims.angle < 180.0) {
        return Err(BrepError::Degenerate(format!("a flange turns by more than 0° and less than 180°, got {}", dims.angle)));
    }
    if !(dims.radius > 0.0 && dims.height > 0.0 && dims.radius.is_finite() && dims.height.is_finite()) {
        return Err(BrepError::Degenerate("a flange needs a positive height and bend radius".into()));
    }
    let ids: Vec<TopoId> = edges
        .iter()
        .map(|e| u128::from_str_radix(e, 16).map(TopoId).map_err(|_| BrepError::Sketch(format!("`{e}` is not an edge ID"))))
        .collect::<Result<_, _>>()?;
    // Narrower than its edge: the end divided first, the flange on its middle (D229).
    if !dims.width.is_edge() || dims.flush {
        let mut body = target.clone();
        let mut middles = Vec::new();
        let mut ways = Vec::new();
        // Each end once, by either of its long edges; by its face's ID, which stays as the ends
        // beside it are cut (their corners, flush, move).
        let mut done: Vec<TopoId> = Vec::new();
        for &e in &ids {
            let end = end_face(target, e)?;
            if done.contains(&end) {
                continue;
            }
            done.push(end);
            ways.push(start_way(target, e)?);
            body = narrowed(feature, &body, end, dims.width, dims.notch, dims.flush.then_some((dims.radius, dims.angle)))?;
        }
        // Each middle by a long edge, once all are cut: it keeps its end's ID.
        for end in done {
            let f = body.faces().binary_search_by_key(&end, |x| x.id).map_err(|_| BrepError::Invalid("flange: an end lost".into()))? as Ix;
            let ([h, _], _) = end_of(&body, f).ok_or_else(|| BrepError::Invalid("flange: an end's middle is no end".into()))?;
            middles.push(body.edges()[body.half_edges()[h as usize].edge as usize].id);
        }
        return flanges_on(feature, &body, &middles, &ways, FlangeDims { width: Width::Edge, flush: false, ..dims });
    }
    let ways = ids.iter().map(|&e| start_way(target, e)).collect::<Result<Vec<_>, _>>()?;
    flanges_on(feature, target, &ids, &ways, dims)
}

/// The way from the start of the width of what edge `e` of `target` stands on to its end: the end
/// of the sheet's first long edge with the lower vertex ID, as a width is measured from; a wall's
/// inner edge, in the order of its IDs.
fn start_way(target: &Body, e: TopoId) -> Result<Vec3, BrepError> {
    let (from, to) = match end_by_edge(target, e) {
        Ok((f, ..)) => {
            let ([l0, _], _) = end_of(target, f).ok_or_else(|| BrepError::Invalid("flange: an end is no end".into()))?;
            let he = |h: Ix| target.half_edges()[h as usize];
            (he(l0).origin, he(target.next(l0)).origin)
        }
        Err(_) => {
            let b = base(target, e, false, &BTreeSet::new())?;
            (b.inner[0], b.inner[1])
        }
    };
    let id = |v: Ix| target.vertices()[v as usize].id;
    let way = target.point(to) - target.point(from);
    Ok(if id(from) < id(to) { way } else { way * -1.0 })
}

/// Flanges on the whole of edges `ids` of `target`, one after another, the start of each width
/// where `ways` says.
fn flanges_on(feature: TopoId, target: &Body, ids: &[TopoId], ways: &[Vec3], dims: FlangeDims) -> Result<Body, BrepError> {
    // Each flange once, by the corners of what it stands on.
    let mut sites: Vec<(TopoId, [TopoId; 4], Base)> = Vec::new();
    let mut site_ways: Vec<Vec3> = Vec::new();
    for (&e, &way) in ids.iter().zip(ways) {
        let base = base(target, e, dims.flip, &BTreeSet::new())?;
        let corners = [base.inner[0], base.inner[1], base.outer[0], base.outer[1]].map(|v| target.vertices()[v as usize].id);
        let mut sorted = corners;
        sorted.sort();
        if sites.iter().all(|(_, c, _)| *c != sorted) {
            sites.push((e, sorted, base));
            site_ways.push(way);
        }
    }
    // Flanges on two ends meeting at a corner leave it open (the corner's relief): neither's side
    // goes into the other's end. Each corner: its two vertices, the way away from the sheet there.
    let mut corners: Vec<([TopoId; 2], Vec3)> = Vec::new();
    for (k, (_, c, a)) in sites.iter().enumerate() {
        for (_, d, b) in &sites[k + 1..] {
            if c.iter().any(|v| d.contains(v)) {
                open_corner(target, a, b, dims.angle)?;
                let shared: Vec<TopoId> = c.iter().copied().filter(|v| d.contains(v)).collect();
                let [v, w] = shared[..] else { return Err(BrepError::Invalid("flange: a corner of more than one edge".into())) };
                corners.push(([v, w], a.out + b.out));
            }
        }
    }
    let bounds = closed(target, &sites, dims)?;
    let taken: BTreeSet<TopoId> = sites.iter().flat_map(|(.., b)| b.gone.iter().map(|&x| target.faces()[x as usize].id)).collect();
    let mut body = target.clone();
    for (((e, _, first), bounds), way) in sites.into_iter().zip(bounds).zip(site_ways) {
        // The side each turns to as the sheet picked had it: flanges made before it may tip
        // which side of the sheet is its front.
        let mut base = base(&body, e, dims.flip, &taken)?;
        if base.up.dot(first.up) < 0.0 {
            base = self::base(&body, e, !dims.flip, &taken)?;
        }
        body = grow(feature, &body, base, dims, bounds, way)?;
    }
    match dims.round {
        Some((d, k)) if !corners.is_empty() => relieve_corners(feature, &body, &corners, 0.5 * d, k),
        _ => Ok(body),
    }
}

/// Round reliefs at `corners` of `body` (D229): about the vertex of each on the sheet's front
/// side, cut through the blank from the face of the front side it is a corner of.
fn relieve_corners(feature: TopoId, body: &Body, corners: &[([TopoId; 2], Vec3)], radius: f64, k: f64) -> Result<Body, BrepError> {
    if !(radius > 0.0 && radius.is_finite()) {
        return Err(BrepError::Degenerate(format!("a relief's diameter is above zero, got {}", 2.0 * radius)));
    }
    let sides = crate::unfold::sheet_sides(body)?;
    let front = crate::unfold::side_of(body, sides[0])?;
    let vertex = |id: TopoId| body.vertices().binary_search_by_key(&id, |v| v.id).map(|v| v as Ix).ok();
    // Each corner by the flat face of the front side it lies on (by its ID: faces are numbered
    // anew after a cut), the faces each in turn.
    let mut by_face: Vec<(TopoId, Vec<Corner>)> = Vec::new();
    for &(ids, away) in corners {
        let found = ids.iter().filter_map(|&id| vertex(id).map(|v| (id, v))).find_map(|(id, v)| {
            let x = front.iter().copied().find(|&x| {
                matches!(body.faces()[x as usize].surface, Surface::Plane(_))
                    && body.face_loops(x).flat_map(|l| body.loop_half_edges(l)).any(|h| body.half_edges()[h as usize].origin == v)
            })?;
            Some((body.faces()[x as usize].id, Corner { key: id, at: body.point(v), away }))
        });
        let (x, corner) =
            found.ok_or_else(|| BrepError::Unsupported("flange: a relief at a corner off the sheet's flat front comes later".into()))?;
        match by_face.iter_mut().find(|(y, _)| *y == x) {
            Some((_, cs)) => cs.push(corner),
            None => by_face.push((x, vec![corner])),
        }
    }
    let mut body = body.clone();
    for (x, cs) in by_face {
        let home = body.faces().binary_search_by_key(&x, |f| f.id).map_err(|_| BrepError::Invalid("relief: a face lost".into()))? as Ix;
        body = crate::bent::relieve(feature, &body, home, &cs, radius, k)?;
    }
    Ok(body)
}

/// Where the walls of flanges `sites` of `target` meeting at corners go, closing them as `dims`
/// says (D229): for each flange, at the start and end of its width, the plane its wall's side
/// goes to; none at an open corner.
fn closed(target: &Body, sites: &[(TopoId, [TopoId; 4], Base)], dims: FlangeDims) -> Result<Vec<[Option<Bound>; 2]>, BrepError> {
    let mut bounds = vec![[None, None]; sites.len()];
    if dims.corner.is_open() {
        return Ok(bounds);
    }
    let g = dims.gap.ok_or_else(|| BrepError::Invalid("a closed corner takes its gap from the part's sheet rule: it has none".into()))?;
    if !(g >= 0.0 && g.is_finite()) {
        return Err(BrepError::Degenerate(format!("a corner's gap is not below zero, got {g}")));
    }
    let shapes: Vec<Shape> = sites.iter().map(|(.., b)| Shape::of(target, b, dims)).collect::<Result<_, _>>()?;
    let first = shapes.first().map_or(Vec3::ZERO, |s| s.w);
    for a in 0..sites.len() {
        for b in a + 1..sites.len() {
            let (ca, cb) = (&sites[a].1, &sites[b].1);
            let Some(&shared) = ca.iter().find(|v| cb.contains(v)) else { continue };
            let corner = target.vertices().binary_search_by_key(&shared, |v| v.id).map(|v| target.point(v as Ix)).unwrap_or(Vec3::ZERO);
            // The end of each width at the corner.
            let end = |k: usize| {
                let base = &sites[k].2;
                usize::from(![base.inner[0], base.outer[0]].iter().any(|&v| target.vertices()[v as usize].id == shared))
            };
            let (ia, oa, ma) = shapes[a].faces(target, &sites[a].2);
            let (ib, ob, mb) = shapes[b].faces(target, &sites[b].2);
            let [ba, bb] = match dims.corner {
                Closing::Open => continue,
                Closing::Miter => {
                    // The plane through where the outer faces meet and where the inner faces do.
                    let l = unit(ma.cross(mb));
                    let meet = |p: Vec3, q: Vec3| solve([ma, mb, l], [p.dot(ma), q.dot(mb), corner.dot(l)]);
                    let (Some(out), Some(inn)) = (meet(oa, ob), meet(ia, ib)) else {
                        return Err(BrepError::Degenerate("flange: walls meeting at a corner are parallel".into()));
                    };
                    let mut mu = unit(l.cross(out - inn));
                    // `a`'s own edge lies on its side of the plane.
                    let middle = (target.point(sites[a].2.inner[0]) + target.point(sites[a].2.inner[1])) * 0.5;
                    if (middle - out).dot(mu) > 0.0 {
                        mu = mu * -1.0;
                    }
                    [Bound { at: out - mu * (0.5 * g), normal: mu }, Bound { at: out + mu * (0.5 * g), normal: mu * -1.0 }]
                }
                Closing::Overlap1 | Closing::Overlap2 => {
                    // The covering wall: the one along the first edge picked, more nearly.
                    let a_along = shapes[a].w.dot(first).abs() >= shapes[b].w.dot(first).abs();
                    let covers = a_along == (dims.corner == Closing::Overlap1);
                    let over = |o: Vec3, m: Vec3| Bound { at: o, normal: m };
                    let under = |i: Vec3, m: Vec3| Bound { at: i - m * g, normal: m };
                    if covers { [over(ob, mb), under(ia, ma)] } else { [under(ib, mb), over(oa, ma)] }
                }
            };
            bounds[a][end(a)] = Some(ba);
            bounds[b][end(b)] = Some(bb);
        }
    }
    if dims.corner == Closing::Miter {
        mitres_in_plane(target, sites, &shapes, g, &mut bounds);
    }
    // A wall of a flange made before beside an end's corner (D229): the new wall goes to its inner
    // face, the gap short of it; that wall stays as it is.
    for ((.., base), ends) in sites.iter().zip(&mut bounds) {
        for (s, bound) in ends.iter_mut().enumerate() {
            if bound.is_some() || base.joins[s].is_some() {
                continue;
            }
            if let Some(x) = base.beside[s]
                && let Some((at, n)) = wall_beside(target, x, base, s)
            {
                *bound = Some(Bound { at: at + n * g, normal: n * -1.0 });
            }
        }
    }
    Ok(bounds)
}

/// Mitres between walls of `sites` lying in one plane (rims on a tray's walls, D233): they share no
/// vertex, and are neighbours by geometry. Two walls not parallel meet where the lines of their
/// feet do, at the end of each nearer to it; each end takes its nearest neighbour. The mitre runs
/// through where their feet meet and where their free edges do, square to the plane, `g` apart.
fn mitres_in_plane(target: &Body, sites: &[(TopoId, [TopoId; 4], Base)], shapes: &[Shape], g: f64, bounds: &mut [[Option<Bound>; 2]]) {
    let feet = |k: usize| sites[k].2.inner.map(|v| shapes[k].turned(target.point(v)));
    // How far from the corner, the two walls and their ends there, the plane each goes to.
    type Corner = (f64, [(usize, usize); 2], [Bound; 2]);
    let mut found: Vec<Corner> = Vec::new();
    for a in 0..sites.len() {
        for b in a + 1..sites.len() {
            let ([a0, a1], [b0, b1]) = (feet(a), feet(b));
            let (sa, sb) = (&shapes[a], &shapes[b]);
            let n = sa.inner_normal;
            let size = [a0, a1, b0, b1].iter().flat_map(|p| p.to_array()).fold(1.0, |m: f64, x| m.max(x.abs()));
            let turned_apart = n.cross(sb.inner_normal);
            let one_plane = within_rounding(turned_apart.dot(turned_apart).sqrt(), 1.0) && within_rounding((b0 - a0).dot(n), size);
            let across = sa.w.cross(sb.w);
            if !one_plane || within_rounding(across.dot(across).sqrt(), 1.0) {
                continue;
            }
            let (na, nb) = (n.cross(sa.w), n.cross(sb.w));
            let meet = |p: Vec3, q: Vec3| solve([n, na, nb], [p.dot(n), p.dot(na), q.dot(nb)]);
            let (Some(foot), Some(free)) = (meet(a0, b0), meet(a0 + sa.along * sa.wall, b0 + sb.along * sb.wall)) else { continue };
            let nearer = |[p, q]: [Vec3; 2]| {
                let (dp, dq) = (foot - p, foot - q);
                if dq.dot(dq) < dp.dot(dp) { (1, dq.dot(dq)) } else { (0, dp.dot(dp)) }
            };
            let ((ea, da), (eb, db)) = (nearer([a0, a1]), nearer([b0, b1]));
            let mut mu = unit(n.cross(free - foot));
            // `a`'s foot lies on its side of the plane.
            if ((a0 + a1) * 0.5 - foot).dot(mu) > 0.0 {
                mu = mu * -1.0;
            }
            let at = [Bound { at: free - mu * (0.5 * g), normal: mu }, Bound { at: free + mu * (0.5 * g), normal: mu * -1.0 }];
            found.push((da.max(db), [(a, ea), (b, eb)], at));
        }
    }
    found.sort_by(|x, y| x.0.total_cmp(&y.0));
    for (_, [(a, ea), (b, eb)], [ba, bb]) in found {
        if bounds[a][ea].is_none() && bounds[b][eb].is_none() {
            bounds[a][ea] = Some(ba);
            bounds[b][eb] = Some(bb);
        }
    }
}

/// The inner face of the wall of a flange whose side is face `x`, beyond the end `s` of the
/// width of what `base` stands on: a point of it and its outward normal, back toward that end.
/// Of the planes beside `x`, a wall's two faces face away from each other; the nearest such inner
/// face; none when there is none.
fn wall_beside(target: &Body, x: Ix, base: &Base, s: usize) -> Option<(Vec3, Vec3)> {
    let he = |h: Ix| target.half_edges()[h as usize];
    let Surface::Plane(side) = target.faces()[x as usize].surface else { return None };
    let planes: Vec<Plane> = target
        .face_loops(x)
        .flat_map(|l| target.loop_half_edges(l))
        .filter_map(|h| match target.faces()[target.loops()[he(he(h).twin).loop_ix as usize].face as usize].surface {
            Surface::Plane(p) if !parallel(p.normal, side.normal) => Some(p),
            _ => None,
        })
        .collect();
    // From the corner on, past the end: the inner face faces back toward it.
    let corner = target.point(base.inner[s]);
    let w = unit(target.point(base.inner[1]) - target.point(base.inner[0]));
    let past = if s == 0 { w * -1.0 } else { w };
    planes
        .iter()
        .filter(|p| planes.iter().any(|q| parallel(q.normal, p.normal) && q.normal.dot(p.normal) < 0.0))
        .filter_map(|p| {
            let n = unit(p.normal);
            let gap = (corner - p.origin).dot(n);
            (n.dot(past) < 0.0 && gap > 0.0).then_some((gap, p.origin, n))
        })
        .min_by(|a, b| a.0.total_cmp(&b.0))
        .map(|(_, at, n)| (at, n))
}

/// The point on the planes `x · rows[k] = rhs[k]`; none when they share no single point.
fn solve(rows: [Vec3; 3], rhs: [f64; 3]) -> Option<Vec3> {
    let [a, b, c] = rows;
    let det = a.dot(b.cross(c));
    if det == 0.0 || !det.is_finite() {
        return None;
    }
    Some((b.cross(c) * rhs[0] + c.cross(a) * rhs[1] + a.cross(b) * rhs[2]) * (1.0 / det))
}

/// Whether flanges `a` and `b` of `target` may meet at a corner, left
/// open: both on ends, the corner convex, neither leaning in over the sheet past a right angle
/// (each then stays off the other's way, beyond its own edge).
fn open_corner(target: &Body, a: &Base, b: &Base, angle: f64) -> Result<(), BrepError> {
    let later = |m: &str| Err(BrepError::Unsupported(format!("flanges meeting at a corner: {m} come later")));
    if !(a.end && b.end) {
        return later("a flange along a wall and one on an end");
    }
    if angle > 90.0 {
        return later("flanges turning past a right angle");
    }
    // Convex: `b`'s far corner lies on the sheet's side of `a`'s end, the side away from where `a`
    // leaves the sheet.
    let p = |v: Ix| target.point(v);
    let plane = (p(a.inner[0]), p(a.inner[1]), p(a.outer[0]));
    let side = |q: Vec3| orient3d_coords(plane.0, plane.1, plane.2, q);
    let shared = |v: Ix| [a.inner, a.outer].iter().flatten().any(|&w| w == v);
    let far = [b.inner, b.outer].into_iter().flatten().find(|&v| !shared(v));
    let inside = far.map(|v| side(p(v)));
    if inside.is_none_or(|s| s == Sign::Zero || s == side(plane.0 + a.out)) {
        return later("flanges at an inner corner");
    }
    Ok(())
}

/// The end whose long edge is `e`: the face, `e`'s half-edge on it, and its two long half-edges.
fn end_by_edge(target: &Body, e: TopoId) -> Result<(Ix, Ix, [Ix; 2]), BrepError> {
    let he = |h: Ix| target.half_edges()[h as usize];
    let face_of = |h: Ix| target.loops()[he(h).loop_ix as usize].face;
    let k = target.edges().binary_search_by_key(&e, |x| x.id).map_err(|_| BrepError::LostReference(format!("edge {e:?} is not in the body")))?;
    let h = target.edges()[k].half;
    // Across the thickness: a face of the sheet's sides (a rectangular plate's) is no end.
    let sides = crate::unfold::side_faces(target).unwrap_or_default();
    [h, he(h).twin]
        .into_iter()
        .filter(|&g| !sides.contains(&face_of(g)))
        .find_map(|g| end_of(target, face_of(g)).filter(|(long, _)| long.contains(&g)).map(|(long, _)| (face_of(g), g, long)))
        .ok_or_else(|| BrepError::Unsupported("flange: a width on a wall's edge along the sheet's side comes later".into()))
}

/// The ID of the end whose long edge is `e`.
fn end_face(target: &Body, e: TopoId) -> Result<TopoId, BrepError> {
    end_by_edge(target, e).map(|(f, ..)| target.faces()[f as usize].id)
}

/// `target` with the end `end_id` divided across at the flange's width `width`: a
/// piece at each end (where the width leaves one) and the middle, which keeps the end's ID; with
/// `notch`, a bend relief cut into the sheet beside each end of the middle that is not the edge's
/// end; with `flush` (the bend's radius and angle), the middle set back into the sheet so that
/// the flange's outer faces meet the sheet's at the edge (D229, the bend flush), the reliefs from
/// there on.
fn narrowed(
    feature: TopoId,
    target: &Body,
    end_id: TopoId,
    width: Width,
    notch: Option<Notch>,
    flush: Option<(f64, f64)>,
) -> Result<Body, BrepError> {
    let he = |h: Ix| target.half_edges()[h as usize];
    let face_of = |h: Ix| target.loops()[he(h).loop_ix as usize].face;
    let f =
        target.faces().binary_search_by_key(&end_id, |x| x.id).map_err(|_| BrepError::LostReference(format!("end {end_id:?} is not in the body")))?
            as Ix;
    let ([l0, l1], _) = end_of(target, f).ok_or_else(|| BrepError::Unsupported("flange: a width on what is no end of the sheet".into()))?;
    if target.edge_curve(&target.edges()[he(l0).edge as usize]).is_some() || target.edge_curve(&target.edges()[he(l1).edge as usize]).is_some() {
        return Err(BrepError::Unsupported("flange: a width on a curved end".into()));
    }
    let Surface::Plane(end) = target.faces()[f as usize].surface else {
        return Err(BrepError::Unsupported("flange: a width on a curved end".into()));
    };
    let id = |v: Ix| target.vertices()[v as usize].id;
    // Round the end: A0 → A1 along one long edge, C0 → C1 back along the other; C1 across from A0.
    let (a0, a1) = (he(l0).origin, he(target.next(l0)).origin);
    let (c0, c1) = (he(l1).origin, he(target.next(l1)).origin);
    let length = (target.point(a1) - target.point(a0)).dot(target.point(a1) - target.point(a0)).sqrt();
    let (sa, sb) = width.span(length)?;
    // The edge's start: its end with the lower vertex ID; along A0 → A1 from A0.
    let (sa, sb) = if id(a0) < id(a1) { (sa, sb) } else { (length - sb, length - sa) };
    let u = (target.point(a1) - target.point(a0)) * (1.0 / length);
    // Into the sheet, square to the end.
    let inward = unit(end.normal) * -1.0;
    let (low, high) = (sa > 0.0, sb < length);
    let across = target.point(c1) - target.point(a0) - u * (target.point(c1) - target.point(a0)).dot(u);
    let t = across.dot(across).sqrt();
    // Flush: back by where the outer faces of the sheet and the flange meet, `(r + t)·tan(a/2)`
    // (at a right angle exactly `r + t`).
    let recess = flush.map_or(0.0, |(r, a)| (r + t) * if a == 90.0 { 1.0 } else { libm::tan(0.5 * a.to_radians()) });
    // Flush and narrower than its edge, the bend would sit against the sheet's rest beside it:
    // only a relief makes room (as bending it does).
    if recess > 0.0 && notch.is_none() && (low || high) {
        return Err(BrepError::Unsupported("a flush flange narrower than its edge takes a bend relief".into()));
    }
    if let Some(n) = notch {
        if !(n.width > 0.0 && n.depth > 0.0 && n.width.is_finite() && n.depth.is_finite()) || (n.round && n.depth < 0.5 * n.width) {
            return Err(BrepError::Degenerate(format!("a bend relief {} wide and {} deep", n.width, n.depth)));
        }
        if (low && sa - n.width <= 0.0) || (high && sb + n.width >= length) {
            return Err(BrepError::Degenerate("a bend relief reaches the edge's end: offset the flange further".into()));
        }
    }
    let mut vertices = target.vertices().to_vec();
    let key = target.faces()[f as usize].id;
    // A new vertex `k` at `point`; the point `s` along the edge from `from`, `deep` into the sheet.
    let mut put = |k: u64, point: Vec3| -> TopoId {
        let v = TopoId::derive("flange.width").id(feature).id(key).index(k).finish();
        vertices.push(Vertex { id: v, point });
        v
    };
    let pos = |from: Ix, s: f64, deep: f64| target.point(from) + u * s + inward * deep;
    // On A0 → A1 at `s`, and across on C0 → C1 (from C1, across from A0), set back by `recess`.
    // At the edge's end, flush, the sheet's corner moves: where the faces beside it meet the
    // set-back end.
    let plane_of = |x: Ix| match target.faces()[x as usize].surface {
        Surface::Plane(p) => Ok((unit(p.normal), p.origin)),
        _ => Err(BrepError::Unsupported("flange: a flush bend beside a curved face".into())),
    };
    let set_back = (target.point(a0) + inward * recess).dot(unit(end.normal));
    let (top, bottom) = (plane_of(face_of(he(l0).twin))?, plane_of(face_of(he(l1).twin))?);
    let corner = |side: Ix, sheet: (Vec3, Vec3)| -> Result<Vec3, BrepError> {
        let side = plane_of(side)?;
        solve([sheet.0, side.0, unit(end.normal)], [sheet.1.dot(sheet.0), side.1.dot(side.0), set_back])
            .ok_or_else(|| BrepError::Degenerate("flange: the sheet's side runs along the end".into()))
    };
    let (side_a, side_b) = (face_of(he(target.next(l1)).twin), face_of(he(target.next(l0)).twin));
    let mut moved: Vec<(TopoId, TopoId)> = Vec::new();
    let mut ends = |k: u64, inner: bool, s: f64, (v, w): (Ix, Ix), side: Ix| -> Result<[TopoId; 2], BrepError> {
        if inner {
            return Ok([put(k, pos(a0, s, recess)), put(k + 1, pos(c1, s, recess))]);
        }
        if recess == 0.0 {
            return Ok([id(v), id(w)]);
        }
        let ids = [put(k, corner(side, top)?), put(k + 1, corner(side, bottom)?)];
        moved.extend([(id(v), ids[0]), (id(w), ids[1])]);
        Ok(ids)
    };
    let [pa, qa] = ends(0, low, sa, (a0, c1), side_a)?;
    let [pb, qb] = ends(2, high, sb, (a1, c0), side_b)?;
    // Each relief: on both lines, its corner on the edge away from the flange, its two corners
    // deep (the side at the flange, then away; round: the arc's ends); its depth from the bend
    // line, which a flush bend sets back.
    let deep = recess + notch.map_or(0.0, |n| if n.round { n.depth - 0.5 * n.width } else { n.depth });
    let mut relief = |k: u64, s_near: f64, s_far: f64| -> [[TopoId; 3]; 2] {
        [(a0, 0), (c1, 1)].map(|(from, l)| {
            let k = 10 + 6 * k + 3 * l;
            [put(k, pos(from, s_far, 0.0)), put(k + 1, pos(from, s_near, deep)), put(k + 2, pos(from, s_far, deep))]
        })
    };
    let w = notch.map_or(0.0, |n| n.width);
    let lows = (notch.is_some() && low).then(|| relief(0, sa, sa - w));
    let highs = (notch.is_some() && high).then(|| relief(1, sb, sb + w));
    let mut faces: Vec<FaceSpec> = Vec::new();
    let surface = target.faces()[f as usize].surface;
    let piece = |n: u64| TopoId::derive("sheet.divided").id(key).index(n).finish();
    if low {
        let (p, q) = lows.map_or((pa, qa), |[p, q]| (p[0], q[0]));
        faces.push(FaceSpec { id: piece(0), surface, loops: vec![vec![id(a0), p, q, id(c1)]] });
    }
    let middle = if recess > 0.0 {
        Surface::Plane(Plane { origin: vertices.iter().find(|x| x.id == pa).map_or(end.origin, |x| x.point), normal: end.normal })
    } else {
        surface
    };
    faces.push(FaceSpec { id: key, surface: middle, loops: vec![vec![pa, pb, qb, qa]] });
    if high {
        let (p, q) = highs.map_or((pb, qb), |[p, q]| (p[0], q[0]));
        faces.push(FaceSpec { id: piece(1), surface, loops: vec![vec![p, id(a1), id(c0), q]] });
    }
    // The faces beside the long edges run them the other way: the new vertices in between.
    let (across0, across1) = (face_of(he(l0).twin), face_of(he(l1).twin));
    let inserted = |x: Ix, from: TopoId, to: TopoId, run: &[TopoId]| -> FaceSpec {
        let mut spec = target.face_spec(x);
        for l in &mut spec.loops {
            let n = l.len();
            if let Some(k) = (0..n).find(|&k| (l[k], l[(k + 1) % n]) == (from, to)) {
                for (j, v) in run.iter().enumerate() {
                    l.insert(k + 1 + j, *v);
                }
            }
        }
        spec
    };
    // Along A1 → A0 (down `s`): the high relief from its far side in, the middle, the low one out.
    let mut run0: Vec<TopoId> = Vec::new();
    if high {
        run0.extend(highs.map_or(vec![], |[p, _]| vec![p[0], p[2], p[1]]));
        run0.push(pb);
    }
    if low {
        run0.push(pa);
        run0.extend(lows.map_or(vec![], |[p, _]| vec![p[1], p[2], p[0]]));
    }
    // Along C1 → C0 (up `s`): the low relief in, the middle, the high one out.
    let mut run1: Vec<TopoId> = Vec::new();
    if low {
        run1.extend(lows.map_or(vec![], |[_, q]| vec![q[0], q[2], q[1]]));
        run1.push(qa);
    }
    if high {
        run1.push(qb);
        run1.extend(highs.map_or(vec![], |[_, q]| vec![q[1], q[2], q[0]]));
    }
    faces.push(inserted(across0, id(a1), id(a0), &run0));
    faces.push(inserted(across1, id(c1), id(c0), &run1));
    let mut curves = target.curve_specs();
    // The reliefs' walls: the side away from the flange, the bottom, the side at it.
    let point = |v: TopoId| vertices.iter().find(|x| x.id == v).map_or(Vec3::ZERO, |x| x.point);
    let across = unit(across);
    for (k, (cut, near, toward)) in [(lows, (pa, qa), 1.0), (highs, (pb, qb), -1.0)].into_iter().enumerate() {
        let (Some([p, q]), Some(n)) = (cut, notch) else { continue };
        let face = |tag: &str, surface: Surface, ids: Vec<TopoId>, outward: Vec3| -> FaceSpec {
            let pts: Vec<Vec3> = ids.iter().map(|&v| point(v)).collect();
            let normal = (0..pts.len()).fold(Vec3::ZERO, |s, i| s + pts[i].cross(pts[(i + 1) % pts.len()]));
            let ids = if normal.dot(outward) < 0.0 { ids.into_iter().rev().collect() } else { ids };
            FaceSpec { id: TopoId::derive(tag).id(feature).id(key).index(k as u64).finish(), surface, loops: vec![ids] }
        };
        // Away from the flange `toward` is −u at the low end: the far side faces the slot, +u.
        let far = u * toward;
        faces.push(face("flange.relief.far", Surface::Plane(Plane { origin: point(p[0]), normal: far }), vec![p[0], p[2], q[2], q[0]], far));
        faces.push(face(
            "flange.relief.near",
            Surface::Plane(Plane { origin: point(p[1]), normal: far * -1.0 }),
            vec![p[1], near.0, near.1, q[1]],
            far * -1.0,
        ));
        let up = inward * -1.0;
        let bottom = if n.round {
            // A half circle from the side at the flange to the far side, through the deepest point.
            let c = (point(p[1]) + point(p[2])) * 0.5;
            let axis = unit((point(p[1]) - c).cross(inward));
            for (a, b) in [(p[1], p[2]), (q[1], q[2])] {
                let centre = (point(a) + point(b)) * 0.5;
                curves.push(CurveSpec { from: a, to: b, curve: Curve::Arc(Arc { center: centre, axis, radius: 0.5 * n.width }) });
            }
            Surface::Cylinder(Cylinder { origin: c, axis: across, radius: 0.5 * n.width, convex: false })
        } else {
            Surface::Plane(Plane { origin: point(p[1]), normal: up })
        };
        faces.push(face("flange.relief.bottom", bottom, vec![p[1], p[2], q[2], q[1]], up));
    }
    faces.extend((0..target.faces().len() as Ix).filter(|&x| x != f && x != across0 && x != across1).map(|x| target.face_spec(x)));
    // Flush at the edge's end: the sheet's corner moved, in every face it was a corner of.
    for face in &mut faces {
        for l in &mut face.loops {
            for v in l.iter_mut() {
                if let Some((_, new)) = moved.iter().find(|(old, _)| old == v) {
                    *v = *new;
                }
            }
        }
    }
    vertices.retain(|x| !moved.iter().any(|(old, _)| *old == x.id));
    build_with_curves(vertices, faces, curves)
}

/// What a flange stands on: the edge of the sheet's side it turns toward (`inner`, at the start
/// and end of its width) and the one across the thickness (`outer`), the way it leaves the sheet
/// (`out`), the side it turns to (`up`), the key its IDs come from, the faces it replaces with
/// what stays of them, and at each end of the width the face its side joins, if any.
struct Base {
    key: TopoId,
    inner: [Ix; 2],
    outer: [Ix; 2],
    out: Vec3,
    up: Vec3,
    gone: Vec<Ix>,
    kept: Vec<FaceSpec>,
    joins: [Option<Ix>; 2],
    /// On an end of the sheet (not along a wall).
    end: bool,
    /// The faces beside its width's start and end where a wall of a flange made before may stand:
    /// an end's across its short edges, a side face's strip's the side face itself.
    beside: [Option<Ix>; 2],
}

/// The flange edge `e` of `body` asks for (see [`flanges`]); its sides go into no face of `taken`
/// (other flanges' ends).
fn base(body: &Body, e: TopoId, flip: bool, taken: &BTreeSet<TopoId>) -> Result<Base, BrepError> {
    let unsupported = |m: &str| BrepError::Unsupported(format!("flange: {m}"));
    let he = |h: Ix| body.half_edges()[h as usize];
    let face_of = |h: Ix| body.loops()[he(h).loop_ix as usize].face;
    let k = body.edges().binary_search_by_key(&e, |x| x.id).map_err(|_| BrepError::LostReference(format!("edge {e:?} is not in the body")))?;
    let h = body.edges()[k].half;
    let plane = |x: Ix| match body.faces()[x as usize].surface {
        Surface::Plane(p) => Some(p),
        _ => None,
    };
    let sides = crate::unfold::side_faces(body).unwrap_or_default();
    // An end's long edge: a flange on that end (a face across the thickness, not of the sheet's
    // sides, which a wall is).
    for g in [h, he(h).twin] {
        let f = face_of(g);
        if !sides.contains(&f)
            && let Some((long, short)) = end_of(body, f)
            && long.contains(&g)
        {
            return end_base(body, f, long, short, flip, taken);
        }
    }
    // A wall's edge on the sheet's side face: the strip of that face over the wall.
    let (wall_h, cap_h) = match (sides.contains(&face_of(h)), sides.contains(&face_of(he(h).twin))) {
        (true, false) => (h, he(h).twin),
        (false, true) => (he(h).twin, h),
        _ => return Err(unsupported("it stands on an edge of the sheet: of an end, or of a wall on the sheet's side")),
    };
    let (wall, cap) = (face_of(wall_h), face_of(cap_h));
    let (Some(pw), Some(pc)) = (plane(wall), plane(cap)) else { return Err(unsupported("it stands along a flat wall, on a flat side")) };
    if !side_face(body, cap, &sides) {
        return Err(unsupported("it stands on a wall's edge on the sheet's side face (one along its bends too, no hole)"));
    }
    // The wall's other side: of the sheet's sides along the side face, parallel to it, nearest
    // (a flange's own side, square to its wall, may be parallel too).
    let across = body
        .loop_half_edges(body.faces()[cap as usize].first_loop)
        .filter(|&g| g != cap_h)
        .filter_map(|g| {
            let x = face_of(he(g).twin);
            let p = plane(x)?;
            (x != wall && sides.contains(&x) && parallel(p.normal, pw.normal)).then(|| (g, (p.origin - pw.origin).dot(unit(pw.normal)).abs()))
        })
        .filter(|(_, d)| *d > 0.0)
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(g, _)| g)
        .ok_or_else(|| unsupported("the wall has no other side along the sheet's side face"))?;
    let (a0, a1) = (he(cap_h).origin, he(body.next(cap_h)).origin);
    let (b0, b1) = (he(across).origin, he(body.next(across)).origin);
    // The other side's ends, matched to this one's by nearness along the edge.
    let d = body.point(a1) - body.point(a0);
    let (b0, b1) = if (body.point(b0) - body.point(a0)).dot(d) < (body.point(b1) - body.point(a0)).dot(d) { (b0, b1) } else { (b1, b0) };
    // Inward, the way the walls beside it stand (D229, was outward in D205): toward the side of
    // the wall its bends turn to (the inner, concave one); a wall with no bend, toward the side
    // picked.
    let inward = !smooth_convex(body, wall);
    let toward_wall = inward != flip;
    let (inner, outer) = if toward_wall { ([a1, a0], [b1, b0]) } else { ([b1, b0], [a1, a0]) };
    // The same whichever edge of the wall is picked: its ends in the order of their IDs.
    let id_of = |v: Ix| body.vertices()[v as usize].id;
    let (inner, outer) = if id_of(inner[0]) > id_of(inner[1]) { ([inner[1], inner[0]], [outer[1], outer[0]]) } else { (inner, outer) };
    let up_face = if toward_wall { wall } else { face_of(he(across).twin) };
    let up = unit(plane(up_face).map_or(pw.normal, |p| p.normal));
    // The side face without the strip: its loop cut at the wall's two edges, each piece closed
    // across the thickness; a piece of two vertices is an end of the sheet already there.
    let spec = body.face_spec(cap);
    let l = &spec.loops[0];
    let id = |v: Ix| body.vertices()[v as usize].id;
    let at = |x: Ix, y: Ix| (0..l.len()).find(|&k| (l[k], l[(k + 1) % l.len()]) == (id(x), id(y)));
    let (Some(ka), Some(kb)) = (at(a0, a1), at(he(across).origin, he(body.next(across)).origin)) else {
        return Err(BrepError::Invalid("flange: the side face does not run along the wall".into()));
    };
    let run = |from: usize, to: usize| -> Vec<TopoId> {
        let mut out = vec![l[from % l.len()]];
        let mut k = from % l.len();
        while k != to % l.len() {
            k = (k + 1) % l.len();
            out.push(l[k]);
        }
        out
    };
    let mut kept = Vec::new();
    let mut joins = [None, None];
    let strip = [id(a0), id(a1), id(b0), id(b1)];
    let first = l.iter().find(|v| !strip.contains(v)).copied();
    for piece in [run(ka + 1, kb), run(kb + 1, ka)] {
        if piece.len() == 2 {
            // The sheet's end across the thickness: the flange's side meets it there.
            let (x, y) = (piece[0], piece[1]);
            let g = body.loop_half_edges(body.faces()[cap as usize].first_loop).find(|&g| id(he(g).origin) == x && id(he(body.next(g)).origin) == y);
            if let Some(g) = g {
                let end = face_of(he(g).twin);
                let s = usize::from(x == id(inner[1]) || y == id(inner[1]));
                joins[s] = plane(end).filter(|p| parallel(p.normal, d) && !taken.contains(&body.faces()[end as usize].id)).map(|_| end);
            }
            continue;
        }
        let face = if first.is_some_and(|v| piece.contains(&v)) {
            spec.id
        } else {
            TopoId::derive("sheet.divided").id(spec.id).id(piece.iter().min().copied().unwrap_or(spec.id)).finish()
        };
        kept.push(FaceSpec { id: face, surface: spec.surface, loops: vec![piece] });
    }
    let mut corners = strip;
    corners.sort();
    let key = TopoId::derive("flange.side").id(corners[0]).id(corners[3]).finish();
    Ok(Base { key, inner, outer, out: unit(pc.normal), up, gone: vec![cap], kept, joins, end: false, beside: [Some(cap); 2] })
}

/// Whether face `y` is a side face of the sheet: across its thickness, along its bends too (a cut's
/// wall in one wall is not), one loop. `sides` are the faces of the sheet's two sides.
fn side_face(body: &Body, y: Ix, sides: &BTreeSet<Ix>) -> bool {
    let he = |h: Ix| body.half_edges()[h as usize];
    let face = &body.faces()[y as usize];
    face.loop_count == 1
        && body.loop_half_edges(face.first_loop).any(|h| {
            let x = body.loops()[he(he(h).twin).loop_ix as usize].face;
            sides.contains(&x) && matches!(body.faces()[x as usize].surface, Surface::Cylinder(_))
        })
}

/// Whether face `x` of `body` goes on smoothly into a convex bend: the outer side of the sheet.
fn smooth_convex(body: &Body, x: Ix) -> bool {
    let he = |h: Ix| body.half_edges()[h as usize];
    body.loop_half_edges(body.faces()[x as usize].first_loop).any(|h| {
        let y = body.loops()[he(he(h).twin).loop_ix as usize].face;
        matches!(body.faces()[y as usize].surface, Surface::Cylinder(c) if c.convex)
            && body.faces()[x as usize]
                .surface
                .normal_at(body.point(he(h).origin))
                .dot(body.faces()[y as usize].surface.normal_at(body.point(he(h).origin)))
                > 0.5
    })
}

/// A flange on end face `f`, its long half-edges `long` and short ones `short`: toward the
/// sheet's front side, or the other.
fn end_base(body: &Body, f: Ix, long: [Ix; 2], short: [Ix; 2], flip: bool, taken: &BTreeSet<TopoId>) -> Result<Base, BrepError> {
    let unsupported = |m: &str| BrepError::Unsupported(format!("flange: {m}"));
    let he = |h: Ix| body.half_edges()[h as usize];
    let beside = |h: Ix| body.loops()[he(he(h).twin).loop_ix as usize].face;
    let plane = |x: Ix| match body.faces()[x as usize].surface {
        Surface::Plane(p) => Some(p),
        _ => None,
    };
    let Surface::Plane(end) = body.faces()[f as usize].surface else { return Err(unsupported("it stands on a flat end face")) };
    let front = crate::unfold::sheet_sides(body).and_then(|s| crate::unfold::side_of(body, s[0])).unwrap_or_default();
    let on_front = |x: Ix| front.contains(&x);
    let k = usize::from(!on_front(beside(long[0])) == !flip);
    let (hi, ho) = (long[k], long[1 - k]);
    let up = unit(plane(beside(hi)).ok_or_else(|| unsupported("the sheet's sides at the end are flat"))?.normal);
    let (i0, i1) = (he(hi).origin, he(body.next(hi)).origin);
    // The outer edge runs the other way round the face.
    let (o1, o0) = (he(ho).origin, he(body.next(ho)).origin);
    let w = body.point(i1) - body.point(i0);
    let starts = |h: Ix| he(h).origin == i0 || he(body.next(h)).origin == i0;
    let short = if starts(short[0]) { short } else { [short[1], short[0]] };
    let joins =
        short.map(|h| Some(beside(h)).filter(|&x| plane(x).is_some_and(|p| parallel(p.normal, w)) && !taken.contains(&body.faces()[x as usize].id)));
    Ok(Base {
        key: body.faces()[f as usize].id,
        inner: [i0, i1],
        outer: [o0, o1],
        out: unit(end.normal),
        up,
        gone: vec![f],
        kept: Vec::new(),
        joins,
        end: true,
        beside: short.map(|h| Some(beside(h))),
    })
}

/// The faces of sheet metal `body` a flange may stand on: across the thickness (not on the
/// sheet's two sides), flat, four edges, the long two on parallel flat sides of the sheet. What
/// the window lights up for a flange; the flange itself checks the same.
pub fn flange_ends(body: &Body) -> Vec<TopoId> {
    let Ok(on_sides) = crate::unfold::side_faces(body) else { return Vec::new() };
    (0..body.faces().len() as Ix).filter(|x| !on_sides.contains(x) && end_of(body, *x).is_some()).map(|x| body.faces()[x as usize].id).collect()
}

/// The edges a flange is picked by, as in Inventor (D205): each end's two long edges, and each
/// flat wall's edges on the sheet's flat side faces; each with its ID and its ends.
pub fn flange_edges(body: &Body) -> Vec<(TopoId, [Vec3; 2])> {
    let he = |h: Ix| body.half_edges()[h as usize];
    let Ok(on_sides) = crate::unfold::side_faces(body) else { return Vec::new() };
    let flat = |x: Ix| matches!(body.faces()[x as usize].surface, Surface::Plane(_));
    let mut out: Vec<(TopoId, [Vec3; 2])> = Vec::new();
    for &x in on_sides.iter().filter(|&&x| flat(x)) {
        for h in body.loop_half_edges(body.faces()[x as usize].first_loop) {
            let y = body.loops()[he(he(h).twin).loop_ix as usize].face;
            if on_sides.contains(&y) || !flat(y) {
                continue;
            }
            let e = &body.edges()[he(h).edge as usize];
            // A side face's edge along a wall, or an end's long edge.
            let end = end_of(body, y).is_some_and(|(long, _)| long.contains(&he(h).twin));
            if end || side_face(body, y, &on_sides) {
                out.push((e.id, [body.point(he(h).origin), body.point(he(body.next(h)).origin)]));
            }
        }
    }
    out.sort_by_key(|e| e.0);
    out.dedup_by_key(|e| e.0);
    out
}

/// Face `f` of `body` as an end a flange stands on: its two long half-edges and two short ones;
/// none if it is not one.
fn end_of(body: &Body, f: Ix) -> Option<([Ix; 2], [Ix; 2])> {
    let he = |h: Ix| body.half_edges()[h as usize];
    let face = &body.faces()[f as usize];
    if !matches!(face.surface, Surface::Plane(_)) || face.loop_count != 1 {
        return None;
    }
    let halves: Vec<Ix> = body.loop_half_edges(face.first_loop).collect();
    if halves.len() != 4 {
        return None;
    }
    let length = |h: Ix| {
        let d = body.point(he(body.next(h)).origin) - body.point(he(h).origin);
        d.dot(d)
    };
    let k = (0..4).max_by(|&a, &b| length(halves[a]).total_cmp(&length(halves[b])))?;
    let (long, short) = ([halves[k], halves[(k + 2) % 4]], [halves[(k + 1) % 4], halves[(k + 3) % 4]]);
    let plane = |h: Ix| match body.faces()[body.loops()[he(he(h).twin).loop_ix as usize].face as usize].surface {
        Surface::Plane(p) => Some(p.normal),
        _ => None,
    };
    parallel(plane(long[0])?, plane(long[1])?).then_some((long, short))
}

/// Whether `x`, worked out from coordinates up to `size`, is no more than the bits a bend's turn
/// and a plane's normal leave (D229).
fn within_rounding(x: f64, size: f64) -> bool {
    x.abs() <= 64.0 * f64::EPSILON * size
}

fn unit(v: Vec3) -> Vec3 {
    v * (1.0 / v.dot(v).sqrt())
}

/// `v` turned by `angle` (radians, as sine and cosine) about unit `axis`.
fn turned(v: Vec3, axis: Vec3, (sn, cs): (f64, f64)) -> Vec3 {
    v * cs + axis.cross(v) * sn + axis * (axis.dot(v) * (1.0 - cs))
}

/// Where a flange's bend and wall go (see [`grow`]): the unit edge `w`, the axis it turns about,
/// the turn, the bend's axis point `c_in`, the wall's way up `along` and its height above the
/// bend, the sheet's thickness, and the inner face's outward normal.
struct Shape {
    w: Vec3,
    axis: Vec3,
    turn: (f64, f64),
    c_in: Vec3,
    along: Vec3,
    wall: f64,
    t: f64,
    inner_normal: Vec3,
}

impl Shape {
    fn of(target: &Body, base: &Base, dims: FlangeDims) -> Result<Self, BrepError> {
        let ([i0, i1], o0, n, up) = (base.inner, base.outer[0], base.out, base.up);
        let w = unit(target.point(i1) - target.point(i0));
        let t = (target.point(i0) - target.point(o0)).dot(up);
        if t.is_nan() || t <= 0.0 {
            return Err(BrepError::Invalid("flange: the inner edge is not inside the sheet's thickness".into()));
        }
        let (r, a) = (dims.radius, dims.angle.to_radians());
        let turn = libm::sincos(a);
        // Turning `n` toward `up`: about `w` or `-w`.
        let axis = if w.cross(n).dot(up) > 0.0 { w } else { w * -1.0 };
        let along = turned(n, axis, turn);
        // At a right angle exactly `r + t` either way: tan 45° and cos 90° in floating point are
        // not quite 1 and 0, and a flange as high as its bend would come out a sliver instead of
        // refused. Square to the sheet: the bend rises `R − c·cos a` to the wall's highest face
        // (`c` the inner radius leaning out, the outer leaning in), the wall `sin a` a unit.
        let big = r + t;
        let (wall, bend) = if dims.angle == 90.0 {
            (dims.height - big, big)
        } else if dims.along {
            let set_back = big * libm::tan(0.5 * a);
            (dims.height - set_back, set_back)
        } else {
            let (sn, cs) = turn;
            let rise = big - if cs >= 0.0 { r } else { big } * cs;
            ((dims.height - rise) / sn, rise)
        };
        if dims.to.top.is_none() && (wall.is_nan() || wall <= 0.0) {
            return Err(BrepError::Degenerate(format!("a flange {} high is no higher than its bend ({bend:.3})", dims.height)));
        }
        // One axis for the bend's both sides, through the inner edge's start `r` up: both sides
        // turn about it, so the two cylinders are coaxial exactly (made apart, they part in the
        // last bits, and the bend has no face across it).
        let c_in = target.point(i0) + up * r;
        Ok(Self { w, axis, turn, c_in, along, wall, t, inner_normal: turned(up, axis, turn) })
    }

    /// Where point `p` of the end goes as the bend turns it: where the bend ends, at the wall.
    fn turned(&self, p: Vec3) -> Vec3 {
        self.c_in + turned(p - self.c_in, self.axis, self.turn)
    }

    /// The wall's inner and outer faces: a point of each, and the outer face's outward normal.
    fn faces(&self, target: &Body, base: &Base) -> (Vec3, Vec3, Vec3) {
        (self.turned(target.point(base.inner[0])), self.turned(target.point(base.outer[0])), self.inner_normal * -1.0)
    }
}

/// A plane a side of a flange's wall goes to (D229): the wall keeps what lies where
/// `(x − at) · normal ≤ 0`.
#[derive(Clone, Copy, Debug)]
struct Bound {
    at: Vec3,
    normal: Vec3,
}

/// The flange `base` asks for on `target`: a bend and a wall sewn to its edges; the wall's side at
/// each end of the width square to it, or gone to `bounds` there (D229): the wall grows or is cut
/// along the edge to the plane, the bend not, a step across the thickness between them. The planes
/// `dims.to` names take their ends' place (`way`: the width's start to its end), and its free edge
/// goes to its own, the wall's height then that alone.
fn grow(feature: TopoId, target: &Body, base: Base, dims: FlangeDims, bounds: [Option<Bound>; 2], way: Vec3) -> Result<Body, BrepError> {
    let shape = Shape::of(target, &base, dims)?;
    let Shape { w, axis, c_in, along, wall, t, inner_normal, .. } = shape;
    let Base { key, inner: [i0, i1], outer: [o0, o1], up: base_up, gone, kept, joins, .. } = base;
    let (r, a) = (dims.radius, dims.angle.to_radians());
    let along_edge = || BrepError::Unsupported("flange: a wall's side to a plane along its edge".into());
    let mut bounds = bounds;
    // The width's start: at the wall's first end or last, as the width runs along `w` or against.
    let start = usize::from(way.dot(w) < 0.0);
    if let Some(l) = &dims.to.start {
        bounds[start] = Some(l.bound(if start == 0 { w * -1.0 } else { w }).ok_or_else(along_edge)?);
    }
    if let Some(l) = &dims.to.end {
        bounds[1 - start] = Some(l.bound(if start == 0 { w } else { w * -1.0 }).ok_or_else(along_edge)?);
    }
    let top = match &dims.to.top {
        // At a right angle exactly `up`: `along` has the bits of cos 90° in it, and a plane along
        // the wall would pass for one far off.
        Some(l) => Some(
            l.bound(if dims.angle == 90.0 { base_up } else { along })
                .ok_or_else(|| BrepError::Unsupported("flange: a wall's free edge to a plane along the wall".into()))?,
        ),
        None => None,
    };
    // How high a side's line is read at: the wall's height, without a plane to take it from.
    let height = if top.is_some() { 1.0 } else { wall };
    let id = |tag: &str, k: u64| TopoId::derive(tag).id(feature).id(key).index(k).finish();
    let mut vertices: Vec<Vertex> = target.vertices().to_vec();
    // Along the edge from `p` to the plane of `b`.
    let reach = |b: &Bound, p: Vec3| -> Result<f64, BrepError> {
        let d = w.dot(b.normal);
        if d == 0.0 {
            return Err(along_edge());
        }
        Ok((b.at - p).dot(b.normal) / d)
    };
    // From each corner of the end, about the bend's axis: where the bend ends, and the wall's top;
    // with a bound at that end, also where the wall's foot meets it. Cut square to the wall, as a
    // laser cuts: both faces of the wall end alike, where the first of them meets the plane. For
    // the inner face and the outer at end `s`: the vertex at the bend's end, at the wall's foot on
    // the bound (itself when it is there), at the top. Short of the bend's end the foot lies on
    // that end's line, the bend's end beyond it bare.
    let mut side = |s: usize, vi: Ix, vo: Ix| -> Result<([[TopoId; 4]; 2], f64), BrepError> {
        let q = [vi, vo].map(|v| shape.turned(target.point(v)));
        let (up_side, foot) = match &bounds[s] {
            None => (0.0, 0.0),
            Some(b) => {
                let out = if s == 0 { -1.0 } else { 1.0 };
                // Of the two faces' reach, the nearer the edge's end.
                let nearer = |x: f64, y: f64| if out > 0.0 { x.min(y) } else { x.max(y) };
                let foot = nearer(reach(b, q[0])?, reach(b, q[1])?);
                let top = nearer(reach(b, q[0] + along * height)?, reach(b, q[1] + along * height)?);
                // A plane through the wall's foot (a plane through the sheet's own corner, an edge
                // flush with one) is there, whatever bits the bend's turn and the plane's normal
                // leave: as near as rounding reaches, no step of no width (D229).
                let size = [q[0], q[1], b.at].iter().flat_map(|p| p.to_array()).fold(0.0, |m: f64, x| m.max(x.abs()));
                (top, if within_rounding(foot, size) { 0.0 } else { foot })
            }
        };
        // Where the side's line meets the free edge's: along the edge and up the wall. Square to
        // the wall, as a side is: both faces end alike, where the first meets the plane.
        let (off, high) = match &top {
            None => (up_side, wall),
            Some(b) => {
                let d = along.dot(b.normal);
                let up = |p: Vec3| (b.at - p).dot(b.normal) / d;
                let (g, k, m) = (up(q[0]).min(up(q[1])), -w.dot(b.normal) / d, (up_side - foot) / height);
                let v = (g + k * foot) / (1.0 - k * m);
                if !(v > 0.0 && v.is_finite()) {
                    return Err(BrepError::Degenerate("flange: the plane of its free edge leaves the wall no height".into()));
                }
                (foot + m * v, v)
            }
        };
        let mut layer = |l: usize, v: Ix| -> [TopoId; 4] {
            let k = 2 * s as u64 + 4 * l as u64;
            let (vq, ve, vs) = (id("flange.v", k), id("flange.v", k + 1), id("flange.v", 8 + k));
            vertices.push(Vertex { id: vq, point: q[l] });
            if foot != 0.0 {
                vertices.push(Vertex { id: vs, point: q[l] + w * foot });
            }
            vertices.push(Vertex { id: ve, point: q[l] + along * high + w * off });
            [target.vertices()[v as usize].id, vq, if foot == 0.0 { vq } else { vs }, ve]
        };
        Ok(([layer(0, vi), layer(1, vo)], foot))
    };
    // Inner and outer, at the start and end of the width.
    let ([[ii0, ji0, fi0, ki0], [oo0, po0, fo0, qo0]], foot0) = side(0, i0, o0)?;
    let ([[ii1, ji1, fi1, ki1], [oo1, po1, fo1, qo1]], foot1) = side(1, i1, o1)?;
    let point = |v: TopoId| vertices.iter().find(|x| x.id == v).map_or(Vec3::ZERO, |x| x.point);
    if (bounds[0].is_some() || bounds[1].is_some() || top.is_some())
        && ((point(fi1) - point(fi0)).dot(w) <= 0.0 || (point(ki1) - point(ki0)).dot(w) <= 0.0)
    {
        return Err(BrepError::Degenerate("flange: the planes of its sides leave the wall no width".into()));
    }
    // Whether the wall's foot at each end stops short of the bend's end.
    let short = [foot0 > 0.0, foot1 < 0.0];
    // A loop of `ids` turning about `outward` by the right hand; a vertex twice in a row once.
    let oriented = |mut ids: Vec<TopoId>, outward: Vec3| -> Vec<TopoId> {
        ids.dedup();
        if ids.len() > 1 && ids.first() == ids.last() {
            ids.pop();
        }
        let p: Vec<Vec3> = ids.iter().map(|&v| point(v)).collect();
        let normal = (0..p.len()).fold(Vec3::ZERO, |s, k| s + p[k].cross(p[(k + 1) % p.len()]));
        if normal.dot(outward) < 0.0 {
            ids.reverse();
        }
        ids
    };
    let in_mid = c_in + turned(target.point(i0) - c_in, axis, libm::sincos(0.5 * a)) - c_in;
    let mut faces: Vec<FaceSpec> = Vec::new();
    let face = |tag: &str, k: u64, surface: Surface, loop_: Vec<TopoId>| FaceSpec { id: id(tag, k), surface, loops: vec![loop_] };
    faces.push(face(
        "flange.bend_in",
        0,
        Surface::Cylinder(Cylinder { origin: c_in, axis: w, radius: r, convex: false }),
        oriented(bend_loop([ii0, ii1, ji1, ji0], [fi0, fi1], short), in_mid * -1.0),
    ));
    faces.push(face(
        "flange.bend_out",
        0,
        Surface::Cylinder(Cylinder { origin: c_in, axis: w, radius: r + t, convex: true }),
        oriented(bend_loop([oo0, oo1, po1, po0], [fo0, fo1], short), in_mid),
    ));
    faces.push(face(
        "flange.wall_in",
        0,
        Surface::Plane(Plane { origin: point(ji0), normal: inner_normal }),
        oriented(vec![if short[0] { fi0 } else { ji0 }, if short[1] { fi1 } else { ji1 }, fi1, ki1, ki0, fi0], inner_normal),
    ));
    faces.push(face(
        "flange.wall_out",
        0,
        Surface::Plane(Plane { origin: point(po0), normal: inner_normal * -1.0 }),
        oriented(vec![if short[0] { fo0 } else { po0 }, if short[1] { fo1 } else { po1 }, fo1, qo1, qo0, fo0], inner_normal * -1.0),
    ));
    // To a plane the free edge may slant: its face square to the wall all the same.
    let end_normal = if top.is_some() {
        let n = unit(inner_normal.cross(point(ki1) - point(ki0)));
        if n.dot(along) < 0.0 { n * -1.0 } else { n }
    } else {
        along
    };
    faces.push(face(
        "flange.end",
        0,
        Surface::Plane(Plane { origin: point(ki0), normal: end_normal }),
        oriented(vec![ki0, ki1, qo1, qo0], end_normal),
    ));
    // The arcs of the bend at both sides, inner and outer, from the end face on.
    let arc = |c: Vec3, r: f64, from: TopoId, to: TopoId| {
        // Counterclockwise about `axis` from the end face.
        CurveSpec { from, to, curve: Curve::Arc(Arc { center: c, axis, radius: r }) }
    };
    let mut curves = target.curve_specs();
    // Each arc's centre on the axis, in its plane.
    let on_axis = |v: Ix| c_in + w * (target.point(v) - c_in).dot(w);
    curves.extend([
        arc(on_axis(i0), r, ii0, ji0),
        arc(on_axis(i1), r, ii1, ji1),
        arc(on_axis(o0), r + t, oo0, po0),
        arc(on_axis(o1), r + t, oo1, po1),
    ]);
    // The sides: into the face at that end of the width where it is square to the width, else
    // faces of their own; at a bound the bend's side, the step and the wall's side on the plane.
    let mut gone: BTreeSet<Ix> = gone.into_iter().collect();
    faces.extend(kept);
    let ends = [[ii0, ji0, fi0, ki0, qo0, fo0, po0, oo0], [ii1, ji1, fi1, ki1, qo1, fo1, po1, oo1]];
    for (s, (join, [ii, ji, fi, ki, qo, fo, po, oo])) in joins.iter().zip(ends).enumerate() {
        let outward = if s == 0 { w * -1.0 } else { w };
        let k = s as u64;
        let bounded = bounds[s].is_some();
        if bounded {
            if (fi, fo) != (ji, po) {
                // Under the wall grown past the bend, facing the sheet; on the bend's end the wall
                // stopped short of, facing up.
                let facing = if short[s] { along } else { along * -1.0 };
                faces.push(face(
                    "flange.step",
                    k,
                    Surface::Plane(Plane { origin: point(ji), normal: facing }),
                    oriented(vec![ji, fi, fo, po], facing),
                ));
            }
            // Square to the wall, facing past the end.
            let mut normal = unit((point(ki) - point(fi)).cross(point(fo) - point(fi)));
            if normal.dot(outward) < 0.0 {
                normal = normal * -1.0;
            }
            faces.push(face("flange.bound", k, Surface::Plane(Plane { origin: point(fi), normal }), oriented(vec![fi, ki, qo, fo], normal)));
        }
        let chain = if bounded { vec![ii, ji, po, oo] } else { vec![ii, ji, ki, qo, po, oo] };
        let Some(x) = *join else {
            faces.push(face("flange.side", k, Surface::Plane(Plane { origin: point(chain[0]), normal: outward }), oriented(chain, outward)));
            continue;
        };
        // That face's edge across the thickness runs from one of the flange's corners to the
        // other: the flange's outline goes in between, the same way.
        let mut spec = target.face_spec(x);
        let (a, b) = (chain[0], chain[chain.len() - 1]);
        for l in &mut spec.loops {
            let n = l.len();
            if let Some(k) = (0..n).find(|&k| (l[k], l[(k + 1) % n]) == (a, b) || (l[k], l[(k + 1) % n]) == (b, a)) {
                let run: Vec<TopoId> =
                    if l[k] == a { chain[1..chain.len() - 1].to_vec() } else { chain[1..chain.len() - 1].iter().rev().copied().collect() };
                for (j, v) in run.iter().enumerate() {
                    l.insert(k + 1 + j, *v);
                }
            }
        }
        faces.push(spec);
        gone.insert(x);
    }
    faces.extend((0..target.faces().len() as Ix).filter(|x| !gone.contains(x)).map(|x| target.face_spec(x)));
    build_with_curves(vertices, faces, curves)
}

/// A bend's face `corners` (the sheet's edge along it, then the wall's end back), the wall's feet
/// `feet` on that end's line where it stops `short` of the bend's end.
fn bend_loop(corners: [TopoId; 4], feet: [TopoId; 2], short: [bool; 2]) -> Vec<TopoId> {
    let [a, b, c, d] = corners;
    let mut l = vec![a, b, c];
    if short[1] {
        l.push(feet[1]);
    }
    if short[0] {
        l.push(feet[0]);
    }
    l.push(d);
    l
}
