//! Ribs rolled by a punch press tool, a ball or a roller, along a path on a flat wall (D250).
//!
//! The section across the path is two arcs on each side of the sheet, both tangent: a crown (the
//! tool's radius on the side it presses from, plus the thickness on the other) and a shoulder at
//! each foot (the die's radius on the far side, plus the thickness on the near one). The sheet
//! keeps its thickness, and its blank does not change: the forming is local. Along a segment the
//! section sweeps into cylinders, along an arc into tori, and at each end of the path it turns
//! half way about the end point: a ball's print, sphere and torus. All of it is exact in STEP.
//!
//! A rib is a local operation on the body: each side of the wall gets a hole along the foot, and
//! the rib's faces fill it. What a rib touches (an edge, a hole, another rib) is refused.

use datum_brep::{
    Arc, Body, BrepError, Curve, CurveSpec, Cylinder, FaceSpec, ProfileArc, ProfileLoop, Surface, TopoId, Torus, Vertex, build_with_curves,
};
use datum_math::{Vec2, Vec3};

use crate::cut::{Frame, outline};
use crate::sketch::{ChainPiece, Placed, open_chains_named};

/// A tool's section: the crown's radius (half the tool's diameter) on the side pressed from, how
/// far the sheet is pressed, and the radius of the die's shoulder.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RibTool {
    pub radius: f64,
    pub depth: f64,
    pub shoulder: f64,
    /// The shortest path the tool rolls.
    pub min_length: f64,
}

/// One rib op on a sheet, kept to be laid again when the sheet under it changes.
#[derive(Clone, Debug, PartialEq)]
pub struct Imprint {
    feature: TopoId,
    face: TopoId,
    frame: [Vec3; 4],
    chains: Vec<Vec<(String, ChainPiece)>>,
    tool: RibTool,
    flip: bool,
}

/// A sheet with ribs: the body as it looks, the body without them (what its blank, flanges and
/// cuts are made from), and the ribs to lay again on a changed one.
#[derive(Clone, Debug, PartialEq)]
pub struct Formed {
    pub body: Body,
    pub blank: Body,
    pub imprints: Vec<Imprint>,
}

fn unsupported(m: &str) -> BrepError {
    BrepError::Unsupported(format!("rib: {m}"))
}

fn degenerate(m: String) -> BrepError {
    BrepError::Degenerate(format!("rib: {m}"))
}

fn unit(v: Vec3) -> Vec3 {
    v * (1.0 / v.dot(v).sqrt())
}

fn sub(a: Vec2, b: Vec2) -> Vec2 {
    Vec2::new(a.x - b.x, a.y - b.y)
}

fn at(p: Vec2, d: Vec2, s: f64) -> Vec2 {
    Vec2::new(p.x + d.x * s, p.y + d.y * s)
}

/// The sketch's open chains as what `sketch` says of them: each must go smoothly (the sketch
/// states every joint tangent) and be no shorter than the tool rolls.
pub(crate) fn imprint_of(feature: TopoId, s: &Placed, tool: RibTool, flip: bool) -> Result<Imprint, BrepError> {
    if s.developed.is_some() {
        return Err(unsupported("a rib on a rolled sheet comes later: sketch on a flat wall"));
    }
    let face = s.face.ok_or_else(|| unsupported("the sketch of its path lies on a flat wall of the sheet"))?;
    if !(tool.radius > 0.0 && tool.depth > 0.0 && tool.shoulder > 0.0) {
        return Err(degenerate(format!("a tool has a diameter, a depth and a shoulder above zero, got {tool:?}")));
    }
    let chains = open_chains_named(s)?;
    if chains.is_empty() {
        return Err(BrepError::Sketch("rib: the sketch has no open line or arc to roll along".into()));
    }
    for chain in &chains {
        for w in chain.windows(2) {
            let (a, b) = (&w[0].0, &w[1].0);
            let pair = if a < b { [a.clone(), b.clone()] } else { [b.clone(), a.clone()] };
            if !s.tangent.contains(&pair) {
                return Err(BrepError::Sketch(format!("rib: a tool cannot turn a corner: make `{a}` and `{b}` tangent")));
            }
        }
        let length: f64 = chain.iter().map(|(_, c)| piece_length(c)).sum();
        if length < tool.min_length {
            return Err(BrepError::Sketch(format!("rib: a path of {length:.1} mm is shorter than the {:.1} mm the tool rolls", tool.min_length)));
        }
    }
    Ok(Imprint { feature, face, frame: s.frame, chains, tool, flip })
}

fn piece_length(c: &ChainPiece) -> f64 {
    match c.arc {
        None => libm::hypot(c.to.x - c.from.x, c.to.y - c.from.y),
        Some(a) => libm::hypot(c.from.x - a.center.x, c.from.y - a.center.y) * a.turn(c.from, c.to).abs(),
    }
}

/// The body with the imprint's ribs on it.
pub(crate) fn imprint(body: &Body, i: &Imprint) -> Result<Body, BrepError> {
    let f = body
        .faces()
        .iter()
        .position(|x| x.id == i.face)
        .ok_or_else(|| BrepError::LostReference(format!("rib: no face {:?} of the sheet", i.face)))?;
    let Surface::Plane(plane) = body.faces()[f].surface else {
        return Err(unsupported("the sketch of its path lies on a flat wall of the sheet"));
    };
    let [origin, ex, ey, ez] = i.frame;
    let frame = Frame::Plane { o: origin, x: ex, y: ey, z: ez };
    let start = i.chains[0][0].1.from;
    // The other side of the wall: the nearest plane facing the other way with the path over it.
    let n = unit(plane.normal);
    let holds = |g: usize, p: Vec2| -> Result<bool, BrepError> {
        let loops = outline(body, g as u32, &frame)?;
        Ok(loops.first().is_some_and(|l| l.winding(p) != 0) && loops[1..].iter().all(|l| l.winding(p) == 0))
    };
    let mut across: Vec<(f64, usize)> = Vec::new();
    for (g, x) in body.faces().iter().enumerate() {
        if let Surface::Plane(p) = x.surface
            && p.normal.cross(plane.normal) == Vec3::ZERO
            && p.normal.dot(plane.normal) < 0.0
        {
            let d = (p.origin - plane.origin).dot(-n);
            if d > 0.0 && holds(g, start)? {
                across.push((d, g));
            }
        }
    }
    across.sort_by(|a, b| a.0.total_cmp(&b.0));
    let &(t, g) = across.first().ok_or_else(|| unsupported("the other side of the wall is not found under the path"))?;
    // The rise: out of the sketch's face, or into it. The side pressed from is opposite.
    let zs = if i.flip { -1.0 } else { 1.0 };
    let z = ez * zs;
    let (inner, outer) = if i.flip { (f, g) } else { (g, f) };
    let base = if i.flip { origin } else { origin - z * t };
    let world = |p: Vec2, h: f64| base + ex * p.x + ey * p.y + z * h;
    let sec = Section::of(i.tool, t)?;
    let mut out = Build {
        vertices: body.vertices().to_vec(),
        faces: (0..body.faces().len() as u32).map(|f| body.face_spec(f)).collect(),
        curves: body.curve_specs(),
    };
    let lay = Lay { feature: i.feature, ez, zs, ex, ey, world: &world, sec };
    for chain in &i.chains {
        let foot = lay.chain(chain, &mut out, [inner, outer])?;
        if foot.edges_touch(None) {
            return Err(degenerate(format!("the rib along `{}` touches itself: a path turning too tight or coming back near itself", chain[0].0)));
        }
        for h in [inner, outer] {
            let loops = outline(body, h as u32, &frame)?;
            let inside = loops.first().is_some_and(|l| l.winding(foot.points[0]) != 0) && loops[1..].iter().all(|l| l.winding(foot.points[0]) == 0);
            // A hole under the rib, past its foot: pressed over, it would not be there.
            let over = || loops[1..].iter().any(|l| foot.winding(l.points[0]) != 0);
            if !inside || loops.iter().any(|l| l.touches(&foot)) || over() {
                return Err(BrepError::Sketch(format!(
                    "rib: the rib along `{}` does not stay inside its wall: its foot runs {:.2} mm each side of the path and past its ends, and reaches an edge, a hole, a bend or another rib",
                    chain[0].0, sec.w
                )));
            }
        }
    }
    build_with_curves(out.vertices, out.faces, out.curves)
}

/// The section of a rib for a sheet `t` thick. `w` is where the foot is from the path; the rest
/// by side of the sheet, `[pressed from, far]`: `c` how far along the crown goes across, `z` how
/// high that is, `crown` and `shoulder` the radii. Heights from the plane pressed from.
#[derive(Clone, Copy)]
struct Section {
    w: f64,
    t: f64,
    crown_z: f64,
    shoulder_z: f64,
    depth: f64,
    a: [f64; 2],
    z: [f64; 2],
    crown: [f64; 2],
    shoulder: [f64; 2],
}

impl Section {
    fn of(tool: RibTool, t: f64) -> Result<Self, BrepError> {
        let (r, h, s) = (tool.radius, tool.depth, tool.shoulder);
        // Crown and shoulder touch where the line through their centres cuts them: the two
        // centres are `reach` apart.
        let reach = r + s + t;
        if h >= reach {
            return Err(degenerate(format!(
                "a depth of {h} mm undercuts the section: a crown of {r} mm and a shoulder of {s} mm on {t} mm reach {reach} mm"
            )));
        }
        let w = libm::sqrt(h * (2.0 * reach - h));
        let (sin, cos) = (w / reach, (reach - h) / reach);
        let crown_z = h - r;
        let crown = [r, r + t];
        let a = crown.map(|c| c * sin);
        let z = crown.map(|c| crown_z + c * cos);
        Ok(Self { w, t, crown_z, shoulder_z: t + s, depth: h, a, z, crown, shoulder: [s + t, s] })
    }

    fn foot_z(&self, layer: usize) -> f64 {
        self.t * layer as f64
    }

    fn top_z(&self, layer: usize) -> f64 {
        self.depth + self.t * layer as f64
    }
}

struct Build {
    vertices: Vec<Vertex>,
    faces: Vec<FaceSpec>,
    curves: Vec<CurveSpec>,
}

/// Points across the section at a station, left to right.
const FOOT_LEFT: u64 = 0;
const TOP_LEFT: u64 = 1;
const APEX: u64 = 2;
const TOP_RIGHT: u64 = 3;
const FOOT_RIGHT: u64 = 4;

struct Lay<'a> {
    feature: TopoId,
    ez: Vec3,
    zs: f64,
    ex: Vec3,
    ey: Vec3,
    world: &'a dyn Fn(Vec2, f64) -> Vec3,
    sec: Section,
}

impl Lay<'_> {
    /// A rib along `chain`, each side of the sheet into faces `sides[layer]` as a hole; returns
    /// the foot as a loop in the sketch's coordinates, counterclockwise.
    fn chain(&self, chain: &[(String, ChainPiece)], out: &mut Build, sides: [usize; 2]) -> Result<ProfileLoop, BrepError> {
        let m = chain.len();
        let sec = self.sec;
        // A station by the curve it starts, the last by the curve it ends.
        let vid = |i: usize, layer: usize, label: u64| {
            let (name, end) = if i < m { (&chain[i].0, 0) } else { (&chain[m - 1].0, 1) };
            TopoId::derive("rib.v").id(self.feature).tag(name).index(end).index(layer as u64).index(label).finish()
        };
        // Each station's place and left normal; at a joint an arc says it, being exact there.
        let stations: Vec<(Vec2, Vec2)> = (0..=m)
            .map(|i| {
                let before = i.checked_sub(1).map(|k| (&chain[k].1, true));
                let after = chain.get(i).map(|(_, c)| (c, false));
                let (c, end) =
                    [before, after].into_iter().flatten().find(|(c, _)| c.arc.is_some()).or(before).or(after).unwrap_or((&chain[0].1, false));
                let p = if end { c.to } else { c.from };
                (p, left(c, p))
            })
            .collect();
        let across = |i: usize, s: f64| at(stations[i].0, stations[i].1, s);
        let w = sec.w;
        // Vertices.
        for (i, _) in stations.iter().enumerate() {
            for layer in 0..2 {
                let (fz, tz, a) = (sec.foot_z(layer), sec.z[layer], sec.a[layer]);
                for (label, s, z) in [(FOOT_LEFT, w, fz), (TOP_LEFT, a, tz), (TOP_RIGHT, -a, tz), (FOOT_RIGHT, -w, fz)] {
                    out.vertices.push(Vertex { id: vid(i, layer, label), point: (self.world)(across(i, s), z) });
                }
                if i == 0 || i == m {
                    out.vertices.push(Vertex { id: vid(i, layer, APEX), point: (self.world)(stations[i].0, sec.top_z(layer)) });
                }
            }
        }
        // Across each station: from the foot to the top and across the crown.
        for (i, _) in stations.iter().enumerate() {
            for layer in 0..2 {
                let shoulders = [(FOOT_LEFT, TOP_LEFT, w), (TOP_RIGHT, FOOT_RIGHT, -w)];
                for (a, b, s) in shoulders {
                    self.section_arc(out, vid(i, layer, a), vid(i, layer, b), (self.world)(across(i, s), sec.shoulder_z), sec.shoulder[layer]);
                }
                let c = (self.world)(stations[i].0, sec.crown_z);
                if i == 0 || i == m {
                    self.section_arc(out, vid(i, layer, TOP_LEFT), vid(i, layer, APEX), c, sec.crown[layer]);
                    self.section_arc(out, vid(i, layer, APEX), vid(i, layer, TOP_RIGHT), c, sec.crown[layer]);
                } else {
                    self.section_arc(out, vid(i, layer, TOP_LEFT), vid(i, layer, TOP_RIGHT), c, sec.crown[layer]);
                }
            }
        }
        let flipped = |layer: usize, mut ids: Vec<TopoId>| {
            let up = (layer as f64 * 2.0 - 1.0) * self.zs > 0.0;
            if !up {
                ids.reverse();
            }
            ids
        };
        let face =
            |name: &str, layer: usize, strip: u64| TopoId::derive("rib.f").id(self.feature).tag(name).index(layer as u64).index(strip).finish();
        let convex = |layer: usize| [layer == 1, layer == 0]; // [crown, shoulder]
        // Along each piece: three strips each side of the sheet.
        for (j, (name, c)) in chain.iter().enumerate() {
            let turn = match c.arc {
                Some(a) => {
                    let radius = libm::hypot(c.from.x - a.center.x, c.from.y - a.center.y);
                    if radius <= w {
                        return Err(degenerate(format!("`{name}` turns at {radius:.2} mm, tighter than the rib's half width of {w:.2} mm")));
                    }
                    Some((a.center, radius, if a.ccw { 1.0 } else { -1.0 }))
                }
                None => None,
            };
            let d = sub(c.to, c.from);
            let tangent = unit(self.ex * d.x + self.ey * d.y);
            for layer in 0..2 {
                let [crown_convex, shoulder_convex] = convex(layer);
                // `[left shoulder, crown, right shoulder]`: where across, how high, what radius.
                let strips = [
                    (w, sec.shoulder_z, sec.shoulder[layer], shoulder_convex),
                    (0.0, sec.crown_z, sec.crown[layer], crown_convex),
                    (-w, sec.shoulder_z, sec.shoulder[layer], shoulder_convex),
                ];
                for (k, (s, z, radius, cvx)) in strips.into_iter().enumerate() {
                    let surface = match turn {
                        None => Surface::Cylinder(Cylinder { origin: (self.world)(across(j, s), z), axis: tangent, radius, convex: cvx }),
                        Some((center, r, sigma)) => {
                            Surface::Torus(Torus { center: (self.world)(center, z), axis: self.ez, major: r - sigma * s, minor: radius, convex: cvx })
                        }
                    };
                    let (right, left) = [(TOP_LEFT, FOOT_LEFT), (TOP_RIGHT, TOP_LEFT), (FOOT_RIGHT, TOP_RIGHT)][k];
                    let mut ids = vec![vid(j, layer, right), vid(j + 1, layer, right)];
                    if k == 1 && j + 1 == m {
                        ids.push(vid(j + 1, layer, APEX));
                    }
                    ids.extend([vid(j + 1, layer, left), vid(j, layer, left)]);
                    if k == 1 && j == 0 {
                        ids.push(vid(j, layer, APEX));
                    }
                    out.faces.push(FaceSpec { id: face(name, layer, k as u64), surface, loops: vec![flipped(layer, ids)] });
                }
                // Along the piece: at each of the four lines of the section.
                if let Some((center, r, sigma)) = turn {
                    let axis = self.ez * sigma;
                    for (label, s, z) in [
                        (FOOT_LEFT, w, sec.foot_z(layer)),
                        (TOP_LEFT, sec.a[layer], sec.z[layer]),
                        (TOP_RIGHT, -sec.a[layer], sec.z[layer]),
                        (FOOT_RIGHT, -w, sec.foot_z(layer)),
                    ] {
                        out.curves.push(CurveSpec {
                            from: vid(j, layer, label),
                            to: vid(j + 1, layer, label),
                            curve: Curve::Arc(Arc { center: (self.world)(center, z), axis, radius: r - sigma * s }),
                        });
                    }
                }
            }
        }
        // The two ends, turned half way about the end point.
        for (i, end) in [(0, false), (m, true)] {
            let name = &chain[if end { m - 1 } else { 0 }].0;
            let q = stations[i].0;
            for layer in 0..2 {
                let [crown_convex, shoulder_convex] = convex(layer);
                let v = |label: u64| vid(i, layer, label);
                // Counterclockwise about the sketch's normal, round the far side of the end.
                let ([fa, fb], [ta, tb]) =
                    if end { ([FOOT_RIGHT, FOOT_LEFT], [TOP_RIGHT, TOP_LEFT]) } else { ([FOOT_LEFT, FOOT_RIGHT], [TOP_LEFT, TOP_RIGHT]) };
                let parallel = |out: &mut Build, a: u64, b: u64, z: f64, radius: f64| {
                    out.curves.push(CurveSpec { from: v(a), to: v(b), curve: Curve::Arc(Arc { center: (self.world)(q, z), axis: self.ez, radius }) });
                };
                parallel(out, fa, fb, sec.foot_z(layer), w);
                parallel(out, ta, tb, sec.z[layer], sec.a[layer]);
                let id = |strip: u64| face(name, layer, strip + if end { 5 } else { 3 });
                let sphere = Torus { center: (self.world)(q, sec.crown_z), axis: self.ez, major: 0.0, minor: sec.crown[layer], convex: crown_convex };
                out.faces.push(FaceSpec { id: id(0), surface: Surface::Torus(sphere), loops: vec![flipped(layer, vec![v(ta), v(tb), v(APEX)])] });
                let ring =
                    Torus { center: (self.world)(q, sec.shoulder_z), axis: self.ez, major: w, minor: sec.shoulder[layer], convex: shoulder_convex };
                out.faces.push(FaceSpec { id: id(1), surface: Surface::Torus(ring), loops: vec![flipped(layer, vec![v(fa), v(fb), v(tb), v(ta)])] });
            }
        }
        // The hole along the foot, each side of the sheet: clockwise seen from outside the face.
        for (layer, &side) in sides.iter().enumerate() {
            let mut ids: Vec<TopoId> = (0..=m).map(|i| vid(i, layer, FOOT_RIGHT)).collect();
            ids.extend((0..=m).rev().map(|i| vid(i, layer, FOOT_LEFT)));
            let up = (layer as f64 * 2.0 - 1.0) * self.zs > 0.0;
            if up {
                ids.reverse();
            }
            out.faces[side].loops.push(ids);
        }
        // The foot in the sketch, counterclockwise: down the right, round the end, back up the left.
        let mut ring = ProfileLoop { points: Vec::new(), vertices: Vec::new(), sides: Vec::new(), arcs: Vec::new() };
        for (j, (_, c)) in chain.iter().enumerate() {
            ring.points.push(across(j, -w));
            ring.arcs.push(c.arc.map(|a| ProfileArc { center: a.center, ccw: a.ccw }));
        }
        ring.points.push(across(m, -w));
        ring.arcs.push(Some(ProfileArc { center: stations[m].0, ccw: true }));
        for j in (0..m).rev() {
            ring.points.push(across(j + 1, w));
            ring.arcs.push(chain[j].1.arc.map(|a| ProfileArc { center: a.center, ccw: !a.ccw }));
        }
        ring.points.push(across(0, w));
        ring.arcs.push(Some(ProfileArc { center: stations[0].0, ccw: true }));
        Ok(ring)
    }

    /// An arc of a section from `a` to `b` about `center`: turning the short way.
    fn section_arc(&self, out: &mut Build, a: TopoId, b: TopoId, center: Vec3, radius: f64) {
        let point = |id: TopoId| out.vertices.iter().rev().find(|v| v.id == id).map_or(Vec3::ZERO, |v| v.point);
        let (u, v) = (point(a) - center, point(b) - center);
        out.curves.push(CurveSpec { from: a, to: b, curve: Curve::Arc(Arc { center, axis: unit(u.cross(v)), radius }) });
    }
}

/// The unit normal to the left of piece `c` at its point `p`.
fn left(c: &ChainPiece, p: Vec2) -> Vec2 {
    let n = |d: Vec2| {
        let len = libm::hypot(d.x, d.y);
        Vec2::new(-d.y / len, d.x / len)
    };
    match c.arc {
        None => n(sub(c.to, c.from)),
        Some(a) => {
            let r = sub(p, a.center);
            // Turning counterclockwise, the centre is to the left.
            let len = libm::hypot(r.x, r.y);
            let s = if a.ccw { -1.0 } else { 1.0 };
            Vec2::new(s * r.x / len, s * r.y / len)
        }
    }
}
