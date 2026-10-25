//! STEP export of a part (D130): ISO 10303-21 text, schema AP242, with the recommendations of
//! the CAx-IF. Exact geometry is written as it is: planes, cylinders, lines and circles. What has
//! no exact STEP type (a cut through a rolled sheet along a circle or an oblique line of its
//! development: a helix, or a curve without a name) is refused, or, when the engineer agrees,
//! written as quintic B-splines within [`DEVIATION`] (D147).
//!
//! Every shell of a body is a `MANIFOLD_SOLID_BREP`: a half-edge loop is an `EDGE_LOOP` of
//! oriented edges, run counterclockwise seen from outside as STEP wants it, so every bound is
//! `.T.` and a face's sense says only whether the body is on the side of its surface's normal.
//! The part carries validation properties (volume, surface area, centroid) the receiving CAD
//! checks the geometry it read against.
//!
//! Entities are numbered in the order of the bodies, and numbers are shortest round-trip
//! decimals: one model gives the same bytes on every machine and thread count.

use std::fmt::{self, Write};

use datum_math::Vec3;

use crate::curve::{Integrals, unit};
use crate::{Body, BrepError, Curve, Ix, Rolled, Surface};

/// The distance two points of the file are taken as one at, in millimetres: what commercial
/// CADs write, far above the rounding of the kernel's coordinates.
const UNCERTAINTY: f64 = 1e-6;

/// How far, in millimetres, a B-spline written for geometry STEP has no exact type for may stray
/// from it (D147): a tenth of [`UNCERTAINTY`], so the receiving CAD cannot tell it from exact.
pub const DEVIATION: f64 = 1e-7;

/// More spans than this for one piece of a cut: the piece is not one the kernel makes.
const MAX_SPANS: usize = 1 << 16;

/// A STEP file, and how much of it is approximated.
pub struct Step {
    pub text: String,
    /// Faces written as B-splines.
    pub approximated: usize,
    /// The largest distance found between those splines and the model, in millimetres.
    pub deviation: f64,
}

/// The faces of `b` STEP has no exact type for: walls of cuts through a rolled sheet along
/// circles or oblique lines of its development.
pub fn inexact_faces(b: &Body) -> usize {
    b.faces.iter().filter(|f| matches!(f.surface, Surface::Wall(w) if exact(&w).is_none())).count()
}

/// The bodies of part `name` as a STEP file, millimetres. Geometry STEP has no exact type for is
/// written as B-splines when `approximate`, else refused (`Unsupported`); so is a body with an
/// inner void.
pub fn step(bodies: &[&Body], name: &str, approximate: bool) -> Result<Step, BrepError> {
    let inexact: usize = bodies.iter().map(|b| inexact_faces(b)).sum();
    if inexact > 0 && !approximate {
        return Err(BrepError::Unsupported(format!(
            "STEP: {inexact} face(s) of cuts through a rolled sheet, along circles or oblique lines of its development, have no exact type"
        )));
    }
    let capacity = bodies.iter().map(|b| b.half_edges.len() * 160 + 4096).sum();
    let mut w = Writer { text: String::with_capacity(capacity), next: 1, approximated: 0, deviation: 0.0 };
    let name = string(name);
    let app = w.put("APPLICATION_CONTEXT('managed model based 3d engineering')");
    w.put(format_args!("APPLICATION_PROTOCOL_DEFINITION('international standard','ap242_managed_model_based_3d_engineering',2014,#{app})"));
    let context = w.put(format_args!("PRODUCT_CONTEXT('',#{app},'mechanical')"));
    let product = w.put(format_args!("PRODUCT({name},{name},'',(#{context}))"));
    w.put(format_args!("PRODUCT_RELATED_PRODUCT_CATEGORY('part',$,(#{product}))"));
    let formation = w.put(format_args!("PRODUCT_DEFINITION_FORMATION('','',#{product})"));
    let stage = w.put(format_args!("PRODUCT_DEFINITION_CONTEXT('part definition',#{app},'design')"));
    let definition = w.put(format_args!("PRODUCT_DEFINITION('design','',#{formation},#{stage})"));
    let shape = w.put(format_args!("PRODUCT_DEFINITION_SHAPE('','',#{definition})"));
    let mm = w.put("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.))");
    let rad = w.put("(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.))");
    let sr = w.put("(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT())");
    let uncertainty = w.put(format_args!(
        "UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE({}),#{mm},'distance_accuracy_value','confusion accuracy')",
        Real(UNCERTAINTY)
    ));
    let ctx = w.put(format_args!(
        "(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{uncertainty}))\
         GLOBAL_UNIT_ASSIGNED_CONTEXT((#{mm},#{rad},#{sr}))REPRESENTATION_CONTEXT('',''))"
    ));
    let mut items = vec![w.placement(Vec3::ZERO, Vec3::new(0.0, 0.0, 1.0), Vec3::new(1.0, 0.0, 0.0))];
    for b in bodies {
        w.body(b, &mut items)?;
    }
    let rep = w.put(format_args!("ADVANCED_BREP_SHAPE_REPRESENTATION({name},{},#{ctx})", List(&items)));
    w.put(format_args!("SHAPE_DEFINITION_REPRESENTATION(#{shape},#{rep})"));

    // Validation properties of the whole part: every CAD that reads them looks there (CAx-IF).
    let (mut volume, mut area, mut moment) = (0.0, 0.0, Vec3::ZERO);
    for b in bodies {
        let m = b.mass_properties();
        (volume, area, moment) = (volume + m.volume, area + m.area, moment + m.centroid * m.volume);
    }
    let centroid = moment * (1.0 / volume);
    let cube = w.put(format_args!("DERIVED_UNIT_ELEMENT(#{mm},3.)"));
    let cube = w.put(format_args!("(DERIVED_UNIT((#{cube}))VOLUME_UNIT())"));
    let square = w.put(format_args!("DERIVED_UNIT_ELEMENT(#{mm},2.)"));
    let square = w.put(format_args!("(AREA_UNIT()DERIVED_UNIT((#{square})))"));
    let values = [
        ("volume", format!("MEASURE_REPRESENTATION_ITEM('volume measure',VOLUME_MEASURE({}),#{cube})", Real(volume))),
        ("surface area", format!("MEASURE_REPRESENTATION_ITEM('surface area measure',AREA_MEASURE({}),#{square})", Real(area))),
        ("centroid", format!("CARTESIAN_POINT('centre point',{})", Coords(centroid))),
    ];
    for (what, item) in values {
        let item = w.put(item);
        let rep = w.put(format_args!("REPRESENTATION('{what}',(#{item}),#{ctx})"));
        let property = w.put(format_args!("PROPERTY_DEFINITION('geometric validation property','{what} of the part',#{shape})"));
        w.put(format_args!("PROPERTY_DEFINITION_REPRESENTATION(#{property},#{rep})"));
    }
    w.text.push_str("ENDSEC;\nEND-ISO-10303-21;\n");
    let approximated = if w.approximated == 0 {
        String::new()
    } else {
        format!(
            ",'{} face(s) of cuts through a rolled sheet approximated by B-splines, largest deviation {} mm (bound {} mm); validation properties of the exact model'",
            w.approximated,
            Real(w.deviation),
            Real(DEVIATION)
        )
    };
    let header = format!(
        "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('Datum part'{approximated}),'2;1');\n\
         FILE_NAME({name},'',(''),(''),'Datum {}','Datum','');\n\
         FILE_SCHEMA(('AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF {{ 1 0 10303 442 1 1 4 }}'));\nENDSEC;\nDATA;\n",
        env!("CARGO_PKG_VERSION")
    );
    Ok(Step { text: header + &w.text, approximated: w.approximated, deviation: w.deviation })
}

struct Writer {
    /// The entities: the header goes before them once it is known what was approximated.
    text: String,
    next: u32,
    approximated: usize,
    deviation: f64,
}

impl Writer {
    /// Writes entity `e` under the next number and returns the number.
    fn put(&mut self, e: impl fmt::Display) -> u32 {
        let n = self.next;
        self.next += 1;
        let _ = writeln!(self.text, "#{n}={e};");
        n
    }

    fn point(&mut self, p: Vec3) -> u32 {
        self.put(format_args!("CARTESIAN_POINT('',{})", Coords(p)))
    }

    fn direction(&mut self, d: Vec3) -> u32 {
        self.put(format_args!("DIRECTION('',{})", Coords(unit(d))))
    }

    /// A frame at `origin`: `z` its axis, `x` its first direction, square to `z`.
    fn placement(&mut self, origin: Vec3, z: Vec3, x: Vec3) -> u32 {
        let (o, z, x) = (self.point(origin), self.direction(z), self.direction(x));
        self.put(format_args!("AXIS2_PLACEMENT_3D('',#{o},#{z},#{x})"))
    }

    /// A circle about `axis` through `center`, its parameter 0 towards `start`.
    fn circle(&mut self, center: Vec3, axis: Vec3, start: Vec3, radius: f64) -> u32 {
        let d = start - center;
        let at = self.placement(center, axis, d - unit(axis) * d.dot(unit(axis)));
        self.put(format_args!("CIRCLE('',#{at},{})", Real(radius)))
    }

    fn line(&mut self, from: Vec3, to: Vec3) -> u32 {
        let (p, d) = (self.point(from), self.direction(to - from));
        let v = self.put(format_args!("VECTOR('',#{d},1.)"));
        self.put(format_args!("LINE('',#{p},#{v})"))
    }

    /// A quintic B-spline of `n` spans through `points` (see [`controls`]).
    fn spline(&mut self, points: &[Vec3], n: usize) -> u32 {
        let points: Vec<u32> = points.iter().map(|&p| self.point(p)).collect();
        let (mults, knots) = knots(n);
        self.put(format_args!("B_SPLINE_CURVE_WITH_KNOTS('',5,{},.UNSPECIFIED.,.F.,.F.,{mults},{knots},.UNSPECIFIED.)", List(&points)))
    }

    /// The wall over `w` between `radii` as a B-spline surface, quintic along the piece in `n`
    /// spans and straight along the radius, as the wall is; and whether its normal points out of
    /// the body.
    fn wall(&mut self, w: &Rolled, radii: [f64; 2], n: usize) -> (u32, bool) {
        let [inner, outer] = radii.map(|r| controls(w, r, n));
        let grid: Vec<[u32; 2]> = inner.iter().zip(&outer).map(|(&a, &b)| [self.point(a), self.point(b)]).collect();
        let (mults, knots) = knots(n);
        let surface = self.put(format_args!(
            "B_SPLINE_SURFACE_WITH_KNOTS('',5,1,{},.UNSPECIFIED.,.F.,.F.,.F.,{mults},(2,2),{knots},(0.,1.),.UNSPECIFIED.)",
            Grid(&grid)
        ));
        // Along the piece, then out along the radius: against the normal out of the body or not.
        let (q, along) = (w.at(0.5).0, hermite(w, radii[0], 0.5)[1]);
        (surface, along.cross(w.dev.radial(q.x)).dot(w.normal(q, radii[0])) > 0.0)
    }

    /// Writes the shells of `b`, one solid each, and adds the solids to `items`.
    fn body(&mut self, b: &Body, items: &mut Vec<u32>) -> Result<(), BrepError> {
        let shells = b.face_shells();
        let (o, mut six_v) = (b.reference(), vec![0.0; b.faces.len()]);
        for f in 0..b.faces.len() as Ix {
            let mut acc = Integrals::default();
            b.integrate_face(f, o, &mut acc);
            six_v[shells[f as usize] as usize] += acc.six_v;
        }
        if let Some(f) = (0..b.faces.len()).find(|&f| shells[f] == f as Ix && six_v[f] < 0.0) {
            return Err(BrepError::Unsupported(format!("STEP: the body has an inner void (face {:?})", b.faces[f].id)));
        }
        let vertices: Vec<u32> = b
            .vertices
            .iter()
            .map(|v| {
                let p = self.point(v.point);
                self.put(format_args!("VERTEX_POINT('',#{p})"))
            })
            .collect();
        // The spans of each inexact wall, and of the edges along it: an edge is then the very
        // boundary of its wall's spline.
        let mut walls = vec![0; b.faces.len()];
        let mut spans = vec![0; b.edges.len()];
        for (f, face) in b.faces.iter().enumerate() {
            let Surface::Wall(w) = face.surface else { continue };
            if exact(&w).is_some() {
                continue;
            }
            let (n, deviation) = fit(&w, radii(b, f as Ix)?)?;
            (walls[f], self.deviation, self.approximated) = (n, self.deviation.max(deviation), self.approximated + 1);
            for h in b.face_loops(f as Ix).flat_map(|l| b.loop_half_edges(l)) {
                let e = b.half_edges[h as usize].edge as usize;
                if matches!(b.edge_curve(&b.edges[e]), Some(Curve::Rolled { piece, .. }) if exact(&piece).is_none()) {
                    spans[e] = n;
                }
            }
        }
        let mut edges = Vec::with_capacity(b.edges.len());
        for (k, e) in b.edges.iter().enumerate() {
            let he = &b.half_edges[e.half as usize];
            let (lo, hi) = (he.origin, b.half_edges[he.twin as usize].origin);
            let (from, to) = (b.point(lo), b.point(hi));
            let curve = match b.edge_curve(e) {
                None => self.line(from, to),
                Some(Curve::Arc(a)) => self.circle(a.center, a.axis, from, a.radius),
                Some(Curve::Rolled { piece, radius }) => match exact(&piece) {
                    Some(Exact::Along) => self.line(from, to),
                    Some(Exact::Around { center, axis }) => self.circle(center, axis, from, radius),
                    None => {
                        let n = match spans[k] {
                            0 => fit(&piece, [radius, radius])?.0,
                            n => n,
                        };
                        self.spline(&controls(&piece, radius, n), n)
                    }
                },
            };
            edges.push(self.put(format_args!("EDGE_CURVE('',#{},#{},#{curve},.T.)", vertices[lo as usize], vertices[hi as usize])));
        }
        let mut faces: Vec<Vec<u32>> = vec![Vec::new(); b.faces.len()];
        for (f, face) in b.faces.iter().enumerate() {
            let p = b.point(b.half_edges[b.loops[face.first_loop as usize].first as usize].origin);
            let (surface, sense) = match face.surface {
                Surface::Plane(pl) => (self.placement(pl.origin, pl.normal, perpendicular(pl.normal)), true),
                Surface::Cylinder(c) => {
                    let at = self.placement(c.origin, c.axis, perpendicular(c.axis));
                    (self.put(format_args!("CYLINDRICAL_SURFACE('',#{at},{})", Real(c.radius))), c.convex)
                }
                Surface::Wall(w) if walls[f] > 0 => self.wall(&w, radii(b, f as Ix)?, walls[f]),
                Surface::Wall(w) => {
                    // A straight piece along the axis or around it: the wall is flat.
                    let n = w.normal(w.from, w.dev.neutral);
                    let at = self.placement(p, n, perpendicular(n));
                    (self.put(format_args!("PLANE('',#{at})")), true)
                }
            };
            let surface = match face.surface {
                Surface::Plane(_) => self.put(format_args!("PLANE('',#{surface})")),
                Surface::Cylinder(_) | Surface::Wall(_) => surface,
            };
            let mut bounds = Vec::with_capacity(face.loop_count as usize);
            for (k, l) in b.face_loops(f as Ix).enumerate() {
                let oriented: Vec<u32> = b
                    .loop_half_edges(l)
                    .map(|h| {
                        let e = b.half_edges[h as usize].edge;
                        let same = b.edges[e as usize].half == h;
                        self.put(format_args!("ORIENTED_EDGE('',*,*,#{},{})", edges[e as usize], logical(same)))
                    })
                    .collect();
                let lp = self.put(format_args!("EDGE_LOOP('',{})", List(&oriented)));
                let kind = if k == 0 { "FACE_OUTER_BOUND" } else { "FACE_BOUND" };
                bounds.push(self.put(format_args!("{kind}('',#{lp},.T.)")));
            }
            let face = self.put(format_args!("ADVANCED_FACE('',{},#{surface},{})", List(&bounds), logical(sense)));
            faces[shells[f] as usize].push(face);
        }
        for f in (0..b.faces.len()).filter(|&f| shells[f] == f as Ix) {
            let shell = self.put(format_args!("CLOSED_SHELL('',{})", List(&faces[f])));
            items.push(self.put(format_args!("MANIFOLD_SOLID_BREP('',#{shell})")));
        }
        Ok(())
    }
}

/// What a piece of a development is in 3D when STEP has an exact type for it.
enum Exact {
    /// Straight along the axis.
    Along,
    /// An arc around the axis, about `axis` through `center`.
    Around { center: Vec3, axis: Vec3 },
}

/// A straight piece along the axis or around it; an arc or an oblique piece of a development is
/// a curve STEP has no exact type for. Compared exactly: the cut gives the ends of such a piece
/// the same coordinate.
fn exact(piece: &Rolled) -> Option<Exact> {
    let (f, t, d) = (piece.from, piece.to, piece.dev);
    match piece.arc {
        Some(_) => None,
        None if f.x == t.x => Some(Exact::Along),
        None if f.y == t.y => Some(Exact::Around { center: d.origin + d.axis * f.y, axis: if t.x > f.x { d.axis } else { -d.axis } }),
        None => None,
    }
}

/// The radii a wall of `b` spans: those of the pieces along its edges.
fn radii(b: &Body, f: Ix) -> Result<[f64; 2], BrepError> {
    let (r0, r1) = b.rolled_radii(f).fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), r| (lo.min(r), hi.max(r)));
    if r0 < r1 { Ok([r0, r1]) } else { Err(BrepError::Degenerate(format!("STEP: wall {:?} spans no thickness", b.faces[f as usize].id))) }
}

/// The point of `piece` at `t` on radius `r`, and its first and second derivatives in `t`. With
/// `φ = u / neutral`, `ρ` the unit vector out from the axis and `τ = axis × ρ`:
/// `c = origin + axis·v + ρ·r`, `c′ = τ·r·u′/N + axis·v′`, `c″ = −ρ·r·(u′/N)² + τ·r·u″/N + axis·v″`.
fn hermite(piece: &Rolled, r: f64, t: f64) -> [Vec3; 3] {
    let (p, d) = piece.at(t);
    let dd = piece.second(t);
    let dev = &piece.dev;
    let (rho, k) = (dev.radial(p.x), r / dev.neutral);
    let tau = dev.axis.cross(rho);
    [dev.point(p, r), tau * (d.x * k) + dev.axis * d.y, rho * (-d.x * d.x * k / dev.neutral) + tau * (dd.x * k) + dev.axis * dd.y]
}

/// The Bézier points of the span of `piece` on radius `r` from `a` to `b` (parameter length
/// `h`): the quintic matching the piece's points and first and second derivatives at both ends.
fn span(a: &[Vec3; 3], b: &[Vec3; 3], h: f64) -> [Vec3; 6] {
    let (h1, h2) = (h / 5.0, h * h / 20.0);
    [a[0], a[0] + a[1] * h1, a[0] + a[1] * (2.0 * h1) + a[2] * h2, b[0] - b[1] * (2.0 * h1) + b[2] * h2, b[0] - b[1] * h1, b[0]]
}

/// The control points of `piece` on radius `r` as a quintic B-spline of `n` equal spans (see
/// [`span`]); joined with matching derivatives under knots of multiplicity 4, the point between
/// two spans is implied (4n + 2 points). Points and derivatives are linear in `r`, so the control
/// points are too: the wall between two radii is exactly the ruled surface between the splines
/// of its edges.
fn controls(piece: &Rolled, r: f64, n: usize) -> Vec<Vec3> {
    let h = 1.0 / n as f64;
    let mut out = Vec::with_capacity(4 * n + 2);
    let mut a = hermite(piece, r, 0.0);
    out.push(a[0]);
    for k in 1..=n {
        let b = hermite(piece, r, k as f64 * h);
        out.extend_from_slice(&span(&a, &b, h)[1..5]);
        a = b;
    }
    out.push(a[0]);
    out
}

/// The largest distance between `piece` on radius `r` and its spline of `n` spans, sampled at
/// eighths of each span. Points of the same parameter are compared: no less than the distance
/// between the curves (on the kernel's cuts, equal to it within a percent).
fn sampled_deviation(piece: &Rolled, r: f64, n: usize) -> f64 {
    let h = 1.0 / n as f64;
    let mut worst: f64 = 0.0;
    let mut a = hermite(piece, r, 0.0);
    for k in 0..n {
        let b = hermite(piece, r, (k + 1) as f64 * h);
        let p = span(&a, &b, h);
        for j in 1..8 {
            let tau = f64::from(j) / 8.0;
            // De Casteljau.
            let mut q = p;
            for level in (1..6).rev() {
                for i in 0..level {
                    q[i] = q[i] + (q[i + 1] - q[i]) * tau;
                }
            }
            let d = q[0] - hermite(piece, r, (k as f64 + tau) * h)[0];
            worst = worst.max(d.dot(d).sqrt());
        }
        a = b;
    }
    worst
}

/// The spans a spline of `piece` needs on each of `radii` to stay within half [`DEVIATION`] where
/// sampled, the margin covering what sampling misses; and the deviation found. From the pieces of
/// at most 45° the integrals use; while it does not hold, more by the sixth power the error falls
/// with: the same piece always gets the same spans.
fn fit(piece: &Rolled, radii: [f64; 2]) -> Result<(usize, f64), BrepError> {
    let mut n = piece.quadrature_pieces();
    loop {
        let d = radii.iter().map(|&r| sampled_deviation(piece, r, n)).fold(0.0, f64::max);
        if d <= DEVIATION / 2.0 {
            return Ok((n, d));
        }
        if n >= MAX_SPANS {
            return Err(BrepError::Degenerate(format!("STEP: a cut's piece needs more than {MAX_SPANS} spline spans")));
        }
        let more = libm::pow(d / (DEVIATION / 2.0), 1.0 / 6.0);
        n = ((n as f64 * more * 1.05) as usize).max(n + 1).min(MAX_SPANS);
    }
}

/// The knot multiplicities and values of a quintic spline of `n` spans joined with matching
/// derivatives: `(6,4,…,4,6)` and `(0.,1.,…,n.)`.
fn knots(n: usize) -> (String, String) {
    let mut mults = String::with_capacity(2 * n + 4);
    let mut values = String::with_capacity(4 * n + 4);
    for k in 0..=n {
        let sep = if k == 0 { "(" } else { "," };
        let m = if k == 0 || k == n { 6 } else { 4 };
        let _ = write!(mults, "{sep}{m}");
        let _ = write!(values, "{sep}{}", Real(k as f64));
    }
    mults.push(')');
    values.push(')');
    (mults, values)
}

/// A unit vector square to `n`: across its smallest coordinate, whatever its length.
fn perpendicular(n: Vec3) -> Vec3 {
    let a = [n.x.abs(), n.y.abs(), n.z.abs()];
    let e = if a[0] <= a[1] && a[0] <= a[2] {
        Vec3::new(1.0, 0.0, 0.0)
    } else if a[1] <= a[2] {
        Vec3::new(0.0, 1.0, 0.0)
    } else {
        Vec3::new(0.0, 0.0, 1.0)
    };
    unit(n.cross(e))
}

fn logical(b: bool) -> &'static str {
    if b { ".T." } else { ".F." }
}

/// Entity numbers as a STEP list: `(#1,#2)`.
struct List<'a>(&'a [u32]);

impl fmt::Display for List<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_char('(')?;
        for (i, n) in self.0.iter().enumerate() {
            write!(f, "{}#{n}", if i == 0 { "" } else { "," })?;
        }
        f.write_char(')')
    }
}

/// Control points of a surface as a STEP list of lists: `((#1,#2),(#3,#4))`.
struct Grid<'a>(&'a [[u32; 2]]);

impl fmt::Display for Grid<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_char('(')?;
        for (i, row) in self.0.iter().enumerate() {
            write!(f, "{}{}", if i == 0 { "" } else { "," }, List(row))?;
        }
        f.write_char(')')
    }
}

/// A point's coordinates as a STEP list of reals.
struct Coords(Vec3);

impl fmt::Display for Coords {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "({},{},{})", Real(self.0.x), Real(self.0.y), Real(self.0.z))
    }
}

/// A STEP real: the shortest decimal that reads back as the number, with the point STEP demands
/// (`1.`, `1.5E-7`); minus zero as zero.
struct Real(f64);

impl fmt::Display for Real {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        debug_assert!(self.0.is_finite());
        // The shortest round-trip form fits in 32 bytes: 17 digits, sign, point, exponent.
        let mut buf = Buffer { bytes: [0; 32], len: 0 };
        write!(buf, "{:?}", self.0 + 0.0)?;
        let s = std::str::from_utf8(&buf.bytes[..buf.len]).map_err(|_| fmt::Error)?;
        let (mantissa, exponent) = s.split_once('e').map_or((s, None), |(m, e)| (m, Some(e)));
        let mantissa = mantissa.strip_suffix(".0").unwrap_or(mantissa);
        f.write_str(mantissa)?;
        if !mantissa.contains('.') {
            f.write_char('.')?;
        }
        if let Some(e) = exponent {
            write!(f, "E{e}")?;
        }
        Ok(())
    }
}

struct Buffer {
    bytes: [u8; 32],
    len: usize,
}

impl Write for Buffer {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let end = self.len + s.len();
        self.bytes.get_mut(self.len..end).ok_or(fmt::Error)?.copy_from_slice(s.as_bytes());
        self.len = end;
        Ok(())
    }
}

/// `s` as a STEP string: quotes doubled, backslashes doubled, and what is beyond printable ASCII
/// (a Cyrillic name) in `\X2\` hex of UTF-16.
fn string(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    out.push('\'');
    let mut wide = false;
    for c in s.chars() {
        let printable = (' '..='~').contains(&c);
        if printable == wide {
            out.push_str(if wide { "\\X0\\" } else { "\\X2\\" });
            wide = !wide;
        }
        match c {
            '\'' => out.push_str("''"),
            '\\' => out.push_str("\\\\"),
            c if printable => out.push(c),
            c => {
                for u in c.encode_utf16(&mut [0; 2]) {
                    let _ = write!(out, "{u:04X}");
                }
            }
        }
    }
    if wide {
        out.push_str("\\X0\\");
    }
    out.push('\'');
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reals_have_a_point_and_read_back() {
        for (x, s) in [(1.0, "1."), (-0.0, "0."), (0.1, "0.1"), (1e-7, "1.E-7"), (1.5e-7, "1.5E-7"), (1e21, "1.E21"), (-2.5, "-2.5")] {
            assert_eq!(Real(x).to_string(), s);
            assert_eq!(s.replace("E", "e").trim_end_matches('.').parse::<f64>().ok(), Some(x + 0.0), "{s}");
        }
    }

    /// A cube: its entities once each, its validation properties, and the same bytes every time.
    #[test]
    fn a_cube() {
        let cube = crate::cuboid(crate::TopoId::derive("test.cube").finish(), Vec3::ZERO, Vec3::new(1.0, 2.0, 3.0)).unwrap();
        let text = step(&[&cube], "Куб", false).unwrap().text;
        let count = |e: &str| text.matches(&format!("={e}(")).count();
        let counts = ["VERTEX_POINT", "EDGE_CURVE", "LINE", "ORIENTED_EDGE", "EDGE_LOOP", "FACE_OUTER_BOUND", "FACE_BOUND", "PLANE", "ADVANCED_FACE"];
        assert_eq!(counts.map(count), [8, 12, 12, 24, 6, 6, 0, 6, 6]);
        assert_eq!((count("CLOSED_SHELL"), count("MANIFOLD_SOLID_BREP")), (1, 1));
        assert!(text.contains("VOLUME_MEASURE(6.)") && text.contains("AREA_MEASURE(22.)"), "{text}");
        assert!(text.contains("CARTESIAN_POINT('centre point',(0.5,1.,1.5))"));
        assert!(text.contains("PRODUCT('\\X2\\041A04430431\\X0\\'"));
        assert_eq!(text, step(&[&cube], "Куб", false).unwrap().text);
    }

    /// A hole's arc in the development of a rolled sheet: the spline stays within the bound
    /// where the fit did not look, the piece run the other way gets the same spans, and the wall
    /// between two radii is the ruled surface between its edges.
    #[test]
    fn a_hole_in_a_rolled_sheet_is_within_the_bound() {
        use crate::{Development, ProfileArc};
        use datum_math::Vec2;
        let dev = Development { origin: Vec3::ZERO, axis: Vec3::new(0.0, 0.0, 1.0), start: Vec3::new(1.0, 0.0, 0.0), neutral: 501.32 };
        let center = Vec2::new(15.0, 15.0);
        let piece = Rolled { dev, from: Vec2::new(17.5, 15.0), to: Vec2::new(13.75, 17.165_063_5), arc: Some(ProfileArc { center, ccw: true }) };
        let radii = [500.0, 503.0];
        let (n, found) = fit(&piece, radii).unwrap();
        assert!(found <= DEVIATION / 2.0);
        assert_eq!(fit(&piece.reversed(), radii).unwrap().0, n);
        for r in [500.0, 501.5, 503.0] {
            assert!(sampled_deviation(&piece, r, n * 16) <= DEVIATION, "r {r}");
            assert_eq!(controls(&piece, r, n).len(), 4 * n + 2);
        }
        let [a, b, m] = [500.0, 503.0, 501.5].map(|r| controls(&piece, r, n));
        for ((a, b), m) in a.iter().zip(&b).zip(&m) {
            let d = (*a + *b) * 0.5 - *m;
            assert!(d.dot(d).sqrt() < 1e-12, "{d:?}");
        }
    }

    #[test]
    fn strings_are_escaped() {
        assert_eq!(string("it's"), "'it''s'");
        assert_eq!(string("a\\b"), "'a\\\\b'");
        assert_eq!(string("Лист 2"), "'\\X2\\041B043804410442\\X0\\ 2'");
    }
}
