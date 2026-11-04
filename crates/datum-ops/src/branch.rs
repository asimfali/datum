//! A branch pipe set on a main one (D227), as both are cut in the blank before rolling: the main
//! shell gets a hole the size of the branch's bore, and the branch's end is cut to sit on the
//! main shell's outside, a gap left for the weld. Both curves are where the generatrices of one
//! pipe meet the other's cylinder ([`Meet`]), exactly; the walls of both cuts stand square to
//! the sheet, as a laser cuts the blank. For now both pipes are shells (one cylinder each side
//! of the sheet), their axes not parallel, the branch within the main pipe's width.

use datum_brep::{
    Body, BrepError, Curve, CurveSpec, Cylinder, Development, FaceSpec, Meet, Rolled, Surface, TopoId, Trace, Vertex, build_with_curves,
};
use datum_math::{Vec2, Vec3};

use crate::trim::Shell;

fn unsupported(m: &str) -> BrepError {
    BrepError::Unsupported(format!("branch: {m}"))
}

/// The inner and outer cylinders of shell `body`.
fn walls(body: &Body) -> Result<[Cylinder; 2], BrepError> {
    if body.faces().iter().any(|f| matches!(f.surface, Surface::Cone(_))) {
        return Err(unsupported("a cone comes later"));
    }
    let mut found = body.faces().iter().filter_map(|f| if let Surface::Cylinder(c) = f.surface { Some(c) } else { None });
    let (Some(a), Some(b), None) = (found.next(), found.next(), found.next()) else {
        return Err(unsupported("each pipe is a rolled shell: one cylinder each side of the sheet"));
    };
    Ok(if a.radius < b.radius { [a, b] } else { [b, a] })
}

fn unit(v: Vec3) -> Vec3 {
    v * (1.0 / v.dot(v).sqrt())
}

/// How the branch stands to the main pipe: the unit axis of the branch towards the main pipe's
/// (from its end that stays to the one cut to fit), and the unit axis of the main pipe.
struct Stand {
    toward: Vec3,
    main_axis: Vec3,
}

impl Stand {
    /// `branch` about `b` towards `main` about `m`. The main pipe's axis is beyond the middle of
    /// the branch, along its axis, on the side of its end cut to fit. Refused when the axes are
    /// parallel or the branch, `radius` from its axis, reaches past the side of the main pipe
    /// (`main_radius` from its axis): every generatrix of the branch must meet the main pipe.
    fn of(branch: &Body, b: &Cylinder, m: &Cylinder, radius: f64, main_radius: f64) -> Result<Self, BrepError> {
        let (ba, ma) = (unit(b.axis), unit(m.axis));
        let across = ma.cross(ba);
        if across == Vec3::ZERO {
            return Err(unsupported("the branch is parallel to the main pipe"));
        }
        // Where the branch's axis comes nearest the main pipe's, along the branch's axis.
        let w0 = b.origin - m.origin;
        let c = ba.dot(ma);
        let nearest = (c * ma.dot(w0) - ba.dot(w0)) / (1.0 - c * c);
        let along = branch.vertices().iter().map(|v| (v.point - b.origin).dot(ba));
        let [lo, hi] = along.fold([f64::INFINITY, f64::NEG_INFINITY], |[lo, hi], t| [lo.min(t), hi.max(t)]);
        let toward = if nearest > 0.5 * (lo + hi) { ba } else { -ba };
        // Off the main pipe's axis by `offset` across both: the branch's side reaches
        // `offset + radius` from it.
        let offset = w0.dot(unit(across)).abs();
        if offset + radius >= main_radius {
            return Err(unsupported("the branch reaches past the side of the main pipe: it must be narrower, or nearer its axis"));
        }
        Ok(Self { toward, main_axis: ma })
    }
}

/// Coordinates in the axes of development `d`: `x` along its start, `z` along its axis.
fn local(d: &Development, p: Vec3) -> Vec3 {
    let w = p - d.origin;
    Vec3::new(w.dot(d.start), w.dot(d.axis.cross(d.start)), w.dot(d.axis))
}

fn local_vector(d: &Development, v: Vec3) -> Vec3 {
    Vec3::new(v.dot(d.start), v.dot(d.axis.cross(d.start)), v.dot(d.axis))
}

/// The end of `branch`, a shell, cut to sit on the outside of `main`: where the branch's neutral
/// layer meets the main pipe's outer cylinder, the end beyond it gone. `k`: the branch's K-factor.
pub(crate) fn saddle(feature: TopoId, branch: &Body, main: &Body, k: f64) -> Result<Body, BrepError> {
    let shell = Shell::of(branch, k)?;
    let dev = shell.d.dev;
    let [b, _] = walls(branch)?;
    let [_, outer] = walls(main)?;
    let stand = Stand::of(branch, &b, &outer, dev.neutral, outer.radius)?;
    let up = stand.toward.dot(dev.axis) > 0.0;
    let meet = Meet {
        origin: local(&dev, outer.origin),
        axis: local_vector(&dev, stand.main_axis),
        radius: outer.radius,
        own: if up { 1.0 } else { -1.0 },
        turn: None,
    };
    let trace = Trace::Meet(meet);
    let [low, high] = shell.band(trace, shell.span[0], shell.span[1]);
    if low <= shell.ends[0] || high >= shell.ends[1] {
        return Err(unsupported(if (up && low <= shell.ends[0]) || (!up && high >= shell.ends[1]) {
            "the main pipe reaches the branch's other end: make the branch longer"
        } else {
            "the branch does not reach the main pipe: make it longer"
        }));
    }
    // The end beyond the curve goes; a corner within its band is a cut's, which `cut_end` refuses.
    let gone: Vec<bool> = branch.vertices().iter().map(|v| shell.uv(v.point).0.y).map(|v| if up { v > high } else { v < low }).collect();
    shell.cut_end(["saddle.v", "saddle.wall"], feature, trace, &gone, up).map_err(|e| match e {
        BrepError::Unsupported(m) => unsupported(&m),
        e => e,
    })
}

/// A hole in `main`, a shell, for `branch` to sit on it: where the branch's bore meets the main
/// pipe's outer cylinder, cut along the main pipe's radius. `k`: the main pipe's K-factor.
pub(crate) fn hole(feature: TopoId, main: &Body, branch: &Body, k: f64) -> Result<Body, BrepError> {
    let shell = Shell::of(main, k)?;
    let dev = shell.d.dev;
    let [inner, outer] = walls(main)?;
    let [bore, _] = walls(branch)?;
    let stand = Stand::of(branch, &bore, &outer, bore.radius, outer.radius)?;
    // φ from the direction square to both axes.
    let meet = Meet { origin: local(&dev, bore.origin), axis: local_vector(&dev, stand.toward), radius: bore.radius, own: outer.radius, turn: None };
    // Four pieces, a quarter turn of the branch each, `u` within half a turn of the sheet's middle.
    let quarter = std::f64::consts::FRAC_PI_2;
    let piece = |k: usize, near: f64| {
        let trace = Trace::Meet(Meet { turn: Some([quarter * k as f64, quarter * (k + 1) as f64]), ..meet });
        let mut p = Rolled { dev, from: Vec2::new(near, 0.0), to: Vec2::new(near, 0.0), arc: None, trace: Some(trace) };
        p.from = p.at(0.0).0;
        p.to = p.at(1.0).0;
        p
    };
    let middle = 0.5 * (shell.span[0] + shell.span[1]);
    let mut pieces: Vec<Rolled> = Vec::with_capacity(4);
    for k in 0..4 {
        let near = pieces.last().map_or(middle, |p: &Rolled| p.to.x);
        pieces.push(piece(k, near));
    }
    // The hole on the left of its pieces, counterclockwise in the development.
    let area: f64 = pieces.iter().map(|p| p.from.x * p.to.y - p.to.x * p.from.y).sum();
    if area < 0.0 {
        pieces = pieces.iter().rev().map(Rolled::reversed).collect();
    }
    // Clear of the sheet's seams and ends and of every other cut through it.
    let [us, vs] = pieces
        .iter()
        .map(Rolled::uv_box)
        .fold([[f64::INFINITY, f64::NEG_INFINITY]; 2], |[u, v], [pu, pv]| [[u[0].min(pu[0]), u[1].max(pu[1])], [v[0].min(pv[0]), v[1].max(pv[1])]]);
    if us[0] <= shell.span[0] || us[1] >= shell.span[1] {
        return Err(unsupported("the hole reaches the main pipe's seam: turn the seam away"));
    }
    if vs[0] <= shell.ends[0] || vs[1] >= shell.ends[1] {
        return Err(unsupported("the hole reaches the main pipe's end"));
    }
    for face in main.faces() {
        if let Surface::Wall(w) = face.surface {
            let [wu, wv] = w.uv_box();
            if wu[0] <= us[1] && us[0] <= wu[1] && wv[0] <= vs[1] && vs[0] <= wv[1] {
                return Err(unsupported("the hole meets another cut of the main pipe: comes later"));
            }
        }
    }
    // The body: the hole through both cylinders, a wall along each piece.
    let mut vertices = main.vertices().to_vec();
    let mut curves = main.curve_specs();
    let ids: Vec<[TopoId; 2]> =
        (0..4u64).map(|k| [0, 1].map(|layer| TopoId::derive("branch.hole.v").id(feature).index(k).index(layer).finish())).collect();
    for (p, id) in pieces.iter().zip(&ids) {
        vertices.push(Vertex { id: id[0], point: dev.point(p.from, outer.radius) });
        vertices.push(Vertex { id: id[1], point: dev.point(p.from, inner.radius) });
    }
    let mut faces: Vec<FaceSpec> = (0..main.faces().len() as u32).map(|f| main.face_spec(f)).collect();
    for spec in &mut faces {
        match spec.surface {
            // Holes run clockwise seen from outside: against the pieces outside, along them inside.
            Surface::Cylinder(c) if c.radius == outer.radius => spec.loops.push(ids.iter().rev().map(|v| v[0]).collect()),
            Surface::Cylinder(c) if c.radius == inner.radius => spec.loops.push(ids.iter().map(|v| v[1]).collect()),
            _ => {}
        }
    }
    for (k, p) in pieces.iter().enumerate() {
        let (a, b) = (ids[k], ids[(k + 1) % 4]);
        for (layer, radius) in [(0, outer.radius), (1, inner.radius)] {
            curves.push(CurveSpec { from: a[layer], to: b[layer], curve: Curve::Rolled { piece: *p, radius } });
        }
        let id = TopoId::derive("branch.hole.wall").id(feature).index(k as u64).finish();
        faces.push(FaceSpec { id, surface: Surface::Wall(*p), loops: vec![vec![a[0], b[0], b[1], a[1]]] });
    }
    build_with_curves(vertices, faces, curves)
}
