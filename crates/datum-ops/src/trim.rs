//! Trimming rolled sheet metal by a plane (D211): the part on one side of a work plane goes, as
//! a laser cuts the blank before rolling. The cut follows where the plane meets the sheet's
//! neutral layer, a wave in the development; its wall stands square to the sheet, along the
//! radius. For now the sheet is a shell (one cylinder each side) and the plane crosses it from
//! seam to seam, cutting off one end whole.

use std::collections::BTreeMap;

use datum_brep::{Body, BrepError, Curve, CurveSpec, FaceSpec, Rolled, Surface, TopoId, Trace, Vertex, Wave, build_with_curves};
use datum_math::{Sign, Vec2, Vec3, orient3d_coords};

use crate::sketch::{Developed, SketchPlane, unrolled};

fn unsupported(m: &str) -> BrepError {
    BrepError::Unsupported(format!("trim: {m}"))
}

/// `body`, a rolled shell, trimmed by `plane`: what lies on the side its normal points to goes
/// (`flip`: the other side). `k`: the K-factor of the sheet's neutral layer.
pub(crate) fn trim(feature: TopoId, body: &Body, plane: &SketchPlane, flip: bool, k: f64) -> Result<Body, BrepError> {
    let shell = Shell::of(body, k)?;
    let dev = shell.d.dev;
    let n_radius = dev.neutral;
    let [o, x, y, n] = plane.axes()?;
    let along = n.dot(dev.axis);
    if along == 0.0 {
        return Err(unsupported("a plane along the axis cuts the sheet lengthwise: comes later"));
    }
    // The plane on the neutral layer: v(u) where n·(origin + axis·v + N·radial(u)) = n·o.
    let wave =
        Wave { a: n.dot(o - dev.origin) / along, b: -n_radius * n.dot(dev.start) / along, c: -n_radius * n.dot(dev.axis.cross(dev.start)) / along };
    let trace = Trace::Wave(wave);
    shell.crosses(trace)?;
    // Each vertex's side, exactly; the side the normal points to goes, unless flipped.
    let side = |p: Vec3| orient3d_coords(o, o + x, o + y, p);
    let mut gone = vec![false; body.vertices().len()];
    for (i, v) in body.vertices().iter().enumerate() {
        gone[i] = match side(v.point) {
            Sign::Zero => return Err(BrepError::Degenerate("trim: a corner of the sheet lies on the plane".into())),
            s => (s == Sign::Negative) != flip,
        };
    }
    // The end that goes is wholly on the far side of the wave: `+v` or `-v` of it.
    let high = shell.band(trace, shell.span[0], shell.span[1])[1];
    let up = {
        let far = body.vertices().iter().enumerate().find(|(i, _)| gone[*i]).map(|(_, v)| shell.uv(v.point).0.y);
        far.ok_or_else(|| unsupported("nothing of the sheet is on the side to cut off"))? > high
    };
    shell.cut_end(["trim.v", "trim.wall"], feature, trace, &gone, up).map_err(|e| match e {
        BrepError::Unsupported(m) => unsupported(&m),
        e => e,
    })
}

/// A rolled shell (one cylinder each side of the sheet) in its development, to cut an end off
/// along a trace of `v` of `u` on its neutral layer (a trim's wave, D211; a branch's end, D227).
pub(crate) struct Shell<'a> {
    pub(crate) body: &'a Body,
    pub(crate) d: Developed,
    /// The two cylinders' radii.
    pub(crate) radii: [f64; 2],
    /// From seam to seam in `u`.
    pub(crate) span: [f64; 2],
    /// The least and greatest `v` of the sheet's corners.
    pub(crate) ends: [f64; 2],
}

impl<'a> Shell<'a> {
    pub(crate) fn of(body: &'a Body, k: f64) -> Result<Self, BrepError> {
        let faces = body.faces();
        if faces.iter().any(|f| matches!(f.surface, Surface::Cone(_))) {
            return Err(unsupported("a cone comes later"));
        }
        let cylinders: Vec<usize> = (0..faces.len()).filter(|&f| matches!(faces[f].surface, Surface::Cylinder(_))).collect();
        let [first, second] = cylinders[..] else { return Err(unsupported("the body is no rolled shell: one cylinder each side of the sheet")) };
        let (Surface::Cylinder(c), Surface::Cylinder(c2)) = (faces[first].surface, faces[second].surface) else {
            return Err(unsupported("no cylinder"));
        };
        let d = unrolled(body, first, &c, k)?;
        let span = [0.0, d.span * d.dev.neutral];
        let mut shell = Self { body, d, radii: [c.radius, c2.radius], span, ends: [0.0; 2] };
        shell.ends =
            body.vertices().iter().map(|v| shell.uv(v.point).0.y).fold([f64::INFINITY, f64::NEG_INFINITY], |[lo, hi], v| [lo.min(v), hi.max(v)]);
        Ok(shell)
    }

    /// The development's coordinates of `p` and its distance from the axis.
    pub(crate) fn uv(&self, p: Vec3) -> (Vec2, f64) {
        self.d.dev.unroll(p, 0.5 * (self.span[0] + self.span[1]))
    }

    /// The piece of `trace` from `u0` to `u1`.
    pub(crate) fn piece(&self, trace: Trace, u0: f64, u1: f64) -> Result<Rolled, BrepError> {
        let n = self.d.dev.neutral;
        let v = |u: f64| trace.v(u, n).ok_or_else(|| BrepError::Invalid("an end is cut along a curve of v of u".into()));
        Ok(Rolled { dev: self.d.dev, from: Vec2::new(u0, v(u0)?), to: Vec2::new(u1, v(u1)?), arc: None, trace: Some(trace) })
    }

    /// The least and greatest `v` of `trace` from `u0` to `u1`.
    pub(crate) fn band(&self, trace: Trace, u0: f64, u1: f64) -> [f64; 2] {
        self.piece(trace, u0, u1).map_or([f64::NAN; 2], |p| p.uv_box()[1])
    }

    /// `trace` crosses the sheet from seam to seam between its ends, or it is refused.
    pub(crate) fn crosses(&self, trace: Trace) -> Result<(), BrepError> {
        let [low, high] = self.band(trace, self.span[0], self.span[1]);
        if low <= self.ends[0] || high >= self.ends[1] {
            return Err(unsupported(if high < self.ends[0] || low > self.ends[1] {
                "the plane misses the sheet"
            } else {
                "the plane leaves the sheet through its end: comes later"
            }));
        }
        Ok(())
    }

    /// The shell with the end beyond `trace` cut off: the vertices `gone` (by the body's order),
    /// on its `+v` side if `up`. The cut's wall stands along the radius. `ids`: the derivations of
    /// the new vertices and the wall.
    pub(crate) fn cut_end(&self, names: [&str; 2], feature: TopoId, trace: Trace, gone: &[bool], up: bool) -> Result<Body, BrepError> {
        let (body, faces, span, n_radius) = (self.body, self.body.faces(), self.span, self.d.dev.neutral);
        let [r0, r1] = self.radii;
        let uv = |p: Vec3| self.uv(p);
        let index = |id: TopoId| body.vertices().binary_search_by_key(&id, |v| v.id).map_err(|_| BrepError::Invalid(format!("trim: vertex {id:?}")));
        // Holes and cuts stay clear of the wave, or the trim is refused.
        for (f, face) in faces.iter().enumerate() {
            let Surface::Wall(w) = face.surface else { continue };
            let ids: Vec<TopoId> = body.face_spec(f as u32).loops.concat();
            let all_gone = ids.iter().map(|&id| index(id).map(|i| gone[i])).collect::<Result<Vec<_>, _>>()?;
            if all_gone.iter().any(|&g| g) {
                return Err(unsupported("a cut in the part cut off, or across the line of the cut: comes later"));
            }
            let [us, vs] = w.uv_box();
            let [lo, hi] = self.band(trace, us[0], us[1]);
            if (up && vs[1] >= lo) || (!up && vs[0] <= hi) {
                return Err(unsupported("a cut reaches the line of the cut: comes later"));
            }
        }
        let point = |u: f64, r: f64| self.d.dev.point(Vec2::new(u, trace.v(u, n_radius).unwrap_or(f64::NAN)), r);
        // Where an edge from a kept vertex to a gone one (along the axis) meets the wave.
        let mut crossings: BTreeMap<(TopoId, TopoId), (TopoId, f64, f64)> = BTreeMap::new();
        let mut vertices: Vec<Vertex> = body.vertices().iter().enumerate().filter(|(i, _)| !gone[*i]).map(|(_, v)| *v).collect();
        let mut cross = |a: TopoId, b: TopoId| -> Result<TopoId, BrepError> {
            let key = (a.min(b), a.max(b));
            if let Some(&(id, ..)) = crossings.get(&key) {
                return Ok(id);
            }
            let kept = if gone[index(a)?] { b } else { a };
            // The edge runs along a seam, on the inside or the outside: the seam's `u` and that
            // face's radius, as the shell has them, not as unrolling the vertex rounds them.
            let (p, r) = uv(body.vertices()[index(kept)?].point);
            let u = if (p.x - span[0]).abs() <= (p.x - span[1]).abs() { span[0] } else { span[1] };
            let r = if (r - r0).abs() <= (r - r1).abs() { r0 } else { r1 };
            let p = Vec2::new(u, p.y);
            let id = TopoId::derive(names[0]).id(feature).id(key.0).id(key.1).finish();
            vertices.push(Vertex { id, point: point(p.x, r) });
            crossings.insert(key, (id, p.x, r));
            Ok(id)
        };
        let mut specs = Vec::with_capacity(faces.len() + 1);
        // The new edges each face got along the cut, as it runs them: its surface, from, to.
        let mut cut_edges: Vec<(Surface, TopoId, TopoId)> = Vec::new();
        for f in 0..faces.len() as u32 {
            let mut spec = body.face_spec(f);
            let mut loops = Vec::with_capacity(spec.loops.len());
            for (k, l) in spec.loops.iter().enumerate() {
                let out: Vec<bool> = l.iter().map(|&id| index(id).map(|i| gone[i])).collect::<Result<_, _>>()?;
                if out.iter().all(|&g| g) {
                    if k == 0 {
                        break;
                    }
                    continue;
                }
                if !out.iter().any(|&g| g) {
                    loops.push(l.clone());
                    continue;
                }
                let m = l.len();
                let mut ring = Vec::with_capacity(m);
                // Into the run cut off, and out of a run begun before the loop's start.
                let (mut entry, mut first_exit) = (None, None);
                for i in 0..m {
                    let (a, b) = (l[i], l[(i + 1) % m]);
                    if !out[i] {
                        ring.push(a);
                    }
                    if out[i] != out[(i + 1) % m] {
                        let c = cross(a, b)?;
                        ring.push(c);
                        match (out[i], entry.take()) {
                            (false, _) => entry = Some(c),
                            (true, Some(e)) => cut_edges.push((spec.surface, e, c)),
                            (true, None) => first_exit = Some(c),
                        }
                    }
                }
                if let (Some(e), Some(c)) = (entry, first_exit) {
                    cut_edges.push((spec.surface, e, c));
                }
                loops.push(ring);
            }
            if loops.is_empty() {
                continue;
            }
            spec.loops = loops;
            specs.push(spec);
        }
        // The wall along the cut: each new edge back, chained.
        let mut next: BTreeMap<TopoId, TopoId> = cut_edges.iter().map(|&(_, a, b)| (b, a)).collect();
        let start = *next.keys().next().ok_or_else(|| BrepError::Invalid("trim: no cut".into()))?;
        let mut wall = vec![start];
        while let Some(v) = next.remove(wall.last().unwrap_or(&start)) {
            if v == start {
                break;
            }
            wall.push(v);
        }
        if !next.is_empty() || wall.len() != cut_edges.len() {
            return Err(BrepError::Invalid("trim: the cut does not close".into()));
        }
        // The wall's piece runs with the part cut off on its left (D68): `+u` if it is `+v`.
        let wall_piece = if up { self.piece(trace, span[0], span[1])? } else { self.piece(trace, span[1], span[0])? };
        specs.push(FaceSpec { id: TopoId::derive(names[1]).id(feature).finish(), surface: Surface::Wall(wall_piece), loops: vec![wall] });
        // Curves: the body's, but those of edges cut off; the wave on each cylinder.
        let kept = |id: TopoId| index(id).is_ok_and(|i| !gone[i]);
        let mut curves: Vec<CurveSpec> = body.curve_specs().into_iter().filter(|s| kept(s.from) && kept(s.to)).collect();
        let at = |id: TopoId| crossings.values().find(|c| c.0 == id).map(|c| (c.1, c.2));
        for (surface, a, b) in &cut_edges {
            if let (Surface::Cylinder(_), Some((u0, r)), Some((u1, _))) = (surface, at(*a), at(*b)) {
                curves.push(CurveSpec { from: *a, to: *b, curve: Curve::Rolled { piece: self.piece(trace, u0, u1)?, radius: r } });
            }
        }
        build_with_curves(vertices, specs, curves)
    }
}
