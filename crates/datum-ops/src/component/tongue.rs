//! The first two-sketch stamp (D279–D281): a straight tongue on a plane wall.

use std::collections::BTreeMap;

use datum_brep::{Body, BrepError, ProfileLoop, Region, TopoId, extrude, segments_touch};
use datum_math::expr::Num;
use datum_math::{Rigid, Vec2, Vec3};
use datum_sketch::Step;

use super::stitch::{Window, stitch};
use super::{Forming, PartDef, STAMP_ROLE, SectionSide, at, evaluate_features, on_wall, section, unit};
use crate::rib::Formed;
use crate::sheet::{Sheet, Side};
use crate::sketch::{Placed, within_rounding};
use crate::unfold::StampCut;
use crate::{Op, Placement, Value, input_sketch, num, validated};

fn invalid(message: impl std::fmt::Display) -> BrepError {
    BrepError::Invalid(format!("component: {message}"))
}

fn unsupported(message: impl std::fmt::Display) -> BrepError {
    BrepError::Unsupported(format!("component: {message}"))
}

fn points<const N: usize>(sketch: &Placed, names: [&str; N]) -> [Vec2; N] {
    names.map(|n| sketch.solved.points[n])
}

fn to_part(sketch: &Placed, p: Vec2) -> Vec3 {
    let [o, x, y, _] = sketch.frame;
    o + x * p.x + y * p.y
}

/// Get the boundary loop in order, beginning at the transition.
fn boundary_loop<'a>(
    name: &str,
    sketch: &'a Placed,
    metadata: &BTreeMap<String, BTreeMap<String, serde_json::Value>>,
) -> Result<[Step<'a>; 4], BrepError> {
    let mut root = None;
    for (curve, construction) in sketch.lines.iter().map(|(n, l)| (n, l.construction)).chain(sketch.arcs.iter().map(|(n, a)| (n, a.construction))) {
        let role = metadata.get(curve).and_then(|p| p.get(STAMP_ROLE));
        match role.and_then(serde_json::Value::as_str) {
            Some("cut") => {}
            Some("transition") => {
                if let Some(previous) = root.replace(curve) {
                    return Err(unsupported(format!("boundary `{name}` has two transitions `{previous}` and `{curve}`")));
                }
            }
            None if role.is_none() && construction => continue,
            None if role.is_none() => return Err(invalid(format!("boundary `{name}`: curve `{curve}` needs a {STAMP_ROLE}"))),
            _ => return Err(invalid(format!("boundary `{name}`: curve `{curve}` needs {STAMP_ROLE} set to cut or transition"))),
        }
        if sketch.arcs.contains_key(curve) {
            return Err(unsupported(format!("boundary `{name}`: arc `{curve}` in a straight tongue")));
        }
    }
    let root = root.ok_or_else(|| invalid(format!("boundary `{name}` needs a transition")))?;
    let keep = |n: &str, _| metadata.get(n).is_some_and(|p| p.contains_key(STAMP_ROLE));
    let contours = sketch.contours(&keep);
    if let Some(p) = contours.odd() {
        return Err(invalid(format!("boundary `{name}` is open or branches at point `{p}`")));
    }
    if sketch.circles.values().any(|c| !c.construction) || sketch.ellipses.values().any(|e| !e.construction) || !sketch.patterns.is_empty() {
        return Err(unsupported(format!("boundary `{name}` must be a loop of marked lines")));
    }
    let [ring] = contours.loops.as_slice() else { return Err(invalid(format!("boundary `{name}` needs one loop"))) };
    if ring.len() != 4 {
        return Err(unsupported(format!("boundary `{name}` of a straight tongue needs four lines, got {}", ring.len())));
    }
    let start = ring.iter().position(|s| s.curve == root).ok_or_else(|| invalid(format!("boundary `{name}` has no transition `{root}`")))?;
    let ring = std::array::from_fn(|i| ring[(i + start) % 4]);
    let [a, b, c, d] = points(sketch, ring.map(|s| s.from));
    for (u, v) in [(0, 2), (1, 3)] {
        let p = [a, b, c, d];
        if segments_touch(p[u], p[(u + 1) % 4], p[v], p[(v + 1) % 4]) {
            return Err(BrepError::Degenerate(format!("component: boundary `{name}`: `{}` touches `{}`", ring[u].curve, ring[v].curve)));
        }
    }
    Ok(ring)
}

fn section_chain<'a>(name: &str, sketch: &'a Placed) -> Result<[Step<'a>; 3], BrepError> {
    let contours = sketch.contours(&|_, construction| !construction);
    if let Some(p) = contours.branches.first() {
        return Err(invalid(format!("section `{name}` branches at point `{p}`")));
    }
    if !contours.loops.is_empty()
        || sketch.circles.values().any(|c| !c.construction)
        || sketch.ellipses.values().any(|e| !e.construction)
        || !sketch.patterns.is_empty()
    {
        return Err(unsupported(format!("section `{name}` needs one open chain")));
    }
    let [path] = contours.paths.as_slice() else { return Err(invalid(format!("section `{name}` needs one connected chain"))) };
    let Ok(path) = <[Step<'_>; 3]>::try_from(path.as_slice()) else {
        return Err(unsupported(format!("section `{name}` of a straight tongue needs three lines, got {}", path.len())));
    };
    if let Some(step) = path.iter().find(|s| sketch.arcs.contains_key(s.curve)) {
        return Err(unsupported(format!("section `{name}`: arc `{}` in a straight tongue", step.curve)));
    }
    Ok(path)
}

/// Replace the material of a plane wall by the tongue (D290–D291).
pub(crate) fn form(
    feature: TopoId,
    def: &PartDef,
    forming: &Forming,
    params: &BTreeMap<String, Num>,
    placements: &[Placement],
    rule: [Option<&Num>; 3],
    values: &[&Value],
) -> Result<Value, BrepError> {
    let (wall, strip, window, cut) = prepare(feature, def, forming, params, placements, rule, values)?;
    let body = stitch(feature, wall, &strip, &window)?;
    validated(&body)?;
    Ok(Value::Formed(Box::new(Formed { body, blank: wall.clone(), imprints: Vec::new(), stamp: Some(cut) })))
}

/// Evaluate the raised strip, the opening that supplies its metal, and its three free cuts.
fn prepare<'a>(
    feature: TopoId,
    def: &PartDef,
    forming: &Forming,
    params: &BTreeMap<String, Num>,
    placements: &[Placement],
    [thickness, radius, k_factor]: [Option<&Num>; 3],
    values: &[&'a Value],
) -> Result<(&'a Body, Body, Window, StampCut), BrepError> {
    let [Placement::Point(placement)] = placements else {
        return Err(unsupported(format!("a straight tongue needs one point placement: part `{}`", def.name)));
    };
    let [Value::Body(wall), _] = values else { return Err(unsupported("a straight tongue needs an unformed sheet and a placement sketch")) };
    let on = input_sketch(values, placement.input, "a straight tongue's placement")?;
    on_wall(def, wall, on, "tongue")?;
    let t = num(thickness.ok_or_else(|| invalid("a straight tongue needs a thickness from the sheet rule"))?)?;
    if t <= 0.0 || !t.is_finite() {
        return Err(invalid("a straight tongue needs positive sheet thickness"));
    }
    if let Some(range) = &def.thickness
        && !(num(&range[0])?..=num(&range[1])?).contains(&t)
    {
        return Err(invalid(format!("part `{}`: sheet thickness {t} mm is outside its thickness range", def.name)));
    }
    let typed = |name: &str| {
        def.features
            .iter()
            .enumerate()
            .find_map(|(i, f)| match &f.op {
                Op::Sketch { sketch, .. } if f.name == name => Some((i, sketch)),
                _ => None,
            })
            .ok_or_else(|| invalid(format!("part `{}` has no sketch `{name}`", def.name)))
    };
    let (profile_i, _) = typed(&forming.section)?;
    let (boundary_i, boundary_def) = typed(&forming.boundary)?;
    let made = evaluate_features(def, feature, params)?;
    let solved = |i: usize| made[i].sketch().ok_or_else(|| invalid(format!("feature `{}` is no sketch", def.features[i].name)));
    let profile = solved(profile_i)?;
    let boundary = solved(boundary_i)?;
    let ring = boundary_loop(&forming.boundary, boundary, &boundary_def.metadata)?;
    let mut chain = section_chain(&forming.section, profile)?;
    let anchor = def.anchor()?;
    let [b0, b1, b2, b3] = points(boundary, ring.map(|s| s.from)).map(|p| to_part(boundary, p) - anchor);
    let z = boundary.frame[3];
    let y = unit(b1 - b0);
    let toward_tip = b3 - b0;
    let x = if y.cross(z).dot(toward_tip) > 0.0 { y.cross(z) } else { z.cross(y) };
    let width = z.cross(x);
    let run = b3.dot(x);
    let lo = b0.dot(width).min(b1.dot(width));
    let hi = b0.dot(width).max(b1.dot(width));
    let scale = [b0, b1, b2, b3, anchor].iter().flat_map(|p| p.to_array()).fold(1.0_f64, |s, x| s.max(x.abs()));
    let zero = |x| within_rounding(x, scale);
    if !(run > 0.0 && lo < 0.0 && 0.0 < hi)
        || ![b0.dot(z), b0.dot(x), b1.dot(x), (b2 - b1).dot(width), (b3 - b0).dot(width), b2.dot(x) - run].into_iter().all(zero)
    {
        return Err(unsupported(format!(
            "boundary `{}` needs a rectangle with its transition `{}` through the anchor",
            forming.boundary, ring[0].curve
        )));
    }
    let positions = |chain: [Step<'_>; 3]| {
        let [a, b, c] = chain;
        points(profile, [a.from, a.to, b.to, c.to]).map(|p| to_part(profile, p) - anchor)
    };
    let mut pos = positions(chain);
    let at_anchor = |v: Vec3| v.to_array().into_iter().all(zero);
    if at_anchor(pos[3]) {
        let [a, b, c] = chain;
        chain = [c, b, a].map(|s| Step { from: s.to, to: s.from, forward: !s.forward, ..s });
        pos = positions(chain);
    }
    let mut front = pos.map(|p| Vec2::new(p.dot(x), p.dot(z)));
    if !at_anchor(pos[0])
        || !pos.iter().all(|p| zero(p.dot(width)))
        || !(zero(front[1].x) && front[1].y > 0.0 && front[2].x > 0.0 && front[3].x > front[2].x && zero(front[3].x - run))
    {
        return Err(unsupported(format!(
            "section `{}` needs a root wall at the anchor, in the plane across transition `{}`, and a tip at the far cut",
            forming.section, ring[0].curve
        )));
    }
    // These are the anchor and the vertical root line after removing frame roundoff.
    front[0] = Vec2::default();
    front[1].x = 0.0;
    let inner = forming.section_side == SectionSide::Inner;
    // Extrusion coordinates are section X and Z, and the negative transverse direction.
    let nodes = [chain[0].from, chain[0].to, chain[1].to, chain[2].to];
    let lines = chain.map(|s| s.curve);
    let id = |kind, side: &str, name: &str| TopoId::derive(kind).id(feature).tag(side).tag(name).finish();
    let root_cap = id("stamp.surface", "root", ring[0].curve);
    let (outer, [bottom, top], [outer_root, inner_root], far, root_cross) = if forming.round_section {
        let radius = num(radius.ok_or_else(|| invalid("a rounded tongue needs a bend radius from the sheet rule"))?)?;
        let k = k_factor.map(num).transpose()?.unwrap_or(0.5);
        let sheet = Sheet { thickness: t, radius, side: if inner { Side::Left } else { Side::Right } };
        let (outer, length, cross) = section::rounded(feature, front, nodes, lines, root_cap, sheet, k)?;
        let last = outer.points.len() - 1;
        let root = outer.points[0].x;
        (outer, [0, last], [root, root], root + length, Some(cross))
    } else {
        let (outer, roots) = sharp_profile(feature, forming, front, nodes, lines, [ring[0].curve, ring[2].curve], t)?;
        let [outer_root, inner_root] = roots;
        // At the root the parent plate still occupies the strip up to its inside edge.
        // Take only the raised strip's volume out of the wall beyond that edge (D291).
        let far = inner_root + outer.area().abs() / t;
        (outer, if inner { [0, 7] } else { [7, 0] }, [outer_root, inner_root], far, None)
    };
    let [top_hi, top_lo] = outer.vertices[top];
    let [bottom_hi, bottom_lo] = outer.vertices[bottom];
    let root_ids = [top_lo, top_hi, bottom_lo, bottom_hi];
    if !far.is_finite() || outer.points.iter().any(|p| p.x >= far) {
        return Err(unsupported(format!("section `{}` does not fit over its developed window", forming.section)));
    }
    let left = if b0.dot(width) < b1.dot(width) { ring[3].curve } else { ring[1].curve };
    let right = if left == ring[3].curve { ring[1].curve } else { ring[3].curve };
    let strip = extrude(&[Region { outer, holes: Vec::new(), caps: [id("stamp.cap", "", right), id("stamp.cap", "", left)] }], -hi, -lo)?;
    let origin = at(def, placement, values)?;
    let [_, bx, by, bz] = boundary.frame;
    let [_, wx, wy, wz] = on.frame;
    let onto_wall = Rigid::into_frame(Vec3::ZERO, bx, by, bz).then(Rigid::frame(origin, wx, wy, wz));
    let strip = strip.transformed(&Rigid::frame(Vec3::ZERO, x, z, -width).then(onto_wall));
    validated(&strip)?;
    let to_world = Rigid::frame(Vec3::ZERO, x, width, z).then(onto_wall);
    let face = on.face.ok_or_else(|| invalid("a tongue needs a placement face"))?;
    let window = Window { face, thickness: t, root_cap, root_ids, root_cross, to_world, sides: [lo, hi], inner_root, outer_root, far };
    let points = [(inner_root, lo), (far, lo), (far, hi), (inner_root, hi)].map(|(x, y)| to_world.apply(Vec3::new(x, y, 0.0)));
    let cut = StampCut { points, normal: wz, ids: [left, ring[2].curve, right].map(|curve| id("stamp.cut", "", curve)) };
    Ok((wall, strip, window, cut))
}

fn sharp_profile(
    feature: TopoId,
    forming: &Forming,
    mut front: [Vec2; 4],
    nodes: [&str; 4],
    [a, b, c]: [&str; 3],
    [root, tip]: [&str; 2],
    t: f64,
) -> Result<(ProfileLoop, [f64; 2]), BrepError> {
    let inner = forming.section_side == SectionSide::Inner;
    let mut back = section::offset(front, if inner { t } else { -t })
        .map_err(|e| BrepError::Degenerate(format!("component: section `{}` (`{a}`, `{b}`, `{c}`): {e}", forming.section)))?;
    let (outer_root, inner_root) = if inner { (back[0].x, front[0].x) } else { (front[0].x, back[0].x) };
    // The strip is the metal above the wall. For an inner reference, the vertical root
    // overlaps the slab up to its top plane. The cut wall owns that part of the root.
    if inner {
        for p in front.iter_mut().chain(back.iter_mut()) {
            p.y -= t;
        }
    }
    front[0].y = 0.0;
    back[0].y = 0.0;
    if front[1..].iter().chain(back[1..].iter()).any(|p| p.y <= 0.0) {
        return Err(unsupported(format!("section `{}` meets the wall beyond its root", forming.section)));
    }
    let id = |kind, side: &str, name: &str| TopoId::derive(kind).id(feature).tag(side).tag(name).finish();
    let outer = ProfileLoop {
        points: front.into_iter().chain(back.into_iter().rev()).collect(),
        vertices: ["reference", "offset"]
            .into_iter()
            .zip([nodes, [nodes[3], nodes[2], nodes[1], nodes[0]]])
            .flat_map(|(side, nodes)| nodes.map(|n| [id("stamp.vertex.low", side, n), id("stamp.vertex.high", side, n)]))
            .collect(),
        sides: [("reference", a), ("reference", b), ("reference", c), ("tip", tip), ("offset", c), ("offset", b), ("offset", a), ("root", root)]
            .map(|(side, name)| id("stamp.surface", side, name))
            .into(),
        arcs: Vec::new(),
    };
    Ok((outer, [outer_root, inner_root]))
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::panic)]

    use super::*;
    use serde_json::{Value as Json, json};

    fn fixture(thickness: f64) -> (Value, Value, Json) {
        let model: Json = serde_json::from_str(include_str!("../../../../tests/models/stamp-tongue-straight.json")).unwrap();
        let [outline, plate, places, stamp, ..] = model["features"].as_array().unwrap().as_slice() else { panic!("unexpected fixture") };
        let eval = |f: &Json, inputs: &[&Value]| {
            let id = TopoId::derive("feature").tag(f["name"].as_str().unwrap()).finish();
            let op = serde_json::from_value::<Op>(operation(f)).unwrap();
            op.eval(id, inputs).unwrap()
        };
        let outline = eval(outline, &[]);
        let mut sheet = plate.clone();
        sheet["thickness"] = json!(thickness);
        let plate = eval(&sheet, &[&outline]);
        let places = eval(places, &[&plate]);
        (plate, places, operation(stamp))
    }

    #[test]
    fn the_tongue_uses_the_wall_frame_on_a_rotated_plate() {
        let (plate, places, json) = fixture(2.0);
        let turn = Rigid::rotation(Vec3::new(1.0, 2.0, 3.0), 0.37).then(Rigid::translation(Vec3::new(40.0, 10.0, 15.0)));
        let original = plate.body().unwrap();
        let plate = Value::Body(original.transformed(&turn));
        let Value::Sketch(mut places) = places else { panic!("expected placement sketch") };
        let [o, x, y, z] = places.frame;
        places.frame = [turn.apply(o), turn.apply_vector(x), turn.apply_vector(y), turn.apply_vector(z)];
        let op = serde_json::from_value::<Op>(json).unwrap();
        let Value::Formed(formed) = op.eval(stamp(), &[&plate, &Value::Sketch(places)]).unwrap() else { panic!("expected formed sheet") };
        formed.body.validate().unwrap();
        assert!(within_rounding(formed.body.volume() - original.volume(), original.volume()));
    }

    fn operation(feature: &Json) -> Json {
        let mut op = feature.as_object().unwrap().clone();
        op.remove("name");
        op.remove("inputs");
        Json::Object(op)
    }

    fn stamp() -> TopoId {
        TopoId::derive("feature").tag("stamp").finish()
    }

    #[test]
    fn the_two_profiles_build_a_swept_tongue_on_either_reference_side() {
        for (side, t) in [("outer", 2.0), ("inner", 1.0)] {
            let (plate, places, mut json) = fixture(t);
            let inputs = [&plate, &places];
            let top = places.sketch().unwrap().frame[0].z;
            json["part"]["forming"]["section_side"] = json!(side);
            json["thickness"] = json!(t);
            let op = serde_json::from_value::<Op>(json.clone()).unwrap();
            let Op::Component { part, params, placements, thickness, .. } = &op else { panic!("expected component") };
            part.validate().unwrap();
            let (_, body, _, _) =
                prepare(stamp(), part, part.forming.as_ref().unwrap(), params, placements, [thickness.as_ref(), None, None], &inputs).unwrap();
            body.validate().unwrap();
            datum_brep::tessellate(&body).unwrap();
            datum_brep::step(&[&body], "tongue", false).unwrap();
            assert!(body.volume() > 0.0);
            let reference = TopoId::derive("stamp.surface").id(stamp()).tag("reference").tag("shelf").finish();
            let offset = TopoId::derive("stamp.surface").id(stamp()).tag("offset").tag("shelf").finish();
            let z = |id| body.faces().iter().find(|f| f.id == id).unwrap().surface.plane().unwrap().origin.z;
            assert_eq!(z(reference), if side == "inner" { top + 3.0 - t } else { top + 3.0 });
            assert_eq!((z(reference) - z(offset)).abs(), t);
            let Value::Formed(formed) = op.eval(stamp(), &inputs).unwrap() else { panic!("expected formed sheet") };
            formed.body.validate().unwrap();
            datum_brep::tessellate(&formed.body).unwrap();
            datum_brep::step(&[&formed.body], "stamped plate", false).unwrap();
            let original_volume = plate.body().unwrap().volume();
            assert!(within_rounding(formed.body.volume() - original_volume, original_volume));
        }
    }

    #[test]
    fn rounded_tongue_has_three_concentric_tangent_bends_on_either_reference_side() {
        use datum_brep::Surface;

        let length = |v: Vec3| libm::sqrt(v.dot(v));
        for (side, t) in [("outer", 1.0), ("inner", 0.75)] {
            let (plate, places, mut json) = fixture(t);
            json["part"]["forming"]["round_section"] = json!(true);
            json["part"]["forming"]["section_side"] = json!(side);
            json["thickness"] = json!(t);
            json["radius"] = json!(0.4);
            let op: Op = serde_json::from_value(json.clone()).unwrap();
            let inputs = [&plate, &places];
            let value = op.eval(stamp(), &inputs).unwrap();
            let Value::Formed(formed) = value else { panic!("expected formed sheet") };
            let body = &formed.body;
            for point in formed.stamp.as_ref().unwrap().points {
                assert!(body.vertices().iter().any(|v| within_rounding(length(v.point - point), t)));
            }
            body.validate().unwrap();
            datum_brep::tessellate(body).unwrap();
            datum_brep::step(&[body], "rounded tongue", false).unwrap();
            let original = plate.body().unwrap().volume();
            assert!(within_rounding(body.volume() - original, original));

            let Op::Component { part, placements, .. } = &op else { panic!("expected component") };
            let [Placement::Point(placement)] = placements.as_slice() else { panic!("expected point placement") };
            let [_, x, _, z] = places.sketch().unwrap().frame;
            // The inside reference is anchored on the underside of the sheet (D288).
            let anchor = at(part, placement, &inputs).unwrap() - z * if side == "inner" { t } else { 0.0 };
            for (line, point) in [("shelf", "shoulder"), ("nose", "nose_tip")] {
                let local: [f64; 2] = serde_json::from_value(json["part"]["features"][0]["op"]["sketch"]["points"][point].clone()).unwrap();
                let on_sketch = anchor + x * local[0] + z * local[1];
                let id = TopoId::derive("sheet.face").id(stamp()).tag(line).index(u64::from(side == "outer")).finish();
                let plane = body.faces().iter().find(|f| f.id == id).unwrap().surface.plane().unwrap();
                assert!(within_rounding((on_sketch - plane.origin).dot(unit(plane.normal)), t));
            }

            let cylinders: Vec<_> = body
                .faces()
                .iter()
                .filter_map(|f| match f.surface {
                    Surface::Cylinder(c) => Some(c),
                    _ => None,
                })
                .collect();
            assert_eq!(cylinders.len(), 6);
            for inner in cylinders.iter().filter(|c| within_rounding(c.radius - 0.4, t)) {
                let matches = cylinders.iter().filter(|outer| {
                    within_rounding(outer.radius - inner.radius - t, t) && within_rounding(length((outer.origin - inner.origin).cross(inner.axis)), t)
                });
                assert_eq!(matches.count(), 1);
            }
            assert_eq!(cylinders.iter().filter(|c| within_rounding(c.radius - 0.4, t)).count(), 3);

            let mut tangencies = 0;
            for edge in body.edges().iter().filter(|edge| body.edge_curve(edge).is_none()) {
                let a = body.half_edges()[edge.half as usize];
                let b = body.half_edges()[a.twin as usize];
                let surface = |half: datum_brep::HalfEdge| body.faces()[body.loops()[half.loop_ix as usize].face as usize].surface;
                let (cyl, plane) = match (surface(a), surface(b)) {
                    (Surface::Cylinder(c), Surface::Plane(p)) | (Surface::Plane(p), Surface::Cylinder(c)) => (c, p),
                    _ => continue,
                };
                let v = body.point(a.origin) - cyl.origin;
                let radial = v - cyl.axis * v.dot(cyl.axis);
                assert!(within_rounding(length(radial.cross(unit(plane.normal))), t));
                tangencies += 1;
            }
            assert_eq!(tangencies, 12);

            json["radius"] = json!(1.25);
            let op: Op = serde_json::from_value(json).unwrap();
            let error = op.eval(stamp(), &inputs).err().unwrap();
            assert!(matches!(error, BrepError::Degenerate(message) if message.contains("root_wall")));
        }
    }

    #[test]
    fn a_boundary_role_is_required_and_only_roles_affect_the_component_key() {
        let (plate, places, mut json) = fixture(2.0);
        let hash = |json: &Json| {
            let mut h = blake3::Hasher::new();
            serde_json::from_value::<Op>(json.clone()).unwrap().hash_params(&mut h);
            h.finalize()
        };
        let before = hash(&json);
        json["part"]["features"][1]["op"]["sketch"]["metadata"]["tip"]["note"] = json!("tooling note");
        assert_eq!(before, hash(&json));
        json["part"]["features"][1]["op"]["sketch"]["metadata"]["tip"][STAMP_ROLE] = json!("unknown");
        assert_ne!(before, hash(&json));
        let op = serde_json::from_value::<Op>(json).unwrap();
        assert!(matches!(op.eval(stamp(), &[&plate, &places]), Err(BrepError::Invalid(m)) if m.contains("`tip`") && m.contains(STAMP_ROLE)));
    }
}
