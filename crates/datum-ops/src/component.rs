//! A library part placed in a part (D260): the definition of the tool, its parameters and its
//! features, the body built once and copied to every placement. The definition is a copy of the
//! library element, written into the model: the model is built without the library (D256).

use std::collections::BTreeMap;
use std::collections::btree_map::Entry;

use datum_brep::{
    Arc, Body, BrepError, Cone, Curve, CurveSpec, Cylinder, FaceSpec, Plane, ProfileArc, ProfileLoop, Surface, TopoId, Torus, Vertex,
    build_with_curves,
};
use datum_math::Rigid;
use datum_math::expr::{ExprError, Num, parameter_values, valid_name};
use datum_math::{Vec2, Vec3};
use schemars::JsonSchema;
use serde::{Deserialize, Deserializer, Serialize};

use crate::sketch::{Placed, closed_loops, nearly_parallel, within_rounding};
use crate::{LineOf, Placement, Value, describe_arity, input_sketch, line_ends, num, sketch_point};

/// A part of the library (D256, D260), written into the model: its `kind` (`stamp` so far), the
/// `features` that build its body (the model's operations, by name), the `roles` of that body
/// (`cut`, `form`, optional `hinge`), the `anchor` where it is put in its local frame, an optional
/// `axis` and the parameter (`fit`) a line placement's length feeds, an optional `thickness`
/// range and the parameters of its size series.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PartDef {
    /// Its name in the library.
    pub name: String,
    /// `stamp` so far.
    pub kind: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    /// The size series: what its features are dimensioned by, in the order they are asked.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub parameters: Vec<PartParameter>,
    /// Typical values, by name: each names its parameters only.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub presets: BTreeMap<String, BTreeMap<String, Num>>,
    /// The features that build the tool's body, each after the ones it takes.
    pub features: Vec<PartFeature>,
    pub roles: Roles,
    /// Where it is put in its local frame, as a point `[x, y, z]`.
    pub anchor: [Num; 3],
    /// The direction the tool points in its local frame, `[x, y, z]`: a line placement turns it
    /// along the line, a point placement keeps it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub axis: Option<[Num; 3]>,
    /// The parameter a line placement's length feeds, as a sketch element stretches by its
    /// anchors (D98, D260); needs an `axis`.
    #[serde(default, deserialize_with = "fit_of", skip_serializing_if = "Option::is_none")]
    pub fit: Option<String>,
    /// The sheet thicknesses it stamps, `[min, max]` in mm.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub thickness: Option<[Num; 2]>,
}

/// A parameter of a part's size series: its `name` and its `value`, a number or an expression of
/// the other parameters.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PartParameter {
    pub name: String,
    pub value: Num,
}

/// `fit` read as text, with the library's own words when it is not.
fn fit_of<'de, D: Deserializer<'de>>(d: D) -> Result<Option<String>, D::Error> {
    Option::<String>::deserialize(d).map_err(|_| serde::de::Error::custom("a part's `fit`: a parameter's name"))
}

/// The roles of a part's body (D256): the features that cut the sheet, form it, and optionally
/// the hinge between them.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Roles {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cut: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub form: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub hinge: Option<String>,
}

/// A feature of a part's definition: its `name`, the `op` (the model's operations, by the same
/// JSON) and the earlier features it takes as `inputs`.
#[derive(Clone, Debug, PartialEq, Serialize, JsonSchema)]
pub struct PartFeature {
    pub name: String,
    pub op: crate::Op,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub inputs: Vec<String>,
}

impl<'de> Deserialize<'de> for PartFeature {
    /// Read by hand so that an operation that does not parse names the feature it is of, as the
    /// library's check did.
    fn deserialize<D: Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        #[derive(Deserialize)]
        #[serde(deny_unknown_fields)]
        struct Fields {
            name: String,
            op: serde_json::Value,
            #[serde(default)]
            inputs: Option<serde_json::Value>,
        }
        let f = Fields::deserialize(d)?;
        let fail = |e: serde_json::Error| serde::de::Error::custom(format!("feature `{}`: {e}", f.name));
        let op = serde_json::from_value(f.op).map_err(fail)?;
        let inputs = match f.inputs {
            None => Vec::new(),
            Some(v) => v
                .as_array()
                .and_then(|a| a.iter().map(serde_json::Value::as_str).collect::<Option<Vec<_>>>())
                .ok_or_else(|| serde::de::Error::custom(format!("feature `{}`: `inputs` are feature names", f.name)))?
                .into_iter()
                .map(str::to_string)
                .collect(),
        };
        Ok(Self { name: f.name, op, inputs })
    }
}

impl PartDef {
    /// What a part of the library must be, checked where it is written: a `stamp`, its features
    /// each after their inputs, roles naming its features, an `axis` that is a direction, a `fit`
    /// naming a parameter, a `thickness` range, presets naming parameters only. The body itself is
    /// built and checked when the part is used.
    pub fn validate(&self) -> Result<(), String> {
        if self.kind != "stamp" {
            return Err("a part's `kind`: `stamp`".into());
        }
        for (i, p) in self.parameters.iter().enumerate() {
            if !valid_name(&p.name) {
                return Err(format!("`{}` is not a valid parameter name: a letter or `_`, then letters, digits or `_`", p.name));
            }
            if self.parameters[..i].iter().any(|q| q.name == p.name) {
                return Err(format!("parameter `{}` given twice", p.name));
            }
        }
        let names: Vec<&str> = self.parameters.iter().map(|p| p.name.as_str()).collect();
        for (preset, values) in &self.presets {
            if let Some(n) = values.keys().find(|n| !names.contains(&n.as_str())) {
                return Err(format!("preset `{preset}`: no parameter `{n}`"));
            }
        }
        if self.anchor.iter().any(|n| n.value().is_none()) {
            return Err("a part's `anchor`: a point [x, y, z]".into());
        }
        if let Some(a) = &self.axis {
            if a.iter().any(|n| n.value().is_none()) {
                return Err("a part's `axis`: a direction [x, y, z]".into());
            }
            if a.iter().all(|n| n.value() == Some(0.0)) {
                return Err("a part's `axis`: not the zero vector".into());
            }
        }
        if let Some(f) = &self.fit {
            if f.trim().is_empty() {
                return Err("a part's `fit`: a parameter's name".into());
            }
            if !names.contains(&f.as_str()) {
                return Err(format!("a part's `fit`: no parameter `{f}`"));
            }
            if self.axis.is_none() {
                return Err("a part's `fit` needs an `axis`: the line's length feeds it".into());
            }
        }
        if let Some(t) = &self.thickness {
            let (Some(min), Some(max)) = (t[0].value(), t[1].value()) else {
                return Err("a part's `thickness`: [min, max]".into());
            };
            if !(min > 0.0 && max >= min) {
                return Err("a part's `thickness`: mm, 0 < min <= max".into());
            }
        }
        if self.features.is_empty() {
            return Err("a part's `features`: a non-empty list".into());
        }
        for (i, f) in self.features.iter().enumerate() {
            if f.name.trim().is_empty() {
                return Err("a feature's `name`: text".into());
            }
            if self.features[..i].iter().any(|g| g.name == f.name) {
                return Err(format!("a feature named `{}` already exists", f.name));
            }
            for input in &f.inputs {
                if !self.features[..i].iter().any(|g| g.name == *input) {
                    return Err(format!("feature `{}`: input `{input}` is not an earlier feature", f.name));
                }
            }
            if !f.op.arity().contains(&f.inputs.len()) {
                return Err(format!("`{}` takes {} input(s), got {}", f.name, describe_arity(f.op.arity()), f.inputs.len()));
            }
        }
        for (role, name) in [("cut", &self.roles.cut), ("form", &self.roles.form), ("hinge", &self.roles.hinge)] {
            if let Some(n) = name
                && !self.features.iter().any(|f| f.name == *n)
            {
                return Err(format!("a part's role `{role}`: no feature `{n}`"));
            }
        }
        if self.roles.cut.is_none() && self.roles.form.is_none() {
            return Err("a part's `roles`: `cut` or `form`".into());
        }
        Ok(())
    }

    /// Its `anchor`, in its local frame.
    fn anchor(&self) -> Result<Vec3, BrepError> {
        crate::vec3(&self.anchor)
    }

    /// Its `axis` as a unit direction; none if it has no axis.
    fn axis(&self) -> Result<Option<Vec3>, BrepError> {
        let Some(a) = &self.axis else { return Ok(None) };
        let a = crate::vec3(a)?;
        let length = a.dot(a).sqrt();
        if length == 0.0 {
            return Err(BrepError::Degenerate(format!("component: part `{}` has an `axis` of no length", self.name)));
        }
        Ok(Some(a * (1.0 / length)))
    }
}

/// The body of a placed tool and its roles (D256, D260): the body of the definition's last
/// feature, the bodies of the features the roles name, for the tasks that cut and form the
/// sheet, and the profile the `cut` role cuts with.
pub struct Tool {
    pub body: Body,
    pub cut: Option<Body>,
    pub form: Option<Body>,
    pub hinge: Option<Body>,
    /// The solved sketch of the `cut` role's extrusion, in the tool's own frame: its plane is the
    /// anchor plane (z = 0), which a placement lays on a wall of the sheet.
    pub cut_sketch: Option<Placed>,
    /// The solved sketch of the `form` role's extrusion: the die's section, across the louver.
    pub form_sketch: Option<Placed>,
}

/// The tool of `def`, its features built in order, `params` giving its parameters other values
/// (none: its own). Element IDs derive from the occurrence `feature` and the feature's place in
/// the definition (§4.2): every placement of the same definition and parameters shares them.
pub fn tool(def: &PartDef, feature: TopoId, params: &BTreeMap<String, Num>) -> Result<Tool, BrepError> {
    let values = parameters(def, params)?;
    let evaluated = parameter_values(&values);
    let lookup = |name: &str| evaluated.get(name).cloned().unwrap_or_else(|| Err(ExprError::Unknown(name.to_string())));
    let mut made: Vec<Value> = Vec::with_capacity(def.features.len());
    for (i, f) in def.features.iter().enumerate() {
        let id = TopoId::derive("component.part").id(feature).index(i as u64).finish();
        let op = f.op.resolve(&lookup).map_err(|e| BrepError::Invalid(format!("component: feature `{}`: {e}", f.name)))?;
        let mut inputs = Vec::with_capacity(f.inputs.len());
        for name in &f.inputs {
            let k = def.features[..i]
                .iter()
                .position(|g| g.name == *name)
                .ok_or_else(|| BrepError::Invalid(format!("component: feature `{}`: input `{name}` is not an earlier feature", f.name)))?;
            inputs.push(&made[k]);
        }
        let names: Vec<&str> = f.inputs.iter().map(String::as_str).collect();
        let value = op.eval_named(id, &inputs, &names).map_err(|e| BrepError::Invalid(format!("component: feature `{}`: {e}", f.name)))?;
        made.push(value);
    }
    let body_of = |name: &str| {
        let k = def
            .features
            .iter()
            .position(|f| f.name == name)
            .ok_or_else(|| BrepError::Invalid(format!("component: no feature `{name}` in part `{}`", def.name)))?;
        made[k].body().cloned().ok_or_else(|| BrepError::Invalid(format!("component: feature `{name}` of part `{}` is no body", def.name)))
    };
    let role = |name: &Option<String>| name.as_deref().map(body_of).transpose();
    let last = made
        .last()
        .and_then(Value::body)
        .cloned()
        .ok_or_else(|| BrepError::Invalid(format!("component: the last feature of part `{}` is no body", def.name)))?;
    let cut_sketch = def.roles.cut.as_deref().map(|name| profile(def, name, &made, "cut")).transpose()?;
    let form_sketch = def.roles.form.as_deref().map(|name| profile(def, name, &made, "form")).transpose()?;
    Ok(Tool { body: last, cut: role(&def.roles.cut)?, form: role(&def.roles.form)?, hinge: role(&def.roles.hinge)?, cut_sketch, form_sketch })
}

/// The sketch the role `role` works with: the first input of its extrusion, a sketch feature of
/// the definition, as it was solved (in the tool's own frame).
fn profile(def: &PartDef, role: &str, made: &[Value], which: &str) -> Result<Placed, BrepError> {
    let not = || BrepError::Invalid(format!("component: the {which} role `{role}` of part `{}` is not an extrusion of a sketch", def.name));
    let k = def.features.iter().position(|f| f.name == role).ok_or_else(not)?;
    let f = &def.features[k];
    if !matches!(f.op, crate::Op::Extrude { .. }) {
        return Err(not());
    }
    let first = f.inputs.first().ok_or_else(not)?;
    let j = def.features[..k].iter().position(|g| g.name == *first).ok_or_else(not)?;
    made[j].sketch().cloned().ok_or_else(not)
}

/// The definition's parameter values, `params` over them; a name the part has not is refused.
fn parameters(def: &PartDef, params: &BTreeMap<String, Num>) -> Result<BTreeMap<String, Num>, BrepError> {
    if let Some(x) = params.keys().find(|x| !def.parameters.iter().any(|p| p.name == **x)) {
        return Err(BrepError::Invalid(format!("component: no parameter `{x}` in part `{}`", def.name)));
    }
    let mut out: BTreeMap<String, Num> = def.parameters.iter().map(|p| (p.name.clone(), p.value.clone())).collect();
    out.extend(params.iter().map(|(k, v)| (k.clone(), v.clone())));
    Ok(out)
}

/// The sheet `values[0]` cut where the tool of `def` at each placement crosses its wall (D260):
/// the `cut` role's profile, laid on the wall by the placement, punches one wall through, as a
/// press punch does (D196), and the cut is recorded in the blank and its DXF. Refused: a part
/// that forms or hinges the sheet (the next step), a sheet with ribs (its blank cannot be laid
/// from the ribbed body: the cut and the ribs laid again come later), a `cut` role that is no
/// extrusion of a sketch, a placement whose profile does not lie in the plane of the wall, a
/// placement on a face the sheet has not.
pub(crate) fn cut(
    feature: TopoId,
    def: &PartDef,
    params: &BTreeMap<String, Num>,
    placements: &[Placement],
    values: &[&Value],
) -> Result<Value, BrepError> {
    let later = |m: String| BrepError::Unsupported(format!("component: {m}"));
    if def.roles.form.is_some() {
        return Err(later(format!("part `{}` forms the sheet: forming comes next", def.name)));
    }
    if def.roles.hinge.is_some() {
        return Err(later(format!("part `{}` has a `hinge`: forming comes next", def.name)));
    }
    if placements.is_empty() {
        return Err(BrepError::Invalid(format!("component: part `{}` is placed nowhere: `placements` is empty", def.name)));
    }
    let mut body = match values.first() {
        Some(Value::Formed(_)) => return Err(later("cutting sheet metal with ribs comes later".into())),
        Some(v) => v.body().cloned().ok_or_else(|| BrepError::Sketch("a component takes the sheet, then a sketch the placements name".into()))?,
        None => return Err(BrepError::Invalid("a component takes the sheet, then a sketch the placements name".into())),
    };
    let mut tools: BTreeMap<Option<u64>, Tool> = BTreeMap::new();
    for (p, placement) in placements.iter().enumerate() {
        let (rigid, fit) = place(def, placement, values)?;
        let made = match tools.entry(fit.map(f64::to_bits)) {
            Entry::Occupied(e) => e.into_mut(),
            Entry::Vacant(e) => {
                let mut overrides = params.clone();
                if let (Some(name), Some(length)) = (def.fit.as_ref(), fit) {
                    overrides.insert(name.clone(), Num::Value(length));
                }
                e.insert(tool(def, feature, &overrides)?)
            }
        };
        let profile = made
            .cut_sketch
            .as_ref()
            .ok_or_else(|| BrepError::Invalid(format!("component: part `{}` has no `cut` role: nothing cuts the sheet", def.name)))?;
        let placed = laid(def, profile, rigid, placement, values, &body)?;
        // Each placement cuts by an ID of its own: its hole is a new element of the sheet (§4.2).
        let id = TopoId::derive("component.cut").id(feature).index(p as u64).finish();
        body = crate::bent::punch(id, &body, &placed, false)?;
    }
    crate::validated(&body)?;
    Ok(Value::Body(body))
}

/// The tool's cut profile at `placement`: its frame moved by `rigid` onto the wall the placement's
/// sketch lies on, that sketch's face and K-factor. Refused unless the profile lies in the wall's
/// plane.
fn laid(def: &PartDef, profile: &Placed, rigid: Rigid, placement: &Placement, values: &[&Value], target: &Body) -> Result<Placed, BrepError> {
    let mut placed = profile.clone();
    let [o, x, y, z] = profile.frame;
    placed.frame = [rigid.apply(o), rigid.apply_vector(x), rigid.apply_vector(y), rigid.apply_vector(z)];
    let k = match placement {
        Placement::Line(l) => l.input,
        Placement::Point(p) => p.input,
    };
    let on = input_sketch(values, k, "a component's placement")?;
    placed.face = on.face;
    placed.k_factor = on.k_factor;
    on_wall(def, target, &placed)?;
    Ok(placed)
}

/// Whether the cut profile `placed` lies in the plane of the face of `target` its placement names:
/// its origin on that plane and its normal along the face's, out of the sheet, to the bits a turn
/// leaves (D229). A part whose profile is not laid on the wall cannot cut it.
fn on_wall(def: &PartDef, target: &Body, placed: &Placed) -> Result<(), BrepError> {
    let wrong = || {
        BrepError::Invalid(format!(
            "component: the cut of part `{}` is not in the plane of its anchor, facing out of the sheet: its profile is off the face the placement names",
            def.name
        ))
    };
    let id = placed.face.ok_or_else(|| {
        BrepError::Invalid(format!("component: a cut takes a sketch placed on a face of the sheet: part `{}` is placed on none", def.name))
    })?;
    let f = target.faces().binary_search_by_key(&id, |g| g.id).map_err(|_| BrepError::LostReference(format!("face {id:?} is not in the body")))?;
    let Surface::Plane(plane) = target.faces()[f].surface else {
        return Err(BrepError::Unsupported(format!("component: a cut of part `{}` on a face that is not plane comes later", def.name)));
    };
    let n = unit(plane.normal);
    let [o, _, _, z] = placed.frame;
    let size = o.to_array().iter().chain(plane.origin.to_array().iter()).fold(1.0_f64, |m, x| m.max(x.abs()));
    if !within_rounding((o - plane.origin).dot(n), size) || !nearly_parallel(z, n) || z.dot(n) <= 0.0 {
        return Err(wrong());
    }
    Ok(())
}

/// Where placement `placement` puts the tool: its motion, and the length a line feeds to `fit`.
fn place(def: &PartDef, placement: &Placement, values: &[&Value]) -> Result<(Rigid, Option<f64>), BrepError> {
    let anchor = def.anchor()?;
    match placement {
        Placement::Line(l) => {
            let (start, along, length) = line(def, l, values)?;
            let axis = def.axis()?.ok_or_else(|| {
                BrepError::Sketch(format!("component: part `{}` has no `axis`: a line placement turns the tool along the line", def.name))
            })?;
            let turn = turning(axis, along);
            Ok((turn.then(Rigid::translation(start - turn.apply(anchor))), def.fit.as_ref().map(|_| length)))
        }
        Placement::Point(p) => Ok((Rigid::translation(at(def, p, values)? - anchor), None)),
    }
}

/// The start of line `l`, the unit direction it runs in and its length.
fn line(def: &PartDef, l: &LineOf, values: &[&Value]) -> Result<(Vec3, Vec3, f64), BrepError> {
    let s = input_sketch(values, l.input, "a component's line placement")?;
    let (a, b) = line_ends(s, &l.line, l.input)?;
    let d = b - a;
    let length = d.dot(d).sqrt();
    if length == 0.0 {
        return Err(BrepError::Degenerate(format!("component: part `{}` is placed along line `{}`, which has no length", def.name, l.line)));
    }
    Ok((a, d * (1.0 / length), length))
}

/// Where point placement `p` puts the tool: a point of its sketch by name, or sketch coordinates.
fn at(def: &PartDef, p: &crate::At, values: &[&Value]) -> Result<Vec3, BrepError> {
    let s = input_sketch(values, p.input, "a component's point placement")?;
    if s.developed.is_some() {
        return Err(BrepError::Unsupported(format!("component: part `{}` is placed on a sketch of a cylinder face", def.name)));
    }
    match (&p.point, &p.at) {
        (Some(name), None) => sketch_point(s, name, p.input),
        (None, Some([x, y])) => {
            let [o, ex, ey, _] = s.frame;
            Ok(o + ex * num(x)? + ey * num(y)?)
        }
        _ => Err(BrepError::Sketch("a point placement is a `point` of the sketch or `at` [x, y], exactly one".into())),
    }
}

/// The motion turning direction `from` to `to` (both unit), the shortest.
fn turning(from: Vec3, to: Vec3) -> Rigid {
    let cross = from.cross(to);
    let dot = from.dot(to);
    if cross.dot(cross) == 0.0 {
        // Along it, or against it: a half turn about any direction square to it.
        return if dot > 0.0 { Rigid::IDENTITY } else { Rigid::half_turn(perpendicular(from)) };
    }
    Rigid::rotation(cross, libm::atan2(cross.dot(cross).sqrt(), dot))
}

/// A direction square to `v` (not zero): across its least component.
fn perpendicular(v: Vec3) -> Vec3 {
    let a = v.to_array();
    let k = (1..3).fold(0, |m, i| if a[i].abs() < a[m].abs() { i } else { m });
    v.cross([Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0)][k])
}

/// `v` as a unit direction.
fn unit(v: Vec3) -> Vec3 {
    v * (1.0 / v.dot(v).sqrt())
}

// ---------------------------------------------------------------------------------------------
// Forming: a louver along a line placement (D261).

/// A piece of the die's section: a line, or an arc about its centre, counterclockwise in the
/// section's plane.
#[derive(Clone, Copy, Debug)]
enum Piece {
    Line,
    Arc { center: Vec2, ccw: bool },
}

/// The die's section across a louver (D261): the tool's working curve, arcs of the sheet rule's
/// radius where it meets the sheet's faces, and the same a thickness `t` off into the material.
/// A closed loop: the upper curve from the left wall tangency to the right, the right end across
/// the thickness, the lower curve back, the left end.
struct Die {
    /// The loop's points: the upper curve `0..=n`, then the lower curve from the right end back.
    points: Vec<Vec2>,
    /// The loop's pieces, from `points[k]` to `points[k + 1]`.
    pieces: Vec<Piece>,
    /// The upper curve's pieces.
    n: usize,
    t: f64,
    /// Where the transitions meet the sheet's faces: half the louver's width.
    w: f64,
}

/// The die's section from the `form` role's sketch `s`, in the section's plane: x across the
/// louver, y up from the sheet's face (y = 0), for a sheet `t` thick and a bend radius `r`. The
/// section is one open chain of lines and arcs, symmetric about the louver's axis, running across
/// it once and dipping below the face: the die's working curve. The arcs of radius `r` join it to
/// the face, the tongue's thickness `t` is the offset into the material.
fn die_of(feature: TopoId, s: &Placed, t: f64, r: f64) -> Result<Die, BrepError> {
    let later = |m: &str| BrepError::Unsupported(format!("component: {m}"));
    let short = |m: &str| BrepError::Degenerate(format!("component: {m}"));
    if s.developed.is_some() {
        return Err(later("forming on a rolled sheet comes later: place the die on a flat wall"));
    }
    if !s.circles.is_empty() || !s.ellipses.is_empty() {
        return Err(later("the die's section is an outline of lines and arcs: no circles or ellipses"));
    }
    let loops = closed_loops(feature, s, &|_| true);
    let ring = match loops.as_slice() {
        [one] => &one.ring,
        [] => return Err(later("the die's section is one closed outline of lines and arcs: the tool's body is its extrusion")),
        _ => return Err(later("the die's section is one closed outline of lines and arcs")),
    };
    // Its lower side, between the widest points: the die's working curve.
    let m = ring.points.len();
    // Its widest points, the lower of them where the outline is square there.
    let by_x = |a: &usize, b: &usize| ring.points[*a].x.total_cmp(&ring.points[*b].x);
    let lo = (0..m).min_by(|a, b| by_x(a, b).then(ring.points[*a].y.total_cmp(&ring.points[*b].y))).unwrap_or(0);
    let hi = (0..m).max_by(|a, b| by_x(a, b).then(ring.points[*b].y.total_cmp(&ring.points[*a].y))).unwrap_or(0);
    // Both ways round, each run left to right with its arcs turned to the run: the working curve
    // is the one below the chord between the ends, whose loop with the chord is then
    // counterclockwise. The ring runs whichever way it was drawn, so the ways' plain areas do not
    // tell them apart: both may come out negative.
    let edge = |from: usize, to: usize| -> Option<ProfileArc> {
        let k = if to == (from + 1) % m { from } else { to };
        ring.arc(k).map(|a| if to == (from + 1) % m { a } else { ProfileArc { center: a.center, ccw: !a.ccw } })
    };
    let way = |from: usize, to: usize| -> Vec<usize> { (0..=(to + m - from) % m).map(|j| (from + j) % m).collect() };
    let below = |c: &[usize]| -> f64 {
        let points: Vec<Vec2> = c.iter().map(|&k| ring.points[k]).collect();
        let arcs: Vec<Option<ProfileArc>> = c.windows(2).map(|w| edge(w[0], w[1])).collect();
        ProfileLoop { points, vertices: Vec::new(), sides: Vec::new(), arcs }.area()
    };
    // A way round run the other way: its points in reverse, its arcs turned.
    let run = |c: &[usize]| -> Vec<usize> {
        let mut r = c.to_vec();
        r.reverse();
        r
    };
    let chain = {
        let (a, b) = (way(lo, hi), run(&way(hi, lo)));
        if below(&a) > 0.0 {
            a
        } else if below(&b) > 0.0 {
            b
        } else {
            return Err(short("the die's section has no side below the chord between its widest points"));
        }
    };
    let mut pieces: Vec<Piece> = Vec::with_capacity(chain.len() - 1);
    for w in chain.windows(2) {
        pieces.push(match edge(w[0], w[1]) {
            None => Piece::Line,
            Some(a) => Piece::Arc { center: a.center, ccw: a.ccw },
        });
    }
    let verts: Vec<Vec2> = chain.iter().map(|&k| ring.points[k]).collect();
    let size = verts.iter().fold(1.0_f64, |m, p| m.max(p.x.abs()).max(p.y.abs()));
    let same = |a: Vec2, b: Vec2| within_rounding(a.x - b.x, size) && within_rounding(a.y - b.y, size);
    // Across the louver once, left to right, below the sheet's face, and pressing into it.
    if verts.windows(2).any(|w| w[0].x >= w[1].x) {
        return Err(later("the die's section runs across the louver from left to right, once"));
    }
    if verts.iter().any(|p| p.y > 0.0) {
        return Err(later("the die's section stays at or below the sheet's face: a section rising out of it comes later"));
    }
    for (k, piece) in pieces.iter().enumerate() {
        let Piece::Arc { center, ccw } = *piece else { continue };
        let arc = ProfileArc { center, ccw };
        let top = Vec2::new(center.x, center.y + libm::hypot(verts[k].x - center.x, verts[k].y - center.y));
        let (at, total) = (arc.turn(verts[k], top), arc.turn(verts[k], verts[k + 1]));
        if at > 0.0 && at < total {
            return Err(later("the die's section stays at or below the sheet's face: a section rising out of it comes later"));
        }
    }
    // It presses into the sheet: a point of it below the face.
    let mut dips = verts.iter().any(|p| p.y < 0.0);
    for (k, piece) in pieces.iter().enumerate() {
        let Piece::Arc { center, ccw } = *piece else { continue };
        let rho = libm::hypot(verts[k].x - center.x, verts[k].y - center.y);
        let arc = ProfileArc { center, ccw };
        let bottom = Vec2::new(center.x, center.y - rho);
        let (at, total) = (arc.turn(verts[k], bottom), arc.turn(verts[k], verts[k + 1]));
        dips |= at > 0.0 && at < total && bottom.y < 0.0;
    }
    if !dips {
        return Err(short("the die's section does not press into the sheet: it stays on its face"));
    }
    // Symmetric about the louver's axis: the end dome is the section turned a half about it.
    let m = verts.len() - 1;
    for k in 0..=m {
        if !same(verts[k], mirror(verts[m - k])) {
            return Err(BrepError::Invalid(format!(
                "component: the die's section is not symmetric about the louver's axis (point {:?} has no mirror): the end dome needs it",
                verts[k]
            )));
        }
    }
    for k in 0..m {
        let ok = match (pieces[k], pieces[m - 1 - k]) {
            (Piece::Line, Piece::Line) => true,
            (Piece::Arc { center: a, ccw: ac }, Piece::Arc { center: b, ccw: bc }) => same(a, mirror(b)) && ac == bc,
            _ => false,
        };
        if !ok {
            return Err(BrepError::Invalid(format!(
                "component: the die's section is not symmetric about the louver's axis (piece {k} has no mirror): the end dome needs it"
            )));
        }
    }
    // The end dome turns each piece under a half turn: a piece of a half turn or more has ends
    // that do not fix the piece of its circle (both halves serve), and its dome's way round is
    // not the section's.
    for (k, piece) in pieces.iter().enumerate() {
        let Piece::Arc { center, ccw } = *piece else { continue };
        let arc = ProfileArc { center, ccw };
        if arc.turn(verts[k], verts[k + 1]) >= std::f64::consts::PI {
            return Err(short("the die's section has an arc of a half turn or more: the end dome cannot run it"));
        }
    }
    // The left half, up to the axis, the last piece cut there.
    let axis = match verts.iter().position(|p| p.x == 0.0) {
        Some(z) => (z, Vec2::new(0.0, 0.0)),
        None => {
            let k = verts.windows(2).position(|w| w[0].x < 0.0 && w[1].x > 0.0).ok_or_else(|| short("the die's section does not cross its axis"))?;
            (k + 1, at_x(&pieces[k], verts[k], verts[k + 1], 0.0)?)
        }
    };
    let mut left_v: Vec<Vec2> = verts[..axis.0].to_vec();
    left_v.push(axis.1);
    let mut left_p: Vec<Piece> = pieces[..axis.0 - 1].to_vec();
    if axis.0 > 0 {
        let k = axis.0 - 1;
        left_p.push(pieces[k]);
    }
    // The transition from the sheet's face to the die's first piece, and the piece from there.
    let (center, touch) = fillet(&left_p[0], left_v[0], left_v[1], r)?;
    let w = -center.x;
    if w <= 0.0 {
        return Err(short("the die's transition to the sheet does not stand left of its axis"));
    }
    left_v[0] = touch;
    let upper: Vec<Vec2> = std::iter::once(Vec2::new(center.x, 0.0)).chain(left_v.iter().copied()).collect();
    let mut upper_p: Vec<Piece> = std::iter::once(Piece::Arc { center, ccw: false }).chain(left_p.iter().copied()).collect();
    let mut upper_v = upper;
    let count = upper_p.len();
    // The right half: the mirror of the left, exactly.
    for k in (0..count).rev() {
        upper_v.push(mirror(upper_v[k]));
        upper_p.push(mirror_piece(&upper_p[k]));
    }
    let n = upper_p.len();
    // The lower curve: the upper's offset by `t` into the material, back.
    let mut low_v: Vec<Vec2> = Vec::with_capacity(n + 1);
    let mut low_p: Vec<Piece> = Vec::with_capacity(n);
    for k in 0..n {
        let (a, b, piece) = offset(&upper_p[k], upper_v[k], upper_v[k + 1], t)?;
        if k == 0 {
            low_v.push(a);
        }
        low_v.push(b);
        low_p.push(piece);
    }
    let mut points = upper_v;
    points.extend(low_v.iter().rev().copied());
    let mut pieces = upper_p;
    pieces.push(Piece::Line);
    for k in (0..n).rev() {
        pieces.push(reverse_piece(&low_p[k]));
    }
    pieces.push(Piece::Line);
    Ok(Die { points, pieces, n, t, w })
}

/// The arc of `radius` from `from` to `to` about `center`, its axis the short way's: these arcs
/// run under a half turn, whose ends and way round fix the piece of the circle they are (a half's
/// do not, both halves serve). None when the ends are one point or stand a half turn apart.
fn arc_between(center: Vec3, from: Vec3, to: Vec3, radius: f64) -> Option<Arc> {
    let cross = (from - center).cross(to - center);
    (cross.dot(cross) > 0.0).then(|| Arc { center, axis: unit(cross), radius })
}

/// `p` mirrored across the louver's axis.
fn mirror(p: Vec2) -> Vec2 {
    Vec2::new(-p.x, p.y)
}

fn mirror_piece(p: &Piece) -> Piece {
    match *p {
        Piece::Line => Piece::Line,
        Piece::Arc { center, ccw } => Piece::Arc { center: mirror(center), ccw },
    }
}

fn reverse_piece(p: &Piece) -> Piece {
    match *p {
        Piece::Line => Piece::Line,
        Piece::Arc { center, ccw } => Piece::Arc { center, ccw: !ccw },
    }
}

fn unit2(d: Vec2) -> Vec2 {
    let l = libm::hypot(d.x, d.y);
    Vec2::new(d.x / l, d.y / l)
}

/// The unit normal to the right of `d`.
fn right(d: Vec2) -> Vec2 {
    let t = unit2(d);
    Vec2::new(t.y, -t.x)
}

/// The unit way `piece` runs at its point `p`.
fn way(piece: &Piece, p: Vec2) -> Vec2 {
    match *piece {
        Piece::Line => unreachable!("a line's way comes from its ends"),
        Piece::Arc { center, ccw } => {
            let r = Vec2::new(p.x - center.x, p.y - center.y);
            let l = libm::hypot(r.x, r.y);
            if ccw { Vec2::new(-r.y / l, r.x / l) } else { Vec2::new(r.y / l, -r.x / l) }
        }
    }
}

/// Where `piece` (from `a` to `b`) meets the line `x = s`.
fn at_x(piece: &Piece, a: Vec2, b: Vec2, s: f64) -> Result<Vec2, BrepError> {
    match *piece {
        Piece::Line => {
            let t = (s - a.x) / (b.x - a.x);
            Ok(Vec2::new(s, a.y + t * (b.y - a.y)))
        }
        Piece::Arc { center, ccw } => {
            let rho = libm::hypot(a.x - center.x, a.y - center.y);
            let dx = s - center.x;
            if dx.abs() > rho {
                return Err(BrepError::Degenerate("component: the die's section does not reach its axis".into()));
            }
            let dy = libm::sqrt(rho * rho - dx * dx);
            let arc = ProfileArc { center, ccw };
            let total = arc.turn(a, b);
            [dy, -dy]
                .into_iter()
                .map(|d| Vec2::new(s, center.y + d))
                .find(|&p| {
                    let t = arc.turn(a, p);
                    t > 0.0 && t < total
                })
                .ok_or_else(|| BrepError::Degenerate("component: the die's section does not reach its axis".into()))
        }
    }
}

/// The arc of radius `r` that meets the sheet's face (y = 0) and the section's first piece (from
/// `a` to `b`) tangentially, leaving the face along +x: its centre and where it touches the piece.
fn fillet(piece: &Piece, a: Vec2, b: Vec2, r: f64) -> Result<(Vec2, Vec2), BrepError> {
    let short = || {
        BrepError::Degenerate(format!(
            "component: a die whose section meets the sheet at ({:.3}, {:.3}) is too shallow, or too flat there, for the {r} mm transition: no arc of that radius is tangent to both",
            a.x, a.y
        ))
    };
    let mut found: Vec<(Vec2, Vec2)> = Vec::new();
    match *piece {
        Piece::Line => {
            let t = unit2(Vec2::new(b.x - a.x, b.y - a.y));
            if t.y == 0.0 {
                return Err(short());
            }
            let n = Vec2::new(t.y, -t.x);
            let s = (-r - a.y - r * n.y) / t.y;
            let len = libm::hypot(b.x - a.x, b.y - a.y);
            if s > 0.0 && s < len {
                found.push((Vec2::new(a.x + r * n.x + s * t.x, -r), Vec2::new(a.x + s * t.x, a.y + s * t.y)));
            }
        }
        Piece::Arc { center, ccw } => {
            let rho = libm::hypot(a.x - center.x, a.y - center.y);
            // The material is outside a counterclockwise arc's circle, inside a clockwise one's:
            // the transition's circle touches it from without, or from within.
            let reach = if ccw { r + rho } else { (r - rho).abs() };
            let dy = -r - center.y;
            if reach * reach >= dy * dy {
                let dx = libm::sqrt(reach * reach - dy * dy);
                for c in [Vec2::new(center.x - dx, -r), Vec2::new(center.x + dx, -r)] {
                    let d = Vec2::new(center.x - c.x, center.y - c.y);
                    let l = libm::hypot(d.x, d.y);
                    let t = if ccw { Vec2::new(c.x + r * d.x / l, c.y + r * d.y / l) } else { Vec2::new(c.x - r * d.x / l, c.y - r * d.y / l) };
                    found.push((c, t));
                }
            }
        }
    }
    // The arc leaves the face along +x, less than a half turn, and meets the piece the section's
    // way: the material stays below it.
    found
        .into_iter()
        .find(|&(c, t)| {
            let v1 = Vec2::new(0.0, r);
            let v2 = Vec2::new(t.x - c.x, t.y - c.y);
            let turn = libm::atan2(v1.x * v2.y - v1.y * v2.x, v1.x * v2.x + v1.y * v2.y);
            let arc = Piece::Arc { center: c, ccw: false };
            let piece_way = match *piece {
                Piece::Line => unit2(Vec2::new(b.x - a.x, b.y - a.y)),
                Piece::Arc { .. } => way(piece, t),
            };
            turn < 0.0 && turn > -std::f64::consts::PI && {
                let w = way(&arc, t);
                w.x * piece_way.x + w.y * piece_way.y > 0.0
            }
        })
        .ok_or_else(short)
}

/// The piece `piece` (from `a` to `b`) `t` to the material's side, with its ends there.
fn offset(piece: &Piece, a: Vec2, b: Vec2, t: f64) -> Result<(Vec2, Vec2, Piece), BrepError> {
    Ok(match *piece {
        Piece::Line => {
            let n = right(Vec2::new(b.x - a.x, b.y - a.y));
            (Vec2::new(a.x + t * n.x, a.y + t * n.y), Vec2::new(b.x + t * n.x, b.y + t * n.y), Piece::Line)
        }
        Piece::Arc { center, ccw } => {
            let rho = libm::hypot(a.x - center.x, a.y - center.y);
            let r2 = if ccw { rho + t } else { rho - t };
            if r2 <= 0.0 {
                return Err(BrepError::Degenerate(format!(
                    "component: an arc of the die's section of {rho} mm is not wider than the sheet is thick ({t} mm)"
                )));
            }
            let k = r2 / rho;
            (
                Vec2::new(center.x + (a.x - center.x) * k, center.y + (a.y - center.y) * k),
                Vec2::new(center.x + (b.x - center.x) * k, center.y + (b.y - center.y) * k),
                Piece::Arc { center, ccw },
            )
        }
    })
}

/// Where the die's section stands: the sketch's frame on the sheet's face, and how far along the
/// louver's axis it is.
#[derive(Clone, Copy)]
struct Stand {
    o: Vec3,
    x: Vec3,
    y: Vec3,
    z: Vec3,
    at: f64,
}

impl Stand {
    fn point(&self, p: Vec2) -> Vec3 {
        self.o + self.x * p.x + self.y * p.y + self.z * self.at
    }

    /// The section point `p` turned a right angle about the sheet's normal at the station, out of
    /// the middle: `x` goes along the louver, on the side away from the middle.
    fn quarter(&self, p: Vec2) -> Vec3 {
        self.o + self.y * p.y + self.z * (self.at - p.x)
    }

    fn along(&self, d: Vec2) -> Vec3 {
        self.x * d.x + self.y * d.y
    }
}

/// The faces a louver adds to the sheet: its tongue, the zone's contour on the wall's two faces,
/// and the zone's wall across the thickness (the blank keeps that one).
struct Louver {
    vertices: Vec<Vertex>,
    curves: Vec<CurveSpec>,
    /// The tongue's faces: its surfaces and its own rim.
    tongue: Vec<FaceSpec>,
    /// The zone's wall across the thickness, for the sheet: its faces and their arcs.
    hole: Vec<FaceSpec>,
    hole_curves: Vec<CurveSpec>,
    top: Vec<TopoId>,
    bottom: Vec<TopoId>,
}

/// The louver of `die` at placement `p`, its middle from `start` to `end` along the sheet.
fn louver(feature: TopoId, p: usize, die: &Die, start: Stand, end: Stand) -> Result<Louver, BrepError> {
    let n = die.n;
    let m = die.pieces.len();
    let vid = |station: u64, k: usize| TopoId::derive("component.form.v").id(feature).index(p as u64).index(station).index(k as u64).finish();
    let fid = |kind: &str, k: usize, station: u64| {
        TopoId::derive("component.form.f").id(feature).index(p as u64).tag(kind).index(k as u64).index(station).finish()
    };
    let stand = |station: u64| if station == 0 { start } else { end };
    let mut vertices = Vec::with_capacity(2 * (m + 1));
    let mut curves: BTreeMap<(TopoId, TopoId), Curve> = BTreeMap::new();
    for station in 0..2u64 {
        let at = stand(station);
        for (k, q) in die.points.iter().enumerate() {
            vertices.push(Vertex { id: vid(station, k), point: at.point(*q) });
        }
        for k in 0..m {
            if let Piece::Arc { center, ccw } = die.pieces[k] {
                let a = die.points[k];
                let rho = libm::hypot(a.x - center.x, a.y - center.y);
                let arc = Arc { center: at.point(center), axis: at.z * if ccw { 1.0 } else { -1.0 }, radius: rho };
                curves.insert((vid(station, k), vid(station, k + 1)), Curve::Arc(arc));
            }
        }
    }
    // The middle: each piece of the section swept from one end to the other. The ends across the
    // thickness are the zone's wall, not the tongue's.
    let mut tongue = Vec::with_capacity(2 * m);
    for k in 0..m {
        if k == n || k == m - 1 {
            continue;
        }
        let surface = swept(&die.pieces[k], die.points[k], die.points[k + 1], start)?;
        let ids = vec![vid(0, k), vid(1, k), vid(1, k + 1), vid(0, k + 1)];
        tongue.push(FaceSpec { id: fid("sweep", k, 0), surface, loops: vec![ids] });
    }
    // The end domes: the section turned a half about the sheet's normal at each end, out of the
    // middle. A half turn is two quarters, its parallels cut at a right angle: a quarter's ends
    // and axis fix the piece of its circle alone (a half's do not, both halves serve), so the
    // turn's way round stays what the section says. A piece and its mirror are one patch: its two
    // meridians.
    // A point of the section on the louver's axis (`x == 0`) does not move under the turn: its
    // quarter vertex is the section's own, and the patches by it lose the parallel that would be
    // of no radius (the loops' dedup and the pairs' `a == b` skip take it from there).
    let on_axis = |k: usize| die.points[k].x == 0.0;
    let qvid = |station: u64, k: usize| {
        if on_axis(k) {
            vid(station, k)
        } else {
            TopoId::derive("component.form.q").id(feature).index(p as u64).index(station).index(k as u64).finish()
        }
    };
    for station in 0..2u64 {
        let at = stand(station);
        for (k, q) in die.points.iter().enumerate() {
            if !on_axis(k) {
                vertices.push(Vertex { id: qvid(station, k), point: at.quarter(*q) });
            }
        }
    }
    let mirrored = |k: usize| if k <= n { n - k } else { 3 * n + 2 - k };
    for station in 0..2u64 {
        let at = stand(station);
        for k in 0..m {
            if k == n || k == m - 1 {
                continue;
            }
            // The piece whose turn goes away from the middle: the right half at the start, the
            // left half at the end.
            let mid = 0.5 * (die.points[k].x + die.points[k + 1].x);
            if !(if station == 0 { mid > 0.0 } else { mid < 0.0 }) {
                continue;
            }
            let j = mirrored(k + 1);
            let surface = turned(&die.pieces[k], die.points[k], die.points[k + 1], at)?;
            // The first quarter, from the piece to the right angle; the second, on to its mirror.
            let mut first = vec![vid(station, k), vid(station, k + 1), qvid(station, k + 1), qvid(station, k)];
            let mut second = vec![qvid(station, k), qvid(station, k + 1), vid(station, j), vid(station, j + 1)];
            if station == 1 {
                first.reverse();
                second.reverse();
            }
            for ids in [&mut first, &mut second] {
                ids.dedup();
                if ids.len() > 1 && ids.first() == ids.last() {
                    ids.pop();
                }
            }
            // Each parallel runs from the piece to its mirror in quarters, cut where it passes
            // the right angle, on the side away from the middle: that end is the same for a piece
            // and its mirror, so both patches write the same quarter, not one the near side's.
            let qpoint = |k: usize| if on_axis(k) { at.point(die.points[k]) } else { at.quarter(die.points[k]) };
            let away = |a: usize, b: usize| if (station == 0) == (die.points[a].x > 0.0) { a } else { b };
            for (a, b) in [(k, j + 1), (k + 1, j)] {
                if a == b {
                    continue;
                }
                let far = away(a, b);
                let near = if far == a { b } else { a };
                let radius = die.points[far].x.abs();
                let center = at.point(Vec2::new(0.0, die.points[far].y));
                for (from, to, fp, tp) in [
                    (vid(station, far), qvid(station, far), at.point(die.points[far]), qpoint(far)),
                    (qvid(station, far), vid(station, near), qpoint(far), at.point(die.points[near])),
                ] {
                    if from == to {
                        continue;
                    }
                    let Some(arc) = arc_between(center, fp, tp, radius) else { continue };
                    curves.insert((from.min(to), from.max(to)), Curve::Arc(arc));
                }
            }
            // The meridian at the right angle: the piece turned there.
            if let Piece::Arc { center, ccw } = die.pieces[k] {
                let a = die.points[k];
                let rho = libm::hypot(a.x - center.x, a.y - center.y);
                let arc = Arc { center: at.quarter(center), axis: at.x * if ccw { 1.0 } else { -1.0 }, radius: rho };
                let (from, to) = (qvid(station, k), qvid(station, k + 1));
                curves.insert((from.min(to), from.max(to)), Curve::Arc(arc));
            }
            tongue.push(FaceSpec { id: fid("dome", k, station), surface, loops: vec![first] });
            tongue.push(FaceSpec { id: fid("dome.q", k, station), surface, loops: vec![second] });
        }
    }
    // The zone's wall across the thickness, for the sheet: the hole the tongue fills. Its own
    // vertices, so its loops turn as the sheet's faces need, apart from the tongue's.
    let wv = |station: u64, k: usize| TopoId::derive("component.form.w").id(feature).index(p as u64).index(station).index(k as u64).finish();
    let corners: [(u64, usize, Vec2); 8] = [
        (0, 0, die.points[0]),
        (0, n, die.points[n]),
        (0, n + 1, die.points[n + 1]),
        (0, 2 * n + 1, die.points[2 * n + 1]),
        (1, 0, die.points[0]),
        (1, n, die.points[n]),
        (1, n + 1, die.points[n + 1]),
        (1, 2 * n + 1, die.points[2 * n + 1]),
    ];
    for &(station, k, q) in &corners {
        vertices.push(Vertex { id: wv(station, k), point: stand(station).point(q) });
    }
    let mut wall_curves: Vec<CurveSpec> = Vec::with_capacity(4);
    for station in 0..2u64 {
        let at = stand(station);
        let side = at.y * if station == 0 { -1.0 } else { 1.0 };
        for (a, b, height) in [(0, n, 0.0), (2 * n + 1, n + 1, -die.t)] {
            let arc = Arc { center: at.point(Vec2::new(0.0, height)), axis: side, radius: die.w };
            wall_curves.push(CurveSpec { from: wv(station, a), to: wv(station, b), curve: Curve::Arc(arc) });
        }
    }
    let wall = vec![
        FaceSpec {
            id: fid("hole.side", 0, 0),
            surface: Surface::Plane(Plane { origin: start.point(Vec2::new(-die.w, 0.0)), normal: start.x }),
            loops: vec![vec![wv(0, 0), wv(1, 0), wv(1, 2 * n + 1), wv(0, 2 * n + 1)]],
        },
        FaceSpec {
            id: fid("hole.side", 1, 0),
            surface: Surface::Plane(Plane { origin: start.point(Vec2::new(die.w, 0.0)), normal: start.x * -1.0 }),
            loops: vec![vec![wv(0, n), wv(0, n + 1), wv(1, n + 1), wv(1, n)]],
        },
        FaceSpec {
            id: fid("hole.round", 0, 0),
            surface: Surface::Cylinder(Cylinder { origin: start.point(Vec2::new(0.0, 0.0)), axis: start.y, radius: die.w, convex: false }),
            loops: vec![vec![wv(0, 0), wv(0, 2 * n + 1), wv(0, n + 1), wv(0, n)]],
        },
        FaceSpec {
            id: fid("hole.round", 1, 0),
            surface: Surface::Cylinder(Cylinder { origin: end.point(Vec2::new(0.0, 0.0)), axis: end.y, radius: die.w, convex: false }),
            loops: vec![vec![wv(1, n), wv(1, n + 1), wv(1, 2 * n + 1), wv(1, 0)]],
        },
    ];
    let wall_top = vec![wv(0, 0), wv(0, n), wv(1, n), wv(1, 0)];
    let wall_bottom = vec![wv(0, 2 * n + 1), wv(1, 2 * n + 1), wv(1, n + 1), wv(0, n + 1)];
    // The tongue's own rim: the same wall, turned out of its metal.
    let rim = vec![
        FaceSpec {
            id: fid("rim.side", 0, 0),
            surface: Surface::Plane(Plane { origin: start.point(Vec2::new(-die.w, 0.0)), normal: start.x * -1.0 }),
            loops: vec![vec![vid(0, 0), vid(0, 2 * n + 1), vid(1, 2 * n + 1), vid(1, 0)]],
        },
        FaceSpec {
            id: fid("rim.side", 1, 0),
            surface: Surface::Plane(Plane { origin: start.point(Vec2::new(die.w, 0.0)), normal: start.x }),
            loops: vec![vec![vid(0, n), vid(1, n), vid(1, n + 1), vid(0, n + 1)]],
        },
        // Each end round the far side, a quarter at a time: a half circle's way round is not
        // fixed by its ends, a quarter's is.
        FaceSpec {
            id: fid("rim.round", 0, 0),
            surface: Surface::Cylinder(Cylinder { origin: start.point(Vec2::new(0.0, 0.0)), axis: start.y, radius: die.w, convex: true }),
            loops: vec![vec![vid(0, 0), qvid(0, n), qvid(0, n + 1), vid(0, 2 * n + 1)]],
        },
        FaceSpec {
            id: fid("rim.round", 0, 1),
            surface: Surface::Cylinder(Cylinder { origin: start.point(Vec2::new(0.0, 0.0)), axis: start.y, radius: die.w, convex: true }),
            loops: vec![vec![qvid(0, n), vid(0, n), vid(0, n + 1), qvid(0, n + 1)]],
        },
        FaceSpec {
            id: fid("rim.round", 1, 0),
            surface: Surface::Cylinder(Cylinder { origin: end.point(Vec2::new(0.0, 0.0)), axis: end.y, radius: die.w, convex: true }),
            loops: vec![vec![vid(1, 0), vid(1, 2 * n + 1), qvid(1, 2 * n + 1), qvid(1, 0)]],
        },
        FaceSpec {
            id: fid("rim.round", 1, 1),
            surface: Surface::Cylinder(Cylinder { origin: end.point(Vec2::new(0.0, 0.0)), axis: end.y, radius: die.w, convex: true }),
            loops: vec![vec![qvid(1, 0), qvid(1, 2 * n + 1), vid(1, n + 1), vid(1, n)]],
        },
    ];
    Ok(Louver {
        vertices,
        curves: curves.into_iter().map(|(k, curve)| CurveSpec { from: k.0, to: k.1, curve }).collect(),
        tongue: tongue.into_iter().chain(rim).collect(),
        hole: wall,
        hole_curves: wall_curves,
        top: wall_top,
        bottom: wall_bottom,
    })
}

/// The surface of the piece of the section swept along the louver's axis: a line a plane, an arc
/// a cylinder.
fn swept(piece: &Piece, a: Vec2, b: Vec2, at: Stand) -> Result<Surface, BrepError> {
    Ok(match *piece {
        Piece::Line => {
            let d = at.along(Vec2::new(b.x - a.x, b.y - a.y));
            Surface::Plane(Plane { origin: at.point(a), normal: unit(at.z.cross(d)) })
        }
        Piece::Arc { center, ccw } => {
            let rho = libm::hypot(a.x - center.x, a.y - center.y);
            Surface::Cylinder(Cylinder { origin: at.point(center), axis: at.z, radius: rho, convex: !ccw })
        }
    })
}

/// The surface of the piece of the section turned about the sheet's normal at the louver's end:
/// a line a plane, a cylinder or a cone, an arc a torus (a sphere about its centre).
fn turned(piece: &Piece, a: Vec2, b: Vec2, at: Stand) -> Result<Surface, BrepError> {
    let axis = at.y;
    let origin = at.point(Vec2::new(0.0, 0.0));
    Ok(match *piece {
        Piece::Line => {
            let d = Vec2::new(b.x - a.x, b.y - a.y);
            let left = Vec2::new(-d.y, d.x);
            if a.y == b.y {
                Surface::Plane(Plane { origin: at.point(a), normal: axis * if d.x > 0.0 { 1.0 } else { -1.0 } })
            } else if a.x == b.x {
                Surface::Cylinder(Cylinder { origin, axis, radius: a.x.abs(), convex: left.x * a.x > 0.0 })
            } else {
                if a.x == 0.0 || b.x == 0.0 {
                    return Err(BrepError::Degenerate(
                        "component: the die's section has a corner on the louver's axis: the end dome would come to a point".into(),
                    ));
                }
                let slope = d.x / d.y * if a.x > 0.0 { 1.0 } else { -1.0 };
                // The cone's apex is on the louver's axis; its origin at `a`'s height there.
                Surface::Cone(Cone { origin: at.point(Vec2::new(0.0, a.y)), axis, radius: a.x.abs(), slope, convex: left.x * a.x > 0.0 })
            }
        }
        Piece::Arc { center, ccw } => {
            let rho = libm::hypot(a.x - center.x, a.y - center.y);
            // The turned circle's centre stands on the louver's axis, `major` from it.
            Surface::Torus(Torus { center: at.point(Vec2::new(0.0, center.y)), axis, major: center.x.abs(), minor: rho, convex: !ccw })
        }
    })
}

/// `body` with louver `l` on its wall's faces `f` and `g`: the zone's contour cut into them, and
/// either the tongue that fills it or the zone's wall across the thickness (the blank's).
fn with_louver(body: &Body, f: usize, g: usize, l: &Louver, tongue: bool, curves: &[CurveSpec]) -> Result<Body, BrepError> {
    let mut faces: Vec<FaceSpec> = Vec::with_capacity(body.faces().len() + l.tongue.len() + l.hole.len());
    for x in 0..body.faces().len() {
        let mut spec = body.face_spec(x as u32);
        if x == f {
            spec.loops.push(l.top.clone());
        } else if x == g {
            spec.loops.push(l.bottom.clone());
        }
        faces.push(spec);
    }
    faces.extend(l.hole.iter().cloned());
    if tongue {
        faces.extend(l.tongue.iter().cloned());
    }
    // Only the louver's vertices its faces use: the blank keeps the zone's contour alone.
    let used: std::collections::BTreeSet<TopoId> = faces.iter().flat_map(|f| f.loops.iter().flatten().copied()).collect();
    let mut vertices: Vec<Vertex> = body.vertices().to_vec();
    vertices.extend(l.vertices.iter().filter(|v| used.contains(&v.id)).copied());
    let mut all = body.curve_specs();
    all.extend(curves.iter().copied());
    build_with_curves(vertices, faces, all)
}

/// The sheet `values[0]` formed where the tool of `def` is placed along lines (D261): the `form`
/// role's section, a chain of lines and arcs across the louver, is the die's working curve; arcs
/// of the sheet rule's radius join it to the sheet's faces, and the wall's material in the
/// forming zone is laid along it a thickness off, a tongue. The wall loses the zone, the tongue
/// takes its place, and the zone's contour is cut into the blank and its DXF. Refused: a part
/// that cuts too (a cut agreed with the die comes next) or hinges the sheet, a point placement, a
/// sheet with ribs, no sheet rule, a thickness outside the part's range, a section that is no
/// symmetric chain of lines and arcs, a die the transitions cannot reach, a zone that reaches an
/// edge, a hole, a bend or another louver.
pub(crate) fn form(
    feature: TopoId,
    def: &PartDef,
    params: &BTreeMap<String, Num>,
    placements: &[Placement],
    thickness: Option<Num>,
    radius: Option<Num>,
    values: &[&Value],
) -> Result<Value, BrepError> {
    let later = |m: String| BrepError::Unsupported(format!("component: {m}"));
    if def.roles.cut.is_some() {
        return Err(later(format!("part `{}` cuts and forms the sheet: a cut agreed with the die comes later", def.name)));
    }
    if def.roles.hinge.is_some() {
        return Err(later(format!("part `{}` has a `hinge`: forming it comes later", def.name)));
    }
    if placements.is_empty() {
        return Err(BrepError::Invalid(format!("component: part `{}` is placed nowhere: `placements` is empty", def.name)));
    }
    let (Some(thickness), Some(radius)) = (thickness, radius) else { return Err(BrepError::Invalid(crate::NO_FORM_RULE.into())) };
    let t = num(&thickness)?;
    let r = num(&radius)?;
    if let Some(range) = &def.thickness {
        let (Some(lo), Some(hi)) = (range[0].value(), range[1].value()) else {
            return Err(BrepError::Invalid(format!("component: part `{}`: `thickness` is [min, max] in mm", def.name)));
        };
        if !(lo <= t && t <= hi) {
            return Err(BrepError::Invalid(format!(
                "component: part `{}` forms sheet of {t} mm, outside its `thickness` range {lo}..={hi} mm",
                def.name
            )));
        }
    }
    if r <= t {
        return Err(BrepError::Degenerate(format!(
            "component: a bend radius of {r} mm on {t} mm sheet: the tongue's inner transition has no radius"
        )));
    }
    let mut body = match values.first() {
        Some(Value::Formed(_)) => return Err(later("forming sheet metal with ribs comes later".into())),
        Some(v) => v.body().cloned().ok_or_else(|| BrepError::Sketch("a component takes the sheet, then a sketch the placements name".into()))?,
        None => return Err(BrepError::Invalid("a component takes the sheet, then a sketch the placements name".into())),
    };
    let mut blank = body.clone();
    let mut tools: BTreeMap<Option<u64>, Tool> = BTreeMap::new();
    for (p, placement) in placements.iter().enumerate() {
        let Placement::Line(l) = placement else {
            return Err(later(format!("part `{}` forms at a point: a point stamp comes with the confirmed leaf profile", def.name)));
        };
        let (rigid, fit) = place(def, placement, values)?;
        let made = match tools.entry(fit.map(f64::to_bits)) {
            Entry::Occupied(e) => e.into_mut(),
            Entry::Vacant(e) => {
                let mut overrides = params.clone();
                if let (Some(name), Some(length)) = (def.fit.as_ref(), fit) {
                    overrides.insert(name.clone(), Num::Value(length));
                }
                e.insert(tool(def, feature, &overrides)?)
            }
        };
        let profile = made
            .form_sketch
            .as_ref()
            .ok_or_else(|| BrepError::Invalid(format!("component: part `{}` has no `form` role: nothing forms the sheet", def.name)))?;
        let [o, ex, ey, ez] = profile.frame;
        let placed = [rigid.apply(o), rigid.apply_vector(ex), rigid.apply_vector(ey), rigid.apply_vector(ez)];
        // The die's section stands across the louver: its plane normal (its extrusion, z) along
        // the line, its y out of the sheet, its origin where the tool's anchor was put.
        let (start, along, length) = line(def, l, values)?;
        let anchor = def.anchor()?;
        let size = placed.iter().flat_map(|v| v.to_array()).fold(1.0_f64, |m, x| m.max(x.abs()));
        let [so, _, sy, sz] = placed;
        let off = so - start;
        if [off.x, off.y, off.z].iter().any(|x| !within_rounding(*x, size)) || !nearly_parallel(sz, along) {
            return Err(BrepError::Invalid(format!(
                "component: the die of part `{}` is not drawn across the louver: its section's plane normal must lie along the tool's `axis`, its origin at the tool's `anchor`",
                def.name
            )));
        }
        let _ = anchor;
        let on = input_sketch(values, l.input, "a component's line placement")?;
        let face = on.face.ok_or_else(|| {
            BrepError::Invalid(format!(
                "component: a formed louver takes a sketch placed on a face of the sheet: part `{}` is placed on none",
                def.name
            ))
        })?;
        let f =
            body.faces().binary_search_by_key(&face, |g| g.id).map_err(|_| BrepError::LostReference(format!("face {face:?} is not in the body")))?;
        let Surface::Plane(plane) = body.faces()[f].surface else {
            return Err(later(format!("part `{}` forms a wall that is not plane: forming a bent wall comes later", def.name)));
        };
        let n = unit(plane.normal);
        if !nearly_parallel(sy, n) || sy.dot(n) <= 0.0 || !within_rounding((so - plane.origin).dot(n), size) {
            return Err(BrepError::Invalid(format!(
                "component: the die of part `{}` is not laid on the wall its placement names: its section's y must point out of the sheet, its origin on the face",
                def.name
            )));
        }
        let die = die_of(feature, profile, t, r)?;
        if 2.0 * die.w >= length {
            return Err(BrepError::Degenerate(format!(
                "component: a louver {length} mm long is not wider than its two end domes ({:.2} mm each)",
                die.w
            )));
        }
        // The zone stays inside the wall, off its edges, holes, bends and other louvers.
        let frame = crate::cut::Frame::Plane { o: so, x: ex, y: ez, z: ey };
        let zone = ProfileLoop {
            points: vec![Vec2::new(-die.w, die.w), Vec2::new(-die.w, length - die.w), Vec2::new(die.w, length - die.w), Vec2::new(die.w, die.w)],
            vertices: Vec::new(),
            sides: Vec::new(),
            arcs: vec![
                None,
                Some(ProfileArc { center: Vec2::new(0.0, length - die.w), ccw: false }),
                None,
                Some(ProfileArc { center: Vec2::new(0.0, die.w), ccw: false }),
            ],
        };
        let centre = Vec2::new(0.0, 0.5 * length);
        let g = opposite_wall(&body, f, &frame, centre, def, t)?;
        for h in [f, g] {
            let loops = crate::cut::outline(&body, h as u32, &frame)?;
            let inside = loops.first().is_some_and(|lp| lp.winding(centre) != 0) && loops[1..].iter().all(|lp| lp.winding(centre) == 0);
            let over = || loops[1..].iter().any(|lp| zone.winding(lp.points[0]) != 0);
            if !inside || loops.iter().any(|lp| lp.touches(&zone)) || over() {
                return Err(BrepError::Sketch(format!(
                    "component: the louver along `{}` does not stay inside its wall: its zone runs {:.2} mm each side of the line and past its ends, and reaches an edge, a hole, a bend or another louver",
                    l.line, die.w
                )));
            }
        }
        let start_at = Stand { o: so, x: ex, y: ey, z: ez, at: die.w };
        let end_at = Stand { o: so, x: ex, y: ey, z: ez, at: length - die.w };
        let louv = louver(feature, p, &die, start_at, end_at)?;
        let mut body_curves = louv.hole_curves.clone();
        body_curves.extend(louv.curves.iter().copied());
        body = with_louver(&body, f, g, &louv, true, &body_curves)?;
        blank = with_louver(&blank, f, g, &louv, false, &louv.hole_curves)?;
    }
    crate::validated(&body)?;
    crate::validated(&blank)?;
    Ok(Value::Formed(Box::new(crate::rib::Formed { body, blank, imprints: Vec::new() })))
}

/// The face of `body` across the wall `f` from the sheet's other side: the nearest plane facing
/// the other way with the zone's centre over it, `t` from `f`.
fn opposite_wall(body: &Body, f: usize, frame: &crate::cut::Frame, centre: Vec2, def: &PartDef, t: f64) -> Result<usize, BrepError> {
    let Surface::Plane(plane) = body.faces()[f].surface else {
        return Err(BrepError::Invalid("component: forming takes a plane wall".into()));
    };
    let n = unit(plane.normal);
    let holds = |g: usize| -> Result<bool, BrepError> {
        let loops = crate::cut::outline(body, g as u32, frame)?;
        Ok(loops.first().is_some_and(|l| l.winding(centre) != 0) && loops[1..].iter().all(|l| l.winding(centre) == 0))
    };
    let mut across: Vec<(f64, usize)> = Vec::new();
    for (g, x) in body.faces().iter().enumerate() {
        if let Surface::Plane(p) = x.surface
            && p.normal.cross(plane.normal) == Vec3::ZERO
            && p.normal.dot(plane.normal) < 0.0
        {
            let d = (p.origin - plane.origin).dot(-n);
            if d > 0.0 && holds(g)? {
                across.push((d, g));
            }
        }
    }
    across.sort_by(|a, b| a.0.total_cmp(&b.0));
    let &(thickness, g) = across.first().ok_or_else(|| {
        BrepError::Invalid(format!("component: the wall of part `{}` is not {t} mm thick: no face across it under the louver", def.name))
    })?;
    if !within_rounding(thickness - t, thickness) {
        return Err(BrepError::Invalid(format!("component: part `{}` forms a wall {thickness} mm thick, and the sheet rule gives {t} mm", def.name)));
    }
    Ok(g)
}
