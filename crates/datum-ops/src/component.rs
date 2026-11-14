//! A library part placed in a part (D260): the definition of the tool, its parameters and its
//! features, the body built once and copied to every placement. The definition is a copy of the
//! library element, written into the model: the model is built without the library (D256).

use std::collections::BTreeMap;
use std::collections::btree_map::Entry;

use datum_brep::{
    Arc, Body, BrepError, Cone, Curve, CurveSpec, Cylinder, FaceSpec, Plane, ProfileArc, Surface, TopoId, Torus, Vertex, build_with_curves,
};
use datum_math::Rigid;
use datum_math::expr::{ExprError, Num, parameter_values, valid_name};
use datum_math::{Vec2, Vec3};
use schemars::JsonSchema;
use serde::{Deserialize, Deserializer, Serialize};

use crate::sketch::{Placed, nearly_parallel, open_chains_named, within_rounding};
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

/// The strip of a louver (D261): the `form` role's section from the hinge line on the louver's
/// axis to its free end, and the same a thickness `t` off into the material, closed across the
/// thickness at both ends. The loop: the profile `0..=n`, the free end, the offset back, the
/// hinge's end; the pieces are between its points.
struct Strip {
    points: Vec<Vec2>,
    pieces: Vec<Piece>,
    /// The free end's distance from the axis: where the strip's middle ends.
    u: f64,
}

/// The strip's section from the `form` role's sketch `s`, in the section's plane: x from the
/// louver's axis (the hinge, x = 0) out to the free end, y up from the sheet's face (y = 0). The
/// section is one closed outline of lines and arcs; its working curve is the side below the chord
/// between its widest points, from the hinge on the axis to the free end, dipping below the face.
/// The strip is that curve and its offset a thickness `t` into the material.
fn strip_of(s: &Placed, t: f64) -> Result<Strip, BrepError> {
    let later = |m: &str| BrepError::Unsupported(format!("component: {m}"));
    let short = |m: &str| BrepError::Degenerate(format!("component: {m}"));
    if s.developed.is_some() {
        return Err(later("forming on a rolled sheet comes later: place the die on a flat wall"));
    }
    if !s.circles.is_empty() || !s.ellipses.is_empty() {
        return Err(later("the strip's section is a chain of lines and arcs: no circles or ellipses"));
    }
    let chains = open_chains_named(s, &|_| true)?;
    let chain = match chains.as_slice() {
        [one] => one,
        [] => return Err(later("the strip's section is one open chain of lines and arcs: the block's `form` feature is its sketch")),
        _ => return Err(later("the strip's section is one chain of lines and arcs")),
    };
    let mut pieces: Vec<Piece> = Vec::with_capacity(chain.len());
    for (_, c) in chain {
        pieces.push(match c.arc {
            None => Piece::Line,
            Some(a) => Piece::Arc { center: a.center, ccw: a.ccw },
        });
    }
    let verts: Vec<Vec2> = std::iter::once(chain[0].1.from).chain(chain.iter().map(|(_, c)| c.to)).collect();
    let size = verts.iter().fold(1.0_f64, |m, p| m.max(p.x.abs()).max(p.y.abs()));
    let same = |a: Vec2, b: Vec2| within_rounding(a.x - b.x, size) && within_rounding(a.y - b.y, size);
    // From the hinge on the louver's axis, on the sheet's face, out to the free end.
    if !same(verts[0], Vec2::new(0.0, 0.0)) {
        return Err(BrepError::Invalid(
            "component: the strip's profile leaves its hinge line: the section's first point must stand on the louver's axis, on the sheet's face"
                .into(),
        ));
    }
    if verts.iter().all(|p| p.y == 0.0) {
        return Err(short("the strip's section does not leave the sheet's face: it stays on it"));
    }
    if verts.windows(2).any(|w| w[0].x >= w[1].x) {
        return Err(later("the strip's section runs from its hinge line out to its free end, once"));
    }
    // The ends of the domes fix their way round only under a half turn.
    for (k, piece) in pieces.iter().enumerate() {
        let Piece::Arc { center, ccw } = *piece else { continue };
        let arc = ProfileArc { center, ccw };
        if arc.turn(verts[k], verts[k + 1]) >= std::f64::consts::PI {
            return Err(short("the strip's section has an arc of a half turn or more: the end dome cannot run it"));
        }
    }
    // The strip: the chain, its offset a thickness into the material, closed across it at both ends.
    let n = pieces.len();
    let mut low_v: Vec<Vec2> = Vec::with_capacity(n + 1);
    let mut low_p: Vec<Piece> = Vec::with_capacity(n);
    for k in 0..n {
        let (a, b, piece) = offset(&pieces[k], verts[k], verts[k + 1], t)?;
        if k == 0 {
            low_v.push(a);
        }
        low_v.push(b);
        low_p.push(piece);
    }
    let u = verts[n].x;
    let mut points = verts;
    points.extend(low_v.iter().rev().copied());
    let mut loop_pieces = pieces;
    loop_pieces.push(Piece::Line);
    for k in (0..n).rev() {
        loop_pieces.push(reverse_piece(&low_p[k]));
    }
    loop_pieces.push(Piece::Line);
    Ok(Strip { points, pieces: loop_pieces, u })
}

/// The arc of `radius` from `from` to `to` about `center`, its axis the short way's: these arcs
/// run under a half turn, whose ends and way round fix the piece of the circle they are (a half's
/// do not, both halves serve). None when the ends are one point or stand a half turn apart.
fn arc_between(center: Vec3, from: Vec3, to: Vec3, radius: f64) -> Option<Arc> {
    let cross = (from - center).cross(to - center);
    (cross.dot(cross) > 0.0).then(|| Arc { center, axis: unit(cross), radius })
}

/// Puts the arc between the two vertices; the same pair may come twice (the patches of two
/// pieces share the parallel) but with the same piece of the circle: otherwise the indexes
/// disagree. Returns why not, naming both arcs.
fn put(curves: &mut BTreeMap<(TopoId, TopoId), (TopoId, Arc)>, from: TopoId, to: TopoId, arc: Arc) -> Result<(), BrepError> {
    let key = (from.min(to), from.max(to));
    let Some(&(was_from, was)) = curves.get(&key) else {
        curves.insert(key, (from, arc));
        return Ok(());
    };
    // The patches of two pieces by the same parallel: the same piece of the circle. The arcs come
    // from the die's own numbers by rigid turns, so theirs agree to the bits a turn leaves (D229);
    // anything more is indexes that disagree.
    let size = was.radius.max(1.0);
    let off = arc.center - was.center;
    let cross = arc.axis.cross(was.axis);
    let same_way = arc.axis.dot(was.axis) > 0.0;
    let was_to = if was_from == key.0 { key.1 } else { key.0 };
    let agrees = within_rounding(arc.radius - was.radius, size)
        && within_rounding(off.dot(off).sqrt(), size)
        && within_rounding(cross.dot(cross).sqrt(), 1.0)
        && ((from == was_from && same_way) || (from == was_to && !same_way));
    if agrees {
        return Ok(());
    }
    Err(BrepError::Invalid(format!(
        "component: two arcs between the same two points disagree: first {:?} -> {:?} about {:?} r {} axis {:?}, again {:?} -> {:?} about {:?} r {} axis {:?}",
        was_from,
        was_to,
        was.center.to_array(),
        was.radius,
        was.axis.to_array(),
        from,
        to,
        arc.center.to_array(),
        arc.radius,
        arc.axis.to_array()
    )))
}

/// `p` mirrored across the louver's axis.
fn mirror(p: Vec2) -> Vec2 {
    Vec2::new(-p.x, p.y)
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

/// The strip's body at placement `p` (D261): its middle from `start` to `end` along the louver,
/// and a half turn of the strip out of each end, the turn cut at a right angle into two quarters
/// (a quarter's ends and axis fix its piece of the circle alone). A point on the hinge line
/// (x = 0) does not move under the turn: its quarter vertex is its own.
fn strip(feature: TopoId, p: usize, st: &Strip, start: Stand, end: Stand) -> Result<Body, BrepError> {
    let m = st.pieces.len();
    let tag = |name: &str, station: u64, k: usize| TopoId::derive(name).id(feature).index(p as u64).index(station).index(k as u64).finish();
    let stand = |station: u64| if station == 0 { start } else { end };
    let on_axis = |k: usize| st.points[k].x == 0.0;
    let vid = |station: u64, k: usize| tag("component.strip.v", station, k);
    let cvid = |station: u64, k: usize| tag("component.strip.c", station, k);
    let qvid = |station: u64, k: usize| if on_axis(k) { vid(station, k) } else { tag("component.strip.q", station, k) };
    let rvid = |station: u64, k: usize| if on_axis(k) { vid(station, k) } else { cvid(station, k) };
    let fid = |kind: &str, k: usize, station: u64| {
        TopoId::derive("component.strip.f").id(feature).index(p as u64).tag(kind).index(k as u64).index(station).finish()
    };
    let mut vertices: Vec<Vertex> = Vec::with_capacity(6 * (m + 1));
    let mut curves: BTreeMap<(TopoId, TopoId), (TopoId, Arc)> = BTreeMap::new();
    for station in 0..2u64 {
        let at = stand(station);
        for (k, q) in st.points.iter().enumerate() {
            vertices.push(Vertex { id: vid(station, k), point: at.point(*q) });
            if !on_axis(k) {
                vertices.push(Vertex { id: cvid(station, k), point: at.point(mirror(*q)) });
                vertices.push(Vertex { id: qvid(station, k), point: at.quarter(*q) });
            }
        }
        for k in 0..m {
            let Piece::Arc { center, ccw } = st.pieces[k] else { continue };
            let a = st.points[k];
            let rho = libm::hypot(a.x - center.x, a.y - center.y);
            // The loop's own arc, its copy across the axis, and the meridian at the right angle.
            let arc = Arc { center: at.point(center), axis: at.z * if ccw { 1.0 } else { -1.0 }, radius: rho };
            put(&mut curves, vid(station, k), vid(station, k + 1), arc)?;
            if !on_axis(k) && !on_axis(k + 1) {
                let (p0, p1) = (at.point(mirror(st.points[k])), at.point(mirror(st.points[k + 1])));
                if let Some(arc) = arc_between(at.point(mirror(center)), p0, p1, rho) {
                    put(&mut curves, cvid(station, k), cvid(station, k + 1), arc)?;
                }
                let (q0, q1) = (at.quarter(st.points[k]), at.quarter(st.points[k + 1]));
                if let Some(arc) = arc_between(at.quarter(center), q0, q1, rho) {
                    put(&mut curves, qvid(station, k), qvid(station, k + 1), arc)?;
                }
            }
        }
    }
    let mut faces: Vec<FaceSpec> = Vec::with_capacity(3 * m);
    // The middle: each piece of the loop swept from one end to the other. The hinge's end, on the
    // axis, sweeps into the strip's side face.
    for k in 0..m {
        let k1 = (k + 1) % m;
        let surface = swept(&st.pieces[k], st.points[k], st.points[k1], start)?;
        faces.push(FaceSpec { id: fid("sweep", k, 0), surface, loops: vec![vec![vid(0, k), vid(1, k), vid(1, k1), vid(0, k1)]] });
    }
    // The ends: the strip turned a half out of the middle, a quarter at a time, and its face where
    // the turn leaves it (the strip's cut across the thickness).
    for station in 0..2u64 {
        let at = stand(station);
        for k in 0..m - 1 {
            let surface = turned(&st.pieces[k], st.points[k], st.points[k + 1], at)?;
            let mut first = vec![vid(station, k), vid(station, k + 1), qvid(station, k + 1), qvid(station, k)];
            let mut second = vec![qvid(station, k), qvid(station, k + 1), rvid(station, k + 1), rvid(station, k)];
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
            // The parallels: from the piece to its copy across the axis, round the far side.
            for (a, b) in [(vid(station, k), qvid(station, k)), (qvid(station, k), rvid(station, k))] {
                if a == b {
                    continue;
                }
                let height = st.points[k].y;
                let radius = st.points[k].x.abs();
                let (pa, pb) =
                    (at.point(st.points[k]), if a == qvid(station, k) { at.quarter(st.points[k]) } else { at.point(mirror(st.points[k])) });
                if let Some(arc) = arc_between(at.point(Vec2::new(0.0, height)), pa, pb, radius) {
                    put(&mut curves, a, b, arc)?;
                }
            }
            faces.push(FaceSpec { id: fid("dome", k, station), surface, loops: vec![first] });
            faces.push(FaceSpec { id: fid("dome.q", k, station), surface, loops: vec![second] });
        }
        // The strip's cut across the thickness, where the turn leaves it.
        let ring: Vec<TopoId> = (0..m).map(|k| rvid(station, k)).collect();
        let area: f64 = (0..m)
            .map(|k| {
                let (a, b) = (mirror(st.points[k]), mirror(st.points[(k + 1) % m]));
                a.x * b.y - a.y * b.x
            })
            .sum();
        let normal = at.z * if area > 0.0 { 1.0 } else { -1.0 };
        faces.push(FaceSpec {
            id: fid("cut", 0, station),
            surface: Surface::Plane(Plane { origin: at.point(Vec2::new(0.0, 0.0)), normal }),
            loops: vec![ring],
        });
    }
    let used: std::collections::BTreeSet<TopoId> = faces.iter().flat_map(|f| f.loops.iter().flatten().copied()).collect();
    vertices.retain(|v| used.contains(&v.id));
    build_with_curves(
        vertices,
        faces,
        curves
            .into_iter()
            .map(|((low, high), (from, arc))| CurveSpec { from, to: if from == low { high } else { low }, curve: Curve::Arc(arc) })
            .collect(),
    )
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

/// The surface of the piece of the section turned about the sheet's normal at the strip's end:
/// a line a plane, a cylinder or a cone, an arc a torus (a sphere about its centre).
fn turned(piece: &Piece, a: Vec2, b: Vec2, at: Stand) -> Result<Surface, BrepError> {
    let axis = at.y;
    let origin = at.point(Vec2::new(0.0, 0.0));
    Ok(match *piece {
        Piece::Line => {
            let d = Vec2::new(b.x - a.x, b.y - a.y);
            let left = Vec2::new(-d.y, d.x);
            if a.y == b.y {
                Surface::Plane(Plane { origin: at.point(a), normal: if d.x > 0.0 { axis } else { axis * -1.0 } })
            } else if a.x == b.x {
                Surface::Cylinder(Cylinder { origin, axis, radius: a.x.abs(), convex: left.x * a.x > 0.0 })
            } else {
                if a.x == 0.0 || b.x == 0.0 {
                    return Err(BrepError::Degenerate(
                        "component: the strip's section has a corner on the louver's axis: the end dome would come to a point".into(),
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

/// TEMPORARY (D261, the strip's step): the strip's body of the section sketch `s`, a sheet `t`
/// thick, `length` long, its middle from `u` to `length - u` along the sketch's z.
pub fn strip_body(feature: TopoId, s: &Placed, t: f64, length: f64) -> Result<Body, BrepError> {
    let st = strip_of(s, t)?;
    let [o, ex, ey, ez] = s.frame;
    let at = |z: f64| Stand { o, x: ex, y: ey, z: ez, at: z };
    strip(feature, 0, &st, at(st.u), at(length - st.u))
}

/// The sheet `values[0]` formed where the tool of `def` is placed along lines (D261): the `form`
/// role's section is the strip's profile, from its hinge line on the louver's axis out to its free
/// end; the strip is that profile and its offset a thickness `t` into the material. The middle is
/// the strip swept along the line, the ends a half turn of it. Refused for now: the strip's
/// placement into the sheet comes in the next step; a part that cuts too, a hinge, a point
/// placement, no sheet rule, a thickness outside the part's range, a profile whose hinge end is
/// off the axis, one that does not press into the sheet, an arc of a half turn or more.
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
    let _ = num(&radius)?;
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
    let mut tools: BTreeMap<Option<u64>, Tool> = BTreeMap::new();
    for placement in placements {
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
        let (start, along, length) = line(def, l, values)?;
        let size = placed.iter().flat_map(|v| v.to_array()).fold(1.0_f64, |m, x| m.max(x.abs()));
        let [so, _, sy, sz] = placed;
        let off = so - start;
        if [off.x, off.y, off.z].iter().any(|x| !within_rounding(*x, size)) || !nearly_parallel(sz, along) {
            return Err(BrepError::Invalid(format!(
                "component: the strip of part `{}` is not drawn across the louver: its section's plane normal must lie along the tool's `axis`, its origin at the tool's `anchor`",
                def.name
            )));
        }
        let on = input_sketch(values, l.input, "a component's line placement")?;
        if on.face.is_none() {
            return Err(BrepError::Invalid(format!(
                "component: a formed strip takes a sketch placed on a face of the sheet: part `{}` is placed on none",
                def.name
            )));
        }
        let st = strip_of(profile, t)?;
        if 2.0 * st.u >= length {
            return Err(BrepError::Degenerate(format!(
                "component: a strip {length} mm long is not wider than its two end domes ({:.2} mm each)",
                st.u
            )));
        }
        let _ = (sy, ez);
        // The strip's body: the middle between the domes and the domes themselves.
        let _ = strip(feature, 0, &st, Stand { o: so, x: ex, y: ey, z: ez, at: st.u }, Stand { o: so, x: ex, y: ey, z: ez, at: length - st.u })?;
    }
    Err(later("the strip's placement into the sheet comes in the next step".into()))
}
