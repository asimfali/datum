//! A library part placed in a part (D260): the definition of the tool, its parameters and its
//! features, the body built once and copied to every placement. The definition is a copy of the
//! library element, written into the model: the model is built without the library (D256).

use std::collections::BTreeMap;
use std::collections::btree_map::Entry;

use datum_brep::{Body, BrepError, CurveSpec, FaceSpec, TopoId, Vertex, build_with_curves};
use datum_math::expr::{ExprError, Num, parameter_values, valid_name};
use datum_math::{Rigid, Vec3};
use schemars::JsonSchema;
use serde::{Deserialize, Deserializer, Serialize};

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
/// feature, and the bodies of the features the roles name, for the tasks that cut and form the
/// sheet.
pub struct Tool {
    pub body: Body,
    pub cut: Option<Body>,
    pub form: Option<Body>,
    pub hinge: Option<Body>,
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
    Ok(Tool { body: last, cut: role(&def.roles.cut)?, form: role(&def.roles.form)?, hinge: role(&def.roles.hinge)? })
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

/// The tool of `def` placed at each of `placements`, as one body: a copy per placement, its
/// elements named by the occurrence, the placement's place and the tool's (§4.2). The body is
/// built once for a definition and a set of parameter values: placements that feed the same
/// length to `fit` share it.
pub fn placed(
    feature: TopoId,
    def: &PartDef,
    params: &BTreeMap<String, Num>,
    placements: &[Placement],
    values: &[&Value],
) -> Result<Body, BrepError> {
    if placements.is_empty() {
        return Err(BrepError::Invalid(format!("component: part `{}` is placed nowhere: `placements` is empty", def.name)));
    }
    let mut tools: BTreeMap<Option<u64>, Tool> = BTreeMap::new();
    let (mut vertices, mut faces, mut curves) = (Vec::new(), Vec::new(), Vec::new());
    for (p, placement) in placements.iter().enumerate() {
        let (rigid, fit) = place(def, placement, values)?;
        let key = fit.map(f64::to_bits);
        let made = match tools.entry(key) {
            Entry::Occupied(e) => e.into_mut(),
            Entry::Vacant(e) => {
                let mut overrides = params.clone();
                if let (Some(name), Some(length)) = (def.fit.as_ref(), fit) {
                    overrides.insert(name.clone(), Num::Value(length));
                }
                e.insert(tool(def, feature, &overrides)?)
            }
        };
        let id = |t: TopoId| TopoId::derive("component.copy").id(feature).index(p as u64).id(t).finish();
        let (v, f, c) = copy(&made.body, &rigid, &id);
        vertices.extend(v);
        faces.extend(f);
        curves.extend(c);
    }
    build_with_curves(vertices, faces, curves)
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

/// `body` moved by `m` as vertex, face and curve specs, its elements named by `id`.
fn copy(body: &Body, m: &Rigid, id: &impl Fn(TopoId) -> TopoId) -> (Vec<Vertex>, Vec<FaceSpec>, Vec<CurveSpec>) {
    let vertices = body.vertices().iter().map(|v| Vertex { id: id(v.id), point: m.apply(v.point) }).collect();
    let faces = (0..body.faces().len() as u32)
        .map(|f| {
            let mut spec = body.face_spec(f);
            spec.id = id(spec.id);
            spec.surface = spec.surface.moved(m);
            spec.loops.iter_mut().flatten().for_each(|v| *v = id(*v));
            spec
        })
        .collect();
    let curves = body.curve_specs().into_iter().map(|c| CurveSpec { from: id(c.from), to: id(c.to), curve: c.curve.moved(m) }).collect();
    (vertices, faces, curves)
}
