//! Levels of a model (D152, D169): the root (the file's own `parameters` and `features`) is a
//! multi-body part, the product's base variant; its parts are its bodies, each named by the
//! feature that began it; the variants of a part are levels below the root, each marked with its
//! part. A level holds only what differs from its parent:
//! parameters, the parent's features changed (by name, merged as JSON merge patches: a sketch
//! gains holes, a size changes, the rest is inherited) and features of its own. A level is
//! evaluated as an ordinary graph made of all that: inherited features keep their names, so their
//! IDs and cache keys are the parent's, and what the variants share is computed once.

use std::borrow::Cow;
use std::collections::BTreeMap;

use datum_graph::{Graph, NodeIx};
use datum_math::expr::Num;
use serde::{Deserialize, Serialize};
use serde_json::{Map, Value};

use datum_ops::Op;

use crate::{add_feature, feature_json};

/// A level, as the file has it.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Level {
    /// The level it differs from; none: the root.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub parent: Option<String>,
    /// The part it is a variant of (by the feature that began its body), on a level right below
    /// the root; the levels below it are of its part.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub part: Option<String>,
    /// The rule (D179, D189) its part takes in it; none: the parent's.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rule: Option<String>,
    /// Parameters it sets: the parent's given other values, or its own.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub parameters: BTreeMap<String, Num>,
    /// The parent's features changed (`name` and the fields that differ; `null` drops a field)
    /// and its own features, whole.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub features: Vec<Map<String, Value>>,
    /// Files before D169: the bodies a part level made. Read to bring them over, never written.
    #[serde(default, skip_serializing)]
    pub output: Option<Vec<String>>,
}

/// What a body is as a part (D169): its name (none: the feature that began it) and its rule
/// (D179, D189: a library rule by name, its material the part's; none: a part of no material).
/// Its designation comes here.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PartInfo {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rule: Option<String>,
}

/// A product variant (D169): the variant of each part and the parts hidden, under a name, to
/// switch the product at once.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Set {
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub chosen: BTreeMap<String, String>,
    #[serde(default, skip_serializing_if = "std::collections::BTreeSet::is_empty")]
    pub hidden: std::collections::BTreeSet<String>,
}

/// How a body is laid flat (D164): seen from its back side, at a K-factor.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FlatSettings {
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub back: bool,
    #[serde(default = "half", skip_serializing_if = "is_half")]
    pub k_factor: Num,
}

fn half() -> Num {
    Num::Value(0.5)
}

fn is_half(n: &Num) -> bool {
    *n == half()
}

impl Default for FlatSettings {
    fn default() -> Self {
        Self { back: false, k_factor: half() }
    }
}

/// A whole model: the root's graph, the levels by name, the parts' names, and the variant of each
/// part that goes into the product (none: the root's own). Parts are keyed by the feature that
/// began their body.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Doc {
    pub root: Graph,
    pub levels: BTreeMap<String, Level>,
    /// What a body is as a part; none: not a part (an envelope, a surface to cut by).
    pub parts: BTreeMap<String, Option<PartInfo>>,
    pub chosen: BTreeMap<String, String>,
    /// Parts not shown in the product.
    pub hidden: std::collections::BTreeSet<String>,
    /// How each body (by its feature) is laid flat, when not the default.
    pub flats: BTreeMap<String, FlatSettings>,
    /// The product's variants by name.
    pub sets: BTreeMap<String, Set>,
}

impl Doc {
    /// The graph level `name` evaluates to (none: the root).
    pub fn graph(&self, name: Option<&str>) -> Result<Graph, String> {
        let Some(name) = name else { return Ok(self.root.clone()) };
        let level = self.levels.get(name).ok_or_else(|| format!("no level `{name}`"))?;
        let parent = self.chain_check(name)?;
        let base = self.graph(parent.as_deref())?;
        apply(&base, level).map_err(|e| format!("level `{name}`: {e}"))
    }

    /// The graph of the root and of every level, in the order of [`Self::tree`]: what a check
    /// of the whole model goes through.
    pub fn graphs(&self) -> Vec<(Option<String>, Result<Graph, String>)> {
        let mut out = vec![(None, Ok(self.root.clone()))];
        out.extend(self.tree().into_iter().map(|(n, _, _)| (Some(n.clone()), self.graph(Some(&n)))));
        out
    }

    /// The parent of level `name`, if the chain up from it reaches the root without a loop.
    fn chain_check(&self, name: &str) -> Result<Option<String>, String> {
        let mut seen = vec![name];
        let mut at = name;
        while let Some(p) = self.levels.get(at).and_then(|l| l.parent.as_deref()) {
            if !self.levels.contains_key(p) {
                return Err(format!("level `{at}`: no parent level `{p}`"));
            }
            if seen.contains(&p) {
                return Err(format!("level `{name}`: its parents go round in a loop"));
            }
            seen.push(p);
            at = p;
        }
        Ok(self.levels[name].parent.clone())
    }

    /// Level `name` (none: the root) made to evaluate to `graph`: what differs from its parent
    /// kept as the level. The parent's features and parameters cannot go in a level: a variant
    /// sets them otherwise (a feature's `enabled`, a parameter's value), it does not drop them.
    pub fn store(&mut self, name: Option<&str>, graph: &Graph) -> Result<(), String> {
        let Some(name) = name else {
            self.root = graph.clone();
            return Ok(());
        };
        let parent = self.chain_check(name)?;
        let base = self.graph(parent.as_deref())?;
        let d = differ(&base, graph)?;
        let level = self.levels.get_mut(name).ok_or_else(|| format!("no level `{name}`"))?;
        (level.parameters, level.features) = (d.parameters, d.features);
        Ok(())
    }

    /// A new variant `name` of part `part` below `parent` (none: the root), the same as it to
    /// begin with. Below a variant, the part is that variant's (`part` may be left out).
    pub fn add_level(&mut self, name: &str, parent: Option<&str>, part: Option<&str>) -> Result<(), String> {
        if name.trim().is_empty() || self.levels.contains_key(name) {
            return Err(format!("a level needs a name of its own: `{name}`"));
        }
        let part = match parent {
            Some(p) => {
                let of = self.part_of(p)?;
                if part.is_some_and(|q| q != of) {
                    return Err(format!("`{p}` is a variant of `{of}`, not of `{}`", part.unwrap_or_default()));
                }
                None
            }
            None => {
                let part = part.ok_or("a variant below the root needs `part`: the body it varies")?;
                if !self.parts().iter().any(|p| p == part) {
                    return Err(format!("no part `{part}`: a part is a body of the root, named by the feature that begins it"));
                }
                Some(part.to_string())
            }
        };
        self.levels.insert(name.to_string(), Level { parent: parent.map(String::from), part, ..Level::default() });
        Ok(())
    }

    /// Level `name` removed, if nothing is below it.
    pub fn remove_level(&mut self, name: &str) -> Result<(), String> {
        if !self.levels.contains_key(name) {
            return Err(format!("no level `{name}`"));
        }
        if let Some((child, _)) = self.levels.iter().find(|(_, l)| l.parent.as_deref() == Some(name)) {
            return Err(format!("`{child}` is below `{name}`: remove it first"));
        }
        self.levels.remove(name);
        self.chosen.retain(|_, c| c != name);
        for set in self.sets.values_mut() {
            set.chosen.retain(|_, c| c != name);
        }
        self.hidden.remove(name);
        Ok(())
    }

    /// The levels as a tree: each with its parent, the root's children first, then theirs, by name.
    pub fn tree(&self) -> Vec<(String, Option<String>, usize)> {
        // Depth first: a level, then what is below it.
        fn down(doc: &Doc, parent: Option<&str>, depth: usize, out: &mut Vec<(String, Option<String>, usize)>) {
            for (n, l) in doc.levels.iter().filter(|(_, l)| l.parent.as_deref() == parent) {
                out.push((n.clone(), l.parent.clone(), depth));
                down(doc, Some(n), depth + 1, out);
            }
        }
        let mut out = Vec::new();
        down(self, None, 0, &mut out);
        out
    }

    /// What level `name` does to its parent: the parent's features it changes, its own features,
    /// the parent's parameters it gives other values, its own parameters.
    pub fn changes(&self, name: &str) -> Result<[Vec<String>; 4], String> {
        let level = self.levels.get(name).ok_or_else(|| format!("no level `{name}`"))?;
        let parent = self.graph(self.chain_check(name)?.as_deref())?;
        let (mut changed, mut own) = (Vec::new(), Vec::new());
        for f in level.features.iter().filter_map(|f| f.get("name").and_then(Value::as_str)) {
            if parent.find(f).is_some() { changed.push(f.to_string()) } else { own.push(f.to_string()) }
        }
        let (set, new): (Vec<String>, Vec<String>) = level.parameters.keys().cloned().partition(|n| parent.parameters().contains_key(n));
        Ok([changed, own, set, new])
    }

    /// Feature `feature` or parameter `parameter` of level `name` as its parent has it again:
    /// what the level changed in it dropped.
    pub fn revert(&mut self, name: &str, feature: Option<&str>, parameter: Option<&str>) -> Result<(), String> {
        let [changed, _, set, _] = self.changes(name)?;
        let level = self.levels.get_mut(name).ok_or_else(|| format!("no level `{name}`"))?;
        if let Some(f) = feature {
            if !changed.iter().any(|c| c == f) {
                return Err(format!("`{name}` does not change the parent's feature `{f}`"));
            }
            level.features.retain(|p| p.get("name").and_then(Value::as_str) != Some(f));
        }
        if let Some(p) = parameter {
            if !set.iter().any(|c| c == p) {
                return Err(format!("`{name}` does not give the parent's parameter `{p}` another value"));
            }
            level.parameters.remove(p);
        }
        Ok(())
    }

    /// The parts: the root's bodies (by the feature that began each), but those marked as none.
    pub fn parts(&self) -> Vec<String> {
        bodies(&self.root).into_iter().map(|(begun, _)| begun).filter(|b| !matches!(self.parts.get(b), Some(None))).collect()
    }

    /// Part `part`'s name: as given, else the feature that began it.
    pub fn part_name(&self, part: &str) -> String {
        self.parts.get(part).cloned().flatten().and_then(|p| p.name).unwrap_or_else(|| part.to_string())
    }

    /// What body `part` is as a part: a part as any other is not written down.
    pub fn put_part(&mut self, part: &str, entry: Option<PartInfo>) {
        if entry == Some(PartInfo::default()) {
            self.parts.remove(part);
        } else {
            self.parts.insert(part.to_string(), entry);
        }
    }

    /// The rule of part `part` in level `level` (none: the root): the nearest level's on the way
    /// up that gives one, else the part's own.
    pub fn rule(&self, part: &str, level: Option<&str>) -> Option<String> {
        let up = std::iter::successors(level.map(String::from), |l| self.levels.get(l).and_then(|l| l.parent.clone()));
        up.filter_map(|l| self.levels.get(&l)?.rule.clone()).next().or_else(|| self.parts.get(part).cloned().flatten()?.rule)
    }

    /// Graph `g` of level `level` (none: the root) as it is built (D193, D197): each sheet metal
    /// part's thickness and bend radius from its rule (the library of `project`), their only
    /// place, and its K-factor, if the rule gives one, into the sketches on faces of its body. A
    /// part without a rule, or whose rule the library lacks, keeps the feature as it is: a file
    /// from before rules has its own numbers, a new one fails asking for a rule. The library is
    /// read only when a sheet metal part has a rule.
    pub fn built<'g>(&self, g: &'g Graph, level: Option<&str>, project: Option<&std::path::Path>) -> Cow<'g, Graph> {
        let sheets: Vec<(usize, String)> = g
            .nodes()
            .iter()
            .enumerate()
            .filter(|(_, n)| matches!(n.op, Op::Sheet { .. } | Op::Plate { .. }))
            .filter_map(|(ix, n)| Some((ix, self.rule(&n.name, level)?)))
            .collect();
        if sheets.is_empty() {
            return Cow::Borrowed(g);
        }
        let rules = crate::library::load("rule", project);
        let mut out = g.clone();
        let mut ks = std::collections::BTreeMap::new();
        for (ix, rule) in sheets {
            let Some(rule) = rules.get(&rule) else { continue };
            if let Some(k) = rule.element["k_factor"].as_f64() {
                ks.insert(ix as NodeIx, k);
            }
            let Some(t) = rule.element["thickness"].as_f64() else { continue };
            let Ok(r) = crate::library::bend_radius(&rule.element["radius"], t) else { continue };
            let mut op = g.nodes()[ix].op.clone();
            match &mut op {
                Op::Sheet { thickness, radius, .. } => (*thickness, *radius) = (Some(Num::Value(t)), Some(Num::Value(r))),
                Op::Plate { thickness, .. } => *thickness = Some(Num::Value(t)),
                _ => {}
            }
            // The same operation with other numbers: its arity holds.
            let _ = out.set_op(ix as NodeIx, op);
        }
        // A sketch on a face of a part's body lays its bends flat with the rule's K.
        for (ix, n) in g.nodes().iter().enumerate() {
            let Op::Sketch { face: Some(_), .. } = &n.op else { continue };
            let Some(&body) = n.inputs.first() else { continue };
            let Some(&k) = body_chain(g, body).last().and_then(|begun| ks.get(&begun)) else { continue };
            let mut op = n.op.clone();
            if let Op::Sketch { k_factor, .. } = &mut op {
                *k_factor = Some(Num::Value(k));
            }
            let _ = out.set_op(ix as NodeIx, op);
        }
        Cow::Owned(out)
    }

    /// The kind of part `part` (D179): `sheet` if its body began as sheet metal (from a path, or a
    /// face of it, D195), else `part`. Set by its first feature, it never changes.
    pub fn kind(&self, part: &str) -> &'static str {
        match self.root.find(part).map(|i| &self.root.nodes()[i as usize].op) {
            Some(Op::Sheet { .. } | Op::Plate { .. }) => "sheet",
            _ => "part",
        }
    }

    /// The part level `name` is a variant of.
    pub fn part_of(&self, name: &str) -> Result<String, String> {
        self.chain_check(name)?;
        let mut at = name.to_string();
        while let Some(p) = self.levels.get(&at).and_then(|l| l.parent.clone()) {
            at = p;
        }
        self.levels.get(&at).and_then(|l| l.part.clone()).ok_or_else(|| format!("level `{at}` is no part's variant"))
    }

    /// The variant of part `part` that goes into the product; none: the root's own.
    pub fn chosen(&self, part: &str) -> Option<String> {
        self.chosen.get(part).filter(|c| self.part_of(c).is_ok_and(|p| p == part)).cloned()
    }

    /// Variant `level` of its part goes into the product; for part `part`, none: the root's own.
    pub fn choose(&mut self, level: Option<&str>, part: Option<&str>) -> Result<(), String> {
        match (level, part) {
            (Some(l), _) => {
                let of = self.part_of(l)?;
                self.chosen.insert(of, l.to_string());
            }
            (None, Some(p)) => {
                self.chosen.remove(p);
            }
            (None, None) => return Err("give `level`, or `part` for the root's own".into()),
        }
        Ok(())
    }

    /// The product as it is now (each part's variant, the parts hidden) kept as set `name`.
    pub fn save_set(&mut self, name: &str) -> Result<(), String> {
        if name.trim().is_empty() {
            return Err("a set needs a name".into());
        }
        self.sets.insert(name.to_string(), Set { chosen: self.chosen.clone(), hidden: self.hidden.clone() });
        Ok(())
    }

    /// The product as set `name` has it.
    pub fn apply_set(&mut self, name: &str) -> Result<(), String> {
        let set = self.sets.get(name).ok_or_else(|| format!("no set `{name}`"))?;
        (self.chosen, self.hidden) = (set.chosen.clone(), set.hidden.clone());
        Ok(())
    }

    /// The set the product is as now, if one.
    pub fn current_set(&self) -> Option<&str> {
        self.sets.iter().find(|(_, s)| s.chosen == self.chosen && s.hidden == self.hidden).map(|(n, _)| n.as_str())
    }

    /// Part `part` shown in the product, or not.
    pub fn show(&mut self, part: &str, visible: bool) -> Result<(), String> {
        if !self.parts().iter().any(|p| p == part) {
            return Err(format!("no part `{part}`"));
        }
        if visible {
            self.hidden.remove(part);
        } else {
            self.hidden.insert(part.to_string());
        }
        Ok(())
    }

    /// The levels below `level` (none: the root), however deep.
    pub fn below(&self, level: Option<&str>) -> Vec<String> {
        let Some(level) = level else { return self.levels.keys().cloned().collect() };
        let under = |n: &String| {
            let mut at = n.as_str();
            while let Some(p) = self.levels.get(at).and_then(|l| l.parent.as_deref()) {
                if p == level {
                    return true;
                }
                at = p;
            }
            false
        };
        // A loop of parents ends the walk: `graph` reports it.
        self.levels.keys().filter(|n| n.as_str() != level && self.chain_check(n).is_ok() && under(n)).cloned().collect()
    }

    /// The patches of feature `feature` in levels `levels`, given to `f`: what a rename in its
    /// sketch rewrites too.
    pub fn patches_of(&mut self, levels: &[String], feature: &str, mut f: impl FnMut(&mut Map<String, Value>)) {
        for (_, l) in self.levels.iter_mut().filter(|(n, _)| levels.contains(n)) {
            for p in l.features.iter_mut().filter(|p| p.get("name").and_then(Value::as_str) == Some(feature)) {
                f(p);
            }
        }
    }
}

/// Whether feature `op` makes a body (not a sketch, a plane or a blank).
fn makes_body(op: &Op) -> bool {
    !matches!(op, Op::Sketch { .. } | Op::Plane { .. } | Op::Unfold { .. })
}

/// The features of the body feature `last` of graph `g` ends in, from `last` back to the one
/// that began it: down its first inputs while they make bodies (a cut's body, a boolean's first).
pub(crate) fn body_chain(g: &Graph, last: NodeIx) -> impl Iterator<Item = NodeIx> + '_ {
    let nodes = g.nodes();
    std::iter::successors(Some(last), |&at| nodes[at as usize].inputs.first().copied().filter(|&j| makes_body(&nodes[j as usize].op)))
}

/// The bodies of graph `g`, in the order they were begun: for each final body feature, the
/// feature that began its body (see [`body_chain`]) and itself.
pub fn bodies(g: &Graph) -> Vec<(String, String)> {
    let nodes = g.nodes();
    let mut out: Vec<(NodeIx, NodeIx)> =
        g.finals().into_iter().filter(|&i| makes_body(&nodes[i as usize].op)).map(|i| (body_chain(g, i).last().unwrap_or(i), i)).collect();
    out.sort_unstable();
    out.into_iter().map(|(a, i)| (nodes[a as usize].name.clone(), nodes[i as usize].name.clone())).collect()
}

impl Doc {
    /// A file from before D169, its parts levels right below the root, brought over: each part
    /// level's own features go into the root, its body becomes a part named as the level; what
    /// else it changed (a parameter, a root feature) stays as a level marked with that part, the
    /// variants below it as they were. Two parts with a feature of one name cannot share a root.
    pub fn bring_over(&mut self) -> Result<(), String> {
        let old: Vec<String> = self.levels.iter().filter(|(_, l)| l.parent.is_none() && l.part.is_none()).map(|(n, _)| n.clone()).collect();
        // A part level's own features: those the frame had not.
        let frame = self.root.clone();
        for name in old {
            let level = self.levels[&name].clone();
            let (own, changed): (Vec<_>, Vec<_>) =
                level.features.into_iter().partition(|f| f.get("name").and_then(Value::as_str).is_some_and(|n| frame.find(n).is_none()));
            let before = bodies(&self.root);
            for f in in_order(own)? {
                let n = f.get("name").and_then(Value::as_str).unwrap_or_default().to_string();
                add_feature(&mut self.root, f).map_err(|e| {
                    format!("part `{name}`: its feature `{n}` is named as another part's: rename one of them in the version it was made in ({e})")
                })?;
            }
            // Its body: the one it named, else the new one; more than one, each a part of its own.
            let made: Vec<String> = bodies(&self.root).into_iter().filter(|b| !before.contains(b)).map(|(begun, _)| begun).collect();
            let given: Option<Vec<String>> =
                level.output.as_ref().map(|o| bodies(&self.root).into_iter().filter(|(_, last)| o.contains(last)).map(|(begun, _)| begun).collect());
            let made = given.unwrap_or(made);
            let Some(first) = made.first().cloned() else { return Err(format!("part `{name}` makes no body")) };
            for (k, b) in made.iter().enumerate() {
                let named = if k == 0 { name.clone() } else { format!("{name} {}", k + 1) };
                self.parts.insert(b.clone(), Some(PartInfo { name: Some(named), ..PartInfo::default() }));
            }
            let keep = !changed.is_empty() || !level.parameters.is_empty();
            if keep {
                let l = self.levels.get_mut(&name).ok_or("a level gone")?;
                (l.features, l.part, l.output) = (changed, Some(first.clone()), None);
            } else {
                self.levels.remove(&name);
                for l in self.levels.values_mut().filter(|l| l.parent.as_deref() == Some(name.as_str())) {
                    (l.parent, l.part) = (None, Some(first.clone()));
                }
            }
            match self.chosen.remove(&name) {
                Some(c) if c != name || keep => {
                    self.chosen.insert(first.clone(), c);
                }
                _ if keep => {
                    self.chosen.insert(first.clone(), name.clone());
                }
                _ => {}
            }
            if self.hidden.remove(&name) {
                self.hidden.insert(first);
            }
        }
        Ok(())
    }
}

/// `base` with `level`'s parameters and features: the parent's changed by merging, the level's
/// own after them; then in an order where each comes after its inputs.
fn apply(base: &Graph, level: &Level) -> Result<Graph, String> {
    let mut features: Vec<Map<String, Value>> = (0..base.nodes().len() as NodeIx).map(|i| feature_json(base, i)).collect();
    for p in &level.features {
        let name = p.get("name").and_then(Value::as_str).ok_or("a feature without `name`")?;
        match features.iter_mut().find(|f| f.get("name").and_then(Value::as_str) == Some(name)) {
            Some(f) => {
                let mut v = Value::Object(std::mem::take(f));
                merge(&mut v, &Value::Object(p.clone()));
                *f = match v {
                    Value::Object(m) => m,
                    _ => Map::new(),
                };
            }
            None if p.contains_key("op") => features.push(p.clone()),
            None => return Err(format!("`{name}` changes no feature of its parent and has no `op` of its own")),
        }
    }
    let mut g = Graph::default();
    for (n, v) in base.parameters().iter().chain(&level.parameters) {
        g.set_parameter(n, v.clone()).map_err(|e| e.to_string())?;
    }
    for f in in_order(features)? {
        add_feature(&mut g, f)?;
    }
    Ok(g)
}

/// Features each after its inputs, keeping their order otherwise: a level's own feature that a
/// parent's feature now takes as input goes before that one.
fn in_order(mut todo: Vec<Map<String, Value>>) -> Result<Vec<Map<String, Value>>, String> {
    let inputs = |f: &Map<String, Value>| -> Vec<String> {
        f.get("inputs").and_then(Value::as_array).map(|a| a.iter().filter_map(Value::as_str).map(String::from).collect()).unwrap_or_default()
    };
    let mut out: Vec<Map<String, Value>> = Vec::with_capacity(todo.len());
    let mut placed: Vec<String> = Vec::new();
    while !todo.is_empty() {
        let Some(k) = todo
            .iter()
            .position(|f| inputs(f).iter().all(|i| placed.contains(i) || !todo.iter().any(|t| t.get("name").and_then(Value::as_str) == Some(i))))
        else {
            return Err("features take each other as inputs in a loop".into());
        };
        let f = todo.remove(k);
        placed.push(f.get("name").and_then(Value::as_str).unwrap_or_default().to_string());
        out.push(f);
    }
    Ok(out)
}

/// What `child` differs from `base` in: parameters set otherwise or anew, features changed (as
/// merge patches) or its own.
fn differ(base: &Graph, child: &Graph) -> Result<Level, String> {
    if let Some(n) = base.parameters().keys().find(|n| !child.parameters().contains_key(*n)) {
        return Err(format!("parameter `{n}` comes from the parent: a variant gives it another value, it cannot remove it"));
    }
    let parameters = child.parameters().iter().filter(|(n, v)| base.parameters().get(*n) != Some(*v)).map(|(n, v)| (n.clone(), v.clone())).collect();
    if let Some(n) = base.nodes().iter().find(|n| child.find(&n.name).is_none()) {
        return Err(format!("feature `{}` comes from the parent: a variant changes it, it cannot remove it", n.name));
    }
    let mut features = Vec::new();
    for i in 0..child.nodes().len() as NodeIx {
        let f = feature_json(child, i);
        let name = child.nodes()[i as usize].name.clone();
        match base.find(&name) {
            None => features.push(f),
            Some(j) => {
                if let Some(Value::Object(mut p)) = patch(&Value::Object(feature_json(base, j)), &Value::Object(f)) {
                    p.insert("name".into(), Value::String(name));
                    features.push(p);
                }
            }
        }
    }
    Ok(Level { parameters, features, ..Level::default() })
}

/// JSON merge patch (RFC 7386): objects merged key by key, `null` removes, anything else replaces;
/// but a sketch's `constraints`, a set, take `{"add": [...], "remove": [...]}`, so that a variant's
/// dimensions added to a parent's sketch leave the parent's own to follow the parent.
pub fn merge(target: &mut Value, patch: &Value) {
    merge_of(target, patch, "");
}

/// [`merge`], into the value under key `key`.
fn merge_of(target: &mut Value, patch: &Value, key: &str) {
    match (target, patch) {
        (t @ (Value::Array(_) | Value::Null), Value::Object(p)) if key == "constraints" => {
            let gone = p.get("remove").and_then(Value::as_array).cloned().unwrap_or_default();
            let mut list = match std::mem::take(t) {
                Value::Array(a) => a,
                _ => Vec::new(),
            };
            list.retain(|c| !gone.contains(c));
            list.extend(p.get("add").and_then(Value::as_array).into_iter().flatten().cloned());
            *t = Value::Array(list);
        }
        (Value::Object(t), Value::Object(p)) => {
            for (k, v) in p {
                if v.is_null() {
                    t.remove(k);
                } else {
                    merge_of(t.entry(k.clone()).or_insert(Value::Null), v, k);
                }
            }
        }
        (t, p) => *t = p.clone(),
    }
}

/// The merge patch taking `a` to `b`; none if they are equal.
fn patch(a: &Value, b: &Value) -> Option<Value> {
    patch_of(a, b, "")
}

/// [`patch`], of the value under key `key`.
fn patch_of(a: &Value, b: &Value, key: &str) -> Option<Value> {
    if a == b {
        return None;
    }
    match (a, b) {
        (Value::Array(x), Value::Array(y)) if key == "constraints" => {
            let add: Vec<&Value> = y.iter().filter(|c| !x.contains(c)).collect();
            let remove: Vec<&Value> = x.iter().filter(|c| !y.contains(c)).collect();
            let mut p = Map::new();
            if !add.is_empty() {
                p.insert("add".into(), serde_json::json!(add));
            }
            if !remove.is_empty() {
                p.insert("remove".into(), serde_json::json!(remove));
            }
            // The same set in another order: nothing to keep.
            (!p.is_empty()).then_some(Value::Object(p))
        }
        (Value::Object(x), Value::Object(y)) => {
            let mut p = Map::new();
            for (k, v) in y {
                match x.get(k) {
                    Some(old) => {
                        if let Some(d) = patch_of(old, v, k) {
                            p.insert(k.clone(), d);
                        }
                    }
                    None => {
                        p.insert(k.clone(), v.clone());
                    }
                }
            }
            for k in x.keys().filter(|k| !y.contains_key(*k)) {
                p.insert(k.clone(), Value::Null);
            }
            Some(Value::Object(p))
        }
        _ => Some(b.clone()),
    }
}
