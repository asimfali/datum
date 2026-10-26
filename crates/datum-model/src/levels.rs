//! Levels of a model (D152): a frame (the root: the file's own `parameters` and `features`), parts
//! below it, their variants below them. A level holds only what differs from its parent:
//! parameters, the parent's features changed (by name, merged as JSON merge patches: a sketch
//! gains holes, a size changes, the rest is inherited) and features of its own. A level is
//! evaluated as an ordinary graph made of all that: inherited features keep their names, so their
//! IDs and cache keys are the parent's, and what the variants share is computed once.

use std::collections::BTreeMap;

use datum_graph::{Graph, NodeIx};
use datum_math::expr::Num;
use serde::{Deserialize, Serialize};
use serde_json::{Map, Value};

use crate::{add_feature, feature_json};

/// A level, as the file has it.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Level {
    /// The level it differs from; none: the root.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub parent: Option<String>,
    /// Parameters it sets: the parent's given other values, or its own.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub parameters: BTreeMap<String, Num>,
    /// The parent's features changed (`name` and the fields that differ; `null` drops a field)
    /// and its own features, whole.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub features: Vec<Map<String, Value>>,
}

/// A whole model: the root's graph and the levels by name.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Doc {
    pub root: Graph,
    pub levels: BTreeMap<String, Level>,
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

    /// A new level `name` below `parent` (none: the root), the same as it to begin with.
    pub fn add_level(&mut self, name: &str, parent: Option<&str>) -> Result<(), String> {
        if name.trim().is_empty() || self.levels.contains_key(name) {
            return Err(format!("a level needs a name of its own: `{name}`"));
        }
        if let Some(p) = parent.filter(|p| !self.levels.contains_key(*p)) {
            return Err(format!("no level `{p}`"));
        }
        self.levels.insert(name.to_string(), Level { parent: parent.map(String::from), ..Level::default() });
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
    Ok(Level { parent: None, parameters, features })
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
