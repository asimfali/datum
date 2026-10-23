//! An `edit_sketch` patch being made: new names clear of the sketch's and of each other. The
//! window builds one per click, a library element placed (D98) builds one; both send it as
//! `edit_sketch`.

use serde_json::{Map, Value, json};

pub struct SketchPatch {
    taken: Vec<String>,
    params: Map<String, Value>,
}

impl SketchPatch {
    /// A patch to sketch feature `feature` (its JSON: `sketch` and `project`).
    pub fn new(feature: &Value) -> Self {
        let sketch = &feature["sketch"];
        let mut taken: Vec<String> =
            ["points", "lines", "arcs", "circles", "measures"].iter().filter_map(|k| sketch[k].as_object()).flat_map(|m| m.keys().cloned()).collect();
        taken.extend(feature["project"].as_object().into_iter().flat_map(|m| m.keys().cloned()));
        Self { taken, params: Map::new() }
    }

    /// `prefix` numbered past every `prefix<k>` taken. A name freed by a removal is not given
    /// again: what referred to the removed element downstream reports it lost rather than
    /// latching onto the new one (D108).
    pub fn fresh(&mut self, prefix: &str) -> String {
        let last = self.taken.iter().filter_map(|n| n.strip_prefix(prefix)?.parse::<u64>().ok()).max().unwrap_or(0);
        let n = format!("{prefix}{}", last + 1);
        self.taken.push(n.clone());
        n
    }

    /// Name `name` given on purpose (a section replaced keeps the name of the one removed, and
    /// with it what referred to it): an error if the sketch has it already.
    pub fn claim(&mut self, name: &str) -> Result<(), String> {
        if name.is_empty() || name.contains('.') || name == datum_ops::ORIGIN || self.taken.iter().any(|t| t == name) {
            return Err(format!("the name `{name}` is taken or not a name"));
        }
        self.taken.push(name.to_string());
        Ok(())
    }

    /// Entry `name` of map `key` (`points`, `lines`, …, `project`) of the patch.
    pub fn set(&mut self, key: &str, name: &str, value: Value) {
        self.params.entry(key).or_insert_with(|| json!({}))[name] = value;
    }

    /// A line, arc or circle, as construction geometry if `construction`.
    pub fn add(&mut self, key: &str, name: &str, mut value: Value, construction: bool) {
        if construction {
            value["construction"] = json!(true);
        }
        self.set(key, name, value);
    }

    pub fn constrain(&mut self, c: Value) {
        if let Value::Array(a) = self.params.entry("add_constraints").or_insert_with(|| json!([])) {
            a.push(c);
        }
    }

    /// The patch, unless it changes nothing.
    pub fn done(self) -> Option<Map<String, Value>> {
        (!self.params.is_empty()).then_some(self.params)
    }
}
