//! The `sheet`, `plate` and `rolled` commands (D185, D194, D195, D207): sheet metal from an open
//! chain of a sketch, a face of it from its closed profiles, or a line of it rolled about an
//! axis, a part of its own made by a sheet rule,
//! whose thickness (and bend radius) it takes (D193).

use datum_graph::Op;
use serde_json::{Map, Value, json};

use super::{Session, text};
use crate::add_feature;

/// The `z0` and `z1` of sheet metal `width` wide (a number or a formula) along the sketch's
/// normal: from the sketch, or half each way if `symmetric`. One place for the session and the
/// window's form.
pub fn sheet_extent(width: &Value, symmetric: bool) -> Result<(Value, Value), String> {
    match (symmetric, width) {
        (false, _) => Ok((json!(0), width.clone())),
        (true, Value::Number(n)) => {
            let h = n.as_f64().ok_or("`z1`: a number or a formula")? / 2.0;
            Ok((json!(-h), json!(h)))
        }
        (true, Value::String(f)) => Ok((json!(format!("-({f}) / 2")), json!(format!("({f}) / 2")))),
        _ => Err("`z1`: a number or a formula".into()),
    }
}

/// The width of sheet metal from `z0` to `z1`, and whether it is symmetric: as [`sheet_extent`]
/// makes them, back. Else the width from `z0` (given as is if `z0` is not 0).
pub fn sheet_width(z0: &Value, z1: &Value) -> (Value, bool) {
    match (z0, z1) {
        (Value::Number(a), Value::Number(b)) if a.as_f64().zip(b.as_f64()).is_some_and(|(a, b)| a == -b && b > 0.0) => {
            (json!(2.0 * b.as_f64().unwrap_or(0.0)), true)
        }
        (Value::String(a), Value::String(b)) => {
            match (a.strip_prefix("-(").and_then(|r| r.strip_suffix(") / 2")), b.strip_prefix('(').and_then(|r| r.strip_suffix(") / 2"))) {
                (Some(f), Some(g)) if f == g => (json!(f), true),
                _ => (z1.clone(), false),
            }
        }
        _ => (z1.clone(), false),
    }
}

impl Session {
    /// Adds the sheet metal the `sheet` command's parameters `p` describe and says what it is:
    /// the part and its rule.
    pub(super) fn sheet(&mut self, p: &Map<String, Value>) -> Result<Value, String> {
        let width = p.get("z1").cloned().ok_or("sheet metal needs `z1`: how far it goes along the sketch's normal")?;
        let (z0, z1) = match p.get("symmetric").and_then(Value::as_bool).unwrap_or(false) {
            false => (p.get("z0").cloned().unwrap_or(json!(0)), width),
            true => sheet_extent(&width, true)?,
        };
        let mut fields: Map<String, Value> = ["path", "start"].iter().filter_map(|&k| Some((k.into(), p.get(k)?.clone()))).collect();
        fields.insert("op".into(), "sheet".into());
        fields.insert("side".into(), p.get("side").cloned().unwrap_or(json!("left")));
        fields.insert("z0".into(), z0);
        fields.insert("z1".into(), z1);
        self.sheet_part(p, fields)
    }

    /// Adds the face of sheet metal the `plate` command's parameters `p` describe, as `sheet`.
    pub(super) fn plate(&mut self, p: &Map<String, Value>) -> Result<Value, String> {
        let mut fields: Map<String, Value> = ["profiles", "flip", "symmetric"].iter().filter_map(|&k| Some((k.into(), p.get(k)?.clone()))).collect();
        fields.insert("op".into(), "plate".into());
        self.sheet_part(p, fields)
    }

    /// Adds the rolled sheet metal the `rolled` command's parameters `p` describe, as `sheet`.
    pub(super) fn rolled(&mut self, p: &Map<String, Value>) -> Result<Value, String> {
        let mut fields: Map<String, Value> = ["axis", "path", "angle", "gap"].iter().filter_map(|&k| Some((k.into(), p.get(k)?.clone()))).collect();
        fields.insert("op".into(), "rolled".into());
        fields.insert("side".into(), p.get("side").cloned().unwrap_or(json!("left")));
        self.sheet_part(p, fields)
    }

    /// Feature `fields` of `p`'s `name` on `p`'s `sketch`, a sheet metal part by `p`'s `rule`.
    fn sheet_part(&mut self, p: &Map<String, Value>, mut fields: Map<String, Value>) -> Result<Value, String> {
        let name = text(p, "name")?;
        let sketch = text(p, "sketch")?;
        if !matches!(self.graph.nodes()[self.find(sketch)? as usize].op, Op::Sketch { .. }) {
            return Err(format!("`{sketch}` is not a sketch"));
        }
        let rule =
            p.get("rule").and_then(Value::as_str).ok_or("sheet metal needs `rule`: a sheet rule of the library, its thickness and bend radius")?;
        fields.insert("name".into(), name.into());
        fields.insert("inputs".into(), json!([sketch]));
        let ix = add_feature(&mut self.graph, fields)?;
        // The rule is of the part's kind, which the document learns from its first feature.
        self.doc.store(self.active.as_deref(), &self.graph)?;
        self.set_rule(name, Some(rule.to_string()))?;
        let mut reply = self.rebuild_recording(ix);
        reply["rule"] = self.doc.rule(name, self.active.as_deref()).into();
        reply["body"] = name.into();
        Ok(reply)
    }
}
