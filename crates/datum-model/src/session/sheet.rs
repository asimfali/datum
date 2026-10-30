//! The `sheet` and `plate` commands (D185, D194, D195): sheet metal from an open chain of a
//! sketch, or a face of it from its closed profiles, a part of its own made by a sheet rule,
//! whose thickness (and bend radius) it takes (D193).

use datum_graph::Op;
use serde_json::{Map, Value, json};

use super::{Session, text};
use crate::add_feature;

impl Session {
    /// Adds the sheet metal the `sheet` command's parameters `p` describe and says what it is:
    /// the part and its rule.
    pub(super) fn sheet(&mut self, p: &Map<String, Value>) -> Result<Value, String> {
        let width = p.get("z1").cloned().ok_or("sheet metal needs `z1`: how far it goes along the sketch's normal")?;
        // Symmetric: half each way.
        let (z0, z1) = match (p.get("symmetric").and_then(Value::as_bool).unwrap_or(false), &width) {
            (false, _) => (p.get("z0").cloned().unwrap_or(json!(0)), width),
            (true, Value::Number(n)) => {
                let h = n.as_f64().ok_or("`z1`: a number or a formula")? / 2.0;
                (json!(-h), json!(h))
            }
            (true, Value::String(f)) => (json!(format!("-({f}) / 2")), json!(format!("({f}) / 2"))),
            _ => return Err("`z1`: a number or a formula".into()),
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
