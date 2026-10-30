//! The `extrude` command (D185, D188): an extrusion of a sketch added with its target body found
//! the way the window finds it, and the part's rule if given (D189). Its mode is given, never
//! guessed: the agent sees no preview.

use datum_graph::{NodeIx, Op};
use datum_ops::Mode;
use serde_json::{Map, Value, json};

use super::{Session, text};
use crate::add_feature;
use crate::levels::{bodies, body_chain};

/// Said when an extrusion is given without its mode.
pub(super) const MODE_REQUIRED: &str = "an extrusion needs `mode`: `new` (a body of its own), `join`, `cut` or `intersect` (with a target \
                                        body)";

impl Session {
    /// Adds the extrusion the `extrude` command's parameters `p` describe and says what it is:
    /// its mode, the body it began or changed, and that part's rule.
    pub(super) fn extrude(&mut self, p: &Map<String, Value>) -> Result<Value, String> {
        let name = text(p, "name")?;
        let mode: Mode = serde_json::from_value(p.get("mode").cloned().ok_or(MODE_REQUIRED)?).map_err(|e| format!("`mode`: {e}"))?;
        let sketch = text(p, "sketch")?;
        if !matches!(self.graph.nodes()[self.find(sketch)? as usize].op, Op::Sketch { .. }) {
            return Err(format!("`{sketch}` is not a sketch"));
        }
        let plane = p.get("plane").map(|_| text(p, "plane")).transpose()?;
        let mut fields: Map<String, Value> =
            ["z0", "z1", "symmetric", "through"].iter().filter_map(|&k| Some((k.into(), p.get(k)?.clone()))).collect();
        fields.insert("op".into(), "extrude".into());
        fields.insert("mode".into(), serde_json::to_value(mode).map_err(|e| e.to_string())?);
        let target = match mode {
            Mode::New => None,
            _ => Some(self.target(name, mode, sketch, plane, &fields, p.get("body").map(|_| text(p, "body")).transpose()?)?),
        };
        let inputs: Vec<&str> = target.iter().map(|(_, last)| last.as_str()).chain([sketch]).chain(plane).collect();
        fields.insert("name".into(), name.into());
        fields.insert("inputs".into(), json!(inputs));
        let ix = add_feature(&mut self.graph, fields)?;
        let body = target.map_or(name.into(), |(begun, _)| begun);
        if let Some(r) = p.get("rule") {
            self.set_rule(&body, r.as_str().map(String::from))?;
        }
        let mut reply = self.rebuild_recording(ix);
        reply["mode"] = mode.name().into();
        reply["rule"] = self.doc.rule(&body, self.active.as_deref()).into();
        reply["body"] = body.into();
        Ok(reply)
    }

    /// The body an extrusion `name` in `mode` of `sketch` (up to `plane`, with `fields`) changes,
    /// as the feature that began it and its last one: the body named (`body`, either of the two),
    /// else the one whose face the sketch lies on, else the one the extrusion meets.
    fn target(
        &mut self,
        name: &str,
        mode: Mode,
        sketch: &str,
        plane: Option<&str>,
        fields: &Map<String, Value>,
        body: Option<&str>,
    ) -> Result<(String, String), String> {
        let all = bodies(&self.graph);
        let list = || all.iter().map(|(begun, _)| format!("`{begun}`")).collect::<Vec<_>>().join(", ");
        let one = |found: Vec<&(String, String)>, none: String| match found[..] {
            [b] => Ok(b.clone()),
            [] => Err(none),
            _ => Err(format!(
                "the extrusion could change bodies {}: give `body`",
                found.iter().map(|(begun, _)| format!("`{begun}`")).collect::<Vec<_>>().join(", ")
            )),
        };
        if let Some(b) = body {
            return one(all.iter().filter(|(begun, last)| begun == b || last == b).collect(), format!("no body `{b}`; the bodies are {}", list()));
        }
        // On a face: the body the face is of.
        let s = self.find(sketch)?;
        let node = &self.graph.nodes()[s as usize];
        if let (Op::Sketch { face: Some(_), .. }, Some(&on)) = (&node.op, node.inputs.first()) {
            let holding: Vec<&(String, String)> =
                all.iter().filter(|(_, last)| self.graph.find(last).is_some_and(|l| body_chain(&self.graph, l).any(|i| i == on))).collect();
            if !holding.is_empty() {
                return one(holding, String::new());
            }
        }
        // Else the bodies whose boxes the extrusion's, made as a body of its own, overlaps (not
        // only touches): the Boolean operation then finds what it really meets.
        let mut g = self.graph.clone();
        let mut tool = fields.clone();
        tool.insert("name".into(), name.into());
        tool.insert("mode".into(), "new".into());
        tool.insert("inputs".into(), json!([sketch].into_iter().chain(plane).collect::<Vec<_>>()));
        let ix = add_feature(&mut g, tool)?;
        let e = self.evaluator.evaluate(&g);
        let (lo, hi) = match e.results[ix as usize].as_deref() {
            Ok(v) => v.body().ok_or("the extrusion makes no body")?.bounds(),
            Err(why) => return Err(format!("the extrusion fails: {why}")),
        };
        let bounds = |last: NodeIx| self.last.results[last as usize].as_deref().ok().and_then(|v| v.body()).map(|b| b.bounds());
        let meets = |(a, b): (datum_math::Vec3, datum_math::Vec3)| a.x < hi.x && lo.x < b.x && a.y < hi.y && lo.y < b.y && a.z < hi.z && lo.z < b.z;
        let met = all.iter().filter(|(_, last)| self.graph.find(last).and_then(bounds).is_some_and(meets)).collect();
        let verb = match mode {
            Mode::Cut => "cut",
            Mode::Intersect => "intersect with",
            _ => "join",
        };
        one(met, format!("the extrusion meets no body: nothing to {verb}; for a body of its own, mode `new`"))
    }
}
