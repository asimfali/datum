//! A pattern made in the window (D155): what is selected first says the kind (a point or a
//! circle: around its centre; one or two lines: along them; a group, or lines and arcs joined
//! into one chain (D165): along it, or filling it if it is closed; nothing: along the sketch's
//! axes); a held right click opens the menu with its
//! numbers; the elements to copy are picked next, the copies previewed as they change. The same
//! menu shifts, leaves out and edits the copies selected. Everything goes as one `edit_sketch`.

use std::collections::BTreeMap;

use serde_json::{Map, Value as Json, json};

use crate::i18n::I18n;
use crate::sketch::{Pick, SketchEdit};

/// What the copies follow.
#[derive(Clone, Debug, PartialEq)]
pub enum Base {
    /// The sketch's axes, or one or two lines (their directions as picked): a grid.
    Grid([[f64; 2]; 2]),
    /// A point (a circle's or arc's centre): around it.
    Around(String),
    /// A group: along it; or, closed, filling it. `new`: the lines and arcs selected, a group
    /// made of them with the pattern (a contour picked as it is).
    Along { group: String, closed: bool, new: Vec<String> },
}

/// The fields of each kind, by key: a number or a formula, empty when not given.
const FIELDS: [&str; 11] = ["n1", "s1", "g1", "n2", "s2", "g2", "angle", "step", "clearance", "n", "g"];

pub struct PatternForm {
    pub base: Base,
    /// Along a closed group: fill it rather than go round it.
    pub fill: bool,
    pub rotate: bool,
    /// The two directions (or the turn, the way along) the other way.
    pub flip: [bool; 2],
    pub fields: BTreeMap<&'static str, String>,
    /// The pattern's name, and the group its cell is (a new one if the elements are picked).
    pub name: String,
    pub of: Option<String>,
    /// Picking the elements to copy: they are the selection.
    pub picking: bool,
    /// The cell's elements, as picked.
    pub cell: Vec<String>,
    /// The sketch as it was: the preview is laid over it, Esc goes back to it.
    pub before: Json,
}

/// `patch` laid over the sketch as it was, solved if that changes what the window has.
fn lay(sk: &mut SketchEdit, before: &Json, patch: Option<&Map<String, Json>>) -> Result<(), String> {
    let mut json = before.clone();
    if let (Some(p), Some(fields)) = (patch, json.as_object_mut()) {
        datum_model::patch_sketch(fields, p)?;
    }
    if sk.json != json {
        let good = std::mem::replace(&mut sk.json, json);
        sk.solve();
        // A preview that does not place (a step of 0, a formula half typed) keeps the picture it
        // had, and says why.
        if let Err(why) = &sk.placed {
            let why = why.clone();
            sk.json = good;
            sk.solve();
            return Err(why);
        }
    }
    Ok(())
}

/// The first selection as what the copies follow, or why it cannot be. Points picked with
/// curves (a frame takes both) are their ends: only the curves count.
pub fn base_of(sk: &SketchEdit) -> Result<Base, String> {
    let sketch = &sk.json["sketch"];
    let picks: Vec<&Pick> = sk.selected.iter().filter(|p| !matches!(p, Pick::Constraint(_) | Pick::Measure(_))).collect();
    if picks.is_empty() {
        return Ok(Base::Grid([[1.0, 0.0], [0.0, 1.0]]));
    }
    let placed = sk.placed.as_ref().map_err(Clone::clone)?;
    if let Some(g) = sk.selected_group() {
        let curves = &sk.groups()[&g];
        let circles = curves.iter().all(|n| placed.circles.contains_key(n));
        return Ok(Base::Along { closed: circles || chain(placed, curves) == Some(true), group: g, new: Vec::new() });
    }
    let curves: Vec<String> = picks.iter().filter_map(|p| if let Pick::Curve(n) = p { Some(n.clone()) } else { None }).collect();
    let at = |n: &str| placed.solved.points.get(n).map(|v| [v.x, v.y]);
    match (&picks[..], &curves[..]) {
        ([Pick::Point(n)], _) => return Ok(Base::Around(n.clone())),
        (_, [n]) if placed.circles.contains_key(n) || placed.arcs.contains_key(n) => {
            let c = if sketch["circles"].get(n).is_some() { &sketch["circles"][n] } else { &sketch["arcs"][n] };
            return Ok(Base::Around(c["center"].as_str().unwrap_or_default().to_string()));
        }
        _ => {}
    }
    let lines: Vec<[f64; 2]> = curves
        .iter()
        .filter_map(|n| {
            let l = placed.lines.get(n)?;
            let (a, b) = (at(&l.from)?, at(&l.to)?);
            let len = libm::hypot(b[0] - a[0], b[1] - a[1]);
            (len > 0.0).then(|| [(b[0] - a[0]) / len, (b[1] - a[1]) / len])
        })
        .collect();
    match (&lines[..], curves.len()) {
        ([d], 1) => return Ok(Base::Grid([*d, [-d[1], d[0]]])),
        ([d, e], 2) => return Ok(Base::Grid([*d, *e])),
        _ => {}
    }
    // Lines and arcs joined into one chain: as a group made of them, or the group they are.
    let why = "Select what the copies follow: a point, a circle, one or two lines, a contour or a group, or nothing for the axes";
    let closed = chain(placed, &curves).ok_or(why)?;
    let same = |held: &Vec<String>| {
        let held: Vec<&String> = held.iter().filter(|n| !placed.solved.points.contains_key(n.as_str())).collect();
        held.len() == curves.len() && curves.iter().all(|c| held.contains(&c))
    };
    if let Some((g, _)) = sk.groups().into_iter().find(|(_, held)| same(held)) {
        return Ok(Base::Along { group: g, closed, new: Vec::new() });
    }
    let group = datum_model::SketchPatch::new(&sk.json).fresh(if closed { "contour" } else { "path" });
    Ok(Base::Along { group, closed, new: curves })
}

/// Lines and arcs `curves`, all of them, as one chain: closed or not; None if they are not one.
fn chain(placed: &datum_ops::Placed, curves: &[String]) -> Option<bool> {
    let c = datum_sketch::contours(&placed.lines, &placed.arcs, &|n, _| curves.iter().any(|m| m == n));
    let all = c.loops.iter().chain(&c.paths).map(Vec::len).sum::<usize>() == curves.len();
    match (&c.loops[..], &c.paths[..]) {
        ([_], []) if all && c.ends.is_empty() && c.branches.is_empty() => Some(true),
        ([], [_]) if all && c.branches.is_empty() => Some(false),
        _ => None,
    }
}

/// A field's text as JSON: a number, a formula, or nothing.
fn num(text: &str) -> Option<Json> {
    let t = text.trim();
    if t.is_empty() {
        return None;
    }
    Some(crate::dims::value_of(t))
}

/// `text` times `k` (a direction's component, or −1): a number, or the formula so multiplied.
fn times(text: &str, k: f64) -> Option<Json> {
    let t = text.trim();
    if t.is_empty() {
        return None;
    }
    Some(match crate::dims::value_of(t).as_f64() {
        Some(v) => json!(v * k),
        None if k == 0.0 => json!(0.0),
        None if k == 1.0 => json!(t),
        None if k == -1.0 => json!(format!("-({t})")),
        None => json!(format!("({t}) * {k}")),
    })
}

/// A JSON number or formula as a field shows it.
fn text(v: &Json) -> String {
    match v {
        Json::Number(n) => {
            n.as_f64().map_or_else(String::new, |x| if x.fract() == 0.0 && x.abs() < 1e15 { format!("{}", x as i64) } else { format!("{x}") })
        }
        Json::String(s) => s.clone(),
        _ => String::new(),
    }
}

impl PatternForm {
    /// A new pattern following `base`, with the counts a first try would want.
    pub fn new(sk: &SketchEdit, base: Base) -> Self {
        let mut patch = datum_model::SketchPatch::new(&sk.json);
        let name = patch.fresh("pattern");
        let mut fields: BTreeMap<&'static str, String> = FIELDS.iter().map(|k| (*k, String::new())).collect();
        let mut set = |k: &'static str, v: &str| {
            fields.insert(k, v.to_string());
        };
        match &base {
            Base::Grid(_) => {
                set("n1", "3");
                set("n2", "1");
            }
            Base::Around(_) => {
                set("n", "6");
                set("angle", "360");
            }
            Base::Along { .. } => set("n", "5"),
        }
        let fill = matches!(base, Base::Along { closed: true, .. });
        Self { base, fill, rotate: true, flip: [false; 2], fields, name, of: None, picking: false, cell: Vec::new(), before: sk.json.clone() }
    }

    /// The menu editing pattern `name` as the sketch has it.
    pub fn edit(sk: &SketchEdit, name: &str) -> Result<Self, String> {
        let p = &sk.json["sketch"]["patterns"][name];
        let base = if let Some(a) = p.get("around") {
            Base::Around(a["center"].as_str().unwrap_or_default().to_string())
        } else if let Some(a) = p.get("along") {
            let group = a["path"].as_str().unwrap_or_default().to_string();
            Base::Along { closed: false, group, new: Vec::new() }
        } else if let Some(f) = p.get("fill").and_then(Json::as_str) {
            Base::Along { closed: true, group: f.to_string(), new: Vec::new() }
        } else {
            Base::Grid([[1.0, 0.0], [0.0, 1.0]])
        };
        let mut form = Self::new(sk, base);
        form.name = name.to_string();
        form.of = p["of"].as_str().map(String::from);
        let mut set = |k: &'static str, v: &Json| {
            form.fields.insert(k, text(v));
        };
        if let Some(a) = p.get("around") {
            set("n", &a["count"]);
            set("angle", a.get("angle").unwrap_or(&Json::Null));
            set("step", a.get("step").unwrap_or(&Json::Null));
            set("g", a.get("grow").unwrap_or(&Json::Null));
            if a.get("step").is_some() {
                form.fields.insert("angle", String::new());
            } else if a.get("angle").is_none() {
                form.fields.insert("angle", "360".into());
            }
        } else if let Some(a) = p.get("along") {
            set("n", a.get("count").unwrap_or(&Json::Null));
            set("s1", a.get("step").unwrap_or(&Json::Null));
            set("g", a.get("grow").unwrap_or(&Json::Null));
            form.rotate = a["rotate"].as_bool().unwrap_or(true);
        } else {
            // A grid's steps back into directions and lengths: along an axis a formula keeps.
            let mut dirs = [[1.0, 0.0], [0.0, 1.0]];
            for i in 0..2 {
                let (x, y) = (&p["step"][i][0], &p["step"][i][1]);
                let (len, d) = match (x.as_f64(), y.as_f64()) {
                    (_, Some(0.0)) => (text(x), [1.0, 0.0]),
                    (Some(0.0), _) => (text(y), [0.0, 1.0]),
                    (Some(a), Some(b)) => {
                        let l = libm::hypot(a, b);
                        (text(&json!(l)), [a / l, b / l])
                    }
                    _ => return Err(format!("pattern `{name}`: a slanted step given by formulas: change it in the model's JSON")),
                };
                dirs[i] = d;
                form.fields.insert(["s1", "s2"][i], len);
                if let Some(c) = p["count"].get(i) {
                    form.fields.insert(["n1", "n2"][i], text(c));
                }
                if let Some(g) = p["grow"].get(i) {
                    let (gx, gy) = (g[0].as_f64().unwrap_or(0.0), g[1].as_f64().unwrap_or(0.0));
                    form.fields.insert(["g1", "g2"][i], text(&json!(gx * d[0] + gy * d[1])));
                }
            }
            if p.get("fill").is_none() {
                form.base = Base::Grid(dirs);
            } else {
                form.fill = true;
            }
            form.fields.insert("clearance", text(p.get("clearance").unwrap_or(&Json::Null)));
        }
        Ok(form)
    }

    /// What the menu shows as the kind.
    pub fn title(&self, i18n: &I18n) -> String {
        match &self.base {
            Base::Grid(_) => i18n.t("pattern-grid", "Pattern on a grid").into(),
            Base::Around(c) => i18n.f("pattern-around", "Pattern around `{ $name }`", &[("name", c)]),
            Base::Along { group, .. } if self.fill => i18n.f("pattern-filling", "Pattern filling `{ $name }`", &[("name", group)]),
            Base::Along { group, .. } => i18n.f("pattern-along", "Pattern along `{ $name }`", &[("name", group)]),
        }
    }

    /// The fields the kind asks, by row: each row's keys, and whether it has a flip (which way).
    pub fn rows<'a>(&self, i18n: &'a I18n) -> Vec<(&'a str, Vec<&'static str>, Option<usize>)> {
        match &self.base {
            Base::Grid(_) => vec![
                (i18n.t("pattern-first", "First direction"), vec!["n1", "s1", "g1"], Some(0)),
                (i18n.t("pattern-second", "Second direction"), vec!["n2", "s2", "g2"], Some(1)),
            ],
            Base::Around(_) => vec![(i18n.t("pattern-row-around", "Around"), vec!["n", "angle", "step", "g"], Some(0))],
            Base::Along { .. } if self.fill => vec![(i18n.t("pattern-steps", "Steps"), vec!["s1", "s2", "clearance"], None)],
            Base::Along { .. } => vec![(i18n.t("pattern-row-along", "Along"), vec!["n", "s1", "g"], Some(0))],
        }
    }

    pub fn label<'a>(i18n: &'a I18n, key: &str) -> &'a str {
        match key {
            "n1" | "n2" | "n" => i18n.t("pattern-count", "Count"),
            "s1" | "s2" => i18n.t("pattern-step", "Step"),
            "g1" | "g2" | "g" => i18n.t("pattern-grows", "Grows by"),
            "angle" => i18n.t("pattern-angle", "Angle, °"),
            "step" => i18n.t("pattern-step-angle", "Step, °"),
            "clearance" => i18n.t("pattern-clearance", "Clearance"),
            _ => "",
        }
    }

    /// The pattern's JSON, as the fields and the base give it.
    fn pattern(&self, of: &str) -> Result<Json, String> {
        let f = |k: &str| self.fields.get(k).map_or("", |s| s.as_str());
        let sign = |i: usize| if self.flip[i] { -1.0 } else { 1.0 };
        let mut p = json!({ "of": of });
        let grid = match &self.base {
            Base::Grid(d) => Some(*d),
            Base::Along { .. } if self.fill => Some([[1.0, 0.0], [0.0, 1.0]]),
            _ => None,
        };
        match &self.base {
            _ if grid.is_some() => {
                let d = grid.unwrap_or_default();
                let step = |i: usize, key: &str| -> Result<Json, String> {
                    let k = sign(i);
                    match (times(f(key), d[i][0] * k), times(f(key), d[i][1] * k)) {
                        (Some(x), Some(y)) => Ok(json!([x, y])),
                        _ => Err("Give each step".into()),
                    }
                };
                p["step"] = json!([step(0, "s1")?, step(1, "s2")?]);
                if let Base::Along { group, .. } = &self.base {
                    p["fill"] = json!(group);
                    if let Some(c) = num(f("clearance")) {
                        p["clearance"] = c;
                    }
                } else {
                    let n = |k: &str| num(f(k)).ok_or_else(|| "Give each count".to_string());
                    p["count"] = json!([n("n1")?, n("n2")?]);
                    if num(f("g1")).is_some() || num(f("g2")).is_some() {
                        let g = |i: usize, key: &str| {
                            let k = sign(i);
                            json!([times(f(key), d[i][0] * k).unwrap_or(json!(0)), times(f(key), d[i][1] * k).unwrap_or(json!(0))])
                        };
                        p["grow"] = json!([g(0, "g1"), g(1, "g2")]);
                    }
                }
            }
            Base::Around(c) => {
                let mut a = json!({"center": c, "count": num(f("n")).ok_or("Give the count")?});
                match (times(f("step"), sign(0)), times(f("angle"), sign(0))) {
                    (Some(s), _) => a["step"] = s,
                    (None, Some(x)) => a["angle"] = x,
                    (None, None) => {}
                }
                if let Some(g) = times(f("g"), sign(0)) {
                    a["grow"] = g;
                }
                p["around"] = a;
            }
            Base::Along { group, .. } => {
                let mut a = json!({ "path": group });
                if let Some(n) = num(f("n")) {
                    a["count"] = n;
                }
                if let Some(s) = times(f("s1"), sign(0)) {
                    a["step"] = s;
                }
                if a.get("count").is_none() && a.get("step").is_none() {
                    return Err("Give the count, the step or both".into());
                }
                if let Some(g) = num(f("g")) {
                    a["grow"] = g;
                }
                if !self.rotate {
                    a["rotate"] = json!(false);
                }
                p["along"] = a;
            }
            Base::Grid(_) => {}
        }
        Ok(p)
    }

    /// The `edit_sketch` patch the menu makes: the cell's group if the elements were picked, and
    /// the pattern. None until there is something to copy.
    pub fn patch(&self) -> Result<Option<Map<String, Json>>, String> {
        let mut patch = datum_model::SketchPatch::new(&self.before);
        if let Base::Along { group, new, .. } = &self.base
            && !new.is_empty()
        {
            patch.claim(group)?;
            patch.set("groups", group, json!(new));
        }
        let of = match (&self.cell[..], &self.of) {
            ([], None) => return Ok(None),
            ([], Some(of)) => of.clone(),
            ([one], _) if self.before["sketch"]["points"].get(one).is_none() => one.clone(),
            (cell, of) => {
                // The cell's group: the pattern's own if it has one, else a new one.
                let group = match of {
                    Some(g) if self.before["sketch"]["groups"].get(g).is_some() => g.clone(),
                    _ => patch.fresh("cell"),
                };
                patch.set("groups", &group, json!(cell));
                group
            }
        };
        patch.set("patterns", &self.name, self.pattern(&of)?);
        Ok(patch.done())
    }

    /// Steps not given yet: twice the cell's size, once it is picked.
    pub fn suggest_steps(&mut self, sk: &SketchEdit) {
        let Ok(p) = &sk.placed else { return };
        let pts: Vec<[f64; 2]> = self.cell.iter().filter_map(|n| SketchEdit::curve_points_of(p, n)).flatten().collect();
        if pts.is_empty() {
            return;
        }
        let lo = pts.iter().fold([f64::INFINITY; 2], |m, v| [m[0].min(v[0]), m[1].min(v[1])]);
        let hi = pts.iter().fold([f64::NEG_INFINITY; 2], |m, v| [m[0].max(v[0]), m[1].max(v[1])]);
        let size = (hi[0] - lo[0]).max(hi[1] - lo[1]);
        if size <= 0.0 {
            return;
        }
        // Two significant figures: a step one would type.
        let step = 2.0 * size;
        let k = 10f64.powi(libm::floor(libm::log10(step)) as i32 - 1);
        let step = text(&json!(libm::round(step / k) * k));
        let keys: &[&'static str] = match &self.base {
            Base::Grid(_) => &["s1", "s2"],
            Base::Along { .. } if self.fill => &["s1", "s2"],
            _ => &[],
        };
        for k in keys {
            if self.fields[k].trim().is_empty() {
                self.fields.insert(k, step.clone());
            }
        }
    }

    /// The elements picked, from the selection.
    pub fn take_cell(&mut self, sk: &SketchEdit) {
        self.cell = sk
            .selected
            .iter()
            .filter_map(|p| match p {
                Pick::Point(n) if n != datum_ops::ORIGIN => Some(n.clone()),
                Pick::Curve(n) => Some(n.clone()),
                _ => None,
            })
            .collect();
    }

    /// Lays the patch over the sketch as it was: the copies seen at once.
    pub fn preview(&self, sk: &mut SketchEdit) -> Result<(), String> {
        lay(sk, &self.before, self.patch()?.as_ref())
    }

    /// Back to the sketch as it was.
    pub fn cancel(&self, sk: &mut SketchEdit) {
        let _ = lay(sk, &self.before, None);
    }
}

/// The menu on copies selected: shift them, leave them out, edit or remove their pattern.
pub struct CopiesForm {
    pub copies: Vec<(String, [i64; 2])>,
    pub shift: [String; 2],
    pub before: Json,
}

impl CopiesForm {
    pub fn new(sk: &SketchEdit) -> Option<Self> {
        let copies: Vec<(String, [i64; 2])> =
            sk.selected.iter().filter_map(|p| if let Pick::Copy(n, k) = p { Some((n.clone(), *k)) } else { None }).collect();
        if copies.is_empty() || copies.len() != sk.selected.len() {
            return None;
        }
        // The shift the first copy has, to change it.
        let (n, k) = &copies[0];
        let had = &sk.json["sketch"]["patterns"][n]["shift"][format!("{},{}", k[0], k[1])];
        let shift = [text(&had[0]), text(&had[1])];
        Some(Self { copies, shift, before: sk.json.clone() })
    }

    /// The axes a shift is given in, by the first copy's pattern.
    pub fn axes<'a>(&self, sk: &SketchEdit, i18n: &'a I18n) -> [&'a str; 2] {
        let p = &sk.json["sketch"]["patterns"][&self.copies[0].0];
        if p.get("along").is_some() {
            [i18n.t("pattern-row-along", "Along"), i18n.t("pattern-across", "Across")]
        } else if p.get("around").is_some() {
            [i18n.t("pattern-degrees", "Degrees"), i18n.t("pattern-outward", "Outward")]
        } else {
            ["X", "Y"]
        }
    }

    /// The single pattern the copies are of, if one.
    pub fn pattern(&self) -> Option<&str> {
        let n = &self.copies[0].0;
        self.copies.iter().all(|(m, _)| m == n).then_some(n.as_str())
    }

    fn patch(&self, sk: &SketchEdit, change: impl FnMut(&mut Json, [i64; 2])) -> Map<String, Json> {
        let mut m = Map::new();
        m.insert("patterns".into(), Json::Object(sk.patterns_changed(&self.copies, change).into_iter().collect()));
        m
    }

    /// Each copy moved by the shift typed (both empty: back where the pattern puts it).
    pub fn shifted(&self, sk: &SketchEdit) -> Map<String, Json> {
        let by = [num(&self.shift[0]).unwrap_or(json!(0)), num(&self.shift[1]).unwrap_or(json!(0))];
        let zero = num(&self.shift[0]).is_none() && num(&self.shift[1]).is_none();
        self.patch(sk, |p, k| {
            let key = format!("{},{}", k[0], k[1]);
            if !p["shift"].is_object() {
                p["shift"] = json!({});
            }
            if let Some(m) = p["shift"].as_object_mut() {
                if zero {
                    m.remove(&key);
                } else {
                    m.insert(key, json!(by));
                }
                if m.is_empty() {
                    p.as_object_mut().map(|o| o.remove("shift"));
                }
            }
        })
    }

    /// The copies left out.
    pub fn removed(&self, sk: &SketchEdit) -> Map<String, Json> {
        self.patch(sk, |p, k| match p["skip"].as_array_mut() {
            Some(s) => s.push(json!(k)),
            None => p["skip"] = json!([k]),
        })
    }

    /// Lays `patch` over the sketch as it was.
    pub fn preview(&self, sk: &mut SketchEdit, patch: &Map<String, Json>) {
        let _ = lay(sk, &self.before, Some(patch));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn edit(sketch: Json) -> SketchEdit {
        let id = datum_brep::TopoId::derive("s").finish();
        SketchEdit::new(
            "s".into(),
            crate::sketch::Context { id, input: None },
            json!({"name": "s", "op": "sketch", "sketch": sketch}),
            BTreeMap::new(),
            None,
        )
    }

    /// A plate 100 × 60, a hole Ø4 at (10, 10), a point o and a construction line from (0, 70).
    fn plate() -> SketchEdit {
        edit(json!({
            "points": {"a": [0, 0], "b": [100, 0], "c": [100, 60], "d": [0, 60], "h": [10, 10], "o": [50, 30], "p": [0, 70], "q": [100, 70]},
            "lines": {"bottom": {"from": "a", "to": "b"}, "right": {"from": "b", "to": "c"}, "top": {"from": "c", "to": "d"}, "left": {"from": "d", "to": "a"},
                      "rail": {"from": "p", "to": "q", "construction": true}},
            "circles": {"hole": {"center": "h", "diameter": 4}},
            "groups": {"panel": ["bottom", "right", "top", "left"], "guide": ["rail"]}
        }))
    }

    fn select(sk: &mut SketchEdit, picks: Vec<Pick>) {
        sk.selected = picks;
    }

    /// What is selected first says the kind.
    #[test]
    fn the_first_selection_says_the_kind() {
        let mut sk = plate();
        assert_eq!(base_of(&sk), Ok(Base::Grid([[1.0, 0.0], [0.0, 1.0]])));
        select(&mut sk, vec![Pick::Point("o".into())]);
        assert_eq!(base_of(&sk), Ok(Base::Around("o".into())));
        select(&mut sk, vec![Pick::Curve("hole".into())]);
        assert_eq!(base_of(&sk), Ok(Base::Around("h".into())), "a circle: its centre");
        select(&mut sk, vec![Pick::Curve("right".into())]);
        assert_eq!(base_of(&sk), Ok(Base::Grid([[0.0, 1.0], [-1.0, 0.0]])));
        select(&mut sk, vec![Pick::Curve("bottom".into()), Pick::Curve("right".into())]);
        assert_eq!(base_of(&sk), Ok(Base::Grid([[1.0, 0.0], [0.0, 1.0]])));
        let g = sk.group_picks("guide");
        select(&mut sk, g);
        assert_eq!(base_of(&sk), Ok(Base::Along { group: "guide".into(), closed: false, new: Vec::new() }));
        let g = sk.group_picks("panel");
        select(&mut sk, g);
        assert_eq!(base_of(&sk), Ok(Base::Along { group: "panel".into(), closed: true, new: Vec::new() }));
        select(&mut sk, vec![Pick::Point("o".into()), Pick::Point("h".into())]);
        assert!(base_of(&sk).is_err());
        // A contour as it is, points framed with it: the group it is, or one made with the pattern.
        let sides = ["bottom", "right", "top", "left"].map(|n| Pick::Curve(n.into()));
        select(&mut sk, [&sides[..], &[Pick::Point("a".into())]].concat());
        assert_eq!(base_of(&sk), Ok(Base::Along { group: "panel".into(), closed: true, new: Vec::new() }));
        sk.json["sketch"]["groups"] = json!({});
        let new: Vec<String> = ["bottom", "right", "top", "left"].map(String::from).into();
        assert_eq!(base_of(&sk), Ok(Base::Along { group: "contour1".into(), closed: true, new: new.clone() }));
        select(&mut sk, sides[..3].to_vec());
        assert_eq!(base_of(&sk), Ok(Base::Along { group: "path1".into(), closed: false, new: new[..3].to_vec() }));
        select(&mut sk, vec![sides[0].clone(), sides[2].clone(), Pick::Curve("rail".into())]);
        assert!(base_of(&sk).is_err(), "not one chain");
        // Filling it: the group goes with the pattern.
        select(&mut sk, sides.to_vec());
        let mut form = PatternForm::new(&sk, base_of(&sk).unwrap());
        form.cell = vec!["hole".into()];
        form.fields.insert("s1", "10".into());
        form.fields.insert("s2", "10".into());
        let patch = Json::Object(form.patch().unwrap().unwrap());
        assert_eq!(patch["groups"]["contour1"], json!(new));
        assert_eq!(patch["patterns"]["pattern1"]["fill"], json!("contour1"));
        form.preview(&mut sk).unwrap();
        assert!(sk.placed.as_ref().unwrap().patterns[0].at.len() > 1);
    }

    /// A grid along the axes: the hole picked becomes a group, the copies seen before Done,
    /// and Esc puts the sketch back.
    #[test]
    fn a_grid_is_previewed_then_sent_or_dropped() {
        let mut sk = plate();
        let mut form = PatternForm::new(&sk, base_of(&sk).unwrap());
        assert_eq!(form.patch(), Ok(None), "nothing to copy yet");
        select(&mut sk, vec![Pick::Curve("hole".into()), Pick::Point("h".into())]);
        form.take_cell(&sk);
        form.suggest_steps(&sk);
        assert_eq!(form.fields["s1"], "8");
        form.fields.insert("n2", "2".into());
        form.flip[1] = false;
        form.preview(&mut sk).unwrap();
        let p = sk.placed.as_ref().unwrap();
        assert_eq!(p.patterns[0].at.len(), 6);
        let patch = Json::Object(form.patch().unwrap().unwrap());
        assert_eq!(patch["groups"]["cell1"], json!(["hole", "h"]));
        assert_eq!(patch["patterns"]["pattern1"], json!({"of": "cell1", "step": [[8.0, 0.0], [0.0, 8.0]], "count": [3.0, 2.0]}));
        form.cancel(&mut sk);
        assert!(sk.placed.as_ref().unwrap().patterns.is_empty());
    }

    /// Around a point, the other way; along a group, a formula for the step; filling a closed
    /// group; a slanted line's grid.
    #[test]
    fn each_kind_writes_its_pattern() {
        let mut sk = plate();
        let form = |base: Base, set: &[(&'static str, &str)], flip: bool| {
            let mut f = PatternForm::new(&sk, base);
            f.cell = vec!["hole".into()];
            for (k, v) in set {
                f.fields.insert(k, v.to_string());
            }
            f.flip[0] = flip;
            Json::Object(f.patch().unwrap().unwrap())["patterns"]["pattern1"].clone()
        };
        assert_eq!(
            form(Base::Around("o".into()), &[("angle", "90"), ("n", "3")], true),
            json!({"of": "hole", "around": {"center": "o", "count": 3.0, "angle": -90.0}})
        );
        assert_eq!(
            form(Base::Along { group: "guide".into(), closed: false, new: Vec::new() }, &[("s1", "w / 4")], true),
            json!({"of": "hole", "along": {"path": "guide", "count": 5.0, "step": "-(w / 4)"}})
        );
        assert_eq!(
            form(Base::Along { group: "panel".into(), closed: true, new: Vec::new() }, &[("s1", "10"), ("s2", "12"), ("clearance", "3")], false),
            json!({"of": "hole", "step": [[10.0, 0.0], [0.0, 12.0]], "fill": "panel", "clearance": 3.0})
        );
        let r = std::f64::consts::FRAC_1_SQRT_2;
        assert_eq!(form(Base::Grid([[r, r], [-r, r]]), &[("s1", "10"), ("s2", "10")], false)["step"][0], json!([10.0 * r, 10.0 * r]));
        // Edited, a pattern comes back as it was written.
        let p = form(Base::Around("o".into()), &[("step", "30"), ("n", "4"), ("g", "5")], false);
        sk.json["sketch"]["patterns"]["ring"] = p.clone();
        let e = PatternForm::edit(&sk, "ring").unwrap();
        assert_eq!((e.fields["n"].as_str(), e.fields["step"].as_str(), e.fields["angle"].as_str(), e.fields["g"].as_str()), ("4", "30", "", "5"));
        assert_eq!(e.of.as_deref(), Some("hole"));
    }

    /// Copies selected: shifted, back, left out; Del leaves them out too.
    #[test]
    fn copies_are_shifted_and_left_out() {
        let mut sk = plate();
        sk.json["sketch"]["patterns"] = json!({"row": {"of": "hole", "count": [3, 1], "step": [[10, 0], [0, 10]]}});
        sk.solve();
        sk.selected = vec![Pick::Copy("row".into(), [1, 0]), Pick::Copy("row".into(), [2, 0])];
        let mut f = CopiesForm::new(&sk).unwrap();
        assert_eq!(f.axes(&sk, &I18n::default()), ["X", "Y"]);
        f.shift = ["".into(), "2".into()];
        let p = f.shifted(&sk);
        assert_eq!(p["patterns"]["row"]["shift"], json!({"1,0": [0, 2.0], "2,0": [0, 2.0]}));
        f.preview(&mut sk, &p);
        assert_eq!(sk.placed.as_ref().unwrap().patterns[0].at[2].1.by.y, 2.0);
        f.shift = [String::new(), String::new()];
        assert!(f.shifted(&sk)["patterns"]["row"].get("shift").is_none(), "an empty shift puts them back");
        assert_eq!(f.removed(&sk)["patterns"]["row"]["skip"], json!([[1, 0], [2, 0]]));
        assert_eq!(sk.delete().unwrap()["patterns"]["row"]["skip"], json!([[1, 0], [2, 0]]));
    }

    /// A copy is hit near its outline, not the cell.
    #[test]
    fn a_copy_is_hit() {
        let mut sk = plate();
        sk.json["sketch"]["patterns"] = json!({"row": {"of": "hole", "count": [3, 1], "step": [[10, 0], [0, 10]]}});
        sk.solve();
        let h = 600.0;
        let (x, y) = sk.camera.offset([22.0, 10.0], h);
        assert_eq!(sk.copy_at(x, y, h), Some(Pick::Copy("row".into(), [1, 0])));
        let (x, y) = sk.camera.offset([12.0, 10.0], h);
        assert_eq!(sk.copy_at(x, y, h), None, "the cell is the hole itself");
    }

    /// The measure tool takes a copy as the circle it is, there; inside the plate, the plate
    /// with its holes, copies too (D177).
    #[test]
    fn measuring_reaches_copies_and_regions() {
        use crate::measure::{Quantity, Shape, Taken, readings, take};
        let mut sk = plate();
        sk.json["sketch"]["patterns"] = json!({"row": {"of": "hole", "count": [3, 1], "step": [[10, 0], [0, 10]]}});
        sk.solve();
        let h = 600.0;
        let at = |q: [f64; 2]| sk.camera.offset(q, h);
        let mut m = Vec::new();
        let (x, y) = at([12.0, 10.0]);
        take(&mut m, sk.measure_at(x, y, h), false);
        let (x, y) = at([22.0, 10.0]);
        take(&mut m, sk.measure_at(x, y, h), false);
        assert!(matches!(m[1], Taken::Shape(Shape::Circle { center: [20.0, 10.0], radius: 2.0 })));
        assert_eq!(readings(&m), [(Quantity::Centres, 10.0), (Quantity::Gap, 6.0)]);
        let (x, y) = at([50.0, 50.0]);
        take(&mut m, sk.measure_at(x, y, h), false);
        let r = readings(&m);
        assert!((r[0].1 - (6000.0 - 12.0 * std::f64::consts::PI)).abs() < 1e-9, "{r:?}");
        assert_eq!(r[1..], [(Quantity::Perimeter, 320.0), (Quantity::Holes, 3.0)]);
        let (x, y) = at([10.0, 10.0]);
        assert!(sk.measure_at(x, y, h).is_some_and(|t| matches!(t, Taken::Shape(Shape::Point([10.0, 10.0])))), "the hole's centre");
        let (x, y) = at([150.0, 30.0]);
        assert!(sk.measure_at(x, y, h).is_none(), "outside everything");
    }
}
