//! What a feature being edited stands on: drawn over the picture, so that its form says why it
//! came out as it did, and what of it Ctrl or Shift and a click adds or takes away.

use datum_brep::{Body, TopoId};
use datum_graph::{Evaluation, Graph, Op};
use datum_math::Vec3;
use datum_ops::Placed;

use crate::scene::{MeshCache, chords};
use crate::sketch::SketchEdit;

/// A closed profile of the sketch a feature is made of.
#[derive(Clone, Debug, PartialEq)]
pub struct Profile {
    pub curves: Vec<String>,
    pub lines: Vec<[Vec3; 2]>,
    pub used: bool,
}

/// What feature it was asked for stands on.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Uses {
    /// The features it is made of, in order.
    pub inputs: Vec<String>,
    /// What it uses and a click does not change: an axis, a path, a body it meets.
    pub lines: Vec<[Vec3; 2]>,
    /// Its work planes.
    pub planes: Vec<String>,
    /// A flange's edges: all it may stand on, those it stands on among them.
    pub edges: Vec<([Vec3; 2], TopoId)>,
    pub used_edges: Vec<TopoId>,
    /// The profiles of its sketch, if it picks among them.
    pub profiles: Vec<Profile>,
}

/// What a click with Shift or Ctrl changes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Toggle {
    Edge(TopoId),
    Profile(usize),
}

/// The lines of curves `names` of sketch `s`.
fn curve_lines<'a>(s: &Placed, names: impl Iterator<Item = &'a String>) -> Vec<[Vec3; 2]> {
    let mut out = Vec::new();
    for pts in names.filter_map(|n| SketchEdit::curve_points(s, n)) {
        chords(s, pts, &mut out);
    }
    out
}

/// The curves of `s` that are not construction.
fn drawn_curves(s: &Placed) -> Vec<String> {
    let lines = s.lines.iter().filter(|(_, l)| !l.construction).map(|(n, _)| n);
    let arcs = s.arcs.iter().filter(|(_, a)| !a.construction).map(|(n, _)| n);
    let circles = s.circles.iter().filter(|(_, c)| !c.construction).map(|(n, _)| n);
    lines.chain(arcs).chain(circles).cloned().collect()
}

/// Profiles of sketch `s` as feature `id` with `names` picks them; none if it has no closed one.
fn profiles_of(id: TopoId, s: &Placed, names: &[String]) -> Vec<Profile> {
    let (Ok(all), Ok(kept)) = (datum_ops::profile_curves(id, s), datum_ops::chosen(id, s, names)) else { return Vec::new() };
    let has = |n: &String| kept.lines.contains_key(n) || kept.arcs.contains_key(n) || kept.circles.contains_key(n);
    all.into_iter().map(|curves| Profile { used: curves.iter().all(has), lines: curve_lines(s, curves.iter()), curves }).collect()
}

/// The ends of the straight edge `id` of `b`.
fn edge_ends(b: &Body, id: TopoId) -> Option<[Vec3; 2]> {
    let e = b.edges().binary_search_by_key(&id, |e| e.id).ok()?;
    let h = b.edges()[e].half;
    Some([b.point(b.half_edges()[h as usize].origin), b.point(b.half_edges()[b.next(h) as usize].origin)])
}

/// What feature `name` stands on as the model is now; none if there is no such feature.
pub fn uses(graph: &Graph, evaluation: &Evaluation, name: &str, cache: &mut MeshCache) -> Option<Uses> {
    let ix = graph.find(name)? as usize;
    let node = &graph.nodes()[ix];
    let input = |k: usize| node.inputs.get(k).and_then(|&i| evaluation.results[i as usize].as_ref().ok());
    let mut u = Uses { inputs: node.inputs.iter().map(|&i| graph.nodes()[i as usize].name.clone()).collect(), ..Uses::default() };
    let picks: Option<&[String]> = match &node.op {
        Op::Extrude { profiles, .. } | Op::Plate { profiles, .. } | Op::Revolve { profiles, .. } => Some(profiles),
        _ => None,
    };
    for (k, &i) in node.inputs.iter().enumerate() {
        let Ok(v) = evaluation.results[i as usize].as_ref() else { continue };
        if let Some(s) = v.sketch() {
            match picks {
                Some(names) => u.profiles = profiles_of(node.id, s, names),
                None => u.lines.extend(curve_lines(s, drawn_curves(s).iter())),
            }
        } else if v.plane().is_some() {
            u.planes.push(graph.nodes()[i as usize].name.clone());
        } else if let Some(b) = v.body().filter(|_| k > 0) {
            // The body it changes is the one on the screen; another (a main pipe) it meets.
            u.lines.extend(cache.get(b).edges.iter().copied());
        }
    }
    match &node.op {
        Op::Flange { edges, .. } => {
            let b = input(0).and_then(|v| v.body())?;
            u.edges = datum_ops::flange_edges(b).into_iter().map(|(id, ends)| (ends, id)).collect();
            u.used_edges = edges.iter().filter_map(|e| u128::from_str_radix(e, 16).ok().map(TopoId)).collect();
            for &id in &u.used_edges {
                if !u.edges.iter().any(|e| e.1 == id)
                    && let Some(ends) = edge_ends(b, id)
                {
                    u.edges.push((ends, id));
                }
            }
        }
        Op::Revolve { axis, .. } | Op::Rolled { axis, .. } => {
            if let Some(s) = node.inputs.iter().find_map(|&i| evaluation.results[i as usize].as_ref().ok()?.sketch()) {
                u.lines.extend(curve_lines(s, std::iter::once(axis)));
            }
        }
        _ => {}
    }
    Some(u)
}

impl Uses {
    /// What `lines_at` (a line's distance from the cursor on the screen, if it is on the screen)
    /// finds within `reach` of the cursor: the nearest edge or profile.
    pub fn nearest(&self, at: impl Fn([Vec3; 2]) -> Option<f32>, reach: f32) -> Option<Toggle> {
        let edges = self.edges.iter().filter_map(|(ends, id)| Some((Toggle::Edge(*id), at(*ends)?)));
        let profiles =
            self.profiles.iter().enumerate().flat_map(|(k, p)| p.lines.iter().filter_map(|l| at(*l)).map(move |d| (Toggle::Profile(k), d)));
        edges.chain(profiles).filter(|(_, d)| *d < reach).min_by(|a, b| a.1.total_cmp(&b.1)).map(|(t, _)| t)
    }

    /// The `profiles` of the feature with profile `k` taken in or out, one curve for each profile
    /// used: none for all of them, as the feature says it. None if `k` is the last one used: a
    /// feature stands on something, and no names would mean all.
    pub fn toggled_profile(&self, k: usize) -> Option<Vec<String>> {
        let used = |j: usize| self.profiles[j].used != (j == k);
        let all = 0..self.profiles.len();
        if all.clone().all(used) {
            return Some(Vec::new());
        }
        let names: Vec<String> = all.filter(|&j| used(j)).filter_map(|j| self.profiles[j].curves.first().cloned()).collect();
        (!names.is_empty()).then_some(names)
    }
}

/// `edges` with `id` taken in or out; none if it is the last one: a flange stands on something.
pub fn toggled_edge(edges: &[String], id: TopoId) -> Option<Vec<String>> {
    let name = format!("{id:?}");
    if edges.contains(&name) {
        let left: Vec<String> = edges.iter().filter(|e| **e != name).cloned().collect();
        (!left.is_empty()).then_some(left)
    } else {
        Some(edges.iter().cloned().chain([name]).collect())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn id(n: u128) -> TopoId {
        TopoId(n)
    }

    fn square(used: bool, name: &str) -> Profile {
        Profile { curves: vec![name.into()], lines: Vec::new(), used }
    }

    #[test]
    fn an_edge_goes_in_and_out_but_the_last_stays() {
        let one = vec![format!("{:?}", id(1))];
        let two = toggled_edge(&one, id(2)).unwrap();
        assert_eq!(two.len(), 2);
        assert_eq!(toggled_edge(&two, id(2)).unwrap(), one);
        assert!(toggled_edge(&one, id(1)).is_none());
    }

    #[test]
    fn a_profile_goes_in_and_out_but_the_last_stays() {
        let u = Uses { profiles: vec![square(true, "a"), square(true, "b"), square(false, "c")], ..Uses::default() };
        assert_eq!(u.toggled_profile(0).unwrap(), ["b"]);
        // All of them used: no names, as a feature on all its profiles says it.
        assert_eq!(u.toggled_profile(2).unwrap(), Vec::<String>::new());
        let last = Uses { profiles: vec![square(true, "a"), square(false, "b")], ..Uses::default() };
        assert!(last.toggled_profile(0).is_none());
    }

    #[test]
    fn the_nearest_within_reach_is_taken() {
        let p = |x: f64| [Vec3::new(x, 0.0, 0.0), Vec3::new(x, 1.0, 0.0)];
        let u = Uses {
            edges: vec![(p(5.0), id(1))],
            profiles: vec![Profile { curves: vec!["a".into()], lines: vec![p(2.0), p(9.0)], used: true }],
            ..Uses::default()
        };
        let at = |l: [Vec3; 2]| Some((l[0].x as f32 - 4.0).abs());
        assert_eq!(u.nearest(at, 6.0), Some(Toggle::Edge(id(1))));
        let far = |l: [Vec3; 2]| Some((l[0].x as f32 - 100.0).abs());
        assert_eq!(u.nearest(far, 6.0), None);
    }

    use datum_model::Session;
    use serde_json::json;

    /// A session on reference model `model`.
    fn open(model: &str) -> Session {
        let mut s = Session::default();
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/models").join(model);
        s.execute("open", &json!({"path": path.display().to_string()})).unwrap();
        s
    }

    fn uses_in(s: &Session, name: &str) -> Option<Uses> {
        uses(s.graph(), s.evaluation(), name, &mut MeshCache::default())
    }

    /// Two squares in one sketch, extruded as `plate` on `profiles` (all of them if empty).
    fn two_squares(profiles: &[&str]) -> Session {
        let mut s = Session::default();
        let square = json!({"points": {"a": [0, 0], "b": [10, 0], "c": [10, 10], "d": [0, 10], "e": [20, 0], "f": [30, 0], "g": [30, 10], "h": [20, 10]},
            "lines": {"ab": {"from": "a", "to": "b"}, "bc": {"from": "b", "to": "c"}, "cd": {"from": "c", "to": "d"}, "da": {"from": "d", "to": "a"},
                "ef": {"from": "e", "to": "f"}, "fg": {"from": "f", "to": "g"}, "gh": {"from": "g", "to": "h"}, "he": {"from": "h", "to": "e"}}});
        s.execute("add", &json!({"name": "outline", "op": "sketch", "sketch": square})).unwrap();
        let mut p = json!({"name": "plate", "sketch": "outline", "mode": "new", "z1": 5});
        if !profiles.is_empty() {
            p["profiles"] = json!(profiles);
        }
        s.execute("extrude", &p).unwrap();
        s
    }

    fn used(u: &Uses) -> Vec<bool> {
        u.profiles.iter().map(|p| p.used).collect()
    }

    /// A flange stands on the edges it was made on, among those its sheet offers.
    #[test]
    fn a_flange_says_its_edges() {
        let s = open("sheet_rims.json");
        let u = uses_in(&s, "tray").unwrap();
        assert_eq!(u.inputs, ["plate"]);
        assert_eq!(u.used_edges.len(), 4, "{:?}", u.used_edges);
        for id in &u.used_edges {
            let (ends, _) = u.edges.iter().find(|e| e.1 == *id).expect("offered");
            assert_ne!(ends[0], ends[1]);
        }
        let rim = uses_in(&s, "rim_back").unwrap();
        assert_eq!(rim.inputs, ["tray"]);
        assert_eq!(rim.used_edges.len(), 1);
        assert!(rim.edges.len() > 4, "{}", rim.edges.len());
        assert!(rim.edges.iter().any(|e| e.1 == rim.used_edges[0]));
    }

    /// A flange with a plane in `to` lists that plane.
    #[test]
    fn a_flange_to_a_plane_lists_the_plane() {
        let s = open("sheet_flange_to_plane.json");
        let u = uses_in(&s, "grown").unwrap();
        assert_eq!(u.inputs, ["plate", "joint", "lid"]);
        assert_eq!(u.planes, ["joint", "lid"]);
    }

    /// An extrusion on one profile of two uses it and its sketch only; on none named, both.
    #[test]
    fn an_extrusion_says_its_profiles() {
        let all = uses_in(&two_squares(&[]), "plate").unwrap();
        assert_eq!(all.inputs, ["outline"]);
        assert_eq!(used(&all), [true, true]);
        assert!(all.profiles.iter().all(|p| !p.lines.is_empty()));
        let s = two_squares(&["ab"]);
        let one = uses_in(&s, "plate").unwrap();
        assert_eq!(one.profiles.len(), 2);
        assert_eq!(one.profiles.iter().filter(|p| p.used).count(), 1);
        assert!(one.profiles.iter().find(|p| p.used).unwrap().curves.contains(&"ab".to_string()));
    }

    /// What a click on a profile gives, fed back through `edit`, is what `uses` then says.
    #[test]
    fn toggled_profiles_come_back_through_edit() {
        let mut s = two_squares(&[]);
        let u = uses_in(&s, "plate").unwrap();
        let k = u.profiles.iter().position(|p| p.curves.contains(&"ab".to_string())).unwrap();
        // Taking one out leaves the other.
        let names = u.toggled_profile(k).unwrap();
        s.execute("edit", &json!({"name": "plate", "profiles": names})).unwrap();
        let u = uses_in(&s, "plate").unwrap();
        assert_eq!(u.profiles.iter().filter(|p| p.used).count(), 1);
        assert!(!u.profiles[k].used);
        // Putting it back is all of them, which a feature says by naming none.
        let names = u.toggled_profile(k).unwrap();
        assert!(names.is_empty());
        s.execute("edit", &json!({"name": "plate", "profiles": names})).unwrap();
        assert_eq!(used(&uses_in(&s, "plate").unwrap()), [true, true]);
        // The last one used stays.
        let mut s = two_squares(&["ab"]);
        let u = uses_in(&s, "plate").unwrap();
        let only = u.profiles.iter().position(|p| p.used).unwrap();
        assert!(u.toggled_profile(only).is_none());
        let other = 1 - only;
        let names = u.toggled_profile(other).unwrap();
        assert!(names.is_empty());
        s.execute("edit", &json!({"name": "plate", "profiles": names})).unwrap();
        assert_eq!(used(&uses_in(&s, "plate").unwrap()), [true, true]);
    }

    /// A flange's edges, taken in by `toggled_edge` and fed back through `edit`, are used.
    #[test]
    fn toggled_edges_come_back_through_edit() {
        let mut s = open("sheet_rims.json");
        let u = uses_in(&s, "rim_back").unwrap();
        let now: Vec<String> = u.used_edges.iter().map(|e| format!("{e:?}")).collect();
        let other = u.edges.iter().map(|e| e.1).find(|e| !u.used_edges.contains(e)).unwrap();
        let both = toggled_edge(&now, other).unwrap();
        s.execute("edit", &json!({"name": "rim_back", "edges": both})).unwrap();
        let after = uses_in(&s, "rim_back").unwrap();
        assert_eq!(after.used_edges.len(), 2);
        assert!(after.used_edges.contains(&other) && after.used_edges.contains(&u.used_edges[0]));
    }

    /// A revolution and a roll draw their axis.
    #[test]
    fn a_revolution_and_a_roll_draw_their_axis() {
        let s = open("revolve.json");
        let u = uses_in(&s, "bushing").unwrap();
        assert_eq!(u.inputs, ["profile"]);
        assert!(!u.lines.is_empty() && u.edges.is_empty());
        assert!(!u.profiles.is_empty() && u.profiles.iter().all(|p| p.used));
        let s = open("rolled.json");
        let u = uses_in(&s, "shell").unwrap();
        assert_eq!(u.inputs, ["shell_profile"]);
        assert!(!u.lines.is_empty());
    }

    /// A trim stands on its sheet and its plane.
    #[test]
    fn a_trim_says_its_plane() {
        let s = open("rolled_trim.json");
        let u = uses_in(&s, "trimmed").unwrap();
        assert_eq!(u.inputs, ["shell", "slanted"]);
        assert_eq!(u.planes, ["slanted"]);
    }

    /// A saddle and a hole in the main pipe show the other pipe's edges, and only that.
    #[test]
    fn a_saddle_shows_the_other_pipe() {
        let s = open("branch_tee.json");
        for (name, inputs) in [("branch_saddle", ["branch", "main"]), ("main_with_hole", ["main", "branch"])] {
            let u = uses_in(&s, name).unwrap();
            assert_eq!(u.inputs, inputs);
            assert!(!u.lines.is_empty(), "{name}");
            // The body it changes is not drawn over: its lines are the other pipe's.
            let other = {
                let b = s.evaluation().results[s.graph().find(inputs[1]).unwrap() as usize].as_ref().unwrap().body().unwrap();
                MeshCache::default().get(b).edges.clone()
            };
            assert_eq!(u.lines.len(), other.len(), "{name}");
        }
    }

    /// A name no feature has is no uses.
    #[test]
    fn an_unknown_name_has_no_uses() {
        let s = open("sheet_rims.json");
        assert!(uses_in(&s, "nothing").is_none());
    }
}
