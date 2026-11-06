//! Lessons (D173, D230): files in the `lessons` folder beside the program, not code. A lesson
//! opens with no model and leads through steps: what to do, why, where it is on screen. A step is
//! done when the window and the model look as it says, however the engineer got there (a key, a
//! menu, a button), so nothing listens to how.

use std::path::{Path, PathBuf};

use serde::Deserialize;
use serde_json::Value as Json;

use crate::actions::ACTIONS;

const FORMAT: &str = "datum-lesson/1";

/// What a lesson may point at besides an action: the menus and the window's own places.
const PLACES: [&str; 10] =
    ["menu.file", "menu.edit", "menu.view", "menu.sketch", "menu.help", "toolbar.material", "toolbar.rule", "view", "sketch.done", "form.ok"];

/// What of the window a step's `done` may ask about; [`matches`] compares with these.
const WINDOW: [&str; 12] =
    ["model", "sketch", "tool", "picking", "form", "sheet_rule", "saved", "table", "project", "project_safe", "own_materials", "own_rules"];

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Lesson {
    format: String,
    pub title: String,
    #[serde(default)]
    pub about: String,
    pub steps: Vec<Step>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Step {
    /// What to do.
    pub what: String,
    /// Why: the designer's thought behind it.
    #[serde(default)]
    pub why: String,
    /// For one who is stuck.
    #[serde(default)]
    pub hint: String,
    /// Where it is: the first of these on screen is marked, so an action in a closed menu can
    /// name the menu next.
    #[serde(default)]
    pub at: Vec<String>,
    /// What the window (`window`) and the model's `inspect` (`model`) must look like, by
    /// [`matches`]. Neither: a step to read, done by its button.
    #[serde(default)]
    pub done: Done,
}

#[derive(Debug, Default, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Done {
    #[serde(default)]
    pub window: Option<Json>,
    #[serde(default)]
    pub model: Option<Json>,
}

impl Lesson {
    pub fn parse(text: &str) -> Result<Self, String> {
        let lesson: Self = serde_json::from_str(text).map_err(|e| e.to_string())?;
        lesson.check()?;
        Ok(lesson)
    }

    /// What in the lesson the window would lead nowhere with, in its author's words: a lesson
    /// gone stale says so instead of leading into nothing.
    fn check(&self) -> Result<(), String> {
        if self.format != FORMAT {
            return Err(format!("format {:?}, this Datum reads {FORMAT:?}", self.format));
        }
        if self.steps.is_empty() {
            return Err("no steps".into());
        }
        for (k, step) in self.steps.iter().enumerate() {
            let n = k + 1;
            if let Some(name) = step.at.iter().find(|a| !known_place(a)) {
                return Err(format!("step {n}: no action or place {name:?}"));
            }
            if let Some(window) = &step.done.window {
                let keys = window.as_object().ok_or_else(|| format!("step {n}: `window` is not an object"))?;
                if let Some(key) = keys.keys().find(|key| !WINDOW.contains(&key.as_str())) {
                    return Err(format!("step {n}: the window has no {key:?}"));
                }
            }
        }
        Ok(())
    }
}

fn known_place(name: &str) -> bool {
    PLACES.contains(&name) || ACTIONS.iter().any(|a| a.name == name)
}

/// Whether `value` is as `pattern` says: an object's keys each match (others are free), each item
/// of an array matches some item, `{"at_least": n}` asks an array or object for n entries, anything
/// else is equal.
pub fn matches(pattern: &Json, value: &Json) -> bool {
    match pattern {
        Json::Object(p) => {
            if let Some(n) = p.get("at_least").and_then(Json::as_u64) {
                let len = value.as_array().map(Vec::len).or_else(|| value.as_object().map(serde_json::Map::len));
                return len.is_some_and(|len| len as u64 >= n);
            }
            p.iter().all(|(key, want)| value.get(key).is_some_and(|got| matches(want, got)))
        }
        Json::Array(p) => value.as_array().is_some_and(|items| p.iter().all(|want| items.iter().any(|got| matches(want, got)))),
        _ => pattern == value,
    }
}

/// The folder of lessons: beside the program, as the starter library.
pub fn folder() -> Option<PathBuf> {
    std::env::current_exe().ok()?.parent().map(|d| d.join("lessons"))
}

/// A lesson file read: its lesson, or what is wrong with it.
pub type Read = (PathBuf, Result<Lesson, String>);

/// The lessons of `dir`, by file name: numbered files come in their order.
pub fn read(dir: &Path) -> Vec<Read> {
    let mut files: Vec<PathBuf> = std::fs::read_dir(dir)
        .into_iter()
        .flatten()
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|x| x == "json"))
        .collect();
    files.sort();
    files
        .into_iter()
        .map(|f| {
            let lesson = std::fs::read_to_string(&f).map_err(|e| e.to_string()).and_then(|t| Lesson::parse(&t));
            (f, lesson)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use serde_json::json;

    use super::*;

    fn lesson(step: &Json) -> String {
        json!({"format": FORMAT, "title": "t", "steps": [step]}).to_string()
    }

    fn refused(text: &str) -> String {
        Lesson::parse(text).expect_err("must be refused")
    }

    const OK_STEP: &str = r#"{"what": "w", "done": {"window": {"model": true}}}"#;

    #[test]
    fn matches_objects_by_subset() {
        let value = json!({"a": 1, "b": {"c": 2, "d": 3}});
        assert!(matches(&json!({"a": 1}), &value));
        assert!(matches(&json!({"b": {"d": 3}}), &value));
        assert!(matches(&json!({}), &value));
        assert!(!matches(&json!({"z": 1}), &value));
        assert!(!matches(&json!({"b": {"z": 1}}), &value));
        assert!(!matches(&json!({"a": 2}), &value));
    }

    #[test]
    fn matches_array_items_against_some_item() {
        let value = json!([{"op": "sketch"}, {"op": "plate", "n": 1}]);
        assert!(matches(&json!([{"op": "plate"}]), &value));
        assert!(matches(&json!([{"op": "plate"}, {"op": "sketch"}]), &value));
        assert!(!matches(&json!([{"op": "revolve"}]), &value));
        assert!(!matches(&json!([{"op": "plate"}, {"op": "revolve"}]), &value));
        assert!(!matches(&json!([{"op": "plate"}]), &json!({"op": "plate"})));
    }

    #[test]
    fn matches_at_least_on_collections() {
        let at_least = |n: u64| json!({"at_least": n});
        let array = json!([1, 2, 3]);
        assert!(matches(&at_least(3), &array));
        assert!(!matches(&at_least(4), &array));
        assert!(matches(&at_least(0), &json!([])));
        let object = json!({"a": 1, "b": 2});
        assert!(matches(&at_least(2), &object));
        assert!(!matches(&at_least(3), &object));
        assert!(!matches(&at_least(1), &json!(5)));
        assert!(!matches(&at_least(0), &json!("text")));
        assert!(!matches(&at_least(0), &Json::Null));
        assert!(matches(&json!({"lines": at_least(2)}), &json!({"lines": {"a": 1, "b": 2}})));
    }

    #[test]
    fn matches_scalars_by_equality() {
        assert!(matches(&json!(true), &json!(true)));
        assert!(!matches(&json!(true), &json!(false)));
        assert!(matches(&json!("sketch"), &json!("sketch")));
        assert!(!matches(&json!("sketch"), &json!("look")));
        assert!(matches(&json!(null), &Json::Null));
        assert!(!matches(&json!({"picking": "sketch"}), &json!({"picking": null})));
        assert!(!matches(&json!({"picking": "sketch"}), &json!({})));
    }

    #[test]
    fn a_good_lesson_parses() {
        assert!(Lesson::parse(&lesson(&serde_json::from_str(OK_STEP).unwrap())).is_ok());
    }

    #[test]
    fn parse_refuses_stale_lessons_naming_the_step() {
        let step = |s: &str| lesson(&serde_json::from_str(s).unwrap());
        let two = |bad: &str| format!(r#"{{"format": "{FORMAT}", "title": "t", "steps": [{OK_STEP}, {bad}]}}"#);
        let e = refused(&two(r#"{"what": "w", "at": ["no.such.thing"], "done": {"window": {"model": true}}}"#));
        assert!(e.contains("step 2") && e.contains("no.such.thing"), "{e}");
        // A step without `done` is one to read, advanced by a button.
        assert!(Lesson::parse(&two(r#"{"what": "w", "done": {}}"#)).is_ok());
        assert!(Lesson::parse(&two(r#"{"what": "w"}"#)).is_ok());
        let e = refused(&two(r#"{"what": "w", "done": {"window": {"bogus": true}}}"#));
        assert!(e.contains("step 2") && e.contains("bogus"), "{e}");
        let e = refused(&step(r#"{"what": "w", "done": {"window": {"bogus": true}}}"#));
        assert!(e.contains("step 1"), "{e}");
        let e = refused(&step(r#"{"what": "w", "done": {"window": true}}"#));
        assert!(e.contains("step 1"), "{e}");
        let e = refused(&two(r#"{"what": "w", "surprise": 1, "done": {"window": {"model": true}}}"#));
        assert!(e.contains("surprise"), "{e}");
        let e = refused(&format!(r#"{{"format": "datum-lesson/0", "title": "t", "steps": [{OK_STEP}]}}"#));
        assert!(e.contains("format"), "{e}");
        let e = refused(&format!(r#"{{"format": "{FORMAT}", "title": "t", "steps": []}}"#));
        assert!(e.contains("no steps"), "{e}");
    }

    fn repo_lessons() -> PathBuf {
        PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../../lessons"))
    }

    #[test]
    fn every_lesson_of_the_repo_parses() {
        let all = read(&repo_lessons());
        assert!(!all.is_empty(), "the repo has no lessons");
        for (file, lesson) in &all {
            assert!(lesson.is_ok(), "{}: {:?}", file.display(), lesson.as_ref().err());
        }
    }

    #[test]
    fn read_sorts_by_name_and_reports_a_broken_file() {
        let dir = std::env::temp_dir().join(format!("datum-lessons-test-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        let good = lesson(&serde_json::from_str(OK_STEP).unwrap());
        std::fs::write(dir.join("20-b.json"), &good).unwrap();
        std::fs::write(dir.join("10-a.json"), "{not json").unwrap();
        std::fs::write(dir.join("05-first.json"), &good).unwrap();
        std::fs::write(dir.join("notes.txt"), "ignored").unwrap();
        let all = read(&dir);
        std::fs::remove_dir_all(&dir).unwrap();
        let names: Vec<_> = all.iter().map(|(f, _)| f.file_name().unwrap().to_string_lossy().to_string()).collect();
        assert_eq!(names, ["05-first.json", "10-a.json", "20-b.json"]);
        assert!(all[0].1.is_ok() && all[1].1.is_err() && all[2].1.is_ok());
    }

    /// The fallback of the mark: a step that names places has some name of a place the window fills
    /// every frame (the menu bar, the view), so the mark is never lost with a menu closed.
    /// Checked by names; that the window fills them is `lesson_card`'s `spots`. Steps without `at`
    /// happen in a window of their own (a table) and have no mark.
    #[test]
    fn every_step_with_places_names_an_always_shown_one() {
        for (file, lesson) in read(&repo_lessons()) {
            let lesson = lesson.unwrap();
            for (k, step) in lesson.steps.iter().enumerate().filter(|(_, s)| !s.at.is_empty()) {
                let shown = step.at.iter().any(|a| a.starts_with("menu.") || a.starts_with("toolbar.") || a == "view");
                assert!(shown, "{} step {}: {:?} has no always-shown place", file.display(), k + 1, step.at);
            }
        }
    }
}
