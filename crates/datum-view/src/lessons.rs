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

/// The keys a lesson may press by name, not being an action's.
const KEYS: [&str; 2] = ["enter", "escape"];

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
    /// What the window does to show the step, or to do it for the engineer.
    #[serde(default)]
    pub show: Vec<Move>,
    /// The choice is the engineer's (a folder, a file, a material): the window shows where, and
    /// does not do it for them.
    #[serde(default)]
    pub own: bool,
    /// What the window (`window`) and the model's `inspect` (`model`) must look like, by
    /// [`matches`]. Neither: a step to read, done by its button.
    #[serde(default)]
    pub done: Done,
}

/// One move of the window's hand (D230).
#[derive(Clone, Debug, Deserialize)]
#[serde(rename_all = "snake_case", deny_unknown_fields)]
pub enum Move {
    /// The cursor goes to this place and stays.
    Point(String),
    /// The cursor goes to this place and clicks.
    Click(String),
    /// The key of this action, as it is now; or `enter`, `escape`.
    Key(String),
    /// The cursor goes to this point of the sketch's view and clicks: from its middle, a share of
    /// its half width and half height, up positive. The zoom depends on the window: millimetres
    /// might be off screen, and an empty sketch has nothing to place by.
    Sketch([f32; 2]),
    /// Writes a library element: what is typed in a table window, which the hand cannot reach.
    Put { level: String, element: Json },
}

impl Move {
    /// Whether the window can take this back: what it writes to a library stays.
    fn undoable(&self) -> bool {
        !matches!(self, Self::Put { .. })
    }
}

impl Step {
    /// Whether the window can show this step and take it back.
    pub fn can_show(&self) -> bool {
        !self.show.is_empty() && self.show.iter().all(Move::undoable)
    }

    /// Whether the window can do this step for the engineer.
    pub fn can_do(&self) -> bool {
        !self.show.is_empty() && !self.own
    }
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
            for m in &step.show {
                match m {
                    Move::Point(name) | Move::Click(name) if !known_place(name) => return Err(format!("step {n}: no action or place {name:?}")),
                    Move::Key(name) if !KEYS.contains(&name.as_str()) && !ACTIONS.iter().any(|a| a.name == name) => {
                        return Err(format!("step {n}: no action or key {name:?}"));
                    }
                    Move::Put { level, .. } if !matches!(level.as_str(), "material" | "rule") => {
                        return Err(format!("step {n}: `put` writes a material or a rule, not {level:?}"));
                    }
                    _ => {}
                }
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

/// Seconds the hand takes to reach a place, and to rest after a move so the eye can follow.
const TRAVEL: f32 = 0.7;
const REST: f32 = 0.5;
/// Seconds it waits for a place to appear before saying it is not there.
const PATIENCE: f32 = 5.0;

/// What a move comes to in the window now.
pub enum Aim {
    Spot(egui::Pos2),
    Keys(egui::Key, egui::Modifiers, String),
    Put(String, Json),
}

enum Phase {
    Next,
    Travel { from: egui::Pos2, to: egui::Pos2, t: f32, press: bool },
    Release(egui::Event),
    Rest(f32),
}

/// What became of a tick of the hand.
pub enum Tick {
    Going,
    /// A library element to write, level and element.
    Put(String, Json),
    Finished,
    /// A place that did not appear.
    Lost(String),
}

/// The window's hand: plays a step's moves as input a person would give, one event a frame, so
/// the moves go the same way as the engineer's (D230).
pub struct Hand {
    moves: Vec<Move>,
    at: usize,
    phase: Phase,
    waited: f32,
    pub cursor: egui::Pos2,
    /// The key last pressed, for the eye, until the next move.
    pub pressed: Option<String>,
}

impl Hand {
    pub fn new(moves: Vec<Move>, cursor: egui::Pos2) -> Self {
        Self { moves, at: 0, phase: Phase::Next, waited: 0.0, cursor, pressed: None }
    }

    /// One frame of `dt` seconds. It waits while `busy`, as a person looks before going on; `aim`
    /// says what a move comes to now, none while its place is not on screen. Input goes to `out`.
    pub fn tick(&mut self, dt: f32, busy: bool, aim: impl Fn(&Move) -> Option<Aim>, out: &mut Vec<egui::Event>) -> Tick {
        let click =
            |pos, pressed| egui::Event::PointerButton { pos, button: egui::PointerButton::Primary, pressed, modifiers: egui::Modifiers::NONE };
        match std::mem::replace(&mut self.phase, Phase::Next) {
            Phase::Next => {
                if busy {
                    return Tick::Going;
                }
                let Some(m) = self.moves.get(self.at) else { return Tick::Finished };
                self.pressed = None;
                match aim(m) {
                    None => {
                        self.waited += dt;
                        if self.waited > PATIENCE {
                            return Tick::Lost(format!("{m:?}"));
                        }
                    }
                    Some(Aim::Spot(to)) => {
                        self.waited = 0.0;
                        self.phase = Phase::Travel { from: self.cursor, to, t: 0.0, press: !matches!(m, Move::Point(_)) };
                    }
                    Some(Aim::Keys(key, modifiers, label)) => {
                        self.waited = 0.0;
                        let event = |pressed| egui::Event::Key { key, physical_key: None, pressed, repeat: false, modifiers };
                        out.push(event(true));
                        self.pressed = Some(label);
                        self.phase = Phase::Release(event(false));
                    }
                    Some(Aim::Put(level, element)) => {
                        self.waited = 0.0;
                        self.phase = Phase::Rest(REST);
                        return Tick::Put(level, element);
                    }
                }
            }
            Phase::Travel { from, to, t, press } => {
                let t = (t + dt / TRAVEL).min(1.0);
                // Eased, as a hand moves.
                self.cursor = from + (to - from) * (t * t * (3.0 - 2.0 * t));
                out.push(egui::Event::PointerMoved(self.cursor));
                self.phase = if t < 1.0 {
                    Phase::Travel { from, to, t, press }
                } else if press {
                    out.push(click(to, true));
                    Phase::Release(click(to, false))
                } else {
                    Phase::Rest(REST)
                };
            }
            Phase::Release(event) => {
                out.push(event);
                self.phase = Phase::Rest(REST);
            }
            Phase::Rest(left) => {
                if left > dt {
                    self.phase = Phase::Rest(left - dt);
                } else {
                    self.at += 1;
                }
            }
        }
        Tick::Going
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

    fn moves(text: &str) -> Vec<Move> {
        let step: Step = serde_json::from_str(text).unwrap();
        step.show
    }

    fn step(text: &str) -> Step {
        serde_json::from_str(text).unwrap()
    }

    #[test]
    fn parse_checks_the_moves_naming_the_step() {
        let two = |bad: &str| format!(r#"{{"format": "{FORMAT}", "title": "t", "steps": [{OK_STEP}, {bad}]}}"#);
        for (show, word) in [
            (r#"[{"click": "no.such.place"}]"#, "no.such.place"),
            (r#"[{"point": "nowhere"}]"#, "nowhere"),
            (r#"[{"key": "no_such_key"}]"#, "no_such_key"),
            (r#"[{"put": {"level": "sketch", "element": {}}}]"#, "sketch"),
        ] {
            let e = refused(&two(&format!(r#"{{"what": "w", "show": {show}}}"#)));
            assert!(e.contains("step 2") && e.contains(word), "{e}");
        }
        let ok = r#"[{"click": "menu.file"}, {"point": "view"}, {"key": "enter"}, {"key": "escape"}, {"key": "file.new"}, {"sketch": [0.5, -0.3]},
                {"put": {"level": "material", "element": {"name": "x"}}}, {"put": {"level": "rule", "element": {}}}]"#;
        let lesson = Lesson::parse(&two(&format!(r#"{{"what": "w", "show": {ok}}}"#))).unwrap();
        let shown = &lesson.steps[1].show;
        assert_eq!(shown.len(), 8);
        assert!(matches!(shown[5], Move::Sketch([x, y]) if x == 0.5 && y == -0.3));
        assert!(matches!(&shown[2], Move::Key(k) if k == "enter"));
    }

    #[test]
    fn can_show_and_can_do() {
        let none = step(r#"{"what": "w"}"#);
        assert!(!none.can_show() && !none.can_do());
        let clicks = step(r#"{"what": "w", "show": [{"click": "menu.file"}, {"key": "enter"}]}"#);
        assert!(clicks.can_show() && clicks.can_do());
        let own = step(r#"{"what": "w", "own": true, "show": [{"click": "menu.file"}]}"#);
        assert!(own.can_show() && !own.can_do());
        let put = step(r#"{"what": "w", "show": [{"click": "menu.file"}, {"put": {"level": "material", "element": {}}}]}"#);
        assert!(!put.can_show() && put.can_do());
    }

    #[test]
    fn a_put_step_of_the_repo_is_not_own() {
        for (file, lesson) in read(&repo_lessons()) {
            for (k, step) in lesson.unwrap().steps.iter().enumerate() {
                if step.show.iter().any(|m| matches!(m, Move::Put { .. })) {
                    assert!(!step.own, "{} step {}: a put step is done by the window", file.display(), k + 1);
                }
            }
        }
    }

    const DT: f32 = 1.0 / 60.0;

    fn pos(x: f32, y: f32) -> egui::Pos2 {
        egui::pos2(x, y)
    }

    /// Plays frames until the hand stops going; each frame's events and the final tick.
    fn play(hand: &mut Hand, aim: impl Fn(&Move) -> Option<Aim>) -> (Vec<Vec<egui::Event>>, Tick) {
        let mut frames = Vec::new();
        for _ in 0..10_000 {
            let mut out = Vec::new();
            let tick = hand.tick(DT, false, &aim, &mut out);
            frames.push(out);
            if !matches!(tick, Tick::Going) {
                return (frames, tick);
            }
        }
        panic!("the hand never stopped");
    }

    fn spot_at(target: egui::Pos2) -> impl Fn(&Move) -> Option<Aim> {
        move |_| Some(Aim::Spot(target))
    }

    fn presses(frames: &[Vec<egui::Event>]) -> Vec<(usize, egui::Pos2, bool)> {
        let mut found = Vec::new();
        for (k, frame) in frames.iter().enumerate() {
            for e in frame {
                if let egui::Event::PointerButton { pos, pressed, .. } = e {
                    found.push((k, *pos, *pressed));
                }
            }
        }
        found
    }

    #[test]
    fn a_click_moves_presses_at_the_target_releases_and_finishes() {
        let target = pos(321.5, 87.25);
        let mut hand = Hand::new(moves(r#"{"what": "w", "show": [{"click": "view"}]}"#), pos(10.0, 10.0));
        let (frames, tick) = play(&mut hand, spot_at(target));
        assert!(matches!(tick, Tick::Finished));
        let found = presses(&frames);
        assert_eq!(found.len(), 2, "one press and one release: {found:?}");
        let (pressed_at, pos_down, down) = found[0];
        assert!(down && pos_down == target);
        assert_eq!((found[1].0, found[1].1, found[1].2), (pressed_at + 1, target, false));
        // The press shares its frame with the last move, which is exactly the target.
        let last_move = frames[pressed_at].iter().rev().find_map(|e| if let egui::Event::PointerMoved(p) = e { Some(*p) } else { None });
        assert_eq!(last_move, Some(target));
        assert!(matches!(frames[pressed_at].last(), Some(egui::Event::PointerButton { .. })));
        // Moves only before the press, in one run of frames, none after.
        let moved: Vec<usize> = (0..frames.len()).filter(|&k| frames[k].iter().any(|e| matches!(e, egui::Event::PointerMoved(_)))).collect();
        assert_eq!(*moved.last().unwrap(), pressed_at);
        assert!(moved.windows(2).all(|w| w[1] == w[0] + 1));
        assert!(moved.len() > 10, "a hand takes its time");
        // The rest follows the release before it finishes.
        assert!(frames.len() > pressed_at + 2 + 10);
        assert_eq!(hand.cursor, target);
    }

    #[test]
    fn a_point_moves_and_does_not_click() {
        let target = pos(200.0, 40.0);
        let mut hand = Hand::new(moves(r#"{"what": "w", "show": [{"point": "view"}]}"#), pos(0.0, 0.0));
        let (frames, tick) = play(&mut hand, spot_at(target));
        assert!(matches!(tick, Tick::Finished));
        assert!(presses(&frames).is_empty());
        let last = frames.iter().flatten().filter_map(|e| if let egui::Event::PointerMoved(p) = e { Some(*p) } else { None }).next_back();
        assert_eq!(last, Some(target));
    }

    #[test]
    fn a_key_is_pressed_then_released_with_its_modifiers() {
        let mods = egui::Modifiers::COMMAND;
        let aim = move |_: &Move| Some(Aim::Keys(egui::Key::S, mods, "Ctrl+S".to_string()));
        let mut hand = Hand::new(moves(r#"{"what": "w", "show": [{"key": "file.save"}, {"point": "view"}]}"#), pos(0.0, 0.0));
        let mut frames = Vec::new();
        let mut pressed_seen = Vec::new();
        // Up to the second move: the label holds until it starts.
        for _ in 0..200 {
            let mut out = Vec::new();
            let tick = hand.tick(DT, false, |m| if matches!(m, Move::Key(_)) { aim(m) } else { None }, &mut out);
            assert!(matches!(tick, Tick::Going));
            pressed_seen.push(hand.pressed.clone());
            frames.push(out);
        }
        let keys: Vec<(usize, bool, egui::Modifiers, egui::Key)> = frames
            .iter()
            .enumerate()
            .flat_map(|(k, f)| {
                f.iter().filter_map(move |e| match e {
                    egui::Event::Key { key, pressed, modifiers, repeat: false, .. } => Some((k, *pressed, *modifiers, *key)),
                    _ => None,
                })
            })
            .collect();
        assert_eq!(keys.len(), 2, "{keys:?}");
        assert_eq!((keys[0].1, keys[0].2, keys[0].3), (true, mods, egui::Key::S));
        assert_eq!((keys[1].0, keys[1].1, keys[1].2, keys[1].3), (keys[0].0 + 1, false, mods, egui::Key::S));
        // The label is there from the press on, through the rest, and until the next move starts
        // (here the second move has no place yet: the label is cleared as it is looked at).
        assert_eq!(pressed_seen[keys[0].0].as_deref(), Some("Ctrl+S"));
        assert_eq!(pressed_seen[keys[1].0].as_deref(), Some("Ctrl+S"));
        assert!(pressed_seen[keys[1].0 + 1..keys[1].0 + 20].iter().all(|p| p.as_deref() == Some("Ctrl+S")));
        assert_eq!(pressed_seen.last().unwrap(), &None, "cleared when the next move starts");
    }

    #[test]
    fn busy_emits_nothing_and_does_not_count_as_waiting() {
        let target = pos(50.0, 50.0);
        let mut hand = Hand::new(moves(r#"{"what": "w", "show": [{"click": "view"}]}"#), pos(0.0, 0.0));
        for _ in 0..1000 {
            let mut out = Vec::new();
            assert!(matches!(hand.tick(DT, true, |_| None, &mut out), Tick::Going));
            assert!(out.is_empty());
        }
        // Not Lost after all that: a place that now exists is gone to and clicked.
        let (frames, tick) = play(&mut hand, spot_at(target));
        assert!(matches!(tick, Tick::Finished));
        assert_eq!(presses(&frames).len(), 2);
        // Busy with nothing to do does not finish either.
        let mut done = Hand::new(Vec::new(), pos(0.0, 0.0));
        let mut out = Vec::new();
        assert!(matches!(done.tick(DT, true, |_| None, &mut out), Tick::Going));
        assert!(matches!(done.tick(DT, false, |_| None, &mut out), Tick::Finished));
    }

    #[test]
    fn busy_in_the_middle_of_a_move_pauses_nothing_but_the_next_one() {
        let mut hand = Hand::new(moves(r#"{"what": "w", "show": [{"click": "view"}, {"click": "view"}]}"#), pos(0.0, 0.0));
        // Run until the rest after the first click, then be busy: nothing starts or finishes.
        let aim = spot_at(pos(5.0, 5.0));
        let mut out = Vec::new();
        for _ in 0..200 {
            hand.tick(DT, false, &aim, &mut out);
        }
        for _ in 0..1000 {
            let mut frame = Vec::new();
            let tick = hand.tick(DT, true, &aim, &mut frame);
            assert!(matches!(tick, Tick::Going));
            assert!(frame.is_empty());
        }
    }

    #[test]
    fn a_place_that_never_appears_is_lost_after_five_seconds() {
        let mut hand = Hand::new(moves(r#"{"what": "w", "show": [{"click": "view"}]}"#), pos(0.0, 0.0));
        let mut out = Vec::new();
        for k in 0..290 {
            assert!(matches!(hand.tick(DT, false, |_| None, &mut out), Tick::Going), "frame {k}");
        }
        let mut lost = false;
        for _ in 0..20 {
            if matches!(hand.tick(DT, false, |_| None, &mut out), Tick::Lost(_)) {
                lost = true;
                break;
            }
        }
        assert!(lost);
        assert!(out.is_empty());
    }

    #[test]
    fn a_place_that_appears_in_time_is_gone_to() {
        let target = pos(70.0, 30.0);
        let mut hand = Hand::new(moves(r#"{"what": "w", "show": [{"click": "view"}]}"#), pos(0.0, 0.0));
        let mut out = Vec::new();
        for _ in 0..250 {
            assert!(matches!(hand.tick(DT, false, |_| None, &mut out), Tick::Going));
        }
        let (frames, tick) = play(&mut hand, spot_at(target));
        assert!(matches!(tick, Tick::Finished));
        assert_eq!(presses(&frames)[0].1, target);
    }

    #[test]
    fn waiting_starts_over_with_each_move() {
        // 4 s of waiting for the first move, found, then 4 s for the second: not lost.
        let mut hand = Hand::new(moves(r#"{"what": "w", "show": [{"point": "view"}, {"click": "menu.file"}]}"#), pos(0.0, 0.0));
        let mut out = Vec::new();
        for _ in 0..240 {
            assert!(matches!(hand.tick(DT, false, |_| None, &mut out), Tick::Going));
        }
        let first = |m: &Move| matches!(m, Move::Point(_)).then(|| Aim::Spot(pos(1.0, 1.0)));
        let mut waited_second = 0;
        for _ in 0..400 {
            let tick = hand.tick(DT, false, first, &mut out);
            assert!(matches!(tick, Tick::Going | Tick::Lost(_)));
            if matches!(tick, Tick::Lost(_)) {
                break;
            }
            waited_second += 1;
        }
        // The second move waited the full patience from its own start, and then was lost.
        assert!(waited_second > 240, "{waited_second}");
    }

    #[test]
    fn a_put_comes_out_once_with_level_and_element() {
        let mut hand =
            Hand::new(moves(r#"{"what": "w", "show": [{"put": {"level": "material", "element": {"name": "x", "density": 1}}}]}"#), pos(0.0, 0.0));
        let aim = |m: &Move| match m {
            Move::Put { level, element } => Some(Aim::Put(level.clone(), element.clone())),
            _ => None,
        };
        let mut puts = Vec::new();
        let mut finished = false;
        for _ in 0..1000 {
            let mut out = Vec::new();
            match hand.tick(DT, false, aim, &mut out) {
                Tick::Put(level, element) => puts.push((level, element)),
                Tick::Finished => {
                    finished = true;
                    break;
                }
                Tick::Going => {}
                Tick::Lost(m) => panic!("lost {m}"),
            }
            assert!(out.is_empty());
        }
        assert!(finished);
        assert_eq!(puts, [("material".to_string(), json!({"name": "x", "density": 1}))]);
    }

    #[test]
    fn moves_play_in_order() {
        let mut hand = Hand::new(moves(r#"{"what": "w", "show": [{"click": "menu.file"}, {"click": "menu.edit"}]}"#), pos(0.0, 0.0));
        let aim = |m: &Move| match m {
            Move::Click(name) if name == "menu.file" => Some(Aim::Spot(pos(10.0, 20.0))),
            Move::Click(name) if name == "menu.edit" => Some(Aim::Spot(pos(90.0, 20.0))),
            _ => None,
        };
        let (frames, tick) = play(&mut hand, aim);
        assert!(matches!(tick, Tick::Finished));
        let found = presses(&frames);
        let positions: Vec<_> = found.iter().map(|(_, p, down)| (*p, *down)).collect();
        assert_eq!(positions, [(pos(10.0, 20.0), true), (pos(10.0, 20.0), false), (pos(90.0, 20.0), true), (pos(90.0, 20.0), false)]);
    }
}
