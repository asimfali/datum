//! The window's actions (D131): one list behind the menu, the keys and the hints. An action has
//! a name (the one `settings.json` uses), a label, a default key and where it applies; the
//! window says when it is available and runs it. The keys in force are the defaults with the
//! engineer's changes (D132).

use egui::{Key, KeyboardShortcut, Modifiers};

/// Every action, in the order of [`ACTIONS`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Act {
    New,
    Open,
    Save,
    SaveAs,
    Close,
    Exit,
    Undo,
    Redo,
    SmoothEdges,
    NewSketch,
    Select,
    Line,
    Arc,
    Circle,
    Point,
    Dimension,
    Measure,
    Rectangle,
    Tangent,
    NewProject,
    OpenProject,
    CloseProject,
    ExportStep,
    Home,
    LookAt,
    WorkPlane,
    SideBar,
    SideBarRight,
    FlatPane,
    ResetLayout,
    Group,
    Pattern,
    Infer,
    Presets,
    Badges,
    Names,
    Dimensions,
    ParameterTable,
    MaterialTable,
}

/// Where an action's key works: anywhere, in the model (no sketch edited), in a sketch.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Context {
    Anywhere,
    Model,
    Sketch,
}

impl Context {
    /// Whether a key could run in both: the same key may serve the model and a sketch, never
    /// anywhere and either.
    fn overlaps(self, other: Self) -> bool {
        self == other || self == Context::Anywhere || other == Context::Anywhere
    }
}

pub struct Action {
    pub act: Act,
    pub name: &'static str,
    pub label: &'static str,
    /// What it does, for the hint; empty when the label says it.
    pub about: &'static str,
    /// The default key; the one in force is in [`Keys`].
    pub key: Option<KeyboardShortcut>,
    pub context: Context,
}

const fn action(act: Act, name: &'static str, label: &'static str, about: &'static str, key: Option<KeyboardShortcut>, context: Context) -> Action {
    Action { act, name, label, about, key, context }
}

const fn ctrl(key: Key) -> Option<KeyboardShortcut> {
    Some(KeyboardShortcut::new(Modifiers::COMMAND, key))
}

const fn plain(key: Key) -> Option<KeyboardShortcut> {
    Some(KeyboardShortcut::new(Modifiers::NONE, key))
}

use Context::{Anywhere, Model, Sketch};

pub const ACTIONS: [Action; 39] = [
    action(Act::New, "file.new", "New", "An empty model", ctrl(Key::N), Anywhere),
    action(Act::Open, "file.open", "Open…", "A model from a file; a .json dropped on the window opens too", ctrl(Key::O), Anywhere),
    action(Act::Save, "file.save", "Save", "", ctrl(Key::S), Anywhere),
    action(Act::SaveAs, "file.save_as", "Save as…", "", Some(KeyboardShortcut::new(Modifiers::COMMAND.plus(Modifiers::SHIFT), Key::S)), Anywhere),
    action(Act::Close, "file.close", "Close", "Close the model", ctrl(Key::W), Anywhere),
    action(Act::Exit, "file.exit", "Exit", "", None, Anywhere),
    action(Act::Undo, "edit.undo", "Undo", "", ctrl(Key::Z), Anywhere),
    action(Act::Redo, "edit.redo", "Redo", "", ctrl(Key::Y), Anywhere),
    action(
        Act::SmoothEdges,
        "view.smooth_edges",
        "Smooth edges",
        "Multisampling (MSAA ×4) when the view is still; while it turns, frames are drawn without it",
        None,
        Anywhere,
    ),
    action(
        Act::NewSketch,
        "sketch.new",
        "New sketch",
        "Then click an origin plane or a plane face; Enter or a right click: XY, picked from the start",
        plain(Key::S),
        Model,
    ),
    action(Act::Select, "sketch.select", "Select", "Select and drag (Esc)", None, Sketch),
    action(Act::Line, "sketch.line", "Line", "Lines in a chain; T or a drag from its end: a tangent arc; Esc ends", plain(Key::L), Sketch),
    action(Act::Arc, "sketch.arc", "Arc", "Centre, start, end", None, Sketch),
    action(Act::Circle, "sketch.circle", "Circle", "Centre, then a point on it", plain(Key::C), Sketch),
    action(Act::Point, "sketch.point", "Point", "A point", plain(Key::P), Sketch),
    action(
        Act::Dimension,
        "sketch.dimension",
        "Dimension",
        "Click a line, an arc or a circle; a point, then a line; two points, then where it goes. Frame two lines: their angle",
        plain(Key::D),
        Sketch,
    ),
    action(
        Act::Measure,
        "tools.measure",
        "Measure",
        "For now in a sketch: click a point, a curve or a pattern's copy, then another: the distance or the angle; a click inside a closed region: its area; Shift adds to it; Esc or F3 ends",
        plain(Key::F3),
        Anywhere,
    ),
    action(Act::Rectangle, "sketch.library_first", "Rectangle", "The first element of the library", plain(Key::R), Sketch),
    action(Act::Tangent, "sketch.tangent_arc", "Tangent arc", "The next segment of the line: an arc tangent to the last one", plain(Key::T), Sketch),
    action(Act::NewProject, "file.new_project", "New project…", "A folder made a project: new models and exports go into it", None, Anywhere),
    action(Act::OpenProject, "file.open_project", "Open project…", "New models, dialogs and the library work in it", None, Anywhere),
    action(Act::CloseProject, "file.close_project", "Close project", "Back to the folder beside the program", None, Anywhere),
    action(
        Act::ExportStep,
        "file.export_step",
        "Export STEP…",
        "The model's bodies for other CADs (AP242, mm), with volume, area and centre of mass to check them by; \
         what STEP has no exact type for is written as B-splines only if you agree",
        None,
        Anywhere,
    ),
    action(
        Act::Home,
        "view.home",
        "Home view",
        "The default direction, the whole model in view; the house over the navigation cube",
        plain(Key::F6),
        Model,
    ),
    action(Act::LookAt, "view.look_at", "Look at", "Then click a face or a plane: the view straight at it", plain(Key::PageUp), Model),
    action(
        Act::WorkPlane,
        "model.plane",
        "Plane",
        "A work plane: click an origin plane, a work plane or a plane face, then type its offset (a number or a formula)",
        plain(Key::CloseBracket),
        Model,
    ),
    action(Act::SideBar, "view.side_bar", "Side bar", "The sections of the icon chosen on the activity bar", ctrl(Key::B), Anywhere),
    action(Act::SideBarRight, "view.side_bar_right", "Side bar on the right", "The activity bar and side bar at the right edge", None, Anywhere),
    action(
        Act::FlatPane,
        "view.flat_pane",
        "Flat pattern",
        "The flat blank in a pane beside the 3D view; drag its tab to put it elsewhere",
        None,
        Anywhere,
    ),
    action(
        Act::ResetLayout,
        "view.reset_layout",
        "Reset layout",
        "The panes, the side bar and the window back as a first run has them",
        None,
        Anywhere,
    ),
    action(
        Act::Group,
        "sketch.group",
        "Group",
        "The selected points and curves as a group: its badge marks them, a click on it selects them",
        plain(Key::G),
        Sketch,
    ),
    action(
        Act::Pattern,
        "sketch.pattern",
        "Pattern",
        "Copies, by what is selected: a point or a circle — around it; one or two lines — a grid along them; \
         lines and arcs closed into a contour, or a group — filling it or along its outline; an open chain — along it; \
         nothing — a grid along the axes. Copies selected: shifted, left out, their pattern edited. \
         Then pick what to copy. The right button held on the sketch does the same",
        plain(Key::A),
        Sketch,
    ),
    action(
        Act::Infer,
        "sketch.infer",
        "Infer constraints",
        "A line drawn takes the constraint it leans to (horizontal, vertical, parallel, perpendicular); off: it goes where you click",
        plain(Key::F7),
        Sketch,
    ),
    action(
        Act::Presets,
        "sketch.presets",
        "By presets",
        "A library element with presets is placed by one of them: ↑ ↓ pick it from the list by the cursor, a click puts it",
        plain(Key::F4),
        Sketch,
    ),
    action(
        Act::Badges,
        "sketch.badges",
        "Badges",
        "Badges of the geometric constraints: click one to select it, Del removes it",
        plain(Key::F8),
        Sketch,
    ),
    action(Act::Names, "sketch.names", "Names", "The names of points, curves, groups and patterns", plain(Key::F9), Sketch),
    action(
        Act::Dimensions,
        "sketch.dimensions",
        "Dimensions",
        "The dimensions' values; hidden, a crowded sketch reads (the dimension tool shows them)",
        plain(Key::F10),
        Sketch,
    ),
    action(
        Act::ParameterTable,
        "view.parameter_table",
        "Parameters table",
        "The model's parameters in a window of their own: Excel ranges paste in and copy out",
        None,
        Anywhere,
    ),
    action(
        Act::MaterialTable,
        "view.material_table",
        "Materials",
        "The library's materials in a window of their own: name, designation, density, description",
        None,
        Anywhere,
    ),
];

impl Act {
    pub fn action(self) -> &'static Action {
        &ACTIONS[self as usize]
    }
}

/// The key of every action in force, by [`Act`].
pub struct Keys([Option<KeyboardShortcut>; ACTIONS.len()]);

impl Default for Keys {
    fn default() -> Self {
        Self(ACTIONS.map(|a| a.key))
    }
}

impl Keys {
    pub fn get(&self, act: Act) -> Option<&KeyboardShortcut> {
        self.0[act as usize].as_ref()
    }

    /// Gives `act` `key` (none: no key), taken from the actions it would run with too; returns
    /// those.
    pub fn set(&mut self, act: Act, key: Option<KeyboardShortcut>) -> Vec<Act> {
        let mut taken = Vec::new();
        if key.is_some() {
            for a in &ACTIONS {
                if a.act != act && a.context.overlaps(act.action().context) && self.0[a.act as usize] == key {
                    self.0[a.act as usize] = None;
                    taken.push(a.act);
                }
            }
        }
        self.0[act as usize] = key;
        taken
    }

    /// The actions whose key is `key`.
    pub fn with(&self, key: &KeyboardShortcut) -> Vec<Act> {
        ACTIONS.iter().map(|a| a.act).filter(|a| self.get(*a) == Some(key)).collect()
    }

    /// The label with its key, as a checkbox shows it: "Tangent arc (T)".
    pub fn labelled(&self, ctx: &egui::Context, i18n: &crate::i18n::I18n, act: Act) -> String {
        let label = i18n.label(act);
        match self.get(act) {
            Some(k) => format!("{label} ({})", ctx.format_shortcut(k)),
            None => label.to_string(),
        }
    }

    /// The hover text: what it does, its key and its name.
    pub fn hint(&self, ctx: &egui::Context, i18n: &crate::i18n::I18n, act: Act) -> String {
        let a = act.action();
        let about = if a.about.is_empty() { i18n.label(act) } else { i18n.about(act) };
        match self.get(act) {
            Some(k) => format!("{about} ({})\n{}", ctx.format_shortcut(k), a.name),
            None => format!("{about}\n{}", a.name),
        }
    }
}

/// Whether `key` was pressed with exactly its modifiers, taking the press: Ctrl+Shift+S is not
/// Ctrl+S, and Shift+S is not S.
pub fn pressed(input: &mut egui::InputState, key: &KeyboardShortcut) -> bool {
    let before = input.events.len();
    input.events.retain(|e| {
        !matches!(e, egui::Event::Key { key: k, pressed: true, modifiers, .. } if *k == key.logical_key && modifiers.matches_exact(key.modifiers))
    });
    input.events.len() < before
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn listed_in_order_named_once() {
        for (k, a) in ACTIONS.iter().enumerate() {
            assert_eq!(a.act as usize, k, "{}", a.name);
            assert!(ACTIONS.iter().filter(|b| b.name == a.name).count() == 1, "{} twice", a.name);
        }
    }

    /// No default key runs two actions.
    #[test]
    fn no_key_twice() {
        for (i, a) in ACTIONS.iter().enumerate() {
            for b in &ACTIONS[i + 1..] {
                assert!(!(a.context.overlaps(b.context) && a.key.is_some() && a.key == b.key), "{} and {}", a.name, b.name);
            }
        }
    }

    /// A key given to one action is taken from another it would run with, not from one in
    /// another context.
    #[test]
    fn a_key_taken() {
        let mut keys = Keys::default();
        let s = KeyboardShortcut::new(Modifiers::NONE, Key::S);
        assert_eq!(keys.set(Act::Line, Some(s)), [], "S of the model is not the sketch's");
        assert_eq!(keys.with(&s), [Act::NewSketch, Act::Line]);
        let ctrl_s = Act::Save.action().key;
        assert_eq!(keys.set(Act::Line, ctrl_s), [Act::Save], "Ctrl+S works in a sketch too");
        assert_eq!((keys.get(Act::Save), keys.get(Act::Line)), (None, ctrl_s.as_ref()));
    }

    #[test]
    fn modifiers_exactly() {
        let run = |modifiers, key: Option<KeyboardShortcut>| {
            let mut input = egui::InputState::default();
            input.events = vec![egui::Event::Key { key: Key::S, physical_key: None, pressed: true, repeat: false, modifiers }];
            pressed(&mut input, &key.unwrap())
        };
        let (save, save_as, sketch) = (Act::Save.action().key, Act::SaveAs.action().key, Act::NewSketch.action().key);
        let ctrl_shift = Modifiers::CTRL | Modifiers::COMMAND | Modifiers::SHIFT;
        assert!(run(ctrl_shift, save_as));
        assert!(!run(ctrl_shift, save), "Ctrl+Shift+S is not Ctrl+S");
        assert!(run(Modifiers::CTRL | Modifiers::COMMAND, save));
        assert!(!run(Modifiers::SHIFT, sketch), "Shift+S is not S");
        assert!(run(Modifiers::NONE, sketch));
    }
}
