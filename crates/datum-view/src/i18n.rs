//! The window's language (D172). A catalog is a `locales/<code>.ftl` file of `key = text` lines
//! (a subset of Fluent's syntax, so a language that needs more can move to Fluent without its
//! files changing); `{ $name }` in a text is filled in when it is shown. English is the text at
//! the call site: a language only has to give what differs, and what it lacks shows in English.

use std::collections::BTreeMap;

use crate::actions::Act;

/// The languages with a catalog: code, its name in itself, the catalog's text.
const CATALOGS: [(&str, &str, &str); 1] = [("ru", "Русский", include_str!("../locales/ru.ftl"))];

/// Every language the window can be in, for its menu: code and name (`system`: none).
pub fn languages() -> Vec<(&'static str, Option<&'static str>)> {
    [("system", None), ("en", Some("English"))].into_iter().chain(CATALOGS.iter().map(|(c, n, _)| (*c, Some(*n)))).collect()
}

/// Whether `code` (`system` too) says a language the window has.
pub fn known(code: &str) -> bool {
    languages().iter().any(|(c, _)| *c == code)
}

/// The codes `language` takes, for what is said when it is something else.
pub fn codes() -> String {
    languages().iter().map(|(c, _)| *c).collect::<Vec<_>>().join(", ")
}

#[derive(Default)]
pub struct I18n {
    texts: BTreeMap<String, String>,
}

impl I18n {
    /// The language `setting` of `settings.json` says: a code, or `system` (also when it says
    /// nothing): the language of the system if the window has it, else English.
    pub fn chosen(setting: Option<&str>) -> Self {
        match setting {
            None | Some("system") => Self::of(&sys_locale::get_locale().unwrap_or_default()),
            Some(code) => Self::of(code),
        }
    }

    /// The language of a locale as the system writes it (`ru-RU`, `ru_RU.UTF-8`): English when
    /// there is no catalog.
    pub fn of(locale: &str) -> Self {
        let code = locale.split(['-', '_', '.']).next().unwrap_or_default().to_ascii_lowercase();
        let text = CATALOGS.iter().find(|(c, ..)| *c == code).map_or("", |(.., t)| t);
        Self { texts: parse(text) }
    }

    /// The text for `key` in this language, else `english`.
    pub fn t<'a>(&'a self, key: &str, english: &'a str) -> &'a str {
        self.texts.get(key).map_or(english, String::as_str)
    }

    /// [`Self::t`] with `{ $name }` filled in from `args`.
    pub fn f(&self, key: &str, english: &str, args: &[(&str, &str)]) -> String {
        let mut text = self.t(key, english).to_string();
        for (name, value) in args {
            text = text.replace(&format!("{{ ${name} }}"), value);
        }
        text
    }

    /// An action's label, and what it does for its hint.
    pub fn label(&self, act: Act) -> &str {
        self.t(&key(act, ""), act.action().label)
    }

    pub fn about(&self, act: Act) -> &str {
        let a = act.action();
        self.t(&key(act, "-about"), a.about)
    }
}

/// An action's catalog key: its name in Fluent's spelling, `file.save_as` → `act-file-save-as`.
fn key(act: Act, suffix: &str) -> String {
    format!("act-{}{suffix}", act.action().name.replace(['.', '_'], "-"))
}

fn parse(text: &str) -> BTreeMap<String, String> {
    text.lines()
        .filter(|l| !l.trim().is_empty() && !l.starts_with('#'))
        .filter_map(|l| l.split_once('=').map(|(k, v)| (k.trim().to_string(), v.trim().to_string())))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::actions::ACTIONS;

    #[test]
    fn a_text_is_the_language_s_else_english() {
        let ru = I18n::of("ru_RU.UTF-8");
        assert_eq!(ru.t("menu-file", "File"), "Файл");
        assert_eq!(ru.t("not-translated", "As is"), "As is");
        assert_eq!(I18n::of("de-DE").t("menu-file", "File"), "File");
        assert_eq!(I18n::of("").t("menu-file", "File"), "File");
        assert_eq!(ru.f("help-log", "x { $path }", &[("path", "C:\\datum.log")]), "Проблемы пишутся в C:\\datum.log");
        assert_eq!(I18n::default().f("help-log", "Logged to { $path }", &[("path", "p")]), "Logged to p");
    }

    /// Every text of the catalogs and the lessons is in the window's font: what it lacks shows as
    /// a square (D119).
    #[test]
    fn every_text_can_be_drawn() {
        let lessons = std::fs::read_dir(concat!(env!("CARGO_MANIFEST_DIR"), "/../../lessons")).unwrap();
        let mut texts: Vec<(String, String)> = CATALOGS.iter().map(|(code, _, text)| ((*code).to_string(), (*text).to_string())).collect();
        texts.extend(lessons.map(|f| {
            let path = f.unwrap().path();
            let text = std::fs::read_to_string(&path).unwrap();
            (path.display().to_string(), text)
        }));
        // The window's code: what is drawn as text in it, not its tests or comments.
        let sources = [
            ("actions.rs", include_str!("actions.rs")),
            ("app.rs", include_str!("app.rs")),
            ("camera.rs", include_str!("camera.rs")),
            ("capture.rs", include_str!("capture.rs")),
            ("constrain.rs", include_str!("constrain.rs")),
            ("cube.rs", include_str!("cube.rs")),
            ("dims.rs", include_str!("dims.rs")),
            ("dock.rs", include_str!("dock.rs")),
            ("draw.rs", include_str!("draw.rs")),
            ("flat.rs", include_str!("flat.rs")),
            ("i18n.rs", include_str!("i18n.rs")),
            ("icons.rs", include_str!("icons.rs")),
            ("lessons.rs", include_str!("lessons.rs")),
            ("log.rs", include_str!("log.rs")),
            ("measure.rs", include_str!("measure.rs")),
            ("origin.rs", include_str!("origin.rs")),
            ("pattern.rs", include_str!("pattern.rs")),
            ("render.rs", include_str!("render.rs")),
            ("scene.rs", include_str!("scene.rs")),
            ("select.rs", include_str!("select.rs")),
            ("settings.rs", include_str!("settings.rs")),
            ("sidebar.rs", include_str!("sidebar.rs")),
            ("sketch.rs", include_str!("sketch.rs")),
            ("table.rs", include_str!("table.rs")),
            ("templates.rs", include_str!("templates.rs")),
            ("update.rs", include_str!("update.rs")),
            ("lib.rs", include_str!("lib.rs")),
        ];
        for (name, source) in sources {
            let code = source.split("#[cfg(test)]").next().unwrap();
            let code: Vec<&str> = code.lines().filter(|l| !l.trim_start().starts_with("//")).map(|l| l.split(" // ").next().unwrap()).collect();
            texts.push((name.to_string(), code.join("\n")));
        }
        let ctx = egui::Context::default();
        ctx.begin_pass(egui::RawInput::default());
        let font = egui::FontId::proportional(14.0);
        ctx.fonts_mut(|fonts| {
            for (name, text) in &texts {
                let missing: Vec<char> = text.chars().filter(|c| !c.is_control() && !fonts.has_glyph(&font, *c)).collect();
                assert!(missing.is_empty(), "{name}: no glyph for {missing:?}");
            }
        });
    }

    /// Every action says its label, and what it does, in every language.
    #[test]
    fn every_action_has_a_russian_text() {
        let ru = I18n::of("ru");
        for a in &ACTIONS {
            assert_ne!(ru.label(a.act), a.label, "{} has no Russian label", a.name);
            if !a.about.is_empty() {
                assert_ne!(ru.about(a.act), a.about, "{} has no Russian text of what it does", a.name);
            }
        }
    }

    /// Every text the window asks for by key has one in each language, and no catalog line
    /// stays when nothing asks for it.
    #[test]
    fn the_catalog_and_the_code_agree() {
        let sources = [
            include_str!("app.rs"),
            include_str!("sidebar.rs"),
            include_str!("dock.rs"),
            include_str!("table.rs"),
            include_str!("cube.rs"),
            include_str!("draw.rs"),
            include_str!("pattern.rs"),
            include_str!("templates.rs"),
            include_str!("constrain.rs"),
        ];
        let mut asked: Vec<&str> = sources
            .iter()
            .flat_map(|source| source.split(".t(").skip(1).chain(source.split(".f(").skip(1)))
            .filter_map(|rest| rest.trim_start().strip_prefix('"')?.split('"').next())
            .collect();
        // The names of measured values, by key in a table (`Quantity::text`).
        asked.extend(include_str!("measure.rs").split("(\"").skip(1).filter_map(|rest| rest.split('"').next()).filter(|k| k.starts_with("measure-")));
        asked.sort_unstable();
        asked.dedup();
        for (code, _, text) in CATALOGS {
            let have = parse(text);
            for k in &asked {
                assert!(have.contains_key(*k), "`{k}` has no text in {code}");
            }
            for k in have.keys().filter(|k| !k.starts_with("act-")) {
                assert!(asked.contains(&k.as_str()), "`{k}` in {code} is asked by nothing");
            }
            for k in have.keys().filter(|k| k.starts_with("act-")) {
                let known = ACTIONS.iter().any(|a| [key(a.act, ""), key(a.act, "-about")].contains(k));
                assert!(known, "`{k}` in {code} is no action's");
            }
        }
    }
}
