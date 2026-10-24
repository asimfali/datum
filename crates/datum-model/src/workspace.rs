//! The folder work is kept in (D115, D133, D141): the project of a model (the nearest folder
//! above it with [`PROJECT_FILE`]), or else the folder beside the program. Files of a kind lie
//! flat in the folder named after it: models in `part`, elements in `library`, exports in
//! `dxf`, `step`, `pdf`; a kind nobody listed (a macro's `stl`) goes the same way. A project may
//! name a kind's folder otherwise, for every engineer in it:
//!
//! ```json
//! {"folders": {"part": "Детали", "stl": "3D-печать"}}
//! ```

use std::path::{Path, PathBuf};

use serde_json::Value;

/// The file that makes a folder a project (VISION §6).
pub const PROJECT_FILE: &str = "datum-project.json";

/// The project the model file `model` belongs to.
pub fn project_root(model: &Path) -> Option<PathBuf> {
    model.ancestors().skip(1).find(|d| d.join(PROJECT_FILE).is_file()).map(Path::to_path_buf)
}

/// The workspace of the model at `model`: its project, or else the folder beside the program.
pub fn root(model: Option<&Path>) -> Option<PathBuf> {
    model.and_then(project_root).or_else(|| std::env::current_exe().ok()?.parent().map(Path::to_path_buf))
}

/// The folder for files of `kind` in the workspace of the model at `model`, not made.
pub fn folder(kind: &str, model: Option<&Path>) -> Result<PathBuf, String> {
    let root = root(model).ok_or("no folder to work in: neither a project nor the program's")?;
    folder_in(&root, kind)
}

/// The folder for files of `kind` in workspace `root`: `kind`, or the name its project gives it.
pub fn folder_in(root: &Path, kind: &str) -> Result<PathBuf, String> {
    let file = root.join(PROJECT_FILE);
    let Ok(text) = std::fs::read_to_string(&file) else { return Ok(root.join(kind)) };
    let project: Value = serde_json::from_str(&text).map_err(|e| format!("{}: {e}", file.display()))?;
    match &project["folders"][kind] {
        Value::Null => Ok(root.join(kind)),
        Value::String(name) if one_name(name) => Ok(root.join(name)),
        other => Err(format!("{}: folders.{kind} is {other}, not one folder name (no path)", file.display())),
    }
}

/// Whether `name` is one folder's name, no path: the layout stays flat.
fn one_name(name: &str) -> bool {
    let mut parts = Path::new(name).components();
    matches!((parts.next(), parts.next()), (Some(std::path::Component::Normal(_)), None)) && !name.contains(['/', '\\'])
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    /// A project named `tag` whose project file says `folders`.
    fn project(tag: &str, folders: &str) -> PathBuf {
        let root = std::env::temp_dir().join(format!("datum-workspace-{tag}"));
        let _ = std::fs::remove_dir_all(&root);
        std::fs::create_dir_all(&root).unwrap();
        std::fs::write(root.join(PROJECT_FILE), folders).unwrap();
        root
    }

    #[test]
    fn a_kind_names_its_folder() {
        let root = project("empty", "{}");
        assert_eq!(folder_in(&root, "dxf").unwrap(), root.join("dxf"));
        assert_eq!(folder_in(&root, "stl").unwrap(), root.join("stl"), "a kind nobody listed");
        let bare = std::env::temp_dir().join("datum-workspace-no-project");
        assert_eq!(folder_in(&bare, "part").unwrap(), bare.join("part"), "no project file: the defaults");
    }

    #[test]
    fn the_project_renames() {
        let root = project("renamed", r#"{"folders": {"part": "Детали", "stl": "3D-печать"}}"#);
        assert_eq!(folder_in(&root, "part").unwrap(), root.join("Детали"));
        assert_eq!(folder_in(&root, "stl").unwrap(), root.join("3D-печать"));
        assert_eq!(folder_in(&root, "dxf").unwrap(), root.join("dxf"));
    }

    /// Only a name: a path, an empty name, `..` or not a string is refused, never guessed.
    #[test]
    fn only_a_name() {
        for (k, bad) in [r#""export/stl""#, r#""export\\stl""#, r#""""#, r#""..""#, r#""/abs""#, "3"].into_iter().enumerate() {
            let root = project(&format!("bad{k}"), &format!(r#"{{"folders": {{"stl": {bad}}}}}"#));
            let e = folder_in(&root, "stl").unwrap_err();
            assert!(e.contains("not one folder name"), "{bad}: {e}");
        }
        let root = project("broken", "{not json");
        assert!(folder_in(&root, "part").is_err(), "a broken project file is said");
    }
}
