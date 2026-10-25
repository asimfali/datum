//! STEP export of the reference models (D130, D147): each writes; what STEP has no exact type
//! for is refused with the features that bring it in unless approximating is agreed to, and then
//! told of in a note beside the file.
#![allow(clippy::unwrap_used)]

use std::path::Path;

use datum_graph::Evaluator;

#[test]
fn reference_models_export_or_say_why_not() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/models");
    let out = std::env::temp_dir().join(format!("datum-step-{}", std::process::id()));
    std::fs::create_dir_all(&out).unwrap();
    let mut files: Vec<_> = std::fs::read_dir(root).unwrap().map(|e| e.unwrap().path()).collect();
    files.sort();
    for path in files {
        let g = datum_model::parse(&std::fs::read_to_string(&path).unwrap()).unwrap();
        let e = Evaluator::default().evaluate(&g);
        let name = path.file_stem().unwrap().to_string_lossy().into_owned();
        let step = out.join(format!("{name}.step"));
        let note = out.join(format!("{name}.step.txt"));
        // Round holes and slots in a rolled sheet: circles of the development, not circles in 3D.
        let inexact: &[&str] = match name.as_str() {
            "rolled_perforation" => &["perforated", "slotted"],
            "rolled_holes" => &["perforated"],
            _ => &[],
        };
        assert_eq!(datum_model::inexact_features(&g, &e), inexact, "{name}");
        if inexact.is_empty() {
            assert_eq!(datum_model::write_step(&g, &e, &step, false), Ok(0), "{name}");
            assert!(!note.exists(), "{name}: a note for exact geometry");
        } else {
            let err = datum_model::write_step(&g, &e, &step, false).unwrap_err();
            assert!(err.starts_with(&format!("`{}`", inexact[0])), "{err}");
            assert!(datum_model::write_step(&g, &e, &step, true).unwrap() > 0, "{name}");
            let told = std::fs::read_to_string(&note).unwrap();
            assert!(told.contains(&format!("Features: {}.", inexact.join(", "))), "{told}");
        }
        let text = std::fs::read_to_string(&step).unwrap();
        assert!(text.starts_with("ISO-10303-21;") && text.ends_with("END-ISO-10303-21;\n"), "{name}");
    }
    std::fs::remove_dir_all(&out).unwrap();
}

/// A note goes once the geometry is exact again: it would tell of splines no longer there.
#[test]
fn an_old_note_goes() {
    let model = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/models/rolled_holes.json");
    let src = std::fs::read_to_string(model).unwrap();
    let g = datum_model::parse(&src).unwrap();
    let out = std::env::temp_dir().join(format!("datum-step-note-{}", std::process::id()));
    std::fs::create_dir_all(&out).unwrap();
    let (step, note) = (out.join("part.step"), out.join("part.step.txt"));
    let e = Evaluator::default().evaluate(&g);
    datum_model::write_step(&g, &e, &step, true).unwrap();
    assert!(note.exists());
    // Without the cut: the sheet alone.
    let mut json: serde_json::Value = serde_json::from_str(&src).unwrap();
    json["features"].as_array_mut().unwrap().truncate(2);
    let g = datum_model::parse(&json.to_string()).unwrap();
    let e = Evaluator::default().evaluate(&g);
    assert_eq!(datum_model::write_step(&g, &e, &step, false), Ok(0));
    assert!(!note.exists());
    std::fs::remove_dir_all(&out).unwrap();
}
