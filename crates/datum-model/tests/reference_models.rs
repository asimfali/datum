//! Reference models in `tests/models`: every feature builds, validates, and matches its pinned
//! hash in `tests/model-hashes.json`. `DATUM_BLESS=1 cargo test` rewrites the pins after an
//! intended change.
#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::{collections::BTreeMap, path::Path};

use datum_graph::Evaluator;

type Pins = BTreeMap<String, BTreeMap<String, String>>;

#[test]
fn reference_models_match_pinned_hashes() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests");
    let mut files: Vec<_> = std::fs::read_dir(root.join("models")).unwrap().map(|e| e.unwrap().path()).collect();
    files.sort();
    let mut actual = Pins::new();
    for path in files {
        let doc = datum_model::parse_doc(&std::fs::read_to_string(&path).unwrap()).unwrap();
        let evaluator = Evaluator::default();
        // The root, then each level (D152), pinned as `<file> [<level>]`.
        for (level, g) in doc.graphs() {
            let g = g.unwrap_or_else(|err| panic!("{}: {err}", path.display()));
            let e = evaluator.evaluate(&g);
            let mut model = BTreeMap::new();
            for (node, r) in g.nodes().iter().zip(e.results) {
                let b = r.unwrap_or_else(|err| panic!("{}: {level:?}: {}: {err}", path.display(), node.name));
                if let Some(body) = b.body() {
                    body.validate().unwrap();
                }
                model.insert(node.name.clone(), b.hash().iter().map(|x| format!("{x:02x}")).collect());
            }
            let file = path.file_name().unwrap().to_string_lossy().into_owned();
            actual.insert(level.map_or_else(|| file.clone(), |l| format!("{file} [{l}]")), model);
        }
    }
    let pins = root.join("model-hashes.json");
    if std::env::var_os("DATUM_BLESS").is_some() {
        std::fs::write(&pins, serde_json::to_string_pretty(&actual).unwrap() + "\n").unwrap();
        return;
    }
    let expected: Pins = serde_json::from_str(&std::fs::read_to_string(&pins).expect("run with DATUM_BLESS=1 to create the pins")).unwrap();
    assert_eq!(actual, expected, "a reference model changed; if intended, rerun with DATUM_BLESS=1");
}
