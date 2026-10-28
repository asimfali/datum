//! The viewer's geometry without a window: camera, anchored vertices, a headless picture.
#![allow(clippy::unwrap_used, clippy::panic)]

use std::path::Path;

use datum_graph::Evaluator;
use datum_math::Vec3;
use datum_view::camera::Camera;
use datum_view::scene::{Geometry, Scene, TRIANGLE_VERTEX};

fn scene(model: &str) -> Scene {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/models");
    let g = datum_model::parse(&std::fs::read_to_string(root.join(model)).unwrap()).unwrap();
    let e = Evaluator::default().evaluate(&g);
    Scene::new(&e.final_values(&g), &mut Default::default())
}

/// #119: a pattern's copies are drawn with their sketch outside it too, as the profiles they make.
#[test]
fn pattern_copies_are_drawn_with_the_sketch() {
    let model = r#"{"format": "datum/1", "features": [{"name": "s", "op": "sketch", "sketch": {
        "points": {"o": [0, 0]}, "circles": {"hole": {"center": "o", "diameter": 4}},
        "patterns": {"row": {"of": "hole", "count": [3, 1], "step": [[10, 0], [0, 10]]}}}}]}"#;
    let g = datum_model::parse(model).unwrap();
    let e = Evaluator::default().evaluate(&g);
    let s = Scene::new(&e.final_values(&g), &mut Default::default());
    // The circle and its two copies, 64 chords each; the last copy's centre at x = 20.
    assert_eq!(s.sketch_lines.len(), 3 * 64);
    let right = s.sketch_lines.iter().flatten().map(|p| p.x).fold(f64::NEG_INFINITY, f64::max);
    assert!((right - 22.0).abs() < 1e-9, "{right}");
}

/// Clip coordinates of `p` (relative to the anchor) under the column-major matrix `m`.
fn project(m: &[f32; 16], p: [f32; 3]) -> [f32; 3] {
    let c = |r: usize| m[r] * p[0] + m[4 + r] * p[1] + m[8 + r] * p[2] + m[12 + r];
    let w = c(3);
    [c(0) / w, c(1) / w, c(2) / w]
}

#[test]
fn fit_fills_the_view_within_the_depth_range() {
    let s = scene("sketch_plate.json");
    let mut cam = Camera::default();
    cam.fit(s.bounds, 1.5);
    let anchor = s.anchor();
    let (lo, hi) = s.bounds.unwrap();
    let radius = (hi - lo).dot(hi - lo).sqrt() * 0.5;
    let (m, _) = cam.matrices(anchor, 1.5, radius);
    let centre = project(&m, [0.0; 3]);
    assert!(centre[0].abs() < 1e-5 && centre[1].abs() < 1e-5, "{centre:?}");
    // Every corner of the bounds is in view and between the near and far planes, and the
    // farthest one out is near the edge: the part fills the picture.
    let mut widest = 0.0f32;
    for k in 0..8 {
        let c = Vec3::new(if k & 1 == 0 { lo.x } else { hi.x }, if k & 2 == 0 { lo.y } else { hi.y }, if k & 4 == 0 { lo.z } else { hi.z }) - anchor;
        let p = project(&m, [c.x as f32, c.y as f32, c.z as f32]);
        assert!(p.iter().take(2).all(|v| v.abs() <= 1.0) && (0.0..=1.0).contains(&p[2]), "corner {k}: {p:?}");
        widest = widest.max(p[0].abs()).max(p[1].abs());
    }
    assert!(widest > 0.9, "{widest}");
}

/// D24: a site 10⁷ m from the origin goes to the GPU as small numbers, with its millimetres.
#[test]
fn far_models_are_drawn_relative_to_an_anchor() {
    let s = scene("far_site.json");
    let anchor = s.anchor();
    assert!(anchor.x.abs() > 1e6 || anchor.y.abs() > 1e6 || anchor.z.abs() > 1e6, "{anchor:?}");
    let g = Geometry::new(&s, anchor);
    let (lo, hi) = s.bounds.unwrap();
    let half = ((hi - lo) * 0.5).to_array().into_iter().fold(0.0f64, f64::max) as f32;
    for v in g.triangles.vertices.chunks(TRIANGLE_VERTEX as usize) {
        let x = f32::from_le_bytes([v[0], v[1], v[2], v[3]]);
        assert!(x.abs() <= half * 1.0001, "{x} beyond {half}");
    }
}

/// A picture of a real model: the part in the middle, background in the corners. Skipped where
/// wgpu finds no adapter at all (not even a software one).
#[test]
fn headless_picture() {
    let s = scene("sketch_plate.json");
    let mut cam = Camera::default();
    cam.fit(s.bounds, 160.0 / 120.0);
    let (w, h) = (160, 120);
    let rgba = match datum_view::capture::picture(&s, &cam, w, h) {
        Ok(p) => p,
        Err(e) if e.contains("no GPU adapter") => {
            eprintln!("skipping: {e}");
            return;
        }
        Err(e) => panic!("{e}"),
    };
    assert_eq!(rgba.len(), (w * h * 4) as usize);
    let at = |x: u32, y: u32| &rgba[((y * w + x) * 4) as usize..((y * w + x) * 4 + 3) as usize];
    let background = datum_view::render::BACKGROUND.map(|c| (c * 255.0).round() as u8);
    assert_eq!(at(0, 0), background);
    assert_eq!(at(w - 1, h - 1), background);
    assert_ne!(at(w / 2, h / 2), background);
}

/// Standard views put the axes where a drawing has them: top shows +X right and +Y up, front
/// +X right and +Z up, right +Y right and +Z up.
#[test]
fn standard_views_follow_drawing_conventions() {
    let screen = |view: &str, p: [f32; 3]| {
        let cam = Camera::view(view).unwrap();
        let (m, _) = cam.matrices(cam.target, 1.0, 1.0);
        let q = project(&m, p);
        [q[0], q[1]]
    };
    let close = |a: [f32; 2], b: [f32; 2]| assert!((a[0] - b[0]).abs() < 1e-5 && (a[1] - b[1]).abs() < 1e-5, "{a:?} vs {b:?}");
    let right_up = |view: &str, right: [f32; 3], up: [f32; 3]| {
        let (r, u) = (screen(view, right), screen(view, up));
        assert!(r[0] > 0.0 && u[1] > 0.0, "{view}: {r:?} {u:?}");
        close([0.0, r[1]], [0.0, 0.0]);
        close([u[0], 0.0], [0.0, 0.0]);
    };
    right_up("top", [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]);
    right_up("front", [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]);
    right_up("right", [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]);
    right_up("back", [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]);
    right_up("left", [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]);
    assert!(Camera::view("isometric").is_err());
}

/// A model ending in an unfold still shows its sheet: the blank is for export.
#[test]
fn an_unfold_does_not_hide_the_sheet() {
    let s = scene("rolled_perforation.json");
    assert!(!s.bodies.is_empty() && s.bounds.is_some());
}
