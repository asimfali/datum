//! A sheet's flat blank holds the material of its body: at K-factor 0.5 the neutral layer lies
//! mid-thickness, so `blank area x thickness == body volume` (walls are prisms, bends and radial
//! cuts integrate to the mid-layer). Checked on every body of every reference model that unfolds,
//! independent of the pinned hashes.
#![allow(clippy::unwrap_used, clippy::panic)]

use std::path::Path;

use datum_brep::{Body, Surface};
use datum_graph::Evaluator;
use datum_math::Vec2;
use datum_ops::{FlatPiece, sheet_sides, unfold_side};

/// Chords per arc: the polygon of an arc is short of its segment by about (2 pi / n)^2 / 6.
const CHORDS: usize = 4096;
/// Where pieces meet, in millimetres.
const JOIN: f64 = 1e-6;

/// A piece as points, arcs counterclockwise; None for a circle (a hole by itself).
fn points(p: &FlatPiece) -> Option<Vec<Vec2>> {
    Some(match p {
        FlatPiece::Line(a, b) => vec![*a, *b],
        FlatPiece::Polyline(pts) => pts.clone(),
        FlatPiece::Arc { center, radius, start, end } => {
            let mut sweep = end - start;
            if sweep <= 0.0 {
                sweep += std::f64::consts::TAU;
            }
            (0..=CHORDS)
                .map(|i| {
                    let a = start + sweep * i as f64 / CHORDS as f64;
                    Vec2::new(center.x + radius * libm::cos(a), center.y + radius * libm::sin(a))
                })
                .collect()
        }
        FlatPiece::Circle { .. } => return None,
    })
}

fn near(a: Vec2, b: Vec2) -> bool {
    (a.x - b.x).abs() <= JOIN && (a.y - b.y).abs() <= JOIN
}

/// The absolute shoelace area of a closed cycle of points.
fn shoelace(cycle: &[Vec2]) -> f64 {
    let mut s = 0.0;
    for (i, a) in cycle.iter().enumerate() {
        let b = cycle[(i + 1) % cycle.len()];
        s += a.x * b.y - b.x * a.y;
    }
    s.abs() / 2.0
}

/// The areas of the closed cycles that `pieces` chain into, end to end in either direction.
fn cycle_areas(pieces: &[&FlatPiece]) -> Result<Vec<f64>, String> {
    let mut pool: Vec<Option<Vec<Vec2>>> = Vec::new();
    let mut areas = Vec::new();
    for p in pieces {
        match points(p) {
            Some(pts) => pool.push(Some(pts)),
            None => {
                let FlatPiece::Circle { radius, .. } = p else { unreachable!() };
                areas.push(std::f64::consts::PI * radius * radius);
            }
        }
    }
    while let Some(i) = pool.iter().position(Option::is_some) {
        let mut cycle = pool[i].take().unwrap();
        let first = cycle[0];
        while !near(*cycle.last().unwrap(), first) {
            let end = *cycle.last().unwrap();
            let Some((j, rev)) = pool.iter().enumerate().find_map(|(j, p)| {
                let p = p.as_ref()?;
                if near(p[0], end) {
                    Some((j, false))
                } else if near(*p.last().unwrap(), end) {
                    Some((j, true))
                } else {
                    None
                }
            }) else {
                return Err(format!("open chain at ({}, {})", end.x, end.y));
            };
            let mut p = pool[j].take().unwrap();
            if rev {
                p.reverse();
            }
            cycle.extend(p.into_iter().skip(1));
        }
        cycle.pop();
        areas.push(shoelace(&cycle));
    }
    Ok(areas)
}

/// The thickness: the two sides' main faces may lie on different walls, so for planes the least
/// distance from the front's plane to a parallel plane of the body, for cylinders the radii apart.
fn thickness(body: &Body) -> Result<f64, String> {
    let [a, b] = sheet_sides(body).map_err(|e| e.to_string())?;
    match (body.faces()[a as usize].surface, body.faces()[b as usize].surface) {
        (Surface::Plane(p), Surface::Plane(_)) => body
            .faces()
            .iter()
            .filter_map(|f| match f.surface {
                Surface::Plane(q) if p.normal.cross(q.normal).dot(p.normal.cross(q.normal)) <= 1e-18 => {
                    Some((q.origin - p.origin).dot(p.normal).abs() / p.normal.dot(p.normal).sqrt())
                }
                _ => None,
            })
            .filter(|&d| d > 1e-9)
            .min_by(f64::total_cmp)
            .ok_or_else(|| "no parallel plane".to_owned()),
        (Surface::Cylinder(p), Surface::Cylinder(q)) => Ok((p.radius - q.radius).abs()),
        _ => Err("sides neither planes nor cylinders".into()),
    }
}

/// The area of the blank: the outer cycle less the holes.
fn blank_area(body: &Body) -> Result<f64, String> {
    let flat = unfold_side(body, false, 0.5).map_err(|e| e.to_string())?;
    let outer: Vec<&FlatPiece> = flat.pieces[flat.outer.clone()].iter().collect();
    let holes: Vec<&FlatPiece> = flat.pieces.iter().enumerate().filter(|(i, _)| !flat.outer.contains(i)).map(|(_, p)| p).collect();
    let outer = cycle_areas(&outer)?;
    if outer.len() != 1 {
        return Err(format!("outer outline is {} cycles", outer.len()));
    }
    Ok(outer[0] - cycle_areas(&holes)?.iter().sum::<f64>())
}

#[test]
fn blank_area_times_thickness_is_body_volume() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/models");
    let mut files: Vec<_> = std::fs::read_dir(root).unwrap().map(|e| e.unwrap().path()).collect();
    files.sort();
    let (mut checked, mut skipped) = (0, Vec::new());
    for path in files {
        let file = path.file_name().unwrap().to_string_lossy().into_owned();
        let doc = datum_model::parse_doc(&std::fs::read_to_string(&path).unwrap()).unwrap();
        let evaluator = Evaluator::default();
        for (level, g) in doc.graphs() {
            let g = g.unwrap();
            let e = evaluator.evaluate(&g);
            // Nodes made of sheet metal: the sheet ops, and the cuts and trims on them.
            let mut sheet = vec![false; g.nodes().len()];
            for (i, (node, r)) in g.nodes().iter().zip(e.results).enumerate() {
                let kind = serde_json::to_value(&node.op).unwrap()["op"].as_str().unwrap().to_owned();
                sheet[i] = match kind.as_str() {
                    "sheet" | "plate" | "flange" => true,
                    "extrude" | "trim" | "branch_hole" | "saddle" => node.inputs.first().is_some_and(|&n| sheet[n as usize]),
                    _ => false,
                };
                if !sheet[i] {
                    continue;
                }
                let Some(body) = r.unwrap().body().cloned() else { continue };
                let name = format!("{file} {level:?} {}", node.name);
                if let Err(err) = unfold_side(&body, false, 0.5) {
                    skipped.push(format!("{name}: {err}"));
                    continue;
                }
                let t = match thickness(&body) {
                    Ok(t) => t,
                    Err(err) => {
                        skipped.push(format!("{name}: {err}"));
                        continue;
                    }
                };
                let area = blank_area(&body).unwrap_or_else(|err| panic!("{name}: {err}"));
                let volume = body.volume();
                assert!(
                    (area * t - volume).abs() <= 1e-3 * volume,
                    "{name}: blank area {area} x thickness {t} = {} but body volume {volume}",
                    area * t
                );
                checked += 1;
            }
        }
    }
    eprintln!("checked {checked}, skipped {}", skipped.len());
    for s in &skipped {
        eprintln!("  skipped {s}");
    }
    assert!(checked >= 15, "only {checked} bodies checked");
}
