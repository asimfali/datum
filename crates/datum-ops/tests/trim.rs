//! A rolled shell trimmed by a work plane (D211): the cut a wave in the development, its wall
//! radial, the blank with a polyline and DXF R2000, STEP only approximated, refusals.
#![allow(clippy::unwrap_used, clippy::panic)]

use std::f64::consts::{PI, TAU};

use datum_brep::{Body, BrepError, Surface, TopoId, Wave, step, tessellate};
use datum_math::Vec3;
use datum_ops::SketchPlane;
use datum_ops::{Flat, FlatPiece, Op, POLYLINE_DEVIATION, Value, unfold_side};
use serde_json::{Value as Json, json};

const T: f64 = 2.0;
const K: f64 = 0.44;
const GAP: f64 = 2.0;
/// Length of the shell along the axis.
const L: f64 = 100.0;

fn id(tag: &str) -> TopoId {
    TopoId::derive(tag).finish()
}

fn op(json: Json) -> Op {
    serde_json::from_value(json).unwrap()
}

fn close(a: f64, b: f64, rel: f64) -> bool {
    (a - b).abs() <= rel * b.abs().max(1.0)
}

/// The axis along X from the origin, and the line `p` to `q`.
fn sketch(p: [f64; 2], q: [f64; 2]) -> Json {
    json!({
        "points": {"a0": [0, 0], "a1": [10, 0], "p": p, "q": q},
        "lines": {"ax": {"from": "a0", "to": "a1", "construction": true}, "path": {"from": "p", "to": "q"}}
    })
}

fn rolled_from(sketch: Json, side: &str) -> Body {
    let s = op(json!({"op": "sketch", "sketch": sketch})).eval(id("profile"), &[]).unwrap();
    let roll = json!({"op": "rolled", "axis": "ax", "side": side, "gap": GAP, "thickness": T, "k_factor": K});
    op(roll).eval(id("roll"), &[&s]).unwrap().body().unwrap().clone()
}

/// The shell of the example: a line at 50 from the axis, 100 long, on `side` of it.
fn shell(side: &str) -> Body {
    rolled_from(sketch([0.0, 50.0], [L, 50.0]), side)
}

/// The plane through `(x, 0, 0)` leaning by `deg` from the one square to the axis, its normal
/// towards +X.
fn plane(x: f64, deg: f64) -> SketchPlane {
    let (s, c) = (libm::sin(deg.to_radians()), libm::cos(deg.to_radians()));
    SketchPlane { origin: Vec3::new(x, 0.0, 0.0), x_axis: Vec3::new(-s, c, 0.0), normal: Vec3::new(c, s, 0.0) }
}

fn trim_op(flip: bool, k: Option<f64>) -> Op {
    op(json!({"op": "trim", "flip": flip, "k_factor": k}))
}

fn trim_by(b: &Body, p: &SketchPlane, flip: bool) -> Result<Body, BrepError> {
    let r = trim_op(flip, Some(K)).eval(id("trim"), &[&Value::Body(b.clone()), &Value::Plane(*p)])?.body().unwrap().clone();
    r.validate().unwrap();
    Ok(r)
}

/// `trim_by`, and the mesh of the result.
fn trimmed(b: &Body, p: &SketchPlane, flip: bool) -> Body {
    let r = trim_by(b, p, flip).unwrap();
    tessellate(&r).unwrap();
    r
}

fn blank(b: &Body) -> Flat {
    unfold_side(b, false, K).unwrap()
}

/// The wave of `plane(x, deg)` on a shell of inner radius `r_in`, the gap's width `GAP / N`, and
/// the angle `phi` the sheet turns through.
///
/// The plane `n·p = n·o` with `n = (cos t, sin t, 0)` meets the neutral cylinder
/// `p = (v, N·cos u/N, N·sin u/N)`-ish (angle `θ = u/N` from the seam's bisector turned by half
/// the gap) where `v·cos t + N·cos θ'·sin t = x·cos t`, so `v = x − N·tan t·cos θ'` with `θ'`
/// counted from the axis `y`. The sheet starts half a gap after the bisector: `θ' = θ + g/2`
/// reads `v = a + b·cos θ + c·sin θ` with `a = x`, `b = −N·tan t·cos(g/2)`,
/// `c = N·tan t·sin(g/2)`.
fn wave_of(x: f64, deg: f64, r_in: f64) -> (Wave, f64, f64) {
    let n = r_in + K * T;
    let g = GAP / n;
    let tan = libm::tan(deg.to_radians());
    (Wave { a: x, b: -n * tan * libm::cos(g / 2.0), c: n * tan * libm::sin(g / 2.0) }, n, TAU - g)
}

/// What is left of the shell on the side the wave lies above (the `-v` end): the sheet is
/// a ring of `(Rout² − Rin²) / 2` per radian, `L(θ) = v(θ)` long, so the volume is
/// `(Rout² − Rin²) / 2 · I` with `I = ∫₀^Φ v dθ = a·Φ + b·sin Φ − c·(cos Φ − 1)`.
fn kept_volume(x: f64, deg: f64, r_in: f64) -> f64 {
    let (w, _, phi) = wave_of(x, deg, r_in);
    let i = w.a * phi + w.b * libm::sin(phi) - w.c * (libm::cos(phi) - 1.0);
    ((r_in + T) * (r_in + T) - r_in * r_in) / 2.0 * i
}

fn whole_volume(r_in: f64) -> f64 {
    let (_, _, phi) = wave_of(0.0, 0.0, r_in);
    ((r_in + T) * (r_in + T) - r_in * r_in) / 2.0 * phi * L
}

fn walls(b: &Body) -> Vec<&Surface> {
    b.faces().iter().map(|f| &f.surface).filter(|s| matches!(s, Surface::Wall(_))).collect()
}

fn side(b: &Body, convex: bool) -> String {
    let f = b.faces().iter().find(|f| matches!(f.surface, Surface::Cylinder(c) if c.convex == convex)).unwrap();
    format!("{:?}", f.id)
}

fn mesh_volume(b: &Body) -> f64 {
    let m = tessellate(b).unwrap();
    let o = m.positions[0];
    m.triangles
        .iter()
        .map(|t| (m.positions[t[0] as usize] - o).dot((m.positions[t[1] as usize] - o).cross(m.positions[t[2] as usize] - o)))
        .sum::<f64>()
        / 6.0
}

#[test]
fn the_example_is_a_wave_cut() {
    let b = trimmed(&shell("left"), &plane(70.0, 20.0), false);
    assert!(close(b.volume(), 44655.52786353672, 1e-9), "{}", b.volume());
    assert!(close(b.volume(), kept_volume(70.0, 20.0, 50.0), 1e-9), "{} vs {}", b.volume(), kept_volume(70.0, 20.0, 50.0));
    let w = walls(&b);
    assert_eq!(w.len(), 1);
    assert!(matches!(w[0], Surface::Wall(r) if r.trace.is_some()));
    // The chords of the mesh lose a little of a round section.
    assert!(close(mesh_volume(&b), b.volume(), 3e-3), "{} vs {}", mesh_volume(&b), b.volume());
}

#[test]
fn the_blank_has_a_polyline_for_the_cut() {
    let b = trimmed(&shell("left"), &plane(70.0, 20.0), false);
    let f = blank(&b);
    assert!(close(f.size.x, 317.68846842929736, 1e-12), "{:?}", f.size);
    assert!(close(f.size.y, 88.5188, 1e-5), "{:?}", f.size);
    assert_eq!(f.counts(), [3, 0, 0]);
    assert_eq!(f.polylines(), 1);
    let FlatPiece::Polyline(pts) = f.pieces.iter().find(|p| matches!(p, FlatPiece::Polyline(_))).unwrap() else { unreachable!() };
    let (w, n, _) = wave_of(70.0, 20.0, 50.0);
    // The blank starts at the corner of its box: x is u from the seam, y is v (the end at v = 0
    // is the blank's bottom or its top, whichever side the face is seen from): find which by
    // the ends, then every point lies on the wave, within the deviation.
    let (first, last) = (pts[0], pts[pts.len() - 1]);
    assert!((first.x - last.x).abs() > 300.0, "{first:?} {last:?}");
    let flipped_x = first.x > last.x;
    let (lo, hi) = (pts.iter().map(|p| p.y).fold(f64::INFINITY, f64::min), pts.iter().map(|p| p.y).fold(f64::NEG_INFINITY, f64::max));
    let [wlo, whi] = w.range(0.0, f.size.x, n);
    let amp = (whi - wlo) / 2.0;
    // The exact max/min of the wave are a ± amplitude, in the blank's frame offset by `shift`.
    assert!(close(hi - lo, 2.0 * amp, 1e-4), "{lo} {hi} {amp}");
    for &mirror_y in &[false, true] {
        let ok = pts.iter().all(|p| {
            let u = if flipped_x { f.size.x - p.x } else { p.x };
            let v = w.at(u, n)[0];
            let y = if mirror_y { f.size.y - p.y } else { p.y };
            (y - v).abs() <= POLYLINE_DEVIATION
        });
        if ok {
            return;
        }
    }
    panic!("the polyline does not follow the wave: {:?}", &pts[..3]);
}

#[test]
fn dxf_r2000_with_one_lwpolyline() {
    let f = blank(&trimmed(&shell("left"), &plane(70.0, 20.0), false));
    let d = f.dxf();
    assert_eq!(d.matches("\nLWPOLYLINE\n").count(), 1);
    assert!(d.contains("$ACADVER") && d.contains("AC1015"));
    assert_eq!(d.matches("\nLINE\n").count(), 3);
}

#[test]
fn step_is_approximated_by_consent() {
    let b = trimmed(&shell("left"), &plane(70.0, 20.0), false);
    let s = step(&[&b], "t", true).unwrap();
    assert_eq!(s.approximated, 1);
    assert!(s.deviation < 1e-6, "{}", s.deviation);
    assert!(matches!(step(&[&b], "t", false), Err(BrepError::Unsupported(_))));
}

#[test]
fn flip_keeps_the_other_end() {
    let s = shell("left");
    let kept = trimmed(&s, &plane(70.0, 20.0), false);
    let other = trimmed(&s, &plane(70.0, 20.0), true);
    assert!(close(kept.volume() + other.volume(), whole_volume(50.0), 1e-9), "{} + {}", kept.volume(), other.volume());
    assert!(close(other.volume(), whole_volume(50.0) - 44655.52786353672, 1e-9));
    assert_eq!(walls(&other).len(), 1);
    // Its blank is the other piece of the rectangle: one polyline again.
    assert_eq!(blank(&other).polylines(), 1);
    // The plane turned over cuts the same as `flip`.
    let mut turned = plane(70.0, 20.0);
    turned.normal = Vec3::ZERO - turned.normal;
    let by_normal = trim_by(&s, &turned, false).unwrap();
    assert!(close(by_normal.volume(), other.volume(), 1e-12));
    assert_eq!(by_normal.hash(), other.hash());
}

#[test]
fn a_shell_on_each_side_of_its_line() {
    for (side, r_in) in [("left", 50.0), ("right", 48.0), ("middle", 49.0)] {
        let b = trimmed(&shell(side), &plane(70.0, 20.0), false);
        let want = kept_volume(70.0, 20.0, r_in);
        assert!(close(b.volume(), want, 1e-9), "{side}: {} vs {want}", b.volume());
        assert_eq!(blank(&b).polylines(), 1, "{side}");
    }
}

#[test]
fn the_same_twice_is_the_same() {
    let s = shell("left");
    let a = trim_by(&s, &plane(70.0, 20.0), false).unwrap();
    let b = trim_by(&s, &plane(70.0, 20.0), false).unwrap();
    assert_eq!(a.hash(), b.hash());
    assert_eq!(blank(&a).hash, blank(&b).hash);
    assert_ne!(a.hash(), s.hash());
}

#[test]
fn what_a_trim_refuses() {
    let s = shell("left");
    let unsupported = |r: Result<Body, BrepError>, what: &str| assert!(matches!(r, Err(BrepError::Unsupported(_))), "{what}: {r:?}");
    // Along the axis.
    let along = SketchPlane { origin: Vec3::ZERO, x_axis: Vec3::new(1.0, 0.0, 0.0), normal: Vec3::new(0.0, 1.0, 0.0) };
    unsupported(trim_by(&s, &along, false), "along the axis");
    // Missing the sheet: beyond either end.
    unsupported(trim_by(&s, &plane(200.0, 0.0), false), "past the far end");
    unsupported(trim_by(&s, &plane(-50.0, 0.0), false), "before the near end");
    // Leaving through an end: the wave crosses v = 100 or v = 0.
    unsupported(trim_by(&s, &plane(90.0, 20.0), false), "through the far end");
    unsupported(trim_by(&s, &plane(10.0, 20.0), false), "through the near end");
    unsupported(trim_by(&s, &plane(50.0, 45.0), false), "through both");
    // A cone.
    let cone = rolled_from(sketch([0.0, 50.0], [L, 80.0]), "left");
    unsupported(trim_by(&cone, &plane(50.0, 0.0), false), "a cone");
    // Not a rolled shell.
    let square = json!({"op": "prism", "polygon": [[0, 0], [10, 0], [10, 10], [0, 10]], "z0": 0, "z1": 5});
    let prism = op(square).eval(id("prism"), &[]).unwrap().body().unwrap().clone();
    unsupported(trim_by(&prism, &plane(5.0, 0.0), false), "a prism");
    // No K-factor.
    let no_k = trim_op(false, None).eval(id("t"), &[&Value::Body(s.clone()), &Value::Plane(plane(70.0, 20.0))]);
    assert!(matches!(no_k, Err(BrepError::Invalid(_))), "{no_k:?}");
}

/// A hole of Ø10 at `(u, v)` through the outer face of `b`, in its development.
fn punched(name: &str, b: &Body, u: f64, v: f64) -> Result<Body, BrepError> {
    let target = Value::Body(b.clone());
    let hole = json!({"points": {"h": [u, v]}, "circles": {"hole": {"center": "h", "diameter": 10}}});
    let placed = op(json!({"op": "sketch", "face": side(b, true), "k_factor": K, "sketch": hole})).eval(id(&format!("{name}.sketch")), &[&target])?;
    let cut = json!({"op": "extrude", "mode": "cut", "through": true});
    let r = op(cut).eval(id(name), &[&target, &placed])?.body().unwrap().clone();
    r.validate().unwrap();
    Ok(r)
}

/// The volume a hole of Ø10 takes out of the sheet of inner radius 50.
fn hole_volume() -> f64 {
    let n = 50.0 + K * T;
    25.0 * PI * (52.0 * 52.0 - 50.0 * 50.0) / (2.0 * n)
}

/// The holed sheet does not mesh: a cylinder face of a rolled shell with a hole cannot be
/// triangulated yet (so before the trim as well), and the checks here are the body's.
#[test]
fn a_hole_far_from_the_wave_survives() {
    let s = shell("left");
    let p = plane(70.0, 20.0);
    // Holed first, then trimmed.
    let holed = punched("hole", &s, 100.0, 20.0).unwrap();
    let a = trim_by(&holed, &p, false).unwrap();
    let want = 44655.52786353672 - hole_volume();
    assert!(close(a.volume(), want, 1e-9), "{} vs {want}", a.volume());
    // Trimmed first, then a sketch on the trimmed outer face and a hole through it.
    let t = trimmed(&s, &p, false);
    let b = punched("hole", &t, 100.0, 20.0).unwrap();
    assert!(close(b.volume(), want, 1e-9), "{} vs {want}", b.volume());
    assert_eq!(walls(&b).len(), 1 + walls(&holed).len());
    assert_eq!(blank(&b).counts(), [3, 0, 1]);
    assert_eq!(blank(&b).polylines(), 1);
    assert_eq!(blank(&a).counts(), [3, 0, 1]);
    assert_eq!(blank(&a).polylines(), 1);
    // On screen too: a hole on a shell of nearly a whole turn lies within its face's angles.
    for b in [&holed, &a, &b] {
        tessellate(b).unwrap();
    }
}

#[test]
fn a_hole_cut_off_or_near_the_wave_is_refused() {
    let s = shell("left");
    let p = plane(70.0, 20.0);
    let (w, n, _) = wave_of(70.0, 20.0, 50.0);
    let (u, v) = (250.0, w.at(250.0, n)[0]);
    // Past the wave, in the part that goes.
    let gone = punched("hole", &s, u, 88.0).unwrap();
    assert!(matches!(trim_by(&gone, &p, false), Err(BrepError::Unsupported(_))));
    // Across the wave: the hole reaches its line.
    let across = punched("hole", &s, u, v).unwrap();
    assert!(matches!(trim_by(&across, &p, false), Err(BrepError::Unsupported(_))));
    // Just under it, within the wave's band (kept clear whole).
    let near = punched("hole", &s, u, v - 3.0).unwrap();
    assert!(matches!(trim_by(&near, &p, false), Err(BrepError::Unsupported(_))));
    // Trimmed first: a hole in the band is refused.
    let t = trimmed(&s, &p, false);
    assert!(punched("hole", &t, u, v - 3.0).is_err());
    assert!(punched("hole", &t, 200.0, 50.0).is_err());
}
