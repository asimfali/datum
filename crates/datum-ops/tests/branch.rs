//! A branch pipe set on a main one (D227): the hole in the main shell and the branch's end.
#![allow(clippy::unwrap_used, clippy::panic)]

use std::f64::consts::{FRAC_PI_2, PI};

use datum_brep::{Body, BrepError, Rolled, Surface, TopoId, Trace, step, tessellate};
use datum_math::{Vec2, Vec3};
use datum_ops::{FlatPiece, Op, POLYLINE_DEVIATION, SketchPlane, Value, unfold_side};
use serde_json::{Value as Json, json};

const T: f64 = 2.0;
const K: f64 = 0.44;

fn id(tag: &str) -> TopoId {
    TopoId::derive(tag).finish()
}

fn op(json: Json) -> Op {
    serde_json::from_value(json).unwrap()
}

fn rolled(plane: &str, p: [f64; 2], q: [f64; 2], tag: &str) -> Body {
    let sketch = json!({
        "points": {"a0": [0, 0], "a1": [0, 10], "b0": [-1000, 0], "b1": [1000, 0], "p": p, "q": q},
        "lines": {"ax": {"from": if plane == "XY" { "b0" } else { "a0" }, "to": if plane == "XY" { "b1" } else { "a1" }, "construction": true}, "path": {"from": "p", "to": "q"}}
    });
    let frame = if plane == "XY" {
        json!({"origin": [0, 0, 0], "x_axis": [1, 0, 0], "normal": [0, 0, 1]})
    } else {
        json!({"origin": [0, 0, 0], "x_axis": [0, 1, 0], "normal": [1, 0, 0]})
    };
    let s = op(json!({"op": "sketch", "sketch": sketch, "plane": frame})).eval(id(&format!("{tag}.sketch")), &[]).unwrap();
    let roll = json!({"op": "rolled", "axis": "ax", "side": "left", "gap": 2.0, "thickness": T, "k_factor": K});
    op(roll).eval(id(tag), &[&s]).unwrap().body().unwrap().clone()
}

/// The main pipe along X, 50 from the axis, from -100 to 100; the branch along Z, 20 from it.
fn tee() -> (Body, Body) {
    (rolled("XY", [-100.0, 50.0], [100.0, 50.0], "main"), rolled("YZ", [20.0, 0.0], [20.0, 120.0], "branch"))
}

fn run(kind: &str, a: &Body, b: &Body) -> Result<Body, BrepError> {
    let r = op(json!({"op": kind, "k_factor": K})).eval(id(kind), &[&Value::Body(a.clone()), &Value::Body(b.clone())])?.body().unwrap().clone();
    r.validate().unwrap();
    tessellate(&r).unwrap();
    Ok(r)
}

/// A pipe about the axis from `origin` along the unit `dir`, the sheet's line `radius` from it (the sheet lies inside it,
/// its inner radius `radius - T`),
/// from `from` to `to` along it; `x_axis` is the direction of the sketch's x (square to `dir`).
fn pipe(origin: [f64; 3], x_axis: [f64; 3], dir: [f64; 3], radius: f64, from: f64, to: f64, tag: &str) -> Body {
    let sketch = json!({
        "points": {"a0": [0, 0], "a1": [0, 10], "p": [radius, from], "q": [radius, to]},
        "lines": {"ax": {"from": "a0", "to": "a1", "construction": true}, "path": {"from": "p", "to": "q"}}
    });
    let [x, d] = [x_axis, dir];
    let normal = [x[1] * d[2] - x[2] * d[1], x[2] * d[0] - x[0] * d[2], x[0] * d[1] - x[1] * d[0]];
    let frame = json!({"origin": origin, "x_axis": x_axis, "normal": normal});
    let s = op(json!({"op": "sketch", "sketch": sketch, "plane": frame})).eval(id(&format!("{tag}.sketch")), &[]).unwrap();
    let roll = json!({"op": "rolled", "axis": "ax", "side": "left", "gap": 2.0, "thickness": T, "k_factor": K});
    op(roll).eval(id(tag), &[&s]).unwrap().body().unwrap().clone()
}

/// A branch with the sheet's line `r` from the axis, along Z from `z0` to `z1`, its axis through `(x, y)`.
fn upright(x: f64, y: f64, r: f64, z0: f64, z1: f64) -> Body {
    pipe([x, y, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], r, z0, z1, "branch")
}

fn close(a: f64, b: f64, rel: f64) -> bool {
    (a - b).abs() <= rel * b.abs().max(1.0)
}

/// Composite Simpson's rule of `f` over `[a, b]` in `n` (even) intervals.
fn simpson(f: impl Fn(f64) -> f64, a: f64, b: f64, n: usize) -> f64 {
    let h = (b - a) / n as f64;
    let odd: f64 = (1..n).step_by(2).map(|k| f(a + h * k as f64)).sum();
    let even: f64 = (2..n).step_by(2).map(|k| f(a + h * k as f64)).sum();
    h / 3.0 * (f(a) + f(b) + 4.0 * odd + 2.0 * even)
}

/// The walls of the cut along a meet, in the order of the body's faces.
fn meets(b: &Body) -> Vec<Rolled> {
    b.faces()
        .iter()
        .filter_map(|f| if let Surface::Wall(r) = f.surface { matches!(r.trace, Some(Trace::Meet(_))).then_some(r) } else { None })
        .collect()
}

fn refused(r: Result<Body, BrepError>, what: &str) {
    assert!(matches!(r, Err(BrepError::Unsupported(_))), "{what}: {:?}", r.as_ref().map(|b| b.hash()));
}

fn volume_removed(whole: &Body, cut: &Body) -> f64 {
    whole.volume() - cut.volume()
}

/// Each ring of the sheet is `(Rout² − Rin²) / 2` per radian (D211).
fn ring(r_in: f64) -> f64 {
    ((r_in + T) * (r_in + T) - r_in * r_in) / 2.0
}

#[test]
fn tee_builds() {
    let (main, branch) = tee();
    let holed = run("branch_hole", &main, &branch).unwrap();
    let cut = run("saddle", &branch, &main).unwrap();
    assert!(holed.volume() < main.volume() && cut.volume() < branch.volume());
    assert_eq!(meets(&holed).len(), 4);
    assert_eq!(meets(&cut).len(), 1);
}

/// The branch (the sheet inside the line at 20: inner radius 18, neutral radius `N`) kept by the saddle on the perpendicular tee: along
/// each generatrix at the angle `φ` (from the start of the development, about its axis) it runs
/// from the cut `z(φ)` to the end at 120, where the neutral layer meets the main pipe's outer
/// cylinder `y² + z² = 52²`, `y` the generatrix's, `N·(e(φ)·Y)`. Computed in world coordinates
/// from the frame the wall reports; `φ` over the sheet's span. The sheet is a ring of `ring(20)`
/// per radian, so the kept volume is `ring · ∫(120 − z) dφ`. Simpson over 4000 intervals of a
/// smooth integrand is exact to rounding; the body's volume is exact too, so 1e-9.
#[test]
fn saddle_removes_what_the_integral_says() {
    let (main, branch) = tee();
    let cut = run("saddle", &branch, &main).unwrap();
    let wall = meets(&cut)[0];
    let (d, [u, _]) = (wall.dev, wall.uv_box());
    let n = d.neutral;
    assert!(close(n, 18.0 + K * T, 1e-12));
    let y_axis = d.axis.cross(d.start);
    let [r_main, r_wall] = [52.0, 120.0];
    let z_of = |phi: f64| {
        let y = n * (libm::cos(phi) * d.start.y + libm::sin(phi) * y_axis.y);
        libm::sqrt(r_main * r_main - y * y)
    };
    let kept = ring(18.0) * simpson(|phi| r_wall - z_of(phi), u[0] / n, u[1] / n, 4000);
    assert!(close(cut.volume(), kept, 1e-9), "{} vs {kept}", cut.volume());
    // The span is the sheet: a whole turn less the gap.
    assert!(close(u[1] - u[0], 2.0 * PI * n - 2.0, 1e-12), "{u:?}");
    assert!(close(volume_removed(&branch, &cut), branch.volume() - kept, 1e-9));
}

/// The hole in the outer cylinder of radius 52 by the bore of radius 18 about Z: at the angle `θ`
/// from +Y about X the bore's edge on it has `y = 52·cos θ`, and the hole spans `x` within
/// `±√(324 − y²)`. The walls are radial, so the sheet loses `ring(50) · ∫Δx dθ`, over `θ` where
/// `|y| < 18` on the side of +Z. With `y = 18·sin s`, `dθ = dy / √(52² − y²)`:
/// `∫ 648·cos²s / √(52² − 324·sin²s) ds` over `s` in `[−π/2, π/2]`, smooth (the square roots of
/// the ends are gone), so Simpson is exact to rounding. The body adds its own quadrature of the
/// four curves: 1e-9.
fn hole_integral() -> f64 {
    simpson(|s| 648.0 * libm::cos(s).powi(2) / libm::sqrt(52.0 * 52.0 - 324.0 * libm::sin(s).powi(2)), -FRAC_PI_2, FRAC_PI_2, 4000)
}

#[test]
fn hole_removes_what_the_integral_says() {
    let (main, branch) = tee();
    let holed = run("branch_hole", &main, &branch).unwrap();
    let want = ring(50.0) * hole_integral();
    let got = volume_removed(&main, &holed);
    assert!(close(got, want, 1e-9), "{got} vs {want}");
    // Moved along the main pipe the hole takes out the same.
    let moved = upright(10.0, 0.0, 20.0, 0.0, 120.0);
    let holed = run("branch_hole", &main, &moved).unwrap();
    assert!(close(volume_removed(&main, &holed), want, 1e-9), "{} vs {want}", volume_removed(&main, &holed));
}

#[test]
fn oblique_and_offset_branches_build() {
    let main = rolled("XY", [-100.0, 50.0], [100.0, 50.0], "main");
    let (c, s) = (libm::cos(PI / 3.0), libm::sin(PI / 3.0));
    let cases = [
        ("oblique", pipe([0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [c, 0.0, s], 20.0, 0.0, 140.0, "oblique")),
        ("offset", upright(0.0, 15.0, 20.0, 0.0, 120.0)),
        ("offset the other way", upright(0.0, -15.0, 20.0, 0.0, 120.0)),
        ("oblique and offset", pipe([0.0, 15.0, 0.0], [0.0, 1.0, 0.0], [c, 0.0, s], 15.0, 0.0, 140.0, "both")),
    ];
    for (name, branch) in cases {
        let holed = run("branch_hole", &main, &branch).unwrap();
        let cut = run("saddle", &branch, &main).unwrap();
        assert!(holed.volume() < main.volume() && cut.volume() < branch.volume(), "{name}");
        for (b, pieces) in [(&holed, 4), (&cut, 1)] {
            let walls = meets(b);
            assert_eq!(walls.len(), pieces, "{name}");
            // The blank: a polyline for each piece, of as many chords as the piece asks for, and
            // every vertex and chord middle within the deviation of the exact curve.
            let flat = unfold_side(b, false, K).unwrap();
            let mut chords: Vec<usize> =
                flat.pieces.iter().filter_map(|p| if let FlatPiece::Polyline(v) = p { Some(v.len() - 1) } else { None }).collect();
            let mut want: Vec<usize> = walls.iter().map(|w| w.polyline(POLYLINE_DEVIATION)).collect();
            chords.sort_unstable();
            want.sort_unstable();
            assert_eq!(chords, want, "{name}");
            for p in flat.pieces.iter().filter_map(|p| if let FlatPiece::Polyline(v) = p { Some(v) } else { None }) {
                assert!(walls.iter().any(|w| follows(w, p)), "{name}: a polyline off every cut curve: {:?}", &p[..2]);
            }
        }
    }
}

/// Distance of `q` from the segment `a`-`b`.
fn from_segment(q: Vec2, a: Vec2, b: Vec2) -> f64 {
    let (d, w) = (Vec2::new(b.x - a.x, b.y - a.y), Vec2::new(q.x - a.x, q.y - a.y));
    let t = ((w.x * d.x + w.y * d.y) / (d.x * d.x + d.y * d.y).max(1e-300)).clamp(0.0, 1.0);
    libm::hypot(w.x - t * d.x, w.y - t * d.y)
}

/// Whether the blank's polyline `pts` runs along the exact curve of `w` (in its development,
/// `x` = `u`, `y` = `v`) within the deviation. The blank is that development mirrored in either
/// direction and moved, so for each of the four mirrorings and each end of the curve (the
/// polyline's ends are the curve's, exactly) the polyline is laid on the curve by that end, and
/// every vertex and chord middle must lie within the deviation (and a hair for rounding) of the
/// curve, sampled in 4000 chords of its own, far finer than the polyline's.
fn follows(w: &Rolled, pts: &[Vec2]) -> bool {
    const SAMPLES: usize = 4000;
    let curve: Vec<Vec2> = (0..=SAMPLES).map(|k| w.at(k as f64 / SAMPLES as f64).0).collect();
    let mids: Vec<Vec2> = pts.windows(2).map(|c| Vec2::new(0.5 * (c[0].x + c[1].x), 0.5 * (c[0].y + c[1].y))).collect();
    let first = pts[0];
    [curve[0], curve[SAMPLES]].iter().any(|&end| {
        [[1.0, 1.0], [-1.0, 1.0], [1.0, -1.0], [-1.0, -1.0]].iter().any(|&[sx, sy]| {
            pts.iter().chain(&mids).all(|&q| {
                let q = Vec2::new(end.x + sx * (q.x - first.x), end.y + sy * (q.y - first.y));
                curve.windows(2).map(|c| from_segment(q, c[0], c[1])).fold(f64::INFINITY, f64::min) <= 1.01 * POLYLINE_DEVIATION
            })
        })
    })
}

fn main_pipe() -> Body {
    rolled("XY", [-100.0, 50.0], [100.0, 50.0], "main")
}

/// Both ops on `main` and `branch`: each is refused, the message naming `why`.
fn both_refused(main: &Body, branch: &Body, what: &str, why: &str) {
    for r in [run("branch_hole", main, branch), run("saddle", branch, main)] {
        match r {
            Err(BrepError::Unsupported(m)) => assert!(m.contains(why), "{what}: {m}"),
            Ok(b) => panic!("{what}: built {:?}", b.hash()),
            Err(e) => panic!("{what}: {e:?}"),
        }
    }
}

#[test]
fn what_a_branch_refuses() {
    let main = main_pipe();
    // Parallel to the main pipe.
    both_refused(&main, &pipe([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], 20.0, -50.0, 50.0, "b"), "parallel", "parallel");
    // Wider than the main pipe, or off its side.
    both_refused(&main, &upright(0.0, 0.0, 60.0, 0.0, 120.0), "wider", "past the side");
    both_refused(&main, &upright(0.0, 35.0, 20.0, 0.0, 120.0), "off the side", "past the side");
    // A cone for the main pipe.
    let cone = rolled("XY", [-100.0, 50.0], [100.0, 80.0], "cone");
    both_refused(&cone, &upright(0.0, 0.0, 20.0, 0.0, 120.0), "a cone", "");
}

#[test]
fn a_branch_too_short_to_reach_is_refused() {
    let main = main_pipe();
    // The cut would be at about z = 48 to 52; the sheet ends at 40.
    refused(run("saddle", &upright(0.0, 0.0, 20.0, 0.0, 40.0), &main), "too short");
    // The sheet starts past the main pipe's side.
    refused(run("saddle", &upright(0.0, 0.0, 20.0, 60.0, 120.0), &main), "starts past the main pipe");
}

#[test]
fn a_hole_near_a_seam_an_end_or_a_cut_is_refused() {
    let main = main_pipe();
    // Through the main pipe's end.
    refused(run("branch_hole", &main, &upright(90.0, 0.0, 20.0, 0.0, 120.0)), "past the end");
    // Over a hole already there.
    let holed = run("branch_hole", &main, &upright(0.0, 0.0, 20.0, 0.0, 120.0)).unwrap();
    refused(run("branch_hole", &holed, &upright(5.0, 0.0, 20.0, 0.0, 120.0)), "over a hole");
    // Over the wave of a trim.
    let plane = SketchPlane { origin: Vec3::new(10.0, 0.0, 0.0), x_axis: Vec3::new(0.0, 1.0, 0.0), normal: Vec3::new(1.0, 0.0, 0.0) };
    let trimmed = op(json!({"op": "trim", "flip": false, "k_factor": K}))
        .eval(id("trim"), &[&Value::Body(main.clone()), &Value::Plane(plane)])
        .unwrap()
        .body()
        .unwrap()
        .clone();
    refused(run("branch_hole", &trimmed, &upright(0.0, 0.0, 20.0, 0.0, 120.0)), "over a trim");
}

/// Branches at each of the four sides: at least one meets the seam of the main pipe, and says so.
#[test]
fn a_hole_over_the_seam_is_refused() {
    let main = main_pipe();
    let sides = [
        ([0.0, 1.0, 0.0], [0.0, 0.0, 1.0]),
        ([0.0, -1.0, 0.0], [0.0, 0.0, 1.0]),
        ([0.0, 0.0, 1.0], [1.0, 0.0, 0.0]),
        ([0.0, 0.0, -1.0], [1.0, 0.0, 0.0]),
    ];
    let seams = sides
        .iter()
        .filter(|&&(dir, x)| match run("branch_hole", &main, &pipe([0.0, 0.0, 0.0], x, dir, 20.0, 0.0, 120.0, "side")) {
            Err(BrepError::Unsupported(m)) => {
                assert!(m.contains("seam"), "{dir:?}: {m}");
                true
            }
            Ok(_) => false,
            Err(e) => panic!("{dir:?}: {e:?}"),
        })
        .count();
    assert_eq!(seams, 1);
}

fn built() -> [Body; 2] {
    let (main, branch) = tee();
    [run("branch_hole", &main, &branch).unwrap(), run("saddle", &branch, &main).unwrap()]
}

#[test]
fn the_same_tee_twice_is_the_same() {
    let (a, b) = (built(), built());
    for (x, y) in a.iter().zip(&b) {
        assert_eq!(x.hash(), y.hash());
        let ids = |b: &Body| b.faces().iter().map(|f| f.id).collect::<Vec<_>>();
        assert_eq!(ids(x), ids(y));
    }
    // The cut's faces come from the feature, not from a count: the holed main keeps every id it had
    // and gains four walls whose ids differ from those of another feature's.
    let (main, branch) = tee();
    let other = op(json!({"op": "branch_hole", "k_factor": K}))
        .eval(id("another"), &[&Value::Body(main.clone()), &Value::Body(branch)])
        .unwrap()
        .body()
        .unwrap()
        .clone();
    let main_ids: Vec<_> = main.faces().iter().map(|f| f.id).collect();
    let (ids_a, ids_other): (Vec<_>, Vec<_>) = (a[0].faces().iter().map(|f| f.id).collect(), other.faces().iter().map(|f| f.id).collect());
    assert!(main_ids.iter().all(|i| ids_a.contains(i) && ids_other.contains(i)));
    assert_eq!(ids_a.len(), main_ids.len() + 4);
    let new_a: Vec<_> = ids_a.iter().filter(|i| !main_ids.contains(i)).collect();
    assert!(new_a.iter().all(|i| !ids_other.contains(i)));
    assert_ne!(a[0].hash(), other.hash());
    assert_ne!(a[0].hash(), main.hash());
}

#[test]
fn the_tee_blanks_and_step() {
    let (main, branch) = tee();
    for (name, b) in [("hole", run("branch_hole", &main, &branch).unwrap()), ("saddle", run("saddle", &branch, &main).unwrap())] {
        let f = unfold_side(&b, false, K).unwrap();
        assert_eq!(f.polylines(), meets(&b).len(), "{name}");
        // Curves with no exact STEP type: only by consent (D147).
        assert!(matches!(step(&[&b], "x", false), Err(BrepError::Unsupported(_))), "{name}");
        step(&[&b], "x", true).unwrap();
    }
}

/// A branch drawn at 60° as an engineer draws it: the axis at an angle to a horizontal line,
/// the wall parallel to it 42 off, all by constraints. The solver holds them parallel only to
/// rounding; as far as the sketch can tell they are (D228), so the branch is a shell, not a cone
/// of no slope, and both cuts take it.
#[test]
fn a_branch_parallel_by_the_solver_is_a_shell() {
    let sketch = json!({
        "points": {"o": [0, 0], "x": [5, 9], "h": [-1, 0], "a": [-35, 20], "b": [45, 160]},
        "lines": {"axis": {"from": "o", "to": "x", "construction": true}, "hz": {"from": "o", "to": "h", "construction": true},
                  "wall": {"from": "a", "to": "b"}},
        "constraints": [{"type": "fix", "point": "o"}, {"type": "horizontal", "line": "hz"},
                        {"type": "angle", "lines": ["hz", "axis"], "degrees": 60}, {"type": "parallel", "lines": ["axis", "wall"]},
                        {"type": "point_line", "point": "a", "line": "axis", "value": 42}]
    });
    let plane = json!({"origin": [0, 0, 0], "x_axis": [1, 0, 0], "normal": [0, -1, 0]});
    let s = op(json!({"op": "sketch", "sketch": sketch, "plane": plane})).eval(id("tilted.sketch"), &[]).unwrap();
    let roll = json!({"op": "rolled", "axis": "axis", "path": "wall", "side": "left", "gap": 2.0, "thickness": T, "k_factor": K});
    let branch = op(roll).eval(id("tilted"), &[&s]).unwrap().body().unwrap().clone();
    assert!(branch.faces().iter().all(|f| !matches!(f.surface, datum_brep::Surface::Cone(_))), "a cone of no slope");
    let main = rolled("XY", [-200.0, 50.0], [200.0, 50.0], "main");
    run("saddle", &branch, &main).unwrap();
    run("branch_hole", &main, &branch).unwrap();
}
