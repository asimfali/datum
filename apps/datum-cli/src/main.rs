//! `datum`: the Datum program — its window (a double click), the command line and the MCP
//! server, in one file (D127).

use std::{
    fs::File,
    io::BufWriter,
    path::{Path, PathBuf},
    process::ExitCode,
    time::Instant,
};

use datum_brep::{Body, Mesh, tessellate};
use datum_graph::{Evaluator, Graph, Value};
use datum_math::expr::Num;

mod export;

const USAGE: &str = "usage:
  datum build <model.json> [--out <file.step|file.stl|file.obj> [--approximate]]
                                                        evaluate a model, export its final bodies;
                                                        --approximate: STEP writes what it has no
                                                        exact type for as B-splines (D147)
  datum dxf <model.json> --out <file.dxf> [--feature <name>]
                                                        flat blank of an unfold feature as DXF (R12)
  datum determinism-check <model.json|dir>...           same result on 1, 4 and 16 threads (C4)
  datum view <model.json>                               window: the model in 3D, parameters to edit
  datum <model.json>                                    the same (a model dropped on datum.exe)
  datum                                                 the viewer without a model: open one from it
  datum snapshot <model.json> --out <file.png> [--size WxH] [--view iso|top|bottom|front|back|left|right]
                                                        picture of the final bodies and sketches
  datum bench-view <model.json> [--size WxH] [--frames N] [--set name=value]...
                                                        GPU time per frame, whole model and close up
  datum mcp                                             MCP server on stdin/stdout
threads: RAYON_NUM_THREADS (default: all cores)";

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match run(&args) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("datum: {e}");
            ExitCode::FAILURE
        }
    }
}

/// The window, on the model at `path` if given. Started by a double click, the console Windows
/// made for it goes; why the window could not open is then said in a message box too.
fn window(path: Option<PathBuf>) -> Result<(), String> {
    #[cfg(windows)]
    let_go_of_own_console();
    datum_view::run(path).inspect_err(|e| datum_view::log::fail(&format!("Datum could not open its window: {e}")))
}

/// Lets go of the console if this program is alone in it: the one Windows made for a double
/// click, which then closes (after a flash). A console shared with a shell (`cmd`) stays, and
/// what is printed goes there (D127).
#[cfg(windows)]
#[allow(unsafe_code)]
fn let_go_of_own_console() {
    #[link(name = "kernel32")]
    unsafe extern "system" {
        fn GetConsoleProcessList(list: *mut u32, count: u32) -> u32;
        fn FreeConsole() -> i32;
    }
    let mut list = [0u32; 2];
    // SAFETY: `list` holds the two IDs it is said to; `FreeConsole` takes nothing. Both only ask
    // the system, and a failure leaves the console as it was.
    unsafe {
        if GetConsoleProcessList(list.as_mut_ptr(), 2) == 1 {
            FreeConsole();
        }
    }
}

fn run(args: &[String]) -> Result<(), String> {
    match args.split_first() {
        Some((cmd, rest)) if cmd == "build" => cmd_build(rest),
        Some((cmd, rest)) if cmd == "dxf" => cmd_dxf(rest),
        Some((cmd, rest)) if cmd == "determinism-check" => cmd_determinism(rest),
        Some((cmd, rest)) if cmd == "snapshot" => cmd_snapshot(rest),
        Some((cmd, rest)) if cmd == "bench-view" => cmd_bench_view(rest),
        Some((cmd, [model])) if cmd == "view" => window(Some(PathBuf::from(model))),
        // A model dropped on the program, or opened with it: show it.
        Some((model, [])) if model.ends_with(".json") && Path::new(model).is_file() => window(Some(PathBuf::from(model))),
        // Started without arguments (a double click): the viewer, with models to open from it.
        None => window(None),
        Some((cmd, [])) if cmd == "mcp" => {
            datum_mcp::Server::default().serve(std::io::stdin().lock(), std::io::stdout().lock()).map_err(|e| e.to_string())
        }
        _ => Err(USAGE.into()),
    }
}

fn load(path: &Path) -> Result<Graph, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    datum_model::parse(&text).map_err(|e| format!("{}: {e}", path.display()))
}

fn hex8(h: [u8; 32]) -> String {
    h[..4].iter().map(|b| format!("{b:02x}")).collect()
}

fn cmd_build(args: &[String]) -> Result<(), String> {
    let (model, out, approximate) = match args {
        [m] => (m, None, false),
        [m, flag, o] if flag == "--out" => (m, Some(Path::new(o)), false),
        [m, flag, o, a] if flag == "--out" && a == "--approximate" => (m, Some(Path::new(o)), true),
        _ => return Err(USAGE.into()),
    };
    let g = load(Path::new(model))?;
    let start = Instant::now();
    let e = Evaluator::default().evaluate(&g);
    let elapsed = start.elapsed();
    let width = g.nodes().iter().map(|n| n.name.len()).max().unwrap_or(0).max(7);
    println!("{:width$}  {:>16}  hash", "feature", "volume");
    for (node, r) in g.nodes().iter().zip(&e.results) {
        match r {
            Ok(v) => match (v.body(), v.flat()) {
                (Some(b), _) => println!("{:width$}  {:>16.6}  {}", node.name, b.volume(), hex8(v.hash())),
                (_, Some(f)) => println!("{:width$}  {:>16}  {}", node.name, format!("flat {:.1}×{:.1}", f.size.x, f.size.y), hex8(v.hash())),
                _ if v.plane().is_some() => println!("{:width$}  {:>16}  {}", node.name, "plane", hex8(v.hash())),
                _ => println!("{:width$}  {:>16}  {}", node.name, "sketch", hex8(v.hash())),
            },
            Err(err) => println!("{:width$}  FAILED: {err}", node.name),
        }
    }
    println!("computed {}, reused {}, {:.2} ms", e.computed, e.reused, elapsed.as_secs_f64() * 1e3);
    if let Some(path) = out {
        if path.extension().is_some_and(|x| x.eq_ignore_ascii_case("step") || x.eq_ignore_ascii_case("stp")) {
            match datum_model::write_step(&g, &e, path, approximate)? {
                0 => println!("written {}", path.display()),
                n => println!("written {} ({n} faces approximated by B-splines: see {}.txt)", path.display(), path.display()),
            }
        } else {
            let bodies: Vec<&Body> = e.final_values(&g).into_iter().filter_map(Value::body).collect();
            write_mesh(&bodies, path)?;
        }
    }
    match e.results.iter().filter(|r| r.is_err()).count() {
        0 => Ok(()),
        n => Err(format!("{n} feature(s) failed")),
    }
}

/// The flat blank of an unfold feature as DXF: the one named, or the model's only one.
fn cmd_dxf(args: &[String]) -> Result<(), String> {
    let (model, out, feature) = match args {
        [m, o, out] if o == "--out" => (m, out, None),
        [m, o, out, f, name] if o == "--out" && f == "--feature" => (m, out, Some(name.as_str())),
        _ => return Err(USAGE.into()),
    };
    let g = load(Path::new(model))?;
    let e = Evaluator::default().evaluate(&g);
    let flats: Vec<_> = g
        .nodes()
        .iter()
        .zip(&e.results)
        .filter_map(|(n, r)| Some((n.name.as_str(), r.as_ref().ok()?.flat()?)))
        .filter(|(n, _)| feature.is_none_or(|f| f == *n))
        .collect();
    let (name, flat) = match flats[..] {
        [one] => one,
        [] => {
            return Err(
                feature.map_or_else(|| "the model has no unfold feature".into(), |f| format!("`{f}` is not an unfold feature that succeeded"))
            );
        }
        _ => return Err(format!("unfold features: {}; choose one with --feature", flats.iter().map(|f| f.0).collect::<Vec<_>>().join(", "))),
    };
    std::fs::write(out, flat.dxf()).map_err(|e| format!("{out}: {e}"))?;
    let [lines, arcs, circles] = flat.counts();
    println!("{name}: {:.3} × {:.3} mm, {lines} lines, {arcs} arcs, {circles} circles → {out}", flat.size.x, flat.size.y);
    Ok(())
}

fn cmd_snapshot(args: &[String]) -> Result<(), String> {
    let [model, rest @ ..] = args else { return Err(USAGE.into()) };
    let (mut out, mut size, mut view) = (None, "1024x768", "iso");
    for pair in rest.chunks(2) {
        match pair {
            [flag, o] if flag == "--out" => out = Some(o),
            [flag, s] if flag == "--size" => size = s,
            [flag, v] if flag == "--view" => view = v,
            _ => return Err(USAGE.into()),
        }
    }
    let out = out.ok_or(USAGE)?;
    let (w, h) = parse_size(size)?;
    let g = load(Path::new(model))?;
    let e = Evaluator::default().evaluate(&g);
    let png = datum_view::capture::snapshot(&e.final_values(&g), view, w, h)?;
    std::fs::write(out, png).map_err(|e| format!("{out}: {e}"))?;
    println!("written {out} ({w}×{h})");
    Ok(())
}

fn parse_size(size: &str) -> Result<(u32, u32), String> {
    size.split_once('x')
        .and_then(|(w, h)| Some((w.parse::<u32>().ok()?, h.parse::<u32>().ok()?)))
        .filter(|&(w, h)| w > 0 && h > 0)
        .ok_or_else(|| "--size takes WxH, e.g. 1024x768".into())
}

fn cmd_bench_view(args: &[String]) -> Result<(), String> {
    let [model, rest @ ..] = args else { return Err(USAGE.into()) };
    let mut g = load(Path::new(model))?;
    let (mut size, mut frames) = ("1600x900", 60);
    for pair in rest.chunks(2) {
        match pair {
            [flag, s] if flag == "--size" => size = s,
            [flag, n] if flag == "--frames" => frames = n.parse().map_err(|_| "--frames takes a whole number")?,
            [flag, kv] if flag == "--set" => {
                let (name, value) = kv.split_once('=').ok_or("--set takes name=value")?;
                let value = value.parse().map_or_else(|_| Num::Expr(value.into()), Num::Value);
                g.set_parameter(name, value).map_err(|e| e.to_string())?;
            }
            _ => return Err(USAGE.into()),
        }
    }
    let (w, h) = parse_size(size)?;
    let start = Instant::now();
    let e = Evaluator::default().evaluate(&g);
    let rebuild = start.elapsed();
    let start = Instant::now();
    let scene = datum_view::scene::Scene::new(&e.final_values(&g), &mut Default::default());
    let tessellation = start.elapsed();
    let (triangles, edges) = scene.bodies.iter().fold((0, 0), |(t, l), m| (t + m.triangles.len(), l + m.edges.len()));
    println!(
        "rebuild {:.0} ms, tessellation {:.0} ms: {triangles} triangles, {edges} edges",
        rebuild.as_secs_f64() * 1e3,
        tessellation.as_secs_f64() * 1e3
    );
    let start = Instant::now();
    let geometry = datum_view::scene::Geometry::new(&scene, scene.anchor());
    let vertices = geometry.triangles.vertices.len() as u64 / datum_view::scene::TRIANGLE_VERTEX;
    println!("GPU geometry {:.0} ms: {vertices} triangle vertices", start.elapsed().as_secs_f64() * 1e3);
    let (adapter, cases) = datum_view::capture::frame_times(&scene, geometry, w, h, frames)?;
    println!("{adapter}, {w}×{h}, median of {frames} frames:");
    for (case, ms) in cases {
        println!("  {case:<36} {ms:>8.2} ms");
    }
    Ok(())
}

fn write_mesh(bodies: &[&Body], path: &Path) -> Result<(), String> {
    let mut mesh = Mesh { positions: Vec::new(), triangles: Vec::new() };
    for b in bodies {
        let m = tessellate(b).map_err(|e| e.to_string())?;
        let base = mesh.positions.len() as u32;
        mesh.positions.extend(m.positions);
        mesh.triangles.extend(m.triangles.iter().map(|t| t.map(|i| i + base)));
    }
    let file = File::create(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let mut w = BufWriter::new(file);
    let res = match path.extension().and_then(|e| e.to_str()).map(str::to_ascii_lowercase).as_deref() {
        Some("stl") => export::write_stl(&mesh, &mut w),
        Some("obj") => export::write_obj(&mesh, &mut w),
        _ => return Err(format!("{}: use .step, .stl or .obj", path.display())),
    };
    res.map_err(|e| format!("{}: {e}", path.display()))?;
    println!("written {} ({} bodies, {} triangles)", path.display(), bodies.len(), mesh.triangles.len());
    Ok(())
}

/// Model files named by `args`; directories give their `*.json` files, sorted.
fn model_files(args: &[String]) -> Result<Vec<PathBuf>, String> {
    let mut files = Vec::new();
    for a in args {
        let p = PathBuf::from(a);
        if p.is_dir() {
            let rd = std::fs::read_dir(&p).map_err(|e| format!("{a}: {e}"))?;
            let mut found: Vec<PathBuf> =
                rd.filter_map(|e| e.ok().map(|e| e.path())).filter(|f| f.extension().is_some_and(|x| x == "json")).collect();
            found.sort();
            files.extend(found);
        } else {
            files.push(p);
        }
    }
    if files.is_empty() { Err(USAGE.into()) } else { Ok(files) }
}

fn cmd_determinism(args: &[String]) -> Result<(), String> {
    let mut failed = 0;
    for path in model_files(args)? {
        let g = load(&path)?;
        // Each result reduced to its hash or its error text.
        let run = |threads: usize| -> Result<Vec<Result<[u8; 32], String>>, String> {
            let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().map_err(|e| e.to_string())?;
            let e = pool.install(|| Evaluator::default().evaluate(&g));
            Ok(e.results.iter().map(|r| r.as_ref().map(|b| b.hash()).map_err(|e| e.to_string())).collect())
        };
        let one = run(1)?;
        let mut mismatch = Vec::new();
        for threads in [4, 16] {
            let other = run(threads)?;
            for ((node, a), b) in g.nodes().iter().zip(&one).zip(&other) {
                if a != b {
                    mismatch.push(format!("{} (threads={threads})", node.name));
                }
            }
        }
        if mismatch.is_empty() {
            println!("ok        {} ({} features)", path.display(), g.nodes().len());
        } else {
            failed += 1;
            println!("MISMATCH  {}: {}", path.display(), mismatch.join(", "));
        }
    }
    if failed == 0 { Ok(()) } else { Err(format!("{failed} model(s) depend on thread count")) }
}
