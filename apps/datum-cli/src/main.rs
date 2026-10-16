//! `datum`: command-line front end of the Datum kernel.

use std::{
    fs::File,
    io::BufWriter,
    path::{Path, PathBuf},
    process::ExitCode,
    time::Instant,
};

use datum_brep::{Body, Mesh, tessellate};
use datum_graph::{Evaluation, Evaluator, Graph};

mod export;

const USAGE: &str = "usage:
  datum build <model.json> [--out <file.stl|file.obj>]   evaluate a model, export its final bodies
  datum determinism-check <model.json|dir>...           same result on 1, 4 and 16 threads (C4)
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

fn run(args: &[String]) -> Result<(), String> {
    match args.split_first() {
        Some((cmd, rest)) if cmd == "build" => cmd_build(rest),
        Some((cmd, rest)) if cmd == "determinism-check" => cmd_determinism(rest),
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
    let (model, out) = match args {
        [m] => (m, None),
        [m, flag, o] if flag == "--out" => (m, Some(Path::new(o))),
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
            Ok(b) => println!("{:width$}  {:>16.6}  {}", node.name, b.volume(), hex8(b.hash())),
            Err(err) => println!("{:width$}  FAILED: {err}", node.name),
        }
    }
    println!("computed {}, reused {}, {:.2} ms", e.computed, e.reused, elapsed.as_secs_f64() * 1e3);
    if let Some(path) = out {
        write_mesh(&final_bodies(&g, &e), path)?;
    }
    match e.results.iter().filter(|r| r.is_err()).count() {
        0 => Ok(()),
        n => Err(format!("{n} feature(s) failed")),
    }
}

/// Bodies of the features nothing else uses.
fn final_bodies<'e>(g: &Graph, e: &'e Evaluation) -> Vec<&'e Body> {
    let mut used = vec![false; g.nodes().len()];
    for n in g.nodes() {
        for &i in &n.inputs {
            used[i as usize] = true;
        }
    }
    e.results.iter().zip(used).filter(|(_, u)| !u).filter_map(|(r, _)| r.as_ref().ok().map(|b| b.as_ref())).collect()
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
        _ => return Err(format!("{}: use .stl or .obj", path.display())),
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
