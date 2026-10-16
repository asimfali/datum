//! `datum`: command-line front end of the Datum kernel.

use std::{fs::File, io::BufWriter, path::Path, process::ExitCode};

use datum_brep::{Body, TopoId, prism, tessellate};
use datum_math::Vec2;

mod export;

const USAGE: &str = "usage: datum prism <x,y> <x,y> <x,y>... --height <h> [--out <file.stl|file.obj>]";

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
    let Some((cmd, rest)) = args.split_first() else { return Err(USAGE.into()) };
    match cmd.as_str() {
        "prism" => cmd_prism(rest),
        _ => Err(format!("unknown command `{cmd}`\n{USAGE}")),
    }
}

fn cmd_prism(args: &[String]) -> Result<(), String> {
    let mut polygon = Vec::new();
    let (mut height, mut out) = (None, None);
    let mut it = args.iter();
    while let Some(a) = it.next() {
        match a.as_str() {
            "--height" => height = Some(it.next().and_then(|v| v.parse::<f64>().ok()).ok_or("--height needs a number")?),
            "--out" => out = Some(it.next().ok_or("--out needs a file name")?),
            p => {
                let (x, y) = p.split_once(',').ok_or_else(|| format!("`{p}` is not a point x,y"))?;
                let num = |s: &str| s.trim().parse::<f64>().map_err(|_| format!("`{p}` is not a point x,y"));
                polygon.push(Vec2::new(num(x)?, num(y)?));
            }
        }
    }
    let height = height.ok_or(USAGE)?;
    let body = prism(TopoId::derive("cli.prism").finish(), &polygon, 0.0, height).map_err(|e| e.to_string())?;
    report(&body);
    if let Some(path) = out {
        write_mesh(&body, Path::new(path))?;
    }
    Ok(())
}

fn report(b: &Body) {
    let valid = match b.validate() {
        Ok(()) => "yes".to_string(),
        Err(e) => format!("NO: {e}"),
    };
    let hash: String = b.hash().iter().map(|x| format!("{x:02x}")).collect();
    println!("valid     {valid}");
    println!("vertices  {}  edges {}  faces {}", b.vertices().len(), b.edges().len(), b.faces().len());
    println!("volume    {}", b.volume());
    println!("hash      {hash}");
}

fn write_mesh(b: &Body, path: &Path) -> Result<(), String> {
    let mesh = tessellate(b).map_err(|e| e.to_string())?;
    let file = File::create(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let mut w = BufWriter::new(file);
    let res = match path.extension().and_then(|e| e.to_str()).map(str::to_ascii_lowercase).as_deref() {
        Some("stl") => export::write_stl(&mesh, &mut w),
        Some("obj") => export::write_obj(&mesh, &mut w),
        _ => return Err(format!("{}: use .stl or .obj", path.display())),
    };
    res.map_err(|e| format!("{}: {e}", path.display()))?;
    println!("written   {} ({} triangles)", path.display(), mesh.triangles.len());
    Ok(())
}
