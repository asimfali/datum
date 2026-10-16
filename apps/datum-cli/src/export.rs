//! Mesh writers: binary STL and Wavefront OBJ.

use std::io::{self, Write};

use datum_brep::Mesh;

pub fn write_stl(mesh: &Mesh, w: &mut impl Write) -> io::Result<()> {
    w.write_all(&[0u8; 80])?;
    w.write_all(&(mesh.triangles.len() as u32).to_le_bytes())?;
    for t in &mesh.triangles {
        let [a, b, c] = t.map(|i| mesh.positions[i as usize]);
        let n = (b - a).cross(c - a);
        let len = n.dot(n).sqrt();
        let n = if len > 0.0 { n * (1.0 / len) } else { n };
        for v in [n, a, b, c] {
            for x in v.to_array() {
                w.write_all(&(x as f32).to_le_bytes())?;
            }
        }
        w.write_all(&[0, 0])?;
    }
    Ok(())
}

/// OBJ keeps full `f64` precision and shared vertices.
pub fn write_obj(mesh: &Mesh, w: &mut impl Write) -> io::Result<()> {
    for p in &mesh.positions {
        writeln!(w, "v {:?} {:?} {:?}", p.x, p.y, p.z)?;
    }
    for t in &mesh.triangles {
        writeln!(w, "f {} {} {}", t[0] + 1, t[1] + 1, t[2] + 1)?;
    }
    Ok(())
}
