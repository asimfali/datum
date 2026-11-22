use datum_brep::{BrepError, ProfileLoop, TopoId, segments_touch};
use datum_math::{Arc2, Sign, Vec2, orient2d, segment_touches_arc};

use crate::sheet::{Sheet, SheetPath, Side, bend_profile};

/// Bend the wall and the tongue as one strip, then trim the wall lead at the root tangency.
pub(super) fn rounded(
    feature: TopoId,
    mut points: [Vec2; 4],
    names: [&str; 4],
    lines: [&str; 3],
    root_cap: TopoId,
    sheet: Sheet,
    k: f64,
) -> Result<(ProfileLoop, f64, f64), BrepError> {
    if !(0.0..=1.0).contains(&k) {
        return Err(BrepError::Invalid(format!("stamp: K-factor must be in 0..=1, got {k}")));
    }
    let Sheet { thickness, radius, side } = sheet;
    let straight_nose = orient2d(points[1], points[2], points[3]) == Sign::Zero;
    if side == Side::Left {
        for point in &mut points {
            point.y -= thickness;
        }
    }
    let [root, wall, shelf, tip] = points;
    let p = [Vec2::new(-2.0 * (radius + thickness), root.y), root, wall, shelf, tip];
    let [a, b, c, d] = names;
    let [wall, shelf, nose] = lines;
    let mut ring =
        bend_profile(feature, SheetPath { p: &p, names: &[a, a, b, c, d], lines: &["", wall, shelf, nose], arcs: &[None; 4] }, sheet, &|a, b| {
            straight_nose && a == shelf && b == nose
        })?;
    ring.points.remove(0);
    ring.points.pop();
    ring.vertices.remove(0);
    ring.vertices.pop();
    ring.sides.remove(0);
    ring.sides.pop();
    ring.arcs.remove(0);
    ring.arcs.pop();
    let last = ring.points.len() - 1;
    ring.sides[last] = root_cap;
    // From the root to the tip on the first face. Offsetting a straight span preserves its
    // length. Each bend takes its own angle at the neutral radius from the inside of the bend.
    let length = (0..last / 2)
        .map(|i| {
            let (a, b) = (ring.points[i], ring.points[i + 1]);
            ring.arc(i).map_or_else(|| libm::hypot(b.x - a.x, b.y - a.y), |arc| (radius + k * thickness) * arc.turn(a, b).abs())
        })
        .sum();
    // The lower root arc crosses the top of the wall where the tongue's side meets the cut.
    let arc = ring.arcs[0].ok_or_else(|| BrepError::Degenerate("stamp: missing root bend".into()))?;
    let cross = Vec2::new(arc.center.x + libm::sqrt(thickness * (2.0 * radius + thickness)), 0.0);
    let tip = ring.points.iter().map(|p| p.x).fold(cross.x, f64::max);
    if ring.points[1..last].iter().any(|p| p.y <= 0.0)
        || (1..last - 1).any(|i| {
            ring.arc(i).is_some_and(|arc| {
                let [from, to] = if arc.ccw { [ring.points[i], ring.points[i + 1]] } else { [ring.points[i + 1], ring.points[i]] };
                segment_touches_arc(Vec2::new(cross.x, 0.0), Vec2::new(tip, 0.0), &Arc2 { center: arc.center, from, to })
            })
        })
    {
        return Err(BrepError::Degenerate("stamp: rounded section meets the wall beyond its root".into()));
    }
    Ok((ring, length, cross.x))
}

/// Offset a three-segment section to its left for positive thickness.
pub(super) fn offset(points: [Vec2; 4], signed_thickness: f64) -> Result<[Vec2; 4], BrepError> {
    let bad = || BrepError::Degenerate("stamp: section offset collapses or intersects".into());
    if !signed_thickness.is_finite() || signed_thickness == 0.0 || points.iter().any(|p| !p.x.is_finite() || !p.y.is_finite()) {
        return Err(bad());
    }
    let mut directions = [Vec2::default(); 3];
    let mut normals = [Vec2::default(); 3];
    for i in 0..3 {
        let d = Vec2::new(points[i + 1].x - points[i].x, points[i + 1].y - points[i].y);
        let length = libm::hypot(d.x, d.y);
        if length == 0.0 || !length.is_finite() {
            return Err(bad());
        }
        directions[i] = Vec2::new(d.x / length, d.y / length);
        normals[i] = Vec2::new(-directions[i].y * signed_thickness, directions[i].x * signed_thickness);
    }
    let shifted = |p: Vec2, n: Vec2| Vec2::new(p.x + n.x, p.y + n.y);
    let mut result = [Vec2::default(); 4];
    result[0] = shifted(points[0], normals[0]);
    result[3] = shifted(points[3], normals[2]);
    for i in 1..3 {
        let a = directions[i - 1];
        let b = directions[i];
        let turn = orient2d(points[i - 1], points[i], points[i + 1]);
        if turn == Sign::Zero {
            if a.x * b.x + a.y * b.y <= 0.0 {
                return Err(bad());
            }
            result[i] = shifted(points[i], normals[i - 1]);
        } else {
            let p = shifted(points[i], normals[i - 1]);
            let cross = a.x * b.y - a.y * b.x;
            if cross == 0.0 {
                return Err(bad());
            }
            let dn = Vec2::new(normals[i].x - normals[i - 1].x, normals[i].y - normals[i - 1].y);
            let t = (dn.x * b.y - dn.y * b.x) / cross;
            result[i] = Vec2::new(p.x + t * a.x, p.y + t * a.y);
        }
    }
    if result[0] == points[0] || result[3] == points[3] || result.iter().any(|p| !p.x.is_finite() || !p.y.is_finite()) {
        return Err(bad());
    }
    // Each offset edge must retain its direction; otherwise a corner has passed its neighbour.
    for i in 0..3 {
        let dx = result[i + 1].x - result[i].x;
        let dy = result[i + 1].y - result[i].y;
        if dx * directions[i].x + dy * directions[i].y <= 0.0 {
            return Err(bad());
        }
    }
    // The section and its offset form one simple closed boundary, including both end caps.
    let boundary = [points[0], points[1], points[2], points[3], result[3], result[2], result[1], result[0]];
    for i in 0..8 {
        for j in i + 1..8 {
            if j == i + 1 || (i == 0 && j == 7) {
                continue;
            }
            if segments_touch(boundary[i], boundary[(i + 1) % 8], boundary[j], boundary[(j + 1) % 8]) {
                return Err(bad());
            }
        }
    }
    Ok(result)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use super::*;

    #[test]
    fn leaves_a_straight_shelf_to_nose_joint_without_a_bend() {
        let points = [Vec2::new(0.0, 0.0), Vec2::new(0.0, 4.0), Vec2::new(4.0, 5.0), Vec2::new(8.0, 6.0)];
        let id = TopoId::derive("stamp test").finish();
        let sheet = Sheet { thickness: 1.0, radius: 0.4, side: Side::Right };
        let (profile, _, _) = rounded(id, points, ["root", "wall", "shelf", "tip"], ["wall", "shelf", "nose"], id, sheet, 0.5).unwrap();
        assert_eq!(profile.arcs.iter().flatten().count(), 4);
        let caps = ["start", "end"].map(|name| TopoId::derive(name).finish());
        assert!(datum_brep::extrude(&[datum_brep::Region { outer: profile, holes: Vec::new(), caps }], 0.0, 1.0).is_ok());
    }

    #[test]
    fn offsets_both_sides_at_two_thicknesses() {
        let p = [Vec2::new(0.0, 0.0), Vec2::new(0.0, 4.0), Vec2::new(5.0, 4.0), Vec2::new(5.0, 8.0)];
        for t in [0.5, 1.0, -0.5, -1.0] {
            let q = offset(p, t).unwrap();
            assert_eq!(q, [Vec2::new(-t, 0.0), Vec2::new(-t, 4.0 + t), Vec2::new(5.0 - t, 4.0 + t), Vec2::new(5.0 - t, 8.0)]);
        }
    }

    #[test]
    fn rejects_degenerate_sections() {
        let p = [Vec2::new(0.0, 0.0), Vec2::new(0.0, 1.0), Vec2::new(1.0, 1.0), Vec2::new(1.0, 2.0)];
        assert!(offset(p, 1.0).is_err());
        assert!(offset(p, 0.0).is_err());
        assert!(offset([p[0], p[0], p[2], p[3]], 0.1).is_err());
        assert!(offset([p[0], p[1], p[0], p[3]], 0.1).is_err());
        assert!(offset([Vec2::new(0.0, 0.0), Vec2::new(2.0, 2.0), Vec2::new(0.0, 2.0), Vec2::new(2.0, 0.0)], 0.1).is_err());
    }
}
