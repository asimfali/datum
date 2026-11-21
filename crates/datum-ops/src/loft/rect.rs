//! Chord facets at the corners and triangular panels opposite the straight sides (D264).

use std::f64::consts::FRAC_PI_2;

use super::*;

type Section = [Vec3; 2];

struct Profiles {
    corners: [Vec3; 4],
    normals: [Vec3; 4],
    circle: Vec3,
    radius: f64,
    limit: f64,
    count: usize,
}

pub(super) fn circle_rectangle(
    feature: TopoId,
    circle: &Placed,
    rectangle: &Placed,
    sheet: Sheet,
    tolerance: f64,
    gap: f64,
) -> Result<Body, BrepError> {
    if !(gap > 0.0 && gap.is_finite()) {
        return Err(BrepError::Degenerate("loft: a closed profile needs a positive seam gap".into()));
    }
    // The closed strip uses outward normals, while the loft's Left is the inside of a circle.
    let side = match sheet.side {
        Side::Left => Side::Right,
        Side::Right => Side::Left,
        Side::Middle => Side::Middle,
    };
    let sheet = Sheet { side, ..sheet };
    let profiles = Profiles::new(circle, rectangle, tolerance)?;
    let (_, strip) = profiles.rounded_strip(sheet, gap)?;
    strip.body(feature, sheet)
}

impl Profiles {
    fn rounded_strip(&self, sheet: Sheet, gap: f64) -> Result<(f64, Strip), BrepError> {
        let rule = BendRule { thickness: sheet.thickness, radius: sheet.radius, side: sheet.side };
        let fit = |round| Strip::new(&self.stations(round, gap)?, rule);
        // The initial fit leaves straight sides of positive length. Shrink the radius until
        // the two tangent lines on a corner facet come together at the narrow end.
        let mut low = 0.0;
        let mut high = self.limit * 0.5;
        let mut strip = fit(high).or_else(|_| {
            high = self.limit - high * 0.5;
            fit(high)
        })?;
        while let mid = low + (high - low) * 0.5
            && mid > low
            && mid < high
        {
            if let Ok(candidate) = fit(mid) {
                strip = candidate;
                high = mid;
            } else {
                low = mid;
            }
        }
        Ok((high, strip))
    }

    fn new(circle: &Placed, rect: &Placed, tolerance: f64) -> Result<Self, BrepError> {
        let unsupported = || BrepError::Unsupported("loft: expected one circle and one closed rectangle on parallel planes".into());
        let mut circles = circle.circles.values().filter(|c| !c.construction);
        let c = circles.next().ok_or_else(unsupported)?;
        if circles.next().is_some()
            || circle.lines.values().any(|l| !l.construction)
            || circle.arcs.values().any(|a| !a.construction)
            || !circle.ellipses.is_empty()
            || circle.developed.is_some()
            || rect.developed.is_some()
        {
            return Err(unsupported());
        }
        let center = profile_point(circle, "circle", &c.center)?;
        let radius = c.diameter.value().ok_or_else(unsupported)? * 0.5;
        let mut axis = rect.frame[3];
        if !within_rounding(length(axis.cross(circle.frame[3])), 1.0) {
            return Err(unsupported());
        }
        let height = (center - rect.frame[0]).dot(axis);
        if !(height.abs() > 0.0 && radius > 0.0 && radius.is_finite()) {
            return Err(BrepError::Degenerate("loft: the circle must have a positive radius and be above the rectangle plane".into()));
        }
        axis = axis * height.signum();
        if rect.circles.values().any(|c| !c.construction) || rect.arcs.values().any(|a| !a.construction) || !rect.ellipses.is_empty() {
            return Err(unsupported());
        }
        let contour = rect.contours(&|_, construction| !construction);
        let ([chain], None) = (contour.loops.as_slice(), contour.odd()) else { return Err(unsupported()) };
        let [first, second, third, fourth] = chain.as_slice() else { return Err(unsupported()) };
        let [a, b, c, d] = [first, second, third, fourth].map(|step| profile_point(rect, "rectangle", step.from));
        let (a, b, c, d) = (a?, b?, c?, d?);
        let x = unit(b - a).ok_or_else(unsupported)?;
        let y = axis.cross(x);
        let width = length(b - a);
        let depth = (c - b).dot(y);
        let scale = width.max(depth.abs());
        if depth == 0.0 || !within_rounding(length(c - b - y * depth), scale) || !within_rounding(length(d - a - y * depth), scale) {
            return Err(unsupported());
        }
        // Regenerate the orthogonal sides from their dimensions so that paired chords
        // have common directions, including after rounding in the sketch solver.
        let (origin, x, y) = if depth > 0.0 { (a, x, y) } else { (b, -x, -y) };
        let vx = x * width;
        let vy = y * depth.abs();
        let limit = 0.5 * width.min(depth.abs());
        let count = chord_count(radius.max(limit), FRAC_PI_2, tolerance)?;
        Ok(Self { corners: [origin, origin + vx, origin + vx + vy, origin + vy], normals: [-x, -y, x, y], circle: center, radius, limit, count })
    }

    fn stations(&self, round: f64, gap: f64) -> Result<Vec<Section>, BrepError> {
        let mut cycle = Vec::with_capacity(4 * (self.count + 1));
        for (i, &corner) in self.corners.iter().enumerate() {
            let u = self.normals[i];
            let v = self.normals[(i + 1) % 4];
            let center = corner - (u + v) * round;
            for j in 0..=self.count {
                let radial = if j == 0 {
                    u
                } else if j == self.count {
                    v
                } else {
                    let (sin, cos) = libm::sincos(FRAC_PI_2 * j as f64 / self.count as f64);
                    u * cos + v * sin
                };
                cycle.push([center + radial * round, self.circle + radial * self.radius]);
            }
        }
        // Split one chord facet into two panels with parallel seam edges. A fixed
        // fraction of the two chords would narrow the gap at the smaller end.
        let seam = self.count / 2;
        let left = cycle[seam];
        let right = cycle[seam + 1];
        let chords = [0, 1].map(|j| right[j] - left[j]);
        let widths = chords.map(length);
        let too_wide = || BrepError::Degenerate("loft: the seam gap does not fit on both ends of its chord facet".into());
        if !widths.into_iter().all(|width| width > gap && width.is_finite()) {
            return Err(too_wide());
        }
        let edge = |side| [0, 1].map(|j| left[j] + chords[j] * (0.5 + side * (0.5 * gap / widths[j])));
        let (start, end) = (edge(1.0), edge(-1.0));
        if start.into_iter().zip(end).any(|(a, b)| a == b) {
            return Err(BrepError::Degenerate("loft: the seam gap is too small to separate its edges".into()));
        }
        let mut open = Vec::with_capacity(cycle.len() + 2);
        open.push(start);
        open.extend_from_slice(&cycle[seam + 1..]);
        open.extend_from_slice(&cycle[..=seam]);
        open.push(end);
        Ok(open)
    }
}

struct Panel {
    normal: Vec3,
    outline: [Vec<Vec3>; 2],
}

struct Strip {
    panels: Vec<Panel>,
    bends: Vec<TaperBend>,
}

impl Strip {
    fn new(stations: &[Section], rule: BendRule) -> Result<Self, BrepError> {
        let cramped = || BrepError::Degenerate("loft: the corner or seam leaves no room between the bends".into());
        let ends = stations.len() - 1;
        let bends: Vec<_> =
            (1..ends).map(|i| bend(stations[i][0], stations[i][1], [stations[i - 1][0], stations[i + 1][0]], 1.0, rule)).collect::<Result<_, _>>()?;
        let mut cuts: Vec<_> = bends.iter().map(|b| [0.0, b.length]).collect();
        let mut panels = Vec::with_capacity(ends);
        for (i, pair) in stations.windows(2).enumerate() {
            let (a, b) = (pair[0], pair[1]);
            let normal = unit((b[0] - a[0]).cross(a[1] - a[0])).ok_or_else(cramped)?;
            let left = i.checked_sub(1).map(|j| bends[j]);
            let right = bends.get(i).copied();
            let outline = [0, 1].map(|j| -> Result<Vec<Vec3>, BrepError> {
                let update = |cut: &mut [f64; 2], at: f64| {
                    cut[j] = if j == 0 { cut[j].max(at) } else { cut[j].min(at) };
                };
                if a[j] == b[j] {
                    let (Some(l), Some(r)) = (left, right) else { return Err(cramped()) };
                    let (p, ls, rs) = line_meeting_at_scale(l.wing_tangent, l.axis, r.middle_tangent, r.axis, l.length.max(r.length))?;
                    update(&mut cuts[i - 1], ls);
                    update(&mut cuts[i], rs);
                    return Ok(vec![p]);
                }
                let edge = b[j] - a[j];
                let start = if let Some(l) = left {
                    let (point, at) = tangent_crossing_at_scale(l.wing_tangent, l.axis, a[j], edge, l.length)?;
                    update(&mut cuts[i - 1], at);
                    point
                } else {
                    a[j]
                };
                let end = if let Some(r) = right {
                    let (point, at) = tangent_crossing_at_scale(r.middle_tangent, r.axis, b[j], -edge, r.length)?;
                    update(&mut cuts[i], at);
                    point
                } else {
                    b[j]
                };
                let width = (end - start).dot(unit(edge).ok_or_else(cramped)?);
                if width < 0.0 {
                    return Err(cramped());
                }
                // At the minimum corner radius both tangent lines may meet here. The
                // intersections were computed on different bend axes; share a vertex if
                // their separation is within rounding of those constructions.
                let scale = length(edge).max(left.map_or(0.0, |b| b.length)).max(right.map_or(0.0, |b| b.length));
                if within_rounding(length(end - start), scale) {
                    return Ok(vec![start + (end - start) * 0.5]);
                }
                Ok(vec![start, end])
            });
            let [a, b] = outline;
            let (a, b) = (a?, b?);
            if a.len() + b.len() < 3 {
                return Err(cramped());
            }
            panels.push(Panel { normal, outline: [a, b] });
        }

        let bends: Vec<_> = bends.into_iter().zip(cuts).map(|(bend, ends)| TaperBend { bend, ends }).collect();
        for (i, TaperBend { bend, ends }) in bends.iter().enumerate() {
            if ends[0] >= ends[1] {
                return Err(cramped());
            }
            let size = bend.length;
            for (j, &end) in ends.iter().enumerate() {
                for (panel, tangent, append) in [(i, bend.middle_tangent, true), (i + 1, bend.wing_tangent, false)] {
                    let rim = &mut panels[panel].outline[j];
                    let index = if append { rim.len() - 1 } else { 0 };
                    let point = tangent + bend.axis * end;
                    if !within_rounding(length(rim[index] - point), size) {
                        rim.insert(if append { rim.len() } else { 0 }, point);
                    }
                }
            }
        }
        Ok(Self { panels, bends })
    }

    fn body(self, feature: TopoId, sheet: Sheet) -> Result<Body, BrepError> {
        let Self { panels, bends } = self;
        let mut shell = LoftShell {
            feature,
            vertices: Vec::with_capacity(12 * panels.len()),
            faces: Vec::with_capacity(10 * panels.len()),
            curves: Vec::with_capacity(4 * bends.len()),
        };
        let (low, high) = thickness_sides(sheet.side, sheet.thickness);
        let name = |i| format!("closed.panel.{i}/{}", panels.len());
        // A section always retains at least one vertex, including at a trimmed triangle tip.
        let last_vertex = |rim: &[usize]| rim[rim.len() - 1];
        let [lower, upper] = [("lower", low, -1.0), ("upper", high, 1.0)].map(|(skin, offset, outward)| {
            let edges: Vec<_> = panels
                .iter()
                .enumerate()
                .map(|(i, p)| {
                    let rim = [0, 1].map(|j| {
                        p.outline[j]
                            .iter()
                            .enumerate()
                            .map(|(m, &point)| shell.vertex(skin, &name(i), &format!("rim.{j}.{m}"), point + p.normal * offset))
                            .collect::<Vec<_>>()
                    });
                    let ids = rim[0].iter().chain(rim[1].iter().rev()).copied().collect();
                    shell.plane(skin, &name(i), p.normal * outward, ids);
                    rim
                })
                .collect();
            for (i, TaperBend { bend, ends }) in bends.iter().enumerate() {
                let entry = [0, 1].map(|j| last_vertex(&edges[i][j]));
                let exit = [0, 1].map(|j| edges[i + 1][j][0]);
                let convex = outward > 0.0;
                let radius = sheet.radius + if convex { sheet.thickness } else { 0.0 };
                shell.face(
                    skin,
                    &format!("{}.bend", name(i)),
                    Surface::Cylinder(Cylinder { origin: bend.center, axis: bend.axis, radius, convex }),
                    vec![entry[0], entry[1], exit[1], exit[0]],
                );
                for j in 0..2 {
                    shell.arc(entry[j], exit[j], bend.center + bend.axis * ends[j], bend.axis, radius);
                }
            }
            edges
        });
        for (i, (low, up)) in lower.iter().zip(&upper).enumerate() {
            for j in 0..2 {
                for (k, (l, u)) in low[j].windows(2).zip(up[j].windows(2)).enumerate() {
                    let forward = if j == 0 { 1.0 } else { -1.0 };
                    let normal = unit((shell.point(l[1]) - shell.point(l[0])).cross(panels[i].normal) * forward)
                        .ok_or_else(|| BrepError::Degenerate("loft: a cut on the rim has no length".into()))?;
                    shell.plane(&name(i), &format!("rim.{j}.{k}"), normal, vec![l[0], l[1], u[1], u[0]]);
                }
            }
            if let Some(TaperBend { bend, .. }) = bends.get(i) {
                for (j, normal) in [(0, -bend.axis), (1, bend.axis)] {
                    shell.plane(
                        &name(i),
                        &format!("bend.end.{j}"),
                        normal,
                        vec![last_vertex(&low[j]), lower[i + 1][j][0], upper[i + 1][j][0], last_vertex(&up[j])],
                    );
                }
            }
        }
        let last = panels.len() - 1;
        for (i, start) in [(0, true), (last, false)] {
            let indices = |rim: &[Vec<usize>; 2]| [0, 1].map(|j| if start { rim[j][0] } else { last_vertex(&rim[j]) });
            let [l0, l1] = indices(&lower[i]);
            let [u0, u1] = indices(&upper[i]);
            let other = if start { last_vertex(&lower[i][0]) } else { lower[i][0][0] };
            shell.wall("seam", if start { "start" } else { "end" }, [l0, l1, u1, u0], other)?;
        }
        shell.build()
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use super::*;

    #[test]
    fn a_closed_strip_has_one_blank_after_cutting_the_seam() {
        let v = Vec3::new;
        let profiles = Profiles {
            corners: [v(-40.0, -50.0, 0.0), v(40.0, -50.0, 0.0), v(40.0, 50.0, 0.0), v(-40.0, 50.0, 0.0)],
            normals: [v(-1.0, 0.0, 0.0), v(0.0, -1.0, 0.0), v(1.0, 0.0, 0.0), v(0.0, 1.0, 0.0)],
            circle: v(0.0, 0.0, 50.0),
            radius: 40.0,
            limit: 40.0,
            count: 4,
        };
        let sheet = Sheet { thickness: 2.0, radius: 2.0, side: Side::Right };
        let rule = BendRule { thickness: sheet.thickness, radius: sheet.radius, side: sheet.side };
        let (round, strip) = profiles.rounded_strip(sheet, 1.0).unwrap();
        assert!((2.0..10.0).contains(&round));
        assert!(Strip::new(&profiles.stations(round - 0.01, 1.0).unwrap(), rule).is_err());
        let start = &strip.panels[0].outline;
        let end = &strip.panels.last().unwrap().outline;
        for j in 0..2 {
            let width = length(start[j][0] - *end[j].last().unwrap());
            assert!(within_rounding(width - 1.0, profiles.radius), "seam gap on section {j}: {width}");
        }
        let body = strip.body(TopoId::derive("loft.test").finish(), sheet).unwrap();
        body.validate().unwrap();
        let flat = crate::unfold_side(&body, false, 0.44).unwrap();
        assert_eq!(flat.outer.len(), flat.pieces.len());
        assert_eq!(flat.bends.len(), 40);
        assert!(flat.overlaps.is_empty());
    }
}
