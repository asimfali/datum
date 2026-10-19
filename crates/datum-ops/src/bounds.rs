//! Axis-aligned bounding boxes and the sweep that pairs overlapping ones.

use std::collections::BTreeMap;

use datum_math::Vec3;

/// Corners `(min, max)`.
pub(crate) type Aabb = (Vec3, Vec3);

pub(crate) fn bounds(mut pts: impl Iterator<Item = Vec3>) -> Aabb {
    let first = pts.next().unwrap_or(Vec3::ZERO);
    pts.fold((first, first), |(lo, hi), p| {
        (Vec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z)), Vec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z)))
    })
}

/// Closed boxes share a point.
pub(crate) fn overlap(a: Aabb, b: Aabb) -> bool {
    a.0.x <= b.1.x && b.0.x <= a.1.x && a.0.y <= b.1.y && b.0.y <= a.1.y && a.0.z <= b.1.z && b.0.z <= a.1.z
}

/// Sorted pairs `(i, j)`, `i < j`, of overlapping boxes of different groups, by sweeping along X.
/// Every group keeps its own list of boxes crossing the sweep line, so boxes of one group are never
/// compared: a body of many triangles costs nothing against itself.
pub(crate) fn overlapping_pairs(boxes: &[Aabb], group: impl Fn(usize) -> usize) -> Vec<(usize, usize)> {
    let mut order: Vec<usize> = (0..boxes.len()).collect();
    order.sort_by(|&a, &b| boxes[a].0.x.total_cmp(&boxes[b].0.x).then(a.cmp(&b)));
    let mut pairs = Vec::new();
    let mut active: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for i in order {
        let (g, b) = (group(i), boxes[i]);
        active.retain(|&h, list| {
            if h != g {
                list.retain(|&o| boxes[o].1.x >= b.0.x);
                pairs.extend(list.iter().filter(|&&o| overlap(boxes[o], b)).map(|&o| (o.min(i), o.max(i))));
            }
            !list.is_empty()
        });
        active.entry(g).or_default().push(i);
    }
    pairs.sort_unstable();
    pairs
}
