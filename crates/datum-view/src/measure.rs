//! Measuring in the sketch (D177): the values of one picked shape, of two, and of regions. Pure
//! computation in sketch coordinates (mm, degrees); the window picks the shapes and shows them.

use datum_brep::Region;
use datum_math::Vec2;

/// What a click measures: a point (a sketch point, a centre, a pattern copy's point), a line, an
/// arc (counterclockwise around `center` from `from` to `to`), a circle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Shape {
    Point([f64; 2]),
    Line([f64; 2], [f64; 2]),
    Arc { center: [f64; 2], from: [f64; 2], to: [f64; 2] },
    Circle { center: [f64; 2], radius: f64 },
}

/// What a click of the measure tool took: a shape, or the region around the cursor.
#[derive(Clone, Debug)]
pub enum Taken {
    Shape(Shape),
    Region(Region),
}

/// `t` taken into what is measured, `add` (Shift) adding a region to regions: a click on nothing
/// starts anew, a region replaces shapes, a third shape starts a new measurement.
pub fn take(measured: &mut Vec<Taken>, t: Option<Taken>, add: bool) {
    let regions = !measured.is_empty() && measured.iter().all(|m| matches!(m, Taken::Region(_)));
    let keep = match &t {
        None => false,
        Some(Taken::Region(_)) => add && regions,
        Some(Taken::Shape(_)) => !regions && measured.len() < 2,
    };
    if !keep {
        measured.clear();
    }
    measured.extend(t);
}

/// The values of what is measured: one shape's, two shapes', or the regions' summed.
pub fn readings(measured: &[Taken]) -> Vec<(Quantity, f64)> {
    match measured {
        [] => Vec::new(),
        [Taken::Shape(a)] => one(a),
        [Taken::Shape(a), Taken::Shape(b)] => two(a, b),
        _ => regions(&measured.iter().filter_map(|m| if let Taken::Region(r) = m { Some(r) } else { None }).collect::<Vec<_>>()),
    }
}

/// The values as read: "Distance: 12.5", in the window's language.
pub fn lines(measured: &[Taken], i18n: &crate::i18n::I18n) -> Vec<String> {
    readings(measured)
        .into_iter()
        .map(|(q, v)| {
            let (key, english) = q.text();
            format!("{}: {}{}", i18n.t(key, english), number(v), if q.angle() { "°" } else { "" })
        })
        .collect()
}

/// One value of a measurement.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Quantity {
    Length,
    Radius,
    Diameter,
    Angle,
    Supplement,
    ArcLength,
    Distance,
    Dx,
    Dy,
    Centres,
    Gap,
    Area,
    Perimeter,
    Holes,
}

impl Quantity {
    /// The catalog key and English text of its name (D174).
    pub fn text(self) -> (&'static str, &'static str) {
        match self {
            Quantity::Length => ("measure-length", "Length"),
            Quantity::Radius => ("measure-radius", "Radius"),
            Quantity::Diameter => ("measure-diameter", "Diameter"),
            Quantity::Angle => ("measure-angle", "Angle"),
            Quantity::Supplement => ("measure-supplement", "180° −"),
            Quantity::ArcLength => ("measure-arc-length", "Arc length"),
            Quantity::Distance => ("measure-distance", "Distance"),
            Quantity::Dx => ("measure-dx", "ΔX"),
            Quantity::Dy => ("measure-dy", "ΔY"),
            Quantity::Centres => ("measure-centres", "Between centres"),
            Quantity::Gap => ("measure-gap", "Gap"),
            Quantity::Area => ("measure-area", "Area"),
            Quantity::Perimeter => ("measure-perimeter", "Perimeter"),
            Quantity::Holes => ("measure-holes", "Holes"),
        }
    }

    /// Whether it is in degrees.
    pub fn angle(self) -> bool {
        matches!(self, Quantity::Angle | Quantity::Supplement)
    }
}

fn dist(a: [f64; 2], b: [f64; 2]) -> f64 {
    libm::hypot(b[0] - a[0], b[1] - a[1])
}

/// The distance from `p` to the infinite line through `a` and `b` (to `a` if they coincide).
fn to_line(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let len = dist(a, b);
    if len == 0.0 {
        return dist(p, a);
    }
    ((b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])).abs() / len
}

/// Centre and radius of what acts as a circle.
fn round(s: &Shape) -> Option<([f64; 2], f64)> {
    match *s {
        Shape::Arc { center, from, .. } => Some((center, dist(from, center))),
        Shape::Circle { center, radius } => Some((center, radius)),
        _ => None,
    }
}

/// The values of one shape alone.
pub fn one(s: &Shape) -> Vec<(Quantity, f64)> {
    match *s {
        Shape::Point(_) => Vec::new(),
        Shape::Line(a, b) => vec![(Quantity::Length, dist(a, b))],
        Shape::Arc { center, from, to } => {
            let r = dist(from, center);
            let (u, v) = ([from[0] - center[0], from[1] - center[1]], [to[0] - center[0], to[1] - center[1]]);
            let t = libm::atan2(u[0] * v[1] - u[1] * v[0], u[0] * v[0] + u[1] * v[1]);
            let sweep = if t < 0.0 { t + std::f64::consts::TAU } else { t };
            vec![(Quantity::Radius, r), (Quantity::Angle, sweep.to_degrees()), (Quantity::ArcLength, r * sweep)]
        }
        Shape::Circle { radius, .. } => vec![(Quantity::Diameter, 2.0 * radius)],
    }
}

/// The values between two shapes.
pub fn two(a: &Shape, b: &Shape) -> Vec<(Quantity, f64)> {
    use Quantity::*;
    match (a, b) {
        (Shape::Point(p), Shape::Point(q)) => vec![(Distance, dist(*p, *q)), (Dx, (q[0] - p[0]).abs()), (Dy, (q[1] - p[1]).abs())],
        (Shape::Point(p), Shape::Line(l, m)) | (Shape::Line(l, m), Shape::Point(p)) => vec![(Distance, to_line(*p, *l, *m))],
        (&Shape::Line(a1, a2), &Shape::Line(b1, b2)) => {
            let (u, v) = ([a2[0] - a1[0], a2[1] - a1[1]], [b2[0] - b1[0], b2[1] - b1[1]]);
            let t = libm::atan2((u[0] * v[1] - u[1] * v[0]).abs(), u[0] * v[0] + u[1] * v[1]).to_degrees();
            let angle = if t > 90.0 { 180.0 - t } else { t };
            // A line pair that reads as parallel is measured by the gap between them (D177).
            if number(angle) == "0" {
                let mid = [(b1[0] + b2[0]) / 2.0, (b1[1] + b2[1]) / 2.0];
                vec![(Distance, to_line(mid, a1, a2))]
            } else {
                vec![(Angle, angle), (Supplement, 180.0 - angle)]
            }
        }
        (Shape::Line(l, m), c) | (c, Shape::Line(l, m)) if round(c).is_some() => {
            let (center, r) = round(c).unwrap_or_default();
            let d = to_line(center, *l, *m);
            vec![(Centres, d), (Gap, d - r)]
        }
        (Shape::Point(p), c) | (c, Shape::Point(p)) if round(c).is_some() => {
            let (center, r) = round(c).unwrap_or_default();
            let d = dist(*p, center);
            vec![(Centres, d), (Gap, (d - r).abs())]
        }
        _ => match (round(a), round(b)) {
            (Some((c1, r1)), Some((c2, r2))) => {
                let d = dist(c1, c2);
                let (big, small) = if r1 >= r2 { (r1, r2) } else { (r2, r1) };
                let mut v = vec![(Centres, d)];
                if d >= r1 + r2 {
                    v.push((Gap, d - r1 - r2));
                } else if d + small <= big {
                    v.push((Gap, big - small - d));
                }
                v
            }
            _ => Vec::new(),
        },
    }
}

/// The net area of regions (outer area minus holes, absolute values), the outer loops'
/// perimeter, and the number of holes, summed over the regions given.
pub fn regions(rs: &[&Region]) -> Vec<(Quantity, f64)> {
    let area: f64 = rs.iter().map(|r| r.outer.area().abs() - r.holes.iter().map(|h| h.area().abs()).sum::<f64>()).sum();
    let perimeter: f64 = rs.iter().map(|r| r.outer.perimeter()).sum();
    let holes: usize = rs.iter().map(|r| r.holes.len()).sum();
    vec![(Quantity::Area, area), (Quantity::Perimeter, perimeter), (Quantity::Holes, holes as f64)]
}

/// The region of `rs` holding point `p`: inside its outer loop and in none of its holes.
pub fn region_at(rs: &[Region], p: [f64; 2]) -> Option<usize> {
    let p = Vec2::new(p[0], p[1]);
    rs.iter().position(|r| r.outer.winding(p) != 0 && r.holes.iter().all(|h| h.winding(p) == 0))
}

/// A value as short as it reads (D177): at most three decimals, trailing zeros and point
/// dropped, "-0" as "0".
pub fn number(x: f64) -> String {
    let s = format!("{x:.3}");
    let s = s.trim_end_matches('0').trim_end_matches('.');
    if s == "-0" { "0".into() } else { s.into() }
}

#[cfg(test)]
mod tests {
    use super::*;
    use datum_brep::{ProfileLoop, TopoId};

    fn get(v: &[(Quantity, f64)], q: Quantity) -> f64 {
        v.iter().find(|(k, _)| *k == q).map(|&(_, x)| x).unwrap_or(f64::NAN)
    }

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() <= 1e-9
    }

    fn square(tag: &str, x: f64, y: f64, side: f64) -> ProfileLoop {
        let points: Vec<Vec2> = [(0.0, 0.0), (side, 0.0), (side, side), (0.0, side)].map(|(a, b)| Vec2::new(x + a, y + b)).to_vec();
        let id = |k: u64| TopoId::derive(tag).index(k).finish();
        ProfileLoop { vertices: (0..4).map(|k| [id(k), id(k + 8)]).collect(), sides: (0..4).map(|k| id(k + 16)).collect(), arcs: Vec::new(), points }
    }

    fn region(holes: Vec<ProfileLoop>) -> Region {
        Region { outer: square("outer", 0.0, 0.0, 10.0), holes, caps: [TopoId::derive("c0").finish(), TopoId::derive("c1").finish()] }
    }

    fn line(a: [f64; 2], b: [f64; 2]) -> Shape {
        Shape::Line(a, b)
    }

    fn circle(center: [f64; 2], radius: f64) -> Shape {
        Shape::Circle { center, radius }
    }

    #[test]
    fn numbers_read_short() {
        assert_eq!(number(12.0), "12");
        assert_eq!(number(12.5), "12.5");
        assert_eq!(number(12.34567), "12.346");
        assert_eq!(number(-0.0001), "0");
        assert_eq!(number(0.0), "0");
        assert_eq!(number(0.0005), "0.001");
        assert_eq!(number(100.0), "100");
    }

    #[test]
    fn names_and_units() {
        assert_eq!(Quantity::Dx.text(), ("measure-dx", "ΔX"));
        assert!(Quantity::Angle.angle() && Quantity::Supplement.angle() && !Quantity::Length.angle());
    }

    #[test]
    fn one_shape() {
        assert!(one(&Shape::Point([1.0, 2.0])).is_empty());
        assert_eq!(one(&line([0.0, 0.0], [3.0, 4.0])), vec![(Quantity::Length, 5.0)]);
        assert_eq!(one(&circle([1.0, 1.0], 2.5)), vec![(Quantity::Diameter, 5.0)]);
        let q = one(&Shape::Arc { center: [0.0, 0.0], from: [2.0, 0.0], to: [0.0, 2.0] });
        assert!(close(get(&q, Quantity::Radius), 2.0) && close(get(&q, Quantity::Angle), 90.0));
        assert!(close(get(&q, Quantity::ArcLength), std::f64::consts::PI));
        let q = one(&Shape::Arc { center: [0.0, 0.0], from: [0.0, 2.0], to: [2.0, 0.0] });
        assert!(close(get(&q, Quantity::Angle), 270.0));
    }

    #[test]
    fn points_and_lines() {
        let q = two(&Shape::Point([1.0, 1.0]), &Shape::Point([4.0, -3.0]));
        assert_eq!((get(&q, Quantity::Distance), get(&q, Quantity::Dx), get(&q, Quantity::Dy)), (5.0, 3.0, 4.0));
        let l = line([0.0, 0.0], [10.0, 0.0]);
        let p = Shape::Point([3.0, -2.0]);
        assert_eq!(two(&p, &l), vec![(Quantity::Distance, 2.0)]);
        assert_eq!(two(&l, &p), two(&p, &l));
    }

    #[test]
    fn line_pairs() {
        let a = line([0.0, 0.0], [10.0, 0.0]);
        let q = two(&a, &line([0.0, 0.0], [0.0, 5.0]));
        assert!(close(get(&q, Quantity::Angle), 90.0) && close(get(&q, Quantity::Supplement), 90.0));
        let t = std::f64::consts::PI / 6.0;
        let b = line([1.0, 1.0], [1.0 + libm::cos(t), 1.0 + libm::sin(t)]);
        let q = two(&a, &b);
        assert!(close(get(&q, Quantity::Angle), 30.0) && close(get(&q, Quantity::Supplement), 150.0));
        let r = two(&b, &a);
        assert!(close(get(&r, Quantity::Angle), 30.0) && close(get(&r, Quantity::Supplement), 150.0));
        // Opposite direction: still an undirected angle.
        let q = two(&a, &line([5.0, 3.0], [5.0 - libm::cos(t), 3.0 - libm::sin(t)]));
        assert!(close(get(&q, Quantity::Angle), 30.0));
        // Parallel: the gap between them.
        let q = two(&a, &line([20.0, 3.0], [2.0, 3.0]));
        assert_eq!(q, vec![(Quantity::Distance, 3.0)]);
    }

    #[test]
    fn circles() {
        let c = circle([0.0, 0.0], 2.0);
        let q = two(&c, &line([-5.0, 3.0], [5.0, 3.0]));
        assert_eq!((get(&q, Quantity::Centres), get(&q, Quantity::Gap)), (3.0, 1.0));
        let q = two(&line([-5.0, 1.0], [5.0, 1.0]), &c);
        assert_eq!(get(&q, Quantity::Gap), -1.0);
        let q = two(&Shape::Point([5.0, 0.0]), &c);
        assert_eq!((get(&q, Quantity::Centres), get(&q, Quantity::Gap)), (5.0, 3.0));
        let q = two(&Shape::Point([1.0, 0.0]), &c);
        assert_eq!(get(&q, Quantity::Gap), 1.0);
        let outside = two(&c, &circle([10.0, 0.0], 3.0));
        assert_eq!((get(&outside, Quantity::Centres), get(&outside, Quantity::Gap)), (10.0, 5.0));
        let inside = two(&circle([1.0, 0.0], 1.0), &circle([0.0, 0.0], 5.0));
        assert_eq!((get(&inside, Quantity::Centres), get(&inside, Quantity::Gap)), (1.0, 3.0));
        assert_eq!(two(&circle([0.0, 0.0], 5.0), &circle([1.0, 0.0], 1.0)), inside);
        let overlap = two(&c, &circle([3.0, 0.0], 2.0));
        assert_eq!(overlap, vec![(Quantity::Centres, 3.0)]);
        let arc = Shape::Arc { center: [10.0, 0.0], from: [13.0, 0.0], to: [10.0, 3.0] };
        assert_eq!(get(&two(&c, &arc), Quantity::Gap), 5.0);
        assert_eq!(get(&two(&arc, &Shape::Point([10.0, 7.0])), Quantity::Gap), 4.0);
    }

    #[test]
    fn clicks_add_up() {
        let (p, r) = (|x: f64| Some(Taken::Shape(Shape::Point([x, 0.0]))), || Some(Taken::Region(region(Vec::new()))));
        let mut m = Vec::new();
        take(&mut m, p(0.0), false);
        take(&mut m, p(3.0), false);
        assert_eq!(readings(&m)[0], (Quantity::Distance, 3.0));
        take(&mut m, p(5.0), false);
        assert_eq!(m.len(), 1, "a third shape starts anew");
        take(&mut m, r(), true);
        take(&mut m, r(), true);
        assert_eq!(readings(&m)[0], (Quantity::Area, 200.0), "Shift adds regions");
        take(&mut m, r(), false);
        assert_eq!(m.len(), 1);
        take(&mut m, p(0.0), true);
        assert!(matches!(m[..], [Taken::Shape(_)]), "a shape after regions starts anew");
        take(&mut m, None, false);
        assert!(m.is_empty() && readings(&m).is_empty());
    }

    #[test]
    fn region_values() {
        let rs = [region(vec![square("hole", 3.0, 3.0, 4.0)]), region(Vec::new())];
        assert_eq!(region_at(&rs, [1.0, 1.0]), Some(0));
        assert_eq!(region_at(&rs, [5.0, 5.0]), Some(1));
        assert_eq!(region_at(&rs, [20.0, 5.0]), None);
        assert_eq!(region_at(&rs[..1], [5.0, 5.0]), None);
        let q = regions(&[&rs[0]]);
        assert_eq!(q, vec![(Quantity::Area, 84.0), (Quantity::Perimeter, 40.0), (Quantity::Holes, 1.0)]);
        let q = regions(&[&rs[0], &rs[1]]);
        assert_eq!(q, vec![(Quantity::Area, 184.0), (Quantity::Perimeter, 80.0), (Quantity::Holes, 1.0)]);
    }
}
