//! The exact path runs only when the `f64` filter cannot settle a sign. Alone in its binary: the
//! fallback counter is process-wide.

use datum_math::{Lpi, Point, Sign, Vec3, exact_fallbacks, orient3d};

#[test]
fn exact_path_only_for_degenerate_signs() {
    // Deterministic pseudo-random points in general position.
    let mut state = 0x2545_f491_4f6c_dd1du64;
    let mut next = || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        (state >> 11) as f64 / (1u64 << 53) as f64 * 200.0 - 100.0
    };
    let mut v = || Vec3::new(next(), next(), next());
    let before = exact_fallbacks();
    let mut decided = 0;
    for _ in 0..1000 {
        let (p, q, r, s, t) = (v(), v(), v(), v(), v());
        let Some(l) = Lpi::new(p, q, r, s, t) else { continue };
        let [a, b, c] = [v(), v(), v()].map(Point::Explicit);
        if orient3d(&Point::Lpi(l), &a, &b, &c) != Sign::Zero {
            decided += 1;
        }
        assert_eq!(orient3d(&Point::Explicit(r), &Point::Explicit(s), &Point::Explicit(t), &Point::Lpi(l)), Sign::Zero);
    }
    assert_eq!(decided, 1000);
    // Only the "on its own plane" checks, one per point, needed exact arithmetic.
    assert_eq!(exact_fallbacks() - before, 1000);
}
