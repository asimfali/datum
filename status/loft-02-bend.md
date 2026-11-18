# Loft, step 2 — one line → two angled lines (the fan from the apex)

## Goal

The second case of the loft op (D264): the transition from one straight
profile to a two-segment angled profile (a V), the first real bends.

## Inputs

- profile A: one line (the reference model: length 10, world (-5,0,0)→(5,0,0));
- profile B: an open chain of two lines meeting at a vertex (the apex; the
  reference model: legs 10 long, 20° off the horizontal, apex at (0,15,0));
- the apex sits over the middle of A (step-2 restriction; general matching of
  open chains comes later, after a third real case).

## Result (the contract, confirmed by the author's Inventor example 2026-11-17)

- **three planar triangular facets**: the middle one (A's start, A's end, the
  apex) lies in A's plane; two wings (A's end, the apex, B's end);
- **two cylindrical bends** along the apex→A's-ends lines, the rule's radius
  (not sharp: production, D181), K-factor of the rule for the development;
- the bend's end at A's edge is a square cut of the thickness (the bend runs
  out onto the side edge; the author confirmed the step on the corner);
- thickness on `side` of the sheet, as in step 1 (left / right / middle);
- the development through `unfold_side`: the flat fan with two bends, the seam
  by the rule of developments of bodies of revolution.

## Refusals

- the apex on the line A (the middle facet has no area) → `Degenerate`;
- a leg of B lying on a bend line (a wing has no area) → `Degenerate`;
- profiles outside the case (arcs, more segments, the apex off the middle) →
  `Unsupported` with names.

## Done when

- [ ] unit tests, reference model in `tests/models/`, blessed hashes;
- [ ] `scripts/check.sh` exit 0.

## Notes

- The author said renders are not needed (the Inventor screenshots were the
  reference); the old done-when "renders before code" is dropped.
- The old assumption "two facets, one bend" is wrong: with the apex over the
  middle and B's ends lifted, the wing quad is not planar (checked by numbers,
  2026-11-17). The fan from the apex to A's ends is the Inventor construction.
- The author's test file `C:\Users\Simanov\datum-demo\part\loft(1 to 2
  line)(2 stage).json` duplicates a leg (`l1`/`l2`, one segment twice); the
  reference model takes the clean chain `p1 → apex → p2`, the length on one
  leg, `equal` on the pair.
- Built on the branch of the closed draft PR #261: the layout validation, the
  two-line recognition and the Left/Right/Middle tests there are reused.
