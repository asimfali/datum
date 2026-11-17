# Loft, step 2 — one line → two angled lines (first bend)

## Goal

The first fold line of the loft op (D264): the transition produces two planar
triangular faces with a bend between them.

## Inputs

- profile A: one line;
- profile B: two lines meeting at a vertex; the vertex sits above the middle
  of A (to be confirmed with the author before code).

## Result

- two planar triangular faces;
- the shared internal edge is a bend line — a bend with the rule radius
  (not sharp: production, D181).

## Done when

- [ ] renders (PNG, 2–3 views) before any code;
- [ ] unit tests, reference model in `tests/models/`, blessed hashes;
- [ ] `scripts/check.sh` exit 0.

## Notes

- General matching of open chains (different segment counts, ends not over
  ends) is deliberately NOT here — later step, after a third real case
  (minimalism rule).
