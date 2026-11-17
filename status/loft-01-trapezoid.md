# Loft, step 1 — two lines → trapezoid (D264 skeleton)

## Goal

The skeleton of the loft-flange op (D264) on the degenerate case: the transition
between two profiles, each a single line, on two parallel planes. Almost all of
the op plumbing is solved here without any curved surface.

## Inputs

Author's test model `C:\develop\datum-test\Детали\loft(1 to 1 line)(first stage).json`
(two sketches + a plane):

- sketch1 (base XY): line 20 mm, horizontal, midpoint at sketch origin →
  along X, x ∈ [−10, 10], y = 0, z = 0;
- plane1: base XZ, offset 15 (normal −Y) → plane y = ±15;
- sketch2 on plane1: line 30 mm, horizontal, midpoint at sketch origin →
  along X, x ∈ [−15, 15], z = 0.

Both lines are parallel to X, both lie in z = 0: the four ends are coplanar,
the transition is ONE flat quadrilateral face — a trapezoid. Not split into
triangles: the D264 marking rule (triangle over each straight segment) applies
where there is a fold; splitting a flat face would create fake bends.

Stored point coordinates in the file diverged from its constraints (same as
жалюзи2) — datum solves constraints; verify the solved geometry on the built
body, read the structure, not the numbers.

## Result

- a new sheet body (sheet rule, thickness/side as in обечайка, D264);
- one planar face (Surface::Plane), thickness faces and edge treatment per
  the sheet rule;
- the flat development (layer 0, D186) of the body.

## Done when

- [ ] the author's model file transferred into `tests/models/` (English name,
  e.g. `loft-01-line.json`);
- [ ] the trapezoid body is fully built in datum (author verifies the result
  in datum itself — no PNG renders in this step);
- [ ] the development (unfold) works on it;
- [ ] `Op` variant + window + MCP wiring (`add-op` path);
- [ ] parallel-planes check, honest `Err(Degenerate)` otherwise (D264);
- [ ] unit tests, reference model blessed, `scripts/check.sh` exit 0.

## Notes

- Ends of open profiles stay open: no end walls.
- Next step (one line → two angled lines) starts ONLY after this step is
  fully done.
