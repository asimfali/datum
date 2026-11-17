# Loft, step 2 — one line → two angled lines (three facets, two bends)

## Goal

Extend the first D264 loft case to the next concrete open-profile pair. One
straight segment of profile A transitions to a two-segment polyline of profile
B. The result is a connected sheet of **three planar triangular facets and
two cylindrical bends**. This topology was clarified by the author on
2026-11-17; the earlier two-facet / one-bend wording was wrong.

## Inputs

- profile A: exactly one straight line;
- profile B: exactly two straight lines sharing one endpoint;
- the shared vertex of B lies above the midpoint of A;
- the two outer endpoints of B correspond to the two endpoints of A;
- the profiles are placed so the three resulting triangular panels are
  non-degenerate and the two bend axes have positive length.

The first fixture should retain the dimensions of step 1 where possible:
profile A is 20 mm, profile B spans 30 mm, and the sketches are 15 mm apart.
Choose a non-zero apex height for the angled profile and record it in the
reference model.

## Result

- three planar triangular facets:
  - left end triangle: A-start, B-start, B-vertex;
  - middle triangle: A-start, A-end, B-vertex;
  - right end triangle: A-end, B-vertex, B-end;
- two real cylindrical bends along the internal edges:
  - A-start ↔ B-vertex;
  - A-end ↔ B-vertex;
- each bend uses the inner radius from the sheet rule and the outer radius
  offset by sheet thickness;
- the bends meet at the B vertex; their shared corner must be trimmed and
  sewn as one manifold junction, not as two overlapping independent caps;
- the finished sheet has consistent thickness and its flat development uses
  the rule's K-factor.

## Geometry constraints

- No cone or sharp crease may substitute for either cylindrical bend.
- The three planar facets must remain planar and must not be replaced by a
  single ruled surface.
- The bend tangent boundaries and the common corner relief must be derived
  from the same exact geometric construction as the two cylinders.
- Reject zero-area triangles, zero-length bends, an invalid fold angle, or a
  bend radius that consumes a facet with `Err(Degenerate)`.
- Do not weaken `Body::validate()` or introduce an arbitrary epsilon to make
  the shared corner pass.

## Done when

- [x] The author supplied the topology reference: three triangular facets and
  two cylindrical bends.
- [x] A pre-code three-view topology preview exists.
- [ ] A reference JSON model is added under `tests/models/`.
- [ ] Unit tests cover the three facets, two cylinders, their common corner,
  thickness, and rule-driven radius/K-factor.
- [ ] The body passes `Body::validate()` on all three `Side` choices.
- [ ] `unfold_side` develops both bends and preserves bend lengths at the
  rule's K-factor.
- [ ] Reference hashes are blessed only after `scripts/check.sh` passes before
  blessing, and checked again afterward.
- [ ] Linux and Windows CI pass.

## Notes

- General matching of arbitrary open chains (different segment counts,
  endpoints not over endpoints) is deliberately out of scope. This step only
  handles one line against exactly two connected lines.
- The common B vertex is a genuine two-bend junction. Its end trimming is the
  main topological risk; do not build the two bends independently and simply
  fuse their end caps.
