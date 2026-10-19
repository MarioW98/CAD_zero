# ADR-0028: SDF bounds contract includes smooth-blend dilation

## Status

**Accepted (2026-10-19).** The decision was taken before this record
was written; the ADR registers it, it does not propose it. Delivery
order: this document (commit 1), the kernel fix + operator audit
(commit 2), the pinned-sweep re-measurement (commit 3), and the
property-suite blind-spot closure (commit 4).

## Context

`field.hpp` defines the third pillar of the `SDFNode` contract:

> `bounds()` — *Local-space bounding box of the region where the
> field can be negative (inside the solid). Used to cull evaluation
> outside the solid of interest.*

Two consumers build on it: `mesh_extract.cpp` (the marching-cubes
auto-bounds grid: `bounds()` + a 1%-of-extent pad) and the ADR-0026
bridge (`sdf_to_brep`, same layout, plus the `min_abs_field` P2
margin mirror). The ray-marcher and the scene-level framing consume
it as well. **P1 — the extraction volume contains the whole surface —
is built on this contract.**

The property extension over `smin k > 0` (FASE 1.1 of the smin
families, 2026-10-18) measured the violation empirically and pinned
it:

* The polynomial blend dips **below** `min(f_a, f_b)` by up to
  **k/16** — with `u = (f_b - f_a)/k`, the deviation is
  `v - min = k * phi(u)`, `phi(u) = (8u - 7u^2 - 1)/16`, minimum
  `-1/16` at `u = 0` (equal operand fields).
* Concentric unit spheres (the pure `u = 0` case): the zero crossing
  sits at **exactly `1 + k/16`** (measured `1.006250 / 1.031250 /
  1.062500` for `k = 0.10 / 0.50 / 1.00`) — **outside** the old
  `[-1,1]^3` hull, and beyond the extractor's 1%-of-extent pad
  (`0.02` here) for `k >~ 0.32`.
* Nested blends accumulate the dip inductively: the total field
  deviation from the hard-min tree is bounded by the envelope
  **`sum(k_i)/16`** over all blend levels.

`SmoothMinSDF::bounds()` returned the plain hull of the children's
boxes. For near-coincident operand surfaces the negative region
escapes that hull — **the bounds() contract is violated**, and P1
inherits the lie. A bounds() that does not contain the negative
region is a lying contract; the contract wins.

## Decision

**`SmoothMinSDF::bounds()` dilates the children's hull by `k/8`
per blend level** (isotropic, applied only for `k > 0`; `k = 0` is
the hard min, where the plain hull is exact).

### Why k/8 and not k/16 — the 2x margin

A point `p` with `smin(p) < 0` satisfies `min(f_a, f_b)(p) < k/16`,
hence `dist(p, neg-child) <= (k/16) / L_child` where `L_child` is
the child subtree's Lipschitz constant. Every node in this library
keeps `L >= 1`: primitives are unit-Lipschitz; union/intersection/
subtraction (min/max), translate, rotate and scale preserve `L`;
`smin` multiplies it by `2.5`; twist by `(1 + 2|k|)`. The escape in
*distance* is therefore at most `k/16`, and `k/8` covers it with
headroom. The margin judgement: **the cost of over-estimating is a
few extra grid samples; the cost of under-estimating is a surface
outside the contract** — the asymmetry that produced this ADR in the
first place. The 2x also buys robustness against future operators
with `L < 1` down to `L >= 0.5`.

### Composition along nested blends

By construction: the parent takes the hull of the children's
**already-dilated** boxes and adds its own `k/8`. The parent's dip
applies to the *fields* (at most `k_p/16` below the min of the child
fields), never to the boxes, so the child dilation is never
re-applied. By induction the root box contains
`hull ⊕ sum(k_i)/16` with the full 2x margin.

### Operator audit (delivered with the fix, commit 2)

Every operator that mixes or warps fields was audited against the
contract; the verdicts live as comments at each `bounds()`:

| Operator | Verdict | Reason |
|---|---|---|
| `UnionSDF` | immune | `min < 0` iff some child is; negative region = union of the children's — inside the hull |
| `IntersectionSDF` | immune | `max < 0` iff both are; subset of each child's region — inside the box intersection |
| `SubtractionSDF` | immune | `max(a,-b) < 0` iff `a<0 ∧ b>0`; subset of a's region — inside a's box alone |
| `SmoothMinSDF` | **fixed** | the k/16 dip; dilate by k/8 |
| `TranslateSDF` | immune | rigid shift of region and box together |
| `RotateSDF` | immune | rigid rotation; corner hull of the rotated child box contains it |
| `ScaleSDF` | immune (s>0) | sign never flips; region and box scale identically (`s <= 0` is an unvalidated degenerate mirror, outside the input class) |
| `TwistSDF` | **known gap, reported** | the warp rotates each `y`-slice by `y·k`; an off-axis child's region can leave the box entirely. Nothing today extracts a twisted off-axis body (the canonical native scene twists at `k = 0.05`, displacement ~0.03, save/load only). The correct dilation is slice-dependent and needs its own decision + pinned sweep — **deferred to a future ADR, not silently ignored** |

## Consequences

1. **Pinned blended counts shift once (commit 3).** The extraction
   grid for blended trees widens by `sum(k_i)/8` per side, so V/E/F
   pins measured on the old layout are re-measured ONCE with a
   throwaway driver and re-frozen (ADR-0025 v5 method). The
   **invariants do not move** — χ, orientability, closedness,
   `E = 3F/2`, `V = F/2 + 2`, `min|f| > 0`, bit-identical
   determinism: they are the proof that the shift is one of grid
   *layout*, not of geometry. Measured outcome: T7/T8/T10
   `1310/3924/2616 -> 1230/3684/2456`, T8b `574/1716/1144 ->
   518/1548/1032`, T9 `5334/15996/10664 -> 4862/14580/9720`; the
   legacy tangent-pair pin `350/1044/696` (d=2, k=0.5, res=8) is
   **unchanged** — the wider box does not change that grid's
   topology.
2. **Non-blend pins are the control.** Spheres, boxes, hard CSG, the
   pierced box, the bridge T1–T6 fixtures and the box-family property
   must not move by a single unit (their `bounds()` paths are the
   audited-immune operators). Verified: the sphere res-16
   `min_abs_field` pin `0.00393593311` and the res-8 `270/804/536`
   pin are bit-identical after the fix. Any movement there would be a
   new bug, not part of the sweep.
3. **P2 margins of blended cases re-frozen.** The honest grid now
   covers the blend shell, so the reported `min|f|` changes meaning:
   T10 carries the new frozen pin `0.00277030468`; the T4-style grid
   check (P1 enclosure + P2 no on-grid zeros) passes on the new
   layout for every re-measured configuration.
4. **Property suite closes the blind spot (commit 4).** The blended
   positivity property (g) asserts against the NEW bounds; a NEW
   stratified property samples at distance bands from the bounds()
   boundary — `[0, eps]`, `[eps, 2eps]`, ... with a declared step —
   densely covering the thin band, because the k/16 shell is exactly
   the violation class that uniform external sampling cannot see.
   The guinea-pig k-sweep (f) is unchanged: the slope does not depend
   on bounds().
5. **Goldens and the native format are untouched.** `bounds()` is
   computed, never serialized; the v0.1/v0.2 canonical scenes contain
   `smooth_min` nodes but only round-trip bytes. No format bump, no
   migration.
6. **The ADR-0026 P1 clause is now honest by construction** for
   blended bodies: the extraction volume derives from a bounds() that
   contains the negative region, not from the hull that happened to
   work for the declared chain class.
