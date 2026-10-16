# ADR-0026: SDF ↔ B-Rep bridge — Phase E, SDF → B-Rep extraction MVP

## Status

**Implemented (2026-10-16) — MVP, app consumer and smooth-union
follow-up landed; the § Consequences min |f| left-over is resolved.**

Accepted 2026-10-16 with two corrections applied on acceptance (see
§ Corrections below). Implementation, in order: the MVP (faceted
rewrite to the topology-era contract, tests T1–T6), the app consumer
(nanobind bindings + the Debug-menu topology report action), and the
Phase E closure (the smooth-union follow-up, tests T7–T10 — which
uncovered and fixed a k>0 clamp-sign bug in `SmoothMinSDF` — plus the
machine-readable `min_abs_field` P2 margin, T11). The bridge contract
is tested by `tests/unit/bridge/test_bridge.cpp` (T1–T11) and
`bindings/python/tests/test_bridge_report.py`.

## Corrections on acceptance (2026-10-16)

1. **D5 — CI matrix.** The draft claimed bit-exactness across
   "GCC 11 / GCC 14 / Clang 14 (the CI matrix)". The real matrix is
   **GCC 11.4 and Clang 14** (ubuntu-22.04; presets `linux-gcc-debug`,
   `linux-gcc-release`, `linux-clang`). GCC 14.2 is the local
   development compiler, not part of the matrix. The D5 text is
   corrected in place. Decision: GCC 14 is **not** added to the CI
   matrix now — a matrix change deserves its own CI observation
   cycle, and nothing in the contract depends on it (sqrt-only
   arithmetic is IEEE-754 correctly-rounded on all three compilers).
2. **P2 — actionable errors.** The on-grid-zero failure surfaced by
   the bridge (degenerate triangle, or a non-manifold assembly
   downstream of an on-grid zero) must be an **actionable runtime
   error**: the message names the likely cause and the remedy —
   offset the sampling grid (e.g. translate the SDF by a small
   generic amount) or change the resolution — never a bare
   assertion. Reflected in § Proposed MVP step 2 and in the
   implementation's error contract.

## Context

ADR-0002 established the dual-kernel vision: SDF and B-Rep as peer
representations — SDF for modeling (primitives, CSG, smooth blends,
ray-marched preview), B-Rep for exact boundary work (topology queries,
booleans with exact contracts, seam-wired analytic surfaces, export).
Phase E is the phase where the two kernels actually meet.

What each side brings to the bridge:

- **SDF side (Phase B/B.5, ADR-0015/0016)**: MC33 extraction
  (Chernyaev 1995 / Lewiner et al. 2003) with asymptotic decider and
  `v12` center vertices; canonical edge-key vertex welding (integer
  keys, no FP tolerance); lexicographic endpoint canonicalization in
  `interp_edge_abs` so that two cells sharing a physical grid edge
  compute the **bit-identical** crossing position; narrow-band culling;
  a global signed-volume orientation pass.
- **B-Rep side (Phase D, ADR-0021/0022/0023/0025 v5)**: radial-edge
  topology with `link_radial_edges` post-pass; the 4-invariant
  `is_closed_manifold` contract (every edge exactly 2 co-edges, coedge
  orientation XOR, loop closure, χ = 2); convex booleans whose assembly
  pipeline (canonical vertex welding via `SharedVertexPool`, edges
  keyed by vertex pairs, T-junction subdivision) is the proven pattern
  for turning polygon soups into valid closed shells.

**A stub bridge already exists** (`core/bridge/`):
`sdf_to_brep` (faceted: 1 triangle → 1 `Face` on a `PlaneSurface`, plus
primitive fitting by `describe()` string parsing) and `brep_to_sdf`
(primitive detection + brute-force `MeshSDF`). The stub **predates
Phase D** and ignores everything Phase D established: it never calls
`link_radial_edges`, sets `shell.set_closed(true)` unconditionally,
derives coedge `reversed` flags from vertex-index order instead of the
edge's intrinsic direction, and silently `continue`s past degenerate
triangles (leaving holes while still claiming closure). Its tests
assert only `face_count() > 0` and surface type names — no topological
invariant has ever been asserted on bridge output. It is a
rendering-era leftover wearing a topology-era interface.

The app today is SDF-primary (ray-marched viewport, parametric SDF
feature tree); no user-facing B-Rep authoring exists. The `.CAD_0`
native format (ADR-0007) serializes SDF feature trees, not B-Rep
bodies.

**Design principle for the whole phase — reuse, not reinvention.** The
bridge does not invent a new topological assembly. It reuses the
ADR-0025 v5 pattern: canonical vertex identity before edge keying,
edges keyed by canonical vertex pairs, `link_radial_edges` as the single
radial-population point, and the 4-invariant contract as a hard
postcondition. In the extraction path this machinery *degenerates to
identity by construction* — the MC welder has already produced shared
vertex indices — which is precisely why this path is the safe first
crossing.

## Decision

### D1. Direction of the bridge: SDF → B-Rep first

**Choice**: SDF → B-Rep (extraction) is the Phase E MVP direction.
B-Rep → SDF remains a stub and gets its own ADR later.

Why this direction serves the system first:

1. **The app's shapes live on the SDF side.** Every shape a user can
   create today exists as an SDF tree. The only route by which those
   shapes can ever reach exact-topology land — query operators, convex
   booleans with χ = 2 contracts, seam-wired analytic surfaces — is
   SDF → B-Rep. The reverse direction has no consumer yet: B-Rep
   bodies exist only as factory outputs and boolean results, there is
   no B-Rep authoring UI and no B-Rep feature tree.
2. **The extraction half of the pipeline is the battle-tested half.**
   MC33 + integer-keyed welding + canonicalized interpolation have been
   exercised since Phase B by every STL/OBJ/3MF export; the topology
   half of the bridge is the part that needs building, and it reuses
   Phase D machinery (see principle above).
3. **It is the first real peer-to-peer operation of the dual kernel**
   (ADR-0002): smooth SDF-modeled shapes become inputs to B-Rep tools.

**Rejected alternatives:**

- *B-Rep → SDF first* — rejected: (a) no app consumer; (b) the
  existing `MeshSDF` is not an honest SDF — its sign comes from a
  majority-of-normals vote, which is wrong for concave bodies, and it
  is not distance-bounded outside a narrow band; a credible B-Rep →
  SDF needs exact distance-to-surface plus a generalized-winding-number
  sign (Jacobson et al. 2013) — research-grade scope with zero
  immediate users; (c) prototyping blends on B-Rep bodies can already
  be approximated today via tessellation → `MeshSDF`.
- *Both directions in parallel* — rejected: doubles the review surface
  of exactly the phase that is supposed to be the stress test of the
  topology contracts.

**Consequences for the other direction, later:** B-Rep → SDF will need
its own ADR (exact surface distance, winding-number sign, Lipschitz
bound for the field contract). The naming decisions of D4 are
direction-specific and must not constrain that ADR: extraction assigns
genealogy for SDF-side features, and the reverse bridge will assign
genealogy for B-Rep-side features, on the same ADR-0005 ABI.

### D2. Manifoldness of the extract

**Choice**: an **input contract** (preconditions P1–P4 on the SDF and
the grid) plus **hard topological postconditions on the output**,
asserted in tests. No output sanitization in the MVP.

What the MC33 pipeline already guarantees by construction:

1. **Topological consistency of the table.** MC33 resolves face and
   interior ambiguities via the asymptotic decider — adjacent cells
   cannot disagree about the topology of the shared face, so the
   stitched surface has no cracks.
2. **Exact integer-keyed vertex identity.** One vertex per grid-edge
   crossing, shared by every triangle that uses it (ADR-0016 canonical
   edge keys). No tolerance-based merging exists anywhere in the mesh
   path: distinct points can never merge, and one point can never
   split within a grid edge.
3. **Bit-identical crossing positions across cells.** `interp_edge_abs`
   canonicalizes the edge endpoints lexicographically, so the two
   (or four) cells sharing a physical grid edge compute the same `t`
   and the same position bit-for-bit — the IntersectionTable philosophy
   (ADR-0025 §6) applied at the MC level.
4. **Consistent orientation.** Per-cell triangulations are emitted with
   consistent winding, and a global signed-volume check flips all
   triangles if the volume comes out negative.

Given 1–4, the welded MC output is a closed 2-manifold with χ = 2 if
and only if the input satisfies:

- **P1 — enclosure.** The surface lies strictly inside the padded
  sampling volume. A surface that exits the volume yields an open
  shell (χ ≠ 2). The extractor's `bounds() + padding` covers this for
  compact SDFs with honest bounds.
- **P2 — no on-grid zeros.** No grid point evaluates to exactly
  `0.0f`. The extractor classifies `f >= 0` as outside; an exact zero
  puts the crossing exactly AT the grid point, where up to four
  distinct grid edges produce the same position with **different**
  canonical keys — distinct coincident vertices, and T-junction-style
  overlapping edges with more than 2 co-edges. This is the same
  pathology the boolean kernel *repairs* by subdivision (ADR-0025
  §10); extraction *rejects* it by precondition, because extraction
  controls its own grid while boolean clipping cannot forbid its
  inputs from creating T-junctions. For the MVP, P2 is **assertible
  directly**: the test plan checks min |f| > 0 over the pinned grids.
- **P3 — feature separation vs cell size.** Two surface sheets closer
  than about one cell can pinch into a singular vertex (non-manifold
  vertex link) or merge into one sheet. Contract: the caller chooses a
  resolution whose cell size is below the minimum feature thickness.
  The MVP sphere satisfies this trivially.
- **P4 — sign fidelity.** The field's sign at grid points must agree
  with inside/outside of the intended solid. A sign error creates
  spurious components (each closed, so χ jumps to 4, 6, … — the
  invariant check catches it, but the geometry is wrong). Lipschitz
  discipline in the SDF operators keeps this honest.

**Why not output sanitization for the MVP**: sanitization (vertex
merging, hole stitching, pinch splitting) silently changes geometry
and duplicates what P1–P4 already prevent for the supported input
class. The boolean kernel needed sanitization because clipping
*creates* near-degenerate output; extraction does not — its
degeneracies are all preventable at the input. Sanitization is
deferred until a real extraction case demonstrates need, and then it
gets its own ADR: it is topology surgery, not hygiene.

**The contract, stated for the MVP (sphere → B-Rep)**: V/E/F depend on
the grid; **χ = 2, `is_closed_manifold`, and `loops_are_closed` do
not** (for the supported input class). The tests assert both facts:
exact V/E/F for pinned (SDF, grid) configurations, and the invariants
at multiple resolutions and generic (off-grid) placements. Degenerate
triangles (zero area, repeated vertex) are contract violations under
P2 → visible errors, never silent skips — the stub's `continue` is
what turned a hole into a lie.

### D3. Face granularity

**Choice**: 1:1 triangles for the MVP — each MC triangle becomes one
`Face` on a `PlaneSurface`.

- **Manifoldness is provable per-triangle.** Planar merging requires
  boundary tracing along loops (a feature-extraction algorithm with
  none of the Phase D machinery behind it) and would be the riskiest
  part of the phase, for zero contract value: nothing downstream
  distinguishes a merged 6-face box from a 12-triangle one — the
  boolean kernel treats faces as polygons either way.
- **Memory.** From ADR-0016 measurements at res = 64 (welded): about
  77k vertices / 95k triangles, hence E = V + F − 2 ≈ 172k edges. The
  radial-edge overhead over a half-edge layout is ~16 bytes per edge
  plus heap per edge (ADR-0012) — order 10 MB at res = 64. Acceptable
  for an MVP bridge whose output is *not* on any interactive path: the
  app viewport ray-marches the SDF and never renders the B-Rep.
- **Query operators.** `faces_around_edge` is O(radial list) per edge
  — independent of face count. What grows with triangles is the
  full-body traversals (`is_closed_manifold`, tessellation): linear,
  fine at MVP scale, and resolution is caller-controlled.
- **`.CAD_0`.** The native format serializes SDF feature trees, not
  B-Rep bodies — today. No format change in the MVP. When B-Rep
  serialization gets its own ADR, 1:1 triangles make the writer
  trivial (positions + 3-coedge loops) and planar merging can be a
  lossless post-pass on top of it.
- **Honesty note.** The current stub header advertises "detecting
  planar patches (groups of coplanar triangles)" — never implemented;
  the stub already does 1:1 triangles. This ADR ratifies the real
  behavior; the stale comment and the unused
  `SdfToBrepOptions::planar_tolerance` are retired at implementation
  time.

**Rejected alternatives:** planar/quad merging now (risk, no consumer);
dual-style quads (that is Dual Contouring territory — ADR-0015
deferred it; it changes vertex placement and loses the on-edge
interpolation guarantee, weakening the D5 determinism argument).

**Consequences**: bridge output at res = 64 is heavy (~95k faces) and
that is fine and documented; anyone needing a compact exact B-Rep uses
the primitive-fit path (sphere/box/cylinder) or a lower resolution.
Planar merging = future ADR when a consumer exists.

### D4. Naming and ADR-0005

**Choice**: the MVP promises exactly this, and nothing more:

1. **Genealogy.** Every extracted body carries one `FeatureId` for the
   extraction feature — stable across rebuilds, invalidated only by
   explicit feature deletion (ADR-0005).
2. **Deterministic order.** `SubshapeRef`s are assigned in extraction
   order (deterministic grid traversal): face *i* = the *i*-th
   triangle emitted; vertex/edge ids follow the canonical weld order.
   Same (SDF, grid) → bit-identical ids (D5). Within a generation,
   ids are stable and meaningful.
3. **No stability across parameter edits.** Changing the radius
   changes V/E/F, so index-based `SubshapeRef`s shift. The MVP does
   **not** attempt subshape tracing through extraction. Re-extract
   after an edit = a new `Generation`; old references dangle and are
   surfaced as user-visible dangling errors — never silently rebound
   (ADR-0005's generation rule is the whole protection).

**Rejected alternatives:**

- *Stable IDs derived from field values* (hash of position, or of the
  crossing's grid edge + parameter) — **rejected**: this is
  geometric-signature naming, the documented "wrong turn" of ADR-0005
  (Andrade 1998; Chen & Luo 2010). A radius edit moves every vertex;
  any position-derived key is either broken or secretly approximate
  matching — and approximate matching is the bug generator ADR-0005
  exists to prevent.
- *Seed-based identity* (stable id per grid-cell crossing, matched
  across rebuilds) — rejected: the set of crossings changes
  discontinuously with parameters; matching cells across rebuilds is
  approximate matching with extra steps. A future "topological
  tracing through extraction" (e.g. cell-provenance for *unchanged*
  cells) is a legitimate Phase F research item and is explicitly
  deferred.

**The honest MVP promise**: determinism (same inputs → same ids) and
genealogy (every subshape traces to the extraction feature).
**Explicitly deferred**: identity preservation across parameter edits
(Phase F subshape tracing).

### D5. FP contract (ADR-0014)

**Choice**: bit-exact determinism for fixed (SDF, grid) on the
supported MVP arithmetic class; **no binary golden file** — exact-count
assertions in the test source are the golden.

Is extraction bit-exact for fixed (SDF, grid, seed)?

- **Grid evaluation** writes disjoint slots per point (no reductions,
  no chunk-then-reduce sums) → thread-count independent.
- **The sphere path's arithmetic** is `dot + sqrt` (and one division
  for the gradient): with `-ffp-contract=off` (ADR-0014) the dot
  product is deterministic, and `std::sqrt` is IEEE-754
  correctly-rounded → bit-exact across the CI matrix (GCC 11.4 and
  Clang 14, ubuntu-22.04; acceptance correction 1). GCC 14.2, the
  local development compiler, is not part of the matrix. No libm
  transcendentals are involved.
- **MC33 traversal** is single-threaded in fixed cell order; each
  crossing is one interpolation whose endpoints are canonicalized
  (bit-identical across sharing cells, by construction — not by
  tolerance).
- **Welding** uses integer canonical keys — no FP in identity.
- **The orientation pass** is a deterministic function of the same
  mesh.

⇒ **Yes, bit-exact, for SDFs whose `sample()` uses only +, −, ×, ÷,
sqrt** (the MVP sphere qualifies). General SDFs that call
`std::sin`/`std::cos` (rotations, some transforms) inherit the ≤ 1 ULP
platform variance documented in ADR-0014: sign classification is
expected to be stable but positions are not contractually bit-exact.
The determinism contract is scoped to the deterministic-arithmetic
class, and the test plan stays inside it. About the "seed" slot:
extraction contains **no RNG** — (SDF, grid) fully determine the
output today; introducing any nondeterminism source would require
amending this ADR.

**Golden file — no, and why:**

1. A binary golden couples the regression test to every float bit of
   every vertex position *and* to a serialization format. The contract
   that matters — exact V/E/F, χ = 2, `is_closed_manifold`,
   `loops_are_closed` — is **integer-valued** and is asserted directly
   in the test source, with the same regression power and a fraction
   of the brittleness.
2. The native format does not serialize B-Rep bodies yet; an
   STL-hash golden would test the IO layer, not the bridge, and would
   float on ADR-0014's 12-decimal quantization policy.
3. When `.CAD_0` gains B-Rep serialization, its own determinism test
   (per ADR-0014) subsumes the binary-golden question — that decision
   belongs to that ADR, not this one.

**Consequences**: bridge tests pin exact counts for pinned (SDF,
grid), measured once at implementation time (throwaway driver, same
method as the ADR-0025 v5 exact-count tests) and then frozen in the
test source. Any future change to the MC tables, the welder, or the
traversal order is a contract change and must update the pinned counts
consciously — the tests make it visible, not silent.

### Experiments

No experiment blocks D1–D5: every decision above is analytic or
architectural. The only measurements needed — the pinned V/E/F counts
for the test plan — belong to the implementation commit, not to this
ADR, and use the established measure-once-freeze-in-test method.

## Proposed MVP

**Scope**: rewrite `sdf_to_brep_faceted` to the topology-era contract.
The public API in `core/bridge` keeps its shape; the behavior becomes
contract-bearing. MVP target: **sphere SDF → `BRepBody`**.

Pipeline (reusing existing assembly patterns — see design principle):

1. `marching_cubes(body, res)` — unchanged (ADR-0015/0016).
2. **Precondition checks** (the P1–P4 encoding): non-empty mesh; the
   pinned-grid test additionally asserts min |f| > 0 (P2) and
   enclosure (P1). Violations are visible errors — never silent
   repair. Every runtime error is **actionable** (acceptance
   correction 2): it names the likely cause and the remedy. The
   on-grid-zero / degenerate-triangle error tells the caller to
   offset the sampling grid or change the resolution — never a bare
   assertion.
3. **Vertex map**: mesh vertex index → `VertexId`, 1:1, no re-welding
   (mesh indices are already canonical). This is the
   `SharedVertexPool` philosophy degenerating to identity — by
   construction, not by tolerance.
4. **Edge map**: keyed by `(min VertexId, max VertexId)` — the exact
   integer pattern of ADR-0025's `assemble_body`. Coedge `reversed`
   flags are derived from the **edge's intrinsic direction vs the
   loop's traversal direction** (fixing the stub's index-order bug).
5. **Degenerate triangle** (zero area / repeated vertex): contract
   violation under P2 → visible error. The stub's silent `continue`
   is removed.
6. `shell.set_closed(...)` reflects the **actual** invariant, not a
   hardcoded `true`; the assembly ends with `link_radial_edges(body)`
   (ADR-0021 §4).
7. **Hard postcondition, in code and in tests**:
   `loops_are_closed` ∧ every edge exactly 2 co-edges ∧ coedge
   orientation XOR ∧ χ = 2 — i.e. `is_closed_manifold(body)` must
   hold. Failure is a bridge bug, not a "documented limitation".

**Test plan** (additions to `tests/unit/bridge/test_bridge.cpp`; the
existing 12 cases stay unchanged — they pass today and must keep
passing):

- **T1 — sphere, res = 16** (pinned radius, center, resolution):
  exact V, E, F (measured once, frozen), χ = 2, `is_closed_manifold`,
  `loops_are_closed`, every edge exactly 2 co-edges + XOR.
- **T2 — sphere, res = 8 and res = 32**: same invariants, different
  pinned counts — demonstrating that V/E/F are grid-dependent while
  the invariants are not.
- **T3 — off-grid sphere** (generic center, e.g. (0.13, 0.29, 0.07)):
  invariants hold — kills axis-aligned coincidences.
- **T4 — preconditions made machine-checkable**: for the pinned grids,
  min |f| > 0 asserted directly (P2), bounds enclosure (P1).
- **T5 — determinism**: two extractions of the same (SDF, grid) →
  identical V/E/F and bit-equal vertex position sequence.
- **T6 — degenerate/empty inputs**: empty SDF → documented, visible,
  non-crashing behavior.
- **First follow-up (not blocking the MVP)**: the smooth-union case
  gets the same hard contracts once the sphere is green — expected to
  pass, promoted only after measurement.

## Explicitly out of scope

- **B-Rep → SDF** (real implementation; stays stub — own ADR later).
- **Blend/smooth-union hard contracts** in the MVP commit (first
  follow-up case, see above).
- **Stable naming across rebuilds** (Phase F subshape tracing;
  D4 defers it explicitly).
- **Planar patch merging** (D3; future ADR when a consumer exists).
- **On-grid-zero recovery** (detection + error only in the MVP; a
  snap rule or symbolic nudge is a separate decision).
- **`.CAD_0` serialization of B-Rep bodies** (own ADR; D3 notes the
  writer implications).
- **Python bindings changes** for the bridge.
- **`MeshSDF` sign fix** (majority-vote heuristic remains a documented
  stub limitation; winding-number sign belongs to the B-Rep → SDF
  ADR).
- **Primitive-fit rework**: the `describe()` string parsing stays as
  is (working, tested); replacing it with a proper `SDFNode` visitor
  is a later hygiene item, deliberately not part of the MVP.
- **Output sanitization** of any kind (D2).

## Consequences

- This commit adds exactly one file (this ADR) and touches zero code
  and zero tests — the Phase E opening rule.
- Implementation (after acceptance) is localized to
  `core/bridge/src/sdf_to_brep.cpp` (faceted path rewrite) plus
  test additions in `tests/unit/bridge/test_bridge.cpp`; the public
  header keeps its API shape, minus the retired `planar_tolerance`
  option.
- The 4-invariant contract (ADR-0022 §7) becomes the bridge's
  acceptance bar — Phase E is its first consumer outside the boolean
  kernel, which is exactly why that contract was built.
- Risks, accepted: pinned counts at res = 32 add a few hundred ms to
  the suite (≈ 6k triangles — negligible); P2 checking needs the
  grid's field values, which `marching_cubes` already computes
  internally — exposing min |f| (or accepting a debug hook) is a
  small, flagged-for-implementation API surface decision.
  **Resolved at Phase E closure**: exposed as the `min_abs_field`
  FIELD of `SdfToBrepReport` — not a separate function: the app's
  Debug action already renders the report, so a field is the
  smallest surface (no new entry point, no extra binding). It is
  path-independent (computed for the primitive path too) and
  diagnostic-only; the grid scan lives in the bridge
  (`sdf_to_brep.cpp`) and mirrors the `mesh_extract.cpp` layout that
  test T4 independently replicates, so the two copies cross-check
  each other.
- If the cross-compiler bit-exactness claim of D5 ever fails in CI,
  the fallback is already scoped: keep the count-and-invariant
  contract (integer-valued, robust) and drop the T5 bit-equality
  assertion — the ADR's contracts survive, only the strongest
  determinism promise narrows.

## References

- ADR-0002 — dual kernel, native SDF (the vision this phase serves).
- ADR-0005 — persistent naming by genealogy (D4's frame).
- ADR-0014 — floating-point determinism (D5's frame).
- ADR-0015, ADR-0016 — marching cubes and vertex welding (the
  extraction half).
- ADR-0021, ADR-0022 — topology data structure and the 4-invariant
  `is_closed_manifold` contract (the acceptance bar).
- ADR-0023 — analytic seam wires (primitive-fit sphere is a closed
  manifold since this ADR).
- ADR-0025 v5 — IntersectionTable / SharedVertexPool / T-junction
  subdivision (the assembly patterns the bridge reuses).
- Chernyaev, E. V. (1995). *Marching Cubes 33: Construction of
  Topologically Correct Isosurfaces*. CERN.
- Lewiner, T., Lopes, H., Vieira, A. W., & Tavares, G. (2003).
  "Efficient implementation of Marching Cubes' cases with topological
  guarantees." *JGT*, 8(2).
- Lorensen, W., & Cline, H. (1987). "Marching Cubes." *SIGGRAPH*.
- Ju, T., Losasso, F., Schaefer, S., & Warren, J. (2002). "Dual
  Contouring of Hermite Data." *SIGGRAPH* (deferred alternative, D3).
- Jacobson, A., Kavan, L., & Sorkine, O. (2013). "Robust Inside-Outside
  Segmentation using Generalized Winding Numbers." *SIGGRAPH* (for the
  future B-Rep → SDF ADR).
- Andrade, M. V. A. (1998); Chen, X., & Luo, Z. (2010) — via
  ADR-0005 (rejected naming approaches, D4).
