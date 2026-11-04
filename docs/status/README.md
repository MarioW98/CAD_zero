# The status register

The README is the **vetrina** — the showcase: what CAD_0 is, the true
status in three lines, the quickstart, the links to the registers.
This document is the **registro**: the full account of what exists
and what does not, the roadmap with its record, and the cronaca —
the narrative history that lived in the README until Consegna B m2
(2026-11-01) moved it here. The chronicle is precious; it lives in
the register, not in the shop window. Every claim below is one the
CI testifies — no sentence may claim what the CI does not verify.

## What exists — the CI testifies

Every push to `main` runs the full matrix on ubuntu-24.04 — GCC 14
debug (ASan/UBSan, the C++ suite **and** the Python bindings under
the preloaded sanitizer), GCC 14 release, Clang 19 — plus the
Python 3.10 syntax-floor job and the subject lint. What follows is
what those runs verify, day after day:

- **The SDF side** — primitives, CSG booleans (union / subtract /
  intersect), smooth blends, transforms, batched vectorized
  evaluation (xsimd — the batch runs serially; oneTBB is an
  off-by-default opt-in, see the ADR-0009 amendment), MC33
  extraction with vertex welding, and the twist warp under its
  explicit contract ([ADR-0029](../adr/0029-twist-warp-contract.md),
  implemented).
- **The B-Rep side** — radial-edge topology with query operators
  ([ADR-0021](../adr/0021-brep-topology-data-structure.md)/[0022](../adr/0022-brep-topology-query-operators.md)),
  analytic seam wires ([ADR-0023](../adr/0023-analytic-seam-wires-and-boolean-intersect.md)),
  booleans on convex polyhedra with input-contract gates
  ([ADR-0024](../adr/0024-boolean-union-and-subtract.md)):
  invalid inputs refused, invalid results refused, every non-empty
  result validated against exact topological contracts (V/E/F
  counts, closed loops, the amended χ contract) — and, since
  H2-u5 ([ADR-0037](../adr/0037-the-coplanar-class.md), accepted
  and implemented 2026-11-02), the union's ALIGNED coplanar pairs
  are SEWN by the overlay: the composite ε_C gate (two typed
  members, zero new constants — the vocabulary's fourth row,
  [ADR-0035 D2](../adr/0035-h2-numerical-taxonomy.md)) reroutes
  ONLY the admitted pairs; the merged region F_A ∪ F_B is
  projected in the one frame, walked through the loops' crossing
  points (the H1 segment arms), emitted as convex fragments, every
  computed point through the canonical pool. The honest box
  answers where the wall refused (P4 8/12/6, the diagonal stair
  P8 20/32/14, the contained flush P11 8/12/6 — D5's numbers
  pinned BEFORE the code, red demonstrated on the pre-overlay
  tree), the ratifications pin the verified-but-unpinned
  (the parete 20/32/14 twice, the edge-touch and the
  across-contact 16/26/12), the transversal paths run byte for
  byte (the additivity proof), and the door is reachable from
  Python (`CAD_0.brep.bool_union`).
- **The exact kernel — G1 complete** ([ADR-0032](../adr/0032-g1-topology-and-exact-primitives.md),
  implemented 2026-10-26): exact primitives with true trimming
  (p-curves on every co-edge), the exact extrude (the HybridBody
  beside its SDF), the exact subtract (the plane∩cylinder SSI, the
  pinned through-hole topology), exact mass properties by the
  divergence theorem with the `all_exact` honesty flag, the report
  and the guide case end-to-end (the D7.5 table, every row PASS).
- **G2 under way** ([ADR-0033](../adr/0033-g2-exact-booleans-and-first-fillet.md),
  accepted): **unit 1 of 6 landed** — the analytic SSI table: four
  typed entries with their derivations as goldens, the p-curves
  typed in both domains, the degenerates pinned, the table boundary
  DECIDED (option (i): the table is the wall — out-of-table pairs
  refused actionably; option (ii) registered priced with its entry
  criterion), all of it queryable from Python
  (`CAD_0.brep.ssi_query`) before any boolean consumes it.
- **Persistent naming** ([ADR-0036](../adr/0036-persistent-naming-and-shape-history.md),
  implemented): the derived name beside the address — factory roots
  named through the role tables, the extrude and the subtract named
  through the descent table, the resolver's four classes through
  mutation, the D5 quartet with the naive counterproof red-on-address.
- **The SDF → B-Rep bridge** ([ADR-0026](../adr/0026-sdf-brep-bridge.md)):
  exact analytic extraction under hard contracts (pinned V/E/F per
  grid, genus-1 extractions, bit-identical re-runs). The inverse
  direction: exact primitive detection (sphere, box, cylinder) plus
  the mesh bridge — **whose sign is the true generalized winding
  number, evaluated hierarchically** (H2-u4 m1, 2026-11-02: van
  Oosterom–Strackee signed solid angles summed in double, the
  classification's sign band derived and declared in
  [ADR-0035 D2.4](../adr/0035-h2-numerical-taxonomy.md), concave
  and 1e6-translated meshes classified right — the old majority
  vote survives only as the tests' red counterproof); **the
  hierarchy (H2-u4 m2)**: a deterministic AABB tree over the
  triangles (true-median split, tie-break by index — byte-identical
  rebuilds) whose far nodes contribute through their dipole proxy
  under the declared ε_w tolerance ([ADR-0035
  D2.5](../adr/0035-h2-numerical-taxonomy.md): the per-node
  threshold ½/100, the accumulation cap 9/10·½, every
  approximation bounded by the derived ε_node and the bound — not a
  felt distance — deciding the traversal), the classification
  IDENTICAL to the exact evaluation's (the derived composition with
  the sign band), the answered distances exact, the budget-zero
  counterproof proving the machine can still answer without
  approximating — and the wall's abatement a NUMBER: on the
  10k-triangle sphere, 4.5× the exact O(F) scan on the mixed
  corpus (near-surface 4.4×, far-field 43×, deep-inside 1.9× — the
  accumulation cap is the honest limit there). The bridge remains
  **watertight only**: open meshes refused by name at load (O(E)
  once — the free-edge count, the rim triangles, the remedy), so
  the tessellation fallback refuses its own non-watertight output
  rather than answer with a wrong sign (a watertight tessellator
  is the registered follow-up).
- **The desktop app** (PySide6) — registry-driven Create menu,
  feature tree, parametric property panel, undo/redo, viewport
  picking with 3D highlight, STL/OBJ import, STL/OBJ/3MF export,
  **booleans as tree features** ([ADR-0027](../adr/0027-phase-f-boolean-feature-slice.md)),
  and the **parametric sketcher** ([ADR-0030](../adr/0030-sketcher-mvp.md)):
  Create → Sketch, draw typed entities by gesture, Feature →
  Extrude, Edit → Subtract — then edit any parameter and the chain
  regenerates deterministically, one undo entry per modification.
  The Debug menu reports topology, mass and provenance.
- **The native format** `.CAD_0` **v0.4.0** ([ADR-0007](../adr/0007-native-format-versioning.md)/[0007a](../adr/0007a-binary-blob-container.md)):
  recipes as the source of truth, bodies the cache, atomic save,
  transactional load, the migrator chain from v0.1.0, bit-exact
  round-trips, byte-identical goldens.

## What does NOT exist yet — said, not hidden

- **The fillet** — G2 units 2–6 ([ADR-0033](../adr/0033-g2-exact-booleans-and-first-fillet.md)):
  the booleans extended onto the table, the rolling-ball fillet,
  the degenerates' corpus, the UI, the guide case end-to-end. The
  SSI table already registers the fillet's toroidal rows — the
  operation itself does not exist.
- **Patterns** (rectangular / circular feature arrays) — not
  started.
- **STEP** import/export — Phase H ([ADR-0010](../adr/0010-step-scope-mvp.md)),
  planned.
- **G3 free surfaces** — declared as the ladder's third rung
  ([ADR-0031](../adr/0031-exact-brep-kernel.md)), not started.
- **The sketcher's constraint solver** — ~~explicitly deferred with
  its entry criterion~~ **LANDED (G4 m2, 2026-11-04 — ADR-0040
  accepted and implemented: the bill, the rank admission, the
  damped min-norm Newton, the bands ε_solve re-homed from the
  sketch kernel's own constants, the three honest classes
  `sketch_*`, the eight goldens of D5 green; the constraints
  queryable from Python before the UI consumes)**; the WORKBENCH
  followed in the SAME day (G4 m3: the honest counter, the classes
  in words, the pin, the rectangle macro, the edit through the
  solve — the golden 8 live). What does NOT exist yet is the
  constraints' SERIALIZATION (m4 owns it: the constraints as FIELDS
  in the sketch payload, never raw structs — the m2's padding
  discovery paid — and the version bump, arbitrated by the register
  against the fillet line's registered need: never two bumps the
  same day); until m4 lands, the constraints are SESSION state
  (declared: closing the project drops them).
- **The inverse bridge's full direction** (B-Rep → SDF) — the sign
  and the watertight contract are IN (H2-u4 m1, the generalized
  winding number), and the sign's evaluation is HIERARCHICAL since
  H2-u4 m2 (the ε_w-bounded BVH, the classification identical by
  the derived composition, the measured speedup above); what does
  NOT exist yet is the **watertight tessellator** (the fallback's
  registered follow-up).
- **Vertex blends** — unscheduled (ADR-0033's no-promises list).
- **The demo GIF** — the recorder ships (`scripts/record_demo.py`);
  the recording itself has not been made (the TODO below, carried
  here from the README).

## Roadmap — the record

| Phase | Scope | Status |
|-------|-------|--------|
| A | Core math, geometry, tolerance model | ✅ done |
| B | SDF kernel, marching cubes, vertex welding | ✅ done |
| C | Viewport architecture, PySide6 desktop app | ✅ done |
| C+ | Native format + migrators, STL/OBJ import, picking, CI matrix, toolchain contract | ✅ done |
| **D** | **Radial-edge B-Rep topology (ADR-0012), query operators, seam wires, convex booleans (union/subtract/intersect) with slab decomposition, T-junction subdivision — and the aligned coplanar union sewn by the overlay (H2-u5, ADR-0037)** | ✅ done ([ADR-0021](../adr/0021-brep-topology-data-structure.md)–[0025](../adr/0025-proper-greiner-hormann.md), [0037](../adr/0037-the-coplanar-class.md)) |
| E | SDF → B-Rep extraction under hard topology contracts ([ADR-0026](../adr/0026-sdf-brep-bridge.md)); the inverse bridge's sign: the true generalized winding number, watertight-only, evaluated hierarchically over the ε_w-bounded BVH ([ADR-0035 D2.4/D2.5](../adr/0035-h2-numerical-taxonomy.md), H2-u4 m1+m2); the watertight tessellator is the registered follow-up | ✅ done |
| **F** | **Modeling: boolean feature slice in the tree + `.CAD_0` v0.3 operation recipes ([ADR-0027](../adr/0027-phase-f-boolean-feature-slice.md)); sketcher MVP — typed 2D profiles, extrude as a feature, `.CAD_0` v0.4 ([ADR-0030](../adr/0030-sketcher-mvp.md))** | ▶️ in progress |
| **G** | **The exact B-Rep kernel — road (a), all in house ([ADR-0031](../adr/0031-exact-brep-kernel.md), accepted; ADR-0003 amended: the license whitelist): G1 exact primitives + exact mass properties ([ADR-0032](../adr/0032-g1-topology-and-exact-primitives.md), implemented), G2 exact booleans + the first fillet ([ADR-0033](../adr/0033-g2-exact-booleans-and-first-fillet.md) **accepted 2026-10-26: D2 = option (i), the table is the wall — option (ii) registered priced with its entry criterion, decided at the next checkpoint**), G3 free surfaces** | ▶️ in progress — **G1 COMPLETE AND CLICK-TESTED (2026-10-26): ADR-0032 Implemented (the flip is the click-test's own act, the ADR-0030 precedent), the ADR-0031 internal checkpoint recorded (the estimates stand), ADR-0033 accepted with the table-boundary decision taken — D2 = option (i), the wall; the review carried one correction to the record (the cylinder pair's unequal-radii limit missed the typed transverse interior, corrected where it stood); G2 unit 1 of 6 LANDED — the table**: the four typed entries, one derivation each (the classical formulas ARE the goldens: √(R²−d²), the radical plane, ±√(R²−r²), the mid-plane lines at ±√(r²−d²/4)), the p-curves typed in BOTH domains (the framed SphereSurface/CircleCurve, the latitude Line2s, the full-period convention), the re-frame rule with its derived frame budget (one axis family per face — the incompatible family finds the wall, option (i) coherent to the end), the degenerates with pinned dispositions (the tangent point/circle/double line, the empty classes named, the co-radial coincident — the dedup rule's trigger), the wall as returned records (the Viviani quartic, the Steinmetz sinusoids in both domains, the corrected unequal-radii gate, plane×cylinder and plane×plane pointed to their landed owners, the cone registered with no writer, the toroidal rows registered for the fillet's unit, the provenance gate on approximated operands) — all of it QUERYABLE from Python (`CAD_0.brep.ssi_query`, with `translate_body` for positioning) before any boolean consumes it, bit-identical across re-runs (hex-digest pinned); no boolean moved, the format did not move, the display did not change — units 2–6 follow (the booleans extended, the fillet, the degenerates' corpus, the UI, the guide case end-to-end); G1 units 1–6 all landed — unit 1, the data model (p-curves on co-edges, Layer-2 provenance, χ = V−E+F−H); unit 2, the primitives with true trimming + the face query in the bindings; unit 3, the exact extrude (the HybridBody on the node); unit 4, the exact subtract (the plane∩cylinder SSI, the pinned through-hole topology, the honest refusals); unit 5, the exact mass properties (the divergence theorem over the exact faces, all_exact the honesty flag, the classical formulas as the tests); unit 6, the report and the guide case end-to-end — the Debug report grew mass + provenance beside χ (with the H term in the label) and genus through `CAD_0.report` (the single consumer of the mass API and the body query, no second opinion), the property panel shows the numbers READ-ONLY (QLabels that cannot emit parameter_changed — the UI-bug lesson made structural, the exact/approximated flag visible, a faceted body's polyhedral numbers never disguised as exact), and the guide-case example prints the D7.5 table side by side (12/17/7/2, χ=0, genus 1, abc − πr²h, I_zz = abc(a²+b²)/12 − πr⁴h/2 — every row PASS) with the radius edit bit-identical through ONE undo entry, running in the CI as a suite member; the v0.4 golden pinned with the table's numbers, the v0.3 golden honestly faceted; **the sketch line (LINEA C, G4) opened 2026-11-04: ADR-0040 "the sketch: the root system, the constraint solver, the bands it declares" ACCEPTED with the register surgery in the same act** (the sketch ratified as the root system ADR-0036 governs — the roots `sk#F:<i>` the chain positions, the resolver's classes the downstream edit already built; ADR-0035's D2 table gaining the SOLVE row with ε_solve re-homed from the sketch kernel's own kJointCoincidenceTol/kParallelAngleTol — one value per world, two consumers, zero drift; ADR-0030's deferral REDEEMED, the entry criterion met; the `sketch_*` family noted in the error taxonomy beside `fillet_*` and `reference_*`) — m2 IL SOLVER (the bill, the rank admission, the damped min-norm Newton, the three honest classes, the eight goldens of D5) lands as the line's first code morso; the parallel discipline holds: not one file of the fillet's line touched, the format's MINOR bump left to the register's arbitration; **m2 LANDED 2026-11-04: the solver green on both compilers — the eight goldens each landing by itself (the triangle 4 iterations machine-exact, the tangent 19 with the case bit pinned, the rectangle 5 with the bill returning 16-16=0, the naive counterproof red-on-frozen-geometry, the over-constrained refused with the spectrum's tail, the 4/5/10 stalled at 35 with the diagnosis naming its own inequality, the degenerate refused in the kernel's own words on the stored bytes, the downstream edit's quartet class-1 silent), the determinism 10/10 byte-identical, the near-band margin measured (the angular 8.39x the ADR rounds to 8.4), the bindings queryable from Python**; **m3 LA WORKBENCH LANDED 2026-11-04: the honest DoF counter on the canvas's status line (the bill's own text, recomputed at the cold sites only — the write routes' own economy), the classes in words through the panel's existing rows (the register consumer pattern, zero new panel code — solved with the free count, sketch_not_solvable with the residual and the named culprit and the diagnosis, sketch_over_constrained with the culprit at the admission, the messages VERBATIM), the pinning gesture (x_i = v, the DIMENSION family's fourth form, the value consumed at gesture time) on the selection-driven Constraints menu with the kernel's own field names, the rectangle macro (press-drag-release: 4 entities + 9 constraint records consuming 13 — 4 coincidences + 2 parallels + 1 perpendicular + 2 lengths, D1's own table — in ONE undo entry, the bill reading 16-13=3 right after the gesture and 16-16=0 after three pins), and the edit with the downstream regeneration (the re-value through the solve, the recipe's fields stored back, the DAG pass re-materializing the dependents — the golden 8 live: the names re-derived identically, ONE undo entry, N rebuilds bit-identical); the suggestions stay value-offers (pinned by test: a gesture near one NEVER writes a constraint); the constraints are SESSION state until m4 (declared: the format bump is m4's own, the register arbitrates the number against the fillet line — never two bumps the same day); not one fillet file touched; the UI smoke suite extended (six new tests, all through the wiring)** |
| H | STEP AP203 MVP (ADR-0010) | planned |

*(The compact version of this table lives in the README — the
vetrina carries the phases, the register carries the record.)*

## Cronaca

The narrative below is the status-muro as it stood in the README at
commit `876b818` (Consegna B m1 landing, 2026-11-01), transferred
here verbatim by Consegna B m2 so the README could become the
vetrina. It is history — it speaks of its moment, and it stays
exactly as it was written.

### The status paragraph, as it stood

> **Status.** A dual-representation CAD kernel, no third-party CAD kernels involved. The **SDF side**: primitives, CSG booleans, smooth blends, MC33 extraction with vertex welding, and the **twist warp under an explicit contract** ([ADR-0029](../adr/0029-twist-warp-contract.md), implemented: cylinder-AABB bounds, the σ(k, r) Lipschitz closed form, the full chain-rule gradient, construction-time validation on every node, and a property closure with twisted bridge extractions). The **B-Rep side**: radial-edge topology with query operators, analytic seam wires, and **boolean operations on convex polyhedra** (`bool_union`/`bool_subtract`/`bool_intersect`) — every non-empty result validated against exact topological contracts (V/E/F counts, closed loops, and the amended χ contract: `is_closed_orientable` for the structural invariants, χ and genus **reported** — derived only for single closed-orientable shells — with `is_closed_manifold` kept as the exact χ=2 compatibility wrapper). The **SDF → B-Rep bridge** ([ADR-0026](../adr/0026-sdf-brep-bridge.md)) extracts native SDF bodies into B-Rep solids — exact analytic primitives (sphere, box, cylinder) or MC33 faceted — under the same hard contracts, pinned V/E/F per grid, genus-1 extractions included (the pierced box: χ=0, genus 1, bit-identical re-runs); the **inverse direction (B-Rep → SDF) is an approximate, documented stub** (primitive detection + mesh SDF), deferred to a future ADR. The PySide6 desktop app adds parametric shape creation, undo/redo, viewport picking with 3D highlight, STL/OBJ import, STL/OBJ/3MF export — and **boolean operations as tree features** ([ADR-0027](../adr/0027-phase-f-boolean-feature-slice.md)): Edit → Subtract builds an operation node whose operands stay referenced by FeatureId (auto-hidden, editable, world-anchored placement), undo/redo is id-stable, and a deleted operand leaves a visible dangling state — never a silent rebind — all persisted by the versioned native format (`.CAD_0` v0.4.0: recipes are the source of truth, bodies the cache; chained migrators from v0.1.0) with atomic save. The **sketcher MVP** ([ADR-0030](../adr/0030-sketcher-mvp.md), implemented) adds parametric 2D profiles: **Create → Sketch** opens a canvas where mouse gestures draw typed entities (segments, circles, arcs — sketch toolbar, shortcuts 1–4), **Feature → Extrude** turns the closed profile into a solid prism, and **Edit → Subtract** cuts it from a body — then edit any entity parameter and the whole chain (sketch → extrude → subtract) regenerates from the recipe, deterministically, one undo entry per modification. The Debug menu reports topology, mass and provenance for any selected body (the report grew with G1: χ with its inner-loop term, genus, the mass properties with the exact/approximated flag, and the per-kind provenance tallies — through `CAD_0.report`, the single consumer of the mass API and the body query). **Roadmap: Phase F (parametric modeling) in progress — the boolean feature slice and the sketcher MVP are in. The great fork is decided ([ADR-0031](../adr/0031-exact-brep-kernel.md) accepted: the exact B-Rep kernel will be built in house, forever; the license whitelist is pinned in ADR-0003), and its first rung is complete ([ADR-0032](../adr/0032-g1-topology-and-exact-primitives.md), implemented 2026-10-26: G1 units 1–6 landed and click-tested — **G1 complete and closed: the D7.5 table read in the report, the volume verified by hand, the radius edited live** — unit 1, the data model: the sketch kernel's 2D types reused as the p-curve vocabulary on co-edges, Layer-2 tolerance + provenance on every vertex/edge/face with the writers' honesty, and the extended χ = V−E+F−H with its backward compatibility proven on every golden; unit 2, the primitives with true trimming: box/sphere/cylinder/tetrahedron faces are now regions of domains — every co-edge carries its typed p-curve (the wall's rectangle walk in (θ,z), the caps' π-sweep arc pairs, the sphere's periodic seam), the cylinder's parametrization corrected to the surfaces' one right-handed rule, and the face query (family, loops, p-curves, provenance) exposed through the bindings as plain values; unit 3, the exact extrude: the sketch-born geometry becomes exact — the profile chain maps to the prism's true faces per the ADR's table, the caps' outer trimming loop is the validated chain itself (entity for entity, hole rims as Circle2 trim curves), and the extrude node materializes the exact B-Rep BESIDE its SDF as the HybridBody ADR-0002 always meant (bridge tolerance 0, the same solid twice — the SDF still feeds display, the exact B-Rep reads through the bindings' query, and out-of-class profiles keep their SDF with the refusal recorded); unit 4, the exact subtract: the exact chain crosses its first curved-surface intersection — the plane∩cylinder SSI entry in the three orientations (⊥ the circle with its Line2 on the cylinder domain, ∥ the line pair with the tangent and empty cases, oblique refused naming the angle), the pinned through-hole topology (pierced faces gaining inner loops split at the seam pair, the surviving wall the patch between the circles with its seam's two co-edges in the one loop, the guide case 12V/17E/7F/H=2 χ=0 genus 1 — same topology as the faceted report, better geometry — and V+2n/E+3n/F+n for every tool profile), the degenerates with their pinned dispositions (coplanar absorbed, tangential/crossing/blind refused with the remedies named, the miss the exact copy), and the subtract node owning both materializations — the v0.4 golden materializes the exact subtract on load while the v0.3 golden stays honestly faceted (its tool is the Y-axis primitive cylinder, not a Z-axis prism — the refusal recorded); unit 5, the exact mass properties: the B-Rep’s first quantitative interrogation — volume, centroid and the inertia tensor by the divergence theorem over the exact faces (V = (1/3)∮x·n and the moment kernels, the 1/5 ansatz constant derived), the planar families by Green’s theorem in the face’s own domain (the winding sign plus the positional outer-minus-inner — one geometric rule for every authored walk), the cylindrical patches by the (θ,z) rectangle integrals, the full sphere by its closed form; all_exact is the honesty flag (a faceted body answers the polyhedral value, never silently mixed), out-of-class faces refuse actionably, and the classical formulas ARE the tests — the guide case’s hand-checkable table abc − πr²h and I_zz = abc(a²+b²)/12 − πr⁴h/2, readable from Python through CAD_0.brep.mass_properties; unit 6, the report and the guide case end-to-end: the Debug report grows mass + provenance beside χ (label honest: V−E+F−H) and genus, the property panel shows the numbers read-only (Volume, Centroid, Inertia, the exact/approximated flag — QLabel rows structurally incapable of emitting parameter_changed, the UI-bug lesson made structural; a faceted body says approximated with its polyhedral numbers, never an exact in disguise), the body-level query `CAD_0.brep.body_query` answers counts/χ/genus/provenance as plain values, and the guide-case example prints the D7.5 table side by side — analytic vs reported, 12/17/7/2, χ=0, genus 1, abc − πr²h, I_zz = abc(a²+b²)/12 − πr⁴h/2, all exact — every row PASS or the difference named, the radius edit bit-identical through ONE undo entry, the example running in the CI as a suite member; the v0.4 golden pinned with the table’s numbers, the v0.3 golden honestly faceted.** The G1 fork follow-up is **accepted and G2 is under way** ([ADR-0033](../adr/0033-g2-exact-booleans-and-first-fillet.md), accepted 2026-10-26: the analytic SSI table, the exact booleans, the first fillet — **the reserved table-boundary decision is taken: option (i), the table is the wall** (out-of-table pairs refused actionably, the ceiling explicit and queryable; option (ii) — declared-tolerance subdivision — registered as a priced candidate with its entry criterion: a real case demanded by an author body, decided at the next internal checkpoint with table and fillet data on the table); one correction carried on the record — the drafted unequal-radii limit of the cylinder pair missed the typed transverse interior, corrected where it stood; **unit 1 of 6 LANDED in the companion commit: the table** — the four typed entries (plane×sphere's circle √(R²−d²) at the foot of the perpendicular, sphere×sphere's radical-plane circle, cylinder×sphere coaxial's two circles at ±√(R²−r²), cylinder×cylinder parallel equal-radii's mid-plane line pair), each a derivation whose golden IS the classical formula, the p-curves typed in BOTH domains, the re-frame rule landed (the framed SphereSurface beside the framed CircleCurve, the seam re-wired in the ADR-0023 pattern, the frame budget derived from the face's own record — one axis family per face, the incompatible family finding the wall), the degenerates pinned (tangencies, the empty classes, the co-radial coincident), and the whole table QUERYABLE from Python through `CAD_0.brep.ssi_query` BEFORE any boolean consumes it (curves, p-curves in both domains, provenance exact/0, the refusals as returned records naming the pair, the class and the remedies — the Viviani quartic, the Steinmetz sinusoids, the corrected unequal-radii gate, the landed pairs pointed to their owners, the cone's no-writer registration, the fillet's toroidal rows) — with `CAD_0.brep.translate_body` exposing the exact side's own placement so the author bodies can be positioned for the query; the ADR-0031 internal checkpoint is recorded: the estimates stand). The CI matrix (GCC 14 debug + release, Clang 19, on ubuntu-24.04 — the same series as the local rite, the presets pin the compilers) judges every push on `main`; the badge above is the live record.

### The Phase G row, as it stood

| **G** | **The exact B-Rep kernel — road (a), all in house ([ADR-0031](../adr/0031-exact-brep-kernel.md), accepted; ADR-0003 amended: the license whitelist): G1 exact primitives + exact mass properties ([ADR-0032](../adr/0032-g1-topology-and-exact-primitives.md), implemented), G2 exact booleans + the first fillet ([ADR-0033](../adr/0033-g2-exact-booleans-and-first-fillet.md) **accepted 2026-10-26: D2 = option (i), the table is the wall — option (ii) registered priced with its entry criterion, decided at the next checkpoint**), G3 free surfaces** | ▶️ in progress — **G1 COMPLETE AND CLICK-TESTED (2026-10-26): ADR-0032 Implemented (the flip is the click-test's own act, the ADR-0030 precedent), the ADR-0031 internal checkpoint recorded (the estimates stand), ADR-0033 accepted with the table-boundary decision taken — D2 = option (i), the wall; the review carried one correction to the record (the cylinder pair's unequal-radii limit missed the typed transverse interior, corrected where it stood); G2 unit 1 of 6 LANDED — the table**: the four typed entries, one derivation each (the classical formulas ARE the goldens: √(R²−d²), the radical plane, ±√(R²−r²), the mid-plane lines at ±√(r²−d²/4)), the p-curves typed in BOTH domains (the framed SphereSurface/CircleCurve, the latitude Line2s, the full-period convention), the re-frame rule with its derived frame budget (one axis family per face — the incompatible family finds the wall, option (i) coherent to the end), the degenerates with pinned dispositions (the tangent point/circle/double line, the empty classes named, the co-radial coincident — the dedup rule's trigger), the wall as returned records (the Viviani quartic, the Steinmetz sinusoids in both domains, the corrected unequal-radii gate, plane×cylinder and plane×plane pointed to their landed owners, the cone registered with no writer, the toroidal rows registered for the fillet's unit, the provenance gate on approximated operands) — all of it QUERYABLE from Python (`CAD_0.brep.ssi_query`, with `translate_body` for positioning) before any boolean consumes it, bit-identical across re-runs (hex-digest pinned); no boolean moved, the format did not move, the display did not change — units 2–6 follow (the booleans extended, the fillet, the degenerates' corpus, the UI, the guide case end-to-end); G1 units 1–6 all landed — unit 1, the data model (p-curves on co-edges, Layer-2 provenance, χ = V−E+F−H); unit 2, the primitives with true trimming + the face query in the bindings; unit 3, the exact extrude (the HybridBody on the node); unit 4, the exact subtract (the plane∩cylinder SSI, the pinned through-hole topology, the honest refusals); unit 5, the exact mass properties (the divergence theorem over the exact faces, all_exact the honesty flag, the classical formulas as the tests); unit 6, the report and the guide case end-to-end — the Debug report grew mass + provenance beside χ (with the H term in the label) and genus through `CAD_0.report` (the single consumer of the mass API and the body query, no second opinion), the property panel shows the numbers READ-ONLY (QLabels that cannot emit parameter_changed — the UI-bug lesson made structural, the exact/approximated flag visible, a faceted body's polyhedral numbers never disguised as exact), and the guide-case example prints the D7.5 table side by side (12/17/7/2, χ=0, genus 1, abc − πr²h, I_zz = abc(a²+b²)/12 − πr⁴h/2 — every row PASS) with the radius edit bit-identical through ONE undo entry, running in the CI as a suite member; the v0.4 golden pinned with the table's numbers, the v0.3 golden honestly faceted |

### The demo-GIF TODO, as it stood

<!-- TODO(badge GIF): record a 15–25s noVNC demo (create box → sketch →
     draw a circle → extrude → subtract → double-click the circle and edit
     the radius live, geometry regenerating) and embed it here:
     <p align="center"><img src="docs/media/demo.gif" width="640" alt="CAD_0 desktop app demo"></p> -->
