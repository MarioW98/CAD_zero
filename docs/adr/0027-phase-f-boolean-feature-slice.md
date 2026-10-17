# ADR-0027: Phase F — boolean feature slice (CSG subtract in the feature tree)

## Status

**Accepted (2026-10-18).** Accepted with three corrections from
review, applied to this document before any code (the review's own
rule):

1. **D6 / genus derivation guard (blocking).** The report derives
   genus = (2 − χ)/2 ONLY when the body is closed-orientable AND
   χ ≤ 2 AND χ is even; anything else reports "χ = N — multi-shell,
   genus n/d" and derives nothing. The rule binds `SdfToBrepReport`,
   the Python binding dict and the Debug report alike (the evenness
   clause is structural: components of closed orientable surfaces
   have even χ, so an odd χ under a passing structural check is an
   assembly defect — deriving a half-integer genus would be an
   assertion, not a report). The components counter stays deferred.
   One editorial alignment rides along: the Test plan's pierced-box
   binding line said "closed yes" — a draft leftover that conflicted
   with D6's own "Closed orientable: yes" report design and with the
   wrapper's χ = 2 semantics; it now reads "closed-orientable yes"
   (the legacy `closed` field is False on the pierced box — the
   wrapper telling the truth about its own contract).
2. **D2 / tool-offset anchoring.** The placement offset is anchored
   in **world coordinates** (sentence added to D2, with the reason
   and the consequence stated).
3. **Test plan / undo interleaving.** The undo-interleaving case
   (delete operand → dangling → undo of the subtract via the LIFO
   stack → redo: same ids, no rebind) is part of the DAG contract
   tests.

Implementation proceeds one commit per coherent unit (Proposed MVP),
CI green after each. This ADR also discharges the ADR-0007 obligation
("each MINOR or MAJOR format change requires a new ADR") for the
`0.2.0 → 0.3.0` bump decided in D4.

## Context

Phase E is closed: the SDF → B-Rep bridge is Implemented (ADR-0026),
with its app consumer (the Debug-menu topology report) and the
smooth-union follow-up. The app is SDF-primary: parametric SDF bodies,
ray-march-class kernel, MC-tessellated display, `.CAD_0` persistence of
SDF trees. Phase F is parametric modeling: a feature tree whose nodes
are *operations with references*, with rebuild, undo/redo, and a
serialized recipe. ADR-0005 said it plainly: "Phase F (modeling
parametrico) is where the real stress tests land."

**Verified current state (2026-10-16, tree at 4cf3a05)** — where
composite CSG already works today, layer by layer:

| Layer | Composite CSG today | Evidence (verified) |
|---|---|---|
| Kernel SDF | **Yes** — full vocabulary | `UnionSDF` (n-ary), `IntersectionSDF`, `SubtractionSDF`, `SmoothMinSDF`; free functions `sdf_union` / `sdf_intersect` / `sdf_subtract`; `TranslateSDF` / `RotateSDF` / `ScaleSDF` / `TwistSDF` |
| Python bindings | **Yes** | `CAD_0.sdf.subtract` / `.translate` / `.union`; `examples/sdf_hole.py` (pierced ball), `examples/sdf_lattice_ball.py` |
| `.CAD_0` v0.2.0 | **Yes** — the whole op vocabulary round-trips | `serialize_node` / `deserialize_node` cover sphere, box, cylinder, torus, cone, capsule, plane, union (n-ary `children`), intersect, subtract, smooth_min, translate, rotate, scale, twist; unknown op → explicit error |
| Scene (C++) | **Body only** | A `SceneNode::body` can hold any tree, but the scene is a **flat list**: no operation nodes, no inter-node references, no rebuild ordering, no genealogy |
| Builder registry | **No** | 4 primitives (sphere, box, cylinder, torus) — the only parametric editing surface (`builders.hpp`) |
| App UI | **No** | Create menu = registry; property panel = builder params only; flat `QTreeWidget`, single selection; no CSG authoring action of any kind |
| Viewport display | **Yes, unchanged path** | MC tessellation (res 48) evaluates *any* body → OpenGL mesh buffers; `node.transform` is serialized but neither exposed nor applied — positioning lives in SDF-tree ops (`translate`), as the examples do |
| Bridge | **No for genus > 0 — blocking** | `sdf_to_brep_faceted` enforces `is_closed_manifold` (ADR-0022 §7 invariant 4: χ = 2) as a hard postcondition → `std::runtime_error`; a pierced box (χ = 0) **cannot be extracted today** |

Three findings that shape the decisions, reported honestly:

1. **The app viewport does not ray-march.** The display path is
   per-node MC tessellation + OpenGL mesh buffers (`viewport_widget.py`).
   The ray-marcher (`core/viewport/ray_march.hpp` + `glsl/sdf_raymarch.frag`)
   is a tested kernel capability with a GLSL twin — not wired into the
   app display. The D5 conclusion ("no new rendering work") survives
   *a fortiori*: the display path is body-agnostic.

2. **Undo/redo is not id-stable today.** Create/delete commands
   re-allocate `SceneNode` ids on redo (documented in
   `_push_create_command`: "may differ between do/redo cycles because
   Scene assigns new IDs"). Any FeatureId-referencing design must fix
   this first (D2). The mechanism already exists: `scene::restore_node`
   (the load path) preserves ids and bumps `next_id`.

3. **The χ contract blocks the MVP shape.** The "hole with a cylinder"
   (box − through-cylinder) is genus 1, χ = 0 — the first genus > 0
   body ever to face the topology contracts. Every extraction asserted
   to date is genus 0: bridge tests T1–T12, the boolean-kernel tests,
   and the goldens (the B-Rep factories are seam-wired since ADR-0023
   and pass — there is no B-Rep torus factory; see the D6 inspection).
   The genus > 0 examples (`sdf_hole.py`, `sphere_to_stl.cpp`) export or
   display meshes and never call the bridge.
   `is_closed_manifold` rejects it (invariant 4), the bridge throws,
   and the Debug report shows an error dialog on the exact shape this
   phase exists to demonstrate. The contract must be amended **before**
   the code (D6).

## Decision

### D1. Scope of the first increment: "hole with a cylinder" before the sketcher

**Choice.** The first Phase F increment is a vertical slice: **CSG
subtract as a feature** — an operation node referencing two operand
nodes, positioned tool, undoable, serializable, extractable — the
pierced box end-to-end. The sketcher (2D profiles, constraints,
extrude) is the **second** increment.

Arguments:

1. **The consumers already exist (verified in the table above).**
   Kernel, bindings, format, and display all carry composite bodies
   today. The slice's new surface is exactly the architecture that
   matters — naming, DAG rebuild, format evolution, χ contract — not
   geometry plumbing.
2. **Immediate demo value.** A parametric hole is the canonical CAD
   moment after primitives: every operand edit (cylinder radius, tool
   offset) re-materializes the result and the extraction report follows.
3. **Architectural risk concentrated where it belongs.** The slice
   forces ADR-0005's genealogy, the dangling-reference rules, the
   format bump, and the χ amendment — every decision the sketcher would
   *also* need, but whose cost stays hidden until later. Sketcher-first
   would postpone exactly these behind a constraint solver.
4. **The cheapest possible probe of the severest contract.** D6 is
   exposed by one box, one cylinder, one subtract.

**Rejected alternatives:**

- *Sketcher-first (the classic order)* — postpones naming, format, and
  χ behind the solver; the feature tree would be retrofitted later;
  no existing consumer for profiles.
- *B-Rep booleans in the app* — the app is SDF-primary (ADR-0002,
  ADR-0026 D1); B-Rep authoring has no tree, no UI, no format; the
  convex-only B-Rep boolean kernel (ADR-0024/0025) is narrower than
  SDF CSG.
- *Library-only (no app action)* — `examples/sdf_hole.py` already is
  this; it exercises none of the naming/format/UI risk the phase
  exists to burn.
- *Composite body without genealogy* (stuff the subtract tree into one
  node, no references) — the status quo trap: un-editable, un-named,
  un-undoable; ADR-0005 never comes into play.

**Size (the answer the Context table determines).** Kernel: zero.
Bindings: the report `genus` field plus a thin scene surface. Display:
zero. The work concentrates in scene/document (DAG protocol + id-stable
commands), format v0.3, one UI action + multi-selection, and the χ
amendment.

### D2. Representation in the feature tree: operation node, operands by FeatureId

**Choice.**

- `SceneNode` gains an optional **operation recipe**:
  `{ type: Subtract (MVP; Union/Intersect reserved, not exposed),
     target: FeatureId, tool: FeatureId,
     placement: {offset_x, offset_y, offset_z} }`.
  The node's payload stays an SDF body — `kind = Sdf` — the honest
  ADR-0002 dispatch: the node's *representation* is an SDF; the
  operation is its *provenance*, exactly as `builder` + `params` are
  the provenance of a primitive. The materialized body is
  `sdf_subtract(target.body, translate(tool.body, offset))`.
- **Offset anchoring (review correction 2).** The placement offset
  is expressed in **world coordinates** and applied as
  `sdf_translate(tool.body, offset)`: the tool's body is whatever its
  (possibly re-edited) builder params currently produce, and the
  offset then positions that body in the world. World anchoring is
  the honest choice for this MVP for three reasons: it is the one
  positioning mechanism that works end-to-end today (kernel
  `translate` op → serialized body → display, the examples' path);
  every registered builder emits a primitive centered on its own
  origin, so "tool body frame" and "world frame" coincide at
  creation and only diverge if we ever add placed builders; and an
  offset anchored in the tool body's *param-dependent* frame would
  silently shift the hole whenever the tool's params are re-edited —
  the exact class of surprise this ADR forbids. Consequence, stated
  plainly: editing the cylinder's radius does not move the hole;
  editing the offset does. (`node.transform` remains
  serialized-but-inert — Context finding — and is not part of this
  contract.)
- **FeatureId = `SceneNode::id`** for this MVP: document-unique,
  monotonic, persisted via `next_id`, preserved by `restore_node`.
  ADR-0005's upper layers (`SubshapeRef`, `Generation`) are *not*
  exercised yet (D3).
- **Rebuild semantics.** The document layer evaluates a DAG: an operand
  param edit → operand factory rebuild → dependents re-materialize in
  topological order → tessellation-dirty cascade. In-app creation order
  guarantees operands-before-operation; the loader validates the DAG
  two-pass (all nodes, then references): missing operand → error
  *state* (the document still opens — see below); cycle or
  self-reference → explicit load error (structurally corrupt).
- **Dangling references — visible, never silent.** Deleting an operand
  *succeeds*; the operation enters a visible error state: it keeps its
  last materialized body (display continues), the tree row shows the
  error (name suffix + "!" in the Type column), the property panel
  shows an actionable message ("operand Cylinder#3 was deleted — undo,
  or delete this operation"), and any re-materialization fails loudly.
  This is ADR-0005's generation rule applied at feature granularity:
  a dangling reference is *reported*, never silently re-picked.
- **Undo/redo.** Subtract is a command: execute = create the operation
  node + hide the operands; undo = remove it + unhide; redo = restore
  **with the same id**. Prerequisite (the verified gap, finding 2):
  create/delete commands must become id-stable across undo/redo
  cycles, via restore-style insertion with pinned ids. Without it,
  undoing an operand's creation would spuriously dangle every
  dependent reference.
- **Violation clause.** Naming by geometric signature — matching
  operands or subshapes by position, size, or shape — is **forbidden**
  (ADR-0005's documented "wrong turn"). References are FeatureIds,
  nothing else.

**Rejected alternatives:**

- *Nested ownership* (the operation owns operand subtrees) — kills
  independent operand editing, makes deletion destructive, and removes
  the genealogy the phase is for.
- *Signature matching for rebinds* — banned by ADR-0005.
- *Auto-cascade deletion of dependents* — destructive and surprising;
  the visible dangling state is the honest CAD behavior.
- *Rebuild-on-load only (no in-memory DAG)* — param edits would not
  propagate; that propagation is the slice's core value.

### D3. The naming promise (the most severe decision)

**Choice.** The MVP promises exactly three things:

1. **Determinism of the rebuild.** The same feature tree (same ids,
   same params, same creation order) produces **bit-identical**
   materialized bodies and bit-identical extraction reports. This
   chains the existing determinism stack: ADR-0014
   (`-ffp-contract=off`), deterministic SDF evaluation, MC33 + integer
   welding, ADR-0026 D5 bridge determinism. The subtract path's
   arithmetic is max/min over the operand fields — inside the
   supported class.
2. **Genealogy.** Every node has a stable FeatureId; an operation's
   operands are queryable by id; deletion surfaces as a visible
   dangling error.
3. **Never a silent rebind.** No automatic re-targeting of references,
   ever, under any circumstance.

The MVP does **not** promise:

- **Subshape stability across rebuilds.** Editing the cylinder radius
  changes the extraction's V/E/F and shifts every index-based
  `SubshapeRef`. Exactly like ADR-0026 D4: re-extract after an edit is
  new topology; old references dangle visibly. Deferred to Phase F-2
  (subshape tracing through extraction — the legitimate research item
  ADR-0026 D4 already deferred). No MVP consumer holds subshape
  references (the Debug report is transient text), so the deferral
  costs nothing now.
- **The `Generation` counter of `ShapeId`.** Nothing holds
  cross-generation references yet; it lands with subshape stability in
  F-2.

**Rejected alternatives:** stable ids derived from field values, or
seed/cell matching across rebuilds — the ADR-0026 D4 rejections carry
over verbatim: both are geometric-signature naming in disguise, and
approximate matching is the bug generator ADR-0005 exists to prevent.

### D4. Format `.CAD_0` v0.3

**Choice.** MINOR bump `0.2.0 → 0.3.0`. This section is the ADR-0007
companion for the bump (no separate 0007b).

An operation node serializes as a normal SDF node **plus** the recipe:

```json
{ "id": 7, "name": "Subtract", "visible": true, "kind": "sdf",
  "body":   { "op": "subtract", "a": { "...": "target tree" },
              "b": { "op": "translate", "child": { "...": "tool tree" },
                     "offset": [0, 0, 0] } },
  "operation": { "type": "subtract", "target": 2, "tool": 3,
                 "params": { "offset_x": 0.0, "offset_y": 0.0,
                             "offset_z": 0.0 } } }
```

- **The recipe is the source of truth; the body is a cache.** The
  materialized `body` makes files self-contained, carries the
  last-good geometry for the dangling state, and doubles as a
  diagnostic. It is *derived data*: coherent by construction in memory
  (the DAG keeps it current), never authoritative.
- **Loader, two-pass.** Pass 1: parse all nodes (bodies from file).
  Pass 2: validate operation metadata — operands exist → rebuild the
  body from the recipe (deterministic; catches stale caches); operand
  missing → keep the file body and enter the error state; cycle or
  self-reference → explicit load error. Load stays transactional
  (strong exception guarantee, unchanged).
- **Writer.** Always writes the current version and the current
  materialized bodies. Atomic save, insertion-ordered, deterministic —
  golden-file byte-stability, unchanged.
- **Migrator.** `0.2.0 → 0.3.0` registered as a content no-op (v0.2
  documents carry no operation fields) — the same precedent as the
  `0.1.0 → 0.2.0` entry. The chain must contain the step so old files
  upgrade.
- **Honest note — ADR-0007 letter vs practice.** ADR-0007's MINOR
  bullet reads "old readers open new files with feature flags set to
  defaults"; the implemented rule (`native.cpp`, since ADR-0007a) is
  stricter: a build facing a different MAJOR.MINOR without a
  registered migrator fails with an explicit error ("cannot be read by
  this build"). This ADR ratifies the implemented rule — explicit
  failure beats defaulted flags under this repo's "surface, never
  drop" principle — and records that the ADR-0007 bullet is to be read
  as the migrator-chain semantics. (Unknown *fields* on the load path
  do degrade gracefully — the unknown-builder-tag precedent — but the
  version check rejects before field parsing.)

**Round-trip tests:** byte-identical save → load → save for a v0.3
pierced-box document; the v0.2 golden migrates and opens with an
identical extraction report; the v0.1 and v0.2 goldens stay
byte-immutable (Test plan).

**Rejected alternatives:**

- *PATCH-only (0.2.1, riding unknown-field tolerance)* — new semantics
  smuggled through field tolerance: a v0.2 reader would round-trip a
  v0.2.1 file **dropping the genealogy silently** (data loss through
  downgrade). Violates "never silence".
- *Recipe-only (no cached body)* — files not self-contained; the
  dangling state unserializable.
- *Body-only (status quo)* — no recipe: no editing, no genealogy, and
  the slice cannot exist.

### D5. UI/UX

**Choice.**

- **Entry point.** Tree multi-selection (switch the feature tree to
  `ExtendedSelection` — the single-selection mode is the only
  selection change needed) of exactly two SDF-kind nodes →
  **Edit → "Subtract (boolean difference)"** opens a minimal dialog:
  target and tool (two combo boxes pre-filled from the selection;
  default target = the lower FeatureId — deterministic, no geometric
  heuristics) plus the tool placement (offset x/y/z, defaults 0).
  OK creates the operation.
- **Where the node appears.** A new top-level row: name "Subtract",
  Type column "CSG"; the label names its operands ("Subtract (Box#2 −
  Cylinder#3)"). The operands stay in the tree — selectable, editable
  as today — but are hidden from the viewport (`visible = false`,
  already a serialized field). The tree **remains flat** for the MVP
  (it is a flat `QTreeWidget` today); hierarchical display is F-2
  polish. The flat list + DAG is the honest state: the tree shows
  creation order, the DAG is the truth.
- **Viewport — zero renderer work.** The operation node is an ordinary
  Sdf node: `update_meshes()` tessellates its composite body through
  the same MC path at res 48, and the pierced box appears. (Premise
  correction, Context finding 1: the display path is MC tessellation
  + OpenGL, not ray-marching; the conclusion holds *a fortiori*. The
  kernel ray-marcher stays available for a future display mode; wiring
  it is explicitly not part of this slice.)
- **Property panel.** Selecting the operation row shows its placement
  params (offset x/y/z — the panel is generic over param dicts, zero
  new widget code) and a read-only operand summary (names + ids).
  Editing an offset re-materializes and refreshes the viewport.
  Selecting an operand row keeps today's param-editing flow untouched.
- **Debug report.** Same flow, amended content (D6): χ, genus, and
  `is_closed_orientable`.

**Rejected alternatives:**

- *Viewport-click operand picking (rubber band)* — picking is
  single-body today; tree selection is the honest MVP surface.
- *Placement on the primitive builders* — two positioning mechanisms
  (`node.transform` and SDF `translate`) would drift apart; the SDF
  tree op is the one mechanism that works end-to-end today. The
  serialized-but-inert `node.transform` stays as is, documented.
- *Hierarchical tree model (QAbstractItemModel rewrite)* — a Phase
  C.7 leftover exists (`feature_tree.py`, dataclass-based, unwired);
  rewiring it is not needed for the slice's value.

### D6. The χ contract and the hole (non-negotiable)

**Fact.** Box − through-cylinder = genus 1 = χ 0. `is_closed_manifold`
(ADR-0022 §7, extended in ADR-0025) asserts invariant 4 (χ = 2) →
returns false → the faceted bridge **throws** (hard postcondition) →
the Debug report errors out on the MVP shape. This is the first
genus > 0 body ever to face the topology contracts (verified: bridge
tests T1–T12, the boolean-kernel tests, and the goldens all assert
genus-0 bodies; the factory sphere/torus fail for the seam-wire
reason — see the inspection below; the genus > 0 examples
export/display meshes and never call the bridge).

**Choice — amend before code, in three coordinated moves:**

1. **Split the contract.** "Closed orientable manifold" = the
   *structural* invariants, the ones that detect assembly defects:
   (0) non-empty + `shell.is_closed()`; (1) every edge exactly 2
   co-edges; (2) coedge orientation XOR; (3) loops closed. "Topological
   classification" = a property of the *input*, not of assembly
   correctness: χ and genus. New predicate
   **`is_closed_orientable(body)`** = invariants 0–3.
   `euler_characteristic(body)` already exists and is **reported**.
   Genus is **derived, never guessed** (review correction 1, the
   blocking rule): `genus = (2 − χ)/2` is reported only when
   `is_closed_orientable(body)` holds AND χ ≤ 2 AND χ is even; any
   other χ reports "χ = N — multi-shell, genus n/d" and derives
   nothing. The clauses: χ ≤ 2 keeps the derivation on the
   single-shell reading (χ = 2 − 2g); χ > 2 means multiple shells
   (χ = 2s − 2g) and the components counter is deferred (out of
   scope), so a negative "genus" can never be displayed; evenness is
   structural — every connected component of a closed orientable
   surface has even χ, hence so does their sum, and an odd χ under a
   passing structural check is an assembly defect the structural
   invariants cannot see — deriving a half-integer genus would be an
   assertion, not a report. This rule binds the report field, the
   binding dict and the Debug report identically.
2. **ADR-0022 §7 amendment** (annotated in place at implementation
   time, pointing here). Invariant 4 was an over-promise of spherical
   topology — correct for every body the repository had produced until
   now, wrong the moment a boolean feature exists. The §7 analysis
   (a duplicated interface ring is χ-*invariant* — only the XOR
   invariant catches it) is **unaffected** and becomes what it always
   actually was: the structural invariants are the assembly-defect
   detectors; χ is the input's topology. §7's rejection entry
   "non-connected shells (χ ≠ 2)" is re-read: multi-shell detection
   moves to the classification layer (χ even and ≠ 2 → not
   sphere-like; a components counter is deferred, out of scope).
3. **Compatibility wrapper — zero silent weakening.**
   `is_closed_manifold(body)` remains defined as
   `is_closed_orientable(body) && euler_characteristic(body) == 2`.
   Every existing call site — the bridge postcondition today, T1–T12,
   the boolean-kernel tests, the factory regression guards — keeps its
   exact current meaning: nothing that passed starts failing, nothing
   that failed starts passing. New code uses `is_closed_orientable`
   plus explicit χ assertions. The bridge's faceted postcondition
   becomes `is_closed_orientable` (hard) with χ **reported**; the
   ADR-0026 D2 sentence "χ = 2, `is_closed_manifold`, and
   `loops_are_closed` do not [depend on the grid]" is amended in place
   to: "χ is grid-independent (a topological invariant of the extracted
   surface, given P1–P4 and correct topology resolution); its *value*
   is 2 − 2g of the input — genus-0 inputs yield χ = 2."

**Inspection of the χ = 2 assumption across the repo** (verified, as
this ADR mandates):

- `tests/unit/bridge/test_bridge.cpp` T1–T12: all genus-0; every
  `is_closed_manifold` / `chi == 2` assertion remains true under the
  wrapper. **No existing test changes.** New pinned genus-1 tests are
  required (Test plan).
- Boolean kernel (ADR-0024/0025 tests): box booleans, corner notch
  χ = 2 — genus 0, unaffected.
- B-Rep factories (verified at implementation time, 2026-10-18):
  `make_box`, `make_tetrahedron`, and the seam-wired `make_sphere` /
  `make_cylinder` (ADR-0023) all pass `is_closed_manifold` — and
  therefore `is_closed_orientable` — with χ = 2; the wrapper
  equivalence holds on each. There is no `make_torus` B-Rep factory;
  the pre-ADR-0023 seam-wire rejection (V = E = 0, caught by
  invariant 0) is exercised by a hand-built periodic-fixture in the
  tests instead. (This line corrects the draft's inspection, which
  predated the ADR-0023 seam-wire fix — facts over drafts.)
- Debug report (app): currently "Closed (is_closed_manifold): yes/NO"
  — after the amendment it shows **χ, genus and the structural
  contract, not just closed yes/no**: "Closed orientable
  (is_closed_orientable): yes", "χ = V − E + F = 0", "Genus = 1" —
  and, for a multi-shell body, "Genus = n/d" (χ shown, genus not
  derived — the correction-1 rule). χ is already a report field;
  `genus` is the new field in `SdfToBrepReport` and the binding dict
  (None when not derived), alongside `orientable`.
- MC33 capability: the asymptotic decider, integer welding, and radial
  assembly are topology-agnostic; genus > 0 needs no new machinery —
  it needs P3 respected (hole diameter well above cell size; the MVP
  geometry at res 48 has orders of magnitude of margin). The risk is
  P3 discipline, not topology: pinned grids and asserted min |f|
  margins in the tests.

**Connectivity caveat (stated, not solved).** χ alone cannot
distinguish one genus-1 shell from a multi-component body whose χ sums
to 0 (e.g. a genus-2 shell against a genus-0 pair). The MVP reports χ
and derives genus only under the correction-1 rule
(`is_closed_orientable` ∧ χ ≤ 2 ∧ χ even); a components/shell counter
is deferred (out of scope). P4 (sign fidelity) and the input class
keep the MVP honest in the meantime.

**Rejected alternatives:**

- *Weakening invariant 4 inside `is_closed_manifold`* (χ → "any even
  value") — silently re-meaing every existing call site: the quiet
  contract change this repository does not do.
- *Asserting χ == 0 in the bridge for the hole* — the bridge must stay
  input-agnostic; genus is the input's property (a sphere extracts to
  χ = 2, a pierced box to χ = 0 — both must pass).
- *A parity/tolerance hack on χ* — χ is the integer identity V − E + F;
  there is nothing to tolerate.

## Proposed MVP — the hole, end-to-end

1. **Contract amendment (D6):** `is_closed_orientable` + the
   compatibility wrapper + `genus` in `SdfToBrepReport`; ADR-0022 §7
   and ADR-0026 D2 amendment annotations. (brep + bridge)
2. **Document layer (D2):** the operation recipe on `SceneNode`; the
   DAG rebuild protocol (materialization as a pure function of
   (recipe, operand bodies), C++-side, tested without PySide);
   id-stable create/delete/subtract commands; the dangling error
   state. (scene + app)
3. **Format (D4):** v0.3 writer/reader (two-pass), the 0.2 → 0.3
   migrator, a v0.3 golden pierced-box document. (io)
4. **App (D5):** `ExtendedSelection`; Edit → Subtract dialog; the
   operation row; operand auto-hide; property-panel placement params;
   Debug report with χ + genus. (app)
5. **Bindings:** the report `genus` field; the minimal scene surface
   the dialog and rebuild need.

Zero kernel changes. Zero renderer changes. The v0.1/v0.2 goldens are
immutable. One commit per coherent unit at implementation time, CI
green after each.

## Test plan (contracts, not coverage)

C++:

- `is_closed_orientable`: box, tetrahedron and the seam-wired
  sphere/cylinder factories true; a hand-built periodic-style body
  (faces but no edges — the pre-ADR-0023 seam-wire shape, invariant 0)
  false; two faces glued back-to-back false (XOR); duplicated
  interface ring false.
- Wrapper equivalence: for every existing genus-0 fixture,
  `is_closed_manifold == (is_closed_orientable && χ == 2)` — in the
  topology suite (factory boxes), the boolean suite (box booleans,
  corner notch) and the bridge suite (every pinned extraction of
  T1–T9).
- `genus_from_chi` (the correction-1 rule, pure integer cases):
  2 → 0, 0 → 1, −2 → 2; 4, 6, 3 → not derived.
- **Bridge genus-1 (the MVP's pinned heart):** pierced box (pinned
  geometry, pinned grid) → exact V/E/F frozen by measurement (the
  house method: measure once, pin, never Approx), χ == 0, genus == 1
  derived, `is_closed_orientable`, loops closed, the compatibility
  wrapper **false** (χ ≠ 2 — the zero-silent-weakening check),
  min |f| > 0, deterministic re-run bit-identical. A second grid:
  counts differ, χ does not — the T2 habit extended to genus 1.
- **Multi-shell rule (correction 1):** two disjoint spheres → χ == 4,
  `is_closed_orientable`, genus NOT derived ("n/d").
- Format: v0.3 round-trip byte-identical; v0.2 golden → migrator →
  identical extraction report; a dangling-operand file loads with the
  error state; a cyclic file fails with an explicit error.
- DAG: operand edit → dependent re-materialized bit-identical to a
  fresh construction; operand delete → visible dangling; undo of the
  delete (same id) → recovered. **Undo interleaving (review
  correction 3):** with the command stack […, Subtract, Delete
  operand], the LIFO order exercises the interleave — after the
  Delete the operation dangles; undo (the Delete, same restored id)
  recovers it; undo of the Subtract removes the operation; redo of
  the Subtract restores it **with the same node id and the same
  operand references** (no rebind, ever); redo of the Delete
  re-dangles it. Ids are compared at every step — any id drift is
  exactly the failure the test exists to catch.

Python / bindings:

- The report dict gains `genus` (int when derived, None when not) and
  `orientable`; pierced-box report χ = 0, genus = 1,
  closed-orientable yes (the legacy `closed` field — the χ == 2
  wrapper — is False here: the hole is exactly the case that field
  was wrong about); two disjoint spheres → χ = 4, genus None.
- UI smoke (offscreen, monkeypatched boxes — the established driver
  pattern): create box + cylinder → subtract → the tree row exists,
  operands hidden, undo removes it, redo restores the **same ids**;
  the Debug action on the operation node shows χ = 0 and genus 1.
- The Python 3.10 f-string checker is clean on every touched file.

## Explicitly out of scope

- **The sketcher** — 2D profiles, constraints, extrude: the second
  increment.
- **Union/intersect in the UI** — kernel and format already carry
  them; the UI action is a follow-up once subtract proves the pattern.
- **B-Rep booleans in the app**, any B-Rep authoring, and the reverse
  bridge (B-Rep → SDF — its own ADR).
- **Subshape stability / persistent naming of extracted entities** —
  Phase F-2; ADR-0026 D4's deferral is reconfirmed here.
- **Hierarchical feature-tree display** (the QAbstractItemModel
  rewrite).
- **Tool rotation** — offset-only placement; rotation follows the T8b
  grid-independence habit when it lands.
- **Smooth subtraction** — `SmoothMinSDF` is union blending; smooth
  subtract is separate research.
- **Multi-component/shell counting** (the D6 connectivity caveat).
- **Param-edit undo** — pre-existing gap (`set_param` is not on the
  command stack today); unchanged by this slice, noted for F-2.
- **`Generation` counter + `SubshapeRef`** — F-2, with subshape
  stability.

## Consequences

**Positive:**

- The three deferred-but-fatal architectural risks — naming semantics,
  format evolution, χ contract — are burned by the cheapest possible
  probe, before the sketcher multiplies surfaces.
- The bridge's contract becomes input-topology-honest: genus is
  reported, not forbidden.
- The undo/redo id instability — a latent defect for *any* future
  reference-holding feature — is fixed as a side effect.
- The migrator chain gets its second real entry (0.2 → 0.3), proving
  the ADR-0007 machinery.

**Negative / risks:**

- `is_closed_manifold` becomes a compatibility wrapper: two names for
  related contracts is a tax. Mitigation: a deprecation note pointing
  at `is_closed_orientable` + explicit χ assertions.
- The flat tree + hidden operands can confuse ("where did my cylinder
  go?"). Mitigation: operands stay in the tree; the operation row
  names them.
- The χ = 0 connectivity ambiguity is documented, not solved (the
  components counter is deferred).
- Files grow (the cached body duplicates operand geometry inside the
  subtract tree): bounded by tree size, acceptable at MVP scale; the
  recipe remains the source of truth.

**Neutral:**

- ADR-0007's MINOR bullet is re-read to match implemented practice
  (recorded in D4).
- ADR-0022 §7 and ADR-0026 D2 receive in-place amendment annotations
  at implementation time — the series' established correction pattern.

## Experiments

None block this ADR — every decision above rests on verified
repository facts (the Context table). The implementation phase must
**measure and freeze** (the house method — measure once, pin exactly):

- The pierced-box extraction counts at the pinned grids (res 32/48)
  and their min |f| margins.
- The v0.3 golden bytes (after the writer is deterministic by
  construction).

If implementation finds MC33 producing a non-manifold vertex at the
pinned geometry (a P3 violation — not expected at these margins), the
pinned geometry or resolution changes; the invariants never do.

## References

- ADR-0005 — persistent naming by genealogy (the parent decision).
- ADR-0002 — dual kernel, honest dispatch.
- ADR-0007 / ADR-0007a — format versioning and the blob container
  (D4 discharges the bump-ADR obligation).
- ADR-0013 — license compatibility (this slice adds no dependency).
- ADR-0022 §7 — the 4-invariant contract (amended here).
- ADR-0026 — the SDF → B-Rep bridge (D4's naming promise mirrored;
  D2's grid-independence sentence amended).
- Andrade (1998); Chen & Luo (2010) — the signature-matching wrong
  turn, as cited by ADR-0005.
