# ADR-0030: The sketcher MVP — parametric 2D profiles, extrude as a feature

## Status

**Draft (2026-10-20).** Docs-only commit, in attesa di review — the
ADR-first pattern of ADR-0026/0027: this document argues the decisions
BEFORE any code exists (the Phase F-2 opening move). Zero code, zero
tests in this commit; implementation proceeds one commit per coherent
unit after acceptance, CI green after each.

One premise correction is folded in at its site (D3, facts over
drafts): the draft brief said "profiles composed by min"; the closed
profile is the INTERSECTION of the entity regions, and the operator
that composes it is max. The min is the union — it appears here only
as a rejected alternative and as the future hierarchical composition
(out of scope, with its entry criterion stated).

**Revision 1 (2026-10-20, same day, pre-implementation): D5 rewritten.**
The draft's D5 was a UI/UX sketch — entities entered through
property-panel dialogs, dragging deferred, snapping deferred to the
solver era. The review direction replaces it with a BINDING
interaction specification: direct manipulation is the outer face, the
recipe is the real object — every graphical gesture is a write of
parameters into the sketch node's recipe (D5.1–D5.8). D6's UI smoke
now asserts the gesture semantics. Two one-sentence amendments ride
along at their sites (D2's chain-closure timing, D4's loader error
split) — declared in D5's preamble, not smuggled.

## Context

ADR-0027 D1 named this increment: *"The sketcher (2D profiles,
constraints, extrude) is the second increment."* The subtract slice
was chosen first precisely to burn the architectural risks the
sketcher would multiply — naming (ADR-0005 genealogy at feature
granularity), the DAG rebuild protocol, format evolution (the v0.3
bump and its migrator), and the χ contract amendment. Those are now
delivered and green in production: the pierced box extracts at genus 1
with bit-identical re-runs, operand edits propagate, dangling
references are visible, and the whole thing round-trips through
`.CAD_0` v0.3. The sketcher inherits an architecture that has already
been stress-tested once — this ADR spends that inheritance instead of
re-deriving it.

**Verified current state (2026-10-19, tree at 3ea5866)** — what exists
today, layer by layer, and what the sketcher MVP lacks:

| Layer | Today | Evidence (verified) |
|---|---|---|
| Kernel SDF 2D | **Nothing** | `field.hpp` vocabulary: primitives (sphere/box/cylinder/torus/cone/capsule/plane), operators (union/intersect/subtract/smin), transforms (translate/rotate/scale/twist) — all 3D; no 2D node, no profile, no extrusion of any kind |
| Feature tree | **Subtract only** | `OperationType::Subtract` (MVP; Union/Intersect reserved); `OperationRecipe{target, tool, offset}`; `rebuild_operations` Kahn pass with visible dangling and explicit cycle errors — the loop iterates the hardcoded roles `{r.target, r.tool}` (`operations.hpp`) |
| Node kinds | **Sdf / Mesh** | `NodeKind` honest dispatch (ADR-0002): a node is ONE thing; no kind carries 2D data |
| Format | **v0.3.0** | `serialize_node`/`deserialize_node` cover the 3D vocabulary + the subtract recipe; migrator chain 0.1.0 → 0.2.0 → 0.3.0 registered and exercised by goldens |
| Builder registry | **4 primitives** | `builders.hpp`: tag + param table + `build` — the single source of truth the property panel consumes; nothing sketches |
| App UI | **No canvas** | Flat `QTreeWidget`, generic property panel over param dicts, MC-tessellated GL viewport; a Phase C.7 `feature_tree.py` (QAbstractItemModel) exists unwired; no 2D surface of any kind |
| Bridge | **Genus-ready** | ADR-0026 P1–P4 + the ADR-0027 D6 amendment: χ reported, genus derived under the closed-orientable/χ≤2/even rule; a genus-1 extraction (pierced box) is pinned in the tests — an extruded ring will face the same contracts and pass |
| The gate | **ADR-0029** | "New operators arrive validated": every new operator lands with invariants→bounds, Jacobian→Lipschitz, chain-rule→gradient, an entry in the validation table, and construction-time validation of its supported class |

Three findings that shape the decisions, reported honestly:

1. **The 2D field does not exist, and it is the densest decision in
   this ADR.** An extruded profile is not a new primitive with a known
   exact formula — it is a COMPOSITE whose value, gradient, Lipschitz
   and bounds must be derived from the entity fields they are built
   from, and whose compositional "lies" (where the field is NOT a
   distance) must be named before the code can tell them silently.
   ADR-0029's gate applies in full: the sketcher's operators arrive
   with their derivations or they do not arrive (D3).
2. **The DAG protocol is role-hardcoded.** `rebuild_operations_core`
   iterates `{r.target, r.tool}`. An extrude has ONE operand (the
   sketch) under a new role; a sketch node is a third `NodeKind` with
   no body, no mesh, and no factory. The protocol generalizes; the
   code does not yet (D2).
3. **The parametric value is already demonstrable end-to-end.** Edit
   propagation, id-stable undo, byte-stable save/load and the genus-1
   extraction all exist. What is missing is exactly the user-facing
   moment: *draw a circle, extrude it, subtract it from a box — the
   parametric hole* — the canonical CAD use case after primitives and
   booleans, combining F-1 and F-2 (D5's CLI deliverable).

## Decision

### D1. Scope MVP: parametric 2D profiles + extrude as a feature; the constraint solver is explicitly OUT

**Choice.** The MVP is: profiles made of typed 2D entities —
**segments, circles, arcs** — where every entity carries its OWN
numeric parameters (segment: two endpoints; circle: center, radius;
arc: center, radius, start/end angles), assembled into a closed
contour on the world XY plane; plus **extrude as a feature**: an
operation node that turns a sketch into a solid prism and enters the
feature tree exactly like the subtract.

The parametricity of this MVP is **propagation, not search**: editing
the circle's radius re-materializes the extrude, which re-materializes
the subtract that consumes it — the whole chain, deterministically,
with the same FeatureIds. The constraint solver (coincidence,
parallelism, tangency between entities) is **explicitly out and
deferred to its own ADR**. The thesis, argued:

1. **First the value, then the research.** A parametric profile that
   extrudes and subtracts is shippable, testable geometry with a
   canonical use case. A constraint solver is a research program: DOF
   counting, convergence, under/over-constrained diagnostics — months
   of work during which nothing else can land on top, because the
   data model itself (what a sketch IS) is what the solver would
   dictate.
2. **Solver-first is the sketcher-first mistake a second time.**
   ADR-0027 D1 rejected sketcher-first because it would postpone
   naming, format and the χ contract behind a solver. Sketcher-with-
   solver-first repeats that exact shape at a smaller scale: it
   postpones the extrusion semantics, the 2D operator derivations and
   the format bump behind the solver, while the profile-without-
   solver already exercises every one of them.
3. **The solver needs something to solve.** Constraints bind
   entity-parameters to entity-parameters. Those parameters exist
   only after this MVP: today there is no entity in the document
   model to constrain. Building the solver first means building it
   against a speculative data model; building it after means building
   it against the real one, with real propagation tests to regress
   against.
4. **Typed entities, not a dumb polyline.** Every entity carries its
   parameters as numbers (the circle's radius is a float, editable in
   the property panel, serialized in the format) — this is what makes
   the value real and what a future solver will bind. A point-cloud
   polyline has no radius to edit and no tangency to declare.

**Rejected alternatives:**

- *Solver-first (the classic sketcher architecture)* — argued above;
  also the documented wrong order for this repo twice over
  (ADR-0027 D1's rejection, restated at sketch granularity).
- *Profiles as free point lists (no typed entities)* — kills entity
  parametricity, the MVP's entire value; also no honest way to declare
  the exact 2D fields D3 needs per entity.
- *Extrude only from code (bindings/script, no sketch node)* —
  `examples/sdf_hole.py` already proves composites-from-code; without
  the sketch node there is no genealogy, no format surface, no
  editing: the F-1 lesson applied to F-2.
- *Revolve/loft/sweep in the same MVP* — each is a different 2D→3D
  semantics with its own derivations; they arrive as amendments with
  their own table rows when a consumer exists (out of scope below).

**Size.** Kernel: the 2D operators + extrusion (the bulk — D3's
derivations and validation). Scene/document: a third NodeKind, one
new operation type, role generalization in the rebuild pass. Format:
v0.4 (one migrator entry). UI: one new canvas widget + one dialog.
The architecture (DAG, undo, panel, bridge) is inherited, not
re-designed.

### D2. Representation: sketch as a tree node, extrude as an operation node — the subtract's genealogy verbatim

**Choice.**

- **`NodeKind::Sketch`** — a third kind, the honest ADR-0002 dispatch:
  a sketch node is ONE thing, 2D data on the world XY plane. It has
  NO body (nothing 3D to evaluate), NO cached mesh (`update_meshes()`
  skips it by kind, exactly as it skips Mesh nodes), NO factory. Its
  payload is the entity list + the contour: an ordered chain of typed
  entities, each carrying its parameters, connected endpoint-to-
  endpoint; a profile may carry exactly one outer chain plus hole
  chains (the ring of D6) — every chain closed **at materialization:
  closure is the extrude's precondition and the visible definition
  state's target, not the sketch node's right to exist (an open chain
  is a legal, visibly-flagged editing state — revision 1, declared in
  D5's preamble)**.
- **World XY, and only world XY, for the MVP.** The plane choice is
  not cosmetic: D3 declares Lipschitz = 1 *because* the profile field
  is evaluated in the same frame it is authored in, with no
  transformation in the composite. Arbitrary work planes would put a
  rigid transform between authoring and evaluation — a different
  derivation (a bounded one, but a real one: L = L₂D·‖J‖, gradient
  rotated) — and are deferred with their entry criterion (out of
  scope).
- **Extrude = an operation node**, `kind = Sdf` with an
  `OperationRecipe`:
  `{ type: Extrude, sketch: FeatureId, distance, direction ±Z }` —
  the same provenance pattern as the subtract's `{target, tool,
  offset}`: the node's representation is an SDF body (the prism), the
  operation is its provenance. `materialize_operation` extends with a
  dispatch on the recipe type; the rebuild pass's operand iteration
  generalizes from the hardcoded `{target, tool}` pair to the recipe's
  **role list** (`{sketch}` for extrude) — the Kahn pass, the
  dangling detection, the cycle errors, and the loader's pass 2 are
  otherwise untouched.
- **Genealogy: identical to the subtract (ADR-0027 D2 verbatim).**
  FeatureId = `SceneNode::id`; the extrude's operand is queryable by
  id; deleting the sketch **succeeds** and the extrude enters the
  visible dangling state — last materialized body kept (display
  continues), the tree row flags it, the property panel shows the
  actionable message, nothing ever re-picks. Undo/redo stays
  id-stable (the F-1 fix carries over); the extrude command is
  create-op + keep-sketch-visible (a sketch has no viewport body to
  hide — the subtract's operand auto-hide does not apply and is
  deliberately not imitated).
- **Editing surface.** Entity parameters are the sketch node's
  editing surface: selecting an entity (canvas click, D5) routes its
  parameters to the generic property panel, exactly as a primitive's
  `params` route today — zero new panel code, the registry contract's
  consumer pattern. A param edit marks the sketch changed, the DAG
  pass re-materializes dependents in topological order, the mesh-dirty
  cascade follows.
- **Violation clause (unchanged, restated because this is where it
  would be violated first):** naming by geometric signature — matching
  entities, vertices or subshapes by position, radius or shape — is
  **forbidden** (ADR-0005). Sketch entities are addressed by
  (FeatureId of the sketch, entity index in the chain) — an index in
  the document's own data, never a geometric match.

**Rejected alternatives:**

- *Sketch owned by the extrude node (nested payload)* — kills
  independent editing, makes one sketch feed multiple features
  impossible, deletes the genealogy the phase exists to exercise;
  the F-1 rejection of nested ownership carries over verbatim.
- *Sketch as an Sdf node with a fake body* — a node that is one thing
  pretending to be another; violates the honest dispatch twice (no
  body to evaluate, no mesh to cache).
- *A separate sketch document outside the tree* — outside the DAG,
  outside the format, outside undo; the value dies in a side file.
- *Hierarchical tree display now* (parent sketch under extrude rows,
  the QAbstractItemModel rewrite; the C.7 leftover exists) — display
  polish; the flat tree + DAG remains the honest MVP state (ADR-0027
  D5's position, reconfirmed).

### D3. The SDF semantics of the extrude (the technically densest decision)

Every operator below arrives per the ADR-0029 gate: invariants →
bounds, Jacobian → Lipschitz, chain rule → gradient, a row in the
validation table, construction-time validation of the supported
class. The lies these operators would tell by default are named
FIRST and closed in the text — the discipline 0029 applied on
arrival rather than in repair.

**The entity fields (2D, exact, one per entity type).** Each entity
carries the exact signed field of its **oriented region** — the
2D solid whose boundary the entity's contour side delimits:

- **Segment → half-plane.** `f(P) = s·((P − p₀)·n̂)` with n̂ the
  segment's unit normal and s = ±1 chosen by the contour's interior
  side. Exact everywhere; ∇f = s·n̂ constant; L = 1.
- **Circle → disc (or complement).** `f(P) = |P − C| − r` for an
  outer contour (interior inside the circle), `f(P) = r − |P − C|`
  for a hole (interior outside — the ring of D6 needs exactly this).
  Exact everywhere; ∇f = ±(P − C)/|P − C|; L = 1.
- **Arc → the curved half-plane: disc or complement, by orientation.**
  The region an arc contributes is the disc it bounds (convex fillet)
  or its complement (concave notch) — NOT the angularly-clamped
  distance-to-the-arc-curve. **Lie closed #1:** "sdArc (exact
  arc-distance, the well-known formula with the angle clamp) is the
  arc's field" — false for the profile. The clamped sdArc measures
  distance to the *curve*; the profile field needs the *region*, and
  the region is bounded by the whole circle, with the neighbouring
  entities' regions cutting it to the arc's angular span. The exact
  clamped formula still exists in the kernel — as the honest picking/
  snapping measurer of D5, never as the profile's field.

**Composition: `f_profile = max_i f_i` — the INTERSECTION of the
entity regions.** This is the premise correction stated in Status:
the draft said *min*. Four half-planes under min are the whole plane;
under max they are the quadrilateral. The sign and the zero-set come
out exactly right under max, and only under max:

- **Sign: exact.** f < 0 iff P is inside every region iff P is inside
  the profile (given the contour-closure condition below).
- **Zero-set: exact.** The profile's boundary is where the max
  crosses zero — each entity's active arc of boundary.
- **Value: NOT a distance field in general — this is lie #2, the one
  the draft flagged, now derived.** For ANY field with L = 1 that
  vanishes on the boundary S: |f(P)| ≤ d(P, S) (a 1-Lipschitz field
  cannot over-reach its own zero-set). The max-composition is exactly
  such a field, and the inequality is where it lives:
  - *Outside, past a convex corner* (interior angle θ): on the corner's
    external bisector at perpendicular distance t from both lines,
    f = t while the true distance is t/sin(θ/2) — **the gap is
    1/sin(θ/2)** (√2 at right angles, unbounded as θ → 0, gone as
    θ → π). The field UNDERSTATES the distance — conservative.
  - *Inside, near removed boundary* (a concave notch eats part of a
    segment's line): the half-plane's field still measures to the
    theoretical infinite line, the true boundary is farther — again
    |f| ≤ d, understated depth.
  - *Where the winning entity's boundary arc is fully active* (the
    common case — mid-segment, mid-arc): f IS the exact distance.
  **What is true, stated once:** the composed field is a 1-Lipschitz
  conservative lower bound of the distance, exact on the active
  boundary, sign- and zero-set-exact everywhere. It is a marching and
  evaluation field, not a measuring stick; measurement (snap, pick,
  dimension display) uses the per-entity exact formulas (D5).
- **Lipschitz: L = 1, proved, not hoped.** max (and min) of
  1-Lipschitz fields is 1-Lipschitz:
  |max(u,v)(p) − max(u,v)(q)| ≤ max(|u(p)−u(q)|, |v(p)−v(q)|) ≤ |p−q|.
  Unlike the twist episode (ADR-0029 D2: the declared 1 + 2|k| was
  violated at k·r ≳ 0.2), this declaration is closed and its proof is
  two lines. No entity, no composition, no slab below carries L > 1 —
  the extruded prism inherits L = 1 and the ray-marcher and the
  extractor consume it unchanged.
- **Gradient: chain rule to the winning branch, the kernel's
  convention.** Where one entity strictly dominates, ∇f_profile is
  that entity's 2D gradient; at ties (the contour's joints) the field
  is non-differentiable and the returned gradient is the winning
  branch's, normalized — **the same max-seam semantics already
  declared for `IntersectionSDF`/`SubtractionSDF`/`BoxSDF`** (lie #3
  closed: "the max's gradient is the true gradient" — a.e. yes, at
  seams it is a documented choice, and the choice never points
  through the corner because the winning branch is the nearest
  boundary).

**The extrusion: `f_solid(x, y, z) = max(f_profile(x, y), slab(z))`,
`slab(z) = |z − d/2| − d/2` for direction +Z (mirror for −Z; d =
distance from the sketch plane).** The intersection of the
infinite profile prism with the extrusion slab. Same class as above,
now in 3D:

- Sign and zero-set exact; **inside exact** (near a prism edge, the
  max of two negatives is the nearer face's true depth); **outside,
  past the horizontal edges** (lateral face meeting the cap plane at
  90°) the field understates by up to √2 — the same 1/sin(θ/2)
  derivation with θ = π/2 (**lie #4** closed: "max(profile, slab) is
  the prism's SDF" — it is the prism's *field*: conservative,
  sign-true, not the metric distance near the edges. The exact prism
  SDF exists in the literature (corner-aware max) and is NOT used:
  it costs branches per sample for a benefit only measurement would
  see, and measurement uses the entity formulas).
- Gradient: (∇f_2D, 0) where the profile branch wins, (0, 0, ±1)
  where the slab wins, winning-branch + normalization at the seam —
  the declared convention.
- **Bounds (the `field.hpp` contract: the box of the region where the
  field can be negative).** The profile's negative region is a subset
  of every entity's region, so `bounds_2D` = the INTERSECTION of the
  entity bounds — the `IntersectionSDF` rule at 2D (half-planes
  contribute unbounded boxes on their interior side; a closed contour
  of them intersects to a bounded box; **an unbounded result is a
  construction-time error: "profile does not close"**). The extrusion
  intersects with the slab's z-range: `bounds = bounds_2D × [z₀, z₁]`.
  Honesty note: the intersection of BOUNDING BOXES over-estimates the
  bounding box of the intersection (conservative, contract-true, a
  wider narrow band than strictly needed — the same over-estimate
  `IntersectionSDF::bounds()` already makes today).
- **The failure regime, closed at construction (lie #5 — the one that
  breaks the SIGN, not the metric).** The max-composition equals the
  profile **iff the profile is really the intersection of the entity
  regions**. The case that violates it: a concave entity (hole circle
  or concave arc) whose FULL circle swallows legitimate parts of the
  profile — e.g. a notch deeper than the profile is wide. Then the
  max classifies those parts outside: not an understated distance, a
  WRONG SIGN. Closure, constructive: **the contour is
  self-certifying** — every joint vertex and every entity's interior
  sample point (midpoint of the segment/arc, circle center excluded)
  must evaluate to the expected side of the composed field (joints:
  |f| ≤ joint tolerance; midpoints of boundary entities: |f| ≤ arc's
  sagitta-scaled tolerance... stated exactly: a boundary entity's
  midpoint must be ON the zero-set, i.e. |f| within the curvature
  tolerance of the entity's own discretization). A contour that lies
  about being the boundary fails loudly at construction with the
  entity and point named — `std::invalid_argument`, the ADR-0029 D5
  family, joined by the loader's error path for free. Plus the
  structural checks: endpoints of consecutive entities coincide (the
  chain closes), every chain is closed, entity parameters finite,
  radii > 0, arc sweep in (0, 2π), distance > 0, direction ∈ {+1, −1}.

**The validation table (the D5-of-0029 equivalent — new rows):**

| Node | Field | Exact regime | Understated regime (gap) | Lipschitz | Gradient | Bounds | Construction checks |
|---|---|---|---|---|---|---|---|
| `Halfplane2D` (segment) | s·((P−p₀)·n̂) | everywhere | — | 1 | s·n̂ | unbounded half-box | endpoints finite, p₀≠p₁ |
| `Disc2D` (circle/arc) | ±(|P−C|−r) | everywhere | — | 1 | ±(P−C)/\|P−C\| | [C±r]² | C finite, r > 0 |
| `ProfileSDF` | max of entities | active boundary, interior near it | convex corners 1/sin(θ/2); notch shadows | 1 (proof above) | winning branch, normalized | intersection of entity bounds | chain closure; joints+midpoints self-certify; bounded result |
| `SlabZ` | \|z−d/2\|−d/2 | everywhere | — | 1 | (0,0,±1) | z-range | d > 0 finite |
| `ExtrudeSDF` | max(profile, slab) | interior; active faces | horizontal edges ≤ √2 | 1 | winning branch, normalized | bounds₂D × z-range | via children |

### D4. Format `.CAD_0` v0.4

**Choice.** MINOR bump `0.3.0 → 0.4.0`; this section is the
ADR-0007 companion for the bump, the ADR-0027 D4 method applied
again.

- A sketch node serializes with `kind: "sketch"`, its entity list
  (each entity: type + explicit parameters + orientation) and its
  chain structure. An extrude node serializes as an Sdf node plus
  the recipe: `{ "operation": { "type": "extrude", "sketch": 12,
  "params": { "distance": 2.0, "direction": 1 } } }` — the recipe is
  the source of truth, the materialized prism body is the cache
  (derived data, self-contained files, the dangling state's
  last-good geometry — every D4-of-0027 clause carries over).
- **Migrator `0.3.0 → 0.4.0`: a deliberate content no-op** (v0.3
  documents carry no sketch fields) — registered so the chain
  `0.1.0 → 0.2.0 → 0.3.0 → 0.4.0` stays explicit and every old file
  upgrades. The implemented stricter-than-letter rule is ratified
  again: a build facing a different MAJOR.MINOR without a registered
  migrator fails explicitly.
- **Loader, two passes, unchanged protocol.** Pass 1 parses all
  nodes (sketch entities validated per the D3 construction checks —
  parameter-class corruption, the NaN radius or r ≤ 0 kind, is a loud
  load error, the ADR-0029 family; an OPEN chain loads as revision
  1's legal editing state, its extrude pending in the visible error —
  the loader splits the two, as D5's preamble declares). Pass 2
  validates references: a missing sketch operand → the extrude keeps
  its file body and enters the visible dangling state (not a load
  error); cycle or self-reference → explicit load error.
- **Goldens.** A new v0.4 golden (the parametric-hole document of
  D5's use case); the v0.1/v0.2/v0.3 goldens stay byte-immutable;
  the v0.3 golden migrates and opens with an identical extraction
  report. Round-trip byte-identical save→load→save for a v0.4
  sketch document.

**Rejected alternatives:** PATCH-riding field tolerance (the
silent-downgrade data loss 0027 D4 already rejected); recipe-only
files (not self-contained); sketch-as-blob (the entities must be
JSON-visible for the migrator chain and for diffability — the same
reason the operation recipe is JSON).

### D5. Interaction specification (binding) — direct manipulation is the outer face, the recipe is the real object

**What this revision is.** The draft's D5 was a UI/UX sketch: entities
entered through property-panel dialogs, dragging deferred, snapping
deferred to the solver era. This revision replaces it with a **binding
interaction specification**: every clause below is normative for MVP
unit 4 — the unit is done when the clauses hold, and D6's smoke asserts
them. The governing principle, stated once and applied everywhere:

> **Direct manipulation is the outer face; the recipe is the real
> object.** Every graphical gesture is a write of parameters into the
> sketch node's recipe. No free geometric state exists that the
> document cannot rewrite.

This is the ADR-0005 discipline at 2D granularity. As silent rebind is
forbidden, so is the silent constraint: nothing graphical may change a
value the user did not see and accept. The canvas (the new QPainter
widget, no GL — the draft's rendering decision stands; the world XY
plane seen from +Z) shows exactly one of three things: (a) a rendering
of recipe parameters, (b) a transient preview of the gesture in
progress, (c) an acceptance glyph for a suggestion (D5.4). There is no
fourth thing; a fourth thing is a bug.

**Two one-sentence amendments folded at their sites** (this ADR's
facts-over-drafts habit, applied to itself): D2's "every chain closed"
becomes the EXTRUDE's precondition and the definition state's target —
not the sketch node's right to exist, or progressive drawing is
impossible (an open chain is a legal, visibly-flagged editing state);
D4's loader splits its error — parameter-class corruption is a loud
load error, an open chain loads as the same legal editing state with
the extrude pending in its visible error. Closure moved from "the
sketch must be born closed" to "the extrude demands closure and the
canvas shows the distance to it" — the natural shape of drawing.

#### D5.1 Creation — drawn on the canvas, with named parameters

**Choice.** Entities are drawn directly on the canvas — never entered
through dialogs. The gestures, pinned:

| Entity | Gesture (pinned) | Parameters written (the names, as serialized — D4) |
|---|---|---|
| **segment** | press at A, drag (live rubber-band A → cursor), release at B | `x0, y0, x1, y1` |
| **circle** | press at C (the center), drag, release — r = ‖release − C‖ | `cx, cy, r` |
| **arc** | press at C, drag, release — fixes r and a₀ = arg(release − C); move (live sweep preview from a₀); click — fixes a₁ = arg(cursor) | `cx, cy, r, a0, a1` |

Angles in radians, CCW positive — the document's own numbers; no
undeclared unit conversion anywhere in the pipeline (a degrees display
is D5.7's: a declared presentation change, never a hidden transform).

- **Live preview during the gesture** — the rubber band, the growing
  circle, the sweeping arc: rendering only, the parameter table
  untouched. The preview is category (b) of the principle; it writes
  nothing.
- **Cancelable**: ESC or right-click during the gesture aborts it — no
  entity, no write, no undo entry. An abort is not an undo: there is
  nothing to undo.
- **Completed gesture = entity with parameters AND names**: the
  gesture's values land in the recipe 1:1 and appear in the property
  panel under the serialized names of the table. The names are part of
  this specification — a parameter without a name cannot be routed,
  edited, serialized or addressed. The panel keeps its TEXT role (the
  registry consumer pattern, zero new panel code — that part of the
  draft survives); what dies is the dialog as the CREATION path.
- **New circles are OUTER contours by a declared default**: the chain
  role (outer / hole — D3's disc vs complement orientation) is a
  recipe parameter, created as outer and togglable per chain in the
  panel, with a distinct aspect per role (D5.3's table) so the default
  is VISIBLE, not silent. Auto-classification by geometric containment
  ("a circle inside the outer chain must be a hole") is rejected
  below: it is exactly the silent write this ADR forbids — orientation
  decided by position.

**Rejected alternatives:**

- *Property-panel dialogs as the creation path (the draft's own text)*
  — form-filling geometry: no gesture, no preview, nowhere for the
  coincidence suggestions to live. The panel keeps text editing
  (D5.2); creation moves to the canvas.
- *Free polyline capture (click-click-click, one entity per session)*
  — the typed-entities decision (D1) verbatim: a polyline has no
  radius to edit; it would also force an entity-splitting pass with
  its own silent choices.
- *Arc by three points (start–mid–end)* — writes derived parameters:
  center and radius are solved from the points inside the gesture and
  the user never sees them being written; the center–radius–angles
  gesture writes exactly the recipe's parameters, 1:1. Three-points
  remains a legitimate future gesture (D5.7) provided it displays the
  derived values before the write.
- *A dedicated sketch editor window owning the node* — rejected in the
  draft, reconfirmed by the principle: the canvas is a VIEW of the
  node (D5.6), not an application around it.

#### D5.2 In-place editing — handles are parameters

**Choice.** Selection first, the draft's honest rule unchanged:
**click picks the nearest entity within 10 px, measured with the
per-entity EXACT formulas** (point–segment distance, |P − C| − r, the
clamped arc distance) — never the composed field (D3's honesty rule:
the field is not a measuring stick). The selected entity shows its
handles and routes its parameters to the panel. Then, editing:

| Entity | Handle | Parameter(s) written |
|---|---|---|
| segment | endpoint 0 / endpoint 1 | `x0, y0` / `x1, y1` |
| circle | center | `cx, cy` |
| circle | radius point (cx + r, cy) | `r` |
| arc | center | `cx, cy` |
| arc | radius point (mid-angle) | `r` |
| arc | endpoint at a₀ / endpoint at a₁ | `a0` / `a1` (r held) |

- **Dragging a handle modifies the corresponding parameter(s)**, and
  the dependents regenerate in the DAG as always. Commit at release:
  during the drag the canvas shows the preview (category (b)); on
  release the parameters are written, the sketch is marked changed,
  and the DAG pass re-materializes every dependent — the established
  propagation (extrude → subtract → extraction) — as ONE undo entry.
  ESC during the drag aborts it: parameters untouched.
- **Double-click on a dimension = text edit of the value**: on an
  endpoint handle, the inline edit of that point's coordinates; on the
  circle/arc radius label, the edit of `r`. The rule for what is
  text-editable is pinned: **a dimension is editable if and only if
  its edit writes named recipe parameters with no hidden anchor
  choice**. The radius qualifies (it writes r). A displayed LENGTH
  (measured, read-only — the exact formulas again) does not: writing
  it requires choosing which endpoint stays fixed — an interaction
  decision, not a parameter write; it stays out with its criterion
  (D5.7). Gesture and text write the SAME named parameter through the
  SAME route — one truth, two affordances.
- **Never floating geometry**: nothing persistent exists on the canvas
  that is not a rendering of recipe parameters. If it is drawn and
  survives the gesture, it is in the recipe; if it is not in the
  recipe, it disappears with the gesture. A handle belongs to exactly
  ONE entity: two connected entities share an endpoint by VALUE (their
  coordinates are equal, D5.4), so dragging either endpoint can break
  the coincidence — the no-solver cost, paid visibly as the definition
  state re-opens (D5.3), never papered over.

**Rejected alternatives:**

- *Continuous write during the drag (live DAG regen per frame)* — N
  regenerations and a burst of undo frames for one user intent;
  enters only with its measured criterion (D5.7).
- *Moving the shared endpoint of both entities (a mini-solver inside
  the drag)* — two systems of truth (values and propagated relations),
  exactly what D1 excludes; the coincidence is values, and its
  breakage is visible.
- *Entity-wide transforms (move/rotate with a pivot)* — writes all an
  entity's parameters at once from an implicit pivot; deferred to
  D5.7 (the pivot must become an explicit, visible choice first).

#### D5.3 Visible definition state — and why the extrude does not care how you got there

**Choice.** In the no-solver regime, "under-defined" has one honest
meaning: CONNECTIVITY, not missing constraints. The operational
definition, pinned:

- An entity endpoint is **connected** when its coordinates coincide
  (within the joint tolerance of the D3 construction checks) with an
  endpoint of an adjacent entity in the same chain.
- A chain is **closed** when every entity in it is connected into a
  loop — a circle IS a closed single-entity chain.
- The sketch is **fully defined** when it carries exactly one outer
  chain and every chain (outer and holes) is closed. Anything else —
  a free endpoint, an open chain — is **under-defined**, and looks it:

| Canvas element | Pinned aspect |
|---|---|
| background | #1E1E1E, flat |
| entity in a closed chain, outer role | #FFFFFF, solid, 2 px |
| entity in a closed chain, hole role | #7E57C2, solid, 2 px |
| under-defined entity (free endpoint / open chain) | #E8A33D, dash [6, 3], 2 px — wins over the role color |
| selected entity | #4FC1FF, solid, 3 px, with 6×6 px square handles |
| gesture preview | #9AA0A6, dash [2, 2], 1 px |
| suggestion glyph | #4FC1FF, 5 px circle marker + the value it would write, as text |
| grid | #3A3F44 dots, 1 world-unit pitch (fixed — D5.7) |
| rejection message | #FF6E6E text in the canvas status bar, persisting until the next gesture |

The aspect table is normative: the state is computed from the recipe
(connectivity of the written values), the aspect is a pure function of
the state, and the smoke asserts the pair. The amber is a worklight,
not a judgment: it says "the extrude would reject this as it stands"
(D5.5) — nothing about the approach being wrong.

**The invariant under it**: a profile extruded from a valid sketch does
not depend on how the definition was reached. The D3 field is a
function of the final parameter values alone — same recipe state,
same prism, bit-identical (ADR-0014) — whether that state was reached
by gesture, by text, by drag, or by file. D6 asserts this as the
via-agnostic property: the same state reached two ways, the same
bytes out.

**Rejected alternatives:**

- *No visible state (all entities alike)* — the user discovers the
  open profile at the extrude, the latest and most expensive moment;
  the amber exists to move that discovery to the cheapest one.
- *A three-state traffic light (under / partially / over)* — without a
  solver there is no "over", and "partially" would say what "loose"
  already says.
- *Badge counts instead of color ("2 free endpoints")* — readable but
  not LOCAL: the state must be seen on the entity that carries it.

#### D5.4 Suggestions, never silent applications

**Choice.** During a gesture, alignment candidates are computed and
OFFERED — never applied. The MVP candidate set, pinned (thresholds in
screen pixels, converted to world units at the current zoom):

| Candidate | Condition (at the cursor) | If accepted, writes |
|---|---|---|
| horizontal | ‖Δy‖ < 6 px between the gesture's anchor and the cursor | `y1 = y0` (the anchor's value) |
| vertical | ‖Δx‖ < 6 px | `x1 = x0` |
| endpoint coincidence | cursor within 8 px of an existing entity endpoint | the new endpoint = that endpoint's coordinates (equal values) |
| grid point | cursor within 6 px of a grid point (1-unit pitch) | the quantized coordinates |

- The active candidate (the nearest) renders as the glyph — category
  (c) of the principle — WITH the value it would write, as text:
  seeing what you accept is the rule.
- **Acceptance is an explicit act**: hold SHIFT at the release/click
  that completes the gesture → the candidate's values are written.
  Complete without SHIFT → the pointer's raw values are written,
  verbatim. No threshold exists below which the snap happens by
  itself. A declared default (the circle's outer role) is not a silent
  application; an undeclared geometric inference (containment,
  near-horizontal auto-flatten) is one, and is forbidden.
- **Accepted = explicit parameters, not constraints.** The horizontal
  suggestion accepted writes `y1 = y0` as two numbers that happen to
  be equal — D1 UNCHANGED: no relation is stored, no propagation is
  armed. Dragging the endpoint later can break the alignment; nothing
  objects, nothing repairs — the honest no-solver cost, and any such
  break is visible as a state change. The coincidence suggestion has
  a second, STRUCTURAL effect: the new entity is appended to the chain
  of the entity it connects to (the chain is recipe structure — D2);
  that appending is what closes contours.
- This is the naming rule extended to 2D: **as the silent rebind is
  forbidden, so is the silent constraint**. The full SOLVER
  (persistent relations, DOF counting, repair on edit) stays deferred
  exactly as D1 deferred it; when its ADR opens, these glyphs become
  the display of its constraints — the vocabulary is already the
  honest one.

**Rejected alternatives:**

- *Auto-snap below threshold (the common 2D-editor default)* — writes
  values the user never confirmed; the 2D form of the silent rebind,
  and the reason this subsection exists. It also lies about precision
  in the draft's old direction — coordinates that look intentional
  when they are accidental.
- *Context-menu candidate selection* — interrupts the gesture; a menu
  is a commit, not an offer.
- *TAB cycling through candidates* — legitimate, deferred with its
  criterion (D5.7): only when real usage shows the nearest-candidate
  rule picking wrong often enough to matter.

#### D5.5 Validation at construction, also in UI — the gesture never completes an out-of-class state

**Choice.** The Extrude command: with a sketch node selected, Edit →
Extrude opens the minimal dialog (distance, direction ±Z; defaults
1.0/+Z); OK creates the operation node — a top-level row "Extrude
(Sketch#N)", Type column "FEAT" — and the prism appears through the
ordinary MC path, zero renderer work. The ADR-0029 D5 error class —
construction-time rejection — is raised to the gesture layer: the same
class, the same predicates, ONE opinion. The kernel's construction
checks (D3's table: p₀ ≠ p₁, r > 0, sweep ∈ (0, 2π), finite
parameters, d > 0, closure at materialization) are exposed and called
by the UI; the UI keeps no second copy of the rules.

- **Rejected at the gesture**: degenerate release (press ≈ release: a
  segment below the gesture threshold), r ≈ 0 (release on the
  center), degenerate sweep (a₁ within the sweep threshold of a₀, or
  at ≥ 2π − that), degenerate drag in editing (the radius point onto
  the center). No entity is created, no parameter changes, and the
  rejection is VISIBLE: the message in the canvas status bar (#FF6E6E,
  the pinned channel), naming the refused value, persisting until the
  next gesture begins.
- **Rejected at the command**: Extrude on a sketch with open chains →
  the command is refused and the open entities are NAMED in the
  message (the 0029 habit: name the offender). Extrude dialog:
  distance ≤ 0 or non-finite → OK is refused with the message.
- **Rejected at the text edit**: r ≤ 0, NaN, empty → the field
  refuses; the previous value stands.
- The gesture threshold (what counts as degenerate) is 2 px at the
  current zoom — DISTINCT from the suggestion thresholds (6/8 px):
  suggestions may offer, only degeneracy refuses. A value that is
  merely ugly but in-class (a very short segment drawn slowly) is
  ACCEPTED — the UI judges class, never taste.

**Rejected alternatives:**

- *Accept-then-fix (create the entity, silently correct the value)* —
  a silent write by definition; it also creates the very out-of-class
  state it was supposed to prevent.
- *A UI-side rulebook (the UI's own validation code)* — divergence is
  the certainty: the UI accepting what the kernel rejects, or
  vice versa; the single source is the point.

#### D5.6 The canvas is a node like the others

**Choice.** The 2D canvas is a VIEW of the sketch node — nothing more,
with exactly the privileges of nothing more:

- **Opening**: Create → Sketch, or double-click on the sketch's row in
  the feature tree. The node lives in the tree with its genealogy and
  its dangling state EXACTLY as the subtract's operands do (ADR-0027
  D2, inherited verbatim by D2 of this ADR): deleting a sketch
  referenced by an extrude SUCCEEDS, the extrude enters the visible
  dangling state — tree row flagged, property panel actionable, last
  body kept, display continues — never silence. The open canvas view
  of the deleted sketch CLOSES: the node is gone; undo recovers it,
  id-stable, and the view can be reopened.
- **One view per sketch** at a time (MVP). Closing the view commits
  NOTHING — there is nothing to commit: every completed gesture
  already wrote. That is the principle; closing just stops looking.
- **View state is not document state**: pan and zoom are transient,
  live nowhere in the recipe; a reopened sketch fits its content.
  (Per-session persistence may come later — D5.7 — never into the
  file.)
- **Undo granularity = the gesture**: a completed creation is one
  entry, a released drag is one entry, a text edit is one entry, an
  aborted anything is zero entries. The command stack stays the
  single history; the canvas keeps no private one.
- **Forbidden, explicitly** (the parallel-document list): persistent
  geometric canvas state beyond the recipe; a draft buffer between
  the gesture and the write; sketch data in any file but the
  `.CAD_0`; a second editing truth — the canvas and the panel always
  show the same recipe, the panel being the text view of what the
  canvas shows graphically.

**Rejected alternatives:** the dedicated editor window (fights the
tree, owns the node — rejected in the draft, reconfirmed by the
principle); the draft-then-commit buffer (a document beside the
document — the F-1 lesson at 2D granularity); multi-view sync (doubles
every write path for a scenario the MVP does not have — D5.7).

#### D5.7 What stays out of this specification, each with its entry criterion

- **Fine snapping** (midpoint, intersection, tangent, perpendicular
  display) — enters when the solver's ADR opens (these are its
  constraint vocabulary) or when measured open-profile rejection rates
  show base coincidence insufficient in practice.
- **Configurable grid** (pitch, subdivisions) — enters when measured
  usage shows the 1-unit pitch useless in the working domain
  (sub-millimeter or tens-of-meters parts) and the requests recur.
- **Degrees display** — a declared presentation toggle over radians;
  enters with the first real sketch session (cosmetic, but it is a
  conversion and it will be labeled).
- **Entity-wide transforms** (move/rotate an entity with a visible
  pivot) — enters when per-parameter editing proves too slow for real
  profiles; the pivot must be an explicit, visible choice (the anchor
  rule of D5.2).
- **Derived-dimension inline editing** (segment length, Δx/Δy) —
  enters per dimension once its anchor semantics is written down
  (which endpoint stays fixed) and requested; today they are read-only
  measures.
- **Live regeneration during the drag** — enters when the measured
  regeneration latency makes release-commit feedback inadequate for
  the workflow.
- **Arbitrary work planes / profiles on faces of bodies** — unchanged,
  D2's and the out-of-scope section's criteria (the frame-change
  derivation to the 0029 standard; subshape stability).
- **Graphic timeline** — enters when real documents carry enough
  features that the flat tree + genealogy stops answering "what
  consumed what, in which order" at a glance; until then the tree IS
  the timeline.
- **Multi-view of one sketch** — enters with collaboration or
  dual-monitor demand; one write path until then.
- **TAB candidate cycling** — enters when the nearest-candidate rule
  mispicks in practice.

#### D5.8 The CLI of the phase — unchanged, and still the acceptance demo

*Sketch a circle → extrude it → subtract it from a box = the parametric
hole, by the book* — delivered as `examples/sketch_hole.py`, exercising
the whole stack: entity creation on a sketch node, the extrude
recipe, the subtract referencing the extrude's FeatureId, an edit of
the circle's radius propagating bit-identically through the chain, a
v0.4 save/load round-trip, and the genus-1 extraction report of the
pierced box. This combines F-1 and F-2 and touches naming, DAG, format
and bridge. The example and the canvas are the two faces of the same
recipe: the example proves the document model, the canvas proves the
interaction model, D6 asserts both.

### D6. Test plan (contracts, not coverage)

C++:

- **2D property families (the ADR-0029 discipline, failure regime in
  range):** pairwise |f| ≤ d(P, boundary) on seeded random point/entity
  grids (the theorem, live); the corner gap pinned — right angle,
  external bisector, f = t vs d = t·√2 (measured once, frozen); L = 1
  vs pairwise slopes on the composed profile; gradient-vs-FD
  component-wise with the seam scope documented (the box/intersect
  precedent); the concave failure regime — a notch- deeper-than-wide
  profile REJECTED at construction with the entity named (the loud
  error, not the silent sign flip); ring profile (outer circle +
  hole circle) evaluates the annulus with exact sign on a stratified
  grid (band × slice, the 0029 habit).
- **Extraction contracts on extruded profiles (the old contract
  works):** extruded disc = cylinder topology at the same grid
  resolution — **same MC configuration (signs at grid points agree,
  same zero-set ⇒ same V/E/F), positions NOT bit-identical** (the
  composite field interpolates differently than the exact cylinder —
  declared, and asserted as a bounded position delta, not a
  coincidence); χ = 2, `is_closed_orientable`, wrapper equivalence,
  loops closed, every edge 2 co-edges + XOR, min |f| > 0 over the
  pinned grid (P2), deterministic re-run bit-identical. **Extruded
  ring = genus 1:** χ = 0, genus derived (the 0027 D6 rule),
  orientable, the compatibility wrapper false (χ ≠ 2 — the
  zero-silent-weakening check), a second grid with different counts
  and the same χ (the T2 habit).
- **DAG:** sketch entity edit → extrude re-materialized
  bit-identical to fresh construction; sketch delete → extrude
  dangling (visible, body kept); undo (same id) → recovered; undo
  interleaving with the subtract on top (the LIFO chain
  sketch→extrude→subtract torn down and rebuilt with the same ids at
  every step).
- **Format:** v0.4 round-trip byte-identical; the v0.3 golden
  migrates with an identical extraction report; v0.1/v0.2/v0.3
  goldens byte-immutable; a dangling-sketch file loads with the
  error state; cyclic/self-reference → explicit load error; a
  corrupt sketch entity (NaN radius, r ≤ 0) → the ADR-0029
  family error, named; an open chain → loads as revision 1's legal
  editing state (D5's amendment), the extrude pending in its
  visible error.

Python / bindings:

- The report and scene surfaces the dialog and example need; the
  sketch/extrude surfaces mirror the subtract's.
- **UI smoke (offscreen, the established driver) — gesture semantics
  asserted (D5's clauses, each mapped to a test):**
  - *Creation*: an injected press–drag–release draws a segment → the
    entity exists WITH the expected parameters (the gesture's values
    in the named fields, 1:1); same for the circle, and for the arc's
    two-phase gesture (radius+start, then sweep+end).
  - *In-place edit*: endpoint drag released → the parameters changed
    AND the dependent DAG re-materialized; the same final state
    reached by text edit produces the bit-identical prism — the
    via-agnostic property of D5.3 (ADR-0014 across interaction
    paths).
  - *Suggestion refused* (release without SHIFT, a candidate under
    threshold active): the recipe holds the pointer's raw values,
    byte-for-byte — no hidden write. *Suggestion accepted* (with
    SHIFT): the candidate's values, explicit; the chain structure
    updated on an accepted coincidence.
  - *Out-of-class input during the gesture* (degenerate release,
    r = 0, degenerate sweep): NO entity created, the rejection
    message visible in the pinned channel (the canvas status bar),
    the recipe byte-identical to before the gesture.
  - *Abort*: ESC mid-gesture → no entity, no undo entry; ESC
    mid-drag → parameters unchanged.
  - *Definition state*: an open chain renders under-defined (the
    pinned amber/dash; the state derived from recipe connectivity);
    Extrude refused with the open entities named; after an accepted
    coincidence closes the chain → Extrude succeeds.
  - *The draft's end-to-end flow stands*: create sketch → circle →
    extrude → subtract from a box → the hole is in the tree and the
    Debug report shows χ = 0, genus 1; radius edit propagates
    (report follows); delete sketch → dangling row; undo →
    recovered, same ids; save/load round-trip with the same report;
    undo is gesture-granular (LIFO over creations and edits).
- Python 3.10 f-string checker clean on every touched file.

## Proposed MVP — the parametric hole, end-to-end

1. **Kernel: the 2D fields + the extrusion (D3)** — `Halfplane2D`,
   `Disc2D`, `ProfileSDF`, `SlabZ`, `ExtrudeSDF`; the validation
   table rows live in the headers; the construction checks; the 2D
   property families with the failure regime in range. (sdf)
2. **Document layer (D2)** — `NodeKind::Sketch` + entity payload +
   chain; `OperationType::Extrude` + recipe; role-list
   generalization of the rebuild pass; id-stable create/extrude
   commands; the dangling state for a missing sketch. (scene)
3. **Format (D4)** — v0.4 writer/reader (two-pass), the 0.3 → 0.4
   migrator, the v0.4 golden, the old goldens immutable. (io)
4. **App + CLI (D5)** — the canvas 2D widget implementing the D5
   interaction clauses (gesture creation, in-place editing, definition
   state, acceptable suggestions, construction-time rejection — the
   whole binding specification), the Extrude dialog, entity params in
   the property panel, `examples/sketch_hole.py`, the UI smoke suite
   asserting the gesture semantics, the genus-1 report on the
   parametric hole. (app + bindings; splits into two commits at
   implementation time if the canvas and the example pull in different
   directions)

Zero renderer changes. The v0.1/v0.2/v0.3 goldens are immutable. One
commit per unit, CI green after each — and the subject-lint gate of
this repo's CI (introduced alongside this ADR) judges every one of
those subjects.

## Explicitly out of scope (each with its entry criterion)

- **The constraint solver** (coincidence, parallelism, tangency) —
  its own ADR. Entry criterion: this MVP delivered and the
  propagation value demonstrated (the parametric hole); the solver
  then binds real entity parameters with real regression tests
  under it.
- **Arbitrary work planes** (sketch on a plane other than world XY) —
  entry criterion: the frame-change derivation written to the 0029
  standard (L = L₂D·‖J‖ with the rigid-J bound, rotated gradient,
  the bounds transform) AND a consumer that needs it (which is
  effectively sketch-on-face).
- **Sketch on faces of existing bodies** — the real target of the
  above. Entry criterion: subshape stability across rebuilds
  (ADR-0026 D4 / ADR-0027 D3's deferral — a referenced FACE must
  survive operand edits, which is persistent naming of extracted
  entities) + arbitrary planes. This is Phase F-2's research core.
- **Multiple outer contours in one sketch** (two disjoint solid
  regions) — needs the hierarchical composition (profile = UNION of
  sub-profiles = min over sub-fields), and min of NON-exact fields
  is not the exact union distance (the sub-fields already
  understate): a new table row and derivation. Entry criterion: a
  use case that a second sketch node cannot serve (today: just make
  a second sketch).
- **Revolve / loft / sweep** — each a different 2D→3D semantics;
  amendments with their own derivations when a consumer exists.
- **Snapping (fine), magnetic grid, fine picking** — D5.7's entry
  criteria. The base suggestion set (horizontal/vertical, endpoint
  coincidence, grid point — offered, never auto-applied) and the
  in-place drag are IN this MVP, per D5.4/D5.2.
- **Graphic timeline** — D5.7's entry criterion (the flat tree +
  genealogy remains the honest reading of consumption order).
- **Hierarchical feature-tree display** — reconfirmed F-2 polish.
- **Param-edit undo** (the pre-existing `set_param` gap, unchanged
  by this phase; noted in 0027 and still open).

## Consequences

**Positive:**

- The second increment spends the architecture the first one burned
  in: naming, DAG, format and bridge contracts are inherited, not
  re-derived — the sketcher's cost concentrates exactly where its
  new value is (the 2D semantics and the canvas).
- The kernel's first 2D operators arrive VALIDATED (the 0029 gate
  honored on arrival): closed Lipschitz proofs, a named conservative
  regime, construction-time rejection of the sign-breaking class —
  the twist postmortem's lesson applied proactively.
- The parametric hole — the canonical CAD moment — runs end-to-end
  through naming, propagation, persistence and a genus-1 extraction.
- The solver is positioned as research ON TOP of a delivered data
  model instead of a blocker in front of a speculative one.

**Negative / risks:**

- The composed field is not a distance field — every consumer that
  needs measurement must use the per-entity exact formulas (the
  picking rule states it; a future contributor who measures with the
  field will get conservatively-short distances — the D3 text is the
  guard).
- A third `NodeKind` touches every kind-dispatch in the tree
  (`update_meshes`, bindings, serializer, app loops — an audited but
  real surface; the Mesh-kind precedent says the skips are cheap).
- The contour self-certification is sampled (joints + entity
  midpoints), not exhaustive: a pathological profile whose
  violation lives strictly between samples could pass construction
  with a locally wrong sign. Declared honestly; the sampling points
  are ON the entities themselves, where the claim is strongest, and
  the property families probe the composed sign on grids.
- The format chain grows again (three migrator hops by now); each
  hop is a no-op but the chain must be walked and tested forever.

**Neutral:**

- The draft's "composed by min" is corrected to max in-place, with
  the reasoning in D3 and the record in Status — facts over drafts.
- The C.7 `feature_tree.py` stays unwired (its moment is the
  hierarchical display, still out of scope).

## Experiments

None block this ADR — every decision rests on verified repository
facts (the Context table) and derivations in closed form. The
implementation phase must **measure and freeze** (the house method):

- The right-angle corner gap on the external bisector (expect f = t,
  d = t√2 to float precision; frozen as the pinned property).
- The extruded-disc vs primitive-cylinder counts at the pinned grids
  (same V/E/F expected from sign-configuration equality; the position
  delta bounded and frozen).
- The v0.4 golden bytes, after the writer is deterministic by
  construction.

If implementation finds the sampled self-certification missing a
real failure shape (a profile that passes construction with a wrong
sign somewhere), the sampling rule is amended in place — the
loud-error class never silently widens.

## References

- ADR-0027 — the first Phase F increment: the architecture this ADR
  inherits (D1's second increment, D2's genealogy verbatim, D4's
  format method, D6's χ amendment).
- ADR-0005 — persistent naming by genealogy (the FeatureId rule and
  the signature-matching ban).
- ADR-0002 — dual kernel, honest dispatch (the third NodeKind).
- ADR-0007 / ADR-0007a — format versioning and the blob container
  (D4 discharges the bump-ADR obligation).
- ADR-0026 — the SDF → B-Rep bridge (P1–P4; the extraction contracts
  D6 reuses; D4's deferral that gates sketch-on-face).
- ADR-0028 / ADR-0029 — the bounds-contract audit and the operator
  gate: "new operators arrive validated" (D3's derivations, table
  and construction checks are that gate applied on arrival).
- Quilez, I. — 2D distance functions (the exact per-entity formulas:
  segment, circle, arc — the "well-known exact formulas" the entity
  fields and the picking rules use).
