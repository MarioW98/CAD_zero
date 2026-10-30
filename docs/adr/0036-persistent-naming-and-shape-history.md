# ADR-0036: H2 unit 6 — persistent naming and the shape history: the derived name, the descent table, the resolution through mutation

## Status

**Accepted (2026-10-31) — the maintainer's review closed the same
day the draft shipped** (the ADR-0033 precedent: the reserved
decision taken and recorded in the act, the review's amendment
riding the acceptance, the register surgery applied in the same
commit). The semantics stand as derived; the implementation ladder
(D6) starts with this act — morso 2, the roots, is green-lit as
the companion feat commit of this acceptance.

**The review's record, registered here:**

1. **The derivation is the contract.** D1's address/name split, D2's
   six relations with the closure property, D3's four resolution
   classes with their dispositions, D5's pre-registered goldens —
   accepted as written. The industrial-contract target the
   maintainer set for the unit (the Parasolid discipline — typed
   tags, derivable tracking, hard invariants — applied to this
   house's foundations) is what the derivation delivers; nothing is
   deferred to implementation discretion.
2. **ONE amendment, from the review, applied to D4** (below): the
   root-edit queue names the **MIDDLE-DELETION** beside the
   middle-insert, and the **rule for shifted roots** is derived and
   registered — the edit path that shifts referenced roots EMITS
   the map or REFUSES with the remedy; `reference_resolved` /
   `reference_ambiguous` / `reference_vanished` remain the only
   resolution classes, and a silent positional match on shifted
   roots is the defect the rule exists to make impossible. The
   operational decision (where the map is emitted, what the refusal
   carries) belongs to the resolver's morso (D6 rung 4); what this
   act pins is the RULE and the BOUNDARY.
3. **The register surgery D6 declares is applied in the same act**:
   ADR-0005 is amended in place (the `SubshapeRef` semantics —
   indices into the declared role tables; the Phase-A consequence
   superseded by this ADR's D5 quartet; the generation clause
   ratified at the feature granularity the scene already practices;
   the ABI itself unmoved). ADR-0032 D2 stays untouched, for the
   reason D6 derives (identity is not arithmetic honesty); ADR-0007/
   0027 stay untouched (the v0.5.0 delta is the fillet unit's).

No code moves in this commit (the ADR-0035 D6 protocol): the roots
are the companion feat commit of this act.

Daughter of **ADR-0005** (the persistent-naming promise this ADR
redeems — the ABI shipped in Phase A and never once populated),
of **ADR-0002 + ADR-0027** (the recipe doctrine: the recipe is
the source of truth, the body is a cache — the foundation the
name's purity stands on), of **ADR-0014** (bit-identical
determinism — the name inherits the bar), of **ADR-0021 and
ADR-0032 D2/D3** (the entity set, the ID scheme, the Layer
contract the names address without touching), of **ADR-0031 D5 +
ADR-0033 D3/D4** (the fillet — the first feature that MUST
reference a subshape, and v1's scale), and of **ADR-0035** (the
method: the contract derived and pinned before the code, the
register surgery declared, the goldens pre-registered).

**Why before the fillet**: the naming contract must exist BEFORE
the feature that consumes it. A fillet whose edge reference is a
positional index is a defect factory wearing a feature's clothes;
this ADR is the floor the G2 fillet unit will stand on, derived
while there is no consumer yet to bend the derivation.

## Context

The state, verified against the tree at 6525fa0 (the repo is the
authority; the working tree is clean on the last green, run
#122):

1. **The boolean paths number positionally, and no operation
   records descent.** In the polygon path, `assemble_body`
   (boolean.cpp:1281) rebuilds the result from scratch:
   `weld_vertex` mints `VertexId = result.vertices.size()`
   (:1320), `get_or_create_edge` mints `EdgeId =
   result.edges.size()` (:1338), faces take `next_face_id++`
   (:1344, :1425). The fragment's operand of origin is KNOWN
   where the clip loop runs (`for (const auto& face_A :
   shell_A.faces())`, :2066) and is DISCARDED at the `ClippedFace`
   boundary — the struct carries `surface` and `polygon`, nothing
   else (:1262-1265). The exact path is better but still
   positional: `exact_subtract` copies A verbatim
   (`R->vertices = A.vertices; R->edges = A.edges`,
   subtract.hpp:1128-1129) and appends the new entities in the
   tool's canonical walk order — a DETERMINISTIC numbering,
   pinned by tests as tables (subtract.hpp:113-139,
   extrude.hpp:73-93) — but the numbering is an ADDRESS, not an
   identity: nothing records that the wall descends from the
   tool's lateral, that the pierced face absorbed the tool's
   circle, that the rim arcs were born of the box's top face
   meeting the wall. **The descent is computed by every algorithm
   in this repo and thrown away by every one of them.**

2. **The promise is unredeemed at the subshape level.** ADR-0005
   shipped the `ShapeId` ABI — `FeatureId` + `SubshapeRef` (a
   path of indices) + `Generation` (shape_id.hpp) — and the
   correction that matters (genealogy only, never geometric
   signature matching). Today: `SubshapeRef` is populated by no
   code path in the tree (the only reader is `to_string`,
   shape.cpp:60); the generation rule lives at FEATURE
   granularity — the scene's `operand_missing` protocol, "a
   dangling reference is REPORTED, never silently re-picked"
   (scene.hpp:271-274); the UI picks at NODE granularity
   (`_selected_node_id`, viewport_widget.py:82); the report
   queries faces BY POSITIONAL ID (query.hpp:119-121: "the
   factories number them by shell order; the booleans number
   their fragments").

3. **Why this is the hard problem of parametric CAD, in one
   sentence**: a feature that references a subshape generated by
   a boolean, after an edit that changes the model's topology
   (grow the radius → the topology re-shapes), either cannot find
   the entity again or finds the WRONG one — because a
   progressive ID is a POSITION, and the position of an entity
   and the identity of an entity are different things that
   today share one number. The failure mode is worse than "not
   found": the silent mis-reference (the id still resolves, to
   another entity — D5's counterproof demonstrates it happening).

4. **What already exists and is reused, not rebuilt**: the
   numbering tables ARE deterministic functions of the recipe
   (the exact path's walk order; the factories' construction
   order) — the discipline ADR-0005 asked for, implemented as
   pinned tables. What they lack is (a) the GENEALOGY (they
   number, they do not relate) and (b) the STABILITY ACROSS
   RECIPE EDITS (an upstream edit that inserts an entity shifts
   every appended position while the identity survives — the
   D5.4 scenario, derived below). This ADR builds ON the tables;
   it does not replace them.

## Decision

### D1. What a name is — the address is not the identity

**The split, pinned first.** Every entity of a materialized body
carries two orthogonal attributes from now on:

- **The ADDRESS** — the positional `FaceId`/`EdgeId`/`VertexId`
  of today (topology.hpp:35-38): the array index, the O(1)
  lookup key, MUTABLE across generations, internal. Every
  existing golden, every existing query, every existing test
  reads the address and keeps working unchanged. The address is
  the body's own bookkeeping.
- **The NAME** — the entity's identity: a STRUCTURE derived from
  the recipe, STABLE across regenerations and across the edits
  that preserve the entity's existence, public (the surface
  features and the UI reference by name, never by address).

**The name is a pure function of the recipe.** Consequence of
ADR-0027 D3 (the recipe is the source of truth; the body is
cache): the name is derived at materialization and derives again,
identically, at every re-materialization — same recipe → same
names, bit-identical, with no stored state and no memory
(ADR-0014's bar, inherited by construction: names are strings and
integers, zero floating-point content, so the determinism is
trivially total). There is nothing to update and nothing to
invalidate; a name that differs after a rebuild is a defect, not
an event.

**The name's structure — three components, mirroring the
recipe's own:**

```
name := <feature> ":" <role>
```

1. **`<feature>`** — the FeatureId of the DAG node whose output
   CREATED the entity. The creating feature owns the name
   forever: an entity that SURVIVES an operation (kept, absorbed)
   keeps its name — the pierced face of the guide case remains
   the BOX's face, not the subtract's (identity is where a thing
   was born, not where it currently sits; the current container
   is the address's business).
2. **`<role>`** — the entity's role in that feature's output,
   drawn from the operation type's **DECLARED ROLE TABLE**. The
   role tables are the existing NUMBERING tables (extrude.hpp,
   subtract.hpp, factory.hpp) made NORMATIVE: the tables the
   tests already pin become the vocabulary the names read. The
   factory roles are the construction roles (make_box's six
   faces: `bottom`, `top`, `front`, `back`, `left`, `right` —
   factory.hpp:126-131; the seam-wired sphere/cylinder carry
   their ADR-0023 seam roles); the extrude's are the walk roles
   (`lat(i)` for chain entity i's lateral, `cap(lo)`/`cap(hi)`,
   the circle special case's single `lat(0)` wall); the
   subtract's born roles are RELATIONAL and carry the genealogy
   inside them (below).
3. **The genealogy** — for entities CREATED by an operation, the
   role names its operands BY THEIR NAMES: the relation is part
   of the role. The genealogy is finite and acyclic because the
   DAG is (a role references only strictly older features).

**The relational roles (the classes the project has today):**

- **The extrude** (born): `ext#F:lat(i)`, `ext#F:cap(lo)`,
  `ext#F:cap(hi)` — genealogically tied to the sketch's chain
  entity i (the authored root; D4 pins the roots' own identity).
  Multi-chain: the chain index prefixes the role
  (`c0:lat(i)`, `c1:lat(i)` — the numbering table's own
  multi-chain layout).
- **The exact subtract** (born): the SSI curves —
  `sub#F:isect[<A-face-role> × <B-entity-role>]@lo|hi` — the
  intersection curve of the named operand pair at the named
  z-level; and the seam — `sub#F:seam[<B-role>]`. The SSI
  table's entries (G2 unit 1) name their born curves in this
  form: the entry id plus the operand pair — one grammar for the
  table and the boolean.
- **The trimmed faces** (absorbed, not born): a face that gains
  an inner loop KEEPS ITS NAME — the piercing is recorded in the
  descent table (D2), not minted into a new identity. The
  pierced top face of the guide case stays `box#1:top`; its new
  inner loop is the born `isect` curve's own name.

**The guide case's names, written out** (the chain:
`box#1` = the factory box, `sk#2` = the circle sketch, `ext#3` =
the tool extrude, `sub#4` = the subtract; box a×b×c, through hole
r — the ADR-0032 D7.5 body):

| Entity | Name | Relation into `sub#4`'s result |
|---|---|---|
| the hole's cylindrical wall | `ext#3:lat(0)` | absorbed (re-framed to the window, region-on-right) |
| the pierced top face | `box#1:top` | absorbed (+ the inner loop born of the rim) |
| the pierced bottom face | `box#1:bottom` | absorbed (same, @lo) |
| the four side faces | `box#1:front` / `back` / `left` / `right` | kept |
| **the top rim circle** (the fillet's reference) | `sub#4:isect[box#1:top × ext#3:lat(0)]@hi` | **born** (the SSI entry plane∩cylinder) |
| the bottom rim circle | `sub#4:isect[box#1:bottom × ext#3:lat(0)]@lo` | born |
| the wall's seam vertical | `sub#4:seam[ext#3:lat(0)]` | born |
| the tool's caps (dead) | `ext#3:cap(lo)`, `ext#3:cap(hi)` | vanished |

A rim circle is the LOGICAL intersection curve — ONE name —
whose materialization is the seam-wired arc pair (the ADR-0023
convention every periodic curve already carries; the pair is the
name's materialized form, not two names). The G2 fillet's
Feature→Fillet stores exactly `sub#4:isect[box#1:top ×
ext#3:lat(0)]@hi` — that string is this ADR's contract with the
fillet.

**The ABI mapping — ADR-0005's `ShapeId` survives, filled with
meaning.** `FeatureId` is the name's first component. The
`SubshapeRef` path's components are indices into the role
tables: the first component indexes the creating operation's
declared role table; the deeper components are the relational
role's operand references (the genealogy chain). `kMaxDepth = 8`
covers v1 with room (the guide case's deepest name resolves at
depth 4). `Generation` keeps the feature-granularity staleness
role it already plays in the scene; subshape resolution (D3)
supersedes the bare staleness check with the four classes — a
stale reference alone does not say WHICH failure it is, and the
user deserves the which. **No ABI change; the promise is redeemed
by giving the existing fields the semantics they were promised
with.**

### D2. The record of descent — the tavola, its relations, its cost

**Every operation, beside the body, emits the descent table:**
for each entity of the result, which operand entities it comes
from, under which relation; and for each operand entity that
died, that it died. The relation vocabulary, NAMED and closed —
six classes, derived from what the operations actually do:

| Relation | Meaning | Guide-case instance |
|---|---|---|
| `born` | created by this operation | the rim circles, the wall's seam |
| `kept` | survives 1:1, untouched — geometry, loops, Layer-2 record all the operand's own | the four sides, the 12 box edges, the 8 corners |
| `absorbed` | survives 1:1, MODIFIED — new loops, re-framed window, trimmed patch | the pierced faces (+ inner loops), the wall (re-framed) |
| `split` | one in → k out | (the circle→two-arc edit, D5.2) |
| `merged` | k in → one out | no instance in the exact path today — and that is a verified fact, not a gap: the flush cap is a 1:1 + 1:0 (the code "emits nothing for them, which IS the dedup", subtract.hpp:1123 — the pierced plane absorbs, the cap vanishes); the class exists because the closure property demands the vocabulary cover the genuine k→1 (the polygon path's coplanar dedup family; a future blend's corner closure) |
| `vanished` | dies in this operation, with the CAUSE recorded | the tool's caps (cut away outside the target — the flush variant: the coplanar dedup, subtract.hpp:69), the tool's rims and seam (re-authored with the window) |

`kept` and `absorbed` are separated ON PURPOSE, and the bill is
the Layer-2 record's ownership: a `kept` entity's provenance and
tolerance remain the operand's own untouched fields (subtract.hpp:
"A's surviving entities keep A's own records untouched"); an
`absorbed` entity carries the operation's mark (the combined
record where the operation creates, the loop changes where it
trims). Folding them would save one vocabulary word and lose the
only place the difference is checkable.

**The closure property, pinned hard**: every entity of the OLD
generation appears in exactly one row (`kept`/`absorbed`/
`split`/`merged`/`vanished`); every entity of the new generation
is `born` at most once (or inherits, via exactly one
`kept`/`absorbed`/`split`-child/`merged` row). A tavola that
loses an entity is a defect caught by its own closure check —
the Layer-1 spirit (an invariant the builder cannot silently
violate) applied to the history.

**Derived state, never parallel truth.** The tavola is recomputed
deterministically from the recipe at materialization — the same
pass that materializes the body, the same rule the HybridBody
twin already follows (scene.hpp:286-296). It is NEVER serialized
(ADR-0027 D3 decides who commands: the recipe does; the tavola is
cache). It lives beside the last materialized body in the scene
node — the state the resolution (D3) reads across an edit, the
same "keep the last good body" the dangling protocol already
practices.

**The ClippedFace lesson, stated as the design rule**: the
descent is not information to invent — it is information the
algorithms already hold transiently and discard (the clip loop
knows the operand face; `ClippedFace` drops it; the exact path
knows the tool walk and re-frames the wall without recording
that it did). The tavola is the RECORDING of known facts, at the
sites that know them, in the pass that already runs. Zero new
geometry, zero new decisions: pure bookkeeping.

**The cost, derived per operation and dimension.** One row per
output entity plus one row per vanished operand entity —
O(|V|+|E|+|F|) of the result and of the dying operands, integers
and strings, no floating-point content (ADR-0014 honored by
construction). The guide case's full table, counted:

| Kind | out rows | vanished rows | total |
|---|---|---|---|
| faces | 7 (4 kept + 3 absorbed) | 2 (the caps) | 9 |
| edges | 17 (12 kept + 5 born) | 5 (the tool's rims ×4, the tool's seam) | 22 |
| vertices | 12 (8 kept + 4 born) | 4 (the tool's own seam/antipode pairs) | 16 |
| **total** | **36** | **11** | **47** |

Forty-seven rows for the guide case — bounded by the golden-pinned
counts the tests already carry (V=12/E=17/F=7, χ=0: the same
numbers, now with their history beside them). At MVP scale this
is noise; the honest asymptotic bound is linear in body size,
per operation, with no geometric computation anywhere in it.

### D3. The resolution through mutation — four classes, each with a disposition

A downstream feature references an entity BY NAME. An edit
happens; the DAG re-materializes; the feature's reference must
resolve against the new generation. There are exactly four
outcomes, and each carries its pinned disposition — the house
error taxonomy (ADR-0033 D4's `fillet_*` family) applied to
parametric time:

1. **MATCH (`reference_resolved`)** — the name re-derives
   identically (the genealogy path untouched by the edit). The
   feature re-runs, silently, as it should. Concrete: the hole's
   radius grows within the class — the rim's name
   `isect[box#1:top × ext#3:lat(0)]@hi` is a STRUCTURAL fact, not
   a geometric one; the rim moved in space and kept its name.
2. **UNIQUE CONTINUER (`reference_renamed`)** — the referenced
   entity's row in the old generation's record has exactly ONE
   living descendant, and that descendant's NAME differs (the
   role table re-indexed under it). The reference follows to
   the new name, AND THE FEATURE'S LOG SAYS SO — the rename map
   is declared, never silent. **The honest v1 status, derived
   not papered**: the resolver branch ships (the tavola carries
   the rows it reads), but v1's authored flows produce no
   instance — the add-at-end root edits never shift a role, and
   the shifting edit (the middle insert: `lat(i)` is positional
   in the chain, an insert before i renames every later lateral)
   is exactly D4's REGISTERED root map, which arrives with a
   real case and brings this class its first consumer and its
   test. A class with machinery and no v1 instance is a
   registered boundary, not a promise.
3. **AMBIGUOUS (`reference_ambiguous`)** — the row has MULTIPLE
   living descendants (a `split` with k > 1 survivors). **The
   reference does NOT guess.** Error, actionable, naming the
   entity and every candidate by name, with the remedy
   (reselect, or reference the set explicitly). A reference is
   not a tolerance decision — the ADR-0035 D3 tie-break
   discipline is for banded arithmetic, and importing it here
   would be exactly the "approximate matching" bug ADR-0005 was
   written to prevent. Concrete: the tool's circle re-authored
   as a two-arc chain — the ONE wall face splits into
   `lat(0)` + `lat(1)`; a feature referencing the wall gets both
   candidates and the remedy, never a coin flip.
4. **VANISHED (`reference_vanished`)** — the row has NO living
   descendant. Error, actionable, naming WHAT died, WHY (the
   tavola's `vanished` row carries the cause — which decision
   killed it), and the remedy. Concrete: the tool's circle
   replaced by a square chain — the rim CIRCLE has no descendant
   (the sketch's authored entity was replaced; the new rims are
   `born` of new entities): the error says the rim died with the
   sketch's circle entity, names the edit class (entity
   replaced), and offers the remedy (reselect the new rim, or
   revert the edit). **Never a mute "not found"**: the not-found
   that cannot explain itself is the failure mode this entire
   ADR exists to make impossible.

And the guard under all four: a reference that finds NO ROW AT
ALL in the old tavola is not a user error — it is a closure
violation, a defect, surfaced as such (D2's closure check is the
resolver's own floor).

### D4. The v1 scope — pinned to the scale, the method of ADR-0031 D5

Not the naming of NX in one ADR — the minimum the CURRENT scale
consumes, with the queue named behind it. **v1 serves exactly two
consumers:**

1. **The fillet** (ADR-0033 D3, the G2 guide case): the
   Feature→Fillet references the rim edge BY NAME. This is the
   first feature in the product's history that cannot be built
   without this contract — the reason the unit exists now.
2. **The UI's face-level query** (the report/query surface): the
   click on a face in the report addresses the face by name; the
   D7.5-style table gains the name column beside family, loops,
   and provenance.

**In v1:**

- The role tables + names for the classes the exact path authors
  today: the three factories (box, sphere, cylinder — the
  construction roles), the extrude (single- and multi-chain),
  the exact subtract (the G1 class), the SSI born curves (the
  G2 unit-1 entries' grammar).
- The tavola for the extrude and the exact subtract (the
  factories' tavola is trivially all-`born` and is derived, not
  stored).
- The resolver: all four classes, on FACE and EDGE references.
- The surfaces: `FaceInfo`/`EdgeInfo` gain the name; the report
  prints it; the digest oracle extends (D5.1).
- The authored roots' identity (the sketch chain entities): the
  chain position IS the root name (the buck stops at authored
  data — there is nothing upstream to derive from), with the
  root rename map emitted by the edit path that knows it —
  v1 covers the canvas's add-at-end flow (the dominant authored
  edit; no positions shift); the middle-insert map is REGISTERED
  (below).

**Out of v1, each by name (the queue):**

- **Middle-insert and middle-deletion root maps** — AMENDED at
  acceptance (the review's one amendment, 2026-10-31): the queue
  names the middle-DELETION beside the middle-insert, because they
  are the same family of shift — an insert before existing chain
  entities renames every later root, and a deletion of an earlier
  chain entity renames them the same way (the surviving roots'
  positions all move down by one). v1 resolves the affected
  references per D3's honest classes (ambiguous/vanished with
  the remedy) rather than guessing a list diff; the maps arrive
  with a real case.

  **The rule for shifted roots, DERIVED and registered here (the
  amendment's second clause): the edit path that shifts referenced
  roots EMITS the root map or REFUSES with the remedy.**
  `reference_resolved` / `reference_ambiguous` / `reference_vanished`
  (with `reference_renamed` as the logged success when the map is
  emitted) remain the ONLY resolution classes — a silent positional
  match on shifted roots is precisely the mis-reference D5.4
  demonstrates, and the rule makes it impossible rather than
  unlikely. The boundary of this pin: the RULE and the CLASSES are
  this act's; the operational decision — where the map is emitted
  (the sketch edit path, the rebuild pass), what the refusal's
  payload carries (the dead roots named, the edit class, the
  remedy) — belongs to the resolver's morso (D6 rung 4), where its
  real cost is visible. The roots' identity is authored data; an
  edit that moves it OWES the map or the error, never silence.
- **Vertex names** — no consumer today (the fillet references
  edges); the tavola already carries the vertices' rows, so the
  vocabulary is an extension, not a rework.
- **The polygon path's tavola** — the faceted world stays
  nameless in v1 (no feature references it; the display body
  addresses by node, ADR-0002's split holds). Registered for the
  day a feature consumes a faceted subshape.
- **Patterns, mirrors, rename across long chains** — the
  features do not exist; the composition rule (names reference
  strictly older features) already contains them when they do.
- **The SDF side** — the dual kernel keeps its own addressing
  (node granularity); naming the SDF tree's leaves is a separate
  derivation with its own scale, if a consumer ever appears.

### D5. The discriminating tests — pinned BEFORE the implementation

The ADR-0034 D4 / ADR-0035 D5 protocol verbatim: every claim has
the test that fails on the naive version, agreed before the code.

1. **REGENERATION — same recipe, same names, bit-identical.**
   The digest oracle (the `query_digest` pattern,
   test_g1_unit4_subtract.cpp:310 — every face's full query
   answer as one comparable string) EXTENDS: each face's record
   gains its name, each co-edge's record gains its edge's name.
   Rebuild N times, on both compilers, on both platforms: the
   digest is byte-identical (ADR-0014 — and trivially so: the
   name carries no floating-point content to diverge).
2. **TOPOLOGICAL EDIT — the reference resolves in class 3 or 4
   WITH THE ERROR THAT NAMES.** Two pinned scenarios, both in
   the exact class, both pure recipe edits: (a) the tool's
   circle re-authored as a two-arc chain — a wall reference
   resolves `reference_ambiguous`, the error naming `lat(0)` and
   `lat(1)` and the remedy; (b) the tool's circle replaced by a
   square chain — the rim reference resolves
   `reference_vanished`, the error naming the dead rim, the
   cause (the authored entity was replaced), and the remedy.
   Never a crash, never a silence — the assertions pin the error
   KIND and its payload, not just the failure.
3. **TOPOLOGY-PRESERVING EDIT — the reference survives (class
   1), the feature re-runs without reselection.** The radius
   grows within the class: the rim's name is structural, the
   fillet's input is the same name, the feature re-runs. Pinned
   together with 4 (they are the same scenario run two ways).
4. **THE NAIVE COUNTERPROOF — the positional id fails where the
   name holds.** The scenario that kills the address-as-
   reference, fully in class: the target is the EXTRUDED square
   prism (`ext#2`, chain of 4 — the same convex planar class as
   the box, the subtract treats it identically); the fillet
   references the top rim `sub#4:isect[ext#2:cap(hi) ×
   ext#3:lat(0)]@hi`, whose materialized arc halves sit at
   positional ids 12-15 (E_A = 12). The edit: the square sketch
   becomes a PENTAGON (one chain entity appended — an
   add-at-end root edit, D4's covered flow). The counts move
   exactly as the pinned formula says (V: 8+4 → 10+4, E: 12+5 →
   15+5, F: 6+1 → 7+1, χ = 0 both ways — the general formula
   `V = V_A+2n, E = E_A+3n, F = F_A+n, H = 2` holds for every
   prism, subtract.hpp:84-88); the rim arcs now sit at 15-18;
   **and the entity that today answers to id 14 is the
   pentagon's last vertical edge.** The test asserts BOTH facts:
   the positional id of the referenced rim moved (the naive
   reference now points at a DIFFERENT, wrong entity — the
   silent mis-reference, demonstrated, not asserted), AND the
   name resolved class 1. The naive version of the test —
   reference by id — fails exactly there. That is the
   discrimination: the naming is new, not repainted.

**The golden declaration (pre-registered, before any code):**

- **No existing golden moves.** The addresses stay (D1's split);
  every pinned count, identity, and byte of the numbering tables
  is untouched by construction. A golden that moves is a defect,
  not a re-pin.
- **The name digest is a NEW oracle**, added beside the existing
  ones; the old oracles run unchanged.
- **Determinism from the first landing**: the name digest is
  bit-identical from its first pin — there is no migration, no
  re-pin, because there is no prior name state to move.

### D6. The consequences — format, register, and the order of the morsi

**The format: v1 touches nothing — derived, not deferred.** The
names derive at materialization; the tavola is derived state;
NOTHING serializes in v1 because no v1 feature stores a name
reference (the fillet does not exist yet). The FIRST serialized
name reference arrives with the fillet's recipe (its edge-name
field), and THAT unit owns the format delta — with the road
derived HERE, by the ADR-0027 rejected-alternatives precedent,
verbatim in spirit: NOT unknown-field tolerance (a v0.4 reader
would round-trip a fillet-bearing file DROPPING the reference
silently — data loss through downgrade, the exact violation
ADR-0027 D4 rejected for the recipe fields), but a **MINOR bump
v0.4.0 → v0.5.0 with a registered migrator** (the chain's
fourth step; native.hpp:87-102 carries the machinery unchanged).
Registered here; decided in the fillet's unit, where its real
cost is visible.

**The register surgery — declared; applied in the same act as
acceptance** (the ADR-0035 D6 protocol):

- **ADR-0005 — amended in place, dated, pointing here**: the
  `SubshapeRef` path gains its semantics (indices into the
  operation types' declared role tables — the numbering tables
  made normative, D1); the Phase-A "minimal test" consequence
  (sphere→box union id stability) is superseded by this ADR's
  D5 quartet; the generation clause is re-read at the feature
  granularity the scene already practices, ratified as written
  intent. The ABI itself: UNCHANGED — the amendment fills
  meaning, it does not move a field.
- **ADR-0032 D2 — NOT amended**, and the derivation of why: the
  name field sits BESIDE the Layer-2 fields on the entity, and
  is not one of them — Layer-2 is arithmetic honesty (floats,
  tolerances, provenance); the name is identity (strings,
  integers, purity). ADR-0032's D2 owns the arithmetic record;
  identity is this ADR's own concern, added by this unit's
  morsi citing this document. No mother touched.
- **ADR-0007/0027 — NOT amended now**; the v0.5.0 requirement is
  registered above for the consuming unit.

**The instrumentation order — the morsi 2+ ladder, each landing
standalone with the full suite green** (the house sequence, the
ADR-0032 D10 pattern), STARTING ONLY on the maintainer's review
of THIS document:

1. **The roots** — the role tables made normative + the
   factories' names (box/sphere/cylinder: pure derivation, the
   trivial all-born tavola derived, the smallest possible
   vertical slice).
2. **The extrude** — names + tavola (born of chain entities);
   the digest extension lands here (D5.1's oracle on the
   extrude's own bodies).
3. **The exact subtract** — names + tavola: the guide case's
   full 47-row table (D2) is the golden; the report's table
   gains the name column.
4. **The resolver** — the four classes + the two query surfaces
   (FaceInfo/EdgeInfo carry the name; the UI's face click
   addresses by name).
5. **The D5 quartet** — the discriminating tests, in the order
   derived (1 with 4's first half; 2's two scenarios; 3 with
   4's second half) — the exit criterion of the unit: the
   naive counterproof demonstrated red-on-address,
   green-on-name.

**The fillet's hook, registered**: when the G2 fillet unit
lands, its recipe's edge reference is a NAME (this vocabulary);
its degenerates (ADR-0033 D4) gain the resolution family —
`reference_ambiguous`, `reference_vanished`, with
`reference_resolved`/`reference_renamed` as the silent and the
logged successes — the same error culture, one more province.

### D7. What this unit does NOT do — the no-promises list

- **No code moves in morso 1**: this delivery is this document
  alone. The morsi 2+ implement, only on the maintainer's
  review of THIS document.
- **No change to the address plane**: the positional ids, the
  numbering tables' values, every existing golden and query —
  untouched (D1's split is additive by construction).
- **No Layer-2 change** (D6's derivation: identity is not
  arithmetic honesty); **no serialization change in v1** (the
  v0.5.0 delta belongs to the fillet's unit).
- **No polygon-path tavola, no SDF-side naming, no vertex
  names, no middle-insert root maps, no patterns/mirrors** —
  each REGISTERED in D4's queue, none silently dropped.
- **No fillet**: the feature is G2's; this ADR is its contract
  floor, derived before the consumer exists so the consumer
  cannot bend it.

## Consequences

- A downstream feature can reference a boolean-generated subshape
  and SURVIVE the edit — the parametric CAD problem this repo
  had not yet answered is answered as a derivation, not a
  heuristic: no signature matching (ADR-0005's own correction,
  upheld), no tie-break guessing (ADR-0035's bands stay where
  they belong), no silent re-picking (the scene's dangling
  discipline, extended to subshapes).
- The descent information stops being computed-and-discarded
  (D2's ClippedFace lesson): every operation's tavola is the
  recording of facts its algorithm already holds, at linear
  bookkeeping cost, with a closure invariant that catches its
  own defects.
- The four resolutions give the edit path a vocabulary the UI
  can render and the user can act on — the "grow the radius and
  the model re-shapes" moment becomes a NAMED outcome, not a
  mystery.
- The G2 fillet unit inherits its reference contract read-made:
  the rim's name is pinned here, its grammar is the SSI table's
  own, and its failure modes are already in the error taxonomy.
- ADR-0005's promise is redeemed eight ADRs later by filling the
  ABI it shipped: the role tables give `SubshapeRef` its
  semantics, the tavole give the genealogy its record, and the
  Phase-A field that was never once populated starts meaning
  something — the house way, derived, not decorated.

## References

- ADR-0005 (the promise — genealogy, never signature; the ABI
  redeemed here), ADR-0002 (the dual kernel — the recipe
  operates both materializations; the SDF side stays
  node-addressed), ADR-0007/0007a (the versioning semantics the
  v0.5.0 delta will ride), ADR-0014 (determinism — the name
  inherits the bar trivially: no floating-point content),
  ADR-0021 (the entity set and ID scheme — the address plane,
  untouched), ADR-0027 D2-D4 (the recipe doctrine, the dangling
  protocol, the rejected-alternatives precedent for the format
  delta), ADR-0030 (the sketch recipes — the authored roots),
  ADR-0031 D5 (the phase ladder and its one-guide-case method),
  ADR-0032 D2/D3/D7 (the data model, the Layer contract, the
  guide case's pinned counts), ADR-0033 D3/D4 (the fillet
  classes and degenerates — v1's first consumer), ADR-0035 (the
  derivation method this ADR follows, and the register-surgery
  protocol its D6 lends).
- The code sites, verified at 6525fa0: boolean.cpp:1262-1265
  (ClippedFace — the origin discarded), :1281-1443 (assemble_body
  — the counters: :1320, :1338, :1344, :1425), :2050-2075 (the
  clip loop that knows the origin); topology.hpp:35-38 (the bare
  uint64 ids — the address plane); shape_id.hpp (the unpopulated
  ABI), shape.cpp:60 (its only reader); scene.hpp:57-61 (the
  recipe doctrine in the header), :213/:526 (the node ids),
  :271-274 (the feature-granularity dangling rule),
  :286-296 (the HybridBody twin — the tavola's lodging
  precedent); extrude.hpp:73-93 and subtract.hpp:69, :84-88,
  :113-139, :1128-1129 (the flush-cap dedup, the general
  formula, the numbering tables, the verbatim copy); factory.hpp:
  126-131 (make_box's construction roles); query.hpp:119-121
  (the positional query surface); test_g1_unit4_subtract.cpp:
  310 (the digest oracle D5.1 extends).
- Andrade, M. V. A. (1998). *Topological Naming in Solid
  Modeling*. PhD thesis, University of Maryland — the cell-tuple
  genealogy line ADR-0005 already cites; this ADR's tavola is
  the operation-level record that line implies.
- Chen, X., & Luo, Z. (2010). *Research on Persistent
  Identification of Topological Entities in Parametric CAD
  Systems*. ASME IDETC — the split/merge/absorb vocabulary's
  literature anchor, as cited by ADR-0005.
