# ADR-0031: The exact B-Rep kernel — the great fork: build, adopt, or hybrid

## Status

**Accepted (2026-10-25) — road (a): all in house, forever.** The
fork this document presented is closed at acceptance: no OCCT,
no third-party CAD kernel, ever. The G2 gate survives as an
internal checkpoint only (estimate vs. progress), not as a fork —
**and that checkpoint is now recorded: § "The G1 internal
checkpoint (2026-10-26)" below (G1 measured against D1; the
estimates stand).** The decision, the license constraint pinned
into ADR-0003 in the same act, and the user's motivation verbatim
are recorded in § "Decision on acceptance" below. D1 stands as the
honest price list that was weighed; D2's proposal was (c1) with the
gate armed — the user chose (a), which D2 explicitly named as
consistent with this document.

Drafted 2026-10-24, reviewed and decided by the user 2026-10-25.
No implementation ships from this document; the first daughter
(ADR-0032, G1) is drafted and enters the user's review before
any phase starts — the house method: invariants and
discriminating tests pinned **before** the phase begins.

What this ADR is to the roadmap: the master decision for **Phase G**
(the "B-Rep advanced" line the phase plan has carried since the
overview document — fillet, chamfer, shell, NURBS booleans), split
here into the ladder G1 / G2 / G3; the fork gate the draft placed
between G1 and G2 is closed by the decision — the gate position
remains, as a checkpoint.

## Decision on acceptance (2026-10-25)

The user's decision, on review of the draft:

1. **Strada (a): TUTTO in casa, per sempre.** The fork-gate of
   G2 is closed at the start: no OCCT, no third-party kernel
   adoption, ever. The G2 gate remains only as an internal
   checkpoint (stima vs avanzamento), not as a fork.
2. **The license constraint**, pinned in ADR-0003 as an amendment
   in the same act: third parties only under licenses without
   usage limits (public domain, MIT, BSD, Apache-2.0, zlib,
   ISC, …). GPL, LGPL, and licenses with exceptions: out.
3. **The motivation, recorded verbatim (the user's words):**

   > indipendenza completa del progetto; andatura lenta oggi,
   > più rapida in prospettiva; i costi di (a) — G2 in casa =
   > anni, non mesi — accettati consapevolmente, con i modi di
   > fallimento di D1 e le contromisure della scala (ogni fase
   > consegnabile da sola, valore visibile a ogni gradino).

   Complete independence of the project; a slow pace today, a
   faster one in perspective; the costs of road (a) — G2 in
   house means years, not months — consciously accepted,
   together with D1's failure modes and the ladder's
   countermeasures (every phase deliverable on its own, visible
   value at every step).

What the decision changes in this document: D2 records the
outcome (its proposal stood; the user chose the pure-sovereignty
outcome it named); D5's G2 entry criterion is amended in place
(fork re-decision → internal checkpoint); the Consequences
reflect the accepted state. What it does not change: D1's price
list and failure modes (the record of what was weighed — the
countermeasure the motivation itself cites), D3/D4
(road-independent by design, now binding on the house road), D6
(the migration and the SDF boundary hold unchanged). A companion
orientation document — the platform vision the decision serves —
is recorded in `docs/architecture/platform-vision.md`.

## The G1 internal checkpoint (2026-10-26) — measured against D1

The gate this ADR kept after the fork closed — "G1's measured
effort and robustness data against this ADR's estimates,
re-baselining the G2 plan — never re-opening the build-vs-adopt
question" — is discharged here, the day after G1's click-test
closed the phase. The checkpoint is the comparison, not a
celebration: every number below is the register's own (the commit
history and ADR-0032's landing record), nothing embellished, and
the open items are named, not footnoted away.

**What D1 estimated, at the granularity D1 spoke.** G1: "house-scale
(several delivery units)" — the class of work Phase D+E+F
exemplified (roughly fourteen ADR units across 2026-10-13 →
10-24). G2: "years, not months — order 1–3+ years of sustained
effort for an honest narrow version (exact where the pair table
reaches, declared-tolerance subdivision elsewhere, degeneracies
rejected with actionable errors rather than handled)", unbounded
for NX-class robustness. D1 did not price the individual delivery
units — no per-unit estimate exists to divide by, and inventing
per-unit ratios now would be exactly the infioritura this section
forbids. The measurable granularities are the phase, the session,
and the unit count.

**What G1 measured (the register, ADR-0032):**

- **Units: six of six**, as the daughter's D10 pinned them — one
  per rung of the order's arrow (data model → primitives →
  extrude → subtract → mass → report), each landed standalone with
  the full local suite green and visible value on delivery. Plus
  one post-landing fix commit and the closure docs (the ADR-0032
  status flip, this checkpoint, and the G2 draft).
- **Sessions: three**, 2026-10-24 → 10-26 (the register's dates:
  the acceptance evening carrying the review, unit 1 and the CI
  fix; the 10-25 session carrying units 2–5; the 10-26 session
  carrying unit 6). Six units in three sessions, from the data
  model to the printed table.
- **Revisions needed after landing: one.** The push of unit 1 went
  red on exactly one CI job (linux-clang, build step: doctest's
  CAPTURE lambda vs structured bindings — a clang ≤ 15 limitation
  gcc 11 accepts); the fix (2e4d4db) root-caused it, reproduced
  the CI pairing locally so that column is never dark again, and
  registered the one inert leftover it met on the way (CLOSED
  2026-10-26, the maintainer's one-word decision taken in the
  ADR-0033 acceptance: CHECK — the unread `rejections` counter in
  test_bridge.cpp now enforces the header's own recorded sentence,
  "0 rejections needed — the guard is a guard, not a filter", as a
  contract; the reasoning is in that commit's body). No other
  unit needed a second landing; no unit was re-delivered.
- **Defects found by the in-run verification, by class:** two
  convention defects caught by the units' own first-run tests and
  fixed in-unit (the cylinder's left-handed parametrization, unit
  2; the surviving wall's region-side walk, unit 4 — the
  tests-first discipline absorbing them BEFORE landing: the
  countermeasure working, not the process failing); one
  portability defect caught by CI after landing (the clang-14
  case above, closed same-day). **Defects found by the user's
  click-test: none** — the exit validation (the D7.5 table in the
  report, the volume verified by hand, the live radius edit)
  passed clean. **The disappointed-vs-promises class is empty:**
  every promise of ADR-0032's D1 landed; every declared exclusion
  stayed excluded and refused actionably (the oblique SSI, the
  v0.3 golden's faceted honesty, trimmed spheres deferred); the
  boundaries G1 discovered on the way (the booleans-consume-
  exactness gate; the exactly-convex class of the faceted
  assembly; the few-ulp precision doctrine of the stored trimming
  floats) were derived, pinned and declared in the units' records
  — none reached the user as a surprise, and none was softened
  after the fact.
- **Robustness data:** the degenerate dispositions of the daughter
  D7 are golden-pinned or refusal-pinned (coplanar absorbed,
  tangential tie-broken by canonical key, crossing/blind/swallow
  refused with remedies, the miss the exact copy); determinism is
  proven per ADR-0014 (bit-identical edit cycles on BOTH
  materializations of the hybrid, hex-digest pinned; the v0.4
  golden re-loads and re-materializes identically); the mass
  precision bound is pinned (~1e-6 relative — the stored-float
  inputs doctrine, five orders tighter than any faceted extract).

**The comparison, and the honest consequence for G2's pricing.**
G1's measured shape — six units in three sessions, one escaped
defect (CI-caught, closed same day), zero user-visible defects,
the disappointed class empty — **confirms D1's G1 estimate at its
own granularity** ("house-scale, several delivery units") and
nothing more. It does not touch the G2 estimate, and the
checkpoint says so plainly: **G1 never met the cliff.** One SSI
pair was intersected (plane∩cylinder, the two typed orientations);
no tangential contact beyond a pinned disposition, no vertex
blends, no non-parallel cylinder pair, no quartic of any kind, no
trimmed sphere. The tail that makes G2 "years, not months" is
exactly the part G1 did not exercise — so the estimate stands,
un-rebaselined: **"order 1–3+ years of sustained effort for an
honest narrow version" remains the working number**, now with one
measured anchor under it: the bounded class runs at the house
cadence (two units per session at G1's size) with the discipline
absorbing the discovery rate (three product-relevant findings in
six units, none escaping to the user). What the experience DOES
update is not the price but the shape of the first bites: the G2
draft (ADR-0033, shipped with this checkpoint, in the user's
review) derives the analytic table's honest boundary — the pairs
of degree > 2 are not typable in the pinned vocabulary; the
Viviani-class quartics and the spiric sections stay out — and puts
to the user, as an explicit reserved decision, the very fork D1's
own wording embedded in "honest narrow version": exact-where-the-
table-reaches with refusal elsewhere (option i), or plus
declared-tolerance subdivision (option ii). The elephant's next
bite is on the table; its total weight is not re-estimated here,
and no promise on the tail is made — by D6's rule and by this
checkpoint's own.

## Context — position, stated honestly

The 3D of today is an implicit SDF evaluated on demand, displayed as
a marching-cubes mesh (MC33, ADR-0015/0016), ray-marched in the
viewport, and persisted as **recipes** (ADR-0027 D4: the recipe is
the source of truth, the body is a cache). The sketcher MVP
(ADR-0030, implemented) closed the loop: draw → extrude → subtract →
edit a radius → the chain regenerates deterministically.

What that solid does **not** carry is exact faces and edges as its
system of reference. Concretely, today:

- The **SDF side is exact as a field** — primitives, CSG composition,
  the 2D profile semantics of ADR-0030 (sign and zero-set exact,
  Lipschitz 1 proved) — but its boundary is an *implicit* zero set.
  The mesh is a sample of it, correct to the grid, never the thing
  itself.
- The **B-Rep side is exact only where it is planar**: radial-edge
  topology (ADR-0021/0022), seam-wired analytic primitives
  (ADR-0023), and boolean operations restricted to convex polyhedra
  via polygon clipping (ADR-0024/0025). A cylinder can exist as an
  exact face; two cylinders cannot be intersected. There is no
  surface-surface intersection of any kind in the tree.
- The **bridge** (ADR-0026) extracts an SDF into either the exact
  analytic primitive (when the SDF *is* one) or a faceted body —
  one triangle, one planar face. The boolean results of the
  sketcher flow (a subtracted hole) therefore extract **faceted**:
  the hole's wall arrives as a prism of planar strips, not as the
  cylinder it mathematically is.
- **Mass properties do not exist at all** — there is no volume, no
  centroid, no inertia API anywhere in the codebase (kernel,
  bindings, or app). The one number users quote first is absent.
- **STEP is planned, not built** (ADR-0010 scoped AP203 ed. 2,
  analytic-only) — and it cannot be built honestly on faceted
  geometry: `ADVANCED_FACE` with `CYLINDRICAL_SURFACE` means the
  cylinder, or it means nothing.

Everything that competes with the NX class of system — robust
constant-radius fillets, exact dimensions, dimensioned drawings,
exact mass properties, STEP as an exchange of exact geometry —
requires the exact B-Rep to be the **source of truth of the solid**.
Not a report generated from a mesh; the solid itself, carrying its
faces and edges as first-class exact objects, with everything else
(display included) derived from it under a declared tolerance.

That is the fork this ADR puts on the table. It is a fork, not a
step, because the two candidate futures have different owners of
exactness: one where this repository owns it (and pays for it), and
one where an industrial kernel owns it (and we pay a different
price, in governance and integration, not in geometry). The honest
form of this document is to price both, name the failure modes of
both, and derive a proposal — with the decision reserved.

### What the fork is NOT

The fork is not "SDF or B-Rep". ADR-0002 settled that: dual
representation, peers, no silent conversion. The SDF side has
delivered — and keeps — the jobs it is structurally better at:
preview and display (ray-marching, MC33), organic modeling
(`smooth_min` blends under the ADR-0028 bounds contract, the twist
warp under ADR-0029), and field evaluation/sampling as a first-class
query. **What works is not killed on principle; it is positioned.**
The fork is about who owns *exactness* — and every road below pins
the same SDF boundary explicitly, so the positioning is part of the
decision, not collateral of it.

## Decision

This ADR records five decisions of structure (D3–D6: the data
model, the hard-problem strategies, the ladder, the migration) that
hold on **every** road, and one fork presentation (D1: the three
roads with honest costs) plus one derivation output (D2: the
proposal — labeled as such, with the reasoning visible, and the
final word explicitly the user's).

### D1. The three roads — costs and failure modes, no implicit preference

Symmetric treatment: what each road buys, what it costs, how it
fails. Nothing in this section ranks the roads; ranking is D2's
job, and D2's output is a proposal, not a decision.

#### Road (a) — build the exact B-Rep kernel in house

**What it buys.**

- **Sovereignty over exactness.** Every geometric decision stays in
  this repository, testable deterministically (ADR-0014 culture:
  bit-identical re-runs, byte-identical goldens), auditable line by
  line. This is the stated long-term goal of the project (README:
  "built without any third-party CAD kernel", ADR-0003) — road (a)
  is the only road that keeps that sentence true without edits.
- **The house has already proven it can do exact.** Not as a hope —
  as delivered artifacts: the 2D sketch kernel (typed entities with
  construction-time validation, the max-composition field semantics
  derived to the ADR-0029 standard), **[Correction on the record
  (H1, 2026-10-29, ADR-0034): the predicates were NOT this. What sat
  in `third_party/robust_predicates/` was a hand-written
  non-adaptive stub imitating Shewchuk's structure — the register
  here claimed a vendoring that had not happened (the license audit
  of 2026-10-16 was the one record that said the truth). The
  authentic `robust.c` is vendored by H1's feat commit, with the
  discriminating adaptivity tests as the proof.]** the radial-edge
  topology with its integer invariants (χ exact, closure
  structural), the seam-wired analytic primitives, and the convex
  planar booleans whose assembly pipeline (canonical vertex keys,
  order-independent dedup) is the proven pattern for turning
  computed geometry into valid closed shells.
- **No governance change.** ADR-0003 stands unamended, the NOTICE
  gains nothing, the license surface of the distribution does not
  move. The fork costs zero in process.
- **Fit.** A kernel built by this house carries this house's
  tolerance discipline (ADR-0004: no global epsilon, typed
  tolerances, explicit on every operation) from day one, instead of
  importing a foreign tolerance semantics and adapting to it.

**What it costs — quantified from first principles, anchors cited.**

The work decomposes into layers with very different risk profiles:

| Layer | Status | Effort class |
|---|---|---|
| Topology (entities, radial-edge, invariants) | **exists** (ADR-0021) | incremental: p-curves on loops |
| Analytic surfaces/curves (plane, cylinder, sphere, cone, torus; line, circle, ellipse) | **exists as classes** (no trimming) | incremental: trimmed-domain machinery |
| Exact mass properties (volume/centroid/inertia from the B-Rep) | absent | **bounded**: divergence-theorem surface integrals over trimmed analytic faces (Mirtich 1996 for the polyhedral baseline; per-face quadrature with closed forms for the analytic families) |
| Analytic-pair SSI (the closed-form intersection table) | absent | bounded-but-wide: each pair is derivable (plane∩cylinder → ellipses/circles; plane∩sphere → circle; plane∩cone → the conic sections; coaxial pairs → circles), the table is finite |
| General SSI + robust booleans on curved faces | absent | **the cliff** — see below |
| Constant-radius fillets incl. vertex blends | absent | **the cliff's worst face** |
| NURBS layer (G3) | absent | large, deferrable |
| STEP writer (AP203 analytic-only) | absent, scoped (ADR-0010: ~3 kLOC estimate) | bounded once geometry exists |

"Bounded" here means: house-scale, comparable to a delivered phase.
Phase D + E + F together landed as roughly fourteen ADR units across
the sessions of 2026-10-13 → 10-24. The bounded rows above are that
class of work — several delivery units, each with pinned invariants
and discriminating tests, per the house method.

**The cliff, quantified honestly.** General surface-surface
intersection is the cemetery of amateur kernels — the phrase is the
industry's, and it earns it. The failure mechanism is structural,
not a matter of skill:

1. **The degenerate tail dominates.** The textbook configurations
   (two cylinders crossing at a sane angle) yield to the analytic
   table plus subdivision in weeks. The robustness problem is the
   *asymptote*: tangential contact, near-coincident faces, sliver
   loops, singular points on seams, configurations where the correct
   output topology itself is unstable under epsilon perturbation of
   the input. Hoffmann (1989) and Fortune's robustness survey both
   make the same point: near degeneracy, geometric computation
   amplifies input error; robustness is not a bug-fixing activity
   but a redesign of the arithmetic (predicates, tolerances,
   tie-breaks) — which is what the vendored predicates and ADR-0004
   are for, but the *space* of degeneracies is open-ended.
2. **The industrial anchors are not encouraging for timelines.**
   OCCT's lineage runs from Matra Datavision's Euclid-era work of
   the early 1990s through 27 years of public development since
   1999 (Wikipedia; the release history) — and its 2026 maintenance
   releases (7.9.2, 7.9.3) still headline *modeling reliability,
   shape healing, concurrent fillet construction* as the work items.
   Twenty-seven years in, the industrial kernel is still shipping
   boolean/fillet robustness fixes. Parasolid and ACIS, the
   commercial anchors, are decades of dedicated teams. This is the
   true price of "industrial robustness": it is asymptotic.
3. **The 90/10 inversion.** A house-built G2 will pass its own test
   corpus in months — and then users will find the tail. Each
   degenerate family discovered in the field spawns a fix cycle,
   and trust erodes exactly where CAD trust matters most (the
   fillet that fails on the user's part, not the demo part).

First-principles estimate, labeled as such: **G1 is house-scale
(several delivery units)**; **G2 in house is years, not months** —
order 1–3+ years of sustained effort for an *honest narrow* version
(exact where the pair table reaches, declared-tolerance subdivision
elsewhere, degeneracies rejected with actionable errors rather than
handled), and **unbounded** for NX-class robustness. G3 (NURBS,
free surfaces) is a further block of the same order as G2.

**Failure modes, named.**

- *The perpetual prototype.* The kernel is forever "one degenerate
  family away" from the features that justified it; fillets and
  STEP stay 'planned' while the tail is chased.
- *Tolerance semantics creep.* Under deadline pressure, ad-hoc
  epsilons appear inside SSI code paths; ADR-0004's no-global-
  epsilon discipline erodes exactly where it is hardest to honor.
- *Opportunity cost.* Every session spent on the tail is a session
  not spent on the parametric modeling, drawings, and UX layers
  that the SDF+recipe spine already supports.

#### Road (b) — adopt OCCT (Open CASCADE Technology)

**What it is.** OCCT is the mature open-source industrial B-Rep
platform: curves and surfaces, booleans (BOPAlgo), fillets and
chamfers, shelling, shape healing, meshing, STEP and IGES exchange,
and the OCAF application-data framework — C++, LGPL, with a
27-year public development record (open-sourced 1999 out of the
Matra Datavision CAS.CADE line; stable release 8.0.1, 2026-07-30;
7.9.x maintenance line active in parallel). It is the engine under
FreeCAD and a long list of commercial products. Its scale is on the
order of a million lines of C++ across tens of link libraries
(public-repo estimate), with module-level integration an explicit
design goal of the platform (their own positioning: "integrate at
toolkit level, keep your own product architecture").

**What it buys.**

- **The cliff, pre-paid.** General SSI booleans, fillets (including
  the hard vertex blends), STEP round-tripping with decades of
  interoperability corpus behind it, shape healing for imported
  garbage — the exact layers where road (a) spends its years,
  available behind a C++ API today.
- **A maintenance ecosystem.** Robustness work continues upstream
  whether or not this project ships any (see the 7.9.x headlines);
  the tail is chased by people whose day job is the tail.
- **Honesty about the tail.** The failure modes of OCCT's boolean
  stack are *documented, known, and versioned* — a different risk
  class from discovering them ourselves.

**What it costs — the license gate (the parked one, paid here).**

ADR-0013's method applies to any adoption: license analysis,
NOTICE entry, per-component flags in the audit. The facts, verified
against the SPDX record and the upstream licensing texts:

- OCCT ≥ 6.7.0 ships under **LGPL-2.1-only with the OCCT exception
  1.0** (SPDX: `LGPL-2.1-only WITH OCCT-exception-1.0`). The
  exception permits distributing object code that incorporates
  material from OCCT header files under terms of the distributor's
  choice, provided prominent notice is given. Plain LGPL-2.1
  obligations otherwise apply: source (or a written offer) for the
  library itself, source for any *modifications to OCCT*, relinking
  rights for users of the combined work.
- Mapped onto CAD_0's Apache-2.0: **dynamic linking of an unmodified
  OCCT is the clean path** — the combined work may stay Apache-2.0;
  the distribution ships OCCT's license and notice texts, the NOTICE
  file gains the OCCT entry with its version pin, and the audit
  gains the per-component flags (the same treatment PySide6/Qt
  already has, one order of magnitude heavier in surface area).
  ADR-0013's own forbidden list already anticipates the shape:
  static linking of LGPL-2.1+ libraries without shipping object
  files is banned — so a static-link build would have to publish
  object files or stay off the table.
- The gate is **mechanical once the decision is taken** (NOTICE
  entry, audit row, build wiring) and **not improvisable before**:
  the decision to link an LGPL kernel into an Apache-2.0
  distribution is the maintainer's, and this ADR does not take it.

**What it costs — governance and culture.**

- **ADR-0003 is violated as written.** It forbids, verbatim, "any
  kernel that ships a full B-Rep / NURBS / boolean stack" on the
  stated ground that such a kernel "would replace, not augment, the
  native kernel" and "compromise the dual-representation design
  (ADR-0002)". OCCT is squarely that kernel. Road (b) therefore
  *requires superseding or amending ADR-0003* — a governance act by
  the user, not a footnote in this document. The README's stated
  long-term goal changes meaning in the same stroke. The honest
  counter-argument ADR-0003 itself invites: the replacement only
  happens if we let OCCT take the SDF side too; a disciplined
  boundary (D6 below) keeps the dual design intact with OCCT as the
  exact-geometry engine. Whether that argument carries the amendment
  is exactly the kind of decision that belongs to the user, with
  this ADR as the briefing.
- **The determinism culture collides.** OCCT does not guarantee
  bit-identical results across platforms or versions — its
  algorithms are tolerance-driven and some paths are parallel. The
  house contract (ADR-0014: bit-identical round-trips,
  byte-identical goldens) must be **scoped to our serialization
  boundary**: results quantized to 12 decimals before writing (the
  mechanism ADR-0014 already mandates), goldens pinned per OCCT
  version, and cross-version byte-identity explicitly *not*
  promised for OCCT-sourced geometry. This is a real narrowing of a
  house invariant, and it must be said out loud, not absorbed
  silently.
- **The tolerance model collides.** OCCT carries per-subshape
  tolerances (`BRep_Tool::Tolerance` on vertices/edges/faces) that
  *are* its robustness currency; ADR-0004 is operation-tolerance
  based with no global epsilon. The adapter must map one into the
  other with a written rule (D3's layering is designed to receive
  it), or every boolean result arrives with tolerance semantics we
  did not choose.
- **Integration weight.** Order-of-a-million-LOC dependency: build
  time, CI matrix cost (three compilers × ASan × that dependency;
  the Codespaces image), disk footprint, and an adapter layer
  between `TopoDS` handles and the house topology — or the
  alternative fork-within-the-fork of adopting `TopoDS` types
  natively, which deletes the ADR-0021 investment and with it the
  kernel's own topological spine. Both sub-roads must be priced in
  the G2 gate; neither is free.
- **Upstream dependency risk.** Fix latency and priority alignment
  live at Open Cascade, not here — the structural cost of any
  foreign engine. Version pinning becomes a house discipline
  (security and behavior alike).

**Failure modes, named.**

- *The hollow kernel.* OCCT absorbs the exact side so thoroughly
  that the native B-Rep module rots; ADR-0002's "peers" becomes a
  README sentence, the SDF side a viewport trick. (The disciplined
  boundary in D6 is the countermeasure, and its enforcement is a
  permanent tax on discipline.)
- *The seam.* The adapter between foreign tolerance semantics,
  foreign IDs, and the recipe DAG becomes the new graveyard — every
  naming bug (ADR-0005's whole reason to exist) now has two
  genesis sites.
- *The culture drift.* "Not invented here" inverts into "not
  debugged here": an order-of-magnitude heavier dependency trains
  the house to treat kernel behavior as weather, not engineering.

#### Road (c) — hybrid, split by phase

The boundary question is not "which kernel" but **which layer of
exactness is bounded and which is asymptotic**:

- **Bounded exactness** — where the closed form exists, the
  degeneracy space is finite, and the house has proven patterns:
  primitives and their trimming, planar and cylindrical /
  cone-family intersections reachable from the analytic table,
  exact mass properties, the sketch→prism→hole chain of the
  existing product spine.
- **Asymptotic exactness** — where robustness is a tail without
  end: general SSI on arbitrary curved pairs, vertex blends, STEP
  interop against the world's corpus of slightly-broken files.

The hybrid roads put the boundary at different places:

- **(c1) house-G1 + engine-G2**: the house kernel owns the bounded
  layer (G1 as in road (a)); OCCT enters at the G2 gate as the
  SSI/boolean/fillet/STEP engine behind a narrow adapter, with the
  recipe DAG and naming (ADR-0005) staying house-native — OCCT as
  an *engine*, never as the document's owner (OCAF is not adopted;
  the recipe DAG is the history system).
- **(c2) OCCT-exact-now**: adopt OCCT for the whole exact side
  immediately, keep SDF for display/organic — road (b) with the
  boundary drawn at the representation level instead of the phase
  level.
- **(c3) house-forever**: road (a), with the hybrid label marking
  that the *SDF* side is permanent regardless — the name reminding
  that every road is hybrid about SDF.

**What it buys.** Each side pays only for its own class of problem:
the house pays bounded costs where it has patterns and provenance,
and buys (or defers) the asymptotic part only when the demand is
real. **What it costs.** Both worlds' fixed costs — the adapter
seam, the tolerance mapping, the license gate (if OCCT is in scope
at any phase, the ADR-0003 question is live at that phase), plus
the permanent discipline of holding a boundary that both sides'
gravity will erode. **How it fails.** The seam (see road (b)); or
the boundary drifting until (c1) silently becomes (c2), with the
native layer demoted to a compatibility shim nobody funds.

### D2. What the derivation proposes — decision reserved to the user

The reasoning, exposed so it can be attacked:

1. The G1 deliverables (exact primitives with real trimming, exact
   mass properties, the parametric hole with true faces) are
   **bounded work on the existing spine** — they reuse the
   topology, the predicates, the 2D kernel, and the recipe DAG as
   they are, and they produce user-visible value (the missing
   volume/centroid numbers; the hole that is *actually* a cylinder)
   on the product's existing spine.
2. Nothing in G1 forecloses any road: the data model (D3) is
   designed to receive an OCCT adapter later; the tolerance
   layering is the one both engines can populate.
3. The G2 fork is where the cost curve breaks (D1's cliff), and it
   is also where the decision's inputs change character: after G1,
   the house owns a corpus of exact bodies, a measured analytic-SSI
   subset, and a real adapter-surface prototype — the fork becomes
   an *empirical* decision instead of a philosophical one.
4. Therefore: **the proposal is (c1) with the fork gate armed** —
   run G1 in house now; take the build-vs-adopt decision at the G2
   entry gate, with G1's data on the table and this ADR's D1 as the
   price list.

This is a proposal, not a decision: the user may rightly prefer
(c2) (buy the cliff now, stop paying bounded costs twice), or (a)
pure (the sovereignty argument is the project's founding sentence),
or reject the ladder's shape entirely. All four outcomes are
consistent with this document; the review exists to pick one.

**Outcome (2026-10-25):** the user chose road (a) — pure, the gate
closed at the start. Of the four outcomes this section declared
consistent with the document, the founding-sentence one: complete
independence, the costs of the cliff accepted consciously. The
derivation above stands as the record of what was weighed; the
G2 gate it proposed to arm is disarmed to an internal checkpoint
by the decision (D5's entry criterion, amended in place).

### D3. The data model — pinned, road-independent

This section is the substance the user's order pins: it holds on
road (a), (b), and (c) alike, because it describes what an exact
B-Rep *is* in this house, not who computes it.

#### Topology: the ADR-0021 spine, ratified and extended

`Vertex / Edge / CoEdge / Loop / Face / Shell / Solid` with the
radial-edge structure (Weiler; ADR-0012/0021) stays the topological
spine — the entity set, the ID scheme, the ownership rules, and
`link_radial_edges` as the single radial-population point are
ratified unchanged. The extension the exact kernel adds is one
field, and it is structural:

- **A `CoEdge` carries a p-curve**: a reference to a 2D curve in
  the parameter domain of the owning `Face`'s surface, alongside
  the existing reference to the 3D `Edge`. The pair (3D space
  curve, 2D p-curve) is the defining structure of an exact B-Rep —
  a loop is not a polygon of 3D points but a **trimming loop** of
  p-curves on the surface. The `LineCurve`/`CircleCurve`/
  `EllipseCurve` set in `curve.hpp` gains its 2D mirror (the 2D
  kernel of ADR-0030 already manipulates exactly these typed
  entities — segments, arcs, circles — with construction-time
  validation; that machinery is the reuse, not a rewrite).

#### Geometry: analytic families now, NURBS gated

Surfaces: plane, cylinder, sphere, cone, torus — the five families
already in `analytic.hpp` — now with **trimmed-domain machinery**
(the untrimmed surface, the trimming loops, and the face *is* the
region between them). NURBS surfaces and the general intersection
curve (a spline) enter only at G3 with their own daughter ADR; the
data model reserves the slot (the `SurfaceGeometry` hierarchy is
open) without implementing it.

#### The tolerance model — the #1 problem, answered in layers

Where does exactness end, and what decides? The answer is a
three-layer discipline, because "tolerance" means three different
things that must never be conflated:

- **Layer 1 — exact, integer, tolerance-free.** Topological
  incidence, loop closure, orientation, and the Euler characteristic
  are integer facts. They are decided by integer arithmetic and
  asserted as hard postconditions, in every test, on every body the
  kernel produces. No epsilon touches this layer — ever.
- **Layer 2 — declared per-subshape.** Every vertex, edge, and face
  carries a linear tolerance field: the bound on the deviation
  between the *stored* geometry and the *nominal* geometry. It is
  set by the operation that created the subshape: factories write
  zero (analytic, exact), intersection-writing operations write
  their declared tolerance, an OCCT adapter maps it from
  `BRep_Tool::Tolerance`. This is the field OCCT-style robustness
  and ADR-0004 discipline meet on — the layering is what lets both
  engines populate the same structure.
- **Layer 3 — decision tolerances, explicit per operation.** Every
  classification (inside / outside / on; merge / split; keep /
  drop) receives a `Tolerance` argument per ADR-0004 — no defaults
  at the C++ signature, no global epsilon — and the operation must
  be passed a tolerance at least the maximum Layer-2 tolerance of
  the subshapes it touches. When a decision falls inside the band,
  the tie-break is **deterministic and logged** (the canonical-key
  pattern of ADR-0016/0025: integer-keyed ordering, never
  input-order dependence) — never a silent coin flip on a float
  comparison.

What is guaranteed, and how it is asserted:

| Claim | Status | Assertion |
|---|---|---|
| Incidence and closure are exact | guaranteed | Layer-1 integer postconditions, hard-fail |
| Euler-Poincaré holds on every closed body | guaranteed | χ as an integer identity — see below |
| Stored geometry within declared tolerance of nominal | guaranteed | Layer-2 field, reported per body |
| Intersection points are the mathematical points | **not guaranteed** | they are within the operation tolerance; provenance declares it (D4.1) |
| Bit-identical results across platforms | scoped | our arithmetic: yes (ADR-0014); serialization: yes, quantized; OCCT-sourced geometry: only at the serialized layer |

#### Euler-Poincaré — the closure invariant, pinned as contract

The house already owns the closure invariant; it has a UI and a
derivation discipline. The Debug-menu topology report (the bridge
`SdfToBrepReport` of ADR-0026, amended by ADR-0027 D6: structural
invariants hard, χ and genus **reported** and derived only where
sound — `genus_from_chi` with its even/≤2 single-shell rules and
the multi-shell n/d honesty) becomes **the invariant of the B-Rep
kernel itself**, promoted from bridge consumer to kernel contract:

- Every closed body the kernel produces must satisfy the structural
  invariants (every edge exactly two co-edges with orientation XOR;
  every loop closed; every face one outer loop; orientable), and
- its χ must be consistent with its shell/genus bookkeeping under
  the generalized Euler-Poincaré relation `V − E + F = 2(S − G)`
  (Mäntylä 1988; single-shell case `χ = 2 − 2G`, the formula
  `genus_from_chi` already implements).

The invariant is integer, so it is exact where everything geometric
is approximate — which is precisely why it is the right closure
contract for a tolerance-carrying kernel: **topology is the truth,
geometry carries its declared error**, and the report says both.

### D4. The hard problems — named, with a strategy each

#### D4.1 Surface-surface intersection

*Why it is the hard core:* the output is a curve whose topology
(number of components, closed vs open, singularity structure) is
unstable near degeneracy, and every downstream consumer (booleans,
fillets, trimming) inherits that instability (Hoffmann 1989;
Patrikalakis & Maekawa 2002 catalogue the singularity taxonomy;
Boender 1998 surveys the algorithm families — marching, lattice,
subdivision — and their failure cases).

*Strategy:*

1. **Exact where the pair admits a closed form.** An analytic-pair
   table, each entry a derivation, not a lookup of folklore:
   plane∩plane → line; plane∩cylinder → ellipse / circle pair /
   tangent lines; plane∩sphere → circle; plane∩cone → the conic
   sections (ellipse, parabola, hyperbola — the classical theory);
   plane∩torus and the coaxial/symmetric pairs → the reachable
   closed forms. The G1 guide case needs exactly one entry:
   plane∩cylinder (the hole's wall against the box's faces —
   circles on the two cap planes, nothing else).
2. **Robust classification, reused.** **[Correction on the record
   (H1, 2026-10-29, ADR-0034): at the time this ADR was accepted the
   "vendored" predicates were the Phase A stub — non-adaptive, naive
   determinants in double — and no exact-path code called them
   anyway; the strategy below described the INTENT, not the tree.
   H1 lands the authentic adaptive `robust.c` (the orient2d/3d,
   incircle sign now actually robust at ulp scale, proven by the
   discriminating tests) and ADR-0034 pins the double arithmetic
   boundary the classification runs in.]** The Shewchuk
   predicates (`orient2d/3d`, `incircle`) classify the arrangement
   steps exactly; the 2D sketch kernel's typed entity machinery
   (ADR-0030) is the curve-arrangement engine in parameter domains
   — the p-curve trimming problem is a 2D problem, and it is the
   2D problem this house just shipped.
3. **Subdivision fallback, declared, never silent.** Where the pair
   has no closed form, adaptive subdivision with interval bounding
   (the `Interval` type in `core/math`) produces an approximated
   intersection curve that **carries its provenance and tolerance**
   in the data model: `exact` or `approximated(tol)` — visible in
   the report, serialized with the body, refused nowhere and hidden
   nowhere. This is the ADR-0026 declared-error culture applied to
   geometry provenance: the user who asks "is this edge exact?"
   gets an answer, not a shrug.

#### D4.2 Exact booleans with tolerances

*Why hard:* the boolean's correctness is only as good as the SSI
feeding it, and the degenerate configurations (coplanar faces,
coincident surfaces, tangential contact, on-vertex contact) are
exactly where naive implementations return bodies that fail Layer-1
invariants — the house's own Phase D boolean bug class (the
grid-straddle and corner cases the test suite carries as scars).

*Strategy:* the ADR-0024/0025 pipeline generalizes — face
classification stays (centroid-then-predicate, robust predicates
for the band), the clip primitive becomes SSI-fed, and the assembly
reuses the proven pattern: canonical vertex identity before edge
keying, integer-keyed dedup, `link_radial_edges`, hard postconditions.
*What is guaranteed:* closed results with exact Layer-1 invariants;
output geometry within declared tolerance; bit-identical re-runs on
the same platform/build (ADR-0014). *What is not:* mathematical
exactness of intersection points (within tolerance, provenance
declared); topology stability under epsilon-perturbation of
degenerate inputs — instead, a **declared tie-break policy** per
degenerate class, deterministic and documented, with the adversarial
corpus (below) pinning each class to a golden.

#### D4.3 Fillets on degenerate cases

*Why hard:* a constant-radius fillet is an offset-surface
intersection plus a trimming problem, and the hard core is the
**vertex blend** — where three or more fillet surfaces meet, the
closure surface has no general closed form and the degenerate
cases (radius too large for the topology, tangential edges,
variable-width slivers) flip the output topology (Hoffmann 1989's
blending chapters are the classic reference). The industrial
evidence is citable from this month: OCCT 7.9.3's release notes
headline *concurrent fillet construction* — 27 years in, the vertex
blend tail is still the work.

*Strategy:* G2 scope is the **edge fillet on the prism/cylinder
class** (rolling ball, analytic flanks, two-and-a-half-D closure) —
bounded, testable, golden-able. Vertex blends enter as *declared
geometry* with the failure regime pinned: the degenerate families
are enumerated, each either closed by a named construction or
rejected with an **actionable error** naming the degeneracy and the
remedy (the ADR-0026 P2 error-contract pattern — never a silent
fallback). The SDF `smooth_min` blend (ADR-0028) is positioned, not
confused with fillets: it is the organic-mode tool — no exact
radius, no G1 contract to analytic faces — and the UI must never
label it "fillet".

#### D4.4 NURBS trimming

*Why hard:* trimming a NURBS surface means solving curve-curve
intersection and arrangement in the parameter domain where the
parameterization itself distorts distances; conditioning of
polynomial bases in monomial form is poor, and knot insertion
propagates error (Farouki 1991 on Bernstein-form conditioning is
the standard citation).

*Strategy:* deferred to G3 with its entry criterion pinned (D5);
the approach when it comes: parameter-domain arrangement via the
2D kernel machinery, Bezier-form operations for stability, Bezier
clipping for intersections, Layer-2 tolerance on every trimmed
face. This ADR pins the slot, not the algorithm — that is the
daughter ADR's job.

### D5. The phase ladder — G1, G2, G3

House style throughout: each phase pins its invariant, its test
class, its entry criterion, and carries **one vertical guide case**
— a single user-meaningful story that crosses every layer of the
phase. Each phase is a **daughter ADR of its own** with its own
method; this ADR fixes the ladder, not the details.

#### G1 — topology and exact primitives

- **Scope.** Box / sphere / cylinder as exact B-Reps *with real
  trimming* (p-curves on loops, D3); exact mass properties computed
  **from the B-Rep** — volume, centroid, inertia — via the
  divergence theorem over trimmed analytic faces (per-face
  quadrature with closed forms for the analytic families; Mirtich
  1996 is the polyhedral baseline the analytic extension improves
  on). The one SSI entry needed: plane∩cylinder.
- **Pinned invariant.** The D3 contract: structural closure + χ
  consistency on every body; mass properties agree with the closed
  form to the declared quadrature tolerance (and the closed forms
  are the *tests*, not the implementation).
- **Test class.** Golden V/E/F per factory and per operation (the
  house's pinned-counts pattern, ADR-0026); closed-form mass
  property goldens (box: 8V/12E/6F, `V = abc`; the seam-wired
  cylinder: 4V/5E/3F per ADR-0023, `V = πr²h`); bit-identical
  re-runs (ADR-0014); `.CAD_0` v0.5 round-trip goldens.
- **Entry criterion.** This ADR accepted (the fork decision taken)
  and the daughter ADR-0031-G1 reviewed with the house method —
  invariants and discriminating tests named **before** the phase
  starts, per the review protocol this draft is entering now.
- **Guide case — the parametric hole, redone in exact B-Rep.** The
  same chain `examples/sketch_hole.py` walks today, rebuilt on
  exactness: extrude from the sketch → a prism whose faces are the
  profile's exact geometry (the circle's wall is a *cylinder
  face*, the caps are *planes* with circular trimming loops) →
  subtract → a hole with **true faces**. And the payoff the mesh
  can never give: **centroid and volume exact, read from the solid**
  — `V = abc − πr²h` for the through-hole, χ = 0, genus 1, the
  same topology numbers the current genus report already certifies
  on the faceted extract, now carried by geometry that *is* the
  mathematics instead of sampling it.

#### G2 — complete exact booleans + the first constant-radius fillet

- **Scope.** The analytic-pair SSI table to the level the product
  demands; booleans on curved faces under the D4.2 guarantees; the
  **first constant-radius edge fillet** — the first feature that is
  *impossible today*, and therefore the kernel's proving ground.
- **Pinned invariant.** D3's, plus the **intersection-provenance
  contract**: every intersection curve in a result body is labeled
  `exact` or `approximated(tol)` (D4.1.3), and the label is
  serialized and reported.
- **Test class.** The adversarial degenerate corpus — tangential
  cylinder pairs, coplanar and coincident faces, on-vertex contact,
  near-miss distances straddling the tolerance band — each family
  pinned to a golden with its tie-break documented (the
  failure-regime-in-range discipline of ADR-0030 D6, applied to
  geometry).
- **Entry criterion.** G1 landed. *(Amended on acceptance,
  2026-10-25 — the original criterion read: "G1 landed **and the
  fork re-decision taken** — the G2 gate is where D1's price list
  meets G1's measured reality (the robustness data of our
  analytic subset, the real cost of the adapter seam, the CI
  delta of an OCCT spike). This ADR arms that gate; it does not
  pre-decide it." The fork is closed by the decision — road (a),
  all in house, forever, the license whitelist of the amended
  ADR-0003 closing the adoption road as policy. The gate survives
  as an internal checkpoint only: G1's measured effort and
  robustness data against this ADR's estimates, re-baselining the
  G2 plan — never re-opening the build-vs-adopt question.)*
- **Guide case — the first fillet.** A constant-radius fillet on a
  box's vertical edge: three planar faces retire, a cylindrical
  face and four new edges are born, χ stays 2, and the mass
  properties shift by exactly the closed form the golden pins.
  Small on purpose: a feature impossible yesterday, exact today,
  and every layer of the kernel crossed by it.

#### G3 — free surfaces

- **Scope.** Loft, guided sweep, G1 blend — the NURBS layer of
  D3/D4.4, with the daughter ADR defining the surface classes, the
  continuity contracts (what "G1 blend" guarantees, where, and how
  it is asserted), and the trimming machinery.
- **Pinned invariant.** D3's closure family, plus the **continuity
  declaration** on blend faces (the guaranteed continuity class is
  part of the face's contract, reported like tolerance and
  provenance).
- **Test class.** Closure goldens + continuity/curvature checks on
  pinned sections + the D3 tolerance reporting on every trimmed
  NURBS face.
- **Entry criterion.** G2 landed; the NURBS daughter ADR accepted.
- **Guide case — a lofted transition.** Circle → rounded-rectangle
  loft between two sketches: the classic duct, impossible in every
  current representation (SDF approximates it, faceted B-Rep
  samples it), exact and closed under the invariants.

### D6. Migration — the DAG, the SDF boundary, and `.CAD_0`

#### How existing DAG and features remap

The recipe spine makes this cheap by design: since ADR-0027 D4 the
**recipe is the source of truth and the body is a cache** —
`rebuild_operations()` re-materializes bodies from recipes on every
parametric edit. The exact kernel therefore lands as a **second
materialization path**, not a migration:

- **Does extrude produce B-Rep?** The extrude operation node gains a
  B-Rep materialization next to its SDF one: same recipe, same
  FeatureId genealogy (ADR-0005), two bodies. The sketch profile's
  segments become plane faces, its arcs become cylinder (or cone)
  faces — the 2D entity types map one-to-one onto the G1 surface
  families, which is why the sketcher was worth building typed.
- **Does the SDF node die?** **No — pinned.** The SDF body stays the
  display and preview representation (ray-marching, MC33, picking)
  and remains the native representation of the organic workflow
  (`smooth_min`, twist — the jobs ADR-0002 gave it that B-Rep cannot
  do); the exact B-Rep body is the source of truth for solid
  properties, dimensions, and export. B-Rep display, where needed
  standalone, goes through `tessellate.hpp` (exists today). The
  bridge (ADR-0026) keeps crossing the two worlds with its declared
  tolerance and its error contract — for the organic shapes that
  will never be analytic.
- **HybridBody comes home.** ADR-0002's third variant — owns both
  representations plus a bridge tolerance — has been a type without
  a job since Phase A. This is its job: the sketch→extrude→subtract
  chain is the first honest `HybridBody` in the tree.

#### `.CAD_0` retro-compatibility — ADR-0007 talks

ADR-0007 anticipated exactly this moment, and its machinery is
already in the tree (the migrator chain, the golden files at every
historical version):

- **Existing v0.4 projects open unchanged, with zero migration.**
  Their recipes already fully describe the sketch/extrude/subtract
  chains; the exact B-Rep materializes on load the way bodies
  re-materialize on every edit today. No file is rewritten, no
  golden moves.
- **The format extends, not versions**: a **MINOR bump to v0.5.0**
  (new recipe kinds — fillet at G2; p-curve and Layer-2 tolerance
  fields where the exact path needs them), per ADR-0007's MINOR
  semantics (old readers open new files, unsupported features
  surfaced — never silently dropped). ADR-0007's own rule — "each
  MINOR or MAJOR format change requires a new ADR (0007b, 0007c, …)
  describing the migration steps" — means the G1 daughter ADR ships
  with its 0007-series companion carrying the v0.4 → v0.5 migrator
  and the new goldens.
- **On the OCCT roads:** the recipes stay house-native regardless
  of the engine behind execution — the parametric spine, the
  naming, and the history never leave this repository (that is the
  boundary that keeps ADR-0002 alive under road (b); OCAF is not
  adopted, the recipe DAG is the history system). The format
  records the engine provenance of materialized bodies only as
  cache metadata, which old readers already ignore by design.

## Consequences

- **On acceptance (2026-10-25):** Phase G runs the G1/G2/G3
  ladder on road (a) — all in house, forever; the G2 gate is an
  internal checkpoint (estimate vs. progress), never a fork. The
  D3 data model, tolerance layers, and Euler-Poincaré contract
  bind every future geometry ADR. The daughter ADR-0032 (G1) is
  drafted and enters the user's review; no phase starts before
  that review closes. ADR-0003 is amended in the same act (the
  license whitelist); the NOTICE does not change (no new
  dependency — road (a) adds none, ever, by policy).
- **The countermeasures the motivation cites are commitments:**
  every phase deliverable on its own, visible value at every
  step — the ladder's design, enforced by the delivery-unit
  discipline of the daughter ADRs; and the D1 failure modes stay
  named and watched (the perpetual prototype, tolerance-semantics
  creep, opportunity cost) with the ladder as their structural
  answer.
- **What this ADR still does not do:** no implementation, no CI
  change, no dependency addition (road (a) forbids the last by
  policy now).
- **What every road shares, this one too:** the SDF boundary of
  D6 (display, preview, evaluation, organic modeling —
  positioned, not killed); the D3 data model; the invariant
  discipline; and the parametric spine — recipes, naming,
  history — that stays this house's, whatever computes the
  geometry. On road (a) what computes the geometry is this
  house too.

## References

House records:

- ADR-0002 (dual representation), ADR-0003 (no third-party CAD
  kernels — the rule road (b) must face), ADR-0004 (tolerance
  model — D3's Layer 3), ADR-0005 (persistent naming by genealogy),
  ADR-0007 + 0007a (format versioning and the 0007-series rule),
  ADR-0010 (STEP AP203 analytic-only scope), ADR-0012/0021/0022
  (radial-edge topology, entity set, invariants), ADR-0013 + NOTICE
  (the license method and the release gate), ADR-0014 (FP
  determinism — scoped in D1(b)), ADR-0015/0016 (MC33, vertex
  welding), ADR-0023/0024/0025 (seam wires; convex booleans;
  Greiner-Hormann and the canonical-key assembly pattern),
  ADR-0026 (the bridge — declared-error culture, P2 error
  contract, the χ/genus report this ADR promotes), ADR-0027
  (recipes as source of truth; D6 invariants), ADR-0028/0029 (SDF
  blend bounds; twist contract — the SDF strengths D6 positions),
  ADR-0030 (the sketcher MVP — the 2D kernel D4.1 reuses, and the
  guide case's current form).
- docs/audits/2026-10-16-third-party-license-audit.md (the
  per-component license inventory the OCCT entry would extend).

Literature and external records:

- Braid, I. C. (1978). *The Synthesis of Solids Bounded by Many
  Faces*. Communications of the ACM 21(4).
- Boender, E. (1998). A survey of intersection algorithms for
  curved surfaces. *Computers & Graphics* 22(1).
- Farouki, R. T. (1991). On the numerical condition of polynomials
  in Bernstein form. *Computer Aided Geometric Design* 8(3).
- Fortune, S. Robustness in Geometric Algorithms. In *Handbook of
  Discrete and Computational Geometry* (Goodman & O'Rourke, eds.).
- Greiner, G., & Hormann, K. (1998). Efficient clipping of
  arbitrary polygons. *ACM Transactions on Graphics* 17(2).
- Hoffmann, C. M. (1989). *Geometric and Solid Modeling: An
  Introduction*. Morgan Kaufmann.
- Lorensen, W. E., & Cline, H. E. (1987). Marching Cubes: A High
  Resolution 3D Surface Construction Algorithm. *SIGGRAPH '87*.
- Mäntylä, M. (1988). *An Introduction to Solid Modeling*. Computer
  Science Press. (Euler operators; the generalized Euler-Poincaré
  relation of D3.)
- Mirtich, B. (1996). Fast and Accurate Computation of Polyhedral
  Mass Properties. *Journal of Graphics Tools* 2(2). (The
  polyhedral baseline G1's analytic quadrature extends.)
- Patrikalakis, N. M., & Maekawa, T. (2002). *Shape Interrogation
  for Computer Aided Design and Manufacturing*. Springer.
- Requicha, A. A. G. (1980). Representations of Rigid Solids:
  Theory, Methods, and Systems. *ACM Computing Surveys* 12(4).
- Shewchuk, J. R. (1997). Adaptive Precision Floating-Point
  Arithmetic and Fast Robust Geometric Predicates. *Discrete &
  Computational Geometry* 18(3).
- Weiler, K. (1986). *Topological Structures for Geometric
  Modeling*. PhD thesis, Rensselaer Polytechnic Institute.
- ISO 10303-42 / AP203 ed. 2 (the STEP scope of ADR-0010).
- Open CASCADE Technology: occt3d.com (license: LGPL-2.1 with the
  OCCT exception; release history — 8.0.1 of 2026-07-30, the 7.9.x
  maintenance line); SPDX license record "OCCT-exception-1.0";
  github.com/Open-Cascade-SAS/OCCT (the public development record
  D1(b) cites for the 27-year anchor).
