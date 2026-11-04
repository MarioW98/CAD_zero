# ADR-0040: G4 unit 1 — the sketch: the root system, the constraint solver, the bands it declares

## Status

**Accepted (2026-11-04) — the maintainer's review and the revisore's
passed; the register surgery applied in this SAME act (D6's protocol,
the ADR-0035/0036 precedent).** Morso 1 delivered this contract as
**Proposed** (docs-only: the derivation pinned, no code moved); the
review lifted the STOP it pins — the same gate ADR-0038 passed before
it — and m2 (IL SOLVER) starts on this acceptance. The surgery,
applied herewith, exactly as D6 declares it: **ADR-0035** amended in
place, dated — the D2 table gains the SOLVE row (ε_solve^lin re-homed
from `kJointCoincidenceTol`, ε_solve^ang from `kParallelAngleTol`,
the vocabulary's own one-value-per-world law); **ADR-0030** amended
in place, dated — the deferral REDEEMED (the entry criterion recorded
as MET, D5.2's value-coincidence and D5.3's definition state gaining
the solver's layer); the **error taxonomy's register** notes the
`sketch_*` family beside `fillet_*` and `reference_*` (ADR-0033 D4's
corpus, one line, nothing rewritten underneath); **ADR-0036 NOT
amended** (D6's own derivation: the grammar unextended — the solve is
value-level, the no-shift declaration covers every constraint edit);
**ADR-0014 NOT amended** (the quantizer rule already covers the
store-back); **ADR-0007/0027's MINOR bump stays REGISTERED and
UNARBITRATED** — the parallel discipline's contact point (a): the
register decides at the landing morso which line takes the next
number, never two bumps the same day. Until the next act of this
line, the mothers stand as amended here and nothing else moves.

**LA DISCIPLINA DEL PARALLELO, declared here, in the head, before
anything else.** Two lines are now open on main — the fillet (G2-u3,
its ADR-0039 under review) and this one — and the discipline that
keeps them from colliding is the method's own, written where it
binds:

- **THIS unit does not touch ONE file of the fillet's line.**
  `subtract`, `extrude`, `ssi`, `naming` — the materialization, its
  tables and its grammar — remain G2-u3's, exactly as ADR-0039 left
  them. The solver is sketch-side (the 2D kernel's entities, the
  scene's payload, the canvas, the format): the solved recipe is
  consumed by the materialization exactly as today's authored recipe
  is — the same certificate, the same walk tables, the same counts.
- **The contact points are contracted HERE, in ADR, and consumed
  after.** There are exactly two, both priced in D6: (a) the FORMAT
  version number — the sketch's constraints need a serialized payload
  the format does not carry, and the fillet's m4 has already
  registered the next MINOR bump for its own recipe; the register
  arbitrates which line lands it first, the other rebases on the
  landed number — **never two bumps the same day**; (b) the ERROR
  TAXONOMY's register — the `sketch_*` family joins the culture
  beside the `fillet_*` and `reference_*` families, one amendment
  note each, nothing rewritten underneath.
- **The operational rule, verbatim from the method: the lines
  alternate their bites on main; NEVER two lines on the same organs
  the same day; NEVER long-lived branches.** The G2 branch lesson is
  written in the register's own record — a branch that lives across
  reviews is a promise nobody can audit — and this ADR's morsi ride
  main one bite at a time, each standalone with the full suite green,
  sequenced against the fillet's morsi by the register.

Daughter of **ADR-0036** (the naming — the sketch is not a planet
that arrived late: it is THE ROOT SYSTEM that ADR already governs;
the genealogy is born from the author entities, the roots
`sk#F:<i>` are the chain positions, the add-at-end edit is covered,
the resolver's four classes are the downstream edit ALREADY BUILT;
D4's amendment — the edit that moves roots emits the map or refuses
— is the rule this ADR inherits and does not touch), of
**ADR-0035** (the vocabulary — the solver carries NEW MEMBERS, typed
or nothing: the satisfaction bands ε_solve beside ε_L/ε_P/ε_w/ε_C,
re-homed from the sketch kernel's own declared constants, D2 below),
of **ADR-0014** (the determinism — the solve is a numerical act,
repeatable bit for bit: same input, same output, the canonical
initial point, the canonical order; the discipline applies HERE
too, D2.2), of **ADR-0030** (the sketcher MVP — the solver's own
deferral, D1's "explicitly out and deferred to its own ADR" with its
entry criterion; the criterion is MET and redeemed here), of the
**CASO GUIDA DEL FORO PARAMETRICO** (the recipe with parameters —
the circle's radius is the recipe's parameter, the edit propagates
the whole chain deterministically; the sketch's dimensions ARE the
recipe's parameters, D3 says it in full), of **G1 intero**
(ADR-0032: what the author entities already do — the typed 2D fields,
the certificate, the exact extrude's table, the walk that consumes
the chain), and of **ADR-0031 D5** (the scale method: MAI "tutte" —
what the guide cases and the user engineer reach).

## Context — why the sketch is not a separate planet

The sketcher's constraint solver has been the register's honest gap
since ADR-0030 pinned it out: "explicitly deferred with its entry
criterion" — the MVP delivered, the propagation demonstrated. The
deferral's DERIVED REASON (0030 D1, argued at length) was the order,
not the difficulty: "first the value, then the research; the solver
needs something to solve — constraints bind entity-parameters to
entity-parameters, and those parameters exist only after the MVP;
building the solver first means building it against a speculative
data model." The MVP shipped 2026-10-24; the parametric hole runs
end-to-end; the entry criterion is met. What the intervening units
built — and this is why the sketch is not arriving at a foreign
planet but at the ROOT SYSTEM it always was — is everything the
solver was waiting for:

1. **The author entities exist, typed, validated, serialized.** The
   2D kernel carries segment / circle / arc (sketch_entities.hpp:
   `SketchSegment{a, b, interior_left}`, `SketchCircle{center,
   radius, hole}`, `SketchArc{center, radius, hole, start_angle,
   sweep}`), each entity's parameters are the recipe's own floats —
   the panel edits them, the format writes them, the certificate
   validates the composed chain (kJointCoincidenceTol = 1e-5, the
   pairwise check, the bounded-closure rule).
2. **The propagation exists and is deterministic.** Edit the
   circle's radius from the panel: the sketch node marks changed, the
   DAG pass re-materializes the extrude then the subtract in
   topological order, one undo entry, bit-identical rebuilds
   (examples/sketch_hole.py, the D7.5 table printed side by side).
   THIS is the value the solver binds: "the parametricity of this MVP
   is propagation, not search" — the solver adds the search on top
   of a delivered data model, exactly the order 0030 D1 argued.
3. **The roots exist.** ADR-0036 D4: "the chain position IS the root
   name (`sk#F:<i>`) — the buck stops at authored data." The roots'
   identity is the chain layout, the sketch's own authored order; the
   add-at-end edit is covered by the no-shift declaration; the
   middle-insert/deletion map is REGISTERED (0036 D4's queue, the
   review's own amendment) with its rule: the edit path that shifts
   referenced roots EMITS the map or REFUSES with the remedy.
4. **The downstream edit exists.** The resolver's four classes
   (H2-u6 morso 5, implemented and quartet-proven
   red-on-address/green-on-name): `reference_resolved` silent,
   `reference_renamed` logged, `reference_ambiguous` and
   `reference_vanished` actionable with payloads. The naive
   counterproof (the pentagon edit, the id-14 mis-reference) is the
   demonstration this ADR's golden 8 consumes again, from the sketch
   side this time.

**What is missing is IN THREE PIECES, and the ADR prices them:**

- **(a) The edit a valle — EXISTS.** The resolver's classes 1/3/4
  ARE the UX of the downstream edit; the covered path v1
  (add-at-end, no-shift) is exactly what the sketch's own edits
  practice today. This ADR adds nothing to it and pays nothing for
  it: it RATIFIES it (D3) and derives the one thing the solver adds
  — the proof that the solve itself is layout-preserving by
  construction (D3.3).
- **(b) The workbench — media.** The canvas exists (the gestures,
  the 10 px exact-formula pick, the handles, the suggestions offered
  never applied, the visible definition state); the property panel
  routes parameters by name. What is missing is the READING of the
  degrees of freedom (the honest counter), the solve's feedback (the
  classes in interface), and the pinning. Derived in D4, priced
  small — the registry pattern carries it, zero new panel code.
- **(c) THE SOLVER — la parte veramente nuova e dura.** The heart of
  this ADR (D2): the constraint graph, the bill of the degrees of
  freedom, the admission's rank test, the damped min-norm Newton
  with the canonical initial point, the typed bands, the three
  honest failure classes, the degenerate verification before the
  recipe write. Everything else in this document is either ratified
  (D3) or priced small (D4); D2 is where the derivation lives.

**Where the tree stands today (verified, the repo the authority; the
working tree clean at b704495, the ADR-0039 delivery):** G2-u3 m1
delivered as patch/zip (the fillet's ADR under review, no code);
G2-u2 complete (the blind door, the crossed corner, the decoupling
rule — d3bb0fc); the naming grammar, the tavola and the resolver
implemented end to end (H2-u6); the coplanar door green (H2-u5);
`joint(i)` born (the crossed corner's landing). The sketcher MVP is
complete: the three entity kinds, the chains (one outer + holes,
closed at materialization — an open chain is a legal, visibly-flagged
editing state, D5.3's own pin), the canvas (1279 lines: the gestures,
the suggestions at 6 px / 8 px, the inline editors, the status line),
the v0.4 format with the sketch payloads, the UI smoke suite. The
register says it honestly: "The sketcher's constraint solver —
explicitly deferred with its entry criterion" — the one Phase-F
sentence left standing, redeemed by this document.

## Decision

### D1. The entities and the constraints v1 — MAI "tutte" (the method of ADR-0031 D5)

**LE ENTITÀ, v1 — the kernel's own three, nothing invented.** The
DoF of an entity is the count of the independent real parameters in
its recipe payload — the same floats the panel edits and the format
writes, DERIVED here, not cited:

| Entity | The recipe's fields | DoF |
|---|---|---|
| **linea** (segment) | `x0, y0, x1, y1` | **4** |
| **cerchio** (circle) | `cx, cy, r` | **3** |
| **arco** (arc) | `cx, cy, r, a0, sweep` | **5** |

**IL RETTANGOLO — DERIVED AS COMPOSITION, not a fourth kind.** The
rectangle is a chain of 4 segments plus a constraint set the macro
writes at creation. The derivation, argued once:

- A fourth entity kind would fork the certificate (a new
  construction-check row), the extrude's walk table (a new numbering
  family), the serialization (a new payload kind) and the naming (a
  new root form) — four surfaces, zero new geometry: a rectangle IS
  four segments and four right angles, the prism-of-n formula
  already pins its materialization (V = 2n, E = 3n, F = n+2, χ = 2:
  the n = 4 case is the box's own 8/12/6).
- The macro's IMPLICIT SET, counted and pinned (the numbers derived
  in scripts/adr0040_derivation.py, verified numerically):

  | The macro writes at creation | Consumes |
  |---|---|
  | 4 coincidences (the chain's joints, closing included) | 8 |
  | 2 parallelisms (the opposite pairs) | 2 |
  | 1 perpendicularity (one corner; the other three follow) | 1 |
  | **the implicit set** | **11** |
  | + 2 length dimensions (w, h at the gesture's values) | 2 |

  The bill at creation: 16 − 13 = **3 free** (the rigid motion:
  translation 2 + rotation 1) — reported, never hidden; the user
  pins (3 pins) and the bill returns: 16 − 16 = 0 FULL. The
  rectangle is the FIRST RECIPE-SIDE MACRO: one gesture (press at a
  corner, drag, release at the opposite corner) writes 4 entities +
  13 constraints in one act, the values visible per ADR-0030 D5.1's
  own rule. The entity kinds stay THREE.

**I VINCOLI v1** — with the consumption of each, DERIVED as the
numerical rank of the constraint's Jacobian at a canonical point
(the script verifies every row):

| Constraint | The equation (v1's form) | Consumes |
|---|---|---|
| **coincidente** (endpoint, endpoint) | the two coordinates equal — any two endpoints of any two entities (a chain joint or not) | **2** |
| **parallelo** (segment, segment) | the cross of the UNIT directions = 0 (scale-invariant: the sketch world's Angular kind) | **1** |
| **perpendicolare** (segment, segment) | the dot of the UNIT directions = 0 | **1** |
| **tangente** (linea, cerchio) | the signed distance from the center to the line = s·r, **s = ±1 the CASE BIT** | **1** |
| **la DIMENSIONE — lunghezza** (segment) | \|AB\| = v, **v > 0** | **1** |
| **la DIMENSIONE — raggio** (circle/arc) | r = v, v > 0 | **1** |
| **la DIMENSIONE — distanza** (point, point) | \|PQ\| = v, **v > 0** (the singularity at v = 0 derived below) | **1** |

**LA TANGENTE — I DUE CASI, e la discriminazione conta.** The
equation alone (distance = r) admits BOTH branches: the line tangent
with the disc on one side or the other. The constraint carries the
case as DATA — **ESTERNO: s = +1** (the signed distance positive
under the left-normal convention: the disc on the far side of the
line from the profile's material at the tangency), **INTERNO:
s = −1** (the chord-limit shape: the disc on the material's side).
The bit is derived at CREATION from the current geometry (the side
the center is on, at the moment the author declares the constraint)
and pinned in the recipe; the solver consumes the sign and NEVER
chooses. Without the bit, an edit that pushes the geometry across
the branch would let Newton settle on the wrong side — the
discrimination is the same door the SSI table's tangent degenerates
keep closed, applied to the sketch.

**LA DISTANZA's v > 0, derived.** The point-point distance at
v = 0 is the coincidence — but the equation's gradient
(P−Q)/\|PQ\| is UNDEFINED at its own solution: a constraint
non-differentiable at the point it demands is unsolvable by Newton
by construction. Refused at validation with the remedy ("use the
coincidence — the right tool for v = 0"). The same rule bans the
zero length (a segment of length 0 has no direction to speak) and
the zero radius (Disc2D's own construction check).

**LE FAMIGLIE IN CODA, A NOME** (never footnoted; each with its
entry criterion, no date):

- **simmetrico** (symmetry about an axis / a point) — entry: a real
  case the pins cannot serve honestly.
- **allineato** (the midpoint on a curve; three points collinear) —
  entry: the same.
- **gli assi** (horizontal / vertical as CONSTRAINTS — the datum
  directions): the D5.4 suggestions (6 px) remain VALUE-OFFERS
  forever until this family lands; the pins substitute in v1. Entry:
  the editing flow that must HOLD the relation under later edits.
- **le equazioni** (equation-driven dimensions) — entry: a consumer
  that needs the formula surface.
- **le spline** — **BLOCCATE da G3**: the sketches do not promise
  curves the kernel does not have. The entity set is the kernel's
  own three; G3 (free surfaces, the ladder's declared third rung,
  not started) owns the curve families, and the sketcher will
  consume them the day they exist — the extrude's class discipline
  (the profile must be in the kernel's class) applied to the
  constraint vocabulary.
- **lo sketch 3D** — blocked by the work planes queue (ADR-0030's
  deferral: arbitrary planes with their frame derivation, then
  sketch-on-face, Phase F-2's research core).

### D2. The solver — the mathematical derivation (the heart)

#### D2.1 The structure: the graph, the bill, the three instruments

**IL GRAFO.** Entities are nodes, constraints are edges, the sketch
is the whole graph — not per-chain. The coincidence typically binds
chain neighbors (the joints), but the tangent binds entities across
chains (the hole tangent to the outer boundary): the constraint's
operand is (entity, endpoint) / (entity, entity), the graph is
sketch-wide. The parameters vector x is the recipe's own floats in
CANONICAL ORDER: chain order, entity order, field order — the
serialization's order, the same order the certificate and the format
already practice. The constraints list is recipe data in authored
order (append-only in v1, D3.3).

**LA BOLLETTA — the honest answer for each state.** With DoF the
sum of the entities' parameters and the consumed the sum of the
admitted constraints' consumptions (the admission guarantees
consumed = rank, D2.1's test):

- **SOTTOVINCOLATO (consumed < DoF): the partial solve — DERIVED,
  never silence, never a refusal.** The policy and its reason: the
  propagation world MUST survive the solver's landing day. The
  parametric hole itself is under-constrained by the solver's own
  bill (a circle with one radius dimension: 3 − 1 = 2 free); today
  that sketch "solves" trivially (nothing to move) and regenerates
  on every edit; a solver that REFUSED it would break every existing
  sketch the day it lands — the regression the value order exists
  to prevent. So the under-constrained sketch SOLVES: the
  constraints hold exactly (within their bands), the FREE DoF stay
  at their authored values as parameters (editable, flowing into the
  recipe exactly as today), and the count is REPORTED (D4's counter:
  "2 gradi liberi"). The free directions are never moved by the
  solver: the step has no component in J's null space (D2.2's
  derivation) — the free DoF are the ones the NEXT edit moves, by
  hand, as always.
- **PIENO (consumed = DoF, rank full): solved.**
- **SOVRABBONDANTE: REFUSED AT ADMISSION, BY NAME.** The class
  **`sketch_over_constrained`**, ACTIONABLE — the house taxonomy's
  shape (the `fillet_*` / `reference_*` families' sibling). The
  admission test: the candidate constraint's Jacobian row, at the
  CURRENT geometry (the canonical point — declared as such, the
  point test's honesty named: the global question is the solve's
  own, D2.2's failures answer it); if the row does not raise the
  rank of the admitted set, the constraint is REDUNDANT — the
  refusal names the culprit (its kind, its operands, its value), the
  bill (e.g. 12/12 FULL), and the remedy (remove it, or remove what
  it duplicates). A `rank = DoF` sketch refuses EVERY further
  constraint by the same test — over-constrained never reaches the
  solver.

**The three instruments, each covering what the others cannot** (the
certificate's own two-voice discipline, ADR-0030 D6 revision 2,
applied at the solver's scale): THE COUNTING (the bill: the
arithmetic, cheap, always displayed), THE ADMISSION (the rank test:
the linear-algebra verdict at the canonical point), THE SOLVE (the
nonlinear verdict: the iterate that lands or the honest failure).
The counting alone lies (dependent rows); the rank alone is local
(a point test); the solve alone is too late (the edit already
happened). Together, and only together, every state has its named
answer — exactly the samples-plus-pairwise sufficiency argument the
profile certificate made against lie #5b.

#### D2.2 The numerical method: the damped min-norm Newton, the bands, the honest failures

**IL METODO.** Newton on the constraint system, in the Levenberg-
damped min-norm form:

    (JᵀJ + λI) δ = −JᵀF        δ applied to x, λ adaptive

The step is the min-norm least-squares step at λ → 0 and the
gradient step at λ → ∞; the free directions are NEVER moved — JᵀF
has no component in J's null space (for v with Jv = 0: vᵀJᵀF =
(Jv)ᵀF = 0), so the under-constrained case's free parameters ride
the iteration untouched at every λ. The protection engages ON THE
REJECTION: a try that does not decrease the distance-to-solved (the
acceptance test's metric, below) escalates λ (×10) and retries; an
accepted step relaxes it (÷4, floored at 1e-14). λ₀ = 1e-6, light
on purpose — the well-posed cases run as pure Newton from the first
step; the damping is the answer to FAILURE, not a pre-emptive tax.
The UNSCALED form is deliberate: the bands are NOT folded into the
linear algebra (1/band ≈ 1e5 would put the conditioning at 1e16 and
the solve under); they enter where they belong — the acceptance
test and the band check. Declared constants: λ₀ 1e-6, ÷4, ×10, the
λ cap 1e12, the step floor 1e-12, the stall window 3, MAX_ITER 64
(the worst measured golden 19, the margin stated honestly: 3.4× —
the cap is the not-solvable boundary, not a promise).

**IL PUNTO INIZIALE CANONICO.** The current geometry — the recipe's
authored values as the solve starts. **MAI restart casuali**: the
determinism of ADR-0014 applied to the solve — same input, same
output, BIT FOR BIT: the canonical initial point, the canonical
order (constraints in recipe order, entities in chain order, fields
in serialization order), pure arithmetic (+, −, ×, ÷, sqrt — the
equations are designed so; the arc's trig is the one exception,
priced below), `-ffp-contract=off` the house flag. N solves from the
same point → identical (the golden's determinism leg): the
derivation script measures it, 10/10 identical.

**LE BANDE — the member ε_solve, in ADR-0035's vocabulary.** The
satisfaction band PER CONSTRAINT: every \|violation\| ≤ its band,
where the band is TYPED per world, RE-HOMED from the sketch
kernel's own declared constants — ONE value per world, the
certificate and the solve the two consumers, ZERO DRIFT (the
D2.1 pattern of ADR-0035: the welding grid doubles as the canonical
key's quantizer):

| Member | Value | Re-homed from (sketch_profile.hpp) | The margin over float32 noise |
|---|---|---|---|
| **ε_solve^lin** (the Linear world: the coincidences, the dimensions, the tangency's distance, the pins on coordinate fields) | **1e-5** absolute (the O(1)–O(10) authored scale) | `kJointCoincidenceTol` — the chain's joint coincidence | 16.8× (5.96e-7 at scale 10) |
| **ε_solve^ang** (the Angular world: the parallel's unit cross, the perpendicular's unit dot, the pins on angle fields) | **1e-6** dimensionless | `kParallelAngleTol` — the seg-seg parallel classification | 8.4× (1.2e-7 on the unit cross) |

The angular member MUST sit at or below the certificate's
classification band (1e-6): a looser solve band would drift from the
materialization's own classification — the drift is the worse evil,
so the margin is the honest 8.4× and it is DECLARED, not hidden.
The vocabulary's new row (ADR-0035's D6 amendment): the **Solve
kind**, beside ε_L / ε_P / ε_w / ε_C.

**LA NON-CONVERGENZA ONESTA.** The class
**`sketch_not_solvable`**, ACTIONABLE, payload: the residual per
constraint (the largest named — the constraint that does not hold),
the values at the exit (bounded: the damped least-squares closest
approach, never a diverged number reported), and the honest
diagnosis. **MAI un'approssimazione spacciata per solved**: the
iterate that fails the band is an ANSWER with a class, not a
solution with an asterisk. The failures: the STALL (no decreasing
step at the λ cap, or the step at the floor with the band unmet,
3 times) and the CAP — either way the same class, NEVER a crash.
The goldens' derivation: the 4/5/10 triangle (the inequality
violated: full rank at admission — the rank test cannot see global
inconsistency, that is the SOLVE's question) exits stalled at 35
iterations with \|F\| = 0.408, the flat chain (A=(0,0),
B=(4.1667,0), C=(9.5,0)) as the bounded closest approach, the
largest violation 0.1667 spread across the length rows, the
diagnosis: "4+5 < 10 — no triangle with these sides exists."

**LA GEOMETRIA DEGENERE DOPO IL SOLVE.** The solution that makes
the chain degenerate — **il segmento a lunghezza zero, l'arco che
si chiude, i punti che coincidono** — is a CLASS of the taxonomy:
**`sketch_degenerate`**, ACTIONABLE. The result of the solve is
VERIFIED against the kernel's degenerate classes BEFORE it enters
the recipe — the solve that degenerates REFUSES WITH THE NAME (il
muro di casa, applied to the solve). The verification pass runs on
the STORED bytes (the certificate's own discipline: the checks
consume the recipe's floats):

- The DIRECT forms are caught EARLIER, at their own gates: a length
  dimension of 0 (validation, v > 0), a self-coincidence plus a
  length (the admission's rank test — the length row is a
  combination of the coincidence rows), an arc's sweep pinned to
  2π (the solve writes it, the construction check
  `sweep ∈ (0, 2π)` refuses — the open interval is the circle
  entity's business).
- The PRODUCED forms are caught HERE: the solve's output, stored to
  the recipe, re-read and checked. The store itself is the guard's
  floor (ADR-0014's amendment: the float32 cast is the value's
  quantizer): the derivation's sharpest case — the self-coincidence
  on one segment lands the endpoints at 3.0 ± 2.1e-11 (1 ulp apart
  in double, 4.4e-16); the float32 store rounds BOTH to 3.0f, the
  stored segment is EXACTLY zero-length, the construction check
  `p0 ≠ p1` refuses with the kernel's own words ("a degenerate
  segment has no normal to orient... give the segment a non-zero
  extent") — the payload is the kernel's class VERBATIM, the
  resolver's class-4 discipline (the payload reads the record the
  operation wrote, never a diagnosis it invents). The recipe is
  UNTOUCHED (the refusal precedes the write).

**The error taxonomy, the sketch_* family** (the culture's third
province, beside `fillet_*` and `reference_*`): `sketch_over_constrained`
(at admission), `sketch_not_solvable` (at the solve),
`sketch_degenerate` (at the verification) — each ACTIONABLE, each
with its payload, never a silence. The successes are answers too:
`solved` with the bill (the free count), the UI's plain words (D4).

**The store-back boundary, priced once.** The solver computes in
double, the recipe stores float32 (the sketch world stays float —
ADR-0035 D4's road (a): the sketch kernel float, the frontier at
the exact path, lossless float→double in the other direction). The
solved doubles → the float32 recipe: the store's quantizer
(ADR-0014's own rule), the noise 5.96e-7 at scale 10, inside every
band by the margins above. The ARC's endpoint equations carry libm
trig (cos/sin of a0) — the one place 1 ULP can differ across
libm/ucrt: the in-platform bit-identity is total; the cross-platform
bar is the SERIALIZED one (the 12-decimal quantizer absorbs the
1ULP: the recipe's floats round-trip identical) — ADR-0014's own
answer, declared, no amendment needed. The v1 goldens pin the
segment/circle guide cases; the arc's trig-carrying branch is
verified by the same machinery and REGISTERED for its first pinned
case (the notch profile's consumer), never footnoted.

**The linear algebra is IN HOUSE.** No Eigen, no LAPACK — the
third-party policy's whitelist unchanged. The systems are v1-small
(the params ≤ ~64, the sketcher's authored scale); the house
implements the one file it needs (the Cholesky/LU with the
fixed-pivot discipline, the SVD's structure where the rank test
needs it): deterministic, pure arithmetic, the same bar the
predicates set. Priced in D6's estimates.

#### D2.3 What the solve is NOT

Not a global optimizer (no branch enumeration, no interval
arithmetic): the canonical initial point chooses the branch, the
author's geometry is the intent. Not a load-time act: the load
never re-solves (the recipe is the truth — the same-doctrine answer
that keeps the body a cache); the solve is an EDIT-TIME act, the
same seat today's parameter edits sit in. Not a second opinion:
the materialization consumes the solved recipe exactly as today's
authored recipe — the certificate, the walk, the counts unchanged
(the parallel discipline's promise, kept by construction).

### D3. The bridge with the naming and the recipe (the piece already standing, ratified and steso)

**D3.1 LE DIMENSIONI SONO I PARAMETRI DELLA RECIPE.** The
parametric hole already practices it: the circle's radius is the
recipe's parameter, the panel edits it, the whole chain regenerates
deterministically. The ADR says it for esteso: the sketch's
dimensions ARE the recipe's parameters. Under the solver the
formalization is exact and minimal:

- The DIMENSION CONSTRAINT carries the value (its data): the radius
  dimension's v IS the parameter — the panel's editable surface for
  that quantity (the constraint list rides the payload beside the
  chains; the panel routes by name, the registry pattern, zero new
  panel code).
- The ENTITY's fields are the solved values (the solve's output,
  written at the edit, serialized with the constraints: both are the
  recipe, the round-trip preserves both, the load trusts both —
  never re-solved, D2.3). The two are consistent BY CONSTRUCTION
  (the solve writes the fields from the constraints); a hand-edited
  file's inconsistency is the next solve's to overwrite, the
  certificate's to catch at materialization.
- The free parameters (the under-constrained's leftover) are edited
  exactly as today: the handle drag, the inline editor — the same
  surface, the same ONE undo entry, the propagation the same chain
  (the sketch → the extrude → the subtract, the DAG pass's
  topological order). IL REVISIONATORE PARAMETRICO CONSUMA QUESTA
  ADR: the edit that regenerates without reselection is the
  quartet's class-1 leg, live (golden 8).

**D3.2 THE COVERED PATH and the four classes as UX — ratified,
not rebuilt.** The v1-covered flows (0036 D4's amendment, the
review's own): add-at-end (no positions shift), the parameter/value
edits. The UX: the radius edit → **classe 1, silenziosa** (the name
re-derives, the feature re-runs, no reselection); the re-author
(the circle re-authored as a two-arc chain) → **classe 3, i
candidati NOMINATI** (`lat(0)` and `lat(1)`, the remedy with them,
never a coin flip); the substitution (the circle replaced by a
square chain) → **classe 4 col perché** (what died, why, the
remedy). The same words as the resolver — no promise new.

**D3.3 THE SOLVER'S OWN EDIT FAMILY, and the root map — the
derivation of what v1 EMITS and what v1 REFUSES.** The solver adds
one edit family to the sketch's surface: the constraint edits (add,
remove, re-value). The derivation that costs nothing and buys the
whole bridge:

- **The solve moves VALUES, never the LAYOUT.** The constraints bind
  parameters; the chain's order, the entities' positions, the roots
  `sk#F:<i>` are untouched by any solve — the no-shift declaration
  covers every constraint edit and every re-solve, BY CONSTRUCTION.
  The roots rule (0036 D4's amendment) is inherited unamended: the
  edit that would shift roots (the middle insert/deletion) REFUSES
  or EMITS the map — the registered boundary, unchanged, priced by
  its consumer (still none today: nothing references a constraint,
  and no constraint edit can move a root).
- **The constraint list's own identity**: the authored order, the
  position — the same honest authored-data addressing as the
  chains' entities (0030 D2's violation clause). The edits are
  append/remove/re-value in v1; the middle-insert of a constraint
  shifts the later positions and is REFUSED by the same family rule
  (registered, priced by its consumer — none today).

### D4. The workbench v1 (the UX scope, DERIVED not painted)

- **La creazione e la selezione delle entità** — EXISTS, untouched:
  the canvas's gestures (the rubber band, the growing circle, the
  sweeping arc), the 10 px exact-formula pick, the handles, the
  suggestions OFFERED never applied (D5.4's set stays value-offers
  until the axes family lands). The rectangle macro's gesture is
  the one addition (D1's composition: press, drag, release at the
  opposite corner — 4 entities + 13 constraints in one act, the
  values visible).
- **LA LETTURA DEI DoF** — the honest counter, NEW and small: after
  every act (a constraint, an edit, a pin) the bill displayed on the
  canvas's status line — "3 gradi liberi", "pieno (0)" — the
  under-constraint REPORTED, never hidden. Recomputed at the cold
  sites only (constraint add/remove, parameter write), never per
  frame — the certificate's own economy.
- **Il feedback del solve** — the classes in interface:
  `solved` (with the free count), `sketch_not_solvable` (col
  residuo, the culprit named), `sketch_over_constrained` (col
  colpevole, at the admission). The panel's text role, the registry
  consumer pattern; zero new panel code.
- **L'edit dei parametri con la rigenerazione a valle** — the
  existing propagation, now through the solve: the edit writes (the
  dimension's value or the free parameter), the solve runs, the
  recipe's fields update, the DAG pass re-materializes the
  dependents — ONE undo entry, bit-identical, the sketch_hole
  example's own discipline.
- **IL PINNING** — il fissare un DoF: the under-constrained that
  becomes full. The pin is the DIMENSION family's fourth form,
  DERIVED here, priced by its consumer (this gesture): the
  raw-parameter dimension — one equation, `x_i = v` (the parameter
  equals the value it had at pin time), consumes 1, linear,
  differentiable everywhere, any v. The three named forms
  (length/radius/distance) are the derived-quantity measures; the
  pin is the atom they measure with. The workbench's honest tool:
  the under-constrained becomes FULL without the queued families
  (symmetry, axes, equations) — the pins carry v1, and the axes
  family's entry criterion is unchanged (a real case the pin cannot
  serve).

### D5. The golden pre-pinnings — the numbers DERIVED before the code

The protocol of always (ADR-0034 D4 / ADR-0035 D5, verbatim): every
claim has the test that fails on the naive version, agreed before
the code. The derivation lives in
`scripts/adr0040_derivation.py` (the numbers below are its output,
verified in double, the same arithmetic the m2 will implement):

1. **IL TRIANGOLO SEMPLICE** — the three lines, the three
   coincidences, the dimensions: the bill **12 − 12 = 0** (the
   coincidences 6 + the lengths 3 + the pins 3 — the pins are D4's
   raw dimensions: x_A = 0, y_A = 0, y_B = 0). The drawn (initial)
   point: the gesture's landing, the joints' copies slightly off —
   s0 (0.10, 0.05)→(4.15, −0.05), s1 (4.10, 0.02)→(0.05, 3.10),
   s2 (0.08, 3.15)→(0.12, −0.02). The solve: **4 iterations**; the
   solution derived COORDINATE FOR COORDINATE: A = (1.2e-30,
   1.5e-30), B = (4, −7.7e-21), C = (−4.3e-17, 3) — the identity
   3-4-5, machine-exact; the measured lengths 4, 5, 3 EXACT. The
   determinism: **10 solves from the same point, byte-identical**.
   The rank at admission (the drawn point): 12/12 FULL.
2. **LA TANGENTE** — linea-cerchio ESTERNA: the bill **7 − 2 = 5**
   (the tangent 1 + the radius dimension 1; the 5 free REPORTED).
   The drawn: the boundary segment (2.20, 0.10)→(1.90, 5.10), the
   hole circle center (−0.05, 2.45), r = 2.10 — the user drew the
   hole tangent to the boundary (the case bit derived at creation:
   ESTERNO, s = +1, the center on the left-normal side). The solve:
   **19 iterations** (the under case's honest count — the free
   directions ride untouched); the solved configuration: the line
   a = (2.1303, 0.0960) b = (1.900, 5.100), the circle
   C = (0.0197, 2.4540), r = 2; the signed distance
   **2.00000000000026 = r** (the band's margin 2.6e-13 ≪ 1e-5);
   **IL PUNTO DI TANGENZA DERIVATO**: T = (2.01756, 2.54595) — the
   foot of the perpendicular from C, \|T − C\| = 2.00000000000026 =
   r, T on the line (the cross 3.05e-16), T on the segment's span
   (the fraction 0.4896 of the length 5.0093) — the certificate's
   business, checked and green.
3. **IL RETTANGOLO VINCOLATO** — the implicit set counted (11: 4
   coincidences 8 + 2 parallels 2 + 1 perpendicular 1), the
   dimensions w = 6, h = 4, the pins 3: the bill **16 − 16 = 0**
   FULL — LA BOLLETTA TORNA. The drawn: the four corners near
   (0,0), (6,0), (6,4), (0,4) with noise; the solve: **5
   iterations**; the solved rectangle EXACT at the pinned placement
   (0,0) (6,0) (6,4) (0,4). The materialization's counts by the
   existing pinned formula (n = 4): **8/12/6, χ = 2** — the box's
   own class, unchanged.
4. **LA CONTROPROVA NAIVE** — the sketch WITHOUT the solver: the
   dimensions ignored (display-only, D5.2's read-only rule — the
   write requires choosing which endpoint stays, an interaction
   decision, not a parameter write), the geometry stays where it
   is. The measured lengths on the naive: \|AB\| = 4.0512 (+5.1e-2),
   \|BC\| = 5.0881 (+8.8e-2), \|CA\| = 3.1703 (+1.7e-1) — the
   assertions of golden 1 FAIL on the naive for the right reason
   and PASS through the solver: **LA DISCRIMINAZIONE**. On today's
   tree the red is the absence itself (the constraint API does not
   exist — the test file's own compile naming it, the m2 red-first
   discipline).
5. **IL SOVRABBONDANTE** — golden 1's sketch (12/12 FULL) plus the
   13th: the distance dimension on (s0.end, s1.end) = 5 — the
   duplicate of the length c4 the user adds thinking it needed. The
   rank test at the canonical point: the 12 admitted rows rank 12;
   the 13-row matrix rank 12 (the spectrum's tail s11 = 3.791e-01,
   s12 = 2.342e-01 — the null direction) → **REFUSED AT ADMISSION**:
   `sketch_over_constrained`, the payload with the KIND (the
   distance on (s0.end, s1.end) = 5), the bill (12/12 FULL), the
   remedy. Never solved, never silent.
6. **LA NON-CONVERGENZA** — the case constructed without solution:
   the triangle with the lengths 4, 5, 10 (4 + 5 < 10 — no triangle
   with these sides exists; the intuitive shape the task names: the
   circle that must touch and contain simultaneously). The rank at
   admission: 12/12 FULL (the rank test cannot see global
   inconsistency — that is the solve's question, the instruments'
   division of labor); the run: **35 iterations, STALLED** —
   `sketch_not_solvable` with the payload: the residual per
   constraint (the largest 0.1667 spread across the length rows),
   \|F\| = 0.408, the values at the exit BOUNDED (the flat chain:
   A = (0,0), B = (4.1667, 0), C = (9.5, 0) — the least-squares
   closest approach, never a diverged number), the diagnosis named.
   **MAI un crash.**
7. **IL DEGENERE** — il solve che azzera un segmento: one segment
   drawn (1, 0)→(5, 0) with the self-coincidence constraint
   (s0.start ≡ s0.end). The bill 4 − 2 = 2 free (reported). The
   solve: **3 iterations**; the landing: a = b = 2.999999999978878
   (1 ulp apart in double, 4.4e-16 — the min-norm midpoint, the
   nearest solution to the drawn point); the STORE: both endpoints
   round to 3.0f, the stored segment is EXACTLY zero-length → the
   verification REFUSES with the name: `sketch_degenerate`, the
   payload the kernel's own class VERBATIM ("a degenerate segment
   has no normal to orient... give the segment a non-zero extent"),
   the recipe UNTOUCHED (the refusal precedes the write).
8. **L'EDIT A VALLE** — il primo atto INTERO dal vivo: the triangle
   solved (golden 1's output), extruded (distance 2, +Z — the
   prism **6/9/5, χ = 2** by the n = 3 formula), THEN the edit: the
   dimension \|BC\| 5 → 5.5. The re-solve from the canonical
   initial point (the pre-edit solved geometry — C = (0, 3)):
   **5 iterations**; the new C DERIVED: (−0.65625, 2.927342811749249)
   — the closed form x = (9+16−30.25)/8, y = √(9−x²), matching the
   solve to machine precision; the branch y > 0 (the initial
   point's side) — deterministic. The names re-derived IDENTICALLY
   (`ext#F:lat(0)`, `lat(1)`, `lat(2)`, `cap(lo)`, `cap(hi)` — the
   roots 0..2 unmoved: the edit is value-level, no-shift); the
   resolver: a reference to `ext#F:lat(1)` resolves **CLASSE 1,
   SILENZIOSA**; the regeneration WITHOUT reselection; N rebuilds
   bit-identical. The quartet's own class-1 leg, live, from the
   sketch side — the consumer the fillet will inherit.

**The golden declaration (pre-registered):** no existing golden
moves (the solver is additive: the recipe's fields, the chains, the
materialization untouched where the values already satisfy the
constraints — the parametric hole's own goldens stand); the eight
are NEW oracles, red-first on today's tree (the absence itself);
determinism from the first landing (no prior solve state to move).

### D6. The consequences — the register surgery, the ladder, the estimates

**The register surgery — declared; applied in the same act as
acceptance** (the ADR-0035 D6 protocol):

- **ADR-0035 — amended in place, dated, pointing here**: the D2
  table gains the SOLVE row — the two members ε_solve^lin = 1e-5
  (re-homed from `kJointCoincidenceTol`) and ε_solve^ang = 1e-6
  (re-homed from `kParallelAngleTol`), the sketch world's own,
  beside ε_L / ε_P / ε_w / ε_C. ONE value per world, the
  certificate and the solve the two consumers, zero drift.
- **ADR-0030 — amended in place, dated, pointing here**: the
  deferral redeemed — D1's "the constraint solver... explicitly out
  and deferred to its own ADR" is paid by THIS document; the entry
  criterion recorded as MET (the MVP delivered 2026-10-24, the
  propagation demonstrated); D5.2's value-coincidence and D5.3's
  connectivity-only definition gain the solver's layer (the
  coincidence becomes the constraint's business; the definition
  state gains the DoF counter).
- **ADR-0036 — NOT amended, and the derivation of why**: the
  grammar is NOT extended (the solver adds no entity kind, no role
  row, no feature kind — the roots' identity is untouched; the
  solve is value-level, the no-shift declaration covers every
  constraint edit, D3.3). The mothers' D6 protocol honored: nothing
  extended, nothing owed.
- **ADR-0014 — NOT amended**: the quantizer rule already covers the
  store-back (the float32 cast IS the quantizer — the 2026-10-29
  amendment's own sentence); the arc's trig and the serialization's
  12-decimal quantizer: ADR-0014's own answer, declared in D2.2.
- **ADR-0007/0027 — the version bump REGISTERED, the number
  ARBITRATED BY THE REGISTER** (the parallel discipline's contact
  point (a)): the sketch recipe gains the CONSTRAINTS array — a new
  serialized payload; a v0.4 reader would round-trip a
  constraints-bearing file DROPPING them silently — data loss
  through downgrade, the exact violation ADR-0027 D4 rejected. NOT
  unknown-field tolerance: a MINOR bump with the 0007-series
  companion and the chained migrator (native.hpp's machinery
  unchanged). The fillet's m4 has registered the same need
  (ADR-0039 D6: the edge-name field, v0.4.0 → v0.5.0); **whichever
  line lands first takes the next number, the other rebases on the
  landed number in the same act — never two bumps the same day.**
  The decision is the register's, at the landing morso, where the
  real cost is visible.
- **The error taxonomy's register** — the `sketch_*` family noted
  beside `fillet_*` and `reference_*` (the culture's amendment note,
  one line, nothing rewritten underneath).
- **docs/status/README.md — the Phase G row advances per morso**
  (the house rhythm), and the "What does NOT exist yet" line for
  the solver flips at m2's landing.

**LA LADDER dei morsi 2+** (each standalone with the full suite
green; STARTING ONLY on the maintainer's review — and the
revisore's — of THIS document; the order by scale — **IL SOLVER
PRIMA; la workbench DOPO; il ponte PER ULTIMO**):

1. **m2 — IL SOLVER** (the heart first): the constraint structures
   (the recipe's data), the graph, the bill, the admission's rank
   test, the damped min-norm Newton with the canonical initial
   point and the canonical order, the bands ε_solve, the three
   classes with their payloads, the degenerate verification on the
   stored bytes, the in-house linear algebra (the one file); the
   bindings' surface (the constraints, the bill, the solve, the
   classes — queryable from Python before the UI consumes, the
   H2-u6 discipline); **every golden of D5 landing green by
   itself** — the eight, red-first on today's tree, each with the
   derived numbers asserted (the script is the pre-registration,
   the m2's tests reproduce its output at the band's margins).
2. **m3 — LA WORKBENCH** (after): the DoF counter on the status
   line (the cold sites only), the solve's feedback (the classes in
   interface), the constraint affordances (the selection-driven
   menu, the registry pattern), the pinning gesture, the rectangle
   macro; the UI smoke suite extended (the gestures' semantics, the
   counter's honesty, the refusal's payload rendered).
3. **m4 — IL PONTE** (last): the format — the constraints array in
   the sketch payload, the version bump (the number per the
   register's arbitration), the companion and the migrator, the
   round-trip goldens (the older links never re-pin); the example
   end-to-end (the guide case with the solve in the chain, the
   printed table, the click-test).

**LE STIME, in the register** (the H2 checkpoint's lesson: measured
granularities, labeled as such, never promises): **m2 ≈ 2 days**
(the solver core + the eight goldens + the bindings — the linear
algebra's one file the smaller half); **m3 ≈ 1–2 days** (the
workbench: the counter, the feedback, the affordances, the smoke);
**m4 ≈ 1 day** (the format's own rite). The tail (the queue's
families) carries NO date — the no-promises list, verbatim from the
mothers.

**The no-promises list:** no code moves in m1 (this document
alone); no fillet file is touched by any morso of this line (the
parallel discipline, D6's arbitration for the ONE shared organ —
the format number); no SSI entry, no boolean, no naming change; the
spline family stays BLOCKED by G3; the 3D sketch stays blocked by
the work planes; the axes/equations/symmetry families carry no
date; no new third-party dependency (the linear algebra in house).

## Consequences

- The register's last standing Phase-F sentence is redeemed with the
  order it was deferred for: the solver lands ON TOP of the
  delivered data model, binding real entity parameters with real
  propagation tests under it — ADR-0030 D1's own thesis, honored to
  the letter.
- The sketch is what it always was — THE ROOT SYSTEM: the genealogy
  born from the author entities, the roots the chain positions, the
  downstream edit the resolver's classes. The solver adds the one
  missing layer — the value-level search — without moving the
  layout, the grammar, the materialization or the format's existing
  links: the proof is D3.3's derivation, and golden 8 is its live
  demonstration.
- The bands it declares are the vocabulary's own: ε_solve's two
  members re-homed from the kernel's existing constants — one value
  per world, the certificate and the solve the two consumers, zero
  drift. The ADR-0035 table gains its row the way the table grew
  before: derived, declared, never a naked constant.
- The honest failures are answers with names: over-constrained
  refused at admission with the culprit, not-solvable exited with
  the bounded residual and the diagnosis, degenerate verified
  against the kernel's own classes BEFORE the recipe — never a
  crash, never a silence, never an approximation passed off as
  solved. The sketch_* family joins the taxonomy beside the
  fillet's and the resolver's.
- The parallel discipline holds its first real test: two lines open
  on main, one organ shared (the format number), the register the
  arbiter, no long-lived branches — the G2 lesson written into the
  method, and this ADR is the first act that practices it.

## References

- ADR-0030 (the sketcher MVP — the deferral redeemed here; D1's
  order thesis; D2's chains and the addressing clause; D5's binding
  specification the workbench extends), ADR-0036 (the naming — the
  roots, the resolver, the amendment D4's rule this ADR inherits
  unamended), ADR-0035 (the vocabulary — the Solve row's amendment;
  D2.1's one-value-two-consumers pattern re-homed), ADR-0014 (the
  determinism — the canonical initial point, the store's quantizer),
  ADR-0032 (G1: the entity set, the certificate, the prism formula,
  the guide case), ADR-0031 D5 (the scale method), ADR-0033/0039
  (the error culture's families; the parallel line's contact
  points), ADR-0007/0027 (the format rule the bump obeys).
- The code sites, verified at b704495: core/sdf/include/CAD_0/sdf/
  sketch_entities.hpp (the entity payloads, the construction
  checks), sketch_profile.hpp (the certificate, the named
  tolerances ε_solve re-homes: kJointCoincidenceTol, the pairwise
  check, the bounded closure), sketch_extrude.hpp; core/brep/
  include/CAD_0/brep/naming.hpp (the roots' grammar: `sk#F:<i>`,
  `sk#F:face`), resolve.hpp (the four classes), descent.hpp;
  core/scene/include/CAD_0/scene/scene.hpp (the sketch payload, the
  recipe's role list); core/io/src/native.cpp (the v0.4 sketch
  payloads the constraints array extends); app/src/CAD_0/ui/
  sketch_canvas.py (the gestures, the suggestions, the status line),
  property_panel.py (the registry pattern); examples/sketch_hole.py
  (the parametric hole, the guide case end-to-end);
  tests/unit/sdf/test_sketch2d_*.cpp (the kernel's own goldens the
  solver must not move).
- scripts/adr0040_derivation.py — THE DERIVATION, this ADR's
  pre-registration: the DoF counts, the consumptions (the numerical
  ranks), the bills, the eight goldens' numbers, the bands' margins,
  the iteration counts. Verified in double; the m2's tests
  reproduce its output at the bands' margins.
