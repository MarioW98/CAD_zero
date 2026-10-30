# ADR-0035: H2 unit 3 — the numerical taxonomy: the three families, the derived bound, the p-curve frontier

## Status

Proposed (2026-10-30) — delivered as **morso 1 of H2-u3, docs-only**:
the contract is derived and pinned HERE; no code moves in this
delivery. The implementation is morsi 2-3, and **morso 2 starts
only on the maintainer's review of this document** — the STOP the
order pins. Until acceptance, every amendment this ADR declares
(D6) stays a proposal: the mothers stand as written.

Daughter of ADR-0004 (the typed tolerance model — completed here
with the Parameter kind), of ADR-0014 (determinism — the quantizer;
the re-pin discipline D5 pre-declares), of ADR-0031 D3 (the three
layers — the bands D3 populates), of ADR-0032 D2 (the p-curve
vocabulary — re-pinned per world in D4), of ADR-0033 (the SSI
table whose double answers the booleans consume), and of ADR-0034
(the arithmetic boundary — extended in D4 to the last float store
inside the exact path: the parametric domain).

**Taxonomy before progression**: this ADR lands before the
booleans' remaining units meet a threshold, because a threshold
without a derivation is a decision the register cannot audit —
the H2-u1 lesson (the gates went relative to body scale) said it
once for the class gates; this ADR says it for every constant
the exact path compares against.

## Context

The state, verified against the tree at f691a98 (the repo is the
authority; the working tree is clean on the last green — nothing
half-done to fold in, nothing to roll back):

1. **H1 (ADR-0034) is complete and green**: the authentic adaptive
   predicates are vendored (`third_party/robust_predicates`), the
   wrappers answer in `core/math/predicate.hpp`, the exact path
   computes and stores in double.
2. **The H2-u1/u2 gates are green**: the input contract validated,
   invalid results refused — and the class gates are already
   RELATIVE to the body's scale (kVolumeRelTol, kAreaRelTol,
   boolean.cpp:52-59): the first taste of this taxonomy, landed
   as an engineering fix before the taxonomy existed.
3. **The two verification questions, answered from the tree:**
   - `core/brep/src/boolean.cpp` — the three constants are still
     there (lines 47-49): kWeldGrid = 1e-6, kPlaneTol = 1e-6,
     kEps = 1e-9. And **kEps serves THREE dimensional worlds at
     once**: a determinant threshold (|denom| > kEps, |cross| >
     kEps — length²), a parameter-domain tolerance (t > kEps,
     t < 1−kEps, u > kEps, t_hi−t_lo < kEps — dimensionless), and
     a bare length (len < kEps, len_sq < kEps² — mm and mm²). One
     constant, three worlds: the dimensional chaos this ADR ends.
   - `core/brep/src/ssi.cpp` — the p-curves still authorize by
     demoting double→float at the authoring sites (the
     `static_cast<float>` block; the header declares the boundary
     per ADR-0034 D1's table row). The same demotion lives in the
     factories (`factory.hpp` `to_domain`) and in the extrude's
     mirrored cap re-authoring. **The parametric domain (t,u) is
     the last float store inside the exact path's inputs**: the
     trimming decisions run on float bytes while every consumer
     downstream of them is double.
4. **Why now**: the booleans' remaining units consume the SSI
   table's answers and the trimming windows; every comparison they
   make must be exact, typed, or banded — never a bare constant.
   The fixed kEps = 1e-9 decides topology on bytes the data cannot
   support (the crossover derivation in D2: at every model scale
   the authored class reaches, the fixed constant is tighter than
   the honest band by four to six orders of magnitude).

## Decision

### D1. Family 1 — topological predicates: the exact sign, no tolerance

Every orientation and critical relation of the STORED bytes goes
through the adaptive predicates — `orient2d/3d`, `incircle2d`
(the vendored robust.c, the `core/math` wrappers). The sign is
EXACT: −, 0, + of the stored doubles, as written. Exact zero is a
fact (a true degeneracy of the bytes) and branches on the
degenerate case. **No epsilon, no band, no tolerance enters a
predicate call — ever.**

What family 1 never answers: whether two entities from different
provenance are the SAME entity. That is equivalence (family 2) —
stored bytes claim to represent the geometry only to a declared
trust radius, and a sign finer than that radius is arithmetic, not
geometry. The family boundary in one sentence: **the predicate
answers the sign of the bytes; the threshold answers the honesty
of the data; the band answers the ordering of the ties — and the
three jobs never share a constant.**

### D2. Family 2 — equivalence thresholds: typed per operation, dimensionally homogeneous

Two entities are coincident when their distance, in the metric of
their world, is under a threshold derived from the operation's
declared tolerance. The worlds, typed once:

| Decision | Kind | Threshold | Derivation |
|---|---|---|---|
| welding, planarity, degenerate-length guards | **Linear** (mm) | ε_L, per operation — kWeldGrid and kPlaneTol are this kind; the pin is the KIND and the per-operation declaration (ADR-0004's model; the values stand) | declared (the house defaults) |
| parameter windows, endpoint exclusions, overlap widths in (t,u) | **Parameter** (domain units) | ε_P, per operation — a SEPARATE typed tolerance; NEVER the linear constant, never one constant for both worlds | declared per operation; MAY be derived per surface family through the family's declared Jacobian (a named derivation in the family's own terms — never a silent share) |
| the segment determinant: parallel / collinear admission | length² | **the derived band, D2.3** | first principles, D2.3 |
| parallelity of normals (kParallelDotMin, kNormalParallelTol) | **Angular** (dimensionless by construction) | typed Angular tolerances, declared | declared (the loose pre-filter stays loose, the strict threshold stays strict — both typed) |
| the division guard (kIsectDenomEps) | none — NOT a tolerance | stays a numeric guard, declared as such | engineering (the comment at boolean.cpp:57-60 already says it; ratified as a rule) |

**D2.1 The Linear world** is ADR-0004's own, unchanged in kind:
one ε_L per operation, explicit at the signature, no global
epsilon. The welding grid doubles as the canonical key's
quantizer (D3) — one declared value, two consumers, zero drift.

**D2.2 The Parameter world** gets its own kind. ADR-0004's
`ToleranceKind` gains **Parameter**: the (t,u) domain's own kind,
compared in domain units, declared per operation. A plane's (u,v)
are lengths in the plane's frame; a cylinder mixes an angle and a
length; a sphere's are angles. The family declares its Jacobian
(its parametrization is derived state — ADR-0002/0031 doctrine),
and an operation MAY map its linear tolerance back through that
Jacobian into a parameter tolerance — as a NAMED derivation, per
family, in the family's terms. What is banned is the bare share:
the same constant answering the linear question and the parameter
question, which is exactly what kEps does today (boolean.cpp:321,
349, 352, 359-364 — all parameter decisions — read the same 1e-9
that lines 317/332 read as a determinant bound and line 937 reads
as a length).

**D2.3 The determinant bound — derived from first principles, not
estimated.** For the segment pair with d₁ = p₂−p₁, d₂ = p₄−p₃:

    det = d₁ × d₂        (2D cross; dimension length²)

det is bilinear in the direction vectors and scales as the square
of the geometric scale. Each operand point is trusted to ε_L (the
operation's LINEAR tolerance — the same world as the welding
band): |Δd₁| ≤ 2ε_L, |Δd₂| ≤ 2ε_L, so

    |Δdet| ≤ |Δd₁||d₂| + |d₁||Δd₂| + |Δd₁||Δd₂|
           ≤ 4·ε_L·L + 4·ε_L²

with L the domain extension (the characteristic length of the
operand pair: the maximal segment length / the domain's span).
At every scale the class admits, ε_L ≪ L — the H2-u1 gates
already refuse near-flat bodies — and the second-order term is
negligible:

    **the parallelism band:  |det| ≤ 4·ε_L·L**

Inside the band the pair is equivalent-to-parallel: a crossing
the bytes suggest is not supported by the data's declared
honesty, and the admission goes to the band machinery (D3).
Outside it, family 1 governs: the sign is exact, and the crossing
point is computed in double. The same derivation covers the
collinearity cross (p₃−p₁)×d₁ and the t/u numerators — the same
bilinear form on the same points — but once the crossing is
admitted, the t/u WINDOW decisions belong to the parameter world
(ε_P, D2.2), never to the determinant's.

**The scale honesty, derived.** Under uniform scaling by s: det
scales as s². The band 4·ε_L·L scales as s² exactly when ε_L is
declared relative to the model scale — the form the H2-u1 gates
already chose ("relative, so the gate means the same thing at
every model scale"). The decision is then scale-invariant. With
today's absolute ε_L = 1e-6 mm the band still grows as s (through
L) while det grows as s² — the declaration, not the derivation,
sets the scale behavior; the derivation only makes the bill
visible. **The pin: the booleans' thresholds declare their ε_L the
way the gates declare theirs — relative where the contract demands
scale invariance — and the band follows.**

**The crossover, derived — the honest bill of the fixed kEps.**
kEps = 1e-9 equals the honest band 4·ε_L·L at L = 2.5·10⁻⁴ mm
(ε_L = 1e-6). Therefore:

- **Above 0.25 µm — every model the authored class reaches**: the
  fixed constant is tighter than the honest band by 4·10⁴ at
  10 mm and 4·10⁶ at 1 m. The code decides topology on arithmetic
  noise: spurious micro-crossings, counts unstable under
  byte-level perturbation of the input.
- **Below 0.25 µm — unreached today**: the failure inverts —
  genuine crossings read as parallel, intersections are lost.

Both ends are the same defect: **a threshold without a
derivation.** The re-homing map for morsi 2-3, use by use
(boolean.cpp at f691a98):

| Site | Today | Re-homes to |
|---|---|---|
| :317 | \|denom\| > kEps | the derived band 4·ε_L·L |
| :321 | t, u vs kEps | ε_P (parameter) |
| :332 | \|cross\| > kEps | the derived band (same form) |
| :339, :377 | len_sq < kEps² | ε_L² (a length decision: the segment is shorter than the linear tolerance) |
| :349, :352, :359-364 | t_hi−t_lo, t_lo vs kEps | ε_P (parameter) |
| :937 | len < kEps | ε_L (a length decision) |
| kWeldGrid (all sites) | 1e-6 bare | Linear, typed, declared — and the canonical key's quantizer (D3) |
| kPlaneTol (all sites) | 1e-6 bare | Linear, typed, declared |
| kParallelDotMin, kNormalParallelTol | bare doubles | Angular, typed, declared |
| kIsectDenomEps | 1e-12 | stays a numeric guard — never a geometric decision |

### D3. Family 3 — the bands: deterministic tie-break on the canonical key, golden-pinned

ADR-0031 D3 Layer 3 pinned the discipline; this ADR inherits it
and names the booleans' instances. Every classification that
falls inside a declared band — the derived determinant band, the
linear welding band, the parameter window band — resolves by
DETERMINISTIC TIE-BREAK on the canonical key: **the existing
pattern** (ADR-0016/0025; in the tree, SnappedKey/snap,
boolean.cpp:69-95 — grid-quantized int64 keys, ordered,
input-order-independent). The canonical key's quantizer IS the
operation's linear tolerance: the tie-break and the equivalence
threshold are ONE declared value — never two sources that can
drift. Never a silent coin flip on a float comparison; the
goldens pin each banded decision (the adversarial corpus of
ADR-0031 D4.2), and determinism (ADR-0014) survives: same bytes
in, same bytes out, bit-identical, even inside the bands.

### D4. The p-curve decision — two roads compared, honest costs, the proposal registered

The question: the parametric domain (t,u) — the p-curve payloads
and every window they feed — is stored float today. The register
says so honestly (ADR-0032 D2.1: the p-curve vocabulary IS the
sketch kernel's typed entity set; ADR-0034 D1's boundary row: the
exact path demotes double→float at the authoring sites, once,
declared — ssi.cpp's `latitude_pc`/`vline_pc`/`sketch_circle`
block, factory.hpp's `to_domain`, the extrude's mirrored cap
re-authored by the 2D kernel in float). The exact path that
CONSUMES the trimming decisions is double. The demotion is a
quantizer (ADR-0014's amendment says so), not a tolerance: it
moves the data — up to ~6·10⁻⁸ relative per value — without
declaring a band, and it is larger than any honest ε_P.

**Road (a) — parallel 2D types in DOUBLE for the BREP parametric
domain, plus the conversion contract at the sketch frontier.**

- The BREP's p-curve payloads become double: the SSI authors in
  double and STORES double (the demotions are removed, not
  converted); the window queries (`face_v_window` and kin) read
  double; the factories' `to_domain` stays double — no cast at
  all.
- **The sketch frontier**: the extrude caps ARE sketch entities —
  extrude.hpp says it in the tree ("the caps' outer loop IS the
  chain itself, entity for entity"). The sketch kernel stays
  float: its world is the canvas, the UI, snapping — display-scale
  consumers (ADR-0002's domain split). At the frontier the
  conversion is **float→double: EXACT** (every float32 is exactly
  a double — no rounding, no band, no quantizer), **ONE
  direction, declared at the site**. The lossy direction
  (double→float) disappears from the exact path entirely.
- **One opinion PER WORLD.** The sketch world keeps its ONE
  vocabulary (its float entities — the sketcher's own); the BREP
  parametric domain gains its ONE vocabulary (the double 2D
  types, built on `math::Vec2d`, which core/math already carries).
  The semantics are mirrored and declared: the same
  construction-time validation rules, the same region-side
  convention, the same CCW-store discipline — the mirror is
  named in the types' contract, and the type-identity
  static_asserts (pcurve.hpp) become per-world asserts. ADR-0032
  D2.1's anti-fork rule survives in its essence: never TWO
  implementations of the same validation in the SAME world.
- **Costs, honestly**: one more typed 2D vocabulary to maintain
  (bounded: segment/circle/arc in double); the register surgery
  (D6, two amendments); the golden re-pin of the p-curve payload
  bytes (ONCE, declared — D5); the frontier conversions at the
  extrude cap authoring sites (lossless, mechanical). NO changes
  to the sketch kernel, the canvas, the UI, the serialization:
  the sketch recipes keep serializing their floats, the native
  format never stored p-curves (the B-Rep is cache, ADR-0027) —
  no format bump.
- **What it buys**: the parametric domain joins the exact path's
  double world — ADR-0034 D1's own principle ("every geometric
  payload the exact world STORES or ANSWERS is stored and
  answered in double") carried to the payload it exempted. The
  carving-out was the D2.1 one-vocabulary pin; this ADR closes
  the exception by splitting the DOMAINS, not by forking the
  validation. And D2's parameter taxonomy becomes enforceable:
  an ε_P on float payloads would be a lie (the float noise is
  bigger than any sane parameter band); on double payloads it is
  a contract.

**Road (b) — migrate the 2D sketch kernel to double.**

- The sketch entities become double; the p-curve vocabulary stays
  ONE (the sketch's), now double; the sketch→BREP frontier
  disappears (double→double, nothing to convert).
- **Costs, honestly**: the radius of action is the whole 2D world
  — the sketch kernel (core/sdf's entities, profile, extrude),
  the canvas and UI (PySide6 float pipelines: snapping, grid,
  gestures, the property panel, the sketch tools — every
  interactive path the UI tests pin), the serialization of the
  sketch recipes (floats today; the bytes change and ADR-0007's
  versioning answers), the SDF bridge's profile evaluation, the
  Python bindings' surface — PLUS everything road (a) still does
  at the SSI/factory authoring sites: the demotions must stop
  regardless; road (b) does not remove that work, it adds to it.
- **What it buys over (a)**: one vocabulary instead of two
  mirrored ones; no frontier contract to maintain; the sketcher's
  authored data gains double precision — which its consumers
  cannot perceive: the canvas is a display-scale world (no eye
  resolves a 16th digit at any zoom), and the determinism bar is
  unchanged either way (the sketch world's serialization
  quantizes to 12 decimals — floats and doubles serialize the
  same, ADR-0014).
- **The honest counterweight**: the sketch world's precision is
  not the bottleneck of any exact-path consumer. The frontier
  conversion in (a) is lossless — the sketch's floats enter the
  BREP exactly as authored. Doubling the sketch kernel solves a
  problem the exact path does not have, at the price of
  auditing every interactive path.

**The derivation: road (a).** The exact path's honesty demands
the parametric domain be double; the sketch world's consumers
are display-scale and stay float; the frontier between them is
one named site (the extrude cap authoring), one direction
(float→double), lossless by construction. The fork is NOT big on
both banks — (a) is bounded and buys the exact path's coherence;
(b) is systemic and buys nothing the exact path consumes — so
the proposal is REGISTERED here, not escalated: if the
maintainer's review disagrees, the amendment is one commit on
this document, before any code moves.

### D5. The golden declaration — written BEFORE the code

Pre-registered for morsi 2-3, the ADR-0034 D4 protocol applied
verbatim:

- **If the bites move geometric bytes — they will**: the derived
  determinant band re-admits parallelisms the fixed kEps denied
  (and denies crossings it admitted); the p-curve payloads change
  from float roundings to the double values the closed forms
  produce. The goldens re-pin **ONCE, in the feat commit that
  moves them, the reason in the commit body**.
- **Topological COUNTS and IDENTITIES do not move.** V/E/F/H, χ,
  genus, the radial-edge XOR are integer facts (Layer 1): the
  tests that pin counts must stay green WITHOUT re-pinning. A
  count that moves is a defect, not a golden — the migration
  does not get to change topology, only the bytes.
- **No golden is relaxed.** A tolerance widened to pass is a
  laundering (ADR-0034 D4's own words, ratified).
- **Determinism FROM the re-pin**: bit-identical regeneration
  (ADR-0014); the digest oracles (rebuild-and-compare) pass
  unchanged — they compare a build against a build, never
  against the pre-migration bytes.
- **Why the declaration lives here, before the code exists**: so
  the re-pin is the execution of a registered decision, never an
  after-the-fact accommodation. The counts-don't-move clause is
  what makes the re-pin reviewable: the reviewer diffs the
  goldens, sees only bytes move, and approves on the strength of
  a rule that existed before the diff.

### D6. The register surgery — declared; applied in the same act as acceptance

Until this ADR is accepted, the mothers stand as written; THIS
document is the only record of the proposed amendments. On
acceptance they apply in the same act (the acceptance commit):

- **ADR-0004 — amended**: `ToleranceKind` gains **Parameter** (the
  (t,u) domain's own kind); the Defaults table gains the
  parameter default; the kind-check discipline extends to it.
  Amended in place, dated, pointing here.
- **ADR-0032 D2.1 — amended**: the p-curve vocabulary becomes
  one-opinion-PER-WORLD (the sketch's float vocabulary stays the
  sketch world's; the BREP parametric domain's double vocabulary
  is its own, mirrored semantics declared). The anti-fork rule
  survives; the static_asserts become per-world. Amended in
  place, dated, pointing here.
- **ADR-0034 D1 (the boundary table's p-curve row) — amended**:
  the boundary reverses to the sketch frontier — float→double,
  lossless, one direction, at the extrude cap authoring sites;
  the double→float authoring-site demotions are REMOVED (the SSI
  and the factories store double). Amended in place, dated,
  pointing here.
- **ADR-0014 — NOT amended**: the quantizer rule already covers
  the new boundary (float→double needs no quantizer — it is
  lossless; the 2026-10-29 amendment stays as written).

### D7. What this unit does NOT do — the no-promises list

- **No code moves in morso 1**: this delivery is docs-only (this
  ADR). Morsi 2-3 implement, only on the maintainer's review of
  THIS document — the STOP the order pins.
- **No boolean algorithm changes**: the taxonomy re-homes the
  DECISION CONSTANTS (typed, derived, banded); the algorithms —
  GH, the SSI table, the assembly — are untouched.
- **No SDF, sketch, canvas, UI, or serialization changes**: road
  (a) leaves the sketch world untouched; the sketch kernel stays
  float.
- **No new SSI entries, no format bump, no CI change.**
- **The H2-u1/u2 gates stand as they are** — already relative,
  already green at f691a98.

## Consequences

- The booleans' remaining ladder consumes this taxonomy: every
  threshold they meet is exact, typed, or banded — and every one
  of the three answers carries its own rule, one remembered
  sentence per family.
- The kEps era ends with its chaos named and every use re-homed
  (D2's map); the register never lied about it (ADR-0034 declared
  the demotions honestly) — this ADR is the honest retirement,
  not an exposure.
- The p-curve payloads stop being the exact path's float
  exception; the parametric domain's taxonomy (D2.2) becomes
  enforceable on data that can actually carry it.
- The review cost of morsi 2-3 is bounded by D5's declaration:
  goldens diff as bytes-only, counts never move, and the
  re-pin's reason was registered before the code existed.

## References

- ADR-0004 (the typed tolerance model — completed here),
  ADR-0014 (determinism, the quantizer, the re-pin discipline),
  ADR-0016/0025 (the canonical-key tie-break pattern), ADR-0031
  D3 (the three layers — the bands), ADR-0032 D2 (the p-curve
  vocabulary; D2.1 amended here), ADR-0033 (the SSI table), and
  ADR-0034 (the arithmetic boundary; D1's p-curve row amended
  here).
- The code sites, verified at f691a98: boolean.cpp:47-95 (the
  constants, the canonical key), :300-400 (segment_intersection,
  the kEps worlds), :937; ssi.cpp:11-12, 35-64, 260-268, 450-458,
  782-841 (the authoring-site demotions, the float windows);
  factory.hpp:59-93 (to_domain); extrude.hpp:3-7, 120, 139 (the
  caps ARE the chain); core/math vec.hpp:139 (Vec2d exists);
  predicate.hpp (the wrapper contract).
- Shewchuk, J. R. (1996). Adaptive Precision Floating-Point
  Arithmetic and Fast Robust Geometric Predicates. CMU-CS-96-140.
- The H2-u1 lesson, in the tree: boolean.cpp:52-59 — the gates
  went relative before the taxonomy had a name.
