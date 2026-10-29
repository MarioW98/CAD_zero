# ADR-0034: H1 — the arithmetic boundary of the exact kernel

## Status

Accepted (2026-10-29). Daughter of ADR-0004 (the typed tolerance
model — whose Context already named "a B-Rep stored in `double`" as
the world to convert into), of ADR-0014 (determinism — the quantizer
this ADR reuses at the display boundary), of ADR-0031 (the exact
kernel fork, road (a)), of ADR-0032 (the G1 spine whose Layer-2
provenance this ADR makes arithmetically honest) and of ADR-0033
(G2 — whose unit 1, the SSI table, is the first consumer of the
boundary this ADR pins). **Remediation before progression**: this
ADR lands BEFORE G2 unit 2 (the extended booleans) resumes, because
the booleans consume the SSI table's answers — the table's
arithmetic is their foundation.

## Context

An external verification found a gap between the register and the
reality, of the class the house treats as **priority zero** — for a
project whose identity is "the register never lies", the gap itself
is the defect, whatever the code does:

1. **The predicates were not what the register said.**
   `third_party/robust_predicates/predicates.c.h` was a hand-written
   NON-ADAPTIVE stub that imitates the structure of Shewchuk's file
   (`exactinit`/`splitter`/error bounds declared, dead code left in,
   a former project's name in the header, a README instructing a
   `curl`-overwrite of a tracked file), while README and ADR-0031
   said "vendored Shewchuk robust predicates". No expansion, no
   adaptive filter existed in the file; the sign of a near-degenerate
   determinant was only as good as plain double arithmetic — the
   one thing robust predicates exist to beat. The license audit
   (docs/audits/2026-10-16-third-party-license-audit.md) was the one
   record that said the truth ("il file in-tree è uno **stub Phase A
   scritto dal progetto**"); the README and ADR-0031 D1/D4.1 did
   not.
2. **The exact path computed in float32.** `core/brep/src/ssi.cpp`
   (the ADR-0033 D1.1 table — the analytic SSI engine G2's booleans
   consume) did its derivations in `math::Vec3f`/`float` — radius,
   offsets, the radical plane's height h = (D²+R₁²−R₂²)/2D, all in
   single precision — while the Layer-2 provenance field (ADR-0032
   D2.4) answered **exact** for those numbers. Single precision
   carries ~7 significant digits and up to ~1e-7 relative error PER
   OPERATION, cascading through the closed forms: the *derivations*
   are exact mathematics, but the *bytes* the table answered were
   float32 roundings of them. "exact" declared more than single
   precision guarantees. (The mass machinery — ADR-0032 D4 — was
   already double internally, with one declared float cast at the
   pinned answer struct; the SSI table and the exact
   factories/subtract/extrude were not.)
3. **The dispatch was stringly-typed.** The SSI table dispatched on
   `family == "plane"` (a runtime string compare) followed by
   `dynamic_cast` chains: a typo in the string compiles and fails at
   runtime — exactly the silent-drift class the house refuses
   everywhere else.

The register's claims are corrected in the same docs commit that
lands this ADR (README, ADR-0031 D1 and D4.1, ADR-0032 D7 pipeline);
the remediation of the code lands in the two feat commits that
follow it (the vendoring of the authentic Shewchuk robust.c; the
double migration of the exact path with the typed dispatch and the
one-time golden re-pin). The method is the house's own: derivations
correct the register, never the reverse; every finding named, never
silent.

## Decision

### D1. The arithmetic boundary — DOUBLE in the exact path, FLOAT at the named boundaries

**The exact B-Rep path is the double domain.** Every arithmetic
operation, classification, predicate and closed form of the exact
world — the SSI table, the exact factories, the exact extrude and
subtract, the polygon booleans, the mass machinery, the robust
predicates themselves — computes in IEEE 754 double, and every
geometric payload the exact world STORES or ANSWERS (vertex
positions, analytic surface parameters, 3D curve payloads, SSI
answers, mass answers) is stored and answered in double.

**FLOAT survives only at named boundaries, one direction each,
declared at the site:**

| Boundary | Direction | Where it lives |
|---|---|---|
| The p-curve vocabulary | double → float32 | The trimming data IS the sketch kernel's typed entity set (ADR-0032 D2.1's one-opinion pin: `PCurve == sdf::SketchEntity`, forked vocabularies "stop compiling"). The sketch kernel is the float domain; the exact path converts at the p-curve authoring site, once, explicitly. |
| Display / mesh | double → float32 | `tessellate_*` output (`sdf::TriangleMesh` positions/normals are the SDF world's float pipeline — its domain, untouched, ADR-0002/0015's split). The renderer and the ray marcher read floats, as ever. |
| The faceted world in | float32 → double | The bridge authors `approximated(tol)` bodies from the mesh's float triangles. The promotion is lossless (every float32 is exactly a double), so no approximation is added — the declared tolerance carries the honesty. |
| Serialization | double → text | Per ADR-0014: output files quantize to 12 decimal digits; the native format serializes recipes (the B-Rep is cache, ADR-0027) — unchanged. |

The boundary is DERIVED, not stylistic: the SDF display path, the
tessellation grid and the sketcher are float pipelines whose
consumers (SIMD batches, the GPU mesh, the 2D canvas) are sized for
float — killing them on principle would be the mirror-image lie
(display data does not need double's bits, and ADR-0014's
determinism bar is met by quantization, not precision). Symmetrically,
the exact world's consumers — the booleans' classification bands
(ADR-0004's explicit tolerances are DOUBLE-typed already), the mass
closed forms, the SSI derivations with their cancellation-prone
subtractions (the radical plane's h, the mid-plane's √(r²−d²/4)) —
lose real digits in float32. The register said "exact"; only double
makes the word mean what it says.

### D2. The predicates — vendored authentic, adaptive, discriminated

`third_party/robust_predicates/` carries the **authentic
`robust.c` by Jonathan Richard Shewchuk (May 18, 1996, public
domain)** — the adaptive implementation: error-bound filters over
exact expansion arithmetic (Two_Sum/Two_Diff/scale_expansion and
friends), `orient2d/3dadapt`, `incircleadapt`, `insphereadapt`. The
provenance header names the source, the revision, the license, the
retrieval chain (the CMU original no longer resolves; the text is
vendored from a faithful mirror — named in the file) and **every
local modification** (the C++ adaptation: K&R definitions →
prototypes, the one mirror-added include removed — nothing else).

The wrapper contract (`core/math/predicate.hpp`) is UNCHANGED —
the same signatures, the same initialize discipline; the consumers
see new numbers only where the old ones were wrong (near-degenerate
inputs, where the stub's naive determinant and the adaptive
predicate disagree — by construction).

**The discriminating tests are the vendoring's proof**
(`tests/unit/math/test_predicate_adaptive.cpp`): the classic
near-degenerate families — points nearly collinear at ulp scale,
`orient3d` nearly coplanar, `incircle` nearly cocircular — pinned
with the exact vectors that build them, each pinned with the
ADAPTIVE answer's sign, and with the **negative block**: the same
inputs through the historical stub's naive double determinant,
CHECKED to fail (the wrong sign) where the vendored predicate
passes. The discrimination — naive fails, adaptive passes, on the
same pinned bytes — is the evidence the file in the tree is the
real thing; a non-adaptive impostor cannot pass the block by
construction.

### D3. The typed dispatch — a typo stops compiling

`SurfaceGeometry` gains `family()` — a typed `SurfaceFamily` enum
(None / Plane / Cylinder / Sphere / Cone / Torus / Unknown) — and
`accept(SurfaceVisitor&)`, the classic visitor hook: each concrete
surface forwards `*this` to the visitor's typed overloads. The
stringly-typed regime ends:

- **The SSI table dispatches on types**: the operand records are a
  `std::variant` of typed payloads (plane/sphere/cylinder/cone/
  torus/monostate) built through the visitor, and the entry
  selection is a `std::visit` overload set — the landed pairs are
  overloads on the REC TYPES, the wall is the generic fallback.
  A typo'd family name is now a **compile error** (no matching
  overload), not a runtime refusal.
- **The string `type_name()` stays for MESSAGES** — the refusals,
  the report, the query records speak family names — but it is
  DERIVED from `family()` through one switch: one naming
  authority, as ADR-0032's query contract demands, now
  typo-proof (the enum is the authority; the string cannot drift
  from it).
- Every site the double migration touches (tessellate's parameter
  windows, the mass machinery's per-family closed forms, the
  subtract's class checks, translate_body's table) switches from
  `std::string(type_name()) == "..."` + `dynamic_cast` chains to
  the enum + visitor — same defect class, same fix, one delivery.

### D4. The golden re-pin — one commit, declared, counted

The migration moves **geometric bytes**: vertex positions,
surface parameters and SSI answers that were float32 roundings
become the double values the closed forms actually produce. The
goldens re-pin **once, in the feat(brep) commit that migrates
them** — the commit body declares it. The culture's invariants
around the re-pin:

- **Topological counts do not move.** The topology is integer —
  V/E/F/H, χ, genus, the radial-edge XOR — and the migration does
  not touch a single count (the tests that pin counts must stay
  green without re-pinning).
- **The determinism discipline (ADR-0014) survives FROM the
  re-pin**: after the feat(brep) commit, regeneration is
  bit-identical from the new bytes; every digest oracle
  (rebuild-and-compare) must pass unchanged, because it compares
  a build against a build, never against the pre-migration bytes.
- **The re-pin is not a license**: a golden whose VALUE changes
  changes by derivation — the new expected bytes are the closed
  form computed in double, checked against the classical formula
  at double tolerance, and the old float bytes are gone. No
  golden is relaxed (a tolerance widened to pass is a laundering,
  and the tests' job is to prevent exactly that).

### D5. Amendments to the series

- **ADR-0014** — the 12-decimal quantizer gains its second named
  use: the double→float conversion at the display/p-curve
  boundaries is the value's quantizer (float32's 24-bit mantissa
  IS the coarsest quantizer in the chain); serialized text keeps
  the 12-decimal rule; internal doubles are never quantized.
  Amended in place.
- **ADR-0032 D4** — the mass answer's storage (double volume,
  float centroid/inertia) is amended to full double: the pinned
  struct's float fields were the last float store INSIDE the
  exact path. Amended in place; the closed-form values the tests
  pin are unchanged (the tests read them through double
  tolerances already).
- **ADR-0032 D2.1** — NOT amended: the p-curve vocabulary stays
  the sketch kernel's float types, one opinion, fork forbidden.
  This ADR adds the declared conversion at the authoring site,
  not a second vocabulary.
- **ADR-0031 D1/D4.1, README** — corrected (not amended): the
  "vendored Shewchuk" claims were false of the stub; the record
  now says what was there, and the vendoring commit makes the
  sentence true going forward.
- **ADR-0033 D1.1** — the table's entries compute in double per
  this ADR; the ADR's derivations are unchanged (they are
  mathematics, not arithmetic); the re-pinned goldens are the
  double answers.

### D6. What H1 does NOT do

- **No SDF change**: the display pipelines, the field evaluation,
  the MC33 extraction, the vertex welding, the twist warp — their
  float world is their domain (ADR-0002's split), untouched.
- **No format change**: the native format serializes recipes; the
  B-Rep is cache (ADR-0027); no version bump, no migrator.
- **No tolerance change**: ADR-0004's model — explicit, typed,
  double-valued — is already the boundary's tolerance discipline.
- **No CI change**: the hygiene delivery (runner 24.04, the
  Windows column, the bindings ASan, versioning, the synthetic
  README, the root cleanup) is the NEXT delivery and is NOT in
  this one.

## Consequences

- "exact" means what it says: closed-form mathematics COMPUTED in
  double by an adaptive-predicated, typo-proof-compiled exact path.
- The boolean unit (G2 unit 2) reopens only on a later order, on
  top of this foundation — its inputs are now double SSI answers
  with provenance exact that the bytes actually support.
- The maintenance cost is one remembered rule: the exact path is
  double; floats appear only where this ADR's boundary table names
  them, one direction, declared at the site. A new float inside
  core/brep's exact arithmetic is a review-blocking regression,
  not a style choice.

## References

- ADR-0004 (tolerances, typed and double — the Context's own
  "B-Rep stored in double" realized), ADR-0014 (determinism, the
  quantizer), ADR-0002 (the dual kernel's domain split — why the
  SDF keeps floats), ADR-0031 (the fork; the corrected D1/D4.1
  claims), ADR-0032 (the Layer-2 provenance this ADR makes
  arithmetically honest; D2.1's one-vocabulary pin; the amended
  D4), ADR-0033 (the SSI table — the first consumer), ADR-0021/
  0022 (the integer topology the re-pin does not touch),
  ADR-0023 (the seam-wire pattern), ADR-0027 (recipes as the
  truth — why no format bump).
- Shewchuk, J. R. (1996). Adaptive Precision Floating-Point
  Arithmetic and Fast Robust Geometric Predicates. CMU-CS-96-140.
  (The vendored file's own paper — the expansion arithmetic and
  the filters the discriminating tests exercise.)
- The license audit that said the truth when the register did not:
  docs/audits/2026-10-16-third-party-license-audit.md.
