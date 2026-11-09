# ADR-0042: K1 m3 — the ellipsoid: the first family to enter under the protocol

## Status

**Accepted (2026-11-09) — the probe unit's own opening act.** The
candidate set was pinned by ADR-0041 D4 (the ellipsoid, or the surface
of revolution on a general straight generatrix — the skew seat), the
choice was made at the opening and named in the register (K1 m2's
delivery report, recorded in the census section: **THE ELLIPSOID**),
and the maintainer's instruction opened this unit with the landing
itself demanded — *«la prima famiglia nuova entra col protocollo»*.
This ADR is the derivation that instruction's own discipline requires:
the family's six protocol rows derived in the open, the entry priced
against the torus baseline's six units (ADR-0041 D3) **BEFORE the
code**, landing only on the pinned success (ADR-0041 D4): the census
row COMPLETE, the delta measured, the golden under the independent
oracle.

No derivation was pre-written — not by ADR-0041 (D4's own sentence),
not by the register's m2 row (the price list names the PATTERNS, never
the mathematics): this document is where the mathematics happens, on
the record, judged by the rite on both compilers like every morso.

Daughter of **ADR-0041** (D1.1 the seven columns this entry must close;
D1.5 the independent oracle — the golden corpus is BY CONSTRUCTION not
independent, the closed forms qualify because they pre-date the kernel;
D2 the six-row protocol the family enters through, the two legal axes,
the banned third — no per-case algorithm exists in this entry, and none
is needed: the typed rows close in closed forms, the rest are walls);
of **ADR-0033** (D2's precision law — option (i) the wall for the
oblique sections and the quartic pairs, each refusal with the pair
named; **option (ii) NOT FIRED**: nothing in this entry asks for an
algorithm, so the registered gate stays shut — «prima il muro, la siepe
quando un caso vero la domanda»); of **ADR-0032** (the frame rule,
∂u×∂v the outward, right-handed by construction — the seed the sphere's
own (pole, azimuth) form planted); of **ADR-0039** (D1.2/D3 — the torus
baseline this entry is priced against, the mass recurrence this entry
consumes); of **ADR-0014** (the determinism clauses — the new forms ride
the existing recurrence's fixed orders, zero new accumulation paths);
of **ADR-0007/0007a** (the version arithmetic at the bump this entry
pays); and of the **register itself** (the census table's seat, the K
row that counts this morso, the sphere's serialization dash whose
pattern this entry redeems).

## Context — the seat, the choice, and what the sphere's row already prices

The census (K1 m2, 2026-11-08) counted today's tree honestly: five
families, 25 PASS cells, 10 dashes, ZERO complete rows. The probe's
seat was declared with the table: the first family to enter under the
protocol, the choice made at the opening — **the ellipsoid, the
sphere's generalization**, its entry priced against the sphere's own
measured row (the re-frame rule, the slit-rectangle seam, the
trig-power recurrence: PASS-patterns the audit verified cell by cell),
its mass forms classical and therefore independent-oracle-able, and
the serialization dash the probe MUST redeem — the recipe the sphere
never paid.

**THE CHOICE AT THE OPENING, the reason named (D4's own discipline):
the ellipsoid lands as the ELLIPSOID OF REVOLUTION — the spheroid,
semiaxes (a, a, c), the sphere's one-parameter generalization.** The
reason, in the open:

1. **The probe measures the protocol's incremental cost — and the
   spheroid is the honest increment.** ADR-0041 D3's conquest is "the
   same six-column census closing for less and less new code per
   family": the entry that measures the protocol must CONSUME the
   inherited machinery maximally, so the delta says what the protocol
   saves rather than what a big family costs. The spheroid consumes
   every sphere PASS-pattern one family over: the slit-rectangle seam
   and the pole vertices (the writer), the (pole, azimuth) frame form
   with e₁ = e₀×n̂ (the frame, right-handed by the same construction),
   the trig-power recurrence with (a, c) coefficients (the mass), the
   typed latitude rows (the SSI: the ⊥ plane's circle with its Line2
   v = const, the coaxial cylinder's pair — the sphere's rows 4 and 6
   one family over), and the pole/equator/nested/coincident
   dispositions (the degenerates).
2. **The triaxial ellipsoid (a, b, c) is the family's NEXT member, and
   this ADR registers it as such.** The triaxial types NOTHING in the
   pinned space-curve vocabulary (every plane section is an ellipse —
   a conic in the section plane's domain, but the space-curve set pins
   circles and lines, and the p-curve image on the surface's own (u,v)
   domain is typed in neither): its SSI column would be all-wall — a
   legal census row, but one that measures nothing about the protocol's
   typed machinery, and a BLUNT first entry. The spheroid's two typed
   rows exercise the p-curve authoring, the stored-frame clause, and
   the band dispositions — the census's discrimination tested on every
   side. The triaxial enters through these same six rows when its
   consumer arrives (the cone's rider form, registered below); the v1
   class carries no b knob — the triaxial is unrepresentable in it,
   said here, never hidden.
3. **The other candidate keeps its registered seat.** The surface of
   revolution on a general straight generatrix (the skew seat) remains
   what D4 pinned: the ruled family's next member, the cone's
   registered rider — its writer question posed honestly when its unit
   opens. The register's own price list (the re-frame rule, the
   slit-rectangle seam, the trig-power recurrence) reads the
   sphere-generalization, not the ruled seat; the choice follows the
   register.

What the sphere's row already prices — and what this entry must pay
itself: the sphere's SERIALIZATION dash is "the recipe question its
first format-bearing consumer will pay". The ellipsoid IS that
consumer: an exact primitive that must ride the file as RECIPE, the
pattern the sphere never had. D6 below pays it, and the price is real
(a format bump, a new node kind, a golden corpus move).

## Decision

### D1. The parametrization (protocol row 1 — the writer's form)

The surface class **`EllipsoidSurface`** — the sphere's own form, one
parameter more:

```
E(u, v) = C + e₀·(a·sin v·cos u) + e₁·(a·sin v·sin u) + n̂·(c·cos v),
e₁ = e₀ × n̂,
```

with **a** the EQUATORIAL semiaxis (the u-circle's radius, v = π/2 the
equator), **c** the POLAR semiaxis (the pole distance, v = 0 and v = π
the poles), C the center. The frame: n̂ the pole (the revolution axis
— INTRINSIC to the spheroid, unlike the sphere where every axis
serves), e₀ the azimuth reference (u = 0 the seam meridian through
+e₀ — the ADR-0023 discipline, a free choice), e₁ derived. The class's
placement knobs: (a, c, C, n̂, e₀) — the SphereSurface re-frame
constructor's own shape, perpendicularity validated LOUDLY (a skewed
frame has no spheroid parametrization in its span; the sphere
constructor's own message form, the remedy named). The factory
constructor `EllipsoidSurface(a, c, C)` pins the DEFAULT FRAME the
world reads: **n̂ = +Z, e₀ = +X** (the revolution axis vertical — the
CylinderZ/TorusZ precedent of the axis the world's primitives stand
on; the sphere's historical +Y pole is the sphere's own history, not a
reason).

The domain: u ∈ [0, 2π) the azimuth (periodic, the seam at u = 0 ≡
2π), v ∈ [0, π] the polar angle from the north pole. The poles are the
parametrization's degenerate points (sin v = 0: the u-circle
collapses) — the sphere's own pole discipline, inherited verbatim: the
degenerate pole edges carry no co-edges, the pointwise checks sample
off the poles.

**The writer** — `make_ellipsoid(a, c, center, feature)`: the
seam-wired single-face body, `make_sphere`'s own topology one family
over — 2 vertices (the poles), 1 edge (the seam meridian, north →
south), 1 face whose single loop is the seam pair (forward the RIGHT
edge of the slit rectangle, Line2 (2π, 0) → (2π, π); reversed the
LEFT, Line2 (0, π) → (0, 0); both material-on-left, the rectangle's
interior). Counts: V = 2, E = 1, F = 1, χ = V−E+F−H = 2, the co-edge
census 1×2 = 2 — the sphere's own numbers, the golden's pins. The
names stamped in the same pass (ADR-0036 D1): FeatureKind **`elp`**
(the grammar's three-letter token), `kEllipsoidNaming` — the face role
`ellipsoid`, the edge role `seam` (the sphere's own vocabulary one
family over). The naming-plane FeatureKind and the family enum
(`SurfaceFamily::Ellipsoid`, family_name "ellipsoid") extend the single
authorities; the tessellator's family-window table gains the case
(u ∈ [0, 2π], v ∈ [0, π] — the sphere's own window); `translate_body`'s
table gains the case (the center moves, the frame carried like the
sphere's — derived state, valid verbatim); the binding
`CAD_0.brep.make_ellipsoid` exposes the writer (the make_box
precedent — the query surface the census reads from Python).

### D2. The frame rule and the chirality pin (protocol row 2 — ONE row)

**The frame rule, derived:** with the parametrization above,

```
∂u×∂v = sin v · ( a·c·sin v·cos u·e₀ + a·c·sin v·sin u·e₁
                + a²·cos v·n̂ ),
```

and the ellipsoid's unnormalized outward normal (the gradient of
ρ²/a² + z²/c², read in the frame basis)
N̂ = (sin v·cos u / a)·e₀ + (sin v·sin u / a)·e₁ + (cos v / c)·n̂. The
frame rule's number:

```
(∂u×∂v) · N̂ = sin v · ( c·sin²v + (a²/c)·cos²v )  >  0
```

for every a, c > 0 and v ∈ (0, π) — **right-handed by construction, in
every frame, pointwise over the whole open domain** (the poles
excluded, sin v = 0 there — the degenerate points D1 declares). The
frame serves the geometry: for the spheroid the pole is INTRINSIC (the
revolution axis) — the re-frame rule's boundary, read honestly. The
sphere's full re-frame freedom (any derived axis serves, the symmetry)
is a property the spheroid LACKS; the SSI entries below type in the
STORED frame when the pole serves — the sphere's own clause, the
spheroid's permanent condition, never a silent re-frame.

**The chirality pin** (the m3 pattern, on the family's own surface):
the pointwise regression guard — (du×dv)·n_out > 0 sampled on a 13×13
grid over the declared window, OFF the poles (the cone's apex
discipline) — plus the surviving evaluate pins (the equator's radius a
and quota 0; the poles at ±c·n̂; the seam meridian u = 0 through +e₀;
the v = π/2 circle's points at (±a, 0, 0)-in-frame). And the
**discriminating counterproof**: the guard FAILS on the left-handed
authoring — a naive u = +azimuth parametrization (e₁ = −e₀×n̂, the
sphere's pre-correction shape) negates (∂u×∂v)·N̂ at EVERY sample; the
test authors the naive form locally and pins the guard RED on it —
the left-handed ellipsoid is UNREPRESENTABLE in the landed class (the
frame rule's one construction), and the guard is what proves it stays
that way.

### D3. The mass family (protocol row 3 — closed forms, ONE recurrence consumed)

**The kernels.** The mass machinery's divergence forms read the
CROSS-PRODUCT normal, never the unit normal (the ellipsoid's
|∂u×∂v| = a·sin v·√(c²sin²v + a²cos²v) does not separate — the unit
normal's kernels would be elliptic). With N = ∂u×∂v as a form in the
trig powers of (u, v):

- x_i as a BForm: C_i + a·e₀ᵢ·(cos u·sin v) + a·e₁ᵢ·(sin u·sin v) +
  c·n̂ᵢ·(cos v);
- N_i as a BForm: a·c·e₀ᵢ·(cos u·sin²v) + a·c·e₁ᵢ·(sin u·sin²v) +
  a²·n̂ᵢ·(cos v·sin v);
- **(x−C)·N = a²·c·sin v** — the spheroid's own miracle (the sphere's
  R³·sin v one family over: the three principal terms collapse to the
  identity sin²v + cos²v = 1), derived here, the volume's kernel.

The Contrib protocol (F0 = ∬ x·N du dv, Fᵢ = ∬ xᵢ²·Nᵢ du dv,
Fᵢⱼ = ∬ xᵢxⱼ·(x·N) du dv) closes every one of these as products of
trig powers — separated by `b_integrate`, each factor closed by the
**SAME `integ_cs` parts recurrence the toroidal family landed
(G2-u3 m3): ZERO new recurrence code** — the delta's falling side, the
protocol's whole point.

**The closed forms (the goldens — classical, pre-dating the kernel,
therefore D1.5-qualified):** the full spheroid
V = (4/3)·π·a²·c, centroid = C, and about the centroid
I_x = I_y = (V/5)·(a² + c²), I_z = (2/5)·V·a².
The guide case this morso pins: **a = 0.75, c = 0.5** — V = 0.375·π =
1.1780972450961724, I_z = 0.2650718801466388, I_x = I_y =
0.1914408023281280 (the register's row carries these).

**The window discipline:** the rectangle-window class, the torus's own
(the loops' Line2 endpoints' bounding box, verified on the boundary,
the non-rectangle refused by name) — the factory's face the full slit
rectangle [0, 2π] × [0, π]. The BAND class is exercised by the test's
**barrel body**: the ellipsoid band between two latitudes (the rim
circles at z₀, z₁ — planar, each split at the seam meridian and its
antipode, the cylinder factory's own rim pattern one family over)
closed by the two planar disc caps — 4V/5E/3F, χ = 2 — and the
machinery's answer cross-checked against the **Cavalieri form**
V_barrel = π·a²·[z − z³/(3c²)] between the limits: the same arithmetic
read twice, the m3 precedent's own shape (the wedge verified BY the
mass). `all_exact` says what is true: every ellipsoid face exact.

**THE INDEPENDENT ORACLE (D1.5 — the name the census row carries):
"the 2·10⁶-point (u,v) midpoint quadrature, in the tests, beside the
classical forms."** The test re-derives V, the centroid and the
moments by its own product midpoint rule over the parametric domain —
the test's own loops, ZERO code shared with the kernel's BForm
machinery or its recurrence (the m3 toroidal precedent's form: the
quadrature in the test, the recurrence in the kernel, the same
question asked twice by two roads) — and the classical forms stand as
the primary goldens (they pre-date the kernel; the golden corpus is by
construction NOT independent, the classical forms are). The tolerance
DECLARED with its conditioning: the full-window u-integrals of the
trig-power kernels are closed EXACTLY by the periodic midpoint rule
(the periodic trapz miracle — the kernels are trig polynomials), the
v-direction converges as O(h²); the agreement measured and pinned
(≤ 2·10⁻⁶ relative, the grid declared: 2000 × 1000 = 2·10⁶ points),
the barrel's band window cross-checked the same way over its own
domain.

### D4. The band vocabulary and the Jacobian (protocol row 4)

The family reads the **Linear** tolerance world (the SSI entries'
bands: |d| against c, r against a — the declared tol, the
plane×sphere entry's own discipline) and the **Angular/cosine** world
(the coaxiality gate: |â·n̂| against 1 − tol, the cylinder×cylinder
parallelism discipline). Zero bare constants: every band reads the
query's declared tolerance, each use documented at its site.

**The Jacobian, declared (ADR-0035 D2.2's clause, the family's own
terms):** the surface measure
J(u, v) = |∂u×∂v| = a·sin v·√(c²·sin²v + a²·cos²v); the parameter
speeds |∂v| = √(a²·cos²v + c²·sin²v) and |∂u| = a·sin v. A linear
tolerance ε_L maps into the parameter world as the NAMED derivation
**ε_v = ε_L / |∂v| ≤ ε_L / min(a, c)** (the v-direction's worst-case
speed min(a,c), the tight bound; the u-direction analog
ε_u = ε_L/(a·sin v), unbounded at the poles — the pole bands are the
seam's own degenerate class, D1's declaration, never silently
shared). Today's consumers read the linear band directly (the entries
below); the derivation is the family's declared row, consumed by the
first parameter-band consumer when it arrives.

### D5. The degenerate dispositions (protocol row 5 — the family's zoo members, pinned at entry)

Each disposition pinned BEFORE the corpus member lands (D2 row 5's own
demand — for a family entering under the protocol, pinned AT entry);
the cross-family fuzz corpus remains the register's one-not-five debt
(D1.2's zoo, its own suite member — the five families' dashes stay
dashes, mai silenzio in either direction).

| Pair | Disposition |
|---|---|
| plane×ellipsoid, ⊥ the stored axis | \|d\| < c: **the circle** at the foot, radius a·√(1−d²/c²) — Circle2 on the plane's domain, the latitude Line2 v = arccos(d/c) on the ellipsoid's (the sphere's row 4 one family over) |
| — at the band edge \|d\| = c | **the tangent POINT** at the pole (Kind::Point) |
| — beyond \|d\| > c | **Empty** ("separate") |
| plane×ellipsoid, oblique or ∥ | **THE WALL** — the section is an ellipse: a conic in the section plane's own domain, but the space-curve vocabulary pins circles and lines, and the p-curve image on the ellipsoid's (u,v) domain is typed in neither — option (i), the pair named, the class honest |
| cylinder×ellipsoid, coaxial | r < a: **the TWO circles** at z = ±c·√(1−r²/a²), radius r — Line2 v = const on the cylinder's domain, the latitude Line2s on the ellipsoid's (the sphere's row 6 one family over) |
| — at the band edge r = a | **the tangent CIRCLE** at the equator z = 0 |
| — beyond r > a | **Empty** (the wall never reaches) |
| cylinder×ellipsoid, non-coaxial | **THE WALL** — the quartic (Bézout on the quadric pair), the Viviani wall's sibling |
| ellipsoid×ellipsoid | identical (a, c, C, frame within tol): **Coincident** — the co-radial dedup rule's trigger, reported; else **THE WALL** — the general pair is the quartic (no radical plane for spheroids, Bézout named); the equal-a coaxial pair's equator circle REGISTERED FOR THE RECORD — typable, awaiting its consumer (the cone's row-8 form) |
| sphere×ellipsoid | **THE WALL** — the quartic (Bézout, the sphere×torus precedent's form) |
| torus×ellipsoid | **THE WALL** — out on degree |
| cone×ellipsoid | **THE WALL** — the cone's row-8 registered record, unreachable (no cone writer) |

**The near-band members** (the zoo's own citizens, landed with the
family): the plane entry at d = c−δ / c / c+δ (δ a declared multiple
of tol) — the ties at the border resolved by the canonical key
(\|\|d\|−c\| ≤ tol → Point, never a shrunk circle); the cylinder entry
at r = a±δ and at the tie r = a (the tangent circle, never two
merged circles); the coaxiality band (the axis tilt just inside the
cosine band → the typed entry holds; just outside → the wall). Each
member's disposition pinned by test — WHICH path answers, WHAT the
message says — the naive counterproofs red on the un-banded read (the
oblique section typed as a circle is the wrong curve: its points leave
the surface — the discrimination, below).

**The naive counterproofs, red where the golden is green:** (a) the
oblique plane read with the sphere's row-4 form produces a circle that
does NOT lie on the ellipsoid (evaluate the claimed curve against the
surface equation — off by the ellipse's eccentricity, the red); (b)
the tangent cylinder r = a read as the two-circle class produces TWO
coincident circles (the pair class where the tie's canonical key says
ONE tangent circle — the double-count red); (c) the tie d = c read
without the band produces a circle of radius a·√(1−(c±δ)²/c²) — a
shrunk-circle class flip inside the band, the key's own red.

### D6. The serialization row (protocol row 6 — the recipe the sphere never paid)

The exact ellipsoid rides the file as an **exact-side PRIMITIVE NODE**
— the fillet's own form one kind over:

- **`NodeKind::Ellipsoid`**, the scene dispatch "ellipsoid", the
  FeatureKind `elp` in the naming plane (the node's own feature id —
  the box's own pattern, the names birthed with the body).
- The payload is **the RECIPE**: the named fields
  **`equatorial_radius`** (a) and **`polar_semiaxis`** (c) — never a
  raw struct, never an address (the m4 padding lesson, standing).
  The value class at load: finite and > 0 for both (the ADR-0029
  family — a non-positive semiaxis degenerates the surface, a loud
  load error, never a best-effort parse).
- The materialized exact body is **DERIVED state**, rebuilt by the
  same pass that materializes every recipe (creation, the DAG pass,
  the loader's pass 2 — `make_ellipsoid` from the named fields,
  deterministically); the display mesh the exact body's own
  tessellation (the m6-correttiva road, `brep::tessellate_body` —
  never a special render); no SDF twin (the fillet's own declaration).
- **THE FORMAT MOVES: v0.6.0 → v0.7.0 (MINOR), the reason declared**
  (a new node kind rides the document — the fillet's m4 precedent,
  the same rite): the NEWER door refuses a v0.8+ file BY NAME; the
  migrator **0.6.0 → 0.7.0** is a deliberate content no-op, the
  chain's sixth step — the frozen goldens walk it; the anachronism
  gate at the payload refuses an ellipsoid node in any file declaring
  < 0.7 (the hand-edited anachronism, the fillet's own gate form).
- **NO v0.6 door — the honest rite, the reason named.** The v0.4 and
  v0.5 doors refuse their predecessors because those eras carried
  UNPERSISTED CONTENT (constraints as session state; fillets on the
  bodies, kernel-only) — opening them would present a model as if
  nothing were lost. The v0.6 era had NO ellipsoids: a v0.6 file loses
  NOTHING at v0.7 (the anachronism gate guarantees it stays that
  way). This is the **v0.3→v0.4 precedent** (the sketch payload — a
  capability-adding bump whose predecessor files walk the chain), not
  the v0.5/v0.6 silent-loss doors; the register says so at the bump.
- **The corpus moves, each with its reason:** `native_v0_7_0.CAD_0`
  NEW (the v0.6 golden's own document, ids 1–9 — the parametric hole,
  the blind chain, the rim fillet — plus the ellipsoid node id 10, the
  guide case a = 0.75, c = 0.5); `native_v0_6_0.CAD_0` stays and now
  WALKS the 0.6→0.7 step (its fillet recipe preserved by the no-op
  migrator, the walk pinned by test — the v0.1–v0.3 goldens'
  discipline one step over); not one frozen golden touched, the
  additivity whole: the m2/m3 pins (10/15/7, 14/20/9) unmoved.
- **The proofs:** the round-trip's digests N = 10 bit-identical (the
  load→read→save cycle, every cycle's structure digest equal the
  first's — the recipe back field for field); the DOORS (the newer by
  name; the anachronism by name with the version named; the value
  class in the loader's own words); the PADDING COUNTERPROOF red on
  the naive (a raw-struct writer built for the occasion — a padded
  payload image — produces two digests for the same content, the
  named-fields writer identical bytes); the RE-RUN AFTER THE LOAD (the
  loaded node re-materializes bit-identically — the digests equal the
  first materialization's, the recipe the truth, the body the cache).
  The scene door: `add_ellipsoid_node` (the add_fillet_node pattern —
  the payload and the materialization installed together, one id) and
  the binding reachable from Python (the round-trip drivable from the
  query surface).
- **Not this morso, said:** the UI exposure (the Create menu, the
  panel's parameter editor) — the census's serialization column asks
  for the round-trip, the named fields, the digests and the doors; the
  first UI consumer is its own morso, registered here.

### D7. The price against the torus baseline — the six units, counted BEFORE the code

ADR-0041 D3's count is the measure; this entry's delta, unit by unit,
declared here and verified in the register's row with the numbers:

| # | The torus paid (D3) | The ellipsoid pays | The delta |
|---|---|---|---|
| 1 | the writer: `TorusZSurface` with the fillet's flank, the seam riding the loop twice | the writer: `EllipsoidSurface` + `make_ellipsoid` (the seam-wired single face — the sphere's pattern CONSUMED), the naming, the family enum, the tessellate window, `translate_body`'s case, the binding | **WRITTEN** — the class, the factory and the exposure new; the pattern inherited |
| 2 | the chirality flip: the registered left-handed PAIR flipped, the one-liner, the pins on both | no flip to make: the family enters AFTER the frame rule, born right-handed by construction (the sphere's e₁ = e₀×n̂ form) — **ABSENT, the reason named**; the GUARD still written fresh (the pointwise rule, the evaluate pins, the naive-left-handed counterproof RED) | **ABSENT** (the flip) + the guard WRITTEN |
| 3 | the mass family: the (u,v) rectangle integrals, ONE parts recurrence landed, the quadrature cross-check | `ellipsoid_contrib` (the BForms with (a, c)) — **the recurrence CONSUMED, zero new recurrence code**; the closed forms classical; the oracle in the tests; the barrel's band window the cavalieri pin | **WRITTEN small** — the contrib alone; the machinery inherited |
| 4 | the SSI rows 9/10: registered BEFORE the family entered (G2-u1's own act), consumed at their tangent limits by construction — ZERO table code at entry | the rows are NOT pre-registered: TWO typed entries (plane×ellipsoid ⊥, cylinder×ellipsoid coaxial) + the walls' dispatch + the degenerate dispositions — **new table code** | **WRITTEN BIG — the honest rising unit**: this entry pays table code the torus never paid at entry (its rows were pre-registered); the row says so |
| 5 | the class-2 construction: the rim flank itself, the rolling ball a construction never a solve | no construction consumes the ellipsoid today — **ABSENT, the reason named** (the family's first consumer is the factory; the ellipsoid flank and the ellipsoid boolean are future rows' own gates, registered) | **ABSENT** |
| 6 | the 47-row tavola: kept/absorbed/born/vanished, the golden 14/20/9 | no operation, no descent — **ABSENT, the reason named** (the primitive's census is its golden's counts: 2/1/1, χ = 2, the co-edge census 1×2 = 2) | **ABSENT** |
| 7 | — (the torus rode the fillet's recipe; the format's v0.6 was the fillet's own bump) | **the serialization recipe + THE FORMAT BUMP**: the node kind, the named fields, the migrator 0.6→0.7, the doors, the anachronism gate, the corpus golden — the sphere's dash redeemed, the price the sphere's row named | **WRITTEN — the unit the era owes**: the probe's extra unit, declared |

**The ratio, declared in advance and measured at the landing:** the
conquest's direction is the ratio falling — the same census closing
for less new code per family. This entry's honest forecast: units 2,
5 and 6 are ABSENT (the flip, the construction, the tavola — three of
the torus's six), unit 3 consumes the recurrence whole, unit 1
consumes the writer's pattern; against that, unit 4 is BIGGER than
the torus's (the pre-registration debt) and unit 7 is NEW (the era's
serialization debt). **Whether the ratio falls is measured, not
asserted — and if it does not fall, the register's row says so and
names the cause** (the two debts the era owes: the table's
pre-registration and the recipe), the early warning D3 demands, said
in the row, not hidden in a review.

## Consequences

**Positive.** The first family enters under the protocol, and the
census gains its FIRST COMPLETE ROW — the seven columns, each with its
pinned evidence, the discrimination demonstrated end to end (a family
that pays every column, not a feature count). The SSI table gains its
first new entries since G2-u1 — the sphere's typed rows one family
over, the stored-frame clause's boundary made honest. The mass
machinery closes a THIRD family on the same recurrence — the
protocol's reuse thesis with numbers. The format gains the exact
primitive's recipe pattern — the question the sphere's dash named,
paid once, the road every future exact primitive walks (the sphere's
own recipe its consumer's morso, the pattern now landed).

**Negative, said.** The row completes the SPHEROID class only — the
triaxial is unrepresentable in the v1 class (no b knob), registered
as the family's next member, its entry through these same six rows
when its consumer arrives. The UI exposure is not this morso (the
Create menu and the panel await the first UI consumer, its own bite).
The ratio may not fall on this entry alone — the two era-debts (the
table's pre-registration, the recipe) are real new code, and the row
carries the numbers either way. The near-band corpus landed is the
FAMILY's slice, not the zoo — the cross-family fuzz corpus stays the
register's one-not-five debt, its own suite member.

**Neutral, registered.** The five families' dashes stay dashes — the
degenerates column's zoo debt and the sphere's serialization dash are
unchanged by this entry (the ellipsoid's row says what the ELLIPSOID
is, never what the others are not). The K row counts the morso; the
census table gains the row; the cronaca stays untouched.

## References

- ADR-0041 — D1.1 the seven columns; D1.5 the independent oracle (the
  golden corpus not independent BY CONSTRUCTION; the classical forms
  qualify); D2 the six-row protocol, the two legal axes, the banned
  third; D3 the torus baseline's six units; D4 the probe's pinned
  success; D6 the status machinery.
- ADR-0033 — D2 option (i) the wall (the oblique sections, the
  quartic pairs); option (ii) REGISTERED, NOT FIRED (nothing in this
  entry asks for an algorithm); the maintainer's motivation, verbatim:
  «il muro è sempre onesto; la siepe, una volta piantata, va
  sorvegliata per sempre; prima il muro, la siepe quando un caso vero
  la domanda.»
- ADR-0032 — the frame rule; the sphere's (pole, azimuth) form.
- ADR-0039 — D1.2/D3 the torus baseline's material (the writer, the
  flip, the forms, the recurrence this entry consumes).
- ADR-0035 — D2.2 the Jacobian clause (D4 above).
- ADR-0014 — the determinism clauses (the forms ride the existing
  recurrence's fixed orders; the digests bit-identical N = 10).
- ADR-0007/0007a — the version arithmetic; the migrator chain's sixth
  step.
- The register — the census table (the seat, the ten dashes, the
  probe's named choice); the K row (m1 the ADR, m2 the table, m3 this
  morso); the m3 precedent (the oracle's form: the quadrature in the
  test, 1.6·10⁷ points, 2·10⁻⁶ relative — this entry's oracle the
  same discipline).
