# ADR-0037: H2 unit 5 — the coplanar class: the contact taxonomy in the plane, the band that decides, the derived capability and its bounds

## Status

Accepted (2026-11-02) — morso 1 delivered this contract as
**Proposed** (docs-only: the derivation pinned, no code moved); the
maintainer's review found no correction — D1's wall map (eleven
placements verified by direct execution, the one refusing brick named
with its structural reason), D2's composite gate (two typed members,
zero new constants, the seat and the order pinned), D3's overlay (the
projection, the 2×3 region table cross-checked on seven verified
cells, the convex-fragment emission, the pool sewing, the 3D↔2D
coherence rules), D4's scope (one capability, three shapes, the queue
named behind it) and D5's pre-pinned numbers (the door's 8/12/6,
20/32/14, 8/12/6; the ratifications 20/32/14, 20/32/14, 16/26/12,
16/26/12) stood as written. **The acceptance act applies the register
surgery D6 declares, in the same act — never a late registry
gesture**: ADR-0035 D2's vocabulary gains the ε_C row (verbatim as D2
pinned it), and ADR-0025 §3's and §7's coplanar-overlap deferrals are
RESOLVED, both amended in place, dated, pointing here. Morso 2 — the
unit's last — implements the door under this contract: the gate at
the pinned seat, the overlay, the goldens written from D5's numbers
BEFORE the code, the red demonstrated on the pre-overlay tree, the
green landed with the door.

Daughter of ADR-0031 D5 (the phase ladder — the one-guide-case
scale method this ADR's D4 applies verbatim), of ADR-0035 (the
numerical taxonomy — the band method D2 extends with a fourth
member, the golden protocol D5 inherits), of ADR-0036 (the naming
grammar D6 extends WITHOUT touching the mothers, the D4-queue
pattern, the D5 counterproof protocol), of ADR-0025 (the polygon
booleans' coplanar history — v1 §3's deferral and v5's
parallel-plane rule are this unit's starting line, and §7's
"subdivision of coplanar-overlapping FACES remains deferred" is
the debt this ADR derives the payment for), of ADR-0023/0024 (the
coplanar dedup rule's origin — the cap-flush semantics D1
ratifies), and of ADR-0033 (the G2 boundary — the exact-path
classes this ADR registers as G2's stay G2's; the boundary is not
moved).

**The debt this ADR pays is pre-registered in the tree, twice.**
The output gate's open-shell refusal names the class and the
destination: "the known PARTIAL COPLANAR OVERLAP class (ADR-0025
v5's limitation): the union of the faces' outside parts leaves the
overlap's rim open, and **until H2 unit 5 sews the shared-face
class the refusal IS the answer**" (boolean.cpp:1553-1556) — and
the H2-u2 golden pins the destination inside the refusal message
itself ("H2 unit 5", test_boolean.cpp:613). This document is that
destination: the derivation that decides which bricks of the wall
become doors, which are already doors nobody had verified, and
which stay walls — each by name, with the reason.

## Context

The state, verified against the tree at 6ab0cf8 (the repo is the
authority; the working tree clean on the last green; the zip
lesson's check — `git ls-files -s scripts/git-hooks/` — reads
100755 again after this delivery's first chore, which restored the
executable bit the H2-u4 m1 round trip had stripped).

### The two paths, and where the plane is crossed today

**The exact path** (exact_subtract, the G1 class of ADR-0032 D7):
the only coplanar contact it survives is the **cap flush** — the
tool's cap coplanar with the pierced face, the profile strictly
inside the footprint. The rule is the ADR-0023/0024 dedup: "a
flush cap ABSORBS — the coplanar dedup rule, the counts
unchanged" (subtract.hpp:68-69), emitted as the absence of an
emission — "the tool's caps die (a flush cap absorbed — nothing is
emitted for them, which IS the dedup)" (subtract.hpp:1199-1200).
Everything more general is refused, each refusal honest — the
diagnosis names what, the remedy names how:

- the **CROSSED** refusal: the tool's profile pokes beyond the
  pierced face's boundary — "the non-annular pierced face is
  honest G2 topology, not G1" (subtract.hpp:1140-1152; pinned at
  test_g1_unit4_subtract.cpp:952);
- the **TANGENT** refusal: the wall tangent to the footprint's
  boundary from inside, within the declared band — the
  deterministic tie-break, canonical-key ordering over the edges,
  the pinned disposition REFUSE (subtract.hpp:1153-1169; pinned at
  test_g1_unit4_subtract.cpp:914, the message bit-identical across
  re-runs);
- the **BLIND** refusal: the tool terminating inside the target —
  "the internal cap face class is G2's (it needs the SSI-driven
  cap insertion, not the pass-through circle pair G1 builds)"
  (subtract.hpp:1175-1186; pinned at :973);
- the **SWALLOW** refusal: the profile swallowing the
  cross-section, the honest empty (subtract.hpp:1104-1110;
  pinned at :1037).

**The polygon path** (bool_union / bool_subtract / bool_intersect,
ADR-0023/0024/0025): the coplanar history is richer than the
refusal suggests. The v5 parallel-plane rule
(classify_parallel_plane, boolean.cpp:1215-1256) already decides
the plane question per (face, clip-plane) with the three-case
band — the rule that killed the grid-straddle spurious caps
(boolean.cpp:2325-2340) — and the slab decomposition plus the
T-junction subdivision (ADR-0025 v5 §8/§10) already tile the
non-convex leftovers.

### The wall map, verified — not assumed

Every claim below was verified by direct execution on the tree at
6ab0cf8 (boxes built with the factories, the operations called
with the house bundle `kBooleanHouseTolerance`); the cases with a
test pin cite it, the cases without one are pinned by this ADR's
D5 so the verification stops being an anecdote. A = [-1,1]³
throughout (make_box half-extents 1,1,1).

| # | The contact (the placement) | op | Today |
|---|---|---|---|
| P4 | B = [-0.5,1.5]×[-1,1]×[-1,1] — the **aligned** coplanar ±Y/±Z pairs CROSS (area overlap, boundaries cross) | union | **REFUSED** — result_edge_not_manifold, "8 of 24" (pinned :586) |
| P8 | B offset (0.5, 0, 0.5) — the aligned pairs cross DIAGONALLY (the merged region non-convex) | union | **REFUSED** — "8 of 40" (verified; unpinned) |
| P11 | B = [-0.5,0.5]×[-1,1]×[-0.5,0.5] strictly inside A, tops flush — the aligned pairs CONTAINED | union | **REFUSED** — "8 of 36" (verified; unpinned; the honest answer is A whole) |
| P1 | B = [1,2]×[-0.5,0.5]×[-0.5,0.5] on A's +X face — the **opposed** pair CONTAINED (the boss) | union | **handled**: V=20 E=32 F=14 χ=2, closed (verified; unpinned) |
| P10 | B = [-2,-1]×[-0.5,1.5]×[-1,1] across A's -X face — the opposed pair CROSSES | union | **handled**: V=16 E=26 F=12 χ=2, closed (verified; unpinned) |
| P7 | the boss of P1 spanning the full width — the aligned ±Y pairs touch EDGE-TO-EDGE (no area) | union | **handled**: V=16 E=26 F=12 χ=2, closed (verified; unpinned) |
| P5 | B = [1,3]×[-1,1]×[-1,1] side-by-side, sharing A's +X face fully (the rotated impilato) | union | **handled**: V=12 E=20 F=10 χ=2 (pinned :322 in the stacked orientation) |
| P2 | B = [0.5,1]×[-0.5,0.5]×[-0.5,0.5] — the flush notch (the opposed pair contained; the parete) | subtract | **handled**: V=20 E=32 F=14 χ=2, closed (verified; unpinned) |
| P6 | B offset (0.3, 0, 0) — the aligned pairs cross (the grid-straddle) | subtract | **handled**: V=8 E=12 F=6 χ=2 (pinned :443; the corner notch :348 is the crossing-opposed subtract, pinned 17/27/12) |
| P9 | B offset (0.5, 0, 0.5) | intersect | **handled**: the overlap box 8/12/6 (pinned :796's intersect half) |
| P3 | P1's placement (the region contact) | intersect | **handled honestly**: the zero-volume contact disk 4/4/1 χ=1, closed=false (the output gate's class 2; pinned :301 in the stacked orientation) |

**The wall's precise shape, one sentence**: of the whole coplanar
zoo, exactly ONE brick refuses today — **the union of two faces
that lie in the same plane with ALIGNED normals and overlapping
regions** (crossing, P4/P8, or contained, P11); every opposed
pair, every subtract, every intersect, every edge-touch and every
impilato answers, most of them already pinned green. The refusal's
own text says why: "the union of the faces' outside parts leaves
the overlap's rim open" — the outside-parts algebra cannot express
the region that belongs to BOTH solids' boundaries at once. That
one sentence is the whole unit: the region between the two faces
is the answer's face, and no assembly of outside fragments can
ever produce it.

**The structural reason, derived once**: for a coplanar pair
(F_A on π, F_B on π), a point of the overlap region F_A ∩ F_B is
inside the CLOSED other solid whenever the other solid's material
lies on the far side of π (the clip keeps {d ≤ ε_L}, and d = 0 on
π) — so the overlap appears in NEITHER outside part, while the
union's true boundary at π (the aligned case: both solids on the
same side) contains it wholly. The subtract never sees the hole in
its algebra because B's coplanar face contributes no cavity cap
(it lies on ∂A, never in A's interior — the v5 parallel-plane
skip, boolean.cpp:2325-2340) and A's face's outside part is
exactly F_A ∖ F_B; the intersect keeps inside parts, which cover
π exactly. Only the union's complement-flavored algebra is
structurally blind in the middle. This is not a bug to fix — it is
a capability to derive, and the derivation is D3.

## Decision

### D1. The taxonomy of the coplanar contact — four classes, each with its guide case and its verified status

The classes are typed by **the contact's geometry in the plane**
(what the two faces' regions do to each other) and **the operands'
configuration across the plane** (where the material lies) — the
two facts D2's gate and D3's algebra consume. The taxonomy is
PER-PATH: the exact path and the polygon path have different
walls, and an honest register does not average them.

**CLASSE 1 — CAP-FLUSH TOTALE** (the cap coplanar and INTERNAL to
the target's face; the contact region strictly inside both
boundaries, no crossing).

- *Guide case*: the G1 parametric hole with the tool's bottom cap
  exactly on the box's bottom face — box [-1,1]³, the tool the
  circle prism r = 0.5, z ∈ [0,3], offset (0,0,-1) → z ∈ [-1,2]:
  the bottom cap FLUSH at z = -1, the top cap above. Result:
  **V=12, E=17, F=7, H=2, χ=0** — the guide case's own counts, no
  spurious face, the wall spanning exactly [z_bot, z_top], the
  bottom face carrying its inner loop at z = -1 (absorbed, not
  duplicated).
- *Status*: **GESTITO, both paths.** The exact path: the dedup
  (subtract.hpp:68-69 the rule, :1199-1200 the emission-less
  absorption), pinned at test_g1_unit4_subtract.cpp:892-912. The
  polygon path: the v5 parallel-plane skip (the coplanar face
  contributes no cap — on ∂A, never interior), the same topology
  through the fragment pipeline.
- **RATIFIED here, the semantics written per estenso** (the rule
  deserves its statement, not just its citation): when the tool's
  cap and the target's face are coplanar within the operation's
  declared bands (D2) and the contact region is internal to both
  boundaries, the pair's contribution to the result's boundary is
  **the target's face ALONE** — the tool's cap is a zero-volume
  duplicate of a boundary that already exists, and the honest
  answer is the absence of a second face, not a merged one. In
  ADR-0036's vocabulary (D6): the target's face is **kept**
  (absorbs the contact; it gains the inner loop where the
  operation pierces it), the tool's cap **vanishes** — the dedup
  is the vanish, and "nothing is emitted" IS the semantics.

**CLASSE 2 — SOVRAPPOSIZIONE PARZIALE** (the contact region
crosses at least one of the two boundaries — the SSI degenerate:
not two transverse surfaces, an OVERLAY 2D; the intersection
"curve" is born IN the plane as the crossing segments of the two
loops).

- *Guide case (the door's own)*: A = [-1,1]³, B =
  [-0.5,1.5]×[-1,1]×[-1,1] — the aligned ±Y/±Z pairs cross; the
  union's honest answer is the box [-1,1.5]×[-1,1]×[-1,1]:
  **V=8, E=12, F=6, χ=2**. The adversarial companion (the
  non-convex merged region): B offset (0.5, 0, 0.5), the merged
  ±Y regions the stair polygon (8 vertices, 2 reflex), the
  answer's counts derived in D5: **V=20, E=32, F=14, χ=2**.
- *Status*: **REFUSED on the union (the wall this unit breaks);
  handled on the subtract and the intersect.** The union:
  result_edge_not_manifold (boolean.cpp:1546-1564), pinned :586
  (axis) — the diagonal (P8) and the contained-flush (P11)
  variants verified refused, unpinned. The subtract: the
  grid-straddle 8/12/6 pinned :443, the corner notch (the opposed
  crossing) 17/27/12 pinned :348, the slab :463. The intersect:
  the overlap box 8/12/6 pinned :796. On the EXACT path the class
  is the CROSSED refusal's own territory — "the non-annular
  pierced face is honest G2 topology, not G1"
  (subtract.hpp:1140-1152) — and STAYS G2's (D4).
- *The disposition*: the crossing contact becomes a door on the
  polygon path's union (D3's overlay); on the exact path it stays
  a registered wall (D4, the reason derived there).

**CLASSE 3 — IMPILATO FACCIA-SU-FACCIA** (the solids share the
cutting plane; the contact region IS the whole face pair — the
classical class of every kernel's first test).

- *Guide case*: A = [-1,1]³, B = [-1,1]×[-1,1]×[1,3] — stacked,
  same cross-section, contact at z = 1 (the opposed pair, regions
  coincident).
- *Status*: **GESTITO, all three operations, with the counts
  pinned.** Union: **V=12, E=20, F=10, χ=2** closed manifold —
  the interface pair dies (the top of A inside B, the bottom of B
  inside A), the shared interface edges weld, and the lateral
  pairs stay SPLIT (the crease — honest topology, a valid closed
  body; see D4's queue for the merge). Pinned :322 (and the
  rotated orientation, P5, verified). Intersect: the zero-volume
  contact answer — the shared disk **V=4, E=4, F=1, χ=1**,
  closed=false, the output gate's class 2 (boolean.cpp:1462-1469,
  :1525-1534), pinned :301. Subtract: the touch removes nothing —
  A unchanged (the deterministic disposition; the polygon side via
  the parallel-plane skip, the exact side via the z-side miss
  band, subtract.hpp:1046-1057: "a touch removes nothing").
- *The disposition*: **RATIFIED.** Nothing to build; the semantics
  (the interface's dissolution as the opposed-coincident pair's
  symmetric difference — D3's table) is derived so the ratification
  is a proof, not a shrug.

**CLASSE 4 — PARETE-NEL-PIANO** (the tool's lateral wall LIES in
the target's face plane — the zero-thickness contact; the class
the big kernels treat worst).

- *Guide case (the polygon path)*: the boss — B =
  [1,2]×[-0.5,0.5]×[-0.5,0.5] on A's +X face, the contact square
  contained in A's +X face: the union answers the box with the
  boss, **V=20, E=32, F=14, χ=2** closed; the flush notch (the
  same placement subtracted): **V=20, E=32, F=14, χ=2** closed
  (P1/P2, both verified, both unpinned — D5 pins them).
- *Status*: **GESTITO on the polygon path — verified, not
  folklore.** The machinery that carries it: the v5 parallel-plane
  rule (the coplanar WALL contributes no cap — it is on ∂A), the
  frame's convex tiling (the holed region as slab fragments —
  four rectangles around the contact, the same discipline the
  corner notch's L-faces practice), the T-junction subdivision
  (the rim sews against the boss's laterals). On the EXACT path:
  **REFUSED, and the refusal is the class's own border** — the
  wall-in-plane is the tolerance band's boundary case: the
  TANGENT refusal (subtract.hpp:1153-1169, pinned :914) when the
  profile's edge lies within the band of the footprint's edge;
  the SAT touch (the deterministic miss — the exact copy,
  subtract.hpp:1080-1095) when the wall's plane touches from
  outside.
- *The disposition*: the class is RATIFIED on the polygon path
  (D5's pins make it un-regressible) and STAYS A REGISTERED WALL
  on the exact path — the reason derived in D3's last section
  (the walk-table authoring and the region-contact disposition the
  G1 class does not carry; G2's own pointer,
  subtract.hpp:1183-1185).

### D2. The band that decides — the coplanarity gate, derived; the fourth member of the vocabulary

**Coplanarity is not a comparison against a constant.** It is a
pair of typed decisions — one per world — composed into one gate;
each member already exists in ADR-0035's taxonomy, and the
derivation here is the COMPOSITION, named. For the face pair
(F_A on plane π_A with unit normal n̂_A, F_B on π_B with n̂_B):

1. **The ANGULAR member** (the two-plane determinant): the
   normal cross ‖n̂_A × n̂_B‖ = sin θ is the plane pair's analog
   of the segment pair's det (ADR-0035 D2.3) — dimensionless,
   bilinear in the unit normals. The pair is
   equivalent-to-coplanar in direction iff

       sin θ ≤ sin θ_strict,   equivalently |n̂_A·n̂_B| ≥ cos θ_strict

   with θ_strict the STRICT parallelity threshold already typed
   and declared (kBoolStrictNormalParallel, boolean.hpp:148-149 —
   the historical 1−10⁻⁶ as its angle, ≈ 0.081°; ADR-0035 D2 row
   4). No new constant: the angular world's own.

2. **The LINEAR member** (the offset window): d(p) = (p − o_B)·n̂_B
   over F_A's polygon. The pair is coplanar in position iff the
   window is inside the band:

       [d_min, d_max] ⊆ [−ε_L, +ε_L]

   — ε_L the operation's declared linear tolerance (the same
   world as the welding band). THE WINDOW, not the single value:
   a near-parallel pair whose d drifts across the band (d_min <
   −ε_L < d_max) is NOT coplanar — it falls back to the
   transversal path and is clipped per-vertex (the rule
   classify_parallel_plane already practices, boolean.cpp:1240-1255:
   the straddle returns NotParallel). The band admits only the
   UNANIMOUSLY-inside pairs — no mixed verdict is reachable.

**The gate, completed** — the precise test that decides the class
(the seat and the order, so the implementation cannot invent
them): in the union's pair processing (the process_faces loop,
boolean.cpp:2184-2213, where inside/coincident/outside is decided
today), BEFORE the clip: (a) both faces planar — the input gate's
own guarantee (validate_boolean_input); (b) the angular member —
outside → the transversal path of today, unchanged byte for byte;
(c) the linear member's window — straddle or strict side → the
transversal path; (d) both inside → **the coplanar class** — and
then ONE more decision, exact, no band: the ALIGNMENT

       sign(n̂_A · n̂_B)

+ : the ALIGNED pair (the flush pair — both solids' material on
the same side of π; the union's boundary at π is the region UNION
F_A ∪ F_B); − : the OPPOSED pair (the contact pair — the material
straddles π; the union's boundary at π is the SYMMETRIC
DIFFERENCE). The sign is exact (Family 1) and unambiguous BY
CONSTRUCTION: the angular member's admission already guaranteed
|n̂_A·n̂_B| ≥ cos θ_strict ≈ 1 − 10⁻⁶ > 0, so inside the band the
dot product's sign cannot be noise. The alignment is not a new
tolerance — it is a predicate the band's own admission makes
exact.

**The position in the vocabulary — the fourth member.** ADR-0035
D2's table gains a row (the register surgery, D6; applied in the
act of acceptance):

| Decision | Kind | Threshold | Derivation |
|---|---|---|---|
| **the coplanarity admission: the planar contact class gate** (added 2026-11-02, H2-u5 m1; see ADR-0037 D2 — and, with it, the alignment member that selects the region algebra, ADR-0037 D3) | **Coplanar** (a composite kind: the angular strict threshold AND the linear offset window — one decision, two members, each in its own world; never one constant for both) | **ε_C = ( sin θ ≤ sin θ_strict ) ∧ ( [d_min, d_max] ⊆ [−ε_L, +ε_L] )** — the two-plane analog of the determinant band: a derived SCHEME, not a stored number, its members drawn from the existing typed worlds | first principles, ADR-0037 D2 |

Like the determinant band |det| ≤ 4·ε_L·L before it, ε_C is a
form, not a field: the code compares two derived quantities
against two ALREADY-DECLARED tolerances, and the vocabulary row
exists so the register can audit the decision as one named thing.
The scale honesty is inherited, not re-derived: the angular member
is scale-free by construction (unit normals); the linear member
rides the operation's ε_L, so under uniform scaling the gate's
verdict is scale-invariant exactly when the ε_L declaration is
relative — D2.3's own pin, ratified for this member.

**IL CASO LIMITE — the sentinel at the border.** The band is not
a cliff; it is a border post with a sentry, and the sentry is a
derived number. Just OUTSIDE the band (θ slightly above
θ_strict), the transversal path computes the intersection line
ℓ = π_A ∩ π_B — and the line's POSITION conditioning degrades as
the band is approached: with the operands' points trusted to ε_L,
the line's position is trusted to

       ε_line(θ) = ε_L / sin θ

(the 2×2 solve of the two plane equations has the normal cross —
magnitude sin θ — as its determinant; the bound is Cramer's rule
plus the operands' trust radius). At the border, sin θ_strict ≈
1.4142·10⁻³ (the threshold's own sine — sin x ≈ x at this scale,
error < 10⁻⁹ relative), so

       ε_line(θ_strict⁺) ≈ 1e-6 / 1.4142e-3 ≈ 7.07·10⁻⁴ mm ≈ 707·ε_L

— the line just outside the band is pinned to seven hundred
tolerances, and the trust tightens as 1/sin θ with the angle: at
10× the band ≈ 71·ε_L, at 100× ≈ 7·ε_L. **The declaration**: the
near-band transversal answers are NOT refused — they have their
path (the classical clip) and their tolerance (the derived
ε_line, monotone, named here); the answers stay honest with a
soft radius. The band is where the CLASS changes — inside it
there is no line to be imprecise about (the answer is the region,
decided at ε_L); outside it the line exists and its trust radius
is the derived bill. No input falls between the chairs: the
window rule's straddle fallback sends every drifting pair down
the transversal path, and the ties at the border resolve by
Family 3's canonical key (deterministic, golden-pinned — the
TANGENT refusal's own discipline). The open question the sentinel
leaves — whether ε_line should SURFACE in the result's Layer-2
record (the intersection curve's reported tolerance) rather than
living only in this derivation — is REGISTERED for G2's
provenance contract (D6's list, by name).

### D3. The algorithm per handled class — the overlay 2D, the region algebra, the sewing

For the classes that are or become doors, one algorithm — the
same arms, one plane less. The overlay consumes the pair the gate
admitted and produces the boundary contribution at π.

**1. THE PROJECTION.** Both faces' polygons project into the
common plane's own (u,v) frame — A's face's frame (the isometry;
for a coplanar-admitted pair the points ARE in the plane, the
frame map is exact, double end to end, no quantizer). The
projection is the GH pipeline's own gesture
(polygon_subtract_3d projects to the face plane before clipping)
— nothing new, one earlier exit.

**2. THE 2D BOOLEAN — the region algebra, the table.** The
contact's answer is a planar region computed by the segment
machinery the path already carries (segment_intersection with the
typed predicates: orient2d exact, the derived determinant band
|det| ≤ 4·ε_L·L for the parallel/collinear admission, ε_P for the
t/u windows — the H1 arms, boolean.cpp:366-452). The region
algebra is a 2×3 table, and every cell is VERIFIED against the
wall map's cases (the table is not a proposal — it is the
generalization the eleven probes already instantiate):

| op \ alignment | OPPOSED (the contact pair) | ALIGNED (the flush pair) |
|---|---|---|
| **union** | F_A Δ F_B — the symmetric difference (the contact region is interior: the material straddles π) | **F_A ∪ F_B — the union region (THE DOOR)** |
| **subtract** | F_A ∖ F_B | F_A ∖ F_B |
| **intersect** | F_A ∩ F_B (the zero-volume contact disk when the solids touch only at π) | F_A ∩ F_B |

The cells cross-checked: the stacked union (opposed, coincident
regions) → Δ = ∅ — the interface dissolves, 12/20/10 ✓ (pinned
:322); the boss (opposed, contained) → Δ = the frame, 20/32/14 ✓
(P1); the across-contact (opposed, crossing) → Δ = the two
strips, 16/26/12 ✓ (P10); the grid-straddle subtract (aligned,
crossing) → ∖ = the outside strip, 8/12/6 ✓ (pinned :443); the
flush notch (opposed, contained) → ∖ = the frame, 20/32/14 ✓
(P2); the region-contact intersect (opposed, contained) → ∩ =
the square, the disk 4/4/1 ✓ (P3); the diagonal intersect
(aligned, crossing) → ∩ = the overlap face, 8/12/6 ✓ (P9). The
one cell the current code cannot express — aligned ∪ — is
exactly the three refusals P4/P8/P11: the outside-parts algebra
decomposes ∪ into (F_A∖F_B) ∪ (F_B∖F_A) and LOSES the middle.
The overlay computes the missing region directly: the crossing
points of the two loops (the degenerate SSI born IN the plane —
segments crossing segments, the 2D arms above), the union
boundary walk through them, the region emitted.

**3. THE EMISSION — the convex-fragment discipline.** The merged
region is emitted as CONVEX FRAGMENTS by the slab decomposition
at the region's vertices — the discipline the path already
practices (the corner notch's L-faces are "2 rectangles per cut
face — the L-shape split by the slab decomposition", pinned
:348's comment; the frame's four rectangles around the contact,
P1). The reason is a derived rule, not a taste: **the overlay's
output feeds the next operation in the DAG, and the input gate
refuses non-convex FACES (input_face_not_convex)** — a merged
8-gon emitted whole would make the composed chain A∪B∪C refuse
its own intermediate body, a one-shot capability. The fragments
keep every output face convex: the composed chain stays in-class.
(The collinear T-vertices the slabs introduce are the existing
T-junction machinery's own food — v5 §10.)

**4. THE SEWING — one point, one instance.** The crossing
points, the kept vertices and the T-junction points enter the
SAME canonical pool the path already shares (the
IntersectionTable / SharedVertexPool discipline, ADR-0025 §6:
each interface point computed ONCE, canonicalized, read by both
consumers — "bit-identical by construction"); the adjacent faces'
fragments reference the same keys, the welding degenerates to
identity, the T-junction subdivision splits the shared edges
where the overlay's boundary vertices land mid-edge, the rim
closes, and the output gate accepts the body (every edge exactly
two oppositely-oriented co-edges, χ even — the honest-solid
class). The alternative — a second computation of the same
geometric point by a different float path — is the exact defect
the IntersectionTable was built to kill (the grid-straddle
lesson); the overlay inherits the law, it does not re-learn it.

**5. THE COHERENCE RULES 3D↔2D** — the ones the exact path
already practices, named so the overlay obeys them explicitly:
(a) the FRAME rule — the plane's own frame, one isometry, never
two (the projection is exact; the polygon world's version of the
p-curve frame discipline, ADR-0032's one-opinion doctrine);
(b) the MATERIAL-SIDE rule — every emitted fragment winds CCW
around the surviving outward normal (the aligned pair's shared
+n̂; the same authoring the clip path preserves — and reverses
for inverted caps, boolean.cpp:2363-2370); (c) the ONE-POINT rule
(the pool, above); (d) the DETERMINISM rule — the canonical key,
input-order-independent, bit-identical regeneration (ADR-0014;
the SnappedKey quantizer IS the operation's ε_L, ADR-0035 D3's
one-value-two-consumers pin).

**6. LA CLASSE PARETE-NEL-PIANO — the derivation of why the exact
path's wall stays.** The polygon path's handling is ratified
(above: the frame tiling, the skip, the T-sewing — verified,
pinned by D5). The exact path's wall is KEPT, and the reason is
derived, not deferred out of fatigue:

- The G1 subtract's builder authors the pierced faces' INNER
  loops from the walk tables (subtract.hpp:113-133) — the
  annular pierced face IS the class contract. A wall-in-plane cut
  produces instead a NOTCH in the face's OUTER loop — the walk's
  re-authoring — and the walk tables have no row for it (the
  morse-4 register's own gap, ADR-0036 D5: "the joints'
  verticals have no vocabulary row yet"); the naming of a
  re-trimmed outer loop does not exist.
- The cavity's wall COINCIDENT with the target's face plane is a
  zero-thickness REGION contact — the dedup rule (D1's ratified
  semantics) has a row for CAPS (they vanish) but none for WALLS
  (a wall is not a duplicate boundary — it IS the boundary, and
  the honest disposition needs the region algebra of this ADR
  INSIDE the walk-table authoring).
- The class is therefore G2's by the code's own twice-written
  pointer — "the internal cap face class is G2's (it needs the
  SSI-driven cap insertion…)" (subtract.hpp:1183-1185) and "the
  non-annular pierced face is honest G2 topology, not G1"
  (:1148-1149) — and the scale agrees: no guide case reaches it
  (the hole strictly inside the footprint; the fillet on the
  rim). The wall stays REGISTERED, its refusal deterministic and
  pinned (the TANGENT band, :914's golden), and the dependency —
  the overlay's region algebra, available to G2's unit when it
  builds the SSI-driven cap insertion — is this ADR's gift
  forward, declared in D6.

### D4. The v1 scope — pinned to the scale, the method of ADR-0031 D5

Not the coplanar problem of a commercial kernel in one ADR — the
minimum the CURRENT scale consumes, with the queue named behind
it. The scale's question, asked honestly: **what do the guide
cases and the consumers actually reach?** The G1 chain (the
parametric hole), the UI's boolean on placed primitives, the
report's face queries. MAI "tutte": the inflated promise is the
defect that opens every ADR of this house.

**IN v1 (morso 2's door — one capability, three shapes):**

1. **The union's aligned coplanar pair with CROSSING regions**
   (P4 axis-aligned, P8 diagonal): the overlay emits F_A ∪ F_B as
   convex fragments. The pre-registered debt — the refusal's own
   named destination.
2. **The union's aligned coplanar pair with CONTAINED regions**
   (P11): the overlay emits the containing region — the union
   answers A whole when B is interior-flush. The same gate, the
   same algebra's containment cell (F_A ∪ F_B = F_A), no extra
   machinery.
3. **The ratification pins**: the parete union/subtract (P1/P2),
   the edge-touch union (P7), the opposed-crossing union (P10) —
   verified green today, pinned by D5 so they can never quietly
   regress. A capability nobody pins is a capability nobody
   kept.

**OUT of v1, each by name (the queue — the ADR-0036 D4 pattern),
each with its reason:**

- **The exact path's general coplanar contact** (the non-annular
  pierced face, the internal cap, the wall-in-plane): G2's
  registered classes — the walk-table re-authoring and the
  SSI-driven cap insertion (subtract.hpp:1148, :1183-1185's own
  pointers; D3's derivation above). The dependency is declared
  (the region algebra travels), the machinery is not built here.
- **The crease merge** (the impilato's lateral canonicalization:
  the stacked union's 10F answer vs the box's 6F): the creased
  body is a valid closed manifold, every golden, query and mass
  property reads it correctly, and NO consumer today
  differentiates 10F from 6F — the merge (face fusion with loop
  concatenation, the edge-touching degenerate of the overlay
  class) is its own derivation with its own risks, priced when a
  consumer arrives (the fillet's flank chains are the natural
  first).
- **The overlay's unification of the three operations' coplanar
  paths**: the subtract and the intersect keep their proven
  paths (pinned green since ADR-0025 v5); D3's table shows they
  already compute the right regions. Re-routing them through the
  overlay is a refactor with a blast radius and no capability
  gain — registered, not scheduled.
- **The naming consumption** (D6's grammar): the polygon path is
  nameless in v1 (ADR-0036 D4's own queue item); the overlay's
  born vocabulary is registered here for that item's day.
- **Curved coplanar pairs** (the coaxial cylinder's coincident
  wall, ADR-0033 entry 7's degenerate; the co-radial sphere
  pair, entry 5's "the coplanar rule's analog"): the G2 table's
  own territory — the polygon path is planar-faced by its input
  gate, and the exact path's curved classes are G2 units 2-6.

### D5. The discriminating tests — pinned BEFORE the code

The ADR-0034 D4 / ADR-0035 D5 / ADR-0036 D5 protocol verbatim:
every claim has the test that fails on the naive version, agreed
before the code. The counts below are DERIVED (the slab + T-junction
rules of D3, the existing disciplines cited by their pins), and
they are pinned HERE — morse 2 writes the tests from these
numbers, demonstrates the red on the current tree, and lands the
overlay green against them.

**THE DOOR'S GOLDENS (new, red on today's tree — the naive
counterproof IS the current refusal):**

1. **The grid-straddle union** (P4: A = [-1,1]³, B =
   [-0.5,1.5]×[-1,1]×[-1,1]): **V=8, E=12, F=6, χ=2**, closed
   manifold, loops closed, every edge exactly 2 opposite
   co-edges — the honest box. The merged ±Y region: x ∈ [-1,1.5],
   z ∈ [-1,1] (convex — one fragment per pair, no slabs); the ±Z
   pairs the same. Today: result_edge_not_manifold (pinned :586)
   — the golden is RED on the refusal: the discrimination, the
   door is new, not repainted.
2. **The diagonal union** (P8: B offset (0.5, 0, 0.5)): **V=20,
   E=32, F=14, χ=2**, closed, the same identities. The derivation
   the reviewer can check: the merged ±Y region is the stair
   (x,z) with vertices (-1,-1), (1,-1), (1,-0.5), (1.5,-0.5),
   (1.5,1.5), (-0.5,1.5), (-0.5,1), (-1,1) — 8 vertices, 2
   reflex — slab-split at x ∈ {-0.5, 1} into THREE convex
   rectangles per cap; the slab boundaries' endpoints at
   (-0.5,-1) and (1,1.5) land MID-BOUNDARY → the T-junction
   subdivision splits the v1-v2 and v5-v6 edges (one T-point on
   each, two pieces each), the other six boundary edges unsplit;
   the verticals exist at the 8 stair vertices only (the
   T-points carry none). V = 2 caps × (8 stair + 2 T) = 20;
   E = 2 × (10 boundary + 2 slab) + 8 verticals = 32; F = 2 × 3
   cap fragments + 8 side faces = 14; χ = 2. (The
   horizontal-slab variant gives the same census
   — the pin is the counts and the identities, not the fragment
   shapes; the shapes' bit-identity is determinism's own pin.)
   Today: refused ("8 of 40") — RED.
3. **The flush-interior union** (P11: B =
   [-0.5,0.5]×[-1,1]×[-0.5,0.5] inside A, tops flush):
   **V=8, E=12, F=6, χ=2** — A whole (B adds nothing; the
   containment cell emits F_A untouched — kept, in D6's
   vocabulary). Today: refused ("8 of 36") — RED.

**THE RATIFICATION GOLDENS (new tests, green today, green after —
the verified handling made un-regressible):**

4. **The boss union** (P1): V=20, E=32, F=14, χ=2, closed — the
   frame's four rectangles around the contact, the rim sewed to
   the boss's laterals.
5. **The flush notch subtract** (P2): V=20, E=32, F=14, χ=2,
   closed — the same census, the notch's cap inverted.
6. **The full-width boss union** (P7): V=16, E=26, F=12, χ=2,
   closed — the edge-touching aligned pairs both survive, the
   T-subdivision closes the shared edges.
7. **The across-contact union** (P10): V=16, E=26, F=12, χ=2,
   closed — the opposed-crossing symmetric difference, the two
   strips.
8. **The existing pins re-asserted verbatim, no re-pin**: the
   stacked union 12/20/10 (:322), the stacked intersect disk
   4/4/1 (:301), the grid-straddle subtract 8/12/6 (:443), the
   corner notch 17/27/12 (:348), the slab :463, the diagonal
   intersect 8/12/6 (:796's intersect half).

**THE WALLS' HONEST REFUSALS (green, with payloads — the wall
that stays is declared, not happened):**

9. The exact path's four, re-asserted verbatim: CROSSED (:952),
   TANGENT bit-identical across re-runs (:914), BLIND (:973),
   SWALLOW (:1037) — the messages' own words are the golden.
10. The union's input-gate refusals (validate_boolean_input's
    payload classes) — untouched by the overlay (the gate runs
    before any pair logic).

**THE GOLDEN DECLARATION (pre-registered, before any code):**

- **The H2-u2 pair re-pins, BY NAME, ONCE** — the wall's own
  witnesses become the door's own goldens: test_boolean.cpp:586
  ("the open shell is refused, not returned") and :619 (the rim
  census in the refusal text) re-pin from the refusal to the
  answer (the P4 counts above; the census assertion maps to
  "0 of 12 edges carry a single co-edge"); test_boolean.cpp:796's
  union half (CHECK_THROWS) re-pins to the same answer — its
  intersect half STANDS. The reason rides the feat commit's body,
  and the reason was registered HERE, before the code existed.
- **The refusal message's own retirement, declared**: the
  open-shell message names "the known PARTIAL COPLANAR OVERLAP
  class … until H2 unit 5 sews the shared-face class the refusal
  IS the answer" (boolean.cpp:1553-1556) — after the door, the
  known class is sewn and the sentence is stale. Morse 2 rewrites
  the message to the gate's generic diagnosis (an unforeseen
  open rim is a kernel bug to report, the remedy says so); the
  class's name leaves the code the same commit the class leaves
  the wall. Declared here so the review can check the wording's
  honesty in the act.
- **No other golden moves.** The transversal paths are untouched
  byte for byte (the gate only REROUTES the admitted pairs; the
  clip, the slab and the T-junction code paths run the same
  instructions on the same inputs for every non-coplanar pair).
  The exact path is untouched entirely — morse 2's code radius
  is the union's pair seat alone. A golden that moves outside
  the named re-pins is a defect, not a re-pin.
- **No golden is relaxed** (ADR-0034 D4's own words, ratified).
- **Determinism from the first landing**: N = 10 rebuilds of
  every new golden's body, byte-identical (the digest pattern),
  on both compilers — the canonical pool and the deterministic
  walk make it true by construction; the tests pin it so it
  stays true by contract.

### D6. The consequences — the vocabulary, the register, the ladder

**The naming vocabulary for the curves born in the overlay — the
grammar added WITHOUT touching the mothers (ADR-0036 D6's own
protocol, applied in reverse: the extension lives HERE, the
mothers stay as written, the consumption is queued).** When the
polygon path gains its tavola (ADR-0036 D4's queue item), the
overlay's entities have their rows ready:

- **The merged region** (the aligned pair's union region, the
  crossing case): a BORN face whose genealogy names both
  operands — the relational form gains the union operator:
  `ovl[<A-face-name> ∪ <B-face-name>]` (render: one logical name
  per region, however many convex fragments materialize it — the
  fragments are the name's materialized form, the rim-circle
  precedent of ADR-0023/0036: ONE name, many halves). The
  containment case needs NO row: the containing face is KEPT
  (survives under its own name — D1's ratified semantics, the
  cap-flush rule's overlay analog); the contained face vanishes.
- **The crossing edges** — the segments where the two loops cross
  in the plane (the degenerate SSI's born curves):
  `ovl:cross[<A-face-name> ∪ <B-face-name>]` — the same genealogy,
  the SSI entries' own discipline (the entry names the pair; the
  level suffix is absent: the plane IS the level).
- **The kept/absorbed dispositions travel from D1**: the frame's
  fragments of a RATIFIED case (the parete) are the containing
  face's own materialization — kept, re-trimmed, its name
  unchanged; the born rows exist only for the door's new regions.
  The grammar's tokens (`ovl`, `cross`) follow the `[a-z][a-z0-9]*`
  law; the operator `∪` is the grammar's first two-operand
  boolean glyph — declared here, one row in the renderer's table
  when the consumer arrives.

**The tolerance register — the fourth member.** ADR-0035 D2's
table gains the coplanarity row (D2's table above) — amended in
place, dated, pointing here. The vocabulary now audits four
derived members: the determinant band (length²), the winding
family (ε_w,noise, ε_w), and the coplanarity band ε_C — each a
form, each with its derivation one citation away.

**The register surgery — declared; applied in the same act as
acceptance** (the ADR-0035 D6 protocol):

- **ADR-0025 — amended in place, dated, pointing here**: §3's
  "Partially overlapping … a documented limitation … (deferred)"
  and §7's "subdivision of coplanar-overlapping FACES remains
  deferred" are RESOLVED by this ADR: the deferral's payment is
  D3's overlay, the class's door is D4's scope, the witnesses are
  D5's goldens. The historical sections stay legible with the
  amendment pointing forward.
- **ADR-0035 — amended in place** (D2's row, above).
- **ADR-0036 — NOT amended**: the grammar extension lives here
  (D6's first section); the queue item it feeds (the polygon
  path's tavola) already exists in 0036 D4's own list. No mother
  touched, the protocol's own words honored.
- **ADR-0031 — NOT amended**: the G2 boundary stands; the
  exact-path coplanar classes stay G2's (D3's derivation, D4's
  queue). This ADR registers the dependency (the region algebra
  travels to G2's cap-insertion unit) without moving the phase
  line.
- **ADR-0009 — NOT amended**: the overlay is serial, like the
  whole polygon path; no parallelism is added in secret, and the
  day ADR-0009 declares the path parallel-clean the overlay
  rides the declaration, not the reverse.

**LA LADDER dei morsi 2+** (the unit is DI 2: morse 2 is the last,
one delivery, landing standalone with the full suite green — the
house sequence, each internal step a verifiable station):

1. **The gate** — the pair-level coplanarity test (D2's four
   steps) seated in the union's process_faces, verdict-only: the
   admitted pairs counted, the behavior unchanged (the overlay
   not yet consuming) — the smallest vertical slice, the gate's
   determinism pinnable alone.
2. **The overlay core** — the projection, the crossing
   computation through the canonical pool, the union region's
   boundary walk, the convex-fragment emission (the slab
   discipline), the sewing; the door's three goldens (D5.1-3)
   green; the H2-u2 pair and :796's union half re-pinned; the
   refusal message retired (D5's declaration).
3. **The ratification pins** (D5.4-7) and the walls'
   re-assertions (D5.9-10) — the exit criterion: every golden of
   this ADR green, the naive counterproof demonstrated red on the
   pre-overlay tree (the red run cited in the feat commit's
   body), the full suite on both compilers, CI all green.

The STOP after this delivery is the order's own: **the ADR passes
the maintainer's review before a morsel of code exists.**

## Consequences

- The coplanar zoo stops being folklore: eleven placements
  verified, four classes typed, the one refusing brick named with
  its structural reason — and three of the four classes were
  already doors nobody had pinned (the parete, the edge-touch,
  the opposed-crossing: verified, and by D5's pins, kept).
- The union's aligned pair becomes a capability with a derived
  contract: the gate (two typed members, no new constants), the
  algebra (a 2×3 table, every cell cross-checked on a verified
  case), the emission (convex fragments — the composed chain
  stays in-class), the sewing (one point, one instance).
- The exact path's wall keeps its honesty and gains a stated
  price: the refusal is deterministic and pinned, the reason is
  derived (the walk-table authoring, the region-contact
  disposition), and the dependency — the overlay's algebra — is
  registered for G2's cap-insertion unit instead of being
  smuggled.
- The tolerance vocabulary gains its fourth member with the
  sentinel's derived number: the near-band transversal line is
  trusted to 707·ε_L at the border, and that bill is now a
  citation, not a surprise.
- The H2-u2 debt — pre-registered in the code's own refusal
  message — is paid by name: the witnesses re-pin by declaration,
  the message retires with its class, and the re-pin's reason was
  registered before the code existed. That is the whole point of
  writing the contract first.

## References

- ADR-0023/0024 (the coplanar dedup rule's origin — the
  cap-flush semantics D1 ratifies), ADR-0025 (v1 §3's deferral,
  v5 §8 the parallel-plane rule, §10 the T-junction subdivision,
  §6 the IntersectionTable — the overlay inherits all four),
  ADR-0031 D5 (the phase ladder and the one-guide-case method),
  ADR-0033 (the G2 boundary; entries 5/7's curved coplanar
  degenerates, registered out of v1's scope), ADR-0034 D4 (the
  golden re-pin protocol), ADR-0035 (D2 the vocabulary D2's row
  joins; D2.3 the band method; D3 the canonical key; D5/D6 the
  protocols), ADR-0036 (D1 the grammar; D4 the queue pattern; D5
  the counterproof protocol; D6 the register-surgery protocol).
- The code sites, verified at 6ab0cf8: subtract.hpp:68-69 (the
  dedup rule), :113-133 (the walk tables), :1046-1057 (the
  z-side miss), :1080-1095 (the SAT touch), :1104-1110 (the
  swallow), :1140-1152 (the CROSSED refusal), :1153-1169 (the
  TANGENT refusal), :1175-1186 (the BLIND refusal), :1193-1200
  (the dedup's emission); boolean.hpp:70-152 (the typed bundle,
  the house values); boolean.cpp:366-452 (the segment machinery,
  the H1 arms), :1166-1174 (polygons_coincident), :1215-1256
  (classify_parallel_plane — the three-case band and the straddle
  fallback), :1462-1534 (the output gate's honest classes),
  :1546-1564 (the open-shell refusal that names this unit),
  :2184-2213 (process_faces — the gate's seat), :2325-2340 (the
  v5 cap skip); test_boolean.cpp:301, :322, :348, :422, :443,
  :463, :515, :586, :619, :796; test_g1_unit4_subtract.cpp:892,
  :914, :952, :973, :1037.
- The wall map's unpinned placements (P1, P2, P7, P8, P10, P11)
  verified by direct execution on the tree at 6ab0cf8 — the
  factories' boxes, the house bundle, the counts and the closure
  read from the returned bodies; D5 pins them so the verification
  outlives the session that performed it.
