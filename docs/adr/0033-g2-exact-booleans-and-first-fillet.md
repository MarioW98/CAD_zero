# ADR-0033: G2 — the analytic SSI table, the exact booleans, and the first fillet

## Status

**Accepted (2026-10-26) — the review closed the same day the draft
shipped, with the reserved decision TAKEN: D2 = option (i), the table
is the wall.** Daughter of ADR-0031 (accepted: road (a), all in
house, forever — its G1 internal checkpoint recorded the same day
this draft shipped: the estimates stand) and of ADR-0032 (implemented:
the exact spine G2 builds on — p-curves, Layer-2 provenance, χ =
V−E+F−H, the HybridBody, the report as the single consumer). The
draft pinned the scope with **one derivation per table entry**, the
fillet's derived class with its counts and volume, the degenerate
dispositions, the unit ladder and the no-promises list — and it
carried **ONE DECISION RESERVED TO THE USER** (D2: the table
boundary), which the acceptance records below: **option (i)**, with
option (ii) REGISTERED as a priced candidate carrying its entry
criterion (not dead — awaiting a real case, decided at the next
internal checkpoint). The ladder runs as written: **six units**.
The house method held as ever: invariants and discriminating tests
were pinned in this document, agreed in review, BEFORE the phase
began; the review moved nothing but D2's lever, and carried one
correction to the record (row 7's unequal-radii note, corrected
below where it stood).

## Context

**Where G1 left the tree (verified, not assumed):**

- The exact chain runs end-to-end on its pinned class: the sketch
  profile → the prism with true faces → the through-hole subtract
  with the plane∩cylinder SSI entry (the ⊥ circle and the ∥ line
  pair; the oblique case honestly refused) → the divergence-theorem
  mass properties → the report (mass + provenance beside χ and
  genus) and the printed, CI-pinned D7.5 table. The HybridBody owns
  both materializations on the extrude and subtract nodes; the
  format did not move (recipes are the truth, the B-Rep is cache).
- The authored bodies carry exactly three surface families: planes,
  cylinders (the Z-axis extrude walls and the factory's Y-axis
  primitive), and the full sphere. Cone and Torus exist as untrimmed
  carrier classes with **no writer** (registered leftovers of G1
  unit 2, awaiting the units that first trim them).
- The p-curve vocabulary is `Line2`/`Circle2`/`Arc2` — the sketch
  kernel's own types, one opinion (`pcurve.hpp`); the 3D curve set
  is `LineCurve`/`CircleCurve`/`EllipseCurve`.

**One correction this draft owes the record, discovered while
deriving the table:** ADR-0032 D5.3's parenthetical — "the ellipse
an `Ellipse2` (the vocabulary of D2 covers them)" — never landed. No
`Ellipse2` exists: D2.1 pinned three p-curve types, the sketch
kernel authors three, and the oblique case that sentence described
refuses on the *cylinder's* domain anyway, so the claim was never
exercised. It is harmless in G1 and **load-bearing in G2**: under
the admission criterion below, the typed table admits no
ellipse-bearing entry at all. The ellipse re-enters geometry only
through D2's option (ii) or through G3's splines — never through a
quiet vocabulary edit.

**What G2 is named for** (ADR-0031 D5's rung): the analytic-pair
SSI table to the level the product demands; the booleans on curved
faces under the D4.2 guarantees; and **the first constant-radius
edge fillet** — the first feature that is impossible today, and
therefore the kernel's proving ground. The elephant is eaten one
bite at a time — the ladder is the method, the checkpoint priced the
animal, and this ADR is the next bite, sized honestly: the table's
bounded entries, one fillet class, the degenerate corpus around
them, and the explicit refusal of the tail (vertex blends) that the
price list says dominates the years.

## Decision

### D1. Scope — the table, the booleans, the fillet; and the explicitly out

**In:**

1. **The analytic SSI table, one derivation per entry** (D1.1): the
   typable pairs the authored class can reach — plane×sphere,
   sphere×sphere, cylinder×sphere coaxial, cylinder×cylinder
   parallel/coaxial — plus the axis-aligned adjacencies the fillet
   itself creates (plane×torus ⊥ axis, cylinder×torus coaxial),
   each entry carrying its closed form, its p-curves in **both**
   parameter domains, and its degenerate limits.
2. **The exact booleans on the extended class** (D2 is their
   boundary): union/intersect/subtract where every curved pair the
   operands carry is in the table — sphere×sphere (the classical
   union), sphere±prism, prism×prism with parallel axes (the
   two-holes body), sphere±box — under ADR-0031 D4.2's guarantees:
   Layer-1 invariants exact, output provenance worst-of-operands,
   tie-breaks deterministic per canonical key, bit-identical
   re-runs.
3. **The constant-radius rolling-ball fillet on the derived class**
   (D3): the box-edge fillet (plane×plane → a cylindrical flank)
   and the hole-rim fillet (plane×cylinder ⊥ at the convex rim →
   a toroidal flank) — **the guide case is the G1 body's own rim**:
   the same chain, one more feature, the table extended by exactly
   one row.
4. **The mass closed forms for the new face families** the above
   produces — the trimmed sphere and the toroidal patch — the
   unit-5 machinery extended; the classical formulas are again the
   tests, never the implementation.
5. **The UI surface and the guide case**: Feature → Fillet; the
   report says everything (the flank's family in the face query,
   the counts, the provenance); the guide case end-to-end with the
   printed, hand-checkable table; the phase's exit is the user's
   click-test (the ADR-0030/0032 lesson, the third phase running).

**Out (explicit, so the review can check the boundary):**

- **Vertex blends** — the corner closure where two or more fillet
  surfaces must meet: the tail's worst face, per ADR-0031 D4.3.
  Unscheduled; the phase is valid without it; every collision is
  refused with the remedy (D4).
- **The spline vocabulary** — any p-curve outside
  `Line2`/`Circle2`/`Arc2` is G3's layer. The out-of-table pairs go
  to D2's decision, never past it; a smuggled sinusoid is exactly
  the silent corruption the provenance contract exists to prevent.
- **Cone authoring** — no writer exists and none is designed here
  (a tapered extrude is a feature decision of its own); the
  plane×cone mathematics is derived in D1.1 for the record and
  stays unreachable, registered.
- **STEP** (ADR-0010 waits for its phase); **G3 free surfaces**;
  the oblique plane×cylinder (G1's D5.3 refusal, unchanged — and
  re-derived below as a *consequence* of the admission criterion,
  which is the point of having one); everything else.

### D1.1. The table — one derivation per entry, one admission criterion

**The admission criterion (the G1 lesson made a rule).** An SSI
entry is *typable* — admits an exact implementation in the pinned
vocabulary — if and only if:

1. the 3D intersection curve is in {`LineCurve`, `CircleCurve`,
   `EllipseCurve`} — a conic;
2. the p-curve is typed in the parameter domain of **each** face
   that owns it — `{Line2, Circle2, Arc2}` in the stored frame, or
   in a frame the operation may deterministically re-author (the
   re-frame rule, below).

Criterion (2) is the stricter one, and the Steinmetz special case
below proves the strictness is real: a pair whose *space* curve
degenerates to conics can still be untypable in the domains. G1's
D5.3 boundary is the rule's first instance: the oblique
plane∩cylinder is an `EllipseCurve` in space and would be an
ellipse on the plane's domain, but on the cylinder's (θ,z) domain
it is z(θ) = z₀ + r·cot θ̃·cos(θ−θ₀) — a sinusoid; no typed p-curve;
refused. The criterion generalizes that refusal from one pair to
the whole table, and the generalization is what makes it honest
rather than habitual.

**The re-frame rule (revolution surfaces).** A sphere's latitude
circle is a `Line2` (v = const) only in a frame whose pole is the
cut plane's normal; the factory's stored pole is arbitrary, and a
placement-dependent refusal would be absurd — the geometry is fine,
only the stored frame argues. The rule: **an operation may re-frame
a surviving revolution face** — rebuild it on the same mathematical
surface with the pole (the axis) aligned to the operation's
canonical axis, re-authoring all its loops in the new domain, the
seam re-wired per the ADR-0023 pattern — because the parametrization
is derived state (the ADR-0002/0031 doctrine: derived state serves
the geometry, never the reverse) and the recipe never records
frames, so the re-frame is a pure function of the recipe and
bit-identical under regeneration (ADR-0014). **The frame budget,
derived:** one re-frame gives the face ONE axis family; every typed
cut on that face must be a latitude of that axis (planes ⊥ to it,
coaxial cylinders, cones). A later operation adding an incompatible
cut family to the same face finds no typed frame — it is
out-of-table, refused with the remedy (or subdivided, if the user
picks D2's option (ii)). The budget is a real boundary, not a
implementation shortcut: two general direction cuts of one sphere
have no common typed frame, and the boundary says so once, for
every entry, instead of per-defect.

**The entries, each a derivation (the reachable set — pairs the
authored class can actually produce):**

| # | Pair (disposition) | 3D curve, derived | p-curves, both domains | Limits / degenerates |
|---|---|---|---|---|
| 0 | plane×plane | line | `Line2` + `Line2` | landed (Phase D) |
| 1 | plane×cylinder ⊥ | circle r, at the axis∩plane | `Circle2` (plane) + `Line2` v=h (cyl) | landed (G1 u4) |
| 2 | plane×cylinder ∥ | line pair at ±θ₀ | `Line2` + `Line2` (θ=const) | tangent: double line; far: empty — landed (G1 u4) |
| 3 | plane×cylinder oblique | ellipse (r, r/sinθ) | ellipse ✗ (plane, would need `Ellipse2`) + sinusoid ✗ (cyl) | **refused** — the criterion's G1 instance |
| 4 | plane×sphere (d<R) | circle √(R²−d²) at the foot | `Circle2` + latitude `Line2` (re-frame to the plane normal) | d=R: tangent point; d>R: empty |
| 5 | sphere×sphere (non-nested, d<R₁+R₂) | circle in the **radical plane** (subtract the equations: 2(c₂−c₁)·x = ‖c₂‖²−‖c₁‖²+R₁²−R₂² — a plane ⊥ to the center line, then entry 4) | latitude `Line2` on each sphere (re-frame each to the center line) | nested: empty; internal/external tangency: a point; equal spheres: the coincident-face degenerate (the co-radial dedup rule, the coplanar rule's analog) |
| 6 | cylinder×sphere coaxial (r<R) | two circles at z=±√(R²−r²) | `Line2` z=const (cyl) + latitude `Line2` (sphere, re-frame to the axis) | r=R: one tangent circle; r>R: empty |
| 7 | cylinder×cylinder parallel, equal r, offset d<2r | **two lines** in the mid-plane, y=±√(r²−d²/4) (subtract: x=d/2) | `Line2` θ=θ₀,θ₁ on each | d=2r: the tangent double line; d>2r: empty; coaxial equal: coincident faces (the degenerate); unequal radii — **corrected at acceptance** (D2's correction block): the transverse case \|r₁−r₂\|<d<r₁+r₂ is two TYPED lines in the offset plane x=(d²+r₁²−r₂²)/(2d), admitted by the criterion; out of the registered entry (equal r) until the unit that consumes it |
| 8 | plane×cone ⊥ axis | circle at constant slant t | `Circle2` + `Line2` t=const | through-apex: the ruling line pair (`Line2`+`Line2`); tangent: the double ruling; **oblique: the conics — refused** (the cone-domain p-curve is t(θ)=d/(n·û(θ)), the secant family — the sinusoid's sibling); **unreachable today: no cone writer** |
| 9 | plane×torus ⊥ axis, \|z\|<ρ | two concentric circles a±√(ρ²−z²) | `Circle2` + `Line2` φ=const-pair (torus) | \|z\|=ρ: one tangent circle; reachable only via the fillet (D3) |
| 10 | cylinder×torus coaxial (\|R−a\|<ρ) | circles at φ=const: a+ρcosφ=R | `Line2` z=const + `Line2` φ=const | the fillet's own adjacencies (the tangency circles of D3 — built by construction, consumed by the table); reachable only via the fillet |

Every row is a derivation the unit's tests pin against the
classical formulas (the circle radii, the radical plane, the
mid-plane lines), and every degenerate carries its disposition in
D4's corpus. The symmetric images (pair order, operand roles) are
the same entries.

### D2. The table boundary — the decision that opens or closes the years, RESERVED TO THE USER

**The derivation, from first principles.** Two quadric surfaces
intersect in an algebraic curve of degree at most 4 (Bézout: the
quadric pair in projective space; Levin's pencil method is the
classical reduction). The pinned vocabulary — lines, circles,
arcs in the domains; lines, circles, ellipses in space — is a
**conic** vocabulary. The pairs of degree > 2 therefore fall
outside it, and the fall is not a matter of effort:

- **cylinder×cylinder, non-parallel and non-coaxial:** the general
  intersection is an irreducible quartic space curve. The named
  special case cuts both ways: the *Steinmetz* disposition (equal
  radii, perpendicular axes) degenerates into **two ellipses**
  (subtract the equations: y²=z²; each ellipse is a plane section)
  — but on each cylinder's domain the p-curve is z = r·sin θ, the
  sinusoid again. **The space-curve degeneration does not rescue
  the domain curve: the boundary is the domain, and the criterion
  holds even where Bézout's bound softens.**
- **sphere×cylinder, non-coaxial:** a quartic; its tangent member
  is **Viviani's curve** (the sphere and the cylinder touching the
  diameter — the classical closed quartic with a figure-eight
  planar projection). Spherical sinusoids in both domains.
- **plane×torus, non-⊥:** the torus is a quartic surface; a plane
  section is a plane quartic — the **spiric sections of Perseus**
  (the axis-parallel subfamily; the Hippopede, the Cassini ovals
  and the Bernoulli lemniscate are its named members). No member is
  a conic; none is typable.
- The remaining pairs (cone×cone general, torus×sphere, torus×torus)
  are out on the same rule and are not derived further here — no
  authored body reaches them, and the boundary says so once instead
  of per pair.

**The proposal — two ways, no third.** The ADR does not choose; the
user does, in review, because the choice sizes the phase:

- **Option (i) — the table is the wall.** The exact booleans run
  only where every curved pair is in D1.1's table; anything else is
  **refused with an actionable error** naming the pair, the
  disposition class, and the remedies (reorient the operands, or
  accept the faceted path, or await G3). This is G1's precedent
  (the oblique refusal) generalized to the whole zoo. Cost: the
  phase is *months of table* — the bounded entries, the fillet, the
  corpus — and nothing more; the capability ceiling is explicit
  and queryable (the refusal is a recorded answer, not a missing
  one).
- **Option (ii) — the table, plus declared subdivision.** The
  out-of-table pairs compute their intersection by **adaptive
  subdivision with interval bounding** (the `Interval` type in
  `core/math`; ADR-0031 D4.1.3's own strategy, made concrete),
  producing a curve that is **`approximated(tol)` in the Layer-2
  provenance field G1 unit 1 planted** — carried, never laundered:
  the report says approximated with the tolerance, the mass flag
  `all_exact` says false with the polyhedral-honest value, the
  serialization is deterministic (canonical-key tie-breaks, ADR-0014
  bit-identical re-runs). Cost: the same months of table, **plus
  the subdivision engine** — and one data-model consequence the
  review must see with open eyes: an approximated intersection
  still needs a p-curve in each owning domain, and a polyline is
  not a `Line2`. Option (ii) therefore admits **one honestly
  approximate p-curve citizen** (a typed, construction-validated
  sampled curve carrying its tolerance in the provenance field,
  never presentable as exact — the faceted world's vocabulary,
  brought into the trimming world behind the same fence the bridge
  already keeps). That is a vocabulary extension G1 never made;
  it is defensible, it is priced, and it is the user's call.

**There is no third way.** A silent spline is G3 smuggled in; a
silent mix is what the provenance contract exists to prevent; an
"exact-ish" quartic is a lie with a tolerance costume on. The
boundary is a wall or a fence — never a curtain.

**Decision on acceptance (2026-10-26) — D2 = option (i): the table
is the wall.** The user's decision, taken in review the same day the
draft shipped:

- **Option (i) stands.** The exact booleans run only where every
  curved pair of the operands is in D1.1's typed table; anything
  else is **refused with an actionable error** naming the pair, the
  disposition class, and the remedies (reorient the operands, accept
  the faceted path, or await G3). The capability ceiling is explicit
  and queryable — the refusal is a recorded answer, not a missing
  one. The ladder runs as written: six units, no unit 2.5.
- **Option (ii) does not die — it is REGISTERED as a priced
  candidate with its entry criterion.** The criterion: a real case
  demanded by an author body (for instance: two non-parallel
  cylindrical walls met in a sketch chain — the Steinmetz
  disposition, typed in neither domain). The decision is taken at
  the **next internal checkpoint**, with the table and the fillet
  measured on the table — empirical, never philosophical. The price
  stands as derived above: the subdivision engine on the existing
  `Interval` type plus the one honestly-approximated p-curve citizen
  behind the same fence the bridge already keeps, landing as **unit
  2.5** between the table and the booleans, priced as its own
  delivery. Until the criterion fires, the register says no to
  subdivision — and says it in the refusal, by name.
- **The user's motivation, recorded verbatim:** «il muro è sempre
  onesto; la siepe, una volta piantata, va sorvegliata per sempre;
  prima il muro, la siepe quando un caso vero la domanda.» ("the
  wall is always honest; the hedge, once planted, must be watched
  forever; first the wall — the hedge when a real case asks for
  it.")

**One correction carried by the acceptance, owed to the record
(row 7's unequal-radii note).** The drafted limit cell read
"unequal radii: empty, or the external/internal tangency line" —
and that disposition is wrong in the transverse interior: two
parallel cylinders of UNEQUAL radii with |r₁−r₂| < d < r₁+r₂
intersect in **two lines** lying in the offset plane x =
(d²+r₁²−r₂²)/(2d) (subtract the equations: a plane ⊥ to the
offset direction; the lines at y = ±√(r₁²−x²)), and the p-curves
are u = const on **both** domains — the admission criterion
ADMITS the pair. The drafted note covered only the empty exterior
and the tangency boundary, and missed the interior; the derivation,
not the draft, is the authority the house recognizes. The
correction's consequence for the unit that lands the table: the
entry implements the **equal-radii class as scoped** (the order's
own boundary), and the unequal pair is refused **at the entry's
gate** — with the refusal's disposition class named honestly (out
of the registered entry, the transverse case typable and awaiting
the unit that consumes it), never as "untypable", which it is not.

The consequence for the ladder: under (i), D5 runs as written, six
units; under (ii), the subdivision engine is **unit 2.5** (between
the table and the booleans) and the ladder grows by exactly that
unit — priced as its own delivery, not amortized into the booleans.

### D3. The fillet — constant radius, rolling ball, on the derived class

**The class is derived, not aspirational.** A constant-radius
rolling-ball fillet is typable exactly where (a) the two adjacent
faces belong to the D1.1 world, (b) the edge is convex (the ball
rolls on the material side, inscribed), and (c) the swept flank is
an analytic surface the family already carries. That derivation
yields **two classes**, and only two, for the whole of G2:

1. **plane×plane (the box edge)** — the ball inscribed in the
   90° corner sweeps a **cylinder** of radius ρ (the classical
   result; the flank is a `CylinderSurface`).
2. **plane×cylinder ⊥ at the rim (the hole edge)** — the ball
   inscribed at the convex rim sweeps a **torus**; the flank is a
   `TorusSurface`, which exists in the analytic family since Phase D
   and takes its writer here, with its first trimming loops — and
   with them the registered leftover G1 unit 2 pinned: the torus
   frame is still the left-handed exception among the analytic
   surfaces, and it flips to the one right-handed rule the day its
   first loop is authored, the same one-line correction the cylinder
   received, with its own discriminating pins.

Everything else — an edge between non-typable faces, a concave
(reentrant) edge, a fillet asked on a seam — is refused with the
pair named and the remedy (D4).

**The rim fillet, derived in full (the guide case).** The G1 body:
box a×b×c, through-hole radius r, the wall a `CylinderSurface`, the
pierced faces annuli. Fillet the top rim with radius ρ. The ball
(inscribed, tangent to both faces) has its center at distance r+ρ
from the axis and depth ρ below the rim plane; as it rolls around
the rim the center sweeps the circle of radius **r+ρ** and the ball
sweeps the solid torus of **major radius r+ρ, minor radius ρ** —
the order's shorthand "maggiore r" reads exactly r+ρ once derived:
the center rides ρ outside the wall. The flank is the quarter of
the tube facing the rim, between the two tangency circles:

- **on the perforated face:** the circle of radius **r+ρ** (the
  ball touches the plane perpendicularly — the tangency point sits
  directly above the center);
- **on the wall:** the circle of radius **r at depth ρ** (the
  tangency point sits directly inward of the center).

Both are `CircleCurve`s in space; both domain p-curves are typed
(the plane's `Circle2`; the torus's `Line2` φ=const pair — entry 10
of the table, consumed by construction).

**Is the rim fillet closed? Yes — the counts, derived under the
G1 seam conventions** (each rim circle splits at the seam pair; the
periodic flank carries its own seam, the ADR-0023 pattern):

| Quantity | Before (G1, D7.5) | After (one rim) | The arithmetic |
|---|---|---|---|
| V | 12 | **14** | −2 rim split vertices, +4 tangency split vertices (2 on the face at r+ρ, 2 on the wall at depth ρ) |
| E | 17 | **20** | −2 rim arcs, +4 tangency arcs (2+2), +1 the flank's seam |
| F | 7 | **8** | +1 the toroidal flank |
| H | 2 | **2** | the pierced face keeps exactly one inner loop — now the r+ρ tangency circle |
| χ | 0 | **14−20+8−2 = 0** | Δχ = +2 −3 +1 −0 = **0** |
| Genus | 1 | **1** | the handle survives: the fillet moves a boundary, it never cuts a new one |

Both rims filleted: **16/23/9/2, χ = 0, genus 1** (the same
arithmetic twice). **The invariant is unperturbed by construction —
and that is the derivation's verdict: the rim fillet is topologically
flat (χ-preserving), exactly like the box-edge fillet below.**

**The volume, hand-checkable (the torus term the order asked
for).** The removed material, per azimuth θ, is the 2D corner
region between the sharp rim and the flank arc: the window
[r, r+ρ] × [c−ρ, c] minus the quarter disc of radius ρ centered
(r+ρ, c−ρ) (the quarter facing the corner — the ball's
cross-section). Integrating with the cylinder measure s·ds·dz·dθ:

```
V_ρ = 2π [ (rρ² + ρ³/2)  −  ( (r+ρ)·πρ²/4 − ρ³/3 ) ]
    = 2π ( rρ² + 5ρ³/6 )  −  (π²/2)(r+ρ)ρ²
```

The first term is the corner-window ring prism; the subtracted
term is **a quarter of the full torus 2π²Rρ² with R = r+ρ** — the
torus term, on the surface of the formula; the −ρ³/3 inside is the
measure correction (the disc does not sit at constant radius).
Checks the golden pins: for r ≫ ρ the formula reads
2πrρ²(1−π/4) — the 2D corner cut (window ρ² minus quarter disc
πρ²/4) revolved at radius r, as it must; the guide case's new
volume is **abc − πr²h − V_ρ**; the centroid stays on the axis by
symmetry; the axial inertia shifts by the same quarter-disc
integral family the unit-5 recurrences already evaluate in closed
form (the golden pins the combined number).

**The box-edge fillet, the classic class (and one correction to
the mother's sketch).** A vertical box edge of length ℓ, filleted
at radius ρ: the flank is the cylinder radius ρ; the tangency
lines run parallel to the edge at distance ρ on each adjacent
face; the profile closes with two quarter-arc edges (top and
bottom). Counts, derived: the edge and its two corner vertices
die; four vertices and four edges are born (2 tangency lines + 2
quarter arcs); one face is born. 8/12/6 → **10/15/7, χ = 2**
(Δχ = +2−3+1 = 0 — flat again). Volume removed: **ℓ·ρ²(1−π/4)**
(the straight prism of the same 2D corner cut — no measure term).
The correction, owed to ADR-0031 D5's guide sketch ("three planar
faces retire"): the derivation says **no face retires** — two
adjacent faces shrink, the two end faces lose a corner each, one
cylindrical face is born; the mother's "four new edges" was right,
its "three faces retire" was the slip. The daughter corrects it
here, on the record, because the counts are the golden's spine.

**Where the fillet lands (the HybridBody, again).** The fillet
node owns both materializations, bridge tolerance 0. The exact
side is the construction above. The SDF twin is **the classical
corner-cut CSG** — the filleted solid is body ∩ ¬(window ∖ tube),
with the window the revolved corner rectangle (a cylinder-shell
ring: cylinder_{r+ρ} ∖ cylinder_r ∩ slab) and the tube the swept
ball-solid — which requires **one new SDF primitive: the torus**
(f = (length(p.xz) − R)² + p.y² − ρ², classical and exact-signed;
its bounds and Lipschitz constant derived under the ADR-0028/0029
discipline, in the unit that lands it). The display path keeps its
architecture; the `smooth_min` blend is *not* the fillet, has no
exact radius, and the UI never labels it so (ADR-0031 D4.3's line,
restated because it is the honest boundary between the two
worlds' strengths).

**The mass families the fillet forces.** A body with a toroidal
face must answer numbers or refuse — and refusing is not
acceptable for the guide case. The unit-5 machinery extends to:

- **the toroidal patch**: the (θ, φ) rectangle integrals on the
  parametrization x(θ,φ) = ((R+ρcosφ)cosθ, (R+ρcosφ)sinθ, ρsinφ)
  — the same recurrence family the cylinder patches already use;
  the classical check is the full torus, V = 2π²Rρ² (the flank is
  a φ-window of it — the D3 table's V_ρ is the removed-quarter
  face of the same arithmetic, and the two derivations must agree:
  that agreement is a discriminator test);
- **the trimmed sphere** (sphere booleans of D1's class produce
  them): the spherical segment and cap closed forms — the
  classical πh²(3R−h)/3 family — are the tests.

### D4. The degenerates of the fillet — each with a disposition, pinned

- **Radius too large for the topology** — three named families,
  each an actionable error carrying the offending bound and the
  remedy (shrink ρ or restructure):
  the tangency line exits the adjacent face (box edge: ρ meets the
  face's extent); the tangency circle exits the annulus (rim:
  r+ρ reaches the perforated face's outer boundary — the
  non-annular pierced face, G1 D7's circle-crossing analog);
  the fillet eats the wall (rim: ρ ≥ h, the tangency circle
  passes the far rim).
- **Two fillets colliding at a corner edge** — the flanks would
  meet at the corner, and the closure patch **is the vertex
  blend**: refused, the remedy named (one edge, a smaller radius,
  or the unscheduled tail). This is the honest G2 refusing the
  worst of the cliff on purpose, at the door.
- **Tangential collapse and slivers** — ρ exactly at a bound, or
  a near-degenerate tangency strip: the deterministic
  canonical-key tie-break decides, the Layer-3 tolerance band is
  declared (ADR-0004), and each family is golden-pinned to its
  chosen disposition.
- **The out-of-class adjacency** — the fillet asked on an edge
  whose faces fall outside D3's two classes: refused naming the
  pair and the remedy (D2's boundary, whichever option the user
  picks).

**The adversarial corpus** (a suite member, like the guide case):
every family above → a golden or a named error, by name —
`fillet_radius_exceeds_face`, `fillet_exits_annulus`,
`fillet_eats_wall`, `fillet_corner_is_vertex_blend`,
`fillet_tangential_collapse` (tie-break golden),
`fillet_sliver` (tie-break golden), `fillet_out_of_class_pair`.
The corpus is the phase's robustness data for the *next*
checkpoint, exactly as G1's dispositions fed this one.

### D5. The unit ladder — the starting proposal (the review can move it)

1. **The table** — the SSI entries as a kernel surface and through
   the bindings (queryable before any boolean consumes them: the
   unit-2 face-query precedent), each entry a derivation with its
   goldens (curves, p-curves in both domains, provenance exact),
   the re-frame rule and its determinism pins, the degenerate
   limits. *Visible value: the intersections answerable from
   Python on authored bodies.*
2. **The extended booleans** — sphere×sphere, sphere±prism,
   prism×prism parallel (the two-holes body), sphere±box; the
   class gates extended, the frame budget enforced (the second
   incompatible cut refused actionably), the honesty chain
   (worst-of provenance), the **trimmed-sphere mass family lands
   here** (its bodies must answer numbers), the historical goldens
   and v0.3/v0.4 projects untouched. *[Under D2 option (ii):
   **unit 2.5 — the subdivision engine** slots before this unit —
   the interval-bounded march, the sampled p-curve citizen, the
   tolerance discipline end-to-end, its own goldens and its own
   refusal surface.]* *Visible value: the classical sphere union,
   exact, with its closed-form volume.*
3. **The fillet** (the emotional summit) — the rolling-ball
   construction on the two classes, the counts and volume goldens
   (the D3 tables), the **torus mass family**, the **torus SDF
   primitive** with its derived bounds, the HybridBody landing,
   and the **v0.5 MINOR format bump with its 0007-series
   companion and migrator** — the fillet recipe is a new recipe
   kind, which is the demonstrated need ADR-0007 requires before
   the format moves; no other unit touches it. *Visible value:
   the first feature impossible yesterday.*
4. **The degenerates + the adversarial corpus** — D4's families,
   each pinned to a golden or a named error; the corpus in the
   CI. *Visible value: the refusals are answers.*
5. **The UI** — Feature → Fillet: select an edge, set the radius
   — a real authored parameter (`parameter_changed` flows, the
   edit regenerates bit-identically, ONE undo entry — the exact
   opposite of the read-only mass rows, which stay read-only);
   the report says everything (the flank's family in the face
   query, the counts with the H term, the provenance exact).
   *Visible value: the feature in the hands.*
6. **The guide case end-to-end + the click-test** — the G1 chain
   plus the rim fillet; the printed table extended by the D3 rows
   (14/20/8/2, χ=0, genus 1, abc − πr²h − V_ρ, all exact); the
   fillet-radius edit bit-identical through ONE undo; the v0.5
   round-trip. **The phase's exit: the user's click-test** — the
   suite covers the contracts, the hands cover the experience
   (the ADR-0030/0032 lesson, enforced).

Each unit: the invariant (ADR-0031 D3's, unchanged — closure,
χ = V−E+F−H hard-fail, provenance on every entity), the
discriminator tests (the classical formulas ARE the tests), the
entry criterion (the previous unit green), and the visible value
named above — the countermeasure the acceptance motivation cites,
enforced per unit.

### D6. The no-promises list

- **Vertex blends are unscheduled.** The phase is valid without
  them; every collision is an honest refusal with the remedy.
  No date, no sprint, no percentage — the tail stays the tail
  (the checkpoint's price list says years; this ADR adds nothing
  to it).
- **G3 is untouched** — no spline vocabulary, no NURBS, no loft:
  the out-of-table pairs follow the D2 decision and never pass it.
- **No new dependencies** — the amended ADR-0003 whitelist binds;
  nothing G2 needs is missing from the tree (the subdivision
  engine, if chosen, is house code on the existing `Interval`
  type).
- **The format moves only where a unit demonstrates the
  necessity** (ADR-0007's rule): the fillet unit ships the v0.5
  MINOR companion — the one new recipe kind — and no other unit
  touches the format, exactly as G1 landed six units without a
  bump.
- **No time promise on the tail.** The only number on the record
  is the checkpoint's, and it stands un-rebaselined.

## Consequences

- If accepted as drafted: G2 starts under the D5 ladder once the
  D2 decision is taken (the ladder's shape depends on it — six
  units under (i), seven under (ii)); each unit lands standalone
  with the full suite green; the README's Phase G row advances per
  unit; the phase closes on its own click-test, and its checkpoint
  feeds the G3 decision the same way G1's fed this draft.
- If amended: the D-sections are independent by design — the
  review can move the table boundary (D2's options are written to
  be chosen between, not merged), the ladder, or the fillet class
  without rewriting the document. The derivations to check
  hardest, named for the review: **D1.1's both-domain criterion**
  (every entry's admission hangs on it) and **D3's arithmetic**
  (the counts and V_ρ — hand-checkable, and the goldens will quote
  them verbatim).
- The faceted bridge, the SDF display path, and every existing
  golden are untouched by acceptance — G2 adds faces and paths, it
  replaces none.
- The findings this draft contributes back to the record: the
  both-domain admission criterion (D5.3's lesson made a rule); the
  `Ellipse2` correction (ADR-0032 D5.3's unlanded parenthetical);
  the re-frame rule and its one-axis-family budget; the rim-fillet
  arithmetic (χ-flat by construction, the volume with the torus
  term); and the correction of ADR-0031 D5's "three faces retire"
  sketch (two shrink, one is born — the derivation, the only
  authority the house recognizes).

## References

House records:

- ADR-0031 (the mother: D1's price list the G1 checkpoint
  measured — the estimates stand; D4.1 the SSI strategy this
  table concretes, D4.2 the boolean guarantees, D4.3 the fillet
  strategy and the vertex-blend tail, D5 the ladder, D6 the
  migration and the v0.5 sketch the fillet unit honors).
- ADR-0032 (the G1 spine: D2's vocabulary and conventions, D5's
  SSI entry and its sinusoid lesson — the criterion's first
  instance, D7's subtract class and degenerate dispositions, D8's
  provenance contract, D9's test discipline, D10's unit method).
- ADR-0021/0022 (the topology spine and invariants), ADR-0023
  (the seam-wire pattern the torus flank reuses), ADR-0024/0025
  (the boolean pipeline and the canonical-key assembly), ADR-0026
  (the bridge's declared-error culture — option (ii)'s fence),
  ADR-0027 (recipes as the source of truth — why frames are never
  serialized), ADR-0030 (the 2D kernel whose types are the
  vocabulary), ADR-0002 (the dual representation the HybridBody
  serves), ADR-0003 as amended (the whitelist), ADR-0004 (the
  typed tolerances every band and tie-break declares), ADR-0005
  (the naming genealogy the fillet recipe joins), ADR-0007 +
  0007a (the format rule: the bump ships with the unit that needs
  it), ADR-0014 (determinism — the re-frame and the subdivision
  both answer to it), ADR-0028/0029 (the SDF bounds discipline
  the torus primitive follows), and the platform vision
  (docs/architecture/platform-vision.md — H1's interrogable faces
  and declared tolerances are what the table and the fence buy).

Literature and classical records:

- Hoffmann, C. M. (1989). *Geometric and Solid Modeling*. Morgan
  Kaufmann. (The blending chapters: rolling-ball surfaces, the
  vertex-blend tail — why D4 refuses it at the door.)
- Patrikalakis, N. M., & Maekawa, T. (2002). *Shape Interrogation
  for Computer Aided Design and Manufacturing*. Springer. (The SSI
  taxonomy and the intersection classification the table's
  boundary formalizes.)
- Levin, J. Z. (1976). A parametric algorithm for drawing pictures
  of solid objects composed of quadric surfaces. *Communications
  of the ACM* 19(10); and Levin (1979), *Computer Graphics and
  Image Processing* 11(3). (The quadric-pair pencil method — why
  the quartics are quartics.)
- Boender, E. (1998). A survey of intersection algorithms for
  curved surfaces. *Computers & Graphics* 22(1). (The algorithm
  families option (ii) chooses among — subdivision with interval
  bounding, its failure modes named.)
- Mäntylä, M. (1988). *An Introduction to Solid Modeling*.
  Computer Science Press. (Euler operators — the counts
  arithmetic of D3 stands on the generalized relation.)
- Mirtich, B. (1996). Fast and Accurate Computation of Polyhedral
  Mass Properties. *Journal of Graphics Tools* 2(2). (The
  polyhedral baseline the new families' closed forms are checked
  against numerically.)
- The classical curves, cited as classical and derived where
  used: **Viviani's curve** (the sphere–tangent-cylinder quartic);
  the **spiric sections of Perseus** (the torus plane sections —
  the Hippopede, the Cassini ovals, the Bernoulli lemniscate as
  named members); the **Steinmetz** bicylinder (the degenerate
  disposition that proves the domain criterion stricter than the
  space-curve one); the spherical segment and torus volume
  formulas (the mass families' tests).
