# ADR-0032: G1 — topology and exact primitives

## Status

**Accepted (2026-10-25, closed in review) — D9's entry criterion
met: invariants and discriminating tests agreed before implementation
began.** Daughter of ADR-0031 (accepted: road (a), all in house,
forever — the G2 gate an internal checkpoint, not a fork). This ADR
pins G1's scope, data-model deltas, the Layer-1 invariant, the hard
derivations (mass properties, plane∩cylinder), the discriminating
test class, and the delivery units. Delivery follows the six units of
D10, each landed standalone with the full suite green; **units 1–5
have landed**:

* **Unit 1 (the data model, D2 + D3)** — the p-curve vocabulary and
  field, the Layer-2 tolerance/provenance fields with the writers'
  honesty (bridge → approximated(grid tol), factories → exact/0,
  booleans → worst-of-operands propagation), and the extended
  `euler_characteristic` χ = V−E+F−H with its backward-compatibility
  proof on every body path and every golden.

* **Unit 2 (the primitives with true trimming, D1.1)** — the
  factories write their p-curves: every co-edge of every factory
  body (box, sphere, cylinder, tetrahedron) carries its typed 2D
  curve in the owning face's domain, the faces now regions of their
  domains (D2.3) — the box's six projected rectangles, the sphere's
  slit rectangle with the periodic seam pair, the cylinder wall's
  rectangle walk in (θ, axis) and the caps' π-sweep arc pairs.
  Counts did not move (box 8/12/6, sphere 2/1/1, cylinder 4/5/3,
  tetra 4/6/4; all H = 0, χ = 2) and every historical golden stays
  bit-identical. **One convention correction, landed as the order
  demanded (fix and document, not work around): the analytic
  surfaces' (u,v) frames are now ALL right-handed around the outward
  normal — the cylinder was the lone left-handed exception, and the
  first real use of the pinned conventions exposed it (the caps and
  the wall, both authored material-on-left, walked the shared rim
  edges in the same direction, breaking the orientation-XOR);
  ConeSurface and TorusSurface remain left-handed, registered for
  the units that will first trim them.** The face query of D8's
  "a query, not an investigation" is exposed in C++
  (`brep/query.hpp`) and through the bindings
  (`CAD_0.brep.face_query`): family, loop counts, per-co-edge
  p-curve type and parameters, provenance — plain values, honest on
  both the exact and the faceted world.

* **Unit 3 (the exact extrude, D6)** — the geometry born from the
  sketch becomes exact: `brep::make_prism` maps the typed profile
  chain to the prism B-Rep per D6's table (segments → planar
  rectangles, arcs → cylindrical patches Δθ×h on the new
  `CylinderZSurface`, the solid circle → the ADR-0023 seam-wired
  pattern verbatim on the Z axis, 4V/5E/3F), and the caps' outer
  trimming loop IS the validated chain itself, entity for entity
  (segment→Line2, arc→Arc2, and — on the hole rims — circle→
  Circle2, the vocabulary's first Circle2 writer: a closed trim
  curve, honestly direction-free). The pinned formulas hold for
  every n (V=2n, E=3n, F=n+2, χ=2; the lens at n=2, the box
  case n=4) and the multi-chain extension is pinned with them
  (each hole adds its wall and one inner cap loop: χ = 2−2·holes,
  the ring reads χ=0 genus 1). **The HybridBody landed (ADR-0031
  D6 honored): the extrude node owns BOTH materializations — the
  SDF twin and the exact B-Rep — with bridge tolerance 0 (the same
  solid, dual native materializations, not a conversion); the SDF
  keeps feeding display unchanged, the subtract's bridge extract
  stays faceted/approximated (the honesty chain of D8), and the
  .CAD_0 format does not move (D11 — the hybrid is derived state,
  re-materialized deterministically on load).** Two class
  boundaries, derived and documented rather than worked around:
  (1) a hole-side ARC (hole=true — a notch or a bite) refuses the
  exact B-Rep loudly, because the typed 2D vocabulary stores CCW
  sweeps only and a notch arc's cap trimming runs CW in BOTH cap
  domains — the same honest exclusion shape as D5.3's oblique SSI;
  the SDF materialization stands, the refusal recorded on the node;
  (2) multi-entity HOLE chains are inexpressible in the 2D kernel
  itself (the max-composition intersects regions; an intersection
  of half-planes is convex and cannot be a hole's complement) —
  holes are circles, one disc-complement entity each, exactly the
  canvas's authoring class. The convention checker's region-side
  model extended to the AUTHORED bits (interior_left / !hole) —
  the sketch kernel's own region semantics, "the same mathematics"
  of D2.1 read verbatim. The frame rule forced CylinderZSurface's
  u-sign (the hole side's outward normal is −radial: u runs the
  reversed azimuth there) — the second real use of the pinned
  conventions, landed as the first one was: fixed and documented.

* **Unit 4 (the exact subtract, D5 + D7)** — the exact chain crosses
  its first intersection between curved surfaces, the phase's most
  supervised step. `brep::exact_subtract` materializes A \ (B +
  offset) on the pinned G1 class (A convex planar-faced with the
  ⊥/∥ orientation gate — an oblique target face refuses naming the
  angle; B a Z-axis prism of the D6 class — the Y-axis primitive
  cylinder refuses naming its axis; both operands exact — the
  booleans consume exactness, the unit-1 finding made a gate). The
  SSI entry of D5 (`ssi_plane_cylinder`) carries the three
  orientations' typed mathematics: ⊥ the circle with the Line2 v=h
  on the cylinder's domain (the hole case), ∥ the line pair / the
  tangent double line / the empty case, oblique REFUSED with the
  angle (the sinusoid p-curve is G3's table, never smuggled in).
  The through-hole's new topology lands as pinned: the pierced
  faces GAIN inner loops (the intersection circle split at the
  seam pair, ADR-0023's pattern), the surviving wall is the
  cylindrical patch BETWEEN the circles (the seam's two co-edges in
  the one loop, XOR satisfied), the tool's caps die — a flush cap
  ABSORBS (the coplanar dedup rule, the counts unchanged) — and
  the guide case's table reads 12V/17E/7F/H=2 → χ=0 genus 1,
  agreeing with the faceted report (same topology, better
  geometry). The general formula holds for every tool profile the
  extrude authors (V=V_A+2n, E=E_A+3n, F=F_A+n, H=2, χ=0 for
  every n; the circle is the seam-wired special case, a prism
  target included). The degenerate classes carry their pinned
  dispositions: coplanar ABSORBED (golden), the tangential wall
  REFUSED by the deterministic canonical-key tie-break (golden on
  the chosen disposition), the crossing circle and the blind hole
  REFUSED with actionable errors naming the remedy (goldens), the
  swallow REFUSED, the miss the EXACT COPY of A. **One convention
  discovered, landed as the order demanded (fix and document, not
  work around): the surviving wall keeps the prism's PHYSICAL
  window walk, re-framed by the flipped orientation — it reads
  REGION-ON-RIGHT (interior_left=false) in its own domain, because
  the pierced faces' inner loops must read CCW for the Arc2
  vocabulary (CCW sweeps only) and the radial-edge XOR ties the
  two walks together: one requirement, seen from both sides (the
  unit-3 material-right precedent, applied to a lateral face).
  The inner loops' region bits are FLIPPED from the caps' (the
  arcs carry hole=true, the segments interior_left=false — the
  material is outside the profile now; the walk direction follows
  each pierced face's own frame parity, exactly the caps' walks).**
  The subtract node owns BOTH materializations (the HybridBody,
  bridge tolerance 0, the SDF still feeding display); the operands'
  B-Reps resolve through their own hybrids or the bridge's
  primitive path — one resolution, no second opinion. The v0.4
  golden, in class, materializes the exact subtract ON LOAD
  (12/17/7/2; the new exact pins live in the unit's suite; the
  SDF-side report pins are unchanged — they pin the display/bridge
  path, which did not move); the v0.3 golden STAYS faceted
  honestly — its tool is the Y-axis primitive cylinder, not a D6
  prism, and the refusal is recorded on the node (the class is the
  class; "col motivo scritto" cuts both ways). The SDF/display
  goldens are bit-identical (verified, git-clean); the radius and
  offset edit cycles regenerate BOTH materializations
  bit-identically (ADR-0014).

* **Unit 5 (the exact mass properties, D4)** — the B-Rep's first
  quantitative interrogation: volume, centroid and the inertia
  tensor by the DIVERGENCE THEOREM over the exact faces
  (`brep::mass_properties`, core/brep/mass_properties.{hpp,cpp}),
  never estimated from a mesh. The method lands as derived: V =
  (1/3)∮x·n, M_i = (1/2)∮x_i²n_i, S_ij = (1/5)∮x_i x_j (x·n) —
  the 1/5 from the primitive-function ansatz's five contraction
  terms — the centroid M/V, the tensor δ·tr(S) − S and the
  classical parallel axis. The per-family closed forms: the
  PLANAR faces by GREEN'S THEOREM in the face's own (u,v) domain
  (the six kernels A/Mu/Mv/Quu/Quv/Qvv from the loops — the
  polygon part by the shoelace pattern, each arc by its
  trigonometric line integrals through DERIVED reduction
  recurrences for ∫cos^i sin^j, a lone Circle2 hole rim by the
  disc's constants), the region semantics read GEOMETRICALLY: the
  first loop is the outer boundary, subsequent loops are holes
  (ADR-0021), each loop's winding sign σ = sign(∮u dv as
  traversed) — outer-minus-inner is positional arithmetic,
  whatever direction each walk runs (the caps' mirror/rotation
  frames, the pierced faces' flipped walks: one rule, the
  region-side bits never consulted). The CYLINDRICAL patches by
  the (θ,z) rectangle integrals on a UNIFIED cylinder frame (the
  extrude's Z-axis and the factory's Y-axis placements, the
  normal sign s and the one substitution factor folded in); the
  window read from the wall's Line2 loops (the bounding box —
  the seam-wired walks split edges, never move the window),
  verified on the boundary. The FULL SPHERE by its closed form
  (the only sphere the exact world produces; trimmed spheres are
  G2's, honestly declared). **The honesty contract: `all_exact`
  is the flag — a body with ANY approximated face answers the
  POLYHEDRAL value (every face reduced to its vertex polygon in
  its own plane, Mirtich 1996's reduction subsumed by the same
  Green machinery), never a silent mix; an exact face without
  p-curves refuses loudly (the machinery honours the record and
  does not launder it). The error contract: out-of-class families
  (cone, torus), the empty body, the non-closed shell and the
  degenerate volume each refuse actionably (the family and the
  remedy named).** The precision doctrine, discovered and pinned:
  the arithmetic is closed-form and double, the INPUTS are the
  stored trimming parameters — the p-curves' floats (the float-π
  sweeps of the seam-wired walks, the projected frames) — so the
  honest bound is a few float ulps (~1e-6 relative), still five
  orders tighter than any faceted extract; the suite pins that
  bound (the exactly-representable bodies pass tighter). The
  classical formulas ARE the tests (D9.3): box abc and
  abc(b²+c²)/12, sphere (4/3)πR³ and (2/5)VR², cylinder πr²h,
  πr⁴h/2 and V(3r²+h²)/12, the circle prism's per-face
  decomposition (0 + (1/3)hπr² + (2/3)πr²h), the sector's pie
  formulas, the ring's Circle2 subtraction, the tetrahedron's
  vertex determinant, the TRANSLATED sphere (the parallel-axis
  check), and THE GUIDE CASE's full table (abc − πr²h, the
  centroid on the axis, I_zz = abc(a²+b²)/12 − πr⁴h/2 — the
  off-centre hole deriving the composite parallel axis by hand).
  The v0.4 golden answers the guide numbers on load,
  bit-identically through a save/reload cycle. Exposed in the
  bindings (`CAD_0.brep.mass_properties`) — the numbers readable
  from Python end-to-end through the scene, the unit-6 report's
  contract usable from day one.

**No promise on G2 appears anywhere in this document.** G2 has
no schedule, no scope here, and no implied commitment: its entry
is ADR-0031's internal checkpoint (estimate vs. progress), and
its ADR will be written when G1's data is on the table.

## Context

ADR-0031 D5 pinned the rung:

> **G1 — topology and exact primitives.** Box / sphere / cylinder
> as exact B-Reps *with real trimming* (p-curves on loops, D3);
> exact mass properties computed **from the B-Rep** — volume,
> centroid, inertia — via the divergence theorem over trimmed
> analytic faces. The one SSI entry needed: plane∩cylinder.
> Guide case: the parametric hole redone in exact B-Rep.

**What exists today (verified in the tree, not assumed):**

- The five analytic surface classes — `PlaneSurface`,
  `CylinderSurface`, `SphereSurface`, `ConeSurface`,
  `TorusSurface` (`core/brep/include/CAD_0/brep/analytic.hpp`)
  — as *untrimmed* carrier surfaces.
- The curve set `LineCurve` / `CircleCurve` / `EllipseCurve`
  (`curve.hpp`) — 3D, standalone.
- The radial-edge spine — `Vertex/Edge/CoEdge/Loop/Face/Shell/
  Solid`, `link_radial_edges`, the loop-closure and
  orientation invariants, `euler_characteristic`,
  `genus_from_chi` (ADR-0021/0022) — with `Loop` already
  distinguishing an outer boundary from holes.
- Seam-wired primitives that are closed manifolds today: the
  sphere (2V/1E/1F) and the cylinder (4V/5E/3F) of ADR-0023.
- Convex-planar booleans (ADR-0024/0025) and the assembly
  pattern behind them: canonical vertex identity, edges keyed by
  vertex pairs, order-independent dedup, hard postconditions.
- The 2D sketch kernel (ADR-0030): typed entities (segment,
  circle, arc) with construction-time validation, closed-chain
  semantics, the region/inside discipline.
- The recipe spine (ADR-0027 D4): recipes are the source of
  truth, bodies are cache, `rebuild_operations()` re-materializes
  on every edit.
- The bridge (ADR-0026): primitive fitting (exact) + MC33
  faceted extraction, with the χ/genus report ADR-0027 D6
  amended — the report the Debug menu renders.

**What does not exist today (verified by search across
`core/`, `bindings/`, `app/`):**

- p-curves of any kind — no co-edge carries a curve in its
  face's parameter domain; a `Loop` today is a cycle of 3D edge
  references only.
- Trimming as data — the analytic faces of the factories are
  bounded by construction (the factory writes the loops), not by
  trim domains that operations can compute with.
- **Any mass-properties API** — no volume, no centroid, no
  inertia, nowhere in kernel, bindings, or app. The number
  users quote first is absent.
- Surface-surface intersection of any kind — the booleans clip
  polygons against planes (Greiner-Hormann on planar faces);
  no curved pair has ever been intersected.
- An exact subtract on curved faces — the sketcher's hole
  extracts **faceted** through the bridge: the wall arrives as a
  prism of planar strips, not the cylinder it mathematically is.

**The guide case, today and tomorrow.** `examples/sketch_hole.py`
walks the chain — box, sketch, circle, extrude, subtract by
FeatureId, radius edit propagating bit-identically, v0.4
round-trip, genus-1 report. The faceted extract of that body
certifies χ = 0, genus 1 — correct topology, approximate
geometry, no mass numbers. G1's guide case is the same chain
rebuilt on exactness: the extrude produces a prism with **true
faces**, the subtract produces a hole whose wall **is** the
cylinder, and the mass numbers are read from the solid — with
the analytic comparison known in advance, so the user can verify
by hand.

## Decision

### D1. Scope — in, and explicitly out

**In:**

1. **True trimming on the primitives**: box, sphere, cylinder as
   exact B-Reps whose loops carry p-curves in their surfaces'
   parameter domains, and whose faces are regions of those
   domains (D2).
2. **Exact mass properties from the B-Rep**: volume, centroid,
   moment-of-inertia tensor — the divergence theorem over
   trimmed analytic faces, closed forms per family, exposed in
   C++ and through the bindings (D4). The API that today exists
   nowhere.
3. **The SSI entry plane ∩ cylinder**, all in-scope orientations,
   closed forms (D5).
4. **The exact extrude**: sketch profile → prism with true faces
   (D6).
5. **The exact subtract on the G1 class** — convex planar A
   minus the exact prism B — producing pierced faces with inner
   loops and a cylindrical wall face (D7).
6. **The guide case end-to-end** and the report: mass numbers +
   provenance + the existing χ/genus, visible in the UI, with
   the analytic comparison (D8, D10 unit 6).

**Out (explicit, so the review can check the boundary):**

- Fillets of any radius — G2's rung, unscheduled.
- NURBS, loft, sweep, blends — G3's rung (the platform vision's
  H2 declares it; it is not redesigned here).
- Any SSI pair beyond plane∩plane (exists) and plane∩cylinder
  (this ADR). Sphere/cone/torus pairs wait for the G2 table.
- The oblique plane∩cylinder case in the *exact* path — the
  honest exclusion with the reason, derived in D5.3: the p-curve
  on the cylinder is not expressible in the typed 2D vocabulary
  G1 has; it needs the spline p-curves of the G3 layer.
- Non-convex operands, sphere/cone subtracts, blind holes
  piercing a face and terminating inside the body (the G1
  subtract requires the prism to exit through faces, D7.4).
- The constraint solver (still deferred, ADR-0030's entry
  criterion unchanged); the faceted bridge (unchanged — it
  remains the honest path for everything outside the exact
  class); the `.CAD_0` format (D11: no version bump in G1
  unless a unit discovers a need).

### D2. Data-model deltas — ADR-0031 D3 made concrete

**One structural extension, four fields, zero rewrites of the
spine.** The ADR-0021 entity set, ID scheme, ownership rules,
and `link_radial_edges` discipline are reused verbatim (the
daughter honors the mother's ratification). The deltas:

1. **The 2D curve vocabulary — reused, not copied.** The p-curve
   types are the sketch kernel's typed 2D entities (segment,
   circle, arc — ADR-0030 D3) exposed to the B-Rep layer as the
   parameter-domain curve set: `Line2`, `Circle2`, `Arc2`. One
   opinion, one implementation — the D5.5 lesson (the UI calls
   the kernel's validation, never a second copy) applied at the
   kernel layer: the B-Rep p-curves and the sketcher's entities
   are the same code, so a trimming loop and the sketch chain
   that produced it cannot disagree.
2. **`CoEdge` carries a p-curve**: a typed 2D curve in the
   owning face's surface parameter domain, alongside the
   existing 3D edge reference. The pair (3D space curve, 2D
   p-curve) is the defining structure of the exact B-Rep — a
   face's loop is a *trimming loop*, a cycle of p-curves on the
   surface, not a polygon of 3D points. Faceted faces (bridge
   output, plane-per-triangle) carry no p-curves — the
   provenance field (delta 4) distinguishes the worlds; the
   invariant is per-world, never fudged.
3. **`Face` carries its parameter-domain region** as the loops
   themselves: the outer loop bounds, the inner loops (already
   in the structure since ADR-0021: "subsequent loops are
   holes") subtract. The face *is* the region between them —
   the mass-property integrals (D4) and the SSI trimming (D5)
   consume exactly this.
4. **Layer-2 tolerance + provenance fields** on vertex, edge,
   face: the linear tolerance bound (factories write 0; the
   intersecting operations write their declared tolerance) and
   the provenance enum `exact | approximated(tol)` — the
   contract of D8: "is this edge exact?" always answerable.

Backward compatibility is structural: every existing body is
representable with empty p-curves and `approximated` faceted
provenance — which is what it is today — and no existing test
goldens move (the counts are untouched by fields).

### D3. The Layer-1 invariant — pinned hard-fail, with the derivation the guide case forces

The order pins: structural closure; integer χ = V−E+F = 2(S−G)
on every closed body, hard-fail. The derivation below confirms
the pin **and corrects it where G1's own guide case breaks it** —
the honest form of honoring a pinned invariant is deriving it,
and the derivation surfaces a term the disk-faced world never
exercised.

**Structural closure (unchanged from ADR-0022 §7 as amended by
ADR-0027 D6, invariants 0–3):** every edge exactly two co-edges
with orientation XOR; every loop closed (end vertex of c_i =
start vertex of c_{i+1 mod n}); every face exactly one outer
loop; the shell orientable. Hard-fail, every body, every test.

**The Euler characteristic of a shell with holed faces —
derived from first principles.** For a closed surface decomposed
into F faces where face f has b(f) boundary loops, the
characteristic is the face-wise sum glued along edges and
vertices:

```
χ = V − E + Σ_f (2 − b(f))
```

A disk face (one loop) contributes 2−1 = 1 — the classical
V−E+F. An annulus face (outer loop + one hole) contributes
2−2 = 0: **a pierced face is not a disk, and counting it as one
inflates χ by exactly its hole count.** With L the total loop
count and H = L − F the inner-loop count:

```
χ = V − E + F − H = 2(S − G)        (Euler-Poincaré, loop term)
```

**Why G1 forces this correction:** the guide case's subtract
produces exactly the holed faces the old formula miscounts. The
pinned counts, derived (D7.5 carries the full table): the box
minus a through-hole has V = 12, E = 17, F = 7, H = 2 →
χ = 12−17+7−2 = 0 → genus 1 ✓ — agreeing with the faceted
report's χ = 0, genus 1, as it must (same topology, better
geometry). Under the pin's literal form (V−E+F = 2(S−G)) the
same body reads 12−17+7 = 2 → "genus 0" — **falsely**, and the
invariant would hard-fail a correct body or pass a wrong one.
The correction is classical (Mäntylä 1988 carries the
generalized relation for B-Reps with holed faces); the
derivation above is the house's own, and it is the reason this
ADR exists as a review rather than a rubber stamp.

**The contract, as G1 pins it:**

- `euler_characteristic()` is **extended** to count inner loops:
  χ = V − E + F − H. The extension is proven backward
  compatible: every body the tree produces today is disk-faced
  (H = 0 — the bridge's triangles, the factories' faces), so
  every existing pinned value is unchanged; the docstring
  carries the derivation and the annulus counterexample.
- `genus_from_chi()` derivation rules are unchanged (even, ≤ 2,
  single closed-orientable shell) and now receive the honest χ.
- Hard-fail in every factory, every boolean, every test, and
  surfaced by the Debug report alongside the structural checks —
  the report ADR-0031 D3 promoted to the kernel's invariant.
- The invariant is integer and exact where everything geometric
  is approximate: topology is the truth, geometry carries its
  declared error (Layer 2), and both are reported.

### D4. Exact mass properties — the divergence theorem over trimmed analytic faces

**The method, from first principles.** For a closed body B with
outward unit normal n on ∂B, the divergence theorem converts
every volume moment into a surface integral:

```
V   = (1/3) ∮_∂B  x·n  dA                      (volume)
M_i = (1/2) ∮_∂B  x_i² n_i dA                  (first moments: ∇·(½ x_i² e_i) = x_i)
```

The pattern generalizes by the primitive-function ansatz: for
the second moments the integrand x_i x_j admits
∇·((1/5) x_i x_j x) = x_i x_j (the constant 1/5 follows from
the monomial's three contraction terms — two from the indices,
three from the position vector; each family's constant is
derived, not looked up, and the closed-form tests below are the
check). The inertia tensor about the origin follows from the
second moments, and to the centroid via the parallel-axis
theorem (classical). Density 1; API consumers scale.

**The closed forms per G1 face family — with the check
derivations that validate the machinery:**

- **Plane face** (region bounded by line and arc loops): the
  volume contribution is (1/3) n·(A·c) — n constant, so the
  integral reduces to the planar region's area A and area
  centroid c. Both are classical composite quantities of the
  loops: the polygon part by Green's theorem (the shoelace
  pattern), each arc contributing the circular-segment and
  sector corrections (the classical segment formulas). The
  decomposition follows the loop structure: outer loop minus
  inner loops — the same arithmetic the face's region semantics
  already define.
- **Cylindrical patch** (θ-interval × z-interval, radius r): on
  the lateral surface x·n = r — *constant* — so the volume
  contribution is (1/3)·r·(patch area) = (1/3) r² Δθ h. The
  moments reduce to powers of the parameterization integrated
  over the rectangle in (θ, z): elementary.
- **Sphere** (full, seam-wired): x·n = R everywhere on the
  sphere → V = (1/3) R·4πR² = (4/3)πR³. Trimmed spherical
  regions arise only from sphere booleans — outside G1's
  subtract class (D1) — so the full-body closed form is the
  G1 scope, honestly stated.

**The end-to-end checks (they pass only if every family is
right):** the circle-prism: caps at z = 0 (n = −ẑ, x·n = 0 →
contribution 0) and z = h (n = +ẑ, x·n = h → (1/3) h πr²), wall
((1/3) r·2πrh = (2/3) πr²h) — total πr²h ✓. The box: six plane
faces, pairwise cancellation of opposite faces' (1/3)n·(A·c)
terms leaves abc ✓. The sphere: (4/3)πR³ ✓. The guide case
(derived in D7.5): abc − πr²h ✓ — the number the user checks by
hand.

**The API and its error contract:**

```cpp
struct MassProperties {
    double volume;
    math::Vec3f centroid;
    math::Mat3 inertia_about_centroid;   // density 1
    // provenance of every contributing face was 'exact'
    bool all_exact;
};
MassProperties mass_properties(const BRepBody&, Tolerance);
```

- Exact for the G1 class: every face planar-with-(line/arc)-loops,
  cylindrical patch, or the full sphere — and every face's
  provenance `exact`. `all_exact` is the honesty flag: if any
  face is `approximated` (a faceted body from the bridge), the
  result is the *polyhedral* divergence-theorem value (Mirtich
  1996's reduction, already implied by the same machinery) and
  the flag says so — never silently mixed.
- Outside the supported class: an actionable error naming the
  face type and the remedy (the ADR-0026 P2 pattern — visible,
  never a wrong number).
- Determinism per ADR-0014: the closed forms use +, −, ×, ÷ and
  sqrt only in the segment/sector constants; same-platform
  bit-identical re-runs, serialization quantized as everywhere.

### D5. The SSI entry — plane ∩ cylinder

**The classical fact (one derivation, two degenerate limits).**
A plane Π (normal m) cutting a right circular cylinder (radius
r, axis â) at angle θ between m and â produces:

- θ = π/2 (plane ⊥ axis): a **circle** of radius r, centered
  where the axis meets the plane. The guide case's cap circles.
- θ → 0 (plane ∥ axis): a **pair of parallel lines** at distance
  r from the axis's projection onto the plane (tangent: a double
  line; farther: empty). The side-breakout case.
- 0 < θ < π/2 (oblique): an **ellipse** with semi-minor r and
  semi-major r/sin θ — the classical conic section; the circle
  and the line pair are its degenerate limits.

All three are closed forms; the 3D curves are typed
(`CircleCurve` exists; the line pair is `LineCurve`).

**The p-curve subtlety — where the honest boundary lands.** The
same intersection curve lives in *two* parameter domains:

- On the **plane's** domain: all three cases are typed — the
  circle is a `Circle2`, the line pair `Line2`s, the ellipse an
  `Ellipse2` (the vocabulary of D2 covers them).
- On the **cylinder's** (θ, z) domain: the ⊥ case is the
  horizontal line z = h (a `Line2` ✓); the ∥ case is the pair
  of vertical lines θ = ±θ₀ (`Line2` ✓); but the **oblique
  case is z(θ) = z₀ + r·cot(θ̃)·cos(θ − θ₀) — a sinusoid**,
  which is not expressible in the typed 2D vocabulary G1 pins
  (D2). Industrial kernels represent it as a spline p-curve
  with a tolerance; that machinery belongs to the G3 NURBS
  layer.

**Decision:** the G1 entry implements the ⊥ and ∥ cases —
fully typed, fully exact, both p-curves `Line2`s on the
cylinder. The oblique case is **declared out** (D1): the
subtract's class check (D7.3) rejects it with an actionable
error naming the angle and the remedy (reorient, or accept the
faceted path), and the entry point for the general case is the
G2 table with the G3 p-curve vocabulary. This is a discovered
boundary, not a convenience cut: the alternative — an
untyped sinusoid smuggled into the typed world — is exactly the
kind of silent corruption the provenance contract exists to
prevent.

### D6. The exact extrude — sketch profile to prism with true faces

**The mapping (the sketcher's typed entities are why this is a
table, not a research problem):**

| Profile entity | Lateral face | Trimming loops in (u, v) |
|---|---|---|
| Segment (length ℓ) | Plane face | Rectangle: 4 `Line2` — the segment at v=0, v=h, the two verticals |
| Arc (radius r, span Δθ) | Cylindrical patch | Rectangle in (θ, z): 4 `Line2` — Δθ × h |
| Circle (closed) | Full cylinder + seam | The ADR-0023 seam-wire pattern, reused verbatim |

**The caps:** the profile's plane regions, bounded by the
*profile chain itself* as the outer trimming loop — the sketch
chain (typed, validated, closed — ADR-0030 D5.3) becomes the
cap's loop, entity for entity, segment→`Line2`, arc→`Arc2`,
circle→`Circle2`. The 2D kernel's region semantics (the
max-composition inside/outside discipline) and the cap's
parameter-domain region are the same mathematics; the reuse is
the point of the typed design.

**The pinned counts (the invariant's arithmetic, derived):** a
closed chain of n entities extrudes to V = 2n, E = 3n, F = n+2
→ χ = 2n − 3n + (n+2) = 2 for every n ✓ — the box is the n = 4
case (8/12/6). The circle profile is the seam-wired special
case: the ADR-0023 cylinder counts, 4V/5E/3F, χ = 2 ✓. Every
prism the extrude can produce is pinned by one of the two
formulas.

**Where the body lands (ADR-0031 D6 honored):** the extrude
operation node materializes the exact B-Rep *next to* the
existing SDF — the node's body becomes ADR-0002's `HybridBody`
owning both, bridge tolerance 0 (the same solid, dual native
materializations, not a conversion). The SDF materialization
keeps feeding the display path unchanged; the B-Rep
materialization is the properties/export truth. Recipes are
untouched: the format does not move (D11).

### D7. The exact subtract on the G1 class

**The class, pinned before the algorithm:** A convex with
planar faces (a box, a planar prism); B the exact prism of D6
(convex profile or circle); the extrusion axis crossing A's
faces only at ⊥ (circles) or ∥ (line pairs) per D5 — the class
check is part of the operation's input contract.

**The pipeline (ADR-0024/0025 generalized, SSI-fed):**

1. **Intersect:** A's planar faces × B's lateral faces via the
   D5 entry (circles on the ⊥ faces, line pairs on the ∥
   faces); A's planar faces × B's caps via the existing
   plane∩plane path (Greiner-Hormann, unchanged).
2. **Classify:** fragments by the proven centroid-then-predicate
   discipline — the centroid for the coarse cut, the vendored
   Shewchuk predicates (`orient3d`) for the band, per ADR-0004's
   explicit tolerance.
3. **Assemble:** the proven pattern, verbatim — canonical vertex
   identity before edge keying, edges by vertex pairs,
   order-independent dedup, `link_radial_edges`, hard
   postconditions (D3's invariant set, now with H).
4. **The new topology the exact path creates:** A's pierced
   faces gain **inner loops** (the intersection circles, split
   at the seam vertices per the ADR-0023 pattern — the counts
   below pin the choice); B's surviving wall is the cylindrical
   patch between the circles; B's caps outside A die; the seam
   edge carries the wall's periodic identification (both its
   co-edges in the wall's single loop — radial-edge legal,
   orientation XOR satisfied).

**The degenerate classes within G1, each with a disposition
(pinned, not discovered later):**

- *Coplanar cap flush with an A face* (the prism's base exactly
  on the box's face): the ADR-0023/0024 coplanar dedup rule
  applies — keep A's face, absorb the flush cap; golden-pinned.
- *Tangential wall* (the hole's wall exactly tangent to an A
  side face from inside): the tie-break is deterministic
  (canonical-key ordering, never input order) and the golden
  pins the chosen disposition; the tolerance band that triggers
  it is the operation's declared Layer-3 tolerance.
- *Circle crossing a face boundary* (the intersection circle
  not strictly interior to the pierced face): **rejected with
  an actionable error** naming the crossing and the remedy
  (move/resize the hole, or accept the faceted path) — the
  non-annular pierced face is honest G2 topology, not G1.
- *Blind hole* (the prism terminating inside A): **rejected
  with an actionable error** — the internal cap face class is
  G2's (it needs the SSI-driven cap insertion, not the
  pass-through circle pair G1 builds).

**The guide case's pinned arithmetic (the table every test
golden and every hand-check reads):** box a×b×c, through-hole
radius r, axis ⊥ the two pierced faces, circles strictly
interior:

| Quantity | Value | Source |
|---|---|---|
| V | 8 + 4 = 12 | box corners + seam/split vertices (2 per circle, ADR-0023 pattern) |
| E | 12 + 2 + 2 + 1 = 17 | box edges + top arc pair + bottom arc pair + seam |
| F | 4 + 2 + 1 = 7 | 4 sides + 2 pierced faces + wall |
| H | 2 | one inner loop per pierced face |
| χ | 12 − 17 + 7 − 2 = 0 | genus 1, single shell — agrees with the faceted report |
| Volume | abc − πr²h | box minus the full cylinder (h = the pierced span) |
| Centroid | on the symmetry axis (centered case: unchanged from the box's) | symmetry |
| I_zz (coaxial) | abc(a²+b²)/12 − πr⁴h/2 | box's I_zz minus the removed cylinder's (m = πr²h, I_zz = mr²/2), parallel-axis on the common axis |

Every row is hand-checkable — the order's "verifica a mano
possibile dall'utente" is this table, and the tests pin it.

### D8. The provenance contract — "is this edge exact?" always has an answer

Every curve-carrying entity (edge, and via it the co-edge's
p-curves) carries the provenance field of D2.4: `exact` or
`approximated(tol)`. The rules:

- Factories write `exact` (closed forms, tolerance 0).
- The SSI entries write `exact` (the conic sections are closed
  forms).
- The faceted bridge path writes `approximated` with the grid
  tolerance it already declares — the ADR-0026
  declared-error culture, now a first-class field instead of a
  report footnote.
- **The answer is a query, not an investigation**: the bindings
  expose it per subshape; the Debug report lists it per body
  (n exact / m approximated, with the max tolerance); the mass
  API's `all_exact` flag (D4) is the aggregate of the same
  field. No code path, UI surface, or export may present
  approximated geometry as exact — the field is the enforcement
  point, checked by tests.

What this buys the platform vision (H1's "declared tolerances"
ask): a preprocessor consuming CAD_0 bodies knows, per edge,
whether it is consuming mathematics or a sample — the difference
that decides meshing strategy, and the reason the field is
serialized in the report rather than living in comments.

### D9. The test class — discriminating, failure regime in range

The G1 tests are written **first** (the review agrees them; the
phase implements against them). Three discriminator families,
each failing today for the right reason:

1. **API-surface discriminators** (fail today with
   `AttributeError`/missing binding — the wiring-test pattern):
   `mass_properties` absent; the p-curve accessor absent; the
   provenance query absent; the extended `euler_characteristic`
   loop-count semantics absent.
2. **Geometry discriminators** (fail today on the faceted path —
   they assert what only the exact path can produce):
   - The hole's wall is **one** cylindrical face (today's faceted
     extract: N planar strips).
   - The wall face's surface is a `CylinderSurface` with the
     sketch's radius (today: `PlaneSurface`s).
   - The pierced faces carry inner loops (today: none).
   - The guide-case table of D7.5: V/E/F/H pinned at
     12/17/7/2, χ = 0, genus 1 — the faceted body reports the
     same χ/genus but never these counts.
3. **Closed-form goldens** (fail today for absence of the API;
   tomorrow they are the arithmetic's proof): volume, centroid,
   inertia for the box (abc, center, abc(a²±…)/12 family), the
   sphere ((4/3)πR³ et al.), the cylinder (πr²h et al.), the
   circle-prism, and the guide case (abc − πr²h; the I_zz
   combination of D7.5). The classical formulas are the *tests*,
   never the implementation — the implementation is the
   divergence-theorem machinery, and the goldens are what proves
   it right per family.

Plus the invariant-extension tests (holed faces: the pierced
face's honest χ; backward compatibility: every existing golden
body H = 0, value unchanged), the degenerate-class dispositions
of D7 (each family pinned to a golden or to its actionable
error), the provenance rules of D8, and the determinism family
(bit-identical re-runs, quantized serialization, per ADR-0014).

**Entry criterion for the phase:** this ADR accepted after
review — invariants and discriminating tests agreed — on top of
ADR-0031 accepted (done, 2026-10-25).

### D10. Delivery units — the house sequence

Six units, the order's arrow (D3 → primitive → extrude →
subtract → massa → report), each landed standalone with the full
suite green and visible value — the countermeasure the
acceptance motivation cites, enforced:

1. **The data model** (D2 + D3): the 2D vocabulary reuse, the
   p-curve field, the tolerance/provenance fields, and the
   extended `euler_characteristic` with its derivation and
   backward-compat proof. No behavior change; the invariant
   function's tests are the unit's proof.
2. **The primitives with true trimming** (D1.1): box, sphere,
   cylinder factories gain p-curves, trim domains, `exact`
   provenance, zero tolerance — counts unchanged (the existing
   goldens are the compatibility proof), new goldens for the
   p-curve surface.
3. **The exact extrude** (D6): the profile→face mapping, the
   seam reuse, the caps from the sketch chains, the
   `HybridBody` landing. Visible value: prisms with true faces
   in the tree, inspectable via the bindings.
4. **The exact subtract** (D5 + D7): the SSI entry (⊥, ∥), the
   class check with its actionable errors, the degenerate
   dispositions, the guide-case counts. Visible value: the hole
   with a cylindrical wall — the geometry the phase is named
   for.
5. **The mass properties** (D4): the divergence-theorem engine,
   the closed forms, the API + bindings, the `all_exact`
   honesty flag. Visible value: the numbers, on every G1 body.
6. **The report and the guide case end-to-end** (D8 + the
   chain): the Debug report grows mass + provenance next to
   χ/genus; the UI shows the numbers read-only; the example
   walks the chain and prints the analytic comparison; the
   docs (README roadmap) updated on landing.

Each unit's entry criterion: the previous unit green. The
phase's exit: the guide case verified end-to-end — including
the user's manual click-test, the ADR-0030 lesson (the suite
covers contracts; the hands cover the experience).

### D11. What G1 does NOT do — the no-promises list

- **G2 is not scheduled, scoped, or implied.** The internal
  checkpoint (ADR-0031, amended D5) compares G1's measured
  effort and robustness against the estimates — nothing more.
- **No fillet, no NURBS, no oblique SSI in the exact path** —
  each with its declared boundary and remedy in D1/D5/D7.
- **The `.CAD_0` format does not move.** Recipes already
  describe the chains; the exact B-Rep is a deterministic
  materialization (the D6 landing), so v0.4 projects open
  unchanged and no migrator is needed. If a unit discovers a
  serialization need (e.g., caching materialized bodies for
  load-time), the 0007-series companion ADR ships with *that*
  unit, per ADR-0007's rule — not before, not speculatively.
- **No new third-party anything** — by the amended ADR-0003's
  whitelist, and by road (a) itself.
- **No UI redesign**: the report gains fields; the property
  panel gains read-only numbers; the sketcher workflow is
  untouched (the same gestures, the same recipes — better
  geometry underneath).

## Consequences

- If accepted: G1 starts under ADR-0032 with the invariants,
  test class, and unit ladder pinned above; the six units land
  in the house cadence (each a delivery with the full suite
  green); the README roadmap's Phase G line moves as units land.
- If amended: the D-sections are independent by design — the
  review can move the oblique-SSI boundary, the unit sequence,
  or the API shape without rewriting the document; the
  invariant's loop term (D3) is the one derivation the review
  should check hardest, because every count in the ADR depends
  on it.
- The faceted bridge, the SDF display path, and every existing
  golden are untouched by this ADR's acceptance — the phase adds
  a path, it does not replace one.
- The discovered findings this draft contributes back to the
  record: the inner-loop term of the Euler invariant (D3), the
  sinusoid p-curve boundary (D5.3), and the HybridBody landing
  (D6) — each derived, each flagged for the review's attention.

## References

House records:

- ADR-0031 (the mother: the ladder, D3's data model and
  tolerance layers this ADR concretes, D5's G1 rung, D6's
  migration and SDF boundary, the accepted road (a)).
- ADR-0021/0022 (the topology spine, the invariants, the query
  operators — reused verbatim), ADR-0023 (the seam-wire pattern
  and the cylinder/sphere counts G1 reuses), ADR-0024/0025
  (the boolean pipeline and the canonical-key assembly pattern
  D7 generalizes), ADR-0026 (the bridge whose declared-error
  culture and χ/genus report D8 and D3 promote), ADR-0027
  (recipes as source of truth — why the format does not move),
  ADR-0030 (the 2D typed kernel D2 reuses; the
  failure-regime-in-range test discipline D9 applies), ADR-0002
  (the HybridBody that finally gets its job), ADR-0004 (the
  explicit tolerances every classification receives), ADR-0005
  (the naming ABI the report's provenance complements),
  ADR-0007 (the format rule D11 honors by not moving it),
  ADR-0014 (determinism, scoped per D4), ADR-0003 as amended
  (the whitelist — why no dependency question exists in G1).
- docs/architecture/platform-vision.md (the consumers these
  contracts serve: H1 asks the invariants, the tolerance
  layers, and the interrogable faces this ADR delivers).

Literature:

- Mäntylä, M. (1988). *An Introduction to Solid Modeling*.
  Computer Science Press. (The generalized Euler-Poincaré
  relation for shells with holed faces — D3's loop term; Euler
  operators.)
- Mirtich, B. (1996). Fast and Accurate Computation of
  Polyhedral Mass Properties. *Journal of Graphics Tools* 2(2).
  (The polyhedral reduction D4's machinery subsumes for
  approximated bodies.)
- Hoffmann, C. M. (1989). *Geometric and Solid Modeling*. Morgan
  Kaufmann. (SSI degeneracy taxonomy — the reason D5/D7 pin
  their degenerate classes up front.)
- The classical conic sections and circular-segment formulas
  (D5's ellipse, D4's planar composites) — textbook geometry,
  cited as classical; the ADR derives what it uses.
- Weiler, K. (1986); Shewchuk, J. R. (1997) — the radial-edge
  structure and the robust predicates the pipeline stands on.
