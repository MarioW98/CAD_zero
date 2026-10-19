# ADR-0029: The twist warp contract — bounds, Lipschitz, gradient, and construction-time validation

## Status

**Accepted (2026-10-20).** Proposed 2026-10-19 as a Draft; revised
the same day on the first review round, which accepted D1–D5 with
three conditions — (1) D1's on-axis grid cost stated plainly: √2 on
the diagonal, not near-coincidence; (2) D3's guinea pig scoped to
bounds and topology, not the gradient; (3) D4's zero-movement
prediction qualified and backed by the two-contract consumer
inventory — all folded in at their sites in the revised draft; the
second review round accepted the revised record without further
corrections. Unlike ADR-0028, which registered a decision already
taken, this record argued one — and the argument is now closed.
Delivery order (the MVP, one commit per unit, suite + CI green
after each, movements classified per the D4 rule): the kernel
contract fix (1), construction-time validation (2), the pinned
sweep (3), the property closure (4). This acceptance commit is
documentation-only: zero code, zero tests, no pins touched.

MVP delivery status: unit 1 (kernel contract fix) and unit 2
(construction validation) delivered and green; unit 3 — the
pinned sweep — verified 2026-10-20: full pin matrix green, goldens
bit-identical, both ray-march watch items re-measured in tolerance
(L 2.0 → 1.618, hit 3.999736 in 11 steps vs plain 4.000000 in 2),
zero pinned movement — the conscious re-pin class remains empty,
no re-freeze issued.

## Context

`field.hpp` defines the `SDFNode` contract this ADR extends to the
twist warp:

> `bounds()` — *Local-space bounding box of the region where the
> field can be negative (inside the solid). Used to cull evaluation
> outside the solid of interest.*

ADR-0028's operator audit closed every mixer and left exactly one
reported gap: `TwistSDF`. The warp rotates each horizontal slice by
θ(y) = y·k around the Y axis; a child whose negative region sits off
the twist axis can have that region rotated clean out of the child's
box (the audit's exemplar: a sphere at (2, 5, 0) under k = 1 has its
y = 5 slice rotated by 5 rad). `TwistSDF::bounds()` returns the
child's box unchanged — the contract is violated for such inputs.
Nothing today extracts a twisted body at all, on- or off-axis: the
only twist-bearing artifact is the native-format canonical scene
(an off-axis composite at k = 0.05, max warp displacement ≈ 0.03),
and it is saved and loaded, never extracted.

Writing the fix down surfaced three adjacent findings that belong in
the same decision, because they share one root cause — **the
implementation treats a non-rigid warp as if it were rigid**:

* the declared Lipschitz factor `1 + 2|k|` is not an upper bound of
  the true field slope (D2 derives the correct closed form and the
  failure regime);
* the returned gradient omits the warp's shear term in Y (D2);
* the audit's input-class question — `ScaleSDF` computes `1/s`
  without validating `s > 0` — generalizes to every operator (D5).

Consumers of what this ADR fixes: `mesh_extract.cpp` (auto-bounds
grid = bounds() + 1%-of-extent pad), the ADR-0026 bridge
(`sdf_to_brep`, same layout plus the `min_abs_field` P2 mirror), the
scene-level framing, the ray marchers (CPU; the step contract
`t += d/L`), and — for the gradient fix — the normal paths (MC
vertex normals, OBJ/STL export bytes, the Python display buffer).
D4 inventories the last two contracts' consumers in full.

## Decision

### D1 — The bound: the AABB of the child's axis-cylinder (closed, conservative)

**Decision: `TwistSDF::bounds()` returns the AABB of the cylinder
that carries the child's box — `[−R, R] × [y_min, y_max] × [−R, R]`,
where `[y_min, y_max]` is the child box's Y range and `R` is the
maximum distance from the Y axis over the child box's corners.**
With the k = 0 passthrough: an identity twist returns the child box
verbatim.

#### The structural facts

The sampler evaluates f(p) = g(u(p)) where u rotates p's (x, z) by
the angle k·p.y (the code's `local` point). The forward warp
φ = u⁻¹ — child space to parent space — is therefore also a
per-slice rotation, by θ(y) = y·k. Three facts:

1. **Y is preserved** — slices map to slices.
2. **Radial distance from the Y axis is preserved** — a rotation in
   the slice is an isometry of the (x, z) plane.
3. **φ is a diffeomorphism** — each slice rotation is invertible.
   (This is D3's topology note: the twisted solid is homeomorphic to
   the child.)

#### Conservativity — demonstrated, not asserted

**Proposition.** Let g be the child field, B = the child's
`bounds()` box with neg(g) ⊆ B (the `field.hpp` contract, holding
inductively per the ADR-0028 audit). Then the twisted field
f = g ∘ u satisfies:

    neg(f) ⊆ [−R, R] × [y_min, y_max] × [−R, R],
    R = max over B's corners of sqrt(x² + z²).

*Proof.*

1. `sample()` computes f(p) = g(u(p)) exactly, so f(p) < 0 ⟺
   u(p) ∈ neg(g) ⟺ p ∈ φ(neg(g)): the warp maps the negative region
   exactly. No approximation enters anywhere in the chain.
2. neg(g) ⊆ B — the child's own contract.
3. φ preserves Y and radial distance (facts 1–2), so
   φ(B) ⊆ C := {p : p.y ∈ [y_min, y_max], |p.xz| ≤ R}. R is
   attained at a box corner because x² + z² is convex in (x, z) and
   a convex function on a rectangle attains its maximum at a vertex
   — four candidates.
4. C ⊆ AABB(C), and AABB(C) is exactly the box above. ∎

Two corollaries worth pinning:

* **Monotone:** the new box always contains the old one (every
  |x|, |z| in B is ≤ R by the same corner argument) — the fix can
  only widen a reported box, never shrink one.
* **k-blind:** the proof uses no property of the angle θ(y) = y·k
  beyond its being a rotation. The bound is valid for every finite
  k, any sign, any magnitude, with no angle-range case analysis.
  This — not tightness — is what makes it five lines instead of a
  case tree.

#### The on-axis cost, stated plainly (review condition 1)

The corollaries are free; the bound is not, and the common case
pays it too: **an on-axis child box does not almost coincide with
its cylinder — the radius is the cross-section's half-diagonal, so
the footprint dilates by √2 on the diagonal.** For a centered
square of half-side a, R = a·√2: both horizontal half-extents
grow by exactly √2, the footprint and the volume double (Y is the
only preserved axis). A centered rectangle [−a, a] × [−b, b] pays
at least √2 on its short axis (both half-extents grow to
√(a² + b²)). The cylinder's radius is tight *given the child's
reported box* — the far corner maps to radius R at every slice,
so no tighter cylinder follows from the box alone — but tight as a
cylinder is not tight as a box: below saturation (total angle
Θ = k·Δy < π/2) the swept union's true AABB stays close to the
child box, and the whole √2 is over-report.

The grid cost, in the extractor's own semantics: `marching_cubes`
fixes the per-axis cell **count** (`MarchingCubesOptions::
resolution`; cell = extent/res, `mesh_extract.cpp`), so an
inflated box means **bigger cells, not more of them**. The on-axis
√2 footprint is paid as cells √2 wider in x and z — 2× the cell
volume at the same res — so surfaces resolve that much coarser,
and buying the old cell size back costs 2× the cells.
Conservativity is provable, monotone, k-blind — and never free.

**Composition** is by construction, exactly as the k/8 dilation
composes: every parent operator consumes the child's reported box
through its own audited rule (hull, intersect, shift, dilate,
scale), and the proof above only ever needed neg ⊆ B_child. A twist
of a twist computes its R from the inner node's reported box — any
conservative box works.

#### The rejected alternative: slice-tight (b), and the trade-off

**(b) Slice-tight:** the AABB of ∪_y R_θ(y)·(B ∩ slice_y) — the
hull over the angle range of the child rectangle rotated per slice.
It is tighter exactly where (a) is loose. Its derivation is
genuinely doable: on each interval between kink angles the extent
function is a pure sinusoid (a·cosθ + b·sinθ), so the max over the
range sits at endpoints, at kink angles (multiples of π/2 and the
center-phase points), or at per-piece peak angles — a finite,
enumerable set. But it is a real proof obligation plus ~60–100
lines of interval-trigonometry code plus its own verification
property, against **zero current consumers**.

The trade-off, explicitly:

| | (a) cylinder AABB | (b) slice-tight |
|---|---|---|
| derivation + proof | 5 lines, k-blind | piecewise-sinusoid lemma, case enumeration |
| on-axis revolution solid (sphere/cylinder/cone) | √2 linear xz over-report (2× cell volume at fixed res), Θ-blind | the child's own box (tight) |
| on-axis box, total angle Θ ≥ π/2 | matches the saturated swept hull — tight for any box-interface bound | same |
| on-axis box, small Θ | O(R) vs the true O(Θ·R) | tight |
| off-axis child (the canonical composite) | +65% / +171% extent (≈ 4.5× cells) | ≈ the true ~0.03 displacement |
| failure mode if the math is wrong | can only over-estimate (coarser grid) | can under-estimate (a lying bound) |

The use class decides it. The twist vocabulary exists for twisted
columns and organic shapes — bodies *placed on the twist axis*: the
case where (a) is at its best, and even there it costs the √2
diagonal dilation stated above (tight only at saturated sweep,
Θ ≥ π/2), while (b) buys back at most that 2× cell volume.
Off-axis twist is a composite-scene curiosity that nothing extracts.
And the asymmetry that decided ADR-0028 decides D1 the same way:
**(a)'s failure mode is a coarser grid — visible, bounded, buyable
back with an explicit res; (b)'s failure mode is the very contract
lie this ADR exists to close, hidden inside interval-trig code with
no consumer to trip over it.** A bound's cost is paid only by
extraction grids; today there are zero twist-extraction consumers,
so (b)'s derivation cost is 100% upfront and its benefit 100%
deferred — the wrong sign for an MVP.

**(b) is named, not dismissed:** it is the deferred refinement, with
the piecewise-sinusoid sketch above as its starting point. Entry
criterion: a real extraction use case for off-axis or small-Θ
twisted bodies whose grid cost materializes (e.g. twisted columns in
production). At that point its derivation, proof, property, and pin
sweep are paid by that use case — the same discipline that paid for
this ADR.

For the record, the canonical composite under (a): the child box
(after the k/8 dilation of its own smin, k = 0.15) is
x ∈ [−1.019, 3.319], z ∈ [−1.319, 1.319], so
R = √(3.319² + 1.319²) ≈ 3.571 and the new xz range is
[−3.571, 3.571] — extents grow +65% (x) and +171% (z): ≈ 4.5× the
cell volume at fixed res, i.e. ≈ 4.5× the cells to buy the old
cell size back. The true warp displacement of its
only off-axis member (the torus at radial distance 2,
y ∈ [−0.32, 0.32], k = 0.05) is ≤ 0.032. That gap — 4.5× cells vs
0.03 — is the honest price of the five-line proof, paid never
(today) or on demand (the entry criterion).

### D2 — Lipschitz and gradient: the warp is not a rigid rotation

Both the declared constant and the returned gradient treat the warp
as rigid. Neither survives the chain rule.

#### The true Lipschitz factor — derivation

f(p) = g(u(p)) with u(p) = the slice rotation of p by −k·p.y. The
Jacobian of u has columns v_x = (c, 0, −s), v_y = (k·u.z, 1, −k·u.x),
v_z = (s, 0, c) — the y-column is the *shear*: the rotation angle
itself depends on p.y, with a lever arm of the point's radius. Then

    |∇f(p)| ≤ σ_max(J(p)) · L_child,
    JᵀJ has λ_max = 1 + (t + √(t² + 4t))/2,   t = (k·r_p)²,

where r_p = |p.xz| is the evaluation point's distance from the
axis. The honest local factor is therefore

    σ(k, r) = √(1 + (t + √(t² + 4t))/2),   t = (k·r)²,

with the sandwich (each direction one line):

    √(1 + (k·r)²)  ≤  σ(k, r)  ≤  1 + |k·r|.

σ grows without bound as r → ∞: **the twisted field is not globally
Lipschitz** — no finite constant can be declared for all of space.
The declaration is necessarily regional, and the region is the
bounds() box of D1: the new declaration is
**L_twist = L_child · σ(k, r_box)** with r_box = √2·R, the corner
distance of the new AABB — covering every in-box consumer, the MC
grid and its 1% pad included. The residual — a ray marching far
off-axis, outside the box, where the local σ exceeds the declared —
is documented here as inherent to any non-globally-Lipschitz warp;
the alternative (declaring ∞) would cripple the marcher for the
on-axis cases that actually exist.

#### The conviction of the declared 1 + 2|k|

The coded factor `L_child · (1 + 2|k|)` is dimensionally blind: k is
radians per unit length, so no function of k alone can bound a
factor that depends on the dimensionless k·r. Numbers:

* k = 0.05, a child box reaching r = 10: true σ = 1.281 vs declared
  1.1 — the true slope exceeds the declared by 16%; the marcher
  over-steps.
* Small-k asymptotics: σ ≈ √(1 + |k|r) fails 1 + 2|k| exactly when
  r ≳ 4 + 4|k| — **k-independent**: any nonzero k on a child
  reaching r ≈ 4 breaks the declared bound. A unit-origin scene
  (r ≤ ~3) never trips it, which is why nothing ever has.
* The canonical composite (k = 0.05, box corner r = 3.571): true
  σ = 1.0933 vs declared 1.1 — the scene runs at **99.4% of the
  declared budget**. Luck, not design.
* The code comment's claimed truth `sqrt(1 + (k·R)²)` is also not
  an upper bound — it is the *lower* edge of the sandwich (it bounds
  only the y-partial |∂f/∂y|, not the gradient norm; at k·r = 1 it
  says 1.414 where the true factor is 1.618).

The declaration has been safe in composition only by accident: the
smin margin (2.5× vs the true 1.75×) absorbs a child
under-declaration up to 2.5/1.75 = 1.429×, which covers the twist
lie up to r ≈ 19 at k = 0.05. `operators.hpp` even cites "TwistSDF
reports a conservative L" as *reason* for that margin — the citation
is false in this regime, and the MVP corrects it.

**Fix (MVP commit 1):** `lipschitz()` returns
`L_child · σ(k, √2·R)` with the closed form above (two square
roots; the sandwich is the proof device and the sanity envelope).
For the classic test geometry (box {1,1,1}, k = 0.5): R = √2,
r_box = 2, σ = 1.618 — *smaller* than the old 2.0, so the marcher
takes ~24% larger steps there, while finally being honest in the
wide-R regime (the canonical composite moves 1.1 → 1.134).

**Empirical mandate:** the property family (D3) verifies the NEW
declaration by pair-Lipschitz and gradient sampling on trees with
twist, with generator ranges that *include* the old failure regime
(r up to ≥ 6 with k = 0.05) — a property whose range avoids the
regime is vacuous. If the new declaration fails empirically: it is
a bug, same procedure as ADR-0028 (separate fix commit, pinned
regression, no invariant loosened).

#### The gradient omits the shear term

The chain rule gives

    ∂f/∂p.y = g.y + k · (u.z · g.x − u.x · g.z)

(the y-column of J dotted with the child gradient at u). The code
returns `g.y` alone: the x/z components are correct, the y component
misses the shear term — wrong whenever the child gradient has a
tangential component at an off-axis point. Consequences: mesh
normals are wrong in Y for any twisted extraction (latent — nothing
extracts twist today; the D3 bridge family would freeze wrong
normals if it landed before this fix). And the current property
suite is blind to it: property (e) checks |∇f| ≤ L, and the buggy
gradient's norm is exactly |∇g| (the xz rotation is an isometry),
so it passes trivially — **a norm-only property cannot see a
direction error**. The D3 family adds gradient-vs-finite-differences
consistency (the property class that would have caught this), and
the MVP commit returns the full chain-rule gradient (note for the
implementation: the shear term uses the child-frame gradient and
the local point — compute it before the forward xz rotation).

### D3 — The property families (added with the implementation, after acceptance)

**Operator family** (`test_operators`), extending the seeded
generator:

* a twist category in `gen_tree`, default-off (the `allow_blend`
  precedent: pinned draw sequences untouched), with on-axis AND
  off-axis placement (translate-then-twist), k in a declared range
  with margins — including negatives (the bound is |k|-independent
  and the Lipschitz even in |k|) — and child radii covering the wide
  regime (r ≥ 6 with k = 0.05: the old declaration's failure zone);
* **liveness guard** (the FASE 1 lesson, sharpened): a body of
  revolution about Y is twist-*invariant* — a twisted on-axis
  sphere is the same sphere — so a generator drawing only
  revolution primitives passes even a wrong twist implementation.
  The guard counts members where the twist actually changes the
  field (probe f_twist ≠ f_child) and requires a threshold;
  non-revolution children (boxes, or any primitive off-axis) must
  be in the mix;
* **positivity outside the NEW bounds, stratified** — the k/16-shell
  lesson: distance bands [0, ε], [ε, 2ε], ... from the bounds()
  boundary, dense in band 0. The twist's violation class is
  additionally *slice-structured* (the escape lives at specific
  heights), so the sampling stratifies by y as well — per-slice
  dense rings just outside the cylinder radius;
* **pair-Lipschitz** against the new declaration, and
  **gradient-vs-FD consistency** (direction, not just norm — D2) —
  these two are the gradient contract's witnesses, run on the
  generator family (the deterministic guinea pig is scoped out of
  the gradient contract: see its bullet);
* the deterministic guinea pig, pinned in-suite — **scope (review
  condition 2): bounds and topology, NOT the gradient.** The
  audit's exemplar — sphere r = 0.25 translated to (2, 5, 0),
  twist k = 1. Its y = 5 slice rotates by 5 rad: the disk center
  moves to (0.567, −1.918), deep outside the old box
  (x ∈ [1.75, 2.25]) — pin f < 0 there (the OLD contract
  violation, frozen as the regression), and pin f > 0 strictly
  outside the NEW box ([−2.264, 2.264] × [4.75, 5.25] ×
  [−2.264, 2.264]); plus the topology half of its scope — extract
  it under the new bound (an explicit res sized by the honest-cell
  rule on the dilated xz extent: res ≈ 96 puts ~10 cells across
  the 0.5-diameter sphere on the 4.53-extent grid; numbers pinned
  at implementation time) and assert the ADR-0026 contract set:
  χ = 2, genus 0, closed, orientable, E = 3F/2, V = F/2 + 2,
  min|f| > 0, bit-identical determinism — the first off-axis
  twisted extraction the kernel could produce at all, exactly the
  extraction the old bound broke. The sphere carries **no gradient
  assertion**: witness separation — a failure there must mean
  bounds or topology; the gradient contract has its own witnesses
  (the generator family's gradient-vs-FD and the σ k-sweep), and
  the sphere's regime (k = 1, r ≈ 3.6 → σ ≈ 3.9) is the
  σ-dominated magnitude story the k-sweep already owns;
* the k-sweep guinea pig (the (f) precedent): measured max slope vs
  the σ(k, r) closed form across a declared (k, r) grid.

**Bridge family** (`test_bridge`): extraction of twisted ON-AXIS
bodies with the full ADR-0026 contract set (the guinea pig above
carries the off-axis member of the topology witness). The
topological note
that makes the contracts assertable: **the twist is a
diffeomorphism — per-slice rotation, invertible — so the twisted
solid is homeomorphic to the child and the genus is preserved: a
twisted genus-0 prism stays genus 0, χ = 2.** Members: on-axis
twisted prisms (the classic twisted column — non-revolution, so the
coupling is real) and prism∪sphere composites; total angle
Θ = k·H in a declared range with the honest-cell margin adapted
(the warped surface's per-cell angular advance bounded; the
P3-style rejection on the effective cell — the T4 lesson);
invariants per member unchanged and unloosened: fit, χ = 2, genus 0,
orientable, loops, E = 3F/2, V = F/2 + 2, min|f| > 0, bit-identical
determinism. Numbers pinned at implementation time.

### D4 — The pinned sweep: inventory and prediction

Every twist usage in the tree, classified (grep-verified, 18 files):

| Site | What it does with twist | Verdict |
|---|---|---|
| `test_transforms.cpp:44` | sanity: value < 0 at origin, L > 1 | holds (new L = σ(0.5, 2) = 1.618 > 1) |
| `test_operators.cpp:471-481` | Lipschitz composes as max | holds (max(1, 1.618) = 1.618, exact) |
| `test_ray_march.cpp:206, 261` | twisted-box march: hit ≈ 4.0 (ε = 0.1), steps ≥ plain | the only twist-adjacent observations that change value at all: L 2.0 → 1.618, ~24% fewer steps. Both assertions robust: the ray lies in the y = 0 plane, where the slice is unrotated — the crossing is exactly 4.0 — and steps ≥ plain holds while L > 1 |
| `test_native.cpp:84` + `golden/native_v0_1_0.CAD_0` | the canonical scene: save/load round-trip only | bytes unchanged — bounds(), lipschitz() and gradients are computed, never serialized; the scene is in-class for D5 |
| Python surface (`sdf_bindings.cpp:60`, `sdf.py`, `cli.py`) | exposure only | no Python test constructs a twist body |
| `ray_march.cpp`, `sdf_raymarch.frag` | comments citing TwistSDF as the L > 1 example | no pinned behavior |
| property generators (`gen_tree`) | — | no twist nodes today (D3 adds the category, default-off) |

**The prediction, qualified (review condition 3): zero movement of
today's pinned set — with an explicit inventory of what
legitimately could move.** What "zero" covers, precisely:

* **the extraction-topology pins** — no pinned test extracts a
  twisted body (grep-verified; the only embedded golden mesh is
  the v0.2.0 imported octahedron, not an MC extraction);
* **the save/load bytes** — `bounds()`, `lipschitz()` and
  gradients are computed, never serialized (`native.cpp` writes
  op + parameters + children only), so both twist-bearing goldens
  are immune by construction;
* **every pin whose tree contains no twist node** — the entire
  ADR-0026/0027 set, under the one-unit rule (the ADR-0028
  consequence-2 discipline, now covering everything).

The sweep commit still runs the full matrix and records the
evidence; the two ray-march soft pins stay the designated watch
items, expected to hold by the margin analysis above. But "nothing
moves" is a statement about today's pin set, not about the
contracts' consumers: the two field-contract fixes change real
downstream values wherever a twisted body meets them, and the
sweep must tell a conscious re-pin from a bug. The inventory:

**Consumers of the gradient contract** (the y-shear fix, MVP
commit 1):

| Consumer | Where | Effect of the fix | Pinned today | Verdict |
|---|---|---|---|---|
| MC vertex normals → `mesh.normals` | `mesh_extract.cpp` (`interp_gradient_abs`, MC33 v12) | the Y component of every normal of every twisted extraction | nothing extracts twist | must not move — nothing to move |
| OBJ export bytes (`vn` lines) | `obj.cpp` | vn bytes of twisted bodies | sphere fixtures only | must not move; a golden embedding the canonical k = 0.05 body would re-pin **consciously** — the class is empty today |
| STL export bytes (facet normal = mean of 3 `vn`) | `stl.cpp` | same | sphere fixtures only | same |
| Python display buffer | `display_normals` (`sdf_bindings.cpp`) → `glNormal3f` (`viewport_widget.py`) | runtime normals of every twisted body — the visible point of the fix | perforated box (ADR-0027, no twist) | the pin must not move; the runtime change is the fix working |
| Bridge | `sdf_to_brep.cpp` — triangle geometry only, never `mesh.normals` | nothing | — | unaffected, on record |

**Consumers of the Lipschitz contract** (the σ declaration, MVP
commit 1):

| Consumer | Where | Effect of the fix | Pinned today | Verdict |
|---|---|---|---|---|
| CPU marcher step budget | `ray_march.cpp` — step = d·scale/max(L, 1) | classic box k = 0.5: L 2.0 → 1.618, ~24% fewer steps; canonical composite: 1.1 → 1.134, ~3% more | the two soft pins (watch items) | robust per the site table above |
| Canonical composite, marched at runtime only | — | today at 99.4% of the declared budget (true σ = 1.0933 vs 1.1 — luck, D2); after the fix the declaration is the proven σ: **conservative — slower, safe** | save/load only | no pin; the behavior change is on record here |
| GLSL marcher | `sdf_raymarch.frag` (unit sphere, L = 1) | nothing | — | unaffected |

**The classification rule the sweep enforces:** a movement on a
twist-free tree, on a save/load byte, or on a topology count is a
new bug — stop, separate fix, pinned regression, no invariant
loosened (the ADR-0028 discipline). A movement on a twisted
extraction's normals or march steps is the **conscious re-pin
class** — re-freeze with the justification in the commit body,
exactly as ADR-0028 re-froze the blended bridge counts. Today
that class is empty; it is inventoried so that a future pin
landing on a twisted body knows its re-pin is legitimate, not
silent.

### D5 — Construction-time validation: close the door on the unsupported class

**Decision: every operator validates its own inputs at construction
and throws `std::invalid_argument` naming the operator, the
parameter, its value, and the supported class.**

The findings that force it (all silent today):

* `ScaleSDF` computes `1/s` unvalidated: s = 0 → inf → NaN fields;
  s < 0 flips the field sign, making the negative region the
  *unbounded complement* of the mirrored child — no box can satisfy
  bounds() for it. This is not a degenerate case of the contract;
  it is outside the class the contract can describe.
* `SmoothMinSDF` with k < 0 silently coerces to the hard-min branch;
  k = NaN flows into the polynomial and produces NaN fields.
* `UnionSDF`'s two-operand constructor *silently drops null
  children* — `sdf_union(released_empty_body, x)` is x: operand loss
  without a trace.
* `IntersectionSDF`, `SubtractionSDF`, all transforms: a null child
  is dereferenced on sample — UB.
* `RotateSDF` with a non-unit quaternion rotates-*scales* the field
  (by ‖q‖²): the rigid-rotation Lipschitz and gradient claims break.
* Primitives: r ≤ 0 gives inverted or empty boxes and division-by-
  zero gradients (`SphereSDF` computes `p / radius_`).

**Why construction, why not documentation.** Construction is the
single choke point: the C++ API, the Python bindings, the native
loader, the property generators, and the UI all pass through it —
validating any outer layer leaves the kernel API unprotected (the
property suite itself constructs nodes directly). Construction is
cold path (once per edit, never per sample), so the branches are
free. The loader *gains* fail-fast corrupt-file detection for free:
`native.cpp` is already throw-based under the ADR-0007
explicit-error policy, and constructor exceptions join the same
error family its tests already assert. And the alternative —
documenting the class without enforcing it — is exactly the status
quo that produced the ScaleSDF finding; ADR-0028's lesson is that
silent tolerance of an undocumented class is where contracts go to
die.

**Cost, honestly:** constructors drop `noexcept` (`sample()` keeps
it — no hot-path change); previously-tolerated garbage now throws,
an external-code compatibility surface (audited in-repo: the only
near-hits are `sdf_scale(cone, 0.5)` and
`sdf_smooth_union(..., 0.0)` — both in-class under the checks below;
the canonical scenes are in-class; predicted zero test churn). And
the list stays minimal on principle: **close the door on the
contract-breaking class — non-finite, sign-flipping,
degenerate-empty, null — and do not legislate taste** (a = b
capsules stay legal: they degenerate to a sphere behind an explicit
guard; spindle tori r > R stay legal: the offset field and its box
remain valid).

**The exact list:**

| Node | Checks (all imply finite) |
|---|---|
| `SphereSDF` | radius > 0 |
| `BoxSDF` | extent.x, extent.y, extent.z > 0 |
| `CylinderSDF` | radius > 0, height > 0 |
| `TorusSDF` | R > 0, r > 0 (r > R = spindle: allowed, documented) |
| `ConeSDF` | r > 0, h > 0 |
| `CapsuleSDF` | a, b finite; radius > 0 (a = b allowed — guarded sphere degeneration) |
| `PlaneSDF` | — (no parameters) |
| `UnionSDF` (2-arg) | a and b non-null — the silent drop becomes the loud error it always should have been |
| `UnionSDF` (N-ary) | non-empty, all non-null |
| `IntersectionSDF` | a, b non-null |
| `SubtractionSDF` | a, b non-null |
| `SmoothMinSDF` | a, b non-null; k ≥ 0 (k = 0 = the documented hard min, stays; k < 0 rejected — the polynomial derivation assumes k > 0) |
| `TranslateSDF` | child non-null; offset finite |
| `RotateSDF` | child non-null; ‖q‖ = 1 within 1e-6 (serialization round-trips at ~1e-9; NaN rejected by the norm check) |
| `ScaleSDF` | child non-null; s > 0 (the motivating rejection) |
| `TwistSDF` | child non-null; k finite (any sign; k = 0 = identity, allowed) |

## MVP (post-acceptance; one commit per unit; suite + CI green after each)

1. **Kernel contract fix** — `TwistSDF`: bounds() = the D1 cylinder
   AABB (with the k = 0 passthrough); lipschitz() = the D2 closed
   form over r_box = √2·R; sample() returns the full chain-rule
   gradient. The KNOWN-GAP comment block becomes the contract text;
   the false "conservative L" citation in `operators.hpp` corrected.
2. **Construction validation** — the D5 list across primitives,
   operators, transforms; constructors lose `noexcept`; loader
   error-path tests extended (a scale s = 0 node in a file → an
   explicit load error, joining the ADR-0007 family).
3. **Pinned sweep** — the D4 inventory re-verified live: full pin
   matrix + goldens bit-identical + the two ray-march watch items
   re-measured within tolerance; recorded in the commit body. No
   re-freeze is expected — the conscious re-pin class (twisted
   normals, march steps) is empty today; per the D4 classification
   rule, a movement on a twist-free tree, a save/load byte, or a
   topology count = stop — new bug, separate fix + pinned
   regression, no invariant loosened.
4. **Property closure** — the D3 families: the generator twist
   category (default-off), stratified per-slice positivity outside
   the new bounds, pair-Lipschitz against the new declaration with
   the failure regime in range, gradient-vs-FD consistency, the
   off-axis guinea-pig pins (bounds + topology, no gradient — the
   review scope), the σ k-sweep, and the twisted-prism bridge
   family with the full contract set.

## Out of scope (explicit)

* **Smooth subtraction / smooth intersection** — future vocabulary;
  a smooth-max-side operator's dip analysis is not analogous to
  smin's and gets its own derivation when it exists.
* **Arbitrary warps** — bend, taper, mirror, repeat, symmetry (the
  `field.hpp` vocabulary comment lists them; none exist in the
  kernel today). Each arrives with its own invariants→bounds,
  Jacobian→Lipschitz, and chain-rule→gradient derivation, its own
  entry in the D5 table, and its own ADR amendment. The gate this
  ADR sets: **new operators arrive validated.**
* **Slice-tight bounds (b)** — deferred with the D1 entry criterion
  and derivation sketch.
* **Global Lipschitz for warps** — the twisted field is honestly
  not globally Lipschitz; the regional declaration plus the
  documented off-axis residual stands. Clamping the warp domain to
  force a global constant would change the field semantics.

## Consequences

1. The `field.hpp` bounds() contract becomes honest for twist, on-
   and off-axis: P1 (the extraction volume contains the surface)
   holds by construction for twisted trees.
2. No pinned count, no golden byte, no pin of any kind moves — the
   D4 qualified prediction, verified live at sweep time: movement
   on a twist-free tree, a save/load byte, or a topology count =
   new bug; the conscious re-pin class (twisted-extraction normals,
   march steps) is inventoried in D4 and empty today.
3. The ray marcher gets honest steps: ~24% fewer in the classic
   regime (L 2.0 → 1.618), safe where the old declaration lied
   (the wide-R regime, true slope up to 16% over declared), and the
   canonical composite's L moves 1.1 → 1.134 — its march becomes
   conservative: ~3% slower, safe by proof instead of by the 99.4%
   luck (D2).
4. Twisted extraction becomes possible with correct normals — the
   bridge family lands on the fixed gradient; nothing existing is
   affected because nothing extracted twist before.
5. Corrupt files and UI typos fail loudly at the constructor
   instead of producing NaN meshes or inside-out solids; zero
   in-class behavior change (audited).
6. The record stops claiming luck as design: the smin 2.5× margin's
   "TwistSDF reports a conservative L" citation is corrected, and
   the canonical scene's 99.4%-of-budget near-miss is on record as
   the accident it was.
7. The canonical composite's bounds grows (+65% / +171% extent) —
   a prospective cost paid only if off-axis twist extraction ever
   happens, and buyable back with an explicit res. The on-axis case
   carries its own standing cost — the √2 diagonal dilation of D1,
   paid by every twisted-column extraction at fixed res, not just
   by the off-axis curiosity.
