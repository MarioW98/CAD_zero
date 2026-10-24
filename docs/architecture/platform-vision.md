# Platform vision — CAD_0 as the geometric core of a CAE platform

Orientation document, not a decision record and not a scope
commitment. It records where the kernel is headed as the core of
a broader CAE platform, so that geometric decisions taken rung by
rung (the ADR-0031 ladder) serve the destinations instead of
discovering them late. It is linked from
[overview.md](overview.md); the decisions themselves live in the
[ADRs](../adr/) as always.

**The rule of this document — stated once, binding on every
line that follows: the horizon orients, it does not widen scope.**
Nothing written here adds a rung to the ladder, moves an entry
criterion, or creates a promise of timing on any future phase.
The ladder (ADR-0031, accepted: road (a), all in house) is the
delivery method — the elephant eaten one piece at a time, each
piece with its own entry criterion and its own visible value.
When a horizon below needs something the ladder does not already
carry, that need is recorded as a *consumer requirement against a
future rung's ADR* — to be weighed in that ADR's review, never
pre-granted here.

## The three horizons

### H1 — The preprocessor (the ANSA role)

The classical CAE preprocessor (the reference style: ANSA, by
BETA CAE Systems): the tool that sits between CAD geometry and
the solver, turning exact geometry into *analysis-ready* models —
quality surface and volume meshes for FEM and CFD, model
preparation (mid-surface extraction, cleanup, defeaturing,
imprinting for connections), named selections for loads and
boundary conditions, and mesh quality control with repair
reporting.

What this horizon asks of the kernel — every item is a kernel
contract, not a meshing promise:

- **Closed bodies with exact integer topology.** The Layer-1
  invariant family (structural closure; the Euler-Poincaré
  relation with the inner-loop term, ADR-0032 D3) is what makes
  "watertight" a *proved* property instead of a mesh statistic.
  A preprocessor that cannot trust the incoming topology spends
  its life healing; one that can, spends it meshing.
- **Interrogable faces.** Face typing (analytic families), the
  query operators (ADR-0022), edge/face adjacency — the surface
  on which meshing rules act (per-face sizing, curvature-driven
  refinement, boundary-layer sources on cylindrical faces).
- **Declared tolerances.** Layer 2/3 (ADR-0031 D3, ADR-0004):
  the mesher must know how much slack the geometry carries to
  decide what "on the surface" means and what a sliver is.
- **Controlled analysis tessellation.** `tessellate.hpp` today is
  display-grade; analysis-grade extraction (sizing fields,
  curvature adaptation, quality metrics on the output) is the
  consumer requirement this horizon records against the G-rungs
  — G1's exact faces are its *input*; the quality machinery
  itself is not kernel scope and is not promised here.
- **Named selections.** ADR-0005's `ShapeId` genealogy
  (`FeatureId + SubshapeRef + Generation`) is exactly the
  persistent identity a named selection needs: a selection that
  survives rebuilds because it is *traced*, not matched. This
  asks the naming ABI to keep being honored as operations grow.

### H2 — Parametric vehicle geometry (the OpenVSP role)

Sections, loft between them, guided surfaces, and the parametric
vehicle model built from those pieces (the reference style:
OpenVSP, NASA Langley's open vehicle sketcher).

This horizon **is G3, declared — not redesigned.** The freeform
rung of the accepted ladder (loft, guided sweep, G1 blend, the
NURBS layer with its daughter ADR) is the vehicle-geometry
engine. What this horizon adds to the picture is the *user
shape* of that rung: the sketcher (ADR-0030) is the section
editor; the recipe DAG (ADR-0027) is the parametric vehicle
model — a wing is a chain of sketches, lofts, and blends whose
parameters propagate the way the parametric hole's radius
already does. The dependencies run through existing ladder
contracts: the 2D typed entities, the persistent naming, the
continuity declarations of the G3 ADR when it is reviewed. No
new scope is created here; the vehicle framing is recorded so
G3's ADR review can check its decisions against a real consumer.

### H3 — Aerodynamic analysis (the XFLR5 role)

Lightweight aerodynamic analysis by vortex and panel methods —
VLM and lifting-line consumers of *coherent surface meshes*
(the reference style: XFLR5).

What this horizon asks of the kernel: **surface mesh extraction
from the exact B-Rep that is watertight by construction,
orientation-consistent, and quality-declared** — the analysis
tessellation of H1, consumed whole by an analysis-side module
that stays out of the kernel (the vortex math, the wake models,
the solver conventions). The dependency is strictly one-way:
analysis reads kernel contracts; the kernel learns nothing from
the solver. This is the discipline that keeps a CAD kernel from
becoming a CAE monolith by accretion.

## How the horizons consume the ladder

| Horizon | Consumes | Ladder position |
|---|---|---|
| H1 preprocessor | Layer-1 invariants, face queries, tolerance layers, `ShapeId` naming, analysis tessellation | G1 contracts (ADR-0032); tessellation quality = requirement recorded against the G-rungs |
| H2 vehicle geometry | sketcher, recipe DAG, loft/sweep/blend, NURBS layer, continuity declarations | **is G3** — declared, not redesigned |
| H3 aerodynamics | watertight orientation-consistent surface extraction from exact B-Rep | G1 exact faces → H1's extraction, consumed analysis-side |

The one-line summary of the whole vision: **the kernel's job is
to be the exact, interrogable, honestly-toleranced truth that
meshing and analysis can trust — and the parametric spine that
makes a vehicle model rebuildable.** Everything past that line
is platform surface, built on the contracts, never inside them.

## References

- ADR-0031 (the ladder, accepted — road (a)), ADR-0032 (G1 draft:
  the Layer-1 invariant with the inner-loop term, exact mass
  properties, true trimming), ADR-0030 (the sketcher — H2's
  section editor), ADR-0027 (recipes — H2's vehicle model),
  ADR-0005 (naming — H1's named selections), ADR-0004 /
  ADR-0031 D3 (tolerance layers), ADR-0022 (query operators).
- Style references, by name only, as orientation anchors:
  ANSA (BETA CAE Systems) for the preprocessor role; OpenVSP
  (NASA Langley) for parametric vehicle geometry; XFLR5 for
  vortex/panel-method analysis.
