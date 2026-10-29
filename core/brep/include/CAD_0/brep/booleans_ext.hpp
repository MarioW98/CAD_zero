// core/brep/include/CAD_0/brep/booleans_ext.hpp
//
// THE EXTENDED BOOLEANS — ADR-0033 D5.2 (G2 unit 2), over the accepted
// boundary: **D2 = option (i), the table is the wall.** The unit 1
// table's entries ANSWER; this unit makes the operations CONSUME them:
// union / intersect / subtract run exact exactly where every curved
// pair of the operands sits in the typed table — the class check IS
// part of the operation's ENTRY CONTRACT, never a posteriori.
//
// THE OPERATIONS (the ladder's four):
//   * sphere×sphere — the classical union, the lens (intersect), the
//     bitten sphere (subtract);
//   * sphere±prism — the extrude's circle profile piercing the sphere
//     (the bead / the rivet / the core) and the box-profile truncation
//     family (the far cap, the near cap, the dome union, the slab);
//   * prism×prism with parallel axes — THE TWO-HOLE BODY: the second
//     cut on a pierced target (exact_subtract's extended class);
//   * the frame budget APPLIED: a trimmed revolution survivor carries
//     ONE axis family (unit 1); the incompatible family a later
//     operation asks for finds the wall, the remedy in the message.
//
// THE HONESTY CHAIN (worst-of-operands): exact ∧ exact = exact — the
// new entities carry combined_provenance; an APPROXIMATED operand is
// refused with the remedy named (the unit-1 gate's own words — the
// exact table consults exact faces, the bridge's faceted world
// stands beside, never laundered). No boolean output ever presents
// approximated geometry as exact.
//
// THE DERIVATIONS this unit pins (the tests quote them verbatim; the
// D7.5 analog — derived BEFORE the goldens, the full tables in
// tests/unit/brep/test_g2_unit2_booleans_ext.cpp's header):
//
//   sphere×sphere (D in the open band, the radical plane at h =
//   (D²+R₁²−R₂²)/2D, ρ = √(R₁²−h²), cap(R,h) = πh²(3R−h)/3):
//     union    4V/4E/2F/0H χ=2 — the two far caps glued at the rim;
//     lens     4V/4E/2F/0H χ=2 — the two near caps;
//     bitten   2V/2E/2F/0H χ=2 — the far cap + the radical disk.
//     Volumes: V₁+V₂−cap(R₁,R₁−h)−cap(R₂,R₂−D+h) / the two near caps
//     / cap(R₁,R₁+h) — the CLASSICAL FORMULAS ARE THE TESTS.
//
//   sphere × circle-prism (coaxial, r < R, the tool spanning the
//   sphere; w = √(R²−r²), the core = πr²·2w + 2·cap(R, R−w)):
//     bead (−)   4V/6E/2F/0H χ=0 genus 1 — the equatorial zone + the
//                wall band between the latitude circles;
//     rivet (∪)  8V/11E/5F/0H χ=2 — the zone + two wall bands + two
//                end disks;
//     core (∩)   6V/7E/3F/0H χ=2 — the two polar caps + the middle
//                wall band.
//
//   sphere × box-profile (ONE wall family cutting, the caps clear —
//   the truncation semantics; d₀ the extreme cut offset):
//     far cap (−)  2V/2E/2F χ=2, volume cap(R, R+d₀);
//     near cap (∩) 2V/2E/2F χ=2, volume cap(R, R−d₀);
//     dome (∪)     11V/15E/7F/1H χ=2 — the box's faces + the far
//                  cap, the cutting face GAINING the circle as an
//                  inner loop (the G1 pierced-face pattern);
//     slab (−)     6V/6E/4F χ=2 — the straddling box keeps two far
//                  caps + two cut disks, volume 2·cap(R, R−a).
//
//   prism×prism (the two-hole guide case): each circle cut appends
//   +4V (the seam split S/A × the two pierced planes), +5E (2+2 rim
//   arcs + 1 seam vertical), +1F (the wall), +2H (one inner loop per
//   pierced face): the second cut lands 16/22/8/4, χ = −2 = 2(1−2),
//   genus 2, volume abc − 2πr²c.
//
// THE CONVENTION THIS UNIT DISCOVERS (the XOR derivation, pinned by
// the tests as a regression guard): the sphere's re-frame azimuth for
// the cylinder pair is u_sign·(the wall's u=0 meridian) — the wall's
// increasing-u direction at the seam. WHY IT IS FORCED: both faces
// read their regions material-on-left, so their walks on a SHARED
// rim must oppose; the wall's u and the sphere's u run the same 3D
// way exactly when the azimuths coincide, so the seam meridian must
// sit on the OPPOSITE side — the negated azimuth. The sphere×sphere
// pair needs no such care: the two caps' poles oppose along the
// center line, so a SHARED azimuth (the radical plane's own u axis)
// already makes their u-directions oppose.
//
// THE DEGENERATES, each with its pinned disposition (the D7.4 analog;
// the messages carry the family NAME, the class and the remedies —
// lowercase-searchable, the u1 discipline):
//   * the CO-RADIAL dedup (equal spheres): union/intersect keep the
//     operand verbatim (one surface, the other absorbed); subtract
//     refuses (the empty — the swallow disposition);
//   * the TANGENCIES: external — union/intersect refuse (the
//     point-contact non-manifold), subtract returns the exact copy
//     (the touch removes nothing, the G1 tie-break); internal —
//     union/intersect read the nested disposition, subtract refuses
//     (the cavity pinches at the contact point);
//   * NESTED / SEPARATE: union the container / refusal (the
//     disconnected union); intersect the contained / refusal (the
//     empty); subtract the operand (the miss);
//   * the bead's: the BLIND tool (the G1 disposition), r ≥ R (the
//     empty), r = R (the tangent circle), NON-COAXIAL (the Viviani
//     quartic — the wall);
//   * the two-hole's: the circles TANGENT or CROSSING on the pierced
//     face → NON-ANNULAR → refused; the UNEQUAL-radii crossing NAMES
//     the offset plane x = (d²+r₁²−r₂²)/(2d) and its two typed lines
//     — admitted by the criterion, awaiting the consuming unit, never
//     "untypable"; NESTED circles → the miss (the exact copy);
//   * the TWO-FAMILY wall cut (the box whose x and y walls both cut):
//     the frame budget's canonical example, refused at the gate;
//   * the budgeted survivor: the incompatible family finds the wall;
//     the compatible second cut refuses naming the multi-cut cascade
//     (the remedy real: re-cut the original operand).
//
// DESIGN (the house rules, applied):
//   * the class check enumerates the operand FACE PAIRS and consults
//     the table's dispositions — the entry contract, before any
//     geometry is built (the pipeline's G1-u4 generalization: the
//     intersection consumes the table, the classification is
//     centroid-then-predicate with the declared Layer-3 bands, the
//     assembly follows the proven pattern — canonical identity BEFORE
//     the keying, order-independent dedup, link_radial_edges, hard
//     postconditions with the H term);
//   * FREE FUNCTIONS, one .cpp beside the ssi table (the unit-1
//     precedent); exact_subtract keeps its subtract.hpp declaration —
//     the sphere-target branch dispatches here through
//     exact_subtract_sphere_target;
//   * pure functions of the operands (ADR-0014): same inputs, same
//     bytes — the hex-digest pin; union/intersect normalize the
//     operand order by the canonical key (center, then radius) so
//     the pair assembles ONE body whatever the call order;
//   * ONE tolerance parameter, the operation's declared Layer-3
//     decision value (the exact_subtract discipline): the linear
//     bands (tangency, coaxiality, the containment margins) and the
//     cosine band (the family parallelism) read the same value, each
//     use documented at its site;
//   * NO boolean moves that is not in the table (unit 3's fillet is
//     untouched); the format does not move; the display does not
//     change.
#pragma once

#include "CAD_0/brep/topology.hpp"
#include "CAD_0/brep/analytic.hpp"

#include <memory>

namespace CAD_0::brep {

// exact_union — A ∪ (B + tool_offset) on the extended class:
//   * sphere×sphere (the classical union, the overlapping open band);
//   * sphere×prism (the rivet for the circle profile coaxial through;
//     the dome for the box-profile truncation).
// In-class inputs materialize the exact result; every out-of-class
// input refuses with an actionable std::invalid_argument (the pair,
// the class, the remedies). Prism×prism unions are Phase D's landed
// opinion (bool_union) — pointed at, never duplicated.
std::unique_ptr<BRepBody> exact_union(const BRepBody& A, const BRepBody& B,
                                      math::Vec3f tool_offset, float tol);

// exact_intersect — A ∩ (B + tool_offset) on the same class: the
// lens (sphere×sphere), the core (the coaxial circle prism), the
// near cap (the box truncation). The same entry contract, the same
// refusal culture.
std::unique_ptr<BRepBody> exact_intersect(const BRepBody& A,
                                          const BRepBody& B,
                                          math::Vec3f tool_offset,
                                          float tol);

// exact_subtract_sphere_target — the sphere-target branch of
// exact_subtract (declared in subtract.hpp, dispatched there for the
// single-sphere-body targets, implemented here beside the kit it
// shares with the union/intersect paths): the bitten sphere, the
// bead, the truncation family, the slab — and the budget refusals.
std::unique_ptr<BRepBody> exact_subtract_sphere_target(
    const BRepBody& A, const BRepBody& B, math::Vec3f tool_offset,
    float tol);

} // namespace CAD_0::brep
