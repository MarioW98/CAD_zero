// core/brep/include/CAD_0/brep/ssi.hpp
//
// The analytic SSI table — ADR-0033 D1.1/D5.1 (G2 unit 1), the first
// unit of the accepted ladder, under the accepted boundary: **D2 =
// option (i), the table is the wall.**
//
// WHAT THIS HEADER IS. The typable surface-surface intersection
// entries the authored class can reach, ONE DERIVATION PER ENTRY,
// consultable as a KERNEL surface and through the bindings BEFORE any
// boolean consumes them (the G1 unit-2 face-query precedent — the
// order's own visible surface: from Python, on author bodies, the
// intersections answer: curves, p-curves in BOTH domains,
// provenance). No boolean moves in this unit (unit 2's); the format
// does not move; the display does not change.
//
// THE ADMISSION CRITERION (the G1 lesson made a rule): an entry is
// typable iff the 3D curve is a conic in the curve vocabulary AND the
// p-curve is typed in the parameter domain of EACH owning face —
// {Line2, Circle2, Arc2} in the stored frame, or in a frame the
// operation may deterministically re-author (the re-frame rule,
// below). The both-domain criterion is the stricter one: the Steinmetz
// pair degenerates to ellipses in space and stays OUT (the sinusoid
// in the domains), and the oblique plane×cylinder is its G1 instance.
//
// THE ENTRIES (each a derivation, ADR-0033 D1.1's table):
//   * plane×sphere (d < R): the circle √(R²−d²) at the foot of the
//     perpendicular; Circle2 on the plane's own domain; the latitude
//     Line2 on the sphere's domain re-framed to the plane's normal.
//   * sphere×sphere (non-nested): the circle in the RADICAL PLANE
//     (h = (D²+R₁²−R₂²)/2D along the center line, ρ = √(R₁²−h²));
//     each sphere re-framed to the center line, azimuth = the derived
//     radical plane's own u axis (the PlaneSurface rule — one frame
//     derivation for every plane in the house).
//   * cylinder×sphere coaxial (r < R): the TWO circles at axial
//     offsets ±√(R²−r²), the cylinder's own radius; Line2 v=const on
//     the cylinder domain, latitudes on the sphere re-framed to the
//     axis (or typed in the stored frame when the pole already
//     serves — the no-reframe path).
//   * cylinder×cylinder parallel, EQUAL radii, offset d < 2r: the
//     two lines in the mid-plane, at ±√(r²−d²/4) along the
//     right-handed vertical δ̂⊥×â; Line2 u=const on each domain,
//     spanning the face's OWN authored v-window (derived from its
//     loops — the window the boolean will trim within).
//
// THE RE-FRAME RULE (the heart of the robustness): an operation may
// re-frame a surviving revolution face — rebuild it on the SAME
// mathematical object with the axis aligned to the operation's
// canonical axis, the loops re-authored in the new domain, the seam
// re-wired in the ADR-0023 slit-rectangle pattern. THE FRAME IS
// DERIVED (it serves the geometry, never the reverse), and the
// re-frame is a PURE function of the recipe: bit-identical under
// regeneration (ADR-0014, the hex-digest pin in the tests).
// `reframe_sphere_face` is the standalone act for the SEAM-ONLY face
// (the factories' full sphere — the authored shape of this unit);
// the query DERIVES and DECLARES the frame in its answer instead
// (read-only, nothing mutates).
//
// THE FRAME BUDGET, derived and pinned: one frame = ONE AXIS FAMILY
// per face. The family is not stored beside the face — it is derived
// from the face's own record: a face whose loops carry typed cuts
// beyond the seam structure (distinct edges beyond the seam edge) has
// its frame budgeted by those cuts. A later operation arriving with
// an INCOMPATIBLE family (a canonical axis not parallel to the
// budgeted one) finds THE WALL: refused actionably with the pair, the
// budget and the remedies — option (i), coherent to the end. The
// budgeted face's family-compatible re-frame (parallel axis, cuts
// re-authored as latitudes of the same family) is the BOOLEAN unit's
// own step (unit 2 re-authors the cuts beside the frame); the
// standalone function says so in its refusal rather than guessing.
//
// THE WALL (the refusals are ANSWERS — recorded, never missing):
//   * Kind::OutOfTable carries the pair name, the disposition class
//     and the remedies (reorient the operands, accept the faceted
//     path, or await G3 — or the named owner: the subtract's own
//     landed entry for plane×cylinder, Phase D for plane×plane, the
//     fillet's unit for the toroidal rows, the register for the
//     cone's no-writer mathematics).
//   * The non-coaxial sphere×cylinder is the Viviani quartic; the
//     non-parallel cylinder pair is the irreducible quartic whose
//     Steinmetz member is typed in neither domain; the UNEQUAL-radii
//     parallel pair is refused at the entry's gate with the
//     acceptance's CORRECTED disposition on record: the transverse
//     case |r₁−r₂| < d < r₁+r₂ is TWO TYPED lines in the offset
//     plane x = (d²+r₁²−r₂²)/(2d) — admitted by the criterion, out
//     of the registered entry until the unit that consumes it; never
//     called "untypable", which it is not.
//   * The provenance gate: the exact table consults EXACT faces. An
//     approximated operand (the bridge's faceted world) is refused
//     with the remedy named — the honesty chain, never laundered.
//
// REGION-SIDE NEUTRALITY: the p-curves in the ANSWER carry neutral
// defaults (interior_left = true, hole = false) — the typed GEOMETRY.
// The region side is the consuming operation's authoring decision
// (the boolean's fragment classification, unit 2); the query answers
// curves and typability, not fragments.
//
// Design notes, per the house rules:
//   * FREE FUNCTIONS, read-only for the query (the ADR-0022 §2
//     pattern); the re-frame is the one mutating act, following the
//     translate_body precedent.
//   * Pure functions, deterministic (ADR-0014): same operands, same
//     bytes — the answer digest is the pin.
//   * ONE tolerance parameter, the operation's declared Layer-3
//     decision tolerance (the exact_subtract discipline): the linear
//     bands (tangency, coaxiality, the offset classes) and the cosine
//     band (axis parallelism) read the same declared value, each use
//     documented at its site.
//   * The face ids are the bodies' own numbering; a missing id is a
//     caller bug and throws std::invalid_argument (diagnosis +
//     remedy, the query_face culture). The out-of-table answer is NOT
//     an exception: it is a returned record — the capability ceiling
//     is explicit and queryable.
#pragma once

#include "CAD_0/brep/analytic.hpp"
#include "CAD_0/brep/curve.hpp"
#include "CAD_0/brep/topology.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace CAD_0::brep {

// One typed 3D curve of the answer, as plain values (the query
// discipline — no handles back into the body).
struct SsiCurve {
    enum class Kind : std::uint8_t {
        Circle,  // center + radius in the (u_ref, v_ref) plane
        Line     // point + direction (the infinite reference line)
    };

    Kind kind{Kind::Circle};

    // Circle payload: the center, the radius, and the DERIVED
    // in-plane frame — evaluate(t) = center + r·(cos t·u_ref +
    // sin t·v_ref), the CircleCurve vocabulary's own form. DOUBLE
    // throughout (H1, ADR-0034): the exact world's answers.
    math::Vec3d center{};
    double radius{0.0};
    math::Vec3d u_ref{};
    math::Vec3d v_ref{};

    // Line payload: a point on the line and the (unit) direction.
    math::Vec3d point{};
    math::Vec3d dir{};

    math::Vec3d evaluate(double t) const {
        if (kind == Kind::Circle)
            return center + u_ref * (radius * std::cos(t)) +
                   v_ref * (radius * std::sin(t));
        return point + dir * t;
    }
};

// One operand's domain answer: the typed p-curves (one per 3D curve,
// in traversal order) in the face's parameter domain — the STORED
// domain when it types them, else the RE-FRAMED domain the entry
// derived (declared here so the consuming operation can rebuild the
// face on it, unit 2's step).
struct SsiDomainAnswer {
    bool reframe_needed{false};
    // The derived frame, when reframe_needed: the new pole (v = 0)
    // and the new azimuth reference (u = 0) — the re-frame rule's
    // (n̂, e₀), e₁ = e₀×n̂ (the SphereSurface form). Double: the
    // declared frame is exact-world data (ADR-0034).
    math::Vec3d frame_pole{};
    math::Vec3d frame_azimuth{};
    // The typed p-curves, in the 3D curves' order. Neutral region
    // bits (see the header's REGION-SIDE NEUTRALITY note).
    std::vector<PCurve> pcurves{};
};

// The table's answer, in full.
struct SsiResult {
    enum class Kind : std::uint8_t {
        Empty,       // no intersection (the class named in the message)
        Point,       // tangency point (sphere families, plane×sphere d=R)
        Circle,      // one 3D circle (plane×sphere; sphere×sphere; cyl×sphere r=R)
        CirclePair,  // two circles (cylinder×sphere coaxial, r<R)
        LinePair,    // two lines (cylinder×cylinder equal r, d<2r)
        TangentLine, // the double line (cylinder×cylinder d=2r)
        Coincident,  // the co-radial degenerate (equal spheres; coaxial equal
                     // cylinders) — the dedup rule's trigger, reported
        OutOfTable   // THE WALL: the refusal record (message carries all)
    };

    Kind kind{Kind::OutOfTable};

    // Layer-2: the computed entries are closed forms on exact faces —
    // exact/0, the pairing IS the contract. The refusals carry the
    // default exact record (nothing was computed; the message is the
    // answer).
    ProvenanceInfo provenance{};

    // The operands' surface families, as stored (the refusal names
    // the pair from these).
    std::string family_a{};
    std::string family_b{};

    // The tangency point (Kind::Point).
    math::Vec3d point{};

    // The 3D curves (Circle/CirclePair/LinePair/TangentLine), in the
    // derivation's canonical order (documented per entry in ssi.cpp).
    std::vector<SsiCurve> curves{};

    // The two domains' answers, operand order.
    SsiDomainAnswer a{};
    SsiDomainAnswer b{};

    // The disposition's own words: the class for the degenerates
    // ("separate", "nested", "external tangency", "the co-radial
    // dedup rule's trigger"), the FULL actionable refusal for
    // OutOfTable (pair, class, remedies).
    std::string message{};
};

// ssi_query — consult the table with two faces of two bodies (the
// boolean's own consultation shape; the same body twice serves the
// intra-body pairs). `tol` is the operation's declared Layer-3
// decision tolerance: the linear bands (tangency, coaxiality, the
// offset classes, the frame-budget parallelism) and the cosine band
// (axis parallelism) read the same value, each use documented at its
// site in ssi.cpp. Pure function, deterministic (ADR-0014): same
// operands, same bytes.
//
// Throws std::invalid_argument for a face id either body does not
// carry (diagnosis + remedy, the query_face culture). Everything
// else — including every refusal — is a RETURNED record: the
// capability ceiling is explicit and queryable, a recorded answer,
// never a missing one.
SsiResult ssi_query(const BRepBody& body_a, FaceId face_a,
                    const BRepBody& body_b, FaceId face_b, double tol);

// The re-frame record: the derived frame the face was rebuilt on (or
// that the query declared).
struct ReframeRecord {
    math::Vec3d pole{};      // n̂: the new v = 0 axis
    math::Vec3d azimuth{};   // e₀: u = 0 is the meridian through +e₀
};

// reframe_sphere_face — the re-frame rule's standalone act: rebuild
// the SEAM-ONLY sphere face on the derived (pole, azimuth) frame —
// the same mathematical object (center, radius), the vertices moved
// to the new poles, the seam pair re-wired in the ADR-0023
// slit-rectangle form, the radial edges re-linked. A pure function of
// (the face's record, the frame): bit-identical at regeneration.
//
// Gates, each actionable:
//   * the face id must exist (the query_face culture);
//   * the surface must be a sphere — the unit's revolution class
//     (the cylinder's frame is fixed by its own axis; a placement is
//     translate_body's job, and the cone/torus have no writer);
//   * (pole, azimuth) must be perpendicular unit-ish directions (the
//     SphereSurface constructor's own validation);
//   * THE FRAME BUDGET: the face must be seam-only (its loops carry
//     no typed cuts — distinct edges beyond the seam). A budgeted
//     face REFUSES: the incompatible family is the wall (option (i));
//     the family-compatible re-frame of a cut face is the boolean
//     unit's own step (unit 2 re-authors the cuts beside the frame) —
//     the refusal says so, it does not guess.
ReframeRecord reframe_sphere_face(BRepBody& body, FaceId face,
                                  const math::Vec3d& pole,
                                  const math::Vec3d& azimuth, double tol);

} // namespace CAD_0::brep
