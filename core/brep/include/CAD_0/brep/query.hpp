// core/brep/include/CAD_0/brep/query.hpp
//
// The face query — ADR-0032 D8's "the answer is a query, not an
// investigation" made concrete for faces (G1 unit 2, the H1 contract
// taking shape): a preprocessor consuming CAD_0 bodies asks "cosa è
// questa faccia" and receives plain values — the surface family, the
// loop counts (outer + inner), and for every co-edge the p-curve's
// type and readable parameters, alongside the Layer-2 provenance
// record.
//
// The BODY query (G1 unit 6, the same D8 contract one rung up): the
// Debug report asked for the whole body as plain values — the counts,
// χ with its inner-loop term, the genus, and the provenance tallies
// per subshape kind — so "questo bordo è esatto?" answers at the
// body level too, without walking faces. `query_body` is a THIN
// aggregator of the existing authorities (euler_characteristic,
// inner_loop_count, is_closed_orientable, loops_are_closed,
// genus_from_chi, the per-subshape provenance fields): no number is
// computed here that is not already a pinned function's own — the
// report that consumes it stays the single opinion.
//
// Design notes, per the house rules:
//
//   * FREE FUNCTIONS, read-only — the ADR-0022 §2 query-operator
//     pattern (no methods on the entities, no mutation, plain
//     values back). The structs below are value records, not
//     handles: callers cannot reach back into the body through
//     them.
//   * The family string is SurfaceGeometry::type_name() verbatim
//     ("plane" | "cylinder" | "sphere" | "cone" | "torus") — one
//     naming authority, no second enum to drift.
//   * PCurveInfo::Kind::None is the FACETED world's honest answer
//     (the bridge's plane-per-triangle faces carry no p-curves; the
//     provenance fields distinguish the worlds, never a guess).
//   * The p-curve payload mirrors the sketch kernel's typed entities
//     VERBATIM (Line2's a/b/interior_left, Arc2's
//     center/radius/hole/start_angle/sweep) — the D2.1 discipline:
//     the same numbers the sketcher would read, no translation.
//   * A face id that does not exist is LOUD: std::invalid_argument
//     naming the valid range and the remedy, the ProvenanceInfo
//     construction-time culture. A silent empty record would be a
//     guess, and a missing face is a caller bug the guess would hide.
//
// Consumers: the Python bindings (CAD_0.brep.face_query — the
// preprocessor's entry point), the unit-2 tests (the provenance
// enumeration and the golden reader), and — from unit 4 on — the
// exact booleans' fragment bookkeeping.
#pragma once

#include "CAD_0/brep/topology.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace CAD_0::brep {

// One co-edge's p-curve as plain values. `edge`/`reversed` keep the
// 3D reference alongside, so a consumer can cross-navigate from the
// domain picture to the topology (and back through the body).
struct PCurveInfo {
    enum class Kind : std::uint8_t {
        None,    // no p-curve — the faceted world
        Line,    // Line2  (SketchSegment): a → b, region side
        Circle,  // Circle2 (SketchCircle): center, radius, disc|complement
        Arc      // Arc2   (SketchArc): center, radius, span, region side
    };

    Kind kind{Kind::None};

    // The 3D reference (the co-edge itself).
    EdgeId edge{0};
    bool reversed{false};

    // Line payload: domain endpoints + region side.
    math::Vec2f a{};
    math::Vec2f b{};
    bool interior_left{false};

    // Circle/Arc payload: center, radius, region side; the arc adds
    // the CCW angular span (radians; sweep in (0, 2π) per the sketch
    // vocabulary's construction rules).
    math::Vec2f center{};
    float radius{0.0f};
    bool hole{false};
    float start_angle{0.0f};
    float sweep{0.0f};
};

// One loop of the face, outer or inner (ADR-0021: "subsequent loops
// are holes").
struct LoopInfo {
    bool is_outer{true};
    std::vector<PCurveInfo> coedges{};
};

// One face's answer, in full.
struct FaceInfo {
    bool has_surface{false};
    std::string family{};  // type_name(): "plane"/"cylinder"/sphere"/...

    std::size_t loop_count{0};
    std::size_t outer_loop_count{0};
    std::size_t inner_loop_count{0};
    std::vector<LoopInfo> loops{};

    // Layer-2 (D8): exact/0 for the analytic world, the declared
    // bound for the faceted one — never guessed, never laundered.
    bool exact{true};
    double linear_tolerance{0.0};
};

// query_face — the face with this id, as plain values. Faces are
// matched by Face::id() (the factories number them by shell order;
// the booleans number their fragments). Throws std::invalid_argument
// naming the valid range and the remedy when no face carries the id.
FaceInfo query_face(const BRepBody& body, FaceId face_id);

// query_all_faces — every face of the body's shell, in shell order.
// An empty body returns an empty vector (a legitimate answer, not an
// error: an empty shell has no faces to describe).
std::vector<FaceInfo> query_all_faces(const BRepBody& body);

// ---------------------------------------------------------------------------
// The body query (G1 unit 6, ADR-0032 D8 at the body level)
// ---------------------------------------------------------------------------

// One subshape kind's provenance tally (D2.4 + D8): how many exact,
// how many approximated, and the maximum declared linear tolerance
// over the kind's subshapes. max_linear_tolerance is 0 for an
// all-exact kind — the pairing IS the contract (exact | 0).
struct ProvenanceTally {
    std::size_t exact{0};
    std::size_t approximated{0};
    double max_linear_tolerance{0.0};
};

// The whole body as plain values — what the Debug report renders and
// the guide case's table reads as its "reported" column. Every field
// is an existing authority's own answer, verbatim:
//
//   * vertex_count / edge_count / face_count — the body's own counts;
//   * inner_loops — inner_loop_count (H, the χ term's own query);
//   * chi — euler_characteristic, χ = V − E + F − H;
//   * orientable / loops_closed — is_closed_orientable /
//     loops_are_closed, the structural contract as reported;
//   * genus — genus_from_chi under the ADR-0027 D6 rule the bridge
//     report already applies: derived only when the body is
//     orientable AND the χ gates hold (χ ≤ 2 and even — the rule
//     genus_from_chi itself encodes). When genus_derived is false the
//     genus must not be read: a multi-shell body (χ > 2) is "n/d",
//     and an odd χ is an assembly defect reported, never smoothed
//     into a half-integer;
//   * vertices / edges / faces — the per-kind provenance tallies
//     (D8's "n exact / m approximated, with the max tolerance").
struct BodySummary {
    std::size_t vertex_count{0};
    std::size_t edge_count{0};
    std::size_t face_count{0};
    std::int64_t inner_loops{0};  // H = Σ_f max(0, loops(f) − 1)
    std::int64_t chi{0};          // V − E + F − H
    bool orientable{false};       // is_closed_orientable
    bool loops_closed{false};     // loops_are_closed
    bool genus_derived{false};    // valid iff true (the D6 rule)
    std::int64_t genus{0};
    ProvenanceTally vertices{};
    ProvenanceTally edges{};
    ProvenanceTally faces{};
};

// query_body — the body's whole answer, as plain values. Throws
// std::invalid_argument (diagnosis + remedy, the P2 pattern) for the
// EMPTY body: with no vertices and no faces there is no shell to
// summarize, and the counts would silently paint χ = 0 / genus 1 on
// nothing — mass_properties refuses the same body for the same
// reason. A non-empty body never throws (every field is a noexcept
// authority's answer).
BodySummary query_body(const BRepBody& body);

} // namespace CAD_0::brep
