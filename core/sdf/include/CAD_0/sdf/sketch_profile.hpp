// core/sdf/include/CAD_0/sdf/sketch_profile.hpp
//
// The 2D profile field and its construction certificate — ADR-0030 D3,
// MVP unit 1 (second coherent group: the COMPOSITE).
//
// Composition: f_profile = max_i f_i — the INTERSECTION of the entity
// regions (the premise correction stated in the ADR's Status: the
// draft said min; four half-planes under min are the whole plane,
// under max they are the quadrilateral).
//
// The validation-table rows (ADR-0030 D3) live here, verbatim:
//
// | Node            | Field       | Exact regime             | Understated                    | L | Gradient                    | Bounds                        | Construction checks |
// |-----------------|-------------|--------------------------|--------------------------------|---|-----------------------------|-------------------------------|---------------------|
// | ProfileSDF      | max of entities | active boundary, interior near it | convex corners 1/sin(θ/2); notch shadows | 1 (proof below) | winning branch, normalized | intersection of entity bounds | chain closure; samples certify the expected side; pairwise intersections certify zero-set completeness; bounded result |
// | SketchEntitySet | —           | pairwise entity intersections, exact closed form: seg–seg (2×2 parametric solve), seg–circle (quadratic in t, roots in [0,1]), circle–circle (‖C1−C2‖ vs r1+r2 and |r1−r2|); arcs participate as their circle's angular span | — | — | — | — | O(n²) in the sketch's entities, cold path (construction/load, never per sample); crossings legal ONLY at declared shared joints; every other crossing, tangent included → std::invalid_argument naming both entities and the point |
//
// Lipschitz = 1, proved, not hoped (the ADR text): max (and min) of
// 1-Lipschitz fields is 1-Lipschitz —
//     |max(u,v)(p) − max(u,v)(q)| ≤ max(|u(p)−u(q)|, |v(p)−v(q)|) ≤ |p−q|
// — a two-line proof, unlike the twist episode (ADR-0029 D2).
//
// The value is NOT a distance field in general (lie #2, derived in the
// ADR): the composed field is a 1-Lipschitz conservative lower bound of
// the distance, EXACT on the active boundary spans, sign- and
// zero-set-exact everywhere. Outside, past a convex corner (interior
// angle θ) the gap is 1/sin(θ/2) (√2 at right angles); inside, near
// removed boundary, the depth is understated. It is a marching and
// evaluation field, not a measuring stick.
//
// Gradient: chain rule to the winning branch, the kernel's convention
// (lie #3): where one entity strictly dominates, ∇f is that entity's
// 2D gradient; at ties (the contour's joints) the winning branch's
// gradient is returned, normalized — the same max-seam semantics
// already declared for IntersectionSDF/SubtractionSDF/BoxSDF. At ties
// the FIRST entity in document order wins (replace only on strictly
// greater — the IntersectionSDF precedent).
//
// The failure regime (lie #5 / #5b) is closed AT CONSTRUCTION with TWO
// instruments, each covering exactly what the other cannot:
//
//   1. THE SAMPLES cover the EXPECTED SIDE. Every joint vertex and
//      every entity's interior checkpoint (the segment/arc midpoint; a
//      full circle samples a point of its own curve — the center,
//      strictly inside the disc, is excluded precisely because no side
//      assumption may be baked in) must evaluate ON the composed
//      zero-set within tolerance.
//   2. THE PAIRWISE INTERSECTION CHECK covers the COMPLETENESS of the
//      zero-set along the active spans. Lie #5b (the review's catch):
//      "joints + midpoints certify the contour" — necessary, not
//      sufficient; a TRANSVERSAL CROSSING between two entities landing
//      strictly between a checkpoint and the next deviates the
//      zero-set from the drawn contour while every sample passes
//      (the closed counterexample — a hole circle crossing an outer
//      segment mid-span — is pinned as an explicit regression in
//      test_sketch2d_profile.cpp). Every pair of entities is checked
//      exactly, in closed form; crossings are legal ONLY at a declared
//      shared joint of the chain; every other crossing, a tangent one
//      included, is std::invalid_argument naming BOTH entities and the
//      crossing point (the ADR-0029 D5 family — the loader inherits
//      the error path for free).
//
// Together, and only together, the two instruments are SUFFICIENT.
//
// Bounds honesty note (the ADR's own clause, plus the gap the ADR left
// and this header closes declaredly): "bounds_2D = the INTERSECTION of
// the entity bounds — the IntersectionSDF rule at 2D; an unbounded
// result is a construction-time error: 'profile does not close'".
// The intersection-of-region-boxes is ORIENTATION-BLIND: a diagonal
// half-plane's region box is the universe (it reaches every axis
// extreme), so a rotated contour of diagonal segments intersects to
// the universe — a perfectly valid closed profile would be refused as
// non-closing. The closure, implemented and pinned: when the region
// boxes fail to bound, the profile is certified bounded by the EXACT
// condition — an outer disc exists (R ⊆ that disc), or the half-plane
// outward normals positively span the plane (max angular gap < π −
// guard; a closed contour of segments always satisfies this, by the
// turning number) — and the bounds fall back to the CONTOUR HULL: the
// AABB of the entity curves, which covers R because a bounded region
// is contained in the convex hull of its boundary. A genuinely
// unbounded profile (no outer disc, non-spanning normals) is still
// refused with the ADR's message.
#pragma once

#include "CAD_0/sdf/sketch_entities.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

namespace CAD_0::sdf {

// Named tolerances of the construction certificate (cold path —
// construction and load, never per sample). Values are absolute on the
// O(1)–O(10) scale the sketcher authors; each is stated where used.
inline constexpr float kJointCoincidenceTol = 1e-5f; // consecutive endpoints coincide; a crossing counts as "at a joint"
inline constexpr float kSampleTol = 1e-4f;           // |f| at the certificate checkpoints
inline constexpr float kParallelAngleTol = 1e-6f;    // seg–seg parallel classification (≈ ADR-0004 angular default)
inline constexpr float kCollinearDistTol = 1e-5f;    // seg–seg collinear classification
inline constexpr float kSpanAngularGuard = 1e-6f;    // positive spanning: max normal gap < π − guard

inline constexpr float kTwoPi = 6.28318530717958647692f;

// One ordered, closed chain of typed entities (ADR-0030 D2): the
// entities are connected endpoint-to-endpoint; a single-circle chain
// is closed by itself. Closure is a MATERIAL-TIME precondition (the
// extrude's), so the sketch node may legally hold open chains — but
// nothing in this header does: ProfileSDF and SketchEntitySet only
// exist for closed chains.
struct SketchChain {
    std::vector<SketchEntity> entities;
};

// --- The definition-state predicates (ADR-0030 D2 revision 1, D5.5) ---
//
// Is a chain CLOSED? The sketch node's visible state (an open chain is
// a legal, flagged editing state), the extrude's closure precondition
// and the canvas's definition-state badge all ask THIS one function —
// the kernel and the UI share one opinion, by construction. The
// judgment is the certificate's own coincidence rule (kJointCoincidenceTol
// over consecutive endpoints, the closing joint included), so a chain
// this predicate calls closed is exactly a chain validate_chain_structure
// accepts, and one it calls open is refused at materialization with
// the gap named.
inline bool chain_is_closed(const SketchChain& c) noexcept {
    const auto& ents = c.entities;
    if (ents.empty()) return false; // nothing drawn: nothing closed
    if (ents.size() == 1)
        return ents[0].kind == SketchEntity::Kind::Circle; // closes on itself
    // A circle has no endpoints to connect: inside a multi-entity chain
    // it can never close (the certificate's structural refusal).
    for (const auto& e : ents)
        if (e.kind == SketchEntity::Kind::Circle) return false;
    for (std::size_t i = 0; i < ents.size(); ++i) {
        const SketchEntity& a = ents[i];
        const SketchEntity& b = ents[(i + 1) % ents.size()];
        const math::Vec2f a_end =
            a.kind == SketchEntity::Kind::Segment ? a.segment.b : arc_end_point(a.arc);
        const math::Vec2f b_start =
            b.kind == SketchEntity::Kind::Segment ? b.segment.a : arc_start_point(b.arc);
        if ((a_end - b_start).length() > kJointCoincidenceTol) return false;
    }
    return true;
}

// The sketch's definition state: every chain closed (an empty sketch
// is not closed — there is no contour to extrude).
inline bool all_chains_closed(const std::vector<SketchChain>& chains) noexcept {
    if (chains.empty()) return false;
    for (const auto& c : chains)
        if (!chain_is_closed(c)) return false;
    return true;
}

namespace sketch_detail {

inline std::string fmt_pt(const math::Vec2f& p) {
    return "(" + std::to_string(p.x) + ", " + std::to_string(p.y) + ")";
}

// Normalize an angle difference into [0, 2π).
inline float normalized_angle(float d) noexcept {
    float r = std::fmod(d, kTwoPi);
    if (r < 0.0f) r += kTwoPi;
    return r;
}

} // namespace sketch_detail

// --- The per-entity class checks, ONE shared opinion (ADR-0030 D5.5) ---
//
// D3's table rows for the SINGLE entity — endpoints finite and
// p0 != p1 (Halfplane2D), center finite and r > 0 (Disc2D), arc
// sweep finite in (0, 2π). The SketchEntitySet certificate runs
// these during its construction stages; this standalone entry
// exposes the SAME predicates to the UI's gesture/text validation
// (D5.5: the canvas calls the kernel's checks, it keeps no second
// copy of the rules). Throws std::invalid_argument with the
// kernel's own message, naming the refused value.

// The sweep predicate, parameterized by the message's context: the
// certificate passes its own prefix and entity_name (byte-identical
// messages to the pre-refactor ones), the standalone entry its own.
inline void validate_arc_sweep(const SketchArc& arc,
                                const std::string& prefix,
                                const std::string& name) {
    const float sweep = arc.sweep;
    const float start = arc.start_angle;
    if (!std::isfinite(start) || !std::isfinite(sweep) ||
        sweep <= 0.0f || sweep >= kTwoPi)
        throw std::invalid_argument(
            prefix + ": arc sweep must be finite and in "
            "(0, 2π), got sweep=" + std::to_string(sweep) +
            " start=" + std::to_string(start) + " on " + name +
            " — a degenerate sweep is an empty or full-circle "
            "arc masquerading as a span (a full circle is the "
            "Circle kind, an empty one is no entity at all). "
            "Remedy: draw the arc with a non-degenerate sweep, "
            "or use a circle entity.");
}

// The standalone per-entity validation: segment/circle/arc class
// checks by CONSTRUCTING the kernel's own fields (the constructors
// ARE the checks) plus the shared sweep predicate for arcs. A
// throw names the refused value; a return means the entity is in
// class (the certificate may still refuse the composed sketch on
// structural, sampling or pairwise grounds — a different class).
inline void validate_sketch_entity(const SketchEntity& e) {
    switch (e.kind) {
    case SketchEntity::Kind::Segment:
        (void)Halfplane2D(e.segment.a, e.segment.b, e.segment.interior_left);
        break;
    case SketchEntity::Kind::Circle:
        (void)Disc2D(e.circle.center, e.circle.radius, e.circle.hole);
        break;
    case SketchEntity::Kind::Arc: {
        validate_arc_sweep(
            e.arc, "sketch entity",
            "arc center=" + sketch_detail::fmt_pt(e.arc.center) +
                " r=" + std::to_string(e.arc.radius));
        (void)Disc2D(e.arc.center, e.arc.radius, e.arc.hole);
        break;
    }
    }
}

// --- SketchEntitySet ----------------------------------------------------
//
// The structural check + the complete construction certificate (the
// table's SketchEntitySet row): NOT a field node — it owns the recipe
// entities, validates the chain structure, and runs the two
// instruments (samples + pairwise). ProfileSDF builds on it; the UI
// (ADR-0030 D5.5) will call the SAME predicates, so the canvas and the
// kernel share one opinion.
class SketchEntitySet {
public:
    explicit SketchEntitySet(std::vector<SketchChain> chains)
        : chains_(std::move(chains)) {
        validate_entity_params();
        build_fields();
        validate_chain_structure();
        certify_samples();
        certify_pairwise();
    }

    // The composed field: max over the entity fields, winning branch
    // (first entity at ties — the IntersectionSDF convention),
    // normalized defensively. Used by the certificate itself and by
    // ProfileSDF; exposed because the UI's refusal path (D5.5) shows
    // the same numbers the kernel judged.
    SDFSample2D sample(const math::Vec2f& p) const noexcept {
        SDFSample2D best{};
        bool first = true;
        for (const auto& f : fields_) {
            const SDFSample2D s = std::visit(
                [p](const auto& e) { return e.sample(p); }, f);
            if (first || s.value > best.value) {
                best = s;
                first = false;
            }
        }
        const float gl = best.gradient.length();
        if (gl > 1e-9f) best.gradient = best.gradient * (1.0f / gl);
        return best;
    }

    std::string describe() const {
        std::string s = "sketch-entity-set(";
        s += std::to_string(flat_count());
        s += " entities in ";
        s += std::to_string(chains_.size());
        s += " chains)";
        return s;
    }

    // --- diagnostics accessors (tests, UI messages) ---
    std::size_t flat_count() const noexcept { return flat_.size(); }
    const std::vector<SketchChain>& chains() const noexcept { return chains_; }

    // The parsed field entities (parallel to the flat entity order).
    // ProfileSDF's bounds computation consumes them; tests consume them
    // for expectations.
    const std::vector<std::variant<Halfplane2D, Disc2D>>& fields() const noexcept {
        return fields_;
    }

    // A stable, human-readable name for an entity: the exact string the
    // certificate's errors use, so a refusal names what the recipe
    // addresses (chain, index, parameters).
    std::string entity_name(std::size_t chain, std::size_t index) const {
        const SketchEntity& e = chains_.at(chain).entities.at(index);
        switch (e.kind) {
        case SketchEntity::Kind::Segment:
            return "segment[chain " + std::to_string(chain) + " #" +
                   std::to_string(index) + "] a=" + sketch_detail::fmt_pt(e.segment.a) +
                   " b=" + sketch_detail::fmt_pt(e.segment.b);
        case SketchEntity::Kind::Circle:
            return "circle[chain " + std::to_string(chain) + " #" +
                   std::to_string(index) + "] center=" + sketch_detail::fmt_pt(e.circle.center) +
                   " r=" + std::to_string(e.circle.radius) +
                   (e.circle.hole ? " hole" : " outer");
        case SketchEntity::Kind::Arc:
            return "arc[chain " + std::to_string(chain) + " #" +
                   std::to_string(index) + "] center=" + sketch_detail::fmt_pt(e.arc.center) +
                   " r=" + std::to_string(e.arc.radius) +
                   " start=" + std::to_string(e.arc.start_angle) +
                   " sweep=" + std::to_string(e.arc.sweep) +
                   (e.arc.hole ? " hole" : " outer");
        }
        return "<unknown>";
    }

private:
    // --- flat addressing ------------------------------------------------
    struct FlatRef {
        std::size_t chain, index;
    };
    struct Joint {
        math::Vec2f point;
        std::size_t e0, e1; // flat indices of the two adjacent entities
    };

    std::size_t flat_of(std::size_t chain, std::size_t index) const noexcept {
        for (std::size_t f = 0; f < flat_.size(); ++f)
            if (flat_[f].chain == chain && flat_[f].index == index) return f;
        return flat_.size(); // unreachable: flat_ covers every entity
    }

    const SketchEntity& entity(std::size_t flat) const {
        return chains_.at(flat_[flat].chain).entities.at(flat_[flat].index);
    }

    // Entity endpoints (the connectivity material). Circles are closed
    // loops and never reach these calls (structure check).
    math::Vec2f start_point(std::size_t flat) const {
        const SketchEntity& e = entity(flat);
        if (e.kind == SketchEntity::Kind::Segment) return e.segment.a;
        return arc_start_point(e.arc);
    }
    math::Vec2f end_point(std::size_t flat) const {
        const SketchEntity& e = entity(flat);
        if (e.kind == SketchEntity::Kind::Segment) return e.segment.b;
        return arc_end_point(e.arc);
    }

    // --- construction stages ---------------------------------------------

    void validate_entity_params() {
        for (std::size_t c = 0; c < chains_.size(); ++c) {
            for (std::size_t i = 0; i < chains_[c].entities.size(); ++i) {
                const SketchEntity& e = chains_[c].entities[i];
                if (e.kind != SketchEntity::Kind::Arc) continue;
                // The shared predicate (D5.5): the certificate and the
                // UI's standalone entry run the SAME check, with the
                // certificate's naming context — byte-identical
                // messages to the pre-refactor form.
                validate_arc_sweep(e.arc, "SketchEntitySet",
                                   entity_name(c, i));
            }
        }
    }

    void build_fields() {
        for (std::size_t c = 0; c < chains_.size(); ++c) {
            for (std::size_t i = 0; i < chains_[c].entities.size(); ++i) {
                const SketchEntity& e = chains_[c].entities[i];
                switch (e.kind) {
                case SketchEntity::Kind::Segment:
                    fields_.emplace_back(Halfplane2D(e.segment.a, e.segment.b,
                                                     e.segment.interior_left));
                    break;
                case SketchEntity::Kind::Circle:
                    fields_.emplace_back(Disc2D(e.circle.center, e.circle.radius,
                                                e.circle.hole));
                    break;
                case SketchEntity::Kind::Arc:
                    fields_.emplace_back(Disc2D(e.arc.center, e.arc.radius,
                                                e.arc.hole));
                    break;
                }
                flat_.push_back(FlatRef{c, i});
            }
        }
    }

    void validate_chain_structure() {
        for (std::size_t c = 0; c < chains_.size(); ++c) {
            const auto& ents = chains_[c].entities;
            if (ents.empty())
                throw std::invalid_argument(
                    "SketchEntitySet: chains must be non-empty, got chain " +
                    std::to_string(c) + " with 0 entities — an empty chain has "
                    "no contour to certify. Remedy: draw at least one entity.");
            if (ents.size() == 1) {
                if (ents[0].kind != SketchEntity::Kind::Circle)
                    throw std::invalid_argument(
                        "SketchEntitySet: a single-entity chain must be a "
                        "circle, got " + entity_name(c, 0) +
                        " — only a circle closes on itself; a lone segment "
                        "or arc leaves the chain open. Remedy: close the "
                        "chain with more entities, or use a circle.");
                continue; // closed by itself; no joints
            }
            // Multi-entity chains: every entity must have endpoints to
            // connect (a circle cannot participate).
            for (std::size_t i = 0; i < ents.size(); ++i)
                if (ents[i].kind == SketchEntity::Kind::Circle)
                    throw std::invalid_argument(
                        "SketchEntitySet: a circle cannot participate in a "
                        "multi-entity chain, got " + entity_name(c, i) +
                        " — a circle has no endpoints to connect. Remedy: "
                        "give the chain its own closed contour, or place the "
                        "circle as its own (hole) chain.");
            // Consecutive coincidence + closure; joints recorded.
            const std::size_t n = ents.size();
            for (std::size_t i = 0; i < n; ++i) {
                const std::size_t j = (i + 1) % n;
                const std::size_t fi = flat_of(c, i);
                const std::size_t fj = flat_of(c, j);
                const math::Vec2f pi = end_point(fi);
                const math::Vec2f pj = start_point(fj);
                if ((pi - pj).length() > kJointCoincidenceTol)
                    throw std::invalid_argument(
                        "SketchEntitySet: consecutive entities must share "
                        "their joint, got " + entity_name(c, i) + " ending at " +
                        sketch_detail::fmt_pt(pi) + " and " + entity_name(c, j) +
                        " starting at " + sketch_detail::fmt_pt(pj) +
                        " — a gap (or a wrong connection order) leaves the "
                        "chain open, and an open chain has no interior to "
                        "certify. Remedy: connect the endpoints (the "
                        "coincidence suggestion, D5.4) or edit the parameters "
                        "until they meet.");
                joints_.push_back(Joint{pi, fi, fj});
            }
        }
    }

    // Certificate instrument 1: the samples (the EXPECTED SIDE).
    void certify_samples() {
        // Joints: on the composed zero-set.
        for (const Joint& j : joints_) {
            const float f = sample(j.point).value;
            if (std::abs(f) > kSampleTol)
                throw std::invalid_argument(
                    "SketchEntitySet: the composed profile must vanish at "
                    "the joint of " + flat_name(j.e0) + " and " +
                    flat_name(j.e1) + " at " + sketch_detail::fmt_pt(j.point) +
                    ", got f=" + std::to_string(f) +
                    " — the contour lies about being the boundary at its own "
                    "checkpoint (some entity's region excludes a point the "
                    "drawing places on the contour). Remedy: check the "
                    "orientation bits and the radii of the entities named "
                    "above.");
        }
        // Entity interior checkpoints: the segment/arc midpoint; a full
        // circle samples a point of its own curve (the center is
        // excluded — no side assumption baked in).
        for (std::size_t f = 0; f < flat_.size(); ++f) {
            const SketchEntity& e = entity(f);
            math::Vec2f m{};
            switch (e.kind) {
            case SketchEntity::Kind::Segment:
                m = (e.segment.a + e.segment.b) * 0.5f;
                break;
            case SketchEntity::Kind::Arc: {
                const float mid_angle = e.arc.start_angle + e.arc.sweep * 0.5f;
                m = e.arc.center + math::Vec2f{e.arc.radius * std::cos(mid_angle),
                                               e.arc.radius * std::sin(mid_angle)};
                break;
            }
            case SketchEntity::Kind::Circle:
                m = e.circle.center + math::Vec2f{e.circle.radius, 0.0f};
                break;
            }
            const float v = sample(m).value;
            if (std::abs(v) > kSampleTol)
                throw std::invalid_argument(
                    "SketchEntitySet: the composed profile must vanish at " +
                    flat_name(f) + "'s checkpoint " + sketch_detail::fmt_pt(m) +
                    ", got f=" + std::to_string(v) +
                    " — the contour lies about being the boundary at its own "
                    "checkpoint (some entity's region excludes a point the "
                    "drawing places on the contour). Remedy: check the "
                    "orientation bits and the radii of the entities named "
                    "above.");
        }
    }

    // Certificate instrument 2: the pairwise intersection check (the
    // COMPLETENESS of the zero-set along the active spans). O(n²),
    // exact, closed form, cold path.
    void certify_pairwise() {
        for (std::size_t i = 0; i < flat_.size(); ++i) {
            for (std::size_t j = i + 1; j < flat_.size(); ++j) {
                const auto crossings = intersect_pair(i, j);
                for (const math::Vec2f& p : crossings) {
                    if (!crossing_is_legal(i, j, p))
                        throw std::invalid_argument(
                            "SketchEntitySet: entities must intersect only "
                            "at declared shared joints, got " + flat_name(i) +
                            " x " + flat_name(j) + " crossing at " +
                            sketch_detail::fmt_pt(p) +
                            " — a crossing off the declared joints (a "
                            "tangent one included) deviates the composed "
                            "zero-set from the drawn contour: the extrusion "
                            "would yield a valid solid that is NOT the drawn "
                            "one, silently (ADR-0030 D3, lie #5b). Remedy: "
                            "move the entities apart, or make the meeting a "
                            "real shared joint of the chain.");
                }
            }
        }
    }

    std::string flat_name(std::size_t flat) const {
        return entity_name(flat_[flat].chain, flat_[flat].index);
    }

    bool crossing_is_legal(std::size_t i, std::size_t j,
                           const math::Vec2f& p) const noexcept {
        for (const Joint& jt : joints_)
            if ((jt.e0 == i && jt.e1 == j) || (jt.e0 == j && jt.e1 == i))
                if ((jt.point - p).length() <= kJointCoincidenceTol)
                    return true;
        return false;
    }

    // --- exact curve intersection, span-filtered -------------------------

    // Does the point p (known to be on the arc's circle) fall inside the
    // arc's active span? Slack at the span ends keeps the JOINT contacts
    // counted (they are then judged by joint proximity).
    bool in_arc_span(const SketchArc& a, const math::Vec2f& p) const noexcept {
        const float rel = sketch_detail::normalized_angle(
            std::atan2(p.y - a.center.y, p.x - a.center.x) - a.start_angle);
        const float slack = kJointCoincidenceTol /
                                std::max(a.radius, kJointCoincidenceTol) +
                            1e-6f;
        return rel <= a.sweep + slack;
    }

    // Span filter for entity `f` (a no-op for segments and circles).
    bool in_span(std::size_t f, const math::Vec2f& p) const {
        const SketchEntity& e = entity(f);
        if (e.kind == SketchEntity::Kind::Arc)
            return in_arc_span(e.arc, p);
        return true;
    }

    std::vector<math::Vec2f> intersect_pair(std::size_t i, std::size_t j) const {
        const SketchEntity& a = entity(i);
        const SketchEntity& b = entity(j);
        const bool a_seg = a.kind == SketchEntity::Kind::Segment;
        const bool b_seg = b.kind == SketchEntity::Kind::Segment;
        const bool a_circ = a.kind == SketchEntity::Kind::Circle ||
                            a.kind == SketchEntity::Kind::Arc;
        const bool b_circ = b.kind == SketchEntity::Kind::Circle ||
                            b.kind == SketchEntity::Kind::Arc;

        std::vector<math::Vec2f> out;
        if (a_seg && b_seg) {
            intersect_segments(a.segment.a, a.segment.b, b.segment.a, b.segment.b,
                               [&](const math::Vec2f& p) {
                                   if (in_span(i, p) && in_span(j, p))
                                       out.push_back(p);
                               });
        } else if (a_seg && b_circ) {
            const SketchCircleLike cb = circle_of(b);
            intersect_segment_circle(a.segment.a, a.segment.b, cb.center, cb.radius,
                                     [&](const math::Vec2f& p) {
                                         if (in_span(i, p) && in_span(j, p))
                                             out.push_back(p);
                                     });
        } else if (a_circ && b_seg) {
            const SketchCircleLike ca = circle_of(a);
            intersect_segment_circle(b.segment.a, b.segment.b, ca.center, ca.radius,
                                     [&](const math::Vec2f& p) {
                                         if (in_span(i, p) && in_span(j, p))
                                             out.push_back(p);
                                     });
        } else if (a_circ && b_circ) {
            const SketchCircleLike ca = circle_of(a);
            const SketchCircleLike cb = circle_of(b);
            intersect_circles(ca, cb, i, j,
                              [&](const math::Vec2f& p) {
                                  if (in_span(i, p) && in_span(j, p))
                                      out.push_back(p);
                              });
        }
        return out;
    }

    struct SketchCircleLike {
        math::Vec2f center;
        float radius;
    };
    static SketchCircleLike circle_of(const SketchEntity& e) {
        if (e.kind == SketchEntity::Kind::Circle)
            return {e.circle.center, e.circle.radius};
        return {e.arc.center, e.arc.radius};
    }

    // seg × seg: the 2×2 parametric solve on unit directions. Parallel
    // non-collinear -> nothing; parallel collinear -> an overlap (or a
    // touching point) reported at its midpoint (a collinear overlap is
    // worse than a tangent contact: the same refusal covers both).
    static void intersect_segments(const math::Vec2f& a1, const math::Vec2f& b1,
                                   const math::Vec2f& a2, const math::Vec2f& b2,
                                   const auto& report) {
        const math::Vec2f d1 = b1 - a1, d2 = b2 - a2;
        const float len1 = d1.length(), len2 = d2.length();
        if (!(len1 > 0.0f) || !(len2 > 0.0f)) return; // validated upstream
        const math::Vec2f u1 = d1 * (1.0f / len1), u2 = d2 * (1.0f / len2);
        const float denom = u1.x * u2.y - u1.y * u2.x; // cross(u1, u2)
        if (std::abs(denom) > kParallelAngleTol) {
            const math::Vec2f w = a2 - a1;
            const float s = (w.x * u2.y - w.y * u2.x) / denom; // along u1
            const float t = (w.x * u1.y - w.y * u1.x) / denom; // along u2
            const float eps1 = kJointCoincidenceTol / len1;
            const float eps2 = kJointCoincidenceTol / len2;
            if (s >= -eps1 && s <= len1 + eps1 && t >= -eps2 && t <= len2 + eps2)
                report(a1 + u1 * s);
            return;
        }
        // Parallel: collinear?
        const math::Vec2f w = a2 - a1;
        if (std::abs(w.x * u1.y - w.y * u1.x) > kCollinearDistTol) return;
        // Collinear: project onto u1 and compare the intervals.
        const float t_a2 = w.dot(u1);
        const float t_b2 = (b2 - a1).dot(u1);
        const float lo2 = std::min(t_a2, t_b2), hi2 = std::max(t_a2, t_b2);
        const float lo = std::max(0.0f, lo2), hi = std::min(len1, hi2);
        if (hi < lo) return; // disjoint
        report(a1 + u1 * ((lo + hi) * 0.5f));
    }

    // seg × circle: substitute the parametric point into |P−C|² = r² —
    // a quadratic in t whose roots in [0, 1] are the crossings (a
    // negative discriminant: none; a double root: the tangent case,
    // reported like any crossing — tangent included).
    static void intersect_segment_circle(const math::Vec2f& a, const math::Vec2f& b,
                                         const math::Vec2f& c, float r,
                                         const auto& report) {
        const math::Vec2f d = b - a;
        const math::Vec2f ac = a - c;
        const float A = d.dot(d);
        if (!(A > 0.0f)) return;
        const float B = 2.0f * d.dot(ac);
        const float C = ac.dot(ac) - r * r;
        const float disc = B * B - 4.0f * A * C;
        if (disc < 0.0f) return;
        const float sq = std::sqrt(disc);
        const float len = std::sqrt(A);
        const float eps = kJointCoincidenceTol / len;
        for (const float t : {(-B - sq) / (2.0f * A), (-B + sq) / (2.0f * A)}) {
            if (t >= -eps && t <= 1.0f + eps)
                report(a + d * t);
        }
    }

    // circle × circle: ‖C1−C2‖ against r1+r2 and |r1−r2|, the standard
    // radical-line construction. Coincident circles (infinite
    // crossings) are refused outright — there is no single crossing
    // point to name, and the overlap is the whole curve.
    void intersect_circles(const SketchCircleLike& a, const SketchCircleLike& b,
                           std::size_t i, std::size_t j, const auto& report) const {
        const math::Vec2f dvec = b.center - a.center;
        const float D = dvec.length();
        if (D <= kJointCoincidenceTol) {
            if (std::abs(a.radius - b.radius) <= kJointCoincidenceTol)
                throw std::invalid_argument(
                    "SketchEntitySet: entities must intersect only at "
                    "declared shared joints, got " + flat_name(i) + " x " +
                    flat_name(j) + " — the two circles are coincident (same "
                    "center and radius): they cross at EVERY point of the "
                    "curve. Remedy: move or resize one of them.");
            return; // concentric, distinct radii: no crossing
        }
        const float sum = a.radius + b.radius;
        const float dif = std::abs(a.radius - b.radius);
        if (D > sum + kJointCoincidenceTol || D < dif - kJointCoincidenceTol)
            return;
        const math::Vec2f u = dvec * (1.0f / D);
        const float aa = (D * D + a.radius * a.radius - b.radius * b.radius) /
                         (2.0f * D);
        const float h2 = a.radius * a.radius - aa * aa;
        const math::Vec2f base = a.center + u * aa;
        if (h2 <= 0.0f) {
            report(base); // tangent (external or internal)
            return;
        }
        const float h = std::sqrt(h2);
        report(base + math::Vec2f{-u.y * h, u.x * h});
        report(base + math::Vec2f{u.y * h, -u.x * h});
    }

    // --- members ----------------------------------------------------------
    std::vector<SketchChain> chains_;
    std::vector<std::variant<Halfplane2D, Disc2D>> fields_;
    std::vector<FlatRef> flat_;
    std::vector<Joint> joints_;
};

// --- ProfileSDF ---------------------------------------------------------
//
// The 2D profile field (the table's ProfileSDF row): max-composition of
// the certified entity set, bounds by intersection (with the declared
// contour-hull closure for the orientation-blind diagonal case), L = 1,
// winning-branch gradient. NOT an SDFNode: it is a 2D creature — the
// extrusion (sketch_extrude.hpp) is the 3D node that carries it.
class ProfileSDF {
public:
    explicit ProfileSDF(std::vector<SketchChain> chains)
        : set_(std::move(chains)) {
        compute_bounds();
    }
    explicit ProfileSDF(SketchEntitySet certified)
        : set_(std::move(certified)) {
        compute_bounds();
    }

    SDFSample2D sample(const math::Vec2f& p) const noexcept {
        return set_.sample(p);
    }

    float lipschitz() const noexcept { return 1.0f; }

    const Bbox2f& bounds() const noexcept { return bounds_; }

    std::size_t entity_count() const noexcept { return set_.flat_count(); }

    const SketchEntitySet& entity_set() const noexcept { return set_; }

    std::string describe() const {
        return "profile(" + std::to_string(set_.flat_count()) + " entities, " +
               std::to_string(set_.chains().size()) + " chains)";
    }

private:
    void compute_bounds() {
        // 1. The region boxes (the IntersectionSDF rule at 2D).
        Bbox2f region = Bbox2f::universe();
        bool has_outer_disc = false;
        std::vector<float> normal_angles;
        for (const auto& f : set_.fields()) {
            region = region.intersect(std::visit(
                [](const auto& e) { return e.bounds(); }, f));
            if (const auto* d = std::get_if<Disc2D>(&f); d && !d->hole())
                has_outer_disc = true;
            if (const auto* h = std::get_if<Halfplane2D>(&f))
                normal_angles.push_back(
                    std::atan2(h->outward_normal().y, h->outward_normal().x));
        }
        // 2. The contour hull: the AABB of the entity curves. Covers R
        //    whenever R is bounded (a bounded region is contained in
        //    the convex hull of its boundary, and the boundary lives on
        //    the curves). Segments contribute endpoints, circles their
        //    boxes, arcs their SPAN boxes (endpoints + the axis
        //    extremes inside the span).
        Bbox2f hull;
        for (const auto& chain : set_.chains())
            for (const SketchEntity& e : chain.entities)
                hull.expand(curve_box(e));
        // 3. Boundedness. If the region boxes failed to bound (a
        //    diagonal half-plane's universe), certify boundedness by
        //    the exact condition and fall back to the hull; a genuinely
        //    unbounded profile is the ADR's construction error.
        if (!region.bounded() && !has_outer_disc && !positive_span(normal_angles))
            throw std::invalid_argument(
                "ProfileSDF: the profile does not close — the intersection "
                "of the entity regions is unbounded (no outer disc bounds "
                "it and the segment outward normals do not positively span "
                "the plane). The extrusion of an unbounded region is not a "
                "solid. Remedy: check the orientation bits — a closed "
                "contour of correctly-oriented entities intersects to a "
                "bounded region.");
        bounds_ = region.intersect(hull);
    }

    // Positive spanning of the outward normals: 0 in the interior of
    // their convex hull ⟺ no closed half-plane contains them all ⟺ the
    // max angular gap (with wraparound) is < π. Fewer than 3 half-plane
    // normals never span. The guard rejects needle-degenerate contours
    // whose largest gap is within kSpanAngularGuard of π (atan2 noise
    // there is of the same order — refusing is the honest side).
    static bool positive_span(const std::vector<float>& angles) {
        if (angles.size() < 3) return false;
        std::vector<float> a = angles;
        std::sort(a.begin(), a.end());
        float max_gap = a.front() + kTwoPi - a.back();
        for (std::size_t i = 1; i < a.size(); ++i)
            max_gap = std::max(max_gap, a[i] - a[i - 1]);
        const float pi = 3.14159265358979323846f;
        return max_gap < pi - kSpanAngularGuard;
    }

    static Bbox2f curve_box(const SketchEntity& e) {
        Bbox2f b;
        switch (e.kind) {
        case SketchEntity::Kind::Segment:
            b.expand(e.segment.a);
            b.expand(e.segment.b);
            break;
        case SketchEntity::Kind::Circle:
            b.expand(math::Vec2f{e.circle.center.x - e.circle.radius,
                                 e.circle.center.y - e.circle.radius});
            b.expand(math::Vec2f{e.circle.center.x + e.circle.radius,
                                 e.circle.center.y + e.circle.radius});
            break;
        case SketchEntity::Kind::Arc: {
            b.expand(arc_start_point(e.arc));
            b.expand(arc_end_point(e.arc));
            for (int k = 0; k < 4; ++k) {
                const float theta = static_cast<float>(k) *
                                    (3.14159265358979323846f * 0.5f);
                const float rel = sketch_detail::normalized_angle(
                    theta - e.arc.start_angle);
                if (rel <= e.arc.sweep) {
                    b.expand(e.arc.center +
                             math::Vec2f{e.arc.radius * std::cos(theta),
                                         e.arc.radius * std::sin(theta)});
                }
            }
            break;
        }
        }
        return b;
    }

    SketchEntitySet set_;
    Bbox2f bounds_;
};

} // namespace CAD_0::sdf
