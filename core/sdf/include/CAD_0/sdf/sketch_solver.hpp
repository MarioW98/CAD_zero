// core/sdf/include/CAD_0/sdf/sketch_solver.hpp
//
// THE CONSTRAINT SOLVER — ADR-0040 D2 (G4 m2, the accepted act): the
// graph, the bill, the admission's rank test, the damped min-norm
// Newton with the canonical initial point, the bands ε_solve, the
// three honest classes, the degenerate verification on the stored
// bytes BEFORE the recipe write. This header is that contract's code
// side, in the sketch kernel's own header-only house style.
//
// THE THREE INSTRUMENTS (D2.1, each covering what the others cannot
// — the certificate's two-voice discipline at the solver's scale):
//
//   1. THE COUNTING — the bill: the DoF (the entities' serialized
//      floats, D1's 4/3/5) minus the consumed (the admitted
//      constraints' consumptions, D1's table). Cheap, always
//      displayed, and a LIE on its own (dependent rows) — which is
//      why the admission exists.
//   2. THE ADMISSION — the rank test at the CANONICAL POINT (the
//      current geometry): the candidate's Jacobian rows must raise
//      the admitted set's rank by exactly the candidate's
//      consumption, or the constraint is REFUSED as redundant —
//      sketch_over_constrained, the culprit named (kind, operands,
//      value), the bill, the remedy. A rank = DoF sketch refuses
//      every further constraint by the same test: the
//      over-constrained NEVER reaches the solver. A point test,
//      honestly named as such — the global question is the solve's
//      own.
//   3. THE SOLVE — the nonlinear verdict: the damped min-norm Newton
//      (JᵀJ + λI) δ = −JᵀF from the canonical initial point (the
//      recipe's current values — MAI restart casuali, ADR-0014's
//      determinism applied to the solve: same input, same output,
//      BIT FOR BIT, the canonical order everywhere, the house's
//      -ffp-contract=off). The iterate that lands is SOLVED with the
//      bill (the free count reported — the under-constrained
//      sketch's free parameters ride the iteration untouched: JᵀF
//      has no component in J's null space); the iterate that fails
//      the band is an ANSWER with a class, never an approximation
//      passed off as solved.
//
// THE UNSCALED FORM (D2.2): the bands are NOT folded into the linear
// algebra (1/band ≈ 1e5 would put the conditioning at 1e16 and the
// solve under); they enter where they belong — the acceptance test
// (the try must decrease the distance to solved, measured IN BAND
// UNITS) and the band check.
//
// THE THREE HONEST CLASSES (the error taxonomy's sketch_* family,
// ADR-0033 D4's amended corpus):
//
//   * sketch_over_constrained — at the ADMISSION (the returned
//     record, actionable: the culprit, the bill, the remedy).
//   * sketch_not_solvable — at the SOLVE: the residual per
//     constraint (the largest named), the values at the exit
//     BOUNDED (the damped least-squares closest approach, never a
//     diverged number), the honest diagnosis. MAI un crash.
//   * sketch_degenerate — at the VERIFICATION: the solved values are
//     stored to the recipe's float32 (the store IS the quantizer,
//     ADR-0014's rule) and re-read, and the kernel's own
//     construction checks (the constructors ARE the checks,
//     sketch_profile.hpp) run on the STORED BYTES: the
//     zero-length segment, the closed arc, the non-positive radius
//     refuse with the kernel's own words VERBATIM — and the refusal
//     PRECEDES the write (the recipe is untouched).
//
// THE BANDS (ADR-0035 D2's SOLVE row, amended the same day this
// header landed): ε_solve^lin = kJointCoincidenceTol and
// ε_solve^ang = kParallelAngleTol, RE-HOMED from the sketch kernel's
// own declared constants — ONE value per world, the certificate and
// the solve the two consumers, ZERO copies: the constants live in
// sketch_profile.hpp, this header only consumes them (the lossless
// float→double promotion).
//
// THE STORE-BACK BOUNDARY: the solver computes in double, the recipe
// stores float32 (the sketch world stays float — ADR-0035 D4's road
// (a)); the float32 store noise (5.96e-7 at scale 10) sits inside
// every band by the declared margins (16.8x linear, 8.4x angular).
//
// THE LINEAR ALGEBRA IS IN HOUSE (sketch_linalg.hpp — the one file,
// D2.2's own pricing): no Eigen, no LAPACK, the whitelist unchanged.
#pragma once

#include "CAD_0/sdf/sketch_entities.hpp"
#include "CAD_0/sdf/sketch_profile.hpp"
#include "CAD_0/sdf/sketch_linalg.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace CAD_0::sdf {

// ---------------------------------------------------------------------------
// The bands and the structure constants (D2.2 — ALL declared, none
// invented; the derivation script's own numbers, the pre-registration
// this implementation reproduces).
// ---------------------------------------------------------------------------

/// ε_solve^lin — the Linear world's member: the coincidences, the
/// dimensions, the tangency's distance, the pins on coordinate
/// fields. RE-HOMED from kJointCoincidenceTol (the chain's joint
/// coincidence): one value, two consumers, zero drift.
inline constexpr double kSolveEpsLin =
    static_cast<double>(kJointCoincidenceTol); // 1e-5

/// ε_solve^ang — the Angular world's member: the parallel's unit
/// cross, the perpendicular's unit dot, the pins on angle fields.
/// RE-HOMED from kParallelAngleTol (the seg–seg parallel
/// classification): the solve's band sits AT the certificate's own
/// classification band — a looser solve band would drift from the
/// materialization's classification, the worse evil.
inline constexpr double kSolveEpsAng =
    static_cast<double>(kParallelAngleTol); // 1e-6

inline constexpr int kSolveMaxIter = 64;        ///< the cap: the not-solvable boundary, not a promise (worst measured golden 19, margin 3.4x)
inline constexpr double kSolveLam0 = 1e-6;      ///< light on purpose: the well-posed cases run as pure Newton from the first step
inline constexpr double kSolveLamUp = 10.0;     ///< the rejection's escalation
inline constexpr double kSolveLamDown = 4.0;    ///< the acceptance's relaxation
inline constexpr double kSolveLamFloor = 1e-14; ///< λ never relaxes below this
inline constexpr double kSolveLamCap = 1e12;    ///< the escalation's boundary
inline constexpr double kSolveStepFloor = 1e-12; ///< ‖δ‖ at the natural convergence
inline constexpr int kSolveStallWindow = 3;     ///< the honest failures' patience
inline constexpr double kSolveRankCutoff = 1e-12; ///< the SVD rank cutoff (the derivation's RCOND)

// ---------------------------------------------------------------------------
// The addressing (D2.1): the operand is (entity, endpoint) /
// (entity, entity) — the sketch's own authored layout, the same
// order the serialization practices.
// ---------------------------------------------------------------------------

/// An entity's address: the chain's index, the entity's position in
/// the chain (the roots' own identity, ADR-0036 D4).
struct SketchEntityRef {
    std::size_t chain{};
    std::size_t entity{};
};

/// Which endpoint of an entity (segments: a and b; arcs: the DERIVED
/// arc_start_point / arc_end_point — the trig-carrying branch;
/// circles have none).
enum class SketchEnd : std::uint8_t { Start, End };

/// A point operand: (entity, endpoint).
struct SketchPointRef {
    SketchEntityRef entity{};
    SketchEnd end{SketchEnd::Start};

    SketchPointRef() = default;
    SketchPointRef(SketchEntityRef e, SketchEnd which) noexcept
        : entity(e), end(which) {}
    /// The ergonomic form (the bindings' surface): end == true is
    /// SketchEnd::End, false is Start — the enum stays the honest
    /// vocabulary, the bool the Python-facing convenience.
    SketchPointRef(SketchEntityRef e, bool is_end) noexcept
        : entity(e), end(is_end ? SketchEnd::End : SketchEnd::Start) {}
};

/// A raw parameter address for the pin (D4): the entity and the
/// FIELD's position in the entity's own serialization order — the
/// recipe's own float, the atom the named dimensions measure with.
/// Segment fields: x0, y0, x1, y1 (0..3). Circle: cx, cy, r (0..2).
/// Arc: cx, cy, r, a0, sweep (0..4).
struct SketchParamRef {
    SketchEntityRef entity{};
    std::uint8_t field{};
};

/// The entity's DoF (D1's table — the count of the independent real
/// parameters in the recipe payload, the same floats the panel edits
/// and the format writes).
inline std::size_t sketch_entity_dof(SketchEntity::Kind k) noexcept {
    switch (k) {
    case SketchEntity::Kind::Segment: return 4;
    case SketchEntity::Kind::Circle: return 3;
    case SketchEntity::Kind::Arc: return 5;
    }
    return 0;
}

/// The pin's world: the pins on coordinate/radius fields are Linear;
/// the pins on the arc's angle fields (a0, sweep — fields 3 and 4)
/// are Angular (the band each |violation| is held to).
inline bool sketch_field_is_angular(SketchEntity::Kind k,
                                    std::uint8_t field) noexcept {
    return k == SketchEntity::Kind::Arc && field >= 3;
}

/// The field's own name, for the renders (the culprit's name is part
/// of every refusal's payload).
inline const char* sketch_field_name(SketchEntity::Kind k,
                                     std::uint8_t field) noexcept {
    if (k == SketchEntity::Kind::Segment) {
        static const char* seg[4] = {"x0", "y0", "x1", "y1"};
        return field < 4 ? seg[field] : "?";
    }
    if (k == SketchEntity::Kind::Circle) {
        static const char* cir[3] = {"cx", "cy", "r"};
        return field < 3 ? cir[field] : "?";
    }
    static const char* arc[5] = {"cx", "cy", "r", "a0", "sweep"};
    return field < 5 ? arc[field] : "?";
}

// ---------------------------------------------------------------------------
// The constraints v1 (D1's table — the kernel's own three entity
// kinds, nothing invented; the consumption of each DERIVED as the
// numerical rank of the constraint's Jacobian at a canonical point,
// verified in the derivation script):
//
//   | Constraint                      | The equation (v1's form)     | Consumes |
//   |---------------------------------|------------------------------|----------|
//   | coincidente (endpoint,endpoint) | the two coordinates equal    |    2     |
//   | parallelo (segment, segment)    | the unit cross = 0           |    1     |
//   | perpendicolare (segment,segment)| the unit dot = 0             |    1     |
//   | tangente (linea, cerchio)       | signed distance = s·r        |    1     |
//   | lunghezza (segment)             | |AB| = v, v > 0              |    1     |
//   | raggio (circle/arc)             | r = v, v > 0                 |    1     |
//   | distanza (point, point)         | |PQ| = v, v > 0              |    1     |
//   | il pin (D4, the raw dimension)  | x_i = v, any v               |    1     |
// ---------------------------------------------------------------------------

struct SketchConstraint {
    enum class Kind : std::uint8_t {
        Coincidence,    ///< (a, b): two endpoint operands — 2 rows, Linear
        Parallel,       ///< (ea, eb): segment × segment — 1 row, Angular
        Perpendicular,  ///< (ea, eb): segment × segment — 1 row, Angular
        Tangent,        ///< (ea, eb) + case_bit: segment × circle — 1 row, Linear
        Length,         ///< (ea) + value: segment — 1 row, Linear
        Radius,         ///< (ea) + value: circle or arc — 1 row, Linear
        Distance,       ///< (a, b) + value: point × point — 1 row, Linear
        Pin,            ///< (pin) + value: the raw-parameter dimension — 1 row
    };
    Kind kind{Kind::Coincidence};
    SketchPointRef a{}, b{};   ///< Coincidence, Distance operands
    SketchEntityRef ea{}, eb{};///< Parallel/Perpendicular (seg,seg); Tangent (seg,cir); Length (seg); Radius (cir/arc)
    double value{};            ///< Length, Radius, Distance, Pin
    int case_bit{};            ///< Tangent: +1 ESTERNO / −1 INTERNO (DATA, never chosen)
    SketchParamRef pin{};      ///< Pin's raw-parameter address
};

/// The constraint's consumption (D1's derived table).
inline std::size_t sketch_constraint_consumption(const SketchConstraint& c) noexcept {
    return c.kind == SketchConstraint::Kind::Coincidence ? 2 : 1;
}

/// The kind's own NAME — the one vocabulary every consumer quotes: the
/// bindings' ``kind`` property, the file's ``"kind"`` field (G4 m4: the
/// constraints ride NAMED fields — kind, operands, value, case bit,
/// pin — and the field's value is the kernel's own word, never a
/// second spelling invented beside it). One home, zero drift.
inline const char* sketch_constraint_kind_name(
    SketchConstraint::Kind k) noexcept {
    switch (k) {
    case SketchConstraint::Kind::Coincidence:   return "coincidence";
    case SketchConstraint::Kind::Parallel:      return "parallel";
    case SketchConstraint::Kind::Perpendicular: return "perpendicular";
    case SketchConstraint::Kind::Tangent:       return "tangent";
    case SketchConstraint::Kind::Length:        return "length";
    case SketchConstraint::Kind::Radius:        return "radius";
    case SketchConstraint::Kind::Distance:      return "distance";
    case SketchConstraint::Kind::Pin:           return "pin";
    }
    return "?";
}

/// The name's kind — the reader's half of the same vocabulary (the
/// file's ``"kind"`` dispatch). Returns false for anything else: the
/// CALLER names the unknown word in its own explicit error (the house
/// idiom — the refusal quotes what it read).
inline bool sketch_constraint_kind_from_name(
    std::string_view name, SketchConstraint::Kind& out) noexcept {
    if (name == "coincidence")   { out = SketchConstraint::Kind::Coincidence; return true; }
    if (name == "parallel")      { out = SketchConstraint::Kind::Parallel; return true; }
    if (name == "perpendicular") { out = SketchConstraint::Kind::Perpendicular; return true; }
    if (name == "tangent")       { out = SketchConstraint::Kind::Tangent; return true; }
    if (name == "length")        { out = SketchConstraint::Kind::Length; return true; }
    if (name == "radius")        { out = SketchConstraint::Kind::Radius; return true; }
    if (name == "distance")      { out = SketchConstraint::Kind::Distance; return true; }
    if (name == "pin")           { out = SketchConstraint::Kind::Pin; return true; }
    return false;
}

// --- The factories, each VALIDATING its value rules with the ADR's
// own remedies (std::invalid_argument, the house idiom; the
// structural rules — operand kinds, in-range addresses — are the
// MODEL's own, at add time, against the real chains).

inline SketchConstraint constraint_coincidence(SketchPointRef a, SketchPointRef b) {
    SketchConstraint c;
    c.kind = SketchConstraint::Kind::Coincidence;
    c.a = a; c.b = b;
    return c;
}

inline SketchConstraint constraint_parallel(SketchEntityRef a, SketchEntityRef b) {
    SketchConstraint c;
    c.kind = SketchConstraint::Kind::Parallel;
    c.ea = a; c.eb = b;
    return c;
}

inline SketchConstraint constraint_perpendicular(SketchEntityRef a, SketchEntityRef b) {
    SketchConstraint c;
    c.kind = SketchConstraint::Kind::Perpendicular;
    c.ea = a; c.eb = b;
    return c;
}

inline SketchConstraint constraint_tangent(SketchEntityRef segment,
                                           SketchEntityRef circle, int case_bit) {
    if (case_bit != 1 && case_bit != -1)
        throw std::invalid_argument(
            "constraint_tangent: the case bit must be +1 (ESTERNO) or "
            "-1 (INTERNO), got " + std::to_string(case_bit) +
            " — the tangent's equation (signed distance = s·r) admits "
            "BOTH branches and the constraint carries the case as DATA: "
            "without the bit an edit that pushes the geometry across "
            "the branch would let Newton settle on the wrong side. "
            "Remedy: derive it at creation with derive_tangent_case_bit "
            "or pass +1/-1 explicitly.");
    SketchConstraint c;
    c.kind = SketchConstraint::Kind::Tangent;
    c.ea = segment; c.eb = circle;
    c.case_bit = case_bit;
    return c;
}

inline SketchConstraint constraint_length(SketchEntityRef segment, double v) {
    if (!std::isfinite(v) || v <= 0.0)
        throw std::invalid_argument(
            "constraint_length: the value must be finite and > 0, got " +
            std::to_string(v) +
            " — a segment of length 0 has no direction to speak (the "
            "length's gradient is undefined at its own solution) and a "
            "negative length is no segment at all. Remedy: use the "
            "coincidence for v = 0 — the right tool for that question.");
    SketchConstraint c;
    c.kind = SketchConstraint::Kind::Length;
    c.ea = segment; c.value = v;
    return c;
}

inline SketchConstraint constraint_radius(SketchEntityRef circle_or_arc, double v) {
    if (!std::isfinite(v) || v <= 0.0)
        throw std::invalid_argument(
            "constraint_radius: the value must be finite and > 0, got " +
            std::to_string(v) +
            " — the zero radius is Disc2D's own construction refusal "
            "(a non-positive radius inverts the field's sign class) and "
            "a negative radius is no disc at all. Remedy: draw the "
            "entity away from zero or remove the dimension.");
    SketchConstraint c;
    c.kind = SketchConstraint::Kind::Radius;
    c.ea = circle_or_arc; c.value = v;
    return c;
}

inline SketchConstraint constraint_distance(SketchPointRef a, SketchPointRef b, double v) {
    if (!std::isfinite(v) || v <= 0.0)
        throw std::invalid_argument(
            "constraint_distance: the value must be finite and > 0, got " +
            std::to_string(v) +
            " — the point-point distance at v = 0 IS the coincidence, "
            "and its equation's gradient (P−Q)/|PQ| is UNDEFINED at "
            "its own solution: a constraint non-differentiable at the "
            "point it demands is unsolvable by Newton by construction. "
            "Remedy: use the coincidence — the right tool for v = 0.");
    SketchConstraint c;
    c.kind = SketchConstraint::Kind::Distance;
    c.a = a; c.b = b; c.value = v;
    return c;
}

inline SketchConstraint constraint_pin(SketchParamRef p, double v) {
    if (!std::isfinite(v))
        throw std::invalid_argument(
            "constraint_pin: the value must be finite, got " +
            std::to_string(v) +
            " — a non-finite pin writes NaN into the recipe. Remedy: "
            "pin a finite value (the parameter's own current value is "
            "the pinning gesture's default).");
    SketchConstraint c;
    c.kind = SketchConstraint::Kind::Pin;
    c.pin = p; c.value = v;
    return c;
}

/// The tangent's case bit, derived at CREATION from the current
/// geometry (D1: the side the center is on, at the moment the author
/// declares the constraint): +1 ESTERNO (the signed distance from
/// the center to the line positive under the left-normal
/// convention), −1 INTERNO. The bit is pinned in the recipe; the
/// solver consumes the sign and NEVER chooses.
inline int derive_tangent_case_bit(const std::vector<SketchChain>& chains,
                                   SketchEntityRef segment,
                                   SketchEntityRef circle) {
    const SketchEntity& s = chains.at(segment.chain).entities.at(segment.entity);
    const SketchEntity& c = chains.at(circle.chain).entities.at(circle.entity);
    const math::Vec2f a = s.segment.a;
    const math::Vec2f d = s.segment.b - s.segment.a;
    const float n = std::sqrt(d.x * d.x + d.y * d.y);
    if (!(n > 0.0f))
        throw std::invalid_argument(
            "derive_tangent_case_bit: the segment is degenerate — no "
            "left normal exists to derive the side from. Remedy: give "
            "the segment a non-zero extent.");
    const math::Vec2f nh{-d.y / n, d.x / n}; // the LEFT unit normal of a->b
    const math::Vec2f u = c.circle.center - a;
    const double sd = static_cast<double>(u.x) * nh.x +
                      static_cast<double>(u.y) * nh.y;
    return sd >= 0.0 ? 1 : -1;
}

/// The entity's render for the operands' names (the roots' own
/// addressing: the chain's position elided on single-chain sketches,
/// carried on multi-chain ones — the naming grammar's own rule).
inline std::string sketch_entity_label(const std::vector<SketchChain>& chains,
                                       SketchEntityRef r) {
    const std::string pos = "s" + std::to_string(r.entity);
    if (chains.size() <= 1) return pos;
    return "c" + std::to_string(r.chain) + " " + pos;
}

inline std::string sketch_point_label(const std::vector<SketchChain>& chains,
                                      SketchPointRef p) {
    return sketch_entity_label(chains, p.entity) +
           (p.end == SketchEnd::End ? ".end" : ".start");
}

/// The constraint's own name — the culprit's render in every refusal
/// ("the distance on (s0.end, s1.end) = 5").
inline std::string describe_constraint(const SketchConstraint& c,
                                       const std::vector<SketchChain>& chains) {
    auto val = [](double v) {
        if (v == std::floor(v) && std::abs(v) < 1e15) {
            char buf[32];
            std::snprintf(buf, sizeof(buf), "%lld",
                          static_cast<long long>(v));
            return std::string(buf);
        }
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.17g", v);
        return std::string(buf);
    };
    switch (c.kind) {
    case SketchConstraint::Kind::Coincidence:
        return "the coincidence on (" + sketch_point_label(chains, c.a) +
               " = " + sketch_point_label(chains, c.b) + ")";
    case SketchConstraint::Kind::Parallel:
        return "the parallel on (" + sketch_entity_label(chains, c.ea) +
               " ∥ " + sketch_entity_label(chains, c.eb) + ")";
    case SketchConstraint::Kind::Perpendicular:
        return "the perpendicular on (" + sketch_entity_label(chains, c.ea) +
               " ⊥ " + sketch_entity_label(chains, c.eb) + ")";
    case SketchConstraint::Kind::Tangent:
        return "the tangent on (" + sketch_entity_label(chains, c.ea) +
               " ~ " + sketch_entity_label(chains, c.eb) + "), " +
               (c.case_bit > 0 ? "ESTERNO" : "INTERNO");
    case SketchConstraint::Kind::Length:
        return "the length |" + sketch_entity_label(chains, c.ea) +
               "| = " + val(c.value);
    case SketchConstraint::Kind::Radius:
        return "the radius r(" + sketch_entity_label(chains, c.ea) +
               ") = " + val(c.value);
    case SketchConstraint::Kind::Distance:
        return "the distance on (" + sketch_point_label(chains, c.a) +
               ", " + sketch_point_label(chains, c.b) + ") = " + val(c.value);
    case SketchConstraint::Kind::Pin:
        return "the pin on " + sketch_entity_label(chains, c.pin.entity) +
               "." + sketch_field_name(chains.empty()
                                           ? SketchEntity::Kind::Segment
                                           : chains.at(c.pin.entity.chain)
                                                 .entities.at(c.pin.entity.entity)
                                                 .kind,
                                       c.pin.field) +
               " = " + val(c.value);
    }
    return "the constraint";
}

// ---------------------------------------------------------------------------
// LA BOLLETTA — the honest answer for each state (D2.1).
// ---------------------------------------------------------------------------

struct SketchBill {
    std::size_t dof{};      ///< the entities' parameters, summed (D1's 4/3/5)
    std::size_t consumed{}; ///< the admitted constraints' consumptions, summed (= rank, by admission)
    std::size_t free() const noexcept { return dof >= consumed ? dof - consumed : 0; }
    bool full() const noexcept { return dof > 0 && consumed >= dof; }
    std::string describe() const {
        return std::to_string(dof) + " - " + std::to_string(consumed) +
               " = " + std::to_string(free()) +
               (full() ? " (FULL)" : " free");
    }
};

// ---------------------------------------------------------------------------
// THE ADMISSION's answer (D2.1): the returned record, actionable —
// never an exception, never a silence. The refusal is the class
// sketch_over_constrained.
// ---------------------------------------------------------------------------

struct SketchAdmission {
    bool admitted{};
    SketchConstraint candidate{};  ///< the culprit when refused: kind, operands, value
    std::size_t rank_before{};     ///< the admitted set's rank at the current geometry
    std::size_t rank_after{};      ///< with the candidate
    std::size_t consumption{};     ///< the candidate's own (D1's table)
    double tail0{}, tail1{};       ///< refused: the augmented spectrum's two smallest singular values
    SketchBill bill{};             ///< the bill at the refusal
    std::string message{};         ///< the actionable text
};

// ---------------------------------------------------------------------------
// THE SOLVE's answer: the statuses and the payloads of the two honest
// failure classes ride the same record (a closed variant in the
// house pattern).
// ---------------------------------------------------------------------------

enum class SketchSolveStatus : std::uint8_t {
    Solved,      ///< 'solved' with the bill (the free count reported)
    NotSolvable, ///< sketch_not_solvable: the residual per constraint, the bounded exit, the diagnosis
    Degenerate,  ///< sketch_degenerate: the kernel's class VERBATIM, the recipe untouched
};

/// One constraint row's residual at the exit (the not-solvable
/// payload: the largest NAMED — the constraint that does not hold).
struct SketchViolation {
    std::size_t constraint{}; ///< the owning constraint's authored index
    std::string label{};      ///< the row's render (the culprit's own name)
    double value{};           ///< the residual, signed
    bool angular{};           ///< the world: false = Linear, true = Angular
};

struct SketchSolveResult {
    SketchSolveStatus status{};
    int iterations{};                 ///< the count (D5's pinned numbers)
    SketchBill bill{};                ///< with the free count reported on Solved
    std::vector<SketchChain> chains{};///< Solved: the STORED (float32) solved recipe; refusals: the input, untouched
    // --- sketch_not_solvable's payload ---
    std::vector<SketchViolation> violations{}; ///< per row, authored order
    std::size_t largest{};            ///< the index (into violations) of the largest |value|, first on ties
    double residual_norm{};           ///< |F| at the exit
    std::vector<double> exit_params{};///< the canonical parameter vector at the exit (double): NotSolvable — the bounded closest approach; Solved/Degenerate — the pre-store doubles
    std::string diagnosis{};          ///< the honest diagnosis
    // --- sketch_degenerate's payload ---
    std::string degenerate_entity{};  ///< the entity's address (chain, position)
    std::string degenerate_message{}; ///< the kernel's own class, VERBATIM
};

// ---------------------------------------------------------------------------
// The solver's private machinery (the canonical layout, the endpoint
// derivation, the residual/Jacobian assembly — the derivation
// script's own formulas, ported exactly).
// ---------------------------------------------------------------------------
namespace sketch_solve_detail {

struct EntitySlot {
    std::size_t chain{};
    std::size_t entity{};
    std::size_t offset{}; ///< the parameter vector's flat start
    SketchEntity::Kind kind{};
};

/// The canonical parameter layout (D2.1): chain order, entity order,
/// field order — the serialization's own order. x is the recipe's
/// own floats in THIS order; the same order the store writes back.
struct Layout {
    std::vector<EntitySlot> slots;
    std::size_t total{};
};

inline Layout build_layout(const std::vector<SketchChain>& chains) {
    Layout L;
    std::size_t off = 0;
    for (std::size_t c = 0; c < chains.size(); ++c) {
        const auto& ents = chains[c].entities;
        for (std::size_t e = 0; e < ents.size(); ++e) {
            L.slots.push_back(EntitySlot{c, e, off, ents[e].kind});
            off += sketch_entity_dof(ents[e].kind);
        }
    }
    L.total = off;
    return L;
}

inline const EntitySlot* find_slot(const Layout& L, SketchEntityRef r) {
    for (const auto& s : L.slots)
        if (s.chain == r.chain && s.entity == r.entity) return &s;
    return nullptr;
}

/// The recipe's current values, loaded into x (float→double, the
/// lossless promotion — the canonical initial point's own source).
inline void load_params(const std::vector<SketchChain>& chains,
                        const Layout& L, std::vector<double>& x) {
    x.assign(L.total, 0.0);
    for (const auto& s : L.slots) {
        const SketchEntity& e = chains[s.chain].entities[s.entity];
        double* p = &x[s.offset];
        switch (s.kind) {
        case SketchEntity::Kind::Segment:
            p[0] = e.segment.a.x; p[1] = e.segment.a.y;
            p[2] = e.segment.b.x; p[3] = e.segment.b.y;
            break;
        case SketchEntity::Kind::Circle:
            p[0] = e.circle.center.x; p[1] = e.circle.center.y;
            p[2] = e.circle.radius;
            break;
        case SketchEntity::Kind::Arc:
            p[0] = e.arc.center.x; p[1] = e.arc.center.y;
            p[2] = e.arc.radius; p[3] = e.arc.start_angle;
            p[4] = e.arc.sweep;
            break;
        }
    }
}

/// The store: x written back as the recipe's float32 fields (the
/// store IS the quantizer, ADR-0014's rule — the verification
/// consumes the STORED bytes, never the doubles).
inline std::vector<SketchChain> store_params(const std::vector<SketchChain>& chains,
                                             const Layout& L,
                                             const std::vector<double>& x) {
    std::vector<SketchChain> out = chains;
    for (const auto& s : L.slots) {
        SketchEntity& e = out[s.chain].entities[s.entity];
        const double* p = &x[s.offset];
        switch (s.kind) {
        case SketchEntity::Kind::Segment:
            e.segment.a.x = static_cast<float>(p[0]);
            e.segment.a.y = static_cast<float>(p[1]);
            e.segment.b.x = static_cast<float>(p[2]);
            e.segment.b.y = static_cast<float>(p[3]);
            break;
        case SketchEntity::Kind::Circle:
            e.circle.center.x = static_cast<float>(p[0]);
            e.circle.center.y = static_cast<float>(p[1]);
            e.circle.radius = static_cast<float>(p[2]);
            break;
        case SketchEntity::Kind::Arc:
            e.arc.center.x = static_cast<float>(p[0]);
            e.arc.center.y = static_cast<float>(p[1]);
            e.arc.radius = static_cast<float>(p[2]);
            e.arc.start_angle = static_cast<float>(p[3]);
            e.arc.sweep = static_cast<float>(p[4]);
            break;
        }
    }
    return out;
}

/// An endpoint at x: segments read their params; arcs derive through
/// the trig (the one priced exception, D2.2 — libm cos/sin of a0).
struct Endpt {
    double x{}, y{};
};

inline Endpt endpoint_at(const std::vector<double>& x,
                         const EntitySlot& s, bool end) {
    const double* p = &x[s.offset];
    if (s.kind == SketchEntity::Kind::Segment)
        return end ? Endpt{p[2], p[3]} : Endpt{p[0], p[1]};
    // the arc: C + r·(cos φ, sin φ), φ = a0 (start) or a0+sweep (end)
    const double phi = end ? p[3] + p[4] : p[3];
    return Endpt{p[0] + p[2] * std::cos(phi), p[1] + p[2] * std::sin(phi)};
}

/// d(endpoint)/d(the entity's own fields) — a 2×dof block, row-major
/// (row 0 = x, row 1 = y). Segments: the identity selection. Arcs:
/// the chain rule through the trig.
inline void endpoint_jac(const EntitySlot& s, const std::vector<double>& x,
                         bool end, double J[2][5]) {
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 5; ++j) J[i][j] = 0.0;
    const double* p = &x[s.offset];
    if (s.kind == SketchEntity::Kind::Segment) {
        J[0][end ? 2 : 0] = 1.0;
        J[1][end ? 3 : 1] = 1.0;
        return;
    }
    const double phi = end ? p[3] + p[4] : p[3];
    const double cs = std::cos(phi), sn = std::sin(phi);
    // d(C + r·(cos φ, sin φ)): dC = I; dr = (cos φ, sin φ);
    // dφ = r·(−sin φ, cos φ) — φ = a0 [+ sweep], the sweep column
    // zero for the start point.
    J[0][0] = 1.0; J[1][1] = 1.0;
    J[0][2] = cs;  J[1][2] = sn;
    J[0][3] = -p[2] * sn; J[1][3] = p[2] * cs;
    if (end) { J[0][4] = -p[2] * sn; J[1][4] = p[2] * cs; }
}

/// The assembled system at x: F (rows), J (rows × total, row-major),
/// and per-row metadata (the world, the label, the owning
/// constraint). All in the AUTHORED order — the canonical order the
/// determinism contract rides.
struct Assembled {
    std::vector<double> F;
    std::vector<double> J;
    std::vector<bool> angular;
    std::vector<std::string> labels;
    std::vector<std::size_t> owners;
};

inline void push_row(Assembled& A, std::size_t total, double value, bool angular,
                     const std::string& label, std::size_t owner) {
    A.F.push_back(value);
    A.angular.push_back(angular);
    A.labels.push_back(label);
    A.owners.push_back(owner);
    A.J.insert(A.J.end(), total, 0.0);
}

/// One constraint's rows appended to A (the derivation script's own
/// formulas, ported exactly — the same arithmetic the pre-registration
/// verified in double).
inline void append_constraint(const Layout& L, const std::vector<double>& x,
                              const SketchConstraint& c, std::size_t owner,
                              const std::vector<SketchChain>& chains,
                              Assembled& A) {
    const std::size_t n = L.total;
    const std::string base = describe_constraint(c, chains);

    // helpers to touch J's LAST row
    auto last_row = [&A, n]() { return &A.J[(A.F.size() - 1) * n]; };

    switch (c.kind) {
    case SketchConstraint::Kind::Coincidence: {
        const EntitySlot* sa = find_slot(L, c.a.entity);
        const EntitySlot* sb = find_slot(L, c.b.entity);
        const bool ea = c.a.end == SketchEnd::End;
        const bool eb = c.b.end == SketchEnd::End;
        double Ja[2][5], Jb[2][5];
        endpoint_jac(*sa, x, ea, Ja);
        endpoint_jac(*sb, x, eb, Jb);
        const std::size_t da = sketch_entity_dof(sa->kind);
        const std::size_t db = sketch_entity_dof(sb->kind);
        const Endpt pa = endpoint_at(x, *sa, ea);
        const Endpt pb = endpoint_at(x, *sb, eb);
        for (int r = 0; r < 2; ++r) {
            push_row(A, n, r == 0 ? pa.x - pb.x : pa.y - pb.y, false,
                     base + ", row " + std::to_string(r + 1), owner);
            double* J = last_row();
            for (std::size_t k = 0; k < da; ++k)
                J[sa->offset + k] += Ja[r][k];
            for (std::size_t k = 0; k < db; ++k)
                J[sb->offset + k] -= Jb[r][k];
        }
        break;
    }
    case SketchConstraint::Kind::Distance: {
        const EntitySlot* sa = find_slot(L, c.a.entity);
        const EntitySlot* sb = find_slot(L, c.b.entity);
        const bool ea = c.a.end == SketchEnd::End;
        const bool eb = c.b.end == SketchEnd::End;
        double Ja[2][5], Jb[2][5];
        endpoint_jac(*sa, x, ea, Ja);
        endpoint_jac(*sb, x, eb, Jb);
        const std::size_t da = sketch_entity_dof(sa->kind);
        const std::size_t db = sketch_entity_dof(sb->kind);
        const Endpt pa = endpoint_at(x, *sa, ea);
        const Endpt pb = endpoint_at(x, *sb, eb);
        const double dx = pa.x - pb.x, dy = pa.y - pb.y;
        const double d = std::sqrt(dx * dx + dy * dy);
        push_row(A, n, d - c.value, false, base, owner);
        double* J = last_row();
        if (d > 0.0) {
            const double ux = dx / d, uy = dy / d;
            for (std::size_t k = 0; k < da; ++k)
                J[sa->offset + k] += ux * Ja[0][k] + uy * Ja[1][k];
            for (std::size_t k = 0; k < db; ++k)
                J[sb->offset + k] -= ux * Jb[0][k] + uy * Jb[1][k];
        }
        break;
    }
    case SketchConstraint::Kind::Pin: {
        const EntitySlot* s = find_slot(L, c.pin.entity);
        push_row(A, n, x[s->offset + c.pin.field] - c.value,
                 sketch_field_is_angular(s->kind, c.pin.field), base, owner);
        last_row()[s->offset + c.pin.field] = 1.0;
        break;
    }
    case SketchConstraint::Kind::Length: {
        const EntitySlot* s = find_slot(L, c.ea);
        const double* p = &x[s->offset];
        const double dx = p[2] - p[0], dy = p[3] - p[1];
        const double d = std::sqrt(dx * dx + dy * dy);
        push_row(A, n, d - c.value, false, base, owner);
        double* J = last_row();
        if (d > 0.0) {
            J[s->offset + 0] = -dx / d; J[s->offset + 1] = -dy / d;
            J[s->offset + 2] = dx / d;  J[s->offset + 3] = dy / d;
        }
        break;
    }
    case SketchConstraint::Kind::Radius: {
        const EntitySlot* s = find_slot(L, c.ea);
        push_row(A, n, x[s->offset + 2] - c.value, false, base, owner);
        last_row()[s->offset + 2] = 1.0;
        break;
    }
    case SketchConstraint::Kind::Parallel:
    case SketchConstraint::Kind::Perpendicular: {
        const EntitySlot* si = find_slot(L, c.ea);
        const EntitySlot* sj = find_slot(L, c.eb);
        const double* pi = &x[si->offset];
        const double* pj = &x[sj->offset];
        const double dxi = pi[2] - pi[0], dyi = pi[3] - pi[1];
        const double dxj = pj[2] - pj[0], dyj = pj[3] - pj[1];
        const double ni = std::sqrt(dxi * dxi + dyi * dyi);
        const double nj = std::sqrt(dxj * dxj + dyj * dyj);
        if (!(ni > 0.0) || !(nj > 0.0))
            throw std::invalid_argument(
                "the sketch solver: a parallel/perpendicular operand is a "
                "zero-length segment — the UNIT direction the equation "
                "consumes does not exist. Remedy: give the segment a "
                "non-zero extent (the degenerate states are the "
                "verification's own class, never the algebra's).");
        const double uix = dxi / ni, uiy = dyi / ni;
        const double ujx = dxj / nj, ujy = dyj / nj;
        // d(u)/d(d) = (I − u uᵀ)/n ; the start negated, the end positive
        const double Pii00 = (1.0 - uix * uix) / ni, Pii01 = (-uix * uiy) / ni;
        const double Pii11 = (1.0 - uiy * uiy) / ni;
        const double Pjj00 = (1.0 - ujx * ujx) / nj, Pjj01 = (-ujx * ujy) / nj;
        const double Pjj11 = (1.0 - ujy * ujy) / nj;
        double f;
        double gxi, gyi, gxj, gyj; // the gradient wrt the unit vectors
        if (c.kind == SketchConstraint::Kind::Parallel) {
            // f = ui × uj (the unit cross — scale-invariant, the Angular kind)
            f = uix * ujy - uiy * ujx;
            gxi = ujy;  gyi = -ujx;   // d(f)/d(ui)
            gxj = -uiy; gyj = uix;    // d(f)/d(uj)
        } else {
            // f = ui · uj (the unit dot)
            f = uix * ujx + uiy * ujy;
            gxi = ujx;  gyi = ujy;    // d(f)/d(ui)
            gxj = uix;  gyj = uiy;    // d(f)/d(uj)
        }
        push_row(A, n, f, true, base, owner);
        double* J = last_row();
        // chain rule: d(f)/d(si.start) = −Piᵀ·g_ui, d(f)/d(si.end) = +Piᵀ·g_ui
        const double gi0 = Pii00 * gxi + Pii01 * gyi;
        const double gi1 = Pii01 * gxi + Pii11 * gyi;
        J[si->offset + 0] = -gi0; J[si->offset + 1] = -gi1;
        J[si->offset + 2] = gi0;  J[si->offset + 3] = gi1;
        const double gj0 = Pjj00 * gxj + Pjj01 * gyj;
        const double gj1 = Pjj01 * gxj + Pjj11 * gyj;
        J[sj->offset + 0] = -gj0; J[sj->offset + 1] = -gj1;
        J[sj->offset + 2] = gj0;  J[sj->offset + 3] = gj1;
        break;
    }
    case SketchConstraint::Kind::Tangent: {
        const EntitySlot* ss = find_slot(L, c.ea);
        const EntitySlot* sc = find_slot(L, c.eb);
        const double* p = &x[ss->offset];
        const double* q = &x[sc->offset];
        const double ax = p[0], ay = p[1];
        const double dx = p[2] - p[0], dy = p[3] - p[1];
        const double len = std::sqrt(dx * dx + dy * dy);
        if (!(len > 0.0))
            throw std::invalid_argument(
                "the sketch solver: the tangent's segment is zero-length — "
                "no left normal exists for the signed distance. Remedy: "
                "give the segment a non-zero extent.");
        const double nhx = -dy / len, nhy = dx / len; // the LEFT unit normal
        const double Cx = q[0], Cy = q[1], r = q[2];
        // f = (C − a)·nh − s·r
        const double f = (Cx - ax) * nhx + (Cy - ay) * nhy -
                         static_cast<double>(c.case_bit) * r;
        push_row(A, n, f, false, base, owner);
        double* J = last_row();
        // d nh / d b = P/n − outer(perp(d)/(n²), d/n), P = [[0,−1],[1,0]]
        const double P00 = 0.0, P01 = -1.0 / len, P10 = 1.0 / len, P11 = 0.0;
        const double px = -dy / (len * len), py = dx / (len * len); // perp(d)/n²
        const double ux = dx / len, uy = dy / len;                  // d/n
        const double D00 = P00 - px * ux, D01 = P01 - px * uy;
        const double D10 = P10 - py * ux, D11 = P11 - py * uy;
        // df/d b = Dᵀ·nh
        const double dbx = D00 * nhx + D10 * nhy;
        const double dby = D01 * nhx + D11 * nhy;
        J[ss->offset + 0] = -nhx;        // df/d a = −nh
        J[ss->offset + 1] = -nhy;
        J[ss->offset + 2] = dbx;         // df/d b
        J[ss->offset + 3] = dby;
        J[sc->offset + 0] = nhx;         // df/d C = +nh
        J[sc->offset + 1] = nhy;
        J[sc->offset + 2] = -static_cast<double>(c.case_bit); // df/d r
        break;
    }
    }
}

inline Assembled assemble(const Layout& L, const std::vector<double>& x,
                          const std::vector<SketchConstraint>& cs,
                          const std::vector<SketchChain>& chains) {
    Assembled A;
    for (std::size_t i = 0; i < cs.size(); ++i)
        append_constraint(L, x, cs[i], i, chains, A);
    return A;
}

} // namespace sketch_solve_detail

// ---------------------------------------------------------------------------
// IL MODELLO — the constraint problem as recipe data: the chains (the
// recipe's entities) and the admitted constraints in AUTHORED ORDER
// (append-only in v1, D3.3 — the middle-insert of a constraint
// shifts the later positions and is refused by the same family rule:
// no such API exists here, by construction).
// ---------------------------------------------------------------------------
class SketchSolveModel {
public:
    /// The model validates the entities on construction (the
    /// kernel's own checks — the constructors ARE the checks): the
    /// canonical initial point must be in-class geometry.
    explicit SketchSolveModel(std::vector<SketchChain> chains)
        : chains_(std::move(chains)) {
        for (std::size_t c = 0; c < chains_.size(); ++c)
            for (std::size_t e = 0; e < chains_[c].entities.size(); ++e)
                validate_sketch_entity(chains_[c].entities[e]);
    }

    const std::vector<SketchChain>& chains() const noexcept { return chains_; }
    const std::vector<SketchConstraint>& constraints() const noexcept {
        return constraints_;
    }

    /// LA BOLLETTA: the DoF, the consumed, the free count (D2.1).
    SketchBill bill() const noexcept {
        SketchBill b;
        for (const auto& c : chains_)
            for (const auto& e : c.entities) b.dof += sketch_entity_dof(e.kind);
        for (const auto& c : constraints_)
            b.consumed += sketch_constraint_consumption(c);
        return b;
    }

    /// THE ADMISSION (D2.1): the rank test at the CURRENT geometry
    /// (the canonical point, declared as such). The candidate's
    /// Jacobian rows must raise the admitted set's rank by exactly
    /// the candidate's consumption; anything less is REDUNDANT — the
    /// returned record carries sketch_over_constrained with the
    /// culprit, the bill and the remedy. The admitted list is
    /// append-only: on success the candidate is appended in authored
    /// order.
    SketchAdmission add_constraint(const SketchConstraint& cand) {
        validate_operand_kinds_(cand);
        using sketch_solve_detail::build_layout;
        using sketch_solve_detail::assemble;
        using sketch_linalg::rank_with_cutoff;
        using sketch_linalg::svd_values;

        const auto L = build_layout(chains_);
        std::vector<double> x;
        sketch_solve_detail::load_params(chains_, L, x);

        const auto adm = assemble(L, x, constraints_, chains_);
        const auto cnd = assemble(L, x, {cand}, chains_);

        std::vector<double> sv_adm;
        svd_values(adm.J, adm.F.size(), L.total, sv_adm);
        const std::size_t rank_before =
            rank_with_cutoff(sv_adm, kSolveRankCutoff);

        // the augmented matrix: the admitted rows, then the candidate's
        std::vector<double> Jaug = adm.J;
        Jaug.insert(Jaug.end(), cnd.J.begin(), cnd.J.end());
        std::vector<double> svaug;
        svd_values(Jaug, adm.F.size() + cnd.F.size(), L.total, svaug);
        const std::size_t rank_after = rank_with_cutoff(svaug, kSolveRankCutoff);
        const std::size_t consumption = sketch_constraint_consumption(cand);
        const std::size_t raise = rank_after >= rank_before
                                      ? rank_after - rank_before : 0;

        SketchAdmission out;
        out.candidate = cand;
        out.rank_before = rank_before;
        out.rank_after = rank_after;
        out.consumption = consumption;
        out.bill = bill();
        if (svaug.size() >= 2) {
            out.tail1 = svaug[svaug.size() - 1];
            out.tail0 = svaug[svaug.size() - 2];
        } else if (!svaug.empty()) {
            out.tail1 = svaug[svaug.size() - 1];
        }

        if (raise == consumption && consumption > 0) {
            constraints_.push_back(cand);
            out.admitted = true;
            return out;
        }

        out.admitted = false;
        out.message =
            "sketch_over_constrained: " + describe_constraint(cand, chains_) +
            " does not raise the admitted set's rank (rank " +
            std::to_string(rank_before) + " -> " + std::to_string(rank_after) +
            ", the constraint consumes " + std::to_string(consumption) +
            ") — it is REDUNDANT at the current geometry. The bill: " +
            out.bill.describe() + ". Remedy: remove it, or remove what "
            "it duplicates.";
        return out;
    }

    /// D3.3's remove (the v1 edit family: add, remove, re-value —
    /// the middle-INSERT has no API by construction). Returns false
    /// on an out-of-range index, silently nothing else: the removal
    /// is a structural no-op for the rank invariant (a subset of an
    /// independent set is independent).
    bool remove_constraint(std::size_t index) {
        if (index >= constraints_.size()) return false;
        constraints_.erase(constraints_.begin() +
                           static_cast<std::ptrdiff_t>(index));
        return true;
    }

    /// D3.3's re-value (the dimension edit golden 8 practices: the
    /// edit writes the value, the re-solve runs from the current
    /// geometry). Only the value-carrying kinds; the value rules are
    /// the factories' own (v > 0 for the named dimensions, any finite
    /// v for the pin).
    bool revalue_constraint(std::size_t index, double v) {
        if (index >= constraints_.size()) return false;
        SketchConstraint& c = constraints_[index];
        switch (c.kind) {
        case SketchConstraint::Kind::Length:
        case SketchConstraint::Kind::Radius:
        case SketchConstraint::Kind::Distance:
            if (!std::isfinite(v) || v <= 0.0) return false;
            break;
        case SketchConstraint::Kind::Pin:
            if (!std::isfinite(v)) return false;
            break;
        default:
            return false; // no value to re-value
        }
        c.value = v;
        return true;
    }

    /// IL SOLVE (D2.2): the damped min-norm Newton from the CANONICAL
    /// INITIAL POINT (the recipe's current values — MAI restart
    /// casuali). On Solved the exit geometry is STORED to the
    /// recipe's float32 and VERIFIED on the stored bytes against the
    /// kernel's degenerate classes BEFORE the model's chains move
    /// (the refusal precedes the write); on the two honest failures
    /// the recipe is UNTOUCHED and the payload is the answer.
    SketchSolveResult solve() {
        using sketch_solve_detail::Assembled;
        using sketch_solve_detail::build_layout;
        using sketch_solve_detail::assemble;
        using sketch_solve_detail::load_params;
        using sketch_solve_detail::store_params;

        const auto L = build_layout(chains_);
        const std::size_t n = L.total;
        std::vector<double> x;
        load_params(chains_, L, x);

        const SketchBill b = bill();
        SketchSolveResult res;
        res.bill = b;

        // The bands vector per row — assembled ONCE (the row set is
        // x-independent; the labels/worlds ride with it).
        const Assembled proto = assemble(L, x, constraints_, chains_);
        std::vector<double> bands(proto.F.size());
        for (std::size_t i = 0; i < proto.F.size(); ++i)
            bands[i] = proto.angular[i] ? kSolveEpsAng : kSolveEpsLin;

        // The honest guard D2.1's own solved predicate demands: a
        // geometry that already holds every constraint within its
        // bands IS solved (the empty-constraint sketch — every
        // pre-solver sketch in every file — is the extreme case; the
        // propagation world survives the landing day by THIS clause,
        // never by an iteration's accident).
        {
            bool all_in_band = true;
            for (std::size_t i = 0; i < proto.F.size(); ++i)
                if (!(std::abs(proto.F[i]) <= bands[i])) { all_in_band = false; break; }
            if (all_in_band) {
                res.status = SketchSolveStatus::Solved;
                res.iterations = 0;
                res.chains = chains_;
                return res;
            }
        }

        // --- the damped min-norm Newton (D2.2, the oracle's own loop) ---
        double lam = kSolveLam0;
        int stall = 0;
        const char* how = "capped";
        int it_done = kSolveMaxIter;

        auto weighted_norm = [](const std::vector<double>& F,
                                const std::vector<double>& bandsv) {
            double s = 0.0;
            for (std::size_t i = 0; i < F.size(); ++i) {
                const double w = F[i] / bandsv[i];
                s += w * w;
            }
            return std::sqrt(s);
        };

        for (int it = 1; it <= kSolveMaxIter; ++it) {
            const Assembled A = assemble(L, x, constraints_, chains_);
            const double nrm = weighted_norm(A.F, bands);

            // the inner loop: try/reject, λ escalates on failure —
            // the UNSCALED algebra (the bands never enter it)
            std::vector<double> delta;
            bool step_ok = false;
            while (true) {
                // A_sys = JᵀJ + λI (n×n), rhs = −JᵀF
                std::vector<double> Asys(n * n, 0.0);
                std::vector<double> rhs(n, 0.0);
                const std::size_t rows = A.F.size();
                for (std::size_t r = 0; r < rows; ++r) {
                    const double* Jr = &A.J[r * n];
                    const double Fr = A.F[r];
                    for (std::size_t i = 0; i < n; ++i) {
                        rhs[i] -= Jr[i] * Fr;
                        if (Jr[i] != 0.0)
                            for (std::size_t j = 0; j < n; ++j)
                                Asys[i * n + j] += Jr[i] * Jr[j];
                    }
                }
                for (std::size_t i = 0; i < n; ++i) Asys[i * n + i] += lam;

                // Cholesky first (the SPD discipline); the deterministic
                // LU fallback when the SPD property is lost to rounding
                // at the λ floor's ill-conditioning (nonsingular: the
                // no-step path only on an exactly-zero pivot).
                std::vector<double> trial = rhs;
                std::vector<double> work = Asys;
                bool solved_ok = sketch_linalg::cholesky_solve(work, trial, n);
                if (!solved_ok) {
                    trial = rhs;
                    work = Asys;
                    solved_ok = sketch_linalg::lu_solve(work, trial, n);
                }
                if (solved_ok) {
                    delta = trial;
                    std::vector<double> x_try = x;
                    for (std::size_t i = 0; i < n; ++i) x_try[i] += delta[i];
                    const Assembled At = assemble(L, x_try, constraints_, chains_);
                    step_ok = weighted_norm(At.F, bands) < nrm;
                    if (step_ok) {
                        x = std::move(x_try);
                        break;
                    }
                }
                if (lam >= kSolveLamCap) break;
                lam *= kSolveLamUp;
            }

            if (!step_ok) {
                // THE ROUNDING FLOOR, honestly named (the m2's own
                // finding, folded where the ADR's text already stood):
                // a full-rank system driven to machine precision can
                // exhaust the strict-decrease test BEFORE the step
                // floor — the weighted residual cannot strictly
                // decrease at the last ulp. The iterate that meets
                // EVERY band IS the solved state (D2.1's own
                // predicate: "the constraints hold exactly (within
                // their bands)") — never a failure wearing a class.
                // (The goldens never land here: their convergence
                // exits through the accepted path with margin; this
                // guard answers the rounding floor the honest way.)
                {
                    const Assembled Afloor = assemble(L, x, constraints_, chains_);
                    bool in_band = true;
                    for (std::size_t i = 0; i < Afloor.F.size(); ++i)
                        if (!(std::abs(Afloor.F[i]) <= bands[i])) {
                            in_band = false;
                            break;
                        }
                    if (in_band && !Afloor.F.empty()) {
                        it_done = it;
                        how = "solved";
                        break;
                    }
                }
                // no decreasing step at the λ cap: the STALL, honestly
                // counted (the window is the declared patience)
                ++stall;
                if (stall >= kSolveStallWindow) {
                    it_done = it;
                    how = "stalled";
                    break;
                }
                continue;
            }

            lam = std::max(lam / kSolveLamDown, kSolveLamFloor);

            const Assembled A2 = assemble(L, x, constraints_, chains_);
            bool band_ok = true;
            for (std::size_t i = 0; i < A2.F.size(); ++i)
                if (!(std::abs(A2.F[i]) <= bands[i])) { band_ok = false; break; }
            double dn = 0.0;
            for (const double v : delta) dn += v * v;
            const bool step_small = std::sqrt(dn) <= kSolveStepFloor;

            if (band_ok && step_small) {
                it_done = it;
                how = "solved";
                break;
            }
            if (step_small && !band_ok) {
                ++stall;
                if (stall >= kSolveStallWindow) {
                    it_done = it;
                    how = "stalled";
                    break;
                }
            } else {
                stall = 0;
            }
        }

        // the capped exit's own honesty: an iterate that spent the cap
        // but MEETS every band is solved by D2.1's predicate (the
        // wandering-in-band case); the out-of-band one keeps the class.
        if (std::string(how) == "capped") {
            const Assembled Acap = assemble(L, x, constraints_, chains_);
            bool in_band = !Acap.F.empty();
            for (std::size_t i = 0; i < Acap.F.size(); ++i)
                if (!(std::abs(Acap.F[i]) <= bands[i])) { in_band = false; break; }
            if (in_band) how = "solved";
        }

        res.iterations = it_done;

        if (std::string(how) == "solved") {
            // THE VERIFICATION ON THE STORED BYTES (D2.2): the solve's
            // doubles are written to the recipe's float32 (the store
            // IS the quantizer) and the kernel's own construction
            // checks run on what the recipe would hold — the refusal
            // PRECEDES the write (the model's chains never move).
            std::vector<SketchChain> stored = store_params(chains_, L, x);
            for (std::size_t c = 0; c < stored.size(); ++c) {
                for (std::size_t e = 0; e < stored[c].entities.size(); ++e) {
                    try {
                        validate_sketch_entity(stored[c].entities[e]);
                    } catch (const std::invalid_argument& ex) {
                        res.status = SketchSolveStatus::Degenerate;
                        res.chains = chains_; // the recipe untouched
                        res.exit_params = x;  // the doubles that degenerated
                        res.degenerate_entity =
                            "chain " + std::to_string(c) + ", entity " +
                            std::to_string(e);
                        res.degenerate_message = ex.what(); // VERBATIM
                        return res;
                    }
                }
            }
            res.status = SketchSolveStatus::Solved;
            res.exit_params = x; // the pre-store doubles (the diagnostic margin's own numbers)
            res.chains = stored;
            chains_ = std::move(stored); // the write — after the green
            return res;
        }

        // --- sketch_not_solvable: the payload IS the answer ---
        res.status = SketchSolveStatus::NotSolvable;
        const Assembled Aexit = assemble(L, x, constraints_, chains_);
        double fsq = 0.0;
        for (std::size_t i = 0; i < Aexit.F.size(); ++i) {
            SketchViolation v;
            v.constraint = Aexit.owners[i];
            v.label = Aexit.labels[i];
            v.value = Aexit.F[i];
            v.angular = Aexit.angular[i];
            res.violations.push_back(v);
            fsq += Aexit.F[i] * Aexit.F[i];
        }
        res.residual_norm = std::sqrt(fsq);
        res.largest = 0;
        for (std::size_t i = 1; i < res.violations.size(); ++i)
            if (std::abs(res.violations[i].value) >
                std::abs(res.violations[res.largest].value))
                res.largest = i;
        res.exit_params = x;
        res.chains = chains_; // the recipe untouched
        res.diagnosis = diagnose_(Aexit);
        return res;
    }

private:
    // The structural validation (the FACTORIES validated the value
    // rules; the MODEL validates the operands against the real
    // chains — the kinds must match the constraint's own table).
    void validate_operand_kinds_(const SketchConstraint& c) const {
        auto kind_of = [this](SketchEntityRef r) -> SketchEntity::Kind {
            if (r.chain >= chains_.size())
                throw std::invalid_argument(
                    "the sketch solver: the operand's chain " +
                    std::to_string(r.chain) + " is out of range (the "
                    "sketch has " + std::to_string(chains_.size()) +
                    ") — the constraint binds real entities. Remedy: "
                    "address an existing chain.");
            const auto& ents = chains_[r.chain].entities;
            if (r.entity >= ents.size())
                throw std::invalid_argument(
                    "the sketch solver: the operand's entity " +
                    std::to_string(r.entity) + " is out of range (chain " +
                    std::to_string(r.chain) + " has " +
                    std::to_string(ents.size()) +
                    ") — the constraint binds real entities. Remedy: "
                    "address an existing entity.");
            return ents[r.entity].kind;
        };
        auto require_point = [&](SketchPointRef p, const char* what) {
            if (kind_of(p.entity) == SketchEntity::Kind::Circle)
                throw std::invalid_argument(
                    std::string(what) +
                    ": a circle has no endpoints — the point operands "
                    "bind segment and arc endpoints (the arc's are "
                    "derived, the trig-carrying branch). Remedy: bind a "
                    "segment or an arc, or pin the circle's center "
                    "fields.");
        };
        auto require_segment = [&](SketchEntityRef r, const char* what) {
            if (kind_of(r) != SketchEntity::Kind::Segment)
                throw std::invalid_argument(
                    std::string(what) +
                    ": the operand must be a SEGMENT — the direction "
                    "equations consume the segment's unit direction. "
                    "Remedy: bind a segment.");
        };
        switch (c.kind) {
        case SketchConstraint::Kind::Coincidence:
            require_point(c.a, "the coincidence");
            require_point(c.b, "the coincidence");
            break;
        case SketchConstraint::Kind::Distance:
            require_point(c.a, "the distance");
            require_point(c.b, "the distance");
            break;
        case SketchConstraint::Kind::Parallel:
            require_segment(c.ea, "the parallel");
            require_segment(c.eb, "the parallel");
            break;
        case SketchConstraint::Kind::Perpendicular:
            require_segment(c.ea, "the perpendicular");
            require_segment(c.eb, "the perpendicular");
            break;
        case SketchConstraint::Kind::Tangent:
            require_segment(c.ea, "the tangent");
            if (kind_of(c.eb) != SketchEntity::Kind::Circle)
                throw std::invalid_argument(
                    "the tangent: the second operand must be a CIRCLE "
                    "(v1's table: linea × cerchio — the arc's tangent "
                    "is the queued family, its first consumer "
                    "registered). Remedy: bind a circle.");
            break;
        case SketchConstraint::Kind::Length:
            require_segment(c.ea, "the length");
            break;
        case SketchConstraint::Kind::Radius:
            if (kind_of(c.ea) == SketchEntity::Kind::Segment)
                throw std::invalid_argument(
                    "the radius: the operand must be a CIRCLE or an ARC "
                    "— a segment has no radius. Remedy: bind a circular "
                    "entity, or use the length on the segment.");
            break;
        case SketchConstraint::Kind::Pin: {
            const auto k = kind_of(c.pin.entity);
            if (c.pin.field >= sketch_entity_dof(k))
                throw std::invalid_argument(
                    "the pin: the field " +
                    std::to_string(c.pin.field) + " is out of range for "
                    "the entity's kind (it carries " +
                    std::to_string(sketch_entity_dof(k)) +
                    " parameters). Remedy: address a real field.");
            break;
        }
        }
    }

    // THE HONEST DIAGNOSIS (D2.2): derived, never a template. The one
    // family the derivation reads from the recipe's own shape is the
    // closed three-segment triangle with three lengths — D5.6's own
    // case: the violated inequality names itself with its numbers.
    // Every other shape gets the honest generic sentence: the largest
    // violation named, the bounded exit declared for what it is.
    std::string diagnose_(const sketch_solve_detail::Assembled& A) const {
        auto fmt = [](double v) {
            if (v == std::floor(v) && std::abs(v) < 1e15) {
                char buf[32];
                std::snprintf(buf, sizeof(buf), "%lld",
                              static_cast<long long>(v));
                return std::string(buf);
            }
            char buf[32];
            std::snprintf(buf, sizeof(buf), "%.6g", v);
            return std::string(buf);
        };

        // the triangle family: one chain, three segments, closed by
        // three coincidences (each segment's both endpoints used
        // exactly once across them), three lengths (one per segment)
        if (chains_.size() == 1 && chains_[0].entities.size() == 3) {
            bool all_seg = true;
            for (const auto& e : chains_[0].entities)
                if (e.kind != SketchEntity::Kind::Segment) all_seg = false;
            std::size_t ncoin = 0, nlen = 0;
            bool endpoints_ok = true;
            bool len_seen[3] = {false, false, false};
            double side[3] = {0, 0, 0};
            int end_use[3][2] = {{0, 0}, {0, 0}, {0, 0}};
            for (const auto& c : constraints_) {
                if (c.kind == SketchConstraint::Kind::Coincidence) {
                    ++ncoin;
                    if (c.a.entity.chain != 0 || c.b.entity.chain != 0 ||
                        c.a.entity.entity > 2 || c.b.entity.entity > 2)
                        endpoints_ok = false;
                    else {
                        ++end_use[c.a.entity.entity]
                                 [c.a.end == SketchEnd::End ? 1 : 0];
                        ++end_use[c.b.entity.entity]
                                 [c.b.end == SketchEnd::End ? 1 : 0];
                    }
                } else if (c.kind == SketchConstraint::Kind::Length) {
                    ++nlen;
                    if (c.ea.chain != 0 || c.ea.entity > 2) {
                        endpoints_ok = false;
                    } else {
                        len_seen[c.ea.entity] = true;
                        side[c.ea.entity] = c.value;
                    }
                }
            }
            const bool cycle =
                all_seg && ncoin == 3 && endpoints_ok && nlen == 3 &&
                len_seen[0] && len_seen[1] && len_seen[2] &&
                end_use[0][0] == 1 && end_use[0][1] == 1 &&
                end_use[1][0] == 1 && end_use[1][1] == 1 &&
                end_use[2][0] == 1 && end_use[2][1] == 1;
            if (cycle) {
                const double a = side[0], b = side[1], cc = side[2];
                if (a + b < cc)
                    return fmt(a) + "+" + fmt(b) + " < " + fmt(cc) +
                           " — no triangle with these sides exists";
                if (a + cc < b)
                    return fmt(a) + "+" + fmt(cc) + " < " + fmt(b) +
                           " — no triangle with these sides exists";
                if (b + cc < a)
                    return fmt(b) + "+" + fmt(cc) + " < " + fmt(a) +
                           " — no triangle with these sides exists";
            }
        }

        // the generic honest diagnosis: the largest violation named
        std::size_t worst = 0;
        for (std::size_t i = 1; i < A.F.size(); ++i)
            if (std::abs(A.F[i]) > std::abs(A.F[worst])) worst = i;
        if (A.F.empty())
            return "the constraints cannot all hold together";
        return "the constraints cannot all hold together: the largest "
               "violation " +
               fmt(std::abs(A.F[worst])) + " on " + A.labels[worst] +
               "; the values at the exit are the damped least-squares "
               "closest approach (bounded), never a solution. Remedy: "
               "loosen a dimension or remove a constraint";
    }

    std::vector<SketchChain> chains_;
    std::vector<SketchConstraint> constraints_;
};

} // namespace CAD_0::sdf
