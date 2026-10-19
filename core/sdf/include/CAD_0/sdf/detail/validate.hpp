// core/sdf/include/CAD_0/sdf/detail/validate.hpp
//
// Construction-time input validation for SDF nodes — ADR-0029 D5.
//
// Every node validates its own inputs at construction and throws
// std::invalid_argument naming the operator, the parameter, its value
// and the supported class. Construction is the single choke point
// (C++ API, Python bindings, native loader, property generators, UI
// all pass through it) and it is cold path — once per edit, never per
// sample — so the branches are free. sample() stays noexcept: no
// hot-path change.
//
// The principle (ADR-0029 D5, minimal list): close the door on the
// contract-breaking class — non-finite, sign-flipping,
// degenerate-empty, null — and do NOT legislate taste (a = b capsules
// stay legal behind their explicit guard; spindle tori r > R stay
// legal). No check beyond the D5 table.
//
// Message style: the bridge style (ADR-0026 D2) — diagnosis + remedy,
// never a bare assert:
//
//   "<op>: <param> must <supported class>, got <value> — <diagnosis>.
//    Remedy: <remedy>."
//
// The exception family joins the native loader's ADR-0007 explicit
// errors (a bad parameter in a file is a fail-fast load error) and
// surfaces in Python as ValueError with the complete message.
//
// This header is a private implementation detail of the public SDF
// headers (primitives/operators/transforms include it), so it ships
// with them; it is not API on its own and may evolve without notice.
#pragma once

#include "CAD_0/math/vec.hpp"
#include "CAD_0/math/quat.hpp"

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace CAD_0::sdf::detail {

// RotateSDF unit-norm tolerance (ADR-0029 D5): serialization
// round-trips quaternions at ~1e-9, so 1e-6 accepts every honest
// round-trip while still catching rotates-that-scale.
inline constexpr float kQuatUnitTol = 1e-6f;

// Assemble and throw the bridge-style message.
[[noreturn]] inline void throw_invalid(const char* op, const std::string& param,
                                       const char* must, const std::string& got,
                                       const char* diagnosis, const char* remedy) {
    throw std::invalid_argument(
        std::string(op) + ": " + param + " must " + must + ", got " + got +
        " — " + diagnosis + " Remedy: " + remedy + ".");
}

inline std::string format_vec3(const math::Vec3f& v) {
    return "(" + std::to_string(v.x) + ", " + std::to_string(v.y) + ", " +
           std::to_string(v.z) + ")";
}

// Scalar must be finite and strictly positive. The negated form
// matters: NaN fails `v > 0` and +inf passes it, so finiteness is an
// explicit arm ("all imply finite" — the D5 table header).
inline void require_positive(float v, const char* op, const char* param,
                             const char* diagnosis, const char* remedy) {
    if (!std::isfinite(v) || v <= 0.0f)
        throw_invalid(op, param, "be finite and > 0", std::to_string(v),
                      diagnosis, remedy);
}

// Scalar must be finite and non-negative (SmoothMinSDF k: k = 0 is the
// documented hard min and stays legal; k < 0 is rejected because the
// polynomial derivation assumes k > 0).
inline void require_non_negative(float v, const char* op, const char* param,
                                 const char* diagnosis, const char* remedy) {
    if (!std::isfinite(v) || v < 0.0f)
        throw_invalid(op, param, "be finite and >= 0", std::to_string(v),
                      diagnosis, remedy);
}

// Scalar must be finite, any sign (TwistSDF k: k = 0 is the identity
// warp and stays legal).
inline void require_finite(float v, const char* op, const char* param,
                           const char* diagnosis, const char* remedy) {
    if (!std::isfinite(v))
        throw_invalid(op, param, "be finite (any sign)", std::to_string(v),
                      diagnosis, remedy);
}

// Vector must be finite on every axis (capsule endpoints, translate
// offset). Reports the whole vector so the offending component is
// visible in context.
inline void require_finite_vec3(const math::Vec3f& v, const char* op,
                                const char* param, const char* diagnosis,
                                const char* remedy) {
    if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z))
        throw_invalid(op, param, "be finite on every axis", format_vec3(v),
                      diagnosis, remedy);
}

// Vector must be finite and strictly positive on every axis (BoxSDF
// extent). Names the first offending axis.
inline void require_positive_vec3(const math::Vec3f& v, const char* op,
                                  const char* param, const char* diagnosis,
                                  const char* remedy) {
    const char* axis = nullptr;
    float bad = 0.0f;
    if (!std::isfinite(v.x) || v.x <= 0.0f) { axis = ".x"; bad = v.x; }
    else if (!std::isfinite(v.y) || v.y <= 0.0f) { axis = ".y"; bad = v.y; }
    else if (!std::isfinite(v.z) || v.z <= 0.0f) { axis = ".z"; bad = v.z; }
    if (axis)
        throw_invalid(op, std::string(param) + axis, "be finite and > 0",
                      std::to_string(bad) + " (extent " + format_vec3(v) + ")",
                      diagnosis, remedy);
}

// Quaternion must have unit norm within kQuatUnitTol. The negated
// comparison catches NaN components (NaN <= tol is false).
inline void require_unit_quat(const math::Quatf& q, const char* op,
                              const char* diagnosis, const char* remedy) {
    const float n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (!(std::abs(n - 1.0f) <= kQuatUnitTol))
        throw_invalid(op, "quaternion q", "have unit norm within 1e-6",
                      std::to_string(n), diagnosis, remedy);
}

// Child pointer must be non-null (operators and transforms).
template <class T>
inline void require_non_null(const std::unique_ptr<T>& p, const char* op,
                             const std::string& param, const char* diagnosis,
                             const char* remedy) {
    if (!p)
        throw_invalid(op, param, "be a constructed (non-null) SDF child",
                      "null", diagnosis, remedy);
}

} // namespace CAD_0::sdf::detail
