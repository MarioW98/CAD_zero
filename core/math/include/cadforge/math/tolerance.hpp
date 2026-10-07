// core/math/include/cadforge/math/tolerance.hpp
//
// Central tolerance model for cadforge. See ADR-0004.
//
// Key principles:
//   1. There is no single global epsilon. Each operation receives an
//      explicit Tolerance object, or fails.
//   2. Tolerances are tracked through the feature tree. The feature tree
//      exposes the *accumulated* tolerance for any subshape, which is
//      essential when bridging SDF and B-Rep.
//   3. Tolerances are unit-aware (mm, rad). Comparing lengths with a radian
//      tolerance is a compile error (TODO: strong types — for now, runtime
//      checks in DEBUG builds).
//
#pragma once

#include <cadforge/config.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

namespace cadforge::math {

enum class ToleranceKind : std::uint8_t {
    Linear,    // mm
    Angular,   // rad
    Relative,  // dimensionless ratio
};

// A tolerance value with explicit kind.
struct Tolerance {
    ToleranceKind kind{ToleranceKind::Linear};
    double value{};

    constexpr Tolerance() = default;
    constexpr Tolerance(ToleranceKind k, double v) noexcept : kind(k), value(v) {}

    // Factory helpers — values chosen to match STEP's default precision.
    static constexpr Tolerance linear(double mm = 1e-6) noexcept {
        return {ToleranceKind::Linear, mm};
    }
    static constexpr Tolerance angular(double rad = 1e-7) noexcept {
        return {ToleranceKind::Angular, rad};
    }
    static constexpr Tolerance relative(double r = 1e-9) noexcept {
        return {ToleranceKind::Relative, r};
    }

    constexpr bool is_linear()  const noexcept { return kind == ToleranceKind::Linear; }
    constexpr bool is_angular() const noexcept { return kind == ToleranceKind::Angular; }

    // Compare two lengths / angles with this tolerance.
    // is_linear assumed for lengths, is_angular for angles.
    bool almost_equal(double a, double b) const noexcept {
        return std::abs(a - b) <= value;
    }

    std::string describe() const;
};

// Default tolerances used when none is specified.
inline constexpr Tolerance kDefaultLinear  = Tolerance::linear();
inline constexpr Tolerance kDefaultAngular = Tolerance::angular();
inline constexpr Tolerance kDefaultRelative = Tolerance::relative();

// Floating-point comparison helpers.
template<typename T>
constexpr bool almost_equal(T a, T b, T eps = std::numeric_limits<T>::epsilon() * 32) noexcept {
    return std::abs(a - b) <= eps;
}

// Mixed-precision conversion loss tracker. Used when an SDF (float) is
// converted to a B-Rep (double) — the conversion may add a small bias.
struct ToleranceAccumulator {
    double linear{0.0};  // accumulated mm error
    double angular{0.0};  // accumulated rad error

    void add_linear(double mm) noexcept { linear += mm; }
    void add_angular(double rad) noexcept { angular += rad; }
    void add(const ToleranceAccumulator& o) noexcept {
        linear += o.linear;
        angular += o.angular;
    }
};

} // namespace cadforge::math
