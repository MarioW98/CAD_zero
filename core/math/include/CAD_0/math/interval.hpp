// core/math/include/CAD_0/math/interval.hpp
//
// Closed/open intervals for ray-SDF intersection and Bvh construction.
//
#pragma once

#include <algorithm>
#include <limits>

namespace CAD_0::math {

template<typename T>
struct Interval {
    T lo{};
    T hi{};

    constexpr Interval() = default;
    constexpr Interval(T lo_, T hi_) noexcept : lo(lo_), hi(hi_) {
        if (lo > hi) std::swap(lo, hi);
    }

    constexpr bool empty()       const noexcept { return lo > hi; }
    constexpr bool contains(T v) const noexcept { return v >= lo && v <= hi; }
    constexpr T    length()      const noexcept { return hi - lo; }
    constexpr T    center()      const noexcept { return (lo + hi) * T{0.5}; }

    constexpr Interval intersect(const Interval& o) const noexcept {
        return {std::max(lo, o.lo), std::min(hi, o.hi)};
    }

    constexpr Interval merged(const Interval& o) const noexcept {
        return {std::min(lo, o.lo), std::max(hi, o.hi)};
    }

    static constexpr Interval empty_interval() noexcept {
        return {std::numeric_limits<T>::infinity(), -std::numeric_limits<T>::infinity()};
    }
    static constexpr Interval universe() noexcept {
        return {-std::numeric_limits<T>::infinity(), std::numeric_limits<T>::infinity()};
    }
};

using Intervalf = Interval<float>;
using Intervald = Interval<double>;

} // namespace CAD_0::math
