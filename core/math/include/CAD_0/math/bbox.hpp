// core/math/include/CAD_0/math/bbox.hpp
//
// Axis-aligned bounding box (AABB). Used by:
//   * SDF evaluation grids (bounds culling)
//   * BVH for ray-SDF intersection
//   * B-Rep face tessellation culling
//
#pragma once

#include "CAD_0/math/vec.hpp"
#include "CAD_0/math/interval.hpp"

#include <cmath>
#include <limits>

namespace CAD_0::math {

template<typename T>
struct Bbox {
    Vec<3, T> min{ std::numeric_limits<T>::infinity(),
                   std::numeric_limits<T>::infinity(),
                   std::numeric_limits<T>::infinity()};
    Vec<3, T> max{-std::numeric_limits<T>::infinity(),
                  -std::numeric_limits<T>::infinity(),
                  -std::numeric_limits<T>::infinity()};

    constexpr Bbox() = default;
    constexpr Bbox(const Vec<3, T>& lo, const Vec<3, T>& hi) noexcept
        : min(lo), max(hi) {}

    constexpr bool empty() const noexcept {
        return min.x > max.x || min.y > max.y || min.z > max.z;
    }

    constexpr Vec<3, T> center() const noexcept {
        return {(min.x + max.x) * T{0.5},
                (min.y + max.y) * T{0.5},
                (min.z + max.z) * T{0.5}};
    }

    constexpr Vec<3, T> extent() const noexcept { return max - min; }

    constexpr T radius() const noexcept {
        const auto e = extent();
        const T m = std::max({e.x, e.y, e.z});
        return m * T{0.5};
    }

    void expand(const Vec<3, T>& p) noexcept {
        min.x = std::min(min.x, p.x); min.y = std::min(min.y, p.y); min.z = std::min(min.z, p.z);
        max.x = std::max(max.x, p.x); max.y = std::max(max.y, p.y); max.z = std::max(max.z, p.z);
    }

    void expand(const Bbox& o) noexcept {
        if (o.empty()) return;
        expand(o.min);
        expand(o.max);
    }

    // Intersect with another bbox. Returns a (possibly empty) bbox that
    // is the intersection of the two.
    Bbox intersect(const Bbox& o) const noexcept {
        Bbox r;
        if (empty() || o.empty()) return r;
        r.min.x = std::max(min.x, o.min.x);
        r.min.y = std::max(min.y, o.min.y);
        r.min.z = std::max(min.z, o.min.z);
        r.max.x = std::min(max.x, o.max.x);
        r.max.y = std::min(max.y, o.max.y);
        r.max.z = std::min(max.z, o.max.z);
        if (r.min.x > r.max.x || r.min.y > r.max.y || r.min.z > r.max.z) {
            // No intersection — return empty
            return Bbox{};
        }
        return r;
    }

    // Distance from a point to the box surface. Negative inside.
    T distance_to(const Vec<3, T>& p) const noexcept {
        const Vec<3, T> d = {
            std::max(std::max(min.x - p.x, p.x - max.x), T{0}),
            std::max(std::max(min.y - p.y, p.y - max.y), T{0}),
            std::max(std::max(min.z - p.z, p.z - max.z), T{0}),
        };
        return d.length();
    }

    bool contains(const Vec<3, T>& p) const noexcept {
        return p.x >= min.x && p.x <= max.x
            && p.y >= min.y && p.y <= max.y
            && p.z >= min.z && p.z <= max.z;
    }

    // Slab-based ray-AABB intersection. Returns the hit interval [tmin,tmax].
    Interval<T> intersect_ray(const Vec<3, T>& origin,
                              const Vec<3, T>& inv_dir) const noexcept {
        // Component-wise: t0 = (min - origin) * inv_dir  (per-axis)
        const Vec<3, T> t0{
            (min.x - origin.x) * inv_dir.x,
            (min.y - origin.y) * inv_dir.y,
            (min.z - origin.z) * inv_dir.z,
        };
        const Vec<3, T> t1{
            (max.x - origin.x) * inv_dir.x,
            (max.y - origin.y) * inv_dir.y,
            (max.z - origin.z) * inv_dir.z,
        };
        const Vec<3, T> tmin{std::min(t0.x, t1.x), std::min(t0.y, t1.y), std::min(t0.z, t1.z)};
        const Vec<3, T> tmax{std::max(t0.x, t1.x), std::max(t0.y, t1.y), std::max(t0.z, t1.z)};
        const T lo = std::max({tmin.x, tmin.y, tmin.z});
        const T hi = std::min({tmax.x, tmax.y, tmax.z});
        return {lo, hi};
    }

    static Bbox empty_bbox() noexcept { return Bbox{}; }
    static Bbox universe() noexcept {
        Bbox b;
        b.min = Vec<3, T>{-std::numeric_limits<T>::infinity(),
                         -std::numeric_limits<T>::infinity(),
                         -std::numeric_limits<T>::infinity()};
        b.max = Vec<3, T>{ std::numeric_limits<T>::infinity(),
                           std::numeric_limits<T>::infinity(),
                           std::numeric_limits<T>::infinity()};
        return b;
    }

    // True if this bbox is the universe (infinite extent on all axes).
    // Used by marching_cubes to detect open SDFs (e.g. PlaneSDF) and
    // skip the winding-fix post-process that only makes sense for
    // closed (bounded) surfaces.
    bool is_universe() const noexcept {
        return std::isinf(min.x) && std::isinf(min.y) && std::isinf(min.z) &&
               std::isinf(max.x) && std::isinf(max.y) && std::isinf(max.z);
    }
};

using Bboxf = Bbox<float>;
using Bboxd = Bbox<double>;

} // namespace CAD_0::math
