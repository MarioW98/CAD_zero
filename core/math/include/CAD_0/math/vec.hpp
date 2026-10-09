// core/math/include/CAD_0/math/vec.hpp
//
// Vec2/Vec3/Vec4 — minimal, header-only, SIMD-friendly linear algebra.
//
// Design notes (see ADR-0001):
//   * Value type — copyable, no virtual functions, no dynamic allocation.
//   * Layout-compatible with float[N] so we can reinterpret_cast to xsimd
//     batches when evaluating SDF fields in bulk (see core/sdf/evaluate.hpp).
//   * No operator overloads that hide cost: dot/cross/normalize are named
//     functions, not operators.
//
#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace CAD_0::math {

template<std::size_t N, typename T = float>
struct Vec;

// --- Vec2 -----------------------------------------------------------------
template<typename T>
struct Vec<2, T> {
    using value_type = T;
    static constexpr std::size_t kDim = 2;

    T x{};
    T y{};

    constexpr Vec() = default;
    constexpr Vec(T x_, T y_) noexcept : x(x_), y(y_) {}

    constexpr T&       operator[](std::size_t i)       { return (i == 0) ? x : y; }
    constexpr const T& operator[](std::size_t i) const { return (i == 0) ? x : y; }

    constexpr Vec operator+(const Vec& o) const noexcept { return {x + o.x, y + o.y}; }
    constexpr Vec operator-(const Vec& o) const noexcept { return {x - o.x, y - o.y}; }
    constexpr Vec operator*(T s)              const noexcept { return {x * s, y * s}; }
    constexpr Vec operator/(T s)              const noexcept { return {x / s, y / s}; }

    constexpr Vec& operator+=(const Vec& o) noexcept { x += o.x; y += o.y; return *this; }
    constexpr Vec& operator-=(const Vec& o) noexcept { x -= o.x; y -= o.y; return *this; }
    constexpr Vec& operator*=(T s)         noexcept { x *= s; y *= s; return *this; }
    constexpr Vec& operator/=(T s)         noexcept { x /= s; y /= s; return *this; }

    constexpr bool operator==(const Vec& o) const noexcept = default;
    constexpr bool operator!=(const Vec& o) const noexcept = default;

    constexpr T dot(const Vec& o)   const noexcept { return x * o.x + y * o.y; }
    constexpr T length_sq()         const noexcept { return dot(*this); }
    T           length()            const noexcept { return std::sqrt(length_sq()); }
    Vec         normalized()        const noexcept;
};

// --- Vec3 -----------------------------------------------------------------
template<typename T>
struct Vec<3, T> {
    using value_type = T;
    static constexpr std::size_t kDim = 3;

    T x{};
    T y{};
    T z{};

    constexpr Vec() = default;
    constexpr Vec(T x_, T y_, T z_) noexcept : x(x_), y(y_), z(z_) {}

    constexpr T&       operator[](std::size_t i)       {
        return (i == 0) ? x : (i == 1) ? y : z;
    }
    constexpr const T& operator[](std::size_t i) const {
        return (i == 0) ? x : (i == 1) ? y : z;
    }

    constexpr Vec operator+(const Vec& o) const noexcept { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vec operator-(const Vec& o) const noexcept { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vec operator*(T s)              const noexcept { return {x * s, y * s, z * s}; }
    constexpr Vec operator/(T s)              const noexcept { return {x / s, y / s, z / s}; }
    constexpr Vec operator-()                const noexcept { return {-x, -y, -z}; }

    constexpr Vec& operator+=(const Vec& o) noexcept { x += o.x; y += o.y; z += o.z; return *this; }
    constexpr Vec& operator-=(const Vec& o) noexcept { x -= o.x; y -= o.y; z -= o.z; return *this; }
    constexpr Vec& operator*=(T s)         noexcept { x *= s; y *= s; z *= s; return *this; }
    constexpr Vec& operator/=(T s)         noexcept { x /= s; y /= s; z /= s; return *this; }

    constexpr bool operator==(const Vec& o) const noexcept = default;
    constexpr bool operator!=(const Vec& o) const noexcept = default;

    constexpr T dot(const Vec& o)   const noexcept { return x * o.x + y * o.y + z * o.z; }
    constexpr Vec cross(const Vec& o) const noexcept {
        return {y * o.z - z * o.y,
                z * o.x - x * o.z,
                x * o.y - y * o.x};
    }
    constexpr T length_sq()         const noexcept { return dot(*this); }
    T           length()            const noexcept { return std::sqrt(length_sq()); }
    Vec         normalized()        const noexcept;
};

// --- Vec4 (homogeneous / quaternion storage) ------------------------------
template<typename T>
struct Vec<4, T> {
    using value_type = T;
    static constexpr std::size_t kDim = 4;
    T x{}, y{}, z{}, w{};

    constexpr Vec() = default;
    constexpr Vec(T x_, T y_, T z_, T w_) noexcept : x(x_), y(y_), z(z_), w(w_) {}

    constexpr T&       operator[](std::size_t i)       {
        return (i == 0) ? x : (i == 1) ? y : (i == 2) ? z : w;
    }
    constexpr const T& operator[](std::size_t i) const {
        return (i == 0) ? x : (i == 1) ? y : (i == 2) ? z : w;
    }

    constexpr Vec operator+(const Vec& o) const noexcept { return {x + o.x, y + o.y, z + o.z, w + o.w}; }
    constexpr Vec operator-(const Vec& o) const noexcept { return {x - o.x, y - o.y, z - o.z, w - o.w}; }
    constexpr Vec operator*(T s)              const noexcept { return {x * s, y * s, z * s, w * s}; }
    constexpr Vec operator/(T s)              const noexcept { return {x / s, y / s, z / s, w / s}; }
    constexpr Vec operator-()                const noexcept { return {-x, -y, -z, -w}; }

    constexpr Vec& operator+=(const Vec& o) noexcept { x += o.x; y += o.y; z += o.z; w += o.w; return *this; }
    constexpr Vec& operator-=(const Vec& o) noexcept { x -= o.x; y -= o.y; z -= o.z; w -= o.w; return *this; }
    constexpr Vec& operator*=(T s)         noexcept { x *= s; y *= s; z *= s; w *= s; return *this; }
    constexpr Vec& operator/=(T s)         noexcept { x /= s; y /= s; z /= s; w /= s; return *this; }

    constexpr bool operator==(const Vec& o) const noexcept = default;
    constexpr bool operator!=(const Vec& o) const noexcept = default;
};

// --- Aliases ---------------------------------------------------------------
using Vec2f = Vec<2, float>;
using Vec3f = Vec<3, float>;
using Vec4f = Vec<4, float>;
using Vec2d = Vec<2, double>;
using Vec3d = Vec<3, double>;
using Vec4d = Vec<4, double>;

// --- Free functions --------------------------------------------------------
template<std::size_t N, typename T>
constexpr T dot(const Vec<N, T>& a, const Vec<N, T>& b) noexcept { return a.dot(b); }

template<typename T>
constexpr Vec<3, T> cross(const Vec<3, T>& a, const Vec<3, T>& b) noexcept { return a.cross(b); }

template<std::size_t N, typename T>
constexpr T length_sq(const Vec<N, T>& v) noexcept { return v.length_sq(); }

template<std::size_t N, typename T>
T length(const Vec<N, T>& v) noexcept { return v.length(); }

template<std::size_t N, typename T>
Vec<N, T> normalize(const Vec<N, T>& v) noexcept { return v.normalized(); }

template<std::size_t N, typename T>
constexpr Vec<N, T> lerp(const Vec<N, T>& a, const Vec<N, T>& b, T t) noexcept {
    return a * (T{1} - t) + b * t;
}

// --- Implementation of normalized() ----------------------------------------
template<typename T>
Vec<2, T> Vec<2, T>::normalized() const noexcept {
    const T len = length();
    return (len > std::numeric_limits<T>::epsilon()) ? (*this / len) : *this;
}
template<typename T>
Vec<3, T> Vec<3, T>::normalized() const noexcept {
    const T len = length();
    return (len > std::numeric_limits<T>::epsilon()) ? (*this / len) : *this;
}

} // namespace CAD_0::math
