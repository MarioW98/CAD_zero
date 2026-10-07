// core/math/include/cadforge/math/quat.hpp
//
// Unit quaternion for compact rotation representation.
//
#pragma once

#include "cadforge/math/vec.hpp"
#include "cadforge/math/mat.hpp"

#include <cmath>

namespace cadforge::math {

template<typename T>
struct Quat {
    T w{}, x{}, y{}, z{};  // (cos(theta/2), sin(theta/2)*axis)

    constexpr Quat() = default;
    constexpr Quat(T w_, T x_, T y_, T z_) noexcept : w(w_), x(x_), y(y_), z(z_) {}

    static Quat identity() noexcept { return {T{1}, T{0}, T{0}, T{0}}; }

    // Build from axis (must be normalized) and angle in radians.
    static Quat from_axis_angle(const Vec<3, T>& axis, T rad) noexcept {
        const T half = rad * T{0.5};
        const T s = std::sin(half);
        return {std::cos(half), axis.x * s, axis.y * s, axis.z * s};
    }

    Quat operator*(const Quat& o) const noexcept {
        return {
            w * o.w - x * o.x - y * o.y - z * o.z,
            w * o.x + x * o.w + y * o.z - z * o.y,
            w * o.y - x * o.z + y * o.w + z * o.x,
            w * o.z + x * o.y - y * o.x + z * o.w
        };
    }

    Quat conjugate() const noexcept { return {w, -x, -y, -z}; }
    Quat normalized() const noexcept {
        const T n = std::sqrt(w*w + x*x + y*y + z*z);
        return (n > std::numeric_limits<T>::epsilon())
            ? Quat{w/n, x/n, y/n, z/n} : *this;
    }

    // Rotate a vector by this quaternion.
    Vec<3, T> rotate(const Vec<3, T>& v) const noexcept {
        const Quat qv{T{0}, v.x, v.y, v.z};
        const Quat r = (*this) * qv * conjugate();
        return {r.x, r.y, r.z};
    }

    // Convert to a 4x4 rotation matrix.
    Mat<4, 4, T> to_matrix() const noexcept {
        const T xx = x * x, yy = y * y, zz = z * z;
        const T xy = x * y, xz = x * z, yz = y * z;
        const T wx = w * x, wy = w * y, wz = w * z;
        Mat<4, 4, T> m = Mat<4, 4, T>::identity();
        m.cols[0].x = T{1} - T{2} * (yy + zz);
        m.cols[0].y = T{2} * (xy + wz);
        m.cols[0].z = T{2} * (xz - wy);
        m.cols[1].x = T{2} * (xy - wz);
        m.cols[1].y = T{1} - T{2} * (xx + zz);
        m.cols[1].z = T{2} * (yz + wx);
        m.cols[2].x = T{2} * (xz + wy);
        m.cols[2].y = T{2} * (yz - wx);
        m.cols[2].z = T{1} - T{2} * (xx + yy);
        return m;
    }
};

using Quatf = Quat<float>;
using Quatd = Quat<double>;

} // namespace cadforge::math
