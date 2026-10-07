// core/math/include/cadforge/math/mat.hpp
//
// Mat4 — column-major 4x4 transform matrix, layout-compatible with OpenGL
// and Vulkan (with appropriate clip-space conversion).
//
#pragma once

#include "cadforge/math/vec.hpp"

#include <array>

namespace cadforge::math {

template<std::size_t M, std::size_t N, typename T = float>
struct Mat;

// Column-major 4x4 matrix.
template<typename T>
struct Mat<4, 4, T> {
    using value_type = T;
    static constexpr std::size_t kRows = 4;
    static constexpr std::size_t kCols = 4;

    // columns[0..3] = X, Y, Z, W basis vectors
    std::array<Vec<4, T>, 4> cols{};

    static constexpr Mat identity() noexcept {
        Mat m{};
        m.cols[0] = {T{1}, T{0}, T{0}, T{0}};
        m.cols[1] = {T{0}, T{1}, T{0}, T{0}};
        m.cols[2] = {T{0}, T{0}, T{1}, T{0}};
        m.cols[3] = {T{0}, T{0}, T{0}, T{1}};
        return m;
    }

    static constexpr Mat translation(const Vec<3, T>& t) noexcept {
        Mat m = identity();
        m.cols[3] = {t.x, t.y, t.z, T{1}};
        return m;
    }

    static Mat rotation_x(T rad) noexcept;
    static Mat rotation_y(T rad) noexcept;
    static Mat rotation_z(T rad) noexcept;
    static Mat scaling(const Vec<3, T>& s) noexcept;

    // Mat * Mat
    Mat operator*(const Mat& o) const noexcept;

    // Mat * Vec4 (point or direction)
    Vec<4, T> operator*(const Vec<4, T>& v) const noexcept;

    // Transform a point (assumes w=1)
    Vec<3, T> transform_point(const Vec<3, T>& p) const noexcept {
        const auto r = (*this) * Vec<4, T>{p.x, p.y, p.z, T{1}};
        return {r.x, r.y, r.z};
    }

    // Transform a direction (assumes w=0)
    Vec<3, T> transform_dir(const Vec<3, T>& d) const noexcept {
        const auto r = (*this) * Vec<4, T>{d.x, d.y, d.z, T{0}};
        return {r.x, r.y, r.z};
    }

    Mat transposed() const noexcept;
    Mat inverse() const noexcept; // for rigid transforms, use inverse_orthonormal
    Mat inverse_orthonormal() const noexcept; // assumes rotation+translation only
};

using Mat4f = Mat<4, 4, float>;
using Mat4d = Mat<4, 4, double>;

// --- Implementation --------------------------------------------------------

template<typename T>
Mat<4, 4, T> Mat<4, 4, T>::rotation_x(T rad) noexcept {
    const T c = std::cos(rad), s = std::sin(rad);
    Mat m = identity();
    m.cols[1].y = c;  m.cols[1].z = s;
    m.cols[2].y = -s; m.cols[2].z = c;
    return m;
}

template<typename T>
Mat<4, 4, T> Mat<4, 4, T>::rotation_y(T rad) noexcept {
    const T c = std::cos(rad), s = std::sin(rad);
    Mat m = identity();
    m.cols[0].x = c;  m.cols[0].z = -s;
    m.cols[2].x = s;  m.cols[2].z = c;
    return m;
}

template<typename T>
Mat<4, 4, T> Mat<4, 4, T>::rotation_z(T rad) noexcept {
    const T c = std::cos(rad), s = std::sin(rad);
    Mat m = identity();
    m.cols[0].x = c;  m.cols[0].y = s;
    m.cols[1].x = -s; m.cols[1].y = c;
    return m;
}

template<typename T>
Mat<4, 4, T> Mat<4, 4, T>::scaling(const Vec<3, T>& s) noexcept {
    Mat m = identity();
    m.cols[0].x = s.x;
    m.cols[1].y = s.y;
    m.cols[2].z = s.z;
    return m;
}

template<typename T>
Mat<4, 4, T> Mat<4, 4, T>::operator*(const Mat& o) const noexcept {
    Mat out{};
    for (std::size_t c = 0; c < 4; ++c) {
        Vec<4, T> acc{T{0}, T{0}, T{0}, T{0}};
        for (std::size_t i = 0; i < 4; ++i) {
            const T coeff = o.cols[c][i];
            acc = acc + cols[i] * coeff;
        }
        out.cols[c] = acc;
    }
    return out;
}

template<typename T>
Vec<4, T> Mat<4, 4, T>::operator*(const Vec<4, T>& v) const noexcept {
    return cols[0] * v.x + cols[1] * v.y + cols[2] * v.z + cols[3] * v.w;
}

template<typename T>
Mat<4, 4, T> Mat<4, 4, T>::transposed() const noexcept {
    Mat m{};
    for (std::size_t r = 0; r < 4; ++r)
        for (std::size_t c = 0; c < 4; ++c)
            m.cols[c][r] = cols[r][c];
    return m;
}

template<typename T>
Mat<4, 4, T> Mat<4, 4, T>::inverse_orthonormal() const noexcept {
    // For a rigid transform M = T * R (column-major: cols[3] = translation,
    // cols[0..2] = rotation columns), the inverse is M^-1(p) = R^T * (p - t).
    //
    // Equivalently: M^-1(p) = R^T * p + (-R^T * t).
    //
    // Column-major convention:
    //   cols[c] = c-th column of M
    //   cols[c].x = M[0][c], cols[c].y = M[1][c], ...
    //
    // For rotation R (cols[0..2] are rotation columns):
    //   R[i][k] = cols[k][i]  (column k, component i)
    //
    // Transpose:
    //   R^T[k][i] = R[i][k] = cols[k][i]
    // So the columns of R^T are exactly the rows of R, which (in column-major)
    // are accessed by reading components across the original cols[0..2]:
    //   R^T col 0 = (R[0][0], R[1][0], R[2][0]) = (cols[0].x, cols[0].y, cols[0].z)
    //
    // Wait — that's just cols[0]! R^T col 0 = R row 0 = (R[0][0], R[1][0], R[2][0])
    // = (cols[0].x, cols[0].y, cols[0].z) = original cols[0]. So R^T's columns
    // are the same as R's columns. This can't be right.
    //
    // Let's redo: R^T's row k = R's column k. So R^T's *column* k = R's *row* k.
    // R's row k = (R[k][0], R[k][1], R[k][2]) = (cols[0][k], cols[1][k], cols[2][k]).
    //
    // So R^T's column k = (cols[0][k], cols[1][k], cols[2][k]).
    //
    // In column-major form, inv.cols[k] = R^T's column k:
    //   inv.cols[0] = (cols[0][0], cols[1][0], cols[2][0]) = (cols[0].x, cols[1].x, cols[2].x)
    //   inv.cols[1] = (cols[0][1], cols[1][1], cols[2][1]) = (cols[0].y, cols[1].y, cols[2].y)
    //   inv.cols[2] = (cols[0][2], cols[1][2], cols[2][2]) = (cols[0].z, cols[1].z, cols[2].z)
    //
    // For -R^T * t, we want each component k of the result:
    //   (R^T * t)[k] = sum_i R^T[k][i] * t[i] = sum_i R[i][k] * t[i]
    //                = sum_i cols[k][i] * t[i]
    //                = cols[k].x * t.x + cols[k].y * t.y + cols[k].z * t.z
    //
    // (Note: cols[k][i] means column k, component i. With our Vec3 indexing,
    // cols[k].x = component 0, cols[k].y = component 1, cols[k].z = component 2.)
    Mat inv = Mat::identity();
    inv.cols[0].x = cols[0].x;  inv.cols[0].y = cols[1].x;  inv.cols[0].z = cols[2].x;
    inv.cols[1].x = cols[0].y;  inv.cols[1].y = cols[1].y;  inv.cols[1].z = cols[2].y;
    inv.cols[2].x = cols[0].z;  inv.cols[2].y = cols[1].z;  inv.cols[2].z = cols[2].z;

    const Vec<3, T> t{cols[3].x, cols[3].y, cols[3].z};
    // (R^T * t)[k] = cols[k].x*t.x + cols[k].y*t.y + cols[k].z*t.z
    // (using ORIGINAL cols, not inv.cols — that's the trick!)
    inv.cols[3].x = -(cols[0].x * t.x + cols[0].y * t.y + cols[0].z * t.z);
    inv.cols[3].y = -(cols[1].x * t.x + cols[1].y * t.y + cols[1].z * t.z);
    inv.cols[3].z = -(cols[2].x * t.x + cols[2].y * t.y + cols[2].z * t.z);
    inv.cols[3].w = T{1};
    return inv;
}

template<typename T>
Mat<4, 4, T> Mat<4, 4, T>::inverse() const noexcept {
    // Generic 4x4 inverse via cofactors. Not optimized — used only when
    // a transform has non-uniform scaling or shear, which is rare in CAD.
    const auto& m = cols;
    const T a2323 = m[2].z * m[3].w - m[2].w * m[3].z;
    const T a1323 = m[2].y * m[3].w - m[2].w * m[3].y;
    const T a1223 = m[2].y * m[3].z - m[2].z * m[3].y;
    const T a0323 = m[2].x * m[3].w - m[2].w * m[3].x;
    const T a0223 = m[2].x * m[3].z - m[2].z * m[3].x;
    const T a0123 = m[2].x * m[3].y - m[2].y * m[3].x;
    const T a2313 = m[1].z * m[3].w - m[1].w * m[3].z;
    const T a1313 = m[1].y * m[3].w - m[1].w * m[3].y;
    const T a1213 = m[1].y * m[3].z - m[1].z * m[3].y;
    const T a2312 = m[1].z * m[2].w - m[1].w * m[2].z;
    const T a1312 = m[1].y * m[2].w - m[1].w * m[2].y;
    const T a1212 = m[1].y * m[2].z - m[1].z * m[2].y;
    const T a0313 = m[1].x * m[3].w - m[1].w * m[3].x;
    const T a0213 = m[1].x * m[3].z - m[1].z * m[3].x;
    const T a0312 = m[1].x * m[2].w - m[1].w * m[2].x;
    const T a0212 = m[1].x * m[2].z - m[1].z * m[2].x;
    const T a0113 = m[1].x * m[3].y - m[1].y * m[3].x;
    const T a0112 = m[1].x * m[2].y - m[1].y * m[2].x;

    const T det =
        m[0].x * ( m[1].y * a2323 - m[1].z * a1323 + m[1].w * a1223 )
      - m[0].y * ( m[1].x * a2323 - m[1].z * a0323 + m[1].w * a0223 )
      + m[0].z * ( m[1].x * a1323 - m[1].y * a0323 + m[1].w * a0123 )
      - m[0].w * ( m[1].x * a1223 - m[1].y * a0223 + m[1].z * a0123 );

    if (det == T{0}) return identity();
    const T inv_det = T{1} / det;

    Mat r{};
    r.cols[0].x =  inv_det * ( m[1].y * a2323 - m[1].z * a1323 + m[1].w * a1223 );
    r.cols[0].y = -inv_det * ( m[0].y * a2323 - m[0].z * a1323 + m[0].w * a1223 );
    r.cols[0].z =  inv_det * ( m[0].y * a2313 - m[0].z * a1313 + m[0].w * a1213 );
    r.cols[0].w = -inv_det * ( m[0].y * a2312 - m[0].z * a1312 + m[0].w * a1212 );
    r.cols[1].x = -inv_det * ( m[1].x * a2323 - m[1].z * a0323 + m[1].w * a0223 );
    r.cols[1].y =  inv_det * ( m[0].x * a2323 - m[0].z * a0323 + m[0].w * a0223 );
    r.cols[1].z = -inv_det * ( m[0].x * a2313 - m[0].z * a0313 + m[0].w * a0213 );
    r.cols[1].w =  inv_det * ( m[0].x * a2312 - m[0].z * a0312 + m[0].w * a0212 );
    r.cols[2].x =  inv_det * ( m[1].x * a1323 - m[1].y * a0323 + m[1].w * a0123 );
    r.cols[2].y = -inv_det * ( m[0].x * a1323 - m[0].y * a0323 + m[0].w * a0123 );
    r.cols[2].z =  inv_det * ( m[0].x * a1313 - m[0].y * a0313 + m[0].w * a0113 );
    r.cols[2].w = -inv_det * ( m[0].x * a1312 - m[0].y * a0312 + m[0].w * a0112 );
    r.cols[3].x = -inv_det * ( m[1].x * a1223 - m[1].y * a0223 + m[1].z * a0123 );
    r.cols[3].y =  inv_det * ( m[0].x * a1223 - m[0].y * a0223 + m[0].z * a0123 );
    r.cols[3].z = -inv_det * ( m[0].x * a1213 - m[0].y * a0213 + m[0].z * a0113 );
    r.cols[3].w =  inv_det * ( m[0].x * a1212 - m[0].y * a0212 + m[0].z * a0112 );
    return r;
}

} // namespace cadforge::math
