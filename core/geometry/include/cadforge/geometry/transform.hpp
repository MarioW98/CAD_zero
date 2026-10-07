// core/geometry/include/cadforge/geometry/transform.hpp
//
// Rigid + non-rigid transform, expressed relative to a DatumCS.
//
#pragma once

#include "cadforge/math/vec.hpp"
#include "cadforge/math/mat.hpp"
#include "cadforge/math/quat.hpp"

namespace cadforge::geometry {

// Datum coordinate system — reference frame for a shape.
struct DatumCS {
    math::Vec3f origin{};
    math::Vec3f x_axis{1.0f, 0.0f, 0.0f};
    math::Vec3f y_axis{0.0f, 1.0f, 0.0f};
    math::Vec3f z_axis{0.0f, 0.0f, 1.0f};

    math::Mat4f to_world() const noexcept {
        math::Mat4f m = math::Mat4f::identity();
        m.cols[0] = {x_axis.x, x_axis.y, x_axis.z, 0.0f};
        m.cols[1] = {y_axis.x, y_axis.y, y_axis.z, 0.0f};
        m.cols[2] = {z_axis.x, z_axis.y, z_axis.z, 0.0f};
        m.cols[3] = {origin.x, origin.y, origin.z, 1.0f};
        return m;
    }

    math::Mat4f to_local() const noexcept {
        return to_world().inverse_orthonormal();
    }
};

// A user-facing transform: translation + rotation (quaternion) + uniform scale.
struct Transform {
    math::Vec3f translation{};
    math::Quatf rotation{};
    float       scale{1.0f};

    math::Mat4f to_matrix() const noexcept {
        math::Mat4f m = math::Mat4f::scaling({scale, scale, scale})
                      * rotation.to_matrix()
                      * math::Mat4f::translation(translation);
        return m;
    }

    static Transform identity() noexcept { return {}; }

    Transform inverse() const noexcept {
        Transform r;
        r.scale = (scale != 0.0f) ? 1.0f / scale : 1.0f;
        r.rotation = rotation.conjugate();
        r.translation = rotation.conjugate().rotate(translation * -r.scale);
        return r;
    }
};

} // namespace cadforge::geometry
