// core/viewport/include/CAD_0/viewport/camera.hpp
//
// Camera system for the CAD_0 viewport.
//
// Two camera modes are supported:
//   * OrbitCamera  — turntable-style, used for CAD model inspection.
//                    Yaw around world Y, pitch clamped to [-89°, +89°]
//                    to avoid gimbal lock. Distance from target is
//                    adjustable via zoom.
//   * (FreeCamera  — deferred to Phase C.5; not needed for MVP)
//
// Coordinate system: right-handed, +Y up, -Z forward (OpenGL convention).
//
// All camera operations are reversible: orbit, pan, and zoom produce
// deltas that can be inverted for undo support.
//
#pragma once

#include "CAD_0/math/vec.hpp"
#include "CAD_0/math/mat.hpp"
#include "CAD_0/math/quat.hpp"
#include "CAD_0/math/bbox.hpp"

#include <cstdint>

namespace CAD_0::viewport {

class OrbitCamera {
public:
    OrbitCamera() = default;

    // Construct with a target point and an initial distance.
    OrbitCamera(const math::Vec3f& target, float distance,
                float yaw_rad = 0.0f, float pitch_rad = 0.0f)
        : target_(target), distance_(distance),
          yaw_(yaw_rad), pitch_(pitch_rad) {
        clamp_pitch();
    }

    // ----- Getters ------------------------------------------------------
    const math::Vec3f& target() const noexcept { return target_; }
    float distance() const noexcept { return distance_; }
    float yaw()   const noexcept { return yaw_; }    // radians, around world Y
    float pitch() const noexcept { return pitch_; }   // radians, around camera X

    // Field of view (vertical) in radians. Default ~45°.
    float fov_y() const noexcept { return fov_y_; }
    void  set_fov_y(float rad) noexcept { fov_y_ = rad; }

    // Aspect ratio (width / height). Caller updates this when the
    // viewport is resized.
    float aspect() const noexcept { return aspect_; }
    void  set_aspect(float a) noexcept { aspect_ = a; }

    // Near and far clipping plane distances (positive, in world units).
    float near_plane() const noexcept { return near_; }
    float far_plane()  const noexcept { return far_; }
    void  set_clip_planes(float n, float f) noexcept {
        near_ = n; far_ = f;
    }

    // ----- Derived state ------------------------------------------------

    // Camera position in world space.
    math::Vec3f position() const noexcept {
        // Spherical-to-cartesian, looking at target.
        const float cp = std::cos(pitch_);
        const float sp = std::sin(pitch_);
        const float cy = std::cos(yaw_);
        const float sy = std::sin(yaw_);
        // Direction FROM target TO camera.
        const math::Vec3f dir{
            cp * sy,
            sp,
            cp * cy,
        };
        return target_ + dir * distance_;
    }

    // Forward (look) direction (camera -Z axis).
    math::Vec3f forward() const noexcept {
        return (target_ - position()).normalized();
    }

    // Right (camera +X) and up (camera +Y) axes.
    math::Vec3f right() const noexcept {
        return forward().cross(world_up_).normalized();
    }
    math::Vec3f up() const noexcept {
        return right().cross(forward()).normalized();
    }

    // View matrix (world → camera).
    math::Mat4f view_matrix() const noexcept {
        const math::Vec3f p = position();
        const math::Vec3f f = forward();
        const math::Vec3f r = right();
        const math::Vec3f u = up();
        // Look-at: camera looks along -Z, so right = +X, up = +Y.
        math::Mat4f m = math::Mat4f::identity();
        m.cols[0] = {r.x, u.x, -f.x, 0.0f};
        m.cols[1] = {r.y, u.y, -f.y, 0.0f};
        m.cols[2] = {r.z, u.z, -f.z, 0.0f};
        m.cols[3] = {-(r.dot(p)), -(u.dot(p)), f.dot(p), 1.0f};
        return m;
    }

    // Projection matrix (camera → clip space). OpenGL convention.
    math::Mat4f projection_matrix() const noexcept {
        const float f = 1.0f / std::tan(fov_y_ * 0.5f);
        math::Mat4f m{};  // zeroed
        m.cols[0].x = f / aspect_;
        m.cols[1].y = f;
        m.cols[2].z = (far_ + near_) / (near_ - far_);
        m.cols[2].w = -1.0f;
        m.cols[3].z = (2.0f * far_ * near_) / (near_ - far_);
        return m;
    }

    math::Mat4f view_projection_matrix() const noexcept {
        return projection_matrix() * view_matrix();
    }

    // ----- Operations (all reversible) -----------------------------------

    // Orbit by (delta_yaw, delta_pitch) in radians.
    // Returns the deltas actually applied (after pitch clamping).
    // The returned deltas can be passed to `orbit_inverse` to exactly
    // reverse the operation.
    struct OrbitDelta {
        float dyaw;
        float dpitch;
    };
    OrbitDelta orbit(float delta_yaw, float delta_pitch) noexcept {
        const float old_yaw = yaw_;
        const float old_pitch = pitch_;
        yaw_   += delta_yaw;
        pitch_ += delta_pitch;
        clamp_pitch();
        // Effective delta = new - old (after clamping).
        return {yaw_ - old_yaw, pitch_ - old_pitch};
    }

    // Inverse of an orbit operation. Useful for undo.
    void orbit_inverse(OrbitDelta d) noexcept {
        yaw_   -= d.dyaw;
        pitch_ -= d.dpitch;
        clamp_pitch();
    }

    // Pan: move the target by (dx_world, dy_world) in camera space.
    // The camera position moves with the target so the view is unchanged.
    void pan(float dx_screen, float dy_screen) noexcept {
        // Scale by distance so that panning feels consistent across zoom levels.
        const float scale = distance_ * std::tan(fov_y_ * 0.5f) * 2.0f;
        const math::Vec3f r = right();
        const math::Vec3f u = up();
        // Screen Y is inverted (top of screen = +Y in pixel coords, -Y in world).
        target_ = target_ + r * (dx_screen * scale / aspect_)
                          + u * (dy_screen * scale);
    }

    // Zoom: multiply distance by a factor. < 1 = zoom in, > 1 = zoom out.
    void zoom(float factor) noexcept {
        if (factor <= 0.0f) return;
        distance_ *= factor;
        // Clamp to a reasonable range relative to the scene.
        distance_ = std::clamp(distance_, min_distance_, max_distance_);
    }

    // Frame a bounding box: adjusts target + distance so the box fits.
    void frame_bounds(const math::Bboxf& b) noexcept {
        if (b.empty()) return;
        target_ = b.center();
        const float r = b.radius();
        // Distance so that the sphere of radius r fits in the FOV.
        const float half_fov = fov_y_ * 0.5f;
        const float tan_h = std::tan(half_fov);
        // Use the smaller of vertical/horizontal fit.
        const float tan_h_x = tan_h * aspect_;
        const float tan_min = std::min(tan_h, tan_h_x);
        distance_ = (tan_min > 0.0f) ? (r / tan_min) : (r * 2.0f);
        // Adjust clip planes to fit.
        const float extent = b.extent().length();
        near_ = std::max(distance_ - extent, extent * 0.001f);
        far_  = distance_ + extent * 2.0f;
        // Adjust min/max distance for zoom clamps.
        min_distance_ = extent * 0.01f;
        max_distance_ = extent * 100.0f;
    }

    // ----- Ray casting (for picking) ------------------------------------

    // Convert screen coordinates (in [-1, 1] NDC, y up) to a world-space ray.
    // Returns origin and direction.
    struct Ray {
        math::Vec3f origin;
        math::Vec3f direction;  // normalized
    };
    Ray ray_from_ndc(float ndc_x, float ndc_y) const noexcept {
        const math::Mat4f inv_vp = view_projection_matrix().inverse();
        // Two points on the ray: near plane (z=-1) and far plane (z=+1).
        const math::Vec4f p_near = inv_vp * math::Vec4f{ndc_x, ndc_y, -1.0f, 1.0f};
        const math::Vec4f p_far  = inv_vp * math::Vec4f{ndc_x, ndc_y,  1.0f, 1.0f};
        // Perspective divide.
        const math::Vec3f o{p_near.x / p_near.w, p_near.y / p_near.w, p_near.z / p_near.w};
        const math::Vec3f q{p_far.x  / p_far.w,  p_far.y  / p_far.w,  p_far.z  / p_far.w};
        return Ray{o, (q - o).normalized()};
    }

private:
    math::Vec3f target_{0.0f, 0.0f, 0.0f};
    float distance_{5.0f};
    float yaw_{0.0f};      // around world Y
    float pitch_{0.3f};    // around camera X (slight downward tilt default)
    float fov_y_{0.7853982f};  // ~45°
    float aspect_{1.0f};
    float near_{0.01f};
    float far_{1000.0f};
    float min_distance_{0.001f};
    float max_distance_{1e6f};
    math::Vec3f world_up_{0.0f, 1.0f, 0.0f};

    // Clamp pitch to avoid gimbal lock at ±90°.
    // Returns the actual pitch delta applied (after clamping).
    float clamp_pitch() noexcept {
        const float max_pitch = 1.5533f;  // ~89°
        const float old_pitch = pitch_;
        pitch_ = std::clamp(pitch_, -max_pitch, max_pitch);
        return pitch_ - old_pitch;
    }
};

} // namespace CAD_0::viewport
