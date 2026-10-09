// core/viewport/src/ray_march.cpp
//
// CPU-side ray-marcher implementation. Mirrors the algorithm in
// glsl/sdf_raymarch.frag for shader logic verification.
//
#include "CAD_0/viewport/ray_march.hpp"

#include <cmath>

namespace CAD_0::viewport {

namespace {

// Compute the surface normal at a point using a tetrahedron of SDF samples.
// This is the same technique used in the fragment shader.
math::Vec3f calc_normal(const CAD_0::sdf::SDFBody& body,
                         const math::Vec3f& p,
                         float eps) {
    // Tetrahedron offsets — 4 samples instead of the 6 used by a
    // cross-pattern finite difference.
    const math::Vec3f k1{1.0f, -1.0f, -1.0f};
    const math::Vec3f k2{-1.0f, -1.0f, 1.0f};
    const math::Vec3f k3{-1.0f, 1.0f, -1.0f};
    const math::Vec3f k4{1.0f, 1.0f, 1.0f};

    const float s1 = body.value(p + k1 * eps);
    const float s2 = body.value(p + k2 * eps);
    const float s3 = body.value(p + k3 * eps);
    const float s4 = body.value(p + k4 * eps);

    math::Vec3f n = k1 * s1 + k2 * s2 + k3 * s3 + k4 * s4;
    const float len = n.length();
    if (len < 1e-9f) return {0.0f, 1.0f, 0.0f};  // fallback
    return n * (1.0f / len);
}

} // namespace

RayMarchResult ray_march_sdf(const CAD_0::sdf::SDFBody& body,
                              const math::Vec3f& ro,
                              const math::Vec3f& rd,
                              const RayMarchOptions& opts) {
    RayMarchResult result{};
    float t = 0.0f;  // distance traveled

    for (std::uint32_t i = 0; i < opts.max_steps; ++i) {
        result.steps_taken = i + 1;
        const math::Vec3f p = ro + rd * t;
        const float d = body.value(p);
        if (d < opts.surface_distance) {
            result.hit = true;
            result.distance = t;
            result.point = p;
            // Normal eps scaled by surface distance for stability.
            result.normal = calc_normal(body, p, opts.surface_distance * 2.0f);
            return result;
        }
        // Advance by the signed distance.
        t += d * opts.step_scale;
        // Check max_distance AFTER advancing, so we don't take another step
        // past the limit (which could produce a false hit).
        if (t > opts.max_distance) break;
    }
    return result;
}

RayMarchResult ray_march_sdf_from_camera(const CAD_0::sdf::SDFBody& body,
                                          const OrbitCamera& camera,
                                          float ndc_x,
                                          float ndc_y,
                                          const RayMarchOptions& opts) {
    // Build the ray from camera through NDC pixel.
    const auto ray = camera.ray_from_ndc(ndc_x, ndc_y);
    return ray_march_sdf(body, ray.origin, ray.direction, opts);
}

} // namespace CAD_0::viewport
