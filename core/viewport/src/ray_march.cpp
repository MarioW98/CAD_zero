// core/viewport/src/ray_march.cpp
//
// CPU-side ray-marcher implementation. Mirrors the algorithm in
// glsl/sdf_raymarch.frag for shader logic verification.
//
// Lipschitz-aware step scaling
// ----------------------------
// The SDF invariant is |f(p + h) - f(p)| ≤ L · h, where L = field.lipschitz().
// To safely advance from p by distance d = |f(p)| (signed distance to
// the surface) without overshooting, we must move at most d / L.
//
// For a unit-Lipschitz SDF (primitives, ScaleSDF with s ≤ 1, etc.) L = 1
// and the safe step is exactly d — matching the GLSL shader's
// `t += d` step. For L > 1 (TwistSDF, SmoothMin with k > 0), the safe
// step is d / L, smaller than d, so the marcher takes more steps but
// never overshoots.
//
// Combined step scale: opts.step_scale / max(L, 1.0f). The max(., 1.0f)
// is a safety net — it prevents the marcher from taking *larger* steps
// than d when L < 1 (which shouldn't happen but is defensive).
//
// Parity with GLSL: when L = 1 (the only case the current shader
// handles — see sdf_raymarch.frag's hardcoded `length(p) - 1.0` sphere),
// the CPU's `t += d * effective_scale` = `t += d`, identical to the
// shader. When we extend the shader to handle L > 1 fields, the same
// `t += d / max(L, 1.0)` formula must be ported.
//
#include "CAD_0/viewport/ray_march.hpp"

#include <algorithm>
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

    // The Lipschitz constant bounds how fast the field can change per
    // unit distance: |f(p + h) - f(p)| ≤ L · h. To safely advance by a
    // distance d = |f(p)| without overshooting the surface, we must
    // move at most d / L. See the file header for the parity analysis
    // with the GLSL shader.
    //
    // The combined step scale = opts.step_scale / max(L, 1.0f). The
    // max(., 1.0f) avoids accidentally *enlarging* steps for L < 1
    // (which shouldn't happen but is a safety net).
    const float body_lipschitz = body.lipschitz();
    const float safe_l = std::max(body_lipschitz, 1.0f);
    const float effective_scale = opts.step_scale / safe_l;

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
        // Advance by min(d, d/L) — equivalently d / max(L, 1). For L = 1
        // (unit-Lipschitz SDF, matching the GLSL shader) this is exactly
        // `t += d`, preserving shader parity. For L > 1 this is d/L,
        // the safe step that prevents overshoot.
        t += d * effective_scale;
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
