// core/viewport/include/CAD_0/viewport/ray_march.hpp
//
// CPU-side ray-marching for SDF preview and testing.
//
// Mirrors the algorithm in glsl/sdf_raymarch.frag so we can verify the
// shader logic without requiring an OpenGL context.
//
// Used by:
//   * Tests (this module)
//   * HeadlessRenderer (optional — could be wired up to produce pixel output)
//   * Picking (ray-march against the SDF to find hit point + normal)
//
#pragma once

#include "CAD_0/math/vec.hpp"
#include "CAD_0/math/mat.hpp"
#include "CAD_0/sdf/field.hpp"
#include "CAD_0/viewport/camera.hpp"

#include <optional>
#include <cstdint>

namespace CAD_0::viewport {

// Result of a ray-march against an SDF.
struct RayMarchResult {
    bool hit{false};
    float distance{0.0f};          // distance traveled (only valid if hit)
    math::Vec3f point{};            // world-space hit point
    math::Vec3f normal{};           // surface normal at hit (normalized)
    std::uint32_t steps_taken{0};   // actual steps used (for profiling)
};

// Configuration for the ray-marcher.
struct RayMarchOptions {
    std::uint32_t max_steps{128};
    float max_distance{100.0f};
    float surface_distance{0.0005f};
    // Step scale factor for sphere-tracing. < 1 makes the marcher
    // more conservative (slower but safer for non-Lipschitz SDFs).
    float step_scale{1.0f};
};

// March a ray against an SDF body.
// `ro` = ray origin, `rd` = ray direction (must be normalized).
// Returns the hit result, or nullopt if no surface was found within max_steps.
RayMarchResult ray_march_sdf(const CAD_0::sdf::SDFBody& body,
                              const math::Vec3f& ro,
                              const math::Vec3f& rd,
                              const RayMarchOptions& opts = {});

// March a ray from the camera through the given NDC coordinates.
// This mirrors the fragment shader's ray construction.
RayMarchResult ray_march_sdf_from_camera(const CAD_0::sdf::SDFBody& body,
                                          const OrbitCamera& camera,
                                          float ndc_x,
                                          float ndc_y,
                                          const RayMarchOptions& opts = {});

} // namespace CAD_0::viewport
