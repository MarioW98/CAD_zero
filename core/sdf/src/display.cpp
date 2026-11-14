// core/sdf/src/display.cpp
//
// The display pipeline's normal buffer — the ONE arithmetic definition
// both the sync road (the binding) and the async road (scene::
// derive_display_mesh) call. See display.hpp for the contract.
#include "CAD_0/sdf/display.hpp"

#include <cstddef>

namespace CAD_0::sdf {

std::vector<math::Vec3f> display_normals(
    const SDFBody& body,
    std::span<const math::Vec3f> positions,
    std::span<const math::Vec3f> mesh_normals) {
    // Same guard constant as the kernel's own normal paths
    // (interp_gradient_abs, CylinderSDF's axis guard): below it a
    // direction is not defined.
    constexpr float kDegenerateGradient = 1e-9f;

    const std::size_t n = positions.size();
    std::vector<math::Vec3f> out;
    out.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        const math::Vec3f g = body.sample(positions[i]).gradient;
        const float gl = g.length();
        if (gl > kDegenerateGradient) {
            out[i] = g * (1.0f / gl);
        } else if (i < mesh_normals.size()) {
            out[i] = mesh_normals[i];
        } else {
            out[i] = math::Vec3f{0.0f, 0.0f, 0.0f};
        }
    }
    return out;
}

} // namespace CAD_0::sdf
