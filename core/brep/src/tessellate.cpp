// core/brep/src/tessellate.cpp
//
// THE DISPLAY BOUNDARY (H1, ADR-0034): the tessellation is the
// float pipeline's own domain (the SDF mesh world); the exact
// surfaces are double. The conversion is ONE DIRECTION (double ->
// float), explicit at each site below, and the parameter windows are
// dispatched on the TYPED family enum (no string compare).
#include "CAD_0/brep/tessellate.hpp"

#include <cmath>
#include <string>

namespace CAD_0::brep {

sdf::TriangleMesh tessellate_face(const Face& face, const TessellationOptions& opts) {
    sdf::TriangleMesh mesh;
    const SurfaceGeometry* surf = face.surface();
    if (!surf) return mesh;

    const std::uint32_t N = opts.resolution;
    const double pi = 3.14159265358979;

    // Determine parametric range based on the TYPED surface family
    // (a typo in a case label does not compile — ADR-0034 D3). For
    // MVP, we use full parametric range for closed surfaces and a
    // fixed range for open ones.
    double u_min = 0.0, u_max = 2.0 * pi;
    double v_min = 0.0, v_max = 1.0;

    switch (surf->family()) {
    case SurfaceFamily::Plane:
        u_min = -10.0; u_max = 10.0;
        v_min = -10.0; v_max = 10.0;
        break;
    case SurfaceFamily::Cylinder:
        v_min = -1.0; v_max = 1.0;  // half-height
        break;
    case SurfaceFamily::Sphere:
        v_min = 0.0; v_max = pi;
        break;
    case SurfaceFamily::Cone:
        v_min = 0.0; v_max = 1.0;
        break;
    case SurfaceFamily::Torus:
        u_min = 0.0; u_max = 2.0 * pi;
        v_min = 0.0; v_max = 2.0 * pi;
        break;
    default:
        break;
    }

    // Sample grid
    const std::uint32_t Nu = N + 1;
    const std::uint32_t Nv = N + 1;
    const double du = (u_max - u_min) / double(N);
    const double dv = (v_max - v_min) / double(N);

    // Generate vertices (the double->float boundary conversion, one
    // direction, declared here — the mesh's own domain).
    const auto to_f3 = [](const math::Vec3d& p) {
        return math::Vec3f{static_cast<float>(p.x),
                           static_cast<float>(p.y),
                           static_cast<float>(p.z)};
    };
    for (std::uint32_t j = 0; j < Nv; ++j) {
        for (std::uint32_t i = 0; i < Nu; ++i) {
            const double u = u_min + double(i) * du;
            const double v = v_min + double(j) * dv;
            mesh.positions.push_back(to_f3(surf->evaluate(u, v)));
            if (opts.compute_normals) {
                mesh.normals.push_back(to_f3(surf->normal(u, v)));
            }
        }
    }

    // Generate triangles (two per grid cell)
    for (std::uint32_t j = 0; j < N; ++j) {
        for (std::uint32_t i = 0; i < N; ++i) {
            const std::uint32_t i00 = j * Nu + i;
            const std::uint32_t i10 = j * Nu + i + 1;
            const std::uint32_t i01 = (j + 1) * Nu + i;
            const std::uint32_t i11 = (j + 1) * Nu + i + 1;

            mesh.indices.push_back(i00);
            mesh.indices.push_back(i10);
            mesh.indices.push_back(i11);

            mesh.indices.push_back(i00);
            mesh.indices.push_back(i11);
            mesh.indices.push_back(i01);
        }
    }

    return mesh;
}

sdf::TriangleMesh tessellate_body(const BRepBody& body, const TessellationOptions& opts) {
    sdf::TriangleMesh mesh;

    for (const auto& face : body.solid.shell().faces()) {
        sdf::TriangleMesh face_mesh = tessellate_face(face, opts);
        // Append face_mesh to mesh
        const std::uint32_t offset = static_cast<std::uint32_t>(mesh.positions.size());
        for (const auto& p : face_mesh.positions) mesh.positions.push_back(p);
        if (opts.compute_normals && !face_mesh.normals.empty()) {
            for (const auto& n : face_mesh.normals) mesh.normals.push_back(n);
        }
        for (const auto idx : face_mesh.indices) mesh.indices.push_back(idx + offset);
    }

    return mesh;
}

} // namespace CAD_0::brep
