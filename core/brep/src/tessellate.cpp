// core/brep/src/tessellate.cpp
#include "CAD_0/brep/tessellate.hpp"

#include <cmath>

namespace CAD_0::brep {

sdf::TriangleMesh tessellate_face(const Face& face, const TessellationOptions& opts) {
    sdf::TriangleMesh mesh;
    const SurfaceGeometry* surf = face.surface();
    if (!surf) return mesh;

    const std::uint32_t N = opts.resolution;
    const float pi = 3.14159265358979f;

    // Determine parametric range based on surface type.
    // For MVP, we use full parametric range for closed surfaces
    // and a fixed range for open ones.
    float u_min = 0.0f, u_max = 2.0f * pi;
    float v_min = 0.0f, v_max = 1.0f;

    const char* tn = surf->type_name();
    if (std::string(tn) == "plane") {
        u_min = -10.0f; u_max = 10.0f;
        v_min = -10.0f; v_max = 10.0f;
    } else if (std::string(tn) == "cylinder") {
        v_min = -1.0f; v_max = 1.0f;  // half-height
    } else if (std::string(tn) == "sphere") {
        v_min = 0.0f; v_max = pi;
    } else if (std::string(tn) == "cone") {
        v_min = 0.0f; v_max = 1.0f;
    } else if (std::string(tn) == "torus") {
        u_min = 0.0f; u_max = 2.0f * pi;
        v_min = 0.0f; v_max = 2.0f * pi;
    }

    // Sample grid
    const std::uint32_t Nu = N + 1;
    const std::uint32_t Nv = N + 1;
    const float du = (u_max - u_min) / float(N);
    const float dv = (v_max - v_min) / float(N);

    // Generate vertices
    for (std::uint32_t j = 0; j < Nv; ++j) {
        for (std::uint32_t i = 0; i < Nu; ++i) {
            const float u = u_min + float(i) * du;
            const float v = v_min + float(j) * dv;
            mesh.positions.push_back(surf->evaluate(u, v));
            if (opts.compute_normals) {
                mesh.normals.push_back(surf->normal(u, v));
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
