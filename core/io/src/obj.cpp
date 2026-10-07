// core/io/src/obj.cpp
#include "cadforge/io/obj.hpp"

#include <cadforge/math/vec.hpp>

#include <fstream>
#include <iomanip>

namespace cadforge::io {

bool export_obj(const std::string& path,
                const cadforge::sdf::TriangleMesh& mesh,
                std::string_view name) {
    std::ofstream f(path);
    if (!f) return false;
    f << std::setprecision(6) << std::scientific;

    // Header comment.
    f << "# Exported by cadforge\n";
    f << "# name: " << name << "\n";
    f << "# vertices: " << mesh.vertex_count() << "\n";
    f << "# triangles: " << mesh.triangle_count() << "\n";
    f << "\n";

    // Vertex positions (1-indexed in OBJ).
    for (const auto& p : mesh.positions) {
        f << "v " << p.x << " " << p.y << " " << p.z << "\n";
    }
    f << "\n";

    // Vertex normals (if present).
    const bool have_normals = !mesh.normals.empty();
    if (have_normals) {
        for (const auto& n : mesh.normals) {
            f << "vn " << n.x << " " << n.y << " " << n.z << "\n";
        }
        f << "\n";
    }

    // Faces — OBJ uses 1-based indexing.
    // Format with normals: f v1//n1 v2//n2 v3//n3
    // Format without:      f v1 v2 v3
    for (std::size_t t = 0; t < mesh.triangle_count(); ++t) {
        const std::uint32_t i0 = mesh.indices[t * 3 + 0] + 1;
        const std::uint32_t i1 = mesh.indices[t * 3 + 1] + 1;
        const std::uint32_t i2 = mesh.indices[t * 3 + 2] + 1;
        if (have_normals) {
            f << "f " << i0 << "//" << i0 << " "
                   << i1 << "//" << i1 << " "
                   << i2 << "//" << i2 << "\n";
        } else {
            f << "f " << i0 << " " << i1 << " " << i2 << "\n";
        }
    }
    return static_cast<bool>(f);
}

} // namespace cadforge::io
