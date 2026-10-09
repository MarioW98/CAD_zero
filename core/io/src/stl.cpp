// core/io/src/stl.cpp
#include "CAD_0/io/stl.hpp"

#include <CAD_0/math/vec.hpp>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <ios>

namespace CAD_0::io {

namespace {

// Compute the normal of a triangle (v0, v1, v2) using the right-hand rule.
// Returns a unit vector; degenerate triangles return (0, 0, 0).
math::Vec3f triangle_normal(const math::Vec3f& v0,
                            const math::Vec3f& v1,
                            const math::Vec3f& v2) noexcept {
    const math::Vec3f e1 = v1 - v0;
    const math::Vec3f e2 = v2 - v0;
    const math::Vec3f n = e1.cross(e2);
    const float len = n.length();
    if (len < 1e-12f) return {0.0f, 0.0f, 0.0f};
    return n * (1.0f / len);
}

} // namespace

bool export_stl_binary(const std::string& path,
                       const CAD_0::sdf::TriangleMesh& mesh,
                       std::string_view name) {
    std::ofstream f(path, std::ios::binary);
    if (!f) return false;

    // 80-byte header.
    std::array<char, 80> header{};
    const std::size_t name_len = std::min<std::size_t>(name.size(), 79);
    std::memcpy(header.data(), name.data(), name_len);
    f.write(header.data(), 80);

    // Triangle count (little-endian uint32).
    const std::uint32_t tri_count = static_cast<std::uint32_t>(mesh.triangle_count());
    f.write(reinterpret_cast<const char*>(&tri_count), 4);

    // Per-triangle: 12 bytes normal + 36 bytes vertices + 2 bytes attribute.
    const bool have_normals = !mesh.normals.empty();
    for (std::size_t t = 0; t < mesh.triangle_count(); ++t) {
        const std::uint32_t i0 = mesh.indices[t * 3 + 0];
        const std::uint32_t i1 = mesh.indices[t * 3 + 1];
        const std::uint32_t i2 = mesh.indices[t * 3 + 2];
        const math::Vec3f v0 = mesh.positions[i0];
        const math::Vec3f v1 = mesh.positions[i1];
        const math::Vec3f v2 = mesh.positions[i2];

        math::Vec3f n;
        if (have_normals) {
            // Average the three vertex normals.
            n = (mesh.normals[i0] + mesh.normals[i1] + mesh.normals[i2]) * (1.0f / 3.0f);
            const float nl = n.length();
            if (nl > 1e-9f) n = n * (1.0f / nl);
        } else {
            n = triangle_normal(v0, v1, v2);
        }

        f.write(reinterpret_cast<const char*>(&n.x), 4);
        f.write(reinterpret_cast<const char*>(&n.y), 4);
        f.write(reinterpret_cast<const char*>(&n.z), 4);

        f.write(reinterpret_cast<const char*>(&v0.x), 12);
        f.write(reinterpret_cast<const char*>(&v1.x), 12);
        f.write(reinterpret_cast<const char*>(&v2.x), 12);

        const std::uint16_t attr = 0;
        f.write(reinterpret_cast<const char*>(&attr), 2);
    }
    return static_cast<bool>(f);
}

bool export_stl_ascii(const std::string& path,
                      const CAD_0::sdf::TriangleMesh& mesh,
                      std::string_view name) {
    std::ofstream f(path);
    if (!f) return false;
    f << std::setprecision(6) << std::scientific;

    f << "solid " << name << "\n";
    const bool have_normals = !mesh.normals.empty();
    for (std::size_t t = 0; t < mesh.triangle_count(); ++t) {
        const std::uint32_t i0 = mesh.indices[t * 3 + 0];
        const std::uint32_t i1 = mesh.indices[t * 3 + 1];
        const std::uint32_t i2 = mesh.indices[t * 3 + 2];
        const math::Vec3f v0 = mesh.positions[i0];
        const math::Vec3f v1 = mesh.positions[i1];
        const math::Vec3f v2 = mesh.positions[i2];

        math::Vec3f n;
        if (have_normals) {
            n = (mesh.normals[i0] + mesh.normals[i1] + mesh.normals[i2]) * (1.0f / 3.0f);
            const float nl = n.length();
            if (nl > 1e-9f) n = n * (1.0f / nl);
        } else {
            n = triangle_normal(v0, v1, v2);
        }
        f << "  facet normal " << n.x << " " << n.y << " " << n.z << "\n";
        f << "    outer loop\n";
        f << "      vertex " << v0.x << " " << v0.y << " " << v0.z << "\n";
        f << "      vertex " << v1.x << " " << v1.y << " " << v1.z << "\n";
        f << "      vertex " << v2.x << " " << v2.y << " " << v2.z << "\n";
        f << "    endloop\n";
        f << "  endfacet\n";
    }
    f << "endsolid " << name << "\n";
    return static_cast<bool>(f);
}

} // namespace CAD_0::io
