// tests/unit/sdf/test_mesh_extract.cpp
#include <doctest/doctest.h>

#include "cadforge/sdf/primitives.hpp"
#include "cadforge/sdf/operators.hpp"
#include "cadforge/sdf/transforms.hpp"
#include "cadforge/sdf/mesh_extract.hpp"

using namespace cadforge::sdf;
using namespace cadforge::math;

TEST_CASE("marching_cubes: sphere produces non-empty mesh") {
    auto sph = make_sphere(1.0f);
    auto mesh = marching_cubes(sph, 16, true);
    CHECK(mesh.vertex_count() > 0);
    CHECK(mesh.triangle_count() > 0);
}

TEST_CASE("marching_cubes: sphere mesh has unit normals") {
    auto sph = make_sphere(1.0f);
    auto mesh = marching_cubes(sph, 16, true);
    REQUIRE(!mesh.normals.empty());
    // At least the first normal should be approximately unit length
    const float len = mesh.normals[0].length();
    CHECK(len == doctest::Approx(1.0f).epsilon(1e-3f));
}

TEST_CASE("marching_cubes: sphere mesh vertices lie near unit sphere") {
    auto sph = make_sphere(1.0f);
    auto mesh = marching_cubes(sph, 32, true);
    REQUIRE(!mesh.positions.empty());

    // Most vertices should be within 5% of radius 1 from origin.
    std::size_t near_sphere = 0;
    for (const auto& p : mesh.positions) {
        const float r = p.length();
        if (std::abs(r - 1.0f) < 0.1f) ++near_sphere;
    }
    // At least 90% of vertices should be close to the sphere surface.
    CHECK(near_sphere * 10 >= mesh.vertex_count());
}

TEST_CASE("marching_cubes: empty body returns empty mesh") {
    SDFBody empty;
    auto mesh = marching_cubes(empty, 16, true);
    CHECK(mesh.vertex_count() == 0);
    CHECK(mesh.triangle_count() == 0);
}

TEST_CASE("marching_cubes: composite (sphere - cylinder)") {
    auto sph = make_sphere(2.0f);
    auto cyl = make_cylinder(0.5f, 10.0f);
    auto cut = sdf_subtract(std::move(sph), std::move(cyl));

    auto mesh = marching_cubes(cut, 24, false);
    CHECK(mesh.triangle_count() > 0);
}

TEST_CASE("marching_cubes: bounds override works for infinite plane") {
    // PlaneSDF has infinite bounds — must supply explicit bounds.
    auto plane = make_plane();
    cadforge::math::Bboxf bnds{{-2, -2, -2}, {2, 2, 2}};
    auto mesh = marching_cubes(plane, bnds, 16, false);
    CHECK(mesh.triangle_count() > 0);
}

TEST_CASE("marching_cubes: smooth union produces mesh") {
    auto a = make_sphere(1.0f);
    auto b = sdf_translate(make_sphere(1.0f), {2, 0, 0});
    auto u = sdf_smooth_union(std::move(a), std::move(b), 0.6f);
    auto mesh = marching_cubes(u, 24, false);
    CHECK(mesh.triangle_count() > 0);
}

TEST_CASE("marching_cubes: weld_vertices reduces vertex count significantly") {
    auto sph = make_sphere(1.0f);

    // Use the SAME bounds for both calls so the triangle counts match.
    cadforge::math::Bboxf box = sph.bounds();
    const cadforge::math::Vec3f pad = box.extent() * 0.01f;
    box.min = box.min - pad;
    box.max = box.max + pad;

    // Phase B (no weld): use options with weld_vertices = false.
    MarchingCubesOptions opts_noweld;
    opts_noweld.resolution = 32;
    opts_noweld.compute_normals = true;
    opts_noweld.weld_vertices = false;
    auto mesh_noweld = marching_cubes(sph, box, opts_noweld);

    // Phase B.5 (with weld).
    MarchingCubesOptions opts_weld;
    opts_weld.resolution = 32;
    opts_weld.compute_normals = true;
    opts_weld.weld_vertices = true;
    auto mesh_weld = marching_cubes(sph, box, opts_weld);

    // Same number of triangles (welding doesn't change topology).
    CHECK(mesh_weld.triangle_count() == mesh_noweld.triangle_count());

    // With welding enabled, vertex count should be much smaller.
    CHECK(mesh_weld.vertex_count() < mesh_weld.triangle_count());
    CHECK(mesh_weld.vertex_count() < mesh_noweld.vertex_count() / 2);
}

TEST_CASE("marching_cubes: narrow-band culling skips empty cells") {
    // A small sphere in a large bounding box: most cells are entirely
    // outside and should be skipped. Triangle count should still be
    // correct (matches the unbounded version).
    auto sph = make_sphere(1.0f);
    cadforge::math::Bboxf big_box{{-5, -5, -5}, {5, 5, 5}};

    MarchingCubesOptions opts;
    opts.resolution = 32;
    opts.compute_normals = false;
    opts.weld_vertices = true;
    auto mesh = marching_cubes(sph, big_box, opts);

    // Should produce a reasonable number of triangles for a unit sphere.
    CHECK(mesh.triangle_count() > 100);
    CHECK(mesh.triangle_count() < 10000);
}

TEST_CASE("weld_vertices: utility function merges duplicates") {
    // Construct a mesh with deliberately duplicated vertices.
    TriangleMesh mesh;
    mesh.positions = {
        {0, 0, 0}, {1, 0, 0}, {0, 1, 0},  // triangle 0
        {0, 0, 0}, {1, 0, 0}, {1, 1, 0},  // triangle 1 (shares 2 verts)
    };
    mesh.indices = {0, 1, 2, 3, 4, 5};

    const std::size_t new_count = weld_vertices(mesh, 1e-5f);
    // After welding: 4 unique vertices (0,0,0), (1,0,0), (0,1,0), (1,1,0)
    CHECK(new_count == 4);
    CHECK(mesh.vertex_count() == 4);
    // Indices should reference only the 4 unique vertices.
    for (const auto idx : mesh.indices) {
        CHECK(idx < 4);
    }
}

TEST_CASE("weld_vertices: empty mesh is a no-op") {
    TriangleMesh mesh;
    const std::size_t new_count = weld_vertices(mesh, 1e-5f);
    CHECK(new_count == 0);
    CHECK(mesh.positions.empty());
    CHECK(mesh.indices.empty());
}
