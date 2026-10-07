// tests/unit/io/test_stl.cpp
#include <doctest/doctest.h>

#include "cadforge/sdf/primitives.hpp"
#include "cadforge/sdf/mesh_extract.hpp"
#include "cadforge/io/stl.hpp"

#include <cstdio>
#include <fstream>
#include <string>

using namespace cadforge;

TEST_CASE("STL: binary export writes valid file") {
    auto sph = sdf::make_sphere(1.0f);
    auto mesh = sdf::marching_cubes(sph, 16, true);

    const std::string path = "/tmp/test_cadforge_sphere.stl";
    REQUIRE(io::export_stl_binary(path, mesh, "test_sphere"));

    // File should exist and be non-empty.
    std::ifstream f(path, std::ios::binary);
    REQUIRE(f.good());
    f.seekg(0, std::ios::end);
    const auto size = f.tellg();
    CHECK(size > 80 + 4);  // header + triangle count

    // Expected size = 80 + 4 + 50 * triangle_count
    const std::size_t expected = 84 + 50ull * mesh.triangle_count();
    CHECK(static_cast<std::size_t>(size) == expected);

    std::remove(path.c_str());
}

TEST_CASE("STL: ascii export writes valid file") {
    auto sph = sdf::make_sphere(1.0f);
    auto mesh = sdf::marching_cubes(sph, 8, false);

    const std::string path = "/tmp/test_cadforge_sphere_ascii.stl";
    REQUIRE(io::export_stl_ascii(path, mesh, "test_sphere_ascii"));

    std::ifstream f(path);
    REQUIRE(f.good());

    std::string line;
    std::getline(f, line);
    CHECK(line.find("solid") == 0);
    CHECK(line.find("test_sphere_ascii") != std::string::npos);

    // Should contain at least one facet
    bool found_facet = false;
    while (std::getline(f, line)) {
        if (line.find("facet") != std::string::npos) {
            found_facet = true;
            break;
        }
    }
    CHECK(found_facet);

    std::remove(path.c_str());
}

TEST_CASE("STL: empty mesh writes empty file") {
    sdf::TriangleMesh empty_mesh;
    const std::string path = "/tmp/test_cadforge_empty.stl";
    REQUIRE(io::export_stl_binary(path, empty_mesh, "empty"));

    std::ifstream f(path, std::ios::binary);
    REQUIRE(f.good());
    f.seekg(0, std::ios::end);
    const auto size = f.tellg();
    // Header (80) + triangle count (4) = 84 bytes
    CHECK(static_cast<std::size_t>(size) == 84);

    std::remove(path.c_str());
}
