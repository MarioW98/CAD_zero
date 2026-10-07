// tests/unit/io/test_obj.cpp
#include <doctest/doctest.h>

#include "cadforge/sdf/primitives.hpp"
#include "cadforge/sdf/mesh_extract.hpp"
#include "cadforge/io/obj.hpp"

#include <cstdio>
#include <fstream>
#include <string>

using namespace cadforge;

TEST_CASE("OBJ: export writes valid file with vertices and faces") {
    auto sph = sdf::make_sphere(1.0f);
    auto mesh = sdf::marching_cubes(sph, 16, true);

    const std::string path = "/tmp/test_cadforge_obj.obj";
    REQUIRE(io::export_obj(path, mesh, "test_sphere"));

    std::ifstream f(path);
    REQUIRE(f.good());

    // Count vertex lines (v x y z) and face lines (f ...).
    std::size_t v_count = 0, vn_count = 0, f_count = 0;
    std::string line;
    while (std::getline(f, line)) {
        if (line.size() >= 2) {
            if (line[0] == 'v' && line[1] == ' ') ++v_count;
            else if (line[0] == 'v' && line[1] == 'n') ++vn_count;
            else if (line[0] == 'f' && line[1] == ' ') ++f_count;
        }
    }
    CHECK(v_count == mesh.vertex_count());
    CHECK(vn_count == mesh.vertex_count());  // we passed compute_normals=true
    CHECK(f_count == mesh.triangle_count());

    std::remove(path.c_str());
}

TEST_CASE("OBJ: export without normals") {
    auto sph = sdf::make_sphere(1.0f);
    sdf::MarchingCubesOptions opts;
    opts.resolution = 16;
    opts.compute_normals = false;
    opts.weld_vertices = true;
    auto mesh = sdf::marching_cubes(sph, sph.bounds(), opts);

    const std::string path = "/tmp/test_cadforge_obj_nonorm.obj";
    REQUIRE(io::export_obj(path, mesh));

    std::ifstream f(path);
    REQUIRE(f.good());

    std::size_t vn_count = 0;
    std::string line;
    while (std::getline(f, line)) {
        if (line.size() >= 2 && line[0] == 'v' && line[1] == 'n') ++vn_count;
    }
    CHECK(vn_count == 0);

    std::remove(path.c_str());
}

TEST_CASE("OBJ: 1-based indexing") {
    // Verify that OBJ indices are 1-based (not 0-based).
    sdf::TriangleMesh mesh;
    mesh.positions = {{0,0,0}, {1,0,0}, {0,1,0}};
    mesh.indices = {0, 1, 2};

    const std::string path = "/tmp/test_cadforge_obj_idx.obj";
    REQUIRE(io::export_obj(path, mesh, "test"));

    std::ifstream f(path);
    std::string line;
    while (std::getline(f, line)) {
        if (line.size() >= 2 && line[0] == 'f' && line[1] == ' ') {
            // Should be "f 1 2 3" (1-based), not "f 0 1 2"
            CHECK(line.find("f 1 2 3") == 0);
            break;
        }
    }
    std::remove(path.c_str());
}
