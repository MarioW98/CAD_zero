// tests/unit/io/test_stl.cpp
#include <doctest/doctest.h>

#include "CAD_0/sdf/primitives.hpp"
#include "CAD_0/sdf/mesh_extract.hpp"
#include "CAD_0/io/stl.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace CAD_0;

TEST_CASE("STL: binary export writes valid file") {
    auto sph = sdf::make_sphere(1.0f);
    auto mesh = sdf::marching_cubes(sph, 16, true);

    const std::string path = "/tmp/test_CAD_0_sphere.stl";
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

    const std::string path = "/tmp/test_CAD_0_sphere_ascii.stl";
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
    const std::string path = "/tmp/test_CAD_0_empty.stl";
    REQUIRE(io::export_stl_binary(path, empty_mesh, "empty"));

    std::ifstream f(path, std::ios::binary);
    REQUIRE(f.good());
    f.seekg(0, std::ios::end);
    const auto size = f.tellg();
    // Header (80) + triangle count (4) = 84 bytes
    CHECK(static_cast<std::size_t>(size) == 84);

    std::remove(path.c_str());
}

// ---- Import (load_stl) -------------------------------------------------------

namespace {

// Deterministic normals-free octahedron: every coordinate is exactly
// representable through the ASCII %.6E path, so export->import->export
// can be compared BYTES-EXACTLY for both STL flavors.
sdf::TriangleMesh octahedron_no_normals() {
    sdf::TriangleMesh m;
    m.positions = {
        math::Vec3f{ 1.0f,  0.0f,  0.0f}, math::Vec3f{-1.0f,  0.0f,  0.0f},
        math::Vec3f{ 0.0f,  1.0f,  0.0f}, math::Vec3f{ 0.0f, -1.0f,  0.0f},
        math::Vec3f{ 0.0f,  0.0f,  1.0f}, math::Vec3f{ 0.0f,  0.0f, -1.0f},
    };
    m.indices = {
        0, 2, 4,  2, 1, 4,  1, 3, 4,  3, 0, 4,
        0, 3, 5,  2, 0, 5,  1, 2, 5,  3, 1, 5,
    };
    return m;
}

std::string read_bytes(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

} // namespace

TEST_CASE("STL: load binary welds duplicated vertices back (sphere)") {
    auto sph = sdf::make_sphere(1.0f);
    const sdf::TriangleMesh mesh = sdf::marching_cubes(sph, 16, true);

    const std::string path = "/tmp/test_CAD_0_import_bin.stl";
    REQUIRE(io::export_stl_binary(path, mesh, "sphere"));

    const sdf::TriangleMesh loaded = io::load_stl(path);

    // The binary STL stores 3 vertices per triangle (duplicated); the
    // loader's weld must restore exactly the original vertex count.
    CHECK(loaded.triangle_count() == mesh.triangle_count());
    CHECK(loaded.vertex_count() == mesh.vertex_count());

    // Positions are the same BITS (weld keeps original coordinates),
    // though in first-appearance order — compare as multisets.
    std::vector<math::Vec3f> a = mesh.positions, b = loaded.positions;
    const auto cmp = [](const math::Vec3f& l, const math::Vec3f& r) {
        if (l.x != r.x) return l.x < r.x;
        if (l.y != r.y) return l.y < r.y;
        return l.z < r.z;
    };
    std::sort(a.begin(), a.end(), cmp);
    std::sort(b.begin(), b.end(), cmp);
    CHECK(a == b);

    // Facet normals are advisory and ignored on import.
    CHECK(loaded.normals.empty());

    std::remove(path.c_str());
}

TEST_CASE("STL: load ascii sphere keeps counts and bounds") {
    auto sph = sdf::make_sphere(1.0f);
    const sdf::TriangleMesh mesh = sdf::marching_cubes(sph, 16, true);

    const std::string path = "/tmp/test_CAD_0_import_ascii.stl";
    REQUIRE(io::export_stl_ascii(path, mesh, "sphere"));

    const sdf::TriangleMesh loaded = io::load_stl(path);
    CHECK(loaded.triangle_count() == mesh.triangle_count());
    CHECK(loaded.vertex_count() == mesh.vertex_count());
    CHECK(loaded.normals.empty());

    // ASCII stores ~7 significant digits: geometry must survive within
    // float-printing tolerance, not bit-exactly.
    math::Bboxf bb;
    for (const auto& p : loaded.positions) bb.expand(p);
    CHECK(bb.min.x == doctest::Approx(-1.0f).epsilon(1e-3));
    CHECK(bb.max.x == doctest::Approx(1.0f).epsilon(1e-3));
    CHECK(bb.max.z == doctest::Approx(1.0f).epsilon(1e-3));

    std::remove(path.c_str());
}

TEST_CASE("STL: welding is idempotent on loaded meshes") {
    auto sph = sdf::make_sphere(1.0f);
    const sdf::TriangleMesh mesh = sdf::marching_cubes(sph, 16, true);
    const std::string path = "/tmp/test_CAD_0_import_idem.stl";
    REQUIRE(io::export_stl_binary(path, mesh, "sphere"));
    sdf::TriangleMesh loaded = io::load_stl(path);

    // Second weld: no-op (positions pairwise distinct post-weld).
    const std::size_t v1 = loaded.vertex_count();
    const std::size_t t1 = loaded.triangle_count();
    const std::size_t v2 = sdf::weld_vertices(loaded, 1e-6f);
    CHECK(v2 == v1);
    CHECK(loaded.triangle_count() == t1);

    std::remove(path.c_str());
}

TEST_CASE("STL: symmetry export -> import -> export is byte-identical") {
    const sdf::TriangleMesh m = octahedron_no_normals();

    const std::string bin1 = "/tmp/test_CAD_0_sym_bin1.stl";
    const std::string bin2 = "/tmp/test_CAD_0_sym_bin2.stl";
    const std::string asc1 = "/tmp/test_CAD_0_sym_asc1.stl";
    const std::string asc2 = "/tmp/test_CAD_0_sym_asc2.stl";

    REQUIRE(io::export_stl_binary(bin1, m, "sym"));
    const sdf::TriangleMesh back_bin = io::load_stl(bin1);
    REQUIRE(io::export_stl_binary(bin2, back_bin, "sym"));
    CHECK(read_bytes(bin1) == read_bytes(bin2));

    REQUIRE(io::export_stl_ascii(asc1, m, "sym"));
    const sdf::TriangleMesh back_asc = io::load_stl(asc1);
    REQUIRE(io::export_stl_ascii(asc2, back_asc, "sym"));
    CHECK(read_bytes(asc1) == read_bytes(asc2));

    std::remove(bin1.c_str()); std::remove(bin2.c_str());
    std::remove(asc1.c_str()); std::remove(asc2.c_str());
}

TEST_CASE("STL: empty mesh round-trips through binary") {
    const std::string path = "/tmp/test_CAD_0_empty_import.stl";
    REQUIRE(io::export_stl_binary(path, sdf::TriangleMesh{}, "empty"));
    const sdf::TriangleMesh loaded = io::load_stl(path);
    CHECK(loaded.vertex_count() == 0);
    CHECK(loaded.triangle_count() == 0);
    std::remove(path.c_str());
}

TEST_CASE("STL: corrupt files are explicit errors") {
    // Missing file.
    CHECK_THROWS_WITH_AS(io::load_stl("/tmp/test_CAD_0_no_such_file.stl"),
        doctest::Contains("cannot open"), std::runtime_error);

    // Truncated binary: the dimensional check fails, the ASCII fallback
    // fails ("solid" keyword missing) — never a silent partial load.
    auto sph = sdf::make_sphere(1.0f);
    const sdf::TriangleMesh mesh = sdf::marching_cubes(sph, 12, false);
    const std::string path = "/tmp/test_CAD_0_trunc.stl";
    REQUIRE(io::export_stl_binary(path, mesh, "trunc"));
    {
        const std::string whole = read_bytes(path);
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        out << whole.substr(0, whole.size() - 10);
    }
    CHECK_THROWS_AS(io::load_stl(path), std::runtime_error);
    std::remove(path.c_str());

    // Garbage text.
    const std::string garbage = "/tmp/test_CAD_0_garbage.stl";
    { std::ofstream out(garbage); out << "hello world, not an STL"; }
    CHECK_THROWS_WITH_AS(io::load_stl(garbage),
        doctest::Contains("must start with the \"solid\""), std::runtime_error);
    std::remove(garbage.c_str());

    // ASCII: a loop with only 2 vertices.
    const std::string two = "/tmp/test_CAD_0_twovtx.stl";
    { std::ofstream out(two);
      out << "solid bad\n"
             " facet normal 0 0 1\n"
             "  outer loop\n"
             "   vertex 0 0 0\n"
             "   vertex 1 0 0\n"
             "  endloop\n"
             " endfacet\n"
             "endsolid bad\n"; }
    CHECK_THROWS_WITH_AS(io::load_stl(two),
        doctest::Contains("exactly 3 vertices"), std::runtime_error);
    std::remove(two.c_str());

    // ASCII: malformed vertex number.
    const std::string badnum = "/tmp/test_CAD_0_badnum.stl";
    { std::ofstream out(badnum);
      out << "solid bad\n"
             " facet normal 0 0 1\n"
             "  outer loop\n"
             "   vertex 0 zero 0\n"
             "   vertex 1 0 0\n"
             "   vertex 0 1 0\n"
             "  endloop\n"
             " endfacet\n"
             "endsolid bad\n"; }
    CHECK_THROWS_WITH_AS(io::load_stl(badnum),
        doctest::Contains("malformed vertex number"), std::runtime_error);
    std::remove(badnum.c_str());

    // ASCII: truncated before endsolid.
    const std::string cut = "/tmp/test_CAD_0_cut.stl";
    { std::ofstream out(cut);
      out << "solid cut\n"
             " facet normal 0 0 1\n"
             "  outer loop\n"
             "   vertex 0 0 0\n"
             "   vertex 1 0 0\n"
             "   vertex 0 1 0\n"
             "  endloop\n"; }
    CHECK_THROWS_WITH_AS(io::load_stl(cut),
        doctest::Contains("expected \"endfacet\""), std::runtime_error);
    std::remove(cut.c_str());
}
