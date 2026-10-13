// tests/unit/io/test_obj.cpp
#include <doctest/doctest.h>

#include "CAD_0/sdf/primitives.hpp"
#include "CAD_0/sdf/mesh_extract.hpp"
#include "CAD_0/io/obj.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

using namespace CAD_0;

TEST_CASE("OBJ: export writes valid file with vertices and faces") {
    auto sph = sdf::make_sphere(1.0f);
    auto mesh = sdf::marching_cubes(sph, 16, true);

    const std::string path = "/tmp/test_CAD_0_obj.obj";
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

    const std::string path = "/tmp/test_CAD_0_obj_nonorm.obj";
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

    const std::string path = "/tmp/test_CAD_0_obj_idx.obj";
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

// ---- Import (load_obj) -------------------------------------------------------

namespace {

// Deterministic octahedron with and without normals — every coordinate
// is exactly representable through the ASCII %.6E path, so the symmetry
// test can compare files BYTES-EXACTLY.
sdf::TriangleMesh octa(bool with_normals) {
    sdf::TriangleMesh m;
    m.positions = {
        math::Vec3f{ 1.0f,  0.0f,  0.0f}, math::Vec3f{-1.0f,  0.0f,  0.0f},
        math::Vec3f{ 0.0f,  1.0f,  0.0f}, math::Vec3f{ 0.0f, -1.0f,  0.0f},
        math::Vec3f{ 0.0f,  0.0f,  1.0f}, math::Vec3f{ 0.0f,  0.0f, -1.0f},
    };
    if (with_normals) m.normals = m.positions;
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

void write_text(const std::string& path, const char* text) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    out << text;
}

} // namespace

TEST_CASE("OBJ: load round-trips an exported sphere (positions, normals, faces)") {
    auto sph = sdf::make_sphere(1.0f);
    const sdf::TriangleMesh mesh = sdf::marching_cubes(sph, 16, true);

    const std::string path = "/tmp/test_CAD_0_import.obj";
    REQUIRE(io::export_obj(path, mesh, "sphere"));
    const sdf::TriangleMesh loaded = io::load_obj(path);

    // OBJ indices are shared: the weld is a no-op and the arrays survive
    // in the same order (ASCII precision aside).
    CHECK(loaded.vertex_count() == mesh.vertex_count());
    CHECK(loaded.triangle_count() == mesh.triangle_count());
    REQUIRE(loaded.normals.size() == mesh.normals.size());
    float max_delta = 0.0f;
    for (std::size_t i = 0; i < mesh.positions.size(); ++i) {
        max_delta = std::max(max_delta,
            std::abs(loaded.positions[i].x - mesh.positions[i].x));
        max_delta = std::max(max_delta,
            std::abs(loaded.positions[i].y - mesh.positions[i].y));
    }
    CHECK(max_delta < 1e-4f); // %.6E carries ~7 significant digits

    std::remove(path.c_str());
}

TEST_CASE("OBJ: n-gon fan triangulation and negative relative indices") {
    const std::string path = "/tmp/test_CAD_0_ngon.obj";
    write_text(path,
        "v 0 0 0\n"
        "v 1 0 0\n"
        "v 1 1 0\n"
        "v 0 1 0\n"
        "v 0.5 1.5 0\n"
        "f 1 2 3 4 5\n"      // pentagon -> 3 fan triangles
        "f -1 -2 -3\n");     // relative: v5, v4, v3

    const sdf::TriangleMesh m = io::load_obj(path);
    REQUIRE(m.vertex_count() == 5);
    REQUIRE(m.triangle_count() == 4);

    // Fan: (0,1,2), (0,2,3), (0,3,4); negative face -> (4,3,2).
    const std::uint32_t want[12] = {
        0,1,2,  0,2,3,  0,3,4,  4,3,2,
    };
    for (int i = 0; i < 12; ++i)
        CHECK(m.indices[static_cast<std::size_t>(i)] == want[i]);
    CHECK(m.normals.empty());

    std::remove(path.c_str());
}

TEST_CASE("OBJ: corner formats v, v/vt, v//vn, v/vt/vn; vt discarded") {
    const std::string path = "/tmp/test_CAD_0_corners.obj";
    write_text(path,
        "# mixed corner styles\n"
        "v 0 0 0\n"
        "v 1 0 0\n"
        "v 0 1 0\n"
        "vt 0.5 0.5\n"       // parsed, validated, discarded
        "vt 0.25 0.75\n"
        "vn 0 0 1\n"
        "vn 0 1 0\n"
        "f 1 2/1 3//1\n"     // v | v/vt | v//vn
        "f 1/2/2 2 3\n");    // v/vt/vn | v | v

    const sdf::TriangleMesh m = io::load_obj(path);
    CHECK(m.vertex_count() == 3);
    CHECK(m.triangle_count() == 2);
    REQUIRE(m.normals.size() == 3);
    // First reference wins: pos 0 gets vn2 (from the second face's
    // 1/2/2? no: corner (v=0, vn=2) appears in face 2 -> normals[0]=vn[1]
    // only if face 1's corner for v0 carried no vn — it did not).
    CHECK(m.normals[0].y == 1.0f); // vn 2 = (0,1,0)
    CHECK(m.normals[2].z == 1.0f); // vn 1 = (0,0,1)
    CHECK(m.normals[1] == math::Vec3f{0.0f, 0.0f, 0.0f}); // never referenced

    std::remove(path.c_str());
}

TEST_CASE("OBJ: duplicate v declarations are welded") {
    const std::string path = "/tmp/test_CAD_0_dupv.obj";
    write_text(path,
        "v 0 0 0\n"
        "v 1 0 0\n"
        "v 0 1 0\n"
        "v 1 0 0\n"          // duplicate of v2 — bit-exact
        "f 1 2 3\n"
        "f 1 4 3\n");

    const sdf::TriangleMesh m = io::load_obj(path);
    CHECK(m.vertex_count() == 3);   // welded
    CHECK(m.triangle_count() == 2);
    CHECK(m.positions[1] == math::Vec3f{1.0f, 0.0f, 0.0f});

    std::remove(path.c_str());
}

TEST_CASE("OBJ: welding is idempotent on loaded meshes") {
    const std::string path = "/tmp/test_CAD_0_idem.obj";
    REQUIRE(io::export_obj(path, octa(true), "idem"));
    sdf::TriangleMesh m = io::load_obj(path);
    const std::size_t v1 = m.vertex_count();
    const std::size_t t1 = m.triangle_count();
    CHECK(sdf::weld_vertices(m, 1e-6f) == v1);
    CHECK(m.triangle_count() == t1);
    std::remove(path.c_str());
}

TEST_CASE("OBJ: symmetry export -> import -> export is byte-identical") {
    for (const bool with_normals : {false, true}) {
        const std::string p1 = "/tmp/test_CAD_0_sym1.obj";
        const std::string p2 = "/tmp/test_CAD_0_sym2.obj";
        REQUIRE(io::export_obj(p1, octa(with_normals), "sym"));
        const sdf::TriangleMesh back = io::load_obj(p1);
        REQUIRE(back.vertex_count() == octa(with_normals).vertex_count());
        REQUIRE(io::export_obj(p2, back, "sym"));
        CHECK(read_bytes(p1) == read_bytes(p2));
        std::remove(p1.c_str());
        std::remove(p2.c_str());
    }
}

TEST_CASE("OBJ: corrupt files are explicit errors") {
    // Missing file.
    CHECK_THROWS_WITH_AS(io::load_obj("/tmp/test_CAD_0_no_such.obj"),
        doctest::Contains("cannot open"), std::runtime_error);

    // Zero index (OBJ is 1-based).
    std::string p = "/tmp/test_CAD_0_e0.obj";
    write_text(p, "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 0 1 2\n");
    CHECK_THROWS_WITH_AS(io::load_obj(p),
        doctest::Contains("1-based"), std::runtime_error);

    // Out of range.
    write_text(p, "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 4\n");
    CHECK_THROWS_WITH_AS(io::load_obj(p),
        doctest::Contains("out of range"), std::runtime_error);

    // Negative index pointing before the first vertex.
    write_text(p, "v 0 0 0\nf -4 -3 -2\n");
    CHECK_THROWS_WITH_AS(io::load_obj(p),
        doctest::Contains("before the first"), std::runtime_error);

    // Face with 2 corners.
    write_text(p, "v 0 0 0\nv 1 0 0\nf 1 2\n");
    CHECK_THROWS_WITH_AS(io::load_obj(p),
        doctest::Contains("at least 3 corners"), std::runtime_error);

    // Geometry-carrying keyword we cannot represent.
    write_text(p, "v 0 0 0\nv 1 0 0\nv 0 1 0\ncurv 1 2 0.5\n");
    CHECK_THROWS_WITH_AS(io::load_obj(p),
        doctest::Contains("unsupported keyword \"curv\""), std::runtime_error);

    // Malformed number.
    write_text(p, "v 0 0 zzz\n");
    CHECK_THROWS_WITH_AS(io::load_obj(p),
        doctest::Contains("malformed vertex number"), std::runtime_error);

    // Vertex with a 4th (w) component — rejected, not silently dropped.
    write_text(p, "v 0 0 0 1\n");
    CHECK_THROWS_WITH_AS(io::load_obj(p),
        doctest::Contains("4th component"), std::runtime_error);

    // Organizational statements are fine (no error, no geometry).
    write_text(p,
        "o part\ng group\ns 1\nmtllib parts.mtl\nusemtl steel\n"
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n");
    CHECK(io::load_obj(p).triangle_count() == 1);

    std::remove(p.c_str());
}
