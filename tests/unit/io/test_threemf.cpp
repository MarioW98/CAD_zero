// tests/unit/io/test_threemf.cpp
#include <doctest/doctest.h>

#include "CAD_0/sdf/primitives.hpp"
#include "CAD_0/sdf/mesh_extract.hpp"
#include "CAD_0/io/threemf.hpp"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

using namespace CAD_0;

TEST_CASE("3MF: export writes valid ZIP archive") {
    auto sph = sdf::make_sphere(1.0f);
    auto mesh = sdf::marching_cubes(sph, 16, true);

    const std::string path = "/tmp/test_CAD_0_3mf.3mf";
    REQUIRE(io::export_3mf(path, mesh, "test_sphere"));

    std::ifstream f(path, std::ios::binary);
    REQUIRE(f.good());

    // Read the file into a buffer.
    std::vector<unsigned char> buf;
    f.seekg(0, std::ios::end);
    const auto size = f.tellg();
    f.seekg(0, std::ios::beg);
    buf.resize(size);
    f.read(reinterpret_cast<char*>(buf.data()), size);

    // ZIP files start with the local file header signature 0x04034b50 ("PK\x03\x04").
    REQUIRE(buf.size() >= 4);
    CHECK(buf[0] == 0x50);
    CHECK(buf[1] == 0x4B);
    CHECK(buf[2] == 0x03);
    CHECK(buf[3] == 0x04);

    // End of central directory record signature 0x06054b50 should be in the
    // last 22 bytes (or more if there's a comment, which we don't write).
    REQUIRE(buf.size() >= 22);
    const std::size_t eocd_pos = buf.size() - 22;
    CHECK(buf[eocd_pos + 0] == 0x50);
    CHECK(buf[eocd_pos + 1] == 0x4B);
    CHECK(buf[eocd_pos + 2] == 0x05);
    CHECK(buf[eocd_pos + 3] == 0x06);

    std::remove(path.c_str());
}

TEST_CASE("3MF: empty mesh returns false") {
    sdf::TriangleMesh empty;
    const std::string path = "/tmp/test_CAD_0_3mf_empty.3mf";
    CHECK_FALSE(io::export_3mf(path, empty));
}

TEST_CASE("3MF: archive contains expected entries") {
    auto sph = sdf::make_sphere(1.0f);
    auto mesh = sdf::marching_cubes(sph, 8, false);

    const std::string path = "/tmp/test_CAD_0_3mf_entries.3mf";
    REQUIRE(io::export_3mf(path, mesh));

    std::ifstream f(path, std::ios::binary);
    REQUIRE(f.good());
    std::vector<unsigned char> buf((std::istreambuf_iterator<char>(f)),
                                    std::istreambuf_iterator<char>());

    // Look for filenames in the archive (each ZIP entry stores the filename
    // in the local header and again in the central directory).
    auto find_substring = [&](const char* needle) -> bool {
        const std::size_t n = std::strlen(needle);
        if (buf.size() < n) return false;
        for (std::size_t i = 0; i + n <= buf.size(); ++i) {
            if (std::memcmp(buf.data() + i, needle, n) == 0) return true;
        }
        return false;
    };

    CHECK(find_substring("[Content_Types].xml"));
    CHECK(find_substring("_rels/.rels"));
    CHECK(find_substring("3D/3dmodel.model"));
    CHECK(find_substring("<model unit=\"millimeter\""));
    CHECK(find_substring("<vertices>"));
    CHECK(find_substring("<triangles>"));

    std::remove(path.c_str());
}
