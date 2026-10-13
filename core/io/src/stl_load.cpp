// core/io/src/stl_load.cpp
//
// STL import — binary and ASCII, with vertex welding (ADR-0016).
// See stl.hpp for the format-detection and error contract.
//
#include "CAD_0/io/stl.hpp"

#include <bit>
#include <charconv>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace CAD_0::io {

namespace {

// Weld tolerance for imports: merges STL's bit-exact per-triangle
// duplicates and any vertices closer than 1e-6 mesh units. Idempotent on
// already-welded meshes (ADR-0016).
constexpr float kImportWeldTolerance = 1e-6f;

// ---- little-endian scalars (byte-by-byte: endian-independent) ---------------

std::uint32_t read_u32_le(const char* p) {
    const auto b0 = static_cast<std::uint32_t>(static_cast<unsigned char>(p[0]));
    const auto b1 = static_cast<std::uint32_t>(static_cast<unsigned char>(p[1]));
    const auto b2 = static_cast<std::uint32_t>(static_cast<unsigned char>(p[2]));
    const auto b3 = static_cast<std::uint32_t>(static_cast<unsigned char>(p[3]));
    return b0 | (b1 << 8) | (b2 << 16) | (b3 << 24);
}

// Floats travel as raw IEEE-754 bit patterns — no arithmetic, no
// endianness assumptions, bit-exact round-trips (ADR-0014).
float read_f32_le(const char* p) {
    return std::bit_cast<float>(read_u32_le(p));
}

[[noreturn]] void stl_error(const std::string& path, const std::string& what) {
    throw std::runtime_error("STL loader: \"" + path + "\": " + what);
}

std::string read_whole_file(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in)
        stl_error(path, "cannot open file for reading");
    std::ostringstream ss;
    ss << in.rdbuf();
    if (in.bad())
        stl_error(path, "I/O error while reading");
    return ss.str();
}

// ---- ASCII tokenizer ----------------------------------------------------------

struct AsciiCursor {
    const char* p;
    const char* end;
    int line = 1;

    void advance() {
        if (p != end) {
            if (*p == '\n') ++line;
            ++p;
        }
    }
    // Skip whitespace; returns false at end of input.
    bool skip_ws() {
        while (p != end) {
            const char c = *p;
            if (c == ' ' || c == '\t' || c == '\r' || c == '\n')
                advance();
            else
                return true;
        }
        return false;
    }
    // Read a token (non-whitespace run). Empty at end of input.
    std::string_view token() {
        skip_ws();
        const char* start = p;
        while (p != end && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n')
            ++p;
        return std::string_view(start, static_cast<std::size_t>(p - start));
    }
};

void skip_to_eol(AsciiCursor& cur) {
    // Skip an optional trailing token run (e.g. the solid's name) up to
    // the newline; the newline itself is left for token() to consume (so
    // the line counter stays correct).
    while (cur.p != cur.end && *cur.p != '\n') ++cur.p;
}

bool iequals(std::string_view a, std::string_view b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i)
        if (std::tolower(static_cast<unsigned char>(a[i])) !=
            std::tolower(static_cast<unsigned char>(b[i])))
            return false;
    return true;
}

float parse_float(AsciiCursor& cur, const std::string& path,
                  const char* context) {
    const std::string_view tok = cur.token();
    if (tok.empty())
        stl_error(path, "line " + std::to_string(cur.line) + ": expected a " +
                            std::string(context) +
                            " number, found end of input");
    float value = 0.0f;
    const char* first = tok.data();
    const char* last = first + tok.size();
    // from_chars: locale-independent, correctly rounded (ADR-0014).
    const auto [ptr, ec] = std::from_chars(first, last, value);
    if (ec != std::errc() || ptr != last)
        stl_error(path, "line " + std::to_string(cur.line) + ": malformed " +
                            std::string(context) + " number \"" +
                            std::string(tok) + "\"");
    return value;
}

// ---- binary path ---------------------------------------------------------------

sdf::TriangleMesh parse_binary(const std::string& bytes,
                                const std::string& path) {
    const std::size_t tri_count = read_u32_le(bytes.data() + 80);

    sdf::TriangleMesh mesh;
    mesh.positions.reserve(tri_count * 3);
    mesh.indices.reserve(tri_count * 3);
    const char* q = bytes.data() + 84;
    for (std::size_t t = 0; t < tri_count; ++t) {
        q += 12; // facet normal: advisory, ignored (see stl.hpp)
        for (int c = 0; c < 3; ++c) {
            mesh.positions.push_back(
                math::Vec3f{read_f32_le(q), read_f32_le(q + 4), read_f32_le(q + 8)});
            q += 12;
            mesh.indices.push_back(
                static_cast<std::uint32_t>(mesh.positions.size() - 1));
        }
        q += 2; // attribute byte count: ignored (some writers store color)
    }
    (void)path;
    return mesh;
}

// ---- ASCII path ----------------------------------------------------------------

sdf::TriangleMesh parse_ascii(const std::string& bytes, const std::string& path) {
    AsciiCursor cur{bytes.data(), bytes.data() + bytes.size(), 1};

    sdf::TriangleMesh mesh;
    std::string_view tok = cur.token();
    if (!iequals(tok, "solid"))
        stl_error(path, "line 1: ASCII STL must start with the \"solid\" "
                        "keyword (found \"" + std::string(tok) + "\")");
    // "solid" may carry a name on the same line — skip it.
    skip_to_eol(cur);

    bool endsolid_seen = false;
    while (true) {
        tok = cur.token();
        if (tok.empty()) {
            if (!endsolid_seen)
                stl_error(path, "unexpected end of input: \"endsolid\" not "
                                "reached (truncated file)");
            break;
        }
        if (iequals(tok, "endsolid")) {
            endsolid_seen = true;
            // Trailing content after endsolid is tolerated (some exporters
            // emit trailing whitespace/newlines).
            break;
        }
        if (iequals(tok, "solid"))
            stl_error(path, "line " + std::to_string(cur.line) +
                                ": nested \"solid\" (multiple solids in one "
                                "file are not supported — import them "
                                "separately)");

        if (!iequals(tok, "facet"))
            stl_error(path, "line " + std::to_string(cur.line) +
                                ": unexpected token \"" + std::string(tok) +
                                "\" (expected \"facet\" or \"endsolid\")");

        // facet normal nx ny nz  (values advisory — parsed for validation,
        // then ignored, matching the binary path's contract)
        tok = cur.token();
        if (!iequals(tok, "normal"))
            stl_error(path, "line " + std::to_string(cur.line) +
                                ": expected \"normal\" after \"facet\"");
        (void)parse_float(cur, path, "facet normal");
        (void)parse_float(cur, path, "facet normal");
        (void)parse_float(cur, path, "facet normal");

        tok = cur.token();
        if (!iequals(tok, "outer"))
            stl_error(path, "line " + std::to_string(cur.line) +
                                ": expected \"outer loop\"");
        tok = cur.token();
        if (!iequals(tok, "loop"))
            stl_error(path, "line " + std::to_string(cur.line) +
                                ": expected \"loop\" after \"outer\"");

        std::size_t corners = 0;
        while (true) {
            tok = cur.token();
            if (iequals(tok, "endloop"))
                break;
            if (tok.empty())
                stl_error(path, "unexpected end of input inside a facet loop "
                                "(truncated file)");
            if (!iequals(tok, "vertex"))
                stl_error(path, "line " + std::to_string(cur.line) +
                                    ": unexpected token \"" + std::string(tok) +
                                    "\" inside loop (expected \"vertex\" or "
                                    "\"endloop\")");
            const math::Vec3f v{parse_float(cur, path, "vertex"),
                                parse_float(cur, path, "vertex"),
                                parse_float(cur, path, "vertex")};
            mesh.positions.push_back(v);
            mesh.indices.push_back(
                static_cast<std::uint32_t>(mesh.positions.size() - 1));
            ++corners;
        }
        if (corners != 3)
            stl_error(path, "line " + std::to_string(cur.line) +
                                ": an STL facet must have exactly 3 vertices, "
                                "found " + std::to_string(corners));

        tok = cur.token();
        if (!iequals(tok, "endfacet"))
            stl_error(path, "line " + std::to_string(cur.line) +
                                ": expected \"endfacet\", found \"" +
                                std::string(tok) + "\"");
    }
    return mesh;
}

} // namespace

sdf::TriangleMesh load_stl(const std::string& path) {
    const std::string bytes = read_whole_file(path);

    // DIMENSIONAL detection first (see stl.hpp): a file whose size is
    // exactly 84 + 50*n with n = LE uint32 at offset 80 is binary.
    if (bytes.size() >= 84) {
        const std::uint64_t n = read_u32_le(bytes.data() + 80);
        if (84 + 50ull * n == static_cast<std::uint64_t>(bytes.size())) {
            sdf::TriangleMesh mesh = parse_binary(bytes, path);
            sdf::weld_vertices(mesh, kImportWeldTolerance);
            return mesh;
        }
    }

    // ASCII fallback; anything that fails inside reports with line info.
    if (!bytes.empty() && bytes.front() == '\xEF') {
        // UTF-8 BOM before "solid" — tolerated by skipping it.
        if (bytes.size() >= 3 && static_cast<unsigned char>(bytes[1]) == 0xBB &&
            static_cast<unsigned char>(bytes[2]) == 0xBF)
            return parse_ascii(bytes.substr(3), path);
    }
    sdf::TriangleMesh mesh = parse_ascii(bytes, path);
    sdf::weld_vertices(mesh, kImportWeldTolerance);
    return mesh;
}

} // namespace CAD_0::io
