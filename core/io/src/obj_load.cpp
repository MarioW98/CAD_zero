// core/io/src/obj_load.cpp
//
// Wavefront OBJ import — position-indexed assembly + welding (ADR-0016).
// See obj.hpp for the supported-subset and error contract.
//
#include "CAD_0/io/obj.hpp"

#include <charconv>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace CAD_0::io {

namespace {

constexpr float kImportWeldTolerance = 1e-6f; // same policy as the STL loader

[[noreturn]] void obj_error(const std::string& path, int line,
                            const std::string& what) {
    throw std::runtime_error("OBJ loader: \"" + path + "\": line " +
                             std::to_string(line) + ": " + what);
}

float parse_float(std::string_view tok, const std::string& path, int line,
                  const char* context) {
    if (tok.empty())
        obj_error(path, line, std::string("expected a ") + context +
                                  " number, found end of token");
    float value = 0.0f;
    const char* first = tok.data();
    const char* last = first + tok.size();
    // from_chars: locale-independent, correctly rounded (ADR-0014).
    const auto [ptr, ec] = std::from_chars(first, last, value);
    if (ec != std::errc() || ptr != last)
        obj_error(path, line, std::string("malformed ") + context +
                                  " number \"" + std::string(tok) + "\"");
    return value;
}

// Parses an integer index; range/negativity validated by the caller's
// resolve step. Rejects non-numeric garbage here.
long parse_index(std::string_view tok, const std::string& path, int line,
                 const char* what) {
    if (tok.empty())
        obj_error(path, line, std::string("empty ") + what + " index");
    long value = 0;
    const char* first = tok.data();
    const char* last = first + tok.size();
    const auto [ptr, ec] = std::from_chars(first, last, value);
    if (ec != std::errc() || ptr != last)
        obj_error(path, line, std::string("malformed ") + what +
                                  " index \"" + std::string(tok) + "\"");
    return value;
}

// OBJ indices: 1-based; negative = relative to the current count.
std::size_t resolve_index(long i, std::size_t count, const std::string& path,
                          int line, const char* what) {
    if (i == 0)
        obj_error(path, line, std::string(what) +
                                  " index 0 — OBJ indices are 1-based");
    if (i > 0) {
        if (static_cast<std::size_t>(i) > count)
            obj_error(path, line, std::string(what) + " index " +
                                      std::to_string(i) + " out of range (" +
                                      std::to_string(count) + " defined)");
        return static_cast<std::size_t>(i - 1);
    }
    const long rel = static_cast<long>(count) + i; // e.g. -1 -> last defined
    if (rel < 0)
        obj_error(path, line, std::string(what) +
                                  " negative index " + std::to_string(i) +
                                  " points before the first " + what);
    return static_cast<std::size_t>(rel);
}

// One face corner after '/'-splitting: which references are present.
struct CornerRefs {
    std::size_t v;
    bool has_vn;
    std::size_t vn;
};

} // namespace

sdf::TriangleMesh load_obj(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in)
        obj_error(path, 0, "cannot open file for reading");
    std::ostringstream ss;
    ss << in.rdbuf();
    if (in.bad())
        obj_error(path, 0, "I/O error while reading");
    std::string text = ss.str();

    // Strip a UTF-8 BOM if present (some Windows exporters emit it).
    if (text.size() >= 3 && static_cast<unsigned char>(text[0]) == 0xEF &&
        static_cast<unsigned char>(text[1]) == 0xBB &&
        static_cast<unsigned char>(text[2]) == 0xBF)
        text = text.substr(3);

    std::vector<math::Vec3f> positions;
    std::vector<math::Vec3f> normals_src; // vn pool
    bool any_vn = false;

    // Corners (v resolved; vn as pool index when present). Kept so the
    // per-position normal assignment can happen after all lines are read.
    struct FaceCorner {
        std::size_t v;
        std::int64_t vn; // -1 = the corner carries no vn reference
    };
    std::vector<FaceCorner> corners; // flattened, in declaration order

    std::size_t vt_count = 0; // parsed and discarded (see obj.hpp)

    // Line-by-line scan.
    std::size_t line_start = 0;
    int line_no = 0;
    const std::size_t n = text.size();
    while (line_start <= n) {
        ++line_no;
        std::size_t line_end = text.find('\n', line_start);
        if (line_end == std::string::npos) line_end = n;
        std::string_view line(text.data() + line_start, line_end - line_start);
        line_start = line_end + 1;

        // Trim \r and leading/trailing blanks; skip comments and blanks.
        while (!line.empty() && (line.front() == ' ' || line.front() == '\t' ||
                                 line.front() == '\r'))
            line.remove_prefix(1);
        while (!line.empty() && (line.back() == '\r' || line.back() == ' ' ||
                                 line.back() == '\t'))
            line.remove_suffix(1);
        if (line.empty() || line.front() == '#')
            continue;

        // First token = keyword.
        std::size_t k = line.find_first_of(" \t");
        if (k == std::string::npos) k = line.size();
        const std::string_view kw = line.substr(0, k);
        std::string_view rest =
            k < line.size() ? line.substr(k + 1) : std::string_view{};

        // Token splitter over the remainder.
        const auto next_token = [&rest]() -> std::string_view {
            while (!rest.empty() && (rest.front() == ' ' || rest.front() == '\t'))
                rest.remove_prefix(1);
            const std::size_t e = rest.find_first_of(" \t");
            const std::string_view tok =
                e == std::string::npos ? rest : rest.substr(0, e);
            rest.remove_prefix(e == std::string::npos ? rest.size() : e + 1);
            return tok;
        };

        if (kw == "v") {
            const float x = parse_float(next_token(), path, line_no, "vertex");
            const float y = parse_float(next_token(), path, line_no, "vertex");
            const float z = parse_float(next_token(), path, line_no, "vertex");
            // A 4th component (w, homogeneous weight) is legal OBJ —
            // parsed-and-ignored is not safe, so require it to be absent.
            if (!next_token().empty())
                obj_error(path, line_no,
                          "vertex has a 4th component (w) — CAD_0 loads "
                          "3-component positions only");
            positions.push_back(math::Vec3f{x, y, z});
        } else if (kw == "vn") {
            const float x = parse_float(next_token(), path, line_no, "normal");
            const float y = parse_float(next_token(), path, line_no, "normal");
            const float z = parse_float(next_token(), path, line_no, "normal");
            normals_src.push_back(math::Vec3f{x, y, z});
        } else if (kw == "vt") {
            // Texture coordinates: parsed for index bookkeeping (negative
            // relative indices in faces depend on the running count) and
            // validated, then discarded — CAD_0 has no UV support.
            std::string_view tok = next_token();
            if (!tok.empty()) (void)parse_float(tok, path, line_no, "vt");
            tok = next_token();
            if (!tok.empty()) (void)parse_float(tok, path, line_no, "vt");
            tok = next_token();
            if (!tok.empty()) (void)parse_float(tok, path, line_no, "vt");
            if (!next_token().empty())
                obj_error(path, line_no,
                          "vt has more than 3 components — malformed line");
            ++vt_count;
        } else if (kw == "f") {
            // Corners: v | v/vt | v//vn | v/vt/vn.
            std::vector<CornerRefs> refs;
            while (true) {
                const std::string_view tok = next_token();
                if (tok.empty()) break;
                if (tok.find('/') == std::string_view::npos) {
                    const long vi =
                        parse_index(tok, path, line_no, "vertex");
                    refs.push_back(CornerRefs{
                        resolve_index(vi, positions.size(), path, line_no,
                                      "vertex"),
                        false, 0});
                    continue;
                }
                // Split on '/' (up to 3 fields: v, vt, vn).
                std::string_view fields[3];
                int nfields = 0;
                std::size_t pos = 0;
                bool too_many = false;
                while (true) {
                    const std::size_t e = tok.find('/', pos);
                    if (e == std::string_view::npos) {
                        if (nfields < 3)
                            fields[nfields++] = tok.substr(pos);
                        else
                            too_many = true;
                        break;
                    }
                    if (nfields >= 3) {
                        too_many = true;
                        break;
                    }
                    fields[nfields++] = tok.substr(pos, e - pos);
                    pos = e + 1;
                }
                if (too_many || nfields < 2)
                    obj_error(path, line_no,
                              "malformed face corner \"" + std::string(tok) +
                                  "\" (expected v, v/vt, v//vn or v/vt/vn)");
                const long vi = parse_index(fields[0], path, line_no, "vertex");
                CornerRefs r{resolve_index(vi, positions.size(), path, line_no,
                                           "vertex"),
                             false, 0};
                if (!fields[1].empty()) {
                    const long ti = parse_index(fields[1], path, line_no, "vt");
                    (void)resolve_index(static_cast<long>(ti), vt_count, path,
                                        line_no, "vt"); // validated, discarded
                }
                if (nfields == 3 && !fields[2].empty()) {
                    const long ni = parse_index(fields[2], path, line_no, "vn");
                    r.vn = resolve_index(ni, normals_src.size(), path, line_no,
                                         "vn");
                    r.has_vn = true;
                    any_vn = true;
                }
                refs.push_back(r);
            }
            if (refs.size() < 3)
                obj_error(path, line_no, "face needs at least 3 corners, "
                                         "found " + std::to_string(refs.size()));
            // n-gon -> triangle fan (c0, ci, ci+1). Every corner keeps
            // its own vn reference — the apex too, or its position could
            // end up without an assigned normal.
            for (std::size_t i = 1; i + 1 < refs.size(); ++i) {
                corners.push_back(
                    FaceCorner{refs[0].v,
                               refs[0].has_vn
                                   ? static_cast<std::int64_t>(refs[0].vn)
                                   : std::int64_t(-1)});
                corners.push_back(
                    FaceCorner{refs[i].v,
                               refs[i].has_vn
                                   ? static_cast<std::int64_t>(refs[i].vn)
                                   : std::int64_t(-1)});
                corners.push_back(
                    FaceCorner{refs[i + 1].v,
                               refs[i + 1].has_vn
                                   ? static_cast<std::int64_t>(refs[i + 1].vn)
                                   : std::int64_t(-1)});
            }
        } else if (kw == "o" || kw == "g" || kw == "s" || kw == "usemtl" ||
                   kw == "mtllib") {
            // Organizational statements carry no geometry — skipped.
        } else {
            obj_error(path, line_no,
                      "unsupported keyword \"" + std::string(kw) +
                          "\" (CAD_0 loads v/vn/vt/f only; geometry-carrying "
                          "statements like curv/surf/vp would be silently "
                          "dropped, so they are rejected)");
        }
    }

    // Assemble the mesh, position-indexed.
    sdf::TriangleMesh mesh;
    mesh.positions = positions;
    mesh.indices.reserve(corners.size());
    for (const auto& c : corners)
        mesh.indices.push_back(static_cast<std::uint32_t>(c.v));

    // Per-position normals: first reference wins (see obj.hpp).
    if (any_vn) {
        mesh.normals.assign(positions.size(), math::Vec3f{0.0f, 0.0f, 0.0f});
        std::vector<char> assigned(positions.size(), 0);
        for (const auto& c : corners) {
            if (c.vn < 0 || assigned[c.v]) continue;
            mesh.normals[c.v] = normals_src[static_cast<std::size_t>(c.vn)];
            assigned[c.v] = 1;
        }
    }

    // ADR-0016 weld: idempotent no-op on a well-formed OBJ (shared
    // indices, no duplicate v declarations); deduplicates duplicates.
    sdf::weld_vertices(mesh, kImportWeldTolerance);
    return mesh;
}

} // namespace CAD_0::io
