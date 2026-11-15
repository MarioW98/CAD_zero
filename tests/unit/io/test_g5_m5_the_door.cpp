// tests/unit/io/test_g5_m5_the_door.cpp
//
// G5 m5 — LA PORTA v0.8.0 (ADR-0043 D6.5/D6.6): the contemporanea's
// own doors, self-contained beside the corpus proofs in
// test_native.cpp (the golden's walk, the additivity, the round-trip
// — those need the corpus builders; these need only the door):
//
//   * THE ANACHRONISM: a fillet_multi node in a file declaring
//     < 0.8 — the hand-edited lie, the fillet's own gate form ("no
//     0.7 writer ever emitted one: the file is not what its version
//     claims").
//   * THE VALUE CLASS at the payload's fields: the entries array
//     non-empty, each edge a non-empty string, each radius finite
//     and > 0 — corruption of the class is a LOUD load error, never
//     a default.
//   * THE MAP'S UNIQUENESS: the same edge string twice is not a map.
//   * THE NEVER-MIX: a fillet_multi node carrying the single's
//     field (or the SDF's), and the single carrying the multi's.
//   * THE MISMATCH AT THE DUAL POINTS (D6.6's own register: the law
//     read at the WRITE and at the re-materialization, two doors one
//     law, NEVER at the kernel): a hand-authored 0.8 file whose
//     twelve entries carry ONE disagreeing radius parses WHOLE (the
//     syntactic class is fine — the law is not the parse's to
//     judge), loads into the VISIBLE refusal state (the machine's
//     own words, D6.6's name, the last good state kept — no body
//     ever materialized), and the SAVE of that scene is REFUSED
//     with the same name — this build never blesses a
//     self-incoherent map with a file. The refusal's provenance is
//     the MACHINE's prefix both times: the kernel is never asked the
//     question the map already answers.
//   * THE PADDING/ABI COUNTERPROOF (the m4 fillet's own subject, one
//     map over): the entry carries a std::string — a naive raw-struct
//     image would carry the string's HEAP ADDRESS (the ABI), not its
//     characters: two same-content entries at different addresses,
//     two different raw digests; the named-fields writer stores the
//     CHARACTERS, and two same-content recipes ride it to IDENTICAL
//     file bytes.
#include <doctest/doctest.h>

#include "CAD_0/brep/fillet.hpp"
#include "CAD_0/io/native.hpp"
#include "CAD_0/scene/scene.hpp"
#include "CAD_0/sdf/primitives.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace scene = CAD_0::scene;
namespace brep = CAD_0::brep;
namespace math = CAD_0::math;
namespace native = CAD_0::io::native;

namespace {

// The minimal honest operand: a fitted box node (id 1) — the same
// form the corpus builders use, hand-written so the DOOR tests carry
// their own document (no corpus dependency).
std::string multi_doc(const std::string& entries_json,
                      const char* version) {
    std::string body =
        "{\"format\": \"CAD_0 native\", \"version\": \"";
    body += version;
    body += "\", \"scene\": { \"next_id\": 3, \"nodes\": ["
            "  { \"id\": 1, \"name\": \"Cube\","
            "    \"body\": { \"op\": \"box\","
            "               \"extent\": [1.0, 1.0, 1.0] } },"
            "  { \"id\": 2, \"name\": \"Twelve\","
            "    \"kind\": \"fillet_multi\","
            "    \"fillet_multi\": { \"edges\": [";
    body += entries_json;
    body += "] } } ] } }";
    return body;
}

// The twelve entries at one radius (or one disagreeing vertical).
std::string twelve_entries(double r, double odd_radius) {
    const char* toks[12] = {"bottom0", "bottom1", "bottom2", "bottom3",
                            "top0", "top1", "top2", "top3",
                            "vertical0", "vertical1", "vertical2",
                            "vertical3"};
    std::string out;
    for (int k = 0; k < 12; ++k) {
        const double radius = (odd_radius > 0.0 && k == 8)
                                  ? odd_radius
                                  : r;
        out += (k ? ", " : "");
        out += "{ \"edge\": \"box#1:";
        out += toks[k];
        out += "\", \"radius\": ";
        out += std::to_string(radius);
        out += " }";
    }
    return out;
}

std::string read_file(const std::filesystem::path& p) {
    std::ifstream in(p, std::ios::binary);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

// fnv1a over raw bytes (the digest the counterproofs speak).
std::string fnv1a_hex(const unsigned char* data, std::size_t n) {
    std::uint64_t h = 1469598103934665603ULL;
    for (std::size_t i = 0; i < n; ++i) {
        h ^= data[i];
        h *= 1099511628211ULL;
    }
    char buf[17];
    std::snprintf(buf, sizeof(buf), "%016llx",
                  static_cast<unsigned long long>(h));
    return std::string(buf);
}

}  // namespace

TEST_SUITE("io native format G5 m5") {

// ---------------------------------------------------------------------------
// THE ANACHRONISM — the hand-edited lie: no 0.7 writer ever emitted a
// fillet_multi node.
// ---------------------------------------------------------------------------
TEST_CASE("G5 m5 the anachronism: a fillet_multi node in a file "
          "declaring 0.7 is not what its version claims") {
    const std::string doc = multi_doc(twelve_entries(0.5, -1.0),
                                      "0.7.0");
    CHECK_THROWS_WITH_AS(native::parse_document(doc),
        doctest::Contains("fillet_multi"), std::runtime_error);
    CHECK_THROWS_WITH_AS(native::parse_document(doc),
        doctest::Contains("v0.8.0 payload"), std::runtime_error);
    CHECK_THROWS_WITH_AS(native::parse_document(doc),
        doctest::Contains("not what its version claims"),
        std::runtime_error);
    // The honest 0.8 declaration of the SAME payload parses whole
    // (the door is the version's, never the payload's existence).
    CHECK_NOTHROW(native::parse_document(
        multi_doc(twelve_entries(0.5, -1.0), "0.8.0")));
}

// ---------------------------------------------------------------------------
// THE VALUE CLASS — the loader's own door: the entries non-empty,
// each edge a non-empty string, each radius finite and > 0.
// ---------------------------------------------------------------------------
TEST_CASE("G5 m5 the value class: the empty map, the empty edge, the "
          "degenerate radius — loud load errors, never defaults") {
    // (a) The empty entries array.
    CHECK_THROWS_WITH_AS(
        native::parse_document(multi_doc("", "0.8.0")),
        doctest::Contains("\"edges\" is empty"), std::runtime_error);
    // (b) The empty edge string.
    {
        const std::string entries =
            "{ \"edge\": \"\", \"radius\": 0.5 }";
        CHECK_THROWS_WITH_AS(
            native::parse_document(multi_doc(entries, "0.8.0")),
            doctest::Contains("\"edge\" is empty"), std::runtime_error);
    }
    // (c) The radius at zero and below (the typed value class, the
    // ADR-0029 family).
    {
        const std::string entries =
            "{ \"edge\": \"box#1:vertical0\", \"radius\": 0.0 }";
        CHECK_THROWS_WITH_AS(
            native::parse_document(multi_doc(entries, "0.8.0")),
            doctest::Contains("must be finite and > 0"),
            std::runtime_error);
    }
    {
        const std::string entries =
            "{ \"edge\": \"box#1:vertical0\", \"radius\": -0.5 }";
        CHECK_THROWS_WITH_AS(
            native::parse_document(multi_doc(entries, "0.8.0")),
            doctest::Contains("must be finite and > 0"),
            std::runtime_error);
    }
    // (d) The non-string edge (the typed field, never a guess).
    {
        const std::string entries =
            "{ \"edge\": 7, \"radius\": 0.5 }";
        CHECK_THROWS_WITH_AS(
            native::parse_document(multi_doc(entries, "0.8.0")),
            doctest::Contains("string for fillet_multi \"edge\""),
            std::runtime_error);
    }
}

// ---------------------------------------------------------------------------
// THE MAP'S UNIQUENESS — the same edge string twice is not a map.
// ---------------------------------------------------------------------------
TEST_CASE("G5 m5 the map's uniqueness: the same edge twice refused "
          "at the parse") {
    const std::string entries =
        "{ \"edge\": \"box#1:vertical0\", \"radius\": 0.5 }, "
        "{ \"edge\": \"box#1:vertical0\", \"radius\": 0.5 }";
    CHECK_THROWS_WITH_AS(
        native::parse_document(multi_doc(entries, "0.8.0")),
        doctest::Contains("twice"), std::runtime_error);
    CHECK_THROWS_WITH_AS(
        native::parse_document(multi_doc(entries, "0.8.0")),
        doctest::Contains("the edge->radius map is a MAP"),
        std::runtime_error);
}

// ---------------------------------------------------------------------------
// THE NEVER-MIX — the representations never mix (ADR-0002): the
// multi node carries the map ALONE.
// ---------------------------------------------------------------------------
TEST_CASE("G5 m5 the never-mix: the multi's field alone, the single's "
          "alone — the honest dispatch both ways") {
    // (a) A fillet_multi node carrying the SINGLE's field.
    {
        std::string doc =
            "{\"format\": \"CAD_0 native\", \"version\": \"0.8.0\","
            " \"scene\": { \"next_id\": 3, \"nodes\": ["
            "  { \"id\": 1, \"name\": \"Cube\","
            "    \"body\": { \"op\": \"box\","
            "               \"extent\": [1.0, 1.0, 1.0] } },"
            "  { \"id\": 2, \"name\": \"Twelve\","
            "    \"kind\": \"fillet_multi\","
            "    \"fillet\": { \"edge\": \"box#1:vertical0\","
            "                 \"radius\": 0.5 },"
            "    \"fillet_multi\": { \"edges\": ["
            "      { \"edge\": \"box#1:vertical0\","
            "        \"radius\": 0.5 } ] } } ] } }";
        CHECK_THROWS_WITH_AS(native::parse_document(doc),
            doctest::Contains("non-multi field \"fillet\""),
            std::runtime_error);
    }
    // (b) A fillet_multi node carrying an SDF-only field.
    {
        const std::string doc =
            "{\"format\": \"CAD_0 native\", \"version\": \"0.8.0\","
            " \"scene\": { \"next_id\": 3, \"nodes\": ["
            "  { \"id\": 1, \"name\": \"Cube\","
            "    \"body\": { \"op\": \"box\","
            "               \"extent\": [1.0, 1.0, 1.0] } },"
            "  { \"id\": 2, \"name\": \"Twelve\","
            "    \"kind\": \"fillet_multi\","
            "    \"body\": { \"op\": \"box\","
            "              \"extent\": [1.0, 1.0, 1.0] },"
            "    \"fillet_multi\": { \"edges\": ["
            "      { \"edge\": \"box#1:vertical0\","
            "        \"radius\": 0.5 } ] } } ] } }";
        CHECK_THROWS_WITH_AS(native::parse_document(doc),
            doctest::Contains("non-multi field \"body\""),
            std::runtime_error);
    }
    // (c) The SINGLE carrying the multi's field (the symmetric door).
    {
        std::string doc =
            "{\"format\": \"CAD_0 native\", \"version\": \"0.8.0\","
            " \"scene\": { \"next_id\": 3, \"nodes\": ["
            "  { \"id\": 1, \"name\": \"Cube\","
            "    \"body\": { \"op\": \"box\","
            "               \"extent\": [1.0, 1.0, 1.0] } },"
            "  { \"id\": 2, \"name\": \"F\", \"kind\": \"fillet\","
            "    \"fillet\": { \"edge\": \"box#1:vertical0\","
            "                 \"radius\": 0.5 },"
            "    \"fillet_multi\": { \"edges\": ["
            "      { \"edge\": \"box#1:vertical0\","
            "        \"radius\": 0.5 } ] } } ] } }";
        CHECK_THROWS_WITH_AS(native::parse_document(doc),
            doctest::Contains("non-fillet field \"fillet_multi\""),
            std::runtime_error);
    }
}

// ---------------------------------------------------------------------------
// THE MISMATCH AT THE DUAL POINTS (D6.6, the law at the write and at
// the re-materialization — NEVER at the kernel): the hand-authored
// twelve with ONE disagreeing vertical.
// ---------------------------------------------------------------------------
TEST_CASE("G5 m5 the mismatch at save/load: two doors one law, the "
          "machine's own words both times, never the kernel's") {
    // The disagreeing map: vertical0 at 0.4, everything else at 0.5.
    const std::string bad = twelve_entries(0.5, 0.4);

    // (a) THE PARSE WALKS WHOLE — the syntactic class is fine; the
    // law is not the parse's to judge (it is read where the body is).
    native::ParsedDocument doc;
    CHECK_NOTHROW(doc = native::parse_document(
                      multi_doc(bad, "0.8.0")));
    REQUIRE(doc.nodes.size() == 2);

    // (b) THE LOAD'S PASS-2: the VISIBLE refusal state — the
    // machine's own words, D6.6's name, the last good state kept
    // (no body ever materialized for a fresh node), the kernel
    // never asked (the prefix the witness).
    const scene::SceneNode* fm = &doc.nodes[1];
    REQUIRE(fm->kind == scene::NodeKind::FilletMulti);
    REQUIRE(fm->fillet_multi.has_value());
    CHECK(fm->fillet_body == nullptr);  // nothing materialized
    REQUIRE_FALSE(fm->fillet_refusal.empty());
    CHECK(fm->fillet_refusal.find(
               "fillet_corner_radii_mismatch") != std::string::npos);
    CHECK(fm->fillet_refusal.rfind("attach_fillet_multi:", 0) == 0);
    CHECK(fm->fillet_refusal.find("fillet_ring_act:") ==
          std::string::npos);
    CHECK(fm->fillet_refusal.find("fillet_edge:") ==
          std::string::npos);

    // (c) THE SAVE DOOR: the same law read at the WRITE — the
    // install is legal (the visible refusal state is a scene state),
    // and the SAVE is REFUSED with the same name: this build never
    // blesses a self-incoherent map with a file.
    scene::Scene sc;
    native::install_document(sc, native::ParsedDocument(doc));
    const std::filesystem::path tmp =
        std::filesystem::temp_directory_path() /
        "cad0_g5m5_mismatch_save.CAD_0";
    std::filesystem::remove(tmp);  // no stale artifact may flake the door
    CHECK_THROWS_WITH_AS(
        native::save_file(tmp, sc, doc.camera),
        doctest::Contains("fillet_corner_radii_mismatch"),
        std::runtime_error);
    CHECK_FALSE(std::filesystem::exists(tmp));

    // (d) THE HONEST TWIN SAVES: the same document at ONE radius
    // round-trips whole (the good map's own control — the door
    // refuses the incoherence, never the payload).
    {
        native::ParsedDocument good = native::parse_document(
            multi_doc(twelve_entries(0.5, -1.0), "0.8.0"));
        scene::Scene sc2;
        native::install_document(
            sc2, native::ParsedDocument(good));
        CHECK_NOTHROW(
            native::save_file(tmp, sc2, good.camera));
        CHECK(std::filesystem::exists(tmp));
        // And the re-loaded good file's node answers 24/48/26.
        const native::ParsedDocument back =
            native::load_file(tmp);
        REQUIRE(back.nodes.size() == 2);
        REQUIRE(back.nodes[1].fillet_body != nullptr);
        CHECK(back.nodes[1].fillet_body->vertex_count() == 24);
        CHECK(back.nodes[1].fillet_body->edge_count() == 48);
        CHECK(back.nodes[1].fillet_body->face_count() == 26);
        std::filesystem::remove(tmp);
    }
}

// ---------------------------------------------------------------------------
// THE PADDING/ABI COUNTERPROOF — the m4 fillet's own subject, one
// map over: the entry carries a std::string, and a naive raw-struct
// image would carry the string's HEAP ADDRESS, not its characters.
// ---------------------------------------------------------------------------
TEST_CASE("G5 m5 the padding counterproof: the naive raw-struct write "
          "dies on the ABI class, the named fields live") {
    // (a) The entry is NOT its fields' semantic size — the string
    // object rides the struct (the pointer, the capacity, the ABI).
    using brep::MultiFilletEntry;
    CHECK(sizeof(MultiFilletEntry) >
          sizeof(double) + 16);  // the characters' own class, at least

    // (b) THE NAIVE IMAGE: two entries with the SAME content (the
    // 41-byte pinned reference — heap-allocated, past the SSO
    // boundary), at DIFFERENT addresses: the raw images differ —
    // the ADDRESS is in the image, not the characters (the m4
    // counterproof's own subject: "a raw image would carry the
    // string's heap address, the ABI, not its characters").
    const std::string pinned =
        "sub#8:isect[box#7:top \xc3\x97 ext#6:lat(0)]@hi";
    REQUIRE(pinned.size() == 41);
    MultiFilletEntry e1{pinned, 0.25};
    MultiFilletEntry e2{pinned, 0.25};
    const std::string naive1 = fnv1a_hex(
        reinterpret_cast<const unsigned char*>(&e1), sizeof(e1));
    const std::string naive2 = fnv1a_hex(
        reinterpret_cast<const unsigned char*>(&e2), sizeof(e2));
    CHECK(naive1 != naive2);

    // (c) THE FIELDS LIVE: two same-content recipes (the entries at
    // different addresses — the allocator's own invariant) ride the
    // LANDED writer to IDENTICAL file bytes.
    const auto scene_with = [](const MultiFilletEntry& e) {
        scene::Scene s;
        (void)s.add_node(
            "Cube", CAD_0::sdf::make_box(
                        math::Vec3f{1.0f, 1.0f, 1.0f}));
        brep::MultiFilletRecipe r;
        r.entries.push_back(e);
        (void)s.add_fillet_multi_node("M", std::move(r));
        return s;
    };
    // A single long-reference entry (a partial-map document, the
    // operand the rim chain's box — the honest in-class case for the
    // ABI subject: the CONTENT is what the file carries).
    const std::filesystem::path pa =
        std::filesystem::temp_directory_path() /
        "cad0_g5m5_pad_a.CAD_0";
    const std::filesystem::path pb =
        std::filesystem::temp_directory_path() /
        "cad0_g5m5_pad_b.CAD_0";
    {
        // Two entries with the same characters, different addresses.
        MultiFilletEntry a{"box#1:vertical2", 0.5};
        MultiFilletEntry b{"box#1:vertical2", 0.5};
        native::save_file(pa, scene_with(a),
                          native::NativeCamera{});
        native::save_file(pb, scene_with(b),
                          native::NativeCamera{});
    }
    CHECK(read_file(pa) == read_file(pb));
    std::filesystem::remove(pa);
    std::filesystem::remove(pb);
}

}  // TEST_SUITE
