// tests/unit/io/test_g6_m5_the_door.cpp
//
// G6 m5 — LA PORTA v0.9.0 (ADR-0045 D7.1/D7.2): the chamfer's own
// doors, self-contained beside the corpus proofs in test_native.cpp
// (the golden's walk, the additivity, the round-trip — those need
// the corpus builders; these need only the door):
//
//   * THE ANACHRONISM: a chamfer/chamfer_multi node in a file
//     declaring < 0.9 — the hand-edited lie, the fillet's own gate
//     form ("no 0.8 writer ever emitted one: the file is not what
//     its version claims").
//   * THE VALUE CLASS at the payload's fields: the entries array
//     non-empty, each edge a non-empty string, each setback finite
//     and > 0 (the single's two named fields, one map over) —
//     corruption of the class is a LOUD load error, never a default.
//   * THE MAP'S UNIQUENESS: the same edge string twice is not a map.
//   * THE NEVER-MIX: the chamfer kinds carry their own payload alone
//     (the single never the multi's field, never the fillet's, never
//     the SDF's; the multi never the single's).
//   * THE SETBACK LAW AT THE DUAL POINTS (ADR-0045 D7.1's own
//     register: the law read at the WRITE and at the
//     re-materialization, two doors one law, NEVER at the kernel): a
//     hand-authored 0.9 file whose twelve entries carry ONE
//     disagreeing setback parses WHOLE (the syntactic class is fine
//     — the law is not the parse's to judge), loads into the
//     VISIBLE refusal state (the machine's own words, D5.4's name,
//     the last good state kept — no body ever materialized), and
//     the SAVE of that scene is REFUSED with the same name — this
//     build never blesses a self-incoherent map with a file. The
//     refusal's provenance is the MACHINE's prefix both times: the
//     kernel is never asked the question the map already answers.
//   * THE STAGE BAND'S PROVENANCE (the kernel's own last line): the
//     twelve at d = 1.0 reaches the KERNEL's strip law — the
//     refusal's provenance is the kernel walk's own prefix, the
//     honest opposite of the law's (the doors' question never asked
//     where the map itself already answers).
//   * THE PADDING/ABI COUNTERPROOF (the m4 fillet's own subject, the
//     chamfer's recipe one feature over): the entry carries a
//     std::string — a naive raw-struct image would carry the
//     string's HEAP ADDRESS (the ABI), not its characters; the
//     named-fields writer stores the CHARACTERS, and two
//     same-content recipes ride it to IDENTICAL file bytes.
//   * THE MIGRATION (the rite's own proof): a v0.8-declaring file
//     WALKS the eighth step whole (the additivity law: no v0.8-era
//     file carries a chamfer node), and the re-save differs by the
//     VERSION FIELD ALONE — pinned here on the door's own document.
#include <doctest/doctest.h>

#include "CAD_0/brep/chamfer.hpp"
#include "CAD_0/brep/mass_properties.hpp"
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
            "    \"kind\": \"chamfer_multi\","
            "    \"chamfer_multi\": { \"edges\": [";
    body += entries_json;
    body += "] } } ] } }";
    return body;
}

// The single-recipe document (the two named fields).
std::string single_doc(const std::string& edge, double setback,
                       const char* version) {
    std::string body =
        "{\"format\": \"CAD_0 native\", \"version\": \"";
    body += version;
    body += "\", \"scene\": { \"next_id\": 3, \"nodes\": ["
            "  { \"id\": 1, \"name\": \"Cube\","
            "    \"body\": { \"op\": \"box\","
            "               \"extent\": [1.0, 1.0, 1.0] } },"
            "  { \"id\": 2, \"name\": \"One\","
            "    \"kind\": \"chamfer\","
            "    \"chamfer\": { \"edge\": \"";
    body += edge;
    body += "\", \"setback\": ";
    body += std::to_string(setback);
    body += " } } ] } }";
    return body;
}

// The twelve entries at one setback (or one disagreeing vertical).
std::string twelve_entries(double d, double odd_setback) {
    const char* toks[12] = {"bottom0", "bottom1", "bottom2", "bottom3",
                            "top0", "top1", "top2", "top3",
                            "vertical0", "vertical1", "vertical2",
                            "vertical3"};
    std::string out;
    for (int k = 0; k < 12; ++k) {
        const double setback = (odd_setback > 0.0 && k == 8)
                                   ? odd_setback
                                   : d;
        out += (k ? ", " : "");
        out += "{ \"edge\": \"box#1:";
        out += toks[k];
        out += "\", \"setback\": ";
        out += std::to_string(setback);
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

TEST_SUITE("io native format G6 m5") {

// ---------------------------------------------------------------------------
// THE ANACHRONISM — the hand-edited lie: no 0.8 writer ever emitted
// a chamfer node (the kind did not exist before the third capability
// bump).
// ---------------------------------------------------------------------------
TEST_CASE("G6 m5 the anachronism: a chamfer node in a file declaring "
          "0.8 is not what its version claims") {
    const std::string doc =
        multi_doc(twelve_entries(0.5, -1.0), "0.8.0");
    CHECK_THROWS_WITH_AS(native::parse_document(doc),
        doctest::Contains("chamfer_multi"), std::runtime_error);
    CHECK_THROWS_WITH_AS(native::parse_document(doc),
        doctest::Contains("v0.9.0 payload"), std::runtime_error);
    CHECK_THROWS_WITH_AS(native::parse_document(doc),
        doctest::Contains("not what its version claims"),
        std::runtime_error);
    // The SINGLE's own gate, the same form.
    CHECK_THROWS_WITH_AS(
        native::parse_document(
            single_doc("box#1:vertical2", 0.5, "0.8.0")),
        doctest::Contains("chamfer"), std::runtime_error);
    CHECK_THROWS_WITH_AS(
        native::parse_document(
            single_doc("box#1:vertical2", 0.5, "0.8.0")),
        doctest::Contains("v0.9.0 payload"), std::runtime_error);
    // The honest 0.9 declaration of the SAME payloads parses whole
    // (the door is the version's, never the payload's existence).
    CHECK_NOTHROW(native::parse_document(
        multi_doc(twelve_entries(0.5, -1.0), "0.9.0")));
    CHECK_NOTHROW(native::parse_document(
        single_doc("box#1:vertical2", 0.5, "0.9.0")));
}

// ---------------------------------------------------------------------------
// THE VALUE CLASS — the loader's own door: the entries non-empty,
// each edge a non-empty string, each setback finite and > 0; the
// single's own two fields the same class.
// ---------------------------------------------------------------------------
TEST_CASE("G6 m5 the value class: the empty map, the empty edge, the "
          "degenerate setback — loud load errors, never defaults") {
    // (a) The empty entries array.
    CHECK_THROWS_WITH_AS(
        native::parse_document(multi_doc("", "0.9.0")),
        doctest::Contains("\"edges\" is empty"), std::runtime_error);
    // (b) The empty edge string (multi and single).
    {
        const std::string entries =
            "{ \"edge\": \"\", \"setback\": 0.5 }";
        CHECK_THROWS_WITH_AS(
            native::parse_document(multi_doc(entries, "0.9.0")),
            doctest::Contains("\"edge\" is empty"),
            std::runtime_error);
    }
    CHECK_THROWS_WITH_AS(
        native::parse_document(single_doc("", 0.5, "0.9.0")),
        doctest::Contains("\"edge\" is empty"), std::runtime_error);
    // (c) The setback at zero and below (the typed value class).
    {
        const std::string entries =
            "{ \"edge\": \"box#1:vertical0\", \"setback\": 0.0 }";
        CHECK_THROWS_WITH_AS(
            native::parse_document(multi_doc(entries, "0.9.0")),
            doctest::Contains("must be finite and > 0"),
            std::runtime_error);
    }
    {
        const std::string entries =
            "{ \"edge\": \"box#1:vertical0\", \"setback\": -0.5 }";
        CHECK_THROWS_WITH_AS(
            native::parse_document(multi_doc(entries, "0.9.0")),
            doctest::Contains("must be finite and > 0"),
            std::runtime_error);
    }
    CHECK_THROWS_WITH_AS(
        native::parse_document(
            single_doc("box#1:vertical2", -0.5, "0.9.0")),
        doctest::Contains("must be finite and > 0"),
        std::runtime_error);
    // (d) The non-string edge and the non-number setback (the typed
    // fields, never a guess).
    {
        const std::string entries =
            "{ \"edge\": 7, \"setback\": 0.5 }";
        CHECK_THROWS_WITH_AS(
            native::parse_document(multi_doc(entries, "0.9.0")),
            doctest::Contains("string for chamfer_multi \"edge\""),
            std::runtime_error);
    }
    {
        const std::string entries =
            "{ \"edge\": \"box#1:vertical0\","
            "  \"setback\": \"big\" }";
        CHECK_THROWS_WITH_AS(
            native::parse_document(multi_doc(entries, "0.9.0")),
            doctest::Contains("number for chamfer_multi \"setback\""),
            std::runtime_error);
    }
}

// ---------------------------------------------------------------------------
// THE MAP'S UNIQUENESS — the same edge string twice is not a map.
// ---------------------------------------------------------------------------
TEST_CASE("G6 m5 the map's uniqueness: the same edge twice refused "
          "at the parse") {
    const std::string entries =
        "{ \"edge\": \"box#1:vertical0\", \"setback\": 0.5 }, "
        "{ \"edge\": \"box#1:vertical0\", \"setback\": 0.5 }";
    CHECK_THROWS_WITH_AS(
        native::parse_document(multi_doc(entries, "0.9.0")),
        doctest::Contains("twice"), std::runtime_error);
    CHECK_THROWS_WITH_AS(
        native::parse_document(multi_doc(entries, "0.9.0")),
        doctest::Contains("the edge->setback map is a MAP"),
        std::runtime_error);
}

// ---------------------------------------------------------------------------
// THE NEVER-MIX — the representations never mix (ADR-0002): the
// chamfer kinds carry their own payload alone.
// ---------------------------------------------------------------------------
TEST_CASE("G6 m5 the never-mix: the single's field alone, the "
          "multi's alone, the fillet's never — the honest dispatch "
          "every way") {
    // (a) A chamfer_multi node carrying the SINGLE's field.
    {
        std::string doc =
            "{\"format\": \"CAD_0 native\", \"version\": \"0.9.0\","
            " \"scene\": { \"next_id\": 3, \"nodes\": ["
            "  { \"id\": 1, \"name\": \"Cube\","
            "    \"body\": { \"op\": \"box\","
            "               \"extent\": [1.0, 1.0, 1.0] } },"
            "  { \"id\": 2, \"name\": \"Twelve\","
            "    \"kind\": \"chamfer_multi\","
            "    \"chamfer\": { \"edge\": \"box#1:vertical0\","
            "                 \"setback\": 0.5 },"
            "    \"chamfer_multi\": { \"edges\": ["
            "      { \"edge\": \"box#1:vertical0\","
            "        \"setback\": 0.5 } ] } } ] } }";
        CHECK_THROWS_WITH_AS(native::parse_document(doc),
            doctest::Contains("non-multi field \"chamfer\""),
            std::runtime_error);
    }
    // (b) A chamfer_multi node carrying an SDF-only field.
    {
        const std::string doc =
            "{\"format\": \"CAD_0 native\", \"version\": \"0.9.0\","
            " \"scene\": { \"next_id\": 3, \"nodes\": ["
            "  { \"id\": 1, \"name\": \"Cube\","
            "    \"body\": { \"op\": \"box\","
            "               \"extent\": [1.0, 1.0, 1.0] } },"
            "  { \"id\": 2, \"name\": \"Twelve\","
            "    \"kind\": \"chamfer_multi\","
            "    \"body\": { \"op\": \"box\","
            "              \"extent\": [1.0, 1.0, 1.0] },"
            "    \"chamfer_multi\": { \"edges\": ["
            "      { \"edge\": \"box#1:vertical0\","
            "        \"setback\": 0.5 } ] } } ] } }";
        CHECK_THROWS_WITH_AS(native::parse_document(doc),
            doctest::Contains("non-multi field \"body\""),
            std::runtime_error);
    }
    // (c) The SINGLE carrying the multi's field (the symmetric door).
    {
        std::string doc =
            "{\"format\": \"CAD_0 native\", \"version\": \"0.9.0\","
            " \"scene\": { \"next_id\": 3, \"nodes\": ["
            "  { \"id\": 1, \"name\": \"Cube\","
            "    \"body\": { \"op\": \"box\","
            "               \"extent\": [1.0, 1.0, 1.0] } },"
            "  { \"id\": 2, \"name\": \"C\", \"kind\": \"chamfer\","
            "    \"chamfer\": { \"edge\": \"box#1:vertical0\","
            "                 \"setback\": 0.5 },"
            "    \"chamfer_multi\": { \"edges\": ["
            "      { \"edge\": \"box#1:vertical0\","
            "        \"setback\": 0.5 } ] } } ] } }";
        CHECK_THROWS_WITH_AS(native::parse_document(doc),
            doctest::Contains("non-chamfer field \"chamfer_multi\""),
            std::runtime_error);
    }
    // (d) The chamfer carrying the FILLET's field (the two replacing
    // features never share a payload).
    {
        std::string doc =
            "{\"format\": \"CAD_0 native\", \"version\": \"0.9.0\","
            " \"scene\": { \"next_id\": 3, \"nodes\": ["
            "  { \"id\": 1, \"name\": \"Cube\","
            "    \"body\": { \"op\": \"box\","
            "               \"extent\": [1.0, 1.0, 1.0] } },"
            "  { \"id\": 2, \"name\": \"C\", \"kind\": \"chamfer\","
            "    \"chamfer\": { \"edge\": \"box#1:vertical0\","
            "                 \"setback\": 0.5 },"
            "    \"fillet\": { \"edge\": \"box#1:vertical1\","
            "                 \"radius\": 0.5 }"
            " } ] } }";
        CHECK_THROWS_WITH_AS(native::parse_document(doc),
            doctest::Contains("non-chamfer field \"fillet\""),
            std::runtime_error);
    }
}

// ---------------------------------------------------------------------------
// THE SETBACK LAW AT THE DUAL POINTS (D5.4, the law at the write and
// at the re-materialization — NEVER at the kernel): the
// hand-authored twelve with ONE disagreeing vertical.
// ---------------------------------------------------------------------------
TEST_CASE("G6 m5 the setback law at save/load: two doors one law, "
          "the machine's own words both times, never the kernel's") {
    // The disagreeing map: vertical0 at 0.4, everything else at 0.5.
    const std::string bad = twelve_entries(0.5, 0.4);

    // (a) THE PARSE WALKS WHOLE — the syntactic class is fine; the
    // law is not the parse's to judge (it is read where the body is).
    native::ParsedDocument doc;
    CHECK_NOTHROW(doc = native::parse_document(
                      multi_doc(bad, "0.9.0")));
    REQUIRE(doc.nodes.size() == 2);

    // (b) THE LOAD'S PASS-2: the VISIBLE refusal state — the
    // machine's own words, D5.4's name, the last good state kept
    // (no body ever materialized for a fresh node), the kernel
    // never asked (the prefix the witness).
    const scene::SceneNode* cm = &doc.nodes[1];
    REQUIRE(cm->kind == scene::NodeKind::ChamferMulti);
    REQUIRE(cm->chamfer_multi.has_value());
    CHECK(cm->chamfer_body == nullptr);  // nothing materialized
    REQUIRE_FALSE(cm->chamfer_refusal.empty());
    CHECK(cm->chamfer_refusal.find(
              "chamfer_corner_setback_mismatch") != std::string::npos);
    CHECK(cm->chamfer_refusal.rfind("attach_chamfer_multi:", 0) == 0);
    CHECK(cm->chamfer_refusal.find("chamfer_ring_act:") ==
          std::string::npos);
    CHECK(cm->chamfer_refusal.find("chamfer_edge:") ==
          std::string::npos);
    // Both values in the payload (D7.1's own demand).
    CHECK(cm->chamfer_refusal.find("0.400000") != std::string::npos);
    CHECK(cm->chamfer_refusal.find("0.500000") != std::string::npos);

    // (c) THE SAVE DOOR: the same law read at the WRITE — the
    // install is legal (the visible refusal state is a scene state),
    // and the SAVE is REFUSED with the same name: this build never
    // blesses a self-incoherent map with a file.
    scene::Scene sc;
    native::install_document(sc, native::ParsedDocument(doc));
    const std::filesystem::path tmp =
        std::filesystem::temp_directory_path() /
        "cad0_g6m5_mismatch_save.CAD_0";
    std::filesystem::remove(tmp);  // no stale artifact may flake the door
    CHECK_THROWS_WITH_AS(
        native::save_file(tmp, sc, doc.camera),
        doctest::Contains("chamfer_corner_setback_mismatch"),
        std::runtime_error);
    CHECK_FALSE(std::filesystem::exists(tmp));

    // (d) THE HONEST TWIN SAVES: the same document at ONE setback
    // round-trips whole (the good map's own control — the door
    // refuses the incoherence, never the payload).
    {
        native::ParsedDocument good = native::parse_document(
            multi_doc(twelve_entries(0.5, -1.0), "0.9.0"));
        scene::Scene sc2;
        native::install_document(
            sc2, native::ParsedDocument(good));
        CHECK_NOTHROW(
            native::save_file(tmp, sc2, good.camera));
        CHECK(std::filesystem::exists(tmp));
        // And the re-loaded good file's node answers 32/48/18 with
        // the closed-form volume at 1e-12 (the recipe the truth, the
        // body the cache).
        const native::ParsedDocument back =
            native::load_file(tmp);
        REQUIRE(back.nodes.size() == 2);
        REQUIRE(back.nodes[1].chamfer_body != nullptr);
        CHECK(back.nodes[1].chamfer_body->vertex_count() == 32);
        CHECK(back.nodes[1].chamfer_body->edge_count() == 48);
        CHECK(back.nodes[1].chamfer_body->face_count() == 18);
        const brep::MassProperties mp = brep::mass_properties(
            *back.nodes[1].chamfer_body,
            math::Tolerance::linear(1e-9));
        CHECK(mp.volume ==
              doctest::Approx(8.0 - 12.0 * 0.25 + 6.0 * 0.125)
                  .epsilon(1e-12));
        std::filesystem::remove(tmp);
    }
}

// ---------------------------------------------------------------------------
// THE STAGE BAND'S PROVENANCE — the kernel's own last line: the
// twelve at d = 1.0 is a question the map does NOT answer (the
// values agree); the kernel's own admission refuses, and the
// refusal's provenance is the KERNEL WALK's prefix — the honest
// opposite of the law's (the two questions never confused).
// ---------------------------------------------------------------------------
TEST_CASE("G6 m5 the stage band's provenance: the kernel asked, its "
          "own words — the law's opposite, never confused") {
    // The twelve at d = 1.0: all values agree, the law holds; the
    // kernel's chain admission (the strip law at the second walk)
    // refuses with its own name.
    native::ParsedDocument doc;
    CHECK_NOTHROW(doc = native::parse_document(
                      multi_doc(twelve_entries(1.0, -1.0),
                                "0.9.0")));
    REQUIRE(doc.nodes.size() == 2);
    const scene::SceneNode* cm = &doc.nodes[1];
    REQUIRE(cm->kind == scene::NodeKind::ChamferMulti);
    CHECK(cm->chamfer_body == nullptr);
    REQUIRE_FALSE(cm->chamfer_refusal.empty());
    // The provenance: the kernel's own walk (chamfer_edge:), NOT the
    // machine's prefix, NOT the law's name — the kernel asked, its
    // words the last line.
    CHECK(cm->chamfer_refusal.find("chamfer_edge:") !=
          std::string::npos);
    CHECK(cm->chamfer_refusal.find(
              "chamfer_setback_exceeds_face") != std::string::npos);
    CHECK(cm->chamfer_refusal.find(
              "chamfer_corner_setback_mismatch") == std::string::npos);
}

// ---------------------------------------------------------------------------
// THE MIGRATION (the rite's own proof, on the door's own document):
// a v0.8-declaring file WALKS the eighth step whole — the additivity
// law: no v0.8-era file carries a chamfer node, nothing is left
// behind; the re-save differs by the VERSION FIELD ALONE.
// ---------------------------------------------------------------------------
TEST_CASE("G6 m5 the migration: a 0.8-declaring file walks the "
          "eighth step, the re-save differing by the version field "
          "alone") {
    // A v0.8-era document: the fitted box alone (no chamfer node —
    // the era could not carry one).
    const std::string v8_doc =
        "{\"format\": \"CAD_0 native\", \"version\": \"0.8.0\","
        " \"scene\": { \"next_id\": 2, \"nodes\": ["
        "  { \"id\": 1, \"name\": \"Cube\","
        "    \"body\": { \"op\": \"box\","
        "               \"extent\": [1.0, 1.0, 1.0] } }"
        " ] }, \"camera\": { \"target\": [0, 0, 0],"
        " \"distance\": 5.0, \"yaw\": 0.0, \"pitch\": 0.3,"
        " \"fov_deg\": 45.0 } }";
    native::ParsedDocument doc;
    CHECK_NOTHROW(doc = native::parse_document(v8_doc));
    CHECK(doc.version == native::NativeVersion{0, 8, 0});
    REQUIRE(doc.nodes.size() == 1);

    // The re-save writes the CURRENT number; patched back to the
    // file's own declaration the bytes are the document WHOLE (the
    // no-op's own proof: nothing else moved) — the patched file is
    // written out and LOADED through the container door (the save
    // writes the ADR-0007a binary container, so the patched bytes
    // go back to a file and load_file answers, never a text parse).
    scene::Scene sc;
    native::install_document(sc, native::ParsedDocument(doc));
    const std::filesystem::path tmp =
        std::filesystem::temp_directory_path() /
        "cad0_g6m5_walk_eighth.CAD_0";
    native::save_file(tmp, sc, doc.camera);
    const std::string fresh = read_file(tmp);
    CHECK(fresh.find("\"version\": \"0.9.0\"") != std::string::npos);
    {
        std::string as_v8 = fresh;
        const std::string v9 = "\"version\": \"0.9.0\"";
        const std::string v8 = "\"version\": \"0.8.0\"";
        const auto pos = as_v8.find(v9);
        REQUIRE(pos != std::string::npos);
        as_v8.replace(pos, v9.size(), v8);
        // The patched file LOADS whole (the eighth step walked, the
        // fitted box back), and its re-save is byte-identical to
        // the first (the no-op moved nothing but the number).
        const std::filesystem::path tmp_patched =
            std::filesystem::temp_directory_path() /
            "cad0_g6m5_walk_eighth_patched.CAD_0";
        {
            std::ofstream out(tmp_patched, std::ios::binary |
                                               std::ios::trunc);
            out.write(as_v8.data(),
                      static_cast<std::streamsize>(as_v8.size()));
        }
        const native::ParsedDocument again =
            native::load_file(tmp_patched);
        REQUIRE(again.nodes.size() == 1);
        REQUIRE(again.nodes[0].chamfer_body == nullptr);
        CHECK(again.nodes[0].kind == scene::NodeKind::Sdf);
        scene::Scene sc2;
        native::install_document(
            sc2, native::ParsedDocument(again));
        const std::filesystem::path tmp2 =
            std::filesystem::temp_directory_path() /
            "cad0_g6m5_walk_eighth2.CAD_0";
        native::save_file(tmp2, sc2, again.camera);
        CHECK(read_file(tmp2) == fresh);
        std::filesystem::remove(tmp_patched);
        std::filesystem::remove(tmp2);
    }
    // The eighth step is REGISTERED (the chain's own hook).
    CHECK(native::migrator_count() == 8);
    std::filesystem::remove(tmp);
}

// ---------------------------------------------------------------------------
// THE PADDING/ABI COUNTERPROOF — the m4 fillet's own subject, the
// chamfer's recipe one feature over: the entry carries a
// std::string, and a naive raw-struct image would carry the
// string's HEAP ADDRESS, not its characters.
// ---------------------------------------------------------------------------
TEST_CASE("G6 m5 the padding counterproof: the naive raw-struct write "
          "dies on the ABI class, the named fields live") {
    // (a) The entry is NOT its fields' semantic size — the string
    // object rides the struct (the pointer, the capacity, the ABI).
    using brep::MultiChamferEntry;
    CHECK(sizeof(MultiChamferEntry) >
          sizeof(double) + 16);  // the characters' own class, at least

    // (b) THE NAIVE IMAGE: two entries with the SAME content (the
    // 41-byte pinned reference — heap-allocated, past the SSO
    // boundary), at DIFFERENT addresses: the raw images differ —
    // the ADDRESS is in the image, not the characters.
    const std::string pinned =
        "sub#8:isect[box#7:top \xc3\x97 ext#6:lat(0)]@hi";
    REQUIRE(pinned.size() == 41);
    MultiChamferEntry e1{pinned, 0.25};
    MultiChamferEntry e2{pinned, 0.25};
    const std::string naive1 = fnv1a_hex(
        reinterpret_cast<const unsigned char*>(&e1), sizeof(e1));
    const std::string naive2 = fnv1a_hex(
        reinterpret_cast<const unsigned char*>(&e2), sizeof(e2));
    CHECK(naive1 != naive2);

    // (c) THE FIELDS LIVE: two same-content recipes (the entries at
    // different addresses — the allocator's own invariant) ride the
    // LANDED writer to IDENTICAL file bytes.
    const auto scene_with = [](const MultiChamferEntry& e) {
        scene::Scene s;
        (void)s.add_node(
            "Cube", CAD_0::sdf::make_box(
                        math::Vec3f{1.0f, 1.0f, 1.0f}));
        brep::MultiChamferRecipe r;
        r.entries.push_back(e);
        (void)s.add_chamfer_multi_node("M", std::move(r));
        return s;
    };
    const std::filesystem::path pa =
        std::filesystem::temp_directory_path() /
        "cad0_g6m5_pad_a.CAD_0";
    const std::filesystem::path pb =
        std::filesystem::temp_directory_path() /
        "cad0_g6m5_pad_b.CAD_0";
    {
        // Two entries with the same characters, different addresses.
        MultiChamferEntry a{"box#1:vertical2", 0.5};
        MultiChamferEntry b{"box#1:vertical2", 0.5};
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
