// tests/unit/brep/test_h2_u6_m4_subtract_names.cpp
//
// H2 unit 6, morso 4 — IL SUBTRACT ESATTO (ADR-0036 D6.3, the accepted
// act + the morse-4 register): the full descent table, the relational
// names, the report column. The first morse whose rows CONSUME the
// previous morsi's born names — the operands of 'sub#4:isect[...]'
// are 'box#1:top' and 'ext#3:lat(0)', morsi 2's and 3's own products —
// and the first tavola with BOTH closure sides: the old generation's
// entities (the target's AND the tool's) each in exactly one row, the
// vanished tail beside the result rows. Every claim its own test, the
// naive version of each one failing where the naming is new:
//
//   1. THE EXACT NAMES — the guide case's D1 table, the EIGHT rows,
//      character for character against the ADR's own strings: the
//      pierced pair (absorbed, NOT renamed), the four sides (kept),
//      the wall (absorbed, its BIRTH name kept), the two rim circles
//      (born — ONE logical name each, the seam-wired arc pair the
//      MATERIALIZED form, ADR-0023: both halves answer to it), the
//      seam vertical (born), the tool's caps (vanished, their own
//      birth names on the dead rows).
//   2. THE TAVOLA PIENA — the guide case's 47 rows (D2's own count:
//      36 out + 11 vanished), pinned ROW BY ROW: entity, relation,
//      the operand ref (which body, kind, address in that body's own
//      numbering), the name, the CAUSE on every vanished row; MERGED
//      zero instances in the exact path — a VERIFIED FACT, not a gap
//      (the flush case re-verified: the coplanar cap is a 1:1 + 1:0,
//      "nothing is emitted for them, which IS the dedup" — the
//      assembly's own comment, subtract.hpp's, cited in the tavola's
//      own derivation comment).
//   3. THE CLOSURE, BOTH SIDES, AND THE COUNTERPROOFS — the result
//      side green, the old side green; then the INJECTED DEFECTS, each
//      one caught and named: a lost result row, a lost vanished row
//      (the old side's own), a double-booking, a dangling operand
//      ref, a born row that books operands, a vanished row that
//      disagrees with itself. The machine takes them all — the
//      closure is a check, never a hope.
//   4. THE ROUND-TRIP ABI — every relational name, idem and back: the
//      stored structure renders the stored name (twice, identically);
//      the frozen bijection where the token is unique, the LEVELLED
//      entry's own row where the pair shares a token (the level is
//      the entry's, the path spends no component on it); the operands'
//      OWN stamps rebuild the genealogy legs. THE DEPTH DERIVED: the
//      rim resolves at 4 (sub -> isect -> box/ext -> the entity), the
//      seam at 3, declared against kMaxDepth = 8 — and the morse-3
//      register's item 4 (the multi-chain operand view) LANDS here,
//      pinned at the naming layer: the machinery renders
//      'c0:lat(0)' inside the brackets the day a multi-chain operand
//      appears (the class checks refuse the ring tools today, so no
//      in-class body produces one — the register said "priced by its
//      consumer", and this is the consumer's act).
//   5. THE PURITY, THE WHOLE CHAIN — the chain materialized per intero
//      (box#1 -> sk#2 -> ext#3 -> sub#4, the ADR's own FeatureIds):
//      the name digest pins the D1 names CHARACTER PER CHARACTER on
//      every body of the chain — 'box#1:top' the pierced face
//      (absorbed, NOT renamed), 'ext#3:lat(0)' the wall (absorbed, the
//      birth name kept), 'sub#4:isect[box#1:top × ext#3:lat(0)]@hi'
//      the upper rim — LA STRINGA DEL FILLET, viva nel digest; N=10
//      rebuilds one answer, bit-identical from the FIRST pinning; the
//      feature part flows (sub#9 is another body, same addresses).
//   6. THE ADDRESS PLAN — unit-4's own pins re-asserted VERBATIM on
//      the same recipe (12/17/7/2, chi = 0, the wall ONE cylinder
//      face, the pierced pair's inner loops, the XOR): the names
//      arrived BESIDE the addresses, the numbering never moved, and
//      the full existing suite running without a re-pin is the global
//      proof this family mirrors (m2's boundary pin moved as it was
//      pre-registered to move — the wall's inheritance, its own
//      commit's act).
//   7. THE BINDINGS + THE REPORT — the Python probe's own file (the
//      m2/m3 precedent): face_query/edge_query carry the names on the
//      whole guide case, the report's table prints the column, the
//      edge leg included (the rim's fillet name — the maintainer's
//      click-test).
//   8. THE LAYER-2 BILL, FIELD BY FIELD — kept carries the operand's
//      own records untouched; the pierced absorbed carry the
//      operand's record with the LOOP change (the trim's mark); the
//      wall absorbed carries the OPERATION's record (combined, at the
//      creation site). The D2 bill verified, not promised.
//
// THE ORACLES (D5's declaration, honored to the letter): the name
// digest — the morse-2 oracle, "<address>|<name>\n" per face in
// address order, FNV-1a 64 — now runs on the WHOLE CHAIN's bodies
// (the box, the tool, the result), and the EDGE leg joins the result
// (the rim circles' and the seam's pins, the honest empties' pins).
// Bit-identical from the FIRST pinning; no existing golden moved (the
// suite is the proof, and this file re-asserts unit-4's values
// verbatim, not new ones). Determinism is ADR-0014's trivial case:
// the names are strings and integers, zero floating-point content —
// the same constants hold under gcc and under clang (the CI's two
// compilers), which is the cross-compiler claim's own proof.
//
// THE HONEST BOUNDARIES, pinned as the design they are: the unnamed
// mode (no naming context) leaves the born edges empty — the
// inheritance (the wall's, the kept faces') flows regardless, it is
// a stamp copy, no table involved; the general chain's born edges
// (the joints' verticals, the segment walls' curves) read honestly
// empty — Phase D's SSI entry is their writer, the extension a table
// row away; vertex names are v1's queue (D4) — empty, never a guess.
#include <doctest/doctest.h>

#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/naming.hpp"
#include "CAD_0/brep/descent.hpp"
#include "CAD_0/brep/query.hpp"
#include "CAD_0/brep/topology.hpp"
#include "CAD_0/brep/extrude.hpp"
#include "CAD_0/brep/subtract.hpp"
#include "CAD_0/sdf/sketch_profile.hpp"
#include "CAD_0/geometry/shape_id.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace {

namespace b = CAD_0::brep;
namespace n = CAD_0::brep::naming;
namespace s = CAD_0::sdf;
using CAD_0::geometry::FeatureId;
using CAD_0::geometry::ShapeId;
using CAD_0::geometry::SubshapeRef;
using Vec2f = CAD_0::math::Vec2f;

// ---------------------------------------------------------------------------
// The name-digest oracles (the morse-2/3 shape, replicated for the
// whole chain): pure integer arithmetic, FNV-1a 64 — ADR-0014 honored
// by construction.
// ---------------------------------------------------------------------------

std::uint64_t fnv1a64(const std::string& str) {
    std::uint64_t h = 0xcbf29ce484222325ull;
    for (const char c : str) {
        h ^= static_cast<std::uint64_t>(static_cast<unsigned char>(c));
        h *= 0x100000001b3ull;
    }
    return h;
}

std::string name_digest(const b::BRepBody& body) {
    std::string out;
    for (b::FaceId i = 0; i < body.face_count(); ++i) {
        out += std::to_string(i);
        out += '|';
        out += b::query_face(body, i).name;
        out += '\n';
    }
    return out;
}

std::string edge_name_digest(const b::BRepBody& body) {
    std::string out;
    for (b::EdgeId i = 0; i < body.edge_count(); ++i) {
        out += std::to_string(i);
        out += '|';
        out += b::query_edge(body, i).name;
        out += '\n';
    }
    return out;
}

// ---------------------------------------------------------------------------
// The recipes (unit 4's own helpers, verbatim — the same profiles the
// G1 pins ride on) and the D1 chain's construction.
// ---------------------------------------------------------------------------

// The guide case's tool recipe: the circle at the origin, r = 0.25,
// strictly interior to the [-1,1]^3 box (the ADR-0032 D7.5 body).
std::vector<s::SketchChain> circle_chains() {
    return {s::SketchChain{
        {s::sketch_circle(Vec2f{0.0f, 0.0f}, 0.25f, false)}}};
}

// The D1 chain (the ADR's own FeatureIds): box#1 the target, sk#2 the
// sketch (the authored roots — the extrude's tavola FROMs name them),
// ext#3 the tool extrude, sub#4 the subtract. `named` selects the
// naming context: WITH it the born edges carry the relational names;
// WITHOUT it (the unnamed mode) they read honestly empty — the wall's
// inheritance flows in both modes (a stamp copy, no table involved).
struct GuideCase {
    std::unique_ptr<b::BRepBody> box;    // box#1
    std::vector<s::SketchChain> chains;  // the sk#2 recipe
    std::unique_ptr<b::BRepBody> tool;   // ext#3
    std::unique_ptr<b::BRepBody> result; // sub#4's result
};

GuideCase guide_case(bool named = true) {
    GuideCase g;
    g.box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
    g.chains = circle_chains();
    g.tool = b::make_prism(g.chains, 3.0f, +1, FeatureId{3});
    if (named) {
        const b::SubtractNaming ctx{
            FeatureId{4}, &n::kBoxNaming, &n::kExtNaming};
        g.result = b::exact_subtract(
            *g.box, *g.tool, {0.0, 0.0, -1.5}, b::kSubtractTolerance, ctx);
    } else {
        g.result = b::exact_subtract(
            *g.box, *g.tool, {0.0, 0.0, -1.5}, b::kSubtractTolerance);
    }
    return g;
}

// The instance's operand registry, the SAME one the stamping site
// built (the feature ids flow from the operands' own stamps, the
// tables from the naming context) — the round-trip's render view.
struct SubRegistry {
    n::OperandBinding operands[2]{};
    n::OperationNaming naming{};
    SubRegistry(const GuideCase& g) {
        const auto& box_top =
            g.box->solid.shell().faces()[1];   // box#1:top's face
        const auto& wall =
            g.tool->solid.shell().faces()[0];  // ext#3:lat(0)'s face
        operands[0] = {box_top.shape_id().feature, &n::kBoxNaming};
        operands[1] = {wall.shape_id().feature, &n::kExtNaming};
        naming = n::OperationNaming{
            n::FeatureKind::Subtract, {}, n::kSubEdgeRoles, operands};
    }
};

} // namespace

// ===========================================================================
// FAMILY 1 — THE EXACT NAMES: the D1 table's eight rows, character for
// character
// ===========================================================================

TEST_CASE("H2-U6-M4 exact names: the guide case's D1 table — the "
          "eight rows, character for character") {
    const auto g = guide_case(true);
    REQUIRE(g.result->face_count() == 7);   // the pinned 12/17/7/2
    REQUIRE(g.result->edge_count() == 17);
    REQUIRE(g.result->vertex_count() == 12);

    // THE ABSORBED, NOT RENAMED (D1: identity is where a thing was
    // born): the pierced pair keeps the BOX's names — the piercing is
    // recorded in the tavola, never minted into a new identity.
    CHECK(b::query_face(*g.result, 0).name == "box#1:bottom");
    CHECK(b::query_face(*g.result, 1).name == "box#1:top");
    // THE KEPT: the four sides, the BOX's own.
    CHECK(b::query_face(*g.result, 2).name == "box#1:front");
    CHECK(b::query_face(*g.result, 3).name == "box#1:back");
    CHECK(b::query_face(*g.result, 4).name == "box#1:left");
    CHECK(b::query_face(*g.result, 5).name == "box#1:right");
    // THE ABSORBED WALL: the tool's lateral re-framed to the window —
    // its BIRTH name kept (the extrude's face, not the subtract's).
    CHECK(b::query_face(*g.result, 6).name == "ext#3:lat(0)");

    // THE BORN, the relational grammar (the SSI entry's own form): the
    // rim circles — ONE logical name each, the seam-wired arc pair its
    // MATERIALIZED form (ADR-0023: BOTH halves answer to the one name;
    // the pair is the name's materialization, not two names), the
    // bottom pair at @lo, the top pair at @hi; the seam vertical.
    CHECK(b::query_edge(*g.result, 12).name ==
          "sub#4:isect[box#1:bottom × ext#3:lat(0)]@lo");
    CHECK(b::query_edge(*g.result, 13).name ==
          "sub#4:isect[box#1:bottom × ext#3:lat(0)]@lo");
    CHECK(b::query_edge(*g.result, 14).name ==
          "sub#4:isect[box#1:top × ext#3:lat(0)]@hi");
    CHECK(b::query_edge(*g.result, 15).name ==
          "sub#4:isect[box#1:top × ext#3:lat(0)]@hi");
    CHECK(b::query_edge(*g.result, 16).name ==
          "sub#4:seam[ext#3:lat(0)]");

    // The operands PER NAME, in the DECLARED order, never reordered:
    // the target's pierced face first, the tool's wall second — the
    // SSI table's own operand order, the grammar's own rule.
    const std::string rim_hi = b::query_edge(*g.result, 14).name;
    CHECK(rim_hi.find("box#1:top") < rim_hi.find("ext#3:lat(0)"));

    // A's edges keep their names (the verbatim copy — m2's own pin,
    // re-asserted on the D1 chain): the twelve box edges.
    for (b::EdgeId i = 0; i < 12; ++i)
        CHECK(b::query_edge(*g.result, i).name ==
              b::query_edge(*g.box, i).name);

    // THE HONEST EMPTIES, pinned as the design they are: vertex
    // names are v1's queue (D4 — no consumer today): the Vertex type
    // carries no name field AT ALL, which is the honest boundary's
    // own shape. The rim circles' halves and the seam are the ONLY
    // born edges named — every one of them asserted non-empty below.
    for (b::EdgeId i = 12; i < 17; ++i)
        REQUIRE_FALSE(b::query_edge(*g.result, i).name.empty());
}

TEST_CASE("H2-U6-M4 the honest boundary: the unnamed mode leaves the "
          "born edges empty; the inheritance flows regardless") {
    const auto g = guide_case(false);
    // No naming context: the born edges (12..16) read honestly EMPTY —
    // their vocabulary is the context's (the D4 queue's discipline,
    // never a guess).
    for (b::EdgeId i = 12; i < 17; ++i)
        CHECK(b::query_edge(*g.result, i).name.empty());
    // The wall's inheritance is a stamp COPY of the operand's own — no
    // table involved, so it flows in the unnamed mode too (m2's
    // boundary pin, landed at morso 4 and re-asserted here at the D1
    // chain's own ids).
    CHECK(b::query_face(*g.result, 6).name == "ext#3:lat(0)");
    CHECK(b::query_face(*g.result, 1).name == "box#1:top");
    // The addresses are IDENTICAL in both modes — the name never moves
    // the address (D1's split, additive by construction).
    const auto named = guide_case(true);
    REQUIRE(named.result->edge_count() == g.result->edge_count());
    REQUIRE(named.result->face_count() == g.result->face_count());
    for (b::EdgeId i = 0; i < 17; ++i) {
        REQUIRE(b::query_edge(*named.result, i).v0 ==
                b::query_edge(*g.result, i).v0);
        REQUIRE(b::query_edge(*named.result, i).v1 ==
                b::query_edge(*g.result, i).v1);
    }
}

// ===========================================================================
// FAMILY 2 — THE TAVOLA PIENA: the 47 rows, row by row
// ===========================================================================

namespace {

// One expected row of the guide case's tavola — the golden's own
// shape: relation, entity, address (in the subject's own body), the
// operand ref (which body, kind, address in THAT body), the name, the
// vanished cause.
struct ExpectedRow {
    b::Descent relation{};
    b::EntityKind entity{};
    std::uint64_t address{};
    bool has_operand{};
    std::uint8_t operand{};
    b::EntityKind operand_kind{};
    std::uint64_t operand_address{};
    const char* name{};
    const char* cause{};
};

const std::vector<ExpectedRow>& guide_case_expected() {
    static const std::vector<ExpectedRow> expected = {
        // ---- the result's vertices: 8 kept + 4 born
        {b::Descent::Kept, b::EntityKind::Vertex, 0,
         true, 0, b::EntityKind::Vertex, 0, "", nullptr},
        {b::Descent::Kept, b::EntityKind::Vertex, 1,
         true, 0, b::EntityKind::Vertex, 1, "", nullptr},
        {b::Descent::Kept, b::EntityKind::Vertex, 2,
         true, 0, b::EntityKind::Vertex, 2, "", nullptr},
        {b::Descent::Kept, b::EntityKind::Vertex, 3,
         true, 0, b::EntityKind::Vertex, 3, "", nullptr},
        {b::Descent::Kept, b::EntityKind::Vertex, 4,
         true, 0, b::EntityKind::Vertex, 4, "", nullptr},
        {b::Descent::Kept, b::EntityKind::Vertex, 5,
         true, 0, b::EntityKind::Vertex, 5, "", nullptr},
        {b::Descent::Kept, b::EntityKind::Vertex, 6,
         true, 0, b::EntityKind::Vertex, 6, "", nullptr},
        {b::Descent::Kept, b::EntityKind::Vertex, 7,
         true, 0, b::EntityKind::Vertex, 7, "", nullptr},
        {b::Descent::Born, b::EntityKind::Vertex, 8,
         false, 0, b::EntityKind::Vertex, 0, "", nullptr},
        {b::Descent::Born, b::EntityKind::Vertex, 9,
         false, 0, b::EntityKind::Vertex, 0, "", nullptr},
        {b::Descent::Born, b::EntityKind::Vertex, 10,
         false, 0, b::EntityKind::Vertex, 0, "", nullptr},
        {b::Descent::Born, b::EntityKind::Vertex, 11,
         false, 0, b::EntityKind::Vertex, 0, "", nullptr},
        // ---- the result's edges: 12 kept + 5 born
        {b::Descent::Kept, b::EntityKind::Edge, 0,
         true, 0, b::EntityKind::Edge, 0, "box#1:bottom0", nullptr},
        {b::Descent::Kept, b::EntityKind::Edge, 1,
         true, 0, b::EntityKind::Edge, 1, "box#1:bottom1", nullptr},
        {b::Descent::Kept, b::EntityKind::Edge, 2,
         true, 0, b::EntityKind::Edge, 2, "box#1:bottom2", nullptr},
        {b::Descent::Kept, b::EntityKind::Edge, 3,
         true, 0, b::EntityKind::Edge, 3, "box#1:bottom3", nullptr},
        {b::Descent::Kept, b::EntityKind::Edge, 4,
         true, 0, b::EntityKind::Edge, 4, "box#1:top0", nullptr},
        {b::Descent::Kept, b::EntityKind::Edge, 5,
         true, 0, b::EntityKind::Edge, 5, "box#1:top1", nullptr},
        {b::Descent::Kept, b::EntityKind::Edge, 6,
         true, 0, b::EntityKind::Edge, 6, "box#1:top2", nullptr},
        {b::Descent::Kept, b::EntityKind::Edge, 7,
         true, 0, b::EntityKind::Edge, 7, "box#1:top3", nullptr},
        {b::Descent::Kept, b::EntityKind::Edge, 8,
         true, 0, b::EntityKind::Edge, 8, "box#1:vertical0", nullptr},
        {b::Descent::Kept, b::EntityKind::Edge, 9,
         true, 0, b::EntityKind::Edge, 9, "box#1:vertical1", nullptr},
        {b::Descent::Kept, b::EntityKind::Edge, 10,
         true, 0, b::EntityKind::Edge, 10, "box#1:vertical2", nullptr},
        {b::Descent::Kept, b::EntityKind::Edge, 11,
         true, 0, b::EntityKind::Edge, 11, "box#1:vertical3", nullptr},
        {b::Descent::Born, b::EntityKind::Edge, 12,
         false, 0, b::EntityKind::Edge, 0,
         "sub#4:isect[box#1:bottom × ext#3:lat(0)]@lo", nullptr},
        {b::Descent::Born, b::EntityKind::Edge, 13,
         false, 0, b::EntityKind::Edge, 0,
         "sub#4:isect[box#1:bottom × ext#3:lat(0)]@lo", nullptr},
        {b::Descent::Born, b::EntityKind::Edge, 14,
         false, 0, b::EntityKind::Edge, 0,
         "sub#4:isect[box#1:top × ext#3:lat(0)]@hi", nullptr},
        {b::Descent::Born, b::EntityKind::Edge, 15,
         false, 0, b::EntityKind::Edge, 0,
         "sub#4:isect[box#1:top × ext#3:lat(0)]@hi", nullptr},
        {b::Descent::Born, b::EntityKind::Edge, 16,
         false, 0, b::EntityKind::Edge, 0,
         "sub#4:seam[ext#3:lat(0)]", nullptr},
        // ---- the result's faces: 4 kept + 3 absorbed
        {b::Descent::Absorbed, b::EntityKind::Face, 0,
         true, 0, b::EntityKind::Face, 0, "box#1:bottom", nullptr},
        {b::Descent::Absorbed, b::EntityKind::Face, 1,
         true, 0, b::EntityKind::Face, 1, "box#1:top", nullptr},
        {b::Descent::Kept, b::EntityKind::Face, 2,
         true, 0, b::EntityKind::Face, 2, "box#1:front", nullptr},
        {b::Descent::Kept, b::EntityKind::Face, 3,
         true, 0, b::EntityKind::Face, 3, "box#1:back", nullptr},
        {b::Descent::Kept, b::EntityKind::Face, 4,
         true, 0, b::EntityKind::Face, 4, "box#1:left", nullptr},
        {b::Descent::Kept, b::EntityKind::Face, 5,
         true, 0, b::EntityKind::Face, 5, "box#1:right", nullptr},
        {b::Descent::Absorbed, b::EntityKind::Face, 6,
         true, 1, b::EntityKind::Face, 0, "ext#3:lat(0)", nullptr},
        // ---- the vanished tail, the dying operand's own order
        {b::Descent::Vanished, b::EntityKind::Vertex, 0,
         true, 1, b::EntityKind::Vertex, 0, "",
         "re-authored with the window"},
        {b::Descent::Vanished, b::EntityKind::Vertex, 1,
         true, 1, b::EntityKind::Vertex, 1, "",
         "re-authored with the window"},
        {b::Descent::Vanished, b::EntityKind::Vertex, 2,
         true, 1, b::EntityKind::Vertex, 2, "",
         "re-authored with the window"},
        {b::Descent::Vanished, b::EntityKind::Vertex, 3,
         true, 1, b::EntityKind::Vertex, 3, "",
         "re-authored with the window"},
        {b::Descent::Vanished, b::EntityKind::Edge, 0,
         true, 1, b::EntityKind::Edge, 0, "",
         "re-authored with the window"},
        {b::Descent::Vanished, b::EntityKind::Edge, 1,
         true, 1, b::EntityKind::Edge, 1, "",
         "re-authored with the window"},
        {b::Descent::Vanished, b::EntityKind::Edge, 2,
         true, 1, b::EntityKind::Edge, 2, "",
         "re-authored with the window"},
        {b::Descent::Vanished, b::EntityKind::Edge, 3,
         true, 1, b::EntityKind::Edge, 3, "",
         "re-authored with the window"},
        {b::Descent::Vanished, b::EntityKind::Edge, 4,
         true, 1, b::EntityKind::Edge, 4, "ext#3:seam",
         "re-authored with the window"},
        {b::Descent::Vanished, b::EntityKind::Face, 1,
         true, 1, b::EntityKind::Face, 1, "ext#3:cap(lo)",
         "cut away outside the target"},
        {b::Descent::Vanished, b::EntityKind::Face, 2,
         true, 1, b::EntityKind::Face, 2, "ext#3:cap(hi)",
         "cut away outside the target"},
    };
    return expected;
}

// Compare one tavola row against one expected row — every field the
// tavola carries (the golden's own shape).
bool row_matches(const b::DescentRow& row, const ExpectedRow& want) {
    if (row.relation != want.relation) return false;
    if (row.entity != want.entity) return false;
    if (row.address != want.address) return false;
    if (row.name != want.name) return false;
    if (want.cause == nullptr) {
        if (!row.cause.empty()) return false;
    } else if (row.cause != want.cause) {
        return false;
    }
    if (want.has_operand) {
        if (row.operands.size() != 1) return false;
        if (row.operands[0].operand != want.operand) return false;
        if (row.operands[0].kind != want.operand_kind) return false;
        if (row.operands[0].address != want.operand_address) return false;
    } else if (!row.operands.empty()) {
        return false;
    }
    return row.from.empty();
}

} // namespace

TEST_CASE("H2-U6-M4 the tavola piena: the guide case's 47 rows, "
          "golden row by row") {
    const auto g = guide_case(true);
    const b::DescentTable t =
        b::subtract_descent(*g.result, *g.box, *g.tool);

    // D2's own count, verified: 36 out + 11 vanished = 47.
    REQUIRE(t.rows.size() == 47);
    CHECK(t.count(b::Descent::Kept) == 24);     // 8V + 12E + 4F
    CHECK(t.count(b::Descent::Absorbed) == 3);  // 2 pierced + 1 wall
    CHECK(t.count(b::Descent::Born) == 9);      // 4V + 5E
    CHECK(t.count(b::Descent::Vanished) == 11); // 4V + 5E + 2F
    CHECK(t.count(b::Descent::Split) == 0);
    // MERGED: ZERO instances — the verified fact (the flush case,
    // below, re-verifies it where the dedup rule bites).
    CHECK(t.count(b::Descent::Merged) == 0);

    // The golden, row by row: entity, relation, the operand ref, the
    // name, the cause — every field the tavola carries.
    const auto& expected = guide_case_expected();
    REQUIRE(expected.size() == t.rows.size());
    for (std::size_t i = 0; i < t.rows.size(); ++i) {
        CAPTURE(i);
        CHECK(row_matches(t.rows[i], expected[i]));
    }

    // The per-kind tally, D2's own table:
    //   faces 7 out (4 kept + 3 absorbed) + 2 vanished (the caps)
    //   edges 17 out (12 kept + 5 born) + 5 (the tool's rims x4, seam)
    //   vertices 12 out (8 kept + 4 born) + 4 (the seam/antipode pairs)
    std::size_t out[3] = {0, 0, 0}, gone[3] = {0, 0, 0};
    for (const auto& row : t.rows) {
        const std::size_t k =
            row.entity == b::EntityKind::Vertex ? 0
            : row.entity == b::EntityKind::Edge ? 1 : 2;
        if (row.relation == b::Descent::Vanished)
            ++gone[k];
        else
            ++out[k];
    }
    CHECK(out[0] == 12); CHECK(out[1] == 17); CHECK(out[2] == 7);
    CHECK(gone[0] == 4); CHECK(gone[1] == 5); CHECK(gone[2] == 2);

    // The born rows carry NO operand and NO from — their genealogy
    // rides their NAME (D1: the relation is part of the role); the
    // kept/absorbed rows carry the operand ref and no from (the two
    // channels serve the two operand classes, each named at its site).
    for (const auto& row : t.rows) {
        if (row.relation == b::Descent::Born) {
            CHECK(row.operands.empty());
            CHECK(row.from.empty());
        } else {
            CHECK(row.from.empty());
        }
    }

    // The dead entities' OWN names on the vanished rows — the caps
    // die as 'ext#3:cap(lo)'/'ext#3:cap(hi)', the seam as
    // 'ext#3:seam' (D1's eighth row, the tool's caps dead, named).
    for (const auto& row : t.rows)
        if (row.relation == b::Descent::Vanished &&
            row.entity == b::EntityKind::Face)
            CHECK(row.name == b::query_face(*g.tool,
                     static_cast<b::FaceId>(row.address)).name);
}

TEST_CASE("H2-U6-M4 merged zero is a VERIFIED FACT: the flush case "
          "re-verified where the dedup rule bites") {
    // The coplanar cap flush (unit 4's own degenerate): the tool's
    // bottom cap exactly ON the box's bottom face — the ADR-0023/0024
    // coplanar dedup rule. The tavola says what the code does: the
    // pierced face absorbs, the flush cap VANISHES under the same
    // cut-away cause — a 1:1 + 1:0, NOT a k->1; NOTHING is emitted
    // for the caps, which IS the dedup (the assembly's own comment,
    // subtract.hpp's, cited by the tavola's derivation). merged stays
    // ZERO — an absence VERIFIED on the very case that could have
    // produced it, never a void.
    const auto box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
    const auto tool = b::make_prism(circle_chains(), 3.0f, +1,
                                    FeatureId{3});
    const b::SubtractNaming ctx{
        FeatureId{4}, &n::kBoxNaming, &n::kExtNaming};
    auto r = b::exact_subtract(*box, *tool, {0.0, 0.0, -1.0},
                               b::kSubtractTolerance, ctx);
    REQUIRE(r->face_count() == 7);  // the flush keeps the guide counts
    const b::DescentTable t = b::subtract_descent(*r, *box, *tool);
    REQUIRE(t.rows.size() == 47);
    CHECK(t.count(b::Descent::Merged) == 0);
    // Both caps die under the SAME cause — the flush variant covered.
    for (const auto& row : t.rows)
        if (row.relation == b::Descent::Vanished &&
            row.entity == b::EntityKind::Face)
            CHECK(row.cause == "cut away outside the target");
    // ...and the closure holds there too.
    std::string why;
    CHECK(b::descent_closed(*r, t, &why));
    CHECK(b::old_generation_closed(*box, *tool, t, &why));
}

// ===========================================================================
// FAMILY 3 — THE CLOSURE, BOTH SIDES, AND THE COUNTERPROOFS
// ===========================================================================

TEST_CASE("H2-U6-M4 the closure: both sides green on the guide case") {
    const auto g = guide_case(true);
    const b::DescentTable t =
        b::subtract_descent(*g.result, *g.box, *g.tool);
    std::string why;
    // The result side: every entity of the result in exactly one row.
    CHECK(b::descent_closed(*g.result, t, &why));
    // The old side: every entity of A AND of B, each in exactly one
    // row's operands — the two-sided closure, active where the first
    // tavola with operands lands it.
    CHECK(b::old_generation_closed(*g.box, *g.tool, t, &why));
    // The derived state's own purity: derived twice, identical rows.
    const b::DescentTable t2 =
        b::subtract_descent(*g.result, *g.box, *g.tool);
    REQUIRE(t2.rows.size() == t.rows.size());
    for (std::size_t i = 0; i < t.rows.size(); ++i) {
        CHECK(t2.rows[i].relation == t.rows[i].relation);
        CHECK(t2.rows[i].entity == t.rows[i].entity);
        CHECK(t2.rows[i].address == t.rows[i].address);
        CHECK(t2.rows[i].operands == t.rows[i].operands);
        CHECK(t2.rows[i].name == t.rows[i].name);
        CHECK(t2.rows[i].cause == t.rows[i].cause);
    }
}

TEST_CASE("H2-U6-M4 the closure's counterproofs: the injected defects "
          "are caught and named, one by one") {
    const auto g = guide_case(true);
    const b::DescentTable good =
        b::subtract_descent(*g.result, *g.box, *g.tool);
    std::string why;
    REQUIRE(b::descent_closed(*g.result, good, &why));
    REQUIRE(b::old_generation_closed(*g.box, *g.tool, good, &why));

    // (1) A LOST RESULT ROW — the tavola that loses an entity is a
    // defect the check names, never a silent gap.
    {
        b::DescentTable broken = good;
        broken.rows.erase(broken.rows.begin());  // vertex 0's row
        CHECK_FALSE(b::descent_closed(*g.result, broken, &why));
        CHECK(why.find("LOST") != std::string::npos);
        CHECK(why.find("vertex 0") != std::string::npos);
        CHECK(why.find("remedy") != std::string::npos);
    }
    // (2) A LOST VANISHED ROW — the OLD side's own: the dead entity
    // loses its row, the old generation's coverage names it.
    {
        b::DescentTable broken = good;
        for (std::size_t i = 0; i < broken.rows.size(); ++i)
            if (broken.rows[i].relation == b::Descent::Vanished &&
                broken.rows[i].entity == b::EntityKind::Face) {
                broken.rows.erase(broken.rows.begin() +
                                  static_cast<std::ptrdiff_t>(i));
                break;
            }
        CHECK(b::descent_closed(*g.result, broken, &why));
        CHECK_FALSE(b::old_generation_closed(*g.box, *g.tool,
                                             broken, &why));
        CHECK(why.find("LOST") != std::string::npos);
        CHECK(why.find("tool's") != std::string::npos);
    }
    // (3) A DOUBLE-BOOKING — two rows carry one entity: the extra row
    // is the defect, read from either side.
    {
        b::DescentTable broken = good;
        broken.rows.push_back(broken.rows[0]);  // vertex 0 twice
        CHECK_FALSE(b::descent_closed(*g.result, broken, &why));
        CHECK(why.find("DOUBLE-BOOKS") != std::string::npos);
    }
    // (4) A DANGLING OPERAND REF — the suspended reference, the exact
    // defect the closure exists to forbid.
    {
        b::DescentTable broken = good;
        broken.rows[0].operands[0].address = 99;  // no such box vertex
        CHECK_FALSE(b::old_generation_closed(*g.box, *g.tool,
                                             broken, &why));
        CHECK(why.find("DANGLES") != std::string::npos);
        CHECK(why.find("99") != std::string::npos);
    }
    // (5) A BORN ROW THAT BOOKS OPERANDS — a born row's genealogy
    // rides its name; operands on one are a bookkeeping defect.
    {
        b::DescentTable broken = good;
        for (auto& row : broken.rows)
            if (row.relation == b::Descent::Born) {
                row.operands = {{0, b::EntityKind::Face, 0}};
                break;
            }
        CHECK_FALSE(b::old_generation_closed(*g.box, *g.tool,
                                             broken, &why));
        CHECK(why.find("BORN row books") != std::string::npos);
    }
    // (6) A VANISHED ROW THAT DISAGREES WITH ITSELF — the row's own
    // subject must be the dead ref's.
    {
        b::DescentTable broken = good;
        for (auto& row : broken.rows)
            if (row.relation == b::Descent::Vanished) {
                row.address = 2;  // the ref still says 0
                break;
            }
        CHECK_FALSE(b::old_generation_closed(*g.box, *g.tool,
                                             broken, &why));
        CHECK(why.find("disagrees with itself") != std::string::npos);
    }
}

// ===========================================================================
// FAMILY 4 — THE ROUND-TRIP ABI (structure -> name -> structure)
// ===========================================================================

TEST_CASE("H2-U6-M4 round-trip: every relational name, idem and back, "
          "character for character") {
    const auto g = guide_case(true);
    const SubRegistry reg(g);

    // THE NO-PARSER BOUNDARY, restated where it bites: the round
    // trip's name -> SubshapeRef leg is the VOCABULARY's frozen
    // bijection (role_index_of — one word) plus the STRUCTURAL
    // components the caller holds (the operands' own stamps — the
    // same knowledge the stamping site read); the param never crosses
    // back through the string, and morse 2's compile-time probes (no
    // from_string, no parse_name) still guard the negative in this
    // suite's own files.
    const std::uint32_t isect_idx =
        n::role_index_of(n::kSubEdgeRoles, "isect");
    const std::uint32_t seam_idx =
        n::role_index_of(n::kSubEdgeRoles, "seam");
    REQUIRE(isect_idx == n::kSubIsectLoRole);  // the pair's FIRST row
    REQUIRE(seam_idx == n::kSubSeamRole);
    REQUIRE(n::role_index_of(n::kBoxFaceRoles, "top") == 1);

    // Leg 2's knowledge, read from the operands' OWN stamps — the
    // same sources the stamping site concatenated: the rim @lo names
    // the pierced BOTTOM face (box#1:bottom, row 0), the rim @hi the
    // pierced TOP face (box#1:top, row 1).
    const SubshapeRef& box_bottom =
        g.box->solid.shell().faces()[0].shape_id().subshape;
    const SubshapeRef& box_top =
        g.box->solid.shell().faces()[1].shape_id().subshape;
    const SubshapeRef& wall =
        g.tool->solid.shell().faces()[0].shape_id().subshape;
    REQUIRE(box_bottom.depth == 1);  // [the box's "bottom" row]
    REQUIRE(box_top.depth == 1);     // [the box's "top" row]
    REQUIRE(wall.depth == 2);        // [lat's row, the chain entity]

    struct Case {
        b::EdgeId id;
        std::uint32_t row;  // the levelled entry's OWN row — the pair
                            // shares a token; the LEVEL is the
                            // entry's, the bijection's first hit
                            // cannot tell them apart (the honest
                            // boundary, pinned as the design it is).
        const SubshapeRef* leg_a;
        const SubshapeRef* leg_b;
    };
    const Case cases[5] = {
        {12, n::kSubIsectLoRole, &box_bottom, &wall},  // @lo, half 1
        {13, n::kSubIsectLoRole, &box_bottom, &wall},  // @lo, half 2
        {14, n::kSubIsectHiRole, &box_top, &wall},     // @hi, half 1
        {15, n::kSubIsectHiRole, &box_top, &wall},     // @hi, half 2
        {16, n::kSubSeamRole, nullptr, &wall},         // the seam
    };
    for (const Case& c : cases) {
        const b::EdgeInfo q = b::query_edge(*g.result, c.id);
        REQUIRE_FALSE(q.name.empty());
        // Leg 1: the stored structure renders the stored name —
        // twice, identically (the render is pure).
        const std::string name1 = n::render_name(
            q.id.feature, q.id.subshape, reg.naming.edges());
        CHECK(name1 == q.name);
        CHECK(n::render_name(q.id.feature, q.id.subshape,
                             reg.naming.edges()) == name1);
        // Leg 2: the row (the levelled entry's own) + the operands'
        // own stamps rebuild the ref — the rebuilt ref EQUALS the
        // stored one and renders the identical name.
        SubshapeRef back;
        back = back.push(c.row);
        if (c.leg_a != nullptr)
            for (std::uint8_t k = 0; k < c.leg_a->depth; ++k)
                back = back.push(c.leg_a->path[k]);
        for (std::uint8_t k = 0; k < c.leg_b->depth; ++k)
            back = back.push(c.leg_b->path[k]);
        CHECK(back == q.id.subshape);
        CHECK(n::render_name(q.id.feature, back,
                             reg.naming.edges()) == name1);
    }

    // The absorbed wall's round-trip: the inherited stamp IS the
    // operand's own — the same structure, the same render, no table.
    {
        const b::FaceInfo wall_q = b::query_face(*g.result, 6);
        const b::FaceInfo operand_q = b::query_face(*g.tool, 0);
        CHECK(wall_q.name == operand_q.name);
        CHECK(wall_q.id.subshape == operand_q.id.subshape);
        CHECK(wall_q.id.feature == operand_q.id.feature);
    }
}

TEST_CASE("H2-U6-M4 the depth declaration: the rim resolves at 4, "
          "the seam at 3 — kMaxDepth = 8 holds with room") {
    const auto g = guide_case(true);
    // THE DERIVED DEPTH of the guide case's names (the morse's own
    // declaration, D1's own derivation): the rim's path is
    // [the isect entry, the box's top row, ext's lat row, the chain
    // entity] — sub -> isect -> box/ext -> the entity: depth 4. The
    // seam: [the seam entry, ext's lat row, the entity] — depth 3.
    // The kept/absorbed names are the OPERANDS' own depths (the box's
    // rows 1, the wall 2) — inheritance copies them verbatim.
    CHECK(b::query_edge(*g.result, 12).id.subshape.depth == 4);
    CHECK(b::query_edge(*g.result, 14).id.subshape.depth == 4);
    CHECK(b::query_edge(*g.result, 16).id.subshape.depth == 3);
    for (b::FaceId i = 0; i < 6; ++i)
        CHECK(b::query_face(*g.result, i).id.subshape.depth == 1);
    CHECK(b::query_face(*g.result, 6).id.subshape.depth == 2);
    for (b::EdgeId i = 0; i < 12; ++i)
        CHECK(b::query_edge(*g.result, i).id.subshape.depth == 1);

    // The declaration itself: 4 < kMaxDepth; the guide case's
    // subtract-names remain v1's DEEPEST (morse 3's own pin, held).
    CHECK(4 < SubshapeRef::kMaxDepth);
    CHECK(SubshapeRef::kMaxDepth == 8);
}

TEST_CASE("H2-U6-M4 the morse-3 register's item 4 LANDS: the "
          "multi-chain operand view inside the relational render") {
    // The register's own words, redeemed by their consumer: "a
    // relational role's OPERAND view renders the FEATURE form today"
    // — no more. A multi-chain operand renders the GROUP form for its
    // chain-owned roles inside the brackets, the path's own first
    // component the group. The subtract's class checks refuse the
    // ring tools (non-convex), so NO in-class body produces a
    // multi-chain operand — the machinery is pinned HERE, at the
    // naming layer, with a hand-built registry: the day a real case
    // arrives, the render is already right.
    const n::OperandBinding mc_operands[2] = {
        {FeatureId{1}, &n::kBoxNaming},
        {FeatureId{3}, &n::kExtNaming, /*multi_chain=*/true},
    };
    const n::OperationNaming mc_naming{
        n::FeatureKind::Subtract, {}, n::kSubEdgeRoles, mc_operands};
    // The path: [isect@hi, box's top, the group, lat's row, entity 0]
    // — depth 5, still under kMaxDepth = 8.
    SubshapeRef mc;
    mc = mc.push(n::kSubIsectHiRole)
             .push(1)   // the box's "top" row
             .push(0)   // the tool's chain group 0
             .push(0)   // lat's row
             .push(0);  // the chain entity
    CHECK(mc.depth == 5);
    CHECK(mc.depth < SubshapeRef::kMaxDepth);
    CHECK(n::render_name(FeatureId{4}, mc, mc_naming.edges()) ==
          "sub#4:isect[box#1:top × ext#3:c0:lat(0)]@hi");
    // And the seam under the same machinery: the group prefixes the
    // CHAIN-OWNED role only — the wall's seam inside the brackets.
    SubshapeRef mc_seam;
    mc_seam = mc_seam.push(n::kSubSeamRole)
                    .push(0)  // the group
                    .push(0)  // lat's row
                    .push(0);  // the entity
    CHECK(n::render_name(FeatureId{4}, mc_seam, mc_naming.edges()) ==
          "sub#4:seam[ext#3:c0:lat(0)]");
}

TEST_CASE("H2-U6-M4 the render is loud: the foreign paths and the "
          "unbound slots refuse with diagnosis + remedy") {
    const FeatureId f{4};
    // An operand the registry does not bind: slot 1 reads binding 1
    // and the registry binds none — the hand-built registry's own
    // refusal (the unnamed mode's honest shape, loud when misused).
    const n::OperationNaming bare{
        n::FeatureKind::Subtract, {}, n::kSubEdgeRoles, {}};
    SubshapeRef dangling;
    dangling = dangling.push(n::kSubIsectHiRole).push(1).push(0).push(0);
    CHECK_THROWS_AS(
        n::render_name(f, dangling, bare.edges()),
        std::invalid_argument);
    // An out-of-table entry row and the empty path (the morsi 2-3
    // refusals, still loud under the new tables).
    SubshapeRef foreign;
    foreign = foreign.push(99);
    CHECK_THROWS_AS(n::render_name(f, foreign, bare.edges()),
                    std::invalid_argument);
    SubshapeRef empty;
    CHECK_THROWS_AS(n::render_name(f, empty, bare.edges()),
                    std::invalid_argument);
    CHECK_THROWS_AS(n::role_index_of(n::kSubEdgeRoles, "lat"),
                    std::invalid_argument);
}

// ===========================================================================
// FAMILY 5 — THE PURITY, THE WHOLE CHAIN (box -> sketch -> extrude ->
// subtract)
// ===========================================================================

TEST_CASE("H2-U6-M4 purity: the whole chain's digests, character for "
          "character — the fillet's string, alive in the digest") {
    const auto g = guide_case(true);

    // BOX#1 — the chain's root body (the factory's own digest, m2's
    // shape, at the D1 chain's id).
    CHECK(name_digest(*g.box) ==
          "0|box#1:bottom\n"
          "1|box#1:top\n"
          "2|box#1:front\n"
          "3|box#1:back\n"
          "4|box#1:left\n"
          "5|box#1:right\n");
    CHECK(edge_name_digest(*g.box) ==
          "0|box#1:bottom0\n"
          "1|box#1:bottom1\n"
          "2|box#1:bottom2\n"
          "3|box#1:bottom3\n"
          "4|box#1:top0\n"
          "5|box#1:top1\n"
          "6|box#1:top2\n"
          "7|box#1:top3\n"
          "8|box#1:vertical0\n"
          "9|box#1:vertical1\n"
          "10|box#1:vertical2\n"
          "11|box#1:vertical3\n");

    // SK#2 -> EXT#3 — the sketch's identity flows through the
    // extrude's tavola (the FROMs' renders, m3's own pins): the
    // authored root 'sk#2:0' owns the wall, the rims and the seam,
    // 'sk#2:face' owns the caps. The chain's sketch leg is the
    // RECIPE (the authored data) — its digest is the extrude's
    // tavola's own FROM renders, re-asserted here on the same chain.
    {
        const b::DescentTable ext_t = b::extrude_descent(
            *g.tool, g.chains, FeatureId{3}, FeatureId{2});
        std::size_t from_e0 = 0, from_face = 0;
        for (const auto& row : ext_t.rows) {
            REQUIRE(row.from.size() == 1);
            if (row.from[0].kind == n::RootKind::Face) ++from_face;
            else ++from_e0;
        }
        CHECK(from_e0 == 10);  // sk#2:0
        CHECK(from_face == 2);  // sk#2:face
        CHECK(n::render_root(
                  {FeatureId{2}, n::RootKind::Entity, 0, 0},
                  /*sketch_multi_chain=*/false) == "sk#2:0");
        CHECK(n::render_root({FeatureId{2}, n::RootKind::Face, 0, 0},
                             false) == "sk#2:face");
    }

    // EXT#3 — the tool (m3's own digest, at the D1 chain's id).
    CHECK(name_digest(*g.tool) ==
          "0|ext#3:lat(0)\n"
          "1|ext#3:cap(lo)\n"
          "2|ext#3:cap(hi)\n");
    CHECK(edge_name_digest(*g.tool) ==
          "0|\n"
          "1|\n"
          "2|\n"
          "3|\n"
          "4|ext#3:seam\n");

    // SUB#4 — the result: the D1 names CARATTERE PER CARATTERE.
    // 'box#1:top' the pierced face (absorbed, NOT renamed),
    // 'ext#3:lat(0)' the wall (absorbed, the birth name kept),
    // 'sub#4:isect[box#1:top × ext#3:lat(0)]@hi' the upper rim —
    // LA STRINGA DEL FILLET, viva nel digest.
    CHECK(name_digest(*g.result) ==
          "0|box#1:bottom\n"
          "1|box#1:top\n"
          "2|box#1:front\n"
          "3|box#1:back\n"
          "4|box#1:left\n"
          "5|box#1:right\n"
          "6|ext#3:lat(0)\n");
    CHECK(edge_name_digest(*g.result) ==
          "0|box#1:bottom0\n"
          "1|box#1:bottom1\n"
          "2|box#1:bottom2\n"
          "3|box#1:bottom3\n"
          "4|box#1:top0\n"
          "5|box#1:top1\n"
          "6|box#1:top2\n"
          "7|box#1:top3\n"
          "8|box#1:vertical0\n"
          "9|box#1:vertical1\n"
          "10|box#1:vertical2\n"
          "11|box#1:vertical3\n"
          "12|sub#4:isect[box#1:bottom × ext#3:lat(0)]@lo\n"
          "13|sub#4:isect[box#1:bottom × ext#3:lat(0)]@lo\n"
          "14|sub#4:isect[box#1:top × ext#3:lat(0)]@hi\n"
          "15|sub#4:isect[box#1:top × ext#3:lat(0)]@hi\n"
          "16|sub#4:seam[ext#3:lat(0)]\n");
}

TEST_CASE("H2-U6-M4 purity: N=10 rebuilds of the whole chain, one "
          "answer, bit-identical from the first pinning") {
    std::string first_faces, first_edges, box_faces, tool_faces;
    for (int i = 0; i < 10; ++i) {
        // Same recipes, fresh materialization every time — no state,
        // no counter, no memoization: the names are the recipe's pure
        // functions, stamped in the pass that builds.
        const auto g = guide_case(true);
        const std::string fd = name_digest(*g.result);
        const std::string ed = edge_name_digest(*g.result);
        const std::string bfd = name_digest(*g.box);
        const std::string tfd = name_digest(*g.tool);
        if (i == 0) {
            first_faces = fd;
            first_edges = ed;
            box_faces = bfd;
            tool_faces = tfd;
        } else {
            REQUIRE(fd == first_faces);
            REQUIRE(ed == first_edges);
            REQUIRE(bfd == box_faces);
            REQUIRE(tfd == tool_faces);
        }
    }
    // The pinned FNV-1a 64 constants — the same values under gcc and
    // under clang (the CI's two compilers): the digests carry zero
    // floating-point content, so the cross-compiler identity is the
    // constants' own triviality. Bit-identical from the FIRST
    // pinning (D5's declaration — no prior name state to move).
    CHECK(fnv1a64(first_faces) == 0x5ff1eed6f2fba4c0ull);
    CHECK(fnv1a64(first_edges) == 0x63cf590ec75cc1deull);
    CHECK(fnv1a64(box_faces) == 0x4340ed4737f7ca41ull);
    CHECK(fnv1a64(tool_faces) == 0x27c8a19f27563383ull);
}

TEST_CASE("H2-U6-M4 the feature part flows: sub#9 is another body, "
          "same roles, same addresses") {
    // A different creator id on the subtract: the born names move
    // with it, the addresses do not (the feature part is the name's
    // own, the address the body's).
    GuideCase g;
    g.box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
    g.chains = circle_chains();
    g.tool = b::make_prism(g.chains, 3.0f, +1, FeatureId{3});
    const b::SubtractNaming ctx{
        FeatureId{9}, &n::kBoxNaming, &n::kExtNaming};
    g.result = b::exact_subtract(
        *g.box, *g.tool, {0.0, 0.0, -1.5}, b::kSubtractTolerance, ctx);
    CHECK(b::query_edge(*g.result, 14).name ==
          "sub#9:isect[box#1:top × ext#3:lat(0)]@hi");
    CHECK(b::query_edge(*g.result, 16).name ==
          "sub#9:seam[ext#3:lat(0)]");
    // The operands' parts are the operands' own — a different sub id
    // never renames the box or the tool.
    CHECK(b::query_face(*g.result, 1).name == "box#1:top");
    CHECK(b::query_face(*g.result, 6).name == "ext#3:lat(0)");

    const auto g4 = guide_case(true);
    // The FACE digest is the OPERANDS' own names — the sub id never
    // appears in a face name (the subtract births no faces, D1's own
    // derivation): sub#9 and sub#4 share it, by design. The EDGE
    // digest carries the born names — THAT is where the feature part
    // of the subtract lives, and the two differ there.
    CHECK(name_digest(*g.result) == name_digest(*g4.result));
    CHECK(edge_name_digest(*g.result) != edge_name_digest(*g4.result));
    // The default (feature 0) serves the standalone callers — the
    // scene's hybrid materialization reads sub#0 until the resolver's
    // morso wires the DAG ids (D6 rung 4, the register's own note).
    GuideCase g0;
    g0.box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
    g0.chains = circle_chains();
    g0.tool = b::make_prism(g0.chains, 3.0f, +1, FeatureId{3});
    const b::SubtractNaming ctx0{
        FeatureId{}, &n::kBoxNaming, &n::kExtNaming};
    g0.result = b::exact_subtract(
        *g0.box, *g0.tool, {0.0, 0.0, -1.5}, b::kSubtractTolerance,
        ctx0);
    CHECK(b::query_edge(*g0.result, 14).name ==
          "sub#0:isect[box#1:top × ext#3:lat(0)]@hi");
}

// ===========================================================================
// FAMILY 6 — THE ADDRESS PLAN (additivity, mirrored from unit 4)
// ===========================================================================

TEST_CASE("H2-U6-M4 the address plan intact: the subtract's answers "
          "read exactly what unit 4 pinned") {
    const auto g = guide_case(true);
    const b::BodySummary sum = b::query_body(*g.result);
    CHECK(sum.vertex_count == 12);
    CHECK(sum.edge_count == 17);
    CHECK(sum.face_count == 7);
    CHECK(sum.inner_loops == 2);
    CHECK(sum.chi == 0);
    CHECK(sum.genus_derived);
    CHECK(sum.genus == 1);

    // The wall: ONE CylinderSurface-family face (the exact wall vs
    // the faceted today), the sketch radius, region-on-right.
    CHECK(b::query_face(*g.result, 6).family == "cylinder");
    // The pierced pair: the box's own planes, now with the inner
    // loops (the rim circles as Arc2 pairs, hole = true).
    for (const b::FaceId pierced : {b::FaceId{0}, b::FaceId{1}}) {
        const b::FaceInfo q = b::query_face(*g.result, pierced);
        CHECK(q.family == "plane");
        CHECK(q.loop_count == 2);
        CHECK(q.outer_loop_count == 1);
        CHECK(q.inner_loop_count == 1);
        CHECK(q.exact);
        CHECK(q.linear_tolerance == 0.0);
    }
    // Every face exact/0, every edge manifold-with-XOR (the radial
    // contract — the seam pair lives in ONE loop).
    for (b::FaceId i = 0; i < 7; ++i) {
        const b::FaceInfo q = b::query_face(*g.result, i);
        CHECK(q.exact);
    }
    for (b::EdgeId i = 0; i < 17; ++i) {
        const b::EdgeInfo q = b::query_edge(*g.result, i);
        CHECK(q.manifold);
        CHECK(q.coedge_count == 2);
        CHECK(q.exact);
    }

    // THE NUMBERING NEVER MOVED: the born entities sit at the SAME
    // addresses unit 4's pins rode on (edges 12..16, vertices 8..11,
    // face 6) — the names ride BESIDE, and the born rim halves sit
    // exactly where the walk tables put them (12/13 the bottom pair,
    // 14/15 the top pair, 16 the seam).
    for (b::EdgeId i = 0; i < 17; ++i)
        CHECK(g.result->edges[i].id() == i);
    CHECK(g.result->vertices.size() == 12);
    CHECK(g.result->vertices[8].id() == 8);
    CHECK(g.result->vertices[11].id() == 11);
    CHECK(g.result->solid.shell().faces()[6].id() == 6);

    // The existing suite's own oracles are the global proof; this
    // family re-asserts the G1 unit-4 values verbatim, and the m2
    // boundary pin moved as it was PRE-REGISTERED to move (the wall's
    // inheritance — m2's own commit act, not a re-pin).
}

// ===========================================================================
// FAMILY 8 — THE LAYER-2 BILL, FIELD BY FIELD (the D2 bill verified,
// not promised)
// ===========================================================================

TEST_CASE("H2-U6-M4 the Layer-2 bill: kept carries the operand's "
          "records; absorbed carries the operation's or the loop "
          "change — field by field") {
    const auto g = guide_case(true);
    const auto& res = g.result->solid.shell().faces();
    const auto& src = g.box->solid.shell().faces();
    const auto& tl = g.tool->solid.shell().faces();

    // The field-by-field comparator (the bill's own unit).
    const auto same_record = [](const b::ProvenanceInfo& a,
                                 const b::ProvenanceInfo& b_) {
        return a.is_exact() == b_.is_exact() &&
               a.linear_tolerance == b_.linear_tolerance;
    };

    // KEPT — the four sides: the operand's own records, UNTOUCHED
    // (geometry, loops, Layer-2 record all the operand's own), and
    // the loop structure unchanged (1 loop, 0 inner).
    for (b::FaceId i : {b::FaceId{2}, b::FaceId{3}, b::FaceId{4},
                        b::FaceId{5}}) {
        CHECK(same_record(res[i].provenance(), src[i].provenance()));
        CHECK(res[i].loops().size() == src[i].loops().size());
        CHECK(res[i].name() == src[i].name());
    }
    // ... the 12 kept edges and the 8 kept corners: the same bill,
    // the copy carried them verbatim.
    for (std::size_t i = 0; i < 12; ++i) {
        CHECK(same_record(g.result->edges[i].provenance(),
                          g.box->edges[i].provenance()));
    }
    for (std::size_t i = 0; i < 8; ++i) {
        CHECK(same_record(g.result->vertices[i].provenance(),
                          g.box->vertices[i].provenance()));
    }

    // ABSORBED, the pierced pair: the RECORD stays the operand's own
    // (the copy carried it — the trim does not launder the record),
    // and the MARK is the LOOP change: +1 inner loop each (the born
    // rim circle), the outer loop untouched.
    for (b::FaceId i : {b::FaceId{0}, b::FaceId{1}}) {
        CHECK(same_record(res[i].provenance(), src[i].provenance()));
        CHECK(res[i].loops().size() == src[i].loops().size() + 1);
        CHECK(b::query_face(*g.result, i).inner_loop_count == 1);
        CHECK(res[i].name() == src[i].name());  // NOT renamed
    }

    // ABSORBED, the wall: the OPERATION's record — combined at the
    // creation site (the combined record where the operation creates,
    // D2's own sentence), NOT the tool's lateral's own record read
    // back: the row says it, the record proves it.
    const b::ProvenanceInfo combined =
        b::combined_provenance(*g.box, *g.tool);
    CHECK(same_record(res[6].provenance(), combined));
    // The operation's mark is visible in the p-curves too: the wall's
    // window was RE-TIMED to [z_bot, z_top] on the FLIPPED orientation
    // (region-on-right) — the tool's lateral's own loop carried the
    // FULL periodic window; the result's wall carries the patch.
    CHECK(res[6].loops().size() == tl[0].loops().size());
    CHECK(res[6].name() == tl[0].name());  // the birth name kept

    // The born edges' records are the operation's own (combined at
    // the creation site) — the same bill, the born side.
    for (std::size_t i = 12; i < 17; ++i)
        CHECK(same_record(g.result->edges[i].provenance(), combined));
    for (std::size_t i = 8; i < 12; ++i)
        CHECK(same_record(g.result->vertices[i].provenance(),
                          combined));
}
