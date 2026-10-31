// tests/unit/brep/test_h2_u6_m2_factory_names.cpp
//
// H2 unit 6, morso 2 — THE ROOTS (ADR-0036 D6.1, the accepted act):
// the factories' names at industrial contract. The discriminating
// seven, every claim its own test, the naive version of each one
// failing where the naming is new and not repainted:
//
//   1. THE EXACT NAMES — golden per token, string for string against
//      ADR-0036 D1's own tables: the box's six faces and twelve
//      edges, the sphere's face and seam, the cylinder's wall, caps,
//      rim arcs and seam. The rendering goldens of the GRAMMAR ride
//      here too: D1's relational names — the rim circles, the seam
//      vertical — pinned character for character.
//   2. THE PURITY — two materializations of the same recipe render
//      the same digest, bit-identical; N=10 rebuilds agree; a
//      different feature id names a different body (the feature part
//      flows, no counter, no state).
//   3. THE UNIQUENESS + TOTALITY — no two entities of a body share
//      a name; every face and every edge of the covered classes HAS
//      one (the box has no anonymous citizens); the classes OUTSIDE
//      v1 (the tetrahedron, the extrude's products, the subtract's
//      born entities) read honestly EMPTY, never a guess.
//   4. THE ROUND-TRIP ABI — for every entity of every factory:
//      name -> SubshapeRef -> name, identical; the SubshapeRef is
//      POPULATED for the first time in the repository's history
//      (ADR-0005 redeemed — its only reader is no longer to_string).
//   5. THE STRUCTURAL COUNTERPROOF — equality is structural (ShapeId
//      ==), never string comparison; the same ShapeId renders
//      identically (the render is pure); NO from-string constructor
//      and NO name parser/comparator exists in the public surface
//      (the compile-time negatives below).
//   6. THE ADDRESS PLAN — the factories' per-face answers (family,
//      loops, provenance) read exactly what unit 2 pinned: the name
//      column is ADDITIVE, and the full existing suite running
//      without a re-pin is the proof this family mirrors.
//   7. THE TAVOLA — the factories' descent table: all-born, one row
//      per entity, closure passing; a hand-broken table (a dropped
//      row, a double-booked one) FAILS the closure check with the
//      diagnosis naming the entity — the invariant's own naive
//      counterproof.
//
// THE ORACLES (D5's declaration, honored to the letter): the name
// digest is a NEW oracle BESIDE the existing ones — "<address>|
// <name>\n" per face in address order, hashed FNV-1a 64 (pinned as
// golden constants below) — bit-identical from the FIRST pinning;
// no existing golden moves (the suite is the proof, and this file
// re-asserts the factories' own unit-2 values verbatim, not new
// ones). Determinism is ADR-0014's trivial case: the names are
// strings and integers, zero floating-point content.
//
// THE VERBATIM-COPY SURVIVAL (D1's rule, for free): the exact
// subtract copies A's entities verbatim — the guide case's kept
// faces keep the BOX's names through the operation, and the born
// entities read empty until morso 4 names them. The storage design
// (the fields ON the entities) is what makes the copy carry them;
// this family pins that property where it lands.
#include <doctest/doctest.h>

#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/naming.hpp"
#include "CAD_0/brep/descent.hpp"
#include "CAD_0/brep/query.hpp"
#include "CAD_0/brep/topology.hpp"
#include "CAD_0/brep/extrude.hpp"
#include "CAD_0/brep/subtract.hpp"
#include "CAD_0/sdf/sketch_entities.hpp"
#include "CAD_0/geometry/shape_id.hpp"

#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

// ---------------------------------------------------------------------------
// THE NEGATIVE SURFACE (family 5) — compile-time pins, meaningful ones:
// each probe is a dependent requires-expression, so the declaration
// set of the headers is consulted at the probe's instantiation — a
// from-string constructor or a name parser/comparator added to the
// headers tomorrow makes the static_assert FIRE, exactly as ordered.
// ---------------------------------------------------------------------------

// A member from_string on the ABI types (static or not — both are
// callable through the object): the kernel never parses strings.
template <typename T>
concept HasFromString =
    requires (const T& v) { v.from_string(std::string{}); };
static_assert(!HasFromString<CAD_0::geometry::ShapeId>,
              "ADR-0036 D1: no constructor from a name string — the "
              "render is one direction, structure -> string");
static_assert(!HasFromString<CAD_0::geometry::SubshapeRef>,
              "ADR-0036 D1: the path is built by materialization, "
              "never parsed");

// Free-function probes, lexically INSIDE the namespace so the
// unqualified lookup consults the naming header's own declarations
// (declared before this point in the TU — a later addition to the
// header is seen at the probe's recompile).
namespace CAD_0::brep::naming {
template <typename... A>
concept HasParseName =
    requires (A&&... a) { parse_name(std::forward<A>(a)...); };
static_assert(!HasParseName<std::string>,
              "ADR-0036 D1: no name parser exists in the public "
              "surface");
template <typename... A>
concept HasNameEquals =
    requires (A&&... a) { name_equals(std::forward<A>(a)...); };
static_assert(!HasNameEquals<std::string, std::string>,
              "ADR-0036 D1: no string comparator exists — equality "
              "is structural (ShapeId's own operators)");
} // namespace CAD_0::brep::naming

namespace {

namespace b = CAD_0::brep;
namespace n = CAD_0::brep::naming;
using CAD_0::geometry::FeatureId;
using CAD_0::geometry::ShapeId;
using CAD_0::geometry::SubshapeRef;
using Vec2f = CAD_0::math::Vec2f;

// ---------------------------------------------------------------------------
// The name-digest oracle (D5.1's extension, a NEW oracle beside the
// old ones): every face in address order as "<address>|<name>\n",
// concatenated; the FNV-1a 64 hash is the compact golden. Pure
// integer arithmetic — ADR-0014 honored by construction.
// ---------------------------------------------------------------------------

std::uint64_t fnv1a64(const std::string& s) {
    std::uint64_t h = 0xcbf29ce484222325ull;
    for (const char c : s) {
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

// The circle profile: the guide case's tool (unit 4's own helper,
// verbatim — the same recipe).
std::vector<CAD_0::sdf::SketchChain> circle_chains(
    Vec2f center = {0.0f, 0.0f}, float radius = 0.25f) {
    return {CAD_0::sdf::SketchChain{
        {CAD_0::sdf::sketch_circle(center, radius, false)}}};
}

// ---------------------------------------------------------------------------
// The GRAMMAR's rendering registries (family 1's relational goldens):
// test-declared, the EXACT shapes ADR-0036 D1 pins for the extrude
// and the subtract — the official tables arrive with their morsi
// (D6 rungs 2-3); the RENDERER and the GRAMMAR are what this morse
// pins, with these goldens, character for character.
// ---------------------------------------------------------------------------

// The extrude's roles (D1: lat(i), cap(lo), cap(hi) — the laterals
// and the caps are FACES; the multi-chain c0:/c1: prefix is morso
// 3's, registered).
constexpr n::RoleEntry kExtFaceRoles[3] = {
    {"lat", n::RoleForm::ArgIndex, nullptr, nullptr, {}},
    {"cap", n::RoleForm::ArgWord, "lo", nullptr, {}},
    {"cap", n::RoleForm::ArgWord, "hi", nullptr, {}},
};
constexpr n::OperationNaming kExtNaming{
    n::FeatureKind::Extrude, kExtFaceRoles, {}, {}};

// The subtract's born edge roles (D1: isect[<A-face> × <B-entity>]
// @lo|hi and seam[<B-role>]) — the levelled entries are their own
// table rows (the SSI entries' discipline: the entry names the pair
// AND the level; the path spends no component on it).
constexpr n::OperandSlot kIsectSlots[2] = {
    {0, false, n::RoleForm::Bare},      // the target's face (bare)
    {1, false, n::RoleForm::ArgIndex},  // the tool's wall (indexed)
};
constexpr n::OperandSlot kSeamSlots[1] = {
    {1, false, n::RoleForm::ArgIndex},
};
constexpr n::RoleEntry kSubEdgeRoles[3] = {
    {"isect", n::RoleForm::Relational, nullptr, "lo", kIsectSlots},
    {"isect", n::RoleForm::Relational, nullptr, "hi", kIsectSlots},
    {"seam",  n::RoleForm::Relational, nullptr, nullptr, kSeamSlots},
};
constexpr n::OperandBinding kSubOperands[2] = {
    {FeatureId{1}, &n::kBoxNaming},   // the target: box#1
    {FeatureId{3}, &kExtNaming},      // the tool: ext#3
};
constexpr n::OperationNaming kSubNaming{
    n::FeatureKind::Subtract, {}, kSubEdgeRoles, kSubOperands};

} // namespace

// ===========================================================================
// FAMILY 1 — THE EXACT NAMES, string for string
// ===========================================================================

TEST_CASE("H2-U6-M2 exact names: the box's six faces, D1's own table") {
    const auto body = b::make_box({1.0, 2.0, 3.0}, FeatureId{1});
    REQUIRE(body->face_count() == 6);
    // factory.hpp:126-131's construction table, ADR-0036 D1's words.
    const char* expected[6] = {"bottom", "top", "front",
                               "back", "left", "right"};
    for (b::FaceId i = 0; i < 6; ++i) {
        const b::FaceInfo q = b::query_face(*body, i);
        CHECK(q.name == std::string("box#1:") + expected[i]);
    }
}

TEST_CASE("H2-U6-M2 exact names: the box's twelve edges, the rings' "
          "own vocabulary") {
    const auto body = b::make_box({1.0, 2.0, 3.0}, FeatureId{1});
    REQUIRE(body->edge_count() == 12);
    // factory.hpp:117-120's construction comment, token-legal: the
    // bottom ring, the top ring, the verticals.
    const char* expected[12] = {
        "bottom0", "bottom1", "bottom2", "bottom3",
        "top0", "top1", "top2", "top3",
        "vertical0", "vertical1", "vertical2", "vertical3"};
    for (b::EdgeId i = 0; i < 12; ++i) {
        const b::EdgeInfo q = b::query_edge(*body, i);
        CHECK(q.name == std::string("box#1:") + expected[i]);
    }
}

TEST_CASE("H2-U6-M2 exact names: the sphere's face and seam, "
          "ADR-0023's own words") {
    const auto body = b::make_sphere(1.5, {0.0, 0.0, 0.0}, FeatureId{7});
    REQUIRE(body->face_count() == 1);
    REQUIRE(body->edge_count() == 1);
    CHECK(b::query_face(*body, 0).name == "sph#7:sphere");
    CHECK(b::query_edge(*body, 0).name == "sph#7:seam");
}

TEST_CASE("H2-U6-M2 exact names: the cylinder's wall, caps, rim arcs "
          "and seam — ADR-0023's own words") {
    const auto body = b::make_cylinder(0.5, 2.0, FeatureId{3});
    REQUIRE(body->face_count() == 3);
    REQUIRE(body->edge_count() == 5);
    CHECK(b::query_face(*body, 0).name == "cyl#3:lateral");
    CHECK(b::query_face(*body, 1).name == "cyl#3:top");
    CHECK(b::query_face(*body, 2).name == "cyl#3:bot");
    CHECK(b::query_edge(*body, 0).name == "cyl#3:top1");
    CHECK(b::query_edge(*body, 1).name == "cyl#3:top2");
    CHECK(b::query_edge(*body, 2).name == "cyl#3:bot1");
    CHECK(b::query_edge(*body, 3).name == "cyl#3:bot2");
    CHECK(b::query_edge(*body, 4).name == "cyl#3:seam");
}

TEST_CASE("H2-U6-M2 the grammar's rendering goldens: D1's relational "
          "names, character for character") {
    // The guide case's born entities (ADR-0036 D1's own table):
    //
    //   the top rim circle    sub#4:isect[box#1:top × ext#3:lat(0)]@hi
    //   the bottom rim circle sub#4:isect[box#1:bottom × ext#3:lat(0)]@lo
    //   the wall's seam       sub#4:seam[ext#3:lat(0)]
    //
    // The paths are the ABI's own encoding (D1's depth derivation):
    // the rim resolves at depth 4 — [isect@hi, box's top, ext's
    // lat, the chain entity 0] — and the seam at depth 3. The
    // operands render in DECLARED order, ' × ' separator, never
    // reordered (the grammar's own rule).
    const FeatureId sub{4};

    SubshapeRef rim_hi;
    rim_hi = rim_hi.push(1).push(1).push(0).push(0);  // isect@hi, top, lat, 0
    CHECK(n::render_name(sub, rim_hi, kSubNaming.edges()) ==
          "sub#4:isect[box#1:top × ext#3:lat(0)]@hi");

    SubshapeRef rim_lo;
    rim_lo = rim_lo.push(0).push(0).push(0).push(0);  // isect@lo, bottom, lat, 0
    CHECK(n::render_name(sub, rim_lo, kSubNaming.edges()) ==
          "sub#4:isect[box#1:bottom × ext#3:lat(0)]@lo");

    SubshapeRef seam;
    seam = seam.push(2).push(0).push(0);  // seam, lat, chain 0
    CHECK(n::render_name(sub, seam, kSubNaming.edges()) ==
          "sub#4:seam[ext#3:lat(0)]");

    // The extrude's own forms (D1: lat(i), cap(lo), cap(hi)).
    const FeatureId ext{3};
    SubshapeRef lat0;
    lat0 = lat0.push(0).push(0);
    CHECK(n::render_name(ext, lat0, kExtNaming.faces()) ==
          "ext#3:lat(0)");
    SubshapeRef lat1;
    lat1 = lat1.push(0).push(1);
    CHECK(n::render_name(ext, lat1, kExtNaming.faces()) ==
          "ext#3:lat(1)");
    SubshapeRef cap_lo;
    cap_lo = cap_lo.push(1);
    CHECK(n::render_name(ext, cap_lo, kExtNaming.faces()) ==
          "ext#3:cap(lo)");
    SubshapeRef cap_hi;
    cap_hi = cap_hi.push(2);
    CHECK(n::render_name(ext, cap_hi, kExtNaming.faces()) ==
          "ext#3:cap(hi)");

    // The depth the ADR derives, verified where it lands: the rim
    // at 4, the seam at 3, the factories' own names at 1 — the
    // budget kMaxDepth = 8 holds with room for the genealogy.
    CHECK(rim_hi.depth == 4);
    CHECK(seam.depth == 3);
    CHECK(rim_hi.depth < SubshapeRef::kMaxDepth);
}

TEST_CASE("H2-U6-M2 the render is loud: out-of-table indices and "
          "unconsumed paths refuse with diagnosis + remedy") {
    const FeatureId f{1};
    SubshapeRef bad;
    bad = bad.push(99);  // the box's face table has six rows
    CHECK_THROWS_AS(
        n::render_name(f, bad, n::kBoxNaming.faces()),
        std::invalid_argument);

    SubshapeRef unconsumed;
    unconsumed = unconsumed.push(0).push(1).push(2);  // bare role, 3 deep
    CHECK_THROWS_AS(
        n::render_name(f, unconsumed, n::kBoxNaming.faces()),
        std::invalid_argument);

    SubshapeRef empty;
    CHECK_THROWS_AS(
        n::render_name(f, empty, n::kBoxNaming.faces()),
        std::invalid_argument);
}

// ===========================================================================
// FAMILY 2 — THE PURITY
// ===========================================================================

TEST_CASE("H2-U6-M2 purity: two materializations of the same recipe, "
          "the same names bit for bit") {
    const auto a = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
    const auto b2 = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
    CHECK(name_digest(*a) == name_digest(*b2));

    const auto s1 = b::make_sphere(0.75, {}, FeatureId{2});
    const auto s2 = b::make_sphere(0.75, {}, FeatureId{2});
    CHECK(b::query_face(*s1, 0).name == b::query_face(*s2, 0).name);
    CHECK(b::query_edge(*s1, 0).id ==
          b::query_edge(*s2, 0).id);
}

TEST_CASE("H2-U6-M2 purity: N=10 rebuilds, one digest") {
    std::string reference;
    for (int i = 0; i < 10; ++i) {
        // Same recipe, fresh materialization every time — no state,
        // no counter, no memoization: the name is the recipe's pure
        // function (ADR-0036 D1).
        const auto body = b::make_cylinder(0.5, 2.0, FeatureId{3});
        const std::string digest = name_digest(*body);
        if (i == 0) {
            reference = digest;
        } else {
            REQUIRE(digest == reference);
        }
    }
    // The pinned golden: the digest string and its FNV-1a 64 hash,
    // bit-identical from the FIRST pinning (D5's declaration — no
    // migration, no re-pin, there is no prior name state to move).
    CHECK(reference ==
          "0|cyl#3:lateral\n"
          "1|cyl#3:top\n"
          "2|cyl#3:bot\n");
    CHECK(fnv1a64(reference) == 0xd5069024276f02e7ull);
}

TEST_CASE("H2-U6-M2 the digest goldens: the box and the sphere") {
    const auto box = b::make_box({1.0, 2.0, 3.0}, FeatureId{1});
    const std::string box_digest = name_digest(*box);
    CHECK(box_digest ==
          "0|box#1:bottom\n"
          "1|box#1:top\n"
          "2|box#1:front\n"
          "3|box#1:back\n"
          "4|box#1:left\n"
          "5|box#1:right\n");
    CHECK(fnv1a64(box_digest) == 0x4340ed4737f7ca41ull);

    const auto sph = b::make_sphere(1.5, {}, FeatureId{7});
    const std::string sph_digest = name_digest(*sph);
    CHECK(sph_digest == "0|sph#7:sphere\n");
    CHECK(fnv1a64(sph_digest) == 0x1cd10464e0db0c9bull);
}

TEST_CASE("H2-U6-M2 the feature part flows: a different creator is a "
          "different body, same roles") {
    const auto a = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
    const auto c = b::make_box({1.0, 1.0, 1.0}, FeatureId{9});
    CHECK(b::query_face(*a, 1).name == "box#1:top");
    CHECK(b::query_face(*c, 1).name == "box#9:top");
    CHECK(name_digest(*a) != name_digest(*c));
    // The default parameter serves the bridge's primitive fit and
    // the standalone callers: feature 0, the roles identical.
    const auto d = b::make_box({1.0, 1.0, 1.0});
    CHECK(b::query_face(*d, 1).name == "box#0:top");
}

// ===========================================================================
// FAMILY 3 — THE UNIQUENESS + TOTALITY
// ===========================================================================

TEST_CASE("H2-U6-M2 uniqueness: no two entities of a body share a "
          "name; totality: no anonymous citizens in the covered "
          "classes") {
    const auto box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
    std::set<std::string> names;
    for (b::FaceId i = 0; i < box->face_count(); ++i) {
        const std::string name = b::query_face(*box, i).name;
        REQUIRE_FALSE(name.empty());  // totality: every face named
        CHECK(names.insert(name).second);  // uniqueness
    }
    for (b::EdgeId i = 0; i < box->edge_count(); ++i) {
        const std::string name = b::query_edge(*box, i).name;
        REQUIRE_FALSE(name.empty());
        CHECK(names.insert(name).second);
    }
    CHECK(names.size() == 6 + 12);

    const auto sph = b::make_sphere(1.0, {}, FeatureId{2});
    names.clear();
    for (b::FaceId i = 0; i < sph->face_count(); ++i)
        CHECK(names.insert(b::query_face(*sph, i).name).second);
    for (b::EdgeId i = 0; i < sph->edge_count(); ++i)
        CHECK(names.insert(b::query_edge(*sph, i).name).second);
    CHECK(names.size() == 2);

    const auto cyl = b::make_cylinder(0.5, 1.0, FeatureId{3});
    names.clear();
    for (b::FaceId i = 0; i < cyl->face_count(); ++i)
        CHECK(names.insert(b::query_face(*cyl, i).name).second);
    for (b::EdgeId i = 0; i < cyl->edge_count(); ++i)
        CHECK(names.insert(b::query_edge(*cyl, i).name).second);
    CHECK(names.size() == 3 + 5);
}

TEST_CASE("H2-U6-M2 the honest empty: the classes outside v1 read "
          "unnamed, never a guess") {
    // The tetrahedron: a factory, but outside ADR-0036 D4's three
    // (the grammar's kind vocabulary has no 'tet') — every entity
    // reads the EMPTY name, the default ShapeId.
    const auto tet = b::make_tetrahedron();
    for (b::FaceId i = 0; i < tet->face_count(); ++i) {
        const b::FaceInfo q = b::query_face(*tet, i);
        CHECK(q.name.empty());
        CHECK(q.id == ShapeId{});
    }
    for (b::EdgeId i = 0; i < tet->edge_count(); ++i) {
        const b::EdgeInfo q = b::query_edge(*tet, i);
        CHECK(q.name.empty());
    }

    // The extrude's products: morso 3 LANDED — the boundary this
    // test pinned as "honestly unnamed today" advanced exactly as
    // its own comment pre-registered. The prism's FACES are named
    // now (the walk roles: lat(0), cap(lo), cap(hi) — the default
    // feature 0 on this path) and the seam edge with them; the
    // walk's OTHER edges and the vertices read honestly empty (the
    // D4 queue's discipline). The goldens live in morse 3's own
    // file (test_h2_u6_m3_extrude_names.cpp); this spot-check pins
    // that the advance is additive — no address moved, the names
    // arrived beside them.
    const auto prism = b::make_prism(circle_chains(), 3.0f, +1);
    REQUIRE(prism->face_count() == 3);
    CHECK(b::query_face(*prism, 0).name == "ext#0:lat(0)");
    CHECK(b::query_face(*prism, 1).name == "ext#0:cap(lo)");
    CHECK(b::query_face(*prism, 2).name == "ext#0:cap(hi)");
    CHECK(b::query_edge(*prism, 4).name == "ext#0:seam");
    for (b::EdgeId i = 0; i < 4; ++i)
        CHECK(b::query_edge(*prism, i).name.empty());
}

// ===========================================================================
// FAMILY 4 — THE ROUND-TRIP ABI (name -> SubshapeRef -> name)
// ===========================================================================

TEST_CASE("H2-U6-M2 round-trip: for every entity of every factory, "
          "ref -> name -> ref -> name, identical") {
    // ADR-0005 REDEEMED: the SubshapeRef is populated for the first
    // time in the repository's history — its only reader is no
    // longer to_string (shape.cpp's debug render); the naming reads
    // it, renders it, and walks it BACK through the role tables'
    // own frozen bijection (role_index_of — the vocabulary reverse,
    // one word, never a string parser: the full name never crosses
    // back, there is no from-string constructor to do it with).
    struct Case {
        const char* what;
        std::unique_ptr<b::BRepBody> body;
        n::OperationNaming naming;
    };
    std::vector<Case> cases;
    cases.push_back({"box", b::make_box({1.0, 1.0, 1.0}, FeatureId{1}),
                     n::kBoxNaming});
    cases.push_back({"sphere", b::make_sphere(1.0, {}, FeatureId{2}),
                     n::kSphereNaming});
    cases.push_back({"cylinder", b::make_cylinder(0.5, 1.0, FeatureId{3}),
                     n::kCylinderNaming});

    for (const auto& c : cases) {
        for (b::FaceId i = 0; i < c.body->face_count(); ++i) {
            const b::FaceInfo q = b::query_face(*c.body, i);
            REQUIRE(q.id.subshape.depth == 1);  // POPULATED, depth 1
            // Leg 1: the stored structure renders the stored name.
            const std::string name1 = n::render_name(
                q.id.feature, q.id.subshape, c.naming.faces());
            CHECK(name1 == q.name);
            // Leg 2: the name's token, looked up in the SAME table
            // the render wrote (the frozen bijection), rebuilds the
            // ref; the rebuilt ref equals the stored one, and
            // renders the identical name.
            const std::size_t colon = name1.find(':');
            REQUIRE(colon != std::string::npos);
            const std::string token = name1.substr(colon + 1);
            const std::uint32_t idx =
                n::role_index_of(c.naming.face_roles, token);
            SubshapeRef back;
            back = back.push(idx);
            CHECK(back == q.id.subshape);
            CHECK(n::render_name(q.id.feature, back,
                                 c.naming.faces()) == name1);
        }
        for (b::EdgeId i = 0; i < c.body->edge_count(); ++i) {
            const b::EdgeInfo q = b::query_edge(*c.body, i);
            REQUIRE(q.id.subshape.depth == 1);
            const std::string name1 = n::render_name(
                q.id.feature, q.id.subshape, c.naming.edges());
            CHECK(name1 == q.name);
            const std::size_t colon = name1.find(':');
            REQUIRE(colon != std::string::npos);
            const std::string token = name1.substr(colon + 1);
            const std::uint32_t idx =
                n::role_index_of(c.naming.edge_roles, token);
            SubshapeRef back;
            back = back.push(idx);
            CHECK(back == q.id.subshape);
            CHECK(n::render_name(q.id.feature, back,
                                 c.naming.edges()) == name1);
        }
    }
}

TEST_CASE("H2-U6-M2 role_index_of is loud on a foreign token") {
    CHECK_THROWS_AS(
        n::role_index_of(n::kBoxFaceRoles, "zenith"),
        std::invalid_argument);
}

// ===========================================================================
// FAMILY 5 — THE STRUCTURAL COUNTERPROOF
// ===========================================================================

TEST_CASE("H2-U6-M2 equality is structural: same structure, equal "
          "ids, identical renders — never a string comparison") {
    // Two independently built bodies of the same recipe: the top
    // faces' ShapeIds are EQUAL (the defaulted structural operator
    // compares feature, path and generation — nothing else exists
    // to compare), and the pure render maps the equal structures to
    // identical strings.
    const auto a = b::make_box({2.0, 2.0, 2.0}, FeatureId{5});
    const auto c = b::make_box({9.0, 9.0, 9.0}, FeatureId{5});
    const ShapeId id_a = b::query_face(*a, 1).id;
    const ShapeId id_c = b::query_face(*c, 1).id;
    CHECK(id_a == id_c);  // structure, not geometry, not string
    CHECK(b::query_face(*a, 1).name == b::query_face(*c, 1).name);

    // The render is a pure function: the same ShapeId under the
    // same registry renders the same bytes on every call.
    for (int i = 0; i < 5; ++i) {
        CHECK(n::render_name(id_a.feature, id_a.subshape,
                             n::kBoxNaming.faces()) == "box#5:top");
    }

    // Different structures are NOT equal — the address (the id 1)
    // is not the identity, and two different roles differ
    // structurally even on the same feature.
    CHECK(!(b::query_face(*a, 1).id == b::query_face(*a, 0).id));

    // THE KIND-AGNOSTIC STRUCTURE, pinned as the design it is: a
    // face and an edge of the SAME feature and the SAME table index
    // (box#1's face 0 'bottom' and edge 0 'bottom0') share ONE
    // ShapeId — {feature 1, path [0]} — because the ABI (ADR-0005,
    // unmoved) carries no entity-kind field. The disambiguation is
    // the REFERENCE's own type, exactly where D4's two consumers
    // put it: the fillet's slot is edge-typed, the UI's click is
    // face-typed, and the render consults the kind's table
    // (faces() vs edges()) — the NAMES differ (bottom vs bottom0)
    // even where the ids coincide. A consumer comparing ids across
    // kinds is comparing across reference types — its own defect,
    // not the ABI's.
    CHECK(b::query_face(*a, 0).id == b::query_edge(*a, 0).id);
    CHECK(b::query_face(*a, 0).name == "box#5:bottom");
    CHECK(b::query_edge(*a, 0).name == "box#5:bottom0");
}

// ===========================================================================
// FAMILY 6 — THE ADDRESS PLAN (additivity, mirrored from unit 2)
// ===========================================================================

TEST_CASE("H2-U6-M2 the address plan intact: the factories' per-face "
          "answers read exactly what unit 2 pinned") {
    const auto box = b::make_box({1.0, 2.0, 3.0}, FeatureId{1});
    for (b::FaceId i = 0; i < box->face_count(); ++i) {
        const b::FaceInfo q = b::query_face(*box, i);
        CHECK(q.family == "plane");
        CHECK(q.has_surface);
        CHECK(q.loop_count == 1);
        CHECK(q.outer_loop_count == 1);
        CHECK(q.inner_loop_count == 0);
        CHECK(q.exact);
        CHECK(q.linear_tolerance == 0.0);
    }
    CHECK(box->vertex_count() == 8);
    CHECK(box->edge_count() == 12);
    CHECK(box->face_count() == 6);

    const auto sph = b::make_sphere(1.0, {}, FeatureId{2});
    const b::FaceInfo sf = b::query_face(*sph, 0);
    CHECK(sf.family == "sphere");
    CHECK(sf.loop_count == 1);
    CHECK(sph->vertex_count() == 2);
    CHECK(sph->edge_count() == 1);
    CHECK(sph->face_count() == 1);

    const auto cyl = b::make_cylinder(0.5, 1.0, FeatureId{3});
    CHECK(b::query_face(*cyl, 0).family == "cylinder");
    CHECK(b::query_face(*cyl, 1).family == "plane");
    CHECK(b::query_face(*cyl, 2).family == "plane");
    CHECK(cyl->vertex_count() == 4);
    CHECK(cyl->edge_count() == 5);
    CHECK(cyl->face_count() == 3);

    // The edge query's own address answers (the new surface, the
    // same discipline): the box's edges are manifold, the endpoints
    // the construction's own pairs.
    const b::EdgeInfo e0 = b::query_edge(*box, 0);
    CHECK(e0.v0 == 0);
    CHECK(e0.v1 == 1);
    CHECK(e0.manifold);
    CHECK(e0.coedge_count == 2);
    CHECK(e0.exact);

    // The loud miss, both surfaces: an id the body does not carry
    // is a caller bug, named with the remedy.
    CHECK_THROWS_AS(b::query_edge(*box, 12), std::invalid_argument);
    CHECK_THROWS_AS(b::query_face(*box, 6), std::invalid_argument);
}

// ===========================================================================
// FAMILY 7 — THE TAVOLA (the factories' descent, all-born and closed)
// ===========================================================================

TEST_CASE("H2-U6-M2 the tavola: all-born, one row per entity, the "
          "counts the goldens pin") {
    const auto box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
    const b::DescentTable t = b::factory_descent(*box);
    // 8V + 12E + 6F = 26 born rows, zero operands, the names beside
    // the addresses on the face/edge rows (vertices: v1 has no
    // names — D4's queue, honestly empty).
    REQUIRE(t.rows.size() == 8 + 12 + 6);
    CHECK(t.count(b::Descent::Born) == 26);
    CHECK(t.count(b::Descent::Kept) == 0);
    CHECK(t.count(b::Descent::Vanished) == 0);
    std::size_t faces = 0, edges = 0, vertices = 0;
    std::size_t named = 0;
    for (const auto& row : t.rows) {
        CHECK(row.relation == b::Descent::Born);
        CHECK(row.operands.empty());
        switch (row.entity) {
        case b::EntityKind::Vertex: ++vertices; break;
        case b::EntityKind::Edge:
            ++edges;
            if (!row.name.empty()) ++named;
            break;
        case b::EntityKind::Face:
            ++faces;
            if (!row.name.empty()) ++named;
            break;
        }
    }
    CHECK(vertices == 8);
    CHECK(edges == 12);
    CHECK(faces == 6);
    CHECK(named == 12 + 6);

    // The sphere: 2V + 1E + 1F.
    const auto sph = b::make_sphere(1.0, {}, FeatureId{2});
    const b::DescentTable ts = b::factory_descent(*sph);
    CHECK(ts.rows.size() == 4);
    CHECK(ts.count(b::Descent::Born) == 4);

    // The cylinder: 4V + 5E + 3F.
    const auto cyl = b::make_cylinder(0.5, 1.0, FeatureId{3});
    const b::DescentTable tc = b::factory_descent(*cyl);
    CHECK(tc.rows.size() == 12);
    CHECK(tc.count(b::Descent::Born) == 12);

    // The closure holds on all three (the check the factory tests
    // call; in debug builds factory_descent already asserted it —
    // this is the release-side call of the same invariant).
    std::string why;
    CHECK(b::descent_closed(*box, t, &why));
    CHECK(b::descent_closed(*sph, ts));
    CHECK(b::descent_closed(*cyl, tc));

    // The tavola is DERIVED state, recomputed identically: the same
    // body, the same table, twice (the recipe doctrine — never a
    // parallel truth).
    const b::DescentTable t2 = b::factory_descent(*box);
    REQUIRE(t2.rows.size() == t.rows.size());
    for (std::size_t i = 0; i < t.rows.size(); ++i) {
        CHECK(t2.rows[i].relation == t.rows[i].relation);
        CHECK(t2.rows[i].entity == t.rows[i].entity);
        CHECK(t2.rows[i].address == t.rows[i].address);
        CHECK(t2.rows[i].name == t.rows[i].name);
    }
}

TEST_CASE("H2-U6-M2 the closure check's naive counterproof: a broken "
          "tavola fails loudly, naming the entity") {
    const auto box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
    const b::DescentTable good = b::factory_descent(*box);

    // A LOST row: the tavola that forgets face 5 fails, the why
    // names the face and the remedy.
    b::DescentTable lost = good;
    lost.rows.pop_back();
    std::string why;
    CHECK_FALSE(b::descent_closed(*box, lost, &why));
    CHECK(why.find("face 5") != std::string::npos);
    CHECK(why.find("remedy") != std::string::npos);

    // A DOUBLE-BOOKED row: the same entity booked twice fails the
    // same way, the count named.
    b::DescentTable doubled = good;
    doubled.rows.push_back(good.rows.back());
    CHECK_FALSE(b::descent_closed(*box, doubled, &why));
    CHECK(why.find("DOUBLE-BOOKS") != std::string::npos);

    // A GHOST row: an entity the body does not have — the same
    // defect read from the other side. (The initializer grew one
    // field with morse 3's `from` — the additive channel, empty
    // here: the hand-broken row has no genealogy to lie about.)
    b::DescentTable ghost = good;
    ghost.rows.push_back({b::Descent::Born, b::EntityKind::Face, 99,
                          {}, "box#1:ghost", {}, ""});
    CHECK_FALSE(b::descent_closed(*box, ghost, &why));
    CHECK(why.find("ghost") != std::string::npos);
}

// ===========================================================================
// THE VERBATIM-COPY SURVIVAL — D1's rule for free, pinned where it
// lands: the exact subtract copies A's entities verbatim, and the
// fields ON the entities make the copy carry the names.
// ===========================================================================

TEST_CASE("H2-U6-M2 the guide case: A's kept faces keep the BOX's "
          "names through the subtract; the born entities read empty "
          "until morso 4") {
    const auto target = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
    const auto tool = b::make_prism(circle_chains(), 3.0f, +1);
    const auto result = b::exact_subtract(
        *target, *tool, {0.0, 0.0, -1.5}, b::kSubtractTolerance);

    REQUIRE(result->face_count() == 7);   // the pinned 12/17/7/2
    REQUIRE(result->edge_count() == 17);
    REQUIRE(result->vertex_count() == 12);

    // A's faces keep their ids AND their names: the pierced top and
    // bottom stay the BOX's faces (absorbed — identity is where a
    // thing was born, D1), the four sides keep theirs (kept).
    for (b::FaceId i = 0; i < 6; ++i) {
        const b::FaceInfo q = b::query_face(*result, i);
        REQUIRE_FALSE(q.name.empty());
        CHECK(q.name == b::query_face(*target, i).name);
    }
    CHECK(b::query_face(*result, 1).name == "box#1:top");
    CHECK(b::query_face(*result, 0).name == "box#1:bottom");

    // The born wall (face 6, the tool's lateral re-framed to the
    // window) and the born edges (12..16) read honestly EMPTY —
    // morso 4's territory, pre-registered here.
    CHECK(b::query_face(*result, 6).name.empty());
    for (b::EdgeId i = 12; i < 17; ++i)
        CHECK(b::query_edge(*result, i).name.empty());
    // A's edges keep their names (the twelve box edges).
    for (b::EdgeId i = 0; i < 12; ++i) {
        const b::EdgeInfo q = b::query_edge(*result, i);
        CHECK_FALSE(q.name.empty());
        CHECK(q.name == b::query_edge(*target, i).name);
    }

    // The tavola side of the same honesty: the derived all-born
    // table of the RESULT would lie (the result is not a factory
    // product) — the factories' derivation is for factory bodies,
    // and the subtract's own tavola is morso 4's. Not called here;
    // the boundary is the pin.
}
