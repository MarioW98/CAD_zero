// tests/unit/brep/test_h2_u6_m3_extrude_names.cpp
//
// H2 unit 6, morso 3 — L'EXTRUDE (ADR-0036 D6.2, the accepted act +
// the morse-3 register): the walk roles normative, the born-of
// genealogy, the digest on the extruded bodies. The discriminating
// families, every claim its own test, the naive version of each one
// failing where the naming is new and not repainted:
//
//   1. THE EXACT NAMES — golden per body, string for string against
//      ADR-0036 D1's own vocabulary: the guide case's tool (the
//      cylinder from the circle: lat(0), cap(lo), cap(hi), the seam)
//      and the multi-chain prism (the ring: c0:lat(0..3), the hole's
//      c1:lat(0) and c1:seam, the caps) — the rendering forms under
//      the OFFICIAL kExtNaming (bare, indexed, word, grouped), every
//      grammar gesture the extrude makes.
//   2. THE PURITY — two extrusions of the same recipe render the
//      identical digest, bit for bit; N=10 rebuilds agree; the
//      feature part flows (a different creator is a different body,
//      same roles).
//   3. THE UNIQUENESS + TOTALITY — no two entities of a body share
//      a name; every FACE is named, the seam edge with them; the
//      walk's OTHER edges (the bottom/top curves, the verticals, the
//      rims) and the vertices read honestly EMPTY — D1's v1 extrude
//      vocabulary's own boundary, pinned as such.
//   4. THE ROUND-TRIP ABI — every parametric form: the stored
//      structure renders the stored name, character for character,
//      both directions through the structure (the render is pure);
//      the vocabulary's frozen bijection (role_index_of) agrees with
//      the path's role component; the rebuilt ref equals the stored
//      one and renders the identical name. THE MAXIMUM DEPTH v1 OF
//      THE EXTRUSION CASE IS 3 (the multi-chain lateral) — declared
//      and pinned, kMaxDepth = 8 covering it with room. The kernel
//      NEVER parses: the param never crosses back through the
//      string (morse 2's compile-time probes still guard the
//      negative).
//   5. THE TAVOLA + THE FROM CLOSURE — the guide case's tool COUNTED
//      and pinned (12 born rows: 4V + 5E + 3F, the FROMs 10 x
//      sk#2:0 and 2 x sk#2:face); the ring counted (32 rows); the
//      result-side closure AND the FROM side green; the drift-guard
//      (a wrong feature refuses loudly); the dangling-root
//      counterproof (a hand-broken FROM fails, naming the root).
//   6. THE ADDRESS PLAN — unit-3's own pins re-asserted verbatim on
//      the same recipes (the counts, the families, the caps' loop
//      structure, the provenance): the names arrived BESIDE the
//      addresses, and the full existing suite running without a
//      re-pin is the global proof this family mirrors.
//
// THE ORACLES (D5's declaration, honored to the letter): the name
// digest — the morse-2 oracle, "<address>|<name>\n" per face in
// address order, FNV-1a 64 — now runs on the extrude's bodies, and
// the EDGE leg joins it here (the same shape over the body's own
// edge array: the seam's pin, the honest empties' pin) — both
// bit-identical from the FIRST pinning, no existing golden moved
// (the suite is the proof, and this file re-asserts unit-3's values
// verbatim). Determinism is ADR-0014's trivial case: the names carry
// no floating-point content.
//
// THE ROOT REFERENCE (the morse-3 register): the FROM renders the
// authored roots — 'sk#2:0' (the single-chain sketch's entity 0),
// 'sk#6:c1:0' (the multi-chain sketch's chain 1, entity 0 — the
// chain component only multi-chain), 'sk#2:face' (the extruded
// region, the caps' own parent) — pinned character for character.
#include <doctest/doctest.h>

#include "CAD_0/brep/extrude.hpp"
#include "CAD_0/brep/descent.hpp"
#include "CAD_0/brep/naming.hpp"
#include "CAD_0/brep/query.hpp"
#include "CAD_0/brep/topology.hpp"
#include "CAD_0/sdf/sketch_profile.hpp"
#include "CAD_0/geometry/shape_id.hpp"

#include <cstdint>
#include <memory>
#include <set>
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
// The name-digest oracles: the morse-2 face digest (D5.1) extended to
// the extrude's bodies, and the EDGE leg — the same shape over the
// body's own edge array, first pinned HERE (the seam's golden, the
// honest empties' golden). Pure integer arithmetic — ADR-0014 honored
// by construction.
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

// ---- the recipes (unit 3's own helpers, verbatim — the same
// profiles the G1 pins ride on) -----------------------------------------

// The guide case's tool: a single circle chain, hole = false (the
// solid disc — unit 4's own recipe).
std::vector<s::SketchChain> circle_chains(
    Vec2f center = {0.5f, 0.25f}, float radius = 1.25f) {
    return {s::SketchChain{
        {s::sketch_circle(center, radius, false)}}};
}

// The ring (unit 3's multi-chain flow): the rectangle + a hole
// circle inside — the outer is the sketch's chain 0, the hole chain
// 1; the construction groups follow (outer = c0, hole = c1).
std::vector<s::SketchChain> ring_chains() {
    return {s::SketchChain{{
                s::sketch_segment({-2.0f, -1.0f}, {2.0f, -1.0f}, true),
                s::sketch_segment({2.0f, -1.0f}, {2.0f, 1.0f}, true),
                s::sketch_segment({2.0f, 1.0f}, {-2.0f, 1.0f}, true),
                s::sketch_segment({-2.0f, 1.0f}, {-2.0f, -1.0f}, true),
            }},
            s::SketchChain{
                {s::sketch_circle({0.0f, 0.0f}, 0.2f, true)}}};
}

} // namespace

// ===========================================================================
// FAMILY 1 — THE EXACT NAMES, string for string
// ===========================================================================

TEST_CASE("H2-U6-M3 exact names: the guide case's tool — lat(0), the "
          "caps, the seam, D1's own table") {
    const auto body = b::make_prism(circle_chains(), 3.0f, +1,
                                     FeatureId{3});
    REQUIRE(body->face_count() == 3);
    REQUIRE(body->edge_count() == 5);
    // ADR-0036 D1's guide-case tool column: the hole's wall is
    // ext#3:lat(0) (the circle special case's SINGLE lat(0) wall),
    // the caps the region's two lids, the seam ADR-0023's own.
    CHECK(b::query_face(*body, 0).name == "ext#3:lat(0)");
    CHECK(b::query_face(*body, 1).name == "ext#3:cap(lo)");
    CHECK(b::query_face(*body, 2).name == "ext#3:cap(hi)");
    CHECK(b::query_edge(*body, 4).name == "ext#3:seam");
    // The walk's other edges — the rim halves — are OUTSIDE D1's v1
    // extrude vocabulary: honestly empty, never a guess.
    for (b::EdgeId i = 0; i < 4; ++i)
        CHECK(b::query_edge(*body, i).name.empty());
}

TEST_CASE("H2-U6-M3 exact names: the multi-chain prism — the c0:lat(i), "
          "the hole's c1, the unprefixed caps") {
    const auto body = b::make_prism(ring_chains(), 1.5f, +1,
                                     FeatureId{5});
    REQUIRE(body->face_count() == 7);
    // The outer chain (the sketch's chain 0 — the construction's
    // group 0): the four laterals, group-prefixed.
    for (int i = 0; i < 4; ++i)
        CHECK(b::query_face(*body, i).name ==
              "ext#5:c0:lat(" + std::to_string(i) + ")");
    // The hole chain (the sketch's chain 1 — the construction's
    // group 1): its wall and its seam carry the SAME group.
    CHECK(b::query_face(*body, 4).name == "ext#5:c1:lat(0)");
    CHECK(b::query_edge(*body, 14).name == "ext#5:c1:seam");
    // The caps are the REGION's own, never a chain's: no group, in
    // any body — the single-chain convention holds BOTH ways.
    CHECK(b::query_face(*body, 5).name == "ext#5:cap(lo)");
    CHECK(b::query_face(*body, 6).name == "ext#5:cap(hi)");
    // The walk's other edges (the contour's 12 + the hole's rims):
    // honestly empty.
    for (b::EdgeId i = 0; i < 14; ++i)
        CHECK(b::query_edge(*body, i).name.empty());
}

TEST_CASE("H2-U6-M3 the grammar's forms under the OFFICIAL tables: "
          "bare, indexed, word, grouped — character for character") {
    const FeatureId ext{3};
    // The indexed form, single-chain: [lat, i] — depth 2.
    SubshapeRef lat0;
    lat0 = lat0.push(n::kExtLatRole).push(0);
    CHECK(n::render_name(ext, lat0, n::kExtNaming.faces()) ==
          "ext#3:lat(0)");
    // The word form, feature-owned: [cap] — depth 1.
    SubshapeRef cap_lo;
    cap_lo = cap_lo.push(n::kExtCapLoRole);
    CHECK(n::render_name(ext, cap_lo, n::kExtNaming.faces()) ==
          "ext#3:cap(lo)");
    SubshapeRef cap_hi;
    cap_hi = cap_hi.push(n::kExtCapHiRole);
    CHECK(n::render_name(ext, cap_hi, n::kExtNaming.faces()) ==
          "ext#3:cap(hi)");
    // The bare form, chain-owned: [seam] single-chain — depth 1.
    SubshapeRef seam;
    seam = seam.push(n::kExtSeamRole);
    CHECK(n::render_name(ext, seam, n::kExtNaming.edges()) ==
          "ext#3:seam");
    // The GROUP form (the morse-3 register): the group is the path's
    // FIRST component, under the GROUP view — 'c<g>:' before the
    // token, the chain-owned roles only.
    SubshapeRef g_lat;
    g_lat = g_lat.push(0).push(n::kExtLatRole).push(3);
    CHECK(n::render_name(ext, g_lat, n::kExtNaming.grouped_faces()) ==
          "ext#3:c0:lat(3)");
    SubshapeRef g_seam;
    g_seam = g_seam.push(1).push(n::kExtSeamRole);
    CHECK(n::render_name(ext, g_seam, n::kExtNaming.grouped_edges()) ==
          "ext#3:c1:seam");
    // The single-chain lat under the group view is NOT that lat: the
    // views are disjoint (the group component is consumed first) —
    // the loud side is family 4's.
}

TEST_CASE("H2-U6-M3 the root reference's render: the register's own "
          "three forms") {
    // The single-chain sketch's entity root: the bare position.
    CHECK(n::render_root({FeatureId{2}, n::RootKind::Entity, 0, 0},
                         /*sketch_multi_chain=*/false) == "sk#2:0");
    CHECK(n::render_root({FeatureId{2}, n::RootKind::Entity, 0, 3},
                         false) == "sk#2:3");
    // The multi-chain sketch's root: the chain component prefixes
    // (the SKETCH's own position — the same only-multi-chain
    // elision as the roles' group).
    CHECK(n::render_root({FeatureId{6}, n::RootKind::Entity, 1, 0},
                         true) == "sk#6:c1:0");
    CHECK(n::render_root({FeatureId{6}, n::RootKind::Entity, 0, 2},
                         true) == "sk#6:c0:2");
    // The sketch's FACE — the extruded region, the caps' parent.
    CHECK(n::render_root({FeatureId{2}, n::RootKind::Face, 0, 0},
                         false) == "sk#2:face");
    CHECK(n::render_root({FeatureId{6}, n::RootKind::Face, 0, 0},
                         true) == "sk#6:face");
}

// ===========================================================================
// FAMILY 2 — THE PURITY
// ===========================================================================

TEST_CASE("H2-U6-M3 purity: two extrusions of the same recipe, the "
          "same digests bit for bit") {
    const auto a = b::make_prism(circle_chains(), 3.0f, +1,
                                 FeatureId{3});
    const auto c = b::make_prism(circle_chains(), 3.0f, +1,
                                 FeatureId{3});
    CHECK(name_digest(*a) == name_digest(*c));
    CHECK(edge_name_digest(*a) == edge_name_digest(*c));

    const auto r1 = b::make_prism(ring_chains(), 1.5f, +1, FeatureId{5});
    const auto r2 = b::make_prism(ring_chains(), 1.5f, +1, FeatureId{5});
    CHECK(name_digest(*r1) == name_digest(*r2));
    CHECK(edge_name_digest(*r1) == edge_name_digest(*r2));
}

TEST_CASE("H2-U6-M3 purity: N=10 rebuilds, one digest, the golden "
          "constants from the FIRST pinning") {
    std::string circle_faces, circle_edges, ring_faces, ring_edges;
    for (int i = 0; i < 10; ++i) {
        // Same recipe, fresh materialization every time — no state,
        // no counter, no memoization: the name is the recipe's pure
        // function (ADR-0036 D1), stamped in the pass that builds.
        const auto tool = b::make_prism(circle_chains(), 3.0f, +1,
                                        FeatureId{3});
        const auto tube = b::make_prism(ring_chains(), 1.5f, +1,
                                        FeatureId{5});
        const std::string fd = name_digest(*tool);
        const std::string ed = edge_name_digest(*tool);
        const std::string rfd = name_digest(*tube);
        const std::string red = edge_name_digest(*tube);
        if (i == 0) {
            circle_faces = fd;
            circle_edges = ed;
            ring_faces = rfd;
            ring_edges = red;
        } else {
            REQUIRE(fd == circle_faces);
            REQUIRE(ed == circle_edges);
            REQUIRE(rfd == ring_faces);
            REQUIRE(red == ring_edges);
        }
    }
    // The pinned goldens: the digest strings and their FNV-1a 64
    // hashes — bit-identical from the FIRST pinning (D5's
    // declaration — there is no prior name state to move).
    CHECK(circle_faces ==
          "0|ext#3:lat(0)\n"
          "1|ext#3:cap(lo)\n"
          "2|ext#3:cap(hi)\n");
    CHECK(fnv1a64(circle_faces) == 0x27c8a19f27563383ull);
    CHECK(circle_edges ==
          "0|\n"
          "1|\n"
          "2|\n"
          "3|\n"
          "4|ext#3:seam\n");
    CHECK(fnv1a64(circle_edges) == 0xeb1b2971bb96a75cull);

    CHECK(ring_faces ==
          "0|ext#5:c0:lat(0)\n"
          "1|ext#5:c0:lat(1)\n"
          "2|ext#5:c0:lat(2)\n"
          "3|ext#5:c0:lat(3)\n"
          "4|ext#5:c1:lat(0)\n"
          "5|ext#5:cap(lo)\n"
          "6|ext#5:cap(hi)\n");
    CHECK(fnv1a64(ring_faces) == 0x897bdf019ea333a3ull);
    CHECK(ring_edges ==
          "0|\n"
          "1|\n"
          "2|\n"
          "3|\n"
          "4|\n"
          "5|\n"
          "6|\n"
          "7|\n"
          "8|\n"
          "9|\n"
          "10|\n"
          "11|\n"
          "12|\n"
          "13|\n"
          "14|ext#5:c1:seam\n");
    CHECK(fnv1a64(ring_edges) == 0xc07812021d9f6c0cull);
}

TEST_CASE("H2-U6-M3 the feature part flows: a different creator is a "
          "different body, same roles; the default reads ext#0") {
    const auto a = b::make_prism(circle_chains(), 3.0f, +1, FeatureId{3});
    const auto c = b::make_prism(circle_chains(), 3.0f, +1, FeatureId{9});
    CHECK(b::query_face(*a, 0).name == "ext#3:lat(0)");
    CHECK(b::query_face(*c, 0).name == "ext#9:lat(0)");
    CHECK(name_digest(*a) != name_digest(*c));
    // The defaulted parameter serves the standalone callers (the
    // scene's hybrid materialization, the bridge): feature 0, the
    // roles identical.
    const auto d = b::make_prism(circle_chains(), 3.0f, +1);
    CHECK(b::query_face(*d, 0).name == "ext#0:lat(0)");
    CHECK(b::query_edge(*d, 4).name == "ext#0:seam");
}

// ===========================================================================
// FAMILY 3 — THE UNIQUENESS + TOTALITY
// ===========================================================================

TEST_CASE("H2-U6-M3 uniqueness: no two entities of a body share a "
          "name; totality: every face named, the boundary pinned") {
    const auto tool = b::make_prism(circle_chains(), 3.0f, +1,
                                    FeatureId{3});
    {
        std::set<std::string> names;
        for (b::FaceId i = 0; i < tool->face_count(); ++i) {
            const std::string name = b::query_face(*tool, i).name;
            REQUIRE_FALSE(name.empty());  // totality: every FACE named
            CHECK(names.insert(name).second);  // uniqueness
        }
        for (b::EdgeId i = 0; i < tool->edge_count(); ++i) {
            const std::string name = b::query_edge(*tool, i).name;
            if (!name.empty()) CHECK(names.insert(name).second);
        }
        CHECK(names.size() == 3 + 1);
    }

    const auto tube = b::make_prism(ring_chains(), 1.5f, +1,
                                    FeatureId{5});
    {
        std::set<std::string> names;
        for (b::FaceId i = 0; i < tube->face_count(); ++i) {
            const std::string name = b::query_face(*tube, i).name;
            REQUIRE_FALSE(name.empty());
            CHECK(names.insert(name).second);
        }
        for (b::EdgeId i = 0; i < tube->edge_count(); ++i) {
            const std::string name = b::query_edge(*tube, i).name;
            if (!name.empty()) CHECK(names.insert(name).second);
        }
        CHECK(names.size() == 7 + 1);
    }

    // THE HONEST BOUNDARY, pinned as the design it is: the walk's
    // OTHER edges — the contour's bottom/top curves and verticals,
    // the solid circle's rim halves, the hole's closed rims — are
    // OUTSIDE D1's v1 extrude vocabulary (the same queue discipline
    // as the vertex names, ADR-0036 D4). They read EMPTY, never a
    // guess; the extension is a table row away when a consumer
    // arrives.
    for (b::EdgeId i = 0; i < 4; ++i)
        CHECK(b::query_edge(*tool, i).name.empty());
    for (b::EdgeId i = 0; i < 14; ++i)
        CHECK(b::query_edge(*tube, i).name.empty());
}

// ===========================================================================
// FAMILY 4 — THE ROUND-TRIP ABI (structure -> name -> structure)
// ===========================================================================

TEST_CASE("H2-U6-M3 round-trip: every parametric form, idem and back, "
          "character for character") {
    // THE NO-PARSER BOUNDARY, restated where it bites: the round
    // trip's name -> SubshapeRef leg is the VOCABULARY's frozen
    // bijection (role_index_of — one word) plus the STRUCTURAL
    // components the caller holds (the group, the chain entity's
    // index — the same knowledge the construction stamped with);
    // the param never crosses back through the string, and morse
    // 2's compile-time probes (no from_string, no parse_name) still
    // guard the negative in this suite's own files.
    struct Case {
        const char* what;
        std::unique_ptr<b::BRepBody> body;
        bool multi_chain;
        n::RoleRegistry lateral_view;  // plain or grouped, by body
        n::RoleRegistry edge_view;     // the seam's own, same rule
        // The walk plan the construction itself held (the numbering
        // table's own layout): per face — a lateral's (group,
        // entity), or the cap's role row.
        struct FacePlan {
            bool cap;
            std::uint32_t group;
            std::uint32_t entity;
            std::uint32_t cap_role;
        };
        std::vector<FacePlan> plan;
        std::uint32_t seam_group;
    };
    // The views are stored copies (the registries are pure data); the
    // CAPS render under the PLAIN view in every body — the region's
    // own, never a chain's.
    const n::RoleRegistry plain_faces = n::kExtNaming.faces();
    const n::RoleRegistry plain_edges = n::kExtNaming.edges();
    const n::RoleRegistry group_faces = n::kExtNaming.grouped_faces();
    const n::RoleRegistry group_edges = n::kExtNaming.grouped_edges();

    std::vector<Case> cases;
    cases.push_back({"circle",
                     b::make_prism(circle_chains(), 3.0f, +1,
                                   FeatureId{3}),
                     false, plain_faces, plain_edges,
                     {{false, 0, 0, 0},
                      {true, 0, 0, n::kExtCapLoRole},
                      {true, 0, 0, n::kExtCapHiRole}},
                     /*seam_group=*/0});
    cases.push_back({"ring",
                     b::make_prism(ring_chains(), 1.5f, +1,
                                   FeatureId{5}),
                     true, group_faces, group_edges,
                     {{false, 0, 0, 0},
                      {false, 0, 1, 0},
                      {false, 0, 2, 0},
                      {false, 0, 3, 0},
                      {false, 1, 0, 0},
                      {true, 0, 0, n::kExtCapLoRole},
                      {true, 0, 0, n::kExtCapHiRole}},
                     /*seam_group=*/1});

    // The frozen bijection, read once: the tokens' own table rows.
    const std::uint32_t lat_idx =
        n::role_index_of(n::kExtFaceRoles, "lat");
    const std::uint32_t cap_lo_idx =
        n::role_index_of(n::kExtFaceRoles, "cap");
    REQUIRE(lat_idx == n::kExtLatRole);
    REQUIRE(cap_lo_idx == n::kExtCapLoRole);
    REQUIRE(n::role_index_of(n::kExtEdgeRoles, "seam") ==
            n::kExtSeamRole);

    for (const auto& c : cases) {
        REQUIRE(c.plan.size() == c.body->face_count());
        for (b::FaceId i = 0; i < c.body->face_count(); ++i) {
            const b::FaceInfo q = b::query_face(*c.body, i);
            REQUIRE_FALSE(q.name.empty());
            const Case::FacePlan& plan = c.plan[i];
            // The lateral renders under the body's lateral view, the
            // cap under the PLAIN one (feature-owned: no group, in
            // any body).
            const n::RoleRegistry& view =
                plan.cap ? plain_faces : c.lateral_view;
            // Leg 1: the stored structure renders the stored name —
            // twice, identically (the render is pure).
            const std::string name1 = n::render_name(
                q.id.feature, q.id.subshape, view);
            CHECK(name1 == q.name);
            CHECK(n::render_name(q.id.feature, q.id.subshape, view) ==
                  name1);
            // Leg 2: the token's bijection + the structural
            // components (the plan's own group and entity — the same
            // knowledge the construction stamped with) rebuild the
            // ref; the rebuilt ref EQUALS the stored one and renders
            // the identical name.
            const std::size_t colon = q.name.find(':');
            REQUIRE(colon != std::string::npos);
            SubshapeRef back;
            if (!plan.cap) {
                if (c.multi_chain) back = back.push(plan.group);
                back = back.push(lat_idx).push(plan.entity);
            } else {
                back = back.push(plan.cap_role);
            }
            CHECK(back == q.id.subshape);
            CHECK(n::render_name(q.id.feature, back, view) == name1);
        }
        // The seam edge (the only named edge class): the same legs,
        // under the body's edge view.
        {
            const b::EdgeId seam_id = c.multi_chain ? 14 : 4;
            const b::EdgeInfo q = b::query_edge(*c.body, seam_id);
            REQUIRE_FALSE(q.name.empty());
            const std::string name1 = n::render_name(
                q.id.feature, q.id.subshape, c.edge_view);
            CHECK(name1 == q.name);
            SubshapeRef back;
            if (c.multi_chain) back = back.push(c.seam_group);
            back = back.push(n::kExtSeamRole);
            CHECK(back == q.id.subshape);
            CHECK(n::render_name(q.id.feature, back, c.edge_view) ==
                  name1);
        }
    }
}

TEST_CASE("H2-U6-M3 the depth declaration: the extrusion case's "
          "maximum v1 depth is 3, kMaxDepth = 8 holds with room") {
    // THE MORSE-3 REGISTER's own declaration, pinned where it lands:
    // the deepest extrude name is the multi-chain lateral
    // [group, lat, i] — depth 3. Single-chain: lat 2, caps 1, seam
    // 1. Multi-chain: lat 3, caps 1 (feature-owned — no group), the
    // seam 2.
    const auto tool = b::make_prism(circle_chains(), 3.0f, +1,
                                    FeatureId{3});
    CHECK(b::query_face(*tool, 0).id.subshape.depth == 2);
    CHECK(b::query_face(*tool, 1).id.subshape.depth == 1);
    CHECK(b::query_face(*tool, 2).id.subshape.depth == 1);
    CHECK(b::query_edge(*tool, 4).id.subshape.depth == 1);

    const auto tube = b::make_prism(ring_chains(), 1.5f, +1,
                                    FeatureId{5});
    for (b::FaceId i = 0; i < 5; ++i)
        CHECK(b::query_face(*tube, i).id.subshape.depth == 3);
    CHECK(b::query_face(*tube, 5).id.subshape.depth == 1);
    CHECK(b::query_face(*tube, 6).id.subshape.depth == 1);
    CHECK(b::query_edge(*tube, 14).id.subshape.depth == 2);

    // The declaration itself: 3 < kMaxDepth, and the guide case's
    // subtract-names (depth 4, morse 2's pin) remain v1's deepest.
    CHECK(3 < SubshapeRef::kMaxDepth);
    CHECK(SubshapeRef::kMaxDepth == 8);
}

TEST_CASE("H2-U6-M3 the render is loud: the view misuses and the "
          "foreign paths refuse with diagnosis + remedy") {
    const FeatureId f{3};
    // A CAP under the GROUP view: the caps are feature-owned — the
    // group is not theirs to carry.
    SubshapeRef cap;
    cap = cap.push(n::kExtCapLoRole);
    CHECK_THROWS_AS(
        n::render_name(f, cap, n::kExtNaming.grouped_faces()),
        std::invalid_argument);

    // A grouped lateral's path under the FEATURE view: the group
    // component is left unconsumed — the views are disjoint.
    SubshapeRef grouped_lat;
    grouped_lat = grouped_lat.push(0).push(n::kExtLatRole).push(0);
    CHECK_THROWS_AS(
        n::render_name(f, grouped_lat, n::kExtNaming.faces()),
        std::invalid_argument);

    // A path that ends inside the group: the group consumed, the
    // role index missing.
    SubshapeRef group_only;
    group_only = group_only.push(0);
    CHECK_THROWS_AS(
        n::render_name(f, group_only, n::kExtNaming.grouped_faces()),
        std::invalid_argument);

    // The out-of-table role index and the empty path (morse 2's
    // own refusals, still loud under the new tables).
    SubshapeRef foreign;
    foreign = foreign.push(99);
    CHECK_THROWS_AS(
        n::render_name(f, foreign, n::kExtNaming.faces()),
        std::invalid_argument);
    SubshapeRef empty;
    CHECK_THROWS_AS(
        n::render_name(f, empty, n::kExtNaming.faces()),
        std::invalid_argument);
    CHECK_THROWS_AS(
        n::role_index_of(n::kExtEdgeRoles, "lat"),
        std::invalid_argument);
}

// ===========================================================================
// FAMILY 5 — THE TAVOLA + THE FROM CLOSURE (the born-of genealogy)
// ===========================================================================

TEST_CASE("H2-U6-M3 the guide case's tavola: COUNTED and pinned, "
          "row by row — 12 born, the FROMs the roots") {
    const auto chains = circle_chains();
    const auto tool = b::make_prism(chains, 3.0f, +1, FeatureId{3});
    const b::DescentTable t = b::extrude_descent(
        *tool, chains, FeatureId{3}, FeatureId{2});

    // D2's shape for the guide case's tool: 4V + 5E + 3F = 12 born
    // rows, zero vanished (the sketch's entities do not die in the
    // extrude — they are the parents), every row with its FROM.
    REQUIRE(t.rows.size() == 12);
    CHECK(t.count(b::Descent::Born) == 12);
    CHECK(t.count(b::Descent::Kept) == 0);
    CHECK(t.count(b::Descent::Vanished) == 0);

    // The rows, in the body's own order (vertices, edges, faces),
    // each born of the authored root: 10 x sk#2:0 (the circle's one
    // entity owns the wall, the rims, the seam and the four
    // vertices), 2 x sk#2:face (the caps — the extruded region).
    const n::RootRef e0{FeatureId{2}, n::RootKind::Entity, 0, 0};
    const n::RootRef face{FeatureId{2}, n::RootKind::Face, 0, 0};
    std::size_t vertex_rows = 0, edge_rows = 0, face_rows = 0;
    std::size_t from_e0 = 0, from_face = 0;
    for (const auto& row : t.rows) {
        CHECK(row.relation == b::Descent::Born);
        REQUIRE(row.from.size() == 1);
        const n::RootRef& root = row.from[0];
        if (root == e0) ++from_e0;
        if (root == face) ++from_face;
        switch (row.entity) {
        case b::EntityKind::Vertex:
            ++vertex_rows;
            CHECK(row.name.empty());  // v1's vertex queue (D4)
            CHECK(row.address < 4);
            break;
        case b::EntityKind::Edge:
            ++edge_rows;
            if (row.address == 4)
                CHECK(row.name == "ext#3:seam");
            else
                CHECK(row.name.empty());
            break;
        case b::EntityKind::Face:
            ++face_rows;
            break;
        }
    }
    CHECK(vertex_rows == 4);
    CHECK(edge_rows == 5);
    CHECK(face_rows == 3);
    CHECK(from_e0 == 10);
    CHECK(from_face == 2);
    // The named rows, verbatim.
    for (const auto& row : t.rows)
        if (row.entity == b::EntityKind::Face && row.address == 0)
            CHECK(row.name == "ext#3:lat(0)");

    // Both closure sides green: the result side (every entity in
    // exactly one row) and the FROM side (every root live upstream).
    std::string why;
    CHECK(b::descent_closed(*tool, t, &why));
    CHECK(b::extrude_froms_resolve(t, chains, &why));

    // The derived state's own purity: derived twice, identical.
    const b::DescentTable t2 = b::extrude_descent(
        *tool, chains, FeatureId{3}, FeatureId{2});
    REQUIRE(t2.rows.size() == t.rows.size());
    for (std::size_t i = 0; i < t.rows.size(); ++i) {
        CHECK(t2.rows[i].relation == t.rows[i].relation);
        CHECK(t2.rows[i].entity == t.rows[i].entity);
        CHECK(t2.rows[i].address == t.rows[i].address);
        CHECK(t2.rows[i].name == t.rows[i].name);
        CHECK(t2.rows[i].from == t.rows[i].from);
    }
}

TEST_CASE("H2-U6-M3 the ring's tavola: counted — 32 born rows, the "
          "roots the sketch's own layout") {
    const auto chains = ring_chains();
    const auto tube = b::make_prism(chains, 1.5f, +1, FeatureId{5});
    const b::DescentTable t = b::extrude_descent(
        *tube, chains, FeatureId{5}, FeatureId{6});

    // 10V + 15E + 7F = 32 born rows. The FROMs: the outer's four
    // entities own 6 rows each (2 joints + 3 curves + 1 lateral),
    // the hole's one entity owns 6 (2 + 3 + 1), the caps the face.
    REQUIRE(t.rows.size() == 32);
    CHECK(t.count(b::Descent::Born) == 32);

    std::size_t from[2][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}};
    std::size_t from_face = 0;
    for (const auto& row : t.rows) {
        REQUIRE(row.from.size() == 1);
        const n::RootRef& root = row.from[0];
        if (root.kind == n::RootKind::Face) {
            ++from_face;
            continue;
        }
        REQUIRE(root.kind == n::RootKind::Entity);
        REQUIRE(root.chain < 2);
        REQUIRE(root.entity < 4);
        ++from[root.chain][root.entity];
    }
    for (std::size_t i = 0; i < 4; ++i)
        CHECK(from[0][i] == 6);  // the outer's entity i: 2V+3E+1F
    CHECK(from[1][0] == 6);      // the hole's entity: 2V+3E+1F
    CHECK(from_face == 2);       // the caps

    // The root renders, the register's multi-chain forms: the
    // outer's entities are sk#6:c0:0..3, the hole's sk#6:c1:0, the
    // caps sk#6:face — the SKETCH's own chain positions (chain 0 =
    // the rect, chain 1 = the hole), which the construction groups
    // mirror here (the outer is the sketch's chain 0).
    for (const auto& row : t.rows) {
        const n::RootRef& root = row.from[0];
        if (root.kind == n::RootKind::Face)
            CHECK(n::render_root(root, true) == "sk#6:face");
        else if (root.chain == 0)
            CHECK(n::render_root(root, true) ==
                  "sk#6:c0:" + std::to_string(root.entity));
        else
            CHECK(n::render_root(root, true) == "sk#6:c1:0");
    }

    std::string why;
    CHECK(b::descent_closed(*tube, t, &why));
    CHECK(b::extrude_froms_resolve(t, chains, &why));
}

TEST_CASE("H2-U6-M3 the drift-guard: a tavola derived with the wrong "
          "feature refuses loudly, naming the entity") {
    const auto chains = circle_chains();
    const auto tool = b::make_prism(chains, 3.0f, +1, FeatureId{3});
    // The wrong creator id: the derived names render ext#9 where the
    // body stamps ext#3 — the tavola would lie, and refuses to ship.
    CHECK_THROWS_AS(
        b::extrude_descent(*tool, chains, FeatureId{9}, FeatureId{2}),
        std::logic_error);
    // The right one derives.
    CHECK_NOTHROW(
        b::extrude_descent(*tool, chains, FeatureId{3}, FeatureId{2}));
}

TEST_CASE("H2-U6-M3 the FROM closure's naive counterproof: a dangling "
          "root fails loudly, named") {
    const auto chains = circle_chains();
    const auto tool = b::make_prism(chains, 3.0f, +1, FeatureId{3});
    const b::DescentTable good = b::extrude_descent(
        *tool, chains, FeatureId{3}, FeatureId{2});
    std::string why;
    CHECK(b::extrude_froms_resolve(good, chains, &why));

    // A hand-broken FROM: a root at entity 99 of a one-entity chain
    // — the suspended reference, caught and named.
    b::DescentTable broken = good;
    broken.rows[0].from[0] = {FeatureId{2}, n::RootKind::Entity, 0, 99};
    CHECK_FALSE(b::extrude_froms_resolve(broken, chains, &why));
    CHECK(why.find("DANGLES") != std::string::npos);
    CHECK(why.find("sk#2:99") != std::string::npos);
    CHECK(why.find("remedy") != std::string::npos);

    // A chain that does not exist, same refusal — the structural
    // coordinates named beside the render (the render elides the
    // chain on a single-chain sketch, the coordinates never do).
    b::DescentTable broken_chain = good;
    broken_chain.rows[0].from[0] = {FeatureId{2},
                                    n::RootKind::Entity, 7, 0};
    CHECK_FALSE(b::extrude_froms_resolve(broken_chain, chains, &why));
    CHECK(why.find("chain 7") != std::string::npos);
    CHECK(why.find("remedy") != std::string::npos);
}

// ===========================================================================
// FAMILY 6 — THE ADDRESS PLAN (additivity, mirrored from unit 3)
// ===========================================================================

TEST_CASE("H2-U6-M3 the address plan intact: the extrude's answers "
          "read exactly what unit 3 pinned") {
    // The circle: 4V/5E/3F, chi = 2 (the seam counts VERBATIM), the
    // wall a cylinder, the caps planes, everything exact/0.
    const auto tool = b::make_prism(circle_chains(), 3.0f, +1,
                                     FeatureId{3});
    const b::BodySummary sum = b::query_body(*tool);
    CHECK(sum.vertex_count == 4);
    CHECK(sum.edge_count == 5);
    CHECK(sum.face_count == 3);
    CHECK(sum.inner_loops == 0);
    CHECK(sum.chi == 2);
    CHECK(sum.genus_derived);
    CHECK(sum.genus == 0);
    CHECK(b::query_face(*tool, 0).family == "cylinder");
    CHECK(b::query_face(*tool, 1).family == "plane");
    CHECK(b::query_face(*tool, 2).family == "plane");
    for (b::FaceId i = 0; i < 3; ++i) {
        const b::FaceInfo q = b::query_face(*tool, i);
        CHECK(q.loop_count == 1);
        CHECK(q.exact);
        CHECK(q.linear_tolerance == 0.0);
    }

    // The ring: 10V/15E/7F, H = 2, chi = 0 (genus 1) — the
    // multi-chain counts, and the caps' two loops (outer + hole).
    const auto tube = b::make_prism(ring_chains(), 1.5f, +1,
                                    FeatureId{5});
    const b::BodySummary rsum = b::query_body(*tube);
    CHECK(rsum.vertex_count == 10);
    CHECK(rsum.edge_count == 15);
    CHECK(rsum.face_count == 7);
    CHECK(rsum.inner_loops == 2);
    CHECK(rsum.chi == 0);
    CHECK(rsum.genus_derived);
    CHECK(rsum.genus == 1);
    for (b::FaceId i = 0; i < 5; ++i) {
        const b::FaceInfo q = b::query_face(*tube, i);
        CHECK(q.loop_count == 1);
        CHECK(q.exact);
    }
    for (const b::FaceId cap : {b::FaceId{5}, b::FaceId{6}}) {
        const b::FaceInfo q = b::query_face(*tube, cap);
        CHECK(q.family == "plane");
        CHECK(q.loop_count == 2);
        CHECK(q.outer_loop_count == 1);
        CHECK(q.inner_loop_count == 1);
        CHECK(q.exact);
    }
    // The edge query's own address answers: the seam is manifold
    // (the radial contract — the seam pair lives in ONE loop).
    const b::EdgeInfo seam = b::query_edge(*tool, 4);
    CHECK(seam.manifold);
    CHECK(seam.coedge_count == 2);
    CHECK(seam.exact);
}
