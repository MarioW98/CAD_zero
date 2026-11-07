// tests/unit/brep/test_g2_u3_m6ter_the_chain.cpp
//
// G2-u3 m6-correttiva-ter — LA CATENA PARAMETRICA AL KERNEL: the
// third click-test's defect B named at the kernel level ("come se
// avessi 2 cubi uno dentro l'altro" — the second fillet consumed the
// pristine box and generated a sibling). The cure this file pins:
// fillet_edge's admission WIDENED from the factory box's frozen
// 8/12/6 to the BOX FAMILY CHAIN — a box with k previous vertical
// fillets, V = 8+2k / E = 12+3k / F = 6+k, the faces the box's six
// planes (possibly shrunk) plus k CylinderZ flanks, every entity
// exact, every face single-loop, chi = 2, closed orientable. The
// second fillet CONSUMES THE CURRENT STATE (the first fillet's
// body); the reference that died refuses by name (the structural
// find's own wall); the tangent wires of a previous fillet refuse by
// family (a plane x cylinder adjacency — the rim class's world);
// the typed band rides the chain (the shared face's reach TIGHTENED
// by the previous fillet's tangent line).
//
// THE CHAIN GOLDEN: box [-1,1]^3 -> fil#2 (box#1:vertical2, the
// (+X,+Y) corner, r = 0.5) -> fil#3 (the KEPT STAMP of
// box#1:vertical0 — the (-X,-Y) corner, r = 0.5, the opposite
// vertical: the wedges disjoint). The counts by the same walk one
// link over:
//
//   V = 10 - 2 + 4 = 12   E = 15 - 1 + 4 = 18   F = 7 + 1 = 8
//   H = 0                 chi = 12 - 18 + 8 = 2 (Delta-chi = 0)
//
// The volume: 8 - 2 * (2 * 0.25 * (1 - pi/4)) = 7 + pi/4 — two
// disjoint wedges, the closed form per fillet l*r^2*(1-pi/4).
//
// THE COUNTERPROOFS (the naive red, per component):
//   * the frozen-signature naive (the un-widened admission) refuses
//     the chain body "outside the class signature" — the sibling's
//     birthplace; the widened admission is the discrimination;
//   * the kept-flank naive (the kept-face copy that forces a plane)
//     cannot carry the previous fillet's CylinderZ flank — the copy
//     reads it VERBATIM (the body copy's own sharing pattern);
//   * the band naive (the box's own reach 2.0 from the far corner)
//     would admit r = 1.5 on the shared face; the chain's reach is
//     the previous tangent line's 1.5 — the typed band refuses,
//     the bound NAMED (1.500000).
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/fillet.hpp"
#include "CAD_0/brep/mass_properties.hpp"
#include "CAD_0/brep/naming.hpp"
#include "CAD_0/brep/descent.hpp"
#include "CAD_0/brep/query.hpp"
#include "CAD_0/brep/topology.hpp"
#include "CAD_0/geometry/shape_id.hpp"

#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <doctest/doctest.h>

namespace {

namespace b = CAD_0::brep;
namespace n = CAD_0::brep::naming;
using CAD_0::geometry::FeatureId;
using CAD_0::geometry::ShapeId;
using CAD_0::geometry::SubshapeRef;
using Vec3d = CAD_0::math::Vec3d;

constexpr double kPi = 3.14159265358979323846;

bool close(double a, double b, double eps = 1e-9) {
    return std::abs(a - b) <= eps;
}

// The chain's construction: box#1 -> fil#2 (vertical2, the (+X,+Y)
// corner) -> fil#3 (the kept stamp of vertical0, the (-X,-Y) corner
// — the reference SURVIVES the first fillet as a stamp copy, the
// ADR-0036 D3 MATCH class through the chain). The naming contexts:
// the ROOT's table (the box's own) both times — the adjacent faces'
// stamps are the box's.
struct ChainGuide {
    std::unique_ptr<b::BRepBody> box;
    std::unique_ptr<b::BRepBody> first;
    std::unique_ptr<b::BRepBody> result;
    ShapeId surviving{};   // box#1:vertical0 (edge 8), kept by fil#2
    ShapeId dead{};        // box#1:vertical2 (edge 10), dead in fil#2
    ShapeId first_wire{};  // fil#2's tangent wire (a kept stamp)

    explicit ChainGuide(double r2 = 0.5, double r3 = 0.5,
                        bool third = true) {
        box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        const ShapeId vertical2{FeatureId{1},
                                SubshapeRef{}.push(10), {}};
        dead = vertical2;
        surviving = ShapeId{FeatureId{1}, SubshapeRef{}.push(8), {}};
        const b::FilletNaming ctx2{FeatureId{2}, &n::kBoxNaming};
        first = b::fillet_edge(*box, vertical2, r2, ctx2);
        // The first fillet's tangent wire (edge 11 of the result:
        // e_tan_A, the back face's landing) — referenced structurally
        // below for the family wall.
        first_wire = first->edges[11].shape_id();
        if (!third) return;
        const b::FilletNaming ctx3{FeatureId{3}, &n::kBoxNaming};
        result = b::fillet_edge(*first, surviving, r3, ctx3);
    }
};

} // namespace

// ===========================================================================
// THE CHAIN GOLDEN — every number, the same walk one link over
// ===========================================================================
TEST_CASE("G2-U3 m6ter the chain golden: the second fillet consumes "
          "the current state — 12/18/8/H=0/chi=2, the volume 7+pi/4 "
          "all exact, the census closes, the stamps survive") {
    const ChainGuide g;
    REQUIRE(g.first != nullptr);
    REQUIRE(g.result != nullptr);

    // THE COUNTS — the walk's own arithmetic one link over.
    REQUIRE(g.result->vertex_count() == 12);
    REQUIRE(g.result->edge_count() == 18);
    REQUIRE(g.result->face_count() == 8);
    REQUIRE(b::inner_loop_count(*g.result) == 0);
    REQUIRE(b::euler_characteristic(*g.result) == 2);
    CHECK(b::euler_characteristic(*g.result) ==
          b::euler_characteristic(*g.box));
    CHECK(b::is_closed_orientable(*g.result));
    CHECK(b::loops_are_closed(*g.result));

    // THE CENSUS: every edge exactly two OPPOSITE co-edges. The
    // per-face row: bottom 6 + top 6 (the two quarter arcs each),
    // the laterals 4 each (the tangent line REPLACES the dead
    // co-edge — the m2 arithmetic holds per link), flank#1 4,
    // flank#2 4 — 36 incidences / 2 = 18.
    {
        std::size_t total = 0;
        for (const auto& e : g.result->edges) {
            REQUIRE(e.coedges().size() == 2);
            CHECK(e.coedges()[0]->reversed() !=
                  e.coedges()[1]->reversed());
        }
        for (const auto& f : g.result->solid.shell().faces())
            for (const auto& loop : f.loops())
                total += loop.coedges().size();
        REQUIRE(total == 36);
        const std::size_t per_face[8] = {6, 6, 4, 4, 4, 4, 4, 4};
        for (std::size_t i = 0; i < 8; ++i) {
            CAPTURE(i);
            REQUIRE(g.result->solid.shell().faces()[i]
                        .loops()[0]
                        .coedges()
                        .size() == per_face[i]);
        }
    }

    // THE VOLUME — two disjoint wedges, the closed form per fillet:
    // 8 - 2 * (2 * r^2 * (1 - pi/4)) at r = 0.5 = 7 + pi/4.
    {
        const auto mp = b::mass_properties(
            *g.result, CAD_0::math::Tolerance::linear(1e-9));
        const double wedge = 0.25 * (1.0 - kPi / 4.0);
        CHECK(close(mp.volume, 7.0 + kPi / 4.0));
        CHECK(close(8.0 - mp.volume, 2.0 * 2.0 * wedge, 1e-12));
        CHECK(mp.all_exact);
        // THE NAIVE READS, red by discrimination: one wedge alone
        // (the sibling's volume, 7.5 + pi/8 — what consuming the
        // PRISTINE box would have produced) and the square-cut
        // chain (8 - 4*0.25) are NOT the answer.
        CHECK_FALSE(close(mp.volume, 7.5 + kPi / 8.0));
        CHECK_FALSE(close(mp.volume, 8.0 - 4.0 * 0.25));
    }

    // THE NAMES — the second flank born under fil#3 with the ROOT's
    // vocabulary (the adjacent faces' stamps are the box's), the
    // first fillet's born names kept verbatim, the box's stamps
    // surviving the second link.
    CHECK(b::query_face(*g.result, 6).name ==
          "fil#2:flank[box#1:back × box#1:right]");
    CHECK(b::query_face(*g.result, 7).name ==
          "fil#3:flank[box#1:front × box#1:left]");
    // The second fillet's tangent wires (edges 14/15: e_tan_A the
    // front's landing, e_tan_B the left's).
    CHECK(b::query_edge(*g.result, 14).name ==
          "fil#3:isect[box#1:front × fil#3:flank[box#1:front × "
          "box#1:left]]");
    CHECK(b::query_edge(*g.result, 15).name ==
          "fil#3:isect[box#1:left × fil#3:flank[box#1:front × "
          "box#1:left]]");
    // The first fillet's tangent wires keep their names (kept rows).
    bool found_first_wire = false;
    for (b::EdgeId i = 0; i < g.result->edge_count(); ++i)
        if (b::query_edge(*g.result, i).name ==
            "fil#2:isect[box#1:back × fil#2:flank[box#1:back × "
            "box#1:right]]")
            found_first_wire = true;
    CHECK(found_first_wire);
    // The surviving stamps: vertical1/vertical3 (the untouched
    // verticals) and the ring edges keep the box's names through
    // BOTH links (the stamp copy, ADR-0036 D3's MATCH).
    bool v1 = false, v3 = false;
    for (b::EdgeId i = 0; i < g.result->edge_count(); ++i) {
        const std::string nm = b::query_edge(*g.result, i).name;
        if (nm == "box#1:vertical1") v1 = true;
        if (nm == "box#1:vertical3") v3 = true;
    }
    CHECK(v1);
    CHECK(v3);
    CHECK(b::query_face(*g.result, 0).name == "box#1:bottom");
    CHECK(b::query_face(*g.result, 5).name == "box#1:right");

    // The kept flank's surface: the FIRST fillet's CylinderZ rides
    // through the kept-face copy VERBATIM (the family check below
    // pins it is still a wall, not a plane).
    {
        CAD_0::brep::fillet_detail::WallReader wr;
        g.result->solid.shell().faces()[6].surface()->accept(wr);
        REQUIRE(wr.wall != nullptr);
        CHECK(close(wr.wall->radius(), 0.5, 1e-12));
        CHECK(close(wr.wall->axis_center().x, 0.5, 1e-12));
        CHECK(close(wr.wall->axis_center().y, 0.5, 1e-12));
        CAD_0::brep::fillet_detail::WallReader wr2;
        g.result->solid.shell().faces()[7].surface()->accept(wr2);
        REQUIRE(wr2.wall != nullptr);
        CHECK(close(wr2.wall->radius(), 0.5, 1e-12));
        CHECK(close(wr2.wall->axis_center().x, -0.5, 1e-12));
        CHECK(close(wr2.wall->axis_center().y, -0.5, 1e-12));
    }

    // THE NUMBERING — the survivors compact, the born append, one
    // link over: the dead vertical0 (old 8) dropped, the second
    // fillet's four born edges appended at 14..17.
    {
        // e_tan_A (14): the front's tangent, bottom -> top; e_tan_B
        // (15): the left's; the arcs 16/17 close the flank's walk.
        REQUIRE(g.result->edges[14].vertex(0) + 1 ==
                g.result->edges[14].vertex(1));
        REQUIRE(g.result->edges[15].vertex(0) + 1 ==
                g.result->edges[15].vertex(1));
        CHECK(g.result->edges[16].vertex(1) ==
              g.result->edges[14].vertex(0));
        CHECK(g.result->edges[17].vertex(1) ==
              g.result->edges[14].vertex(1));
    }

    // THE DIGEST'S PURITY: the same chain built twice is
    // bit-identical (ADR-0014's discipline, the edit contract's
    // own floor).
    {
        const ChainGuide again;
        REQUIRE(again.result != nullptr);
        CHECK(g.result->vertex_count() ==
              again.result->vertex_count());
        for (std::size_t i = 0; i < g.result->vertices.size(); ++i) {
            CHECK(g.result->vertices[i].position().x ==
                  again.result->vertices[i].position().x);
            CHECK(g.result->vertices[i].position().y ==
                  again.result->vertices[i].position().y);
            CHECK(g.result->vertices[i].position().z ==
                  again.result->vertices[i].position().z);
        }
        for (b::EdgeId i = 0; i < g.result->edge_count(); ++i)
            CHECK(b::query_edge(*g.result, i).name ==
                  b::query_edge(*again.result, i).name);
    }
}

// ===========================================================================
// THE WALLS — the chain's own refusals, each by name
// ===========================================================================
TEST_CASE("G2-U3 m6ter the chain walls: the dead reference, the "
          "curved family's tangent wire, the tightened band") {
    const ChainGuide g(0.5, 0.5, /*third=*/false);
    REQUIRE(g.first != nullptr);

    // THE DEAD REFERENCE (the VANISHED class at the kernel): the
    // first fillet consumed vertical2 — the structural find against
    // the CURRENT state's stamps answers nothing, the refusal names
    // the closure violation (the scene layers the killer's name on
    // top; the kernel speaks the structural truth).
    bool threw_dead = false;
    try {
        const b::FilletNaming ctx{FeatureId{3}, &n::kBoxNaming};
        auto r = b::fillet_edge(*g.first, g.dead, 0.25, ctx);
        (void)r;
    } catch (const std::invalid_argument& e) {
        threw_dead = true;
        const std::string what = e.what();
        CHECK(what.find("names no edge of this body") !=
              std::string::npos);
    }
    REQUIRE(threw_dead);

    // THE CURVED FAMILY'S WALL: the first fillet's tangent wire is
    // VERTICAL (its endpoints share (x, y)) — the verticality gate
    // passes — but its adjacent faces are a plane and the CYLINDER
    // flank: the admission (a) refuses by name, the rim class's
    // world pointed to.
    bool threw_family = false;
    try {
        const b::FilletNaming ctx{FeatureId{3}, &n::kBoxNaming};
        auto r = b::fillet_edge(*g.first, g.first_wire, 0.25, ctx);
        (void)r;
    } catch (const std::invalid_argument& e) {
        threw_family = true;
        const std::string what = e.what();
        CHECK(what.find("an adjacent face is non-planar") !=
              std::string::npos);
    }
    REQUIRE(threw_family);

    // THE TIGHTENED BAND: the second fillet on vertical1 — the
    // ADJACENT vertical (the (+X,-Y) corner, sharing the RIGHT face
    // with the first fillet's vertical2). The right face's inward
    // reach from vertical1 is the previous tangent line's 1.5 (the
    // box's own would be 2.0): r = 1.5 leaves the band, the bound
    // NAMED in the refusal; r = 0.5 lands (the chain golden's
    // sibling case).
    {
        const ShapeId vertical1{FeatureId{1},
                                SubshapeRef{}.push(9), {}};
        bool threw_band = false;
        try {
            const b::FilletNaming ctx{FeatureId{3},
                                      &n::kBoxNaming};
            auto r = b::fillet_edge(*g.first, vertical1, 1.5, ctx);
            (void)r;
        } catch (const std::invalid_argument& e) {
            threw_band = true;
            const std::string what = e.what();
            CHECK(what.find("fillet_radius_exceeds_face") !=
                  std::string::npos);
            // THE CHAIN'S OWN BOUND: the reach read from the shared
            // face's CURRENT loop — 1.5, the previous tangent's
            // retreat, NOT the box's 2.0.
            CHECK(what.find("1.500000") != std::string::npos);
        }
        REQUIRE(threw_band);
        // In-band on the same shared face: r = 0.5 lands — the
        // adjacent-vertical chain (the wedges at neighboring
        // corners, the shared face shrunk twice).
        const b::FilletNaming ctx{FeatureId{3}, &n::kBoxNaming};
        auto adj = b::fillet_edge(*g.first, vertical1, 0.5, ctx);
        REQUIRE(adj != nullptr);
        REQUIRE(adj->vertex_count() == 12);
        REQUIRE(adj->edge_count() == 18);
        REQUIRE(adj->face_count() == 8);
        CHECK(b::euler_characteristic(*adj) == 2);
        const auto mp = b::mass_properties(
            *adj, CAD_0::math::Tolerance::linear(1e-9));
        // Two disjoint wedges at neighboring corners: the same
        // closed form (the wedges do not overlap at these radii).
        CHECK(close(mp.volume, 7.0 + kPi / 4.0));
        CHECK(mp.all_exact);
    }
}

// ===========================================================================
// THE TAVOLA — the chain pair's descent, the closure both sides
// ===========================================================================
TEST_CASE("G2-U3 m6ter the chain tavola: the descent of the second "
          "link — the kept flank, the absorbed quartet, the born "
          "four, the vanished vertical named with its cause") {
    const ChainGuide g;
    REQUIRE(g.result != nullptr);
    const b::DescentTable tav =
        b::fillet_descent(*g.result, *g.first);

    // The row census by relation: faces out 8 (3 kept + 4 absorbed
    // + 1 born: back, right and the FIRST FLANK kept — the flank is
    // a kept row now, the chain's own novelty), edges out 18 (10
    // kept + 4 absorbed + 4 born), vertices out 12 (8 kept + 4
    // born); the vanished: 1 edge (vertical0) + 2 vertices (its
    // corners).
    std::size_t faces[6] = {0, 0, 0, 0, 0, 0};
    std::size_t edges[6] = {0, 0, 0, 0, 0, 0};
    std::size_t verts[6] = {0, 0, 0, 0, 0, 0};
    for (const auto& row : tav.rows) {
        std::size_t* book = row.entity == b::EntityKind::Face
                                ? faces
                                : (row.entity == b::EntityKind::Edge
                                       ? edges
                                       : verts);
        book[static_cast<std::size_t>(row.relation)] += 1;
    }
    CHECK(faces[static_cast<std::size_t>(b::Descent::Kept)] == 3);
    CHECK(faces[static_cast<std::size_t>(b::Descent::Absorbed)] == 4);
    CHECK(faces[static_cast<std::size_t>(b::Descent::Born)] == 1);
    CHECK(faces[static_cast<std::size_t>(b::Descent::Vanished)] == 0);
    CHECK(edges[static_cast<std::size_t>(b::Descent::Kept)] == 10);
    CHECK(edges[static_cast<std::size_t>(b::Descent::Absorbed)] == 4);
    CHECK(edges[static_cast<std::size_t>(b::Descent::Born)] == 4);
    CHECK(edges[static_cast<std::size_t>(b::Descent::Vanished)] == 1);
    CHECK(verts[static_cast<std::size_t>(b::Descent::Kept)] == 8);
    CHECK(verts[static_cast<std::size_t>(b::Descent::Born)] == 4);
    CHECK(verts[static_cast<std::size_t>(b::Descent::Vanished)] == 2);

    // THE VANISHED ROW'S CAUSE — the death IS the feature's act:
    // "replaced by the flank" (the m2 vocabulary, one link over).
    bool found_cause = false;
    for (const auto& row : tav.rows) {
        if (row.relation == b::Descent::Vanished &&
            row.entity == b::EntityKind::Edge) {
            CHECK(row.name == "box#1:vertical0");
            CHECK(row.cause.find("flank") != std::string::npos);
            found_cause = true;
        }
    }
    REQUIRE(found_cause);

    // THE KEPT FLANK'S ROW: the first fillet's flank survives the
    // second link as a KEPT face row under its own name.
    bool kept_flank = false;
    for (const auto& row : tav.rows) {
        if (row.relation == b::Descent::Kept &&
            row.entity == b::EntityKind::Face &&
            row.name == "fil#2:flank[box#1:back × box#1:right]") {
            kept_flank = true;
        }
    }
    CHECK(kept_flank);
}
