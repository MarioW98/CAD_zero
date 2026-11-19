// tests/unit/scene/test_g6_m5_recipe.cpp
//
// G6 m5 — LA RICETTA E LA PORTA V0.9.0, THE CHAMFER'S MACHINE
// (ADR-0045 D7.1/D7.2/D8.5's morse: "il ChamferRecipe, la forma
// multi, il rito del bump con la companion e il migrator, i golden
// del round-trip"): the recipe driven through the SCENE — the
// derived chain REPLAYED (D4.1: the vertical passage first, each
// entry its own walk, then THE RING AND MITER ACT, indivisible),
// the machine's own laws verified.
//
// THE SETBACK LAW BEFORE THE KERNEL (D5.4/D4.1, "the connected
// edge-corner graph forces one setback per component"): the doors'
// own — a map whose entries disagree at a shared corner is refused
// with chamfer_corner_setback_mismatch at CREATION, and the refusal
// is the MACHINE's own words (the "attach_chamfer_multi:"
// provenance), the kernel NEVER asked — the message's own prefix is
// the witness, and the test pins it. The opposite pinned too: the
// twelve at d = 1.0 is the KERNEL's question (the values agree, the
// law holds) — the refusal's provenance is the kernel walk's own
// prefix, the two questions never confused.
//
// THE CONTEGGI (the machine's own arithmetic, the defect detector):
// the single's landing state 8+2k/12+3k/6+k at k0+1, the vertical
// passage's states at every step (k0 the operand's own — the chain
// completion case), the act's signature 16/24/10 before the call,
// the verdict 32/48/18 with chi = 2 after.
//
// THE GOLDEN OF THE TWELVE (D4.3/D6.3, one road over): the full
// cube at d = 0.5 reads V = 8 - 12d^2 + 6d^3 = 5.75 — the machine's
// body through the kernel's divergence theorem against the test's
// own closed form, 1e-12. THE PARTIALS: the verticals at MIXED
// setbacks free among themselves (no two verticals share a corner),
// the additive prism formula 8 - sum_i d_i^2 at 1e-12. THE SINGLE
// RING ENTRY: the m3 corner act through the widened dispatch, the
// counts 14/21/9 from the k=1 chain, V = 8 - 3d^2 + (3/4)d^3.
//
// THE DETERMINISM (ADR-0014): the same recipe, re-created, the same
// digest — bit-identical; the restore road (the pinned id) answers
// the same body. THE REGENERATION: the DAG pass re-run ten times,
// the digest never moving. THE ORDER PERMUTATIONS: the same logical
// map in a different authored order — the same geometry, the same
// twelve bevel names, both digests deterministic per recipe.
#include <doctest/doctest.h>

#include "CAD_0/brep/chain_gate.hpp"
#include "CAD_0/brep/chamfer.hpp"
#include "CAD_0/brep/descent.hpp"
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/mass_properties.hpp"
#include "CAD_0/brep/naming.hpp"
#include "CAD_0/brep/query.hpp"
#include "CAD_0/brep/topology.hpp"
#include "CAD_0/scene/operations.hpp"
#include "CAD_0/scene/scene.hpp"
#include "CAD_0/sdf/primitives.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace b = CAD_0::brep;
namespace scene = CAD_0::scene;
namespace math = CAD_0::math;

namespace {

constexpr double kD = 0.5;

// The twelve-edge closed form (the test's own arithmetic, zero code
// shared with the kernel — the M4 oracle's own equation).
double twelve_volume(double d) {
    return 8.0 - 12.0 * d * d + 6.0 * d * d * d;
}

// The single vertical's prism formula (the m1 golden's own).
double single_volume(double d) { return 8.0 - d * d; }

// The partials' additive prism formula: 8 - sum d_i^2.
double partial_volume(const std::vector<double>& setbacks) {
    double v = 8.0;
    for (const double d : setbacks) v -= d * d;
    return v;
}

// The m3 corner act's form from the k=1 chain.
double corner_volume(double d) {
    return 8.0 - 3.0 * d * d + 0.75 * d * d * d;
}

// The scene's guide: a fitted box (the bridge's feature-aware fit,
// node id 1 — the names box#1:*).
struct GuideScene {
    scene::Scene s;
    std::uint32_t box_id{0};
    GuideScene() {
        box_id = s.add_node(
            "Box", CAD_0::sdf::make_box(
                       math::Vec3f{1.0f, 1.0f, 1.0f}));
    }
};

// The twelve entries of the cube at one setback, in the roles' own
// order (bottom ring, top ring, verticals — kBoxEdgeRoles).
std::vector<std::pair<std::string, double>> twelve(double d) {
    std::vector<std::pair<std::string, double>> out;
    for (const char* tok :
         {"bottom0", "bottom1", "bottom2", "bottom3",
          "top0", "top1", "top2", "top3",
          "vertical0", "vertical1", "vertical2", "vertical3"})
        out.emplace_back(std::string("box#1:") + tok, d);
    return out;
}

b::MultiChamferRecipe map_of(
    const std::vector<std::pair<std::string, double>>& entries) {
    b::MultiChamferRecipe r;
    for (const auto& [ref, setback] : entries)
        r.entries.push_back(b::MultiChamferEntry{ref, setback});
    return r;
}

// The body digest (the chain_gate's own — the same witness the
// landed chain tests use).
std::string body_digest(const b::BRepBody& body) {
    return b::chain_digest(body);
}

}  // namespace

// ---------------------------------------------------------------------------
// THE SINGLE VERTICAL — the recipe's first form through the scene:
// the m1 walk one road over, the counts 10/15/7 with chi = 2, the
// volume 8 - d^2 = 7.75 exact, the operand discovered, the tavola
// lodged, the names under the node's own feature.
// ---------------------------------------------------------------------------
TEST_CASE("G6 m5 the single vertical: 10/15/7 chi=2, the volume "
          "8-d^2 at 1e-12, the bevel named under the node's feature") {
    GuideScene g;
    const auto fid = g.s.add_chamfer_node(
        "", b::ChamferRecipe{"box#1:vertical2", kD});
    REQUIRE(fid != 0);

    const scene::SceneNode* n = g.s.get_node(fid);
    REQUIRE(n != nullptr);
    REQUIRE(n->kind == scene::NodeKind::Chamfer);
    REQUIRE(n->chamfer_body != nullptr);
    REQUIRE(n->chamfer_refusal.empty());

    const b::BRepBody& body = *n->chamfer_body;
    CHECK(body.vertices.size() == 10);
    CHECK(body.edges.size() == 15);
    CHECK(body.face_count() == 7);
    CHECK(b::euler_characteristic(body) == 2);
    CHECK(b::is_closed_orientable(body));

    // The operand discovered (the box), the tavola lodged.
    CHECK(n->chamfer_operand_id == g.box_id);
    CHECK_FALSE(n->tavola.rows.empty());

    // THE GOLDEN (the m1 golden one road over): V = 8 - d^2 = 7.75.
    const b::MassProperties mp =
        b::mass_properties(body, math::Tolerance{});
    CHECK(mp.volume == doctest::Approx(single_volume(kD)).epsilon(1e-12));
    CHECK(mp.all_exact);

    // THE NAME under the node's own feature: the guide case's bevel.
    bool has_v2 = false;
    for (b::FaceId f = 0; f < body.face_count(); ++f) {
        if (b::query_face(body, f).name ==
            "che#2:bevel[box#1:back × box#1:right]")
            has_v2 = true;
    }
    CHECK(has_v2);
}

// ---------------------------------------------------------------------------
// THE SINGLE RING ENTRY — the widened dispatch one feature over:
// the m3 corner act through the scene (the reference a ring edge at
// the bevel-occupied corner), the counts 14/21/9 from the k=1
// chain, V = 8 - 3d^2 + (3/4)d^3.
// ---------------------------------------------------------------------------
TEST_CASE("G6 m5 the single ring entry: the m3 corner act through "
          "the dispatch, 14/21/9, the corner volume at 1e-12") {
    GuideScene g;
    const auto first = g.s.add_chamfer_node(
        "", b::ChamferRecipe{"box#1:vertical2", kD});
    REQUIRE(first != 0);
    // The ring edge at the corner (1,1,1): top1, the Y-parallel at
    // x right (the m3 test's own reference).
    const auto second = g.s.add_chamfer_node(
        "", b::ChamferRecipe{"box#1:top1", kD});
    REQUIRE(second != 0);

    const scene::SceneNode* n = g.s.get_node(second);
    REQUIRE(n != nullptr);
    REQUIRE(n->chamfer_body != nullptr);
    REQUIRE(n->chamfer_refusal.empty());
    CHECK(n->chamfer_body->vertices.size() == 14);
    CHECK(n->chamfer_body->edges.size() == 21);
    CHECK(n->chamfer_body->face_count() == 9);
    CHECK(b::euler_characteristic(*n->chamfer_body) == 2);
    // The operand is the CHAIN's current state (the single's body).
    CHECK(n->chamfer_operand_id == first);
    const b::MassProperties mp = b::mass_properties(
        *n->chamfer_body, math::Tolerance{});
    CHECK(mp.volume ==
          doctest::Approx(corner_volume(kD)).epsilon(1e-12));
}

// ---------------------------------------------------------------------------
// THE TWELVE — the whole cube through the contemporanea: the derived
// chain replayed (the four verticals, then the act), the verdict
// 32/48/18 with chi = 2, the closed form 8-12d^2+6d^3 at 1e-12, the
// tavola lodged, the operand discovered.
// ---------------------------------------------------------------------------
TEST_CASE("G6 m5 the contemporanea consumes the twelve spigoli: "
          "32/48/18 chi=2, the volume 5.75 at 1e-12") {
    GuideScene g;
    const auto fid =
        g.s.add_chamfer_multi_node("", map_of(twelve(kD)));
    REQUIRE(fid != 0);

    const scene::SceneNode* n = g.s.get_node(fid);
    REQUIRE(n != nullptr);
    REQUIRE(n->kind == scene::NodeKind::ChamferMulti);
    REQUIRE(n->chamfer_body != nullptr);
    REQUIRE(n->chamfer_refusal.empty());

    const b::BRepBody& body = *n->chamfer_body;
    CHECK(body.vertices.size() == 32);
    CHECK(body.edges.size() == 48);
    CHECK(body.face_count() == 18);
    CHECK(b::euler_characteristic(body) == 2);
    CHECK(b::is_closed_orientable(body));

    // The operand discovered (the box), the tavola lodged.
    CHECK(n->chamfer_operand_id == g.box_id);
    CHECK_FALSE(n->tavola.rows.empty());

    // THE GOLDEN (the M4 golden one road over, through the recipe).
    const b::MassProperties mp =
        b::mass_properties(body, math::Tolerance{});
    CHECK(mp.volume ==
          doctest::Approx(twelve_volume(kD)).epsilon(1e-12));
    CHECK(mp.all_exact);

    // ONE FEATURE, TWELVE BEVELS (the node's own id, the operand
    // pairs distinct — the naming grammar's own census).
    std::size_t bevels = 0;
    for (b::FaceId f = 0; f < body.face_count(); ++f) {
        const std::string name = b::query_face(body, f).name;
        if (name.rfind("che#2:bevel[", 0) == 0) ++bevels;
    }
    CHECK(bevels == 12);
}

// ---------------------------------------------------------------------------
// THE PARTIAL WORLD: the verticals at MIXED setbacks — the law's own
// arithmetic keeps them free (no two verticals share a corner), the
// counts the formula at k = 4, the volume the additive prism
// formula at 1e-12.
// ---------------------------------------------------------------------------
TEST_CASE("G6 m5 the partials: the verticals at mixed setbacks, the "
          "prism formula at 1e-12") {
    GuideScene g;
    const std::vector<double> setbacks{0.5, 0.3, 0.4, 0.5};
    std::vector<std::pair<std::string, double>> entries;
    for (int k = 0; k < 4; ++k)
        entries.emplace_back(
            std::string("box#1:vertical") + std::to_string(k),
            setbacks[static_cast<std::size_t>(k)]);
    const auto fid =
        g.s.add_chamfer_multi_node("", map_of(entries));
    REQUIRE(fid != 0);

    const scene::SceneNode* n = g.s.get_node(fid);
    REQUIRE(n != nullptr);
    REQUIRE(n->chamfer_body != nullptr);
    REQUIRE(n->chamfer_refusal.empty());
    CHECK(n->chamfer_body->vertices.size() == 16);
    CHECK(n->chamfer_body->edges.size() == 24);
    CHECK(n->chamfer_body->face_count() == 10);

    const b::MassProperties mp = b::mass_properties(
        *n->chamfer_body, math::Tolerance{});
    CHECK(mp.volume ==
          doctest::Approx(partial_volume(setbacks)).epsilon(1e-12));
}

// ---------------------------------------------------------------------------
// THE SETBACK LAW AT CREATION, BY NAME, NEVER AT THE KERNEL (D5.4):
// the full twelve with ONE disagreeing vertical — the corner it
// shares with the ring refuses the map, and the refusal is the
// MACHINE's own (the "attach_chamfer_multi:" provenance — the kernel
// never asked, the message's own prefix the witness).
// ---------------------------------------------------------------------------
TEST_CASE("G6 m5 the setback law refused at creation by name, the "
          "machine's own words — never the kernel's") {
    GuideScene g;
    auto entries = twelve(kD);
    entries[8].second = 0.4;  // box#1:vertical0 disagrees
    bool refused = false;
    std::string what;
    try {
        (void)g.s.add_chamfer_multi_node("", map_of(entries));
    } catch (const std::invalid_argument& e) {
        refused = true;
        what = e.what();
    }
    REQUIRE(refused);
    // D5.4's own name, the two setbacks, the shared corner, the
    // remedy.
    CHECK(what.find("chamfer_corner_setback_mismatch") !=
          std::string::npos);
    CHECK(what.find("box#1:vertical0") != std::string::npos);
    CHECK(what.find("0.400000") != std::string::npos);
    CHECK(what.find("0.500000") != std::string::npos);
    // THE MACHINE'S OWN WORDS: the refusal's provenance is the
    // machine's prefix — the kernel (chamfer_edge:/
    // chamfer_ring_act:) was never asked the question the map
    // already answers.
    CHECK(what.rfind("attach_chamfer_multi:", 0) == 0);
    CHECK(what.find("chamfer_ring_act:") == std::string::npos);
    CHECK(what.find("chamfer_edge:") == std::string::npos);
    // The scene untouched: no node, no id consumed.
    CHECK(g.s.node_count() == 1);
}

// ---------------------------------------------------------------------------
// THE STAGE BAND'S PROVENANCE — the honest opposite: the twelve at
// d = 1.0 agrees everywhere (the law holds), and the KERNEL's own
// admission refuses (the strip law at the second walk) — the
// provenance the kernel walk's own prefix, never the machine's.
// ---------------------------------------------------------------------------
TEST_CASE("G6 m5 the stage band's provenance: the kernel asked at "
          "d=1.0, its own words — the two questions never confused") {
    GuideScene g;
    bool refused = false;
    std::string what;
    try {
        (void)g.s.add_chamfer_multi_node(
            "", map_of(twelve(1.0)));
    } catch (const std::invalid_argument& e) {
        refused = true;
        what = e.what();
    }
    REQUIRE(refused);
    CHECK(what.find("chamfer_edge:") != std::string::npos);
    CHECK(what.find("chamfer_setback_exceeds_face") !=
          std::string::npos);
    CHECK(what.find("chamfer_corner_setback_mismatch") ==
          std::string::npos);
    CHECK(g.s.node_count() == 1);
}

// ---------------------------------------------------------------------------
// THE INDIVISIBILITY (D4.1): the half act refused — a partial ring
// declaration names its own counts; the ring without the complete
// vertical passage names the act's signature (the formula's own
// numbers, 16/24/10).
// ---------------------------------------------------------------------------
TEST_CASE("G6 m5 the indivisibility: the partial ring and the "
          "incomplete vertical passage refused with the derivation's "
          "own numbers") {
    GuideScene g;
    {
        // 4 verticals + 3 ring edges: the map declares 3 of 8.
        std::vector<std::pair<std::string, double>> entries;
        for (const char* tok : {"top0", "top1", "top2"})
            entries.emplace_back(std::string("box#1:") + tok, kD);
        for (const char* tok :
             {"vertical0", "vertical1", "vertical2", "vertical3"})
            entries.emplace_back(std::string("box#1:") + tok, kD);
        std::size_t ring = 0;
        for (const auto& [ref, d] : entries) {
            (void)d;
            if (ref.find(":bottom") != std::string::npos ||
                ref.find(":top") != std::string::npos)
                ++ring;
        }
        REQUIRE(ring == 3);
        bool refused = false;
        std::string what;
        try {
            (void)g.s.add_chamfer_multi_node("", map_of(entries));
        } catch (const std::invalid_argument& e) {
            refused = true;
            what = e.what();
        }
        REQUIRE(refused);
        CHECK(what.find("ring spigoli") != std::string::npos);
        CHECK(what.find("indivisible") != std::string::npos);
    }
    {
        // 2 verticals + the whole ring: the vertical passage
        // incomplete — the act's signature named.
        std::vector<std::pair<std::string, double>> entries;
        for (const char* tok :
             {"bottom0", "bottom1", "bottom2", "bottom3",
              "top0", "top1", "top2", "top3"})
            entries.emplace_back(std::string("box#1:") + tok, kD);
        entries.emplace_back("box#1:vertical0", kD);
        entries.emplace_back("box#1:vertical1", kD);
        bool refused = false;
        std::string what;
        try {
            (void)g.s.add_chamfer_multi_node("", map_of(entries));
        } catch (const std::invalid_argument& e) {
            refused = true;
            what = e.what();
        }
        REQUIRE(refused);
        CHECK(what.find("16/24/10") != std::string::npos);
        CHECK(what.find("FOUR VERTICAL") != std::string::npos);
    }
    {
        // The ring alone on the naked box: 8/12/6 is not 16/24/10.
        std::vector<std::pair<std::string, double>> entries;
        for (const char* tok :
             {"bottom0", "bottom1", "bottom2", "bottom3",
              "top0", "top1", "top2", "top3"})
            entries.emplace_back(std::string("box#1:") + tok, kD);
        bool refused = false;
        std::string what;
        try {
            (void)g.s.add_chamfer_multi_node("", map_of(entries));
        } catch (const std::invalid_argument& e) {
            refused = true;
            what = e.what();
        }
        REQUIRE(refused);
        CHECK(what.find("16/24/10") != std::string::npos);
    }
}

// ---------------------------------------------------------------------------
// THE UNANIMITY OF THE OPERAND: the map spanning two bodies refused
// — the contemporanea replays ONE body's derived chain.
// ---------------------------------------------------------------------------
TEST_CASE("G6 m5 the map spanning two operands refused — the act "
          "is indivisible over one body") {
    scene::Scene s;
    (void)s.add_node("BoxA", CAD_0::sdf::make_box(
                                 math::Vec3f{1.0f, 1.0f, 1.0f}));
    (void)s.add_node("BoxB", CAD_0::sdf::make_box(
                                 math::Vec3f{1.0f, 1.0f, 1.0f}));
    std::vector<std::pair<std::string, double>> entries;
    entries.emplace_back("box#1:vertical0", kD);
    entries.emplace_back("box#2:vertical1", kD);
    bool refused = false;
    std::string what;
    try {
        (void)s.add_chamfer_multi_node("", map_of(entries));
    } catch (const std::invalid_argument& e) {
        refused = true;
        what = e.what();
    }
    REQUIRE(refused);
    CHECK(what.find("TWO operands") != std::string::npos);
    CHECK(s.node_count() == 2);
}

// ---------------------------------------------------------------------------
// THE CHAIN COMPLETION: a single chamfer's chain (one vertical) +
// the contemporanea's remainder (three verticals + the ring) — the
// machine's k0 read from the operand's own counts, the act landing
// from the half-passed state, the verdict the same 32/48/18.
// ---------------------------------------------------------------------------
TEST_CASE("G6 m5 the chain completion: the single's link completed "
          "by the contemporanea, the same verdict") {
    GuideScene g;
    const auto single = g.s.add_chamfer_node(
        "", b::ChamferRecipe{"box#1:vertical0", kD});
    REQUIRE(single != 0);

    std::vector<std::pair<std::string, double>> entries;
    for (const char* tok :
         {"bottom0", "bottom1", "bottom2", "bottom3",
          "top0", "top1", "top2", "top3"})
        entries.emplace_back(std::string("box#1:") + tok, kD);
    for (const char* tok : {"vertical1", "vertical2", "vertical3"})
        entries.emplace_back(std::string("box#1:") + tok, kD);
    const auto fid =
        g.s.add_chamfer_multi_node("", map_of(entries));
    REQUIRE(fid != 0);

    const scene::SceneNode* n = g.s.get_node(fid);
    REQUIRE(n != nullptr);
    REQUIRE(n->chamfer_body != nullptr);
    REQUIRE(n->chamfer_refusal.empty());
    CHECK(n->chamfer_body->vertices.size() == 32);
    CHECK(n->chamfer_body->edges.size() == 48);
    CHECK(n->chamfer_body->face_count() == 18);
    CHECK(b::euler_characteristic(*n->chamfer_body) == 2);
    // The operand is the CHAIN's current state (the single chamfer).
    CHECK(n->chamfer_operand_id == single);
    const b::MassProperties mp = b::mass_properties(
        *n->chamfer_body, math::Tolerance{});
    CHECK(mp.volume ==
          doctest::Approx(twelve_volume(kD)).epsilon(1e-12));
}

// ---------------------------------------------------------------------------
// THE DEAD REFERENCE — the honest door (no operand carries the
// name): the creation refuses loudly, the scene untouched.
// ---------------------------------------------------------------------------
TEST_CASE("G6 m5 the dead reference: the honest door, the scene "
          "untouched") {
    GuideScene g;
    bool refused = false;
    std::string what;
    try {
        (void)g.s.add_chamfer_node(
            "", b::ChamferRecipe{"box#9:vertical0", kD});
    } catch (const std::invalid_argument& e) {
        refused = true;
        what = e.what();
    }
    REQUIRE(refused);
    CHECK(what.find("names no edge") != std::string::npos);
    CHECK(what.find("box#9:vertical0") != std::string::npos);
    CHECK(g.s.node_count() == 1);
}

// ---------------------------------------------------------------------------
// THE DETERMINISM (ADR-0014): the same recipe, re-created in a
// fresh scene, the same final digest — bit-identical; the restore
// road (the pinned id) answers the same body.
// ---------------------------------------------------------------------------
TEST_CASE("G6 m5 the determinism: the re-created recipe's digest "
          "bit-identical, the restore road the same body") {
    GuideScene g1;
    const auto f1 =
        g1.s.add_chamfer_multi_node("", map_of(twelve(kD)));
    REQUIRE(f1 != 0);
    const auto d1 = body_digest(*g1.s.get_node(f1)->chamfer_body);

    GuideScene g2;
    const auto f2 =
        g2.s.add_chamfer_multi_node("", map_of(twelve(kD)));
    REQUIRE(f2 != 0);
    const auto d2 = body_digest(*g2.s.get_node(f2)->chamfer_body);
    CHECK(d1 == d2);

    // The restore road (the command redo's own): the pinned id, the
    // same recipe, the same body — THE SAME CREATOR ID (the born
    // names ride the digest: the first scene's contemporanea is
    // node 2, so the redo restores AT 2).
    GuideScene g3;
    const auto f3 = g3.s.restore_chamfer_multi_node(
        2, "Redo", map_of(twelve(kD)));
    REQUIRE(f3 == 2);
    CHECK(g3.s.get_node(2)->name == "Redo");
    CHECK(body_digest(*g3.s.get_node(2)->chamfer_body) == d1);

    // The SINGLE's own determinism, one form over.
    GuideScene g4;
    const auto s1 = g4.s.add_chamfer_node(
        "", b::ChamferRecipe{"box#1:vertical2", kD});
    REQUIRE(s1 != 0);
    GuideScene g5;
    const auto s2 = g5.s.add_chamfer_node(
        "", b::ChamferRecipe{"box#1:vertical2", kD});
    REQUIRE(s2 != 0);
    CHECK(body_digest(*g4.s.get_node(s1)->chamfer_body) ==
          body_digest(*g5.s.get_node(s2)->chamfer_body));
}

// ---------------------------------------------------------------------------
// THE ORDER PERMUTATIONS: the same logical map (the twelve) in a
// different authored order — the same geometry (the counts, the
// volume), the same TWELVE BEVEL NAMES as a set (the operand pairs
// are the box's own); the numbering is per-recipe and both orders
// deterministic at the re-run.
// ---------------------------------------------------------------------------
TEST_CASE("G6 m5 the order permutations: the same logical recipe, "
          "the same geometry and names, both digests deterministic") {
    const auto bevel_names = [](const b::BRepBody& body) {
        std::vector<std::string> out;
        for (b::FaceId f = 0; f < body.face_count(); ++f) {
            const std::string name =
                b::query_face(body, f).name;
            if (name.find(":bevel[") != std::string::npos)
                out.push_back(name);
        }
        std::sort(out.begin(), out.end());
        return out;
    };
    // (a) The roles' own order.
    GuideScene ga;
    const auto fa =
        ga.s.add_chamfer_multi_node("", map_of(twelve(kD)));
    REQUIRE(fa != 0);
    const auto& body_a = *ga.s.get_node(fa)->chamfer_body;
    // (b) The REVERSED order (verticals first, the ring reversed).
    std::vector<std::pair<std::string, double>> reversed;
    for (const char* tok :
         {"vertical3", "vertical2", "vertical1", "vertical0"})
        reversed.emplace_back(std::string("box#1:") + tok, kD);
    for (const char* tok :
         {"top3", "top2", "top1", "top0",
          "bottom3", "bottom2", "bottom1", "bottom0"})
        reversed.emplace_back(std::string("box#1:") + tok, kD);
    GuideScene gb;
    const auto fb =
        gb.s.add_chamfer_multi_node("", map_of(reversed));
    REQUIRE(fb != 0);
    const auto& body_b = *gb.s.get_node(fb)->chamfer_body;

    // The same geometry.
    CHECK(body_a.vertices.size() == body_b.vertices.size());
    CHECK(body_a.edges.size() == body_b.edges.size());
    CHECK(body_a.face_count() == body_b.face_count());
    const b::MassProperties mpb = b::mass_properties(
        body_b, math::Tolerance{});
    CHECK(mpb.volume ==
          doctest::Approx(twelve_volume(kD)).epsilon(1e-12));
    // The same twelve bevel names as a set.
    const auto names_a = bevel_names(body_a);
    const auto names_b = bevel_names(body_b);
    REQUIRE(names_a.size() == 12);
    REQUIRE(names_b.size() == 12);
    CHECK(names_a == names_b);
    // Both orders deterministic at the re-run.
    {
        GuideScene gc;
        const auto fc = gc.s.add_chamfer_multi_node(
            "", map_of(reversed));
        REQUIRE(fc != 0);
        CHECK(body_digest(*gc.s.get_node(fc)->chamfer_body) ==
              body_digest(body_b));
    }
}

// ---------------------------------------------------------------------------
// THE REGENERATION (the recipe the truth, the body the cache): the
// DAG pass re-run TEN times over the same scene — the chamfer's
// body rebuilt from the recipe every time, the digest NEVER moving,
// the refusal never appearing; a downstream single re-reads the
// contemporanea's re-run body (the document order's own promise).
// ---------------------------------------------------------------------------
TEST_CASE("G6 m5 the regeneration: ten DAG passes, the digest never "
          "moving, the downstream single re-reading the current "
          "state") {
    GuideScene g;
    const auto single = g.s.add_chamfer_node(
        "", b::ChamferRecipe{"box#1:vertical0", kD});
    REQUIRE(single != 0);
    std::vector<std::pair<std::string, double>> entries;
    for (const char* tok :
         {"bottom0", "bottom1", "bottom2", "bottom3",
          "top0", "top1", "top2", "top3"})
        entries.emplace_back(std::string("box#1:") + tok, kD);
    for (const char* tok : {"vertical1", "vertical2", "vertical3"})
        entries.emplace_back(std::string("box#1:") + tok, kD);
    const auto fid =
        g.s.add_chamfer_multi_node("", map_of(entries));
    REQUIRE(fid != 0);
    // A downstream single on the CONTEMPORANEA's own chain state:
    // the reference (vertical2) died in the act, and the discovery
    // answers VANISHED — the superseded state named, the killer
    // named, the cause carried (the tavola's own words). The
    // creation door REFUSES (the Phase F philosophy: nothing
    // installed, no sibling body on the stale state — the defect
    // the discovery's correttiva closed, closed for both kinds).
    {
        bool refused = false;
        std::string what;
        try {
            (void)g.s.add_chamfer_node(
                "", b::ChamferRecipe{"box#1:vertical2", kD});
        } catch (const std::invalid_argument& e) {
            refused = true;
            what = e.what();
        }
        REQUIRE(refused);
        CHECK(what.find("SUPERSEDED") != std::string::npos);
        CHECK(what.find("VANISHED") != std::string::npos);
        CHECK(what.find("replaced by the bevel") !=
              std::string::npos);
        CHECK(g.s.node_count() == 3);
    }

    const std::string want =
        body_digest(*g.s.get_node(fid)->chamfer_body);
    const std::string want_single =
        body_digest(*g.s.get_node(single)->chamfer_body);
    for (int cycle = 0; cycle < 10; ++cycle) {
        (void)scene::rebuild_operations(g.s);
        const scene::SceneNode* n = g.s.get_node(fid);
        REQUIRE(n != nullptr);
        REQUIRE(n->chamfer_body != nullptr);
        REQUIRE(n->chamfer_refusal.empty());
        CHECK(body_digest(*n->chamfer_body) == want);
        CHECK(body_digest(*g.s.get_node(single)->chamfer_body) ==
              want_single);
    }
}
