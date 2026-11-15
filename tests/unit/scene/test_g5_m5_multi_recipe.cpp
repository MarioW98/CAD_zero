// tests/unit/scene/test_g5_m5_multi_recipe.cpp
//
// G5 m5 — LA RICETTA MULTI-SPIGOLO, THE CONTEMPORANEA'S MACHINE
// (ADR-0043 D6.5/D6.6/D9's fifth morso: "la ricetta multi-spigolo e
// la porta v0.8.0 (la mappa spigolo→raggio, i campi nominali, il
// migrator, i golden del formato)"): the many-edge recipe driven
// through the SCENE — the derived chain REPLAYED (D5.1: the
// vertical passage first, each entry its own walk, then THE RING
// AND CORNERS ACT, indivisible), the machine's own laws verified.
//
// THE ANGLES LAW BEFORE THE KERNEL (D6.6/D1, "the connected
// edge-corner graph forces one radius"): the doors' own — a map
// whose entries disagree at a shared corner is refused with
// fillet_corner_radii_mismatch at CREATION, and the refusal is the
// MACHINE's own words (the "attach_fillet_multi:" provenance), the
// kernel NEVER asked — the message's own prefix is the witness, and
// the test pins it.
//
// THE CONTEGGI (the machine's own arithmetic, the defect detector):
// the vertical passage's states the chain formula 8+2k/12+3k/6+k at
// every step (k0 the operand's own — the chain completion case),
// the act's signature 16/24/10 before the call, the verdict
// 24/48/26 with chi = 2 after.
//
// THE GOLDEN OF STEINER (D7.2, the morphological opening): the
// full cube at r = 0.5 reads V = 4 + 11*pi/12 — the machine's body
// through the kernel's divergence theorem (mass_properties) against
// the test's own closed form, 1e-12, the m4 kernel golden re-read
// through the recipe's road. THE PARTIALS (D7.3): the verticals at
// MIXED radii free among themselves (no two verticals share a
// corner — the law's own arithmetic), the additive wedge formula
// 8 - sum_i 2*r_i^2*(1 - pi/4) at 1e-12.
//
// THE SILENT EDIT OF r (D5.4's m6 leg, the uniform edit): the whole
// map re-run at a new radius, the counts FIXED, the volume the
// closed form at the new r. THE DETERMINISM: the same recipe,
// re-created, the same digest — bit-identical (ADR-0014).
//
// THE INDIVISIBILITY (D5.1: "mezzo atto è un corpo che la gate
// rifiuta"): the half act — a partial ring declaration, or the ring
// without the complete vertical passage — refused with the
// derivation's own numbers. THE UNANIMITY OF THE OPERAND: the map
// spanning two bodies refused, the act indivisible. THE CHAIN
// COMPLETION: a single fillet's chain completed by the
// contemporanea — the machine's k0 read from the operand's own
// counts, the act landing from the half-passed state.
#include <doctest/doctest.h>

#include "CAD_0/brep/chain_gate.hpp"
#include "CAD_0/brep/descent.hpp"
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/fillet.hpp"
#include "CAD_0/brep/mass_properties.hpp"
#include "CAD_0/brep/naming.hpp"
#include "CAD_0/brep/query.hpp"
#include "CAD_0/brep/topology.hpp"
#include "CAD_0/scene/scene.hpp"
#include "CAD_0/sdf/primitives.hpp"

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

constexpr double kPi = 3.14159265358979323846;
constexpr double kR = 0.5;

// Steiner's morphological opening (D7.2): V(r) = a^3 + 6a^2 r +
// 3 pi a r^2 + (4/3) pi r^3 with a = 2 - 2r — the test's own
// arithmetic, zero code shared with the kernel.
double steiner(double r) {
    const double a = 2.0 - 2.0 * r;
    return a * a * a + 6.0 * a * a * r +
           3.0 * kPi * a * r * r +
           (4.0 / 3.0) * kPi * r * r * r;
}

// The partials' additive wedge formula (D7.3): 8 - sum 2 r_i^2
// (1 - pi/4) — each vertical's wedge measured against the box.
double wedge_sum(const std::vector<double>& radii) {
    double v = 8.0;
    for (const double r : radii)
        v -= 2.0 * r * r * (1.0 - kPi / 4.0);
    return v;
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

// The twelve entries of the cube at one radius, in the roles'
// own order (bottom ring, top ring, verticals — kBoxEdgeRoles).
std::vector<std::pair<std::string, double>> twelve(double r) {
    std::vector<std::pair<std::string, double>> out;
    for (const char* tok :
         {"bottom0", "bottom1", "bottom2", "bottom3",
          "top0", "top1", "top2", "top3",
          "vertical0", "vertical1", "vertical2", "vertical3"})
        out.emplace_back(std::string("box#1:") + tok, r);
    return out;
}

b::MultiFilletRecipe map_of(
    const std::vector<std::pair<std::string, double>>& entries) {
    b::MultiFilletRecipe r;
    for (const auto& [ref, radius] : entries)
        r.entries.push_back(b::MultiFilletEntry{ref, radius});
    return r;
}

}  // namespace

// ---------------------------------------------------------------------------
// THE TWELVE — the whole cube through the contemporanea: the derived
// chain replayed (the four verticals, then the act), the verdict
// 24/48/26 with chi = 2, Steiner's golden at 1e-12, the tavola
// lodged, the operand discovered.
// ---------------------------------------------------------------------------
TEST_CASE("G5 m5 the contemporanea consumes the twelve spigoli: "
          "24/48/26 chi=2, Steiner's 4+11pi/12 at 1e-12") {
    GuideScene g;
    const auto fid = g.s.add_fillet_multi_node("", map_of(twelve(kR)));
    REQUIRE(fid != 0);

    const scene::SceneNode* n = g.s.get_node(fid);
    REQUIRE(n != nullptr);
    REQUIRE(n->kind == scene::NodeKind::FilletMulti);
    REQUIRE(n->fillet_body != nullptr);
    REQUIRE(n->fillet_refusal.empty());

    const b::BRepBody& body = *n->fillet_body;
    CHECK(body.vertices.size() == 24);
    CHECK(body.edges.size() == 48);
    CHECK(body.face_count() == 26);
    CHECK(b::euler_characteristic(body) == 2);
    CHECK(b::is_closed_orientable(body));

    // The operand discovered (the box), the tavola lodged.
    CHECK(n->fillet_operand_id == g.box_id);
    CHECK_FALSE(n->tavola.rows.empty());

    // THE GOLDEN OF STEINER (D7.2) — the kernel's divergence
    // theorem against the test's own closed form.
    const b::MassProperties mp =
        b::mass_properties(body, math::Tolerance{});
    const double want = 4.0 + 11.0 * kPi / 12.0;
    CHECK(mp.volume == doctest::Approx(want).epsilon(1e-12));
    CHECK(mp.volume == doctest::Approx(steiner(kR)).epsilon(1e-12));
}

// ---------------------------------------------------------------------------
// THE PARTIAL WORLD (D7.3): the verticals at MIXED radii — the law's
// own arithmetic keeps them free (no two verticals share a corner),
// the counts the formula at k = 4, the volume the additive wedge
// formula at 1e-12.
// ---------------------------------------------------------------------------
TEST_CASE("G5 m5 the partials: the verticals at mixed radii, the "
          "wedge formula at 1e-12") {
    GuideScene g;
    const std::vector<double> radii{0.5, 0.3, 0.4, 0.5};
    std::vector<std::pair<std::string, double>> entries;
    for (int k = 0; k < 4; ++k)
        entries.emplace_back(
            std::string("box#1:vertical") + std::to_string(k),
            radii[static_cast<std::size_t>(k)]);
    const auto fid = g.s.add_fillet_multi_node("", map_of(entries));
    REQUIRE(fid != 0);

    const scene::SceneNode* n = g.s.get_node(fid);
    REQUIRE(n != nullptr);
    REQUIRE(n->fillet_body != nullptr);
    REQUIRE(n->fillet_refusal.empty());
    CHECK(n->fillet_body->vertices.size() == 16);
    CHECK(n->fillet_body->edges.size() == 24);
    CHECK(n->fillet_body->face_count() == 10);

    const b::MassProperties mp = b::mass_properties(
        *n->fillet_body, math::Tolerance{});
    CHECK(mp.volume ==
          doctest::Approx(wedge_sum(radii)).epsilon(1e-12));
}

// ---------------------------------------------------------------------------
// THE ANGLES LAW AT CREATION, BY NAME, NEVER AT THE KERNEL (D6.6):
// the full twelve with ONE disagreeing vertical — the corner it
// shares with the ring refuses the map, and the refusal is the
// MACHINE's own (the "attach_fillet_multi:" provenance — the kernel
// never asked, the message's own prefix the witness).
// ---------------------------------------------------------------------------
TEST_CASE("G5 m5 the angles law refused at creation by name, the "
          "machine's own words — never the kernel's") {
    GuideScene g;
    auto entries = twelve(kR);
    entries[8].second = 0.4;  // box#1:vertical0 disagrees
    bool refused = false;
    std::string what;
    try {
        (void)g.s.add_fillet_multi_node("", map_of(entries));
    } catch (const std::invalid_argument& e) {
        refused = true;
        what = e.what();
    }
    REQUIRE(refused);
    // D6.6's own name, the two radii, the shared corner, the remedy.
    CHECK(what.find("fillet_corner_radii_mismatch") !=
          std::string::npos);
    CHECK(what.find("box#1:vertical0") != std::string::npos);
    CHECK(what.find("0.4") != std::string::npos);
    // THE MACHINE'S OWN WORDS: the refusal's provenance is the
    // machine's prefix — the kernel (fillet_edge:/fillet_ring_act:)
    // was never asked the question the map already answers.
    CHECK(what.rfind("attach_fillet_multi:", 0) == 0);
    CHECK(what.find("fillet_ring_act:") == std::string::npos);
    CHECK(what.find("fillet_edge:") == std::string::npos);
    // The scene untouched: no node, no id consumed.
    CHECK(g.s.node_count() == 1);
}

// ---------------------------------------------------------------------------
// THE INDIVISIBILITY (D5.1): the half act refused — a partial ring
// declaration names its own counts; the ring without the complete
// vertical passage names the act's signature (the formula's own
// numbers, 16/24/10).
// ---------------------------------------------------------------------------
TEST_CASE("G5 m5 the indivisibility: the partial ring and the "
          "incomplete vertical passage refused with the derivation's "
          "own numbers") {
    GuideScene g;
    {
        // 4 verticals + 3 ring edges: the map declares 3 of 8.
        std::vector<std::pair<std::string, double>> entries;
        for (const char* tok : {"top0", "top1", "top2"})
            entries.emplace_back(std::string("box#1:") + tok, kR);
        for (const char* tok :
             {"vertical0", "vertical1", "vertical2", "vertical3"})
            entries.emplace_back(std::string("box#1:") + tok, kR);
        std::size_t ring = 0;
        for (const auto& [ref, r] : entries) {
            (void)r;
            if (ref.find(":bottom") != std::string::npos ||
                ref.find(":top") != std::string::npos)
                ++ring;
        }
        REQUIRE(ring == 3);
        bool refused = false;
        std::string what;
        try {
            (void)g.s.add_fillet_multi_node("", map_of(entries));
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
            entries.emplace_back(std::string("box#1:") + tok, kR);
        entries.emplace_back("box#1:vertical0", kR);
        entries.emplace_back("box#1:vertical1", kR);
        bool refused = false;
        std::string what;
        try {
            (void)g.s.add_fillet_multi_node("", map_of(entries));
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
            entries.emplace_back(std::string("box#1:") + tok, kR);
        bool refused = false;
        std::string what;
        try {
            (void)g.s.add_fillet_multi_node("", map_of(entries));
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
TEST_CASE("G5 m5 the map spanning two operands refused — the act "
          "is indivisible over one body") {
    scene::Scene s;
    (void)s.add_node("BoxA", CAD_0::sdf::make_box(
                                 math::Vec3f{1.0f, 1.0f, 1.0f}));
    (void)s.add_node("BoxB", CAD_0::sdf::make_box(
                                 math::Vec3f{1.0f, 1.0f, 1.0f}));
    std::vector<std::pair<std::string, double>> entries;
    entries.emplace_back("box#1:vertical0", kR);
    entries.emplace_back("box#2:vertical1", kR);
    bool refused = false;
    std::string what;
    try {
        (void)s.add_fillet_multi_node("", map_of(entries));
    } catch (const std::invalid_argument& e) {
        refused = true;
        what = e.what();
    }
    REQUIRE(refused);
    CHECK(what.find("TWO operands") != std::string::npos);
    CHECK(s.node_count() == 2);
}

// ---------------------------------------------------------------------------
// THE CHAIN COMPLETION: a single fillet's chain (one vertical) +
// the contemporanea's remainder (three verticals + the ring) — the
// machine's k0 read from the operand's own counts, the act landing
// from the half-passed state, the verdict the same 24/48/26.
// ---------------------------------------------------------------------------
TEST_CASE("G5 m5 the chain completion: the single's link completed "
          "by the contemporanea, the same verdict") {
    GuideScene g;
    const auto single = g.s.add_fillet_node(
        "", b::FilletRecipe{"box#1:vertical0", kR});
    REQUIRE(single != 0);

    std::vector<std::pair<std::string, double>> entries;
    for (const char* tok :
         {"bottom0", "bottom1", "bottom2", "bottom3",
          "top0", "top1", "top2", "top3"})
        entries.emplace_back(std::string("box#1:") + tok, kR);
    for (const char* tok : {"vertical1", "vertical2", "vertical3"})
        entries.emplace_back(std::string("box#1:") + tok, kR);
    const auto fid = g.s.add_fillet_multi_node("", map_of(entries));
    REQUIRE(fid != 0);

    const scene::SceneNode* n = g.s.get_node(fid);
    REQUIRE(n != nullptr);
    REQUIRE(n->fillet_body != nullptr);
    REQUIRE(n->fillet_refusal.empty());
    CHECK(n->fillet_body->vertices.size() == 24);
    CHECK(n->fillet_body->edges.size() == 48);
    CHECK(n->fillet_body->face_count() == 26);
    CHECK(b::euler_characteristic(*n->fillet_body) == 2);
    // The operand is the CHAIN's current state (the single fillet).
    CHECK(n->fillet_operand_id == single);
    const b::MassProperties mp = b::mass_properties(
        *n->fillet_body, math::Tolerance{});
    CHECK(mp.volume ==
          doctest::Approx(4.0 + 11.0 * kPi / 12.0).epsilon(1e-12));
}

// ---------------------------------------------------------------------------
// THE SILENT EDIT OF r (the uniform edit, D5.4's m6 leg): the whole
// map re-run at a new radius — the counts FIXED, the volume the
// closed form at the new r; a downstream consumer re-runs with it.
// ---------------------------------------------------------------------------
TEST_CASE("G5 m5 the uniform silent edit of r: the counts fixed, "
          "the volume at the new radius") {
    GuideScene g;
    const auto fid = g.s.add_fillet_multi_node("", map_of(twelve(kR)));
    REQUIRE(fid != 0);

    const std::string verdict =
        g.s.set_fillet_multi_radius(fid, 0.3);
    CHECK(verdict == "reference_resolved");

    const scene::SceneNode* n = g.s.get_node(fid);
    REQUIRE(n != nullptr);
    REQUIRE(n->fillet_body != nullptr);
    CHECK(n->fillet_body->vertices.size() == 24);
    CHECK(n->fillet_body->edges.size() == 48);
    CHECK(n->fillet_body->face_count() == 26);
    const b::MassProperties mp = b::mass_properties(
        *n->fillet_body, math::Tolerance{});
    CHECK(mp.volume == doctest::Approx(steiner(0.3)).epsilon(1e-12));

    // A bad radius is the door's own loud refusal.
    bool refused = false;
    try {
        (void)g.s.set_fillet_multi_radius(fid, -0.1);
    } catch (const std::invalid_argument&) {
        refused = true;
    }
    CHECK(refused);
}

// ---------------------------------------------------------------------------
// THE DETERMINISM (ADR-0014): the same recipe, re-created in a
// fresh scene, the same final digest — bit-identical; the restore
// road (the pinned id) answers the same body.
// ---------------------------------------------------------------------------
TEST_CASE("G5 m5 the determinism: the re-created recipe's digest "
          "bit-identical, the restore road the same body") {
    GuideScene g1;
    const auto f1 = g1.s.add_fillet_multi_node("", map_of(twelve(kR)));
    REQUIRE(f1 != 0);
    const auto d1 = b::chain_digest(
        *g1.s.get_node(f1)->fillet_body);

    GuideScene g2;
    const auto f2 = g2.s.add_fillet_multi_node("", map_of(twelve(kR)));
    REQUIRE(f2 != 0);
    const auto d2 = b::chain_digest(
        *g2.s.get_node(f2)->fillet_body);
    CHECK(d1 == d2);

    // The restore road (the command redo's own): the pinned id, the
    // same recipe, the same body — THE SAME CREATOR ID (the born
    // names ride the digest: the first scene's contemporanea is
    // node 2, so the redo restores AT 2 — the command's captured id
    // is the digest's own provenance).
    GuideScene g3;
    const auto f3 = g3.s.restore_fillet_multi_node(
        2, "Redo", map_of(twelve(kR)));
    REQUIRE(f3 == 2);
    CHECK(g3.s.get_node(2)->name == "Redo");
    CHECK(b::chain_digest(*g3.s.get_node(2)->fillet_body) == d1);
}

// ---------------------------------------------------------------------------
// THE ADMISSION (the picker's dry run): the verdict is the VALUE —
// the twelve admitted with the operand named; the mismatching map
// refused with the law's own name; the half act refused with the
// derivation's numbers — never a throw.
// ---------------------------------------------------------------------------
TEST_CASE("G5 m5 the admission: the verdict the value, the law "
          "declared before every Apply") {
    GuideScene g;
    {
        const auto a = g.s.fillet_admission_multi(twelve(kR));
        CHECK(a.admitted);
        CHECK(a.reason.empty());
        CHECK(a.operand_id == g.box_id);
        CHECK(a.operand_name == "Box");
    }
    {
        auto entries = twelve(kR);
        entries[8].second = 0.4;
        const auto a = g.s.fillet_admission_multi(entries);
        CHECK_FALSE(a.admitted);
        CHECK(a.reason.find("fillet_corner_radii_mismatch") !=
              std::string::npos);
    }
    {
        std::vector<std::pair<std::string, double>> entries;
        for (const char* tok :
             {"bottom0", "bottom1", "bottom2", "bottom3",
              "top0", "top1", "top2", "top3"})
            entries.emplace_back(std::string("box#1:") + tok, kR);
        const auto a = g.s.fillet_admission_multi(entries);
        CHECK_FALSE(a.admitted);
        CHECK(a.reason.find("16/24/10") != std::string::npos);
    }
    {
        // The empty map and the duplicate edge: the value-class
        // verdicts, still values, never throws.
        const auto empty = g.s.fillet_admission_multi({});
        CHECK_FALSE(empty.admitted);
        std::vector<std::pair<std::string, double>> dup;
        dup.emplace_back("box#1:vertical0", kR);
        dup.emplace_back("box#1:vertical0", kR);
        const auto dupv = g.s.fillet_admission_multi(dup);
        CHECK_FALSE(dupv.admitted);
        CHECK(dupv.reason.find("twice") != std::string::npos);
    }
}

// ---------------------------------------------------------------------------
// THE PICKER'S 12-IN (the Consequences' own register: "il picker
// passa da 4-in a 12-in"): the SINGLE admission now admits the RING
// edges too — the geometric dispatch widened (fillet_ring_edge, the
// m2 walk; the m3 corner trigger inside it at an occupied corner).
// ---------------------------------------------------------------------------
TEST_CASE("G5 m5 the single admission's 12-in: the ring edges "
          "admitted through the widened dispatch, the wall down") {
    GuideScene g;
    for (const char* tok :
         {"bottom0", "bottom1", "bottom2", "bottom3",
          "top0", "top1", "top2", "top3",
          "vertical0", "vertical1", "vertical2", "vertical3"}) {
        const std::string ref = std::string("box#1:") + tok;
        const auto a = g.s.fillet_admission(ref, 0.3);
        INFO("reference: ", ref);
        CHECK(a.admitted);
        CHECK(a.operand_id == g.box_id);
    }
    // The single APPLY on a ring edge: the m2 walk through the
    // scene — the counts one axis over (10/15/7, the m2 golden).
    const auto fid = g.s.add_fillet_node(
        "", b::FilletRecipe{"box#1:bottom0", 0.3});
    REQUIRE(fid != 0);
    const scene::SceneNode* n = g.s.get_node(fid);
    REQUIRE(n != nullptr);
    REQUIRE(n->fillet_body != nullptr);
    CHECK(n->fillet_body->vertices.size() == 10);
    CHECK(n->fillet_body->edges.size() == 15);
    CHECK(n->fillet_body->face_count() == 7);
    CHECK(n->fillet_refusal.empty());
}
