// tests/unit/brep/test_k1_m6_survival_ledger.cpp
//
// K1 m6 — IL SURVIVAL LEDGER: LA STABILITÀ DELLE FEATURE DOPO GLI
// EDIT PARAMETRICI, LE MORTI CON NOME (ADR-0041 D1.4, the family
// protocol's fourth metric — the corpus, the baseline, the counter-
// proofs, the determinism).
//
// THE BASELINE (FASE 3): the two pinned references on the guide body
// — the chain-5's blind (box#1 -> sk#2 -> ext#3 -> sub#4, the m5
// Chain5's own construction, 12/17/8, H=1, chi=2, the volume
// 8 - 0.375*pi):
//   * box#1:vertical2 — the factory's edge, KEPT through the
//     subtract (the tavola's own row, verified on the tree);
//   * sub#4:isect[box#1:top x ext#3:lat(0)]@hi — the rim, BORN at
//     the subtract (the seam-wired pair, ONE logical name).
// Every member of the corpus parts from this baseline: the verdicts
// are comparable member per member, deterministic (ADR-0014: the
// re-runs bit-identical, the digests pinned).
//
// THE CORPUS (FASE 2) — the five typed members, each with its class:
//   1. the SILENT RADIUS EDIT (the fillet's own parameter, r 0.25 ->
//      0.125): the references resolve, the feature re-runs, the
//      counts stable 14/20/9, the volume the D5 closed form at the
//      new r — MATCH expected; the rim's death at the MEAL (fil#5's
//      own act) registered with the killer, the cause verbatim
//      ("replaced by the flank"), the step.
//   2. the SKETCH ROOT EDIT (the middle insert, the map declared —
//      ADR-0036 D4's registered case arriving with its real
//      consumer): the D-tool's chord split in three, the arc
//      VERBATIM moving root 1 -> 3, the map {sk#2:1 -> sk#2:3}
//      declared by the edit path — the arc's wall lat(1) -> lat(3)
//      UNIQUE CONTINUER, the map rendered on the payload, never
//      silence. The rim's seat DECLARED (the chain tool's exit
//      edges are ADR-0038 D4's queued row, unnamed — the m6ter
//      probe's own finding, said never hidden).
//   3. the LOAD BOOLEAN RE-RUN (the chain hydrated from the file):
//      the sketch payload carried VERBATIM (no solve at load — the
//      loader never consults the solver), the boolean re-materialized
//      from the recipe (pass 2, the recipes the truth), the re-run
//      after the load bit-identical — MATCH expected.
//   4. the CHAINED FILLET EDIT (the m6-correttiva-ter's chain box ->
//      fil#2 -> fil#3, the radius edited 0.5 -> 0.25): the m5 chain
//      gate GREEN at every step of the re-materialized chain, the
//      counts fixed 12/18/8 at the edited radius, the volume
//      all-exact — vertical2's death at fil#2 and vertical0's at
//      fil#3 registered (the kept stamp surviving the first link and
//      dying the second — m5's own lesson, now the ledger's rows).
//      The rim's seat DECLARED (the m6ter chain carries no subtract).
//   5. the RECIPE RE-RUN (the recipe's fields re-read): the
//      edge-reference string found BY RENDER against the
//      re-materialized operand (fillet_reference_find — the m4
//      recipe door), the radius its own typed field, the re-run
//      bit-identical — MATCH expected.
//
// THE FRACTION, WITH THE REGISTER: 9/10 — the NUMERATOR pinned,
// never guessed (ADR-0041 D1.4, verbatim: "references that resolve
// class 1 (`reference_resolved`, the feature re-running silently,
// ONE undo entry) over the total"): class 1 ALONE counts — nine
// MATCH in the numerator of the ten (reference, member) pairs, the
// one RENAMED (UNIQUE CONTINUER — a survival of the NAME, not a
// resolve) riding the denominator and the register's counts, never
// the numerator — and the deaths' register
// beside it in the same value: three deaths, each fully named (the
// rim killed by fil#5 at member 1's meal; vertical2 killed by fil#2
// and vertical0 killed by fil#3 at member 4's links — the cause
// "replaced by the flank" verbatim, from the consumers' own tavole).
// The fraction never without the register: the verdict carries both,
// the test pins both.
//
// THE CONTROPROVA (the naive red, the discipline's own): a death
// registered ANONYMOUS — the tavola's vanished row with its cause
// WIPED, the meal step with no killer declared — and the ledger
// REFUSES the member (std::invalid_argument, the diagnosis naming
// the anonymous death, the remedy with it). The test FAILS if the
// ledger registers the anonymous death: the refusal IS the assertion.
// The guard beside them: a reference that finds no row at all (the
// closure violation) refuses the member too — never a ledger on a
// defective tavola.
//
// THE MACHINERY — the edit-caused classes 3 and 4 through the door
// (the m5 row's declared seats: "the mid-chain edits of classes 2/3
// — D1.4's survival ledger, the metric after"): the D5.2 scenarios
// re-read through the ledger's own door — the lens re-authoring (the
// wall AMBIGUOUS, lat(0) E lat(1) nominated, the candidates named on
// the entry) and the square chain (the rim VANISHED, "the authored
// entity was replaced", the killer and the step declared). Class 2
// rode member 2 above; classes 3 and 4 ride here: the door's
// coverage of the resolver's whole vocabulary, one test per class.
//
// THE DETERMINISM: the corpus built twice — the ledger's text
// identical (strings and integers, zero floating-point content: the
// ADR-0014 trivial case), the FNV-1a 64 of the text PINNED (the
// cross-run identity, the m5 digest's own pattern); every member's
// own digest (the chain digests, the caller's witnesses) bit-
// identical between the two builds.
#include "CAD_0/brep/survival_ledger.hpp"
#include "CAD_0/brep/chain_gate.hpp"
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/fillet.hpp"
#include "CAD_0/brep/mass_properties.hpp"
#include "CAD_0/brep/naming.hpp"
#include "CAD_0/brep/descent.hpp"
#include "CAD_0/brep/query.hpp"
#include "CAD_0/brep/resolve.hpp"
#include "CAD_0/brep/topology.hpp"
#include "CAD_0/brep/extrude.hpp"
#include "CAD_0/brep/subtract.hpp"
#include "CAD_0/scene/scene.hpp"
#include "CAD_0/scene/operations.hpp"
#include "CAD_0/io/native.hpp"
#include "CAD_0/sdf/sketch_profile.hpp"
#include "CAD_0/geometry/shape_id.hpp"

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <doctest/doctest.h>

namespace {

namespace b = CAD_0::brep;
namespace n = CAD_0::brep::naming;
namespace s = CAD_0::sdf;
namespace sc = CAD_0::scene;
namespace io = CAD_0::io::native;
using CAD_0::geometry::FeatureId;
using CAD_0::geometry::ShapeId;
using CAD_0::geometry::SubshapeRef;
using Vec2f = CAD_0::math::Vec2f;

constexpr double kPi = 3.14159265358979323846;

bool close(double a, double b, double eps = 1e-9) {
    return std::abs(a - b) <= eps;
}

// The rim fillet's removed volume, the ADR-0039 D1.2 closed form
// (the m5 test's own helper, quoted):
//   V_r = 2*pi*[a*r^2*(1 - pi/4) + r^3*(5/6 - pi/4)]
double wedge_volume(double a, double r) {
    return 2.0 * kPi *
           (a * r * r * (1.0 - kPi / 4.0) +
            r * r * r * (5.0 / 6.0 - kPi / 4.0));
}

std::uint64_t fnv1a64(const std::string& str) {
    std::uint64_t h = 0xcbf29ce484222325ull;
    for (const char c : str) {
        h ^= static_cast<std::uint64_t>(
            static_cast<unsigned char>(c));
        h *= 0x100000001b3ull;
    }
    return h;
}

// ---------------------------------------------------------------------------
// THE CHAIN-5 (the m5 shape, quoted): box#1 -> sk#2 -> ext#3 -> sub#4
// -> fil#5 — the baseline's own chain. Every step's product kept, the
// tavole derived beside.
// ---------------------------------------------------------------------------

struct Chain5 {
    std::unique_ptr<b::BRepBody> box;
    std::vector<s::SketchChain> chains;
    std::unique_ptr<b::BRepBody> tool;
    std::unique_ptr<b::BRepBody> blind;
    std::unique_ptr<b::BRepBody> result;
    ShapeId rim{};   // the subtract's born rim (the fillet's meal)
    ShapeId v2{};    // box#1:vertical2 — the KEPT factory edge

    explicit Chain5(double fillet_radius = 0.25) {
        box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        chains.push_back(s::SketchChain{
            {s::sketch_circle({0.0f, 0.0f}, 0.5f, false)}});
        tool = b::make_prism(chains, 2.0f, +1, FeatureId{3});
        const b::SubtractNaming sub_ctx{
            FeatureId{4}, &n::kBoxNaming, &n::kExtNaming};
        blind = b::exact_subtract(*box, *tool, {0.0, 0.0, -0.5},
                                  b::kSubtractTolerance, sub_ctx);
        for (std::size_t i = 0; i < blind->edge_count(); ++i) {
            const auto e = b::query_edge(*blind, i);
            if (e.name ==
                "sub#4:isect[box#1:top \xc3\x97 ext#3:lat(0)]@hi") {
                rim = blind->edges[i].shape_id();
                break;
            }
        }
        for (std::size_t i = 0; i < blind->edge_count(); ++i) {
            const auto e = b::query_edge(*blind, i);
            if (e.name == "box#1:vertical2") {
                v2 = blind->edges[i].shape_id();
                break;
            }
        }
        const b::FilletNaming fil_ctx{FeatureId{5}, &n::kBoxNaming,
                                      FeatureId{3}, &n::kExtNaming};
        result = b::fillet_rim_edge(*blind, rim, fillet_radius,
                                    fil_ctx);
    }
};

// The subtract's INSTANCE declaration (the m5 test's own shape — the
// rim's grammar: the creating operation's tables with the operands'
// bindings).
struct SubDecl {
    n::OperandBinding bindings[2]{};
    n::OperationNaming decl{};

    SubDecl() {
        bindings[0] = {FeatureId{1}, &n::kBoxNaming};
        bindings[1] = {FeatureId{3}, &n::kExtNaming};
        decl = n::OperationNaming{
            n::FeatureKind::Subtract, {}, n::kSubEdgeRoles,
            {bindings, 2}};
    }
};

// ---------------------------------------------------------------------------
// THE D-CHAIN (member 2's own): box#1 -> sk#2 [chord, arc] -> ext#3
// -> sub#4 — the tool authored CHORD-FIRST so the arc (the
// cylindrical wall) rides ROOT 1: the middle insert (the chord split
// in three) moves the arc VERBATIM 1 -> 3, the map declared — the
// rename's own seat, probed on the tree before the pin.
// ---------------------------------------------------------------------------

struct DChain {
    std::unique_ptr<b::BRepBody> box;
    std::vector<s::SketchChain> chains;
    std::unique_ptr<b::BRepBody> tool;
    std::unique_ptr<b::BRepBody> blind;
    ShapeId v2{};
    ShapeId wall{};  // ext#3:lat(1) — the ARC's wall, absorbed

    DChain(std::vector<s::SketchChain> authored) {

        box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        chains = std::move(authored);
        tool = b::make_prism(chains, 3.0f, +1, FeatureId{3});
        const b::SubtractNaming sub_ctx{
            FeatureId{4}, &n::kBoxNaming, &n::kExtNaming};
        blind = b::exact_subtract(*box, *tool, {0.0, 0.0, -1.5},
                                  b::kSubtractTolerance, sub_ctx);
        for (std::size_t i = 0; i < blind->edge_count(); ++i) {
            if (b::query_edge(*blind, i).name == "box#1:vertical2") {
                v2 = blind->edges[i].shape_id();
                break;
            }
        }
        for (std::size_t i = 0; i < blind->face_count(); ++i) {
            const auto f = b::query_face(*blind, i);
            if (f.name == "ext#3:lat(1)") {
                wall = f.id;
                break;
            }
        }
    }
};

std::vector<s::SketchChain> d_chords() {
    const float pi = 3.14159265358979323846f;
    const float r = 0.5f;
    return {s::SketchChain{{
        s::sketch_segment(Vec2f{-r, 0.0f}, Vec2f{r, 0.0f}, true),
        s::sketch_arc(Vec2f{0.0f, 0.0f}, r, false, 0.0f, pi),
    }}};
}

// THE EDIT (member 2's own): the middle insert — the chord split in
// three segments (the inserts at positions 1 and 2), the ARC
// VERBATIM at root 3. The edit path declares the map
// {sk#2: c0:e1 -> sk#2: c0:e3} — the arc's own survival.
std::vector<s::SketchChain> d_split() {
    const float pi = 3.14159265358979323846f;
    const float r = 0.5f;
    return {s::SketchChain{{
        s::sketch_segment(Vec2f{-r, 0.0f}, Vec2f{-r / 3.0f, 0.0f},
                          true),
        s::sketch_segment(Vec2f{-r / 3.0f, 0.0f}, Vec2f{r / 3.0f, 0.0f},
                          true),
        s::sketch_segment(Vec2f{r / 3.0f, 0.0f}, Vec2f{r, 0.0f}, true),
        s::sketch_arc(Vec2f{0.0f, 0.0f}, r, false, 0.0f, pi),
    }}};
}

// ---------------------------------------------------------------------------
// THE M6TER CHAIN (member 4's own): box -> fil#2 (vertical2) ->
// fil#3 (vertical0) — the m5 chained fillets, the radius edited.
// ---------------------------------------------------------------------------

struct M6ter {
    std::unique_ptr<b::BRepBody> box;
    std::unique_ptr<b::BRepBody> first;   // fil#2's result
    std::unique_ptr<b::BRepBody> result;  // fil#3's result
    ShapeId v2{};
    ShapeId v0{};

    M6ter(double r2 = 0.5, double r3 = 0.5) {
        box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        v2 = ShapeId{FeatureId{1}, SubshapeRef{}.push(10), {}};
        v0 = ShapeId{FeatureId{1}, SubshapeRef{}.push(8), {}};
        const b::FilletNaming ctx2{FeatureId{2}, &n::kBoxNaming};
        const b::FilletNaming ctx3{FeatureId{3}, &n::kBoxNaming};
        first = b::fillet_edge(*box, v2, r2, ctx2);
        result = b::fillet_edge(*first, v0, r3, ctx3);
    }
};

// ---------------------------------------------------------------------------
// THE MINIMAL CHAIN-5 SCENE (member 3's own): the nodes boxed at the
// kernel's own FeatureIds (box 1, sketch 2, extrude 3, subtract 4,
// fillet 5) so the rim's string IS the pinned baseline — probed on
// the tree before the pin.
// ---------------------------------------------------------------------------

struct LoadedChain {
    sc::Scene scene;
    std::unique_ptr<b::BRepBody> rerun_blind;
    std::unique_ptr<b::BRepBody> rerun_fil;
    std::string blind_digest;
    std::string fil_digest;
    std::string chains_digest;
    std::size_t rebuilt{0};
    std::string rim_string;

    LoadedChain() {
        {
            sc::SceneNode box_n;
            box_n.id = 1;
            box_n.name = "Box";
            box_n.builder = "box";
            box_n.params = {{"extent_x", 1.0f}, {"extent_y", 1.0f},
                            {"extent_z", 1.0f}};
            box_n.body = s::make_box(CAD_0::math::Vec3f{1.0, 1.0, 1.0});
            scene.restore_node(std::move(box_n));
        }
        {
            std::vector<s::SketchChain> chains;
            chains.push_back({{s::sketch_circle(
                Vec2f{0.0f, 0.0f}, 0.5f, false)}});
            scene.add_sketch_node("Sketch", std::move(chains), {});
        }
        {
            sc::OperationRecipe r;
            r.type = sc::OperationType::Extrude;
            r.sketch = 2;
            r.distance = 2.0f;
            r.direction = 1;
            scene.add_operation_node("Extrude", r);
        }
        {
            sc::OperationRecipe r;
            r.type = sc::OperationType::Subtract;
            r.target = 1;
            r.tool = 3;
            r.offset_z = -0.5f;
            scene.add_operation_node("Subtract", r);
        }
        // The reference DISCOVERED by the find (the m4 recipe door's
        // own discipline — never typed by hand): the top rim, the
        // seam-wired pair taken whole.
        const sc::SceneNode* sub = scene.get_node(4);
        REQUIRE(sub != nullptr);
        REQUIRE(sub->hybrid != nullptr);
        for (std::size_t i = 0; i < sub->hybrid->brep->edge_count();
             ++i) {
            const b::EdgeInfo e =
                b::query_edge(*sub->hybrid->brep, i);
            if (!e.name.empty() &&
                e.name.find("@hi") != std::string::npos &&
                e.name.find("isect") != std::string::npos) {
                rim_string = e.name;
                break;
            }
        }
        REQUIRE(rim_string ==
                "sub#4:isect[box#1:top \xc3\x97 ext#3:lat(0)]@hi");
        scene.add_fillet_node("Fillet",
                              b::FilletRecipe{rim_string, 0.25});

        // The creation's digests (the pointers re-fetched: the
        // scene's vector reallocates on every add).
        const sc::SceneNode* sub2 = scene.get_node(4);
        const sc::SceneNode* fil2 = scene.get_node(5);
        blind_digest = b::chain_digest(*sub2->hybrid->brep);
        fil_digest = b::chain_digest(*fil2->fillet_body);
        chains_digest =
            b::sketch_digest(scene.get_node(2)->sketch);

        // SAVE -> LOAD: the chain hydrated (the sketch payload
        // VERBATIM — no solve at load; the derived bodies
        // re-materialized by pass 2 from the recipes alone).
        const auto tmp = std::filesystem::temp_directory_path() /
                         "k1_m6_load_rerun.CAD_0";
        io::save_file(tmp, scene, io::NativeCamera{});
        const io::ParsedDocument doc = io::load_file(tmp);
        std::filesystem::remove(tmp);
        REQUIRE(doc.nodes.size() == 5);
        // The sketch payload carried verbatim: the loader never
        // consults the solver (the solved chains ARE the payload).
        REQUIRE(b::sketch_digest(doc.nodes[1].sketch) ==
                chains_digest);

        // THE RE-RUN AFTER THE LOAD: install + rebuild_operations —
        // the boolean re-materialized from the recipe, the digests
        // bit-identical (ADR-0014's N-step clause through the file's
        // own round-trip).
        sc::Scene re;
        io::install_document(re, io::ParsedDocument(doc));
        const auto rr = sc::rebuild_operations(re);
        rebuilt = rr.rebuilt;
        const sc::SceneNode* rsub = re.get_node(4);
        const sc::SceneNode* rfil = re.get_node(5);
        REQUIRE(rsub != nullptr);
        REQUIRE(rsub->hybrid != nullptr);
        REQUIRE(rfil != nullptr);
        REQUIRE(rfil->fillet_body != nullptr);
        REQUIRE(b::chain_digest(*rsub->hybrid->brep) == blind_digest);
        REQUIRE(b::chain_digest(*rfil->fillet_body) == fil_digest);
        rerun_blind =
            std::make_unique<b::BRepBody>(*rsub->hybrid->brep);
        b::link_radial_edges(*rerun_blind);
        rerun_fil =
            std::make_unique<b::BRepBody>(*rfil->fillet_body);
        b::link_radial_edges(*rerun_fil);
    }
};

} // namespace

// ===========================================================================
// THE BASELINE — the two pinned references on the guide body
// ===========================================================================

TEST_CASE("K1 m6 the baseline: the two pinned references on the guide "
          "body — vertical2 KEPT through the subtract, the rim BORN at "
          "it, the digest deterministic") {
    const Chain5 c;
    const Chain5 again;
    REQUIRE(c.blind != nullptr);
    REQUIRE(c.result != nullptr);
    REQUIRE(!(c.rim == ShapeId{}));
    REQUIRE(!(c.v2 == ShapeId{}));

    // THE GUIDE BODY: the m5 golden's own numbers.
    CHECK(c.blind->vertex_count() == 12);
    CHECK(c.blind->edge_count() == 17);
    CHECK(c.blind->face_count() == 8);

    // vertical2 — the factory's edge, KEPT through the subtract (the
    // tavola's own row: the probe's finding, pinned).
    const auto tav = b::subtract_descent(*c.blind, *c.box, *c.tool);
    bool v2_kept = false;
    for (const auto& row : tav.rows) {
        if (row.shape_id == c.v2) {
            CHECK(row.relation == b::Descent::Kept);
            CHECK(row.name == "box#1:vertical2");
            v2_kept = true;
        }
    }
    CHECK(v2_kept);

    // The rim — BORN at the subtract, the seam-wired pair ONE
    // logical name: the pinned string answers on the guide body.
    const auto rim_ids = b::query_edge_by_name(*c.blind, c.rim);
    CHECK(rim_ids.size() == 2);  // the pair taken whole

    // THE DETERMINISM: the same recipe built twice — every digest
    // bit-identical (ADR-0014, the baseline's own floor).
    CHECK(b::chain_digest(*c.blind) == b::chain_digest(*again.blind));
    CHECK(b::chain_digest(*c.result) ==
          b::chain_digest(*again.result));
}

// ===========================================================================
// THE CORPUS — the five typed members, the fraction WITH the register
// ===========================================================================

TEST_CASE("K1 m6 the corpus: five typed members — the classes, the "
          "deaths named, the fraction 9/10 with the register beside it") {
    const SubDecl sub_grammar{};

    // ---- The shared chains -------------------------------------------
    const Chain5 original(0.25);
    const Chain5 edited(0.125);
    const DChain d_old(d_chords());
    const DChain d_new(d_split());
    const M6ter m6ter_original(0.5, 0.5);
    const M6ter m6ter_edited(0.25, 0.5);
    const LoadedChain loaded;
    REQUIRE(original.blind != nullptr);
    REQUIRE(edited.result != nullptr);
    REQUIRE(d_old.blind != nullptr);
    REQUIRE(d_new.blind != nullptr);
    REQUIRE(m6ter_original.result != nullptr);
    REQUIRE(m6ter_edited.result != nullptr);

    // The tavole (the evidence's own storage, alive through the
    // ledger call).
    const auto sub_tav = b::subtract_descent(
        *original.blind, *original.box, *original.tool);
    const auto fil_tav = b::fillet_descent(*edited.result,
                                           *edited.blind);
    const auto d_sub_tav = b::subtract_descent(
        *d_old.blind, *d_old.box, *d_old.tool);
    const auto box_tav = b::factory_descent(*m6ter_edited.box);
    const auto old_fil2_tav = b::fillet_descent(
        *m6ter_original.first, *m6ter_original.box);
    const auto new_fil2_tav = b::fillet_descent(
        *m6ter_edited.first, *m6ter_edited.box);
    const auto fil3_tav = b::fillet_descent(
        *m6ter_edited.result, *m6ter_edited.first);
    const auto load_sub_tav = b::subtract_descent(
        *loaded.scene.get_node(4)->hybrid->brep,
        *sc::operand_exact_brep(*loaded.scene.get_node(1), "box"),
        *sc::operand_exact_brep(*loaded.scene.get_node(3), "tool"));

    // The edit contexts (the edit paths' own declarations).
    const b::EditContext no_shift{/*roots_no_shift=*/true, {}};
    b::EditContext chord_map;
    chord_map.root_map.push_back(
        {n::RootRef{FeatureId{2}, n::RootKind::Entity, 0, 1},
         n::RootRef{FeatureId{2}, n::RootKind::Entity, 0, 3}});

    std::vector<b::SurvivalMember> members;

    // ---- Member 1: the SILENT RADIUS EDIT ----------------------------
    {
        b::SurvivalMember m;
        m.kind = b::SurvivalEditKind::SilentRadiusEdit;
        m.name = "the silent radius edit: fil#5 at r=0.125";
        m.digest = b::chain_digest(*edited.result);
        m.references = {
            {"box#1:vertical2", original.v2, &n::kBoxNaming, false},
            {"sub#4:isect[box#1:top \xc3\x97 ext#3:lat(0)]@hi",
             original.rim, &sub_grammar.decl, false}};
        b::SurvivalStep edit_step;
        edit_step.kind = b::SurvivalStepKind::Edit;
        edit_step.name =
            "sub#4 re-materialized (the edit touches fil#5 alone)";
        edit_step.refs = {
            {0, &sub_tav, edited.blind.get(), no_shift},
            {1, &sub_tav, edited.blind.get(), no_shift}};
        b::SurvivalStep meal_step;
        meal_step.kind = b::SurvivalStepKind::Meal;
        meal_step.name = "fil#5 at r=0.125";
        meal_step.killer = "fil#5";
        meal_step.refs = {
            {0, &fil_tav, edited.result.get(), b::EditContext{}},
            {1, &fil_tav, edited.result.get(), b::EditContext{}}};
        m.steps = {std::move(edit_step), std::move(meal_step)};
        members.push_back(std::move(m));
    }

    // ---- Member 2: the SKETCH ROOT EDIT (the map declared) ------------
    {
        b::SurvivalMember m;
        m.kind = b::SurvivalEditKind::SketchRootEdit;
        m.name = "the sketch root edit: the chord split in three, "
                 "the map declared";
        m.digest = b::chain_digest(*d_new.blind);
        m.references = {
            {"box#1:vertical2", d_old.v2, &n::kBoxNaming, false},
            {"ext#3:lat(1)", d_old.wall, &n::kExtNaming, false}};
        m.seats_declared = {
            "the rim rides the subtract's circle chains - the "
            "chain tool's exit edges are ADR-0038 D4's queued row "
            "(unnamed), and this member's rename seat is the arc's "
            "wall"};
        b::SurvivalStep edit_step;
        edit_step.kind = b::SurvivalStepKind::Edit;
        edit_step.name = "sk#2 the middle insert (the chord split, "
                         "the arc 1 -> 3, the map declared)";
        edit_step.refs = {
            {0, &d_sub_tav, d_new.blind.get(), b::EditContext{}},
            {1, &d_sub_tav, d_new.blind.get(), chord_map}};
        m.steps = {std::move(edit_step)};
        members.push_back(std::move(m));
    }

    // ---- Member 3: the LOAD BOOLEAN RE-RUN ----------------------------
    {
        b::SurvivalMember m;
        m.kind = b::SurvivalEditKind::LoadBooleanRerun;
        m.name = "the load boolean re-run: the chain hydrated, the "
                 "boolean re-materialized from the recipe";
        m.digest = loaded.blind_digest;
        m.references = {
            {"box#1:vertical2", original.v2, &n::kBoxNaming, false},
            {"sub#4:isect[box#1:top \xc3\x97 ext#3:lat(0)]@hi",
             original.rim, &sub_grammar.decl, false}};
        b::SurvivalStep edit_step;
        edit_step.kind = b::SurvivalStepKind::Edit;
        edit_step.name = "the load's re-materialization (pass 2, "
                         "the recipes the truth; the sketch payload "
                         "verbatim - no solve at load)";
        edit_step.refs = {
            {0, &load_sub_tav, loaded.rerun_blind.get(), no_shift},
            {1, &load_sub_tav, loaded.rerun_blind.get(), no_shift}};
        m.steps = {std::move(edit_step)};
        members.push_back(std::move(m));
    }

    // ---- Member 4: the CHAINED FILLET EDIT ----------------------------
    {
        b::SurvivalMember m;
        m.kind = b::SurvivalEditKind::ChainedFilletEdit;
        m.name = "the chained fillet edit: fil#2's radius 0.5 -> 0.25";
        m.digest = b::chain_digest(*m6ter_edited.result);
        m.references = {
            {"box#1:vertical2", m6ter_original.v2, &n::kBoxNaming,
             false},
            {"box#1:vertical0", m6ter_original.v0, &n::kBoxNaming,
             false}};
        m.seats_declared = {
            "the m6ter chain carries no subtract - the rim rides "
            "the subtract's chains (members 1, 3 and 5); this "
            "member's own seats are the baseline's vertical2 and "
            "the m5 kept-stamp vertical0"};
        // Step 1 — THE EDIT: the chain re-materialized at the new
        // radius. vertical2's chain (the box) is untouched: it
        // resolves on the operand and fil#2 re-runs; vertical0 rides
        // the FIRST LINK: the old fil#2's kept row against the
        // re-materialized link — the kept stamp survives the edit.
        b::SurvivalStep edit_step;
        edit_step.kind = b::SurvivalStepKind::Edit;
        edit_step.name = "fil#2 re-materialized at r=0.25";
        edit_step.refs = {
            {0, &box_tav, m6ter_edited.box.get(),
             b::EditContext{}},
            {1, &old_fil2_tav, m6ter_edited.first.get(),
             b::EditContext{}}};
        // Step 2 — THE FIRST MEAL: fil#2 consumes vertical2.
        b::SurvivalStep meal2;
        meal2.kind = b::SurvivalStepKind::Meal;
        meal2.name = "fil#2 at r=0.25 consumes vertical2";
        meal2.killer = "fil#2";
        meal2.refs = {{0, &new_fil2_tav,
                       m6ter_edited.first.get(),
                       b::EditContext{}}};
        // Step 3 — THE SECOND MEAL: fil#3 consumes vertical0 (the
        // kept stamp that survived the first link dies at the
        // second — m5's own lesson).
        b::SurvivalStep meal3;
        meal3.kind = b::SurvivalStepKind::Meal;
        meal3.name = "fil#3 at r=0.5 consumes vertical0";
        meal3.killer = "fil#3";
        meal3.refs = {{1, &fil3_tav, m6ter_edited.result.get(),
                       b::EditContext{}}};
        m.steps = {std::move(edit_step), std::move(meal2),
                   std::move(meal3)};
        members.push_back(std::move(m));
    }

    // ---- Member 5: the RECIPE RE-RUN ----------------------------------
    {
        // The recipe's fields RE-READ (the m4 door): the string found
        // BY RENDER on the re-materialized operand, the radius its
        // own typed field — the find's answer IS the pinned rim.
        const std::string rim_string =
            "sub#4:isect[box#1:top \xc3\x97 ext#3:lat(0)]@hi";
        const ShapeId found = b::fillet_reference_find(
            *original.blind, rim_string);
        CHECK(found == original.rim);
        const b::FilletNaming fil_ctx{
            FeatureId{5}, &n::kBoxNaming, FeatureId{3},
            &n::kExtNaming};
        auto rerun = b::fillet_rim_edge(*original.blind, found, 0.25,
                                        fil_ctx);
        REQUIRE(rerun != nullptr);
        CHECK(b::chain_digest(*rerun) ==
              b::chain_digest(*original.result));

        b::SurvivalMember m;
        m.kind = b::SurvivalEditKind::RecipeRerun;
        m.name = "the recipe re-run: the fields re-read, the find, "
                 "the DAG re-materialized";
        m.digest = b::chain_digest(*rerun);
        m.references = {
            {"box#1:vertical2", original.v2, &n::kBoxNaming, false},
            {"sub#4:isect[box#1:top \xc3\x97 ext#3:lat(0)]@hi",
             original.rim, &sub_grammar.decl, false}};
        b::SurvivalStep edit_step;
        edit_step.kind = b::SurvivalStepKind::Edit;
        edit_step.name = "the recipe's fields re-read (the find on "
                         "the re-materialized operand, the radius "
                         "its own field)";
        edit_step.refs = {
            {0, &sub_tav, original.blind.get(), no_shift},
            {1, &sub_tav, original.blind.get(), no_shift}};
        m.steps = {std::move(edit_step)};
        members.push_back(std::move(m));
    }

    // ---- THE LEDGER ----------------------------------------------------
    const b::SurvivalLedger ledger = b::survival_ledger(members);

    // THE ARITHMETIC: ten (reference, member) pairs — nine MATCH,
    // one RENAMED, the fraction 9/10 with the NUMERATOR pinned
    // (ADR-0041 D1.4: "references that resolve class 1 ... over the
    // total" — class 1 ALONE; the one RENAMED rides the
    // denominator, never the numerator) — and the register beside
    // it: three deaths, each fully named.
    CHECK(ledger.members.size() == 5);
    CHECK(ledger.total == 10);
    CHECK(ledger.resolved == 9);
    CHECK(ledger.renamed == 1);
    CHECK(ledger.ambiguous == 0);
    CHECK(ledger.vanished == 0);
    CHECK(ledger.fraction() == std::pair<std::size_t,
                                         std::size_t>{9, 10});
    REQUIRE(ledger.deaths.size() == 3);

    // ---- THE MATRIX, member per member ---------------------------------
    const auto& m1 = ledger.members[0];
    CHECK(m1.entries[0].code == "reference_resolved");  // vertical2
    CHECK(m1.entries[1].code == "reference_resolved");  // the rim
    CHECK(m1.entries[1].dead);                          // ...at the meal
    CHECK(m1.entries[1].killer == "fil#5");
    CHECK(m1.entries[1].cause == "replaced by the flank");
    CHECK(m1.entries[1].step == "fil#5 at r=0.125");
    CHECK(m1.entries[1].died ==
          "sub#4:isect[box#1:top \xc3\x97 ext#3:lat(0)]@hi");
    CHECK(m1.entries[1].remedy.find("survives the operation") !=
          std::string::npos);

    const auto& m2 = ledger.members[1];
    CHECK(m2.entries[0].code == "reference_resolved");  // vertical2
    CHECK(m2.entries[1].code == "reference_renamed");   // the wall
    CHECK(m2.entries[1].renamed_from == "ext#3:lat(1)");
    CHECK(m2.entries[1].renamed_to == "ext#3:lat(3)");
    CHECK(m2.entries[1].renamed_map == "sk#2:1 -> sk#2:3");
    CHECK_FALSE(m2.entries[1].dead);
    REQUIRE(m2.seats_declared.size() == 1);
    CHECK(m2.seats_declared[0].find("queued row") !=
          std::string::npos);

    const auto& m3 = ledger.members[2];
    CHECK(m3.entries[0].code == "reference_resolved");
    CHECK(m3.entries[1].code == "reference_resolved");
    CHECK_FALSE(m3.entries[1].dead);

    const auto& m4 = ledger.members[3];
    CHECK(m4.entries[0].code == "reference_resolved");  // vertical2
    CHECK(m4.entries[0].dead);                          // ...at fil#2
    CHECK(m4.entries[0].killer == "fil#2");
    CHECK(m4.entries[0].cause == "replaced by the flank");
    CHECK(m4.entries[1].code == "reference_resolved");  // vertical0
    CHECK(m4.entries[1].dead);                          // ...at fil#3
    CHECK(m4.entries[1].killer == "fil#3");
    CHECK(m4.entries[1].cause == "replaced by the flank");
    REQUIRE(m4.seats_declared.size() == 1);
    CHECK(m4.seats_declared[0].find("no subtract") !=
          std::string::npos);

    const auto& m5 = ledger.members[4];
    CHECK(m5.entries[0].code == "reference_resolved");
    CHECK(m5.entries[1].code == "reference_resolved");
    CHECK_FALSE(m5.entries[1].dead);

    // ---- THE REGISTER, riga per riga -----------------------------------
    const std::string reg = ledger.deaths_text();
    CHECK(reg.find("fil#5") != std::string::npos);
    CHECK(reg.find("fil#2") != std::string::npos);
    CHECK(reg.find("fil#3") != std::string::npos);
    CHECK(reg.find("replaced by the flank") != std::string::npos);
    CHECK(reg.find("the silent radius edit") != std::string::npos);
    CHECK(reg.find("the chained fillet edit") != std::string::npos);
    // The register's lines: one per death (3 lines, the remedy riding
    // each tail).
    CHECK(std::count(reg.begin(), reg.end(), '\n') == 3);

    // ---- THE MEMBERS' OWN WITNESSES ------------------------------------
    // (1) The counts stable at the edited radius; the volume the D5
    // closed form; the upstream digests unchanged, the edited
    // step's changed (an edit is not a drift).
    CHECK(edited.result->vertex_count() == 14);
    CHECK(edited.result->edge_count() == 20);
    CHECK(edited.result->face_count() == 9);
    {
        const auto mp = b::mass_properties(
            *edited.result, CAD_0::math::Tolerance::linear(1e-9));
        CHECK(close(mp.volume, 6.810780258845277, 1e-12));
        CHECK(close(mp.volume,
                    (8.0 - 0.375 * kPi) -
                        wedge_volume(0.5, 0.125), 1e-12));
        CHECK(mp.all_exact);
    }
    CHECK(b::chain_digest(*edited.box) ==
          b::chain_digest(*original.box));
    CHECK(b::sketch_digest(edited.chains) ==
          b::sketch_digest(original.chains));
    CHECK(b::chain_digest(*edited.tool) ==
          b::chain_digest(*original.tool));
    CHECK(b::chain_digest(*edited.blind) ==
          b::chain_digest(*original.blind));
    CHECK(b::chain_digest(*edited.result) !=
          b::chain_digest(*original.result));

    // (2) The rename's continuation ANSWERS on the new guide body
    // (the by-name surface — the feature's own next step).
    {
        ShapeId want{FeatureId{3},
                     SubshapeRef{}.push(0).push(3), {}};
        const auto ids = b::query_face_by_name(*d_new.blind, want);
        REQUIRE(ids.size() == 1);
        CHECK(b::query_face(*d_new.blind, ids[0]).name ==
              "ext#3:lat(3)");
        // The counts: the split's own arithmetic (the probe's
        // finding: 16/24/10).
        CHECK(d_new.blind->vertex_count() == 16);
        CHECK(d_new.blind->edge_count() == 24);
        CHECK(d_new.blind->face_count() == 10);
    }

    // (3) The load's own witnesses (the probe's findings): the
    // sketch payload verbatim, the re-run rebuilt, the digests equal.
    CHECK(loaded.rebuilt == 2);
    CHECK(b::chain_digest(*loaded.rerun_blind) == loaded.blind_digest);
    CHECK(b::chain_digest(*loaded.rerun_fil) == loaded.fil_digest);

    // (4) THE GATE AT EVERY STEP of the re-materialized chain (the
    // m5 instrument reused — the mandate's own "gate a ogni passo"):
    // the box, the edited fil#2, the edited fil#3 — all green, the
    // counts fixed at the edited radius, the volume all-exact.
    {
        const auto g0 = b::chain_gate(b::ChainStep{
            m6ter_edited.box.get(), &box_tav, nullptr, nullptr,
            nullptr, {}, ""});
        CHECK(g0.pass);
        CHECK(g0.v == 8);
        CHECK(g0.e == 12);
        CHECK(g0.f == 6);
        const auto g1 = b::chain_gate(b::ChainStep{
            m6ter_edited.first.get(), &new_fil2_tav,
            m6ter_edited.box.get(), nullptr, nullptr, {}, ""});
        CHECK(g1.pass);
        CHECK(g1.v == 10);
        CHECK(g1.e == 15);
        CHECK(g1.f == 7);
        CHECK(g1.chi == 2);
        const auto g2 = b::chain_gate(b::ChainStep{
            m6ter_edited.result.get(), &fil3_tav,
            m6ter_edited.first.get(), nullptr, nullptr, {}, ""});
        CHECK(g2.pass);
        CHECK(g2.v == 12);
        CHECK(g2.e == 18);
        CHECK(g2.f == 8);
        CHECK(g2.chi == 2);
        const auto mp = b::mass_properties(
            *m6ter_edited.result,
            CAD_0::math::Tolerance::linear(1e-9));
        CHECK(mp.all_exact);
    }

    // (5) The find's answer is the pinned rim (asserted above) and
    // the re-run's digest equals the original's (asserted above).
}

// ===========================================================================
// THE CONTROPROVA — the anonymous death refused; the guard refused
// ===========================================================================

TEST_CASE("K1 m6 the controprova: the anonymous death REFUSED — the "
          "wiped cause, the killer-less step, the guard — never a "
          "partial ledger") {
    const SubDecl sub_grammar{};
    const Chain5 original(0.25);
    const Chain5 edited(0.125);
    REQUIRE(original.blind != nullptr);
    REQUIRE(edited.result != nullptr);

    // ---- (1) THE WIPED CAUSE: the tavola's vanished row with its
    // cause erased — the resolver answers Vanished with an empty
    // why, and the ledger REFUSES to register the member. THE TEST
    // FAILS if the death is registered anonymous: the refusal IS the
    // assertion (REQUIRE_THROWS_AS fails on a normal return).
    {
        auto fil_tav = b::fillet_descent(*edited.result,
                                         *edited.blind);
        bool wiped = false;
        for (auto& row : fil_tav.rows) {
            if (row.relation == b::Descent::Vanished &&
                row.shape_id == original.rim) {
                row.cause = "";  // the anonymity injected
                wiped = true;
            }
        }
        REQUIRE(wiped);
        const auto sub_tav = b::subtract_descent(
            *original.blind, *original.box, *original.tool);

        b::SurvivalMember m;
        m.kind = b::SurvivalEditKind::SilentRadiusEdit;
        m.name = "the wiped-cause evidence";
        m.references = {
            {"sub#4:isect[box#1:top \xc3\x97 ext#3:lat(0)]@hi",
             original.rim, &sub_grammar.decl, false}};
        b::SurvivalStep edit_step;
        edit_step.kind = b::SurvivalStepKind::Edit;
        edit_step.name = "sub#4 re-materialized";
        edit_step.refs = {{0, &sub_tav, edited.blind.get(),
                           b::EditContext{/*roots_no_shift=*/true, {}}}};
        b::SurvivalStep meal_step;
        meal_step.kind = b::SurvivalStepKind::Meal;
        meal_step.name = "fil#5 at r=0.125";
        meal_step.killer = "fil#5";
        meal_step.refs = {{0, &fil_tav, edited.result.get(),
                           b::EditContext{}}};
        m.steps = {std::move(edit_step), std::move(meal_step)};

        bool refused = false;
        try {
            const auto ledger = b::survival_ledger({m});
            (void)ledger;  // the anonymous death registered: the FAIL
        } catch (const std::invalid_argument& e) {
            refused = true;
            const std::string what = e.what();
            CHECK(what.find("a death without its name is refused") !=
                  std::string::npos);
            CHECK(what.find("cause: ''") != std::string::npos);
            CHECK(what.find("ADR-0041 D1.4") != std::string::npos);
            CHECK(what.find("Remedy:") != std::string::npos);
        }
        CHECK(refused);
    }

    // ---- (2) THE KILLER-LESS STEP: the meal declared without its
    // killer — the death cannot name WHO: refused the same.
    {
        const auto sub_tav = b::subtract_descent(
            *original.blind, *original.box, *original.tool);
        const auto fil_tav = b::fillet_descent(*edited.result,
                                               *edited.blind);
        b::SurvivalMember m;
        m.kind = b::SurvivalEditKind::SilentRadiusEdit;
        m.name = "the killer-less evidence";
        m.references = {
            {"sub#4:isect[box#1:top \xc3\x97 ext#3:lat(0)]@hi",
             original.rim, &sub_grammar.decl, false}};
        b::SurvivalStep edit_step;
        edit_step.kind = b::SurvivalStepKind::Edit;
        edit_step.name = "sub#4 re-materialized";
        edit_step.refs = {{0, &sub_tav, edited.blind.get(),
                           b::EditContext{/*roots_no_shift=*/true, {}}}};
        b::SurvivalStep meal_step;
        meal_step.kind = b::SurvivalStepKind::Meal;
        meal_step.name = "fil#5 at r=0.125";
        meal_step.killer = "";  // the killer withheld
        meal_step.refs = {{0, &fil_tav, edited.result.get(),
                           b::EditContext{}}};
        m.steps = {std::move(edit_step), std::move(meal_step)};

        bool refused = false;
        try {
            const auto ledger = b::survival_ledger({m});
            (void)ledger;
        } catch (const std::invalid_argument& e) {
            refused = true;
            const std::string what = e.what();
            CHECK(what.find("killer: ''") != std::string::npos);
        }
        CHECK(refused);
    }

    // ---- (3) THE CONTROL (the discrimination): the SAME evidence
    // with the names whole registers — the death named, the fraction
    // with the register. The refusal is the ANONYMITY's own, never
    // the scenario's.
    {
        const auto sub_tav = b::subtract_descent(
            *original.blind, *original.box, *original.tool);
        const auto fil_tav = b::fillet_descent(*edited.result,
                                               *edited.blind);
        b::SurvivalMember m;
        m.kind = b::SurvivalEditKind::SilentRadiusEdit;
        m.name = "the control: the names whole";
        m.references = {
            {"sub#4:isect[box#1:top \xc3\x97 ext#3:lat(0)]@hi",
             original.rim, &sub_grammar.decl, false}};
        b::SurvivalStep edit_step;
        edit_step.kind = b::SurvivalStepKind::Edit;
        edit_step.name = "sub#4 re-materialized";
        edit_step.refs = {{0, &sub_tav, edited.blind.get(),
                           b::EditContext{/*roots_no_shift=*/true, {}}}};
        b::SurvivalStep meal_step;
        meal_step.kind = b::SurvivalStepKind::Meal;
        meal_step.name = "fil#5 at r=0.125";
        meal_step.killer = "fil#5";
        meal_step.refs = {{0, &fil_tav, edited.result.get(),
                           b::EditContext{}}};
        m.steps = {std::move(edit_step), std::move(meal_step)};

        const auto ledger = b::survival_ledger({m});
        CHECK(ledger.total == 1);
        CHECK(ledger.resolved == 1);
        REQUIRE(ledger.deaths.size() == 1);
        CHECK(ledger.deaths[0].cause == "replaced by the flank");
        CHECK(ledger.deaths[0].killer == "fil#5");
    }

    // ---- (4) THE GUARD: a reference that finds no row at all — the
    // closure violation in the evidence's tavola: the member refused,
    // the resolver's detail verbatim.
    {
        const auto sub_tav = b::subtract_descent(
            *original.blind, *original.box, *original.tool);
        const ShapeId orphan{FeatureId{99},
                             SubshapeRef{}.push(0), {}};
        b::SurvivalMember m;
        m.kind = b::SurvivalEditKind::SilentRadiusEdit;
        m.name = "the orphan evidence";
        m.references = {{"the never-existed", orphan,
                         &sub_grammar.decl, false}};
        b::SurvivalStep edit_step;
        edit_step.kind = b::SurvivalStepKind::Edit;
        edit_step.name = "sub#4 re-materialized";
        edit_step.refs = {{0, &sub_tav, edited.blind.get(),
                           b::EditContext{/*roots_no_shift=*/true, {}}}};
        m.steps = {std::move(edit_step)};

        bool refused = false;
        try {
            const auto ledger = b::survival_ledger({m});
            (void)ledger;
        } catch (const std::invalid_argument& e) {
            refused = true;
            const std::string what = e.what();
            CHECK(what.find("defective") != std::string::npos);
        }
        CHECK(refused);
    }
}

// ===========================================================================
// THE MACHINERY — the edit-caused classes 3 and 4 through the door
// (the D5.2 scenarios re-read: the lens and the square)
// ===========================================================================

TEST_CASE("K1 m6 the machinery: the edit-caused classes through the "
          "door — the lens AMBIGUOUS with the candidates named, the "
          "square VANISHED with the cause and the killer declared") {
    // The D5.2 scenarios' own recipes (quoted from the quartet test):
    // the THROUGH guide (the offset (0,0,-1.5), the height 3).
    const auto circle_chains = [](float radius) {
        return std::vector<s::SketchChain>{s::SketchChain{{
            s::sketch_circle(Vec2f{0.0f, 0.0f}, radius, false)}}};
    };
    const auto lens_chains = [] {
        const float r = 0.3f;
        const Vec2f c1{-0.12f, 0.0f}, c2{0.12f, 0.0f};
        const float y = std::sqrt(r * r - 0.12f * 0.12f);
        const Vec2f a{0.0f, y}, bb{0.0f, -y};
        const float pi = 3.14159265358979323846f;
        const float two_pi = 2.0f * pi;
        const float ang_b1 =
            std::atan2(bb.y - c1.y, bb.x - c1.x);
        const float ang_a1 =
            std::atan2(a.y - c1.y, a.x - c1.x);
        float span1 = ang_a1 - ang_b1;
        if (span1 < 0.0f) span1 += two_pi;
        const float ang_a2 =
            std::atan2(a.y - c2.y, a.x - c2.x);
        const float ang_b2 =
            std::atan2(bb.y - c2.y, bb.x - c2.x);
        float span2 = ang_b2 - ang_a2;
        if (span2 < 0.0f) span2 += two_pi;
        return std::vector<s::SketchChain>{s::SketchChain{{
            s::sketch_arc(c1, r, false, ang_b1, span1),
            s::sketch_arc(c2, r, false, ang_a2, span2),
        }}};
    };
    const auto square_chains = [] {
        const float half = 0.2f;
        return std::vector<s::SketchChain>{s::SketchChain{{
            s::sketch_segment(Vec2f{-half, -half},
                              Vec2f{half, -half}, true),
            s::sketch_segment(Vec2f{half, -half},
                              Vec2f{half, half}, true),
            s::sketch_segment(Vec2f{half, half},
                              Vec2f{-half, half}, true),
            s::sketch_segment(Vec2f{-half, half},
                              Vec2f{-half, -half}, true),
        }}};
    };
    const auto guide = [&](const std::vector<s::SketchChain>& chains) {
        auto box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        auto tool = b::make_prism(chains, 3.0f, +1, FeatureId{3});
        const b::SubtractNaming ctx{
            FeatureId{4}, &n::kBoxNaming, &n::kExtNaming};
        auto result = b::exact_subtract(*box, *tool, {0.0, 0.0, -1.5},
                                        b::kSubtractTolerance, ctx);
        struct G {
            std::unique_ptr<b::BRepBody> box;
            std::unique_ptr<b::BRepBody> tool;
            std::unique_ptr<b::BRepBody> result;
        };
        return G{std::move(box), std::move(tool), std::move(result)};
    };

    const SubDecl sub_grammar{};
    const auto old_g = guide(circle_chains(0.25f));
    REQUIRE(old_g.result != nullptr);

    // The picks (the D5.2 pins, re-derived here): the wall (the
    // absorbed face, the birth name kept) and the rim (the fillet's
    // own string).
    ShapeId wall{};
    bool wall_found = false;
    for (std::size_t i = 0; i < old_g.result->face_count(); ++i) {
        const auto f = b::query_face(*old_g.result, i);
        if (f.name == "ext#3:lat(0)") {
            wall = f.id;
            wall_found = true;
            break;
        }
    }
    REQUIRE(wall_found);
    ShapeId rim{};
    bool rim_found = false;
    for (std::size_t i = 0; i < old_g.result->edge_count(); ++i) {
        const auto e = b::query_edge(*old_g.result, i);
        if (e.name ==
            "sub#4:isect[box#1:top \xc3\x97 ext#3:lat(0)]@hi") {
            rim = e.id;
            rim_found = true;
            break;
        }
    }
    REQUIRE(rim_found);
    const auto old_tav = b::subtract_descent(
        *old_g.result, *old_g.box, *old_g.tool);

    // ---- (a) THE LENS: the wall AMBIGUOUS — lat(0) E lat(1)
    // nominated, the candidates named on the entry (class 3 through
    // the door: the split's own answer, the D5.2a precedent).
    {
        const auto new_g = guide(lens_chains());
        REQUIRE(new_g.result != nullptr);
        b::SurvivalMember m;
        m.kind = b::SurvivalEditKind::SketchRootEdit;
        m.name = "the lens re-authoring (the split)";
        m.references = {{"ext#3:lat(0)", wall, &n::kExtNaming,
                         false}};
        b::SurvivalStep edit_step;
        edit_step.kind = b::SurvivalStepKind::Edit;
        edit_step.name = "sk#2 the circle re-authored as a chain of "
                         "two arcs";
        edit_step.refs = {{0, &old_tav, new_g.result.get(),
                           b::EditContext{}}};
        m.steps = {std::move(edit_step)};

        const auto ledger = b::survival_ledger({m});
        CHECK(ledger.total == 1);
        CHECK(ledger.ambiguous == 1);
        REQUIRE(ledger.members[0].entries.size() == 1);
        const auto& e = ledger.members[0].entries[0];
        CHECK(e.code == "reference_ambiguous");
        REQUIRE(e.candidates.size() == 2);
        CHECK(e.candidates[0] == "ext#3:lat(0)");
        CHECK(e.candidates[1] == "ext#3:lat(1)");
        CHECK_FALSE(e.dead);  // the ambiguity is not a death
        CHECK(ledger.deaths.empty());
    }

    // ---- (b) THE SQUARE: the rim VANISHED — "the authored entity
    // was replaced", the killer and the step declared (class 4
    // through the door: the replacement's own answer, the D5.2b
    // precedent).
    {
        const auto new_g = guide(square_chains());
        REQUIRE(new_g.result != nullptr);
        b::SurvivalMember m;
        m.kind = b::SurvivalEditKind::SketchRootEdit;
        m.name = "the square replacement";
        m.references = {
            {"sub#4:isect[box#1:top \xc3\x97 ext#3:lat(0)]@hi", rim,
             &sub_grammar.decl, false}};
        b::SurvivalStep edit_step;
        edit_step.kind = b::SurvivalStepKind::Edit;
        edit_step.name = "sk#2 the circle replaced by the square "
                         "chain";
        edit_step.killer = "sk#2 re-authored";
        edit_step.refs = {{0, &old_tav, new_g.result.get(),
                           b::EditContext{}}};
        m.steps = {std::move(edit_step)};

        const auto ledger = b::survival_ledger({m});
        CHECK(ledger.total == 1);
        CHECK(ledger.vanished == 1);
        REQUIRE(ledger.deaths.size() == 1);
        const auto& d = ledger.deaths[0];
        CHECK(d.code == "reference_vanished");
        CHECK(d.cause == "the authored entity was replaced");
        CHECK(d.killer == "sk#2 re-authored");
        CHECK(d.step ==
              "sk#2 the circle replaced by the square chain");
        CHECK(d.remedy.find("reselect") != std::string::npos);
    }
}

// ===========================================================================
// THE DETERMINISM — the corpus twice, the text identical, the digest
// pinned
// ===========================================================================

TEST_CASE("K1 m6 the determinism: the corpus built twice — the "
          "ledger's text identical, the FNV pinned, the members' "
          "digests bit-identical") {
    const SubDecl sub_grammar{};

    // The corpus bundle — the chains, the tavole and the members OWN
    // TOGETHER (the evidence's pointers must outlive the ledger
    // call: the bundle is the owner, the locals' addresses never
    // dangle).
    struct Corpus {
        std::unique_ptr<Chain5> original, edited;
        std::unique_ptr<DChain> d_old, d_new;
        std::unique_ptr<M6ter> m6ter_original, m6ter_edited;
        b::DescentTable sub_tav, fil_tav, d_sub_tav, box_tav,
            old_fil2_tav, new_fil2_tav, fil3_tav;
        std::vector<b::SurvivalMember> members;
    };

    const auto build = [&]() {
        Corpus c;
        c.original = std::make_unique<Chain5>(0.25);
        c.edited = std::make_unique<Chain5>(0.125);
        c.d_old = std::make_unique<DChain>(d_chords());
        c.d_new = std::make_unique<DChain>(d_split());
        c.m6ter_original = std::make_unique<M6ter>(0.5, 0.5);
        c.m6ter_edited = std::make_unique<M6ter>(0.25, 0.5);

        c.sub_tav = b::subtract_descent(
            *c.original->blind, *c.original->box,
            *c.original->tool);
        c.fil_tav = b::fillet_descent(*c.edited->result,
                                      *c.edited->blind);
        c.d_sub_tav = b::subtract_descent(
            *c.d_old->blind, *c.d_old->box, *c.d_old->tool);
        c.box_tav = b::factory_descent(*c.m6ter_edited->box);
        c.old_fil2_tav = b::fillet_descent(
            *c.m6ter_original->first, *c.m6ter_original->box);
        c.new_fil2_tav = b::fillet_descent(
            *c.m6ter_edited->first, *c.m6ter_edited->box);
        c.fil3_tav = b::fillet_descent(*c.m6ter_edited->result,
                                       *c.m6ter_edited->first);

        const b::EditContext no_shift{true, {}};
        b::EditContext chord_map;
        chord_map.root_map.push_back(
            {n::RootRef{FeatureId{2}, n::RootKind::Entity, 0, 1},
             n::RootRef{FeatureId{2}, n::RootKind::Entity, 0, 3}});

        std::vector<b::SurvivalMember>& members = c.members;
        {
            b::SurvivalMember m;
            m.kind = b::SurvivalEditKind::SilentRadiusEdit;
            m.name = "the silent radius edit: fil#5 at r=0.125";
            m.digest = b::chain_digest(*c.edited->result);
            m.references = {
                {"box#1:vertical2", c.original->v2, &n::kBoxNaming,
                 false},
                {"sub#4:isect[box#1:top \xc3\x97 ext#3:lat(0)]@hi",
                 c.original->rim, &sub_grammar.decl, false}};
            b::SurvivalStep es;
            es.kind = b::SurvivalStepKind::Edit;
            es.name = "sub#4 re-materialized (the edit touches "
                      "fil#5 alone)";
            es.refs = {
                {0, &c.sub_tav, c.edited->blind.get(), no_shift},
                {1, &c.sub_tav, c.edited->blind.get(), no_shift}};
            b::SurvivalStep ms;
            ms.kind = b::SurvivalStepKind::Meal;
            ms.name = "fil#5 at r=0.125";
            ms.killer = "fil#5";
            ms.refs = {
                {0, &c.fil_tav, c.edited->result.get(),
                 b::EditContext{}},
                {1, &c.fil_tav, c.edited->result.get(),
                 b::EditContext{}}};
            m.steps = {std::move(es), std::move(ms)};
            members.push_back(std::move(m));
        }
        {
            b::SurvivalMember m;
            m.kind = b::SurvivalEditKind::SketchRootEdit;
            m.name = "the sketch root edit: the chord split in "
                     "three, the map declared";
            m.digest = b::chain_digest(*c.d_new->blind);
            m.references = {
                {"box#1:vertical2", c.d_old->v2, &n::kBoxNaming,
                 false},
                {"ext#3:lat(1)", c.d_old->wall, &n::kExtNaming,
                 false}};
            m.seats_declared = {
                "the rim rides the subtract's circle chains - the "
                "chain tool's exit edges are ADR-0038 D4's queued "
                "row (unnamed), and this member's rename seat is "
                "the arc's wall"};
            b::SurvivalStep es;
            es.kind = b::SurvivalStepKind::Edit;
            es.name = "sk#2 the middle insert (the chord split, "
                      "the arc 1 -> 3, the map declared)";
            es.refs = {
                {0, &c.d_sub_tav, c.d_new->blind.get(),
                 b::EditContext{}},
                {1, &c.d_sub_tav, c.d_new->blind.get(),
                 chord_map}};
            m.steps = {std::move(es)};
            members.push_back(std::move(m));
        }
        {
            b::SurvivalMember m;
            m.kind = b::SurvivalEditKind::ChainedFilletEdit;
            m.name =
                "the chained fillet edit: fil#2's radius 0.5 -> 0.25";
            m.digest = b::chain_digest(*c.m6ter_edited->result);
            m.references = {
                {"box#1:vertical2", c.m6ter_original->v2,
                 &n::kBoxNaming, false},
                {"box#1:vertical0", c.m6ter_original->v0,
                 &n::kBoxNaming, false}};
            m.seats_declared = {
                "the m6ter chain carries no subtract - the rim "
                "rides the subtract's chains (members 1, 3 and 5); "
                "this member's own seats are the baseline's "
                "vertical2 and the m5 kept-stamp vertical0"};
            b::SurvivalStep es;
            es.kind = b::SurvivalStepKind::Edit;
            es.name = "fil#2 re-materialized at r=0.25";
            es.refs = {
                {0, &c.box_tav, c.m6ter_edited->box.get(),
                 b::EditContext{}},
                {1, &c.old_fil2_tav, c.m6ter_edited->first.get(),
                 b::EditContext{}}};
            b::SurvivalStep m2;
            m2.kind = b::SurvivalStepKind::Meal;
            m2.name = "fil#2 at r=0.25 consumes vertical2";
            m2.killer = "fil#2";
            m2.refs = {{0, &c.new_fil2_tav,
                        c.m6ter_edited->first.get(),
                        b::EditContext{}}};
            b::SurvivalStep m3;
            m3.kind = b::SurvivalStepKind::Meal;
            m3.name = "fil#3 at r=0.5 consumes vertical0";
            m3.killer = "fil#3";
            m3.refs = {{1, &c.fil3_tav,
                        c.m6ter_edited->result.get(),
                        b::EditContext{}}};
            m.steps = {std::move(es), std::move(m2), std::move(m3)};
            members.push_back(std::move(m));
        }
        return c;
    };

    const Corpus a = build();
    const Corpus b_ = build();
    const auto ledger_a = b::survival_ledger(a.members);
    const auto ledger_b = b::survival_ledger(b_.members);

    // THE TEXT identical (strings and integers, zero floating-point
    // content — ADR-0014's trivial case): the corpus answers the
    // same ledger, run after run.
    CHECK(ledger_a.text() == ledger_b.text());
    CHECK_FALSE(ledger_a.text().empty());
    // The counts and the register equal.
    CHECK(ledger_a.total == ledger_b.total);
    CHECK(ledger_a.resolved == ledger_b.resolved);
    CHECK(ledger_a.deaths.size() == ledger_b.deaths.size());
    // The members' digests (the chain digests — the callers' own
    // witnesses) bit-identical between the two builds.
    for (std::size_t i = 0; i < a.members.size(); ++i)
        CHECK(a.members[i].digest == b_.members[i].digest);
    // THE PIN — the FNV-1a 64 of the text, the cross-run identity
    // (the m5 digest's own pattern; the value pinned at the first
    // green run and frozen from then on).
    CHECK(fnv1a64(ledger_a.text()) == 0x961ff463edcd8d63ull);
}
