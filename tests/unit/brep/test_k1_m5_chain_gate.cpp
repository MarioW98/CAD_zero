// tests/unit/brep/test_k1_m5_chain_gate.cpp
//
// K1 m5 — IL CHAIN GATE: LA CORRETTEZZA TOPOLOGICA DOPO OGNI PASSO
// (ADR-0041 D1.3, the chain benchmark's own suite member — the gate
// the metric's definition demands, run after EVERY step of the
// sequences the tree can author).
//
// THE BASELINE — THE CHAIN-5: box → sketch → extrude → subtract →
// fillet, the guide chain the m6 protocol walked by hand (the box,
// the blind hole, the rim filleted at r = 0.25). The gate after
// each of the five steps, the verdict per step in the row:
//
//   box      8/12/6  H=0 chi=2 — the factory tavola all born (26)
//   sketch   the profile's own door: closed chains, the digest pure
//   extrude  4/5/3   H=0 chi=2 — the seam-wired circle (ADR-0023),
//            the tavola born 12, the FROM side resolving
//   subtract 12/17/8 H=1 chi=2 — the blind body, the tavola 47
//            rows (kept 25 / absorbed 3 / born 9 / vanished 10),
//            the volume 8 − 0.375π exact
//   fillet   14/20/9 H=1 chi=2 — the rim filleted, the tavola 47
//            rows (kept 30 / absorbed 3 / born 10 / vanished 4),
//            the volume 6.775059758517764 (the ADR-0039 D5 golden)
//
// THE CENSUS BY NAME, in movement (ADR-0036 D3's classes at the
// gate): the SURVIVING reference (box#1:top through the subtract,
// absorbed, its stamp answering; the kept stamp through the chained
// fillets), the BORN one (the rim the subtract births, the fillet's
// flank), the DEAD one (the rim the fillet consumes — vanished,
// the tavola's own cause "replaced by the flank" riding the row);
// the classes 2/3 refused by name (the registered v1 boundary).
//
// THE SEQUENCES the tree can author, each gated at every step: the
// re-run of the recipe (the chain built twice, every step's digest
// bit-identical — drift = 0, ADR-0014's N-step form); the chained
// fillets (the m6-correttiva-ter's box → fil#2 → fil#3, the
// reference that dies refusing and the stamp that survives); the
// silent edit (the chain re-run at r = 0.125 — the upstream steps'
// digests UNCHANGED, the edited step's changed: the edit is VISIBLE
// exactly where it acts, the counts fixed 14/20/9 for every in-band
// r, the volume the D5 closed form at the new r).
//
// THE CONTROPROVA (the naive red, per component — a gate that
// approves everything is no gate): a LOST CO-EDGE, an OPEN LOOP, an
// ODD CHI (the isolated vertex), a LOST NAME (the stamp wiped), a
// DRIFTING DIGEST (the position nudged) — each caught WITH ITS OWN
// NAMED MESSAGE, on the honest chain green.
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
#include "CAD_0/sdf/sketch_profile.hpp"
#include "CAD_0/geometry/shape_id.hpp"

#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <doctest/doctest.h>

namespace {

namespace b = CAD_0::brep;
namespace n = CAD_0::brep::naming;
namespace s = CAD_0::sdf;
using CAD_0::geometry::FeatureId;
using CAD_0::geometry::ShapeId;
using CAD_0::geometry::SubshapeRef;
using CAD_0::math::Vec3d;

constexpr double kPi = 3.14159265358979323846;

bool close(double a, double b, double eps = 1e-9) {
    return std::abs(a - b) <= eps;
}

// The rim fillet's removed volume, the ADR-0039 D1.2 closed form
// (quoted, never re-derived differently):
//   V_r = 2*pi*[a*r^2*(1 - pi/4) + r^3*(5/6 - pi/4)]
double wedge_volume(double a, double r) {
    return 2.0 * kPi *
           (a * r * r * (1.0 - kPi / 4.0) +
            r * r * r * (5.0 / 6.0 - kPi / 4.0));
}

// THE CHAIN-5's own construction: box#1 -> sk#2 -> ext#3 -> sub#4
// -> fil#5 (the RimGuide's own ids, ADR-0039 D3's). Every step's
// product kept, the tavole derived beside (the caller gates them).
struct Chain5 {
    std::unique_ptr<b::BRepBody> box;
    std::vector<s::SketchChain> chains;
    std::unique_ptr<b::BRepBody> tool;
    std::unique_ptr<b::BRepBody> blind;
    std::unique_ptr<b::BRepBody> result;
    ShapeId rim{};       // the subtract's born rim (the fillet's meal)
    ShapeId top{};       // box#1:top — the surviving absorbed face

    explicit Chain5(double fillet_radius = 0.25) {
        // Step 1 — the box.
        box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        // Step 2 — the sketch (the profile: the circle a = 0.5).
        chains.push_back(s::SketchChain{
            {s::sketch_circle({0.0f, 0.0f}, 0.5f, false)}});
        // Step 3 — the extrude (the prism, height 2, +Z).
        tool = b::make_prism(chains, 2.0f, +1, FeatureId{3});
        // Step 4 — the subtract (the tool offset to (0,0,-0.5): the
        // tool z in [-0.5, 1.5], the floor at -0.5, the rim through
        // the top at z = 1).
        const b::SubtractNaming sub_ctx{
            FeatureId{4}, &n::kBoxNaming, &n::kExtNaming};
        blind = b::exact_subtract(*box, *tool, {0.0, 0.0, -0.5},
                                  b::kSubtractTolerance, sub_ctx);
        // The born rim BY NAME (the structural ShapeId, never a
        // parse of the render), and the surviving top beside it —
        // the two references the census moves through the chain.
        for (std::size_t i = 0; i < blind->edge_count(); ++i) {
            const auto e = b::query_edge(*blind, i);
            if (e.name ==
                "sub#4:isect[box#1:top \xc3\x97 ext#3:lat(0)]@hi") {
                rim = blind->edges[i].shape_id();
                break;
            }
        }
        for (std::size_t i = 0; i < blind->face_count(); ++i) {
            const auto f = b::query_face(*blind, i);
            if (f.name == "box#1:top") {
                top = blind->solid.shell().faces()[i].shape_id();
                break;
            }
        }
        // Step 5 — the fillet on the rim.
        const b::FilletNaming fil_ctx{FeatureId{5}, &n::kBoxNaming,
                                      FeatureId{3}, &n::kExtNaming};
        result = b::fillet_rim_edge(*blind, rim, fillet_radius,
                                    fil_ctx);
    }
};

// The step's tavole, derived (the ADR-0036 D4 rule: derived, never
// stored — the recipe is the source of truth, the tavola is cache).
struct Chain5Tavole {
    b::DescentTable box_tav{};     // factory_descent: all born
    b::DescentTable ext_tav{};     // extrude_descent: the FROMs
    b::DescentTable sub_tav{};     // subtract_descent: 47 rows
    b::DescentTable fil_tav{};     // fillet_descent: 47 rows

    explicit Chain5Tavole(const Chain5& c) {
        box_tav = b::factory_descent(*c.box);
        ext_tav = b::extrude_descent(*c.tool, c.chains,
                                     FeatureId{3}, FeatureId{2});
        sub_tav = b::subtract_descent(*c.blind, *c.box, *c.tool);
        fil_tav = b::fillet_descent(*c.result, *c.blind);
    }
};

// The five steps' gate calls for one chain — the row's own walk.
// `rerun` supplies the re-run's digests (empty strings = the drift
// honestly n/a; the baseline's own test supplies the real re-run).
struct Chain5Verdicts {
    b::ChainGateVerdict box_v{};
    b::SketchGateVerdict sketch_v{};
    b::ChainGateVerdict ext_v{};
    b::ChainGateVerdict sub_v{};
    b::ChainGateVerdict fil_v{};
};

Chain5Verdicts gate_chain5(const Chain5& c, const Chain5Tavole& t,
                           const std::string& box_rerun,
                           const std::string& sketch_rerun,
                           const std::string& ext_rerun,
                           const std::string& sub_rerun,
                           const std::string& fil_rerun) {
    Chain5Verdicts v;

    // Step 1 — the box (the factory root: no old generation behind
    // it, the census's old side vacuously closed).
    v.box_v = b::chain_gate(b::ChainStep{
        c.box.get(), &t.box_tav, nullptr, nullptr, nullptr, {},
        box_rerun});

    // Step 2 — the sketch (the profile's own door).
    v.sketch_v = b::chain_gate_sketch(c.chains, sketch_rerun);

    // Step 3 — the extrude (the FROM side is the old side: the
    // chains the walls descend from).
    v.ext_v = b::chain_gate(b::ChainStep{
        c.tool.get(), &t.ext_tav, nullptr, nullptr, &c.chains, {},
        ext_rerun});

    // Step 4 — the subtract (two operands; the census in movement:
    // the surviving top and the born rim both held).
    v.sub_v = b::chain_gate(b::ChainStep{
        c.blind.get(), &t.sub_tav, c.box.get(), c.tool.get(),
        nullptr,
        {{c.top, b::Resolution::Kind::Resolved},
         {c.rim, b::Resolution::Kind::Resolved}},
        sub_rerun});

    // Step 5 — the fillet (one operand; the rim held DEAD — the
    // death the feature's own act is, the top still held alive).
    v.fil_v = b::chain_gate(b::ChainStep{
        c.result.get(), &t.fil_tav, c.blind.get(), nullptr, nullptr,
        {{c.rim, b::Resolution::Kind::Vanished},
         {c.top, b::Resolution::Kind::Resolved}},
        fil_rerun});

    return v;
}

} // namespace

// ===========================================================================
// THE BASELINE — the chain-5, the gate after every step, the drift
// zero against the re-run (the same recipe, built twice)
// ===========================================================================
TEST_CASE("K1 m5 the chain-5 baseline: the gate green after each of "
          "the five steps — the counts, the tavole's census, the "
          "held references, the drift zero against the re-run") {
    const Chain5 c;
    const Chain5Tavole t(c);
    REQUIRE(c.blind != nullptr);
    REQUIRE(c.result != nullptr);

    // THE RE-RUN — the same recipe, built again (ADR-0014's N-step
    // determinism clause: every digest bit-identical).
    const Chain5 again;
    const Chain5Tavole t2(again);

    const auto v = gate_chain5(
        c, t, b::chain_digest(*again.box),
        b::sketch_digest(again.chains),
        b::chain_digest(*again.tool), b::chain_digest(*again.blind),
        b::chain_digest(*again.result));

    // ---- Step 1: THE BOX — 8/12/6, the factory tavola all born.
    {
        const auto& g = v.box_v;
        CHECK(g.pass);
        REQUIRE(g.failures.empty());
        CHECK(g.not_applicable.empty());
        CHECK(g.v == 8);
        CHECK(g.e == 12);
        CHECK(g.f == 6);
        CHECK(g.h == 0);
        CHECK(g.chi == 2);
        CHECK(g.born == 26);   // 8 V + 12 E + 6 F
        CHECK(g.kept == 0);
        CHECK(g.absorbed == 0);
        CHECK(g.vanished == 0);
        CHECK(g.radial_two_opposite);
        CHECK(g.loops_closed);
        CHECK(g.chi_even_orientable);
        CHECK(g.census_by_name);
        CHECK(g.drift_zero);
        // The digest: the drift's own floor (the same box twice).
        CHECK(g.digest == b::chain_digest(*c.box));
    }

    // ---- Step 2: THE SKETCH — the profile's own door: closed, the
    // digest pure; the body predicates n/a BY NAME (three of them,
    // never silent).
    {
        const auto& g = v.sketch_v;
        CHECK(g.pass);
        REQUIRE(g.failures.empty());
        CHECK(g.loops_closed);
        CHECK(g.drift_zero);
        REQUIRE(g.not_applicable.size() == 3);
        CHECK(g.not_applicable[0] ==
              b::GateCheck::RadialTwoOpposite);
        CHECK(g.not_applicable[1] ==
              b::GateCheck::ChiEvenOrientable);
        CHECK(g.not_applicable[2] == b::GateCheck::CensusByName);
        CHECK(g.digest == b::sketch_digest(c.chains));
    }

    // ---- Step 3: THE EXTRUDE — 4/5/3, the seam-wired circle, the
    // FROM side resolving, the tavola born 12.
    {
        const auto& g = v.ext_v;
        CHECK(g.pass);
        REQUIRE(g.failures.empty());
        CHECK(g.not_applicable.empty());
        CHECK(g.v == 4);
        CHECK(g.e == 5);
        CHECK(g.f == 3);
        CHECK(g.h == 0);
        CHECK(g.chi == 2);
        CHECK(g.born == 12);   // 4 V + 5 E + 3 F
        CHECK(g.kept == 0);
        CHECK(g.absorbed == 0);
        CHECK(g.vanished == 0);
        CHECK(g.census_by_name);
        CHECK(g.drift_zero);
    }

    // ---- Step 4: THE SUBTRACT — the blind body 12/17/8, H = 1,
    // χ = 2, the 47-row tavola (kept 25 / absorbed 3 / born 9 /
    // vanished 10 — the u2-m4 golden, the gate's own arithmetic).
    {
        const auto& g = v.sub_v;
        CHECK(g.pass);
        REQUIRE(g.failures.empty());
        CHECK(g.not_applicable.empty());
        CHECK(g.v == 12);
        CHECK(g.e == 17);
        CHECK(g.f == 8);
        CHECK(g.h == 1);
        CHECK(g.chi == 2);
        CHECK(g.kept == 25);
        CHECK(g.absorbed == 3);
        CHECK(g.born == 9);
        CHECK(g.vanished == 10);
        CHECK(g.split == 0);
        CHECK(g.merged == 0);
        CHECK(g.radial_two_opposite);
        CHECK(g.loops_closed);
        CHECK(g.chi_even_orientable);
        CHECK(g.census_by_name);
        CHECK(g.drift_zero);
        // The volume — the u2 golden, all exact.
        const auto mp = b::mass_properties(
            *c.blind, CAD_0::math::Tolerance::linear(1e-9));
        CHECK(close(mp.volume, 8.0 - 0.375 * kPi));
        CHECK(mp.all_exact);
    }

    // ---- Step 5: THE FILLET — the rim filleted: 14/20/9, H = 1,
    // χ = 2, the 47-row tavola (kept 30 / absorbed 3 / born 10 /
    // vanished 4 — the m3 golden), the volume the ADR-0039 D5
    // number, the held rim DEAD with the cause, the top ALIVE.
    {
        const auto& g = v.fil_v;
        CHECK(g.pass);
        REQUIRE(g.failures.empty());
        CHECK(g.not_applicable.empty());
        CHECK(g.v == 14);
        CHECK(g.e == 20);
        CHECK(g.f == 9);
        CHECK(g.h == 1);
        CHECK(g.chi == 2);
        CHECK(g.kept == 30);
        CHECK(g.absorbed == 3);
        CHECK(g.born == 10);
        CHECK(g.vanished == 4);
        CHECK(g.split == 0);
        CHECK(g.merged == 0);
        CHECK(g.radial_two_opposite);
        CHECK(g.loops_closed);
        CHECK(g.chi_even_orientable);
        CHECK(g.census_by_name);
        CHECK(g.drift_zero);
        // The volume — the ADR-0039 D5 golden, all exact.
        const auto mp = b::mass_properties(
            *c.result, CAD_0::math::Tolerance::linear(1e-9));
        CHECK(close(mp.volume, 6.775059758517764, 1e-12));
        CHECK(close(mp.volume,
                    (8.0 - 0.375 * kPi) -
                        wedge_volume(0.5, 0.25), 1e-12));
        CHECK(mp.all_exact);
    }

    // THE RE-RUN'S OWN GATE — the second chain is as honest as the
    // first (the gate must not privilege the run it watched being
    // built): every step green on the re-run too, the digests
    // equal.
    {
        const auto v2 = gate_chain5(
            again, t2, b::chain_digest(*c.box),
            b::sketch_digest(c.chains), b::chain_digest(*c.tool),
            b::chain_digest(*c.blind),
            b::chain_digest(*c.result));
        CHECK(v2.box_v.pass);
        CHECK(v2.sketch_v.pass);
        CHECK(v2.ext_v.pass);
        CHECK(v2.sub_v.pass);
        CHECK(v2.fil_v.pass);
        CHECK(v2.box_v.digest == v.box_v.digest);
        CHECK(v2.sketch_v.digest == v.sketch_v.digest);
        CHECK(v2.ext_v.digest == v.ext_v.digest);
        CHECK(v2.sub_v.digest == v.sub_v.digest);
        CHECK(v2.fil_v.digest == v.fil_v.digest);
    }
}

// ===========================================================================
// THE CENSUS BY NAME — the classes D3 at the gate: the surviving,
// the born, the dead; the boundary refused; the closure's floor
// ===========================================================================
TEST_CASE("K1 m5 the census by name: the D3 classes in movement — "
          "the death real with its cause, the classes 2/3 refused "
          "by name, the closure violation named") {
    const Chain5 c;
    const Chain5Tavole t(c);
    REQUIRE(c.blind != nullptr);
    REQUIRE(c.result != nullptr);
    REQUIRE(!(c.rim == ShapeId{}));

    // THE DEAD ONE — the rim through the fillet: the tavola books it
    // vanished with the cause "replaced by the flank", and the
    // identity answers NOWHERE on the result (the death is real).
    {
        const auto g = b::chain_gate(b::ChainStep{
            c.result.get(), &t.fil_tav, c.blind.get(), nullptr,
            nullptr,
            {{c.rim, b::Resolution::Kind::Vanished}}, ""});
        CHECK(g.pass);
        CHECK(g.census_by_name);
        // The cause rides the tavola's row (the m3 golden, verbatim).
        bool cause_found = false;
        for (const auto& row : t.fil_tav.rows) {
            if (row.relation == b::Descent::Vanished &&
                row.entity == b::EntityKind::Edge &&
                row.shape_id == c.rim) {
                CHECK(row.cause == "replaced by the flank");
                cause_found = true;
            }
        }
        CHECK(cause_found);
    }

    // THE EXPECTED-DEAD ONE THAT STILL ANSWERS — the corruption the
    // census by name exists to catch: hold the rim ALIVE through
    // the fillet (the lie), and the gate names it.
    {
        const auto g = b::chain_gate(b::ChainStep{
            c.result.get(), &t.fil_tav, c.blind.get(), nullptr,
            nullptr,
            {{c.rim, b::Resolution::Kind::Resolved}}, ""});
        CHECK_FALSE(g.pass);
        REQUIRE(g.failures.size() >= 1);
        bool named = false;
        for (const auto& f : g.failures) {
            if (f.check == b::GateCheck::CensusByName &&
                f.message.find("books it VANISHED") !=
                    std::string::npos &&
                f.message.find("replaced by the flank") !=
                    std::string::npos) {
                named = true;
            }
        }
        CHECK(named);
    }

    // THE SURVIVING ONE held dead (the mirror lie): the top is kept
    // through the fillet — holding it dead is refused with the
    // tavola's own word.
    {
        const auto g = b::chain_gate(b::ChainStep{
            c.result.get(), &t.fil_tav, c.blind.get(), nullptr,
            nullptr,
            {{c.top, b::Resolution::Kind::Vanished}}, ""});
        CHECK_FALSE(g.pass);
        bool named = false;
        for (const auto& f : g.failures) {
            if (f.check == b::GateCheck::CensusByName &&
                f.message.find("books it alive") !=
                    std::string::npos) {
                named = true;
            }
        }
        CHECK(named);
    }

    // THE BOUNDARY — the classes 2/3 (RENAMED, AMBIGUOUS) have no
    // authored v1 instance: the gate REFUSES them by name (the
    // registered boundary, never a guess — ADR-0036 D3.2/D3.3).
    {
        bool threw = false;
        try {
            const auto g = b::chain_gate(b::ChainStep{
                c.result.get(), &t.fil_tav, c.blind.get(), nullptr,
                nullptr,
                {{c.top, b::Resolution::Kind::Renamed}}, ""});
            (void)g;
        } catch (const std::invalid_argument& e) {
            threw = true;
            const std::string what = e.what();
            CHECK(what.find("the registered v1 boundary") !=
                  std::string::npos);
            CHECK(what.find("reference_renamed") ==
                  std::string::npos);  // the code never guessed
        }
        REQUIRE(threw);
    }

    // THE CLOSURE'S FLOOR — a reference that finds no row at all is
    // a closure violation (D2's own invariant, the resolver's
    // floor), named as the defect it is — never a mute "not found".
    {
        const ShapeId orphan{FeatureId{99},
                             SubshapeRef{}.push(0), {}};
        const auto g = b::chain_gate(b::ChainStep{
            c.result.get(), &t.fil_tav, c.blind.get(), nullptr,
            nullptr,
            {{orphan, b::Resolution::Kind::Resolved}}, ""});
        CHECK_FALSE(g.pass);
        bool named = false;
        for (const auto& f : g.failures) {
            if (f.check == b::GateCheck::CensusByName &&
                f.message.find("no row at all") !=
                    std::string::npos &&
                f.message.find("closure violation") !=
                    std::string::npos) {
                named = true;
            }
        }
        CHECK(named);
    }
}

// ===========================================================================
// THE CHAINED FILLETS — the m6-correttiva-ter's sequence, gated at
// every step: the stamp that survives one link and dies the next
// ===========================================================================
TEST_CASE("K1 m5 the chained fillets: box -> fil#2 -> fil#3 gated "
          "at every step — the dead reference refusing, the kept "
          "stamp surviving the first link and dying the second") {
    // The m6ter's chain (the pattern reused, the golden's own
    // numbers: 8/12/6 -> 10/15/7 -> 12/18/8, all chi = 2).
    auto box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
    const ShapeId vertical2{FeatureId{1}, SubshapeRef{}.push(10),
                            {}};
    const ShapeId vertical0{FeatureId{1}, SubshapeRef{}.push(8),
                            {}};
    const b::FilletNaming ctx2{FeatureId{2}, &n::kBoxNaming};
    auto first = b::fillet_edge(*box, vertical2, 0.5, ctx2);
    const b::FilletNaming ctx3{FeatureId{3}, &n::kBoxNaming};
    auto result = b::fillet_edge(*first, vertical0, 0.5, ctx3);
    REQUIRE(first != nullptr);
    REQUIRE(result != nullptr);

    // THE GATE AT EVERY STEP — the box, the first fillet, the
    // second (the tavole derived beside each: the factories' and
    // the fillets' own).
    {
        const auto tav_box = b::factory_descent(*box);
        const auto g0 = b::chain_gate(b::ChainStep{
            box.get(), &tav_box, nullptr, nullptr, nullptr, {}, ""});
        CHECK(g0.pass);
        CHECK(g0.v == 8);
        CHECK(g0.e == 12);
        CHECK(g0.f == 6);
        CHECK(g0.born == 26);
    }
    {
        const auto tav1 = b::fillet_descent(*first, *box);
        // The first link: vertical2 dies (the feature's own act),
        // vertical0 survives as a KEPT STAMP (the MATCH class
        // through the link — the m6ter's own lesson).
        const auto g1 = b::chain_gate(b::ChainStep{
            first.get(), &tav1, box.get(), nullptr, nullptr,
            {{vertical2, b::Resolution::Kind::Vanished},
             {vertical0, b::Resolution::Kind::Resolved}},
            ""});
        CHECK(g1.pass);
        REQUIRE(g1.failures.empty());
        CHECK(g1.v == 10);
        CHECK(g1.e == 15);
        CHECK(g1.f == 7);
        CHECK(g1.h == 0);
        CHECK(g1.chi == 2);
        // The m2 golden's census, by the tavola's own rows: faces
        // out 7 (2 kept + 4 absorbed + 1 born), edges out 15 (7
        // kept + 4 absorbed + 4 born), vertices out 10 (6 kept + 4
        // born), the vanished 1 edge + 2 vertices — 35 rows.
        CHECK(g1.kept == 15);
        CHECK(g1.absorbed == 8);
        CHECK(g1.born == 9);
        CHECK(g1.vanished == 3);
    }
    {
        const auto tav2 = b::fillet_descent(*result, *first);
        // The second link: the KEPT STAMP of the first link is now
        // the meal (vertical0 dies at fil#3), and the first flank
        // survives as a kept row (the chain's own novelty).
        const auto g2 = b::chain_gate(b::ChainStep{
            result.get(), &tav2, first.get(), nullptr, nullptr,
            {{vertical0, b::Resolution::Kind::Vanished}},
            ""});
        CHECK(g2.pass);
        REQUIRE(g2.failures.empty());
        CHECK(g2.v == 12);
        CHECK(g2.e == 18);
        CHECK(g2.f == 8);
        CHECK(g2.h == 0);
        CHECK(g2.chi == 2);
        // The m6ter golden's census, by the tavola's own rows:
        // faces out 8 (3 kept + 4 absorbed + 1 born — the first
        // flank now a KEPT row, the chain's own novelty), edges
        // out 18 (10 kept + 4 absorbed + 4 born), vertices out 12
        // (8 kept + 4 born), the vanished 1 edge + 2 vertices — 41
        // rows.
        CHECK(g2.kept == 21);
        CHECK(g2.absorbed == 8);
        CHECK(g2.born == 9);
        CHECK(g2.vanished == 3);
        // The volume: two disjoint wedges, the closed form.
        const auto mp = b::mass_properties(
            *result, CAD_0::math::Tolerance::linear(1e-9));
        CHECK(close(mp.volume, 7.0 + kPi / 4.0));
        CHECK(mp.all_exact);
    }

    // THE DRIFT on the chain: the same two fillets built again,
    // every digest bit-identical (the m6ter's own purity clause,
    // grown into the gate's instrument).
    {
        auto box2 = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        auto first2 = b::fillet_edge(*box2, vertical2, 0.5, ctx2);
        auto result2 = b::fillet_edge(*first2, vertical0, 0.5,
                                      ctx3);
        const auto tav2 = b::fillet_descent(*result, *first);
        const auto g = b::chain_gate(b::ChainStep{
            result.get(), &tav2, first.get(), nullptr, nullptr, {},
            b::chain_digest(*result2)});
        CHECK(g.pass);
        CHECK(g.drift_zero);
        CHECK(g.digest == b::chain_digest(*result2));
    }
}

// ===========================================================================
// THE SILENT EDIT — the chain re-run at r = 0.125: the upstream
// steps' digests UNCHANGED, the edited step's changed, the counts
// fixed, the volume the closed form at the new r, the rim
// reference resolving class 1 through the edit
// ===========================================================================
TEST_CASE("K1 m5 the silent edit: the chain at r = 0.125 — the "
          "upstream digests unchanged, the edited step's changed, "
          "the counts fixed 14/20/9, the rim reference resolved") {
    const Chain5 edited(0.125);
    const Chain5Tavole t(edited);
    REQUIRE(edited.result != nullptr);

    // The counts FIXED for every in-band r (the edit contract's own
    // clause): 14/20/9, H = 1, chi = 2 — the gate green throughout.
    const auto v = gate_chain5(edited, t, "", "", "", "", "");
    CHECK(v.box_v.pass);
    CHECK(v.sketch_v.pass);
    CHECK(v.ext_v.pass);
    CHECK(v.sub_v.pass);
    CHECK(v.fil_v.pass);
    CHECK(v.fil_v.v == 14);
    CHECK(v.fil_v.e == 20);
    CHECK(v.fil_v.f == 9);
    CHECK(v.fil_v.h == 1);
    CHECK(v.fil_v.chi == 2);

    // THE EDIT'S OWN VISIBILITY — the discrimination the drift
    // predicate exists to make: the upstream steps (the box, the
    // sketch, the extrude, the subtract — the radius edit touches
    // ONLY the fillet's recipe) reproduce BIT-IDENTICALLY, and the
    // edited step's digest CHANGES (an edit is not a drift; a drift
    // is not an edit — the gate tells them apart by the recipe).
    const Chain5 original(0.25);
    CHECK(v.box_v.digest == b::chain_digest(*original.box));
    CHECK(v.sketch_v.digest == b::sketch_digest(original.chains));
    CHECK(v.ext_v.digest == b::chain_digest(*original.tool));
    CHECK(v.sub_v.digest == b::chain_digest(*original.blind));
    CHECK(v.fil_v.digest != b::chain_digest(*original.result));

    // The volume — the D5 closed form at the new r (the click-test
    // protocol's own number, quoted).
    {
        const auto mp = b::mass_properties(
            *edited.result, CAD_0::math::Tolerance::linear(1e-9));
        CHECK(close(mp.volume, 6.810780258845277, 1e-12));
        CHECK(close(mp.volume,
                    (8.0 - 0.375 * kPi) -
                        wedge_volume(0.5, 0.125), 1e-12));
        CHECK(mp.all_exact);
    }

    // THE RIM REFERENCE across the edit — class 1, the m3 pattern
    // consumed: the old generation's tavola (the subtract's), the
    // new body (the re-materialized blind — the radius edit does
    // not touch the subtract's recipe), the no-shift declaration.
    // The answer: reference_resolved, the feature re-runs SILENTLY.
    {
        const b::DescentTable old_tav = b::subtract_descent(
            *original.blind, *original.box, *original.tool);
        const n::OperandBinding sub_bindings[2] = {
            {FeatureId{1}, &n::kBoxNaming},
            {FeatureId{3}, &n::kExtNaming}};
        const n::OperationNaming sub_decl{
            n::FeatureKind::Subtract, {}, n::kSubEdgeRoles,
            {sub_bindings, 2}};
        const b::Generation old_gen{nullptr, &old_tav, &sub_decl,
                                    false};
        const b::Generation new_gen{edited.blind.get(), nullptr,
                                    &sub_decl, false};
        const b::EditContext edit{/*roots_no_shift=*/true, {}};
        const auto r = b::resolve(original.rim, old_gen, new_gen,
                                  edit);
        CHECK(r.kind == b::Resolution::Kind::Resolved);
        // The census by name holds the rim through the edited
        // subtract as it held through the original: the identity
        // answers on the new blind.
        const auto g = b::chain_gate(b::ChainStep{
            edited.blind.get(), &t.sub_tav, edited.box.get(),
            edited.tool.get(), nullptr,
            {{original.rim, b::Resolution::Kind::Resolved}}, ""});
        CHECK(g.pass);
    }
}

// ===========================================================================
// THE CONTROPROVA — the naive red, per component: each corruption
// caught WITH ITS OWN NAMED MESSAGE; the honest chain green above
// ===========================================================================
TEST_CASE("K1 m5 the gate's controprova: the lost co-edge, the open "
          "loop, the odd chi, the lost name, the drifting digest — "
          "each caught with its own named message") {
    // Each corruption starts from a FRESH blind body (the mutations
    // must not pollute each other) with the tavola derived from the
    // honest state.
    const auto fresh = [] {
        return Chain5(0.25);
    };

    // ---- (1) THE LOST CO-EDGE: one radial entry removed — the
    // radial predicate names the edge and the count.
    {
        Chain5 c = fresh();
        REQUIRE(c.blind != nullptr);
        const auto tav = b::subtract_descent(*c.blind, *c.box,
                                             *c.tool);
        const auto* kept_ce = c.blind->edges[3].coedges()[0];
        c.blind->edges[3].clear_coedges();
        c.blind->edges[3].add_coedge(kept_ce);
        const auto g = b::chain_gate(b::ChainStep{
            c.blind.get(), &tav, c.box.get(), c.tool.get(), nullptr,
            {}, ""});
        CHECK_FALSE(g.pass);
        CHECK_FALSE(g.radial_two_opposite);
        bool named = false;
        for (const auto& f : g.failures) {
            if (f.check == b::GateCheck::RadialTwoOpposite &&
                f.message.find("edge 3") != std::string::npos &&
                f.message.find("carries 1 co-edges") !=
                    std::string::npos &&
                f.message.find("exactly two required") !=
                    std::string::npos) {
                named = true;
            }
        }
        CHECK(named);
    }

    // ---- (2) THE OPEN LOOP: an endpoint moved — the loop
    // predicate names the face, the loop, the break and the two
    // vertices that do not meet.
    {
        Chain5 c = fresh();
        REQUIRE(c.blind != nullptr);
        const auto tav = b::subtract_descent(*c.blind, *c.box,
                                             *c.tool);
        c.blind->edges[3].set_vertex(1, 99);
        const auto g = b::chain_gate(b::ChainStep{
            c.blind.get(), &tav, c.box.get(), c.tool.get(), nullptr,
            {}, ""});
        CHECK_FALSE(g.pass);
        CHECK_FALSE(g.loops_closed);
        bool named = false;
        for (const auto& f : g.failures) {
            if (f.check == b::GateCheck::LoopsClosed &&
                f.message.find("the open loop") !=
                    std::string::npos &&
                f.message.find("loop 0 breaks at") !=
                    std::string::npos &&
                f.message.find("end vertex 99") !=
                    std::string::npos) {
                named = true;
            }
        }
        CHECK(named);
    }

    // ---- (3) THE ODD CHI: an isolated vertex appended — V grows
    // by one, chi flips odd — and the SAME unbooked entity is the
    // census's own catch (a vertex no row books): the corruption
    // is named TWICE, by the parity and by the closure, two names
    // for one disease. Radial and loops stay green — the surgery
    // touches no co-edge and no loop.
    {
        Chain5 c = fresh();
        REQUIRE(c.blind != nullptr);
        const auto tav = b::subtract_descent(*c.blind, *c.box,
                                             *c.tool);
        c.blind->vertices.push_back(b::Vertex({9.0, 9.0, 9.0}));
        const auto g = b::chain_gate(b::ChainStep{
            c.blind.get(), &tav, c.box.get(), c.tool.get(), nullptr,
            {}, ""});
        CHECK_FALSE(g.pass);
        CHECK(g.radial_two_opposite);   // untouched — the surgery
        CHECK(g.loops_closed);          // is surgical
        CHECK_FALSE(g.chi_even_orientable);
        CHECK_FALSE(g.census_by_name);
        REQUIRE(g.failures.size() == 2);
        bool chi_named = false, census_named = false;
        for (const auto& f : g.failures) {
            if (f.check == b::GateCheck::ChiEvenOrientable) {
                chi_named =
                    f.message.find(
                        "chi = V-E+F-H = 3") !=
                        std::string::npos &&
                    f.message.find("ODD") != std::string::npos;
            }
            if (f.check == b::GateCheck::CensusByName) {
                census_named =
                    f.message.find("does not close") !=
                    std::string::npos;
            }
        }
        CHECK(chi_named);
        CHECK(census_named);
    }

    // ---- (4) THE LOST NAME: the wall's stamp wiped on the result
    // — the census by name sees the row and the body disagree (the
    // name is lost or laundered), the address-side closure alone
    // would never see it.
    {
        Chain5 c = fresh();
        REQUIRE(c.blind != nullptr);
        const auto tav = b::subtract_descent(*c.blind, *c.box,
                                             *c.tool);
        // The wall (the tool's lateral, absorbed) — found BY NAME,
        // never by a magic index.
        bool wiped = false;
        for (auto& face : c.blind->solid.shell_mut().faces_mut()) {
            if (face.name() == "ext#3:lat(0)") {
                face.set_shape_id(ShapeId{});
                face.set_name("");
                wiped = true;
            }
        }
        REQUIRE(wiped);
        const auto g = b::chain_gate(b::ChainStep{
            c.blind.get(), &tav, c.box.get(), c.tool.get(), nullptr,
            {}, ""});
        CHECK_FALSE(g.pass);
        CHECK_FALSE(g.census_by_name);
        bool named = false;
        for (const auto& f : g.failures) {
            if (f.check == b::GateCheck::CensusByName &&
                f.message.find("lost or laundered") !=
                    std::string::npos &&
                f.message.find("ext#3:lat(0)") !=
                    std::string::npos) {
                named = true;
            }
        }
        CHECK(named);
    }

    // ---- (5) THE DRIFTING DIGEST: one vertex position nudged on
    // the run the gate watches — the re-run's digest (the honest
    // body's) diverges at the vertex's own line, named.
    {
        Chain5 c = fresh();
        REQUIRE(c.blind != nullptr);
        const auto tav = b::subtract_descent(*c.blind, *c.box,
                                             *c.tool);
        const std::string honest = b::chain_digest(*c.blind);
        const Vec3d p = c.blind->vertices[0].position();
        c.blind->vertices[0].set_position(
            {p.x + 1e-9, p.y, p.z});
        const auto g = b::chain_gate(b::ChainStep{
            c.blind.get(), &tav, c.box.get(), c.tool.get(), nullptr,
            {}, honest});
        CHECK_FALSE(g.pass);
        CHECK_FALSE(g.drift_zero);
        REQUIRE(g.failures.size() >= 1);
        bool named = false;
        for (const auto& f : g.failures) {
            if (f.check == b::GateCheck::DriftZero &&
                f.message.find("differ at line 2") !=
                    std::string::npos &&
                f.message.find("v0") != std::string::npos) {
                named = true;
            }
        }
        CHECK(named);
    }
}
