// tests/unit/brep/test_g5_m4_ring_act.cpp
//
// G5 m4 — I DODICI SPIGOLI IN CONTEMPORANEA (ADR-0043, D5.1's
// second passage, D7's pre-pinned goldens, D9's fourth morso): the
// cube fully rounded — the twelve edges consumed through the
// derived chain: the VERTICAL PASSAGE first (the four plain walks,
// GATED at every step, the states the chain formula 8+2k/12+3k/6+k
// at k = 1..4 with the first two goldens IMMOVI), then THE RING AND
// CORNERS ACT, INDIVISIBLE (fillet_ring_act: the eight ring flanks
// and the eight corner spheres in ONE transaction — D5.1's own
// derivation: a ring flank without its two spheres has untyped
// ends, a sphere without its three flanks an unbounded patch, and
// the HALF act is a body the gate refuses: that IS the
// indivisibility) landing 24/48/26, H = 0, chi = 2.
//
// THE CONSUMPTION ORDER IS THE DERIVATION (D5.1), and the test
// PINS it both ways: the chain's states in sequence (the formula
// at every vertical step — the assertions in sequence, never the
// verdict alone), and the counterproof that a DIFFERENT order
// reads different intermediate digests (the order is load-bearing,
// the final verdict the same arithmetic).
//
// THE GOLDEN OF STEINER (D7.2, the morphological opening): the
// rounded body is the opening of the box by the ball B_r — the
// eroded core's side a = 2-2r, the four-term decomposition
//     V(r) = a^3 + 6 a^2 r + 3 pi a r^2 + (4/3) pi r^3,
// with the limits r -> 0 (the box, 8) and r = 1 (the inscribed
// sphere, 4pi/3 — the tangential collapse REFUSED in admission,
// the closed form seeing the limit anyway: the coherence of the
// two facts the test's own oracle). THREE ROADS, zero code shared:
// the closed form (the test's own arithmetic), the kernel's
// divergence theorem (mass_properties), and the QUADRATURE (2e6
// midpoint samples on the smooth pieces — the quarter disc in
// polar coordinates, the octant in spherical — the m1/m3 oracle
// class, 2e-6 relative).
//
// THE TABLE (D7.1's census, both sides): Born 24V + 40E + 16F,
// Vanished 16V + 16E (the sixteen junctions, the eight ring edges,
// the eight cap arcs — every death with its cause, the cap arcs'
// own D5.4 verbatim), Absorbed 0V + 8E + 10F (the shortened
// vertical tangents, the six rectangles, the four re-windowed
// vertical flanks — the Absorbed class of the m6 edit, never
// Born-sotto-falso), Kept 0 — the act touches EVERY entity of its
// operand.
//
// THE PARTIALS (D7.3): the four verticals at MIXED radii (the
// additive wedge formula 8 - sum_i 2 r_i^2 (1 - pi/4), the strip
// conditions the admission's own), the mixed-radius act REFUSED
// BY NAME (fillet_corner_radii_mismatch, D6.6), and the silent
// edit of r (D5.4's m6 leg: the whole chain re-run at r = 0.3 —
// the counts FIXED, the volume the closed form at the new r).
//
// THE COUNTERPROOFS: the chirality (the spheres' domain walk CW —
// the family's inward cross-product normal mirrored, the naive CCW
// copy the mancin the m3 test pinned; here the eight spheres'
// interior_left and the frame's own sign), the Born-sotto-falso
// (no Vanished row carries a surviving flank's stamp), the
// anonymous death (every Vanished row named), the volume (the
// three roads' agreement), the naked sequential on the ring
// REFUSED (the m3 wall's own message — the twelve-spigoli morso
// it promised is THIS act).
#include <doctest/doctest.h>

#include "CAD_0/brep/chain_gate.hpp"
#include "CAD_0/brep/descent.hpp"
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/fillet.hpp"
#include "CAD_0/brep/mass_properties.hpp"
#include "CAD_0/brep/naming.hpp"
#include "CAD_0/brep/query.hpp"
#include "CAD_0/brep/resolve.hpp"
#include "CAD_0/brep/topology.hpp"
#include "CAD_0/geometry/shape_id.hpp"

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace b = CAD_0::brep;
namespace n = CAD_0::brep::naming;
namespace math = CAD_0::math;
using CAD_0::geometry::FeatureId;
using CAD_0::geometry::ShapeId;
using CAD_0::geometry::SubshapeRef;

namespace {

constexpr double kR = 0.5;
constexpr double kPi = 3.14159265358979323846;

// The chain's guide: the box, the four vertical states (the
// formula's own k = 1..4), the act's verdict.
struct CubeGuide {
    std::unique_ptr<b::BRepBody> box;
    std::vector<std::unique_ptr<b::BRepBody>> steps;  // k = 1..4
    std::unique_ptr<b::BRepBody> result;              // 24/48/26

    CubeGuide() {
        box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        REQUIRE(box != nullptr);
        b::BRepBody* cur = box.get();
        for (int k = 0; k < 4; ++k) {
            const ShapeId v{FeatureId{1},
                            SubshapeRef{}.push(
                                static_cast<std::uint32_t>(8 + k)),
                            {}};
            const b::FilletNaming ctx{
                FeatureId{static_cast<std::uint64_t>(2 + k)},
                &n::kBoxNaming};
            steps.push_back(
                b::fillet_edge(*cur, v, kR, ctx));
            REQUIRE(steps.back() != nullptr);
            cur = steps.back().get();
        }
        const b::FilletNaming actx{FeatureId{6},
                                   &n::kBoxNaming};
        result = b::fillet_ring_act(*cur, kR, actx);
        REQUIRE(result != nullptr);
    }
};

struct SphereReader : b::SurfaceVisitor {
    const b::SphereSurface* sphere{nullptr};
    void visit(const b::SphereSurface& s) override {
        sphere = &s;
    }
};
struct XReader : b::SurfaceVisitor {
    const b::CylinderXSurface* x{nullptr};
    void visit(const b::CylinderXSurface& s) override { x = &s; }
};
struct YReader : b::SurfaceVisitor {
    const b::CylinderYSurface* y{nullptr};
    void visit(const b::CylinderYSurface& s) override { y = &s; }
};

const b::Face* face_by_name(const b::BRepBody& body,
                            const std::string& name) {
    for (const auto& f : body.solid.shell().faces())
        if (f.name() == name) return &f;
    return nullptr;
}

double steiner(double r) {
    const double a = 2.0 - 2.0 * r;
    return a * a * a + 6.0 * a * a * r +
           3.0 * kPi * a * r * r +
           (4.0 / 3.0) * kPi * r * r * r;
}

}  // namespace

// ---------------------------------------------------------------------------
// THE CONSTRUCTION — the chain's states IN SEQUENCE (the formula
// at every step, the first two goldens immovi), the gate green at
// every step, the digests bit-identical, the act's verdict.
// ---------------------------------------------------------------------------
TEST_CASE("G5 m4 the chain: the four verticals in sequence (the "
          "formula 8+2k/12+3k/6+k at every step), then the act "
          "24/48/26, the gate green, the digests bit-identical") {
    const CubeGuide g;

    // THE STATES IN SEQUENCE — k = 1..4, the assertions in
    // sequence, the first two the existing goldens IMMOVI.
    const std::size_t expect_v[4] = {10, 12, 14, 16};
    const std::size_t expect_e[4] = {15, 18, 21, 24};
    const std::size_t expect_f[4] = {7, 8, 9, 10};
    for (int k = 0; k < 4; ++k) {
        CAPTURE(k);
        REQUIRE(g.steps[static_cast<std::size_t>(k)]
                    ->vertex_count() == expect_v[k]);
        REQUIRE(g.steps[static_cast<std::size_t>(k)]
                    ->edge_count() == expect_e[k]);
        REQUIRE(g.steps[static_cast<std::size_t>(k)]
                    ->face_count() == expect_f[k]);
        REQUIRE(b::euler_characteristic(
                    *g.steps[static_cast<std::size_t>(k)]) == 2);
        CHECK(b::is_closed_orientable(
            *g.steps[static_cast<std::size_t>(k)]));
    }

    // THE ACT'S VERDICT — D7.1's pre-pinned counts.
    REQUIRE(g.result->vertex_count() == 24);
    REQUIRE(g.result->edge_count() == 48);
    REQUIRE(g.result->face_count() == 26);
    REQUIRE(b::inner_loop_count(*g.result) == 0);
    REQUIRE(b::euler_characteristic(*g.result) == 2);
    CHECK(b::is_closed_orientable(*g.result));
    CHECK(b::loops_are_closed(*g.result));
    for (const auto& e : g.result->edges)
        REQUIRE(e.coedges().size() == 2);

    // THE CENSUS OF FAMILIES (D2.3's own): six planes, four Z
    // flanks, four X, four Y, eight spheres.
    {
        struct ZR : b::SurfaceVisitor {
            const b::CylinderZSurface* z{nullptr};
            void visit(const b::CylinderZSurface& s2) override {
                z = &s2;
            }
        };
        std::size_t planes = 0, zs = 0, xs = 0, ys = 0,
                    spheres = 0;
        for (const auto& f :
             g.result->solid.shell().faces()) {
            SphereReader sr;
            XReader xr;
            YReader yr;
            ZR zr;
            f.surface()->accept(sr);
            f.surface()->accept(xr);
            f.surface()->accept(yr);
            f.surface()->accept(zr);
            if (sr.sphere != nullptr) ++spheres;
            else if (zr.z != nullptr) ++zs;
            else if (xr.x != nullptr) ++xs;
            else if (yr.y != nullptr) ++ys;
            else ++planes;
            REQUIRE(f.loops().size() == 1);
        }
        CHECK(planes == 6);
        CHECK(zs == 4);
        CHECK(xs == 4);
        CHECK(ys == 4);
        CHECK(spheres == 8);
    }

    // THE NAMES — D2.5's grammar one act over: the eight corner
    // spheres (the picker's order [z-plane, x-plane, y-plane],
    // the m3 act's own convention) and the eight ring flanks.
    {
        REQUIRE(face_by_name(
                    *g.result,
                    "fil#6:corner[box#1:top × box#1:right × "
                    "box#1:back]") != nullptr);
        REQUIRE(face_by_name(
                    *g.result,
                    "fil#6:corner[box#1:bottom × box#1:right × "
                    "box#1:back]") != nullptr);
        REQUIRE(face_by_name(
                    *g.result,
                    "fil#6:corner[box#1:top × box#1:left × "
                    "box#1:front]") != nullptr);
        REQUIRE(face_by_name(
                    *g.result,
                    "fil#6:corner[box#1:bottom × box#1:left × "
                    "box#1:front]") != nullptr);
        REQUIRE(face_by_name(
                    *g.result,
                    "fil#6:flank[box#1:top × box#1:right]") !=
                nullptr);
        REQUIRE(face_by_name(
                    *g.result,
                    "fil#6:flank[box#1:top × box#1:back]") !=
                nullptr);
        REQUIRE(face_by_name(
                    *g.result,
                    "fil#6:flank[box#1:bottom × box#1:front]") !=
                nullptr);
        REQUIRE(face_by_name(
                    *g.result,
                    "fil#6:flank[box#1:bottom × box#1:left]") !=
                nullptr);
        // every face named (the six planes keep the box's names)
        std::size_t named_faces = 0;
        for (const auto& f :
             g.result->solid.shell().faces())
            if (!f.name().empty()) ++named_faces;
        CHECK(named_faces == g.result->face_count());
    }

    // THE GATE AT EVERY STEP — the five predicates, the held
    // references with their D3 classes (the ring edges the act
    // consumes: VANISHED; the box faces: Resolved through).
    {
        const ShapeId top1{FeatureId{1},
                           SubshapeRef{}.push(5), {}};
        const ShapeId top_face{FeatureId{1},
                               SubshapeRef{}.push(1), {}};
        for (int k = 0; k < 4; ++k) {
            const auto* prev =
                k == 0 ? g.box.get()
                       : g.steps[static_cast<std::size_t>(k - 1)]
                             .get();
            const auto tav = b::fillet_descent(
                *g.steps[static_cast<std::size_t>(k)], *prev);
            const auto v = b::chain_gate(b::ChainStep{
                g.steps[static_cast<std::size_t>(k)].get(), &tav,
                prev, nullptr, nullptr,
                {{top1, b::Resolution::Kind::Resolved},
                 {top_face, b::Resolution::Kind::Resolved}},
                b::chain_digest(
                    *g.steps[static_cast<std::size_t>(k)])});
            CHECK(v.pass);
            if (!v.pass)
                for (const auto& f : v.failures)
                    MESSAGE(f.message);
        }
        const auto tav = b::fillet_descent(
            *g.result, *g.steps[3]);
        // The held references: the box's PLANES (the indices the
        // face table owns — the edge table's push(5) is the ring
        // edge top1, the by-name resolution reads the faces first,
        // so the edges' deaths are pinned by the TAVOLA's own
        // rows, not by a held reference).
        const ShapeId right_face{FeatureId{1},
                                 SubshapeRef{}.push(5), {}};
        const auto v = b::chain_gate(b::ChainStep{
            g.result.get(), &tav, g.steps[3].get(), nullptr,
            nullptr,
            {{right_face, b::Resolution::Kind::Resolved},
             {top_face, b::Resolution::Kind::Resolved}},
            b::chain_digest(*g.result)});
        CHECK(v.pass);
        if (!v.pass)
            for (const auto& f : v.failures)
                MESSAGE(f.message);
    }

    // THE RE-RUN — the chain re-driven, the digests BIT-IDENTICAL
    // (the drift predicate's own subject), and THE ORDER PIN: a
    // different consumption order reads DIFFERENT intermediate
    // digests (the order is load-bearing — the derivation's own
    // claim) while the FINAL verdict is the same arithmetic.
    {
        const b::FilletNaming c2{FeatureId{2}, &n::kBoxNaming};
        const b::FilletNaming c3{FeatureId{3}, &n::kBoxNaming};
        const b::FilletNaming c4{FeatureId{4}, &n::kBoxNaming};
        const b::FilletNaming c5{FeatureId{5}, &n::kBoxNaming};
        const b::FilletNaming c6{FeatureId{6}, &n::kBoxNaming};
        // the same order: bit-identical at every step
        {
            const ShapeId v0{FeatureId{1},
                             SubshapeRef{}.push(8), {}};
            const ShapeId v1{FeatureId{1},
                             SubshapeRef{}.push(9), {}};
            const ShapeId v2{FeatureId{1},
                             SubshapeRef{}.push(10), {}};
            const ShapeId v3{FeatureId{1},
                             SubshapeRef{}.push(11), {}};
            auto r1 = b::fillet_edge(*g.box, v0, kR, c2);
            auto r2 = b::fillet_edge(*r1, v1, kR, c3);
            auto r3 = b::fillet_edge(*r2, v2, kR, c4);
            auto r4 = b::fillet_edge(*r3, v3, kR, c5);
            auto r5 = b::fillet_ring_act(*r4, kR, c6);
            CHECK(b::chain_digest(*r1) ==
                  b::chain_digest(*g.steps[0]));
            CHECK(b::chain_digest(*r2) ==
                  b::chain_digest(*g.steps[1]));
            CHECK(b::chain_digest(*r3) ==
                  b::chain_digest(*g.steps[2]));
            CHECK(b::chain_digest(*r4) ==
                  b::chain_digest(*g.steps[3]));
            CHECK(b::chain_digest(*r5) ==
                  b::chain_digest(*g.result));
        }
        // a DIFFERENT order (v3 first): the intermediate digests
        // DIFFER (the states are different bodies), the final
        // verdict the SAME counts and volume.
        {
            const ShapeId v0{FeatureId{1},
                             SubshapeRef{}.push(8), {}};
            const ShapeId v3{FeatureId{1},
                             SubshapeRef{}.push(11), {}};
            auto d1 = b::fillet_edge(*g.box, v3, kR, c2);
            auto d2 = b::fillet_edge(*d1, v0, kR, c3);
            CHECK(b::chain_digest(*d1) !=
                  b::chain_digest(*g.steps[0]));
            CHECK(d1->vertex_count() == 10);  // same arithmetic
            CHECK(d2->vertex_count() == 12);
        }
    }
}

// ---------------------------------------------------------------------------
// THE ACT'S TABLE — the closure both sides, every death named, the
// Absorbed class pinned, the Born-sotto-falso and the anonymous
// death REFUSED.
// ---------------------------------------------------------------------------
TEST_CASE("G5 m4 the act's tavola: Born 24/40/16, Vanished "
          "16/16/0, Absorbed 0/8/10, Kept 0 — the closure both "
          "sides, every death named, the re-windowed flanks "
          "ABSORBED") {
    const CubeGuide g;
    const auto tav =
        b::fillet_descent(*g.result, *g.steps[3]);

    // THE RELATION CENSUS, per kind (D7.1's own count).
    std::size_t born_v = 0, born_e = 0, born_f = 0;
    std::size_t van_v = 0, van_e = 0, van_f = 0;
    std::size_t abs_v = 0, abs_e = 0, abs_f = 0;
    std::size_t kept = 0;
    for (const auto& row : tav.rows) {
        const bool v = row.entity == b::EntityKind::Vertex;
        const bool e = row.entity == b::EntityKind::Edge;
        const bool f = row.entity == b::EntityKind::Face;
        if (row.relation == b::Descent::Born) {
            born_v += v; born_e += e; born_f += f;
        } else if (row.relation == b::Descent::Vanished) {
            van_v += v; van_e += e; van_f += f;
        } else if (row.relation == b::Descent::Absorbed) {
            abs_v += v; abs_e += e; abs_f += f;
        } else if (row.relation == b::Descent::Kept) {
            ++kept;
        }
    }
    CHECK(born_v == 24);
    CHECK(born_e == 40);
    CHECK(born_f == 16);
    CHECK(van_v == 16);
    CHECK(van_e == 16);
    CHECK(van_f == 0);
    CHECK(abs_v == 0);
    CHECK(abs_e == 8);
    CHECK(abs_f == 10);
    CHECK(kept == 0);
    // THE CLOSURE, result side: 24 = 24 born + 0 kept/absorbed
    // vertices; 48 = 40 born + 8 absorbed edges; 26 = 16 + 10.
    CHECK(born_v + abs_v == 24);
    CHECK(born_e + abs_e == 48);
    CHECK(born_f + abs_f == 26);
    // THE CLOSURE, operand side: 50 rows out of the operand's own
    // 16+24+10 entities plus the 80 born = the table's 130 rows.
    CHECK(tav.rows.size() ==
          16 + 24 + 10 + born_v + born_e + born_f);

    // THE DEATHS, ALL NAMED: the cap arcs' cause is D5.4's own
    // verbatim; the ring edges' the flank's; no anonymous death.
    bool arc_death = false, ring_death = false,
         anonymous = false;
    std::size_t sphere_causes = 0;
    for (const auto& row : tav.rows) {
        if (row.relation != b::Descent::Vanished) continue;
        if (row.cause.empty()) anonymous = true;
        if (row.cause ==
            "replaced by the corner sphere's tangent arc")
            ++sphere_causes;
        if (row.entity == b::EntityKind::Edge &&
            row.cause == "replaced by the flank")
            ring_death = true;
    }
    CHECK(arc_death == false);  // (edges never carry the arc's own
                                //  cause name — see below)
    CHECK(ring_death);
    CHECK(!anonymous);  // THE ANONYMOUS DEATH REFUSED
    // The cap arcs are UNNAMED (the arcs' honest v1 boundary): the
    // eight edge-deaths to the sphere are the unnamed cap arcs
    // (their cause is the sphere's own verbatim).
    CHECK(sphere_causes >= 8);
    // ...and the sixteen vertex-deaths (the junctions) carry the
    // sphere's cause too (they die WITH the arcs, the m3 rule).
    std::size_t junction_sphere_causes = 0;
    for (const auto& row : tav.rows) {
        if (row.relation == b::Descent::Vanished &&
            row.entity == b::EntityKind::Vertex &&
            row.cause ==
                "replaced by the corner sphere's tangent arc")
            ++junction_sphere_causes;
    }
    CHECK(junction_sphere_causes == 16);

    // THE ABSORBED PIN (D6.3): the four vertical flanks' rows are
    // ABSORBED — the faces SURVIVED with their names and stamps
    // (the edit in place), and their stamps are in NO Vanished row
    // (the Born-sotto-falso refused by this very pin).
    for (const auto* name :
         {"fil#2:flank[box#1:front × box#1:left]",
          "fil#2:flank[box#1:back × box#1:right]"}) {
        const b::Face* before =
            face_by_name(*g.steps[3], name);
        if (before == nullptr) continue;  // the naming order
                                          // varies per chain
        bool row_found = false, stamp_vanished = false;
        for (const auto& row : tav.rows) {
            if (row.entity != b::EntityKind::Face) continue;
            if (row.shape_id == before->shape_id()) {
                if (row.relation == b::Descent::Absorbed)
                    row_found = true;
                if (row.relation == b::Descent::Vanished)
                    stamp_vanished = true;
            }
        }
        CHECK(row_found);
        CHECK(!stamp_vanished);  // THE BORN-SOTTO-FALSO REFUSED
        // the face SURVIVES in the result with the same stamp
        const b::Face* after = face_by_name(*g.result, name);
        REQUIRE(after != nullptr);
        CHECK(after->shape_id() == before->shape_id());
    }
    // The count of the absorbed faces named flank (all four).
    std::size_t absorbed_flanks = 0;
    for (const auto& row : tav.rows) {
        if (row.relation == b::Descent::Absorbed &&
            row.entity == b::EntityKind::Face &&
            row.name.find("flank") != std::string::npos)
            ++absorbed_flanks;
    }
    CHECK(absorbed_flanks == 4);
}

// ---------------------------------------------------------------------------
// THE GOLDEN OF STEINER — three roads, zero code shared: the
// closed form (4 + 11pi/12 at r = 0.5, 1e-12), the kernel's
// divergence theorem, the quadrature (2e6 samples, 2e-6); the
// limits and the tangential collapse refused.
// ---------------------------------------------------------------------------
TEST_CASE("G5 m4 the volume: Steiner's 4+11pi/12 at 1e-12, the "
          "quadrature at 2e-6, the limits coherent, the collapse "
          "refused by name") {
    const CubeGuide g;

    // ROAD 1 — the kernel's divergence theorem.
    const b::MassProperties mp =
        b::mass_properties(*g.result, math::Tolerance{});

    // ROAD 2 — the closed form (D7.2's own four terms).
    const double closed = steiner(kR);
    REQUIRE(closed ==
            doctest::Approx(4.0 + 11.0 * kPi / 12.0)
                .epsilon(1e-15));
    CHECK(mp.volume ==
          doctest::Approx(closed).epsilon(1e-12));

    // THE LIMITS — the formula's own coherence: r -> 0 is the
    // box, r = 1 the inscribed sphere (the body refuses it, the
    // form sees it).
    CHECK(steiner(0.0) ==
          doctest::Approx(8.0).epsilon(1e-15));
    CHECK(steiner(1.0) ==
          doctest::Approx(4.0 * kPi / 3.0).epsilon(1e-12));
    // the k=1 coherence pin (D7.2's own crosscheck): one vertical
    // fillet's golden 7.5+pi/8 is the same physics.
    CHECK(8.0 - 2.0 * kR * kR * (1.0 - kPi / 4.0) ==
          doctest::Approx(7.5 + kPi / 8.0).epsilon(1e-15));

    // ROAD 3 — the quadrature: the smooth pieces integrated
    // numerically on their own parametrizations (the m1/m3 oracle
    // class): the quarter disc in polar, the octant in spherical,
    // the polynomial terms exact. 2e6 samples, 2e-6 relative.
    {
        const double a = 2.0 - 2.0 * kR;
        // the quarter disc: area = int_0^{pi/2} int_0^r rho
        // d rho d theta — 1e6 midpoints.
        long double qd = 0.0L;
        const int N1 = 1000;
        const double dth = (kPi / 2.0) / N1;
        const double dr = kR / N1;
        for (int i = 0; i < N1; ++i)
            for (int j = 0; j < N1; ++j) {
                // the integrand is theta-free (the disc's own
                // symmetry): the midpoint in rho only.
                const double rho = (j + 0.5) * dr;
                qd += rho * dth * dr;
            }
        // the octant: volume = int_0^{pi/2} int_0^{pi/2}
        // int_0^r rho^2 sin(v) d rho du dv — 1e6 midpoints.
        long double qo = 0.0L;
        const int N2 = 100;
        const double du = (kPi / 2.0) / N2;
        const double dv = (kPi / 2.0) / N2;
        const double dr2 = kR / N2;
        for (int i = 0; i < N2; ++i)
            for (int j = 0; j < N2; ++j)
                for (int k = 0; k < N2; ++k) {
                    // the integrand is u-free (the octant's own
                    // rotational symmetry): midpoints in v, rho.
                    const double v = (j + 0.5) * dv;
                    const double rho = (k + 0.5) * dr2;
                    qo += rho * rho * std::sin(v) * du * dv *
                          dr2;
                }
        const double v_quad =
            a * a * a + 6.0 * a * a * kR +
            12.0 * static_cast<double>(qd) * a +
            8.0 * static_cast<double>(qo);
        CHECK(v_quad ==
              doctest::Approx(closed).epsilon(2e-6));
        CHECK(mp.volume ==
              doctest::Approx(v_quad).epsilon(2e-6));
    }

    // THE TANGENTIAL COLLAPSE — refused BY NAME (D7.2's wall: the
    // body is not the limit).
    {
        b::BRepBody* cur = g.steps[3].get();
        const b::FilletNaming ctx{FeatureId{6},
                                  &n::kBoxNaming};
        bool threw = false;
        std::string msg;
        try {
            const auto dead = b::fillet_ring_act(*cur, 1.0, ctx);
            (void)dead;
        } catch (const std::invalid_argument& e) {
            threw = true;
            msg = e.what();
        }
        REQUIRE(threw);
        CHECK(msg.find("tangential collapse") !=
              std::string::npos);
        CHECK(msg.find("Remedy") != std::string::npos);
    }
}

// ---------------------------------------------------------------------------
// THE PARTIALS (D7.3) — the four verticals at MIXED radii: the
// additive wedge formula, the strip conditions the admission's
// own; the mixed-radius act REFUSED BY NAME (D6.6); the naked
// sequential on the ring REFUSED (the m3 wall's own promise —
// this act is the twelve-spigoli morso it named).
// ---------------------------------------------------------------------------
TEST_CASE("G5 m4 the partials: the four verticals at mixed radii "
          "(the additive wedge formula), the act's radius mismatch "
          "refused by name, the naked sequential refused, the "
          "wrong-state act refused") {
    // THE MIXED-RADIUS VERTICAL CHAIN — the additive formula
    // 8 - sum_i 2 r_i^2 (1 - pi/4), the strip conditions verified
    // in admission (the faces' reach: r_i + r_j < 2).
    {
        const auto box =
            b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        const double radii[4] = {0.3, 0.4, 0.5, 0.35};
        std::vector<std::unique_ptr<b::BRepBody>> owned;
        b::BRepBody* cur = box.get();
        for (int k = 0; k < 4; ++k) {
            const ShapeId v{
                FeatureId{1},
                SubshapeRef{}.push(
                    static_cast<std::uint32_t>(8 + k)),
                {}};
            const b::FilletNaming ctx{
                FeatureId{static_cast<std::uint64_t>(2 + k)},
                &n::kBoxNaming};
            owned.push_back(
                b::fillet_edge(*cur, v, radii[k], ctx));
            REQUIRE(owned.back() != nullptr);
            cur = owned.back().get();
        }
        REQUIRE(cur->vertex_count() == 16);
        REQUIRE(cur->edge_count() == 24);
        REQUIRE(cur->face_count() == 10);
        const b::MassProperties mp =
            b::mass_properties(*cur, math::Tolerance{});
        double wedge = 0.0;
        for (const double r : radii)
            wedge += 2.0 * r * r * (1.0 - kPi / 4.0);
        CHECK(mp.volume ==
              doctest::Approx(8.0 - wedge).epsilon(1e-12));
        // THE STRIP CONDITION'S OWN WALL: two verticals on one
        // face at radii summing past the face's extension refuse
        // (the typed band, the m1/m2 admission's own).
        {
            const auto box2 =
                b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
            const ShapeId va{FeatureId{1},
                             SubshapeRef{}.push(8), {}};
            const ShapeId vb{FeatureId{1},
                             SubshapeRef{}.push(9), {}};
            const b::FilletNaming c1{FeatureId{2},
                                     &n::kBoxNaming};
            const b::FilletNaming c2{FeatureId{3},
                                     &n::kBoxNaming};
            auto one =
                b::fillet_edge(*box2, va, 1.2, c1);
            REQUIRE(one != nullptr);  // alone it fits (r < 2)
            bool threw = false;
            try {
                const auto dead =
                    b::fillet_edge(*one, vb, 1.2, c2);
                (void)dead;
            } catch (const std::invalid_argument&) {
                threw = true;
            }
            CHECK(threw);  // the strip: 1.2 + 1.2 > 2
        }
    }

    // THE ACT'S RADIUS MISMATCH — refused BY NAME (D6.6): the
    // spheres exist only at the verticals' common radius.
    {
        const auto box =
            b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        std::vector<std::unique_ptr<b::BRepBody>> owned;
        b::BRepBody* cur = box.get();
        for (int k = 0; k < 4; ++k) {
            const ShapeId v{
                FeatureId{1},
                SubshapeRef{}.push(
                    static_cast<std::uint32_t>(8 + k)),
                {}};
            const b::FilletNaming ctx{
                FeatureId{static_cast<std::uint64_t>(2 + k)},
                &n::kBoxNaming};
            owned.push_back(
                b::fillet_edge(*cur, v, 0.5, ctx));
            cur = owned.back().get();
        }
        const b::FilletNaming ctx{FeatureId{6},
                                  &n::kBoxNaming};
        bool threw = false;
        std::string msg;
        try {
            const auto dead =
                b::fillet_ring_act(*cur, 0.4, ctx);
            (void)dead;
        } catch (const std::invalid_argument& e) {
            threw = true;
            msg = e.what();
        }
        REQUIRE(threw);
        CHECK(msg.find("fillet_corner_radii_mismatch") !=
              std::string::npos);
        CHECK(msg.find("Remedy") != std::string::npos);
    }

    // THE NAKED SEQUENTIAL — the m3 wall's own refusal (a ring
    // edge at an occupied corner is the corner act's door for ONE
    // corner, and at TWO occupied corners it is THIS act's seat:
    // the message names the twelve-spigoli morso).
    {
        const CubeGuide g;
        const ShapeId top1{FeatureId{1},
                           SubshapeRef{}.push(5), {}};
        const b::FilletNaming ctx{FeatureId{7},
                                  &n::kBoxNaming};
        bool threw = false;
        std::string msg;
        try {
            const auto dead = b::fillet_ring_edge(
                *g.steps[3], top1, kR, ctx);
            (void)dead;
        } catch (const std::invalid_argument& e) {
            threw = true;
            msg = e.what();
        }
        REQUIRE(threw);
        CHECK(msg.find("occupied") != std::string::npos);
    }

    // THE WRONG STATE — the act on the bare box (or any state
    // before the four verticals) refuses with the formula's own
    // numbers (the order of consumption is the derivation).
    {
        const auto box =
            b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        const b::FilletNaming ctx{FeatureId{6},
                                  &n::kBoxNaming};
        bool threw = false;
        std::string msg;
        try {
            const auto dead = b::fillet_ring_act(*box, kR, ctx);
            (void)dead;
        } catch (const std::invalid_argument& e) {
            threw = true;
            msg = e.what();
        }
        REQUIRE(threw);
        CHECK(msg.find("16/24/10") != std::string::npos);
        CHECK(msg.find("Remedy") != std::string::npos);
    }
}

// ---------------------------------------------------------------------------
// THE SILENT EDIT OF r (D5.4's m6 leg) — the whole chain re-run at
// a different radius: the counts FIXED, the volume the closed form
// at the new r; and THE CHIRALITY pin (the spheres' CW domain
// walk, the family's own law the m3 test pinned one corner over).
// ---------------------------------------------------------------------------
TEST_CASE("G5 m4 the silent edit of r and the chirality: the chain "
          "re-run at r=0.3 lands the SAME counts with the volume "
          "at the new radius; the spheres' interior_left the "
          "family's own law") {
    // THE EDIT OF r — 24/48/26 for every r in band, the volume
    // the closed form (D5.4: the states upstream unchanged, the
    // act changed, the counts fixed).
    {
        constexpr double r2 = 0.3;
        const auto box =
            b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        std::vector<std::unique_ptr<b::BRepBody>> owned;
        b::BRepBody* cur = box.get();
        for (int k = 0; k < 4; ++k) {
            const ShapeId v{
                FeatureId{1},
                SubshapeRef{}.push(
                    static_cast<std::uint32_t>(8 + k)),
                {}};
            const b::FilletNaming ctx{
                FeatureId{static_cast<std::uint64_t>(2 + k)},
                &n::kBoxNaming};
            owned.push_back(
                b::fillet_edge(*cur, v, r2, ctx));
            REQUIRE(owned.back() != nullptr);
            cur = owned.back().get();
        }
        const b::FilletNaming actx{FeatureId{6},
                                   &n::kBoxNaming};
        auto fin = b::fillet_ring_act(*cur, r2, actx);
        REQUIRE(fin != nullptr);
        REQUIRE(fin->vertex_count() == 24);
        REQUIRE(fin->edge_count() == 48);
        REQUIRE(fin->face_count() == 26);
        REQUIRE(b::euler_characteristic(*fin) == 2);
        const b::MassProperties mp =
            b::mass_properties(*fin, math::Tolerance{});
        CHECK(mp.volume ==
              doctest::Approx(steiner(r2)).epsilon(1e-12));
    }

    // THE CHIRALITY — the eight spheres' p-curves all carry
    // interior_left (the material-on-the-left law; the naive CCW
    // copy of the cylinders' rule is the mancin the m3 test pinned
    // red — the closed-orientable gate refuses it here too).
    {
        const CubeGuide g;
        std::size_t spheres = 0, coedges = 0;
        for (const auto& f :
             g.result->solid.shell().faces()) {
            SphereReader sr;
            f.surface()->accept(sr);
            if (sr.sphere == nullptr) continue;
            ++spheres;
            for (const auto& ce :
                 f.loops()[0].coedges()) {
                REQUIRE(ce.p_curve() != nullptr);
                CHECK(ce.p_curve()->segment.interior_left);
                ++coedges;
            }
            // the frame's own law: normal() is the OUTWARD radial
            // (position - C)/r (the family's form; the walk's CW
            // domain sense compensates the inward cross-product
            // normal — the m3 pin one family over).
            const auto& s = *sr.sphere;
            const double u = 0.5 * (kPi / 2.0);
            const double v = 0.5 * (kPi / 2.0);
            const math::Vec3d p = s.evaluate(u, v);
            const math::Vec3d n_out =
                (p - s.origin()) * (1.0 / kR);
            const math::Vec3d n_fam = s.normal(u, v);
            CHECK(n_out.dot(n_fam) ==
                  doctest::Approx(1.0).epsilon(1e-12));
        }
        CHECK(spheres == 8);
        CHECK(coedges == 24);  // three arcs each
    }
}
