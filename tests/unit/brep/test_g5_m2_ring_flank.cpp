// tests/unit/brep/test_g5_m2_ring_flank.cpp
//
// G5 m2 — IL FIANCO DEL RING (ADR-0043, D2.2/D2.3/D2.5/D9): the
// rolling ball on the box's RING edge, the CylinderX/CylinderY
// flanks' WRITER — the m1 classes' first face-birthing consumer
// (the census's writer sites).
//
// THE STAGING DECLARATION (the prompt asked for the corner act; the
// ADR is the law of the campaign and its D9 prices THIS morso as the
// ring flank — "la costruzione chiusa, il walk, la formula, i golden
// 10/15/7 un asse sopra" — with the corner act at its own TWO morsi
// (D9's 3-4, "il rischio D6.2/3 vive qui"). The prompt's own rule
// hands the staging to the ADR; this file lands the ADR's morso and
// the corner act's WALL — the occupied corner refused with the
// occupying flank named, the quartic never consulted).
//
// THE GUIDE CASE (D2.2's own): box [-1,1]^3 -> fil#2 on
// box#1:top1 (the top x right edge, the Y-parallel spigolo
// (1, y, 1), r = 0.5): the flank born CylinderYSurface(0.5, -1,
// 1-r, 1-r) — the axis on the bisector, the tangent lines placed
// exactly (x = 1-r on the top, z = 1-r on the right), ZERO solves.
// THE GOLDEN: 10/15/7 with H = 0, chi = 2 (the same walk's
// arithmetic one axis over — "gli spigoli del ring sono verticali
// NEL LORO frame"); the volume V = 8 - 2*r^2*(1 - pi/4) = 7.5 +
// pi/8 = 7.892699081698725 at 1e-12 — THE SAME NUMBER as the
// vertical golden (7.5 + pi/8, the G2-u3 m2 guide): the same
// physics one axis over, the coherence pin; the chain fil#3 on
// box#1:bottom0 (the bottom x front edge, the X-parallel spigolo,
// NON-ADJACENT — the corners disjoint): 12/18/8, V = 7 + pi/4 —
// the m6ter chain's own number, the closed form per fillet
// l*r^2*(1-pi/4) twice.
//
// THE WALLS (the counterproofs): the corner collision (a previous
// fillet's flank at the corner — vertical-then-ring AND
// ring-then-ring — refused with the occupying flank named, the
// corner act's seat declared), the mixed order (ring-then-vertical
// refused by fillet_edge's own admission — the declared boundary,
// the register names it), the typed band on the chain (the shared
// face's reach TIGHTENED to 1.5 by the first tangent line, the
// bound NAMED), the vertical reference (fillet_edge's own class),
// and the CHIRALITY COUNTERPROOF on the writer's own frame (the
// naive flipped-e1 authoring RED at every sample, the born flank
// green on the same grid — the m1/m3 pattern).
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

namespace {

namespace b = CAD_0::brep;
namespace n = CAD_0::brep::naming;
using CAD_0::geometry::FeatureId;
using CAD_0::geometry::ShapeId;
using CAD_0::geometry::SubshapeRef;
using Vec3d = CAD_0::math::Vec3d;

constexpr double kPi = 3.14159265358979323846;
constexpr double kR = 0.5;
constexpr double kHalfPi = kPi / 2.0;

// The ring chain's construction: box#1 -> fil#2 (box#1:top1, the
// top x right edge, Y-parallel, r = 0.5) -> fil#3 (box#1:bottom0,
// the bottom x front edge, X-parallel, NON-ADJACENT — top1's
// corners are (1, +-1, 1), bottom0's are (+-1, -1, -1): disjoint).
// The naming contexts: the ROOT's table both times (the adjacent
// faces' stamps are the box's).
struct RingGuide {
    std::unique_ptr<b::BRepBody> box;
    std::unique_ptr<b::BRepBody> first;
    std::unique_ptr<b::BRepBody> result;
    ShapeId top1{};       // box#1:top1 (edge 5), dead in fil#2
    ShapeId bottom0{};    // box#1:bottom0 (edge 0), dead in fil#3
    ShapeId vertical2{};  // box#1:vertical2 (edge 10)

    explicit RingGuide(bool chain = true) {
        box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        top1 = ShapeId{FeatureId{1}, SubshapeRef{}.push(5), {}};
        bottom0 = ShapeId{FeatureId{1}, SubshapeRef{}.push(0), {}};
        vertical2 = ShapeId{FeatureId{1}, SubshapeRef{}.push(10), {}};
        const b::FilletNaming ctx2{FeatureId{2}, &n::kBoxNaming};
        first = b::fillet_ring_edge(*box, top1, kR, ctx2);
        if (!chain) return;
        const b::FilletNaming ctx3{FeatureId{3}, &n::kBoxNaming};
        result = b::fillet_ring_edge(*first, bottom0, kR, ctx3);
    }
};

} // namespace

// ===========================================================================
// THE GOLDEN k = 1 — the ring flank born, every number the same walk
// one axis over
// ===========================================================================
TEST_CASE("G5 m2 the golden: fil#2 on box#1:top1 — 10/15/7 with H=0 "
          "and chi=2, the flank CylinderYSurface on the bisector, the "
          "census closes, the volume 7.5+pi/8 all exact") {
    const RingGuide g(/*chain=*/false);
    REQUIRE(g.first != nullptr);

    // THE COUNTS — the walk's own arithmetic one axis over.
    REQUIRE(g.first->vertex_count() == 10);
    REQUIRE(g.first->edge_count() == 15);
    REQUIRE(g.first->face_count() == 7);
    REQUIRE(b::inner_loop_count(*g.first) == 0);
    REQUIRE(b::euler_characteristic(*g.first) == 2);
    CHECK(b::euler_characteristic(*g.first) ==
          b::euler_characteristic(*g.box));
    CHECK(b::is_closed_orientable(*g.first));
    CHECK(b::loops_are_closed(*g.first));

    // THE CENSUS: every edge exactly two OPPOSITE co-edges. The
    // per-face row (the factory's face order bottom/top/front/back/
    // left/right, the flank appended): the caps front and back +1
    // arc each (5), the laterals top and right keep 4 (the tangent
    // line REPLACES the dead co-edge), the far bottom and left 4,
    // the flank 4 — 30 incidences / 2 = 15.
    {
        for (const auto& e : g.first->edges) {
            REQUIRE(e.coedges().size() == 2);
            CHECK(e.coedges()[0]->reversed() !=
                  e.coedges()[1]->reversed());
        }
        std::size_t total = 0;
        for (const auto& f : g.first->solid.shell().faces())
            for (const auto& loop : f.loops())
                total += loop.coedges().size();
        REQUIRE(total == 30);
        const std::size_t per_face[7] = {4, 4, 5, 5, 4, 4, 4};
        for (std::size_t i = 0; i < 7; ++i) {
            CAPTURE(i);
            REQUIRE(g.first->solid.shell().faces()[i]
                        .loops()[0]
                        .coedges()
                        .size() == per_face[i]);
        }
    }

    // THE FLANK — born on the m1 placement: CylinderYSurface with
    // the axis ON THE BISECTOR (x = 1-r, z = 1-r), the base at the
    // low endpoint's quota (y = -1), the seam meridian u=0 at +X
    // (the right face's tangent line, the window's own edge).
    const auto& flank_face = g.first->solid.shell().faces()[6];
    struct YReader : b::SurfaceVisitor {
        const b::CylinderYSurface* y{nullptr};
        void visit(const b::CylinderYSurface& s) override { y = &s; }
    } yr;
    flank_face.surface()->accept(yr);
    REQUIRE(yr.y != nullptr);
    CHECK(yr.y->radius() == doctest::Approx(kR).epsilon(1e-12));
    CHECK(yr.y->y_base() == doctest::Approx(-1.0).epsilon(1e-12));
    CHECK(yr.y->axis_x() == doctest::Approx(1.0 - kR).epsilon(1e-12));
    CHECK(yr.y->axis_z() == doctest::Approx(1.0 - kR).epsilon(1e-12));
    CHECK(flank_face.surface()->family() ==
          b::SurfaceFamily::Cylinder);
    // The tangent lines placed EXACTLY: evaluate on the seam (u=0)
    // is the right face's tangent line; u=-pi/2 the top's.
    const Vec3d seam = yr.y->evaluate(0.0, 1.0);
    CHECK(seam.x == doctest::Approx(1.0).epsilon(1e-12));
    CHECK(seam.y == doctest::Approx(0.0).epsilon(1e-12));
    CHECK(seam.z == doctest::Approx(1.0 - kR).epsilon(1e-12));
    const Vec3d top_tan = yr.y->evaluate(-kHalfPi, 1.0);
    CHECK(top_tan.x == doctest::Approx(1.0 - kR).epsilon(1e-12));
    CHECK(top_tan.y == doctest::Approx(0.0).epsilon(1e-12));
    CHECK(top_tan.z == doctest::Approx(1.0).epsilon(1e-12));
    // All the flank's points at distance R from the axis line
    // {(1-r, y, 1-r)}; the frame rule pointwise on the born flank
    // (the writer's own frame, the m1 law).
    for (int i = 0; i <= 12; ++i) {
        const double u = -kHalfPi + kHalfPi * i / 12.0;
        for (int j = 0; j <= 6; ++j) {
            const double v = 2.0 * j / 6.0;  // y in [-1, 1]
            const Vec3d p = yr.y->evaluate(u, v);
            CHECK(std::hypot(p.x - (1.0 - kR), p.z - (1.0 - kR)) ==
                  doctest::Approx(kR).epsilon(1e-12));
            const double h = 1e-7;
            const Vec3d pu = yr.y->evaluate(u, v);
            const Vec3d du = (yr.y->evaluate(u + h, v) - pu) / h;
            const Vec3d dv = (yr.y->evaluate(u, v + h) - pu) / h;
            CHECK(du.cross(dv).dot(yr.y->normal(u, v)) > 0.0);
        }
    }

    // THE MASS — the closed form: V = 8 - 2*r^2*(1 - pi/4) (the
    // removed strip: the corner square minus the quarter disc, per
    // unit length, times the edge's length 2) = 7.5 + pi/8 — THE
    // SAME NUMBER as the vertical golden (the G2-u3 m2 guide): the
    // same physics one axis over, the coherence pin. The centroid:
    // the removed strip's cross-section centroid at
    // xc = [r^2(1-r/2) - (1-r)pi r^2/4 - r^3/3] / [r^2(1-pi/4)] =
    // 0.888316030527124 (the diagonal's foot), the body's centroid
    // (-(2*A*xc)/V, 0, -(2*A*xc)/V) with A = r^2(1-pi/4).
    const b::MassProperties mp = b::mass_properties(
        *g.first, CAD_0::math::Tolerance::linear(1e-9));
    CHECK(mp.all_exact);
    CHECK(mp.volume == doctest::Approx(7.5 + kPi / 8.0).epsilon(1e-12));
    CHECK(mp.volume ==
          doctest::Approx(8.0 - 2.0 * kR * kR *
                                    (1.0 - kPi / 4.0))
              .epsilon(1e-12));
    CHECK(mp.volume ==
          doctest::Approx(7.892699081698725).epsilon(1e-12));
    const double A = kR * kR * (1.0 - kPi / 4.0);
    const double xc =
        (kR * kR * (1.0 - kR / 2.0) -
         (1.0 - kR) * kPi * kR * kR / 4.0 - kR * kR * kR / 3.0) /
        A;
    CHECK(xc == doctest::Approx(0.888316030527124).epsilon(1e-12));
    const double V = 8.0 - 2.0 * A;
    CHECK(mp.centroid.x ==
          doctest::Approx(-(2.0 * A * xc) / V).epsilon(1e-12));
    CHECK(mp.centroid.y == doctest::Approx(0.0).epsilon(1e-12));
    CHECK(mp.centroid.z ==
          doctest::Approx(-(2.0 * A * xc) / V).epsilon(1e-12));
    CHECK(mp.centroid.x ==
          doctest::Approx(-0.012076619776158).epsilon(1e-12));

    // THE ORACLE, road two — the test's own midpoint quadrature of
    // the REMOVED STRIP's cross-section (the square [1-r,1]^2 minus
    // the quarter disc centered (1-r, 1-r)), ZERO code shared with
    // the kernel: V = 8 - 2 * (the area integral), the agreement at
    // the declared 2e-6.
    {
        double area = 0.0;
        const int N = 4000;
        const double d = kR / N;
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < N; ++j) {
                const double x = 1.0 - kR + (i + 0.5) * d;
                const double z = 1.0 - kR + (j + 0.5) * d;
                const double dx = x - (1.0 - kR);
                const double dz = z - (1.0 - kR);
                if (dx * dx + dz * dz >= kR * kR) area += d * d;
            }
        CHECK(mp.volume ==
              doctest::Approx(8.0 - 2.0 * area).epsilon(2e-6));
    }

    // THE NAIVE COUNTERPROOF (the volume): the box form and the
    // half-cylinder form are NOT the filleted body's volume.
    CHECK_FALSE(mp.volume == doctest::Approx(8.0).epsilon(1e-3));
    CHECK_FALSE(mp.volume ==
                doctest::Approx(8.0 - kPi * kR * kR / 2.0)
                    .epsilon(1e-3));

    // THE NAMES — the same grammar as the verticals' (D2.5: "la
    // STESSA grammatica... nessuna estensione necessaria"), the
    // operands the referenced edge's adjacent faces in the co-edge
    // order, the tangent wires the isect pattern.
    const std::string flank_name = flank_face.name();
    CHECK(flank_name.find("flank[box#1:") != std::string::npos);
    CHECK(flank_name.find("fil#2:") == 0);
    CHECK((flank_name.find("box#1:top") != std::string::npos &&
           flank_name.find("box#1:right") != std::string::npos));
    // The tangent wires carry the isect grammar, one per adjacent
    // face (the landing faces), the flank nested.
    std::size_t wires = 0;
    for (const auto& e : g.first->edges) {
        if (e.name().find("fil#2:isect[") == 0) {
            ++wires;
            CHECK(e.name().find("fil#2:flank[") != std::string::npos);
        }
    }
    CHECK(wires == 2);
}

// ===========================================================================
// THE CHAIN k = 2 — the non-adjacent ring chain: both placements,
// the gate green at every step, the digest bit-identical to the
// re-run, the volume 7 + pi/4 (the m6ter chain's own number)
// ===========================================================================
TEST_CASE("G5 m2 the chain: fil#3 on box#1:bottom0 — 12/18/8, the "
          "CylinderX flank beside the CylinderY one, the gate green "
          "at every step, the digest bit-identical, V = 7+pi/4") {
    const RingGuide g;
    REQUIRE(g.first != nullptr);
    REQUIRE(g.result != nullptr);

    // THE COUNTS — the chain formula 8+2k/12+3k/6+k at k = 2.
    REQUIRE(g.result->vertex_count() == 12);
    REQUIRE(g.result->edge_count() == 18);
    REQUIRE(g.result->face_count() == 8);
    REQUIRE(b::inner_loop_count(*g.result) == 0);
    REQUIRE(b::euler_characteristic(*g.result) == 2);
    CHECK(b::is_closed_orientable(*g.result));
    CHECK(b::loops_are_closed(*g.result));
    for (const auto& e : g.result->edges)
        REQUIRE(e.coedges().size() == 2);

    // BOTH PLACEMENTS — the chain carries one CylinderY flank (fil#2,
    // the top1 edge) and one CylinderX flank (fil#3, the bottom0
    // edge, the axis on the bisector at (y, z) = (-(1-r), -(1-r)),
    // the base at x = -1): the kept flank rides through the second
    // walk VERBATIM (its native class, the value copy).
    struct XReader : b::SurfaceVisitor {
        const b::CylinderXSurface* x{nullptr};
        void visit(const b::CylinderXSurface& s) override { x = &s; }
    } xr;
    struct YReader : b::SurfaceVisitor {
        const b::CylinderYSurface* y{nullptr};
        void visit(const b::CylinderYSurface& s) override { y = &s; }
    } yr;
    std::size_t x_flanks = 0, y_flanks = 0;
    for (const auto& f : g.result->solid.shell().faces()) {
        xr.x = nullptr;
        yr.y = nullptr;
        f.surface()->accept(xr);
        f.surface()->accept(yr);
        if (xr.x != nullptr) {
            ++x_flanks;
            CHECK(xr.x->radius() == doctest::Approx(kR).epsilon(1e-12));
            CHECK(xr.x->x_base() ==
                  doctest::Approx(-1.0).epsilon(1e-12));
            CHECK(xr.x->axis_y() ==
                  doctest::Approx(-(1.0 - kR)).epsilon(1e-12));
            CHECK(xr.x->axis_z() ==
                  doctest::Approx(-(1.0 - kR)).epsilon(1e-12));
            CHECK(f.name().find("fil#3:flank[box#1:bottom") !=
                  std::string::npos);
            CHECK(f.name().find("box#1:front") != std::string::npos);
        }
        if (yr.y != nullptr) {
            ++y_flanks;
            CHECK(f.name().find("fil#2:flank[") == 0);
        }
    }
    CHECK(x_flanks == 1);
    CHECK(y_flanks == 1);

    // THE MASS — two disjoint wedges: V = 8 - 2*(2*r^2*(1-pi/4)) =
    // 7 + pi/4 (the m6ter chain's own number, the closed form per
    // fillet l*r^2*(1-pi/4) twice — the same physics any axis pair).
    const b::MassProperties mp = b::mass_properties(
        *g.result, CAD_0::math::Tolerance::linear(1e-9));
    CHECK(mp.all_exact);
    CHECK(mp.volume ==
          doctest::Approx(7.0 + kPi / 4.0).epsilon(1e-12));
    CHECK(mp.volume ==
          doctest::Approx(7.785398163397448).epsilon(1e-12));

    // THE GATE at every step — the tavola derived (fillet_descent's
    // by-name diff), the held survivors RESOLVED, the digest drift
    // against the re-run (the chain re-run from the recipes, the
    // digests bit-identical).
    {
        const auto tav1 = b::fillet_descent(*g.first, *g.box);
        const auto tav2 =
            b::fillet_descent(*g.result, *g.first);
        const ShapeId top3{FeatureId{1}, SubshapeRef{}.push(7), {}};
        const ShapeId top_face{FeatureId{1}, SubshapeRef{}.push(1),
                               {}};
        const auto v1 = b::chain_gate(b::ChainStep{
            g.first.get(), &tav1, g.box.get(), nullptr, nullptr,
            {{top3, b::Resolution::Kind::Resolved},
             {top_face, b::Resolution::Kind::Resolved}},
            b::chain_digest(*g.first)});
        CHECK(v1.pass);
        const auto v2 = b::chain_gate(b::ChainStep{
            g.result.get(), &tav2, g.first.get(), nullptr, nullptr,
            {{top3, b::Resolution::Kind::Resolved},
             {top_face, b::Resolution::Kind::Resolved}},
            b::chain_digest(*g.result)});
        CHECK(v2.pass);
        if (!v1.pass || !v2.pass) {
            for (const auto& f : v1.failures) MESSAGE(f.message);
            for (const auto& f : v2.failures) MESSAGE(f.message);
        }

        // THE RE-RUN — the chain re-driven from the recipes, the
        // digest BIT-IDENTICAL (the drift predicate's own subject).
        const b::FilletNaming rc2{FeatureId{2}, &n::kBoxNaming};
        const b::FilletNaming rc3{FeatureId{3}, &n::kBoxNaming};
        auto rerun1 =
            b::fillet_ring_edge(*g.box, g.top1, kR, rc2);
        auto rerun2 =
            b::fillet_ring_edge(*rerun1, g.bottom0, kR, rc3);
        CHECK(b::chain_digest(*rerun1) == b::chain_digest(*g.first));
        CHECK(b::chain_digest(*rerun2) ==
              b::chain_digest(*g.result));
    }
}

// ===========================================================================
// THE WALLS — the counterproofs, each refusal with its own named
// message (the P2 culture)
// ===========================================================================
TEST_CASE("G5 m2 the walls: the occupied corner refused with the "
          "flank named (the corner act's seat), the typed band on "
          "the chain, the vertical reference, the mixed order's "
          "declared boundary") {
    // (1) THE CORNER COLLISION, vertical-then-ring: fillet_edge on
    // box#1:vertical2 first (the (+X,+Y) corner occupied), then
    // fillet_ring_edge on box#1:top1 — the corner (1,1,1) is
    // shared: REFUSED with the occupying flank named and the corner
    // act's seat declared (the quartic never consulted).
    {
        const auto box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        const ShapeId vertical2{FeatureId{1},
                                SubshapeRef{}.push(10), {}};
        const ShapeId top1{FeatureId{1}, SubshapeRef{}.push(5), {}};
        const b::FilletNaming vctx{FeatureId{2}, &n::kBoxNaming};
        const auto one =
            b::fillet_edge(*box, vertical2, kR, vctx);
        REQUIRE(one != nullptr);
        const b::FilletNaming rctx{FeatureId{3}, &n::kBoxNaming};
        bool threw = false;
        std::string msg;
        try {
            const auto dead =
                b::fillet_ring_edge(*one, top1, kR, rctx);
            (void)dead;
        } catch (const std::invalid_argument& e) {
            threw = true;
            msg = e.what();
        }
        REQUIRE(threw);
        CHECK(msg.find("occupied") != std::string::npos);
        CHECK(msg.find("flank") != std::string::npos);
        CHECK(msg.find("quartic") != std::string::npos);
        CHECK(msg.find("Remedy") != std::string::npos);
    }

    // (2) THE CORNER COLLISION, ring-then-ring: fil#2 on top1, then
    // fil#3 on box#1:top2 (the top x back edge — the corner (1,1,1)
    // SHARED with top1): REFUSED the same wall.
    {
        const RingGuide g(/*chain=*/false);
        const ShapeId top2{FeatureId{1}, SubshapeRef{}.push(6), {}};
        const b::FilletNaming ctx{FeatureId{3}, &n::kBoxNaming};
        bool threw = false;
        std::string msg;
        try {
            const auto dead =
                b::fillet_ring_edge(*g.first, top2, kR, ctx);
            (void)dead;
        } catch (const std::invalid_argument& e) {
            threw = true;
            msg = e.what();
        }
        REQUIRE(threw);
        CHECK(msg.find("occupied") != std::string::npos);
    }

    // (3) THE MIXED ORDER'S DECLARED BOUNDARY: ring-then-vertical —
    // fillet_edge's own admission takes CylinderZ flanks only (its
    // message names the class families); the mixed chain in THIS
    // order is the declared boundary the register names (the
    // corner act's morsi own the meeting).
    {
        const RingGuide g(/*chain=*/false);
        const b::FilletNaming ctx{FeatureId{3}, &n::kBoxNaming};
        bool threw = false;
        std::string msg;
        try {
            const auto dead =
                b::fillet_edge(*g.first, g.vertical2, kR, ctx);
            (void)dead;
        } catch (const std::invalid_argument& e) {
            threw = true;
            msg = e.what();
        }
        REQUIRE(threw);
        CHECK(msg.find("class families") != std::string::npos);
    }

    // (4) THE TYPED BAND on the chain: fil#2 on top1 (r=0.5), then
    // fil#3 on box#1:top3 (the top x left edge — the SHARED top
    // face, its reach TIGHTENED to 1.5 by the first tangent line):
    // r = 1.5 REFUSED with the bound NAMED (the m6ter's own
    // counterproof one axis over).
    {
        const RingGuide g(/*chain=*/false);
        const ShapeId top3{FeatureId{1}, SubshapeRef{}.push(7), {}};
        const b::FilletNaming ctx{FeatureId{3}, &n::kBoxNaming};
        bool threw = false;
        std::string msg;
        try {
            const auto dead =
                b::fillet_ring_edge(*g.first, top3, 1.5, ctx);
            (void)dead;
        } catch (const std::invalid_argument& e) {
            threw = true;
            msg = e.what();
        }
        REQUIRE(threw);
        CHECK(msg.find("fillet_radius_exceeds_face") !=
              std::string::npos);
        CHECK(msg.find("1.500000") != std::string::npos);
    }

    // (5) THE VERTICAL REFERENCE: fillet_ring_edge on
    // box#1:vertical2 — fillet_edge's own class, refused by name.
    {
        const auto box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        const ShapeId vertical2{FeatureId{1},
                                SubshapeRef{}.push(10), {}};
        const b::FilletNaming ctx{FeatureId{2}, &n::kBoxNaming};
        bool threw = false;
        std::string msg;
        try {
            const auto dead =
                b::fillet_ring_edge(*box, vertical2, kR, ctx);
            (void)dead;
        } catch (const std::invalid_argument& e) {
            threw = true;
            msg = e.what();
        }
        REQUIRE(threw);
        CHECK(msg.find("vertical") != std::string::npos);
        CHECK(msg.find("fillet_edge") != std::string::npos);
    }

    // (6) THE DEAD REFERENCE: box#1:top1 after fil#2 — the
    // structural find's own wall (the reference that died refuses
    // by name).
    {
        const RingGuide g(/*chain=*/false);
        const b::FilletNaming ctx{FeatureId{3}, &n::kBoxNaming};
        bool threw = false;
        try {
            const auto dead =
                b::fillet_ring_edge(*g.first, g.top1, kR, ctx);
            (void)dead;
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        REQUIRE(threw);
    }
}

// ===========================================================================
// THE CHIRALITY COUNTERPROOF — the naive flipped-ê₁ authoring of the
// ring flank's frame RED at every sample, the born flank green on
// the same grid (the m1/m3 pattern, read on the WRITER's output)
// ===========================================================================
TEST_CASE("G5 m2 the counterproof: the naive left-handed ring "
          "authoring is RED at every sample — the born flank green "
          "on the same grid (the discriminating pin, no knob)") {
    // THE NAIVE AUTHORING: ê₁ = +ẑ for the Y placement (ê₁ = â × ê₀
    // NEGATED) — evaluate (1-r + R cos u, v, 1-r + R sin u), the
    // historical left-handed cylinder's own mistake one family
    // over. It is NOT the landed writer (the m1 classes derive ê₁
    // from the product; the born flank carries the right-handed
    // frame); this local authoring exists so the guard's
    // discrimination is pinned.
    struct NaiveLeftHandedY {
        Vec3d evaluate(double u, double v) const {
            return {1.0 - kR + kR * std::cos(u), v,
                    1.0 - kR + kR * std::sin(u)};
        }
        Vec3d normal(double u, double) const {
            return {std::cos(u), 0.0, std::sin(u)};
        }
    };
    const NaiveLeftHandedY naive;
    int red = 0, total = 0;
    for (int i = 0; i <= 12; ++i) {
        const double u = -kPi + 2.0 * kPi * i / 12.0;
        for (int j = 0; j <= 12; ++j) {
            const double v = -1.0 + 2.0 * j / 12.0;
            const double h = 1e-7;
            const Vec3d pu = naive.evaluate(u, v);
            const Vec3d du = (naive.evaluate(u + h, v) - pu) / h;
            const Vec3d dv = (naive.evaluate(u, v + h) - pu) / h;
            ++total;
            if (du.cross(dv).dot(naive.normal(u, v)) <= 0.0) ++red;
        }
    }
    CHECK(red == total);
    CHECK(total == 169);

    // The born flank (the writer's own output) passes the same
    // grid — the pair read together, the red is the naive's alone.
    const RingGuide g(/*chain=*/false);
    struct YReader : b::SurfaceVisitor {
        const b::CylinderYSurface* y{nullptr};
        void visit(const b::CylinderYSurface& s) override { y = &s; }
    } yr;
    g.first->solid.shell().faces()[6].surface()->accept(yr);
    REQUIRE(yr.y != nullptr);
    int green = 0;
    for (int i = 0; i <= 12; ++i) {
        const double u = -kPi + 2.0 * kPi * i / 12.0;
        for (int j = 0; j <= 12; ++j) {
            const double v = -1.0 + 2.0 * j / 12.0;
            const double h = 1e-7;
            const Vec3d pu = yr.y->evaluate(u, v);
            const Vec3d du = (yr.y->evaluate(u + h, v) - pu) / h;
            const Vec3d dv = (yr.y->evaluate(u, v + h) - pu) / h;
            if (du.cross(dv).dot(yr.y->normal(u, v)) > 0.0) ++green;
        }
    }
    CHECK(green == 169);
}
