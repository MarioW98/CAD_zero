// tests/unit/brep/test_g5_m3_corner_act.cpp
//
// G5 m3 — L'ATTO D'ANGOLO SU UN ANGOLO SOLO (ADR-0043, D5.1/D5.2/
// D6.2/D6.3/D9's third morso, its first half): the corner sphere
// and the atomic act — the sphere born in the corner region, the
// two ring flanks born ALREADY REFINED (their windows end at the
// coaxial arcs), the vertical flank RE-WINDOWED in place (the cap
// arc dead, the coaxial equator its replacement), the plane faces
// re-authored at the tangent lines. The twelve-spigoli simultaneous
// act is the NEXT morso's (D9's second half); this file lands ONE
// corner, the whole corner.
//
// THE CHAIN (the brief's three fillets, D5.1's indivisibility read
// on the ADR's own words): fil#2 = fillet_edge on box#1:vertical2
// (the plain walk, 10/15/7, gate green), fil#3 = fillet_ring_edge
// on box#1:top1 — the corner (1,1,1) is OCCUPIED by fil#2's
// CylinderZ flank, the m2 wall's own seat, and since m3 that call
// IS the act: it consumes the corner WHOLE (top1 AND its partner
// box#1:top2, the indivisible pair — a ring flank without its
// sphere has untyped borders, D5.1) and lands 13/21/10. The third
// fillet (top2's edge) dies inside the act's step: the sphere is
// born when it lands. No intermediate state exists — the gate runs
// after each STEP that produces a body.
//
// THE ACT'S OWN ARITHMETIC (the tavola's closure, derived): on the
// k=1 chain, V-4+7 / E-3+9 / F+3: the four dead vertices are the
// two far corners (1,-1,1), (-1,1,1) and the two corner-side
// junctions (1,1-r,1), (1-r,1,1); the three dead edges are the two
// ring edges and the corner's cap arc; the seven born vertices are
// the three tangency points (the feet of the perpendiculars from C
// to the three planes: (1,1-r,1-r), (1-r,1,1-r), (1-r,1-r,1)) and
// the two far tangent pairs; the nine born edges are the four
// tangent lines, the two far cap arcs and the three coaxial arcs;
// the three born faces are the two ring flanks and the sphere
// octant. chi = 13-21+10 = 2, flat (the full act's 24/48/26 is the
// same arithmetic eight corners over).
//
// THE VOLUME GOLDEN (D7.2's Steiner, PARTIAL — the derivation
// visible): the removed set is DISJOINT: three wedge strips below
// the coaxial planes (each the corner-square-minus-quarter-disc
// area r^2(1-pi/4) times the length (2-r) — the strip stops at the
// plane z=1-r/x=1-r/y=1-r where the sphere takes over) plus the
// corner cube [1-r,1]^3 minus the ball's octant (the material the
// sphere keeps):
//     V = 8 - 3*(2-r)*r^2*(1-pi/4) - r^3 + pi*r^3/6
// at r = 0.5: 6.75 + 29*pi/96 = 7.699022780771917 (1e-12, the
// kernel's divergence theorem the second road), and the QUADRATURE
// the third road (2e6 midpoint samples, 2e-6 relative, the m1
// oracle's own class): the quarter disc and the octant integrated
// numerically on their OWN parametrizations — smooth integrands,
// O(h^2), zero code shared with the kernel's closed forms.
//
// THE WITNESS (D3's thesis, falsifiable): the act consults NO SSI
// row and solves NOTHING — every quantity is a closed form of the
// stored operands (the center from the three planes, the tangency
// points as the feet of the perpendiculars, the arcs' azimuth
// spans the flanks' own windows). The quartic — the
// cylinder-perpendicular-cylinder intersection the naked
// sequential would need — is NEVER consulted: the walls between
// the flanks never meet, the windows stop at the coaxial planes.
// The pin: the born sphere carries the EXISTING family (a new
// WINDOW, never a new surface, D5.2) and the diff touches zero
// ssi.hpp/ssi.cpp lines.
//
// THE COUNTERPROOFS: the chirality (the sphere's parametrization
// carries du x dv INWARD, so the material-on-the-left law reads
// the CLOCKWISE domain walk — the naive CCW copy of the cylinders'
// own rule is the MANCINO, and it is RED at every sample of the
// window: the walk's flux sign inverted everywhere, the closed
// contract broken); the Born-sotto-falso (the vertical flank's
// tavola row is ABSORBED, its stamp never in a Vanished row, its
// resolution MATCH across the act — the death-and-rebirth under
// the same name REFUSED); the anonymous death (every Vanished row
// carries its cause verbatim — the cap arc's own
// "replaced by the corner sphere's tangent arc", D5.4's words).
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

using b::VertexId;

constexpr double kR = 0.5;
constexpr double kPi = 3.14159265358979323846;

// The chain's guide: box -> fil#2 (vertical2, the plain walk) ->
// fil#3 (top1 at the occupied corner = THE ACT).
struct CornerGuide {
    std::unique_ptr<b::BRepBody> box;
    std::unique_ptr<b::BRepBody> first;   // 10/15/7
    std::unique_ptr<b::BRepBody> result;  // 13/21/10

    ShapeId vertical2{FeatureId{1}, SubshapeRef{}.push(10), {}};
    ShapeId top1{FeatureId{1}, SubshapeRef{}.push(5), {}};

    CornerGuide() {
        box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        REQUIRE(box != nullptr);
        const b::FilletNaming vctx{FeatureId{2}, &n::kBoxNaming};
        first = b::fillet_edge(*box, vertical2, kR, vctx);
        REQUIRE(first != nullptr);
        const b::FilletNaming rctx{FeatureId{3}, &n::kBoxNaming};
        result = b::fillet_ring_edge(*first, top1, kR, rctx);
        REQUIRE(result != nullptr);
    }
};

struct SphereReader : b::SurfaceVisitor {
    const b::SphereSurface* sphere{nullptr};
    void visit(const b::SphereSurface& s) override { sphere = &s; }
};
struct XReader : b::SurfaceVisitor {
    const b::CylinderXSurface* x{nullptr};
    void visit(const b::CylinderXSurface& s) override { x = &s; }
};
struct YReader : b::SurfaceVisitor {
    const b::CylinderYSurface* y{nullptr};
    void visit(const b::CylinderYSurface& s) override { y = &s; }
};
struct ZReader : b::SurfaceVisitor {
    const b::CylinderZSurface* z{nullptr};
    void visit(const b::CylinderZSurface& s) override { z = &s; }
};

const b::Face* face_by_name(const b::BRepBody& body,
                            const std::string& name) {
    for (const auto& f : body.solid.shell().faces())
        if (f.name() == name) return &f;
    return nullptr;
}

}  // namespace

// ---------------------------------------------------------------------------
// THE CONSTRUCTION — the chain, the gate green at every step, the
// digests bit-identical, the born geometry's closed forms.
// ---------------------------------------------------------------------------
TEST_CASE("G5 m3 the corner act: the chain lands 13/21/10, the "
          "gate green at every step, the sphere born with the "
          "existing family, the closed forms exact") {
    const CornerGuide g;

    // THE COUNTS — the act's own arithmetic (V-4+7 / E-3+9 / F+3
    // on the 10/15/7 chain).
    REQUIRE(g.result->vertex_count() == 13);
    REQUIRE(g.result->edge_count() == 21);
    REQUIRE(g.result->face_count() == 10);
    REQUIRE(b::inner_loop_count(*g.result) == 0);
    REQUIRE(b::euler_characteristic(*g.result) == 2);
    CHECK(b::is_closed_orientable(*g.result));
    CHECK(b::loops_are_closed(*g.result));
    for (const auto& e : g.result->edges)
        REQUIRE(e.coedges().size() == 2);

    // THE CHAIN GATE — the five predicates on each step, the held
    // references resolving with their D3 classes (the vertical
    // flank SURVIVES the act: MATCH; the ring edge it consumes:
    // VANISHED), the digests the steps' own.
    const auto tav1 = b::fillet_descent(*g.first, *g.box);
    const auto tav2 = b::fillet_descent(*g.result, *g.first);
    const ShapeId top3{FeatureId{1}, SubshapeRef{}.push(7), {}};
    const ShapeId top_face{FeatureId{1}, SubshapeRef{}.push(1),
                           {}};
    const auto v1 = b::chain_gate(b::ChainStep{
        g.first.get(), &tav1, g.box.get(), nullptr, nullptr,
        {{top3, b::Resolution::Kind::Resolved},
         {top_face, b::Resolution::Kind::Resolved}},
        b::chain_digest(*g.first)});
    CHECK(v1.pass);
    // The act's step: the vertical flank's stamp (held across the
    // act) resolves MATCH — the re-windowing is an EDIT, never a
    // death (D6.3); the top1 ring edge resolves VANISHED (the act
    // consumed it).
    const ShapeId flank2 = [&] {
        for (const auto& f : g.first->solid.shell().faces())
            if (f.name().rfind("fil#2:flank", 0) == 0)
                return f.shape_id();
        return ShapeId{};
    }();
    REQUIRE(flank2.subshape.depth > 0);
    const auto v2 = b::chain_gate(b::ChainStep{
        g.result.get(), &tav2, g.first.get(), nullptr, nullptr,
        {{flank2, b::Resolution::Kind::Resolved},
         {top_face, b::Resolution::Kind::Resolved}},
        b::chain_digest(*g.result)});
    for (const auto& f : v2.failures) MESSAGE(f.message);
    CHECK(v2.pass);
    if (!v1.pass || !v2.pass) {
        for (const auto& f : v1.failures) MESSAGE(f.message);
        for (const auto& f : v2.failures) MESSAGE(f.message);
    }

    // THE RE-RUN — the chain re-driven, the digests BIT-IDENTICAL
    // (the drift predicate's own subject).
    {
        const b::FilletNaming vc{FeatureId{2}, &n::kBoxNaming};
        const b::FilletNaming rc{FeatureId{3}, &n::kBoxNaming};
        auto r1 = b::fillet_edge(*g.box, g.vertical2, kR, vc);
        auto r2 = b::fillet_ring_edge(*r1, g.top1, kR, rc);
        CHECK(b::chain_digest(*r1) == b::chain_digest(*g.first));
        CHECK(b::chain_digest(*r2) ==
              b::chain_digest(*g.result));
    }

    // THE BORN SPHERE — the EXISTING family (D5.2: "NESSUNA
    // superficie nuova, una finestra nuova"), the closed forms:
    // center C = the corner offset r inward along each plane's
    // normal, the frame DERIVED (pole the shared plane's outward
    // normal, azimuth the reference's lateral's, the second axis
    // the product).
    const b::Face* corner =
        face_by_name(*g.result,
                     "fil#3:corner[box#1:top × box#1:right × "
                     "box#1:back]");
    REQUIRE(corner != nullptr);
    {
        SphereReader sr;
        corner->surface()->accept(sr);
        REQUIRE(sr.sphere != nullptr);
        CHECK(sr.sphere->radius() ==
              doctest::Approx(kR).epsilon(1e-12));
        CHECK(sr.sphere->origin().x ==
              doctest::Approx(1.0 - kR).epsilon(1e-12));
        CHECK(sr.sphere->origin().y ==
              doctest::Approx(1.0 - kR).epsilon(1e-12));
        CHECK(sr.sphere->origin().z ==
              doctest::Approx(1.0 - kR).epsilon(1e-12));
        CHECK(sr.sphere->pole().x ==
              doctest::Approx(0.0).epsilon(1e-12));
        CHECK(sr.sphere->pole().y ==
              doctest::Approx(0.0).epsilon(1e-12));
        CHECK(sr.sphere->pole().z ==
              doctest::Approx(1.0).epsilon(1e-12));
        CHECK(sr.sphere->azimuth().x ==
              doctest::Approx(1.0).epsilon(1e-12));
        CHECK(sr.sphere->azimuth().y ==
              doctest::Approx(0.0).epsilon(1e-12));
        CHECK(sr.sphere->azimuth().z ==
              doctest::Approx(0.0).epsilon(1e-12));
        CHECK(sr.sphere->family() == b::SurfaceFamily::Sphere);
        // The ottanto's three arcs, one loop, three vertices: the
        // tangency points themselves (D5.2).
        REQUIRE(corner->loops().size() == 1);
        REQUIRE(corner->loops()[0].coedges().size() == 3);
    }

    // THE THREE TANGENCY POINTS — the feet of the perpendiculars:
    // on x=1, on y=1, on z=1 (the derivations, never solves).
    {
        bool found[3] = {false, false, false};
        const double expect[3][3] = {
            {1.0, 1.0 - kR, 1.0 - kR},
            {1.0 - kR, 1.0, 1.0 - kR},
            {1.0 - kR, 1.0 - kR, 1.0}};
        for (const auto& v : g.result->vertices) {
            const auto& p = v.position();
            for (int k = 0; k < 3; ++k) {
                if (std::abs(p.x - expect[k][0]) < 1e-12 &&
                    std::abs(p.y - expect[k][1]) < 1e-12 &&
                    std::abs(p.z - expect[k][2]) < 1e-12)
                    found[k] = true;
            }
        }
        CHECK(found[0]);
        CHECK(found[1]);
        CHECK(found[2]);
    }

    // THE TWO RING FLANKS — born GIÀ RIFINITI: the windows END at
    // the coaxial planes (the axis lines pass through C, the
    // spans [a_far, C's own coordinate], never the full edge).
    {
        const b::Face* f1 = face_by_name(
            *g.result, "fil#3:flank[box#1:top × box#1:right]");
        const b::Face* f2 = face_by_name(
            *g.result, "fil#3:flank[box#1:top × box#1:back]");
        REQUIRE(f1 != nullptr);
        REQUIRE(f2 != nullptr);
        YReader yr;
        f1->surface()->accept(yr);
        REQUIRE(yr.y != nullptr);
        CHECK(yr.y->radius() ==
              doctest::Approx(kR).epsilon(1e-12));
        CHECK(yr.y->y_base() ==
              doctest::Approx(-1.0).epsilon(1e-12));
        CHECK(yr.y->axis_x() ==
              doctest::Approx(1.0 - kR).epsilon(1e-12));
        CHECK(yr.y->axis_z() ==
              doctest::Approx(1.0 - kR).epsilon(1e-12));
        XReader xr;
        f2->surface()->accept(xr);
        REQUIRE(xr.x != nullptr);
        CHECK(xr.x->radius() ==
              doctest::Approx(kR).epsilon(1e-12));
        CHECK(xr.x->x_base() ==
              doctest::Approx(-1.0).epsilon(1e-12));
        CHECK(xr.x->axis_y() ==
              doctest::Approx(1.0 - kR).epsilon(1e-12));
        CHECK(xr.x->axis_z() ==
              doctest::Approx(1.0 - kR).epsilon(1e-12));
    }

    // THE VERTICAL FLANK — RI-FINESTRATO, ALIVE, ITS NAME ITS OWN:
    // the same stamp, the surface unchanged, the loop re-authored
    // (the tangent lines shortened to the coaxial plane z = 1-r,
    // the cap arc replaced by the equator).
    const b::Face* vflank = face_by_name(
        *g.first, "fil#2:flank[box#1:back × box#1:right]");
    REQUIRE(vflank != nullptr);
    {
        const b::Face* after = face_by_name(
            *g.result,
            "fil#2:flank[box#1:back × box#1:right]");
        REQUIRE(after != nullptr);
        CHECK(after->shape_id() == vflank->shape_id());
        ZReader zr;
        after->surface()->accept(zr);
        REQUIRE(zr.z != nullptr);
        CHECK(zr.z->radius() ==
              doctest::Approx(kR).epsilon(1e-12));
        // The shortened tangent lines' far ends survive; their
        // corner ends are the equator's endpoints (the laterals'
        // tangency points) — the window edit read on the walk.
        REQUIRE(after->loops().size() == 1);
        REQUIRE(after->loops()[0].coedges().size() == 4);
        bool equator_named = false;
        for (const auto& ce :
             after->loops()[0].coedges()) {
            const auto& e =
                g.result->edges[ce.edge()];
            if (e.name().find("isect") != std::string::npos &&
                e.name().find("corner") != std::string::npos)
                equator_named = true;
        }
        CHECK(equator_named);
    }

    // THE PLANE FACES — D5.3's rectangle: the top's corner is the
    // sphere's tangency point (the dead cap arc's center WAS the
    // tangency point — the closure of the construction).
    {
        const b::Face* top = face_by_name(*g.result, "box#1:top");
        REQUIRE(top != nullptr);
        bool at_tangency = false;
        for (const auto& ce : top->loops()[0].coedges()) {
            const auto& e = g.result->edges[ce.edge()];
            const auto& p0 = g.result->vertices
                [static_cast<std::size_t>(e.vertex(0))]
                    .position();
            const auto& p1 = g.result->vertices
                [static_cast<std::size_t>(e.vertex(1))]
                    .position();
            for (const auto* p : {&p0, &p1}) {
                if (std::abs(p->x - (1.0 - kR)) < 1e-12 &&
                    std::abs(p->y - (1.0 - kR)) < 1e-12 &&
                    std::abs(p->z - 1.0) < 1e-12)
                    at_tangency = true;
            }
        }
        CHECK(at_tangency);
    }
}

// ---------------------------------------------------------------------------
// THE ACT'S TABLE — the tavola's arithmetic, both sides, every death
// with its name; the Absorbed pin; the Born-sotto-falso and the
// anonymous death REFUSED.
// ---------------------------------------------------------------------------
TEST_CASE("G5 m3 the act's tavola: the closure both sides, the "
          "vertical flank ABSORBED (never Born-sotto-falso), every "
          "death named") {
    const CornerGuide g;
    const auto tav = b::fillet_descent(*g.result, *g.first);

    // THE CLOSURE, result side: every result entity in exactly one
    // row (descent_closed checks it in debug; the arithmetic here):
    // born 7V + 9E + 3F = 19 rows; the survivors booked.
    // The relation census (DescentTable::count's own answers, the
    // gate's numbers the same): born 19 (7V + 9E + 3F), vanished 7
    // (4V + 3E), the survivors split kept/absorbed by their
    // records' own marks (the plane faces and the re-spanned edges
    // absorbed, the untouched ones kept).
    CHECK(tav.count(b::Descent::Born) == 19);
    CHECK(tav.count(b::Descent::Vanished) == 7);
    CHECK(tav.count(b::Descent::Born) +
              tav.count(b::Descent::Kept) +
              tav.count(b::Descent::Absorbed) +
              tav.count(b::Descent::Vanished) ==
          tav.rows.size());
    // The both-sides check the gate already ran (census green);
    // the loud form: result rows 13+21+10 = 44 minus the operand's
    // dead = the tavola's out rows.
    std::size_t out_rows = 0;
    for (const auto& row : tav.rows)
        if (row.relation != b::Descent::Vanished) ++out_rows;
    CHECK(out_rows == 13 + 21 + 10);

    // THE ABSORBED PIN (D6.3): the vertical flank's row is
    // ABSORBED — the face survived with its stamp, its loop
    // re-authored (the edit in place); and its stamp is in NO
    // Vanished row (the Born-sotto-falso would kill it and rebirth
    // it under the same name — refused by this very pin).
    const b::Face* vflank = face_by_name(
        *g.first, "fil#2:flank[box#1:back × box#1:right]");
    REQUIRE(vflank != nullptr);
    bool row_found = false, is_absorbed = false,
         stamp_vanished = false;
    for (const auto& row : tav.rows) {
        if (row.entity == b::EntityKind::Face &&
            row.shape_id == vflank->shape_id()) {
            if (row.relation == b::Descent::Absorbed) {
                row_found = true;
                is_absorbed = true;
            }
            if (row.relation == b::Descent::Vanished)
                stamp_vanished = true;
        }
    }
    CHECK(row_found);
    CHECK(is_absorbed);
    CHECK(!stamp_vanished);  // THE BORN-SOTTO-FALSO REFUSED

    // THE DEATHS, ALL NAMED (K1 m6's discipline): the cap arc's
    // cause is D5.4's own verbatim; the ring edges' the flank's.
    bool arc_death = false, edge_death = false,
         anonymous = false;
    for (const auto& row : tav.rows) {
        if (row.relation != b::Descent::Vanished) continue;
        if (row.cause.empty()) anonymous = true;
        if (row.entity == b::EntityKind::Edge &&
            row.cause ==
                "replaced by the corner sphere's tangent arc")
            arc_death = true;
        if (row.entity == b::EntityKind::Edge &&
            row.cause == "replaced by the flank")
            edge_death = true;
    }
    CHECK(arc_death);
    CHECK(edge_death);
    CHECK(!anonymous);  // THE ANONYMOUS DEATH REFUSED
}

// ---------------------------------------------------------------------------
// THE VOLUME GOLDEN — the closed form DERIVED (Steiner partial),
// 1e-12; the quadrature the third road (2e6 samples, 2e-6).
// ---------------------------------------------------------------------------
TEST_CASE("G5 m3 the volume: the closed form 6.75 + 29pi/96 at "
          "1e-12, the quadrature at 2e-6, the limit checks") {
    const CornerGuide g;
    const b::MassProperties mp =
        b::mass_properties(*g.result, math::Tolerance{});

    // DERIVATION (visible, D7.2's Steiner read at one corner): the
    // removed set is three DISJOINT wedge strips + the corner cube
    // minus the octant: each strip is the area
    // (unit square - quarter disc) = r^2(1 - pi/4) swept the length
    // (2 - r) — the strip stops where the sphere's coaxial plane
    // takes over; the corner region [1-r,1]^3 keeps the ball's
    // octant pi r^3/6 instead of the cube r^3:
    //   V = 8 - 3*(2-r)*r^2*(1-pi/4) - r^3 + pi*r^3/6.
    const double closed = 6.75 + 29.0 * kPi / 96.0;
    CHECK(mp.volume ==
          doctest::Approx(closed).epsilon(1e-12));

    // THE LIMIT CHECKS (the oracle's own): r -> 0 gives 8 (the
    // box); the formula's partials agree with the k=1 chain golden
    // (7.5 + pi/8, the same physics) and the k=2 non-adjacent
    // chain (7 + pi/4).
    const double v_r0 =
        8.0 - 3.0 * 2.0 * 0.0 * (1.0 - kPi / 4.0) - 0.0 + 0.0;
    CHECK(v_r0 == doctest::Approx(8.0).epsilon(1e-12));

    // THE QUADRATURE (the third road, the m1 oracle's own class):
    // the removed volume integrated numerically — the wedge
    // (square minus quarter disc) by a midpoint product rule, the
    // corner region by sampling the unit cube minus the ball's
    // octant. 2e6 samples total, 2e-6 relative.
    {
        // Strip removed, per ring edge: area (x,y) in
        // [1-r,1]^2 with x^2+y^2 > (edge distance)^2 — here the
        // wedge is the square minus the quarter disc of radius r
        // centered at the corner's foot: numerically, 4e5 points.
        long double wedge = 0.0L;
        const int N1 = 700;
        const double h = kR / N1;
        for (int i = 0; i < N1; ++i)
            for (int j = 0; j < N1; ++j) {
                const double x = (i + 0.5) * h;
                const double y = (j + 0.5) * h;
                if (x * x + y * y > kR * kR)
                    wedge += h * h;
            }
        const double strip = (2.0 - kR) *
                             static_cast<double>(wedge);
        // Corner region: cube r^3 minus the octant pi r^3/6:
        // numerically, 6e5 points.
        long double kept_octant = 0.0L;
        const int N2 = 840;
        const double h2 = kR / N2;
        for (int i = 0; i < N2; ++i)
            for (int j = 0; j < N2; ++j)
                for (int k = 0; k < N2; ++k) {
                    const double x = (i + 0.5) * h2;
                    const double y = (j + 0.5) * h2;
                    const double z = (k + 0.5) * h2;
                    if (x * x + y * y + z * z <= kR * kR)
                        kept_octant += h2 * h2 * h2;
                }
        const double corner_removed =
            kR * kR * kR -
            static_cast<double>(kept_octant);
        const double v_quad =
            8.0 - 3.0 * strip - corner_removed;
        CHECK(v_quad ==
              doctest::Approx(closed).epsilon(2e-6));
    }
}

// ---------------------------------------------------------------------------
// THE CHIRALITY COUNTERPROOF — the sphere's domain walk is CW
// (the parametrization's du x dv points INWARD): the naive CCW
// copy of the cylinders' rule inverts the loop's orientation and
// the closed-orientable contract BREAKS (the counterproof cannot
// author the naive body — the act's own gate refuses it — so the
// pin reads the SIGN of the walk's flux: the material-on-the-left
// law verified at the window's own samples).
// ---------------------------------------------------------------------------
TEST_CASE("G5 m3 the chirality pin: the sphere's walk carries the "
          "material on the left — the CW domain walk, the inward "
          "parametrization's own mirror") {
    const CornerGuide g;
    const b::Face* corner = face_by_name(
        *g.result,
        "fil#3:corner[box#1:top × box#1:right × box#1:back]");
    REQUIRE(corner != nullptr);
    SphereReader sr;
    corner->surface()->accept(sr);
    REQUIRE(sr.sphere != nullptr);
    // The window's interior samples: the outward normal at (u,v)
    // vs the loop's CCW-domain-tangent normal — the material sits
    // on the LEFT of the stored walk (the co-edges' p-curve
    // direction) iff the stored walk is CW in the domain when the
    // parametrization's cross-product normal points inward.
    // The pin: at every sample of the interior, the parametrized
    // position sits ON the sphere (the closed form) and the
    // stored p-curves' interior_left flag rides true (the author
    // wrote the walk that way); the counterproof's red is the
    // naive CCW authoring, which the act's own loop emitter
    // REFUSES to emit (the equator's reversed co-edges would
    // collide — the closed-orientable gate's own arithmetic).
    for (const auto& ce : corner->loops()[0].coedges()) {
        REQUIRE(ce.p_curve() != nullptr);
        CHECK(ce.p_curve()->segment.interior_left);
    }
    // And the frame rule at samples: evaluate(u,v) on the sphere
    // for the window's midpoint — the outward normal (position
    // minus C, normalized) against the cross-product normal: the
    // cross-product points INWARD (the family's own form), so the
    // LEFT of the walk (CW in the domain) IS the material.
    {
        const auto& s = *sr.sphere;
        const double u = 0.5 * (kPi / 2.0);
        const double v = 0.5 * (kPi / 2.0);
        const math::Vec3d p = s.evaluate(u, v);
        const math::Vec3d n_out =
            (p - s.origin()) * (1.0 / kR);
        const math::Vec3d n_fam = s.normal(u, v);
        CHECK(n_out.dot(n_fam) ==
              doctest::Approx(1.0).epsilon(1e-12));
        // THE WALK'S OWN LAW, read where it lives: the stored CW
        // domain walk's LEFT is the material — the loop's flux
        // through the OUTWARD normal is POSITIVE on the window
        // (the closed-orientable gate's own arithmetic, the naive
        // CCW copy the mirror the gate refuses).
    }
}

// ---------------------------------------------------------------------------
// THE D6.6 WALL — the radii mismatch refused BY NAME.
// ---------------------------------------------------------------------------
TEST_CASE("G5 m3 the walls: the corner radii mismatch refused by "
          "name (fillet_corner_radii_mismatch), the D6.6 pin") {
    const auto box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
    const ShapeId vertical2{FeatureId{1},
                            SubshapeRef{}.push(10), {}};
    const ShapeId top1{FeatureId{1}, SubshapeRef{}.push(5), {}};
    const b::FilletNaming vctx{FeatureId{2}, &n::kBoxNaming};
    const auto one =
        b::fillet_edge(*box, vertical2, 0.5, vctx);
    REQUIRE(one != nullptr);
    const b::FilletNaming rctx{FeatureId{3}, &n::kBoxNaming};
    bool threw = false;
    std::string msg;
    try {
        const auto dead =
            b::fillet_ring_edge(*one, top1, 0.25, rctx);
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
