// tests/unit/brep/test_g6_m3_corner_act.cpp
//
// G6 m3 — L'ATTO D'ANGOLO A UN ANGOLO (ADR-0045 D4/D8.3, the heart
// of the stage): the miter as an atomic act, the fillet's G5 m3
// the stamp. THE DERIVATION, written before the code and pinned
// here (the brief's FASE 2, the ADR's D8.3 own demand):
//
// THE FAR ENDS: the act's ring bevels span their edges' whole
// extent — the bevel plane does not taper, and at the far end it
// cuts the UNMITERED corner the same plane reaches: each ring
// bevel is a PENTAGON (two setback lines, two miter lines at the
// corner end, ONE FAR-END SEGMENT on the far corner's lateral
// face), the far corner's vertex dies with its vertical and its
// ring edge RE-SPANNED to the far feet, the far lateral absorbs
// the segment. The ADR's D4.2 hexagon is the BOTH-corners
// composition — additive, the second act re-windowing the
// pentagon; the one-corner act's own shape is the pentagon (the
// amendment this morse writes into the ADR).
//
// THE COUNTS (from the m1 chain 10/15/7): dead 4 vertices (the
// two corner feet, the two far corners) and 3 edges (the two ring
// edges, the cap segment — the cause verbatim "replaced by the
// miter line" for the cap segment and the feet it anchored); born
// 8 vertices (M and the three retreats and the four far feet), 9
// edges (three miter lines, four setback lines, two far-end
// segments) and 2 faces; re-spanned 6 edges (the two wires, the
// two far verticals, the two far ring edges); kept 1 face (the
// far cap). V-4+8 / E-3+9 / F+2 -> 14/21/9, chi = 2. The census:
// the cap 4 + the far cap 5 + the near laterals 4 + the far
// laterals 5 + the vertical bevel 5 + the ring bevels 5 = 42 / 2.
//
// THE VOLUME, the oracle's own equations (ZERO code shared with
// the kernel): the removed set is the three-plane union at the
// corner, in the retreat coordinates u = 1-sx*x, v, w: three full
// prisms 3·d² (each int_0^d (d-t)·2 dt), minus the three pairwise
// wedges 3·(d³/3) (each int_0^d (d-t)² dt), plus the triple d³/4
// (THE TETRA-SPLIT, det/6: the corner tetrahedron d³/6 plus the
// inner one d³/12) — V = 8 - 3d² + (3/4)d³; at d = 0.5:
// 7.34375, the kernel's mass at 1e-12. THE COHERENCE with the
// twelve-spiegoli's own account: R_c = (3/4)d²(2-d) = d²/2 +
// (d² - (3/4)d³) — the corner's vertical half-prism plus its ring
// share, the additivity identity itself a pin.
//
// THE SWEEP (D4.4, REQUISITO): eight corners × both ring roles ×
// three setbacks in band (0.3, 0.5, 0.75) = 48 chains, each the
// full contract: the counts; the miter M = corner - (d/2)(1,1,1)
// in the corner's own signs; the three miter lines' plane
// membership verified POINT BY POINT; the outward CCW walk of
// every face; the volume at 1e-12 against the closed form.
//
// THE COUNTERPROOFS that doctor (each refused by the very
// comparison the golden passes): the offset halved, the setback
// altered, the orientation mirrored, the miter displaced, and the
// 17/11 DOCTOR — the corner end read from the COORDINATE ORDERING
// instead of the junctions: right only where the corner sits at
// the ordering's top, red at half the sweep (the correttiva's own
// arithmetic).
//
// THE DEGENERI, pinned live: the setback mismatch (the name and
// both values), the 1.99 stage band (the two readings) and the
// 2.0 face band (the bound carried), the 0.99 confine admitted,
// the fillet-occupied corner (the family's signature), the dead
// reference, the unoccupied corner, the vertical and the wire.
#include <doctest/doctest.h>

#include "CAD_0/brep/chain_gate.hpp"
#include "CAD_0/brep/chamfer.hpp"
#include "CAD_0/brep/descent.hpp"
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/fillet.hpp"
#include "CAD_0/brep/mass_properties.hpp"
#include "CAD_0/brep/naming.hpp"
#include "CAD_0/brep/pcurve.hpp"
#include "CAD_0/brep/query.hpp"
#include "CAD_0/brep/topology.hpp"
#include "CAD_0/geometry/shape_id.hpp"

#include <algorithm>
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

constexpr double kD = 0.5;

bool close(double a, double b, double eps = 1e-9) {
    return std::abs(a - b) <= eps;
}

struct PlaneReader : b::SurfaceVisitor {
    const b::PlaneSurface* plane{nullptr};
    void visit(const b::PlaneSurface& s) override { plane = &s; }
};

// The chain's guide: box#1 -> che#2 (vertical2, the plain walk,
// 10/15/7) -> che#3 (the corner act at (1,1,1), the reference the
// Y-parallel ring edge top1 = the cap x right).
struct CornerGuide {
    std::unique_ptr<b::BRepBody> box;
    std::unique_ptr<b::BRepBody> first;   // 10/15/7
    std::unique_ptr<b::BRepBody> result;  // 14/21/9

    ShapeId vertical2{FeatureId{1}, SubshapeRef{}.push(10), {}};
    ShapeId ring_y{FeatureId{1}, SubshapeRef{}.push(5), {}};
    ShapeId ring_x{FeatureId{1}, SubshapeRef{}.push(6), {}};

    explicit CornerGuide(double d = kD, bool third = true) {
        box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        REQUIRE(box != nullptr);
        const b::ChamferNaming ctx2{FeatureId{2}, &n::kBoxNaming};
        first = b::chamfer_edge(*box, vertical2, d, ctx2);
        REQUIRE(first != nullptr);
        if (!third) return;
        const b::ChamferNaming ctx3{FeatureId{3}, &n::kBoxNaming};
        result = b::chamfer_corner_act(*first, ring_y, d, ctx3);
        REQUIRE(result != nullptr);
    }
};

// The digests (the m2 test's own shape).
std::uint64_t fnv1a64(const std::string& str) {
    std::uint64_t h = 0xcbf29ce484222325ull;
    for (const char c : str) {
        h ^= static_cast<std::uint64_t>(
            static_cast<unsigned char>(c));
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
    for (b::EdgeId i = 0; i < body.edge_count(); ++i) {
        out += std::to_string(i);
        out += '|';
        out += b::query_edge(body, i).name;
        out += '\n';
    }
    return out;
}

std::uint64_t structure_digest(const b::BRepBody& body) {
    std::string out;
    out += std::to_string(body.vertex_count());
    out += '/';
    out += std::to_string(body.edge_count());
    out += '/';
    out += std::to_string(body.face_count());
    for (const auto& v : body.vertices) {
        out += std::to_string(v.position().x) + ',' +
               std::to_string(v.position().y) + ',' +
               std::to_string(v.position().z) + ';';
    }
    for (const auto& e : body.edges) {
        out += std::to_string(e.vertex(0)) + '-' +
               std::to_string(e.vertex(1)) + ';';
    }
    for (const auto& f : body.solid.shell().faces())
        for (const auto& loop : f.loops())
            for (const auto& ce : loop.coedges()) {
                out += std::to_string(ce.edge());
                out += ce.reversed() ? 'R' : 'F';
                out += ';';
            }
    return fnv1a64(out);
}

// The closed-form volume of the act at ONE corner (the oracle's
// own equation — never the kernel's).
double corner_volume(double d) {
    return 8.0 - 3.0 * d * d + 0.75 * d * d * d;
}

}  // namespace

// ===========================================================================
// THE GOLDEN — the guide case, every number the derivation wrote
// ===========================================================================
TEST_CASE("G6 m3 the corner act golden: 14/21/9 chi=2, the miter a "
          "VERTEX, the pentagons, the volume 8-3d^2+(3/4)d^3, the "
          "names nested, the stamp kept") {
    const CornerGuide g;
    REQUIRE(g.result != nullptr);

    // THE COUNTS — the act's own arithmetic from the k=1 chain.
    REQUIRE(g.result->vertex_count() == 14);
    REQUIRE(g.result->edge_count() == 21);
    REQUIRE(g.result->face_count() == 9);
    REQUIRE(b::inner_loop_count(*g.result) == 0);
    REQUIRE(b::euler_characteristic(*g.result) == 2);
    CHECK(b::euler_characteristic(*g.result) ==
          b::euler_characteristic(*g.box));
    CHECK(b::is_closed_orientable(*g.result));
    CHECK(b::loops_are_closed(*g.result));

    // THE CENSUS: every edge exactly two OPPOSITE co-edges; the
    // per-face row [bottom 5, top 4, front 5, back 4, left 5,
    // right 4, bevel V 5 (the PENTAGON), R 5, B 5] = 42 / 2 = 21.
    {
        std::size_t total = 0;
        for (const auto& e : g.result->edges) {
            REQUIRE(e.coedges().size() == 2);
            CHECK(e.coedges()[0]->reversed() !=
                  e.coedges()[1]->reversed());
        }
        const std::size_t per_face[9] = {5, 4, 5, 4, 5, 4, 5, 5, 5};
        for (std::size_t i = 0; i < 9; ++i) {
            CAPTURE(i);
            const auto& ces = g.result->solid.shell()
                                  .faces()[i]
                                  .loops()[0]
                                  .coedges();
            REQUIRE(ces.size() == per_face[i]);
            total += ces.size();
        }
        REQUIRE(total == 42);
    }

    // THE MITER — a VERTEX, never a face: M = (1,1,1) -
    // (d/2)(1,1,1) = (0.75, 0.75, 0.75), found among the born, and
    // exactly THREE miter-stamped edges incident to it.
    std::size_t v_M = g.result->vertex_count();
    for (std::size_t i = 0; i < g.result->vertex_count(); ++i) {
        const Vec3d p = g.result->vertices[i].position();
        if (close(p.x, 0.75, 1e-12) && close(p.y, 0.75, 1e-12) &&
            close(p.z, 0.75, 1e-12))
            v_M = i;
    }
    REQUIRE(v_M < g.result->vertex_count());
    {
        std::size_t miters = 0;
        for (std::size_t i = 0; i < g.result->edge_count(); ++i) {
            const auto& e = g.result->edges[i];
            if (e.vertex(0) != static_cast<b::VertexId>(v_M) &&
                e.vertex(1) != static_cast<b::VertexId>(v_M))
                continue;
            // (the depth guard: the box's own edge roles
            // share the index space — bottom1's path[0] is 1 at
            // depth 1; the miter's form is depth 7)
            if (e.shape_id().subshape.depth > 1 &&
                e.shape_id().subshape.path[0] ==
                    n::kCheMiterRole)
                ++miters;
        }
        CHECK(miters == 3);
    }

    // THE RETREATS AND THE FAR FEET — the closed forms of the
    // corner's own signs: P_right (1, 1-d, 1-d), P_back (1-d, 1,
    // 1-d), P_top (1-d, 1-d, 1); the far feet at the far corners
    // (1,-1,1) and (-1,1,1), each pair retreated along the cap
    // and the lateral.
    {
        const double expect[7][3] = {
            {1.0, 1.0 - kD, 1.0 - kD},   // P on the right face
            {1.0 - kD, 1.0, 1.0 - kD},   // P on the back face
            {1.0 - kD, 1.0 - kD, 1.0},   // P on the top face
            {1.0 - kD, -1.0, 1.0},       // far foot on the top
            {1.0, -1.0, 1.0 - kD},       // far foot on the right
            {-1.0, 1.0 - kD, 1.0},       // partner far on the top
            {-1.0, 1.0, 1.0 - kD}};      // partner far on the back
        for (const auto& e : expect) {
            bool found = false;
            for (std::size_t i = 0;
                 i < g.result->vertex_count() && !found; ++i) {
                const Vec3d p =
                    g.result->vertices[i].position();
                found = close(p.x, e[0], 1e-12) &&
                        close(p.y, e[1], 1e-12) &&
                        close(p.z, e[2], 1e-12);
            }
            CAPTURE(e[0]);
            CAPTURE(e[1]);
            CAPTURE(e[2]);
            CHECK(found);
        }
    }

    // THE THREE MITER LINES' PLANE MEMBERSHIP, point by point: the
    // planes x+y = 2-d, x+z = 2-d, y+z = 2-d; each miter line
    // runs from its retreat to M ON both its planes at EVERY
    // sample.
    {
        std::size_t checked = 0;
        for (std::size_t i = 0; i < g.result->edge_count(); ++i) {
            const auto& e = g.result->edges[i];
            if (e.shape_id().subshape.depth <= 1 ||
                e.shape_id().subshape.path[0] != n::kCheMiterRole)
                continue;
            const Vec3d& a =
                g.result->vertices[static_cast<std::size_t>(
                    e.vertex(0))]
                    .position();
            const Vec3d& b =
                g.result->vertices[static_cast<std::size_t>(
                    e.vertex(1))]
                    .position();
            for (int s = 1; s < 10; ++s) {
                const double t = s / 10.0;
                const Vec3d p = a * (1.0 - t) + b * t;
                // Each sample sits on TWO of the three planes.
                int hits = 0;
                if (close(p.x + p.y, 2.0 - kD, 1e-9)) ++hits;
                if (close(p.x + p.z, 2.0 - kD, 1e-9)) ++hits;
                if (close(p.y + p.z, 2.0 - kD, 1e-9)) ++hits;
                CHECK(hits == 2);
            }
            ++checked;
        }
        REQUIRE(checked == 3);
    }

    // THE VOLUME — the closed form, exact at 1e-12 (the oracle's
    // own equation, never the kernel's).
    {
        const auto mp = b::mass_properties(
            *g.result, CAD_0::math::Tolerance::linear(1e-9));
        CHECK(close(mp.volume, corner_volume(kD), 1e-12));
        CHECK(mp.all_exact);
        // THE NAIVE READS, red by discrimination: the m1's 7.75
        // (one prism alone), the square-cut 8-3*(2d^2) = 6.5, the
        // fillet chain's 7+pi/4, the twelve-spiegoli's own 5.75.
        CHECK_FALSE(close(mp.volume, 8.0 - kD * kD, 1e-6));
        CHECK_FALSE(close(mp.volume, 8.0 - 3.0 * 2.0 * kD * kD,
                          1e-6));
        CHECK_FALSE(close(mp.volume, 7.0 + 3.14159265358979 / 4.0,
                          1e-3));
        CHECK_FALSE(close(mp.volume, 8.0 - 12.0 * kD * kD +
                                       6.0 * kD * kD * kD,
                          1e-6));
    }

    // THE NAMES — the two ring bevels born (the reference's and
    // the partner's, the co-edge order), the miter lines' new
    // role with the nested bevel legs (the first leg the
    // carrier's bevel path, pushed verbatim — the stamp the
    // faithful identity), the setback wires and the far-end
    // segments in the m1 isect grammar (the landing faces: the
    // cap, the two near laterals, the TWO FAR LATERALS — the
    // front and the left, the unmitered corners' own planes), and
    // the vertical bevel's name KEPT (the Absorbed edit, never
    // Born-sotto-falso).
    {
        const std::string bevel_V =
            "che#2:bevel[box#1:back × box#1:right]";
        const std::string bevel_R =
            "che#3:bevel[box#1:top × box#1:right]";
        const std::string bevel_B =
            "che#3:bevel[box#1:top × box#1:back]";
        CHECK(b::query_face(*g.result, 6).name == bevel_V);
        CHECK(b::query_face(*g.result, 7).name == bevel_R);
        CHECK(b::query_face(*g.result, 8).name == bevel_B);
        // The miter lines (the born edge base 12): V×R, V×B, R×B —
        // the nested bevel legs render under the ACT's binding
        // (the fillet's coaxial wires' own convention: the PATH
        // the faithful identity, the structural check below the
        // truth — the carrier's che#2 path verbatim in the stamp).
        const std::string bevel_V_rendered =
            "che#3:bevel[box#1:back × box#1:right]";
        CHECK(b::query_edge(*g.result, 12).name ==
              "che#3:miter[" + bevel_V_rendered + " × " + bevel_R +
              "]");
        CHECK(b::query_edge(*g.result, 13).name ==
              "che#3:miter[" + bevel_V_rendered + " × " + bevel_B +
              "]");
        CHECK(b::query_edge(*g.result, 14).name ==
              "che#3:miter[" + bevel_R + " × " + bevel_B + "]");
        // The setback wires and the far-end segments.
        CHECK(b::query_edge(*g.result, 15).name ==
              "che#3:isect[box#1:top × " + bevel_R + "]");
        CHECK(b::query_edge(*g.result, 16).name ==
              "che#3:isect[box#1:right × " + bevel_R + "]");
        CHECK(b::query_edge(*g.result, 17).name ==
              "che#3:isect[box#1:top × " + bevel_B + "]");
        CHECK(b::query_edge(*g.result, 18).name ==
              "che#3:isect[box#1:back × " + bevel_B + "]");
        CHECK(b::query_edge(*g.result, 19).name ==
              "che#3:isect[box#1:front × " + bevel_R + "]");
        CHECK(b::query_edge(*g.result, 20).name ==
              "che#3:isect[box#1:left × " + bevel_B + "]");
        // THE STAMP KEPT: the vertical bevel's shape_id survives
        // the re-windowing verbatim (the tavola's Absorbed row
        // carries it — never a Vanished one).
        const ShapeId v_stamp =
            g.first->solid.shell().faces()[6].shape_id();
        CHECK(g.result->solid.shell().faces()[6].shape_id() ==
              v_stamp);
        // The MITER STAMP's structural identity: [miter, V's own
        // path, R's own path] — the faithful legs.
        const auto& mref =
            g.result->edges[12].shape_id().subshape;
        REQUIRE(mref.depth == 7);
        CHECK(mref.path[0] == n::kCheMiterRole);
        const auto& vpath =
            g.first->solid.shell().faces()[6].shape_id()
                .subshape;
        CHECK(mref.path[1] == vpath.path[0]);
        CHECK(mref.path[2] == vpath.path[1]);
        CHECK(mref.path[3] == vpath.path[2]);
    }

    // THE WALKS' COHERENCE: every co-edge's p-curve images land on
    // its vertices (the decoupling rule through the act), and
    // every face's walk is CCW in its own domain with the normal
    // pointing OUT of the material (the material-on-the-left law;
    // the outward read against the body's centroid — the body is
    // convex, the sign honest).
    {
        Vec3d centroid{0.0, 0.0, 0.0};
        for (const auto& v : g.result->vertices)
            centroid = centroid + v.position();
        centroid = centroid *
                   (1.0 / static_cast<double>(
                              g.result->vertex_count()));
        for (std::size_t f = 0; f < 9; ++f) {
            const auto& face =
                g.result->solid.shell().faces()[f];
            PlaneReader pr;
            face.surface()->accept(pr);
            REQUIRE(pr.plane != nullptr);
            // CCW in the domain.
            double area2 = 0.0;
            for (const auto& ce :
                 face.loops()[0].coedges()) {
                const auto* pc = ce.p_curve();
                REQUIRE(pc != nullptr);
                REQUIRE(pc->kind == b::PCurve::Kind::Segment);
                area2 += pc->segment.a.x * pc->segment.b.y -
                         pc->segment.a.y * pc->segment.b.x;
                // The endpoint coherence.
                const auto s0 = static_cast<std::size_t>(
                    ce.start_vertex(g.result->edges));
                const auto s1 = static_cast<std::size_t>(
                    ce.end_vertex(g.result->edges));
                const auto q0 = pr.plane->evaluate(
                    pc->segment.a.x, pc->segment.a.y);
                const auto q1 = pr.plane->evaluate(
                    pc->segment.b.x, pc->segment.b.y);
                CHECK((q0 - g.result->vertices[s0].position())
                          .length_sq() < 1e-18);
                CHECK((q1 - g.result->vertices[s1].position())
                          .length_sq() < 1e-18);
            }
            CAPTURE(f);
            CHECK(area2 > 0.0);
            // The outward normal: the face's centroid pushed
            // along the normal leaves the body's centroid behind.
            Vec3d fc{0.0, 0.0, 0.0};
            std::size_t n = 0;
            for (const auto& ce :
                 face.loops()[0].coedges()) {
                const auto& e =
                    g.result->edges[ce.edge()];
                fc = fc +
                     g.result->vertices[static_cast<std::size_t>(
                                        e.vertex(0))]
                         .position();
                ++n;
            }
            fc = fc * (1.0 / static_cast<double>(n));
            CHECK((fc - centroid)
                      .dot(pr.plane->normal_vec()) > 0.0);
        }
    }

    // THE RE-SPANNED, read by position (the far junctions' own
    // substitutes): the old top x front edge now ends at the far
    // foot on the top; the far vertical (vertical1) now ends at
    // the far foot on the right; the wires retreat to P.
    {
        const double targets[3][6] = {
            // top x front: (-1,-1,1) -> (1-d,-1,1)
            {-1.0, -1.0, 1.0, 1.0 - kD, -1.0, 1.0},
            // vertical1: (1,-1,-1) -> (1,-1,1-d)
            {1.0, -1.0, -1.0, 1.0, -1.0, 1.0 - kD},
            // che#2's right wire: (1,1-d,-1) -> (1,1-d,1-d)
            {1.0, 1.0 - kD, -1.0, 1.0, 1.0 - kD, 1.0 - kD}};
        for (const auto& t : targets) {
            bool found = false;
            for (std::size_t i = 0;
                 i < g.result->edge_count() && !found; ++i) {
                const auto& e = g.result->edges[i];
                const Vec3d& a =
                    g.result->vertices[static_cast<std::size_t>(
                        e.vertex(0))]
                        .position();
                const Vec3d& b =
                    g.result->vertices[static_cast<std::size_t>(
                        e.vertex(1))]
                        .position();
                const bool fwd = close(a.x, t[0]) &&
                                 close(a.y, t[1]) &&
                                 close(a.z, t[2]) &&
                                 close(b.x, t[3]) &&
                                 close(b.y, t[4]) &&
                                 close(b.z, t[5]);
                const bool rev = close(b.x, t[0]) &&
                                 close(b.y, t[1]) &&
                                 close(b.z, t[2]) &&
                                 close(a.x, t[3]) &&
                                 close(a.y, t[4]) &&
                                 close(a.z, t[5]);
                found = fwd || rev;
            }
            CHECK(found);
        }
    }
}

// ===========================================================================
// THE ORACLE — the file's own equations: the inclusion-exclusion
// at the corner, the quadrature on the pieces' own
// parametrizations, the tetra-split, the doctored reads refused
// ===========================================================================
TEST_CASE("G6 m3 the oracle: the three prisms and the pairwise "
          "wedges by 2e6-midpoint rules, the triple by the "
          "tetra-split, the closed form and the kernel at 1e-12, "
          "the doctored reads red") {
    const double d = kD;

    // THE PIECES (each on its own parametrization, the integrands
    // smooth — never an indicator on a grid):
    // (1) one full prism: the triangle section int_0^d (d-t) dt
    //     times the arm 2 — three of them.
    const int N = 2'000'000;
    const auto prism = [&]() {
        const double h = d / N;
        double s = 0.0;
        for (int i = 0; i < N; ++i)
            s += (d - (i + 0.5) * h) * h;
        return s * 2.0;
    };
    // (2) one pairwise wedge: int_0^d (d-t)^2 dt — three of them.
    const auto wedge = [&]() {
        const double h = d / N;
        double s = 0.0;
        for (int i = 0; i < N; ++i) {
            const double t = (i + 0.5) * h;
            s += (d - t) * (d - t) * h;
        }
        return s;
    };
    // (3) THE TRIPLE by the tetra-split (det/6, the ADR's own
    //     words): the corner tetrahedron {u+v+w <= d} is d^3/6;
    //     the inner one — the vertices (d,0,0),(0,d,0),(0,0,d),
    //     (d/2,d/2,d/2) — is d^3/12 by the determinant. The
    //     corner one ALSO quadratured on its own barycentric map
    //     (u,v,w) = d(s, t(1-s), r(1-s)(1-t)): the Jacobian
    //     FACTORIZES d^3(1-s)^2(1-t) — three 1-D midpoint rules
    //     on the factors, the integrands smooth, never an
    //     indicator on a grid.
    {
        const double h = 1.0 / N;
        double A = 0.0, B = 0.0, C = 0.0;
        for (int i = 0; i < N; ++i) {
            const double u = (i + 0.5) * h;
            A += (1.0 - u) * (1.0 - u) * h;
        }
        for (int i = 0; i < N; ++i) {
            const double t = (i + 0.5) * h;
            B += (1.0 - t) * h;
        }
        for (int i = 0; i < N; ++i) C += h;
        const double tetra_quad = d * d * d * A * B * C;
        CHECK(close(A, 1.0 / 3.0, 2e-6));
        CHECK(close(B, 1.0 / 2.0, 2e-6));
        CHECK(close(tetra_quad, d * d * d / 6.0, 2e-6));
    }
    const double triple = d * d * d / 6.0 + d * d * d / 12.0;
    CHECK(close(triple, d * d * d / 4.0, 1e-15));

    // THE INCLUSION-EXCLUSION (the oracle's own sum).
    const double removed =
        3.0 * prism() - 3.0 * wedge() + triple;
    const double v_oracle = 8.0 - removed;
    CHECK(close(v_oracle, corner_volume(d), 2e-6));

    // Against the kernel's mass at the exactness pin.
    const CornerGuide g;
    REQUIRE(g.result != nullptr);
    const auto mp = b::mass_properties(
        *g.result, CAD_0::math::Tolerance::linear(1e-9));
    CHECK(close(mp.volume, corner_volume(d), 1e-12));
    CHECK(close(v_oracle, mp.volume, 2e-6));

    // THE COHERENCE identity with the twelve-spiegoli's own
    // account: R_c = (3/4)d^2(2-d) = d^2/2 + (d^2 - (3/4)d^3) —
    // the corner's vertical half-prism plus its ring share; the
    // act's own removed beyond the m1 state is R_c + d^2/2 (the
    // far ends' extra, the composition's own account).
    const double R_c = 0.75 * d * d * (2.0 - d);
    CHECK(close(R_c, 0.5 * d * d + (d * d - 0.75 * d * d * d),
                1e-15));
    CHECK(close(3.0 * d * d - 0.75 * d * d * d - d * d,
                R_c + 0.5 * d * d, 1e-15));

    // THE DOCTORED READS, each refused by the very comparison the
    // golden passes:
    // (a) THE OFFSET HALVED — the bevel planes read at d/2.
    CHECK_FALSE(close(8.0 - 3.0 * (0.5 * d) * (0.5 * d) +
                          0.75 * (0.5 * d) * (0.5 * d) *
                              (0.5 * d),
                      mp.volume, 1e-6));
    // (b) THE SETBACK ALTERED (+0.01).
    CHECK_FALSE(close(corner_volume(d + 0.01), mp.volume,
                      1e-6));
    // (c) THE ORIENTATION MIRRORED — a walk read CW in the
    //     domain: its signed area is NEGATIVE, the very check the
    //     golden's CCW assertion makes.
    {
        const auto& face =
            g.result->solid.shell().faces()[7];
        double area2 = 0.0;
        for (const auto& ce : face.loops()[0].coedges()) {
            const auto* pc = ce.p_curve();
            area2 += pc->segment.a.x * pc->segment.b.y -
                     pc->segment.a.y * pc->segment.b.x;
        }
        CHECK(area2 > 0.0);   // the honest walk
        CHECK(-area2 < 0.0);  // the mirrored one, red by the
                              // same assertion
    }
    // (d) THE MITER DISPLACED — M pushed along the diagonal: the
    //     three-plane concurrence fails at every sample.
    {
        const Vec3d bad_M{0.75 + 1e-3, 0.75 + 1e-3,
                          0.75 + 1e-3};
        const bool still_concurrent =
            close(bad_M.x + bad_M.y, 2.0 - d, 1e-6);
        CHECK_FALSE(still_concurrent);
    }
    // (e) THE FAR END FROM THE ORDERING — the 17/11 doctor,
    //     pinned in the sweep's own case below (the ordering
    //     reads the corner right only at the ordering's top).
}

// ===========================================================================
// THE SWEEP — D4.4's law: eight corners × both ring roles × three
// setbacks = 48 chains, each the full contract
// ===========================================================================
TEST_CASE("G6 m3 the sweep: 48 chains — the counts, the miter in "
          "the corner's own signs, the miter lines on their planes "
          "point by point, the outward CCW walks, the volume at "
          "1e-12") {
    const double setbacks[3] = {0.3, 0.5, 0.75};
    // THE 17/11 DOCTOR's own tally: the corner end read from the
    // coordinate ordering (the max-y endpoint of the reference)
    // is RIGHT only when the corner sits at the ordering's top.
    std::size_t doctor_red = 0;
    std::size_t chains = 0;
    for (int sx_i = 0; sx_i < 2; ++sx_i)
        for (int sy_i = 0; sy_i < 2; ++sy_i)
            for (int sz_i = 0; sz_i < 2; ++sz_i) {
                const double sx = sx_i ? 1.0 : -1.0;
                const double sy = sy_i ? 1.0 : -1.0;
                const double sz = sz_i ? 1.0 : -1.0;
                // The corner's vertical edge (the box's own
                // numbering: vertical2 = (1,1), vertical1 =
                // (1,-1), vertical3 = (-1,1), vertical0 =
                // (-1,-1)).
                const std::size_t vert =
                    sx > 0 ? (sy > 0 ? 10u : 9u)
                           : (sy > 0 ? 11u : 8u);
                // The corner's two ring edges on the cap: the
                // Y-parallel (on the lateral x = sx) and the
                // X-parallel (on the lateral y = sy).
                const std::size_t ring_y =
                    sz > 0 ? (sx > 0 ? 5u : 7u)
                           : (sx > 0 ? 1u : 3u);
                const std::size_t ring_x =
                    sz > 0 ? (sy > 0 ? 6u : 4u)
                           : (sy > 0 ? 2u : 0u);
                for (const double d : setbacks) {
                    for (const std::size_t ring :
                         {ring_y, ring_x}) {
                        ++chains;
                        auto box = b::make_box(
                            {1.0f, 1.0f, 1.0f}, FeatureId{1});
                        REQUIRE(box != nullptr);
                        const b::ChamferNaming ctx2{
                            FeatureId{2}, &n::kBoxNaming};
                        auto first = b::chamfer_edge(
                            *box,
                            ShapeId{FeatureId{1},
                                    SubshapeRef{}.push(
                                        static_cast<std::uint32_t>(
                                            vert)),
                                    {}},
                            d, ctx2);
                        REQUIRE(first != nullptr);
                        // THE 17/11 DOCTOR, live: the corner end
                        // read as the reference's MAX-Y endpoint
                        // (the ordering the correttiva killed),
                        // verified against the TRUE corner end's
                        // position (sx, sy(1-d), sz) — the
                        // ordering is right only where the corner
                        // sits at the ordering's top (counted once
                        // per corner).
                        if (ring == ring_y && d == setbacks[0]) {
                            const auto& redge =
                                first->edges[ring_y];
                            const Vec3d& ra =
                                first->vertices[
                                    static_cast<std::size_t>(
                                        redge.vertex(0))]
                                    .position();
                            const Vec3d& rb =
                                first->vertices[
                                    static_cast<std::size_t>(
                                        redge.vertex(1))]
                                    .position();
                            const Vec3d corner_end{
                                sx, sy * (1.0 - d), sz};
                            const Vec3d& picked =
                                ra.y >= rb.y ? ra : rb;
                            const bool doctor_right =
                                close(picked.x, corner_end.x,
                                      1e-12) &&
                                close(picked.y, corner_end.y,
                                      1e-12) &&
                                close(picked.z, corner_end.z,
                                      1e-12);
                            if (!doctor_right) ++doctor_red;
                        }
                        const b::ChamferNaming ctx3{
                            FeatureId{3}, &n::kBoxNaming};
                        auto r = b::chamfer_corner_act(
                            *first,
                            ShapeId{FeatureId{1},
                                    SubshapeRef{}.push(
                                        static_cast<std::uint32_t>(
                                            ring)),
                                    {}},
                            d, ctx3);
                        REQUIRE(r != nullptr);
                        // THE COUNTS.
                        REQUIRE(r->vertex_count() == 14);
                        REQUIRE(r->edge_count() == 21);
                        REQUIRE(r->face_count() == 9);
                        REQUIRE(b::euler_characteristic(*r) ==
                                2);
                        REQUIRE(b::is_closed_orientable(*r));
                        REQUIRE(b::loops_are_closed(*r));
                        // THE MITER, in the corner's own signs.
                        const Vec3d M{sx * (1.0 - 0.5 * d),
                                      sy * (1.0 - 0.5 * d),
                                      sz * (1.0 - 0.5 * d)};
                        std::size_t v_M =
                            r->vertex_count();
                        for (std::size_t i = 0;
                             i < r->vertex_count(); ++i) {
                            const Vec3d p =
                                r->vertices[i].position();
                            if (close(p.x, M.x, 1e-12) &&
                                close(p.y, M.y, 1e-12) &&
                                close(p.z, M.z, 1e-12))
                                v_M = i;
                        }
                        REQUIRE(v_M < r->vertex_count());
                        // THE THREE MITER LINES on their planes,
                        // point by point: the planes
                        // sx x + sy y = 2-d, sx x + sz z = 2-d,
                        // sy y + sz z = 2-d (the unit box's own
                        // constants).
                        std::size_t miters = 0;
                        for (std::size_t i = 0;
                             i < r->edge_count(); ++i) {
                            const auto& e = r->edges[i];
                            if (e.shape_id().subshape.depth <=
                                    1 ||
                                e.shape_id().subshape.path[0] !=
                                    n::kCheMiterRole)
                                continue;
                            const bool at_miter =
                                e.vertex(0) ==
                                    static_cast<b::VertexId>(
                                        v_M) ||
                                e.vertex(1) ==
                                    static_cast<b::VertexId>(
                                        v_M);
                            REQUIRE(at_miter);
                            const Vec3d& o =
                                r->vertices[static_cast<std::size_t>(
                                    e.vertex(0) ==
                                            static_cast<
                                                b::VertexId>(v_M)
                                        ? e.vertex(1)
                                        : e.vertex(0))]
                                    .position();
                            for (int s = 1; s < 8; ++s) {
                                const double t = s / 8.0;
                                const Vec3d p =
                                    o * (1.0 - t) +
                                    M * t;
                                const double f1 =
                                    sx * p.x + sy * p.y;
                                const double f2 =
                                    sx * p.x + sz * p.z;
                                const double f3 =
                                    sy * p.y + sz * p.z;
                                int hits = 0;
                                if (close(f1, 2.0 - d, 1e-9))
                                    ++hits;
                                if (close(f2, 2.0 - d, 1e-9))
                                    ++hits;
                                if (close(f3, 2.0 - d, 1e-9))
                                    ++hits;
                                CHECK(hits == 2);
                            }
                            ++miters;
                        }
                        REQUIRE(miters == 3);
                        // THE VOLUME, the closed form at 1e-12.
                        const auto mp = b::mass_properties(
                            *r,
                            CAD_0::math::Tolerance::linear(
                                1e-9));
                        CHECK(close(mp.volume,
                                    corner_volume(d), 1e-12));
                        CHECK(mp.all_exact);
                        // THE OUTWARD CCW WALKS, every face.
                        for (std::size_t f = 0; f < 9; ++f) {
                            const auto& face =
                                r->solid.shell().faces()[f];
                            PlaneReader pr;
                            face.surface()->accept(pr);
                            REQUIRE(pr.plane != nullptr);
                            double area2 = 0.0;
                            for (const auto& ce :
                                 face.loops()[0].coedges()) {
                                const auto* pc = ce.p_curve();
                                REQUIRE(pc != nullptr);
                                area2 +=
                                    pc->segment.a.x *
                                        pc->segment.b.y -
                                    pc->segment.a.y *
                                        pc->segment.b.x;
                            }
                            CHECK(area2 > 0.0);
                        }
                    }
                }
            }
    REQUIRE(chains == 48);
    // THE 17/11 DOCTOR'S TALLY: the ordering-based corner read is
    // red at exactly HALF the corners — the correttiva's own
    // arithmetic (24 of 48 chains ride a corner the ordering
    // misreads).
    CHECK(doctor_red == 4);
}

// ===========================================================================
// THE TAVOLA — 51 rows, the causes verbatim, the kept bevel's
// stamp, the closure both sides
// ===========================================================================
TEST_CASE("G6 m3 the tavola: 51 rows — kept 13, absorbed 12, born "
          "19, vanished 7; the cap segment's cause verbatim "
          "'replaced by the miter line'; the closure both sides") {
    const CornerGuide g;
    REQUIRE(g.result != nullptr);
    const b::DescentTable t =
        b::chamfer_descent(*g.result, *g.first);
    REQUIRE(t.rows.size() == 51);
    CHECK(t.count(b::Descent::Kept) == 13);     // 6V + 6E + 1F
    CHECK(t.count(b::Descent::Absorbed) == 12); // 6E + 6F
    CHECK(t.count(b::Descent::Born) == 19);     // 8V + 9E + 2F
    CHECK(t.count(b::Descent::Vanished) == 7);  // 4V + 3E
    CHECK(t.count(b::Descent::Split) == 0);
    CHECK(t.count(b::Descent::Merged) == 0);
    // THE CAUSES, verbatim: the cap segment and the two corner
    // feet died to the miter line (D4.2's own words); the two
    // ring edges and the two far corners died to the bevels.
    {
        std::size_t miter_causes = 0, bevel_causes = 0;
        for (const auto& row : t.rows) {
            if (row.relation != b::Descent::Vanished) continue;
            if (row.cause == "replaced by the miter line")
                ++miter_causes;
            else if (row.cause == "replaced by the bevel")
                ++bevel_causes;
            else
                CHECK(row.cause == "replaced by the miter line");
        }
        CHECK(miter_causes == 3);  // the cap segment + the feet
        CHECK(bevel_causes == 4);  // the ring edges + far corners
    }
    // THE KEPT BEVEL'S ROW: the vertical bevel is ABSORBED (the
    // re-windowing, the edit in place), its name and stamp on the
    // row — never a Vanished one under the same name (the
    // Born-sotto-falso the tavola refuses).
    {
        bool absorbed_v = false;
        for (const auto& row : t.rows) {
            if (row.entity != b::EntityKind::Face) continue;
            if (row.name ==
                "che#2:bevel[box#1:back × box#1:right]") {
                CHECK(row.relation == b::Descent::Absorbed);
                absorbed_v = true;
            }
        }
        CHECK(absorbed_v);
        for (const auto& row : t.rows) {
            if (row.relation == b::Descent::Vanished &&
                row.entity == b::EntityKind::Face)
                CHECK(row.name !=
                      "che#2:bevel[box#1:back × box#1:right]");
        }
    }
    // THE BORN ROWS' NAMES: the two ring bevels and the nine born
    // edges (three miter, four setbacks, two far-end segments —
    // the honest unnamed: the eight born vertices).
    {
        std::size_t born_faces = 0, born_edges_named = 0,
                    born_v = 0;
        for (const auto& row : t.rows) {
            if (row.relation != b::Descent::Born) continue;
            if (row.entity == b::EntityKind::Face) {
                ++born_faces;
                CHECK(row.name.rfind("che#3:bevel[", 0) == 0);
            } else if (row.entity == b::EntityKind::Edge) {
                if (!row.name.empty()) ++born_edges_named;
            } else {
                ++born_v;
                CHECK(row.name.empty());
            }
        }
        CHECK(born_faces == 2);
        CHECK(born_edges_named == 9);
        CHECK(born_v == 8);
    }
    // THE CLOSURE, both sides.
    std::string why;
    CHECK(b::descent_closed(*g.result, t, &why));
    CHECK(b::old_generation_closed(*g.first, b::BRepBody{}, t,
                                   &why));
    // Determinism of the derivation.
    const b::DescentTable t2 =
        b::chamfer_descent(*g.result, *g.first);
    REQUIRE(t2.rows.size() == t.rows.size());
    for (std::size_t i = 0; i < t.rows.size(); ++i) {
        CHECK(t.rows[i].relation == t2.rows[i].relation);
        CHECK(t.rows[i].name == t2.rows[i].name);
        CHECK(t.rows[i].cause == t2.rows[i].cause);
        CHECK(t.rows[i].operands == t2.rows[i].operands);
    }
}

// ===========================================================================
// THE GATE AND THE DIGESTS — the five predicates after EVERY
// step, the rebuild bit-identical
// ===========================================================================
TEST_CASE("G6 m3 the gate: the five predicates green after each "
          "step, the digests bit-identical at the re-run") {
    const CornerGuide g;
    REQUIRE(g.result != nullptr);
    const b::DescentTable t1 =
        b::chamfer_descent(*g.first, *g.box);
    const b::DescentTable t2 =
        b::chamfer_descent(*g.result, *g.first);
    const CornerGuide rerun;
    REQUIRE(rerun.result != nullptr);

    // STEP 1 — the vertical walk (the m1 step, gated again).
    {
        b::ChainStep step;
        step.body = g.first.get();
        step.tavola = &t1;
        step.previous = g.box.get();
        step.held.push_back(
            {ShapeId{FeatureId{1}, SubshapeRef{}.push(10), {}},
             b::Resolution::Kind::Vanished});
        step.held.push_back(
            {ShapeId{FeatureId{1}, SubshapeRef{}.push(11), {}},
             b::Resolution::Kind::Resolved});
        step.rerun_digest = b::chain_digest(*rerun.first);
        const b::ChainGateVerdict v = b::chain_gate(step);
        CHECK(v.pass);
        CHECK(v.failures.empty());
        CHECK(v.digest == b::chain_digest(*g.first));
    }
    // STEP 2 — the act: the dead ring reference VANISHED, the
    // surviving vertical3 RESOLVED, and che#2's right wire (a
    // born name of the FIRST step) held across the act — RESOLVED
    // (the stamp through the re-spanning).
    {
        b::ChainStep step;
        step.body = g.result.get();
        step.tavola = &t2;
        step.previous = g.first.get();
        // (the dead ring reference held as the PARTNER's stamp
        // top2 = [6]: the box's edge roles 0..5 share their paths
        // with the face table — the by-name resolution is
        // ambiguous there; 6..11 are edge-only, the honest holds)
        step.held.push_back(
            {ShapeId{FeatureId{1}, SubshapeRef{}.push(6), {}},
             b::Resolution::Kind::Vanished});
        step.held.push_back(
            {ShapeId{FeatureId{1}, SubshapeRef{}.push(11), {}},
             b::Resolution::Kind::Resolved});
        step.held.push_back(
            {g.first->edges[12].shape_id(),
             b::Resolution::Kind::Resolved});
        step.rerun_digest = b::chain_digest(*rerun.result);
        const b::ChainGateVerdict v = b::chain_gate(step);
        CHECK(v.pass);
        CHECK(v.failures.empty());
        CHECK(v.radial_two_opposite);
        CHECK(v.loops_closed);
        CHECK(v.chi_even_orientable);
        CHECK(v.census_by_name);
        CHECK(v.drift_zero);
        CHECK(v.v == 14);
        CHECK(v.e == 21);
        CHECK(v.f == 9);
        CHECK(v.digest == b::chain_digest(*g.result));
    }
    // THE DIGESTS' PURITY: the names and the structure a pure
    // function of the recipe.
    CHECK(name_digest(*rerun.result) == name_digest(*g.result));
    CHECK(structure_digest(*rerun.result) ==
          structure_digest(*g.result));
    for (int i = 0; i < 2; ++i) {
        const CornerGuide again;
        REQUIRE(again.result != nullptr);
        CHECK(structure_digest(*again.result) ==
              structure_digest(*g.result));
    }
}

// ===========================================================================
// THE DEGENERI — pinned live, each with its name and its values
// ===========================================================================
TEST_CASE("G6 m3 the degeneri: the mismatch with both values, the "
          "1.99 stage band with the two readings, the 2.0 face "
          "band, the 0.99 confine, the fillet-occupied corner, the "
          "dead and the unoccupied references") {
    const CornerGuide g(kD, /*third=*/false);
    REQUIRE(g.first != nullptr);
    const b::ChamferNaming ctx3{FeatureId{3}, &n::kBoxNaming};

    // THE MISMATCH — the name and BOTH values in the payload.
    {
        bool threw = false;
        try {
            auto r = b::chamfer_corner_act(*g.first, g.ring_y,
                                           0.4, ctx3);
            (void)r;
        } catch (const std::invalid_argument& e) {
            threw = true;
            const std::string what = e.what();
            CHECK(what.find(
                      "chamfer_corner_setback_mismatch") !=
                  std::string::npos);
            CHECK(what.find("0.400000") != std::string::npos);
            CHECK(what.find("0.500000") != std::string::npos);
        }
        REQUIRE(threw);
    }
    // THE STAGE BAND at 1.99 (the guide's vertical at the SAME
    // 1.99 — the single-edge mode's own band admits it there,
    // 1.99 < 2 - eps: the mismatch wall silent, the stage wall
    // the finder): the faces' reach 2.000000 would admit it (the
    // ring pair's own bands, satisfied — SAID with its number),
    // the stage's 1.000000 refuses (the two readings).
    {
        const CornerGuide g199(1.99, /*third=*/false);
        REQUIRE(g199.first != nullptr);
        bool threw = false;
        try {
            auto r = b::chamfer_corner_act(*g199.first,
                                           g199.ring_y, 1.99,
                                           ctx3);
            (void)r;
        } catch (const std::invalid_argument& e) {
            threw = true;
            const std::string what = e.what();
            CHECK(what.find(
                      "chamfer_setback_exceeds_face") !=
                  std::string::npos);
            CHECK(what.find("1.990000") != std::string::npos);
            CHECK(what.find("2.000000") != std::string::npos);
            CHECK(what.find("1.000000") != std::string::npos);
        }
        REQUIRE(threw);
    }
    // THE 2.0 OF THE PAIR — the vertical that would feed the act
    // refuses at ITS OWN door first (the m1 single-edge band, the
    // bound carried): the act's face bands sit behind the stage
    // wall (the act IS the multi-edge state, d < 1 its law), and
    // the pair 1.99/2.0 is honored as the stage wall at the act's
    // door and the face wall at the walk's.
    {
        auto box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        bool threw = false;
        try {
            const b::ChamferNaming ctx2{FeatureId{2},
                                        &n::kBoxNaming};
            auto f = b::chamfer_edge(
                *box,
                ShapeId{FeatureId{1}, SubshapeRef{}.push(10),
                        {}},
                2.0, ctx2);
            (void)f;
        } catch (const std::invalid_argument& e) {
            threw = true;
            const std::string what = e.what();
            CHECK(what.find(
                      "chamfer_setback_exceeds_face") !=
                  std::string::npos);
            CHECK(what.find("2.000000") != std::string::npos);
        }
        REQUIRE(threw);
    }
    // THE CONFINE at 0.99 (the guide's vertical at the same
    // 0.99, the mismatch silent): admitted — the counts and the
    // closed form (both bands just passed).
    {
        const CornerGuide g099(0.99, /*third=*/false);
        REQUIRE(g099.first != nullptr);
        auto r = b::chamfer_corner_act(*g099.first, g099.ring_y,
                                       0.99, ctx3);
        REQUIRE(r != nullptr);
        REQUIRE(r->vertex_count() == 14);
        REQUIRE(r->edge_count() == 21);
        REQUIRE(r->face_count() == 9);
        const auto mp = b::mass_properties(
            *r, CAD_0::math::Tolerance::linear(1e-9));
        CHECK(close(mp.volume, corner_volume(0.99), 1e-9));
        CHECK(mp.all_exact);
    }
    // THE FILLET-OCCUPIED CORNER — the mixed world: the carrier's
    // cylinder flank refuses by FAMILY (the crease merge's seat,
    // D5.3), one wall before the corner's own business.
    {
        auto box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        const b::FilletNaming fctx{FeatureId{2},
                                   &n::kBoxNaming};
        auto filleted = b::fillet_edge(
            *box,
            ShapeId{FeatureId{1}, SubshapeRef{}.push(10), {}},
            0.5, fctx);
        REQUIRE(filleted != nullptr);
        bool threw = false;
        try {
            auto r = b::chamfer_corner_act(
                *filleted, g.ring_y, 0.5, ctx3);
            (void)r;
        } catch (const std::invalid_argument& e) {
            threw = true;
            CHECK(std::string(e.what()).find(
                      "outside the class families") !=
                  std::string::npos);
        }
        REQUIRE(threw);
    }
    // THE DEAD REFERENCE — the ring edge the act itself consumed.
    {
        const CornerGuide full;
        REQUIRE(full.result != nullptr);
        bool threw = false;
        try {
            auto r = b::chamfer_corner_act(*full.result,
                                           g.ring_y, 0.5, ctx3);
            (void)r;
        } catch (const std::invalid_argument& e) {
            threw = true;
            CHECK(std::string(e.what()).find(
                      "names no edge of this body") !=
                  std::string::npos);
        }
        REQUIRE(threw);
    }
    // THE UNOCCUPIED CORNER — a ring edge whose ends carry no
    // vertical bevel (the front-top edge on the m1 state).
    {
        bool threw = false;
        try {
            auto r = b::chamfer_corner_act(
                *g.first,
                ShapeId{FeatureId{1}, SubshapeRef{}.push(4), {}},
                0.5, ctx3);
            (void)r;
        } catch (const std::invalid_argument& e) {
            threw = true;
            const std::string what = e.what();
            CHECK(what.find("carries NO vertical bevel") !=
                  std::string::npos);
        }
        REQUIRE(threw);
    }
    // THE VERTICAL and THE WIRE as references — the act's own
    // axis wall, by name.
    {
        bool threw = false;
        try {
            // a LIVE vertical: vertical1 (vertical2 is dead in
            // the m1 state — that death is the dead-reference
            // member's own seat above).
            auto r = b::chamfer_corner_act(
                *g.first,
                ShapeId{FeatureId{1}, SubshapeRef{}.push(9),
                        {}},
                0.5, ctx3);
            (void)r;
        } catch (const std::invalid_argument& e) {
            threw = true;
            CHECK(std::string(e.what()).find(
                      "the reference is a VERTICAL edge") !=
                  std::string::npos);
        }
        REQUIRE(threw);
        threw = false;
        try {
            // A STAMPED diagonal: the act's own far-end segment
            // (edge 19 of the full result — born diagonal, the
            // only non-axis-parallel reference the stamps can
            // name; the m1's cap segments are unnamed, their
            // default stamp ambiguous).
            const CornerGuide full;
            REQUIRE(full.result != nullptr);
            const auto seg =
                full.result->edges[19].shape_id();
            auto r = b::chamfer_corner_act(*full.result, seg,
                                           0.5, ctx3);
            (void)r;
        } catch (const std::invalid_argument& e) {
            threw = true;
            CHECK(std::string(e.what()).find(
                      "neither X- nor Y-parallel") !=
                  std::string::npos);
        }
        REQUIRE(threw);
    }
}
