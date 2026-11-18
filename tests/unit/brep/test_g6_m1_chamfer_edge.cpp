// tests/unit/brep/test_g6_m1_chamfer_edge.cpp
//
// G6 m1 — LO SPIGOLO VERTICALE SINGOLO DEL BOX, IL BEVEL PIANO
// (ADR-0045 D1.2/D2/D8.1, the chamfer stage 1's first bite): the
// flat bevel — the second feature that SUBSTITUTES instead of
// subtracting, every contact a LINE, zero intersection solves.
// This file pins:
//
//   * THE GOLDEN — the walk's every number: 10/15/7, H = 0,
//     chi = 2 (THE SAME COUNTS as the fillet k=1 — the structural
//     coherence pin, read side by side: the same four born
//     POSITIONS at equal setback and radius, the boundary between
//     them the different geometry — the straight segment where the
//     fillet rolled its quarter arc), the census 30/2, the closed
//     positions of the four born feet, the volume V = 8 - d^2 =
//     7.75 exact at d = 0.5, the bevel's plane (the bisector
//     (1,1,0)/sqrt(2) through (0.75, 0.75, 0)), the window the
//     rectangle [−1,1] x [−d/sqrt(2), d/sqrt(2)] in the bevel's own
//     frame, every p-curve a Line2;
//   * THE ORACLE (the m3 law, verbatim: equations proprie, mai
//     dalla produzione) — the volume re-derived by THIS FILE's own
//     midpoint rule over the removed prism's own parametrization
//     (the cross-section int_0^d (d-u) du, the arm 2 — 2e6
//     midpoints), against the closed form at the declared 2e-6 and
//     against the kernel's mass at 1e-12; THE COUNTERPROOFS that
//     doctor — the offset altered (the setback halved), the
//     setback altered (+0.01), the naive reads (no-cut 8, the
//     square-cut 8-2d^2, the fillet's own 7.5+pi/8), the
//     orientation flipped (the reversed walk's signed area) — every
//     one REFUSED by the very comparisons the golden passes;
//   * THE NAMES — che#F:bevel[box#1:back × box#1:right] and the
//     setback wires che#F:isect[<landing> × <the bevel>], the
//     chamfer segments honestly unnamed, the shrink faces'
//     inheritance, the unnamed mode, the digest's purity (the names
//     a pure function of the recipe);
//   * THE TAVOLA — chamfer_descent's 35 rows (the same shape the
//     fillet's m2 landed: 15 kept / 8 absorbed / 9 born / 3
//     vanished), the closure both sides, the causes verbatim
//     ("replaced by the bevel"), the determinism of the derivation;
//   * THE GATE — the chain gate's five predicates at the single
//     step (the radial, the loops, the chi, the census by name with
//     the held references — the dead vertical2 VANISHED, the
//     surviving vertical3 RESOLVED — and the drift: the re-run's
//     digest bit-identical);
//   * THE REFUSALS — the band's bound NAMED (1.99 admitted, 2.0
//     refused chamfer_setback_exceeds_face — the single edge's own
//     band; the multi-edge strip law d < 1 documented in the
//     payload's words, never met by this walk), the non-positive
//     setback, the horizontal edge, the foreign reference, the
//     bodies outside the class signature (the sphere, the blind
//     body, the FILLETED box — the mixed corner's family refused
//     by signature, D5.3 — and the CHAMFERED box itself: the chain
//     is the next morse's seat, D8.2, the fillet's own m2/m6ter
//     history);
//   * THE FRAME FROM THE PAYLOAD (the m6 correttiva's lesson, made
//     contractual): the TRANSLATED box and the RESIZED box walk the
//     same closed forms — the counts, the volume (16 - d^2 on the
//     {2,1,1} box), the names identical, the world's origin and the
//     unit box's ±1 never consulted.
#include <doctest/doctest.h>

#include "CAD_0/brep/analytic.hpp"
#include "CAD_0/brep/chain_gate.hpp"
#include "CAD_0/brep/chamfer.hpp"
#include "CAD_0/brep/descent.hpp"
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/fillet.hpp"
#include "CAD_0/brep/mass_properties.hpp"
#include "CAD_0/brep/naming.hpp"
#include "CAD_0/brep/pcurve.hpp"
#include "CAD_0/brep/query.hpp"
#include "CAD_0/brep/subtract.hpp"
#include "CAD_0/brep/topology.hpp"
#include "CAD_0/sdf/sketch_profile.hpp"
#include "CAD_0/geometry/shape_id.hpp"

#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace {

namespace b = CAD_0::brep;
namespace n = CAD_0::brep::naming;
using CAD_0::geometry::FeatureId;
using CAD_0::geometry::ShapeId;
using CAD_0::geometry::SubshapeRef;
using Vec2d = CAD_0::math::Vec2d;
using Vec3d = CAD_0::math::Vec3d;

constexpr double kPi = 3.14159265358979323846;
constexpr double kD = 0.5;

bool close(double a, double b, double eps = 1e-9) {
    return std::abs(a - b) <= eps;
}

// The guide case's construction: box#1 (the factory body, the
// feature id 1), the reference box#1:vertical2 (the edge table's
// index 10 — the (+X,+Y) vertical, edge 10, vertices 2 -> 6), the
// chamfer che#2 at d.
struct ChamferGuide {
    std::unique_ptr<b::BRepBody> box;
    std::unique_ptr<b::BRepBody> result;
    ShapeId reference{};

    explicit ChamferGuide(double setback = kD, bool named = true) {
        box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        reference = ShapeId{FeatureId{1},
                            SubshapeRef{}.push(10), {}};
        const b::ChamferNaming ctx{FeatureId{2}, &n::kBoxNaming};
        result = b::chamfer_edge(*box, reference, setback,
                                 named ? ctx : b::ChamferNaming{});
    }
};

// The digests (the m2 fillet test's own shape, replicated): every
// face's and edge's NAME, the co-edge leg, and the full-structure
// digest (counts, positions, pairs, co-edge sequences, the
// p-curves' typed payloads).
std::uint64_t fnv1a64(const std::string& str) {
    std::uint64_t h = 0xcbf29ce484222325ull;
    for (const char c : str) {
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
        out += std::to_string(v.position().x) + ',';
        out += std::to_string(v.position().y) + ',';
        out += std::to_string(v.position().z) + ';';
    }
    for (const auto& e : body.edges) {
        out += std::to_string(e.vertex(0)) + '-';
        out += std::to_string(e.vertex(1)) + ';';
    }
    for (const auto& f : body.solid.shell().faces()) {
        for (const auto& loop : f.loops()) {
            for (const auto& ce : loop.coedges()) {
                out += std::to_string(ce.edge());
                out += ce.reversed() ? 'R' : 'F';
                const auto* pc = ce.p_curve();
                if (pc != nullptr) {
                    out += 'p';
                    out += std::to_string(
                        static_cast<int>(pc->kind));
                    if (pc->kind == b::PCurve::Kind::Segment) {
                        out += std::to_string(
                                   pc->segment.a.x) + ',' +
                               std::to_string(pc->segment.a.y) +
                               ',' +
                               std::to_string(pc->segment.b.x) +
                               ',' +
                               std::to_string(pc->segment.b.y);
                    } else if (pc->kind ==
                               b::PCurve::Kind::Arc) {
                        out += std::to_string(
                                   pc->arc.center.x) + ',' +
                               std::to_string(pc->arc.center.y) +
                               ',' +
                               std::to_string(pc->arc.radius) +
                               ',' +
                               std::to_string(pc->arc.start_angle) +
                               ',' +
                               std::to_string(pc->arc.sweep);
                    }
                }
                out += ';';
            }
        }
    }
    return fnv1a64(out);
}

// A planar face's walk signed area in its own domain (the factory's
// own discipline: the outer loop encloses its region CCW — the
// re-authoring must not turn it).
double signed_area(const b::BRepBody& body, std::size_t f) {
    struct R : b::SurfaceVisitor {
        const b::PlaneSurface* p{};
        void visit(const b::PlaneSurface& pl) override { p = &pl; }
    } r;
    body.solid.shell().faces()[f].surface()->accept(r);
    double a = 0.0;
    const auto& ces =
        body.solid.shell().faces()[f].loops()[0].coedges();
    for (std::size_t k = 0; k < ces.size(); ++k) {
        const auto va = ces[k].start_vertex(body.edges);
        const auto vb =
            ces[(k + 1) % ces.size()].start_vertex(body.edges);
        const auto da =
            body.vertices[static_cast<std::size_t>(va)].position() -
            r.p->origin();
        const auto db =
            body.vertices[static_cast<std::size_t>(vb)].position() -
            r.p->origin();
        a += da.dot(r.p->u_axis()) * db.dot(r.p->v_axis()) -
             db.dot(r.p->u_axis()) * da.dot(r.p->v_axis());
    }
    return 0.5 * a;
}

} // namespace

// ===========================================================================
// THE GOLDEN — the guide case's every number, from the ADR's table
// ===========================================================================
TEST_CASE("G6 m1 the golden: the box edge chamfered at d=0.5 — "
          "10/15/7/H=0/chi=2, the volume 8-d^2 all exact, the "
          "census 30/2, the output gate green") {
    const ChamferGuide g(kD);
    REQUIRE(g.result != nullptr);

    // THE COUNTS — the walk's table reproduced: the edge dies (-1
    // edge, -2 corners), four edges born (2 setback lines, 2
    // chamfer segments — ALL LINES, never arcs), four vertices
    // born, one face born.
    REQUIRE(g.result->vertex_count() == 10);
    REQUIRE(g.result->edge_count() == 15);
    REQUIRE(g.result->face_count() == 7);
    REQUIRE(b::inner_loop_count(*g.result) == 0);
    // THE CHI IDENTITY, visible: 10 - 15 + 7 = 2 = the box's own;
    // Delta-chi = +2 -3 +1 = 0 — the chamfer is chi-flat.
    REQUIRE(b::euler_characteristic(*g.result) == 2);
    CHECK(b::euler_characteristic(*g.result) ==
          b::euler_characteristic(*g.box));
    CHECK(b::is_closed_orientable(*g.result));
    CHECK(b::loops_are_closed(*g.result));

    // THE COHERENCE PIN, side by side (ADR-0045 D6.4): the fillet
    // k=1 at the same parameter answers THE SAME counts — the walk
    // is the same arithmetic, and the four born POSITIONS are the
    // same feet (the setback line and the tangent line both sit at
    // distance d from the edge); the boundary between them is the
    // different geometry.
    {
        auto fbox = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        const b::FilletNaming fctx{FeatureId{2}, &n::kBoxNaming};
        auto f = b::fillet_edge(*fbox, g.reference, 0.5, fctx);
        REQUIRE(f != nullptr);
        REQUIRE(f->vertex_count() ==
                g.result->vertex_count());
        REQUIRE(f->edge_count() == g.result->edge_count());
        REQUIRE(f->face_count() == g.result->face_count());
        for (std::size_t i = 6; i < 10; ++i) {
            CAPTURE(i);
            CHECK(f->vertices[i].position().x ==
                  g.result->vertices[i].position().x);
            CHECK(f->vertices[i].position().y ==
                  g.result->vertices[i].position().y);
            CHECK(f->vertices[i].position().z ==
                  g.result->vertices[i].position().z);
        }
        // ... and the VOLUMES differ (the prism against the wedge):
        // the two closed forms, two geometries, one cube.
        const auto mp_f = b::mass_properties(
            *f, CAD_0::math::Tolerance::linear(1e-9));
        const auto mp_c = b::mass_properties(
            *g.result, CAD_0::math::Tolerance::linear(1e-9));
        CHECK_FALSE(close(mp_f.volume, mp_c.volume, 1e-3));
    }

    // THE CENSUS: every edge exactly two OPPOSITE co-edges; the
    // total 30 incidences over the seven faces (right 4 + back 4 +
    // top 5 + bottom 5 + front 4 + left 4 + bevel 4).
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
        REQUIRE(total == 30);
        // The per-face census (the ADR's own row — the same as the
        // fillet's: the caps gain the segment where the fillet
        // gained its arc).
        const std::size_t per_face[7] = {5, 5, 4, 4, 4, 4, 4};
        for (std::size_t i = 0; i < 7; ++i) {
            CAPTURE(i);
            REQUIRE(g.result->solid.shell().faces()[i]
                        .loops()[0]
                        .coedges()
                        .size() == per_face[i]);
        }
    }

    // The id discipline (the factories' own).
    for (std::size_t i = 0; i < g.result->vertices.size(); ++i)
        CHECK(g.result->vertices[i].id() ==
              static_cast<b::VertexId>(i));
    for (std::size_t i = 0; i < g.result->edges.size(); ++i)
        CHECK(g.result->edges[i].id() ==
              static_cast<b::EdgeId>(i));

    // THE NUMBERING TABLE — the survivors compact, the born append:
    // vertices 6..9 = SA_lo, SA_hi, SB_lo, SB_hi (A = the back
    // face — the edge's FIRST co-edge sits on face 3; B = right).
    // THE CLOSED POSITIONS: the setback feet — the corner (1,1)
    // offset by d along the OTHER face's inward direction: on the
    // back face at x = 1-d, on the right face at y = 1-d.
    {
        const Vec3d pv[4] = {
            {0.5, 1.0, -1.0}, {0.5, 1.0, 1.0},
            {1.0, 0.5, -1.0}, {1.0, 0.5, 1.0}};
        for (std::size_t i = 0; i < 4; ++i) {
            CAPTURE(i);
            const Vec3d p = g.result->vertices[6 + i].position();
            CHECK(close(p.x, pv[i].x, 1e-12));
            CHECK(close(p.y, pv[i].y, 1e-12));
            CHECK(close(p.z, pv[i].z, 1e-12));
        }
        // The edges: 0..9 kept (old 0..9), 10 = the far vertical
        // (old 11, its endpoints remapped: old 3 -> 2, old 7 -> 5),
        // 11 = e_sb_A (6->7), 12 = e_sb_B (8->9), 13 = e_seg_lo
        // (8->6), 14 = e_seg_hi (9->7) — the deterministic B->A
        // convention on the segments, the fillet's own.
        const std::pair<std::size_t, std::size_t> pe[5] = {
            {2, 5}, {6, 7}, {8, 9}, {8, 6}, {9, 7}};
        for (std::size_t i = 0; i < 5; ++i) {
            CAPTURE(i);
            CHECK(g.result->edges[10 + i].vertex(0) ==
                  pe[i].first);
            CHECK(g.result->edges[10 + i].vertex(1) ==
                  pe[i].second);
        }
        // The re-spanned ring edges: old edge 1 (1->2) becomes
        // (1 -> 8) [SB_lo]; old 2 (2->3) becomes (6 -> 2) [SA_lo
        // first, the survivor old 3 -> new 2]; old 5 (5->6) becomes
        // (4 -> 9); old 6 (6->7) becomes (7 -> 5) — the same
        // substitutions the fillet's walk makes (the same feet).
        const std::pair<std::size_t, std::size_t> rs[4] = {
            {1, 8}, {6, 2}, {4, 9}, {7, 5}};
        const std::size_t rs_idx[4] = {1, 2, 5, 6};
        for (std::size_t i = 0; i < 4; ++i) {
            CAPTURE(i);
            CHECK(g.result->edges[rs_idx[i]].vertex(0) ==
                  rs[i].first);
            CHECK(g.result->edges[rs_idx[i]].vertex(1) ==
                  rs[i].second);
        }
    }

    // THE VOLUME — the classical form IS the test: V = 8 - d^2
    // (the removed region the right triangular prism of legs d and
    // height the edge's own span 2: the section d^2/2, the arm 2).
    // All_exact: the bevel's polygonal window and the planar
    // machinery the same arithmetic read twice.
    {
        const auto mp = b::mass_properties(
            *g.result, CAD_0::math::Tolerance::linear(1e-9));
        CHECK(close(mp.volume, 8.0 - kD * kD, 1e-12));
        CHECK(close(8.0 - mp.volume, 2.0 * (kD * kD / 2.0), 1e-12));
        CHECK(mp.all_exact);
        // THE NAIVE READS, red by discrimination: the no-cut naive
        // (8), the square-cut naive (8 - 2d^2 = 7.5 — the corner
        // window removed as a square-section prism, the triangle
        // forgotten), the fillet's own form at the same parameter
        // (7.5 + pi/8 — the rolling ball's wedge: a DIFFERENT
        // geometry, never the flat bevel's answer).
        CHECK_FALSE(close(mp.volume, 8.0, 1e-6));
        CHECK_FALSE(close(mp.volume, 8.0 - 2.0 * kD * kD, 1e-6));
        CHECK_FALSE(close(mp.volume, 7.5 + kPi / 8.0, 1e-3));
        // The centroid: on the x = y diagonal, z = 0 (the prism's
        // own symmetry — the golden pins the symmetry).
        CHECK(close(mp.centroid.x, mp.centroid.y, 1e-12));
        CHECK(std::abs(mp.centroid.z) < 1e-9);
        CHECK(mp.centroid.x < 0.0);  // the corner's material left
    }

    // The provenance: exact throughout (the operand exact; the born
    // constructed).
    for (const auto& v : g.result->vertices)
        CHECK(v.provenance().is_exact());
    for (const auto& e : g.result->edges)
        CHECK(e.provenance().is_exact());
    for (const auto& f : g.result->solid.shell().faces())
        CHECK(f.provenance().is_exact());

    // THE BEVEL'S SURFACE: the PlaneSurface bisector — the normal
    // (n_A + n_B)/sqrt(2) = (1,1,0)/sqrt(2) (the outward bisector),
    // the origin the mid-height point corner - (d/2)(n_A + n_B) =
    // (0.75, 0.75, 0) — the miter's own plane in section (D2.3).
    {
        const auto* surf =
            g.result->solid.shell().faces()[6].surface();
        REQUIRE(surf != nullptr);
        CHECK(surf->family() == b::SurfaceFamily::Plane);
        const auto* pl =
            dynamic_cast<const b::PlaneSurface*>(surf);
        REQUIRE(pl != nullptr);
        CHECK(close(pl->normal_vec().x, 1.0 / std::sqrt(2.0),
                    1e-12));
        CHECK(close(pl->normal_vec().y, 1.0 / std::sqrt(2.0),
                    1e-12));
        CHECK(close(pl->normal_vec().z, 0.0, 1e-12));
        CHECK(close(pl->origin().x, 0.75, 1e-12));
        CHECK(close(pl->origin().y, 0.75, 1e-12));
        CHECK(close(pl->origin().z, 0.0, 1e-12));
        // THE FRAME RULE, pointwise over the window (the analytic
        // surface's own guard): (du x dv) . n_out > 0.
        for (int k = 0; k <= 8; ++k) {
            const double u = -1.0 + 2.0 * k / 8.0;
            const double v = -kD / std::sqrt(2.0) +
                             (2.0 * kD / std::sqrt(2.0)) * k / 8.0;
            const double du = 1e-7;
            const Vec3d puu = pl->evaluate(u + du, v);
            const Vec3d pu = pl->evaluate(u, v);
            const Vec3d pvv = pl->evaluate(u, v + du);
            const Vec3d d_u = (puu - pu) * (1.0 / du);
            const Vec3d d_v = (pvv - pu) * (1.0 / du);
            const Vec3d cr = d_u.cross(d_v);
            const Vec3d nn = pl->normal(u, v);
            CHECK(cr.dot(nn) > 0.0);
        }
        // THE WINDOW IS THE RECTANGLE [−1, 1] x [−d/sqrt(2),
        // d/sqrt(2)]: every p-curve a Line2 on the boundary (the
        // mass machinery's patch class — the naive full-setback
        // width d would read the projection wrong: the feet sit at
        // +-d/sqrt(2) along the in-plane cross axis).
        double u_min = 1e9, u_max = -1e9, v_min = 1e9, v_max = -1e9;
        for (const auto& ce :
             g.result->solid.shell().faces()[6].loops()[0]
                 .coedges()) {
            const auto* pc = ce.p_curve();
            REQUIRE(pc != nullptr);
            REQUIRE(pc->kind == b::PCurve::Kind::Segment);
            u_min = std::min({u_min, pc->segment.a.x,
                              pc->segment.b.x});
            u_max = std::max({u_max, pc->segment.a.x,
                              pc->segment.b.x});
            v_min = std::min({v_min, pc->segment.a.y,
                              pc->segment.b.y});
            v_max = std::max({v_max, pc->segment.a.y,
                              pc->segment.b.y});
        }
        CHECK(close(u_min, -1.0, 1e-12));
        CHECK(close(u_max, 1.0, 1e-12));
        CHECK(close(v_min, -kD / std::sqrt(2.0), 1e-12));
        CHECK(close(v_max, kD / std::sqrt(2.0), 1e-12));
        CHECK_FALSE(close(v_max - v_min, kD));  // the naive width
        // The walk: the CCW rectangle (the low segment, the A
        // setback line, the high segment, the B setback line) —
        // the co-edge table (the same deterministic shape the
        // fillet's flank walks: 13F, 11F, 14R, 12R).
        const auto& ces =
            g.result->solid.shell().faces()[6].loops()[0]
                .coedges();
        REQUIRE(ces.size() == 4);
        CHECK(ces[0].edge() == 13);
        CHECK(ces[0].reversed() == false);
        CHECK(ces[1].edge() == 11);
        CHECK(ces[1].reversed() == false);
        CHECK(ces[2].edge() == 14);
        CHECK(ces[2].reversed() == true);
        CHECK(ces[3].edge() == 12);
        CHECK(ces[3].reversed() == true);
        // The bevel's own walk encloses its window CCW (the
        // interior-left discipline, the re-authoring's own check).
        CHECK(signed_area(*g.result, 6) > 0.0);
    }

    // THE DECOUPLING (ADR-0038 m4): the absorbed faces keep their
    // NATIVE frames — the same mathematical planes, origin and
    // normal, copied bit-identical; the walks re-authored in the
    // result's canonical direction (CCW, interior on the left).
    {
        struct PlaneRef {
            Vec3d origin, normal;
        };
        const auto plane_of = [](const b::BRepBody& body,
                                 std::size_t f) {
            const auto* s =
                body.solid.shell().faces()[f].surface();
            struct R : b::SurfaceVisitor {
                const b::PlaneSurface* p{};
                void visit(const b::PlaneSurface& pl) override {
                    p = &pl;
                }
            } r;
            s->accept(r);
            return PlaneRef{r.p->origin(), r.p->normal_vec()};
        };
        for (std::size_t f = 0; f < 6; ++f) {
            const PlaneRef a = plane_of(*g.box, f);
            const PlaneRef rr = plane_of(*g.result, f);
            CHECK(a.origin.x == rr.origin.x);
            CHECK(a.origin.y == rr.origin.y);
            CHECK(a.origin.z == rr.origin.z);
            CHECK(a.normal.x == rr.normal.x);
            CHECK(a.normal.y == rr.normal.y);
            CHECK(a.normal.z == rr.normal.z);
        }
        // The caps' chamfer segments: typed Line2 in the caps' own
        // domains (NEVER an Arc — the flat bevel's own boundary);
        // the re-spanned and setback co-edges typed Line2
        // everywhere; every p-curve's endpoints the projected
        // vertices (the DIRECTION convention).
        const auto& top =
            g.result->solid.shell().faces()[1].loops()[0]
                .coedges();
        REQUIRE(top.size() == 5);
        std::size_t segs = 0;
        for (const auto& ce : top) {
            const auto* pc = ce.p_curve();
            REQUIRE(pc != nullptr);
            CHECK(pc->kind == b::PCurve::Kind::Segment);
            if (ce.edge() == 13 || ce.edge() == 14) ++segs;
        }
        CHECK(segs == 1);  // the top cap's own segment (the hi end)
        // NO Arc anywhere in the body: the walk is ALL LINES (the
        // flat bevel's own signature, D2.2).
        for (std::size_t f = 0; f < 7; ++f)
            for (const auto& loop :
                 g.result->solid.shell().faces()[f].loops())
                for (const auto& ce : loop.coedges()) {
                    REQUIRE(ce.p_curve() != nullptr);
                    CHECK(ce.p_curve()->kind ==
                          b::PCurve::Kind::Segment);
                }
        // Every co-edge of every face: the endpoint coherence (the
        // image under the surface == the vertices, within the
        // Linear band — the coherence contract).
        for (std::size_t f = 0; f < 7; ++f) {
            const auto& face =
                g.result->solid.shell().faces()[f];
            const auto* s = face.surface();
            for (const auto& loop : face.loops())
                for (const auto& ce : loop.coedges()) {
                    const auto* pc = ce.p_curve();
                    REQUIRE(pc != nullptr);
                    const auto s0 = static_cast<std::size_t>(
                        ce.start_vertex(g.result->edges));
                    const auto s1 = static_cast<std::size_t>(
                        ce.end_vertex(g.result->edges));
                    const auto q0 = s->evaluate(
                        pc->segment.a.x, pc->segment.a.y);
                    const auto q1 = s->evaluate(
                        pc->segment.b.x, pc->segment.b.y);
                    CHECK((q0 -
                           g.result->vertices[s0].position())
                              .length_sq() < 1e-18);
                    CHECK((q1 -
                           g.result->vertices[s1].position())
                              .length_sq() < 1e-18);
                }
        }
        // The re-authored walks' canonical direction: the caps'
        // and laterals' outer loops still enclose their regions
        // CCW in their domains (the signed area positive — the
        // factory's own discipline, unchanged by the re-authoring).
        for (const std::size_t f : {0u, 1u, 2u, 3u, 4u, 5u}) {
            CAPTURE(f);
            CHECK(signed_area(*g.result, f) > 0.0);
            CHECK(signed_area(*g.box, f) > 0.0);
        }
    }

    // THE LIMITS (the form's own coherence): d -> 0 is the box.
    {
        const ChamferGuide tiny(1e-9);
        REQUIRE(tiny.result != nullptr);
        const auto mp = b::mass_properties(
            *tiny.result, CAD_0::math::Tolerance::linear(1e-9));
        CHECK(close(mp.volume, 8.0, 1e-15));
    }
}

// ===========================================================================
// THE ORACLE — the m3 law: the volume re-derived with THIS FILE's
// own equations (zero production construction), the counterproofs
// that doctor REFUSED
// ===========================================================================
TEST_CASE("G6 m1 the oracle: the volume by the file's own midpoint "
          "rule at 2e6 samples, the closed form, the kernel's mass "
          "— and the doctored reads red") {
    // THE FILE'S OWN QUADRATURE (the landed road-3 idiom: the piece
    // in its own parametrization, the polynomial terms exact, never
    // an indicator on a straddled grid): the removed prism's
    // cross-section is the right triangle int_0^d (d-u) du over the
    // arm 2 — a 1-D midpoint rule, 2e6 samples, the affine
    // integrand.
    const int N = 2'000'000;
    const double h = kD / N;
    double section = 0.0;
    for (int i = 0; i < N; ++i)
        section += (kD - (i + 0.5) * h) * h;
    const double removed = 2.0 * section;
    const double v_quad = 8.0 - removed;

    // Against the closed form at the ADR's declared bar...
    CHECK(close(v_quad, 8.0 - kD * kD, 2e-6));
    // ...and against the kernel's own mass at the exactness pin.
    const ChamferGuide g(kD);
    REQUIRE(g.result != nullptr);
    const auto mp = b::mass_properties(
        *g.result, CAD_0::math::Tolerance::linear(1e-9));
    CHECK(close(mp.volume, 8.0 - kD * kD, 1e-12));
    CHECK(close(v_quad, mp.volume, 2e-6));

    // THE DOCTORED READS, each refused by the very comparison the
    // golden passes (the contract discriminates, it does not
    // rubber-stamp):
    // (a) THE OFFSET ALTERED — the naive bevel at half the setback
    //     (the plane read at d/2): its volume is NOT the body's.
    const double v_half_offset = 8.0 - (0.5 * kD) * (0.5 * kD);
    CHECK_FALSE(close(v_half_offset, mp.volume, 1e-6));
    // (b) THE SETBACK ALTERED (+0.01 in the oracle's own equation):
    //     the doctored rule must disagree at the golden's pin.
    const double v_altered = 8.0 - (kD + 0.01) * (kD + 0.01);
    CHECK_FALSE(close(v_altered, mp.volume, 1e-6));
    // (c) THE NAIVE CUTS — the no-cut (8) and the square-cut
    //     (8 - 2d^2: the corner window as a square-section prism).
    CHECK_FALSE(close(8.0, mp.volume, 1e-6));
    CHECK_FALSE(close(8.0 - 2.0 * kD * kD, mp.volume, 1e-6));
    // (d) THE ORIENTATION FLIPPED — the bevel's walk authored CW
    //     (the naive interior-right): the reversed walk's signed
    //     area is NEGATIVE — the interior-left rule refuses what
    //     the landed walk passes (the pair read together).
    {
        const auto& ces =
            g.result->solid.shell().faces()[6].loops()[0]
                .coedges();
        REQUIRE(ces.size() == 4);
        const auto* pl = dynamic_cast<const b::PlaneSurface*>(
            g.result->solid.shell().faces()[6].surface());
        REQUIRE(pl != nullptr);
        // The REVERSED walk: the same four vertices, the other
        // direction — the naive authoring's own polygon.
        std::size_t rev[4] = {3, 2, 1, 0};
        double a = 0.0;
        for (std::size_t k = 0; k < 4; ++k) {
            const auto va =
                ces[rev[k]].start_vertex(g.result->edges);
            const auto vb = ces[rev[(k + 1) % 4]]
                                .start_vertex(g.result->edges);
            const auto da =
                g.result->vertices[static_cast<std::size_t>(va)]
                    .position() -
                pl->origin();
            const auto db =
                g.result->vertices[static_cast<std::size_t>(vb)]
                    .position() -
                pl->origin();
            a += da.dot(pl->u_axis()) * db.dot(pl->v_axis()) -
                 db.dot(pl->u_axis()) * da.dot(pl->v_axis());
        }
        CHECK(0.5 * a < 0.0);  // the naive CW: red
        CHECK(signed_area(*g.result, 6) > 0.0);  // the landed: green
    }
    // (e) THE INJECTED DEFECT, live (the m2 counterproof's own
    //     shape): the dead edge re-added to the result — it carries
    //     ZERO co-edges (its faces were re-authored without it) and
    //     the manifold gate refuses the body the golden passes.
    {
        b::BRepBody naive;
        naive.vertices = g.result->vertices;
        naive.edges = g.result->edges;
        naive.edges.push_back(b::Edge{2, 6});
        naive.edges.back().set_id(
            static_cast<b::EdgeId>(naive.edges.size() - 1));
        naive.solid = g.result->solid;  // the faces, verbatim
        b::link_radial_edges(naive);
        REQUIRE(naive.edges.back().coedges().size() == 0);
        CHECK_FALSE(b::is_closed_orientable(naive));
        CHECK(b::euler_characteristic(naive) == 1);  // chi-odd
        CHECK(b::genus_from_chi(
                  b::euler_characteristic(naive)) ==
              std::nullopt);
    }
}

// ===========================================================================
// THE NAMES — D3's table, character for character
// ===========================================================================
TEST_CASE("G6 m1 the names: the bevel and the setback wires, the "
          "same-feature genealogy, the shrink faces' names kept") {
    const ChamferGuide g(kD);
    REQUIRE(g.result != nullptr);

    // The bevel: che#2:bevel[box#1:back × box#1:right] — the
    // operands the edge's adjacent faces in the co-edge order (A =
    // back, the first co-edge's face; B = right).
    REQUIRE(g.result->solid.shell().faces()[6].shape_id()
                .subshape.depth > 0);
    CHECK(g.result->solid.shell().faces()[6].name() ==
          "che#2:bevel[box#1:back × box#1:right]");
    CHECK(g.result->solid.shell().faces()[6].shape_id().feature ==
          FeatureId{2});
    // The setback wires: che#2:isect[<the landing face> × <the
    // bevel>] — the level ABSENT, the setback IS the level; the
    // landing face of each wire is the adjacent it retires.
    const std::string bevel =
        "che#2:bevel[box#1:back × box#1:right]";
    CHECK(g.result->edges[11].name() ==
          "che#2:isect[box#1:back × " + bevel + "]");
    CHECK(g.result->edges[12].name() ==
          "che#2:isect[box#1:right × " + bevel + "]");
    // The chamfer segments (the caps' own boundary, not contacts):
    // honestly UNNAMED (the fillet's arcs' own precedent — the
    // isect grammar names the CONTACT curves).
    CHECK(g.result->edges[13].name().empty());
    CHECK(g.result->edges[14].name().empty());
    // The four born vertices: unnamed by construction (the v1
    // discipline, ADR-0036 D4 — vertex names have no consumer
    // today; the tavola's born vertex rows read empty below).
    // The shrink faces KEEP theirs (the inheritance is a stamp
    // copy, no table involved).
    CHECK(g.result->solid.shell().faces()[3].name() ==
          "box#1:back");
    CHECK(g.result->solid.shell().faces()[5].name() ==
          "box#1:right");
    CHECK(g.result->solid.shell().faces()[1].name() == "box#1:top");

    // THE UNNAMED MODE, honest beside the named one: without the
    // context the born entities read empty, the shrink faces' names
    // still theirs, the walk identical (the digest's structure
    // agrees — only the names differ).
    const ChamferGuide unnamed(kD, false);
    REQUIRE(unnamed.result != nullptr);
    CHECK(unnamed.result->solid.shell().faces()[6].name().empty());
    CHECK(unnamed.result->edges[11].name().empty());
    CHECK(unnamed.result->edges[12].name().empty());
    CHECK(unnamed.result->solid.shell().faces()[3].name() ==
          "box#1:back");
    REQUIRE(unnamed.result->vertex_count() == 10);
    REQUIRE(unnamed.result->edge_count() == 15);
    CHECK(structure_digest(*unnamed.result) ==
          structure_digest(*g.result));

    // THE DIGEST'S PURITY: the names a pure function of the recipe
    // — another instance of the SAME construction, the same names
    // byte for byte (never of the body's address).
    {
        const ChamferGuide g2(kD);
        REQUIRE(g2.result != nullptr);
        CHECK(name_digest(*g2.result) == name_digest(*g.result));
    }
    // The feature part flows: another chamfer id reads che#3.
    {
        auto box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        const ShapeId ref{FeatureId{1}, SubshapeRef{}.push(10),
                          {}};
        const b::ChamferNaming ctx{FeatureId{3},
                                   &n::kBoxNaming};
        auto r = b::chamfer_edge(*box, ref, 0.5, ctx);
        REQUIRE(r != nullptr);
        CHECK(r->solid.shell().faces()[6].name() ==
              "che#3:bevel[box#1:back × box#1:right]");
    }
    // Another edge, another operand order: the (+X,-Y) vertical
    // (vertical1, edge 9) — the faces right (+X) and front (-Y),
    // the shell's order putting FRONT first.
    {
        auto box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        const ShapeId ref{FeatureId{1}, SubshapeRef{}.push(9), {}};
        const b::ChamferNaming ctx{FeatureId{2},
                                   &n::kBoxNaming};
        auto r = b::chamfer_edge(*box, ref, 0.5, ctx);
        REQUIRE(r != nullptr);
        REQUIRE(r->vertex_count() == 10);
        REQUIRE(r->edge_count() == 15);
        REQUIRE(r->face_count() == 7);
        CHECK(b::euler_characteristic(*r) == 2);
        CHECK(b::is_closed_orientable(*r));
        CHECK(r->solid.shell().faces()[6].name() ==
              "che#2:bevel[box#1:front × box#1:right]");
        const auto mp = b::mass_properties(
            *r, CAD_0::math::Tolerance::linear(1e-9));
        CHECK(close(mp.volume, 8.0 - 0.25, 1e-12));
        CHECK(mp.all_exact);
    }

    // THE BIT-IDENTICAL REBUILD (the m4 N-run discipline, the
    // digest's determinism): ten rebuilds, one digest.
    {
        std::uint64_t first = 0;
        for (int i = 0; i < 10; ++i) {
            const ChamferGuide gi(kD);
            REQUIRE(gi.result != nullptr);
            const std::uint64_t d =
                structure_digest(*gi.result);
            if (i == 0)
                first = d;
            else
                CHECK(d == first);
        }
    }
}

// ===========================================================================
// THE TAVOLA — chamfer_descent's 35 rows, the closure both sides
// ===========================================================================
TEST_CASE("G6 m1 the tavola: the 35 rows, kept/absorbed/born/"
          "vanished counted, the closure green both sides") {
    const ChamferGuide g(kD);
    REQUIRE(g.result != nullptr);
    const b::DescentTable t =
        b::chamfer_descent(*g.result, *g.box);

    // THE COUNT (the walk's own arithmetic): 35 rows — the result's
    // 10 + 15 + 7 = 32, plus the vanished tail 1 edge + 2 vertices.
    REQUIRE(t.rows.size() == 35);
    CHECK(t.count(b::Descent::Kept) == 15);      // 6 V + 7 E + 2 F
    CHECK(t.count(b::Descent::Absorbed) == 8);   // 4 E + 4 F
    CHECK(t.count(b::Descent::Born) == 9);       // 4 V + 4 E + 1 F
    CHECK(t.count(b::Descent::Vanished) == 3);   // 2 V + 1 E
    CHECK(t.count(b::Descent::Split) == 0);
    CHECK(t.count(b::Descent::Merged) == 0);

    // The faces: out 7 = 2 kept [front, left] + 4 absorbed [the
    // laterals, the caps] + 1 born [the bevel]; vanished 0. The
    // edges: out 15 = 7 kept + 4 absorbed [the re-spanned] + 4 born
    // [the setback lines, the segments]; vanished 1 [the dead
    // vertical]. The vertices: out 10 = 6 kept + 4 born; vanished 2
    // [the dead corners].
    {
        std::size_t kept_v = 0, kept_e = 0, kept_f = 0;
        std::size_t abs_v = 0, abs_e = 0, abs_f = 0;
        std::size_t born_v = 0, born_e = 0, born_f = 0;
        std::size_t van_v = 0, van_e = 0, van_f = 0;
        for (const auto& row : t.rows) {
            const bool k = row.relation == b::Descent::Kept;
            const bool a = row.relation == b::Descent::Absorbed;
            const bool bo = row.relation == b::Descent::Born;
            const bool va = row.relation == b::Descent::Vanished;
            if (row.entity == b::EntityKind::Vertex) {
                kept_v += k; abs_v += a; born_v += bo; van_v += va;
            } else if (row.entity == b::EntityKind::Edge) {
                kept_e += k; abs_e += a; born_e += bo; van_e += va;
            } else {
                kept_f += k; abs_f += a; born_f += bo; van_f += va;
            }
        }
        CHECK(kept_v == 6);  CHECK(abs_v == 0);
        CHECK(born_v == 4);  CHECK(van_v == 2);
        CHECK(kept_e == 7);  CHECK(abs_e == 4);
        CHECK(born_e == 4);  CHECK(van_e == 1);
        CHECK(kept_f == 2);  CHECK(abs_f == 4);
        CHECK(born_f == 1);  CHECK(van_f == 0);
    }

    // The vanished rows' cause, verbatim: "replaced by the bevel"
    // — the edge's death IS the feature's act (the fillet's own
    // cause one family over).
    {
        std::size_t causes = 0;
        for (const auto& row : t.rows) {
            if (row.relation != b::Descent::Vanished) continue;
            CHECK(row.cause == "replaced by the bevel");
            // The dead edge's row: the vertical2's own identity.
            if (row.entity == b::EntityKind::Edge) {
                CHECK(row.address == 10);
                CHECK(row.shape_id ==
                      g.box->edges[10].shape_id());
            }
            ++causes;
        }
        CHECK(causes == 3);
    }

    // The born rows' names: the bevel and the two wires named, the
    // segments honestly empty.
    for (const auto& row : t.rows) {
        if (row.relation != b::Descent::Born) continue;
        if (row.entity == b::EntityKind::Face)
            CHECK(row.name ==
                  "che#2:bevel[box#1:back × box#1:right]");
        if (row.entity == b::EntityKind::Edge) {
            const bool named = !row.name.empty();
            if (named)
                CHECK(row.name.rfind("che#2:isect[", 0) == 0);
        }
    }

    // THE CLOSURE, both sides green (the hard invariant).
    std::string why;
    CHECK(b::descent_closed(*g.result, t, &why));
    {
        const b::BRepBody no_second{};
        CHECK(b::old_generation_closed(*g.box, no_second, t,
                                       &why));
    }

    // Determinism of the derivation itself: the same bodies, the
    // same table, byte for byte (the recipe is the source of truth,
    // the tavola is cache).
    const b::DescentTable t2 =
        b::chamfer_descent(*g.result, *g.box);
    REQUIRE(t2.rows.size() == t.rows.size());
    for (std::size_t i = 0; i < t.rows.size(); ++i) {
        CHECK(t.rows[i].relation == t2.rows[i].relation);
        CHECK(t.rows[i].entity == t2.rows[i].entity);
        CHECK(t.rows[i].address == t2.rows[i].address);
        CHECK(t.rows[i].name == t2.rows[i].name);
        CHECK(t.rows[i].cause == t2.rows[i].cause);
        CHECK(t.rows[i].operands == t2.rows[i].operands);
        CHECK(t.rows[i].shape_id == t2.rows[i].shape_id);
    }
}

// ===========================================================================
// THE GATE — the chain gate's five predicates at the single step
// ===========================================================================
TEST_CASE("G6 m1 the gate: the five predicates at the single step "
          "— the census by name with the held references, the drift "
          "zero at the re-run's digest") {
    const ChamferGuide g(kD);
    REQUIRE(g.result != nullptr);
    const b::DescentTable t =
        b::chamfer_descent(*g.result, *g.box);

    // The re-run (the drift's own subject): the same recipe, a
    // fresh body — the digest bit-identical.
    const ChamferGuide rerun(kD);
    REQUIRE(rerun.result != nullptr);

    b::ChainStep step;
    step.body = g.result.get();
    step.tavola = &t;
    step.previous = g.box.get();
    // The references the chain holds across the step: the dead
    // vertical2 (VANISHED — the tavola's own cause) and the
    // surviving vertical3 (RESOLVED — class 1).
    step.held.push_back(
        {ShapeId{FeatureId{1}, SubshapeRef{}.push(10), {}},
         CAD_0::brep::Resolution::Kind::Vanished});
    step.held.push_back(
        {ShapeId{FeatureId{1}, SubshapeRef{}.push(11), {}},
         CAD_0::brep::Resolution::Kind::Resolved});
    step.rerun_digest = b::chain_digest(*rerun.result);

    const b::ChainGateVerdict v = b::chain_gate(step);
    CHECK(v.pass);
    CHECK(v.failures.empty());
    CHECK(v.radial_two_opposite);
    CHECK(v.loops_closed);
    CHECK(v.chi_even_orientable);
    CHECK(v.census_by_name);
    CHECK(v.drift_zero);
    CHECK(v.not_applicable.empty());
    // The step's own numbers (the row's arithmetic).
    CHECK(v.v == 10);
    CHECK(v.e == 15);
    CHECK(v.f == 7);
    CHECK(v.h == 0);
    CHECK(v.chi == 2);
    CHECK(v.born == 9);
    CHECK(v.kept == 15);
    CHECK(v.absorbed == 8);
    CHECK(v.vanished == 3);
    // The digest IS the re-run's (the drift's own left side).
    CHECK(v.digest == step.rerun_digest);
    CHECK(v.digest == b::chain_digest(*g.result));
}

// ===========================================================================
// THE REFUSALS — the band's bound named, the families by name, the
// frame from the payload (translated and resized boxes)
// ===========================================================================
TEST_CASE("G6 m1 the refusals: the setback out of band carries the "
          "bound, the class families refuse by name, the chain is "
          "the next morse's seat") {
    auto box = [] {
        return b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
    };
    const ShapeId ref{FeatureId{1}, SubshapeRef{}.push(10), {}};
    const b::ChamferNaming ctx{FeatureId{2}, &n::kBoxNaming};

    const auto try_chamfer = [&](double setback,
                                 std::string& msg) {
        try {
            auto r = b::chamfer_edge(*box(), ref, setback, ctx);
            (void)r;
            return false;
        } catch (const std::invalid_argument& e) {
            msg = e.what();
            return true;
        }
    };

    // THE SINGLE EDGE'S OWN BAND (D5.1's first half — the member
    // this walk can meet): the setback line strictly inside the
    // faces (the reach 2, the epsilon the declared tol). 1.99
    // ADMITTED (the counts the golden's own); 2.0 REFUSED — the
    // line would sit ON the boundary; 2.5 refused with every
    // number in the message.
    {
        auto r = b::chamfer_edge(*box(), ref, 1.99, ctx);
        REQUIRE(r != nullptr);
        REQUIRE(r->vertex_count() == 10);
        REQUIRE(r->edge_count() == 15);
        REQUIRE(r->face_count() == 7);
        CHECK(b::euler_characteristic(*r) == 2);
        const auto mp = b::mass_properties(
            *r, CAD_0::math::Tolerance::linear(1e-9));
        CHECK(close(mp.volume, 8.0 - 1.99 * 1.99, 1e-9));
    }
    {
        std::string msg;
        REQUIRE(try_chamfer(2.0, msg));
        CHECK(msg.find("chamfer_setback_exceeds_face") !=
              std::string::npos);
    }
    {
        std::string msg;
        REQUIRE(try_chamfer(2.5, msg));
        CHECK(msg.find("chamfer_setback_exceeds_face") !=
              std::string::npos);
        CHECK(msg.find("2.500000") != std::string::npos);
        CHECK(msg.find("2.000000") != std::string::npos);
        CHECK(msg.find("0.000001") != std::string::npos);
    }
    // THE STATE BAND (D5.1's second half) is NOT this walk's wall:
    // the strip law d < 1 binds the MULTI-EDGE mode (the ring act's
    // own morsi) — a single chamfer at 1.5 is IN its own band and
    // the walk is honest about it (admitted, the golden's counts).
    {
        auto r = b::chamfer_edge(*box(), ref, 1.5, ctx);
        REQUIRE(r != nullptr);
        REQUIRE(r->vertex_count() == 10);
        const auto mp = b::mass_properties(
            *r, CAD_0::math::Tolerance::linear(1e-9));
        CHECK(close(mp.volume, 8.0 - 2.25, 1e-9));
    }

    // The non-positive setback: refused with the number.
    {
        std::string msg;
        REQUIRE(try_chamfer(0.0, msg));
        CHECK(msg.find("setback") != std::string::npos);
        REQUIRE(try_chamfer(-0.5, msg));
        CHECK(msg.find("-0.500000") != std::string::npos);
    }

    // The horizontal edge (a ring edge, not a vertical): the class
    // takes the VERTICAL spigolo — the refusal names the ring's own
    // seat.
    {
        try {
            auto r = b::chamfer_edge(
                *box(),
                ShapeId{FeatureId{1}, SubshapeRef{}.push(0), {}},
                0.5, ctx);
            (void)r;
            FAIL("the horizontal edge must refuse");
        } catch (const std::invalid_argument& e) {
            CHECK(std::string(e.what()).find("not vertical") !=
                  std::string::npos);
        }
    }

    // The reference against the wrong body: the structural find
    // refuses (the closure culture).
    {
        try {
            auto other = b::make_box({1.0, 1.0, 1.0},
                                     FeatureId{2});
            auto r = b::chamfer_edge(
                *other,
                ShapeId{FeatureId{1}, SubshapeRef{}.push(10), {}},
                0.5, ctx);
            (void)r;
            FAIL("the foreign reference must refuse");
        } catch (const std::invalid_argument& e) {
            CHECK(std::string(e.what()).find("names no edge") !=
                  std::string::npos);
        }
    }

    // The bodies outside the class signature: the sphere (1 face, 0
    // edges), the blind body (12/17/8), and — the two walls this
    // stage's own words register — the FILLETED box (10/15/7 with a
    // CYLINDER: the mixed corner's family, D5.3, refused by
    // signature) and the CHAMFERED box itself (10/15/7 all planes:
    // the CHAIN is the next morse's seat, D8.2 — the fillet's own
    // m2/m6ter history, the counts' k and the bevel-stamp wall the
    // widening's own derivation).
    {
        const auto sphere = b::make_sphere(1.0, {}, FeatureId{1});
        try {
            auto r = b::chamfer_edge(
                *sphere, ShapeId{FeatureId{1},
                                 SubshapeRef{}.push(0), {}},
                0.5, ctx);
            (void)r;
            FAIL("the sphere must refuse");
        } catch (const std::invalid_argument& e) {
            CHECK(std::string(e.what()).find(
                      "outside the class signature") !=
                  std::string::npos);
        }
        std::vector<CAD_0::sdf::SketchChain> chains;
        chains.push_back(CAD_0::sdf::SketchChain{
            {CAD_0::sdf::sketch_circle({0.0f, 0.0f}, 0.25f,
                                       false)}});
        auto tool = b::make_prism(chains, 3.0f, +1, FeatureId{3});
        const b::SubtractNaming sub_ctx{
            FeatureId{4}, &n::kBoxNaming, &n::kExtNaming};
        auto blind = b::exact_subtract(*box(), *tool,
                                       {0.0, 0.0, -1.5},
                                       b::kSubtractTolerance,
                                       sub_ctx);
        REQUIRE(blind != nullptr);
        try {
            auto r = b::chamfer_edge(
                *blind,
                ShapeId{FeatureId{1}, SubshapeRef{}.push(10), {}},
                0.1, ctx);
            (void)r;
            FAIL("the blind body must refuse");
        } catch (const std::invalid_argument& e) {
            CHECK(std::string(e.what()).find(
                      "outside the class signature") !=
                  std::string::npos);
        }
        // The filleted box: the mixed world refused by signature.
        {
            const b::FilletNaming fctx{FeatureId{2},
                                       &n::kBoxNaming};
            auto filleted =
                b::fillet_edge(*box(), ref, 0.5, fctx);
            REQUIRE(filleted != nullptr);
            try {
                auto r = b::chamfer_edge(
                    *filleted,
                    ShapeId{FeatureId{1},
                            SubshapeRef{}.push(9), {}},
                    0.5, ctx);
                (void)r;
                FAIL("the filleted body must refuse");
            } catch (const std::invalid_argument& e) {
                CHECK(std::string(e.what()).find(
                          "outside the class signature") !=
                      std::string::npos);
            }
        }
        // The chamfered box: the chain's widening is the next
        // morse's own seat — refused here, BY NAME, the remedy
        // carrying the seat.
        {
            auto chamfered = b::chamfer_edge(*box(), ref, 0.5,
                                             ctx);
            REQUIRE(chamfered != nullptr);
            try {
                auto r = b::chamfer_edge(
                    *chamfered,
                    ShapeId{FeatureId{1},
                            SubshapeRef{}.push(9), {}},
                    0.5, ctx);
                (void)r;
                FAIL("the chained body must refuse (this morse)");
            } catch (const std::invalid_argument& e) {
                CHECK(std::string(e.what()).find(
                          "outside the class signature") !=
                      std::string::npos);
                CHECK(std::string(e.what()).find(
                          "next "
                          "morse's own seat") !=
                      std::string::npos);
            }
        }
    }
}

// ===========================================================================
// THE FRAME FROM THE PAYLOAD — the m6 correttiva's lesson, made
// contractual (the translated and resized boxes walk the same
// closed forms)
// ===========================================================================
TEST_CASE("G6 m1 the frame from the payload: the translated and "
          "resized boxes — the same walk, the same names, the "
          "world's origin never consulted") {
    // THE TRANSLATED BOX: the closed forms read the corner, the
    // normals and the spans from the PAYLOAD — a translated cube is
    // the SAME construction (the m6 correttiva's catch (2) was a
    // translated cube CRASHING the ring act's world-sign reads; the
    // chamfer's walk carries the cure from day one).
    {
        auto box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        b::translate_body(*box, {3.0, -2.0, 5.0});
        const ShapeId ref{FeatureId{1}, SubshapeRef{}.push(10),
                          {}};
        const b::ChamferNaming ctx{FeatureId{2},
                                   &n::kBoxNaming};
        auto r = b::chamfer_edge(*box, ref, 0.5, ctx);
        REQUIRE(r != nullptr);
        REQUIRE(r->vertex_count() == 10);
        REQUIRE(r->edge_count() == 15);
        REQUIRE(r->face_count() == 7);
        CHECK(b::euler_characteristic(*r) == 2);
        CHECK(b::is_closed_orientable(*r));
        CHECK(b::loops_are_closed(*r));
        // The born feet: the translated corner's own (3.5, -1) and
        // (4, -1.5) — the payload's, never the world's (1,1)-corner
        // reads.
        CHECK(close(r->vertices[6].position().x, 3.5, 1e-12));
        CHECK(close(r->vertices[6].position().y, -1.0, 1e-12));
        CHECK(close(r->vertices[6].position().z, 4.0, 1e-12));
        CHECK(close(r->vertices[8].position().x, 4.0, 1e-12));
        CHECK(close(r->vertices[8].position().y, -1.5, 1e-12));
        // The names identical (the recipe's pure function), the
        // volume the golden's own.
        CHECK(r->solid.shell().faces()[6].name() ==
              "che#2:bevel[box#1:back × box#1:right]");
        const auto mp = b::mass_properties(
            *r, CAD_0::math::Tolerance::linear(1e-9));
        CHECK(close(mp.volume, 8.0 - 0.25, 1e-12));
        CHECK(mp.all_exact);
        // The tavola closes on the translated pair too.
        const b::DescentTable t =
            b::chamfer_descent(*r, *box);
        std::string why;
        CHECK(b::descent_closed(*r, t, &why));
        CHECK(t.count(b::Descent::Vanished) == 3);
    }
    // THE RESIZED BOX: the {2,1,1} box (x half-extent 2) — the
    // vertical2 at (2,1), the reach 4, the volume 16 - d^2 (the
    // removed prism the same cross-section over the same arm: the
    // edge's own span, read from the payload).
    {
        auto box = b::make_box({2.0, 1.0, 1.0}, FeatureId{1});
        const ShapeId ref{FeatureId{1}, SubshapeRef{}.push(10),
                          {}};
        const b::ChamferNaming ctx{FeatureId{2},
                                   &n::kBoxNaming};
        auto r = b::chamfer_edge(*box, ref, 0.5, ctx);
        REQUIRE(r != nullptr);
        REQUIRE(r->vertex_count() == 10);
        REQUIRE(r->edge_count() == 15);
        REQUIRE(r->face_count() == 7);
        CHECK(b::euler_characteristic(*r) == 2);
        CHECK(close(r->vertices[6].position().x, 1.5, 1e-12));
        CHECK(close(r->vertices[6].position().y, 1.0, 1e-12));
        CHECK(close(r->vertices[8].position().x, 2.0, 1e-12));
        CHECK(close(r->vertices[8].position().y, 0.5, 1e-12));
        const auto mp = b::mass_properties(
            *r, CAD_0::math::Tolerance::linear(1e-9));
        CHECK(close(mp.volume, 16.0 - 0.25, 1e-12));
        CHECK(mp.all_exact);
        // The resized band is the faces' OWN reaches — and the two
        // adjacent faces READ DIFFERENTLY on the asymmetric box (the
        // back face's reach 4 along x, the right face's 2 along y):
        // the binding bound is the MINIMUM (the right face's), the
        // payload's own arithmetic. A setback the unit box would
        // refuse (1.5 < 2 here) is ADMITTED; 2.5 exits the RIGHT
        // face — refused with the binding bound in the message.
        auto wide = b::chamfer_edge(*box, ref, 1.5, ctx);
        REQUIRE(wide != nullptr);
        const auto mpw = b::mass_properties(
            *wide, CAD_0::math::Tolerance::linear(1e-9));
        CHECK(close(mpw.volume, 16.0 - 2.25, 1e-9));
        try {
            auto bad = b::chamfer_edge(*box, ref, 2.5, ctx);
            (void)bad;
            FAIL("the wide setback must refuse on the right face's "
                 "reach");
        } catch (const std::invalid_argument& e) {
            const std::string msg = e.what();
            CHECK(msg.find("chamfer_setback_exceeds_face") !=
                  std::string::npos);
            CHECK(msg.find("2.000000") != std::string::npos);
        }
    }
}
