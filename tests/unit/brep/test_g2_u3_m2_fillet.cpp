// tests/unit/brep/test_g2_u3_m2_fillet.cpp
//
// G2 UNIT 3, m2 — IL FILLET, IL MORSO 1: LO SPIGOLO DEL BOX, IL
// FIANCO CILINDRICO (ADR-0039 D1.1, the cylindrical flank). The
// goldens were pinned by the ADR BEFORE this code (D5's protocol,
// the red-first discipline: on the pre-m2 tree the call refuses —
// the fillet did not exist, the red was the absence itself); this
// file lands them green on the implementation, every number quoted
// from the derivation, never re-derived differently.
//
// THE GUIDE CASE: the box [-1,1]^3, the (+X,+Y) corner's VERTICAL
// edge — `box#1:vertical2` (edge 10, vertices 2=(1,1,-1) ->
// 6=(1,1,1)), the faces box#1:right (+X) and box#1:back (+Y) —
// filleted at r = 0.5.
//
// THE DERIVED COUNTS (D1.1's table, the walk visible):
//   V = 8 - 2 + 4 = 10;  E = 12 - 1 + 4 = 15;  F = 6 + 1 = 7;
//   H = 0;  chi = 10 - 15 + 7 = 2 (Delta-chi = +2 -3 +1 = 0).
//   The census: right 4 + back 4 + top 5 + bottom 5 + front 4 +
//   left 4 + flank 4 = 30 / 2 = 15 edges, every edge exactly two
//   OPPOSITE co-edges.
//
// THE VOLUME: abc - l*r^2*(1 - pi/4) = 8 - 2*0.25*(1-pi/4) =
// 7.5 + pi/8 = 7.892699081698725 — all_exact, the wedge formula
// A(pi/2, r) = r^2*(1-pi/4) verified BY the mass (the flank patch's
// window integrals and the analytic wedge the same arithmetic read
// twice).
//
// THE COUNTERPROOFS (the naive red, per component):
//   * the flank's window: the naive full-cylinder read (the ring
//     without the quarter window) answers the whole cylinder's
//     displacement pi*r^2*l — the golden's removed volume
//     l*r^2*(1-pi/4) is NOT it, and the square-cut naive
//     (8 - 2r^2) is not it either: the pinned window [u0, u0+pi/2]
//     x [0, l] IS the discrimination;
//   * the dead edge: the naive that keeps it counts 12/16/7 and
//     reads chi = 3 (odd — the gate refuses); the injected defect
//     (the dead edge re-added, co-edge-less) is refused by the very
//     gate the golden passes;
//   * the re-spanning: the naive that skips the arc insertion breaks
//     the census (the arc left with ONE co-edge) and the loop's
//     chaining — the injected defect is refused the same way.
//
// THE EDIT CONTRACT (0036 D6): the radius edit is class 1 SILENT —
// the reference resolves reference_resolved, the feature re-runs,
// the counts stay 10/15/7 for EVERY in-band r, and the same r
// rebuilds bit-identically (N = 10, the digest).
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

// The guide case's construction: box#1 (the factory body, the
// feature id 1), the reference box#1:vertical2 (the edge table's
// index 10 — the (+X,+Y) vertical, edge 10, vertices 2 -> 6), the
// fillet fil#2 at r.
struct FilletGuide {
    std::unique_ptr<b::BRepBody> box;
    std::unique_ptr<b::BRepBody> result;
    ShapeId reference{};

    explicit FilletGuide(double radius = 0.5,
                         bool named = true) {
        box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        reference = ShapeId{FeatureId{1},
                            SubshapeRef{}.push(10), {}};
        const b::FilletNaming ctx{FeatureId{2}, &n::kBoxNaming};
        result = b::fillet_edge(*box, reference, radius,
                                named ? ctx : b::FilletNaming{});
    }
};

// The digests (the m4 oracle's own shape, replicated): every face's
// and edge's NAME, plus the co-edge leg (every co-edge's edge name
// — D5.1's extension), and the full-structure digest (the door's
// own: counts, positions, pairs, co-edge sequences, the p-curves'
// typed payloads).
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

std::string coedge_name_digest(const b::BRepBody& body) {
    std::string out;
    for (b::FaceId i = 0; i < body.face_count(); ++i) {
        const auto info = b::query_face(body, i);
        for (const auto& loop : info.loops)
            for (const auto& pc : loop.coedges) {
                out += std::to_string(i);
                out += '|';
                out += std::to_string(pc.edge);
                out += '|';
                out += b::query_edge(body, pc.edge).name;
                out += '\n';
            }
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

} // namespace

// ===========================================================================
// THE GOLDEN — the guide case's every number, from the ADR's table
// ===========================================================================
TEST_CASE("G2-U3 m2 the golden: the box edge filleted at r=0.5 — "
          "10/15/7/H=0/chi=2, the volume 7.5+pi/8 all exact, the "
          "census 30/2, the output gate green") {
    const FilletGuide g(0.5);
    REQUIRE(g.result != nullptr);

    // THE COUNTS — D1.1's table reproduced: the edge dies (-1 edge,
    // -2 corners), four edges born (2 tangents, 2 arcs), four
    // vertices born, one face born.
    REQUIRE(g.result->vertex_count() == 10);
    REQUIRE(g.result->edge_count() == 15);
    REQUIRE(g.result->face_count() == 7);
    REQUIRE(b::inner_loop_count(*g.result) == 0);
    // THE CHI IDENTITY, visible: 10 - 15 + 7 = 2 = the box's own;
    // Delta-chi = +2 -3 +1 = 0 — the fillet is chi-flat.
    REQUIRE(b::euler_characteristic(*g.result) == 2);
    CHECK(b::euler_characteristic(*g.result) ==
          b::euler_characteristic(*g.box));
    CHECK(b::is_closed_orientable(*g.result));
    CHECK(b::loops_are_closed(*g.result));

    // THE CENSUS: every edge exactly two OPPOSITE co-edges; the
    // total 30 incidences over the seven faces (right 4 + back 4 +
    // top 5 + bottom 5 + front 4 + left 4 + flank 4).
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
        // The per-face census, the ADR's own row.
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
    // vertices 6..9 = T_A_lo, T_A_hi, T_B_lo, T_B_hi (A = the back
    // face — the edge's FIRST co-edge sits on face 3; B = right).
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
        // 11 = e_tan_A (6->7), 12 = e_tan_B (8->9), 13 = e_arc_lo
        // (8->6), 14 = e_arc_hi (9->7).
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
        // (1 -> 8) [T_B_lo]; old 2 (2->3) becomes (6 -> 2)
        // [T_A_lo first, the survivor old 3 -> new 2]; old 5 (5->6)
        // becomes (4 -> 9) [the survivor old 5 -> new 4]; old 6
        // (6->7) becomes (7 -> 5) [T_A_hi first, old 7 -> new 5].
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

    // THE VOLUME — the classical form IS the test: 8 - 2*0.25*(1 -
    // pi/4) = 7.5 + pi/8, all_exact. The wedge A(pi/2, r) =
    // r^2*(1-pi/4) verified BY the mass: the removed volume
    // 8 - V == l*A — the flank patch's window integrals and the
    // analytic wedge the same arithmetic read twice.
    {
        const auto mp = b::mass_properties(
            *g.result, CAD_0::math::Tolerance::linear(1e-9));
        const double wedge = 0.25 * (1.0 - kPi / 4.0);
        CHECK(close(mp.volume, 7.5 + kPi / 8.0));
        CHECK(close(8.0 - mp.volume, 2.0 * wedge, 1e-12));
        CHECK(mp.all_exact);
        // THE NAIVE READS, red by discrimination: the no-cut naive
        // (8), the square-cut naive (8 - 2r^2 = 7.5 — the corner
        // window removed as a prism, the quarter discs forgotten),
        // the whole-cylinder displacement (pi r^2 l — what a flank
        // without the window would have to remove). None is the
        // answer; the window IS the discrimination.
        CHECK_FALSE(close(mp.volume, 8.0));
        CHECK_FALSE(close(mp.volume, 8.0 - 2.0 * 0.25));
        CHECK_FALSE(close(8.0 - mp.volume, kPi * 0.25 * 2.0));
        // The centroid: on the x = y diagonal, z = 0 (the wedge's
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

    // The flank's surface: the CylinderZ quarter window — the axis
    // (0.5, 0.5), the radius 0.5, z_base -1, u_sign +1 (the
    // right-handed frame, the material inside the cylinder).
    {
        const auto* surf =
            g.result->solid.shell().faces()[6].surface();
        REQUIRE(surf != nullptr);
        CHECK(surf->family() == b::SurfaceFamily::Cylinder);
        const auto* cyl = dynamic_cast<const b::CylinderZSurface*>(
            surf);
        REQUIRE(cyl != nullptr);
        CHECK(close(cyl->axis_center().x, 0.5, 1e-12));
        CHECK(close(cyl->axis_center().y, 0.5, 1e-12));
        CHECK(close(cyl->radius(), 0.5, 1e-12));
        CHECK(close(cyl->z_base(), -1.0, 1e-12));
        CHECK(cyl->u_sign() == 1.0);
        // THE FRAME RULE, pointwise over the window (the analytic
        // surface's own guard): (du x dv) . n_out > 0.
        for (int k = 0; k <= 8; ++k) {
            const double u = (kPi / 2.0) * k / 8.0;
            const double v = 2.0 * k / 8.0;
            const double du = 1e-7;
            const Vec3d puu = cyl->evaluate(u + du, v);
            const Vec3d pu = cyl->evaluate(u, v);
            const Vec3d pvv = cyl->evaluate(u, v + du);
            const Vec3d d_u = (puu - pu) * (1.0 / du);
            const Vec3d d_v = (pvv - pu) * (1.0 / du);
            const Vec3d cr = d_u.cross(d_v);
            const Vec3d nn = cyl->normal(u, v);
            CHECK(cr.dot(nn) > 0.0);
        }
        // The flank's window IS the rectangle [0, pi/2] x [0, 2]:
        // every p-curve a Line2 on the boundary (the mass
        // machinery's patch class, the u-extent pi/2 — the naive
        // full ring would read 2 pi).
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
        CHECK(close(u_min, 0.0, 1e-12));
        CHECK(close(u_max, kPi / 2.0, 1e-12));
        CHECK(close(v_min, 0.0, 1e-12));
        CHECK(close(v_max, 2.0, 1e-12));
        // The walk: the CCW rectangle (the low arc, the A tangent,
        // the high arc, the B tangent) — the co-edge table.
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
            // The factories' planes: the visitor's typed read.
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
        // The caps' arcs: typed Arc2 in the caps' domains, the
        // material on the disc side (hole = false), the CCW quarter
        // span; the re-spanned and tangent co-edges typed Line2
        // everywhere; every p-curve's endpoints the projected
        // vertices (the DIRECTION convention).
        const auto& top =
            g.result->solid.shell().faces()[1].loops()[0]
                .coedges();
        REQUIRE(top.size() == 5);
        bool arc_seen = false;
        for (const auto& ce : top) {
            const auto* pc = ce.p_curve();
            REQUIRE(pc != nullptr);
            if (ce.edge() == 14) {
                arc_seen = true;
                REQUIRE(pc->kind == b::PCurve::Kind::Arc);
                CHECK(pc->arc.hole == false);
                CHECK(close(pc->arc.radius, 0.5, 1e-12));
                CHECK(close(pc->arc.sweep, kPi / 2.0, 1e-12));
                // The top's domain (u, v) = (-x, -y): the center
                // (0.5, 0.5, 1) -> (-0.5, -0.5).
                CHECK(close(pc->arc.center.x, -0.5, 1e-12));
                CHECK(close(pc->arc.center.y, -0.5, 1e-12));
            } else {
                CHECK(pc->kind == b::PCurve::Kind::Segment);
            }
        }
        REQUIRE(arc_seen);
        // Every co-edge of every face: typed, and the endpoint
        // coherence (the image under the surface == the vertices,
        // within the Linear band — the coherence contract).
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
                    if (pc->kind ==
                        b::PCurve::Kind::Segment) {
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
                    } else if (pc->kind == b::PCurve::Kind::Arc) {
                        const auto q0 = s->evaluate(
                            b::pcurve_arc_start_point(pc->arc).x,
                            b::pcurve_arc_start_point(pc->arc).y);
                        const auto q1 = s->evaluate(
                            b::pcurve_arc_end_point(pc->arc).x,
                            b::pcurve_arc_end_point(pc->arc).y);
                        CHECK((q0 -
                               g.result->vertices[s0].position())
                                  .length_sq() < 1e-18);
                        CHECK((q1 -
                               g.result->vertices[s1].position())
                                  .length_sq() < 1e-18);
                    }
                }
        }
        // The re-authored walks' canonical direction: the caps'
        // and laterals' outer loops still enclose their regions
        // CCW in their domains (the chord polygon's signed area
        // positive — the factory's own discipline, unchanged by the
        // re-authoring).
        const auto signed_area = [](const b::BRepBody& body,
                                    std::size_t f) {
            struct R : b::SurfaceVisitor {
                const b::PlaneSurface* p{};
                void visit(const b::PlaneSurface& pl) override {
                    p = &pl;
                }
            } r;
            body.solid.shell().faces()[f].surface()->accept(r);
            double a = 0.0;
            const auto& ces =
                body.solid.shell().faces()[f].loops()[0]
                    .coedges();
            for (std::size_t k = 0; k < ces.size(); ++k) {
                const auto va = ces[k].start_vertex(body.edges);
                const auto vb =
                    ces[(k + 1) % ces.size()].start_vertex(
                        body.edges);
                const auto da =
                    body.vertices[static_cast<std::size_t>(va)]
                        .position() -
                    r.p->origin();
                const auto db =
                    body.vertices[static_cast<std::size_t>(vb)]
                        .position() -
                    r.p->origin();
                a += da.dot(r.p->u_axis()) * db.dot(r.p->v_axis()) -
                     db.dot(r.p->u_axis()) * da.dot(r.p->v_axis());
            }
            return 0.5 * a;
        };
        for (const std::size_t f : {0u, 1u, 2u, 3u, 4u, 5u}) {
            CAPTURE(f);
            CHECK(signed_area(*g.result, f) > 0.0);
            CHECK(signed_area(*g.box, f) > 0.0);
        }
    }
}

// ===========================================================================
// THE NAMES — D3's table, character for character
// ===========================================================================
TEST_CASE("G2-U3 m2 the names: the flank and the tangent wires, "
          "the same-feature genealogy, the shrink faces' names kept") {
    const FilletGuide g(0.5);
    REQUIRE(g.result != nullptr);

    // The flank: fil#2:flank[box#1:back × box#1:right] — the edge's
    // own declared co-edge order (the first co-edge of edge 10 sits
    // on face 3, the back).
    CHECK(b::query_face(*g.result, 6).name ==
          "fil#2:flank[box#1:back × box#1:right]");
    // The tangent wires: fil#2:isect[<the landing face> × <the
    // flank>] — the level ABSENT, the tangency IS the level; the
    // second operand the SAME feature's flank (the same-feature
    // extension, born earlier in the construction's walk).
    CHECK(b::query_edge(*g.result, 11).name ==
          "fil#2:isect[box#1:back × fil#2:flank[box#1:back × "
          "box#1:right]]");
    CHECK(b::query_edge(*g.result, 12).name ==
          "fil#2:isect[box#1:right × fil#2:flank[box#1:back × "
          "box#1:right]]");
    // The quarter arcs: honestly unnamed (the vocabulary's boundary
    // — the isect grammar names the CONTACT curves; the arcs are
    // the flank's boundary at the caps' planes).
    CHECK(b::query_edge(*g.result, 13).name.empty());
    CHECK(b::query_edge(*g.result, 14).name.empty());
    // The shrink faces and the re-spanned edges KEEP their names
    // (the stamp copy): the mark rides the tavola's absorbed rows.
    CHECK(b::query_face(*g.result, 0).name == "box#1:bottom");
    CHECK(b::query_face(*g.result, 1).name == "box#1:top");
    CHECK(b::query_face(*g.result, 2).name == "box#1:front");
    CHECK(b::query_face(*g.result, 3).name == "box#1:back");
    CHECK(b::query_face(*g.result, 4).name == "box#1:left");
    CHECK(b::query_face(*g.result, 5).name == "box#1:right");
    CHECK(b::query_edge(*g.result, 1).name == "box#1:bottom1");
    CHECK(b::query_edge(*g.result, 2).name == "box#1:bottom2");
    CHECK(b::query_edge(*g.result, 5).name == "box#1:top1");
    CHECK(b::query_edge(*g.result, 6).name == "box#1:top2");
    CHECK(b::query_edge(*g.result, 10).name == "box#1:vertical3");

    // The structural round-trip: the by-name queries answer the
    // addresses (the name plane's own door).
    {
        const auto flank_id =
            g.result->solid.shell().faces()[6].shape_id();
        const auto faces = b::query_face_by_name(*g.result,
                                                 flank_id);
        REQUIRE(faces.size() == 1);
        CHECK(faces[0] == 6);
        const auto wire_id =
            g.result->edges[11].shape_id();
        const auto edges =
            b::query_edge_by_name(*g.result, wire_id);
        REQUIRE(edges.size() == 1);
        CHECK(edges[0] == 11);
    }

    // The unnamed mode: the born entities read honestly empty, the
    // inheritance flows (the shrink faces' names kept — a stamp
    // copy, no table involved).
    {
        const FilletGuide g2(0.5, /*named=*/false);
        REQUIRE(g2.result != nullptr);
        CHECK(b::query_face(*g2.result, 6).name.empty());
        CHECK(b::query_edge(*g2.result, 11).name.empty());
        CHECK(b::query_edge(*g2.result, 12).name.empty());
        CHECK(b::query_face(*g2.result, 3).name == "box#1:back");
        CHECK(b::query_face(*g2.result, 5).name == "box#1:right");
    }

    // The depths: the isect wire's path resolves at depth 5 (the
    // role, the landing leg, the flank's role and its two legs) —
    // kMaxDepth = 8 covers it with room.
    CHECK(g.result->edges[11].shape_id().subshape.depth == 5);
    CHECK(g.result->solid.shell().faces()[6]
              .shape_id()
              .subshape.depth == 3);
}

// ===========================================================================
// THE TAVOLA — the 35 rows, the closure both sides
// ===========================================================================
TEST_CASE("G2-U3 m2 the tavola: the 35 rows, kept/absorbed/born/"
          "vanished counted, the closure green both sides") {
    const FilletGuide g(0.5);
    REQUIRE(g.result != nullptr);
    const b::DescentTable t = b::fillet_descent(*g.result, *g.box);

    // THE COUNT (D3's own arithmetic): 35 rows — the result's 10 +
    // 15 + 7 = 32, plus the vanished tail 1 edge + 2 vertices.
    REQUIRE(t.rows.size() == 35);
    CHECK(t.count(b::Descent::Kept) == 15);      // 6 V + 7 E + 2 F
    CHECK(t.count(b::Descent::Absorbed) == 8);   // 4 E + 4 F
    CHECK(t.count(b::Descent::Born) == 9);       // 4 V + 4 E + 1 F
    CHECK(t.count(b::Descent::Vanished) == 3);   // 2 V + 1 E
    CHECK(t.count(b::Descent::Split) == 0);
    CHECK(t.count(b::Descent::Merged) == 0);

    // The faces: out 7 = 2 kept [front, left] + 4 absorbed [the
    // laterals, the caps] + 1 born [the flank]; vanished 0.
    // The edges: out 15 = 7 kept + 4 absorbed [the re-spanned] +
    // 4 born [the tangents, the arcs]; vanished 1 [the dead
    // vertical — "replaced by the flank"]. The vertices: out 10 =
    // 6 kept + 4 born; vanished 2 [the dead corners, the same
    // cause].
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

    // The vanished rows' cause, verbatim: "replaced by the flank"
    // — the edge's death IS the feature's act.
    {
        std::size_t causes = 0;
        for (const auto& row : t.rows) {
            if (row.relation != b::Descent::Vanished) continue;
            CHECK(row.cause == "replaced by the flank");
            // The dead edge's row: the vertical2's own identity
            // (the tavola's rows carry the dead entity's stamp).
            if (row.entity == b::EntityKind::Edge) {
                CHECK(row.address == 10);
                CHECK(row.shape_id ==
                      g.box->edges[10].shape_id());
            }
            ++causes;
        }
        CHECK(causes == 3);
    }

    // The born rows' names: the flank and the two wires named, the
    // arcs honestly empty.
    for (const auto& row : t.rows) {
        if (row.relation != b::Descent::Born) continue;
        if (row.entity == b::EntityKind::Face)
            CHECK(row.name ==
                  "fil#2:flank[box#1:back × box#1:right]");
        if (row.entity == b::EntityKind::Edge) {
            const bool named = !row.name.empty();
            if (named)
                CHECK(row.name.rfind("fil#2:isect[", 0) == 0);
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
    // same table, byte for byte (the recipe is the source of
    // truth, the tavola is cache).
    const b::DescentTable t2 =
        b::fillet_descent(*g.result, *g.box);
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
// THE COUNTERPROOFS — the naive red, per component (the injected
// defects refused by the very gates the golden passes)
// ===========================================================================
TEST_CASE("G2-U3 m2 the counterproof: the dead edge kept is "
          "chi-odd and co-edge-less — the gate refuses what the "
          "golden passes") {
    const FilletGuide g(0.5);
    REQUIRE(g.result != nullptr);

    // THE NAIVE ARITHMETIC, pinned as the test's own numbers: the
    // construction that forgets the dead edge counts 12 vertices /
    // 16 edges / 7 faces and reads chi = 12 - 16 + 7 = 3 — ODD, the
    // gate's own refusal (a closed orientable body has even chi;
    // genus_from_chi returns nullopt). The golden's 10/15/7 reads
    // 2.
    const std::int64_t naive_chi = 12 - 16 + 7;
    CHECK(naive_chi == 3);
    CHECK(naive_chi % 2 != 0);
    CHECK(b::genus_from_chi(naive_chi) == std::nullopt);
    CHECK(b::euler_characteristic(*g.result) == 2);

    // THE INJECTED DEFECT, live: the dead vertical re-added to the
    // result (its two adjacent faces were re-authored without it —
    // it carries ZERO co-edges). The manifold gate refuses the
    // body the golden passes.
    {
        b::BRepBody naive;
        naive.vertices = g.result->vertices;
        naive.edges = g.result->edges;
        naive.edges.push_back(b::Edge{0, 3});
        naive.edges.back().set_id(
            static_cast<b::EdgeId>(naive.edges.size() - 1));
        naive.solid = g.result->solid;  // the faces, verbatim
        b::link_radial_edges(naive);
        REQUIRE(naive.edges.back().coedges().size() == 0);
        CHECK_FALSE(b::is_closed_orientable(naive));
        CHECK(b::is_closed_orientable(*g.result));
    }

    // THE RE-SPANNING DEFECT, live: the arc stripped from the top
    // cap's loop — the arc is left with ONE co-edge (the flank's)
    // and the cap's walk no longer chains. Both gates refuse. (The
    // naive body is rebuilt with the same mathematical planes — the
    // PlaneSurface constructor is deterministic in the normal, the
    // decoupling test's own bit-identical frames.)
    {
        struct R : b::SurfaceVisitor {
            const b::PlaneSurface* p{};
            void visit(const b::PlaneSurface& pl) override {
                p = &pl;
            }
        };
        b::BRepBody naive;
        naive.vertices = g.result->vertices;
        naive.edges = g.result->edges;
        auto& shell = naive.solid.shell_mut();
        shell.set_closed(true);
        for (std::size_t f = 0; f < 7; ++f) {
            const auto& src =
                g.result->solid.shell().faces()[f];
            b::Face face;
            if (f == 6) {
                // The flank: the cylinder copied through its own
                // getters (the typed dispatch — a plane reader
                // would dereference nothing here).
                struct CR : b::SurfaceVisitor {
                    const b::CylinderZSurface* c{};
                    void visit(
                        const b::CylinderZSurface& cy) override {
                        c = &cy;
                    }
                } cr;
                src.surface()->accept(cr);
                face = b::Face(std::make_shared<
                    b::CylinderZSurface>(
                        cr.c->axis_center(), cr.c->radius(),
                        cr.c->z_base(),
                        cr.c->u_sign() > 0.0 ? 1 : -1));
            } else {
                R rd;
                src.surface()->accept(rd);
                face = b::Face(std::make_shared<b::PlaneSurface>(
                    rd.p->origin(), rd.p->normal_vec()));
            }
            b::Loop loop;
            for (const auto& ce : src.loops()[0].coedges()) {
                if (f == 1 && ce.edge() == 14)
                    continue;  // THE DEFECT: the arc stripped
                loop.add_coedge(ce);
            }
            face.add_loop(std::move(loop));
            face.set_id(static_cast<b::FaceId>(f));
            shell.add_face(std::move(face));
        }
        b::link_radial_edges(naive);
        CHECK_FALSE(b::loops_are_closed(naive));
        REQUIRE(naive.edges[14].coedges().size() == 1);
        CHECK_FALSE(b::is_closed_orientable(naive));
        CHECK(b::loops_are_closed(*g.result));
    }
}

TEST_CASE("G2-U3 m2 the counterproof: the volume's naive reads "
          "red — the window IS the discrimination") {
    // The component's own arithmetic, pinned beside the golden: a
    // flank without the u-window (the full ring) would have to
    // remove the whole cylinder's displacement pi*r^2*l =
    // pi*0.25*2 = pi/2; the square-cut naive (the corner window as
    // a prism) removes 2*r^2 = 0.5; the true wedge removes
    // l*r^2*(1-pi/4) = 0.10730091830127586. Three different
    // numbers — the golden's is the closed form's.
    const double r = 0.5, l = 2.0;
    const double wedge = l * r * r * (1.0 - kPi / 4.0);
    const double square_naive = l * r * r;
    const double cylinder_naive = kPi * r * r * l;
    CHECK(close(wedge, 0.10730091830127586, 1e-15));
    CHECK(wedge < square_naive);
    CHECK(wedge < cylinder_naive);
    CHECK_FALSE(close(wedge, square_naive));
    CHECK_FALSE(close(wedge, cylinder_naive));

    const FilletGuide g(0.5);
    REQUIRE(g.result != nullptr);
    const auto mp = b::mass_properties(
        *g.result, CAD_0::math::Tolerance::linear(1e-9));
    CHECK(close(8.0 - mp.volume, wedge, 1e-12));
    // And the window itself: the quarter [0, pi/2] x [0, 2], never
    // the full ring (2 pi) — the naive's u-extent, pinned red.
    double u_min = 1e9, u_max = -1e9;
    for (const auto& ce :
         g.result->solid.shell().faces()[6].loops()[0].coedges()) {
        const auto* pc = ce.p_curve();
        REQUIRE(pc != nullptr);
        u_min = std::min({u_min, pc->segment.a.x,
                          pc->segment.b.x});
        u_max = std::max({u_max, pc->segment.a.x,
                          pc->segment.b.x});
    }
    CHECK(close(u_max - u_min, kPi / 2.0, 1e-12));
    CHECK_FALSE(close(u_max - u_min, 2.0 * kPi));
}

// ===========================================================================
// THE REFUSALS — the band's bound named, the families by name
// ===========================================================================
TEST_CASE("G2-U3 m2 the refusals: the radius out of band carries "
          "the bound, the class families refuse by name") {
    auto box = [] {
        return b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
    };
    const ShapeId ref{FeatureId{1}, SubshapeRef{}.push(10), {}};
    const b::FilletNaming ctx{FeatureId{2}, &n::kBoxNaming};

    const auto try_fillet = [&](double radius,
                                std::string& msg) {
        try {
            auto r = b::fillet_edge(*box(), ref, radius, ctx);
            (void)r;
            return false;
        } catch (const std::invalid_argument& e) {
            msg = e.what();
            return true;
        }
    };

    // THE BAND (the member this bite can meet): r = 2.5 against
    // the box's reach 2 — the tangent line exits the face, the
    // bound NAMED (the reach, the epsilon, the remedy's number).
    {
        std::string msg;
        REQUIRE(try_fillet(2.5, msg));
        CHECK(msg.find("fillet_radius_exceeds_face") !=
              std::string::npos);
        CHECK(msg.find("2.500000") != std::string::npos);
        CHECK(msg.find("2.000000") != std::string::npos);
        CHECK(msg.find("0.000001") != std::string::npos);
    }
    // The band's edge itself (r exactly at the reach): refused —
    // the tangent line would sit ON the boundary, the strict
    // interiority the derivation demands.
    {
        std::string msg;
        REQUIRE(try_fillet(2.0, msg));
        CHECK(msg.find("fillet_radius_exceeds_face") !=
              std::string::npos);
    }
    // The annulus and the wall members: the box class cannot meet
    // them (no pierced face, no wall span) — they stay registered
    // with their seats in m3's rim class; the payload here never
    // names them.
    {
        std::string msg;
        REQUIRE(try_fillet(2.5, msg));
        CHECK(msg.find("fillet_exits_annulus") ==
              std::string::npos);
        CHECK(msg.find("fillet_eats_wall") == std::string::npos);
    }

    // The non-positive radius: refused with the number.
    {
        std::string msg;
        REQUIRE(try_fillet(0.0, msg));
        CHECK(msg.find("radius") != std::string::npos);
        REQUIRE(try_fillet(-0.5, msg));
        CHECK(msg.find("-0.500000") != std::string::npos);
    }

    // The horizontal edge (a ring edge, not a vertical): the class
    // takes the VERTICAL spigolo — the refusal names it.
    {
        try {
            auto r = b::fillet_edge(
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

    // The reference against the wrong body (the feature part
    // disagrees — box#2's reference on box#1's stamps): the
    // structural find refuses (the closure culture).
    {
        try {
            auto other = b::make_box({1.0, 1.0, 1.0},
                                     FeatureId{2});
            auto r = b::fillet_edge(
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

    // The body outside the class signature: the sphere (1 face, 0
    // edges) and the blind body (12/17/8) both refuse — the
    // signature's own numbers carried.
    {
        const auto sphere = b::make_sphere(1.0, {}, FeatureId{1});
        try {
            auto r = b::fillet_edge(
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
            auto r = b::fillet_edge(
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
    }
}

// ===========================================================================
// THE EDIT CONTRACT — the radius edit class 1 SILENT, the counts
// fixed for every in-band r, the bit-identical rebuild
// ===========================================================================
TEST_CASE("G2-U3 m2 the edit contract: the radius edit resolves "
          "class 1 SILENT, the counts fixed for every in-band r") {
    // THE RESOLUTION (0036 D6's hook, consumed): the reference
    // box#1:vertical2 across the radius edit — the old generation
    // (the box's own tavola), the new (the box re-materialized
    // identically: the radius edit does not touch the box's
    // recipe), the edit context (the parameter edit's no-shift
    // declaration). The answer: reference_resolved — class 1, the
    // feature re-runs, SILENTLY.
    {
        auto box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        const b::DescentTable old_tav =
            b::factory_descent(*box);
        auto box_again =
            b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        const b::Generation old_gen{
            nullptr, &old_tav, &n::kBoxNaming, false};
        const b::Generation new_gen{
            box_again.get(), nullptr, &n::kBoxNaming, false};
        const b::EditContext ctx{/*roots_no_shift=*/true, {}};
        const ShapeId ref{FeatureId{1}, SubshapeRef{}.push(10),
                          {}};
        const b::Resolution r =
            b::resolve(ref, old_gen, new_gen, ctx);
        CHECK(r.kind == b::Resolution::Kind::Resolved);
        CHECK(std::string(b::resolution_code(r.kind)) ==
              "reference_resolved");
        CHECK(r.resolved.name == "box#1:vertical2");
    }

    // THE FEATURE RE-RUNS: the counts stay 10/15/7 for EVERY
    // in-band r, the volume the closed form's own value each time —
    // the parameter edit moves the geometry, never the topology
    // (the class's chi-flatness, the edit contract's own pin). The
    // scene's parameter_changed flow (ONE undo entry) is the
    // recipe/UI morse's own landing (m4/m5, ADR-0039 D6's ladder);
    // the kernel's contract is pinned HERE.
    for (const double radius :
         {0.05, 0.25, 0.5, 0.9, 1.5, 1.9}) {
        CAPTURE(radius);
        const FilletGuide g(radius);
        REQUIRE(g.result != nullptr);
        REQUIRE(g.result->vertex_count() == 10);
        REQUIRE(g.result->edge_count() == 15);
        REQUIRE(g.result->face_count() == 7);
        REQUIRE(b::inner_loop_count(*g.result) == 0);
        REQUIRE(b::euler_characteristic(*g.result) == 2);
        CHECK(b::is_closed_orientable(*g.result));
        const auto mp = b::mass_properties(
            *g.result, CAD_0::math::Tolerance::linear(1e-9));
        CHECK(close(mp.volume,
                    8.0 - 2.0 * radius * radius *
                              (1.0 - kPi / 4.0)));
        CHECK(mp.all_exact);
    }

    // THE BIT-IDENTICAL RE-RUN (the same r, N = 10): the name
    // digest, the co-edge leg and the full-structure digest all
    // byte-identical — ADR-0014's bar, the names pure functions of
    // the recipe (zero floating-point content in the digest's name
    // legs; the structure digest's doubles the closed forms' own).
    {
        std::string first_names, first_coedges;
        std::uint64_t first_struct = 0;
        for (int i = 0; i < 10; ++i) {
            const FilletGuide g(0.5);
            REQUIRE(g.result != nullptr);
            const std::string names = name_digest(*g.result);
            const std::string coedges =
                coedge_name_digest(*g.result);
            const std::uint64_t structure =
                structure_digest(*g.result);
            if (i == 0) {
                first_names = names;
                first_coedges = coedges;
                first_struct = structure;
            } else {
                INFO("rebuild ", i);
                CHECK(names == first_names);
                CHECK(coedges == first_coedges);
                CHECK(structure == first_struct);
            }
        }
    }
}

// ===========================================================================
// THE DIGEST'S PURITY — the names pure functions of the recipe,
// zero floating-point content (the drift's own guard)
// ===========================================================================
TEST_CASE("G2-U3 m2 the digest's purity: the names a pure function "
          "of the recipe — the chain's ids, the operands' order") {
    // The same construction through ANOTHER box instance (a fresh
    // make_box, the same feature ids): the same names, byte for
    // byte — the name is a function of the RECIPE (the feature ids
    // and the roles), never of the body's address.
    const FilletGuide g1(0.5);
    const FilletGuide g2(0.5);
    REQUIRE(g1.result != nullptr);
    REQUIRE(g2.result != nullptr);
    CHECK(name_digest(*g1.result) == name_digest(*g2.result));
    CHECK(coedge_name_digest(*g1.result) ==
          coedge_name_digest(*g2.result));

    // The feature part flows: another fillet id reads fil#3.
    {
        auto box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        const ShapeId ref{FeatureId{1}, SubshapeRef{}.push(10),
                          {}};
        const b::FilletNaming ctx{FeatureId{3},
                                  &n::kBoxNaming};
        auto r = b::fillet_edge(*box, ref, 0.5, ctx);
        REQUIRE(r != nullptr);
        CHECK(b::query_face(*r, 6).name ==
              "fil#3:flank[box#1:back × box#1:right]");
    }

    // Another edge, another operand order: the (+X,-Y) vertical
    // (vertical1, edge 9, vertices 1=(1,-1,-1) -> 5=(1,-1,1)) —
    // the faces right (+X) and front (-Y), the shell's order
    // putting FRONT first (face 2 before face 5).
    {
        auto box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        const ShapeId ref{FeatureId{1}, SubshapeRef{}.push(9), {}};
        const b::FilletNaming ctx{FeatureId{2},
                                  &n::kBoxNaming};
        auto r = b::fillet_edge(*box, ref, 0.5, ctx);
        REQUIRE(r != nullptr);
        REQUIRE(r->vertex_count() == 10);
        REQUIRE(r->edge_count() == 15);
        REQUIRE(r->face_count() == 7);
        CHECK(b::euler_characteristic(*r) == 2);
        CHECK(b::is_closed_orientable(*r));
        // The operand order: the edge 9's first co-edge sits on
        // face 2 (the front) — the flank reads front × right.
        CHECK(b::query_face(*r, 6).name ==
              "fil#2:flank[box#1:front × box#1:right]");
        const auto mp = b::mass_properties(
            *r, CAD_0::math::Tolerance::linear(1e-9));
        CHECK(close(mp.volume, 7.5 + kPi / 8.0));
        CHECK(mp.all_exact);
    }
}
