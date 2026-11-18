// tests/unit/brep/test_g6_m4_the_twelve_edges.cpp
//
// G6 m4 — I DODICI SPIGOLI (ADR-0045 D4/D8.4, the ring-and-miter
// act's own morse): the complete twelve-edge chamfer of the canonical
// cube, atomic. THE DERIVATION, written before the code and pinned
// here (the ADR's D4 own demand):
//
// THE TWO STAGES (D4.1): the vertical passage first (the four
// already-supported walks, each gated — the landed M1/M2 machine
// verbatim, 8+2k/12+3k/6+k up to 16/24/10), then THE RING AND MITER
// ACT, INDIVISIBLE: the miter is born from the THREE planes jointly
// — no two-bevel state contains it — and any sequential order
// re-windows faces born earlier in the same recipe. The act lands
// the whole remaining cube at once.
//
// THE SHAPES (D4.2): the eight MITERS at M = corner −
// (d/2)(n_x+n_y+n_z) — VERTICES, never faces; the TWENTY-FOUR
// RETREATS P_k = corner − d·(the two normals ≠ k), four per
// re-windowed plane; the TWENTY-FOUR MITER LINES (each from a
// retreat to M, each on exactly the two bevels whose planes contain
// it); the SIXTEEN RING SETBACK LINES; the eight RING BEVELS born
// GIÀ RIFINITI as HEXAGONS (both ends mitred, never a cap segment);
// the four VERTICAL BEVELS re-windowed to hexagons in place (the cap
// segments dead with the cause verbatim "replaced by the miter
// line", the wires RE-SPANNED to the retreats — Absorbed, never
// Born-sotto-falso); the six PLANES re-windowed to the retreats'
// rectangles.
//
// THE COUNTS (D4.3, from 16/24/10): dead 16 vertices (the cap tips)
// and 16 edges (the 8 ring edges + the 8 cap segments); born 32
// vertices, 40 edges and 8 faces; re-spanned 8 (the wires) ->
// V/E/F = 32/48/18, chi = 2, H = 0. The co-edge census: 12 bevels ×
// 6 + 6 planes × 4 = 96 / 2 = 48; every retreat 3 faces and 3 edges;
// every miter 3 faces (the three bevels) and 3 edges (the three
// miter lines) — MANIFOLD everywhere.
//
// THE VOLUME (D6.3, the oracle's own equations, ZERO code shared
// with the kernel): the removed corner region Rc = (3/4)d²(2−d) by
// inclusion-exclusion in the retreat coordinates — three half-wedges
// 3·(d²/2) (each int_0^d (d-t)dt for the unit arm — the prism's
// half the corner owns), minus the three pairwise wedges 3·(d³/3)
// (each int_0^d (d-t)²dt), plus the triple d³/4 (THE TETRA-SPLIT
// det/6: the corner tetrahedron d³/6 plus the inner one d³/12) —
// eight congruent corners: V(d) = 8 − 12d² + 6d³, V(0.5) = 5.75
// exact, the limit d -> 1 the rhombododecahedron's 2 (SEEN by the
// closed form, REFUSED by the admission). The coherence with the
// m3 act's own account: R_c = (3/4)d²(2−d) = d²/2 + (d² − (3/4)d³)
// — the corner's vertical half-prism plus its ring share.
//
// THE 17/11 LAW (D4.4, REQUISITO): every span end read from the
// TOPOLOGY — the ring edges' end corners from the PLANES' OWN SIGNS
// (never an ordering of coordinates), the wires' feet substitutes
// through the JUNCTIONS (the cap segments' caps), the corner's
// coordinates PER AXIS (the x lateral carries x, the y lateral y —
// the m3 sweep's own find). The doctor pinned live: the corner's x
// read from the y lateral's origin is red on the very payload.
//
// THE DOORS: chamfer_twelve_edges (the complete operation — the
// fresh box in, the validated 32/48/18 out, the intermediates the
// door's own internals, one feature owning everything born) and
// chamfer_ring_act (the atomic act from the four-vertical state, the
// fillet's ring act one feature over).
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
#include "CAD_0/brep/subtract.hpp"
#include "CAD_0/brep/topology.hpp"
#include "CAD_0/geometry/shape_id.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
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

// The complete operation's guide: the fresh box in, che#2 owning
// everything the door births (one operation, one DAG node).
struct TwelveGuide {
    std::unique_ptr<b::BRepBody> box;
    std::unique_ptr<b::BRepBody> result;

    explicit TwelveGuide(double d = kD) {
        box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        REQUIRE(box != nullptr);
        const b::ChamferNaming ctx{FeatureId{2}, &n::kBoxNaming};
        result = b::chamfer_twelve_edges(*box, d, ctx);
        REQUIRE(result != nullptr);
    }
};

// The manual chain's guide: the four verticals fired per-edge
// (che#2..che#5, the G5 m4 stamp) then the act (che#6) — the
// door-free composition the ring act's own tests read.
struct RingGuide {
    std::unique_ptr<b::BRepBody> box;
    std::vector<std::unique_ptr<b::BRepBody>> owned;
    std::unique_ptr<b::BRepBody> result;
    bool reverse{false};

    explicit RingGuide(double d = kD, bool rev = false) : reverse(rev) {
        box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        REQUIRE(box != nullptr);
        const std::uint32_t order[4] = {11u, 10u, 9u, 8u};
        const b::BRepBody* cur = box.get();
        for (int k = 0; k < 4; ++k) {
            const std::uint32_t role =
                reverse ? order[k]
                        : static_cast<std::uint32_t>(8 + k);
            const b::ChamferNaming ctx{
                FeatureId{static_cast<std::uint64_t>(2 + k)},
                &n::kBoxNaming};
            owned.push_back(b::chamfer_edge(
                *cur,
                ShapeId{FeatureId{1},
                        SubshapeRef{}.push(role), {}},
                d, ctx));
            REQUIRE(owned.back() != nullptr);
            cur = owned.back().get();
        }
        const b::ChamferNaming actx{FeatureId{6}, &n::kBoxNaming};
        result = b::chamfer_ring_act(*cur, d, actx);
        REQUIRE(result != nullptr);
    }

    const b::BRepBody& four() const { return *owned.back(); }
};

// The digests (the m3 test's own shape).
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

// The closed-form volume of the complete twelve-edge chamfer (the
// oracle's own equation — never the kernel's).
double twelve_volume(double d) {
    return 8.0 - 12.0 * d * d + 6.0 * d * d * d;
}

// The per-corner removed region (the coherence identity with the m3
// act's account: the corner's vertical half-prism plus its ring
// share).
double corner_removed(double d) {
    return 0.75 * d * d * (2.0 - d);
}

// The expected born positions at a corner (the derivation's own
// closed forms, in the corner's signs, on the payload's center —
// the translated cube the same closed forms one offset over).
Vec3d miter_at(int sx, int sy, int sz, double d,
               Vec3d c = {0.0, 0.0, 0.0}) {
    return {c.x + sx * (1.0 - 0.5 * d),
            c.y + sy * (1.0 - 0.5 * d),
            c.z + sz * (1.0 - 0.5 * d)};
}
Vec3d retreat_at(int axis, int sx, int sy, int sz, double d,
                 Vec3d c = {0.0, 0.0, 0.0}) {
    const double s[3] = {static_cast<double>(sx),
                         static_cast<double>(sy),
                         static_cast<double>(sz)};
    const double r[3] = {1.0 - (axis == 0 ? 0.0 : d),
                         1.0 - (axis == 1 ? 0.0 : d),
                         1.0 - (axis == 2 ? 0.0 : d)};
    return {c.x + s[0] * r[0], c.y + s[1] * r[1],
            c.z + s[2] * r[2]};
}

// The full per-build contract, shared by the golden and the sweep:
// the counts, the census, the miters and the retreats in the
// corner's own signs, the miter lines on their planes point by
// point, the outward CCW walks, the volume at 1e-12.
void check_full_contract(const b::BRepBody& r, double d,
                         Vec3d center = {0.0, 0.0, 0.0}) {
    REQUIRE(r.vertex_count() == 32);
    REQUIRE(r.edge_count() == 48);
    REQUIRE(r.face_count() == 18);
    REQUIRE(b::inner_loop_count(r) == 0);
    REQUIRE(b::euler_characteristic(r) == 2);
    REQUIRE(b::is_closed_orientable(r));
    REQUIRE(b::loops_are_closed(r));

    // THE CENSUS: every edge exactly two OPPOSITE co-edges; the
    // per-face rows — six planes of four, twelve bevels of six.
    std::size_t total = 0, planes4 = 0, bevels6 = 0;
    for (const auto& e : r.edges) {
        REQUIRE(e.coedges().size() == 2);
        CHECK(e.coedges()[0]->reversed() !=
              e.coedges()[1]->reversed());
    }
    for (std::size_t f = 0; f < 18; ++f) {
        const auto& ces =
            r.solid.shell().faces()[f].loops()[0].coedges();
        total += ces.size();
        if (ces.size() == 4)
            ++planes4;
        else if (ces.size() == 6)
            ++bevels6;
        else
            FAIL("a face carries ", ces.size(),
                 " boundary edges — the contract is 4 (planes) or "
                 "6 (bevels)");
    }
    CHECK(planes4 == 6);
    CHECK(bevels6 == 12);
    REQUIRE(total == 96);

    // THE EIGHT MITERS in the corner's own signs, and their
    // incidence: three faces (the bevels) and three miter-stamped
    // edges each.
    for (int sx_i = 0; sx_i < 2; ++sx_i)
        for (int sy_i = 0; sy_i < 2; ++sy_i)
            for (int sz_i = 0; sz_i < 2; ++sz_i) {
                const int sx = sx_i ? 1 : -1;
                const int sy = sy_i ? 1 : -1;
                const int sz = sz_i ? 1 : -1;
                const Vec3d M =
                    miter_at(sx, sy, sz, d, center);
                std::size_t v_M = r.vertex_count();
                for (std::size_t i = 0; i < r.vertex_count(); ++i) {
                    const Vec3d p = r.vertices[i].position();
                    if (close(p.x, M.x, 1e-12) &&
                        close(p.y, M.y, 1e-12) &&
                        close(p.z, M.z, 1e-12))
                        v_M = i;
                }
                REQUIRE(v_M < r.vertex_count());
                std::size_t miter_edges = 0, miter_faces = 0;
                for (std::size_t i = 0; i < r.edge_count(); ++i) {
                    const auto& e = r.edges[i];
                    if (e.vertex(0) !=
                            static_cast<b::VertexId>(v_M) &&
                        e.vertex(1) !=
                            static_cast<b::VertexId>(v_M))
                        continue;
                    ++miter_edges;
                    if (e.shape_id().subshape.depth > 1 &&
                        e.shape_id().subshape.path[0] ==
                            n::kCheMiterRole)
                        ++miter_faces;
                }
                CHECK(miter_edges == 3);
                CHECK(miter_faces == 3);
            }

    // THE TWENTY-FOUR RETREATS at the closed forms.
    for (int sx_i = 0; sx_i < 2; ++sx_i)
        for (int sy_i = 0; sy_i < 2; ++sy_i)
            for (int sz_i = 0; sz_i < 2; ++sz_i)
                for (int axis = 0; axis < 3; ++axis) {
                    const Vec3d P = retreat_at(
                        axis, sx_i ? 1 : -1, sy_i ? 1 : -1,
                        sz_i ? 1 : -1, d, center);
                    bool found = false;
                    for (std::size_t i = 0;
                         i < r.vertex_count() && !found; ++i) {
                        const Vec3d p = r.vertices[i].position();
                        found = close(p.x, P.x, 1e-12) &&
                                close(p.y, P.y, 1e-12) &&
                                close(p.z, P.z, 1e-12);
                    }
                    CAPTURE(axis);
                    CHECK(found);
                }

    // THE MITER LINES on their planes, point by point: every
    // miter-stamped edge runs from its retreat to its corner's M,
    // each interior sample on EXACTLY TWO of the corner's three
    // bevel planes (the concurrence's own law).
    {
        std::size_t checked = 0;
        for (std::size_t i = 0; i < r.edge_count(); ++i) {
            const auto& e = r.edges[i];
            if (e.shape_id().subshape.depth <= 1 ||
                e.shape_id().subshape.path[0] != n::kCheMiterRole)
                continue;
            const Vec3d& a =
                r.vertices[static_cast<std::size_t>(e.vertex(0))]
                    .position();
            const Vec3d& b =
                r.vertices[static_cast<std::size_t>(e.vertex(1))]
                    .position();
            const int sx = a.x + b.x > 2.0 * center.x ? 1 : -1;
            const int sy = a.y + b.y > 2.0 * center.y ? 1 : -1;
            const int sz = a.z + b.z > 2.0 * center.z ? 1 : -1;
            const double rhs_xy =
                2.0 - d +
                sx * center.x + sy * center.y;
            const double rhs_xz =
                2.0 - d +
                sx * center.x + sz * center.z;
            const double rhs_yz =
                2.0 - d +
                sy * center.y + sz * center.z;
            for (int s = 1; s < 8; ++s) {
                const double t = s / 8.0;
                const Vec3d p = a * (1.0 - t) + b * t;
                int hits = 0;
                if (close(sx * p.x + sy * p.y, rhs_xy, 1e-9))
                    ++hits;
                if (close(sx * p.x + sz * p.z, rhs_xz, 1e-9))
                    ++hits;
                if (close(sy * p.y + sz * p.z, rhs_yz, 1e-9))
                    ++hits;
                CHECK(hits == 2);
            }
            ++checked;
        }
        REQUIRE(checked == 24);
    }

    // THE OUTWARD CCW WALKS, every face: the signed area positive in
    // the domain, the p-curves' images on the vertices, the normal
    // pointing away from the body's centroid.
    {
        Vec3d centroid{0.0, 0.0, 0.0};
        for (const auto& v : r.vertices)
            centroid = centroid + v.position();
        centroid = centroid *
                   (1.0 / static_cast<double>(
                              r.vertex_count()));
        for (std::size_t f = 0; f < 18; ++f) {
            const auto& face = r.solid.shell().faces()[f];
            PlaneReader pr;
            face.surface()->accept(pr);
            REQUIRE(pr.plane != nullptr);
            double area2 = 0.0;
            Vec3d fc{0.0, 0.0, 0.0};
            std::size_t n = 0;
            for (const auto& ce : face.loops()[0].coedges()) {
                const auto* pc = ce.p_curve();
                REQUIRE(pc != nullptr);
                REQUIRE(pc->kind == b::PCurve::Kind::Segment);
                area2 += pc->segment.a.x * pc->segment.b.y -
                         pc->segment.a.y * pc->segment.b.x;
                const auto s0 = static_cast<std::size_t>(
                    ce.start_vertex(r.edges));
                const auto s1 = static_cast<std::size_t>(
                    ce.end_vertex(r.edges));
                const auto q0 = pr.plane->evaluate(
                    pc->segment.a.x, pc->segment.a.y);
                const auto q1 = pr.plane->evaluate(
                    pc->segment.b.x, pc->segment.b.y);
                CHECK((q0 - r.vertices[s0].position())
                          .length_sq() < 1e-18);
                CHECK((q1 - r.vertices[s1].position())
                          .length_sq() < 1e-18);
                fc = fc + r.vertices[s0].position();
                ++n;
            }
            CAPTURE(f);
            CHECK(area2 > 0.0);
            fc = fc * (1.0 / static_cast<double>(n));
            CHECK((fc - centroid).dot(pr.plane->normal_vec()) >
                  0.0);
        }
    }

    // THE VOLUME, the closed form at 1e-12.
    const auto mp =
        b::mass_properties(r, CAD_0::math::Tolerance::linear(1e-9));
    CHECK(close(mp.volume, twelve_volume(d), 1e-12));
    CHECK(mp.all_exact);
}

}  // namespace

// ===========================================================================
// THE GOLDEN — the complete operation, every number the derivation
// wrote
// ===========================================================================
TEST_CASE("G6 m4 the golden: the complete twelve-edge operation — "
          "32/48/18 chi=2, the eight miters, the hexagons, the "
          "volume 5.75, the names, one feature owning everything") {
    const TwelveGuide g;
    check_full_contract(*g.result, kD);

    // THE NAIVE READS, red by discrimination: the box 8, the chain
    // 7.5, the one-corner act 7.34375, the naive prism count
    // 8-12d^2 = 5, the fillet's own Steiner 4+11pi/12.
    {
        const auto mp = b::mass_properties(
            *g.result, CAD_0::math::Tolerance::linear(1e-9));
        CHECK_FALSE(close(mp.volume, 8.0, 1e-6));
        CHECK_FALSE(close(mp.volume, 8.0 - 2.0 * kD * kD, 1e-6));
        CHECK_FALSE(close(mp.volume,
                          8.0 - 3.0 * kD * kD +
                              0.75 * kD * kD * kD,
                          1e-6));
        CHECK_FALSE(close(mp.volume, 8.0 - 12.0 * kD * kD, 1e-6));
        CHECK_FALSE(close(mp.volume,
                          4.0 + 11.0 * 3.14159265358979 / 12.0,
                          1e-3));
        // THE MISSING MITER — one corner left unmitered would read
        // V + Rc; THE INCORRECT COUNT — a missing corner correction
        // 8-12d^2 already red above.
        CHECK_FALSE(close(mp.volume,
                          5.75 + corner_removed(kD), 1e-6));
    }

    // THE TWELVE BEVELS' NAMES, derived from the box's own payload:
    // for each of the twelve edges the operand pair in the edge's
    // own co-edge order (the m1 discipline), rendered under the
    // door's one feature.
    {
        std::vector<std::string> expected;
        for (std::size_t ei = 0; ei < g.box->edge_count(); ++ei) {
            const auto& e = g.box->edges[ei];
            if (e.coedges().size() != 2) continue;
            std::string legs[2];
            for (int c = 0; c < 2; ++c) {
                const b::CoEdge* ce = e.coedges()[c];
                std::size_t face = g.box->face_count();
                for (std::size_t f = 0;
                     f < g.box->face_count(); ++f)
                    for (const auto& loop :
                         g.box->solid.shell().faces()[f].loops())
                        for (const auto& other :
                             loop.coedges())
                            if (&other == ce) face = f;
                REQUIRE(face < g.box->face_count());
                legs[c] = b::query_face(*g.box, face).name;
            }
            expected.push_back("che#2:bevel[" + legs[0] + " × " +
                               legs[1] + "]");
        }
        REQUIRE(expected.size() == 12);
        std::vector<std::string> got;
        for (std::size_t f = 0; f < 18; ++f) {
            const std::string name =
                b::query_face(*g.result, f).name;
            if (name.rfind("che#2:bevel[", 0) == 0)
                got.push_back(name);
        }
        REQUIRE(got.size() == 12);
        for (const auto& name : expected)
            CHECK(std::find(got.begin(), got.end(), name) !=
                  got.end());
        // Two literal pins, the landed tests' own anchor among them:
        // vertical2's bevel (the m1/m3 guide case) and a ring bevel.
        bool has_v2 = false, has_ring = false;
        for (const auto& name : got) {
            if (name ==
                "che#2:bevel[box#1:back × box#1:right]")
                has_v2 = true;
            if (name.find("box#1:top") != std::string::npos)
                has_ring = true;
        }
        CHECK(has_v2);
        CHECK(has_ring);
    }

    // THE EDGE NAME CENSUS: the twenty-four miter lines under the
    // miter role, the sixteen ring setback wires + the eight
    // re-spanned vertical wires under the isect role — every edge
    // named exactly once, the honest grammar.
    {
        std::size_t miters = 0, isects = 0;
        for (std::size_t i = 0; i < 48; ++i) {
            const auto& sub = g.result->edges[i].shape_id().subshape;
            REQUIRE(sub.depth > 1);
            if (sub.path[0] == n::kCheMiterRole)
                ++miters;
            else if (sub.path[0] == n::kCheIsectRole)
                ++isects;
            else
                FAIL("a final edge outside the chamfer's grammar");
        }
        CHECK(miters == 24);
        CHECK(isects == 24);
    }

    // THE RE-SPANNED WIRES: the vertical setback lines now run
    // between the retreats (the m1 state's full-height spans gone).
    {
        // vertical2's wire on the back face: x=1-d, y=1, from
        // z=-(1-d) to z=+(1-d).
        bool found = false;
        for (std::size_t i = 0; i < 48 && !found; ++i) {
            const auto& e = g.result->edges[i];
            const Vec3d& a =
                g.result->vertices[static_cast<std::size_t>(
                    e.vertex(0))]
                    .position();
            const Vec3d& b =
                g.result->vertices[static_cast<std::size_t>(
                    e.vertex(1))]
                    .position();
            const double lo = 1.0 - kD;
            const bool fwd = close(a.x, lo) && close(a.y, 1.0) &&
                             close(a.z, -lo) && close(b.x, lo) &&
                             close(b.y, 1.0) && close(b.z, lo);
            const bool rev = close(b.x, lo) && close(b.y, 1.0) &&
                             close(b.z, -lo) && close(a.x, lo) &&
                             close(a.y, 1.0) && close(a.z, lo);
            found = fwd || rev;
        }
        CHECK(found);
    }

    // THE DETERMINISM: the names and the structure a pure function
    // of the recipe — the re-run bit-identical.
    {
        const TwelveGuide rerun;
        CHECK(name_digest(*rerun.result) ==
              name_digest(*g.result));
        CHECK(structure_digest(*rerun.result) ==
              structure_digest(*g.result));
    }
}

// ===========================================================================
// THE ORACLE — the file's own equations: the inclusion-exclusion at
// the corner on the pieces' OWN parametrizations, the tetra-split,
// the closed form and the kernel at 1e-12, the doctored reads red,
// the honest conditioning note measured
// ===========================================================================
TEST_CASE("G6 m4 the oracle: the half-wedges and the pairwise wedges "
          "by 2e6-midpoint rules, the triple by the tetra-split, the "
          "eight corners summed, the closed form and the kernel at "
          "1e-12, the doctored reads red") {
    const int N = 2'000'000;

    // THE PIECES (each on its own parametrization, the integrands
    // smooth — never an indicator on a grid): (1) one half-wedge —
    // the triangular section int_0^d (d-t) dt for the unit arm the
    // corner owns of its edge's prism (the full prism d^2 split
    // between the two corners, the miter planes the split).
    const auto half_wedge = [&](double d) {
        const double h = d / N;
        double s = 0.0;
        for (int i = 0; i < N; ++i)
            s += (d - (i + 0.5) * h) * h;
        return s;
    };
    // (2) one pairwise wedge: int_0^d (d-t)^2 dt.
    const auto wedge = [&](double d) {
        const double h = d / N;
        double s = 0.0;
        for (int i = 0; i < N; ++i) {
            const double t = (i + 0.5) * h;
            s += (d - t) * (d - t) * h;
        }
        return s;
    };
    // (3) THE TRIPLE by the tetra-split (det/6, the ADR's own
    // words): the corner tetrahedron {u+v+w <= d} is d^3/6 — ALSO
    // quadratured on its own barycentric map (u,v,w) =
    // d(s, t(1-s), r(1-s)(1-t)), the Jacobian FACTORIZED
    // d^3(1-s)^2(1-t) into three smooth 1-D integrands; the inner
    // tetrahedron — (d,0,0),(0,d,0),(0,0,d),(d/2,d/2,d/2) — is
    // d^3/12 by the determinant.
    {
        const double d = kD;
        const double h = 1.0 / N;
        double A = 0.0, B = 0.0, C = 0.0;
        for (int i = 0; i < N; ++i) {
            const double s = (i + 0.5) * h;
            A += (1.0 - s) * (1.0 - s) * h;
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
    const auto triple = [](double d) {
        return d * d * d / 6.0 + d * d * d / 12.0;
    };
    CHECK(close(triple(kD), kD * kD * kD / 4.0, 1e-15));

    // THE INCLUSION-EXCLUSION per corner, then the eight corners
    // (congruent by the cube's symmetry), against the closed form at
    // the declared 2e-6 bar and the kernel at 1e-12 — at EVERY
    // admitted setback the sweep visits, not one.
    for (const double d : {0.3, 0.5, 0.75, 1.0 / 3.0}) {
        const double rc = 3.0 * half_wedge(d) - 3.0 * wedge(d) +
                          triple(d);
        CAPTURE(d);
        CHECK(close(rc, corner_removed(d), 2e-6));
        CHECK(close(8.0 - 8.0 * rc, twelve_volume(d), 2e-6));
        const TwelveGuide g(d);
        const auto mp = b::mass_properties(
            *g.result, CAD_0::math::Tolerance::linear(1e-9));
        CHECK(close(mp.volume, twelve_volume(d), 1e-12));
        CHECK(close(8.0 - 8.0 * rc, mp.volume, 2e-6));
    }

    // THE LIMITS the closed form sees and the admission refuses:
    // d -> 0 -> 8, d -> 1 -> 2 (the rhombododecahedron, (16/9)*sqrt3
    // * a^3 with a = sqrt(3)/2 — 2 exact). The d = 1 build itself is
    // the degeneri's own seat below (refused, never built).
    CHECK(close(twelve_volume(0.0), 8.0, 1e-15));
    CHECK(close(twelve_volume(1.0), 2.0, 1e-15));
    {
        const double a = std::sqrt(3.0) / 2.0;
        CHECK(close((16.0 / 9.0) * std::sqrt(3.0) * a * a * a,
                    2.0, 1e-15));
    }
    // d approaching zero: the build still exact, the volume the
    // closed form's.
    {
        const TwelveGuide g(0.01);
        REQUIRE(g.result->vertex_count() == 32);
        const auto mp = b::mass_properties(
            *g.result, CAD_0::math::Tolerance::linear(1e-9));
        CHECK(close(mp.volume, twelve_volume(0.01), 1e-12));
    }

    // THE COHERENCE identity with the m3 act's account: R_c =
    // (3/4)d^2(2-d) = d^2/2 + (d^2 - (3/4)d^3) — the corner's
    // vertical half-prism plus its ring share.
    CHECK(close(corner_removed(kD),
                0.5 * kD * kD +
                    (kD * kD - 0.75 * kD * kD * kD),
                1e-15));

    // THE HONEST CONDITIONING NOTE, measured (ADR-0045 D6.3's own
    // words): the NAKED INDICATOR on the corner's octant errs O(h)
    // — the flat boundaries have no cancellation — measured here at
    // 126^3 (the SAME 2e6-sample budget the pieces' rules enjoy):
    // ~3.9e-5 against the pieces' own 1e-13-class agreement, twenty
    // times over the declared 2e-6 bar — the pieces' own
    // parametrizations the protocol.
    {
        const double d = kD;
        const int M = 126;
        const double h = 1.0 / M;
        std::size_t inside = 0;
        for (int i = 0; i < M; ++i)
            for (int j = 0; j < M; ++j)
                for (int k = 0; k < M; ++k) {
                    const double u = (i + 0.5) * h;
                    const double v = (j + 0.5) * h;
                    const double w = (k + 0.5) * h;
                    if (u + v < d || v + w < d || u + w < d)
                        ++inside;
                }
        const double indicator =
            static_cast<double>(inside) * h * h * h;
        // O(h), NOT the declared 2e-6 contract the pieces meet at
        // the same budget — but the right order of magnitude.
        CHECK(std::abs(indicator - corner_removed(d)) >
              2e-6);
        CHECK(close(indicator, corner_removed(d), 1e-3));
    }

    // THE DOCTORED READS, each refused by the very comparison the
    // golden passes:
    // (a) THE OFFSET HALVED — the bevel planes read at d/2.
    CHECK_FALSE(close(
        8.0 - 12.0 * (0.5 * kD) * (0.5 * kD) +
            6.0 * (0.5 * kD) * (0.5 * kD) * (0.5 * kD),
        twelve_volume(kD), 1e-6));
    // (b) THE SETBACK ALTERED (+0.01).
    CHECK_FALSE(close(twelve_volume(kD + 0.01),
                      twelve_volume(kD), 1e-6));
    // (c) THE MITER DISPLACED — M pushed along the diagonal: the
    //     three-plane concurrence fails at every sample.
    {
        const Vec3d bad_M{0.75 + 1e-3, 0.75 + 1e-3, 0.75 + 1e-3};
        CHECK_FALSE(close(bad_M.x + bad_M.y, 2.0 - kD, 1e-6));
    }
    // (d) THE MITER'S OCTANT LOST — M read on the corner itself
    //     (d=0, no retreat): the retreats and the miter would
    //     collapse, the volume the naive 8-12d^2.
    CHECK_FALSE(close(8.0 - 12.0 * kD * kD,
                      twelve_volume(kD), 1e-6));
}

// ===========================================================================
// THE RING ACT'S OWN GOLDEN — the manual composition (per-edge
// features, the G5 m4 stamp): the same counts and geometry, the
// nested names, the stamp KEPT through the re-windowing, the
// re-spanned wires read by position
// ===========================================================================
TEST_CASE("G6 m4 the ring act on the manual chain: 32/48/18 with the "
          "same closed forms, the vertical bevels' names and stamps "
          "KEPT (Absorbed, never Born-sotto-falso), the miter legs "
          "nested, the re-spanned wires at the retreats") {
    const RingGuide g;
    check_full_contract(*g.result, kD);

    // THE NAMES: the four vertical bevels keep che#2..che#5 and
    // their operand pairs; the eight ring bevels carry che#6; the
    // miter wires' legs render under the ACT's binding with the
    // carrier's path verbatim in the stamp (the fillet's coaxial
    // convention, the structural check below the truth).
    {
        std::size_t v_bevels = 0, r_bevels = 0;
        for (std::size_t f = 0; f < 18; ++f) {
            const std::string name =
                b::query_face(*g.result, f).name;
            const std::string prefix = name.substr(0, 6);
            if (name.find(":bevel[") == std::string::npos)
                continue;
            const bool ring = prefix == "che#6:";
            if (ring)
                ++r_bevels;
            else
                ++v_bevels;
        }
        CHECK(v_bevels == 4);
        CHECK(r_bevels == 8);
        // The m1 guide case's own bevel, its stamp through the act —
        // vertical2 fired THIRD in this chain, its feature che#4.
        bool has_v2 = false;
        for (std::size_t f = 0; f < 18; ++f)
            if (b::query_face(*g.result, f).name ==
                "che#4:bevel[box#1:back × box#1:right]")
                has_v2 = true;
        CHECK(has_v2);
    }

    // THE STAMP KEPT: the vertical bevels' shape_ids survive the
    // re-windowing VERBATIM (the tavola's Absorbed rows carry them —
    // never Vanished ones under the same names). The bevel born at
    // link k sits at face 6+k of that link's state.
    {
        for (int k = 0; k < 4; ++k) {
            const std::size_t fk =
                6 + static_cast<std::size_t>(k);
            const ShapeId stamp =
                g.owned[static_cast<std::size_t>(k)]
                    ->solid.shell()
                    .faces()[fk]
                    .shape_id();
            bool found = false;
            for (std::size_t f = 0; f < 18; ++f)
                if (g.result->solid.shell().faces()[f]
                        .shape_id() == stamp)
                    found = true;
            CAPTURE(k);
            CHECK(found);
        }
        for (std::size_t f = 0; f < 18; ++f)
            CHECK(g.result->solid.shell()
                      .faces()[f]
                      .shape_id()
                      .subshape.depth > 0);
    }

    // THE MITER STAMP's structural identity, one anchor read fully:
    // a V×R miter's first leg is the vertical bevel's own path
    // pushed verbatim ([miter, bevel, legA, legB, bevel, legA, legB]
    // — depth 7), the faithful identity.
    {
        const ShapeId v_stamp =
            g.owned[0]->solid.shell().faces()[6].shape_id();
        bool seen = false;
        for (std::size_t i = 0; i < 48 && !seen; ++i) {
            const auto& sub =
                g.result->edges[i].shape_id().subshape;
            if (sub.depth != 7 ||
                sub.path[0] != n::kCheMiterRole)
                continue;
            if (sub.path[1] == v_stamp.subshape.path[0] &&
                sub.path[2] == v_stamp.subshape.path[1] &&
                sub.path[3] == v_stamp.subshape.path[2])
                seen = true;
        }
        CHECK(seen);
    }

    // THE RE-SPANNED WIRES at the retreats, one read by position:
    // che#2's right-face wire — x=1, y=1-d, from z=-(1-d) to
    // z=+(1-d) — the m1 state's full-height span (z=-1..1) gone.
    {
        bool found = false;
        for (std::size_t i = 0; i < 48 && !found; ++i) {
            const auto& e = g.result->edges[i];
            const Vec3d& a =
                g.result->vertices[static_cast<std::size_t>(
                    e.vertex(0))]
                    .position();
            const Vec3d& b =
                g.result->vertices[static_cast<std::size_t>(
                    e.vertex(1))]
                    .position();
            const double lo = 1.0 - kD;
            const bool fwd = close(a.x, 1.0) &&
                             close(a.y, lo) &&
                             close(a.z, -lo) && close(b.x, 1.0) &&
                             close(b.y, lo) && close(b.z, lo);
            const bool rev = close(b.x, 1.0) &&
                             close(b.y, lo) &&
                             close(b.z, -lo) && close(a.x, 1.0) &&
                             close(a.y, lo) && close(a.z, lo);
            found = fwd || rev;
        }
        CHECK(found);
    }

    // THE DETERMINISM on the manual chain too.
    {
        const RingGuide rerun;
        CHECK(name_digest(*rerun.result) ==
              name_digest(*g.result));
        CHECK(structure_digest(*rerun.result) ==
              structure_digest(*g.result));
    }
}

// ===========================================================================
// THE SWEEP — D4.4's law one act over: every setback in band, both
// frames (the origin and the translated cube — the closed forms read
// the payload), both doors, the reversed insertion order, the
// repeated determinism, and THE 17/11 DOCTOR live
// ===========================================================================
TEST_CASE("G6 m4 the sweep: three setbacks x both frames x both "
          "doors, the reversed insertion order the same geometry, "
          "the rebuild bit-identical, the 17/11 doctor red at every "
          "corner") {
    const Vec3d offset{3.0, -2.0, 5.0};
    for (const double d : {0.3, 0.5, 0.75}) {
        // (a) the manual chain + the act, origin frame.
        { const RingGuide g(d); check_full_contract(*g.result, d); }
        // (b) the complete door, origin frame.
        {
            const TwelveGuide g(d);
            check_full_contract(*g.result, d);
        }
        // (c) THE TRANSLATED FRAME (the m2 lesson: the payload's own
        //     coordinates, the world's origin nobody's) — both doors.
        {
            auto box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
            b::translate_body(*box, offset);
            const b::ChamferNaming ctx{FeatureId{2},
                                       &n::kBoxNaming};
            auto r = b::chamfer_twelve_edges(*box, d, ctx);
            REQUIRE(r != nullptr);
            check_full_contract(*r, d, offset);
        }
    }

    // THE REVERSED INSERTION ORDER (verticals fired 11,10,9,8): the
    // same counts, the same closed forms, the same volume — and the
    // same TWELVE BEVEL NAMES as a set (the operand pairs are the
    // box's own; the numbering legitimately differs, the digests
    // are per-recipe and both deterministic).
    {
        const RingGuide fwd(kD, false);
        const RingGuide rev(kD, true);
        check_full_contract(*rev.result, kD);
        std::vector<std::string> fwd_names, rev_names;
        for (const b::BRepBody* r : {fwd.result.get(),
                                     rev.result.get()})
            for (std::size_t f = 0; f < 18; ++f) {
                const std::string name =
                    b::query_face(*r, f).name;
                if (name.find(":bevel[") != std::string::npos) {
                    if (r == fwd.result.get())
                        fwd_names.push_back(name);
                    else
                        rev_names.push_back(name);
                }
            }
        REQUIRE(fwd_names.size() == 12);
        REQUIRE(rev_names.size() == 12);
        std::sort(fwd_names.begin(), fwd_names.end());
        std::sort(rev_names.begin(), rev_names.end());
        // The feature ids differ by construction (per-edge
        // che#2..che#5 in the firing order) — compare the OPERAND
        // PAIRS (the names' bevel[...] parts).
        const auto pair_of = [](const std::string& s) {
            return s.substr(s.find("bevel["));
        };
        std::vector<std::string> fwd_pairs, rev_pairs;
        for (std::size_t i = 0; i < 12; ++i) {
            fwd_pairs.push_back(pair_of(fwd_names[i]));
            rev_pairs.push_back(pair_of(rev_names[i]));
        }
        std::sort(fwd_pairs.begin(), fwd_pairs.end());
        std::sort(rev_pairs.begin(), rev_pairs.end());
        CHECK(fwd_pairs == rev_pairs);
        // Both orders deterministic at the re-run.
        {
            const RingGuide rev2(kD, true);
            CHECK(name_digest(*rev2.result) ==
                  name_digest(*rev.result));
            CHECK(structure_digest(*rev2.result) ==
                  structure_digest(*rev.result));
        }
    }

    // THE 17/11 DOCTOR, live on the four-vertical state: the
    // corner's x read from the Y LATERAL'S ORIGIN (the doctored
    // per-axis read — the m3 sweep's own find, the role's lateral
    // permutes, the corner's planes do not) is RED AT EVERY CORNER
    // on the very payload (the factory's laterals carry axis-centred
    // origins: the y lateral's x is 0, the corner's x is +-1) — the
    // construction's own read (the plane's coordinate on ITS OWN
    // axis, the frame's center + the signs) the one the golden
    // passes.
    {
        const RingGuide g(kD, false);
        const b::BRepBody& four = g.four();
        // The two y-laterals (front/back) and the two x-laterals —
        // the AXIS-PURE planes only (the bevels' bisector normals
        // are diagonal: |n.x| = |n.y| = 1/sqrt(2), and a loose
        // >0.5 test would book a bevel as a lateral).
        const b::PlaneSurface* y_lat[2] = {nullptr, nullptr};
        const b::PlaneSurface* x_lat[2] = {nullptr, nullptr};
        for (std::size_t f = 0; f < 10; ++f) {
            PlaneReader pr;
            four.solid.shell().faces()[f].surface()->accept(pr);
            if (pr.plane == nullptr) continue;
            const Vec3d n = pr.plane->normal_vec();
            const bool pure_x =
                std::abs(n.x) > 0.5 && std::abs(n.y) < 0.5 &&
                std::abs(n.z) < 0.5;
            const bool pure_y =
                std::abs(n.y) > 0.5 && std::abs(n.x) < 0.5 &&
                std::abs(n.z) < 0.5;
            if (pure_y)
                y_lat[n.y > 0.0 ? 1 : 0] = pr.plane;
            if (pure_x)
                x_lat[n.x > 0.0 ? 1 : 0] = pr.plane;
        }
        REQUIRE(y_lat[0] != nullptr);
        REQUIRE(y_lat[1] != nullptr);
        REQUIRE(x_lat[0] != nullptr);
        REQUIRE(x_lat[1] != nullptr);
        std::size_t doctor_red = 0;
        for (int sx = -1; sx <= 1; sx += 2)
            for (int sy = -1; sy <= 1; sy += 2) {
                // The doctored corner read: x from the y lateral's
                // origin, y from the x lateral's.
                const double doctored_x =
                    y_lat[sy > 0 ? 1 : 0]->origin().x;
                const double doctored_y =
                    x_lat[sx > 0 ? 1 : 0]->origin().y;
                const bool right =
                    close(doctored_x,
                          static_cast<double>(sx), 1e-12) &&
                    close(doctored_y,
                          static_cast<double>(sy), 1e-12);
                if (!right) ++doctor_red;
            }
        CHECK(doctor_red == 4);  // red at EVERY corner
        // The honest read (each coordinate from its own axis's
        // plane) agrees with the true corner everywhere.
        for (int sx = -1; sx <= 1; sx += 2)
            for (int sy = -1; sy <= 1; sy += 2) {
                const double honest_x =
                    x_lat[sx > 0 ? 1 : 0]->origin().x;
                const double honest_y =
                    y_lat[sy > 0 ? 1 : 0]->origin().y;
                CHECK(close(honest_x,
                            static_cast<double>(sx), 1e-12));
                CHECK(close(honest_y,
                            static_cast<double>(sy), 1e-12));
            }
    }
}

// ===========================================================================
// THE TAVOLA — the act's own descent from the four-vertical state:
// 130 rows, the causes verbatim, the closure both sides
// ===========================================================================
TEST_CASE("G6 m4 the tavola: 130 rows — born 80, absorbed 18, kept 0, "
          "vanished 32; the cap segments' cause verbatim 'replaced by "
          "the miter line', the ring edges' 'replaced by the bevel'; "
          "the closure both sides") {
    const RingGuide g;
    const b::DescentTable t =
        b::chamfer_descent(*g.result, g.four());
    REQUIRE(t.rows.size() == 130);
    CHECK(t.count(b::Descent::Born) == 80);      // 32V + 40E + 8F
    CHECK(t.count(b::Descent::Absorbed) == 18);  // 8E + 10F
    CHECK(t.count(b::Descent::Kept) == 0);
    CHECK(t.count(b::Descent::Vanished) == 32);  // 16V + 16E
    CHECK(t.count(b::Descent::Split) == 0);
    CHECK(t.count(b::Descent::Merged) == 0);

    // THE CAUSES, verbatim: the eight cap segments AND the sixteen
    // dead feet died to the miter line (D4.2's own words — the feet
    // ride their cap segments' causes); the eight ring edges died to
    // the bevels.
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
        CHECK(miter_causes == 24);  // 8 cap segments + 16 feet
        CHECK(bevel_causes == 8);   // the ring edges
    }

    // THE VERTICAL BEVELS' ROWS: Absorbed (the re-windowing, the
    // edit in place), their names on the rows — never a Vanished row
    // under the same names (the Born-sotto-falso the tavola refuses)
    // — and the eight ring bevels Born with che#6's names.
    {
        std::size_t absorbed_v = 0, born_rings = 0;
        for (const auto& row : t.rows) {
            if (row.entity != b::EntityKind::Face) continue;
            if (row.name.find(":bevel[") == std::string::npos)
                continue;
            if (row.name.rfind("che#6:", 0) == 0) {
                CHECK(row.relation == b::Descent::Born);
                ++born_rings;
            } else {
                CHECK(row.relation == b::Descent::Absorbed);
                ++absorbed_v;
            }
        }
        CHECK(absorbed_v == 4);
        CHECK(born_rings == 8);
        for (const auto& row : t.rows) {
            if (row.relation == b::Descent::Vanished &&
                row.entity == b::EntityKind::Face)
                CHECK(row.name.find(":bevel[") ==
                      std::string::npos);
        }
    }

    // THE BORN EDGES' NAMES: the forty born rows all named (the
    // twenty-four miters, the sixteen setback wires) — the honest
    // grammar, the eight re-spanned wires the absorbed rows.
    {
        std::size_t born_named = 0, absorbed_wires = 0;
        for (const auto& row : t.rows) {
            if (row.entity != b::EntityKind::Edge) continue;
            if (row.relation == b::Descent::Born) {
                CHECK_FALSE(row.name.empty());
                ++born_named;
            } else if (row.relation == b::Descent::Absorbed) {
                ++absorbed_wires;
            }
        }
        CHECK(born_named == 40);
        CHECK(absorbed_wires == 8);
    }

    // THE CLOSURE, both sides — the sixteen deaths booked, no ghost.
    std::string why;
    CHECK(b::descent_closed(*g.result, t, &why));
    CHECK(b::old_generation_closed(g.four(), b::BRepBody{}, t,
                                   &why));

    // Determinism of the derivation.
    const b::DescentTable t2 =
        b::chamfer_descent(*g.result, g.four());
    REQUIRE(t2.rows.size() == t.rows.size());
    for (std::size_t i = 0; i < t.rows.size(); ++i) {
        CHECK(t.rows[i].relation == t2.rows[i].relation);
        CHECK(t.rows[i].name == t2.rows[i].name);
        CHECK(t.rows[i].cause == t2.rows[i].cause);
        CHECK(t.rows[i].operands == t2.rows[i].operands);
    }
}

// ===========================================================================
// THE GATE AND THE DIGESTS — the five predicates after EVERY step of
// the six-step chain (four walks + the act), the references HELD
// across the act (the survival accounting: the dead ring edge
// Vanished with its cause, the surviving wire and the vertical
// bevel's stamp RESOLVED — the stamp through the Absorbed edit), the
// digests bit-identical at the re-run
// ===========================================================================
TEST_CASE("G6 m4 the gate: the five predicates green after each of "
          "the six steps, the held references across the act, the "
          "digests bit-identical at the re-run") {
    const RingGuide g;
    const RingGuide rerun;

    // STEPS 1..4 — the vertical walks (the m1 machine, gated again:
    // the link's dead vertical Vanished at its own link, a survivor
    // two links ahead Resolved — and at the last link, where every
    // vertical is dead, a ring edge held alive through the whole
    // passage; the held roles are the EDGE-ONLY ones, 6..11 — the
    // roles 0..5 share their paths with the face table, the m3
    // gate's own lesson).
    for (int k = 0; k < 4; ++k) {
        const b::BRepBody& prev =
            k == 0 ? *g.box
                   : *g.owned[static_cast<std::size_t>(k - 1)];
        const b::DescentTable tk =
            b::chamfer_descent(*g.owned[static_cast<std::size_t>(k)],
                               prev);
        b::ChainStep step;
        step.body = g.owned[static_cast<std::size_t>(k)].get();
        step.tavola = &tk;
        step.previous = &prev;
        // The link's own dead vertical (roles 8..11 in the firing
        // order) and a survivor: the next-in-order vertical while
        // one lives, the ring edge top2 (role 6, edge-only) at the
        // last link — alive through the whole vertical passage.
        const std::uint32_t dead_role =
            g.reverse ? static_cast<std::uint32_t>(11 - k)
                      : static_cast<std::uint32_t>(8 + k);
        step.held.push_back(
            {ShapeId{FeatureId{1},
                     SubshapeRef{}.push(dead_role), {}},
             b::Resolution::Kind::Vanished});
        if (k < 3)
            step.held.push_back(
                {ShapeId{FeatureId{1},
                         SubshapeRef{}.push(dead_role + 1), {}},
                 b::Resolution::Kind::Resolved});
        else
            step.held.push_back(
                {ShapeId{FeatureId{1},
                         SubshapeRef{}.push(6), {}},
                 b::Resolution::Kind::Resolved});
        step.rerun_digest = b::chain_digest(
            *rerun.owned[static_cast<std::size_t>(k)]);
        const b::ChainGateVerdict v = b::chain_gate(step);
        CAPTURE(k);
        CHECK(v.pass);
        CHECK(v.failures.empty());
        CHECK(v.digest ==
              b::chain_digest(
                  *g.owned[static_cast<std::size_t>(k)]));
    }

    // STEP 5 — THE ACT: the dead ring edge box#1:top2 (role 6, an
    // EDGE-ONLY role — the roles 0..5 share their paths with the
    // face table, the honest holds' own law) VANISHED with the
    // tavola's cause; che#2's first setback wire (a born name of
    // the FIRST link) held across the act — RESOLVED, the stamp
    // through the re-spanning; the vertical bevel's own stamp
    // RESOLVED (the Absorbed survival).
    {
        const b::DescentTable t5 =
            b::chamfer_descent(*g.result, g.four());
        b::ChainStep step;
        step.body = g.result.get();
        step.tavola = &t5;
        step.previous = &g.four();
        step.held.push_back(
            {ShapeId{FeatureId{1}, SubshapeRef{}.push(6), {}},
             b::Resolution::Kind::Vanished});
        step.held.push_back(
            {g.owned[0]->edges[12].shape_id(),
             b::Resolution::Kind::Resolved});
        step.held.push_back(
            {g.owned[0]->solid.shell().faces()[6].shape_id(),
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
        CHECK(v.v == 32);
        CHECK(v.e == 48);
        CHECK(v.f == 18);
        CHECK(v.born == 80);
        CHECK(v.absorbed == 18);
        CHECK(v.vanished == 32);
        CHECK(v.digest == b::chain_digest(*g.result));
        // The dead reference's cause, read from the tavola the gate
        // walked: the ring edge died to the bevel.
        bool cause_seen = false;
        for (const auto& row : t5.rows)
            if (row.relation == b::Descent::Vanished &&
                row.entity == b::EntityKind::Edge &&
                row.name == "box#1:top2") {
                CHECK(row.cause == "replaced by the bevel");
                cause_seen = true;
            }
        CHECK(cause_seen);
    }

    // THE DIGESTS' PURITY: the names and the structure pure
    // functions of the recipe, twice more.
    CHECK(name_digest(*rerun.result) == name_digest(*g.result));
    CHECK(structure_digest(*rerun.result) ==
          structure_digest(*g.result));
    for (int i = 0; i < 2; ++i) {
        const RingGuide again;
        CHECK(structure_digest(*again.result) ==
              structure_digest(*g.result));
    }
}

// ===========================================================================
// THE DEGENERI — pinned live, each with its name and its values
// ===========================================================================
TEST_CASE("G6 m4 the degeneri: the state walls (the box, the k=1 "
          "chain, the m3 state), the mismatch with both values, the "
          "mixed-radius chain, the stage band with the two readings, "
          "the d=1 collapse refused at the chain's own door, the "
          "confine 0.99 admitted, the non-cube by name, the "
          "fillet-occupied carrier by family") {
    // THE STATE WALLS — the ring act on anything but the
    // four-vertical state, each refusal naming the doors.
    {
        auto box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        const b::ChamferNaming ctx{FeatureId{6},
                                   &n::kBoxNaming};
        bool threw = false;
        try {
            auto r = b::chamfer_ring_act(*box, 0.5, ctx);
            (void)r;
        } catch (const std::invalid_argument& e) {
            threw = true;
            CHECK(std::string(e.what()).find(
                      "after the FOUR VERTICAL CHAMFERS") !=
                  std::string::npos);
        }
        REQUIRE(threw);
        // The k=1 state.
        const b::ChamferNaming ctx2{FeatureId{2},
                                    &n::kBoxNaming};
        auto first = b::chamfer_edge(
            *box,
            ShapeId{FeatureId{1}, SubshapeRef{}.push(10), {}},
            0.5, ctx2);
        REQUIRE(first != nullptr);
        threw = false;
        try {
            auto r = b::chamfer_ring_act(*first, 0.5, ctx);
            (void)r;
        } catch (const std::invalid_argument& e) {
            threw = true;
            CHECK(std::string(e.what()).find(
                      "after the FOUR VERTICAL CHAMFERS") !=
                  std::string::npos);
        }
        REQUIRE(threw);
        // The twelve-edge door on the k=1 state: the fresh-box wall.
        threw = false;
        try {
            auto r = b::chamfer_twelve_edges(*first, 0.5, ctx);
            (void)r;
        } catch (const std::invalid_argument& e) {
            threw = true;
            CHECK(std::string(e.what()).find(
                      "FRESH factory box") != std::string::npos);
        }
        REQUIRE(threw);
    }

    // THE MISMATCH — the name and BOTH values in the payload (the
    // chain at 0.4, the act asked at 0.5).
    {
        const RingGuide g(0.4);
        const b::ChamferNaming ctx{FeatureId{6},
                                   &n::kBoxNaming};
        bool threw = false;
        try {
            auto r = b::chamfer_ring_act(g.four(), 0.5, ctx);
            (void)r;
        } catch (const std::invalid_argument& e) {
            threw = true;
            const std::string what = e.what();
            CHECK(what.find(
                      "chamfer_corner_setback_mismatch") !=
                  std::string::npos);
            CHECK(what.find("0.500000") != std::string::npos);
            CHECK(what.find("0.400000") != std::string::npos);
        }
        REQUIRE(threw);
    }

    // THE MIXED-RADIUS CHAIN — the verticals at 0.3/0.5 (the chain
    // admits them: the strips never meet), the act refusing at the
    // mismatch whatever d it is asked (the miter needs the common
    // setback — the connected graph's law, D5.4).
    {
        auto box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        std::vector<std::unique_ptr<b::BRepBody>> owned;
        const b::BRepBody* cur = box.get();
        for (int k = 0; k < 4; ++k) {
            const double d = (k % 2 == 0) ? 0.3 : 0.5;
            const b::ChamferNaming ctx{
                FeatureId{static_cast<std::uint64_t>(2 + k)},
                &n::kBoxNaming};
            owned.push_back(b::chamfer_edge(
                *cur,
                ShapeId{FeatureId{1},
                        SubshapeRef{}.push(
                            static_cast<std::uint32_t>(8 + k)),
                        {}},
                d, ctx));
            REQUIRE(owned.back() != nullptr);
            cur = owned.back().get();
        }
        REQUIRE(cur->vertex_count() == 16);
        const b::ChamferNaming actx{FeatureId{6},
                                    &n::kBoxNaming};
        for (const double d : {0.3, 0.4, 0.5}) {
            bool threw = false;
            try {
                auto r = b::chamfer_ring_act(*cur, d, actx);
                (void)r;
            } catch (const std::invalid_argument& e) {
                threw = true;
                CHECK(std::string(e.what()).find(
                          "chamfer_corner_setback_mismatch") !=
                      std::string::npos);
            }
            REQUIRE(threw);
        }
    }

    // THE STAGE BAND with the TWO READINGS — the act asked at 1.0
    // on a 0.5 chain: the faces' current reach 2.000000 would admit
    // it (the single edge's m1 band, satisfied), the stage's
    // 1.000000 refuses (the collapse, the body not the limit).
    {
        const RingGuide g(0.5);
        const b::ChamferNaming ctx{FeatureId{6},
                                   &n::kBoxNaming};
        bool threw = false;
        try {
            auto r = b::chamfer_ring_act(g.four(), 1.0, ctx);
            (void)r;
        } catch (const std::invalid_argument& e) {
            threw = true;
            const std::string what = e.what();
            CHECK(what.find(
                      "chamfer_setback_exceeds_face") !=
                  std::string::npos);
            CHECK(what.find("1.000000") != std::string::npos);
            CHECK(what.find("2.000000") != std::string::npos);
        }
        REQUIRE(threw);
    }

    // THE d = 1 COLLAPSE at the COMPLETE DOOR — the first vertical
    // admits 1.0 (the single edge's own band, the reach 2), the
    // SECOND walk refuses at THE STRIP LAW's own reading: the m1
    // machinery reads the CURRENT face's reach, and the first
    // strip's retreat has tightened it to exactly 2-d (the m2
    // test's own lesson — D1.3's strip law enforced by the landed
    // machinery itself, one wall before the stage band would name
    // the collapse). The door publishes nothing, the source body
    // untouched.
    {
        auto box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        const b::ChamferNaming ctx{FeatureId{2},
                                   &n::kBoxNaming};
        for (const double d : {1.0, 1.5}) {
            bool threw = false;
            try {
                auto r = b::chamfer_twelve_edges(*box, d, ctx);
                (void)r;
            } catch (const std::invalid_argument& e) {
                threw = true;
                const std::string what = e.what();
                // The strip law's own named wall, the bound carried.
                CHECK(what.find(
                          "chamfer_setback_exceeds_face") !=
                      std::string::npos);
                CHECK(what.find("exits the") != std::string::npos);
                CHECK(what.find("the setback line") !=
                      std::string::npos);
            }
            REQUIRE(threw);
        }
        // The non-finite and the non-positive at the door's own
        // gate.
        for (const double d : {0.0, -0.5,
                               std::numeric_limits<double>::
                                   quiet_NaN()}) {
            bool threw = false;
            try {
                auto r = b::chamfer_twelve_edges(*box, d, ctx);
                (void)r;
            } catch (const std::invalid_argument& e) {
                threw = true;
                CHECK(std::string(e.what()).find(
                          "finite and > 0") != std::string::npos);
            }
            REQUIRE(threw);
        }
        // The source untouched: the box still 8/12/6.
        REQUIRE(box->vertex_count() == 8);
        REQUIRE(box->edge_count() == 12);
        REQUIRE(box->face_count() == 6);
    }

    // THE CONFINE at 0.99 — both bands just passed, the build exact.
    {
        const RingGuide g(0.99);
        REQUIRE(g.result->vertex_count() == 32);
        REQUIRE(g.result->edge_count() == 48);
        REQUIRE(g.result->face_count() == 18);
        const auto mp = b::mass_properties(
            *g.result, CAD_0::math::Tolerance::linear(1e-9));
        CHECK(close(mp.volume, twelve_volume(0.99), 1e-9));
        CHECK(mp.all_exact);
    }

    // THE NON-CUBE by name — the box (1, 0.8, 1) chains fine (the
    // m1 reads the faces' own reaches), the act refuses the
    // equal-sides law with the half-extents in the payload.
    {
        auto box = b::make_box({1.0, 0.8, 1.0}, FeatureId{1});
        std::vector<std::unique_ptr<b::BRepBody>> owned;
        const b::BRepBody* cur = box.get();
        for (int k = 0; k < 4; ++k) {
            const b::ChamferNaming ctx{
                FeatureId{static_cast<std::uint64_t>(2 + k)},
                &n::kBoxNaming};
            owned.push_back(b::chamfer_edge(
                *cur,
                ShapeId{FeatureId{1},
                        SubshapeRef{}.push(
                            static_cast<std::uint32_t>(8 + k)),
                        {}},
                0.3, ctx));
            REQUIRE(owned.back() != nullptr);
            cur = owned.back().get();
        }
        REQUIRE(cur->vertex_count() == 16);
        const b::ChamferNaming actx{FeatureId{6},
                                    &n::kBoxNaming};
        bool threw = false;
        try {
            auto r = b::chamfer_ring_act(*cur, 0.3, actx);
            (void)r;
        } catch (const std::invalid_argument& e) {
            threw = true;
            const std::string what = e.what();
            CHECK(what.find("not a cube") != std::string::npos);
            CHECK(what.find("0.800000") != std::string::npos);
        }
        REQUIRE(threw);
    }

    // THE FILLET-OCCUPIED CARRIER — the fillet's own four-vertical
    // state (16/24/10, the counts matching!) refused BY FAMILY at
    // the shared chain-carrier wall: the curved flanks are the mixed
    // corner's family (D5.3), one wall before the act's own
    // business.
    {
        auto box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        std::vector<std::unique_ptr<b::BRepBody>> owned;
        const b::BRepBody* cur = box.get();
        for (int k = 0; k < 4; ++k) {
            const b::FilletNaming ctx{
                FeatureId{static_cast<std::uint64_t>(2 + k)},
                &n::kBoxNaming};
            owned.push_back(b::fillet_edge(
                *cur,
                ShapeId{FeatureId{1},
                        SubshapeRef{}.push(
                            static_cast<std::uint32_t>(8 + k)),
                        {}},
                0.5, ctx));
            REQUIRE(owned.back() != nullptr);
            cur = owned.back().get();
        }
        REQUIRE(cur->vertex_count() == 16);
        REQUIRE(cur->edge_count() == 24);
        REQUIRE(cur->face_count() == 10);
        const b::ChamferNaming actx{FeatureId{6},
                                    &n::kBoxNaming};
        bool threw = false;
        try {
            auto r = b::chamfer_ring_act(*cur, 0.5, actx);
            (void)r;
        } catch (const std::invalid_argument& e) {
            threw = true;
            CHECK(std::string(e.what()).find(
                      "outside the class families") !=
                  std::string::npos);
        }
        REQUIRE(threw);
    }
}


