// tests/unit/brep/test_g1_unit4_subtract.cpp
//
// G1 UNIT 4 — THE EXACT SUBTRACT: the discriminating tests (ADR-0032
// D5+D7, written FIRST per D9 — they fail today for the right reason:
// the SSI entry, the class check and the builder do not exist yet;
// the build itself names the absence, the unit-1 precedent).
//
// What this file pins, family by family (the order's AREA 5):
//
//   * THE SSI ENTRY (D5) — plane∩cylinder in the three orientations:
//     ⊥ the circle (typed on both domains), ∥ the line pair / the
//     tangent double line / the empty case, OBLIQUE the refusal data
//     (the angle — the sinusoid p-curve is G3's, never smuggled in);
//   * THE GUIDE CASE (D7.5's pinned table) — box a×b×c minus the
//     through circle prism: V=12, E=17, F=7, H=2, χ=0, genus 1; the
//     wall is ONE CylinderSurface-family face with the sketch's
//     radius (today's faceted extract: N planar strips); the pierced
//     faces KEEP their outer loops (the factory's own p-curves) and
//     GAIN inner loops (the intersection circle split at the seam
//     pair, ADR-0023's pattern);
//   * THE FORMULAS — every tool profile the extrude can author: a
//     chain of n entities → V=V_A+2n, E=E_A+3n, F=F_A+n, H=2, χ=0
//     (the square n=4, the mixed sector n=3, a prism TARGET);
//   * THE DEGENERATES, each with its pinned disposition (D7.4): the
//     coplanar cap ABSORBED (the same counts), the tangential wall
//     REFUSED (the deterministic tie-break), the crossing circle
//     REFUSED (actionable), the blind hole REFUSED (actionable), the
//     oblique target REFUSED (the angle named), the Y-axis tool
//     REFUSED (the v0.3 chain's shape), the swallow REFUSED, the miss
//     the EXACT COPY, the non-convex/ring/non-exact operands REFUSED;
//   * THE LANDING — the subtract node owns BOTH materializations
//     (HybridBody, bridge tolerance 0); the v0.4 golden materializes
//     the exact subtract NOW (the faceted superstate retires, the
//     reason documented); the v0.3 golden STAYS faceted (its tool is
//     the Y-axis primitive cylinder — the honest refusal recorded);
//   * DETERMINISM (ADR-0014) — a radius edit and an offset edit
//     regenerate BOTH materializations bit-identically.
//
// The conventions this unit discovers are pinned here as regression
// guards (the header documents the derivation):
//   * the surviving wall keeps the prism's PHYSICAL window walk,
//     re-framed by the flipped orientation — it reads REGION-ON-RIGHT
//     in its own domain (interior_left = false), because the pierced
//     faces' inner loops must read CCW for the Arc2 vocabulary (CCW
//     sweeps only) and the radial-edge XOR ties the two: one
//     requirement, seen from both sides;
//   * the inner loops' region bits are FLIPPED from the caps' (the
//     material is now OUTSIDE the profile: hole = true, and the
//     segments' interior_left negated) — the bits carry the region,
//     the unit-3 material-right precedent applied to a hole boundary.
#include <doctest/doctest.h>

#include "CAD_0/brep/subtract.hpp"
#include "CAD_0/brep/extrude.hpp"
#include "CAD_0/brep/analytic.hpp"
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/query.hpp"
#include "CAD_0/brep/topology.hpp"
#include "CAD_0/sdf/sketch_entities.hpp"
#include "CAD_0/sdf/sketch_profile.hpp"
#include "CAD_0/sdf/sketch_extrude.hpp"
#include "CAD_0/bridge/sdf_to_brep.hpp"
#include "CAD_0/scene/scene.hpp"
#include "CAD_0/scene/operations.hpp"
#include "CAD_0/io/native.hpp"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

namespace {

using CAD_0::brep::BRepBody;
using CAD_0::brep::EdgeId;
using CAD_0::brep::FaceId;
using CAD_0::brep::VertexId;
using CAD_0::math::Vec2f;
using CAD_0::math::Vec3f;
namespace b = CAD_0::brep;
namespace s = CAD_0::sdf;

constexpr float kPi = 3.14159265358979f;
constexpr float kTwoPi = 6.28318530717958647692f;

bool close(float a, float b, float tol = 1e-5f) { return std::abs(a - b) <= tol; }
bool close2(Vec2f a, Vec2f b, float tol = 1e-5f) {
    return close(a.x, b.x, tol) && close(a.y, b.y, tol);
}
bool close3(Vec3f a, Vec3f b, float tol = 1e-5f) {
    return close(a.x, b.x, tol) && close(a.y, b.y, tol) &&
           close(a.z, b.z, tol);
}

// ---- the profiles (the unit-3 vocabulary, reused) -----------------------

// The circle profile: the guide case's tool.
std::vector<s::SketchChain> circle_chains(Vec2f center = {0.0f, 0.0f},
                                          float radius = 0.25f) {
    return {s::SketchChain{{s::sketch_circle(center, radius, false)}}};
}

// The square profile (a segment-only convex chain, n = 4).
std::vector<s::SketchChain> square_chains(Vec2f lo = {-0.3f, -0.3f},
                                          Vec2f hi = {0.3f, 0.3f}) {
    return {s::SketchChain{{
        s::sketch_segment(lo, {hi.x, lo.y}, true),          // bottom, east
        s::sketch_segment({hi.x, lo.y}, hi, true),          // right, north
        s::sketch_segment(hi, {lo.x, hi.y}, true),          // top, west
        s::sketch_segment({lo.x, hi.y}, lo, true),          // left, south
    }}};
}

// The sector: 2 segments from the centre + the closing arc
// (n = 3, segments + a fillet arc — the mixed profile).
std::vector<s::SketchChain> sector_chains() {
    const float a0 = -0.2f * kPi, a1 = 0.35f * kPi;
    const float r = 0.4f;
    const Vec2f c{0.0f, 0.0f};
    const Vec2f ps = c + Vec2f{r * std::cos(a0), r * std::sin(a0)};
    const Vec2f pe = c + Vec2f{r * std::cos(a1), r * std::sin(a1)};
    return {s::SketchChain{{
        s::sketch_segment(c, ps, true),
        s::sketch_arc(c, r, false, a0, a1 - a0),
        s::sketch_segment(pe, c, true),
    }}};
}

// A hand-built NON-CONVEX L prism — the refused tool class, as a
// synthetic class-check fixture (the unit-1 annulus precedent: the
// 2D kernel itself cannot author a non-convex chain — its region
// composition intersects half-planes, and an intersection of
// half-planes is convex — so the non-convex TOOL cannot come from
// the exact world; the fixture feeds the class check directly).
// Structure: the L profile's 6 joints CCW (J0..J5), z in [-1.5, 1.5]:
// 12 vertices (2i = J_i at z_bot, 2i+1 = at z_top), 18 edges, 8
// faces (6 vertical laterals + the -Z/+Z caps), every face planar
// with ONE loop (the caps' loops carry the 6 chain co-edges; the
// laterals' loops are placeholders — the class check reads the
// surfaces, the loop counts and the vertex half-spaces, never the
// lateral content).
std::unique_ptr<BRepBody> ell_prism_body() {
    const Vec2f J[6] = {{-0.6f, -0.6f}, {0.6f, -0.6f}, {0.6f, 0.1f},
                        {0.1f, 0.1f},  {0.1f, 0.6f}, {-0.6f, 0.6f}};
    const Vec3f lat_n[6] = {{0, -1, 0}, {1, 0, 0}, {0, 1, 0},
                            {1, 0, 0},  {0, 1, 0}, {-1, 0, 0}};
    auto body = std::make_unique<BRepBody>();
    auto& shell = body->solid.shell_mut();
    shell.set_closed(true);
    for (std::size_t i = 0; i < 6; ++i) {
        b::Vertex vb{Vec3f{J[i].x, J[i].y, -1.5f}};
        vb.set_id(static_cast<b::VertexId>(2 * i));
        body->vertices.push_back(vb);
        b::Vertex vt{Vec3f{J[i].x, J[i].y, 1.5f}};
        vt.set_id(static_cast<b::VertexId>(2 * i + 1));
        body->vertices.push_back(vt);
    }
    for (std::size_t i = 0; i < 6; ++i) {
        const std::size_t prev = (i + 5) % 6;
        b::Edge eb{static_cast<b::VertexId>(2 * prev), static_cast<b::VertexId>(2 * i)};
        eb.set_id(static_cast<b::EdgeId>(i));
        body->edges.push_back(eb);
        b::Edge et{static_cast<b::VertexId>(2 * prev + 1),
                static_cast<b::VertexId>(2 * i + 1)};
        et.set_id(static_cast<b::EdgeId>(6 + i));
        body->edges.push_back(et);
        b::Edge ev{static_cast<b::VertexId>(2 * i),
                static_cast<b::VertexId>(2 * i + 1)};
        ev.set_id(static_cast<b::EdgeId>(12 + i));
        body->edges.push_back(ev);
    }
    {
        b::Face f(std::make_shared<b::PlaneSurface>(Vec3f{0, 0, -1.5f},
                                                 Vec3f{0, 0, -1}));
        b::Loop l;
        for (std::size_t i = 0; i < 6; ++i)
            l.add_coedge(b::CoEdge{static_cast<b::EdgeId>(i), false});
        f.add_loop(std::move(l));
        f.set_id(0);
        shell.add_face(std::move(f));
    }
    {
        b::Face f(std::make_shared<b::PlaneSurface>(Vec3f{0, 0, 1.5f},
                                                 Vec3f{0, 0, 1}));
        b::Loop l;
        for (std::size_t i = 0; i < 6; ++i)
            l.add_coedge(b::CoEdge{static_cast<b::EdgeId>(6 + i), false});
        f.add_loop(std::move(l));
        f.set_id(1);
        shell.add_face(std::move(f));
    }
    for (std::size_t i = 0; i < 6; ++i) {
        b::Face f(std::make_shared<b::PlaneSurface>(
            Vec3f{J[i].x, J[i].y, -1.5f}, lat_n[i]));
        b::Loop l;
        f.add_loop(std::move(l));
        f.set_id(static_cast<b::FaceId>(2 + i));
        shell.add_face(std::move(f));
    }
    return body;
}

// The ring profile (a hole circle — the caps carry inner loops: the
// refused tool class).
std::vector<s::SketchChain> ring_tool_chains() {
    auto chains = square_chains({-0.5f, -0.5f}, {0.5f, 0.5f});
    chains.push_back(s::SketchChain{
        {s::sketch_circle({0.0f, 0.0f}, 0.1f, true)}});
    return chains;
}

// ---- the structural contract --------------------------------------------

void check_exact_provenance(const BRepBody& body) {
    for (const auto& v : body.vertices) {
        CHECK(v.provenance().is_exact());
        CHECK(v.provenance().linear_tolerance == 0.0);
    }
    for (const auto& e : body.edges) {
        CHECK(e.provenance().is_exact());
        CHECK(e.provenance().linear_tolerance == 0.0);
    }
    for (const auto& f : body.solid.shell().faces()) {
        CHECK(f.provenance().is_exact());
        CHECK(f.provenance().linear_tolerance == 0.0);
    }
}

// (body, V, E, F, H, chi) — the counts, the closed-orientable
// contract, the loop closure, and the manifold XOR on every edge.
void check_body_invariants(const BRepBody& body, std::size_t V, std::size_t E,
                           std::size_t F, std::int64_t H, std::int64_t chi) {
    REQUIRE(body.vertex_count() == V);
    REQUIRE(body.edge_count() == E);
    REQUIRE(body.face_count() == F);
    REQUIRE(b::inner_loop_count(body) == H);
    REQUIRE(b::euler_characteristic(body) == chi);
    CHECK(b::is_closed_orientable(body));
    CHECK(b::loops_are_closed(body));
    // id == vector index (the factories' numbering discipline).
    for (std::size_t i = 0; i < body.vertices.size(); ++i)
        CHECK(body.vertices[i].id() == static_cast<b::VertexId>(i));
    for (std::size_t i = 0; i < body.edges.size(); ++i)
        CHECK(body.edges[i].id() == static_cast<b::EdgeId>(i));
    for (std::size_t i = 0; i < body.face_count(); ++i)
        CHECK(body.solid.shell().faces()[i].id() == static_cast<b::FaceId>(i));
    // Every edge: exactly two co-edges, opposite orientation (the
    // radial-edge XOR — the assembly-detecting invariant).
    for (const auto& e : body.edges) {
        REQUIRE(e.coedges().size() == 2);
        CHECK(e.coedges()[0]->reversed() != e.coedges()[1]->reversed());
    }
}

// The guide scene's pieces: the box target + the circle tool prism
// translated so it spans z in [-1.5, 1.5] (a strict through-hole in
// the [-1,1]^3 box), the disc at the origin strictly interior.
struct GuideCase {
    std::unique_ptr<BRepBody> a;   // make_box({1,1,1})
    std::unique_ptr<BRepBody> b;   // make_prism(circle r=0.25, 3.0, +1)
    Vec3f offset{0.0f, 0.0f, -1.5f};
};

GuideCase guide_case() {
    GuideCase g;
    g.a = b::make_box({1.0f, 1.0f, 1.0f});
    g.b = b::make_prism(circle_chains(), 3.0f, +1);
    return g;
}

// Every p-curve of a face's loop, chained endpoint-to-endpoint in the
// domain (the DIRECTION convention's own closure discipline).
bool pcurves_chain(const BRepBody& body, const CAD_0::brep::Face& face,
                   std::size_t loop_index) {
    const auto& loop = face.loops()[loop_index];
    const std::size_t n = loop.coedges().size();
    for (std::size_t i = 0; i < n; ++i) {
        const auto* pc = loop.coedges()[i].p_curve();
        if (pc == nullptr) return false;
        const auto* next = loop.coedges()[(i + 1) % n].p_curve();
        if (next == nullptr) return false;
        Vec2f end{}, next_start{};
        if (pc->kind == s::SketchEntity::Kind::Segment) {
            end = pc->segment.b;
        } else if (pc->kind == s::SketchEntity::Kind::Arc) {
            end = s::arc_end_point(pc->arc);
        } else {
            return false;  // a Circle2 is closed: single-co-edge loops only
        }
        if (next->kind == s::SketchEntity::Kind::Segment) {
            next_start = next->segment.a;
        } else if (next->kind == s::SketchEntity::Kind::Arc) {
            next_start = s::arc_start_point(next->arc);
        } else {
            return false;
        }
        if (!close2(end, next_start)) return false;
    }
    (void)body;
    return true;
}

// The query digest (the unit-3 determinism oracle): every face's full
// query answer as one comparable string.
std::string query_digest(const BRepBody& body) {
    std::string out;
    for (std::size_t i = 0; i < body.face_count(); ++i) {
        out += CAD_0::brep::query_face(body, static_cast<b::FaceId>(i)).family;
        out += ":";
        const auto q = CAD_0::brep::query_face(body, static_cast<b::FaceId>(i));
        for (const auto& loop : q.loops) {
            out += "[";
            for (const auto& ce : loop.coedges) {
                out += std::to_string(ce.edge) + "," +
                       (ce.reversed ? "R" : "F") + ",";
                switch (ce.kind) {
                case CAD_0::brep::PCurveInfo::Kind::None: out += "none"; break;
                case CAD_0::brep::PCurveInfo::Kind::Line:
                    out += "l(" + std::to_string(ce.a.x) + "," +
                           std::to_string(ce.a.y) + ")->(" +
                           std::to_string(ce.b.x) + "," +
                           std::to_string(ce.b.y) + ")," +
                           (ce.interior_left ? "L" : "R");
                    break;
                case CAD_0::brep::PCurveInfo::Kind::Circle:
                    out += "c(" + std::to_string(ce.center.x) + "," +
                           std::to_string(ce.center.y) + ")," +
                           std::to_string(ce.radius) + "," +
                           (ce.hole ? "H" : "D");
                    break;
                case CAD_0::brep::PCurveInfo::Kind::Arc:
                    out += "a(" + std::to_string(ce.center.x) + "," +
                           std::to_string(ce.center.y) + ")," +
                           std::to_string(ce.radius) + "," +
                           (ce.hole ? "H" : "D") + "," +
                           std::to_string(ce.start_angle) + "," +
                           std::to_string(ce.sweep);
                    break;
                }
                out += ";";
            }
            out += "]";
        }
        out += "\n";
    }
    return out;
}

std::string sdf_digest(const CAD_0::sdf::SDFBody& body) {
    // A deterministic sample of the field (the unit-3 oracle's shape).
    std::string out;
    for (int i = 0; i <= 6; ++i)
        for (int j = 0; j <= 6; ++j)
            for (int k = 0; k <= 6; ++k) {
                const Vec3f p{-1.3f + 0.4f * static_cast<float>(i),
                              -1.3f + 0.4f * static_cast<float>(j),
                              -1.3f + 0.4f * static_cast<float>(k)};
                out += std::to_string(body.sample(p).value) + ",";
            }
    return out;
}

std::uint32_t add_extrude(CAD_0::scene::Scene& sc, const std::string& name,
                          std::uint32_t sketch_id, float distance,
                          int direction) {
    CAD_0::scene::OperationRecipe r;
    r.type = CAD_0::scene::OperationType::Extrude;
    r.sketch = sketch_id;
    r.distance = distance;
    r.direction = direction;
    return sc.add_operation_node(name, r);
}

std::uint32_t add_subtract(CAD_0::scene::Scene& sc, const std::string& name,
                           std::uint32_t target, std::uint32_t tool,
                           Vec3f offset = {0.0f, 0.0f, 0.0f}) {
    CAD_0::scene::OperationRecipe r;
    r.type = CAD_0::scene::OperationType::Subtract;
    r.target = target;
    r.tool = tool;
    r.offset_x = offset.x;
    r.offset_y = offset.y;
    r.offset_z = offset.z;
    return sc.add_operation_node(name, r);
}

// A factory box node (the bridge's primitive path serves its exact
// B-Rep — the class check's other operand source).
std::uint32_t add_box_node(CAD_0::scene::Scene& sc, const std::string& name,
                           Vec3f half) {
    return sc.add_node(name, s::make_box(half));
}

} // namespace

// ---------------------------------------------------------------------------
// FAMILY 1 — the SSI entry (D5): the three orientations, typed
// ---------------------------------------------------------------------------
TEST_CASE("G1-U4 SSI: plane perpendicular to the axis - the CIRCLE, typed") {
    // The plane z = 1 (normal +Z) cutting the cylinder centred
    // (0.5, 0.25), r = 1.25, z_base = -1: the circle of radius 1.25
    // centred where the axis meets the plane; on the cylinder's (u,v)
    // domain the horizontal line v = 2 (z - z_base).
    const b::PlaneSurface plane{{0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 1.0f}};
    const b::CylinderZSurface cyl{{0.5f, 0.25f}, 1.25f, -1.0f, +1};

    const auto ssi = b::ssi_plane_cylinder(plane, cyl, b::kSubtractTolerance,
                              b::kSubtractTolerance);
    REQUIRE(ssi.kind == b::PlaneCylinderSSI::Kind::Circle);
    CHECK(close3(ssi.circle_center, {0.5f, 0.25f, 1.0f}));
    CHECK(close(ssi.circle_radius, 1.25f));
    CHECK(close(ssi.cylinder_v, 2.0f));
    // (The plane-domain p-curve of this circle is a Circle2 centred at
    // the axis point's domain image — the payload the pipeline authors
    // on the pierced face; the geometric pins above carry the entry's
    // side of the contract.)
}

TEST_CASE("G1-U4 SSI: plane parallel to the axis - the LINE PAIR, tangent, empty") {
    const b::CylinderZSurface cyl{{0.5f, 0.25f}, 1.25f, -1.0f, +1};

    // A vertical plane through x = 0.5 (normal +X): the axis-to-plane
    // distance d = 0 < r -> TWO vertical lines at azimuth +/- pi/2.
    {
        const b::PlaneSurface plane{{0.5f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}};
        const auto ssi = b::ssi_plane_cylinder(plane, cyl,
                                               b::kSubtractTolerance,
                                               b::kSubtractTolerance);
        REQUIRE(ssi.kind == b::PlaneCylinderSSI::Kind::LinePair);
        CHECK(close(ssi.axis_distance, 0.0f));
        // The azimuths: the plane's normal direction mu = 0; the lines
        // at phi = mu +/- arccos(d/r) = +/- pi/2.
        CHECK(close(std::abs(std::cos(ssi.phi_a)), 0.0f, 1e-4f));
        CHECK(close(std::abs(std::cos(ssi.phi_b)), 0.0f, 1e-4f));
        CHECK(close3(ssi.line_dir, {0.0f, 0.0f, 1.0f}));
        // Each line's point lies on the cylinder and on the plane.
        const Vec3f pa = ssi.line_point;
        CHECK(close(std::hypot(pa.x - 0.5f, pa.y - 0.25f), 1.25f));
        CHECK(close(pa.x, 0.5f));
    }

    // Tangent: the plane at distance exactly r -> the DOUBLE line.
    {
        const b::PlaneSurface plane{{1.75f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}};
        const auto ssi = b::ssi_plane_cylinder(plane, cyl,
                                               b::kSubtractTolerance,
                                               b::kSubtractTolerance);
        REQUIRE(ssi.kind == b::PlaneCylinderSSI::Kind::TangentLine);
        CHECK(close(ssi.axis_distance, 1.25f));
        CHECK(close(ssi.phi_a, ssi.phi_b));
    }

    // Beyond: no intersection at all.
    {
        const b::PlaneSurface plane{{2.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}};
        const auto ssi = b::ssi_plane_cylinder(plane, cyl,
                                               b::kSubtractTolerance,
                                               b::kSubtractTolerance);
        REQUIRE(ssi.kind == b::PlaneCylinderSSI::Kind::Empty);
    }
}

TEST_CASE("G1-U4 SSI: the OBLIQUE plane - refused with the angle (the sinusoid is G3's)") {
    const b::CylinderZSurface cyl{{0.0f, 0.0f}, 1.0f, -1.0f, +1};
    // A 45-degree tilted plane through the origin.
    const b::PlaneSurface plane{{0.0f, 0.0f, 0.0f},
                                {0.0f, -kPi / 4.0f, kPi / 4.0f}};
    const auto ssi = b::ssi_plane_cylinder(plane, cyl, b::kSubtractTolerance,
                              b::kSubtractTolerance);
    REQUIRE(ssi.kind == b::PlaneCylinderSSI::Kind::Oblique);
    // The angle between the plane's normal and the axis: 45 degrees.
    CHECK(close(ssi.angle, kPi / 4.0f, 1e-4f));
}

// ---------------------------------------------------------------------------
// FAMILY 2 — THE GUIDE CASE (D7.5's pinned table): box [-1,1]^3 minus
// the circle prism r=0.25 through Z.  V=12, E=17, F=7, H=2, chi=0.
// ---------------------------------------------------------------------------
TEST_CASE("G1-U4 guide: box minus the circle prism - THE pinned table 12/17/7/2") {
    const GuideCase g = guide_case();
    auto r = b::exact_subtract(*g.a, *g.b, g.offset, b::kSubtractTolerance);
    REQUIRE(r != nullptr);

    check_body_invariants(*r, 12, 17, 7, 2, 0);
    CHECK(b::genus_from_chi(b::euler_characteristic(*r)).value_or(-1) == 1);
    check_exact_provenance(*r);

    // ---- THE VERTICES (the numbering discipline, mirrored from the
    // extrude's own tables): 0..7 the box's corners untouched; then
    // the tool's canonical walk (the +Z cap's, CCW in the world):
    // J_0 = the seam S (+X), J_1 = the antipode A (-X). 2i = J_i at
    // z_bot, 2i+1 = J_i at z_top.
    const Vec3f pv[12] = {
        {-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},   // box bottom
        {-1, -1, 1},  {1, -1, 1},  {1, 1, 1},  {-1, 1, 1},    // box top
        {0.25f, 0, -1}, {0.25f, 0, 1},                        // S_b, S_t
        {-0.25f, 0, -1}, {-0.25f, 0, 1},                      // A_b, A_t
    };
    // (The tool's joints are read from the cap's arc/segment endpoint
    // images — trigonometry carries ~1 ulp, so the pin is geometric
    // (1e-6), not bit-exact; the DETERMINISM pins below carry the
    // bit-stability side of the contract.)
    for (std::size_t i = 0; i < 12; ++i) {
        CAPTURE(i);
        CHECK(close3(r->vertices[i].position(), pv[i], 1e-6f));
    }

    // ---- THE EDGES: 0..11 the box's; then the tool's walk tables:
    // 12 = the bottom of entity 0 (the -Y half, A_b -> S_b),
    // 13 = the bottom of entity 1 (the +Y half, S_b -> A_b),
    // 14/15 = their top copies, 16 = the seam (the vertical at J_0,
    // S_b -> S_t). NO antipodal vertical: the antipode is the split
    // vertex only (the pinned E's "+1 seam", not "+2").
    const std::pair<std::size_t, std::size_t> pe[17] = {
        {0, 1}, {1, 2}, {2, 3}, {3, 0},           // box bottom ring
        {4, 5}, {5, 6}, {6, 7}, {7, 4},           // box top ring
        {0, 4}, {1, 5}, {2, 6}, {3, 7},           // box verticals
        {10, 8},                                   // 12: -Y half bottom
        {8, 10},                                   // 13: +Y half bottom
        {11, 9},                                   // 14: -Y half top
        {9, 11},                                   // 15: +Y half top
        {8, 9},                                    // 16: the seam
    };
    for (std::size_t i = 0; i < 17; ++i) {
        CAPTURE(i);
        CHECK(r->edges[i].vertex(0) == pe[i].first);
        CHECK(r->edges[i].vertex(1) == pe[i].second);
    }

    // ---- THE FACES: 0..5 the box's own (0 bottom -Z, 1 top +Z,
    // 2..5 the sides — their loops carried over VERBATIM); 6 = the
    // wall. The pierced faces (0 and 1) GAIN one inner loop each.
    const auto& faces = r->solid.shell().faces();
    REQUIRE(faces.size() == 7);
    for (FaceId fid = 0; fid < 6; ++fid) {
        CAPTURE(fid);
        const auto& f = faces[fid];
        CHECK(f.loops().size() == (fid <= 1 ? 2 : 1));
        CHECK(std::string(f.surface()->type_name()) == "plane");
    }
    // The pierced faces' OUTER loops are the factory's own (4 Line2
    // p-curves, material-on-left, bit-identical to make_box's).
    for (const FaceId fid : {FaceId{0}, FaceId{1}}) {
        const auto& outer = faces[fid].loops()[0];
        REQUIRE(outer.coedges().size() == 4);
        REQUIRE(outer.is_outer());
        for (const auto& ce : outer.coedges()) {
            REQUIRE(ce.p_curve() != nullptr);
            CHECK(ce.p_curve()->kind == s::SketchEntity::Kind::Segment);
            CHECK(ce.p_curve()->segment.interior_left == true);
        }
        // The inner loop is NOT the outer (the flag says so, and the
        // positional H term already counted it).
        const auto& inner = faces[fid].loops()[1];
        CHECK_FALSE(inner.is_outer());
    }
}

TEST_CASE("G1-U4 guide: the wall - ONE CylinderSurface-family face, the sketch radius, region-on-right") {
    const GuideCase g = guide_case();
    auto r = b::exact_subtract(*g.a, *g.b, g.offset, b::kSubtractTolerance);
    REQUIRE(r != nullptr);

    // ONE cylindrical face: the hole's wall (the today world's faceted
    // extract of the same body is N planar strips — the contrast test
    // below).
    std::size_t cylinders = 0;
    for (std::size_t i = 0; i < r->face_count(); ++i) {
        const auto q = b::query_face(*r, static_cast<b::FaceId>(i));
        if (q.family == "cylinder") ++cylinders;
    }
    REQUIRE(cylinders == 1);

    const auto& wall = r->solid.shell().faces()[6];
    const auto* cyl = dynamic_cast<const b::CylinderZSurface*>(wall.surface());
    REQUIRE(cyl != nullptr);
    CHECK(close(cyl->radius(), 0.25f, 0.0f));
    CHECK(close2(cyl->axis_center(), {0.0f, 0.0f}, 0.0f));
    CHECK(close(cyl->z_base(), -1.0f, 0.0f));
    // The hole side: the frame rule forces u_sign = -1 (the outward
    // normal points INTO the hole, -radial).
    CHECK(cyl->u_sign() == -1.0f);

    // THE WALL'S LOOP — the ADR-0023 seam-wired walk on the window
    // [-2pi, 0] x [0, 2] (the patch BETWEEN the circles), six co-edges:
    //   [seam REV (down the right, u=0), bot_1 FWD, bot_0 FWD (west
    //    along the bottom), seam FWD (up the left, u=-2pi), top_0 REV,
    //    top_1 REV (east along the top)]
    // every Line2 region-on-RIGHT (interior_left = false) — the
    // convention this unit discovers and pins: the wall keeps the
    // prism's physical walk, re-framed by the flipped orientation.
    REQUIRE(wall.loops().size() == 1);
    const auto& ces = wall.loops()[0].coedges();
    REQUIRE(ces.size() == 6);
    const std::pair<EdgeId, bool> want[6] = {
        {16, true}, {13, false}, {12, false}, {16, false}, {14, true}, {15, true},
    };
    for (std::size_t i = 0; i < 6; ++i) {
        CAPTURE(i);
        CHECK(ces[i].edge() == want[i].first);
        CHECK(ces[i].reversed() == want[i].second);
        REQUIRE(ces[i].p_curve() != nullptr);
        CHECK(ces[i].p_curve()->kind == s::SketchEntity::Kind::Segment);
        CHECK(ces[i].p_curve()->segment.interior_left == false);
    }
    // The window walk's Line2 payloads (the domain rectangle):
    //   seam down (0,2)->(0,0); bot_1 (0,0)->(-pi,0); bot_0
    //   (-pi,0)->(-2pi,0); seam up (-2pi,0)->(-2pi,2); top_0
    //   (-2pi,2)->(-pi,2); top_1 (-pi,2)->(0,2).
    CHECK(close2(ces[0].p_curve()->segment.a, {0.0f, 2.0f}));
    CHECK(close2(ces[0].p_curve()->segment.b, {0.0f, 0.0f}));
    CHECK(close2(ces[1].p_curve()->segment.a, {0.0f, 0.0f}));
    CHECK(close2(ces[1].p_curve()->segment.b, {-kPi, 0.0f}));
    CHECK(close2(ces[2].p_curve()->segment.a, {-kPi, 0.0f}));
    CHECK(close2(ces[2].p_curve()->segment.b, {-kTwoPi, 0.0f}));
    CHECK(close2(ces[3].p_curve()->segment.a, {-kTwoPi, 0.0f}));
    CHECK(close2(ces[3].p_curve()->segment.b, {-kTwoPi, 2.0f}));
    CHECK(close2(ces[4].p_curve()->segment.a, {-kTwoPi, 2.0f}));
    CHECK(close2(ces[4].p_curve()->segment.b, {-kPi, 2.0f}));
    CHECK(close2(ces[5].p_curve()->segment.a, {-kPi, 2.0f}));
    CHECK(close2(ces[5].p_curve()->segment.b, {0.0f, 2.0f}));
    // The seam's TWO co-edges live in this ONE loop (the periodic
    // identification), opposite flags — pinned above by the table.
    std::size_t seam_uses = 0;
    for (const auto& ce : ces)
        if (ce.edge() == 16) ++seam_uses;
    CHECK(seam_uses == 2);
    // The p-curves chain endpoint-to-endpoint in the domain.
    CHECK(pcurves_chain(*r, wall, 0));
}

TEST_CASE("G1-U4 guide: the pierced faces' INNER LOOPS - the circle as Arc2 pairs, hole=true") {
    const GuideCase g = guide_case();
    auto r = b::exact_subtract(*g.a, *g.b, g.offset, b::kSubtractTolerance);
    REQUIRE(r != nullptr);

    const auto& faces = r->solid.shell().faces();

    // THE TOP FACE (+Z, the rotation domain): the inner loop walks the
    // canonical cycle [top_0 FWD (A_t -> S_t via -Y), top_1 FWD (S_t ->
    // A_t via +Y)] — CCW in the domain (the vocabulary's only sweep
    // direction), the region bit FLIPPED (hole = true: the material is
    // OUTSIDE the circle now).
    {
        const auto& inner = faces[1].loops()[1];
        REQUIRE(inner.coedges().size() == 2);
        const std::pair<EdgeId, bool> want[2] = {{14, false}, {15, false}};
        for (std::size_t i = 0; i < 2; ++i) {
            CAPTURE(i);
            CHECK(inner.coedges()[i].edge() == want[i].first);
            CHECK(inner.coedges()[i].reversed() == want[i].second);
        }
        // The Arc2 payloads: centre (0,0) in the top face's (u,v)
        // (its frame is (-x,-y) and the hole is at the world origin),
        // radius 0.25, hole = true, sweeps pi. top_0 starts at
        // dom(A_t) = (0.25, 0) (angle 0); top_1 at dom(S_t) =
        // (-0.25, 0) (angle pi).
        const auto* a0 = inner.coedges()[0].p_curve();
        const auto* a1 = inner.coedges()[1].p_curve();
        REQUIRE(a0 != nullptr);
        REQUIRE(a1 != nullptr);
        CHECK(a0->kind == s::SketchEntity::Kind::Arc);
        CHECK(a1->kind == s::SketchEntity::Kind::Arc);
        CHECK(close2(a0->arc.center, {0.0f, 0.0f}));
        CHECK(close(a0->arc.radius, 0.25f));
        CHECK(a0->arc.hole == true);
        CHECK(close(a0->arc.sweep, kPi));
        CHECK(close(std::cos(a0->arc.start_angle), 1.0f, 1e-5f));
        CHECK(close(std::sin(a0->arc.start_angle), 0.0f, 1e-5f));
        CHECK(close2(a1->arc.center, {0.0f, 0.0f}));
        CHECK(close(a1->arc.radius, 0.25f));
        CHECK(a1->arc.hole == true);
        CHECK(close(a1->arc.sweep, kPi));
        CHECK(close(std::cos(a1->arc.start_angle), -1.0f, 1e-5f));
        CHECK(pcurves_chain(*r, faces[1], 1));
    }

    // THE BOTTOM FACE (-Z, the mirror domain): the inner loop walks
    // the clockwise cycle [bot_0 REV (S_b -> A_b via -Y), bot_1 REV
    // (A_b -> S_b via +Y)] — CW in the world, CCW in the mirror
    // domain (the -Z cap's own walk parity, unit 3's precedent).
    {
        const auto& inner = faces[0].loops()[1];
        REQUIRE(inner.coedges().size() == 2);
        const std::pair<EdgeId, bool> want[2] = {{12, true}, {13, true}};
        for (std::size_t i = 0; i < 2; ++i) {
            CAPTURE(i);
            CHECK(inner.coedges()[i].edge() == want[i].first);
            CHECK(inner.coedges()[i].reversed() == want[i].second);
        }
        const auto* a0 = inner.coedges()[0].p_curve();
        const auto* a1 = inner.coedges()[1].p_curve();
        REQUIRE(a0 != nullptr);
        REQUIRE(a1 != nullptr);
        CHECK(a0->kind == s::SketchEntity::Kind::Arc);
        CHECK(a1->kind == s::SketchEntity::Kind::Arc);
        CHECK(close2(a0->arc.center, {0.0f, 0.0f}));
        CHECK(close(a0->arc.radius, 0.25f));
        CHECK(a0->arc.hole == true);
        CHECK(close(a0->arc.sweep, kPi));
        CHECK(a1->arc.hole == true);
        CHECK(close(a1->arc.sweep, kPi));
        CHECK(pcurves_chain(*r, faces[0], 1));
    }

    // The wall's arcs and the inner loops' arcs are the SAME edges
    // with OPPOSITE orientation flags (the radial XOR, verified
    // structurally by check_body_invariants — here by inspection of
    // the tables above: the wall used 14/15 REV and 12/13 FWD, the
    // pierced faces the opposite).
}

TEST_CASE("G1-U4 guide: the exact wall vs the faceted today - ONE cylinder vs N strips") {
    // The same body through the SDF side's faceted bridge: N planar
    // strips, approximated, no p-curves. Both honest; only one exact.
    CAD_0::scene::Scene sc;
    const auto bid = add_box_node(sc, "Box", {1.0f, 1.0f, 1.0f});
    const auto sid = sc.add_sketch_node("S", circle_chains());
    const auto eid = add_extrude(sc, "Extrude", sid, 3.0f, +1);
    const auto did = add_subtract(sc, "Sub", bid, eid, {0.0f, 0.0f, -1.5f});
    const auto* n = sc.get_node(did);
    REQUIRE(n != nullptr);

    CAD_0::bridge::SdfToBrepOptions opts;
    opts.fit_primitives = false;
    opts.march_resolution = 24;
    auto faceted = CAD_0::bridge::sdf_to_brep_faceted(n->body, opts);
    REQUIRE(faceted != nullptr);
    CHECK(faceted->face_count() > 7);
    for (std::size_t i = 0; i < faceted->face_count(); ++i) {
        const auto q = b::query_face(*faceted, static_cast<b::FaceId>(i));
        CHECK(q.family == "plane");
        CHECK_FALSE(q.exact);
        for (const auto& loop : q.loops)
            for (const auto& ce : loop.coedges)
                CHECK(ce.kind == CAD_0::brep::PCurveInfo::Kind::None);
    }
}

// ---------------------------------------------------------------------------
// FAMILY 3 — THE FORMULAS: every tool profile the extrude authors
// ---------------------------------------------------------------------------
TEST_CASE("G1-U4 formulas: the square prism tool (n=4) - V=16 E=24 F=10 H=2") {
    // A box target (V_A=8, E_A=12, F_A=6) minus the square prism:
    // V = 8 + 2n, E = 12 + 3n, F = 6 + n, H = 2, chi = 0 for every n.
    auto a = b::make_box({1.0f, 1.0f, 1.0f});
    auto tool = b::make_prism(square_chains(), 3.0f, +1);
    auto r = b::exact_subtract(*a, *tool, {0.0f, 0.0f, -1.5f},
                                b::kSubtractTolerance);
    REQUIRE(r != nullptr);
    check_body_invariants(*r, 16, 24, 10, 2, 0);
    check_exact_provenance(*r);

    // The new vertices: the square's joints J_0..J_3 at z_bot/top.
    // (The canonical walk = the +Z cap's = the authored CCW cycle:
    // J_0=(0.3,-0.3), J_1=(0.3,0.3), J_2=(-0.3,0.3), J_3=(-0.3,-0.3).)
    const Vec3f pv8[8] = {
        {0.3f, -0.3f, -1.0f}, {0.3f, -0.3f, 1.0f},   // J_0 bot, top
        {0.3f, 0.3f, -1.0f}, {0.3f, 0.3f, 1.0f},     // J_1
        {-0.3f, 0.3f, -1.0f}, {-0.3f, 0.3f, 1.0f},   // J_2
        {-0.3f, -0.3f, -1.0f}, {-0.3f, -0.3f, 1.0f}, // J_3
    };
    for (std::size_t i = 0; i < 8; ++i) {
        CAPTURE(i);
        CHECK(close3(r->vertices[8 + i].position(), pv8[i], 1e-6f));
    }

    // The four walls are PLANES (the segment entities), and every
    // wall's Line2 walk is region-on-right.
    for (FaceId fid = 6; fid < 10; ++fid) {
        const auto& f = r->solid.shell().faces()[fid];
        CHECK(std::string(f.surface()->type_name()) == "plane");
        REQUIRE(f.loops().size() == 1);
        REQUIRE(f.loops()[0].coedges().size() == 4);
        for (const auto& ce : f.loops()[0].coedges()) {
            REQUIRE(ce.p_curve() != nullptr);
            CHECK(ce.p_curve()->kind == s::SketchEntity::Kind::Segment);
            CHECK(ce.p_curve()->segment.interior_left == false);
        }
        CHECK(pcurves_chain(*r, f, 0));
    }

    // The pierced faces' inner loops: 4 Line2 with the FLIPPED
    // authored bits (the material is outside the square now).
    for (const FaceId fid : {FaceId{0}, FaceId{1}}) {
        const auto& inner = r->solid.shell().faces()[fid].loops()[1];
        REQUIRE(inner.coedges().size() == 4);
        for (const auto& ce : inner.coedges()) {
            REQUIRE(ce.p_curve() != nullptr);
            CHECK(ce.p_curve()->kind == s::SketchEntity::Kind::Segment);
            CHECK(ce.p_curve()->segment.interior_left == false);
        }
        CHECK(pcurves_chain(*r, r->solid.shell().faces()[fid], 1));
    }
}

TEST_CASE("G1-U4 formulas: the sector tool (n=3, segments + arc) - the mixed profile") {
    auto a = b::make_box({1.0f, 1.0f, 1.0f});
    auto tool = b::make_prism(sector_chains(), 3.0f, +1);
    auto r = b::exact_subtract(*a, *tool, {0.0f, 0.0f, -1.5f},
                                b::kSubtractTolerance);
    REQUIRE(r != nullptr);
    // V = 8 + 6, E = 12 + 9, F = 6 + 3, H = 2, chi = 0.
    check_body_invariants(*r, 14, 21, 9, 2, 0);
    check_exact_provenance(*r);

    // The arc's wall: ONE cylindrical patch among the 3 walls, its
    // window on the REVERSED u (the cavity side), region-on-right.
    const auto& faces = r->solid.shell().faces();
    std::size_t cyl_walls = 0, plane_walls = 0;
    for (FaceId fid = 6; fid < 9; ++fid) {
        const auto& f = faces[fid];
        const auto t = std::string(f.surface()->type_name());
        if (t == "cylinder") {
            ++cyl_walls;
            const auto* cyl =
                dynamic_cast<const b::CylinderZSurface*>(f.surface());
            REQUIRE(cyl != nullptr);
            CHECK(close(cyl->radius(), 0.4f, 0.0f));
            CHECK(cyl->u_sign() == -1.0f);
            CHECK(close(cyl->z_base(), -1.0f, 0.0f));
            REQUIRE(f.loops().size() == 1);
            for (const auto& ce : f.loops()[0].coedges()) {
                REQUIRE(ce.p_curve() != nullptr);
                CHECK(ce.p_curve()->kind == s::SketchEntity::Kind::Segment);
                CHECK(ce.p_curve()->segment.interior_left == false);
            }
            CHECK(pcurves_chain(*r, f, 0));
        } else if (t == "plane") {
            ++plane_walls;
        }
    }
    CHECK(cyl_walls == 1);
    CHECK(plane_walls == 2);

    // The pierced faces' inner loops carry the sector's entities:
    // 2 Line2 + 1 Arc2 (hole = true — the flipped fillet bit).
    for (const FaceId fid : {FaceId{0}, FaceId{1}}) {
        const auto& inner = faces[fid].loops()[1];
        REQUIRE(inner.coedges().size() == 3);
        std::size_t arcs = 0, lines = 0;
        for (const auto& ce : inner.coedges()) {
            REQUIRE(ce.p_curve() != nullptr);
            if (ce.p_curve()->kind == s::SketchEntity::Kind::Arc) {
                ++arcs;
                CHECK(close(ce.p_curve()->arc.radius, 0.4f));
                CHECK(ce.p_curve()->arc.hole == true);
                CHECK(close(ce.p_curve()->arc.sweep, 0.55f * kPi));
            } else {
                CHECK(ce.p_curve()->kind == s::SketchEntity::Kind::Segment);
                CHECK(ce.p_curve()->segment.interior_left == false);
                ++lines;
            }
        }
        CHECK(arcs == 1);
        CHECK(lines == 2);
        CHECK(pcurves_chain(*r, faces[fid], 1));
    }
}

TEST_CASE("G1-U4 formulas: a PRISM TARGET - the same shape through the extrude's own body") {
    // A = the extruded rectangle (a planar prism, V_A=8/E_A=12/F_A=6 —
    // structurally the box's tables): the same guide counts.
    auto a = b::make_prism(square_chains({-1.0f, -1.0f}, {1.0f, 1.0f}),
                           2.0f, +1);
    auto tool = b::make_prism(circle_chains(), 3.0f, +1);
    auto r = b::exact_subtract(*a, *tool, {0.0f, 0.0f, -0.5f},
                                b::kSubtractTolerance);
    REQUIRE(r != nullptr);
    // The prism target spans z in [0, 2]; the offset tool z in
    // [-0.5, 2.5]: a strict through-hole.
    check_body_invariants(*r, 12, 17, 7, 2, 0);
    check_exact_provenance(*r);
}

// ---------------------------------------------------------------------------
// FAMILY 4 — THE DEGENERATE CLASSES, each with its PINNED DISPOSITION
// (D7.4: pinned now, not discovered after)
// ---------------------------------------------------------------------------
TEST_CASE("G1-U4 degenerate: the coplanar cap flush - ABSORBED, the same counts") {
    // The tool's bottom cap exactly ON the box's bottom face (the
    // offset lands z_B'_lo at z_bot = -1): the ADR-0023/0024 coplanar
    // dedup rule — keep A's face, absorb the flush cap. The counts
    // are the guide's own (no spurious face, no duplicate).
    auto a = b::make_box({1.0f, 1.0f, 1.0f});
    auto tool = b::make_prism(circle_chains(), 3.0f, +1);  // z in [0,3]
    // z_B'_lo = 0 + oz = -1 -> oz = -1; z_B'_hi = 2 > z_top = 1.
    auto r = b::exact_subtract(*a, *tool, {0.0f, 0.0f, -1.0f},
                                b::kSubtractTolerance);
    REQUIRE(r != nullptr);
    check_body_invariants(*r, 12, 17, 7, 2, 0);
    check_exact_provenance(*r);

    // The coplanar disposition is about NO spurious geometry: the
    // wall spans exactly [z_bot, z_top] and the bottom face carries
    // the inner loop at z = -1 (absorbed, not duplicated).
    CHECK(close(r->vertices[8].position().z, -1.0f, 0.0f));   // S_b
    CHECK(close(r->vertices[9].position().z, 1.0f, 0.0f));    // S_t
    REQUIRE(r->solid.shell().faces()[0].loops().size() == 2);
}

TEST_CASE("G1-U4 degenerate: the tangential wall from inside - the DETERMINISTIC refusal") {
    // The hole's wall tangent to the box's +X face from inside: the
    // disc at (0.75, 0), r = 0.25, the footprint edge x = 1. The
    // pierced face would pinch at the tangency (a non-manifold
    // vertex): the tie-break is deterministic — canonical-key
    // ordering over the edges, never input order — and the chosen
    // disposition is the refusal. The golden pins the message's own
    // words: the tangency named, the remedy named.
    auto a = b::make_box({1.0f, 1.0f, 1.0f});
    auto tool = b::make_prism(circle_chains({0.75f, 0.0f}, 0.25f), 3.0f, +1);
    bool threw = false;
    std::string msg;
    try {
        auto r = b::exact_subtract(*a, *tool, {0.0f, 0.0f, -1.5f},
                                    b::kSubtractTolerance);
        (void)r;
    } catch (const std::invalid_argument& e) {
        threw = true;
        msg = e.what();
    }
    REQUIRE(threw);
    CHECK(msg.find("TANGENT") != std::string::npos);
    CHECK(msg.find("Remedy") != std::string::npos);
    // Determinism: the same input, the same refusal (bit-identical).
    bool threw2 = false;
    std::string msg2;
    try {
        auto r = b::exact_subtract(*a, *tool, {0.0f, 0.0f, -1.5f},
                                    b::kSubtractTolerance);
        (void)r;
    } catch (const std::invalid_argument& e) {
        threw2 = true;
        msg2 = e.what();
    }
    CHECK(threw2);
    CHECK(msg == msg2);
}

TEST_CASE("G1-U4 degenerate: the crossing circle - the actionable refusal") {
    // The disc at (0.9, 0), r = 0.35: the circle crosses the +X face
    // boundary (0.9 + 0.35 > 1 with margin) — the non-annular pierced
    // face is honest G2 topology, not G1.
    auto a = b::make_box({1.0f, 1.0f, 1.0f});
    auto tool = b::make_prism(circle_chains({0.9f, 0.0f}, 0.35f), 3.0f, +1);
    bool threw = false;
    std::string msg;
    try {
        auto r = b::exact_subtract(*a, *tool, {0.0f, 0.0f, -1.5f},
                                    b::kSubtractTolerance);
        (void)r;
    } catch (const std::invalid_argument& e) {
        threw = true;
        msg = e.what();
    }
    REQUIRE(threw);
    CHECK(msg.find("CROSSES") != std::string::npos);
    CHECK(msg.find("Remedy") != std::string::npos);
}

TEST_CASE("G1-U4 degenerate: the blind hole - the actionable refusal") {
    // The tool ends INSIDE A (z_B'_hi = 0.5 < z_top = 1): the
    // internal cap face class is G2's.
    auto a = b::make_box({1.0f, 1.0f, 1.0f});
    auto tool = b::make_prism(circle_chains(), 1.6f, +1);  // z in [0,1.6]
    // offset -1.2 -> z in [-1.2, 0.4]: the top cap is inside.
    bool threw = false;
    std::string msg;
    try {
        auto r = b::exact_subtract(*a, *tool, {0.0f, 0.0f, -1.2f},
                                    b::kSubtractTolerance);
        (void)r;
    } catch (const std::invalid_argument& e) {
        threw = true;
        msg = e.what();
    }
    REQUIRE(threw);
    CHECK(msg.find("BLIND") != std::string::npos);
    CHECK(msg.find("Remedy") != std::string::npos);
}

TEST_CASE("G1-U4 degenerate: the oblique target (a tetrahedron) - the angle named") {
    // A tetrahedron's faces are neither perpendicular nor parallel to
    // Z: the oblique SSI is the sinusoid p-curve, G3's table — the
    // refusal names the angle and the remedy (reorient, or accept the
    // faceted path).
    auto a = b::make_tetrahedron();
    auto tool = b::make_prism(circle_chains(), 3.0f, +1);
    bool threw = false;
    std::string msg;
    try {
        auto r = b::exact_subtract(*a, *tool, {0.0f, 0.0f, -1.5f},
                                    b::kSubtractTolerance);
        (void)r;
    } catch (const std::invalid_argument& e) {
        threw = true;
        msg = e.what();
    }
    REQUIRE(threw);
    CHECK(msg.find("oblique") != std::string::npos);
    CHECK(msg.find("Remedy") != std::string::npos);
}

TEST_CASE("G1-U4 degenerate: the Y-axis cylinder tool (the v0.3 chain's shape) - the structural refusal") {
    // The factory primitive cylinder's axis is Y: not a Z-axis prism
    // of the D6 class (its caps carry +/-Y normals). The v0.3 golden's
    // own shape — it STAYS on the faceted path, the refusal recorded.
    auto a = b::make_box({1.0f, 1.0f, 1.0f});
    auto tool = b::make_cylinder(0.25f, 3.0f);
    bool threw = false;
    std::string msg;
    try {
        auto r = b::exact_subtract(*a, *tool, {0.0f, 0.0f, 0.0f},
                                    b::kSubtractTolerance);
        (void)r;
    } catch (const std::invalid_argument& e) {
        threw = true;
        msg = e.what();
    }
    REQUIRE(threw);
    CHECK(msg.find("prism") != std::string::npos);
    CHECK(msg.find("Remedy") != std::string::npos);
}

TEST_CASE("G1-U4 degenerate: the swallow - the refusal") {
    // The disc r = 3 at the origin swallows the footprint [-1,1]^2:
    // every target face would be removed — the empty solid. Refused
    // with the remedy (the node's SDF path stands).
    auto a = b::make_box({1.0f, 1.0f, 1.0f});
    auto tool = b::make_prism(circle_chains({0.0f, 0.0f}, 3.0f), 3.0f, +1);
    bool threw = false;
    std::string msg;
    try {
        auto r = b::exact_subtract(*a, *tool, {0.0f, 0.0f, -1.5f},
                                    b::kSubtractTolerance);
        (void)r;
    } catch (const std::invalid_argument& e) {
        threw = true;
        msg = e.what();
    }
    REQUIRE(threw);
    CHECK(msg.find("SWALLOW") != std::string::npos);
}

TEST_CASE("G1-U4 degenerate: the miss - the EXACT COPY (nothing removed)") {
    // The tool beside the target (the disc at (5, 5), no overlap):
    // the mathematically true exact result is A itself — the copy,
    // provenance exact, counts and chi preserved.
    auto a = b::make_box({1.0f, 1.0f, 1.0f});
    auto tool = b::make_prism(circle_chains({5.0f, 5.0f}, 0.25f), 3.0f, +1);
    auto r = b::exact_subtract(*a, *tool, {0.0f, 0.0f, -1.5f},
                                b::kSubtractTolerance);
    REQUIRE(r != nullptr);
    check_body_invariants(*r, 8, 12, 6, 0, 2);
    check_exact_provenance(*r);
    for (std::size_t i = 0; i < 8; ++i)
        CHECK(close3(r->vertices[i].position(),
                     a->vertices[i].position(), 0.0f));
    // The copy is a COPY: the query answers match A's own.
    CHECK(query_digest(*r) == query_digest(*a));

    // The z-side miss (the tool entirely above): the copy too.
    auto r2 = b::exact_subtract(*a, *tool, {0.0f, 0.0f, 10.0f},
                                 b::kSubtractTolerance);
    REQUIRE(r2 != nullptr);
    check_body_invariants(*r2, 8, 12, 6, 0, 2);
}

TEST_CASE("G1-U4 degenerate: the non-convex L tool - the refusal") {
    auto a = b::make_box({1.0f, 1.0f, 1.0f});
    auto tool = ell_prism_body();
    bool threw = false;
    std::string msg;
    try {
        auto r = b::exact_subtract(*a, *tool, {0.0f, 0.0f, -1.5f},
                                    b::kSubtractTolerance);
        (void)r;
    } catch (const std::invalid_argument& e) {
        threw = true;
        msg = e.what();
    }
    REQUIRE(threw);
    CHECK(msg.find("CONVEX") != std::string::npos);
}

TEST_CASE("G1-U4 degenerate: the ring tool (the caps' inner loops) - the refusal") {
    auto a = b::make_box({1.0f, 1.0f, 1.0f});
    auto tool = b::make_prism(ring_tool_chains(), 3.0f, +1);
    bool threw = false;
    std::string msg;
    try {
        auto r = b::exact_subtract(*a, *tool, {0.0f, 0.0f, -1.5f},
                                    b::kSubtractTolerance);
        (void)r;
    } catch (const std::invalid_argument& e) {
        threw = true;
        msg = e.what();
    }
    REQUIRE(threw);
    CHECK(msg.find("CONVEX") != std::string::npos);
}

TEST_CASE("G1-U4 degenerate: a non-exact operand - the refusal (the honesty chain)") {
    // The worst-of propagation has a harder edge: the exact subtract
    // CONSUMES exact operands (the unit-1 finding — the booleans
    // consume exactness). An approximated tool refuses loudly; the
    // record is the thing under test (the machinery honours it).
    auto a = b::make_box({1.0f, 1.0f, 1.0f});
    auto tool = b::make_prism(circle_chains(), 3.0f, +1);
    const auto approx = CAD_0::brep::ProvenanceInfo::approximated(1e-3);
    for (auto& v : tool->vertices) v.set_provenance(approx);
    bool threw = false;
    std::string msg;
    try {
        auto r = b::exact_subtract(*a, *tool, {0.0f, 0.0f, -1.5f},
                                    b::kSubtractTolerance);
        (void)r;
    } catch (const std::invalid_argument& e) {
        threw = true;
        msg = e.what();
    }
    REQUIRE(threw);
    CHECK(msg.find("exact") != std::string::npos);
}

TEST_CASE("G1-U4 degenerate: the sphere target - the curved-face refusal") {
    // A sphere is convex but not PLANAR-faced: out of the G1 class.
    auto a = b::make_sphere(1.0f);
    auto tool = b::make_prism(circle_chains(), 3.0f, +1);
    bool threw = false;
    try {
        auto r = b::exact_subtract(*a, *tool, {0.0f, 0.0f, -1.5f},
                                    b::kSubtractTolerance);
        (void)r;
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    REQUIRE(threw);
}

// ---------------------------------------------------------------------------
// FAMILY 5 — THE LANDING (ADR-0031 D6 / ADR-0032 D7): the subtract
// node owns BOTH materializations; the goldens; determinism
// ---------------------------------------------------------------------------
TEST_CASE("G1-U4 landing: the subtract node owns BOTH materializations, bridge tolerance 0") {
    CAD_0::scene::Scene sc;
    const auto bid = add_box_node(sc, "Box", {1.0f, 1.0f, 1.0f});
    const auto sid = sc.add_sketch_node("S", circle_chains());
    const auto eid = add_extrude(sc, "Extrude", sid, 3.0f, +1);
    const auto did = add_subtract(sc, "Sub", bid, eid, {0.0f, 0.0f, -1.5f});

    const auto* n = sc.get_node(did);
    REQUIRE(n != nullptr);
    REQUIRE(n->hybrid != nullptr);
    CHECK(n->hybrid->brep != nullptr);
    CHECK(n->hybrid->sdf != nullptr);
    CHECK(n->hybrid->bridge_tolerance.linear == 0.0);
    CHECK(n->hybrid->bridge_tolerance.angular == 0.0);
    CHECK(n->hybrid->role == CAD_0::geometry::HybridRole::BRepWithSDFShell);
    CHECK(n->hybrid_refusal.empty());

    // The exact side: the guide table.
    check_body_invariants(*n->hybrid->brep, 12, 17, 7, 2, 0);
    check_exact_provenance(*n->hybrid->brep);

    // The SDF twin: the same recipe's materialization (the sdf
    // subtract of the operands — the display body's own field).
    REQUIRE(sc.get_node(eid) != nullptr);
    s::SDFBody fresh = s::sdf_subtract(
        sc.get_node(bid)->body,
        s::sdf_translate(sc.get_node(eid)->body, {0.0f, 0.0f, -1.5f}));
    for (const Vec3f p : {Vec3f{0.0f, 0.0f, 0.0f}, Vec3f{0.9f, 0.0f, 0.0f},
                          Vec3f{0.0f, 0.0f, 0.9f}, Vec3f{0.3f, 0.3f, 0.3f}}) {
        CHECK(n->hybrid->sdf->sample(p).value == fresh.sample(p).value);
        CHECK(n->body.sample(p).value == fresh.sample(p).value);
    }

    // The DISPLAY path is UNCHANGED: the node's body IS the SDF
    // subtract the world already knew — the exact side landed BESIDE
    // it, never under it.
    CHECK(sdf_digest(n->body) == sdf_digest(fresh));
}

TEST_CASE("G1-U4 landing: the out-of-class target keeps its SDF, the refusal VISIBLE") {
    // A sphere target: convex but NOT planar-faced — out of the G1
    // class. The refusal is recorded on the node (the query surface
    // reads it), the SDF display path stands, the body is the sdf
    // subtract the world already knew. NO silent downgrade, NO
    // regression: the node's behaviour is exactly what it was, plus
    // the honest reason.
    CAD_0::scene::Scene sc;
    const auto tid = sc.add_node("Sphere", s::make_sphere(1.0f));

    const auto sid = sc.add_sketch_node("S", circle_chains());
    const auto eid = add_extrude(sc, "Extrude", sid, 3.0f, +1);
    const auto did = add_subtract(sc, "Sub", tid, eid, {0.0f, 0.0f, -1.5f});
    const auto* n = sc.get_node(did);
    REQUIRE(n != nullptr);
    CHECK(n->hybrid == nullptr);
    REQUIRE_FALSE(n->hybrid_refusal.empty());
    CHECK(n->hybrid_refusal.find("non-planar") != std::string::npos);

    // The SDF side materialized regardless (the faceted path stands).
    CHECK_FALSE(n->operation_pending);
    CHECK_FALSE(n->operand_missing);
    // The body is the sdf subtract (not the sphere's own field).
    CHECK(n->body.sample({0.0f, 0.0f, 0.0f}).value >
          sc.get_node(tid)->body.sample({0.0f, 0.0f, 0.0f}).value);
}

TEST_CASE("G1-U4 landing: the v0.4 golden materializes the EXACT subtract NOW") {
    // The parametric-hole document (sketch -> extrude -> subtract):
    // in class (the Z-axis circle prism through the box), so on load
    // the subtract node's hybrid materializes — 12/17/7/2, exact/0.
    // The faceted superstate retires WITH THE REASON DOCUMENTED (the
    // ADR's landing record); the SDF/display goldens are untouched
    // (they pin the node's body, which did not move).
    const std::filesystem::path golden =
        std::filesystem::path(CAD_0_SOURCE_DIR) / "tests" / "golden" /
        "native_v0_4_0.CAD_0";
    REQUIRE(std::filesystem::exists(golden));

    auto doc = CAD_0::io::native::load_file(golden);
    CAD_0::scene::Scene sc;
    CAD_0::io::native::install_document(sc, std::move(doc));

    const auto* sub = sc.get_node(4);
    REQUIRE(sub != nullptr);
    REQUIRE(sub->operation.has_value());
    CHECK(sub->operation->type == CAD_0::scene::OperationType::Subtract);
    REQUIRE(sub->hybrid != nullptr);
    CHECK(sub->hybrid_refusal.empty());
    check_body_invariants(*sub->hybrid->brep, 12, 17, 7, 2, 0);
    check_exact_provenance(*sub->hybrid->brep);

    // The tool's own hybrid (the extrude) is untouched by the landing.
    const auto* ex = sc.get_node(2);
    REQUIRE(ex != nullptr);
    REQUIRE(ex->hybrid != nullptr);
    check_body_invariants(*ex->hybrid->brep, 4, 5, 3, 0, 2);

    // The faceted extract of the SAME body still exists as the
    // display-side diagnostic (unchanged pins, unchanged path): its
    // report reads chi = 0, genus 1 — the SAME topology, coarser
    // geometry (the honest agreement the ADR pins).
    CAD_0::bridge::SdfToBrepOptions opts;
    opts.march_resolution = 48;
    const auto rep = CAD_0::bridge::sdf_to_brep_report(sub->body, opts);
    CHECK(rep.chi == 0);
    CHECK(rep.orientable);
    CHECK(rep.genus_derived);
    CHECK(rep.genus == 1);
}

TEST_CASE("G1-U4 landing: the v0.3 golden STAYS faceted - the Y-axis tool's refusal") {
    // The pierced-box document's tool is the PARAMETRIC CYLINDER
    // (the factory primitive, axis Y) — not a Z-axis prism of the D6
    // class. The exact subtract refuses; the faceted superstate
    // REMAINS with the reason recorded (the order's own "col motivo
    // scritto" applied honestly: the class is the class).
    const std::filesystem::path golden =
        std::filesystem::path(CAD_0_SOURCE_DIR) / "tests" / "golden" /
        "native_v0_3_0.CAD_0";
    REQUIRE(std::filesystem::exists(golden));

    auto doc = CAD_0::io::native::load_file(golden);
    CAD_0::scene::Scene sc;
    CAD_0::io::native::install_document(sc, std::move(doc));

    const auto* sub = sc.get_node(3);
    REQUIRE(sub != nullptr);
    REQUIRE(sub->operation.has_value());
    CHECK(sub->hybrid == nullptr);
    REQUIRE_FALSE(sub->hybrid_refusal.empty());
    CHECK(sub->hybrid_refusal.find("prism") != std::string::npos);

    // The SDF side is exactly what it always was (the pins in
    // test_native still hold; the display path never moved).
    CAD_0::bridge::SdfToBrepOptions opts;
    opts.march_resolution = 48;
    const auto rep = CAD_0::bridge::sdf_to_brep_report(sub->body, opts);
    CHECK(rep.fit == "faceted");
    CHECK(rep.chi == 0);
    CHECK(rep.genus == 1);
}

TEST_CASE("G1-U4 determinism: the radius edit regenerates BOTH materializations bit-identically") {
    CAD_0::scene::Scene sc;
    const auto bid = add_box_node(sc, "Box", {1.0f, 1.0f, 1.0f});
    const auto sid = sc.add_sketch_node("S", circle_chains());
    const auto eid = add_extrude(sc, "Extrude", sid, 3.0f, +1);
    const auto did = add_subtract(sc, "Sub", bid, eid, {0.0f, 0.0f, -1.5f});

    const auto* n = sc.get_node(did);
    REQUIRE(n != nullptr);
    REQUIRE(n->hybrid != nullptr);
    const std::string q1 = query_digest(*n->hybrid->brep);
    const std::string f1 = sdf_digest(n->body);

    // Edit the sketch's radius: 0.25 -> 0.4 (still strictly inside).
    auto chains = sc.get_node(sid)->sketch;
    chains[0].entities[0].circle.radius = 0.4f;
    sc.sketch_set_entity(sid, 0, 0, chains[0].entities[0]);
    auto res = CAD_0::scene::rebuild_operations(sc);
    REQUIRE(res.rebuilt == 2);  // the extrude and the subtract

    REQUIRE(n->hybrid != nullptr);
    const std::string q2 = query_digest(*n->hybrid->brep);
    const std::string f2 = sdf_digest(n->body);
    CHECK(q2 != q1);   // the geometry DID change...
    CHECK(f2 != f1);
    // ...and the re-edit back regenerates BOTH bit-identically.
    auto chains2 = sc.get_node(sid)->sketch;
    chains2[0].entities[0].circle.radius = 0.25f;
    sc.sketch_set_entity(sid, 0, 0, chains2[0].entities[0]);
    auto res2 = CAD_0::scene::rebuild_operations(sc);
    REQUIRE(res2.rebuilt == 2);
    REQUIRE(n->hybrid != nullptr);
    CHECK(query_digest(*n->hybrid->brep) == q1);
    CHECK(sdf_digest(n->body) == f1);

    // The exact body after the full cycle still passes the contract.
    check_body_invariants(*n->hybrid->brep, 12, 17, 7, 2, 0);
}

TEST_CASE("G1-U4 determinism: the offset edit - the same bit-identity") {
    CAD_0::scene::Scene sc;
    const auto bid = add_box_node(sc, "Box", {1.0f, 1.0f, 1.0f});
    const auto sid = sc.add_sketch_node("S", circle_chains({0.2f, 0.1f}, 0.25f));
    const auto eid = add_extrude(sc, "Extrude", sid, 3.0f, +1);
    const auto did = add_subtract(sc, "Sub", bid, eid, {0.0f, 0.0f, -1.5f});
    const auto* n = sc.get_node(did);
    REQUIRE(n != nullptr);
    REQUIRE(n->hybrid != nullptr);
    const std::string q1 = query_digest(*n->hybrid->brep);
    const std::string f1 = sdf_digest(n->body);

    // Move the hole (still strictly inside), then back — the recipe
    // edit + the DAG pass (the set_operation_offset surface's own
    // path, driven directly).
    auto* mut = sc.get_node_mut(did);
    REQUIRE(mut != nullptr);
    mut->operation->offset_x = 0.3f;
    mut->operation->offset_y = -0.2f;
    auto res = CAD_0::scene::rebuild_operations(sc);
    REQUIRE(res.rebuilt == 2);  // the extrude AND the subtract
    REQUIRE(sc.get_node(did)->hybrid != nullptr);
    CHECK(query_digest(*sc.get_node(did)->hybrid->brep) != q1);
    auto* mut2 = sc.get_node_mut(did);
    REQUIRE(mut2 != nullptr);
    mut2->operation->offset_x = 0.0f;
    mut2->operation->offset_y = 0.0f;
    auto res2 = CAD_0::scene::rebuild_operations(sc);
    REQUIRE(res2.rebuilt == 2);
    REQUIRE(sc.get_node(did)->hybrid != nullptr);
    CHECK(query_digest(*sc.get_node(did)->hybrid->brep) == q1);
    CHECK(sdf_digest(sc.get_node(did)->body) == f1);
}

TEST_CASE("G1-U4 landing: the honesty chain UPDATED - the subtract now says exact") {
    // Unit 3 pinned "the subtracted body says approximated" (its
    // exact chain ended at the operands THEN — unit 4 was not built).
    // The chain now extends: the in-class subtract's OWN materialization
    // is exact. The BRIDGE path (the SDF-side extract) is unchanged —
    // still faceted, still approximated: two materializations, both
    // honest, never confused.
    CAD_0::scene::Scene sc;
    const auto bid = add_box_node(sc, "Box", {1.0f, 1.0f, 1.0f});
    const auto sid = sc.add_sketch_node("S", circle_chains());
    const auto eid = add_extrude(sc, "Extrude", sid, 3.0f, +1);
    const auto did = add_subtract(sc, "Sub", bid, eid, {0.0f, 0.0f, -1.5f});
    const auto* n = sc.get_node(did);
    REQUIRE(n != nullptr);
    REQUIRE(n->hybrid != nullptr);
    check_exact_provenance(*n->hybrid->brep);

    CAD_0::bridge::SdfToBrepOptions opts;
    opts.fit_primitives = false;
    opts.march_resolution = 20;
    auto faceted = CAD_0::bridge::sdf_to_brep_faceted(n->body, opts);
    REQUIRE(faceted != nullptr);
    std::size_t approx = 0, with_pcurve = 0;
    for (const auto& fi : b::query_all_faces(*faceted)) {
        if (!fi.exact) ++approx;
        for (const auto& loop : fi.loops)
            for (const auto& ce : loop.coedges)
                if (ce.kind != CAD_0::brep::PCurveInfo::Kind::None) ++with_pcurve;
    }
    CHECK(approx == faceted->face_count());
    CHECK(with_pcurve == 0);
}
