// tests/unit/brep/test_g2_u3_m6bis_the_trim.cpp
//
// G2-u3 m6-CORRETTIVA-BIS — THE SIX CROSSED PLANES, THE CURE WITHOUT
// ANYTHING NEW (the click-test's second verdict, 2026-11-07: NON
// PASSATO — the fillet on the box showed six crossed planes in the
// viewport, not the raccordo; the frame fluid, the batching's victory
// intact; the kernel innocent on the tree, CI green, the m5 report
// doors holding).
//
// THE DIAGNOSIS this file's counterproof rides (headless, the
// scene/report ports, no GL): brep::tessellate_face NEVER READ the
// face's boundary — the Plane family sampled the hardcoded
// [-10,10]^2 parametric window, the closed families their full
// sweeps. On the box guide case at the display resolution 48 the
// display mesh carried 16,807 vertices / 32,256 triangles (exactly
// the fixed per-face grid 7 x (49^2 / 2*48^2), zero trimming), of
// which 16,272 vertices (96.8%) and 31,000 triangles lay OUTSIDE the
// filleted body — the planes reaching +-10 in a [-1,1]^3 world, the
// flank sampled over the full 360 degrees for its quarter. The
// buffer assembly the batch walks was measured SOUND (the index
// bounds, the ranges, the contiguity) — the birthplace is the
// tessellation itself, and it is named here.
//
// THE CURE pinned by this file: the face's OWN window — the
// parametric box of its boundary's p-curves — and the trim by the
// boundary's own shape: the even-odd region test over the loops
// sampled as polylines (outer loop in, inner loops out; the seam
// jumps closed with straight segments — the sphere's meridian pair
// closes as the window rectangle in the unrolled plane). The region
// BITS (interior_left / hole) are NOT read here: they are the mass
// machinery's vocabulary — on the blind hole's internal cap the
// inverted bit declares the material side flipped for the VOLUME
// (subtract D1.1), and the same hole=true that means "outside" on
// the pierced top face's rim means the disc on the cap — the bits
// answer "where the material lies", the SHAPES answer "where the
// patch is", and the display tessellates the patch.
//
// THE COUNTERPROOF, naive red on the uncured tip (the same numbers
// the diagnosis measured, quoted in the register's
// m6-correttiva-bis row):
//   * the phantom census: NO display vertex beyond the body's own
//     world (the box grown by one grid cell) — uncured: +-10, RED;
//   * the flank's sweep: the quarter [0, pi/2] at the (+X,+Y)
//     corner, never the full turn — uncured: 360 degrees, RED;
//   * the rim case's opening NEVER lidded: no vertex on the top
//     plane inside the tangent circle's disc — uncured: the pierced
//     face's grid covers the axis, RED;
//   * the raccordo EXISTS (the positive pin, the m6 display test's
//     own predicate, unmoved): vertices on the corner cylinder
//     strictly between the trimmed planes;
//   * the counts freed from the fixed grid: the corner cells the
//     arc cuts away are dropped — the fixed 7 x 2N^2 arithmetic is
//     gone with the untrimmed windows;
//   * the faceted fallback stays honest: a face whose co-edges
//     carry no p-curves (the bridge's plane-per-triangle world)
//     falls back to the family window — declared, never a guess.
//
// The guide cases are the register's own: the box (ADR-0039 D5, the
// m2 golden) and the blind rim (D5, the m3 golden) — every number
// quoted from them, never re-derived differently.
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/fillet.hpp"
#include "CAD_0/brep/naming.hpp"
#include "CAD_0/brep/query.hpp"
#include "CAD_0/brep/subtract.hpp"
#include "CAD_0/brep/tessellate.hpp"
#include "CAD_0/brep/topology.hpp"
#include "CAD_0/geometry/shape_id.hpp"
#include "CAD_0/sdf/sketch_profile.hpp"

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <doctest/doctest.h>

using namespace CAD_0;

namespace {

namespace b = brep;
namespace n = brep::naming;
namespace s = sdf;
using geometry::FeatureId;
using geometry::ShapeId;
using geometry::SubshapeRef;

// The box guide case (the m2 golden's own build): the box [-1,1]^3,
// the (+X,+Y) corner's vertical edge — edge 10, `box#1:vertical2` —
// filleted at r.
struct FilletGuide {
    std::unique_ptr<b::BRepBody> box;
    std::unique_ptr<b::BRepBody> result;

    explicit FilletGuide(double radius = 0.5) {
        box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        const ShapeId reference{FeatureId{1},
                                SubshapeRef{}.push(10), {}};
        const b::FilletNaming ctx{FeatureId{2}, &n::kBoxNaming};
        result = b::fillet_edge(*box, reference, radius, ctx);
    }
};

// The blind rim guide case (the m3 golden's own build): the box, the
// circle 0.5 prism height 2, the subtract at offset (0,0,-0.5) — the
// rim through the top at z = 1 — filleted at r = 0.25.
struct RimGuide {
    std::unique_ptr<b::BRepBody> box;
    std::unique_ptr<b::BRepBody> tool;
    std::unique_ptr<b::BRepBody> blind;
    std::unique_ptr<b::BRepBody> result;
    math::Vec3d offset{0.0, 0.0, -0.5};

    explicit RimGuide(double radius = 0.25) {
        box = b::make_box({1.0, 1.0, 1.0}, FeatureId{1});
        std::vector<s::SketchChain> chains;
        chains.push_back(s::SketchChain{
            {s::sketch_circle({0.0f, 0.0f}, 0.5f, false)}});
        tool = b::make_prism(chains, 2.0f, +1, FeatureId{3});
        const b::SubtractNaming sub_ctx{
            FeatureId{4}, &n::kBoxNaming, &n::kExtNaming};
        blind = b::exact_subtract(*box, *tool, offset,
                                  b::kSubtractTolerance, sub_ctx);
        ShapeId rim{};
        for (std::size_t i = 0; i < blind->edge_count(); ++i) {
            if (b::query_edge(*blind, i).name ==
                "sub#4:isect[box#1:top \xc3\x97 ext#3:lat(0)]@hi") {
                rim = blind->edges[i].shape_id();
                break;
            }
        }
        const b::FilletNaming ctx{FeatureId{5}, &n::kBoxNaming,
                                  FeatureId{3}, &n::kExtNaming};
        result = b::fillet_rim_edge(*blind, rim, radius, ctx);
    }
};

constexpr double kPi = 3.14159265358979;

// One grid cell's reach at the display resolution 48 over the widest
// window (the box faces' [-1,1]^2): 2/48, with the diagonal's slack —
// the stairstep's own bound, the class MC's road already declares.
constexpr float kCell = 0.06f;

} // namespace

TEST_SUITE("brep the trimmed display tessellation "
           "(G2-u3 m6-correttiva-bis)") {

TEST_CASE("the box guide: the six crossed planes are dead, the "
          "raccordo lives") {
    const FilletGuide guide(0.5);
    REQUIRE(guide.result != nullptr);

    brep::TessellationOptions opts;
    opts.resolution = 48;
    const sdf::TriangleMesh mesh =
        brep::tessellate_body(*guide.result, opts);
    REQUIRE(mesh.positions.size() > 0);
    REQUIRE(mesh.indices.size() > 0);

    // THE PHANTOM CENSUS: every display vertex inside the body's own
    // world — the box grown by one cell. Uncured: the planes reach
    // +-10 (16,272 of 16,807 vertices outside), RED here.
    for (const auto& v : mesh.positions) {
        CHECK(std::abs(v.x) <= 1.0f + kCell);
        CHECK(std::abs(v.y) <= 1.0f + kCell);
        CHECK(std::abs(v.z) <= 1.0f + kCell);
    }

    // THE FLANK'S SWEEP: the rolling surface is the QUARTER cylinder
    // at the (+X,+Y) corner — every vertex on the corner axis-radius
    // STRICTLY BETWEEN the trimmed planes (the m6 predicate's own
    // z-guard: the top/bottom planes' grid points may sit on the
    // tangent circle's radius legitimately) sits in the quarter's
    // angular window. The radius tolerance is the float32 boundary's
    // own (the flank's vertices land on the circle to ~6e-8; the
    // kept planes' near-tangent grid neighbours sit no closer than
    // ~1e-3 off it — the two classes never meet at 1e-4). Uncured:
    // the full 360 degrees, RED.
    for (const auto& v : mesh.positions) {
        if (std::abs(v.z) >= 0.9f) continue;
        const float dx = v.x - 0.5f;
        const float dy = v.y - 0.5f;
        const float dist = std::sqrt(dx * dx + dy * dy);
        if (std::abs(dist - 0.5f) > 1e-4f) continue;
        const float ang = std::atan2(dy, dx);
        CHECK(ang >= -kCell);
        CHECK(ang <= kPi / 2.0f + kCell);
    }

    // THE RACCORDO EXISTS (the positive pin, the m6 display test's
    // own predicate): a vertex on the corner cylinder strictly
    // between the trimmed planes.
    bool flank_found = false;
    for (const auto& v : mesh.positions) {
        const float dx = v.x - 0.5f;
        const float dy = v.y - 0.5f;
        if (std::abs(std::sqrt(dx * dx + dy * dy) - 0.5f) < 1e-3f &&
            v.x < 1.0f - 1e-3f && v.y < 1.0f - 1e-3f &&
            std::abs(v.z) < 0.9f) {
            flank_found = true;
            break;
        }
    }
    CHECK(flank_found);

    // THE COUNTS FREED FROM THE FIXED GRID: the arc's corner cells
    // are dropped — the untrimmed arithmetic 7 faces x 2N^2 is gone.
    // (At N = 48 the two corner-cut faces lose the cells beyond the
    // r = 0.5 arcs: the cut region's area (1 - pi/4) r^2 ~ 0.05 per
    // face against the cell's (2/48)^2 ~ 0.0017 — dozens of cells.)
    const std::size_t fixed_grid = 7u * 2u * 48u * 48u;
    CHECK(mesh.triangle_count() < fixed_grid);
    CHECK(mesh.triangle_count() > fixed_grid - 4u * 48u);

    // The normals' own discipline: one per vertex, when asked.
    CHECK(mesh.normals.size() == mesh.positions.size());
}

TEST_CASE("the rim guide: the opening never lidded, the world "
          "bounded") {
    const RimGuide guide(0.25);
    REQUIRE(guide.result != nullptr);

    brep::TessellationOptions opts;
    opts.resolution = 48;
    const sdf::TriangleMesh mesh =
        brep::tessellate_body(*guide.result, opts);
    REQUIRE(mesh.positions.size() > 0);

    // THE PHANTOM CENSUS, rim edition: the filleted blind body lives
    // inside the box [-1,1]^3 — one cell of slack. Uncured: the
    // pierced top face's grid reaches +-10, RED here.
    for (const auto& v : mesh.positions) {
        CHECK(std::abs(v.x) <= 1.0f + kCell);
        CHECK(std::abs(v.y) <= 1.0f + kCell);
        CHECK(std::abs(v.z) <= 1.0f + kCell);
    }

    // THE OPENING NEVER LIDDED: no vertex ON the top plane (z = 1)
    // inside the tangent circle's disc — the opening the rim fillet
    // widened to r = 0.75 stays open, the cavity seen for what it
    // is. The cured top face's nearest vertex to the axis is a
    // corner-cut cell's throw from the tangent circle (>= ~0.69);
    // the tube's own points at z ~ 1 sit ON the tangent circle
    // (rho ~ 0.75); the uncured grid covers the axis itself (rho
    // ~ 0) — the pin's window [0, 0.6] discriminates all three.
    for (const auto& v : mesh.positions) {
        if (v.z <= 1.0f - 1e-3f) continue;
        CHECK(std::sqrt(v.x * v.x + v.y * v.y) >= 0.6f);
    }

    // The rim fillet's flank exists in the flesh: the tube's own
    // points — off the top plane, off the wall, strictly inside the
    // corner's quarter-annulus window the torus owns.
    bool tube_found = false;
    for (const auto& v : mesh.positions) {
        const float rho = std::sqrt(v.x * v.x + v.y * v.y);
        if (v.z < 0.55f || v.z > 0.95f) continue;
        if (rho > 0.55f && rho < 0.95f) { tube_found = true; break; }
    }
    CHECK(tube_found);
}

TEST_CASE("the faceted fallback: a face without p-curves keeps the "
          "declared family window") {
    // The bridge's plane-per-triangle world carries NO p-curves —
    // the trim's honest answer there is the fallback, never a guess:
    // the family window (the legacy road), the full grid. This pins
    // the boundary of the cure itself: the trim reads ONLY faces
    // that speak the exact vocabulary, whole.
    auto surf = std::make_shared<b::PlaneSurface>(
        math::Vec3d{0.0, 0.0, 0.0}, math::Vec3d{0.0, 0.0, 1.0});
    b::Face face(surf);
    b::Loop loop;
    // A closed square walk whose co-edges carry NO p-curves.
    const std::vector<b::CoEdge> ces{
        b::CoEdge{0, false}, b::CoEdge{1, false},
        b::CoEdge{2, false}, b::CoEdge{3, false}};
    for (const auto& ce : ces) loop.add_coedge(ce);
    face.add_loop(std::move(loop));

    brep::TessellationOptions opts;
    opts.resolution = 4;
    const sdf::TriangleMesh mesh = brep::tessellate_face(face, opts);
    CHECK(mesh.positions.size() > 0);
    CHECK(mesh.triangle_count() > 0);
    // The family window's own arithmetic, unchanged: the plane's
    // legacy [-10,10]^2 at N = 4 — (N+1)^2 vertices, 2N^2 triangles.
    CHECK(mesh.positions.size() == 25u);
    CHECK(mesh.triangle_count() == 32u);
}

} // TEST_SUITE
