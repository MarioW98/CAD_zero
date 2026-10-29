// tests/unit/brep/test_boolean.cpp
//
// Tests for boolean operations on B-Rep bodies — ADR-0023/0024/0025.
//
// Covers:
//   * Seam wire factories (make_sphere, make_cylinder) produce proper
//     closed manifolds (inversion of the ADR-0022 regression guards).
//   * bool_intersect: overlapping / disjoint / nested / identical boxes,
//     curved-input rejection, degenerate face-touching inputs.
//   * bool_union / bool_subtract: identical-body shortcut, disjoint and
//     containment cases, corner-notch and slab exact-count regressions.
//   * Every non-empty closed result: exact V/E/F counts, χ = 2,
//     loops_are_closed, is_closed_manifold.
//   * Degenerate zero-volume results are asserted exactly as
//     documented (ADR-0025: the face-touching intersect answers the
//     shared disk).
//   * H2 unit 2: the Layer-1 postconditions are HARD — the partial
//     coplanar-overlap open shell is REFUSED with its name (the
//     golden refusal messages), never returned; the multi-shell
//     union (χ = 4) is accepted as a classification.
//
#include <doctest/doctest.h>

#include "CAD_0/brep/topology.hpp"
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/boolean.hpp"
// Test-only seam — NOT part of the public kernel API (see header).
#include "CAD_0/brep/detail/boolean_test_hooks.hpp"

#include <cmath>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using namespace CAD_0::brep;
using namespace CAD_0::math;

// ---------------------------------------------------------------------------
// Seam wire factories — ADR-0023 §1
// ---------------------------------------------------------------------------

TEST_CASE("ADR-0023: make_sphere is a closed manifold (seam wire fix)") {
    auto body = make_sphere(2.0f);
    REQUIRE(body != nullptr);
    CHECK(body->vertex_count() == 2);   // north + south poles
    CHECK(body->edge_count()   == 1);   // single seam edge
    CHECK(body->face_count()   == 1);   // single spherical face
    CHECK(euler_characteristic(*body) == 2);
    CHECK(is_closed_manifold(*body));
    CHECK(loops_are_closed(*body));
}

TEST_CASE("ADR-0023: make_sphere seam edge has 2 coedges on the same face") {
    auto body = make_sphere(2.0f);
    REQUIRE(body->edge_count() == 1);
    const auto& seam = body->edges[0];
    REQUIRE(seam.coedges().size() == 2);
    // Both co-edges reference edge 0.
    for (const CoEdge* ce : seam.coedges()) {
        CHECK(ce->edge() == 0);
    }
    // One forward, one reversed.
    int fwd = 0, rev = 0;
    for (const CoEdge* ce : seam.coedges()) {
        if (ce->reversed()) ++rev; else ++fwd;
    }
    CHECK(fwd == 1);
    CHECK(rev == 1);
}

TEST_CASE("ADR-0023: make_cylinder is a closed manifold (seam wire fix)") {
    auto body = make_cylinder(1.0f, 2.0f);
    REQUIRE(body != nullptr);
    CHECK(body->vertex_count() == 4);   // 2 top + 2 bottom rim
    CHECK(body->edge_count()   == 5);   // 2 top arcs + 2 bottom arcs + 1 seam
    CHECK(body->face_count()   == 3);   // lateral + 2 caps
    CHECK(euler_characteristic(*body) == 2);
    CHECK(is_closed_manifold(*body));
    CHECK(loops_are_closed(*body));
}

TEST_CASE("ADR-0023: make_cylinder seam edge has 2 coedges on lateral face") {
    auto body = make_cylinder(1.0f, 2.0f);
    // Edge 4 is the seam (v_bot1 → v_top1).
    REQUIRE(body->edge_count() >= 5);
    const auto& seam = body->edges[4];
    REQUIRE(seam.coedges().size() == 2);
    // Both co-edges reference edge 4.
    for (const CoEdge* ce : seam.coedges()) {
        CHECK(ce->edge() == 4);
    }
    // One forward, one reversed (opposite orientations).
    int fwd = 0, rev = 0;
    for (const CoEdge* ce : seam.coedges()) {
        if (ce->reversed()) ++rev; else ++fwd;
    }
    CHECK(fwd == 1);
    CHECK(rev == 1);
}

TEST_CASE("ADR-0023: make_cylinder every non-seam edge has 2 distinct faces") {
    auto body = make_cylinder(1.0f, 2.0f);
    // Edges 0-3 are rim arcs; each is shared by a cap and the lateral.
    for (EdgeId eid = 0; eid < 4; ++eid) {
        const auto& edge = body->edges[eid];
        REQUIRE(edge.coedges().size() == 2);
        // The two co-edges must have opposite orientations.
        bool r0 = edge.coedges()[0]->reversed();
        bool r1 = edge.coedges()[1]->reversed();
        CHECK(r0 != r1);
    }
}

// ---------------------------------------------------------------------------
// bool_intersect — overlapping boxes
// ---------------------------------------------------------------------------

TEST_CASE("ADR-0023: bool_intersect of two overlapping boxes produces a box") {
    // Box A: half-extents (1,1,1), centered at origin → [-1,1]³
    // Box B: half-extents (1,1,1), centered at (0.5, 0, 0) → [-0.5, 1.5]³
    // Intersection: [−0.5, 1] × [−1, 1] × [−1, 1]
    // Expected: 8V / 12E / 6F, χ = 2.
    auto A = make_box(Vec3d{1, 1, 1});
    auto B = make_box(Vec3d{1, 1, 1});
    // Translate B by (0.5, 0, 0): shift all vertices.
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3d{0.5f, 0, 0});
    }

    auto result = bool_intersect(*A, *B);
    REQUIRE(result != nullptr);
    CHECK(result->vertex_count() == 8);
    CHECK(result->edge_count()   == 12);
    CHECK(result->face_count()   == 6);
    CHECK(euler_characteristic(*result) == 2);
    CHECK(is_closed_manifold(*result));
    CHECK(loops_are_closed(*result));
}

TEST_CASE("ADR-0023: bool_intersect of disjoint boxes is empty") {
    auto A = make_box(Vec3d{1, 1, 1});
    auto B = make_box(Vec3d{1, 1, 1});
    // Move B far away — no overlap.
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3d{10, 0, 0});
    }
    auto result = bool_intersect(*A, *B);
    REQUIRE(result != nullptr);
    CHECK(result->face_count() == 0);
    CHECK_FALSE(is_closed_manifold(*result));  // empty body, not a manifold
}

TEST_CASE("ADR-0023: bool_intersect of nested boxes returns the inner box") {
    // A = big box (half-extents 2), B = small box (half-extents 1) inside A.
    auto A = make_box(Vec3d{2, 2, 2});
    auto B = make_box(Vec3d{1, 1, 1});  // centered at origin, inside A
    auto result = bool_intersect(*A, *B);
    REQUIRE(result != nullptr);
    // B is entirely inside A, so A ∩ B = B.
    CHECK(result->vertex_count() == 8);
    CHECK(result->edge_count()   == 12);
    CHECK(result->face_count()   == 6);
    CHECK(euler_characteristic(*result) == 2);
    CHECK(is_closed_manifold(*result));
    CHECK(loops_are_closed(*result));
}

TEST_CASE("ADR-0023: bool_intersect of identical boxes returns the same box") {
    auto A = make_box(Vec3d{1, 1, 1});
    auto B = make_box(Vec3d{1, 1, 1});
    auto result = bool_intersect(*A, *B);
    REQUIRE(result != nullptr);
    CHECK(result->vertex_count() == 8);
    CHECK(result->edge_count()   == 12);
    CHECK(result->face_count()   == 6);
    CHECK(euler_characteristic(*result) == 2);
    CHECK(is_closed_manifold(*result));
    CHECK(loops_are_closed(*result));
}

TEST_CASE("ADR-0023: bool_intersect result is a closed manifold") {
    // A=[-1,1]³, B shift (0.5,0.5,0.5) → A ∩ B = [-0.5,1]³, a box:
    // 8V/12E/6F, χ=2, closed manifold.
    auto A = make_box(Vec3d{1, 1, 1});
    auto B = make_box(Vec3d{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3d{0.5f, 0.5f, 0.5f});
    }
    auto result = bool_intersect(*A, *B);
    REQUIRE(result != nullptr);
    CHECK(result->vertex_count() == 8);
    CHECK(result->edge_count()   == 12);
    CHECK(result->face_count()   == 6);
    CHECK(euler_characteristic(*result) == 2);
    CHECK(is_closed_manifold(*result));
    CHECK(loops_are_closed(*result));
    // Every edge must have exactly 2 co-edges (manifold contract).
    for (const auto& edge : result->edges) {
        CHECK(edge.coedges().size() == 2);
    }
}

TEST_CASE("ADR-0023: bool_intersect every edge has opposite coedge orientations") {
    // A=[-1,1]³, B shift (0.5,0,0) → A ∩ B = [-0.5,1]×[-1,1]×[-1,1]:
    // 8V/12E/6F, χ=2, closed manifold; every edge carries one forward
    // and one reversed coedge.
    auto A = make_box(Vec3d{1, 1, 1});
    auto B = make_box(Vec3d{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3d{0.5f, 0, 0});
    }
    auto result = bool_intersect(*A, *B);
    REQUIRE(result != nullptr);
    CHECK(result->vertex_count() == 8);
    CHECK(result->edge_count()   == 12);
    CHECK(result->face_count()   == 6);
    CHECK(euler_characteristic(*result) == 2);
    CHECK(is_closed_manifold(*result));
    CHECK(loops_are_closed(*result));
    for (const auto& edge : result->edges) {
        REQUIRE(edge.coedges().size() == 2);
        bool r0 = edge.coedges()[0]->reversed();
        bool r1 = edge.coedges()[1]->reversed();
        CHECK(r0 != r1);
    }
}

// ---------------------------------------------------------------------------
// bool_intersect with curved-surface body — must throw
// ---------------------------------------------------------------------------

TEST_CASE("ADR-0023: bool_intersect with sphere throws invalid_argument") {
    auto A = make_box(Vec3d{1, 1, 1});
    auto B = make_sphere(2.0f);  // curved surface, not planar
    CHECK_THROWS_AS(bool_intersect(*A, *B), std::invalid_argument);
}

// ---------------------------------------------------------------------------
// ADR-0025 — Degenerate touch inputs (zero-volume intersections)
// ---------------------------------------------------------------------------
//
// Boxes that touch face-to-face intersect in a zero-volume square.
// Sutherland-Hodgman keeps exactly the shared square (its vertices
// lie on the clipping planes within kPlaneTol) and the canonicalize
// weld drops the duplicate from the second pass, so the result is a
// single degenerate face: 4V/4E/1F, χ = 1 (a disk), loops closed,
// but NOT a closed manifold — each of the 4 edges carries a single
// coedge (the body has a boundary).
//

TEST_CASE("ADR-0025: bool_intersect of boxes touching at a face returns the degenerate shared face") {
    // Box A: [-1,1]³
    // Box B: [1,3] × [-1,1] × [-1,1]  (touches A along the whole x=1 face)
    // A ∩ B is the zero-volume square x=1, (y,z) ∈ [-1,1]². Clipping
    // keeps only that square (welded once): V=4, E=4, F=1,
    // χ = 4−4+1 = 1 (a disk), loops closed, is_closed_manifold=false
    // (every edge has a single coedge — the body has a boundary).
    auto A = make_box(Vec3d{1, 1, 1});
    auto B = make_box(Vec3d{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3d{2.0f, 0, 0});
    }
    auto result = bool_intersect(*A, *B);
    REQUIRE(result != nullptr);
    CHECK(result->vertex_count() == 4);
    CHECK(result->edge_count()   == 4);
    CHECK(result->face_count()   == 1);
    CHECK(euler_characteristic(*result) == 1);
    CHECK(loops_are_closed(*result));
    CHECK_FALSE(is_closed_manifold(*result));
}

TEST_CASE("ADR-0025: bool_intersect of stacked boxes (coplanar contact faces) returns the degenerate shared face") {
    // Box A: [-1,1]³
    // Box B: [-1,1] × [-1,1] × [1,3]  (sits on top of A, contact at z=1)
    // Same zero-volume contract as the face-touching case above: the
    // result is the single shared square z=1 → V=4, E=4, F=1, χ=1,
    // loops closed, not a closed manifold.
    auto A = make_box(Vec3d{1, 1, 1});
    auto B = make_box(Vec3d{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3d{0, 0, 2.0f});
    }
    auto result = bool_intersect(*A, *B);
    REQUIRE(result != nullptr);
    CHECK(result->vertex_count() == 4);
    CHECK(result->edge_count()   == 4);
    CHECK(result->face_count()   == 1);
    CHECK(euler_characteristic(*result) == 1);
    CHECK(loops_are_closed(*result));
    CHECK_FALSE(is_closed_manifold(*result));
}

TEST_CASE("ADR-0025: bool_union of boxes touching at a face — strict closed-manifold contract") {
    // Box A: [-1,1]³ → 8V/12E/6F
    // Box B: [-1,1] × [-1,1] × [1,3] → 8V/12E/6F (shifted by (0,0,2))
    // Union: a 2×2×4 box. Topology of the union:
    //   - 4 bottom vertices of A + 4 top vertices of B + 4 shared at z=1 = 12V
    //   - 12 edges of A + 12 edges of B - 4 shared at z=1 + 4 verticals merged
    //     = 12 + 12 - 4 = 20E (the 4 shared bottom-of-B / top-of-A edges weld)
    //   - 5 faces of A (top skipped, inside B) + 5 faces of B (bottom skipped,
    //     inside A) = 10F
    //   - χ = 12 - 20 + 10 = 2 ✓
    //   - is_closed_manifold: every edge has 2 co-edges with opposite orientations
    auto A = make_box(Vec3d{1, 1, 1});
    auto B = make_box(Vec3d{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3d{0, 0, 2.0f});
    }
    auto result = bool_union(*A, *B);
    REQUIRE(result != nullptr);
    CHECK(result->vertex_count() == 12);
    CHECK(result->edge_count()   == 20);
    CHECK(result->face_count()   == 10);
    CHECK(euler_characteristic(*result) == 2);
    CHECK(is_closed_manifold(*result));
    CHECK(loops_are_closed(*result));
}

TEST_CASE("ADR-0025 v5: bool_subtract of box minus a corner — exact counts, closed manifold") {
    // Box A: [-1,1]³ → 8V/12E/6F
    // Box B: [0,2]³ (overlaps the +X corner of A, shifted by (1,1,1))
    // A \ B = A with the [0,1]³ corner removed.
    //
    // Topology (verified after ADR-0025 v5 — parallel-plane rule + cap
    // loop reversal + T-junction edge subdivision):
    //   V = 17: 7 original corners (the (1,1,1) corner is removed)
    //           + 6 cut points on A's original edges at x/y/z = 0
    //           + 3 axis points (1,0,0),(0,1,0),(0,0,1)
    //           + the origin (0,0,0) where the 3 caps meet
    //   F = 12: 3 intact faces (-X,-Y,-Z) + 2 rectangles per cut face
    //           (+X,+Y,+Z — the L-shape split by the slab decomposition)
    //           + 3 cavity caps at x=0, y=0, z=0
    //   E = 27, χ = 17 − 27 + 12 = 2 ✓
    //   is_closed_manifold: every edge exactly 2 co-edges with opposite
    //   orientations (the T-junction subdivision splits the long L-split
    //   edges at the shared vertices so all edges match pairwise).
    auto A = make_box(Vec3d{1, 1, 1});
    auto B = make_box(Vec3d{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3d{1.0f, 1.0f, 1.0f});
    }
    auto result = bool_subtract(*A, *B);
    REQUIRE(result != nullptr);
    CHECK(result->vertex_count() == 17);
    CHECK(result->edge_count()   == 27);
    CHECK(result->face_count()   == 12);
    CHECK(euler_characteristic(*result) == 2);
    CHECK(loops_are_closed(*result));
    CHECK(is_closed_manifold(*result));
}

TEST_CASE("ADR-0025: bool_subtract of disjoint boxes returns A unchanged") {
    // A \ (disjoint B) = A. B is entirely outside A: every face of A
    // survives intact as its own outside fragment, and no face of B
    // lies inside A, so no caps are emitted. Result: 8V/12E/6F.
    auto A = make_box(Vec3d{1, 1, 1});
    auto B = make_box(Vec3d{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3d{10, 0, 0});
    }
    auto result = bool_subtract(*A, *B);
    REQUIRE(result != nullptr);
    CHECK(result->vertex_count() == 8);
    CHECK(result->edge_count()   == 12);
    CHECK(result->face_count()   == 6);
    CHECK(euler_characteristic(*result) == 2);
    CHECK(is_closed_manifold(*result));
    CHECK(loops_are_closed(*result));
}

TEST_CASE("ADR-0025: bool_subtract where B contains A returns empty") {
    // A is entirely inside B: every face of A is fully inside B and is
    // dropped; no face of B is inside A (B is larger), so no caps are
    // emitted. Result: empty body.
    auto A = make_box(Vec3d{1, 1, 1});
    auto B = make_box(Vec3d{3, 3, 3});  // A is entirely inside B
    auto result = bool_subtract(*A, *B);
    REQUIRE(result != nullptr);
    CHECK(result->face_count() == 0);
    CHECK_FALSE(is_closed_manifold(*result));  // empty body
}

TEST_CASE("ADR-0025: bool_subtract of identical boxes returns empty") {
    // A == B → bodies_identical shortcut triggers → empty body.
    auto A = make_box(Vec3d{1, 1, 1});
    auto B = make_box(Vec3d{1, 1, 1});
    auto result = bool_subtract(*A, *B);
    REQUIRE(result != nullptr);
    CHECK(result->face_count() == 0);
    CHECK_FALSE(is_closed_manifold(*result));  // empty body
}

TEST_CASE("ADR-0025 v5: bool_subtract of overlapping boxes — exact counts, closed manifold") {
    // Box A: [-1,1]³
    // Box B: [-0.5, 1.5] × [-1, 1] × [-1, 1] (overlaps +X half of A)
    // A \ B = A with the [0,1] × [-1,1] × [-1,1] slab removed.
    // Expected: V=8, E=12, F=6, χ=2, is_closed_manifold=true (clean snap:
    // 0.5/kWeldGrid = 500000 exactly, no grid-straddle).
    auto A = make_box(Vec3d{1, 1, 1});
    auto B = make_box(Vec3d{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3d{0.5f, 0, 0});
    }
    auto result = bool_subtract(*A, *B);
    REQUIRE(result != nullptr);
    CHECK(result->vertex_count() == 8);
    CHECK(result->edge_count()   == 12);
    CHECK(result->face_count()   == 6);
    CHECK(euler_characteristic(*result) == 2);
    CHECK(loops_are_closed(*result));
    CHECK(is_closed_manifold(*result));
}

TEST_CASE("ADR-0025: bool_subtract partial-overlap with grid-straddle (regression for unwelded interface ring)") {
    // A=[-1,1]³, B shift (0.3,0,0) → B=[-0.7,1.3]×[-1,1]×[-1,1].
    // A \ B = [-1,-0.7]×[-1,1]×[-1,1] (slab of A left of B's -X face).
    // The cut plane is x=-0.7 = B's -X face → Part 2 cap, no missing face.
    // Expected: V=8, E=12, F=6, χ=2, is_closed_manifold=true, loops_are_closed=true.
    auto A = make_box(Vec3d{1, 1, 1});
    auto B = make_box(Vec3d{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3d{0.3f, 0, 0});
    }
    auto result = bool_subtract(*A, *B);
    REQUIRE(result != nullptr);
    CHECK(result->vertex_count() == 8);
    CHECK(result->edge_count()   == 12);
    CHECK(result->face_count()   == 6);
    CHECK(euler_characteristic(*result) == 2);
    CHECK(loops_are_closed(*result));
    CHECK(is_closed_manifold(*result));
}

TEST_CASE("ADR-0025: bool_subtract slab decomposition (subject shares edge with clip, no straddle)") {
    // A=[-1,1]³, B shift (0.5,0,0) → B=[-0.5,1.5]×[-1,1]×[-1,1].
    // A \ B = [-1,-0.5]×[-1,1]×[-1,1].
    // All vertices of A's lateral faces are on B's boundary (shared edges).
    // This is the slab decomposition primitive test: polygon_subtract_3d
    // must produce the outside fragment [-1,-0.5]×[-1,1] for each lateral face.
    // V=8, E=12, F=6, χ=2, is_closed_manifold=true.
    auto A = make_box(Vec3d{1, 1, 1});
    auto B = make_box(Vec3d{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3d{0.5f, 0, 0});
    }
    auto result = bool_subtract(*A, *B);
    REQUIRE(result != nullptr);
    CHECK(result->vertex_count() == 8);
    CHECK(result->edge_count()   == 12);
    CHECK(result->face_count()   == 6);
    CHECK(euler_characteristic(*result) == 2);
    CHECK(loops_are_closed(*result));
    CHECK(is_closed_manifold(*result));
}

TEST_CASE("ADR-0025 v5 FASE 2: minimal slab decomposition — exactly 1 outside fragment") {
    // Grid-straddle case reduced to A's +Y face (y=1):
    //   subject = x ∈ [-1,1], z ∈ [-1,1] at y=1  (A's +Y face, CCW from +Y)
    //   clip    = x ∈ [-0.7,1], z ∈ [-1,1] at y=1 (the inside part after
    //             clipping against B = [-0.7,1.3]×[-1,1]×[-1,1])
    // polygon_subtract_3d must return exactly ONE fragment: the outside
    // rectangle x ∈ [-1,-0.7], z ∈ [-1,1]. The three degenerate slivers
    // from the coincident boundary edges (x=1, z=±1) collapse to ≤ 2
    // vertices after deduplication and are dropped (FASE 1 evidence).
    const std::vector<Vec3d> subject = {
        Vec3d{ 1.0f, 1.0f, -1.0f}, Vec3d{-1.0f, 1.0f, -1.0f},
        Vec3d{-1.0f, 1.0f,  1.0f}, Vec3d{ 1.0f, 1.0f,  1.0f}};
    const std::vector<Vec3d> clip = {
        Vec3d{ 1.0f, 1.0f, -1.0f}, Vec3d{-0.7f, 1.0f, -1.0f},
        Vec3d{-0.7f, 1.0f,  1.0f}, Vec3d{ 1.0f, 1.0f,  1.0f}};
    const auto frags = polygon_subtract_fragments_for_tests(
        subject, clip, Vec3d{0.0f, 1.0f, 0.0f});
    REQUIRE(frags.size() == 1);
    REQUIRE(frags[0].size() == 4);
    // The single fragment is the outside rectangle: every vertex lies in
    // x ∈ [-1,-0.7], y = 1, z ∈ [-1,1].
    for (const auto& v : frags[0]) {
        CHECK(v.x <= -0.7f + 1e-5f);
        CHECK(v.x >= -1.0f - 1e-5f);
        CHECK(std::abs(v.y - 1.0f) < 1e-5f);
        CHECK(v.z >= -1.0f - 1e-5f);
        CHECK(v.z <=  1.0f + 1e-5f);
    }
}

TEST_CASE("ADR-0025: bool_intersect of nested boxes (B inside A) returns B") {
    // Box A: half-extents (3,3,3) → [-3,3]³
    // Box B: half-extents (1,1,1) → [-1,1]³ (inside A)
    auto A = make_box(Vec3d{3, 3, 3});
    auto B = make_box(Vec3d{1, 1, 1});
    auto result = bool_intersect(*A, *B);
    REQUIRE(result != nullptr);
    // B is entirely inside A, so A ∩ B = B.
    CHECK(result->vertex_count() == 8);
    CHECK(result->edge_count()   == 12);
    CHECK(result->face_count()   == 6);
    CHECK(euler_characteristic(*result) == 2);
    CHECK(is_closed_manifold(*result));
    CHECK(loops_are_closed(*result));
}

TEST_CASE("ADR-0025: bool_union of identical boxes returns one box (regression)") {
    // ADR-0025 identical-body shortcut must trigger.
    auto A = make_box(Vec3d{1, 1, 1});
    auto B = make_box(Vec3d{1, 1, 1});
    auto result = bool_union(*A, *B);
    REQUIRE(result != nullptr);
    CHECK(result->vertex_count() == 8);
    CHECK(result->edge_count()   == 12);
    CHECK(result->face_count()   == 6);
    CHECK(euler_characteristic(*result) == 2);
    CHECK(is_closed_manifold(*result));
    CHECK(loops_are_closed(*result));
}

TEST_CASE("ADR-0025: bool_subtract of identical boxes returns empty (regression)") {
    auto A = make_box(Vec3d{1, 1, 1});
    auto B = make_box(Vec3d{1, 1, 1});
    auto result = bool_subtract(*A, *B);
    REQUIRE(result != nullptr);
    CHECK(result->face_count() == 0);
    CHECK_FALSE(is_closed_manifold(*result));  // empty body
}

TEST_CASE("ADR-0025: bool_intersect of identical boxes returns one box (regression)") {
    auto A = make_box(Vec3d{1, 1, 1});
    auto B = make_box(Vec3d{1, 1, 1});
    auto result = bool_intersect(*A, *B);
    REQUIRE(result != nullptr);
    // A ∩ A = A: Sutherland-Hodgman keeps all 6 faces of the first
    // pass and the canonicalize weld drops the duplicate second pass.
    // Result: 8V/12E/6F, χ=2, closed manifold.
    CHECK(result->vertex_count() == 8);
    CHECK(result->edge_count()   == 12);
    CHECK(result->face_count()   == 6);
    CHECK(euler_characteristic(*result) == 2);
    CHECK(is_closed_manifold(*result));
    CHECK(loops_are_closed(*result));
}

// ---------------------------------------------------------------------------
// H2 unit 2 — THE OUTPUT GATE: the open-shell refusal.
// ---------------------------------------------------------------------------
//
// bool_union(A, B) = (A's faces outside B) ∪ (B's faces outside A).
// When the two boxes share full lateral faces (partial coplanar
// overlap), each side's lateral faces are clipped against the other
// solid, leaving a gap over the overlap region: the assembly is an
// open shell — 8 rim edges with a single co-edge. While the ADR-0025
// v5 limitation stood, that non-manifold body was RETURNED as a
// BRepBody (two tests asserted it) and could enter the feature tree
// spacciato per valido. H2 unit 2 flipped the contract: the Layer-1
// postconditions are hard, the result is REFUSED with its name, and
// the refusal message IS the golden. The class's full handling is
// H2 unit 5 — the refusal is the declared interim answer.

TEST_CASE("H2-u2: bool_union with partial coplanar overlap — the open shell is refused, not returned") {
    // Box A: [-1,1]³
    // Box B: [-0.5, 1.5] × [-1, 1] × [-1, 1] (overlaps +X half of A;
    //        the ±Y and ±Z faces are coplanar with A's)
    // A ∪ B = [-1,1.5] × [-1,1] × [-1,1], but with partial coplanar
    // overlap the union's assembly keeps only A's faces left of x=-0.5
    // and B's faces right of x=1: two open half-shells whose 8 rim
    // edges (4 at x=-0.5, 4 at x=1) carry a single co-edge. The old
    // contract returned that body (16V/24E/10F, chi=2, NOT a closed
    // manifold); the H2 contract refuses it.
    auto A = make_box(Vec3d{1, 1, 1});
    auto B = make_box(Vec3d{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3d{0.5f, 0, 0});
    }
    bool threw = false;
    try {
        auto result = bool_union(*A, *B);
        (void)result;
    } catch (const std::runtime_error& e) {
        threw = true;
        const std::string msg = e.what();
        INFO("refusal message: ", msg);
        CHECK(msg.find("bool_union") != std::string::npos);
        CHECK(msg.find("result_edge_not_manifold") != std::string::npos);
        CHECK(msg.find("8 of 24") != std::string::npos);
        CHECK(msg.find("PARTIAL COPLANAR OVERLAP") != std::string::npos);
        CHECK(msg.find("H2 unit 5") != std::string::npos);
        CHECK(msg.find("Remedy:") != std::string::npos);
    }
    CHECK_MESSAGE(threw, "the open shell must be refused, not returned");
}

TEST_CASE("H2-u2: the open-shell refusal names the rim census (the golden message)") {
    // The census that used to be asserted on the RETURNED body (8 rim
    // edges / 16 interior edges) now lives in the refusal text: the
    // message IS the record of what the assembly produced.
    auto A = make_box(Vec3d{1, 1, 1});
    auto B = make_box(Vec3d{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3d{0.5f, 0, 0});
    }
    try {
        auto result = bool_union(*A, *B);
        (void)result;
        FAIL("the open shell must be refused, not returned");
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        CHECK(msg.find("8 of 24 edges carry a single co-edge") !=
              std::string::npos);
        CHECK(msg.find("must not enter the feature tree") !=
              std::string::npos);
    }
}

TEST_CASE("H2-u2: the union of disjoint bodies is a valid multi-shell answer (chi = 4, accepted)") {
    // Two disjoint boxes: every face survives, the assembly carries
    // TWO closed shells in one body — chi = 16-24+12 = 4. Per the
    // ADR-0022 amendment, multi-shell is a CLASSIFICATION, not an
    // assembly defect: the output gate accepts it (even chi, structure
    // passing) and genus_from_chi declines to derive one — the
    // reported genus reads n/d, never a negative.
    auto A = make_box(Vec3d{1, 1, 1});
    auto B = make_box(Vec3d{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3d{10, 0, 0});
    }
    auto result = bool_union(*A, *B);
    REQUIRE(result != nullptr);
    CHECK(result->vertex_count() == 16);
    CHECK(result->edge_count()   == 24);
    CHECK(result->face_count()   == 12);
    CHECK(euler_characteristic(*result) == 4);
    CHECK(is_closed_orientable(*result));
    CHECK_FALSE(genus_from_chi(4).has_value());
    CHECK(loops_are_closed(*result));
}

TEST_CASE("ADR-0024: bool_union of identical boxes returns A (identical-body shortcut)") {
    // ADR-0025: bodies_identical() shortcut detects A == B and returns
    // a copy of A. The result is 8V/12E/6F with χ = 2 and is_closed_manifold.
    auto A = make_box(Vec3d{1, 1, 1});
    auto B = make_box(Vec3d{1, 1, 1});  // same position
    auto result = bool_union(*A, *B);
    REQUIRE(result != nullptr);
    CHECK(result->vertex_count() == 8);
    CHECK(result->edge_count()   == 12);
    CHECK(result->face_count()   == 6);
    CHECK(euler_characteristic(*result) == 2);
    CHECK(is_closed_manifold(*result));
    CHECK(loops_are_closed(*result));
}

TEST_CASE("H2-u2: the refusal replaces the old open-shell coedge census") {
    // HISTORICAL: this test asserted the returned open shell's coedge
    // census (24 edges: 16 with 2 co-edges, 8 rim edges with 1). H2
    // unit 2 refuses that body — the census survives in the refusal
    // message (the golden above); nothing with a boundary rim reaches
    // a caller anymore.
    auto A = make_box(Vec3d{1, 1, 1});
    auto B = make_box(Vec3d{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3d{0.5f, 0, 0});
    }
    CHECK_THROWS_AS(bool_union(*A, *B), std::runtime_error);
}

// ---------------------------------------------------------------------------
// ADR-0024 — bool_subtract
// ---------------------------------------------------------------------------
//
// bool_subtract(A, B) = (A's faces outside B) ∪ (B's faces inside A,
// with inverted normals). For a box minus an overlapping box, the
// result is a box with a corner notch removed.
//

TEST_CASE("ADR-0024: bool_subtract of overlapping boxes — corner notch, exact counts, closed manifold") {
    // Box A: [-1,1]³, B shift (0.5,0.5,0.5) → B=[-0.5,1.5]³.
    // A \ B = A minus the corner sub-cube [-0.5,1]³ — same corner-cut
    // family as the ADR-0025 v5 test above, with cut planes at -0.5
    // (0.5/kWeldGrid snaps exactly, no grid-straddle):
    //   V = 17: 7 original corners (the (1,1,1) corner is removed)
    //           + 6 cut/split points on A's edges at x/y/z = -0.5
    //           + 3 reflex corners of the L-shaped faces
    //           + 1 corner where the 3 cavity caps meet
    //   F = 12: 3 intact faces + 2 rectangles per L-shaped face
    //           (slab decomposition split) + 3 cavity caps
    //   E = 27, χ = 17 − 27 + 12 = 2, closed manifold, loops closed.
    auto A = make_box(Vec3d{1, 1, 1});
    auto B = make_box(Vec3d{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3d{0.5f, 0.5f, 0.5f});
    }
    auto result = bool_subtract(*A, *B);
    REQUIRE(result != nullptr);
    CHECK(result->vertex_count() == 17);
    CHECK(result->edge_count()   == 27);
    CHECK(result->face_count()   == 12);
    CHECK(euler_characteristic(*result) == 2);
    CHECK(loops_are_closed(*result));
    CHECK(is_closed_manifold(*result));
}

TEST_CASE("ADR-0024: bool_subtract of disjoint boxes returns A unchanged") {
    // A \ (disjoint B) = A: every face of A survives intact (no overlap
    // with B), and no face of B lies inside A, so Part 2 emits no caps.
    // Result is A: 8V/12E/6F.
    auto A = make_box(Vec3d{1, 1, 1});
    auto B = make_box(Vec3d{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3d{10, 0, 0});
    }
    auto result = bool_subtract(*A, *B);
    REQUIRE(result != nullptr);
    CHECK(result->vertex_count() == 8);
    CHECK(result->edge_count()   == 12);
    CHECK(result->face_count()   == 6);
    CHECK(euler_characteristic(*result) == 2);
    CHECK(is_closed_manifold(*result));
    CHECK(loops_are_closed(*result));
}

TEST_CASE("ADR-0024: bool_subtract where B contains A returns empty") {
    auto A = make_box(Vec3d{1, 1, 1});
    auto B = make_box(Vec3d{3, 3, 3});  // A is entirely inside B
    auto result = bool_subtract(*A, *B);
    REQUIRE(result != nullptr);
    CHECK(result->face_count() == 0);
    CHECK_FALSE(is_closed_manifold(*result));  // empty body
}

TEST_CASE("ADR-0024: bool_subtract of overlapping boxes — every edge has exactly 2 coedges") {
    // Corner-notch result (17V/27E/12F): the T-junction subdivision
    // makes every edge match pairwise, so all 27 edges carry exactly
    // 2 coedges (closed-manifold contract).
    auto A = make_box(Vec3d{1, 1, 1});
    auto B = make_box(Vec3d{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3d{0.5f, 0.5f, 0.5f});
    }
    auto result = bool_subtract(*A, *B);
    REQUIRE(result != nullptr);
    REQUIRE(result->edge_count() == 27);
    for (const auto& edge : result->edges) {
        CHECK(edge.coedges().size() == 2);
    }
}

TEST_CASE("ADR-0024: bool_subtract of overlapping boxes — χ = 2 on the corner notch") {
    // Same corner-notch input as the tests above: 17V/27E/12F gives
    // χ = 17 − 27 + 12 = 2 on a closed manifold.
    auto A = make_box(Vec3d{1, 1, 1});
    auto B = make_box(Vec3d{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3d{0.5f, 0.5f, 0.5f});
    }
    auto result = bool_subtract(*A, *B);
    REQUIRE(result != nullptr);
    CHECK(result->vertex_count() == 17);
    CHECK(result->edge_count()   == 27);
    CHECK(result->face_count()   == 12);
    CHECK(euler_characteristic(*result) == 2);
    CHECK(loops_are_closed(*result));
    CHECK(is_closed_manifold(*result));
}

// ---------------------------------------------------------------------------
// Cross-validation: union is consistent with intersect
// ---------------------------------------------------------------------------

TEST_CASE("ADR-0024: bool_union and bool_intersect of overlapping boxes — exact counts on both sides") {
    // A=[-1,1]³, B shift (0.5,0,0) → overlap [-0.5,1]×[-1,1]×[-1,1].
    //   intersect = the overlap box: 8V/12E/6F, χ=2, closed manifold.
    //   union = the partial coplanar overlap class — REFUSED since H2
    //           unit 2 (the open shell, 8 rim edges of 24, is never
    //           returned); the intersect side keeps its exact counts.
    auto A = make_box(Vec3d{1, 1, 1});
    auto B = make_box(Vec3d{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3d{0.5f, 0, 0});
    }
    CHECK_THROWS_AS(bool_union(*A, *B), std::runtime_error);
    auto i = bool_intersect(*A, *B);
    REQUIRE(i != nullptr);
    CHECK(i->vertex_count() == 8);
    CHECK(i->edge_count()   == 12);
    CHECK(i->face_count()   == 6);
    CHECK(euler_characteristic(*i) == 2);
    CHECK(loops_are_closed(*i));
    CHECK(is_closed_manifold(*i));
}

TEST_CASE("ADR-0027: wrapper equivalence on the genus-0 boolean fixtures") {
    // D6 move 3 (the zero-silent-weakening contract) over THIS suite's
    // fixtures: is_closed_manifold == (is_closed_orientable && χ == 2)
    // on the box booleans and the corner notch — the exact bodies that
    // made ADR-0025 v5. The identity must hold on the healthy results
    // (closed, χ = 2), on the zero-volume contact disk (false through
    // BOTH halves: χ = 1 and structurally open), and on the multi-shell
    // disjoint union (false through the χ half ONLY: the structure is
    // sound, χ = 4 is a classification). The partial coplanar open
    // shell — the historical χ=2-but-open fixture — is refused since
    // H2 unit 2 and cannot be a fixture anymore: that refusal is the
    // very point of the unit. If the wrapper is ever edited, these are
    // the fixture classes that must notice.
    auto check = [](const BRepBody& b) {
        CHECK(is_closed_manifold(b) ==
              (is_closed_orientable(b) && euler_characteristic(b) == 2));
    };

    auto box_at = [](Vec3d shift) {
        auto b = make_box(Vec3d{1, 1, 1});
        for (auto& v : b->vertices) {
            v.set_position(v.position() + shift);
        }
        return b;
    };

    // bool_intersect of overlapping boxes: the overlap box, 8V/12E/6F.
    {
        auto i = bool_intersect(*make_box(Vec3d{1, 1, 1}), *box_at({0.5f, 0, 0}));
        REQUIRE(i != nullptr);
        CHECK(euler_characteristic(*i) == 2);
        check(*i);
    }
    // bool_union of boxes touching at a face: 12V/20E/10F, χ = 2.
    {
        auto u = bool_union(*make_box(Vec3d{1, 1, 1}), *box_at({0, 0, 2.0f}));
        REQUIRE(u != nullptr);
        CHECK(euler_characteristic(*u) == 2);
        check(*u);
    }
    // The corner notch (the ADR-0025 v5 flagship): 17V/27E/12F, χ = 2.
    {
        auto d = bool_subtract(*make_box(Vec3d{1, 1, 1}), *box_at({1.0f, 1.0f, 1.0f}));
        REQUIRE(d != nullptr);
        CHECK(euler_characteristic(*d) == 2);
        check(*d);
    }
    // The slab (clean snap): 8V/12E/6F, χ = 2.
    {
        auto d = bool_subtract(*make_box(Vec3d{1, 1, 1}), *box_at({0.5f, 0, 0}));
        REQUIRE(d != nullptr);
        CHECK(euler_characteristic(*d) == 2);
        check(*d);
    }
    // Disjoint subtract: A unchanged, 8V/12E/6F, χ = 2.
    {
        auto d = bool_subtract(*make_box(Vec3d{1, 1, 1}), *box_at({10, 0, 0}));
        REQUIRE(d != nullptr);
        CHECK(euler_characteristic(*d) == 2);
        check(*d);
    }
    // The zero-volume contact disk (the face-touching intersect, the
    // pinned ADR-0025 class): χ = 1, structurally open — the wrapper
    // is false through BOTH halves, and the equivalence holds.
    {
        auto d = bool_intersect(*make_box(Vec3d{1, 1, 1}), *box_at({2.0f, 0, 0}));
        REQUIRE(d != nullptr);
        CHECK(euler_characteristic(*d) == 1);
        CHECK_FALSE(is_closed_orientable(*d));
        CHECK_FALSE(is_closed_manifold(*d));
        check(*d);
    }
    // The multi-shell disjoint union (H2 unit 2's accepted
    // classification): χ = 4, structure sound — the wrapper is false
    // through the χ half ONLY, and the equivalence still holds.
    {
        auto u = bool_union(*make_box(Vec3d{1, 1, 1}), *box_at({10, 0, 0}));
        REQUIRE(u != nullptr);
        CHECK(euler_characteristic(*u) == 4);
        CHECK(is_closed_orientable(*u));
        CHECK_FALSE(is_closed_manifold(*u));
        check(*u);
    }
    // The partial coplanar-overlap union: REFUSED since H2 unit 2 —
    // the open shell is not a fixture anymore, it is an error.
    {
        CHECK_THROWS_AS(bool_union(*make_box(Vec3d{1, 1, 1}),
                                   *box_at({0.5f, 0, 0})),
                        std::runtime_error);
    }
}

// ===========================================================================
// Property-based family (mini-traccia PARTE 2, area 3 — B-Rep boolean).
// THE ANALYTIC-ORACLE FAMILY: random axis-aligned boxes with variable
// overlap. The intersection of two axis-aligned boxes is a box computable
// IN CLOSED FORM, so the property checks exact V/E/F AND the vertex
// positions — the highest degree of verification (property with oracle).
//
// Generator input class, declared (a violation is a GENERATOR bug,
// classified as such — never a kernel bug):
//   * half-extents of A and B: uniform in [0.5, 1.5] per axis; A stays
//     centered at the origin, B is placed by its center.
//   * B's center: uniform in [-0.8, 0.8]^3, REJECTED unless, on every
//     axis, the intersection side is >= 0.30 (>> kWeldGrid = 1e-6) and
//     no A/B face pair lies within 1e-3 of coincidence (coincident
//     faces are a degenerate regime the pinned suites already cover;
//     1e-3 = 1000x kWeldGrid keeps the clipping well-conditioned).
//
// Why a dedicated make_box_at (instead of shifting make_box() vertices
// post hoc, as the pinned suites do): the documented bool_intersect
// precondition is convex polyhedra whose faces are planar. Shifting
// only the vertices leaves every PlaneSurface at its origin-centered
// plane — a face whose vertices no longer lie on its own surface. That
// input is OUT OF CLASS for an oracle that checks POSITIONS (it can
// still count 8V/12E/6F by topology while being geometrically
// self-inconsistent). make_box_at translates the vertices AND the
// plane origins together, so the body is coherent by construction.
// ===========================================================================

#include "property/property_prng.hpp"

#include <memory>

namespace {

namespace prop = CAD_0::test::property;

// Axis-aligned box B-Rep with half-extents `e` centered at `c` — the
// coherent translate of brep::make_box (factory.hpp): same vertex/edge/
// face numbering and outward normals, PlaneSurface origins translated
// WITH the vertices so every face's vertices lie on its own plane.
std::unique_ptr<BRepBody> make_box_at(Vec3d e, Vec3d c) {
    auto body = std::make_unique<BRepBody>();
    auto& solid = body->solid;
    auto& shell = solid.shell_mut();
    shell.set_closed(true);

    const double x = e.x, y = e.y, z = e.z;
    const double cx = c.x, cy = c.y, cz = c.z;
    body->vertices = {
        Vertex{{cx - x, cy - y, cz - z}}, Vertex{{cx + x, cy - y, cz - z}},
        Vertex{{cx + x, cy + y, cz - z}}, Vertex{{cx - x, cy + y, cz - z}},
        Vertex{{cx - x, cy - y, cz + z}}, Vertex{{cx + x, cy - y, cz + z}},
        Vertex{{cx + x, cy + y, cz + z}}, Vertex{{cx - x, cy + y, cz + z}},
    };
    for (std::size_t i = 0; i < body->vertices.size(); ++i)
        body->vertices[i].set_id(static_cast<VertexId>(i));

    body->edges = {
        Edge{0,1}, Edge{1,2}, Edge{2,3}, Edge{3,0},
        Edge{4,5}, Edge{5,6}, Edge{6,7}, Edge{7,4},
        Edge{0,4}, Edge{1,5}, Edge{2,6}, Edge{3,7},
    };
    for (std::size_t i = 0; i < body->edges.size(); ++i)
        body->edges[i].set_id(static_cast<EdgeId>(i));

    auto add_plane_face = [&shell](
        Vec3d origin, Vec3d normal, FaceId id,
        EdgeId e0, bool r0, EdgeId e1, bool r1,
        EdgeId e2, bool r2, EdgeId e3, bool r3) {
        Face face(std::make_shared<PlaneSurface>(origin, normal));
        Loop loop;
        loop.add_coedge(CoEdge{e0, r0});
        loop.add_coedge(CoEdge{e1, r1});
        loop.add_coedge(CoEdge{e2, r2});
        loop.add_coedge(CoEdge{e3, r3});
        face.add_loop(std::move(loop));
        face.set_id(id);
        shell.add_face(std::move(face));
    };

    // Same numbering as factory.hpp (see there for the cross-product
    // orientation checks — unchanged by the translation).
    add_plane_face({cx, cy, cz - z}, {0, 0, -1}, 0,  3, true,  2, true,  1, true,  0, true);
    add_plane_face({cx, cy, cz + z}, {0, 0,  1}, 1,  4, false, 5, false, 6, false, 7, false);
    add_plane_face({cx, cy - y, cz}, {0, -1, 0}, 2,  0, false, 9, false, 4, true,  8, true);
    add_plane_face({cx, cy + y, cz}, {0,  1, 0}, 3,  2, false, 11, false, 6, true, 10, true);
    add_plane_face({cx - x, cy, cz}, {-1, 0, 0}, 4,  3, false, 8, false, 7, true, 11, true);
    add_plane_face({cx + x, cy, cz}, { 1, 0, 0}, 5,  1, false, 10, false, 5, true,  9, true);

    link_radial_edges(*body);
    return body;
}

} // namespace

TEST_CASE("property: bool_intersect of random axis-aligned boxes matches the analytic box oracle") {
    const std::uint32_t seed = 0xA11CED01u;
    prop::Prng prng(seed);
    // kWeldGrid, mirrored from core/brep/src/boolean.cpp:44 (private
    // there; same value, same meaning — the kernel's welding grid).
    constexpr float kOracleWeldGrid = 1e-6f;

    for (int iter = 0; iter < 25; ++iter) {
        const Vec3d eA{prng.frange(0.5f, 1.5f),
                       prng.frange(0.5f, 1.5f),
                       prng.frange(0.5f, 1.5f)};
        const Vec3d eB{prng.frange(0.5f, 1.5f),
                       prng.frange(0.5f, 1.5f),
                       prng.frange(0.5f, 1.5f)};

        // Rejection sampling until in-class (deterministic: the same
        // seed replays the same draws, so a failure is reproducible).
        Vec3d shift{};
        Vec3d lo{}, hi{};
        {
            int tries = 0;
            for (;;) {
                ++tries;
                REQUIRE(tries < 100000);  // generator bug if exhausted
                shift = Vec3d{prng.frange(-0.8f, 0.8f),
                              prng.frange(-0.8f, 0.8f),
                              prng.frange(-0.8f, 0.8f)};
                const double sa[3] = {shift.x, shift.y, shift.z};
                const double ea[3] = {eA.x, eA.y, eA.z};
                const double eb[3] = {eB.x, eB.y, eB.z};
                double l3[3], h3[3];
                bool ok = true;
                for (int i = 0; i < 3; ++i) {
                    l3[i] = std::max(-ea[i], sa[i] - eb[i]);
                    h3[i] = std::min(ea[i], sa[i] + eb[i]);
                    if (h3[i] - l3[i] < 0.30f) ok = false;  // min side (>> weld)
                    if (std::abs((sa[i] - eb[i]) - (-ea[i])) < 1e-3f) ok = false;
                    if (std::abs((sa[i] + eb[i]) - ea[i]) < 1e-3f) ok = false;
                }
                if (ok) {
                    lo = Vec3d{l3[0], l3[1], l3[2]};
                    hi = Vec3d{h3[0], h3[1], h3[2]};
                    break;
                }
            }
        }

        auto A = make_box_at(eA, Vec3d{0, 0, 0});
        auto B = make_box_at(eB, shift);
        auto result = bool_intersect(*A, *B);

        INFO("seed=", seed, " iter=", iter,
             " eA=(", eA.x, ",", eA.y, ",", eA.z, ")",
             " eB=(", eB.x, ",", eB.y, ",", eB.z, ")",
             " shift=(", shift.x, ",", shift.y, ",", shift.z, ")");
        INFO("oracle box: [", lo.x, ",", lo.y, ",", lo.z, "] .. [",
             hi.x, ",", hi.y, ",", hi.z, "]");

        REQUIRE(result != nullptr);
        INFO("check: V == 8 (oracle)"); CHECK(result->vertex_count() == 8);
        INFO("check: E == 12 (oracle)"); CHECK(result->edge_count() == 12);
        INFO("check: F == 6 (oracle)"); CHECK(result->face_count() == 6);
        INFO("check: euler characteristic == 2");
        CHECK(euler_characteristic(*result) == 2);
        INFO("check: is_closed_manifold");
        CHECK(is_closed_manifold(*result));
        INFO("check: loops_are_closed");
        CHECK(loops_are_closed(*result));

        // Positions: the 8 analytic corners of [lo, hi]^3.
        const Vec3d corners[8] = {
            {lo.x, lo.y, lo.z}, {hi.x, lo.y, lo.z},
            {lo.x, hi.y, lo.z}, {hi.x, hi.y, lo.z},
            {lo.x, lo.y, hi.z}, {hi.x, lo.y, hi.z},
            {lo.x, hi.y, hi.z}, {hi.x, hi.y, hi.z},
        };
        bool covered[8] = {false, false, false, false, false, false, false, false};
        for (std::size_t i = 0; i < result->vertex_count(); ++i) {
            const Vec3d p = result->vertices[i].position();
            double best = 1e30f;
            int best_j = -1;
            for (int j = 0; j < 8; ++j) {
                const Vec3d d = p - corners[j];
                const double dl = d.length();
                if (dl < best) { best = dl; best_j = j; }
            }
            INFO("vertex ", i, " p=(", p.x, ",", p.y, ",", p.z, ")",
                 " nearest oracle corner #", best_j, " dist=", best);
            INFO("check: dist(result vertex, nearest oracle corner) <= kWeldGrid (1e-6)");
            CHECK(best <= kOracleWeldGrid);
            if (best <= kOracleWeldGrid) covered[best_j] = true;
        }
        bool all_covered = true;
        for (int j = 0; j < 8; ++j) all_covered = all_covered && covered[j];
        INFO("check: every oracle corner is matched by a result vertex (bijection)");
        CHECK(all_covered);
    }
}

// Sanity for the property family's factory: make_box_at(c = 0) must be
// structurally identical to brep::make_box, and a translated box must
// still be a closed orientable manifold on its own. If this fails, the
// ORACLE FAMILY's failures are generator bugs, not kernel bugs.
TEST_CASE("property-family factory sanity: make_box_at is a coherent closed box") {
    const Vec3d e{1.25f, 0.75f, 1.0f};
    const Vec3d c{0.3f, -0.4f, 0.2f};

    auto at_origin = make_box_at(e, Vec3d{0, 0, 0});
    auto reference = make_box(e);
    CHECK(at_origin->vertex_count() == reference->vertex_count());
    CHECK(at_origin->edge_count() == reference->edge_count());
    CHECK(at_origin->face_count() == reference->face_count());
    CHECK(euler_characteristic(*at_origin) == 2);
    CHECK(is_closed_orientable(*at_origin));

    auto moved = make_box_at(e, c);
    CHECK(euler_characteristic(*moved) == 2);
    CHECK(is_closed_orientable(*moved));
    CHECK(loops_are_closed(*moved));
    // Vertices really at c +- e (the factory translated them).
    bool found_min = false, found_max = false;
    for (const auto& v : moved->vertices) {
        const Vec3d p = v.position();
        if (p.x == c.x - e.x && p.y == c.y - e.y && p.z == c.z - e.z) found_min = true;
        if (p.x == c.x + e.x && p.y == c.y + e.y && p.z == c.z + e.z) found_max = true;
    }
    CHECK(found_min);
    CHECK(found_max);

    // Identical boxes through the property factory: intersection = the
    // box itself (mirrors the pinned "identical boxes" case).
    auto same = bool_intersect(*make_box_at(e, Vec3d{0, 0, 0}),
                               *make_box_at(e, Vec3d{0, 0, 0}));
    REQUIRE(same != nullptr);
    CHECK(same->vertex_count() == 8);
    CHECK(same->edge_count() == 12);
    CHECK(same->face_count() == 6);
    CHECK(euler_characteristic(*same) == 2);
    CHECK(is_closed_manifold(*same));

    // The pinned overlapping case through the property factory: A at
    // the origin, B shifted by (0.5, 0, 0) — expected [-0.5, 1]^3.
    auto ov = bool_intersect(*make_box_at(Vec3d{1, 1, 1}, Vec3d{0, 0, 0}),
                             *make_box_at(Vec3d{1, 1, 1}, Vec3d{0.5f, 0, 0}));
    REQUIRE(ov != nullptr);
    CHECK(ov->vertex_count() == 8);
    CHECK(ov->edge_count() == 12);
    CHECK(ov->face_count() == 6);
    CHECK(euler_characteristic(*ov) == 2);
    CHECK(is_closed_manifold(*ov));
}

// PINNED REGRESSION — the bug the property family found, made literal
// and generator-independent. This is seed 0xA11CED01, iteration 0 of
// "property: bool_intersect ... analytic box oracle" (values printed
// with 9 significant digits — exact float round-trip). Before the
// weld grid-boundary-straddle repair in assemble_body this produced
// V=10 / E=16 / chi=0 (non-manifold): the corner (-0.386547, -1.088843,
// -1.186705) was computed twice by different clipping paths, one ulp
// apart, and the two occurrences landed on opposite sides of a kWeldGrid
// cell boundary, so the snapped-key weld kept both as "distinct"
// vertices ~1e-10 apart. The repair (proximity pass honoring the
// kWeldGrid contract) welds them; the oracle holds exactly.
TEST_CASE("pinned regression: weld grid-boundary straddle on a box-pair intersection") {
    const Vec3d eA{1.0915029f, 1.08884263f, 1.40153289f};
    const Vec3d eB{1.12535167f, 1.38211584f, 1.032341f};
    const Vec3d shift{0.738804758f, -0.448618621f, -0.154364496f};

    auto A = make_box_at(eA, Vec3d{0, 0, 0});
    auto B = make_box_at(eB, shift);
    auto result = bool_intersect(*A, *B);
    REQUIRE(result != nullptr);

    // Analytic oracle: [max(-eA, shift-eB), min(eA, shift+eB)] per axis.
    const Vec3d lo{std::max(-eA.x, shift.x - eB.x),
                   std::max(-eA.y, shift.y - eB.y),
                   std::max(-eA.z, shift.z - eB.z)};
    const Vec3d hi{std::min(eA.x, shift.x + eB.x),
                   std::min(eA.y, shift.y + eB.y),
                   std::min(eA.z, shift.z + eB.z)};

    CHECK(result->vertex_count() == 8);
    CHECK(result->edge_count() == 12);
    CHECK(result->face_count() == 6);
    CHECK(euler_characteristic(*result) == 2);
    CHECK(is_closed_manifold(*result));
    CHECK(loops_are_closed(*result));

    // Positions within kWeldGrid (1e-6) of the analytic corners.
    constexpr float kTol = 1e-6f;
    const Vec3d corners[8] = {
        {lo.x, lo.y, lo.z}, {hi.x, lo.y, lo.z},
        {lo.x, hi.y, lo.z}, {hi.x, hi.y, lo.z},
        {lo.x, lo.y, hi.z}, {hi.x, lo.y, hi.z},
        {lo.x, hi.y, hi.z}, {hi.x, hi.y, hi.z},
    };
    for (std::size_t i = 0; i < result->vertex_count(); ++i) {
        const Vec3d p = result->vertices[i].position();
        double best = 1e30f;
        for (const Vec3d& c : corners) {
            const Vec3d d = p - c;
            const double dl = d.length();
            if (dl < best) best = dl;
        }
        INFO("vertex ", i, " dist to nearest oracle corner = ", best);
        CHECK(best <= kTol);
    }
}

// ===========================================================================
// H2 unit 1 — THE INPUT GATE: the adversarial corpus (error goldens)
// ===========================================================================
//
// For EVERY violated prerequisite of the declared class (boolean.hpp)
// one crafted body, one refusal, and the message IS the golden: it
// names the operation, the operand, the error class, the responsible
// entity with its measured evidence, and the remedy. The reverse
// proof is the whole suite above: every valid body in the historical
// corpus passes the gate unchanged (rifiutare troppo è difetto quanto
// accettare troppo).
//
// The craft helpers build bodies through the public topology API
// (vertices, edges, co-edges, radial linking) exactly like
// assemble_body does — keyed edges, canonical direction, the
// reversed flag derived from the traversal — so a crafted body is
// indistinguishable from a factory body except for the defect it
// carries.

namespace {

struct H2FaceSpec {
    // loop 0 is the face's outer loop (the class allows exactly one);
    // vertex indices, CCW around `normal`.
    std::vector<std::vector<std::size_t>> loops;
    Vec3d normal;
};

// Build a body from explicit vertices and face loop tables. Edges are
// keyed (min,max) like assemble_body's; the co-edge's reversed flag is
// derived from the traversal direction (a > b), so two faces listing a
// shared edge in opposite directions produce the orientable pairing
// automatically — and in the SAME direction produce the XOR defect on
// demand.
std::unique_ptr<BRepBody> build_h2_body(
    const std::vector<Vec3d>& pts,
    const std::vector<H2FaceSpec>& face_specs
) {
    auto body = std::make_unique<BRepBody>();
    body->vertices.reserve(pts.size());
    for (std::size_t i = 0; i < pts.size(); ++i) {
        body->vertices.push_back(Vertex{pts[i]});
        body->vertices.back().set_id(i);
    }
    std::map<std::pair<VertexId, VertexId>, EdgeId> edge_map;
    auto edge_for = [&](VertexId a, VertexId b) -> EdgeId {
        const auto key = std::minmax(a, b);
        auto it = edge_map.find({key.first, key.second});
        if (it != edge_map.end()) return it->second;
        const EdgeId eid = body->edges.size();
        body->edges.push_back(Edge{key.first, key.second});
        body->edges.back().set_id(eid);
        edge_map.emplace(std::make_pair(key.first, key.second), eid);
        return eid;
    };
    FaceId fid = 0;
    for (const auto& spec : face_specs) {
        auto surface = std::make_shared<PlaneSurface>(
            pts[spec.loops[0][0]], spec.normal);
        Face face(surface);
        for (const auto& loop : spec.loops) {
            Loop l;
            for (std::size_t i = 0; i < loop.size(); ++i) {
                const VertexId a = loop[i];
                const VertexId b = loop[(i + 1) % loop.size()];
                const EdgeId eid = edge_for(a, b);
                l.add_coedge(CoEdge{eid, a > b});
            }
            face.add_loop(std::move(l));
        }
        face.set_id(fid++);
        body->solid.shell_mut().add_face(std::move(face));
    }
    link_radial_edges(*body);
    return body;
}

// A prism over the CCW (viewed from +z) xy polygon, from z0 to z1.
// Bottom cap normal -z (loop CW in xy), top cap +z (CCW in xy), the
// sides' outward normal (dy, -dx, 0) per CCW edge, each side quad
// wound (a_bot, b_bot, b_top, a_top) — CCW around its normal.
std::unique_ptr<BRepBody> build_h2_prism(
    const std::vector<Vec3d>& poly_ccw, double z0, double z1
) {
    const std::size_t n = poly_ccw.size();
    std::vector<Vec3d> pts;
    pts.reserve(2 * n);
    for (const auto& p : poly_ccw) pts.push_back(Vec3d{p.x, p.y, z0});
    for (const auto& p : poly_ccw) pts.push_back(Vec3d{p.x, p.y, z1});
    std::vector<H2FaceSpec> specs;
    std::vector<std::size_t> top(n), bot(n);
    for (std::size_t i = 0; i < n; ++i) {
        top[i] = n + i;
        bot[i] = i;
    }
    std::vector<std::size_t> bot_rev(bot.rbegin(), bot.rend());
    specs.push_back({{bot_rev}, Vec3d{0, 0, -1}});
    specs.push_back({{top}, Vec3d{0, 0, 1}});
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t a = i, b = (i + 1) % n;
        const double dx = poly_ccw[b].x - poly_ccw[a].x;
        const double dy = poly_ccw[b].y - poly_ccw[a].y;
        const double len = std::sqrt(dx * dx + dy * dy);
        specs.push_back({{{a, b, n + b, n + a}},
                         Vec3d{dy / len, -dx / len, 0}});
    }
    return build_h2_body(pts, specs);
}

// Assert the operation refuses with the named class (and the house's
// actionable shape: the remedy is in the message).
template <typename Fn>
void expect_input_rejected(Fn&& op, const char* cls) {
    bool threw = false;
    try {
        op();
    } catch (const std::invalid_argument& e) {
        threw = true;
        const std::string msg = e.what();
        INFO("refusal message: ", msg);
        CHECK(msg.find(cls) != std::string::npos);
        CHECK(msg.find("Remedy:") != std::string::npos);
        CHECK(msg.find("boolean.hpp") != std::string::npos);
    }
    CHECK_MESSAGE(threw, "expected the operation to refuse with ", cls);
}

} // namespace

TEST_CASE("H2-u1: the gate refuses the empty body by name") {
    auto A = make_box(Vec3d{1, 1, 1});
    BRepBody empty;
    expect_input_rejected([&] { bool_union(empty, *A); },
                          "input_empty_body");
    expect_input_rejected([&] { bool_subtract(*A, empty); },
                          "input_empty_body");
    expect_input_rejected([&] { bool_intersect(empty, empty); },
                          "input_empty_body");
}

TEST_CASE("H2-u1: the gate refuses torn topology references by name") {
    auto A = make_box(Vec3d{1, 1, 1});
    auto B = make_box(Vec3d{1, 1, 1});
    // An edge endpoint pointing past the vertex table.
    B->edges[0].set_vertex(0, 999);
    expect_input_rejected([&] { bool_intersect(*A, *B); },
                          "input_topology_reference");
}

TEST_CASE("H2-u1: the gate refuses non-plane surfaces by name") {
    auto A = make_box(Vec3d{1, 1, 1});
    auto S = make_sphere(1.0);
    expect_input_rejected([&] { bool_intersect(*A, *S); },
                          "input_surface_not_plane");
    // A face with NO surface is the same class (the family is 'none').
    auto B = make_box(Vec3d{1, 1, 1});
    B->solid.shell_mut().faces_mut()[0].set_surface(nullptr);
    expect_input_rejected([&] { bool_union(*A, *B); },
                          "input_surface_not_plane");
}

TEST_CASE("H2-u1: the gate refuses holed faces (inner loops) by name") {
    // A box whose top face carries an extra inner square loop — the
    // holed-face class the old code silently mis-read (loops()[0] only).
    std::vector<Vec3d> pts{
        {-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1},          // top rim
        {-0.5, -0.5, 1}, {0.5, -0.5, 1}, {0.5, 0.5, 1}, {-0.5, 0.5, 1},
        {-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},      // bottom
    };
    std::vector<H2FaceSpec> specs{
        {{{8, 11, 10, 9}}, Vec3d{0, 0, -1}},                     // bottom
        {{{0, 1, 2, 3}, {4, 5, 6, 7}}, Vec3d{0, 0, 1}},          // holed top
        {{{8, 9, 1, 0}}, Vec3d{0, -1, 0}},                       // sides
        {{{9, 10, 2, 1}}, Vec3d{1, 0, 0}},
        {{{10, 11, 3, 2}}, Vec3d{0, 1, 0}},
        {{{11, 8, 0, 3}}, Vec3d{-1, 0, 0}},
    };
    auto holed = build_h2_body(pts, specs);
    auto A = make_box(Vec3d{1, 1, 1});
    expect_input_rejected([&] { bool_subtract(*A, *holed); },
                          "input_face_inner_loops");
}

TEST_CASE("H2-u1: the gate refuses open loops by name") {
    // A proper box whose bottom loop's LAST co-edge has its orientation
    // flag flipped: the walk breaks between co-edge 2 (ends at vertex
    // 4) and co-edge 3 (starts at vertex 7) — the loop does not chain,
    // and no geometry check can save it. Built through the public API:
    // the co-edges are owned by the face's loop, so the corrupted face
    // replaces the original one wholesale.
    std::vector<Vec3d> pts{
        {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
        {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1},
    };
    std::vector<H2FaceSpec> specs{
        {{{3, 2, 1, 0}}, Vec3d{0, 0, -1}},
        {{{4, 5, 6, 7}}, Vec3d{0, 0, 1}},
        {{{0, 4, 5, 1}}, Vec3d{0, -1, 0}},
        {{{1, 5, 6, 2}}, Vec3d{1, 0, 0}},
        {{{2, 6, 7, 3}}, Vec3d{0, 1, 0}},
        {{{3, 7, 4, 0}}, Vec3d{-1, 0, 0}},
    };
    auto broken = build_h2_body(pts, specs);
    // The bottom face is spec 0; its loop walks edges 0..3 with flags
    // (t, t, t, f). Flip the last flag: the chain 0-1-2 breaks at 3.
    Loop corrupted;
    corrupted.add_coedge(CoEdge{0, true});
    corrupted.add_coedge(CoEdge{1, true});
    corrupted.add_coedge(CoEdge{2, true});
    corrupted.add_coedge(CoEdge{3, true});  // flipped: should be false
    Face face(std::make_shared<PlaneSurface>(pts[3], Vec3d{0, 0, -1}));
    face.add_loop(std::move(corrupted));
    face.set_id(0);
    broken->solid.shell_mut().faces_mut()[0] = std::move(face);
    link_radial_edges(*broken);
    auto A = make_box(Vec3d{1, 1, 1});
    expect_input_rejected([&] { bool_union(*A, *broken); },
                          "input_loop_open");
}

TEST_CASE("H2-u1: the gate refuses open shells (1-coedge edges) by name") {
    // A box without its top face: the rim edges carry one co-edge.
    std::vector<Vec3d> pts{
        {-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1},
        {-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
    };
    std::vector<H2FaceSpec> specs{
        {{{7, 6, 5, 4}}, Vec3d{0, 0, -1}},                       // bottom
        {{{0, 4, 5, 1}}, Vec3d{0, -1, 0}},                       // sides
        {{{1, 5, 6, 2}}, Vec3d{1, 0, 0}},
        {{{2, 6, 7, 3}}, Vec3d{0, 1, 0}},
        {{{3, 7, 4, 0}}, Vec3d{-1, 0, 0}},
    };
    auto open = build_h2_body(pts, specs);
    auto A = make_box(Vec3d{1, 1, 1});
    expect_input_rejected([&] { bool_intersect(*A, *open); },
                          "input_edge_not_manifold");
}

TEST_CASE("H2-u1: the gate refuses the orientability XOR violation by name") {
    // Two triangles glued back-to-back, both wound the same way: every
    // edge has 2 co-edges with the SAME orientation (ADR-0022 §5
    // invariant 2's classic defect).
    std::vector<Vec3d> pts{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
    std::vector<H2FaceSpec> specs{
        {{{0, 1, 2}}, Vec3d{0, 0, 1}},
        {{{0, 1, 2}}, Vec3d{0, 0, -1}},
    };
    auto folded = build_h2_body(pts, specs);
    auto A = make_box(Vec3d{1, 1, 1});
    expect_input_rejected([&] { bool_subtract(*A, *folded); },
                          "input_coedge_orientation");
}

TEST_CASE("H2-u1: the gate refuses off-plane loop vertices by name") {
    auto A = make_box(Vec3d{1, 1, 1});
    auto B = make_box(Vec3d{1, 1, 1});
    // Drag one vertex out of three of its faces' planes.
    B->vertices[0].set_position(B->vertices[0].position()
                                + Vec3d{0.1, 0.1, 0.1});
    expect_input_rejected([&] { bool_intersect(*A, *B); },
                          "input_loop_off_plane");
}

TEST_CASE("H2-u1: the gate refuses inward face normals by name") {
    // A full box authored with every normal pointing INTO the solid:
    // the divergence volume with the stored normals is negative.
    std::vector<Vec3d> pts{
        {-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1},
        {-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
    };
    std::vector<H2FaceSpec> specs{
        {{{7, 6, 5, 4}}, Vec3d{0, 0, 1}},    // bottom, normal up: inward
        {{{0, 1, 2, 3}}, Vec3d{0, 0, -1}},   // top, normal down: inward
        {{{0, 4, 5, 1}}, Vec3d{0, 1, 0}},
        {{{1, 5, 6, 2}}, Vec3d{-1, 0, 0}},
        {{{2, 6, 7, 3}}, Vec3d{0, -1, 0}},
        {{{3, 7, 4, 0}}, Vec3d{1, 0, 0}},
    };
    auto inside_out = build_h2_body(pts, specs);
    auto A = make_box(Vec3d{1, 1, 1});
    expect_input_rejected([&] { bool_union(*A, *inside_out); },
                          "input_face_orientation");
}

TEST_CASE("H2-u1: the gate refuses zero-volume (flat) operands by name") {
    // Two coplanar squares glued along all three edges — a closed
    // orientable "body" with zero volume.
    std::vector<Vec3d> pts{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
    std::vector<H2FaceSpec> specs{
        {{{0, 1, 2}}, Vec3d{0, 0, 1}},
        {{{0, 2, 1}}, Vec3d{0, 0, -1}},
    };
    auto flat = build_h2_body(pts, specs);
    auto A = make_box(Vec3d{1, 1, 1});
    expect_input_rejected([&] { bool_subtract(*A, *flat); },
                          "input_zero_volume");
}

TEST_CASE("H2-u1: the gate refuses CW loop winding by name") {
    // A box whose top face's normal is flipped while its loop stays
    // CCW-around-+z: the divergence volume stays positive (one face's
    // contribution is small against the whole), the loop reads CW
    // around its own normal.
    std::vector<Vec3d> pts{
        {-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1},
        {-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
    };
    std::vector<H2FaceSpec> specs{
        {{{7, 6, 5, 4}}, Vec3d{0, 0, -1}},
        {{{0, 1, 2, 3}}, Vec3d{0, 0, -1}},   // top with a flipped normal
        {{{0, 4, 5, 1}}, Vec3d{0, -1, 0}},
        {{{1, 5, 6, 2}}, Vec3d{1, 0, 0}},
        {{{2, 6, 7, 3}}, Vec3d{0, 1, 0}},
        {{{3, 7, 4, 0}}, Vec3d{-1, 0, 0}},
    };
    auto flipped = build_h2_body(pts, specs);
    auto A = make_box(Vec3d{1, 1, 1});
    expect_input_rejected([&] { bool_intersect(*A, *flipped); },
                          "input_loop_winding");
}

TEST_CASE("H2-u1: the gate refuses non-convex FACES by name") {
    // The L-prism: closed, manifold, planar, outward — and the L caps
    // carry a reflex vertex.
    const std::vector<Vec3d> L{
        {0, 0, 0}, {2, 0, 0}, {2, 1, 0}, {1, 1, 0}, {1, 2, 0}, {0, 2, 0},
    };
    auto lprism = build_h2_prism(L, 0.0, 1.0);
    auto A = make_box(Vec3d{1, 1, 1});
    expect_input_rejected([&] { bool_union(*A, *lprism); },
                          "input_face_not_convex");
}

TEST_CASE("H2-u1: the gate refuses the non-convex POLYHEDRON by name") {
    // Every face a convex polygon (the L split into a convex pentagon
    // and a convex quad on each cap) — but the SOLID is not the
    // intersection of its half-spaces: the arm's back wall has the
    // bar's far corners beyond it.
    std::vector<Vec3d> pts{
        {0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {2, 1, 0},
        {1, 1, 0}, {1, 2, 0}, {0, 2, 0},              // bottom (z=0)
        {0, 0, 1}, {1, 0, 1}, {2, 0, 1}, {2, 1, 1},
        {1, 1, 1}, {1, 2, 1}, {0, 2, 1},              // top (z=1)
    };
    std::vector<H2FaceSpec> specs{
        // bottom caps (normal -z, loops CW in xy)
        {{{6, 5, 4, 1, 0}}, Vec3d{0, 0, -1}},   // bar pentagon
        {{{4, 3, 2, 1}}, Vec3d{0, 0, -1}},      // arm quad
        // top caps (+z, CCW in xy)
        {{{7, 8, 11, 12, 13}}, Vec3d{0, 0, 1}},
        {{{8, 9, 10, 11}}, Vec3d{0, 0, 1}},
        // sides, CCW boundary walk
        {{{0, 1, 8, 7}}, Vec3d{0, -1, 0}},      // y=0 (bar)
        {{{1, 2, 9, 8}}, Vec3d{0, -1, 0}},      // y=0 (arm)
        {{{2, 3, 10, 9}}, Vec3d{1, 0, 0}},      // x=2
        {{{3, 4, 11, 10}}, Vec3d{0, 1, 0}},     // y=1 (arm back)
        {{{4, 5, 12, 11}}, Vec3d{1, 0, 0}},     // x=1 (notch wall)
        {{{5, 6, 13, 12}}, Vec3d{0, 1, 0}},     // y=2
        {{{6, 0, 7, 13}}, Vec3d{-1, 0, 0}},     // x=0
    };
    auto notchy = build_h2_body(pts, specs);
    auto A = make_box(Vec3d{1, 1, 1});
    expect_input_rejected([&] { bool_subtract(*A, *notchy); },
                          "input_not_convex");
}

TEST_CASE("H2-u1: the reverse proof — the gate accepts exactly the declared class") {
    // The factory corpus (the same bodies every golden above and below
    // is built from) passes the gate: the operations run and answer.
    auto A = make_box(Vec3d{1, 1, 1});
    auto B = make_box(Vec3d{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3d{0.5f, 0, 0});
    }
    CHECK(bool_subtract(*A, *B) != nullptr);       // closed solid answer
    auto C = make_box(Vec3d{1, 1, 1});
    for (auto& v : C->vertices) {
        v.set_position(v.position() + Vec3d{1.0f, 1.0f, 1.0f});
    }
    auto cut = bool_subtract(*A, *C);
    REQUIRE(cut != nullptr);
    CHECK(cut->vertex_count() == 17);
    // The tetrahedron (triangles, not quads) is in the class too.
    auto T = make_tetrahedron();
    auto cube = make_box(Vec3d{1, 1, 1});
    CHECK(bool_intersect(*T, *cube) != nullptr);
}
