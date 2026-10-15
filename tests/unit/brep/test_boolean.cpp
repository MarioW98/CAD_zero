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
//   * Degenerate zero-volume results and the bool_union partial
//     coplanar-overlap limitation are asserted exactly as documented
//     (ADR-0025 v5 Consequences — known limitation).
//
#include <doctest/doctest.h>

#include "CAD_0/brep/topology.hpp"
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/boolean.hpp"
// Test-only seam — NOT part of the public kernel API (see header).
#include "CAD_0/brep/detail/boolean_test_hooks.hpp"

#include <cmath>
#include <stdexcept>
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
    auto A = make_box(Vec3f{1, 1, 1});
    auto B = make_box(Vec3f{1, 1, 1});
    // Translate B by (0.5, 0, 0): shift all vertices.
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3f{0.5f, 0, 0});
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
    auto A = make_box(Vec3f{1, 1, 1});
    auto B = make_box(Vec3f{1, 1, 1});
    // Move B far away — no overlap.
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3f{10, 0, 0});
    }
    auto result = bool_intersect(*A, *B);
    REQUIRE(result != nullptr);
    CHECK(result->face_count() == 0);
    CHECK_FALSE(is_closed_manifold(*result));  // empty body, not a manifold
}

TEST_CASE("ADR-0023: bool_intersect of nested boxes returns the inner box") {
    // A = big box (half-extents 2), B = small box (half-extents 1) inside A.
    auto A = make_box(Vec3f{2, 2, 2});
    auto B = make_box(Vec3f{1, 1, 1});  // centered at origin, inside A
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
    auto A = make_box(Vec3f{1, 1, 1});
    auto B = make_box(Vec3f{1, 1, 1});
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
    auto A = make_box(Vec3f{1, 1, 1});
    auto B = make_box(Vec3f{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3f{0.5f, 0.5f, 0.5f});
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
    auto A = make_box(Vec3f{1, 1, 1});
    auto B = make_box(Vec3f{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3f{0.5f, 0, 0});
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
    auto A = make_box(Vec3f{1, 1, 1});
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
    auto A = make_box(Vec3f{1, 1, 1});
    auto B = make_box(Vec3f{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3f{2.0f, 0, 0});
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
    auto A = make_box(Vec3f{1, 1, 1});
    auto B = make_box(Vec3f{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3f{0, 0, 2.0f});
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
    auto A = make_box(Vec3f{1, 1, 1});
    auto B = make_box(Vec3f{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3f{0, 0, 2.0f});
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
    auto A = make_box(Vec3f{1, 1, 1});
    auto B = make_box(Vec3f{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3f{1.0f, 1.0f, 1.0f});
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
    auto A = make_box(Vec3f{1, 1, 1});
    auto B = make_box(Vec3f{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3f{10, 0, 0});
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
    auto A = make_box(Vec3f{1, 1, 1});
    auto B = make_box(Vec3f{3, 3, 3});  // A is entirely inside B
    auto result = bool_subtract(*A, *B);
    REQUIRE(result != nullptr);
    CHECK(result->face_count() == 0);
    CHECK_FALSE(is_closed_manifold(*result));  // empty body
}

TEST_CASE("ADR-0025: bool_subtract of identical boxes returns empty") {
    // A == B → bodies_identical shortcut triggers → empty body.
    auto A = make_box(Vec3f{1, 1, 1});
    auto B = make_box(Vec3f{1, 1, 1});
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
    auto A = make_box(Vec3f{1, 1, 1});
    auto B = make_box(Vec3f{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3f{0.5f, 0, 0});
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
    auto A = make_box(Vec3f{1, 1, 1});
    auto B = make_box(Vec3f{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3f{0.3f, 0, 0});
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
    auto A = make_box(Vec3f{1, 1, 1});
    auto B = make_box(Vec3f{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3f{0.5f, 0, 0});
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
    const std::vector<Vec3f> subject = {
        Vec3f{ 1.0f, 1.0f, -1.0f}, Vec3f{-1.0f, 1.0f, -1.0f},
        Vec3f{-1.0f, 1.0f,  1.0f}, Vec3f{ 1.0f, 1.0f,  1.0f}};
    const std::vector<Vec3f> clip = {
        Vec3f{ 1.0f, 1.0f, -1.0f}, Vec3f{-0.7f, 1.0f, -1.0f},
        Vec3f{-0.7f, 1.0f,  1.0f}, Vec3f{ 1.0f, 1.0f,  1.0f}};
    const auto frags = polygon_subtract_fragments_for_tests(
        subject, clip, Vec3f{0.0f, 1.0f, 0.0f});
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
    auto A = make_box(Vec3f{3, 3, 3});
    auto B = make_box(Vec3f{1, 1, 1});
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
    auto A = make_box(Vec3f{1, 1, 1});
    auto B = make_box(Vec3f{1, 1, 1});
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
    auto A = make_box(Vec3f{1, 1, 1});
    auto B = make_box(Vec3f{1, 1, 1});
    auto result = bool_subtract(*A, *B);
    REQUIRE(result != nullptr);
    CHECK(result->face_count() == 0);
    CHECK_FALSE(is_closed_manifold(*result));  // empty body
}

TEST_CASE("ADR-0025: bool_intersect of identical boxes returns one box (regression)") {
    auto A = make_box(Vec3f{1, 1, 1});
    auto B = make_box(Vec3f{1, 1, 1});
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
// ADR-0024 — bool_union
// ---------------------------------------------------------------------------
//
// bool_union(A, B) = (A's faces outside B) ∪ (B's faces outside A).
// When the two boxes share full lateral faces (partial coplanar
// overlap), each side's lateral faces are clipped against the other
// solid, leaving a gap over the overlap region: the result is an open
// shell — the documented ADR-0025 v5 known limitation.
//

TEST_CASE("ADR-0024: bool_union of overlapping boxes — open shell, partial coplanar overlap (ADR-0025 v5 known limitation)") {
    // Box A: [-1,1]³
    // Box B: [-0.5, 1.5] × [-1, 1] × [-1, 1] (overlaps +X half of A;
    //        the ±Y and ±Z faces are coplanar with A's)
    // A ∪ B = [-1,1.5] × [-1,1] × [-1,1], but with partial coplanar
    // overlap the union keeps only A's faces left of x=-0.5 and B's
    // faces right of x=1 (ADR-0025 v5 Consequences — known limitation).
    // The result is two open half-shells:
    //   A-part: box [-1,-0.5]×[-1,1]² without the x=-0.5 face → 8V/12E/5F
    //   B-part: box [1,1.5]×[-1,1]²   without the x=1     face → 8V/12E/5F
    //   V = 8+8 = 16, E = 12+12 = 24, F = 5+5 = 10, χ = 16−24+10 = 2,
    //   loops closed; NOT a closed manifold: the 8 rim edges (4 at
    //   x=-0.5, 4 at x=1) carry a single coedge each.
    auto A = make_box(Vec3f{1, 1, 1});
    auto B = make_box(Vec3f{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3f{0.5f, 0, 0});
    }
    auto result = bool_union(*A, *B);
    REQUIRE(result != nullptr);
    CHECK(result->vertex_count() == 16);
    CHECK(result->edge_count()   == 24);
    CHECK(result->face_count()   == 10);
    CHECK(euler_characteristic(*result) == 2);
    CHECK(loops_are_closed(*result));
    CHECK_FALSE(is_closed_manifold(*result));
}

TEST_CASE("ADR-0024: bool_union of identical boxes returns A (identical-body shortcut)") {
    // ADR-0025: bodies_identical() shortcut detects A == B and returns
    // a copy of A. The result is 8V/12E/6F with χ = 2 and is_closed_manifold.
    auto A = make_box(Vec3f{1, 1, 1});
    auto B = make_box(Vec3f{1, 1, 1});  // same position
    auto result = bool_union(*A, *B);
    REQUIRE(result != nullptr);
    CHECK(result->vertex_count() == 8);
    CHECK(result->edge_count()   == 12);
    CHECK(result->face_count()   == 6);
    CHECK(euler_characteristic(*result) == 2);
    CHECK(is_closed_manifold(*result));
    CHECK(loops_are_closed(*result));
}

TEST_CASE("ADR-0024: bool_union of overlapping boxes — exact coedge census (open shell)") {
    // Same input as the open-shell union test above: 24 edges — the 16
    // edges of the two closed half-shell "rings" carry exactly 2
    // coedges, the 8 open-rim edges (4 at x=-0.5, 4 at x=1) carry
    // exactly 1. No edge is an orphan and none exceeds 2 coedges.
    auto A = make_box(Vec3f{1, 1, 1});
    auto B = make_box(Vec3f{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3f{0.5f, 0, 0});
    }
    auto result = bool_union(*A, *B);
    REQUIRE(result != nullptr);
    REQUIRE(result->edge_count() == 24);
    std::size_t boundary = 0, interior = 0;
    for (const auto& edge : result->edges) {
        REQUIRE(edge.coedges().size() <= 2);
        if (edge.coedges().size() == 1) ++boundary;
        else ++interior;
    }
    CHECK(boundary == 8);
    CHECK(interior == 16);
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
    auto A = make_box(Vec3f{1, 1, 1});
    auto B = make_box(Vec3f{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3f{0.5f, 0.5f, 0.5f});
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
    auto A = make_box(Vec3f{1, 1, 1});
    auto B = make_box(Vec3f{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3f{10, 0, 0});
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
    auto A = make_box(Vec3f{1, 1, 1});
    auto B = make_box(Vec3f{3, 3, 3});  // A is entirely inside B
    auto result = bool_subtract(*A, *B);
    REQUIRE(result != nullptr);
    CHECK(result->face_count() == 0);
    CHECK_FALSE(is_closed_manifold(*result));  // empty body
}

TEST_CASE("ADR-0024: bool_subtract of overlapping boxes — every edge has exactly 2 coedges") {
    // Corner-notch result (17V/27E/12F): the T-junction subdivision
    // makes every edge match pairwise, so all 27 edges carry exactly
    // 2 coedges (closed-manifold contract).
    auto A = make_box(Vec3f{1, 1, 1});
    auto B = make_box(Vec3f{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3f{0.5f, 0.5f, 0.5f});
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
    auto A = make_box(Vec3f{1, 1, 1});
    auto B = make_box(Vec3f{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3f{0.5f, 0.5f, 0.5f});
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
    //   union = the open shell of the partial coplanar overlap
    //           (ADR-0025 v5 known limitation): 16V/24E/10F, χ=2,
    //           loops closed, not a closed manifold.
    auto A = make_box(Vec3f{1, 1, 1});
    auto B = make_box(Vec3f{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3f{0.5f, 0, 0});
    }
    auto u = bool_union(*A, *B);
    auto i = bool_intersect(*A, *B);
    REQUIRE(u != nullptr);
    REQUIRE(i != nullptr);
    CHECK(i->vertex_count() == 8);
    CHECK(i->edge_count()   == 12);
    CHECK(i->face_count()   == 6);
    CHECK(euler_characteristic(*i) == 2);
    CHECK(loops_are_closed(*i));
    CHECK(is_closed_manifold(*i));
    CHECK(u->vertex_count() == 16);
    CHECK(u->edge_count()   == 24);
    CHECK(u->face_count()   == 10);
    CHECK(euler_characteristic(*u) == 2);
    CHECK(loops_are_closed(*u));
    CHECK_FALSE(is_closed_manifold(*u));
}
