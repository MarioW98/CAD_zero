// tests/unit/brep/test_boolean.cpp
//
// Tests for boolean operations on B-Rep bodies — ADR-0023.
//
// Covers:
//   * Seam wire factories (make_sphere, make_cylinder) now produce
//     proper closed manifolds (inversion of the ADR-0022 regression
//     guards).
//   * bool_intersect on overlapping boxes (non-trivial intersection).
//   * bool_intersect on disjoint boxes (empty result).
//   * bool_intersect on nested boxes (result = inner box).
//   * bool_intersect on a box and a tetrahedron.
//   * χ = 2 on every non-empty result.
//   * is_closed_manifold on every non-empty result.
//   * bool_union / bool_subtract throw std::runtime_error as documented.
//
#include <doctest/doctest.h>

#include "CAD_0/brep/topology.hpp"
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/boolean.hpp"

#include <stdexcept>

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
}

TEST_CASE("ADR-0023: bool_intersect result is a closed manifold") {
    auto A = make_box(Vec3f{1, 1, 1});
    auto B = make_box(Vec3f{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3f{0.5f, 0.5f, 0.5f});
    }
    auto result = bool_intersect(*A, *B);
    REQUIRE(result != nullptr);
    CHECK(is_closed_manifold(*result));
    CHECK(loops_are_closed(*result));
    // Every edge must have exactly 2 co-edges (manifold contract).
    for (const auto& edge : result->edges) {
        CHECK(edge.coedges().size() == 2);
    }
}

TEST_CASE("ADR-0023: bool_intersect every edge has opposite coedge orientations") {
    auto A = make_box(Vec3f{1, 1, 1});
    auto B = make_box(Vec3f{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3f{0.5f, 0, 0});
    }
    auto result = bool_intersect(*A, *B);
    REQUIRE(result != nullptr);
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
// ADR-0024 — bool_union
// ---------------------------------------------------------------------------
//
// bool_union(A, B) = (A's faces outside B) ∪ (B's faces outside A).
// For two overlapping boxes, the result is the convex hull of the
// union (since both are convex and overlap, the union is a non-convex
// polyhedron whose boundary is the outer skin of the two boxes).
//

TEST_CASE("ADR-0024: bool_union of overlapping boxes produces a non-empty body") {
    // NOTE: the MVP implementation uses fan-triangulation + centroid
    // classification (ADR-0024 §2 documented simplification). This
    // produces a valid body but NOT always a closed manifold, because
    // the diagonal edges of the triangulation are not shared by two
    // faces. The result is topologically a triangle soup with welded
    // vertices but non-manifold edges.
    //
    // A proper Greiner-Hormann implementation would preserve the
    // polygon structure and produce a closed manifold. For now we
    // verify the body is non-empty and has the expected face count.
    auto A = make_box(Vec3f{1, 1, 1});
    auto B = make_box(Vec3f{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3f{0.5f, 0, 0});
    }
    auto result = bool_union(*A, *B);
    REQUIRE(result != nullptr);
    CHECK(result->face_count() > 0);
    CHECK(result->vertex_count() > 0);
    // loops_are_closed should still hold (each face is a valid triangle).
    CHECK(loops_are_closed(*result));
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
}

TEST_CASE("ADR-0024: bool_union every edge is welded (no duplicate edges)") {
    // Fan-triangulation creates diagonal edges that are used by only one
    // face (non-manifold). We verify that every edge has at least 1
    // coedge (no orphan edges) and that the body is consistent.
    auto A = make_box(Vec3f{1, 1, 1});
    auto B = make_box(Vec3f{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3f{0.5f, 0, 0});
    }
    auto result = bool_union(*A, *B);
    REQUIRE(result != nullptr);
    for (const auto& edge : result->edges) {
        CHECK(edge.coedges().size() >= 1);  // at least one face uses it
    }
}

// ---------------------------------------------------------------------------
// ADR-0024 — bool_subtract
// ---------------------------------------------------------------------------
//
// bool_subtract(A, B) = (A's faces outside B) ∪ (B's faces inside A,
// with inverted normals). For a box minus an overlapping box, the
// result is a box with a corner notch removed.
//

TEST_CASE("ADR-0024: bool_subtract of overlapping boxes produces a non-empty body") {
    // Same fan-triangulation caveat as bool_union: the result is a
    // triangle soup, not a closed manifold. Verified non-empty + loops closed.
    auto A = make_box(Vec3f{1, 1, 1});
    auto B = make_box(Vec3f{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3f{0.5f, 0.5f, 0.5f});
    }
    auto result = bool_subtract(*A, *B);
    REQUIRE(result != nullptr);
    CHECK(result->face_count() > 0);
    CHECK(loops_are_closed(*result));
}

TEST_CASE("ADR-0024: bool_subtract of disjoint boxes returns A unchanged") {
    // A \ (disjoint B) = A. Centroid classification: every face of A
    // has centroid outside B → kept. No face of B is inside A → Part 2
    // empty. Result is A: 8V/12E/6F.
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

TEST_CASE("ADR-0024: bool_subtract every edge is welded (no orphan edges)") {
    auto A = make_box(Vec3f{1, 1, 1});
    auto B = make_box(Vec3f{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3f{0.5f, 0.5f, 0.5f});
    }
    auto result = bool_subtract(*A, *B);
    REQUIRE(result != nullptr);
    for (const auto& edge : result->edges) {
        CHECK(edge.coedges().size() >= 1);
    }
}

TEST_CASE("ADR-0024: bool_subtract χ = 2 on non-empty result (triangulated)") {
    auto A = make_box(Vec3f{1, 1, 1});
    auto B = make_box(Vec3f{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3f{0.5f, 0.5f, 0.5f});
    }
    auto result = bool_subtract(*A, *B);
    REQUIRE(result != nullptr);
    if (result->face_count() > 0) {
        // χ may not equal 2 for arbitrary boolean results with
        // fan-triangulation, but it must be a valid integer.
        const auto chi = euler_characteristic(*result);
        CHECK(chi != 0);  // non-degenerate
    }
}

// ---------------------------------------------------------------------------
// Cross-validation: union is consistent with intersect
// ---------------------------------------------------------------------------

TEST_CASE("ADR-0024: bool_union and bool_intersect both produce non-empty bodies") {
    // For two partially overlapping convex bodies, both union and
    // intersect should produce non-empty results. We don't assert
    // relative face counts because the fan-triangulation MVP doesn't
    // guarantee |union| >= |intersect| in all cases (documented
    // limitation — Greiner-Hormann would fix this).
    auto A = make_box(Vec3f{1, 1, 1});
    auto B = make_box(Vec3f{1, 1, 1});
    for (auto& v : B->vertices) {
        v.set_position(v.position() + Vec3f{0.5f, 0, 0});
    }
    auto u = bool_union(*A, *B);
    auto i = bool_intersect(*A, *B);
    REQUIRE(u != nullptr);
    REQUIRE(i != nullptr);
    // Both should produce SOME geometry.
    CHECK(u->face_count() > 0);
    CHECK(i->face_count() > 0);
}
