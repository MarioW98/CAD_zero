// tests/unit/bridge/test_bridge.cpp
//
// Tests for the SDF ↔ B-Rep bridge.
//
#include <doctest/doctest.h>

#include "CAD_0/bridge/sdf_to_brep.hpp"
#include "CAD_0/bridge/brep_to_sdf.hpp"
#include "CAD_0/sdf/primitives.hpp"
#include "CAD_0/sdf/operators.hpp"
#include "CAD_0/sdf/transforms.hpp"
#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/topology.hpp"

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <algorithm>
#include <string>
#include <vector>

#include "CAD_0/math/quat.hpp"

using namespace CAD_0::bridge;
using CAD_0::math::Vec3f;

// Use explicit namespace to disambiguate make_sphere/make_box/make_cylinder
// (they exist in both CAD_0::sdf and CAD_0::brep)
namespace s = CAD_0::sdf;
namespace b = CAD_0::brep;

TEST_CASE("Bridge: SDF sphere → B-Rep sphere (primitive fit)") {
    auto sph = s::make_sphere(2.0f);
    SdfToBrepOptions opts;
    opts.fit_primitives = true;
    auto body = sdf_to_brep(sph, opts);

    REQUIRE(body != nullptr);
    // Analytic seam wire (ADR-0023): a sphere is ONE face with a
    // degenerate seam — 2 vertices, 1 edge (the pole-to-pole seam),
    // 1 face. Pinned, not just face_count() > 0.
    CHECK(body->vertex_count() == 2);
    CHECK(body->edge_count() == 1);
    CHECK(body->face_count() == 1);
    CHECK(std::string(body->solid.shell().faces()[0].surface()->type_name()) == "sphere");
}

TEST_CASE("Bridge: SDF box → B-Rep box (primitive fit)") {
    auto box = s::make_box(Vec3f{1, 1, 1});
    SdfToBrepOptions opts;
    opts.fit_primitives = true;
    auto body = sdf_to_brep(box, opts);

    REQUIRE(body != nullptr);
    CHECK(body->vertex_count() == 8);
    CHECK(body->edge_count() == 12);
    CHECK(body->face_count() == 6);
}

TEST_CASE("Bridge: SDF cylinder → B-Rep cylinder (primitive fit)") {
    auto cyl = s::make_cylinder(1.0f, 2.0f);
    SdfToBrepOptions opts;
    opts.fit_primitives = true;
    auto body = sdf_to_brep(cyl, opts);

    REQUIRE(body != nullptr);
    // Analytic seam wires (ADR-0023): lateral face + 2 caps —
    // 4 vertices, 5 edges, 3 faces. Pinned.
    CHECK(body->vertex_count() == 4);
    CHECK(body->edge_count() == 5);
    CHECK(body->face_count() == 3);
    CHECK(std::string(body->solid.shell().faces()[0].surface()->type_name()) == "cylinder");
}

TEST_CASE("Bridge: SDF sphere → B-Rep faceted (no primitive fit)") {
    auto sph = s::make_sphere(1.0f);
    SdfToBrepOptions opts;
    opts.fit_primitives = false;
    opts.march_resolution = 8;
    auto body = sdf_to_brep(sph, opts);

    REQUIRE(body != nullptr);
    // Same configuration as contract test T2 (sphere r=1 centered,
    // res=8): pinned counts measured once and frozen (ADR-0025 v5),
    // plus the grid-independent contract — not face_count() > 0.
    CHECK(body->vertex_count() == 270);
    CHECK(body->edge_count() == 804);
    CHECK(body->face_count() == 536);
    CHECK(b::euler_characteristic(*body) == 2);
    CHECK(b::is_closed_manifold(*body));
    CHECK(b::loops_are_closed(*body));
    for (const auto& face : body->solid.shell().faces()) {
        CHECK(std::string(face.surface()->type_name()) == "plane");
    }
}

TEST_CASE("Bridge: SDF smooth_union → B-Rep faceted") {
    auto a = s::make_sphere(1.0f);
    auto sdf_b = s::sdf_translate(s::make_sphere(1.0f), {2, 0, 0});
    auto u = s::sdf_smooth_union(std::move(a), std::move(sdf_b), 0.5f);

    SdfToBrepOptions opts;
    opts.fit_primitives = true;
    opts.march_resolution = 8;
    auto body = sdf_to_brep(u, opts);

    REQUIRE(body != nullptr);
    // Pinned (two unit spheres, centers (0,0,0)/(2,0,0) — exactly
    // tangent — k=0.5, res=8), measured once on the FIXED SmoothMinSDF
    // (2026-10-16 clamp-sign fix) and frozen. Counts satisfy the
    // closed-mesh identities (E = 3F/2, V = F/2 + 2). Note: with the
    // pre-fix operator this configuration extracted a different (much
    // coarser) shape — the pin also guards the operator through the
    // bridge. The primitive-fit dispatch falls through to faceted for
    // an smin root (see T10).
    CHECK(body->vertex_count() == 350);
    CHECK(body->edge_count() == 1044);
    CHECK(body->face_count() == 696);
    CHECK(b::euler_characteristic(*body) == 2);
    CHECK(b::is_closed_manifold(*body));
    CHECK(b::loops_are_closed(*body));
    for (const auto& face : body->solid.shell().faces()) {
        CHECK(std::string(face.surface()->type_name()) == "plane");
    }
}

TEST_CASE("Bridge: B-Rep sphere → SDF sphere") {
    auto body = b::make_sphere(2.0f);
    auto sdf_body = brep_to_sdf(*body);

    REQUIRE(sdf_body != nullptr);
    // brep_to_sdf detects the analytic sphere face and returns the
    // EXACT primitive SDF (not a mesh approximation) — so the values
    // are hand-derived and bit-exact: f(p) = |p| - 2. sqrt of exact
    // squares is exact here (0, 9, and the 1.9 roundtrip), so == is
    // sound, no epsilon.
    CHECK(sdf_body->value(Vec3f{0, 0, 0}) == -2.0f);
    CHECK(sdf_body->value(Vec3f{3, 0, 0}) == 1.0f);
    CHECK(sdf_body->value(Vec3f{0, 1.9f, 0}) == -(2.0f - 1.9f));
}

TEST_CASE("Bridge: B-Rep box → SDF box") {
    auto body = b::make_box(Vec3f{1, 1, 1});
    auto sdf_body = brep_to_sdf(*body);

    REQUIRE(sdf_body != nullptr);
    // Box detection (6 plane faces, 8V/12E) returns the EXACT box
    // primitive SDF with half-extents from the body bounds — the
    // values are hand-derived and bit-exact (no sqrt on these paths):
    // center → -1 (nearest wall), (5,0,0) → +4, the (0.5,0.5,0.5)
    // corner direction → -0.5 (nearest wall at distance 0.5).
    CHECK(sdf_body->value(Vec3f{0, 0, 0}) == -1.0f);
    CHECK(sdf_body->value(Vec3f{5, 0, 0}) == 4.0f);
    CHECK(sdf_body->value(Vec3f{0.5f, 0.5f, 0.5f}) == -0.5f);
}

TEST_CASE("Bridge: B-Rep cylinder → SDF cylinder") {
    auto body = b::make_cylinder(1.0f, 4.0f);
    auto sdf_body = brep_to_sdf(*body);

    REQUIRE(sdf_body != nullptr);
    // Cylinder detection (3 faces: cylinder + 2 caps) returns the
    // EXACT CylinderSDF (radius from the face, height from the
    // surface). Hand-derived, bit-exact — note the axis is Y and the
    // distance is 2D on (rho, |y|):
    //   center → max(-1 radial, -2 axial) = -1
    //   (5,0,0) → rho - r = 4 (outside, axial inside → no sqrt mixing)
    //   (0,0,1.9) → rho = sqrt(1.9^2) = 1.9 (roundtrip), radial
    //     outside distance 1.9 - 1
    CHECK(sdf_body->value(Vec3f{0, 0, 0}) == -1.0f);
    CHECK(sdf_body->value(Vec3f{5, 0, 0}) == 4.0f);
    CHECK(sdf_body->value(Vec3f{0, 0, 1.9f}) == 1.9f - 1.0f);
}

TEST_CASE("Bridge: round-trip sphere SDF → B-Rep → SDF") {
    auto original = s::make_sphere(2.0f);
    auto body = sdf_to_brep(original);
    auto roundtrip = brep_to_sdf(*body);

    REQUIRE(roundtrip != nullptr);
    // Default options fit the primitive first, so the round-trip is
    // analytic end-to-end: EXACT values, hand-derived (f = |p| - 2).
    CHECK(roundtrip->value(Vec3f{0, 0, 0}) == -2.0f);
    CHECK(roundtrip->value(Vec3f{3, 0, 0}) == 1.0f);
    CHECK(roundtrip->value(Vec3f{0, 1.9f, 0}) == -(2.0f - 1.9f));
}

TEST_CASE("Bridge: round-trip box SDF → B-Rep → SDF") {
    auto original = s::make_box(Vec3f{1, 1, 1});
    auto body = sdf_to_brep(original);
    auto roundtrip = brep_to_sdf(*body);

    REQUIRE(roundtrip != nullptr);
    // Analytic round-trip through the box primitive fit: EXACT values,
    // hand-derived (center → -1, (5,0,0) → +4).
    CHECK(roundtrip->value(Vec3f{0, 0, 0}) == -1.0f);
    CHECK(roundtrip->value(Vec3f{5, 0, 0}) == 4.0f);
}

TEST_CASE("Bridge: faceted B-Rep → SDF mesh") {
    auto a = s::make_sphere(1.0f);
    auto sdf_b = s::sdf_translate(s::make_sphere(1.0f), {2, 0, 0});
    auto u = s::sdf_smooth_union(std::move(a), std::move(sdf_b), 0.5f);

    SdfToBrepOptions opts;
    opts.fit_primitives = true;
    opts.march_resolution = 8;
    auto body = sdf_to_brep(u, opts);

    REQUIRE(body != nullptr);
    // The extraction itself is the pinned contract of the smooth-union
    // legacy test above (350/1044/696, measured on the fixed smin).
    CHECK(body->vertex_count() == 350);
    CHECK(body->edge_count() == 1044);
    CHECK(body->face_count() == 696);

    auto sdf_body = brep_to_sdf(*body);
    REQUIRE(sdf_body != nullptr);

    // No primitive matches a faceted body, so brep_to_sdf falls back
    // to tessellate_body + MeshSDF. WHY NOT EXACT PINS HERE (documented
    // per the sweep's rule): the fallback tessellates EVERY planar face
    // over a default UV domain, which INFLATES the mesh far beyond the
    // body (measured: 356,352 triangles, bounds ~ ±10-15 on a body of
    // extent ~4) and makes the majority-vote sign unreliable at
    // moderate outside distances (e.g. (5,0,0) reads as inside). That
    // is pre-existing behavior of the documented-stub inverse bridge —
    // not something this sweep changes. The strongest HONEST assertions
    // are therefore the ones below: deep-inside and waist signs (the
    // majority vote is solidly correct there) and the waist depth.
    //
    // Waist (1,0,0): the analytic smin dip at the tangent point is
    // k/16 = 0.03125 (see the SmoothMin regression tests); the faceted
    // res-8 surface + tessellation perturb it, so a 50% band around
    // k/16 is the honest coherence check (measured: -0.0348413).
    const float k = 0.5f;
    const float waist = sdf_body->value(Vec3f{1, 0, 0});
    CHECK(waist < 0.0f);
    CHECK(waist == doctest::Approx(-k / 16.0f).epsilon(0.5f));
    // Deep inside (center of sphere A): solidly negative.
    CHECK(sdf_body->value(Vec3f{0, 0, 0}) < 0.0f);
    // The mesh must cover the source body ([-1, 3] x [-1, 1] x [-1, 1]
    // for the two unit spheres at distance 2).
    auto bnds = sdf_body->bounds();
    CHECK(!bnds.empty());
    CHECK(bnds.min.x <= -1.0f);
    CHECK(bnds.max.x >= 3.0f);
    CHECK(bnds.min.y <= -1.0f);
    CHECK(bnds.max.y >= 1.0f);
    CHECK(bnds.min.z <= -1.0f);
    CHECK(bnds.max.z >= 1.0f);
}

// ---------------------------------------------------------------------------
// ADR-0026 MVP — topology-era extraction contract (T1–T6).
//
// Pinned counts were measured ONCE with a throwaway driver on the pinned
// configurations below (GCC, Release, -ffp-contract=off) and frozen here,
// the same method as the ADR-0025 v5 exact-count tests. For a closed
// triangle mesh the counts satisfy E = 3F/2 and V = F/2 + 2 (chi = 2);
// the pinned values below match those identities, which is the cheapest
// hand-derivation cross-check.
//
// The contract under test (ADR-0026 D2): V/E/F depend on the grid; chi=2,
// is_closed_manifold and loops_are_closed DO NOT.
// ---------------------------------------------------------------------------

// The smooth-union configuration pinned by T7–T10 (and whose grid T4
// also checks): two unit spheres, centers 1.2 apart, polynomial smin
// blend k = 0.4 — the ADR-0026 "first follow-up" body.
static s::SDFBody smooth_union_two_unit_spheres(Vec3f ca, Vec3f cb) {
    auto a = s::make_sphere(1.0f);
    if (ca != Vec3f{0, 0, 0}) a = s::sdf_translate(std::move(a), ca);
    auto b = s::sdf_translate(s::make_sphere(1.0f), cb);
    return s::sdf_smooth_union(std::move(a), std::move(b), 0.4f);
}

// T1 — sphere r=1 centered, res=16: pinned counts + full contract.
TEST_CASE("Bridge T1: sphere res=16 - pinned counts + manifold contract") {
    auto sph = s::make_sphere(1.0f);
    SdfToBrepOptions opts;
    opts.fit_primitives = false;
    opts.march_resolution = 16;
    auto body = sdf_to_brep_faceted(sph, opts);

    REQUIRE(body != nullptr);
    // Pinned (sphere r=1 centered, res=16): measured once, frozen.
    CHECK(body->vertex_count() == 1158);
    CHECK(body->edge_count() == 3468);
    CHECK(body->face_count() == 2312);
    // Grid-independent contracts.
    CHECK(b::euler_characteristic(*body) == 2);
    CHECK(b::is_closed_manifold(*body));
    CHECK(b::loops_are_closed(*body));
    // Explicit invariants 1-2 (ADR-0022 sec. 7): every edge exactly 2
    // co-edges, with opposite orientation (XOR).
    for (std::size_t e = 0; e < body->edge_count(); ++e) {
        const auto& ce = body->edges[e].coedges();
        REQUIRE(ce.size() == 2);
        CHECK(ce[0]->reversed() != ce[1]->reversed());
    }
}

// T2 — res=8 and res=32: different pinned counts, same contracts —
// V/E/F are grid-dependent, the invariants are not.
TEST_CASE("Bridge T2: sphere res=8/32 - grid-dependent counts, invariant contracts") {
    struct Pin { std::uint32_t res; std::size_t v, e, f; };
    const Pin pins[] = {
        {8,  270,   804,   536},
        {32, 4662, 13980,  9320},
    };
    for (const auto& pin : pins) {
        auto sph = s::make_sphere(1.0f);
        SdfToBrepOptions opts;
        opts.fit_primitives = false;
        opts.march_resolution = pin.res;
        auto body = sdf_to_brep_faceted(sph, opts);

        REQUIRE(body != nullptr);
        CHECK(body->vertex_count() == pin.v);
        CHECK(body->edge_count() == pin.e);
        CHECK(body->face_count() == pin.f);
        CHECK(b::euler_characteristic(*body) == 2);
        CHECK(b::is_closed_manifold(*body));
        CHECK(b::loops_are_closed(*body));
    }
}

// T3 — off-grid center (0.13, 0.29, 0.07): the contracts survive a
// generic placement. Note (measured): the pinned counts are IDENTICAL
// to T1's — the extraction is translation-invariant for sphere inputs
// because bounds() is symmetric about the center and the 1% pad is
// proportional, so the padded grid is symmetric about the center for
// any translation. Pinning them documents that invariance.
TEST_CASE("Bridge T3: off-grid sphere (0.13, 0.29, 0.07) - invariants survive generic placement") {
    auto sph = s::sdf_translate(s::make_sphere(1.0f), {0.13f, 0.29f, 0.07f});
    SdfToBrepOptions opts;
    opts.fit_primitives = false;
    opts.march_resolution = 16;
    auto body = sdf_to_brep_faceted(sph, opts);

    REQUIRE(body != nullptr);
    CHECK(body->vertex_count() == 1158);
    CHECK(body->edge_count() == 3468);
    CHECK(body->face_count() == 2312);
    CHECK(b::euler_characteristic(*body) == 2);
    CHECK(b::is_closed_manifold(*body));
    CHECK(b::loops_are_closed(*body));
}

// T4 — the pinned grids satisfy the input contract (ADR-0026 D2):
// P2 = no on-grid zeros (min |f| > 0 over ALL grid points), P1 =
// enclosure (every boundary grid point strictly outside, f > 0).
// The grid construction must mirror core/sdf/src/mesh_extract.cpp:
// bounds() padded by 1% of extent, points at min + extent * (i / res).
TEST_CASE("Bridge T4: pinned grids satisfy the input contract (P1 enclosure, P2 no on-grid zeros)") {
    auto check_grid = [](const s::SDFBody& sdf, std::uint32_t res) {
        CAD_0::math::Bboxf bx = sdf.bounds();
        REQUIRE(!bx.empty());
        const CAD_0::math::Vec3f pad = bx.extent() * 0.01f;
        bx.min = bx.min - pad;
        bx.max = bx.max + pad;
        const auto ext = bx.extent();

        float min_abs_f = 1e30f;
        bool boundary_all_outside = true;
        for (std::uint32_t zi = 0; zi <= res; ++zi) {
            for (std::uint32_t yi = 0; yi <= res; ++yi) {
                for (std::uint32_t xi = 0; xi <= res; ++xi) {
                    const float fx = bx.min.x + ext.x * (static_cast<float>(xi) / static_cast<float>(res));
                    const float fy = bx.min.y + ext.y * (static_cast<float>(yi) / static_cast<float>(res));
                    const float fz = bx.min.z + ext.z * (static_cast<float>(zi) / static_cast<float>(res));
                    const float f = sdf.value({fx, fy, fz});
                    min_abs_f = std::min(min_abs_f, std::abs(f));
                    const bool boundary = xi == 0 || yi == 0 || zi == 0 ||
                                          xi == res || yi == res || zi == res;
                    if (boundary && !(f > 0.0f)) boundary_all_outside = false;
                }
            }
        }
        CHECK(min_abs_f > 0.0f);          // P2: no grid point exactly on the surface
        CHECK(boundary_all_outside);      // P1: surface strictly inside the padded volume
    };

    check_grid(s::make_sphere(1.0f), 8);
    check_grid(s::make_sphere(1.0f), 16);
    check_grid(s::make_sphere(1.0f), 32);
    check_grid(s::sdf_translate(s::make_sphere(1.0f), {0.13f, 0.29f, 0.07f}), 16);
    // The smooth-union grids pinned by T7–T9 satisfy the input contract
    // too (P1 enclosure — including the k/4 blend bulge at the waist —
    // and P2 no on-grid zeros). This is the classification instrument:
    // if a T7–T9 contract ever fails, this check tells a P1/P2 input
    // problem apart from a bridge bug.
    check_grid(smooth_union_two_unit_spheres({0, 0, 0}, {1.2f, 0, 0}), 16);
    check_grid(
        smooth_union_two_unit_spheres({0.13f, 0.29f, 0.07f}, {1.33f, 0.29f, 0.07f}),
        16);
}

// T5 — determinism (ADR-0026 D5): two extractions of the same (SDF,
// grid) produce identical V/E/F and a bit-equal vertex position
// sequence (== on floats, no epsilon).
TEST_CASE("Bridge T5: determinism - same (SDF, grid), bit-identical output") {
    auto extract = [] {
        auto sph = s::sdf_translate(s::make_sphere(1.0f), {0.13f, 0.29f, 0.07f});
        SdfToBrepOptions opts;
        opts.fit_primitives = false;
        opts.march_resolution = 16;
        return sdf_to_brep_faceted(sph, opts);
    };
    auto first = extract();
    auto second = extract();
    REQUIRE(first != nullptr);
    REQUIRE(second != nullptr);

    CHECK(first->vertex_count() == second->vertex_count());
    CHECK(first->edge_count() == second->edge_count());
    CHECK(first->face_count() == second->face_count());
    REQUIRE(first->vertex_count() == second->vertex_count());
    for (std::size_t i = 0; i < first->vertex_count(); ++i) {
        CHECK(first->vertices[i].position() == second->vertices[i].position());
    }
}

// T6 — empty input: a visible, actionable error — never a crash, never
// a silently broken body (the pre-Phase-D stub returned an empty body
// with shell.is_closed() == true).
TEST_CASE("Bridge T6: empty SDF - visible error, no crash") {
    s::SDFBody empty;
    SdfToBrepOptions opts;
    opts.fit_primitives = true;

    CHECK_THROWS_AS(sdf_to_brep(empty, opts), std::invalid_argument);
    CHECK_THROWS_AS(sdf_to_brep_faceted(empty, opts), std::invalid_argument);
}

// ---------------------------------------------------------------------------
// ADR-0026 Phase E closure — smooth-union follow-up (the "first
// follow-up" the ADR promised): a genuinely blended body through the
// bridge, under the same hard contracts as the sphere MVP. Pinned
// counts measured ONCE with a throwaway driver (GCC, Release,
// -ffp-contract=off — the flag the root CMakeLists already enforces)
// and frozen here, same method as T1–T3 (ADR-0025 v5).
//
// Prerequisite kernel fix (same session, core/sdf/operators.hpp): the
// k>0 clamp of SmoothMinSDF had the sign flipped, so "smooth union"
// followed the LARGER child — an inflated smooth intersection. These
// pins are measured against the FIXED operator.
// ---------------------------------------------------------------------------

// T7 — smooth union, res=16: pinned counts + full contract. No
// degenerate triangles is guaranteed three ways: (1) the faceted path
// THROWS on a zero-area / repeated-index triangle, so a successful
// extraction already proves none exist; (2) a repeated-index triangle
// would create a self-loop edge and break the per-edge 2-coedge XOR
// below; (3) the pinned counts match the closed-mesh identities.
TEST_CASE("Bridge T7: smooth-union res=16 - pinned counts + manifold contract") {
    auto u = smooth_union_two_unit_spheres({0, 0, 0}, {1.2f, 0, 0});
    SdfToBrepOptions opts;
    opts.fit_primitives = false;
    opts.march_resolution = 16;
    auto body = sdf_to_brep_faceted(u, opts);

    REQUIRE(body != nullptr);
    // Pinned (two unit spheres, centers (0,0,0)/(1.2,0,0), k=0.4,
    // res=16): measured once, frozen.
    CHECK(body->vertex_count() == 1310);
    CHECK(body->edge_count() == 3924);
    CHECK(body->face_count() == 2616);
    // Grid-independent contracts.
    CHECK(b::euler_characteristic(*body) == 2);
    CHECK(b::is_closed_manifold(*body));
    CHECK(b::loops_are_closed(*body));
    // Explicit invariants 1-2 (ADR-0022 sec. 7): every edge exactly 2
    // co-edges, with opposite orientation (XOR).
    for (std::size_t e = 0; e < body->edge_count(); ++e) {
        const auto& ce = body->edges[e].coedges();
        REQUIRE(ce.size() == 2);
        CHECK(ce[0]->reversed() != ce[1]->reversed());
    }
}

// T8 — off-grid centers (both + (0.13, 0.29, 0.07)): the contracts
// survive a generic placement. Note (measured): the pinned counts are
// IDENTICAL to T7's — the same translation invariance as T3 (bounds()
// is analytic per child, the union box translates rigidly with the
// shape, and the 1% pad is proportional, so the padded grid is the
// same relative lattice for any translation). Pinning them documents
// that invariance; the grid-INdependence demonstration is carried by
// the different counts of T9.
TEST_CASE("Bridge T8: off-grid smooth-union - invariants survive generic placement") {
    auto u = smooth_union_two_unit_spheres({0.13f, 0.29f, 0.07f}, {1.33f, 0.29f, 0.07f});
    SdfToBrepOptions opts;
    opts.fit_primitives = false;
    opts.march_resolution = 16;
    auto body = sdf_to_brep_faceted(u, opts);

    REQUIRE(body != nullptr);
    CHECK(body->vertex_count() == 1310);
    CHECK(body->edge_count() == 3924);
    CHECK(body->face_count() == 2616);
    CHECK(b::euler_characteristic(*body) == 2);
    CHECK(b::is_closed_manifold(*body));
    CHECK(b::loops_are_closed(*body));
}

// T8b — ROTATED instance of the T7 body. Translation alone cannot
// prove grid-independence of the contracts: the padded grid translates
// rigidly with the body, so the surface keeps the same alignment
// against the lattice (T3/T8 pinned that invariance). A GENERIC
// ROTATION is the honest probe: RotateSDF::bounds() returns the corner
// hull of the rotated child AABB — a different (larger) box — so the
// same resolution now samples a different lattice against the same
// shape. Counts are therefore DIFFERENT from T7's (pinned below,
// measured once like every other pin), while the invariants must not
// move. The rotation (axis (1,2,3) normalized, 0.6 rad) is generic:
// no symmetry of the two-sphere body survives it relative to the grid.
TEST_CASE("Bridge T8b: rotated smooth-union - invariants survive generic orientation") {
    auto u = smooth_union_two_unit_spheres({0, 0, 0}, {1.2f, 0, 0});
    const CAD_0::math::Quatf q =
        CAD_0::math::Quatf::from_axis_angle(Vec3f{1, 2, 3}.normalized(), 0.6f);
    auto r = s::sdf_rotate(std::move(u), q);
    SdfToBrepOptions opts;
    opts.fit_primitives = false;
    opts.march_resolution = 16;
    auto body = sdf_to_brep_faceted(r, opts);

    REQUIRE(body != nullptr);
    // Pinned (T7 body rotated by q, res=16): measured once, frozen.
    // Deliberately != T7's 1310/3924/2616: the counts are orientation
    // dependent (the rotated AABB is larger, so res=16 cells are
    // coarser relative to the shape) — the invariants are not.
    CHECK(body->vertex_count() == 574);
    CHECK(body->edge_count() == 1716);
    CHECK(body->face_count() == 1144);
    CHECK(b::euler_characteristic(*body) == 2);
    CHECK(b::is_closed_manifold(*body));
    CHECK(b::loops_are_closed(*body));
    // Closed-mesh identities hold for the pin (E = 3F/2, V = F/2 + 2).
}

// T9 — res=32: different pinned counts, same contracts — V/E/F are
// grid-dependent, the invariants are not.
TEST_CASE("Bridge T9: smooth-union res=32 - grid-dependent counts, invariant contracts") {
    auto u = smooth_union_two_unit_spheres({0, 0, 0}, {1.2f, 0, 0});
    SdfToBrepOptions opts;
    opts.fit_primitives = false;
    opts.march_resolution = 32;
    auto body = sdf_to_brep_faceted(u, opts);

    REQUIRE(body != nullptr);
    // Pinned (same configuration as T7, res=32).
    CHECK(body->vertex_count() == 5334);
    CHECK(body->edge_count() == 15996);
    CHECK(body->face_count() == 10664);
    CHECK(b::euler_characteristic(*body) == 2);
    CHECK(b::is_closed_manifold(*body));
    CHECK(b::loops_are_closed(*body));
}

// T10 — NEGATIVE fit test: the blended body must NOT match a
// primitive. sdf_to_brep_report with fit_primitives=true dispatches
// through the primitive detector first; an smin-rooted SDF must fall
// through to the faceted path. A "sphere"/"box"/"cylinder" label
// here would mean the primitive detection matched a composite body.
TEST_CASE("Bridge T10: smooth-union fit is faceted - blended bodies never match primitives") {
    auto u = smooth_union_two_unit_spheres({0, 0, 0}, {1.2f, 0, 0});
    SdfToBrepOptions opts;
    opts.fit_primitives = true;
    opts.march_resolution = 16;
    auto rep = sdf_to_brep_report(u, opts);

    CHECK(rep.fit == "faceted");
    // The report carries the same pinned contract as T7.
    CHECK(rep.vertex_count == 1310);
    CHECK(rep.edge_count == 3924);
    CHECK(rep.face_count == 2616);
    CHECK(rep.chi == 2);
    CHECK(rep.closed);
    CHECK(rep.loops);
}

// T11 — min |f| over the sampled grid (ADR-0026 Consequences
// left-over, resolved as a report field): the pinned sphere's P2
// margin, measured once and frozen. Deterministic float: the build
// enforces -ffp-contract=off and the value class is sqrt-only
// (ADR-0026 D5) — verified bit-identical across the release and
// ASan-debug presets before freezing. Path-independent: the primitive
// path reports the same margin at the same resolution.
TEST_CASE("Bridge T11: report min_abs_field - frozen P2 margin, path-independent") {
    {
        auto sph = s::make_sphere(1.0f);
        SdfToBrepOptions opts;
        opts.fit_primitives = false;
        opts.march_resolution = 16;
        auto rep = sdf_to_brep_report(sph, opts);
        CHECK(rep.min_abs_field > 0.0f);             // a P2 margin exists
        CHECK(rep.min_abs_field == 0.00393593311f);  // measured, frozen
    }
    {
        auto sph = s::make_sphere(1.0f);
        SdfToBrepOptions opts;
        opts.fit_primitives = true;
        opts.march_resolution = 16;
        auto rep = sdf_to_brep_report(sph, opts);
        CHECK(rep.fit == "sphere");
        // The stat measures the grid, not the winning path.
        CHECK(rep.min_abs_field == 0.00393593311f);
    }
}

// T12 — grid-mirror coherence (Group C, consolidation sweep): the
// report's min_abs_field is computed by grid_min_abs_field() in
// sdf_to_brep.cpp, which MIRRORS the marching-cubes auto-bounds layout
// of core/sdf/src/mesh_extract.cpp (bounds() + 1% pad, points at
// min + extent * (i / res)). The mirror is deliberately not shared
// with mesh_extract (two copies cross-check); this test adds the THIRD
// side of the triangle — an independent scan of the mesh_extract
// layout must reproduce the bridge's number BIT-EXACTLY. If either
// copy drifts (different pad, different point formula, different
// bounds), the equality breaks and forces a conscious sync.
TEST_CASE("Bridge T12: report min_abs_field matches the mesh_extract grid layout") {
    const auto scan = [](const s::SDFBody& sdf, std::uint32_t res) {
        // Replicates mesh_extract.cpp's auto-bounds grid EXACTLY (the
        // same expression, the same order): bounds() + 1% pad, then
        // min + extent * (i / res) per axis.
        CAD_0::math::Bboxf bx = sdf.bounds();
        REQUIRE(!bx.empty());
        const CAD_0::math::Vec3f pad = bx.extent() * 0.01f;
        bx.min = bx.min - pad;
        bx.max = bx.max + pad;
        const auto ext = bx.extent();

        float min_abs_f = 1e30f;
        for (std::uint32_t zi = 0; zi <= res; ++zi) {
            for (std::uint32_t yi = 0; yi <= res; ++yi) {
                for (std::uint32_t xi = 0; xi <= res; ++xi) {
                    const float fx = bx.min.x + ext.x * (static_cast<float>(xi) / static_cast<float>(res));
                    const float fy = bx.min.y + ext.y * (static_cast<float>(yi) / static_cast<float>(res));
                    const float fz = bx.min.z + ext.z * (static_cast<float>(zi) / static_cast<float>(res));
                    min_abs_f = std::min(min_abs_f, std::abs(sdf.value({fx, fy, fz})));
                }
            }
        }
        return min_abs_f;
    };

    {
        // The T1/T11 sphere: the independent scan must reproduce BOTH
        // the report field and the frozen T11 pin.
        auto sph = s::make_sphere(1.0f);
        SdfToBrepOptions opts;
        opts.fit_primitives = false;
        opts.march_resolution = 16;
        auto rep = sdf_to_brep_report(sph, opts);
        CHECK(scan(sph, 16) == rep.min_abs_field);       // bit-identical
        CHECK(rep.min_abs_field == 0.00393593311f);      // the T11 pin
    }
    {
        // A composite body (bounds via union of children): the same
        // coherence on a different bounds() path.
        auto u = smooth_union_two_unit_spheres({0, 0, 0}, {1.2f, 0, 0});
        SdfToBrepOptions opts;
        opts.fit_primitives = false;
        opts.march_resolution = 16;
        auto rep = sdf_to_brep_report(u, opts);
        CHECK(scan(u, 16) == rep.min_abs_field);
    }
}

// ---------------------------------------------------------------------------
// ADR-0027 D6 (Phase F, accepted) — the genus-1 contract: the pierced
// box, the genus derivation rule, and the wrapper equivalence over the
// genus-0 fixtures of this suite.
//
// The pierced-box configuration pinned by T13–T15:
//   target = make_box({1,1,1})           → [-1,1]^3 (half-extents)
//   tool   = make_cylinder(0.25f, 3.0f)  → along Y, r = 0.25,
//                                        y ∈ [-1.5, 1.5] (through hole)
//   body   = sdf_subtract(box, cylinder) → genus 1, χ = 0 — the FIRST
//                                        genus > 0 body of the repo.
// Pinned counts were measured ONCE with a throwaway driver (GCC,
// Release, -ffp-contract=off — the same method as T1–T3) and frozen.
// ---------------------------------------------------------------------------

static s::SDFBody pierced_box() {
    auto box = s::make_box(Vec3f{1, 1, 1});
    auto cyl = s::make_cylinder(0.25f, 3.0f);
    return s::sdf_subtract(std::move(box), std::move(cyl));
}

// T13 — the MVP's pinned heart: the first genus-1 extraction. The
// structural contract holds; χ = 0 is REPORTED and the genus derived
// (χ ≤ 2, even); the compatibility wrapper keeps failing it — its
// exact historical meaning, the zero-silent-weakening check.
TEST_CASE("Bridge T13: pierced box res=48 - genus 1 pinned counts + structural contract") {
    SdfToBrepOptions opts;
    opts.fit_primitives = false;
    opts.march_resolution = 48;
    auto body = sdf_to_brep_faceted(pierced_box(), opts);

    REQUIRE(body != nullptr);
    // Pinned (pierced box, res=48): measured once, frozen. The
    // closed-mesh identities hold with genus: E = 3F/2 and
    // V − E + F = 0 (one shell, genus 1).
    CHECK(body->vertex_count() == 15104);
    CHECK(body->edge_count() == 45312);
    CHECK(body->face_count() == 30208);
    CHECK(body->edge_count() == 3 * body->face_count() / 2);

    // Grid-independent contracts, genus-1 edition.
    CHECK(b::euler_characteristic(*body) == 0);
    CHECK(b::is_closed_orientable(*body));
    CHECK(b::loops_are_closed(*body));
    // The compatibility wrapper keeps its exact historical meaning
    // (structural ∧ χ == 2): the pierced box FAILS it, exactly as it
    // did before the amendment. This is the wrapper telling the truth
    // about its own contract — not an extraction error.
    CHECK_FALSE(b::is_closed_manifold(*body));

    // Explicit invariants 1-2 (ADR-0022 sec. 7, structural): every
    // edge exactly 2 co-edges, opposite orientation (XOR).
    for (std::size_t e = 0; e < body->edge_count(); ++e) {
        const auto& ce = body->edges[e].coedges();
        REQUIRE(ce.size() == 2);
        CHECK(ce[0]->reversed() != ce[1]->reversed());
    }

    // The report: χ = 0, genus DERIVED = 1 (acceptance correction 1),
    // orientable, loops; the legacy closed field False; the P2 margin
    // measured once and frozen.
    const auto rep = sdf_to_brep_report(pierced_box(), opts);
    CHECK(rep.fit == "faceted");
    CHECK(rep.chi == 0);
    CHECK(rep.orientable);
    CHECK(rep.genus_derived);
    CHECK(rep.genus == 1);
    CHECK_FALSE(rep.closed);
    CHECK(rep.loops);
    CHECK(rep.min_abs_field > 0.0f);
    CHECK(rep.min_abs_field == 0.00218443573f);
}

// T13b — determinism (the T5 habit, genus-1 edition): two fresh
// constructions of the same (SDF, grid) produce bit-identical output.
TEST_CASE("Bridge T13b: pierced box determinism - bit-identical re-run") {
    auto extract = [] {
        SdfToBrepOptions opts;
        opts.fit_primitives = false;
        opts.march_resolution = 48;
        return sdf_to_brep_faceted(pierced_box(), opts);
    };
    auto first = extract();
    auto second = extract();
    REQUIRE(first != nullptr);
    REQUIRE(second != nullptr);

    CHECK(first->vertex_count() == second->vertex_count());
    CHECK(first->edge_count() == second->edge_count());
    CHECK(first->face_count() == second->face_count());
    REQUIRE(first->vertex_count() == second->vertex_count());
    for (std::size_t i = 0; i < first->vertex_count(); ++i) {
        CHECK(first->vertices[i].position() == second->vertices[i].position());
    }
}

// T14 — second grid (the T2 habit, genus-1 edition): counts differ,
// χ = 0 does not — V/E/F are grid-dependent, the topological class
// is not.
TEST_CASE("Bridge T14: pierced box res=32 - counts differ, chi = 0 invariant") {
    SdfToBrepOptions opts;
    opts.fit_primitives = false;
    opts.march_resolution = 32;
    auto body = sdf_to_brep_faceted(pierced_box(), opts);

    REQUIRE(body != nullptr);
    // Pinned (pierced box, res=32): measured once, frozen. Deliberately
    // != T13's 15104/45312/30208 — the counts are grid-dependent.
    CHECK(body->vertex_count() == 6544);
    CHECK(body->edge_count() == 19632);
    CHECK(body->face_count() == 13088);

    CHECK(b::euler_characteristic(*body) == 0);   // invariant
    CHECK(b::is_closed_orientable(*body));
    CHECK(b::loops_are_closed(*body));
    CHECK_FALSE(b::is_closed_manifold(*body));    // wrapper: χ ≠ 2

    const auto rep = sdf_to_brep_report(pierced_box(), opts);
    CHECK(rep.chi == 0);
    CHECK(rep.genus_derived);
    CHECK(rep.genus == 1);
    CHECK(rep.min_abs_field > 0.0f);
    CHECK(rep.min_abs_field == 0.00499999523f);
}

// T15 — the genus derivation rule on a multi-shell body (acceptance
// correction 1): two disjoint spheres → χ = 4 → genus NOT derived
// ("multi-shell, n/d"), while the structural contract holds.
TEST_CASE("Bridge T15: disjoint two-sphere union - chi = 4, genus n/d (multi-shell rule)") {
    auto a = s::make_sphere(1.0f);
    auto b = s::sdf_translate(s::make_sphere(1.0f), {5.0f, 0.0f, 0.0f});
    auto two = s::sdf_union(std::move(a), std::move(b));
    SdfToBrepOptions opts;
    opts.fit_primitives = false;
    opts.march_resolution = 16;
    const auto rep = sdf_to_brep_report(two, opts);

    // Pinned (two disjoint spheres r=1, centers 5 apart, res=16):
    // measured once, frozen. Two shells → χ = 2 + 2 = 4.
    CHECK(rep.vertex_count == 1172);
    CHECK(rep.edge_count == 3504);
    CHECK(rep.face_count == 2336);
    CHECK(rep.chi == 4);

    // The structural contract holds; the genus is NOT derived (χ > 2 —
    // the components counter is deferred, ADR-0027 out of scope), and
    // the wrapper is False (χ ≠ 2).
    CHECK(rep.orientable);
    CHECK(rep.loops);
    CHECK_FALSE(rep.genus_derived);
    CHECK_FALSE(rep.closed);
    CHECK(rep.min_abs_field > 0.0f);
    CHECK(rep.min_abs_field == 0.00473845005f);
}

// T16 — wrapper equivalence over this suite's genus-0 fixtures
// (ADR-0027 D6 move 3): is_closed_manifold == (is_closed_orientable
// && χ == 2) on every pinned extraction of T1–T9 — nothing that
// passed starts failing, nothing that failed starts passing.
TEST_CASE("Bridge T16: wrapper equivalence on the genus-0 fixtures") {
    auto check = [](const s::SDFBody& sdf, std::uint32_t res) {
        SdfToBrepOptions opts;
        opts.fit_primitives = false;
        opts.march_resolution = res;
        auto body = sdf_to_brep_faceted(sdf, opts);
        REQUIRE(body != nullptr);
        CHECK(b::euler_characteristic(*body) == 2);  // the fixture class
        CHECK(b::is_closed_manifold(*body) ==
              (b::is_closed_orientable(*body) &&
               b::euler_characteristic(*body) == 2));
    };
    check(s::make_sphere(1.0f), 16);                                        // T1
    check(s::make_sphere(1.0f), 8);                                         // T2
    check(s::make_sphere(1.0f), 32);                                        // T2
    check(s::sdf_translate(s::make_sphere(1.0f), {0.13f, 0.29f, 0.07f}), 16); // T3
    check(smooth_union_two_unit_spheres({0, 0, 0}, {1.2f, 0, 0}), 16);      // T7
    check(smooth_union_two_unit_spheres({0.13f, 0.29f, 0.07f},
                                        {1.33f, 0.29f, 0.07f}), 16);        // T8
    check(smooth_union_two_unit_spheres({0, 0, 0}, {1.2f, 0, 0}), 32);      // T9
    {
        // T8b (rotated): the same equivalence under a rotation.
        auto u = smooth_union_two_unit_spheres({0, 0, 0}, {1.2f, 0, 0});
        const CAD_0::math::Quatf q =
            CAD_0::math::Quatf::from_axis_angle(Vec3f{1, 2, 3}.normalized(), 0.6f);
        check(s::sdf_rotate(std::move(u), q), 16);
    }
}

// ===========================================================================
// Property-based families (mini-traccia PARTE 2, area 2 — the bridge).
//
// Seeded generators (tests/property/property_prng.hpp) build members of
// the SUPPORTED INPUT CLASS, declared here with its explicit margins
// against the ADR-0026 preconditions (an input outside the class is a
// GENERATOR bug, classified as such — never a kernel bug):
//
//   * spheres: radius in [0.5, 2.0], center generic — sampled uniform
//     in [-0.4, 0.4]^3 plus a FIXED KNOWN IRRATIONAL offset (P2: the
//     grid never aligns with the surface, no on-grid zeros):
//       +0.0137255020f  (sqrt(2)-1, scaled by 1/33.137…, truncated)
//       — any fixed irrational-derived constant would do; what is
//     pinned is that it is declared, not accidental.
//   * pierced boxes: target box [-1,1]^3 (half-extents 1), through
//     cylinder h = 3.0 (the T13 geometry), hole radius in [0.34, 0.45],
//     hole center uniform in [-0.25, 0.25]^3 + the same irrational
//     offset. P3 margin, EXPLICIT: at march_resolution = 24 the cell
//     size is (2 + 2%) / 24 ~= 0.0842 (bounds + 1% pad), so
//       hole radius   >= 0.34      ~ 4.04 cells  (T14's pinned margin)
//       box wall      >= 1 - 0.27 - 0.45 ~ 3.3 cells
//     both above the ~1-cell pinch threshold of ADR-0026 P3 by > 3x.
//     P1 holds by construction (compact SDF, honest bounds); P4 is the
//     sign discipline the SDF property family covers.
//
// Resolution 24, DECLARED per the mini-traccia cost rule (the pinned
// suites stay at 48/32; the family keeps the total suite cost bounded).
// Measured cost of the two families (25 + 25 members, this build):
// see the numbers in each TEST_CASE comment — reported, not optimized.
//
// Invariants asserted for EVERY member (they must NOT depend on the
// grid — ADR-0026 D2 as amended by ADR-0027 D6):
//   sphere:       chi = 2, orientable, loops, E = 3F/2, V = F/2 + 2,
//                 min|f| > 0 (P2, machine-readable)
//   pierced box:  chi = 0, genus = 1 (derived), orientable, loops,
//                 E = 3F/2, V - E + F = 0, min|f| > 0
// ===========================================================================

#include "property/property_prng.hpp"

namespace {

namespace prop = CAD_0::test::property;

// The fixed, known irrational offset (P2 discipline — declared, not
// accidental). Same constant for both families.
constexpr float kOffGridOffset = 0.0137255020f;

Vec3f off_grid_center(prop::Prng& prng, float lo, float hi) {
    return Vec3f{prng.frange(lo, hi) + kOffGridOffset,
                 prng.frange(lo, hi) - kOffGridOffset,
                 prng.frange(lo, hi) + kOffGridOffset};
}

// Generic unit direction (rejection in the cube, avoiding both the
// degenerate center and the corners): the smooth-union family below
// chains its spheres along a RANDOM direction, so the members hit the
// grid in generic orientations — the rotation lesson of T8b, which
// pure translations cannot teach.
Vec3f random_unit_dir(prop::Prng& prng) {
    for (;;) {
        const Vec3f d{prng.frange(-1.0f, 1.0f),
                      prng.frange(-1.0f, 1.0f),
                      prng.frange(-1.0f, 1.0f)};
        const float l = d.length();
        if (l > 0.2f && l < 1.0f) return d * (1.0f / l);
    }
}

} // namespace

TEST_CASE("property: bridge spheres (seeded family) - genus-0 invariants at res 24") {
    const std::uint32_t seed = 0xB0D61001u;
    prop::Prng prng(seed);
    // Measured once on this build (GCC 14.2, Release, -ffp-contract=off):
    // 25 members at res 24 = ~0.14 s total (report call per member).
    for (int iter = 0; iter < 25; ++iter) {
        const float r = prng.frange(0.5f, 2.0f);
        const Vec3f c = off_grid_center(prng, -0.4f, 0.4f);
        auto body = s::sdf_translate(s::make_sphere(r), c);

        SdfToBrepOptions opts;
        opts.fit_primitives = false;  // exercise the MC33 faceted path
        opts.march_resolution = 24;   // declared above (cost rule)
        const SdfToBrepReport rep = sdf_to_brep_report(body, opts);

        INFO("seed=", seed, " iter=", iter, " r=", r,
             " center=(", c.x, ",", c.y, ",", c.z, ")");
        INFO("V=", rep.vertex_count, " E=", rep.edge_count, " F=", rep.face_count,
             " chi=", rep.chi, " min|f|=", rep.min_abs_field);

        INFO("check: fit == \"faceted\" (primitive fitting disabled)");
        CHECK(rep.fit == "faceted");
        INFO("check: chi == 2 (genus 0, one shell)");
        CHECK(rep.chi == 2);
        INFO("check: orientable (is_closed_orientable)");
        CHECK(rep.orientable);
        INFO("check: loops are closed");
        CHECK(rep.loops);
        INFO("check: E == 3F/2 (closed pure-triangle extract)");
        CHECK(rep.edge_count == 3 * rep.face_count / 2);
        INFO("check: V == F/2 + 2 (Euler identity for genus 0)");
        CHECK(2 * rep.vertex_count == rep.face_count + 4);
        INFO("check: min|f| > 0 on the sampling grid (P2 — no on-grid zeros)");
        CHECK(rep.min_abs_field > 0.0f);
    }
}

TEST_CASE("property: bridge pierced boxes (seeded family) - genus-1 invariants at res 24") {
    const std::uint32_t seed = 0xB0D61002u;
    prop::Prng prng(seed);
    // Measured once on this build (GCC 14.2, Release, -ffp-contract=off):
    // 25 members at res 24 = ~0.20 s total (report call per member).
    for (int iter = 0; iter < 25; ++iter) {
        const float rh = prng.frange(0.34f, 0.45f);  // P3 margin declared above
        const Vec3f c = off_grid_center(prng, -0.25f, 0.25f);
        auto body = s::sdf_subtract(
            s::make_box(Vec3f{1, 1, 1}),
            s::sdf_translate(s::make_cylinder(rh, 3.0f), c));

        SdfToBrepOptions opts;
        opts.fit_primitives = false;
        opts.march_resolution = 24;   // declared above (cost rule)
        const SdfToBrepReport rep = sdf_to_brep_report(body, opts);

        INFO("seed=", seed, " iter=", iter, " hole_r=", rh,
             " hole_center=(", c.x, ",", c.y, ",", c.z, ")");
        INFO("V=", rep.vertex_count, " E=", rep.edge_count, " F=", rep.face_count,
             " chi=", rep.chi, " min|f|=", rep.min_abs_field);

        INFO("check: chi == 0 (V - E + F = 0, genus 1)");
        CHECK(rep.chi == 0);
        INFO("check: genus derived == 1 (the blocking rule)");
        CHECK(rep.genus_derived);
        CHECK(rep.genus == 1);
        INFO("check: orientable (is_closed_orientable)");
        CHECK(rep.orientable);
        INFO("check: loops are closed");
        CHECK(rep.loops);
        INFO("check: E == 3F/2 (closed pure-triangle extract)");
        CHECK(rep.edge_count == 3 * rep.face_count / 2);
        INFO("check: min|f| > 0 on the sampling grid (P2 — no on-grid zeros)");
        CHECK(rep.min_abs_field > 0.0f);
    }
}

// ===========================================================================
// Property-based family — EXTENSION: smooth unions (FASE 2 of the
// property extension, smin k > 0).
//
// Tree = smooth union of 2-3 generated spheres chained along a random
// unit direction. Supported input class, declared with its margins:
//
//   * sphere count n in {2, 3}; radii r in [0.55, 1.0] (>= 2.4 cells
//     at the worst-case cell below — each sphere individually resolved).
//   * chain TOPOLOGY: collinear centers (c_{i+1} = c_i + u * d_i, one
//     fixed random unit direction). The 2D profile of the hard union
//     along the chain is a chain of overlapping disks — simply
//     connected and touching the axis — so the revolved solid is a
//     topological ball; the blend only fillets crevices or bridges
//     gaps THROUGH the axis (the neck dip k/16 is centered on it), so
//     genus 0 is structural for this class, not hoped for.
//   * per ADJACENT pair, one of two declared regimes:
//       - OVERLAP: d = r_i + r_j - o, o in [0.35, 0.65] (2.0-3.7
//         cells). Hard-union waist >= 2 cells at the worst case
//         (r1 = r2 = 0.55, o = 0.65 -> waist 0.50; unequal worst
//         r1 = 1.0, r2 = 0.55, o = 0.35 -> 0.45).
//       - NECK: equal radii per pair (r_j := r_i — the closed-form
//         margin below is exact only for equal spheres), gap
//         g = d - 2r in [0.02, k/8 - 0.02] (away from exact touching
//         and from the neck-collapse boundary g = k/8 where the blend
//         stops bridging), and the neck must be FAT:
//             neck diameter = 2*sqrt((r + k/16)^2 - (r + g/2)^2) >= 2 cells
//         enforced by deterministic rejection (the formula is the
//         zero-set radius of the bridging tube at its thinnest).
//   * blend k per pair in [0.45, 0.70] — a declared SUBRANGE of the
//     FASE 1 range [0.10, 1.00]: the blend band |f_a - f_b| < k maps
//     to a fillet strip of width >= k/2 >= 0.225 >= 1 cell at the
//     worst-case cell (extent <= 5.3 for 3-chains -> cell <= 0.225).
//   * P2: base center off-grid (kOffGridOffset), random chain
//     direction, generic radii/gaps — no on-grid surface alignment.
//
// BOUNDS()/extraction-volume safety (the FASE 1.1 verification,
// applied before these trees were used here): the extraction grid is
// bounds() + 1% of extent (sdf_to_brep.cpp); the blend's k/16 dilation
// can escape a plain hull ONLY where the two operand fields stay
// within k/7 of each other up to the hull boundary (near-coincident
// surfaces — the concentric counterexample, see test_operators.cpp).
// In this chain class the near-coincident surfaces (crevices, necks)
// are strictly INTERIOR to the hull: measured on family-shaped
// geometry with the standalone probe driver (kept with the other
// measurement drivers OUTSIDE the repo), min f on the padded hull
// faces = +0.0155..+0.0202 > 0. Safe by construction + probe.
//
// Invariants asserted for EVERY member (resolution 24, declared per
// the cost rule; fit_primitives = false so the negative on fitting —
// fit == "faceted" — stays part of the contract on blends):
//   chi = 2, genus = 0 (derived), orientable, loops, E = 3F/2,
//   V = F/2 + 2, min|f| > 0.
//
// Measured cost on this build (GCC 14.2 Release, -ffp-contract=off):
// see the number in the TEST_CASE — reported, not optimized; the
// iterations stay FIXED at 25.
// ===========================================================================
TEST_CASE("property: bridge smooth unions (seeded family) - genus-0 invariants at res 24") {
    const std::uint32_t seed = 0xB0D61003u;
    prop::Prng prng(seed);
    // Measured once on this build (GCC 14.2, Release, -ffp-contract=off):
    // 25 members at res 24 = ~0.16 s total (11 neck members, 14
    // overlap-only — both regimes live).
    int neck_members = 0;
    int overlap_members = 0;
    for (int iter = 0; iter < 25; ++iter) {
        // --- generate, with deterministic rejection on the P3 margins ---
        std::vector<float> radii, dists, ks;
        std::vector<int> neck_pair;  // 1 = neck regime, 0 = overlap
        float hull_extent = 0.0f;
        {
            int tries = 0;
            for (;;) {
                ++tries;
                REQUIRE(tries < 100000);  // generator bug if exhausted
                radii.clear();
                dists.clear();
                ks.clear();
                neck_pair.clear();
                const std::uint32_t n = prng.coin() ? 2u : 3u;
                radii.push_back(prng.frange(0.55f, 1.0f));
                for (std::uint32_t p = 0; p + 1 < n; ++p) {
                    const float k = prng.frange(0.45f, 0.70f);
                    ks.push_back(k);
                    if (prng.coin()) {
                        // OVERLAP regime: any radii, overlap 2.0-3.7 cells.
                        radii.push_back(prng.frange(0.55f, 1.0f));
                        dists.push_back(radii[p] + radii[p + 1] -
                                        prng.frange(0.35f, 0.65f));
                        neck_pair.push_back(0);
                    } else {
                        // NECK regime: equal radii (exact margin), gap
                        // away from 0 and from the collapse boundary k/8.
                        radii.push_back(radii[p]);
                        const float g = prng.frange(0.02f, k / 8.0f - 0.02f);
                        dists.push_back(2.0f * radii[p] + g);
                        neck_pair.push_back(1);
                    }
                }
                // P3 margin check at the ACTUAL cell size: the hull
                // extent is r_first + sum(dists) + r_last; the extract
                // pads it by 1% and divides by the resolution.
                hull_extent = radii.front();
                for (std::size_t i = 0; i < dists.size(); ++i) hull_extent += dists[i];
                hull_extent += radii.back();
                const float cell = hull_extent * 1.02f / 24.0f;
                bool ok = true;
                for (std::size_t p = 0; p < neck_pair.size(); ++p) {
                    if (neck_pair[p] == 1) {
                        const float r = radii[p];  // equal pair
                        const float g = dists[p] - 2.0f * r;
                        const float rad = r + ks[p] / 16.0f;
                        const float half_gap = r + g / 2.0f;
                        if (rad <= half_gap) { ok = false; break; }  // no neck at all
                        const float neck_rad =
                            std::sqrt(rad * rad - half_gap * half_gap);
                        if (2.0f * neck_rad < 2.0f * cell) { ok = false; break; }
                    }
                }
                if (ok) break;
            }
        }
        const bool has_neck = std::find(neck_pair.begin(), neck_pair.end(), 1) != neck_pair.end();
        if (has_neck) ++neck_members; else ++overlap_members;

        // --- build the chain: nested smin along the generated direction ---
        const Vec3f base = off_grid_center(prng, -0.3f, 0.3f);
        const Vec3f dir = random_unit_dir(prng);
        s::SDFBody body = s::sdf_translate(s::make_sphere(radii[0]), base);
        Vec3f c = base;
        for (std::size_t p = 0; p + 1 < radii.size(); ++p) {
            c = c + dir * dists[p];
            s::SDFBody next = s::sdf_translate(s::make_sphere(radii[p + 1]), c);
            body = s::sdf_smooth_union(std::move(body), std::move(next), ks[p]);
        }

        SdfToBrepOptions opts;
        opts.fit_primitives = false;  // the fitting negative holds on blends
        opts.march_resolution = 24;   // declared above (cost rule)
        const SdfToBrepReport rep = sdf_to_brep_report(body, opts);

        INFO("seed=", seed, " iter=", iter,
             " radii=(", [&]{ std::string s; for (float r : radii) s += std::to_string(r) + " "; return s; }(), ")",
             " base=(", base.x, ",", base.y, ",", base.z, ")",
             " dir=(", dir.x, ",", dir.y, ",", dir.z, ")");
        INFO("dists=(", [&]{ std::string s; for (float d : dists) s += std::to_string(d) + " "; return s; }(), ")",
             " ks=(", [&]{ std::string s; for (float k : ks) s += std::to_string(k) + " "; return s; }(), ")",
             " neck=(", [&]{ std::string s; for (int v : neck_pair) s += std::to_string(v) + " "; return s; }(), ")",
             " hull_extent=", hull_extent);
        INFO("V=", rep.vertex_count, " E=", rep.edge_count, " F=", rep.face_count,
             " chi=", rep.chi, " genus=", rep.genus,
             " min|f|=", rep.min_abs_field, " fit=", rep.fit);

        INFO("check: fit == \"faceted\" (primitive fitting disabled — the negative on fitting holds on blends)");
        CHECK(rep.fit == "faceted");
        INFO("check: chi == 2 (genus 0, one shell)");
        CHECK(rep.chi == 2);
        INFO("check: genus derived == 0 (the derivation rule on chi = 2)");
        CHECK(rep.genus_derived);
        CHECK(rep.genus == 0);
        INFO("check: orientable (is_closed_orientable)");
        CHECK(rep.orientable);
        INFO("check: loops are closed");
        CHECK(rep.loops);
        INFO("check: E == 3F/2 (closed pure-triangle extract)");
        CHECK(rep.edge_count == 3 * rep.face_count / 2);
        INFO("check: V == F/2 + 2 (Euler identity for genus 0)");
        CHECK(2 * rep.vertex_count == rep.face_count + 4);
        INFO("check: min|f| > 0 on the sampling grid (P2 — no on-grid zeros)");
        CHECK(rep.min_abs_field > 0.0f);
    }
    // Regime liveness for THIS seed (deterministic): both the overlap
    // and the neck regime must actually occur.
    INFO("check: both regimes occur (neck members = ", neck_members,
         ", overlap-only members = ", overlap_members, ")");
    CHECK(neck_members >= 3);
    CHECK(overlap_members >= 3);
}
