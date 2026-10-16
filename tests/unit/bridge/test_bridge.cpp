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
    CHECK(body->face_count() > 0);
    CHECK(body->vertex_count() > 0);
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
    CHECK(body->face_count() > 0);
    for (const auto& face : body->solid.shell().faces()) {
        CHECK(std::string(face.surface()->type_name()) == "plane");
    }
}

TEST_CASE("Bridge: B-Rep sphere → SDF sphere") {
    auto body = b::make_sphere(2.0f);
    auto sdf_body = brep_to_sdf(*body);

    REQUIRE(sdf_body != nullptr);
    CHECK(sdf_body->value(Vec3f{0, 0, 0}) == doctest::Approx(-2.0f).epsilon(0.01f));
    CHECK(sdf_body->value(Vec3f{3, 0, 0}) == doctest::Approx(1.0f).epsilon(0.01f));
}

TEST_CASE("Bridge: B-Rep box → SDF box") {
    auto body = b::make_box(Vec3f{1, 1, 1});
    auto sdf_body = brep_to_sdf(*body);

    REQUIRE(sdf_body != nullptr);
    CHECK(sdf_body->value(Vec3f{0, 0, 0}) < 0.0f);
    CHECK(sdf_body->value(Vec3f{5, 0, 0}) > 0.0f);
}

TEST_CASE("Bridge: B-Rep cylinder → SDF cylinder") {
    auto body = b::make_cylinder(1.0f, 4.0f);
    auto sdf_body = brep_to_sdf(*body);

    REQUIRE(sdf_body != nullptr);
    CHECK(sdf_body->value(Vec3f{0, 0, 0}) < 0.0f);
    CHECK(sdf_body->value(Vec3f{5, 0, 0}) > 0.0f);
}

TEST_CASE("Bridge: round-trip sphere SDF → B-Rep → SDF") {
    auto original = s::make_sphere(2.0f);
    auto body = sdf_to_brep(original);
    auto roundtrip = brep_to_sdf(*body);

    REQUIRE(roundtrip != nullptr);
    CHECK(roundtrip->value(Vec3f{0, 0, 0}) == doctest::Approx(-2.0f).epsilon(0.01f));
    CHECK(roundtrip->value(Vec3f{3, 0, 0}) == doctest::Approx(1.0f).epsilon(0.01f));
}

TEST_CASE("Bridge: round-trip box SDF → B-Rep → SDF") {
    auto original = s::make_box(Vec3f{1, 1, 1});
    auto body = sdf_to_brep(original);
    auto roundtrip = brep_to_sdf(*body);

    REQUIRE(roundtrip != nullptr);
    CHECK(roundtrip->value(Vec3f{0, 0, 0}) < 0.0f);
    CHECK(roundtrip->value(Vec3f{5, 0, 0}) > 0.0f);
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
    CHECK(body->face_count() > 0);

    auto sdf_body = brep_to_sdf(*body);
    REQUIRE(sdf_body != nullptr);

    auto bnds = sdf_body->bounds();
    CHECK(!bnds.empty());
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
