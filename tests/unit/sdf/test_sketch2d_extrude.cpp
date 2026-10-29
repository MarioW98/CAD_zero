// tests/unit/sdf/test_sketch2d_extrude.cpp
//
// ADR-0030 D3, MVP unit 1 — the extrusion: SlabZ and ExtrudeSDF unit
// contracts, then the extraction contracts of D6 on extruded profiles
// (the disc = cylinder topology at the same grid, the ring = genus 1,
// determinism, the P2 margin).
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "CAD_0/bridge/sdf_to_brep.hpp"
#include "CAD_0/brep/topology.hpp"
#include "CAD_0/math/quat.hpp"
#include "CAD_0/sdf/primitives.hpp"
#include "CAD_0/sdf/sketch_extrude.hpp"
#include "CAD_0/sdf/transforms.hpp"
#include "property/property_prng.hpp"

using namespace CAD_0::sdf;
using namespace CAD_0::math;
using namespace CAD_0::bridge;
namespace b = CAD_0::brep;

namespace {

constexpr float kPi = 3.14159265358979323846f;

SDFBody extruded_disc(float r, float d, int dir) {
    return SDFBody(std::make_unique<ExtrudeSDF>(
        std::vector<SketchChain>{
            SketchChain{{sketch_circle({0.0f, 0.0f}, r, /*hole=*/false)}}},
        d, dir));
}

SDFBody extruded_ring(float r_out, float r_in, float d, int dir) {
    return SDFBody(std::make_unique<ExtrudeSDF>(
        std::vector<SketchChain>{
            SketchChain{{sketch_circle({0.0f, 0.0f}, r_out, /*hole=*/false)}},
            SketchChain{{sketch_circle({0.0f, 0.0f}, r_in, /*hole=*/true)}}},
        d, dir));
}

// The reference cylinder: y-aligned primitive rotated +90° about X
// (axis Y → Z) and translated to span z ∈ [0, d] — the same solid as
// the extruded disc, the EXACT primitive SDF against the composite
// field. The rotation's quaternion matrix carries ~3e-8 noise, so the
// padded grids differ at the 1e-8 scale — six orders below the 1% pad;
// the sign pattern at grid points is unaffected (measured, frozen).
SDFBody reference_cylinder(float r, float d) {
    return sdf_translate(
        sdf_rotate(make_cylinder(r, d),
                   Quatf::from_axis_angle({1.0f, 0.0f, 0.0f}, kPi * 0.5f)),
        {0.0f, 0.0f, d * 0.5f});
}

} // namespace

// --- SlabZ -----------------------------------------------------------------

TEST_CASE("SlabZ: the slab field, direction +Z, pinned") {
    const SlabZ slab(1.6f, +1);
    CHECK(slab.sample({0.0f, 0.0f, 0.8f}).value == doctest::Approx(-0.8f)); // mid-plane, deepest
    CHECK(slab.sample({5.0f, -3.0f, 0.0f}).value == doctest::Approx(0.0f).epsilon(1e-6f)); // cap
    CHECK(slab.sample({0.0f, 0.0f, 1.6f}).value == doctest::Approx(0.0f).epsilon(1e-6f)); // cap
    CHECK(slab.sample({0.0f, 0.0f, 2.6f}).value == doctest::Approx(1.0f));
    CHECK(slab.sample({0.0f, 0.0f, -0.5f}).value == doctest::Approx(0.5f));
    CHECK(slab.lipschitz() == 1.0f);
    // Gradient: (0, 0, ±1) by side of the mirror plane.
    CHECK(slab.sample({0.0f, 0.0f, 2.0f}).gradient.z == doctest::Approx(1.0f));
    CHECK(slab.sample({0.0f, 0.0f, -1.0f}).gradient.z == doctest::Approx(-1.0f));
    // Bounds: universe in x/y, the z-range [0, d].
    const auto bb = slab.bounds();
    CHECK(bb.min.x == -std::numeric_limits<float>::infinity());
    CHECK(bb.max.y == std::numeric_limits<float>::infinity());
    CHECK(bb.min.z == doctest::Approx(0.0f));
    CHECK(bb.max.z == doctest::Approx(1.6f));
}

TEST_CASE("SlabZ: direction -Z mirrors the range") {
    const SlabZ slab(1.6f, -1);
    CHECK(slab.sample({0.0f, 0.0f, -0.8f}).value == doctest::Approx(-0.8f));
    CHECK(slab.sample({0.0f, 0.0f, 0.0f}).value == doctest::Approx(0.0f).epsilon(1e-6f));
    CHECK(slab.sample({0.0f, 0.0f, -1.6f}).value == doctest::Approx(0.0f).epsilon(1e-6f));
    CHECK(slab.sample({0.0f, 0.0f, 0.9f}).value == doctest::Approx(0.9f)); // above the solid: distance to the cap z = 0
    const auto bb = slab.bounds();
    CHECK(bb.min.z == doctest::Approx(-1.6f));
    CHECK(bb.max.z == doctest::Approx(0.0f));
}

TEST_CASE("SlabZ: construction validation names the parameter") {
    REQUIRE_THROWS_AS(SlabZ(0.0f, +1), std::invalid_argument);
    REQUIRE_THROWS_AS(SlabZ(-0.5f, +1), std::invalid_argument);
    REQUIRE_THROWS_AS(SlabZ(std::numeric_limits<float>::quiet_NaN(), +1),
                      std::invalid_argument);
    REQUIRE_THROWS_AS(SlabZ(1.0f, 0), std::invalid_argument);
    REQUIRE_THROWS_AS(SlabZ(1.0f, 2), std::invalid_argument);
    try {
        SlabZ s(1.0f, 0);
        (void)s;
    } catch (const std::invalid_argument& e) {
        const std::string msg = e.what();
        CHECK(msg.find("SlabZ") != std::string::npos);
        CHECK(msg.find("direction") != std::string::npos);
    }
}

// --- ExtrudeSDF: the field ---------------------------------------------------

TEST_CASE("ExtrudeSDF: extruded disc — values, gradient branches, bounds") {
    const ExtrudeSDF ex(
        std::vector<SketchChain>{
            SketchChain{{sketch_circle({0.0f, 0.0f}, 0.7f, false)}}},
        1.6f, +1);

    CHECK(ex.lipschitz() == 1.0f);
    // Inside: the profile branch wins at (0.3, 0, 0.8).
    auto s = ex.sample({0.3f, 0.0f, 0.8f});
    CHECK(s.value == doctest::Approx(-0.4f)); // max(0.3-0.7, -0.8)
    CHECK(s.gradient.x == doctest::Approx(1.0f));
    CHECK(s.gradient.z == doctest::Approx(0.0f));
    // Lateral face: exact regime (f IS the distance mid-face).
    s = ex.sample({2.0f, 0.0f, 0.8f});
    CHECK(s.value == doctest::Approx(1.3f));
    CHECK(s.gradient.x == doctest::Approx(1.0f));
    // Cap: the slab branch wins above.
    s = ex.sample({0.0f, 0.0f, 2.6f});
    CHECK(s.value == doctest::Approx(1.0f));
    CHECK(s.gradient.z == doctest::Approx(1.0f));
    // Below the sketch plane.
    s = ex.sample({0.0f, 0.0f, -0.5f});
    CHECK(s.value == doctest::Approx(0.5f));
    CHECK(s.gradient.z == doctest::Approx(-1.0f));
    // On the cap and on the lateral surface: zero-set members.
    CHECK(ex.sample({0.3f, 0.0f, 1.6f}).value == doctest::Approx(0.0f).epsilon(1e-6f));
    CHECK(ex.sample({0.7f, 0.0f, 0.8f}).value == doctest::Approx(0.0f).epsilon(1e-6f));
    // Past the horizontal edge (lateral x cap, 90°): understated by up
    // to √2 (lie #4, live): f = 1.3, true corner distance sqrt(1.3²+1²).
    const float f_corner = ex.sample({2.0f, 0.0f, 2.6f}).value;
    CHECK(f_corner == doctest::Approx(1.3f));
    const float d_corner =
        (Vec3f{2.0f, 0.0f, 2.6f} - Vec3f{0.7f, 0.0f, 1.6f}).length();
    CHECK(d_corner == doctest::Approx(std::sqrt(1.3f * 1.3f + 1.0f)).epsilon(1e-5f));
    CHECK(f_corner <= d_corner); // the theorem, live at the edge

    // Bounds: bounds₂D × z-range.
    const auto bb = ex.bounds();
    CHECK(bb.min.x == doctest::Approx(-0.7f));
    CHECK(bb.max.x == doctest::Approx(0.7f));
    CHECK(bb.min.y == doctest::Approx(-0.7f));
    CHECK(bb.max.y == doctest::Approx(0.7f));
    CHECK(bb.min.z == doctest::Approx(0.0f));
    CHECK(bb.max.z == doctest::Approx(1.6f));
}

TEST_CASE("ExtrudeSDF: direction -Z mirrors the solid") {
    const ExtrudeSDF ex(
        std::vector<SketchChain>{
            SketchChain{{sketch_circle({0.0f, 0.0f}, 0.7f, false)}}},
        1.6f, -1);
    CHECK(ex.sample({0.3f, 0.0f, -0.8f}).value == doctest::Approx(-0.4f));
    CHECK(ex.sample({0.0f, 0.0f, 0.9f}).value == doctest::Approx(0.9f)); // above the solid: max(-0.7, |0.9+0.8|-0.8)
    const auto bb = ex.bounds();
    CHECK(bb.min.z == doctest::Approx(-1.6f));
    CHECK(bb.max.z == doctest::Approx(0.0f));
}

TEST_CASE("ExtrudeSDF: construction is via children (the D5 family)") {
    // A crossed chain (lie #5b's fixture) refuses HERE — the profile's
    // certificate runs inside the extrusion's construction.
    const std::vector<SketchChain> crossed{
        SketchChain{{sketch_segment({0.0f, 0.0f}, {1.0f, 0.0f}, true),
                     sketch_segment({1.0f, 0.0f}, {1.0f, 1.0f}, true),
                     sketch_segment({1.0f, 1.0f}, {0.0f, 1.0f}, true),
                     sketch_segment({0.0f, 1.0f}, {0.0f, 0.0f}, true)}},
        SketchChain{{sketch_circle({0.7f, 0.15f}, 0.2f, true)}}};
    REQUIRE_THROWS_AS(
        ExtrudeSDF(crossed, 1.0f, +1), std::invalid_argument);
    // The slab's own parameters, same family.
    REQUIRE_THROWS_AS(
        ExtrudeSDF(std::vector<SketchChain>{
                       SketchChain{{sketch_circle({0, 0}, 1.0f, false)}}},
                   0.0f, +1),
        std::invalid_argument);
    REQUIRE_THROWS_AS(
        ExtrudeSDF(std::vector<SketchChain>{
                       SketchChain{{sketch_circle({0, 0}, 1.0f, false)}}},
                   1.0f, 0),
        std::invalid_argument);
}

TEST_CASE("ExtrudeSDF: L = 1 — pairwise 3D slopes (seeded)") {
    CAD_0::test::property::Prng prng(0x5EED2D04u);
    const ExtrudeSDF ex(
        std::vector<SketchChain>{
            SketchChain{{sketch_circle({0.0f, 0.0f}, 0.7f, false)}}},
        1.6f, +1);
    int checked = 0;
    for (int pt = 0; pt < 128; ++pt) {
        const Vec3f p{prng.frange(-1.5f, 1.5f), prng.frange(-1.5f, 1.5f),
                      prng.frange(-0.6f, 2.2f)};
        const Vec3f q{prng.frange(-1.5f, 1.5f), prng.frange(-1.5f, 1.5f),
                      prng.frange(-0.6f, 2.2f)};
        const float d = (p - q).length();
        if (d < 1e-3f) continue;
        CHECK(std::abs(ex.sample(p).value - ex.sample(q).value) <= d + 1e-5f);
        ++checked;
    }
    CHECK(checked > 100); // liveness
}

// --- The extraction contracts (D6) -------------------------------------------

TEST_CASE("Extraction: extruded disc = cylinder topology at the same grid "
           "(same signs ⇒ same V/E/F, positions NOT bit-identical, bounded delta)") {
    SdfToBrepOptions opts;
    opts.fit_primitives = false;
    opts.march_resolution = 16;

    auto a = sdf_to_brep_faceted(extruded_disc(0.7f, 1.6f, +1), opts);
    auto c = sdf_to_brep_faceted(reference_cylinder(0.7f, 1.6f), opts);
    REQUIRE(a != nullptr);
    REQUIRE(c != nullptr);

    // Same MC configuration: identical V/E/F (measured once, frozen).
    CHECK(a->vertex_count() == c->vertex_count());
    CHECK(a->edge_count() == c->edge_count());
    CHECK(a->face_count() == c->face_count());
    // Pin the counts themselves (extruded disc r=0.7, d=1.6, res=16).
    CHECK(a->vertex_count() == 1286);
    CHECK(a->edge_count() == 3852);
    CHECK(a->face_count() == 2568);
    // The closed-mesh identity.
    CHECK(a->edge_count() == 3 * a->face_count() / 2);

    // Grid-independent contracts.
    CHECK(b::euler_characteristic(*a) == 2);
    CHECK(b::is_closed_manifold(*a));
    CHECK(b::loops_are_closed(*a));
    for (std::size_t e = 0; e < a->edge_count(); ++e) {
        const auto& ce = a->edges[e].coedges();
        REQUIRE(ce.size() == 2);
        CHECK(ce[0]->reversed() != ce[1]->reversed());
    }

    // Positions are NOT bit-identical (the composite interpolates
    // differently than the exact cylinder — declared) ...
    REQUIRE(a->vertex_count() == c->vertex_count());
    bool any_diff = false;
    for (std::size_t i = 0; i < a->vertex_count(); ++i)
        if (a->vertices[i].position() != c->vertices[i].position())
            any_diff = true;
    CHECK(any_diff);
    // ... but bounded: corresponding vertices live in the same cells,
    // so every vertex of one extraction is within one grid step of
    // some vertex of the other.
    const float step = 2.0f * 0.7f * 1.02f / 15.0f; // padded extent / (res-1), x axis
    for (std::size_t i = 0; i < a->vertex_count(); ++i) {
        const Vec3d& va = a->vertices[i].position();
        double best = std::numeric_limits<double>::infinity();
        for (std::size_t j = 0; j < c->vertex_count(); ++j)
            best = std::min(best, (va - c->vertices[j].position()).length());
        CHECK(best <= 1.01 * step);
    }

    // P2: the pinned grid has no on-surface grid point.
    const auto rep = sdf_to_brep_report(extruded_disc(0.7f, 1.6f, +1), opts);
    CHECK(rep.chi == 2);
    CHECK(rep.min_abs_field > 0.0f);
}

TEST_CASE("Extraction: extruded ring = genus 1 — pinned counts, derived "
           "genus, the wrapper keeps telling its truth") {
    SdfToBrepOptions opts;
    opts.fit_primitives = false;
    opts.march_resolution = 48;
    auto body = sdf_to_brep_faceted(extruded_ring(1.0f, 0.4f, 1.0f, +1), opts);
    REQUIRE(body != nullptr);

    // Pinned (ring r_out=1, r_in=0.4, d=1, res=48): measured once, frozen.
    CHECK(body->vertex_count() == 15336);
    CHECK(body->edge_count() == 46008);
    CHECK(body->face_count() == 30672);
    CHECK(body->edge_count() == 3 * body->face_count() / 2);

    // Grid-independent contracts, genus-1 edition.
    CHECK(b::euler_characteristic(*body) == 0);
    CHECK(b::is_closed_orientable(*body));
    CHECK(b::loops_are_closed(*body));
    // The compatibility wrapper keeps its exact historical meaning
    // (structural ∧ χ == 2): the genus-1 ring FAILS it — the wrapper
    // telling the truth about its own contract.
    CHECK_FALSE(b::is_closed_manifold(*body));
    for (std::size_t e = 0; e < body->edge_count(); ++e) {
        const auto& ce = body->edges[e].coedges();
        REQUIRE(ce.size() == 2);
        CHECK(ce[0]->reversed() != ce[1]->reversed());
    }

    // The report: χ = 0, genus DERIVED = 1, orientable, loops; the P2
    // margin measured once and frozen.
    const auto rep = sdf_to_brep_report(extruded_ring(1.0f, 0.4f, 1.0f, +1), opts);
    CHECK(rep.fit == "faceted");
    CHECK(rep.chi == 0);
    CHECK(rep.orientable);
    CHECK(rep.genus_derived);
    CHECK(rep.genus == 1);
    CHECK_FALSE(rep.closed);
    CHECK(rep.loops);
    CHECK(rep.min_abs_field > 0.0f);
    CHECK(rep.min_abs_field == 0.000331163406f);
}

TEST_CASE("Extraction: extruded ring — determinism, bit-identical re-run") {
    auto extract = [] {
        SdfToBrepOptions opts;
        opts.fit_primitives = false;
        opts.march_resolution = 48;
        return sdf_to_brep_faceted(extruded_ring(1.0f, 0.4f, 1.0f, +1), opts);
    };
    auto first = extract();
    auto second = extract();
    REQUIRE(first != nullptr);
    REQUIRE(second != nullptr);

    CHECK(first->vertex_count() == second->vertex_count());
    CHECK(first->edge_count() == second->edge_count());
    CHECK(first->face_count() == second->face_count());
    REQUIRE(first->vertex_count() == second->vertex_count());
    for (std::size_t i = 0; i < first->vertex_count(); ++i)
        CHECK(first->vertices[i].position() == second->vertices[i].position());
}

TEST_CASE("Extraction: extruded ring at a second grid — counts differ, "
           "chi = 0 does not (the T2/T14 habit)") {
    SdfToBrepOptions opts;
    opts.fit_primitives = false;
    opts.march_resolution = 32;
    auto body = sdf_to_brep_faceted(extruded_ring(1.0f, 0.4f, 1.0f, +1), opts);
    REQUIRE(body != nullptr);
    // Different counts than res=48 (grid-dependent), same topological class.
    CHECK(body->vertex_count() != 15336);
    CHECK(b::euler_characteristic(*body) == 0);
    CHECK(b::is_closed_orientable(*body));
    CHECK(b::loops_are_closed(*body));
}
