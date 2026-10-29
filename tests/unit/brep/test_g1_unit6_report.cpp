// tests/unit/brep/test_g1_unit6_report.cpp
//
// G1 UNIT 6 — THE REPORT AND THE GUIDE CASE END-TO-END: the
// discriminating tests (ADR-0032 D8 + D10.6). The report is the
// single consumer of the unit-5 mass API and the unit-2/6 queries —
// what THIS file pins is the consumer's own foundations:
//
//   * THE BODY QUERY (query_body, the report's topology and
//     provenance source): the guide case's whole answer (12/17/7/2,
//     chi = 0, genus 1, the per-kind tallies all exact); the factory
//     bodies' answers (the backward-compatibility proof's own
//     numbers); the faceted world's answer (every subshape
//     approximated, the max declared tolerance carried, never
//     laundered); the empty body's refusal (the P2 culture — no
//     shell, no summary); the multi-shell genus n/d (the D6 rule,
//     mirrored from the bridge report);
//   * THE NO-SECOND-OPINION CHECK: query_body's every number equals
//     the DIRECT authority's answer (euler_characteristic,
//     inner_loop_count, genus_from_chi) on the same body — the
//     aggregator aggregates, it never computes;
//   * THE GUIDE CASE'S GOLDEN (the v0.4 document): loads, exact
//     materialization, and the report's numbers ARE the D7.5 table —
//     abc - pi r^2 h, the axial I_zz, the counts — pinned with the
//     suites' precision doctrine and bit-identical on re-query;
//   * THE v0.3 GOLDEN (the honest faceted twin): loads, the exact
//     path refuses (the Y-axis tool — the refusal recorded), and the
//     faceted report path agrees with the table where it can (chi = 0,
//     genus 1 — the same topology) while declaring itself
//     approximated where it must (all_exact = false, the polyhedral
//     volume within the grid's reach, never presented as exact).
//
// The report TEXT itself (the rendering) is pinned on the Python
// side — bindings/python/tests/test_g1_unit6_report.py — where the
// UI and the example consume it; the C++ side pins the DATA.
#include <doctest/doctest.h>

#include "CAD_0/brep/factory.hpp"
#include "CAD_0/brep/query.hpp"
#include "CAD_0/brep/mass_properties.hpp"
#include "CAD_0/brep/subtract.hpp"
#include "CAD_0/brep/extrude.hpp"
#include "CAD_0/brep/topology.hpp"
#include "CAD_0/brep/boolean.hpp"
#include "CAD_0/sdf/sketch_entities.hpp"
#include "CAD_0/bridge/sdf_to_brep.hpp"
#include "CAD_0/scene/scene.hpp"
#include "CAD_0/io/native.hpp"

#include <cmath>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <vector>

namespace {

namespace b = CAD_0::brep;
namespace s = CAD_0::sdf;
using CAD_0::brep::BRepBody;
using CAD_0::math::Vec2f;
using CAD_0::math::Vec3d;

constexpr double kPid = 3.141592653589793;

// The unit-5 precision doctrine, verbatim: closed-form double
// machinery over float trimming inputs — a few float ulps.
bool close_d(double a, double b, double rel = 1e-6,
             double abs_tol = 1e-6) {
    return std::abs(a - b) <=
           std::max(abs_tol, rel * std::max(std::abs(a), std::abs(b)));
}

// The circle profile: the guide case's tool (the unit-4 vocabulary).
std::vector<s::SketchChain> circle_chains(Vec2f center = {0.0f, 0.0f},
                                          float radius = 0.25f) {
    return {s::SketchChain{{s::sketch_circle(center, radius, false)}}};
}

// The guide case: box [-1,1]^3 minus the circle prism r=0.25
// through Z (the prism spans z in [-1.5, 1.5], a strict through-hole).
std::unique_ptr<BRepBody> guide_body() {
    auto a = b::make_box({1.0f, 1.0f, 1.0f});
    auto tool = b::make_prism(circle_chains(), 3.0f, +1);
    auto r = b::exact_subtract(*a, *tool, {0.0f, 0.0f, -1.5f},
                               b::kSubtractTolerance);
    REQUIRE(r != nullptr);
    return r;
}

// The no-second-opinion oracle: the aggregator's every number equals
// the direct authority's own answer on the same body.
void check_mirrors_the_authorities(const BRepBody& body) {
    const b::BodySummary q = b::query_body(body);
    CHECK(q.vertex_count == body.vertex_count());
    CHECK(q.edge_count == body.edge_count());
    CHECK(q.face_count == body.face_count());
    CHECK(q.inner_loops == b::inner_loop_count(body));
    CHECK(q.chi == b::euler_characteristic(body));
    CHECK(q.orientable == b::is_closed_orientable(body));
    CHECK(q.loops_closed == b::loops_are_closed(body));
    const auto direct_genus = b::genus_from_chi(b::euler_characteristic(body));
    CHECK(q.genus_derived == (q.orientable && direct_genus.has_value()));
    if (q.genus_derived)
        CHECK(q.genus == direct_genus.value());
}

// The all-exact tallies (the exact world's own).
void check_all_exact_tallies(const b::BodySummary& q) {
    CHECK(q.vertices.approximated == 0);
    CHECK(q.edges.approximated == 0);
    CHECK(q.faces.approximated == 0);
    CHECK(q.vertices.exact == q.vertex_count);
    CHECK(q.edges.exact == q.edge_count);
    CHECK(q.faces.exact == q.face_count);
    CHECK(q.vertices.max_linear_tolerance == 0.0);
    CHECK(q.edges.max_linear_tolerance == 0.0);
    CHECK(q.faces.max_linear_tolerance == 0.0);
}

} // namespace

// ---------------------------------------------------------------------------
// FAMILY 1 — THE BODY QUERY: the guide case's whole answer (D7.5's
// reported column, at the source).
// ---------------------------------------------------------------------------
TEST_CASE("G1-U6 query_body: the guide case - 12/17/7/2, chi 0, genus 1, all exact") {
    auto body = guide_body();
    const b::BodySummary q = b::query_body(*body);

    CHECK(q.vertex_count == 12);
    CHECK(q.edge_count == 17);
    CHECK(q.face_count == 7);
    CHECK(q.inner_loops == 2);
    CHECK(q.chi == 0);
    CHECK(q.orientable);
    CHECK(q.loops_closed);
    REQUIRE(q.genus_derived);
    CHECK(q.genus == 1);
    check_all_exact_tallies(q);

    // The no-second-opinion check, on the guide body itself.
    check_mirrors_the_authorities(*body);

    // Determinism (ADR-0014): the same body, re-queried, answers the
    // same summary field by field.
    const b::BodySummary q2 = b::query_body(*body);
    CHECK(q2.vertex_count == q.vertex_count);
    CHECK(q2.edge_count == q.edge_count);
    CHECK(q2.face_count == q.face_count);
    CHECK(q2.inner_loops == q.inner_loops);
    CHECK(q2.chi == q.chi);
    CHECK(q2.genus == q.genus);
    CHECK(q2.vertices.exact == q.vertices.exact);
    CHECK(q2.faces.exact == q.faces.exact);
}

TEST_CASE("G1-U6 query_body: the factory bodies - the backward-compat numbers") {
    // The box: 8/12/6, H = 0, chi = 2, genus 0, all exact.
    {
        auto box = b::make_box({1.0f, 2.0f, 3.0f});
        const b::BodySummary q = b::query_body(*box);
        CHECK(q.vertex_count == 8);
        CHECK(q.edge_count == 12);
        CHECK(q.face_count == 6);
        CHECK(q.inner_loops == 0);
        CHECK(q.chi == 2);
        REQUIRE(q.genus_derived);
        CHECK(q.genus == 0);
        check_all_exact_tallies(q);
        check_mirrors_the_authorities(*box);
    }
    // The sphere: 2/1/1 (the seam wire), chi = 2, genus 0.
    {
        auto sph = b::make_sphere(1.0f);
        const b::BodySummary q = b::query_body(*sph);
        CHECK(q.vertex_count == 2);
        CHECK(q.edge_count == 1);
        CHECK(q.face_count == 1);
        CHECK(q.chi == 2);
        REQUIRE(q.genus_derived);
        CHECK(q.genus == 0);
        check_all_exact_tallies(q);
    }
    // The cylinder: 4/5/3, chi = 2, genus 0.
    {
        auto cyl = b::make_cylinder(0.5f, 2.0f);
        const b::BodySummary q = b::query_body(*cyl);
        CHECK(q.vertex_count == 4);
        CHECK(q.edge_count == 5);
        CHECK(q.face_count == 3);
        CHECK(q.chi == 2);
        REQUIRE(q.genus_derived);
        CHECK(q.genus == 0);
    }
}

TEST_CASE("G1-U6 query_body: the FACETED world - approximated tallies, the declared bound, never laundered") {
    // The v0.3-style body: the subtract whose exact path refuses (the
    // Y-axis primitive-cylinder tool — the unit-4 landing's own
    // pinned refusal), so the report's body is the faceted extract
    // of its SDF twin (resolution 24, fit OFF): every subshape
    // approximated, the max tolerance the grid's own declared bound.
    CAD_0::scene::Scene sc;
    const auto box_id = sc.add_node(
        "Box", s::make_box(CAD_0::math::Vec3f{1.0f, 1.0f, 1.0f}));
    const auto tool_id = sc.add_node(
        "Cyl", s::make_cylinder(0.25f, 2.0f));
    CAD_0::scene::OperationRecipe r;
    r.type = CAD_0::scene::OperationType::Subtract;
    r.target = box_id;
    r.tool = tool_id;
    const auto sub_id = sc.add_operation_node("Sub", r);
    const auto* node = sc.get_node(sub_id);
    REQUIRE(node != nullptr);
    // The exact path refuses the Y-axis primitive cylinder (the
    // unit-4 landing's own pinned refusal) — the honest remainder is
    // the SDF, and the faceted extract of it is the report's body.
    CHECK(node->hybrid == nullptr);
    REQUIRE_FALSE(node->hybrid_refusal.empty());

    CAD_0::bridge::SdfToBrepOptions opts;
    opts.fit_primitives = false;
    opts.march_resolution = 24;
    auto faceted = CAD_0::bridge::sdf_to_brep_faceted(node->body, opts);
    REQUIRE(faceted != nullptr);

    const b::BodySummary q = b::query_body(*faceted);
    // Same topology, coarser geometry: chi = 0, genus 1 (the honest
    // agreement the ADR pins — the faceted extract of the pierced
    // box is the same genus-1 shape).
    CHECK(q.orientable);
    CHECK(q.loops_closed);
    REQUIRE(q.genus_derived);
    CHECK(q.genus == 1);
    CHECK(q.chi == 0);

    // The D8 honesty: EVERY subshape approximated, the max tolerance
    // the declared grid bound (> 0), exact counts all zero.
    CHECK(q.vertices.exact == 0);
    CHECK(q.edges.exact == 0);
    CHECK(q.faces.exact == 0);
    CHECK(q.vertices.approximated == q.vertex_count);
    CHECK(q.edges.approximated == q.edge_count);
    CHECK(q.faces.approximated == q.face_count);
    CHECK(q.vertices.max_linear_tolerance > 0.0);
    CHECK(q.edges.max_linear_tolerance > 0.0);
    CHECK(q.faces.max_linear_tolerance > 0.0);
    check_mirrors_the_authorities(*faceted);

    // The mass API's aggregate agrees with the tallies (D8: the
    // all_exact flag IS the same field, aggregated) — and the value
    // is the polyhedral one, within the grid's reach of the table.
    const auto mp = b::mass_properties(*faceted,
                                       CAD_0::math::Tolerance::linear(1e-6));
    CHECK_FALSE(mp.all_exact);
    const double abc = 8.0;
    const double table = abc - kPid * 0.25 * 0.25 * 2.0;
    CHECK(close_d(mp.volume, table, 2e-2, 2e-2));
}

TEST_CASE("G1-U6 query_body: the EMPTY body refuses - the P2 culture") {
    const BRepBody empty;
    bool threw = false;
    try {
        (void)b::query_body(empty);
    } catch (const std::invalid_argument& exc) {
        threw = true;
        const std::string msg = exc.what();
        // Diagnosis + remedy, the construction-time culture: the
        // message names the empty body and the remedy.
        CHECK(msg.find("empty body") != std::string::npos);
        CHECK(msg.find("remedy") != std::string::npos);
    }
    CHECK(threw);
}

TEST_CASE("G1-U6 query_body: the MULTI-SHELL body - genus n/d, the D6 rule") {
    // Two disjoint boxes in one body (the union's honest assembly,
    // vertex-translated the test_boolean way): chi = 4 > 2, so the
    // genus is NOT derived — "n/d", never a fabricated half-integer
    // (the ADR-0027 D6 rule the bridge report already applies,
    // mirrored here). Verified empirically before pinning: the
    // disjoint union keeps both shells, closed and orientable.
    auto a = b::make_box({0.5f, 0.5f, 0.5f});
    auto far = b::make_box({0.5f, 0.5f, 0.5f});
    for (auto& v : far->vertices)
        v.set_position(v.position() + Vec3d{3.0f, 0, 0});
    auto uni = b::bool_union(*a, *far);
    REQUIRE(uni != nullptr);

    const b::BodySummary q = b::query_body(*uni);
    CHECK(q.vertex_count == 16);
    CHECK(q.face_count == 12);
    CHECK(q.orientable);
    CHECK(q.chi == 4);  // two shells: 2 + 2
    CHECK_FALSE(q.genus_derived);
    check_mirrors_the_authorities(*uni);
}

// ---------------------------------------------------------------------------
// FAMILY 2 — THE GOLDENS: the guide case pinned with its numbers
// (v0.4), the honest faceted twin (v0.3).
// ---------------------------------------------------------------------------
TEST_CASE("G1-U6 golden v0.4: the guide body's report IS the D7.5 table") {
    const std::filesystem::path golden =
        std::filesystem::path(CAD_0_SOURCE_DIR) / "tests" / "golden" /
        "native_v0_4_0.CAD_0";
    REQUIRE(std::filesystem::exists(golden));

    auto doc = CAD_0::io::native::load_file(golden);
    CAD_0::scene::Scene sc;
    CAD_0::io::native::install_document(sc, std::move(doc));

    const auto* sub = sc.get_node(4);
    REQUIRE(sub != nullptr);
    REQUIRE(sub->hybrid != nullptr);
    CHECK(sub->hybrid_refusal.empty());

    // The topology block: the pinned table.
    const b::BodySummary q = b::query_body(*sub->hybrid->brep);
    CHECK(q.vertex_count == 12);
    CHECK(q.edge_count == 17);
    CHECK(q.face_count == 7);
    CHECK(q.inner_loops == 2);
    CHECK(q.chi == 0);
    REQUIRE(q.genus_derived);
    CHECK(q.genus == 1);
    check_all_exact_tallies(q);

    // The mass block: the D7.5 formulas (a = b = c = 2, r = 0.25,
    // h = 2 — the pierced span), the precision doctrine's bounds.
    const auto mp = b::mass_properties(*sub->hybrid->brep,
                                       CAD_0::math::Tolerance::linear(1e-6));
    REQUIRE(mp.all_exact);
    const double abc = 8.0;
    CHECK(close_d(mp.volume, abc - kPid * 0.0625 * 2.0));
    CHECK(close_d(mp.centroid.x, 0.0, 1e-6, 1e-9));
    CHECK(close_d(mp.centroid.y, 0.0, 1e-6, 1e-9));
    CHECK(close_d(mp.centroid.z, 0.0, 1e-6, 1e-9));
    CHECK(close_d(mp.inertia_about_centroid(2, 2),
                  abc * 8.0 / 12.0 - kPid * 0.25 * 0.25 * 0.25 * 0.25
                      * 2.0 / 2.0));
    CHECK(close_d(mp.inertia_about_centroid(0, 0),
                  abc * 8.0 / 12.0 -
                      kPid * 0.0625 * 2.0 * (3.0 * 0.0625 + 4.0) / 12.0,
                  1e-5));
    // The symmetric body: the products vanish.
    CHECK(close_d(mp.inertia_about_centroid(0, 1), 0.0, 1e-6, 1e-9));

    // Determinism: the re-query and the re-computation are
    // bit-identical (the golden re-runs the same bytes).
    const b::BodySummary q2 = b::query_body(*sub->hybrid->brep);
    CHECK(q2.chi == q.chi);
    const auto mp2 = b::mass_properties(*sub->hybrid->brep,
                                        CAD_0::math::Tolerance::linear(1e-6));
    CHECK(mp2.volume == mp.volume);
    CHECK(mp2.inertia_about_centroid(2, 2) == mp.inertia_about_centroid(2, 2));
}

TEST_CASE("G1-U6 golden v0.3: the honest faceted twin - refused exact, agreed topology") {
    const std::filesystem::path golden =
        std::filesystem::path(CAD_0_SOURCE_DIR) / "tests" / "golden" /
        "native_v0_3_0.CAD_0";
    REQUIRE(std::filesystem::exists(golden));

    auto doc = CAD_0::io::native::load_file(golden);
    CAD_0::scene::Scene sc;
    CAD_0::io::native::install_document(sc, std::move(doc));

    const auto* sub = sc.get_node(3);
    REQUIRE(sub != nullptr);
    // The Y-axis primitive-cylinder tool: the exact path refuses (the
    // unit-4 landing's pinned refusal — the honesty, not a failure).
    CHECK(sub->hybrid == nullptr);
    REQUIRE_FALSE(sub->hybrid_refusal.empty());

    // The report path for this body is the dispatching extraction —
    // the faceted one (the tool is not a prism, the target-minus-tool
    // is not a primitive): chi = 0, genus 1 (the same topology as the
    // table), every number approximated, never an exact in disguise.
    CAD_0::bridge::SdfToBrepOptions opts;
    opts.march_resolution = 48;
    auto faceted = CAD_0::bridge::sdf_to_brep_faceted(sub->body, opts);
    REQUIRE(faceted != nullptr);

    const b::BodySummary q = b::query_body(*faceted);
    CHECK(q.chi == 0);
    CHECK(q.orientable);
    REQUIRE(q.genus_derived);
    CHECK(q.genus == 1);
    CHECK(q.vertices.exact == 0);
    CHECK(q.faces.approximated == q.face_count);

    const auto mp = b::mass_properties(*faceted,
                                       CAD_0::math::Tolerance::linear(1e-6));
    CHECK_FALSE(mp.all_exact);
    // The v0.3 golden's own table: box 2x2x2 pierced by the cylinder
    // r = 0.25 (the pierced span 2): abc - pi r^2 h, within the
    // grid's reach at resolution 48.
    const double table = 8.0 - kPid * 0.0625 * 2.0;
    CHECK(close_d(mp.volume, table, 2e-2, 2e-2));
}
