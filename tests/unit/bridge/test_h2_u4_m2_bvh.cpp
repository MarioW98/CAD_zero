// tests/unit/bridge/test_h2_u4_m2_bvh.cpp
//
// H2-u4 MORSO 2 — THE BVH OF THE WINDING NUMBER: the discriminant
// tests. The hierarchy approximates far nodes under the DECLARED
// budget ε_w with every approximation bounded by the derived ε_node
// (ADR-0035 D2.5); these tests prove the bound empirically point by
// point (not asserted by faith), pin the classification invariance
// (zero sign flips against morso 1's exact evaluation on the whole
// corpus), replicate morso 1's stress through the hierarchy, measure
// the speed number (a number, not an adjective), pin the
// determinism (N rebuilds, one digest), hold the watertight refusal
// in front of the build, and prove the counterproof: at budget zero
// the machine answers with the exact evaluation — the escape route
// from its own optimization exists and is taken.
#include <doctest/doctest.h>

#include "CAD_0/bridge/brep_to_sdf.hpp"
#include "CAD_0/bridge/detail/winding_test_hooks.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace CAD_0::bridge;
using CAD_0::math::Vec3d;
using CAD_0::math::Vec3f;

// ---------------------------------------------------------------------------
// The corpus meshes — m1's builders (tests/unit/bridge/test_bridge.cpp),
// self-contained here so this TU owns its own corpus.
// ---------------------------------------------------------------------------

// The closed cube [0,2]^3, 8 vertices / 12 triangles, every face
// outward-oriented (verified per-face in m1's builder).
static CAD_0::sdf::TriangleMesh closed_cube_mesh() {
    CAD_0::sdf::TriangleMesh m;
    const float c = 2.0f;
    m.positions = {
        {0, 0, 0}, {c, 0, 0}, {c, c, 0}, {0, c, 0},
        {0, 0, c}, {c, 0, c}, {c, c, c}, {0, c, c}};
    m.indices = {
        3, 2, 1,  3, 1, 0,   // bottom
        4, 5, 6,  4, 6, 7,   // top
        0, 1, 5,  0, 5, 4,   // front
        2, 3, 7,  2, 7, 6,   // back
        3, 0, 4,  3, 4, 7,   // left
        1, 2, 6,  1, 6, 5};  // right
    return m;
}

// The closed L-shaped prism (m1's non-convex corpus mesh): section
// (0,0),(3,0),(3,1),(1,1),(1,3),(0,3), extruded z in [0,3].
// 12 vertices / 20 triangles, all outward.
static CAD_0::sdf::TriangleMesh closed_l_mesh() {
    CAD_0::sdf::TriangleMesh m;
    const float X = 3.0f, Y1 = 1.0f, Y2 = 3.0f, X1 = 1.0f, H = 3.0f;
    m.positions = {
        {0, 0, 0}, {X, 0, 0}, {X, Y1, 0}, {X1, Y1, 0}, {X1, Y2, 0}, {0, Y2, 0},
        {0, 0, H}, {X, 0, H}, {X, Y1, H}, {X1, Y1, H}, {X1, Y2, H}, {0, Y2, H}};
    m.indices = {
        0, 2, 1,  0, 3, 2,  0, 4, 3,  0, 5, 4,
        6, 7, 8,  6, 8, 9,  6, 9, 10,  6, 10, 11,
        0, 1, 7,   0, 7, 6,
        1, 2, 8,   1, 8, 7,
        2, 3, 9,   2, 9, 8,
        3, 4, 10,  3, 10, 9,
        4, 5, 11,  4, 11, 10,
        5, 0, 6,   5, 6, 11};
    return m;
}

// The corner tetrahedron translated to 1e6 (m1's far-from-origin
// stress mesh): all vertices exact in float32, the INPUT honest.
static CAD_0::sdf::TriangleMesh far_tetra_mesh() {
    const float T = 1000000.0f;
    CAD_0::sdf::TriangleMesh m;
    m.positions = {
        {T, T, T}, {T + 2, T, T}, {T, T + 2, T}, {T, T, T + 2}};
    m.indices = {0, 3, 2, 0, 1, 3, 0, 2, 1, 1, 2, 3};
    return m;
}

// The high-tessellation sphere for the speed number (B4.4): a UV
// sphere, radius 2, 52 latitude rings (2 poles + 50 full), 100
// longitude segments — 5002 vertices / 10000 triangles exactly
// (χ = V − E + F = 5002 − 15000 + 10000 = 2, a closed shell). The
// orientation is the parametrization's own (∂φ × ∂θ = sinφ·r̂ points
// OUT everywhere), the caps verified against the quad rows' edge
// directions; the watertight audit at load is the machine's own
// confirmation.
static CAD_0::sdf::TriangleMesh uv_sphere_mesh(double radius, int rings,
                                               int segments) {
    CAD_0::sdf::TriangleMesh m;
    const int full = rings - 2; // the non-pole rings
    m.positions.reserve(static_cast<std::size_t>(2 + full * segments));
    m.positions.push_back(
        Vec3f{0, 0, static_cast<float>(radius)}); // north pole (v0)
    for (int i = 1; i <= full; ++i) {
        const double phi = M_PI * static_cast<double>(i) /
                           static_cast<double>(rings - 1);
        for (int j = 0; j < segments; ++j) {
            const double theta = 2.0 * M_PI * static_cast<double>(j) /
                                 static_cast<double>(segments);
            m.positions.push_back(Vec3f{
                static_cast<float>(radius * std::sin(phi) * std::cos(theta)),
                static_cast<float>(radius * std::sin(phi) * std::sin(theta)),
                static_cast<float>(radius * std::cos(phi))});
        }
    }
    m.positions.push_back(
        Vec3f{0, 0, static_cast<float>(-radius)}); // south pole (last)
    const auto ring_vertex = [&](int i, int j) -> std::uint32_t {
        // i in [1, full]; the ring block starts after the north pole.
        return static_cast<std::uint32_t>(1 + (i - 1) * segments +
                                          (j % segments));
    };
    const std::uint32_t north = 0;
    const std::uint32_t south =
        static_cast<std::uint32_t>(m.positions.size() - 1);

    m.indices.reserve(static_cast<std::size_t>(2 * segments * rings));
    // North cap: (pole, v(1,j), v(1,j+1)) — outward.
    for (int j = 0; j < segments; ++j)
        m.indices.insert(m.indices.end(),
                         {north, ring_vertex(1, j), ring_vertex(1, j + 1)});
    // The quad rows: (v(i,j), v(i+1,j), v(i+1,j+1)) and
    // (v(i,j), v(i+1,j+1), v(i,j+1)) — outward by the parametrization.
    for (int i = 1; i < full; ++i) {
        for (int j = 0; j < segments; ++j) {
            m.indices.insert(m.indices.end(),
                             {ring_vertex(i, j), ring_vertex(i + 1, j),
                              ring_vertex(i + 1, j + 1)});
            m.indices.insert(m.indices.end(),
                             {ring_vertex(i, j), ring_vertex(i + 1, j + 1),
                              ring_vertex(i, j + 1)});
        }
    }
    // South cap: (pole, v(full,j+1), v(full,j)) — outward.
    for (int j = 0; j < segments; ++j)
        m.indices.insert(m.indices.end(),
                         {south, ring_vertex(full, j + 1),
                          ring_vertex(full, j)});
    return m;
}

// ---------------------------------------------------------------------------
// The deterministic corpus: lattices + named points, no randomness —
// the same points on every run, every compiler.
// ---------------------------------------------------------------------------
struct CorpusPoint {
    Vec3f query;
    std::string where; // named for the failure message
};

static std::vector<CorpusPoint> cube_corpus() {
    std::vector<CorpusPoint> pts;
    const float xs[5] = {-1.0f, 0.5f, 1.0f, 1.5f, 3.0f};
    for (float x : xs)
        for (float y : xs)
            for (float z : xs)
                pts.push_back({{x, y, z}, "cube lattice"});
    // The six face centres + the body centre (the on-surface and the
    // equidistant tie points — the honest edges of the classifier).
    pts.push_back({{1, 1, 2}, "cube top face centre"});
    pts.push_back({{1, 1, 0}, "cube bottom face centre"});
    pts.push_back({{2, 1, 1}, "cube right face centre"});
    pts.push_back({{0, 1, 1}, "cube left face centre"});
    pts.push_back({{1, 0, 1}, "cube front face centre"});
    pts.push_back({{1, 2, 1}, "cube back face centre"});
    pts.push_back({{1, 1, 1}, "cube body centre"});
    pts.push_back({{1, 0, 0}, "cube bottom-front edge"});
    pts.push_back({{0, 0, 0}, "cube corner"});
    return pts;
}

static std::vector<CorpusPoint> l_corpus() {
    std::vector<CorpusPoint> pts;
    const float xs[5] = {0.0f, 0.75f, 1.5f, 2.25f, 3.0f};
    for (float x : xs)
        for (float y : xs)
            for (float z : xs)
                pts.push_back({{x, y, z}, "L lattice"});
    pts.push_back({{2, 2, 1.5f}, "L notch centre"});
    pts.push_back({{1.5f, 1.5f, 1.5f}, "L notch corner"});
    pts.push_back({{0.5f, 0.5f, 1.5f}, "L horizontal arm"});
    pts.push_back({{0.5f, 2.5f, 1.5f}, "L vertical arm"});
    return pts;
}

static std::vector<CorpusPoint> tetra_corpus() {
    const float T = 1000000.0f;
    return {
        {{T + 0.25f, T + 0.25f, T + 1.625f}, "tetra just outside slanted"},
        {{T + 0.625f, T + 0.625f, T + 0.625f}, "tetra just inside slanted"},
        {{T + 0.25f, T + 0.25f, T + 0.25f}, "tetra deep inside"},
        {{T + 3, T + 3, T + 3}, "tetra far outside diagonal"},
    };
}

static std::vector<CorpusPoint> sphere_corpus() {
    std::vector<CorpusPoint> pts;
    const float xs[3] = {-3.0f, 0.7f, 3.0f};
    for (float x : xs)
        for (float y : xs)
            for (float z : xs)
                pts.push_back({{x, y, z}, "sphere lattice"});
    // Near-surface, off the exact surface (no d = 0 ties).
    pts.push_back({{2.1f, 0, 0}, "sphere just outside +x"});
    pts.push_back({{1.9f, 0, 0}, "sphere just inside +x"});
    pts.push_back({{-2.05f, 0.3f, -0.4f}, "sphere near -x"});
    pts.push_back({{0.4f, -1.95f, 0.7f}, "sphere near -y"});
    // The far field, where the proxy chain carries the answer: the
    // root itself is a closed shell — its dipole is zero, and at
    // (30,0,0) the ROOT is approximated (ε_root ≤ ε_w) with an
    // exactly-zero contribution.
    pts.push_back({{10, 10, 10}, "sphere far diagonal (proxy chain)"});
    pts.push_back({{30, 0, 0}, "sphere very far +x (root proxied)"});
    return pts;
}

// ---------------------------------------------------------------------------
// B4.1 — THE CONFORMITY TO THE BOUND: for every corpus point, the
// hierarchical winding against morso 1's exact sum, the difference
// bounded by the accumulated ε_node bound plus BOTH paths' derived
// arithmetic bands. The bound PROVEN, point by point.
// ---------------------------------------------------------------------------
TEST_CASE("H2-u4 m2 B4.1: the bound conformance - |w_bvh - w_exact| <= "
          "B_accum + 2*noise band, every corpus point") {
    const double threshold = winding_budget_for_tests();
    const double cap = winding_accumulation_cap_for_tests();
    REQUIRE(threshold == doctest::Approx(0.005).epsilon(1e-12));
    REQUIRE(cap == doctest::Approx(0.45).epsilon(1e-12));

    struct Case {
        const char* name;
        CAD_0::sdf::TriangleMesh mesh;
        std::vector<CorpusPoint> points;
    };
    std::vector<Case> cases;
    cases.push_back({"cube", closed_cube_mesh(), cube_corpus()});
    cases.push_back({"L", closed_l_mesh(), l_corpus()});
    cases.push_back({"tetra1e6", far_tetra_mesh(), tetra_corpus()});
    cases.push_back({"sphere10k", uv_sphere_mesh(2.0, 52, 100),
                     sphere_corpus()});

    for (const auto& c : cases) {
        const WindingBVHForTests bvh(c.mesh);
        const std::size_t F = bvh.triangle_count();
        const double noise = winding_sign_band_for_tests(F);
        // |w_bvh - w_exact| <= (B_accum + noise_bvh) + noise_exact.
        const double slack = noise + noise;
        for (const auto& p : c.points) {
            const Vec3d q{static_cast<double>(p.query.x),
                          static_cast<double>(p.query.y),
                          static_cast<double>(p.query.z)};
            const WindingQueryResult r = bvh.winding(q, threshold);
            const double w_exact = bvh.exact_winding(q);
            // The traversal's own accounting: the accumulated bound
            // never exceeds the composition's cap.
            CHECK_MESSAGE(r.bound <= cap,
                          c.name, " ", p.where,
                          ": B_accum=", r.bound, " > cap=", cap);
            // The bound, proven: the difference is carried by the
            // accumulated bound plus the two arithmetic bands.
            CHECK_MESSAGE(std::abs(r.w - w_exact) <= r.bound + slack,
                          c.name, " ", p.where,
                          ": |w_bvh-w_exact|=",
                          std::abs(r.w - w_exact),
                          " > B_accum+2*band=", r.bound + slack);
        }
    }
}

// ---------------------------------------------------------------------------
// B4.2 — THE CLASSIFICATION INVARIATA: zero sign flips against the
// exact evaluation on the whole corpus — and stronger: the answered
// VALUES are bit-identical (the distance path is exact in both, the
// sign is the same integer, so the bytes cannot differ), the
// gradients pinned where the nearest triangle is unique.
// ---------------------------------------------------------------------------
TEST_CASE("H2-u4 m2 B4.2: the classification invariance - zero flips, "
          "the answered bytes identical to the exact morso 1") {
    const double threshold = winding_budget_for_tests();

    struct Case {
        const char* name;
        CAD_0::sdf::TriangleMesh mesh;
        std::vector<CorpusPoint> points;
    };
    std::vector<Case> cases;
    cases.push_back({"cube", closed_cube_mesh(), cube_corpus()});
    cases.push_back({"L", closed_l_mesh(), l_corpus()});
    cases.push_back({"tetra1e6", far_tetra_mesh(), tetra_corpus()});
    cases.push_back({"sphere10k", uv_sphere_mesh(2.0, 52, 100),
                     sphere_corpus()});

    for (const auto& c : cases) {
        const WindingBVHForTests bvh(c.mesh);
        for (const auto& p : c.points) {
            const auto exact = bvh.exact_sample(p.query);
            const auto hier = bvh.sample(p.query, threshold);
            // Zero sign flips.
            CHECK_MESSAGE((exact.value < 0.0f) == (hier.value < 0.0f),
                          c.name, " ", p.where,
                          ": sign flipped (exact=", exact.value,
                          " bvh=", hier.value, ")");
            // The answered value, bit-identical: the sign is the same
            // integer (the composition's derivation), the distance is
            // the same exact minimum.
            CHECK_MESSAGE(exact.value == hier.value,
                          c.name, " ", p.where,
                          ": value moved (exact=", exact.value,
                          " bvh=", hier.value, ")");
            CHECK_MESSAGE(std::abs(exact.value) == std::abs(hier.value),
                          c.name, " ", p.where,
                          ": the exact distance moved");
        }
    }

    // The gradient, where the nearest triangle is unique by
    // construction (m1's own pinned points, now through the
    // hierarchy): 0.25 below the cube's bottom face — (0,0,-1).
    {
        const WindingBVHForTests bvh(closed_cube_mesh());
        const auto s = bvh.sample(Vec3f{1, 1, 0.25f}, threshold);
        CHECK(s.value == -0.25f);
        CHECK(s.gradient == Vec3f{0, 0, -1});
        const auto s_out = bvh.sample(Vec3f{1, 1, -0.25f}, threshold);
        CHECK(s_out.value == 0.25f);
        CHECK(s_out.gradient == Vec3f{0, 0, -1});
    }
    // The L's notch, 1.0 above the y=1 wall (the unique nearest
    // surface): the outward normal (0,1,0).
    {
        const WindingBVHForTests bvh(closed_l_mesh());
        const auto s = bvh.sample(Vec3f{2.5f, 2, 1.5f}, threshold);
        CHECK(s.value == 1.0f);
        CHECK(s.gradient == Vec3f{0, 1, 0});
    }
}

// ---------------------------------------------------------------------------
// B4.3 — THE STRESS REPLICATED: morso 1's far-from-origin (1e6) and
// non-convex cases, through the hierarchy, the same pinned values.
// ---------------------------------------------------------------------------
TEST_CASE("H2-u4 m2 B4.3: the replicated stress - the 1e6 tetrahedron "
          "and the non-convex L, through the BVH") {
    const double budget = winding_budget_for_tests();
    const float T = 1000000.0f;
    {
        const WindingBVHForTests bvh(far_tetra_mesh());
        const double kOut = 0.125 / std::sqrt(3.0);
        CHECK(bvh.sample(Vec3f{T + 0.25f, T + 0.25f, T + 1.625f}, budget)
                  .value ==
              doctest::Approx(static_cast<float>(kOut)).epsilon(1e-5f));
        CHECK(bvh.sample(Vec3f{T + 0.625f, T + 0.625f, T + 0.625f},
                         budget)
                  .value ==
              doctest::Approx(static_cast<float>(-kOut)).epsilon(1e-5f));
        CHECK(bvh.sample(Vec3f{T + 0.25f, T + 0.25f, T + 0.25f}, budget)
                  .value == -0.25f);
        const double kFar = 7.0 / std::sqrt(3.0);
        CHECK(bvh.sample(Vec3f{T + 3, T + 3, T + 3}, budget).value ==
              doctest::Approx(static_cast<float>(kFar)).epsilon(1e-5f));
    }
    {
        const WindingBVHForTests bvh(closed_l_mesh());
        CHECK(bvh.sample(Vec3f{2, 2, 1.5f}, budget).value == 1.0f);
        CHECK(bvh.sample(Vec3f{1.5f, 1.5f, 1.5f}, budget).value == 0.5f);
        CHECK(bvh.sample(Vec3f{0.5f, 0.5f, 1.5f}, budget).value == -0.5f);
        CHECK(bvh.sample(Vec3f{0.5f, 2.5f, 1.5f}, budget).value == -0.5f);
    }
}

// ---------------------------------------------------------------------------
// B4.4 — THE SPEED NUMBER: ONE big mesh (the 10k-triangle sphere),
// the measured confrontation — queries/sec, the hierarchy against the
// exact O(F). The claim is a NUMBER in the report, measured here,
// asserted only where the measurement is robust (the hierarchy must
// beat the exact scan on the same corpus; the margin is printed).
// ---------------------------------------------------------------------------
TEST_CASE("H2-u4 m2 B4.4: the speed number - queries/sec measured, "
          "BVH vs exact O(F) on the 10k-triangle sphere") {
    const auto mesh = uv_sphere_mesh(2.0, 52, 100);
    const WindingBVHForTests bvh(mesh);
    REQUIRE(bvh.triangle_count() == 10000);
    const double thr = winding_budget_for_tests();

    // Sanity before speed: the deep-inside query is negative (the
    // sphere is outward-oriented) and the far query is positive.
    CHECK(bvh.sample(Vec3f{0.1f, 0, 0}, thr).value < 0.0f);
    CHECK(bvh.sample(Vec3f{30, 0, 0}, thr).value > 0.0f);

    // The query classes, measured separately — the honest breakdown:
    // deep-inside (the hard case: the whole surface surrounds the
    // query, the accumulation room is the limit), near-surface and
    // far-field (where the proxy chain collapses the work).
    const std::vector<Vec3f> inside = {
        {0.7f, 0.7f, 0.7f}, {-0.7f, 0.7f, 0.7f}, {0.7f, -0.7f, 0.7f},
        {-0.7f, -0.7f, 0.7f}, {0.7f, 0.7f, -0.7f}, {-0.7f, 0.7f, -0.7f},
        {0.7f, -0.7f, -0.7f}, {-0.7f, -0.7f, -0.7f}};
    const std::vector<Vec3f> near_surf = {
        {2.1f, 0, 0}, {1.9f, 0, 0}, {-2.1f, 0, 0}, {0, 2.1f, 0},
        {0, 0, 2.1f}, {0, 0, -2.1f}, {1.5f, 1.5f, 0}, {-1.5f, 0, 1.5f}};
    const std::vector<Vec3f> far_field = {
        {3, 3, 3}, {-3, 3, 3}, {3, -3, 3}, {3, 3, -3}, {-3, -3, 3},
        {-3, 3, -3}, {3, -3, -3}, {-3, -3, -3}, {10, 10, 10}, {30, 0, 0},
        {0, -30, 0}, {-25, 5, 5}};
    REQUIRE(inside.size() + near_surf.size() + far_field.size() == 28);

    const auto measure = [&](const std::vector<Vec3f>& qs) {
        struct Timings {
            double sec_bvh, sec_exact;
            float acc_bvh, acc_exact;
        };
        const auto run_bvh = [&] {
            float acc = 0.0f;
            for (const auto& q : qs)
                acc += bvh.sample(q, thr).value;
            return acc;
        };
        const auto run_exact = [&] {
            float acc = 0.0f;
            for (const auto& q : qs) acc += bvh.exact_sample(q).value;
            return acc;
        };
        (void)run_bvh();  // warm-up
        (void)run_exact();
        const auto t0 = std::chrono::steady_clock::now();
        const float acc_bvh = run_bvh();
        const auto t1 = std::chrono::steady_clock::now();
        const float acc_exact = run_exact();
        const auto t2 = std::chrono::steady_clock::now();
        return Timings{std::chrono::duration<double>(t1 - t0).count(),
                       std::chrono::duration<double>(t2 - t1).count(),
                       acc_bvh, acc_exact};
    };

    const auto ti = measure(inside);
    const auto tn = measure(near_surf);
    const auto tf = measure(far_field);
    REQUIRE(ti.sec_bvh > 0.0);
    REQUIRE(ti.sec_exact > 0.0);
    REQUIRE(tn.sec_bvh > 0.0);
    REQUIRE(tn.sec_exact > 0.0);
    REQUIRE(tf.sec_bvh > 0.0);
    REQUIRE(tf.sec_exact > 0.0);

    // The classes agree with the exact evaluation (B4.2's claim).
    CHECK(ti.acc_bvh == ti.acc_exact);
    CHECK(tn.acc_bvh == tn.acc_exact);
    CHECK(tf.acc_bvh == tf.acc_exact);

    // The total: the hierarchy must beat the exact scan on the whole
    // corpus (inside + near + far — the honest mix).
    const double sec_bvh = ti.sec_bvh + tn.sec_bvh + tf.sec_bvh;
    const double sec_exact = ti.sec_exact + tn.sec_exact + tf.sec_exact;
    const std::size_t n_total =
        inside.size() + near_surf.size() + far_field.size();
    const double qps_bvh = static_cast<double>(n_total) / sec_bvh;
    const double qps_exact = static_cast<double>(n_total) / sec_exact;
    CHECK_MESSAGE(qps_bvh > qps_exact,
                  "the hierarchy must beat the exact scan: qps_bvh=",
                  qps_bvh, " qps_exact=", qps_exact);

    // The numbers for the report, printed every run (greppable) —
    // per class and total: the claim is a NUMBER, not an adjective.
    const auto print_class = [&](const char* name, std::size_t n,
                                 double sb, double se) {
        const double nd = static_cast<double>(n);
        std::cout << "[h2-u4-m2-speed] " << name << " queries=" << n
                  << " exact_qps=" << (nd / se) << " bvh_qps=" << (nd / sb)
                  << " speedup=" << (se / sb) << "x\n";
    };
    print_class("inside", inside.size(), ti.sec_bvh, ti.sec_exact);
    print_class("near-surface", near_surf.size(), tn.sec_bvh, tn.sec_exact);
    print_class("far-field", far_field.size(), tf.sec_bvh, tf.sec_exact);
    std::cout << "[h2-u4-m2-speed] TOTAL F=10000 queries=" << n_total
              << " exact_qps=" << qps_exact << " bvh_qps=" << qps_bvh
              << " speedup=" << (qps_bvh / qps_exact) << "x\n";
}

// ---------------------------------------------------------------------------
// B4.5 — THE DETERMINISM: N rebuilds of the same mesh bytes, the
// sampled grid's digest bit-identical — the true-median build with
// its fixed index tie-break is a pure function of the bytes
// (ADR-0014). The digest oracle compares build against build, per
// compiler; the cross-compiler column is this same test green on the
// Clang preset (the classification and the counts are the portable
// facts — ADR-0035 D5's own discipline).
// ---------------------------------------------------------------------------
static std::uint64_t fnv1a64(const void* data, std::size_t n,
                             std::uint64_t h) {
    const auto* bytes = static_cast<const unsigned char*>(data);
    for (std::size_t i = 0; i < n; ++i) {
        h ^= bytes[i];
        h *= 1099511628211ULL; // FNV-1a 64-bit prime
    }
    return h;
}

TEST_CASE("H2-u4 m2 B4.5: the determinism - N rebuilds, one digest") {
    const auto mesh = closed_cube_mesh();
    const auto lmesh = closed_l_mesh();

    const auto digest_of = [](const CAD_0::sdf::TriangleMesh& m) {
        auto body = mesh_to_sdf(m);
        REQUIRE(body != nullptr);
        REQUIRE(body->root() != nullptr);
        std::uint64_t h = 14695981039346656037ULL; // FNV offset basis
        const float xs[5] = {-1.0f, 0.5f, 1.0f, 1.5f, 3.0f};
        for (float x : xs)
            for (float y : xs)
                for (float z : xs) {
                    const auto s = body->root()->sample(Vec3f{x, y, z});
                    const float g[3] = {s.gradient.x, s.gradient.y,
                                        s.gradient.z};
                    h = fnv1a64(&s.value, sizeof(float), h);
                    h = fnv1a64(g, sizeof(g), h);
                }
        return h;
    };

    // Three builds of each mesh — the same bytes in, the same digest
    // out (the build and the query both deterministic).
    const std::uint64_t d1 = digest_of(mesh);
    const std::uint64_t d2 = digest_of(mesh);
    const std::uint64_t d3 = digest_of(mesh);
    CHECK(d1 == d2);
    CHECK(d2 == d3);
    const std::uint64_t l1 = digest_of(lmesh);
    const std::uint64_t l2 = digest_of(lmesh);
    CHECK(l1 == l2);
    CHECK(d1 != l1); // different meshes, different digests

    // The handle's own rebuild determinism, at the winding level:
    // two independent trees, the same winding bytes.
    const WindingBVHForTests a(closed_cube_mesh());
    const WindingBVHForTests b(closed_cube_mesh());
    for (const auto& p : cube_corpus()) {
        const Vec3d q{static_cast<double>(p.query.x),
                      static_cast<double>(p.query.y),
                      static_cast<double>(p.query.z)};
        const auto ra = a.winding(q, winding_budget_for_tests());
        const auto rb = b.winding(q, winding_budget_for_tests());
        REQUIRE(ra.w == rb.w); // bit-identical, not Approx
        REQUIRE(ra.bound == rb.bound);
    }
}

// ---------------------------------------------------------------------------
// B4.6 — THE OPEN MESH REFUSED STILL: the watertight audit BEFORE
// the build, morso 1's order intact — and the same contract at the
// seam's door (one contract, both entries).
// ---------------------------------------------------------------------------
TEST_CASE("H2-u4 m2 B4.6: the open mesh refused by name, BEFORE the "
          "build - both doors") {
    auto m = closed_cube_mesh();
    m.indices.resize(m.indices.size() - 3); // triangle 11 out: 3 free edges
    REQUIRE(m.indices.size() == 33);

    // Door 1: the public mesh_to_sdf.
    bool threw = false;
    std::string what;
    try {
        (void)mesh_to_sdf(m);
    } catch (const std::invalid_argument& e) {
        threw = true;
        what = e.what();
    } catch (...) {
        FAIL("the refusal's KIND is wrong: not a std::invalid_argument");
    }
    REQUIRE(threw);
    CHECK(what.find("NOT WATERTIGHT") != std::string::npos);
    CHECK(what.find("3 free edge(s)") != std::string::npos);
    CHECK(what.find("Remedy") != std::string::npos);

    // Door 2: the seam's builder — the audit runs before any build
    // there too (the same throw, the same payload).
    threw = false;
    try {
        const WindingBVHForTests bvh(m);
        (void)bvh;
    } catch (const std::invalid_argument& e) {
        threw = true;
        CHECK(std::string(e.what()).find("NOT WATERTIGHT") !=
              std::string::npos);
    } catch (...) {
        FAIL("the seam's refusal KIND is wrong");
    }
    CHECK(threw);
}

// ---------------------------------------------------------------------------
// B4.7 — THE COUNTERPROOF OF THE BOUND: a traversal with ε_w = 0
// (never approximate — every node descends, every leaf evaluates)
// must give the EXACT answer, machine-built: zero accumulated bound,
// the winding within the derived reordering noise of the exact sum,
// the classification identical, the answered value bit-identical.
// The escape route from its own optimization exists and is proven.
// ---------------------------------------------------------------------------
TEST_CASE("H2-u4 m2 B4.7: the counterproof - budget zero gives the "
          "exact answer, the escape route proven") {
    struct Case {
        const char* name;
        CAD_0::sdf::TriangleMesh mesh;
        std::vector<CorpusPoint> points;
    };
    std::vector<Case> cases;
    cases.push_back({"cube", closed_cube_mesh(), cube_corpus()});
    cases.push_back({"L", closed_l_mesh(), l_corpus()});
    cases.push_back({"tetra1e6", far_tetra_mesh(), tetra_corpus()});
    cases.push_back({"sphere10k", uv_sphere_mesh(2.0, 52, 100),
                     sphere_corpus()});

    for (const auto& c : cases) {
        const WindingBVHForTests bvh(c.mesh);
        const std::size_t F = bvh.triangle_count();
        const double noise = winding_sign_band_for_tests(F);
        for (const auto& p : c.points) {
            const Vec3d q{static_cast<double>(p.query.x),
                          static_cast<double>(p.query.y),
                          static_cast<double>(p.query.z)};
            const WindingQueryResult r0 = bvh.winding(q, 0.0);
            const double w_exact = bvh.exact_winding(q);

            // Nothing was approximated — the accounting says zero.
            CHECK_MESSAGE(r0.bound == 0.0, c.name, " ", p.where,
                          ": budget zero but B_accum=", r0.bound);

            // The winding: the same terms, tree order instead of
            // index order — within the derived reordering noise
            // (both Kahan sums sit within ε_w,noise of the truth).
            CHECK_MESSAGE(std::abs(r0.w - w_exact) <= 2.0 * noise,
                          c.name, " ", p.where,
                          ": |w(0)-w_exact|=", std::abs(r0.w - w_exact),
                          " > 2*band=", 2.0 * noise);

            // The answered sample: bit-identical to the exact one
            // (the sign rounds to the same integer, the distance is
            // the same exact minimum).
            const auto s0 = bvh.sample(p.query, 0.0);
            const auto se = bvh.exact_sample(p.query);
            CHECK_MESSAGE(s0.value == se.value, c.name, " ", p.where,
                          ": value (", s0.value, " vs ", se.value, ")");
            CHECK_MESSAGE((s0.value < 0.0f) == (se.value < 0.0f),
                          c.name, " ", p.where, ": sign flipped");
        }
    }

    // The gradient through the zero-budget traversal, at the unique
    // nearest points (the tie-free evidence, bit-pinned).
    {
        const WindingBVHForTests bvh(closed_cube_mesh());
        const auto s = bvh.sample(Vec3f{1, 1, 0.25f}, 0.0);
        CHECK(s.value == -0.25f);
        CHECK(s.gradient == Vec3f{0, 0, -1});
    }
}
