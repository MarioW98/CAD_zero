// tests/unit/sdf/test_perf3_evaluator.cpp
//
// PERF 3 — THE EVALUATOR'S PERMANENT SEAT: the bit-identity of the
// batch kernels, the dispatch witness, and the thread-count
// determinism of the parallel evaluate. A ctest member of every
// column; the TBB half runs where TBB is compiled in (the opt-in
// column) and SKIPS DECLARED elsewhere — never a silence.
//
// THE CONTRACT, three pins:
//
//   1. BIT-IDENTITY (ADR-0014, extended to the batch road): for every
//      node class with a batch road — the eight leaf kernels and the
//      composing interior — sample_batch() and the per-point sample()
//      produce the SAME BITS for every value and every gradient
//      component of every corpus point. Not approximately: memcmp.
//      A kernel that reassociates one dot product or swaps one
//      division for a reciprocal moves a bit and goes red here,
//      before any digest downstream could mask it with a tolerance.
//
//   2. THE WITNESS (the audit's instrument, made permanent — the
//      brief's own law that declared/executed discrepancies must be
//      catchable by the suite, not only by manual audit): the counters
//      of detail/batch_stats.hpp are pinned to the EXACT arithmetic
//      per corpus. A leaf kernel that stops running (reverted, broken
//      dispatch) moves its corpus's vector delta to zero and the pin
//      goes red NAMING the class. The declared scalar seats (twist,
//      extrude) are pinned the other way: ZERO vector points — a
//      rogue "vectorized" twist that changes one cos by one ULP is
//      caught by the bit-identity pin AND by this one.
//
//   3. THREAD DETERMINISM (the TBB half): the same evaluation under
//      thread counts {1, 2, 4, hardware} produces the SAME FNV-1a 64
//      digest, equal to the serial reference digest computed by the
//      per-point sample() loop in this very test. The partitioning is
//      arithmetic blocks with no reductions (evaluate.cpp's own
//      declaration); this pin is the proof the declaration cannot
//      outlive the code.
//
// THE CORPORA, deterministic by construction: fixed integer seeds
// through the suite's own LCG (tests/property/property_prng.hpp's
// model — no time source, no unseeded random), grids that cross the
// surfaces, points ON the axes and the mirror planes (the branches'
// own edges), and odd lengths (n % width != 0) so every kernel's
// scalar tail is exercised beside its lanes.
#include <doctest/doctest.h>

#include "CAD_0/sdf/primitives.hpp"
#include "CAD_0/sdf/operators.hpp"
#include "CAD_0/sdf/transforms.hpp"
#include "CAD_0/sdf/sketch_entities.hpp"
#include "CAD_0/sdf/sketch_profile.hpp"
#include "CAD_0/sdf/sketch_extrude.hpp"
#include "CAD_0/sdf/evaluate.hpp"
#include "CAD_0/sdf/detail/batch_stats.hpp"

#include <CAD_0/config.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <span>
#include <string>
#include <vector>

using namespace CAD_0::sdf;
using namespace CAD_0::math;

namespace {

// The suite's deterministic LCG (the property tests' own model: fixed
// seed, integer state, no platform dependence).
struct Lcg {
    std::uint64_t s;
    explicit Lcg(std::uint64_t seed) : s(seed * 6364136223846793005ULL + 1442695040888963407ULL) {}
    std::uint32_t next() {
        s = s * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<std::uint32_t>(s >> 33);
    }
    float uniform(float lo, float hi) {
        return lo + (hi - lo) * (static_cast<float>(next() >> 8) / 16777216.0f);
    }
};

// A corpus that crosses surfaces, hits the axes, and lands on the
// branch edges: a jittered grid plus the exact axis/center points.
std::vector<Vec3f> make_corpus(std::size_t n, float lo, float hi,
                               std::uint64_t seed) {
    std::vector<Vec3f> pts;
    pts.reserve(n + 8);
    Lcg rng(seed);
    for (std::size_t i = 0; i < n; ++i) {
        pts.push_back({rng.uniform(lo, hi), rng.uniform(lo, hi),
                       rng.uniform(lo, hi)});
    }
    // The branch edges: on-axis, origin, mirror plane, unit axes.
    pts.push_back({0.0f, 0.0f, 0.0f});
    pts.push_back({0.0f, 0.5f, 0.0f});
    pts.push_back({0.5f, 0.0f, 0.0f});
    pts.push_back({0.0f, 0.0f, 0.5f});
    return pts;
}

// Bit-exact comparison of the two roads over a corpus.
bool batch_equals_scalar(const SDFNode& node,
                         const std::vector<Vec3f>& pts,
                         std::string* first_diff = nullptr) {
    const std::size_t n = pts.size();
    std::vector<float> bv(n);
    std::vector<Vec3f> bg(n);
    node.sample_batch(std::span<const Vec3f>(pts.data(), n), bv.data(),
                      bg.data());
    for (std::size_t i = 0; i < n; ++i) {
        const SDFSample s = node.sample(pts[i]);
        if (std::memcmp(&bv[i], &s.value, sizeof(float)) != 0) {
            if (first_diff)
                *first_diff = "value at corpus point " + std::to_string(i);
            return false;
        }
        for (int c = 0; c < 3; ++c) {
            if (std::memcmp(&bg[i][static_cast<std::size_t>(c)],
                            &s.gradient[static_cast<std::size_t>(c)],
                            sizeof(float)) != 0) {
                if (first_diff)
                    *first_diff = "gradient[" + std::to_string(c) +
                                  "] at corpus point " + std::to_string(i);
                return false;
            }
        }
    }
    return true;
}

using Witness = CAD_0::sdf::detail::BatchWitness::Snapshot;

Witness witness_now() { return detail::batch_witness().read(); }

struct WitnessDelta {
    std::uint64_t vector_points;
    std::uint64_t scalar_points;
};

WitnessDelta witness_delta(const Witness& before, const Witness& after) {
    return {after.vector_points - before.vector_points,
            after.scalar_points - before.scalar_points};
}

// FNV-1a 64 over the canonical rendering — the house instrument
// (count in the header, every float as its %08x bit pattern).
[[maybe_unused]] std::uint64_t fnv_feed(std::uint64_t h, const char* data, std::size_t len) {
    for (std::size_t i = 0; i < len; ++i) {
        h ^= static_cast<unsigned char>(data[i]);
        h *= 0x100000001b3ull;
    }
    return h;
}

[[maybe_unused]] std::uint64_t result_digest(const EvalResult& r) {
    std::uint64_t h = 0xcbf29ce484222325ull;
    char buf[16];
    std::snprintf(buf, sizeof buf, "%08zx", r.values.size());
    h = fnv_feed(h, buf, 8);
    for (float v : r.values) {
        std::uint32_t b;
        std::memcpy(&b, &v, 4);
        std::snprintf(buf, sizeof buf, "%08x", b);
        h = fnv_feed(h, buf, 8);
    }
    for (const Vec3f& g : r.gradients) {
        for (int c = 0; c < 3; ++c) {
            std::uint32_t b;
            std::memcpy(&b, &g[static_cast<std::size_t>(c)], 4);
            std::snprintf(buf, sizeof buf, "%08x", b);
            h = fnv_feed(h, buf, 8);
        }
    }
    return h;
}

} // namespace

// ---------------------------------------------------------------------------
// PIN 1 — the leaves' kernels are bit-identical to their sample()
// ---------------------------------------------------------------------------

TEST_CASE("perf3 bit-identity: the eight leaf kernels") {
    const std::size_t n = 1001; // odd: every kernel's scalar tail rides too
    const std::vector<Vec3f> corpus = make_corpus(n, -1.8f, 1.8f, 0xA5A5A5A5ull);

    struct Case {
        const char* name;
        std::unique_ptr<SDFNode> node;
    };
    std::vector<Case> cases;
    cases.push_back({"sphere r=1", std::make_unique<SphereSDF>(1.0f)});
    cases.push_back({"sphere r=0.37", std::make_unique<SphereSDF>(0.37f)});
    cases.push_back({"box", std::make_unique<BoxSDF>(Vec3f{0.75f, 0.5f, 1.1f})});
    cases.push_back({"box cube", std::make_unique<BoxSDF>(Vec3f{0.6f, 0.6f, 0.6f})});
    cases.push_back({"cylinder", std::make_unique<CylinderSDF>(0.6f, 1.7f)});
    cases.push_back({"cylinder squat", std::make_unique<CylinderSDF>(0.9f, 0.3f)});
    cases.push_back({"torus", std::make_unique<TorusSDF>(1.0f, 0.3f)});
    cases.push_back({"torus spindle", std::make_unique<TorusSDF>(0.4f, 0.7f)});
    cases.push_back({"cone", std::make_unique<ConeSDF>(0.8f, 1.5f)});
    cases.push_back({"cone squat", std::make_unique<ConeSDF>(1.2f, 0.4f)});
    cases.push_back({"capsule", std::make_unique<CapsuleSDF>(
        Vec3f{-1.0f, -0.3f, 0.2f}, Vec3f{1.0f, 0.4f, -0.5f}, 0.45f)});
    cases.push_back({"capsule degenerate a=b", std::make_unique<CapsuleSDF>(
        Vec3f{0.2f, 0.2f, 0.2f}, Vec3f{0.2f, 0.2f, 0.2f}, 0.5f)});
    cases.push_back({"plane", std::make_unique<PlaneSDF>()});
    cases.push_back({"slab-z +Z", std::make_unique<SlabZ>(1.4f, 1)});
    cases.push_back({"slab-z -Z", std::make_unique<SlabZ>(0.8f, -1)});

    for (const Case& c : cases) {
        std::string diff;
        if (!batch_equals_scalar(*c.node, corpus, &diff)) {
            FAIL("kernel not bit-identical: ", c.name, " — first diff: ", diff);
        }
    }
}

// ---------------------------------------------------------------------------
// PIN 1 (interior) — the composing interior is bit-identical by
// construction; these corpora pin the construction
// ---------------------------------------------------------------------------

TEST_CASE("perf3 bit-identity: the composing interior") {
    const std::size_t n = 1001;
    const std::vector<Vec3f> corpus = make_corpus(n, -2.2f, 2.2f, 0xC0FFEEull);

    // Every interior class, over primitive children that exercise the
    // children's own tails and lanes.
    {
        auto u = sdf_union(make_sphere(1.0f),
                           sdf_translate(make_sphere(0.8f), {1.6f, 0.2f, -0.4f}));
        std::string d;
        if (!batch_equals_scalar(*u.root(), corpus, &d))
            FAIL("union 2-ary not bit-identical: ", d);
    }
    {
        // N-ary (the deserialization form) with three children.
        std::vector<std::unique_ptr<SDFNode>> kids;
        kids.push_back(std::make_unique<SphereSDF>(0.9f));
        kids.push_back(std::make_unique<TranslateSDF>(
            std::make_unique<BoxSDF>(Vec3f{0.4f, 0.4f, 0.4f}),
            Vec3f{1.2f, 0.0f, 0.3f}));
        kids.push_back(std::make_unique<CylinderSDF>(0.5f, 1.2f));
        SDFBody u{std::make_unique<UnionSDF>(std::move(kids))};
        std::string d;
        if (!batch_equals_scalar(*u.root(), corpus, &d))
            FAIL("union N-ary not bit-identical: ", d);
    }
    {
        auto x = sdf_intersect(make_sphere(1.1f),
                               sdf_translate(make_box(Vec3f{0.5f, 0.5f, 0.5f}),
                                             {0.3f, 0.0f, 0.0f}));
        std::string d;
        if (!batch_equals_scalar(*x.root(), corpus, &d))
            FAIL("intersection not bit-identical: ", d);
    }
    {
        auto s = sdf_subtract(make_box(Vec3f{0.8f, 0.8f, 0.8f}),
                              sdf_translate(make_sphere(0.5f), {0.4f, 0.4f, 0.0f}));
        std::string d;
        if (!batch_equals_scalar(*s.root(), corpus, &d))
            FAIL("subtraction not bit-identical: ", d);
    }
    {
        auto sm = sdf_smooth_union(make_sphere(0.9f),
                                   sdf_translate(make_sphere(0.7f), {1.4f, 0.0f, 0.0f}),
                                   0.4f);
        std::string d;
        if (!batch_equals_scalar(*sm.root(), corpus, &d))
            FAIL("smooth union k>0 not bit-identical: ", d);
    }
    {
        auto sm0 = sdf_smooth_union(make_sphere(0.9f),
                                    sdf_translate(make_sphere(0.7f), {1.4f, 0.0f, 0.0f}),
                                    0.0f);
        std::string d;
        if (!batch_equals_scalar(*sm0.root(), corpus, &d))
            FAIL("smooth union k=0 (hard min) not bit-identical: ", d);
    }
    {
        const auto q = Quatf::from_axis_angle(Vec3f{0.0f, 1.0f, 0.0f}, 0.7f);
        auto r = sdf_rotate(make_torus(1.0f, 0.3f), q);
        std::string d;
        if (!batch_equals_scalar(*r.root(), corpus, &d))
            FAIL("rotate not bit-identical: ", d);
    }
    {
        auto s2 = sdf_scale(make_cylinder(0.5f, 1.0f), 2.5f);
        std::string d;
        if (!batch_equals_scalar(*s2.root(), corpus, &d))
            FAIL("scale up not bit-identical: ", d);
        auto s3 = sdf_scale(make_capsule(Vec3f{-1, 0, 0}, Vec3f{1, 0, 0}, 0.3f), 0.4f);
        std::string d3;
        if (!batch_equals_scalar(*s3.root(), corpus, &d3))
            FAIL("scale down not bit-identical: ", d3);
    }
    {
        // A DEEP tree — the scratch pool's own depth: four interior
        // levels over mixed leaves.
        auto deep = sdf_union(
            sdf_translate(make_sphere(0.7f), {-1.2f, 0.1f, 0.0f}),
            sdf_smooth_union(
                sdf_rotate(make_box(Vec3f{0.45f, 0.45f, 0.45f}),
                           Quatf::from_axis_angle(Vec3f{0.0f, 0.0f, 1.0f}, 0.5f)),
                sdf_scale(sdf_translate(make_cone(0.5f, 0.9f), {1.4f, -0.2f, 0.0f}),
                          1.3f),
                0.25f));
        std::string d;
        if (!batch_equals_scalar(*deep.root(), corpus, &d))
            FAIL("deep composite not bit-identical: ", d);
    }
}

// ---------------------------------------------------------------------------
// PIN 1 (declared seats) — twist and extrude ride the scalar default
// ---------------------------------------------------------------------------

TEST_CASE("perf3 bit-identity: the declared scalar seats") {
    const std::size_t n = 1001;
    const std::vector<Vec3f> corpus = make_corpus(n, -1.6f, 1.6f, 0x5EED5EEDull);

    {
        auto tw = sdf_twist(make_box(Vec3f{0.7f, 0.9f, 0.7f}), 0.3f);
        std::string d;
        if (!batch_equals_scalar(*tw.root(), corpus, &d))
            FAIL("twist not bit-identical: ", d);
    }
    {
        // The extrusion: the sketcher's own vocabulary (a closed
        // single-circle chain — the extrude test's own idiom), both
        // directions.
        ExtrudeSDF ex(std::vector<SketchChain>{
                          SketchChain{{sketch_circle({0.0f, 0.0f}, 0.8f,
                                                    /*hole=*/false)}}},
                      1.2f, 1);
        std::string d;
        if (!batch_equals_scalar(ex, corpus, &d))
            FAIL("extrude not bit-identical: ", d);
        ExtrudeSDF exm(std::vector<SketchChain>{
                           SketchChain{{sketch_circle({0.2f, -0.1f}, 0.6f,
                                                     /*hole=*/false)}}},
                       0.9f, -1);
        std::string dm;
        if (!batch_equals_scalar(exm, corpus, &dm))
            FAIL("extrude -Z not bit-identical: ", dm);
    }
}

// ---------------------------------------------------------------------------
// PIN 2 — the witness: exact path arithmetic per corpus
// ---------------------------------------------------------------------------

TEST_CASE("perf3 witness: the exact path split, pinned") {
    const std::size_t n = 1001; // 1001 % 4 == 1 on every SIMD width we ship
    const std::size_t tail = n % 4; // exercised tails
    const std::vector<Vec3f> corpus = make_corpus(n, -1.5f, 1.5f, 0xBADFACEull);
    std::vector<float> v(n);
    std::vector<Vec3f> g(n);
    const auto span = std::span<const Vec3f>(corpus.data(), n);

    // A leaf with a kernel: vector n-tail, scalar tail.
    {
        const auto before = witness_now();
        SphereSDF(1.0f).sample_batch(span, v.data(), g.data());
        const auto d = witness_delta(before, witness_now());
        CHECK(d.vector_points == n - tail);
        CHECK(d.scalar_points == tail);
    }
    {
        const auto before = witness_now();
        SlabZ(1.4f, 1).sample_batch(span, v.data(), g.data());
        const auto d = witness_delta(before, witness_now());
        CHECK(d.vector_points == n - tail);
        CHECK(d.scalar_points == tail);
    }

    // A tree of two translated sphere kernels: each child serves its
    // own n points at the leaves — vector 2*(n-tail), scalar 2*tail.
    // The interior composes and counts nothing (the declared
    // semantics: the witness counts leaf field arithmetic by path).
    {
        auto u = sdf_union(make_sphere(1.0f),
                           sdf_translate(make_sphere(0.8f), {1.6f, 0.0f, 0.0f}));
        const auto before = witness_now();
        u.root()->sample_batch(span, v.data(), g.data());
        const auto d = witness_delta(before, witness_now());
        CHECK(d.vector_points == 2 * (n - tail));
        CHECK(d.scalar_points == 2 * tail);
    }

    // N-ary union of three leaves.
    {
        std::vector<std::unique_ptr<SDFNode>> kids;
        kids.push_back(std::make_unique<SphereSDF>(0.9f));
        kids.push_back(std::make_unique<SphereSDF>(0.5f));
        kids.push_back(std::make_unique<PlaneSDF>());
        SDFBody u{std::make_unique<UnionSDF>(std::move(kids))};
        const auto before = witness_now();
        u.root()->sample_batch(span, v.data(), g.data());
        const auto d = witness_delta(before, witness_now());
        CHECK(d.vector_points == 3 * (n - tail));
        CHECK(d.scalar_points == 3 * tail);
    }

    // The declared scalar seats: ZERO vector points, all scalar —
    // pinned both ways (the "(or viceversa)" of the brief's law: a
    // rogue vectorized twist is red here the moment it runs).
    {
        auto tw = sdf_twist(make_box(Vec3f{0.7f, 0.7f, 0.7f}), 0.4f);
        const auto before = witness_now();
        tw.root()->sample_batch(span, v.data(), g.data());
        const auto d = witness_delta(before, witness_now());
        CHECK(d.vector_points == 0);
        CHECK(d.scalar_points == n);
    }
    {
        ExtrudeSDF ex(std::vector<SketchChain>{
                          SketchChain{{sketch_circle({0.0f, 0.0f}, 0.7f,
                                                    /*hole=*/false)}}},
                      1.0f, 1);
        const auto before = witness_now();
        ex.sample_batch(span, v.data(), g.data());
        const auto d = witness_delta(before, witness_now());
        CHECK(d.vector_points == 0);
        CHECK(d.scalar_points == n);
    }
}

// ---------------------------------------------------------------------------
// The backend rides the batch road (the integration half: the SoA
// scatter, the guards, the tiny inputs)
// ---------------------------------------------------------------------------

TEST_CASE("perf3 backend: evaluate equals the per-point road") {
    const std::vector<Vec3f> corpus = make_corpus(777, -1.7f, 1.7f, 0xD00Dull);
    auto body = sdf_union(
        sdf_translate(make_sphere(0.8f), {-0.7f, 0.0f, 0.0f}),
        sdf_smooth_union(make_box(Vec3f{0.5f, 0.5f, 0.5f}),
                         sdf_rotate(make_cylinder(0.4f, 1.1f),
                                    Quatf::from_axis_angle(Vec3f{1, 0, 0}, 0.6f)),
                         0.3f));

    EvalResult out;
    CpuEvalBackend backend;
    backend.evaluate(body, std::span<const Vec3f>(corpus.data(), corpus.size()), out);
    REQUIRE(out.values.size() == corpus.size());
    REQUIRE(out.gradients.size() == corpus.size());
    for (std::size_t i = 0; i < corpus.size(); ++i) {
        const SDFSample s = body.root()->sample(corpus[i]);
        if (std::memcmp(&out.values[i], &s.value, sizeof(float)) != 0)
            FAIL("backend value differs at ", i);
        for (int c = 0; c < 3; ++c)
            if (std::memcmp(&out.gradients[i][static_cast<std::size_t>(c)],
                            &s.gradient[static_cast<std::size_t>(c)],
                            sizeof(float)) != 0)
                FAIL("backend gradient differs at ", i);
    }

    // The guards: empty body, empty span, single point.
    SDFBody empty;
    EvalResult e;
    backend.evaluate(empty, std::span<const Vec3f>(), e);
    CHECK(e.values.empty());
    std::vector<Vec3f> one{{0.3f, -0.2f, 0.5f}};
    backend.evaluate(empty, std::span<const Vec3f>(one.data(), 1), e);
    REQUIRE(e.values.size() == 1);
    CHECK(e.values[0] == 0.0f);
    backend.evaluate(body, std::span<const Vec3f>(one.data(), 1), e);
    REQUIRE(e.values.size() == 1);
    const SDFSample s = body.root()->sample(one[0]);
    CHECK(std::memcmp(&e.values[0], &s.value, sizeof(float)) == 0);
}

// ---------------------------------------------------------------------------
// PIN 3 — thread-count determinism (the TBB column's own seat)
// ---------------------------------------------------------------------------

#if defined(CAD_0_USE_TBB) && CAD_0_USE_TBB

TEST_CASE("perf3 determinism: the digest identical at every thread count") {
    // The corpus: big enough to partition (the backend's own gate is
    // 1024 points), a leaf body and a composite.
    const std::size_t n = 8192;
    const std::vector<Vec3f> corpus = make_corpus(n, -1.9f, 1.9f, 0xFEEDFACEull);
    const auto span = std::span<const Vec3f>(corpus.data(), n);

    auto leaf = make_sphere(1.0f);
    auto tree = sdf_union(
        sdf_translate(make_sphere(0.8f), {-0.9f, 0.1f, 0.0f}),
        sdf_smooth_union(
            sdf_rotate(make_box(Vec3f{0.45f, 0.45f, 0.45f}),
                       Quatf::from_axis_angle(Vec3f{0, 1, 0}, 0.7f)),
            sdf_scale(make_torus(0.8f, 0.25f), 1.1f), 0.3f));

    // The serial reference — the per-point sample() road, the authority
    // the parallel runs must equal bit for bit. Same digest helper, so
    // the rendering cannot differ between the two roads being compared
    // (the digest before the times: a comparison of two renderings
    // measures nothing).
    for (const SDFBody* body : {&leaf, &tree}) {
        EvalResult ref_out;
        ref_out.values.resize(n);
        ref_out.gradients.resize(n);
        for (std::size_t i = 0; i < n; ++i) {
            const SDFSample s = body->root()->sample(corpus[i]);
            ref_out.values[i] = s.value;
            ref_out.gradients[i] = s.gradient;
        }
        const std::uint64_t ref = result_digest(ref_out);

        std::uint64_t prev = 0;
        bool first = true;
        for (unsigned threads : {1u, 2u, 4u, 8u}) {
            CpuEvalBackend backend(threads);
            EvalResult out;
            backend.evaluate(*body, span, out);
            const std::uint64_t d = result_digest(out);
            if (!first)
                REQUIRE_MESSAGE(d == prev,
                                "thread count changed the digest: the "
                                "partitioning is not deterministic");
            REQUIRE_MESSAGE(d == ref,
                            "the parallel digest differs from the serial "
                            "reference: bit-identity broken across the "
                            "thread boundary");
            prev = d;
            first = false;
        }
    }
}

#else // !CAD_0_USE_TBB

TEST_CASE("perf3 determinism: declared skip off the TBB column") {
    std::printf("  [perf3] the thread-count determinism digest runs on the "
                "TBB column (-DCAD_0_USE_TBB=ON): this column ships the "
                "serial batch road, the digest's serial reference and its "
                "bit-identity pins above judge it — the parallel half "
                "SKIPS here, DECLARED, never silent\n");
}

#endif // CAD_0_USE_TBB
