// tests/unit/sdf/test_evaluate.cpp
#include <doctest/doctest.h>

#include "CAD_0/sdf/primitives.hpp"
#include "CAD_0/sdf/operators.hpp"
#include "CAD_0/sdf/transforms.hpp"
#include "CAD_0/sdf/evaluate.hpp"

#include <string>
#include <vector>

using namespace CAD_0::sdf;
using namespace CAD_0::math;

TEST_CASE("CpuEvalBackend: empty body returns zero") {
    SDFBody body;
    std::vector<Vec3f> pts = {{0, 0, 0}, {1, 1, 1}};
    EvalResult out;
    CpuEvalBackend backend;
    backend.evaluate(body, std::span<const Vec3f>(pts.data(), pts.size()), out);
    CHECK(out.values.size() == 2);
    CHECK(out.values[0] == 0.0f);
    CHECK(out.values[1] == 0.0f);
}

TEST_CASE("CpuEvalBackend: single sphere") {
    auto body = make_sphere(1.0f);
    std::vector<Vec3f> pts = {
        {0, 0, 0},     // inside
        {2, 0, 0},     // outside
        {0, 1, 0},     // surface
    };
    EvalResult out;
    CpuEvalBackend backend;
    backend.evaluate(body, std::span<const Vec3f>(pts.data(), pts.size()), out);
    REQUIRE(out.values.size() == 3);
    CHECK(out.values[0] == doctest::Approx(-1.0f).epsilon(1e-5f));
    CHECK(out.values[1] == doctest::Approx(1.0f).epsilon(1e-5f));
    CHECK(out.values[2] == doctest::Approx(0.0f).epsilon(1e-5f));
}

TEST_CASE("CpuEvalBackend: gradients are unit-length on surface") {
    auto body = make_sphere(1.0f);
    std::vector<Vec3f> pts = {{1, 0, 0}}; // on the surface
    EvalResult out;
    CpuEvalBackend backend;
    backend.evaluate(body, std::span<const Vec3f>(pts.data(), pts.size()), out);
    REQUIRE(out.gradients.size() == 1);
    CHECK(out.gradients[0].length() == doctest::Approx(1.0f).epsilon(1e-4f));
}

TEST_CASE("CpuEvalBackend: large batch parallel evaluation") {
    auto body = make_sphere(2.0f);
    // Generate a 16x16x16 grid (4096 points) inside [-3, 3]^3
    constexpr int N = 16;
    std::vector<Vec3f> pts;
    pts.reserve(N * N * N);
    for (int xi = 0; xi < N; ++xi) {
        for (int yi = 0; yi < N; ++yi) {
            for (int zi = 0; zi < N; ++zi) {
                const float fx = -3.0f + 6.0f * (static_cast<float>(xi) / (N - 1));
                const float fy = -3.0f + 6.0f * (static_cast<float>(yi) / (N - 1));
                const float fz = -3.0f + 6.0f * (static_cast<float>(zi) / (N - 1));
                pts.push_back({fx, fy, fz});
            }
        }
    }
    EvalResult out;
    CpuEvalBackend backend;
    backend.evaluate(body, std::span<const Vec3f>(pts.data(), pts.size()), out);
    REQUIRE(out.values.size() == pts.size());

    // The grid maps [-3, 3] to 16 points; the closest grid point to (0,0,0)
    // is at xi = yi = zi = 7.5 (rounded to 7 or 8). Either gives |coord| ≈
    // 0.2. We just check the value is within the expected range for a
    // point near the center of a sphere of radius 2:
    //   value ≈ |p| - 2 ≈ 0.2 - 2 = -1.8 (or close)
    const std::size_t center_idx = 7 * N * N + 7 * N + 7;
    const float v = out.values[center_idx];
    // Should be in [-1.85, -1.75] given our grid resolution.
    CHECK(v < -1.5f);
    CHECK(v > -2.0f);
}

TEST_CASE("CpuEvalBackend: composite body (union + transform)") {
    // Two unit spheres at (0,0,0) and (2,0,0).
    auto a = make_sphere(1.0f);
    auto b = sdf_translate(make_sphere(1.0f), {2, 0, 0});
    auto u = sdf_union(std::move(a), std::move(b));

    std::vector<Vec3f> pts = {{0, 0, 0}, {2, 0, 0}, {1, 0, 0}};
    EvalResult out;
    CpuEvalBackend backend;
    backend.evaluate(u, std::span<const Vec3f>(pts.data(), pts.size()), out);
    REQUIRE(out.values.size() == 3);
    // Sphere A center → value = -1 (inside, surface of B is at 1 unit distance)
    CHECK(out.values[0] == doctest::Approx(-1.0f).epsilon(1e-5f));
    // Sphere B center → value = -1
    CHECK(out.values[1] == doctest::Approx(-1.0f).epsilon(1e-5f));
    // Midpoint (1,0,0) → distance to each sphere surface = 0 (it lies on
    // the surface of both spheres, so union value = 0).
    CHECK(out.values[2] == doctest::Approx(0.0f).epsilon(1e-5f));
}

TEST_CASE("GpuEvalBackend: stub falls back to CPU") {
    auto body = make_sphere(1.0f);
    std::vector<Vec3f> pts = {{0, 0, 0}};
    EvalResult out;
    GpuEvalBackend gpu;
    gpu.evaluate(body, std::span<const Vec3f>(pts.data(), pts.size()), out);
    CHECK(out.values[0] == doctest::Approx(-1.0f).epsilon(1e-5f));
}

// ---------------------------------------------------------------------------
// Task 3 — GpuEvalBackend link-safety + name() regression tests
//
// Previously, GpuEvalBackend::name() was defined inline in the header.
// This could cause "multiple definition" link errors in TBB/GPU builds
// where the header is included from multiple TUs. The fix moves the
// definition out-of-line into evaluate.cpp and makes the name string
// reflect the actual backend status ("gpu-stub (cpu-fallback)") so
// callers can detect the fallback at runtime — no silent GPU→CPU
// degradation (see ADR-0008).
// ---------------------------------------------------------------------------

TEST_CASE("GpuEvalBackend: name reflects stub status (no silent fallback)") {
    GpuEvalBackend gpu;
    const char* n = gpu.name();
    CHECK(n != nullptr);
    // The name must contain "stub" (or "fallback") so callers can
    // detect that this is NOT a real GPU backend.
    const std::string s(n);
    CHECK(s.find("stub") != std::string::npos);
}

TEST_CASE("CpuEvalBackend: name is \"cpu\"") {
    CpuEvalBackend cpu;
    CHECK(std::string(cpu.name()) == "cpu");
}

TEST_CASE("GpuEvalBackend: evaluate matches CpuEvalBackend on the same input") {
    // The GPU stub falls back to a fresh CpuEvalBackend, so the output
    // must be identical to a direct CPU evaluation. This guards against
    // future divergence if someone "optimizes" the stub.
    auto body = make_sphere(2.0f);
    std::vector<Vec3f> pts = {{0, 0, 0}, {2, 0, 0}, {3, 0, 0}, {1, 1, 1}};
    EvalResult out_cpu, out_gpu;
    CpuEvalBackend cpu;
    GpuEvalBackend  gpu;
    cpu.evaluate(body, std::span<const Vec3f>(pts.data(), pts.size()), out_cpu);
    gpu.evaluate(body, std::span<const Vec3f>(pts.data(), pts.size()), out_gpu);
    REQUIRE(out_cpu.values.size() == out_gpu.values.size());
    for (std::size_t i = 0; i < out_cpu.values.size(); ++i) {
        CHECK(out_gpu.values[i] == doctest::Approx(out_cpu.values[i]).epsilon(1e-5f));
    }
}

TEST_CASE("GpuEvalBackend: polymorphic dispatch through EvalBackend base") {
    // Verify that the EvalBackend abstract base works polymorphically —
    // the vtable for GpuEvalBackend must be properly emitted in the .cpp
    // (not inlined in the header), or else this test would fail to link.
    auto body = make_sphere(1.0f);
    std::vector<Vec3f> pts = {{0, 0, 0}};
    EvalResult out;

    EvalBackend* backend = nullptr;
    GpuEvalBackend gpu;
    backend = &gpu;
    backend->evaluate(body, std::span<const Vec3f>(pts.data(), pts.size()), out);
    CHECK(out.values[0] == doctest::Approx(-1.0f).epsilon(1e-5f));

    // The name() call goes through the vtable.
    CHECK(std::string(backend->name()).find("stub") != std::string::npos);
}
