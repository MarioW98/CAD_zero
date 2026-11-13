// tests/unit/sdf/test_perf2_mesh_memory.cpp
//
// PERF 2 — THE BIT-IDENTITY PINS AND THE MEMORY GATE (the CI seat of
// the slice-streamed tessellator, the register's perf row's own).
//
// Two assertions, the brief's own two, and nothing else:
//
//   1. THE DIGEST PINS — the tessellation's output must be
//      BIT-IDENTICAL to the volume-built walk at every resolution
//      tested: same triangles, same positions, same normals, same
//      indices. The digests are FNV-1a 64 over the canonical
//      rendering (the counts in the header, every float as its %08x
//      bit pattern, every index likewise — the m5/m6/m7/m8 suites'
//      own instrument, the cross-run identity's single authority in
//      this house). A reduction that changes one triangle is a
//      defect, not an improvement — the pin says so in one
//      comparison. The pins were read from the VOLUME-BUILT walk
//      (the pre-PERF-2 tree, f025375) with this same corpus and
//      this same rendering: they are the old walk's own bits.
//
//   2. THE MEMORY GATE — the tessellation's peak at resolution 256
//      must stay under a threshold RELATIVE to the baseline, with
//      the margin declared: the baseline is the volume walk's five
//      live arrays (points 12 B/pt + field 4 + gradient 12 + values
//      4 + gradients 12 = 44 B/pt; 44 x 257^3 = 746,882,092 B =
//      712.28 MiB — measured 713.32 MiB peak-RSS delta in a fresh
//      process on the pre-change tree, the same probe, 0.15%
//      apart), and the gate sits at 1/4 of it (186,720,523 B): the
//      slice-streamed walk measured 73.17 MiB in the same fresh
//      process conditions (~2.4x below the gate), and a REVERT to
//      the volume walk lands at ~712 MiB, ~4x ABOVE the gate — the
//      red cannot hide. Memory, unlike frame-time, is measured in
//      CI: this test is a ctest member and runs in every column.
//
// THE METHOD, declared: the allocator's retained arena is returned
// to the OS first (malloc_trim — a warm arena would pre-count the
// extraction's pages in the floor); the peak is the process's own
// VmHWM read from /proc/self/status AFTER /proc/self/clear_refs is
// written "5" (the peak-RSS reset, Linux 2.6.35+); the delta is
// VmHWM minus the VmRSS floor at reset time, so whatever the
// process retained before the gate is not counted against the
// extraction. Linux is
// the probe's own home: where /proc is absent the gate and its
// counterproof SKIP DECLARED (never a silence), the digests still
// judge every platform.
//
// THE COUNTERPROOF (a gate that cannot say no is a decorative
// number, not a gate): the volume walk's own five arrays — the same
// sizes the old tree allocated, every element written AND read
// through a volatile sink so the optimizer cannot elide the
// footprint — must push the SAME probe ABOVE the same threshold:
// the gate goes red on the simulated revert, named. The digests'
// own counterproof is the pin itself: one drifted bit moves the FNV
// and the test fails naming the resolution.
#include <doctest/doctest.h>

#include "CAD_0/sdf/primitives.hpp"
#include "CAD_0/sdf/mesh_extract.hpp"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <malloc.h>
#include <string>
#include <vector>

namespace s = CAD_0::sdf;

// ---------------------------------------------------------------------------
// FNV-1a 64 — the m5/m6/m7/m8 suites' own instrument, quoted.
// ---------------------------------------------------------------------------
namespace {

std::uint64_t fnv_feed(std::uint64_t h, const char* data, std::size_t n) {
    for (std::size_t i = 0; i < n; ++i) {
        h ^= static_cast<std::uint64_t>(
            static_cast<unsigned char>(data[i]));
        h *= 0x100000001b3ull;
    }
    return h;
}

std::string hex16(std::uint64_t v) {
    char buf[24];
    std::snprintf(buf, sizeof buf, "%016llx",
                  static_cast<unsigned long long>(v));
    return std::string(buf);
}

// The canonical rendering, chunked so no giant string is ever built:
// the header carries the counts, every position (and normal, when
// asked) renders as three %08x bit patterns, every triangle as three
// %08x indices. The rendering is the probe's own — the pins were read
// with it on the pre-change tree and this suite never re-calibrates
// them: the identity, not the number, is the contract.
std::uint64_t mesh_digest(const s::TriangleMesh& m, std::uint32_t res,
                          bool normals) {
    std::uint64_t h = 0xcbf29ce484222325ull;
    char buf[160];
    int len = std::snprintf(buf, sizeof buf,
                            "perf2-mesh sphere r=1 res=%u normals=%u "
                            "verts=%zu tris=%zu\n",
                            res, normals ? 1u : 0u,
                            m.positions.size(), m.triangle_count());
    h = fnv_feed(h, buf, len);
    for (std::size_t i = 0; i < m.positions.size(); ++i) {
        std::uint32_t bx, by, bz;
        std::memcpy(&bx, &m.positions[i].x, 4);
        std::memcpy(&by, &m.positions[i].y, 4);
        std::memcpy(&bz, &m.positions[i].z, 4);
        len = std::snprintf(buf, sizeof buf, "v %08x %08x %08x\n",
                            bx, by, bz);
        h = fnv_feed(h, buf, len);
        if (normals && i < m.normals.size()) {
            std::uint32_t nx, ny, nz;
            std::memcpy(&nx, &m.normals[i].x, 4);
            std::memcpy(&ny, &m.normals[i].y, 4);
            std::memcpy(&nz, &m.normals[i].z, 4);
            len = std::snprintf(buf, sizeof buf, "n %08x %08x %08x\n",
                                nx, ny, nz);
            h = fnv_feed(h, buf, len);
        }
    }
    for (std::size_t t = 0; t < m.triangle_count(); ++t) {
        len = std::snprintf(buf, sizeof buf, "t %08x %08x %08x\n",
                            m.indices[t * 3 + 0],
                            m.indices[t * 3 + 1],
                            m.indices[t * 3 + 2]);
        h = fnv_feed(h, buf, len);
    }
    return h;
}

// ---------------------------------------------------------------------------
// The corpus's pins — read from the VOLUME-BUILT walk (the pre-PERF-2
// tree, f025375) with this same corpus (sphere r=1, the legacy
// padded-bounds overload, the ADR-0015 resolution ladder's own
// primitive) and this same canonical rendering. The counts ride the
// table so a failure names its shape, not only its FNV.
// ---------------------------------------------------------------------------
struct DigestPin {
    std::uint32_t resolution;
    bool normals;
    std::size_t verts;
    std::size_t tris;
    std::uint64_t fnv;
};

constexpr DigestPin kPins[] = {
    {16, true, 1158, 2312, 0x2c1f89226dcce752ull},
    {32, true, 4662, 9320, 0x6ea6f4ba616799eeull},
    {64, true, 18558, 37112, 0xc0817a26cd3f421aull},
    {96, true, 41766, 83528, 0x3b8b864370c51d21ull},
    {64, false, 18558, 37112, 0xa027411fcc99a186ull},
};

// The gate's own pins — the baseline arithmetic and the threshold.
constexpr std::uint64_t kGridSide = 257;  // resolution 256 -> 257 vertices/axis
constexpr std::uint64_t kGridPoints =
    kGridSide * kGridSide * kGridSide;  // 16,974,593
constexpr std::uint64_t kOldWalkBytes =
    44ull * kGridPoints;  // the volume walk's five live arrays
constexpr double kGateFraction = 0.25;  // the declared margin: 1/4 of baseline

constexpr std::uint64_t kGateThresholdBytes =
    kOldWalkBytes / 4;  // 186,720,523 B ~ 178 MiB

// The 256^3 digest pin — the gate's own resolution, verified inside
// the gate so one extraction carries both assertions.
constexpr std::uint64_t kPin256Fnv = 0x6bd925dfe2806f8bull;
constexpr std::size_t kPin256Verts = 296910;
constexpr std::size_t kPin256Tris = 593816;

#ifdef __linux__

// ---------------------------------------------------------------------------
// THE PEAK PROBE — VmHWM (the kernel's own peak-RSS counter) with the
// clear_refs "5" reset, the delta measured against the reset-time
// floor so prior retention is not counted against the extraction.
// ---------------------------------------------------------------------------
long read_status_kb(const char* key) {
    std::FILE* f = std::fopen("/proc/self/status", "r");
    if (f == nullptr) return -1;
    char line[256];
    long val = -1;
    while (std::fgets(line, sizeof line, f) != nullptr) {
        if (std::strncmp(line, key, std::strlen(key)) == 0) {
            val = std::atol(line + std::strlen(key));
            break;
        }
    }
    std::fclose(f);
    return val;
}

bool reset_peak() {
    std::FILE* f = std::fopen("/proc/self/clear_refs", "w");
    if (f == nullptr) return false;
    const bool ok = std::fputs("5", f) >= 0;
    std::fclose(f);
    return ok;
}

// THE PROBE, whole: trim (return the allocator's retained arena to
// the OS, so the extraction's own pages are never pre-counted in the
// floor), then begin (reset + floor) then end (peak − the reset-time
// floor). The floor is captured AT RESET — never the current RSS at
// read time, which would collapse to zero against memory that is
// still live (the counterproof's own catch: a probe that measures
// only transient overshoot is not a probe).
struct PeakProbe {
    long long floor_bytes;
    bool ok;
};

PeakProbe begin_probe() {
    malloc_trim(0);
    if (!reset_peak()) return {-1, false};
    const long rss = read_status_kb("VmRSS:");
    if (rss < 0) return {-1, false};
    return {static_cast<long long>(rss) * 1024, true};
}

long long end_probe_bytes(const PeakProbe& p) {
    const long hwm = read_status_kb("VmHWM:");
    if (!p.ok || hwm < 0) return -1;
    return static_cast<long long>(hwm) * 1024 - p.floor_bytes;
}

#endif  // __linux__

}  // namespace

// ---------------------------------------------------------------------------
// 1. THE DIGEST PINS — the bit-identity, judged on every platform.
// ---------------------------------------------------------------------------
TEST_CASE("perf2: the slice-streamed tessellator is bit-identical - the "
          "pinned digests at every tested resolution") {
    auto sph = s::make_sphere(1.0f);
    for (const DigestPin& pin : kPins) {
        auto mesh = s::marching_cubes(sph, pin.resolution, pin.normals);
        INFO("resolution ", pin.resolution,
             " normals ", pin.normals ? 1 : 0);
        REQUIRE(mesh.vertex_count() == pin.verts);
        REQUIRE(mesh.triangle_count() == pin.tris);
        const std::uint64_t d =
            mesh_digest(mesh, pin.resolution, pin.normals);
        CHECK_MESSAGE(d == pin.fnv,
                      "the digest DRIFTED at resolution ",
                      pin.resolution, " (normals ",
                      pin.normals ? 1 : 0, "): got ", hex16(d),
                      " want ", hex16(pin.fnv),
                      " - a reduction that changes the triangles is a "
                      "defect, not an improvement");
        std::printf("  [perf2] res=%3u normals=%u  verts=%7zu  "
                    "tris=%7zu  digest=%s == pin\n",
                    pin.resolution, pin.normals ? 1u : 0u,
                    mesh.vertex_count(), mesh.triangle_count(),
                    hex16(d).c_str());
    }
}

#ifdef __linux__

// ---------------------------------------------------------------------------
// 2. THE MEMORY GATE at 256^3 — one extraction, both assertions: the
// digest pin (the second assertion of the brief) and the peak under
// the threshold (the first).
// ---------------------------------------------------------------------------
TEST_CASE("perf2: the memory gate at 256^3 - the peak is the slice's and "
          "the surface's, never the volume's") {
    const PeakProbe probe = begin_probe();
    REQUIRE(probe.ok);

    auto sph = s::make_sphere(1.0f);
    auto mesh = s::marching_cubes(sph, 256, true);

    const long long delta = end_probe_bytes(probe);
    REQUIRE(delta >= 0);

    // The bit-identity, judged inside the gate: the extraction that
    // paid the peak is the extraction whose digest is believed.
    REQUIRE(mesh.vertex_count() == kPin256Verts);
    REQUIRE(mesh.triangle_count() == kPin256Tris);
    const std::uint64_t d = mesh_digest(mesh, 256, true);
    CHECK_MESSAGE(d == kPin256Fnv,
                  "the 256^3 digest DRIFTED: got ", hex16(d),
                  " want ", hex16(kPin256Fnv),
                  " - the memory numbers below are NOT to be believed");

    std::printf("  [perf2] gate 256^3: peak delta %.2f MiB "
                "(floor %.2f MiB), threshold %.2f MiB "
                "= %.0f%% of the %.2f MiB baseline\n",
                static_cast<double>(delta) / 1048576.0,
                static_cast<double>(probe.floor_bytes) / 1048576.0,
                static_cast<double>(kGateThresholdBytes) / 1048576.0,
                100.0 * kGateFraction,
                static_cast<double>(kOldWalkBytes) / 1048576.0);

    CHECK_MESSAGE(static_cast<std::uint64_t>(delta) < kGateThresholdBytes,
                  "the tessellation's peak at 256^3 (", delta,
                  " B) grew past the gate (", kGateThresholdBytes,
                  " B = ", kGateFraction * 100.0,
                  "% of the volume walk's ",
                  kOldWalkBytes,
                  " B): memory is growing with the VOLUME again, not "
                  "with the slice and the surface");
}

// ---------------------------------------------------------------------------
// 3. THE COUNTERPROOF — the volume walk's five arrays, the same sizes
// the old tree allocated, every element written AND read through a
// volatile sink (the optimizer may not elide the footprint): the SAME
// probe must read ABOVE the same threshold. The gate's teeth, proven
// in CI, on the simulated revert.
// ---------------------------------------------------------------------------
TEST_CASE("perf2: the counterproof - the volume walk's five arrays trip "
          "the gate red") {
    const PeakProbe probe = begin_probe();
    REQUIRE(probe.ok);

    volatile float sink = 0.0f;
    long long delta = -1;
    {
        // The five arrays the volume-built walk held live together
        // during its bulk evaluate: points, field, gradient, values,
        // gradients — 44 B/pt.
        const std::size_t total =
            std::size_t(kGridSide) * kGridSide * kGridSide;
        std::vector<CAD_0::math::Vec3f> points(total);
        std::vector<float> field(total);
        std::vector<CAD_0::math::Vec3f> grad(total);
        std::vector<float> values(total);
        std::vector<CAD_0::math::Vec3f> gradients(total);
        for (std::size_t i = 0; i < total; ++i) {
            points[i] = {1.0f, 2.0f, 3.0f};
            field[i] = 1.0f;
            grad[i] = {1.0f, 0.0f, 0.0f};
            values[i] = -1.0f;
            gradients[i] = {0.0f, 1.0f, 0.0f};
        }
        // Read back through the sink: the stores are live, the pages
        // are resident, the footprint is the old walk's own.
        float acc = 0.0f;
        for (std::size_t i = 0; i < total; ++i) {
            acc += points[i].x + field[i] + grad[i].x + values[i] +
                   gradients[i].y;
        }
        sink = acc;
        delta = end_probe_bytes(probe);
    }
    REQUIRE(delta >= 0);
    std::printf("  [perf2] counterproof: the five arrays' peak delta "
                "%.2f MiB vs the %.2f MiB threshold\n",
                static_cast<double>(delta) / 1048576.0,
                static_cast<double>(kGateThresholdBytes) / 1048576.0);
    CHECK_MESSAGE(
        static_cast<std::uint64_t>(delta) > kGateThresholdBytes,
        "the counterproof FAILED to trip the gate: the volume walk's "
        "own five arrays (", kOldWalkBytes, " B) measured ", delta,
        " B, under the threshold ", kGateThresholdBytes,
        " B - a gate that cannot say no is a decorative number");
    // The sink is read so the reads are live: the counterproof's own
    // footprint is the point, and the compiler is told so.
    CHECK(sink != -12345.0f);
}

#else  // !__linux__

TEST_CASE("perf2: the memory gate - declared skip off Linux") {
    std::printf("  [perf2] the /proc peak probe (VmHWM + clear_refs) is "
                "Linux's own: the memory gate and its counterproof SKIP "
                "here, DECLARED - never a silence; the digest pins above "
                "still judge this platform, and the CI's three Linux "
                "columns judge the gate every run\n");
}

#endif  // __linux__
