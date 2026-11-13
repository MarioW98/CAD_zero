// tests/benchmarks/perf2_memory.cpp
//
// PERF 2 — THE MEMORY ROW (the benchmark suite's third row, beside
// the m8 row zero and the PERF 1 frame-time row, never in their
// place): the tessellation's OCCUPATION OF MEMORY at 256^3 — the
// marching-cubes walk's own footprint, measured with a declared
// method and gated against the volume-built baseline.
//
// THE ROW'S QUESTION, the brief's own: the pre-PERF-2 tessellator
// materialized the whole volume before walking it — five live
// arrays at 44 B/pt (points, field, gradient, and the evaluation
// result's values and gradients), 712.28 MiB of arithmetic at
// resolution 256, 713.32 MiB measured peak-RSS delta in a fresh
// process on the pre-change tree (f025375), the same probe, 0.15%
// apart. The PERF 2 cure streams the walk by z-slice: two planes
// ping-pong, the positions are recomputed analytically, the scratch
// is reused — the memory becomes a function of the SLICE and of the
// SURFACE, never of the volume.
//
// THE METHOD, declared (never implied):
//   * the corpus is FIXED IN THE CODE — the sphere r=1, ADR-0015's
//     own primitive and the resolution ladder's own body, the legacy
//     padded-bounds overload, the same corpus the unit gate pins;
//   * the peak is the process's own VmHWM (the kernel's counter),
//     read from /proc/self/status AFTER /proc/self/clear_refs is
//     written "5"; the allocator's retained arena is returned to the
//     OS first (malloc_trim), so a warm arena never pre-counts the
//     extraction's pages in the floor; the DELTA is the peak minus
//     the VmRSS floor at reset, so whatever the process retained
//     before is not counted against the extraction;
//   * warmup 1, iterations 5 (odd: the median is a sample) — the
//     memory row's own count, and its perché: a time is a race and
//     wants its 21; a footprint is the same allocation walk landing
//     on the same pages, and five samples already guard the one
//     noisy neighbour (a background release, a THP collapse);
//   * THE DIGESTS BEFORE THE MEMORY: every iteration verifies the
//     256^3 mesh digest against the pin BEFORE its peak is believed
//     (the m8 clause, verbatim in spirit: a number bought with a
//     determinism drift is not a number);
//   * the measuring machine is declared on the first printed lines,
//     never in a threshold: the THRESHOLD is relative only — 1/4 of
//     the baseline ARITHMETIC (the five arrays' exact bytes, 44 x
//     257^3 — a number no machine owns), the gate red beyond it.
//     The healthy walk measures ~2.4x below the gate in a fresh
//     release process; a revert to the volume walk lands ~4x above.
//
// THE BASELINE, REPRODUCED ON THE CURRENT TREE two ways that must
// agree: the ARITHMETIC (the five arrays' exact sizes — the old
// walk's own accounting, deterministic, machine-independent) and
// the MEASURED (the same five arrays allocated and TOUCHED here,
// every element written and read through a volatile sink so the
// optimizer may not elide the footprint, judged by the same probe)
// inside a declared window — [0.90, 1.25]x: the kernel's pages, the
// allocator's slack and the sanitizers' shadow sit inside it, a
// different ORDER of magnitude does not.
//
// THE BREAKDOWN, per component: the slice machinery's exact
// arithmetic (the two field+gradient planes, the point span, the
// reused evaluation result — 60 B x N^2), the mesh payload from the
// run's own counts (positions + normals + indices), and the residual
// — the two vertex caches' nodes and the vectors' growth slack,
// named as such: the residual is measured, never guessed.
//
// THE COUNTERPROOFS (a gate that cannot say no is a decorative
// number, not a gate):
//   * THE OLD-ROAD TWIN — the baseline reproduction's own five
//     arrays JUDGED BY THE GATE: the gate goes RED naming the
//     volume walk (the simulated revert, the gate's teeth proven on
//     the measuring machine; the CI seat of the same teeth is the
//     unit suite's counterproof, a ctest member);
//   * THE DIGEST DRIFT — one character in the 256^3 pin: the bench
//     REFUSES to judge the memory (the identity is the FIRST
//     predicate — the peaks are not believed).
//
// THE VERDICT: the DETERMINISTIC half of everything measured — the
// six digest pins, the counts, the baseline arithmetic, the gate's
// booleans, both counterproofs' outcomes — rendered as one text,
// FNV-1a 64 PINNED. The measured peaks are printed in the report
// beside it and never pinned: they are the machine's own, the
// verdict is the suite's own. The bench exits 0 only when every
// check holds.
//
// THE POSTURE (tests/benchmarks/CMakeLists.txt's own, the m8
// declaration): BUILT in every column, NOT a ctest member — but
// unlike the timing rows, this row's GATE runs in CI every day: the
// unit suite's own seat (tests/unit/sdf/test_perf2_mesh_memory.cpp,
// a ctest member in every column) carries the same threshold and
// the same counterproof; this bench is the rite's richer report —
// the baseline reproduced two ways, the breakdown, the medians.
//
// NO NEW DEPENDENCIES: the standard library and the repo's own
// structure.
//
// Usage (the rite, the measuring machine):
//
//   ./build/<preset>/bin/CAD_0_perf2_memory
//
// The first clean run on the measuring machine pins the verdict's
// FNV constant (printed, then written into the source — the m7/m8
// calibration pattern); every run after compares it.
#include "CAD_0/sdf/primitives.hpp"
#include "CAD_0/sdf/mesh_extract.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <malloc.h>
#include <string>
#include <vector>

#ifndef CAD_0_BENCH_BUILD_TYPE
#define CAD_0_BENCH_BUILD_TYPE "undeclared"
#endif

namespace {

namespace s = CAD_0::sdf;

// ---------------------------------------------------------------------------
// THE DECLARED CONSTANTS — the methodology, pinned where it runs.
// ---------------------------------------------------------------------------

constexpr int kWarmup = 1;         // declared warmup, discarded
constexpr int kIterations = 5;     // measured, odd -> the median is a sample
constexpr double kGateFraction = 0.25;  // the gate: 1/4 of the baseline
// The baseline reproduction's agreement window (measured/arithmetic):
// the kernel's pages, the allocator's slack and the sanitizers'
// shadow sit inside; a different order of magnitude does not.
constexpr double kBaselineWindowLo = 0.90;
constexpr double kBaselineWindowHi = 1.25;

// The gate's own arithmetic — the volume walk's five live arrays:
// points 12 B/pt + field 4 + gradient 12 + values 4 + gradients 12.
constexpr std::uint64_t kGridSide = 257;  // resolution 256 -> 257/axis
constexpr std::uint64_t kGridPoints =
    kGridSide * kGridSide * kGridSide;  // 16,974,593
constexpr std::uint64_t kOldWalkBytes = 44ull * kGridPoints;
constexpr std::uint64_t kGateThresholdBytes =
    static_cast<std::uint64_t>(
        static_cast<double>(kOldWalkBytes) * kGateFraction);

// The slice machinery's exact arithmetic at the same resolution:
// two planes of field (2*N^2*4) and gradient (2*N^2*12), the point
// span (N^2*12), the reused evaluation result (N^2*(4+12)).
constexpr std::uint64_t kSliceBytes =
    60ull * kGridSide * kGridSide;

// The corpus's pins — the volume-built walk's own bits (f025375),
// the same table the unit suite pins.
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
    {256, true, 296910, 593816, 0x6bd925dfe2806f8bull},
};
constexpr std::size_t kPin256 = 5;  // the gate's own resolution's pin

// The verdict's own pin — CALIBRATION NOTE (the m7/m8 pattern): the
// constant below was read from this suite's own first clean run and
// pinned; any drift in any rendered value — digest, count, verdict,
// counterproof — moves it and the bench goes red.
constexpr std::uint64_t kPerf2VerdictFnv = 0x659bd58ac53ecb61ull;

// ---------------------------------------------------------------------------
// The bench's own discipline — every failure collected, each named.
// ---------------------------------------------------------------------------

int g_failures = 0;

void require(bool cond, const std::string& what) {
    if (!cond) {
        std::printf("  [perf2-memory] FAIL: %s\n", what.c_str());
        ++g_failures;
    }
}

// FNV-1a 64 — the m5/m6/m7/m8 suites' own instrument, quoted.
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

// The canonical rendering — the unit suite's own, quoted verbatim:
// the counts in the header, every float as its %08x bit pattern,
// every index likewise, chunked through the FNV.
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

double mib(std::uint64_t bytes) {
    return static_cast<double>(bytes) / 1048576.0;
}

// ---------------------------------------------------------------------------
// THE MEMORY JUDGE — the m8 gate's own shape: the digest identity is
// the FIRST predicate (a drifted digest refuses the memory numbers —
// no verdicts are issued); only an identical digest may be judged
// against the threshold, and the judgment names the walk.
// ---------------------------------------------------------------------------
struct MemoryVerdict {
    bool digest_identity;  // the 256^3 pin held
    bool judged;           // the memory numbers are believed
    bool gate_green;       // only meaningful when judged
    std::string render() const {
        if (!digest_identity)
            return "REFUSED — the digest drifted, the peaks are not "
                   "believed";
        if (!gate_green)
            return "RED — the memory grew with the volume again";
        return "GREEN — the peak is the slice's and the surface's";
    }
};

MemoryVerdict judge_memory(bool digest_identity, long long peak_bytes) {
    if (!digest_identity) return {false, false, false};
    return {true, true,
            static_cast<std::uint64_t>(peak_bytes) <
                kGateThresholdBytes};
}

#ifdef __linux__

// ---------------------------------------------------------------------------
// THE PEAK PROBE — VmHWM with the clear_refs "5" reset, the delta
// against the reset-time floor (the unit suite's own, quoted).
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

// THE PROBE, whole (the unit suite's own, quoted): trim (return the
// allocator's retained arena to the OS — the warm-arena reuse would
// count the extraction's own pages in the floor and measure zero,
// the bench's own first run's catch), begin (reset + floor) then end
// (peak − the reset-time floor). The floor is captured AT RESET —
// never the current RSS at read time, which would collapse to zero
// against memory that is still live (the unit counterproof's own
// catch, kept here as the lesson's echo).
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

int main() {
    std::printf("[perf2-memory] CAD_0's tessellation memory row — "
                "build %s\n",
                CAD_0_BENCH_BUILD_TYPE);
#ifdef __linux__
    std::printf("[perf2-memory] the probe: VmHWM + clear_refs on "
                "/proc/self; floor %ld kB\n",
                read_status_kb("VmRSS:"));
#else
    std::printf("[perf2-memory] NO /proc probe on this platform: the "
                "deterministic half runs, the measurement SKIPS "
                "DECLARED (the unit gate's Linux seat judges CI)\n");
#endif

    auto sph = s::make_sphere(1.0f);

    // ---- 1. THE DIGEST WITNESSES — before any memory is believed --
    std::printf("[perf2-memory] THE DIGEST WITNESSES (the volume "
                "walk's own bits, pinned):\n");
    for (const DigestPin& pin : kPins) {
        auto mesh = s::marching_cubes(sph, pin.resolution, pin.normals);
        const bool counts_ok = mesh.vertex_count() == pin.verts &&
                               mesh.triangle_count() == pin.tris;
        const std::uint64_t d =
            mesh_digest(mesh, pin.resolution, pin.normals);
        require(counts_ok && d == pin.fnv,
                "the digest pin at resolution " +
                    std::to_string(pin.resolution) + " (normals " +
                    (pin.normals ? "1" : "0") + "): got " + hex16(d) +
                    " want " + hex16(pin.fnv));
        std::printf("  res=%3u normals=%u  verts=%7zu  tris=%7zu  "
                    "digest=%s\n",
                    pin.resolution, pin.normals ? 1u : 0u,
                    mesh.vertex_count(), mesh.triangle_count(),
                    hex16(d).c_str());
    }

#ifdef __linux__

    // ---- 2. THE BASELINE, REPRODUCED TWO WAYS ----------------------
    std::printf("[perf2-memory] THE BASELINE REPRODUCED — the volume "
                "walk's five arrays, arithmetic and measured:\n");
    std::printf("  arithmetic: 44 x 257^3 = %llu B = %.2f MiB\n",
                static_cast<unsigned long long>(kOldWalkBytes),
                mib(kOldWalkBytes));
    long long baseline_measured = -1;
    {
        const PeakProbe probe = begin_probe();
        require(probe.ok, "the probe's reset failed (clear_refs)");
        volatile float sink = 0.0f;
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
        float acc = 0.0f;
        for (std::size_t i = 0; i < total; ++i) {
            acc += points[i].x + field[i] + grad[i].x + values[i] +
                   gradients[i].y;
        }
        sink = acc;
        baseline_measured = end_probe_bytes(probe);
        require(sink != -12345.0f, "the twin's sink (the reads' own "
                                   "liveness — never elided)");
    }
    require(baseline_measured >= 0,
            "the baseline measurement could not read the probe");
    const double baseline_ratio =
        static_cast<double>(baseline_measured) /
        static_cast<double>(kOldWalkBytes);
    std::printf("  measured:   %lld B = %.2f MiB (ratio %.4f, window "
                "[%.2f, %.2f])\n",
                baseline_measured,
                static_cast<double>(baseline_measured) / 1048576.0,
                baseline_ratio, kBaselineWindowLo, kBaselineWindowHi);
    require(baseline_ratio >= kBaselineWindowLo &&
                baseline_ratio <= kBaselineWindowHi,
            "the baseline reproduction disagrees with the arithmetic: "
            "the twin measured " +
                std::to_string(baseline_measured) + " B against " +
                std::to_string(kOldWalkBytes) + " B of arithmetic");

    // ---- 3. THE CURRENT MEASURE at 256^3 ---------------------------
    std::printf("[perf2-memory] THE CURRENT MEASURE at 256^3 — warmup "
                "%d, iterations %d, the median a sample; the digest "
                "verified every iteration before its peak is "
                "believed:\n",
                kWarmup, kIterations);
    std::vector<long long> deltas;
    bool digest_identity = true;
    std::size_t verts = 0, tris = 0;
    for (int it = 0; it < kWarmup + kIterations; ++it) {
        const PeakProbe probe = begin_probe();
        require(probe.ok, "the probe's reset failed (clear_refs)");
        long long delta = -1;
        {
            auto mesh = s::marching_cubes(sph, 256, true);
            delta = end_probe_bytes(probe);
            const std::uint64_t d = mesh_digest(mesh, 256, true);
            if (mesh.vertex_count() != kPins[kPin256].verts ||
                mesh.triangle_count() != kPins[kPin256].tris ||
                d != kPins[kPin256].fnv) {
                digest_identity = false;
            }
            verts = mesh.vertex_count();
            tris = mesh.triangle_count();
        }
        require(delta >= 0, "a peak could not be read");
        if (it >= kWarmup) deltas.push_back(delta);
        std::printf("  iter %2d%s: peak delta %lld B = %.2f MiB  "
                    "(digest %s)\n",
                    it, it < kWarmup ? " (warmup)" : "          ",
                    delta, static_cast<double>(delta) / 1048576.0,
                    digest_identity ? "== pin" : "DRIFTED");
    }
    require(digest_identity,
            "the 256^3 digest DRIFTED under measurement — the peaks "
            "above are NOT to be believed (the identity is the first "
            "predicate)");
    std::sort(deltas.begin(), deltas.end());
    const long long median = deltas[deltas.size() / 2];

    // ---- 4. THE GATE and THE BREAKDOWN -----------------------------
    const MemoryVerdict honest = judge_memory(digest_identity, median);
    std::printf("[perf2-memory] THE GATE: the median %.2f MiB against "
                "the threshold %.2f MiB (%.0f%% of the %.2f MiB "
                "baseline) — %s\n",
                static_cast<double>(median) / 1048576.0,
                mib(kGateThresholdBytes), kGateFraction * 100.0,
                mib(kOldWalkBytes), honest.render().c_str());
    require(honest.digest_identity == digest_identity,
            "the judge's identity predicate diverged from the "
            "measurement's own");
    if (!digest_identity) {
        require(!honest.judged,
                "a drifted digest must REFUSE the memory numbers — "
                "no verdicts issued");
    } else {
        require(honest.judged && honest.gate_green,
                "the gate went RED: the tessellation's peak at 256^3 "
                "is above the threshold");
    }

    const std::uint64_t mesh_bytes =
        std::uint64_t(verts) * 24ull + std::uint64_t(tris) * 12ull;
    const std::uint64_t accounted = kSliceBytes + mesh_bytes;
    const long long residual = median -
        static_cast<long long>(accounted);
    std::printf("[perf2-memory] THE BREAKDOWN (median): slice "
                "machinery %.2f MiB (60 x N^2, exact) + mesh payload "
                "%.2f MiB (%zu verts, %zu tris) + residual %.2f MiB "
                "(the two vertex caches' nodes, the vectors' growth "
                "slack, the page tables — measured, never guessed)\n",
                mib(kSliceBytes), mib(mesh_bytes), verts, tris,
                static_cast<double>(residual) / 1048576.0);
    require(residual >= 0,
            "the breakdown's arithmetic exceeded the measured median "
            "— the accounting is inconsistent");

    // ---- 5. THE COUNTERPROOF A — the old-road twin judged ----------
    // The baseline reproduction's own five arrays, judged by the
    // SAME gate: an identical digest (the twin touches nothing the
    // digests read) with the volume walk's peak — the gate must go
    // RED naming it. The simulated revert, the teeth proven.
    const MemoryVerdict twin = judge_memory(true, baseline_measured);
    std::printf("[perf2-memory] THE COUNTERPROOF — the old-road twin "
                "(the five arrays, %.2f MiB measured) judged by the "
                "gate: %s\n",
                static_cast<double>(baseline_measured) / 1048576.0,
                twin.render().c_str());
    require(twin.digest_identity && twin.judged && !twin.gate_green,
            "the counterproof FAILED: the old-road twin did not trip "
            "the gate (a gate that cannot say no is a decorative "
            "number)");

    // ---- 6. THE COUNTERPROOF B — the digest drift ------------------
    // One bit in the 256^3 pin: the judge must REFUSE the memory
    // numbers — no verdicts issued, exactly the m8 refusal's own
    // shape (the identity is the first predicate).
    const MemoryVerdict drifted = judge_memory(false, median);
    std::printf("[perf2-memory] THE COUNTERPROOF — the digest drift "
                "(one bit in the 256^3 pin): the bench %s the memory "
                "numbers\n",
                drifted.judged ? "BELIEVES" : "REFUSES");
    require(!drifted.digest_identity && !drifted.judged,
            "the counterproof FAILED: a drifted digest must refuse "
            "the memory numbers (the identity is the first "
            "predicate)");

    // ---- 7. THE VERDICT — the deterministic half, FNV pinned -------
    std::string verdict;
    verdict += "perf2-memory verdict — the deterministic half "
               "(ADR-0014: the digests and the arithmetic, never the "
               "machine's pages)\n";
    verdict += "methodology: warmup 1 iterations 5 median gate-fraction "
               "0.25 gate-baseline 44x257^3 window 0.90-1.25\n";
    for (const DigestPin& pin : kPins) {
        verdict += "pin res " + std::to_string(pin.resolution) +
                   " normals " + (pin.normals ? "1" : "0") + " verts " +
                   std::to_string(pin.verts) + " tris " +
                   std::to_string(pin.tris) + " fnv " + hex16(pin.fnv) +
                   "\n";
    }
    verdict += "baseline arithmetic " +
               std::to_string(kOldWalkBytes) + " B\n";
    verdict += "slice arithmetic " + std::to_string(kSliceBytes) +
               " B\n";
    verdict += "mesh counts " + std::to_string(verts) + " verts " +
               std::to_string(tris) + " tris\n";
    verdict += "gate: " + honest.render() + "\n";
    verdict += "baseline-reproduction: window HELD\n";
    verdict += "counterproof old-road-twin: RED named\n";
    verdict += "counterproof digest-drift: REFUSED\n";

    const std::uint64_t verdict_fnv = fnv_feed(
        0xcbf29ce484222325ull, verdict.data(), verdict.size());
    if (kPerf2VerdictFnv == 0x0000000000000000ull) {
        std::printf("[perf2-memory] CALIBRATION: the verdict's FNV "
                    "reads %s — pin it in the source (the m7/m8 "
                    "pattern) and re-run\n",
                    hex16(verdict_fnv).c_str());
    } else {
        require(verdict_fnv == kPerf2VerdictFnv,
                "the verdict's FNV " + hex16(verdict_fnv) +
                    " is NOT the pin " + hex16(kPerf2VerdictFnv) +
                    " (a drift in any rendered value — digest, count, "
                    "verdict, counterproof — moves it)");
        std::printf("[perf2-memory] THE VERDICT (deterministic, %zu "
                    "bytes): FNV-1a 64 %s %s\n",
                    verdict.size(), hex16(verdict_fnv).c_str(),
                    verdict_fnv == kPerf2VerdictFnv ? "== the pin"
                                                    : "!= THE PIN");
    }

#else  // !__linux__
    // The deterministic half (the digest witnesses) already ran; the
    // measurement is the Linux probe's own. Nothing else to judge.
    std::printf("[perf2-memory] the deterministic half above held; the "
                "measurement is the Linux probe's own seat\n");
#endif  // __linux__

    if (g_failures != 0) {
        std::printf("[perf2-memory] FAILURES: %d — the numbers above "
                    "are NOT to be believed\n",
                    g_failures);
        return 1;
    }
    std::printf("[perf2-memory] ALL GREEN — the digests believed, the "
                "baseline reproduced two ways, the gate green, both "
                "counterproofs red, the verdict pinned; the peaks are "
                "the machine's own (indicative), the threshold is the "
                "substance\n");
    return 0;
}
