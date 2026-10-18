// tests/property/property_prng.hpp
//
// Seeded, deterministic PRNG for the property-based test families (the
// mini-traccia PARTE 1 infrastructure — test-only, zero kernel changes,
// zero new dependencies).
//
// Contract:
//   * std::mt19937 is bit-exactly specified by the C++ standard (fixed
//     engine parameters), so the raw engine output is portable. The
//     std:: distributions are NOT portable (implementation-defined) —
//     every mapping here is hand-rolled from the raw 32-bit output
//     instead, so one seed reproduces one family bit-a-bit on any
//     conforming platform and compiler.
//   * The seed is PART OF THE TEST: never the clock, never entropy.
//     Every property test CAPTUREs (seed, iteration) plus the full
//     generated parameters, and INFOs the name of the violated check —
//     a failure is a reproducible diagnosis, not a flake.
//   * int_below() uses a plain modulo: for the small counts used by
//     the generators (n <= 16) the bias is negligible, and — more to
//     the point — it is deterministic. Reproducibility outranks
//     uniformity for this purpose.
#pragma once

#include <cstdint>
#include <random>

namespace CAD_0::test::property {

class Prng {
public:
    explicit Prng(std::uint32_t seed) : engine_(seed) {}

    // Raw engine output — the only std-specified quantity. The engine's
    // result_type is std::uint_fast32_t (64-bit here); the cast is exact
    // (mt19937 produces 32-bit values by specification).
    std::uint32_t next_u32() { return static_cast<std::uint32_t>(engine_()); }

    // Uniform in [0, 1) with 24-bit precision (top 24 bits of one draw).
    double unit() {
        return static_cast<double>(engine_() >> 8) * (1.0 / 16777216.0);
    }

    // Uniform double in [lo, hi).
    double range(double lo, double hi) { return lo + unit() * (hi - lo); }

    // Uniform float in [lo, hi) — the SDF field is float-valued, so the
    // generated parameters are float by construction.
    float frange(float lo, float hi) {
        return static_cast<float>(range(lo, hi));
    }

    // Uniform integer in [0, n) — small n only (see the modulo note in
    // the header comment). n == 0 returns 0 (defensive; generators
    // always pass n >= 1).
    std::uint32_t int_below(std::uint32_t n) {
        if (n == 0u) return 0u;
        return static_cast<std::uint32_t>(engine_() % n);
    }

    bool coin() { return (engine_() & 1u) != 0u; }

private:
    std::mt19937 engine_;
};

} // namespace CAD_0::test::property
