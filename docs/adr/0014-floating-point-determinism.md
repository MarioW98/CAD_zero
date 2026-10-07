# ADR-0014: Floating-point determinism across platforms

## Status
Accepted (2025-10-07)

## Context
The same CAD model rebuilt on Linux (GCC) and Windows (MSVC) must
produce byte-identical output files (STEP, 3MF, native format).
Without explicit policy, this fails for three reasons:

1. **FP contraction** (`a*b + c` computed in one FMA instruction)
   differs between compilers.
2. **Reduction order** in parallel sums differs across thread counts.
3. **Math library** differences (`libm` vs `ucrt`) can produce
   different `sin`/`cos` results by 1 ULP.

## Decision

### Compiler flags
| Compiler | Flag | Effect |
|----------|------|--------|
| GCC, Clang | `-ffp-contract=off` | Disables FMA contraction |
| MSVC | `/fp:strict` | Disables FMA contraction + exceptions |

Both are set in `CMakeLists.txt` regardless of build type.

### Reduction order
- Parallel sums in SDF evaluation use a fixed chunk-then-reduce
  pattern: each thread sums its chunk in increasing index order,
  and the partial sums are combined in worker-id order.
- The TBB `parallel_reduce` call must use `deterministic_reduce`
  (custom wrapper, added in Phase B) when writing output files.

### Math functions
- We use `std::sin`, `std::cos`, `std::sqrt` directly. These are
  the libm/ucrt implementations; they differ by ≤1 ULP across
  platforms. This is acceptable for *visualization* (ray marching,
  mesh extraction) but NOT for *output files* that must round-trip.
- For output-file serialization, results are quantized to 12 decimal
  digits (≈ 40 bits of mantissa, well below `double`'s 52 bits) before
  formatting. Two runs producing results that differ by ≤1 ULP thus
  produce the same serialized text.

### Determinism tests
- `tests/regression/fp_determinism/` contains models that are
  built, serialized to STEP, and hashed (SHA-256) in CI on Linux
  and Windows. The hashes must match.
- The CI matrix runs both platforms and the test reports a diff if
  hashes diverge.

## Consequences
- We accept ~5-10% performance overhead from disabling FMA on
  AMD Zen 4 / Intel Skylake+ (which benefit from FMA). This is
  recovered by xsimd where FMA is explicitly enabled for batched
  operations that do not write output.
- The native project format always serializes quantized numbers.
  Internal `double` values are not quantized.
- Visual ray marching is non-deterministic (allowed); only the
  feature tree's serialized output is held to the determinism bar.
