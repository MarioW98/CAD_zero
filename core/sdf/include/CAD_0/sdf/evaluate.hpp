// core/sdf/include/CAD_0/sdf/evaluate.hpp
//
// Bulk SDF evaluation backend. The SDFNode::sample() API evaluates a
// single point; in practice the mesh extractor and the ray marcher
// need to evaluate a *batch* of points (often millions per frame).
//
// We expose an abstract EvalBackend with two implementations:
//   * CpuEvalBackend  — scalar + xsimd-vectorized (default)
//   * GpuEvalBackend  — stub (post-MVP, see ADR-0008)
//
// The dispatch happens at runtime so the Python API can switch backends
// without rebuilding the model.
//
#pragma once

#include "CAD_0/sdf/field.hpp"

#include <span>
#include <vector>

namespace CAD_0::sdf {

// Output buffers for a batched evaluation.
struct EvalResult {
    std::vector<float> values;        // signed distances
    std::vector<math::Vec3f> gradients; // gradients (∇f)
};

// Abstract evaluation backend.
class EvalBackend {
public:
    virtual ~EvalBackend() = default;

    // Evaluate `body` at all points in `points` and write to `out`.
    // The implementation is responsible for parallelism (CPU SIMD, TBB,
    // or GPU compute).
    virtual void evaluate(const SDFBody& body,
                          std::span<const math::Vec3f> points,
                          EvalResult& out) = 0;

    virtual const char* name() const noexcept = 0;
};

// Default CPU backend — scalar + xsimd. Auto-vectorized via xsimd::batch.
class CpuEvalBackend final : public EvalBackend {
public:
    explicit CpuEvalBackend(unsigned num_threads = 0);

    void evaluate(const SDFBody& body,
                  std::span<const math::Vec3f> points,
                  EvalResult& out) override;

    const char* name() const noexcept override { return "cpu"; }

private:
    unsigned num_threads_;
};

// GPU backend — stub. Implementations deferred to post-MVP. The interface
// exists in Phase A so that the rest of the system can be written against
// a stable abstraction (see ADR-0008).
//
// All methods are declared here and *defined* in evaluate.cpp. This is
// intentional: defining methods inline in the header can cause
// "multiple definition" link errors when the header is included from
// multiple TUs that are linked together, especially in TBB/GPU builds.
// Keeping the definitions out-of-line guarantees a single symbol.
class GpuEvalBackend final : public EvalBackend {
public:
    GpuEvalBackend();
    void evaluate(const SDFBody& body,
                  std::span<const math::Vec3f> points,
                  EvalResult& out) override;
    // Defined in evaluate.cpp. Returns "gpu-stub (cpu-fallback)" so
    // callers can see at runtime that the GPU backend is not actually
    // executing on the GPU — no silent fallback.
    const char* name() const noexcept override;
};

} // namespace CAD_0::sdf
