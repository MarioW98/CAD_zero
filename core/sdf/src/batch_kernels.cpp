// core/sdf/src/batch_kernels.cpp
//
// PERF 3 — THE BATCH KERNELS: the xsimd leaves and the composing
// interior, one file so the scalar form and its lane twin sit side by
// side for review (the house discipline: the proof is the diff).
//
// THE AUDIT THIS FILE CLOSES (the register's PERF 3 row): the tree
// declared an "xsimd-vectorized" evaluator since ADR-0008 (2025-10-07)
// while every point rode a virtual sample() call — zero SIMD
// instructions in the generated code, the include dead weight, the
// throughput of the scalar walk. The audit's evidence: source (no xsimd
// symbol referenced; the TU compiles with the include removed),
// assembly (the loop is `call *0x10(%rax)` + movss, one point per
// iteration, zero packed instructions in the whole TU), throughput
// (the scalar tree walk at ~56M pts/s against a bit-identical xsimd
// kernel at ~711M pts/s on the declared sandbox machine, W=4 SSE2).
// Esito (a): the vectorized path is made real and verified HERE.
//
// THE BIT-IDENTITY LAW (ADR-0014, extended to the batch road): every
// kernel must produce, for every point of every finite input, bits
// identical to its class's sample(). The transcription rules, each
// load-bearing and each verifiable in the diff below:
//   * the SAME expression tree per lane: the dot products keep the
//     scalar association ((x*x + y*y) + z*z), never reassociated;
//   * -ffp-contract=off (the project's own flag, CMakeLists.txt)
//     guarantees no FMA fusion difference between the forms;
//   * division stays division (Vec3f::normalized() divides by len),
//     reciprocal stays reciprocal (the operators' defensive normalize
//     multiplies by 1/len) — the two forms differ by 1 ULP and the
//     kernels keep each where the scalar form has it;
//   * min/max are transcribed as select(b>a, b, a) / select(b<a, b, a)
//     — EXACTLY std::max/std::min semantics per lane (leftmost wins
//     ties, NaN compares false, ±0 stays first) — NOT xsimd::min/max,
//     whose lane tie/NaN conventions differ;
//   * sqrt is the IEEE per-lane sqrt (xsimd::sqrt -> vsqrtps/sqrtps),
//     correctly rounded like the scalar sqrtss — identical bits;
//   * uniform quantities (radius broadcasts, the cone's L/cos/sin, the
//     slab's zc) are computed ONCE with the same expressions the
//     scalar form runs per point — same inputs, same ops, same bits;
//   * the scalar tail (n % width points) calls the class's OWN
//     sample() — devirtualized (final), the same function, the same
//     bits by construction;
//   * the interior (operators, rigid transforms) composes its
//     children's batches with the SAME per-point expressions its
//     sample() uses, in scalar form — bit-identity by construction,
//     no transcription at all.
//
// THE DECLARED SEATS that stay scalar (said, not hidden): TwistSDF
// (libm cos/sin per point is the bit-honest road — ADR-0014's math
// library clause; the witness counts every twist point scalar) and
// ExtrudeSDF (the 2D profile walk is its own arithmetic family).
// The dispatch witness (detail/batch_stats.hpp) counts every leaf
// point by its path; the suite pins the exact split per corpus.
//
// THE SCRATCH POOL: the interior's recursion needs per-level buffers
// (a child's batch output, a transform's mapped points). A
// thread_local pool, one level per recursion depth, RAII-claimed —
// no per-call allocation on the steady path, no cross-thread sharing
// under TBB. Raw pointers into a level's heap buffers are held across
// child calls (the buffers do not move; the level objects might, so
// references are never held across a child call).
#include "CAD_0/sdf/primitives.hpp"
#include "CAD_0/sdf/operators.hpp"
#include "CAD_0/sdf/transforms.hpp"
#include "CAD_0/sdf/sketch_extrude.hpp"
#include "CAD_0/sdf/detail/batch_stats.hpp"

#include <xsimd/xsimd.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace CAD_0::sdf {

namespace {

using B = xsimd::batch<float>;
constexpr std::size_t kW = B::size;

// std::max/std::min, per lane, exactly: leftmost wins ties (the select
// picks b only on the STRICT comparison), NaN compares false, ±0 stays
// first. NOT xsimd::min/max — their tie/NaN lane conventions differ.
inline B lane_max(const B& a, const B& b) {
    return xsimd::select(b > a, b, a);
}
inline B lane_min(const B& a, const B& b) {
    return xsimd::select(b < a, b, a);
}

// The witness counters, one fetch_add per chunk.
inline void count_vector(std::size_t points) {
    detail::batch_witness().vector_points.fetch_add(points,
                                                    std::memory_order_relaxed);
}
inline void count_scalar(std::size_t points) {
    detail::batch_witness().scalar_points.fetch_add(points,
                                                    std::memory_order_relaxed);
}

// ---- The thread_local scratch pool (see the file header) ---------------
struct BatchScratch {
    std::vector<float> values;
    std::vector<math::Vec3f> gradients;
    std::vector<math::Vec3f> points;
};

class BatchScratchPool {
public:
    // Claims the next level. The returned reference is valid until the
    // NEXT level() call on the same thread — callers copy out raw
    // pointers before recursing (level objects move when the pool
    // vector grows; their heap buffers do not).
    BatchScratch& level() {
        if (levels_.size() <= depth_) levels_.resize(depth_ + 1);
        return levels_[depth_++];
    }
    void release() { --depth_; }

private:
    std::vector<BatchScratch> levels_;
    std::size_t depth_ = 0;
};

inline BatchScratchPool& scratch_pool() {
    static thread_local BatchScratchPool pool;
    return pool;
}

struct ScratchClaim {
    BatchScratchPool& pool_;
    explicit ScratchClaim(BatchScratchPool& p) : pool_(p) {}
    ~ScratchClaim() { pool_.release(); }
};

// The interior's defensive gradient normalize, the operators' own form
// (reciprocal-multiply — see the transcription rules): gl > 1e-9 ?
// g * (1/gl) : g, with gl = sqrt(((gx*gx + gy*gy) + gz*gz)).
inline math::Vec3f defensive_normalize(const math::Vec3f& g) {
    const float gl = g.length();
    if (gl > 1e-9f) return g * (1.0f / gl);
    return g;
}

} // namespace

// ===========================================================================
// SphereSDF — the scalar form, then its lane twin
// ===========================================================================

SDFSample SphereSDF::sample(const math::Vec3f& p) const noexcept {
    const math::Vec3f g = p / radius_;
    const float d = p.length() - radius_;
    return SDFSample{d, g.normalized()};
}

void SphereSDF::sample_batch(std::span<const math::Vec3f> points,
                             float* values,
                             math::Vec3f* gradients) const noexcept {
    const std::size_t n = points.size();
    const std::size_t nv = n - (n % kW);
    const B br(radius_);
    const B beps(std::numeric_limits<float>::epsilon());
    for (std::size_t i = 0; i < nv; i += kW) {
        float xb[kW], yb[kW], zb[kW];
        for (std::size_t k = 0; k < kW; ++k) {
            xb[k] = points[i + k].x;
            yb[k] = points[i + k].y;
            zb[k] = points[i + k].z;
        }
        const B px = B::load_unaligned(xb);
        const B py = B::load_unaligned(yb);
        const B pz = B::load_unaligned(zb);
        // g = p / radius_ (component division, as Vec3f::operator/)
        const B gx = px / br;
        const B gy = py / br;
        const B gz = pz / br;
        // d = sqrt(((x*x + y*y) + z*z)) - r  — the dot's own association
        const B d = xsimd::sqrt((px * px + py * py) + pz * pz) - br;
        // g.normalized(): len > eps ? g / len : g (DIVISION, as written)
        const B glen = xsimd::sqrt((gx * gx + gy * gy) + gz * gz);
        const auto m = glen > beps;
        const B nx = xsimd::select(m, gx / glen, gx);
        const B ny = xsimd::select(m, gy / glen, gy);
        const B nz = xsimd::select(m, gz / glen, gz);
        float db[kW], nxb[kW], nyb[kW], nzb[kW];
        xsimd::store_unaligned(db, d);
        xsimd::store_unaligned(nxb, nx);
        xsimd::store_unaligned(nyb, ny);
        xsimd::store_unaligned(nzb, nz);
        for (std::size_t k = 0; k < kW; ++k) {
            values[i + k] = db[k];
            gradients[i + k] = math::Vec3f{nxb[k], nyb[k], nzb[k]};
        }
    }
    for (std::size_t i = nv; i < n; ++i) {
        const SDFSample s = sample(points[i]);
        values[i] = s.value;
        gradients[i] = s.gradient;
    }
    count_vector(nv);
    count_scalar(n - nv);
}

// ===========================================================================
// BoxSDF
// ===========================================================================

SDFSample BoxSDF::sample(const math::Vec3f& p) const noexcept {
    const math::Vec3f q{std::abs(p.x) - extent_.x,
                        std::abs(p.y) - extent_.y,
                        std::abs(p.z) - extent_.z};
    const math::Vec3f outer{std::max(q.x, 0.0f),
                            std::max(q.y, 0.0f),
                            std::max(q.z, 0.0f)};
    const float d = outer.length() + std::min(std::max({q.x, q.y, q.z}), 0.0f);
    // Gradient = sign(p) * sign(qmax)
    math::Vec3f g{
        (q.x > q.y && q.x > q.z) ? (p.x > 0 ? 1.0f : -1.0f) : 0.0f,
        (q.y > q.x && q.y > q.z) ? (p.y > 0 ? 1.0f : -1.0f) : 0.0f,
        (q.z > q.x && q.z > q.y) ? (p.z > 0 ? 1.0f : -1.0f) : 0.0f,
    };
    return SDFSample{d, g};
}

void BoxSDF::sample_batch(std::span<const math::Vec3f> points,
                          float* values,
                          math::Vec3f* gradients) const noexcept {
    const std::size_t n = points.size();
    const std::size_t nv = n - (n % kW);
    const B bex(extent_.x), bey(extent_.y), bez(extent_.z);
    const B bzero(0.0f), bone(1.0f), bmone(-1.0f);
    for (std::size_t i = 0; i < nv; i += kW) {
        float xb[kW], yb[kW], zb[kW];
        for (std::size_t k = 0; k < kW; ++k) {
            xb[k] = points[i + k].x;
            yb[k] = points[i + k].y;
            zb[k] = points[i + k].z;
        }
        const B px = B::load_unaligned(xb);
        const B py = B::load_unaligned(yb);
        const B pz = B::load_unaligned(zb);
        const B qx = xsimd::abs(px) - bex;
        const B qy = xsimd::abs(py) - bey;
        const B qz = xsimd::abs(pz) - bez;
        // outer = max(q, 0) per component — lane_max == std::max
        const B ox = lane_max(qx, bzero);
        const B oy = lane_max(qy, bzero);
        const B oz = lane_max(qz, bzero);
        // max({qx,qy,qz}) — the leftmost-wins fold, as std::max's il form
        const B m_ab = lane_max(qx, qy);
        const B m3 = lane_max(m_ab, qz);
        const B d = xsimd::sqrt((ox * ox + oy * oy) + oz * oz)
                  + lane_min(m3, bzero);
        // Gradient: sign(p) * sign(qmax), the three strict conditions
        const B gx = xsimd::select((qx > qy) & (qx > qz),
                                   xsimd::select(px > bzero, bone, bmone),
                                   bzero);
        const B gy = xsimd::select((qy > qx) & (qy > qz),
                                   xsimd::select(py > bzero, bone, bmone),
                                   bzero);
        const B gz = xsimd::select((qz > qx) & (qz > qy),
                                   xsimd::select(pz > bzero, bone, bmone),
                                   bzero);
        float db[kW], gxb[kW], gyb[kW], gzb[kW];
        xsimd::store_unaligned(db, d);
        xsimd::store_unaligned(gxb, gx);
        xsimd::store_unaligned(gyb, gy);
        xsimd::store_unaligned(gzb, gz);
        for (std::size_t k = 0; k < kW; ++k) {
            values[i + k] = db[k];
            gradients[i + k] = math::Vec3f{gxb[k], gyb[k], gzb[k]};
        }
    }
    for (std::size_t i = nv; i < n; ++i) {
        const SDFSample s = sample(points[i]);
        values[i] = s.value;
        gradients[i] = s.gradient;
    }
    count_vector(nv);
    count_scalar(n - nv);
}

// ===========================================================================
// CylinderSDF
// ===========================================================================

SDFSample CylinderSDF::sample(const math::Vec3f& p) const noexcept {
    // 2D cross-section: d = (rho - r, |y| - half_h)
    const float rho = std::sqrt(p.x * p.x + p.z * p.z);
    const float dx = rho - radius_;
    const float dy = std::abs(p.y) - half_height_;

    // Outside distance (positive): only the components that exceed 0.
    const float ox = std::max(dx, 0.0f);
    const float oy = std::max(dy, 0.0f);
    const float d_out = std::sqrt(ox * ox + oy * oy);

    // Inside distance (negative): max of the (negative) signed
    // distances. If we're outside on both axes this is ≤ 0 so the
    // min(d_out, ...) branch picks d_out.
    const float d_in = std::min(std::max(dx, dy), 0.0f);

    const float dist = d_out + d_in;

    // ----- Gradient (normalized) -----
    // The closest surface element is determined by which of dx, dy
    // is the *positive* maximum (i.e. which one we are "exiting"
    // through). If both are negative (inside) we pick the one
    // closest to zero (i.e. the max).
    math::Vec3f g{};
    if (dy >= dx) {
        // Closest to a cap. Determine which one by the sign of p.y.
        g = math::Vec3f{0.0f, (p.y >= 0.0f) ? 1.0f : -1.0f, 0.0f};
    } else {
        // Closest to the lateral surface. Outward normal in 3D is
        // the radial unit vector (x, 0, z)/rho.
        if (rho > 1e-9f) {
            const float inv_rho = 1.0f / rho;
            g = math::Vec3f{p.x * inv_rho, 0.0f, p.z * inv_rho};
        } else {
            // On the axis: the lateral surface can't be the closest
            // (we're inside the cylinder), so this branch shouldn't
            // trigger. Fall back to +Y as a safe default.
            g = math::Vec3f{0.0f, 1.0f, 0.0f};
        }
    }
    return SDFSample{dist, g};
}

void CylinderSDF::sample_batch(std::span<const math::Vec3f> points,
                               float* values,
                               math::Vec3f* gradients) const noexcept {
    const std::size_t n = points.size();
    const std::size_t nv = n - (n % kW);
    const B br(radius_), bhh(half_height_);
    const B bzero(0.0f), bone(1.0f), bmone(-1.0f), beps(1e-9f);
    for (std::size_t i = 0; i < nv; i += kW) {
        float xb[kW], yb[kW], zb[kW];
        for (std::size_t k = 0; k < kW; ++k) {
            xb[k] = points[i + k].x;
            yb[k] = points[i + k].y;
            zb[k] = points[i + k].z;
        }
        const B px = B::load_unaligned(xb);
        const B py = B::load_unaligned(yb);
        const B pz = B::load_unaligned(zb);
        const B rho = xsimd::sqrt(px * px + pz * pz);
        const B dx = rho - br;
        const B dy = xsimd::abs(py) - bhh;
        const B ox = lane_max(dx, bzero);
        const B oy = lane_max(dy, bzero);
        const B d_out = xsimd::sqrt(ox * ox + oy * oy);
        const B d_in = lane_min(lane_max(dx, dy), bzero);
        const B dist = d_out + d_in;
        // Gradient: the same three-branch select as sample()'s ifs.
        // inv_rho computed unconditionally (1/0 = inf, discarded by the
        // select — the picked lane never reads it).
        const B inv_rho = B(1.0f) / rho;
        const B cap_y = xsimd::select(py >= bzero, bone, bmone);
        const B rad_x = px * inv_rho;
        const B rad_z = pz * inv_rho;
        const auto lat = rho > beps;
        const B lat_x = xsimd::select(lat, rad_x, bzero);
        const B lat_y = xsimd::select(lat, bzero, bone);
        const B lat_z = xsimd::select(lat, rad_z, bzero);
        const auto cap = dy >= dx;
        const B gx = xsimd::select(cap, bzero, lat_x);
        const B gy = xsimd::select(cap, cap_y, lat_y);
        const B gz = xsimd::select(cap, bzero, lat_z);
        float db[kW], gxb[kW], gyb[kW], gzb[kW];
        xsimd::store_unaligned(db, dist);
        xsimd::store_unaligned(gxb, gx);
        xsimd::store_unaligned(gyb, gy);
        xsimd::store_unaligned(gzb, gz);
        for (std::size_t k = 0; k < kW; ++k) {
            values[i + k] = db[k];
            gradients[i + k] = math::Vec3f{gxb[k], gyb[k], gzb[k]};
        }
    }
    for (std::size_t i = nv; i < n; ++i) {
        const SDFSample s = sample(points[i]);
        values[i] = s.value;
        gradients[i] = s.gradient;
    }
    count_vector(nv);
    count_scalar(n - nv);
}

// ===========================================================================
// TorusSDF
// ===========================================================================

SDFSample TorusSDF::sample(const math::Vec3f& p) const noexcept {
    const math::Vec2f q{std::sqrt(p.x * p.x + p.z * p.z) - R_, p.y};
    const float d = q.length() - r_;
    math::Vec2f g = q;
    const float ql = q.length();
    if (ql > 1e-9f) g = g * (1.0f / ql);
    math::Vec3f g3{p.x, 0.0f, p.z};
    const float g3l = g3.length();
    if (g3l > 1e-9f) g3 = g3 * (1.0f / g3l);
    return SDFSample{d, math::Vec3f{g3.x * g.x, g.y, g3.z * g.x}};
}

void TorusSDF::sample_batch(std::span<const math::Vec3f> points,
                            float* values,
                            math::Vec3f* gradients) const noexcept {
    const std::size_t n = points.size();
    const std::size_t nv = n - (n % kW);
    const B bR(R_), br(r_);
    const B bzero(0.0f), bone(1.0f), beps(1e-9f);
    for (std::size_t i = 0; i < nv; i += kW) {
        float xb[kW], yb[kW], zb[kW];
        for (std::size_t k = 0; k < kW; ++k) {
            xb[k] = points[i + k].x;
            yb[k] = points[i + k].y;
            zb[k] = points[i + k].z;
        }
        const B px = B::load_unaligned(xb);
        const B py = B::load_unaligned(yb);
        const B pz = B::load_unaligned(zb);
        // q = (rho - R, y); both normalizes use the RECIPROCAL form
        // (the scalar's own), threshold 1e-9 — not normalized()'s eps.
        const B qx = xsimd::sqrt(px * px + pz * pz) - bR;
        const B qy = py;
        const B d = xsimd::sqrt(qx * qx + qy * qy) - br;
        const B ql = xsimd::sqrt(qx * qx + qy * qy);
        const B inv_ql = B(1.0f) / ql;
        const auto mq = ql > beps;
        const B g2x = xsimd::select(mq, qx * inv_ql, qx);
        const B g2y = xsimd::select(mq, qy * inv_ql, qy);
        const B g3l = xsimd::sqrt((px * px + bzero * bzero) + pz * pz);
        const B inv_g3 = B(1.0f) / g3l;
        const auto m3 = g3l > beps;
        const B n3x = xsimd::select(m3, px * inv_g3, px);
        const B n3z = xsimd::select(m3, pz * inv_g3, pz);
        const B gx = n3x * g2x;
        const B gy = g2y;
        const B gz = n3z * g2x;
        float db[kW], gxb[kW], gyb[kW], gzb[kW];
        xsimd::store_unaligned(db, d);
        xsimd::store_unaligned(gxb, gx);
        xsimd::store_unaligned(gyb, gy);
        xsimd::store_unaligned(gzb, gz);
        for (std::size_t k = 0; k < kW; ++k) {
            values[i + k] = db[k];
            gradients[i + k] = math::Vec3f{gxb[k], gyb[k], gzb[k]};
        }
    }
    for (std::size_t i = nv; i < n; ++i) {
        const SDFSample s = sample(points[i]);
        values[i] = s.value;
        gradients[i] = s.gradient;
    }
    count_vector(nv);
    count_scalar(n - nv);
}

// ===========================================================================
// ConeSDF
// ===========================================================================

SDFSample ConeSDF::sample(const math::Vec3f& p) const noexcept {
    const float rho = std::sqrt(p.x * p.x + p.z * p.z);
    const float y = p.y;

    // Outward normal of the lateral line: (h, r) / L.
    const float L = std::sqrt(r_ * r_ + h_ * h_);
    const float cos_theta = (L > 1e-9f) ? h_ / L : 1.0f;
    const float sin_theta = (L > 1e-9f) ? r_ / L : 0.0f;

    // Signed distance to the infinite lateral line.
    // pa = (rho - r, y), outward normal = (cos_theta, sin_theta).
    // ww = dot(pa, outward_normal).
    const float ww = (rho - r_) * cos_theta + y * sin_theta;

    // d = max(ww, -y).
    const float d_base = -y;
    const float d = std::max(ww, d_base);

    // Gradient.
    math::Vec3f g{};
    if (d_base >= ww) {
        g = math::Vec3f{0.0f, -1.0f, 0.0f};
    } else if (rho > 1e-9f) {
        const float inv_rho = 1.0f / rho;
        g = math::Vec3f{cos_theta * p.x * inv_rho,
                        sin_theta,
                        cos_theta * p.z * inv_rho};
    } else {
        g = math::Vec3f{0.0f, sin_theta, 0.0f};
    }
    return SDFSample{d, g};
}

void ConeSDF::sample_batch(std::span<const math::Vec3f> points,
                           float* values,
                           math::Vec3f* gradients) const noexcept {
    const std::size_t n = points.size();
    const std::size_t nv = n - (n % kW);
    // The uniform quantities, computed ONCE with the scalar form's own
    // expressions (same inputs, same ops, same bits as per point).
    const float L = std::sqrt(r_ * r_ + h_ * h_);
    const float cos_theta = (L > 1e-9f) ? h_ / L : 1.0f;
    const float sin_theta = (L > 1e-9f) ? r_ / L : 0.0f;
    const B bct(cos_theta), bst(sin_theta), br(r_);
    const B bzero(0.0f), bmone(-1.0f), beps(1e-9f);
    for (std::size_t i = 0; i < nv; i += kW) {
        float xb[kW], yb[kW], zb[kW];
        for (std::size_t k = 0; k < kW; ++k) {
            xb[k] = points[i + k].x;
            yb[k] = points[i + k].y;
            zb[k] = points[i + k].z;
        }
        const B px = B::load_unaligned(xb);
        const B py = B::load_unaligned(yb);
        const B pz = B::load_unaligned(zb);
        const B rho = xsimd::sqrt(px * px + pz * pz);
        const B ww = (rho - br) * bct + py * bst;
        const B d_base = -py;
        const B d = lane_max(ww, d_base);
        // Gradient: the same three-branch select.
        const B inv_rho = B(1.0f) / rho;
        const B lat_x = bct * px * inv_rho;
        const B lat_z = bct * pz * inv_rho;
        const auto lat = rho > beps;
        const B lx = xsimd::select(lat, lat_x, bzero);
        const B ly = xsimd::select(lat, bst, bst);
        const B lz = xsimd::select(lat, lat_z, bzero);
        const auto base = d_base >= ww;
        const B gx = xsimd::select(base, bzero, lx);
        const B gy = xsimd::select(base, bmone, ly);
        const B gz = xsimd::select(base, bzero, lz);
        float db[kW], gxb[kW], gyb[kW], gzb[kW];
        xsimd::store_unaligned(db, d);
        xsimd::store_unaligned(gxb, gx);
        xsimd::store_unaligned(gyb, gy);
        xsimd::store_unaligned(gzb, gz);
        for (std::size_t k = 0; k < kW; ++k) {
            values[i + k] = db[k];
            gradients[i + k] = math::Vec3f{gxb[k], gyb[k], gzb[k]};
        }
    }
    for (std::size_t i = nv; i < n; ++i) {
        const SDFSample s = sample(points[i]);
        values[i] = s.value;
        gradients[i] = s.gradient;
    }
    count_vector(nv);
    count_scalar(n - nv);
}

// ===========================================================================
// CapsuleSDF
// ===========================================================================

SDFSample CapsuleSDF::sample(const math::Vec3f& p) const noexcept {
    math::Vec3f pa = p - a_;
    const math::Vec3f ba = b_ - a_;
    float t = pa.dot(ba) / std::max(length_sq_, 1e-12f);
    t = std::clamp(t, 0.0f, 1.0f);
    const math::Vec3f closest = a_ + ba * t;
    const math::Vec3f diff = p - closest;
    const float dist = diff.length() - r_;
    return SDFSample{dist, diff.normalized()};
}

void CapsuleSDF::sample_batch(std::span<const math::Vec3f> points,
                              float* values,
                              math::Vec3f* gradients) const noexcept {
    const std::size_t n = points.size();
    const std::size_t nv = n - (n % kW);
    // Uniform quantities, the scalar form's own expressions.
    const math::Vec3f ba = b_ - a_;
    const float denom = std::max(length_sq_, 1e-12f);
    const B bax(ba.x), bay(ba.y), baz(ba.z);
    const B bax_(a_.x), bay_(a_.y), baz_(a_.z);
    const B bden(denom), br(r_);
    const B bzero(0.0f), bone(1.0f);
    const B beps(std::numeric_limits<float>::epsilon());
    for (std::size_t i = 0; i < nv; i += kW) {
        float xb[kW], yb[kW], zb[kW];
        for (std::size_t k = 0; k < kW; ++k) {
            xb[k] = points[i + k].x;
            yb[k] = points[i + k].y;
            zb[k] = points[i + k].z;
        }
        const B px = B::load_unaligned(xb);
        const B py = B::load_unaligned(yb);
        const B pz = B::load_unaligned(zb);
        const B pax = px - bax_;
        const B pay = py - bay_;
        const B paz = pz - baz_;
        // pa.dot(ba) — the dot's own association, then / denom
        const B t = (pax * bax + pay * bay) + paz * baz;
        B tc = t / bden;
        // std::clamp(t, 0, 1) = min(max(t, 0), 1) — the lane forms
        tc = lane_min(lane_max(tc, bzero), bone);
        const B cx = bax_ + bax * tc;
        const B cy = bay_ + bay * tc;
        const B cz = baz_ + baz * tc;
        const B dx = px - cx;
        const B dy = py - cy;
        const B dz = pz - cz;
        const B dist = xsimd::sqrt((dx * dx + dy * dy) + dz * dz) - br;
        // diff.normalized(): len > eps ? diff / len : diff (DIVISION)
        const B dl = xsimd::sqrt((dx * dx + dy * dy) + dz * dz);
        const auto m = dl > beps;
        const B gx = xsimd::select(m, dx / dl, dx);
        const B gy = xsimd::select(m, dy / dl, dy);
        const B gz = xsimd::select(m, dz / dl, dz);
        float db[kW], gxb[kW], gyb[kW], gzb[kW];
        xsimd::store_unaligned(db, dist);
        xsimd::store_unaligned(gxb, gx);
        xsimd::store_unaligned(gyb, gy);
        xsimd::store_unaligned(gzb, gz);
        for (std::size_t k = 0; k < kW; ++k) {
            values[i + k] = db[k];
            gradients[i + k] = math::Vec3f{gxb[k], gyb[k], gzb[k]};
        }
    }
    for (std::size_t i = nv; i < n; ++i) {
        const SDFSample s = sample(points[i]);
        values[i] = s.value;
        gradients[i] = s.gradient;
    }
    count_vector(nv);
    count_scalar(n - nv);
}

// ===========================================================================
// PlaneSDF
// ===========================================================================

SDFSample PlaneSDF::sample(const math::Vec3f& p) const noexcept {
    // Inside (y > 0) → negative distance (so negate y).
    // Outside (y < 0) → positive distance.
    return SDFSample{-p.y, math::Vec3f{0.0f, -1.0f, 0.0f}};
}

void PlaneSDF::sample_batch(std::span<const math::Vec3f> points,
                            float* values,
                            math::Vec3f* gradients) const noexcept {
    const std::size_t n = points.size();
    const std::size_t nv = n - (n % kW);
    const B bzero(0.0f), bmone(-1.0f);
    for (std::size_t i = 0; i < nv; i += kW) {
        float yb[kW];
        for (std::size_t k = 0; k < kW; ++k) yb[k] = points[i + k].y;
        const B py = B::load_unaligned(yb);
        const B d = -py;
        float db[kW];
        xsimd::store_unaligned(db, d);
        for (std::size_t k = 0; k < kW; ++k) {
            values[i + k] = db[k];
            gradients[i + k] = math::Vec3f{0.0f, -1.0f, 0.0f};
        }
    }
    for (std::size_t i = nv; i < n; ++i) {
        const SDFSample s = sample(points[i]);
        values[i] = s.value;
        gradients[i] = s.gradient;
    }
    count_vector(nv);
    count_scalar(n - nv);
}

// ===========================================================================
// SlabZ (sketch_extrude.hpp) — the eighth leaf kernel
// ===========================================================================

SDFSample SlabZ::sample(const math::Vec3f& p) const noexcept {
    const float zc = center_z();
    const float f = std::abs(p.z - zc) - distance_ * 0.5f;
    // Gradient: (0, 0, sign(z − zc)) — at the mirror plane the +Z
    // branch wins by convention (the field there is at its deepest,
    // −d/2: no surface consumer asks for the normal there).
    const float gz = (p.z >= zc) ? 1.0f : -1.0f;
    return SDFSample{f, math::Vec3f{0.0f, 0.0f, gz}};
}

void SlabZ::sample_batch(std::span<const math::Vec3f> points,
                         float* values,
                         math::Vec3f* gradients) const noexcept {
    const std::size_t n = points.size();
    const std::size_t nv = n - (n % kW);
    // Uniform: same expressions the scalar form runs per point.
    const float zc = center_z();
    const float half = distance_ * 0.5f;
    const B bzc(zc), bhalf(half);
    const B bone(1.0f), bmone(-1.0f);
    for (std::size_t i = 0; i < nv; i += kW) {
        float zb[kW];
        for (std::size_t k = 0; k < kW; ++k) zb[k] = points[i + k].z;
        const B pz = B::load_unaligned(zb);
        const B f = xsimd::abs(pz - bzc) - bhalf;
        const B gz = xsimd::select(pz >= bzc, bone, bmone);
        float db[kW], gzb[kW];
        xsimd::store_unaligned(db, f);
        xsimd::store_unaligned(gzb, gz);
        for (std::size_t k = 0; k < kW; ++k) {
            values[i + k] = db[k];
            gradients[i + k] = math::Vec3f{0.0f, 0.0f, gzb[k]};
        }
    }
    for (std::size_t i = nv; i < n; ++i) {
        const SDFSample s = sample(points[i]);
        values[i] = s.value;
        gradients[i] = s.gradient;
    }
    count_vector(nv);
    count_scalar(n - nv);
}

// ===========================================================================
// UnionSDF — the interior begins: children's batches composed with the
// SAME per-point expressions sample() uses (scalar form, bit-identity
// by construction)
// ===========================================================================

SDFSample UnionSDF::sample(const math::Vec3f& p) const noexcept {
    if (children_.empty()) return SDFSample{1e30f, {}};
    auto r = children_[0]->sample(p);
    for (std::size_t i = 1; i < children_.size(); ++i) {
        const auto s = children_[i]->sample(p);
        if (s.value < r.value) r = s;
    }
    // Normalize the picked gradient (defensive — children of
    // composites may not have unit gradient).
    math::Vec3f g = r.gradient;
    const float gl = g.length();
    if (gl > 1e-9f) g = g * (1.0f / gl);
    r.gradient = g;
    return r;
}

void UnionSDF::sample_batch(std::span<const math::Vec3f> points,
                            float* values,
                            math::Vec3f* gradients) const noexcept {
    const std::size_t n = points.size();
    if (children_.empty()) {
        // The defensive branch, mirrored from sample().
        for (std::size_t i = 0; i < n; ++i) {
            values[i] = 1e30f;
            gradients[i] = math::Vec3f{};
        }
        return;
    }
    children_[0]->sample_batch(points, values, gradients);
    if (children_.size() > 1) {
        auto& pool = scratch_pool();
        BatchScratch& s = pool.level();
        ScratchClaim claim(pool);
        s.values.resize(n);
        s.gradients.resize(n);
        float* const sv = s.values.data();
        math::Vec3f* const sg = s.gradients.data();
        for (std::size_t c = 1; c < children_.size(); ++c) {
            children_[c]->sample_batch(points, sv, sg);
            for (std::size_t i = 0; i < n; ++i) {
                if (sv[i] < values[i]) {
                    values[i] = sv[i];
                    gradients[i] = sg[i];
                }
            }
        }
    }
    // The defensive normalize, the scalar form's own reciprocal form,
    // applied to the picked gradient of every point.
    for (std::size_t i = 0; i < n; ++i)
        gradients[i] = defensive_normalize(gradients[i]);
}

// ===========================================================================
// IntersectionSDF
// ===========================================================================

SDFSample IntersectionSDF::sample(const math::Vec3f& p) const noexcept {
    auto sa = a_->sample(p);
    const auto sb = b_->sample(p);
    // Max of the two signed distances.
    if (sb.value > sa.value) {
        sa = sb;
    }
    // Normalize the gradient we picked (defensive — children of
    // composites may not have unit gradient).
    math::Vec3f g = sa.gradient;
    const float gl = g.length();
    if (gl > 1e-9f) g = g * (1.0f / gl);
    sa.gradient = g;
    return sa;
}

void IntersectionSDF::sample_batch(std::span<const math::Vec3f> points,
                                   float* values,
                                   math::Vec3f* gradients) const noexcept {
    const std::size_t n = points.size();
    a_->sample_batch(points, values, gradients);
    auto& pool = scratch_pool();
    BatchScratch& s = pool.level();
    ScratchClaim claim(pool);
    s.values.resize(n);
    s.gradients.resize(n);
    float* const sv = s.values.data();
    math::Vec3f* const sg = s.gradients.data();
    b_->sample_batch(points, sv, sg);
    for (std::size_t i = 0; i < n; ++i) {
        float v = values[i];
        math::Vec3f g = gradients[i];
        if (sv[i] > v) {
            v = sv[i];
            g = sg[i];
        }
        values[i] = v;
        gradients[i] = defensive_normalize(g);
    }
}

// ===========================================================================
// SubtractionSDF
// ===========================================================================

SDFSample SubtractionSDF::sample(const math::Vec3f& p) const noexcept {
    auto sa = a_->sample(p);
    const auto sb = b_->sample(p);
    // Inside b => outside the result, so negate b's distance and take max.
    const float bneg = -sb.value;
    if (bneg > sa.value) {
        sa.value = bneg;
        // Result inherits -∇b as its gradient. Normalize it to keep
        // |∇f| = 1 (assuming b is a proper SDF with |∇b| = 1 on the
        // surface; this also clamps any slight over-unit magnitude
        // from composited children).
        math::Vec3f g = -sb.gradient;
        const float gl = g.length();
        if (gl > 1e-9f) g = g * (1.0f / gl);
        sa.gradient = g;
    } else {
        // Result inherits ∇a. Normalize for the same reason.
        math::Vec3f g = sa.gradient;
        const float gl = g.length();
        if (gl > 1e-9f) g = g * (1.0f / gl);
        sa.gradient = g;
    }
    return sa;
}

void SubtractionSDF::sample_batch(std::span<const math::Vec3f> points,
                                  float* values,
                                  math::Vec3f* gradients) const noexcept {
    const std::size_t n = points.size();
    a_->sample_batch(points, values, gradients);
    auto& pool = scratch_pool();
    BatchScratch& s = pool.level();
    ScratchClaim claim(pool);
    s.values.resize(n);
    s.gradients.resize(n);
    float* const sv = s.values.data();
    math::Vec3f* const sg = s.gradients.data();
    b_->sample_batch(points, sv, sg);
    for (std::size_t i = 0; i < n; ++i) {
        const float bneg = -sv[i];
        if (bneg > values[i]) {
            values[i] = bneg;
            gradients[i] = defensive_normalize(-sg[i]);
        } else {
            gradients[i] = defensive_normalize(gradients[i]);
        }
    }
}

// ===========================================================================
// SmoothMinSDF
// ===========================================================================

SDFSample SmoothMinSDF::sample(const math::Vec3f& p) const noexcept {
    const auto sa = a_->sample(p);
    const auto sb = b_->sample(p);
    if (k_ <= 0.0f) {
        // k=0 → hard min; pick the closer child and normalize its
        // gradient defensively (the child might be a composite
        // whose gradient magnitude slightly exceeds 1).
        auto r = (sb.value < sa.value) ? sb : sa;
        const float gl = r.gradient.length();
        if (gl > 1e-9f) r.gradient = r.gradient * (1.0f / gl);
        return r;
    }
    const float diff = sb.value - sa.value;
    // h = 1 when b ≤ a - k (v follows b, the SMALLER child — see
    // the bug-fix note in the class comment); the sign of the
    // clamp argument is what selects min vs max semantics.
    const float h = std::clamp(0.5f - 0.5f * diff / k_, 0.0f, 1.0f);
    const float v = sa.value + diff * h - k_ * h * (1.0f - h) * 0.25f;
    // Full chain-rule gradient:
    //   ∇v = (1 - h)·∇a + h·∇b + (3/8)·(2h - 1)·(∇b - ∇a)
    // for 0 < h < 1, and ∇v = ∇a or ∇b in the clamped region.
    // The clamped branches fall out automatically because (2h - 1)
    // is ±1 at h = 0 or h = 1 but (3/8)·(±1)·(∇b - ∇a) is the
    // (linear extrapolation) contribution; we must NOT add it in
    // the clamped region. Use a coefficient that vanishes at the
    // clamp boundaries.
    const float chain_coeff = (h > 0.0f && h < 1.0f)
        ? (3.0f / 8.0f) * (2.0f * h - 1.0f)
        : 0.0f;
    const math::Vec3f ga = sa.gradient;
    const math::Vec3f gb = sb.gradient;
    math::Vec3f g = ga * (1.0f - h) + gb * h + (gb - ga) * chain_coeff;
    return SDFSample{v, g};
}

void SmoothMinSDF::sample_batch(std::span<const math::Vec3f> points,
                                float* values,
                                math::Vec3f* gradients) const noexcept {
    const std::size_t n = points.size();
    a_->sample_batch(points, values, gradients);
    auto& pool = scratch_pool();
    BatchScratch& s = pool.level();
    ScratchClaim claim(pool);
    s.values.resize(n);
    s.gradients.resize(n);
    float* const sv = s.values.data();
    math::Vec3f* const sg = s.gradients.data();
    b_->sample_batch(points, sv, sg);
    if (k_ <= 0.0f) {
        // k=0 → hard min (the uniform branch, as sample() takes it)
        for (std::size_t i = 0; i < n; ++i) {
            float v = values[i];
            math::Vec3f g = gradients[i];
            if (sv[i] < v) {
                v = sv[i];
                g = sg[i];
            }
            values[i] = v;
            gradients[i] = defensive_normalize(g);
        }
        return;
    }
    for (std::size_t i = 0; i < n; ++i) {
        const float av = values[i];
        const float bv = sv[i];
        const math::Vec3f ga = gradients[i];
        const math::Vec3f gb = sg[i];
        const float diff = bv - av;
        const float h = std::clamp(0.5f - 0.5f * diff / k_, 0.0f, 1.0f);
        const float v = av + diff * h - k_ * h * (1.0f - h) * 0.25f;
        const float chain_coeff = (h > 0.0f && h < 1.0f)
            ? (3.0f / 8.0f) * (2.0f * h - 1.0f)
            : 0.0f;
        values[i] = v;
        gradients[i] = ga * (1.0f - h) + gb * h + (gb - ga) * chain_coeff;
    }
}

// ===========================================================================
// TranslateSDF / RotateSDF / ScaleSDF — the rigid transforms: the domain
// maps are the SAME scalar functions (bit-identity by construction),
// the child rides its own batch
// ===========================================================================

SDFSample TranslateSDF::sample(const math::Vec3f& p) const noexcept {
    return child_->sample(p - offset_);
}

void TranslateSDF::sample_batch(std::span<const math::Vec3f> points,
                                float* values,
                                math::Vec3f* gradients) const noexcept {
    const std::size_t n = points.size();
    auto& pool = scratch_pool();
    BatchScratch& s = pool.level();
    ScratchClaim claim(pool);
    s.points.resize(n);
    math::Vec3f* const local = s.points.data();
    for (std::size_t i = 0; i < n; ++i)
        local[i] = points[i] - offset_;
    child_->sample_batch(
        std::span<const math::Vec3f>(local, n), values, gradients);
}

SDFSample RotateSDF::sample(const math::Vec3f& p) const noexcept {
    const math::Vec3f local = inv_q_.rotate(p);
    auto s = child_->sample(local);
    s.gradient = qmat_.transform_dir(s.gradient);
    return s;
}

void RotateSDF::sample_batch(std::span<const math::Vec3f> points,
                             float* values,
                             math::Vec3f* gradients) const noexcept {
    const std::size_t n = points.size();
    auto& pool = scratch_pool();
    BatchScratch& s = pool.level();
    ScratchClaim claim(pool);
    s.points.resize(n);
    math::Vec3f* const local = s.points.data();
    for (std::size_t i = 0; i < n; ++i)
        local[i] = inv_q_.rotate(points[i]);
    child_->sample_batch(
        std::span<const math::Vec3f>(local, n), values, gradients);
    for (std::size_t i = 0; i < n; ++i)
        gradients[i] = qmat_.transform_dir(gradients[i]);
}

SDFSample ScaleSDF::sample(const math::Vec3f& p) const noexcept {
    auto r = child_->sample(p * inv_s_);
    // f(x) = scale · g(x/scale). The value scales linearly.
    r.value *= scale_;
    // ∇f(x) = ∇g(x/scale) — the gradient is inherited unchanged.
    // |∇f| = |∇g| ≤ lipschitz(g) = lipschitz(f), so the Lipschitz
    // invariant is preserved without any modification.
    return r;
}

void ScaleSDF::sample_batch(std::span<const math::Vec3f> points,
                            float* values,
                            math::Vec3f* gradients) const noexcept {
    const std::size_t n = points.size();
    auto& pool = scratch_pool();
    BatchScratch& s = pool.level();
    ScratchClaim claim(pool);
    s.points.resize(n);
    math::Vec3f* const local = s.points.data();
    for (std::size_t i = 0; i < n; ++i)
        local[i] = points[i] * inv_s_;
    child_->sample_batch(
        std::span<const math::Vec3f>(local, n), values, gradients);
    for (std::size_t i = 0; i < n; ++i)
        values[i] *= scale_;
}

} // namespace CAD_0::sdf
