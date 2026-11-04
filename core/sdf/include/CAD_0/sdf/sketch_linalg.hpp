// core/sdf/include/CAD_0/sdf/sketch_linalg.hpp
//
// THE IN-HOUSE LINEAR ALGEBRA — ADR-0040 D2.2 (G4 m2, the accepted
// act): "No Eigen, no LAPACK — the third-party policy's whitelist
// unchanged. The systems are v1-small (the params <= ~64, the
// sketcher's authored scale); the house implements the one file it
// needs (the Cholesky/LU with the fixed-pivot discipline, the SVD's
// structure where the rank test needs it): deterministic, pure
// arithmetic, the same bar the predicates set."
//
// This header is that one file. Three instruments, each with its
// consumer inside the solver:
//
//   * CHOLESKY — the Newton step's own solve: the damped normal
//     equations (JᵀJ + λI) δ = −JᵀF are symmetric positive definite
//     by construction (λ > 0), and Cholesky is the SPD discipline:
//     no pivoting, the pivot order IS the canonical parameter order,
//     every operation a fixed sequence of +,−,×,÷,√. Backward
//     stable for SPD inputs — the conditioning the ADR declares
//     (λ₀ 1e-6 keeps the well-posed cases at pure-Newton quality;
//     the λ floor 1e-14 is the under-constrained case's honest
//     tail, where the fallback below answers).
//
//   * LU, PARTIAL PIVOTING — the deterministic fallback when
//     Cholesky refuses (a pivot not positive: the SPD property lost
//     to rounding at the λ floor's ill-conditioning — the matrix
//     still nonsingular). The "fixed-pivot discipline" of the ADR
//     is the DETERMINISM of the rule, stated here as the contract:
//     the pivot search is the plain row-argmax with ties to the
//     LOWEST index — the same input, the same factorization, bit
//     for bit, on every platform (ADR-0014's own bar; the same
//     class LAPACK's dgesv practices, in house, one algorithm, no
//     thresholds, no adaptivity).
//
//   * SVD (ONE-SIDED JACOBI, HESTENES) — the singular values the
//     rank test reads (the admission's verdict and the refusal's
//     spectrum tail). The rotations are the RATIONAL form — no
//     trigonometry: t = sign(ζ)/(|ζ| + √(1+ζ²)), c = 1/√(1+t²),
//     s = c·t — pure arithmetic (+,−,×,÷,√) exactly as the
//     equations of ADR-0040 D2.2 are. The sweep order is the fixed
//     (i,j) lexicographic order, the convergence criterion is the
//     relative off-diagonal inner product, the sweep cap is 60 —
//     every choice a constant, none data-dependent: deterministic.
//
// The matrices are dense row-major double on std::vector<double> —
// v1-small by contract; no allocation tricks, no blocking, nothing
// that could reorder an addition. The functions are pure: they
// touch nothing but their arguments.
#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

namespace CAD_0::sdf::sketch_linalg {

// ---------------------------------------------------------------------------
// Cholesky — A (n×n row-major, SPD) factorized in place as L·Lᵀ, then
// A·x = b solved; b is REPLACED by x. Returns false when a pivot is
// not positive (the honest failure: the caller takes the no-step
// path or the LU fallback — never a crash, never a garbage solve).
// ---------------------------------------------------------------------------
inline bool cholesky_solve(std::vector<double>& A, std::vector<double>& b,
                           std::size_t n) {
    if (n == 0) return true;
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j <= i; ++j) {
            double sum = A[i * n + j];
            for (std::size_t k = 0; k < j; ++k)
                sum -= A[i * n + k] * A[j * n + k];
            if (i == j) {
                if (!(sum > 0.0)) return false; // not numerically SPD
                A[i * n + i] = std::sqrt(sum);
            } else {
                A[i * n + j] = sum / A[j * n + j];
            }
        }
    }
    // forward: L y = b
    for (std::size_t i = 0; i < n; ++i) {
        double sum = b[i];
        for (std::size_t k = 0; k < i; ++k) sum -= A[i * n + k] * b[k];
        b[i] = sum / A[i * n + i];
    }
    // backward: Lᵀ x = y
    for (std::size_t ii = 0; ii < n; ++ii) {
        const std::size_t i = n - 1 - ii;
        double sum = b[i];
        for (std::size_t k = i + 1; k < n; ++k) sum -= A[k * n + i] * b[k];
        b[i] = sum / A[i * n + i];
    }
    return true;
}

// ---------------------------------------------------------------------------
// LU with partial pivoting — A (n×n row-major) factorized in place as
// P·A = L·U, then A·x = b solved; b is REPLACED by x. The pivot rule
// is FIXED: the row of the maximal |value| in the column, ties to the
// lowest index — deterministic, no thresholds. Returns false only on
// an exactly-zero pivot (singular — the caller's no-step path).
// ---------------------------------------------------------------------------
inline bool lu_solve(std::vector<double>& A, std::vector<double>& b,
                     std::size_t n) {
    if (n == 0) return true;
    std::vector<std::size_t> piv(n);
    for (std::size_t i = 0; i < n; ++i) piv[i] = i;
    for (std::size_t k = 0; k < n; ++k) {
        // the fixed pivot search: max |A[i][k]| for i >= k, lowest index
        std::size_t best = k;
        double best_abs = std::abs(A[k * n + k]);
        for (std::size_t i = k + 1; i < n; ++i) {
            const double v = std::abs(A[i * n + k]);
            if (v > best_abs) { best_abs = v; best = i; }
        }
        if (!(best_abs > 0.0)) return false; // exactly singular
        if (best != k) {
            for (std::size_t j = 0; j < n; ++j)
                std::swap(A[k * n + j], A[best * n + j]);
            std::swap(b[k], b[best]);
            std::swap(piv[k], piv[best]);
        }
        const double pivval = A[k * n + k];
        for (std::size_t i = k + 1; i < n; ++i) {
            const double f = A[i * n + k] / pivval;
            A[i * n + k] = f;
            if (f != 0.0)
                for (std::size_t j = k + 1; j < n; ++j)
                    A[i * n + j] -= f * A[k * n + j];
            b[i] -= f * b[k];
        }
    }
    // backward: U x = y (b already carries L's application)
    for (std::size_t ii = 0; ii < n; ++ii) {
        const std::size_t i = n - 1 - ii;
        double sum = b[i];
        for (std::size_t k = i + 1; k < n; ++k) sum -= A[i * n + k] * b[k];
        b[i] = sum / A[i * n + i];
    }
    return true;
}

// ---------------------------------------------------------------------------
// The one-sided Jacobi SVD's singular values (Hestenes): the columns
// of A (m×n row-major) orthogonalized by rational rotations until
// every pair's inner product is relatively negligible (or the sweep
// cap 60 — a bounded, deterministic loop). s returns n values in
// DESCENDING order. For the rank question the values' ABSOLUTE
// order against the declared cutoff is all that matters; the Jacobi
// sweep delivers them to full double precision on the v1-small
// systems the solver builds.
// ---------------------------------------------------------------------------
inline void svd_values(const std::vector<double>& A, std::size_t m,
                       std::size_t n, std::vector<double>& s) {
    s.assign(n, 0.0);
    if (m == 0 || n == 0) return;
    // Work on a column-major copy: col j = A[:, j].
    std::vector<double> col(m * n);
    for (std::size_t i = 0; i < m; ++i)
        for (std::size_t j = 0; j < n; ++j) col[j * m + i] = A[i * n + j];

    constexpr int kMaxSweeps = 60;
    constexpr double kOffTol = 1e-15; // relative: |xᵀy| <= tol·|x||y|

    for (int sweep = 0; sweep < kMaxSweeps; ++sweep) {
        bool rotated = false;
        for (std::size_t i = 0; i < n; ++i) {
            for (std::size_t j = i + 1; j < n; ++j) {
                double* xi = &col[i * m];
                double* xj = &col[j * m];
                double aii = 0.0, ajj = 0.0, aij = 0.0;
                for (std::size_t k = 0; k < m; ++k) {
                    aii += xi[k] * xi[k];
                    ajj += xj[k] * xj[k];
                    aij += xi[k] * xj[k];
                }
                if (aii == 0.0 || ajj == 0.0) continue;
                if (std::abs(aij) <= kOffTol * std::sqrt(aii * ajj)) continue;
                rotated = true;
                // The rational rotation (no trig):
                //   ζ = (ajj − aii) / (2 aij)
                //   t = sign(ζ)/(|ζ| + √(1+ζ²))   [t = tan of the Jacobi angle]
                //   c = 1/√(1+t²), s = c·t
                const double zeta = (ajj - aii) / (2.0 * aij);
                const double t =
                    (zeta >= 0.0 ? 1.0 : -1.0) /
                    (std::abs(zeta) + std::sqrt(1.0 + zeta * zeta));
                const double c = 1.0 / std::sqrt(1.0 + t * t);
                const double sn = c * t;
                for (std::size_t k = 0; k < m; ++k) {
                    const double xk = xi[k], yk = xj[k];
                    xi[k] = c * xk - sn * yk;
                    xj[k] = sn * xk + c * yk;
                }
            }
        }
        if (!rotated) break;
    }
    for (std::size_t j = 0; j < n; ++j) {
        double norm = 0.0;
        for (std::size_t k = 0; k < m; ++k) norm += col[j * m + k] * col[j * m + k];
        s[j] = std::sqrt(norm);
    }
    // Descending — a fixed insertion sort (n is v1-small; the order
    // of equal values is the column order, deterministic).
    for (std::size_t i = 1; i < n; ++i) {
        const double v = s[i];
        std::size_t k = i;
        while (k > 0 && s[k - 1] < v) { // strict: stable on ties
            s[k] = s[k - 1];
            --k;
        }
        s[k] = v;
    }
}

// The numerical rank: the singular values strictly above the declared
// ABSOLUTE cutoff (1e-12 — the derivation's RCOND, the same number
// the pre-registration script judged with).
inline std::size_t rank_with_cutoff(const std::vector<double>& s,
                                    double cutoff) {
    std::size_t r = 0;
    for (const double v : s)
        if (v > cutoff) ++r;
    return r;
}

} // namespace CAD_0::sdf::sketch_linalg
