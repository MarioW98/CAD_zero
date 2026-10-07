// core/math/include/cadforge/math/predicate.hpp
//
// Adaptive robust geometric predicates (Shewchuk 1997, public domain).
//
// Wraps the C implementation vendored under third_party/robust_predicates/.
// The wrapper exposes:
//   * orient2d(a, b, c)        — signed area of triangle (a,b,c)
//   * orient3d(a, b, c, d)     — sign of (b-a) x (c-a) . (d-a)
//   * incircle2d(a, b, c, d)   — >0 if d is inside circle through a,b,c
//
// All return values are exact (sign is correct even when floating-point
// arithmetic would round the wrong way). Magnitude is informational only.
//
// See ADR-0002 for rationale on using Shewchuk instead of CGAL.
//
#pragma once

#include "cadforge/math/vec.hpp"

namespace cadforge::math::predicate {

// Initialize floating-point round-off error bounds. MUST be called once
// before any predicate. Idempotent and thread-safe after first call.
void initialize() noexcept;

// 2D orientation. Returns:
//   > 0 if a,b,c are counter-clockwise
//   < 0 if a,b,c are clockwise
//   = 0 if collinear
double orient2d(const Vec<2, double>& a,
                const Vec<2, double>& b,
                const Vec<2, double>& c) noexcept;

double orient2d(const Vec<2, float>& a,
                const Vec<2, float>& b,
                const Vec<2, float>& c) noexcept;

// 3D orientation. Returns:
//   > 0 if d is below plane a,b,c (assuming right-handed coord system)
//   < 0 if d is above plane
//   = 0 if coplanar
double orient3d(const Vec<3, double>& a,
                const Vec<3, double>& b,
                const Vec<3, double>& c,
                const Vec<3, double>& d) noexcept;

double orient3d(const Vec<3, float>& a,
                const Vec<3, float>& b,
                const Vec<3, float>& c,
                const Vec<3, float>& d) noexcept;

// In-circle test for 2D points. Returns:
//   > 0 if d is inside circle through a,b,c
//   < 0 if outside
//   = 0 if cocircular
double incircle2d(const Vec<2, double>& a,
                  const Vec<2, double>& b,
                  const Vec<2, double>& c,
                  const Vec<2, double>& d) noexcept;

} // namespace cadforge::math::predicate
