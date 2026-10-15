// core/brep/include/CAD_0/brep/detail/boolean_test_hooks.hpp
//
// TEST-ONLY SEAM — NOT PART OF THE PUBLIC KERNEL API.
//
// This header exposes internal implementation details of the boolean
// kernel (ADR-0025) to the unit-test suite. It is not part of the
// public CAD_0/brep API: it will evolve without guarantee and may
// change or be removed at any time, without notice and without a
// deprecation cycle. Production code must include only
// CAD_0/brep/boolean.hpp and must never call anything declared here.
//
#pragma once

#include "CAD_0/math/vec.hpp"

#include <vector>

namespace CAD_0::brep {

// Direct access to the internal polygon_subtract_3d slab decomposition
// (ADR-0025 v5, FASE 2) so the unit-test suite can assert the
// "exactly one outside fragment" contract on a minimal slab case.
//
// Both polygons are coplanar with `plane_normal` as surface normal.
// Returns the outside fragments of subject \ clip, one point loop per
// fragment.
std::vector<std::vector<math::Vec3f>> polygon_subtract_fragments_for_tests(
    const std::vector<math::Vec3f>& subject,
    const std::vector<math::Vec3f>& clip,
    math::Vec3f plane_normal);

} // namespace CAD_0::brep
