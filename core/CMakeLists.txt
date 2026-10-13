# core/CMakeLists.txt — top-level core aggregation

add_subdirectory(math)
add_subdirectory(geometry)
add_subdirectory(sdf)
add_subdirectory(brep)
add_subdirectory(io)
add_subdirectory(viewport)
add_subdirectory(scene)
add_subdirectory(bridge)

# Concurrency is added when CAD_0_USE_TBB is set; otherwise a header-only
# scalar fallback is provided.
add_subdirectory(concurrency)

# Bridge and modeling land in later phases.
if(CAD_0_BUILD_BREP)
    add_subdirectory(bridge)
    add_subdirectory(modeling)
endif()

# Convenience aggregate target for downstream consumers.
add_library(CAD_0_core INTERFACE)
target_link_libraries(CAD_0_core INTERFACE
    CAD_0_math
    CAD_0_geometry
    CAD_0_sdf
    CAD_0_concurrency
    CAD_0_brep
    CAD_0_io
    CAD_0_viewport
    CAD_0_scene
    CAD_0_bridge
)

# ---------------------------------------------------------------------------
# FASE C: -Werror on the kernel and its test suite — nothing else.
#
# Scope (deliberately narrow, see docs/audits/2026-10-13-warning-audit.md):
#   * the 8 compiled kernel targets below (their own .cpp TUs);
#   * CAD_0_tests (registers in tests/CMakeLists.txt) — it compiles every
#     kernel header, so header-level diagnostics turn into errors there;
#   * NOT the Python bindings (_CAD_0), NOT examples, NOT third-party
#     targets: -Werror must not leak into nanobind/Python TUs. That is why
#     the flag is PRIVATE on these targets instead of INTERFACE on
#     CAD_0_core (an INTERFACE option would reach every consumer).
#
# CAD_0_scene is header-only (INTERFACE): it has no TUs of its own; its
# headers are enforced through CAD_0_tests.
#
# Third-party headers reachable from these TUs are already consumed as
# SYSTEM includes (FASE B): robust_predicates, xsimd (CAD_0_sdf), Python
# (Python3::Module already exports -isystem), doctest in tests. Verified
# warning-free with the CI compilers — clang 14 and gcc 11.4
# (ubuntu-22.04, preset linux-gcc11-ci) — and with gcc 14.2 local: still
# NO -Wno-error= exceptions. The one gcc-11 false positive found so far
# ('maybe_unused' attribute ignored on non-static data members, GCC < 12)
# was fixed by REWRITING the pattern (compiler-gated attribute in
# opengl_renderer.hpp), not by suppressing the diagnostic — that is the
# required style for any future case (see the "Contratto di toolchain"
# section in docs/audits/2026-10-13-warning-audit.md). If a
# compiler-specific false positive shows up that genuinely cannot be
# rewritten (documented GCC 12+ stringop-overflow family noted for oneTBB
# at the top of the root CMakeLists — currently OFF), add a narrowly
# scoped and commented -Wno-error= HERE — never a global -Wno-*, and
# never an exemption from this list.
# ---------------------------------------------------------------------------
set(CAD_0_KERNEL_WERROR_TARGETS
    CAD_0_math
    CAD_0_geometry
    CAD_0_sdf
    CAD_0_brep
    CAD_0_io
    CAD_0_viewport
    CAD_0_concurrency
    CAD_0_bridge
)
foreach(_kernel_target IN LISTS CAD_0_KERNEL_WERROR_TARGETS)
    if(TARGET ${_kernel_target})
        target_compile_options(${_kernel_target} PRIVATE
            $<$<CXX_COMPILER_ID:GNU,Clang>:-Werror>)
    endif()
endforeach()
