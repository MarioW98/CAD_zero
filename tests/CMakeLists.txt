# tests/CMakeLists.txt — top-level test aggregator
#
# We use doctest as the unit testing framework. Tests are compiled into
# a single executable `CAD_0_tests` to minimize TBB/spdlog init overhead.

set(CAD_0_TEST_SOURCES
    unit/math/test_vec.cpp
    unit/math/test_mat.cpp
    unit/math/test_bbox.cpp
    unit/math/test_predicate.cpp
    unit/math/test_predicate_adaptive.cpp
    unit/math/test_tolerance.cpp
    unit/geometry/test_shape.cpp
    unit/geometry/test_shape_id.cpp
    unit/sdf/test_primitives.cpp
    unit/sdf/test_operators.cpp
    unit/sdf/test_transforms.cpp
    unit/sdf/test_evaluate.cpp
    unit/sdf/test_mesh_extract.cpp
    unit/sdf/test_sketch2d_entities.cpp
    unit/sdf/test_sketch2d_profile.cpp
    unit/sdf/test_sketch2d_extrude.cpp
    unit/sdf/test_sketch2d_solver.cpp
    unit/io/test_stl.cpp
    unit/io/test_obj.cpp
    unit/io/test_threemf.cpp
    unit/io/test_native.cpp
    unit/scene/test_builders.cpp
    unit/scene/test_mesh_nodes.cpp
    unit/scene/test_operations.cpp
    unit/scene/test_sketch_scene.cpp
    unit/scene/test_g2_u3_m6_fillet_display.cpp
    unit/viewport/test_camera.cpp
    unit/viewport/test_command_stack.cpp
    unit/viewport/test_selection.cpp
    unit/viewport/test_renderer.cpp
    unit/viewport/test_ray_march.cpp
    unit/brep/test_topology.cpp
    unit/brep/test_analytic_surfaces.cpp
    unit/brep/test_boolean.cpp
    unit/brep/test_g1_data_model.cpp
    unit/brep/test_g1_unit2_pcurves.cpp
    unit/brep/test_g1_unit3_extrude.cpp
    unit/brep/test_g1_unit4_subtract.cpp
    unit/brep/test_g1_unit5_mass.cpp
    unit/brep/test_g1_unit6_report.cpp
    unit/brep/test_g2_unit1_ssi_table.cpp
    unit/brep/test_g2_unit2_blind_wall.cpp
    unit/brep/test_g2_unit2_m3_crossed.cpp
    unit/brep/test_g2_unit2_m4_blind_door.cpp
    unit/brep/test_g2_u3_m2_fillet.cpp
    unit/brep/test_g2_u3_m3_toroidal.cpp
    unit/brep/test_g2_u3_m6bis_the_trim.cpp
    unit/brep/test_g2_u3_m6ter_the_chain.cpp
    unit/brep/test_k1_m3_ellipsoid.cpp
    unit/brep/test_k1_m4_zoo.cpp
    unit/brep/test_k1_m5_chain_gate.cpp
    unit/brep/test_k1_m6_survival_ledger.cpp
    unit/brep/test_k1_m7_oracle_suite.cpp
    unit/brep/test_h2_u3_pcurve_double.cpp
    unit/brep/test_h2_u6_m2_factory_names.cpp
    unit/brep/test_h2_u6_m3_extrude_names.cpp
    unit/brep/test_h2_u6_m4_subtract_names.cpp
    unit/brep/test_h2_u6_m5_resolver.cpp
    unit/brep/test_h2_u6_m6_d5_quartet.cpp
    unit/bridge/test_bridge.cpp
    unit/bridge/test_h2_u4_m2_bvh.cpp
    main.cpp
)

add_executable(CAD_0_tests ${CAD_0_TEST_SOURCES})

target_include_directories(CAD_0_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/core/math/include
    # Property-based families (mini-traccia): seeded generators live in
    # tests/property/ and are shared by the per-area test TUs.
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/core/geometry/include
    ${CMAKE_SOURCE_DIR}/core/sdf/include
    ${CMAKE_SOURCE_DIR}/core/concurrency/include
    ${CMAKE_SOURCE_DIR}/core/brep/include
    ${CMAKE_SOURCE_DIR}/core/scene/include
    ${CMAKE_SOURCE_DIR}/core/io/include
    ${CMAKE_SOURCE_DIR}/core/viewport/include
    ${CMAKE_SOURCE_DIR}/core/scene/include
    ${CMAKE_SOURCE_DIR}/core/bridge/include
    ${CMAKE_BINARY_DIR}/include
)
# FASE B: vendored Shewchuk predicates — SYSTEM (same as CAD_0_math).
target_include_directories(CAD_0_tests SYSTEM PRIVATE
    ${CMAKE_SOURCE_DIR}/third_party/robust_predicates)

target_link_libraries(CAD_0_tests PRIVATE
    CAD_0_math
    CAD_0_geometry
    CAD_0_sdf
    CAD_0_concurrency
    CAD_0_io
    CAD_0_viewport
    CAD_0_brep
    CAD_0_bridge
)

# doctest is fetched via CPM. If CPM exposes the doctest target, link it.
# FASE B (warning bonifica): doctest is third-party — its headers are
# consumed as SYSTEM (-isystem) so its diagnostics can never surface
# (nor break -Werror) in first-party test TUs.
if(TARGET doctest)
    target_link_libraries(CAD_0_tests PRIVATE doctest)
    get_target_property(CAD_0_DOCTEST_INC doctest INTERFACE_INCLUDE_DIRECTORIES)
    if(CAD_0_DOCTEST_INC)
        set_target_properties(doctest PROPERTIES
            INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${CAD_0_DOCTEST_INC}")
    endif()
    target_include_directories(CAD_0_tests SYSTEM PRIVATE
        $<TARGET_PROPERTY:doctest,INTERFACE_INCLUDE_DIRECTORIES>)
elseif(TARGET doctest::doctest)
    target_link_libraries(CAD_0_tests PRIVATE doctest::doctest)
endif()

# doctest main() — only main.cpp defines DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN.
# Other test files just include <doctest/doctest.h> without the macro.
set_source_files_properties(main.cpp PROPERTIES COMPILE_DEFINITIONS
    "DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN")

# Source dir for golden files (tests/golden/) and other test assets.
target_compile_definitions(CAD_0_tests PRIVATE
    CAD_0_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

# Register tests with CTest — manual registration since doctest_discover_tests
# requires the doctest CMake module which isn't always available via CPM.
add_test(NAME CAD_0_tests COMMAND CAD_0_tests)

# The Python bindings suite as a ctest member (Consegna B m1, 2026-11-01).
#
# The bindings — nanobind, the memory bridges — are the most exposed
# surface of the library and were the least protected: the H1
# correction report had declared the gap (the binding tests ran
# WITHOUT AddressSanitizer while the same C++ paths were instrumented
# through the suite above). This target closes it: the suite that ran
# as a bare workflow step now runs inside ctest, in EVERY job of the
# CI matrix, and in the instrumented build (linux-gcc-debug,
# CAD_0_SANITIZE=ON) the launcher preloads the matching libasan so
# the module imports UNDER the sanitizer — the same instrumented
# build the C++ suite consumes, consumed by the Python layer too.
#
# The launcher (scripts/run_python_bindings_suite.py) owns the whole
# environment (PYTHONPATH, the ASan preload, detect_leaks=0) and
# verifies the instrumentation against the module itself — a config
# that promises ASan and a module that lacks it fails the run.
#
# Linux-only registration: the workflow's columns are Linux; the
# Windows column is a registered unit, not a testimony (see
# docs/audits/2026-11-01-ci-runner-asan-and-attestations.md).
if(CAD_0_BUILD_PYTHON AND UNIX AND NOT CMAKE_CROSSCOMPILING)
    # Configure-time half of the ASan contract: when this build is
    # instrumented with GNU (GCC links libasan dynamically), resolve
    # the runtime now and hand it to the launcher. A configure that
    # cannot produce the preload path fails HERE, not as a flaky
    # runtime surprise.
    set(CAD_0_BINDINGS_ASAN_RUNTIME "")
    if(CAD_0_SANITIZE AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        execute_process(
            COMMAND ${CMAKE_CXX_COMPILER} -print-file-name=libasan.so
            OUTPUT_VARIABLE CAD_0_BINDINGS_LIBASAN
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )
        if(NOT IS_ABSOLUTE "${CAD_0_BINDINGS_LIBASAN}")
            message(FATAL_ERROR
                "CAD_0_SANITIZE=ON with GNU but "
                "'${CMAKE_CXX_COMPILER} -print-file-name=libasan.so' "
                "returned '${CAD_0_BINDINGS_LIBASAN}' — the ASan runtime "
                "is not resolvable, and the bindings suite runs under the "
                "sanitizer or the configuration fails (install the GCC "
                "ASan runtime, e.g. libasan8; the gap is not closed by "
                "skipping the target).")
        endif()
        set(CAD_0_BINDINGS_ASAN_RUNTIME "${CAD_0_BINDINGS_LIBASAN}")
    endif()
    add_test(
        NAME CAD_0_python_bindings
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/scripts/run_python_bindings_suite.py
            --python-dir ${CMAKE_BINARY_DIR}/python
            --tests-dir ${CMAKE_SOURCE_DIR}/bindings/python/tests
            --asan-runtime "${CAD_0_BINDINGS_ASAN_RUNTIME}"
            --asan-compiler ${CMAKE_CXX_COMPILER}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
endif()

# FASE C: the kernel test suite compiles every kernel header, so it gets
# -Werror exactly like the kernel targets themselves (see
# core/CMakeLists.txt for the full scope rationale). GNU/Clang only.
target_compile_options(CAD_0_tests PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Werror>)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${CAD_0_TEST_SOURCES})
