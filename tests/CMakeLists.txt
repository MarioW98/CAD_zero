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
    unit/io/test_stl.cpp
    unit/io/test_obj.cpp
    unit/io/test_threemf.cpp
    unit/io/test_native.cpp
    unit/scene/test_builders.cpp
    unit/scene/test_mesh_nodes.cpp
    unit/scene/test_operations.cpp
    unit/scene/test_sketch_scene.cpp
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
    unit/brep/test_h2_u3_pcurve_double.cpp
    unit/bridge/test_bridge.cpp
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

# FASE C: the kernel test suite compiles every kernel header, so it gets
# -Werror exactly like the kernel targets themselves (see
# core/CMakeLists.txt for the full scope rationale). GNU/Clang only.
target_compile_options(CAD_0_tests PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Werror>)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${CAD_0_TEST_SOURCES})
