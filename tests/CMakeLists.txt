# tests/CMakeLists.txt — top-level test aggregator
#
# We use doctest as the unit testing framework. Tests are compiled into
# a single executable `cadforge_tests` to minimize TBB/spdlog init overhead.

set(CADFORGE_TEST_SOURCES
    unit/math/test_vec.cpp
    unit/math/test_mat.cpp
    unit/math/test_bbox.cpp
    unit/math/test_predicate.cpp
    unit/math/test_tolerance.cpp
    unit/geometry/test_shape.cpp
    unit/geometry/test_shape_id.cpp
    unit/sdf/test_primitives.cpp
    unit/sdf/test_operators.cpp
    unit/sdf/test_transforms.cpp
    unit/sdf/test_evaluate.cpp
    unit/sdf/test_mesh_extract.cpp
    unit/io/test_stl.cpp
    unit/io/test_obj.cpp
    unit/io/test_threemf.cpp
    main.cpp
)

add_executable(cadforge_tests ${CADFORGE_TEST_SOURCES})

target_include_directories(cadforge_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/core/math/include
    ${CMAKE_SOURCE_DIR}/core/geometry/include
    ${CMAKE_SOURCE_DIR}/core/sdf/include
    ${CMAKE_SOURCE_DIR}/core/concurrency/include
    ${CMAKE_SOURCE_DIR}/core/brep/include
    ${CMAKE_SOURCE_DIR}/core/io/include
    ${CMAKE_SOURCE_DIR}/third_party/robust_predicates
    ${CMAKE_BINARY_DIR}/include
)

target_link_libraries(cadforge_tests PRIVATE
    cadforge_math
    cadforge_geometry
    cadforge_sdf
    cadforge_concurrency
    cadforge_io
)

# doctest is fetched via CPM. If CPM exposes the doctest target, link it.
if(TARGET doctest)
    target_link_libraries(cadforge_tests PRIVATE doctest)
    target_include_directories(cadforge_tests PRIVATE
        $<TARGET_PROPERTY:doctest,INTERFACE_INCLUDE_DIRECTORIES>)
elseif(TARGET doctest::doctest)
    target_link_libraries(cadforge_tests PRIVATE doctest::doctest)
endif()

# doctest main() — only main.cpp defines DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN.
# Other test files just include <doctest/doctest.h> without the macro.
set_source_files_properties(main.cpp PROPERTIES COMPILE_DEFINITIONS
    "DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN")

# Register tests with CTest — manual registration since doctest_discover_tests
# requires the doctest CMake module which isn't always available via CPM.
add_test(NAME cadforge_tests COMMAND cadforge_tests)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${CADFORGE_TEST_SOURCES})
