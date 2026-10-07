# bindings/python/CMakeLists.txt

set(CAD_0_PY_SOURCES
    src/module.cpp
    src/math_bindings.cpp
    src/geometry_bindings.cpp
    src/sdf_bindings.cpp
    src/mesh_bindings.cpp
)

find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)

nanobind_add_module(
    _CAD_0
    STABLE_ABI
    LTO
    NB_DOMAIN CAD_0
    ${CAD_0_PY_SOURCES}
)

target_link_libraries(_CAD_0 PRIVATE
    CAD_0_math
    CAD_0_geometry
    CAD_0_sdf
    CAD_0_concurrency
)

target_include_directories(_CAD_0 PRIVATE
    ${CMAKE_SOURCE_DIR}/core/math/include
    ${CMAKE_SOURCE_DIR}/core/geometry/include
    ${CMAKE_SOURCE_DIR}/core/sdf/include
    ${CMAKE_SOURCE_DIR}/core/concurrency/include
)

# Install the compiled extension next to the CAD_0/ Python package.
install(TARGETS _CAD_0 LIBRARY DESTINATION CAD_0)

# Copy the Python package source into the build tree for editable installs.
add_custom_target(CAD_0_python_copy ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/CAD_0
        ${CMAKE_BINARY_DIR}/python/CAD_0
    COMMAND ${CMAKE_COMMAND} -E copy
        $<TARGET_FILE:_CAD_0>
        ${CMAKE_BINARY_DIR}/python/CAD_0/_CAD_0$<TARGET_FILE_SUFFIX:_CAD_0>
    DEPENDS _CAD_0
)
set_target_properties(CAD_0_python_copy PROPERTIES
    FOLDER "bindings/python")
