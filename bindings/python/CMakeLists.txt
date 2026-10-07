# bindings/python/CMakeLists.txt

set(CADFORGE_PY_SOURCES
    src/module.cpp
    src/math_bindings.cpp
    src/geometry_bindings.cpp
    src/sdf_bindings.cpp
    src/mesh_bindings.cpp
)

find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)

nanobind_add_module(
    _cadforge
    STABLE_ABI
    LTO
    NB_DOMAIN cadforge
    ${CADFORGE_PY_SOURCES}
)

target_link_libraries(_cadforge PRIVATE
    cadforge_math
    cadforge_geometry
    cadforge_sdf
    cadforge_concurrency
)

target_include_directories(_cadforge PRIVATE
    ${CMAKE_SOURCE_DIR}/core/math/include
    ${CMAKE_SOURCE_DIR}/core/geometry/include
    ${CMAKE_SOURCE_DIR}/core/sdf/include
    ${CMAKE_SOURCE_DIR}/core/concurrency/include
)

# Install the compiled extension next to the cadforge/ Python package.
install(TARGETS _cadforge LIBRARY DESTINATION cadforge)

# Copy the Python package source into the build tree for editable installs.
add_custom_target(cadforge_python_copy ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/cadforge
        ${CMAKE_BINARY_DIR}/python/cadforge
    COMMAND ${CMAKE_COMMAND} -E copy
        $<TARGET_FILE:_cadforge>
        ${CMAKE_BINARY_DIR}/python/cadforge/_cadforge$<TARGET_FILE_SUFFIX:_cadforge>
    DEPENDS _cadforge
)
set_target_properties(cadforge_python_copy PROPERTIES
    FOLDER "bindings/python")
