# bindings/python/CMakeLists.txt

set(CAD_0_PY_SOURCES
    src/module.cpp
    src/math_bindings.cpp
    src/geometry_bindings.cpp
    src/sdf_bindings.cpp
    src/mesh_bindings.cpp
    src/scene_bindings.cpp
)

# Find Python3 with Development.Module component (provides Python.h + libpython).
# CMake will look up the Python interpreter pointed to by Python3_EXECUTABLE
# (or the first Python in PATH if not specified).
#
# If this fails with "Could NOT find Python3 (missing: Development.Module)":
#   * On Debian/Ubuntu: apt install python3-dev
#   * In a venv: the venv's Python must have its dev headers available.
#     For uv-managed venvs, the headers ship with the Python install (see
#     `<venv>/lib/python*/site-packages/.../include` or use `uv python install`).
#   * Pass -DPython3_EXECUTABLE=/path/to/python to CMake to pick a specific one.
#
# Note: find_package(Python3) is also called at the top-level CMakeLists.txt
# (so the Python3::Module imported target is available to core/concurrency/).
# Calling it again here is a no-op (CMake caches the result).
find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)

nanobind_add_module(
    _CAD_0
    STABLE_ABI
    LTO
    NB_DOMAIN CAD_0
    ${CAD_0_PY_SOURCES}
)

# Workaround: nanobind-static (created by nanobind_add_module) doesn't always
# inherit Python3_INCLUDE_DIRS in its compile command — particularly when
# Python is in a non-standard location (e.g. uv-managed venv). We add it
# explicitly here, AFTER nanobind_add_module creates the target.
if(TARGET nanobind-static)
    target_include_directories(nanobind-static PRIVATE
        ${Python3_INCLUDE_DIRS}
    )
endif()

target_link_libraries(_CAD_0 PRIVATE
    CAD_0_math
    CAD_0_geometry
    CAD_0_sdf
    CAD_0_concurrency
    CAD_0_io
    CAD_0_viewport
    CAD_0_brep
    CAD_0_scene
)

target_include_directories(_CAD_0 PRIVATE
    ${CMAKE_SOURCE_DIR}/core/math/include
    ${CMAKE_SOURCE_DIR}/core/geometry/include
    ${CMAKE_SOURCE_DIR}/core/sdf/include
    ${CMAKE_SOURCE_DIR}/core/concurrency/include
    ${CMAKE_SOURCE_DIR}/core/brep/include
    ${CMAKE_SOURCE_DIR}/core/io/include
    ${CMAKE_SOURCE_DIR}/core/viewport/include
    ${CMAKE_SOURCE_DIR}/core/scene/include
    ${CMAKE_BINARY_DIR}/include
    ${Python3_INCLUDE_DIRS}
)

# Install the compiled extension next to the CAD_0/ Python package.
install(TARGETS _CAD_0 LIBRARY DESTINATION CAD_0)

# Copy the Python package source into the build tree for editable installs.
# 1. Copy the bindings/python/CAD_0/ package (kernel API: sdf, io, viewer, etc.)
# 2. Copy the app/src/CAD_0/ui/ directory (PySide6 UI)
# 3. Copy the compiled extension _CAD_0.so
# 4. Merge __init__.py to include both kernel and UI imports
add_custom_target(CAD_0_python_copy ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/CAD_0
        ${CMAKE_BINARY_DIR}/python/CAD_0
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/app/src/CAD_0/ui
        ${CMAKE_BINARY_DIR}/python/CAD_0/ui
    COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_SOURCE_DIR}/app/src/CAD_0/__init__.py
        ${CMAKE_BINARY_DIR}/python/CAD_0/__init__.py
    COMMAND ${CMAKE_COMMAND} -E copy
        $<TARGET_FILE:_CAD_0>
        ${CMAKE_BINARY_DIR}/python/CAD_0/_CAD_0$<TARGET_FILE_SUFFIX:_CAD_0>
    DEPENDS _CAD_0
)
set_target_properties(CAD_0_python_copy PROPERTIES
    FOLDER "bindings/python")
