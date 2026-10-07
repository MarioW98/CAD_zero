# core/brep/CMakeLists.txt — Phase A stub
#
# In Phase A this module is empty. The CMakeLists exists so that
# cadforge_geometry can link against the include path and have
# brep::BRepBody be a complete type (see core/brep/include/).
#
# Phase D will add real source files: topology.cpp, analytic.cpp,
# boolean.cpp, etc.

set(CADFORGE_BREP_HEADERS
    include/cadforge/brep/topology.hpp
)

add_library(cadforge_brep INTERFACE ${CADFORGE_BREP_HEADERS})

target_include_directories(cadforge_brep INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(cadforge_brep INTERFACE cxx_std_20)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/include FILES ${CADFORGE_BREP_HEADERS})
