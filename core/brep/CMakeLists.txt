# core/brep/CMakeLists.txt — Phase A stub
#
# In Phase A this module is empty. The CMakeLists exists so that
# CAD_0_geometry can link against the include path and have
# brep::BRepBody be a complete type (see core/brep/include/).
#
# Phase D will add real source files: topology.cpp, analytic.cpp,
# boolean.cpp, etc.

set(CAD_0_BREP_HEADERS
    include/CAD_0/brep/topology.hpp
)

add_library(CAD_0_brep INTERFACE ${CAD_0_BREP_HEADERS})

target_include_directories(CAD_0_brep INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(CAD_0_brep INTERFACE cxx_std_20)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/include FILES ${CAD_0_BREP_HEADERS})
