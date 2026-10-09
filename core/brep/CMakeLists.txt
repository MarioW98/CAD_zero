# core/brep/CMakeLists.txt — Phase D B-Rep kernel

set(CAD_0_BREP_HEADERS
    include/CAD_0/brep/topology.hpp
    include/CAD_0/brep/analytic.hpp
    include/CAD_0/brep/curve.hpp
    include/CAD_0/brep/tessellate.hpp
    include/CAD_0/brep/factory.hpp
)

set(CAD_0_BREP_SOURCES
    src/tessellate.cpp
)

add_library(CAD_0_brep STATIC ${CAD_0_BREP_SOURCES} ${CAD_0_BREP_HEADERS})

target_include_directories(CAD_0_brep
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_BINARY_DIR}/include
)

target_link_libraries(CAD_0_brep
    PUBLIC
        CAD_0_math
        CAD_0_sdf
)

target_compile_features(CAD_0_brep PUBLIC cxx_std_20)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/include FILES ${CAD_0_BREP_HEADERS})

install(TARGETS CAD_0_brep ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY include/CAD_0 DESTINATION include)
