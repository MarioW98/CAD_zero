# core/brep/CMakeLists.txt — Phase D B-Rep kernel

set(CAD_0_BREP_HEADERS
    include/CAD_0/brep/topology.hpp
    include/CAD_0/brep/pcurve.hpp
    include/CAD_0/brep/analytic.hpp
    include/CAD_0/brep/curve.hpp
    include/CAD_0/brep/tessellate.hpp
    include/CAD_0/brep/factory.hpp
    include/CAD_0/brep/extrude.hpp
    include/CAD_0/brep/subtract.hpp
    include/CAD_0/brep/ssi.hpp
    include/CAD_0/brep/mass_properties.hpp
    include/CAD_0/brep/query.hpp
    include/CAD_0/brep/boolean.hpp
    include/CAD_0/brep/detail/boolean_test_hooks.hpp
)

set(CAD_0_BREP_SOURCES
    src/topology.cpp
    src/tessellate.cpp
    src/mass_properties.cpp
    src/query.cpp
    src/boolean.cpp
    src/ssi.cpp
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

# detail/ headers are test-only seams, not public API — never installed.
install(TARGETS CAD_0_brep ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY include/CAD_0 DESTINATION include
        PATTERN "detail" EXCLUDE)
