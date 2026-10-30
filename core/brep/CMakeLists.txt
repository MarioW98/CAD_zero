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
    include/CAD_0/brep/naming.hpp
    include/CAD_0/brep/descent.hpp
    include/CAD_0/brep/detail/boolean_test_hooks.hpp
)

set(CAD_0_BREP_SOURCES
    src/topology.cpp
    src/tessellate.cpp
    src/mass_properties.cpp
    src/query.cpp
    src/boolean.cpp
    src/ssi.cpp
    src/naming.cpp
    src/descent.cpp
)

add_library(CAD_0_brep STATIC ${CAD_0_BREP_SOURCES} ${CAD_0_BREP_HEADERS})

target_include_directories(CAD_0_brep
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        # The naming ABI (ADR-0005, as amended by ADR-0036 D1):
        # topology.hpp includes geometry/shape_id.hpp — a LEAF header
        # (config + std only, nothing of brep on its side), so the
        # include path is PUBLIC while the LINK stays below without
        # geometry (CAD_0_geometry links CAD_0_brep PRIVATE — a target
        # cycle must not form). No CAD_0_brep symbol needs geometry's:
        # the ShapeId is fully inline in every use brep makes of it.
        ${CMAKE_SOURCE_DIR}/core/geometry/include
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
