# core/geometry/CMakeLists.txt

set(CAD_0_GEOMETRY_HEADERS
    include/CAD_0/geometry/representation.hpp
    include/CAD_0/geometry/shape_id.hpp
    include/CAD_0/geometry/transform.hpp
    include/CAD_0/geometry/shape_bodies.hpp
    include/CAD_0/geometry/shape.hpp
    include/CAD_0/geometry/visitor.hpp
)

set(CAD_0_GEOMETRY_SOURCES
    src/shape.cpp
)

add_library(CAD_0_geometry STATIC ${CAD_0_GEOMETRY_SOURCES} ${CAD_0_GEOMETRY_HEADERS})

target_include_directories(CAD_0_geometry
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_BINARY_DIR}/include
        ${CMAKE_SOURCE_DIR}/core/sdf/include
        ${CMAKE_SOURCE_DIR}/core/brep/include
)

target_link_libraries(CAD_0_geometry
    PUBLIC CAD_0_math
    PRIVATE CAD_0_brep
)
target_compile_features(CAD_0_geometry PUBLIC cxx_std_20)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/include FILES ${CAD_0_GEOMETRY_HEADERS})

install(TARGETS CAD_0_geometry ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY include/CAD_0 DESTINATION include)
