# core/geometry/CMakeLists.txt

set(CADFORGE_GEOMETRY_HEADERS
    include/cadforge/geometry/representation.hpp
    include/cadforge/geometry/shape_id.hpp
    include/cadforge/geometry/transform.hpp
    include/cadforge/geometry/shape_bodies.hpp
    include/cadforge/geometry/shape.hpp
    include/cadforge/geometry/visitor.hpp
)

set(CADFORGE_GEOMETRY_SOURCES
    src/shape.cpp
)

add_library(cadforge_geometry STATIC ${CADFORGE_GEOMETRY_SOURCES} ${CADFORGE_GEOMETRY_HEADERS})

target_include_directories(cadforge_geometry
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_BINARY_DIR}/include
        ${CMAKE_SOURCE_DIR}/core/sdf/include
        ${CMAKE_SOURCE_DIR}/core/brep/include
)

target_link_libraries(cadforge_geometry
    PUBLIC cadforge_math
    PRIVATE cadforge_brep
)
target_compile_features(cadforge_geometry PUBLIC cxx_std_20)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/include FILES ${CADFORGE_GEOMETRY_HEADERS})

install(TARGETS cadforge_geometry ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY include/cadforge DESTINATION include)
