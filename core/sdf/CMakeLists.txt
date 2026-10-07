# core/sdf/CMakeLists.txt

set(CADFORGE_SDF_HEADERS
    include/cadforge/sdf/field.hpp
    include/cadforge/sdf/primitives.hpp
    include/cadforge/sdf/operators.hpp
    include/cadforge/sdf/transforms.hpp
    include/cadforge/sdf/evaluate.hpp
    include/cadforge/sdf/mesh_extract.hpp
)

set(CADFORGE_SDF_SOURCES
    src/evaluate.cpp
    src/mesh_extract.cpp
)

add_library(cadforge_sdf STATIC ${CADFORGE_SDF_SOURCES} ${CADFORGE_SDF_HEADERS})

target_include_directories(cadforge_sdf
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_BINARY_DIR}/include
)

target_link_libraries(cadforge_sdf
    PUBLIC
        cadforge_math
        cadforge_geometry
    PRIVATE
        xsimd
)

if(TBB::tbb)
    target_link_libraries(cadforge_sdf PRIVATE TBB::tbb)
    target_compile_definitions(cadforge_sdf PRIVATE CADFORGE_USE_TBB=1)
endif()

target_compile_features(cadforge_sdf PUBLIC cxx_std_20)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/include FILES ${CADFORGE_SDF_HEADERS})

install(TARGETS cadforge_sdf ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY include/cadforge DESTINATION include)
