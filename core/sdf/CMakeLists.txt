# core/sdf/CMakeLists.txt

set(CAD_0_SDF_HEADERS
    include/CAD_0/sdf/field.hpp
    include/CAD_0/sdf/primitives.hpp
    include/CAD_0/sdf/operators.hpp
    include/CAD_0/sdf/transforms.hpp
    include/CAD_0/sdf/evaluate.hpp
    include/CAD_0/sdf/mesh_extract.hpp
)

set(CAD_0_SDF_SOURCES
    src/evaluate.cpp
    src/mesh_extract.cpp
)

add_library(CAD_0_sdf STATIC ${CAD_0_SDF_SOURCES} ${CAD_0_SDF_HEADERS})

target_include_directories(CAD_0_sdf
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_BINARY_DIR}/include
)

target_link_libraries(CAD_0_sdf
    PUBLIC
        CAD_0_math
        CAD_0_geometry
    PRIVATE
        xsimd
)

if(TBB::tbb)
    target_link_libraries(CAD_0_sdf PRIVATE TBB::tbb)
    target_compile_definitions(CAD_0_sdf PRIVATE CAD_0_USE_TBB=1)
endif()

target_compile_features(CAD_0_sdf PUBLIC cxx_std_20)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/include FILES ${CAD_0_SDF_HEADERS})

install(TARGETS CAD_0_sdf ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY include/CAD_0 DESTINATION include)
