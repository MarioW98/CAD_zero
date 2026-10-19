# core/sdf/CMakeLists.txt

set(CAD_0_SDF_HEADERS
    include/CAD_0/sdf/field.hpp
    include/CAD_0/sdf/primitives.hpp
    include/CAD_0/sdf/operators.hpp
    include/CAD_0/sdf/transforms.hpp
    include/CAD_0/sdf/detail/validate.hpp
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

# FASE B (warning bonifica): xsimd's internal SIMD shims emit clang
# conversion diagnostics (-Wshorten-64-to-32, -Wfloat-conversion) inside
# their own headers when CAD_0_sdf TUs instantiate them. Third-party code:
# mark its interface include directories SYSTEM so diagnostics from
# inside xsimd do not surface in first-party builds. Include resolution
# and the compiled code are unchanged (only -I becomes -isystem).
get_target_property(CAD_0_XSIMD_INC xsimd INTERFACE_INCLUDE_DIRECTORIES)
if(CAD_0_XSIMD_INC)
    set_target_properties(xsimd PROPERTIES
        INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${CAD_0_XSIMD_INC}")
endif()

# Link TBB when enabled. TBB::tbb is an imported target created by
# oneTBB's CMake config. We check both names for robustness.
if(CAD_0_USE_TBB)
    if(TARGET TBB::tbb)
        target_link_libraries(CAD_0_sdf PRIVATE TBB::tbb)
    elseif(TARGET tbb)
        target_link_libraries(CAD_0_sdf PRIVATE tbb)
    endif()
    target_compile_definitions(CAD_0_sdf PUBLIC CAD_0_USE_TBB=1)
endif()

target_compile_features(CAD_0_sdf PUBLIC cxx_std_20)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/include FILES ${CAD_0_SDF_HEADERS})

install(TARGETS CAD_0_sdf ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY include/CAD_0 DESTINATION include)
