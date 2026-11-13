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
    src/batch_kernels.cpp   # PERF 3: the xsimd leaf kernels + the
                           # composing interior (xsimd stays PRIVATE
                           # to this TU — the headers never see it)
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

# PERF 3, audit finding #2 — the SAME treatment for oneTBB: the TBB
# opt-in had never been compiled by any column (the register's own
# record), and the first compile wall was exactly this one — oneTBB
# v2022.0.0's own headers instantiate int -> depth_t (unsigned char)
# conversions inside tbb/parallel_for.h that surface as -Wconversion
# errors under the kernel targets' -Werror, in OUR TUs. The xsimd
# precedent applies verbatim: TBB's interface include directories
# become SYSTEM — the diagnostics from inside oneTBB's headers stop
# surfacing in first-party builds, the compiled code unchanged.
# (Verified in the sandbox before landing: evaluate.cpp with the TBB
# parallel_for instantiation compiles clean under the full project
# flag set + -Werror once the include is -isystem.)
#
# TBB::tbb may arrive as an IMPORTED target or as an ALIAS to the
# CPM-built `tbb` (this fetch: the alias) — properties land on the
# real target either way.
if(CAD_0_USE_TBB)
    foreach(_tbb_candidate TBB::tbb tbb)
        if(TARGET ${_tbb_candidate})
            get_target_property(_tbb_aliased ${_tbb_candidate} ALIASED_TARGET)
            if(_tbb_aliased)
                set(_tbb_real ${_tbb_aliased})
            else()
                set(_tbb_real ${_tbb_candidate})
            endif()
            get_target_property(_tbb_inc ${_tbb_real} INTERFACE_INCLUDE_DIRECTORIES)
            if(_tbb_inc)
                set_target_properties(${_tbb_real} PROPERTIES
                    INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_tbb_inc}")
            endif()
            break()
        endif()
    endforeach()
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
