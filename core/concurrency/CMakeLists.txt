# core/concurrency/CMakeLists.txt

set(CAD_0_CONCURRENCY_HEADERS
    include/CAD_0/concurrency/thread_pool.hpp
    include/CAD_0/concurrency/parallel_for.hpp
    include/CAD_0/concurrency/gil_guard.hpp
    include/CAD_0/concurrency/task_graph.hpp
)

set(CAD_0_CONCURRENCY_SOURCES
    src/thread_pool.cpp
)

add_library(CAD_0_concurrency STATIC ${CAD_0_CONCURRENCY_SOURCES} ${CAD_0_CONCURRENCY_HEADERS})

target_include_directories(CAD_0_concurrency
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_BINARY_DIR}/include
)

target_link_libraries(CAD_0_concurrency PUBLIC CAD_0_math)

# Link TBB when enabled. TBB::tbb is an imported target created by
# oneTBB's CMake config. We check both names for robustness.
# PERF 3, audit finding #2: the SYSTEM-include treatment for oneTBB —
# the same cure core/sdf applies (see the block there for the finding,
# its precedent, and the ALIASED_TARGET resolution): oneTBB v2022.0.0's
# headers instantiate conversion warnings under the project's
# -Wconversion that surface as errors in first-party TUs; SYSTEM stops
# them, the code unchanged.
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
            if(NOT _tbb_aliased)
                target_link_libraries(CAD_0_concurrency PRIVATE ${_tbb_candidate})
            else()
                target_link_libraries(CAD_0_concurrency PRIVATE tbb)
            endif()
            break()
        endif()
    endforeach()
    target_compile_definitions(CAD_0_concurrency PRIVATE CAD_0_USE_TBB=1)
endif()

# When Python bindings are enabled, the GIL guard needs Python.h.
# find_package(Python3) is called once at the top-level CMakeLists.txt,
# so the Python3::Module target is available here.
if(CAD_0_BUILD_PYTHON)
    target_compile_definitions(CAD_0_concurrency PUBLIC CAD_0_BUILD_PYTHON=1)
    if(TARGET Python3::Module)
        target_link_libraries(CAD_0_concurrency PRIVATE Python3::Module)
    else()
        # Fallback: at least add the include dirs if the imported target
        # isn't available (shouldn't happen with modern CMake + Development.Module).
        target_include_directories(CAD_0_concurrency PRIVATE ${Python3_INCLUDE_DIRS})
    endif()
endif()

target_compile_features(CAD_0_concurrency PUBLIC cxx_std_20)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/include FILES ${CAD_0_CONCURRENCY_HEADERS})

install(TARGETS CAD_0_concurrency ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY include/CAD_0 DESTINATION include)
