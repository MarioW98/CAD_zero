# core/concurrency/CMakeLists.txt

set(CADFORGE_CONCURRENCY_HEADERS
    include/cadforge/concurrency/thread_pool.hpp
    include/cadforge/concurrency/parallel_for.hpp
    include/cadforge/concurrency/gil_guard.hpp
    include/cadforge/concurrency/task_graph.hpp
)

set(CADFORGE_CONCURRENCY_SOURCES
    src/thread_pool.cpp
)

add_library(cadforge_concurrency STATIC ${CADFORGE_CONCURRENCY_SOURCES} ${CADFORGE_CONCURRENCY_HEADERS})

target_include_directories(cadforge_concurrency
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_BINARY_DIR}/include
)

target_link_libraries(cadforge_concurrency PUBLIC cadforge_math)

if(TBB::tbb)
    target_link_libraries(cadforge_concurrency PRIVATE TBB::tbb)
    target_compile_definitions(cadforge_concurrency PRIVATE CADFORGE_USE_TBB=1)
endif()

if(CADFORGE_BUILD_PYTHON)
    target_compile_definitions(cadforge_concurrency PUBLIC CADFORGE_BUILD_PYTHON=1)
    # We need Python.h — but we don't link against libpython directly
    # (nanobind does that for us). Find_Python3 is enough for headers.
    find_package(Python3 COMPONENTS Development.Module REQUIRED)
    target_link_libraries(cadforge_concurrency PRIVATE Python3::Module)
endif()

target_compile_features(cadforge_concurrency PUBLIC cxx_std_20)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/include FILES ${CADFORGE_CONCURRENCY_HEADERS})

install(TARGETS cadforge_concurrency ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY include/cadforge DESTINATION include)
