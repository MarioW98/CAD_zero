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

if(TBB::tbb)
    target_link_libraries(CAD_0_concurrency PRIVATE TBB::tbb)
    target_compile_definitions(CAD_0_concurrency PRIVATE CAD_0_USE_TBB=1)
endif()

if(CAD_0_BUILD_PYTHON)
    target_compile_definitions(CAD_0_concurrency PUBLIC CAD_0_BUILD_PYTHON=1)
    # We need Python.h — but we don't link against libpython directly
    # (nanobind does that for us). Find_Python3 is enough for headers.
    find_package(Python3 COMPONENTS Development.Module REQUIRED)
    target_link_libraries(CAD_0_concurrency PRIVATE Python3::Module)
endif()

target_compile_features(CAD_0_concurrency PUBLIC cxx_std_20)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/include FILES ${CAD_0_CONCURRENCY_HEADERS})

install(TARGETS CAD_0_concurrency ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY include/CAD_0 DESTINATION include)
