# core/io/CMakeLists.txt — Phase B.5 (STL + OBJ + 3MF export)

set(CADFORGE_IO_HEADERS
    include/cadforge/io/stl.hpp
    include/cadforge/io/obj.hpp
    include/cadforge/io/threemf.hpp
)

set(CADFORGE_IO_SOURCES
    src/stl.cpp
    src/obj.cpp
    src/threemf.cpp
)

add_library(cadforge_io STATIC ${CADFORGE_IO_SOURCES} ${CADFORGE_IO_HEADERS})

target_include_directories(cadforge_io
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_BINARY_DIR}/include
        ${CMAKE_SOURCE_DIR}/core/math/include
        ${CMAKE_SOURCE_DIR}/core/geometry/include
        ${CMAKE_SOURCE_DIR}/core/sdf/include
)

target_link_libraries(cadforge_io
    PUBLIC
        cadforge_sdf
        cadforge_math
)

target_compile_features(cadforge_io PUBLIC cxx_std_20)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/include FILES ${CADFORGE_IO_HEADERS})

install(TARGETS cadforge_io ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY include/cadforge DESTINATION include)
