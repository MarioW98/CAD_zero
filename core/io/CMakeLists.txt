# core/io/CMakeLists.txt — Phase B.5 (STL + OBJ + 3MF export)

set(CAD_0_IO_HEADERS
    include/CAD_0/io/stl.hpp
    include/CAD_0/io/obj.hpp
    include/CAD_0/io/threemf.hpp
)

set(CAD_0_IO_SOURCES
    src/stl.cpp
    src/obj.cpp
    src/threemf.cpp
)

add_library(CAD_0_io STATIC ${CAD_0_IO_SOURCES} ${CAD_0_IO_HEADERS})

target_include_directories(CAD_0_io
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_BINARY_DIR}/include
        ${CMAKE_SOURCE_DIR}/core/math/include
        ${CMAKE_SOURCE_DIR}/core/geometry/include
        ${CMAKE_SOURCE_DIR}/core/sdf/include
)

target_link_libraries(CAD_0_io
    PUBLIC
        CAD_0_sdf
        CAD_0_math
)

target_compile_features(CAD_0_io PUBLIC cxx_std_20)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/include FILES ${CAD_0_IO_HEADERS})

install(TARGETS CAD_0_io ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY include/CAD_0 DESTINATION include)
