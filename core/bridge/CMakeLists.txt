# core/bridge/CMakeLists.txt

set(CAD_0_BRIDGE_HEADERS
    include/CAD_0/bridge/sdf_to_brep.hpp
    include/CAD_0/bridge/brep_to_sdf.hpp
)

set(CAD_0_BRIDGE_SOURCES
    src/sdf_to_brep.cpp
    src/brep_to_sdf.cpp
)

add_library(CAD_0_bridge STATIC ${CAD_0_BRIDGE_SOURCES} ${CAD_0_BRIDGE_HEADERS})

target_include_directories(CAD_0_bridge
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_BINARY_DIR}/include
)

target_link_libraries(CAD_0_bridge
    PUBLIC
        CAD_0_math
        CAD_0_sdf
        CAD_0_brep
)

target_compile_features(CAD_0_bridge PUBLIC cxx_std_20)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/include FILES ${CAD_0_BRIDGE_HEADERS})

install(TARGETS CAD_0_bridge ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY include/CAD_0 DESTINATION include)
