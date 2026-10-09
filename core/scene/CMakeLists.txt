# core/scene/CMakeLists.txt

set(CAD_0_SCENE_HEADERS
    include/CAD_0/scene/scene.hpp
)

add_library(CAD_0_scene INTERFACE ${CAD_0_SCENE_HEADERS})

target_include_directories(CAD_0_scene INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(CAD_0_scene INTERFACE
    CAD_0_math
    CAD_0_geometry
    CAD_0_sdf
    CAD_0_io
)

target_compile_features(CAD_0_scene INTERFACE cxx_std_20)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/include FILES ${CAD_0_SCENE_HEADERS})
