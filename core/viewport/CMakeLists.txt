# core/viewport/CMakeLists.txt — Phase C viewport module

set(CAD_0_VIEWPORT_HEADERS
    include/CAD_0/viewport/camera.hpp
    include/CAD_0/viewport/command_stack.hpp
    include/CAD_0/viewport/selection.hpp
    include/CAD_0/viewport/renderer.hpp
    include/CAD_0/viewport/ray_march.hpp
    include/CAD_0/viewport/opengl_renderer.hpp
    # GLSL shaders — installed as data files
    include/CAD_0/viewport/glsl/sdf_raymarch.vert
    include/CAD_0/viewport/glsl/sdf_raymarch.frag
)

set(CAD_0_VIEWPORT_SOURCES
    src/renderer.cpp
    src/ray_march.cpp
    src/opengl_renderer.cpp
)

option(CAD_0_BUILD_OPENGL "Build the OpenGL 4.5 renderer (requires GL context)" OFF)

add_library(CAD_0_viewport STATIC ${CAD_0_VIEWPORT_SOURCES} ${CAD_0_VIEWPORT_HEADERS})

target_include_directories(CAD_0_viewport
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(CAD_0_viewport
    PUBLIC
        CAD_0_math
        CAD_0_geometry
        CAD_0_sdf
)

if(CAD_0_BUILD_OPENGL)
    target_compile_definitions(CAD_0_viewport PUBLIC CAD_0_BUILD_OPENGL=1)
    # In a real build we would find_package(glad) here.
endif()

target_compile_features(CAD_0_viewport PUBLIC cxx_std_20)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/include FILES ${CAD_0_VIEWPORT_HEADERS})

install(TARGETS CAD_0_viewport ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY include/CAD_0 DESTINATION include)
install(DIRECTORY include/CAD_0/viewport/glsl DESTINATION share/CAD_0/glsl)
