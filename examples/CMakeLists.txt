# examples/CMakeLists.txt

add_executable(example_sphere_to_stl cpp/sphere_to_stl.cpp)

target_link_libraries(example_sphere_to_stl PRIVATE
    cadforge_math
    cadforge_geometry
    cadforge_sdf
    cadforge_io
)

target_include_directories(example_sphere_to_stl PRIVATE
    ${CMAKE_SOURCE_DIR}/core/math/include
    ${CMAKE_SOURCE_DIR}/core/geometry/include
    ${CMAKE_SOURCE_DIR}/core/sdf/include
    ${CMAKE_SOURCE_DIR}/core/io/include
)
