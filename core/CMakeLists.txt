# core/CMakeLists.txt — top-level core aggregation

add_subdirectory(math)
add_subdirectory(geometry)
add_subdirectory(sdf)
add_subdirectory(brep)
add_subdirectory(io)
add_subdirectory(viewport)
add_subdirectory(scene)
add_subdirectory(bridge)

# Concurrency is added when CAD_0_USE_TBB is set; otherwise a header-only
# scalar fallback is provided.
add_subdirectory(concurrency)

# Bridge and modeling land in later phases.
if(CAD_0_BUILD_BREP)
    add_subdirectory(bridge)
    add_subdirectory(modeling)
endif()

# Convenience aggregate target for downstream consumers.
add_library(CAD_0_core INTERFACE)
target_link_libraries(CAD_0_core INTERFACE
    CAD_0_math
    CAD_0_geometry
    CAD_0_sdf
    CAD_0_concurrency
    CAD_0_brep
    CAD_0_io
    CAD_0_viewport
    CAD_0_scene
    CAD_0_bridge
)
