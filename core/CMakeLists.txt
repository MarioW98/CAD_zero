# core/CMakeLists.txt — top-level core aggregation

add_subdirectory(math)
add_subdirectory(geometry)
add_subdirectory(sdf)
add_subdirectory(brep)
add_subdirectory(io)

# Concurrency is added when CADFORGE_USE_TBB is set; otherwise a header-only
# scalar fallback is provided.
add_subdirectory(concurrency)

# Bridge and modeling land in later phases.
if(CADFORGE_BUILD_BREP)
    add_subdirectory(bridge)
    add_subdirectory(modeling)
endif()

# Convenience aggregate target for downstream consumers.
add_library(cadforge_core INTERFACE)
target_link_libraries(cadforge_core INTERFACE
    cadforge_math
    cadforge_geometry
    cadforge_sdf
    cadforge_concurrency
    cadforge_brep
    cadforge_io
)
