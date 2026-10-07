# core/math/CMakeLists.txt

set(CADFORGE_MATH_HEADERS
    include/cadforge/math/vec.hpp
    include/cadforge/math/mat.hpp
    include/cadforge/math/quat.hpp
    include/cadforge/math/interval.hpp
    include/cadforge/math/bbox.hpp
    include/cadforge/math/tolerance.hpp
    include/cadforge/math/predicate.hpp
)

set(CADFORGE_MATH_SOURCES
    src/tolerance.cpp
    src/predicate.cpp
)

# Vendored Shewchuk predicates — public domain.
set(ROBUST_PREDICATES_DIR
    ${CMAKE_SOURCE_DIR}/third_party/robust_predicates)

add_library(cadforge_math STATIC ${CADFORGE_MATH_SOURCES} ${CADFORGE_MATH_HEADERS})

target_include_directories(cadforge_math
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/third_party/robust_predicates
    PRIVATE
        ${CMAKE_BINARY_DIR}/include
)

target_compile_features(cadforge_math PUBLIC cxx_std_20)

# Headers -> IDE grouping
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/include FILES ${CADFORGE_MATH_HEADERS})

# Install rule for downstream consumers
install(TARGETS cadforge_math
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)
install(DIRECTORY include/cadforge DESTINATION include)
