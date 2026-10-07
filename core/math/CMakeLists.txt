# core/math/CMakeLists.txt

set(CAD_0_MATH_HEADERS
    include/CAD_0/math/vec.hpp
    include/CAD_0/math/mat.hpp
    include/CAD_0/math/quat.hpp
    include/CAD_0/math/interval.hpp
    include/CAD_0/math/bbox.hpp
    include/CAD_0/math/tolerance.hpp
    include/CAD_0/math/predicate.hpp
)

set(CAD_0_MATH_SOURCES
    src/tolerance.cpp
    src/predicate.cpp
)

# Vendored Shewchuk predicates — public domain.
set(ROBUST_PREDICATES_DIR
    ${CMAKE_SOURCE_DIR}/third_party/robust_predicates)

add_library(CAD_0_math STATIC ${CAD_0_MATH_SOURCES} ${CAD_0_MATH_HEADERS})

target_include_directories(CAD_0_math
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/third_party/robust_predicates
    PRIVATE
        ${CMAKE_BINARY_DIR}/include
)

target_compile_features(CAD_0_math PUBLIC cxx_std_20)

# Headers -> IDE grouping
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR}/include FILES ${CAD_0_MATH_HEADERS})

# Install rule for downstream consumers
install(TARGETS CAD_0_math
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)
install(DIRECTORY include/CAD_0 DESTINATION include)
