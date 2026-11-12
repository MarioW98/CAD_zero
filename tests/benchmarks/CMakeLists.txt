# tests/benchmarks/CMakeLists.txt — K1 m8: the benchmark suite's own
# home, SEPARATE from the unit tests (ADR-0041 D5 — the performance
# twin of the census: capabilities get columns, costs get NUMBERS).
#
# THE POSTURE, declared: the target is BUILT in every column of the
# rite (the compile discipline — the same flags the unit suite
# carries, -Werror among them, so a warning here is a red anywhere),
# and it is NOT a ctest member — deliberately. The bench measures
# TIMES and gates on their ratios; the shared CI runner is not the
# measuring machine (the register's m8 row: the absolute numbers are
# indicative, the ratios are the substance). The suite runs by the
# rite — on the measuring machine, both compilers, the same presets
# the CI builds — and its exit code is a real check when run: the
# digests, the witnesses, the gate and both counterproof reds must
# all hold or the bench says so and fails. Registering it under
# ctest would put a timing gate on machines whose noise nobody
# pinned; the maintainer registers it the day he wants that, his
# call, his push.

add_executable(CAD_0_benchmarks bench_k1_m8.cpp)

target_include_directories(CAD_0_benchmarks PRIVATE
    ${CMAKE_SOURCE_DIR}/core/math/include
    ${CMAKE_SOURCE_DIR}/core/geometry/include
    ${CMAKE_SOURCE_DIR}/core/sdf/include
    ${CMAKE_SOURCE_DIR}/core/concurrency/include
    ${CMAKE_SOURCE_DIR}/core/brep/include
    ${CMAKE_SOURCE_DIR}/core/scene/include
    ${CMAKE_SOURCE_DIR}/core/io/include
    ${CMAKE_BINARY_DIR}/include
)

target_link_libraries(CAD_0_benchmarks PRIVATE
    CAD_0_math
    CAD_0_geometry
    CAD_0_sdf
    CAD_0_concurrency
    CAD_0_scene
    CAD_0_io
    CAD_0_brep
)

# The build type rides the binary (the run's first lines declare
# what it was built as — the machine context is the report's own
# discipline, and the bench prints its half of it).
target_compile_definitions(CAD_0_benchmarks PRIVATE
    CAD_0_BENCH_BUILD_TYPE="${CMAKE_BUILD_TYPE}"
)

# The kernel compile discipline, exactly the unit suite's own (the
# bench compiles the kernel's headers: a warning here is a red
# anywhere). GNU/Clang only.
target_compile_options(CAD_0_benchmarks PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Werror>)
