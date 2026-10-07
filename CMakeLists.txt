cmake_minimum_required(VERSION 3.22)

# ---------------------------------------------------------------------------
# Top-level CMake project for cadforge
# ---------------------------------------------------------------------------
#
# Build phases:
#   * core library (C++20) — math, geometry, sdf, concurrency, brep, bridge
#   * python bindings via nanobind (Phase A)
#   * unit tests via doctest (optional, BUILD_TESTING=ON)
#
# This file sets up only the core library + bindings. The desktop app
# (PySide6) lives entirely in Python and is invoked via `pip install`.
#
# ---------------------------------------------------------------------------
project(cadforge
    VERSION 0.1.0
    DESCRIPTION "CADForge — dual-representation CAD kernel (native SDF + B-Rep)"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Determinism: disable FP contraction so that the same model produces the
# same STEP file on Linux, Windows, and macOS. See ADR-0014.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-ffp-contract=off)
elseif(MSVC)
    add_compile_options(/fp:strict)
endif()

# Position independent code for shared libs and bindings.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Output to a single bin/ for easier discovery.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Default to RelWithDebInfo if nothing was specified.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

option(CADFORGE_BUILD_TESTING  "Build unit + regression tests" ON)
option(CADFORGE_BUILD_BENCHMARKS "Build benchmarks" OFF)
option(CADFORGE_BUILD_PYTHON   "Build Python bindings (nanobind)" ON)
option(CADFORGE_BUILD_EXAMPLES "Build C++ examples" ON)
option(CADFORGE_SANITIZE       "Enable ASan/UBSan (debug builds only)" OFF)
option(CADFORGE_USE_TBB        "Use oneTBB for thread pool" ON)

# ---------------------------------------------------------------------------
# Dependencies — fetched via CPM.cmake for reproducibility
# ---------------------------------------------------------------------------
include(${CMAKE_SOURCE_DIR}/cmake/CPM.cmake)

# fmt — fast logging + formatting
cpmaddpackage(
    NAME fmt
    GITHUB_REPOSITORY fmtlib/fmt
    GIT_TAG 10.2.1
)

# spdlog — async logging
cpmaddpackage(
    NAME spdlog
    GITHUB_REPOSITORY gabime/spdlog
    GIT_TAG v1.14.1
)

# doctest — unit testing
if(CADFORGE_BUILD_TESTING)
    cpmaddpackage(
        NAME doctest
        GITHUB_REPOSITORY doctest/doctest
        GIT_TAG v2.4.11
    )
endif()

# xsimd — SIMD abstraction layer (AVX2/AVX-512/NEON/scalar)
cpmaddpackage(
    NAME xsimd
    GITHUB_REPOSITORY xtensor-stack/xsimd
    GIT_TAG 12.1.1
)

# nanobind — Python bindings (replaces pybind11, see ADR-0011)
if(CADFORGE_BUILD_PYTHON)
    cpmaddpackage(
        NAME nanobind
        GITHUB_REPOSITORY wjakob/nanobind
        GIT_TAG v2.2.0
    )
endif()

# oneTBB — thread pool + task graph (see ADR-0009)
if(CADFORGE_USE_TBB)
    cpmaddpackage(
        NAME TBB
        GITHUB_REPOSITORY oneapi-src/oneTBB
        GIT_TAG v2021.13.0
    )
endif()

# Robust predicates (Shewchuk public domain) — vendored under third_party
# (no CPM needed, see third_party/README.md)

# ---------------------------------------------------------------------------
# Compiler warnings — strict but cross-platform
# ---------------------------------------------------------------------------
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(
        -Wall -Wextra -Wpedantic
        -Wshadow -Wnon-virtual-dtor
        -Wno-unused-parameter
        -Wconversion -Wno-sign-conversion
    )
    if(CADFORGE_SANITIZE)
        add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
        add_link_options(-fsanitize=address,undefined)
    endif()
elseif(MSVC)
    add_compile_options(/W4 /permissive- /Wv:19.40 /Zc:__cplusplus /utf-8)
    if(CADFORGE_SANITIZE)
        message(WARNING "ASan/UBSan not natively supported on MSVC — skipped")
    endif()
endif()

# ---------------------------------------------------------------------------
# Subdirectories
# ---------------------------------------------------------------------------
add_subdirectory(core)
if(CADFORGE_BUILD_PYTHON)
    add_subdirectory(bindings/python)
endif()
if(CADFORGE_BUILD_TESTING)
    enable_testing()
    add_subdirectory(tests)
endif()
if(CADFORGE_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# ---------------------------------------------------------------------------
# Config header — exposes version + feature flags
# ---------------------------------------------------------------------------
configure_file(
    ${CMAKE_SOURCE_DIR}/cmake/cadforge_config.h.in
    ${CMAKE_BINARY_DIR}/include/cadforge/config.h
    @ONLY
)

message(STATUS "")
message(STATUS "cadforge ${PROJECT_VERSION}")
message(STATUS "  C++ standard     : ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build type       : ${CMAKE_BUILD_TYPE}")
message(STATUS "  Python bindings  : ${CADFORGE_BUILD_PYTHON}")
message(STATUS "  Testing          : ${CADFORGE_BUILD_TESTING}")
message(STATUS "  TBB              : ${CADFORGE_USE_TBB}")
message(STATUS "  Sanitizers       : ${CADFORGE_SANITIZE}")
message(STATUS "")
