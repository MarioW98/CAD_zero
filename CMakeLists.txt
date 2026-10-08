cmake_minimum_required(VERSION 3.22)

# ---------------------------------------------------------------------------
# Top-level CMake project for CAD_0
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
project(CAD_0
    VERSION 0.1.0
    DESCRIPTION "CAD_0 — dual-representation CAD kernel (native SDF + B-Rep)"
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

option(CAD_0_BUILD_TESTING  "Build unit + regression tests" ON)
option(CAD_0_BUILD_BENCHMARKS "Build benchmarks" OFF)
option(CAD_0_BUILD_PYTHON   "Build Python bindings (nanobind)" ON)
option(CAD_0_BUILD_EXAMPLES "Build C++ examples" ON)
option(CAD_0_SANITIZE       "Enable ASan/UBSan (debug builds only)" OFF)
# TBB is OFF by default because the CPM-fetched oneTBB headers don't always
# end up on the include path of downstream targets (depends on the CMake
# generator and the imported target's INTERFACE_INCLUDE_DIRECTORIES).
# The build works perfectly without TBB (single-threaded fallback).
# Enable explicitly with -DCAD_0_USE_TBB=ON if you need parallelism and
# have oneTBB properly installed system-wide.
option(CAD_0_USE_TBB        "Use oneTBB for thread pool" OFF)

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
# v2.4.12 fixes CMake 4.x compatibility (cmake_minimum_required bumped to 3.5)
if(CAD_0_BUILD_TESTING)
    cpmaddpackage(
        NAME doctest
        GITHUB_REPOSITORY doctest/doctest
        GIT_TAG v2.4.12
    )
endif()

# xsimd — SIMD abstraction layer (AVX2/AVX-512/NEON/scalar)
cpmaddpackage(
    NAME xsimd
    GITHUB_REPOSITORY xtensor-stack/xsimd
    GIT_TAG 12.1.1
)

# nanobind — Python bindings (replaces pybind11, see ADR-0011)
if(CAD_0_BUILD_PYTHON)
    cpmaddpackage(
        NAME nanobind
        GITHUB_REPOSITORY wjakob/nanobind
        GIT_TAG v2.2.0
    )
endif()

# oneTBB — thread pool + task graph (see ADR-0009)
# v2022.0.0 fixes GCC 12+ -Werror false positives on atomic_base.h
# (stringop-overflow). Older versions (v2021.x) fail on GCC 14.
if(CAD_0_USE_TBB)
    cpmaddpackage(
        NAME TBB
        GITHUB_REPOSITORY oneapi-src/oneTBB
        GIT_TAG v2022.0.0
        OPTIONS
            "TBB_TEST OFF"
            "TBB_STRICT OFF"
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
    if(CAD_0_SANITIZE)
        add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
        add_link_options(-fsanitize=address,undefined)
    endif()
elseif(MSVC)
    add_compile_options(/W4 /permissive- /Wv:19.40 /Zc:__cplusplus /utf-8)
    if(CAD_0_SANITIZE)
        message(WARNING "ASan/UBSan not natively supported on MSVC — skipped")
    endif()
endif()

# ---------------------------------------------------------------------------
# Subdirectories
# ---------------------------------------------------------------------------

# Find Python3 once at the top level so all subdirectories can use the
# Python3::Module imported target. We use Development.Module (not
# Development) because we only need Python.h, not the full libpython
# (nanobind links the libpython it needs).
#
# Pass -DPython3_EXECUTABLE=/path/to/python to CMake to pick a specific
# interpreter (e.g. when using a venv or uv-managed Python).
if(CAD_0_BUILD_PYTHON)
    find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)
    message(STATUS "Python3_EXECUTABLE:  ${Python3_EXECUTABLE}")
    message(STATUS "Python3_INCLUDE_DIRS: ${Python3_INCLUDE_DIRS}")
    message(STATUS "Python3_VERSION:     ${Python3_VERSION}")
endif()

add_subdirectory(core)
if(CAD_0_BUILD_PYTHON)
    add_subdirectory(bindings/python)
endif()
if(CAD_0_BUILD_TESTING)
    enable_testing()
    add_subdirectory(tests)
endif()
if(CAD_0_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()
# Phase C: install the PySide6 desktop UI package. This is a pure-Python
# addition; no C++ is built here.
if(CAD_0_BUILD_PYTHON)
    add_subdirectory(app)
endif()

# ---------------------------------------------------------------------------
# Config header — exposes version + feature flags
# ---------------------------------------------------------------------------
configure_file(
    ${CMAKE_SOURCE_DIR}/cmake/CAD_0_config.h.in
    ${CMAKE_BINARY_DIR}/include/CAD_0/config.h
    @ONLY
)

message(STATUS "")
message(STATUS "CAD_0 ${PROJECT_VERSION}")
message(STATUS "  C++ standard     : ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build type       : ${CMAKE_BUILD_TYPE}")
message(STATUS "  Python bindings  : ${CAD_0_BUILD_PYTHON}")
message(STATUS "  Testing          : ${CAD_0_BUILD_TESTING}")
message(STATUS "  TBB              : ${CAD_0_USE_TBB}")
message(STATUS "  Sanitizers       : ${CAD_0_SANITIZE}")
message(STATUS "")
