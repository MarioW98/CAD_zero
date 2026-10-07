// tests/main.cpp
//
// The doctest framework needs a single translation unit with
// DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN defined. We do that here so the
// per-module test files can simply `#include <doctest/doctest.h>`
// without re-defining the macro.
//
// (See tests/CMakeLists.txt: CADFORGE_TEST_SOURCES include this file
// last; the macro is defined via target_compile_definitions.)
//
#include <doctest/doctest.h>
