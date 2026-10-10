# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/home/z/my-project/CAD_0/build_check/_deps/xsimd-src")
  file(MAKE_DIRECTORY "/home/z/my-project/CAD_0/build_check/_deps/xsimd-src")
endif()
file(MAKE_DIRECTORY
  "/home/z/my-project/CAD_0/build_check/_deps/xsimd-build"
  "/home/z/my-project/CAD_0/build_check/_deps/xsimd-subbuild/xsimd-populate-prefix"
  "/home/z/my-project/CAD_0/build_check/_deps/xsimd-subbuild/xsimd-populate-prefix/tmp"
  "/home/z/my-project/CAD_0/build_check/_deps/xsimd-subbuild/xsimd-populate-prefix/src/xsimd-populate-stamp"
  "/home/z/my-project/CAD_0/build_check/_deps/xsimd-subbuild/xsimd-populate-prefix/src"
  "/home/z/my-project/CAD_0/build_check/_deps/xsimd-subbuild/xsimd-populate-prefix/src/xsimd-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/z/my-project/CAD_0/build_check/_deps/xsimd-subbuild/xsimd-populate-prefix/src/xsimd-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/z/my-project/CAD_0/build_check/_deps/xsimd-subbuild/xsimd-populate-prefix/src/xsimd-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
