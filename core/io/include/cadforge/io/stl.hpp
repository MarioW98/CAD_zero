// core/io/include/cadforge/io/stl.hpp
//
// STL (stereolithography) mesh export — both binary and ASCII formats.
//
// Binary STL is the de-facto standard for 3D printing and CAM. It uses
// 50 bytes per triangle: 12 bytes normal, 36 bytes for 3 vertices (3 floats
// each), 2 bytes for an attribute byte count (always 0).
//
// ASCII STL is human-readable but ~6x larger. Useful for debugging.
//
#pragma once

#include "cadforge/sdf/mesh_extract.hpp"

#include <string>
#include <string_view>

namespace cadforge::io {

// Write a triangle mesh to a binary STL file.
// Returns true on success, false on I/O error.
bool export_stl_binary(const std::string& path,
                       const cadforge::sdf::TriangleMesh& mesh,
                       std::string_view name = "cadforge");

// Write a triangle mesh to an ASCII STL file.
bool export_stl_ascii(const std::string& path,
                      const cadforge::sdf::TriangleMesh& mesh,
                      std::string_view name = "cadforge");

} // namespace cadforge::io
