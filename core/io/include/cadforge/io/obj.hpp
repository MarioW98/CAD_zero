// core/io/include/cadforge/io/obj.hpp
//
// OBJ mesh export — Wavefront .obj format.
//
// OBJ is the de-facto interchange format for 3D graphics. It supports:
//   * vertex positions (v x y z)
//   * vertex normals (vn x y z)
//   * faces (f v1//n1 v2//n2 v3//n3)  — vertex//normal indices
//
// We do NOT emit UV coordinates (texture mapping is out of scope for CAD).
// Material files (.mtl) are not generated; consumers can supply their own.
//
#pragma once

#include "cadforge/sdf/mesh_extract.hpp"

#include <string_view>

namespace cadforge::io {

// Write a triangle mesh to a Wavefront OBJ file.
// Returns true on success, false on I/O error.
bool export_obj(const std::string& path,
                const cadforge::sdf::TriangleMesh& mesh,
                std::string_view name = "cadforge");

} // namespace cadforge::io
