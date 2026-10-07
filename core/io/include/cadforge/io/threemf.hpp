// core/io/include/cadforge/io/threemf.hpp
//
// 3MF mesh export — minimal implementation of the 3D Manufacturing Format.
//
// 3MF is an XML-based format wrapped in a ZIP archive. It is the modern
// replacement for STL in additive manufacturing, supporting:
//   * Multiple parts per file
//   * Vertex normals
//   * Color/material references
//   * Metadata
//
// This is a *minimal* writer supporting the core model:
//   * Single object with triangle mesh
//   * Vertex positions + triangle indices
//   * Optional vertex normals (via "vertex" attributes)
//
// References:
//   * 3MF Specification: https://3mf.io/specification/
//   * XML namespace: http://schemas.microsoft.com/3dmanufacturing/core/2015/02
//
// The ZIP archive is created with no compression (store method) for
// simplicity. This produces larger files but is sufficient for the
// MVP — proper DEFLATE compression can be added later.
//
#pragma once

#include "cadforge/sdf/mesh_extract.hpp"

#include <string_view>

namespace cadforge::io {

// Write a triangle mesh to a 3MF file.
// Returns true on success, false on I/O error.
bool export_3mf(const std::string& path,
                const cadforge::sdf::TriangleMesh& mesh,
                std::string_view name = "cadforge",
                std::string_view application = "cadforge");

} // namespace cadforge::io
