// core/io/include/CAD_0/io/stl.hpp
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

#include "CAD_0/sdf/mesh_extract.hpp"

#include <string>
#include <string_view>

namespace CAD_0::io {

// Write a triangle mesh to a binary STL file.
// Returns true on success, false on I/O error.
bool export_stl_binary(const std::string& path,
                       const CAD_0::sdf::TriangleMesh& mesh,
                       std::string_view name = "CAD_0");

// Write a triangle mesh to an ASCII STL file.
bool export_stl_ascii(const std::string& path,
                      const CAD_0::sdf::TriangleMesh& mesh,
                      std::string_view name = "CAD_0");

// ---- Import (STL loader) ----------------------------------------------------
//
// Load a triangle mesh from an STL file — binary OR ASCII, auto-detected.
//
// Format detection is DIMENSIONAL first (a binary file can also start with
// the word "solid"): a file whose size is exactly 84 + 50*n, where n is
// the uint32 little-endian triangle count at offset 80, is binary;
// anything else is tried as ASCII and must start with the "solid"
// keyword. Files that satisfy neither are a truncation/corruption error,
// never a guess.
//
// WELDING (ADR-0016): binary STL stores every triangle's vertices
// duplicated (3 per facet, shared corners repeated bit-exactly). The
// loader welds them back with the public sdf::weld_vertices utility
// (tolerance 1e-6 mesh units — merges the exact duplicates and anything
// closer than 1 um; far below any printable feature at mm scale). The
// weld is idempotent: already-welded meshes pass through unchanged.
//
// NORMALS: STL facet normals are advisory and in the wild frequently
// zero or wrong, so they are ignored — the loaded mesh carries NO
// normals. Consumers that need shading normals compute them from the
// geometry.
//
// Errors: std::runtime_error with the path (and line number for ASCII)
// in the message on I/O failure, truncation, or malformed content — the
// explicit-error discipline of the io module (never silently partial).
CAD_0::sdf::TriangleMesh load_stl(const std::string& path);

} // namespace CAD_0::io
