// core/io/include/CAD_0/io/obj.hpp
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

#include "CAD_0/sdf/mesh_extract.hpp"

#include <string_view>

namespace CAD_0::io {

// Write a triangle mesh to a Wavefront OBJ file.
// Returns true on success, false on I/O error.
bool export_obj(const std::string& path,
                const CAD_0::sdf::TriangleMesh& mesh,
                std::string_view name = "CAD_0");

// ---- Import (OBJ loader) ----------------------------------------------------
//
// Load a triangle mesh from a Wavefront OBJ file.
//
// Supported: vertex positions (v), vertex normals (vn), faces (f) with
// corner formats v, v/vt, v//vn, v/vt/vn — negative indices (relative,
// OBJ 1-based) resolved per array; n-gons triangulated as a FAN
// (c0, ci, ci+1). Texture coordinates (vt) are PARSED and discarded:
// CAD_0 has no UV support (see the export note above); their indices are
// still validated and still resolve for relative indexing.
//
// Organizational statements (o, g, s, usemtl, mtllib, comments) are
// skipped — they carry no geometry. Any other keyword (curv, surf, vp,
// ...) is an EXPLICIT error: those files carry geometry we cannot
// represent, and silently dropping it would corrupt the import.
//
// OBJ indices are shared, so the mesh is assembled position-indexed
// (positions in v-declaration order) and then passed through
// sdf::weld_vertices (ADR-0016) like the STL loader — on a well-formed
// OBJ the weld is an idempotent no-op (the guarantee is tested); on
// files with duplicate v declarations it deduplicates them. Normals are
// stored per position, first reference wins (OBJ allows a position to
// be referenced with different vn per corner; CAD_0's mesh model has
// one normal per vertex).
//
// Errors: std::runtime_error with the path and line number on I/O
// failure, out-of-range/zero indices, faces with fewer than 3 corners,
// malformed numbers, or unsupported keywords — explicit, never partial.
CAD_0::sdf::TriangleMesh load_obj(const std::string& path);

} // namespace CAD_0::io
