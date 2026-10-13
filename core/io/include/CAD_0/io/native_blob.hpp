// core/io/include/CAD_0/io/native_blob.hpp
//
// Binary blob container for the native .CAD_0 format — implements
// ADR-0007a (companion of ADR-0007).
//
// Layering: this header is the CODEC + FRAMING layer. It knows bytes:
//   * the container frame (magic "CAD0BLOB", lengths, JSON block, blob
//     section) and its auto-detection rules;
//   * the "raw" mesh blob encoding (float32/uint32, little-endian,
//     std::bit_cast — bit-exact round-trips by construction, ADR-0014).
// It does NOT know the document schema (that is native.hpp/native.cpp:
// the JSON "meshes" headers, migrators, scene nodes).
//
// Endianness: every multi-byte value is read/written byte-by-byte, so the
// codec produces identical bytes on big- and little-endian hosts.
//
// Errors: truncation, length inconsistencies, size/count mismatches and
// out-of-range indices are std::runtime_error with explicit messages —
// surfaced to the user (ADR-0007: fail loudly, never silently).
//
#pragma once

#include "CAD_0/sdf/mesh_extract.hpp"

#include <array>
#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

namespace CAD_0::io::native {

// ---- Container frame (ADR-0007a) --------------------------------------------

// 8-byte magic opening every v0.2.0 container.
inline constexpr std::array<char, 8> kContainerMagic{
    'C', 'A', 'D', '0', 'B', 'L', 'O', 'B'};

// Frame layout: [magic 8][json_len u32 LE][blob_len u32 LE][JSON][blob].
inline constexpr std::size_t kContainerHeaderSize = 16;

// True iff `file_bytes` starts with the container magic (the v0.2.0
// binary framing). Detection happens BEFORE any parsing.
bool has_container_magic(std::string_view file_bytes) noexcept;

// Build a full container file from its two blocks. Throws on a JSON block
// larger than 4 GiB (uint32 length field — see ADR-0007a).
std::string build_container(std::string_view json_text,
                            std::string_view blob_section);

// Split a container file into its JSON text and blob section. Throws
// std::runtime_error (explicit, with the sizes involved) when the file is
// shorter than the frame, the magic does not match, or the declared
// lengths do not add up to the actual file size (truncation/corruption).
struct ContainerParts {
    std::string_view json_text;
    std::string_view blob_section;
};
ContainerParts split_container(std::string_view file_bytes);

// ---- "raw" mesh blob encoding (ADR-0007a) -----------------------------------
//
// Payload layout (little-endian, tightly packed, in this order):
//   positions : float32 ×3 × vertex_count      (12 bytes/vertex)
//   normals   : float32 ×3 × vertex_count      (12 bytes/vertex, iff
//               has_normals — taken from `mesh.normals` being non-empty)
//   indices   : uint32  ×3 × triangle_count    (12 bytes/triangle)
//
// Floats travel as IEEE-754 bit patterns via std::bit_cast — no
// arithmetic on the codec path, so -0.0/denormals/NaNs round-trip
// bit-exactly (ADR-0014).

// Exact payload size for the given counts (the same formula the writer
// and the reader agree on — a mismatch is a load error).
std::size_t raw_blob_size(std::size_t vertex_count,
                          std::size_t triangle_count,
                          bool has_normals) noexcept;

// Encode a mesh into its "raw" blob payload.
std::string encode_blob_raw(const sdf::TriangleMesh& mesh);

// Decode a "raw" blob payload. `blob_id` names the mesh in error
// messages. Validates:
//   * `size == raw_blob_size(vertex_count, triangle_count, has_normals)`;
//   * every triangle index < vertex_count.
// Throws std::runtime_error with the id and the offending numbers.
sdf::TriangleMesh decode_blob_raw(std::string_view payload,
                                  const std::string& blob_id,
                                  std::size_t vertex_count,
                                  std::size_t triangle_count,
                                  bool has_normals);

} // namespace CAD_0::io::native
