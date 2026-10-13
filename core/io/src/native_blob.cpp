// core/io/src/native_blob.cpp
//
// Implementation of the binary blob container + "raw" mesh codec for the
// native .CAD_0 format (ADR-0007a). See native_blob.hpp for the layout
// and the layering rationale.
//
#include "CAD_0/io/native_blob.hpp"

#include <bit>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <string_view>

namespace CAD_0::io::native {

namespace {

// ---- little-endian scalars (byte-by-byte: endian-independent) ---------------

void write_u32_le(std::string& out, std::uint32_t v) {
    out.push_back(static_cast<char>(v & 0xFFu));
    out.push_back(static_cast<char>((v >> 8) & 0xFFu));
    out.push_back(static_cast<char>((v >> 16) & 0xFFu));
    out.push_back(static_cast<char>((v >> 24) & 0xFFu));
}

std::uint32_t read_u32_le(const char* p) {
    const auto b0 = static_cast<std::uint32_t>(static_cast<unsigned char>(p[0]));
    const auto b1 = static_cast<std::uint32_t>(static_cast<unsigned char>(p[1]));
    const auto b2 = static_cast<std::uint32_t>(static_cast<unsigned char>(p[2]));
    const auto b3 = static_cast<std::uint32_t>(static_cast<unsigned char>(p[3]));
    return b0 | (b1 << 8) | (b2 << 16) | (b3 << 24);
}

void write_f32_le(std::string& out, float v) {
    write_u32_le(out, std::bit_cast<std::uint32_t>(v));
}

float read_f32_le(const char* p) {
    return std::bit_cast<float>(read_u32_le(p));
}

// ---- error helper -----------------------------------------------------------

[[noreturn]] void blob_error(const std::string& blob_id, const std::string& what) {
    throw std::runtime_error("native format: mesh blob \"" + blob_id +
                             "\": " + what);
}

} // namespace

// ---- container frame ---------------------------------------------------------

bool has_container_magic(std::string_view file_bytes) noexcept {
    return file_bytes.size() >= kContainerMagic.size() &&
           std::memcmp(file_bytes.data(), kContainerMagic.data(),
                       kContainerMagic.size()) == 0;
}

std::string build_container(std::string_view json_text,
                            std::string_view blob_section) {
    constexpr auto kMax = std::numeric_limits<std::uint32_t>::max();
    if (json_text.size() > static_cast<std::size_t>(kMax) ||
        blob_section.size() > static_cast<std::size_t>(kMax))
        throw std::runtime_error(
            "native format: container block exceeds the 4 GiB uint32 "
            "length field (ADR-0007a) — json " +
            std::to_string(json_text.size()) + " bytes, blob " +
            std::to_string(blob_section.size()) + " bytes");

    std::string out;
    out.reserve(kContainerHeaderSize + json_text.size() + blob_section.size());
    out.append(kContainerMagic.data(), kContainerMagic.size());
    write_u32_le(out, static_cast<std::uint32_t>(json_text.size()));
    write_u32_le(out, static_cast<std::uint32_t>(blob_section.size()));
    out.append(json_text.data(), json_text.size());
    out.append(blob_section.data(), blob_section.size());
    return out;
}

ContainerParts split_container(std::string_view file_bytes) {
    if (!has_container_magic(file_bytes))
        throw std::runtime_error(
            "native format: not a CAD_0 container (bad magic) — expected \"" +
            std::string(kContainerMagic.data(), kContainerMagic.size()) + "\"");

    if (file_bytes.size() < kContainerHeaderSize)
        throw std::runtime_error(
            "native format: container truncated — " +
            std::to_string(file_bytes.size()) + " bytes, header alone is " +
            std::to_string(kContainerHeaderSize));

    const std::uint32_t json_len =
        read_u32_le(file_bytes.data() + kContainerMagic.size());
    const std::uint32_t blob_len =
        read_u32_le(file_bytes.data() + kContainerMagic.size() + 4);

    const std::size_t expected =
        kContainerHeaderSize + static_cast<std::size_t>(json_len) +
        static_cast<std::size_t>(blob_len);
    if (file_bytes.size() != expected)
        throw std::runtime_error(
            "native format: container lengths do not add up — declared " +
            std::to_string(expected) + " bytes (json " + std::to_string(json_len) +
            " + blob " + std::to_string(blob_len) + "), file is " +
            std::to_string(file_bytes.size()) + " bytes (truncated or corrupt)");

    const char* base = file_bytes.data();
    return ContainerParts{
        std::string_view(base + kContainerHeaderSize, json_len),
        std::string_view(base + kContainerHeaderSize + json_len, blob_len),
    };
}

// ---- "raw" mesh codec --------------------------------------------------------

std::size_t raw_blob_size(std::size_t vertex_count,
                          std::size_t triangle_count,
                          bool has_normals) noexcept {
    const std::size_t pos = vertex_count * 12;
    const std::size_t nrm = has_normals ? vertex_count * 12 : 0;
    const std::size_t idx = triangle_count * 12;
    return pos + nrm + idx;
}

std::string encode_blob_raw(const sdf::TriangleMesh& mesh) {
    const bool has_normals = !mesh.normals.empty();
    const std::size_t vertex_count = mesh.positions.size();
    const std::size_t triangle_count = mesh.triangle_count();

    std::string out;
    out.reserve(raw_blob_size(vertex_count, triangle_count, has_normals));

    for (const auto& p : mesh.positions) {
        write_f32_le(out, p.x);
        write_f32_le(out, p.y);
        write_f32_le(out, p.z);
    }
    if (has_normals) {
        for (const auto& n : mesh.normals) {
            write_f32_le(out, n.x);
            write_f32_le(out, n.y);
            write_f32_le(out, n.z);
        }
    }
    for (const std::uint32_t i : mesh.indices)
        write_u32_le(out, i);

    return out;
}

sdf::TriangleMesh decode_blob_raw(std::string_view payload,
                                  const std::string& blob_id,
                                  std::size_t vertex_count,
                                  std::size_t triangle_count,
                                  bool has_normals) {
    const std::size_t expected_size =
        raw_blob_size(vertex_count, triangle_count, has_normals);
    if (payload.size() != expected_size)
        blob_error(blob_id, "payload size mismatch — header implies " +
                                std::to_string(expected_size) +
                                " bytes for " + std::to_string(vertex_count) +
                                " vertices / " + std::to_string(triangle_count) +
                                " triangles / has_normals=" +
                                (has_normals ? "true" : "false") +
                                ", blob section holds " +
                                std::to_string(payload.size()));

    const char* p = payload.data();
    sdf::TriangleMesh mesh;
    mesh.positions.resize(vertex_count);
    for (std::size_t i = 0; i < vertex_count; ++i) {
        mesh.positions[i] = math::Vec3f{read_f32_le(p), read_f32_le(p + 4),
                                        read_f32_le(p + 8)};
        p += 12;
    }
    if (has_normals) {
        mesh.normals.resize(vertex_count);
        for (std::size_t i = 0; i < vertex_count; ++i) {
            mesh.normals[i] = math::Vec3f{read_f32_le(p), read_f32_le(p + 4),
                                          read_f32_le(p + 8)};
            p += 12;
        }
    }
    mesh.indices.resize(triangle_count * 3);
    for (std::size_t t = 0; t < triangle_count; ++t) {
        const std::uint32_t i0 = read_u32_le(p);
        const std::uint32_t i1 = read_u32_le(p + 4);
        const std::uint32_t i2 = read_u32_le(p + 8);
        if (i0 >= vertex_count || i1 >= vertex_count || i2 >= vertex_count)
            blob_error(blob_id, "triangle " + std::to_string(t) +
                                    " references vertex out of range (" +
                                    std::to_string(i0) + ", " + std::to_string(i1) +
                                    ", " + std::to_string(i2) +
                                    " vs vertex_count " +
                                    std::to_string(vertex_count) + ")");
        mesh.indices[t * 3 + 0] = i0;
        mesh.indices[t * 3 + 1] = i1;
        mesh.indices[t * 3 + 2] = i2;
        p += 12;
    }
    return mesh;
}

} // namespace CAD_0::io::native
