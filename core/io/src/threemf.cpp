// core/io/src/threemf.cpp
//
// Minimal 3MF writer (XML + uncompressed ZIP wrapper).
//
// The 3MF format is XML wrapped in a ZIP archive. We produce:
//   - "[Content_Types].xml"  — declares the file's content types
//   - "_rels/.rels"          — top-level relationships
//   - "3D/3dmodel.model"    — the actual 3D model XML
//
// The ZIP uses the "store" method (no compression) for simplicity.
// CRC32 is computed for each entry because the ZIP format requires it.
//
#include "CAD_0/io/threemf.hpp"

#include <CAD_0/math/vec.hpp>

#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>

namespace CAD_0::io {

namespace {

// CRC32 (IEEE 802.3 polynomial 0xEDB88320) — required by the ZIP format.
// Implementation table-driven, public-domain style.
std::uint32_t crc32(const unsigned char* data, std::size_t len) {
    static std::uint32_t table[256];
    static bool init = false;
    if (!init) {
        for (std::uint32_t i = 0; i < 256; ++i) {
            std::uint32_t c = i;
            for (int k = 0; k < 8; ++k) {
                c = (c & 1) ? (0xEDB88320U ^ (c >> 1)) : (c >> 1);
            }
            table[i] = c;
        }
        init = true;
    }
    std::uint32_t crc = 0xFFFFFFFFU;
    for (std::size_t i = 0; i < len; ++i) {
        crc = table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFFU;
}

// Pack a 32-bit little-endian value into 4 bytes.
void put_u32_le(std::vector<unsigned char>& out, std::uint32_t v) {
    out.push_back(static_cast<unsigned char>(v & 0xFF));
    out.push_back(static_cast<unsigned char>((v >> 8) & 0xFF));
    out.push_back(static_cast<unsigned char>((v >> 16) & 0xFF));
    out.push_back(static_cast<unsigned char>((v >> 24) & 0xFF));
}

void put_u16_le(std::vector<unsigned char>& out, std::uint16_t v) {
    out.push_back(static_cast<unsigned char>(v & 0xFF));
    out.push_back(static_cast<unsigned char>((v >> 8) & 0xFF));
}

// Add one entry to the ZIP archive.
// `data` is the uncompressed content; we store it without compression.
struct ZipEntry {
    std::string name;
    std::vector<unsigned char> data;
    std::uint32_t crc;
    std::uint32_t offset;       // offset of local header within the archive
};

void add_zip_entry(std::vector<unsigned char>& out,
                    std::vector<ZipEntry>& entries,
                    const std::string& name,
                    const std::string& content) {
    ZipEntry e;
    e.name = name;
    e.data.assign(content.begin(), content.end());
    e.crc = crc32(e.data.data(), e.data.size());
    e.offset = static_cast<std::uint32_t>(out.size());

    // Local file header signature.
    put_u32_le(out, 0x04034b50);
    put_u16_le(out, 20);               // version needed to extract
    put_u16_le(out, 0);                 // general purpose bit flag
    put_u16_le(out, 0);                 // compression method = store
    put_u16_le(out, 0);                 // last mod file time
    put_u16_le(out, 0);                 // last mod file date
    put_u32_le(out, e.crc);             // CRC-32
    put_u32_le(out, static_cast<std::uint32_t>(e.data.size()));  // compressed size
    put_u32_le(out, static_cast<std::uint32_t>(e.data.size()));  // uncompressed size
    put_u16_le(out, static_cast<std::uint16_t>(name.size()));    // filename length
    put_u16_le(out, 0);                 // extra field length
    out.insert(out.end(), name.begin(), name.end());
    out.insert(out.end(), e.data.begin(), e.data.end());

    entries.push_back(std::move(e));
}

void write_central_directory(std::vector<unsigned char>& out,
                              const std::vector<ZipEntry>& entries) {
    const std::uint32_t cd_start = static_cast<std::uint32_t>(out.size());

    for (const auto& e : entries) {
        // Central directory file header signature.
        put_u32_le(out, 0x02014b50);
        put_u16_le(out, 20);               // version made by
        put_u16_le(out, 20);               // version needed to extract
        put_u16_le(out, 0);                 // general purpose bit flag
        put_u16_le(out, 0);                 // compression method = store
        put_u16_le(out, 0);                 // last mod file time
        put_u16_le(out, 0);                 // last mod file date
        put_u32_le(out, e.crc);
        put_u32_le(out, static_cast<std::uint32_t>(e.data.size()));
        put_u32_le(out, static_cast<std::uint32_t>(e.data.size()));
        put_u16_le(out, static_cast<std::uint16_t>(e.name.size()));
        put_u16_le(out, 0);                 // extra field length
        put_u16_le(out, 0);                 // file comment length
        put_u16_le(out, 0);                 // disk number start
        put_u16_le(out, 0);                 // internal file attributes
        put_u32_le(out, 0);                 // external file attributes
        put_u32_le(out, e.offset);          // relative offset of local header
        out.insert(out.end(), e.name.begin(), e.name.end());
    }

    const std::uint32_t cd_size = static_cast<std::uint32_t>(out.size()) - cd_start;

    // End of central directory record.
    put_u32_le(out, 0x06054b50);
    put_u16_le(out, 0);                                              // disk number
    put_u16_le(out, 0);                                              // disk with CD
    put_u16_le(out, static_cast<std::uint16_t>(entries.size()));  // entries on this disk
    put_u16_le(out, static_cast<std::uint16_t>(entries.size()));  // total entries
    put_u32_le(out, cd_size);                                       // size of CD
    put_u32_le(out, cd_start);                                       // offset of CD
    put_u16_le(out, 0);                                              // comment length
}

std::string build_3dmodel_xml(const CAD_0::sdf::TriangleMesh& mesh,
                               std::string_view name,
                               std::string_view application) {
    std::ostringstream xml;
    xml << std::setprecision(6) << std::scientific;

    xml << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    xml << "<model unit=\"millimeter\" "
           "xml:lang=\"en-US\" "
           "xmlns=\"http://schemas.microsoft.com/3dmanufacturing/core/2015/02\" "
           "xmlns:m=\"http://schemas.microsoft.com/3dmanufacturing/material/2015/02\">\n";

    xml << "  <metadata name=\"Title\">" << name << "</metadata>\n";
    xml << "  <metadata name=\"Application\">" << application << "</metadata>\n";
    xml << "  <metadata name=\"GenerationDate\">2025-10-07</metadata>\n";

    xml << "  <resources>\n";
    xml << "    <object id=\"1\" type=\"model\">\n";
    xml << "      <mesh>\n";

    // Vertices.
    xml << "        <vertices>\n";
    for (const auto& p : mesh.positions) {
        xml << "          <vertex x=\"" << p.x << "\" y=\"" << p.y << "\" z=\"" << p.z << "\"/>\n";
    }
    xml << "        </vertices>\n";

    // Triangles.
    xml << "        <triangles>\n";
    for (std::size_t t = 0; t < mesh.triangle_count(); ++t) {
        const std::uint32_t v1 = mesh.indices[t * 3 + 0];
        const std::uint32_t v2 = mesh.indices[t * 3 + 1];
        const std::uint32_t v3 = mesh.indices[t * 3 + 2];
        xml << "          <triangle v1=\"" << v1 << "\" v2=\"" << v2 << "\" v3=\"" << v3 << "\"/>\n";
    }
    xml << "        </triangles>\n";

    xml << "      </mesh>\n";
    xml << "    </object>\n";
    xml << "  </resources>\n";

    xml << "  <build>\n";
    xml << "    <item objectid=\"1\" transform=\"1 0 0 0 1 0 0 0 1 0 0 0\"/>\n";
    xml << "  </build>\n";
    xml << "</model>\n";
    return xml.str();
}

std::string build_content_types_xml() {
    return std::string(
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
        "<Types xmlns=\"http://schemas.openxmlformats.org/package/2006/content-types\">\n"
        "  <Default Extension=\"rels\" ContentType=\"application/vnd.openxmlformats-package.relationships+xml\"/>\n"
        "  <Default Extension=\"model\" ContentType=\"application/vnd.ms-package.3dmanufacturing-3dmodel+xml\"/>\n"
        "</Types>\n"
    );
}

std::string build_rels_xml() {
    return std::string(
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
        "<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">\n"
        "  <Relationship Target=\"/3D/3dmodel.model\" Id=\"rel0\" "
        "Type=\"http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel\"/>\n"
        "</Relationships>\n"
    );
}

} // namespace

bool export_3mf(const std::string& path,
                const CAD_0::sdf::TriangleMesh& mesh,
                std::string_view name,
                std::string_view application) {
    if (mesh.triangle_count() == 0) return false;

    std::vector<unsigned char> zip;
    std::vector<ZipEntry> entries;

    add_zip_entry(zip, entries, "[Content_Types].xml", build_content_types_xml());
    add_zip_entry(zip, entries, "_rels/.rels", build_rels_xml());
    add_zip_entry(zip, entries, "3D/3dmodel.model",
                   build_3dmodel_xml(mesh, name, application));
    write_central_directory(zip, entries);

    std::ofstream f(path, std::ios::binary);
    if (!f) return false;
    f.write(reinterpret_cast<const char*>(zip.data()), zip.size());
    return static_cast<bool>(f);
}

} // namespace CAD_0::io
