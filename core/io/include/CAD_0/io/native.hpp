// core/io/include/CAD_0/io/native.hpp
//
// Native project format (.CAD_0) — implements ADR-0007 + ADR-0007a.
//
// Format (v0.2.0):
//   * A binary container (ADR-0007a, codec in native_blob.hpp):
//     [magic "CAD0BLOB" | json_len u32 LE | blob_len u32 LE] + JSON block
//     + blob section. Files starting with '{' are legacy v0.1.0 pure-JSON
//     documents and load through the 0.1.0 -> 0.2.0 migrator.
//   * JSON for structure (in-house parser, see json.hpp — ADR-0003 forbids
//     new third-party deps without a companion ADR).
//   * Bulk data (imported mesh geometry) lives in the blob section as
//     "raw" blobs: float32/uint32, little-endian, bit-exact round-trips
//     (ADR-0014). The JSON carries one header per blob (id, offset, size,
//     counts, encoding) in the top-level "meshes" array — see ADR-0007a
//     for the honest MINOR-bump semantics (new readers open old files;
//     writers always write the current version).
//   * MAJOR.MINOR.PATCH `version` field with the semantics of ADR-0007:
//       - PATCH bumps are forward-compatible: old readers open new files
//         (unknown fields are ignored).
//       - MINOR bumps run the Migrator chain (upgrade-only).
//       - MAJOR bumps are incompatible: handled by a Migrator chain.
//   * zstd blob compression is deferred (ADR-0003, no new deps); the
//     "encoding" field in the blob header makes it a future clean bump.
//
// Transactional load: parse_document() validates and builds every node
// WITHOUT touching the scene; install_document() only then commits.
// A failed parse leaves the scene untouched (strong exception guarantee).
//
// Explicit errors: unknown SDF ops, missing fields, version mismatches
// without a registered migrator, malformed JSON, and excessively nested
// input (JSON or SDF tree) all throw std::runtime_error with a
// descriptive message — surfaced to the user (ADR-0007: "surfaces
// unsupported features rather than silently dropping them").
//
// Atomic save: the file is written to `<path>.tmp`, flushed, and renamed
// over the target — a crash mid-save never destroys the previous file.
//
#pragma once

#include "CAD_0/io/json.hpp"
#include "CAD_0/io/native_blob.hpp"
#include "CAD_0/math/vec.hpp"
#include "CAD_0/scene/scene.hpp"
#include "CAD_0/sdf/field.hpp"

#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace CAD_0::io::native {

// ---- Versioning (ADR-0007) -------------------------------------------------

struct NativeVersion {
    unsigned major{0};
    unsigned minor{1};
    unsigned patch{0};

    constexpr bool operator==(const NativeVersion& o) const noexcept {
        return major == o.major && minor == o.minor && patch == o.patch;
    }
    constexpr bool same_major_minor(const NativeVersion& o) const noexcept {
        return major == o.major && minor == o.minor;
    }
};

// The version this build reads and writes. 0.2.0: binary blob container
// for imported meshes (ADR-0007a). A migrator upgrades 0.1.0 documents.
inline constexpr NativeVersion kNativeVersion{0, 2, 0};

// A Migrator transforms a parsed JSON document from `from_version` to
// `to_version`. Registered in a chain indexed by the version pair. The
// first real entry is the no-op 0.1.0 -> 0.2.0 upgrade (v0.1.0 documents
// carry no blobs; a missing "meshes" member parses as "no meshes") —
// see ADR-0007a. Any file whose MAJOR.MINOR differs from the current
// version and has no registered migrator fails with an explicit error.
using Migrator = std::function<void(json::JsonValue&)>;

void register_migrator(NativeVersion from, NativeVersion to, Migrator m);
std::size_t migrator_count() noexcept; // test hook: 1 (0.1.0 -> 0.2.0)

// Parse "MAJOR.MINOR.PATCH" (e.g. "0.1.0"). Throws std::runtime_error
// with an explicit message on malformed input.
NativeVersion parse_version(std::string_view text);

// ---- Camera ----------------------------------------------------------------

// Orbit camera state as the desktop app drives it. fov is stored in
// degrees (matching the UI). Serialized in the document's "camera" block.
struct NativeCamera {
    math::Vec3f target{0.0f, 0.0f, 0.0f};
    float distance{5.0f};
    float yaw{0.0f};      // radians, around world +Y
    float pitch{0.3f};    // radians, clamped by the UI to ±~89°
    float fov_deg{45.0f};
};

// ---- Serialization ---------------------------------------------------------

// Maximum SDF tree nesting accepted when deserializing a document. Trees
// deeper than this are rejected with an explicit "too deeply nested"
// error: deserialization is recursive, so unbounded depth would risk a
// stack overflow on corrupt or hostile files. Legitimate SDF trees are
// shallow (the canonical composite is ~6 levels; even heavy modeling
// rarely exceeds a few dozen) — 128 leaves generous headroom while
// staying far below the JSON parser's kMaxNestingDepth (256), so the
// SDF-layer error fires first (with the domain-specific message) for
// trees between the two limits.
inline constexpr int kMaxSDFNestingDepth = 128;

// Serialize one SDF tree node (recursive; the root of a body's tree).
// Every node carries an "op" name; deserialization dispatches on it and
// fails explicitly on unknown ops.
json::JsonValue serialize_node(const sdf::SDFNode& node);

// A mesh carried in the document's blob section (ADR-0007a). `id` is the
// stable blob identifier referenced by "mesh"-kind scene nodes (wired in
// the scene layer); the geometry is the decoded TriangleMesh.
struct DocumentMesh {
    std::string id;
    sdf::TriangleMesh mesh;
};

// Serialize a full document (scene + camera + mesh library) as pretty
// JSON text — this is the JSON BLOCK of the container (see save_file for
// the full binary file). `meshes` emits the top-level "meshes" blob
// header array (omitted when empty). Deterministic, insertion-ordered —
// golden-file byte-stability.
std::string serialize_document(const scene::Scene& s, const NativeCamera& cam,
                               const std::vector<DocumentMesh>& meshes = {});

// ---- Deserialization (transactional, two stages) ---------------------------

// Stage 1 — pure parse: validates the version (running migrators when
// registered), parses every node, and builds the full node list. Never
// touches a Scene; throws std::runtime_error with explicit messages.
//
// Mesh blobs: `parse_document(text)` reads the JSON block only; a
// document that carries mesh blobs is rejected with an explicit "use
// load_file" error (the geometry lives in the file's blob section, not
// in the text). `parse_document(text, blob_section)` — the entry point
// load_file uses — additionally decodes and validates every mesh.
struct ParsedDocument {
    NativeVersion version{};
    // Fully formed (id, body, transform, params, builder) — and, for
    // nodes with a recognized builder tag, a re-wired parametric factory
    // so post-load editing rebuilds the body (set_param semantics).
    std::vector<scene::SceneNode> nodes;
    // Mesh library decoded from the blob section (ADR-0007a). The scene
    // layer wires "mesh"-kind nodes to these entries by id.
    std::vector<DocumentMesh> meshes;
    std::uint32_t next_id{1};
    NativeCamera camera{};
};
ParsedDocument parse_document(std::string_view text);
ParsedDocument parse_document(std::string_view text,
                               std::string_view blob_section);

// Rebuild a single SDF tree from JSON. Unknown "op" → explicit error;
// trees nested deeper than kMaxSDFNestingDepth → explicit error.
std::unique_ptr<sdf::SDFNode> deserialize_node(const json::JsonValue& v);

// Stage 2 — commit: clears the scene and installs the parsed nodes with
// their original ids, then restores next_id (clamped past the highest id).
// Notifies NodeAdded per node (UI stays in sync; callers clear their
// dirty flag after install completes).
void install_document(scene::Scene& s, ParsedDocument&& doc);

// ---- File-level helpers ----------------------------------------------------

// Atomic save: serialize -> build the ADR-0007a container -> write
// <path>.tmp -> flush -> rename over path. `meshes` (optional) carries
// imported-mesh geometry into the blob section. Throws
// std::runtime_error (with the path in the message) on I/O failure; the
// original file is left untouched in that case.
void save_file(const std::filesystem::path& path,
               const scene::Scene& s, const NativeCamera& cam,
               const std::vector<DocumentMesh>& meshes = {});

// Read + parse_document. Auto-detects the framing (CAD0BLOB container vs
// legacy v0.1.0 pure JSON — the latter runs the migrator) and decodes
// mesh blobs. Throws on missing file, I/O error, or any parse error
// (explicit message).
ParsedDocument load_file(const std::filesystem::path& path);

} // namespace CAD_0::io::native
