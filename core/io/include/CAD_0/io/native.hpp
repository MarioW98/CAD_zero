// core/io/include/CAD_0/io/native.hpp
//
// Native project format (.CAD_0) — implements ADR-0007.
//
// Format (v0.1.0, Phase A):
//   * JSON for structure (in-house parser, see json.hpp — ADR-0003 forbids
//     new third-party deps without a companion ADR).
//   * MAJOR.MINOR.PATCH `version` field with the semantics of ADR-0007:
//       - PATCH bumps are forward-compatible: old readers open new files
//         (unknown fields are ignored).
//       - MINOR bumps are backward-compatible: old readers open new files
//         with feature flags defaulted.
//       - MAJOR bumps are incompatible: handled by a Migrator chain.
//   * No binary blob in Phase A: the scene is fully parametric (SDF trees +
//     params), so cached meshes are re-tessellated on load rather than
//     persisted. zstd blobs arrive with embedded meshes/voxels (ADR-0007)
//     and will require a companion ADR.
//
// Transactional load: parse_document() validates and builds every node
// WITHOUT touching the scene; install_document() only then commits.
// A failed parse leaves the scene untouched (strong exception guarantee).
//
// Explicit errors: unknown SDF ops, missing fields, version mismatches
// without a registered migrator, and malformed JSON all throw
// std::runtime_error with a descriptive message — surfaced to the user
// (ADR-0007: "surfaces unsupported features rather than silently
// dropping them").
//
// Atomic save: the file is written to `<path>.tmp`, flushed, and renamed
// over the target — a crash mid-save never destroys the previous file.
//
#pragma once

#include "CAD_0/io/json.hpp"
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

// The version this build reads and writes. Phase A: 0.1.0 (ADR-0007).
inline constexpr NativeVersion kNativeVersion{0, 1, 0};

// A Migrator transforms a parsed JSON document from `from_version` to
// `to_version`. Registered in a chain indexed by the version pair; the
// registry is EMPTY in Phase A (ADR-0007 Consequences) — any file whose
// MAJOR.MINOR differs from the current version fails with an explicit
// "no migrator registered" error until migrators are added.
using Migrator = std::function<void(json::JsonValue&)>;

void register_migrator(NativeVersion from, NativeVersion to, Migrator m);
std::size_t migrator_count() noexcept; // test hook: 0 in Phase A

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

// Serialize one SDF tree node (recursive; the root of a body's tree).
// Every node carries an "op" name; deserialization dispatches on it and
// fails explicitly on unknown ops.
json::JsonValue serialize_node(const sdf::SDFNode& node);

// Serialize a full document (scene + camera) as pretty JSON text.
std::string serialize_document(const scene::Scene& s, const NativeCamera& cam);

// ---- Deserialization (transactional, two stages) ---------------------------

// Stage 1 — pure parse: validates the version (running migrators when
// registered), parses every node, and builds the full node list. Never
// touches a Scene; throws std::runtime_error with explicit messages.
struct ParsedDocument {
    NativeVersion version{};
    // Fully formed (id, body, transform, params, builder) — and, for
    // nodes with a recognized builder tag, a re-wired parametric factory
    // so post-load editing rebuilds the body (set_param semantics).
    std::vector<scene::SceneNode> nodes;
    std::uint32_t next_id{1};
    NativeCamera camera{};
};
ParsedDocument parse_document(std::string_view text);

// Rebuild a single SDF tree from JSON. Unknown "op" → explicit error.
std::unique_ptr<sdf::SDFNode> deserialize_node(const json::JsonValue& v);

// Stage 2 — commit: clears the scene and installs the parsed nodes with
// their original ids, then restores next_id (clamped past the highest id).
// Notifies NodeAdded per node (UI stays in sync; callers clear their
// dirty flag after install completes).
void install_document(scene::Scene& s, ParsedDocument&& doc);

// ---- File-level helpers ----------------------------------------------------

// Atomic save: serialize → write <path>.tmp → flush → rename over path.
// Throws std::runtime_error (with the path in the message) on I/O failure;
// the original file is left untouched in that case.
void save_file(const std::filesystem::path& path,
               const scene::Scene& s, const NativeCamera& cam);

// Read + parse_document. Throws on missing file, I/O error, or any parse
// error (explicit message).
ParsedDocument load_file(const std::filesystem::path& path);

} // namespace CAD_0::io::native
