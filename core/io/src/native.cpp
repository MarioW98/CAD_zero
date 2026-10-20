// core/io/src/native.cpp
//
// Implementation of the native .CAD_0 project format (ADR-0007).
// See native.hpp for the format rationale and API contract.
//
#include "CAD_0/io/native.hpp"

#include "CAD_0/scene/builders.hpp"
#include "CAD_0/scene/operations.hpp"
#include "CAD_0/sdf/operators.hpp"
#include "CAD_0/sdf/primitives.hpp"
#include "CAD_0/sdf/transforms.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>

#ifndef _WIN32
#include <fcntl.h>
#include <unistd.h>
#endif

namespace CAD_0::io::native {

using json::JsonArray;
using json::JsonObject;
using json::JsonValue;

namespace {

// ---- Migrator registry (ADR-0007a: first real entry 0.1.0 -> 0.2.0) -----

struct MigratorKey {
    NativeVersion from, to;
    bool operator<(const MigratorKey& o) const noexcept {
        if (from.major != o.from.major) return from.major < o.from.major;
        if (from.minor != o.from.minor) return from.minor < o.from.minor;
        if (from.patch != o.from.patch) return from.patch < o.from.patch;
        if (to.major != o.to.major) return to.major < o.to.major;
        if (to.minor != o.to.minor) return to.minor < o.to.minor;
        return to.patch < o.to.patch;
    }
};

std::map<MigratorKey, Migrator>& migrator_registry() {
    static std::map<MigratorKey, Migrator> registry;
    return registry;
}

// The 0.1.0 -> 0.2.0 migrator (ADR-0007a): a deliberate no-op. v0.1.0
// documents carry no blob payloads, and a missing "meshes" member
// already parses as "no meshes" (the same tolerance PATCH files rely
// on). The entry exists so the upgrade path is explicit, registered and
// exercised by the frozen v0.1.0 golden file on every CI run.
const bool kMigrator0102Registered [[maybe_unused]] = [] {
    register_migrator(NativeVersion{0, 1, 0}, NativeVersion{0, 2, 0},
                      [](json::JsonValue&) {
                          // no-op by design — see ADR-0007a.
                      });
    return true;
}();

// The 0.2.0 -> 0.3.0 migrator (ADR-0027 D4): a deliberate CONTENT
// no-op. v0.2.0 documents carry no operation fields, and a missing
// "operation" member parses as "not an operation node" — the same
// field tolerance PATCH files rely on. The entry exists so the chain
// 0.1.0 → 0.2.0 → 0.3.0 is explicit, registered, and exercised by
// both frozen goldens on every CI run (the v0.1.0 one walks BOTH
// steps).
const bool kMigrator0203Registered [[maybe_unused]] = [] {
    register_migrator(NativeVersion{0, 2, 0}, NativeVersion{0, 3, 0},
                      [](json::JsonValue&) {
                          // no-op by design — see ADR-0027 D4.
                      });
    return true;
}();

std::string version_to_string(const NativeVersion& v) {
    return std::to_string(v.major) + "." + std::to_string(v.minor) + "." +
           std::to_string(v.patch);
}

// ---- JSON <-> math helpers --------------------------------------------------

[[noreturn]] void type_error(const std::string& what, const JsonValue& where) {
    std::string kind = where.is_null() ? "null" :
                       where.is_bool() ? "bool" :
                       where.is_number() ? "number" :
                       where.is_string() ? "string" :
                       where.is_array() ? "array" : "object";
    throw std::runtime_error("native format: expected " + what + " but found " + kind);
}

double require_number(const JsonValue& v, const char* field) {
    if (!v.is_number()) type_error(std::string("number for field \"") + field + "\"", v);
    return v.as_number();
}

math::Vec3f json_to_vec3(const JsonValue& v, const char* field) {
    if (!v.is_array() || v.as_array().size() != 3)
        type_error(std::string("3-element array for field \"") + field + "\"", v);
    return math::Vec3f{
        static_cast<float>(require_number(v.at(0), field)),
        static_cast<float>(require_number(v.at(1), field)),
        static_cast<float>(require_number(v.at(2), field)),
    };
}

JsonValue vec3_to_json(const math::Vec3f& p) {
    return JsonValue(JsonArray{
        JsonValue(static_cast<double>(p.x)),
        JsonValue(static_cast<double>(p.y)),
        JsonValue(static_cast<double>(p.z)),
    });
}

math::Quatf json_to_quat(const JsonValue& v, const char* field) {
    if (!v.is_array() || v.as_array().size() != 4)
        type_error(std::string("4-element array [w,x,y,z] for field \"") + field + "\"", v);
    return math::Quatf{
        static_cast<float>(require_number(v.at(0), field)),
        static_cast<float>(require_number(v.at(1), field)),
        static_cast<float>(require_number(v.at(2), field)),
        static_cast<float>(require_number(v.at(3), field)),
    };
}

JsonValue quat_to_json(const math::Quatf& q) {
    return JsonValue(JsonArray{
        JsonValue(static_cast<double>(q.w)),
        JsonValue(static_cast<double>(q.x)),
        JsonValue(static_cast<double>(q.y)),
        JsonValue(static_cast<double>(q.z)),
    });
}

// ---- Mesh blob headers (ADR-0007a) -----------------------------------------

// The blob id of a scene node's mesh: stable across save/load because
// node ids round-trip exactly (deterministic, no extra state).
std::string mesh_blob_id(const scene::SceneNode& node) {
    return "mesh" + std::to_string(node.id);
}

// The document's full mesh library, in "meshes" array order: every
// Mesh-kind scene node (node order) followed by the caller's extra
// entries (e.g. the unattached meshes of a previously loaded document,
// preserved for byte-identical re-serialization). Duplicate ids are an
// explicit error — a reference would be ambiguous.
std::vector<DocumentMesh> collect_document_meshes(
    const scene::Scene& s, const std::vector<DocumentMesh>& extra_meshes) {
    std::vector<DocumentMesh> meshes;
    meshes.reserve(extra_meshes.size());
    for (const auto& node : s.nodes())
        if (node.kind == scene::NodeKind::Mesh)
            meshes.push_back(DocumentMesh{mesh_blob_id(node), node.mesh});
    for (const auto& dm : extra_meshes)
        meshes.push_back(dm);

    for (std::size_t i = 0; i < meshes.size(); ++i)
        for (std::size_t j = i + 1; j < meshes.size(); ++j)
            if (meshes[i].id == meshes[j].id)
                throw std::runtime_error(
                    "native format: duplicate mesh blob id \"" + meshes[i].id +
                    "\" (a mesh node and an extra library entry, or two "
                    "entries, share the same id — references would be "
                    "ambiguous)");
    return meshes;
}

// Header record for one mesh blob, as it appears in the JSON "meshes"
// array. Internal: parse_document uses it to locate and validate the
// payload inside the blob section before decoding.
struct MeshHeaderRecord {
    std::string id;
    std::string encoding;
    std::size_t offset{0};
    std::size_t size{0};
    std::size_t vertex_count{0};
    std::size_t triangle_count{0};
    bool has_normals{false};
};

// Non-negative, integral double -> size_t. The JSON parser stores every
// number as double; header fields are counts/offsets and must be exact.
std::size_t require_count(const JsonValue& v, const std::string& blob_id,
                          const char* field) {
    if (!v.is_number())
        type_error(std::string("number for mesh blob field \"") + field + "\"", v);
    const double d = v.as_number();
    if (!(d >= 0.0) || d != std::floor(d) ||
        d > static_cast<double>(std::numeric_limits<std::uint32_t>::max()))
        throw std::runtime_error(
            "native format: mesh blob \"" + blob_id + "\": field \"" + field +
            "\" must be a non-negative integer (got " + std::to_string(d) + ")");
    return static_cast<std::size_t>(d);
}

// Parse + validate the JSON "meshes" array into header records. Strict:
// unknown encodings, duplicate ids and non-integer counts are explicit
// errors (ADR-0007/0007a — surface, never silently drop).
std::vector<MeshHeaderRecord> parse_mesh_headers(const JsonValue& meshes_v) {
    if (!meshes_v.is_array())
        type_error("array for \"meshes\"", meshes_v);

    std::vector<MeshHeaderRecord> records;
    records.reserve(meshes_v.as_array().size());
    for (const auto& entry : meshes_v.as_array()) {
        if (!entry.is_object())
            type_error("object for \"meshes\" entry", entry);

        MeshHeaderRecord r;
        if (const JsonValue* id_v = entry.find("id")) {
            if (!id_v->is_string())
                type_error("string for mesh blob \"id\"", *id_v);
            r.id = id_v->as_string();
        } else {
            throw std::runtime_error(
                "native format: \"meshes\" entry is missing required field \"id\"");
        }
        if (r.id.empty())
            throw std::runtime_error(
                "native format: mesh blob id must be a non-empty string");

        if (const JsonValue* enc = entry.find("encoding")) {
            if (!enc->is_string())
                type_error("string for mesh blob \"encoding\"", *enc);
            r.encoding = enc->as_string();
        } else {
            throw std::runtime_error(
                "native format: mesh blob \"" + r.id +
                "\" is missing required field \"encoding\"");
        }
        if (r.encoding != "raw")
            throw std::runtime_error(
                "native format: mesh blob \"" + r.id + "\" uses unsupported "
                "encoding \"" + r.encoding + "\" (this build reads \"raw\"; "
                "zstd arrives with a future version bump — ADR-0007a)");

        r.offset = require_count(entry.at("offset"), r.id, "offset");
        r.size = require_count(entry.at("size"), r.id, "size");
        r.vertex_count =
            require_count(entry.at("vertex_count"), r.id, "vertex_count");
        r.triangle_count =
            require_count(entry.at("triangle_count"), r.id, "triangle_count");
        if (const JsonValue* hn = entry.find("has_normals")) {
            if (!hn->is_bool())
                type_error("bool for mesh blob \"has_normals\"", *hn);
            r.has_normals = hn->as_bool();
        }

        for (const auto& prev : records)
            if (prev.id == r.id)
                throw std::runtime_error(
                    "native format: duplicate mesh blob id \"" + r.id + "\"");
        records.push_back(std::move(r));
    }
    return records;
}

// Decode every mesh from the blob section according to its header.
// Validates offset/size against the section and leaves a clear error
// trail naming the blob id (transactional: throws before any scene use).
std::vector<DocumentMesh> decode_meshes(
    const std::vector<MeshHeaderRecord>& records,
    std::string_view blob_section) {
    std::vector<DocumentMesh> meshes;
    meshes.reserve(records.size());
    for (const auto& r : records) {
        if (r.offset > blob_section.size() ||
            r.size > blob_section.size() - r.offset)
            throw std::runtime_error(
                "native format: mesh blob \"" + r.id + "\" extends beyond the "
                "blob section (offset " + std::to_string(r.offset) + ", size " +
                std::to_string(r.size) + ", section is " +
                std::to_string(blob_section.size()) + " bytes)");
        const auto payload =
            blob_section.substr(r.offset, r.size);
        meshes.push_back(DocumentMesh{
            r.id, decode_blob_raw(payload, r.id, r.vertex_count,
                                  r.triangle_count, r.has_normals)});
    }
    return meshes;
}

// ---- SDF tree serialization -------------------------------------------------

JsonValue serialize_node_impl(const sdf::SDFNode& n) {
    using namespace sdf;
    if (auto* s = dynamic_cast<const SphereSDF*>(&n)) {
        return JsonValue(JsonObject{}).set("op", "sphere").set("radius", s->radius());
    }
    if (auto* b = dynamic_cast<const BoxSDF*>(&n)) {
        return JsonValue(JsonObject{}).set("op", "box").set("extent", vec3_to_json(b->extent()));
    }
    if (auto* c = dynamic_cast<const CylinderSDF*>(&n)) {
        return JsonValue(JsonObject{}).set("op", "cylinder")
            .set("radius", c->radius()).set("height", c->height());
    }
    if (auto* t = dynamic_cast<const TorusSDF*>(&n)) {
        return JsonValue(JsonObject{}).set("op", "torus")
            .set("R", t->major_radius()).set("r", t->minor_radius());
    }
    if (auto* k = dynamic_cast<const ConeSDF*>(&n)) {
        return JsonValue(JsonObject{}).set("op", "cone")
            .set("r", k->base_radius()).set("h", k->height());
    }
    if (auto* cap = dynamic_cast<const CapsuleSDF*>(&n)) {
        return JsonValue(JsonObject{}).set("op", "capsule")
            .set("a", vec3_to_json(cap->point_a()))
            .set("b", vec3_to_json(cap->point_b()))
            .set("r", cap->radius());
    }
    if (dynamic_cast<const PlaneSDF*>(&n)) {
        return JsonValue(JsonObject{}).set("op", "plane");
    }
    if (auto* u = dynamic_cast<const UnionSDF*>(&n)) {
        JsonArray children;
        children.reserve(u->children().size());
        for (const auto& c : u->children())
            children.push_back(serialize_node_impl(*c));
        return JsonValue(JsonObject{}).set("op", "union").set("children", std::move(children));
    }
    if (auto* i = dynamic_cast<const IntersectionSDF*>(&n)) {
        return JsonValue(JsonObject{}).set("op", "intersect")
            .set("a", serialize_node_impl(*i->child_a()))
            .set("b", serialize_node_impl(*i->child_b()));
    }
    if (auto* d = dynamic_cast<const SubtractionSDF*>(&n)) {
        return JsonValue(JsonObject{}).set("op", "subtract")
            .set("a", serialize_node_impl(*d->child_a()))
            .set("b", serialize_node_impl(*d->child_b()));
    }
    if (auto* m = dynamic_cast<const SmoothMinSDF*>(&n)) {
        return JsonValue(JsonObject{}).set("op", "smooth_min")
            .set("a", serialize_node_impl(*m->child_a()))
            .set("b", serialize_node_impl(*m->child_b()))
            .set("k", m->blend_k());
    }
    if (auto* tr = dynamic_cast<const TranslateSDF*>(&n)) {
        return JsonValue(JsonObject{}).set("op", "translate")
            .set("offset", vec3_to_json(tr->offset()))
            .set("child", serialize_node_impl(*tr->wrapped()));
    }
    if (auto* ro = dynamic_cast<const RotateSDF*>(&n)) {
        return JsonValue(JsonObject{}).set("op", "rotate")
            .set("q", quat_to_json(ro->quat()))
            .set("child", serialize_node_impl(*ro->wrapped()));
    }
    if (auto* sc = dynamic_cast<const ScaleSDF*>(&n)) {
        return JsonValue(JsonObject{}).set("op", "scale")
            .set("s", sc->scale())
            .set("child", serialize_node_impl(*sc->wrapped()));
    }
    if (auto* tw = dynamic_cast<const TwistSDF*>(&n)) {
        return JsonValue(JsonObject{}).set("op", "twist")
            .set("k", tw->twist_k())
            .set("child", serialize_node_impl(*tw->wrapped()));
    }
    throw std::runtime_error(
        "native format: cannot serialize SDF node of type '" + n.describe() +
        "' — no native-format serializer registered for this node type");
}

// ---- SDF tree deserialization -----------------------------------------------

const JsonValue& require_field(const JsonValue& v, const char* field) {
    if (!v.is_object()) type_error("object for SDF node", v);
    const JsonValue* f = v.find(field);
    if (!f)
        throw std::runtime_error(std::string("native format: SDF node is missing "
            "required field \"") + field + "\"");
    return *f;
}

std::unique_ptr<sdf::SDFNode> deserialize_node_impl(const JsonValue& v, int depth) {
    using namespace sdf;
    // Depth guard: reject runaway SDF nesting with an explicit error
    // naming the limit BEFORE the recursion can overflow the stack.
    // (The JSON parser independently caps nesting at 256; this check
    // fires first for trees between the two limits and also protects
    // the public deserialize_node() on programmatically built JSON.)
    if (depth > kMaxSDFNestingDepth)
        throw std::runtime_error(
            "native format: SDF tree too deeply nested (max " +
            std::to_string(kMaxSDFNestingDepth) + " levels) — a legitimate "
            "SDF tree is far shallower; the file is likely corrupt or hostile");
    if (!v.is_object()) type_error("object for SDF node", v);
    const JsonValue* op_v = v.find("op");
    if (!op_v || !op_v->is_string())
        throw std::runtime_error("native format: SDF node has no \"op\" field "
                                 "(explicit error per ADR-0007: unsupported "
                                 "features must surface, not drop silently)");
    const std::string op = op_v->as_string();

    if (op == "sphere")
        return std::make_unique<SphereSDF>(
            static_cast<float>(require_number(require_field(v, "radius"), "radius")));
    if (op == "box")
        return std::make_unique<BoxSDF>(json_to_vec3(require_field(v, "extent"), "extent"));
    if (op == "cylinder")
        return std::make_unique<CylinderSDF>(
            static_cast<float>(require_number(require_field(v, "radius"), "radius")),
            static_cast<float>(require_number(require_field(v, "height"), "height")));
    if (op == "torus")
        return std::make_unique<TorusSDF>(
            static_cast<float>(require_number(require_field(v, "R"), "R")),
            static_cast<float>(require_number(require_field(v, "r"), "r")));
    if (op == "cone")
        return std::make_unique<ConeSDF>(
            static_cast<float>(require_number(require_field(v, "r"), "r")),
            static_cast<float>(require_number(require_field(v, "h"), "h")));
    if (op == "capsule")
        return std::make_unique<CapsuleSDF>(
            json_to_vec3(require_field(v, "a"), "a"),
            json_to_vec3(require_field(v, "b"), "b"),
            static_cast<float>(require_number(require_field(v, "r"), "r")));
    if (op == "plane")
        return std::make_unique<PlaneSDF>();
    if (op == "union") {
        const JsonValue& children = require_field(v, "children");
        if (!children.is_array())
            type_error("array for field \"children\"", children);
        std::vector<std::unique_ptr<SDFNode>> cs;
        cs.reserve(children.as_array().size());
        for (const auto& c : children.as_array())
            cs.push_back(deserialize_node_impl(c, depth + 1));
        return std::make_unique<UnionSDF>(std::move(cs));
    }
    if (op == "intersect")
        return std::make_unique<IntersectionSDF>(
            deserialize_node_impl(require_field(v, "a"), depth + 1),
            deserialize_node_impl(require_field(v, "b"), depth + 1));
    if (op == "subtract")
        return std::make_unique<SubtractionSDF>(
            deserialize_node_impl(require_field(v, "a"), depth + 1),
            deserialize_node_impl(require_field(v, "b"), depth + 1));
    if (op == "smooth_min")
        return std::make_unique<SmoothMinSDF>(
            deserialize_node_impl(require_field(v, "a"), depth + 1),
            deserialize_node_impl(require_field(v, "b"), depth + 1),
            static_cast<float>(require_number(require_field(v, "k"), "k")));
    if (op == "translate")
        return std::make_unique<TranslateSDF>(
            deserialize_node_impl(require_field(v, "child"), depth + 1),
            json_to_vec3(require_field(v, "offset"), "offset"));
    if (op == "rotate")
        return std::make_unique<RotateSDF>(
            deserialize_node_impl(require_field(v, "child"), depth + 1),
            json_to_quat(require_field(v, "q"), "q"));
    if (op == "scale")
        return std::make_unique<ScaleSDF>(
            deserialize_node_impl(require_field(v, "child"), depth + 1),
            static_cast<float>(require_number(require_field(v, "s"), "s")));
    if (op == "twist")
        return std::make_unique<TwistSDF>(
            deserialize_node_impl(require_field(v, "child"), depth + 1),
            static_cast<float>(require_number(require_field(v, "k"), "k")));

    throw std::runtime_error("native format: unknown SDF op \"" + op + "\" "
        "(explicit error per ADR-0007 — this file was likely written by a "
        "newer CAD_0 version)");
}

// ---- Transform / params / camera --------------------------------------------

JsonValue transform_to_json(const geometry::Transform& t) {
    return JsonValue(JsonObject{})
        .set("translation", vec3_to_json(t.translation))
        .set("rotation", quat_to_json(t.rotation))
        .set("scale", t.scale);
}

geometry::Transform json_to_transform(const JsonValue& v) {
    geometry::Transform t;
    t.translation = json_to_vec3(v.at("translation"), "translation");
    t.rotation = json_to_quat(v.at("rotation"), "rotation");
    t.scale = static_cast<float>(require_number(v.at("scale"), "scale"));
    return t;
}

JsonValue params_to_json(const std::map<std::string, scene::ParamValue>& params) {
    JsonObject out;
    for (const auto& [key, value] : params) {
        if (auto* f = std::get_if<float>(&value)) {
            out.emplace_back(key, JsonValue(static_cast<double>(*f)));
        } else if (auto* s = std::get_if<std::string>(&value)) {
            out.emplace_back(key, JsonValue(*s));
        } else if (auto* b = std::get_if<bool>(&value)) {
            out.emplace_back(key, JsonValue(*b));
        }
    }
    return JsonValue(std::move(out));
}

std::map<std::string, scene::ParamValue> json_to_params(const JsonValue& v) {
    std::map<std::string, scene::ParamValue> out;
    if (!v.is_object()) type_error("object for node \"params\"", v);
    for (const auto& [key, value] : v.as_object()) {
        if (value.is_number()) {
            out[key] = static_cast<float>(value.as_number());
        } else if (value.is_string()) {
            out[key] = value.as_string();
        } else if (value.is_bool()) {
            out[key] = value.as_bool();
        } else {
            throw std::runtime_error("native format: param \"" + key +
                "\" has unsupported value type (expected number, string or bool)");
        }
    }
    return out;
}

// ---- Parametric builder registry consumption ------------------------------
//
// The builders themselves (tags, parameter schemas, defaults, body
// construction) live in ONE place: core/scene/builders.hpp
// (scene::builder_registry — the single source of truth, also exposed to
// Python via the scene bindings so the desktop UI consumes only it).
// This file is a pure CONSUMER: after a load it re-wires each parsed
// node's factory from the persisted `builder` tag so post-load editing
// works through the kernel API.
//
// Unknown or missing builder tags deliberately stay non-parametric: the
// body itself is fully serialized, so such a file still opens and
// round-trips — the tag is editing metadata, not scene data (ADR-0007
// PATCH compatibility: unknown fields degrade gracefully rather than
// failing the load). The explicit creation API (scene::default_params /
// scene::make_body) DOES fail loudly on unknown tags — the tolerant and
// strict contracts are deliberately distinct.

JsonValue camera_to_json(const NativeCamera& cam) {
    return JsonValue(JsonObject{})
        .set("target", vec3_to_json(cam.target))
        .set("distance", cam.distance)
        .set("yaw", cam.yaw)
        .set("pitch", cam.pitch)
        .set("fov_deg", cam.fov_deg);
}

NativeCamera json_to_camera(const JsonValue& v) {
    NativeCamera cam;
    cam.target = json_to_vec3(v.at("target"), "target");
    cam.distance = static_cast<float>(require_number(v.at("distance"), "distance"));
    cam.yaw = static_cast<float>(require_number(v.at("yaw"), "yaw"));
    cam.pitch = static_cast<float>(require_number(v.at("pitch"), "pitch"));
    cam.fov_deg = static_cast<float>(require_number(v.at("fov_deg"), "fov_deg"));
    return cam;
}

} // anonymous namespace

// ---- public API -------------------------------------------------------------

void register_migrator(NativeVersion from, NativeVersion to, Migrator m) {
    migrator_registry()[MigratorKey{from, to}] = std::move(m);
}

std::size_t migrator_count() noexcept {
    return migrator_registry().size();
}

NativeVersion parse_version(std::string_view text) {
    unsigned major = 0, minor = 0, patch = 0;
    char dot1 = 0, dot2 = 0;
    const int fields = std::sscanf(std::string(text).c_str(), "%u.%u.%u%c",
                                   &major, &minor, &patch, &dot2);
    (void)dot1;
    if (fields < 3 || (fields == 4 && dot2 != '\0'))
        throw std::runtime_error("native format: malformed version string \"" +
                                 std::string(text) + "\" (expected \"MAJOR.MINOR.PATCH\")");
    return NativeVersion{major, minor, patch};
}

json::JsonValue serialize_node(const sdf::SDFNode& node) {
    return serialize_node_impl(node);
}

std::unique_ptr<sdf::SDFNode> deserialize_node(const json::JsonValue& v) {
    return deserialize_node_impl(v, 0);
}

std::string serialize_document(const scene::Scene& s, const NativeCamera& cam,
                               const std::vector<DocumentMesh>& extra_meshes) {
    // Full document mesh library (ADR-0007a): Mesh-kind nodes' payloads
    // (node order, ids "mesh<node id>") + the caller's extra entries.
    // collect_document_meshes() validates id uniqueness.
    const std::vector<DocumentMesh> meshes =
        collect_document_meshes(s, extra_meshes);

    JsonObject scene_json;
    scene_json.emplace_back("next_id", JsonValue(static_cast<double>(s.next_id())));

    JsonArray nodes;
    nodes.reserve(s.nodes().size());
    for (const auto& node : s.nodes()) {
        JsonObject n;
        n.emplace_back("id", JsonValue(static_cast<double>(node.id)));
        n.emplace_back("name", JsonValue(node.name));
        n.emplace_back("visible", JsonValue(node.visible));
        n.emplace_back("transform", transform_to_json(node.transform));
        if (node.kind == scene::NodeKind::Mesh) {
            // A Mesh node IS its geometry: kind + blob reference, no
            // builder/params/body (ADR-0002 honest dispatch — an imported
            // mesh is never faked as an SDF).
            n.emplace_back("kind", JsonValue("mesh"));
            n.emplace_back("mesh", JsonValue(JsonObject{})
                                      .set("blob", JsonValue(mesh_blob_id(node))));
        } else if (node.kind == scene::NodeKind::Sketch) {
            // v0.3 has no sketch payload section: refuse loudly (a
            // null body deref would crash; an empty body would lie).
            // The sketch/extrude format is v0.4 (ADR-0030 D4).
            throw std::runtime_error(
                "native format: cannot serialize the Sketch-kind node \"" +
                node.name + "\" (#" + std::to_string(node.id) +
                ") - the sketch payload is a v0.4 concern (ADR-0030 D4); "
                "v0.3 serializes Sdf and Mesh nodes only");
        } else {
            if (!node.builder.empty())
                n.emplace_back("builder", JsonValue(node.builder));
            if (!node.params.empty())
                n.emplace_back("params", params_to_json(node.params));
            n.emplace_back("body", serialize_node_impl(*node.body.root()));
            // v0.3.0 (ADR-0027 D4): the operation recipe — the node's
            // provenance, serialized right after the cached body it
            // explains. The exact ADR-0027 D4 shape:
            //   "operation": { "type": "subtract", "target": 2,
            //                  "tool": 3, "params": { "offset_x": … } }
            if (node.operation) {
                if (node.operation->type != scene::OperationType::Subtract)
                    throw std::runtime_error(
                        "native format: cannot serialize an Extrude "
                        "operation - v0.3 serializes subtract recipes only; "
                        "the extrude recipe is v0.4 (ADR-0030 D4)");
                JsonValue op(JsonObject{});
                op.as_object().emplace_back(
                    "type",
                    JsonValue("subtract"));
                op.as_object().emplace_back(
                    "target",
                    JsonValue(static_cast<double>(node.operation->target)));
                op.as_object().emplace_back(
                    "tool",
                    JsonValue(static_cast<double>(node.operation->tool)));
                op.as_object().emplace_back(
                    "params",
                    JsonValue(JsonObject{})
                        .set("offset_x",
                             JsonValue(static_cast<double>(
                                 node.operation->offset_x)))
                        .set("offset_y",
                             JsonValue(static_cast<double>(
                                 node.operation->offset_y)))
                        .set("offset_z",
                             JsonValue(static_cast<double>(
                                 node.operation->offset_z))));
                n.emplace_back("operation", std::move(op));
            }
        }
        nodes.push_back(JsonValue(std::move(n)));
    }
    scene_json.emplace_back("nodes", JsonValue(std::move(nodes)));

    JsonValue doc(JsonObject{});
    doc.as_object().emplace_back("format", JsonValue("CAD_0 native"));
    doc.as_object().emplace_back("version", JsonValue(version_to_string(kNativeVersion)));
    doc.as_object().emplace_back("scene", JsonValue(std::move(scene_json)));
    doc.as_object().emplace_back("camera", camera_to_json(cam));

    // ADR-0007a: the top-level "meshes" blob-header array. Omitted when
    // empty — a mesh-free v0.2.0 document differs from its v0.1.0
    // counterpart ONLY in the version string. Offsets are relative to the
    // start of the blob section and are assigned in array order (the
    // writer packs payloads back-to-back in the same order).
    if (!meshes.empty()) {
        JsonArray mesh_headers;
        mesh_headers.reserve(meshes.size());
        std::size_t offset = 0;
        for (const auto& dm : meshes) {
            const bool has_normals = !dm.mesh.normals.empty();
            const std::size_t size =
                raw_blob_size(dm.mesh.positions.size(),
                              dm.mesh.triangle_count(), has_normals);
            mesh_headers.push_back(JsonValue(JsonObject{})
                .set("id", JsonValue(dm.id))
                .set("encoding", JsonValue("raw"))
                .set("offset", JsonValue(static_cast<double>(offset)))
                .set("size", JsonValue(static_cast<double>(size)))
                .set("vertex_count",
                     JsonValue(static_cast<double>(dm.mesh.positions.size())))
                .set("triangle_count",
                     JsonValue(static_cast<double>(dm.mesh.triangle_count())))
                .set("has_normals", JsonValue(has_normals)));
            offset += size;
        }
        doc.as_object().emplace_back("meshes", JsonValue(std::move(mesh_headers)));
    }
    return doc.dump(true);
}

ParsedDocument parse_document(std::string_view text,
                               std::string_view blob_section) {
    JsonValue root = json::JsonValue::parse(text);

    if (!root.is_object())
        type_error("object at document root", root);

    // "format" tag — explicit check so foreign files fail loudly.
    if (const JsonValue* fmt = root.find("format")) {
        if (!fmt->is_string() || fmt->as_string() != "CAD_0 native")
            throw std::runtime_error("native format: unexpected \"format\" tag — "
                                     "this is not a CAD_0 native document");
    } else {
        throw std::runtime_error("native format: missing \"format\" tag");
    }

    // Version + ADR-0007 compatibility rules.
    const JsonValue& version_v = root.at("version");
    if (!version_v.is_string())
        type_error("version string", version_v);
    NativeVersion file_version = parse_version(version_v.as_string());

    if (!file_version.same_major_minor(kNativeVersion)) {
        // MAJOR or MINOR mismatch → the migrator CHAIN, walked one
        // MINOR step at a time (ADR-0007): 0.1.0 → 0.2.0 → 0.3.0 → …
        // Each step's lookup key normalizes the file's PATCH to zero:
        // a migrator registered from (M.m.0) upgrades EVERY M.m.x —
        // PATCH differences are compatible in both directions
        // (ADR-0007), so 0.1.3 opens with the 0.1.0 → 0.2.0 step.
        // Migrators run forward (upgrade) only; a missing step is an
        // explicit error naming both ends. The step search is bounded
        // to the current MAJOR.MINOR and the minor strictly increases,
        // so the walk always terminates. ParsedDocument::version keeps
        // reporting the FILE's original version (the historical fact);
        // the migrated representation is the one this build produces.
        const auto& registry = migrator_registry();
        NativeVersion current = file_version;
        while (!current.same_major_minor(kNativeVersion)) {
            const NativeVersion from{current.major, current.minor, 0};
            const auto it = std::find_if(
                registry.begin(), registry.end(),
                [&from](const std::pair<MigratorKey, Migrator>& kv) {
                    return kv.first.from == from &&
                           kv.first.to.major == from.major &&
                           kv.first.to.minor > from.minor &&
                           kv.first.to.minor <= kNativeVersion.minor;
                });
            if (it == registry.end()) {
                throw std::runtime_error(
                    "native format: file version " +
                    version_to_string(file_version) +
                    " cannot be read by this build (expects " +
                    version_to_string(kNativeVersion) +
                    "): no migrator registered from " +
                    version_to_string(from) +
                    " — ADR-0007 requires an explicit migrator chain "
                    "for MAJOR/MINOR changes");
            }
            it->second(root);
            current = it->first.to;
        }
    }
    // PATCH differences are forward/backward compatible (ADR-0007).

    // Mesh blob library (ADR-0007a; optional) — parsed and decoded BEFORE
    // the nodes so Mesh-kind nodes can wire their payload inline. The
    // header array is validated in every mode; the payload decode needs
    // the blob section, which only load_file can supply.
    std::vector<DocumentMesh> mesh_library;
    if (const JsonValue* meshes_v = root.find("meshes")) {
        const std::vector<MeshHeaderRecord> records =
            parse_mesh_headers(*meshes_v);
        if (!records.empty())
            mesh_library = decode_meshes(records, blob_section);
    }

    const JsonValue& scene_v = root.at("scene");
    if (!scene_v.is_object())
        type_error("object for \"scene\"", scene_v);

    ParsedDocument doc;
    doc.version = file_version;

    // next_id (required).
    doc.next_id = static_cast<std::uint32_t>(require_number(scene_v.at("next_id"), "next_id"));
    if (doc.next_id == 0)
        throw std::runtime_error("native format: \"next_id\" must be >= 1");

    // nodes.
    const JsonValue& nodes_v = scene_v.at("nodes");
    if (!nodes_v.is_array())
        type_error("array for scene \"nodes\"", nodes_v);

    for (const auto& node_v : nodes_v.as_array()) {
        if (!node_v.is_object())
            type_error("object for scene node", node_v);

        scene::SceneNode node;
        node.id = static_cast<std::uint32_t>(require_number(node_v.at("id"), "id"));

        // Node kind (FASE 2): absent -> Sdf (v0.1.0/v0.2.0 SDF nodes stay
        // byte-identical); "mesh" -> an imported-geometry node; anything
        // else is an explicit error (honest dispatch, ADR-0002).
        std::string mesh_ref; // blob id for Mesh-kind nodes
        if (const JsonValue* kind_v = node_v.find("kind")) {
            if (!kind_v->is_string())
                type_error("string for node \"kind\"", *kind_v);
            const std::string kind = kind_v->as_string();
            if (kind == "mesh") {
                node.kind = scene::NodeKind::Mesh;
                const JsonValue& mesh_v = node_v.at("mesh");
                if (!mesh_v.is_object())
                    type_error("object for node \"mesh\"", mesh_v);
                const JsonValue& blob_v = mesh_v.at("blob");
                if (!blob_v.is_string())
                    type_error("string for node \"mesh\" \"blob\"", blob_v);
                mesh_ref = blob_v.as_string();
                if (mesh_ref.empty())
                    throw std::runtime_error(
                        "native format: node " + std::to_string(node.id) +
                        " has an empty mesh blob reference");
            } else {
                throw std::runtime_error(
                    "native format: unknown node \"kind\" \"" + kind +
                    "\" (this build reads Sdf nodes and \"mesh\" nodes; the "
                    "file was likely written by a newer CAD_0 version)");
            }
        }

        if (node.kind == scene::NodeKind::Sdf) {
            node.mesh_dirty = true; // re-tessellate SDF caches after load
        } else {
            node.mesh_dirty = false; // the Mesh payload IS final — never
                                     // re-tessellated (update_meshes skips)
        }

        if (const JsonValue* name = node_v.find("name")) {
            if (!name->is_string()) type_error("string for node \"name\"", *name);
            node.name = name->as_string();
        } else {
            node.name = node.kind == scene::NodeKind::Mesh
                            ? "Mesh_" + std::to_string(node.id)
                            : "Shape_" + std::to_string(node.id);
        }

        if (const JsonValue* vis = node_v.find("visible")) {
            if (!vis->is_bool()) type_error("bool for node \"visible\"", *vis);
            node.visible = vis->as_bool();
        }

        if (const JsonValue* tr = node_v.find("transform"))
            node.transform = json_to_transform(*tr);

        if (node.kind == scene::NodeKind::Mesh) {
            // No builder/params/body for an imported mesh (ADR-0002):
            // those fields belong to the SDF representation. Surprising
            // leftovers on a mesh node are surfaced, not ignored.
            for (const char* leftover : {"builder", "params", "body", "operation"})
                if (node_v.find(leftover))
                    throw std::runtime_error(
                        std::string("native format: node ") +
                        std::to_string(node.id) + " is a \"mesh\" node but "
                        "carries an SDF-only field \"" + leftover + "\" — "
                        "the two representations never mix");
            // Wire the payload from the decoded library (references are
            // validated HERE — transactional, before any scene use).
            const auto entry = std::find_if(
                mesh_library.begin(), mesh_library.end(),
                [&](const DocumentMesh& dm) { return dm.id == mesh_ref; });
            if (entry == mesh_library.end())
                throw std::runtime_error(
                    "native format: node " + std::to_string(node.id) +
                    " references mesh blob \"" + mesh_ref +
                    "\" which is not declared in the document's \"meshes\" "
                    "array (missing or truncated blob section?)");
            node.mesh = entry->mesh;
            doc.nodes.push_back(std::move(node));
            continue;
        }

        if (const JsonValue* builder = node_v.find("builder")) {
            if (!builder->is_string()) type_error("string for node \"builder\"", *builder);
            node.builder = builder->as_string();
        }

        if (const JsonValue* params = node_v.find("params"))
            node.params = json_to_params(*params);

        // Re-wire the parametric factory from the persisted builder tag so
        // post-load editing works through the kernel API (set_node_param
        // rebuilds the body and flags mesh_dirty). The builders themselves
        // live in scene/builders.hpp (single source); unknown tags
        // deliberately leave the node non-parametric (tolerant load path).
        if (const scene::BuilderSpec* spec = scene::find_builder(node.builder))
            node.factory = spec->factory();

        // v0.3.0 (ADR-0027 D4) — the operation recipe (optional). The
        // recipe is parsed HERE (pass 1); reference validation and
        // re-materialization run in pass 2 below, once every node is
        // known. "operand_missing" is NEVER read from the file — it is
        // derived state, recomputed by the pass.
        if (const JsonValue* op_v = node_v.find("operation")) {
            if (!op_v->is_object())
                type_error("object for node \"operation\"", *op_v);
            scene::OperationRecipe recipe;

            const JsonValue& type_v = op_v->at("type");
            if (!type_v.is_string())
                type_error("string for operation \"type\"", type_v);
            const std::string type = type_v.as_string();
            if (type == "subtract") {
                recipe.type = scene::OperationType::Subtract;
            } else {
                throw std::runtime_error(
                    "native format: node " + std::to_string(node.id) +
                    " has an unknown operation type \"" + type +
                    "\" (this build reads \"subtract\"; the file was "
                    "likely written by a newer CAD_0 version — "
                    "ADR-0027 D4)");
            }

            const auto operand_id =
                [&node](const JsonValue& v, const char* field) -> std::uint32_t {
                    if (!v.is_number())
                        type_error(std::string("number for operation \"") +
                                   field + "\"", v);
                    const double d = v.as_number();
                    if (!(d >= 0.0) || d != std::floor(d) ||
                        d > static_cast<double>(
                            std::numeric_limits<std::uint32_t>::max()))
                        throw std::runtime_error(
                            "native format: node " +
                            std::to_string(node.id) + " operation \"" +
                            field + "\" must be a non-negative integer "
                            "(got " + std::to_string(d) + ")");
                    return static_cast<std::uint32_t>(d);
                };
            recipe.target = operand_id(op_v->at("target"), "target");
            recipe.tool = operand_id(op_v->at("tool"), "tool");

            // Offsets live under "params" (the ADR-0027 D4 shape);
            // each defaults to 0 when absent (the same field tolerance
            // PATCH files rely on — a writer always emits all three).
            if (const JsonValue* prm = op_v->find("params")) {
                if (!prm->is_object())
                    type_error("object for operation \"params\"", *prm);
                if (const JsonValue* v = prm->find("offset_x"))
                    recipe.offset_x =
                        static_cast<float>(require_number(*v, "offset_x"));
                if (const JsonValue* v = prm->find("offset_y"))
                    recipe.offset_y =
                        static_cast<float>(require_number(*v, "offset_y"));
                if (const JsonValue* v = prm->find("offset_z"))
                    recipe.offset_z =
                        static_cast<float>(require_number(*v, "offset_z"));
            }
            node.operation = recipe;
        }

        node.body = sdf::SDFBody(deserialize_node_impl(node_v.at("body"), 0));
        doc.nodes.push_back(std::move(node));
    }

    // ---- Pass 2 (ADR-0027 D4): validate the operation DAG over the
    // FULL node list, then re-materialize. Self-references and cycles
    // are structural corruption — explicit load errors (the load stays
    // transactional: nothing was installed yet). A missing operand is
    // NOT corruption: the node keeps its file body and enters the
    // visible error state; a present operand re-materializes the body
    // from the recipe (deterministic — catches stale caches).
    {
        bool has_ops = false;
        for (const auto& n : doc.nodes)
            if (n.operation) { has_ops = true; break; }
        if (has_ops) {
            for (const auto& n : doc.nodes) {
                if (!n.operation) continue;
                // The self-reference pre-check, over the RECIPE'S ROLE
                // LIST (the D2 generalization — an Extrude referencing
                // itself as its own sketch is caught here, exactly as a
                // Subtract referencing itself as its own target).
                const auto roles = scene::recipe_roles(*n.operation);
                for (std::size_t role = 0; role < roles.size(); ++role)
                    if (roles[role] == n.id)
                        throw std::runtime_error(
                            "native format: node " + std::to_string(n.id) +
                            " operation references ITSELF (" +
                            scene::recipe_role_name(*n.operation, role) +
                            " == own id) — structurally corrupt "
                            "(ADR-0027 D2)");
            }
            scene::rebuild_document_operations(doc.nodes);
        }
    }

    // camera (optional — default if absent).
    if (const JsonValue* cam = root.find("camera"))
        doc.camera = json_to_camera(*cam);

    // The full decoded library travels with the document: referenced
    // entries back the scene nodes; unreferenced entries (e.g. the
    // golden file's) survive for byte-identical re-serialization.
    doc.meshes = std::move(mesh_library);

    return doc;
}

ParsedDocument parse_document(std::string_view text) {
    // JSON-only entry point. A document that carries mesh blobs is
    // rejected explicitly: the geometry lives in the file's blob section,
    // not in the text — silently returning header-only meshes (empty
    // geometry) would be a data-loss footgun (ADR-0007a).
    {
        const JsonValue root = json::JsonValue::parse(text);
        if (const JsonValue* meshes_v = root.find("meshes");
            meshes_v && meshes_v->is_array() && !meshes_v->as_array().empty())
            throw std::runtime_error(
                "native format: document carries " +
                std::to_string(meshes_v->as_array().size()) +
                " mesh blob(s); the geometry lives in the file's blob "
                "section, not in the JSON text — use load_file() "
                "(ADR-0007a)");
    }
    return parse_document(text, {});
}

void install_document(scene::Scene& s, ParsedDocument&& doc) {
    s.clear();
    for (auto& node : doc.nodes)
        s.restore_node(std::move(node));
    s.set_next_id(doc.next_id);
    // NOTE (ADR-0007a): doc.meshes — the decoded mesh library — is NOT
    // consumed here in the format layer. The scene layer wires
    // "mesh"-kind nodes to these entries by blob id (see the import
    // feature); callers that need the meshes keep the ParsedDocument.
}

void save_file(const std::filesystem::path& path,
               const scene::Scene& s, const NativeCamera& cam,
               const std::vector<DocumentMesh>& extra_meshes) {
    // Two blocks, one file (ADR-0007a): the JSON document (structure +
    // blob headers) and the blob section (payloads, packed back-to-back
    // in "meshes" array order). collect_document_meshes(s, extra) is the
    // single source of that order — serialize_document performs the
    // SAME collect for its headers, so the JSON offsets and this loop
    // agree by construction (both start from the scene + the SAME extra
    // entries; passing the already-collected list here would double the
    // scene entries and trip the duplicate-id guard).
    const std::vector<DocumentMesh> meshes =
        collect_document_meshes(s, extra_meshes);
    const std::string json_text = serialize_document(s, cam, extra_meshes);
    std::string blob_section;
    for (const auto& dm : meshes)
        blob_section += encode_blob_raw(dm.mesh);
    const std::string file = build_container(json_text, blob_section);

    const std::filesystem::path tmp = path.string() + ".tmp";
    {
        std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
        if (!out)
            throw std::runtime_error("native format: cannot open \"" + tmp.string() +
                                     "\" for writing");
        out.write(file.data(), static_cast<std::streamsize>(file.size()));
        out.flush();
        if (!out) {
            out.close();
            std::error_code ec;
            std::filesystem::remove(tmp, ec);
            throw std::runtime_error("native format: failed writing \"" + tmp.string() + "\"");
        }
#ifndef _WIN32
        // Durability: fsync before rename so a crash never leaves a
        // half-written file at the final path. (POSIX; skipped on Windows.)
        if (FILE* f = std::fopen(tmp.string().c_str(), "rb")) {
            ::fsync(::fileno(f));
            std::fclose(f);
        }
#endif
    }
    // Atomic replace (rename is atomic on POSIX).
    std::error_code ec;
    std::filesystem::rename(tmp, path, ec);
    if (ec) {
        std::filesystem::remove(tmp, ec);
        throw std::runtime_error("native format: cannot replace \"" + path.string() +
                                 "\": " + ec.message());
    }
}

ParsedDocument load_file(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in)
        throw std::runtime_error("native format: cannot open \"" + path.string() +
                                 "\" for reading");
    std::ostringstream ss;
    ss << in.rdbuf();
    if (in.bad())
        throw std::runtime_error("native format: I/O error reading \"" + path.string() + "\"");
    const std::string bytes = ss.str();

    // Framing auto-detection (ADR-0007a), checked BEFORE any parsing:
    //   1. CAD0BLOB magic -> v0.2.0 binary container;
    //   2. leading '{'   -> legacy v0.1.0 pure-JSON (migrator path);
    //   3. anything else -> explicit "not a CAD_0 document" error.
    if (has_container_magic(bytes)) {
        const ContainerParts parts = split_container(bytes);
        return parse_document(parts.json_text, parts.blob_section);
    }
    if (!bytes.empty() && bytes.front() == '{')
        return parse_document(bytes); // legacy pure-JSON v0.1.0

    std::string first_byte = "<none: empty file>";
    if (!bytes.empty()) {
        char buf[8];
        std::snprintf(buf, sizeof(buf), "0x%02X",
                      static_cast<unsigned>(static_cast<unsigned char>(bytes.front())));
        first_byte = buf;
    }
    throw std::runtime_error(
        "native format: cannot open \"" + path.string() +
        "\": not a CAD_0 native document — expected the CAD0BLOB container "
        "or a legacy JSON document (first byte is " + first_byte + ")");
}

} // namespace CAD_0::io::native
