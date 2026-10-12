// core/io/src/native.cpp
//
// Implementation of the native .CAD_0 project format (ADR-0007).
// See native.hpp for the format rationale and API contract.
//
#include "CAD_0/io/native.hpp"

#include "CAD_0/sdf/operators.hpp"
#include "CAD_0/sdf/primitives.hpp"
#include "CAD_0/sdf/transforms.hpp"

#include <cstdio>
#include <fstream>
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

// ---- Migrator registry (Phase A: empty per ADR-0007) -----------------------

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
    static std::map<MigratorKey, Migrator> registry; // empty in Phase A
    return registry;
}

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

std::unique_ptr<sdf::SDFNode> deserialize_node_impl(const JsonValue& v) {
    using namespace sdf;
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
            cs.push_back(deserialize_node_impl(c));
        return std::make_unique<UnionSDF>(std::move(cs));
    }
    if (op == "intersect")
        return std::make_unique<IntersectionSDF>(
            deserialize_node_impl(require_field(v, "a")),
            deserialize_node_impl(require_field(v, "b")));
    if (op == "subtract")
        return std::make_unique<SubtractionSDF>(
            deserialize_node_impl(require_field(v, "a")),
            deserialize_node_impl(require_field(v, "b")));
    if (op == "smooth_min")
        return std::make_unique<SmoothMinSDF>(
            deserialize_node_impl(require_field(v, "a")),
            deserialize_node(require_field(v, "b")),
            static_cast<float>(require_number(require_field(v, "k"), "k")));
    if (op == "translate")
        return std::make_unique<TranslateSDF>(
            deserialize_node_impl(require_field(v, "child")),
            json_to_vec3(require_field(v, "offset"), "offset"));
    if (op == "rotate")
        return std::make_unique<RotateSDF>(
            deserialize_node_impl(require_field(v, "child")),
            json_to_quat(require_field(v, "q"), "q"));
    if (op == "scale")
        return std::make_unique<ScaleSDF>(
            deserialize_node_impl(require_field(v, "child")),
            static_cast<float>(require_number(require_field(v, "s"), "s")));
    if (op == "twist")
        return std::make_unique<TwistSDF>(
            deserialize_node_impl(require_field(v, "child")),
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
    return deserialize_node_impl(v);
}

std::string serialize_document(const scene::Scene& s, const NativeCamera& cam) {
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
        if (!node.builder.empty())
            n.emplace_back("builder", JsonValue(node.builder));
        if (!node.params.empty())
            n.emplace_back("params", params_to_json(node.params));
        n.emplace_back("body", serialize_node_impl(*node.body.root()));
        nodes.push_back(JsonValue(std::move(n)));
    }
    scene_json.emplace_back("nodes", JsonValue(std::move(nodes)));

    JsonValue doc(JsonObject{});
    doc.as_object().emplace_back("format", JsonValue("CAD_0 native"));
    doc.as_object().emplace_back("version", JsonValue(version_to_string(kNativeVersion)));
    doc.as_object().emplace_back("scene", JsonValue(std::move(scene_json)));
    doc.as_object().emplace_back("camera", camera_to_json(cam));
    return doc.dump(true);
}

ParsedDocument parse_document(std::string_view text) {
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
    const NativeVersion file_version = parse_version(version_v.as_string());

    if (!file_version.same_major_minor(kNativeVersion)) {
        // MAJOR or MINOR mismatch → migrator chain.
        const auto& registry = migrator_registry();
        const MigratorKey key{file_version, kNativeVersion};
        const auto it = registry.find(key);
        if (it == registry.end()) {
            throw std::runtime_error(
                "native format: file version " + version_to_string(file_version) +
                " cannot be read by this build (expects " +
                version_to_string(kNativeVersion) +
                "): no migrator registered for that upgrade — "
                "ADR-0007 requires an explicit migrator for MAJOR/MINOR changes");
        }
        it->second(root);
    }
    // PATCH differences are forward/backward compatible (ADR-0007).

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
        node.mesh_dirty = true; // always re-tessellate after load (no cached blobs in Phase A)

        if (const JsonValue* name = node_v.find("name")) {
            if (!name->is_string()) type_error("string for node \"name\"", *name);
            node.name = name->as_string();
        } else {
            node.name = "Shape_" + std::to_string(node.id);
        }

        if (const JsonValue* vis = node_v.find("visible")) {
            if (!vis->is_bool()) type_error("bool for node \"visible\"", *vis);
            node.visible = vis->as_bool();
        }

        if (const JsonValue* tr = node_v.find("transform"))
            node.transform = json_to_transform(*tr);

        if (const JsonValue* builder = node_v.find("builder")) {
            if (!builder->is_string()) type_error("string for node \"builder\"", *builder);
            node.builder = builder->as_string();
        }

        if (const JsonValue* params = node_v.find("params"))
            node.params = json_to_params(*params);

        node.body = sdf::SDFBody(deserialize_node_impl(node_v.at("body")));
        doc.nodes.push_back(std::move(node));
    }

    // camera (optional — default if absent).
    if (const JsonValue* cam = root.find("camera"))
        doc.camera = json_to_camera(*cam);

    return doc;
}

void install_document(scene::Scene& s, ParsedDocument&& doc) {
    s.clear();
    for (auto& node : doc.nodes)
        s.restore_node(std::move(node));
    s.set_next_id(doc.next_id);
}

void save_file(const std::filesystem::path& path,
               const scene::Scene& s, const NativeCamera& cam) {
    const std::string text = serialize_document(s, cam);

    const std::filesystem::path tmp = path.string() + ".tmp";
    {
        std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
        if (!out)
            throw std::runtime_error("native format: cannot open \"" + tmp.string() +
                                     "\" for writing");
        out.write(text.data(), static_cast<std::streamsize>(text.size()));
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
    return parse_document(ss.str());
}

} // namespace CAD_0::io::native
