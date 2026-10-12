// core/scene/include/CAD_0/scene/builders.hpp
//
// Parametric shape builder registry — the SINGLE SOURCE OF TRUTH for the
// builder tags, their editable parameter schemas and their defaults.
//
// Before this registry existed, the same knowledge lived in two copies
// that could silently diverge: core/io/src/native.cpp
// (make_builder_factory — post-load factory re-wiring) and the desktop
// UI (app/src/CAD_0/ui/main_window.py: _make_default_params /
// _make_body / _rebuild_node / on_create_*). Now both are consumers:
//   * io/native.cpp re-wires loaded nodes through find_builder();
//   * the Python bindings expose the registry (CAD_0.scene.builder_tags
//     / builder_specs / default_params / make_body / has_builder) and
//     the UI consumes ONLY those.
//
// Semantics (unchanged from the previous copies — the on-disk format and
// the golden file are untouched):
//   * tags + param names are exactly what the native format persists
//     as `builder` + `params` (ADR-0007);
//   * an UNKNOWN tag on the LOAD path stays gracefully non-parametric
//     (the body is fully serialized — PATCH compatibility);
//   * an UNKNOWN tag on the EXPLICIT API (default_params / make_body)
//     is an explicit std::runtime_error — the kernel and the UI both
//     fail loudly instead of guessing.
//
// scene is a header-only INTERFACE library, so the registry is inline
// (function-local statics guarantee a single instance across TUs).
//
#pragma once

#include "CAD_0/scene/scene.hpp" // ParamValue, ShapeFactory

#include "CAD_0/math/vec.hpp"
#include "CAD_0/sdf/primitives.hpp"

#include <functional>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace CAD_0::scene {

// One editable numeric parameter of a builder: name + default value.
// The name is persisted as a key of the node's `params` map.
struct BuilderParam {
    std::string name;
    float default_value{0.0f};
};

// Read a float param, falling back to `fallback` when the key is absent
// or holds a non-float value (the historical make_builder_factory
// semantics — kept so set_param on a wrong-typed value degrades the same
// way it always did).
inline float param_float(const std::map<std::string, ParamValue>& params,
                         const std::string& key, float fallback) {
    if (auto it = params.find(key); it != params.end())
        if (const auto* f = std::get_if<float>(&it->second)) return *f;
    return fallback;
}

// A parametric shape builder.
//
// `params` is THE parameter table: the property-panel schema AND the
// persisted defaults. `build` constructs the SDF body from a parameter
// map; make() fills missing keys from the table first, so `build` always
// sees a complete map (its own fallbacks are belt-and-braces and must
// mirror the table — pinned by tests/unit/scene/test_builders.cpp).
struct BuilderSpec {
    std::string tag;          // "sphere" — persisted as SceneNode::builder
    std::string display_name; // "Sphere" — UI labels (Create menu, status)
    std::vector<BuilderParam> params;

    // SDFBody(const std::map<std::string, ParamValue>&) — reads a map
    // that make() has already merged with the defaults above.
    std::function<sdf::SDFBody(const std::map<std::string, ParamValue>&)> build;

    // The default parameter map (schema keys -> defaults).
    std::map<std::string, ParamValue> default_params() const {
        std::map<std::string, ParamValue> out;
        for (const auto& p : params) out[p.name] = p.default_value;
        return out;
    }

    // Merge the defaults under `overrides` (existing keys win — emplace
    // does not overwrite) and build the body.
    sdf::SDFBody make(std::map<std::string, ParamValue> overrides) const {
        for (const auto& p : params)
            overrides.emplace(p.name, p.default_value);
        if (!build)
            throw std::runtime_error("builder registry: spec \"" + tag +
                                     "\" has no build function");
        return build(overrides);
    }

    // A ShapeFactory suitable for SceneNode::factory (used by the load
    // path's re-wiring and by anything that wants parametric editing).
    // The spec is captured by value so the lambda is self-contained.
    ShapeFactory factory() const;
};

inline ShapeFactory BuilderSpec::factory() const {
    BuilderSpec copy = *this;
    return [spec = std::move(copy)](const std::map<std::string, ParamValue>& p) {
        return spec.make(p);
    };
}

// ---- The registry ------------------------------------------------------------

// All registered builders, in Create-menu order. This table is the one
// place where tags, parameter schemas, defaults and body construction
// are defined; everything else (io load path, Python bindings, desktop
// UI) consumes it.
inline const std::vector<BuilderSpec>& builder_registry() {
    static const std::vector<BuilderSpec> registry = {
        {"sphere", "Sphere",
         {{"radius", 1.0f}},
         [](const std::map<std::string, ParamValue>& p) {
             return sdf::make_sphere(param_float(p, "radius", 1.0f));
         }},
        {"box", "Box",
         {{"extent_x", 0.5f}, {"extent_y", 0.5f}, {"extent_z", 0.5f}},
         [](const std::map<std::string, ParamValue>& p) {
             return sdf::make_box(math::Vec3f{
                 param_float(p, "extent_x", 0.5f),
                 param_float(p, "extent_y", 0.5f),
                 param_float(p, "extent_z", 0.5f)});
         }},
        {"cylinder", "Cylinder",
         {{"radius", 0.5f}, {"height", 2.0f}},
         [](const std::map<std::string, ParamValue>& p) {
             return sdf::make_cylinder(param_float(p, "radius", 0.5f),
                                       param_float(p, "height", 2.0f));
         }},
        {"torus", "Torus",
         {{"major_radius", 1.0f}, {"minor_radius", 0.3f}},
         [](const std::map<std::string, ParamValue>& p) {
             return sdf::make_torus(param_float(p, "major_radius", 1.0f),
                                    param_float(p, "minor_radius", 0.3f));
         }},
    };
    return registry;
}

// Look up a builder by tag; nullptr when not registered (the LOAD path
// relies on this to keep unknown tags gracefully non-parametric).
inline const BuilderSpec* find_builder(const std::string& tag) {
    for (const auto& spec : builder_registry())
        if (spec.tag == tag) return &spec;
    return nullptr;
}

// All registered tags, in registry order.
inline std::vector<std::string> builder_tags() {
    std::vector<std::string> tags;
    tags.reserve(builder_registry().size());
    for (const auto& spec : builder_registry()) tags.push_back(spec.tag);
    return tags;
}

namespace detail {
inline std::string registered_tags_list() {
    std::string out;
    for (const auto& tag : builder_tags()) {
        if (!out.empty()) out += ", ";
        out += "\"" + tag + "\"";
    }
    return out;
}
} // namespace detail

// The defaults for one builder. Unknown tag → explicit error (this is
// the creation/editing API, not the tolerant load path).
inline std::map<std::string, ParamValue> default_params(const std::string& tag) {
    const BuilderSpec* spec = find_builder(tag);
    if (!spec)
        throw std::runtime_error("builder registry: unknown builder tag \"" +
                                 tag + "\" (registered: " +
                                 detail::registered_tags_list() + ")");
    return spec->default_params();
}

// Build a body for one builder; missing params fall back to the table
// defaults. Unknown tag → explicit error.
inline sdf::SDFBody make_body(const std::string& tag,
                              const std::map<std::string, ParamValue>& params) {
    const BuilderSpec* spec = find_builder(tag);
    if (!spec)
        throw std::runtime_error("builder registry: unknown builder tag \"" +
                                 tag + "\" (registered: " +
                                 detail::registered_tags_list() + ")");
    return spec->make(params);
}

} // namespace CAD_0::scene
