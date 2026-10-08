// core/scene/include/CAD_0/scene/scene.hpp
//
// Scene — a collection of named SDF bodies with transforms.
//
#pragma once

#include "CAD_0/sdf/field.hpp"
#include "CAD_0/sdf/primitives.hpp"
#include "CAD_0/sdf/operators.hpp"
#include "CAD_0/sdf/transforms.hpp"
#include "CAD_0/sdf/mesh_extract.hpp"
#include "CAD_0/math/vec.hpp"
#include "CAD_0/math/mat.hpp"
#include "CAD_0/geometry/transform.hpp"

#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <variant>
#include <vector>

namespace CAD_0::scene {

// A parameter value that can be edited from the property panel.
using ParamValue = std::variant<float, std::string, bool>;

// A shape factory: given the current parameters, rebuild the SDFBody.
// This allows interactive editing — change a radius, the body is rebuilt.
using ShapeFactory = std::function<sdf::SDFBody(const std::map<std::string, ParamValue>&)>;

struct SceneNode {
    std::uint32_t id{0};
    std::string name;
    sdf::SDFBody body;
    geometry::Transform transform{};
    bool visible{true};
    sdf::TriangleMesh cached_mesh;
    bool mesh_dirty{true};

    // Shape factory + parameters for interactive editing.
    // When params change, the body is rebuilt via the factory.
    ShapeFactory factory;
    std::map<std::string, ParamValue> params;

    // Rebuild the SDF body from the factory + current params.
    void rebuild_body() {
        if (factory) {
            body = factory(params);
            mesh_dirty = true;
        }
    }

    // Set a parameter and rebuild if a factory exists.
    void set_param(const std::string& key, ParamValue value) {
        params[key] = std::move(value);
        rebuild_body();
    }
};

enum class SceneChange {
    NodeAdded,
    NodeRemoved,
    NodeModified,
    NodeVisibilityChanged,
    SceneCleared,
};

using SceneChangeCallback = std::function<void(SceneChange, std::uint32_t)>;

class Scene {
public:
    Scene() = default;

    std::uint32_t add_node(const std::string& name, sdf::SDFBody body) {
        SceneNode node;
        node.id = next_id_++;
        node.name = name.empty() ? ("Shape_" + std::to_string(node.id)) : name;
        node.body = std::move(body);
        node.mesh_dirty = true;
        nodes_.push_back(std::move(node));
        notify(SceneChange::NodeAdded, nodes_.back().id);
        return nodes_.back().id;
    }

    // Add a node with a shape factory for interactive editing.
    std::uint32_t add_node(const std::string& name, sdf::SDFBody body,
                           ShapeFactory factory, std::map<std::string, ParamValue> params) {
        SceneNode node;
        node.id = next_id_++;
        node.name = name.empty() ? ("Shape_" + std::to_string(node.id)) : name;
        node.body = std::move(body);
        node.factory = std::move(factory);
        node.params = std::move(params);
        node.mesh_dirty = true;
        nodes_.push_back(std::move(node));
        notify(SceneChange::NodeAdded, nodes_.back().id);
        return nodes_.back().id;
    }

    // Set a parameter on a node (triggers rebuild + re-tessellation).
    void set_node_param(std::uint32_t id, const std::string& key, ParamValue value) {
        if (auto* n = get_node_mut(id)) {
            n->set_param(key, std::move(value));
            notify(SceneChange::NodeModified, id);
        }
    }

    bool remove_node(std::uint32_t id) {
        auto it = std::find_if(nodes_.begin(), nodes_.end(),
            [&](const SceneNode& n) { return n.id == id; });
        if (it == nodes_.end()) return false;
        nodes_.erase(it);
        notify(SceneChange::NodeRemoved, id);
        return true;
    }

    const SceneNode* get_node(std::uint32_t id) const {
        auto it = std::find_if(nodes_.begin(), nodes_.end(),
            [&](const SceneNode& n) { return n.id == id; });
        return (it != nodes_.end()) ? &(*it) : nullptr;
    }

    SceneNode* get_node_mut(std::uint32_t id) {
        auto it = std::find_if(nodes_.begin(), nodes_.end(),
            [&](const SceneNode& n) { return n.id == id; });
        return (it != nodes_.end()) ? &(*it) : nullptr;
    }

    void mark_dirty(std::uint32_t id) {
        if (auto* n = get_node_mut(id)) {
            n->mesh_dirty = true;
            notify(SceneChange::NodeModified, id);
        }
    }

    void set_visible(std::uint32_t id, bool visible) {
        if (auto* n = get_node_mut(id)) {
            n->visible = visible;
            notify(SceneChange::NodeVisibilityChanged, id);
        }
    }

    void clear() {
        nodes_.clear();
        notify(SceneChange::SceneCleared, 0);
    }

    const std::vector<SceneNode>& nodes() const noexcept { return nodes_; }
    std::size_t size() const noexcept { return nodes_.size(); }
    bool empty() const noexcept { return nodes_.empty(); }

    math::Bboxf bounds() const {
        math::Bboxf b;
        for (const auto& n : nodes_) {
            if (!n.visible) continue;
            b.expand(n.body.bounds());
        }
        return b;
    }

    void update_meshes(std::uint32_t resolution = 48) {
        for (auto& n : nodes_) {
            if (n.mesh_dirty && n.visible) {
                n.cached_mesh = sdf::marching_cubes(n.body, resolution, true);
                n.mesh_dirty = false;
            }
        }
    }

    void set_change_callback(SceneChangeCallback cb) {
        callback_ = std::move(cb);
    }

private:
    std::vector<SceneNode> nodes_;
    std::uint32_t next_id_{1};
    SceneChangeCallback callback_;

    void notify(SceneChange change, std::uint32_t node_id) {
        if (callback_) callback_(change, node_id);
    }
};

} // namespace CAD_0::scene
