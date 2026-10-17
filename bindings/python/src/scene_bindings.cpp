// bindings/python/src/scene_bindings.cpp
//
// Python bindings for the Scene class, OrbitCamera, and CommandStack.
//
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/unique_ptr.h>

#include "CAD_0/scene/scene.hpp"
#include "CAD_0/scene/builders.hpp"
#include "CAD_0/scene/operations.hpp"
#include "CAD_0/viewport/camera.hpp"
#include "CAD_0/viewport/command_stack.hpp"
#include "CAD_0/sdf/primitives.hpp"
#include "CAD_0/sdf/operators.hpp"
#include "CAD_0/sdf/transforms.hpp"
#include "CAD_0/math/vec.hpp"
#include "CAD_0/math/quat.hpp"

namespace nb = nanobind;
using namespace CAD_0;

void bind_scene(nb::module_& m) {
    auto scene_mod = m.def_submodule("scene", "Scene management");

    // SceneChange enum
    nb::enum_<scene::SceneChange>(scene_mod, "SceneChange")
        .value("NodeAdded", scene::SceneChange::NodeAdded)
        .value("NodeRemoved", scene::SceneChange::NodeRemoved)
        .value("NodeModified", scene::SceneChange::NodeModified)
        .value("NodeVisibilityChanged", scene::SceneChange::NodeVisibilityChanged)
        .value("SceneCleared", scene::SceneChange::SceneCleared);

    // SceneNode — readable view; `body` and `mesh_dirty` are writable so the
    // UI can rebuild a node's SDF interactively (Bug 1 fix: assigning to a
    // read-only property raised "AttributeError: property of 'SceneNode'
    // object has no setter" and killed the app during parameter editing).
    nb::class_<scene::SceneNode>(scene_mod, "SceneNode")
        .def_ro("id", &scene::SceneNode::id)
        // `name` is writable (ADR-0027 D5): the subtract command names
        // its node after its operands ("Subtract (Box#1 - Cylinder#2)")
        // so the row, the file and the Debug report all identify what
        // was subtracted.
        .def_rw("name", &scene::SceneNode::name)
        .def_ro("visible", &scene::SceneNode::visible)
        // Node kind (ADR-0002 honest dispatch): "sdf" or "mesh".
        // Python code that assumes an SDF body MUST check this first —
        // a Mesh node has no body to evaluate, no params, no factory.
        .def_prop_ro("kind", [](const scene::SceneNode& n) {
            return n.kind == scene::NodeKind::Mesh ? "mesh" : "sdf";
        })
        // Mesh-kind payload: THE geometry (imported STL/OBJ).
        .def_prop_ro("mesh", [](const scene::SceneNode& n) -> const sdf::TriangleMesh& {
            return n.mesh;
        }, nb::rv_policy::reference_internal)
        .def_rw("mesh_dirty", &scene::SceneNode::mesh_dirty)
        .def_prop_rw("body",
            [](const scene::SceneNode& n) -> const sdf::SDFBody& {
                return n.body;
            },
            [](scene::SceneNode& n, const sdf::SDFBody& b) {
                // Sdf-kind only: assigning an SDF body to a Mesh node is
                // a category error (the node would claim BOTH
                // representations — exactly the ambiguity ADR-0002
                // forbids). Fail loudly instead of corrupting the node.
                if (n.kind == scene::NodeKind::Mesh)
                    throw std::runtime_error(
                        "SceneNode.body: cannot assign an SDF body to a "
                        "\"mesh\"-kind node — the node's geometry is its "
                        "imported TriangleMesh payload (ADR-0002)");
                // Deep-clone the incoming tree (SDFBody copy ctor clones the
                // subtree) and flag the cached mesh for re-tessellation.
                n.body = b;
                n.mesh_dirty = true;
            },
            nb::rv_policy::reference_internal)
        // The mesh every consumer (viewport, export) sees: the
        // tessellation cache for Sdf nodes, THE geometry for Mesh
        // nodes (display_mesh() dispatch — see scene.hpp).
        .def_prop_ro("cached_mesh", [](const scene::SceneNode& n) -> const sdf::TriangleMesh& {
            return n.display_mesh();
        }, nb::rv_policy::reference_internal)
        .def("rebuild_body", &scene::SceneNode::rebuild_body)
        .def("set_param", [](scene::SceneNode& n, const std::string& key, float v) {
            n.set_param(key, v);
        }, nb::arg("key"), nb::arg("value"))
        // Builder tag + params — the editing metadata persisted by the
        // native format (ADR-0007). `builder` names the registered
        // NodeBuilder (e.g. "sphere"); `params` round-trips through
        // save/load so the property panel keeps working after a reopen.
        .def_rw("builder", &scene::SceneNode::builder)
        .def_prop_ro("params", [](const scene::SceneNode& n) {
            nb::dict out;
            for (const auto& [key, value] : n.params) {
                if (auto* f = std::get_if<float>(&value)) out[key.c_str()] = *f;
                else if (auto* s = std::get_if<std::string>(&value)) out[key.c_str()] = *s;
                else if (auto* b = std::get_if<bool>(&value)) out[key.c_str()] = *b;
            }
            return out;
        })
        .def("set_params", [](scene::SceneNode& n, const nb::dict& d) {
            for (auto [k, v] : d) {
                const std::string key = nb::cast<std::string>(k);
                if (nb::isinstance<nb::float_>(v) || nb::isinstance<nb::int_>(v))
                    n.params[key] = static_cast<float>(nb::cast<double>(v));
                else if (nb::isinstance<nb::str>(v))
                    n.params[key] = nb::cast<std::string>(v);
                else if (nb::isinstance<nb::bool_>(v))
                    n.params[key] = nb::cast<bool>(v);
                else
                    throw std::runtime_error("param \"" + key +
                                             "\" must be number, string or bool");
            }
        }, nb::arg("params"))
        // ---- Operation provenance (Phase F, ADR-0027 D2) ----
        // The recipe as a plain dict (None when the node is not an
        // operation): {"type": "subtract", "target": id, "tool": id,
        // "offset_x"/"offset_y"/"offset_z": float}. Read-only from
        // Python: edits go through Scene.set_operation_offset (which
        // re-materializes through the DAG pass — the recipe and the
        // body never drift apart).
        .def_prop_ro("operation", [](const scene::SceneNode& n) -> nb::object {
            if (!n.operation) return nb::none();
            nb::dict d;
            d["type"] = n.operation->type == scene::OperationType::Subtract
                            ? "subtract" : "?";
            d["target"] = n.operation->target;
            d["tool"] = n.operation->tool;
            d["offset_x"] = n.operation->offset_x;
            d["offset_y"] = n.operation->offset_y;
            d["offset_z"] = n.operation->offset_z;
            return nb::object(d);
        })
        // The visible dangling state: an operand was deleted; the node
        // keeps its last materialized body and reports the error
        // (Scene.operation_error carries the actionable message).
        .def_ro("operand_missing", &scene::SceneNode::operand_missing);

    // ---- Parametric builder registry (single source of truth) -----------
    //
    // core/scene/builders.hpp defines the builders (tags, param schemas,
    // defaults, body construction) ONCE; these bindings expose the
    // registry so the desktop UI consumes only the kernel API:
    //   builder_tags()   -> ["sphere", "box", ...] (Create-menu order)
    //   builder_specs()  -> [{"tag", "display_name", "params": {name: default}}]
    //   default_params(tag) -> {name: default}   (RuntimeError on unknown tag)
    //   make_body(tag, params) -> SDFBody        (RuntimeError on unknown tag;
    //                                              missing keys fall back to
    //                                              the table defaults)
    //   has_builder(tag) -> bool
    // The unknown-tag contract is deliberately different from the load
    // path: loading a file with an unknown builder tag stays tolerant
    // (non-parametric but loadable, ADR-0007 PATCH compatibility), while
    // asking the registry to BUILD one is a programming error and fails
    // loudly on both sides.
    scene_mod.def("builder_tags", []() {
        nb::list out;
        for (const auto& tag : scene::builder_tags()) out.append(tag);
        return out;
    });

    scene_mod.def("builder_specs", []() {
        nb::list out;
        for (const auto& spec : scene::builder_registry()) {
            nb::dict d;
            d["tag"] = spec.tag;
            d["display_name"] = spec.display_name;
            nb::dict params;
            for (const auto& p : spec.params) params[p.name.c_str()] = p.default_value;
            d["params"] = params;
            out.append(d);
        }
        return out;
    });

    scene_mod.def("default_params", [](const std::string& tag) {
        nb::dict out;
        for (const auto& [key, value] : scene::default_params(tag)) {
            if (auto* f = std::get_if<float>(&value)) out[key.c_str()] = *f;
            else if (auto* s = std::get_if<std::string>(&value)) out[key.c_str()] = *s;
            else if (auto* b = std::get_if<bool>(&value)) out[key.c_str()] = *b;
        }
        return out;
    }, nb::arg("tag"),
       "Default parameters for a registered builder. Raises RuntimeError "
       "for an unknown tag.");

    scene_mod.def("make_body", [](const std::string& tag, const nb::dict& params) {
        std::map<std::string, scene::ParamValue> p;
        for (auto [k, v] : params) {
            const std::string key = nb::cast<std::string>(k);
            if (nb::isinstance<nb::float_>(v) || nb::isinstance<nb::int_>(v))
                p[key] = static_cast<float>(nb::cast<double>(v));
            else if (nb::isinstance<nb::str>(v))
                p[key] = nb::cast<std::string>(v);
            else if (nb::isinstance<nb::bool_>(v))
                p[key] = nb::cast<bool>(v);
            else
                throw std::runtime_error("param \"" + key +
                                         "\" must be number, string or bool");
        }
        return scene::make_body(tag, p); // throws for an unknown tag
    }, nb::arg("tag"), nb::arg("params"),
       "Build an SDFBody from a builder tag + params dict. Missing keys "
       "fall back to the registry defaults. Raises RuntimeError for an "
       "unknown tag.");

    scene_mod.def("has_builder", [](const std::string& tag) {
        return scene::find_builder(tag) != nullptr;
    }, nb::arg("tag"));

    // Scene
    nb::class_<scene::Scene>(scene_mod, "Scene")
        .def(nb::init<>())
        .def("add_node", [](scene::Scene& s, const std::string& name, sdf::SDFBody body) {
            return s.add_node(name, std::move(body));
        }, nb::arg("name"), nb::arg("body"))
        // Imported-mesh node: kind = "mesh", geometry = the payload
        // (welded by the loader). No body/factory/params — see
        // SceneNode.kind (ADR-0002).
        .def("add_node", [](scene::Scene& s, const std::string& name,
                            sdf::TriangleMesh mesh) {
            return s.add_node(name, std::move(mesh));
        }, nb::arg("name"), nb::arg("mesh"))
        .def("remove_node", &scene::Scene::remove_node, nb::arg("id"))
        .def("get_node", [](scene::Scene& s, std::uint32_t id) -> scene::SceneNode* {
            return s.get_node_mut(id);
        }, nb::arg("id"), nb::rv_policy::reference_internal)
        .def("mark_dirty", &scene::Scene::mark_dirty, nb::arg("id"))
        .def("set_visible", &scene::Scene::set_visible, nb::arg("id"), nb::arg("visible"))
        .def("set_node_param", [](scene::Scene& s, std::uint32_t id, const std::string& key, float v) {
            s.set_node_param(id, key, v);
        }, nb::arg("id"), nb::arg("key"), nb::arg("value"))
        .def("clear", &scene::Scene::clear)
        .def("size", &scene::Scene::size)
        .def("empty", &scene::Scene::empty)
        .def("bounds", &scene::Scene::bounds)
        .def("update_meshes", &scene::Scene::update_meshes, nb::arg("resolution") = 48)
        .def("node_count", &scene::Scene::node_count)
        .def("node_at", [](scene::Scene& s, std::size_t i) -> scene::SceneNode* {
            return s.node_at(i);
        }, nb::arg("index"), nb::rv_policy::reference_internal)
        // ---- change notifications + native-format hooks ----
        //
        // set_change_callback(cb) installs a callable Scene(cb_kind, node_id).
        // All Scene mutations fire it (NodeAdded/NodeRemoved/NodeModified/
        // NodeVisibilityChanged/SceneCleared) — the UI uses it for its
        // dirty flag. Pass None to detach (done on window close to break
        // the Python↔C++ reference cycle).
        //
        // NOTE on thread-safety: callbacks fire from C++ while the GIL is
        // held (all Scene methods are invoked with it held; the native
        // loader re-acquires it before installing nodes), so calling back
        // into Python here is safe.
        .def("set_change_callback",
             [](scene::Scene& s, std::function<void(scene::SceneChange, std::uint32_t)> cb) {
                 // Passing None (accepted via .none()) yields an empty
                 // std::function — i.e. detach. This breaks the Python↔C++
                 // reference cycle when the window closes.
                 s.set_change_callback(std::move(cb));
             },
             nb::arg("callback").none())
        .def("next_id", &scene::Scene::next_id)
        // ---- Phase F operations (ADR-0027 D2) ----
        //
        // The subtract feature's scene surface. add_operation_node
        // creates the operation from LIVE operands (a later deletion
        // is what produces the visible dangling state — never a silent
        // rebind); restore_operation_node / restore_sdf_node are the
        // id-pinned re-creation entries the undo/redo commands use
        // (a redo that would land on a live id raises — references
        // never drift); rebuild_operations is the DAG pass (edit
        // propagation + dangling detection + recovery, one call);
        // set_operation_offset edits the placement and re-materializes
        // through the same pass; operation_error carries the
        // actionable dangling message for the property panel.
        .def("add_operation_node",
             [](scene::Scene& s, const std::string& name,
                std::uint32_t target, std::uint32_t tool,
                float ox, float oy, float oz) {
                 scene::OperationRecipe r;
                 r.type = scene::OperationType::Subtract;
                 r.target = target;
                 r.tool = tool;
                 r.offset_x = ox;
                 r.offset_y = oy;
                 r.offset_z = oz;
                 return s.add_operation_node(name, r);
             },
             nb::arg("name"), nb::arg("target"), nb::arg("tool"),
             nb::arg("offset_x") = 0.0f, nb::arg("offset_y") = 0.0f,
             nb::arg("offset_z") = 0.0f,
             "Create a subtract operation node referencing two live "
             "SDF operands by id, materialized from their current "
             "bodies (the tool offset is in world coordinates). "
             "Returns the new node id.")
        .def("restore_operation_node",
             [](scene::Scene& s, std::uint32_t node_id,
                const std::string& name,
                std::uint32_t target, std::uint32_t tool,
                float ox, float oy, float oz) {
                 scene::OperationRecipe r;
                 r.type = scene::OperationType::Subtract;
                 r.target = target;
                 r.tool = tool;
                 r.offset_x = ox;
                 r.offset_y = oy;
                 r.offset_z = oz;
                 return s.restore_operation_node(node_id, name, r);
             },
             nb::arg("node_id"), nb::arg("name"), nb::arg("target"),
             nb::arg("tool"), nb::arg("offset_x") = 0.0f,
             nb::arg("offset_y") = 0.0f, nb::arg("offset_z") = 0.0f,
             "Re-create a subtract operation node with a PINNED id "
             "(command redo). Raises RuntimeError if the id is in use "
             "or an operand is missing — never a silent rebind.")
        .def("restore_sdf_node",
             [](scene::Scene& s, std::uint32_t node_id,
                const std::string& name, sdf::SDFBody body) {
                 return s.restore_sdf_node(node_id, name, std::move(body));
             },
             nb::arg("node_id"), nb::arg("name"), nb::arg("body"),
             "Re-create a plain SDF node with a PINNED id (command "
             "redo of create/delete). Builder tag + params are "
             "re-attached by the caller. Raises RuntimeError if the id "
             "is in use.")
        .def("rebuild_operations",
             [](scene::Scene& s) {
                 const scene::OperationRebuildResult r =
                     scene::rebuild_operations(s);
                 nb::dict out;
                 out["rebuilt"] = r.rebuilt;
                 nb::list dang;
                 for (const auto id : r.dangling) dang.append(id);
                 out["dangling"] = dang;
                 return out;
             },
             "The DAG rebuild pass: re-materialize every operation node "
             "from the current operand bodies (topological order). "
             "Returns {\"rebuilt\": count, \"dangling\": [ids]}. Raises "
             "RuntimeError on cyclic references (structurally corrupt).")
        .def("set_operation_offset",
             [](scene::Scene& s, std::uint32_t node_id,
                float ox, float oy, float oz) {
                 scene::SceneNode* n = s.get_node_mut(node_id);
                 if (n == nullptr || !n->operation)
                     throw std::runtime_error(
                         "set_operation_offset: node " +
                         std::to_string(node_id) +
                         " is not an operation node");
                 n->operation->offset_x = ox;
                 n->operation->offset_y = oy;
                 n->operation->offset_z = oz;
                 const scene::OperationRebuildResult r =
                     scene::rebuild_operations(s);
                 nb::dict out;
                 out["rebuilt"] = r.rebuilt;
                 nb::list dang;
                 for (const auto id : r.dangling) dang.append(id);
                 out["dangling"] = dang;
                 return out;
             },
             nb::arg("node_id"), nb::arg("offset_x"), nb::arg("offset_y"),
             nb::arg("offset_z"),
             "Edit an operation's tool placement (world coordinates) "
             "and re-materialize through the DAG pass. Returns the "
             "rebuild result dict (see rebuild_operations).")
        .def("operation_error",
             [](const scene::Scene& s, std::uint32_t node_id) {
                 return scene::operation_error(s, node_id);
             },
             nb::arg("node_id"),
             "The actionable dangling message for an operation node "
             "(empty string when the node is not dangling).");

    // OrbitCamera — bind to Python so the viewport widget can use it
    auto cam_mod = m.def_submodule("camera", "Camera system");

    nb::class_<viewport::OrbitCamera>(cam_mod, "OrbitCamera")
        .def(nb::init<>())
        .def(nb::init<const math::Vec3f&, float, float, float>(),
             nb::arg("target"), nb::arg("distance"),
             nb::arg("yaw") = 0.0f, nb::arg("pitch") = 0.3f)
        .def_prop_ro("target", [](const viewport::OrbitCamera& c) { return c.target(); })
        .def("distance", &viewport::OrbitCamera::distance)
        .def("yaw", &viewport::OrbitCamera::yaw)
        .def("pitch", &viewport::OrbitCamera::pitch)
        .def("fov_y", &viewport::OrbitCamera::fov_y)
        .def("set_fov_y", &viewport::OrbitCamera::set_fov_y)
        .def("aspect", &viewport::OrbitCamera::aspect)
        .def("set_aspect", &viewport::OrbitCamera::set_aspect)
        .def("near_plane", &viewport::OrbitCamera::near_plane)
        .def("far_plane", &viewport::OrbitCamera::far_plane)
        .def("set_clip_planes", &viewport::OrbitCamera::set_clip_planes)
        .def("position", &viewport::OrbitCamera::position)
        .def("forward", &viewport::OrbitCamera::forward)
        .def("right", &viewport::OrbitCamera::right)
        .def("up", &viewport::OrbitCamera::up)
        .def("orbit", &viewport::OrbitCamera::orbit, nb::arg("delta_yaw"), nb::arg("delta_pitch"))
        .def("pan", &viewport::OrbitCamera::pan, nb::arg("dx_screen"), nb::arg("dy_screen"))
        .def("zoom", &viewport::OrbitCamera::zoom, nb::arg("factor"))
        .def("frame_bounds", [](viewport::OrbitCamera& c, const math::Bboxf& b) {
            c.frame_bounds(b);
        }, nb::arg("bounds"))
        .def("view_matrix", &viewport::OrbitCamera::view_matrix)
        .def("projection_matrix", &viewport::OrbitCamera::projection_matrix)
        .def("view_projection_matrix", &viewport::OrbitCamera::view_projection_matrix);

    // CommandStack — undo/redo support.
    // CommandStack is move-only (contains vector<unique_ptr<Command>>),
    // so we use nb::is_final to prevent nanobind from trying to generate
    // a copy constructor.
    //
    // Bug 4 note: next_undo_description()/next_redo_description() return
    // `const std::string&` pointing at a string owned by the Command
    // inside the stack. They used to be bound with `rv_policy::reference`,
    // which does NOT tie the lifetime to the stack — safe today only
    // because nanobind's std::string caster copies the characters into an
    // immutable Python str. The (misleading) policy is removed; the copy
    // semantics are what we actually want.
    auto cmd_mod = m.def_submodule("commands", "Undo/redo command stack");

    nb::class_<viewport::Command>(cmd_mod, "Command")
        .def("execute", &viewport::Command::execute)
        .def("undo", &viewport::Command::undo)
        .def("description", &viewport::Command::description);

    nb::class_<viewport::LambdaCommand, viewport::Command>(cmd_mod, "LambdaCommand")
        .def(nb::init<std::function<void()>, std::function<void()>, std::string>(),
             nb::arg("execute_fn"), nb::arg("undo_fn"), nb::arg("desc") = "command");

    nb::class_<viewport::CommandStack>(cmd_mod, "CommandStack", nb::is_final())
        .def(nb::init<>())
        .def(nb::init<std::size_t>(), nb::arg("max_size"))
        .def("push", [](viewport::CommandStack& cs, std::function<void()> exec_fn,
                        std::function<void()> undo_fn, std::string desc) {
            cs.push(std::move(exec_fn), std::move(undo_fn), std::move(desc));
        }, nb::arg("execute_fn"), nb::arg("undo_fn"), nb::arg("desc") = "command")
        .def("undo", &viewport::CommandStack::undo)
        .def("redo", &viewport::CommandStack::redo)
        .def("can_undo", &viewport::CommandStack::can_undo)
        .def("can_redo", &viewport::CommandStack::can_redo)
        .def("undo_count", &viewport::CommandStack::undo_count)
        .def("redo_count", &viewport::CommandStack::redo_count)
        .def("next_undo_description", &viewport::CommandStack::next_undo_description)
        .def("next_redo_description", &viewport::CommandStack::next_redo_description)
        .def("clear", &viewport::CommandStack::clear)
        .def("max_size", &viewport::CommandStack::max_size)
        .def("set_max_size", &viewport::CommandStack::set_max_size);
}
