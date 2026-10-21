// bindings/python/src/scene_bindings.cpp
//
// Python bindings for the Scene class, OrbitCamera, and CommandStack.
//
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include "CAD_0/scene/scene.hpp"
#include "CAD_0/scene/builders.hpp"
#include "CAD_0/scene/operations.hpp"
#include "CAD_0/viewport/camera.hpp"
#include "CAD_0/viewport/command_stack.hpp"
#include "CAD_0/sdf/primitives.hpp"
#include "CAD_0/sdf/operators.hpp"
#include "CAD_0/sdf/transforms.hpp"
#include "CAD_0/sdf/sketch_entities.hpp"
#include "CAD_0/sdf/sketch_profile.hpp"
#include "CAD_0/math/vec.hpp"
#include "CAD_0/math/quat.hpp"

#include <tuple>

namespace nb = nanobind;
using namespace CAD_0;

namespace {

// 2-tuple -> Vec2f for the sketch entity factories (Vec2f is not a
// bound class; the canvas authors entities in plain (x, y) tuples).
math::Vec2f as_vec2(const std::tuple<float, float>& t) {
    return math::Vec2f{std::get<0>(t), std::get<1>(t)};
}
nb::tuple vec2_to_tuple(const math::Vec2f& p) {
    return nb::make_tuple(p.x, p.y);
}

// The recipe as a plain dict — the subtract's shape (Phase F) or the
// extrude's (ADR-0030 D2), dispatched on the type. Read-only from
// Python: edits go through the scene entries (the recipe and the body
// never drift apart).
nb::dict operation_to_dict(const scene::OperationRecipe& r) {
    nb::dict d;
    if (r.type == scene::OperationType::Extrude) {
        d["type"] = "extrude";
        d["sketch"] = r.sketch;
        d["distance"] = r.distance;
        d["direction"] = r.direction;
    } else {
        d["type"] = "subtract";
        d["target"] = r.target;
        d["tool"] = r.tool;
        d["offset_x"] = r.offset_x;
        d["offset_y"] = r.offset_y;
        d["offset_z"] = r.offset_z;
    }
    return nb::dict(d);
}

} // namespace

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
        // Node kind (ADR-0002 honest dispatch): "sdf", "mesh" or
        // "sketch". Python code that assumes an SDF body MUST check
        // this first — a Mesh node has no body to evaluate, a Sketch
        // node is 2D data with no body and no mesh at all.
        .def_prop_ro("kind", [](const scene::SceneNode& n) {
            return scene::node_kind_name(n.kind);
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
        // ---- Operation provenance (Phase F, ADR-0027 D2; ADR-0030 D2) ----
        // The recipe as a plain dict (None when the node is not an
        // operation): subtract -> {"type": "subtract", "target": id,
        // "tool": id, "offset_x"/"offset_y"/"offset_z": float};
        // extrude -> {"type": "extrude", "sketch": id, "distance":
        // float, "direction": ±1}. Read-only from Python: edits go
        // through the Scene entries (which re-materialize through the
        // DAG pass — the recipe and the body never drift apart).
        .def_prop_ro("operation", [](const scene::SceneNode& n) -> nb::object {
            if (!n.operation) return nb::none();
            return nb::object(operation_to_dict(*n.operation));
        })
        // The visible dangling state: an operand was deleted; the node
        // keeps its last materialized body and reports the error
        // (Scene.operation_error carries the actionable message).
        .def_ro("operand_missing", &scene::SceneNode::operand_missing)
        // The visible PENDING state (ADR-0030 D2 revision 1): the
        // sketch operand exists but its profile does not certify (an
        // edit opened the chain). The last materialized body is kept;
        // Scene.operation_error carries the certificate's message.
        .def_ro("operation_pending", &scene::SceneNode::operation_pending);

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

    // ---- Sketch entities + chains (ADR-0030 D2/D3) ------------------
    //
    // The 2D recipe structures the canvas (unit 4) authors and the
    // property panel reads. Entities are addressed by (chain, index)
    // in the sketch node's payload — never by geometric match
    // (ADR-0005).
    nb::class_<sdf::SketchEntity>(scene_mod, "SketchEntity")
        .def_prop_ro("kind", [](const sdf::SketchEntity& e) {
            switch (e.kind) {
            case sdf::SketchEntity::Kind::Segment: return "segment";
            case sdf::SketchEntity::Kind::Circle:  return "circle";
            case sdf::SketchEntity::Kind::Arc:     return "arc";
            }
            return "?";
        });

    scene_mod.def("sketch_segment",
        [](const std::tuple<float, float>& a, const std::tuple<float, float>& b,
           bool interior_left) {
            return sdf::sketch_segment(as_vec2(a), as_vec2(b), interior_left);
        },
        nb::arg("a"), nb::arg("b"), nb::arg("interior_left") = true,
        "A segment entity: the half-plane bounded by the line through "
        "a and b, interior on the left of a->b by default.");
    scene_mod.def("sketch_circle",
        [](const std::tuple<float, float>& center, float radius, bool hole) {
            return sdf::sketch_circle(as_vec2(center), radius, hole);
        },
        nb::arg("center"), nb::arg("radius"), nb::arg("hole") = false,
        "A circle entity: the disc (hole=False) or its complement "
        "(hole=True — the ring's hole).");
    scene_mod.def("sketch_arc",
        [](const std::tuple<float, float>& center, float radius, bool hole,
           float start_angle, float sweep) {
            return sdf::sketch_arc(as_vec2(center), radius, hole, start_angle, sweep);
        },
        nb::arg("center"), nb::arg("radius"), nb::arg("hole") = false,
        nb::arg("start_angle"), nb::arg("sweep"),
        "An arc entity: the curved half-plane (disc or complement) of "
        "the CCW span [start_angle, start_angle + sweep)." );

    nb::class_<sdf::SketchChain>(scene_mod, "SketchChain")
        .def(nb::init<std::vector<sdf::SketchEntity>>(), nb::arg("entities"),
             "An ordered chain of entities, connected "
             "endpoint-to-endpoint (a single circle closes itself).");

    // ---- D5.5: the kernel's per-entity class checks, exposed for the
    // canvas's gesture/text validation (unit 4). ONE opinion: the UI
    // calls the kernel's predicates, it keeps no second copy of the
    // rules.
    scene_mod.def("sketch_entity_error",
        [](const sdf::SketchEntity& e) -> std::string {
            try {
                sdf::validate_sketch_entity(e);
                return "";
            } catch (const std::invalid_argument& ex) {
                return std::string(ex.what());
            }
        },
        nb::arg("entity"),
        "The per-entity construction checks of D3's table "
        "(endpoints finite and p0 != p1; center finite and "
        "r > 0; arc sweep finite in (0, 2π)) — the SAME "
        "predicates the SketchEntitySet certificate runs. "
        "Returns the kernel's error message, or an empty "
        "string when the entity is in class (ADR-0030 D5.5: "
        "the gesture/text validation calls the kernel, never "
        "a UI-side rulebook).");

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
        // ---- Sketch nodes (ADR-0030 D2, unit 2) ----
        //
        // The sketch surface the canvas (unit 4) and the tests drive:
        // creation (with or without initial chains), the entity
        // editing primitives, the readable payload, and the derived
        // definition state. An open chain is a legal editing state —
        // the certificate runs at the extrude, never here.
        .def("add_sketch_node",
             [](scene::Scene& s, const std::string& name) {
                 return s.add_sketch_node(name);
             },
             nb::arg("name") = "",
             "Create an empty sketch node (kind = \"sketch\"): 2D entity "
             "chains on the world XY plane — no body, no mesh, no "
             "factory. Returns the new node id.")
        .def("add_sketch_node",
             [](scene::Scene& s, const std::string& name,
                std::vector<sdf::SketchChain> chains) {
                 return s.add_sketch_node(name, std::move(chains));
             },
             nb::arg("name"), nb::arg("chains"),
             "Create a sketch node with initial chains. Returns the new "
             "node id.")
        .def("restore_sketch_node",
             [](scene::Scene& s, std::uint32_t id, const std::string& name,
                std::vector<sdf::SketchChain> chains) {
                 return s.restore_sketch_node(id, name, std::move(chains));
             },
             nb::arg("node_id"), nb::arg("name"), nb::arg("chains"),
             "Re-create a sketch node with a PINNED id (command redo "
             "of a sketch create/delete). Raises RuntimeError if the id "
             "is in use — never a silent rebind.")
        .def("sketch_add_entity",
             &scene::Scene::sketch_add_entity,
             nb::arg("id"), nb::arg("chain"), nb::arg("entity"),
             "Append an entity to a chain (chain == the current chain "
             "count opens a new chain). Raises RuntimeError on a "
             "non-sketch node.")
        .def("sketch_set_entity",
             &scene::Scene::sketch_set_entity,
             nb::arg("id"), nb::arg("chain"), nb::arg("index"),
             nb::arg("entity"),
             "Replace an entity (a parameter edit). Entities are "
             "addressed by (chain, index).")
        .def("sketch_remove_entity",
             &scene::Scene::sketch_remove_entity,
             nb::arg("id"), nb::arg("chain"), nb::arg("index"),
             "Delete an entity; removing the last entity of a chain "
             "leaves the empty chain (a legal editing state).")
        .def("sketch_closed", &scene::Scene::sketch_closed, nb::arg("id"),
             "The visible definition state: True when every chain is "
             "closed (an empty sketch is not closed). Derived, never "
             "stored — the extrude's closure precondition.")
        .def("sketch_chains",
             [](const scene::Scene& s, std::uint32_t id) -> nb::list {
                 const scene::SceneNode* n = s.get_node(id);
                 if (n == nullptr || n->kind != scene::NodeKind::Sketch)
                     throw std::runtime_error(
                         "sketch_chains: node " + std::to_string(id) +
                         " is not a sketch (ADR-0002 honest dispatch)");
                 nb::list chains;
                 for (const auto& c : n->sketch) {
                     nb::list ents;
                     for (const auto& e : c.entities) {
                         nb::dict d;
                         switch (e.kind) {
                         case sdf::SketchEntity::Kind::Segment:
                             d["kind"] = "segment";
                             d["a"] = vec2_to_tuple(e.segment.a);
                             d["b"] = vec2_to_tuple(e.segment.b);
                             d["interior_left"] = e.segment.interior_left;
                             break;
                         case sdf::SketchEntity::Kind::Circle:
                             d["kind"] = "circle";
                             d["center"] = vec2_to_tuple(e.circle.center);
                             d["radius"] = e.circle.radius;
                             d["hole"] = e.circle.hole;
                             break;
                         case sdf::SketchEntity::Kind::Arc:
                             d["kind"] = "arc";
                             d["center"] = vec2_to_tuple(e.arc.center);
                             d["radius"] = e.arc.radius;
                             d["hole"] = e.arc.hole;
                             d["start_angle"] = e.arc.start_angle;
                             d["sweep"] = e.arc.sweep;
                             break;
                         }
                         ents.append(d);
                     }
                     chains.append(ents);
                 }
                 return chains;
             },
             nb::arg("id"),
             "The sketch payload as [[entity dict, ...], ...] — the "
             "panel/canvas read surface.")
        // ---- D5.3: per-chain closure, the kernel's single opinion ——
        // the same chain_is_closed behind sketch_closed and the
        // extrude's closure precondition. The canvas's amber aspect
        // derives from this, never from a UI-side connectivity copy.
        .def("sketch_chain_closed",
             [](const scene::Scene& s, std::uint32_t id,
                std::size_t chain) {
                 const scene::SceneNode* n = s.get_node(id);
                 if (n == nullptr || n->kind != scene::NodeKind::Sketch)
                     throw std::runtime_error(
                         "sketch_chain_closed: node " +
                         std::to_string(id) +
                         " is not a sketch (ADR-0002 honest dispatch)");
                 if (chain >= n->sketch.size())
                     throw std::runtime_error(
                         "sketch_chain_closed: chain index " +
                         std::to_string(chain) + " is past the end (the "
                         "sketch has " +
                         std::to_string(n->sketch.size()) + " chains)");
                 return sdf::chain_is_closed(n->sketch[chain]);
             },
             nb::arg("id"), nb::arg("chain"),
             "The definition state of ONE chain (chain_is_closed, "
             "the kernel's single opinion). The canvas's amber "
             "aspect derives from this.")
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
        // ---- Extrude (ADR-0030 D2, unit 2) ----
        //
        // The extrude's scene surface, mirroring the subtract's:
        // creation from a LIVE sketch (closure is the precondition —
        // an open sketch raises the certificate's ValueError, nothing
        // is written), id-pinned re-creation for the command redo.
        // The recipe edits (distance/direction) go through unit 4's
        // dialog entries; the canvas does not exist yet.
        .def("add_extrude_node",
             [](scene::Scene& s, const std::string& name,
                std::uint32_t sketch_id, float distance, int direction) {
                 scene::OperationRecipe r;
                 r.type = scene::OperationType::Extrude;
                 r.sketch = sketch_id;
                 r.distance = distance;
                 r.direction = direction;
                 return s.add_operation_node(name, r);
             },
             nb::arg("name") = "", nb::arg("sketch"),
             nb::arg("distance") = 1.0f, nb::arg("direction") = 1,
             "Create an extrude operation from a LIVE closed sketch, "
             "materialized as the prism body (SlabZ-bounded). Raises "
             "ValueError for an open/uncertifiable sketch (nothing "
             "written), RuntimeError for a missing or non-sketch "
             "operand. Returns the new node id.")
        .def("restore_extrude_node",
             [](scene::Scene& s, std::uint32_t node_id, const std::string& name,
                std::uint32_t sketch_id, float distance, int direction) {
                 scene::OperationRecipe r;
                 r.type = scene::OperationType::Extrude;
                 r.sketch = sketch_id;
                 r.distance = distance;
                 r.direction = direction;
                 return s.restore_operation_node(node_id, name, r);
             },
             nb::arg("node_id"), nb::arg("name"), nb::arg("sketch"),
             nb::arg("distance") = 1.0f, nb::arg("direction") = 1,
             "Re-create an extrude operation node with a PINNED id "
             "(command redo). Raises RuntimeError if the id is in use "
             "or the sketch is missing — never a silent rebind.")
        .def("rebuild_operations",
             [](scene::Scene& s) {
                 const scene::OperationRebuildResult r =
                     scene::rebuild_operations(s);
                 nb::dict out;
                 out["rebuilt"] = r.rebuilt;
                 nb::list dang;
                 for (const auto id : r.dangling) dang.append(id);
                 out["dangling"] = dang;
                 nb::list pend;
                 for (const auto id : r.pending) pend.append(id);
                 out["pending"] = pend;
                 return out;
             },
             "The DAG rebuild pass: re-materialize every operation node "
             "from the current operand bodies (topological order). "
             "Returns {\"rebuilt\": count, \"dangling\": [ids], "
             "\"pending\": [ids]}. Raises RuntimeError on cyclic "
             "references (structurally corrupt).")
        .def("set_operation_offset",
             [](scene::Scene& s, std::uint32_t node_id,
                float ox, float oy, float oz) {
                 scene::SceneNode* n = s.get_node_mut(node_id);
                 if (n == nullptr || !n->operation)
                     throw std::runtime_error(
                         "set_operation_offset: node " +
                         std::to_string(node_id) +
                         " is not an operation node");
                 if (n->operation->type != scene::OperationType::Subtract)
                     throw std::runtime_error(
                         "set_operation_offset: node " +
                         std::to_string(node_id) +
                         " is an extrude operation - the tool offset is "
                         "a subtract field (the extrude placement is "
                         "the recipe's distance/direction, ADR-0030 "
                         "D2)");
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
                 nb::list pend;
                 for (const auto id : r.pending) pend.append(id);
                 out["pending"] = pend;
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
