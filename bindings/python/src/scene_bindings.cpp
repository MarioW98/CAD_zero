// bindings/python/src/scene_bindings.cpp
//
// Python bindings for the Scene class, OrbitCamera, and CommandStack.
//
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/unique_ptr.h>

#include "CAD_0/scene/scene.hpp"
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

    // SceneNode — read-only view
    nb::class_<scene::SceneNode>(scene_mod, "SceneNode")
        .def_ro("id", &scene::SceneNode::id)
        .def_ro("name", &scene::SceneNode::name)
        .def_ro("visible", &scene::SceneNode::visible)
        .def_ro("mesh_dirty", &scene::SceneNode::mesh_dirty)
        .def_prop_ro("body", [](const scene::SceneNode& n) -> const sdf::SDFBody& {
            return n.body;
        }, nb::rv_policy::reference_internal)
        .def_prop_ro("cached_mesh", [](const scene::SceneNode& n) -> const sdf::TriangleMesh& {
            return n.cached_mesh;
        }, nb::rv_policy::reference_internal)
        .def("rebuild_body", &scene::SceneNode::rebuild_body)
        .def("set_param", [](scene::SceneNode& n, const std::string& key, float v) {
            n.set_param(key, v);
        }, nb::arg("key"), nb::arg("value"));

    // Scene
    nb::class_<scene::Scene>(scene_mod, "Scene")
        .def(nb::init<>())
        .def("add_node", [](scene::Scene& s, const std::string& name, sdf::SDFBody body) {
            return s.add_node(name, std::move(body));
        }, nb::arg("name"), nb::arg("body"))
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
        .def_prop_ro("nodes", [](scene::Scene& s) {
            return &s.nodes();
        }, nb::rv_policy::reference_internal);

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

    // CommandStack — undo/redo support
    auto cmd_mod = m.def_submodule("commands", "Undo/redo command stack");

    nb::class_<viewport::Command>(cmd_mod, "Command")
        .def("execute", &viewport::Command::execute)
        .def("undo", &viewport::Command::undo)
        .def("description", &viewport::Command::description);

    nb::class_<viewport::LambdaCommand, viewport::Command>(cmd_mod, "LambdaCommand")
        .def(nb::init<std::function<void()>, std::function<void()>, const char*>(),
             nb::arg("execute_fn"), nb::arg("undo_fn"), nb::arg("desc") = "command");

    nb::class_<viewport::CommandStack>(cmd_mod, "CommandStack")
        .def(nb::init<>())
        .def(nb::init<std::size_t>(), nb::arg("max_size"))
        .def("push", [](viewport::CommandStack& cs, std::function<void()> exec_fn,
                        std::function<void()> undo_fn, const char* desc) {
            cs.push(std::move(exec_fn), std::move(undo_fn), desc);
        }, nb::arg("execute_fn"), nb::arg("undo_fn"), nb::arg("desc") = "command")
        .def("undo", &viewport::CommandStack::undo)
        .def("redo", &viewport::CommandStack::redo)
        .def("can_undo", &viewport::CommandStack::can_undo)
        .def("can_redo", &viewport::CommandStack::can_redo)
        .def("undo_count", &viewport::CommandStack::undo_count)
        .def("redo_count", &viewport::CommandStack::redo_count)
        .def("next_undo_description", &viewport::CommandStack::next_undo_description,
             nb::rv_policy::reference)
        .def("next_redo_description", &viewport::CommandStack::next_redo_description,
             nb::rv_policy::reference)
        .def("clear", &viewport::CommandStack::clear)
        .def("max_size", &viewport::CommandStack::max_size)
        .def("set_max_size", &viewport::CommandStack::set_max_size);
}
