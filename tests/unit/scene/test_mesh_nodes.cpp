// tests/unit/scene/test_mesh_nodes.cpp
//
// Mesh-kind scene nodes (FASE 2 of the STL/OBJ import feature):
//   * kind discriminant + honest dispatch (ADR-0002)
//   * update_meshes() never touches Mesh nodes — their "cached mesh"
//     IS the geometry (display_mesh())
//   * bounds() covers mesh positions
//   * parametric editing is a no-op on Mesh nodes
//   * notifications and removal behave like any node
//
#include <doctest/doctest.h>

#include "CAD_0/sdf/mesh_extract.hpp"
#include "CAD_0/sdf/primitives.hpp"
#include "CAD_0/scene/scene.hpp"

#include <cmath>
#include <vector>

using namespace CAD_0;

namespace {

sdf::TriangleMesh test_octahedron() {
    sdf::TriangleMesh m;
    m.positions = {
        math::Vec3f{ 1.0f,  0.0f,  0.0f}, math::Vec3f{-1.0f,  0.0f,  0.0f},
        math::Vec3f{ 0.0f,  1.0f,  0.0f}, math::Vec3f{ 0.0f, -1.0f,  0.0f},
        math::Vec3f{ 0.0f,  0.0f,  1.0f}, math::Vec3f{ 0.0f,  0.0f, -1.0f},
    };
    m.indices = {
        0, 2, 4,  2, 1, 4,  1, 3, 4,  3, 0, 4,
        0, 3, 5,  2, 0, 5,  1, 2, 5,  3, 1, 5,
    };
    return m;
}

} // namespace

TEST_SUITE("scene mesh nodes") {

TEST_CASE("kind discriminant: Sdf default, Mesh after add_node(name, mesh)") {
    scene::Scene s;
    const auto sdf_id = s.add_node("sphere", sdf::make_sphere(1.0f));
    const auto mesh_id = s.add_node("imported", test_octahedron());

    REQUIRE(s.get_node(sdf_id) != nullptr);
    REQUIRE(s.get_node(mesh_id) != nullptr);
    CHECK(s.get_node(sdf_id)->kind == scene::NodeKind::Sdf);
    CHECK(s.get_node(mesh_id)->kind == scene::NodeKind::Mesh);

    // The mesh node has NO SDF-side state: no builder tag, no params.
    CHECK(s.get_node(mesh_id)->builder.empty());
    CHECK(s.get_node(mesh_id)->params.empty());

    // Default name when empty.
    scene::Scene s2;
    const auto id = s2.add_node("", test_octahedron());
    CHECK(s2.get_node(id)->name.rfind("Mesh_", 0) == 0);

    // mesh_dirty is false from the start: nothing to re-tessellate.
    CHECK(s.get_node(mesh_id)->mesh_dirty == false);
}

TEST_CASE("update_meshes skips Mesh nodes — the geometry survives verbatim") {
    scene::Scene s;
    const sdf::TriangleMesh octa = test_octahedron();
    const auto mesh_id = s.add_node("imported", test_octahedron());
    const auto sdf_id = s.add_node("sphere", sdf::make_sphere(1.0f));

    s.update_meshes(48);

    const scene::SceneNode* mn = s.get_node(mesh_id);
    REQUIRE(mn != nullptr);
    // Payload untouched, bit-for-bit.
    CHECK(mn->mesh.positions == octa.positions);
    CHECK(mn->mesh.indices == octa.indices);
    CHECK(mn->mesh.normals.empty()); // never fabricated

    // The honest accessor: geometry for Mesh, cache for Sdf.
    CHECK(&mn->display_mesh() == &mn->mesh);
    const scene::SceneNode* sn = s.get_node(sdf_id);
    REQUIRE(sn != nullptr);
    CHECK(&sn->display_mesh() == &sn->cached_mesh);
    CHECK(sn->cached_mesh.vertex_count() > 0); // SDF was tessellated
    CHECK(sn->mesh_dirty == false);
    CHECK(mn->mesh_dirty == false); // still false — never flagged
}

TEST_CASE("bounds() dispatches by kind: mesh positions included") {
    scene::Scene s;
    const auto mesh_id = s.add_node("imported", test_octahedron());
    s.set_visible(mesh_id, false);
    // Hidden mesh contributes nothing.
    math::Bboxf empty = s.bounds();

    s.set_visible(mesh_id, true);
    const math::Bboxf b = s.bounds();
    CHECK(b.min.x == doctest::Approx(-1.0f));
    CHECK(b.max.x == doctest::Approx(1.0f));
    CHECK(b.min.z == doctest::Approx(-1.0f));
    CHECK(b.max.z == doctest::Approx(1.0f));
    (void)empty;
}

TEST_CASE("parametric editing is a no-op on Mesh nodes") {
    scene::Scene s;
    const auto mesh_id = s.add_node("imported", test_octahedron());

    // set_node_param on a Mesh node: no factory, no params, no crash.
    s.set_node_param(mesh_id, "radius", 42.0f);
    const scene::SceneNode* n = s.get_node(mesh_id);
    REQUIRE(n != nullptr);
    CHECK(n->params.empty());
    CHECK(n->kind == scene::NodeKind::Mesh);
    CHECK(n->mesh.positions.size() == 6); // geometry untouched

    // rebuild_body: nothing to rebuild (also on a mutable handle).
    if (auto* mut = s.get_node_mut(mesh_id)) mut->rebuild_body();
    CHECK(s.get_node(mesh_id)->mesh.positions.size() == 6);
}

TEST_CASE("notifications: Mesh nodes fire NodeAdded like any other") {
    scene::Scene s;
    std::vector<std::pair<scene::SceneChange, std::uint32_t>> events;
    s.set_change_callback([&](scene::SceneChange c, std::uint32_t id) {
        events.emplace_back(c, id);
    });

    const auto id = s.add_node("imported", test_octahedron());
    REQUIRE(events.size() == 1);
    CHECK(events[0].first == scene::SceneChange::NodeAdded);
    CHECK(events[0].second == id);

    CHECK(s.remove_node(id));
    REQUIRE(events.size() == 2);
    CHECK(events[1].first == scene::SceneChange::NodeRemoved);

    // restore_node (native-format load path) keeps the kind + payload.
    scene::SceneNode restored;
    restored.id = 77;
    restored.name = "restored";
    restored.kind = scene::NodeKind::Mesh;
    restored.mesh = test_octahedron();
    restored.mesh_dirty = false;
    s.restore_node(std::move(restored));
    REQUIRE(s.get_node(77) != nullptr);
    CHECK(s.get_node(77)->kind == scene::NodeKind::Mesh);
    CHECK(s.get_node(77)->mesh.positions.size() == 6);
    CHECK(s.next_id() == 78);
}

TEST_CASE("mixed scene: Mesh and Sdf nodes coexist without interference") {
    scene::Scene s;
    const auto sphere_id = s.add_node("sphere", sdf::make_sphere(1.5f));
    const auto mesh_id = s.add_node("octa", test_octahedron());
    const auto box_id = s.add_node("box", sdf::make_box(math::Vec3f{0.5f, 0.5f, 0.5f}));

    s.update_meshes(16);

    CHECK(s.size() == 3);
    CHECK(s.get_node(sphere_id)->cached_mesh.triangle_count() > 0);
    CHECK(s.get_node(box_id)->cached_mesh.triangle_count() > 0);
    CHECK(s.get_node(mesh_id)->mesh.triangle_count() == 8);
    // SDF evaluation untouched by the presence of mesh nodes.
    CHECK(s.get_node(sphere_id)->body.value(math::Vec3f{0.0f, 0.0f, 0.0f}) ==
          doctest::Approx(-1.5f));
}

} // TEST_SUITE
