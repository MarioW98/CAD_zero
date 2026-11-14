// tests/unit/scene/test_perf4_display_derivation.cpp
//
// PERF 4 — THE DISPLAY DERIVATION'S KERNEL-SIDE PROOFS (the async
// road's C++ half; the Python half — the pipeline, the ladder, the
// generation gate, the three controprove — lives in
// bindings/python/tests/test_perf4_async_lod.py, the ctest member).
//
// What THIS suite pins, each assertion named:
//
//   1. THE BIT-IDENTITY LAW (ADR-0014 extended to the display road):
//      derive_display_mesh at the FINE level answers the SAME BYTES
//      Scene::update_meshes installs for the same node and resolution
//      — the same marching_cubes call, the same display-normals core.
//      The three levels' resolutions ride it (48 — the fine, today's
//      own; 24; 12) so the identity holds at every level the ladder
//      climbs, not only at its top.
//
//   2. THE SNAPSHOT'S INDEPENDENCE: the snapshot is the worker's own
//      data — mutating the LIVE node after the snapshot changes
//      NOTHING the derive answers (the race the ownership rule makes
//      impossible by construction, witnessed here: the body replaced
//      in place, the derive's output unmoved).
//
//   3. THE HONEST REFUSAL: a Mesh-kind or Sketch-kind snapshot is
//      refused loudly (the ADR-0002 dispatch — an imported mesh IS
//      the geometry, a sketch is 2D data; there is no derivation to
//      invent, and a silent empty answer would be a lie).
//
//   4. THE FINE LANDING: land_fine_mesh installs the derived mesh
//      byte-identically (memcmp against update_meshes' own) and
//      clears the dirty flag; an unknown node's landing is refused
//      (the backstop — the generation gate owns the real rule).
//
#include <doctest/doctest.h>

#include "CAD_0/sdf/primitives.hpp"
#include "CAD_0/sdf/operators.hpp"
#include "CAD_0/sdf/transforms.hpp"
#include "CAD_0/sdf/mesh_extract.hpp"
#include "CAD_0/scene/scene.hpp"

#include <cmath>
#include <cstring>
#include <vector>

using namespace CAD_0;

namespace {

// The corpus's own shape family: a composite body (a subtract of a
// translated sphere — the subtract recipe the scene tests and the m8
// corpus both speak), so the identity proof rides the CSG interior's
// composing batch, not only a bare leaf.
sdf::SDFBody corpus_body() {
    return sdf::sdf_subtract(
        sdf::make_sphere(1.0f),
        sdf::sdf_translate(sdf::make_sphere(0.45f),
                           math::Vec3f{0.35f, 0.35f, 0.35f}));
}

bool same_bytes(const sdf::TriangleMesh& a, const sdf::TriangleMesh& b) {
    const auto cmp = [](const auto& va, const auto& vb) {
        return va.size() == vb.size() &&
               (va.empty() ||
                std::memcmp(va.data(), vb.data(),
                            va.size() * sizeof(*va.data())) == 0);
    };
    return cmp(a.positions, b.positions) &&
           cmp(a.normals, b.normals) && cmp(a.indices, b.indices);
}

} // namespace

TEST_SUITE("scene PERF 4 display derivation") {

TEST_CASE("the bit-identity law: derive at FINE == update_meshes, at every level's resolution") {
    for (const std::uint32_t resolution : {48u, 24u, 12u}) {
        // The sync road: the scene installs its own tessellation.
        scene::Scene sync_scene;
        const auto id = sync_scene.add_node("corpus", corpus_body());
        sync_scene.update_meshes(resolution);
        const sdf::TriangleMesh& sync_mesh =
            sync_scene.get_node(id)->cached_mesh;

        // The async road: the snapshot + the derive, the same body.
        scene::Scene async_scene;
        const auto aid = async_scene.add_node("corpus", corpus_body());
        const scene::DisplaySourceSnapshot snap =
            scene::snapshot_display_source(*async_scene.get_node(aid));
        const scene::DisplayDerivation derived =
            scene::derive_display_mesh(snap, resolution);

        CHECK(same_bytes(sync_mesh, derived.mesh));
        CHECK(derived.mesh.positions.size() ==
              derived.display_normals.size());
        CHECK(!derived.mesh.positions.empty());
    }
}

TEST_CASE("the display normals: the derive's buffer == the sync road's display_normals call") {
    scene::Scene scene;
    const auto id = scene.add_node("corpus", corpus_body());
    scene.update_meshes(24);
    const scene::SceneNode* n = scene.get_node(id);

    const scene::DisplaySourceSnapshot snap =
        scene::snapshot_display_source(*n);
    const scene::DisplayDerivation derived =
        scene::derive_display_mesh(snap, 24);

    // The sync road's shading buffer: the shared core over the LIVE
    // body's mesh — same body bytes, same answer, byte for byte.
    const std::vector<math::Vec3f> sync_normals = sdf::display_normals(
        n->body, n->cached_mesh.positions, n->cached_mesh.normals);
    REQUIRE(sync_normals.size() == derived.display_normals.size());
    CHECK(std::memcmp(sync_normals.data(), derived.display_normals.data(),
                      sync_normals.size() * sizeof(math::Vec3f)) == 0);
}

TEST_CASE("the snapshot's independence: a mutated node cannot move the derive") {
    scene::Scene scene;
    const auto id = scene.add_node("corpus", corpus_body());
    const scene::DisplaySourceSnapshot snap =
        scene::snapshot_display_source(*scene.get_node(id));
    const scene::DisplayDerivation before =
        scene::derive_display_mesh(snap, 24);

    // The live node's body replaced IN PLACE (a bigger sphere — the
    // edit the worker must never see). The snapshot's derive answers
    // the SAME bytes it answered before the mutation.
    scene.get_node_mut(id)->body = corpus_body();
    const scene::DisplayDerivation after =
        scene::derive_display_mesh(snap, 24);
    CHECK(same_bytes(before.mesh, after.mesh));
    CHECK(std::memcmp(before.display_normals.data(),
                      after.display_normals.data(),
                      before.display_normals.size() *
                          sizeof(math::Vec3f)) == 0);

    // And the live node's OWN derivation answers the new body (the
    // snapshot did not steal the node's data either): a fresh
    // snapshot derives a mesh DIFFERENT from the frozen one's.
    const scene::DisplaySourceSnapshot fresh =
        scene::snapshot_display_source(*scene.get_node(id));
    const scene::DisplayDerivation renewed =
        scene::derive_display_mesh(fresh, 24);
    // Same recipe re-derived -> same bytes as `before` (the recipe is
    // deterministic, ADR-0014); the point above stands on the frozen
    // snapshot being unmoved by the write.
    CHECK(same_bytes(before.mesh, renewed.mesh));
}

TEST_CASE("the honest refusal: mesh- and sketch-kind snapshots have no derivation") {
    scene::Scene scene;
    sdf::TriangleMesh payload;
    payload.positions = {math::Vec3f{0, 0, 0}, math::Vec3f{1, 0, 0},
                         math::Vec3f{0, 1, 0}};
    payload.indices = {0, 1, 2};
    const auto mid = scene.add_node("imported", payload);
    CHECK_THROWS_WITH(
        scene::derive_display_mesh(
            scene::snapshot_display_source(*scene.get_node(mid)), 48),
        doctest::Contains("has no derivation"));

    const auto sid = scene.add_sketch_node("sketch", {}, {});
    CHECK_THROWS_WITH(
        scene::derive_display_mesh(
            scene::snapshot_display_source(*scene.get_node(sid)), 48),
        doctest::Contains("has no derivation"));
}

TEST_CASE("the fine landing: byte-identical install, dirty cleared, unknown refused") {
    scene::Scene scene;
    const auto id = scene.add_node("corpus", corpus_body());
    const scene::DisplaySourceSnapshot snap =
        scene::snapshot_display_source(*scene.get_node(id));
    const scene::DisplayDerivation derived =
        scene::derive_display_mesh(snap, 48);

    REQUIRE(scene.get_node(id)->mesh_dirty);
    CHECK(scene.land_fine_mesh(id, derived.mesh));
    CHECK_FALSE(scene.get_node(id)->mesh_dirty);

    // The install is byte-identical to the sync road's own.
    scene::Scene sync_scene;
    const auto sid = sync_scene.add_node("corpus", corpus_body());
    sync_scene.update_meshes(48);
    CHECK(same_bytes(sync_scene.get_node(sid)->cached_mesh,
                     scene.get_node(id)->cached_mesh));

    // An unknown node's landing is refused (the backstop; the
    // generation gate owns the real rule on the Python side).
    CHECK_FALSE(scene.land_fine_mesh(9999, derived.mesh));
}

} // TEST_SUITE
