// tests/unit/scene/test_g2_u3_m6_fillet_display.cpp
//
// G2-u3 m6-CORRETTIVA, MORSO A — IL FILLET VISO: the click-test's
// defect 1 (2026-11-06, the register's first "non passato"): the
// fillet applied did NOT show in the viewport. The m5 declarations
// were honest — scene.hpp's own comment said "no display mesh
// (update_meshes() skips it by kind)" — the defect was DECLARED, now
// it is registered AND corrected with this bite.
//
// THE CORRECTION'S CONTRACT (the morso's own words): the filleted
// body produces its display mesh and the viewport shows it AFTER
// Apply — the mesh path is the exact bodies' OWN road
// (brep::tessellate_body — the same tessellator the bridge walks,
// core/bridge/src/brep_to_sdf.cpp), NEVER a special render of the
// fillet.
//
// THE PINS:
//   * the road: update_meshes' fillet cache is bit-identical to
//     brep::tessellate_body over the same body (the deterministic
//     same-call answer, ADR-0014) — one definition, the scene rides
//     it, it never re-implements it;
//   * the counterproof: the displayed mesh is NOT the flat operand's
//     — the flank cylinder ((x-0.5)^2+(y-0.5)^2 = 0.25 at the (+X,+Y)
//     corner, r = 0.5) EXISTS in the fillet's vertices while the
//     operand's MC mesh has no vertex off the box planes;
//   * the radius edit re-tessellates (the counts may move with r, the
//     road never does);
//   * the refusal keeps the last good body — and its mesh keeps
//     rendering (the visible state);
//   * the additivity: the operand's own mesh is unmoved by the
//     fillet's arrival.
//
// The guide case is the m2/m6 one: the box [-1,1]^3, the edge
// box#1:vertical2, r = 0.5 — the numbers quoted from the register.
#include "CAD_0/brep/tessellate.hpp"
#include "CAD_0/sdf/primitives.hpp"
#include "CAD_0/scene/scene.hpp"
#include "CAD_0/math/vec.hpp"

#include <cmath>
#include <cstdint>
#include <string>

#include <doctest/doctest.h>

using namespace CAD_0;

namespace {

// The flank test: the (+X,+Y) corner's rolling surface is the cylinder
// of radius 0.5 centred on the line x = y = 0.5 (the box kept at
// [0.5,1]^2 in that corner's quadrant — the m2 derivation's own
// window). A mesh vertex ON that cylinder, strictly between the two
// trimmed planes (x < 1 and y < 1), is a point only the FILLETED body
// carries: the flat box's surface lives on the planes |x|=1, |y|=1,
// |z|=1 alone.
constexpr float kCorner = 0.5f;   // the cylinder's centre in x and y
constexpr float kRadius = 0.5f;
constexpr float kTol = 1e-3f;     // float32 store + interpolation

bool on_flank_cylinder(const math::Vec3f& v) {
    const float dx = v.x - kCorner;
    const float dy = v.y - kCorner;
    const float dist = std::sqrt(dx * dx + dy * dy);
    return std::abs(dist - kRadius) < kTol &&
           v.x < 1.0f - kTol && v.y < 1.0f - kTol &&
           std::abs(v.z) < 0.9f;
}

bool on_box_planes(const math::Vec3f& v) {
    // Every vertex of the flat box's isosurface sits on a face plane
    // (within the float32 tolerance of the interpolation).
    return std::abs(std::abs(v.x) - 1.0f) < kTol ||
           std::abs(std::abs(v.y) - 1.0f) < kTol ||
           std::abs(std::abs(v.z) - 1.0f) < kTol;
}

} // namespace

TEST_SUITE("scene fillet display mesh (G2-u3 m6-correttiva)") {

TEST_CASE("the fillet node produces its display mesh on the exact "
          "bodies' road") {
    scene::Scene s;
    const auto box_id =
        s.add_node("Box", sdf::make_box(math::Vec3f{1.0f, 1.0f, 1.0f}));
    const auto fid = s.add_fillet_node(
        "", brep::FilletRecipe{"box#1:vertical2", 0.5});

    const scene::SceneNode* f = s.get_node(fid);
    REQUIRE(f != nullptr);
    REQUIRE(f->kind == scene::NodeKind::Fillet);
    REQUIRE(f->fillet_body != nullptr);

    // Nothing tessellated yet (the cache is dirty from creation).
    REQUIRE(f->cached_mesh.positions.empty());
    CHECK(f->mesh_dirty);

    s.update_meshes(48);

    // THE ROAD: bit-identical to brep::tessellate_body over the same
    // body at the display resolution — the same call, the same answer
    // (deterministic, ADR-0014); the scene never re-implements it.
    brep::TessellationOptions opts;
    opts.resolution = 48;
    const sdf::TriangleMesh ref = brep::tessellate_body(
        *f->fillet_body, opts);
    CHECK(f->cached_mesh.positions == ref.positions);
    CHECK(f->cached_mesh.indices == ref.indices);
    CHECK(f->cached_mesh.positions.size() > 0);
    CHECK(f->cached_mesh.indices.size() > 0);
    CHECK(f->cached_mesh.normals.size() ==
          f->cached_mesh.positions.size());
    CHECK_FALSE(f->mesh_dirty);

    // The honest accessor dispatches the fillet's cache like any
    // body's (display_mesh: the Mesh kind's payload, else the cache).
    CHECK(&f->display_mesh() == &f->cached_mesh);
    (void)box_id;
}

TEST_CASE("the counterproof: the flank shows, the flat operand's "
          "planes do not answer") {
    scene::Scene s;
    s.add_node("Box", sdf::make_box(math::Vec3f{1.0f, 1.0f, 1.0f}));
    const auto fid = s.add_fillet_node(
        "", brep::FilletRecipe{"box#1:vertical2", 0.5});
    s.update_meshes(48);

    const scene::SceneNode* f = s.get_node(fid);
    const scene::SceneNode* b = s.get_node(1);
    REQUIRE(f != nullptr);
    REQUIRE(b != nullptr);
    REQUIRE(b->kind == scene::NodeKind::Sdf);

    // THE FLANK EXISTS IN SCENE: the fillet's mesh carries vertices on
    // the corner cylinder, strictly between the trimmed planes — the
    // rounded edge made visible.
    bool flank_found = false;
    for (const auto& v : f->cached_mesh.positions)
        if (on_flank_cylinder(v)) { flank_found = true; break; }
    CHECK(flank_found);

    // THE FLAT BOX IS NOT THE ANSWER: every MC vertex of the uncut
    // operand sits on the box planes — none can be a flank point.
    for (const auto& v : b->cached_mesh.positions)
        if (on_flank_cylinder(v)) { CHECK(false); break; }
    for (const auto& v : b->cached_mesh.positions)
        CHECK(on_box_planes(v));

    // And the two meshes are not each other (the counts differ — the
    // operand's MC grid against the exact body's per-face grids).
    CHECK(f->cached_mesh.positions.size() !=
          b->cached_mesh.positions.size());
}

TEST_CASE("the radius edit re-tessellates: the road holds for the "
          "new body") {
    scene::Scene s;
    s.add_node("Box", sdf::make_box(math::Vec3f{1.0f, 1.0f, 1.0f}));
    const auto fid = s.add_fillet_node(
        "", brep::FilletRecipe{"box#1:vertical2", 0.5});
    s.update_meshes(48);

    // The class-1 silent edit (the m5 contract): ONE scene mutation,
    // the re-run answering reference_resolved.
    const std::string answer = s.set_fillet_radius(fid, 0.25);
    REQUIRE(answer == "reference_resolved");
    CHECK(s.get_node(fid)->mesh_dirty);   // marked: the mesh follows

    s.update_meshes(48);
    const scene::SceneNode* f = s.get_node(fid);
    REQUIRE(f->fillet_body != nullptr);
    brep::TessellationOptions opts;
    opts.resolution = 48;
    const sdf::TriangleMesh ref = brep::tessellate_body(
        *f->fillet_body, opts);
    CHECK(f->cached_mesh.positions == ref.positions);
    CHECK(f->cached_mesh.indices == ref.indices);

    // The flank is still there — at the new radius the cylinder is
    // centred on the new corner (1-r, 1-r) with radius r.
    bool flank_found = false;
    for (const auto& v : f->cached_mesh.positions) {
        const float dx = v.x - 0.75f;
        const float dy = v.y - 0.75f;
        const float dist = std::sqrt(dx * dx + dy * dy);
        if (std::abs(dist - 0.25f) < kTol && v.x < 1.0f - kTol &&
            v.y < 1.0f - kTol && std::abs(v.z) < 0.9f) {
            flank_found = true;
            break;
        }
    }
    CHECK(flank_found);
}

TEST_CASE("the refusal keeps the last good body — and its mesh keeps "
          "rendering") {
    scene::Scene s;
    s.add_node("Box", sdf::make_box(math::Vec3f{1.0f, 1.0f, 1.0f}));
    const auto fid = s.add_fillet_node(
        "", brep::FilletRecipe{"box#1:vertical2", 0.5});
    s.update_meshes(48);

    // The out-of-band radius (the m5 bound, quoted verbatim): 2.5
    // against the typed band's bound 2.000000 - 0.000001 — the re-run
    // REFUSES, the body stays the last good one.
    const std::string answer = s.set_fillet_radius(fid, 2.5);
    REQUIRE(answer != "reference_resolved");
    REQUIRE(answer.find("fillet_radius_exceeds_face") !=
            std::string::npos);

    s.update_meshes(48);
    const scene::SceneNode* f = s.get_node(fid);
    REQUIRE(f->fillet_body != nullptr);
    CHECK(f->fillet_refusal.empty() == false);
    // The KEPT body still shows: its mesh is the last good body's own.
    brep::TessellationOptions opts;
    opts.resolution = 48;
    const sdf::TriangleMesh ref = brep::tessellate_body(
        *f->fillet_body, opts);
    CHECK(f->cached_mesh.positions.size() == ref.positions.size());
    CHECK(f->cached_mesh.positions.size() > 0);
}

TEST_CASE("the additivity: the operands' meshes are unmoved by the "
          "fillet's arrival") {
    scene::Scene s;
    const auto box_id =
        s.add_node("Box", sdf::make_box(math::Vec3f{1.0f, 1.0f, 1.0f}));
    s.update_meshes(48);

    const scene::SceneNode* b = s.get_node(box_id);
    REQUIRE(b != nullptr);
    const std::size_t verts = b->cached_mesh.positions.size();
    const std::size_t tris = b->cached_mesh.indices.size();
    std::vector<math::Vec3f> snapshot = b->cached_mesh.positions;
    REQUIRE(verts > 0);

    s.add_fillet_node("", brep::FilletRecipe{"box#1:vertical2", 0.5});
    s.update_meshes(48);

    const scene::SceneNode* b2 = s.get_node(box_id);
    REQUIRE(b2 != nullptr);
    CHECK(b2->cached_mesh.positions.size() == verts);
    CHECK(b2->cached_mesh.indices.size() == tris);
    CHECK(b2->cached_mesh.positions == snapshot);
}

TEST_CASE("the loaded fillet re-tessellates: the mesh is a cache, "
          "never serialized") {
    // The loader marks the fillet node dirty with the SDF rule (the
    // mesh is derived state): a fresh update_meshes rebuilds it from
    // the re-materialized body — the scene-side twin of the round
    // trip's own re-run (the m4/m5 rows' contract, extended to the
    // display cache by this bite).
    scene::Scene s;
    s.add_node("Box", sdf::make_box(math::Vec3f{1.0f, 1.0f, 1.0f}));
    const auto fid = s.add_fillet_node(
        "", brep::FilletRecipe{"box#1:vertical2", 0.5});
    s.update_meshes(48);
    const std::size_t verts = s.get_node(fid)->cached_mesh.positions.
        size();
    REQUIRE(verts > 0);

    // The scene node's copy (what restore_node installs — the same
    // dirty discipline the loader writes): the cache travels empty and
    // dirty, the body re-materializes, the mesh follows.
    const scene::SceneNode* f = s.get_node(fid);
    CHECK_FALSE(f->mesh_dirty);
    scene::SceneNode copy = *f;
    copy.mesh_dirty = true;   // the loader's own mark for Fillet
    copy.cached_mesh = {};
    scene::Scene s2;
    s2.restore_node(std::move(copy));
    s2.update_meshes(48);
    const scene::SceneNode* f2 = s2.get_node(fid);
    REQUIRE(f2 != nullptr);
    CHECK(f2->cached_mesh.positions.size() == verts);
}

} // TEST_SUITE
