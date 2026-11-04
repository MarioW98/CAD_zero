// tests/unit/io/test_native.cpp
//
// Native project format (.CAD_0, ADR-0007 + ADR-0007a + ADR-0027 D4
// + ADR-0030 D4 + ADR-0040 D3.1/D6) — unit tests.
//
// Coverage per the feature plan:
//   * round-trip: SDF evaluation is BIT-IDENTICAL after save/load
//   * golden v0.1.0 (frozen legacy fixture): loads through the
//     migrator CHAIN (0.1.0 → 0.2.0 → 0.3.0 → 0.4.0 → 0.5.0); SDF
//     values bit-identical
//   * golden v0.2.0: loads through the 0.2.0 → 0.3.0 migrator; the
//     migrated re-serialization is byte-identical to a fresh save of
//     the current build
//   * golden v0.3.0: the pierced-box document with an operation recipe;
//     migrates with an identical extraction report (χ = 0 / genus 1,
//     ADR-0027's MVP shape)
//   * golden v0.4.0 (ADR-0030 D4): FROZEN HISTORY since the v0.5 bump
//     — the VERSION DOOR's own fixture (G4 m4): a 0.4-declaring file
//     is refused WITH THE REMEDY ("the format moved to v0.5: re-save
//     from the sketch")
//   * golden v0.5.0 (ADR-0040 D3.1, G4 m4): the parametric hole WITH
//     its radius dimension — the constraints ride the sketch payload
//     in NAMED FIELDS beside the chains; extraction χ = 0 / genus 1
//     (the load never solves — the pins are the authored geometry's);
//     byte-identical round-trip, constraints and all
//   * v0.5 sketch payload: parameter-class corruption is a loud load
//     error; an open chain loads as the legal editing state with its
//     extrude in the visible PENDING state; a missing sketch is the
//     visible DANGLING state; operand-kind mismatches, self-references
//     and pre-v0.4 anachronisms are explicit load errors; stale cached
//     extrude bodies are caught by the pass-2 rebuild
//   * container framing: magic/legacy detection, length validation,
//     truncation errors
//   * "raw" blob codec: bit-exact round-trips (incl. -0.0/denormals)
//   * ID round-trip: node ids and next_id survive a cycle exactly
//   * operation recipes (ADR-0027 D4): two-pass load (rebuild / dangling
//     / cycle / self-reference are all explicit), byte-stable round-trip
//   * explicit errors: corrupted JSON, unknown ops, missing fields,
//     version mismatches without a migrator — all throw with messages,
//     and the scene is left untouched (transactional load)
//   * atomic save: no .tmp residue, original file preserved on failure
//
#include <doctest/doctest.h>

#include "CAD_0/io/json.hpp"
#include "CAD_0/io/native.hpp"
#include "CAD_0/io/native_blob.hpp"
#include "CAD_0/bridge/sdf_to_brep.hpp"
#include "CAD_0/sdf/operators.hpp"
#include "CAD_0/sdf/primitives.hpp"
#include "CAD_0/sdf/transforms.hpp"

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace native = CAD_0::io::native;
namespace json = CAD_0::io::json;
using namespace CAD_0;

// ---- helpers ----------------------------------------------------------------

namespace {

// A canonical scene exercising every serialized feature: two nodes with
// ids/params/builder tags, a composite SDF tree (union + subtract +
// translate + smooth_min), non-trivial transform + camera.
scene::Scene make_canonical_scene() {
    scene::Scene s;
    {
        sdf::SDFBody sphere = sdf::make_sphere(1.5f);
        scene::SceneNode n;
        n.id = 7;
        n.name = "Primary sphere";
        n.builder = "sphere";
        n.params = {{"radius", 1.5f}};
        n.transform.translation = math::Vec3f{0.25f, -1.0f, 0.5f};
        n.transform.scale = 1.0f;
        n.body = sphere;
        s.restore_node(std::move(n));
    }
    {
        // union(subtract(box, cylinder), translate(torus, [2,0,0])) smoothed
        // with a sphere — covers every operator + transform op.
        sdf::SDFBody box = sdf::make_box(math::Vec3f{1.0f, 1.0f, 1.0f});
        sdf::SDFBody cyl = sdf::make_cylinder(0.5f, 2.0f);
        sdf::SDFBody cut = sdf::sdf_subtract(std::move(box), std::move(cyl));

        sdf::SDFBody torus = sdf::make_torus(1.0f, 0.3f);
        sdf::SDFBody moved = sdf::sdf_translate(std::move(torus), math::Vec3f{2.0f, 0.0f, 0.0f});

        sdf::SDFBody joined = sdf::sdf_union(std::move(cut), std::move(moved));

        sdf::SDFBody sphere2 = sdf::make_sphere(0.4f);
        sdf::SDFBody blended =
            sdf::sdf_smooth_union(std::move(joined), std::move(sphere2), 0.15f);

        sdf::SDFBody twisted = sdf::sdf_twist(std::move(blended), 0.05f);
        sdf::SDFBody scaled = sdf::sdf_scale(std::move(twisted), 1.25f);

        scene::SceneNode n2;
        n2.id = 9;
        n2.name = "Composite";
        n2.visible = false; // exercised too — must round-trip
        n2.body = scaled;
        s.restore_node(std::move(n2));
    }
    s.set_next_id(10);
    return s;
}

native::NativeCamera make_canonical_camera() {
    native::NativeCamera cam;
    cam.target = math::Vec3f{0.5f, 0.25f, -0.75f};
    cam.distance = 8.25f;
    cam.yaw = 0.625f;
    cam.pitch = -0.125f;
    cam.fov_deg = 55.0f;
    return cam;
}

// Sample a body on a deterministic grid; returns the values.
std::vector<float> sample_grid(const sdf::SDFBody& body, int n) {
    std::vector<float> out;
    out.reserve(static_cast<std::size_t>(n) * n * n);
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            for (int k = 0; k < n; ++k) {
                const math::Vec3f p{-1.5f + 3.0f * static_cast<float>(i) /
                                        static_cast<float>(n - 1),
                                    -1.5f + 3.0f * static_cast<float>(j) /
                                        static_cast<float>(n - 1),
                                    -1.5f + 3.0f * static_cast<float>(k) /
                                        static_cast<float>(n - 1)};
                out.push_back(body.value(p));
            }
    return out;
}

std::filesystem::path golden_path() {
    // tests are run from the build dir; the golden file lives in the source
    // tree at tests/golden/.
    return std::filesystem::path(CAD_0_SOURCE_DIR) / "tests" / "golden" /
           "native_v0_1_0.CAD_0";
}

std::filesystem::path golden_v2_path() {
    // The v0.2.0 golden: ADR-0007a container with one embedded mesh.
    return std::filesystem::path(CAD_0_SOURCE_DIR) / "tests" / "golden" /
           "native_v0_2_0.CAD_0";
}

std::filesystem::path golden_v3_path() {
    // The v0.3.0 golden (ADR-0027 D4): the pierced-box document — a
    // parametric box, a parametric cylinder, and a subtract operation
    // node carrying its recipe.
    return std::filesystem::path(CAD_0_SOURCE_DIR) / "tests" / "golden" /
           "native_v0_3_0.CAD_0";
}

std::filesystem::path golden_v4_path() {
    // The v0.4.0 golden (ADR-0030 D4): the parametric-hole document —
    // a sketch, an extrude consuming it, a subtract consuming them
    // both. The MVP use case, frozen: "draw a circle, extrude it,
    // subtract it from a box". FROZEN HISTORY since the v0.5 bump
    // (G4 m4): the bytes a v0.4.0 writer produced, kept verbatim as
    // the version door's own fixture (the gate test below refuses it
    // WITH THE REMEDY — the file declares 0.4.0, this build reads
    // 0.5.0, and the v0.4 writer never wrote a constraint).
    return std::filesystem::path(CAD_0_SOURCE_DIR) / "tests" / "golden" /
           "native_v0_4_0.CAD_0";
}

std::filesystem::path golden_v5_path() {
    // The v0.5.0 golden (ADR-0040 D3.1, G4 m4): the parametric hole
    // WITH its constraint — the radius dimension rides the sketch
    // payload's NAMED FIELDS beside the chains. Regenerable via
    // CAD_0_GENERATE_GOLDEN=1 after an INTENTIONAL format change (the
    // companion declaration required per ADR-0007 — the reason rides
    // the landing commit's body: "v0.5: the constraints ride named
    // fields").
    return std::filesystem::path(CAD_0_SOURCE_DIR) / "tests" / "golden" /
           "native_v0_5_0.CAD_0";
}

// The pierced-box scene of the v0.3.0 golden (ADR-0027's MVP shape):
// box (id 1, builder, extent (1,1,1)), cylinder (id 2, builder, r=0.25
// h=3 — the through-hole tool along Y), subtract operation (id 3,
// target 1, tool 2, zero offset).
scene::Scene make_pierced_scene() {
    scene::Scene s;
    {
        scene::SceneNode n;
        n.id = 1;
        n.name = "Box";
        n.builder = "box";
        n.params = {{"extent_x", 1.0f}, {"extent_y", 1.0f},
                    {"extent_z", 1.0f}};
        n.body = sdf::make_box(math::Vec3f{1.0f, 1.0f, 1.0f});
        s.restore_node(std::move(n));
    }
    {
        scene::SceneNode n;
        n.id = 2;
        n.name = "Cylinder";
        n.builder = "cylinder";
        n.params = {{"radius", 0.25f}, {"height", 3.0f}};
        n.body = sdf::make_cylinder(0.25f, 3.0f);
        s.restore_node(std::move(n));
    }
    {
        scene::OperationRecipe r;
        r.type = scene::OperationType::Subtract;
        r.target = 1;
        r.tool = 2;
        const std::uint32_t op = s.add_operation_node("Subtract", r);
        REQUIRE(op == 3);
        // The operands are hidden in-app (the operation is the display)
        // — and the golden freezes exactly that state.
        s.set_visible(1, false);
        s.set_visible(2, false);
    }
    s.set_next_id(4);
    return s;
}

// The parametric-hole scene of the v0.4.0/v0.5.0 goldens (ADR-0030's
// use case, ADR-0040 D3.1's guide case): draw a circle, extrude it,
// subtract it from a box — the through-hole is PARAMETRIC now (the
// sketch is its provenance, not a builder cylinder). Geometry: the
// circle r=0.25 at the origin, extruded 3 units +Z (the prism spans
// z in [0, 3]), consumed by the subtract with offset_z = -1.5 — the
// tool spans z in [-1.5, 1.5], the same through-hole the v0.3 golden
// cut with a primitive. Since v0.5 (G4 m4) the sketch carries the
// RADIUS DIMENSION — the guide case's own truth: the circle's radius
// IS the recipe's parameter, and the constraint list rides the
// payload beside the chains (the bill at rest: 3 − 1 = 2 free,// reported, never hidden; the load NEVER solves — D2.3).
scene::Scene make_parametric_hole_scene() {
    scene::Scene s;
    {
        std::vector<sdf::SketchChain> chains;
        chains.push_back({{sdf::sketch_circle(
            math::Vec2f{0.0f, 0.0f}, 0.25f, false)}});
        std::vector<sdf::SketchConstraint> constraints;
        constraints.push_back(sdf::constraint_radius(
            sdf::SketchEntityRef{0, 0}, 0.25));
        const std::uint32_t id = s.add_sketch_node(
            "Hole_Sketch", std::move(chains), std::move(constraints));
        REQUIRE(id == 1);
    }
    {
        scene::OperationRecipe r;
        r.type = scene::OperationType::Extrude;
        r.sketch = 1;
        r.distance = 3.0f;
        r.direction = 1;
        const std::uint32_t id = s.add_operation_node("Extrude", r);
        REQUIRE(id == 2);
    }
    {
        scene::SceneNode n;
        n.id = 3;
        n.name = "Box";
        n.builder = "box";
        n.params = {{"extent_x", 1.0f}, {"extent_y", 1.0f},
                    {"extent_z", 1.0f}};
        n.body = sdf::make_box(math::Vec3f{1.0f, 1.0f, 1.0f});
        s.restore_node(std::move(n));
    }
    {
        scene::OperationRecipe r;
        r.type = scene::OperationType::Subtract;
        r.target = 3;
        r.tool = 2;
        r.offset_z = -1.5f;
        const std::uint32_t id = s.add_operation_node("Subtract", r);
        REQUIRE(id == 4);
    }
    // The operands are hidden in-app (the operation is the display) —
    // and the golden freezes exactly that state (the v0.3 golden's
    // own convention).
    s.set_visible(1, false);
    s.set_visible(2, false);
    s.set_visible(3, false);
    s.set_next_id(5);
    return s;
}

// The embedded mesh of the v0.2.0 golden: a regular octahedron — 6
// vertices, 8 triangles, deterministic to the last bit. Normals are the
// positions themselves (every vertex is a unit vector).
sdf::TriangleMesh make_golden_mesh() {
    sdf::TriangleMesh m;
    m.positions = {
        math::Vec3f{ 1.0f,  0.0f,  0.0f}, math::Vec3f{-1.0f,  0.0f,  0.0f},
        math::Vec3f{ 0.0f,  1.0f,  0.0f}, math::Vec3f{ 0.0f, -1.0f,  0.0f},
        math::Vec3f{ 0.0f,  0.0f,  1.0f}, math::Vec3f{ 0.0f,  0.0f, -1.0f},
    };
    m.normals = m.positions;
    m.indices = {
        0, 2, 4,  2, 1, 4,  1, 3, 4,  3, 0, 4,  // +Z hemisphere
        0, 3, 5,  2, 0, 5,  1, 2, 5,  3, 1, 5,  // -Z hemisphere
    };
    return m;
}

std::vector<native::DocumentMesh> make_golden_meshes() {
    std::vector<native::DocumentMesh> out;
    out.push_back(native::DocumentMesh{"mesh_1", make_golden_mesh()});
    return out;
}

std::string read_file(const std::filesystem::path& p) {
    std::ifstream in(p, std::ios::binary);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

} // namespace

// ---- tests ------------------------------------------------------------------

TEST_SUITE("io native format") {

TEST_CASE("version constant + migrator chain (ADR-0007/0007a/0027/0030/0040)") {
    CHECK(native::kNativeVersion.major == 0);
    CHECK(native::kNativeVersion.minor == 5);
    CHECK(native::kNativeVersion.patch == 0);
    // The registered chain: 0.1.0 -> 0.2.0 (ADR-0007a, no-op),
    // 0.2.0 -> 0.3.0 (ADR-0027 D4, content no-op), 0.3.0 -> 0.4.0
    // (ADR-0030 D4, content no-op) and 0.4.0 -> 0.5.0 (ADR-0040 D6,
    // content no-op — the step the pre-sketch goldens walk THROUGH;
    // a file DECLARING 0.4.x never reaches it: the version door
    // refuses it with the remedy, see the gate test below).
    CHECK(native::migrator_count() == 4);

    const auto v = native::parse_version("0.5.0");
    CHECK(v == native::kNativeVersion);

    CHECK_THROWS_WITH_AS(native::parse_version("0.1"),
        doctest::Contains("malformed version"), std::runtime_error);
    CHECK_THROWS_WITH_AS(native::parse_version("1.2.3.4"),
        doctest::Contains("malformed version"), std::runtime_error);
    CHECK_THROWS_WITH_AS(native::parse_version("abc"),
        doctest::Contains("malformed version"), std::runtime_error);
}

TEST_CASE("round-trip: SDF evaluation is bit-identical") {
    scene::Scene s = make_canonical_scene();
    const native::NativeCamera cam = make_canonical_camera();

    const std::string text = native::serialize_document(s, cam);

    scene::Scene loaded;
    const native::NativeCamera cam2 = [&] {
        // parse + install (transactional stages, exercised separately below)
        auto doc = native::parse_document(text);
        const native::NativeCamera out = doc.camera;
        native::install_document(loaded, std::move(doc));
        return out;
    }();

    REQUIRE(loaded.size() == 2);
    REQUIRE(loaded.node_count() == 2);

    // Bit-exact evaluation on both bodies.
    const scene::SceneNode* a = loaded.node_at(0);
    const scene::SceneNode* b = loaded.node_at(1);
    REQUIRE(a != nullptr);
    REQUIRE(b != nullptr);
    const auto va0 = sample_grid(a->body, 7);
    const auto va1 = sample_grid(s.node_at(0)->body, 7);
    CHECK(va0 == va1);
    const auto vb0 = sample_grid(b->body, 9);
    const auto vb1 = sample_grid(s.node_at(1)->body, 9);
    CHECK(vb0 == vb1);

    // Camera round-trips exactly.
    CHECK(cam2.target.x == cam.target.x);
    CHECK(cam2.target.y == cam.target.y);
    CHECK(cam2.target.z == cam.target.z);
    CHECK(cam2.distance == cam.distance);
    CHECK(cam2.yaw == cam.yaw);
    CHECK(cam2.pitch == cam.pitch);
    CHECK(cam2.fov_deg == cam.fov_deg);
}

TEST_CASE("round-trip: node metadata (id, next_id, name, visible, builder, params, transform)") {
    scene::Scene s = make_canonical_scene();
    const native::NativeCamera cam{};
    const std::string text = native::serialize_document(s, cam);

    scene::Scene loaded;
    native::install_document(loaded, native::parse_document(text));

    // ID round-trip: exact ids, exact next_id.
    CHECK(loaded.node_at(0)->id == 7);
    CHECK(loaded.node_at(1)->id == 9);
    CHECK(loaded.next_id() == 10);

    // next_id clamping: adding after load must not collide.
    const std::uint32_t fresh = loaded.add_node("x", sdf::make_sphere(1.0f));
    CHECK(fresh >= 10);

    // Metadata.
    const scene::SceneNode* n = loaded.node_at(0);
    CHECK(n->name == "Primary sphere");
    CHECK(n->builder == "sphere");
    CHECK(n->params.at("radius") == scene::ParamValue{1.5f});
    CHECK(n->transform.translation.x == 0.25f);
    CHECK(n->transform.translation.y == -1.0f);
    CHECK(n->transform.translation.z == 0.5f);
    CHECK(loaded.node_at(1)->visible == false);
    CHECK(loaded.node_at(1)->builder.empty());
}

TEST_CASE("golden v0.1.0 (frozen legacy fixture): loads through the migrator") {
    // The v0.1.0 golden is FROZEN HISTORY (ADR-0007a): a pure-JSON
    // document written before the binary container existed. It must never
    // be regenerated — its whole purpose is exercising the 0.1.0 -> 0.2.0
    // migrator on every CI run. There is deliberately NO regen path here.
    const std::filesystem::path gp = golden_path();
    const std::string missing = "golden file missing: " + gp.string();
    REQUIRE_MESSAGE(std::filesystem::exists(gp), missing);

    const std::string golden = read_file(gp);

    // Legacy framing sanity: the fixture must still start with '{'.
    // (If this ever fails someone regenerated it as a container —
    // the legacy detection path would silently stop being tested.)
    REQUIRE(!golden.empty());
    CHECK(golden.front() == '{');

    // The migrator path: version 0.1.0 != 0.2.0 -> registry lookup ->
    // registered no-op migrator -> parse proceeds.
    scene::Scene s;
    const native::ParsedDocument doc = native::parse_document(golden);
    native::ParsedDocument doc_copy = doc;
    native::install_document(s, std::move(doc_copy));
    REQUIRE(s.node_count() == 2);

    // SDF evaluation is bit-identical to the canonical scene (the migrator
    // and the container bump did not perturb a single float).
    CHECK(sample_grid(s.node_at(0)->body, 7) ==
          sample_grid(make_canonical_scene().node_at(0)->body, 7));
    CHECK(sample_grid(s.node_at(1)->body, 9) ==
          sample_grid(make_canonical_scene().node_at(1)->body, 9));

    // Metadata survived the upgrade.
    CHECK(s.node_at(0)->id == 7);
    CHECK(s.node_at(1)->id == 9);
    CHECK(s.next_id() == 10);
    CHECK(s.node_at(0)->name == "Primary sphere");
    CHECK(s.node_at(0)->builder == "sphere");
    CHECK(s.node_at(0)->params.at("radius") == scene::ParamValue{1.5f});
    CHECK(s.node_at(1)->visible == false);

    // No meshes in a v0.1.0 document.
    CHECK(doc.meshes.empty());
}

TEST_CASE("golden v0.2.0: loads through the 0.2→0.3 migrator; migrated save == fresh v0.3 save") {
    // Regeneration mode (maintainer-only, never active in CI):
    //   CAD_0_GENERATE_GOLDEN=1 ./CAD_0_tests
    // rewrites the v0.2.0 golden after an INTENTIONAL format change
    // (which requires a companion ADR per ADR-0007/0007a). The v0.1.0
    // golden is frozen and is NEVER rewritten by this path.
    // NOTE (ADR-0027 D4 bump): the golden BYTES still declare 0.2.0 —
    // frozen history. The current writer emits 0.3.0, so the fresh-vs-
    // golden byte comparison of the pre-0.3 era is replaced by its
    // honest successor: the MIGRATED document re-serializes to exactly
    // the bytes a natively-built v0.3 document produces (the migrator
    // loses nothing; the drift detector survives the bump).
    if (const char* env = std::getenv("CAD_0_GENERATE_GOLDEN");
        env && std::string_view(env) == "1") {
        std::filesystem::create_directories(golden_v2_path().parent_path());
        // A v0.2.0 golden must be written by a v0.2.0 writer — this
        // regeneration path is frozen history too: it rewrites the
        // fixture ONLY from the checked-in bytes' own content via the
        // migrator, which cannot be exercised here. Kept as an explicit
        // failure so nobody "regenerates" a v0.2 golden with a v0.3
        // writer by accident.
        MESSAGE("golden v0.2.0 is FROZEN (ADR-0027 D4): regeneration "
                "refused by a v0.3.0 writer");
        return;
    }

    const std::filesystem::path gp = golden_v2_path();
    const std::string missing =
        "golden file missing: " + gp.string();
    REQUIRE_MESSAGE(std::filesystem::exists(gp), missing);
    const std::string golden = read_file(gp);

    // The fixture still declares 0.2.0 (frozen history — if this ever
    // fails, someone rewrote it with a newer writer).
    CHECK(golden.find("\"version\": \"0.2.0\"") != std::string::npos);

    const std::filesystem::path tmp =
        std::filesystem::temp_directory_path() / "cad0_golden_v2_check.CAD_0";

    // (a) The fresh save of the canonical scene + meshes (the
    // reference bytes for (c)).
    native::save_file(tmp, make_canonical_scene(),
                      make_canonical_camera(), make_golden_meshes());
    const std::string fresh_save = read_file(tmp);
    CHECK(fresh_save.find("\"version\": \"0.5.0\"") != std::string::npos);
    std::filesystem::remove(tmp);

    // (b) Load the golden: the 0.2.0 → 0.3.0 migrator runs, framing,
    // nodes and mesh all decode; SDF values are bit-identical to the
    // canonical scene; the mesh is exactly the octahedron we wrote.
    const native::ParsedDocument doc = native::load_file(gp);
    REQUIRE(doc.meshes.size() == 1);
    CHECK(doc.meshes[0].id == "mesh_1");
    const sdf::TriangleMesh& gm = doc.meshes[0].mesh;
    const sdf::TriangleMesh& want = make_golden_mesh();
    REQUIRE(gm.positions.size() == 6);
    REQUIRE(gm.triangle_count() == 8);
    CHECK(gm.positions == want.positions);   // bit-exact vectors
    CHECK(gm.normals == want.normals);
    CHECK(gm.indices == want.indices);

    scene::Scene s;
    native::ParsedDocument doc_copy = doc;
    native::install_document(s, std::move(doc_copy));
    CHECK(sample_grid(s.node_at(0)->body, 7) ==
          sample_grid(make_canonical_scene().node_at(0)->body, 7));

    // (c) Re-serialize the loaded (migrated) document: byte-identical
    // to the fresh save — the migrator loses nothing, and the drift
    // detector (blob offsets, JSON determinism, endian-stable
    // encoding) keeps working across the bump.
    native::save_file(tmp, s, doc.camera, doc.meshes);
    CHECK(read_file(tmp) == fresh_save);
    std::filesystem::remove(tmp);
}

// ---------------------------------------------------------------------------
// v0.3.0 — operation recipes (ADR-0027 D4)
// ---------------------------------------------------------------------------

TEST_CASE("operations: v0.3 round-trip is byte-identical (pierced box)") {
    scene::Scene s = make_pierced_scene();
    const native::NativeCamera cam = make_canonical_camera();

    const std::filesystem::path a =
        std::filesystem::temp_directory_path() / "cad0_ops_rt_a.CAD_0";
    const std::filesystem::path b =
        std::filesystem::temp_directory_path() / "cad0_ops_rt_b.CAD_0";

    native::save_file(a, s, cam);
    const native::ParsedDocument doc = native::load_file(a);
    scene::Scene s2;
    native::install_document(s2, native::ParsedDocument(doc));
    native::save_file(b, s2, doc.camera);
    CHECK(read_file(a) == read_file(b));

    std::filesystem::remove(a);
    std::filesystem::remove(b);

    // The recipe survived the cycle exactly (source of truth).
    const scene::SceneNode* op = s2.get_node(3);
    REQUIRE(op != nullptr);
    REQUIRE(op->operation.has_value());
    CHECK(op->operation->type == scene::OperationType::Subtract);
    CHECK(op->operation->target == 1);
    CHECK(op->operation->tool == 2);
    CHECK(op->operation->offset_x == 0.0f);
    CHECK(op->operation->offset_y == 0.0f);
    CHECK(op->operation->offset_z == 0.0f);
    CHECK_FALSE(op->operand_missing);
    // Ids and next_id round-trip exactly.
    CHECK(s2.get_node(1) != nullptr);
    CHECK(s2.get_node(2) != nullptr);
    CHECK(s2.next_id() == 4);
    // The operand visibility (hidden in-app) round-trips.
    CHECK_FALSE(s2.get_node(1)->visible);
    CHECK_FALSE(s2.get_node(2)->visible);

    // The materialized body is bit-identical to a fresh construction
    // (the loader re-materialized from the recipe — pass 2 — and got
    // exactly the in-app result; a stale cache would not survive).
    CHECK(sample_grid(s2.get_node(3)->body, 7) ==
          sample_grid(s.get_node(3)->body, 7));
}

TEST_CASE("golden v0.3.0: pierced box with recipe - migrates, extraction chi=0 genus=1") {
    // Regeneration mode (maintainer-only, never active in CI):
    //   CAD_0_GENERATE_GOLDEN=1 ./CAD_0_tests
    // rewrites the v0.3.0 golden after an INTENTIONAL format change
    // (companion ADR required per ADR-0007).
    //
    // NOTE (ADR-0030 D4 bump): the golden BYTES still declare 0.3.0 —
    // frozen history. The current writer emits 0.4.0, so the pre-0.4
    // "fresh save == golden bytes" check is replaced by its honest
    // successor (the same one the v0.2.0 golden test adopted at the
    // 0.3 bump): the MIGRATED document re-serializes to exactly the
    // bytes a fresh v0.4.0 save of the same scene produces. The
    // extraction report — the D6 clause — is asserted IDENTICAL below
    // (same T13 pins the v0.3 golden always carried).
    if (const char* env = std::getenv("CAD_0_GENERATE_GOLDEN");
        env && std::string_view(env) == "1") {
        std::filesystem::create_directories(golden_v3_path().parent_path());
        // A v0.3.0 golden must be written by a v0.3.0 writer — frozen
        // history, exactly like the v0.2.0 one: regenerating it with a
        // v0.4.0 writer would falsify the fixture's own version.
        MESSAGE("golden v0.3.0 is FROZEN (ADR-0030 D4): regeneration "
                "refused by a v0.4.0 writer");
        return;
    }

    const std::filesystem::path gp = golden_v3_path();
    const std::string missing = "golden file missing: " + gp.string();
    REQUIRE_MESSAGE(std::filesystem::exists(gp), missing);
    const std::string golden = read_file(gp);
    CHECK(golden.find("\"version\": \"0.3.0\"") != std::string::npos);

    const std::filesystem::path tmp =
        std::filesystem::temp_directory_path() / "cad0_golden_v3_check.CAD_0";

    // (a) The migrated golden re-serializes to exactly the bytes a
    // fresh v0.5.0 save of the same scene produces — the migrator
    // (0.3.0 → 0.4.0 → 0.5.0, content no-op) loses nothing, and the
    // drift detector survives the bump.
    native::save_file(tmp, make_pierced_scene(), make_canonical_camera());
    const std::string fresh_save = read_file(tmp);
    CHECK(fresh_save.find("\"version\": \"0.5.0\"") != std::string::npos);
    std::filesystem::remove(tmp);

    const native::ParsedDocument doc = native::load_file(gp);
    {
        scene::Scene s;
        native::ParsedDocument doc_copy = doc;
        native::install_document(s, std::move(doc_copy));
        native::save_file(tmp, s, doc.camera);
        CHECK(read_file(tmp) == fresh_save);
        std::filesystem::remove(tmp);
    }

    // (b) The golden carries the recipe in the ADR-0027 D4 shape.
    CHECK(golden.find("\"operation\"") != std::string::npos);
    CHECK(golden.find("\"type\": \"subtract\"") != std::string::npos);

    // (c) Load: the operation node is intact and NOT dangling; the
    // extraction report on its body is the ADR-0027 MVP contract —
    // the first genus > 0 document in the repository's history:
    // χ = 0, genus 1, closed-orientable — IDENTICAL across the 0.4
    // bump (the D6 clause: "the v0.3 golden migrates with an identical
    // extraction report").
    REQUIRE(doc.nodes.size() == 3);
    const scene::SceneNode* op = &doc.nodes[2];
    REQUIRE(op->id == 3);
    REQUIRE(op->operation.has_value());
    CHECK(op->operation->target == 1);
    CHECK(op->operation->tool == 2);
    CHECK_FALSE(op->operand_missing);

    bridge::SdfToBrepOptions opts;
    opts.march_resolution = 48;
    const bridge::SdfToBrepReport rep =
        bridge::sdf_to_brep_report(op->body, opts);
    CHECK(rep.fit == "faceted");
    CHECK(rep.chi == 0);
    CHECK(rep.orientable);
    CHECK(rep.genus_derived);
    CHECK(rep.genus == 1);
    CHECK(rep.loops);
    // The T13 pins (the same body, the same grid).
    CHECK(rep.vertex_count == 15104);
    CHECK(rep.edge_count == 45312);
    CHECK(rep.face_count == 30208);
}

// ---------------------------------------------------------------------------
// v0.4.0 — the sketcher's document layer (ADR-0030 D4); v0.5.0 — the
// constraints' named fields (ADR-0040 D3.1, G4 m4)
// ---------------------------------------------------------------------------

TEST_CASE("version door: a v0.4-declaring file is refused WITH THE REMEDY (G4 m4)") {
    // The v0.4.0 golden is FROZEN HISTORY — and since the v0.5 bump it
    // is the version door's own fixture: the bytes a v0.4.0 writer
    // produced, declaring 0.4.0, refused by this 0.5.0 build with the
    // remedy. The gate's reason is the bump's reason: the v0.4 writer
    // never wrote a constraint (they were session state in that era),
    // so a v0.4 sketch's authored constraints are exactly what the
    // file cannot carry — opening it as v0.5 would present the sketch
    // as if nothing were lost. KIND and payload pinned: the exception
    // is std::runtime_error, the message carries the remedy verbatim.
    const std::filesystem::path gp = golden_v4_path();
    const std::string missing = "golden file missing: " + gp.string();
    REQUIRE_MESSAGE(std::filesystem::exists(gp), missing);
    const std::string golden = read_file(gp);

    // The fixture still declares its own era (frozen bytes, verbatim).
    CHECK(golden.find("\"version\": \"0.4.0\"") != std::string::npos);
    // ... and it carries the v0.4 payload shape (the sketch, the
    // recipes) — the document the remedy speaks of.
    CHECK(golden.find("\"kind\": \"sketch\"") != std::string::npos);
    CHECK(golden.find("\"type\": \"extrude\"") != std::string::npos);

    // THE DOOR: load_file refuses, naming both versions, with the
    // remedy the register practices ("the format moved to v0.5:
    // re-save from the sketch").
    CHECK_THROWS_WITH_AS(native::load_file(gp),
        doctest::Contains("the format moved to v0.5: re-save from the "
                          "sketch"), std::runtime_error);
    // The same door on the TEXT entry point (parse_document): a
    // minimal 0.4-declaring document — the gate is the VERSION's own,
    // before any payload is even looked at, and the refusal names
    // both ends. PATCH does not save a 0.4 file: the door is the
    // MINOR's (0.4.7 is the same era, the same lossy writer).
    const std::string decl_v4 =
        "{\"format\": \"CAD_0 native\", \"version\": \"0.4.0\","
        " \"scene\": { \"next_id\": 1, \"nodes\": [] }}";
    CHECK_THROWS_WITH_AS(native::parse_document(decl_v4),
        doctest::Contains("the format moved to v0.5: re-save from the "
                          "sketch"), std::runtime_error);
    CHECK_THROWS_WITH_AS(native::parse_document(decl_v4),
        doctest::Contains("0.4.0"), std::runtime_error);
    const std::string decl_v4_patch =
        "{\"format\": \"CAD_0 native\", \"version\": \"0.4.7\","
        " \"scene\": { \"next_id\": 1, \"nodes\": [] }}";
    CHECK_THROWS_WITH_AS(native::parse_document(decl_v4_patch),
        doctest::Contains("the format moved to v0.5"), std::runtime_error);

    // The remedy is not a silent rewrite: the file on disk is
    // untouched by the refusal (nothing "migrated" behind the user).
    CHECK(read_file(gp) == golden);
}

TEST_CASE("golden v0.5.0: the parametric hole with its radius dimension - named fields, chi=0 genus=1") {
    // Regeneration mode (maintainer-only, never active in CI):
    //   CAD_0_GENERATE_GOLDEN=1 ./CAD_0_tests
    // rewrites the v0.5.0 golden after an INTENTIONAL format change
    // (companion declaration required per ADR-0007; the v0.5 landing's
    // own reason: "v0.5: the constraints ride named fields").
    if (const char* env = std::getenv("CAD_0_GENERATE_GOLDEN");
        env && std::string_view(env) == "1") {
        std::filesystem::create_directories(golden_v5_path().parent_path());
        native::save_file(golden_v5_path(), make_parametric_hole_scene(),
                          make_canonical_camera());
        MESSAGE("golden file (re)written: ", golden_v5_path().string());
        return;
    }

    const std::filesystem::path gp = golden_v5_path();
    const std::string missing = "golden file missing: " + gp.string();
    REQUIRE_MESSAGE(std::filesystem::exists(gp), missing);
    const std::string golden = read_file(gp);
    CHECK(golden.find("\"version\": \"0.5.0\"") != std::string::npos);

    // (a) The fresh save of the parametric-hole scene IS the golden
    // bytes (the writer is the current one — no bump in between).
    const std::filesystem::path tmp =
        std::filesystem::temp_directory_path() / "cad0_golden_v5_check.CAD_0";
    native::save_file(tmp, make_parametric_hole_scene(), make_canonical_camera());
    CHECK(read_file(tmp) == golden);
    std::filesystem::remove(tmp);

    // (b) The golden carries the v0.5 payload: the sketch node, the
    // extrude recipe, the subtract recipe — and the CONSTRAINTS array
    // beside the chains, in NAMED FIELDS (the radius dimension on the
    // hole circle: the guide case's own parameter, D3.1). The fields
    // are pinned as text — the m2's padding lesson's mirror: the file
    // says the FIELD's name, never a struct's byte.
    CHECK(golden.find("\"kind\": \"sketch\"") != std::string::npos);
    CHECK(golden.find("\"type\": \"circle\"") != std::string::npos);
    CHECK(golden.find("\"type\": \"extrude\"") != std::string::npos);
    CHECK(golden.find("\"type\": \"subtract\"") != std::string::npos);
    CHECK(golden.find("\"constraints\"") != std::string::npos);
    CHECK(golden.find("\"kind\": \"radius\"") != std::string::npos);
    CHECK(golden.find("\"value\": 0.25") != std::string::npos);
    CHECK(golden.find("\"chain\"") != std::string::npos);
    CHECK(golden.find("\"entity\"") != std::string::npos);

    // (c) Load: four nodes, every recipe intact, nothing dangling or
    // pending. The sketch's entity data is bit-exact; the CONSTRAINT
    // rides the payload exactly as authored (the load never solves —
    // D2.3: the recipe is the truth, the bill at rest 3 − 1 = 2 free
    // is the NEXT cold act's own answer, never the loader's).
    const native::ParsedDocument doc = native::load_file(gp);
    REQUIRE(doc.nodes.size() == 4);

    const scene::SceneNode* sk = &doc.nodes[0];
    REQUIRE(sk->id == 1);
    REQUIRE(sk->kind == scene::NodeKind::Sketch);
    REQUIRE(sk->sketch.size() == 1);
    REQUIRE(sk->sketch[0].entities.size() == 1);
    REQUIRE(sk->sketch[0].entities[0].kind ==
            sdf::SketchEntity::Kind::Circle);
    CHECK(sk->sketch[0].entities[0].circle.center ==
          math::Vec2f{0.0f, 0.0f});
    CHECK(sk->sketch[0].entities[0].circle.radius == 0.25f);
    CHECK(sk->sketch[0].entities[0].circle.hole == false);
    CHECK(sdf::chain_is_closed(sk->sketch[0]));

    // THE CONSTRAINT, field for field: kind, operand (the canonical
    // index chain 0 → entity 0), value — the authored dimension.
    REQUIRE(sk->sketch_constraints.size() == 1);
    const sdf::SketchConstraint& rc = sk->sketch_constraints[0];
    CHECK(rc.kind == sdf::SketchConstraint::Kind::Radius);
    CHECK(rc.ea.chain == 0);
    CHECK(rc.ea.entity == 0);
    CHECK(rc.value == 0.25);

    const scene::SceneNode* ex = &doc.nodes[1];
    REQUIRE(ex->id == 2);
    REQUIRE(ex->kind == scene::NodeKind::Sdf);
    REQUIRE(ex->operation.has_value());
    CHECK(ex->operation->type == scene::OperationType::Extrude);
    CHECK(ex->operation->sketch == 1);
    CHECK(ex->operation->distance == 3.0f);
    CHECK(ex->operation->direction == 1);
    CHECK_FALSE(ex->operand_missing);
    CHECK_FALSE(ex->operation_pending);
    // The extrude's body IS the prism (re-materialized by pass 2 from
    // the recipe, deterministically — a stale cache would not survive).
    CHECK(std::string(ex->body.root()->describe())
              .rfind("extrude(", 0) == 0);

    const scene::SceneNode* box = &doc.nodes[2];
    REQUIRE(box->id == 3);
    CHECK(box->builder == "box");

    const scene::SceneNode* sub = &doc.nodes[3];
    REQUIRE(sub->id == 4);
    REQUIRE(sub->operation.has_value());
    CHECK(sub->operation->type == scene::OperationType::Subtract);
    CHECK(sub->operation->target == 3);
    CHECK(sub->operation->tool == 2);
    CHECK(sub->operation->offset_z == -1.5f);
    CHECK_FALSE(sub->operand_missing);

    // next_id and the operand visibility round-trip exactly.
    CHECK(doc.next_id == 5);
    CHECK_FALSE(sk->visible);
    CHECK_FALSE(ex->visible);
    CHECK_FALSE(box->visible);

    // (d) The extraction report on the final body: the parametric hole
    // is genus 1 — the SAME topological contract as the v0.3 pierced
    // box, now reached through the sketcher's path (D6's clause). The
    // constraint rides the payload WITHOUT touching the geometry: the
    // pins are EXACTLY the v0.4 golden's own pins (the load never
    // solves — the solved values are the authored values).
    bridge::SdfToBrepOptions opts;
    opts.march_resolution = 48;
    const bridge::SdfToBrepReport rep =
        bridge::sdf_to_brep_report(sub->body, opts);
    CHECK(rep.fit == "faceted");
    CHECK(rep.chi == 0);
    CHECK(rep.orientable);
    CHECK(rep.genus_derived);
    CHECK(rep.genus == 1);
    CHECK(rep.loops);
    // The pins (frozen on the golden, re-run bit-identical — the T13
    // discipline). They are EXACTLY the v0.3 golden's own pins: the
    // extrude-prism field (max of disc and slab) has the same
    // isosurface as the builder cylinder's, so the parametric path
    // and the primitive path extract the SAME manifold — the
    // strongest possible statement that the sketcher changes nothing
    // about the geometry it reproduces.
    CHECK(rep.vertex_count == 15104);
    CHECK(rep.edge_count == 45312);
    CHECK(rep.face_count == 30208);

    // (e) Bit-stability: the same body, re-extracted, is identical
    // (the T13b discipline).
    const bridge::SdfToBrepReport rep2 =
        bridge::sdf_to_brep_report(sub->body, opts);
    CHECK(rep2.vertex_count == rep.vertex_count);
    CHECK(rep2.edge_count == rep.edge_count);
    CHECK(rep2.face_count == rep.face_count);
    CHECK(rep2.chi == rep.chi);
}

TEST_CASE("sketch: v0.5 round-trip is byte-identical (parametric hole, constraints and all)") {
    scene::Scene s = make_parametric_hole_scene();
    const native::NativeCamera cam = make_canonical_camera();

    const std::filesystem::path a =
        std::filesystem::temp_directory_path() / "cad0_sketch_rt_a.CAD_0";
    const std::filesystem::path b =
        std::filesystem::temp_directory_path() / "cad0_sketch_rt_b.CAD_0";

    // N CYCLES (the digest proof): every re-save is byte-identical to
    // the FIRST save — the constraints, the solved fields, the
    // recipes, the whole payload; the cycle feeds itself (the re-saved
    // file is the next load's input), so a drift anywhere would
    // compound and surface. Both compilers run this suite; the
    // cross-compiler byte-stability is the golden's own pin.
    native::save_file(a, s, cam);
    const std::string first_save = read_file(a);
    scene::Scene s2;
    for (int cycle = 0; cycle < 3; ++cycle) {
        const native::ParsedDocument doc = native::load_file(a);
        native::install_document(s2, native::ParsedDocument(doc));
        native::save_file(b, s2, doc.camera);
        CHECK(read_file(b) == first_save);
        std::filesystem::copy_file(
            b, a, std::filesystem::copy_options::overwrite_existing);
    }

    std::filesystem::remove(a);
    std::filesystem::remove(b);

    // The sketch payload survived the cycle exactly (bit-level): the
    // chains, the entity kinds, every parameter and orientation bit.
    const scene::SceneNode* sk2 = s2.get_node(1);
    REQUIRE(sk2 != nullptr);
    REQUIRE(sk2->kind == scene::NodeKind::Sketch);
    REQUIRE(sk2->sketch.size() == 1);
    REQUIRE(sk2->sketch[0].entities.size() == 1);
    CHECK(sk2->sketch[0].entities[0].kind ==
          sdf::SketchEntity::Kind::Circle);
    CHECK(sk2->sketch[0].entities[0].circle.center ==
          math::Vec2f{0.0f, 0.0f});
    CHECK(sk2->sketch[0].entities[0].circle.radius == 0.25f);
    CHECK(sk2->sketch[0].entities[0].circle.hole == false);

    // THE CONSTRAINTS survived the cycle exactly (the v0.5 payload's
    // own member): kind, canonical operand indices, value — field for
    // field, the authored dimension back (D3.1: the round-trip
    // preserves BOTH — the entity's solved fields and the constraint
    // list; the load trusts both, never re-solves).
    REQUIRE(sk2->sketch_constraints.size() == 1);
    CHECK(sk2->sketch_constraints[0].kind ==
          sdf::SketchConstraint::Kind::Radius);
    CHECK(sk2->sketch_constraints[0].ea.chain == 0);
    CHECK(sk2->sketch_constraints[0].ea.entity == 0);
    CHECK(sk2->sketch_constraints[0].value == 0.25);

    // The recipes survived exactly (source of truth).
    const scene::SceneNode* ex2 = s2.get_node(2);
    REQUIRE(ex2->operation.has_value());
    CHECK(ex2->operation->type == scene::OperationType::Extrude);
    CHECK(ex2->operation->sketch == 1);
    CHECK(ex2->operation->distance == 3.0f);
    CHECK(ex2->operation->direction == 1);

    const scene::SceneNode* sub2 = s2.get_node(4);
    REQUIRE(sub2->operation.has_value());
    CHECK(sub2->operation->offset_z == -1.5f);

    // Ids, next_id and visibility round-trip exactly.
    CHECK(s2.next_id() == 5);
    CHECK_FALSE(s2.get_node(1)->visible);
    CHECK_FALSE(s2.get_node(2)->visible);
    CHECK_FALSE(s2.get_node(3)->visible);

    // The materialized bodies are bit-identical to the fresh
    // construction (pass 2 rebuilt from the recipes — deterministically).
    CHECK(sample_grid(s2.get_node(4)->body, 7) ==
          sample_grid(s.get_node(4)->body, 7));
    CHECK(sample_grid(s2.get_node(2)->body, 7) ==
          sample_grid(s.get_node(2)->body, 7));
}

// ---------------------------------------------------------------------------
// G4 m4 — THE PADDING COUNTERPROOF, THE ALL-KINDS ROUND-TRIP, THE
// DOORS, THE RE-SOLVE AFTER THE LOAD (ADR-0040 D2.3/D3.1)
// ---------------------------------------------------------------------------

namespace {

// FNV-1a, the test's own digest: a number over bytes, nothing more —
// the point is the EQUALITY, never the cryptography.
std::string fnv1a_hex(const unsigned char* data, std::size_t n) {
    std::uint64_t h = 1469598103934665603ull;
    for (std::size_t i = 0; i < n; ++i) {
        h ^= data[i];
        h *= 1099511628211ull;
    }
    char buf[17];
    std::snprintf(buf, sizeof(buf), "%016llx",
                  static_cast<unsigned long long>(h));
    return std::string(buf);
}

// The m2's comparator discipline, verbatim in spirit: the VALUES the
// recipe serializes are compared FIELD for FIELD, never as raw bytes —
// the padding is not part of any contract.
bool constraint_value_identical(const sdf::SketchConstraint& a,
                                const sdf::SketchConstraint& b) {
    if (a.kind != b.kind) return false;
    const auto same_f = [](double x, double y) {
        return std::memcmp(&x, &y, sizeof(double)) == 0;
    };
    if (!same_f(a.value, b.value)) return false;
    if (a.case_bit != b.case_bit) return false;
    if (a.a.entity.chain != b.a.entity.chain ||
        a.a.entity.entity != b.a.entity.entity || a.a.end != b.a.end)
        return false;
    if (a.b.entity.chain != b.b.entity.chain ||
        a.b.entity.entity != b.b.entity.entity || a.b.end != b.b.end)
        return false;
    if (a.ea.chain != b.ea.chain || a.ea.entity != b.ea.entity)
        return false;
    if (a.eb.chain != b.eb.chain || a.eb.entity != b.eb.entity)
        return false;
    if (a.pin.entity.chain != b.pin.entity.chain ||
        a.pin.entity.entity != b.pin.entity.entity ||
        a.pin.field != b.pin.field)
        return false;
    return true;
}

bool entity_value_identical(const sdf::SketchEntity& a,
                            const sdf::SketchEntity& b) {
    if (a.kind != b.kind) return false;
    const auto f32_same = [](float x, float y) {
        return std::memcmp(&x, &y, 4) == 0;
    };
    switch (a.kind) {
    case sdf::SketchEntity::Kind::Segment:
        return f32_same(a.segment.a.x, b.segment.a.x) &&
               f32_same(a.segment.a.y, b.segment.a.y) &&
               f32_same(a.segment.b.x, b.segment.b.x) &&
               f32_same(a.segment.b.y, b.segment.b.y) &&
               a.segment.interior_left == b.segment.interior_left;
    case sdf::SketchEntity::Kind::Circle:
        return f32_same(a.circle.center.x, b.circle.center.x) &&
               f32_same(a.circle.center.y, b.circle.center.y) &&
               f32_same(a.circle.radius, b.circle.radius) &&
               a.circle.hole == b.circle.hole;
    case sdf::SketchEntity::Kind::Arc:
        return f32_same(a.arc.center.x, b.arc.center.x) &&
               f32_same(a.arc.center.y, b.arc.center.y) &&
               f32_same(a.arc.radius, b.arc.radius) &&
               f32_same(a.arc.start_angle, b.arc.start_angle) &&
               f32_same(a.arc.sweep, b.arc.sweep) &&
               a.arc.hole == b.arc.hole;
    }
    return false;
}

bool chains_value_identical(const std::vector<sdf::SketchChain>& a,
                            const std::vector<sdf::SketchChain>& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t c = 0; c < a.size(); ++c) {
        if (a[c].entities.size() != b[c].entities.size()) return false;
        for (std::size_t e = 0; e < a[c].entities.size(); ++e)
            if (!entity_value_identical(a[c].entities[e], b[c].entities[e]))
                return false;
    }
    return true;
}

} // namespace

TEST_CASE("G4 m4 — the padding counterproof: the naive raw-struct write dies, the named fields live") {
    // THE HEART OF THE MORSO. The m2's discovery, demonstrated as the
    // discrimination that justifies it: a raw-struct image carries the
    // TAIL PADDING the language leaves unspecified (the m2 observed it
    // across compilers — GCC's stack reuse masked it, CLANG's did
    // not), so two instances with IDENTICAL field content can differ
    // as bytes: two saves of the same sketch, two digests — the silent
    // death of ADR-0014's determinism. The naive writer here is built
    // for the occasion (the "version ingenua": memcpy the whole
    // struct, digest the image) and is PROVEN DEAD; the fields writer
    // (the landed one) is proven stable on the very same poisoned
    // objects.
    using sdf::SketchConstraint;

    // (a) THE HOLES EXIST: the struct's byte count strictly exceeds
    // the sum of its members' own byte counts — the language's own
    // arithmetic, the space where the garbage lives.
    const std::size_t member_bytes =
        sizeof(SketchConstraint::Kind) + sizeof(sdf::SketchPointRef) +
        sizeof(sdf::SketchPointRef) + sizeof(sdf::SketchEntityRef) +
        sizeof(sdf::SketchEntityRef) + sizeof(double) + sizeof(int) +
        sizeof(sdf::SketchParamRef);
    CHECK(sizeof(SketchConstraint) > member_bytes);

    // (b) TWO INSTANCES, SAME CONTENT: the second is a copy of the
    // first through the factory-built object; its TAIL padding is
    // then scribbled differently (the last bytes of the struct are
    // padding — the m2's own observation, simulated deterministically
    // so both compilers demonstrate it, not just the one that
    // happened to expose it).
    SketchConstraint c1 = sdf::constraint_length(
        sdf::SketchEntityRef{0, 2}, 4.0);
    SketchConstraint c2 = c1;
    unsigned char* raw2 = reinterpret_cast<unsigned char*>(&c2);
    for (std::size_t i = sizeof(c2) - 7; i < sizeof(c2); ++i)
        raw2[i] = static_cast<unsigned char>(0xAA ^ i);

    // (c) THE NAIVE DIES: the raw-struct digests DIFFER for the same
    // content — the assertion the naive's own determinism contract
    // would write, and the red it would run: this is WHERE it dies.
    const std::string naive1 = fnv1a_hex(
        reinterpret_cast<const unsigned char*>(&c1), sizeof(c1));
    const std::string naive2 = fnv1a_hex(
        reinterpret_cast<const unsigned char*>(&c2), sizeof(c2));
    CHECK(naive1 != naive2);

    // (d) THE FIELDS LIVE: the same two objects — one clean, one with
    // poisoned padding — serialize through the landed writer to
    // IDENTICAL file bytes. The padding does not exist in the file:
    // every field read and written BY NAME.
    const auto scene_with = [](const SketchConstraint& c) {
        scene::Scene s;
        std::vector<sdf::SketchChain> chains;
        chains.push_back({{sdf::sketch_segment(
            math::Vec2f{0.0f, 0.0f}, math::Vec2f{4.0f, 0.0f}, true),
            sdf::sketch_segment(math::Vec2f{4.0f, 0.0f},
                                math::Vec2f{4.0f, 3.0f}, true),
            sdf::sketch_segment(math::Vec2f{4.0f, 3.0f},
                                math::Vec2f{0.0f, 0.0f}, true)}});
        s.add_sketch_node("S", std::move(chains), {c});
        return s;
    };
    const std::filesystem::path pa =
        std::filesystem::temp_directory_path() / "cad0_pad_fields_a.CAD_0";
    const std::filesystem::path pb =
        std::filesystem::temp_directory_path() / "cad0_pad_fields_b.CAD_0";
    native::save_file(pa, scene_with(c1), make_canonical_camera());
    native::save_file(pb, scene_with(c2), make_canonical_camera());
    CHECK(read_file(pa) == read_file(pb));

    // And the digest over the FILES: one hash for the same content —
    // the contract the naive broke, held by the fields.
    const std::string file1 = read_file(pa);
    const std::string file2 = read_file(pb);
    CHECK(fnv1a_hex(reinterpret_cast<const unsigned char*>(file1.data()),
                    file1.size()) ==
          fnv1a_hex(reinterpret_cast<const unsigned char*>(file2.data()),
                    file2.size()));
    std::filesystem::remove(pa);
    std::filesystem::remove(pb);
}

TEST_CASE("G4 m4 — the all-eight-kinds round-trip: N cycles, byte-identical, the authored list back field for field") {
    // One sketch carrying EVERY constraint kind v1 has — the
    // coincidence, the parallel, the perpendicular, the tangent (with
    // its case bit), the length, the radius, the distance, the pin
    // (with its parameter index) — round-tripped N times. The save
    // bytes never move; the loaded list is the authored list, field
    // for field; the pin that pointed at the circle's cx BEFORE the
    // save points at the same field AFTER the load (the canonical
    // indices are stable BY CONSTRUCTION: chain → entity → field is
    // the serialization's own order).
    scene::Scene s;
    {
        std::vector<sdf::SketchChain> chains;
        // chain 0: three segments (the triangle's shape).
        chains.push_back({{sdf::sketch_segment(
                               math::Vec2f{0.10f, 0.05f},
                               math::Vec2f{4.15f, -0.05f}, true),
                           sdf::sketch_segment(math::Vec2f{4.10f, 0.02f},
                                               math::Vec2f{0.05f, 3.10f},
                                               true),
                           sdf::sketch_segment(math::Vec2f{0.08f, 3.15f},
                                               math::Vec2f{0.12f, -0.02f},
                                               true)}});
        // chain 1: one circle (the tangent's and the radius's target).
        chains.push_back({{sdf::sketch_circle(
            math::Vec2f{-3.0f, 1.0f}, 2.0f, false)}});
        std::vector<sdf::SketchConstraint> authored;
        authored.push_back(sdf::constraint_coincidence(
            sdf::SketchPointRef{sdf::SketchEntityRef{0, 0},
                                sdf::SketchEnd::End},
            sdf::SketchPointRef{sdf::SketchEntityRef{0, 1},
                                sdf::SketchEnd::Start}));
        authored.push_back(sdf::constraint_parallel(
            sdf::SketchEntityRef{0, 0}, sdf::SketchEntityRef{0, 2}));
        authored.push_back(sdf::constraint_perpendicular(
            sdf::SketchEntityRef{0, 1}, sdf::SketchEntityRef{0, 2}));
        authored.push_back(sdf::constraint_tangent(
            sdf::SketchEntityRef{0, 0}, sdf::SketchEntityRef{1, 0}, -1));
        authored.push_back(
            sdf::constraint_length(sdf::SketchEntityRef{0, 0}, 4.0));
        authored.push_back(
            sdf::constraint_radius(sdf::SketchEntityRef{1, 0}, 2.0));
        authored.push_back(sdf::constraint_distance(
            sdf::SketchPointRef{sdf::SketchEntityRef{0, 0},
                                sdf::SketchEnd::Start},
            sdf::SketchPointRef{sdf::SketchEntityRef{0, 2},
                                sdf::SketchEnd::End},
            5.0));
        // THE PIN: the circle's cx (chain 1, entity 0, field 0).
        authored.push_back(sdf::constraint_pin(
            sdf::SketchParamRef{sdf::SketchEntityRef{1, 0}, 0}, 0.5));
        REQUIRE(s.add_sketch_node("AllKinds", std::move(chains),
                                  std::move(authored)) == 1);
    }

    const std::filesystem::path a =
        std::filesystem::temp_directory_path() / "cad0_kinds_rt_a.CAD_0";
    const std::filesystem::path b =
        std::filesystem::temp_directory_path() / "cad0_kinds_rt_b.CAD_0";
    native::save_file(a, s, make_canonical_camera());
    const std::string first_save = read_file(a);

    scene::Scene s2;
    for (int cycle = 0; cycle < 4; ++cycle) {
        const native::ParsedDocument doc = native::load_file(a);
        native::install_document(s2, native::ParsedDocument(doc));
        native::save_file(b, s2, doc.camera);
        CHECK(read_file(b) == first_save);
        std::filesystem::copy_file(
            b, a, std::filesystem::copy_options::overwrite_existing);
    }
    std::filesystem::remove(a);
    std::filesystem::remove(b);

    // The loaded list IS the authored list: eight constraints, every
    // kind in authored order, every field identical — the operands as
    // canonical indices, the tangent's case bit, the pin's parameter
    // index, the values exact.
    const scene::SceneNode* sk = s2.get_node(1);
    REQUIRE(sk != nullptr);
    REQUIRE(sk->kind == scene::NodeKind::Sketch);
    REQUIRE(sk->sketch_constraints.size() == 8);
    using K = sdf::SketchConstraint::Kind;
    const sdf::SketchConstraint* cs = sk->sketch_constraints.data();
    CHECK(cs[0].kind == K::Coincidence);
    CHECK(cs[1].kind == K::Parallel);
    CHECK(cs[2].kind == K::Perpendicular);
    CHECK(cs[3].kind == K::Tangent);
    CHECK(cs[4].kind == K::Length);
    CHECK(cs[5].kind == K::Radius);
    CHECK(cs[6].kind == K::Distance);
    CHECK(cs[7].kind == K::Pin);
    // The point operands: (entity, end) — the coincidence's pair.
    CHECK(cs[0].a.entity.chain == 0);
    CHECK(cs[0].a.entity.entity == 0);
    CHECK(cs[0].a.end == sdf::SketchEnd::End);
    CHECK(cs[0].b.entity.chain == 0);
    CHECK(cs[0].b.entity.entity == 1);
    CHECK(cs[0].b.end == sdf::SketchEnd::Start);
    // The tangent's case bit: DATA, round-tripped as written (−1).
    CHECK(cs[3].case_bit == -1);
    CHECK(cs[3].ea.entity == 0);
    CHECK(cs[3].eb.chain == 1);
    CHECK(cs[3].eb.entity == 0);
    // The values: exact decimals, quantizer-transparent.
    CHECK(cs[4].value == 4.0);
    CHECK(cs[5].value == 2.0);
    CHECK(cs[6].value == 5.0);
    CHECK(cs[7].value == 0.5);
    // THE PIN'S STABLE ADDRESS: chain 1, entity 0, field 0 — the
    // circle's cx, the same field before and after the round-trip.
    CHECK(cs[7].pin.entity.chain == 1);
    CHECK(cs[7].pin.entity.entity == 0);
    CHECK(cs[7].pin.field == 0);
    // And the field-wise identity against a FRESH authored build (the
    // m2's comparator discipline: values, never raw bytes).
    sdf::SketchConstraint fresh[8] = {
        sdf::constraint_coincidence(
            sdf::SketchPointRef{sdf::SketchEntityRef{0, 0},
                                sdf::SketchEnd::End},
            sdf::SketchPointRef{sdf::SketchEntityRef{0, 1},
                                sdf::SketchEnd::Start}),
        sdf::constraint_parallel(sdf::SketchEntityRef{0, 0},
                                 sdf::SketchEntityRef{0, 2}),
        sdf::constraint_perpendicular(sdf::SketchEntityRef{0, 1},
                                      sdf::SketchEntityRef{0, 2}),
        sdf::constraint_tangent(sdf::SketchEntityRef{0, 0},
                                sdf::SketchEntityRef{1, 0}, -1),
        sdf::constraint_length(sdf::SketchEntityRef{0, 0}, 4.0),
        sdf::constraint_radius(sdf::SketchEntityRef{1, 0}, 2.0),
        sdf::constraint_distance(
            sdf::SketchPointRef{sdf::SketchEntityRef{0, 0},
                                sdf::SketchEnd::Start},
            sdf::SketchPointRef{sdf::SketchEntityRef{0, 2},
                                sdf::SketchEnd::End},
            5.0),
        sdf::constraint_pin(
            sdf::SketchParamRef{sdf::SketchEntityRef{1, 0}, 0}, 0.5),
    };
    for (int i = 0; i < 8; ++i)
        CHECK(constraint_value_identical(cs[i], fresh[i]));
    CHECK(chains_value_identical(sk->sketch, s.get_node(1)->sketch));
}

TEST_CASE("G4 m4 — the doors: the NEWER file refused by name; the hand-edited file is the NEXT solve's (D2.3/D3.1)") {
    // (a) THE NEWER DOOR, by name: a file declaring a MINOR beyond
    // this build's is refused naming both versions and the direction
    // — migrators run upgrade-only, a downgrade is never guessed.
    const std::string newer_minor =
        "{\"format\": \"CAD_0 native\", \"version\": \"0.6.0\","
        " \"scene\": { \"next_id\": 1, \"nodes\": [] }}";
    CHECK_THROWS_WITH_AS(native::parse_document(newer_minor),
        doctest::Contains("0.6.0"), std::runtime_error);
    CHECK_THROWS_WITH_AS(native::parse_document(newer_minor),
        doctest::Contains("NEWER CAD_0"), std::runtime_error);
    CHECK_THROWS_WITH_AS(native::parse_document(newer_minor),
        doctest::Contains("upgrade-only"), std::runtime_error);

    // (b) THE VALUE CLASS IS THE LOADER'S: a tangent with a case bit
    // that is not ±1, a length at 0 — the FACTORIES' own rules,
    // enforced at load exactly as the entities' parameter class is.
    const std::string bad_case_bit =
        "{\"format\": \"CAD_0 native\", \"version\": \"0.5.0\","
        " \"scene\": { \"next_id\": 2, \"nodes\": ["
        "  { \"id\": 1, \"kind\": \"sketch\","
        "    \"sketch\": { \"chains\": ["
        "      { \"entities\": ["
        "        { \"type\": \"segment\", \"a\": [0.0, 0.0],"
        "          \"b\": [4.0, 0.0], \"interior_left\": true },"
        "        { \"type\": \"circle\", \"center\": [0.0, 2.0],"
        "          \"radius\": 2.0, \"hole\": false } ] } ],"
        "      \"constraints\": ["
        "        { \"kind\": \"tangent\","
        "          \"ea\": { \"chain\": 0, \"entity\": 0 },"
        "          \"eb\": { \"chain\": 0, \"entity\": 1 },"
        "          \"case_bit\": 0 } ] } } ] } }";
    CHECK_THROWS_WITH_AS(native::parse_document(bad_case_bit),
        doctest::Contains("the case bit must be +1 (ESTERNO) or -1 "
                          "(INTERNO)"), std::invalid_argument);

    // (c) THE UNKNOWN KIND: refused by name, with the newer/hand-edited
    // hint (the queued families — symmetry, axes, equations — are NOT
    // kind names a file may invent).
    const std::string queued_family =
        "{\"format\": \"CAD_0 native\", \"version\": \"0.5.0\","
        " \"scene\": { \"next_id\": 2, \"nodes\": ["
        "  { \"id\": 1, \"kind\": \"sketch\","
        "    \"sketch\": { \"chains\": ["
        "      { \"entities\": ["
        "        { \"type\": \"segment\", \"a\": [0.0, 0.0],"
        "          \"b\": [4.0, 0.0], \"interior_left\": true } ] } ],"
        "      \"constraints\": ["
        "        { \"kind\": \"symmetry\" } ] } } ] } }";
    CHECK_THROWS_WITH_AS(native::parse_document(queued_family),
        doctest::Contains("unknown constraint kind \"symmetry\""),
        std::runtime_error);

    // (d) THE HAND-EDITED FILE IS THE NEXT SOLVE'S (D3.1, inchiodate):
    // a constraint addressing an entity that does not exist LOADS —
    // the loader never validates the structure against the chains
    // (D2.3: the recipe is the truth; the load never solves) — and
    // the very next SOLVE (the model's admission, against the real
    // chains) refuses it with the kernel's own words. The
    // inconsistency belongs to the solve, never to the loader.
    const std::string hand_edited =
        "{\"format\": \"CAD_0 native\", \"version\": \"0.5.0\","
        " \"scene\": { \"next_id\": 2, \"nodes\": ["
        "  { \"id\": 1, \"kind\": \"sketch\","
        "    \"sketch\": { \"chains\": ["
        "      { \"entities\": ["
        "        { \"type\": \"segment\", \"a\": [0.0, 0.0],"
        "          \"b\": [4.0, 0.0], \"interior_left\": true } ] } ],"
        "      \"constraints\": ["
        "        { \"kind\": \"length\","
        "          \"ea\": { \"chain\": 0, \"entity\": 99 },"
        "          \"value\": 4.0 } ] } } ] } }";
    const native::ParsedDocument doc = native::parse_document(hand_edited);
    REQUIRE(doc.nodes.size() == 1);
    REQUIRE(doc.nodes[0].sketch_constraints.size() == 1);
    // The NEXT solve catches it: the model's structural rule, the
    // kernel's own message (the workbench's rebuild route is this
    // very add — the words the interface would say).
    sdf::SketchSolveModel model(doc.nodes[0].sketch);
    CHECK_THROWS_WITH_AS(model.add_constraint(doc.nodes[0].sketch_constraints[0]),
        doctest::Contains("out of range"), std::invalid_argument);
}

TEST_CASE("G4 m4 — the re-solve after the load: the golden-1 triangle and the golden-8 edit, bit-identical through the round-trip") {
    // THE PROOF THE ROUND-TRIP LOSES NOTHING OF THE GRAPH: the
    // triangle (12 − 12 = 0 FULL, D5.1's own numbers) solved, saved,
    // loaded — and the loaded state solves to ITSELF; then the
    // golden-8 edit (|BC| 5 → 5.5) applied on BOTH sides — the
    // pre-save scene and the loaded one — and the two solves agree,
    // bit for bit, with the ADR's derived C' = (−0.65625,
    // 2.927342811749249) (x = (9+16−30.25)/8, y = √(9−x²)).
    const auto triangle_scene = [] {
        scene::Scene s;
        std::vector<sdf::SketchChain> chains;
        chains.push_back({{sdf::sketch_segment(
                               math::Vec2f{0.10f, 0.05f},
                               math::Vec2f{4.15f, -0.05f}, true),
                           sdf::sketch_segment(math::Vec2f{4.10f, 0.02f},
                                               math::Vec2f{0.05f, 3.10f},
                                               true),
                           sdf::sketch_segment(math::Vec2f{0.08f, 3.15f},
                                               math::Vec2f{0.12f, -0.02f},
                                               true)}});
        std::vector<sdf::SketchConstraint> cs;
        cs.push_back(sdf::constraint_coincidence(
            sdf::SketchPointRef{sdf::SketchEntityRef{0, 0},
                                sdf::SketchEnd::End},
            sdf::SketchPointRef{sdf::SketchEntityRef{0, 1},
                                sdf::SketchEnd::Start}));
        cs.push_back(sdf::constraint_coincidence(
            sdf::SketchPointRef{sdf::SketchEntityRef{0, 1},
                                sdf::SketchEnd::End},
            sdf::SketchPointRef{sdf::SketchEntityRef{0, 2},
                                sdf::SketchEnd::Start}));
        cs.push_back(sdf::constraint_coincidence(
            sdf::SketchPointRef{sdf::SketchEntityRef{0, 2},
                                sdf::SketchEnd::End},
            sdf::SketchPointRef{sdf::SketchEntityRef{0, 0},
                                sdf::SketchEnd::Start}));
        cs.push_back(
            sdf::constraint_length(sdf::SketchEntityRef{0, 0}, 4.0));
        cs.push_back(
            sdf::constraint_length(sdf::SketchEntityRef{0, 1}, 5.0));
        cs.push_back(
            sdf::constraint_length(sdf::SketchEntityRef{0, 2}, 3.0));
        cs.push_back(sdf::constraint_pin(
            sdf::SketchParamRef{sdf::SketchEntityRef{0, 0}, 0}, 0.0));
        cs.push_back(sdf::constraint_pin(
            sdf::SketchParamRef{sdf::SketchEntityRef{0, 0}, 1}, 0.0));
        cs.push_back(sdf::constraint_pin(
            sdf::SketchParamRef{sdf::SketchEntityRef{0, 0}, 3}, 0.0));
        s.add_sketch_node("Triangle", std::move(chains), std::move(cs));
        return s;
    };
    const auto solve_of = [](const scene::Scene& s) {
        sdf::SketchSolveModel model(s.get_node(1)->sketch);
        for (const auto& c : s.get_node(1)->sketch_constraints) {
            const auto rec = model.add_constraint(c);
            REQUIRE(rec.admitted);
        }
        return model.solve();
    };

    // (a) PRE-SAVE: the golden-1 solve — 12 − 12 = 0 FULL, solved,
    // the identity 3-4-5 landed (the derived coordinates).
    scene::Scene pre = triangle_scene();
    sdf::SketchSolveResult r_pre = solve_of(pre);
    REQUIRE(r_pre.status == sdf::SketchSolveStatus::Solved);
    CHECK(r_pre.bill.describe() == "12 - 12 = 0 (FULL)");
    // The store-back (the workbench's own route): the solved chains
    // become the recipe, THEN the file is written.
    {
        scene::SceneNode* n = pre.get_node_mut(1);
        n->sketch = r_pre.chains;
    }

    // (b) SAVE → LOAD (the round-trip): the file carries BOTH the
    // solved fields and the constraint list (D3.1: both are the
    // recipe, the load trusts both, never re-solved).
    const std::filesystem::path a =
        std::filesystem::temp_directory_path() / "cad0_resolve_rt.CAD_0";
    native::save_file(a, pre, make_canonical_camera());
    const native::ParsedDocument doc = native::load_file(a);
    scene::Scene post;
    native::install_document(post, native::ParsedDocument(doc));
    std::filesystem::remove(a);

    // (c) THE LOADED STATE SOLVES TO ITSELF: bit-identical chains —
    // the graph survived the round-trip whole.
    sdf::SketchSolveResult r_post = solve_of(post);
    REQUIRE(r_post.status == sdf::SketchSolveStatus::Solved);
    CHECK(r_post.bill.describe() == "12 - 12 = 0 (FULL)");
    CHECK(chains_value_identical(r_post.chains, r_pre.chains));

    // (d) THE GOLDEN-8 EDIT ON BOTH SIDES: |BC| (s1's length) 5 →
    // 5.5 — the re-value through the factory (index 4 in the
    // authored list), the re-solve from the pre-edit solved geometry.
    // Both sides agree, bit for bit, with the ADR's closed form.
    const auto edit_and_solve = [&solve_of](scene::Scene& s) {
        scene::SceneNode* n = s.get_node_mut(1);
        n->sketch_constraints[4] = sdf::constraint_length(
            sdf::SketchEntityRef{0, 1}, 5.5);
        return solve_of(s);
    };
    sdf::SketchSolveResult r_edit_pre = edit_and_solve(pre);
    sdf::SketchSolveResult r_edit_post = edit_and_solve(post);
    REQUIRE(r_edit_pre.status == sdf::SketchSolveStatus::Solved);
    REQUIRE(r_edit_post.status == sdf::SketchSolveStatus::Solved);
    CHECK(chains_value_identical(r_edit_pre.chains, r_edit_post.chains));
    // The derived C' — the golden-8 closed form, on BOTH sides. The
    // solved values are the STORED float32 (the store IS the
    // quantizer, ADR-0014): the closed form holds at the store's own
    // resolution (~2.4e-7 at scale 3), and the two SIDES agree
    // exactly, bit for bit.
    const sdf::SketchSegment& c_pre =
        r_edit_pre.chains[0].entities[2].segment;
    const sdf::SketchSegment& c_post =
        r_edit_post.chains[0].entities[2].segment;
    CHECK(c_pre.a.x == doctest::Approx(-0.65625).epsilon(1e-6));
    CHECK(c_pre.a.y == doctest::Approx(2.927342811749249).epsilon(1e-6));
    CHECK(c_post.a.x == c_pre.a.x);
    CHECK(c_post.a.y == c_pre.a.y);
    CHECK(c_pre.b.x == c_post.b.x);
    CHECK(c_pre.b.y == c_post.b.y);
}

TEST_CASE("sketch: parameter-class corruption is a loud load error (ADR-0029 family)") {
    const auto sketch_doc =
        [](const char* entities, const char* version = "0.5.0") {
            return std::string("{\"format\": \"CAD_0 native\", \"version\": \"") +
                   version +
                   "\", \"scene\": { \"next_id\": 2, \"nodes\": ["
                   "  { \"id\": 1, \"name\": \"S\", \"kind\": \"sketch\","
                   "    \"sketch\": { \"chains\": [ { \"entities\": [ " +
                   std::string(entities) + " ] } ] } }"
                   " ] } }";
        };

    // radius <= 0 (the NaN radius's integer sibling: JSON has no NaN
    // literal, but the class is the same — a non-positive radius
    // degenerates the disc).
    CHECK_THROWS_WITH_AS(native::parse_document(sketch_doc(
        "{ \"type\": \"circle\", \"center\": [0.0, 0.0], \"radius\": 0.0,"
        " \"hole\": false }")),
        doctest::Contains("radius must be > 0"), std::runtime_error);
    CHECK_THROWS_WITH_AS(native::parse_document(sketch_doc(
        "{ \"type\": \"circle\", \"center\": [0.0, 0.0], \"radius\": -0.25,"
        " \"hole\": false }")),
        doctest::Contains("radius must be > 0"), std::runtime_error);
    // A huge-but-parseable number overflows the FLOAT cast to infinity
    // (the parser itself rejects anything beyond double range first —
    // "number out of range" — so 1e300 is the class's representative:
    // a finite double that ceases to be a finite float).
    CHECK_THROWS_WITH_AS(native::parse_document(sketch_doc(
        "{ \"type\": \"circle\", \"center\": [0.0, 0.0], \"radius\": 1e300,"
        " \"hole\": false }")),
        doctest::Contains("not finite"), std::runtime_error);
    // The same overflow through the CENTER components.
    CHECK_THROWS_WITH_AS(native::parse_document(sketch_doc(
        "{ \"type\": \"circle\", \"center\": [1e300, 0.0], \"radius\": 0.25,"
        " \"hole\": false }")),
        doctest::Contains("must be finite"), std::runtime_error);
    // Degenerate segment (p0 == p1).
    CHECK_THROWS_WITH_AS(native::parse_document(sketch_doc(
        "{ \"type\": \"segment\", \"a\": [0.0, 0.0], \"b\": [0.0, 0.0],"
        " \"interior_left\": true }")),
        doctest::Contains("p0 != p1"), std::runtime_error);
    // Degenerate arc sweeps: zero, negative, a full turn.
    CHECK_THROWS_WITH_AS(native::parse_document(sketch_doc(
        "{ \"type\": \"arc\", \"center\": [0.0, 0.0], \"radius\": 0.25,"
        " \"hole\": false, \"start_angle\": 0.0, \"sweep\": 0.0 }")),
        doctest::Contains("sweep must lie in (0, 2"), std::runtime_error);
    CHECK_THROWS_WITH_AS(native::parse_document(sketch_doc(
        "{ \"type\": \"arc\", \"center\": [0.0, 0.0], \"radius\": 0.25,"
        " \"hole\": false, \"start_angle\": 0.0, \"sweep\": -1.0 }")),
        doctest::Contains("sweep must lie in (0, 2"), std::runtime_error);
    CHECK_THROWS_WITH_AS(native::parse_document(sketch_doc(
        "{ \"type\": \"arc\", \"center\": [0.0, 0.0], \"radius\": 0.25,"
        " \"hole\": false, \"start_angle\": 0.0,"
        " \"sweep\": 6.283185307179586 }")),
        doctest::Contains("sweep must lie in (0, 2"), std::runtime_error);
    // Unknown entity type.
    CHECK_THROWS_WITH_AS(native::parse_document(sketch_doc(
        "{ \"type\": \"spline\", \"a\": [0.0, 0.0], \"b\": [1.0, 0.0] }")),
        doctest::Contains("unknown entity type"), std::runtime_error);
    // A sketch node never carries the other representations' fields.
    CHECK_THROWS_WITH_AS(native::parse_document(
        "{\"format\": \"CAD_0 native\", \"version\": \"0.5.0\","
        " \"scene\": { \"next_id\": 2, \"nodes\": ["
        "  { \"id\": 1, \"name\": \"S\", \"kind\": \"sketch\","
        "    \"body\": { \"op\": \"box\", \"extent\": [1.0, 1.0, 1.0] },"
        "    \"sketch\": { \"chains\": [ { \"entities\": ["
        "      { \"type\": \"circle\", \"center\": [0.0, 0.0],"
        "        \"radius\": 0.25, \"hole\": false } ] } ] } }"
        " ] } }"),
        doctest::Contains("non-sketch field"), std::runtime_error);
}

TEST_CASE("sketch: an open chain loads as the legal editing state; its extrude is the visible pending state") {
    // The loader SPLITS the two (D5's preamble amendment, D4's clause):
    // the sketch loads — an open chain is legal, visibly-flagged 2D
    // data (revision 1) — while the extrude's materialization refuses
    // (chain closure is the extrude's precondition): visible PENDING
    // state, the file body kept, the certificate's own message
    // carried. NOT a load error: recovery is the next edit (close the
    // chain) away.
    const std::string text = R"JSON({
      "format": "CAD_0 native",
      "version": "0.5.0",
      "scene": {
        "next_id": 3,
        "nodes": [
          { "id": 1, "name": "Open_Sketch", "kind": "sketch",
            "sketch": { "chains": [ { "entities": [
              { "type": "segment", "a": [0.0, 0.0], "b": [1.0, 0.0],
                "interior_left": true },
              { "type": "segment", "a": [1.0, 0.0], "b": [1.0, 1.0],
                "interior_left": true }
            ] } ] } },
          { "id": 2, "name": "Extrude",
            "body": { "op": "box", "extent": [1.0, 1.0, 1.0] },
            "operation": { "type": "extrude", "sketch": 1,
                           "params": { "distance": 2.0, "direction": 1 } } }
        ]
      }
    })JSON";

    const native::ParsedDocument doc = native::parse_document(text);
    REQUIRE(doc.nodes.size() == 2);

    const scene::SceneNode* sk = &doc.nodes[0];
    REQUIRE(sk->id == 1);
    REQUIRE(sk->kind == scene::NodeKind::Sketch);
    REQUIRE(sk->sketch.size() == 1);
    // The legal editing state: the chain is OPEN, by the kernel's own
    // predicate (the one opinion the sketch node, the extrude's
    // precondition and the canvas share).
    CHECK(sdf::chain_is_closed(sk->sketch[0]) == false);

    const scene::SceneNode* ex = &doc.nodes[1];
    REQUIRE(ex->id == 2);
    REQUIRE(ex->operation.has_value());
    CHECK(ex->operation->type == scene::OperationType::Extrude);
    // The visible PENDING state — not dangling (the sketch EXISTS),
    // not healthy (the certificate refused).
    CHECK(ex->operation_pending);
    CHECK_FALSE(ex->operand_missing);
    // The file body is kept (the last-good cache).
    CHECK(ex->body.root() != nullptr);
    CHECK(std::string(ex->body.root()->describe()).rfind("box(", 0) == 0);
    // The pending error carries the certificate's own message (the
    // gap named, the remedy included).
    CHECK(ex->operation_pending_error.find("chain open") !=
          std::string::npos);
    CHECK(ex->operation_pending_error.find("Remedy") !=
          std::string::npos);
}

TEST_CASE("sketch: a missing sketch operand loads with the visible dangling state") {
    // v0.4 file whose extrude references sketch 99, which does not
    // exist: the load SUCCEEDS (the document opens), the extrude
    // keeps its FILE body (the last-good display geometry) and enters
    // the visible dangling state — never a load error, never a rebind.
    const std::string text = R"JSON({
      "format": "CAD_0 native",
      "version": "0.5.0",
      "scene": {
        "next_id": 3,
        "nodes": [
          { "id": 1, "name": "Extrude",
            "body": { "op": "box", "extent": [1.0, 1.0, 1.0] },
            "operation": { "type": "extrude", "sketch": 99,
                           "params": { "distance": 2.0, "direction": 1 } } }
        ]
      }
    })JSON";

    const native::ParsedDocument doc = native::parse_document(text);
    REQUIRE(doc.nodes.size() == 1);
    const scene::SceneNode* ex = &doc.nodes[0];
    REQUIRE(ex->operation.has_value());
    CHECK(ex->operation->sketch == 99);
    CHECK(ex->operand_missing);
    CHECK_FALSE(ex->operation_pending);
    CHECK(ex->body.root() != nullptr);
    CHECK(std::string(ex->body.root()->describe()).rfind("box(", 0) == 0);
}

TEST_CASE("sketch: extrude self-reference and operand-kind mismatch are explicit load errors") {
    // Self-reference: the extrude is its own sketch.
    const std::string selfref = R"JSON({
      "format": "CAD_0 native",
      "version": "0.5.0",
      "scene": { "next_id": 2, "nodes": [
        { "id": 1, "name": "Ouroboros",
          "body": { "op": "box", "extent": [1.0, 1.0, 1.0] },
          "operation": { "type": "extrude", "sketch": 1,
                         "params": { "distance": 2.0, "direction": 1 } } }
      ] }
    })JSON";
    CHECK_THROWS_WITH_AS(native::parse_document(selfref),
        doctest::Contains("references ITSELF"), std::runtime_error);

    // Operand-kind mismatch, both directions: an extrude consuming an
    // Sdf node, a subtract consuming a Sketch node. Structurally
    // corrupt — no writer can produce either (creation validates
    // kinds; a node's kind never mutates) — and the materializer
    // would read the wrong payload (an empty chain list, an empty
    // body). Explicit, by role.
    const std::string ex_of_sdf = R"JSON({
      "format": "CAD_0 native",
      "version": "0.5.0",
      "scene": { "next_id": 3, "nodes": [
        { "id": 1, "name": "Box",
          "body": { "op": "box", "extent": [1.0, 1.0, 1.0] } },
        { "id": 2, "name": "Extrude",
          "body": { "op": "box", "extent": [1.0, 1.0, 1.0] },
          "operation": { "type": "extrude", "sketch": 1,
                         "params": { "distance": 2.0, "direction": 1 } } }
      ] }
    })JSON";
    CHECK_THROWS_WITH_AS(native::parse_document(ex_of_sdf),
        doctest::Contains("must be a sketch-kind node"), std::runtime_error);

    const std::string sub_of_sketch = R"JSON({
      "format": "CAD_0 native",
      "version": "0.5.0",
      "scene": { "next_id": 3, "nodes": [
        { "id": 1, "name": "S", "kind": "sketch",
          "sketch": { "chains": [ { "entities": [
            { "type": "circle", "center": [0.0, 0.0], "radius": 0.25,
              "hole": false } ] } ] } },
        { "id": 2, "name": "Subtract",
          "body": { "op": "box", "extent": [1.0, 1.0, 1.0] },
          "operation": { "type": "subtract", "target": 1, "tool": 1 } }
      ] }
    })JSON";
    CHECK_THROWS_WITH_AS(native::parse_document(sub_of_sketch),
        doctest::Contains("must be a sdf-kind node"), std::runtime_error);
}

TEST_CASE("sketch: extrude recipe parameter-class corruption is a loud load error") {
    const auto extrude_doc = [](const char* params) {
        return std::string(R"JSON({
          "format": "CAD_0 native", "version": "0.5.0",
          "scene": { "next_id": 3, "nodes": [
            { "id": 1, "name": "S", "kind": "sketch",
              "sketch": { "chains": [ { "entities": [
                { "type": "circle", "center": [0.0, 0.0], "radius": 0.25,
                  "hole": false } ] } ] } },
            { "id": 2, "name": "Extrude",
              "body": { "op": "box", "extent": [1.0, 1.0, 1.0] },
              "operation": { "type": "extrude", "sketch": 1,
                             "params": { )JSON") +
               std::string(params) + " } } } ] } }";
    };

    CHECK_THROWS_WITH_AS(native::parse_document(extrude_doc(
        "\"distance\": 0.0, \"direction\": 1")),
        doctest::Contains("distance\" must be finite and > 0"),
        std::runtime_error);
    CHECK_THROWS_WITH_AS(native::parse_document(extrude_doc(
        "\"distance\": -2.0, \"direction\": 1")),
        doctest::Contains("distance\" must be finite and > 0"),
        std::runtime_error);
    CHECK_THROWS_WITH_AS(native::parse_document(extrude_doc(
        "\"distance\": 1e300, \"direction\": 1")),
        doctest::Contains("distance\" must be finite and > 0"),
        std::runtime_error);
    CHECK_THROWS_WITH_AS(native::parse_document(extrude_doc(
        "\"distance\": 2.0, \"direction\": 0")),
        doctest::Contains("direction\" must be +1 or -1"),
        std::runtime_error);
    CHECK_THROWS_WITH_AS(native::parse_document(extrude_doc(
        "\"distance\": 2.0, \"direction\": 2")),
        doctest::Contains("direction\" must be +1 or -1"),
        std::runtime_error);
}

TEST_CASE("sketch: pre-v0.4 anachronisms are explicit load errors") {
    // A sketch payload or an extrude recipe in a file that DECLARES a
    // pre-0.4 version: no writer of that version ever emitted one (the
    // v0.3 writer refused loudly) — the file is not what its version
    // claims. The migrator is a content no-op and cannot bless it.
    const std::string sketch_in_v03 =
        "{\"format\": \"CAD_0 native\", \"version\": \"0.3.0\","
        " \"scene\": { \"next_id\": 2, \"nodes\": ["
        "  { \"id\": 1, \"name\": \"S\", \"kind\": \"sketch\","
        "    \"sketch\": { \"chains\": [ { \"entities\": ["
        "      { \"type\": \"circle\", \"center\": [0.0, 0.0],"
        "        \"radius\": 0.25, \"hole\": false } ] } ] } }"
        " ] } }";
    CHECK_THROWS_WITH_AS(native::parse_document(sketch_in_v03),
        doctest::Contains("not what its version claims"), std::runtime_error);

    const std::string extrude_in_v03 =
        "{\"format\": \"CAD_0 native\", \"version\": \"0.3.0\","
        " \"scene\": { \"next_id\": 3, \"nodes\": ["
        "  { \"id\": 1, \"name\": \"Box\","
        "    \"body\": { \"op\": \"box\", \"extent\": [1.0, 1.0, 1.0] } },"
        "  { \"id\": 2, \"name\": \"Extrude\","
        "    \"body\": { \"op\": \"box\", \"extent\": [1.0, 1.0, 1.0] },"
        "    \"operation\": { \"type\": \"extrude\", \"sketch\": 1,"
        "                     \"params\": { \"distance\": 2.0,"
        "                                  \"direction\": 1 } } }"
        " ] } }";
    CHECK_THROWS_WITH_AS(native::parse_document(extrude_in_v03),
        doctest::Contains("not what its version claims"), std::runtime_error);
}

TEST_CASE("sketch: a stale cached extrude body is caught by the pass-2 rebuild") {
    // The file's cached body disagrees with the recipe (the sketch is
    // a circle r=0.5, the cache says a plain box). Pass 2
    // re-materializes from the recipe: the loaded body must be the
    // extrude prism, not the stale box cache.
    const std::string stale = R"JSON({
      "format": "CAD_0 native",
      "version": "0.5.0",
      "scene": {
        "next_id": 3,
        "nodes": [
          { "id": 1, "name": "S", "kind": "sketch",
            "sketch": { "chains": [ { "entities": [
              { "type": "circle", "center": [0.0, 0.0], "radius": 0.5,
                "hole": false } ] } ] } },
          { "id": 2, "name": "Extrude",
            "body": { "op": "box", "extent": [1.0, 1.0, 1.0] },
            "operation": { "type": "extrude", "sketch": 1,
                           "params": { "distance": 2.0, "direction": 1 } } }
        ]
      }
    })JSON";

    const native::ParsedDocument doc = native::parse_document(stale);
    REQUIRE(doc.nodes.size() == 2);
    const scene::SceneNode* ex = &doc.nodes[1];
    REQUIRE(ex->id == 2);
    // The rebuild won: the body is the extrude prism (per the recipe),
    // not the stale plain-box cache the file carried.
    CHECK(std::string(ex->body.root()->describe()).rfind("extrude(", 0) == 0);
    CHECK_FALSE(ex->operand_missing);
    CHECK_FALSE(ex->operation_pending);
}

TEST_CASE("operations: file with a missing operand loads with the error state") {
    // Hand-built v0.3 document whose operation references operand 2,
    // which does not exist: the load SUCCEEDS (the document opens),
    // the operation keeps its FILE body (last-good display geometry)
    // and enters the visible dangling state.
    const std::string text = R"JSON({
      "format": "CAD_0 native",
      "version": "0.3.0",
      "scene": {
        "next_id": 4,
        "nodes": [
          { "id": 1, "name": "Box", "builder": "box",
            "params": { "extent_x": 1.0, "extent_y": 1.0, "extent_z": 1.0 },
            "body": { "op": "box", "extent": [1.0, 1.0, 1.0] } },
          { "id": 3, "name": "Subtract",
            "body": { "op": "subtract",
                      "a": { "op": "box", "extent": [1.0, 1.0, 1.0] },
                      "b": { "op": "translate",
                             "child": { "op": "cylinder", "radius": 0.25, "height": 3.0 },
                             "offset": [0.0, 0.0, 0.0] } },
            "operation": { "type": "subtract", "target": 1, "tool": 2,
                           "params": { "offset_x": 0.0, "offset_y": 0.0,
                                       "offset_z": 0.0 } } }
        ]
      }
    })JSON";

    const native::ParsedDocument doc = native::parse_document(text);
    REQUIRE(doc.nodes.size() == 2);
    const scene::SceneNode* op =
        doc.nodes[0].id == 3 ? &doc.nodes[0] : &doc.nodes[1];
    REQUIRE(op->id == 3);
    REQUIRE(op->operation.has_value());
    CHECK(op->operation->tool == 2);
    // The visible dangling state: the FILE body is kept.
    CHECK(op->operand_missing);
    CHECK(op->body.root() != nullptr);
    // The file body is the subtract tree serialized in the file (the
    // last-good cache), not a rebuilt anything.
    std::string desc = op->body.root()->describe();
    CHECK(desc.rfind("subtract(", 0) == 0);
}

TEST_CASE("operations: cyclic file is an explicit load error") {
    // A TRUE cycle (no self-reference): op 1 targets op 2, op 2 targets
    // op 1 — neither can be materialized first.
    const std::string cyclic = R"JSON({
      "format": "CAD_0 native",
      "version": "0.3.0",
      "scene": {
        "next_id": 4,
        "nodes": [
          { "id": 1, "name": "A", "body": { "op": "box", "extent": [1.0, 1.0, 1.0] },
            "operation": { "type": "subtract", "target": 2, "tool": 3 } },
          { "id": 2, "name": "B", "body": { "op": "box", "extent": [1.0, 1.0, 1.0] },
            "operation": { "type": "subtract", "target": 1, "tool": 3 } },
          { "id": 3, "name": "C", "body": { "op": "box", "extent": [1.0, 1.0, 1.0] } }
        ]
      }
    })JSON";
    CHECK_THROWS_WITH_AS(native::parse_document(cyclic),
        doctest::Contains("cyclic"), std::runtime_error);
}

TEST_CASE("operations: self-reference is an explicit load error") {
    const std::string selfref = R"JSON({
      "format": "CAD_0 native",
      "version": "0.3.0",
      "scene": {
        "next_id": 2,
        "nodes": [
          { "id": 1, "name": "Ouroboros",
            "body": { "op": "box", "extent": [1.0, 1.0, 1.0] },
            "operation": { "type": "subtract", "target": 1, "tool": 1 } }
        ]
      }
    })JSON";
    CHECK_THROWS_WITH_AS(native::parse_document(selfref),
        doctest::Contains("references ITSELF"), std::runtime_error);
}

TEST_CASE("operations: unknown operation type / bad fields are explicit errors") {
    const std::string unknown_type = R"JSON({
      "format": "CAD_0 native",
      "version": "0.3.0",
      "scene": {
        "next_id": 2,
        "nodes": [
          { "id": 1, "name": "X", "body": { "op": "box", "extent": [1.0, 1.0, 1.0] },
            "operation": { "type": "intersect", "target": 1, "tool": 1 } }
        ]
      }
    })JSON";
    CHECK_THROWS_WITH_AS(native::parse_document(unknown_type),
        doctest::Contains("unknown operation type"), std::runtime_error);

    // A mesh node can never carry an operation (the two
    // representations never mix — ADR-0002). The two-argument
    // parse_document is the load_file entry point: the text-only one
    // rejects blob-carrying documents before node parsing (ADR-0007a),
    // which would mask the check under test.
    const std::string mesh_op = R"JSON({
      "format": "CAD_0 native",
      "version": "0.3.0",
      "scene": {
        "next_id": 2,
        "nodes": [
          { "id": 1, "name": "M", "kind": "mesh",
            "mesh": { "blob": "mesh_1" },
            "operation": { "type": "subtract", "target": 1, "tool": 1 } }
        ]
      },
      "meshes": [ { "id": "mesh_1", "encoding": "raw", "offset": 0, "size": 0,
                    "vertex_count": 0, "triangle_count": 0 } ]
    })JSON";
    CHECK_THROWS_WITH_AS(native::parse_document(mesh_op, ""),
        doctest::Contains("carries an SDF-only field"), std::runtime_error);
}

TEST_CASE("operations: stale cached body is caught by the pass-2 rebuild") {
    // The file's cached body disagrees with the recipe (the box is
    // intact in the recipe operands, but the cache says a plain box).
    // Pass 2 re-materializes from the recipe — the loaded body must be
    // the subtract, not the stale cache.
    const std::string stale = R"JSON({
      "format": "CAD_0 native",
      "version": "0.3.0",
      "scene": {
        "next_id": 4,
        "nodes": [
          { "id": 1, "name": "Box", "builder": "box",
            "params": { "extent_x": 1.0, "extent_y": 1.0, "extent_z": 1.0 },
            "body": { "op": "box", "extent": [1.0, 1.0, 1.0] } },
          { "id": 2, "name": "Cylinder", "builder": "cylinder",
            "params": { "radius": 0.25, "height": 3.0 },
            "body": { "op": "cylinder", "radius": 0.25, "height": 3.0 } },
          { "id": 3, "name": "Subtract",
            "body": { "op": "box", "extent": [1.0, 1.0, 1.0] },
            "operation": { "type": "subtract", "target": 1, "tool": 2 } }
        ]
      }
    })JSON";

    const native::ParsedDocument doc = native::parse_document(stale);
    const scene::SceneNode* op = &doc.nodes[2];
    REQUIRE(op->id == 3);
    // The rebuild won: the body is the subtract (per the recipe), not
    // the stale plain-box cache the file carried.
    CHECK(op->body.root()->describe().rfind("subtract(", 0) == 0);
    CHECK_FALSE(op->operand_missing);
}

TEST_CASE("container: magic + legacy detection, explicit error on foreign files") {
    const std::string json_text =
        native::serialize_document(make_canonical_scene(), {});
    const std::string container = native::build_container(json_text, {});

    // Magic detection.
    CHECK(native::has_container_magic(container));
    CHECK(!native::has_container_magic(json_text)); // legacy JSON
    CHECK(!native::has_container_magic(""));        // too short

    // Round-trip through the codec level.
    const native::ContainerParts parts = native::split_container(container);
    CHECK(parts.json_text == json_text);
    CHECK(parts.blob_section.empty());

    // Legacy JSON (starts with '{') parses through the text entry point.
    CHECK_NOTHROW(native::parse_document(json_text));

    // Foreign binary fed as text fails at the JSON parse with the usual
    // line/column diagnostic (framing detection is load_file's job and
    // is covered by the save/load test below).
    const std::string foreign = "PK\x03\x04garbage";
    CHECK_THROWS_WITH_AS(native::parse_document(foreign, {}),
        doctest::Contains("parse error at line"), std::runtime_error);
}

TEST_CASE("container: truncation and length mismatch are explicit errors") {
    const std::string json_text =
        native::serialize_document(make_canonical_scene(), {});
    const std::string container =
        native::build_container(json_text, "12345678");

    // Chopping bytes off the end -> truncation error.
    const std::string truncated = container.substr(0, container.size() - 4);
    CHECK_THROWS_WITH_AS(native::split_container(truncated),
        doctest::Contains("do not add up"), std::runtime_error);

    // Corrupting json_len (+100) -> the declared total no longer matches.
    std::string bad = container;
    bad[11] = static_cast<char>(bad[11] + 100); // high byte of json_len
    CHECK_THROWS_WITH_AS(native::split_container(bad),
        doctest::Contains("do not add up"), std::runtime_error);

    // Frame shorter than the 16-byte header.
    CHECK_THROWS_WITH_AS(native::split_container("CAD0BLOB"),
        doctest::Contains("truncated"), std::runtime_error);

    // Wrong magic with full length.
    std::string wrong_magic = container;
    wrong_magic[0] = 'X';
    CHECK_THROWS_WITH_AS(native::split_container(wrong_magic),
        doctest::Contains("bad magic"), std::runtime_error);
}

TEST_CASE("blob codec: raw encoding round-trips bit-exactly") {
    sdf::TriangleMesh m;
    m.positions = {
        math::Vec3f{-0.0f, 1.5f, -2.25f},
        math::Vec3f{ 1e-40f, 3.14159f, 1e30f},   // denormal, pi, huge
        math::Vec3f{ 0.0f, 0.0f, 0.0f},
    };
    m.indices = {0, 1, 2, 1, 2, 0};
    m.normals = {
        math::Vec3f{ 0.0f, 1.0f, 0.0f},
        math::Vec3f{ 1.0f, 0.0f, -0.0f},
        math::Vec3f{ 0.0f, 0.0f, 1.0f},
    };

    const std::string blob = native::encode_blob_raw(m);
    CHECK(blob.size() == native::raw_blob_size(3, 2, true));

    // Bit-level spot check: normals[1].z is -0.0 (sign bit set) — sits at
    // blob offset 36 (positions) + 12 (normal 0) + 8 (x,y of normal 1).
    // Little-endian: 00 00 00 80.
    CHECK(static_cast<unsigned char>(blob[36 + 12 + 8 + 3]) == 0x80);

    const sdf::TriangleMesh back =
        native::decode_blob_raw(blob, "codec_test", 3, 2, true);
    CHECK(back.positions == m.positions); // -0.0 and denormals preserved
    CHECK(back.normals == m.normals);
    CHECK(back.indices == m.indices);

    // Without normals.
    sdf::TriangleMesh no_n = m;
    no_n.normals.clear();
    const sdf::TriangleMesh back2 =
        native::decode_blob_raw(native::encode_blob_raw(no_n),
                                "codec_test", 3, 2, false);
    CHECK(back2.normals.empty());
    CHECK(back2.positions == m.positions);

    // Empty mesh.
    const sdf::TriangleMesh empty =
        native::decode_blob_raw("", "codec_test", 0, 0, false);
    CHECK(empty.positions.empty());
    CHECK(empty.indices.empty());

    // Size/count mismatch and out-of-range indices are explicit errors.
    CHECK_THROWS_WITH_AS(native::decode_blob_raw(blob, "bad", 3, 2, false),
        doctest::Contains("payload size mismatch"), std::runtime_error);
    CHECK_THROWS_WITH_AS(
        native::decode_blob_raw(
            std::string(3 * 12 + 3 * 12, '\0') +
                std::string(12, '\xAB'), "bad", 3, 1, true),
        doctest::Contains("out of range"), std::runtime_error);
}

TEST_CASE("meshes: JSON header validation and blob section slicing") {
    // One triangle: 3 positions (36 bytes) + 3 uint32 indices (12 bytes).
    const std::string header_ok = R"JSON({
      "format": "CAD_0 native", "version": "0.2.0",
      "scene": { "next_id": 1, "nodes": [] },
      "meshes": [ { "id": "mesh_1", "encoding": "raw",
                    "offset": 0, "size": 48, "vertex_count": 3,
                    "triangle_count": 1, "has_normals": false } ]
    })JSON";

    // Blob content is zeros — valid: three (0,0,0) vertices and the
    // triangle (0,0,0). The point of this test is header validation and
    // section slicing, not geometry.
    const std::string blob(3 * 12 + 3 * 4, '\0');

    const native::ParsedDocument doc = native::parse_document(header_ok, blob);
    REQUIRE(doc.meshes.size() == 1);
    CHECK(doc.meshes[0].id == "mesh_1");
    CHECK(doc.meshes[0].mesh.vertex_count() == 3);
    CHECK(doc.meshes[0].mesh.triangle_count() == 1);

    // A blob section too short for the declared offset/size.
    const std::string short_blob(10, '\0');
    CHECK_THROWS_WITH_AS(native::parse_document(header_ok, short_blob),
        doctest::Contains("beyond the blob section"), std::runtime_error);

    // Unsupported encoding (zstd is deferred — ADR-0003/0007a).
    const std::string header_zstd = R"JSON({
      "format": "CAD_0 native", "version": "0.2.0",
      "scene": { "next_id": 1, "nodes": [] },
      "meshes": [ { "id": "m", "encoding": "zstd",
                    "offset": 0, "size": 10, "vertex_count": 0,
                    "triangle_count": 0 } ]
    })JSON";
    CHECK_THROWS_WITH_AS(native::parse_document(header_zstd, std::string(10, '\0')),
        doctest::Contains("zstd"), std::runtime_error);

    // Duplicate ids.
    const std::string header_dup = R"JSON({
      "format": "CAD_0 native", "version": "0.2.0",
      "scene": { "next_id": 1, "nodes": [] },
      "meshes": [ { "id": "m", "encoding": "raw", "offset": 0, "size": 0,
                    "vertex_count": 0, "triangle_count": 0 },
                  { "id": "m", "encoding": "raw", "offset": 0, "size": 0,
                    "vertex_count": 0, "triangle_count": 0 } ]
    })JSON";
    CHECK_THROWS_WITH_AS(native::parse_document(header_dup, ""),
        doctest::Contains("duplicate mesh blob id"), std::runtime_error);

    // Non-integer counts.
    const std::string header_frac = R"JSON({
      "format": "CAD_0 native", "version": "0.2.0",
      "scene": { "next_id": 1, "nodes": [] },
      "meshes": [ { "id": "m", "encoding": "raw", "offset": 0.5, "size": 0,
                    "vertex_count": 0, "triangle_count": 0 } ]
    })JSON";
    CHECK_THROWS_WITH_AS(native::parse_document(header_frac, ""),
        doctest::Contains("non-negative integer"), std::runtime_error);
}

TEST_CASE("meshes: text-only entry point rejects documents carrying blobs") {
    const std::string text = R"JSON({
      "format": "CAD_0 native", "version": "0.2.0",
      "scene": { "next_id": 1, "nodes": [] },
      "meshes": [ { "id": "mesh_1", "encoding": "raw", "offset": 0,
                    "size": 0, "vertex_count": 0, "triangle_count": 0 } ]
    })JSON";
    CHECK_THROWS_WITH_AS(native::parse_document(text),
        doctest::Contains("use load_file()"), std::runtime_error);
}

TEST_CASE("save/load: file round-trip through the container, legacy via load_file") {
    std::filesystem::path tmp_dir = std::filesystem::temp_directory_path();
    const std::filesystem::path f =
        tmp_dir / "cad0_fase0_container.CAD_0";
    const std::filesystem::path legacy =
        tmp_dir / "cad0_fase0_legacy.CAD_0";

    // SDF-only scene: blob section empty, magic present.
    native::save_file(f, make_canonical_scene(), make_canonical_camera());
    const std::string bytes = read_file(f);
    REQUIRE(bytes.size() >= native::kContainerHeaderSize);
    CHECK(std::memcmp(bytes.data(), native::kContainerMagic.data(),
                      native::kContainerMagic.size()) == 0);
    const native::ContainerParts parts = native::split_container(bytes);
    CHECK(parts.blob_section.empty());

    // File-level round-trip: bit-identical SDF values.
    const native::ParsedDocument doc = native::load_file(f);
    scene::Scene s;
    native::ParsedDocument doc_copy = doc;
    native::install_document(s, std::move(doc_copy));
    CHECK(sample_grid(s.node_at(0)->body, 7) ==
          sample_grid(make_canonical_scene().node_at(0)->body, 7));
    CHECK(doc.meshes.empty());

    // A legacy pure-JSON file (written by hand as v0.1.0) loads through
    // load_file's framing detection + the migrator.
    {
        std::ofstream out(legacy, std::ios::binary | std::ios::trunc);
        out << R"JSON({
          "format": "CAD_0 native", "version": "0.1.0",
          "scene": { "next_id": 2, "nodes": [ { "id": 1,
            "body": { "op": "sphere", "radius": 1.25 } } ] }
        })JSON";
    }
    const native::ParsedDocument legacy_doc = native::load_file(legacy);
    REQUIRE(legacy_doc.nodes.size() == 1);
    CHECK(legacy_doc.version == native::parse_version("0.1.0"));
    CHECK(legacy_doc.nodes[0].body.value(math::Vec3f{0.0f, 0.0f, 0.0f}) ==
          doctest::Approx(-1.25f));

    // A foreign binary file fails the framing check.
    const std::filesystem::path foreign = tmp_dir / "cad0_fase0_foreign.bin";
    {
        std::ofstream out(foreign, std::ios::binary | std::ios::trunc);
        out << "PK\x03\x04 not a CAD_0 file";
    }
    CHECK_THROWS_WITH_AS(native::load_file(foreign),
        doctest::Contains("not a CAD_0 native document"), std::runtime_error);

    std::filesystem::remove(f);
    std::filesystem::remove(legacy);
    std::filesystem::remove(foreign);
}

TEST_CASE("corruption: malformed JSON reports line/column; scene untouched") {
    scene::Scene s = make_canonical_scene();
    const std::string broken = native::serialize_document(s, {}) .substr(0, 40);

    scene::Scene target = make_canonical_scene();
    const std::size_t before = target.size();
    CHECK_THROWS_WITH_AS(native::parse_document(broken),
        doctest::Contains("parse error at line"), std::runtime_error);
    CHECK(target.size() == before); // untouched
}

TEST_CASE("corruption: unknown SDF op is an explicit error") {
    const std::string doc = R"JSON({
      "format": "CAD_0 native",
      "version": "0.1.0",
      "scene": { "next_id": 2, "nodes": [ { "id": 1, "body": { "op": "flux_capacitor" } } ] }
    })JSON";
    CHECK_THROWS_WITH_AS(native::parse_document(doc),
        doctest::Contains("unknown SDF op \"flux_capacitor\""), std::runtime_error);
}

TEST_CASE("corruption: missing fields are explicit errors") {
    const std::string no_next = R"JSON({
      "format": "CAD_0 native", "version": "0.1.0",
      "scene": { "nodes": [] }
    })JSON";
    CHECK_THROWS_WITH_AS(native::parse_document(no_next),
        doctest::Contains("next_id"), std::runtime_error);

    const std::string no_body = R"JSON({
      "format": "CAD_0 native", "version": "0.1.0",
      "scene": { "next_id": 2, "nodes": [ { "id": 1 } ] }
    })JSON";
    CHECK_THROWS_WITH_AS(native::parse_document(no_body),
        doctest::Contains("body"), std::runtime_error);

    const std::string no_op = R"JSON({
      "format": "CAD_0 native", "version": "0.1.0",
      "scene": { "next_id": 2, "nodes": [ { "id": 1, "body": { "radius": 1.0 } } ] }
    })JSON";
    CHECK_THROWS_WITH_AS(native::parse_document(no_op),
        doctest::Contains("no \"op\" field"), std::runtime_error);
}

TEST_CASE("version mismatch without a migrator is an explicit error (ADR-0007)") {
    const std::string newer = R"JSON({
      "format": "CAD_0 native",
      "version": "1.0.0",
      "scene": { "next_id": 1, "nodes": [] }
    })JSON";
    CHECK_THROWS_WITH_AS(native::parse_document(newer),
        doctest::Contains("no migrator registered"), std::runtime_error);

    // PATCH differences are compatible (both directions).
    const std::string patch_newer = R"JSON({
      "format": "CAD_0 native",
      "version": "0.1.3",
      "scene": { "next_id": 1, "nodes": [] }
    })JSON";
    auto doc = native::parse_document(patch_newer); // must not throw
    CHECK(doc.nodes.empty());
}

TEST_CASE("foreign/wrong formats fail loudly") {
    CHECK_THROWS_WITH_AS(native::parse_document("{\"version\": \"0.1.0\"}"),
        doctest::Contains("missing \"format\""), std::runtime_error);
    CHECK_THROWS_WITH_AS(native::parse_document(
        "{\"format\": \"something else\", \"version\": \"0.1.0\"}"),
        doctest::Contains("not a CAD_0 native document"), std::runtime_error);
}

TEST_CASE("corruption: out-of-class SDF parameters are explicit load errors (ADR-0029 D5)") {
    // A scale node with s = 0 in a file: the constructor's
    // std::invalid_argument propagates out of parse_document and joins
    // the ADR-0007 explicit-error family — same test pattern as the
    // corruption cases above, exception type std::invalid_argument
    // (ValueError at the Python binding). The message is the FULL
    // bridge-style constructor message: operator, parameter, value,
    // supported class, diagnosis, remedy.
    const std::string zero_scale = R"JSON({
      "format": "CAD_0 native",
      "version": "0.1.0",
      "scene": { "next_id": 2, "nodes": [ { "id": 1, "body": {
        "op": "scale", "s": 0.0,
        "child": { "op": "sphere", "radius": 1.0 } } } ] }
    })JSON";
    CHECK_THROWS_WITH_AS(native::parse_document(zero_scale),
        doctest::Contains("ScaleSDF: scale factor s must be finite and > 0"),
        std::invalid_argument);
    CHECK_THROWS_WITH_AS(native::parse_document(zero_scale),
        doctest::Contains("got 0.000000"), std::invalid_argument);
    CHECK_THROWS_WITH_AS(native::parse_document(zero_scale),
        doctest::Contains("Remedy: scale by s > 0"), std::invalid_argument);

    // The same check's other out-of-class sign: s < 0 in a file.
    const std::string neg_scale = R"JSON({
      "format": "CAD_0 native",
      "version": "0.1.0",
      "scene": { "next_id": 2, "nodes": [ { "id": 1, "body": {
        "op": "scale", "s": -1.5,
        "child": { "op": "sphere", "radius": 1.0 } } } ] }
    })JSON";
    CHECK_THROWS_WITH_AS(native::parse_document(neg_scale),
        doctest::Contains("ScaleSDF: scale factor s must be finite and > 0"),
        std::invalid_argument);

    // An empty union children list in a file: the N-ary check.
    const std::string empty_union = R"JSON({
      "format": "CAD_0 native",
      "version": "0.1.0",
      "scene": { "next_id": 2, "nodes": [ { "id": 1, "body": {
        "op": "union", "children": [] } } ] }
    })JSON";
    CHECK_THROWS_WITH_AS(native::parse_document(empty_union),
        doctest::Contains("UnionSDF: children must be a non-empty list"),
        std::invalid_argument);
}

TEST_CASE("atomic save: writes file, no tmp residue; failure leaves original intact") {
    scene::Scene s = make_canonical_scene();
    const native::NativeCamera cam = make_canonical_camera();

    std::filesystem::path tmp_dir = std::filesystem::temp_directory_path();
    tmp_dir /= "cad0_test_native_atomic";
    std::filesystem::create_directories(tmp_dir);
    const std::filesystem::path path = tmp_dir / "project.CAD_0";

    native::save_file(path, s, cam);
    CHECK(std::filesystem::exists(path));
    CHECK(!std::filesystem::exists(path.string() + ".tmp"));

    // Round-trip through the file API.
    scene::Scene loaded;
    const native::NativeCamera cam2 = [&] {
        auto doc = native::load_file(path);
        const native::NativeCamera out = doc.camera;
        native::install_document(loaded, std::move(doc));
        return out;
    }();
    CHECK(loaded.size() == 2);
    CHECK(cam2.distance == cam.distance);

    // Missing file → explicit error.
    CHECK_THROWS_WITH_AS(native::load_file(tmp_dir / "nope.CAD_0"),
        doctest::Contains("cannot open"), std::runtime_error);

    std::error_code ec;
    std::filesystem::remove_all(tmp_dir, ec);
}

TEST_CASE("json writer: deterministic, insertion-ordered output") {
    using CAD_0::io::json::JsonValue;
    JsonValue v;
    v.set("zeta", 1).set("alpha", JsonValue("s")).set("mid", JsonValue(true));
    const std::string out = v.dump(false);
    CHECK(out == R"({"zeta":1,"alpha":"s","mid":true})");

    // Float32 round-trip through the shortest-round-trip formatter.
    const float f = 0.1f; // classic non-representable value
    const double d = static_cast<double>(f);
    const JsonValue num(d);
    const std::string text = num.dump(false); // e.g. "0.1"
    const float back = static_cast<float>(std::stod(text));
    CHECK(back == f);
}

// ---- post-load parametric editing (factory re-wiring) -----------------------
//
// Diagnostic for the suspected bug: SceneNode::factory is a std::function
// and cannot be serialized. After a load it was EMPTY, so set_node_param()
// on a loaded node updated `params` but silently did NOT rebuild the body
// (rebuild_body() no-ops without a factory) and did not re-flag
// mesh_dirty. The UI hides this by rebuilding bodies in Python, but the
// kernel contract is that set_param rebuilds the body — the UI is a
// client of the kernel, not the only place editing may work.
//
// The fix re-wires a parametric ShapeFactory on every loaded node that
// carries a recognized `builder` tag (sphere/box/cylinder/torus — the
// same tags + param names the UI persists). These tests pin the contract:
//   * a loaded node with a known builder tag is fully editable;
//   * a node without a builder tag (or an unknown one) stays
//     non-parametric: it still loads and round-trips its body, but
//     set_param only updates the stored params.
//
// The source scenes below are built exactly the way the desktop app
// builds them (add_node + builder tag + params, no factory) so the saved
// file is byte-identical to a real user project.

namespace {

// Radius of a sphere body, read back through the JSON serializer.
float serialized_radius(const sdf::SDFBody& body) {
    const auto v = native::serialize_node(*body.root());
    return static_cast<float>(v.at("radius").as_number());
}

} // namespace

TEST_CASE("post-load param edit: set_node_param rebuilds body + flags mesh_dirty") {
    // Scene as the app saves it: builder tag + params, factory never set.
    scene::Scene s;
    {
        scene::SceneNode n;
        n.id = 4;
        n.name = "Sphere";
        n.builder = "sphere";
        n.params = {{"radius", 1.5f}};
        n.body = sdf::make_sphere(1.5f);
        s.restore_node(std::move(n));
    }
    s.set_next_id(5);

    std::filesystem::path tmp_dir = std::filesystem::temp_directory_path();
    tmp_dir /= "cad0_test_native_edit";
    std::filesystem::create_directories(tmp_dir);
    const std::filesystem::path path = tmp_dir / "sphere.CAD_0";
    native::save_file(path, s, native::NativeCamera{});

    scene::Scene loaded;
    native::install_document(loaded, native::load_file(path));

    scene::SceneNode* n = loaded.get_node_mut(4);
    REQUIRE(n != nullptr);

    // The viewport consumes the post-load dirty flag on first paint.
    loaded.update_meshes(8);
    REQUIRE(n->mesh_dirty == false);

    const auto before = sample_grid(n->body, 7);
    REQUIRE(serialized_radius(n->body) == 1.5f);

    // The kernel-level edit the property panel relies on conceptually.
    loaded.set_node_param(4, "radius", 3.0f);

    // (a) params were updated...
    CHECK(n->params.at("radius") == scene::ParamValue{3.0f});
    // (b) the BODY was rebuilt from them (the actual bug)...
    const auto after = sample_grid(n->body, 7);
    CHECK(before != after);
    CHECK(serialized_radius(n->body) == 3.0f);
    const auto expected = sample_grid(sdf::make_sphere(3.0f), 7);
    CHECK(after == expected); // bit-identical to a fresh sphere(3.0)
    // (c) ...and re-tessellation was flagged.
    CHECK(n->mesh_dirty == true);

    // The edit persists across another save/load cycle.
    native::save_file(path, loaded, native::NativeCamera{});
    scene::Scene reloaded;
    native::install_document(reloaded, native::load_file(path));
    CHECK(reloaded.get_node(4)->params.at("radius") == scene::ParamValue{3.0f});
    CHECK(serialized_radius(reloaded.get_node(4)->body) == 3.0f);

    std::error_code ec;
    std::filesystem::remove_all(tmp_dir, ec);
}

TEST_CASE("post-load param edit: every UI builder tag re-wires its factory") {
    struct EditCase {
        const char* builder;
        std::map<std::string, scene::ParamValue> params;
        std::string edit_key;
        float edit_value;
        sdf::SDFBody before;
        sdf::SDFBody expected_after;
    };

    std::vector<EditCase> cases;
    cases.push_back({"sphere", {{"radius", 1.5f}}, "radius", 3.0f,
                     sdf::make_sphere(1.5f), sdf::make_sphere(3.0f)});
    cases.push_back({"box", {{"extent_x", 0.5f}, {"extent_y", 0.5f}, {"extent_z", 0.5f}},
                     "extent_y", 1.25f,
                     sdf::make_box(math::Vec3f{0.5f, 0.5f, 0.5f}),
                     sdf::make_box(math::Vec3f{0.5f, 1.25f, 0.5f})});
    cases.push_back({"cylinder", {{"radius", 0.5f}, {"height", 2.0f}},
                     "height", 4.0f,
                     sdf::make_cylinder(0.5f, 2.0f),
                     sdf::make_cylinder(0.5f, 4.0f)});
    cases.push_back({"torus", {{"major_radius", 1.0f}, {"minor_radius", 0.3f}},
                     "major_radius", 2.0f,
                     sdf::make_torus(1.0f, 0.3f),
                     sdf::make_torus(2.0f, 0.3f)});

    std::filesystem::path tmp_dir = std::filesystem::temp_directory_path();
    tmp_dir /= "cad0_test_native_edit_all";
    std::filesystem::create_directories(tmp_dir);

    for (const auto& c : cases) {
        CAPTURE(c.builder);

        scene::Scene s;
        {
            scene::SceneNode n;
            n.id = 1;
            n.name = c.builder;
            n.builder = c.builder;
            n.params = c.params;
            n.body = c.before;
            s.restore_node(std::move(n));
        }
        s.set_next_id(2);

        const std::filesystem::path path = tmp_dir / (std::string(c.builder) + ".CAD_0");
        native::save_file(path, s, native::NativeCamera{});

        scene::Scene loaded;
        native::install_document(loaded, native::load_file(path));
        loaded.update_meshes(8); // consume the post-load dirty flag
        scene::SceneNode* n = loaded.get_node_mut(1);
        REQUIRE(n != nullptr);
        REQUIRE(n->mesh_dirty == false);

        loaded.set_node_param(1, c.edit_key, c.edit_value);

        // Body rebuilt to exactly the expected primitive.
        const auto got = sample_grid(n->body, 7);
        const auto want = sample_grid(c.expected_after, 7);
        CHECK(got == want);
        CHECK(sample_grid(c.before, 7) != got);
        CHECK(n->mesh_dirty == true);
    }

    std::error_code ec;
    std::filesystem::remove_all(tmp_dir, ec);
}

TEST_CASE("post-load param edit: nodes without a builder tag stay non-parametric") {
    // A composite body (no builder tag) — set_param updates the stored
    // params only; the body and the tessellation state are untouched.
    scene::Scene s;
    {
        scene::SceneNode n;
        n.id = 2;
        n.name = "Composite";
        n.body = sdf::sdf_subtract(sdf::make_box(math::Vec3f{1.0f, 1.0f, 1.0f}),
                                   sdf::make_sphere(0.75f));
        s.restore_node(std::move(n));
    }
    s.set_next_id(3);

    std::filesystem::path tmp_dir = std::filesystem::temp_directory_path();
    tmp_dir /= "cad0_test_native_edit_composite";
    std::filesystem::create_directories(tmp_dir);
    const std::filesystem::path path = tmp_dir / "composite.CAD_0";
    native::save_file(path, s, native::NativeCamera{});

    scene::Scene loaded;
    native::install_document(loaded, native::load_file(path));
    loaded.update_meshes(8);
    scene::SceneNode* n = loaded.get_node_mut(2);
    REQUIRE(n != nullptr);
    REQUIRE(n->mesh_dirty == false);

    const auto before = sample_grid(n->body, 7);

    loaded.set_node_param(2, "radius", 3.0f); // no factory → params only

    CHECK(n->params.at("radius") == scene::ParamValue{3.0f});
    CHECK(sample_grid(n->body, 7) == before); // body untouched
    CHECK(n->mesh_dirty == false);            // no spurious re-tessellation

    std::error_code ec;
    std::filesystem::remove_all(tmp_dir, ec);
}

TEST_CASE("post-load param edit: unknown builder tag loads but is non-parametric") {
    // A file written by a newer CAD_0 with a builder this build does not
    // know: PATCH-compatible per ADR-0007 — the body still round-trips,
    // the node is simply not parametrically editable.
    const std::string doc = R"JSON({
      "format": "CAD_0 native",
      "version": "0.1.0",
      "scene": { "next_id": 2, "nodes": [ {
        "id": 1, "name": "FromTheFuture", "builder": "metaball",
        "params": { "radius": 1.5 },
        "body": { "op": "sphere", "radius": 1.5 }
      } ] }
    })JSON";

    scene::Scene loaded;
    native::install_document(loaded, native::parse_document(doc));
    loaded.update_meshes(8);
    scene::SceneNode* n = loaded.get_node_mut(1);
    REQUIRE(n != nullptr);
    REQUIRE(n->mesh_dirty == false);
    const auto before = sample_grid(n->body, 7);

    loaded.set_node_param(1, "radius", 3.0f);

    CHECK(sample_grid(n->body, 7) == before); // no factory, no rebuild
    CHECK(n->mesh_dirty == false);
    // Body + params still round-trip (re-serialization is stable).
    CHECK(serialized_radius(n->body) == 1.5f);
    CHECK(n->params.at("radius") == scene::ParamValue{3.0f});
}

// ---- depth limits (stack-overflow hardening) ---------------------------------
//
// Diagnostic: both the JSON parser (Parser::parse_value ->
// parse_object/parse_array) and the SDF deserializer
// (deserialize_node_impl) were recursive with NO depth guard. A hostile
// or corrupt .CAD_0 file with ~100k nested arrays recursed that deep and
// stack-overflowed (SIGSEGV) before ANY error handling could run. The
// tests below pin the contract:
//   * JSON nesting above kMaxNestingDepth (256) is an explicit
//     std::runtime_error naming the limit — never a crash;
//   * SDF trees above kMaxSDFNestingDepth (128) throw with a message
//     that names the maximum depth (legitimate trees are far shallower);
//   * legitimately deep trees (50 translate/scale levels) still load and
//     round-trip bit-identically — the guard must not break real files.

namespace {

// A chain of `n` translate wrappers around a sphere leaf, as JSON text.
std::string translate_chain(int n) {
    std::string body;
    for (int i = 0; i < n; ++i)
        body += "{\"op\":\"translate\",\"offset\":[0,0,0],\"child\":";
    body += "{\"op\":\"sphere\",\"radius\":1.0}";
    for (int i = 0; i < n; ++i)
        body += "}";
    return body;
}

// A full .CAD_0 document whose single node's body is `body_json`.
std::string document_with_body(const std::string& body_json) {
    return "{\"format\":\"CAD_0 native\",\"version\":\"0.1.0\",\"scene\":"
           "{\"next_id\":2,\"nodes\":[{\"id\":1,\"body\":" +
           body_json + "}]}}";
}

// "[" * n + "0" + "]" * n — n nested arrays around a number.
std::string nested_arrays(std::size_t n) {
    std::string s(n, '[');
    s += '0';
    s.append(n, ']');
    return s;
}

} // namespace

TEST_CASE("depth limit: 100k nested arrays is an explicit error, not a crash") {
    // The hostile-input case from the bug report. Before the guard this
    // crashed the process (stack overflow) — now it must throw.
    CHECK_THROWS_WITH_AS(
        json::JsonValue::parse(nested_arrays(100000)),
        doctest::Contains("too deeply nested (max 256"), std::runtime_error);
}

TEST_CASE("depth limit: JSON nesting boundary is exactly 256") {
    // 256 nested arrays parse fine (innermost value sits at depth 256)
    // and re-serialize — the writer's recursion is bounded by the same
    // tree the parser accepted.
    const auto v = json::JsonValue::parse(nested_arrays(256));
    CHECK(v.is_array());
    CHECK(!v.dump(false).empty());

    // 257 → explicit error naming the limit.
    CHECK_THROWS_WITH_AS(
        json::JsonValue::parse(nested_arrays(257)),
        doctest::Contains("too deeply nested (max 256"), std::runtime_error);

    // Objects count as nesting too (same guard must fire).
    std::string nested_obj;
    for (int i = 0; i < 257; ++i) nested_obj += "{\"a\":";
    nested_obj += '1';
    for (int i = 0; i < 257; ++i) nested_obj += '}';
    CHECK_THROWS_WITH_AS(
        json::JsonValue::parse(nested_obj),
        doctest::Contains("too deeply nested (max 256"), std::runtime_error);
}

TEST_CASE("depth limit: SDF tree beyond 128 levels is an explicit error") {
    // 200 translate wrappers: JSON depth ~204 (under the 256 JSON guard,
    // so the SDF-layer limit is what must fire) but 200 SDF levels.
    const std::string doc = document_with_body(translate_chain(200));
    CHECK_THROWS_WITH_AS(
        native::parse_document(doc),
        doctest::Contains("too deeply nested (max 128"), std::runtime_error);
}

TEST_CASE("depth limit: SDF boundary is exactly 128 levels") {
    // 128 wrappers + leaf (leaf at depth 128): loads.
    const auto doc = native::parse_document(document_with_body(translate_chain(128)));
    CHECK(doc.nodes.size() == 1);
    REQUIRE(doc.nodes.front().body.root() != nullptr);

    // 129 wrappers (leaf at depth 129): explicit error.
    CHECK_THROWS_WITH_AS(
        native::parse_document(document_with_body(translate_chain(129))),
        doctest::Contains("too deeply nested (max 128"), std::runtime_error);
}

TEST_CASE("depth limit: legitimately deep tree (50 translate/scale) loads and round-trips") {
    // Deeper than any real project but far inside the limits: the guard
    // must not break legitimate files. Alternating translate/scale, built
    // exactly the way the kernel API composes trees.
    sdf::SDFBody body = sdf::make_sphere(1.0f);
    for (int i = 0; i < 50; ++i) {
        const float fi = static_cast<float>(i);
        if (i % 2 == 0)
            body = sdf::sdf_translate(std::move(body),
                                      math::Vec3f{0.01f * fi, 0.0f, 0.0f});
        else
            body = sdf::sdf_scale(std::move(body), 1.0f + 0.001f * fi);
    }

    scene::Scene s;
    {
        scene::SceneNode n;
        n.id = 1;
        n.name = "deep";
        n.body = body;
        s.restore_node(std::move(n));
    }
    s.set_next_id(2);

    const std::string text = native::serialize_document(s, native::NativeCamera{});

    scene::Scene loaded;
    native::install_document(loaded, native::parse_document(text));
    REQUIRE(loaded.size() == 1);

    // Bit-identical SDF evaluation after the round-trip.
    CHECK(sample_grid(loaded.node_at(0)->body, 5) == sample_grid(s.node_at(0)->body, 5));

    // The loaded deep tree is still usable at the kernel level
    // (re-tessellation walks the whole 50-level tree).
    loaded.update_meshes(8);
    CHECK(loaded.get_node_mut(1)->mesh_dirty == false);
    CHECK(!loaded.get_node_mut(1)->cached_mesh.positions.empty());
}

} // TEST_SUITE

// ---- Mesh nodes in the native format (FASE 2) --------------------------------

TEST_CASE("mixed scene: save -> load preserves mesh, SDF values and ids") {
    scene::Scene s;
    s.restore_node([] {
        scene::SceneNode n;
        n.id = 7;
        n.name = "Primary sphere";
        n.builder = "sphere";
        n.params = {{"radius", 1.5f}};
        n.transform.translation = math::Vec3f{0.25f, -1.0f, 0.5f};
        n.transform.scale = 1.0f;
        n.body = sdf::make_sphere(1.5f);
        return n;
    }());
    {
        scene::SceneNode n;
        n.id = 11;
        n.name = "imported_octa";
        n.kind = scene::NodeKind::Mesh;
        n.mesh = make_golden_mesh();
        n.mesh_dirty = false;
        n.transform.translation = math::Vec3f{5.0f, 0.0f, 0.0f};
        s.restore_node(std::move(n));
    }
    s.set_next_id(12);

    const std::filesystem::path f =
        std::filesystem::temp_directory_path() / "cad0_fase2_mixed.CAD_0";
    native::save_file(f, s, make_canonical_camera());

    // ---- load back
    const native::ParsedDocument doc = native::load_file(f);
    REQUIRE(doc.nodes.size() == 2);
    REQUIRE(doc.meshes.size() == 1);
    CHECK(doc.meshes[0].id == "mesh11"); // "mesh" + node id

    const scene::SceneNode& m = doc.nodes[1];
    REQUIRE(m.kind == scene::NodeKind::Mesh);
    // Mesh identical: counts...
    CHECK(m.mesh.vertex_count() == 6);
    CHECK(m.mesh.triangle_count() == 8);
    // ...and exact geometry (positions, normals, indices).
    const sdf::TriangleMesh& want = make_golden_mesh();
    CHECK(m.mesh.positions == want.positions);
    CHECK(m.mesh.normals == want.normals);
    CHECK(m.mesh.indices == want.indices);
    // ...and a position checksum (sum of all components).
    double sum = 0.0;
    for (const auto& p : m.mesh.positions) sum += p.x + p.y + p.z;
    CHECK(sum == doctest::Approx(0.0)); // octahedron: symmetric

    // Bounds dispatch by kind (mesh positions).
    scene::Scene loaded;
    native::ParsedDocument doc_copy = doc;
    native::install_document(loaded, std::move(doc_copy));
    math::Bboxf mb;
    for (const auto& p : loaded.get_node(11)->mesh.positions) mb.expand(p);
    CHECK(mb.min.x == doctest::Approx(-1.0f));
    CHECK(mb.max.x == doctest::Approx(1.0f));
    // Scene bounds: the sphere (r=1.5) dominates the octahedron (r=1).
    CHECK(loaded.bounds().max.x == doctest::Approx(1.5f));

    // SDF nodes stay BIT-IDENTICAL.
    CHECK(sample_grid(loaded.get_node(7)->body, 7) ==
          sample_grid(s.get_node(7)->body, 7));
    CHECK(loaded.get_node(7)->builder == "sphere");
    CHECK(loaded.get_node(7)->params.at("radius") == scene::ParamValue{1.5f});

    // Ids round-trip exactly.
    CHECK(loaded.get_node(7)->id == 7);
    CHECK(loaded.get_node(11)->id == 11);
    CHECK(loaded.next_id() == 12);

    // update_meshes never re-tessellates the mesh node.
    loaded.update_meshes(48);
    CHECK(loaded.get_node(11)->mesh.positions == want.positions);
    CHECK(loaded.get_node(11)->mesh_dirty == false);

    // Re-save: byte-identical (stable blob ids from node ids).
    const std::filesystem::path f2 =
        std::filesystem::temp_directory_path() / "cad0_fase2_mixed2.CAD_0";
    native::save_file(f2, loaded, doc.camera);
    CHECK(read_file(f2) == read_file(f));

    std::filesystem::remove(f);
    std::filesystem::remove(f2);
}

TEST_CASE("mesh nodes: transactional load on corrupted blob section") {
    // Build a valid mixed-scene file, then corrupt the blob section
    // (declare a mesh header whose payload would decode garbage — the
    // index guard fires) and check the scene stays untouched.
    scene::Scene s;
    {
        scene::SceneNode n;
        n.id = 3;
        n.name = "octa";
        n.kind = scene::NodeKind::Mesh;
        n.mesh = make_golden_mesh();
        n.mesh_dirty = false;
        s.restore_node(std::move(n));
    }
    const std::filesystem::path f =
        std::filesystem::temp_directory_path() / "cad0_fase2_corrupt.CAD_0";
    native::save_file(f, s, {});

    // Corrupt: flip bytes inside the blob section (the last 12 bytes are
    // one triangle's indices -> out-of-range after the flip).
    std::string bytes = read_file(f);
    for (std::size_t i = bytes.size() - 12; i < bytes.size(); ++i)
        bytes[i] = static_cast<char>(0x7F);
    const std::filesystem::path bad =
        std::filesystem::temp_directory_path() / "cad0_fase2_bad.CAD_0";
    {
        std::ofstream out(bad, std::ios::binary | std::ios::trunc);
        out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    }

    scene::Scene target;
    target.add_node("keepme", sdf::make_sphere(1.0f));
    const std::size_t before = target.size();
    CHECK_THROWS_WITH_AS(native::load_file(bad),
        doctest::Contains("out of range"), std::runtime_error);
    CHECK(target.size() == before); // transactional: untouched

    // Truncated blob section (chop the whole file mid-blob): the frame
    // length check fires first, still explicit, still untouched.
    std::string chopped = read_file(f).substr(0, read_file(f).size() - 40);
    const std::filesystem::path trunc =
        std::filesystem::temp_directory_path() / "cad0_fase2_trunc.CAD_0";
    {
        std::ofstream out(trunc, std::ios::binary | std::ios::trunc);
        out.write(chopped.data(), static_cast<std::streamsize>(chopped.size()));
    }
    CHECK_THROWS_WITH_AS(native::load_file(trunc),
        doctest::Contains("truncated or corrupt"), std::runtime_error);
    CHECK(target.size() == before);

    // A mesh node referencing a blob that is not declared -> explicit
    // error naming the missing reference (hand-crafted document).
    const std::string dangling = R"JSON({
      "format": "CAD_0 native", "version": "0.2.0",
      "scene": { "next_id": 4, "nodes": [ { "id": 3, "kind": "mesh",
        "mesh": { "blob": "mesh999" } } ] },
      "meshes": [ { "id": "mesh3", "encoding": "raw", "offset": 0,
                    "size": 0, "vertex_count": 0, "triangle_count": 0 } ]
    })JSON";
    std::string blob_section(1, '\0');
    CHECK_THROWS_WITH_AS(native::parse_document(dangling, blob_section),
        doctest::Contains("references mesh blob \"mesh999\""), std::runtime_error);

    std::filesystem::remove(f);
    std::filesystem::remove(bad);
    std::filesystem::remove(trunc);
}

TEST_CASE("mesh nodes: kind dispatch is strict (unknown kinds, SDF leftovers)") {
    // Unknown kind.
    const std::string unknown = R"JSON({
      "format": "CAD_0 native", "version": "0.2.0",
      "scene": { "next_id": 2, "nodes": [ { "id": 1, "kind": "voxel" } ] }
    })JSON";
    CHECK_THROWS_WITH_AS(native::parse_document(unknown, {}),
        doctest::Contains("unknown node \"kind\" \"voxel\""), std::runtime_error);

    // A "mesh" node carrying an SDF-only field: surfaced, never ignored.
    const std::string mixed = R"JSON({
      "format": "CAD_0 native", "version": "0.2.0",
      "scene": { "next_id": 2, "nodes": [ { "id": 1, "kind": "mesh",
        "mesh": { "blob": "mesh1" },
        "body": { "op": "sphere", "radius": 1.0 } } ] },
      "meshes": [ { "id": "mesh1", "encoding": "raw", "offset": 0,
                    "size": 0, "vertex_count": 0, "triangle_count": 0 } ]
    })JSON";
    CHECK_THROWS_WITH_AS(native::parse_document(mixed, ""),
        doctest::Contains("SDF-only field \"body\""), std::runtime_error);
}

// ===========================================================================
// Property-based family (mini-traccia PARTE 2, area 4 — native format).
//
// Generated scenes — parametric primitives through the builder registry,
// Subtract operation recipes (chained allowed), and DELIBERATE dangling
// references (1 in 4 recipes points at an id no node owns; ADR-0027 D2
// loads it as the visible error state) — must round-trip:
//   * byte-identical: save -> load -> save reproduces the same bytes;
//   * report-identical: the extraction summary (version, node count,
//     next_id, and every node's id/name/kind/visibility/builder/params/
//     transform/recipe/operand_missing) is the same before and after.
//
// Declared generator ranges: sphere radius [0.3, 1.5]; box extent per
// axis [0.2, 1.0]; cylinder radius [0.15, 0.5] and height [0.8, 3.0];
// translations in [-1.5, 1.5] per axis; recipe offsets in [-0.3, 0.3];
// camera target [-2, 2], distance [4, 12], yaw [-3, 3], pitch
// [-1.2, 1.2], fov [35, 70]. Outside these = generator bug.
// ===========================================================================

#include "property/property_prng.hpp"
#include "CAD_0/scene/builders.hpp"
#include "CAD_0/scene/operations.hpp"

#include <map>
#include <sstream>

namespace {

namespace prop = CAD_0::test::property;

// The extraction report: everything the loader recovers, rendered as a
// deterministic string. Identical strings mean identical extractions.
std::string extraction_summary(const native::ParsedDocument& doc) {
    std::ostringstream os;
    os << "version=" << doc.version.major << "." << doc.version.minor
       << "." << doc.version.patch << " nodes=" << doc.nodes.size()
       << " meshes=" << doc.meshes.size() << " next_id=" << doc.next_id;
    for (const auto& n : doc.nodes) {
        os << " | node#" << n.id << " name=" << n.name
           << " kind=" << (n.kind == scene::NodeKind::Mesh ? "mesh" : "sdf")
           << " visible=" << (n.visible ? 1 : 0)
           << " builder=" << n.builder;
        for (const auto& [k, v] : n.params) {
            os << " p:" << k << "=";
            if (const auto* f = std::get_if<float>(&v)) os << *f;
            else if (const auto* sv = std::get_if<std::string>(&v)) os << *sv;
            else if (const auto* bv = std::get_if<bool>(&v)) os << *bv;
        }
        os << " t=(" << n.transform.translation.x << ","
           << n.transform.translation.y << ","
           << n.transform.translation.z << ")"
           << " s=" << n.transform.scale;
        if (n.operation) {
            os << " op{target=" << n.operation->target
               << " tool=" << n.operation->tool
               << " off=(" << n.operation->offset_x << ","
               << n.operation->offset_y << "," << n.operation->offset_z << ")}";
        }
        os << " missing=" << (n.operand_missing ? 1 : 0);
    }
    return os.str();
}

} // namespace

TEST_CASE("property: generated scenes round-trip byte-identically through the native format") {
    const std::uint32_t seed = 0xF0CA0101u;
    prop::Prng prng(seed);
    const std::filesystem::path path1 =
        std::filesystem::temp_directory_path() / "cad0_property_fmt_a.CAD_0";
    const std::filesystem::path path2 =
        std::filesystem::temp_directory_path() / "cad0_property_fmt_b.CAD_0";

    for (int iter = 0; iter < 25; ++iter) {
        scene::Scene s;
        native::NativeCamera cam;
        cam.target = math::Vec3f{prng.frange(-2.0f, 2.0f),
                                 prng.frange(-2.0f, 2.0f),
                                 prng.frange(-2.0f, 2.0f)};
        cam.distance = prng.frange(4.0f, 12.0f);
        cam.yaw = prng.frange(-3.0f, 3.0f);
        cam.pitch = prng.frange(-1.2f, 1.2f);
        cam.fov_deg = prng.frange(35.0f, 70.0f);

        // 1..4 parametric primitives through the builder registry.
        std::vector<std::uint32_t> ids;
        const int n_prim = 1 + static_cast<int>(prng.int_below(4u));
        for (int k = 0; k < n_prim; ++k) {
            const char* tag = nullptr;
            switch (prng.int_below(3u)) {
            case 0: tag = "sphere"; break;
            case 1: tag = "box"; break;
            default: tag = "cylinder"; break;
            }
            const scene::BuilderSpec* spec = scene::find_builder(tag);
            REQUIRE(spec != nullptr);
            auto params = spec->default_params();
            if (tag == std::string("sphere")) {
                params["radius"] = prng.frange(0.3f, 1.5f);
            } else if (tag == std::string("box")) {
                params["extent_x"] = prng.frange(0.2f, 1.0f);
                params["extent_y"] = prng.frange(0.2f, 1.0f);
                params["extent_z"] = prng.frange(0.2f, 1.0f);
            } else {
                params["radius"] = prng.frange(0.15f, 0.5f);
                params["height"] = prng.frange(0.8f, 3.0f);
            }
            const std::uint32_t id = s.add_node(
                std::string(tag) + std::to_string(k),
                spec->make(params), spec->factory(), params);
            ids.push_back(id);
            if (auto* n = s.get_node_mut(id)) {
                n->transform.translation = math::Vec3f{
                    prng.frange(-1.5f, 1.5f),
                    prng.frange(-1.5f, 1.5f),
                    prng.frange(-1.5f, 1.5f)};
                n->transform.scale = prng.frange(0.5f, 2.0f);
                n->visible = prng.coin();
            }
        }

        // 0..2 Subtract recipes. Dangling references are produced the
        // SUPPORTED way (ADR-0027 D2: "a later deletion produces the
        // visible dangling state, an absent operand at creation is a
        // caller bug"): every 4th recipe is created from live operands
        // and then its tool node is DELETED.
        const int n_op = static_cast<int>(prng.int_below(3u));
        for (int k = 0; k < n_op; ++k) {
            scene::OperationRecipe r;
            r.type = scene::OperationType::Subtract;
            r.target = ids[prng.int_below(
                static_cast<std::uint32_t>(ids.size()))];
            r.tool = ids[prng.int_below(
                static_cast<std::uint32_t>(ids.size()))];
            const bool make_dangling = (prng.int_below(4u) == 0u) && ids.size() >= 2;
            if (make_dangling && r.target == r.tool) {
                // Pick a different tool so the deletion leaves exactly
                // one dangling operand (target stays alive).
                for (std::uint32_t cand : ids) {
                    if (cand != r.target) { r.tool = cand; break; }
                }
            }
            r.offset_x = prng.frange(-0.3f, 0.3f);
            r.offset_y = prng.frange(-0.3f, 0.3f);
            r.offset_z = prng.frange(-0.3f, 0.3f);
            s.add_operation_node("Op" + std::to_string(k), r);
            if (make_dangling && r.target != r.tool) {
                REQUIRE(s.remove_node(r.tool));
                // Later recipes must pick among LIVE operands only
                // (an operand deleted by an earlier recipe is absent
                // at creation — the caller bug the scene layer
                // rejects; keep `ids` in sync with reality).
                std::erase(ids, r.tool);
            }
        }

        // save -> load -> save: bytes must match exactly.
        native::save_file(path1, s, cam);
        auto doc1 = native::load_file(path1);
        const native::NativeCamera cam1 = doc1.camera;
        const std::string sum1 = extraction_summary(doc1);
        scene::Scene s2;
        native::install_document(s2, std::move(doc1));
        native::save_file(path2, s2, cam1);
        const std::string bytes1 = read_file(path1);
        const std::string bytes2 = read_file(path2);
        auto doc2 = native::load_file(path2);
        const std::string sum2 = extraction_summary(doc2);

        INFO("seed=", seed, " iter=", iter, " n_prim=", n_prim, " n_op=", n_op);
        INFO("report #1: ", sum1);
        INFO("report #2: ", sum2);
        INFO("check: save -> load -> save is byte-identical");
        CHECK(bytes1 == bytes2);
        INFO("check: extraction report identical before/after the round-trip");
        CHECK(sum1 == sum2);
    }
    std::filesystem::remove(path1);
    std::filesystem::remove(path2);
}
