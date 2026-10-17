// tests/unit/io/test_native.cpp
//
// Native project format (.CAD_0, ADR-0007 + ADR-0007a + ADR-0027 D4) —
// unit tests.
//
// Coverage per the feature plan:
//   * round-trip: SDF evaluation is BIT-IDENTICAL after save/load
//   * golden v0.1.0 (frozen legacy fixture): loads through the
//     migrator CHAIN (0.1.0 → 0.2.0 → 0.3.0); SDF values bit-identical
//   * golden v0.2.0: loads through the 0.2.0 → 0.3.0 migrator; the
//     migrated re-serialization is byte-identical to a fresh v0.3 save
//   * golden v0.3.0: the pierced-box document with an operation recipe;
//     extraction report χ = 0 / genus 1 (ADR-0027's MVP shape)
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

TEST_CASE("version constant + migrator chain (ADR-0007/0007a/0027)") {
    CHECK(native::kNativeVersion.major == 0);
    CHECK(native::kNativeVersion.minor == 3);
    CHECK(native::kNativeVersion.patch == 0);
    // The registered chain: 0.1.0 -> 0.2.0 (ADR-0007a, no-op) and
    // 0.2.0 -> 0.3.0 (ADR-0027 D4, content no-op).
    CHECK(native::migrator_count() == 2);

    const auto v = native::parse_version("0.3.0");
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

    // (a) The fresh v0.3.0 save of the canonical scene + meshes (the
    // reference bytes for (c)).
    native::save_file(tmp, make_canonical_scene(),
                      make_canonical_camera(), make_golden_meshes());
    const std::string fresh_v3 = read_file(tmp);
    CHECK(fresh_v3.find("\"version\": \"0.3.0\"") != std::string::npos);
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
    // to the fresh v0.3.0 save — the migrator loses nothing, and the
    // drift detector (blob offsets, JSON determinism, endian-stable
    // encoding) keeps working across the bump.
    native::save_file(tmp, s, doc.camera, doc.meshes);
    CHECK(read_file(tmp) == fresh_v3);
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

TEST_CASE("golden v0.3.0: pierced box with recipe - extraction chi=0 genus=1") {
    // Regeneration mode (maintainer-only, never active in CI):
    //   CAD_0_GENERATE_GOLDEN=1 ./CAD_0_tests
    // rewrites the v0.3.0 golden after an INTENTIONAL format change
    // (companion ADR required per ADR-0007).
    if (const char* env = std::getenv("CAD_0_GENERATE_GOLDEN");
        env && std::string_view(env) == "1") {
        std::filesystem::create_directories(golden_v3_path().parent_path());
        native::save_file(golden_v3_path(), make_pierced_scene(),
                          make_canonical_camera());
        MESSAGE("golden file (re)written: ", golden_v3_path().string());
        return;
    }

    const std::filesystem::path gp = golden_v3_path();
    const std::string missing = "golden file missing: " + gp.string();
    REQUIRE_MESSAGE(std::filesystem::exists(gp), missing);
    const std::string golden = read_file(gp);
    CHECK(golden.find("\"version\": \"0.3.0\"") != std::string::npos);

    // (a) The fresh save of the pierced-box scene IS the golden bytes.
    const std::filesystem::path tmp =
        std::filesystem::temp_directory_path() / "cad0_golden_v3_check.CAD_0";
    native::save_file(tmp, make_pierced_scene(), make_canonical_camera());
    CHECK(read_file(tmp) == golden);
    std::filesystem::remove(tmp);

    // (b) The golden carries the recipe in the ADR-0027 D4 shape.
    CHECK(golden.find("\"operation\"") != std::string::npos);
    CHECK(golden.find("\"type\": \"subtract\"") != std::string::npos);

    // (c) Load: the operation node is intact and NOT dangling; the
    // extraction report on its body is the ADR-0027 MVP contract —
    // the first genus > 0 document in the repository's history:
    // χ = 0, genus 1, closed-orientable.
    const native::ParsedDocument doc = native::load_file(gp);
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
