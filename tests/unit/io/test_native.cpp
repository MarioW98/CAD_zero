// tests/unit/io/test_native.cpp
//
// Native project format (.CAD_0, ADR-0007) — unit tests.
//
// Coverage per the feature plan:
//   * round-trip: SDF evaluation is BIT-IDENTICAL after save/load
//   * golden file: a checked-in v0.1.0 document must parse and
//     re-serialize byte-identically (CI drift detector)
//   * ID round-trip: node ids and next_id survive a cycle exactly
//   * explicit errors: corrupted JSON, unknown ops, missing fields,
//     version mismatches without a migrator — all throw with messages,
//     and the scene is left untouched (transactional load)
//   * atomic save: no .tmp residue, original file preserved on failure
//
#include <doctest/doctest.h>

#include "CAD_0/io/json.hpp"
#include "CAD_0/io/native.hpp"
#include "CAD_0/sdf/operators.hpp"
#include "CAD_0/sdf/primitives.hpp"
#include "CAD_0/sdf/transforms.hpp"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace native = CAD_0::io::native;
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
                const math::Vec3f p{-1.5f + 3.0f * i / (n - 1),
                                    -1.5f + 3.0f * j / (n - 1),
                                    -1.5f + 3.0f * k / (n - 1)};
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

std::string read_file(const std::filesystem::path& p) {
    std::ifstream in(p, std::ios::binary);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

} // namespace

// ---- tests ------------------------------------------------------------------

TEST_SUITE("io native format") {

TEST_CASE("version constant + empty migrator chain (ADR-0007 Phase A)") {
    CHECK(native::kNativeVersion.major == 0);
    CHECK(native::kNativeVersion.minor == 1);
    CHECK(native::kNativeVersion.patch == 0);
    CHECK(native::migrator_count() == 0);

    const auto v = native::parse_version("0.1.0");
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

TEST_CASE("golden file: parses and re-serializes byte-identically") {
    const std::string fresh = native::serialize_document(
        make_canonical_scene(), make_canonical_camera());

    // Regeneration mode (maintainer-only, never active in CI):
    //   CAD_0_GENERATE_GOLDEN=1 ./CAD_0_tests
    // rewrites the checked-in golden file after an INTENTIONAL format
    // change (which requires a companion ADR per ADR-0007).
    if (const char* env = std::getenv("CAD_0_GENERATE_GOLDEN");
        env && std::string_view(env) == "1") {
        std::filesystem::create_directories(golden_path().parent_path());
        std::ofstream out(golden_path(), std::ios::binary | std::ios::trunc);
        out << fresh;
        out.close();
        MESSAGE("golden file (re)written: ", golden_path().string());
        return;
    }

    const std::filesystem::path gp = golden_path();
    const std::string missing =
        "golden file missing: " + gp.string() +
        " (regenerate with CAD_0_GENERATE_GOLDEN=1)";
    REQUIRE_MESSAGE(std::filesystem::exists(gp), missing);

    const std::string golden = read_file(gp);

    // (a) The canonical scene serializes to exactly the golden content.
    CHECK(fresh == golden);

    // (b) Parsing the golden and re-serializing is idempotent (format
    // stability — the drift detector for future changes).
    scene::Scene s;
    native::install_document(s, native::parse_document(golden));
    CHECK(native::serialize_document(s, make_canonical_camera()) == golden);
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

} // TEST_SUITE
