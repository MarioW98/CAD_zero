// tests/unit/viewport/test_renderer.cpp
//
// Tests for the HeadlessRenderer class.
//
// The headless renderer doesn't produce pixels; it records draw calls
// and produces a flat-colored framebuffer. These tests verify that the
// rendering pipeline (begin/draw/end) works correctly without requiring
// an OpenGL context.
//
#include <doctest/doctest.h>

#include "CAD_0/viewport/renderer.hpp"
#include "CAD_0/viewport/camera.hpp"
#include "CAD_0/math/vec.hpp"

#include <vector>

using namespace CAD_0::viewport;
using namespace CAD_0::math;

namespace {

RenderMesh make_test_mesh() {
    RenderMesh m;
    m.positions = {
        {0, 0, 0}, {1, 0, 0}, {0, 1, 0},
    };
    m.normals = {
        {0, 0, 1}, {0, 0, 1}, {0, 0, 1},
    };
    m.indices = {0, 1, 2};
    return m;
}

} // namespace

TEST_CASE("HeadlessRenderer: initialize sets dimensions") {
    HeadlessRenderer r;
    REQUIRE(r.initialize(640, 480));
    CHECK(r.width() == 640);
    CHECK(r.height() == 480);
    CHECK(r.framebuffer().size() == 640u * 480u * 3);
}

TEST_CASE("HeadlessRenderer: initialize rejects zero dimensions") {
    HeadlessRenderer r;
    CHECK_FALSE(r.initialize(0, 0));
    CHECK_FALSE(r.initialize(640, 0));
    CHECK_FALSE(r.initialize(0, 480));
    CHECK_FALSE(r.initialize(-1, -1));
}

TEST_CASE("HeadlessRenderer: resize changes framebuffer size") {
    HeadlessRenderer r;
    REQUIRE(r.initialize(640, 480));
    r.resize(800, 600);
    CHECK(r.width() == 800);
    CHECK(r.height() == 600);
    CHECK(r.framebuffer().size() == 800u * 600u * 3);
}

TEST_CASE("HeadlessRenderer: resize is a no-op for same dimensions") {
    HeadlessRenderer r;
    REQUIRE(r.initialize(640, 480));
    r.resize(640, 480);
    CHECK(r.width() == 640);
    CHECK(r.height() == 480);
}

TEST_CASE("HeadlessRenderer: begin_frame clears to background color") {
    HeadlessRenderer r;
    REQUIRE(r.initialize(10, 10));

    RenderOptions opts;
    opts.background_color = {1.0f, 0.0f, 0.0f};  // red
    OrbitCamera cam;
    r.begin_frame(cam, opts);

    const auto& fb = r.framebuffer();
    REQUIRE(fb.size() >= 3);
    CHECK(fb[0] == 255);  // R
    CHECK(fb[1] == 0);    // G
    CHECK(fb[2] == 0);    // B
}

TEST_CASE("HeadlessRenderer: draw_mesh records draw call") {
    HeadlessRenderer r;
    REQUIRE(r.initialize(10, 10));

    OrbitCamera cam;
    RenderOptions opts;
    r.begin_frame(cam, opts);

    auto mesh = make_test_mesh();
    r.draw_mesh(mesh, Mat4f::identity());
    CHECK(r.mesh_count() == 1);
}

TEST_CASE("HeadlessRenderer: end_frame returns stats") {
    HeadlessRenderer r;
    REQUIRE(r.initialize(10, 10));

    OrbitCamera cam;
    RenderOptions opts;
    r.begin_frame(cam, opts);

    auto mesh = make_test_mesh();
    r.draw_mesh(mesh, Mat4f::identity());
    r.draw_mesh(mesh, Mat4f::identity());

    const auto stats = r.end_frame();
    CHECK(stats.draw_calls == 2);
    CHECK(stats.triangles == 2);  // each mesh has 1 triangle
    CHECK(stats.vertices == 6);   // each mesh has 3 vertices
}

TEST_CASE("HeadlessRenderer: draw_grid and draw_axes count separately") {
    HeadlessRenderer r;
    REQUIRE(r.initialize(10, 10));

    OrbitCamera cam;
    RenderOptions opts;
    r.begin_frame(cam, opts);
    r.draw_grid();
    r.draw_axes();
    const auto stats = r.end_frame();
    CHECK(stats.draw_calls == 2);
    CHECK(stats.triangles >= 80 + 6);
}

TEST_CASE("HeadlessRenderer: draw calls outside begin_frame are ignored") {
    HeadlessRenderer r;
    REQUIRE(r.initialize(10, 10));

    auto mesh = make_test_mesh();
    r.draw_mesh(mesh, Mat4f::identity());  // before begin_frame — no-op
    CHECK(r.mesh_count() == 0);
}

TEST_CASE("HeadlessRenderer: name returns 'headless'") {
    HeadlessRenderer r;
    CHECK(std::string(r.name()) == "headless");
}

TEST_CASE("HeadlessRenderer: multiple frames reset stats") {
    HeadlessRenderer r;
    REQUIRE(r.initialize(10, 10));

    OrbitCamera cam;
    RenderOptions opts;

    // First frame: 2 meshes.
    r.begin_frame(cam, opts);
    auto mesh = make_test_mesh();
    r.draw_mesh(mesh, Mat4f::identity());
    r.draw_mesh(mesh, Mat4f::identity());
    auto stats1 = r.end_frame();
    CHECK(stats1.draw_calls == 2);

    // Second frame: 1 mesh.
    r.begin_frame(cam, opts);
    r.draw_mesh(mesh, Mat4f::identity());
    auto stats2 = r.end_frame();
    CHECK(stats2.draw_calls == 1);
    CHECK(stats2.triangles == 1);
}
