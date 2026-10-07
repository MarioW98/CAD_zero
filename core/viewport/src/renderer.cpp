// core/viewport/src/renderer.cpp
//
// HeadlessRenderer implementation.
//
// The headless renderer doesn't actually rasterize — it just records draw
// calls and produces a flat-colored framebuffer. Its purpose is to let
// the rendering pipeline be tested in CI without an OpenGL context.
//
// Phase C.6 will add the real OpenGL renderer.
//
#include "CAD_0/viewport/renderer.hpp"

#include <algorithm>
#include <cstring>
#include <chrono>

namespace CAD_0::viewport {

bool HeadlessRenderer::initialize(int width, int height) {
    if (width <= 0 || height <= 0) return false;
    width_ = width;
    height_ = height;
    framebuffer_.assign(static_cast<std::size_t>(width) * height * 3, 0);
    return true;
}

void HeadlessRenderer::resize(int width, int height) {
    if (width <= 0 || height <= 0) return;
    if (width == width_ && height == height_) return;
    width_ = width;
    height_ = height;
    framebuffer_.assign(static_cast<std::size_t>(width) * height * 3, 0);
}

void HeadlessRenderer::begin_frame(const OrbitCamera& camera,
                                    const RenderOptions& opts) {
    camera_ = camera;
    opts_ = opts;
    frame_active_ = true;
    mesh_count_ = 0;
    last_stats_ = FrameStats{};

    // Clear the framebuffer to the background color.
    const auto& bg = opts.background_color;
    const std::uint8_t r = static_cast<std::uint8_t>(std::clamp(bg.x, 0.0f, 1.0f) * 255);
    const std::uint8_t g = static_cast<std::uint8_t>(std::clamp(bg.y, 0.0f, 1.0f) * 255);
    const std::uint8_t b = static_cast<std::uint8_t>(std::clamp(bg.z, 0.0f, 1.0f) * 255);
    for (std::size_t i = 0; i + 2 < framebuffer_.size(); i += 3) {
        framebuffer_[i + 0] = r;
        framebuffer_[i + 1] = g;
        framebuffer_[i + 2] = b;
    }
}

void HeadlessRenderer::draw_mesh(const RenderMesh& mesh,
                                  const math::Mat4f& model_matrix) {
    (void)model_matrix;  // not used in headless mode
    if (!frame_active_) return;
    mesh_count_++;
    last_stats_.draw_calls++;
    last_stats_.triangles += mesh.indices.size() / 3;
    last_stats_.vertices += mesh.positions.size();
}

void HeadlessRenderer::draw_grid() {
    if (!frame_active_) return;
    last_stats_.draw_calls++;
    // Grid is drawn as ~40 line segments (20 per axis) → 80 triangles
    // for a wireframe representation. We count them as 1 draw call.
    last_stats_.triangles += 80;
}

void HeadlessRenderer::draw_axes() {
    if (!frame_active_) return;
    last_stats_.draw_calls++;
    // 3 axes × 2 triangles each → 6 triangles.
    last_stats_.triangles += 6;
}

FrameStats HeadlessRenderer::end_frame() {
    frame_active_ = false;
    last_stats_.frame_time_ms = 0.0;  // headless: no real timing
    return last_stats_;
}

} // namespace CAD_0::viewport
