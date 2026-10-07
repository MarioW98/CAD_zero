// core/viewport/include/CAD_0/viewport/renderer.hpp
//
// Abstract renderer interface for the CAD_0 viewport.
//
// The Renderer interface decouples the viewport's rendering from the
// concrete graphics API (OpenGL 4.5 baseline, Vulkan optional). Two
// implementations are planned:
//
//   * HeadlessRenderer  — used for tests; renders to a CPU-side framebuffer.
//   * OpenGLRenderer    — Phase C.6, renders to a Qt OpenGL widget.
//   * VulkanRenderer    — post-MVP.
//
// The interface is intentionally minimal: the viewport calls `begin_frame`,
// `draw_*`, `end_frame`. The renderer owns the GPU resources (VAOs, VBOs,
// shader programs) and translates the draw calls into the underlying API.
//
// All methods are synchronous; the viewport is single-threaded.
//
#pragma once

#include "CAD_0/math/vec.hpp"
#include "CAD_0/math/mat.hpp"
#include "CAD_0/math/bbox.hpp"
#include "CAD_0/viewport/camera.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace CAD_0::viewport {

// A renderable triangle mesh, in the format produced by core/sdf.
// We don't depend on core/sdf here to avoid a layering cycle — the
// viewport takes raw arrays.
struct RenderMesh {
    std::vector<math::Vec3f> positions;
    std::vector<math::Vec3f> normals;
    std::vector<std::uint32_t> indices;
};

// Render options passed per-frame.
struct RenderOptions {
    bool show_grid = true;
    bool show_axes = true;
    bool wireframe = false;
    bool backface_culling = true;
    math::Vec3f background_color{0.15f, 0.15f, 0.18f};
    math::Vec3f mesh_color{0.7f, 0.75f, 0.8f};
};

// Statistics returned by `end_frame`. Useful for profiling and for the
// status bar of the viewport.
struct FrameStats {
    std::uint64_t draw_calls{0};
    std::uint64_t triangles{0};
    std::uint64_t vertices{0};
    double frame_time_ms{0.0};
};

// Abstract renderer interface.
class Renderer {
public:
    virtual ~Renderer() = default;

    // Initialize the renderer with a viewport of given dimensions.
    // Returns false if initialization failed (e.g. no GL context).
    virtual bool initialize(int width, int height) = 0;

    // Resize the framebuffer. Called when the viewport widget is resized.
    virtual void resize(int width, int height) = 0;

    // ----- Frame lifecycle ------------------------------------------------

    // Begin a new frame. Clears the framebuffer and sets up the camera.
    virtual void begin_frame(const OrbitCamera& camera,
                             const RenderOptions& opts) = 0;

    // Draw a triangle mesh with the current camera.
    virtual void draw_mesh(const RenderMesh& mesh,
                           const math::Mat4f& model_matrix) = 0;

    // Draw the world grid (XZ plane, major lines every 10 units).
    virtual void draw_grid() = 0;

    // Draw the world axes (RGB for XYZ).
    virtual void draw_axes() = 0;

    // End the frame and present the framebuffer (if applicable).
    // Returns statistics for the frame.
    virtual FrameStats end_frame() = 0;

    // ----- Resource management ------------------------------------------

    // For Phase C.4 we don't have GPU resource handles — meshes are
    // uploaded per-frame via `draw_mesh`. Phase C.6 will add VBO/IBO
    // caching keyed on a mesh id returned by `upload_mesh(mesh)`.

    // Return a human-readable name (e.g. "opengl-4.5", "headless").
    virtual const char* name() const noexcept = 0;
};

// Headless renderer — writes to a CPU-side framebuffer. Used in tests
// to verify that the rendering pipeline produces expected output without
// requiring a real OpenGL context.
class HeadlessRenderer final : public Renderer {
public:
    HeadlessRenderer() = default;
    ~HeadlessRenderer() override = default;

    bool initialize(int width, int height) override;
    void resize(int width, int height) override;

    void begin_frame(const OrbitCamera& camera,
                     const RenderOptions& opts) override;
    void draw_mesh(const RenderMesh& mesh,
                   const math::Mat4f& model_matrix) override;
    void draw_grid() override;
    void draw_axes() override;
    FrameStats end_frame() override;

    const char* name() const noexcept override { return "headless"; }

    // ----- Headless-specific inspection --------------------------------

    int width() const noexcept { return width_; }
    int height() const noexcept { return height_; }

    // Read the framebuffer as RGB bytes (width * height * 3).
    // Useful for snapshot tests — write to a PNG for inspection.
    const std::vector<std::uint8_t>& framebuffer() const noexcept {
        return framebuffer_;
    }

    // Number of meshes drawn in the current frame (before end_frame).
    std::size_t mesh_count() const noexcept { return mesh_count_; }

private:
    int width_{0};
    int height_{0};
    std::vector<std::uint8_t> framebuffer_;  // RGB
    OrbitCamera camera_{};
    RenderOptions opts_{};
    bool frame_active_{false};
    std::size_t mesh_count_{0};
    FrameStats last_stats_{};
};

} // namespace CAD_0::viewport
