// core/viewport/include/CAD_0/viewport/opengl_renderer.hpp
//
// OpenGL 4.5 implementation of the Renderer interface.
//
// This file is compiled only when CAD_0_BUILD_OPENGL=ON. It depends
// on GLAD (or system GL headers) and is intended to be hosted inside a
// Qt OpenGL context (PySide6 QWindow in Phase C.6).
//
// Phase A-C status: written but not built in CI. The HeadlessRenderer
// (see renderer.hpp) is the tested baseline; this file is the production
// counterpart that will be exercised manually once PySide6 lands.
//
#pragma once

#include "CAD_0/viewport/renderer.hpp"

namespace CAD_0::viewport {

// OpenGL 4.5 renderer. Requires a current GL context (created by the
// PySide6 QWindow before initialize() is called).
class OpenGLRenderer final : public Renderer {
public:
    OpenGLRenderer() = default;
    ~OpenGLRenderer() override;

    OpenGLRenderer(const OpenGLRenderer&) = delete;
    OpenGLRenderer& operator=(const OpenGLRenderer&) = delete;

    bool initialize(int width, int height) override;
    void resize(int width, int height) override;

    void begin_frame(const OrbitCamera& camera,
                     const RenderOptions& opts) override;
    void draw_mesh(const RenderMesh& mesh,
                   const math::Mat4f& model_matrix) override;
    void draw_grid() override;
    void draw_axes() override;
    FrameStats end_frame() override;

    const char* name() const noexcept override { return "opengl-4.5"; }

private:
    bool initialized_{false};
    int width_{0};
    int height_{0};

    // VAO for the full-screen triangle used by the SDF ray-march shader.
    unsigned int empty_vao_{0};

    // Shader program for SDF ray-marching.
    unsigned int raymarch_program_{0};

    // Uniform buffer objects (UBOs) for camera + options.
    unsigned int camera_ubo_{0};
    unsigned int options_ubo_{0};

    // Simple shader for tessellated meshes (B-Rep + mesh extraction output).
    unsigned int mesh_program_{0};

    FrameStats last_stats_{};
};

} // namespace CAD_0::viewport
