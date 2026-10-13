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

// [[maybe_unused]] on non-static data members is standard C++17, but
// GCC 11 — the ubuntu-22.04 CI compiler, see
// docs/audits/2026-10-13-warning-audit.md ("Toolchain contract") — does
// not implement it in that position and warns "'maybe_unused' attribute
// ignored" [-Wattributes], which the FASE C -Werror (core/CMakeLists.txt)
// turns into a build failure. GCC 12+ honors it; clang needs it to keep
// -Wunused-private-field quiet in the TUs that include this header
// without using these members (that diagnostic is clang-only, so where
// the attribute is absent no warning can fire). Gate the attribute to
// the compilers that actually understand it — a rewrite, not a
// suppression: no -Wno-error=, no pragma, -Werror stays as-is.
#if defined(__clang__) || (defined(__GNUC__) && __GNUC__ >= 12)
#  define CAD_0_NSDM_MAYBE_UNUSED [[maybe_unused]]
#else
#  define CAD_0_NSDM_MAYBE_UNUSED
#endif

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
    // CAD_0_NSDM_MAYBE_UNUSED (see above): these members are used only
    // in opengl_renderer.cpp; clang's per-TU -Wunused-private-field fires
    // in the other TUs of CAD_0_viewport that include this header
    // without touching them.
    CAD_0_NSDM_MAYBE_UNUSED bool initialized_{false};
    CAD_0_NSDM_MAYBE_UNUSED int width_{0};
    CAD_0_NSDM_MAYBE_UNUSED int height_{0};

    // VAO for the full-screen triangle used by the SDF ray-march shader.
    CAD_0_NSDM_MAYBE_UNUSED unsigned int empty_vao_{0};

    // Shader program for SDF ray-marching.
    CAD_0_NSDM_MAYBE_UNUSED unsigned int raymarch_program_{0};

    // Uniform buffer objects (UBOs) for camera + options.
    CAD_0_NSDM_MAYBE_UNUSED unsigned int camera_ubo_{0};
    CAD_0_NSDM_MAYBE_UNUSED unsigned int options_ubo_{0};

    // Simple shader for tessellated meshes (B-Rep + mesh extraction output).
    CAD_0_NSDM_MAYBE_UNUSED unsigned int mesh_program_{0};

    FrameStats last_stats_{};
};

#undef CAD_0_NSDM_MAYBE_UNUSED

} // namespace CAD_0::viewport
