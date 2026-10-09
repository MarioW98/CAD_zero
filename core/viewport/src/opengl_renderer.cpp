// core/viewport/src/opengl_renderer.cpp
//
// OpenGL 4.5 renderer implementation.
//
// Built only when CAD_0_BUILD_OPENGL=ON. The implementation uses
// direct OpenGL function calls (no GLAD wrapper) — assumes the hosting
// Qt context has resolved all function pointers.
//
// Phase C.6 status: written but not exercised in CI. Manual testing
// requires a machine with OpenGL 4.5 support and PySide6 installed.
//
#include "CAD_0/viewport/opengl_renderer.hpp"

// We rely on the system GL headers when CAD_0_BUILD_OPENGL is on.
// Otherwise this TU is compiled as an empty stub (see CMakeLists).
#if defined(CAD_0_BUILD_OPENGL) && CAD_0_BUILD_OPENGL

#include <glad/gl.h>

#include <fstream>
#include <sstream>
#include <vector>
#include <chrono>

namespace CAD_0::viewport {

namespace {

// Read a file into a string. Used to load GLSL shaders from disk.
std::string read_file(const char* path) {
    std::ifstream f(path);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

// Compile a GLSL shader of the given type. Returns 0 on failure.
unsigned int compile_shader(unsigned int type, const char* source) {
    const unsigned int shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);
    int success = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char log[1024];
        glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
        // In production we'd log this via spdlog.
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

// Link a vertex + fragment shader into a program. Returns 0 on failure.
unsigned int link_program(unsigned int vs, unsigned int fs) {
    const unsigned int prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    glLinkProgram(prog);
    int success = 0;
    glGetProgramiv(prog, GL_LINK_STATUS, &success);
    if (!success) {
        glDeleteProgram(prog);
        return 0;
    }
    return prog;
}

} // namespace

OpenGLRenderer::~OpenGLRenderer() {
    if (!initialized_) return;
    if (empty_vao_)        glDeleteVertexArrays(1, &empty_vao_);
    if (raymarch_program_) glDeleteProgram(raymarch_program_);
    if (mesh_program_)    glDeleteProgram(mesh_program_);
    if (camera_ubo_)       glDeleteBuffers(1, &camera_ubo_);
    if (options_ubo_)      glDeleteBuffers(1, &options_ubo_);
}

bool OpenGLRenderer::initialize(int width, int height) {
    if (width <= 0 || height <= 0) return false;
    width_ = width;
    height_ = height;

    // Empty VAO for the full-screen triangle (positions generated in VS).
    glGenVertexArrays(1, &empty_vao_);

    // UBOs for camera + options (std140 layout, binding 0 and 1).
    glGenBuffers(1, &camera_ubo_);
    glBindBuffer(GL_UNIFORM_BUFFER, camera_ubo_);
    glBufferData(GL_UNIFORM_BUFFER, sizeof(float) * 4 * 5, nullptr, GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_UNIFORM_BUFFER, 0, camera_ubo_);

    glGenBuffers(1, &options_ubo_);
    glBindBuffer(GL_UNIFORM_BUFFER, options_ubo_);
    glBufferData(GL_UNIFORM_BUFFER, sizeof(float) * 4 * 2, nullptr, GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_UNIFORM_BUFFER, 1, options_ubo_);

    // Load and compile the SDF ray-march shader.
    // In production these paths come from a resource system; for now
    // they are hard-coded relative to the working directory.
    const std::string vs_src = read_file("CAD_0/viewport/glsl/sdf_raymarch.vert");
    const std::string fs_src = read_file("CAD_0/viewport/glsl/sdf_raymarch.frag");
    if (vs_src.empty() || fs_src.empty()) return false;
    const auto vs = compile_shader(GL_VERTEX_SHADER, vs_src.c_str());
    const auto fs = compile_shader(GL_FRAGMENT_SHADER, fs_src.c_str());
    if (!vs || !fs) return false;
    raymarch_program_ = link_program(vs, fs);
    glDeleteShader(vs);
    glDeleteShader(fs);
    if (!raymarch_program_) return false;

    initialized_ = true;
    return true;
}

void OpenGLRenderer::resize(int width, int height) {
    if (width <= 0 || height <= 0) return;
    width_ = width;
    height_ = height;
    // The actual GL viewport is set by the hosting QWindow; we just
    // record the dimensions for aspect ratio computation.
}

void OpenGLRenderer::begin_frame(const OrbitCamera& camera,
                                  const RenderOptions& opts) {
    if (!initialized_) return;
    last_stats_ = FrameStats{};

    // Set the GL viewport.
    glViewport(0, 0, width_, height_);

    // Clear the framebuffer.
    const auto& bg = opts.background_color;
    glClearColor(bg.x, bg.y, bg.z, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Update the camera UBO.
    // Layout (std140, binding 0):
    //   vec4 u_camera_pos;     // xyz = position, w = fov_y
    //   vec4 u_camera_target;   // xyz = target,   w = aspect
    //   vec4 u_camera_right;    // xyz = right axis
    //   vec4 u_camera_up;       // xyz = up axis
    //   vec4 u_camera_forward;  // xyz = forward
    const auto pos = camera.position();
    const auto tgt = camera.target();
    const auto right = camera.right();
    const auto up = camera.up();
    const auto fwd = camera.forward();
    const float aspect = float(width_) / float(height_);

    struct CameraUBO {
        float pos[4];
        float target[4];
        float right[4];
        float up[4];
        float forward[4];
    } cam_ubo{};
    cam_ubo.pos[0] = pos.x;     cam_ubo.pos[1] = pos.y;     cam_ubo.pos[2] = pos.z;     cam_ubo.pos[3] = camera.fov_y();
    cam_ubo.target[0] = tgt.x;  cam_ubo.target[1] = tgt.y;  cam_ubo.target[2] = tgt.z;  cam_ubo.target[3] = aspect;
    cam_ubo.right[0] = right.x; cam_ubo.right[1] = right.y; cam_ubo.right[2] = right.z; cam_ubo.right[3] = 0.0f;
    cam_ubo.up[0] = up.x;       cam_ubo.up[1] = up.y;       cam_ubo.up[2] = up.z;       cam_ubo.up[3] = 0.0f;
    cam_ubo.forward[0] = fwd.x; cam_ubo.forward[1] = fwd.y; cam_ubo.forward[2] = fwd.z; cam_ubo.forward[3] = 0.0f;

    glBindBuffer(GL_UNIFORM_BUFFER, camera_ubo_);
    glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(cam_ubo), &cam_ubo);

    // Update the options UBO.
    struct OptionsUBO {
        float options[4];
        float colors[4];
    } opt_ubo{};
    opt_ubo.options[0] = opts.show_grid ? 1.0f : 0.0f;
    opt_ubo.options[1] = opts.show_axes ? 1.0f : 0.0f;
    opt_ubo.options[2] = opts.wireframe ? 1.0f : 0.0f;
    opt_ubo.options[3] = 0.0f;
    opt_ubo.colors[0] = opts.mesh_color.x;
    opt_ubo.colors[1] = opts.mesh_color.y;
    opt_ubo.colors[2] = opts.mesh_color.z;
    opt_ubo.colors[3] = 0.0f;

    glBindBuffer(GL_UNIFORM_BUFFER, options_ubo_);
    glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(opt_ubo), &opt_ubo);

    if (opts.backface_culling) glEnable(GL_CULL_FACE);
    else                        glDisable(GL_CULL_FACE);

    if (opts.wireframe) glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    else                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void OpenGLRenderer::draw_mesh(const RenderMesh& mesh,
                                const math::Mat4f& model_matrix) {
    (void)model_matrix;  // model matrix is not yet wired to the mesh shader
    if (!initialized_) return;

    // For Phase C.6 we use the ray-march shader for the SDF preview.
    // The mesh shader (for tessellated B-Rep output) is a stub — it will
    // be implemented when Phase D lands and we have a real B-Rep body to
    // tessellate.

    last_stats_.draw_calls++;
    last_stats_.triangles += mesh.indices.size() / 3;
    last_stats_.vertices += mesh.positions.size();
}

void OpenGLRenderer::draw_grid() {
    if (!initialized_) return;
    // The grid is drawn inside the SDF fragment shader via a uniform
    // flag (u_options.x). Nothing to do here.
    last_stats_.draw_calls++;
    last_stats_.triangles += 80;
}

void OpenGLRenderer::draw_axes() {
    if (!initialized_) return;
    last_stats_.draw_calls++;
    last_stats_.triangles += 6;
}

FrameStats OpenGLRenderer::end_frame() {
    last_stats_.frame_time_ms = 0.0;  // TODO: measure with steady_clock
    return last_stats_;
}

} // namespace CAD_0::viewport

#else  // !CAD_0_BUILD_OPENGL

// Empty TU when OpenGL is disabled. The class declaration is still
// available (in the header) for downstream code that wants to
// conditionally reference it.

namespace CAD_0::viewport {

OpenGLRenderer::~OpenGLRenderer() = default;
bool OpenGLRenderer::initialize(int, int) { return false; }
void OpenGLRenderer::resize(int, int) {}
void OpenGLRenderer::begin_frame(const OrbitCamera&, const RenderOptions&) {}
void OpenGLRenderer::draw_mesh(const RenderMesh&, const math::Mat4f&) {}
void OpenGLRenderer::draw_grid() {}
void OpenGLRenderer::draw_axes() {}
FrameStats OpenGLRenderer::end_frame() { return FrameStats{}; }

} // namespace CAD_0::viewport

#endif // CAD_0_BUILD_OPENGL
