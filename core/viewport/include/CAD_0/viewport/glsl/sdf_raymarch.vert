// core/viewport/include/CAD_0/viewport/glsl/sdf_raymarch.vert
//
// Vertex shader for SDF ray-marching.
// Renders a full-screen triangle (3 vertices) and passes screen
// coordinates to the fragment shader.
//
#version 450 core

// Vertex positions for a full-screen triangle covering NDC [-1, +1].
// The third vertex is at (-1, 3) so the triangle covers the whole screen.
const vec2 positions[3] = vec2[3](
    vec2(-1.0, -1.0),
    vec2( 3.0, -1.0),
    vec2(-1.0,  3.0)
);

out vec2 v_ndc;  // normalized device coordinates [-1, 1]

void main() {
    vec2 p = positions[gl_VertexID];
    v_ndc = p;
    gl_Position = vec4(p, 0.0, 1.0);
}
