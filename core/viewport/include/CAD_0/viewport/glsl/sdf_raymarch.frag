// core/viewport/include/CAD_0/viewport/glsl/sdf_raymarch.frag
//
// Fragment shader for SDF ray-marching.
//
// CPU/GPU Parity Contract
// ------------------------
// The CPU ray marcher (core/viewport/src/ray_march.cpp) and this shader
// MUST produce identical hit distances for the same scene. The parity
// contract is:
//
//   step = d / max(L, 1.0)
//
// where d = scene_sdf(p) and L = field.lipschitz(). For unit-Lipschitz
// SDFs (primitives, ScaleSDF with s ≤ 1), L = 1 and step = d, matching
// this shader's `t += d`. For L > 1 (TwistSDF, SmoothMin with k > 0),
// the CPU marcher divides by L; the shader will need the same treatment
// when it gains L > 1 scene support.
//
// The escape check `if (t > MAX_DIST) return -1.0` is placed BEFORE
// evaluating scene_sdf(p) to avoid unnecessary SDF evaluations when the
// ray has already escaped. This matches the CPU marcher's behavior.
//
// MAX_STEPS, MAX_DIST, and SURF_DIST are exposed as uniforms with
// defaults matching RayMarchOptions (max_steps=128, max_distance=100.0,
// surface_distance=0.0005).
//
#version 450 core

in vec2 v_ndc;  // [-1, 1]

out vec4 frag_color;

// Camera uniforms (set per-frame).
layout(std140, binding = 0) uniform CameraBlock {
    vec4 u_camera_pos;       // xyz = position, w = fov_y
    vec4 u_camera_target;    // xyz = target,   w = aspect
    vec4 u_camera_right;     // xyz = right axis, w = unused
    vec4 u_camera_up;        // xyz = up axis,   w = unused
    vec4 u_camera_forward;   // xyz = forward,    w = unused
};

// Render options.
layout(std140, binding = 1) uniform OptionsBlock {
    vec4 u_options;  // x = show_grid, y = show_axes, z = wireframe, w = unused
    vec4 u_colors;   // xyz = mesh_color, w = unused
};

// Ray-march uniforms. Defaults match RayMarchOptions in ray_march.hpp.
// Override at draw time to tune quality/performance.
layout(location = 10) uniform int   u_max_steps = 128;
layout(location = 11) uniform float u_max_dist  = 100.0;
layout(location = 12) uniform float u_surf_dist = 0.0005;

// Scene SDF — a sphere of radius 1 at the origin.
// In Phase C.6 this will be replaced by a uniform-buffer-driven SDF tree
// evaluated by the GPU. For now we hardcode a simple scene so the shader
// can be tested in isolation.
float scene_sdf(vec3 p) {
    return length(p) - 1.0;
}

// Compute the surface normal at a point via gradient of the SDF.
// Uses a small tetrahedron of 4 samples (faster than the 6-sample cross).
vec3 calc_normal(vec3 p) {
    const vec2 k = vec2(1.0, -1.0);
    return normalize(
        k.xyy * scene_sdf(p + k.xyy * u_surf_dist) +
        k.yyx * scene_sdf(p + k.yyx * u_surf_dist) +
        k.yxy * scene_sdf(p + k.yxy * u_surf_dist) +
        k.xxx * scene_sdf(p + k.xxx * u_surf_dist)
    );
}

// Ray-march from `ro` (ray origin) in direction `rd` (normalized).
// Returns the distance to the surface, or -1.0 if no hit.
//
// CPU/GPU parity: for L=1 SDFs, step = d (same as CPU's d/max(L,1)).
// The escape check is BEFORE the SDF evaluation to avoid wasted work.
float ray_march(vec3 ro, vec3 rd) {
    float t = 0.0;  // distance traveled
    for (int i = 0; i < u_max_steps; i++) {
        // Escape check FIRST — don't evaluate SDF if we're already past
        // the max distance. This matches the CPU marcher's behavior:
        // t += d * effective_scale; if (t > max_distance) break;
        if (t > u_max_dist) return -1.0;

        vec3 p = ro + rd * t;
        float d = scene_sdf(p);
        if (d < u_surf_dist) return t;     // hit
        t += d;                            // step = d / max(L, 1) = d for L=1
    }
    return -1.0;
}

// Simple ambient + Lambert diffuse lighting.
vec3 lighting(vec3 p, vec3 n) {
    const vec3 light_dir = normalize(vec3(0.5, 0.8, 0.3));
    const float ambient = 0.2;
    const float diffuse_strength = 0.8;
    const float diffuse = max(dot(n, light_dir), 0.0);
    return u_colors.xyz * (ambient + diffuse_strength * diffuse);
}

void main() {
    // Build the ray from the camera through this pixel.
    // The NDC is [-1, 1] with +y up.
    vec2 uv = v_ndc;
    vec3 ro = u_camera_pos.xyz;
    vec3 forward = u_camera_forward.xyz;
    vec3 right = u_camera_right.xyz;
    vec3 up = u_camera_up.xyz;
    float fov_y = u_camera_pos.w;
    float aspect = u_camera_target.w;

    // Convert uv to a direction: tan(fov/2) * (uv.x * aspect * right + uv.y * up) + forward.
    float tan_half = tan(fov_y * 0.5);
    vec3 rd = normalize(
        forward + (uv.x * aspect * tan_half) * right + (uv.y * tan_half) * up
    );

    // March the ray.
    float t = ray_march(ro, rd);

    if (t < 0.0) {
        // Background — gradient sky.
        float sky = clamp(rd.y * 0.5 + 0.5, 0.0, 1.0);
        frag_color = vec4(mix(vec3(0.05), vec3(0.3, 0.4, 0.6), sky), 1.0);
        return;
    }

    // Surface hit — compute lighting.
    vec3 p = ro + rd * t;
    vec3 n = calc_normal(p);
    vec3 color = lighting(p, n);

    // Optional grid overlay (when the surface point lies near an integer grid line).
    if (u_options.x > 0.5) {
        vec3 abs_p = abs(fract(p) - 0.5);
        float grid_dist = min(min(abs_p.x, abs_p.y), abs_p.z);
        float grid_factor = smoothstep(0.0, 0.05, grid_dist);
        color = mix(color * 1.3, color, grid_factor);
    }

    frag_color = vec4(color, 1.0);
}
