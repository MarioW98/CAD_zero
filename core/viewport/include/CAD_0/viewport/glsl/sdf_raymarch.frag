// core/viewport/include/CAD_0/viewport/glsl/sdf_raymarch.frag
//
// Fragment shader for SDF ray-marching.
//
// For each pixel, computes the world-space ray from the camera through
// the pixel, then marches the ray through the SDF field until it hits
// the surface (or escapes to background). Applies simple ambient + diffuse
// lighting with a single directional light.
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

// Maximum ray-march steps and minimum surface distance.
const int MAX_STEPS = 128;
const float MAX_DIST = 100.0;
const float SURF_DIST = 0.0005;

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
        k.xyy * scene_sdf(p + k.xyy * SURF_DIST) +
        k.yyx * scene_sdf(p + k.yyx * SURF_DIST) +
        k.yxy * scene_sdf(p + k.yxy * SURF_DIST) +
        k.xxx * scene_sdf(p + k.xxx * SURF_DIST)
    );
}

// Ray-march from `ro` (ray origin) in direction `rd` (normalized).
// Returns the distance to the surface, or -1.0 if no hit.
float ray_march(vec3 ro, vec3 rd) {
    float t = 0.0;  // distance traveled
    for (int i = 0; i < MAX_STEPS; i++) {
        vec3 p = ro + rd * t;
        float d = scene_sdf(p);
        if (d < SURF_DIST) return t;     // hit
        if (t > MAX_DIST) return -1.0;   // escaped
        t += d;                           // step by signed distance
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
