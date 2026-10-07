# ADR-0017: Viewport architecture — headless-first rendering

## Status
Accepted (2025-10-07) — Phase C (steps 1-5 completed, 6-8 pending).

## Context
The CAD_0 viewport needs to render both SDF (ray-marched) and B-Rep
(tessellated) geometry, support orbit/pan/zoom camera, picking, undo/redo,
and selection. The challenge is testing all this without requiring an
OpenGL context in CI.

The original Phase C plan called for PySide6 + OpenGL 4.5 as the baseline,
with Vulkan as an optional backend. But PySide6 cannot run headless in CI
without a display, and OpenGL tests require a GL context (typically via
EGL or a virtual framebuffer).

## Decision
**Headless-first rendering**: define an abstract `Renderer` interface that
can be implemented by:
1. `HeadlessRenderer` — CPU-side framebuffer, records draw calls and
   produces a flat-colored image. Used in tests and for headless CI.
2. `OpenGLRenderer` — Phase C.6, real GPU rendering via PySide6's QWindow.
3. `VulkanRenderer` — post-MVP.

All camera/selection/command-stack logic is **UI-agnostic** and lives in
`core/viewport/`. The PySide6 layer (Phase C.6+) is a thin adapter that:
- Translates Qt input events (mouse, keyboard) into camera operations
- Wraps `SelectionModel` in a `QAbstractListModel` for the property panel
- Hosts a `QWindow` subclass that drives the `OpenGLRenderer`

### Module layout
```
core/viewport/
├── include/CAD_0/viewport/
│   ├── camera.hpp           # OrbitCamera — turntable, ray-from-ndc
│   ├── command_stack.hpp    # undo/redo stack with Command pattern
│   ├── selection.hpp        # ShapeId-based selection model
│   ├── renderer.hpp         # Renderer interface + HeadlessRenderer
│   ├── ray_march.hpp        # CPU ray-marcher (mirrors GLSL frag shader)
│   └── glsl/
│       ├── sdf_raymarch.vert
│       └── sdf_raymarch.frag
└── src/
    ├── renderer.cpp         # HeadlessRenderer implementation
    └── ray_march.cpp        # CPU ray-marcher implementation
```

### Camera (OrbitCamera)
Turntable-style: yaw around world Y, pitch clamped to ±89° to avoid
gimbal lock. Distance from target adjustable via zoom. All operations
are reversible — `orbit()` returns a delta that `orbit_inverse()`
exactly undoes.

`ray_from_ndc(x, y)` converts normalized device coordinates to a
world-space ray, used for picking.

### Command stack
Classic Command pattern: each user action is a `Command` with `execute()`
and `undo()`. The `CommandStack` maintains undo + redo stacks with a
configurable max size (default 1000). A `LambdaCommand` helper lets
simple commands be defined inline.

### Selection model
Operates on `ShapeId` (genealogy-based, see ADR-0005). Supports
Replace / Add / Subtract / Toggle modes for multi-select. Hover is
independent of selection — useful for preview.

### Ray-marching
The CPU ray-marcher (`ray_march.hpp`) mirrors the GLSL fragment shader
algorithm. This lets us:
- Verify shader logic without an OpenGL context
- Use the same algorithm for picking (click → ray → hit point + normal)
- Optionally produce pixel output in the HeadlessRenderer

## Test results
**165/165 tests pass** (54 viewport tests + 111 from earlier phases):

| Module | Tests | Notes |
|---|---|---|
| Camera | 16 | orbit, pan, zoom, frame_bounds, ray_from_ndc |
| Command stack | 15 | undo/redo, max_size, lambda commands |
| Selection | 16 | Replace/Add/Subtract/Toggle, hover |
| Renderer | 11 | HeadlessRenderer draw calls + framebuffer |
| Ray-march | 12 | hits, misses, normals, max_distance, max_steps |

## Consequences
- The viewport module is fully testable in CI without a display.
- Phase C.6 (PySide6 widget) is the first non-testable layer; it will
  ship as a thin adapter with manual testing.
- The OpenGL renderer will be the second `Renderer` implementation,
  added in Phase C.6. No API changes needed — the interface is stable.
- The GLSL shaders are installed as data files; they can be hot-reloaded
  in development.

## Future work
1. **Phase C.6**: `OpenGLRenderer` + PySide6 `QWindow` subclass
2. **Phase C.7**: Feature tree + property panel (wraps SelectionModel)
3. **Phase C.8**: Application shell (window, menus, toolbar)
4. **Phase C.9 (post-MVP)**: GPU-driven SDF evaluation (replace the
   hardcoded `scene_sdf` in the fragment shader with a UBO-backed SDF tree)
5. **Phase C.10 (post-MVP)**: Vulkan backend
