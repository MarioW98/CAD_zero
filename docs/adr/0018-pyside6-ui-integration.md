# ADR-0018: PySide6 UI integration

## Status
Accepted (2025-10-07) — Phase C (steps 6-8 completed).

## Context
Phase C.1-C.5 (ADR-0017) established the headless-first viewport
architecture: a fully testable `core/viewport/` C++ module exposing
camera, command stack, selection, renderer, and ray-march components.

The remaining task is to assemble these into a usable desktop
application: a window with menus, a viewport widget, a feature tree
panel, and a property panel.

The original plan called for PySide6 (Python) as the primary UI, with
Qt6 C++ removed entirely (ADR-0006). This ADR documents how that
decision is realized in code.

## Decision
The PySide6 UI lives under `app/src/CAD_0/ui/` as a pure-Python
package. The Python package is merged with the compiled extension
package at install time so that `import CAD_0.ui.main_window`
works.

### Module layout
```
app/
├── CMakeLists.txt              # installs the ui/ Python package
├── resources/                  # icons, themes, fonts (placeholder)
└── src/
    └── CAD_0/
        ├── __init__.py         # version string
        └── ui/
            ├── __init__.py
            ├── __main__.py     # `python -m CAD_0.ui` entry point
            ├── viewport_widget.py   # ViewportWidget + _ViewportWindow
            ├── feature_tree.py     # FeatureTreeModel + FeatureTreePanel
            ├── property_panel.py   # PropertyPanel
            └── main_window.py      # MainWindow (assembles everything)
```

### Component responsibilities

**ViewportWidget** (`viewport_widget.py`)
- Hosts a `QOpenGLWindow` via `QWidget.createWindowContainer`
- Captures mouse/wheel events and translates them into camera operations
  (orbit on left-drag, pan on middle-drag, zoom on wheel)
- Stores camera state in Python (yaw, pitch, distance, target) — will
  be replaced with the C++ `OrbitCamera` once nanobind bindings land
- Public API mirrors `OrbitCamera`: `set_camera_target`, `set_camera_distance`,
  `set_camera_yaw_pitch`, `frame_all`

**FeatureTreePanel** (`feature_tree.py`)
- `QTreeView` subclass backed by `FeatureTreeModel` (`QAbstractItemModel`)
- `FeatureNode` dataclass: name, representation (SDF/BRep/Hybrid), visibility,
  selection, children, parameters
- Columns: Name (editable, with checkbox) + Representation
- Selection model emits `currentChanged` → PropertyPanel updates

**PropertyPanel** (`property_panel.py`)
- `QFormLayout` with one row per parameter
- Auto-creates the right editor widget based on the parameter's type:
  - `bool` → `QCheckBox`
  - `int` → `QSpinBox`
  - `float` → `QDoubleSpinBox`
  - `str` → `QLineEdit`
  - other → read-only `QLabel`
- Emits `parameter_changed(name, value)` when the user edits a parameter

**MainWindow** (`main_window.py`)
- Assembles viewport (central widget) + feature tree (left dock, top) +
  property panel (left dock, bottom)
- Menu bar: File (New/Open/Save/Export STL|OBJ|3MF/Quit), Edit (Undo/Redo),
  View (Toggle grid/axes/Frame all), Help (About)
- Toolbar with quick access to Undo, Redo, grid toggle, axes toggle
- Status bar for transient messages
- `_on_tree_selection_changed` updates the property panel when the user
  clicks a feature in the tree

### OpenGLRenderer (`core/viewport/opengl_renderer.{hpp,cpp}`)
The second concrete `Renderer` implementation. Built only when
`CAD_0_BUILD_OPENGL=ON`. Uses:
- OpenGL 4.5 core profile (matches the QSurfaceFormat in viewport_widget)
- UBOs (binding 0 = camera, binding 1 = options) for std140 layout
- The GLSL shaders from `core/viewport/include/CAD_0/viewport/glsl/`
- An empty VAO for the full-screen triangle used by the ray-march shader

When `CAD_0_BUILD_OPENGL=OFF` (default in CI), the file compiles
as an empty stub so the build doesn't require GL headers.

### Why QOpenGLWindow and not QOpenGLWidget
- Lower latency: QOpenGLWindow has a direct connection to the platform's
  surface, while QOpenGLWidget goes through Qt's compositor
- Better VR compatibility: QOpenGLWindow can be wrapped in a native
  window handle for OpenXR
- Trade-off: QOpenGLWidget integrates better with Qt's stylesheet
  system, but for a CAD viewport where the GL surface is fullscreen
  within its container, the integration cost is minimal

## Testability
**The PySide6 UI layer is the only non-tested layer of the codebase.**
Everything below it is fully covered by doctest (165/165 tests pass).

The smoke tests in `bindings/python/tests/test_ui_smoke.py` verify that
the Python modules at least parse cleanly (via `ast.parse`), so syntax
errors are caught in CI without requiring PySide6 or a display.

## Future work
1. **Wire the OrbitCamera bindings** — once nanobind is configured for
   the `core/viewport` module, `ViewportWidget` should hold a C++
   `OrbitCamera` instance instead of Python state.
2. **Connect the feature tree to the C++ feature tree** — Phase F will
   define the feature tree in C++; the Python model will wrap it.
3. **Implement the OpenGLRenderer draw calls** — currently `draw_mesh`
   is a stub that just counts triangles. Phase D (B-Rep) will require
   real mesh rendering.
4. **GPU-driven SDF evaluation** — replace the hardcoded `scene_sdf`
   function in the fragment shader with a UBO-backed SDF tree.
5. **Vulkan backend** — post-MVP, a third `Renderer` implementation.
