"""CAD_0.ui.viewport_widget — PySide6 viewport with real OpenGL rendering.

Uses PyOpenGL for fixed-function pipeline (glMatrixMode, glBegin, etc.)
and QOpenGLWindow as the rendering surface.

Requires: pip install PyOpenGL PySide6
"""

from __future__ import annotations

import sys
import math
from typing import Optional, Any

import numpy as np
from PySide6.QtCore import Qt, QPoint, Signal
from PySide6.QtGui import QSurfaceFormat, QMouseEvent, QWheelEvent
from PySide6.QtOpenGL import QOpenGLWindow
from PySide6.QtWidgets import QWidget, QVBoxLayout

# Import OpenGL via PyOpenGL — provides ALL fixed-function + modern GL
from OpenGL.GL import (
    glEnable, glDisable, glClearColor, glClear, glViewport,
    glMatrixMode, glLoadIdentity, glFrustum, glMultMatrixf, glTranslatef,
    glLightfv, glColor3f, glBegin, glEnd, glVertex3f, glNormal3f,
    glLineWidth,
    GL_DEPTH_TEST, GL_LEQUAL, GL_LIGHTING, GL_LIGHT0,
    GL_COLOR_MATERIAL, GL_NORMALIZE,
    GL_COLOR_BUFFER_BIT, GL_DEPTH_BUFFER_BIT,
    GL_PROJECTION, GL_MODELVIEW,
    GL_LINES, GL_TRIANGLES,
    GL_POSITION, GL_DIFFUSE, GL_AMBIENT,
    glDepthFunc,
)


class ViewportWidget(QWidget):
    """A QWidget that hosts an OpenGL viewport with real rendering."""

    clicked = Signal(float, float)

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._scene = None
        self._setup_ui()

        # Camera state
        self._cam_target = (0.0, 0.0, 0.0)
        self._cam_distance = 5.0
        self._cam_yaw = 0.0
        self._cam_pitch = 0.3
        self._cam_fov = 45.0
        self._cam_aspect = 1.0

        # NOTE (BUG 2 fix): no mouse-tracking state here. With the GL path
        # active, mouse/wheel events are delivered to the embedded
        # _ViewportWindow (see its class docstring) — the wrapper never
        # receives them, so navigation lives on the window. The camera
        # state below stays on the wrapper because paintGL() and
        # MainWindow's picking already read it there.

        # Display flags
        self._show_grid = True
        self._show_axes = True

        # Tessellation resolution for the display pipeline (View ->
        # Tessellation Resolution; graphics sprint optional item). 48 is
        # the default every pinned count and golden assumes; the setter
        # + MainWindow's re-dirty pass implement the menu's re-tess.
        self._tess_resolution = 48

        # Mesh data cache
        self._mesh_positions = None
        self._mesh_normals = None
        self._mesh_indices = None
        # Selection highlight state: node id -> (first tri, last tri) in
        # the concatenated index buffer, plus the currently selected id.
        # Populated by _refresh_meshes so _draw_meshes can repaint just
        # the selected node's triangles in the highlight color.
        self._node_ranges = {}
        self._selected_node_id = None

    def _setup_ui(self) -> None:
        fmt = QSurfaceFormat()
        fmt.setVersion(2, 1)
        fmt.setProfile(QSurfaceFormat.CompatibilityProfile)
        fmt.setDepthBufferSize(24)
        fmt.setSwapBehavior(QSurfaceFormat.DoubleBuffer)
        QSurfaceFormat.setDefaultFormat(fmt)

        # Try to create a real GL window. In headless / no-GPU environments
        # this can fail and Qt may segfault during cleanup; we offer a
        # non-GL fallback so the rest of the application still works.
        self._gl_window = None
        self._gl_fallback_label = None
        self._gl_container = None
        gl_available = self._check_gl_available(fmt)
        if gl_available:
            try:
                self._gl_window = _ViewportWindow(self)
                self._gl_window.setFormat(fmt)
                container = QWidget.createWindowContainer(self._gl_window, self)
                container.setFocusPolicy(Qt.StrongFocus)
                self._gl_container = container
            except Exception as e:
                print(f"[viewport] GL window creation failed: {e}")
                self._gl_window = None
                self._gl_container = None

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        if self._gl_container is not None:
            layout.addWidget(self._gl_container)
        else:
            from PySide6.QtWidgets import QLabel
            self._gl_fallback_label = QLabel(
                "OpenGL not available.\n"
                "The C++ kernel and scene graph still work, but the 3D "
                "viewport requires a GPU or software OpenGL renderer."
            )
            self._gl_fallback_label.setAlignment(Qt.AlignCenter)
            layout.addWidget(self._gl_fallback_label)
        self.setLayout(layout)

    @staticmethod
    def _check_gl_available(fmt: QSurfaceFormat) -> bool:
        """Probe whether we can actually create a GL context.

        This avoids the QOpenGLWindow segfault that happens on cleanup
        when GL context creation fails (common in headless environments).
        """
        try:
            from PySide6.QtGui import QOpenGLContext
            ctx = QOpenGLContext()
            ctx.setFormat(fmt)
            if not ctx.create():
                print("[viewport] GL context probe failed — using fallback widget")
                return False
            return True
        except Exception as e:
            print(f"[viewport] GL probe error: {e}")
            return False

    def _request_update(self) -> None:
        if self._gl_window is not None:
            self._gl_window.requestUpdate()
        else:
            self.update()

    # ----- Public API -----

    def set_scene(self, scene: Any) -> None:
        self._scene = scene
        self._refresh_meshes()

    def _refresh_meshes(self) -> None:
        if self._scene is None:
            return
        if hasattr(self._scene, 'update_meshes'):
            self._scene.update_meshes(self._tess_resolution)
        positions = []
        normals = []
        indices = []
        offset = 0
        tri_offset = 0
        node_ranges = {}
        count = self._scene.node_count()
        for i in range(count):
            node = self._scene.node_at(i)
            if node is None or not node.visible:
                continue
            mesh = node.cached_mesh
            pos = mesh.positions
            idx = mesh.indices
            if pos is None or len(pos) == 0 or idx is None or len(idx) == 0:
                continue
            # Shading normals come from the field, not the mesh (the
            # graphics-sprint display pipeline — see _display_normals).
            # Positions/indices are still the MC mesh: exports and the
            # bridge read those from the node, byte-identically.
            nor = _display_normals(node)
            n_tris = len(idx) // 3
            node_ranges[node.id] = (tri_offset, tri_offset + n_tris)
            tri_offset += n_tris
            positions.append(pos)
            if nor is not None and len(nor) == len(pos):
                normals.append(nor)
            else:
                normals.append(np.zeros_like(pos))
            indices.append(idx + offset)
            offset += len(pos)
        self._node_ranges = node_ranges
        if positions:
            self._mesh_positions = np.concatenate(positions)
            self._mesh_normals = np.concatenate(normals)
            self._mesh_indices = np.concatenate(indices)
        else:
            self._mesh_positions = None
            self._mesh_indices = None
        self._request_update()

    def set_camera_target(self, x: float, y: float, z: float) -> None:
        self._cam_target = (x, y, z)
        self._request_update()

    def set_camera_distance(self, d: float) -> None:
        self._cam_distance = max(0.001, d)
        self._request_update()

    def set_camera_yaw_pitch(self, yaw: float, pitch: float) -> None:
        self._cam_yaw = yaw
        self._cam_pitch = max(-1.5533, min(1.5533, pitch))
        self._request_update()

    def frame_all(self) -> None:
        if self._scene is not None and not self._scene.empty():
            b = self._scene.bounds()
            self._cam_target = (
                (b.min.x + b.max.x) * 0.5,
                (b.min.y + b.max.y) * 0.5,
                (b.min.z + b.max.z) * 0.5,
            )
            extent = b.extent()
            radius = max(extent.x, extent.y, extent.z) * 0.5
            self._cam_distance = max(radius * 3.0, 2.0)
        else:
            self._cam_target = (0.0, 0.0, 0.0)
            self._cam_distance = 5.0
            self._cam_yaw = 0.0
            self._cam_pitch = 0.3
        self._request_update()

    # ----- Project persistence (native .CAD_0 format) -----------------------

    def camera_state(self) -> dict:
        """Current camera as a dict for CAD_0.io.save()."""
        return {
            "target": tuple(self._cam_target),
            "distance": self._cam_distance,
            "yaw": self._cam_yaw,
            "pitch": self._cam_pitch,
            "fov_deg": self._cam_fov,
        }

    def apply_camera_state(self, cam: dict) -> None:
        """Restore the camera from a dict produced by CAD_0.io.load()."""
        try:
            target = tuple(cam.get("target", (0.0, 0.0, 0.0)))
            self.set_camera_target(
                float(target[0]), float(target[1]), float(target[2]))
            self.set_camera_distance(float(cam.get("distance", 5.0)))
            self.set_camera_yaw_pitch(
                float(cam.get("yaw", 0.0)), float(cam.get("pitch", 0.3)))
            fov = float(cam.get("fov_deg", 45.0))
            if 10.0 <= fov <= 120.0:
                self._cam_fov = fov
        except (TypeError, ValueError, IndexError):
            # Malformed camera block: keep the current camera (the shapes
            # are still valid — camera is a convenience, not geometry).
            pass
        self._request_update()

    def set_show_grid(self, show: bool) -> None:
        self._show_grid = show
        self._request_update()

    def set_selection(self, node_id) -> None:
        """Highlight ``node_id`` in the 3D viewport (None = none).

        Selection feedback: MainWindow calls this whenever the feature
        tree selection changes (from a click OR from the tree itself) so
        the picked shape is visibly highlighted in 3D, not just in the
        tree/panel. Unknown ids (deleted nodes) simply draw nothing.
        """
        if self._selected_node_id != node_id:
            self._selected_node_id = node_id
            self._request_update()

    def set_show_axes(self, show: bool) -> None:
        self._show_axes = show
        self._request_update()

    def set_tessellation_resolution(self, resolution: int) -> None:
        """Set the display tessellation resolution (View menu).

        Stores the value for the next _refresh_meshes; actually
        re-tessellating is the caller's business (MainWindow marks the
        Sdf nodes dirty first — update_meshes skips clean caches, so
        the resolution change alone would be a no-op).
        """
        self._tess_resolution = int(resolution)

    # ----- Input -----
    #
    # No mouse/wheel handlers on the wrapper (BUG 2 fix): when the GL
    # path is active the embedded _ViewportWindow owns all pointer input
    # (see _ViewportWindow for orbit/pan/zoom and the picking bridge).
    # In the no-GL fallback path this widget shows a static label; there
    # is no camera to steer and no scene to pick, so nothing is lost.


class _ViewportWindow(QOpenGLWindow):
    """The actual QOpenGLWindow that renders the scene.

    Event routing (BUG 2 fix): with GL active, this window is embedded
    via QWidget.createWindowContainer() and the windowing system
    delivers ALL mouse/wheel events to it — the wrapper ViewportWidget
    never sees them (that is why navigation was dead while Ctrl+O kept
    working). Navigation therefore lives HERE, in one place only. The
    camera state it mutates stays on the wrapper (self._parent._cam_*),
    exactly where paintGL()/resizeGL() and MainWindow's picking already
    read it.

    Picking bridge: a short left click (press + release without a real
    drag) is re-emitted through the WRAPPER's ``clicked(ndc_x, ndc_y)``
    signal — the same signal MainWindow._on_viewport_click has always
    been connected to. The wrapper owns the signal because it is the
    public widget; this window is a private rendering detail the rest
    of the app never touches directly.

    Focus: the container widget carries StrongFocus; on press we call
    requestActivate() so the embedded window itself becomes the focus
    object, and focusOutEvent() drops any in-flight drag so a lost
    window can never leave a "stuck button".
    """

    def __init__(self, parent_widget: ViewportWidget) -> None:
        super().__init__()
        self._parent = parent_widget
        self._initialized = False
        # Navigation state (moved from the wrapper — see class docstring).
        self._last_mouse_pos: Optional[QPoint] = None
        self._mouse_button = Qt.MouseButton.NoButton
        self._drag_distance = 0

    # ----- Navigation (orbit / pan / zoom) -----

    def mousePressEvent(self, event: QMouseEvent) -> None:
        self._last_mouse_pos = event.position().toPoint()
        self._mouse_button = event.button()
        self._drag_distance = 0
        self.requestActivate()

    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        if self._last_mouse_pos is None:
            return
        delta = event.position().toPoint() - self._last_mouse_pos
        self._last_mouse_pos = event.position().toPoint()
        self._drag_distance += abs(delta.x()) + abs(delta.y())

        if self._mouse_button == Qt.MouseButton.LeftButton:
            sensitivity = 0.005
            self._parent._cam_yaw += delta.x() * sensitivity
            self._parent._cam_pitch -= delta.y() * sensitivity
            self._parent._cam_pitch = max(-1.5533, min(1.5533, self._parent._cam_pitch))
            self._parent._request_update()
        elif self._mouse_button == Qt.MouseButton.MiddleButton:
            sensitivity = 0.01 * self._parent._cam_distance
            tx, ty, tz = self._parent._cam_target
            tx -= delta.x() * sensitivity
            ty += delta.y() * sensitivity
            self._parent._cam_target = (tx, ty, tz)
            self._parent._request_update()

    def mouseReleaseEvent(self, event: QMouseEvent) -> None:
        # Picking bridge (class docstring): short click -> wrapper's
        # ``clicked`` signal -> MainWindow._on_viewport_click.
        if (self._mouse_button == Qt.MouseButton.LeftButton and
            self._drag_distance < 5 and event.button() == Qt.MouseButton.LeftButton):
            pos = event.position().toPoint()
            w = max(1, self.width())
            h = max(1, self.height())
            ndc_x = (2.0 * pos.x() / w) - 1.0
            ndc_y = 1.0 - (2.0 * pos.y() / h)
            self._parent.clicked.emit(ndc_x, ndc_y)
        self._last_mouse_pos = None
        self._mouse_button = Qt.MouseButton.NoButton

    def wheelEvent(self, event: QWheelEvent) -> None:
        notches = event.angleDelta().y() / 120.0
        factor = 0.9 if notches > 0 else 1.1
        self._parent._cam_distance *= factor ** abs(notches)
        self._parent._cam_distance = max(0.001, min(1e6, self._parent._cam_distance))
        self._parent._request_update()

    # ----- Focus -----

    def focusOutEvent(self, event) -> None:
        # Losing the window mid-drag (alt-tab, modal dialog) must not
        # leave a pressed button behind: drop the drag state so the
        # next press starts from a clean slate.
        self._last_mouse_pos = None
        self._mouse_button = Qt.MouseButton.NoButton
        super().focusOutEvent(event)

    def initializeGL(self) -> None:
        # Guard against headless / no-GPU environments: if Qt failed to
        # create a GL context, all PyOpenGL calls would crash. Detect this
        # early and skip every subsequent paintGL.
        try:
            from PySide6.QtGui import QOpenGLContext
            ctx = QOpenGLContext.currentContext()
            if ctx is None:
                print("[viewport] no GL context available — GL rendering disabled")
                self._initialized = False
                return
        except Exception:
            self._initialized = False
            return
        try:
            glEnable(GL_DEPTH_TEST)
            glDepthFunc(GL_LEQUAL)
            glEnable(GL_LIGHTING)
            glEnable(GL_LIGHT0)
            glEnable(GL_COLOR_MATERIAL)
            glEnable(GL_NORMALIZE)
            self._initialized = True
        except Exception as e:
            print(f"[viewport] GL initialization failed: {e}")
            self._initialized = False

    def resizeGL(self, w: int, h: int) -> None:
        if not self._initialized:
            return
        try:
            self._parent._cam_aspect = w / max(1, h)
            glViewport(0, 0, w, h)
        except Exception:
            pass

    def paintGL(self) -> None:
        if not self._initialized:
            return
        # Re-check the context on every paint — in headless environments
        # the context may become invalid between frames.
        try:
            from PySide6.QtGui import QOpenGLContext
            ctx = QOpenGLContext.currentContext()
            if ctx is None:
                return
        except Exception:
            return
        try:
            self._paint_impl()
        except Exception as e:
            # Don't let a GL error crash the whole app — log and skip.
            # In real desktop usage with a working GPU this never fires.
            print(f"[viewport] paint error (suppressed): {e}")

    def _paint_impl(self) -> None:
        """Actual GL painting — wrapped by paintGL for safety."""

        glClearColor(0.15, 0.15, 0.18, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        # Projection
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        fov_y = self._parent._cam_fov
        aspect = self._parent._cam_aspect
        near_p = 0.01
        far_p = 1000.0
        top = near_p * math.tan(math.radians(fov_y) * 0.5)
        bottom = -top
        right = top * aspect
        left = -right
        glFrustum(left, right, bottom, top, near_p, far_p)

        # View (look-at)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        cp = math.cos(self._parent._cam_pitch)
        sp = math.sin(self._parent._cam_pitch)
        cy = math.cos(self._parent._cam_yaw)
        sy = math.sin(self._parent._cam_yaw)
        cam_dir = (cp * sy, sp, cp * cy)
        tx, ty, tz = self._parent._cam_target
        cam_pos = (tx + cam_dir[0] * self._parent._cam_distance,
                   ty + cam_dir[1] * self._parent._cam_distance,
                   tz + cam_dir[2] * self._parent._cam_distance)

        fx = tx - cam_pos[0]
        fy = ty - cam_pos[1]
        fz = tz - cam_pos[2]
        fl = math.sqrt(fx*fx + fy*fy + fz*fz)
        if fl < 1e-9:
            fx, fy, fz = 0, 0, -1
        else:
            fx, fy, fz = fx/fl, fy/fl, fz/fl
        rx = -fz
        ry = 0.0
        rz = fx
        rl = math.sqrt(rx*rx + rz*rz)
        if rl > 1e-9:
            rx, rz = rx/rl, rz/rl
        else:
            rx, rz = 1, 0
        ux = ry * fz - rz * fy
        uy = rz * fx - rx * fz
        uz = rx * fy - ry * fx

        m = (GLfloat * 16)(
            rx, ux, -fx, 0,
            ry, uy, -fy, 0,
            rz, uz, -fz, 0,
            0, 0, 0, 1
        )
        glMultMatrixf(m)
        glTranslatef(-cam_pos[0], -cam_pos[1], -cam_pos[2])

        # Light
        glLightfv(GL_LIGHT0, GL_POSITION, [0.5, 0.8, 0.3, 0.0])
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [0.8, 0.8, 0.8, 1.0])
        glLightfv(GL_LIGHT0, GL_AMBIENT, [0.2, 0.2, 0.2, 1.0])

        # Grid
        if self._parent._show_grid:
            self._draw_grid()
        # Axes
        if self._parent._show_axes:
            self._draw_axes()
        # Meshes
        if self._parent._mesh_positions is not None and self._parent._mesh_indices is not None:
            self._draw_meshes()

    def _draw_grid(self) -> None:
        glDisable(GL_LIGHTING)
        glColor3f(0.3, 0.3, 0.3)
        glBegin(GL_LINES)
        extent = 5.0
        step = 1.0
        i = -extent
        while i <= extent:
            glVertex3f(i, 0, -extent)
            glVertex3f(i, 0, extent)
            glVertex3f(-extent, 0, i)
            glVertex3f(extent, 0, i)
            i += step
        glEnd()
        glEnable(GL_LIGHTING)

    def _draw_axes(self) -> None:
        glDisable(GL_LIGHTING)
        glLineWidth(2.0)
        glColor3f(1, 0, 0)
        glBegin(GL_LINES)
        glVertex3f(0, 0, 0)
        glVertex3f(2, 0, 0)
        glEnd()
        glColor3f(0, 1, 0)
        glBegin(GL_LINES)
        glVertex3f(0, 0, 0)
        glVertex3f(0, 2, 0)
        glEnd()
        glColor3f(0, 0, 1)
        glBegin(GL_LINES)
        glVertex3f(0, 0, 0)
        glVertex3f(0, 0, 2)
        glEnd()
        glLineWidth(1.0)
        glEnable(GL_LIGHTING)

    def _draw_meshes(self) -> None:
        positions = self._parent._mesh_positions
        normals = self._parent._mesh_normals
        indices = self._parent._mesh_indices
        if positions is None or len(positions) == 0:
            return
        glColor3f(0.7, 0.75, 0.8)
        has_normals = normals is not None and len(normals) == len(positions)
        n_tris = len(indices) // 3
        self._draw_tri_range(positions, normals, indices,
                             has_normals, 0, n_tris)
        # Selection highlight (visible feedback for "Click on shape |
        # Select it"): repaint ONLY the selected node's triangles in
        # amber, on top of the base pass. Same geometry, so the depth
        # test (LEQUAL) lets the repaint win exactly where the shape is
        # visible — no z-fighting, no halo.
        sel = self._parent._selected_node_id
        if sel is not None:
            rng = self._parent._node_ranges.get(sel)
            if rng is not None and rng[1] > rng[0]:
                glColor3f(1.0, 0.62, 0.10)
                self._draw_tri_range(positions, normals, indices,
                                     has_normals, rng[0], rng[1])

    @staticmethod
    def _draw_tri_range(positions, normals, indices, has_normals,
                        tri_start: int, tri_end: int) -> None:
        n_tris = tri_end - tri_start
        if n_tris <= 0:
            return
        batch_size = min(4096, n_tris)
        for batch_start in range(tri_start, tri_end, batch_size):
            batch_end = min(batch_start + batch_size, tri_end)
            glBegin(GL_TRIANGLES)
            for t in range(batch_start, batch_end):
                i0 = int(indices[t * 3 + 0])
                i1 = int(indices[t * 3 + 1])
                i2 = int(indices[t * 3 + 2])
                if has_normals:
                    glNormal3f(float(normals[i0][0]), float(normals[i0][1]), float(normals[i0][2]))
                glVertex3f(float(positions[i0][0]), float(positions[i0][1]), float(positions[i0][2]))
                if has_normals:
                    glNormal3f(float(normals[i1][0]), float(normals[i1][1]), float(normals[i1][2]))
                glVertex3f(float(positions[i1][0]), float(positions[i1][1]), float(positions[i1][2]))
                if has_normals:
                    glNormal3f(float(normals[i2][0]), float(normals[i2][1]), float(normals[i2][2]))
                glVertex3f(float(positions[i2][0]), float(positions[i2][1]), float(positions[i2][2]))
            glEnd()


# Need GLfloat for the matrix array
from ctypes import c_float as GLfloat


def _display_normals(node):
    """Display-only normal buffer for a scene node (graphics sprint).

    SDF nodes: normals from the EXACT field gradient sampled at every
    mesh vertex — n = normalize(node.body.sample(p).gradient), computed
    in C++ by CAD_0.sdf.display_normals. The gradient of a CSG min is
    piecewise: smooth across curved regions (the hole wall shades as a
    true cylinder) and it picks ONE operand's direction at a live edge —
    the box/hole crease stays sharp instead of smearing into a dark
    band. Degenerate vertices (|gradient| ~ 0) keep the mesh normal; the
    buffer never contains NaN.

    Mesh nodes (imported geometry): the mesh's own normals — there is no
    field to sample (ADR-0002 honest dispatch on node.kind).

    DISPLAY PIPELINE ONLY: the MC mesh itself (positions, indices, mesh
    normals) is untouched here — the exporters, the bridge and the
    golden files consume mesh.normals and stay byte-identical.
    """
    mesh = node.cached_mesh
    pos = mesh.positions
    if pos is None or len(pos) == 0:
        return mesh.normals
    if getattr(node, "kind", "sdf") != "sdf":
        # Imported geometry: its normals ARE the data.
        return mesh.normals
    try:
        import CAD_0
        compute = CAD_0.sdf.display_normals
    except (ImportError, AttributeError):
        # Kernel absent (fallback mode) or a stale extension build
        # predating display_normals: keep the app alive with mesh
        # normals and SAY SO — a silent shading downgrade would hide a
        # deployment bug.
        print("[viewport] CAD_0.sdf.display_normals unavailable "
              "(no kernel or stale extension build) — "
              "falling back to mesh normals")
        return mesh.normals
    return compute(node.body, pos, mesh.normals)
