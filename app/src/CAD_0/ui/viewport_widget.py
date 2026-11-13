"""CAD_0.ui.viewport_widget — PySide6 viewport with real OpenGL rendering.

Uses PyOpenGL for fixed-function pipeline (glMatrixMode, client
arrays, etc.) and QOpenGLWindow as the rendering surface.

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
    glLightfv, glColor3f,
    glLineWidth,
    GL_DEPTH_TEST, GL_LEQUAL, GL_LIGHTING, GL_LIGHT0,
    GL_COLOR_MATERIAL, GL_NORMALIZE,
    GL_COLOR_BUFFER_BIT, GL_DEPTH_BUFFER_BIT,
    GL_PROJECTION, GL_MODELVIEW,
    GL_LINES, GL_TRIANGLES,
    GL_POSITION, GL_DIFFUSE, GL_AMBIENT,
    glDepthFunc,
    GL_FLOAT, GL_UNSIGNED_INT, GL_UNSIGNED_BYTE,
    GL_VERTEX_ARRAY, GL_NORMAL_ARRAY, GL_COLOR_ARRAY,
    glEnableClientState, glDisableClientState,
    glVertexPointer, glNormalPointer, glColorPointer, glDrawElements,
    glGenBuffers, glBindBuffer, glBufferData, glDeleteBuffers,
    GL_ARRAY_BUFFER, GL_ELEMENT_ARRAY_BUFFER, GL_STATIC_DRAW,
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

        # THE DRAW MIRROR (PERF 1 — the VBO/IBO cache's contract side):
        # one _NodeDrawEntry per VISIBLE feature, holding the node's
        # derived mesh and its digest. This replaces the old whole-scene
        # concatenated buffers (_mesh_positions/_mesh_normals/
        # _mesh_indices): a concatenation is a per-edit rebuild of
        # EVERYTHING — the very courtesy flush the cache exists to
        # refuse. The mirror is per-feature because the cache is.
        self._draw_nodes: list = []
        # Selection highlight state: node id -> (first tri, last tri) of
        # that node's OWN buffer (PERF 1: per-feature buffers — the
        # range is the node's whole buffer, kept as a pair for the
        # non-empty check the picking tests pin), plus the selected
        # ids. Extended selection: EVERY selected body is highlighted
        # (the subtract's target AND tool), while _selected_node_id
        # keeps the anchor (the single-selection surface the picking
        # tests pin).
        self._node_ranges = {}
        self._selected_node_id = None
        self._selected_node_ids: set = set()

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
        """Build the draw mirror: one entry per visible feature.

        The digest is computed HERE, once per refresh (an edit gesture,
        a visibility flip, a scene load — never per frame): the paint
        that follows only compares strings. The per-refresh cost is the
        price of not trusting age, measured and declared (~1 ms/MB on
        the guide-chain meshes, blake2b-64); the per-frame cost is zero.
        """
        if self._scene is None:
            return
        if hasattr(self._scene, 'update_meshes'):
            self._scene.update_meshes(self._tess_resolution)
        entries = []
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
            # Positions/indices are still the node's own mesh: exports
            # and the bridge read those from the node, byte-identically.
            nor = _display_normals(node)
            pos = np.ascontiguousarray(pos, dtype="<f4")
            idx = np.ascontiguousarray(idx, dtype="<u4")
            if nor is not None and len(nor) == len(pos):
                nor = np.ascontiguousarray(nor, dtype="<f4")
            else:
                nor = np.zeros_like(pos, dtype="<f4")
            digest = _display_mesh_digest(pos, nor, idx)
            n_tris = len(idx) // 3
            node_ranges[node.id] = (0, n_tris)
            entries.append(_NodeDrawEntry(node.id, pos, nor, idx, digest))
        self._draw_nodes = entries
        self._node_ranges = node_ranges
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

        ``node_id`` may be a single id, None, or any iterable of ids:
        the extended tree selection passes EVERY selected row, so both
        bodies of a subtract pair light up — the highlight is a view
        of the whole selection, not just its anchor. The anchor stays
        readable as ``_selected_node_id`` (deterministically the
        smallest id when several are selected).
        """
        if node_id is None:
            ids = set()
        elif isinstance(node_id, (list, tuple, set, frozenset)):
            ids = {i for i in node_id if i is not None}
        else:
            ids = {node_id}
        if ids != self._selected_node_ids:
            self._selected_node_ids = ids
            self._selected_node_id = min(ids) if ids else None
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
        # THE VBO/IBO CACHE (PERF 1 — Phase C.6's registered seat, the
        # register's G row): one _NodeGpuBuffers per feature, the
        # digest of the bytes each entry holds. Owned by the window
        # because GL resources live and die with the context.
        self._node_buffers: dict = {}
        # The static overlays (grid + axes) in their own buffers,
        # uploaded ONCE at GL initialization — never per frame.
        self._overlay_buffers = None

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
        # THE CONTEXT-DEATH RULE (PERF 1): this method runs on a FRESH
        # context — if the cache holds entries, their buffer ids were
        # allocated on a context that is now gone, and GL ids do not
        # survive their context. The dead context's resources died
        # with it (there is nothing to glDeleteBuffers on the new
        # one), so the dict is simply reborn empty and the next mesh
        # pass re-uploads every visible feature — the first-sight act
        # again, declared. Serving an id across contexts would draw
        # from a dead name (or another buffer's storage): the digest
        # labels die WITH the ids they certify, never after them.
        if self._node_buffers:
            self._node_buffers = {}
        try:
            glEnable(GL_DEPTH_TEST)
            glDepthFunc(GL_LEQUAL)
            glEnable(GL_LIGHTING)
            glEnable(GL_LIGHT0)
            glEnable(GL_COLOR_MATERIAL)
            glEnable(GL_NORMALIZE)
            self._init_overlay_buffers()
            self._initialized = True
        except Exception as e:
            print(f"[viewport] GL initialization failed: {e}")
            self._initialized = False

    def _init_overlay_buffers(self) -> None:
        """The grid and the axes into their OWN static buffers (PERF 1).

        Before the VBO morso these were the frame's last immediate-mode
        walks — 44 glVertex3f for the grid, 6 more for the axes, every
        frame, forever: bounded (O(1) in scene size) but a per-frame
        vertex upload all the same, and the stable path's contract is
        ZERO. Uploaded once here, at GL initialization: the frame's
        every vertex byte rides a buffer from this day.
        """
        extent = 5.0
        step = 1.0
        lines = []
        i = -extent
        while i <= extent:
            lines.append((i, 0.0, -extent, i, 0.0, extent))
            lines.append((-extent, 0.0, i, extent, 0.0, i))
            i += step
        grid_pos = np.ascontiguousarray(
            np.array(lines, dtype="<f4").reshape(-1, 3))
        grid_idx = np.ascontiguousarray(
            np.arange(len(grid_pos), dtype="<u4"))
        axes_pos = np.ascontiguousarray(
            np.array([(0, 0, 0), (2, 0, 0),
                      (0, 0, 0), (0, 2, 0),
                      (0, 0, 0), (0, 0, 2)], dtype="<f4"))
        # ubyte colors: per-vertex RGB — the color array is the one
        # addition the fixed-function pipeline needs for a single
        # draw call to carry the axes' three colors.
        axes_col = np.ascontiguousarray(
            np.array([(255, 0, 0), (255, 0, 0),
                      (0, 255, 0), (0, 255, 0),
                      (0, 0, 255), (0, 0, 255)], dtype=np.ubyte))
        axes_idx = np.ascontiguousarray(
            np.arange(6, dtype="<u4"))

        class _Overlays:
            __slots__ = ("grid_vbo", "grid_ibo", "axes_vbo",
                         "axes_cbo", "axes_ibo", "grid_count",
                         "axes_count")

        ov = _Overlays()
        ov.grid_vbo, ov.grid_ibo, ov.axes_vbo, ov.axes_cbo, ov.axes_ibo = (
            int(b) for b in glGenBuffers(5))
        ov.grid_count = len(grid_idx)
        ov.axes_count = len(axes_idx)
        glBindBuffer(GL_ARRAY_BUFFER, ov.grid_vbo)
        glBufferData(GL_ARRAY_BUFFER, grid_pos, GL_STATIC_DRAW)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ov.grid_ibo)
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, grid_idx, GL_STATIC_DRAW)
        glBindBuffer(GL_ARRAY_BUFFER, ov.axes_vbo)
        glBufferData(GL_ARRAY_BUFFER, axes_pos, GL_STATIC_DRAW)
        glBindBuffer(GL_ARRAY_BUFFER, ov.axes_cbo)
        glBufferData(GL_ARRAY_BUFFER, axes_col, GL_STATIC_DRAW)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ov.axes_ibo)
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, axes_idx, GL_STATIC_DRAW)
        self._overlay_buffers = ov

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
        if self._parent._draw_nodes:
            self._draw_meshes()

    def _draw_grid(self) -> None:
        if self._overlay_buffers is None:
            return
        ov = self._overlay_buffers
        glDisable(GL_LIGHTING)
        glColor3f(0.3, 0.3, 0.3)
        glEnableClientState(GL_VERTEX_ARRAY)
        glBindBuffer(GL_ARRAY_BUFFER, ov.grid_vbo)
        glVertexPointer(3, GL_FLOAT, 0, None)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ov.grid_ibo)
        glDrawElements(GL_LINES, int(ov.grid_count),
                       GL_UNSIGNED_INT, None)
        glDisableClientState(GL_VERTEX_ARRAY)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0)
        glBindBuffer(GL_ARRAY_BUFFER, 0)
        glEnable(GL_LIGHTING)

    def _draw_axes(self) -> None:
        if self._overlay_buffers is None:
            return
        ov = self._overlay_buffers
        glDisable(GL_LIGHTING)
        glLineWidth(2.0)
        glEnableClientState(GL_VERTEX_ARRAY)
        glEnableClientState(GL_COLOR_ARRAY)
        glBindBuffer(GL_ARRAY_BUFFER, ov.axes_vbo)
        glVertexPointer(3, GL_FLOAT, 0, None)
        glBindBuffer(GL_ARRAY_BUFFER, ov.axes_cbo)
        glColorPointer(3, GL_UNSIGNED_BYTE, 0, None)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ov.axes_ibo)
        glDrawElements(GL_LINES, int(ov.axes_count),
                       GL_UNSIGNED_INT, None)
        glDisableClientState(GL_COLOR_ARRAY)
        glDisableClientState(GL_VERTEX_ARRAY)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0)
        glBindBuffer(GL_ARRAY_BUFFER, 0)
        glLineWidth(1.0)
        glEnable(GL_LIGHTING)

    def _draw_meshes(self) -> None:
        """The mesh pass: ensure the buffers, then draw one range per
        feature (the m6-correttiva grammar riding PERF 1's buffers)."""
        entries = self._parent._draw_nodes
        if not entries:
            self._drop_absent_buffers(set())
            return
        live = {}
        glColor3f(0.7, 0.75, 0.8)
        # The array states are the mesh pass's own (the grid and the
        # axes manage theirs); the pointers ride the per-range binds
        # below — the m6-correttiva plumbing, counted per range.
        glEnableClientState(GL_VERTEX_ARRAY)
        glEnableClientState(GL_NORMAL_ARRAY)
        for entry in entries:
            bufs = self._ensure_node_buffers(entry)
            live[entry.node_id] = bufs
            self._draw_tri_range(bufs, 0, entry.triangle_count)
        # Selection highlight (visible feedback for "Click on shape |
        # Select it"): repaint the SELECTED nodes' triangles in amber,
        # on top of the base pass — every selected body of an extended
        # selection, not just one (both operands of a subtract pair
        # light up). Same buffers, so the depth test (LEQUAL) lets the
        # repaint win exactly where the shape is visible — no
        # z-fighting, no halo.
        selected = self._parent._selected_node_ids
        if selected:
            glColor3f(1.0, 0.62, 0.10)
            for sel in sorted(selected):
                bufs = live.get(sel)
                if bufs is not None and bufs.triangle_count > 0:
                    self._draw_tri_range(bufs, 0, bufs.triangle_count)
        glDisableClientState(GL_NORMAL_ARRAY)
        glDisableClientState(GL_VERTEX_ARRAY)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0)
        glBindBuffer(GL_ARRAY_BUFFER, 0)
        # The nodes that left the mirror hand their buffers back (the
        # removal rule — see _drop_absent_buffers).
        self._drop_absent_buffers(set(live))

    def _ensure_node_buffers(self, entry: "_NodeDrawEntry") -> "_NodeGpuBuffers":
        """THE INVALIDATION CONTRACT (PERF 1) — every rule named, with
        its perché:

        1. THE DIGEST IS THE ONLY KEY. A feature's buffers are reused
           iff the digest of the mesh they HOLD equals the digest of
           the mesh the mirror DECLARES — never age, never the scene's
           dirty flag, never object identity. The comparison is at the
           point of service (here), so a stale buffer cannot be drawn
           without the digest having certified the bytes in it.

        2. AN EDIT TOUCHES ONE FEATURE -> ONE FEATURE RE-UPLOADS. The
           scene's own dirty-flag discipline re-tessellates only the
           touched features (and the declared dependents — the operand
           edit that invalidates a consumer re-derives the consumer's
           mesh); untouched features keep byte-identical meshes, their
           digests match, their buffers are served with ZERO upload.
           There is NO whole-scene flush here — per-feature buffers ARE
           the granularity, and a courtesy flush is what this cache
           exists to refuse.

        3. THE UPLOAD IS THE EXCEPTION, NOT THE RULE. glBufferData runs
           exactly when a digest differs (a re-derived mesh, a first
           sight) — the stable path (orbit, selection, visibility —
           any frame that changes no mesh bytes) uploads NOTHING.

        Why glBufferData and not glBufferSubData: a re-derived mesh can
        change SIZE (a fillet at a new radius, a resolution bump);
        glBufferData re-specifies the storage wholesale — one call, the
        old bytes gone, the entry's digest replaced by the mesh's own.
        """
        cached = self._node_buffers.get(entry.node_id)
        if cached is not None and cached.digest == entry.digest:
            # The bytes in these buffers ARE the mesh's — verified by
            # the digest, the only witness the cache trusts.
            return cached
        if cached is None:
            pos_buf, nor_buf, idx_buf = (int(b) for b in glGenBuffers(3))
            cached = _NodeGpuBuffers(pos_buf, nor_buf, idx_buf)
            self._node_buffers[entry.node_id] = cached
        # (Re)specify the three buffers with the entry's own arrays.
        glBindBuffer(GL_ARRAY_BUFFER, cached.pos_buf)
        glBufferData(GL_ARRAY_BUFFER, entry.positions, GL_STATIC_DRAW)
        glBindBuffer(GL_ARRAY_BUFFER, cached.nor_buf)
        glBufferData(GL_ARRAY_BUFFER, entry.normals, GL_STATIC_DRAW)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cached.idx_buf)
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, entry.indices,
                     GL_STATIC_DRAW)
        cached.digest = entry.digest
        cached.vertex_count = entry.vertex_count
        cached.triangle_count = entry.triangle_count
        return cached

    def _drop_absent_buffers(self, live_ids: set) -> None:
        """THE REMOVAL RULE: a node that left the mirror (hidden,
        deleted, the scene replaced) hands its buffers back —
        glDeleteBuffers at the next mesh pass, and ONLY the absent
        ones. GPU memory is the scene's own currency: a cache that
        hoards dead features' buffers is a leak wearing a cache's
        name."""
        absent = [nid for nid in self._node_buffers if nid not in live_ids]
        for nid in absent:
            bufs = self._node_buffers.pop(nid)
            ids = np.ascontiguousarray(
                [bufs.pos_buf, bufs.nor_buf, bufs.idx_buf],
                dtype=np.uint32)
            glDeleteBuffers(len(ids), ids)

    @staticmethod
    def _draw_tri_range(bufs: "_NodeGpuBuffers", tri_start: int,
                        tri_end: int) -> None:
        """ONE draw call per range (G2-u3 m6-correttiva's grammar,
        riding PERF 1's buffers): the range's triangles drawn from the
        feature's OWN VBO/IBO — the vertex and normal pointers read the
        buffer bound at pointer time, the indices the IBO bound at
        draw time, the range's first index at its byte offset. Zero
        per-vertex calls, zero per-frame uploads: the geometry was
        uploaded once at its digest's first sight (see
        _ensure_node_buffers — the m6-correttiva profile's own cause,
        the per-vertex walk, stays dead; the per-frame implicit upload
        of client-side arrays dies with it). The pointer-per-range
        plumbing is the GL 2.1 compatibility road's own grammar —
        counted per range, never per vertex (no VAO: that is GL 3.0's
        road and the widget's context is 2.1 — the declared bound).
        """
        n_tris = tri_end - tri_start
        if n_tris <= 0 or bufs is None:
            return
        glBindBuffer(GL_ARRAY_BUFFER, bufs.pos_buf)
        glVertexPointer(3, GL_FLOAT, 0, None)
        glBindBuffer(GL_ARRAY_BUFFER, bufs.nor_buf)
        glNormalPointer(GL_FLOAT, 0, None)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, bufs.idx_buf)
        glDrawElements(GL_TRIANGLES, int(n_tris * 3),
                       GL_UNSIGNED_INT, int(tri_start * 3 * 4))


# Need GLfloat for the matrix array
from ctypes import c_float as GLfloat


# ===========================================================================
# PERF 1 — THE DISPLAY MESH'S IDENTITY (the chain digest's own model,
# applied to the display payload)
# ===========================================================================
#
# The VBO/IBO cache (below) does not trust AGE — a buffer is not "fresh
# enough" — it trusts the IDENTITY of the derived mesh: a blake2b-64
# over the exact bytes a VBO would upload (positions, display normals,
# indices), the same one-comparable-value contract the chain digests
# hold for the kernel's bodies (chain_digest: "the whole body as one
# comparable value" — here the whole DISPLAY mesh as one comparable
# value). Why blake2b and not the house FNV-1a: the chain digests are
# rendered in C++ over strings the kernel already walks; this digest
# runs in the refresh path over megabytes of float payload, where a
# byte-sequential FNV in Python would cost seconds — blake2b is the
# standard library's own C-speed instrument (~1 ms/MB, measured on the
# guide-chain meshes), a DIFFERENT instrument with a DIFFERENT seat,
# named as such. The bytes are DECLARED little-endian ('<f4'/'<u4') so
# the digest is well-defined on every machine that can build the
# module; the digest never leaves the process — it is a cache key and
# a proof of identity, never a serialized value.

def _display_mesh_digest(positions, normals, indices) -> str:
    """blake2b-64 hex over the display payload's declared-LE bytes.

    Any change in the mesh — one float32 in one vertex, one index —
    moves the digest; an unchanged mesh answers the same digest on
    every call, every machine, every run.
    """
    import hashlib
    h = hashlib.blake2b(digest_size=8)
    for part, dtype in ((positions, "<f4"), (normals, "<f4"),
                        (indices, "<u4")):
        # ascontiguousarray with an LE dtype: a no-op view on every
        # machine this module runs on, an honest conversion on one
        # that is not — the bytes are declared, never assumed.
        flat = np.ascontiguousarray(part, dtype=dtype).ravel()
        h.update(flat.view(np.uint8))
    return h.hexdigest()


class _NodeDrawEntry:
    """One visible feature's display mesh, mirrored for the draw.

    Built by _refresh_meshes (the wrapper — no GL): the arrays are the
    node's own derived mesh (positions/indices the scene's cache, the
    display normals the field's), typed contiguous, with the mesh's
    digest. The paint uploads NOTHING the mirror did not declare.
    """

    __slots__ = ("node_id", "positions", "normals", "indices",
                 "digest", "vertex_count", "triangle_count")

    def __init__(self, node_id, positions, normals, indices, digest):
        self.node_id = node_id
        self.positions = positions
        self.normals = normals
        self.indices = indices
        self.digest = digest
        self.vertex_count = len(positions)
        self.triangle_count = len(indices) // 3


class _NodeGpuBuffers:
    """The GL buffer objects holding ONE feature's display mesh, with
    the digest of the bytes they hold — the cache's own entry."""

    __slots__ = ("pos_buf", "nor_buf", "idx_buf", "digest",
                 "vertex_count", "triangle_count")

    def __init__(self, pos_buf, nor_buf, idx_buf):
        self.pos_buf = pos_buf
        self.nor_buf = nor_buf
        self.idx_buf = idx_buf
        self.digest = None          # set at the first upload
        self.vertex_count = 0
        self.triangle_count = 0


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
