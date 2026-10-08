"""CAD_0.ui.viewport_widget — PySide6 viewport with real OpenGL rendering.

This version renders the scene's tessellated meshes using OpenGL 2.1
compatibility profile (fixed-function pipeline). It works with any GPU
that supports OpenGL 2.1+, without needing GLAD or GLEW.

Phase C.6+: the viewport now shows actual geometry from the Scene.
"""

from __future__ import annotations

from typing import Optional, Any

from PySide6.QtCore import Qt, QPoint, Signal
from PySide6.QtGui import QSurfaceFormat, QMouseEvent, QWheelEvent
from PySide6.QtOpenGL import QOpenGLWindow
from PySide6.QtWidgets import QWidget, QVBoxLayout

# Try to import the C++ kernel — if not available, we fall back to
# pure-Python camera state (the viewport still opens but shows nothing).
try:
    import CAD_0._CAD_0 as _cpp
    _HAS_CPP = True
except ImportError:
    _HAS_CPP = False


class ViewportWidget(QWidget):
    """A QWidget that hosts an OpenGL viewport with real rendering."""

    # Emitted when the user clicks in the viewport (for picking).
    clicked = Signal(float, float)  # NDC x, y

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._scene = None
        self._setup_ui()
        self._setup_input_handlers()

        # Camera state (Python-side; mirrors OrbitCamera).
        self._cam_target = (0.0, 0.0, 0.0)
        self._cam_distance = 5.0
        self._cam_yaw = 0.0
        self._cam_pitch = 0.3
        self._cam_fov = 45.0
        self._cam_aspect = 1.0

        # Mouse tracking state.
        self._last_mouse_pos: Optional[QPoint] = None
        self._mouse_button = Qt.MouseButton.NoButton

        # Show grid/axes flags.
        self._show_grid = True
        self._show_axes = True

        # Mesh data cache (set from outside).
        self._mesh_positions = None
        self._mesh_normals = None
        self._mesh_indices = None

    def _setup_ui(self) -> None:
        fmt = QSurfaceFormat()
        fmt.setVersion(2, 1)  # Use OpenGL 2.1 compatibility profile
        fmt.setProfile(QSurfaceFormat.CompatibilityProfile)
        fmt.setDepthBufferSize(24)
        fmt.setSwapBehavior(QSurfaceFormat.DoubleBuffer)
        QSurfaceFormat.setDefaultFormat(fmt)

        self._gl_window = _ViewportWindow(self)
        self._gl_window.setFormat(fmt)
        container = QWidget.createWindowContainer(self._gl_window, self)
        container.setFocusPolicy(Qt.StrongFocus)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(container)
        self.setLayout(layout)

    def _setup_input_handlers(self) -> None:
        pass

    # ----- Public API -----

    def set_scene(self, scene: Any) -> None:
        """Set the CAD_0 Scene object to render."""
        self._scene = scene
        self._refresh_meshes()

    def _refresh_meshes(self) -> None:
        """Re-tessellate dirty meshes in the scene."""
        if self._scene is None:
            return
        # Update meshes in the kernel
        if hasattr(self._scene, 'update_meshes'):
            self._scene.update_meshes(48)
        # Collect all visible mesh data
        positions = []
        normals = []
        indices = []
        offset = 0
        for node in self._scene.nodes():
            if not node.visible or node.cached_mesh.positions is None:
                continue
            pos = node.cached_mesh.positions
            nor = node.cached_mesh.normals
            idx = node.cached_mesh.indices
            if len(pos) == 0 or len(idx) == 0:
                continue
            import numpy as np
            positions.append(pos)
            if nor is not None and len(nor) == len(pos):
                normals.append(nor)
            else:
                normals.append(np.zeros_like(pos))
            # Offset indices
            idx_offset = idx + offset
            indices.append(idx_offset)
            offset += len(pos)
        if positions:
            import numpy as np
            self._mesh_positions = np.concatenate(positions)
            self._mesh_normals = np.concatenate(normals)
            self._mesh_indices = np.concatenate(indices)
        else:
            self._mesh_positions = None
            self._mesh_indices = None
        self._gl_window.request_update()

    def set_camera_target(self, x: float, y: float, z: float) -> None:
        self._cam_target = (x, y, z)
        self._gl_window.request_update()

    def set_camera_distance(self, d: float) -> None:
        self._cam_distance = max(0.001, d)
        self._gl_window.request_update()

    def set_camera_yaw_pitch(self, yaw: float, pitch: float) -> None:
        self._cam_yaw = yaw
        self._cam_pitch = max(-1.5533, min(1.5533, pitch))
        self._gl_window.request_update()

    def frame_all(self) -> None:
        """Auto-frame the scene bounds."""
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
        self._gl_window.request_update()

    def set_show_grid(self, show: bool) -> None:
        self._show_grid = show
        self._gl_window.request_update()

    def set_show_axes(self, show: bool) -> None:
        self._show_axes = show
        self._gl_window.request_update()

    # ----- Input handlers -----

    def mousePressEvent(self, event: QMouseEvent) -> None:
        self._last_mouse_pos = event.position().toPoint()
        self._mouse_button = event.button()
        self.setFocus()

    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        if self._last_mouse_pos is None:
            return
        delta = event.position().toPoint() - self._last_mouse_pos
        self._last_mouse_pos = event.position().toPoint()

        if self._mouse_button == Qt.MouseButton.LeftButton:
            sensitivity = 0.005
            self._cam_yaw += delta.x() * sensitivity
            self._cam_pitch -= delta.y() * sensitivity
            self._cam_pitch = max(-1.5533, min(1.5533, self._cam_pitch))
            self._gl_window.request_update()
        elif self._mouse_button == Qt.MouseButton.MiddleButton:
            sensitivity = 0.01 * self._cam_distance
            tx, ty, tz = self._cam_target
            tx -= delta.x() * sensitivity
            ty += delta.y() * sensitivity
            self._cam_target = (tx, ty, tz)
            self._gl_window.request_update()

    def mouseReleaseEvent(self, event: QMouseEvent) -> None:
        self._last_mouse_pos = None
        self._mouse_button = Qt.MouseButton.NoButton

    def wheelEvent(self, event: QWheelEvent) -> None:
        notches = event.angleDelta().y() / 120.0
        factor = 0.9 if notches > 0 else 1.1
        self._cam_distance *= factor ** abs(notches)
        self._cam_distance = max(0.001, min(1e6, self._cam_distance))
        self._gl_window.request_update()

    def _get_viewport_widget(self):
        """Return self so the GL window can access camera/mesh state."""
        return self


class _ViewportWindow(QOpenGLWindow):
    """The actual QOpenGLWindow that renders the scene."""

    def __init__(self, parent_widget: ViewportWidget) -> None:
        super().__init__()
        self._parent = parent_widget

    def initializeGL(self) -> None:
        from PySide6.QtGui import QOpenGLFunctions
        self._gl = QOpenGLFunctions(self)
        self._gl.initializeOpenGLFunctions()
        self._gl.glEnable(self._gl.GL_DEPTH_TEST)
        self._gl.glEnable(self._gl.GL_LIGHTING)
        self._gl.glEnable(self._gl.GL_LIGHT0)
        self._gl.glEnable(self._gl.GL_COLOR_MATERIAL)
        self._gl.glEnable(self._gl.GL_NORMALIZE)

    def resizeGL(self, w: int, h: int) -> None:
        self._parent._cam_aspect = w / max(1, h)
        self._gl.glViewport(0, 0, w, h)

    def paintGL(self) -> None:
        gl = self._gl
        gl.glClearColor(0.15, 0.15, 0.18, 1.0)
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)

        # Set up projection
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glLoadIdentity()
        import math
        fov_y = self._parent._cam_fov
        aspect = self._parent._cam_aspect
        near_p = 0.01
        far_p = 1000.0
        f = 1.0 / math.tan(math.radians(fov_y) * 0.5)
        # glFrustum(left, right, bottom, top, near, far)
        top = near_p * math.tan(math.radians(fov_y) * 0.5)
        bottom = -top
        right = top * aspect
        left = -right
        gl.glFrustum(left, right, bottom, top, near_p, far_p)

        # Set up view (look-at)
        gl.glMatrixMode(gl.GL_MODELVIEW)
        gl.glLoadIdentity()

        # Camera position from spherical coords
        import math
        cp = math.cos(self._parent._cam_pitch)
        sp = math.sin(self._parent._cam_pitch)
        cy = math.cos(self._parent._cam_yaw)
        sy = math.sin(self._parent._cam_yaw)
        cam_dir = (cp * sy, sp, cp * cy)
        tx, ty, tz = self._parent._cam_target
        cam_pos = (tx + cam_dir[0] * self._parent._cam_distance,
                   ty + cam_dir[1] * self._parent._cam_distance,
                   tz + cam_dir[2] * self._parent._cam_distance)

        # gluLookAt(eye, center, up)
        # Implemented manually via glMultMatrix
        fx = tx - cam_pos[0]
        fy = ty - cam_pos[1]
        fz = tz - cam_pos[2]
        # Normalize forward
        fl = math.sqrt(fx*fx + fy*fy + fz*fz)
        if fl < 1e-9:
            fx, fy, fz = 0, 0, -1
        else:
            fx, fy, fz = fx/fl, fy/fl, fz/fl
        # Up = (0,1,0)
        # Right = forward x up
        rx = fy * 0 - fz * 1  # = -fz
        ry = fz * 0 - fx * 0  # = 0
        rz = fx * 1 - fy * 0  # = fx
        rl = math.sqrt(rx*rx + ry*ry + rz*rz)
        if rl > 1e-9:
            rx, ry, rz = rx/rl, ry/rl, rz/rl
        else:
            rx, ry, rz = 1, 0, 0
        # Recompute up = right x forward
        ux = ry * fz - rz * fy
        uy = rz * fx - rx * fz
        uz = rx * fy - ry * fx

        m = [
            rx, ux, -fx, 0,
            ry, uy, -fy, 0,
            rz, uz, -fz, 0,
            0, 0, 0, 1
        ]
        gl.glMultMatrixf(m)
        gl.glTranslatef(-cam_pos[0], -cam_pos[1], -cam_pos[2])

        # Set up light
        gl.glLightfv(gl.GL_LIGHT0, gl.GL_POSITION,
                     [0.5, 0.8, 0.3, 0.0])
        gl.glLightfv(gl.GL_LIGHT0, gl.GL_DIFFUSE,
                     [0.8, 0.8, 0.8, 1.0])
        gl.glLightfv(gl.GL_LIGHT0, gl.GL_AMBIENT,
                     [0.2, 0.2, 0.2, 1.0])

        # Draw grid
        if self._parent._show_grid:
            self._draw_grid(gl)

        # Draw axes
        if self._parent._show_axes:
            self._draw_axes(gl)

        # Draw meshes
        if self._parent._mesh_positions is not None and self._parent._mesh_indices is not None:
            self._draw_meshes(gl)

    def _draw_grid(self, gl) -> None:
        gl.glDisable(gl.GL_LIGHTING)
        gl.glColor3f(0.3, 0.3, 0.3)
        gl.glBegin(gl.GL_LINES)
        extent = 5.0
        step = 1.0
        i = -extent
        while i <= extent:
            gl.glVertex3f(i, 0, -extent)
            gl.glVertex3f(i, 0, extent)
            gl.glVertex3f(-extent, 0, i)
            gl.glVertex3f(extent, 0, i)
            i += step
        gl.glEnd()
        gl.glEnable(gl.GL_LIGHTING)

    def _draw_axes(self, gl) -> None:
        gl.glDisable(gl.GL_LIGHTING)
        gl.glLineWidth(2.0)
        # X axis - red
        gl.glColor3f(1, 0, 0)
        gl.glBegin(gl.GL_LINES)
        gl.glVertex3f(0, 0, 0)
        gl.glVertex3f(2, 0, 0)
        gl.glEnd()
        # Y axis - green
        gl.glColor3f(0, 1, 0)
        gl.glBegin(gl.GL_LINES)
        gl.glVertex3f(0, 0, 0)
        gl.glVertex3f(0, 2, 0)
        gl.glEnd()
        # Z axis - blue
        gl.glColor3f(0, 0, 1)
        gl.glBegin(gl.GL_LINES)
        gl.glVertex3f(0, 0, 0)
        gl.glVertex3f(0, 0, 2)
        gl.glEnd()
        gl.glLineWidth(1.0)
        gl.glEnable(gl.GL_LIGHTING)

    def _draw_meshes(self, gl) -> None:
        import numpy as np
        positions = self._parent._mesh_positions
        normals = self._parent._mesh_normals
        indices = self._parent._mesh_indices

        if positions is None or len(positions) == 0:
            return

        # Draw the mesh using glBegin/glEnd (simple, no VBO needed)
        gl.glColor3f(0.7, 0.75, 0.8)

        # If we have normals, use them for lighting
        has_normals = normals is not None and len(normals) == len(positions)

        n_tris = len(indices) // 3
        # Batch in groups to avoid too many calls
        batch_size = min(4096, n_tris)
        for batch_start in range(0, n_tris, batch_size):
            batch_end = min(batch_start + batch_size, n_tris)
            gl.glBegin(gl.GL_TRIANGLES)
            for t in range(batch_start, batch_end):
                i0 = indices[t * 3 + 0]
                i1 = indices[t * 3 + 1]
                i2 = indices[t * 3 + 2]
                if has_normals:
                    gl.glNormal3f(normals[i0][0], normals[i0][1], normals[i0][2])
                gl.glVertex3f(positions[i0][0], positions[i0][1], positions[i0][2])
                if has_normals:
                    gl.glNormal3f(normals[i1][0], normals[i1][1], normals[i1][2])
                gl.glVertex3f(positions[i1][0], positions[i1][1], positions[i1][2])
                if has_normals:
                    gl.glNormal3f(normals[i2][0], normals[i2][1], normals[i2][2])
                gl.glVertex3f(positions[i2][0], positions[i2][1], positions[i2][2])
            gl.glEnd()
