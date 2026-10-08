"""CAD_0.ui.viewport_widget — PySide6 viewport with real OpenGL rendering.

Renders the scene's tessellated meshes using OpenGL 2.1 compatibility
profile (fixed-function pipeline).
"""

from __future__ import annotations

import sys
import math
import os
from typing import Optional, Any

import numpy as np
from PySide6.QtCore import Qt, QPoint, Signal
from PySide6.QtGui import QSurfaceFormat, QMouseEvent, QWheelEvent, QOpenGLFunctions
from PySide6.QtOpenGL import QOpenGLWindow
from PySide6.QtWidgets import QWidget, QVBoxLayout

# OpenGL constants — PySide6.QtGui.QOpenGLFunctions doesn't expose them
# as attributes, so we import them from PySide6.QtOpenGL or use PyOpenGL.
# Fallback: define them manually (standard OpenGL 2.1 values).
try:
    from OpenGL.GL import *  # noqa: F401,F403
    _HAS_PYOPENGL = True
except ImportError:
    _HAS_PYOPENGL = False

# OpenGL constant values (from the OpenGL spec)
GL_DEPTH_TEST = 0x0B71
GL_LEQUAL = 0x0203
GL_LIGHTING = 0x0B50
GL_LIGHT0 = 0x4000
GL_COLOR_MATERIAL = 0x0B57
GL_NORMALIZE = 0x0BA1
GL_COLOR_BUFFER_BIT = 0x4000
GL_DEPTH_BUFFER_BIT = 0x0100
GL_PROJECTION = 0x1701
GL_MODELVIEW = 0x1700
GL_LINES = 0x0001
GL_TRIANGLES = 0x0004
GL_POSITION = 0x1203
GL_DIFFUSE = 0x1201
GL_AMBIENT = 0x1200


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

        # Mouse tracking
        self._last_mouse_pos: Optional[QPoint] = None
        self._mouse_button = Qt.MouseButton.NoButton
        self._drag_distance = 0

        # Display flags
        self._show_grid = True
        self._show_axes = True

        # Mesh data cache
        self._mesh_positions = None
        self._mesh_normals = None
        self._mesh_indices = None

    def _setup_ui(self) -> None:
        fmt = QSurfaceFormat()
        fmt.setVersion(2, 1)
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

    def _request_update(self) -> None:
        """Trigger a repaint of the GL window."""
        self._gl_window.requestUpdate()

    # ----- Public API -----

    def set_scene(self, scene: Any) -> None:
        self._scene = scene
        self._refresh_meshes()

    def _refresh_meshes(self) -> None:
        if self._scene is None:
            return
        if hasattr(self._scene, 'update_meshes'):
            self._scene.update_meshes(48)
        positions = []
        normals = []
        indices = []
        offset = 0
        # Use node_count() + node_at() instead of nodes() to avoid
        # vector copy issues with move-only SceneNode.
        count = self._scene.node_count()
        for i in range(count):
            node = self._scene.node_at(i)
            if node is None or not node.visible:
                continue
            pos = node.cached_mesh.positions
            nor = node.cached_mesh.normals
            idx = node.cached_mesh.indices
            if pos is None or len(pos) == 0 or idx is None or len(idx) == 0:
                continue
            positions.append(pos)
            if nor is not None and len(nor) == len(pos):
                normals.append(nor)
            else:
                normals.append(np.zeros_like(pos))
            indices.append(idx + offset)
            offset += len(pos)
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

    def set_show_grid(self, show: bool) -> None:
        self._show_grid = show
        self._request_update()

    def set_show_axes(self, show: bool) -> None:
        self._show_axes = show
        self._request_update()

    # ----- Input handlers -----

    def mousePressEvent(self, event: QMouseEvent) -> None:
        self._last_mouse_pos = event.position().toPoint()
        self._mouse_button = event.button()
        self._drag_distance = 0
        self.setFocus()

    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        if self._last_mouse_pos is None:
            return
        delta = event.position().toPoint() - self._last_mouse_pos
        self._last_mouse_pos = event.position().toPoint()
        self._drag_distance += abs(delta.x()) + abs(delta.y())

        if self._mouse_button == Qt.MouseButton.LeftButton:
            sensitivity = 0.005
            self._cam_yaw += delta.x() * sensitivity
            self._cam_pitch -= delta.y() * sensitivity
            self._cam_pitch = max(-1.5533, min(1.5533, self._cam_pitch))
            self._request_update()
        elif self._mouse_button == Qt.MouseButton.MiddleButton:
            sensitivity = 0.01 * self._cam_distance
            tx, ty, tz = self._cam_target
            tx -= delta.x() * sensitivity
            ty += delta.y() * sensitivity
            self._cam_target = (tx, ty, tz)
            self._request_update()

    def mouseReleaseEvent(self, event: QMouseEvent) -> None:
        if (self._mouse_button == Qt.MouseButton.LeftButton and
            self._drag_distance < 5 and event.button() == Qt.MouseButton.LeftButton):
            pos = event.position().toPoint()
            w = max(1, self.width())
            h = max(1, self.height())
            ndc_x = (2.0 * pos.x() / w) - 1.0
            ndc_y = 1.0 - (2.0 * pos.y() / h)
            self.clicked.emit(ndc_x, ndc_y)
        self._last_mouse_pos = None
        self._mouse_button = Qt.MouseButton.NoButton

    def wheelEvent(self, event: QWheelEvent) -> None:
        notches = event.angleDelta().y() / 120.0
        factor = 0.9 if notches > 0 else 1.1
        self._cam_distance *= factor ** abs(notches)
        self._cam_distance = max(0.001, min(1e6, self._cam_distance))
        self._request_update()


class _ViewportWindow(QOpenGLWindow):
    """The actual QOpenGLWindow that renders the scene."""

    def __init__(self, parent_widget: ViewportWidget) -> None:
        super().__init__()
        self._parent = parent_widget
        self._gl = None

    def initializeGL(self) -> None:
        self._gl = QOpenGLFunctions()
        self._gl.initializeOpenGLFunctions()
        gl = self._gl
        gl.glEnable(GL_DEPTH_TEST)
        gl.glEnable(GL_LIGHTING)
        gl.glEnable(GL_LIGHT0)
        gl.glEnable(GL_COLOR_MATERIAL)
        gl.glEnable(GL_NORMALIZE)

    def resizeGL(self, w: int, h: int) -> None:
        self._parent._cam_aspect = w / max(1, h)
        if self._gl:
            self._gl.glViewport(0, 0, w, h)

    def paintGL(self) -> None:
        if self._gl is None:
            return
        gl = self._gl
        gl.glClearColor(0.15, 0.15, 0.18, 1.0)
        gl.glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        # Projection
        gl.glMatrixMode(GL_PROJECTION)
        gl.glLoadIdentity()
        fov_y = self._parent._cam_fov
        aspect = self._parent._cam_aspect
        near_p = 0.01
        far_p = 1000.0
        top = near_p * math.tan(math.radians(fov_y) * 0.5)
        bottom = -top
        right = top * aspect
        left = -right
        gl.glFrustum(left, right, bottom, top, near_p, far_p)

        # View (look-at)
        gl.glMatrixMode(GL_MODELVIEW)
        gl.glLoadIdentity()

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

        m = [
            rx, ux, -fx, 0,
            ry, uy, -fy, 0,
            rz, uz, -fz, 0,
            0, 0, 0, 1
        ]
        gl.glMultMatrixf(m)
        gl.glTranslatef(-cam_pos[0], -cam_pos[1], -cam_pos[2])

        # Light
        gl.glLightfv(GL_LIGHT0, GL_POSITION, [0.5, 0.8, 0.3, 0.0])
        gl.glLightfv(GL_LIGHT0, GL_DIFFUSE, [0.8, 0.8, 0.8, 1.0])
        gl.glLightfv(GL_LIGHT0, GL_AMBIENT, [0.2, 0.2, 0.2, 1.0])

        # Grid
        if self._parent._show_grid:
            self._draw_grid(gl)
        # Axes
        if self._parent._show_axes:
            self._draw_axes(gl)
        # Meshes
        if self._parent._mesh_positions is not None and self._parent._mesh_indices is not None:
            self._draw_meshes(gl)

    def _draw_grid(self, gl) -> None:
        gl.glDisable(GL_LIGHTING)
        gl.glColor3f(0.3, 0.3, 0.3)
        gl.glBegin(GL_LINES)
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
        gl.glEnable(GL_LIGHTING)

    def _draw_axes(self, gl) -> None:
        gl.glDisable(GL_LIGHTING)
        gl.glLineWidth(2.0)
        gl.glColor3f(1, 0, 0)
        gl.glBegin(GL_LINES)
        gl.glVertex3f(0, 0, 0)
        gl.glVertex3f(2, 0, 0)
        gl.glEnd()
        gl.glColor3f(0, 1, 0)
        gl.glBegin(GL_LINES)
        gl.glVertex3f(0, 0, 0)
        gl.glVertex3f(0, 2, 0)
        gl.glEnd()
        gl.glColor3f(0, 0, 1)
        gl.glBegin(GL_LINES)
        gl.glVertex3f(0, 0, 0)
        gl.glVertex3f(0, 0, 2)
        gl.glEnd()
        gl.glLineWidth(1.0)
        gl.glEnable(GL_LIGHTING)

    def _draw_meshes(self, gl) -> None:
        positions = self._parent._mesh_positions
        normals = self._parent._mesh_normals
        indices = self._parent._mesh_indices
        if positions is None or len(positions) == 0:
            return
        gl.glColor3f(0.7, 0.75, 0.8)
        has_normals = normals is not None and len(normals) == len(positions)
        n_tris = len(indices) // 3
        batch_size = min(4096, n_tris)
        for batch_start in range(0, n_tris, batch_size):
            batch_end = min(batch_start + batch_size, n_tris)
            gl.glBegin(GL_TRIANGLES)
            for t in range(batch_start, batch_end):
                i0 = int(indices[t * 3 + 0])
                i1 = int(indices[t * 3 + 1])
                i2 = int(indices[t * 3 + 2])
                if has_normals:
                    gl.glNormal3f(float(normals[i0][0]), float(normals[i0][1]), float(normals[i0][2]))
                gl.glVertex3f(float(positions[i0][0]), float(positions[i0][1]), float(positions[i0][2]))
                if has_normals:
                    gl.glNormal3f(float(normals[i1][0]), float(normals[i1][1]), float(normals[i1][2]))
                gl.glVertex3f(float(positions[i1][0]), float(positions[i1][1]), float(positions[i1][2]))
                if has_normals:
                    gl.glNormal3f(float(normals[i2][0]), float(normals[i2][1]), float(normals[i2][2]))
                gl.glVertex3f(float(positions[i2][0]), float(positions[i2][1]), float(positions[i2][2]))
            gl.glEnd()
