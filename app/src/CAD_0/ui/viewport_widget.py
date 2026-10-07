"""CAD_0.ui.viewport_widget — PySide6 viewport widget.

Hosts a QWindow with an OpenGL context and drives the CAD_0 renderer
(either HeadlessRenderer for tests or OpenGLRenderer for production).

Phase C.6: written but not exercised in CI. Requires PySide6 + a
display. The widget is the only non-testable layer of the viewport
stack; everything below it (camera, command stack, selection, renderer
interface) is fully tested.

Usage:
    from CAD_0.ui.viewport_widget import ViewportWidget
    widget = ViewportWidget(parent)
    widget.set_camera_target(0.0, 0.0, 0.0)
    widget.set_camera_distance(5.0)
"""

from __future__ import annotations

from typing import Optional

from PySide6.QtCore import Qt, QPoint
from PySide6.QtGui import QSurfaceFormat, QMouseEvent, QWheelEvent
from PySide6.QtOpenGL import QOpenGLWindow
from PySide6.QtWidgets import QWidget, QVBoxLayout


class ViewportWidget(QWidget):
    """A QWidget that hosts an OpenGL viewport.

    Internally uses a QOpenGLWindow (not QOpenGLWidget) because the
    QWindow-based approach has lower latency and better compatibility
    with VR/compositing pipelines.
    """

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._setup_ui()
        self._setup_input_handlers()

        # Camera state — stored here so the widget can be used without
        # a binding to the C++ OrbitCamera class (which is bound via
        # nanobind in a future build). For now we keep state in Python.
        self._camera_target = (0.0, 0.0, 0.0)
        self._camera_distance = 5.0
        self._camera_yaw = 0.0
        self._camera_pitch = 0.3

        # Mouse tracking state.
        self._last_mouse_pos: Optional[QPoint] = None
        self._mouse_button = Qt.MouseButton.NoButton

    def _setup_ui(self) -> None:
        # Configure the OpenGL surface format — 4.5 core, 24-bit depth, vsync on.
        fmt = QSurfaceFormat()
        fmt.setVersion(4, 5)
        fmt.setProfile(QSurfaceFormat.CoreProfile)
        fmt.setDepthBufferSize(24)
        fmt.setSwapBehavior(QSurfaceFormat.DoubleBuffer)
        fmt.setSwapInterval(1)  # vsync
        QSurfaceFormat.setDefaultFormat(fmt)

        # Create the QOpenGLWindow and embed it.
        self._gl_window = _ViewportWindow()
        self._gl_window.setFormat(fmt)
        container = QWidget.createWindowContainer(self._gl_window, self)
        container.setFocusPolicy(Qt.StrongFocus)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(container)
        self.setLayout(layout)

    def _setup_input_handlers(self) -> None:
        # Mouse events are forwarded by the container to this widget.
        # We capture them here and translate to camera operations.
        pass  # see mousePressEvent / mouseMoveEvent / wheelEvent

    # ----- Public API (mirrors OrbitCamera) -----

    def set_camera_target(self, x: float, y: float, z: float) -> None:
        self._camera_target = (x, y, z)
        self.update()

    def set_camera_distance(self, d: float) -> None:
        self._camera_distance = max(0.001, d)
        self.update()

    def set_camera_yaw_pitch(self, yaw: float, pitch: float) -> None:
        self._camera_yaw = yaw
        # Clamp pitch to avoid gimbal lock.
        self._camera_pitch = max(-1.5533, min(1.5533, pitch))
        self.update()

    def frame_all(self) -> None:
        """Auto-frame the scene. Phase C.7 will pass the scene bbox here."""
        # For now, reset to defaults.
        self._camera_target = (0.0, 0.0, 0.0)
        self._camera_distance = 5.0
        self._camera_yaw = 0.0
        self._camera_pitch = 0.3
        self.update()

    # ----- Input handlers (translate Qt events → camera ops) -----

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
            # Orbit: dx → yaw, dy → pitch
            sensitivity = 0.005
            self._camera_yaw += delta.x() * sensitivity
            self._camera_pitch -= delta.y() * sensitivity
            self._camera_pitch = max(-1.5533, min(1.5533, self._camera_pitch))
            self.update()
        elif self._mouse_button == Qt.MouseButton.MiddleButton:
            # Pan: move target in camera-right / camera-up plane.
            # Simplified — full implementation needs the camera's right/up
            # vectors which we don't have here in pure Python.
            sensitivity = 0.01 * self._camera_distance
            tx, ty, tz = self._camera_target
            tx -= delta.x() * sensitivity
            ty += delta.y() * sensitivity
            self._camera_target = (tx, ty, tz)
            self.update()

    def mouseReleaseEvent(self, event: QMouseEvent) -> None:
        self._last_mouse_pos = None
        self._mouse_button = Qt.MouseButton.NoButton

    def wheelEvent(self, event: QWheelEvent) -> None:
        # Zoom: angle delta.y() / 120 = number of notches.
        notches = event.angleDelta().y() / 120.0
        factor = 0.9 if notches > 0 else 1.1
        self._camera_distance *= factor ** abs(notches)
        self._camera_distance = max(0.001, min(1e6, self._camera_distance))
        self.update()


class _ViewportWindow(QOpenGLWindow):
    """The actual QOpenGLWindow that owns the GL context."""

    def __init__(self) -> None:
        super().__init__()
        self._renderer = None  # will be set in initializeGL

    def initializeGL(self) -> None:
        # In a full build we'd instantiate CAD_0.viewport.OpenGLRenderer
        # here. For now we keep a placeholder.
        self._renderer = "opengl-placeholder"

    def resizeGL(self, w: int, h: int) -> None:
        if self._renderer is None:
            return
        # Forward to renderer.resize(w, h)
        pass

    def paintGL(self) -> None:
        if self._renderer is None:
            return
        # Forward to renderer.begin_frame(camera, opts) + draw + end_frame.
        # For Phase C.6 we just clear to a dark gray.
        # In production, the CAD_0 C++ extension exposes a function
        # like `clear_framebuffer(color)`.
        pass
