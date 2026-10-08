"""CAD_0.ui.main_window — Application main window with working scene creation.

Now connects to the C++ Scene kernel and renders meshes in the viewport.
"""

from __future__ import annotations

from typing import Optional

from PySide6.QtCore import Qt
from PySide6.QtGui import QAction, QKeySequence
from PySide6.QtWidgets import (
    QApplication,
    QDockWidget,
    QFileDialog,
    QMainWindow,
    QMessageBox,
    QStatusBar,
    QToolBar,
    QWidget,
)

from .viewport_widget import ViewportWidget
from .property_panel import PropertyPanel

# Simple feature tree (no QAbstractItemModel for now — just QTreeWidget)
from PySide6.QtWidgets import QTreeWidget, QTreeWidgetItem, QAbstractItemView


class MainWindow(QMainWindow):
    """The CAD_0 application's main window."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("CAD_0 — Phase C+ Desktop")
        self.resize(1280, 800)

        # Try to import the C++ kernel
        try:
            import CAD_0
            self._cpp = CAD_0
            self._scene = CAD_0.scene.Scene()
            self._has_kernel = True
        except ImportError:
            self._cpp = None
            self._scene = None
            self._has_kernel = False

        self._setup_central_widget()
        self._setup_dock_widgets()
        self._setup_menus()
        self._setup_toolbar()
        self._setup_statusbar()
        self._connect_signals()

        if not self._has_kernel:
            QMessageBox.warning(self, "Kernel not found",
                "Could not import the CAD_0 C++ kernel.\n"
                "Run with PYTHONPATH pointing to the build/python directory.")

    # ----- Setup -----

    def _setup_central_widget(self) -> None:
        self._viewport = ViewportWidget(self)
        self.setCentralWidget(self._viewport)
        if self._scene is not None:
            self._viewport.set_scene(self._scene)

    def _setup_dock_widgets(self) -> None:
        # Simple QTreeWidget for the feature tree
        self._feature_tree = QTreeWidget(self)
        self._feature_tree.setHeaderLabels(["Name", "Type"])
        self._feature_tree.setSelectionBehavior(QAbstractItemView.SelectRows)
        self._feature_tree.setSelectionMode(QAbstractItemView.SingleSelection)

        self._tree_dock = QDockWidget("Feature Tree", self)
        self._tree_dock.setWidget(self._feature_tree)
        self._tree_dock.setMinimumWidth(250)
        self.addDockWidget(Qt.LeftDockWidgetArea, self._tree_dock)

        self._property_panel = PropertyPanel(self)
        self._props_dock = QDockWidget("Properties", self)
        self._props_dock.setWidget(self._property_panel)
        self.addDockWidget(Qt.LeftDockWidgetArea, self._props_dock)
        self.splitDockWidget(self._tree_dock, self._props_dock, Qt.Vertical)

    def _setup_menus(self) -> None:
        # ----- File menu -----
        file_menu = self.menuBar().addMenu("&File")

        new_action = QAction("&New Scene", self)
        new_action.setShortcut(QKeySequence.New)
        new_action.triggered.connect(self.on_new)
        file_menu.addAction(new_action)

        file_menu.addSeparator()

        export_stl = QAction("Export &STL...", self)
        export_stl.triggered.connect(lambda: self._on_export("stl"))
        file_menu.addAction(export_stl)

        export_obj = QAction("Export &OBJ...", self)
        export_obj.triggered.connect(lambda: self._on_export("obj"))
        file_menu.addAction(export_obj)

        export_3mf = QAction("Export &3MF...", self)
        export_3mf.triggered.connect(lambda: self._on_export("3mf"))
        file_menu.addAction(export_3mf)

        file_menu.addSeparator()

        quit_action = QAction("&Quit", self)
        quit_action.setShortcut(QKeySequence.Quit)
        quit_action.triggered.connect(self.close)
        file_menu.addAction(quit_action)

        # ----- Create menu -----
        create_menu = self.menuBar().addMenu("&Create")

        create_sphere = QAction("Create &Sphere", self)
        create_sphere.triggered.connect(self.on_create_sphere)
        create_menu.addAction(create_sphere)

        create_box = QAction("Create &Box", self)
        create_box.triggered.connect(self.on_create_box)
        create_menu.addAction(create_box)

        create_cylinder = QAction("Create &Cylinder", self)
        create_cylinder.triggered.connect(self.on_create_cylinder)
        create_menu.addAction(create_cylinder)

        create_torus = QAction("Create &Torus", self)
        create_torus.triggered.connect(self.on_create_torus)
        create_menu.addAction(create_torus)

        # ----- View menu -----
        view_menu = self.menuBar().addMenu("&View")

        self._toggle_grid = QAction("Show &Grid", self, checkable=True)
        self._toggle_grid.setChecked(True)
        self._toggle_grid.triggered.connect(self.on_toggle_grid)
        view_menu.addAction(self._toggle_grid)

        self._toggle_axes = QAction("Show &Axes", self, checkable=True)
        self._toggle_axes.setChecked(True)
        self._toggle_axes.triggered.connect(self.on_toggle_axes)
        view_menu.addAction(self._toggle_axes)

        frame_all = QAction("&Frame All", self)
        frame_all.setShortcut(QKeySequence("F"))
        frame_all.triggered.connect(self.on_frame_all)
        view_menu.addAction(frame_all)

        # ----- Help menu -----
        help_menu = self.menuBar().addMenu("&Help")
        about = QAction("&About CAD_0", self)
        about.triggered.connect(self.on_about)
        help_menu.addAction(about)

    def _setup_toolbar(self) -> None:
        toolbar = QToolBar("Main", self)
        toolbar.setMovable(False)
        toolbar.addAction(self._toggle_grid)
        toolbar.addAction(self._toggle_axes)
        self.addToolBar(toolbar)

    def _setup_statusbar(self) -> None:
        self._status = QStatusBar(self)
        self.setStatusBar(self._status)
        self._status.showMessage("Ready — try Create → Sphere")

    def _connect_signals(self) -> None:
        self._feature_tree.itemSelectionChanged.connect(self._on_tree_selection_changed)

    # ----- Slots -----

    def on_new(self) -> None:
        if self._scene is not None:
            self._scene.clear()
        self._feature_tree.clear()
        self._refresh_viewport()
        self._status.showMessage("New scene", 3000)

    def _add_shape_to_scene(self, name: str, body) -> None:
        """Add an SDF body to the scene and update the UI."""
        if not self._has_kernel:
            QMessageBox.warning(self, "No kernel", "C++ kernel not available.")
            return
        node_id = self._scene.add_node(name, body)
        # Add to tree widget
        item = QTreeWidgetItem([name, "SDF"])
        item.setData(0, Qt.UserRole, node_id)
        self._feature_tree.addTopLevelItem(item)
        self._refresh_viewport()
        self._status.showMessage(f"Created {name} (id={node_id})", 3000)

    def _refresh_viewport(self) -> None:
        """Re-tessellate and redraw."""
        self._viewport._refresh_meshes()

    def on_create_sphere(self) -> None:
        if not self._has_kernel:
            return
        body = self._cpp.sdf.sphere(radius=1.0)
        self._add_shape_to_scene("Sphere", body)

    def on_create_box(self) -> None:
        if not self._has_kernel:
            return
        body = self._cpp.sdf.box(self._cpp.math.Vec3f(0.5, 0.5, 0.5))
        self._add_shape_to_scene("Box", body)

    def on_create_cylinder(self) -> None:
        if not self._has_kernel:
            return
        body = self._cpp.sdf.cylinder(radius=0.5, height=2.0)
        self._add_shape_to_scene("Cylinder", body)

    def on_create_torus(self) -> None:
        if not self._has_kernel:
            return
        body = self._cpp.sdf.torus(R=1.0, r=0.3)
        self._add_shape_to_scene("Torus", body)

    def _on_export(self, format_name: str) -> None:
        extensions = {
            "stl": "STL Mesh (*.stl)",
            "obj": "Wavefront OBJ (*.obj)",
            "3mf": "3D Manufacturing Format (*.3mf)",
        }
        path, _ = QFileDialog.getSaveFileName(
            self, f"Export {format_name.upper()}", "",
            f"{extensions[format_name]};;All Files (*)"
        )
        if not path or not self._has_kernel:
            return
        # Export first visible mesh
        for node in self._scene.nodes():
            if node.visible and node.cached_mesh.vertex_count() > 0:
                self._cpp.io.export(path, node.cached_mesh)
                self._status.showMessage(f"Exported: {path}", 3000)
                return
        self._status.showMessage("No mesh to export", 3000)

    def on_toggle_grid(self, checked: bool) -> None:
        self._viewport.set_show_grid(checked)

    def on_toggle_axes(self, checked: bool) -> None:
        self._viewport.set_show_axes(checked)

    def on_frame_all(self) -> None:
        self._viewport.frame_all()
        self._status.showMessage("Framed all", 1500)

    def on_about(self) -> None:
        QMessageBox.about(
            self,
            "About CAD_0",
            "<h3>CAD_0</h3>"
            "<p>Dual-representation CAD kernel — native SDF + B-Rep.</p>"
            "<p>Phase C+ — working viewport with real mesh rendering.</p>"
            "<p>License: Apache-2.0</p>"
        )

    def _on_tree_selection_changed(self) -> None:
        items = self._feature_tree.selectedItems()
        if not items:
            return
        item = items[0]
        node_id = item.data(0, Qt.UserRole)
        if node_id is None:
            return
        node = self._scene.get_node(node_id)
        if node is None:
            return
        self._property_panel.set_parameters(node.name, {
            "radius": 1.0 if "Sphere" in node.name else 0.0,
        })


def main() -> None:
    """Application entry point."""
    import sys
    app = QApplication(sys.argv)
    app.setApplicationName("CAD_0")
    app.setOrganizationName("CAD_0")

    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
